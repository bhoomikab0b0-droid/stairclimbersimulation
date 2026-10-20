"""
Step geometry extraction from a single levelled LiDAR scan.

Pipeline
1. Local surface orientation: PCA over a sliding window of consecutive beams
   labels each point Horizontal (tread), Vertical (riser) or neither.
2. Consecutive points with the same label are grouped into line segments.
3. The nearest horizontal segment is the tread we stand on (z0); the next
   horizontal level above it is the next tread (z1) -> riser height h = z1 - z0.
4. The vertical segment between them gives the riser distance; the next
   riser/level above gives the tread depth d.
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import config as C

MIN_TREAD_SEEN = 0.04       # a horizontal level must span at least this (m)
SENSOR_LEVEL_BAND = 0.008   # faces whose top reaches sensor height might be taller
H_TOL = np.radians(20)
V_TOL = np.radians(70)


@dataclass
class Segment:
    kind: str              # 'H' or 'V'
    pts: np.ndarray

    @property
    def z(self):
        return float(np.median(self.pts[:, 1]))

    @property
    def x(self):
        return float(np.median(self.pts[:, 0]))

    @property
    def xmin(self):
        return float(self.pts[:, 0].min())

    @property
    def xmax(self):
        return float(self.pts[:, 0].max())

    @property
    def zmin(self):
        return float(self.pts[:, 1].min())

    @property
    def zmax(self):
        return float(self.pts[:, 1].max())


@dataclass
class StepObservation:
    step_ahead: bool                 # is there a riser in front of us?
    ground_z: Optional[float] = None # current tread height relative to sensor
    height: Optional[float] = None   # riser height h
    riser_dist: Optional[float] = None  # horizontal distance sensor -> riser
    depth: Optional[float] = None    # depth of the next tread (None = not seen)
    is_landing: bool = False         # next tread is a long landing
    blocked: bool = False            # a face rises above the sensor; tread not visible
    segments: list = field(default_factory=list)


def label_points(pts, min_extent=0.015, max_each_side=20, max_gap=0.05):
    """Label each point H/V from the principal direction of its neighbourhood.
    The neighbourhood grows until it spans at least `min_extent`: at close range
    beams land < 1 mm apart, far below the range noise, and a fixed small window
    would then measure noise instead of the surface."""
    n = len(pts)
    labels = np.array(["-"] * n)
    for i in range(n):
        if np.isnan(pts[i, 0]):
            continue
        idx = [i]
        ends = {-1: i, 1: i}
        for _ in range(max_each_side):
            grown = False
            for step in (-1, 1):
                k2 = ends[step] + step
                if 0 <= k2 < n and not np.isnan(pts[k2, 0]) and np.linalg.norm(pts[k2] - pts[ends[step]]) <= max_gap:
                    idx.append(k2)
                    ends[step] = k2
                    grown = True
            if np.linalg.norm(pts[ends[1]] - pts[ends[-1]]) >= 2 * min_extent or not grown:
                break
        if len(idx) < 3 or np.linalg.norm(pts[ends[1]] - pts[ends[-1]]) < min_extent:
            continue
        w = pts[idx] - pts[idx].mean(0)
        _, _, vt = np.linalg.svd(w, full_matrices=False)
        ang = abs(np.arctan2(vt[0, 1], vt[0, 0]))
        ang = min(ang, np.pi - ang)
        if ang < H_TOL:
            labels[i] = "H"
        elif ang > V_TOL:
            labels[i] = "V"
    return labels


def segment(pts, labels, max_gap=0.06, min_pts=3):
    segs, cur = [], []
    prev = None
    for i in range(len(pts)):
        lab = labels[i]
        if lab not in ("H", "V"):
            if cur:
                segs.append(Segment(prev, np.array(cur)))
            cur, prev = [], None
            continue
        if cur and (lab != prev or np.linalg.norm(pts[i] - cur[-1]) > max_gap):
            segs.append(Segment(prev, np.array(cur)))
            cur = []
        cur.append(pts[i])
        prev = lab
    if cur:
        segs.append(Segment(prev, np.array(cur)))
    return [s for s in segs if len(s.pts) >= min_pts]


def _merge_levels(hsegs, tol=0.015):
    """Group horizontal segments that lie on the same plane height (treads)."""
    levels = []
    for s in sorted(hsegs, key=lambda s: s.z):
        if levels and abs(s.z - levels[-1][0]) < tol:
            group = levels[-1][1] + [s]
            pts = np.vstack([g.pts for g in group])
            levels[-1] = (float(np.median(pts[:, 1])), group)
        else:
            levels.append((s.z, [s]))
    return [(z, Segment("H", np.vstack([g.pts for g in grp]))) for z, grp in levels]


def _merge_faces(vsegs, tol=0.02):
    """Group vertical segments at the same distance into whole faces (risers);
    sensor noise often breaks one riser into several short pieces."""
    faces = []
    for s in sorted(vsegs, key=lambda s: s.x):
        if faces and abs(s.x - faces[-1].x) < tol:
            faces[-1] = Segment("V", np.vstack([faces[-1].pts, s.pts]))
        else:
            faces.append(Segment("V", s.pts))
    return faces


def detect_step(pts, ground_z_prior=None) -> StepObservation:
    """ground_z_prior: tread height relative to the sensor when the robot is
    standing level on it (known from the sensor's mounting height). Used when
    the tread under the robot is not visible to the LiDAR."""
    valid = ~np.isnan(pts[:, 0])
    if valid.sum() < 10:
        return StepObservation(step_ahead=False)
    labels = label_points(pts)
    segs = segment(pts, labels)
    hsegs = [s for s in segs if s.kind == "H"]
    faces = _merge_faces([s for s in segs if s.kind == "V"])
    if not hsegs and ground_z_prior is None:
        return StepObservation(step_ahead=False, segments=segs)

    levels = _merge_levels(hsegs)
    if ground_z_prior is not None:
        # prefer a measured level that matches the prior
        match = [z for z, _ in levels if abs(z - ground_z_prior) < 0.02]
        z0 = match[0] if match else ground_z_prior
    else:
        # ground = level that contains the nearest horizontal return
        nearest = min(hsegs, key=lambda s: s.xmin)
        z0 = next(z for z, lv in levels if abs(z - nearest.z) < 0.015)

    # a tread must be seen over a real length; short blobs are noise on a riser face
    above = [(z, lv) for z, lv in levels if z > z0 + C.MIN_RISER and lv.xmax - lv.xmin >= MIN_TREAD_SEEN]
    if not above:
        # A face rising from our tread whose top we cannot see (riser taller
        # than the sensor, or a wall): report it as blocked so we never climb blind.
        walls = [f for f in faces if f.zmin < z0 + 0.06 and f.zmax > z0 + C.MIN_RISER]
        if walls:
            w = walls[0]
            if w.zmax > -SENSOR_LEVEL_BAND:
                return StepObservation(True, z0, w.zmax - z0, w.x, None, False, True, segs)
            # The face's top edge is visible below sensor level, so it is a step
            # whose tread is too close to sensor height for the beams to sample
            # it: the edge itself gives the riser height.
            nxt = [f for f in faces if f.x > w.x + 0.1 and f.zmax > w.zmax + 0.02]
            depth = nxt[0].x - w.x if nxt else None
            # Sparse tread recovery: the few grazing beams that land on the
            # tread are too far apart to be labelled, but their height is exact.
            x_end = nxt[0].x - 0.02 if nxt else w.x + 1.0
            p = pts[valid]
            on_tread = p[(p[:, 0] > w.x + 0.02) & (p[:, 0] < x_end) & (np.abs(p[:, 1] - w.zmax) < 0.015)]
            top = w.zmax
            if len(on_tread) >= 2 and np.std(on_tread[:, 1]) < 0.005:
                top = float(np.median(on_tread[:, 1]))
            return StepObservation(True, z0, top - z0, w.x, depth, False, False, segs)
        return StepObservation(step_ahead=False, ground_z=z0, segments=segs)
    z1, tread1 = above[0]
    h = z1 - z0

    # riser: the face spanning between the two levels, just in front of the tread
    risers = [f for f in faces if f.zmin < z1 - 0.01 and f.zmax > z0 + 0.01 and f.x < tread1.xmin + 0.03]
    riser_x = risers[-1].x if risers else tread1.xmin

    # depth: next riser face rising above tread1, else next level's leading edge
    depth = None
    next_faces = [f for f in faces if f.x > riser_x + 0.1 and f.zmax > z1 + 0.02]
    if next_faces:
        depth = next_faces[0].x - riser_x
    elif len(above) > 1 and above[1][1].xmin > riser_x + 0.1:
        depth = above[1][1].xmin - riser_x
    seen_extent = tread1.xmax - riser_x
    is_landing = depth is None and seen_extent > 0.6
    return StepObservation(True, z0, h, riser_x, depth, is_landing, False, segs)
