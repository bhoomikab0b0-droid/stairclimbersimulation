"""Procedural staircase: floor, solid step blocks, top landing and side walls."""
from dataclasses import dataclass, field

import numpy as np
import pybullet as p

import config as C


@dataclass
class Staircase:
    risers_x: list = field(default_factory=list)   # x of each riser face
    heights: list = field(default_factory=list)    # riser height of each step
    depths: list = field(default_factory=list)     # tread depth of each step (last = landing)
    tread_z: list = field(default_factory=list)    # top surface z of each tread
    width: float = 0.9
    body_ids: list = field(default_factory=list)

    @property
    def n_steps(self):
        return len(self.heights)

    def tread_bounds(self, i):
        """(x0, x1, z) of tread i (1-based; 0 = the floor before the stairs)."""
        if i == 0:
            return (self.risers_x[0] - 1.0, self.risers_x[0], 0.0)
        return (self.risers_x[i - 1], self.risers_x[i - 1] + self.depths[i - 1], self.tread_z[i - 1])

    def tread_index_at(self, x):
        """Which tread the world x coordinate lies over."""
        idx = 0
        for i, rx in enumerate(self.risers_x):
            if x >= rx:
                idx = i + 1
        return idx


def _box(half, pos, rgba, friction=C.FRICTION_STAIR):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    bid = p.createMultiBody(0, col, vis, basePosition=pos)
    p.changeDynamics(bid, -1, lateralFriction=friction)
    return bid


def build_staircase(spec: C.StairSpec) -> Staircase:
    rng = np.random.default_rng(spec.seed)
    st = Staircase(width=spec.width)

    # Floor
    plane = p.createCollisionShape(p.GEOM_PLANE)
    floor = p.createMultiBody(0, plane)
    p.changeDynamics(floor, -1, lateralFriction=C.FRICTION_STAIR)
    p.changeVisualShape(floor, -1, textureUniqueId=-1, rgbaColor=[0.80, 0.80, 0.78, 1])
    st.body_ids.append(floor)

    x, z = 0.0, 0.0
    for i in range(spec.n_steps):
        h, d = spec.height, spec.depth
        if spec.heights:
            h = spec.heights[min(i, len(spec.heights) - 1)]
        if spec.randomize:
            h += rng.uniform(-spec.height_jitter, spec.height_jitter)
            d += rng.uniform(-spec.depth_jitter, spec.depth_jitter)
        last = i == spec.n_steps - 1
        if last:
            d = spec.landing
        z += h
        st.risers_x.append(x)
        st.heights.append(h)
        st.depths.append(d)
        st.tread_z.append(z)
        shade = 0.55 + 0.08 * (i % 2)
        st.body_ids.append(_box([d / 2, spec.width / 2, z / 2], [x + d / 2, 0, z / 2],
                                [shade, shade * 0.95, shade * 0.85, 1]))
        x += d

    # Stepped side walls (stair stringers / skirting) that follow the profile,
    # 0.35 m above each tread: high enough for the side rangefinders, low
    # enough that a camera can look over them.
    wall_h = 0.35
    segments = [(-spec.approach - 1.0, 0.0, 0.0)] + [
        (st.risers_x[i], st.risers_x[i] + st.depths[i], st.tread_z[i]) for i in range(spec.n_steps)]
    for x0, x1, z in segments:
        for side in (+1, -1):
            top = z + wall_h
            st.body_ids.append(_box([(x1 - x0) / 2, 0.02, top / 2],
                                    [(x0 + x1) / 2, side * (spec.width / 2 + 0.02), top / 2],
                                    [0.93, 0.92, 0.90, 1]))
    return st
