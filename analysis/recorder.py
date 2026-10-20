"""
Records a run for the web replay viewer (web/index.html).

The file holds everything the browser needs to redraw the run without
PyBullet: robot visual geometry, staircase geometry, a pose for every robot
link at REC_HZ, telemetry (state, sensor estimates, actuator positions), LiDAR
beams while scanning, when each dirt cell was cleaned, and the final report.

    python main.py --headless --randomize --seed 3 --record web/data/random.json
"""
import json
import math

import numpy as np
import pybullet as p

import config as C

REC_HZ = 15
GEOM = {p.GEOM_BOX: "box", p.GEOM_CYLINDER: "cyl", p.GEOM_SPHERE: "sph"}


def _r(v, n=4):
    return [round(float(x), n) for x in v]


class Recorder:
    def __init__(self, robot, stairs, dirt, fsm, per, scenario):
        self.robot, self.stairs, self.dirt, self.fsm, self.per = robot, stairs, dirt, fsm, per
        self.scenario = scenario
        self.frames = []
        self.lidar = {}          # frame index -> {"o": origin, "h": hits}
        self.every = max(1, C.PHYSICS_HZ // REC_HZ)
        self._k = 0
        self._last_scan_id = None

        # which links carry visuals, and their geometry in the link frame
        self.links = []
        self.shapes = []
        for v in p.getVisualShapeData(robot.id):
            link, gtype, dims, pos, orn, rgba = v[1], v[2], v[3], v[5], v[6], v[7]
            if gtype not in GEOM:
                continue
            if link not in self.links:
                self.links.append(link)
            self.shapes.append(dict(l=self.links.index(link), g=GEOM[gtype], d=_r(dims), p=_r(pos),
                                    q=_r(orn), c=_r(rgba[:3], 3)))

        self.clean_frame = {i: np.full(t["clean"].shape, -1, int) for i, t in dirt.treads.items()}

    def _link_pose(self, link):
        if link == -1:
            pos, orn = p.getBasePositionAndOrientation(self.robot.id)
        else:
            st = p.getLinkState(self.robot.id, link, computeForwardKinematics=True)
            pos, orn = st[4], st[5]
        return _r(pos) + _r(orn)

    def step(self, t):
        self._k += 1
        if self._k % self.every:
            return
        fi = len(self.frames)
        fsm, rob = self.fsm, self.robot
        est = fsm.est
        rr, pp, _ = rob.true_rpy()
        self.frames.append(dict(
            t=round(t, 3), s=fsm.state, tr=fsm.tread,
            P=[self._link_pose(l) for l in self.links],
            pi=round(math.degrees(-pp), 2), ro=round(math.degrees(rr), 2),
            fl=round(rob.q("L_front_lift") * 1000, 1), lg=round(rob.q("L_leg") * 1000, 1),
            sp=round(rob.span() * 1000, 1), tm=round(fsm.trim * 1000, 2),
            h=round(est.height * 1000, 1) if est is not None and est.height is not None else None,
            d=round(est.depth * 1000, 1) if est is not None and est.depth is not None else None,
            g=fsm.gait.name if fsm.gait is not None else None,
            cv=round(self.dirt.coverage(), 4),
        ))
        for i, tr in self.dirt.treads.items():
            new = tr["clean"] & (self.clean_frame[i] < 0)
            self.clean_frame[i][new] = fi
        lid = self.per.lidar
        if fsm.state == "SCAN" and lid.last_world_hits is not None and id(lid.last_world_hits) != self._last_scan_id:
            self._last_scan_id = id(lid.last_world_hits)
            hits = lid.last_world_hits[::2]
            self.lidar[fi] = dict(o=_r(lid.last_origin, 3),
                                  h=[_r(h, 3) if not np.any(np.isnan(h)) else None for h in hits])

    def save(self, path, summary):
        st = self.stairs
        dirt = []
        for i, tr in self.dirt.treads.items():
            dirt.append(dict(tread=i, z=round(tr["z"], 4), xs=_r(tr["xs"]), ys=_r(tr["ys"]),
                             f=self.clean_frame[i].ravel().tolist()))
        data = dict(
            scenario=self.scenario, hz=REC_HZ, link_count=len(self.links), shapes=self.shapes,
            stairs=dict(risers=_r(st.risers_x), heights=_r(st.heights), depths=_r(st.depths),
                        tread_z=_r(st.tread_z), width=st.width, wall_h=0.35),
            frames=self.frames, lidar=self.lidar, dirt=dirt,
            summary={k: v for k, v in summary.items() if k != "events"},
            events=[dict(t=e[0], s=e[1], n=e[2]) for e in self.fsm.events if e[0] is not None],
        )
        with open(path, "w") as f:
            json.dump(data, f, separators=(",", ":"), default=lambda v: None if v is None else float(v))
        return path
