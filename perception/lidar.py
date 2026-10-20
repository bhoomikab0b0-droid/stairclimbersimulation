"""
Simulated range sensors.

* Front LiDAR: a vertical fan of rays (x-z plane) from the front unit, like a
  2D LiDAR / depth-camera column looking at the stairs ahead.
* Side rangefinders: ultrasonic/IR sensors on each back unit facing the walls.

Ranges get Gaussian noise; points are then levelled with the (noisy) IMU
attitude, exactly as an onboard computer would have to do.
"""
import numpy as np
import pybullet as p

import config as C


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class FrontLidar:
    def __init__(self, robot, link="L_lidar", rng=None):
        self.robot = robot
        self.link = link
        self.rng = rng or np.random.default_rng(1)
        self.angles = np.linspace(C.LIDAR_FOV_DOWN, C.LIDAR_FOV_UP, C.LIDAR_RAYS)
        # unit ray directions in the sensor frame (x forward, z up)
        self.dirs = np.stack([np.cos(self.angles), np.zeros_like(self.angles), np.sin(self.angles)], 1)
        self.last_world_hits = None
        self._debug_ids = []

    def scan(self, imu_roll, imu_pitch_up):
        """Returns Nx2 array of (x forward, z up) points in a gravity-levelled
        frame centred on the sensor, ordered by beam angle (NaN for no return)."""
        pos, orn = self.robot.link_pose(self.link)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        origin = np.array(pos)
        world_dirs = self.dirs @ R.T
        ends = origin + world_dirs * C.LIDAR_RANGE
        res = p.rayTestBatch([origin.tolist()] * len(ends), ends.tolist())
        frac = np.array([r[2] if r[0] not in (-1, self.robot.id) else np.nan for r in res])
        rng_m = frac * C.LIDAR_RANGE
        rng_m = rng_m + self.rng.normal(0, C.LIDAR_NOISE, len(rng_m))
        self.last_world_hits = origin + world_dirs * rng_m[:, None]
        self.last_origin = origin
        pts = self.dirs * rng_m[:, None]                       # sensor frame
        # level using IMU: undo body pitch (nose-up positive) and roll
        level = pts @ (_rot_y(-imu_pitch_up) @ _rot_x(imu_roll)).T
        return level[:, [0, 2]]

    def draw(self, every=4):
        """Debug-draw a subset of beams in the GUI."""
        if self.last_world_hits is None:
            return
        hits = self.last_world_hits[::every]
        while len(self._debug_ids) < len(hits):
            self._debug_ids.append(p.addUserDebugLine([0, 0, 0], [0, 0, 0.001], [1, 0, 0]))
        for k, h in enumerate(hits):
            ok = not np.any(np.isnan(h))
            end = h if ok else self.last_origin
            p.addUserDebugLine(self.last_origin.tolist(), end.tolist(), [1, 0.15, 0.1] if ok else [0, 0, 0],
                               lineWidth=1, replaceItemUniqueId=self._debug_ids[k])


class SideRangefinders:
    """One outward-facing rangefinder on each back unit -> stair width."""

    def __init__(self, robot, rng=None, max_range=1.0):
        self.robot = robot
        self.rng = rng or np.random.default_rng(2)
        self.max_range = max_range

    def measure(self):
        out = {}
        for side, sgn in (("L", 1), ("R", -1)):
            pos, orn = self.robot.link_pose(f"{side}_back")
            R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            y_dir = R[:, 1] * sgn
            start = np.array(pos) + y_dir * (C.UNIT_WIDTH / 2 + 0.001)
            end = start + y_dir * self.max_range
            hit = p.rayTest(start.tolist(), end.tolist())[0]
            if hit[0] in (-1, self.robot.id):
                out[side] = np.nan
            else:
                out[side] = hit[2] * self.max_range + self.rng.normal(0, C.SIDE_RANGE_NOISE)
        return out
