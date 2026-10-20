"""
Perception module: turns raw LiDAR / rangefinder / IMU / odometry data into a
fused estimate of the next step (height, depth, riser position) and the stair
width, using Kalman filters.
"""
from dataclasses import dataclass

import numpy as np

import config as C
from perception.kalman import Kalman1D
from perception.lidar import FrontLidar, SideRangefinders
from perception.step_detector import detect_step

# Positions along the robot's x axis, measured from the frame (odometry) origin
FRONT_FACE_X = C.BACK_LEN / 2 + C.FRONT_LEN + 0.001
LIDAR_X = FRONT_FACE_X + 0.01
LIDAR_HEIGHT = C.BASE_Z + 0.05     # sensor height above the tread when level on wheels


@dataclass
class StepEstimate:
    step_ahead: bool
    height: float = None
    depth: float = None
    riser_x: float = None        # riser position in the odometry frame
    width: float = None
    is_landing: bool = False
    blocked: bool = False
    h_std: float = None
    d_std: float = None
    n: int = 0


class Perception:
    def __init__(self, robot, rng):
        self.robot = robot
        self.lidar = FrontLidar(robot, rng=rng)
        self.side = SideRangefinders(robot, rng=rng)
        r = C.LIDAR_NOISE ** 2 * 2
        self.kf_h = Kalman1D(q=1e-8, r=r)
        self.kf_d = Kalman1D(q=1e-8, r=r * 4)
        self.kf_riser = Kalman1D(q=1e-8, r=r)
        self.kf_w = Kalman1D(q=1e-8, r=C.SIDE_RANGE_NOISE ** 2 * 2)
        self.votes_step = 0
        self.votes_none = 0
        self.landing_votes = 0
        self.blocked_votes = 0
        self.last_obs = None
        self.last_pts = None
        self.history = []            # (raw_h, kf_h, raw_d, kf_d) per scan, for plots

    def reset(self):
        for kf in (self.kf_h, self.kf_d, self.kf_riser):
            kf.reset()
        self.votes_step = self.votes_none = self.landing_votes = self.blocked_votes = 0
        self.history = []

    def observe(self, odom_x):
        roll, pitch_up = self.robot.imu()
        pts = self.lidar.scan(roll, pitch_up)
        obs = detect_step(pts, ground_z_prior=-LIDAR_HEIGHT)
        self.last_obs, self.last_pts = obs, pts
        if obs.step_ahead:
            self.votes_step += 1
            self.blocked_votes += obs.blocked
            self.kf_h.update(obs.height)
            self.kf_riser.update(odom_x + LIDAR_X + obs.riser_dist)
            if obs.depth is not None:
                self.kf_d.update(obs.depth)
            if obs.is_landing:
                self.landing_votes += 1
        else:
            self.votes_none += 1
        self.history.append((obs.height, self.kf_h.x, obs.depth, self.kf_d.x))
        return obs

    def measure_width(self):
        d = self.side.measure()
        if np.isnan(d["L"]) or np.isnan(d["R"]):
            return self.kf_w.x
        return self.kf_w.update(self.robot.span() + C.UNIT_WIDTH + d["L"] + d["R"])

    def estimate(self) -> StepEstimate:
        if self.votes_step <= self.votes_none:
            return StepEstimate(step_ahead=False, width=self.kf_w.x, n=self.votes_none)
        return StepEstimate(True, self.kf_h.x, self.kf_d.x, self.kf_riser.x, self.kf_w.x,
                            self.landing_votes > self.votes_step / 2,
                            self.blocked_votes > self.votes_step / 2,
                            self.kf_h.std, self.kf_d.std if self.kf_d.x is not None else None, self.votes_step)
