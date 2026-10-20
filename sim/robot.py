"""Thin wrapper around the PyBullet multibody: joint access, sensors' raw data, actuation."""
import numpy as np
import pybullet as p

import config as C
from sim.robot_urdf import write_urdf

SIDES = ("L", "R")


class Robot:
    def __init__(self, start_x, span, rng=None):
        self.rng = rng or np.random.default_rng(0)
        urdf = write_urdf()
        self.id = p.loadURDF(urdf, [start_x, 0, C.BASE_Z + 0.003], useFixedBase=False)
        self.joints, self.links = {}, {}
        for j in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, j)
            self.joints[info[1].decode()] = j
            self.links[info[12].decode()] = j
        self.links["frame"] = -1
        self.wheels = [j for n, j in self.joints.items() if "wheel" in n]
        self.prismatic = [j for n, j in self.joints.items() if p.getJointInfo(self.id, j)[2] == p.JOINT_PRISMATIC]

        # Disable default velocity motors so we fully control every joint
        for j in self.joints.values():
            p.setJointMotorControl2(self.id, j, p.VELOCITY_CONTROL, force=0)
        for s in SIDES:
            p.resetJointState(self.id, self.joints[f"{s}_span"], span / 2)

        for w in self.wheels:
            # Omni-wheel approximation: full grip in the rolling direction,
            # reduced grip along the axle (passive rollers) so the unit can slide sideways.
            p.changeDynamics(self.id, w, lateralFriction=C.FRICTION_WHEEL,
                             anisotropicFriction=[1, 1, 0.3], rollingFriction=0.0005)
        for s in SIDES:
            p.changeDynamics(self.id, self.links[f"{s}_leg"], lateralFriction=C.FRICTION_PAD)
            for part in ("back", "front"):
                p.changeDynamics(self.id, self.links[f"{s}_{part}"], lateralFriction=0.3)
        self._odom_bias = 0.0
        self._last_true_x = self.true_x()

    # ------------------------------------------------------------------ joints
    def q(self, name):
        return p.getJointState(self.id, self.joints[name])[0]

    def qd(self, name):
        return p.getJointState(self.id, self.joints[name])[1]

    def set_velocity(self, name, v, force):
        p.setJointMotorControl2(self.id, self.joints[name], p.VELOCITY_CONTROL, targetVelocity=v, force=force)

    def set_position(self, name, q, max_vel=0.3, force=100.0):
        p.setJointMotorControl2(self.id, self.joints[name], p.POSITION_CONTROL, targetPosition=q,
                                maxVelocity=max_vel, force=force, positionGain=0.3)

    def drive(self, v, yaw_rate=0.0):
        """Forward speed (m/s) plus a turn rate (rad/s) applied as differential
        wheel speeds between the left and right drive units."""
        half_track = self.span() / 2
        for name, j in self.joints.items():
            if "wheel" not in name:
                continue
            side_v = v - yaw_rate * half_track if name.startswith("L") else v + yaw_rate * half_track
            p.setJointMotorControl2(self.id, j, p.VELOCITY_CONTROL, targetVelocity=side_v / C.WHEEL_RADIUS,
                                    force=C.WHEEL_TORQUE)

    # ------------------------------------------------------------------ poses
    def link_pose(self, name):
        if name == "frame":
            return p.getBasePositionAndOrientation(self.id)
        st = p.getLinkState(self.id, self.links[name], computeForwardKinematics=True)
        return st[4], st[5]

    def true_x(self):
        return p.getBasePositionAndOrientation(self.id)[0][0]

    def true_rpy(self):
        return p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id)[1])

    # ------------------------------------------------------------------ proprioception
    def imu(self):
        """Noisy (roll, pitch_up) in radians. pitch_up > 0 means nose up."""
        r, pch, _ = self.true_rpy()
        n = self.rng.normal(0, C.IMU_NOISE, 2)
        return r + n[0], -pch + n[1]

    def imu_yaw(self):
        """Noisy heading (rad, CCW positive) from the IMU's gyro/magnetometer."""
        return self.true_rpy()[2] + self.rng.normal(0, C.IMU_NOISE)

    def odometry_x(self):
        """Wheel odometry along the stairs, with a slow random-walk error."""
        x = self.true_x()
        dx = x - self._last_true_x
        self._last_true_x = x
        if abs(dx) > 1e-5:
            self._odom_bias += self.rng.normal(0, C.ODOM_NOISE) * min(1.0, abs(dx) / 0.001)
        return x + self._odom_bias

    def span(self):
        return self.q("L_span") + self.q("R_span")
