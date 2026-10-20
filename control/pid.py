"""PID controller and a joint servo built on it (PID on position -> velocity command)."""
import numpy as np


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, out_limit=np.inf, i_limit=np.inf):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit, self.i_limit = out_limit, i_limit
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev = None

    def step(self, err, dt):
        self.i = float(np.clip(self.i + err * dt, -self.i_limit, self.i_limit))
        d = 0.0 if self.prev is None else (err - self.prev) / dt
        self.prev = err
        return float(np.clip(self.kp * err + self.ki * self.i + self.kd * d, -self.out_limit, self.out_limit))


class JointServo:
    """Drives one actuator joint to a target with a PID loop that outputs a
    velocity command (like a motor driver taking a speed setpoint)."""

    def __init__(self, robot, joint, kp=8.0, ki=2.0, kd=0.0, max_vel=0.06, force=200.0):
        self.robot, self.joint = robot, joint
        self.pid = PID(kp, ki, kd, out_limit=max_vel, i_limit=0.01)
        self.force = force
        self.target = robot.q(joint)

    def set(self, target, max_vel=None, force=None):
        if max_vel is not None:
            self.pid.out_limit = max_vel
        if force is not None:
            self.force = force
        self.target = target

    def update(self, dt):
        err = self.target - self.robot.q(self.joint)
        self.robot.set_velocity(self.joint, self.pid.step(err, dt), self.force)
        return err

    def error(self):
        return self.target - self.robot.q(self.joint)

    def at_target(self, tol=0.002):
        return abs(self.error()) < tol
