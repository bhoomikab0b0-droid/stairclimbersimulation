"""
Climbing + cleaning state machine (the robot's decision layer).

Per step:
  SCAN -> [ALIGN_WIDTH] -> APPROACH -> [CLEAN_TREAD] -> FRONT_LIFT -> ADVANCE_FRONT
       -> PLACE_FRONT -> REAR_LIFT -> ADVANCE_REAR -> RETRACT_REAR -> SCAN ...
At the top: SCAN finds no riser -> CLEAN_TREAD (landing) -> DONE.
Any safety violation -> SAFE_ABORT (stop, hold all actuators).

Front units (red) climb first while back units (blue) stabilise, then the back
units are lifted by the rear racks and driven up: the "coordinated climbing
gait" of method claim 9(d).
"""
import math

import numpy as np

import config as C
from control.gait_library import select_gait
from control.pid import PID, JointServo
from perception.perception import FRONT_FACE_X
from sim.robot_urdf import HEAD_BOTTOM_BELOW_BASE

APPROACH_GAP = 0.015       # front face -> riser gap before lifting
BACK_GAP = 0.008           # back unit front face -> riser after ADVANCE_FRONT
PAD_RISER_GAP = 0.004      # rear pad -> riser after ADVANCE_REAR
PAD_FRONT_X = -(C.BACK_LEN / 2 + C.PAD_BEHIND) + C.PAD_RADIUS
SCANS_PER_STEP = 12
SETTLE_TIME = 0.4
STATE_TIMEOUT = 30.0
HEAD_DOWN = C.BASE_Z - HEAD_BOTTOM_BELOW_BASE - 0.004
SIDES = ("L", "R")


class ClimbFSM:
    def __init__(self, robot, perception, dirt=None):
        self.robot, self.per, self.dirt = robot, perception, dirt
        self.servo = {}
        for s in SIDES:
            self.servo[f"{s}_front_lift"] = JointServo(robot, f"{s}_front_lift", force=C.RACK_FORCE)
            self.servo[f"{s}_leg"] = JointServo(robot, f"{s}_leg", force=C.RACK_FORCE)
            self.servo[f"{s}_span"] = JointServo(robot, f"{s}_span", kp=4.0, ki=0.5, max_vel=0.12, force=C.SPAN_FORCE)
        self.drive_pid = PID(2.5, 0.3, 0.0, out_limit=C.DRIVE_SPEED, i_limit=0.02)
        self.pitch_pid = PID(0.25, 0.8, 0.0, out_limit=0.03, i_limit=0.04)
        self.yaw_pid = PID(3.0, 0.5, 0.0, out_limit=0.5, i_limit=0.2)
        self.trim = 0.0
        self.state, self.t_enter, self.entered = "SCAN", 0.0, False
        self.tread = 0                # 0 = floor, i = i-th tread
        self.est = None
        self.gait = None
        self.h_contact = None
        self.riser_hist = []          # odometry x of every riser climbed
        self.side_samples = []
        self.events = []
        self.step_records = []
        self.final = False
        self.result = None
        self.pitch = self.roll = 0.0
        self._tick = 0
        self._stall_t = 0.0
        self._progress = []
        self.clean_wps, self.wp_i = [], 0

    # ------------------------------------------------------------------ helpers
    @property
    def finished(self):
        return self.state in ("DONE", "SAFE_ABORT")

    def goto(self, state, t, note=None):
        self.events.append((round(t, 2), state, note or ""))
        self.state, self.t_enter, self.entered = state, t, False
        self.drive_pid.reset()
        self.yaw_pid.reset()
        self._progress = []

    def abort(self, t, reason):
        self.result = f"ABORTED: {reason}"
        for sv in self.servo.values():
            sv.set(self.robot.q(sv.joint))
        self.goto("SAFE_ABORT", t, reason)

    def both(self, kind, target, **kw):
        for s in SIDES:
            self.servo[f"{s}_{kind}"].set(target, **kw)

    def both_at(self, kind, tol=0.002):
        return all(self.servo[f"{s}_{kind}"].at_target(tol) for s in SIDES)

    def drive_to(self, target, odom, dt, vmax, contact_window=0.03):
        """Distance PID along the stairs + heading-hold PID (keeps yaw = 0 so the
        side units stay parallel to the walls).

        Also returns True if, within `contact_window` of the target, the robot
        stops making progress: it has touched the riser (front touch sensors /
        motor stall), which absorbs odometry drift."""
        self.drive_pid.out_limit = vmax
        err = target - odom
        v = self.drive_pid.step(err, dt)
        yaw_rate = self.yaw_pid.step(-self.robot.imu_yaw(), dt)
        self.robot.drive(v, yaw_rate)
        if abs(err) < 0.002:
            return True
        self._progress.append(odom)
        if len(self._progress) > int(0.5 / dt):
            moved = abs(self._progress[-1] - self._progress.pop(0))
            if err > 0 and err < contact_window and moved < 0.001:
                self.events.append((None, self.state, f"contact detected {err * 1000:.0f} mm before odometry target"))
                return True
        return False

    def _coordinate_front(self, dt, retracting):
        """Keep the front unit resting on the upper tread while the rear racks
        move the back unit, plus an IMU pitch-levelling PID trim (CoM stability)."""
        # nose-down (pitch < 0) means the front rack is too extended -> lower it
        self.trim = self.pitch_pid.step(self.pitch, dt)
        for s in SIDES:
            back_lift = self.robot.q(f"{s}_leg") - C.PAD_GAP
            if retracting:
                tgt = min(0.0, self.h_contact - back_lift)
            else:
                tgt = self.h_contact - max(back_lift, 0.0)
            self.servo[f"{s}_front_lift"].set(tgt + self.trim)

    # ------------------------------------------------------------------ main tick
    def tick(self, t, dt):
        self._tick += 1
        odom = self.robot.odometry_x()
        self.roll, self.pitch = self.robot.imu()
        if not self.finished:
            if abs(self.pitch) > C.MAX_PITCH or abs(self.roll) > C.MAX_ROLL:
                self.abort(t, f"tilt limit exceeded (pitch {math.degrees(self.pitch):.1f} deg, "
                              f"roll {math.degrees(self.roll):.1f} deg)")
            elif t - self.t_enter > STATE_TIMEOUT:
                self.abort(t, f"timeout in {self.state}")
        first = not self.entered
        self.entered = True
        getattr(self, "_" + self.state.lower())(t, dt, odom, first)
        for sv in self.servo.values():
            sv.update(dt)
        if self.state != "CLEAN_TREAD":
            for j in ("clean_head", "clean_reach", "clean_rail"):
                self.robot.set_position(j, 0.0, max_vel=0.3, force=80)

    # ------------------------------------------------------------------ states
    def _scan(self, t, dt, odom, first):
        if first:
            self.robot.drive(0)
            self.per.reset()
            self.side_samples = []
            self._scans = 0
        self.robot.drive(0)
        if t - self.t_enter < SETTLE_TIME:
            return
        if self._tick % max(1, C.PHYSICS_HZ // C.LIDAR_HZ):
            return
        self.per.observe(odom)
        self.per.measure_width()
        self.side_samples.append(self.per.side.measure())
        self._scans += 1
        if self._scans < SCANS_PER_STEP:
            return
        est = self.est = self.per.estimate()
        rec = dict(tread=self.tread, step_ahead=est.step_ahead, h_est=est.height, d_est=est.depth,
                   riser_est=est.riser_x, w_est=est.width, h_std=est.h_std, blocked=est.blocked,
                   scans=list(self.per.history), scan_pts=self.per.last_pts,
                   t=round(t, 2))
        self.step_records.append(rec)
        if not est.step_ahead:
            if self.tread == 0:
                self.result = "no stairs detected"
                self.goto("DONE", t, self.result)
            else:
                self.final = True
                self.goto("CLEAN_TREAD", t, "top landing reached")
            return
        if est.blocked:
            self.abort(t, f"riser of at least {est.height * 1000:.0f} mm rises above the sensor: "
                          f"too high to climb")
            return
        self.gait, reason = select_gait(est.height, est.depth, est.width)
        rec["gait"] = self.gait.name if self.gait else None
        if self.gait is None:
            self.abort(t, reason)
            return
        # per-side frame targets so each unit keeps WALL_CLEARANCE to its wall
        dl = np.nanmean([s["L"] for s in self.side_samples])
        dr = np.nanmean([s["R"] for s in self.side_samples])
        lt = self.robot.q("L_span") + dl - C.WALL_CLEARANCE
        rt = self.robot.q("R_span") + dr - C.WALL_CLEARANCE
        lo, hi = C.SPAN_MIN / 2, C.SPAN_MAX / 2
        lt, rt = float(np.clip(lt, lo, hi)), float(np.clip(rt, lo, hi))
        self.servo["L_span"].set(lt)
        self.servo["R_span"].set(rt)
        note = (f"step {self.tread + 1}: h={est.height * 1000:.1f} mm, "
                f"d={'%.1f mm' % (est.depth * 1000) if est.depth else 'long/landing'}, "
                f"w={(est.width or 0) * 1000:.0f} mm -> gait {self.gait.name}")
        if abs(lt - self.robot.q("L_span")) > 0.005 or abs(rt - self.robot.q("R_span")) > 0.005:
            self.goto("ALIGN_WIDTH", t, note)
        else:
            self.goto("APPROACH", t, note)

    def _align_width(self, t, dt, odom, first):
        self.robot.drive(0)
        if t - self.t_enter > 0.3 and self.both_at("span", 0.003):
            self.goto("APPROACH", t, f"frame width x = {self.robot.span() * 1000:.0f} mm")

    def _approach(self, t, dt, odom, first):
        target = self.est.riser_x - APPROACH_GAP - FRONT_FACE_X
        if self.drive_to(target, odom, dt, self.gait.drive_speed):
            self.robot.drive(0)
            if self.tread >= 1:
                self.goto("CLEAN_TREAD", t)
            else:
                self.goto("FRONT_LIFT", t)

    def _front_lift(self, t, dt, odom, first):
        self.robot.drive(0)
        if first:
            self.both("front_lift", self.est.height + self.gait.lift_margin,
                      max_vel=self.gait.lift_speed, force=C.RACK_FORCE)
            self.both("leg", 0.0, max_vel=self.gait.lift_speed)
        elif self.both_at("front_lift"):
            self.goto("ADVANCE_FRONT", t)

    def _advance_front(self, t, dt, odom, first):
        target = self.est.riser_x - BACK_GAP - C.BACK_LEN / 2
        if self.drive_to(target, odom, dt, self.gait.drive_speed):
            self.robot.drive(0)
            self.goto("PLACE_FRONT", t)

    def _place_front(self, t, dt, odom, first):
        self.robot.drive(0)
        if first:
            # lower compliantly onto the upper tread; stop on contact
            self.both("front_lift", self.est.height - 0.006, max_vel=0.03, force=20.0)
            self._stall_t = 0.0
        moving = any(abs(self.robot.qd(f"{s}_front_lift")) > 0.003 for s in SIDES)
        self._stall_t = 0.0 if moving else self._stall_t + dt
        if t - self.t_enter > 0.4 and (self._stall_t > 0.3 or self.both_at("front_lift")):
            # proprioceptive riser height: where the front wheels actually touched
            self.h_contact = float(np.mean([self.robot.q(f"{s}_front_lift") for s in SIDES]))
            self.step_records[-1]["h_contact"] = self.h_contact
            # front rack must be able to out-run the rear racks to keep the body level
            self.both("front_lift", self.h_contact, force=C.RACK_FORCE, max_vel=2 * self.gait.lift_speed)
            self.pitch_pid.reset()
            self.goto("REAR_LIFT", t, f"front units on tread (contact at {self.h_contact * 1000:.1f} mm)")

    def _rear_lift(self, t, dt, odom, first):
        self.robot.drive(0)
        if first:
            self.both("leg", self.h_contact + self.gait.lift_margin + C.PAD_GAP, max_vel=self.gait.lift_speed)
        self._coordinate_front(dt, retracting=False)
        if t - self.t_enter > 0.3 and self.both_at("leg", 0.003):
            self.goto("ADVANCE_REAR", t)

    def _advance_rear(self, t, dt, odom, first):
        self._coordinate_front(dt, retracting=False)
        target = self.est.riser_x - PAD_RISER_GAP - PAD_FRONT_X
        if self.drive_to(target, odom, dt, self.gait.drive_speed):
            self.robot.drive(0)
            self.goto("RETRACT_REAR", t)

    def _retract_rear(self, t, dt, odom, first):
        self.robot.drive(0)
        if first:
            self.both("leg", 0.0, max_vel=self.gait.lift_speed)
        self._coordinate_front(dt, retracting=True)
        if self.both_at("leg", 0.003) and t - self.t_enter > 0.3:
            self.trim = 0.0
            self.both("front_lift", 0.0)
            if self.both_at("front_lift", 0.004):
                self.riser_hist.append(self.est.riser_x)
                self.tread += 1
                self.goto("SCAN", t, f"climbed step {self.tread}")

    def _clean_tread(self, t, dt, odom, first):
        self.robot.drive(0)
        if first:
            self._plan_cleaning(odom)
        if self.wp_i >= len(self.clean_wps):
            for j in ("clean_head", "clean_reach", "clean_rail"):
                self.robot.set_position(j, 0.0, max_vel=0.3, force=80)
            if abs(self.robot.q("clean_head")) < 0.005 and abs(self.robot.q("clean_reach")) < 0.01 \
                    and abs(self.robot.q("clean_rail")) < 0.01:
                if self.final:
                    self.result = "SUCCESS: top of staircase reached"
                    self.goto("DONE", t, self.result)
                else:
                    self.goto("FRONT_LIFT", t)
            return
        y, reach, head = self.clean_wps[self.wp_i]
        self.robot.set_position("clean_rail", y, max_vel=0.25, force=80)
        self.robot.set_position("clean_reach", reach, max_vel=0.2, force=80)
        self.robot.set_position("clean_head", head, max_vel=0.1, force=80)
        if (abs(self.robot.q("clean_rail") - y) < 0.004 and abs(self.robot.q("clean_reach") - reach) < 0.004
                and abs(self.robot.q("clean_head") - head) < 0.003):
            self.wp_i += 1
        if self.dirt is not None and self.robot.q("clean_head") > HEAD_DOWN - 0.01:
            pos, _ = self.robot.link_pose("clean_head")
            self.dirt.clean_at(pos[0], pos[1], pos[2] - 0.024,
                               C.HEAD_DEPTH / 2, C.HEAD_WIDTH / 2 + C.EDGE_BRUSH)

    def _plan_cleaning(self, odom):
        """Boustrophedon (lawn-mower) sweep of the tread under the robot."""
        x0 = self.riser_hist[-1] if self.riser_hist else odom - 0.3
        x1 = x0 + 0.5 if self.final else self.est.riser_x
        lo = max(x0 + C.HEAD_DEPTH / 2 - 0.02, odom - C.REACH_MAX)
        hi = min(x1 - C.HEAD_DEPTH / 2 - 0.005, odom + C.REACH_MAX)
        n_rows = max(1, int(math.ceil((hi - lo) / 0.12)) + 1)
        rows = np.linspace(lo, hi, n_rows) - odom
        ymax = self.robot.q("L_span") - C.UNIT_WIDTH / 2 - C.HEAD_WIDTH / 2 - 0.01
        ymin = -(self.robot.q("R_span") - C.UNIT_WIDTH / 2 - C.HEAD_WIDTH / 2 - 0.01)
        wps = [(ymin, rows[0], 0.0), (ymin, rows[0], HEAD_DOWN)]
        side = ymin
        for k, rx in enumerate(rows):
            if k > 0:
                wps.append((side, rx, HEAD_DOWN))
            side = ymax if side == ymin else ymin
            wps.append((side, rx, HEAD_DOWN))
        wps.append((side, rows[-1], 0.0))
        self.clean_wps, self.wp_i = wps, 0

    def _done(self, t, dt, odom, first):
        self.robot.drive(0)

    def _safe_abort(self, t, dt, odom, first):
        self.robot.drive(0)
