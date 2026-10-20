"""
Adaptive Stair-Climbing Vacuum Cleaner — autonomous simulation.

Examples
  python main.py                              # GUI, 5 standard steps
  python main.py --randomize --seed 3         # every step a different size
  python main.py --headless --randomize       # fast run, prints the report
  python main.py --height 0.24                # riser too high -> safe abort
"""
import argparse
import csv
import json
import math
import os
import time

import numpy as np
import pybullet as p

import config as C
from cleaning.dirt import DirtMap
from control.fsm import ClimbFSM
from perception.perception import FRONT_FACE_X, Perception
from sim.robot import Robot
from sim.stairs import build_staircase


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steps", type=int, default=5)
    ap.add_argument("--height", type=float, default=0.17, help="riser height (m)")
    ap.add_argument("--heights", type=str, default="",
                    help="comma-separated riser height per step, e.g. 0.16,0.17,0.24")
    ap.add_argument("--depth", type=float, default=0.35, help="tread depth (m)")
    ap.add_argument("--width", type=float, default=0.90, help="stair width wall-to-wall (m)")
    ap.add_argument("--randomize", action="store_true", help="vary every step's height and depth")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--span", type=float, default=0.60, help="initial frame width x (m)")
    ap.add_argument("--headless", action="store_true", help="no GUI, run as fast as possible")
    ap.add_argument("--speed", type=float, default=1.0, help="GUI playback speed multiplier")
    ap.add_argument("--max-time", type=float, default=600.0, help="simulated seconds before giving up")
    ap.add_argument("--out", default="runs", help="folder for logs and plots")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--record", metavar="FILE.json", help="record the run for the web replay viewer")
    ap.add_argument("--title", default=None, help="scenario title shown in the web viewer")
    ap.add_argument("--snapshots", action="store_true",
                    help="save rendered PNG frames at every state change (works headless)")
    return ap.parse_args()


class Overlay:
    """On-screen status text that follows the robot in the GUI."""

    def __init__(self):
        self.ids = [p.addUserDebugText(" ", [0, 0, 0]) for _ in range(4)]

    def update(self, base, lines):
        for k, (txt, col) in enumerate(lines[:4]):
            p.addUserDebugText(txt, [base[0] - 0.3, base[1] + 0.2, base[2] + 0.75 - 0.08 * k], col,
                               textSize=1.3, replaceItemUniqueId=self.ids[k])


def run(args):
    p.connect(p.DIRECT if args.headless else p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(C.DT)
    p.setPhysicsEngineParameter(numSolverIterations=100)
    gui = not args.headless
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    spec = C.StairSpec(n_steps=args.steps, height=args.height, depth=args.depth, width=args.width,
                       randomize=args.randomize, seed=args.seed,
                       heights=tuple(float(v) for v in getattr(args, "heights", "").split(",") if v))
    stairs = build_staircase(spec)
    rng = np.random.default_rng(args.seed + 100)
    span0 = min(args.span, args.width - C.UNIT_WIDTH - 0.04)
    if span0 < C.SPAN_MIN:
        print(f"Stairwell {args.width:.2f} m is narrower than the robot at its minimum frame width "
              f"({C.SPAN_MIN + C.UNIT_WIDTH:.2f} m): not supported (patent limitation).")
        p.disconnect()
        return None
    robot = Robot(start_x=-0.35 - FRONT_FACE_X, span=span0, rng=rng)
    dirt = DirtMap(stairs, visual=gui or args.snapshots, seed=args.seed)
    per = Perception(robot, rng)
    fsm = ClimbFSM(robot, per, dirt)
    overlay = Overlay() if gui else None
    recorder = None
    if getattr(args, "record", None):
        from analysis.recorder import Recorder
        recorder = Recorder(robot, stairs, dirt, fsm, per, dict(
            title=args.title or "Stair climb", steps=args.steps, randomize=args.randomize, seed=args.seed,
            width=args.width, height=args.height, depth=args.depth))
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    log, t, k = [], 0.0, 0
    log_every = C.PHYSICS_HZ // 30
    wall0 = time.time()
    last_state = None
    while t < args.max_time:
        fsm.tick(t, C.DT)
        p.stepSimulation()
        t += C.DT
        k += 1
        if recorder:
            recorder.step(t)
        if fsm.state != last_state:
            ev = fsm.events[-1] if fsm.events else (0, fsm.state, "")
            print(f"[{t:7.2f}s] {fsm.state:<13} {ev[2]}")
            if args.snapshots:
                snapshot(args, robot, t, fsm.state, last_state)
            last_state = fsm.state
        if k % log_every == 0:
            pos = p.getBasePositionAndOrientation(robot.id)[0]
            rr, pp, _ = robot.true_rpy()
            log.append(dict(t=round(t, 3), state=fsm.state, tread=fsm.tread, x=pos[0], z=pos[2],
                            pitch_deg=math.degrees(-pp), roll_deg=math.degrees(rr),
                            imu_pitch_deg=math.degrees(fsm.pitch),
                            front_lift=robot.q("L_front_lift"), leg=robot.q("L_leg"),
                            front_lift_target=fsm.servo["L_front_lift"].target,
                            leg_target=fsm.servo["L_leg"].target, trim=fsm.trim,
                            span=robot.span(), coverage=dirt.coverage()))
            if gui:
                if fsm.state == "SCAN":
                    per.lidar.draw()
                est = fsm.est
                est_txt = (f"est h={est.height * 1000:.0f}mm d={est.depth * 1000 if est.depth else 0:.0f}mm"
                           if est is not None and est.step_ahead else "est: -")
                overlay.update(pos, [
                    (f"STATE: {fsm.state}", [0.9, 0.1, 0.1]),
                    (f"tread {fsm.tread}/{stairs.n_steps}   x={robot.span() * 1000:.0f}mm", [0, 0, 0]),
                    (est_txt, [0, 0, 0.6]),
                    (f"pitch {math.degrees(fsm.pitch):+.1f} deg  cleaned {dirt.coverage() * 100:.0f}%", [0, 0.4, 0]),
                ])
                p.resetDebugVisualizerCamera(1.8, -120, -28, [pos[0] + 0.2, 0, pos[2] - 0.05])
        if gui:
            lag = t / args.speed - (time.time() - wall0)
            if lag > 0:
                time.sleep(lag)
        if fsm.finished:
            # linger a little so the final pose is visible / settled
            for _ in range(int(C.PHYSICS_HZ * (2.0 if gui else 0.5))):
                fsm.tick(t, C.DT)
                p.stepSimulation()
                t += C.DT
                if recorder:
                    recorder.step(t)
                if gui:
                    time.sleep(C.DT / args.speed)
            break

    result = fsm.result or "TIMEOUT: max simulated time reached"
    summary = report(fsm, stairs, dirt, log, result, t)
    save(args, fsm, stairs, log, summary)
    if recorder:
        print(f"Replay recorded to {recorder.save(args.record, summary)}")
    p.disconnect()
    return summary


_snap_n = [0]


def snapshot(args, robot, t, state, prev_state):
    """Offscreen render of the scene (TinyRenderer, no GUI needed). The frame is
    taken at the END of the previous state, so it shows what that state achieved."""
    import matplotlib.pyplot as plt
    pos = p.getBasePositionAndOrientation(robot.id)[0]
    view = p.computeViewMatrixFromYawPitchRoll([pos[0] + 0.2, 0, pos[2] - 0.05], 1.8, -120, -28, 0, 2)
    proj = p.computeProjectionMatrixFOV(50, 1.6, 0.05, 10)
    w, h = 960, 600
    img = p.getCameraImage(w, h, view, proj, renderer=p.ER_TINY_RENDERER, shadow=1,
                           lightDirection=[0.4, -0.6, 1])[2]
    img = np.reshape(np.asarray(img, dtype=np.uint8), (h, w, 4))[:, :, :3]
    folder = os.path.join(args.out, "snapshots")
    os.makedirs(folder, exist_ok=True)
    _snap_n[0] += 1
    plt.imsave(os.path.join(folder, f"{_snap_n[0]:03d}_{t:06.1f}s_after_{prev_state or 'START'}.png"), img)


def report(fsm, stairs, dirt, log, result, t):
    rows = []
    risers0 = fsm.step_records[0]["riser_est"] if fsm.step_records and fsm.step_records[0]["riser_est"] else None
    for r in fsm.step_records:
        i = r["tread"]
        if not r["step_ahead"] or r.get("blocked") or i >= stairs.n_steps:
            continue
        true_h, true_d = stairs.heights[i], stairs.depths[i]
        rows.append(dict(step=i + 1, h_true=true_h, h_est=r["h_est"], h_contact=r.get("h_contact"),
                         d_true=true_d if i < stairs.n_steps - 1 else None, d_est=r["d_est"],
                         riser_true=stairs.risers_x[i], riser_est=r["riser_est"], gait=r.get("gait")))
    h_err = [abs(r["h_est"] - r["h_true"]) for r in rows if r["h_est"] is not None]
    d_err = [abs(r["d_est"] - r["d_true"]) for r in rows if r["d_est"] is not None and r["d_true"] is not None]
    pitch = [abs(l["pitch_deg"]) for l in log]
    roll = [abs(l["roll_deg"]) for l in log]
    summary = dict(
        result=result, sim_time_s=round(t, 1), steps_climbed=fsm.tread, steps_total=stairs.n_steps,
        stair_width=stairs.width, frame_width_final=round(fsm.robot.span(), 3),
        width_est=fsm.per.kf_w.x,
        mean_abs_height_error_mm=round(1000 * float(np.mean(h_err)), 2) if h_err else None,
        max_abs_height_error_mm=round(1000 * float(np.max(h_err)), 2) if h_err else None,
        mean_abs_depth_error_mm=round(1000 * float(np.mean(d_err)), 2) if d_err else None,
        max_pitch_deg=round(max(pitch), 2) if pitch else None,
        max_roll_deg=round(max(roll), 2) if roll else None,
        cleaning_coverage_total=round(dirt.coverage(), 3),
        cleaning_coverage_reachable={i: round(dirt.reachable_coverage(i, fsm.robot.span()), 3) for i in dirt.treads},
        steps=rows, events=fsm.events,
    )
    print("\n==================== RUN SUMMARY ====================")
    print(f"Result           : {result}")
    print(f"Steps climbed    : {fsm.tread}/{stairs.n_steps}   in {t:.1f} s simulated")
    if fsm.per.kf_w.x:
        print(f"Stair width      : true {stairs.width * 1000:.0f} mm, est {fsm.per.kf_w.x * 1000:.0f} mm, "
              f"frame x set to {fsm.robot.span() * 1000:.0f} mm")
    print(f"{'step':>4} {'gait':>9} {'h true':>8} {'h est':>8} {'h touch':>8} {'d true':>8} {'d est':>8}   (mm)")
    for r in rows:
        f = lambda v: f"{v * 1000:8.1f}" if v is not None else "       -"
        print(f"{r['step']:>4} {str(r['gait']):>9} {f(r['h_true'])} {f(r['h_est'])} {f(r['h_contact'])} "
              f"{f(r['d_true'])} {f(r['d_est'])}")
    print(f"Height error     : mean {summary['mean_abs_height_error_mm']} mm, max {summary['max_abs_height_error_mm']} mm")
    print(f"Depth error      : mean {summary['mean_abs_depth_error_mm']} mm")
    print(f"Max tilt         : pitch {summary['max_pitch_deg']} deg, roll {summary['max_roll_deg']} deg")
    reach = summary["cleaning_coverage_reachable"]
    print(f"Cleaning         : {summary['cleaning_coverage_total'] * 100:.1f}% of all tread area; "
          f"between side units per tread: " + ", ".join(f"{k}:{v * 100:.0f}%" for k, v in reach.items()))
    print("=====================================================\n")
    return summary


def save(args, fsm, stairs, log, summary):
    name = time.strftime("%Y%m%d-%H%M%S") + (f"_seed{args.seed}" if args.randomize else "")
    out = os.path.join(args.out, name)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "log.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(log[0].keys()))
        w.writeheader()
        w.writerows(log)
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2, default=float)
    truth = dict(heights=stairs.heights, depths=stairs.depths, risers=stairs.risers_x)
    with open(os.path.join(out, "scans.json"), "w") as f:
        json.dump(dict(truth=truth, steps=[dict(tread=r["tread"], scans=r["scans"]) for r in fsm.step_records]),
                  f, default=lambda v: None if v is None else float(v))
    if fsm.step_records and fsm.step_records[0]["scan_pts"] is not None:
        # first scan from the floor: used for the perception figure
        np.save(os.path.join(out, "first_scan.npy"), fsm.step_records[0]["scan_pts"])
    print(f"Logs saved to {out}/")
    if not args.no_plots:
        from analysis.plots import make_plots
        make_plots(out)


if __name__ == "__main__":
    run(parse_args())
