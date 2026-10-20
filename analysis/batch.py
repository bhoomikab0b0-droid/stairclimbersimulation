"""
Batch evaluation: run many randomized staircases headless and report the
metrics from the project proposal (climbing success rate, perception error,
tilt, cleaning coverage, time per step).

    python -m analysis.batch --runs 20
"""
import argparse
import contextlib
import io
import json
import os
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main import run  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--steps", type=int, default=5)
    ap.add_argument("--out", default="runs/batch")
    a = ap.parse_args()
    rng = np.random.default_rng(1234)
    results = []
    for k in range(a.runs):
        width = float(rng.uniform(0.75, 1.3))
        args = SimpleNamespace(steps=a.steps, height=0.17, depth=0.35, width=width, randomize=True, seed=k,
                               span=0.6, headless=True, speed=1.0, max_time=600.0, out=a.out, no_plots=True, snapshots=False, record=None, title=None)
        with contextlib.redirect_stdout(io.StringIO()):
            s = run(args)
        ok = s["result"].startswith("SUCCESS")
        results.append(dict(seed=k, width=round(width, 3), ok=ok, result=s["result"],
                            steps=s["steps_climbed"], h_err=s["mean_abs_height_error_mm"],
                            d_err=s["mean_abs_depth_error_mm"], max_pitch=s["max_pitch_deg"],
                            max_roll=s["max_roll_deg"], time=s["sim_time_s"],
                            clean=float(np.mean(list(s["cleaning_coverage_reachable"].values())))))
        print(f"seed {k:3d}  width {width:.2f} m  {'OK  ' if ok else 'FAIL'}  steps {s['steps_climbed']}/{a.steps}  "
              f"h_err {s['mean_abs_height_error_mm']} mm  pitch {s['max_pitch_deg']} deg  {s['result']}")
    ok = [r for r in results if r["ok"]]
    total_steps = sum(r["steps"] for r in results)
    attempted = sum(min(a.steps, r["steps"] + (0 if r["ok"] else 1)) for r in results)
    summary = dict(
        runs=len(results), successful_runs=len(ok),
        step_climb_success_rate=round(total_steps / max(1, attempted), 3),
        mean_height_error_mm=round(float(np.mean([r["h_err"] for r in results if r["h_err"] is not None])), 2),
        mean_depth_error_mm=round(float(np.mean([r["d_err"] for r in results if r["d_err"] is not None])), 2),
        worst_pitch_deg=max(r["max_pitch"] for r in results),
        worst_roll_deg=max(r["max_roll"] for r in results),
        mean_time_per_step_s=round(float(np.mean([r["time"] / a.steps for r in ok])), 1) if ok else None,
        mean_reachable_cleaning=round(float(np.mean([r["clean"] for r in ok])), 3) if ok else None,
        runs_detail=results)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "batch_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print("\n" + json.dumps({k: v for k, v in summary.items() if k != "runs_detail"}, indent=2))


if __name__ == "__main__":
    main()
