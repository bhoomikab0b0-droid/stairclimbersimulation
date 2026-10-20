# Adaptive Stair-Climbing Vacuum Cleaner: Autonomous Simulation

A 3D physics simulation (Python + PyBullet) of the stair-climbing cleaning robot described in the team's invention disclosure, *Adaptive Stair-Climbing Vacuum Cleaner*. The design comes from MIT Bengaluru.

The robot climbs the staircase on its own:
1. It **perceives** each step with a simulated LiDAR, side rangefinders and an IMU.
2. It **decides** with a finite state machine and a gait library.
3. It **controls** its rack-and-pinion lifts, frame width and wheels with PID loops.
4. It **cleans** each tread with the movable mop/vacuum module.

## Quick start

```bash
./setup.sh                      # one time: creates .venv, installs pybullet (patched build on macOS)
source .venv/bin/activate

python main.py                  # GUI: 5 standard steps
python main.py --randomize --seed 3          # every step a different height/depth
python main.py --width 1.2 --randomize       # wider stairs -> frame extends
python main.py --headless --randomize        # no window, prints report + saves plots
python main.py --headless --snapshots        # also saves rendered frames of every gait phase
python main.py --height 0.24                 # riser too high  -> SAFE_ABORT
python main.py --depth 0.29                  # tread too shallow -> SAFE_ABORT
python -m analysis.batch --runs 30           # success-rate evaluation over random staircases
python -m pytest tests/                      # perception unit tests
```

Each run writes a report folder to `runs/<timestamp>/`. It contains `log.csv`, `summary.json`, the plots, and the snapshots if you asked for them. In the GUI, the overlay above the robot shows the current state, the step estimates, the pitch and the cleaning coverage. LiDAR beams are drawn while the robot scans.

## From the patent design to the simulation

| Patent / proposal | In the simulation |
|---|---|
| Two side drive units, each with a red front unit (100 mm) and a blue back unit (200 mm); height 314 mm | Two rigid bodies per side with mass, built to those dimensions (`config.py`) |
| Rack-and-pinion climbing actuator | `*_front_lift`: a prismatic rack that raises the red front unit relative to the blue back unit. `*_leg`: a rear rack with a pad that pushes the back unit up |
| Two telescopic rods with variable width *x* (500–1500 mm) and a belt-driven linear actuator | `L_span` / `R_span` prismatic joints. Each side unit is set to keep 30 mm clearance to its wall |
| Omni wheels (3 per side) | Driven wheels with low sideways friction (passive-roller approximation) |
| Central cleaning unit, movable mop, vacuum, edge brushes | Unit that slides along the rods (`clean_rail`), moves fore/aft (`clean_reach`) and lowers its head (`clean_head`). It runs a lawn-mower sweep over each tread |
| Camera + LiDAR, ultrasonic/IR, IMU, Kalman filtering | Vertical-fan LiDAR (160 beams, ±4 mm noise), side rangefinders, noisy IMU, wheel odometry with drift, and 1-D Kalman filters |
| Raspberry Pi (AI) + Arduino (motors), PID, gait library | `perception/` (Pi), `control/fsm.py` + `gait_library.py` (decision), `control/pid.py` (motor loops) |

**Interpretation note:** the patent text calls *x* the "step depth", but the drawings and slide 6 show *x* spanning the **stair width**, with the side units on either side of the stairs. The simulation follows the drawings, so *x* adapts to the stair width. Height and depth are handled by the racks and the gait.

## Architecture

```
            ┌──────────────── PERCEPTION ─────────────────┐
 LiDAR ───► │ scan levelled by IMU → PCA line labelling → │
            │ tread/riser segments → h, d, riser distance │──► Kalman filters ─┐
 Side IR ─► │ wall distances → stair width w              │                    │
 IMU ─────► │ pitch / roll / yaw                          │                    ▼
 Odometry ► │ position along the stairs                   │        ┌──── DECISION ────┐
            └─────────────────────────────────────────────┘        │ state machine +  │
                                                                   │ gait library +   │
                                                                   │ safety monitor   │
                                                                   └────────┬─────────┘
                                                                            ▼
                         ┌───────────────────── CONTROL ─────────────────────────┐
                         │ rack PIDs · IMU pitch-levelling PID · distance PID     │
                         │ · heading-hold PID · frame-width servo · cleaning moves│
                         └────────────────────────────────────────────────────────┘
```

### Climbing gait (one step)

| State | What happens |
|---|---|
| `SCAN` | 12 LiDAR scans are Kalman-fused into the riser height, tread depth, riser position and stair width |
| `ALIGN_WIDTH` | The telescopic rods set *x* so both side units sit 30 mm from their walls |
| `APPROACH` | Drive until the front units are 15 mm from the riser (distance PID + heading-hold PID) |
| `CLEAN_TREAD` | The mop/vacuum head lowers and sweeps the tread in rows (method claim 9f: drive units stationary) |
| `FRONT_LIFT` | The front racks lift the red units by *h* + margin; the blue units stabilise (claim 9d) |
| `ADVANCE_FRONT` | The back-unit wheels push the raised front units over the next tread |
| `PLACE_FRONT` | The front units lower compliantly until they touch. The contact height is a second, proprioceptive measurement of *h* |
| `REAR_LIFT` | The rear racks lift the blue units. The front rack moves the opposite way so the front stays on the tread, and an IMU pitch PID trims it to keep the body level |
| `ADVANCE_REAR` | The front-unit wheels pull the raised back units over the tread |
| `RETRACT_REAR` | The rear racks retract and the back units land on the tread |

When there is no riser ahead, the robot cleans the landing and enters `DONE`. Any safety violation sends it to `SAFE_ABORT`, where it stops and holds every actuator. The violations are: pitch above 20°, roll above 15°, a state timeout, or a step the gait library rejects.

The gait library chooses `LOW` for risers up to 130 mm, `STANDARD` up to 185 mm and `HIGH` up to 220 mm. It rejects a riser above the rack limit, a tread shallower than 320 mm (the side unit must fit on it), and stairs narrower or wider than the frame can span. A riser that rises above the LiDAR (so its top can't be seen) is also rejected.

## Results (30 random staircases, `python -m analysis.batch --runs 30`)

Each staircase has 5 steps. Riser heights are 140–200 mm, tread depths 325–375 mm and widths 0.75–1.30 m, all varied randomly step by step.

| Metric | Result |
|---|---|
| Runs reaching the top | **30 / 30** |
| Step climbing success rate | **100 %** |
| Mean riser-height error (LiDAR + Kalman) | **1.7 mm** |
| Mean tread-depth error | **0.3 mm** |
| Worst body pitch / roll during climbing | **7.1° / 0.07°** (limits 20° / 15°) |
| Mean time per step (including cleaning) | 31.7 s |
| Tread area cleaned between the side units | 93.6 % |

Safety cases: a 240 mm riser triggers `SAFE_ABORT` (too high), a 290 mm tread triggers `SAFE_ABORT` (too shallow), and a 0.60 m stairwell is rejected as narrower than the robot.

## Interactive web simulation (for presenting)

`web/index.html` is a live simulation that runs in the browser. Nothing is prerecorded. You can set:
- **Staircase:** layout (U-turn with a half landing, or straight), riser height, tread depth, width, steps per flight, landing depth, and step-to-step variation.
- **Surfaces:** wood, tile, marble or carpet for each flight and the landing. The surface changes wheel slip, LiDAR noise, cleaning mode and speed.
- **Obstacles:** a toy block, a shoe or a parcel, placed on any step or the landing, and added or removed while the robot runs.
- **Sensor noise:** the LiDAR range noise.

What the page shows:
- The robot in 3D, with every sensor drawn: LiDAR beams, depth-camera points, ultrasonic cones, IR cliff beams, and the occupancy map and A* path on the landing.
- A picture-in-picture view from the robot's own depth camera.
- A live Sense → Perceive → Decide → Act panel.
- Live charts: the LiDAR segmentation, Kalman convergence, and pitch with the rack positions.
- A decision log and a per-step accuracy report.

The web version reuses the perception and control algorithms from the Python code, ported to JavaScript and extended with:
- obstacle detection and a surface classifier from the depth camera
- the landing planner and the 180° turn
- sensor-terminated moves, where the bumper or cliff sensor ends a move so wheel slip cannot fool the robot

Instead of PyBullet it uses a rigid-body contact model. Each step, the body's resting pose is solved from its wheel and pad contacts, and the model includes collisions, wheel slip and sensor noise.

To show it: double-click `web/index.html` (works offline; three.js and the fonts are bundled in `web/vendor` and `web/fonts`).
`StairClimber_WebApp.zip` is the same app packaged for a USB stick or email.

```bash
node web/test/batch.js                     # 24 random staircases, headless
node web/test/scenarios.js                 # obstacle and safety scenarios
```

Plots produced for every run:
- `perception_lidar.png`: the segmented LiDAR scan.
- `perception_kalman.png`: Kalman convergence.
- `perception_accuracy.png`: estimated vs true height and depth.
- `control_racks.png`: PID tracking of the racks.
- `control_tilt.png`: pitch and roll with the safety limits.
- `fsm_timeline.png`: the state machine timeline.

## Simplifications (honest list)

- **Omni wheels** are ordinary wheels with reduced sideways friction. There are no individual rollers.
- **Rear rack pad** is a low-friction skid so the back unit can be pulled forward on it. The patent describes a high-friction pad; a real build could add a small horizontal slide or roller.
- **Cleaning** is computed geometrically. A dirt cell counts as clean when the lowered head (plus edge brushes) passes over it. Suction airflow is not simulated, and the strips directly under the side units are not reachable by the central module.
- **Camera / CNN surface classification**, **SLAM** and **reinforcement learning** from the proposal are not part of this simulation. Step detection uses geometric LiDAR processing, which is what the climbing decision actually needs.
- **Both side units move in sync**, and only ascent is simulated (descent is future work).
- The unit width (150 mm) is an assumption; the drawing label is unclear.

## Code map

```
config.py                 dimensions, limits, noise levels (patent values marked)
main.py                   runner: world setup, main loop, GUI overlay, report, logging
sim/stairs.py             procedural staircase + stepped side walls
sim/robot_urdf.py         generates the robot model (URDF) from config
sim/robot.py              joint access, IMU, odometry, wheel drive
perception/lidar.py       ray-cast LiDAR fan + side rangefinders (with noise)
perception/step_detector.py  scan → treads/risers → h, d, riser distance
perception/kalman.py      1-D Kalman filter
perception/perception.py  fuses everything into a step estimate
control/pid.py            PID + joint servo
control/gait_library.py   gait profiles and feasibility checks
control/fsm.py            the climbing/cleaning state machine
cleaning/dirt.py          dirt grid and coverage
analysis/plots.py         report figures
analysis/batch.py         multi-run evaluation
tests/                    perception unit tests (synthetic scans)
```
