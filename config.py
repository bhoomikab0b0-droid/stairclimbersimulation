"""
Central configuration for the Adaptive Stair-Climbing Vacuum Cleaner simulation.

All lengths are in metres, masses in kg, angles in radians unless noted.
Values marked [patent] come from the Invention Disclosure Form drawings/tables;
values marked [assumed] are engineering choices made for the simulation.
"""
import math
from dataclasses import dataclass

# ----------------------------------------------------------------------------
# Robot geometry
# ----------------------------------------------------------------------------
FRONT_LEN = 0.10          # [patent] red front unit length (top view: 100 mm)
BACK_LEN = 0.20           # [patent] blue back unit length (top view: 200 mm)
UNIT_WIDTH = 0.15         # [assumed] lateral width of each side unit
TOTAL_HEIGHT = 0.314      # [patent] total height incl. wheels (314 mm)
WHEEL_RADIUS = 0.035      # [assumed] omni-wheel radius
WHEEL_WIDTH = 0.025
GROUND_CLEARANCE = 0.03   # wheel bottom -> body bottom
BODY_HEIGHT = TOTAL_HEIGHT - GROUND_CLEARANCE
BASE_Z = GROUND_CLEARANCE + BODY_HEIGHT / 2   # body centre height above the tread
BACK_WHEEL_X = 0.06       # back-unit wheels at +/- this from back-unit centre

# Adjustable connecting frame (telescopic rods), 'x' in the patent drawings
SPAN_MIN = 0.50           # [patent] x min = 50 cm
SPAN_MAX = 1.50           # [patent] x max = 150 cm
WALL_CLEARANCE = 0.03     # gap kept between each side unit and the stair wall

# Rack-and-pinion climbing actuators
RACK_TRAVEL = 0.25        # [assumed] max vertical stroke of each rack
FRONT_LIFT_MIN = -0.05
PAD_RADIUS = 0.012        # rear rack pad (modelled as a low-friction skid/roller)
PAD_GAP = 0.01            # pad sits this far above ground when the rack is retracted
PAD_BEHIND = 0.01         # pad centre is this far behind the back unit's rear face

# Cleaning unit [patent: 150 mm]
CLEANER_SIZE = 0.15
HEAD_DEPTH = 0.15         # mop/vacuum head footprint (x)
HEAD_WIDTH = 0.15         # mop/vacuum head footprint (y)
EDGE_BRUSH = 0.04         # edge brushes widen the cleaned strip on each side
REACH_MAX = 0.20          # fore/aft travel of the movable mop module

# Masses
M_BACK = 2.5              # electronics, battery, rods housing
M_FRONT = 1.0
M_FRAME = 0.3
M_CLEANER = 0.6
M_HEAD = 0.2
M_WHEEL = 0.1
M_LEG = 0.1

# ----------------------------------------------------------------------------
# Actuator limits
# ----------------------------------------------------------------------------
RACK_FORCE = 250.0        # N, max force of the rack-and-pinion motors
RACK_FORCE_COMPLIANT = 60.0
SPAN_FORCE = 300.0
WHEEL_TORQUE = 6.0        # N*m
DRIVE_SPEED = 0.08        # m/s nominal

# Friction
FRICTION_STAIR = 0.9
FRICTION_WHEEL = 1.0
FRICTION_PAD = 0.03

# ----------------------------------------------------------------------------
# Sensors
# ----------------------------------------------------------------------------
LIDAR_RAYS = 160
LIDAR_FOV_DOWN = math.radians(-85)
LIDAR_FOV_UP = math.radians(25)
LIDAR_RANGE = 2.0
LIDAR_NOISE = 0.004       # m, 1-sigma range noise
LIDAR_HZ = 20
SIDE_RANGE_NOISE = 0.005  # side ultrasonic/IR rangefinders
IMU_NOISE = math.radians(0.3)
ODOM_NOISE = 0.0005       # m per control tick random walk

# ----------------------------------------------------------------------------
# Safety limits (patent "Limitations" section)
# ----------------------------------------------------------------------------
MAX_PITCH = math.radians(20)
MAX_ROLL = math.radians(15)
MIN_RISER = 0.04          # anything lower is treated as flat floor
MODULE_LEN = BACK_LEN + FRONT_LEN
MIN_TREAD = MODULE_LEN + 0.02          # tread must fit the whole side unit + approach gap

# ----------------------------------------------------------------------------
# Simulation timing
# ----------------------------------------------------------------------------
PHYSICS_HZ = 240
DT = 1.0 / PHYSICS_HZ


@dataclass
class StairSpec:
    """Geometry of the staircase to generate."""
    n_steps: int = 5
    height: float = 0.17       # riser height
    depth: float = 0.35        # tread depth
    width: float = 0.90        # stair width (wall to wall)
    randomize: bool = False    # vary each step's height/depth independently
    height_jitter: float = 0.03
    depth_jitter: float = 0.025
    seed: int = 0
    heights: tuple = ()        # optional explicit riser height for each step
    approach: float = 0.6      # floor length in front of the first riser
    landing: float = 1.0       # depth of the top landing
