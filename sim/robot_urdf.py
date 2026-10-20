"""
Generates the robot URDF from config.py so the geometry always matches the
patent dimensions.

Kinematic tree (x = forward/up the stairs, y = across the stairs, z = up):

frame (centre of the telescopic rods)
 |- L_span / R_span        prismatic y  : telescopic rods, sets width 'x'
 |   `- {L,R}_back         blue back unit (electronics, battery)
 |       |- *_wheel_br/bf  continuous   : omni wheels
 |       |- *_front_lift   prismatic z  : rack-and-pinion lifting the red front unit
 |       |   `- {L,R}_front red front unit
 |       |       |- *_wheel_f continuous
 |       |       `- *_lidar   fixed      : front-facing LiDAR / camera
 |       `- *_leg          prismatic -z : rear rack with pad (pushes body up)
 `- clean_rail             prismatic y  : cleaning unit slides along the rods
     `- clean_reach        prismatic x  : movable mop module fore/aft
         `- clean_head     prismatic -z : lowers mop + vacuum inlet onto the tread
"""
import math
import os

import config as C

BLUE = "0.12 0.25 0.85 1"
RED = "0.80 0.10 0.10 1"
GREEN = "0.20 0.65 0.20 1"
YELLOW = "0.95 0.85 0.10 1"
BLACK = "0.08 0.08 0.08 1"
ORANGE = "0.90 0.45 0.10 1"
TEAL = "0.10 0.70 0.55 1"


def _inertia_box(m, x, y, z):
    return (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12)


def _inertial(m, ixx, iyy, izz):
    return (f'<inertial><mass value="{m}"/>'
            f'<inertia ixx="{ixx:.6g}" ixy="0" ixz="0" iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}"/></inertial>')


def _box_geom(x, y, z):
    return f'<geometry><box size="{x} {y} {z}"/></geometry>'


def _visual(geom, rgba, xyz="0 0 0", rpy="0 0 0"):
    return f'<visual><origin xyz="{xyz}" rpy="{rpy}"/>{geom}<material name="m_{rgba.replace(" ", "_")}"><color rgba="{rgba}"/></material></visual>'


def _collision(geom, xyz="0 0 0", rpy="0 0 0"):
    return f'<collision><origin xyz="{xyz}" rpy="{rpy}"/>{geom}</collision>'


def _link(name, inertial, visuals=(), collisions=()):
    return f'<link name="{name}">{inertial}{"".join(visuals)}{"".join(collisions)}</link>'


def _joint(name, jtype, parent, child, xyz, axis="1 0 0", lower=None, upper=None, effort=1000, vel=10):
    lim = ""
    if lower is not None:
        lim = f'<limit lower="{lower}" upper="{upper}" effort="{effort}" velocity="{vel}"/>'
    elif jtype == "continuous":
        lim = f'<limit effort="{effort}" velocity="{vel}"/>'
    return (f'<joint name="{name}" type="{jtype}"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz}" rpy="0 0 0"/><axis xyz="{axis}"/>{lim}</joint>')


def _wheel(name):
    r, w = C.WHEEL_RADIUS, C.WHEEL_WIDTH
    geom = f'<geometry><cylinder radius="{r}" length="{w}"/></geometry>'
    rpy = f"{math.pi / 2} 0 0"
    ixx = C.M_WHEEL * (3 * r * r + w * w) / 12
    hub = f'<geometry><cylinder radius="{r * 0.55}" length="{w * 1.05}"/></geometry>'
    return _link(name, _inertial(C.M_WHEEL, ixx, C.M_WHEEL * r * r / 2, ixx),
                 [_visual(geom, BLACK, rpy=rpy), _visual(hub, ORANGE, rpy=rpy)],
                 [_collision(geom, rpy=rpy)])


def build_urdf() -> str:
    H = C.BODY_HEIGHT
    parts = ['<?xml version="1.0"?>', '<robot name="stair_climber">']

    # Central frame: short rod section; rod stubs on each back unit complete the picture
    frame_vis = [_visual(f'<geometry><cylinder radius="0.012" length="0.7"/></geometry>', BLACK,
                         xyz=f"0 0 {dz}", rpy=f"{math.pi / 2} 0 0") for dz in (0.08, 0.02)]
    parts.append(_link("frame", _inertial(C.M_FRAME, *_inertia_box(C.M_FRAME, 0.05, 0.7, 0.05)), frame_vis))

    for side, sgn in (("L", 1), ("R", -1)):
        # telescopic rod joint sets the half-span
        parts.append(_joint(f"{side}_span", "prismatic", "frame", f"{side}_back", "0 0 0",
                            axis=f"0 {sgn} 0", lower=C.SPAN_MIN / 2, upper=C.SPAN_MAX / 2))
        rod_len = 0.45
        rods = [_visual(f'<geometry><cylinder radius="0.016" length="{rod_len}"/></geometry>', BLACK,
                        xyz=f"0 {-sgn * (C.UNIT_WIDTH / 2 + rod_len / 2)} {dz}", rpy=f"{math.pi / 2} 0 0")
                for dz in (0.08, 0.02)]
        rack = _visual(_box_geom(0.012, 0.03, H + C.RACK_TRAVEL), BLACK,
                       xyz=f"{C.BACK_LEN / 2 - 0.006} 0 {C.RACK_TRAVEL / 2}")
        body = _box_geom(C.BACK_LEN, C.UNIT_WIDTH, H)
        parts.append(_link(f"{side}_back", _inertial(C.M_BACK, *_inertia_box(C.M_BACK, C.BACK_LEN, C.UNIT_WIDTH, H)),
                           [_visual(body, BLUE), rack] + rods, [_collision(body)]))

        wz = -H / 2 + C.WHEEL_RADIUS - C.GROUND_CLEARANCE
        for tag, wx in (("br", -C.BACK_WHEEL_X), ("bf", C.BACK_WHEEL_X)):
            parts.append(_joint(f"{side}_wheel_{tag}", "continuous", f"{side}_back", f"{side}_wheel_{tag}",
                                f"{wx} {sgn * (C.UNIT_WIDTH / 2 - C.WHEEL_WIDTH / 2 - 0.005)} {wz}", axis="0 1 0"))
            parts.append(_wheel(f"{side}_wheel_{tag}"))

        # Front unit on the climbing rack
        parts.append(_joint(f"{side}_front_lift", "prismatic", f"{side}_back", f"{side}_front",
                            f"{C.BACK_LEN / 2 + C.FRONT_LEN / 2 + 0.001} 0 0", axis="0 0 1",
                            lower=C.FRONT_LIFT_MIN, upper=C.RACK_TRAVEL))
        fbody = _box_geom(C.FRONT_LEN, C.UNIT_WIDTH, H)
        parts.append(_link(f"{side}_front", _inertial(C.M_FRONT, *_inertia_box(C.M_FRONT, C.FRONT_LEN, C.UNIT_WIDTH, H)),
                           [_visual(fbody, RED)], [_collision(fbody)]))
        parts.append(_joint(f"{side}_wheel_f", "continuous", f"{side}_front", f"{side}_wheel_f",
                            f"0 {sgn * (C.UNIT_WIDTH / 2 - C.WHEEL_WIDTH / 2 - 0.005)} {wz}", axis="0 1 0"))
        parts.append(_wheel(f"{side}_wheel_f"))
        parts.append(_joint(f"{side}_lidar", "fixed", f"{side}_front", f"{side}_lidar",
                            f"{C.FRONT_LEN / 2 + 0.01} 0 0.05"))
        parts.append(_link(f"{side}_lidar", _inertial(0.01, 1e-6, 1e-6, 1e-6),
                           [_visual(_box_geom(0.02, 0.04, 0.05), TEAL)]))

        # Rear rack leg with pad
        pad_z = -H / 2 - C.GROUND_CLEARANCE + C.PAD_GAP + C.PAD_RADIUS
        parts.append(_joint(f"{side}_leg", "prismatic", f"{side}_back", f"{side}_leg",
                            f"{-C.BACK_LEN / 2 - C.PAD_BEHIND} 0 {pad_z}", axis="0 0 -1",
                            lower=0, upper=C.RACK_TRAVEL + C.PAD_GAP + 0.02))
        pad = f'<geometry><sphere radius="{C.PAD_RADIUS}"/></geometry>'
        parts.append(_link(f"{side}_leg", _inertial(C.M_LEG, 1e-4, 1e-4, 1e-4),
                           [_visual(pad, ORANGE), _visual(_box_geom(0.012, 0.025, 0.33), BLACK, xyz="0 0 0.165")],
                           [_collision(pad)]))

    # Cleaning unit (visual + mass only; cleaning is computed geometrically)
    parts.append(_joint("clean_rail", "prismatic", "frame", "clean_rail", "0 0 0", axis="0 1 0",
                        lower=-C.SPAN_MAX / 2, upper=C.SPAN_MAX / 2))
    parts.append(_link("clean_rail", _inertial(0.05, 1e-4, 1e-4, 1e-4),
                       [_visual(_box_geom(0.05, 0.06, 0.1), BLACK, xyz="0 0 0.05")]))
    parts.append(_joint("clean_reach", "prismatic", "clean_rail", "clean_reach", "0 0 0", axis="1 0 0",
                        lower=-C.REACH_MAX, upper=C.REACH_MAX))
    cs = C.CLEANER_SIZE
    parts.append(_link("clean_reach", _inertial(C.M_CLEANER, *_inertia_box(C.M_CLEANER, cs, cs, cs)),
                       [_visual(_box_geom(cs, cs, cs), GREEN, xyz=f"0 0 {-0.02}")]))
    head_z = -0.02 - cs / 2
    parts.append(_joint("clean_head", "prismatic", "clean_reach", "clean_head", f"0 0 {head_z}", axis="0 0 -1",
                        lower=0, upper=0.2))
    brushes = [_visual(f'<geometry><cylinder radius="0.02" length="0.015"/></geometry>', YELLOW,
                       xyz=f"{C.HEAD_DEPTH / 2 - 0.02} {dy} -0.02") for dy in (-0.05, 0, 0.05)]
    parts.append(_link("clean_head", _inertial(C.M_HEAD, 1e-3, 1e-3, 1e-3),
                       [_visual(_box_geom(C.HEAD_DEPTH, C.HEAD_WIDTH, 0.02), YELLOW, xyz="0 0 -0.01")] + brushes))
    parts.append("</robot>")
    return "\n".join(parts)


HEAD_BOTTOM_BELOW_BASE = 0.02 + C.CLEANER_SIZE / 2 + 0.02   # head bottom below frame origin at q=0


def write_urdf(path=None) -> str:
    if path is None:
        path = os.path.join(os.path.dirname(__file__), "_generated", "stair_climber.urdf")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(build_urdf())
    return path
