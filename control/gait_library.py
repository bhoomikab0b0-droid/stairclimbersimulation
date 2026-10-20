"""
Gait library (patent: "Step Detection & Gait Library — algorithms match
detected stair geometry to an optimized, stable step negotiation gait").

Each profile sets how much clearance the racks add above the riser and how
fast the wheels/racks move. Steps the hardware cannot handle are rejected
with a reason, which sends the robot into its safe fallback state.
"""
from dataclasses import dataclass

import config as C


@dataclass(frozen=True)
class GaitProfile:
    name: str
    max_height: float     # use this profile for risers up to this height
    lift_margin: float    # extra rack lift above the riser
    drive_speed: float    # m/s
    lift_speed: float     # m/s rack speed


GAITS = (
    GaitProfile("LOW", 0.13, 0.020, 0.09, 0.07),
    GaitProfile("STANDARD", 0.185, 0.025, 0.08, 0.06),
    GaitProfile("HIGH", C.RACK_TRAVEL - 0.03, 0.030, 0.05, 0.04),
)


def select_gait(height, depth, width):
    """Returns (profile, None) or (None, reason)."""
    if width is not None:
        span_needed = width - C.UNIT_WIDTH - 2 * C.WALL_CLEARANCE
        if span_needed < C.SPAN_MIN:
            return None, f"stairs too narrow ({width:.2f} m) for the minimum frame width"
        if span_needed > C.SPAN_MAX:
            return None, f"stairs too wide ({width:.2f} m) for the maximum frame width"
    if height is None:
        return None, "no step geometry"
    if depth is not None and depth < C.MIN_TREAD:
        return None, f"tread too shallow ({depth * 1000:.0f} mm < {C.MIN_TREAD * 1000:.0f} mm)"
    for g in GAITS:
        if height <= g.max_height:
            return g, None
    return None, f"riser too high ({height * 1000:.0f} mm > {GAITS[-1].max_height * 1000:.0f} mm rack limit)"
