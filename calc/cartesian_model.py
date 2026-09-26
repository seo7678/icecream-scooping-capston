"""Shared layout, geometry and load assumptions for the Cartesian architecture.

Every number here is an ASSUMPTION unless its comment cites a source. The
Cartesian scripts (gantry_motor_sizing, z_axis_sizing, cycle_time_estimate,
scoop_load_path, tub_lane_planner) import this file so a measured value only
has to be changed once.

Coordinate conventions (docs/coordinate_and_flavor_mapping.md)
  Machine frame M : origin at home corner, X_M along the bridge beam (= drag
                    direction), Y_M = bridge travel, Z_M up.
  Z_M             : height of the scoop pivot C above the deck plane (tub rims).
                    Every point of the scoop is within R of C, so the lowest
                    point of the scoop at any pitch is Z_M - R.
  theta_s         : scoop pitch about Y through C. theta_s = 0 -> opening faces
                    +X_M. theta_s < 0 -> opening faces forward-DOWN (attack pose).
                    theta_s > 0 -> opening turning up (capture).
"""

import math

# ---------------------------------------------------------------- scoop ---
R_SCOOP = 35.0            # mm, bowl radius (A06, measure the store scoop)
ATTACK_DEG = 30.0         # deg, opening forward-down during dive/drag (ASSUMPTION)
CLOSE_DEG = 90.0          # deg, final capture pitch
Z_C_LEVER = 22.0          # mm, drag-force centroid below C (calc/scoop_model)
STEM_LEN = 300.0          # mm, pivot C -> head clamp (food module)
HEAD_LEN = 100.0          # mm, head housing (clamp, Z load cell, theta drive)

# ---------------------------------------------------------------- tubs ----
TUB_D_TOP = 230.0         # mm inner diameter at rim (A10)
TUB_D_BOTTOM = 210.0      # mm inner diameter at bottom (taper, ASSUMPTION)
TUB_DEPTH = 250.0         # mm (A10)
WALL_MARGIN = 10.0        # mm clearance scoop <-> tub wall
FLOOR_MARGIN = 10.0       # mm clearance scoop <-> tub floor

# ---------------------------------------------------------------- loads ---
F_X_CASES = (50.0, 100.0, 150.0, 200.0)   # N drag force (A01, NOT measured)
K_SIDE = 0.3              # F_y / F_x side force from asymmetric cuts (ASSUMPTION)
K_VERT = 0.5              # |F_z| / F_x (A04)

# ---------------------------------------------------------------- layout --
# V1: one row of 3 tubs along X in a chest freezer, rinse + cup on the same line.
# Product vision: 2 rows x 4 tubs. Coordinates are tub/station centres [mm].
LAYOUT_V1 = {
    "HOME": (0.0, 0.0),
    "RINSE": (120.0, 0.0),
    "TUB1": (340.0, 0.0),
    "TUB2": (600.0, 0.0),
    "TUB3": (860.0, 0.0),
    "CUP": (1120.0, 0.0),
}
LAYOUT_PRODUCT = {
    "HOME": (0.0, 0.0),
    "RINSE": (1400.0, 80.0),
    "TUB1": (340.0, 160.0), "TUB2": (600.0, 160.0), "TUB3": (860.0, 160.0), "TUB4": (1120.0, 160.0),
    "TUB5": (340.0, 440.0), "TUB6": (600.0, 440.0), "TUB7": (860.0, 440.0), "TUB8": (1120.0, 440.0),
    "CUP": (1400.0, 300.0),
}

# ---------------------------------------------------------------- heights -
CUP_RIM_Z_RAISED = 80.0   # mm, cup standing on a scale at deck level
CUP_RIM_Z_RECESSED = 0.0  # mm, cup station recessed so the cup rim is flush with the deck
Z_MARGIN = 25.0           # mm vertical clearance above obstacles


def z_safe(cup_rim_z):
    """Lowest pivot height at which XY travel is allowed."""
    return max(0.0, cup_rim_z) + R_SCOOP + Z_MARGIN


def z_min():
    """Deepest pivot height (scoop just above the tub floor)."""
    return -TUB_DEPTH + FLOOR_MARGIN + R_SCOOP


def tub_diameter_at(depth_below_rim):
    """Inner diameter of a linearly tapered tub at a given depth below the rim."""
    f = min(1.0, max(0.0, depth_below_rim / TUB_DEPTH))
    return TUB_D_TOP + (TUB_D_BOTTOM - TUB_D_TOP) * f


# ---------------------------------------------------------------- masses --
MASS = {                   # kg, ASSUMPTION (update from CAD)
    "food_module": 0.8,
    "head": 2.0,           # theta gearmotor, clamp, Z load cell, housing
    "z_column": 4.5,       # moving Z member (profile/steel tube) + rails
    "z_drive": 1.5,        # Z motor + screw + brake (rides on X carriage)
    "x_carriage": 3.0,     # plate + blocks + Z guide blocks
    "bridge": 9.0,         # X beam + X rails + X screw + end plates (moves in Y)
}


def moving_mass(axis):
    z_mass = MASS["food_module"] + MASS["head"] + MASS["z_column"]
    x_mass = z_mass + MASS["z_drive"] + MASS["x_carriage"]
    y_mass = x_mass + MASS["bridge"]
    return {"Z": z_mass, "X": x_mass, "Y": y_mass}[axis]


def ellipse_segment_area(a, b, h):
    if b <= 0 or h >= b:
        return 0.0
    t = h / b
    return a * b * (math.acos(t) - t * math.sqrt(1.0 - t * t))


def swept_area(theta_deg, depth, r=R_SCOOP):
    """Cut cross-section [mm^2] for a lowest-rim depth `depth` at pitch theta."""
    b = r * math.cos(math.radians(theta_deg))
    h = b - depth
    return ellipse_segment_area(r, b, h)
