"""Shared geometry and load model for the scoop head calculations.

All material numbers here are ASSUMPTIONS unless noted. They exist so that the
sizing logic can be exercised end to end; every one of them must be replaced
by the measured values from experiments P0-3 and P1-1 (docs/17, docs/24).

Coordinate convention (docs/11_final_mechanism.md):
  x : stroke direction, horizontal
  z : vertical, up positive; the local ice cream surface is z = 0
  theta : scoop pitch about a horizontal axis through the bowl centre C,
          perpendicular to x. theta = 0 -> rim plane vertical, opening faces +x.
          theta > 0 tilts the opening upward.
  h : height of C above the local surface (set by the Z lock / touch-off).
"""

import math

# --- product / portion (Korean BR Single Regular = 115 g; research/products.md) ---
TARGET_MASS_G = 115.0
DENSITY_CASES_G_CM3 = (0.55, 0.65, 0.75)  # ASSUMPTION: overrun-dependent, measure
DEFAULT_DENSITY = 0.65                       # ASSUMPTION

# --- scoop geometry (ASSUMPTION until the store scoop is measured, task P0-4) ---
R_BOWL_CASES_MM = (30.0, 35.0, 40.0)
R_BOWL_MM = 35.0


def ellipse_segment_area(a, b, h):
    """Area of the part of an ellipse (semi-axes a horizontal, b vertical,
    centre at height h above a horizontal line) that lies below the line.
    Returns 0 if the ellipse does not reach the line."""
    if b <= 0 or h >= b:
        return 0.0
    if h <= -b:
        return math.pi * a * b
    t = h / b
    return a * b * (math.acos(t) - t * math.sqrt(1.0 - t * t))


def swept_area_mm2(theta_rad, h_mm, r_mm=R_BOWL_MM):
    """Cross-section of material cut per unit stroke length [mm^2].

    The rim circle, tilted by theta, projects onto the y-z plane as an ellipse
    with semi-axes r (lateral) and r*cos(theta) (vertical). Everything of that
    projection below the surface is swept by the translating rim."""
    return ellipse_segment_area(r_mm, r_mm * math.cos(theta_rad), h_mm)


def cut_depth_mm(theta_rad, h_mm, r_mm=R_BOWL_MM):
    """Depth of the lowest rim point below the surface [mm] (negative = clear)."""
    return r_mm * math.cos(theta_rad) - h_mm


def segment_centroid_below_centre_mm(theta_rad, h_mm, r_mm=R_BOWL_MM):
    """Vertical distance from C down to the centroid of the swept segment [mm].
    Used as the lever arm of the drag force about the pitch axis."""
    b = r_mm * math.cos(theta_rad)
    if h_mm >= b:
        return 0.0
    # circle-segment centroid, then scale vertically by cos(theta)
    t = h_mm / b
    alpha = 2.0 * math.acos(t)  # central angle of the segment on the unit-scaled circle
    area_unit = 0.5 * (alpha - math.sin(alpha))
    if area_unit <= 0:
        return 0.0
    ybar_unit = 4.0 * math.sin(alpha / 2.0) ** 3 / (3.0 * (alpha - math.sin(alpha)))
    return ybar_unit * b


def ball_volume_cm3(mass_g=TARGET_MASS_G, density=DEFAULT_DENSITY):
    return mass_g / density


# --- material load model -------------------------------------------------------
# F_x = u(T, product, edge) * A * (v / v_ref)^n
# u : specific cutting resistance [kPa = mN/mm^2] -- the single most important
#     unknown of the project. Four ASSUMPTION cases chosen so that a 20 mm deep
#     cut with a 35 mm bowl (A ~ 907 mm^2) gives ~35 / 80 / 145 / 225 N.
U_CASES_KPA = {"soft": 40.0, "medium": 90.0, "hard": 160.0, "very_hard": 250.0}
V_REF_MM_S = 80.0
RATE_EXPONENT_CASES = (0.05, 0.15, 0.30)  # ASSUMPTION: rate sensitivity n, measure (P1-1)
K_VERTICAL = 0.5  # ASSUMPTION: |F_z| / F_x for the drag phase


def drag_force_n(u_kpa, area_mm2, v_mm_s, n=0.15):
    v = max(v_mm_s, 1e-3)
    return u_kpa * 1e-3 * area_mm2 * (v / V_REF_MM_S) ** n
