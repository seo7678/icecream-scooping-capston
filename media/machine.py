"""Shared V1 geometry, colours and drawing helpers for the figures and the video.

All dimensions are mm in the machine frame M (docs/coordinate_and_flavor_mapping.md):
X_M along the beam (= drag direction), Y_M toward the back, Z_M up, deck = 0.
Z_M is always the height of the scoop pivot C. Numbers come from
calc/cartesian_model.py and the V1 design docs; they are ASSUMPTIONS, not
measurements.

Requires: matplotlib, numpy, koreanize-matplotlib (NanumGothic font).
"""

import math
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle, FancyBboxPatch  # noqa: E402

try:
    import koreanize_matplotlib  # noqa: F401  (registers NanumGothic)
except ImportError:  # pragma: no cover
    print("pip install koreanize-matplotlib  (Korean labels)")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "calc"))
import cartesian_model as cm  # noqa: E402

plt.rcParams["axes.unicode_minus"] = False
# Latin, arrows and math symbols from DejaVu Sans; Hangul falls back to NanumGothic
plt.rcParams["font.family"] = ["DejaVu Sans", "NanumGothic"]

# ------------------------------------------------------------------ geometry
R = cm.R_SCOOP                       # 35 bowl radius
B_ATT = R * math.cos(math.radians(cm.ATTACK_DEG))   # 30.3 rim depth below C at -30 deg
STEM = cm.STEM_LEN                   # 300 C -> head
HEAD = cm.HEAD_LEN                   # 100 head housing
COL_LEN = 555                        # Z column (steel tube 60x60x4)
COL_W = 60
LEVER = 25                           # parallelogram lever length
PUSH_OFF = LEVER
Z_SAFE = cm.z_safe(cm.CUP_RIM_Z_RECESSED)   # 60 (recessed cup)
Z_TOP = 160.0
Z_MIN = cm.z_min()                   # -205
DISPENSE_Z = cm.CUP_RIM_Z_RECESSED + R + 15  # 50

FLOOR = -850.0
DECK_T = 20.0
BEAM_Z = (760.0, 860.0)
BEAM_Y = (200.0, 250.0)
BEAM_X = (-200.0, 1350.0)
POSTS_X = [(-260.0, -180.0), (1370.0, 1450.0)]
ZBLOCKS = (580.0, 730.0)
ZPLATE = (555.0, 1175.0)             # Z drive plate on the Y slide (fixed height)
FREEZER_X = (200.0, 1000.0)
FREEZER_Y = (-250.0, 250.0)
TUB_D_TOP, TUB_D_BOT, TUB_DEPTH = cm.TUB_D_TOP, cm.TUB_D_BOTTOM, cm.TUB_DEPTH

RINSE_X = cm.LAYOUT_V1["RINSE"][0]   # 120
CUP_X = cm.LAYOUT_V1["CUP"][0]       # 1120
CUP_TOP_D, CUP_BOT_D, CUP_H = 80.0, 60.0, 75.0

# flavour, name, tub x, surface z (example map, firmware/coordinate_map_example.md), colour
FLAVORS = [
    ("VAN", "바닐라", cm.LAYOUT_V1["TUB1"][0], -62.0, "#EFDDA6"),
    ("CHO", "초콜릿", cm.LAYOUT_V1["TUB2"][0], -118.0, "#6B4226"),
    ("PNB", "땅콩버터", cm.LAYOUT_V1["TUB3"][0], -214.0, "#C9955B"),
]

# ------------------------------------------------------------------ colours
C_STEEL = "#8C96A0"
C_STEEL_D = "#5E6770"
C_ALU = "#B9C2CA"
C_MOVE = "#2F6FB0"        # moving axes
C_MOVE_L = "#9CC0E6"
C_HEAD = "#3C4B5C"
C_FOOD = "#D98E04"        # food-zone stainless highlight
C_SENSOR = "#D1343A"
C_SAFE = "#E0B000"
C_FREEZER = "#E9EEF3"
C_DECK = "#C9D1D8"
C_WATER = "#BFE3F2"
C_CUP = "#F7F7F2"
C_TEXT = "#1E2328"
C_GRID = "#DADFE4"


def tub_half_width(z):
    """Inner half width of the tub at height z (z <= 0)."""
    return cm.tub_diameter_at(-z) / 2.0


# ------------------------------------------------------------------ scoop geometry (X-Z section)
def bowl_polygon(cx, cz, th_deg, r=R, n=60):
    """Closed polygon of the bowl half-disk (region enclosed by shell + rim chord)."""
    th = math.radians(th_deg)
    a = np.linspace(th + math.pi / 2, th + 3 * math.pi / 2, n)
    return np.column_stack([cx + r * np.cos(a), cz + r * np.sin(a)])


def rim_ends(cx, cz, th_deg, r=R):
    th = math.radians(th_deg)
    t = np.array([-math.sin(th), math.cos(th)])
    c = np.array([cx, cz])
    return c + r * t, c - r * t          # (upper/front end at attack pose, lower end)


def lower_envelope(poly, xs):
    """Lowest z of a closed polygon above each x in xs (nan where the polygon is absent)."""
    out = np.full(xs.shape, np.nan)
    p = np.vstack([poly, poly[:1]])
    for (x1, z1), (x2, z2) in zip(p[:-1], p[1:]):
        lo, hi = min(x1, x2), max(x1, x2)
        m = (xs >= lo) & (xs <= hi)
        if not m.any():
            continue
        if hi - lo < 1e-9:
            z = np.full(m.sum(), min(z1, z2))
        else:
            z = z1 + (z2 - z1) * (xs[m] - x1) / (x2 - x1)
        out[m] = np.fmin(out[m], z)
    return out


def lever_tip(cx, cz, th_deg):
    """Lower lever tip (fixed to the scoop). Neutral (horizontal, pointing -X) at theta = +30."""
    phi = math.radians(180.0 + (th_deg - 30.0))
    return cx + LEVER * math.cos(phi), cz + LEVER * math.sin(phi)


# ------------------------------------------------------------------ drawing
def draw_scoop(ax, cx, cz, th, fill=0.0, ball=False, color=C_FOOD, ice="#EFDDA6", lw=2.2, z=6):
    """Scoop bowl (section), rim, pivot, optional chip fill / captured ball."""
    poly = bowl_polygon(cx, cz, th)
    if ball:
        ax.add_patch(Circle((cx, cz), R - 1.5, fc=ice, ec="#8a7a55", lw=0.6, zorder=z))
    elif fill > 0:
        # chip curled against the shell: annulus between the shell and r_in
        r_in = R * math.sqrt(max(0.0, 1.0 - min(1.0, fill)))
        outer = bowl_polygon(cx, cz, th, R - 1.2)
        inner = bowl_polygon(cx, cz, th, r_in)[::-1] if r_in > 0.5 else np.array([[cx, cz]])
        ax.add_patch(Polygon(np.vstack([outer, inner]), closed=True, fc=ice, ec="#9c8550", lw=0.5, zorder=z))
    ax.plot(poly[:, 0], poly[:, 1], color=color, lw=lw, solid_capstyle="round", zorder=z + 1)
    return poly


def draw_head(ax, X, Z, th, fill=0.0, ball=False, ice="#EFDDA6", show_col=True, detail=True, z=5):
    """Food module + head + Z column at pivot position (X, Z) and pitch th."""
    top = Z + STEM
    # stem (fork down to the pivot pin)
    ax.add_patch(Rectangle((X - 7.5, Z + 4), 15, STEM - 4, fc="#D7DCE1", ec=C_FOOD, lw=1.0, zorder=z))
    # push-rod: from the lower lever tip, parallel to the stem, to the upper lever
    lx, lz = lever_tip(X, Z, th)
    ax.plot([X, lx], [Z, lz], color=C_FOOD, lw=2.4, zorder=z + 2)
    ax.plot([X, lx], [top, lz + STEM], color=C_HEAD, lw=2.4, zorder=z + 2)
    ax.plot([lx, lx], [lz, lz + STEM], color=C_FOOD, lw=1.3, zorder=z + 1)
    for px, pz in ((lx, lz), (lx, lz + STEM)):
        ax.add_patch(Circle((px, pz), 2.8, fc="white", ec=C_HEAD, lw=0.8, zorder=z + 3))
    # drip umbrella + head housing
    ax.add_patch(Rectangle((X - 85, top - 4), 170, 5, fc=C_FOOD, ec="none", alpha=0.9, zorder=z + 1))
    ax.add_patch(FancyBboxPatch((X - 60, top + 2), 120, HEAD - 2, boxstyle="round,pad=0,rounding_size=6",
                                fc=C_HEAD, ec="none", zorder=z + 1))
    if detail:
        ax.add_patch(Circle((X + 22, top + 55), 24, fc="#6C7E92", ec="#26313d", lw=0.8, zorder=z + 2))
        ax.add_patch(Circle((X + 22, top + 55), 6, fc="#26313d", zorder=z + 3))
    draw_scoop(ax, X, Z, th, fill=fill, ball=ball, ice=ice, z=z + 3)
    ax.add_patch(Circle((X, Z), 3.2, fc="white", ec=C_HEAD, lw=0.9, zorder=z + 6))
    if show_col:
        cb = top + HEAD
        ax.add_patch(Rectangle((X - COL_W / 2, cb), COL_W, COL_LEN, fc=C_STEEL, ec=C_STEEL_D, lw=0.8, zorder=z - 1))


def draw_frame_static(ax, surfaces=None, tub_profiles=None, show_cup=True, cup_fill=None, labels=False):
    """Everything that does not move (front elevation, looking along +Y)."""
    # floor
    ax.add_patch(Rectangle((-400, FLOOR - 40), 2000, 40, fc="#E3E6E9", ec="none", zorder=0))
    # posts + braces
    for x0, x1 in POSTS_X:
        ax.add_patch(Rectangle((x0, FLOOR), x1 - x0, BEAM_Z[1] + 20 - FLOOR, fc=C_ALU, ec="#98A3AD", lw=0.8, zorder=1))
    ax.plot([POSTS_X[0][1], POSTS_X[0][1] + 220], [BEAM_Z[0] - 250, BEAM_Z[0]], color=C_ALU, lw=4, zorder=1)
    ax.plot([POSTS_X[1][0], POSTS_X[1][0] - 220], [BEAM_Z[0] - 250, BEAM_Z[0]], color=C_ALU, lw=4, zorder=1)
    # X beam, rails, screw, motor
    ax.add_patch(Rectangle((BEAM_X[0], BEAM_Z[0]), BEAM_X[1] - BEAM_X[0], BEAM_Z[1] - BEAM_Z[0],
                           fc=C_STEEL, ec=C_STEEL_D, lw=1.0, zorder=2))
    for zr in (BEAM_Z[0] + 12, BEAM_Z[1] - 12):
        ax.plot([BEAM_X[0] + 30, BEAM_X[1] - 60], [zr, zr], color="#44505b", lw=2.2, zorder=3)
    ax.plot([BEAM_X[0] + 40, BEAM_X[1] - 60], [810, 810], color="#707b86", lw=3.2, zorder=3,
            dashes=(3, 1.2))
    ax.add_patch(Rectangle((BEAM_X[1] - 60, 772), 90, 76, fc="#2E3A46", ec="none", zorder=3))
    # freezer (cut away) + deck
    ax.add_patch(Rectangle((FREEZER_X[0], FLOOR), FREEZER_X[1] - FREEZER_X[0], -DECK_T - FLOOR,
                           fc=C_FREEZER, ec="#AAB5BF", lw=1.0, zorder=1))
    ax.add_patch(Rectangle((FREEZER_X[0] - 20, -DECK_T), FREEZER_X[1] - FREEZER_X[0] + 40, DECK_T,
                           fc=C_DECK, ec="#8E9AA5", lw=0.8, zorder=3))
    # tubs (section) with ice cream
    for i, (fid, name, tx, s, col) in enumerate(FLAVORS):
        top_hw, bot_hw = TUB_D_TOP / 2, TUB_D_BOT / 2
        ax.add_patch(Polygon([(tx - top_hw - 3, 0), (tx + top_hw + 3, 0), (tx + bot_hw + 3, -TUB_DEPTH - 3),
                              (tx - bot_hw - 3, -TUB_DEPTH - 3)], closed=True, fc="#F4F1EA", ec="#9A8F7A",
                             lw=0.9, zorder=2))
        if tub_profiles is not None and fid in tub_profiles:
            xs, zs = tub_profiles[fid]
            pts = [(tx - bot_hw, -TUB_DEPTH)] + list(zip(xs, zs)) + [(tx + bot_hw, -TUB_DEPTH)]
        else:
            ss = surfaces.get(fid, s) if surfaces else s
            pts = [(tx - bot_hw, -TUB_DEPTH), (tx - tub_half_width(ss), ss), (tx + tub_half_width(ss), ss),
                   (tx + bot_hw, -TUB_DEPTH)]
        ax.add_patch(Polygon(pts, closed=True, fc=col, ec="none", zorder=2.5))
        if labels:
            ax.text(tx, -TUB_DEPTH - 22, f"TUB{i + 1}\n{name}", ha="center", va="top", fontsize=8, color=C_TEXT,
                    linespacing=1.1)
    # rinse station
    ax.add_patch(Rectangle((RINSE_X - 30, FLOOR), 60, FLOOR * -1 - 160, fc="#D9DEE3", ec="none", zorder=1))
    ax.add_patch(Rectangle((RINSE_X - 80, -160), 160, 160, fc="#DCE3E8", ec="#8E9AA5", lw=0.9, zorder=2))
    ax.add_patch(Rectangle((RINSE_X - 74, -154), 148, 124, fc=C_WATER, ec="none", zorder=2.5))
    # cup station: stand, recessed well, scale, locator ring
    ax.add_patch(Rectangle((CUP_X - 30, FLOOR), 60, -FLOOR - 140, fc="#D9DEE3", ec="none", zorder=1))
    ax.add_patch(Rectangle((CUP_X - 95, -140), 190, 140, fc="#DCE3E8", ec="#8E9AA5", lw=0.9, zorder=2))
    ax.add_patch(Rectangle((CUP_X - 60, -118), 120, 16, fc="#F1C4C6", ec=C_SENSOR, lw=0.9, zorder=3))
    if show_cup:
        ax.add_patch(Polygon([(CUP_X - CUP_TOP_D / 2, 0), (CUP_X + CUP_TOP_D / 2, 0),
                              (CUP_X + CUP_BOT_D / 2, -CUP_H), (CUP_X - CUP_BOT_D / 2, -CUP_H)], closed=True,
                             fc=C_CUP, ec="#9aa3ab", lw=0.9, zorder=3.5))
    # eject comb (hook strip) at the cup station, +X side
    ax.plot([CUP_X + 55, CUP_X + 55, CUP_X + 36], [0, 34, 34], color=C_FOOD, lw=2.2, zorder=4,
            solid_capstyle="round")
    # enclosure (dashed) + Z_SAFE line
    ax.plot([-320, -320, 1500, 1500], [0, 1220, 1220, 0], color="#9FB3C8", lw=1.0, ls=(0, (5, 4)), zorder=0.5)


def draw_carriage(ax, X, Z, th, fill=0.0, ball=False, ice="#EFDDA6", detail=True):
    """X carriage, Y slide, Z drive plate and blocks at X; Z column + head at pivot height Z."""
    # X carriage plate on the beam
    ax.add_patch(Rectangle((X - 100, 735), 200, 150, fc=C_MOVE, ec="#1E4E80", lw=1.0, zorder=3.4))
    # Z drive plate (on the Y slide), blocks, Z screw + motor
    ax.add_patch(Rectangle((X - 70, ZPLATE[0]), 140, ZPLATE[1] - ZPLATE[0], fc=C_MOVE_L, ec=C_MOVE, lw=0.9, zorder=3.6))
    ax.plot([X + 50, X + 50], [ZPLATE[0] + 15, ZPLATE[1] - 45], color="#5B6B7B", lw=2.4, zorder=4.4, dashes=(2.5, 1))
    ax.add_patch(Rectangle((X + 30, ZPLATE[1] - 45), 42, 45, fc="#2E3A46", ec="none", zorder=4.5))
    draw_head(ax, X, Z, th, fill=fill, ball=ball, ice=ice, detail=detail, z=5)
    for zb in ZBLOCKS:
        ax.add_patch(Rectangle((X - 38, zb - 12), 76, 24, fc="#26313D", ec="none", zorder=7))
    # Z nut + load cell (moves with the column)
    col_bottom = Z + STEM + HEAD
    ax.add_patch(Rectangle((X + 36, col_bottom + COL_LEN - 70), 28, 22, fc=C_SENSOR, ec="none", zorder=7))
    # X nut load cell
    ax.add_patch(Rectangle((X - 14, 800), 28, 20, fc=C_SENSOR, ec="none", zorder=7))
