#!/usr/bin/env python3
"""V1-L (budget <= 1,000,000 KRW) mechanical sizing — moving-bed vs moving-gantry vs kit.

Run:  python3 _chief/work/mech/v1l_mech_calc.py
Out:  stdout (markdown) + _chief/work/mech/v1l_mech_calc_output.md

What it answers (all loads, stiffnesses and prices are ASSUMPTIONS unless a
source tag says otherwise; see data/assumptions.md A01-A47 and the SOURCES
block below):
  1. Load levels for the 360x165x120 gelato pan (A40) and proposed stop / jam limits
  2. Scoop-tip compliance of L-A (moving bed), L-A-lite, L-B (moving gantry), L-C (kits)
     - the repo stiffness model (calc/scoop_load_path.py) is re-implemented here
       and checked against the published V1-S numbers first
  3. X drive: TR8 lead 2/4/8, NEMA17 torque-speed (ASSUMED curve), self-locking,
     nut pressure, critical speed, bed inertia error on the force platform
  4. Z drive: stroke, self-locking of TR8 lead 2/4/8, power-off behaviour
  5. Theta drive: required torque, NEMA17 + 1:27 planetary rating, alternatives
  6. Strength at the jam load (drive stall force)
  7. Frame cut list, laser-cut / 3D-printed parts, rough parts list with price ranges

Why not `import scoop_load_path` / `capstone_alternatives`: both write their
output files at import time (they would touch repo files). Their formulas are
copied here (with the source named) and the copy is verified against
calc/output/capstone_alternatives.md §2 before use.
Only side-effect-free modules are imported: cartesian_model, scoop_model,
tub_lane_planner (its main() is guarded).
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "calc"))

import cartesian_model as cm      # noqa: E402  (constants + geometry, no file output)
import scoop_model as sm          # noqa: E402  (no file output)
import tub_lane_planner as tlp    # noqa: E402  (main() guarded)

OUT = os.path.join(HERE, "v1l_mech_calc_output.md")
lines = []
p = lines.append


def table(h, rows):
    p("| " + " | ".join(h) + " |")
    p("|" + "|".join("---" for _ in h) + "|")
    for r in rows:
        p("| " + " | ".join(str(c) for c in r) + " |")
    p("")


def f1(x):
    return f"{x:.1f}"


def f2(x):
    return f"{x:.2f}"


def f3(x):
    return f"{x:.3f}"


# =============================================================================
# SOURCES (tags as in research/components.md)
#  [S1] Misumi HFS5-2040: Ix 1.358e4, Iy 5.13e4 mm^4, 0.88 kg/m        [SNIPPET]
#  [S2] TNUTZ EXM-2020: I 0.67 cm^4, A 1.638 cm^2, 0.454 kg/m            [SNIPPET]
#  [S3] Misumi HFS8-4040 10.4 cm^4; HFS8-4080 19.8 / 71.9 cm^4;
#       HFS8-8080 129.1 cm^4 (repo research/components.md)               [SNIPPET]
#  [S4] StepperOnline 17HS19-1684S-PG27: ratio 26.85:1, max permissible
#       torque 3 N·m, moment (momentary) permissible torque 5 N·m        [SNIPPET]
#  [S5] StepperOnline 17HS19-2004S1: holding torque 59 N·cm, 2 A         [SNIPPET]
#  [S6] Shigley's MED (Budynas & Nisbett), §8-2 power screws: T_R, T_L,
#       self-locking f·secα > tanλ, efficiency e = F·l/(2π·T_R);
#       Table 8-4 steel/bronze bearing pressure: 17-24 MPa low speed,
#       11-17 MPa <= 10 fpm, 5.5-9.7 MPa at 20-40 fpm, far lower beyond  [SNIPPET]
#  [S7] THK A15-32 critical speed N1 = λ2·d_r/L²·1e7, λ2 incl. 0.8 factor
#       (repo research/components.md)                                     [SNIPPET]
#  [S8] Gates GT2 6 mm working tension ~28 N (repo research/components.md) [MULTI]
#  [S9] MGN12H C0 5.88 kN, M0P 36 N·m (repo research/components.md)       [SNIPPET]
#  [S10] S-type load cells: deflection at Emax 0.04-0.14 mm (generic)    [SNIPPET]
#  [S11] Genmitsu CNC 3018-PRO: 300x180x45 mm work area, T8 lead 4,
#        X guide rods Ø10x360, NEMA17                                     [SNIPPET]
#  [S12] Lead-screw friction coefficient 0.1-0.3 by material/lube (pbclinear,
#        firgelli pages)                                                  [SNIPPET]
#  [S13] NEMA17 worm gearbox backlash 15-18 arcmin, self-locking only
#        "under specific loads and lubrication"                          [SNIPPET]
# =============================================================================

# ---------------------------------------------------------------- materials
E_AL, G_AL = 69e3, 26e3          # MPa (same as calc/scoop_load_path.py)
E_ST = 200e3                     # MPa, steel screw
E_SS = 193e3                     # MPa, 304 (repo)
SY_304 = 205.0                   # MPa, 304 annealed minimum yield (ASTM A240 min)
SY_AL = 110.0                    # MPa, 6063-T5 extrusion yield — ASSUMPTION (lower-bound typical)
RHO_ST = 7850.0                  # kg/m^3

# ---------------------------------------------------------------- sections (mm^4, mm^2)
PROF = {
    "2020": {"I_w": 0.67e4, "I_s": 0.67e4, "A": 163.8, "J": 0.25e4, "kg_m": 0.454},   # [S2]; J ASSUMPTION
    "2040": {"I_w": 1.358e4, "I_s": 5.13e4, "A": 326.0, "J": 0.8e4, "kg_m": 0.88},    # [S1]; A from mass/2.70; J ASSUMPTION
    "4040": {"I_w": 10.4e4, "I_s": 10.4e4, "A": 640.0, "J": 2.6e4, "kg_m": 1.73},     # [S3]; J Bosch 40x40 2.6 cm^4 (repo)
    "4080": {"I_w": 19.8e4, "I_s": 71.9e4, "A": 1109.0, "J": 6.0e4, "kg_m": 3.0},     # [S3]; J ASSUMPTION (repo), kg/m from A
    "8080": {"I_w": 129.1e4, "I_s": 129.1e4, "A": 1691.0, "J": 40.0e4, "kg_m": 4.57}, # [S3]; J ASSUMPTION (repo)
}


def round_tube_I(do, t):
    di = do - 2 * t
    return math.pi * (do ** 4 - di ** 4) / 64.0


STEM_I = {"30x3": round_tube_I(30, 3), "25x2": round_tube_I(25, 2), "25x3": round_tube_I(25, 3)}
STEM_C = {"30x3": 15.0, "25x2": 12.5, "25x3": 12.5}

# ---------------------------------------------------------------- guides / drives
K_MGN12 = 60e3        # N/mm per block — ASSUMPTION (repo: catalogue value NOT FOUND)
K_MGN9 = 30e3         # N/mm per block — ASSUMPTION
K_HG15 = 200e3        # N/mm per block — ASSUMPTION (repo 'M' config)
TR8_D, TR8_P = 8.0, 2.0            # mm; all common T8 leads (2/4/8) use a 2 mm pitch with 1/2/4 starts
TR8_DM = TR8_D - TR8_P / 2.0       # 7.0 mm mean (pitch) diameter
TR8_DR = TR8_D - TR8_P             # 6.0 mm root diameter — ASSUMPTION (≈ d − P)
ALPHA_THREAD = math.radians(15.0)  # trapezoidal 30° thread → half angle 15°
MU_CASES = (0.10, 0.15, 0.20)      # brass nut on steel — ASSUMPTION inside 0.1-0.3 [S12]
MU_NOM = 0.15                      # ASSUMPTION
K_NUT = 10e3          # N/mm, brass T8 nut axial — ASSUMPTION
K_SUPPORT = 5e3       # N/mm, KFL08 pillow block + collars, axial — ASSUMPTION
NUT_LEN = 15.0        # mm engaged length of a T8 flange nut — ASSUMPTION
BACKLASH_NUT = 0.10   # mm, plain T8 brass nut — ASSUMPTION

# stepper (17HS19-2004S1 class, holding 0.59 N·m [S5])
T_LO = 0.40           # N·m usable pull-out torque below the corner speed @24 V — ASSUMPTION
N_CORNER = 400.0      # rpm, above it torque ~ 1/speed (constant power) — ASSUMPTION
RPM_CAP = 1500.0      # rpm practical upper limit (mid-band resonance, step rate) — ASSUMPTION
J_ROTOR = 8.2e-6      # kg·m^2 — ASSUMPTION (NEMA17 48 mm class)
T_DETENT = 0.01       # N·m unpowered detent torque — ASSUMPTION
SF_STEPPER = 2.0      # required torque margin for an open-loop stepper — design rule (ASSUMPTION)


def stepper_torque(n_rpm):
    """ASSUMED 24 V pull-out curve: flat T_LO up to N_CORNER, then constant power."""
    n = max(n_rpm, 1.0)
    return T_LO if n <= N_CORNER else T_LO * N_CORNER / n


# theta gearmotor (17HS19-1684S-PG27 class)
PG_RATIO = 26.85      # [S4]
PG_T_CONT = 3.0       # N·m max permissible (continuous) [S4]
PG_T_PEAK = 5.0       # N·m moment (momentary) permissible [S4]
PG_ETA = 0.80         # 2-stage planetary efficiency — ASSUMPTION
PG_T_LO = 0.35        # N·m motor-side usable torque (44 N·cm holding class) — ASSUMPTION
PG_BACKLASH_DEG = 1.0 # deg — ASSUMPTION (MG series spec 30 arcmin; economy PG ~1°)

# force platform
SB_CAP_KG = 50.0      # S-beam (X) capacity recommended for V1-L (jam protection) — design choice
SB_DEFL = 0.14        # mm at rated load — conservative end of 0.04-0.14 mm [S10]
SP_CAP_KG = 30.0      # single-point cell (vertical) — design choice
SP_DEFL = 0.30        # mm at rated load — ASSUMPTION
K_SB = SB_CAP_KG * 9.81 / SB_DEFL
K_SP = SP_CAP_KG * 9.81 / SP_DEFL

# bolted joint rotational stiffness (head bracket-column, stem clamp) — ASSUMPTION range
K_JOINT = {"soft": 2e7, "mid": 5e7, "stiff": 1e8}   # N·mm/rad

# ---------------------------------------------------------------- loads
U_CASES = (40.0, 90.0, 160.0)     # kPa A02
K_SIDE, K_VERT = cm.K_SIDE, cm.K_VERT   # 0.3, 0.5 (A28)
A_DRAG = 0.2          # m/s^2 drag start/stop ramp — ASSUMPTION (firmware setting)
A_TRAVEL = 0.5        # m/s^2 travel accel of the bed — ASSUMPTION (firmware setting)
F_SEAL_PER_BLOCK = 1.0   # N wiper/seal drag per block — ASSUMPTION
MU_RAIL = 0.01        # rolling friction — ASSUMPTION (repo)

# ---------------------------------------------------------------- geometry (A40, A41)
PAN = {"L": 360.0, "W": 165.0, "H": 120.0}   # A40
R = cm.R_SCOOP                               # 35 (A06)
ATT = math.radians(-cm.ATTACK_DEG)           # -30° (A25)
STEM = 200.0                                 # A41
HEAD = cm.HEAD_LEN                           # 100
Z_SAFE = cm.z_safe(cm.CUP_RIM_Z_RECESSED)    # 60: cup rim flush with pan rim (A31)
Z_TOP = 100.0                                # A41
C_MIN = -PAN["H"] + cm.FLOOR_MARGIN + R      # -75
LANE = PAN["L"] - 2 * (R + cm.WALL_MARGIN)   # 270
Y_LANES = (-38.0, 0.0, 38.0)                 # manual index positions

# stack heights above the table (L-A) — ASSUMPTION (design layout)
STACK = {"base 2040 on edge": 40.0, "MGN12 rail+block": 13.0, "bed frame 2040 flat": 20.0,
         "deck plywood 6": 6.0, "force platform": 60.0, "holder floor (XPS 50 + ply 6)": 56.0,
         "pan": PAN["H"]}
H_RIM = sum(STACK.values())                  # pan rim above table
BED_BLOCK_Z = 40.0 + 6.5                     # bed block centre above table

# L-A bridge
Z_L = Z_TOP + STEM + HEAD + 20.0             # 420: lower Z block / lower cross-beam (A41 H_BLOCK)
H_B = 220.0                                  # cross-beam spacing = Z block spacing — design choice
Z_U = Z_L + H_B
S_CB = 480.0                                 # cross-beam span between towers (bed 400 wide + clearance)
LEG_FOOT = 450.0                             # tower leg foot offset in X from the bridge — design choice
S_BED_BLOCKS = 280.0                         # bed block spacing along X — design choice (fits 800 rails, see §3)
HOLDER_WALL = 56.0                           # XPS 50 + plywood 6 — ASSUMPTION (same as floor)
CUP_R = 45.0                                 # cup outer radius (Ø90 cup) — ASSUMPTION
MGN12H_BLOCK_L = 45.4                        # mm block length — ASSUMPTION (HIWIN MGN12H class)
RAIL_BED = 800.0
X_SCREW_L = 600.0                            # bed screw length
X_SCREW_NUT_MAX = 550.0                      # worst nut-to-fixed-bearing distance
Z_SCREW_L = 300.0

# masses [kg] — ASSUMPTION unless derived
M_ICE = PAN["L"] * PAN["W"] * PAN["H"] * 1e-9 * 0.8 * 650.0     # 80 % full, ρ 0.65 (A07) — derived
MASS_BED = {"ice cream (80 % full)": M_ICE, "pan": 1.5, "insulated holder": 1.0,
            "platform plates + float rails + cells": 1.8, "bed frame + deck": 2.6,
            "blocks + nut + cup station": 0.8}
M_TOP = M_ICE + 1.5 + 1.0 + 1.0          # mass carried by the S-beam (pan, holder, top plate) — ASSUMPTION
M_BED = sum(MASS_BED.values())
MASS_Z = {"4080 column 0.5 m": 0.5 * PROF["4080"]["kg_m"], "MGN12 rail 450": 0.3,
          "head bracket + clamps + plates": 0.8, "theta PG27 motor": 0.65,
          "food module (stem, scoop, rods)": 0.6, "Z motor + screw on column": 0.5}
M_Z = sum(MASS_Z.values())
W_Z = M_Z * 9.81

# ============================================================================= 1. loads
p("# V1-L 기계 계산 — generated")
p("")
p("> `python3 _chief/work/mech/v1l_mech_calc.py`. **모든 하중·강성·마찰·가격은 ASSUMPTION** (출처 태그 붙은 값 제외). "
  "기존 강성 모델(`calc/scoop_load_path.py`)을 옮겨 쓰고, 먼저 V1-S 공개 결과로 재현을 확인한다.")
p("")
p("## 1. 하중 — 젤라토 팬 360×165×120, 레인 270 mm")
p("")


def depth_for_portion(travel):
    for d10 in range(20, 300):
        d = d10 / 10.0
        v, a = tlp.stroke_volume(d, travel)
        if v >= tlp.V_TARGET:
            return d, a
    return None, None


D_PORTION, A_PORTION = depth_for_portion(LANE)
H_C = R * math.cos(ATT) - D_PORTION                     # C above the local surface
Z_CENT = sm.segment_centroid_below_centre_mm(ATT, H_C)  # force centroid below C (drag lever)
rows = []
for u in U_CASES:
    F = u * 1e-3 * A_PORTION
    rows.append([f"{u:.0f}", f1(D_PORTION), f"{A_PORTION:.0f}", f"{F:.0f}", f"{K_VERT*F:.0f}", f"{K_SIDE*F:.0f}",
                 f2(F * Z_CENT / 1e3), f2(F * R / 1e3)])
table(["u [kPa] (A02)", "깊이 d [mm]", "A [mm²]", "F_x [N]", "F_z = 0.5F_x", "F_y = 0.3F_x",
       f"τ_hold = F·z_c (z_c {Z_CENT:.1f} mm) [N·m]", "τ_close = F·R (A05) [N·m]"], rows)
F_NOM = 90.0 * 1e-3 * A_PORTION
F_UP = 160.0 * 1e-3 * A_PORTION
F_STOP = 1.2 * F_UP
p(f"- 레인 {LANE:.0f} mm에서 1 portion(115 g) 깊이 {D_PORTION:.1f} mm, 절삭단면 {A_PORTION:.0f} mm² (`tub_lane_planner.stroke_volume` 재사용). "
  f"z_c는 `scoop_model.segment_centroid_below_centre_mm`로 계산({Z_CENT:.1f} mm; repo 상수 22 mm와 일치).")
p(f"- 속도 효과: F ∝ (v/80)ⁿ(A03). 드래그 30 mm/s, n = 0.15면 ×{(30/80)**0.15:.2f}. **보수적으로 무시**(80 mm/s 기준 힘 사용).")
p("")
p("**제안 하중 수준** (E0 실측 전까지):")
p("")
V_DRAG_SEL = 30.0
LEAD_X, LEAD_Z = 4.0, 2.0


def screw_eff(lead, mu):
    """Shigley §8-2 [S6]: efficiency when driving against the load (thrust collar on rolling bearing ignored)."""
    dm, sec = TR8_DM, 1.0 / math.cos(ALPHA_THREAD)
    t_r = 0.5 * dm * (lead + math.pi * mu * dm * sec) / (math.pi * dm - mu * lead * sec)   # per N of load
    return lead / (2 * math.pi * t_r)


def screw_torque_lower(F, lead, mu):
    """Shigley §8-2 [S6]: T_L = F·dm/2·(π·f·dm·secα − l)/(π·dm + f·l·secα); < 0 → load back-drives."""
    dm, sec = TR8_DM, 1.0 / math.cos(ALPHA_THREAD)
    return F * dm / 2 * (math.pi * mu * dm * sec - lead) / (math.pi * dm + mu * lead * sec) / 1e3   # N·m


def mu_min_selflock(lead):
    """self-locking when f·secα > tanλ  (Shigley §8-2 [S6])."""
    lam = math.atan(lead / (math.pi * TR8_DM))
    return math.tan(lam) * math.cos(ALPHA_THREAD)


F_JAM_X = 2 * math.pi * screw_eff(LEAD_X, MU_NOM) * T_LO / (LEAD_X / 1e3)
table(["수준", "F_x [N]", "근거", "쓰임"], [
    ["공칭 F_nom", f"{F_NOM:.0f}", "u 90 kPa(A02 medium) × A", "강성 목표 ≤ 1 mm"],
    ["설계 F_d", f"{F_UP:.0f}", "u 160 kPa(A02 hard) × A", "강성·추력·θ 토크 설계, 깊이 적응 시작(F_TARGET)"],
    ["정지 한계 F_STOP", f"{F_STOP:.0f}", "1.2 × F_d (힘 잡음·변동 여유, 설계 선택)", "호스트 피드 홀드, 추력 여유 계산"],
    ["걸림(jam) F_JAM", f"{F_JAM_X:.0f}", f"X 구동 최대 추력: 2π·η·T_LO/l (TR8 리드 {LEAD_X:.0f}, μ {MU_NOM})", "강도 안전율(항복 SF ≥ 2), 센서 과부하"],
])
p("- 스테퍼는 걸리면 탈조(스톨)한다 → **걸림 하중의 상한 = 구동계가 낼 수 있는 최대 추력**. 구조 강도는 이 값으로 본다.")
p("- 세로·옆 힘은 A28 비율(F_z = 0.5 F_x, F_y = 0.3 F_x). F_z 부호(위/아래)는 **미확인** → E0에서 측정.")
p("")

# ============================================================================= 2. stiffness
p("## 2. 스쿱 끝 강성")
p("")
p("### 2.0 기존 모델 재현 확인 (`calc/scoop_load_path.py` 식을 옮겨 씀)")
p("")

REPO_CFG = {   # copied from calc/scoop_load_path.py CONFIGS
    "L": {"col_EI": E_AL * 10.4e4, "beam_EI": E_AL * 71.9e4, "block_k": 60e3, "block_s": 60.0,
          "drive": "belt", "belt_ksp": 18.9e3, "stem": "25x3"},
    "M": {"col_EI": E_AL * 71.9e4, "beam_EI": E_AL * 129.1e4, "block_k": 200e3, "block_s": 150.0,
          "drive": "belt", "belt_ksp": 150e3, "stem": "25x3"},
}


def repo_tip_dx(cfg, F, c, stem, h_block, beam_span=1450.0):
    """calc/scoop_load_path.tip_dx (arch 1) with explicit stem / H_BLOCK."""
    ls = stem + cm.Z_C_LEVER
    out = {"stem": F * ls ** 3 / (3 * E_SS * STEM_I[cfg["stem"]])}
    col_bottom = c + stem + cm.HEAD_LEN
    ov = h_block - col_bottom
    below = cm.HEAD_LEN + stem + cm.Z_C_LEVER
    lever = h_block - (c - cm.Z_C_LEVER)
    EI = cfg["col_EI"]
    M = F * below
    d_end = F * ov ** 3 / (3 * EI) + M * ov ** 2 / (2 * EI)
    rot = F * ov ** 2 / (2 * EI) + M * ov / EI
    out["column"] = d_end + rot * below
    K_rot = cfg["block_k"] * cfg["block_s"] ** 2 / 2.0
    out["Z blocks"] = F * lever ** 2 / K_rot
    out["X blocks"] = F * lever ** 2 / K_rot
    out["beam"] = F * lever * beam_span / (12 * cfg["beam_EI"]) * lever
    l1, l2 = 560.0, 590.0
    out["X drive"] = F / (cfg["belt_ksp"] * (1 / l1 + 1 / l2))
    return out, lever


PUBLISHED = {("L", 50): 1.36, ("L", 80): 2.17, ("L", 110): 2.99, ("M", 50): 0.23, ("M", 80): 0.38, ("M", 110): 0.52}
rows = []
ok_all = True
for (key, F), pub in PUBLISHED.items():
    d, lev = repo_tip_dx(REPO_CFG[key], F, C_MIN, STEM, Z_L)
    tot = sum(d.values())
    ok = abs(tot - pub) <= 0.01
    ok_all &= ok
    rows.append([key, f"{F}", f2(tot), f2(pub), "●" if ok else "✘"])
table(["repo 구성", "F_x [N]", "이 스크립트 [mm]", "capstone_alternatives.md §2 [mm]", "일치(±0.01)"], rows)
assert ok_all, "re-implementation of the repo stiffness model does not reproduce published V1-S values"
p(f"→ 재현됨(레버 {lev:.0f} mm). 아래 V1-L 모델은 같은 식에 **구성별 하중 경로 요소**(교차빔 면외 굽힘, 타워, 베드 블록, 리드스크류 축강성, S-빔)를 더한 것이다.")
p("")


def cantilever_column(F, EI, ov, below):
    """column overhang ov below the lower Z block pair, force at `below` under the column bottom."""
    M = F * below
    d_end = F * ov ** 3 / (3 * EI) + M * ov ** 2 / (2 * EI)
    rot = F * ov ** 2 / (2 * EI) + M * ov / EI
    return d_end + rot * below


def screw_axial_k(length_to_bearing):
    a = math.pi * TR8_DR ** 2 / 4.0
    k_s = E_ST * a / length_to_bearing
    return 1.0 / (1.0 / k_s + 1.0 / K_NUT + 1.0 / K_SUPPORT)


def tower_k(leg):
    """A-frame tower: two inclined legs (front/back) from the bridge top to feet at ±LEG_FOOT (pin-jointed truss)."""
    h = H_RIM + Z_U
    L = math.hypot(LEG_FOOT, h)
    c2 = (LEG_FOOT / L) ** 2
    return 2 * E_AL * PROF[leg]["A"] * c2 / L


CAND = {
    "L-A": {"name": "L-A 이동 베드(추천 구성)", "col": "4080", "cb": "4080", "leg": "2040", "z_rails": 1,
            "stem": "30x3"},
    "L-A-lite": {"name": "L-A 경량(4040 기둥·빔, 2020 다리)", "col": "4040", "cb": "4040", "leg": "2020", "z_rails": 1,
                 "stem": "30x3"},
    "L-B": {"name": "L-B 이동 갠트리(4080 X빔, TR8)", "col": "4080", "xbeam": "4080", "x_span": 850.0, "s_x": 100.0,
            "s_z": 150.0, "z_rails": 1, "stem": "30x3", "k_tower": None},
}


def tip_LA(cfg, F, c):
    """Drag-direction tip displacement [mm] of the moving-bed layout, by element."""
    out = {}
    ls = STEM + cm.Z_C_LEVER
    out["stem"] = F * ls ** 3 / (3 * E_SS * STEM_I[cfg["stem"]])
    ov = Z_L - (c + STEM + HEAD)
    below = HEAD + STEM + cm.Z_C_LEVER
    out["Z column"] = cantilever_column(F, E_AL * PROF[cfg["col"]]["I_s"], ov, below)
    pz = c - cm.Z_C_LEVER
    arm_l = Z_L - pz                                    # force point below the lower block / beam
    R_u = F * arm_l / H_B                               # statics of a rigid Z assembly on two supports
    R_l = F + R_u
    k_blk = cfg["z_rails"] * K_MGN12                    # one block per rail at each cross-beam
    d_l, d_u = R_l / k_blk, R_u / k_blk
    out["Z guide"] = d_l + (d_l + d_u) / H_B * arm_l
    EI_cb = E_AL * PROF[cfg["cb"]]["I_s"]               # 80 (or 40) side along X
    d_l, d_u = R_l * S_CB ** 3 / (48 * EI_cb), R_u * S_CB ** 3 / (48 * EI_cb)   # simply supported (conservative)
    out["cross-beams"] = d_l + (d_l + d_u) / H_B * arm_l
    kt = tower_k(cfg["leg"])
    d_l, d_u = R_l / 2 / kt, R_u / 2 / kt
    out["towers"] = d_l + (d_l + d_u) / H_B * arm_l
    out["X drive (bed)"] = F / screw_axial_k(X_SCREW_NUT_MAX)
    h_bed = H_RIM + pz - BED_BLOCK_Z
    K_bed = 2 * K_MGN12 * S_BED_BLOCKS ** 2 / 2.0       # 2 rails × 2 blocks, pitch couple
    out["bed blocks"] = F * h_bed ** 2 / K_bed
    out["S-beam"] = F / K_SB
    return out


def tip_LB(cfg, F, c):
    out = {}
    ls = STEM + cm.Z_C_LEVER
    out["stem"] = F * ls ** 3 / (3 * E_SS * STEM_I[cfg["stem"]])
    ov = Z_L - (c + STEM + HEAD)
    below = HEAD + STEM + cm.Z_C_LEVER
    out["Z column"] = cantilever_column(F, E_AL * PROF[cfg["col"]]["I_s"], ov, below)
    pz = c - cm.Z_C_LEVER
    s_z = cfg["s_z"]
    arm_l = Z_L - pz
    R_u = F * arm_l / s_z
    R_l = F + R_u
    k_blk = cfg["z_rails"] * K_MGN12
    d_l, d_u = R_l / k_blk, R_u / k_blk
    out["Z guide"] = d_l + (d_l + d_u) / s_z * arm_l
    lever_x = Z_L + s_z / 2 - pz
    K_x = K_MGN12 * cfg["s_x"] ** 2                    # 4 blocks at ±s_x/2 on 2 rails
    out["X carriage"] = F * lever_x ** 2 / K_x
    EI_b = E_AL * PROF[cfg["xbeam"]]["I_s"]
    out["X beam"] = F * lever_x * cfg["x_span"] / (12 * EI_b) * lever_x
    out["towers"] = F / tower_k("2040")                # braced towers, same stiffness assumption as L-A
    out["X drive"] = F / screw_axial_k(700.0)
    out["S-beam"] = F / K_SB
    return out


def side_LA(cfg, Fy, c):
    ls = STEM + cm.Z_C_LEVER
    d = Fy * ls ** 3 / (3 * E_SS * STEM_I[cfg["stem"]])
    ov = Z_L - (c + STEM + HEAD)
    below = HEAD + STEM + cm.Z_C_LEVER
    d += cantilever_column(Fy, E_AL * PROF[cfg["col"]]["I_w"], ov, below)
    pz = c - cm.Z_C_LEVER
    arm_l = Z_L - pz
    R_u = Fy * arm_l / H_B
    R_l = Fy + R_u
    k_blk = cfg["z_rails"] * K_MGN12
    d += R_l / k_blk + (R_l + R_u) / k_blk / H_B * arm_l
    # in-plane portal with two 2020 knee braces (leg ~ +500 above table → lower cross-beam)
    kb_L = math.hypot(140.0, 235.0)
    k_knee = 2 * E_AL * PROF["2020"]["A"] * (140.0 / kb_L) ** 2 / kb_L
    d += Fy / k_knee
    return d


def side_LB(cfg, Fy, c):
    ls = STEM + cm.Z_C_LEVER
    d = Fy * ls ** 3 / (3 * E_SS * STEM_I[cfg["stem"]])
    ov = Z_L - (c + STEM + HEAD)
    below = HEAD + STEM + cm.Z_C_LEVER
    d += cantilever_column(Fy, E_AL * PROF[cfg["col"]]["I_w"], ov, below)
    pz = c - cm.Z_C_LEVER
    lever_x = Z_L + cfg["s_z"] / 2 - pz
    T = Fy * lever_x
    phi = T * cfg["x_span"] / (4 * G_AL * PROF[cfg["xbeam"]]["J"])       # repo tip_dy torsion term
    d += phi * lever_x
    d += Fy * cfg["x_span"] ** 3 / (48 * E_AL * PROF[cfg["xbeam"]]["I_w"])
    K_roll = 4 * K_MGN12 * 20.0 ** 2                                      # 2 rails 40 mm apart on the beam face
    d += T / K_roll * lever_x
    return d


def joint_dx(F, kj):
    """head bracket-to-column joint (moment F·below) + stem clamp (moment F·ls)."""
    below = HEAD + STEM + cm.Z_C_LEVER
    ls = STEM + cm.Z_C_LEVER
    return F * below ** 2 / kj + F * ls ** 2 / kj


p("### 2.1 드래그 방향(X) 스쿱 끝 변위 — 요소별 (체결부 제외, 기존 모델과 같은 기준)")
p("")
p(f"기하(A41): 스템 {STEM:.0f}, 헤드 {HEAD:.0f}, 하부 Z 블록/하부 교차빔 z_l = {Z_L:.0f}, 상부 z_u = {Z_U:.0f} (팬 테두리 기준 mm). "
  f"C 범위 +{Z_TOP:.0f} … {C_MIN:.0f}. 팬 테두리는 책상 위 {H_RIM:.0f} mm.")
p("")
res = {}
rows = []
for key in ("L-A", "L-A-lite", "L-B"):
    cfg = CAND[key]
    for c, clab in ((C_MIN, "바닥"), (5.0, "가득")):
        for F in (F_NOM, F_UP):
            d = tip_LA(cfg, F, c) if key.startswith("L-A") else tip_LB(cfg, F, c)
            tot = sum(d.values())
            res[(key, round(c), round(F))] = (d, tot)
            cells = [key, f"{clab} C={c:.0f}", f"{F:.0f}"]
            for k in ("stem", "Z column", "Z guide", "cross-beams", "X carriage", "X beam", "towers",
                      "X drive (bed)", "X drive", "bed blocks", "S-beam"):
                if k in d:
                    cells.append(f3(d[k]))
                else:
                    cells.append("–")
            cells += [f"**{tot:.2f}**", f"{F/tot:.0f}", "●" if tot <= 1.0 else "✘"]
            rows.append(cells)
table(["후보", "수위", "F_x [N]", "stem", "Z 기둥", "Z 가이드", "교차빔", "X 캐리지", "X 빔", "타워",
       "X 구동(베드)", "X 구동", "베드 블록", "S-빔", "합계 [mm]", "강성 [N/mm]", "≤ 1 mm"], rows)

# L-C: kit / printer frames
d_Lc, _ = repo_tip_dx(REPO_CFG["L"], F_UP, C_MIN, STEM, Z_L)
tot_Lc = sum(d_Lc.values())
p(f"- **L-C(키트·중고 프린터)**: 가장 유리하게 보아 기존 'L 3D-printer-like'(MGN12 60 mm 간격, 4040 기둥, GT2 벨트) 모델로도 "
  f"F_d {F_UP:.0f} N에서 **{tot_Lc:.2f} mm**(그중 벨트 {d_Lc['X drive']:.2f} mm). 실제 3018 CNC(Ø10 봉)·FDM 프린터(V휠, 한쪽 Z)는 이보다 약하다.")
p(f"- GT2 6 mm 작업장력 ~28 N [S8] 대비 드래그 {F_NOM:.0f} / {F_UP:.0f} N = **{F_NOM/28:.1f} / {F_UP/28:.1f}배** → 벨트 축으로 드래그 불가(강성 이전에 강도).")
p("- **해석**: L-A는 X 캐리지의 **회전 항(레버² 항)이 없다**. 드래그 힘은 팬→플랫폼→베드→베드 리드스크류 너트로 가고, 베드 블록이 받는 모멘트 팔은 "
  f"{H_RIM + C_MIN - cm.Z_C_LEVER - BED_BLOCK_Z:.0f}–{H_RIM + 5 - cm.Z_C_LEVER - BED_BLOCK_Z:.0f} mm(레버 517 mm 대비 짧음)에 블록 간격 {S_BED_BLOCKS:.0f} mm라 "
  "0.001 mm 수준이다. 그러나 **베드 구동(리드스크류 축강성)은 여전히 루프 안**이다(베드를 X로 붙잡는 것이 너트이므로). 대신 짧고(600 mm) 드래그 중 하중 방향이 한쪽이라 백래시가 한쪽으로 붙는다.")
p("- 대가: 헤드가 고정 브리지(YZ면 포털)에 달리므로 **드래그 힘이 포털 면에 수직(면외)** 이다. 교차빔 2개(상·하, 간격 220)로 모멘트를 짝힘으로 바꾸고 "
  "타워를 A-프레임(삼각형)으로 해야 위 표 값이 나온다. 교차빔 1개(비틀림으로 받음)면 아래 §2.3처럼 크게 무너진다.")
p("")

p("### 2.2 체결부(볼트 조인트) 민감도 — 레포 모델에 없던 항")
p("")
p("헤드 브래킷–Z 기둥 체결(모멘트 F·(헤드+스템+z_c))과 스템 클램프(SK30 2개, 모멘트 F·(스템+z_c))의 회전강성 k_j를 가정 범위로 넣었다: "
  "δ_j = F·a²/k_j (a = 모멘트 팔).")
p("")
rows = []
for key in ("L-A", "L-A-lite", "L-B"):
    for F in (F_NOM, F_UP):
        base = res[(key, round(C_MIN), round(F))][1]
        cells = [key, f"{F:.0f}", f2(base)]
        for lab in ("stiff", "mid", "soft"):
            t = base + joint_dx(F, K_JOINT[lab])
            cells.append(f"{t:.2f} {'●' if t <= 1.0 else '✘'}")
        rows.append(cells)
table(["후보", "F_x [N]", "체결부 제외 [mm]", f"+ 체결부 k_j = {K_JOINT['stiff']:.0e}", f"{K_JOINT['mid']:.0e} (중간 가정)",
       f"{K_JOINT['soft']:.0e} N·mm/rad"], rows)
p("- 체결부 강성은 **카탈로그·실측 근거가 없다**. 헤드 브래킷은 8 mm 알루미늄판 + M6 4개 이상, 스템은 SK30 2개를 60 mm 떨어뜨려 잡는 것으로 가정했다.")
p("- 결론: 부재·레일만으로는 L-A가 F_d에서도 여유가 크지만, **실제 스쿱 끝 강성은 조립 후 측정해야 확정**된다(§2.5 보상 조건).")
p("")

p("### 2.3 옆방향(F_y = 0.3 F_x) 변위와 교차빔 1개 구성")
p("")
rows = []
for key in ("L-A", "L-A-lite", "L-B"):
    cfg = CAND[key]
    for F in (F_NOM, F_UP):
        Fy = K_SIDE * F
        d = side_LA(cfg, Fy, C_MIN) if key.startswith("L-A") else side_LB(cfg, Fy, C_MIN)
        rows.append([key, f"{F:.0f}", f"{Fy:.0f}", f2(d), "●" if d <= 1.0 else "✘"])
cfgB8 = dict(CAND["L-B"], xbeam="8080")
for F in (F_NOM, F_UP):
    d = side_LB(cfgB8, K_SIDE * F, C_MIN)
    rows.append(["L-B, X빔 8080", f"{F:.0f}", f"{K_SIDE*F:.0f}", f2(d), "●" if d <= 1.0 else "✘"])
table(["후보", "F_x [N]", "F_y [N]", "옆 변위 [mm]", "≤ 1 mm"], rows)
# single cross-beam (torsion) variant of L-A
for prof in ("4080", "4040"):
    T = F_UP * (Z_L + H_B / 2 - (C_MIN - cm.Z_C_LEVER))
    phi = T * S_CB / (4 * G_AL * PROF[prof]["J"])
    p(f"- L-A를 교차빔 **1개**({prof})로 만들면 드래그 모멘트 {T/1e3:.0f} N·m를 비틀림으로 받는다: φ = T·S/(4GJ) → 스쿱 끝 +{phi * (Z_L + H_B/2 - (C_MIN - cm.Z_C_LEVER)):.1f} mm (J 가정값). **금지.**")
p("- L-A에서 옆힘은 포털 면 안(面內)이다. 무릎 가새(2020, 교차빔↔다리) 2개로 삼각형을 만들면 옆 강성은 기둥·스템이 정한다.")
p("- L-B에서는 옆힘이 X 빔을 **비튼다**(레버 ~0.6 m). 4080 한 개면 1 mm를 넘고, 8080이나 닫힌 상자 단면이 필요하다(비용·무게 증가).")
p("")

p("### 2.4 스쿱 끝 X 변위가 portion(깊이)에 주는 영향 — 무엇이 실제로 중요한가")
p("")
dLA, totLA = res[("L-A", round(C_MIN), round(F_UP))]
phi_frame = totLA / (Z_L - (C_MIN - cm.Z_C_LEVER))
dd_pitch = R * math.sin(abs(ATT)) * phi_frame
# vertical compliance (depth-relevant) under F_z at F_d
Fz = K_VERT * F_UP
k_zdrive = screw_axial_k(150.0)
EI_cb_v = E_AL * PROF["4080"]["I_w"]
d_cb_v = Fz * S_CB ** 3 / (48 * EI_cb_v)
d_sp = Fz / K_SP
d_z_tot = Fz / k_zdrive + d_cb_v + d_sp
dd_theta = R * math.sin(abs(ATT)) * math.radians(PG_BACKLASH_DEG)
sens = 1.5 / D_PORTION            # dA/A per mm near d (A ~ d^1.5 locally) — ASSUMPTION
table(["항목", "값", "식·비고"], [
    ["X 변위에 따른 프레임 회전 φ", f"{math.degrees(phi_frame):.3f}°", f"δ_x/레버 = {totLA:.2f}/{Z_L - (C_MIN - cm.Z_C_LEVER):.0f}"],
    ["φ에 의한 깊이 변화", f"{dd_pitch:.3f} mm", "Δd = R·sin30°·φ → 무시 가능"],
    [f"F_z = {Fz:.0f} N 수직 변위: Z 구동(TR8 리드2)", f"{Fz/k_zdrive:.3f} mm", "나사·너트·지지 직렬(가정 강성)"],
    ["  + 상부 교차빔 수직 굽힘(4080, 40 수직)", f"{d_cb_v:.3f} mm", "F·S³/(48EI)"],
    [f"  + 단일점 로드셀({SP_CAP_KG:.0f} kg, 정격 처짐 {SP_DEFL} mm 가정)", f"{d_sp:.3f} mm", "팬이 눌려 내려감"],
    ["수직 합계(깊이 오차, 보상 전)", f"**{d_z_tot:.2f} mm**", f"portion 약 {d_z_tot*sens*100:.1f} % (dA/A ≈ 1.5/d = {sens*100:.0f} %/mm)"],
    ["Z 너트 백래시(하중 반전 시)", f"≤ {BACKLASH_NUT:.2f} mm", f"F_z(위)가 W_z {W_Z:.0f} N을 넘으면(F_x > {W_Z/K_VERT:.0f} N, F_z 부호 미확인) 너트가 반대 면으로 → ≈ {BACKLASH_NUT*sens*100:.0f} %"],
    [f"θ 백래시 {PG_BACKLASH_DEG:.0f}°(가정)", f"{dd_theta:.2f} mm", "드래그 토크가 한 방향이라 한쪽으로 붙음 → 반복 가능"],
])
p("- **X 변위 ≤ 1 mm 목표는 깊이 정밀도 때문이 아니다.** X 변위는 레인 끝 벽 여유(10 mm)와 떨림(stick-slip) 여부에만 영향이 있다. "
  "portion을 정하는 것은 **수직 컴플라이언스(F_z)** 이고, 이것은 힘 플랫폼이 F_z를 재므로 보상할 수 있다.")
p("")

p("### 2.5 소프트웨어 보상 — 허용 조건")
p("")
p("보상식: `z_cmd = z_target + C_z(Z)·F_z,meas` (스트로크 간·20 mm 구간 적응 때 적용; 공개 펌웨어라 드래그 중 실시간 수정은 하지 않음). X는 보상하지 않는다(레인 방향 오차).")
p("")
table(["조건(조립 후 교정 시험)", "합격 기준(제안)", "안 되면"], [
    ["교정: 스쿱 끝에 0 → F_STOP → 0 N, 3회, Z 3높이(+5 / −35 / −75)", "선형 회귀 R² ≥ 0.99", "비선형 → 보상 금지, 원인(유격·체결 미끄럼) 수리"],
    ["히스테리시스(같은 F에서 부하·제하 차)", "≤ 0.10 mm", "체결부 미끄럼 → 볼트 추가·접착 고정"],
    ["반복성(회차·날짜 간 C 표준편차)", "≤ 10 % 그리고 ≤ 0.05 mm @ F_d", "보상 대신 드래그 힘 목표 하향"],
    ["영점 복귀(제하 후 잔류)", "≤ 0.05 mm", "미끄럼 → 수리"],
    ["드래그 중 힘 진동(stick-slip)", "F 진폭 ≤ 20 % (5 Hz 이상 성분)", "속도·깊이 변경, 강성 보강"],
    ["힘 측정 불확도 × C", f"예: ±3 N × {1/ (F_UP/ (res[('L-A', round(C_MIN), round(F_UP))][1] + joint_dx(F_UP, K_JOINT['mid']))):.4f} mm/N ≈ ±{3 * (res[('L-A', round(C_MIN), round(F_UP))][1] + joint_dx(F_UP, K_JOINT['mid'])) / F_UP:.3f} mm", "무시 가능"],
])

# ============================================================================= 3. X drive
p("## 3. X 구동 (L-A: 베드) — TR8 리드 선택")
p("")
p("동력나사 식 [S6]: T_R = F·d_m/2 · (l + π·f·d_m·secα)/(π·d_m − f·l·secα), η = F·l/(2π·T_R), 자립 조건 f·secα > tanλ (λ = atan(l/(π·d_m))), "
  f"d_m = {TR8_DM:.1f} mm, α = 15°. 스테퍼 곡선(가정): {T_LO} N·m 평탄 → {N_CORNER:.0f} rpm 이후 일정 출력. 요구 여유 ≥ {SF_STEPPER}.")
p("")
f_fric = MU_RAIL * M_BED * 9.81 + 4 * F_SEAL_PER_BLOCK
F_req_drag = F_STOP + f_fric + M_BED * A_DRAG
J_screw = math.pi * RHO_ST * (X_SCREW_L / 1e3) * (TR8_D / 1e3) ** 4 / 32.0
p(f"- 베드 가동 질량 {M_BED:.1f} kg (" + ", ".join(f"{k} {v:.1f}" for k, v in MASS_BED.items()) + ")")
p(f"- 요구 추력(드래그) = F_STOP {F_STOP:.0f} + 마찰 {f_fric:.1f} + m·a_drag {M_BED*A_DRAG:.1f} = **{F_req_drag:.0f} N**")
x_lane0 = R + cm.WALL_MARGIN
x_cup = PAN["L"] + HOLDER_WALL + 10.0 + CUP_R
travel = x_cup - x_lane0 + 2 * 20.0
rail_need = S_BED_BLOCKS + MGN12H_BLOCK_L + travel
assert rail_need <= RAIL_BED, "bed rails too short"
p(f"- 베드 행정: 레인 시작 C(팬 안쪽 x = {x_lane0:.0f}) → 컵 중심(x = {PAN['L']:.0f} + 홀더 벽 {HOLDER_WALL:.0f} + 틈 10 + 컵 반경 {CUP_R:.0f} = {x_cup:.0f}) "
  f"= {x_cup - x_lane0:.0f} mm + 양끝 여유 20 → **{travel:.0f} mm**. 레일 필요 길이 = 블록 간격 {S_BED_BLOCKS:.0f} + 블록 {MGN12H_BLOCK_L:.0f} + 행정 = {rail_need:.0f} ≤ {RAIL_BED:.0f} mm. "
  "컵 위치가 곧 **서비스 위치**다(헤드가 홀더 밖 → 뚜껑·레인 인덱스 작업).")
p("")
rows = []
n_crit = 9.7 * TR8_DR / X_SCREW_L ** 2 * 1e7          # supported-supported [S7]
lead_summary = {}
for lead in (2.0, 4.0, 8.0):
    eta = screw_eff(lead, MU_NOM)
    lam = math.degrees(math.atan(lead / (math.pi * TR8_DM)))
    mumin = mu_min_selflock(lead)
    margins = []
    for v in (20.0, 30.0, 40.0):
        n = v / lead * 60.0
        alpha = A_DRAG * 2 * math.pi / (lead / 1e3)
        t_req = F_req_drag * (lead / 1e3) / (2 * math.pi * eta) + (J_ROTOR + J_screw) * alpha
        t_av = stepper_torque(n)
        margins.append(t_av / t_req)
        rows.append([f"{lead:.0f}", f"{v:.0f}", f"{n:.0f}", f3(t_req), f3(t_av), f"{t_av/t_req:.2f}",
                     "●" if t_av / t_req >= SF_STEPPER else "✘"])
    # rapid: highest speed with margin >= SF at travel load, capped by RPM_CAP and critical speed
    f_travel = M_BED * A_TRAVEL + f_fric
    v_rapid = 0.0
    for n in np.arange(100.0, RPM_CAP + 1.0, 10.0):
        alpha = A_TRAVEL * 2 * math.pi / (lead / 1e3)
        t_req = f_travel * (lead / 1e3) / (2 * math.pi * eta) + (J_ROTOR + J_screw) * alpha
        if stepper_torque(n) / t_req >= SF_STEPPER and n <= n_crit:
            v_rapid = n * lead / 60.0
    f_max = 2 * math.pi * eta * T_LO / (lead / 1e3)
    fpm = {v: math.pi * TR8_DM * (v / lead * 60.0) / math.cos(math.radians(lam)) / 1e3 / 0.3048 for v in (30.0, 40.0)}
    lead_summary[lead] = dict(eta=eta, lam=lam, mumin=mumin, v_rapid=v_rapid, f_max=f_max, fpm=fpm, margins=margins)
table(["리드 l [mm]", "드래그 v [mm/s]", "n [rpm]", "T_req [N·m]", "T_avail (가정) [N·m]", "여유", f"≥ {SF_STEPPER}"], rows)
A_bear = math.pi * TR8_DM * (TR8_P / 2) * (NUT_LEN / TR8_P)
rows = []
def shigley_class(fpm):
    if fpm <= 10:
        return "11–17 MPa 구간"
    if fpm <= 40:
        return "5.5–9.7 MPa 구간"
    return "표의 저속 구간 밖(허용 면압 크게 낮음)"


press = F_req_drag / A_bear
for lead, s in lead_summary.items():
    selfl = f"가정 μ {MU_CASES[0]}–{MU_CASES[-1]}에서 자립" if s["mumin"] < MU_CASES[0] else ("μ에 따라 경계" if s["mumin"] < MU_CASES[-1] else "역구동")
    rows.append([f"{lead:.0f}", f"{s['lam']:.1f}", f"{s['eta']*100:.0f}", f"{s['mumin']:.3f} → {selfl}", f"{s['v_rapid']:.0f}",
                 f"{s['f_max']:.0f}", f"{s['fpm'][30.0]:.0f} fpm → {shigley_class(s['fpm'][30.0])}",
                 f"{s['fpm'][40.0]:.0f} fpm → {shigley_class(s['fpm'][40.0])}"])
table(["리드", "λ [°]", f"η @ μ {MU_NOM} [%]", "자립 한계 μ_min", "이송 최대 [mm/s]", "최대 추력(스톨) [N]",
       "너트 미끄럼속도 @ 30 mm/s", "@ 40 mm/s"], rows)
p(f"- 너트 면압 p = 추력 / (π·d_m·h·n), h = P/2 = 1 mm, 맞물림 {NUT_LEN/TR8_P:.1f}산(너트 {NUT_LEN:.0f} mm 가정) → {A_bear:.0f} mm². "
  f"F_STOP 추력에서 p = {F_req_drag:.0f} N / {A_bear:.0f} mm² = **{press:.2f} MPa** — Shigley 표 8-4 [S6]의 20–40 fpm 구간 하한 5.5 MPa 대비 {5.5/press:.0f}배. "
  "40 fpm을 넘는 미끄럼속도는 표 값이 크게 낮아지므로 피한다.")
p(f"- 나사 임계속도(지지-지지, L {X_SCREW_L:.0f}, d_r {TR8_DR:.0f}) N1 = 9.7·d_r/L²·10⁷ = **{n_crit:.0f} rpm** [S7] → 양끝 지지 필수(한쪽 자유면 {3.4 * TR8_DR / X_SCREW_L**2 * 1e7:.0f} rpm).")
n_crit_B = 9.7 * TR8_DR / 800.0 ** 2 * 1e7
p(f"- 비교: L-B의 X 나사(TR8 × 800) 임계속도 {n_crit_B:.0f} rpm → 리드 4 이송 ≤ {n_crit_B*4/60:.0f} mm/s.")
ls4 = lead_summary[LEAD_X]
p(f"- **선정: X = TR8 리드 {LEAD_X:.0f}(2줄)**, 드래그 **{V_DRAG_SEL:.0f} mm/s**(여유 {ls4['margins'][1]:.2f}), 이송 ~{ls4['v_rapid']:.0f} mm/s. "
  "리드 2는 같은 선속도에서 rpm이 2배라 토크·너트 미끄럼속도에서 불리, 리드 8은 저속 토크 한계로 여유 < 2.")
p("- X는 자립이 필요 없다(정전 시 베드가 서면 됨). 리드 4는 μ에 따라 경계 → 손으로 베드를 밀 수 있다고 가정하지 말고 나사 손잡이로 돌린다.")
p("")

p("### 3.1 베드 가속이 힘 플랫폼 F_x에 주는 관성 오차")
p("")
f_sb = math.sqrt(K_SB * 1e3 / M_TOP) / (2 * math.pi)
rows = []
for lab, a in (("드래그 시작·끝 램프", A_DRAG), ("이송 가감속", A_TRAVEL), ("이송 가감속(공격적)", 1.0)):
    e = M_TOP * a
    rows.append([lab, f"{a:.1f}", f1(e), f"{e/F_NOM*100:.1f} %", f"{e/F_UP*100:.1f} %"])
table(["구간", "a [m/s²]", f"m_top·a (m_top {M_TOP:.1f} kg) [N]", "F_nom 대비", "F_d 대비"], rows)
t_ramp = V_DRAG_SEL / 1e3 / A_DRAG
p(f"- S-빔({SB_CAP_KG:.0f} kg, k ≈ {K_SB:.0f} N/mm) 위 질량 {M_TOP:.1f} kg의 고유진동수 ≈ **{f_sb:.0f} Hz**. HX711 80 SPS(나이퀴스트 40 Hz)로 읽으면 이 떨림이 에일리어싱된다 → 10 SPS 모드(내장 평균) 또는 소프트웨어 평균.")
p(f"- 대책: (1) F_x 판정은 **등속 구간만** 사용(램프 {t_ramp:.2f} s·{0.5*A_DRAG*t_ramp**2*1e3:.1f} mm + 안정 0.3 s 제외, 레인 {LANE:.0f} mm 중 대부분 사용), "
  "(2) 램프 구간은 `F_cut = F_meas − m_top·a_cmd` (m_top은 수직 셀이 매번 직접 잰다), (3) 스트로크마다 정지 상태에서 F_x 영점, "
  "(4) portion 질량은 베드 정지 후 ≥ 1 s(A32). 이송 중 m_top·a는 A37 이송 충돌 임계 30 N보다 충분히 작다.")
p("")

# ============================================================================= 4. Z drive
p("## 4. Z 구동 — 행정, 자립, 정전")
p("")
stroke = Z_TOP - C_MIN
p(f"- 행정: C = +{Z_TOP:.0f}(Z_TOP, Z_SAFE {Z_SAFE:.0f} + 40) … {C_MIN:.0f}(팬 바닥 여유 {cm.FLOOR_MARGIN:.0f} + R {R:.0f}) = **{stroke:.0f} mm**, "
  f"원점·리밋 여유 ±10 → {stroke + 20:.0f} mm → **TR8 × {Z_SCREW_L:.0f}, MGN12 레일 450**(기둥 500).")
p(f"- Z_SAFE = R + 25 = {Z_SAFE:.0f} mm: 컵 받침 높이를 컵 테두리 ≤ 팬 테두리로 맞춘다(A31). 베드 X 이동은 **Z ≥ Z_SAFE에서만**(R7), 예외는 레인 안 드래그(기존 G_SAFE_Z 규칙과 같음).")
p(f"- Z 가동 질량 {M_Z:.1f} kg → W_z = {W_Z:.0f} N (" + ", ".join(f"{k} {v:.2f}" for k, v in MASS_Z.items()) + ")")
p("")
rows = []
for lead in (2.0, 4.0, 8.0):
    mumin = mu_min_selflock(lead)
    cells = [f"{lead:.0f}", f"{mumin:.3f}"]
    for mu in MU_CASES:
        tl = screw_torque_lower(W_Z, lead, mu)
        if tl >= 0:
            cells.append(f"자립 (+{tl*1e3:.1f} mN·m 필요)")
        else:
            hold = "멈춤 턱토크로 유지?" if -tl < T_DETENT else "**낙하**"
            cells.append(f"역구동 {-tl*1e3:.1f} mN·m → {hold}")
    eta = screw_eff(lead, MU_NOM)
    v_z = min(RPM_CAP, 1200.0) * lead / 60.0
    f_crush = 2 * math.pi * eta * T_LO / (lead / 1e3) + W_Z
    cells += [f"{v_z:.0f}", f"{f_crush:.0f}"]
    rows.append(cells)
table(["리드", "μ_min(자립)", f"정전, μ {MU_CASES[0]}", f"μ {MU_CASES[1]}", f"μ {MU_CASES[2]}",
       "Z 속도 @1200 rpm [mm/s]", "최대 하강력(스톨+자중) [N]"], rows)
Fz_hold = K_VERT * F_STOP + W_Z
t_zreq = Fz_hold * (LEAD_Z / 1e3) / (2 * math.pi * screw_eff(LEAD_Z, MU_NOM))
p(f"- 무전원 멈춤 턱토크 가정 {T_DETENT*1e3:.0f} mN·m(ASSUMPTION). 진동이 있으면 턱토크 유지는 믿지 않는다.")
p(f"- **선정: Z = TR8 리드 2(1줄)**. μ ≥ {mu_min_selflock(2.0):.3f}이면 자립 → **브레이크·가스스프링 불필요**(V1-S S05·S06 삭제). "
  f"요구 토크(0.5·F_STOP 위 + 자중) {t_zreq:.3f} N·m @ ≤ 600 rpm → 여유 {stepper_torque(600)/t_zreq:.1f}.")
p("- 조건: (1) 조립 후 **정전 유지 시험** — 최대 하중(자중 + 추 5 kg)을 걸고 전원 차단, 프레임을 두드리며 10분 동안 처짐 ≤ 0.1 mm, "
  "(2) 윤활은 얇게(과윤활 시 μ < 0.09 가능), (3) Z 나사 위에 손잡이 — E-stop 뒤 손으로 올림(2 mm/회전).")
p(f"- 대가: 하강력이 커서(~{2*math.pi*screw_eff(2.0, MU_NOM)*T_LO/0.002 + W_Z:.0f} N) 손 끼임 한계(ISO/TS 15066 준정적 140 N, repo gantry_motor_sizing §5)를 넘는다 → hold-to-run·가드 유지, "
  f"Z 전류 제한, 힘 플랫폼 셀({SP_CAP_KG:.0f} kg = {SP_CAP_KG*9.81:.0f} N) 보호용 **기계식 과부하 스톱**(간극 0.3 mm)을 둔다.")
p("")

# ============================================================================= 5. theta
p("## 5. θ 구동")
p("")
p(f"- 기존 7 N·m의 근거: `docs/13_engineering_calculations.md` §1 — τ_close = F·R = **200 N**(A23 하드웨어 상한) × 0.035 m (A05: F_close = F_x). "
  "V1-L은 힘이 절반 이하라 이 값을 그대로 쓰지 않는다.")
p("")
rows = []
for lab, F in (("F_nom", F_NOM), ("F_d", F_UP), ("F_STOP", F_STOP)):
    th, tc = F * Z_CENT / 1e3, F * R / 1e3
    rows.append([lab, f"{F:.0f}", f2(th), f2(tc), "●" if tc <= PG_T_CONT else ("△ 순간 정격 안" if tc <= PG_T_PEAK else "✘"),
                 f"{tc/0.025:.0f}"])
table(["수준", "F_x [N]", "τ_hold [N·m]", "τ_close [N·m]", f"PG27 정격 {PG_T_CONT:.0f} / 순간 {PG_T_PEAK:.0f} N·m [S4]",
       "이중 push-rod 최대 로드력 τ/l, l 25 mm [N]"], rows)
rows = []
for t_close in (0.8, 1.5):
    n_out = 120.0 / 360.0 / t_close * 60.0
    n_mot = n_out * PG_RATIO
    t_out = PG_T_LO * min(1.0, 400.0 / max(n_mot, 1.0)) * PG_RATIO * PG_ETA
    rows.append([f"{t_close}", f"{n_out:.0f}", f"{n_mot:.0f}", f2(t_out), f2(min(t_out, PG_T_PEAK))])
table(["닫기 120° 시간 [s]", "출력 [rpm]", "모터 [rpm]", "모터×감속×η 출력 [N·m]", "기어 정격 반영 상한 [N·m]"], rows)
p("- V1-S 문서의 'NEMA17 + 1:27 → 출력 ~9 N·m'는 모터 토크 × 감속비다. **기어박스 허용 토크는 3 N·m(연속) / 5 N·m(순간)** [S4] → 설계에 9 N·m를 쓸 수 없다.")
p(f"- 판정: **NEMA17 + 1:27 유지(조건부)**. F_nom에서 τ_close {F_NOM*R/1e3:.1f} N·m < 3 N·m. F_d(단단한 제품)에서는 {F_UP*R/1e3:.1f} N·m로 연속 정격을 넘어 순간 정격 안에만 든다 "
  "→ 모터 전류로 출력 ≤ 5 N·m 제한, **E0에서 τ_close를 재서 > 3 N·m이면 교체**(C-Gate).")
Lr = STEM
Pcr = math.pi ** 2 * E_SS * (math.pi * 8 ** 4 / 64) / Lr ** 2
p(f"- 이중 push-rod(90° 위상) **유지**: 뒤집기 배출(θ −90°)에 필요, 추가비 ~1–2만 원. Ø8 × {Lr:.0f} 304 좌굴 P_cr = π²EI/L² = {Pcr/1e3:.1f} kN → "
  f"최대 로드력 {PG_T_PEAK/0.025:.0f} N(기어 순간 정격) 대비 SF {Pcr/(PG_T_PEAK/0.025):.0f}.")
p("")
table(["θ 대안", "비용(대략)", "출력·유지", "판정"], [
    ["NEMA17 + 1:27 유성 (현재)", "4–7만 원", "3 / 5 N·m [S4], 역구동됨(전류로 유지), 백래시 ~1°(가정)", "**유지(조건부)**"],
    ["NEMA17 + 웜 30:1", "4–8만 원", "정격 토크 **미확인**, 자립은 '하중·윤활 조건부' [S13], 백래시 15–18′ [S13], 효율 낮음(가정 40–50 %) → 25 rpm에서 출력 부족 가능", "대안: E0에서 τ_close > 3 N·m이면 정격 확인 후"],
    ["GT2 벨트 감속", "1–2만 원", f"출력 풀리에서 τ/r ≤ 28 N [S8] → 3.5 N·m에 r ≥ {3.5/28*1e3:.0f} mm", "기각"],
    ["HTD5M 2단 벨트", "3–5만 원", "1:9 이상 필요, 출력 풀리 Ø100급, 헤드 부피·질량↑, 벨트 늘음 = θ 컴플라이언스", "기각"],
    ["NEMA23 + 유성", "8–15만 원", "여유 큼", "예산 초과 쪽 — 교체 1순위 후보"],
    ["자작 웜(3D프린트 하우징)", "1–3만 원", "정렬·강성 불확실(R9 위험)", "기각"],
])

# ============================================================================= 6. strength
p("## 6. 강도 — 걸림 하중 F_JAM에서")
p("")
F_J = F_JAM_X
ls = STEM + cm.Z_C_LEVER
sig_stem = F_J * ls * STEM_C["30x3"] / STEM_I["30x3"]
pz = C_MIN - cm.Z_C_LEVER
R_u_J = F_J * (Z_L - pz) / H_B
R_l_J = F_J + R_u_J
M_cb = R_l_J * S_CB / 4
sig_cb = M_cb * 40.0 / PROF["4080"]["I_s"]
table(["부재", "하중", "응력·하중", "허용", "SF"], [
    ["스템 Ø30×3 304 (클램프부)", f"M = {F_J:.0f} N × {ls:.0f} mm", f"σ = {sig_stem:.0f} MPa", f"σ_y {SY_304:.0f} MPa", f"{SY_304/sig_stem:.1f}"],
    ["하부 Z 블록 MGN12H", f"R_l = F·(1 + {Z_L - pz:.0f}/{H_B:.0f})", f"{R_l_J:.0f} N", "C0 5.88 kN [S9]", f"{5880/R_l_J:.1f}"],
    ["하부 교차빔 4080 (단순지지)", f"M = R_l·S/4", f"σ = {sig_cb:.0f} MPa", f"σ_y {SY_AL:.0f} MPa(가정)", f"{SY_AL/sig_cb:.1f}"],
    [f"S-빔 {SB_CAP_KG:.0f} kg", f"F_JAM {F_J:.0f} N", f"{F_J/(SB_CAP_KG*9.81)*100:.0f} % 정격", "안전 과부하 ~150 %(가정)", "● (20 kg 셀이면 ✘ → 기계식 스톱)"],
    ["push-rod Ø8 304", f"τ ≤ {PG_T_PEAK} N·m", f"{PG_T_PEAK/0.025:.0f} N", f"P_cr {Pcr/1e3:.1f} kN", f"{Pcr/(PG_T_PEAK/0.025):.0f}"],
    ["교차빔–다리 볼트 체결(M6, 2개, 마찰)", f"R_l/2 = {R_l_J/2:.0f} N", "", "2 × 3.5 kN × μ 0.2 = 1.4 kN(가정)", f"{1400/(R_l_J/2):.1f}"],
])
p("- 강도는 여유가 있다. **강성과 체결부 미끄럼이 설계를 지배**한다(레포의 결론과 같음).")
res_sp = SP_CAP_KG * 1000 / 2 ** 16
p(f"- 셀 용량을 올린 대가: 단일점 {SP_CAP_KG:.0f} kg 셀 유효 분해능 ≈ {res_sp:.2f} g(HX711 유효 16비트 가정, A45와 같은 방식) → 접촉 3 N = 306 g, "
  f"portion 허용 −5 % = 5.8 g 대비 충분.")
p("")

# ============================================================================= 7. frame, parts
p("## 7. 프레임 절단표 (L-A 추천 구성)")
p("")
H_TOWER = H_RIM + Z_U
L_LEG = math.hypot(LEG_FOOT, H_TOWER)
cut = [
    ("4080 (슬롯8)", "교차빔 상·하", S_CB, 2), ("4080 (슬롯8)", "Z 기둥(이동)", 500.0, 1),
    ("2040 (슬롯6)", "베이스 세로(X)", 1000.0, 2), ("2040 (슬롯6)", "베이스 가로(Y)", 560.0, 3),
    ("2040 (슬롯6)", "타워 다리(A-프레임, 경사)", round(L_LEG + 20, -1), 4), ("2040 (슬롯6)", "타워 수평 타이(z_l 높이)", 210.0, 2),
    ("2040 (슬롯6)", "베드 세로 틀", 650.0, 2), ("2020 (슬롯6)", "베드 가로대", 360.0, 2),
    ("2020 (슬롯6)", "무릎 가새(옆힘)", 300.0, 2), ("2020 (슬롯6)", "가드 틀", 750.0, 2),
]
rows = []
tot_len = {}
for prof, use, L, n in cut:
    rows.append([prof, use, f"{L:.0f}", n, f"{L*n/1e3:.2f}"])
    tot_len[prof] = tot_len.get(prof, 0.0) + L * n / 1e3
table(["프로파일", "용도", "길이 [mm]", "개수", "소계 [m]"], rows)
p("합계: " + ", ".join(f"{k} {v:.2f} m" for k, v in tot_len.items()) + f". 타워 높이(책상 위) {H_TOWER:.0f} mm, 다리 발 ±{LEG_FOOT:.0f} mm, 다리 길이 ≈ {L_LEG:.0f} mm(양끝 각도 절단 또는 피벗 조인트).")
p("")
table(["가공 방식", "부품"], [
    ["학교 레이저커터(합판·MDF·아크릴 ≤ 6 mm)", "베드 데크(인덱스 구멍 3쌍), 단열 홀더 외피·팬 턱 받침·X 스톱 블록(적층), 뚜껑, 가드 옆판(아크릴 5 mm), "
     "**알루미늄판 드릴 템플릿**(MDF), 전장판"],
    ["3D 프린터(PETG/PLA) — 하중 적은 것만", "컵 받침, 레인 스위치·리밋 스위치 마운트, 케이블 가이드, 드립 우산 목업, Z 손잡이, 스페이서"],
    ["금속판 절단(외주 DXF 또는 판재 재단 + 드릴)", "헤드 브래킷(Al 8 mm), Z 블록 마운트판 ×2, Z 너트 브래킷, θ 모터 마운트, 상부 크랭크 ×2, 하부 레버판(스쿱 클램프)"],
    ["손 공구(허용: 드릴·탭)", "프로파일 끝 중심구멍 탭(M8/M5), 판 구멍(템플릿 사용), 튜브 절단"],
    ["사지 않는 것", "용접, 선반·밀링 정밀가공, 볼스크류 모듈"],
])

p("## 8. 개략 기계 부품 리스트 (가격은 **대략 범위, ASSUMPTION** — 전자 담당 견적으로 교체)")
p("")
PARTS = [
    # id, group, part, spec, qty, low, high (만 원), note
    ("M01", "프레임", "알루미늄 프로파일 4080(슬롯8)", f"{tot_len['4080 (슬롯8)']:.2f} m, 절단 주문", "1식", 2.7, 4.5, "교차빔·Z 기둥"),
    ("M02", "프레임", "알루미늄 프로파일 2040(슬롯6)", f"{tot_len['2040 (슬롯6)']:.2f} m, 절단 주문", "1식", 5.0, 8.0, "베이스·타워·베드"),
    ("M03", "프레임", "알루미늄 프로파일 2020(슬롯6)", f"{tot_len['2020 (슬롯6)']:.2f} m", "1식", 0.8, 1.5, "가새·가드 틀"),
    ("M04", "프레임", "코너 브래킷·T너트·볼트·각도 조인트·조절 발", "슬롯6/8 혼용", "1식", 3.0, 5.0, "각도 조인트 8개"),
    ("M05", "X(베드)", "MGN12H 레일 800 + 블록", "레일 2, 블록 4", "1식", 5.0, 9.0, "비용 절감안: V슬롯 휠(1–2만)"),
    ("M06", "Z", "MGN12H 레일 450 + 블록", "레일 1, 블록 2", "1식", 2.2, 4.0, "기둥 뒷면"),
    ("M07", "X(베드)", "TR8 리드4 × 600 + 황동 너트, KFL08 ×3, 커플러 5×8, 축 칼라", "", "1식", 1.2, 2.5, "모터 쪽 KFL08 2개로 양방향 고정"),
    ("M08", "Z", "TR8 리드2 × 300 + 황동 너트, KFL08 ×2, 커플러, 손잡이", "", "1식", 0.8, 1.5, "자립 → 브레이크 없음"),
    ("M09", "구동", "NEMA17 59 N·cm급(17HS19-2004S1류)", "X, Z", "2", 2.8, 5.0, "드라이버는 전자"),
    ("M10", "θ", "NEMA17 + 1:27 유성(17HS19-1684S-PG27류)", "3 / 5 N·m 정격", "1", 4.0, 7.0, ""),
    ("M11", "헤드", "알루미늄판 6–8 mm 절단품", "브래킷·마운트·크랭크 6–8종", "1식", 2.0, 5.0, "외주 DXF 또는 재단 + 템플릿 드릴"),
    ("M12", "헤드", "SK30 축 서포트 ×2, 로드엔드 M8 ×4, 핀 Ø8 304, 플랜지 부싱, 칼라", "", "1식", 1.5, 3.0, ""),
    ("M13", "식품 모듈", "304 튜브 Ø30×3 × 250, 304 봉 Ø8 × 2(200), 볼트 캡 + 식품용 실리콘", "", "1벌", 1.5, 3.0, "PoC(시식 없음)"),
    ("M14", "식품 모듈", "매장용 304 스쿱", "", "2", 2.0, 4.0, "1벌 + 예비"),
    ("M15", "힘 플랫폼(기구)", "MGN9 150 레일 ×2 + 블록, 과부하 스톱, 스페이서", "셀·HX711은 전자", "1식", 2.0, 3.5, ""),
    ("M16", "용기·홀더", "합판 6 mm(레이저컷), XPS 50 mm 1장, 인덱스 핀 Ø8 ×2", "", "1식", 2.5, 4.5, ""),
    ("M17", "3D 프린트", "PETG/PLA 필라멘트", "", "1 kg", 0.0, 2.0, "학교 지급이면 0"),
    ("M18", "안전(기구)", "아크릴 5 mm 옆판 ×2(레이저컷) + 경첩·손잡이", "", "1식", 1.5, 3.5, "E-stop·hold-to-run은 전자"),
    ("M19", "배선 기구", "케이블 체인 10×15 × 1 m", "베드 로드셀, Z/θ", "2", 0.8, 1.5, ""),
    ("M20", "기타", "그리스, 나사고정제, 탭 M5/M8, 드릴 비트", "", "1식", 1.0, 2.0, ""),
]
lo = sum(x[5] for x in PARTS)
hi = sum(x[6] for x in PARTS)
table(["ID", "구분", "품목", "규격", "수량", "가격 범위 [만 원]", "비고"],
      [[x[0], x[1], x[2], x[3], x[4], f"{x[5]:.1f}–{x[6]:.1f}", x[7]] for x in PARTS])
p(f"**기계 소계(추정 범위): {lo:.0f}–{hi:.0f}만 원** (젤라토 팬 2개 4–8만 원은 시험 소모품으로 따로).")
p("")
CUTS = [("베드 가이드를 V슬롯 휠로(M05)", 4.0, 7.0), ("스쿱 1개만(M14)", 1.0, 2.0), ("가드를 합판으로(M18)", 0.5, 1.5),
        ("중고 FDM 프린터 부품 기증(NEMA17·커플러·일부 프로파일, 기계 쪽만)", 3.0, 6.0)]
table(["절감 레버", "절감 [만 원]"], [[a, f"{b:.1f}–{c:.1f}"] for a, b, c in CUTS])
p(f"- 모두 쓰면 기계 ≈ {lo - sum(c[2] for c in CUTS):.0f}–{hi - sum(c[1] for c in CUTS):.0f}만 원. 베드 V휠은 드래그 힘이 너트로 가므로 강성 문제는 작지만(§2.1 베드 블록 항), 걸림 하중에서 휠 변형은 확인 필요.")
p("")

p("## 9. V1-S에서 빼는 것과 대신하는 것")
p("")
table(["V1-S 항목(BOM ID)", "V1-S 추정가", "V1-L", "근거"], [
    ["X 볼스크류 모듈 + NEMA23 폐루프(S02, S04)", "43만", "베드 + TR8 리드4 + MGN12 + NEMA17 개루프", "§2.1, §3"],
    ["Z 볼스크류 모듈 + 브레이크 폐루프 + 가스스프링(S03, S05, S06)", "35만", "TR8 리드2(자립) + MGN12 1줄 + NEMA17", "§4"],
    ["자동 Y 슬라이드(S08)", "7만", "수동 인덱스 핀(−38/0/+38) + 레인 확인 스위치", "레인 변경은 층마다 2회, 헤드 Z_TOP·구동 정지 상태에서만"],
    ["가드 3면 폴리카보네이트 + 인터록(S27)", "10만", "아크릴 옆판 2장 + hold-to-run + E-stop 유지", "완전 인클로저는 이후 단계"],
    ["프로파일 4040·4080 ~8 m(S01)", "22만", "2040 중심 + 4080 1.5 m", "§7"],
    ["식품 모듈 2벌(S15–S17)", "13만", "1벌 + 예비 스쿱", ""],
    ["힘 플랫폼 20 kg 셀", "–", "S-빔 50 kg + 단일점 30 kg + 기계식 과부하 스톱", "§6 걸림 하중"],
])

# ============================================================================= 10. plywood hybrid
# Chief request 2026-09-28: BOM (elec/v1l_bom.csv) over R8 → frame to plywood hybrid,
# purchase+shipping (unconditional) <= 880,000 KRW, tip X <= 1 mm @ 100 N (members + joints, mid assumption).
# Extra sources:
#  [S14] birch plywood 18 mm, E ≈ 7,930 MPa lengthwise (spec snippet)                 [SNIPPET]
#  [S15] EN 1995-1-1 (Eurocode 5) Table 7.1: K_ser = ρ_m^1.5·d/23 per shear plane per
#        dowel/bolt (dlubal knowledge base); ×2 for steel-to-timber (EN 1995-1-1 §7.1(3),
#        원문 확인 필요)                                                               [SNIPPET]
#  [S16] daelimwood.com: CP 구조용 내수·방수 합판 18T 1220×2440 28,500원                  [SNIPPET]
#  [S17] 합판 재단 1컷 1,000원 (국내 목재 재단 서비스 검색 요약)                           [SNIPPET]
p("## 10. 예산 맞춤 합판 하이브리드 (Chief 요청 2026-09-28)")
p("")
E_PLY = 5000.0      # MPa, in-plane E of 18T structural (CP pine) plywood — ASSUMPTION (below birch 7,930 [S14])
G_PLY = 400.0       # MPa, in-plane (panel) shear modulus — ASSUMPTION
KAPPA = 5.0 / 6.0   # Timoshenko shear coefficient, rectangle
T_PLY = 18.0        # mm
RHO_PLY = {"stiff": 550.0, "mid": 500.0, "soft": 450.0}   # kg/m^3 mean density — ASSUMPTION (CP ~500)
WOOD_FACTOR = {"stiff": 2.0, "mid": 1.0, "soft": 0.5}     # stiff: ×2 metal-to-timber [S15]; soft: workmanship/moisture — ASSUMPTION
K_SBR16 = 20e3      # N/mm per SBR16UU open bushing block — ASSUMPTION (BOM L13 replaced MGN12 on the bed)
K_XCELL = 50 * 9.81 / SP_DEFL   # BOM L30: 50 kg single-point bar cell mounted horizontally (Chief 절감 3), 0.3 mm @ rated — ASSUMPTION
K_DEF = (0.8, 1.0)  # creep factor plywood, service class 1 / 2 (EN 1995-1-1 Table 3.2 — 기억 기반, 원문 확인 필요)
SWELL_T = 0.003     # thickness swelling per 1 % moisture content change — ASSUMPTION (generic plywood)
SWELL_L = 0.0002    # in-plane swelling per 1 % MC — ASSUMPTION
H_OUT = 8.0         # W/m²K outer film coefficient of the holder shell — ASSUMPTION

H_SBR = 45.0        # SBR16UU total height (rail base → block top) — ASSUMPTION (catalogue class)
STACK_P = {"SBR16 rail+block": H_SBR, "bed deck 18T": 18.0, "force platform": 60.0, "holder floor": 56.0, "pan": PAN["H"]}
H_RIM_P = sum(STACK_P.values())            # pan rim above the base-plate top
W_PANEL = 500.0                            # side panel width along X (fits the one-sheet nesting)
H_PANEL = H_RIM_P + Z_U                    # panel height = top cross plate level
S_PANEL = 480.0                            # panel centre spacing (bed 400 wide + clearance)
SPAN_PLATE = S_PANEL - T_PLY               # clear span of the cross plates
B_PLATE = 270.0                            # cross-plate depth along X (drag direction) — design choice
RIB_H = 90.0                               # vertical front rib under each cross plate — design choice
WALL_BOTTOM = 60.0                         # back wall bottom above the pan rim (bed passes under)
N_BLK_BOLTS, D_BLK_BOLT = 6, 6.0           # Z block mount (Al angle) → cross plate, per level
N_END_BOLTS, D_END_BOLT = 4, 6.0           # H1 only: 4080 cross-beam end → panel, per end
LOOP_JOINTS = [("KFL08 고정단 → 베이스", 4, 5.0), ("X 너트 브래킷 → 베드 데크", 4, 5.0), ("X 셀 마운트 → 베드 데크", 4, 6.0)]
S_BLK_BED = 280.0


def k_ser(d, lab):
    """EN 1995-1-1 Table 7.1 [S15]: K_ser = ρ_m^1.5·d/23 [N/mm] per shear plane per bolt (snug hole)."""
    return RHO_PLY[lab] ** 1.5 * d / 23.0 * WOOD_FACTOR[lab]


def panel_defl(loads, W=W_PANEL, H=H_PANEL, n=4001):
    """In-plane plywood panel as a cantilever from the base (glued foot cleats): deflection at each load
    height by unit-load integration, bending + Timoshenko shear (numpy)."""
    z = np.linspace(0.0, H, n)
    EI = E_PLY * T_PLY * W ** 3 / 12.0
    GA = KAPPA * G_PLY * T_PLY * W
    M = sum(P * np.clip(zj - z, 0.0, None) for zj, P in loads)
    V = sum(P * (z < zj) for zj, P in loads)
    out = []
    for zi, _ in loads:
        m = np.clip(zi - z, 0.0, None)
        v = (z < zi).astype(float)
        out.append(float(np.trapezoid(M * m / EI, z) + np.trapezoid(V * v / GA, z)))
    return out


def plate_beam(R, span=SPAN_PLATE, b=B_PLATE, t=T_PLY):
    """Cross plate lying flat, bent in its own plane by a mid-span force R (simply supported, Timoshenko)."""
    I = t * b ** 3 / 12.0
    return R * span ** 3 / (48 * E_PLY * I) + R * span / (4 * KAPPA * G_PLY * t * b)


def tip_hybrid(var, F, c, jw="mid"):
    """Drag-direction tip displacement [mm] by element. var: 'orig' (current BOM: 4080 cross-beams,
    2040 A-frames, SBR16 bed, bar cell), 'H1' (plywood base/bed/panels/back wall, 4080 cross-beams kept),
    'H2' (H1 + plywood cross plates; 4080 only for the Z column). Metal head joints are added separately."""
    d = {}
    ls = STEM + cm.Z_C_LEVER
    d["stem"] = F * ls ** 3 / (3 * E_SS * STEM_I["30x3"])
    ov = Z_L - (c + STEM + HEAD)
    below = HEAD + STEM + cm.Z_C_LEVER
    d["Z column"] = cantilever_column(F, E_AL * PROF["4080"]["I_s"], ov, below)
    pz = c - cm.Z_C_LEVER
    arm = Z_L - pz
    R_u = F * arm / H_B
    R_l = F + R_u

    def two_level(dl, du_signed):
        return dl + (dl - du_signed) / H_B * arm

    d["Z guide"] = two_level(R_l / K_MGN12, -R_u / K_MGN12)
    if var in ("orig", "H1"):
        EI = E_AL * PROF["4080"]["I_s"]
        span = S_CB if var == "orig" else SPAN_PLATE
        d["cross-beams"] = two_level(R_l * span ** 3 / (48 * EI), -R_u * span ** 3 / (48 * EI))
    else:
        d["cross-beams"] = two_level(plate_beam(R_l), -plate_beam(R_u))
    if var == "orig":
        kt = tower_k("2040")
        d["towers"] = two_level(R_l / 2 / kt, -R_u / 2 / kt)
    else:
        zl, zu = H_RIM_P + Z_L, H_RIM_P + Z_U
        dl, du = panel_defl([(zl, R_l / 2), (zu, -R_u / 2)])
        d["towers"] = two_level(dl, du)
    d["X drive"] = F / screw_axial_k(X_SCREW_NUT_MAX)
    h_bed = (H_RIM_P if var != "orig" else H_RIM - BED_BLOCK_Z) + pz
    d["bed blocks"] = F * h_bed ** 2 / (2 * K_SBR16 * S_BLK_BED ** 2 / 2.0)
    d["X cell"] = F / K_XCELL
    wood = 0.0
    if var != "orig":
        wood += sum(F / (n * k_ser(dd, jw)) for _, n, dd in LOOP_JOINTS)
        if var == "H1":
            ke = N_END_BOLTS * k_ser(D_END_BOLT, jw)
            wood += two_level(R_l / 2 / ke, -R_u / 2 / ke)
        else:
            kb = N_BLK_BOLTS * k_ser(D_BLK_BOLT, jw)
            wood += two_level(R_l / kb, -R_u / kb)
    d["wood joints"] = wood
    return d


VARS = [("orig", "원안(현재 BOM): 4080 교차빔·기둥, 2040 A-프레임, SBR16, 바형 셀"),
        ("H1", "H1: 합판 베이스·베드·측판·뒷벽 + 4080 교차빔 유지"),
        ("H2", "**H2(추천)**: H1 + 합판 교차판(270×18 + 앞 립), 4080은 Z 기둥만")]
p("### 10.1 무엇을 합판으로 바꾸나")
p("")
table(["부위", "원안", "H2 (추천)", "나무 쪽 조건"], [
    ["베이스", "2040 틀 1000×560", f"18T 판 1000×560 + 아래 리브 18×80×1000 ×3(접착·나사), 리브 끝 고무 패드 4", "SBR16 장착면 심 조정(§10.4)"],
    ["베드", "2040 틀 + 2020 + 6T 데크", "18T 데크 650×400 (SBR16UU 4개 직접 볼트)", "인서트 너트, 방수 도장"],
    ["타워", "2040 경사 다리 4 + 각도 조인트 8", f"18T **직사각 측판 {W_PANEL:.0f}×{H_PANEL:.0f} ×2**(면내 강성), 발 클리트 접착·나사", "측판이 옆 가드 겸용"],
    ["교차빔", "4080 ×2", f"18T **교차판 {SPAN_PLATE:.0f}×{B_PLATE:.0f} ×2**(평평하게 — 굽힘 깊이를 X 방향으로) + 앞 립 18×{RIB_H:.0f}", "클리트로 측판에 **접착** + 나사"],
    ["옆힘 격막", "2020 무릎가새 ×2", f"18T **뒷벽 {SPAN_PLATE:.0f}×{H_PANEL - H_RIM_P - WALL_BOTTOM:.0f}**(테두리 위 {WALL_BOTTOM:.0f} mm부터, 베드는 그 아래로 통과)", "접착"],
    ["Z 블록 마운트", "4080에 T너트", f"알루미늄 앵글 40×40×4를 교차판 앞 가장자리에 M6 {N_BLK_BOLTS}개(+에폭시)", "구멍 여유 0"],
    ["Z 기둥", "4080 500", "**4080 500 유지**(MGN12 레일 장착, 이동 부재)", "–"],
])
p("- 교차빔을 합판 '상자보'로 세우지 않고 **평평하게 눕힌 판**으로 한 이유: 드래그 짝힘 R_l, R_u는 **수평(X)** 이다. 굽힘 깊이가 X 방향이어야 하므로 판을 눕혀 폭 270 mm를 깊이로 쓴다. "
  "수직 하중(Z 너트)은 앞 립이 받는다.")
p("")

p("### 10.2 스쿱 끝 X 변위 — 원안 vs 하이브리드 (C = −75)")
p("")
p(f"합판 E = {E_PLY:.0f} MPa, G = {G_PLY:.0f} MPa(면내, **가정**; 자작 18 mm 스니펫 7,930 MPa [S14]보다 낮게), 나무 체결 K_ser = ρ_m^1.5·d/23 [S15] "
  f"(중간: ρ {RHO_PLY['mid']:.0f}, ×1 / 단단: ρ {RHO_PLY['stiff']:.0f}, ×2 금속–나무 / 무름: ρ {RHO_PLY['soft']:.0f}, ×0.5). 금속 체결부(헤드 브래킷·스템 클램프)는 §2.2와 같은 k_j 범위.")
p("")
rows = []
hyb = {}
for var, lab in VARS:
    for F in (F_NOM, F_UP):
        base = tip_hybrid(var, F, C_MIN, "mid")
        mem = sum(v for k, v in base.items() if k != "wood joints")
        tot = {}
        for jw, jm in (("stiff", "stiff"), ("mid", "mid"), ("soft", "soft")):
            dw = tip_hybrid(var, F, C_MIN, jw)["wood joints"]
            tot[jw] = mem + dw + joint_dx(F, K_JOINT[jm])
        hyb[(var, round(F))] = (base, mem, tot)
        rows.append([lab if F == F_NOM else "", f"{F:.0f}", f2(mem), f2(base["wood joints"]), f2(joint_dx(F, K_JOINT["mid"])),
                     f"**{tot['mid']:.2f}** {'●' if tot['mid'] <= 1.0 else '✘'}", f2(tot["stiff"]), f"{tot['soft']:.2f} {'●' if tot['soft'] <= 1.0 else '✘'}"])
table(["구성", "F_x [N]", "부재·레일·구동·셀", "+ 나무 체결(중간)", "+ 금속 체결(중간)", "합계(중간)", "합계(단단)", "합계(무름)"], rows)
b2 = hyb[("H2", round(F_UP))][0]
table(["H2 @ F_d 요소", *b2.keys()], [["[mm]", *[f3(v) for v in b2.values()]]])
b0 = hyb[("orig", round(F_UP))][0]
p(f"- 측판(면내) 항 {b2['towers']:.3f} mm < 원안 2040 A-프레임 {b0['towers']:.3f} mm: 18T 판 {W_PANEL:.0f} mm 폭은 면내로 매우 강하다. "
  f"대신 교차판 항 {b2['cross-beams']:.3f} mm(전단 변형이 대부분) > 4080 {b0['cross-beams']:.3f} mm, 나무 체결 {b2['wood joints']:.3f} mm가 새로 생긴다.")
t2 = hyb[("H2", round(F_UP))][2]
t2n = hyb[("H2", round(F_NOM))][2]
p(f"- **판정**: H2는 F_d {F_UP:.0f} N에서 중간 가정 **{t2['mid']:.2f} mm ≤ 1 mm**(여유 작음), F_nom에서 {t2n['mid']:.2f} mm. 무른 가정이면 {t2['soft']:.2f} mm로 넘는다 → "
  "§2.5 보상 조건(히스테리시스 ≤ 0.10 mm 등)을 **조립 후 측정으로 통과해야** 허용. 나무 볼트 구멍에 여유가 있으면 미끄럼이 히스테리시스로 나타나 보상 조건을 깨므로, "
  "**구멍 = 볼트 지름(여유 0) 또는 에폭시 고정**이 전제다.")
kb6 = N_BLK_BOLTS * k_ser(D_BLK_BOLT, "mid")
p(f"- 여유를 늘리는 순서(비용 0에 가까움): (1) Z 블록 마운트 앵글을 에폭시 + 볼트로 붙임(볼트 {N_BLK_BOLTS}개 강성 {kb6:.0f} N/mm가 이 항을 정함), "
  "(2) 교차판·측판·뒷벽을 모두 접착(나사만이면 이음 미끄럼 항이 추가됨), (3) 헤드 브래킷 볼트 추가(§2.2 금속 항).")
p("")

p("### 10.3 옆힘(F_y)과 수직(깊이) — 나무 구조에서 새로 확인할 것")
p("")
Fy = K_SIDE * F_UP
h_low = H_RIM_P + WALL_BOTTOM
I_out = W_PANEL * T_PLY ** 3 / 12.0
k_sway_pin = 2 * 3 * E_PLY * I_out / h_low ** 3
k_sway_fix = 2 * 12 * E_PLY * I_out / h_low ** 3
k_sway_nowall = 2 * 3 * E_PLY * I_out / H_PANEL ** 3
side_core = side_LA(CAND["L-A"], Fy, C_MIN) - Fy / (2 * E_AL * PROF["2020"]["A"] * (140.0 / math.hypot(140.0, 235.0)) ** 2 / math.hypot(140.0, 235.0))
# vertical: upper cross plate + front rib (L-section) under the Z nut
A_f, A_w = B_PLATE * T_PLY, T_PLY * RIB_H
y_f, y_w = T_PLY / 2, T_PLY + RIB_H / 2
y_c = (A_f * y_f + A_w * y_w) / (A_f + A_w)
I_L = B_PLATE * T_PLY ** 3 / 12 + A_f * (y_c - y_f) ** 2 + T_PLY * RIB_H ** 3 / 12 + A_w * (y_w - y_c) ** 2
I_flat = B_PLATE * T_PLY ** 3 / 12
Fz = K_VERT * F_UP


def ss_mid(P, L, EI):
    return P * L ** 3 / (48 * EI)


d_nut_rib, d_nut_flat = ss_mid(Fz, SPAN_PLATE, E_PLY * I_L), ss_mid(Fz, SPAN_PLATE, E_PLY * I_flat)
d_dead_rib = ss_mid(W_Z, SPAN_PLATE, E_PLY * I_L)
# base plate + 3 ribs (π-section) sag under the bed at mid-span, supported on pads at the rib ends
BW, RIB, NRIB, L_PAD = 560.0, 80.0, 3, 900.0
A_p, A_r = BW * T_PLY, NRIB * T_PLY * RIB
y_p, y_r = T_PLY / 2, T_PLY + RIB / 2
y_b = (A_p * y_p + A_r * y_r) / (A_p + A_r)
I_base = BW * T_PLY ** 3 / 12 + A_p * (y_b - y_p) ** 2 + NRIB * T_PLY * RIB ** 3 / 12 + A_r * (y_r - y_b) ** 2
I_base0 = BW * T_PLY ** 3 / 12
W_bed = M_BED * 9.81
sag_rib, sag_flat = ss_mid(W_bed, L_PAD, E_PLY * I_base), ss_mid(W_bed, L_PAD, E_PLY * I_base0)
dsag_ice = ss_mid(M_ICE * 9.81, L_PAD, E_PLY * I_base)
# bed deck under F_z between blocks (strip 400 wide, span 300)
d_deck = ss_mid(Fz, 300.0, E_PLY * 400.0 * T_PLY ** 3 / 12)
table(["항목", "값", "식·비고"], [
    [f"옆 변위 @ F_y {Fy:.0f} N: 기둥·스템·Z 가이드", f"{side_core:.2f} mm", "§2.3과 같은 식"],
    ["  + 측판 흔들림(뒷벽 격막 있음, 발 핀/고정)", f"{Fy/k_sway_pin:.2f} / {Fy/k_sway_fix:.2f} mm", f"측판 면외 k = 2·c·E·I/h³, h = 테두리+{WALL_BOTTOM:.0f} = {h_low:.0f} mm, c = 3 / 12"],
    ["  (참고) 뒷벽 없이", f"{Fy/k_sway_nowall:.1f} mm", f"h = {H_PANEL:.0f} mm 외팔 → **뒷벽 필수**"],
    [f"Z 너트 수직(F_z {Fz:.0f} N): 교차판 + 앞 립", f"{d_nut_rib:.3f} mm", f"L자 단면 I = {I_L:.3g} mm⁴. 립 없으면 {d_nut_flat:.2f} mm → **앞 립 필수**"],
    [f"  자중(W_z {W_Z:.0f} N) 처짐의 크리프", f"{d_dead_rib*1e3:.1f} → {d_dead_rib*(1+K_DEF[0])*1e3:.1f}–{d_dead_rib*(1+K_DEF[1])*1e3:.1f} µm", "u_fin = u_inst·(1 + k_def), k_def 0.8–1.0(확인 필요). 매 스트로크 터치오프가 흡수"],
    [f"베이스 처짐(베드 {M_BED:.1f} kg 가운데, 패드 간격 {L_PAD:.0f})", f"{sag_rib:.3f} mm (리브 3) / {sag_flat:.2f} mm (판만)", f"**리브 필수**. 아이스크림 {M_ICE:.1f} kg 소진에 따른 변화 {dsag_ice:.3f} mm"],
    [f"베드 데크 처짐(F_z {Fz:.0f} N, 블록 사이 300)", f"{d_deck:.3f} mm", "F_z 측정값으로 보상 가능"],
])
T_AIR, RH = 22.0, 0.5
g = math.log(RH) + 17.62 * T_AIR / (243.12 + T_AIR)
T_DEW = 243.12 * g / (17.62 - g)                     # Magnus formula
U = 1.0 / (0.05 / 0.034 + 1.0 / H_OUT)
T_SHELL = T_AIR - U * (T_AIR - (-14.0)) / H_OUT
p("### 10.4 나무 위험과 대책")
p("")
table(["위험", "계산·근거", "대책"], [
    ["**결로·물방울**", f"실내 {T_AIR:.0f} °C / RH {RH*100:.0f} %의 이슬점 {T_DEW:.1f} °C (Magnus 식). 단열 홀더 외피 표면 ≈ {T_SHELL:.1f} °C → 외피는 결로 없음. "
     "**−14 °C 팬 테두리·뚜껑 안쪽·스쿱·스템·push-rod**는 이슬점 아래 → 성에·물방울이 베드 데크와 베이스 위로 떨어짐",
     "전 합판 수성 우레탄 2회(마구리 먼저, BOM L90), 홀더·컵 주변 데크에 물받이(알루미늄 테이프·PET 판), 시험 후 닦고 건조, 기계를 냉동고 옆 습한 곳에 두지 않음"],
    ["습기 팽창", f"함수율 3 %p 변화 가정: 두께 {18*SWELL_T*3:.2f} mm, 길이 1 m당 {1000*SWELL_L*3:.1f} mm (일반 계수 가정)",
     "고르게 부풀면 터치오프가 흡수. 국부 젖음(물방울)은 레일 높이를 바꾼다 → 도장·물받이, 계절마다 베드 높이 지도 재측정"],
    ["크리프", f"k_def 0.8–1.0(합판, 사용등급 1–2, 확인 필요): 자중 처짐 ×1.8–2.0", "드래그 하중은 수 초라 크리프 무관. 자중 처짐은 터치오프로 흡수. 장기 보관 시 베드를 가운데 두지 않음"],
    ["체결부 무름·미끄럼", f"K_ser(M6, 중간) = {k_ser(6, 'mid'):.0f} N/mm/볼트 — 금속–금속보다 무름. 구멍 여유는 그대로 미끄럼(EN 1995 볼트 구멍 여유 허용)",
     "구멍 = 볼트 지름(여유 0), 인서트 너트, 금속–나무는 에폭시 + 볼트, 나무–나무는 본드 + 나사"],
    ["평탄도(SBR16 장착면)", "구조용 합판은 휨·뒤틀림이 있음(수치 미확인). 레일은 지지형이라 면을 그대로 따름 → 베드 높이가 X에 따라 변하면 드래그 중 깊이가 변함",
     "베이스에 리브 접착 후 장착. 브리지에 다이얼 게이지를 달고 베드를 움직이며 **베드 상면 높이 흔들림 ≤ 0.1 mm**가 되게 SBR16 받침(150 mm 간격)에 심. 남는 값은 X–Z 지도로 G-code 보정(정적·반복 가능)"],
])

p("### 10.5 예산 — BOM 반영 결과 (`elec/v1l_bom.csv`, `mech/v1l_bom_plywood_update.py`)")
p("")
BOM_CSV = os.path.abspath(os.path.join(HERE, "..", "elec", "v1l_bom.csv"))
if os.path.exists(BOM_CSV):
    import csv as _csv
    with open(BOM_CSV, encoding="utf-8", newline="") as fh:
        bom = list(_csv.reader(fh))[1:]
    fr = [r for r in bom if r[1] == "Frame" and int(r[7] or 0) > 0]
    table(["ID", "품목", "수량", "합계 [원]", "가격 상태"], [[r[0], r[2], r[4], f"{int(r[7]):,}", r[10]] for r in fr] +
          [["", "**프레임 소계**", "", f"**{sum(int(r[7]) for r in fr):,}**", "원안 227,452"]] +
          [[r[0], r[2], r[4], f"{int(r[7]):,}", r[10]] for r in bom if r[0] in ("L63", "L64")])
    sums = {r[0]: int(r[7]) for r in bom if r[0].startswith("SUM")}
    table(["합계 행", "금액 [원]", "비고"], [[k, f"{v:,}", next(r[14] for r in bom if r[0] == k)] for k, v in sums.items()])
else:
    p("(BOM 파일 없음)")

p("### 10.6 합판 1장 재단도 (1220 × 2440, 톱날 3 mm)")
p("")
KERF = 3.0
BANDS = [  # (band length along the sheet, [(piece, width, length, qty)])
    (2 * W_PANEL + KERF, [("측판", H_PANEL, W_PANEL, 2), ("베이스 리브(옆 띠 281 폭)", 80.0, 1000.0, 3)]),
    (560.0, [("베이스 판", 1000.0, 560.0, 1), ("발 클리트(옆 띠)", 60.0, 560.0, 3)]),
    (H_PANEL - H_RIM_P - WALL_BOTTOM, [("뒷벽", SPAN_PLATE, H_PANEL - H_RIM_P - WALL_BOTTOM, 1), ("베드 데크", 650.0, 400.0, 1),
                                       ("앞 립(데크 아래 650×180)", SPAN_PLATE, RIB_H, 2), ("발 클리트(오른쪽 띠 108)", 60.0, 580.0, 1)]),
    (B_PLATE, [("교차판", SPAN_PLATE, B_PLATE, 2), ("교차판 클리트(옆 296×270)", 60.0, B_PLATE, 4)]),
]
used_L = sum(b[0] for b in BANDS) + KERF * (len(BANDS) - 1)
assert used_L <= 2440.0, "plywood nesting does not fit one sheet"
assert 1000.0 <= 2 * W_PANEL + KERF and H_PANEL + KERF + 3 * 80.0 + 2 * KERF <= 1220.0
assert SPAN_PLATE + KERF + 650.0 + KERF + 60.0 <= 1220.0 and 2 * SPAN_PLATE + KERF + 4 * 60.0 + 3 * KERF <= 1220.0
rows = []
area = 0.0
for L, pcs in BANDS:
    for name, w, l, q in pcs:
        rows.append([f"{L:.0f}", name, f"{w:.0f} × {l:.0f}", q])
        area += w * l * q / 1e6
table(["띠 길이 [mm]", "부품", "치수 [mm]", "개수"], rows)
p(f"- 띠 길이 합 {used_L:.0f} ≤ 2440 mm, 부품 면적 {area:.2f} m² / 판 2.98 m² ({area/2.977*100:.0f} %). **재단 실수 여유가 거의 없다** → 판매처 패널쏘 재단(직선 16컷, BOM L87). "
  "X 스톱 블록·팬 턱 받침은 자투리(데크 아래 188×180, 리브 옆 35×1000)와 6T(L08)로.")
p("- 측판이 옆 가드를 겸한다: 브리지 구간(X ±250) 옆은 18T 판이 막고, 베드–측판 틈은 "
  f"{(S_PANEL - T_PLY) / 2 - 200:.0f} mm. 정면은 아크릴 창 {SPAN_PLATE:.0f}×{H_PANEL - H_RIM_P - WALL_BOTTOM:.0f}(경첩) 1장 → 가드 아크릴 900×600 2장 → 1장.")
p("")

with open(OUT, "w", encoding="utf-8") as fh:
    fh.write("\n".join(lines) + "\n")
print("\n".join(lines))
