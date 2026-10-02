#!/usr/bin/env python3
"""V1-L (budget <= 1,000,000 KRW) mechanical sizing — RT1 revision (Red Team round 1, 2026-09-28).

Run:  python3 _chief/work/mech/v1l_mech_calc.py
Out:  stdout (markdown) + _chief/work/mech/v1l_mech_calc_output.md

Current design = H2R: moving bed (X, TR8 lead 4) + fixed plywood bridge (18T side panels,
flat cross plates, back wall) carrying Z (4080 column, TR8 lead 2) and theta (NEMA17 + 1:27).
The metal L-A frame of the first draft is kept only as "이전안" for comparison.

Sections
  0. decided values (DECISIONS.md 2026-09-28 + values fixed here)
  1. loads, stop levels, jam load range (holding torque x mu)
  2. candidate stiffness (이전안 metal L-A, L-A-lite, L-B, L-C) — repo model re-implemented + checked
  3. X drive at one fixed motor current; bed inertia on the X cell
  4. Z drive: stroke, rail/column length, self-locking, crush force
  5. theta drive
  6. H2R: joint details (angle vs clamp bracket), tip displacement, side/vertical, force platform
     stop cage, strength at the jam load, wood risks, plywood nesting (with in-band asserts),
     calibration load, C15 (GN 1/3 pan) check, budget read back from elec/v1l_bom.csv

All loads, stiffnesses, friction, masses and prices are ASSUMPTIONS unless a source tag says otherwise.
Why not `import scoop_load_path` / `capstone_alternatives`: both write repo files at import time.
Their formulas are copied and verified against calc/output/capstone_alternatives.md §2 (§2.0).
"""

import csv
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
BOM_CSV = os.path.abspath(os.path.join(HERE, "..", "elec", "v1l_bom.csv"))
lines = []
p = lines.append


def table(h, rows):
    p("| " + " | ".join(h) + " |")
    p("|" + "|".join("---" for _ in h) + "|")
    for r in rows:
        p("| " + " | ".join(str(c) for c in r) + " |")
    p("")


def f2(x):
    return f"{x:.2f}"


def f3(x):
    return f"{x:.3f}"


# =============================================================================
# SOURCES (tags as in research/components.md; "snippet" = search summary only)
#  [S1] Misumi HFS5-2040: Ix 1.358e4, Iy 5.13e4 mm^4, 0.88 kg/m                    snippet
#  [S2] TNUTZ EXM-2020: I 0.67 cm^4, A 1.638 cm^2                                  snippet
#  [S3] Misumi HFS8-4040 10.4 cm^4; 4080 19.8 / 71.9; 8080 129.1 (research/components.md)
#  [S4] StepperOnline 17HS19-1684S-PG27: 26.85:1, 3 N·m max permissible, 5 N·m momentary   snippet
#  [S5] StepperOnline 17HS19-2004S1: holding 59 N·cm @ 2.0 A                       snippet
#  [S6] Shigley's MED §8-2 power screws; Table 8-4 steel/bronze bearing pressure   textbook + snippet
#  [S7] THK A15-32 critical speed N1 = λ2·d_r/L²·1e7 (research/components.md)
#  [S8] GT2 6 mm working tension ~28 N (research/components.md)
#  [S9] MGN12H C0 5.88 kN, M0P 36 N·m (research/components.md)
#  [S10] S-type load cells: deflection at Emax 0.04-0.14 mm (generic)             snippet
#  [S11] Genmitsu CNC 3018-PRO: 300x180x45 mm, T8 lead 4, X rods Ø10              snippet
#  [S12] lead-screw friction coefficient 0.1-0.3                                   snippet
#  [S13] NEMA17 worm gearbox backlash 15-18 arcmin, self-locking conditional       snippet
#  [S14] birch plywood 18 mm E ≈ 7,930 MPa lengthwise                              snippet
#  [S15] EN 1995-1-1 Table 7.1: K_ser = ρ_m^1.5·d/23 per shear plane per bolt (dlubal KB);
#        ×2 metal-to-timber §7.1(3) (원문 확인 필요)                              snippet
#  [S16] daelimwood: CP 구조용 내수·방수 합판 18T 1220×2440 28,500원               snippet
#  [S17] 합판 재단 1컷 1,000원 (국내 목재 재단 서비스 검색 요약)                     snippet
#  [S18] Pololu DRV8825 carrier: ~1.5 A/phase without heat sink, 2.2 A with sufficient
#        cooling (0.5 W sense resistors); IC 2.5 A                                 snippet
#  [S19] Red Team B.0 (work/redteam_v1l.md): Johansen check of the M6 through-bolts ~2.0 kN/bolt
# =============================================================================

# ---------------------------------------------------------------- materials
E_AL, G_AL = 69e3, 26e3          # MPa (calc/scoop_load_path.py)
E_ST = 200e3                     # MPa, steel screw
E_SS = 193e3                     # MPa, 304 (repo)
SY_304 = 205.0                   # MPa, 304 annealed minimum yield (ASTM A240 min)
SY_AL = 110.0                    # MPa, 6063-T5 extrusion yield — ASSUMPTION (lower-bound typical)
RHO_ST = 7850.0                  # kg/m^3

PROF = {
    "2020": {"I_w": 0.67e4, "I_s": 0.67e4, "A": 163.8, "J": 0.25e4, "kg_m": 0.454},   # [S2]; J ASSUMPTION
    "2040": {"I_w": 1.358e4, "I_s": 5.13e4, "A": 326.0, "J": 0.8e4, "kg_m": 0.88},    # [S1]; A from mass; J ASSUMPTION
    "4040": {"I_w": 10.4e4, "I_s": 10.4e4, "A": 640.0, "J": 2.6e4, "kg_m": 1.73},     # [S3]
    "4080": {"I_w": 19.8e4, "I_s": 71.9e4, "A": 1109.0, "J": 6.0e4, "kg_m": 3.0},     # [S3]; J ASSUMPTION
    "8080": {"I_w": 129.1e4, "I_s": 129.1e4, "A": 1691.0, "J": 40.0e4, "kg_m": 4.57}, # [S3]; J ASSUMPTION
}


def round_tube_I(do, t):
    di = do - 2 * t
    return math.pi * (do ** 4 - di ** 4) / 64.0


STEM_I = {"30x3": round_tube_I(30, 3), "25x2": round_tube_I(25, 2), "25x3": round_tube_I(25, 3)}
STEM_C = {"30x3": 15.0, "25x2": 12.5, "25x3": 12.5}

# ---------------------------------------------------------------- guides / drives
K_MGN12 = 60e3        # N/mm per block — ASSUMPTION (catalogue value NOT FOUND)
K_SBR16 = 20e3        # N/mm per SBR16UU open bushing block — ASSUMPTION
TR8_D, TR8_P = 8.0, 2.0
TR8_DM = TR8_D - TR8_P / 2.0       # 7.0 mm
TR8_DR = TR8_D - TR8_P             # 6.0 mm — ASSUMPTION (≈ d − P)
ALPHA_THREAD = math.radians(15.0)
MU_CASES = (0.10, 0.15, 0.20)      # brass nut on steel — ASSUMPTION inside 0.1-0.3 [S12]
MU_NOM = 0.15                      # ASSUMPTION
K_NUT = 10e3          # N/mm, brass T8 nut axial — ASSUMPTION
K_SUPPORT = 5e3       # N/mm, KFL08 + collars, axial — ASSUMPTION
NUT_LEN = 15.0        # mm — ASSUMPTION
BACKLASH_NUT = 0.10   # mm — ASSUMPTION

# X motor 17HS19-2004S1 [S5] on DRV8825 [S18]
I_RATED_X = 2.0       # A [S5]
I_SET_X = 1.8         # A — DECISION (this revision): heat sink + fan (BOM L51), below the 2.2 A cooled limit [S18]
T_PULLOUT_RATED = 0.40  # N·m usable pull-out below the corner speed @ rated current, 24 V — ASSUMPTION
T_HOLD_RATED = 0.59   # N·m holding @ rated current [S5]
N_CORNER = 400.0      # rpm, constant power above — ASSUMPTION
RPM_CAP = 1500.0      # rpm — ASSUMPTION
J_ROTOR = 8.2e-6      # kg·m^2 — ASSUMPTION
T_DETENT = 0.01       # N·m — ASSUMPTION
SF_STEPPER = 2.0      # required torque margin (open loop) — design rule

# Z motor 17HS4401 class (BOM L15)
T_HOLD_Z = 0.40       # N·m holding (common listing 40 N·cm, 확인 필요) — ASSUMPTION
T_PULLOUT_Z = 0.28    # N·m usable pull-out below the corner speed — ASSUMPTION


def stepper_torque(n_rpm, t_lo):
    """ASSUMED 24 V pull-out curve: flat t_lo up to N_CORNER, then constant power."""
    n = max(n_rpm, 1.0)
    return t_lo if n <= N_CORNER else t_lo * N_CORNER / n


T_LO_X = T_PULLOUT_RATED * I_SET_X / I_RATED_X    # torque ∝ current — ASSUMPTION

# theta gearmotor (17HS19-1684S-PG27 class)
PG_RATIO, PG_T_CONT, PG_T_PEAK = 26.85, 3.0, 5.0   # [S4]
PG_ETA = 0.80         # ASSUMPTION
PG_T_LO = 0.35        # N·m motor side — ASSUMPTION
PG_BACKLASH_DEG = 1.0 # ASSUMPTION

# force platform (BOM L29, L30, L33) — DECISION: 20 kg vertical + 50 kg X single-point + leaf flexure
CELL_DEFL = 0.30      # mm at rated load, single-point bar cell — ASSUMPTION (datasheet 확인 필요)
V_CELL_KG, X_CELL_KG = 20.0, 50.0
SAFE_OVL = 1.5        # safe overload 150 % (BOM L29/L30 listing)
K_VCELL = V_CELL_KG * 9.81 / CELL_DEFL
K_XCELL = X_CELL_KG * 9.81 / CELL_DEFL
K_ROD_END = 20e3      # N/mm per steel M6 rod end in the X link — ASSUMPTION
K_SCREW_AL = 20e3     # N/mm per preloaded M5 steel screw in a tapped Al plate (shear) — ASSUMPTION
K_JOINT = {"soft": 2e7, "mid": 5e7, "stiff": 1e8}   # N·mm/rad, metal head joints — ASSUMPTION

# ---------------------------------------------------------------- decided values (DECISIONS.md 2026-09-28)
U_CASES = (40.0, 90.0, 160.0)     # kPa A02
K_SIDE, K_VERT = cm.K_SIDE, cm.K_VERT   # 0.3, 0.5 (A28)
F_TARGET = 82.5       # N — DECISION (depth adaptation, 0.55 × F_DESIGN 150 of ctrl)
F_HOST_STOP = 105.0   # N — DECISION
F_NANO_STOP = 120.0   # N — DECISION (= mechanical F_STOP)
F_STOP = F_NANO_STOP
SPS = 80              # HX711 — DECISION
V_DRAG = 30.0         # mm/s — DECISION
V_TRAVEL = 60.0       # mm/s — DECISION ($110)
A_X = 0.5             # m/s² — DECISION ($120 = 500 mm/s², one value per axis)
A_Z = 0.3             # m/s² ($122, ctrl)
F_SEAL_PER_BLOCK = 1.0   # N — ASSUMPTION
MU_RAIL = 0.01        # ASSUMPTION

# ---------------------------------------------------------------- geometry (A40, A41)
PAN = {"L": 360.0, "W": 165.0, "H": 120.0}   # A40
R = cm.R_SCOOP
ATT = math.radians(-cm.ATTACK_DEG)
STEM = 200.0
HEAD = cm.HEAD_LEN
Z_SAFE = cm.z_safe(cm.CUP_RIM_Z_RECESSED)    # 60
Z_TOP = 100.0
C_MIN = -PAN["H"] + cm.FLOOR_MARGIN + R      # -75
LANE = PAN["L"] - 2 * (R + cm.WALL_MARGIN)   # 270
Z_L = Z_TOP + STEM + HEAD + 20.0             # 420 lower Z block (A41 H_BLOCK)
H_B = 220.0                                  # block spacing = cross plate spacing
Z_U = Z_L + H_B
OVERTRAVEL = 10.0
BLOCK_L = 45.4        # MGN12H block length — ASSUMPTION (HIWIN class)
X_SCREW_L = 600.0
X_SCREW_NUT_MAX = 550.0
Z_SCREW_L = 300.0

# H2R stack above the base-plate top (mm) — DESIGN (RT1: bed spacer for X-motor clearance, 80 mm platform)
STACK_P = {"SBR16 rail+block": 45.0, "bed spacer (18T offcut)": 18.0, "bed deck 18T": 18.0,
           "force platform (spacer 10 + cell 22 + bar 10 + flexure 26 + top 12)": 80.0,
           "holder floor (XPS 50 + ply 6)": 56.0, "pan": PAN["H"]}
H_RIM_P = sum(STACK_P.values())
BED_BLOCK_Z_P = 22.0
X_AXIS_H = 30.0       # X screw axis above the base-plate top (KFL08 on 10T upright) — DESIGN
H_RIM_METAL, BED_BLOCK_Z_METAL = 315.0, 46.5  # 이전안 metal stack (first draft)

# masses [kg]
M_ICE = PAN["L"] * PAN["W"] * PAN["H"] * 1e-9 * 0.8 * 650.0     # 80 % full, ρ 0.65 (A07)
MASS_BED = {"ice cream (80 % full)": M_ICE, "pan": 1.5, "insulated holder": 1.0,
            "platform (plates, cells, flexure, stops)": 1.8, "bed deck + spacers": 2.6,
            "blocks + nut + cup station": 0.8}
M_TOP = M_ICE + 1.5 + 1.0 + 1.0              # on the X cell (pan, holder, top plate) — ASSUMPTION
M_BED = sum(MASS_BED.values())
MASS_Z = {"4080 column 0.55 m": 0.55 * PROF["4080"]["kg_m"], "MGN12 rail 500": 0.33,
          "head bracket + clamps + plates": 0.8, "theta PG27 motor": 0.65,
          "food module": 0.6, "Z motor + screw + KFL08 on column": 0.55}
M_Z = sum(MASS_Z.values())
W_Z = M_Z * 9.81


def depth_for_portion(travel):
    for d10 in range(20, 300):
        d = d10 / 10.0
        v, a = tlp.stroke_volume(d, travel)
        if v >= tlp.V_TARGET:
            return d, a
    return None, None


def screw_eff(lead, mu):
    """Shigley §8-2 [S6]: efficiency driving against the load (thrust bearing friction ignored)."""
    dm, sec = TR8_DM, 1.0 / math.cos(ALPHA_THREAD)
    t_r = 0.5 * dm * (lead + math.pi * mu * dm * sec) / (math.pi * dm - mu * lead * sec)
    return lead / (2 * math.pi * t_r)


def screw_torque_lower(F, lead, mu):
    """Shigley §8-2 [S6]: T_L < 0 → the load back-drives the screw."""
    dm, sec = TR8_DM, 1.0 / math.cos(ALPHA_THREAD)
    return F * dm / 2 * (math.pi * mu * dm * sec - lead) / (math.pi * dm + mu * lead * sec) / 1e3


def mu_min_selflock(lead):
    lam = math.atan(lead / (math.pi * TR8_DM))
    return math.tan(lam) * math.cos(ALPHA_THREAD)


def thrust(T, lead, mu):
    return 2 * math.pi * screw_eff(lead, mu) * T / (lead / 1e3)


D_PORTION, A_PORTION = depth_for_portion(LANE)
H_C = R * math.cos(ATT) - D_PORTION
Z_CENT = sm.segment_centroid_below_centre_mm(ATT, H_C)
F_NOM = 90.0 * 1e-3 * A_PORTION
F_D = 160.0 * 1e-3 * A_PORTION
LEAD_X, LEAD_Z = 4.0, 2.0
JAM = {}
for lab, T in (("풀아웃 0.40 N·m @ 2.0 A (초판 가정)", T_PULLOUT_RATED), ("정지 토크 0.59 N·m @ 2.0 A [S5]", T_HOLD_RATED),
               (f"정지 토크 @ {I_SET_X} A (설정 전류)", T_HOLD_RATED * I_SET_X / I_RATED_X),
               ("정지 토크 @ 1.5 A", T_HOLD_RATED * 1.5 / I_RATED_X)):
    JAM[lab] = [thrust(T, LEAD_X, mu) for mu in MU_CASES]
F_JAM_MAX = thrust(T_HOLD_RATED, LEAD_X, 0.10)
F_JAM_MIN = thrust(T_PULLOUT_RATED, LEAD_X, 0.20)
F_JAM_SET = thrust(T_HOLD_RATED * I_SET_X / I_RATED_X, LEAD_X, 0.10)

# ============================================================================= 0. decided values
p("# V1-L 기계 계산 — generated (RT1 개정)")
p("")
p("> `python3 _chief/work/mech/v1l_mech_calc.py`. **모든 하중·강성·마찰·질량·가격은 ASSUMPTION**(출처 태그 붙은 값 제외). "
  "현재안은 **H2R**(이동 베드 + 합판 브리지, RT1 개정). 금속 L-A 프레임은 **이전안**으로 비교에만 쓴다.")
p("")
p("## 0. 결정값 (DECISIONS.md 2026-09-28 + 이 계산)")
p("")
table(["항목", "값", "출처"], [
    ["F_nominal (u 90 kPa, 1 portion)", f"{F_NOM:.0f} N", "§1"],
    ["F_d (u 160 kPa, 1 portion) — 강성·추력·θ 설계 하중", f"{F_D:.0f} N", "§1"],
    ["F_TARGET (깊이 적응)", f"{F_TARGET} N", "결정"],
    ["호스트 정지 / Nano 정지(= F_STOP)", f"{F_HOST_STOP:.0f} / {F_NANO_STOP:.0f} N", "결정"],
    ["F_JAM (X 걸림 추력, 강도·스톱 검토)", f"{F_JAM_MIN:.0f}–{F_JAM_MAX:.0f} N (상한 = 정지 토크 @ 2.0 A, μ 0.10)", "§1"],
    ["X 모터 전류", f"{I_SET_X} A (DRV8825 + 방열판·팬; 한계 1.5 A 무방열 / 2.2 A 냉각 [S18], 클론 검출저항 확인 필요)", "§3"],
    ["HX711", f"{SPS} SPS", "결정"],
    ["드래그 / 이송", f"{V_DRAG:.0f} / {V_TRAVEL:.0f} mm/s", "결정"],
    ["X / Z 가속", f"{A_X*1e3:.0f} / {A_Z*1e3:.0f} mm/s² (축당 하나, $120/$122)", "결정·제어"],
    ["힘 플랫폼", "수직 20 kg 단일점(L29) + X 50 kg 단일점(L30) + 판스프링 플렉서(L33) + 데크 접지 스톱 케이지", "결정·§6.5"],
    ["X / Z 리드", "TR8 리드 4 / 리드 2(자립)", "결정"],
    ["프레임", "H2R 합판 하이브리드(18T 1장), 4080은 Z 기둥 0.55 m만", "결정·§6"],
    ["가드", "아크릴 3T 정면 창 1장(470×520 재단), 옆은 합판 측판", "§6"],
    ["예비 스쿱", "없음(C9)", "BOM"],
    ["Z_SAFE / Z_TOP", f"{Z_SAFE:.0f} / {Z_TOP:.0f} mm (팬 테두리 기준)", "A31·A41"],
])

# ============================================================================= 1. loads
p("## 1. 하중 — 젤라토 팬 360×165×120, 레인 270 mm")
p("")
rows = []
for u in U_CASES:
    F = u * 1e-3 * A_PORTION
    rows.append([f"{u:.0f}", f"{D_PORTION:.1f}", f"{A_PORTION:.0f}", f"{F:.0f}", f"{K_VERT*F:.0f}", f"{K_SIDE*F:.0f}",
                 f2(F * Z_CENT / 1e3), f2(F * R / 1e3)])
table(["u [kPa] (A02)", "깊이 d [mm]", "A [mm²]", "F_x [N]", "F_z = 0.5F_x", "F_y = 0.3F_x",
       f"τ_hold = F·z_c (z_c {Z_CENT:.1f}) [N·m]", "τ_close = F·R (A05) [N·m]"], rows)
p(f"- 1 portion 깊이 {D_PORTION:.1f} mm, 절삭단면 {A_PORTION:.0f} mm²(`tub_lane_planner.stroke_volume`), z_c는 `scoop_model`로 계산. "
  f"속도 효과 (v/80)ⁿ(A03)는 30 mm/s에서 ×{(30/80)**0.15:.2f} → 보수적으로 무시.")
p(f"- u 160 kPa에서 1 portion 힘 {F_D:.0f} N > F_TARGET {F_TARGET} N → 단단한 제품은 깊이 적응으로 **2스트로크**가 된다(구조 하중은 올리지 않음).")
p("")
p("**걸림 하중 F_JAM = 2π·η·T/l** (TR8 리드 4, Shigley 효율 [S6]):")
p("")
table(["모터 토크 가정", *[f"μ {m}" for m in MU_CASES]], [[k, *[f"{v:.0f} N" for v in vs]] for k, vs in JAM.items()])
p(f"- 초판의 'F_JAM 329 N = 스톨 상한'은 풀아웃·μ 0.15 한 점이었다(Red Team M1). **강도·스톱은 상한 {F_JAM_MAX:.0f} N**(정지 토크 @ 정격 2.0 A, μ 0.10)으로 본다. "
  f"설정 전류 {I_SET_X} A에서는 {F_JAM_SET:.0f} N이지만, 전류 설정 실수에 대비해 정격 기준을 쓴다.")
p("")

# ============================================================================= 2. candidate stiffness
p("## 2. 스쿱 끝 강성 — 후보 비교 (초판 판정의 근거, 금속 L-A는 이전안)")
p("")
p("### 2.0 기존 모델 재현 확인")
p("")
REPO_CFG = {
    "L": {"col_EI": E_AL * 10.4e4, "beam_EI": E_AL * 71.9e4, "block_k": 60e3, "block_s": 60.0, "belt_ksp": 18.9e3, "stem": "25x3"},
    "M": {"col_EI": E_AL * 71.9e4, "beam_EI": E_AL * 129.1e4, "block_k": 200e3, "block_s": 150.0, "belt_ksp": 150e3, "stem": "25x3"},
}


def repo_tip_dx(cfg, F, c, stem, h_block, beam_span=1450.0):
    """calc/scoop_load_path.tip_dx (arch 1) with explicit stem / H_BLOCK."""
    ls = stem + cm.Z_C_LEVER
    out = {"stem": F * ls ** 3 / (3 * E_SS * STEM_I[cfg["stem"]])}
    ov = h_block - (c + stem + cm.HEAD_LEN)
    below = cm.HEAD_LEN + stem + cm.Z_C_LEVER
    lever = h_block - (c - cm.Z_C_LEVER)
    EI = cfg["col_EI"]
    M = F * below
    out["column"] = F * ov ** 3 / (3 * EI) + M * ov ** 2 / (2 * EI) + (F * ov ** 2 / (2 * EI) + M * ov / EI) * below
    K_rot = cfg["block_k"] * cfg["block_s"] ** 2 / 2.0
    out["Z blocks"] = F * lever ** 2 / K_rot
    out["X blocks"] = F * lever ** 2 / K_rot
    out["beam"] = F * lever * beam_span / (12 * cfg["beam_EI"]) * lever
    out["X drive"] = F / (cfg["belt_ksp"] * (1 / 560.0 + 1 / 590.0))
    return out, lever


PUBLISHED = {("L", 50): 1.36, ("L", 80): 2.17, ("L", 110): 2.99, ("M", 50): 0.23, ("M", 80): 0.38, ("M", 110): 0.52}
rows, ok_all = [], True
for (key, F), pub in PUBLISHED.items():
    d, lev = repo_tip_dx(REPO_CFG[key], F, C_MIN, STEM, Z_L)
    tot = sum(d.values())
    ok_all &= abs(tot - pub) <= 0.01
    rows.append([key, F, f2(tot), f2(pub), "●" if abs(tot - pub) <= 0.01 else "✘"])
table(["repo 구성", "F_x [N]", "이 스크립트 [mm]", "capstone_alternatives.md §2 [mm]", "일치(±0.01)"], rows)
assert ok_all, "repo stiffness model not reproduced"


def cantilever_column(F, EI, ov, below):
    M = F * below
    return F * ov ** 3 / (3 * EI) + M * ov ** 2 / (2 * EI) + (F * ov ** 2 / (2 * EI) + M * ov / EI) * below


def screw_axial_k(length_to_bearing):
    k_s = E_ST * math.pi * TR8_DR ** 2 / 4.0 / length_to_bearing
    return 1.0 / (1.0 / k_s + 1.0 / K_NUT + 1.0 / K_SUPPORT)


def tower_k(leg, h):
    """A-frame tower (metal 이전안): two inclined legs, feet at ±450 (pin-jointed truss)."""
    L = math.hypot(450.0, h)
    return 2 * E_AL * PROF[leg]["A"] * (450.0 / L) ** 2 / L


def joint_dx(F, kj):
    """metal head joints: head bracket-to-column (arm head+stem+z_c) + stem clamp (arm stem+z_c)."""
    below = HEAD + STEM + cm.Z_C_LEVER
    ls = STEM + cm.Z_C_LEVER
    return F * below ** 2 / kj + F * ls ** 2 / kj


def two_level(dl, du_signed, arm, hb=H_B):
    """tip displacement from translations at the lower / upper guide level (upper signed, + = same sense)."""
    return dl + (dl - du_signed) / hb * arm


def common_head(F, c, col="4080"):
    ls = STEM + cm.Z_C_LEVER
    d = {"stem": F * ls ** 3 / (3 * E_SS * STEM_I["30x3"])}
    ov = Z_L - (c + STEM + HEAD)
    d["Z column"] = cantilever_column(F, E_AL * PROF[col]["I_s"], ov, HEAD + STEM + cm.Z_C_LEVER)
    arm = Z_L - (c - cm.Z_C_LEVER)
    R_u = F * arm / H_B
    R_l = F + R_u
    d["Z guide"] = two_level(R_l / K_MGN12, -R_u / K_MGN12, arm)
    return d, arm, R_l, R_u


def tip_metal(F, c, col="4080", cb="4080", leg="2040"):
    """이전안 metal L-A with the current sensors (X 50 kg single-point cell, SBR16 bed)."""
    d, arm, R_l, R_u = common_head(F, c, col)
    EI = E_AL * PROF[cb]["I_s"]
    d["cross-beams"] = two_level(R_l * 480.0 ** 3 / (48 * EI), -R_u * 480.0 ** 3 / (48 * EI), arm)
    kt = tower_k(leg, H_RIM_METAL + Z_U)
    d["towers"] = two_level(R_l / 2 / kt, -R_u / 2 / kt, arm)
    d["X drive"] = F / screw_axial_k(X_SCREW_NUT_MAX)
    h_bed = H_RIM_METAL - BED_BLOCK_Z_METAL + c - cm.Z_C_LEVER
    d["bed blocks"] = F * h_bed ** 2 / (K_SBR16 * 280.0 ** 2)
    d["X cell"] = F / K_XCELL
    return d


def tip_LB(F, c, xbeam="4080"):
    d, arm, R_l, R_u = common_head(F, c)
    s_z, s_x, span = 150.0, 100.0, 850.0
    arm_b = Z_L - (c - cm.Z_C_LEVER)
    R_u2 = F * arm_b / s_z
    d["Z guide"] = two_level((F + R_u2) / K_MGN12, -R_u2 / K_MGN12, arm_b, s_z)
    lever_x = Z_L + s_z / 2 - (c - cm.Z_C_LEVER)
    d["X carriage"] = F * lever_x ** 2 / (K_MGN12 * s_x ** 2)
    d["X beam"] = F * lever_x * span / (12 * E_AL * PROF[xbeam]["I_s"]) * lever_x
    d["towers"] = F / tower_k("2040", H_RIM_METAL + Z_U)
    d["X drive"] = F / screw_axial_k(700.0)
    d["X cell"] = F / K_XCELL
    return d


def side_LB(Fy, c, xbeam="4080"):
    ls = STEM + cm.Z_C_LEVER
    d = Fy * ls ** 3 / (3 * E_SS * STEM_I["30x3"])
    d += cantilever_column(Fy, E_AL * PROF["4080"]["I_w"], Z_L - (c + STEM + HEAD), HEAD + STEM + cm.Z_C_LEVER)
    lever_x = Z_L + 75.0 - (c - cm.Z_C_LEVER)
    T = Fy * lever_x
    d += T * 850.0 / (4 * G_AL * PROF[xbeam]["J"]) * lever_x
    d += Fy * 850.0 ** 3 / (48 * E_AL * PROF[xbeam]["I_w"])
    d += T / (4 * K_MGN12 * 20.0 ** 2) * lever_x
    return d


rows = []
for lab, fn in (("이전안 L-A 금속(4080 교차빔·기둥, 2040 A-프레임)", lambda F: tip_metal(F, C_MIN)),
                ("L-A 경량(4040, 2020 다리)", lambda F: tip_metal(F, C_MIN, "4040", "4040", "2020")),
                ("L-B 이동 갠트리(4080 X빔)", lambda F: tip_LB(F, C_MIN))):
    for F in (F_NOM, F_D):
        d = fn(F)
        tot = sum(d.values())
        rows.append([lab if F == F_NOM else "", f"{F:.0f}", f2(tot), f2(tot + joint_dx(F, K_JOINT["mid"])),
                     f2(tot + joint_dx(F, K_JOINT["soft"]))])
table(["후보(현재 센서: X 50 kg 단일점, SBR16)", "F_x [N]", "부재·레일·구동 [mm]", "+ 금속 체결 중간", "+ 금속 체결 무름"], rows)
d_Lc, _ = repo_tip_dx(REPO_CFG["L"], F_D, C_MIN, STEM, Z_L)
p(f"- L-C(키트·중고 프린터): 가장 유리한 'L 3D-printer-like' 모델로도 {sum(d_Lc.values()):.2f} mm @ {F_D:.0f} N(벨트 {d_Lc['X drive']:.2f}), "
  f"GT2 작업장력 ~28 N [S8] < {F_NOM:.0f} N → 기각. 3018 Z 행정 45 mm [S11] < 필요 195 mm.")
p(f"- 옆힘(F_y {K_SIDE*F_D:.0f} N): L-B 4080 X빔 비틀림 {side_LB(K_SIDE*F_D, C_MIN):.2f} mm, 8080이면 {side_LB(K_SIDE*F_D, C_MIN, '8080'):.2f} mm. "
  "L-A는 옆힘이 브리지 면 안이라 작다(§6.4).")
p("")

# ============================================================================= 3. X drive
p("## 3. X 구동 — 전류 하나로 정하고 여유 재계산")
p("")
f_fric = MU_RAIL * M_BED * 9.81 + 4 * F_SEAL_PER_BLOCK
J_screw = math.pi * RHO_ST * (X_SCREW_L / 1e3) * (TR8_D / 1e3) ** 4 / 32.0


def x_margin(I, v, mu, phase="steady", lead=LEAD_X):
    """steady drag at F_STOP (no acceleration) or drag-start acceleration at F_TARGET."""
    n = v / lead * 60.0
    t_av = stepper_torque(n, T_PULLOUT_RATED * I / I_RATED_X)
    if phase == "steady":
        F = F_STOP + f_fric
        t_req = F * (lead / 1e3) / (2 * math.pi * screw_eff(lead, mu))
    else:
        F = F_TARGET + f_fric + M_BED * A_X
        t_req = F * (lead / 1e3) / (2 * math.pi * screw_eff(lead, mu)) + (J_ROTOR + J_screw) * A_X * 2 * math.pi / (lead / 1e3)
    return t_av / t_req, t_req, t_av


p(f"가정: 풀아웃 {T_PULLOUT_RATED} N·m @ {I_RATED_X} A(평탄, {N_CORNER:.0f} rpm 이후 일정 출력), **토크 ∝ 전류**. 정속 드래그 요구 추력 = F_STOP {F_STOP:.0f} + 마찰 {f_fric:.1f} N "
  f"(정속 구간엔 가속 없음). 드래그 시작 가속 구간 = F_TARGET {F_TARGET} + 마찰 + m·a({M_BED:.1f} kg × {A_X*1e3:.0f} mm/s²) + 회전관성. 요구 여유 ≥ {SF_STEPPER}.")
p("")
rows = []
for I in (1.5, 1.8, 2.0):
    cells = [f"{I}"]
    for v, mu in ((30, 0.15), (30, 0.20), (25, 0.20), (20, 0.20)):
        m, _, _ = x_margin(I, v, mu)
        cells.append(f"{m:.2f} {'●' if m >= SF_STEPPER else '✘'}")
    ma, _, _ = x_margin(I, V_DRAG, 0.20, "accel")
    cells.append(f"{ma:.2f}")
    rows.append(cells)
table(["X 전류 [A]", "30 mm/s μ 0.15", "30 mm/s μ 0.20", "25 mm/s μ 0.20", "20 mm/s μ 0.20", "가속 구간(30 mm/s, μ 0.20)"], rows)
m_sel, treq_sel, tav_sel = x_margin(I_SET_X, V_DRAG, MU_NOM)
m_sel20, _, _ = x_margin(I_SET_X, V_DRAG, 0.20)
m_25, _, _ = x_margin(I_SET_X, 25.0, 0.20)
p(f"- **결정: X {I_SET_X} A**(DRV8825 냉각 한계 2.2 A [S18]의 {I_SET_X/2.2*100:.0f} %, 방열판 + 팬 L51). 드래그 30 mm/s, μ 0.15에서 여유 **{m_sel:.2f}** "
  f"(T_req {treq_sel:.3f} / T_avail {tav_sel:.3f} N·m). 1.5 A면 {x_margin(1.5, 30, 0.15)[0]:.2f}로 요구 미달이라 쓰지 않는다.")
p(f"- μ 0.20(너트 마찰 큼)이면 30 mm/s에서 {m_sel20:.2f} → **T1 시험에서 무부하 나사 토크로 μ를 재서 μ ≥ 0.18이면 드래그 25 mm/s**(여유 {m_25:.2f})로 내린다. "
  "F_STOP은 결정값(120 N) 유지.")
p(f"- 이송 {V_TRAVEL:.0f} mm/s = {V_TRAVEL/LEAD_X*60:.0f} rpm, 가속 {A_X*1e3:.0f} mm/s²: 여유 "
  f"{stepper_torque(V_TRAVEL/LEAD_X*60, T_LO_X) / ((M_BED*A_X + f_fric)*(LEAD_X/1e3)/(2*math.pi*screw_eff(LEAD_X, 0.20)) + (J_ROTOR+J_screw)*A_X*2*math.pi/(LEAD_X/1e3)):.1f} (μ 0.20).")
p("")
rows = []
n_crit = 9.7 * TR8_DR / X_SCREW_L ** 2 * 1e7
A_bear = math.pi * TR8_DM * (TR8_P / 2) * (NUT_LEN / TR8_P)
for lead in (2.0, 4.0, 8.0):
    m30, _, _ = x_margin(I_SET_X, 30.0, MU_NOM, lead=lead)
    fpm = math.pi * TR8_DM * (30.0 / lead * 60.0) / math.cos(math.atan(lead / (math.pi * TR8_DM))) / 1e3 / 0.3048
    rows.append([f"{lead:.0f}", f"{m30:.2f}", f"{mu_min_selflock(lead):.3f}", f"{fpm:.0f}",
                 f"{thrust(T_HOLD_RATED, lead, 0.10):.0f}"])
table(["리드", f"여유 @ 30 mm/s, {I_SET_X} A, μ 0.15", "자립 μ_min", "너트 미끄럼 @30 mm/s [fpm]", "걸림 상한(정지 토크, μ 0.10) [N]"], rows)
p(f"- 리드 4 유지. 너트 면압 {(F_STOP+f_fric)/A_bear:.2f} MPa(Shigley 표 8-4 20–40 fpm 구간 하한 5.5 MPa 대비 여유 [S6]), 임계속도 {n_crit:.0f} rpm(양끝 지지 [S7]).")
p("")
p("### 3.1 베드 가속이 X 셀에 주는 관성 성분")
p("")
f_n = math.sqrt(K_XCELL * 1e3 / M_TOP) / (2 * math.pi)
table(["구간", "a [mm/s²]", f"m_top·a (m_top {M_TOP:.1f} kg) [N]", "F_nom 대비"],
      [["드래그 시작·끝 / 이송 (축당 가속 하나)", f"{A_X*1e3:.0f}", f"{M_TOP*A_X:.1f}", f"{M_TOP*A_X/F_NOM*100:.1f} %"]])
p(f"- X 50 kg 단일점 셀(정격 처짐 {CELL_DEFL} mm 가정, k ≈ {K_XCELL:.0f} N/mm) 위 {M_TOP:.1f} kg의 고유진동수 ≈ **{f_n:.0f} Hz**(초판 111 Hz는 S-빔 기준). "
  f"HX711 {SPS} SPS(나이퀴스트 {SPS/2:.0f} Hz)보다 높다 → 제어 §7 방식(가속 시작 후 3 mm ~ 감속 전 구간 평균, 과부하 임계 + m·a_max)으로 처리, T6 시험으로 맥놀이 확인.")
p("")

# ============================================================================= 4. Z drive
p("## 4. Z 구동 — 행정, 레일·기둥 길이, 자립, 하강력")
p("")
stroke = Z_TOP - C_MIN
z_cb_lo, z_cb_hi = C_MIN - OVERTRAVEL + STEM + HEAD, Z_TOP + OVERTRAVEL + STEM + HEAD   # column-bottom heights
rail_from = (Z_L - BLOCK_L / 2) - z_cb_hi          # in column coordinates (0 = old column bottom)
rail_to = (Z_U + BLOCK_L / 2) - z_cb_lo
RAIL_Z, COL_EXT, COL_Z = 500.0, 50.0, 550.0
assert -COL_EXT <= rail_from and rail_to <= RAIL_Z - COL_EXT, "Z rail does not keep both blocks engaged"
assert (Z_U + BLOCK_L / 2) - z_cb_lo <= COL_Z - COL_EXT
p(f"- 행정 C +{Z_TOP:.0f} … {C_MIN:.0f} = {stroke:.0f} mm, 원점·리밋 여유 ±{OVERTRAVEL:.0f} → {stroke+2*OVERTRAVEL:.0f} mm.")
p(f"- **레일·기둥 길이(RT1에서 새로 확인)**: 블록(길이 {BLOCK_L}) 중심 z_l {Z_L:.0f} / z_u {Z_U:.0f}. 기둥 좌표(초판 기둥 끝 = 0)에서 레일이 덮어야 할 범위는 "
  f"{rail_from:.0f} … {rail_to:.0f} mm. 초판(레일 450을 기둥 끝에 맞춤)은 C_max + 여유에서 하부 블록이 레일 끝을 {-rail_from:.0f} mm 벗어났다. "
  f"→ **기둥 {COL_Z:.0f}(헤드 브래킷 안으로 {COL_EXT:.0f} 연장), 레일 {RAIL_Z:.0f}**(기둥 끝에 맞춤, −{COL_EXT:.0f} … {RAIL_Z-COL_EXT:.0f}).")
p(f"- Z 가동 질량 {M_Z:.1f} kg → W_z {W_Z:.0f} N.")
p("")
rows = []
for lead in (2.0, 4.0, 8.0):
    cells = [f"{lead:.0f}", f"{mu_min_selflock(lead):.3f}"]
    for mu in MU_CASES:
        tl = screw_torque_lower(W_Z, lead, mu)
        cells.append("자립" if tl >= 0 else (f"역구동 {-tl*1e3:.1f} mN·m → " + ("턱토크로 겨우" if -tl < T_DETENT else "**낙하**")))
    cells.append(f"{thrust(T_HOLD_Z, lead, 0.10) + W_Z:.0f}")
    rows.append(cells)
table(["리드", "μ_min", "정전 μ 0.10", "μ 0.15", "μ 0.20", "최대 하강력(정지 토크 0.40 N·m, μ 0.10) + 자중 [N]"], rows)
t_hold_req = (K_VERT * F_STOP + W_Z) * (LEAD_Z / 1e3) / (2 * math.pi * screw_eff(LEAD_Z, 0.20))
n_z = 30.0 / LEAD_Z * 60
t_lift_req = (W_Z + M_Z * A_Z) * (LEAD_Z / 1e3) / (2 * math.pi * screw_eff(LEAD_Z, 0.20))
p(f"- 드래그 중 Z는 멈춰 있다 → 0.5·F_STOP 위 + 자중 {K_VERT*F_STOP+W_Z:.0f} N 유지에 {t_hold_req:.3f} N·m(정지 토크 {T_HOLD_Z} 대비 {T_HOLD_Z/t_hold_req:.1f}배). "
  f"상승 30 mm/s({n_z:.0f} rpm)에서 {t_lift_req:.3f} N·m vs 가정 {stepper_torque(n_z, T_PULLOUT_Z):.3f} → {stepper_torque(n_z, T_PULLOUT_Z)/t_lift_req:.1f}배.")
p(f"- **리드 2 유지**: μ ≥ {mu_min_selflock(2.0):.3f}이면 자립 → 브레이크·가스스프링 없음(정전 유지 시험 조건). 최대 하강력 ≈ {thrust(T_HOLD_Z, 2.0, 0.10)+W_Z:.0f} N "
  f"→ 수직 셀(안전 과부하 {V_CELL_KG*9.81*SAFE_OVL:.0f} N)보다 크다 → **수직 스톱이 주 보호**(§6.5), Z 전류 제한, hold-to-run.")
p("")

# ============================================================================= 5. theta
p("## 5. θ 구동")
p("")
rows = []
for lab, F in (("F_nom", F_NOM), ("F_TARGET", F_TARGET), ("F_d", F_D), ("F_STOP", F_STOP)):
    tc = F * R / 1e3
    rows.append([lab, f"{F:g}" if lab == "F_TARGET" else f"{F:.0f}", f2(F * Z_CENT / 1e3), f2(tc), "●" if tc <= PG_T_CONT else ("△ 순간 정격 안" if tc <= PG_T_PEAK else "✘"), f"{tc/0.025:.0f}"])
table(["수준", "F_x [N]", "τ_hold [N·m]", "τ_close [N·m]", f"PG27 {PG_T_CONT:.0f} / {PG_T_PEAK:.0f} N·m [S4]", "이중 push-rod 로드력 τ/l [N]"], rows)
Pcr = math.pi ** 2 * E_SS * (math.pi * 8 ** 4 / 64) / STEM ** 2
p(f"- 기존 7 N·m = 200 N × 35 mm(초판 V1 기준) → V1-L에 과대. 깊이 적응이 F_TARGET {F_TARGET} N을 지키면 τ_close {F_TARGET*R/1e3:.2f} N·m로 연속 정격 안. "
  "**NEMA17 + 1:27 조건부 유지**(출력 ≤ 5 N·m 전류 제한, E0에서 τ_close > 3 N·m이면 NEMA23 + 유성 — Red Team M2: θ 기어모터는 E0 뒤 주문 권장).")
p(f"- push-rod Ø8 × {STEM:.0f} 좌굴 {Pcr/1e3:.1f} kN, 기어 순간 정격 로드력 {PG_T_PEAK/0.025:.0f} N 대비 SF {Pcr/(PG_T_PEAK/0.025):.0f}.")
p("")

# ============================================================================= 6. H2R plywood hybrid
p("## 6. H2R 합판 하이브리드 (현재안, RT1 개정)")
p("")
E_PLY, G_PLY, KAPPA, T_PLY = 5000.0, 400.0, 5.0 / 6.0, 18.0   # MPa, MPa — ASSUMPTION (below birch [S14])
RHO_PLY = {"stiff": 550.0, "mid": 500.0, "soft": 450.0}      # ASSUMPTION
WOOD_FACTOR = {"stiff": 2.0, "mid": 1.0, "soft": 0.5}        # ×2 metal-to-timber [S15]; soft = workmanship — ASSUMPTION
K_DEF = (0.8, 1.0)                                           # EN 1995-1-1 Table 3.2 (원문 확인 필요)
W_PANEL, S_PANEL = 500.0, 480.0
H_PANEL = H_RIM_P + Z_U
SPAN_PLATE = S_PANEL - T_PLY
B_PLATE, RIB_H, RIB_SETBACK = 270.0, 90.0, 50.0
WALL_BOTTOM = 130.0   # mm above the pan rim — RT1 (was 60): holder-top gap ≥ 100 mm (ISO 13854 hand value, 원문 확인 필요)
HOLDER_TOP = 10.0     # holder top frame above the rim — ASSUMPTION
AL_T = 10.0           # Al 10T brackets (BOM L20)


def k_ser(d, lab, planes=1):
    """EN 1995-1-1 Table 7.1 [S15]: ρ_m^1.5·d/23 per shear plane per bolt (snug or epoxy-bedded hole)."""
    return RHO_PLY[lab] ** 1.5 * d / 23.0 * WOOD_FACTOR[lab] * planes


def cant(P, a, b, t, E=E_AL):
    """cantilever leg deflection at the load point."""
    return P * a ** 3 / (3 * E * b * t ** 3 / 12.0)


def panel_defl(loads, W=W_PANEL, H=H_PANEL, n=4001):
    z = np.linspace(0.0, H, n)
    EI = E_PLY * T_PLY * W ** 3 / 12.0
    GA = KAPPA * G_PLY * T_PLY * W
    M = sum(P * np.clip(zj - z, 0.0, None) for zj, P in loads)
    V = sum(P * (z < zj) for zj, P in loads)
    return [float(np.trapezoid(M * np.clip(zi - z, 0.0, None) / EI, z) + np.trapezoid(V * (z < zi) / GA, z)) for zi, _ in loads]


def plate_beam(Rf, span=SPAN_PLATE, b=B_PLATE, t=T_PLY):
    return Rf * span ** 3 / (48 * E_PLY * t * b ** 3 / 12.0) + Rf * span / (4 * KAPPA * G_PLY * t * b)


def joints_loop(F, var, jw):
    """drag-loop joints outside the bridge: KFL08 fixed end → base, X nut bracket → deck, X cell → deck (+ link)."""
    d = {}
    if var == "H2":      # first H2: bolts only + KFL08 on a 40×40×4 angle
        d["KFL08 받침"] = F / (4 * k_ser(5, jw)) + cant(F, 30.0, 40.0, 4.0)
        d["X 너트 브래킷"] = F / (4 * k_ser(5, jw))
        d["X 셀 마운트"] = F / (4 * k_ser(6, jw))
    else:                # H2R: 10T brackets, epoxy-bedded M6 bolts, X link with 2 rod ends
        rot = lambda h, s: F * h / (K_SCREW_AL * s ** 2 / 2.0) * h        # edge-screw pair joint
        d["KFL08 받침"] = F / (4 * k_ser(6, jw)) + cant(F, 30.0, 70.0, AL_T) + rot(30.0, 40.0)
        d["X 너트 브래킷"] = F / (4 * k_ser(6, jw)) + cant(F, 35.0, 50.0, AL_T) + rot(35.0, 30.0)
        d["X 셀 마운트 + 링크"] = F / (4 * k_ser(6, jw)) + 2 * F / K_ROD_END
    return d


def block_mount(Rl, Ru, arm, var, jw, a_angle=25.0):
    """Z block mount → cross plate, two levels. H2: angle 40×40×4 (leg cantilever a) + 6 M6 single shear.
    H2R: ㄷ-clamp (10T web between 10T flanges, 4 M6 through-bolts in double shear, M5 edge screws)."""
    if var == "H2":
        kb = 6 * k_ser(6, jw)
        leg = lambda Rf: cant(Rf, a_angle, 100.0, 4.0)
        return two_level(Rl / kb + leg(Rl), -(Ru / kb + leg(Ru)), arm)
    kb = 4 * k_ser(6, jw, planes=2)
    web = lambda Rf: Rf * 38.0 ** 3 / (48 * E_AL * 100.0 * AL_T ** 3 / 12.0) + Rf / (4 * K_SCREW_AL)
    return two_level(Rl / kb + web(Rl), -(Ru / kb + web(Ru)), arm)


def tip_ply(F, c, var="H2R", jw="mid", a_angle=25.0):
    d, arm, R_l, R_u = common_head(F, c)
    d["cross plates"] = two_level(plate_beam(R_l), -plate_beam(R_u), arm)
    dl, du = panel_defl([(H_RIM_P + Z_L, R_l / 2), (H_RIM_P + Z_U, -R_u / 2)])
    d["side panels"] = two_level(dl, du, arm)
    d["X drive"] = F / screw_axial_k(X_SCREW_NUT_MAX)
    d["bed blocks"] = F * (H_RIM_P - BED_BLOCK_Z_P + c - cm.Z_C_LEVER) ** 2 / (K_SBR16 * 280.0 ** 2)
    d["X cell"] = F / K_XCELL
    d["Z block mounts"] = block_mount(R_l, R_u, arm, var, jw, a_angle)
    d.update(joints_loop(F, var, jw))
    return d


MEMBER_KEYS = ("stem", "Z column", "Z guide", "cross plates", "side panels", "X drive", "bed blocks", "X cell")


def total(F, var, jw, jm, c=C_MIN, a_angle=25.0):
    if var == "metal":
        return sum(tip_metal(F, c).values()) + joint_dx(F, K_JOINT[jm])
    return sum(tip_ply(F, c, var, jw, a_angle).values()) + joint_dx(F, K_JOINT[jm])


p("### 6.1 무엇이 바뀌었나 (H2 → H2R)")
p("")
table(["항목", "H2 (7a375d3)", "H2R (RT1)", "이유"], [
    ["Z 블록 마운트", "Al 앵글 40×40×4, M6 6개 한쪽 전단", "**ㄷ자 클램프**: 10T 웹(100×80, 블록 중심 = 교차판 두께 중심) + 10T 위·아래 플랜지(100×40), M6 관통 4개 **2면 전단**, 에폭시 베딩", "앵글 다리 외팔 굽힘(모델 누락, Red Team M3) 제거"],
    ["KFL08 고정단 받침", "앵글 40×40×4 (다리 4 mm)", "10T 직립판 70×(축 높이 30) + 10T 발판, M5 모서리 나사 2 + M6 관통 4(에폭시)", "다리 굽힘 0.06 mm @ 100 N 제거"],
    ["X 너트 브래킷", "(BOM 누락)", "10T L(데크 아래, 축까지 35)", "Red Team H2 누락 품목"],
    ["힘 플랫폼", "직렬 적층(수직 셀을 F_x가 지남)", "**병렬**: 수직 셀 → 10T 가로대 → 판스프링 → 상판 / X 셀은 로드엔드 링크로 상판 ↔ **데크** 직결, 스톱 케이지도 데크에", "수직 셀 보호(Red Team M7)"],
    ["베드", "데크를 SBR16UU에 직접", "18T 받침(자투리) 위에 데크 → X 모터 위 통과 간극", "X 모터 마운트(누락 품목)"],
    ["뒷벽·정면 창 하단", "테두리 + 60", f"**테두리 + {WALL_BOTTOM:.0f}**", "홀더 상면과 끼임 간격(Red Team M5)"],
    ["Z 기둥 / 레일", "4080 500 / MGN12 450", f"4080 {COL_Z:.0f} / MGN12 {RAIL_Z:.0f}", "블록 레일 이탈(§4)"],
    ["베드 데크", "650×400", "650×394", "재단 띠 적층 오류(Red Team L1)"],
])

p("### 6.2 체결부 상세 모델 (@ F_d, C = −75, 나무 중간 가정)")
p("")
arm0 = Z_L - (C_MIN - cm.Z_C_LEVER)
Ru0 = F_D * arm0 / H_B
Rl0 = F_D + Ru0
rows = []
for a in (15.0, 25.0, 35.0):
    rows.append([f"H2 앵글 40×40×4, 하중점 {a:.0f} mm", f3(block_mount(Rl0, Ru0, arm0, "H2", "mid", a)),
                 f3(two_level(cant(Rl0, a, 100.0, 4.0), -cant(Ru0, a, 100.0, 4.0), arm0))])
rows.append(["**H2R ㄷ자 클램프 10T**", f3(block_mount(Rl0, Ru0, arm0, "H2R", "mid")), "0 (웹이 위·아래로 지지)"])
table(["Z 블록 마운트", "마운트 항 합계 [mm]", "그중 다리/웹 굽힘 [mm]"], rows)
jl_h2, jl_h2r = joints_loop(F_D, "H2", "mid"), joints_loop(F_D, "H2R", "mid")
table(["베드·베이스 쪽 체결", "H2 [mm]", "H2R [mm]"],
      [[k, f3(v), f3(v2)] for (k, v), (k2, v2) in zip(jl_h2.items(), jl_h2r.items())])
p(f"- Red Team이 계산한 앵글 다리 항(+0.05 / +0.24 / +0.65 mm @ 하중점 15 / 25 / 35 mm)을 이 모델도 재현한다(두 번째 열). ㄷ자 클램프는 블록 중심을 교차판 두께 중심에 두고 "
  "웹을 위·아래 플랜지가 잡아 외팔 굽힘이 없다.")
p("")

p("### 6.3 스쿱 끝 X 변위 — 이전안 vs H2 vs H2R (C = −75)")
p("")
rows = []
for var, lab in (("metal", "이전안 L-A 금속"), ("H2", "H2(앵글, 하중점 25 mm — 누락 항 포함)"), ("H2R", "**H2R (현재안)**")):
    for F in (F_NOM, F_D):
        mid = total(F, var, "mid", "mid")
        rows.append([lab if F == F_NOM else "", f"{F:.0f}", f"**{mid:.2f}** {'●' if mid <= 1 else '✘'}",
                     f2(total(F, var, "stiff", "stiff")), f"{total(F, var, 'soft', 'soft'):.2f} {'●' if total(F, var, 'soft', 'soft') <= 1 else '✘'}",
                     f2(total(F, var, "soft", "mid")), f2(total(F, var, "mid", "soft")), f"{F/mid:.0f}"])
table(["구성", "F_x [N]", "중간/중간", "단단/단단", "무름/무름", "나무 무름 + 금속 중간", "나무 중간 + 금속 무름", "경로 강성(중간) [N/mm]"], rows)
dR = tip_ply(F_D, C_MIN, "H2R", "mid")
table(["H2R @ F_d 요소", *dR.keys(), "금속 헤드 체결"], [["[mm]", *[f3(v) for v in dR.values()], f3(joint_dx(F_D, K_JOINT["mid"]))]])
r_mid, r_soft = total(F_D, "H2R", "mid", "mid"), total(F_D, "H2R", "soft", "soft")
r_mid_n, r_soft_n = total(F_NOM, "H2R", "mid", "mid"), total(F_NOM, "H2R", "soft", "soft")
p(f"- **판정(조립 후 교정 통과 조건부)**: H2R 중간 가정 {r_mid_n:.2f} / {r_mid:.2f} mm @ {F_NOM:.0f} / {F_D:.0f} N, 무름 가정 {r_soft_n:.2f} / {r_soft:.2f} mm. "
  "어느 경우든 **조립 후 스쿱 끝 교정**(물병 하중 0 → 120 → 0 N ×3, Z 3높이)에서 R² ≥ 0.99, 히스테리시스 ≤ 0.10 mm, 영점 복귀 ≤ 0.05 mm를 통과해야 쓴다. "
  "통과하고 X 변위 > 1 mm면 F_TARGET·속도를 낮춰 운용하고, 깊이는 C_z·F_z 보상으로 맞춘다.")
p(f"- 운용 상한은 F_TARGET {F_TARGET} N(깊이 적응)이다: 그때 H2R 중간 {total(F_TARGET, 'H2R', 'mid', 'mid'):.2f} / 무름 {total(F_TARGET, 'H2R', 'soft', 'soft'):.2f} mm. "
  f"Nano 정지 {F_NANO_STOP:.0f} N에서는 중간 {total(F_NANO_STOP, 'H2R', 'mid', 'mid'):.2f} mm.")
p(f"- 제어 담당용 X 경로 강성(스쿱 ↔ 팬): 중간 {F_D/r_mid:.0f} N/mm, 무름 {F_D/r_soft:.0f} N/mm(초판 231–287 N/mm는 금속안·체결부 제외 값).")
p("")

p("### 6.4 옆힘·수직")
p("")
Fy = K_SIDE * F_D
ls = STEM + cm.Z_C_LEVER
side_core = (Fy * ls ** 3 / (3 * E_SS * STEM_I["30x3"])
             + cantilever_column(Fy, E_AL * PROF["4080"]["I_w"], Z_L - (C_MIN + STEM + HEAD), HEAD + STEM + cm.Z_C_LEVER)
             + two_level(Fy * arm0 / H_B / K_MGN12 + Fy / K_MGN12, -Fy * arm0 / H_B / K_MGN12, arm0))
h_low = H_RIM_P + WALL_BOTTOM
I_out = W_PANEL * T_PLY ** 3 / 12.0
sway = {c_: Fy / (2 * c_ * E_PLY * I_out / h_low ** 3) for c_ in (3, 12)}
A_f, A_w = (B_PLATE - RIB_SETBACK) * T_PLY, T_PLY * RIB_H
y_c = (A_f * T_PLY / 2 + A_w * (T_PLY + RIB_H / 2)) / (A_f + A_w)
I_L = (B_PLATE - RIB_SETBACK) * T_PLY ** 3 / 12 + A_f * (y_c - T_PLY / 2) ** 2 + T_PLY * RIB_H ** 3 / 12 + A_w * (T_PLY + RIB_H / 2 - y_c) ** 2
Fz = K_VERT * F_D


def ss_mid(P, L, EI):
    return P * L ** 3 / (48 * EI)


table(["항목", "값", "비고"], [
    [f"옆 변위 @ F_y {Fy:.0f} N: 스템·기둥·Z 가이드", f"{side_core:.2f} mm", "4080 약축"],
    [f"  + 측판 흔들림(뒷벽 하단 테두리 + {WALL_BOTTOM:.0f} → 자유 높이 {h_low:.0f} mm)", f"{sway[3]:.2f} (발 핀) / {sway[12]:.2f} (발 고정) mm", "초판(+60)보다 커짐 — 발 클리트 양면 접착으로 고정 쪽에 가깝게"],
    [f"Z 너트 자리 수직 @ F_z {Fz:.0f} N (교차판 + 앞 립, 립은 앞에서 {RIB_SETBACK:.0f} 뒤)", f"{ss_mid(Fz, SPAN_PLATE, E_PLY*I_L):.3f} mm", f"I = {I_L:.3g} mm⁴"],
    ["  자중 처짐 크리프", f"×{1+K_DEF[0]:.1f}–{1+K_DEF[1]:.1f}", "k_def(원문 확인 필요), 터치오프가 흡수"],
    [f"베드 데크 처짐 @ F_z {Fz:.0f} N (블록 사이 300, 폭 394)", f"{ss_mid(Fz, 300.0, E_PLY*394.0*T_PLY**3/12):.3f} mm", "F_z 측정으로 보상 가능"],
    ["X 모터 위 데크 통과 간극", f"{STACK_P['SBR16 rail+block'] + STACK_P['bed spacer (18T offcut)'] - (X_AXIS_H + 21.0):.0f} mm",
     f"데크 아래면 = SBR16 45 + 받침 18, 모터 위 끝 = 축 {X_AXIS_H:.0f} + 21 → 모터 덮개(3D 프린트)로 막음"],
])

p("### 6.5 힘 플랫폼 — 병렬 배치와 데크 접지 스톱 케이지 (Red Team M7)")
p("")
p("```")
p("  홀더 ─ 상판(6T×2 적층 12 mm, 350×300) ──[로드엔드]── X 링크(M6) ──[로드엔드]── X 셀 50 kg(자유단) ─ 고정단 ─ Al 10T 받침 ─ 베드 데크")
p("           │ 판스프링 ×2(X로만 유연, Y·Z 강함)")
p("         Al 10T 가로대 ─ 수직 셀 20 kg(자유단) ─ 고정단 ─ Al 10T 스페이서 ─ 베드 데크")
p("  스톱 케이지(모두 베드 데크에 볼트): ±X 2, ±Y 2, 상판 네 모서리 아래 수직 4 — M6 세트스크루 + 잠금너트, 틈새 게이지로 간극 설정")
p("```")
p("")
k_flex = 0.01 * K_XCELL          # flexure X stiffness ≤ 1 % of the cell (BOM L33 condition)
x_def_stop = F_STOP / K_XCELL
gap_x = 0.15
F_x_stop = gap_x * K_XCELL
v_normal = (M_TOP * 9.81 + K_VERT * F_STOP) / K_VCELL
gap_v = 0.30
F_v_stop = gap_v * K_VCELL
table(["항목", "값", "판정·비고"], [
    ["정상 최대 F_x(F_STOP + m·a)에서 X 셀 처짐", f"{(F_STOP+M_TOP*A_X)/K_XCELL:.3f} mm", f"X 스톱 간극 {gap_x} mm보다 작아야 함 ●"],
    [f"X 스톱 간극 {gap_x} mm → 닿는 힘", f"{F_x_stop:.0f} N", f"F_STOP·관성({F_STOP+M_TOP*A_X:.0f} N) < 닿는 힘 < X 셀 안전 과부하 {X_CELL_KG*9.81*SAFE_OVL:.0f} N ●"],
    [f"걸림 상한 {F_JAM_MAX:.0f} N 중 스톱이 받는 몫", f"{F_JAM_MAX - F_x_stop:.0f} N", f"X 셀에는 최대 {F_x_stop:.0f} N(정격의 {F_x_stop/(X_CELL_KG*9.81)*100:.0f} %)"],
    ["수직 셀 정상 하중(팬·홀더·상판 + 0.5·F_STOP)", f"{M_TOP*9.81 + K_VERT*F_STOP:.0f} N → 처짐 {v_normal:.3f} mm", f"수직 스톱 간극 {gap_v} mm보다 작음 ●"],
    [f"수직 스톱 간극 {gap_v} mm → 닿는 힘", f"{F_v_stop:.0f} N", f"안전 과부하 {V_CELL_KG*9.81*SAFE_OVL:.0f} N 이내 ●. Z 하강력 ~{thrust(T_HOLD_Z, 2.0, 0.10)+W_Z:.0f} N의 나머지는 스톱이 받음"],
    ["수직 셀이 받는 F_x 몫(판스프링 강성 ≤ 셀의 1 %)", f"≤ {F_STOP*k_flex/(K_XCELL+k_flex):.1f} N @ F_STOP", "옆하중 허용치 **데이터시트 확인 필요**"],
    ["수직 셀이 받는 F_y(판스프링이 Y로 강함)", f"≤ {K_SIDE*F_STOP:.0f} N @ F_STOP", "옆하중 허용치 **데이터시트 확인 필요**; ±Y 스톱 간극 0.15 mm"],
    ["F_x 모멘트(팬 높이 ~150 mm − 링크 높이)", f"≈ {F_STOP*0.15:.0f} N·m @ F_STOP", "상판 → 판스프링 → 가로대로 짝힘 → 수직 셀 편심 하중. 허용 모멘트 **데이터시트 확인 필요**; 걸림 때는 수직 스톱 4개가 받음"],
])
p("- 간극 값은 시작값이다. 물병 하중으로 셀 처짐을 잰 뒤 '정상 최대에서는 안 닿고, 안전 과부하 전에 닿게' 틈새 게이지로 다시 맞춘다(전자 §2.2와 같은 원칙).")
p("")

p(f"### 6.6 강도 — F_JAM 상한 {F_JAM_MAX:.0f} N")
p("")
F_J = F_JAM_MAX
Ru_J = F_J * arm0 / H_B
Rl_J = F_J + Ru_J
sig_stem = F_J * ls * STEM_C["30x3"] / STEM_I["30x3"]
M_pl = Rl_J * SPAN_PLATE / 4
sig_pl = M_pl * (B_PLATE / 2) / (T_PLY * B_PLATE ** 3 / 12)
table(["부재", "하중", "응력·하중", "허용", "SF"], [
    ["스템 Ø30×3 304", f"M = {F_J:.0f} × {ls:.0f}", f"{sig_stem:.0f} MPa", f"{SY_304:.0f} MPa", f"{SY_304/sig_stem:.1f}"],
    ["하부 Z 블록 MGN12H", f"R_l = F·(1 + {arm0:.0f}/{H_B:.0f})", f"{Rl_J:.0f} N", "C0 5.88 kN [S9]", f"{5880/Rl_J:.1f}"],
    ["ㄷ자 클램프 M6 관통 4개(2면 전단)", f"R_l {Rl_J:.0f} N", f"{Rl_J/8:.0f} N/전단면", "Johansen 약 2.0 kN/볼트 특성값 [S19]", f"{4*2000/Rl_J:.1f}"],
    ["하부 교차판 18T(면내 굽힘)", f"M = R_l·L/4", f"{sig_pl:.1f} MPa", "합판 굽힘 강도(수 MPa 이상, 가정)", "여유 큼"],
    ["X 셀 50 kg(스톱 케이지 있음)", f"≤ {F_x_stop:.0f} N", f"정격의 {F_x_stop/(X_CELL_KG*9.81)*100:.0f} %", f"안전 과부하 {X_CELL_KG*9.81*SAFE_OVL:.0f} N", "●"],
    ["수직 셀 20 kg(스톱 케이지 있음)", f"≤ {F_v_stop:.0f} N", f"정격의 {F_v_stop/(V_CELL_KG*9.81)*100:.0f} %", f"안전 과부하 {V_CELL_KG*9.81*SAFE_OVL:.0f} N", "●"],
    ["push-rod Ø8", f"τ ≤ {PG_T_PEAK} N·m", f"{PG_T_PEAK/0.025:.0f} N", f"P_cr {Pcr/1e3:.1f} kN", f"{Pcr/(PG_T_PEAK/0.025):.0f}"],
])

p("### 6.7 합판 1장 재단도 (1220 × 2440, 톱날 3 mm) — 띠 안 적층까지 검사")
p("")
KERF = 3.0
RIB_W = math.floor((1220.0 - H_PANEL - KERF - 2 * KERF) / 3.0)
DECK_W = 394.0
WALL_H = Z_U - WALL_BOTTOM
BANDS = [  # (band length, [columns across the 1220 width: (name, across, along, stacked pieces along)])
    (2 * W_PANEL + KERF, [("측판", H_PANEL, [W_PANEL, W_PANEL]), ("베이스 리브 ×3(나란히)", 3 * RIB_W + 2 * KERF, [1000.0])]),
    (560.0, [("베이스 판", 1000.0, [560.0]), ("발 클리트 ×3(나란히)", 3 * 60.0 + 2 * KERF, [560.0])]),
    (580.0, [("뒷벽 + 아래 클리트 띠", SPAN_PLATE, [WALL_H, 580.0 - WALL_H - KERF]), ("베드 데크 + 앞 립 ×2", 650.0, [DECK_W, RIB_H, RIB_H]),
             ("발 클리트", 60.0, [580.0])]),
    (B_PLATE, [("교차판 ×2(나란히)", 2 * SPAN_PLATE + KERF, [B_PLATE]), ("교차판 클리트 ×4(나란히)", 4 * 60.0 + 3 * KERF, [B_PLATE])]),
]
rows = []
used_L = 0.0
for L, cols in BANDS:
    across = sum(c[1] for c in cols) + KERF * (len(cols) - 1)
    assert across <= 1220.0 + 1e-9, f"band {L}: {across} > 1220"
    for name, w, stack in cols:
        along = sum(stack) + KERF * (len(stack) - 1)
        assert along <= L + 1e-9, f"{name}: {along} > band {L}"
        rows.append([f"{L:.0f}", name, f"{w:.0f}", " + ".join(f"{s:.0f}" for s in stack), f"{along:.0f} ≤ {L:.0f}"])
    used_L += L
used_L += KERF * (len(BANDS) - 1)
assert used_L <= 2440.0
table(["띠 길이", "부품", "폭 [mm]", "띠 방향 적층 [mm]", "검사"], rows)
p(f"- 띠 길이 합 {used_L:.0f} ≤ 2440. 베이스 리브 폭 {RIB_W:.0f} mm(측판이 {H_PANEL:.0f}로 높아져 줄어듦), 베드 데크 650×{DECK_W:.0f}(Red Team L1). "
  f"뒷벽은 {SPAN_PLATE:.0f}×{WALL_H:.0f}(하단 테두리 + {WALL_BOTTOM:.0f}).")
BW = 560.0
A_p, A_r = BW * T_PLY, 3 * T_PLY * RIB_W
y_b = (A_p * T_PLY / 2 + A_r * (T_PLY + RIB_W / 2)) / (A_p + A_r)
I_base = BW * T_PLY ** 3 / 12 + A_p * (y_b - T_PLY / 2) ** 2 + 3 * T_PLY * RIB_W ** 3 / 12 + A_r * (T_PLY + RIB_W / 2 - y_b) ** 2
p(f"- 베이스 처짐(베드 {M_BED:.1f} kg 가운데, 패드 간격 900): {ss_mid(M_BED*9.81, 900.0, E_PLY*I_base):.3f} mm, 아이스크림 소진 변화 {ss_mid(M_ICE*9.81, 900.0, E_PLY*I_base):.3f} mm.")
p("")

p("### 6.8 교정 하중 (Red Team L5)")
p("")
MU_PULLEY = 0.05      # pulley friction loss — ASSUMPTION
BOTTLE = 2.04         # kg per 2 L bottle incl. bottle — ASSUMPTION
need = 1.1 * F_STOP / (1 - MU_PULLEY) / 9.81
n_b = math.ceil(need / BOTTLE)
table(["항목", "값"], [
    ["목표 교정 하중(스쿱 끝·셀)", f"1.1 × F_STOP = {1.1*F_STOP:.0f} N"],
    [f"도르래 마찰 {MU_PULLEY*100:.0f} %(가정) 보정 후 필요 질량", f"{need:.1f} kg"],
    [f"2 L 물병({BOTTLE} kg) 개수", f"**{n_b}개** → {n_b*BOTTLE*9.81*(1-MU_PULLEY):.0f} N (기존 6개 = {6*BOTTLE*9.81*(1-MU_PULLEY):.0f} N으로 부족)"],
    ["Z 정전 유지 시험(자중 + 5 kg)", "물병 3개(6.1 kg)로 겸용"],
])

p("### 6.9 C15 — 젤라토 팬 → GN 1/3 325×176×150 판단")
p("")
lx_gn = 325.0 - 2 * (R + cm.WALL_MARGIN)
d_gn, a_gn = depth_for_portion(lx_gn)
F_gn_nom, F_gn_d = 90e-3 * a_gn, 160e-3 * a_gn
C_MIN_GN = -150.0 + cm.FLOOR_MARGIN + R
arm_gn = Z_L - (C_MIN_GN - cm.Z_C_LEVER)
table(["경우", "C 최저 [mm]", "레버 [mm]", "Z 행정 [mm]", "F @ 90 / 160 kPa [N]", "H2R 중간 @ 1 portion 단단 제품 [mm]"], [
    ["젤라토 팬(현재)", f"{C_MIN:.0f}", f"{arm0:.0f}", f"{Z_TOP-C_MIN:.0f}", f"{F_NOM:.0f} / {F_D:.0f}", f2(total(F_D, 'H2R', 'mid', 'mid'))],
    ["GN 1/3, 채움 ≤ 120 mm(바닥부터)", f"{C_MIN_GN:.0f}", f"{arm_gn:.0f}", f"{Z_TOP-C_MIN_GN:.0f}", f"{F_gn_nom:.0f} / {F_gn_d:.0f}",
     f2(total(F_gn_d, 'H2R', 'mid', 'mid', c=C_MIN_GN))],
    ["GN 1/3 + 거짓 바닥 30 mm", f"{C_MIN:.0f}", f"{arm0:.0f}", f"{Z_TOP-C_MIN:.0f}", f"{F_gn_nom:.0f} / {F_gn_d:.0f}", f2(total(F_gn_d, 'H2R', 'mid', 'mid'))],
])
p(f"- 채움 높이만 120 mm로 두면 레버는 **그대로가 아니다**: 스쿱이 팬 바닥까지 내려가므로 C 최저 {C_MIN_GN:.0f}, 레버 {arm_gn:.0f} mm, Z 행정 +30 mm. "
  "레버를 지키려면 30 mm 거짓 바닥이 필요하다. 그래도 레인 {:.0f} mm라 1 portion 힘이 {:.0f} / {:.0f} N으로 커져, F_d 기준 변위가 1 mm를 넘는다. ".format(lx_gn, F_gn_nom, F_gn_d)
  + "팬 높이 +30 mm로 측판이 높아져 재단도 리브 폭이 더 준다. 테이퍼(GN 팬은 쌓기용으로 벽이 기울 수 있음, 미확인)면 아래층 레인이 더 짧다.")
p("- **판정: 기각(현재안 유지)**. 절감 15,100원보다 결정값(F_nominal 56 / F_d 100) 재산정과 문서 전체 수정 비용이 크다. 젤라토 팬을 구하지 못할 때만 거짓 바닥 조건으로 다시 본다.")
p("")

p("### 6.10 예산 — BOM 읽기 (`elec/v1l_bom.csv`)")
p("")
if os.path.exists(BOM_CSV):
    with open(BOM_CSV, encoding="utf-8", newline="") as fh:
        bom = list(csv.reader(fh))[1:]
    mine = [r for r in bom if r[0].startswith("L") and ("[기계 RT1]" in r[14] or r[1] == "Frame") and int(r[7] or 0) > 0]
    table(["ID", "구분", "품목", "수량", "합계 [원]", "가격 상태"], [[r[0], r[1], r[2], r[4], f"{int(r[7]):,}", r[10]] for r in mine])
    sums = {r[0]: (int(r[7]), r[14]) for r in bom if r[0].startswith("SUM")}
    table(["합계 행", "금액 [원]", "비고"], [[k, f"{v:,}", n] for k, (v, n) in sums.items()])
else:
    p("(BOM 파일 없음)")

with open(OUT, "w", encoding="utf-8") as fh:
    fh.write("\n".join(lines) + "\n")
print("\n".join(lines))
