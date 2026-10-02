"""Undergraduate-capstone obstacles: numbers behind the alternative mechanisms.

Run:  python3 calc/capstone_alternatives.py
Out:  calc/output/capstone_alternatives.md

Answers, with the same models as the rest of calc/ (all loads and material
values are ASSUMPTIONS; see data/assumptions.md A25-A45):
  1. Rectangular gelato pan vs round tub: lane length, depth and drag force for one portion
  2. V1-S (student) geometry: shorter stem / Z stroke -> shorter lever -> tip deflection per frame type
  3. Pitch drive: single parallelogram (dead points, 120 deg) vs two push-rods 90 deg apart (360 deg)
  4. Cold chain without a chest freezer: insulated pan holder warm-up time
  5. Force platform under the pan instead of nut load cells: contact / portion resolution
  6. Off-the-shelf motion firmware (host-side force loop): overrun distance vs latency
"""

import math
import os

import cartesian_model as cm
import scoop_load_path as sl
import tub_lane_planner as tlp

OUT = os.path.join(os.path.dirname(__file__), "output", "capstone_alternatives.md")
lines = []
p = lines.append

U_CASES = (40.0, 90.0, 160.0)          # kPa (A02 cases)
F_TARGET = 110.0
V_TARGET = tlp.V_TARGET
RHO = 0.65

# ---------------------------------------------------------------- containers (ASSUMPTION)
PAN = {"name": "젤라토 팬 360×165×120 (≈5 L)", "L": 360.0, "W": 165.0, "H": 120.0}
GN13 = {"name": "GN 1/3 팬 325×176×150", "L": 325.0, "W": 176.0, "H": 150.0}
MARGIN = cm.WALL_MARGIN
R = cm.R_SCOOP


def pan_lane(pan):
    """C travel along the long side and allowed lane offsets (vertical walls assumed)."""
    lx = pan["L"] - 2 * (R + MARGIN)
    ymax = pan["W"] / 2 - (R + MARGIN)
    return lx, ymax


def depth_for_volume(travel, v_target=V_TARGET):
    for d10 in range(20, 300):
        d = d10 / 10.0
        v, a = tlp.stroke_volume(d, travel)
        if v >= v_target:
            return d, a, v
    return None, None, None


def table(h, rows):
    p("| " + " | ".join(h) + " |")
    p("|" + "|".join("---" for _ in h) + "|")
    for r in rows:
        p("| " + " | ".join(str(c) for c in r) + " |")
    p("")


p("# Capstone alternatives — generated")
p("")
p("> `python3 calc/capstone_alternatives.py`. 해석은 `docs/capstone_challenges_and_alternatives.md`. **모든 하중·재료·열 물성은 ASSUMPTION.**")
p("")

# ---------------------------------------------------------------- 1. container
p("## 1. 직사각 팬 vs 원형 통 — 1 portion(115 g)에 필요한 깊이와 드래그 힘")
p("")
rows = []
cases = [("원형 통 Ø230, 표면 −62, 레인 y = 0", tlp.lane_length(0.0, 62.0, 24.0)),
         ("원형 통 Ø230, 표면 −62, 레인 y = ±40", tlp.lane_length(40.0, 62.0, 24.0))]
for pan in (PAN, GN13):
    lx, ymax = pan_lane(pan)
    cases.append((f"{pan['name']}, 모든 레인(|y| ≤ {ymax:.0f})", lx))
for name, L in cases:
    d, a, v = depth_for_volume(L)
    if d is None:
        rows.append([name, f"{L:.0f}", "불가", "–", "–", "–"])
        continue
    rows.append([name, f"{L:.0f}", f"{d:.1f}", f"{a:.0f}"] + [f"{u * 1e-3 * a:.0f}" for u in (90.0, 160.0)])
table(["용기·레인", "C 이동 [mm]", "필요 깊이 d [mm]", "절삭단면 A [mm²]", "F @ 90 kPa [N]", "F @ 160 kPa [N]"], rows)
lx, ymax = pan_lane(PAN)
w = tlp.cut_width(14.0)
p(f"- 팬은 벽이 직선이라 **모든 레인 길이가 같다**({lx:.0f} mm). 원형 통의 옆 레인 문제가 사라진다.")
p(f"- 레인이 길어 얕게(≈{depth_for_volume(lx)[0]:.0f} mm) 끌어도 1 portion이 된다 → 드래그 힘이 원형 통 대비 약 절반.")
p(f"- 레인 배치 예: y = −{ymax:.0f} / 0 / +{ymax:.0f} (절삭 폭 ≈ {w:.0f} mm, 간격 {ymax:.0f} mm → 겹침). 벽 쪽 띠 {PAN['W'] / 2 - ymax - w / 2:.0f} mm는 사람이 정리.")
p("")

# ---------------------------------------------------------------- 2. V1-S lever and deflection
p("## 2. V1-S(학생 제작형) 기하 — 레버를 줄이면 3D 프린터식 구조도 쓸 만해진다")
p("")
STEM_S = 200.0                     # stem for a 120 mm deep pan (head bottom stays >= 100 mm above deck)
Z_TOP_S = 100.0                    # Z_SAFE 60 + 40
C_MIN_S = -PAN["H"] + cm.FLOOR_MARGIN + R
H_BLOCK_S = Z_TOP_S + STEM_S + cm.HEAD_LEN + 20.0
geo_v1 = sl.geometry(cm.z_min())
saved = (cm.STEM_LEN, sl.H_BLOCK)
cm.STEM_LEN, sl.H_BLOCK = STEM_S, H_BLOCK_S
geo_s = sl.geometry(C_MIN_S)
cm.STEM_LEN, sl.H_BLOCK = saved
p(f"- V1: 통 깊이 250 → 스템 300, C_min {cm.z_min():.0f}, 하부 Z 블록 {saved[1]:.0f} → **레버 {geo_v1[2]:.0f} mm**")
p(f"- V1-S: 팬 깊이 {PAN['H']:.0f} → 스템 {STEM_S:.0f}, Z 행정 {Z_TOP_S - C_MIN_S:.0f} (C {Z_TOP_S:.0f} … {C_MIN_S:.0f}), 하부 Z 블록 {H_BLOCK_S:.0f} → **레버 {geo_s[2]:.0f} mm**")
p("")
rows = []
for key in ("L", "M", "S30"):
    cfg = sl.CONFIGS[key]
    for F in (50.0, 80.0, 110.0):
        d1 = sum(sl.tip_dx(cfg, F, cm.z_min(), 1).values())
        cm.STEM_LEN, sl.H_BLOCK = STEM_S, H_BLOCK_S
        d2 = sum(sl.tip_dx(cfg, F, C_MIN_S, 1).values())
        cm.STEM_LEN, sl.H_BLOCK = saved
        rows.append([cfg["name"].split(" (")[0], f"{F:.0f}", f"{d1:.2f}", f"{d2:.2f}", "●" if d2 <= 1.0 else "–"])
table(["구조", "F_x [N]", "V1 레버 변위 [mm]", "V1-S 레버 변위 [mm]", "V1-S ≤ 1 mm"], rows)
p("- 변위는 대부분 레버의 제곱에 비례하는 항(기둥·블록·빔 회전)이다. 얕은 팬 + 짧은 스템으로 레버를 줄이고, 긴 레인으로 힘을 줄이면 둘이 곱으로 줄어든다.")
p("- 벨트 X(구성 L)는 여전히 드래그 축으로 부적합(작업장력). **X는 볼스크류 모듈**을 쓴다(구성 M/S 계열의 구동계).")
p("")

# ---------------------------------------------------------------- 3. pitch drive
p("## 3. 피치 구동 — 평행링크 1개 vs 90° 위상 push-rod 2개")
p("")
L_LEVER = 25.0
rows = []
for tau in (3.2, 7.0):
    single = []
    for phi in (0, 30, 45, 60, 75, 85):
        single.append(tau / (L_LEVER * 1e-3 * math.cos(math.radians(phi))))
    dual_max = tau / (L_LEVER * 1e-3)
    rows.append([f"{tau:.1f}"] + [f"{v:.0f}" for v in single] + [f"{dual_max:.0f}"])
table(["τ [N·m]", "단일 φ=0°", "30°", "45°", "60°", "75°", "85°", "이중(90° 위상), 최대"], rows)
p("- 이중 push-rod(기관차 연결봉과 같은 원리)는 한 로드가 사점일 때 다른 로드가 토크를 전한다. 최소노름 분배 F₁ = τcosφ/l, F₂ = −τsinφ/l → **어느 각도에서도 로드 힘 ≤ τ/l**, 회전 범위 제한 없음.")
p("- 효과: θ를 −90°(개구부 아래)까지 돌릴 수 있어 **뒤집기 + 흔들기 배출**이 가능해지고, 드래그 중 레버각 제약도 사라진다.")
p("- 대가: 로드·핀 1세트 추가(식품영역 부품 +3), 두 로드 길이·크랭크 반경 공차가 맞지 않으면 서로 버틴다 → 한쪽 하부 핀을 긴 구멍(±0.3 mm)으로.")
p("")

# ---------------------------------------------------------------- 4. insulated holder
p("## 4. 냉동고 없이 시험하기 — 단열 팬 홀더의 온도 상승 시간")
p("")
K_XPS = 0.034                      # W/mK
T_WALL = 0.05                      # m XPS
A_WALL = 2 * (0.36 * 0.12 + 0.165 * 0.12) + 0.36 * 0.165          # sides + bottom of the pan
A_OPEN = 0.36 * 0.165
H_OPEN = 10.0                      # W/m2K, free convection + radiation at the open top (ASSUMPTION)
T_AIR, T_IC = 22.0, -14.0
MASS = PAN["L"] * PAN["W"] * PAN["H"] * 1e-9 * 0.8 * RHO * 1000    # kg (80 % full)
rows = []
for c_app, lab in ((3000.0, "낮게 가정"), (6000.0, "높게 가정")):   # J/kgK apparent heat capacity incl. latent (ASSUMPTION)
    for top, tl in ((False, "뚜껑 열림"), (True, "시험 사이 뚜껑 덮음(XPS 30 mm)")):
        q_wall = K_XPS * A_WALL * (T_AIR - T_IC) / T_WALL
        q_top = (K_XPS * A_OPEN * (T_AIR - T_IC) / 0.03) if top else H_OPEN * A_OPEN * (T_AIR - T_IC)
        q = q_wall + q_top
        t2 = MASS * c_app * 2.0 / q / 60.0
        rows.append([lab, tl, f"{q:.1f}", f"{t2:.0f}"])
table(["겉보기 비열", "상부 조건", "열유입 [W]", "+2 K까지 [분]"], rows)
p(f"- 팬 {MASS:.1f} kg(80 % 채움). 겉보기 비열(잠열 포함)은 −14 °C 부근 값이 확인되지 않아 두 경우로 둔다 `[미검증]`.")
p("- 결론: 단열 홀더(XPS 50 mm) + 시험 사이 뚜껑이면 **한 번 꺼내서 수십 분** 시험할 수 있다. E0 프로토콜의 '3분 창'은 맨통을 상온에 둔 경우다.")
p("- 판정 온도 유지는 통 표면 열전대로 확인(±1 K 벗어나면 냉동고에 복귀). 드라이아이스는 쓰지 않는다(표면이 과냉되어 온도 구배가 생김).")
p("")

# ---------------------------------------------------------------- 5. force platform
p("## 5. 팬 아래 힘 플랫폼 — 너트 로드셀 대신")
p("")
m_platform = MASS + 1.5 + 1.0     # ice cream + pan + holder (kg, ASSUMPTION)
cap = 20.0                         # kg single-point load cell
res_g = cap * 1000 / 2 ** 16       # conservative effective resolution (16 effective bits of HX711 at 10 Hz)
p(f"- 수직: 단일점 로드셀 {cap:.0f} kg(플랫폼 {m_platform:.1f} kg 위). 유효 분해능 ≈ {res_g:.1f} g(HX711 유효 16비트 가정).")
p(f"  - 접촉 3 N = {3 / 9.81 * 1000:.0f} g → 분해능의 {3 / 9.81 * 1000 / res_g:.0f}배로 충분히 검출.")
p(f"  - portion = 스쿱 전후 팬 질량 차 → ±{2 * res_g:.0f} g 수준(진동이 멈춘 뒤 측정). **컵 저울이 필요 없다.**")
p("- 수평: 플랫폼을 X 방향 볼 레일에 올리고 S-빔 로드셀 1개로 X 방향만 구속 → F_x를 직접(마찰 보정 후).")
p("- 이점: 상용 볼스크류 모듈의 너트에 로드셀을 끼우는 가공이 필요 없다. 연구 데이터(F_x, F_z)가 아이스크림이 실제로 받은 힘이 된다(legacy SA6/C44와 같은 개념).")
p("")

# ---------------------------------------------------------------- 6. host loop latency
p("## 6. 상용 모션 펌웨어(G-code) + 호스트 힘 루프 — 지연 동안 더 가는 거리")
p("")
rows = []
for lat in (30, 60, 100, 150):
    for v in (40.0, 80.0):
        rows.append([f"{lat}", f"{v:.0f}", f"{v * lat / 1000:.1f}"])
table(["지연 [ms]", "드래그 속도 [mm/s]", "정지 판단 후 추가 이동 [mm]"], rows)
p("- Klipper/grblHAL 같은 공개 펌웨어는 궤적을 미리 버퍼링하므로 **드래그 도중 Z를 실시간으로 바꾸는 깊이 적응은 어렵다.**")
p("- 대안: (a) 드래그를 20 mm 구간으로 나눠 구간마다 힘을 보고 다음 구간 Z를 정함(구간 적응), (b) **스트로크 간 적응**: 첫 스트로크의 평균 힘으로 u를 추정해 다음 스트로크 깊이를 정함, (c) 과부하는 호스트가 즉시 정지(피드 홀드) — 100 ms 지연이면 80 mm/s에서 8 mm 더 가므로 정지 한계를 구조 설계 하중보다 충분히 낮게 둔다.")
p("")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print("\n".join(lines))
