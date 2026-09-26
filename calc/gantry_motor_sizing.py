"""Global axis (X, Y) and local-axis sizing for the Cartesian architecture.

Run:  python3 calc/gantry_motor_sizing.py
Out:  calc/output/gantry_motor_sizing.md

Covers
  - X_M (bridge axis): Arch 1 uses it for travel AND drag -> must deliver the
    cutting force; checked for ball-screw critical speed and DN limit.
  - Y_M: V1 short cross-slide (lanes), product moving bridge (belt).
  - Arch 2 alternative: belt X/Y + local short x_scoop screw.
  - Maximum force each drive can physically produce (safety input).
Loads are ASSUMPTION CASES (calc/cartesian_model.py). Stepper torque-speed
curves are a generic ASSUMPTION; replace with the chosen motor's datasheet.
"""

import math
import os

import cartesian_model as cm

OUT = os.path.join(os.path.dirname(__file__), "output", "gantry_motor_sizing.md")

RHO_STEEL = 7850.0
ETA_BS = 0.9
MU_RAIL = 0.01          # rolling friction coefficient of profile rails (ASSUMPTION)
F_SEAL = 10.0           # N, block seals + wipers per axis (ASSUMPTION)
A_TRAVEL = 1.0          # m/s^2 travel acceleration (ASSUMPTION, conservative for 25 kg)
V_DRAG = 80.0           # mm/s drag speed
SF = 1.5

lines = []
p = lines.append


def table(h, rows):
    p("| " + " | ".join(h) + " |")
    p("|" + "|".join("---" for _ in h) + "|")
    for r in rows:
        p("| " + " | ".join(r) + " |")
    p("")


def critical_rpm(d_root_mm, length_mm, support="fixed-supported"):
    """THK A15-32: N1 = lambda2 * d_r / L^2 * 1e7 [rpm]; lambda2 already includes the 0.8 factor
    (research/components.md). d_r (thread minor diameter) is estimated as nominal - ball dia."""
    lam = {"fixed-fixed": 21.9, "fixed-supported": 15.1, "supported-supported": 9.7, "fixed-free": 3.4}[support]
    return lam * d_root_mm / length_mm ** 2 * 1e7


def stepper_torque(n_rpm, t_hold):
    """Generic closed-loop NEMA23 curve (ASSUMPTION): linear drop to 35 % at 1500 rpm.
    Catalogue holding torques found: Leadshine CS-M22331 3.1 N m (0-1900 rpm range),
    57HSE3N 3 N m rated 1000 rpm; torque-at-speed values were NOT found."""
    return t_hold * max(0.2, 1.0 - 0.65 * min(n_rpm, 2000.0) / 1500.0)


p("# Gantry motor sizing — generated")
p("")
p("> `python3 calc/gantry_motor_sizing.py`. 절삭력·질량·마찰·스테퍼 토크곡선은 **ASSUMPTION**.")
p("")
p("## 1. 가동 질량 (cartesian_model.MASS, CAD 전 추정)")
p("")
table(["축", "움직이는 질량 [kg]", "구성"], [
    ["Z", f"{cm.moving_mass('Z'):.1f}", "식품 모듈 + 헤드 + Z 기둥"],
    ["X", f"{cm.moving_mass('X'):.1f}", "Z 질량 + Z 구동 + X 캐리지"],
    ["Y (제품: 이동 브리지)", f"{cm.moving_mass('Y'):.1f}", "X 질량 + 브리지 빔·레일·스크류"],
])

# ------------------------------------------------------------------ X axis
p("## 2. X_M (브리지 축) — 아키텍처 1: 이동 + 드래그 겸용")
p("")
L_SCREW = 1150.0
screws = [("SFU1610", 16.0, 12.8, 10.0), ("SFU2010", 20.0, 16.8, 10.0), ("SFU2020", 20.0, 16.8, 20.0)]   # d_r estimated
rows = []
for name, d, dr, lead in screws:
    nc = critical_rpm(dr, L_SCREW)
    n_dn = 70000.0 / d            # THK: N2 = 70,000 / D (D ~ nominal dia, ball-centre dia)
    n_max = min(nc, n_dn)
    v_max = n_max * lead / 60.0
    rows.append([name, f"{L_SCREW:.0f}", f"{nc:.0f}", f"{n_dn:.0f}", f"{n_max:.0f}", f"{v_max:.0f}"])
table(["스크류", "길이 [mm]", "N1 임계 (고정-지지, 0.8 포함) [rpm]", "DN 70,000 한계 [rpm]", "허용 [rpm]", "최대 이송 [mm/s]"], rows)
p("→ 1.15 m X축에서 SFU1610은 임계속도로 ~245 mm/s, SFU2010은 ~320 mm/s까지 가능(스테퍼 토크 때문에 실제로는 1,500 rpm = 250 mm/s에서 제한). SFU2020은 이송이 빠르지만 드래그 토크가 2배. 벨트는 드래그 하중 경로에 들어가 강성이 부족(calc/output/scoop_load_path.md). d_r(골지름)은 호칭경 − 볼 지름 추정값.")
p("")

m_x = cm.moving_mass("X")
rows = []
for name, d, dr, lead in screws[1:]:
    j_screw = math.pi * RHO_STEEL * (L_SCREW / 1e3) * (d / 1e3) ** 4 / 32.0
    j_rotor = 4.8e-5                                   # kg m^2, NEMA23 (ASSUMPTION)
    alpha = A_TRAVEL * 2 * math.pi / (lead / 1e3)
    f_fric = MU_RAIL * m_x * 9.81 + F_SEAL
    t_load_acc = (m_x * A_TRAVEL + f_fric) * (lead / 1e3) / (2 * math.pi * ETA_BS)
    t_inertia = (j_screw + j_rotor) * alpha
    n_travel = min(critical_rpm(dr, L_SCREW), 70000.0 / d, 1500.0)   # also cap at 1500 rpm (stepper)
    t_travel = t_load_acc + t_inertia
    for F in cm.F_X_CASES:
        t_drag = (F + f_fric) * (lead / 1e3) / (2 * math.pi * ETA_BS)
        n_drag = V_DRAG / lead * 60.0
        rows.append([name, f"{F:.0f}", f"{t_drag:.3f} @ {n_drag:.0f}", f"{t_travel:.3f} @ {n_travel:.0f}",
                     f"{max(SF*t_drag/stepper_torque(n_drag, 2.0), SF*t_travel/stepper_torque(n_travel, 2.0))*100:.0f} %"])
table(["스크류", "F_drag [N]", "드래그 토크 [N·m @ rpm]", "이송 가속 토크 [N·m @ rpm]", "NEMA23 2 N·m 대비 사용률(SF1.5)"], rows)
p(f"가정: 이송 가속 {A_TRAVEL} m/s², 레일 구름마찰 μ = {MU_RAIL}, 씰 {F_SEAL:.0f} N, 드래그 {V_DRAG:.0f} mm/s.")
p("→ **X: SFU2010 + 폐루프 NEMA23(2 N·m급)** 으로 200 N 드래그와 ~250 mm/s 이송을 함께 만족(가정 토크곡선 기준, 사용률 ~35 %).")
p("")

# ------------------------------------------------------------------ Y axis
p("## 3. Y_M")
p("")
p("### 3.1 V1 — 고정 빔 + 짧은 Y 크로스슬라이드(레인 ±80 mm)")
m_ycs = cm.moving_mass("Z") + cm.MASS["z_drive"]
f_side = cm.K_SIDE * max(cm.F_X_CASES)
t_hold = f_side * 0.005 / (2 * math.pi * ETA_BS)
p(f"- 가동질량(Z 기둥 + 헤드 + Z 구동) ≈ {m_ycs:.1f} kg, 옆힘 F_y = {cm.K_SIDE}·F_x = {f_side:.0f} N")
p(f"- SFU1605(150 mm) 유지토크 = {t_hold:.3f} N·m → NEMA17(0.45 N·m)로 충분. 크로스슬라이드 레일은 드래그 힘 200 N과 모멘트 161 N·m를 **가로방향으로** 받으므로 HGR15 2레일 × 2블록(간격 ≥ 120 mm).")
p("")
p("### 3.2 제품 — 이동 브리지(2열 이상)")
m_y = cm.moving_mass("Y")
r_pulley = 15.9                                         # mm, HTD 5M 20T pitch radius
for ratio in (1.0, 3.0):
    f_acc = m_y * A_TRAVEL + MU_RAIL * m_y * 9.81 + 2 * F_SEAL
    t = f_acc * r_pulley / 1e3 / 0.95 / ratio
    t_hold_y = f_side * r_pulley / 1e3 / 0.95 / ratio
    p(f"- HTD 5M-15 벨트, 풀리 20T, 감속 {ratio:.0f}:1 → 가속 토크 {t:.2f} N·m, 옆힘 유지 {t_hold_y:.2f} N·m (모터축)")
p("- 드래그 방향이 X(빔 축)이므로 Y 구동은 **옆힘(가정 60 N)만** 받는다 → 벨트 허용(스트레치 ~0.1 mm, HTD 5M-15 강성 가정). 브리지 래킹 방지: 양쪽 벨트를 크로스샤프트로 동기화한 **단일 모터**.")
p("")

# ------------------------------------------------------------------ Arch 2
p("### 3.3 벨트 강도 점검 (Gates 자료 환산, research/components.md)")
p("")
p("- GT2 6 mm 권장 작업장력 ≈ 26–28 N, 10 mm ≈ 44 N → **드래그 200 N을 벨트로 전달하는 것은 강도부터 불가**(아키텍처 1의 X축에 GT2 금지).")
p("- HTD 5M 15 mm 작업장력 ≈ 268 N(환산, UNCERTAIN) → Y 브리지의 가속(~40 N)·옆힘(60 N)에는 충분, 드래그 축으로는 여유 부족.")
p("")

p("## 4. 아키텍처 2 비교 — 벨트 XY + 로컬 x_scoop")
p("")
p("- 로컬 x_scoop: SFU1610 250 mm + 24 V DC 60 W(레거시 calc §2와 동일): 200 N @ 0.354 N·m.")
p("- 전역 X/Y는 벨트(빠름, 저가)지만 **드래그 중 반력이 여전히 X 캐리지를 통과**하므로 벨트 대신 받을 **레일 클램프/인덱스 핀**이 필요(없으면 GT2에서 +2.3 mm, calc/output/scoop_load_path.md).")
p("- 모터 수: X, Y, Z, x_s, θ = **5** (+ 잠금 액추에이터 1–2) vs 아키텍처 1: X, Y, Z, θ = **4**.")
p("")

# ------------------------------------------------------------------ max force
p("## 5. 각 구동계가 낼 수 있는 최대 힘 (안전 입력)")
p("")
rows = []
for name, t_peak, lead_or_r, kind in (("X: SFU2010 + NEMA23 2 N·m", 2.0, 10.0, "screw"),
                                      ("X: 전류 70 % 제한", 1.4, 10.0, "screw"),
                                      ("Y 크로스슬라이드: SFU1605 + NEMA17 0.45 N·m", 0.45, 5.0, "screw"),
                                      ("Y 브리지: HTD 20T, 3:1, NEMA23 2 N·m", 2.0, 15.9, "belt3")):
    if kind == "screw":
        f = 2 * math.pi * ETA_BS * t_peak / (lead_or_r / 1e3)
    else:
        f = t_peak * 3.0 * 0.95 / (lead_or_r / 1e3)
    rows.append([name, f"{f:.0f}"])
table(["구동", "최대 힘 [N]"], rows)
p("모든 구동이 ISO/TS 15066 손 준정적 한계 140 N을 넘는다 → **힘 제한은 1차 방호가 될 수 없다.** 작업영역 인클로저 + 인터록 + 컵 베이 감지가 1차(docs/collision_and_safety.md). 드래그 필요 힘(가정 200 N)을 내야 하므로 구동력을 140 N 아래로 낮추는 것도 불가능하다.")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {OUT}")
