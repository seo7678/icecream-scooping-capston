"""Z axis sizing and surface-detection resolution.

Run:  python3 calc/z_axis_sizing.py
Out:  calc/output/z_axis_sizing.md

Questions:
  1. What screw / motor / brake / counterbalance does Z need?
  2. Which surface-detection method is good enough? The detection threshold
     force pushes the rim into the ice cream before contact is declared; that
     indentation becomes a depth (portion) error.
All material numbers are ASSUMPTIONS (probe pressure from research/papers.md B3
is a -20 degC probe value used only as an order of magnitude).
"""

import math
import os

import cartesian_model as cm

OUT = os.path.join(os.path.dirname(__file__), "output", "z_axis_sizing.md")
ETA = 0.9
G = 9.81

lines = []
p = lines.append


def table(h, rows):
    p("| " + " | ".join(h) + " |")
    p("|" + "|".join("---" for _ in h) + "|")
    for r in rows:
        p("| " + " | ".join(r) + " |")
    p("")


m_z = cm.moving_mass("Z")
W = m_z * G
z_top = cm.z_safe(cm.CUP_RIM_Z_RAISED) + 20.0
stroke = z_top - cm.z_min() + 20.0

p("# Z axis sizing & surface detection — generated")
p("")
p("> `python3 calc/z_axis_sizing.py`. 질량·절삭력·압입압력은 **ASSUMPTION**.")
p("")
p("## 1. 행정·하중")
p("")
table(["항목", "값", "근거"], [
    ["Z 가동질량", f"{m_z:.1f} kg → 자중 {W:.0f} N", "cartesian_model.MASS"],
    ["Z_SAFE (컵이 데크 위 80 mm)", f"{cm.z_safe(cm.CUP_RIM_Z_RAISED):.0f} mm", "장애물 + R + 25 mm"],
    ["Z_SAFE (컵 스테이션 매립)", f"{cm.z_safe(cm.CUP_RIM_Z_RECESSED):.0f} mm", "통 테두리만 장애물"],
    ["가장 깊은 피벗 높이 Z_min", f"{cm.z_min():.0f} mm", "통 깊이 250, 바닥 여유 10"],
    ["필요 행정", f"{stroke:.0f} mm → **400 mm**", "Z_top − Z_min + 여유"],
    ["절삭 수직력 F_z", f"±{cm.K_VERT*200:.0f} N @ F_x = 200 N", "A04"],
])

p("## 2. 스크류·모터·브레이크")
p("")
lead = 10.0                    # SFU1610 vertical
v_fast, a_z = 150.0, 1.0
n_fast = v_fast / lead * 60
rows = []
for cb_label, cb in (("counterbalance 없음", 0.0), ("counterbalance = 자중 × 1.05", W * 1.05)):
    net_down = W - cb                                  # + means gravity pulls down
    f_up_cut = cm.K_VERT * 200.0                       # cutting pushes scoop up
    f_worst = max(abs(net_down) + m_z * a_z, abs(net_down - f_up_cut) + m_z * a_z, f_up_cut + abs(net_down))
    t = f_worst * lead / 1e3 / (2 * math.pi * ETA)
    t_back = abs(net_down) * lead / 1e3 * ETA / (2 * math.pi)   # torque needed to hold against backdriving
    rows.append([cb_label, f"{net_down:+.0f}", f"{f_worst:.0f}", f"{t:.3f}", f"{t_back:.3f}"])
table(["구성", "순 중력 [N, +아래]", "최악 축력 [N]", "모터 토크 [N·m]", "무전원 역구동 토크 [N·m]"], rows)
p(f"- SFU1610, {v_fast:.0f} mm/s → {n_fast:.0f} rpm, 500 mm 길이면 임계속도 여유 큼.")
p("- 볼스크류는 역구동된다 → **스프링 작동 전자 브레이크 필수**(전원 차단 시 잠김).")
p("- counterbalance를 자중보다 약간 크게(×1.05) 하면 전원·브레이크 동시 고장에서도 헤드가 **천천히 올라간다(fail-safe up)**. 이때 모터 토크 요구가 줄어 **NEMA17급(0.45 N·m)으로 충분** → 모터가 낼 수 있는 하강력도 작아진다(아래).")
p("")
for label, t_peak in (("NEMA23 2 N·m", 2.0), ("NEMA17 0.45 N·m", 0.45)):
    f_crush = 2 * math.pi * ETA * t_peak / (lead / 1e3)
    p(f"- {label} 최대 하강력 ≈ {f_crush:.0f} N (+ counterbalance 없을 때 자중 {W:.0f} N)")
p("")
p("→ 선정: **SFU1610 400 mm + NEMA17 폐루프(또는 NEMA23 전류제한) + 스프링 브레이크 + 가스스프링/정하중스프링 counterbalance.** 하강력 ~250 N도 손 한계 140 N을 넘으므로 컵 베이·인클로저 분리가 필요.")
p("")

p("## 3. 표면 검출 — 임계힘 → 압입 깊이 → portion 오차")
p("")
p("rim 최저점이 먼저 닿는 자세(opening 앞-아래, 공격각 α). 접촉 곡률반경 ρ = R / cos α, 날 두께 t,")
p("유효 압입압력 p일 때 접촉 길이 l = F/(p·t), 압입 δ = l²/(8ρ). (얇은 날끝이 닿는 선접촉 근사, ASSUMPTION)")
p("")
alpha = math.radians(cm.ATTACK_DEG)
rho = cm.R_SCOOP / math.cos(alpha)
t_rim = 1.0
sens = 0.065    # d(ln A)/dd per mm near d = 22 mm (calc/output/portion_model.md, 5-7 %/mm)
L_VALID = 60.0   # mm, beyond this contact length the line-contact model is meaningless
methods = [
    ("Z축 인라인 로드셀(가이드된 축방향 마운트)", 3.0, "±0.5 N 노이즈, 3 N 임계"),
    ("스프링 예압 + 마이크로스위치(1–2 mm 행정, 하드스톱)", 10.0, "예압 10 N"),
    ("Z 모터 전류(폐루프 서보/BLDC)", 30.0, "마찰·가속 변동 ±10–20 N → 임계 30 N"),
    ("스테퍼 StallGuard류 부하 추정", 60.0, "저속에서 부정확, 임계 ≥ 50 N"),
]
rows = []
for p_ind in (0.3, 1.0):
    for name, F, note in methods:
        l = F / (p_ind * t_rim)
        if l > L_VALID:
            d_min = L_VALID ** 2 / (8 * rho)
            rows.append([name, f"{p_ind}", f"{F:.0f}", f"> {d_min:.0f} (모델 범위 밖)", f"> {d_min*sens*100:.0f} %", note])
            continue
        delta = l ** 2 / (8 * rho)
        rows.append([name, f"{p_ind}", f"{F:.0f}", f"{delta:.2f}", f"{delta*sens*100:.1f} %", note])
table(["방법", "압입압력 p [MPa]", "임계힘 [N]", "압입 δ [mm]", "portion 오차", "비고"], rows)
p("- 비접촉(ToF/초음파) 거리센서: 접촉력 0이지만 성에·결로·흰/검은 표면 반사 차이로 mm급 정확도를 보장하기 어렵다(미검증) → 보조로만.")
p("- 검출 지연: 접근속도 v × 지연(필터 20 ms + 루프 1 ms). v = 10 mm/s → 0.2 mm, v = 50 mm/s → 1.1 mm 과주행.")
p("")
p("- 압입 δ의 평균은 **보정 가능한 편향**이다(측정된 δ만큼 Z0를 올려 잡으면 됨). 보정할 수 없는 것은 경도·표면 상태에 따른 **δ의 흩어짐**이고, 그 크기는 δ 자체에 비례한다 → 임계힘을 낮출수록 흩어짐의 절대값이 작아진다.")
p("")
p("**결론:** 압입압력이 낮은(부드러운) 경우까지 portion 흩어짐을 수 % 이내로 두려면 **임계 ≤ 3–5 N**이 필요하다. 모터 전류(≥ 30 N)나 스테퍼 부하 추정은 모델 범위를 넘을 만큼 깊게 눌러 버려 표면 기준으로 쓸 수 없다. 스프링 스위치(10 N)는 단단한 제품(p ≈ 1 MPa)에서만 충분하다. → **Z 인라인 로드셀(가이드된 축방향 마운트, 3 N 임계) + 최종 10 mm를 10 mm/s로 접근**을 기본안으로 한다. 압입압력 p와 δ는 E0(drag test) 날에 함께 측정한다.")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {OUT}")
