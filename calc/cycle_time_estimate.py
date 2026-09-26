"""Serving-cycle time of the Cartesian machine, segment by segment.

Run:  python3 calc/cycle_time_estimate.py
Out:  calc/output/cycle_time_estimate.md

Trapezoidal motion profiles per segment. Scenarios:
  layout      V1 (one row) / product (2 x 4)
  tub         nearest / farthest from the cup station
  level       full tub / near-empty tub
  cup station raised (Z_SAFE 140) / recessed (Z_SAFE 60)
  motion      sequential (Z then XY) / blended (X and Y together, XY starts as soon as Z >= Z_SAFE)
All speeds, accelerations and dwell times are ASSUMPTIONS (drive choices from
calc/gantry_motor_sizing.py, z_axis_sizing.py). The manual scooping cycle is
NOT known (P0-1); it is shown only as a parameter to compare against.
"""

import math
import os

import cartesian_model as cm

OUT = os.path.join(os.path.dirname(__file__), "output", "cycle_time_estimate.md")

AX = {  # vmax [mm/s], amax [mm/s^2]
    "X": (250.0, 1000.0),       # SFU2010 + NEMA23
    "Y_v1": (100.0, 1000.0),    # short cross-slide SFU1605
    "Y_prod": (400.0, 1000.0),  # belt bridge
    "Z": (150.0, 1000.0),       # SFU1610 + counterbalance
}
V_PROBE, L_PROBE = 10.0, 10.0   # final approach speed / distance
T_TOUCH_SETTLE = 0.15           # s, detect + stop + tare
V_DIVE, V_DRAG = 40.0, 80.0     # mm/s along path
T_CLOSE = 0.8                   # s, 120 deg closing
T_EJECT = 1.2                   # s, over-rotate + shake
T_SCALE = 1.0                   # s, scale settle
T_OPERATOR = 3.0                # s, select flavour + place cup + take cup (operator-attended)
DRAG_LEN, DEPTH = 130.0, 25.0   # mm (calc/output/tub_lane_planner.md, centre lane)
Z_DISP_ABOVE_CUP = 15.0         # mm clearance scoop bottom -> cup rim at dispense


def trap(dist, vmax, amax):
    d = abs(dist)
    if d == 0:
        return 0.0
    d_acc = vmax ** 2 / amax
    if d <= d_acc:
        return 2 * math.sqrt(d / amax)
    return 2 * vmax / amax + (d - d_acc) / vmax


def xy_time(p0, p1, yaxis, blended):
    tx = trap(p1[0] - p0[0], *AX["X"])
    ty = trap(p1[1] - p0[1], *AX[yaxis])
    return max(tx, ty) if blended else tx + ty


def cycle(layout, tub, level_depth, cup_rim, blended, yaxis):
    zs = cm.z_safe(cup_rim)
    tub_xy, cup_xy = layout[tub], layout["CUP"]
    seg = []
    # start: head parked above the cup station at Z_SAFE (end of previous cycle)
    seg.append(("XY → tub", xy_time(cup_xy, tub_xy, yaxis, blended)))
    # expected surface known from the cut-history model -> fast descent to 10 mm above it
    c_touch = -level_depth + cm.R_SCOOP * math.cos(math.radians(cm.ATTACK_DEG))
    seg.append(("Z fast descent", trap(zs - (c_touch + L_PROBE), *AX["Z"])))
    seg.append(("Z probe + touch-off", L_PROBE / V_PROBE + T_TOUCH_SETTLE))
    dive = DEPTH / math.sin(math.radians(30.0))
    seg.append(("dive", dive / V_DIVE))
    seg.append(("drag", DRAG_LEN / V_DRAG))
    seg.append(("close", T_CLOSE))
    c_after = c_touch - DEPTH
    seg.append(("Z lift to Z_SAFE", trap(zs - c_after, *AX["Z"])))
    seg.append(("XY → cup", xy_time(tub_xy, cup_xy, yaxis, blended)))
    c_disp = cup_rim + cm.R_SCOOP + Z_DISP_ABOVE_CUP
    seg.append(("Z down to dispense", trap(zs - c_disp, *AX["Z"])))
    seg.append(("eject", T_EJECT))
    seg.append(("scale settle", T_SCALE))
    seg.append(("Z retract", trap(zs - c_disp, *AX["Z"])))
    return seg


lines = []
p = lines.append
p("# Cycle time estimate — generated")
p("")
p("> `python3 calc/cycle_time_estimate.py`. 속도·가속도·체류시간은 **ASSUMPTION**(선정 구동계 기준). 사람 스쿠핑 사이클은 **미측정**(P0-1).")
p("")
p("## 1. 기준 사이클 분해 — V1, TUB2, 절반 수위, 컵 매립(Z_SAFE 60), blended")
p("")
seg = cycle(cm.LAYOUT_V1, "TUB2", 125.0, cm.CUP_RIM_Z_RECESSED, True, "Y_v1")
p("| 구간 | 시간 [s] |")
p("|---|---|")
for name, t in seg:
    p(f"| {name} | {t:.2f} |")
tot = sum(t for _, t in seg)
p(f"| **기계 합계** | **{tot:.1f}** |")
p(f"| (작업자: 맛 선택·컵 놓기/가져가기, 기계와 병렬 아님) | {T_OPERATOR:.1f} |")
p("")

p("## 2. 시나리오 비교 (기계 시간, 초)")
p("")
p("| 배치 | 통 | 수위 | 컵 | 이동 | 기계 [s] | 시간당 스쿱 |")
p("|---|---|---|---|---|---|---|")
results = []
for lay_name, layout, yaxis, tubs in (("V1 1열", cm.LAYOUT_V1, "Y_v1", ("TUB3", "TUB1")),
                                      ("제품 2×4", cm.LAYOUT_PRODUCT, "Y_prod", ("TUB4", "TUB5"))):
    for tub in tubs:
        for lvl, lvl_name in ((20.0, "새 통"), (220.0, "거의 빈")):
            for cup, cup_name in ((cm.CUP_RIM_Z_RAISED, "올림"), (cm.CUP_RIM_Z_RECESSED, "매립")):
                for blended in (False, True):
                    t = sum(x for _, x in cycle(layout, tub, lvl, cup, blended, yaxis))
                    results.append(t)
                    p(f"| {lay_name} | {tub} | {lvl_name} | {cup_name} | {'blended' if blended else 'sequential'} | {t:.1f} | {3600/(t+T_OPERATOR):.0f} |")
p("")
p(f"범위: **{min(results):.1f}–{max(results):.1f} s/스쿱** (기계). 작업자 {T_OPERATOR:.0f} s를 더한 시간당 처리량은 표의 마지막 열.")
p("")

p("## 3. 무엇이 시간을 지배하나")
p("")
seg_far = cycle(cm.LAYOUT_V1, "TUB1", 220.0, cm.CUP_RIM_Z_RAISED, False, "Y_v1")
tot_far = sum(t for _, t in seg_far)
p("| 구간(최악: V1 TUB1, 거의 빈 통, 컵 올림, sequential) | 시간 [s] | 비율 |")
p("|---|---|---|")
for name, t in sorted(seg_far, key=lambda s: -s[1]):
    p(f"| {name} | {t:.2f} | {t/tot_far*100:.0f} % |")
p(f"| 합계 | {tot_far:.1f} | |")
p("")
p("개선 순서: (1) 컵 스테이션 매립(Z_SAFE 140 → 60)으로 Z 이동 단축, (2) X·Y 동시 이동, (3) 컵 스테이션을 통 열 가운데에 배치, (4) Z 리드 20 mm, (5) 저울 안정화 시간을 다음 사이클의 XY 이동과 겹치기(측정은 헤드가 떠난 뒤 확정).")
p("")
p("## 4. 멀티 스쿱 주문")
p("")
for n, label in ((1, "싱글"), (2, "더블"), (3, "파인트(3맛)"), (4, "쿼터(4맛)")):
    t = n * (tot + 0.0)
    p(f"- {label}: 스쿱 {n}회 ≈ {t:.0f} s (+ 맛 교체 헹굼 시 회당 ~4 s, + 파인트 이상은 눌러담기 수작업)")
p("")
p("**판단:** 기계 1스쿱 ≈ 15–25 s(가정)는 숙련자의 손 스쿠핑보다 느릴 가능성이 크다(사람 값 미측정). 기계의 가치는 속도가 아니라 **작업자 손이 비는 시간**(그동안 결제·토핑·포장)과 부하 제거에서 나와야 한다. 사이클타임은 설계 KPI로 두고 E2에서 잰다.")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {OUT}")
