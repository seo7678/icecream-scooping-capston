"""Portion-mass model and error budget for a volumetric (depth x length) scoop.

Run:  python3 calc/portion_model.py
Out:  calc/output/portion_model.md, calc/output/fig_portion_tradeoff.png

Model: m = rho * k_p * A(d) * L
  rho : product density (overrun)             -- measure (P0-5)
  k_p : packing factor ball vs tub (voids, compression of the curl) -- measure
  A(d): swept cross-section at depth d        -- geometry (scoop_model)
  L   : stroke length                          -- machine-controlled

First-order error propagation (independent errors):
  CV_m^2 = CV_rho^2 + CV_kp^2 + (dlnA/dd * sigma_d)^2 + (sigma_L / L)^2
"""

import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import scoop_model as sm  # noqa: E402

OUT_MD = os.path.join(os.path.dirname(__file__), "output", "portion_model.md")
OUT_FIG = os.path.join(os.path.dirname(__file__), "output", "fig_portion_tradeoff.png")

R = sm.R_BOWL_MM
V_TARGET_MM3 = sm.ball_volume_cm3() * 1e3


def area(d):
    return sm.swept_area_mm2(0.0, R - d, R)


def dlnA_dd(d, eps=0.01):
    return (math.log(area(d + eps)) - math.log(area(d - eps))) / (2 * eps)


lines = []
p = lines.append

p("# Portion model — generated output")
p("")
p("> 생성: `python3 calc/portion_model.py`. ρ, k_p, σ_d는 ASSUMPTION(측정 전).")
p("")
p("## 1. 깊이 민감도: 얕고 긴 stroke vs 깊고 짧은 stroke")
p("")
p(f"R = {R:.0f} mm, 목표 V = {V_TARGET_MM3/1e3:.0f} cm³ (115 g @ ρ=0.65).")
p("")
p("| d [mm] | A [mm²] | L for target [mm] | ∂lnA/∂d [%/mm] | CV_m from σ_d=0.5 mm | σ_d=1 mm | σ_d=2 mm | F at u=90 kPa [N] |")
p("|---|---|---|---|---|---|---|---|")
for d in (10, 15, 20, 25, 30, 35):
    a = area(d)
    s = dlnA_dd(d) * 100
    L = V_TARGET_MM3 / a
    F = 90e-3 * a
    p(f"| {d} | {a:.0f} | {L:.0f} | {s:.1f} | {0.5*s:.1f} % | {1*s:.1f} % | {2*s:.1f} % | {F:.0f} |")
p("")
p("핵심 결과:")
p("- 같은 부피를 뜰 때 **peak force ∝ A(d)**, **stroke ∝ 1/A(d)** 이다. 따라서 총 절삭일 W = F·L = u·V는 d 선택과 무관하게 **u·V로 거의 고정**된다.")
p("  → 기계는 필요한 총 일을 줄이지 못한다. 줄일 수 있는 것은 (1) 사람에게서 모터로 옮기는 것, (2) peak force와 stroke 길이의 교환, (3) u 자체(온도·날·마찰)뿐이다.")
p("- 깊이 오차 1 mm에 대한 질량 민감도는 d=15 mm에서 ~9 %/mm, d=30 mm에서 ~4 %/mm → **깊게 자를수록 정량성이 좋고, 대신 peak force가 커진다.**")
p("")

p("## 2. 기계식 정량의 오차 예산 (volumetric floor)")
p("")
p("| 시나리오 | CV_ρ | CV_kp | σ_d [mm] @ d=20 | σ_L [mm] | 예측 CV_m |")
p("|---|---|---|---|---|---|")
scen = [
    ("낙관 (균일 제품, 정밀 기준면)", 0.02, 0.03, 0.5, 1.0),
    ("기준 (ASSUMPTION)", 0.03, 0.05, 1.0, 2.0),
    ("비관 (표면 요철·기포 편차 큼)", 0.05, 0.08, 2.0, 3.0),
]
d0 = 20.0
L0 = V_TARGET_MM3 / area(d0)
for name, cr, ck, sd, sl in scen:
    cv = math.sqrt(cr ** 2 + ck ** 2 + (dlnA_dd(d0) * sd) ** 2 + (sl / L0) ** 2)
    p(f"| {name} | {cr*100:.0f} % | {ck*100:.0f} % | {sd} | {sl} | **{cv*100:.1f} %** |")
p("")
p("해석:")
p("- 부피를 아무리 정확히 제어해도 **밀도(overrun)·packing 편차는 부피 제어로 제거할 수 없다** → 기계식 정량의 CV 하한은 √(CV_ρ² + CV_kp²).")
p("- 사람의 CV는 **자료 없음**. 사람은 공 크기를 눈으로 보며 보정하므로(시각 피드백) 생각보다 낮을 수 있다 → P0-2에서 반드시 측정.")
p("- 매장이 저울로 최소중량을 맞추는 방식(research/products.md, 미확인)이라면, 정량 가치는 CV 자체보다 **top-up(추가 한 번 더 뜨기) 횟수와 과다배식 g**로 평가해야 한다.")
p("")

p("## 3. 부피 적분형 적응 stroke (Architecture A의 제어 아이디어)")
p("")
p("깊이를 부하에 맞춰 줄이면 A가 작아지고 같은 부피를 채우려면 L이 길어진다. 통 chord 한계 L_max를 넘으면 부족분이 생긴다.")
p("")
p("| F_limit [N] | u [kPa] | 허용 A [mm²] | 가능 d [mm] | 필요 L [mm] | L_max=190 mm에서 달성 부피 [%] |")
p("|---|---|---|---|---|---|")
for Flim in (120.0, 150.0):
    for u in (40.0, 90.0, 160.0, 250.0):
        a_allow = Flim / (u * 1e-3)
        # find d with area(d) = a_allow (bounded by d<=R)
        lo, hi = 0.5, R
        if area(hi) <= a_allow:
            d = hi
        else:
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if area(mid) > a_allow:
                    hi = mid
                else:
                    lo = mid
            d = 0.5 * (lo + hi)
        a = area(d)
        L = V_TARGET_MM3 / a
        frac = min(1.0, a * 190.0 / V_TARGET_MM3)
        p(f"| {Flim:.0f} | {u:.0f} | {a_allow:.0f} | {d:.1f} | {L:.0f} | {frac*100:.0f} % |")
p("")
p("→ '매우 단단함(250 kPa)' 케이스에서는 힘 상한을 지키면 한 번의 stroke로 115 g을 못 채운다. 이때 선택지는")
p("(a) 두 번째 stroke, (b) 작업자에게 '통 템퍼링 필요' 표시, (c) 힘 상한 상향(안전 재평가). 이 판단 자체가 제품 사양이다.")

with open(OUT_MD, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")

# figure: force vs stroke length at constant volume, for several u
fig, ax = plt.subplots(figsize=(6.4, 4.0), dpi=150)
ds = [d / 2 for d in range(10, 71)]
for u, style in ((40.0, ":"), (90.0, "-"), (160.0, "--"), (250.0, "-.")):
    Ls = [V_TARGET_MM3 / area(d) for d in ds]
    Fs = [u * 1e-3 * area(d) for d in ds]
    ax.plot(Ls, Fs, style, color="#2b5c8a", label=f"u = {u:.0f} kPa")
ax.axvline(190, color="#999999", lw=1)
ax.text(194, 385, "tub chord limit\n(ASSUMPTION)", fontsize=7, color="#555555", va="top")
ax.axhline(140, color="#b04a2f", lw=1)
ax.text(262, 147, "ISO/TS 15066 hand/finger quasi-static 140 N", fontsize=7, color="#b04a2f", va="bottom")
ax.set_xlim(80, 420)
ax.set_ylim(0, 420)
ax.set_xlabel("stroke length L for 115 g [mm]")
ax.set_ylabel("peak drag force F [N]")
ax.set_title("Same portion, different depth: peak force vs stroke (R = 35 mm)", fontsize=9)
ax.legend(fontsize=7, frameon=False)
ax.grid(alpha=0.25)
fig.tight_layout()
fig.savefig(OUT_FIG)
print(f"wrote {OUT_MD}\nwrote {OUT_FIG}")
