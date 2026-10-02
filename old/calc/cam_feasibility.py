"""Architecture B feasibility: can one motor + a fixed track cam produce theta(x)?

Run:  python3 calc/cam_feasibility.py
Out:  calc/output/cam_feasibility.md

Mechanism (docs/11 §B): the sweep carriage is driven by the single motor. A
follower lever of length a, rigidly tied (via the push-rod parallelogram) to
the scoop pitch, carries a roller that runs in a track fixed to the head frame.
With the lever tangent perpendicular to the stroke, the pressure angle is

    tan(alpha) = a * dtheta/dx

and the forces are
    F_n        = tau / (a * cos(alpha))       (roller-track contact force)
    F_x,extra  = tau * dtheta/dx              (added load on the sweep drive)

so the closing work tau*dtheta must be paid by the sweep motor over the
closing travel L_close.
"""

import math
import os

OUT = os.path.join(os.path.dirname(__file__), "output", "cam_feasibility.md")

DTHETA = math.radians(120.0)   # closing rotation
TAU = 5.25                     # N·m, R = 35 mm, F = 150 N (ASSUMPTION case, docs/13 §4)
ALPHA_LIM = 30.0               # deg, conservative pressure-angle limit for a roller follower

lines = []
p = lines.append
p("# Architecture B — track-cam feasibility (generated)")
p("")
p(f"> Δθ_close = 120°, τ_close = {TAU} N·m (ASSUMPTION case). 압력각 한계 {ALPHA_LIM:.0f}° (보수적 설계값).")
p("")
p("| lever a [mm] | L_close [mm] | 압력각 α [°] | F_n roller [N] | sweep 축 추가 힘 [N] | 판정 |")
p("|---|---|---|---|---|---|")
for a in (15.0, 20.0, 30.0, 40.0):
    for L in (30.0, 50.0, 75.0, 100.0, 150.0):
        k = DTHETA / L                       # rad/mm
        alpha = math.degrees(math.atan(a * k))
        fn = TAU / (a / 1e3 * math.cos(math.radians(alpha)))
        fx = TAU * k * 1e3
        ok = "OK" if alpha <= ALPHA_LIM else "✘ 압력각 초과"
        p(f"| {a:.0f} | {L:.0f} | {alpha:.0f} | {fn:.0f} | {fx:.0f} | {ok} |")
p("")
lmin = {a: a * DTHETA / math.tan(math.radians(ALPHA_LIM)) for a in (15.0, 20.0, 30.0, 40.0)}
p("최소 closing 이동거리 L_close,min = a·Δθ / tan α_lim:")
p("")
p("| a [mm] | L_close,min [mm] |")
p("|---|---|")
for a, v in lmin.items():
    p(f"| {a:.0f} | {v:.0f} |")
p("")
p("결론:")
p("- 사람처럼 **짧은 거리(≤30 mm)에서 손목을 확 꺾어 닫는 동작은 고정 트랙 캠으로 만들 수 없다**(레버 15–40 mm에서 압력각 46–70° → 걸림 위험).")
p("- 레버를 15–20 mm로 줄이면 closing을 55–75 mm에 걸쳐 '굴리며 닫는(rolling close)' 동작으로 만들 수 있지만,")
p("  롤러 접촉력이 ~290–380 N으로 커지고, sweep 모터가 closing 일(τ·Δθ ≈ 11 J)을 떠안아 closing 구간(75–100 mm)에서 sweep 축 요구력이 110–150 N 늘어난다.")
p("- 대안은 시간 기반 캠(크랭크-슬라이더의 사점 dwell 동안 판캠이 θ를 돌림)이다. 1모터는 유지되지만 캠축·링크가 추가된다.")
p("- 따라서 B의 성립 조건은 **V1에서 'rolling close(≥60 mm)'로도 공 형상/성공률이 유지되는가**이다(docs/17, E4).")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {OUT}")
