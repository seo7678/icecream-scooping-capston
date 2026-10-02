"""Quasi-static simulation of the drag phase under three control strategies.

Run:  python3 calc/load_adaptive_sim.py
Out:  calc/output/load_adaptive_sim.md, calc/output/fig_load_adaptive.png

Purpose: decide, BEFORE building, which adaptation is worth implementing.
  S0  fixed path, fixed speed                 (no adaptation, hardware current limit only)
  S1  fixed path, speed adaptation + reverse  (all a 1-motor cam head can do, Arch. B)
  S2  depth adaptation + volume integration   (needs an independent pitch axis, Arch. A)

Material and drive numbers are ASSUMPTIONS (calc/scoop_model.py, docs/13).
What the simulation can show: how the ranking of S0/S1/S2 depends on the
material hardness u and rate exponent n -- the quantities experiment P1-1 measures.
It cannot show absolute forces or times; those are only as good as the assumptions.
"""

import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import scoop_model as sm  # noqa: E402

OUT_MD = os.path.join(os.path.dirname(__file__), "output", "load_adaptive_sim.md")
OUT_FIG = os.path.join(os.path.dirname(__file__), "output", "fig_load_adaptive.png")

DT = 0.001
R = sm.R_BOWL_MM
V_TARGET = sm.ball_volume_cm3() * 1e3
L_MAX = 190.0                 # usable chord in the tub (ASSUMPTION)
D_FIXED = 22.0                # fixed-path depth, chosen so the medium case fills in < L_MAX
D_MAX_S2 = 28.0               # deepest cut S2 may use (h set by the Z lock)
X_ENTRY = 30.0                # dive length
T_LIMIT = 8.0                 # time budget for the drag phase [s]

# drive: SFU1610 + 24 V DC motor with hardware current limit
LEAD, ETA, KT, I0, I_LIM = 10.0, 0.9, 0.05, 0.4, 7.5
F_HW = 2 * math.pi * ETA * KT * (I_LIM - I0) / (LEAD / 1e3)   # ~200 N

# force estimate from motor current (docs/14): gain error + first-order low-pass
PROXY_GAIN = 0.90             # ASSUMPTION: estimate reads 10 % low
LPF_TAU = 0.02
V_NOM, V_MIN = 80.0, 10.0
# Thresholds live in ESTIMATE units and must sit below (1 - worst proxy error) * F_HW.
# The first draft used F_STOP = 185 N: with a proxy reading 10 % low it could never
# trip, so every overload ended as a hardware stall instead of a controlled reverse.
F_STOP = 0.80 * F_HW
F_LO, F_HI = 0.45 * F_HW, 0.70 * F_HW
F_TARGET = 0.55 * F_HW
THETA_RATE_MAX = 3.0          # rad/s, pitch-axis speed limit


def theta_max(h):
    return math.acos(min(0.999, h / R)) - 0.02


def entry_floor(x, h):
    """Minimum pitch during the dive: the rim may not go deeper faster than this ramp."""
    if x >= X_ENTRY:
        return 0.0
    return theta_max(h) * (1.0 - x / X_ENTRY)


def u_profile(x, u0, inclusion):
    if inclusion == "chunk" and 110.0 <= x <= 125.0:
        return u0 * 2.5       # frozen chunk / ice-crystal layer (ASSUMPTION)
    if inclusion == "jam" and x >= 140.0:
        return u0 * 8.0       # hard obstacle (tub wall, fused block) (ASSUMPTION)
    return u0


def force_at(u, area, v, n):
    return u * 1e-3 * area * (max(v, 2.0) / sm.V_REF_MM_S) ** n


def solve_speed(u, area, v_cmd, n):
    """Feed speed reached: the command, unless the hardware current limit binds."""
    if force_at(u, area, v_cmd, n) <= F_HW:
        return v_cmd
    if force_at(u, area, 2.0, n) > F_HW:
        return 0.0
    lo, hi = 2.0, v_cmd
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if force_at(u, area, mid, n) > F_HW else (mid, hi)
    return lo


def fixed_path_length(h):
    vol, x = 0.0, 0.0
    while vol < V_TARGET and x < L_MAX:
        vol += sm.swept_area_mm2(entry_floor(x, h), h, R) * 0.1
        x += 0.1
    return x


H_FIXED = R - D_FIXED
H_S2 = R - D_MAX_S2
L_FIXED = fixed_path_length(H_FIXED)


def run(strategy, u0, n, inclusion, trace=False):
    h = H_S2 if strategy == "S2" else H_FIXED
    t = x = vol = f_est = f_peak = over_t = stall_t = integ = 0.0
    theta = theta_max(h)
    retries = 0
    reversing_to = None
    events = []
    tr = {"t": [], "F": [], "v": [], "d": []}
    while t < T_LIMIT:
        if strategy != "S2":
            theta = entry_floor(x, h)
        area = sm.swept_area_mm2(theta, h, R)
        u = u_profile(x, u0, inclusion)
        if reversing_to is not None:
            v_cmd = -40.0
        elif strategy == "S1":
            k = min(1.0, max(0.0, (F_HI - f_est) / (F_HI - F_LO)))
            v_cmd = V_MIN + (V_NOM - V_MIN) * k
        else:
            v_cmd = V_NOM
        if v_cmd >= 0:
            v = solve_speed(u, area, v_cmd, n)
            f = min(force_at(u, area, v, n), F_HW) if area > 0 else 0.0
        else:
            v, f = v_cmd, 0.0
        f_est += (PROXY_GAIN * f - f_est) * DT / LPF_TAU
        f_peak = max(f_peak, f)

        if strategy == "S2" and reversing_to is None:
            e = f_est - F_TARGET
            if f_est > F_STOP:
                dtheta = THETA_RATE_MAX          # overload: pitch up at full rate
            else:
                integ = max(-50.0, min(50.0, integ + e * DT))
                dtheta = max(-THETA_RATE_MAX, min(THETA_RATE_MAX, 0.03 * e + 0.05 * integ))
            theta = max(entry_floor(x, h), min(theta_max(h), theta + dtheta * DT))

        x += v * DT
        if v > 0:
            vol += area * v * DT
        t += DT
        if trace:
            tr["t"].append(t)
            tr["F"].append(f)
            tr["v"].append(v)
            tr["d"].append(max(0.0, sm.cut_depth_mm(theta, h, R)))

        if reversing_to is not None:
            if x <= reversing_to:
                reversing_to = None
                over_t = 0.0
            continue
        # S1 reverses on overload; S2 reverses only once it can no longer pitch up
        relief_left = strategy == "S2" and theta < theta_max(h) - 1e-3
        if strategy in ("S1", "S2") and not relief_left:
            over_t = over_t + DT if f_est > F_STOP else 0.0
            if over_t > 0.05:
                retries += 1
                events.append("overload")
                if retries > 2:
                    events.append("JAM-abort")
                    break
                reversing_to = x - 5.0
                continue
        stall_t = stall_t + DT if (v <= 0.0 and not relief_left) else 0.0
        if stall_t > 0.3:
            events.append("STALL")
            break
        if strategy == "S2":
            if vol >= V_TARGET or x >= L_MAX:
                break
        elif x >= L_FIXED:
            break
    frac = vol / V_TARGET
    if t >= T_LIMIT:
        events.append("TIMEOUT")
    ok = frac >= 0.95 and not any(e in ("STALL", "JAM-abort", "TIMEOUT") for e in events)
    if frac < 0.95 and not any(e in ("STALL", "JAM-abort", "TIMEOUT") for e in events):
        events.append("SHORT")
    return {"t": t, "F_peak": f_peak, "vol_frac": frac, "events": events, "ok": ok, "trace": tr}


def fmt(r):
    ev = sorted(set(r["events"]), key=r["events"].index)
    tag = "✔" if r["ok"] else "✘ " + ",".join(ev)
    return f"{r['F_peak']:.0f} N / {r['t']:.1f} s / {r['vol_frac']*100:.0f} % {tag}"


lines = []
p = lines.append
p("# Load-adaptive drag simulation — generated output")
p("")
p("> 생성: `python3 calc/load_adaptive_sim.py`. 재료(u, n)·구동계·proxy 오차는 모두 **ASSUMPTION**.")
p("> 이 표는 절대값이 아니라 **전략 간 순위가 어떤 조건에서 뒤집히는지**를 보기 위한 것이다.")
p("")
p(f"- 고정경로(S0/S1): 깊이 {D_FIXED:.0f} mm, 진입 {X_ENTRY:.0f} mm dive, stroke {L_FIXED:.0f} mm (medium 제품 기준 설계)")
p(f"- S2: 최대깊이 {D_MAX_S2:.0f} mm, 힘 목표 {F_TARGET:.0f} N(추정치)로 pitch를 조절, 목표부피 {V_TARGET/1e3:.0f} cm³ 도달 또는 chord {L_MAX:.0f} mm에서 종료")
p(f"- 하드웨어 힘 상한 {F_HW:.0f} N(전류제한), proxy gain {PROXY_GAIN}, 과부하 판정 {F_STOP:.0f} N(추정치), 시간예산 {T_LIMIT:.0f} s")
p("")
p("셀 = peak force / drag 시간 / 목표부피 달성률, ✔ = 부피 ≥ 95 % 이고 중단 없음.")
p("")
p("| u case | n | inclusion | S0 fixed | S1 speed-adapt | S2 depth-adapt |")
p("|---|---|---|---|---|---|")
stats = {s: {"ok": 0, "n": 0, "over_target": 0} for s in ("S0", "S1", "S2")}
for case in ("soft", "medium", "hard", "very_hard"):
    for n in sm.RATE_EXPONENT_CASES:
        for inc in ("none", "chunk") + (("jam",) if case == "medium" else ()):
            rs = {s: run(s, sm.U_CASES_KPA[case], n, inc) for s in ("S0", "S1", "S2")}
            for s, r in rs.items():
                stats[s]["n"] += 1
                stats[s]["ok"] += int(r["ok"])
                stats[s]["over_target"] += int(r["F_peak"] > 150.0)
            p(f"| {case} ({sm.U_CASES_KPA[case]:.0f} kPa) | {n} | {inc} | "
              + " | ".join(fmt(rs[s]) for s in ("S0", "S1", "S2")) + " |")
p("")
p("## 요약")
p("")
p("| Strategy | 성공 / 전체 | peak force > 150 N 발생 |")
p("|---|---|---|")
for s in ("S0", "S1", "S2"):
    st = stats[s]
    p(f"| {s} | {st['ok']} / {st['n']} | {st['over_target']} / {st['n']} |")
p("")

with open(OUT_MD, "w", encoding="utf-8") as fh:
    fh.write("\n".join(lines) + "\n")

fig, axes = plt.subplots(3, 1, figsize=(6.4, 6.2), dpi=150, sharex=True)
colors = {"S0": "#888888", "S1": "#c07a2c", "S2": "#2b5c8a"}
for s in ("S0", "S1", "S2"):
    tr = run(s, sm.U_CASES_KPA["hard"], 0.15, "chunk", trace=True)["trace"]
    axes[0].plot(tr["t"], tr["F"], color=colors[s], lw=1.2, label=s)
    axes[1].plot(tr["t"], tr["v"], color=colors[s], lw=1.2)
    axes[2].plot(tr["t"], tr["d"], color=colors[s], lw=1.2)
axes[0].axhline(F_HW, color="#b04a2f", lw=0.8, ls="--")
axes[0].set_ylabel("drag force [N]")
axes[1].set_ylabel("feed v [mm/s]")
axes[2].set_ylabel("cut depth [mm]")
axes[2].set_xlabel("time [s]")
axes[0].legend(fontsize=7, frameon=False, ncol=3)
axes[0].set_title("hard product (160 kPa, n = 0.15), chunk at x = 110-125 mm — ASSUMPTION case", fontsize=8)
for a in axes:
    a.grid(alpha=0.25)
fig.tight_layout()
fig.savefig(OUT_FIG)
print(f"wrote {OUT_MD}\nwrote {OUT_FIG}")
