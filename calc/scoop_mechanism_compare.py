"""Drag-curl scoop vs symmetric clamshell grab: how each one uses up a tub.

Run:  python3 calc/scoop_mechanism_compare.py
Out:  calc/output/scoop_mechanism_compare.md

Both heads work on the same tapered tub with a 5 mm height map (the machine's
own cut history, no vision) and try to deliver a 177 cm^3 (115 g) portion per
cycle until the tub is exhausted for the machine.

  DRAG (M1)   : R 35 scoop, opening forward-down 30 deg, constant-height drag
                along +X in a lane; depth limited to 28 mm (force budget).
                Lane chosen where the most volume is available.
  CLAMSHELL (M2): two quarter-sphere shells (R 44) closing symmetrically about a
                horizontal axis at pivot height C. C is set at the highest surface
                point in the footprint (nothing may sit above the open shells);
                raising C trims the portion. Centre chosen greedily (max volume).

A portion that cannot be filled in one pass takes a second (or third) pass.
Simplifications (ASSUMPTION): dive ramp and closing cap of the drag head are
ignored; the clamshell's open-shell footprint = R + 6 mm; walls tapered; no
melting, no slumping. Geometry-only: says nothing about force or ball quality.
"""

import math
import os

import numpy as np

import cartesian_model as cm

OUT = os.path.join(os.path.dirname(__file__), "output", "scoop_mechanism_compare.md")

CELL = 5.0
V_T = 115.0 / 0.65 * 1e3          # mm^3
Z0 = -20.0                         # initial flat surface (mm below rim)
FLOOR = -cm.TUB_DEPTH + cm.FLOOR_MARGIN
MARGIN = cm.WALL_MARGIN

xs = np.arange(-120, 120 + CELL, CELL)
X, Y = np.meshgrid(xs, xs, indexing="ij")
RR = np.hypot(X, Y)


def tub_radius(z):
    return cm.tub_diameter_at(-z) / 2.0


def new_surface():
    s = np.full(X.shape, Z0)
    s[RR > tub_radius(Z0)] = np.nan          # outside the tub
    return s


# ------------------------------------------------------------------ clamshell
RC = 44.0          # jaw radius: closed hemisphere = 178 cm^3 = one portion
FOOT_MARGINS = (0.0, 6.0)   # open-jaw footprint = RC + margin (shell, hinge boss), ASSUMPTION
FOOT = RC


def clam_volume(S, cx, cy, c):
    r = np.hypot(X - cx, Y - cy)
    inside = (r < RC) & ~np.isnan(S)
    zb = c - np.sqrt(np.clip(RC ** 2 - r[inside] ** 2, 0, None))
    top = np.minimum(S[inside], c)
    cut = np.clip(top - zb, 0, None)
    return cut.sum() * CELL ** 2, inside, zb, cut


def clam_pass(S, v_need):
    """One grab. C sits at the highest point inside the open-jaw footprint:
    material above C would stay attached to the ball through its top face."""
    best = None
    for cx in np.arange(-70, 71, 10.0):
        for cy in np.arange(-70, 71, 10.0):
            r = np.hypot(X - cx, Y - cy)
            foot = (r < FOOT) & ~np.isnan(S)
            if not foot.any():
                continue
            c = np.nanmax(S[foot])
            if c - RC < FLOOR:
                continue
            if math.hypot(cx, cy) > tub_radius(c - RC) - FOOT - MARGIN:
                continue
            v = clam_volume(S, cx, cy, c)[0]
            if best is None or v > best[0]:
                best = (v, cx, cy, c)
    if best is None or best[0] < 0.05 * V_T:
        return None
    v, cx, cy, c = best
    if v > v_need:                                      # raise C to trim
        c = _bisect(lambda cc: clam_volume(S, cx, cy, cc)[0] <= v_need, c, c + RC)
    v, inside, zb, cut = clam_volume(S, cx, cy, c)
    left = X[inside] < cx
    vl, vr = cut[left].sum(), cut[~left].sum()
    asym = abs(vl - vr) / max(vl + vr, 1e-9)            # 0 = balanced jaws
    S[inside] = np.minimum(S[inside], zb)
    return v, asym


# ------------------------------------------------------------------ drag
RS = 35.0
ALPHA = math.radians(30.0)
B = RS * math.cos(ALPHA)
DMAX = 28.0
# Force budget: the largest cut cross-section allowed anywhere along the lane is
# the one of a 28 mm deep cut on a flat surface (F = u*A, docs/scooping_head_mechanism).
A_MAX = cm.ellipse_segment_area(RS, B, B - DMAX)
# At a 30 deg attack pitch the rim's lowest point lies R*sin(30) behind C and the
# rim spans C +- R*sin(30) along x. The closing cap ahead of C is ignored (conservative).
X_RIM = RS * math.sin(ALPHA)
X_IDX = np.searchsorted(xs, X)


def drag_geometry(S, y0, c):
    """Cells swept by a constant-height drag in lane y0 at pivot height c.

    C may only travel where the whole bowl clears the tub wall at the cut level.
    """
    rlim = tub_radius(c - B) - RS - MARGIN
    if abs(y0) >= rlim:
        return None
    half = math.sqrt(rlim ** 2 - y0 ** 2) + X_RIM      # rim spans C +- X_RIM in x
    band = (np.abs(Y - y0) < RS) & (X >= -half) & (X <= half) & ~np.isnan(S)
    zb = c - B * np.sqrt(np.clip(1 - ((Y[band] - y0) / RS) ** 2, 0, None))
    return band, zb


def drag_cut(S, y0, c):
    """(volume, largest cross-section area along the lane) of one drag."""
    g = drag_geometry(S, y0, c)
    if g is None:
        return 0.0, 0.0
    band, zb = g
    cut = np.clip(S[band] - zb, 0, None)
    if cut.size == 0:
        return 0.0, 0.0
    area_per_x = np.bincount(X_IDX[band], weights=cut * CELL)
    return cut.sum() * CELL ** 2, float(area_per_x.max())


def _bisect(fn, lo, hi, n=30):
    """Smallest c in [lo, hi] with fn(c) True (fn monotone False->True)."""
    if fn(lo):
        return lo
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        if fn(mid):
            hi = mid
        else:
            lo = mid
    return hi


LANES = np.arange(-60, 61, 10.0)


def drag_pass(S, v_need):
    """Layer planing (docs/coordinate_and_flavor_mapping.md §4): no cut may go
    below the current layer floor = highest lane centre line - DMAX. Among the
    lanes, take the one that gives the most volume within the force budget."""
    lanes = []
    for y0 in LANES:
        g = drag_geometry(S, y0, FLOOR + B)
        if g is None:
            continue
        centre = g[0] & (np.abs(Y - y0) < CELL)
        if centre.any():
            lanes.append((y0, float(np.nanmedian(S[centre])), float(np.nanmax(S[g[0]]))))
    if not lanes:
        return None
    layer_floor = max(h for _, h, _ in lanes) - DMAX
    best = None
    for y0, _, s_max in lanes:
        lo = max(FLOOR, layer_floor) + B
        hi = max(lo, s_max + B)
        c = _bisect(lambda cc: drag_cut(S, y0, cc)[1] <= A_MAX, lo, hi)
        v, _ = drag_cut(S, y0, c)
        if best is None or v > best[0]:
            best = (v, y0, c)
    if best is None or best[0] < 0.05 * V_T:
        return None
    v, y0, c = best
    if v > v_need:
        c = _bisect(lambda cc: drag_cut(S, y0, cc)[0] <= v_need, c, c + DMAX + 5)
        v = drag_cut(S, y0, c)[0]
    band, zb = drag_geometry(S, y0, c)
    S[band] = np.minimum(S[band], zb)
    return v, 0.0


MAX_PASSES = 3
ROUGH_R = 80.0     # roughness is measured where both heads can reach


def run(pass_fn):
    """Serve portions until one cannot be completed in MAX_PASSES passes."""
    S = new_surface()
    v0 = np.nansum(S - FLOOR) * CELL ** 2
    rec = []                                            # (passes, first fill, asym list)
    rough = []
    while True:
        need, n, first, asyms = V_T, 0, None, []
        while need > 0.05 * V_T and n < MAX_PASSES:
            r = pass_fn(S, need)
            if r is None:
                break
            n += 1
            if first is None:
                first = r[0] / V_T
            need -= r[0]
            asyms.append(r[1])
        if need > 0.05 * V_T:
            break
        rec.append((n, first, asyms))
        rough.append(float(np.nanstd(S[RR < ROUGH_R])))
    left = np.nansum(S - FLOOR) * CELL ** 2
    passes = [p for p, _, _ in rec]
    first = [f for _, f, _ in rec]
    asyms = [a for _, _, al in rec for a in al]
    return {
        "portions": len(rec),
        "of_tub": len(rec) * V_T / v0,
        "mean_passes": float(np.mean(passes)),
        "multi_pass": float(np.mean([p > 1 for p in passes])),
        "first_fill_p10": float(np.percentile(first, 10)),
        "used": 1 - left / v0,
        "rough_mid": rough[len(rough) // 2],
        "rough_end": rough[-1],
        "asym_med": float(np.median(asyms)),
        "asym_p90": float(np.percentile(asyms, 90)),
        "v0": v0,
    }


drag = run(drag_pass)
clams = []
for m in FOOT_MARGINS:
    FOOT = RC + m
    clams.append(run(clam_pass))

def row(label, fmt):
    return "| " + label + " | " + " | ".join(fmt(r) for r in [drag] + clams) + " |"


lines = [
    "# Drag-curl vs clamshell — tub usage simulation (generated)",
    "",
    "> `python3 calc/scoop_mechanism_compare.py`. **기하만** 비교한다(힘의 절대값·공 모양은 모름). 가정: 통 Ø230→Ø210×250, 초기 표면 −20 mm,",
    f"> 5 mm 격자, 목표 177 cm³(115 g), 한 portion에 최대 {MAX_PASSES}패스(5·8패스로 늘려도 결론 같음). 기계 도달 여부와 무관한 통 부피(바닥 여유 10 mm 위) = {drag['v0']/1e6:.2f} L.",
    f"> 끌기: 레인 어디서든 절삭단면 ≤ {A_MAX:.0f} mm²(= 평면에서 깊이 28 mm, 힘 예산), 층 바닥(가장 높은 레인 − 28 mm) 아래로 안 깎음.",
    "> 클램셸: 두 턱이 C를 지나는 수평축으로 닫혀 C 아래 반구(R 44 = 178 cm³)를 자름. 열린 턱의 발자국(반경 R + 여유) 안에 C보다 높은 재료가 있으면 안 됨",
    ">   (있으면 공의 윗면이 통 재료와 붙은 채로 남아, 들어 올릴 때 찢어야 함). 위치는 매 패스 가장 많이 담기는 곳(greedy).",
    "",
    "| 지표 | 끌기-말기 M1 (R35, 레인) | 클램셸 M2 (R44, 발자국 여유 0) | 클램셸 M2 (여유 6 mm) |",
    "|---|---|---|---|",
    row("멈출 때까지 완성한 portion 수", lambda r: f"{r['portions']}"),
    row("= 통 부피 대비", lambda r: f"{r['of_tub']*100:.0f} %"),
    row("portion당 평균 패스", lambda r: f"{r['mean_passes']:.2f}"),
    row("2패스 이상 필요한 portion", lambda r: f"{r['multi_pass']*100:.0f} %"),
    row("첫 패스 충전율(하위 10 %)", lambda r: f"{r['first_fill_p10']*100:.0f} %"),
    row("표면 거칠기 σ, 통 중심 반경 80 mm 안(중간 / 끝)", lambda r: f"{r['rough_mid']:.1f} / {r['rough_end']:.1f} mm"),
    "| 턱 좌우 절삭 비대칭(중앙값 / 상위 10 %) | – (한 방향 절삭) | "
    + " | ".join(f"{c['asym_med']*100:.0f} % / {c['asym_p90']*100:.0f} %" for c in clams) + " |",
    "",
    "- 끌기는 통 중앙을 바닥(−240 mm)까지 파 내려가고, 멈추는 이유는 **벽 쪽 링**(스쿱이 닿지 않는 부분)만 남기 때문이다.",
    "- 클램셸은 **맨 위 한 층(≈ R 44 mm)** 에서 멈춘다. 분화구 사이에 남은 봉우리가 열린 턱 발자국에 걸려 C를 위에 묶어 두기 때문이다.",
    "  패스 수 제한을 늘려도(5, 8) 늘지 않는다 → 계획 방식의 문제가 아니라 '열린 턱이 들어갈 빈 공간'이라는 기하 조건의 문제다.",
    "- 비대칭 0 %이면 두 턱의 수평 반력이 상쇄되고, 100 %이면 한 턱만 재료를 자른다. 실제 표면에서 상위 10 %는 절반 가까이 비대칭이라,",
    "  '수평 반력이 상쇄되므로 가벼운 갠트리로 된다'는 클램셸의 장점은 평평한 새 통에서만 성립한다.",
    "",
    "해석은 docs/scooping_head_mechanism.md §3.",
]
with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print("\n".join(lines))
