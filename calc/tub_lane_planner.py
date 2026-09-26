"""Tub workspace, lane pattern and per-lane depth needed for one portion.

Run:  python3 calc/tub_lane_planner.py
Out:  calc/output/tub_lane_planner.md

The drag direction is fixed to +X_M (no yaw axis), so the only pattern that
keeps the surface flat without extra axes is a raster of parallel lanes cut
layer by layer (like a facing pass on a CNC mill). This script finds how long
each lane can be inside a tapered tub with wall/floor keep-out, how deep the
scoop must cut to fill one 115 g portion in that length, and what that does
to the drag force. Geometry/material values are ASSUMPTIONS (cartesian_model).
"""

import math
import os

import cartesian_model as cm

OUT = os.path.join(os.path.dirname(__file__), "output", "tub_lane_planner.md")

ALPHA = cm.ATTACK_DEG
BETA_DIVE = 30.0                  # deg, dive path angle below horizontal (ASSUMPTION)
V_TARGET = 115.0 / 0.65 * 1e3     # mm^3 (115 g at rho 0.65, A07)
CLOSE_CAPTURE = 0.5               # fraction of the spherical cap captured by closing (ASSUMPTION)
U_CASES = (90.0, 160.0)           # kPa (A02 medium / hard)

lines = []
p = lines.append


def table(h, rows):
    p("| " + " | ".join(h) + " |")
    p("|" + "|".join("---" for _ in h) + "|")
    for r in rows:
        p("| " + " | ".join(r) + " |")
    p("")


def cut_width(depth):
    b = cm.R_SCOOP * math.cos(math.radians(ALPHA))
    h = b - depth
    if h <= -b:
        return 2 * cm.R_SCOOP
    return 2 * cm.R_SCOOP * math.sqrt(max(0.0, 1 - (h / b) ** 2))


def lane_length(y, surface_depth, depth):
    """Allowed travel of pivot C along X in a lane at offset y (sphere envelope)."""
    r_t = cm.tub_diameter_at(surface_depth + depth) / 2.0
    r_c = r_t - cm.R_SCOOP - cm.WALL_MARGIN
    if abs(y) >= r_c:
        return 0.0
    return 2 * math.sqrt(r_c ** 2 - y ** 2)


def stroke_volume(depth, travel):
    """Dive ramp + drag at constant depth + closing cap (ASSUMPTION fraction)."""
    a = cm.swept_area(-ALPHA, depth)
    l_dive = depth / math.tan(math.radians(BETA_DIVE))
    l_drag = max(0.0, travel - l_dive)
    b = cm.R_SCOOP * math.cos(math.radians(ALPHA))
    h = b - depth
    d_cap = cm.R_SCOOP - h
    v_cap = math.pi * d_cap ** 2 * (3 * cm.R_SCOOP - d_cap) / 3.0
    return 0.5 * a * min(l_dive, travel) + a * l_drag + CLOSE_CAPTURE * v_cap, a


def depth_for_portion(travel):
    b_max = cm.R_SCOOP * math.cos(math.radians(ALPHA)) - 0.5
    for d10 in range(50, int(b_max * 10)):
        d = d10 / 10.0
        v, a = stroke_volume(d, travel)
        if v >= V_TARGET:
            return d, a, v
    v, a = stroke_volume(b_max, travel)
    return None, a, v


p("# Tub workspace & lane planner — generated")
p("")
p(f"> `python3 calc/tub_lane_planner.py`. 통 Ø{cm.TUB_D_TOP:.0f}→Ø{cm.TUB_D_BOTTOM:.0f} × {cm.TUB_DEPTH:.0f} mm, 스쿱 R {cm.R_SCOOP:.0f}, 공격각 {ALPHA:.0f}°, dive 경로각 {BETA_DIVE:.0f}°, 벽 여유 {cm.WALL_MARGIN:.0f} mm, 목표 {V_TARGET/1e3:.0f} cm³(115 g). **모두 ASSUMPTION.**")
p("")
p("## 1. 레인별 허용 드래그 길이와 1 portion에 필요한 깊이")
p("")
rows = []
for s, label in ((20.0, "새 통"), (125.0, "절반"), (220.0, "거의 빈 통")):
    for y in (0.0, 40.0):
        L = lane_length(y, s, 25.0)
        d, a, v = depth_for_portion(L)
        if d is None:
            rows.append([label, f"{s:.0f}", f"±{y:.0f}", f"{L:.0f}", "불가(최대 깊이에서도 미달)", f"{a:.0f}", f"{v/V_TARGET*100:.0f} %", "–", "–"])
        else:
            f90, f160 = U_CASES[0] * 1e-3 * a, U_CASES[1] * 1e-3 * a
            rows.append([label, f"{s:.0f}", f"±{y:.0f}", f"{L:.0f}", f"{d:.1f}", f"{a:.0f}", "100 %", f"{f90:.0f}", f"{f160:.0f}"])
table(["수위", "표면 깊이 [mm]", "레인 y", "허용 이동 [mm]", "필요 깊이 d [mm]", "절삭단면 A [mm²]", "달성 부피", "F @ 90 kPa [N]", "F @ 160 kPa [N]"], rows)
p("해석:")
p("- 레거시 가정(stroke 182–195 mm, d = 20–22 mm)은 **통 벽 여유를 반영하지 않은 값**이었다. Ø230 통에서 R35 스쿱의 중심 레인은 ~140 mm, 옆 레인(±40)은 ~115 mm만 쓸 수 있다.")
p("- 그래서 1 portion에 필요한 깊이가 커지고, 드래그 힘 F = u·A가 레거시 가정보다 커진다. 통이 줄수록(테이퍼) 레인이 더 짧아진다.")
p("- 선택지: (a) 깊게 1회(힘↑), (b) 2회 stroke(시간↑), (c) 스쿱 R 축소는 오히려 portion↓, (d) 공격각 α를 줄여 최대 깊이(R·cos α)를 늘림. → **E3/E4에서 α와 d를 요인으로 넣는다.**")
p("")

p("## 2. 레인 패턴 — 한 지점만 파이는 문제")
p("")
w = cut_width(25.0)
p(f"깊이 25 mm에서 절삭 폭 ≈ {w:.0f} mm. 드래그 방향이 +X로 고정(요 축 없음)이므로 비교 결과는 다음과 같다.")
p("")
table(["패턴", "필요 축", "표면 평탄 유지", "구현", "판정"], [
    ["한 좌표 반복", "–", "✘ 한 줄만 깊게 파임(구덩이)", "최저", "**금지**"],
    ["래스터 레인(층별 평삭)", "X, Y(레인 이동)", "● 층 단위로 고르게 내려감", "레인 인덱스 + 층 카운터", "**채택**"],
    ["그리드 시작점", "X, Y", "○ 래스터와 사실상 같음(시작 x만 다름)", "같음", "래스터에 포함"],
    ["나선/방사", "X, Y + **요(yaw) 회전**", "●", "스쿱 방향을 바꿔야 함 → 축 추가", "기각(축 증가)"],
])
lanes = [-40.0, 0.0, 40.0]
p(f"기본 패턴(ASSUMPTION): 한 층 = 레인 y = {lanes} 순환(겹침 {w-40:.0f} mm), 레인마다 시작 x를 허용 구간의 앞쪽 끝에 둔다. 층이 끝나면 다음 층(표면이 약 d만큼 내려감). 각 스쿱 시작 전에 **1점 터치오프**로 실제 표면을 확인하고, 기계는 자기가 깎은 이력으로 셀 높이 지도(10 mm 격자)를 갱신한다(비전 없음).")
p("")

p("## 3. 통 1개에서 기계가 뜰 수 있는 양")
p("")
r_top, r_bot = cm.TUB_D_TOP / 2, cm.TUB_D_BOTTOM / 2
v_tub = math.pi * cm.TUB_DEPTH / 3 * (r_top ** 2 + r_top * r_bot + r_bot ** 2)
reach = (r_top - cm.R_SCOOP - cm.WALL_MARGIN + w / 2) / r_top
p(f"- 통 부피 ≈ {v_tub/1e6:.1f} L (가정 치수), 115 g portion ≈ {v_tub/V_TARGET:.0f}개 분량")
p(f"- 기계 도달 반경 ≈ 벽에서 {r_top*(1-reach):.0f} mm 안쪽까지 → 면적 기준 ≈ {reach**2*100:.0f} %, 바닥 여유 {cm.FLOOR_MARGIN:.0f} mm 제외")
p(f"- → 면적 기준 상한으로는 통 하나에서 약 {reach**2*(1-cm.FLOOR_MARGIN/cm.TUB_DEPTH)*v_tub/V_TARGET:.0f}개. 레인 끝(C 이동 범위 ± rim 폭)과 테이퍼까지 넣은 5 mm 격자 시뮬레이션(`calc/output/scoop_mechanism_compare.md`)은 **약 30개(통 부피의 59 %)** 다. 범위 30–43개로 본다.")
p("- **벽 쪽 링은 사람이 정리**해야 한다(통 교체 시 또는 주기적으로). 이 비율은 실측 통 치수와 E3 실측 절삭 형상으로 다시 계산한다.")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {OUT}")
