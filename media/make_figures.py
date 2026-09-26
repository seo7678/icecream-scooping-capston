"""Static figures: overall structure (axonometric + 3-view), head mechanism,
scoop sequence and reaction load path.

Run:  python3 media/make_figures.py
Out:  media/fig1_overall_axonometric.png, fig2_three_view.png, fig3_head_mechanism.png,
      fig4_scoop_sequence.png, fig5_load_path.png

Geometry and loads are the V1 design values (calc/cartesian_model.py,
docs/*.md). All loads are ASSUMPTIONS; nothing is measured yet.
"""

import math
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle, FancyArrowPatch, FancyBboxPatch, Ellipse

import machine as m
from machine import cm

HERE = os.path.dirname(os.path.abspath(__file__))
DPI = 150


def save(fig, name):
    path = os.path.join(HERE, name)
    fig.savefig(path, dpi=DPI, facecolor="white")
    plt.close(fig)
    print("wrote", path)


# =====================================================================================
# Fig 1 — axonometric overview (own painter's algorithm, no mplot3d depth artefacts)
# =====================================================================================
AZ, EL = math.radians(24.0), math.radians(21.0)
GRAD = np.array([-math.sin(AZ) * math.cos(EL), math.cos(AZ) * math.cos(EL), -math.sin(EL)])


def proj(p):
    x, y, z = p
    x1 = x * math.cos(AZ) + y * math.sin(AZ)
    y1 = -x * math.sin(AZ) + y * math.cos(AZ)
    return x1, z * math.cos(EL) + y1 * math.sin(EL), y1 * math.cos(EL) - z * math.sin(EL)


def shade(color, f):
    c = np.array(matplotlib_colors.to_rgb(color))
    return tuple(np.clip(c * f, 0, 1))


import matplotlib.colors as matplotlib_colors  # noqa: E402

FACES = []   # (depth, verts2d, fc, ec, lw, alpha, zbias)


def box(x0, x1, y0, y1, z0, z1, color, ec="#3b4650", lw=0.5, alpha=1.0, zbias=0.0):
    v = {(i, j, k): (x, y, z) for i, x in enumerate((x0, x1)) for j, y in enumerate((y0, y1))
         for k, z in enumerate((z0, z1))}
    faces = [
        ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), np.array([0, -1, 0]), 0.92),   # front  (-y)
        ((0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1), np.array([0, 1, 0]), 0.80),    # back
        ((0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), np.array([-1, 0, 0]), 0.78),   # left
        ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1), np.array([1, 0, 0]), 0.78),    # right (+x)
        ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1), np.array([0, 0, 1]), 1.00),    # top
        ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), np.array([0, 0, -1]), 0.6),    # bottom
    ]
    for a, b, c, d, n, f in faces:
        if n @ GRAD >= 0:
            continue
        pts = [proj(v[k]) for k in (a, b, c, d)]
        depth = np.mean([p[2] for p in pts])
        FACES.append((depth - zbias, [(p[0], p[1]) for p in pts], shade(color, f), ec, lw, alpha))


def disk(cx, cy, z, r, color, ec="#3b4650", lw=0.5, zbias=0.0, n=64, alpha=1.0):
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    pts = [proj((cx + r * math.cos(a), cy + r * math.sin(a), z)) for a in t]
    depth = proj((cx, cy, z))[2]
    FACES.append((depth - zbias, [(p[0], p[1]) for p in pts], color, ec, lw, alpha))


def flush():
    out = sorted(FACES, key=lambda f: -f[0])
    FACES.clear()
    return out


def fig_axonometric():
    fig = plt.figure(figsize=(14, 8.4))
    ax = fig.add_axes([0.0, 0.0, 0.66, 0.9])
    ax.set_aspect("equal")
    ax.axis("off")
    X, Y, Z, TH = 600.0, 0.0, 30.0, -30.0    # pivot C over TUB2, descending in attack pose

    # floor
    FACES.append((1e9, [proj(p)[:2] for p in [(-420, -420, -850), (1600, -420, -850), (1600, 420, -850),
                                                (-420, 420, -850)]], "#EEF1F4", "none", 0, 1.0))
    # posts + braces + beam + rails + screw + motor
    for x0, x1 in m.POSTS_X:
        box(x0, x1, 185, 265, -850, 880, m.C_ALU)
    box(-200, 1350, 200, 250, 760, 860, m.C_STEEL)
    box(-170, 1320, 186, 200, 772, 786, "#44505b", lw=0.3)
    box(-170, 1320, 186, 200, 834, 848, "#44505b", lw=0.3)
    box(1290, 1370, 178, 250, 772, 848, "#2E3A46")
    # freezer + deck + stands
    box(*m.FREEZER_X, *m.FREEZER_Y, -850, -20, "#F2F5F8", ec="#9aa6b2")
    box(m.FREEZER_X[0] - 20, m.FREEZER_X[1] + 20, -270, 270, -20, 0, m.C_DECK, ec="#8E9AA5")
    box(m.RINSE_X - 80, m.RINSE_X + 80, -80, 80, -160, 0, "#DCE3E8", ec="#8E9AA5")
    box(m.RINSE_X - 25, m.RINSE_X + 25, -25, 25, -850, -160, "#D0D6DC", ec="#9aa6b2")
    box(m.CUP_X - 95, m.CUP_X + 95, -95, 95, -140, 0, "#DCE3E8", ec="#8E9AA5")
    box(m.CUP_X - 25, m.CUP_X + 25, -25, 25, -850, -140, "#D0D6DC", ec="#9aa6b2")
    base = flush()
    for d, verts, fc, ec, lw, al in base:
        ax.add_patch(Polygon(verts, closed=True, fc=fc, ec=ec, lw=lw, alpha=al, zorder=1))

    # tub openings with ice cream (clipped to the opening)
    for fid, name, tx, s, col in m.FLAVORS:
        t = np.linspace(0, 2 * math.pi, 90)
        rim = [proj((tx + 115 * math.cos(a), 115 * math.sin(a), 0))[:2] for a in t]
        op = Polygon(rim, closed=True, fc="#E9E4D8", ec="#6f6656", lw=1.0, zorder=2)
        ax.add_patch(op)
        rs = m.tub_half_width(s)
        sfc = [proj((tx + rs * math.cos(a), rs * math.sin(a), s))[:2] for a in t]
        p = Polygon(sfc, closed=True, fc=col, ec="none", zorder=2.1)
        ax.add_patch(p)
        p.set_clip_path(op)
    # rinse water, cup
    t = np.linspace(0, 2 * math.pi, 60)
    ax.add_patch(Polygon([proj((m.RINSE_X + 70 * math.cos(a) * 1.0, 70 * math.sin(a), -20))[:2] for a in t],
                         closed=True, fc=m.C_WATER, ec="#7aa9bf", lw=0.6, zorder=2))
    ax.add_patch(Polygon([proj((m.CUP_X + 40 * math.cos(a), 40 * math.sin(a), 0))[:2] for a in t],
                         closed=True, fc="white", ec="#8E9AA5", lw=0.9, zorder=2))
    ax.add_patch(Polygon([proj((m.CUP_X + 31 * math.cos(a), 31 * math.sin(a), -60))[:2] for a in t],
                         closed=True, fc="#e8ecef", ec="none", zorder=2.05))

    # moving parts
    box(X - 100, X + 100, 160, 200, 735, 885, m.C_MOVE, ec="#1E4E80")
    box(X - 80, X + 80, -70, 170, 700, 735, "#5C8FC8", ec="#1E4E80")
    box(X - 70, X + 70, 35, 55, m.ZPLATE[0], m.ZPLATE[1], m.C_MOVE_L, ec=m.C_MOVE, zbias=-600)
    box(X + 30, X + 72, 35, 77, m.ZPLATE[1] - 45, m.ZPLATE[1], "#2E3A46")
    cb = Z + m.STEM + m.HEAD
    box(X - 30, X + 30, -30, 30, cb, cb + m.COL_LEN, m.C_STEEL, ec=m.C_STEEL_D)
    box(X - 60, X + 60, -45, 45, Z + m.STEM, cb, m.C_HEAD, ec="#1b242e")
    box(X - 10, X + 50, -95, -45, Z + m.STEM + 25, Z + m.STEM + 85, "#6C7E92", ec="#26313d")
    box(X - 85, X + 85, -70, 70, Z + m.STEM - 5, Z + m.STEM, m.C_FOOD, ec="#a86d00", lw=0.4)
    box(X - 7.5, X + 7.5, -7.5, 7.5, Z, Z + m.STEM - 5, "#D7DCE1", ec=m.C_FOOD)
    lx, lz = m.lever_tip(X, Z, TH)
    box(lx - 4, lx + 4, -4, 4, lz, lz + m.STEM - 5, "#E7C77f", ec=m.C_FOOD, lw=0.4)
    moving = flush()
    for d, verts, fc, ec, lw, al in moving:
        ax.add_patch(Polygon(verts, closed=True, fc=fc, ec=ec, lw=lw, alpha=al, zorder=3))
    # scoop: sphere silhouette + rim circle
    c2 = proj((X, 0, Z))
    ax.add_patch(Circle(c2[:2], m.R, fc="#E3B865", ec="#a86d00", lw=0.9, zorder=4))
    th = math.radians(TH)
    u2 = np.array([-math.sin(th), 0, math.cos(th)])
    rim = [proj(np.array([X, 0, Z]) + m.R * (math.cos(a) * np.array([0, 1, 0]) + math.sin(a) * u2))[:2]
           for a in np.linspace(0, 2 * math.pi, 60)]
    ax.add_patch(Polygon(rim, closed=True, fc="#F4E4BE", ec="#a86d00", lw=0.9, zorder=4.1))

    # enclosure (transparent) + cup bay + HMI
    E = [(-320, -300, 0), (1500, -300, 0), (1500, 300, 0), (-320, 300, 0),
         (-320, -300, 1220), (1500, -300, 1220), (1500, 300, 1220), (-320, 300, 1220)]
    P = [proj(p)[:2] for p in E]
    for a, b in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]:
        ax.plot([P[a][0], P[b][0]], [P[a][1], P[b][1]], color="#7FA3C6", lw=0.9, ls=(0, (5, 3)), zorder=5)
    ax.add_patch(Polygon([P[0], P[1], P[5], P[4]], closed=True, fc="#9CC0E6", ec="none", alpha=0.07, zorder=5))
    bay = [proj(p)[:2] for p in [(1040, -300, 0), (1200, -300, 0), (1200, -300, 220), (1040, -300, 220)]]
    ax.add_patch(Polygon(bay, closed=True, fc="white", ec="#c24", lw=1.0, ls="--", alpha=0.9, zorder=5.1))
    for zz in (60, 150):
        for xx in (1040, 1200):
            q = proj((xx, -300, zz))
            ax.add_patch(Circle(q[:2], 9, fc=m.C_SENSOR, ec="none", zorder=5.2))
    door = [proj(p)[:2] for p in [(0, -300, 20), (1000, -300, 20), (1000, -300, 1180), (0, -300, 1180)]]
    ax.add_patch(Polygon(door, closed=True, fc="none", ec="#7FA3C6", lw=1.2, zorder=5.1))
    hmi = [proj(p)[:2] for p in [(-290, -302, 160), (-150, -302, 160), (-150, -302, 320), (-290, -302, 320)]]
    ax.add_patch(Polygon(hmi, closed=True, fc="#26313d", ec="#111", lw=0.6, zorder=5.3))
    es = proj((-220, -302, 400))
    ax.add_patch(Circle(es[:2], 26, fc="#FFD400", ec="#333", lw=0.6, zorder=5.3))
    ax.add_patch(Circle(es[:2], 16, fc="#D0021B", ec="none", zorder=5.4))

    # callouts: anchors projected, labels stacked in two columns in anchor order (no crossings)
    calls = [
        (1, (250, 200, 860)), (2, (X + 100, 180, 885)), (3, (X - 80, -70, 718)), (4, (X + 50, 56, 1175)),
        (5, (X - 30, -30, 820)), (6, (X - 60, -45, Z + 360)), (7, (X - 7, -8, Z + 150)), (8, (340, -60, -10)),
        (9, (600, -250, -400)), (10, (m.CUP_X + 95, -60, -70)), (11, (m.RINSE_X - 80, -40, -80)),
        (12, (1200, -300, 150)), (13, (-220, -302, 240)), (14, (1500, -300, 420)),
    ]
    anchors = {n: proj(p3)[:2] for n, p3 in calls}
    xs_all = [pt[0] for f in base + moving for pt in f[1]]
    ys_all = [pt[1] for f in base + moving for pt in f[1]]
    x_lo, x_hi = min(xs_all), max(xs_all)
    y_lo, y_hi = min(ys_all), max(ys_all)
    mid = 0.5 * (x_lo + x_hi)
    left = sorted([n for n in anchors if anchors[n][0] < mid], key=lambda n: -anchors[n][1])
    right = sorted([n for n in anchors if anchors[n][0] >= mid], key=lambda n: -anchors[n][1])
    gap = 125.0
    for side, col_x in ((left, x_lo - 170), (right, x_hi + 170)):
        ys = [anchors[n][1] for n in side]            # start level with the anchor, then spread apart
        for i in range(1, len(ys)):
            ys[i] = min(ys[i], ys[i - 1] - gap)
        for i in range(len(ys) - 2, -1, -1):
            ys[i] = max(ys[i], ys[i + 1] + gap)
        shift = max(0.0, max(ys) - (y_hi - 40))
        ys = [y - shift for y in ys]
        for n, ty in zip(side, ys):
            ax_, ay_ = anchors[n]
            ax.plot([ax_, col_x], [ay_, ty], color="#44505b", lw=0.7, zorder=6)
            ax.add_patch(Circle((ax_, ay_), 9, fc="#1F2A36", ec="none", zorder=6))
            ax.add_patch(Circle((col_x, ty), 42, fc="#1F2A36", ec="white", lw=1.0, zorder=7))
            ax.text(col_x, ty, str(n), color="white", ha="center", va="center", fontsize=9, fontweight="bold",
                    zorder=8)

    ax.set_xlim(x_lo - 240, x_hi + 240)
    ax.set_ylim(y_lo - 40, y_hi + 60)

    # legend panel
    lg = fig.add_axes([0.655, 0.05, 0.335, 0.86])
    lg.axis("off")
    lg.set_xlim(0, 1)
    lg.set_ylim(0, 1)
    items = [
        ("X 빔 · 볼스크류 SFU2010 · NEMA23", "강관 100×50×4, 행정 1,150 mm, 통 간 이동 + 드래그"),
        ("X 캐리지 (HGH15 ×4)", "X 볼너트 로드셀 50 kg = 드래그 힘 F_x"),
        ("Y 크로스슬라이드 (SFU1605)", "±80 mm 레인 이동 (제품: 이동 브리지)"),
        ("Z 구동 (SFU1610 · NEMA17)", "스프링 브레이크 + counterbalance 1.05, 행정 400 mm"),
        ("Z 기둥 (강관 60×60×4)", "Z 볼너트 로드셀 20 kg = 표면 검출 3 N"),
        ("스쿠핑 헤드", "θ 유성 기어드모터 ≥ 8 N·m, 평행링크, 드립 우산"),
        ("식품 모듈", "스쿱 R35 · 스템 Ø30×3 · push-rod Ø8, 퀵핀 2개로 분리"),
        ("통 ×3", "Ø230→Ø210 × 250, 데크 웰이 옆힘 구속"),
        ("냉동고 + 304 데크", "통을 디핑 온도로 유지"),
        ("컵 스테이션", "테두리 = 데크 높이, 1 kg 로드셀, 배출 빗"),
        ("헹굼 스테이션", "맛이 바뀌면 자동 헹굼"),
        ("컵 베이 2빔", "손 감지 → 전 축 STO (하드와이어)"),
        ("HMI + E-stop", "맛 버튼, 정지범주 0"),
        ("인클로저", "가동부 전체 분리, 도어 인터록 → STO"),
    ]
    lg.text(0.0, 0.985, "구성 요소", fontsize=13, fontweight="bold", va="top", color=m.C_TEXT)
    for i, (a, b) in enumerate(items):
        y = 0.93 - i * 0.063
        lg.add_patch(Circle((0.025, y - 0.012), 0.018, fc="#1F2A36", ec="none", transform=lg.transAxes))
        lg.text(0.025, y - 0.012, str(i + 1), color="white", ha="center", va="center", fontsize=7.5,
                fontweight="bold")
        lg.text(0.065, y, a, fontsize=9.3, va="top", color=m.C_TEXT, fontweight="bold")
        lg.text(0.065, y - 0.027, b, fontsize=8.2, va="top", color="#51606e")
    fig.text(0.012, 0.965, "Cartesian 자동 맛 선택 스쿠퍼 — 전체 구조 (V1)", fontsize=16, fontweight="bold",
             color=m.C_TEXT)
    fig.text(0.012, 0.935, "직교 3축(X·Y·Z) + 로컬 피치 θ 1축 = 모터 4개. 자세: 피벗 C가 TUB2 위, 공격 자세 θ = −30°. "
             "치수·하중은 설계값·가정값", fontsize=9.5, color="#51606e")
    save(fig, "fig1_overall_axonometric.png")


# =====================================================================================
# Fig 2 — three-view drawing with main dimensions
# =====================================================================================
def dim(ax, p0, p1, off, text, horizontal=True, fs=7.5, color="#34495e"):
    (x0, y0), (x1, y1) = p0, p1
    if horizontal:
        y = off
        ax.plot([x0, x0], [y0, y], color=color, lw=0.5)
        ax.plot([x1, x1], [y1, y], color=color, lw=0.5)
        ax.annotate("", xy=(x0, y), xytext=(x1, y), arrowprops=dict(arrowstyle="<->", color=color, lw=0.7))
        ax.text((x0 + x1) / 2, y + 12, text, ha="center", va="bottom", fontsize=fs, color=color,
                bbox=dict(fc="white", ec="none", pad=0.5))
    else:
        x = off
        ax.plot([x0, x], [y0, y0], color=color, lw=0.5)
        ax.plot([x1, x], [y1, y1], color=color, lw=0.5)
        ax.annotate("", xy=(x, y0), xytext=(x, y1), arrowprops=dict(arrowstyle="<->", color=color, lw=0.7))
        ax.text(x + 12, (y0 + y1) / 2, text, ha="left", va="center", fontsize=fs, color=color, rotation=90,
                bbox=dict(fc="white", ec="none", pad=0.5))


def fig_three_view():
    fig = plt.figure(figsize=(16, 10))
    fig.text(0.012, 0.972, "3면도 — 정면 · 평면 · 측면 (V1, 단위 mm)", fontsize=15, fontweight="bold", color=m.C_TEXT)
    fig.text(0.012, 0.948, "정면: 스쿱이 TUB1에서 드래그 중(C = −57, θ = −30°). 좌표값은 조회표 예시값, 치수는 설계값 — 실측·CAD로 교체",
             fontsize=9, color="#51606e")
    # --- front (X-Z)
    ax = fig.add_axes([0.01, 0.30, 0.56, 0.63])
    ax.set_aspect("equal")
    ax.axis("off")
    m.draw_frame_static(ax, labels=True)
    Xc, Zc = 375.0, -62 + m.B_ATT - 24
    prof_x = np.arange(340 - 114, 340 + 114, 1.0)
    prof = np.full(prof_x.shape, -62.0)
    for xx in np.arange(275, Xc + 0.1, 2.0):
        zc = min(-62 + m.B_ATT, -62 + m.B_ATT - (xx - 275) * math.tan(math.radians(30)))
        zc = max(zc, Zc)
        env = m.lower_envelope(m.bowl_polygon(xx, zc, -30), prof_x)
        ok = ~np.isnan(env)
        prof[ok] = np.fmin(prof[ok], env[ok])
    m.draw_frame_static(ax, tub_profiles={"VAN": (prof_x, prof)}, labels=False)
    m.draw_carriage(ax, Xc, Zc, -30, fill=0.5)
    ax.axhline(m.Z_SAFE, xmin=0.03, xmax=0.97, color=m.C_SAFE, lw=0.8, ls=(0, (6, 4)))
    ax.text(-310, m.Z_SAFE + 10, "Z_SAFE = 60 (C 높이)", fontsize=7, color="#9a7a00")
    dim(ax, (m.POSTS_X[0][0], m.FLOOR), (m.POSTS_X[1][1], m.FLOOR), m.FLOOR - 120, "포스트 외곽 1,710")
    dim(ax, (0, 0), (1150, 0), -640, "X 행정 1,150 (HOME 0 → CUP 1,120)")
    dim(ax, (340, -250), (600, -250), -450, "통 간격 260")
    dim(ax, (m.POSTS_X[1][1], m.FLOOR), (m.POSTS_X[1][1], 0), 1520, "데크 높이 850", horizontal=False)
    dim(ax, (m.POSTS_X[1][1], 0), (m.POSTS_X[1][1], 860), 1520, "빔 상단 860", horizontal=False)
    dim(ax, (1300, 860), (1300, 1175), 1600, "Z 구동판 1,175", horizontal=False)
    ax.set_xlim(-420, 1720)
    ax.set_ylim(-1030, 1260)
    ax.text(-400, 1230, "정면도 (−Y에서 +Y 방향으로 봄)", fontsize=11, fontweight="bold", color=m.C_TEXT)

    # --- top (X-Y), placed below the front view
    at = fig.add_axes([0.01, 0.015, 0.56, 0.27])
    at.set_aspect("equal")
    at.axis("off")
    at.add_patch(Rectangle((-320, -300), 1820, 600, fc="none", ec="#7FA3C6", lw=0.9, ls=(0, (5, 3))))
    at.add_patch(Rectangle((m.FREEZER_X[0] - 20, -270), m.FREEZER_X[1] - m.FREEZER_X[0] + 40, 540,
                           fc=m.C_DECK, ec="#8E9AA5"))
    for fid, name, tx, s, col in m.FLAVORS:
        at.add_patch(Circle((tx, 0), 115, fc=col, ec="#6f6656", lw=0.8))
    for yl in (-40, 0, 40):
        at.plot([340 - 66, 340 + 66], [yl, yl], color="#6b5a2a", lw=0.7, ls=":")
    at.add_patch(Rectangle((m.RINSE_X - 80, -80), 160, 160, fc=m.C_WATER, ec="#8E9AA5"))
    at.add_patch(Rectangle((m.CUP_X - 95, -95), 190, 190, fc="#DCE3E8", ec="#8E9AA5"))
    at.add_patch(Circle((m.CUP_X, 0), 40, fc="white", ec="#8E9AA5"))
    for x0, x1 in m.POSTS_X:
        at.add_patch(Rectangle((x0, 185), x1 - x0, 80, fc=m.C_ALU, ec="#98A3AD"))
    at.add_patch(Rectangle((-200, 200), 1550, 50, fc=m.C_STEEL, ec=m.C_STEEL_D))
    at.add_patch(Rectangle((Xc - 100, 160), 200, 40, fc=m.C_MOVE, ec="#1E4E80"))
    at.add_patch(Rectangle((Xc - 80, -70), 160, 240, fc="#5C8FC8", ec="#1E4E80", alpha=0.55))
    at.add_patch(Rectangle((Xc - 30, -30), 60, 60, fc=m.C_STEEL, ec=m.C_STEEL_D))
    at.add_patch(Rectangle((1040, -300), 160, 8, fc=m.C_SENSOR, ec="none"))
    at.text(1215, -340, "컵 베이 개구 + 2빔", ha="left", va="center", fontsize=7.5, color=m.C_SENSOR)
    at.text(340, -150, "레인 y = −40 / 0 / +40", ha="center", fontsize=7, color="#6b5a2a")
    dim(at, (m.FREEZER_X[0], -270), (m.FREEZER_X[1], -270), -345, "냉동고 800 × 500")
    at.annotate("", xy=(Xc + 95, -70), xytext=(Xc + 95, 170), arrowprops=dict(arrowstyle="<->", color="#1E4E80", lw=0.8))
    at.text(Xc + 105, 150, "Y ±80", fontsize=7.5, color="#1E4E80", va="center", ha="left")
    at.text(-400, 400, "평면도 (위에서, 뒤 = +Y)", fontsize=11, fontweight="bold", color=m.C_TEXT, va="top")
    at.text(-310, -285, "앞(작업자 쪽, −Y)", fontsize=7.5, color="#51606e")
    at.set_xlim(-420, 1720)
    at.set_ylim(-560, 422)

    # --- side (Y-Z), looking along -X from the right
    asd = fig.add_axes([0.575, 0.30, 0.2, 0.63])
    asd.set_aspect("equal")
    asd.axis("off")
    # here the horizontal axis is -Y (front on the left)
    def Rct(y0, y1, z0, z1, **kw):
        asd.add_patch(Rectangle((y0, z0), y1 - y0, z1 - z0, **kw))
    Rct(-270, 270, -20, 0, fc=m.C_DECK, ec="#8E9AA5")
    Rct(-250, 250, m.FLOOR, -20, fc=m.C_FREEZER, ec="#AAB5BF")
    hw_t, hw_b = 115, 105
    asd.add_patch(Polygon([(-hw_t, 0), (hw_t, 0), (hw_b, -250), (-hw_b, -250)], fc="#F4F1EA", ec="#9A8F7A"))
    asd.add_patch(Polygon([(-hw_b, -250), (-m.tub_half_width(-62), -62), (m.tub_half_width(-62), -62), (hw_b, -250)],
                          fc="#EFDDA6", ec="none"))
    Rct(185, 265, m.FLOOR, 880, fc=m.C_ALU, ec="#98A3AD")
    Rct(200, 250, 760, 860, fc=m.C_STEEL, ec=m.C_STEEL_D)
    Rct(160, 200, 735, 885, fc=m.C_MOVE, ec="#1E4E80")
    Rct(-70, 170, 700, 735, fc="#5C8FC8", ec="#1E4E80")
    Rct(35, 55, m.ZPLATE[0], m.ZPLATE[1], fc=m.C_MOVE_L, ec=m.C_MOVE)
    Rct(35, 77, m.ZPLATE[1] - 45, m.ZPLATE[1], fc="#2E3A46", ec="none")
    cb = Zc + m.STEM + m.HEAD
    Rct(-30, 30, cb, cb + m.COL_LEN, fc=m.C_STEEL, ec=m.C_STEEL_D)
    for zb in m.ZBLOCKS:
        Rct(30, 45, zb - 12, zb + 12, fc="#26313D", ec="none")
    Rct(-45, 45, Zc + m.STEM, cb, fc=m.C_HEAD, ec="none")
    Rct(-95, -45, Zc + m.STEM + 25, Zc + m.STEM + 85, fc="#6C7E92", ec="#26313d")
    Rct(-70, 70, Zc + m.STEM - 5, Zc + m.STEM, fc=m.C_FOOD, ec="none")
    Rct(-7.5, 7.5, Zc, Zc + m.STEM - 5, fc="#D7DCE1", ec=m.C_FOOD)
    asd.add_patch(Circle((0, Zc), m.R, fc="#E3B865", ec="#a86d00", lw=0.9, alpha=0.9))
    asd.plot([-300, -300], [0, 1220], color="#7FA3C6", lw=0.9, ls=(0, (5, 3)))
    asd.plot([300, 300], [0, 1220], color="#7FA3C6", lw=0.9, ls=(0, (5, 3)))
    asd.plot([-300, 300], [1220, 1220], color="#7FA3C6", lw=0.9, ls=(0, (5, 3)))
    asd.add_patch(Rectangle((-320, m.FLOOR - 40), 640, 40, fc="#E3E6E9", ec="none"))
    dim(asd, (0, 780), (225, 780), 1050, "225", fs=7)
    dim(asd, (-250, m.FLOOR), (250, m.FLOOR), m.FLOOR - 120, "깊이 500", fs=7)
    asd.annotate("C", xy=(0, Zc), xytext=(-150, Zc - 120), fontsize=9, color=m.C_TEXT,
                 arrowprops=dict(arrowstyle="-", lw=0.6))
    asd.text(-310, -300 + 30, "← 앞(작업자)", fontsize=8, color="#51606e")
    asd.text(-420, 1230, "측면도 (+X 쪽에서 봄)", fontsize=11, fontweight="bold", color=m.C_TEXT, ha="left")

    asd.set_xlim(-420, 420)
    asd.set_ylim(-1030, 1260)
    note = fig.add_axes([0.785, 0.30, 0.205, 0.63])
    note.axis("off")
    notes = [
        ("투영", "3각법 배치, 세 뷰 모두 같은 축척. 평면도는 정면도와 X가 맞춰져 있다."),
        ("빔은 뒤, 기둥은 앞", "X 빔은 통 열보다 225 mm 뒤(Y 200–250)에 있다. Y 크로스슬라이드가 기둥을 레인 위(Y = 0 ± 80)로 내민다."),
        ("높이", "바닥 → 데크 850, 데크 → 빔 상단 860, Z 구동판 상단 1,175(데크 기준). 컵 테두리 = 데크 높이(매립)."),
        ("Z 위치", "Z_M = 피벗 C 높이. 이동 가능 +160 … −205, Z_SAFE 60 이상에서만 XY 이동."),
        ("통", "Ø230 → Ø210 × 250, 중심 X 340 / 600 / 860, 레인 y = −40 / 0 / +40."),
        ("치수의 성격", "설계값·조회표 예시값. 매장 통·스쿱 실측(P0-4)과 CAD 후 교체."),
    ]
    y = 0.98
    for h, t in notes:
        note.text(0, y, h, fontsize=9.5, fontweight="bold", color=m.C_TEXT, va="top")
        note.text(0, y - 0.035, t, fontsize=8.3, color="#3b4650", va="top", wrap=True)
        y -= 0.16
    save(fig, "fig2_three_view.png")


# =====================================================================================
# Fig 3 — head mechanism
# =====================================================================================
def fig_head():
    fig = plt.figure(figsize=(15, 8.6))
    fig.text(0.012, 0.965, "스쿠핑 헤드 메커니즘 — 평행링크로 θ 전달 (단면, X–Z 평면)", fontsize=15, fontweight="bold",
             color=m.C_TEXT)
    fig.text(0.012, 0.938, "모터는 드립 우산 위(비식품 영역), 음식에 닿는 부분은 스쿱·스템·push-rod·핀뿐. "
             "레버각 φ = θ − 30°, 사용 범위 ±60° → θ = −30° … +90°", fontsize=9.5, color="#51606e")
    poses = [(-30.0, "① 공격 자세 θ = −30°\n개구부 앞-아래 (dive·drag)"), (30.0, "② 레버 중립 θ = +30°\n(φ = 0, 전달 효율 최대)"),
             (90.0, "③ 포획 자세 θ = +90°\n개구부 위 (닫기 끝·이동)")]
    for i, (th, title) in enumerate(poses):
        ax = fig.add_axes([0.005 + i * 0.222, 0.04, 0.215, 0.86])
        ax.set_aspect("equal")
        ax.axis("off")
        X, Z = 0.0, 0.0
        m.draw_head(ax, X, Z, th, ball=(th == 90.0), fill=0.35 if th == -30 else 0.0, show_col=True)
        lx, lz = m.lever_tip(X, Z, th)
        phi = th - 30.0
        ax.text(0, -62, title, ha="center", va="top", fontsize=10.5, color=m.C_TEXT)
        ax.annotate(f"φ = {phi:+.0f}°", xy=(lx * 0.6, lz * 0.6), xytext=(-150, -20 if th != 90 else 30), fontsize=9.5,
                    color="#a86d00", arrowprops=dict(arrowstyle="-", color="#a86d00", lw=0.6))
        ax.annotate(f"φ = {phi:+.0f}°", xy=(lx * 0.6, 300 + lz * 0.6), xytext=(-150, 262), fontsize=9.5,
                    color="#a86d00", arrowprops=dict(arrowstyle="-", color="#a86d00", lw=0.6))
        if i == 0:
            notes = [((22, 355), (70, 452), "θ 유성 기어드모터\n24 V ≥ 8 N·m, 엔코더·전류"),
                     ((-80, 298), (-158, 340), "드립 우산 (304)\n위 비식품 / 아래 식품"),
                     ((lx, 321.7), (70, 275), "상부 레버 l = 25\n(모터 출력축)"),
                     ((lx, 170), (-158, 170), "push-rod Ø8 (304)\n스템 밖에 노출"),
                     ((7.5, 110), (45, 110), "스템 Ø30×3\n양끝 용접 밀봉"),
                     ((lx * 0.5, lz * 0.5), (-158, 80), "하부 레버 l = 25\n(스쿱에 고정)"),
                     ((0, 0), (48, 20), "피벗 C = 볼 중심\nØ8 핀"),
                     ((-26, -24), (48, -40), "스쿱 R35 (304)\n매장과 같은 모델")]
            for xy, xyt, t in notes:
                ax.annotate(t, xy=xy, xytext=xyt, fontsize=8, color=m.C_TEXT, va="center",
                            arrowprops=dict(arrowstyle="-", color="#7a8793", lw=0.6), zorder=30,
                            bbox=dict(fc="white", ec="none", alpha=0.85, pad=1))
        ax.set_xlim(-165, 165)
        ax.set_ylim(-110, 475)

    # rod force chart
    ac = fig.add_axes([0.715, 0.52, 0.27, 0.36])
    phis = np.linspace(-80, 80, 321)
    for tau, col, lab in ((3.2, m.C_MOVE, "드래그 유지 3.2 N·m"), (7.0, m.C_SENSOR, "닫기 7.0 N·m")):
        F = tau / (m.LEVER * 1e-3 * np.cos(np.radians(phis)))
        ac.plot(phis, F, color=col, lw=1.6, label=lab)
    ac.axvspan(-60, 60, color="#DFF0D8", alpha=0.6, lw=0)
    ac.set_ylim(0, 1500)
    ac.set_xlim(-80, 80)
    ac.set_xlabel("레버각 φ [°]", fontsize=9)
    ac.set_ylabel("push-rod 축력 [N]", fontsize=9)
    ac.tick_params(labelsize=8)
    ac.legend(fontsize=8, frameon=False, loc="upper center")
    ac.text(0, 60, "사용 범위 ±60°", ha="center", fontsize=8, color="#3c763d")
    ac.set_title("평행링크 사점: F_rod = τ / (l·cos φ)", fontsize=9.5, loc="left")
    ac.grid(color=m.C_GRID, lw=0.5)

    tx = fig.add_axes([0.715, 0.05, 0.27, 0.40])
    tx.axis("off")
    lines = [
        ("왜 평행링크인가", "θ 모터를 드립 우산 위에 두고, 식품 영역에는 막대와 핀만 남긴다. 기어·벨트·전선이 음식 위로 가지 않는다."),
        ("왜 ±60°만 쓰나", "레버가 로드와 나란해지는 ±90°에서 사점. ±60°에서 축력 2배(560 N @ 7 N·m), 좌굴 4.26 kN으로 안전율 ≥ 7."),
        ("그 결과", "θ 범위 −30° … +90°. 180° 과회전으로 공을 쏟는 배출은 불가 → 배출은 빗 걸기(시험 전)."),
        ("분리", "퀵핀 2개(스템 클램프, push-rod 상단)로 식품 모듈 통째 분리. 일반·알레르겐 2벌."),
    ]
    y = 1.0
    for h, t in lines:
        tx.text(0, y, h, fontsize=10, fontweight="bold", color=m.C_TEXT, va="top")
        tx.text(0, y - 0.075, t, fontsize=8.6, color="#3b4650", va="top", wrap=True)
        y -= 0.25
    save(fig, "fig3_head_mechanism.png")


# =====================================================================================
# Fig 4 — scoop sequence in the tub
# =====================================================================================
def fig_sequence():
    S0 = -62.0
    TX = 340.0
    d = 24.0
    ZC0 = S0 + m.B_ATT
    zc_drag = ZC0 - d
    r_t = cm.tub_diameter_at(-(zc_drag - m.R)) / 2
    half = r_t - m.R - cm.WALL_MARGIN
    xa, xb = TX - half, TX + half
    x_dive = xa + d / math.tan(math.radians(30))
    xs = np.arange(TX - 114, TX + 114, 0.5)
    prof = np.full(xs.shape, S0)

    def sweep(path):
        for (x, z, th) in path:
            env = m.lower_envelope(m.bowl_polygon(x, z, th), xs)
            ok = ~np.isnan(env)
            prof[ok] = np.fmin(prof[ok], env[ok])

    path = [(xa + t * (x_dive - xa), ZC0 - t * d, -30) for t in np.linspace(0, 1, 60)]
    path += [(x, zc_drag, -30) for x in np.linspace(x_dive, xb, 120)]
    path += [(xb, zc_drag, th) for th in np.linspace(-30, 90, 60)]
    sweep(path)

    fig = plt.figure(figsize=(15, 8.2))
    fig.text(0.012, 0.962, "한 스쿱의 동작 — 절입 · 끌기 · 닫기 (TUB1 단면, 레인 y = 0)", fontsize=15, fontweight="bold",
             color=m.C_TEXT)
    fig.text(0.012, 0.935, f"예: 표면 −62 mm, 깊이 d = {d:.0f} mm, 레인 {xb - xa:.0f} mm(벽 여유 10 mm). "
             "힘은 F = u·A(d), u는 가정값", fontsize=9.5, color="#51606e")
    ax = fig.add_axes([0.01, 0.05, 0.64, 0.86])
    ax.set_aspect("equal")
    ax.axis("off")
    hw_t, hw_b = 115, 105
    ax.add_patch(Polygon([(TX - hw_t - 3, 0), (TX + hw_t + 3, 0), (TX + hw_b + 3, -253), (TX - hw_b - 3, -253)],
                         fc="#F4F1EA", ec="#9A8F7A", lw=1.0))
    pts = [(TX - hw_b, -250)] + list(zip(xs, prof)) + [(TX + hw_b, -250)]
    ax.add_patch(Polygon(pts, closed=True, fc="#EFDDA6", ec="#b89a50", lw=0.6))
    ax.plot([TX - hw_t, TX + hw_t], [S0, S0], color="#b89a50", lw=0.6, ls=":")
    ax.add_patch(Rectangle((TX - 170, -20), 50, 20, fc=m.C_DECK, ec="#8E9AA5"))
    ax.add_patch(Rectangle((TX + 120, -20), 50, 20, fc=m.C_DECK, ec="#8E9AA5"))
    # keep-out band
    ax.plot([xa, xa], [-250, 40], color=m.C_SENSOR, lw=0.6, ls="--")
    ax.plot([xb, xb], [-250, 40], color=m.C_SENSOR, lw=0.6, ls="--")
    ax.text(xa, 44, "C 이동 한계", ha="center", fontsize=7, color=m.C_SENSOR)
    ax.text(xb, 44, "C 이동 한계", ha="center", fontsize=7, color=m.C_SENSOR)
    # trajectories
    cp = np.array([(p[0], p[1]) for p in path])
    ax.plot(cp[:, 0], cp[:, 1], color=m.C_MOVE, lw=1.0, ls="--")
    lows = np.array([m.rim_ends(*p)[1] for p in path])
    ax.plot(lows[:, 0], lows[:, 1], color=m.C_SENSOR, lw=1.0, ls=":")
    stages = [
        (xa, ZC0, -30, 0.0, False, "①", "접근·터치오프\nθ −30°, 10 mm/s\nΔF_z ≥ 3 N → Z0"),
        (x_dive, zc_drag, -30, 0.25, False, "②", "Dive 끝\nX·Z 30° 보간\n깊이 d"),
        (xb, zc_drag, -30, 0.8, False, "③", "Drag 끝\nX 80 mm/s\nF_x > 110 N → Z↑"),
        (xb, zc_drag, 90, 0.0, True, "④", "Close\nθ −30° → +90°\n앞쪽 구면 캡 절단"),
    ]
    alphas = [0.55, 0.65, 0.8, 1.0]
    for (x, z, th, fill, ball, num, txt), al in zip(stages, alphas):
        poly = m.bowl_polygon(x, z, th)
        if ball:
            ax.add_patch(Circle((x, z), m.R - 1.5, fc="#EFDDA6", ec="#8a7a55", lw=0.6, alpha=al, zorder=5))
        elif fill:
            r_in = m.R * math.sqrt(1 - fill)
            ax.add_patch(Polygon(np.vstack([m.bowl_polygon(x, z, th, m.R - 1.2), m.bowl_polygon(x, z, th, r_in)[::-1]]),
                                 closed=True, fc="#EFDDA6", ec="#9c8550", lw=0.4, alpha=al, zorder=5))
        ax.plot(poly[:, 0], poly[:, 1], color=m.C_FOOD, lw=2.0, alpha=al, zorder=6)
        ax.plot([x, x], [z, z + 60], color="#9aa3ab", lw=3, alpha=al * 0.8, zorder=4)
        ax.add_patch(Circle((x, z), 2.5, fc="white", ec=m.C_HEAD, lw=0.8, zorder=7))
        lo = m.rim_ends(x, z, th)[1]
        ax.add_patch(Circle(lo, 1.8, fc=m.C_SENSOR, ec="none", zorder=7))
        dx_lab = {"③": -16, "④": 16}.get(num, 0)
        ax.text(x + dx_lab, z + 68, num, ha="center", fontsize=12, fontweight="bold", color=m.C_TEXT, zorder=8)
    # stage text row
    for i, (x, z, th, fill, ball, num, txt) in enumerate(stages):
        ax.text(650, 95 - i * 72, f"{num} {txt}", fontsize=9, va="top", ha="left", color=m.C_TEXT, linespacing=1.35)
    ax.text(650, 95 - 4 * 72, "⑤ Lift\nZ_SAFE까지 수직 상승\n볼은 드래그 홈에서\n수직으로 빠진다", fontsize=9, va="top",
            color=m.C_TEXT, linespacing=1.35)
    # depth dimension
    xd = x_dive + 30
    ax.annotate("", xy=(xd, S0), xytext=(xd, S0 - d), arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text(xd + 3, S0 - d / 2, f"d = {d:.0f}", fontsize=8.5, va="center")
    ax.annotate("30°", xy=((xa + x_dive) / 2, (ZC0 + zc_drag) / 2), xytext=((xa + x_dive) / 2 - 60, ZC0 - 40),
                fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.5))
    ax.plot([], [], color=m.C_MOVE, ls="--", label="피벗 C 경로")
    ax.plot([], [], color=m.C_SENSOR, ls=":", label="rim 최저점 경로 = 절삭 바닥")
    ax.legend(loc="lower left", fontsize=8, frameon=False)
    ax.set_xlim(160, 780)
    ax.set_ylim(-265, 120)

    # swept area inset
    ai = fig.add_axes([0.68, 0.52, 0.30, 0.36])
    ai.set_aspect("equal")
    b = m.B_ATT
    t = np.linspace(0, 2 * math.pi, 200)
    ai.plot(m.R * np.cos(t), b * np.sin(t), color=m.C_FOOD, lw=1.2)
    hh = b - d
    yy = np.linspace(-m.R, m.R, 400)
    zz = -b * np.sqrt(np.clip(1 - (yy / m.R) ** 2, 0, None))
    mask = zz <= -hh
    ai.fill_between(yy[mask], zz[mask], -hh, color="#EFDDA6", ec="#b89a50")
    ai.axhline(-hh, color="#8a7a55", lw=0.8)
    ai.text(m.R + 2, -hh, " 표면", fontsize=8, va="center")
    A = cm.swept_area(30, d)
    ai.text(0, -hh - (b - hh) / 2 - 2, f"A = {A:,.0f} mm²", ha="center", fontsize=9, fontweight="bold")
    ai.set_title("절삭단면 A (진행방향으로 본 rim 투영)\n타원 반축 R = 35, R·cos30° = 30.3", fontsize=9, loc="left")
    ai.set_xlim(-45, 60)
    ai.set_ylim(-38, 38)
    ai.axis("off")

    at = fig.add_axes([0.68, 0.05, 0.30, 0.42])
    at.axis("off")
    rows = [("깊이 d", "20 mm", "25 mm", "28 mm")]
    for dd in (20, 25, 28):
        pass
    txt = ["깊이 d [mm]            20      25      28",
           "절삭단면 A [mm²]      959   1,297   1,505",
           f"F @ u 90 kPa [N]       86     117     135",
           f"F @ u 160 kPa [N]     153     207     241"]
    at.text(0, 1.0, "힘 = u · A(d)  (u: 비절삭저항, 가정 40/90/160/250 kPa)", fontsize=9.5, fontweight="bold",
            va="top", color=m.C_TEXT)
    for i, l in enumerate(txt):
        at.text(0, 0.86 - i * 0.085, l, fontsize=9, family=["DejaVu Sans Mono", "NanumGothic"], va="top")
    at.text(0, 0.46, "· 통 벽 때문에 레인이 90–140 mm로 짧아 1 portion(115 g)에 d ≈ 24–29 mm 필요\n"
            "· 드래그 중 F_x가 110 N을 넘으면 Z를 올려 깊이를 줄인다(깊이 적응)\n"
            "· 공격 자세 −30°에서는 볼 뒷면이 절삭 터널 밖으로 나가지 않는다\n"
            "· 공격 자세의 힘 이득은 미측정 → E0에서 0° vs −30° 비교",
            fontsize=8.6, va="top", color="#3b4650", linespacing=1.6)
    save(fig, "fig4_scoop_sequence.png")


# =====================================================================================
# Fig 5 — reaction load path
# =====================================================================================
def fig_load_path():
    fig = plt.figure(figsize=(15, 8.6))
    fig.text(0.012, 0.965, "반력 경로 — 절삭력이 손 대신 기계 구조로 흐른다 (F_x = 200 N, 가장 깊은 스쿱)", fontsize=15,
             fontweight="bold", color=m.C_TEXT)
    fig.text(0.012, 0.938, "calc/scoop_load_path.py · 구성 S30(강관 기둥·빔 + HGH15 + 볼스크류 + Ø30 스템). 하중은 가정 설계 케이스",
             fontsize=9.5, color="#51606e")
    ax = fig.add_axes([0.0, 0.03, 0.6, 0.88])
    ax.set_aspect("equal")
    ax.axis("off")
    X, Z = 860.0, cm.z_min()
    m.draw_frame_static(ax, surfaces={"PNB": -214.0})
    m.draw_carriage(ax, X, Z, -30, fill=0.5, ice="#C9955B")
    # force at the scoop (reaction on the scoop, opposite to the +X drag)
    fz = Z - 22
    ax.add_patch(FancyArrowPatch((X + 10, fz), (X - 190, fz), arrowstyle="-|>", mutation_scale=16, lw=2.4,
                                 color=m.C_SENSOR, zorder=20))
    ax.text(X - 120, fz - 75, "F_x = 200 N (스쿱이 받는 반력)", fontsize=8.5, color=m.C_SENSOR, ha="center", va="top",
            zorder=22, bbox=dict(fc="white", ec=m.C_SENSOR, lw=0.6, pad=2))
    # lever dimension
    ax.annotate("", xy=(X + 150, fz), xytext=(X + 150, m.ZBLOCKS[0]), arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text(X + 160, (fz + m.ZBLOCKS[0]) / 2, "레버 807 mm\nM = 161 N·m", fontsize=8.5, va="center")
    # path arrows (numbered)
    path = [(X, fz + 30), (X, Z + 300), (X, m.ZBLOCKS[0]), (X - 60, 760), (X - 300, 810), (-200, 810),
            (-220, 300), (-220, -800), (640, -830), (740, -40), (800, -170)]
    for (a, b) in zip(path[:-1], path[1:]):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=10, lw=1.4, color="#7B61A8",
                                     connectionstyle="arc3,rad=0.0", zorder=19, alpha=0.9))
    labels = [((X - 40, Z + 150), "① 스템 0.49"), ((X - 45, Z + 480), "② Z 기둥 0.32"),
              ((X + 45, 610), "③ Z 블록 0.06"), ((X - 20, 900), "④ Y·X 블록 0.12"),
              ((400, 880), "⑤ X 볼스크류 0.01 · 빔 0.05"), ((-200, 400), "⑥ 포스트·베이스"),
              ((250, -780), "⑦ 베이스 → 냉동고 · 데크 웰"), ((420, -440), "⑧ 통 → 아이스크림 (루프가 닫힘)")]
    for (x, y), t in labels:
        ax.text(x, y, t, fontsize=8.3, color="#4A3A78", ha="center",
                bbox=dict(fc="white", ec="#b9aed6", lw=0.6, pad=1.8), zorder=21)
    ax.set_xlim(-420, 1560)
    ax.set_ylim(-900, 1250)

    # right: bar of deflection contributions per configuration
    ab = fig.add_axes([0.66, 0.55, 0.32, 0.33])
    names = ["L 3D 프린터식", "M 보강 프로파일", "S 강관 + Ø25 스템", "S30 (채택)"]
    vals = [10.86, 2.18, 1.40, 0.99]
    cols = ["#C9CED4", "#C9CED4", "#C9CED4", m.C_MOVE]
    ab.barh(names[::-1], vals[::-1], color=cols[::-1], height=0.6)
    for i, v in enumerate(vals[::-1]):
        ab.text(v + 0.15, i, f"{v:.2f} mm", va="center", fontsize=8.5)
    ab.axvline(1.0, color=m.C_SENSOR, lw=0.8, ls="--")
    ab.set_ylim(-0.95, 3.5)
    ab.text(1.1, -0.72, "목표 ~1 mm", fontsize=8, color=m.C_SENSOR, va="center")
    ab.set_xlim(0, 13)
    ab.set_title("스쿱 끝 변위 @ 200 N (드래그 방향)", fontsize=9.5, loc="left")
    ab.tick_params(labelsize=8.5)
    for sp in ("top", "right"):
        ab.spines[sp].set_visible(False)

    tx = fig.add_axes([0.66, 0.05, 0.32, 0.42])
    tx.axis("off")
    items = [
        ("닫힌 구조 루프", "스쿱 → 갠트리 → 포스트 → 베이스 → 데크 웰 → 통 → 아이스크림 → 스쿱. 공작기계처럼 공구와 공작물 사이의 상대 변위가 깊이 정확도를 정한다."),
        ("용량이 아니라 강성", "HGH15 블록 짝힘 ~0.54 kN vs 정격 17 kN으로 용량은 충분. 문제는 0.8 m 레버에서의 휨이다."),
        ("드래그 방향 = 빔 축", "빔을 비틀지 않고 축력으로 받는다. 벨트(GT2)는 작업장력 ~28 N이라 드래그 축으로 못 쓴다."),
        ("로드셀은 볼너트에", "X·Z 로드셀은 축방향 힘만 받는다(모멘트 경로 밖). 모멘트는 블록 짝힘이 받는다."),
    ]
    y = 1.0
    for h, t in items:
        tx.text(0, y, h, fontsize=10, fontweight="bold", color=m.C_TEXT, va="top")
        tx.text(0, y - 0.07, t, fontsize=8.6, color="#3b4650", va="top", wrap=True)
        y -= 0.25
    save(fig, "fig5_load_path.png")


if __name__ == "__main__":
    fig_axonometric()
    fig_three_view()
    fig_head()
    fig_sequence()
    fig_load_path()
