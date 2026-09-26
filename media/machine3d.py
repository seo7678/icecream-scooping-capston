"""Axonometric (3D-looking) drawing of the V1 machine with all three global axes
and the local pitch axis, for fig1/fig6 and the 3D operation video.

A small painter's-algorithm renderer is used instead of mplot3d so that the
large flat parts sort correctly and the tub interiors can be clipped to their
openings. Coordinates: machine frame M (mm), Z_M = pivot C height.
"""

import math

import numpy as np
import matplotlib.colors as mcolors
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle, Polygon, PathPatch, FancyArrowPatch
from matplotlib.path import Path

import machine as m

# ------------------------------------------------------------------ Y cross-slide geometry (V1)
YB_X = (60.0, 90.0)          # two Y beams at X_C +- 60..90 (fork, the Z plate passes between them)
YB_Y = (-140.0, 200.0)
YB_Z = (700.0, 725.0)
YBRACKET_Y = (170.0, 200.0)  # beams bolted to the X carriage here
YSLIDER_DY = (30.0, 80.0)    # slider plate spans Y_C+30 .. Y_C+80
YSCREW_DX = 105.0
ZPLATE_HALF = 40.0
YSTROKE = 80.0


def convex_hull(P):
    """Monotone-chain convex hull of 2D points (returns vertices in order)."""
    pts = sorted(map(tuple, np.round(P, 6)))
    if len(pts) < 3:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


class Axo:
    def __init__(self, az_deg=24.0, el_deg=21.0):
        self.az, self.el = math.radians(az_deg), math.radians(el_deg)
        ca, sa, ce, se = math.cos(self.az), math.sin(self.az), math.cos(self.el), math.sin(self.el)
        self.grad = np.array([-sa * ce, ca * ce, -se])            # direction of increasing depth
        self.toward = -self.grad
        self._c = (ca, sa, ce, se)
        self.faces = []

    def proj(self, p):
        ca, sa, ce, se = self._c
        x, y, z = p
        x1 = x * ca + y * sa
        y1 = -x * sa + y * ca
        return x1, z * ce + y1 * se, y1 * ce - z * se

    def pts(self, P):
        P = np.asarray(P, float)
        ca, sa, ce, se = self._c
        x1 = P[:, 0] * ca + P[:, 1] * sa
        y1 = -P[:, 0] * sa + P[:, 1] * ca
        return np.column_stack([x1, P[:, 2] * ce + y1 * se])

    def depth(self, p):
        return self.proj(p)[2]

    def box(self, x0, x1, y0, y1, z0, z1, color, ec="#3b4650", lw=0.5, alpha=1.0, zbias=0.0):
        v = {(i, j, k): (x, y, z) for i, x in enumerate((x0, x1)) for j, y in enumerate((y0, y1))
             for k, z in enumerate((z0, z1))}
        faces = [
            ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), (0, -1, 0), 0.92),
            ((0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1), (0, 1, 0), 0.80),
            ((0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (-1, 0, 0), 0.78),
            ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1), (1, 0, 0), 0.78),
            ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1), (0, 0, 1), 1.00),
            ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, -1), 0.60),
        ]
        rgb = np.array(mcolors.to_rgb(color))
        for a, b, c, d, n, f in faces:
            if np.dot(n, self.grad) >= 0:
                continue
            P = [self.proj(v[k]) for k in (a, b, c, d)]
            dep = np.mean([p[2] for p in P])
            self.faces.append((dep - zbias, [(p[0], p[1]) for p in P], tuple(np.clip(rgb * f, 0, 1)), ec, lw, alpha))

    def flush(self):
        out = sorted(self.faces, key=lambda f: -f[0])
        self.faces = []
        return out

    def draw(self, ax, faces, zorder, clip=None, alpha=None):
        for _, verts, fc, ec, lw, al in faces:
            p = Polygon(verts, closed=True, fc=fc, ec=ec, lw=lw, alpha=al if alpha is None else alpha, zorder=zorder)
            ax.add_patch(p)
            if clip is not None:
                p.set_clip_path(clip)

    # ---------------------------------------------------------------- helpers
    def circle3(self, c, r, u, v, n=72):
        t = np.linspace(0, 2 * math.pi, n, endpoint=False)
        c = np.asarray(c, float)
        return self.pts([c + r * (math.cos(a) * u + math.sin(a) * v) for a in t])

    def rim_ellipse(self, cx, cy, z, r, n=90):
        return self.circle3((cx, cy, z), r, np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), n)

    def above_clip(self, ax, cx, cy, r, top=5000.0):
        """Region = opening ellipse at (cx, cy, 0) plus everything above its near (lower) arc.
        Parts below the deck are visible only through the opening."""
        e = self.rim_ellipse(cx, cy, 0.0, r, 180)
        i_l, i_r = int(np.argmin(e[:, 0])), int(np.argmax(e[:, 0]))
        # lower arc: points between left and right extremes with smaller v
        idx = np.arange(len(e))
        a1 = np.roll(idx, -i_l)
        k = int(np.where(a1 == i_r)[0][0])
        arc1, arc2 = e[a1[:k + 1]], e[np.r_[a1[k:], a1[:1]]]
        lower = arc1 if arc1[:, 1].mean() < arc2[:, 1].mean() else arc2[::-1]
        if lower[0, 0] > lower[-1, 0]:
            lower = lower[::-1]
        verts = np.vstack([lower, [[lower[-1, 0], top], [lower[0, 0], top]]])
        patch = Polygon(verts, closed=True, fc="none", ec="none")
        ax.add_patch(patch)
        return patch


# ------------------------------------------------------------------ static parts
def base_faces(A):
    for x0, x1 in m.POSTS_X:
        A.box(x0, x1, 185, 265, -850, 880, m.C_ALU)
    A.box(-200, 1350, 200, 250, 760, 860, m.C_STEEL)
    A.box(-170, 1320, 186, 200, 772, 786, "#44505b", lw=0.3)
    A.box(-170, 1320, 186, 200, 834, 848, "#44505b", lw=0.3)
    A.box(1290, 1370, 178, 250, 772, 848, "#2E3A46")
    A.box(*m.FREEZER_X, *m.FREEZER_Y, -850, -20, "#F2F5F8", ec="#9aa6b2")
    A.box(m.FREEZER_X[0] - 20, m.FREEZER_X[1] + 20, -270, 270, -20, 0, m.C_DECK, ec="#8E9AA5")
    A.box(m.RINSE_X - 80, m.RINSE_X + 80, -80, 80, -160, 0, "#DCE3E8", ec="#8E9AA5")
    A.box(m.RINSE_X - 25, m.RINSE_X + 25, -25, 25, -850, -160, "#D0D6DC", ec="#9aa6b2")
    A.box(m.CUP_X - 95, m.CUP_X + 95, -95, 95, -140, 0, "#DCE3E8", ec="#8E9AA5")
    A.box(m.CUP_X - 25, m.CUP_X + 25, -25, 25, -850, -140, "#D0D6DC", ec="#9aa6b2")
    return A.flush()


def draw_floor(ax, A):
    ax.add_patch(Polygon(A.pts([(-420, -420, -850), (1600, -420, -850), (1600, 420, -850), (-420, 420, -850)]),
                         closed=True, fc="#EEF1F4", ec="none", zorder=0.5))


def draw_tubs(ax, A, heightmaps=None, levels=None):
    """Tub openings with ice cream surfaces. heightmaps: fid -> (xs, ys, S) grid; levels: fid -> flat z."""
    for fid, name, tx, s, col in m.FLAVORS:
        rim = A.rim_ellipse(tx, 0, 0, 115)
        op = Polygon(rim, closed=True, fc="#E9E4D8", ec="#6f6656", lw=1.0, zorder=2)
        ax.add_patch(op)
        if heightmaps and fid in heightmaps:
            xs, ys, S, S0 = heightmaps[fid]
            h = (xs[1] - xs[0]) / 2
            XX, YY = np.meshgrid(xs, ys, indexing="ij")
            ok = np.isfinite(S)
            cx, cy, cz = XX[ok], YY[ok], S[ok]
            n = cx.size
            corners = np.empty((n, 4, 3))
            for k, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
                corners[:, k, 0] = cx + sx * h
                corners[:, k, 1] = cy + sy * h
                corners[:, k, 2] = cz
            P = A.pts(corners.reshape(-1, 3)).reshape(n, 4, 2)
            ca, sa, ce, se = A._c
            dep = (-cx * sa + cy * ca) * ce - cz * se
            f = np.clip((S0 - cz) / 40.0, 0, 1)[:, None]
            base = np.array(mcolors.to_rgb(col))[None, :]
            dark = np.array(mcolors.to_rgb("#7a5a1c"))[None, :]
            cols = base * (1 - 0.6 * f) + dark * 0.6 * f
            order = np.argsort(dep)[::-1]
            pc = PolyCollection(P[order], facecolors=cols[order], edgecolors="none", zorder=2.1)
            ax.add_collection(pc)
            pc.set_clip_path(op)
        else:
            z = (levels or {}).get(fid, s)
            rs = m.tub_half_width(z)
            p = Polygon(A.rim_ellipse(tx, 0, z, rs), closed=True, fc=col, ec="none", zorder=2.1)
            ax.add_patch(p)
            p.set_clip_path(op)
    # rinse water + cup opening
    ax.add_patch(Polygon(A.rim_ellipse(m.RINSE_X, 0, -20, 70), closed=True, fc=m.C_WATER, ec="#7aa9bf", lw=0.6,
                         zorder=2))


def draw_cup(ax, A, present=True, balls=()):
    ax.add_patch(Polygon(A.rim_ellipse(m.CUP_X, 0, 0, 46), closed=True, fc="#c9d1d8", ec="#8E9AA5", lw=0.9,
                         zorder=2))
    if present:
        op = Polygon(A.rim_ellipse(m.CUP_X, 0, 0, 40), closed=True, fc="white", ec="#8E9AA5", lw=0.9, zorder=2.2)
        ax.add_patch(op)
        ax.add_patch(Polygon(A.rim_ellipse(m.CUP_X, 0, -60, 31), closed=True, fc="#e8ecef", ec="none", zorder=2.25))
        clip = A.above_clip(ax, m.CUP_X, 0, 40)
        for (bx, by, bz, br, col) in balls:
            c = A.proj((bx, by, bz))
            b = Circle(c[:2], br, fc=col, ec="#8a7a55", lw=0.6, zorder=6.5)
            ax.add_patch(b)
            b.set_clip_path(clip)


# ------------------------------------------------------------------ moving parts
def carriage_faces(A, X, Y, Z, th):
    """Faces of X carriage, Y cross-slide, Z plate/column and head at pose (X, Y, Z, th)."""
    A.box(X - 100, X + 100, 160, 200, 735, 885, m.C_X, ec="#1E4E80")                        # X carriage
    A.box(X - 95, X + 95, *YBRACKET_Y, 700, 740, "#16736A", ec="#0f5a52")                   # Y bracket
    for s in (-1, 1):                                                                         # Y beams + rails
        x0, x1 = sorted((X + s * YB_X[0], X + s * YB_X[1]))
        A.box(x0, x1, *YB_Y, *YB_Z, m.C_Y, ec="#0f5a52")
        A.box(x0 + 8, x1 - 8, YB_Y[0] + 10, YB_Y[1] - 20, YB_Z[1], YB_Z[1] + 6, "#26313D", lw=0.2)
    A.box(X + YSCREW_DX - 4, X + YSCREW_DX + 4, YB_Y[0] + 10, YBRACKET_Y[0], 708, 716, "#707b86", lw=0.2)
    A.box(X + YSCREW_DX - 18, X + YSCREW_DX + 18, YBRACKET_Y[0], YBRACKET_Y[1] + 30, 694, 730, "#2E3A46")  # Y motor
    A.box(X - 90, X + YSCREW_DX + 10, Y + YSLIDER_DY[0], Y + YSLIDER_DY[1], 731, 746, m.C_Y_L, ec="#0f5a52")
    A.box(X - ZPLATE_HALF, X + ZPLATE_HALF, Y + 30, Y + 50, m.ZPLATE[0], m.ZPLATE[1], m.C_Z_L, ec=m.C_Z,
          zbias=-600)
    A.box(X - 21, X + 21, Y + 30, Y + 72, m.ZPLATE[1] - 45, m.ZPLATE[1], "#2E3A46")        # Z motor
    cb = Z + m.STEM + m.HEAD
    A.box(X - 30, X + 30, Y - 30, Y + 30, cb, cb + m.COL_LEN, m.C_STEEL, ec=m.C_STEEL_D)     # Z column
    A.box(X - 60, X + 60, Y - 45, Y + 45, Z + m.STEM, cb, m.C_HEAD, ec="#1b242e")           # head
    A.box(X - 10, X + 50, Y - 95, Y - 45, Z + m.STEM + 25, Z + m.STEM + 85, "#6C7E92", ec="#26313d")
    A.box(X - 85, X + 85, Y - 70, Y + 70, Z + m.STEM - 5, Z + m.STEM, m.C_FOOD, ec="#a86d00", lw=0.4)
    return A.flush()


def food_faces(A, X, Y, Z, th):
    A.box(X - 7.5, X + 7.5, Y - 7.5, Y + 7.5, Z + 4, Z + m.STEM - 5, "#D7DCE1", ec=m.C_FOOD)
    lx, lz = m.lever_tip(X, Z, th)
    A.box(lx - 4, lx + 4, Y - 4, Y + 4, lz, lz + m.STEM - 5, "#E7C77F", ec=m.C_FOOD, lw=0.4)
    return A.flush()


def draw_scoop3d(ax, A, X, Y, Z, th, fill=0.0, ball_r=None, ice="#EFDDA6", clip=None, zorder=6):
    """Hemispherical bowl: shell silhouette + rim; chip or captured ball."""
    t = math.radians(th)
    n = np.array([math.cos(t), 0.0, math.sin(t)])
    C = np.array([X, Y, Z])
    u = np.array([0.0, 1.0, 0.0])
    v = np.cross(n, u)
    pts = []
    # sample the half sphere on the -n side (the shell)
    for a in np.linspace(0, math.pi / 2, 10):
        for b in np.linspace(0, 2 * math.pi, 40, endpoint=False):
            d = -math.cos(a) * n + math.sin(a) * (math.cos(b) * u + math.sin(b) * v)
            pts.append(C + m.R * d)
    hull = convex_hull(A.pts(pts))
    shell = Polygon(hull, closed=True, fc="#E7B75A", ec="#a86d00", lw=0.8, zorder=zorder)
    ax.add_patch(shell)
    rim = A.circle3(C, m.R, u, v)
    facing = float(np.dot(n, A.toward)) > 0
    rim_p = Polygon(rim, closed=True, fc="#F6E6C2" if facing else "none", ec="#a86d00", lw=0.9, zorder=zorder + 0.1)
    ax.add_patch(rim_p)
    patches = [shell, rim_p]
    if fill > 0 and ball_r is None and facing:
        ch = Polygon(A.circle3(C, m.R * 0.92, u, v), closed=True, fc=ice, ec="none", alpha=min(1.0, 0.25 + fill),
                     zorder=zorder + 0.2)
        ax.add_patch(ch)
        patches.append(ch)
    if ball_r:
        c2 = A.proj(C)
        b = Circle(c2[:2], ball_r, fc=ice, ec="#8a7a55", lw=0.6, zorder=zorder + 0.3)
        ax.add_patch(b)
        patches.append(b)
    pv = A.proj(C)
    pin = Circle(pv[:2], 4, fc="white", ec=m.C_HEAD, lw=0.7, zorder=zorder + 0.4)
    ax.add_patch(pin)
    patches.append(pin)
    if clip is not None:
        for p in patches:
            p.set_clip_path(clip)
    return patches


def draw_enclosure(ax, A, alpha=0.06):
    E = [(-320, -300, 0), (1500, -300, 0), (1500, 300, 0), (-320, 300, 0),
         (-320, -300, 1220), (1500, -300, 1220), (1500, 300, 1220), (-320, 300, 1220)]
    P = A.pts(E)
    for a, b in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]:
        ax.plot([P[a][0], P[b][0]], [P[a][1], P[b][1]], color="#7FA3C6", lw=0.8, ls=(0, (5, 3)), zorder=9)
    ax.add_patch(Polygon([P[0], P[1], P[5], P[4]], closed=True, fc="#9CC0E6", ec="none", alpha=alpha, zorder=9))
    bay = A.pts([(1040, -300, 0), (1200, -300, 0), (1200, -300, 220), (1040, -300, 220)])
    ax.add_patch(Polygon(bay, closed=True, fc="white", ec="#c24", lw=1.0, ls="--", alpha=0.6, zorder=9.1))
    for zz in (60, 150):
        for xx in (1040, 1200):
            q = A.proj((xx, -300, zz))
            ax.add_patch(Circle(q[:2], 9, fc=m.C_SENSOR, ec="none", zorder=9.2))
    hmi = A.pts([(-290, -302, 160), (-150, -302, 160), (-150, -302, 320), (-290, -302, 320)])
    ax.add_patch(Polygon(hmi, closed=True, fc="#26313d", ec="#111", lw=0.6, zorder=9.3))
    es = A.proj((-220, -302, 400))
    ax.add_patch(Circle(es[:2], 26, fc="#FFD400", ec="#333", lw=0.6, zorder=9.3))
    ax.add_patch(Circle(es[:2], 16, fc="#D0021B", ec="none", zorder=9.4))


def axis_arrow(ax, A, p0, p1, label, color, lw=2.2, fs=11, zorder=12, label_at=1.0, offset=(0, 0)):
    a, b = A.proj(p0)[:2], A.proj(p1)[:2]
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="<|-|>", mutation_scale=13, lw=lw, color=color, zorder=zorder))
    lx = a[0] + (b[0] - a[0]) * label_at + offset[0]
    ly = a[1] + (b[1] - a[1]) * label_at + offset[1]
    ax.text(lx, ly, label, color=color, fontsize=fs, fontweight="bold", ha="center", va="center", zorder=zorder + 1,
            bbox=dict(fc="white", ec=color, lw=0.8, boxstyle="round,pad=0.2"))


AXIS_COL = {"X": m.C_X, "Y": m.C_Y, "Z": m.C_Z, "θ": m.C_TH}


def axis_style(key, active):
    """Active axis: full colour, thick; inactive: same hue, pale and thin (colour code stays readable)."""
    col = AXIS_COL[key]
    if not active or key in active:
        return col, 2.4, 1.0
    return col, 1.2, 0.35


def draw_axis_arrows(ax, A, X, Y, Z, active=(), fs=11):
    cb = Z + m.STEM + m.HEAD
    specs = [
        ("X", (X - 190, 150, 925), (X + 190, 150, 925), 1.0, (30, 0)),
        ("Y", (X - 125, YB_Y[0] + 5, 760), (X - 125, YB_Y[1] - 30, 760), 0.0, (-26, -12)),
        ("Z", (X - 115, Y, Z + 140), (X - 115, Y, Z + 440), 0.5, (-30, 0)),
    ]
    for key, p0, p1, at, off in specs:
        col, lw, al = axis_style(key, active)
        a, b = A.proj(p0)[:2], A.proj(p1)[:2]
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle="<|-|>", mutation_scale=13, lw=lw, color=col, alpha=al,
                                     zorder=12))
        lx, ly = a[0] + (b[0] - a[0]) * at + off[0], a[1] + (b[1] - a[1]) * at + off[1]
        ax.text(lx, ly, key, color="white" if al == 1.0 else col, fontsize=fs, fontweight="bold", ha="center",
                va="center", zorder=13, alpha=1.0 if al == 1.0 else 0.8,
                bbox=dict(fc=col if al == 1.0 else "white", ec=col, lw=0.8, boxstyle="round,pad=0.2"))
    col, lw, al = axis_style("θ", active)
    t = np.linspace(math.radians(-80), math.radians(80), 30)
    arc = A.pts([(X - 62 * math.cos(a), Y, Z + 62 * math.sin(a)) for a in t])
    ax.plot(arc[:, 0], arc[:, 1], color=col, lw=lw, alpha=al, zorder=12)
    ax.add_patch(FancyArrowPatch(tuple(arc[-3]), tuple(arc[-1]), arrowstyle="-|>", mutation_scale=12, color=col,
                                 lw=lw, alpha=al, zorder=12))
    ax.text(arc[0][0] - 22, arc[0][1] - 10, "θ", color="white" if al == 1.0 else col, fontsize=fs,
            fontweight="bold", ha="center", va="center", zorder=13,
            bbox=dict(fc=col if al == 1.0 else "white", ec=col, lw=0.8, boxstyle="round,pad=0.2"))


def draw_triad(ax, A, origin_screen, L=150, fs=9):
    ox, oy = origin_screen
    for vec, lab in (((1, 0, 0), "X_M"), ((0, 1, 0), "Y_M"), ((0, 0, 1), "Z_M")):
        d = A.proj(np.array(vec) * L)[:2]
        o = A.proj((0, 0, 0))[:2]
        dx, dy = d[0] - o[0], d[1] - o[1]
        ax.add_patch(FancyArrowPatch((ox, oy), (ox + dx, oy + dy), arrowstyle="-|>", mutation_scale=11,
                                     color="#34495e", lw=1.4, zorder=12))
        ax.text(ox + dx * 1.22, oy + dy * 1.22, lab, fontsize=fs, color="#34495e", ha="center", va="center",
                zorder=12)


def draw_machine(ax, A, X, Y, Z, th, fill=0.0, ball_r=None, heightmaps=None, cup=True, cup_balls=(),
                 active=(), arrows=True, enclosure=True, free_ball=None, ice="#EFDDA6"):
    draw_floor(ax, A)
    A.draw(ax, base_faces(A), 1)
    draw_tubs(ax, A, heightmaps=heightmaps)
    draw_cup(ax, A, present=cup, balls=cup_balls)
    A.draw(ax, carriage_faces(A, X, Y, Z, th), 4)
    over_tub = None
    for fid, name, tx, s, col in m.FLAVORS:
        if abs(X - tx) < 110 and Z - m.R < 10:
            over_tub = tx
    clip = A.above_clip(ax, over_tub, 0, 115) if over_tub is not None else None
    A.draw(ax, food_faces(A, X, Y, Z, th), 5, clip=clip)
    draw_scoop3d(ax, A, X, Y, Z, th, fill=fill, ball_r=ball_r, ice=ice, clip=clip, zorder=6)
    if free_ball is not None:
        bx, by, bz, br = free_ball
        c = A.proj((bx, by, bz))
        ax.add_patch(Circle(c[:2], br, fc=ice, ec="#8a7a55", lw=0.6, zorder=6.6))
    if enclosure:
        draw_enclosure(ax, A)
    if arrows:
        draw_axis_arrows(ax, A, X, Y, Z, active=active)
