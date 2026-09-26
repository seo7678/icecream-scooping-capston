"""3D operation video: one automatic single scoop that uses all four axes (X, Y, Z, theta).

Run:  python3 media/make_operation_video_3d.py
Out:  media/operation_3d.mp4 (1280x720, 30 fps, real time) and media/operation_3d.gif

The order lands on a fresh layer of TUB1, so the lane rule (highest lane, ties in
lane_set order -40 / 0 / +40) sends the head to lane y = -40: the Y cross-slide
moves out and back. The ice cream is cut on a 2.5 mm height map by the swept
half-ball of the scoop, so lane length, overlap and the closing cap come out of
the geometry. Cutting force F = u * dV/ds with u = 90 kPa (ASSUMPTION A02);
captured volume = dive + drag + half of the closing cut (CLOSE_CAPTURE, as in
calc/tub_lane_planner.py). The eject step is schematic. Nothing here is measured.

Requires: numpy, matplotlib, koreanize-matplotlib, imageio-ffmpeg.
"""

import math
import os
import subprocess
import sys
import warnings

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Circle, Polygon, Rectangle, FancyBboxPatch, FancyArrowPatch

import machine as m
import machine3d as m3
from machine import cm
import tub_lane_planner as tlp

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_MP4 = os.path.join(HERE, "operation_3d.mp4")
OUT_GIF = os.path.join(HERE, "operation_3d.gif")
MONO = ["DejaVu Sans Mono", "NanumGothic"]
FPS = 30
DT = 1.0 / FPS
try:
    import imageio_ffmpeg
    FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
    plt.rcParams["animation.ffmpeg_path"] = FFMPEG
except ImportError:  # pragma: no cover
    FFMPEG = "ffmpeg"

# ------------------------------------------------------------------ order + assumptions
FID, FNAME, TX, S_EST, ICE = m.FLAVORS[0]
S_TRUE = S_EST - 1.5
LANE_Y = -40.0                        # fresh layer: first lane of lane_set [-40, 0, +40]
U_KPA, F_TARGET, F_STOP = 90.0, 110.0, 160.0
RHO, M_TARGET, K_F = 0.65, 115.0, 1.00
V_TARGET = M_TARGET / (RHO * K_F) * 1e3
CLOSE_CAPTURE = 0.5
CUP_G = 5.0
V_XY, A_XY, V_Z, A_Z = 250.0, 1000.0, 150.0, 1000.0
D_PLAN = 26.0

# ------------------------------------------------------------------ height map of TUB1
HM = 2.5
xs = TX + np.arange(-116.0, 116.01, HM)
ys = np.arange(-116.0, 116.01, HM)
XX, YY = np.meshgrid(xs, ys, indexing="ij")
RR = np.hypot(XX - TX, YY)
S = np.where(RR <= m.tub_half_width(S_TRUE), S_TRUE, np.nan)
J_LANE = int(np.argmin(np.abs(ys - LANE_Y)))


def half_ball_lowest(dx, dy, th_deg):
    """Lowest z (relative to C) of the scoop half-ball above horizontal offsets (dx, dy); nan if none."""
    t = math.radians(th_deg)
    ct, st = math.cos(t), math.sin(t)
    rho2 = dx * dx + dy * dy
    h = np.sqrt(np.clip(m.R ** 2 - rho2, 0, None))
    low = -h.copy()
    if st < -1e-9:
        bound = -ct * dx / st                 # dz >= bound
        low = np.maximum(low, bound)
        low = np.where(low <= h, low, np.nan)
    elif st > 1e-9:
        bound = -ct * dx / st                 # dz <= bound
        low = np.where(-h <= bound, low, np.nan)
    else:
        low = np.where(ct * dx <= 0, low, np.nan)
    return np.where(rho2 < m.R ** 2, low, np.nan)


def cut(X, Y, Z, th):
    """Apply the scoop at pose; return removed volume [mm^3]."""
    i0, i1 = np.searchsorted(xs, X - m.R - HM), np.searchsorted(xs, X + m.R + HM)
    j0, j1 = np.searchsorted(ys, Y - m.R - HM), np.searchsorted(ys, Y + m.R + HM)
    sub = S[i0:i1, j0:j1]
    dz = half_ball_lowest(XX[i0:i1, j0:j1] - X, YY[i0:i1, j0:j1] - Y, th)
    zlow = Z + dz
    mask = np.isfinite(zlow) & np.isfinite(sub) & (zlow < sub)
    removed = float(np.sum((sub - zlow)[mask])) * HM * HM
    sub[mask] = zlow[mask]
    return removed


def trapezoid(dist, vmax, amax):
    L = abs(dist)
    sign = 1.0 if dist >= 0 else -1.0
    if L < 1e-9:
        return 0.0, lambda t: 0.0
    ta = vmax / amax
    if L < vmax * ta:
        ta = math.sqrt(L / amax)
        vmax = amax * ta
        T = 2 * ta
    else:
        T = L / vmax + ta

    def s(t):
        t = min(max(t, 0.0), T)
        if t < ta:
            return sign * 0.5 * amax * t * t
        if t > T - ta:
            return sign * (L - 0.5 * amax * (T - t) ** 2)
        return sign * (0.5 * amax * ta * ta + vmax * (t - ta))
    return T, s


def ease(u):
    u = min(max(u, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * u)


# ------------------------------------------------------------------ simulation
DESC = {
    "IDLE": "대기 — 작업자가 컵을 매립 웰에 놓는다",
    "FLAVOR_SELECT": "바닐라 → 조회표: TUB1 (340, 0) · 새 층이라 레인 규칙상 y = −40 (lane_set 순서)",
    "CHECK_CUP": "컵 확인: 저울 5.0 g > 3 g, 컵 베이 빔 비어 있음",
    "XY_MOVE_TO_TUB": "Z ≥ Z_SAFE에서 X와 Y를 동시에 이동 (Y 크로스슬라이드 0 → −40) · θ를 공격 자세로",
    "Z_APPROACH": "Z 빠른 하강: 표면 추정보다 10 mm 위까지",
    "SURFACE_DETECT": "10 mm/s · Z 볼너트 로드셀 ΔF ≥ 3 N → 표면 기준 Z0",
    "SCOOP_DIVE": "X·Z 30° 보간으로 절입 (Y는 레인에 고정)",
    "SCOOP_DRAG": "X 드래그 80 mm/s · F_x > 110 N이면 Z를 올림 · 레인 y = −40은 벽 때문에 짧다",
    "SCOOP_CLOSE": "X 정지, θ −30° → +90° 로 공 완성",
    "Z_LIFT": "Z_SAFE까지 상승 (그 전에는 XY 잠금)",
    "XY_MOVE_TO_CUP": "컵 스테이션으로: X와 Y 동시 이동 (Y −40 → 0)",
    "BAY_CHECK": "컵 베이 빔 비어 있음 확인",
    "Z_DISPENSE": "배출 높이로 하강",
    "EJECT": "배출: θ −30° + 흔들기 + X 후퇴 (방식은 V1a 시험으로 확정 — 개략 표시)",
    "Z_RETRACT": "Z_SAFE 복귀",
    "PORTION_CHECK": "저울 안정 1 s → 질량 판정 · 절삭 이력 지도 갱신",
    "DONE": "완료",
}
STATES = [("FLAVOR_SELECT", "맛 선택"), ("XY_MOVE_TO_TUB", "XY 이동"), ("Z_APPROACH", "Z 접근"),
          ("SURFACE_DETECT", "표면 검출"), ("SCOOP_DIVE", "Dive"), ("SCOOP_DRAG", "Drag"), ("SCOOP_CLOSE", "Close"),
          ("Z_LIFT", "Z 상승"), ("XY_MOVE_TO_CUP", "컵 이동"), ("EJECT", "배출"), ("PORTION_CHECK", "계량")]
STRIP_MAP = {"IDLE": "FLAVOR_SELECT", "CHECK_CUP": "FLAVOR_SELECT", "BAY_CHECK": "XY_MOVE_TO_CUP",
             "Z_DISPENSE": "EJECT", "Z_RETRACT": "EJECT", "DONE": "PORTION_CHECK"}
ACTIVE = {"XY_MOVE_TO_TUB": ("X", "Y", "θ"), "Z_APPROACH": ("Z",), "SURFACE_DETECT": ("Z",),
          "SCOOP_DIVE": ("X", "Z"), "SCOOP_DRAG": ("X", "Z"), "SCOOP_CLOSE": ("θ",), "Z_LIFT": ("Z",),
          "XY_MOVE_TO_CUP": ("X", "Y"), "Z_DISPENSE": ("Z",), "EJECT": ("θ", "X"), "Z_RETRACT": ("Z",)}

frames = []


class Sim:
    pass


sim = Sim()
sim.t, sim.X, sim.Y, sim.Z, sim.th = 0.0, 0.0, 0.0, m.Z_TOP, 90.0
sim.fill, sim.ball, sim.ball_free, sim.ball_in_cup, sim.cup = 0.0, False, None, False, False
sim.scale, sim.Fx, sim.Fz, sim.V, sim.state, sim.note, sim.pressed = 0.0, 0.0, 0.0, 0.0, "IDLE", "", False
sim.touch_t, sim.mass, sim.ball_r = None, None, m.R - 1.5
S0_MAP = S.copy()
RENDER_EVERY = 1
np.random.seed(5)


def snap():
    noise = np.random.normal(0, 0.4)
    frames.append(dict(t=sim.t, X=sim.X, Y=sim.Y, Z=sim.Z, th=sim.th, fill=sim.fill, ball=sim.ball,
                       ball_free=sim.ball_free, ball_in_cup=sim.ball_in_cup, cup=sim.cup, scale=sim.scale,
                       Fx=max(0.0, sim.Fx + noise), Fz=max(0.0, sim.Fz + 0.5 * noise), V=sim.V, state=sim.state,
                       note=sim.note, pressed=sim.pressed, touch_t=sim.touch_t, S=S.copy(), ball_r=sim.ball_r))
    sim.t += DT


def hold(sec, state, fn=None):
    for i in range(int(round(sec / DT))):
        sim.state = state
        if fn:
            fn(i * DT)
        snap()


def move(targets, vmax, amax, state, extra=None):
    start = {k: getattr(sim, k) for k in targets}
    dist = math.sqrt(sum((targets[k] - start[k]) ** 2 for k in targets))
    T, s = trapezoid(dist, vmax, amax)
    n = max(1, int(math.ceil(T / DT)))
    for i in range(1, n + 1):
        u = s(min(i * DT, T)) / dist if dist > 0 else 1.0
        for k in targets:
            setattr(sim, k, start[k] + (targets[k] - start[k]) * u)
        sim.state = state
        if extra:
            extra(i * DT, T)
        snap()


# plan
lane_len = tlp.lane_length(LANE_Y, -S_EST, D_PLAN)
X_START, X_END = TX - lane_len / 2, TX + lane_len / 2
Z_EST = S_EST + m.B_ATT


def idle_fn(t):
    if t >= 0.3:
        sim.cup = True
        sim.scale = CUP_G * min(1.0, (t - 0.3) / 0.25)


hold(1.1, "IDLE", idle_fn)
sim.pressed = True
sim.note = f"레인 y = {LANE_Y:+.0f}: C 이동 {lane_len:.0f} mm (가운데 레인은 {tlp.lane_length(0, -S_EST, D_PLAN):.0f} mm)"
hold(0.6, "FLAVOR_SELECT")
hold(0.4, "CHECK_CUP")
th0 = sim.th
move({"X": X_START, "Y": LANE_Y}, V_XY, A_XY, "XY_MOVE_TO_TUB",
     lambda t, T: setattr(sim, "th", th0 + (-30.0 - th0) * ease(t / 0.8)))
sim.th = -30.0
sim.note = ""
move({"Z": Z_EST + 10.0}, V_Z, A_Z, "Z_APPROACH")
i_low = int(np.argmin(np.abs(xs - (sim.X - m.R * 0.5))))
z_contact = S[i_low, J_LANE] + m.B_ATT
while sim.Z > z_contact:
    sim.Z = max(z_contact, sim.Z - 10.0 * DT)
    sim.state = "SURFACE_DETECT"
    snap()
sim.touch_t = sim.t
for i in range(3):
    sim.Fz = 3.0 * (i + 1) / 3
    sim.state = "SURFACE_DETECT"
    sim.note = f"접촉: Z0 = {z_contact:.1f} mm (표면 {z_contact - m.B_ATT:.1f}, 지도 대비 {z_contact - m.B_ATT - S_EST:+.1f} mm)"
    snap()
Z0 = sim.Z
sim.Fz = 0.0

# dive (30 deg path), force from removed volume per path length
v_dd = 0.0
d = 0.0
step = 40.0 * DT
while d < D_PLAN - 1e-6:
    ds = min(step, (D_PLAN - d) / math.sin(math.radians(30)))
    sim.X += ds * math.cos(math.radians(30))
    sim.Z -= ds * math.sin(math.radians(30))
    d = Z0 - sim.Z
    dv = cut(sim.X, sim.Y, sim.Z, sim.th)
    v_dd += dv
    sim.V = v_dd
    sim.Fx = U_KPA * 1e-3 * dv / ds
    sim.Fz = 0.5 * sim.Fx
    sim.fill = sim.V / V_TARGET
    sim.state = "SCOOP_DIVE"
    sim.note = f"깊이 d = {d:.1f} mm"
    snap()

# drag with depth adaptation on measured force
d_cmd = D_PLAN
while True:
    dx = 80.0 * DT
    sim.X += dx
    err = (F_TARGET - sim.Fx) / F_TARGET
    d_cmd = min(D_PLAN, max(0.0, d_cmd + DT * 30.0 * max(-1.0, min(1.0, 3.0 * err))))
    sim.Z = Z0 - d_cmd
    dv = cut(sim.X, sim.Y, sim.Z, sim.th)
    v_dd += dv
    sim.V = v_dd
    sim.Fx = U_KPA * 1e-3 * dv / dx
    sim.Fz = 0.5 * sim.Fx
    sim.fill = sim.V / V_TARGET
    sim.state = "SCOOP_DRAG"
    ad = " · 깊이 적응" if d_cmd < D_PLAN - 0.2 else ""
    sim.note = f"깊이 d = {d_cmd:.1f} mm · F_x = {sim.Fx:.0f} N{ad}"
    snap()
    cap_guess = math.pi * (m.R - (m.B_ATT - d_cmd)) ** 2 * (3 * m.R - (m.R - (m.B_ATT - d_cmd))) / 3.0
    if sim.V + CLOSE_CAPTURE * cap_guess >= V_TARGET or sim.X >= X_END:
        break
d_end = d_cmd

# close: rotation cuts the front cap; half of it is captured
v_close = 0.0
n = int(round(0.8 / DT))
for i in range(1, n + 1):
    sim.th = -30.0 + 120.0 * ease(i / n)
    th_prev = -30.0 + 120.0 * ease((i - 1) / n)
    dv = cut(sim.X, sim.Y, sim.Z, sim.th)
    v_close += dv
    sim.V = v_dd + CLOSE_CAPTURE * v_close
    d_arc = m.R * math.radians(max(1e-6, sim.th - th_prev))      # rim travel this step
    sim.Fx = U_KPA * 1e-3 * dv / d_arc                               # tangential cutting force at the rim
    sim.Fz = 0.5 * sim.Fx
    sim.fill = sim.V / V_TARGET
    sim.state = "SCOOP_CLOSE"
    sim.note = "공 닫기 (앞쪽 구면 캡 절단, 절반 포획 가정)"
    snap()
sim.ball, sim.fill, sim.Fx, sim.Fz = True, 0.0, 0.0, 0.0
sim.mass = RHO * sim.V / 1e3 * K_F
sim.ball_r = min(m.R - 1.5, (3 * sim.V / (4 * math.pi)) ** (1 / 3))
sim.note = f"포획 부피 {sim.V / 1e3:.0f} cm³ → 예측 {sim.mass:.0f} g"

move({"Z": m.Z_SAFE}, V_Z, A_Z, "Z_LIFT")
move({"X": m.CUP_X, "Y": 0.0}, V_XY, A_XY, "XY_MOVE_TO_CUP")
hold(0.2, "BAY_CHECK")
move({"Z": m.DISPENSE_Z}, 50.0, 500.0, "Z_DISPENSE")
n = int(round(0.5 / DT))
for i in range(1, n + 1):
    sim.th = 90.0 - 120.0 * ease(i / n)
    sim.state = "EJECT"
    snap()
hold(0.5, "EJECT", lambda t: setattr(sim, "th", -30.0 + 5.0 * math.sin(2 * math.pi * 10.0 * t)))
sim.th = -30.0
bx, bz = sim.X, sim.Z
sim.ball = False
sim.ball_free = (bx, 0.0, bz)
move({"X": m.CUP_X - 40.0}, 150.0, 1500.0, "EJECT")
vz = 0.0
while sim.ball_free[2] > -2.0:
    vz -= 9810.0 * DT
    sim.ball_free = (bx, 0.0, max(-2.0, sim.ball_free[2] + vz * DT))
    sim.state = "EJECT"
    snap()
sim.ball_free, sim.ball_in_cup = None, True
move({"Z": m.Z_SAFE}, 50.0, 500.0, "Z_RETRACT")
target = CUP_G + sim.mass
dev = (sim.mass - M_TARGET) / M_TARGET * 100
verdict = "허용범위(−5/+10 %) 안" if -5 <= dev <= 10 else ("부족 → 화면 경고·로그 (V1은 자동 보충 없음)" if dev < -5
                                                            else "과다 → 경고·로그")


def settle(t):
    sim.scale = target + (sim.scale - target) * math.exp(-DT / 0.12) + 1.5 * math.exp(-t / 0.25) * math.sin(18 * t)
    sim.note = f"계량 {sim.mass:.1f} g (목표 115 g, {dev:+.1f} %) · {verdict}"


hold(1.0, "PORTION_CHECK", settle)
sim.scale = target
if dev < -5:
    sim.note += (f"\n원인: 옆 레인 y = {LANE_Y:+.0f}은 통 벽 때문에 C 이동이 {lane_len:.0f} mm뿐 "
                 f"(가운데 {tlp.lane_length(0, -S_EST, D_PLAN):.0f} mm) + 힘 목표 110 N")
hold(2.2, "PORTION_CHECK")
T_CYCLE = next(f["t"] for f in frames if f["state"] == "PORTION_CHECK") + 1.0 - next(
    f["t"] for f in frames if f["state"] == "XY_MOVE_TO_TUB")
DESC["DONE"] = f"완료 — 기계 사이클 약 {T_CYCLE:.1f} s (가정 속도 기준). 다음 스쿱은 이 층의 다음 레인으로"


def take_cup(t):
    if t > 0.4:
        sim.cup, sim.ball_in_cup, sim.scale = False, False, 0.0


hold(1.4, "DONE", take_cup)

# ------------------------------------------------------------------ rendering
W, H, DPI = 12.8, 7.2, 100
fig = plt.figure(figsize=(W, H), dpi=DPI, facecolor="white")
ax_ban = fig.add_axes([0.0, 0.925, 1.0, 0.075])
ax_3d = fig.add_axes([0.0, 0.065, 0.53, 0.86])
ax_sec = fig.add_axes([0.54, 0.505, 0.27, 0.405])
ax_top = fig.add_axes([0.82, 0.505, 0.17, 0.405])
ax_sig = fig.add_axes([0.575, 0.265, 0.41, 0.17])
ax_txt = fig.add_axes([0.54, 0.07, 0.45, 0.14])
ax_strip = fig.add_axes([0.005, 0.005, 0.99, 0.05])
A = m3.Axo(24.0, 22.0)
TS = np.array([f["t"] for f in frames])
FX = np.array([f["Fx"] for f in frames])
FZ = np.array([f["Fz"] for f in frames])
VIEW = A.pts([(-260, -300, -330), (1450, -300, -330), (-260, 300, 1180), (1450, 300, 1180), (-260, 300, -330)])
RD = 2                                   # render the height map at 5 mm
xs_r, ys_r = xs[::RD], ys[::RD]


def render_map(Sm):
    n0, n1 = (Sm.shape[0] // RD) * RD, (Sm.shape[1] // RD) * RD
    blk = Sm[:n0, :n1].reshape(n0 // RD, RD, n1 // RD, RD)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)       # all-NaN blocks outside the tub
        return np.nanmin(np.nanmin(blk, axis=3), axis=1)


def draw_frame(k):
    f = frames[k]
    for a in (ax_ban, ax_3d, ax_sec, ax_top, ax_sig, ax_txt, ax_strip):
        a.cla()
    # banner
    ax_ban.axis("off")
    ax_ban.set_xlim(0, 1)
    ax_ban.set_ylim(0, 1)
    ax_ban.add_patch(Rectangle((0, 0), 1, 1, fc="#1F2A36"))
    ax_ban.text(0.012, 0.66, "Cartesian 자동 맛 선택 스쿠퍼 — X·Y·Z·θ 4축 작동 (V1, 시뮬레이션)", color="white",
                fontsize=13, fontweight="bold", va="center")
    ax_ban.text(0.012, 0.24, f"{f['state']}  ·  {DESC.get(f['state'], '')}", color="#FFD37A", fontsize=10.5,
                va="center")
    ax_ban.text(0.988, 0.66, f"t = {f['t']:5.1f} s   실시간 1×", color="white", fontsize=11, ha="right",
                va="center", family=MONO)
    act = ACTIVE.get(f["state"], ())
    x0 = 0.63
    for key in ("X", "Y", "Z", "θ"):
        on = key in act
        col = m3.AXIS_COL[key]
        ax_ban.add_patch(FancyBboxPatch((x0, 0.1), 0.028, 0.3, boxstyle="round,pad=0.004",
                                        fc=col if on else "#1F2A36", ec=col, lw=1.2))
        ax_ban.text(x0 + 0.014, 0.25, key, color="white" if on else col, fontsize=10, fontweight="bold",
                    ha="center", va="center")
        x0 += 0.034
    ok = f["Z"] >= m.Z_SAFE - 0.01
    ax_ban.text(0.988, 0.24, "Z ≥ Z_SAFE → XY 허용" if ok else "Z < Z_SAFE → XY 잠금", color="#7CD992" if ok else "#FFB36B",
                fontsize=10, ha="right", va="center")

    # 3D
    a = ax_3d
    a.set_aspect("equal")
    a.axis("off")
    Sr = render_map(f["S"])
    hm = {FID: (xs_r[:Sr.shape[0]], ys_r[:Sr.shape[1]], Sr, S_TRUE)}
    cup_balls = [(m.CUP_X, 0.0, -2.0, f["ball_r"] * 0.9, ICE)] if (f["ball_in_cup"] and f["cup"]) else ()
    fb = None
    if f["ball_free"] is not None:
        bx, by, bz = f["ball_free"]
        fb = (bx, by, bz, f["ball_r"] * 0.9)
    m3.draw_machine(a, A, f["X"], f["Y"], f["Z"], f["th"], fill=f["fill"],
                    ball_r=f["ball_r"] * 0.9 if f["ball"] else None, heightmaps=hm, cup=f["cup"],
                    cup_balls=cup_balls, active=act if act else ("__none__",), free_ball=fb, ice=ICE)
    a.set_xlim(VIEW[:, 0].min(), VIEW[:, 0].max())
    a.set_ylim(VIEW[:, 1].min(), VIEW[:, 1].max())
    q = A.proj((m.CUP_X, -120, -150))
    a.text(q[0], q[1], f"저울 {f['scale']:.1f} g", fontsize=9, color=m.C_SENSOR, ha="center", family=MONO,
           zorder=20, bbox=dict(fc="white", ec="none", alpha=0.85, pad=1))
    for i, (fid, name, *_r) in enumerate(m.FLAVORS):
        on = f["pressed"] and fid == FID
        q = A.proj((-150, -302, 300 - i * 55))
        a.text(q[0] + 30, q[1], name, fontsize=7.5, ha="left", va="center", zorder=20,
               color="white" if on else m.C_TEXT,
               bbox=dict(fc=m.C_X if on else "#EEF2F6", ec="#8795a3", lw=0.5, boxstyle="round,pad=0.25"))
    m3.draw_triad(a, A, (VIEW[:, 0].min() + 70, VIEW[:, 1].min() + 60), L=110, fs=8)

    # section at the lane
    s_ = ax_sec
    j = J_LANE
    prof = f["S"][:, j]
    w, hgt = 300.0, 300.0 * (0.405 * H) / (0.27 * W)
    cx, cz = f["X"], f["Z"] - 10
    if f["state"] in ("EJECT", "Z_DISPENSE", "Z_RETRACT", "PORTION_CHECK", "BAY_CHECK", "DONE", "XY_MOVE_TO_CUP") \
            and f["X"] > 800:
        cx, cz = m.CUP_X - 10, 10
    s_.set_xlim(cx - w / 2, cx + w / 2)
    s_.set_ylim(cz - hgt / 2, cz + hgt / 2)
    s_.set_aspect("equal")
    s_.set_xticks([])
    s_.set_yticks([])
    for sp in s_.spines.values():
        sp.set_color("#9aa6b2")
    y0 = ys[j]
    zz = np.linspace(0, -m.TUB_DEPTH, 30)
    wall = np.sqrt(np.clip(np.array([m.tub_half_width(z) for z in zz]) ** 2 - y0 ** 2, 0, None))
    s_.add_patch(Polygon(np.vstack([np.column_stack([TX - wall, zz]), np.column_stack([TX + wall, zz])[::-1]]),
                         closed=True, fc="#F4F1EA", ec="#9A8F7A", lw=0.9))
    okp = np.isfinite(prof)
    pts = np.column_stack([xs[okp], prof[okp]])
    if len(pts):
        s_.add_patch(Polygon(np.vstack([[pts[0][0], -m.TUB_DEPTH], pts, [pts[-1][0], -m.TUB_DEPTH]]),
                             closed=True, fc=ICE, ec="#b89a50", lw=0.6))
    s_.add_patch(Rectangle((TX - 170, -20), 170 - wall[0], 20, fc=m.C_DECK, ec="#8E9AA5"))
    s_.add_patch(Rectangle((TX + wall[0], -20), 170 - wall[0], 20, fc=m.C_DECK, ec="#8E9AA5"))
    # cup station section
    s_.add_patch(Rectangle((m.CUP_X - 95, -140), 190, 140, fc="#DCE3E8", ec="#8E9AA5"))
    if f["cup"]:
        s_.add_patch(Polygon([(m.CUP_X - 40, 0), (m.CUP_X + 40, 0), (m.CUP_X + 30, -75), (m.CUP_X - 30, -75)],
                             closed=True, fc=m.C_CUP, ec="#9aa3ab"))
        if f["ball_in_cup"]:
            s_.add_patch(Circle((m.CUP_X, -2), f["ball_r"] * 0.9, fc=ICE, ec="#8a7a55", lw=0.6))
    if f["ball_free"] is not None:
        s_.add_patch(Circle((f["ball_free"][0], f["ball_free"][2]), f["ball_r"] * 0.9, fc=ICE, ec="#8a7a55", lw=0.6,
                            zorder=12))
    m.draw_head(s_, f["X"], f["Z"], f["th"], fill=f["fill"], ball=False, ice=ICE, show_col=True)
    if f["ball"]:
        s_.add_patch(Circle((f["X"], f["Z"]), f["ball_r"] * 0.9, fc=ICE, ec="#8a7a55", lw=0.6, zorder=12))
    s_.axhline(m.Z_SAFE, color=m.C_SAFE, lw=0.9, ls=(0, (6, 4)))
    s_.text(0.02, 0.975, f"단면 (레인 y = {LANE_Y:+.0f})", transform=s_.transAxes, fontsize=9, va="top", zorder=30,
            bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.2))
    s_.text(0.98, 0.975, f"θ = {f['th']:+.0f}°", transform=s_.transAxes, fontsize=9.5, va="top", ha="right",
            family=MONO, zorder=30, bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.2))

    # top view of TUB1 height map
    t_ = ax_top
    dep = S_TRUE - f["S"]
    img = np.where(np.isfinite(dep), dep, np.nan).T
    t_.imshow(img, origin="lower", extent=(xs[0], xs[-1], ys[0], ys[-1]), cmap="YlOrBr", vmin=0, vmax=40,
              interpolation="nearest")
    t_.add_patch(Circle((TX, 0), 115, fc="none", ec="#6f6656", lw=0.9))
    for yl in (-40, 0, 40):
        t_.plot([TX - 70, TX + 70], [yl, yl], color=m.C_Y, lw=0.8, ls=":")
        t_.text(TX - 112, yl, f"{yl:+d}" if yl else "0", fontsize=7, color=m.C_Y, va="center")
    if abs(f["X"] - TX) < 150:
        t_.add_patch(Circle((f["X"], f["Y"]), m.R, fc="none", ec=m.C_TH, lw=1.3))
        t_.plot(f["X"], f["Y"], "+", color=m.C_TH, ms=7)
    t_.set_xlim(TX - 120, TX + 120)
    t_.set_ylim(-120, 120)
    t_.set_aspect("equal")
    t_.set_xticks([])
    t_.set_yticks([])
    t_.set_title("TUB1 평면 (깊이 음영, Y 레인)", fontsize=8.5, loc="left", pad=2)
    t_.annotate("", xy=(TX + 110, -105), xytext=(TX + 60, -105), arrowprops=dict(arrowstyle="-|>", color=m.C_X, lw=1.2))
    t_.text(TX + 58, -105, "X", fontsize=8, color=m.C_X, ha="right", va="center", fontweight="bold")
    t_.annotate("", xy=(TX + 110, -55), xytext=(TX + 110, -105), arrowprops=dict(arrowstyle="-|>", color=m.C_Y, lw=1.2))
    t_.text(TX + 110, -50, "Y", fontsize=8, color=m.C_Y, ha="center", va="bottom", fontweight="bold")

    # signals
    g = ax_sig
    g.set_xlim(TS[0], TS[-1])
    g.set_ylim(0, 175)
    g.plot(TS[:k + 1], FX[:k + 1], color=m.C_X, lw=1.4, label="F_x (X 볼너트 로드셀)")
    g.plot(TS[:k + 1], FZ[:k + 1], color=m.C_Z, lw=1.2, label="F_z (Z 볼너트 로드셀)")
    g.axhline(F_TARGET, color="#E08A00", lw=0.8, ls="--")
    g.axhline(F_STOP, color=m.C_SENSOR, lw=0.8, ls="--")
    g.text(TS[-1], F_TARGET + 3, "목표 110 N ", ha="right", fontsize=7, color="#b36b00")
    g.text(TS[-1], F_STOP + 3, "정지 160 N ", ha="right", fontsize=7, color=m.C_SENSOR)
    g.axvline(f["t"], color="#333", lw=0.6)
    g.tick_params(labelsize=7)
    g.legend(loc="upper left", fontsize=7, frameon=False)
    g.set_title("힘 신호 [N] vs 시간 [s] (u = 90 kPa 가정)", fontsize=8.5, loc="left", pad=2)

    # readouts + note
    x_ = ax_txt
    x_.axis("off")
    x_.set_xlim(0, 1)
    x_.set_ylim(0, 1)
    rows = [("X", f"{f['X']:7.1f} mm", m.C_X), ("Y", f"{f['Y']:7.1f} mm", m.C_Y), ("Z (C)", f"{f['Z']:7.1f} mm", m.C_Z),
            ("θ", f"{f['th']:+7.1f} °", m.C_TH), ("부피", f"{f['V'] / 1e3:5.0f} / {V_TARGET / 1e3:.0f} cm³", m.C_TEXT),
            ("저울", f"{f['scale']:6.1f} g", m.C_TEXT)]
    for i, (k_, v_, col) in enumerate(rows):
        cxp = 0.0 + (i % 3) * 0.335
        cyp = 0.9 - (i // 3) * 0.3
        x_.text(cxp, cyp, k_, fontsize=9, color=col, fontweight="bold", va="center")
        x_.text(cxp + 0.3, cyp, v_, fontsize=9, color=m.C_TEXT, va="center", ha="right", family=MONO)
    if f["note"]:
        x_.text(0.0, 0.2, f["note"], fontsize=8.6, va="center", color="#243447",
                bbox=dict(fc="#FFF6DA", ec="#E0C36A", lw=0.6, pad=2.5))

    # strip
    st = STRIP_MAP.get(f["state"], f["state"])
    ax_strip.axis("off")
    ax_strip.set_xlim(0, len(STATES))
    ax_strip.set_ylim(0, 1)
    for i, (code, label) in enumerate(STATES):
        on = code == st
        ax_strip.add_patch(FancyBboxPatch((i + 0.05, 0.12), 0.9, 0.76, boxstyle="round,pad=0,rounding_size=0.12",
                                          fc="#2F6FB0" if on else "#EEF2F6", ec="#b8c4cf", lw=0.6))
        ax_strip.text(i + 0.5, 0.5, label, ha="center", va="center", fontsize=8.5,
                      color="white" if on else "#51606e", fontweight="bold" if on else "normal")


if __name__ == "__main__":
    print(f"frames {len(frames)}  {len(frames) / FPS:.1f} s  cycle ~{T_CYCLE:.1f} s  lane {lane_len:.0f} mm  "
          f"d_end {d_end:.1f}  V {frames[-1]['V'] / 1e3:.0f} cm3  mass {sim.mass:.1f} g ({dev:+.1f} %)")
    if "--preview" in sys.argv:
        for k in [int(v) for v in sys.argv[sys.argv.index("--preview") + 1].split(",")]:
            draw_frame(k)
            fig.savefig(f"/tmp/claude-0/p3_{k}.png", dpi=DPI)
        raise SystemExit
    writer = FFMpegWriter(fps=FPS, bitrate=2800, codec="libx264",
                          extra_args=["-pix_fmt", "yuv420p", "-preset", "slow", "-crf", "22"])
    with writer.saving(fig, OUT_MP4, dpi=DPI):
        for k in range(len(frames)):
            draw_frame(k)
            writer.grab_frame()
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-i", OUT_MP4, "-vf",
                    "fps=12,scale=800:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=160[p];"
                    "[b][p]paletteuse=dither=bayer:bayer_scale=4", OUT_GIF], check=True)
    print("wrote", OUT_MP4, os.path.getsize(OUT_MP4) // 1024, "KB;", OUT_GIF, os.path.getsize(OUT_GIF) // 1024, "KB")
