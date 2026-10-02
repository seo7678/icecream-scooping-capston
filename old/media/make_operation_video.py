"""Operation video: one automatic single scoop on the V1 Cartesian machine.

Run:  python3 media/make_operation_video.py
Out:  media/operation.mp4 (1280x720, 30 fps, real time) and media/operation.gif

The motion is simulated from the design numbers (axis speeds, touch-off,
30 deg dive, drag with depth adaptation, 0.8 s close, cup station, scale).
Cutting force uses u = 90 kPa (ASSUMPTION A02); nothing here is measured.
The eject step is drawn schematically: the eject method is still open (V1a test).

Requires: numpy, matplotlib, koreanize-matplotlib, imageio-ffmpeg.
"""

import math
import os
import subprocess

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Circle, Polygon, Rectangle, FancyBboxPatch

import machine as m
from machine import cm

HERE = os.path.dirname(os.path.abspath(__file__))
MONO = ["DejaVu Sans Mono", "NanumGothic"]
OUT_MP4 = os.path.join(HERE, "operation.mp4")
OUT_GIF = os.path.join(HERE, "operation.gif")
FPS = 30
DT = 1.0 / FPS

try:
    import imageio_ffmpeg
    FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
    plt.rcParams["animation.ffmpeg_path"] = FFMPEG
except ImportError:  # pragma: no cover
    FFMPEG = "ffmpeg"

# ------------------------------------------------------------------ order + assumptions
FID, FNAME, TX, S_EST, ICE = m.FLAVORS[0]      # vanilla, TUB1 (340, 0), map estimate -62
S_TRUE = S_EST - 1.5                           # real surface 1.5 mm lower than the map
U_KPA = 90.0                                   # A02 medium (ASSUMPTION)
F_TARGET, F_STOP = 110.0, 160.0                # A37
RHO = 0.65                                     # g/cm^3 (A07)
M_TARGET = 115.0
K_F = 1.00
V_TARGET = M_TARGET / (RHO * K_F) * 1e3        # mm^3
CLOSE_CAPTURE = 0.5                            # tub_lane_planner.py
CUP_G = 5.0
V_XY, A_XY = 250.0, 1000.0
V_Z, A_Z = 150.0, 1000.0


def area(d):
    return cm.swept_area(cm.ATTACK_DEG, max(0.0, d))


def cap_volume(d):
    h = m.B_ATT - d
    dc = m.R - h
    return math.pi * dc ** 2 * (3 * m.R - dc) / 3.0


def lane_half(depth_below_rim_of_c):
    r_t = cm.tub_diameter_at(depth_below_rim_of_c) / 2.0
    return r_t - m.R - cm.WALL_MARGIN


# plan (same rules as calc/tub_lane_planner.py)
D_PLAN = 26.0                                  # <= d_max 28 (vanilla); adaptation may reduce it
Z_C_DRAG = S_EST - D_PLAN + m.B_ATT
HALF = lane_half(-(Z_C_DRAG - m.R))
X_START, X_END = TX - HALF, TX + HALF
Z_EST = S_EST + m.B_ATT                        # C height at expected contact


def trapezoid(dist, vmax, amax):
    """Return (duration, s(t)) for a rest-to-rest move of length |dist|."""
    L = abs(dist)
    sign = 1.0 if dist >= 0 else -1.0
    if L < 1e-9:
        return 0.0, lambda t: 0.0
    ta = vmax / amax
    if L < vmax * ta:                          # triangular
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
            tr = T - t
            return sign * (L - 0.5 * amax * tr * tr)
        return sign * (0.5 * amax * ta * ta + vmax * (t - ta))
    return T, s


def ease(u):
    u = min(max(u, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * u)


# ------------------------------------------------------------------ simulation
STATES = [  # code, short Korean label for the strip
    ("FLAVOR_SELECT", "맛 선택"), ("XY_MOVE_TO_TUB", "XY 이동"), ("Z_APPROACH", "Z 접근"),
    ("SURFACE_DETECT", "표면 검출"), ("SCOOP_DIVE", "Dive"), ("SCOOP_DRAG", "Drag"),
    ("SCOOP_CLOSE", "Close"), ("Z_LIFT", "Z 상승"), ("XY_MOVE_TO_CUP", "컵 이동"),
    ("EJECT", "배출"), ("PORTION_CHECK", "계량"),
]
DESC = {
    "IDLE": "대기 — 작업자가 컵을 매립 웰에 놓는다",
    "FLAVOR_SELECT": "맛 버튼 → 조회표: TUB1 (340, 0) · 레인 y = 0 · 표면 추정 -62 mm",
    "CHECK_CUP": "컵 확인: 저울 5.0 g > 3 g, 컵 베이 빔 비어 있음",
    "XY_MOVE_TO_TUB": "Z ≥ Z_SAFE에서만 XY 이동 · 이동하면서 스쿱을 공격 자세(θ -30°)로",
    "Z_APPROACH": "Z 빠른 하강: 표면 추정보다 10 mm 위까지 (150 mm/s)",
    "SURFACE_DETECT": "10 mm/s로 접근 · Z 볼너트 로드셀 ΔF ≥ 3 N → 표면 기준 Z0",
    "SCOOP_DIVE": "X·Z 동시 보간 30° 경사로 절입 (θ -30° 유지)",
    "SCOOP_DRAG": "X 드래그 80 mm/s · F_x > 110 N이면 Z를 올려 깊이 축소 · 부피 적분",
    "SCOOP_CLOSE": "X 정지, θ -30° → +90° 로 닫아 공 완성",
    "Z_LIFT": "Z 상승 → Z_SAFE (그 전에는 XY 잠금)",
    "XY_MOVE_TO_CUP": "컵 스테이션으로 이동 (Z ≥ Z_SAFE)",
    "BAY_CHECK": "컵 베이 빔 비어 있음 확인 후 하강",
    "EJECT": "배출: θ -30° + 흔들기 + X 후퇴 (방식은 V1a 시험으로 확정 — 개략 표시)",
    "PORTION_CHECK": "저울 안정 1 s → 질량 판정 · 맛별 보정계수 k_f 갱신 · 절삭 이력 지도 갱신",
    "DONE": "완료 — 작업자가 컵을 가져간다 (기계 사이클 약 15 s)",
}

xs_tub = np.arange(TX - m.TUB_D_TOP / 2 + 1, TX + m.TUB_D_TOP / 2 - 1 + 0.5, 1.0)
frames = []


def snapshot(**kw):
    frames.append(kw)


class Sim:
    def __init__(self):
        self.t = 0.0
        self.X, self.Y, self.Z, self.th = 0.0, 0.0, Z_TOP_START, 90.0
        self.fill = 0.0
        self.ball = False
        self.ball_free = None          # (x, z) of a released ball
        self.ball_in_cup = False
        self.cup = False
        self.scale = 0.0
        self.Fx = self.Fz = 0.0
        self.V = 0.0
        self.state = "IDLE"
        self.flavor_pressed = False
        self.profile = np.array([S_TRUE if abs(x - TX) <= m.tub_half_width(S_TRUE) else np.nan for x in xs_tub])
        self.cut_band = None
        self.trail = []
        self.touch_t = None
        self.note = ""
        self.mass = None

    def cut(self):
        if self.Z - m.R > S_TRUE + 2:
            return
        env = m.lower_envelope(m.bowl_polygon(self.X, self.Z, self.th), xs_tub)
        ok = ~np.isnan(env)
        self.profile[ok] = np.fmin(self.profile[ok], env[ok])

    def step(self):
        noise = np.random.normal(0, 0.4)
        self.trail.append((self.X, self.Z))
        self.trail = self.trail[-240:]
        snapshot(t=self.t, X=self.X, Y=self.Y, Z=self.Z, th=self.th, fill=self.fill, ball=self.ball,
                 ball_free=self.ball_free, ball_in_cup=self.ball_in_cup, cup=self.cup, scale=self.scale,
                 Fx=max(0.0, self.Fx + noise), Fz=max(0.0, self.Fz + 0.5 * noise), V=self.V, state=self.state,
                 pressed=self.flavor_pressed, profile=self.profile.copy(), cut_band=self.cut_band,
                 trail=list(self.trail), touch_t=self.touch_t, note=self.note, mass=self.mass)
        self.t += DT


Z_TOP_START = m.Z_TOP
sim = Sim()
np.random.seed(3)


def hold(seconds, state=None, fn=None):
    n = int(round(seconds / DT))
    for i in range(n):
        if state:
            sim.state = state
        if fn:
            fn(i * DT)
        sim.step()


def move(axis_targets, vmax, amax, state, extra=None):
    """Synchronised straight-line move of several axes (dict axis -> target)."""
    start = {a: getattr(sim, a) for a in axis_targets}
    dist = math.sqrt(sum((axis_targets[a] - start[a]) ** 2 for a in axis_targets))
    T, s = trapezoid(dist, vmax, amax)
    n = max(1, int(math.ceil(T / DT)))
    for i in range(1, n + 1):
        u = s(min(i * DT, T)) / dist if dist > 0 else 1.0
        for a in axis_targets:
            setattr(sim, a, start[a] + (axis_targets[a] - start[a]) * u)
        sim.state = state
        if extra:
            extra(i * DT, T)
        sim.step()


# 1) idle: cup placed, flavour pressed
def idle_fn(t):
    if t >= 0.3:
        sim.cup = True
        sim.scale = CUP_G * min(1.0, (t - 0.3) / 0.25)


hold(1.1, "IDLE", idle_fn)
sim.flavor_pressed = True
hold(0.5, "FLAVOR_SELECT")
hold(0.4, "CHECK_CUP")

# 2) XY to lane start, pitch to attack pose on the way
th0 = sim.th


def pitch_on_the_way(t, T):
    sim.th = th0 + (-30.0 - th0) * ease(t / 0.8)


move({"X": X_START}, V_XY, A_XY, "XY_MOVE_TO_TUB", pitch_on_the_way)
sim.th = -30.0

# 3) Z fast approach, then probe at 10 mm/s
move({"Z": Z_EST + 10.0}, V_Z, A_Z, "Z_APPROACH")
z_contact = S_TRUE + m.B_ATT
while sim.Z > z_contact:
    sim.Z = max(z_contact, sim.Z - 10.0 * DT)
    sim.state = "SURFACE_DETECT"
    sim.step()
sim.touch_t = sim.t
for i in range(3):                              # 3 N reached, held 20 ms
    sim.Fz = 3.0 * (i + 1) / 3
    sim.state = "SURFACE_DETECT"
    sim.note = f"접촉: Z0 = {z_contact:.1f} mm (표면 {S_TRUE:.1f}, 지도 대비 {S_TRUE - S_EST:+.1f} mm)"
    sim.step()
Z0 = sim.Z
sim.Fz = 0.0

# 4) dive along a 30 deg path to D_PLAN, integrating the cut volume
d = 0.0
dive_len = D_PLAN / math.sin(math.radians(30.0))
v_dive = 40.0
steps = int(math.ceil(dive_len / (v_dive * DT)))
for i in range(steps):
    ds = min(v_dive * DT, dive_len - i * v_dive * DT)
    dx, dz = ds * math.cos(math.radians(30.0)), -ds * math.sin(math.radians(30.0))
    sim.X += dx
    sim.Z += dz
    d = Z0 - sim.Z
    sim.V += area(d) * dx
    sim.Fx = U_KPA * 1e-3 * area(d)
    sim.Fz = 0.5 * sim.Fx
    sim.fill = sim.V / V_TARGET
    sim.state = "SCOOP_DIVE"
    sim.note = f"깊이 d = {d:.1f} mm"
    sim.cut()
    sim.step()

# 5) drag with depth adaptation until V (+ closing cap share) reaches the target or lane end
tau = 0.12
while True:
    d_force = D_PLAN
    while area(d_force) * U_KPA * 1e-3 > F_TARGET and d_force > 5:
        d_force -= 0.05
    d_cmd = min(D_PLAN, d_force)
    d += (d_cmd - d) * min(1.0, DT / tau)
    sim.Z = Z0 - d
    dx = 80.0 * DT
    sim.X += dx
    sim.V += area(d) * dx
    sim.Fx = U_KPA * 1e-3 * area(d)
    sim.Fz = 0.5 * sim.Fx
    sim.fill = sim.V / V_TARGET
    sim.state = "SCOOP_DRAG"
    adapt = " · 깊이 적응 중" if d < D_PLAN - 0.2 else ""
    sim.note = f"깊이 d = {d:.1f} mm · F_x = {sim.Fx:.0f} N{adapt}"
    sim.cut()
    sim.step()
    if sim.V + CLOSE_CAPTURE * cap_volume(d) >= V_TARGET or sim.X >= X_END:
        break
d_end = d

# 6) close
v_before = sim.V
n = int(round(0.8 / DT))
for i in range(1, n + 1):
    u = ease(i / n)
    sim.th = -30.0 + 120.0 * u
    sim.V = v_before + CLOSE_CAPTURE * cap_volume(d_end) * u
    sim.fill = sim.V / V_TARGET
    sim.Fx = U_KPA * 1e-3 * area(d_end) * (1 - u) * 0.6
    sim.Fz = 0.5 * sim.Fx
    sim.state = "SCOOP_CLOSE"
    sim.note = "공 닫기 (구면 캡 절단)"
    sim.cut()
    sim.step()
sim.ball, sim.fill, sim.Fx, sim.Fz = True, 0.0, 0.0, 0.0
sim.mass = RHO * sim.V / 1e3 * K_F
sim.note = f"적분 부피 {sim.V / 1e3:.0f} cm³ → 예측 {sim.mass:.0f} g"

# 7) lift to Z_SAFE, update the cut-history map
move({"Z": m.Z_SAFE}, V_Z, A_Z, "Z_LIFT")
sim.cut_band = (X_START - 17.5, sim.X + m.R * 0.9)

# 8) to the cup station
move({"X": m.CUP_X}, V_XY, A_XY, "XY_MOVE_TO_CUP")
hold(0.2, "BAY_CHECK")
move({"Z": m.DISPENSE_Z}, 50.0, 500.0, "Z_DISPENSE")

# 9) eject (schematic): tilt, shake, retract; the ball stays and drops into the cup
n = int(round(0.5 / DT))
for i in range(1, n + 1):
    sim.th = 90.0 - 120.0 * ease(i / n)
    sim.state = "EJECT"
    sim.step()


def shake(t):
    sim.th = -30.0 + 5.0 * math.sin(2 * math.pi * 10.0 * t)


hold(0.5, "EJECT", shake)
sim.th = -30.0
ball_x, ball_z = sim.X, sim.Z
sim.ball = False
sim.ball_free = (ball_x, ball_z)
move({"X": m.CUP_X - 40.0}, 150.0, 1500.0, "EJECT")
z_rest = -2.0
vz = 0.0
while sim.ball_free[1] > z_rest:
    vz -= 9810.0 * DT
    bz = max(z_rest, sim.ball_free[1] + vz * DT)
    sim.ball_free = (ball_x, bz)
    sim.state = "EJECT"
    sim.step()
sim.ball_free = None
sim.ball_in_cup = True

# 10) retract and weigh
move({"Z": m.Z_SAFE}, 50.0, 500.0, "Z_RETRACT")
target = CUP_G + sim.mass
k_new = K_F + 0.2 * ((sim.mass / (RHO * sim.V / 1e3)) - K_F)
dev = (sim.mass - M_TARGET) / M_TARGET * 100


if -5.0 <= dev <= 10.0:
    VERDICT = "허용범위(-5/+10 %) 안 → 완료, 로그"
elif dev < -5.0:
    VERDICT = "부족 → 화면 경고·로그 (V1은 자동 보충 없음)"
else:
    VERDICT = "과다 → 경고·로그"


def settle(t):
    sim.scale = target + (sim.scale - target) * math.exp(-DT / 0.12) + 1.5 * math.exp(-t / 0.25) * math.sin(18 * t)
    sim.note = f"계량 {sim.mass:.1f} g (목표 115 g, {dev:+.1f} %) · {VERDICT}"


hold(1.0, "PORTION_CHECK", settle)
sim.scale = target
if d_end < D_PLAN - 0.2:
    sim.note += f"\n원인: 힘 목표 110 N 때문에 깊이 {d_end:.1f} mm로 제한 (가정 u = 90 kPa) → E0 실측 후 재설정"
hold(1.6, "PORTION_CHECK")
hold(0.4, "DONE")


def take_cup(t):
    if t > 0.4:
        sim.cup = False
        sim.ball_in_cup = False
        sim.scale = 0.0


hold(1.4, "DONE", take_cup)
T_CYCLE = next(f["t"] for f in frames if f["state"] == "PORTION_CHECK") + 1.0 - next(
    f["t"] for f in frames if f["state"] == "XY_MOVE_TO_TUB")
DESC["DONE"] = f"완료 — 작업자가 컵을 가져간다 (기계 사이클 약 {T_CYCLE:.1f} s, 가정 속도 기준)"

# ------------------------------------------------------------------ rendering
W, H, DPI = 12.8, 7.2, 100
fig = plt.figure(figsize=(W, H), dpi=DPI, facecolor="white")
ax_ban = fig.add_axes([0.0, 0.925, 1.0, 0.075])
ax_main = fig.add_axes([0.0, 0.065, 0.57, 0.86])
ax_zoom = fig.add_axes([0.585, 0.475, 0.405, 0.435])
ax_sig = fig.add_axes([0.62, 0.27, 0.365, 0.165])
ax_top = fig.add_axes([0.585, 0.07, 0.2, 0.15])
ax_txt = fig.add_axes([0.795, 0.07, 0.2, 0.15])
ax_strip = fig.add_axes([0.005, 0.005, 0.99, 0.05])

TS = np.array([f["t"] for f in frames])
FX = np.array([f["Fx"] for f in frames])
FZ = np.array([f["Fz"] for f in frames])
STRIP_MAP = {"IDLE": "FLAVOR_SELECT", "CHECK_CUP": "FLAVOR_SELECT", "BAY_CHECK": "XY_MOVE_TO_CUP",
             "Z_DISPENSE": "EJECT", "Z_RETRACT": "EJECT", "DONE": "PORTION_CHECK"}
XY_ALLOWED = {"IDLE", "FLAVOR_SELECT", "CHECK_CUP", "XY_MOVE_TO_TUB", "XY_MOVE_TO_CUP", "BAY_CHECK", "DONE",
              "PORTION_CHECK", "Z_LIFT", "Z_APPROACH"}
t0_scoop = next(f["t"] for f in frames if f["state"] == "XY_MOVE_TO_TUB")


def draw_frame(k):
    f = frames[k]
    for a in (ax_ban, ax_main, ax_zoom, ax_sig, ax_top, ax_txt, ax_strip):
        a.cla()

    # ---- banner
    ax_ban.axis("off")
    ax_ban.set_xlim(0, 1)
    ax_ban.set_ylim(0, 1)
    ax_ban.add_patch(Rectangle((0, 0), 1, 1, fc="#1F2A36"))
    ax_ban.text(0.012, 0.66, "Cartesian 자동 맛 선택 스쿠퍼 — 1회 작동 (V1, 시뮬레이션)", color="white",
                fontsize=13, fontweight="bold", va="center")
    ax_ban.text(0.012, 0.24, f"{f['state']}  ·  {DESC.get(f['state'], '')}", color="#FFD37A", fontsize=10.5,
                va="center")
    ax_ban.text(0.988, 0.66, f"t = {f['t']:5.1f} s   실시간 1×", color="white", fontsize=11, ha="right",
                va="center", family=MONO)
    xy_ok = f["Z"] >= m.Z_SAFE - 0.01
    if xy_ok:
        lab, col = "Z ≥ Z_SAFE → XY 이동 허용", "#7CD992"
    elif f["state"] in ("SCOOP_DIVE", "SCOOP_DRAG"):
        lab, col = "Z < Z_SAFE → XY 잠금 (예외: 현재 통 안 드래그)", "#FFB36B"
    elif f["state"] in ("EJECT", "Z_DISPENSE", "Z_RETRACT"):
        lab, col = "Z < Z_SAFE → XY 잠금 (예외: 컵 베이 안 배출)", "#FFB36B"
    else:
        lab, col = "Z < Z_SAFE → XY 이동 금지", "#FF8A8A"
    ax_ban.text(0.988, 0.24, lab, color=col, fontsize=10, ha="right", va="center")

    # ---- main front elevation
    a = ax_main
    a.set_xlim(-330, 1470)
    a.set_ylim(-305, 1221)
    a.set_aspect("equal")
    a.axis("off")
    m.draw_frame_static(a, tub_profiles={FID: (xs_tub, np.nan_to_num(f["profile"], nan=S_TRUE))},
                        show_cup=f["cup"], labels=True)
    a.axhline(m.Z_SAFE, xmin=0.02, xmax=0.98, color=m.C_SAFE, lw=1.0, ls=(0, (6, 4)), zorder=0.8)
    a.text(-300, m.Z_SAFE + 12, "Z_SAFE (C 높이 60 mm)", fontsize=7.5, color="#9a7a00")
    tr = np.array(f["trail"])
    if len(tr) > 2:
        a.plot(tr[:, 0], tr[:, 1], color=m.C_MOVE, lw=0.9, alpha=0.45, zorder=9)
    m.draw_carriage(a, f["X"], f["Z"], f["th"], fill=f["fill"], ball=f["ball"], ice=ICE)
    if f["ball_free"] is not None:
        a.add_patch(Circle(f["ball_free"], m.R - 1.5, fc=ICE, ec="#8a7a55", lw=0.6, zorder=12))
    if f["ball_in_cup"] and f["cup"]:
        a.add_patch(Circle((m.CUP_X, -2), m.R - 1.5, fc=ICE, ec="#8a7a55", lw=0.6, zorder=3.4))
    a.text(m.CUP_X, -160, f"저울 {f['scale']:.1f} g", ha="center", va="top", fontsize=8.5, color=m.C_SENSOR,
           family=MONO)
    a.text(m.CUP_X, -190, "컵 스테이션", ha="center", va="top", fontsize=8, color=m.C_TEXT)
    a.text(m.RINSE_X, -175, "헹굼", ha="center", va="top", fontsize=8, color=m.C_TEXT)
    a.text(1340, 1000, "X 빔 · 볼스크류\n(드래그 방향 = 빔 축)", ha="center", va="bottom", fontsize=7.5,
           color=m.C_STEEL_D, zorder=30, bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.5))
    # HMI
    for i, (fid, name, *_rest) in enumerate(m.FLAVORS):
        on = f["pressed"] and fid == FID
        a.add_patch(FancyBboxPatch((-300 + i * 95, 210), 82, 44, boxstyle="round,pad=2",
                                   fc="#2F6FB0" if on else "#E8EDF2", ec="#6d7d8c", lw=0.7, zorder=8))
        a.text(-300 + i * 95 + 41, 232, name, ha="center", va="center", fontsize=7,
               color="white" if on else m.C_TEXT, zorder=9)
    a.text(-300, 270, "HMI", fontsize=7.5, color=m.C_TEXT)

    # ---- zoom
    z = ax_zoom
    cx = min(max(f["X"], -150), 1400)
    cz = f["Z"] - 15
    if f["state"] in ("EJECT", "Z_DISPENSE", "Z_RETRACT", "PORTION_CHECK", "BAY_CHECK", "DONE"):
        cx, cz = m.CUP_X - 10, 10
    w, h = 330, 330 * (0.435 * H) / (0.405 * W)
    z.set_xlim(cx - w / 2, cx + w / 2)
    z.set_ylim(cz - h / 2, cz + h / 2)
    z.set_aspect("equal")
    z.set_xticks([])
    z.set_yticks([])
    for sp in z.spines.values():
        sp.set_color("#9aa6b2")
    m.draw_frame_static(z, tub_profiles={FID: (xs_tub, np.nan_to_num(f["profile"], nan=S_TRUE))},
                        show_cup=f["cup"])
    m.draw_head(z, f["X"], f["Z"], f["th"], fill=f["fill"], ball=f["ball"], ice=ICE, show_col=True)
    if f["ball_free"] is not None:
        z.add_patch(Circle(f["ball_free"], m.R - 1.5, fc=ICE, ec="#8a7a55", lw=0.6, zorder=12))
    if f["ball_in_cup"] and f["cup"]:
        z.add_patch(Circle((m.CUP_X, -2), m.R - 1.5, fc=ICE, ec="#8a7a55", lw=0.6, zorder=3.4))
    # rim lowest-point trail
    th = f["th"]
    lo = m.rim_ends(f["X"], f["Z"], th)[1]
    z.add_patch(Circle(lo, 2.2, fc=m.C_SENSOR, ec="none", zorder=14))
    z.axhline(m.Z_SAFE, color=m.C_SAFE, lw=0.9, ls=(0, (6, 4)))
    z.text(0.015, 0.97, "확대 (단면, 레인 y = 0)", transform=z.transAxes, fontsize=9, va="top",
           color=m.C_TEXT, bbox=dict(fc="white", ec="none", alpha=0.8, pad=1.5), zorder=30)
    z.text(0.985, 0.97, f"θ = {th:+.0f}°", transform=z.transAxes, fontsize=10, va="top", ha="right",
           color=m.C_TEXT, family=MONO, bbox=dict(fc="white", ec="none", alpha=0.8, pad=1.5), zorder=30)
    if f["note"] and f["state"] not in ("IDLE",):
        z.text(0.015, 0.03, f["note"], transform=z.transAxes, fontsize=8.5, va="bottom", color="#243447",
               bbox=dict(fc="#FFF6DA", ec="#E0C36A", lw=0.6, pad=2.5), zorder=30)
    if f["state"] == "EJECT":
        z.text(0.985, 0.03, "배출 방식은 시험 전 — 개략 표시", transform=z.transAxes, fontsize=8, ha="right",
               va="bottom", color="#8a4b00")

    # ---- signals
    s = ax_sig
    s.set_xlim(TS[0], TS[-1])
    s.set_ylim(0, 175)
    s.plot(TS[:k + 1], FX[:k + 1], color=m.C_MOVE, lw=1.4, label="F_x (X 볼너트 로드셀)")
    s.plot(TS[:k + 1], FZ[:k + 1], color="#7B61A8", lw=1.2, label="F_z (Z 볼너트 로드셀)")
    s.axhline(F_TARGET, color="#E08A00", lw=0.8, ls="--")
    s.axhline(F_STOP, color=m.C_SENSOR, lw=0.8, ls="--")
    s.text(TS[-1], F_TARGET + 3, "목표 110 N ", ha="right", fontsize=7, color="#b36b00")
    s.text(TS[-1], F_STOP + 3, "정지 160 N ", ha="right", fontsize=7, color=m.C_SENSOR)
    if f["touch_t"] is not None:
        s.axvline(f["touch_t"], color="#555", lw=0.6, ls=":")
        s.text(f["touch_t"], 70, " 접촉 3 N", fontsize=7, color="#444")
    s.axvline(f["t"], color="#333", lw=0.6)
    s.set_ylabel("N", fontsize=8)
    s.tick_params(labelsize=7)
    s.legend(loc="upper left", fontsize=7, frameon=False)
    s.set_title("힘 신호 [N] vs 시간 [s] (u = 90 kPa 가정)", fontsize=8.5, loc="left", pad=2)

    # ---- top view
    t_ = ax_top
    t_.set_xlim(-60, 1260)
    t_.set_ylim(-190, 190)
    t_.set_aspect("equal")
    t_.set_xticks([])
    t_.set_yticks([])
    t_.set_title("평면도 (위에서)", fontsize=8.5, loc="left", pad=2)
    t_.add_patch(Rectangle((m.FREEZER_X[0], -170), m.FREEZER_X[1] - m.FREEZER_X[0], 340, fc=m.C_DECK, ec="#8E9AA5"))
    for fid, name, tx, sfc, col in m.FLAVORS:
        t_.add_patch(Circle((tx, 0), 115, fc=col, ec="#8a7f6a", lw=0.6))
    for yl in (-40, 0, 40):
        t_.plot([TX - 70, TX + 70], [yl, yl], color="#6b5a2a", lw=0.5, ls=":")
    if f["cut_band"] is not None:
        x0, x1 = f["cut_band"]
        t_.add_patch(Rectangle((x0, -34), x1 - x0, 68, fc="#8a6d2a", alpha=0.45, ec="none"))
    t_.add_patch(Rectangle((m.RINSE_X - 60, -60), 120, 120, fc=m.C_WATER, ec="#8E9AA5"))
    t_.add_patch(Circle((m.CUP_X, 0), 45, fc=m.C_CUP, ec="#8E9AA5"))
    t_.plot([-40, 1240], [165, 165], color=m.C_STEEL, lw=3)
    t_.plot(f["X"], f["Y"], marker="o", ms=7, mfc="none", mec=m.C_MOVE, mew=1.8)
    t_.plot(f["X"], f["Y"], marker="+", ms=9, color=m.C_MOVE, mew=1.2)

    # ---- readouts
    x_ = ax_txt
    x_.axis("off")
    x_.set_xlim(0, 1)
    x_.set_ylim(0, 1)
    rows = [
        ("X / Y", f"{f['X']:7.1f} / {f['Y']:4.1f} mm"),
        ("Z (C 높이)", f"{f['Z']:7.1f} mm"),
        ("θ (피치)", f"{f['th']:+7.1f} °"),
        ("F_x / F_z", f"{f['Fx']:5.0f} / {f['Fz']:3.0f} N"),
        ("부피", f"{f['V'] / 1e3:5.0f} / {V_TARGET / 1e3:.0f} cm³"),
        ("저울", f"{f['scale']:7.1f} g"),
    ]
    for i, (k_, v_) in enumerate(rows):
        y = 0.92 - i * 0.165
        x_.text(0.02, y, k_, fontsize=8.5, color="#51606e", va="center")
        x_.text(0.98, y, v_, fontsize=8.5, color=m.C_TEXT, va="center", ha="right", family=MONO)

    # ---- state strip
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
    print(f"frames {len(frames)}  duration {len(frames) / FPS:.1f} s  machine cycle ~{T_CYCLE:.1f} s")
    print(f"d_end {d_end:.1f} mm, V {frames[-1]['V'] / 1e3:.0f} cm3, lane {X_START:.0f}-{X_END:.0f}")
    import sys
    if "--preview" in sys.argv:
        for k in [int(x) for x in sys.argv[sys.argv.index("--preview") + 1].split(",")]:
            draw_frame(k)
            fig.savefig(f"/tmp/claude-0/prev_{k}.png", dpi=DPI)
        raise SystemExit
    writer = FFMpegWriter(fps=FPS, bitrate=2400, codec="libx264",
                          extra_args=["-pix_fmt", "yuv420p", "-preset", "slow", "-crf", "22"])
    with writer.saving(fig, OUT_MP4, dpi=DPI):
        for k in range(len(frames)):
            draw_frame(k)
            writer.grab_frame()
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-i", OUT_MP4, "-vf",
                    "fps=12,scale=800:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=160[p];"
                    "[b][p]paletteuse=dither=bayer:bayer_scale=4", OUT_GIF], check=True)
    print("wrote", OUT_MP4, os.path.getsize(OUT_MP4) // 1024, "KB;", OUT_GIF, os.path.getsize(OUT_GIF) // 1024, "KB")
