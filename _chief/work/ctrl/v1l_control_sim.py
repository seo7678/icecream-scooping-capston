#!/usr/bin/env python3
"""V1-L (저가 구성 L-A: 이동 베드 X + 고정 브리지 Z·θ + 수동 Y 레인) 제어 계산.

Run:  python3 _chief/work/ctrl/v1l_control_sim.py
Out:  stdout  +  _chief/work/ctrl/v1l_control_sim_output.md
      (+ _chief/work/ctrl/fig_v1l_bed_inertia.png, matplotlib이 있을 때만)

다루는 것 (문서 `v1l_control.md` §3–§6의 숫자는 모두 여기서 나온다):
  1. 모션 보드 스텝 주파수 점검 (GRBL 1.1 / ATmega328P)
  2. 과부하 정지 지연 예산 → 추가 이동 거리 → 추가 힘, 정지 한계 제안
  3. 표면 검출(터치오프): 하강 속도 × HX711 샘플레이트 → Z0 오차, 2단 접근
  4. 깊이 제어: (a) 구간 적응 (b) 스트로크 간 u 추정 (c) 컴플라이언스 보상 + C 교정 모의
  5. 이동 베드 관성: 가속 구간 F_x 보정 vs 등속 구간만 사용 (시간영역 모의)
  6. 워치독: 노트북이 멈췄을 때 감독 없이 움직이는 거리

규칙: 근거 없는 상수는 모두 `# ASSUMPTION`. 데이터시트 값도 확인 전에는 "확인 필요".
결과는 실측이 아니다. 모의(synthetic) 데이터는 표 제목에 '모의'라고 적는다.
"""

import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_MD = os.path.join(HERE, "v1l_control_sim_output.md")
OUT_FIG = os.path.join(HERE, "fig_v1l_bed_inertia.png")
RNG = np.random.default_rng(20260928)          # 고정 시드 → 재현 가능

lines = []


def p(s=""):
    lines.append(s)


def table(h, rows):
    p("| " + " | ".join(h) + " |")
    p("|" + "|".join("---" for _ in h) + "|")
    for r in rows:
        p("| " + " | ".join(str(c) for c in r) + " |")
    p("")


def f1(x):
    return f"{x:.1f}"


def f2(x):
    return f"{x:.2f}"


def f3(x):
    return f"{x:.3f}"


# =====================================================================================
# 0. 상수
# =====================================================================================
# ---- 스쿱·레인 기하: calc/cartesian_model.py, calc/tub_lane_planner.py와 같은 식
R_SCOOP = 35.0            # mm, A06                                       # ASSUMPTION
ATTACK_DEG = 30.0         # deg, 공격 자세 −30° (A25)                       # ASSUMPTION
BETA_DIVE = 30.0          # deg, dive 경로 경사                              # ASSUMPTION
CLOSE_CAPTURE = 0.5       # 닫기로 담기는 구면 캡 비율                        # ASSUMPTION
LANE = 270.0              # mm, 젤라토 팬 360 mm 레인의 C 이동 (A40, capstone_alternatives §1)  # ASSUMPTION
RHO = 0.65                # g/cm³ (A07)                                      # ASSUMPTION
M_TARGET = 115.0          # g (A09, [MULTI] 확인됨)
V_TARGET = M_TARGET / RHO * 1e3   # mm³
G = 9.81

# ---- 힘 (A01/A02/A37 비율을 L-A 설계 하중에 적용)
F_DESIGN = 150.0          # N, L-A 구조 설계 하중 — 기계 담당 확정 필요          # ASSUMPTION
F_TARGET = 0.55 * F_DESIGN  # A37 비율
F_STOP_A37 = 0.80 * F_DESIGN  # A37 비율
K_VERT = 0.5              # |F_z|/F_x (A04, A28)                              # ASSUMPTION
U_CASES = (40.0, 90.0, 160.0, 250.0)   # kPa (A02)                           # ASSUMPTION
DFDX_CUT = (160.0 - 90.0) * 1e-3 * 624.0 / 10.0   # N/mm: 경도 90→160 kPa가 10 mm에 걸쳐 변할 때  # ASSUMPTION
F_JAM = 329.0             # N, X 스톨 추력 (mech/v1l_mech_calc_output §1, 계산값)                # ASSUMPTION
K_PATH = (70.0, 230.0, 290.0)   # N/mm, 스쿱 끝 ↔ 팬 X 경로 강성: L-A-lite 71, L-A 231–287 (mech/v1l_mech_calc_output §2.1, 계산값)  # ASSUMPTION

# ---- HX711 (AVIA 데이터시트 값 — 확인 필요)
HX711 = {
    80: {"Ts": 1 / 80, "settle": 0.050},   # RATE=1: 80 SPS, 정착시간 50 ms  (데이터시트, 확인 필요)
    10: {"Ts": 1 / 10, "settle": 0.400},   # RATE=0: 10 SPS, 정착시간 400 ms (데이터시트, 확인 필요)
}
HX_GROUP_FRAC = 0.5       # 내부 디지털 필터 군지연 = 정착시간 × 0.5 (sinc⁴ 유사 가정)   # ASSUMPTION
HX_ORDER = 4              # 필터 모델: 샘플 주기 길이 이동평균 4단 (정착 = 4 Ts와 일치)  # ASSUMPTION

# ---- 지연 요소 [ms] (min, nominal, worst)
LAT = {
    "nano":      (0.3, 0.5, 1.0),     # HX711 2개 비트뱅 읽기(24 bit) + 비교                  # ASSUMPTION
    "ser_in":    (2.1, 2.1, 2.1),     # 24 B × 10 bit / 115200 baud (계산)
    "usb_in":    (1.0, 4.0, 16.0),    # USB-직렬 칩(CH340/FTDI) 수신 지연. FTDI 기본 latency timer 16 ms  # ASSUMPTION(확인 필요)
    "py":        (0.5, 2.0, 30.0),    # 파이썬 파싱·판단 + OS 스케줄링·GC 튐                    # ASSUMPTION
    "usb_out":   (1.0, 2.0, 16.0),    # 노트북 → Uno(16U2) 1 byte                               # ASSUMPTION
    "grbl_rt":   (0.05, 0.5, 1.0),    # 실시간 문자/핀 인터럽트 → 메인 루프 처리               # ASSUMPTION
    "grbl_seg":  (0.0, 40.0, 50.0),   # 이미 준비된 스텝 세그먼트(6개 × ~10 ms) 소진 후 감속 시작  # ASSUMPTION(확인 필요)
    "grbl_rst":  (0.05, 0.1, 0.5),    # 리셋 핀 → 스텝 즉시 중단                                 # ASSUMPTION
    "probe":     (0.05, 0.2, 0.5),    # 프로브 핀은 스텝 ISR에서 감시                           # ASSUMPTION
}
A_X = 500.0               # mm/s², GRBL $120 (베드 X 가속)                         # ASSUMPTION
A_Z = 300.0               # mm/s², GRBL $122                                       # ASSUMPTION
V_DRAG_CASES = (20.0, 30.0, 40.0)   # mm/s (과제 지정)

# ---- 구조 컴플라이언스 (스쿱 끝 수직 처짐)
C_ZZ = 0.030              # mm/N, 수직 힘 → 수직 처짐 (3D프린터식 1.36 mm @ 50 N 수준)  # ASSUMPTION
C_ZX = 0.010              # mm/N, 수평 힘 → 수직 처짐 (캐리지 회전 × 팁 수평 오프셋)  # ASSUMPTION

# ---- 표면 검출
F_TOUCH = 3.0             # N (A34)
DELTA_TH = {"단단함(p=1 MPa)": 0.03, "부드러움(p=0.3 MPa)": 0.31}   # mm @ 3 N (z_axis_surface_detection §4, A33)
PROBE_MARGIN = 5.0        # mm, Z_est 위에서 저속 전환 (팬 지도 + 1점 터치오프 이력)    # ASSUMPTION

# ---- 이동 베드
M_PLAT = (5.0, 6.0, 7.2)  # kg, S-빔 위 질량 (과제 5–7 kg, 기계 담당 m_top 7.2 kg)        # ASSUMPTION
M_TRUE, M_EST = 7.2, 6.2  # kg, 모의 참값 / 보정에 쓰는 추정값(1 kg 틀림)                     # ASSUMPTION
K_SBEAM = (3.5e6, 1.5e6, 0.5e6)  # N/m: S-빔 50 kg 3504 N/mm(기계 담당), 레일·체결 포함 낮은 경우 2개  # ASSUMPTION
ZETA_PLAT = 0.05          # 플랫폼 진동 감쇠비                                        # ASSUMPTION
HX_NOISE_N = 0.05         # N rms, 모터 구동 중 HX711 잡음                           # ASSUMPTION

# ---- 워치독
HB_PERIOD = 0.100         # s, 호스트 → Nano 하트비트 주기                           # ASSUMPTION(설계값)
V_TRAVEL = 60.0           # mm/s, 베드 이송 최대 (GRBL $110 = 3600 mm/min)           # ASSUMPTION(설계값)
X_WIN_OVER = 2.0          # mm, X-창 스위치 끝 = 레인 끝 + 2 mm                     # ASSUMPTION(설계값)
WALL_MARGIN = 10.0        # mm, 레인 끝에서 스쿱 가장자리 ↔ 팬 벽 (A27)            # ASSUMPTION
WD_TIMEOUTS = (0.2, 0.3, 0.5)   # s                                                  # ASSUMPTION(설계값)


# =====================================================================================
# 공통 모델
# =====================================================================================
def ellipse_segment_area(a, b, h):
    if b <= 0 or h >= b:
        return 0.0
    h = max(h, -b)
    t = h / b
    return a * b * (math.acos(t) - t * math.sqrt(1.0 - t * t))


def swept_area(depth, r=R_SCOOP):
    """절삭 단면 [mm²] — calc/cartesian_model.swept_area(−30°, d)와 같은 식."""
    if depth <= 0:
        return 0.0
    b = r * math.cos(math.radians(ATTACK_DEG))
    return ellipse_segment_area(r, b, b - depth)


def cap_volume(depth):
    b = R_SCOOP * math.cos(math.radians(ATTACK_DEG))
    d_cap = R_SCOOP - (b - depth)
    return math.pi * d_cap ** 2 * (3 * R_SCOOP - d_cap) / 3.0


def stroke_volume(depth, travel=LANE):
    """dive 램프 + 등깊이 drag + 닫기 캡 — calc/tub_lane_planner.stroke_volume과 같은 식."""
    a = swept_area(depth)
    l_dive = depth / math.tan(math.radians(BETA_DIVE))
    l_drag = max(0.0, travel - l_dive)
    return 0.5 * a * min(l_dive, travel) + a * l_drag + CLOSE_CAPTURE * cap_volume(depth)


def bisect(fn, lo, hi, target, n=60):
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        if fn(mid) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


B_ATTACK = R_SCOOP * math.cos(math.radians(ATTACK_DEG))


def depth_for_volume(v, travel=LANE):
    return bisect(lambda d: stroke_volume(d, travel), 0.0, B_ATTACK, v)


def depth_for_force(f, u_kpa):
    return bisect(lambda d: u_kpa * 1e-3 * swept_area(d), 0.0, 2 * B_ATTACK - 0.01, f)


D_PORTION = depth_for_volume(V_TARGET)
A_PORTION = swept_area(D_PORTION)
SENS = (stroke_volume(D_PORTION + 0.05) - stroke_volume(D_PORTION - 0.05)) / 0.1 / V_TARGET * 100.0  # %/mm


def lat_path(path, sps=80):
    """경로별 지연 요소 목록 [(이름, min, nom, worst) ms]."""
    hx = HX711[sps]
    ts = hx["Ts"] * 1e3
    gd = hx["settle"] * HX_GROUP_FRAC * 1e3
    comps = [(f"HX711 변환 대기(샘플 위상, {sps} SPS)", 0.0, ts / 2, ts),
             (f"HX711 내부 필터 군지연(정착 {hx['settle']*1e3:.0f} ms × {HX_GROUP_FRAC})", gd, gd, gd),
             ("Nano 읽기·임계 비교", *LAT["nano"])]
    if path == "host":
        comps += [("Nano→노트북 직렬(24 B @115200)", *LAT["ser_in"]),
                  ("USB-직렬 수신 지연", *LAT["usb_in"]),
                  ("파이썬 처리 + OS 스케줄링", *LAT["py"]),
                  ("노트북→GRBL '!' 송신(USB)", *LAT["usb_out"])]
    if path in ("host", "pin_hold"):
        comps += [("GRBL 실시간 처리", *LAT["grbl_rt"]),
                  ("GRBL 세그먼트 버퍼 소진(등속 진행)", *LAT["grbl_seg"])]
    if path == "pin_reset":
        comps += [("GRBL 리셋 핀 → 스텝 중단", *LAT["grbl_rst"])]
    if path == "probe":
        comps += [("GRBL 프로브 핀 감시(스텝 ISR)", *LAT["probe"])]
    return comps


def lat_total(path, sps=80):
    c = lat_path(path, sps)
    return tuple(sum(x[i] for x in c) for i in (1, 2, 3))   # ms (min, nom, worst)


PATH_NAME = {
    "host": "A 호스트 경유 '!' (feed hold)",
    "pin_hold": "B Nano → GRBL 도어 핀 (feed hold)",
    "pin_reset": "C Nano → GRBL 리셋 핀 (즉시 정지)",
}


def stop_distance(v, t_ms, path, a=A_X):
    """판단 지연 동안 등속 + (감속 경로면) 감속 거리 [mm]."""
    s = v * t_ms * 1e-3
    if path != "pin_reset":
        s += v * v / (2 * a)
    return s


# =====================================================================================
# 머리말
# =====================================================================================
p("# V1-L 제어 계산 — generated")
p("")
p("> `python3 _chief/work/ctrl/v1l_control_sim.py`. 해석은 `_chief/work/ctrl/v1l_control.md`.")
p("> **모든 힘·강성·지연 값은 ASSUMPTION 또는 데이터시트 값(확인 필요)이다. 실측 결과가 아니다.** 모의 데이터는 '모의'로 표시.")
p("")
p("## 0. 기준값")
p("")
table(["항목", "값", "출처"], [
    ["레인 C 이동", f"{LANE:.0f} mm", "젤라토 팬 360 (A40), capstone_alternatives §1"],
    ["1 portion 목표", f"{M_TARGET:.0f} g → {V_TARGET/1e3:.1f} cm³ (ρ {RHO})", "A07, A09"],
    ["1 portion 깊이 d_portion", f"{D_PORTION:.1f} mm (A = {A_PORTION:.0f} mm²)", "stroke_volume, 같은 식"],
    ["깊이 민감도 dV/V", f"{SENS:.1f} %/mm", "d_portion에서 수치미분"],
    ["F_DESIGN (L-A 구조 설계 하중)", f"{F_DESIGN:.0f} N", "ASSUMPTION — 기계 담당 확정 필요"],
    ["F_TARGET / F_STOP (A37 비율)", f"{F_TARGET:.1f} / {F_STOP_A37:.0f} N", "0.55 / 0.80 × F_DESIGN"],
    ["드래그 힘 @ d_portion", " / ".join(f"{u:.0f} kPa → {u*1e-3*A_PORTION:.0f} N" for u in U_CASES), "F = u·A (A02)"],
    ["경도 변화 램프 dF/dx", f"{DFDX_CUT:.1f} N/mm", "90→160 kPa를 10 mm에 (ASSUMPTION)"],
    ["베드 X 가속 / Z 가속", f"{A_X:.0f} / {A_Z:.0f} mm/s²", "GRBL $120/$122 설정값 가정"],
])

# =====================================================================================
# 1. 스텝 주파수
# =====================================================================================
p("## 1. GRBL 1.1(ATmega328P) 스텝 주파수 점검")
p("")
p("GRBL 1.1 on Uno의 최대 스텝 주파수는 약 30 kHz로 알려져 있다(**확인 필요**). 풀스텝 200/rev 가정.")
p("")
STEP_LIMIT = 30e3   # Hz                                                              # ASSUMPTION(확인 필요)
rows = []
for axis, lead, gear, us, v, unit in [
        ("X 베드 (TR8 리드 4)", 4.0, 1, 8, 60.0, "mm/s"), ("X 베드 (TR8 리드 4)", 4.0, 1, 8, 100.0, "mm/s"),
        ("X 베드 (TR8 리드 4)", 4.0, 1, 4, 100.0, "mm/s"),
        ("Z (TR8 리드 2)", 2.0, 1, 8, 40.0, "mm/s"), ("Z (TR8 리드 2)", 2.0, 1, 4, 40.0, "mm/s"),
        ("θ (Y 채널, °)", None, 27, 8, 150.0, "°/s"), ("θ (Y 채널, °)", None, 27, 16, 150.0, "°/s")]:
    if lead:
        spu = 200 * us / lead
        drive = f"리드 {lead:.0f} mm, 1/{us}"
    else:
        spu = 200 * us * gear / 360.0
        drive = f"감속 1:{gear}, 1/{us}"
    f_step = spu * v
    rows.append([axis, drive, f"{spu:.1f} step/{unit.split('/')[0]}", f"{v:.0f} {unit}",
                 f"{f_step/1e3:.1f} kHz", "●" if f_step <= STEP_LIMIT * 0.8 else ("△" if f_step <= STEP_LIMIT else "✗")])
table(["축", "구동", "분해능", "최대 속도", "스텝 주파수", "≤ 24 kHz(●) / ≤ 30(△)"], rows)
p("- 구동계는 기계 담당 L-A(`mech/v1l_mech.md`): X TR8 리드 4, Z TR8 리드 2(자립), θ NEMA17 + 1:27. θ 닫기 150 °/s는 120°를 0.8 s에 도는 값(A32).")
p("- **X와 Z는 1/4 마이크로스텝**(분해능 X 0.005 mm, Z 0.0025 mm)이면 이송 100 mm/s·Z 40 mm/s까지 여유. X 1/8은 60 mm/s에서 24 kHz로 경계. θ는 1/8까지.")
p("")

# =====================================================================================
# 2. 지연 예산 → 정지 거리 → 추가 힘
# =====================================================================================
p("## 2. 과부하 정지 지연 예산")
p("")
p("### 2.1 지연 요소 (80 SPS)")
p("")
for path in ("host", "pin_hold", "pin_reset"):
    p(f"**{PATH_NAME[path]}**")
    p("")
    rows = [[n, f1(a), f1(b), f1(c)] for n, a, b, c in lat_path(path)]
    t = lat_total(path)
    rows.append(["**합계**", f"**{t[0]:.1f}**", f"**{t[1]:.1f}**", f"**{t[2]:.1f}**"])
    table(["요소", "min [ms]", "nominal [ms]", "worst [ms]"], rows)
rows = []
for path in ("host", "pin_hold", "pin_reset"):
    for sps in (80, 10):
        t = lat_total(path, sps)
        rows.append([PATH_NAME[path], sps, f1(t[1]), f1(t[2])])
p("### 2.2 샘플레이트별 합계")
p("")
table(["경로", "HX711 SPS", "nominal [ms]", "worst [ms]"], rows)
p("- 10 SPS는 샘플 대기(최대 100 ms)와 필터 지연(~200 ms)만으로 300 ms가 넘는다. **HX711 모듈의 RATE 핀을 80 SPS로 배선**해야 한다(모듈마다 다름, 확인 필요).")
p(f"- 직렬 대역: 2채널 80 SPS × 24 B ≈ {80*24} B/s, 115200 baud 용량 11 520 B/s의 {80*24/11520*100:.0f} %.")
p("")

p("### 2.3 드래그 속도별 추가 이동 거리와 추가 힘 (80 SPS, 감속 a = "
  f"{A_X:.0f} mm/s²)")
p("")
p("추가 힘 = 경로 강성 × 추가 이동. '경도 램프'는 절삭 중 경도가 올라가는 경우(dF/dx = "
  f"{DFDX_CUT:.1f} N/mm), 'k=…'는 강체 장애물(청크·팬 벽)에 걸린 경우다.")
p("")
rows = []
for v in V_DRAG_CASES:
    for path in ("host", "pin_hold", "pin_reset"):
        t = lat_total(path)
        s_nom = stop_distance(v, t[1], path)
        s_w = stop_distance(v, t[2], path)
        rows.append([f"{v:.0f}", PATH_NAME[path], f1(t[2]), f2(s_nom), f2(s_w),
                     f1(DFDX_CUT * s_w)] + [f"{k * s_w:.0f}" for k in K_PATH])
table(["v [mm/s]", "경로", "지연 worst [ms]", "추가 이동 nom [mm]", "추가 이동 worst [mm]",
       "ΔF 경도 램프 [N]"] + [f"ΔF 강체 k={k:.0f} N/mm [N]" for k in K_PATH], rows)

p("### 2.4 감속도 민감도 (경로 B, v = 30 mm/s, worst)")
p("")
rows = []
tB = lat_total("pin_hold")[2]
for a in (200.0, 250.0, 500.0, 1000.0):
    s = stop_distance(30.0, tB, "pin_hold", a)
    rows.append([f"{a:.0f}", f2(30.0 ** 2 / (2 * a)), f2(s), f1(DFDX_CUT * s)])
table(["a [mm/s²]", "감속 거리 [mm]", "총 추가 이동 [mm]", "ΔF 경도 램프 [N]"], rows)
p("- GRBL은 축마다 가속도가 하나($120)라 드래그용 완만한 램프(예: 200 mm/s²)를 따로 줄 수 없다. 낮추면 정지 거리도 같이 늘어난다.")
p("")

p("### 2.5 정지 한계 제안")
p("")
p(f"조건: 정지 후 최대 힘 = F_trip + ΔF(worst) ≤ F_DESIGN = {F_DESIGN:.0f} N (경도 램프 기준). "
  "F_trip 상한을 5 N 단위로 내림.")
p("")
rows = []
trip_max = {}
for v in V_DRAG_CASES:
    for path in ("host", "pin_hold"):
        s_w = stop_distance(v, lat_total(path)[2], path)
        fmax = math.floor((F_DESIGN - DFDX_CUT * s_w) / 5.0) * 5.0
        trip_max[(v, path)] = fmax
        rows.append([f"{v:.0f}", PATH_NAME[path], f2(s_w), f"{fmax:.0f}", f"{fmax / F_DESIGN:.2f}"])
table(["v [mm/s]", "경로", "추가 이동 worst [mm]", "F_trip 상한 [N]", "× F_DESIGN"], rows)

V_REC = 30.0
F_STOP_HW = min(F_STOP_A37, trip_max[(V_REC, "pin_hold")])
F_STOP_HOST = math.floor(0.7 * F_DESIGN / 5.0) * 5.0
p(f"**제안 (v_drag = {V_REC:.0f} mm/s):** 호스트 F_STOP_HOST = {F_STOP_HOST:.0f} N (0.70 × F_DESIGN, 먼저 걸림, 로그·재시도용), "
  f"Nano 하드웨어 F_STOP_HW = {F_STOP_HW:.0f} N (min(A37 0.80, 표 상한), 노트북과 무관하게 동작). "
  f"깊이 적응 목표 F_TARGET = {F_TARGET:.1f} N.")
p("")
p("강체 장애물은 힘 정지로 막을 수 없는 속도가 있다. 아래는 F_trip = F_STOP_HW에서 정지 후 힘이 F_DESIGN을 넘지 않는 최대 속도다.")
p("")
rows = []
for k in K_PATH:
    budget = (F_DESIGN - F_STOP_HW) / k       # mm
    for path in ("pin_hold", "host"):
        t = lat_total(path)[2] * 1e-3
        # v t + v²/2a = budget → v = a(−t + sqrt(t² + 2 budget/a))
        vmax = A_X * (-t + math.sqrt(t * t + 2 * budget / A_X))
        rows.append([f"{k:.0f}", PATH_NAME[path], f2(budget), f1(vmax)])
table(["경로 강성 k [N/mm]", "경로", "허용 추가 이동 [mm]", "허용 속도 [mm/s]"], rows)
p("- 강체 충돌은 keep-out(팬 벽 여유)·속도 제한·X 드라이버 전류 제한(스톨 추력)으로 막고, 힘 정지는 경도 변화·과깊이용으로 쓴다.")
p(f"- 강체 충돌 힘의 실제 상한은 X 스톨 추력이다: 기계 담당 계산 F_JAM ≈ {F_JAM:.0f} N(TR8 리드 4, NEMA17 0.4 N·m). "
  "위 표의 수백 N 값은 그 전에 모터가 탈조해 멈춘다는 뜻이다(위치 상실 → 재원점). 드라이버 전류를 낮추면 상한도 낮아지지만 드래그 여유(2.25)도 줄어든다.")
p("")

# =====================================================================================
# 3. 표면 검출
# =====================================================================================
p("## 3. 표면 검출(터치오프)")
p("")
p("Z0 오차 = 임계 압입 δ_th(재료) + 구조 처짐 C_zz·F_touch + 하강 속도 × 검출 지연. "
  "앞의 두 항과 지연의 평균은 **고정 편향**이라 제품별로 보정할 수 있고, 남는 것은 샘플 위상에 따른 흩어짐이다.")
p("")
p("- 경로 P: Nano가 F_z ≥ 3 N이면 GRBL 프로브 핀을 바꾸고 `G38.2`가 **그 순간의 위치를 기록**(PRB). 지연 = HX711 + Nano + 핀 감시.")
p("- 경로 H: 노트북이 힘을 보고 '!'. 위치는 정지 후 좌표에서 추정 → 호스트 경로 전체 지연의 흩어짐이 오차가 된다.")
p("")


def touch_lat(path, sps):
    if path == "P":
        return lat_total("probe", sps)
    t = lat_total("host", sps)
    return t


rows = []
touch_res = {}
for v in (1.0, 2.0, 5.0, 10.0):
    for path, sps in (("P", 80), ("P", 10), ("H", 80)):
        tmin, tnom, tw = touch_lat(path, sps)
        bias = v * tnom * 1e-3
        pp = v * (tw - tmin) * 1e-3
        sigma = pp / math.sqrt(12)
        over = v * LAT["grbl_seg"][2] * 1e-3 + v * v / (2 * A_Z)
        t_app = PROBE_MARGIN / v
        touch_res[(v, path, sps)] = (bias, sigma, pp)
        rows.append([f"{v:.0f}", f"{path} ({sps} SPS)", f1(tnom), f3(bias), f3(pp), f3(sigma),
                     f2(over), f1(t_app), f2(sigma * SENS)])
table(["v_z [mm/s]", "경로", "지연 nom [ms]", "지연 편향 [mm]", "흩어짐 p-p [mm]", "σ [mm]",
       "트리거 후 추가 압입 [mm]", f"마지막 {PROBE_MARGIN:.0f} mm 시간 [s]", "portion σ [%]"], rows)
p(f"- 재료 편향 δ_th(3 N): " + ", ".join(f"{k} {v:.2f} mm" for k, v in DELTA_TH.items())
  + f". 구조 처짐 C_zz·3 N = {C_ZZ*F_TOUCH:.2f} mm. 재료 차이 {max(DELTA_TH.values())-min(DELTA_TH.values()):.2f} mm가 "
  "80 SPS 지연 흩어짐보다 크다 → 제품별 δ_bias 보정(E0)이 더 중요.")
p(f"- portion σ는 깊이 민감도 {SENS:.1f} %/mm로 환산한 값(지연 흩어짐만).")
p("")

p("### 3.1 접근 방식 비교 (경로 P, 80 SPS)")
p("")
rows = []
fast = 15.0    # mm/s, Z_est + margin까지 빠른 하강                                     # ASSUMPTION
retract = 1.0  # mm, 2단 접근 후퇴량                                                     # ASSUMPTION
for name, stages in [("1단 10 mm/s", [10.0]), ("1단 5 mm/s", [5.0]), ("1단 2 mm/s", [2.0]),
                     ("2단 10 → 1 mm/s (1 mm 후퇴)", [10.0, 1.0])]:
    v1 = stages[0]
    t_total = PROBE_MARGIN / v1
    if len(stages) == 2:
        v2 = stages[1]
        over1 = v1 * LAT["grbl_seg"][2] * 1e-3 + v1 * v1 / (2 * A_Z) + touch_res[(v1, "P", 80)][0]
        t_total += retract / fast + 0.1 + (retract + over1) / v2   # 후퇴 + 정착 0.1 s + 재접근
        v_last = v2
    else:
        v_last = v1
    b, s, pp = touch_res[(v_last, "P", 80)]
    rows.append([name, f1(t_total), f3(b), f3(s), f2(s * SENS)])
table(["방식", "접근 시간 [s]", "지연 편향 [mm]", "σ [mm]", "portion σ [%]"], rows)
p("- 80 SPS + 프로브 핀이면 1단 5 mm/s로도 σ ≈ 수십 µm이다. 2단 접근은 10 SPS나 호스트 경로일 때, 또는 연구용 반복성 시험에서 쓴다.")
p("")

# =====================================================================================
# 4. 깊이 제어
# =====================================================================================
p("## 4. 깊이 제어 (펌웨어가 궤적을 버퍼링하는 조건)")
p("")
SEG = 20.0        # mm, 구간 길이 (과제 지정)
D_MIN = 3.0       # mm, 최소 절입                                                        # ASSUMPTION
DD_MAX = 4.0      # mm, 구간당 깊이 변화 한계 (Z 경사 ≤ 11°)                              # ASSUMPTION
F_NOISE = 0.02    # 구간 평균 힘 측정 오차 (상대, 1σ)                                     # ASSUMPTION


def u_profile(kind):
    if kind == "S1 균일 90 kPa":
        return lambda x: 90.0
    if kind == "S2 90 + 단단한 띠 160 (x 100–160)":
        return lambda x: 160.0 if 100.0 <= x < 160.0 else 90.0
    if kind == "S3 균일 160 kPa":
        return lambda x: 160.0
    if kind == "S4 균일 250 kPa":
        return lambda x: 250.0
    raise ValueError(kind)


def seg_mean_u(uf, x0, x1, n=21):
    xs = np.linspace(x0, x1, n)
    return float(np.mean([uf(x) for x in xs]))


def run_segments(uf, mode, lookahead, rng, c_eff_true=0.0, c_eff_est=0.0, u_first=None):
    """구간 적응 모의. mode: 'fixed' | 'adapt'.  c_eff: F_x 1 N당 수직 팁 처짐 [mm/N].

    반환: dict(F_max, n_over, trip, V_true, V_est, reason, extra_time)
    """
    d_plan = D_PORTION
    if u_first is not None:                      # (b)의 û로 첫 깊이를 정함
        d_plan = min(D_PORTION, depth_for_force(F_TARGET, u_first))
    l_dive = d_plan / math.tan(math.radians(BETA_DIVE))
    # dive: 절반 단면 (균일 u로 근사), 깊이는 계획값
    x = l_dive
    v_true = 0.5 * swept_area(d_plan) * l_dive
    v_est = v_true
    committed = [d_plan] * (lookahead + 1)      # 이미 GRBL에 들어간 구간 깊이(명령)
    comp_prev = 0.0                              # 직전 측정으로 계산한 보상량 [mm]
    f_hist, d_hist = [], []
    f_max, n_over, trip = 0.0, 0, False
    reason = "레인 끝"
    k = 0
    while x < LANE - 1e-6:
        d_cmd = committed[k]
        seg_len = min(SEG, LANE - x)
        # 부피 목표 도달 예측 → 마지막 구간 길이 조정 (호스트가 큐잉 시점에 계산)
        a_est_seg = swept_area(d_cmd)
        need = V_TARGET - CLOSE_CAPTURE * cap_volume(d_cmd) - v_est
        last = False
        if a_est_seg > 0 and need <= a_est_seg * seg_len:
            seg_len = max(0.0, need / a_est_seg)
            last = True
        u = seg_mean_u(uf, x, x + max(seg_len, 1e-3))
        # 실제 깊이: 명령 + 보상 − 처짐 (고정점 반복)
        z_comp = comp_prev if mode == "adapt" else 0.0
        d_act = d_cmd + z_comp
        for _ in range(30):
            fx = u * 1e-3 * swept_area(d_act)
            d_act = max(0.0, d_cmd + z_comp - c_eff_true * fx)
        fx = u * 1e-3 * swept_area(d_act)
        f_meas = fx * (1 + F_NOISE * rng.standard_normal())
        f_max = max(f_max, fx)
        if fx > F_TARGET * 1.1:
            n_over += 1
        if fx > F_STOP_HW:
            trip = True
            reason = "과부하 정지"
            v_true += swept_area(d_act) * seg_len * 0.5
            break
        v_true += swept_area(d_act) * seg_len
        d_est = d_cmd + z_comp - c_eff_est * f_meas      # 호스트가 아는 실제 깊이 추정
        v_est += swept_area(max(d_est, 0.0)) * seg_len
        f_hist.append(fx)
        d_hist.append(d_act)
        x += seg_len
        if last:
            reason = "부피 도달"
            break
        # 다음 결정: 이 구간 측정 → (k + lookahead + 1) 구간 깊이
        if mode == "adapt":
            u_hat = f_meas / max(swept_area(max(d_est, 0.1)), 1e-6) * 1e3
            d_next = min(D_PORTION, depth_for_force(F_TARGET, u_hat))
            d_next = max(D_MIN, min(d_next, committed[-1] + DD_MAX, ))
            d_next = max(d_next, committed[-1] - DD_MAX)
            comp_prev = c_eff_est * f_meas
        else:
            d_next = d_plan
        committed.append(d_next)
        k += 1
    v_true += CLOSE_CAPTURE * cap_volume(d_hist[-1] if d_hist else d_plan)
    v_est += CLOSE_CAPTURE * cap_volume(d_hist[-1] if d_hist else d_plan)
    n_seg = len(f_hist)
    extra = n_seg * (30.0 / A_X) if lookahead == 0 and mode == "adapt" else 0.0   # 구간마다 정지-재가속 (v=30)
    return dict(F_max=f_max, n_over=n_over, trip=trip, V_true=v_true, V_est=v_est,
                reason=reason, extra_time=extra, n_seg=n_seg)


p("### 4.1 (a) 구간 적응 — 20 mm 구간, 구간 평균 F_x로 다음 구간 깊이 결정 (모의)")
p("")
p(f"규칙: û = F̄/A(d), d_next = min(d_portion, A⁻¹(F_TARGET/û)), 구간당 |Δd| ≤ {DD_MAX:.0f} mm, d ≥ {D_MIN:.0f} mm. "
  f"L = GRBL에 미리 넣어 둔 구간 수(0이면 구간마다 정지). 힘 측정 오차 {F_NOISE*100:.0f} % (1σ), 컴플라이언스 없음(§4.3에서 따로).")
p("")
rows = []
for sc in ("S1 균일 90 kPa", "S2 90 + 단단한 띠 160 (x 100–160)", "S3 균일 160 kPa", "S4 균일 250 kPa"):
    uf = u_profile(sc)
    u_nom = uf(LANE / 2)
    for label, mode, la, uf0 in (("고정 깊이", "fixed", 0, None), ("구간 적응 L=0", "adapt", 0, None),
                                 ("구간 적응 L=1", "adapt", 1, None), ("구간 적응 L=1 + 첫 깊이 û(+10 %)", "adapt", 1, 1.1 * u_nom)):
        if uf0 is not None and sc.startswith("S2"):
            continue
        r = run_segments(uf, mode, la, np.random.default_rng(7), u_first=uf0)
        rows.append([sc, label, f1(r["F_max"]), r["n_over"], "예" if r["trip"] else "–",
                     f"{r['V_true']/V_TARGET*100:.0f}", r["reason"], f2(r["extra_time"])])
table(["경도 시나리오", "제어", "F_max [N]", f"F > 1.1·F_TARGET 구간 수", f"F > F_STOP_HW({F_STOP_HW:.0f} N)",
       "부피 / 목표 [%]", "종료", "추가 시간 [s]"], rows)
p("- L=1은 결정이 두 구간(40 mm) 늦어 단단한 띠 입구에서 한 구간 더 목표를 넘는다. L=0은 반응이 빠르지만 구간마다 정지·재가속.")
p("- 목표보다 단단한 제품(S3·S4)은 한 스트로크로 1 portion이 안 된다 → §4.2 두 번째 스트로크.")
p("- 첫 구간은 측정 전이라 구간 적응만으로는 막지 못한다(S4에서 첫 구간에 과부하 정지). **첫 깊이는 (b)의 û로 정하고, (a)는 레인 안의 경도 변화만 맡는다**(마지막 행).")
p("")

# ---------------- 4.2 스트로크 간 적응
p("### 4.2 (b) 스트로크 간 적응 — 첫 스트로크 평균 힘으로 u 추정 (모의)")
p("")
N_ORD = 8
U_DRIFT = np.linspace(170.0, 100.0, N_ORD)   # kPa, 단열 홀더에서 데워지며 물러짐        # ASSUMPTION
LAMBDA_U = 0.5                                # û 지수이동평균                             # ASSUMPTION
U_PRIOR = 250.0                               # kPa, 첫 주문의 보수적 사전값                 # ASSUMPTION


def scoop_order(u_true, u_hat_prior, rng, strategy):
    """한 주문: 스트로크 1 (+ 부족하면 스트로크 2). 반환 (V_total, strokes, F_max, u_meas, d1)."""
    if strategy == "fixed":
        d1 = D_PORTION
    else:
        d1 = min(D_PORTION, depth_for_force(F_TARGET, u_hat_prior))
    f1_ = u_true * 1e-3 * swept_area(d1)
    v1 = stroke_volume(d1)
    f_meas = f1_ * (1 + F_NOISE * rng.standard_normal())
    u_meas = f_meas / swept_area(d1) * 1e3
    strokes, v_tot, f_max = 1, v1, f1_
    if v1 < 0.95 * V_TARGET and strategy != "fixed":
        v_rem = V_TARGET - v1
        d2 = max(D_MIN, depth_for_volume(v_rem))
        u_for_2 = u_meas if strategy == "estimate" else u_hat_prior
        d2 = min(d2, depth_for_force(F_TARGET, u_for_2))
        v2 = stroke_volume(d2)
        f2_ = u_true * 1e-3 * swept_area(d2)
        strokes, v_tot, f_max = 2, v1 + v2, max(f_max, f2_)
    return v_tot, strokes, f_max, u_meas, d1


rows = []
summary = {}
for strategy, title in (("estimate", "u 추정(EMA λ=0.5)"), ("prior", "추정 없음(항상 250 kPa 가정)"), ("fixed", "고정 깊이(힘 한계 무시)")):
    rng = np.random.default_rng(11)
    u_hat = U_PRIOR
    errs, strokes_l, fmax_l = [], [], []
    for i, u in enumerate(U_DRIFT):
        v_tot, st, fmax, u_meas, d1 = scoop_order(u, u_hat, rng, strategy)
        errs.append((v_tot / V_TARGET - 1) * 100)
        strokes_l.append(st)
        fmax_l.append(fmax)
        if strategy == "estimate":
            rows.append([i + 1, f"{u:.0f}", f"{u_hat:.0f}", f1(d1), f1(fmax), st, f"{(v_tot/V_TARGET-1)*100:+.0f}"])
            u_hat = u_hat + LAMBDA_U * (u_meas - u_hat)
    summary[title] = (np.mean(np.abs(errs)), max(fmax_l), np.mean(strokes_l), sum(f > F_STOP_HW for f in fmax_l))
table(["주문", "u 실제 [kPa]", "û 사용 [kPa]", "d1 [mm]", "F_max [N]", "스트로크 수", "부피 오차 [%]"], rows)
table(["전략", "평균 |부피 오차| [%]", "F_max [N]", "평균 스트로크 수", f"F_STOP_HW 초과 횟수"],
      [[k, f1(v[0]), f1(v[1]), f2(v[2]), v[3]] for k, v in summary.items()])
p(f"- 가정: u가 {U_DRIFT[0]:.0f} → {U_DRIFT[-1]:.0f} kPa로 {N_ORD}회에 걸쳐 선형 감소. 두 번째 스트로크는 **따로 뜬 두 번째 공**(담은 채 재절입하지 않음).")
p("- 두 번째 스트로크 깊이는 같은 주문의 첫 스트로크에서 잰 u로 정한다. 다음 주문의 첫 깊이는 û(EMA)로 정한다.")
p("")

# ---------------- 4.3 컴플라이언스 보상
p("### 4.3 (c) 컴플라이언스 보상")
p("")
p("Z 목표 보정: Z_cmd = Z0 − d − (C_zz·F_z + C_zx·F_x). 스쿱이 위로 밀려 얕아지는 만큼 더 내려 명령한다.")
p("")
p("#### C 교정 절차 모의 (모의 데이터 — 실측 아님)")
p("")
CAL_KG = np.array([0, 1, 2, 3, 5, 7, 10], dtype=float)   # 추 [kg]                       # ASSUMPTION(시험 설계)
DIAL_SIGMA = 0.003     # mm, 다이얼 게이지 판독 잡음 (0.01 mm 눈금)                          # ASSUMPTION
HYST = 0.02            # mm, 부하/제하 분기 차 (체결 미끄럼·백래시)                           # ASSUMPTION
PULLEY_FRIC = 0.05     # 수평 하중용 도르래 마찰 → 힘 오차(편향)                              # ASSUMPTION


def fit_slope(f, y):
    A = np.vstack([f, np.ones_like(f)]).T
    coef, res, *_ = np.linalg.lstsq(A, y, rcond=None)
    n = len(f)
    resid = y - A @ coef
    s2 = resid @ resid / max(n - 2, 1)
    se = math.sqrt(s2 / np.sum((f - f.mean()) ** 2))
    return coef[0], se


cal_rows = []
cal_rng = np.random.default_rng(3)
for name, c_true, fric in (("C_zz (수직 추 → 수직 처짐)", C_ZZ, 0.0), ("C_zx (수평 도르래 → 수직 처짐)", C_ZX, PULLEY_FRIC)):
    f_nom = CAL_KG * G
    f_act = f_nom * (1 - fric)       # 도르래 마찰만큼 실제 힘이 작다
    y_up = c_true * f_act + DIAL_SIGMA * cal_rng.standard_normal(len(f_nom))
    y_dn = c_true * f_act + HYST + DIAL_SIGMA * cal_rng.standard_normal(len(f_nom))
    y_dn[0] = y_dn[0] - HYST          # 0 kg 복귀점은 원점 근처로 (단순화)
    s_up, se_up = fit_slope(f_nom, y_up)
    s_dn, se_dn = fit_slope(f_nom[::-1], y_dn[::-1])
    c_est = 0.5 * (s_up + s_dn)
    ci = 1.96 * math.sqrt(0.25 * (se_up ** 2 + se_dn ** 2) + (0.5 * (s_up - s_dn)) ** 2)
    cal_rows.append([name, f3(c_true), f"{s_up:.4f} / {s_dn:.4f}", f"{c_est:.4f} ± {ci:.4f}",
                     f"{(c_est/c_true-1)*100:+.1f}"])
table(["교정 항목", "C 참값(모의) [mm/N]", "부하/제하 기울기", "C 추정 ± 95 % [mm/N]", "편향 [%]"], cal_rows)
p("- 추 교정의 통계 불확실도는 수 % 수준이다. 그러나 실제 절삭 하중은 작용점(분포 하중)·온도(−14 °C)·체결 상태가 교정과 달라, "
  "**잔여 오차는 C 불확실도 ±20 %와 ±50 %로 본다**(아래 표).")
p("- 도르래 마찰은 C_zx를 체계적으로 치우치게 한다(모의에서 −5 % 근처). 도르래 대신 푸시풀 게이지로 수평 하중을 직접 재면 줄어든다.")
p("")

p("#### 보상 후 잔여 깊이 오차")
p("")
SIGMA_F = 2.0    # N, 등속 구간 평균 힘 측정 오차 (마찰·잔류 관성·잡음)                       # ASSUMPTION
rows = []
for czz in (0.002, 0.01, 0.03, 0.05):   # 0.002 ≈ 기계 담당 추정(0.08 mm @ F_z 50 N)
    czx = czz / 3.0                                                                          # ASSUMPTION 비율
    for fx in (56.0, 100.0):
        fz = K_VERT * fx
        delta = czz * fz + czx * fx
        c_eff = czz * K_VERT + czx
        for eps in (0.2, 0.5):
            resid_bias = eps * delta
            resid_rand = c_eff * SIGMA_F
            resid = resid_bias + resid_rand
            rows.append([f3(czz), f"{fx:.0f}", f2(delta), f"{delta*SENS:.0f}", f"±{eps*100:.0f}",
                         f2(resid), f"{resid*SENS:.1f}"])
table(["C_zz [mm/N] (C_zx = C_zz/3)", "F_x [N] (F_z = 0.5 F_x)", "보상 전 오차 [mm]", "보상 전 portion [%]",
       "C 불확실도", "보상 후 잔여 [mm] (편향 + C·σ_F)", "보상 후 portion [%]"], rows)
p(f"- σ_F = {SIGMA_F:.0f} N(등속 구간 평균 힘 오차) 가정. portion 환산은 깊이 민감도 {SENS:.1f} %/mm.")
p("- 편향 부분은 portion 저울 피드백(k_f EMA, `cup_dispensing_station.md` §5 — V1-L에서는 팬 질량 차)으로 여러 스쿱에 걸쳐 흡수된다. 스쿱마다 흩어지는 부분만 남는다.")
p("")

# 구간 적응 + 보상: 지연된 힘으로 보상할 때의 추가 오차 (S2)
p("#### 구간 적응과 결합할 때 (S2 단단한 띠, C_zz = 0.03, C_zx = 0.01, 모의)")
p("")
c_eff_true = C_ZZ * K_VERT + C_ZX
rows = []
for label, ce_est in (("보상 없음", 0.0), ("보상, C 정확", c_eff_true), ("보상, C +50 %", 1.5 * c_eff_true), ("보상, C −50 %", 0.5 * c_eff_true)):
    for la in (0, 1):
        r = run_segments(u_profile("S2 90 + 단단한 띠 160 (x 100–160)"), "adapt", la, np.random.default_rng(7),
                         c_eff_true=c_eff_true, c_eff_est=ce_est)
        rows.append([label, la, f1(r["F_max"]), f"{r['V_true']/V_TARGET*100:.0f}", f"{r['V_est']/V_TARGET*100:.0f}"])
table(["보상", "L", "F_max [N]", "실제 부피 / 목표 [%]", "호스트 추정 부피 / 목표 [%]"], rows)
p("- 보상 없음: 호스트는 명령 깊이로 부피를 적분하므로 실제보다 많이 떴다고 믿고 일찍 멈춘다 → portion 부족.")
p("- 보상량은 직전 구간 힘으로 계산하므로, 힘이 구간 사이에 크게 바뀌는 곳(띠 입구)에서는 한 구간 동안 C·ΔF만큼 어긋난다.")
p("")

# =====================================================================================
# 5. 이동 베드 관성
# =====================================================================================
p("## 5. 이동 베드 관성 — 가속 구간 F_x")
p("")
rows = []
for m in M_PLAT:
    rows.append([f"{m:.1f}"] + [f1(m * a * 1e-3) for a in (250.0, 500.0, 1000.0, 2000.0)])
table(["플랫폼 질량 [kg]", "a = 250", "500", "1000", "2000 mm/s² → m·a [N]"], rows)
for k in K_SBEAM:
    for m in (5.0, 7.0):
        fn = math.sqrt(k / m) / (2 * math.pi)
        p(f"- S-빔 경로 강성 {k/1e6:.1f} N/µm, {m:.0f} kg → 고유진동수 {fn:.0f} Hz (80 SPS 나이퀴스트 40 Hz)")
p("")


def sinc_filter(x, n_win, order=HX_ORDER):
    y = x.copy()
    ker = np.ones(n_win) / n_win
    for _ in range(order):
        y = np.convolve(y, ker)[: len(x)]   # 인과 이동평균
    return y


def bed_sim(m_true, m_est, k_s, v=30.0, a=A_X, f_cut0=56.0, dt=1e-4, T=1.2, seed=5):
    rng = np.random.default_rng(seed)
    n = int(T / dt)
    t = np.arange(n) * dt
    # 베드 속도 프로파일(사다리꼴): 0.1 s 시작, 등속 구간 20 mm, 정지 (정지 후 이완까지 창 안에)
    t0 = 0.1
    ta = v / a
    s_cruise = 20.0
    tc = s_cruise / v
    vb = np.zeros(n)
    ab = np.zeros(n)
    for i, ti in enumerate(t):
        if t0 <= ti < t0 + ta:
            ab[i] = a
            vb[i] = a * (ti - t0)
        elif t0 + ta <= ti < t0 + ta + tc:
            vb[i] = v
        elif t0 + ta + tc <= ti < t0 + 2 * ta + tc:
            ab[i] = -a
            vb[i] = v - a * (ti - t0 - ta - tc)
    sb = np.cumsum(vb) * dt
    # 절삭력: 이동 시작 후 2 mm 안에 증가, 정지 후 τ=0.1 s로 이완                       # ASSUMPTION
    fcut = np.zeros(n)
    for i in range(1, n):
        if vb[i] > 0:
            fcut[i] = f_cut0 * min(1.0, sb[i] / 2.0)
        else:
            fcut[i] = fcut[i - 1] * math.exp(-dt / 0.1)
    # 플랫폼: m y'' + c y' + k y = m a_b + F_cut,   센서 = k y + c y'   (단위 SI)
    c = 2 * ZETA_PLAT * math.sqrt(k_s * m_true)
    y = 0.0
    yd = 0.0
    fs = np.zeros(n)
    for i in range(n):
        ydd = (m_true * ab[i] * 1e-3 + fcut[i] - c * yd - k_s * y) / m_true
        yd += ydd * dt
        y += yd * dt
        fs[i] = k_s * y + c * yd
    # HX711 모델: sinc⁴ + 80 SPS 샘플 + 잡음
    nwin = int(round(HX711[80]["Ts"] / dt))
    fs_f = sinc_filter(fs, nwin)
    ref_f = sinc_filter(fcut, nwin)
    a_f = sinc_filter(ab * 1e-3, nwin)
    phase = int(rng.integers(0, nwin))
    idx = np.arange(phase, n, nwin)
    meas = fs_f[idx] + HX_NOISE_N * rng.standard_normal(len(idx))
    ref = ref_f[idx]
    moving = (t[idx] >= t0) & (t[idx] <= t0 + 2 * ta + tc)
    est = {
        "A 원신호": meas,
        "B − m̂·a_cmd (필터 없음)": meas - m_est * ab[idx] * 1e-3,
        "C − m̂·H(a_cmd) (같은 필터)": meas - m_est * a_f[idx],
    }
    # D: 등속 구간만 (가속 끝 + 4 Ts 이후, 감속 시작 전)
    mask_d = (t[idx] >= t0 + ta + HX_ORDER * HX711[80]["Ts"]) & (t[idx] < t0 + ta + tc)
    out = {}
    for kname, e in est.items():
        err = np.abs(e - ref)[moving]
        out[kname] = (float(err.max()), float(np.sqrt(np.mean(err ** 2))))
    err_d = np.abs(meas - ref)[mask_d]
    out["D 등속 구간만(원신호)"] = (float(err_d.max()), float(np.sqrt(np.mean(err_d ** 2))))
    lost_mm = v * ta / 2 + v * HX_ORDER * HX711[80]["Ts"] + v * ta / 2   # 가속 거리 + 정착 + 감속 거리
    series = dict(t=t, fs=fs, fcut=fcut, idx=idx, meas=meas, ref=ref, estC=est["C − m̂·H(a_cmd) (같은 필터)"],
                  estB=est["B − m̂·a_cmd (필터 없음)"], mask_d=mask_d)
    return out, lost_mm, series


rows = []
for k in K_SBEAM:
    fn = math.sqrt(k / M_TRUE) / (2 * math.pi)
    x_ = fn / 80.0
    att = abs(math.sin(math.pi * x_) / (math.pi * x_)) ** HX_ORDER
    x10 = fn / 10.0
    att10 = abs(math.sin(math.pi * x10) / (math.pi * x10)) ** HX_ORDER
    db = lambda g: "< −100" if g < 1e-5 else f"{20*math.log10(g):.0f}"
    rows.append([f"{k/1e6:.1f}", f"{fn:.0f}", db(att), db(att10)])
p(f"HX711 필터를 sinc⁴(샘플 주기 길이)로 가정했을 때 플랫폼 진동({M_TRUE:.1f} kg)이 얼마나 줄어드는지:")
p("")
table(["S-빔 경로 강성 [N/µm]", "f_n [Hz]", "80 SPS 감쇠 [dB]", "10 SPS 감쇠 [dB]"], rows)
p("- 80 SPS에서도 에일리어싱 성분이 수십 dB 줄어든다(필터 모양 가정 — 확인 필요, T6). 10 SPS로 내리면 감쇠는 조금 더 크지만 지연이 250 ms 이상 늘어난다(§2.2).")
p("")
p(f"### 5.1 시간영역 모의 (v = 30 mm/s, a = 500 mm/s², 절삭력 56 N, m 참값 {M_TRUE:.1f} kg / 추정 {M_EST:.1f} kg) — 모의")
p("")
p("오차 기준 = 같은 HX711 필터를 통과한 참 절삭력. 즉 필터 지연 자체는 빼고, 관성·진동이 섞인 양만 본다.")
p("")
rows = []
series_keep = None
for k in K_SBEAM:
    res, lost, ser = bed_sim(M_TRUE, M_EST, k)
    if series_keep is None:
        series_keep = ser
    fn = math.sqrt(k / M_TRUE) / (2 * math.pi)
    for name, (emax, erms) in res.items():
        rows.append([f"{k/1e6:.1f} ({fn:.0f} Hz)", name, f2(emax), f2(erms)])
table(["S-빔 경로 강성 [N/µm] (f_n)", "추정 방식", "최대 오차 [N]", "RMS 오차 [N]"], rows)
p(f"- 등속 구간만 쓰면 레인 양 끝에서 약 {lost:.1f} mm(가속·정착·감속) 동안 판정을 쉰다(레인 {LANE:.0f} mm의 {lost/LANE*100:.1f} %).")
p(f"- **과부하 정지 판정은 모든 구간에서 해야 한다.** 가속 구간에는 m·a_max({M_TRUE:.1f} kg × 0.5 m/s² = {M_TRUE*0.5:.1f} N)만큼 임계를 올리거나 방식 C로 보정한다. "
  "u 추정·깊이 적응·부피 적분에는 방식 D(등속 구간 평균)를 쓴다.")
p("")

# =====================================================================================
# 6. 워치독
# =====================================================================================
p("## 6. 워치독 — 노트북이 멈추거나 USB가 끊겼을 때")
p("")
p("Nano는 하트비트(주기 100 ms)가 T_WD 동안 없으면 GRBL 도어 핀(허가선)을 끊는다 → feed hold. "
  "Nano의 과부하 판정은 노트북과 무관하게 계속 동작한다.")
p("")
rows = []
t_pin = (LAT["grbl_rt"][2] + LAT["grbl_seg"][2]) * 1e-3
for twd in WD_TIMEOUTS:
    for v, what in ((30.0, "드래그"), (V_TRAVEL, "베드 이송(Z ≥ Z_SAFE)")):
        a = A_X
        s = v * (twd + t_pin) + v * v / (2 * a)
        rows.append([f"{twd*1e3:.0f}", what, f"{v:.0f}", f1(s)])
table(["T_WD [ms]", "동작", "v [mm/s]", "감독 없이 더 가는 거리 [mm]"], rows)
p("- 하트비트가 끊긴 순간부터 T_WD + GRBL 반응 + 감속. 드래그에서는 호스트가 미리 넣어 둔 궤적(L=1이면 최대 2구간 = 40 mm)보다 짧다. 이송은 한 블록이 길어(수백 mm) 워치독이 실제로 멈추는 장치다. 이송은 Z ≥ Z_SAFE에서만 하므로 팬과 부딪히지 않고, 베드 끝은 소프트·하드 리밋이 막는다.")
p("- 하트비트 주기 100 ms, T_WD = 300 ms를 제안(노트북 OS 스케줄링 튐 30 ms에 여유 10배).")
p("")

p("### 6.1 R7 하드웨어 이중화 — 게이트를 우회한 이송이 창 밖으로 나갈 때")
p("")
p(f"Nano가 'Z-높이 스위치 OFF 그리고 X-창 스위치 OFF'를 보면 GRBL 도어 입력을 끊는다(스위치는 HX711을 거치지 않음). "
  f"창 끝 = 레인 끝 + {X_WIN_OVER:.0f} mm, 팬 벽까지 C 여유 = 벽 여유 {WALL_MARGIN:.0f} mm − {X_WIN_OVER:.0f} mm.")
p("")
rows = []
for v in (40.0, 60.0, 80.0, 100.0):
    for label, tc in (("nominal", LAT["grbl_seg"][1]), ("worst", LAT["grbl_seg"][2])):
        s_ = v * (LAT["grbl_rt"][2] + tc) * 1e-3 + v * v / (2 * A_X)
        margin = WALL_MARGIN - X_WIN_OVER - s_
        rows.append([f"{v:.0f}", label, f2(s_), f2(margin), "●" if margin > 0 else "✗ 벽 접촉"])
table(["이송 속도 [mm/s]", "세그먼트 소진", "창 밖 추가 이동 [mm]", "벽까지 남는 여유 [mm]", "판정"], rows)
p(f"- **X 최대 속도($110)를 {V_TRAVEL:.0f} mm/s 이하로** 두면 게이트 우회 버그가 나도 벽 앞에서 멈춘다(계산상 여유 ~1 mm, 정지 거리 시험 T1로 확인). "
  "이송 속도를 올리려면 창을 좁히거나 벽 여유를 늘린다.")
p("")

p("## 7. 이 출력에서 ASSUMPTION인 것 (요약)")
p("")
p("F_DESIGN 150 N, 경로 강성 70/230/290 N/mm(기계 담당 계산값), 경도 램프 4.4 N/mm, HX711 필터 군지연 = 정착/2, GRBL 세그먼트 소진 0–50 ms, "
  "USB·파이썬 지연, 감속도 500 mm/s², C_zz 0.002–0.05·C_zx = C_zz/3 mm/N, σ_F 2 N, 플랫폼 5–7.2 kg·S-빔 강성 0.5–3.5 N/µm, u 경도 시나리오와 변화율. "
  "데이터시트 값(HX711 10/80 SPS, 정착 400/50 ms)과 GRBL 동작은 v1l_control.md '확인 필요' 목록에서 확인한다.")
p("")

# =====================================================================================
# 그림 (선택)
# =====================================================================================
FIG_NOTE = "matplotlib 없음 → 그림 생략"
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # 팔레트: dataviz 기준 팔레트 1–3번 슬롯(검증: CVD ΔE 9.2, 정상시 27.6, aqua 대비 경고 → 범례 + §5.1 표로 보완)
    SURF, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
    S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
    s = series_keep
    ti = s["t"][s["idx"]]
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.6), dpi=130, sharey=True, facecolor=SURF)
    windows = ((0.05, 0.40, "Drag start (bed accelerates)"), (0.70, 1.05, "Drag stop (bed decelerates)"))
    for ax, (x0, x1, ttl) in zip(axes, windows):
        ax.set_facecolor(SURF)
        ax.plot(s["t"], s["fs"], color=GRID, lw=1.0, label="S-beam force, before HX711 filter")
        ax.plot(ti, s["ref"], color=INK, lw=1.5, label="True cutting force (same filter)")
        ax.plot(ti, s["meas"], color=S1, lw=1.5, label="A  raw 80 SPS reading")
        ax.plot(ti, s["estB"], color=S2, lw=1.5, label="B  minus m*a_cmd (unfiltered)")
        ax.plot(ti, s["estC"], color=S3, lw=1.5, label="C  minus m*H(a_cmd) (same filter)")
        ax.set_xlim(x0, x1)
        ax.set_title(ttl, color=INK, fontsize=9, loc="left")
        ax.set_xlabel("time [s]", color=MUTED, fontsize=8)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=7, length=0)
        ax.grid(axis="y", color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    axes[0].set_ylabel("F_x [N]", color=MUTED, fontsize=8)
    h, lab = axes[0].get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", ncol=3, frameon=False, fontsize=7, labelcolor=INK)
    fig.suptitle(f"Moving-bed inertia in F_x (simulated: {M_TRUE:.1f} kg on S-beam, 500 mm/s^2, 30 mm/s, estimator mass {M_EST:.1f} kg)",
                 x=0.01, ha="left", color=INK, fontsize=9)
    fig.tight_layout(rect=(0, 0.14, 1, 0.95))
    fig.savefig(OUT_FIG, facecolor=SURF)
    plt.close(fig)
    FIG_NOTE = f"그림: `{os.path.basename(OUT_FIG)}` (§5.1 모의, 첫 번째 강성 조건)"
except Exception as exc:   # 그림은 선택 사항
    FIG_NOTE = f"그림 생략 ({type(exc).__name__})"
p(f"> {FIG_NOTE}")
p("")

text = "\n".join(lines)
print(text)
with open(OUT_MD, "w", encoding="utf-8") as fh:
    fh.write(text + "\n")
