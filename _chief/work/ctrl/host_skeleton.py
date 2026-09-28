#!/usr/bin/env python3
"""V1-L 호스트 골격 — 학생 노트북에서 도는 상태기계 (GRBL 1.1 모션 보드 + Arduino Nano 힘 보드).

    python3 host_skeleton.py --mock                       # 가짜 GRBL·힘 보드로 한 사이클
    python3 host_skeleton.py --mock --scenario overload   # 드래그 중 단단한 덩어리 → 정지 → FAULT → 복구
    python3 host_skeleton.py --mock --scenario hang       # 노트북 1 s 멈춤 → Nano 워치독 → 도어 hold → FAULT
    python3 host_skeleton.py --mock --scenario gate       # R7: 호스트 게이트 거부 + 게이트 우회 시 Nano 인터록
    python3 host_skeleton.py --grbl /dev/ttyACM0 --nano /dev/ttyUSB0   # 실제 하드웨어 (pyserial 필요, 미검증)

구조
    Host (상태기계, R7 게이트, 과부하 감시, 워치독, CSV 로그)
      ├─ GrblClient  : GRBL 1.1 텍스트 프로토콜 (send-response, '?' 상태, '!' '~' 0x18 실시간 문자)
      └─ NanoClient  : 힘 보드 프로토콜 (아래)
    포트는 mock(MockGrblPort, MockNanoPort)과 실제(SerialLinePort)가 같은 write()/readlines()를 가진다.
    → 호스트 코드 경로는 mock과 실제가 같다.

Nano 힘 보드 프로토콜 (115200 baud, 줄 단위) — 펌웨어는 아직 없음, 이 파일의 MockNanoPort가 기준 동작
    Nano → 호스트 :  F,<ms>,<Fx N>,<Fz N>,<flags>        80 SPS
    호스트 → Nano :  H          하트비트 (100 ms마다)
                     T          영점(tare)
                     P1 / P0    프로브 출력 무장/해제 (무장 중 Fz ≥ 3 N → GRBL 프로브 핀 트리거)
                     E1 / E0    허가선(GRBL 도어 입력) 무장/해제
                     R          래치(과부하·워치독·인터록) 해제 — 작업자 확인 뒤에만
                     U          R7 인터록 1회 우회 (Z가 낮은 채 창 밖에 멈췄을 때 Z 상승 복구용,
                                Z-높이 스위치가 켜지면 자동 해제. 데드맨은 그대로 필요)
                     L<fx>,<fz> 하드웨어 과부하 한계 [N]
    flags 비트: 아래 FL_* 상수.

모든 수치는 ASSUMPTION(`v1l_control.md`, `v1l_control_sim_output.md`). GRBL 동작 중 '확인 필요' 항목은
mock이 가정대로 흉내 낼 뿐이며, 실제 보드에서 T-시험(문서 §7)으로 확인해야 한다.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import random
import re
import sys
import time
from collections import deque

# =====================================================================================
# 설정 — 모두 ASSUMPTION. 시험 뒤 여기 한 곳에서 바꾼다.
# =====================================================================================


class Cfg:
    # 좌표계 (G-code 기준). 이동 베드지만 X는 '팬 기준 스쿱 C 위치'로 쓴다($3로 방향 반전).
    #   X: 팬 안쪽 벽(0) … 반대 벽(360) 방향, Z: 팬 테두리 기준 C 높이(위 +), θ: GRBL Y 채널 [°]
    PAN_LEN = 360.0            # mm (A40)                                           # ASSUMPTION
    R_SCOOP = 35.0             # mm (A06)                                           # ASSUMPTION
    X_HOME = 0.0
    X_LANE = (45.0, 315.0)     # C 드래그 범위 = 벽 여유 R + 10                     # ASSUMPTION
    X_WIN = (43.0, 317.0)      # Nano X-창 스위치(하드웨어 R7 이중화) 작동 범위      # ASSUMPTION
    X_CUP = 430.0              # 베드 위 컵 위치                                    # ASSUMPTION
    X_LIM = (-2.0, 460.0)      # 소프트 리밋 ($130)                                  # ASSUMPTION
    Z_TOP, Z_SAFE = 100.0, 60.0   # Z_SAFE = 테두리 + R + 25 (A31 방식)             # ASSUMPTION
    Z_LIM = (-75.0, 100.0)     # C 최저 = 팬 바닥 −120 + 10 + R                     # ASSUMPTION
    TH_HOME, TH_ATTACK, TH_CAPTURE, TH_EJECT = 95.0, -30.0, 90.0, -120.0            # ASSUMPTION
    TH_LIM = (-150.0, 95.0)
    LANES_Y = (-38.0, 0.0, 38.0)   # 수동 인덱스 핀 위치                             # ASSUMPTION
    SURFACE_EST = -22.0        # 표면 추정(지도·직전 터치오프). 실제보다 높게 잡아도 됨  # ASSUMPTION

    # 속도·가속 (GRBL $110–$122와 맞춘다)
    V_TRAVEL = 60.0            # mm/s, $110 = 3600 mm/min — 게이트 우회 시 정지 거리 제한  # ASSUMPTION
    V_Z_FAST = 20.0            # mm/s                                                  # ASSUMPTION
    V_PROBE = 5.0              # mm/s (v1l_control_sim §3.1)                          # ASSUMPTION
    V_DRAG = 30.0              # mm/s (v1l_control_sim §2.5)                          # ASSUMPTION
    V_TH = 150.0               # °/s                                                   # ASSUMPTION
    A_X = 500.0                # mm/s²                                                 # ASSUMPTION
    PROBE_MARGIN, PROBE_OVERTRAVEL = 5.0, 15.0   # mm                                  # ASSUMPTION
    T_CLOSE = 0.8              # s (A32)                                               # ASSUMPTION
    T_SETTLE = 1.0             # s, 칭량 전 정착                                        # ASSUMPTION

    # 힘 (F_DESIGN 150 N 가정, A37 비율 — v1l_control_sim §2.5)
    F_TOUCH = 3.0
    F_TARGET = 82.5
    F_STOP_HOST = 105.0
    F_STOP_HW = 120.0
    F_TRAVEL = 30.0            # 이송·Z 이동 중 예상 밖 힘 = 충돌 의심
    M_PLAT = 6.0               # kg, 관성 보정·임계 여유용 추정치                         # ASSUMPTION
    INERTIA_MASK = 3.0         # mm, 드래그 시작 후 힘 평균에서 뺄 거리(가속+정착)        # ASSUMPTION

    # 깊이 제어
    SEG = 20.0                 # mm 구간
    LOOKAHEAD = 1              # GRBL에 미리 넣어 둘 구간 수 (감독 없는 이동 상한 = (L+1)·SEG)
    D_MIN, D_MAX, DD_MAX = 3.0, 20.0, 4.0                                             # ASSUMPTION
    C_ZZ, C_ZX = 0.030, 0.010  # mm/N, 추 교정값(§7 T3)                                # ASSUMPTION
    K_VERT_PRIOR = 0.5         # Fz/Fx 사전값 (A04)                                     # ASSUMPTION
    DELTA_BIAS = 0.10          # mm, 제품별 터치오프 편향(E0에서 측정)                   # ASSUMPTION
    U_PRIOR = 160.0            # kPa, 첫 주문 u 추정 (보수적)                            # ASSUMPTION
    LAMBDA_U = 0.5             # û EMA                                                  # ASSUMPTION
    LAMBDA_K = 0.2             # k_f EMA (A36)
    RHO = 0.65                 # g/cm³ (A07)                                            # ASSUMPTION
    M_TARGET = 115.0           # g (A09)
    BETA_DIVE = 30.0           # °                                                      # ASSUMPTION
    CLOSE_CAPTURE = 0.5        #                                                        # ASSUMPTION

    # 감독
    HB_PERIOD = 0.10           # s, Nano 하트비트
    T_WD = 0.30                # s, Nano 워치독 (Nano 쪽 설정, mock이 사용)
    STATUS_PERIOD = 0.05       # s, GRBL '?' 폴링 (20 Hz; GRBL 권장 상한 확인 필요)
    GRBL_TIMEOUT = 0.50        # s, 상태 응답 없음 → COMM 결함
    NANO_TIMEOUT = 0.20        # s, 힘 샘플 없음 → COMM 결함
    LOG_DECIM = 4              # 힘 샘플 4개 중 1개 기록 (20 Hz) + 모든 이벤트


G = 9.81
B_ATTACK = Cfg.R_SCOOP * math.cos(math.radians(30.0))    # rim 최저점이 C보다 30.3 mm 아래

FL_TOUCH, FL_PERMIT, FL_OVERLOAD, FL_WATCHDOG = 1, 2, 4, 8
FL_DEADMAN, FL_LANE_PIN, FL_Z_HIGH, FL_X_WIN, FL_INTERLOCK = 16, 32, 64, 128, 256
LATCHES = {FL_OVERLOAD: "OVERLOAD_HW", FL_WATCHDOG: "WATCHDOG", FL_INTERLOCK: "INTERLOCK_R7"}


# =====================================================================================
# 절삭 기하 (calc/cartesian_model.py, tub_lane_planner.py와 같은 식)
# =====================================================================================
def swept_area(depth, r=Cfg.R_SCOOP):
    if depth <= 0:
        return 0.0
    b = B_ATTACK
    h = max(b - depth, -b)
    if h >= b:
        return 0.0
    t = h / b
    return r * b * (math.acos(t) - t * math.sqrt(1.0 - t * t))


def cap_volume(depth, r=Cfg.R_SCOOP):
    d_cap = r - (B_ATTACK - depth)
    return math.pi * d_cap ** 2 * (3 * r - d_cap) / 3.0


def bisect(fn, lo, hi, target, n=50):
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if fn(mid) < target else (lo, mid)
    return 0.5 * (lo + hi)


def depth_for_force(f, u_kpa):
    return bisect(lambda d: u_kpa * 1e-3 * swept_area(d), 0.0, 2 * B_ATTACK - 0.01, f)


def stroke_volume(d, travel):
    a = swept_area(d)
    l_dive = d / math.tan(math.radians(Cfg.BETA_DIVE))
    return 0.5 * a * min(l_dive, travel) + a * max(0.0, travel - l_dive) + Cfg.CLOSE_CAPTURE * cap_volume(d)


V_TARGET = Cfg.M_TARGET / Cfg.RHO * 1e3
D_PORTION = bisect(lambda d: stroke_volume(d, Cfg.X_LANE[1] - Cfg.X_LANE[0]), 0.0, B_ATTACK, V_TARGET)


class Fault(Exception):
    def __init__(self, code, note=""):
        super().__init__(f"{code} {note}")
        self.code, self.note = code, note


# =====================================================================================
# 실제 포트 (pyserial은 여기서만 import)
# =====================================================================================
class SerialLinePort:
    def __init__(self, dev, baud=115200):
        try:
            import serial                               # 실제 모드에서만 필요
        except ImportError:
            raise SystemExit("실제 모드에는 pyserial이 필요하다: python3 -m pip install pyserial")
        self.ser = serial.Serial(dev, baud, timeout=0)
        self.buf = b""

    def write(self, data: bytes):
        self.ser.write(data)

    def readlines(self):
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        parts = self.buf.split(b"\n")
        self.buf = parts.pop()
        return [s.decode(errors="replace").strip() for s in parts if s.strip()]


class RealRig:
    """실제 하드웨어. Uno/Nano는 포트를 열면 DTR로 리셋된다(확인 필요) → 2 s 대기."""

    def __init__(self, grbl_dev, nano_dev):
        self.grbl_port = SerialLinePort(grbl_dev)
        self.nano_port = SerialLinePort(nano_dev)
        time.sleep(2.0)
        self.grbl_port.write(b"\r\n\r\n")

    def now(self):
        return time.monotonic()

    def step(self):
        time.sleep(0.002)

    def operator_confirm(self, msg):
        return input(f"[작업자] {msg}  Enter=확인 / q=취소 > ").strip().lower() != "q"

    def summary(self):
        return {}


# =====================================================================================
# 프로토콜 클라이언트
# =====================================================================================
class GrblClient:
    STATUS_RE = re.compile(r"<([A-Za-z]+(?::\d)?)\|MPos:([-\d.]+),([-\d.]+),([-\d.]+)"
                           r"(?:\|Bf:(\d+),(\d+))?(?:\|FS:([-\d.]+),([-\d.]+))?")

    def __init__(self, port, clock):
        self.port, self.clock = port, clock
        self.acks = deque()
        self.alarms = deque()
        self.prb = None
        self.banner = False
        self.st = {"state": "Unknown", "x": 0.0, "th": 0.0, "z": 0.0, "bf": 0, "v": 0.0}
        self.st_t = -1.0

    def pump(self):
        for line in self.port.readlines():
            if line.startswith("<"):
                m = self.STATUS_RE.match(line)
                if m:
                    self.st = {"state": m.group(1), "x": float(m.group(2)), "th": float(m.group(3)),
                               "z": float(m.group(4)), "bf": int(m.group(5) or 0),
                               "v": float(m.group(7) or 0.0) / 60.0}
                    self.st_t = self.clock()
            elif line == "ok" or line.startswith("error:"):
                self.acks.append(line)
            elif line.startswith("ALARM:"):
                self.alarms.append(int(line.split(":")[1]))
            elif line.startswith("[PRB:"):
                x, th, z = (float(v) for v in line[5:].split(":")[0].split(","))
                ok = line.rstrip("]").endswith(":1")
                self.prb = (x, th, z, ok)
            elif line.startswith("Grbl"):
                self.banner = True

    def send_line(self, line):
        self.port.write((line + "\n").encode())

    def realtime(self, ch: bytes):
        self.port.write(ch)


class NanoClient:
    def __init__(self, port, clock):
        self.port, self.clock = port, clock
        self.new = deque()
        self.flags = 0
        self.last_t = -1.0
        self.fx = self.fz = 0.0

    def pump(self):
        for line in self.port.readlines():
            if line.startswith("F,"):
                _, ms, fx, fz, fl = line.split(",")
                self.fx, self.fz, self.flags = float(fx), float(fz), int(fl)
                self.last_t = self.clock()
                self.new.append((self.last_t, self.fx, self.fz, self.flags, int(ms)))

    def cmd(self, s):
        self.port.write((s + "\n").encode())


# =====================================================================================
# MOCK: 가짜 GRBL 1.1 (텍스트 프로토콜, 15블록 플래너, feed hold, 도어, 프로브, 소프트 리셋, 소프트 리밋)
# =====================================================================================
class MockGrblPort:
    AX_V = (100.0, 200.0, 30.0)      # $110–$112 [단위/s] — X는 host가 V_TRAVEL로 제한
    AX_A = (500.0, 1000.0, 300.0)    # $120–$122
    SEG_COAST = 0.040                # s, feed hold 후 감속 시작까지 등속 (확인 필요 — v1l_control_sim §2)
    PLANNER = 15
    WORD = re.compile(r"([A-Z])([-+]?\d*\.?\d+)")

    def __init__(self, clock, cfg):
        self.clock, self.cfg = clock, cfg
        self.lim = [cfg.X_LIM, cfg.TH_LIM, cfg.Z_LIM]
        self.pos = [150.0, 20.0, 90.0]   # 전원 투입 위치 (원점 모름)
        self.plan_pos = list(self.pos)
        self.vel = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.homed = False
        self.alarm = True
        self.q = deque()
        self.rx = deque()
        self.inbuf = b""
        self.out = ["Grbl 1.1h ['$' for help]", "[MSG:'$H'|'$X' to unlock]"]
        self.cur = None
        self.v = 0.0
        self.hold = None          # None | 'coast' | 'decel' | 'done'
        self.hold_kind = None     # 'Hold' | 'Door' | 'Cancel'(프로브)
        self.coast = 0.0
        self.sync_pending = None  # 동기 명령($H, G38.2, G4)이 끝나야 다음 줄을 읽는다
        self.abs = True
        self.inv = False
        self.feed = 600.0
        self.door_open = lambda: False
        self.probe_trig = lambda: False
        self.prb_done = False
        self.events = []

    # ---- 호스트 쪽 인터페이스
    def write(self, data: bytes):
        for b in data:
            c = bytes([b])
            if c == b"?":
                self.out.append(self._status())
            elif c == b"!":
                self._start_hold("Hold")
            elif c == b"~":
                self._cycle_start()
            elif c == b"\x18":
                self._soft_reset()
            elif c == b"\n":
                self.rx.append(self.inbuf.decode().strip())
                self.inbuf = b""
            elif c != b"\r":
                self.inbuf += c
        self._accept()

    def readlines(self):
        out, self.out = self.out, []
        return out

    # ---- 상태
    def _state(self):
        if self.alarm:
            return "Alarm"
        if self.hold_kind == "Door":
            if self.hold in ("coast", "decel"):
                return "Door:2"
            return "Door:1" if self.door_open() else "Door:0"
        if self.hold_kind == "Hold":
            return "Hold:1" if self.hold in ("coast", "decel") else "Hold:0"
        if self.cur is not None and self.cur["kind"] == "HOME":
            return "Home"
        if self.cur is not None or self.q:
            return "Run"
        return "Idle"

    def _status(self):
        x, th, z = self.pos
        bf = self.PLANNER - len(self.q) - (1 if self.cur else 0)
        return f"<{self._state()}|MPos:{x:.3f},{th:.3f},{z:.3f}|Bf:{bf},{128 - len(self.rx)}|FS:{self.v*60:.0f},{self.feed:.0f}>"

    # ---- 실시간 명령
    def _start_hold(self, kind):
        if self.cur is None or self.alarm or self.hold_kind in ("Hold", "Door"):
            if kind == "Door" and self.cur is None and not self.alarm and self.hold_kind is None:
                self.hold_kind, self.hold = "Door", "done"      # 정지 중 도어 열림 → Door 상태
            return
        self.hold_kind, self.hold, self.coast = kind, "coast", self.SEG_COAST
        self.events.append((self.clock(), f"{kind}_START", list(self.pos)))

    def _cycle_start(self):
        if self.hold != "done":
            return
        if self.hold_kind == "Door" and self.door_open():
            return                                   # 도어가 열려 있으면 재개 불가(레벨 감지)
        self.hold_kind, self.hold = None, None

    def _soft_reset(self):
        moving = self.cur is not None and (self.v > 1e-6 or self.hold in ("coast", "decel"))
        if moving or (self.cur is not None and self.cur["kind"] == "HOME"):
            self.homed = False
            self.alarm = True
            self.out.append("ALARM:3")                # 움직이는 중 리셋 → 위치 상실
        self.q.clear(); self.rx.clear(); self.cur = None; self.v = 0.0
        self.vel = [0.0] * 3; self.acc = [0.0] * 3
        self.hold = self.hold_kind = None; self.sync_pending = None
        self.plan_pos = list(self.pos)
        self.abs, self.inv = True, False
        self.out.append("Grbl 1.1h ['$' for help]")
        if not self.homed:
            self.alarm = True
            self.out.append("[MSG:'$H'|'$X' to unlock]")

    def _raise_alarm(self, code):
        self.alarm = True
        self.out.append(f"ALARM:{code}")
        self.q.clear(); self.rx.clear(); self.cur = None; self.v = 0.0
        self.vel = [0.0] * 3; self.acc = [0.0] * 3
        self.sync_pending = None; self.hold = self.hold_kind = None
        self.plan_pos = list(self.pos)

    # ---- 줄 해석
    def _accept(self):
        while self.rx and self.sync_pending is None:
            nq = len(self.q) + (1 if self.cur else 0)
            if nq >= self.PLANNER:
                return
            line = self.rx.popleft().upper().replace(" ", "")
            self._line(line)

    def _line(self, line):
        if line == "":
            self.out.append("ok"); return
        if line == "$H":
            if self.door_open():
                self.out.append("error:9"); return   # 단순화: 도어 열림이면 원점 거부
            self.alarm = False
            c = self.cfg
            for tgt in ([self.pos[0], self.pos[1], c.Z_TOP], [c.X_HOME, self.pos[1], c.Z_TOP],
                        [c.X_HOME, c.TH_HOME, c.Z_TOP]):             # Z 먼저(R7), 다음 X, 다음 θ
                self._queue("HOME", tgt, 15.0 if tgt[2] != self.pos[2] else 40.0, check=False)
            self.sync_pending = self.q[-1]
            return
        if line == "$X":
            self.alarm = False
            self.out.append("[MSG:Caution: Unlocked]"); self.out.append("ok"); return
        if line.startswith("$"):
            self.out.append("ok"); return
        if self.alarm:
            self.out.append("error:9"); return         # 알람 중 G-code 잠금
        words = self.WORD.findall(line)
        gs = [float(v) for k, v in words if k == "G"]
        ax = {k: float(v) for k, v in words if k in "XYZ"}
        if "F" in dict(words):
            self.feed = float(dict(words)["F"])
        for g in gs:
            if g == 90: self.abs = True
            elif g == 91: self.abs = False
            elif g == 93: self.inv = True
            elif g == 94: self.inv = False
        if 4.0 in gs:
            p_ = float(dict(words).get("P", 0.0))
            self._queue("DWELL", list(self.plan_pos), 0.0, dwell=p_, check=False)
            self.sync_pending = self.q[-1]
            return
        motion = None
        if 38.2 in gs:
            motion = "PROBE"
        elif 0.0 in gs:
            motion = "G0"
        elif 1.0 in gs or ax:
            motion = "G1"
        if motion and ax:
            tgt = list(self.plan_pos)
            for i, k in enumerate("XYZ"):
                if k in ax:
                    tgt[i] = ax[k] if self.abs else tgt[i] + ax[k]
            if motion == "PROBE" and self.probe_trig():
                self._raise_alarm(4); return            # 시작부터 트리거 상태
            if not self._queue(motion, tgt, self.feed / 60.0):
                return
            if motion == "PROBE":
                self.prb_done = False
                self.sync_pending = self.q[-1]
                return
        self.out.append("ok")

    def _queue(self, kind, tgt, feed, dwell=0.0, check=True):
        if check:
            for i in range(3):
                if not (self.lim[i][0] - 1e-6 <= tgt[i] <= self.lim[i][1] + 1e-6):
                    self._raise_alarm(2)                  # 소프트 리밋 (위치 유지)
                    return False
        d = [tgt[i] - self.plan_pos[i] for i in range(3)]
        L = math.sqrt(sum(x * x for x in d))
        u = [x / L for x in d] if L > 1e-9 else [0.0, 0.0, 0.0]
        vmax = min([self.AX_V[i] / abs(u[i]) for i in range(3) if abs(u[i]) > 1e-9] or [1.0])
        if kind == "G0":
            feed = vmax
        elif self.inv and kind == "G1" and L > 0:
            feed = L * self.feed / 60.0                  # G93: F = 1/분 → 블록 시간 = 1/F 분
        a = min([self.AX_A[i] / abs(u[i]) for i in range(3) if abs(u[i]) > 1e-9] or [1.0])
        blk = {"kind": kind, "start": list(self.plan_pos), "tgt": list(tgt), "u": u, "L": L,
               "vmax": min(feed, vmax) if feed > 0 else vmax, "a": a, "s": 0.0, "dwell": dwell, "t": 0.0}
        self.q.append(blk)
        self.plan_pos = list(tgt)
        return True

    def _exit_speed(self, blk):
        if self.hold:
            return 0.0
        if self.q and self.q[0]["kind"] == blk["kind"] == "G1":
            n = self.q[0]
            cos = sum(blk["u"][i] * n["u"][i] for i in range(3))
            if cos > 0.95:
                return min(blk["vmax"], n["vmax"])
        return 0.0

    def advance(self, dt):
        # 도어 입력은 레벨 감지: 열리면 hold
        if self.door_open() and not self.alarm and self.hold_kind != "Door":
            if self.cur is not None:
                if self.hold_kind == "Hold" and self.hold == "done":
                    self.hold_kind = "Door"
                else:
                    self.hold_kind, self.hold, self.coast = "Door", "coast", self.SEG_COAST
                    self.events.append((self.clock(), "DOOR_START", list(self.pos)))
            else:
                self._start_hold("Door")
        self._accept()
        if self.cur is None:
            if not self.q or (self.hold == "done"):
                self.vel = [0.0] * 3; self.acc = [0.0] * 3
                return
            self.cur = self.q.popleft()
        blk = self.cur
        if blk["kind"] == "DWELL":
            blk["t"] += dt
            if blk["t"] >= blk["dwell"]:
                self._finish_block()
            return
        # 프로브: 스텝 ISR처럼 매 틱 감시, 트리거 순간 위치 기록 → 감속(모션 취소)
        if blk["kind"] == "PROBE" and not self.prb_done and self.probe_trig():
            self.prb_done = True
            x, th, z = self.pos
            self._prb_line = f"[PRB:{x:.3f},{th:.3f},{z:.3f}:1]"
            self.hold_kind, self.hold, self.coast = "Cancel", "coast", self.SEG_COAST
        v_old = self.v
        s_rem = blk["L"] - blk["s"]
        if self.hold == "coast":
            self.coast -= dt
            if self.coast <= 0:
                self.hold = "decel"
        elif self.hold == "decel":
            self.v = max(0.0, self.v - blk["a"] * dt)
        elif self.hold == "done":
            self.v = 0.0
        else:
            v_exit = self._exit_speed(blk)
            if self.v * self.v - v_exit * v_exit >= 2 * blk["a"] * s_rem:
                self.v = max(v_exit, self.v - blk["a"] * dt)
            else:
                self.v = min(blk["vmax"], self.v + blk["a"] * dt)
        ds = min(self.v * dt, s_rem)
        blk["s"] += ds
        for i in range(3):
            self.pos[i] = blk["start"][i] + blk["u"][i] * blk["s"]
            self.vel[i] = blk["u"][i] * self.v
            self.acc[i] = blk["u"][i] * (self.v - v_old) / dt
        if self.hold == "decel" and self.v <= 0.0:
            self.hold = "done"
            self.events.append((self.clock(), f"{self.hold_kind}_STOPPED", list(self.pos)))
            if self.hold_kind == "Cancel":                # 프로브 정지: 블록 버리고 Idle
                self.hold = self.hold_kind = None
                self.out.append(self._prb_line)
                self.plan_pos = list(self.pos)
                self._finish_block(done=True)
            return
        if blk["L"] - blk["s"] <= 1e-9:
            if blk["kind"] == "PROBE" and not self.prb_done:
                self._raise_alarm(5)                     # 끝까지 접촉 없음
                return
            self._finish_block()

    def _finish_block(self, done=False):
        blk = self.cur
        self.cur = None
        if not (blk["kind"] == "G1" and self.q and self.q[0]["kind"] == "G1"):
            self.v = 0.0                                  # 다음 블록으로 속도를 넘기지 않음
        if blk is self.sync_pending:
            self.sync_pending = None
            if blk["kind"] == "HOME":
                self.homed = True
            self.out.append("ok")
        self._accept()


# =====================================================================================
# MOCK: 가짜 Nano 힘 보드 + 팬·아이스크림 물리
# =====================================================================================
class Ice:
    """레인 방향 1D 표면 높이와 경도. 절삭 이력(깎인 높이)을 기억한다."""

    DX = 0.5

    def __init__(self, cfg, scenario):
        self.x = [i * self.DX for i in range(int(cfg.PAN_LEN / self.DX) + 1)]
        self.zs = [-24.0 for _ in self.x]                      # 실제 표면 (80 % 채움)      # ASSUMPTION
        self.u = [90.0 for _ in self.x]                        # kPa                       # ASSUMPTION
        if scenario == "overload":
            for i, xi in enumerate(self.x):
                if 170.0 <= xi <= 186.0:
                    self.u[i] = 600.0                          # 단단한 덩어리(청크)       # ASSUMPTION

    def idx(self, x):
        return max(0, min(len(self.x) - 1, int(round(x / self.DX))))

    def cut(self, x0, x1, z_rim):
        vol = 0.0
        for i in range(self.idx(min(x0, x1)), self.idx(max(x0, x1)) + 1):
            d = self.zs[i] - z_rim
            if d > 0:
                vol += swept_area(d) * self.DX
                self.zs[i] = z_rim
        return vol


class MockNanoPort:
    TS = 1 / 80.0          # 80 SPS
    DELAY = 0.025          # HX711 필터 군지연 가정 (v1l_control_sim §2)
    NOISE = 0.05           # N rms                                                    # ASSUMPTION
    K_C = 20.0             # N/mm, 정지 압입 강성(부드러운 쪽)                          # ASSUMPTION
    C_ZZ_TRUE, C_ZX_TRUE = 0.033, 0.011   # 실제 구조(교정값보다 +10 %)                   # ASSUMPTION
    M_PLAT_TRUE = 6.5      # kg                                                       # ASSUMPTION

    def __init__(self, clock, grbl, ice, cfg):
        self.clock, self.g, self.ice, self.cfg = clock, grbl, ice, cfg
        self.rng = random.Random(4)
        self.hist = deque(maxlen=400)
        self.inbuf = b""
        self.out = []
        self.next_t = 0.0
        self.tare = (0.0, 0.0)
        self.probe_armed = False
        self.touch = False
        self.permit_armed = False
        self.last_hb = -1e9
        self.latch = 0
        self.lim = (cfg.F_STOP_HW, cfg.F_STOP_HW)
        self.deadman = True        # mock: 작업자가 hold-to-run을 누르고 있음
        self.lane_pin = True
        self.r7_override = False
        # 물리 상태
        self.engaged = False
        self.fx = self.fz = 0.0
        self.x_prev = grbl.pos[0]
        self.removed_mm3 = 0.0     # 팬에서 빠진 부피(누적)
        self.scoop_mm3 = 0.0       # 스쿱 안
        self.cup_g = 0.0
        self.d_last = 0.0
        self.cap_added = False
        self.collision_mm = 0.0
        self.z_act = grbl.pos[2]

    # ---- 호스트 쪽
    def write(self, data: bytes):
        self.inbuf += data
        while b"\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\n", 1)
            self._cmd(line.decode().strip())

    def readlines(self):
        out, self.out = self.out, []
        return out

    def _cmd(self, c):
        now = self.clock()
        if c == "H":
            self.last_hb = now
        elif c == "T":
            self.tare = (self._raw()[0], self._raw()[1])
        elif c == "P1":
            self.probe_armed, self.touch = True, False
        elif c == "P0":
            self.probe_armed, self.touch = False, False
        elif c == "E1":
            self.permit_armed, self.last_hb = True, now
        elif c == "E0":
            self.permit_armed = False
        elif c == "R":
            self.latch = 0
        elif c == "U":
            self.r7_override = True
        elif c.startswith("L"):
            a, b = c[1:].split(",")
            self.lim = (float(a), float(b))

    # ---- GRBL 쪽 하드웨어 출력
    def z_high(self):
        return self.g.pos[2] >= self.cfg.Z_SAFE - 1.0          # 스위치 작동점 Z_SAFE − 1 mm   # ASSUMPTION

    def x_win(self):
        return self.cfg.X_WIN[0] <= self.g.pos[0] <= self.cfg.X_WIN[1]

    def permit(self):
        return (self.permit_armed and self.latch == 0 and self.deadman and self.lane_pin
                and (self.z_high() or self.x_win() or self.r7_override))

    def door_open(self):
        return not self.permit()               # INVERT: 풀업(무전원) = 도어 열림 = hold

    def probe_triggered(self):
        return (not self.probe_armed) or self.touch   # $6=1: 풀업(무전원·미무장) = 트리거

    # ---- 물리
    def _raw(self):
        """HX711 지연을 반영한 (Fx, Fz) — DELAY 전의 값."""
        t = self.clock() - self.DELAY
        for (ti, fx, fz) in reversed(self.hist):
            if ti <= t:
                return fx, fz
        return (self.hist[0][1], self.hist[0][2]) if self.hist else (0.0, 0.0)

    def advance(self, dt):
        cfg, g = self.cfg, self.g
        x, th, zc = g.pos
        vx = g.vel[0]
        # 구조 컴플라이언스: 스쿱이 받는 힘만큼 위로 처짐 (직전 틱 힘 사용)
        self.z_act = zc + self.C_ZZ_TRUE * self.fz + self.C_ZX_TRUE * self.fx
        z_rim = self.z_act - B_ATTACK
        fx = fz = 0.0
        if abs(th - cfg.TH_ATTACK) < 2.0:
            lead = x + 1.0
            d = self.ice.zs[self.ice.idx(lead)] - z_rim
            if d > 0:
                if vx > 0.5:
                    self.engaged = True
                    fx = self.ice.u[self.ice.idx(lead)] * 1e-3 * swept_area(d)
                    fz = cfg.K_VERT_PRIOR * fx
                    vol = self.ice.cut(self.x_prev, x, z_rim)
                    self.removed_mm3 += vol
                    self.scoop_mm3 += vol
                    self.d_last = d
                elif not self.engaged:
                    fz = self.K_C * d                  # 터치오프 압입
        if self.engaged and th > cfg.TH_ATTACK + 60.0 and not self.cap_added:
            cap = cfg.CLOSE_CAPTURE * cap_volume(self.d_last)
            self.removed_mm3 += cap
            self.scoop_mm3 += cap
            self.cap_added = True
        if self.engaged and z_rim > max(self.ice.zs) + 1.0:
            self.engaged = False
        # 배출: 컵 위, Z ≥ Z_SAFE, 개구부 아래로
        if th <= -90.0 and abs(x - cfg.X_CUP) < 20.0 and zc >= cfg.Z_SAFE - 1.0 and self.scoop_mm3 > 0:
            self.cup_g += self.scoop_mm3 * cfg.RHO * 1e-3
            self.scoop_mm3 = 0.0
            self.cap_added = False
        # 팬 벽 충돌 감시 (mock 전용 판정)
        bottom = self.z_act - cfg.R_SCOOP
        if bottom < 0.0:
            pen = max(x + cfg.R_SCOOP - cfg.PAN_LEN, -(x - cfg.R_SCOOP), 0.0)
            self.collision_mm = max(self.collision_mm, pen)
        self.fx, self.fz = fx, fz
        self.x_prev = x
        # 플랫폼이 재는 힘: 절삭 + 관성(m·a_bed), 수직은 접촉 − 팬에서 빠진 무게
        f_plat_x = fx + self.M_PLAT_TRUE * g.acc[0] * 1e-3
        f_plat_z = fz - self.removed_mm3 * cfg.RHO * 1e-3 * 1e-3 * G
        self.hist.append((self.clock(), f_plat_x, f_plat_z))
        # 80 SPS 샘플
        now = self.clock()
        if now + 1e-12 >= self.next_t:
            self.next_t += self.TS
            rx, rz = self._raw()
            mx = rx - self.tare[0] + self.rng.gauss(0, self.NOISE)
            mz = rz - self.tare[1] + self.rng.gauss(0, self.NOISE)
            if self.probe_armed and mz >= cfg.F_TOUCH:
                self.touch = True
            if self.permit_armed:
                if abs(mx) >= self.lim[0] or mz >= self.lim[1]:
                    self.latch |= FL_OVERLOAD
                if now - self.last_hb > cfg.T_WD:
                    self.latch |= FL_WATCHDOG
                if self.r7_override and self.z_high():
                    self.r7_override = False
                if not (self.z_high() or self.x_win() or self.r7_override):
                    self.latch |= FL_INTERLOCK
            fl = self.latch
            fl |= FL_TOUCH if self.touch else 0
            fl |= FL_PERMIT if self.permit() else 0
            fl |= FL_DEADMAN if self.deadman else 0
            fl |= FL_LANE_PIN if self.lane_pin else 0
            fl |= FL_Z_HIGH if self.z_high() else 0
            fl |= FL_X_WIN if self.x_win() else 0
            self.out.append(f"F,{now*1e3:.0f},{mx:.3f},{mz:.3f},{fl}")


class MockRig:
    DT = 0.0025

    def __init__(self, cfg, scenario):
        self.t = 0.0
        clock = self.now
        self.ice = Ice(cfg, scenario)
        self.grbl_port = MockGrblPort(clock, cfg)
        self.nano_port = MockNanoPort(clock, self.grbl_port, self.ice, cfg)
        self.grbl_port.door_open = self.nano_port.door_open
        self.grbl_port.probe_trig = self.nano_port.probe_triggered

    def now(self):
        return self.t

    def step(self):
        self.t += self.DT
        self.grbl_port.advance(self.DT)
        self.nano_port.advance(self.DT)

    def operator_confirm(self, msg):
        print(f"  [mock 작업자] {msg} → 확인 (2 s)")
        for _ in range(int(2.0 / self.DT)):
            self.step()
        return True

    def summary(self):
        n = self.nano_port
        return {"cup_g": n.cup_g, "removed_g": n.removed_mm3 * Cfg.RHO * 1e-3,
                "collision_mm": n.collision_mm, "grbl_events": self.grbl_port.events}


# =====================================================================================
# 로그
# =====================================================================================
class Logger:
    COLS = ["t_ms", "nano_ms", "state", "event", "X", "Z", "theta", "Fx", "Fz", "grbl", "flags", "lane_y", "Z0", "d_cmd", "note"]

    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.fh = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.fh)
        self.w.writerow(self.COLS)
        self.events = []

    def row(self, **kw):
        self.w.writerow([kw.get(c, "") for c in self.COLS])
        if kw.get("event"):
            self.events.append(kw)

    def close(self):
        self.fh.close()


# =====================================================================================
# 호스트 상태기계
# =====================================================================================
TRAVEL, SCOOP = "TRAVEL", "SCOOP"
STOPPED = ("Idle", "Hold:0", "Door:0", "Door:1", "Alarm")


class Host:
    def __init__(self, rig, cfg, log, scenario="normal"):
        self.rig, self.cfg, self.log, self.scenario = rig, cfg, log, scenario
        self.grbl = GrblClient(rig.grbl_port, rig.now)
        self.nano = NanoClient(rig.nano_port, rig.now)
        self.state = "BOOT"
        self.in_fault = False
        self.plan = {"x": None, "z": None, "th": None}
        self.t_hb = self.t_poll = -1.0
        self.n_sample = 0
        self.sample_hook = None
        self.lane_y = None
        self.z0 = None
        self.d_cmd = None
        self.u_hat = cfg.U_PRIOR
        self.k_f = 1.0
        self.fmax = 0.0
        self.blocked = 0
        self.frozen_once = False
        self._seg_done = 0
        self.t_last_tx = -1.0
        self.grace = -1.0
        self.result = {}
        self.t_state = {}

    # ---------------------------------------------------------------- 공통
    def now(self):
        return self.rig.now()

    def enter(self, state, event=None, note=""):
        self.state = state
        self.t_state.setdefault(state, self.now())
        self._log(event or f"ENTER_{state}", note)
        print(f"  t={self.now():7.3f} s  → {state:12s} {note}")

    def _log(self, event="", note="", fx="", fz="", flags="", nano_ms=""):
        s = self.grbl.st
        self.log.row(t_ms=f"{self.now()*1e3:.0f}", nano_ms=nano_ms, state=self.state, event=event, X=f"{s['x']:.2f}",
                     Z=f"{s['z']:.2f}", theta=f"{s['th']:.1f}", Fx=fx, Fz=fz, grbl=s["state"], flags=flags,
                     lane_y="" if self.lane_y is None else self.lane_y,
                     Z0="" if self.z0 is None else f"{self.z0:.3f}",
                     d_cmd="" if self.d_cmd is None else f"{self.d_cmd:.2f}", note=note)

    def pump(self):
        """한 루프: 물리/직렬 → 하트비트 → 상태 폴링 → 힘 감시 → 결함 판정."""
        # hang 시나리오: 드래그 중 노트북이 1 s 멈춘다 (물리는 계속, 호스트 처리 없음)
        if self.scenario == "hang" and self.state == "DRAG" and not self.frozen_once and self._seg_done >= 2:
            self.frozen_once = True
            self._log("HOST_FREEZE_1S", "노트북 멈춤 모의: 하트비트·감시 중단")
            t_end = self.now() + 1.0
            while self.now() < t_end:
                self.rig.step()
        self.rig.step()
        self.grbl.pump()
        self.nano.pump()
        now = self.now()
        if now - self.t_hb >= self.cfg.HB_PERIOD:
            self.nano.cmd("H")
            self.t_hb = now
        if now - self.t_poll >= self.cfg.STATUS_PERIOD:
            self.grbl.realtime(b"?")
            self.t_poll = now
        while self.nano.new:
            t, fx, fz, fl, ms = self.nano.new.popleft()
            self.n_sample += 1
            if self.sample_hook:
                self.sample_hook(t, fx, fz)
            if self.n_sample % self.cfg.LOG_DECIM == 0:
                self._log("", "", f"{fx:.2f}", f"{fz:.2f}", fl, ms)
            if not self.in_fault:
                self._check_force(fx, fz)
        if self.in_fault:
            return
        # 결함 판정 (우선순위: 알람 → Nano 래치 → 도어 → 통신)
        if self.grbl.alarms:
            code = self.grbl.alarms.popleft()
            raise Fault(f"GRBL_ALARM_{code}", {2: "소프트 리밋", 3: "움직이는 중 리셋", 4: "프로브 초기 트리거",
                                               5: "프로브 접촉 없음(NO_SURFACE)"}.get(code, ""))
        for bit, code in LATCHES.items():
            if self.nano.flags & bit:
                raise Fault(code, "Nano 하드웨어 경로가 GRBL 도어 입력을 끊음")
        if self.grbl.st["state"].startswith("Door") and self.state not in ("HOME", "SELECT_LANE"):
            raise Fault("DOOR_HOLD", "허가선 끊김(데드맨·레인 핀·Nano)")
        if now < self.grace:
            return
        if self.grbl.st_t > 0 and now - self.grbl.st_t > self.cfg.GRBL_TIMEOUT:
            raise Fault("COMM_GRBL", "상태 응답 없음")
        if self.nano.last_t > 0 and now - self.nano.last_t > self.cfg.NANO_TIMEOUT:
            raise Fault("COMM_NANO", "힘 샘플 없음")

    def _check_force(self, fx, fz):
        c = self.cfg
        if self.state in ("DIVE", "DRAG", "CLOSE"):
            lim = c.F_STOP_HOST
        elif self.state in ("TOUCHOFF", "LIFT", "TO_CUP", "SELECT_LANE", "EJECT"):
            lim = c.F_TRAVEL + c.M_PLAT * c.A_X * 1e-3          # 관성 여유
        else:
            return
        f = max(abs(fx), fz)
        self.fmax = max(self.fmax, abs(fx)) if self.state in ("DIVE", "DRAG") else self.fmax
        if f > lim:
            self.grbl.realtime(b"!")                              # 먼저 멈추고 그다음 기록
            raise Fault("OVERLOAD_HOST", f"|F|={f:.1f} N > {lim:.1f} N in {self.state}")

    def wait_until(self, pred, timeout, what):
        t_end = self.now() + timeout
        while not pred():
            if self.now() > t_end:
                raise Fault(f"TIMEOUT_{what}")
            self.pump()

    def send(self, line, timeout=60.0):
        """send-response: 'ok'를 받을 때까지 감시하며 기다린다(G4·G38.2·$H는 동작이 끝나야 ok)."""
        self.grbl.send_line(line)
        self.t_last_tx = self.now()
        self._log("TX", line)
        self.wait_until(lambda: bool(self.grbl.acks), timeout, "ACK")
        ack = self.grbl.acks.popleft()
        if ack != "ok":
            raise Fault("GRBL_ERROR", f"{line} → {ack}")

    def sync(self):
        """G4 P0.01: 앞선 모든 동작이 끝나야 ok가 온다 (GRBL 동기화 관용구)."""
        self.send("G4 P0.01")
        self.wait_until(lambda: self.grbl.st["state"] == "Idle", 2.0, "IDLE")

    def fresh_status(self):
        t0 = self.now()
        self.wait_until(lambda: self.grbl.st_t > t0, 1.0, "STATUS")
        return self.grbl.st

    # ---------------------------------------------------------------- R7 게이트
    def _in_pan_workspace(self, x, z):
        c = self.cfg
        return c.X_LANE[0] - 1e-6 <= x <= c.X_LANE[1] + 1e-6 and z >= c.Z_LIM[0]

    def request_xy_move(self, x, z=None, kind=TRAVEL, feed=None):
        """수평 이동 요청은 모두 여기로 (R7). 수평 모터는 X(이동 베드) 하나, Y는 수동 레인 핀.

        TRAVEL : 계획 Z와 측정 Z가 모두 Z_SAFE 이상일 때만.
        SCOOP  : DIVE/DRAG 상태이고 목표가 팬 레인 작업공간 안일 때만 (Z < Z_SAFE 허용).
        거부되면 EV_XY_BLOCKED_LOW_Z를 기록하고 False.
        """
        c = self.cfg
        z_tgt = self.plan["z"] if z is None else z
        z_meas = self.fresh_status()["z"]
        z_low = min(self.plan["z"], z_meas)
        ok = False
        if kind == TRAVEL and z is None and z_low >= c.Z_SAFE:
            ok = True
        elif kind == SCOOP and self.state in ("DIVE", "DRAG") and self._in_pan_workspace(x, z_tgt) \
                and self._in_pan_workspace(self.plan["x"], self.plan["z"]):
            ok = True
        if not ok:
            self.blocked += 1
            self._log("EV_XY_BLOCKED_LOW_Z", f"x→{x:.1f} kind={kind} z_plan={self.plan['z']:.2f} z_meas={z_meas:.2f}")
            print(f"  t={self.now():7.3f} s    EV_XY_BLOCKED_LOW_Z: X→{x:.0f} ({kind}) 거부, Z={z_low:.1f} < Z_SAFE={c.Z_SAFE:.0f}")
            return False
        v = feed or (c.V_TRAVEL if kind == TRAVEL else c.V_DRAG)
        words = f"X{x:.3f}" + (f" Z{z:.3f}" if z is not None else "")
        self.send(f"G1 {words} F{v*60:.0f}")
        self.plan["x"] = x
        if z is not None:
            self.plan["z"] = z
        return True

    def move_x(self, *a, **kw):
        if not self.request_xy_move(*a, **kw):
            raise Fault("XY_BLOCKED_LOW_Z", "R7 게이트 거부")

    def move_z(self, z, v):
        self.send(f"G1 Z{z:.3f} F{v*60:.0f}")        # Z 단독 이동은 항상 허용(소프트 리밋 안)
        self.plan["z"] = z

    def move_th(self, th, duration=None):
        if duration:
            self.send(f"G93 G1 Y{th:.2f} F{60.0/duration:.3f}")   # G93: θ와 mm 단위 섞임 회피, 시간 지정
            self.send("G94")
        else:
            self.send(f"G1 Y{th:.2f} F{self.cfg.V_TH*60:.0f}")
        self.plan["th"] = th

    def confirm(self, msg):
        """작업자 확인. 기다리는 동안 호스트가 감시를 못 하므로 허가선은 미리 끊어 둔다(E0)."""
        ok = self.rig.operator_confirm(msg)
        self.grace = self.now() + 0.3                        # 확인 직후 상태·샘플이 새로 올 때까지
        return ok

    def arm_permit(self):
        self.nano.cmd("E1")
        self.wait_until(lambda: bool(self.nano.flags & FL_PERMIT), 1.0, "PERMIT")
        self.pump(); self.pump()
        if self.grbl.st["state"].startswith("Door"):
            self.grbl.realtime(b"~")                        # 도어 닫힘 + 사이클 시작 → 복귀
            self.wait_until(lambda: not self.grbl.st["state"].startswith("Door"), 1.0, "DOOR_CLEAR")

    # ---------------------------------------------------------------- 상태
    def st_home(self):
        self.enter("HOME")
        self.wait_until(lambda: self.grbl.banner and self.nano.last_t > 0, 5.0, "BOOT")
        self.nano.cmd(f"L{self.cfg.F_STOP_HW:.1f},{self.cfg.F_STOP_HW:.1f}")
        self.arm_permit()
        self.send("$H", timeout=60.0)                       # Z 먼저 → X → θ (GRBL HOMING_CYCLE_0..2)
        self.send("G21 G90 G94")
        st = self.fresh_status()
        self.plan = {"x": st["x"], "z": st["z"], "th": st["th"]}
        self._log("HOMED", f"X={st['x']:.1f} Z={st['z']:.1f} θ={st['th']:.1f}")

    def st_select_lane(self, lane_y):
        self.enter("SELECT_LANE", note=f"y={lane_y:+.0f}")
        st = self.fresh_status()
        if min(st["z"], self.plan["z"]) < self.cfg.Z_SAFE:
            self.move_z(self.cfg.Z_SAFE + 10.0, self.cfg.V_Z_FAST)
            self.sync()
        self.nano.cmd("E0")                                  # 사람이 손을 넣는 동안 모든 축 허가 해제
        if not self.confirm(f"레인 핀을 y={lane_y:+.0f} mm 구멍에 꽂으세요"):
            raise Fault("OPERATOR_ABORT")
        self.wait_until(lambda: bool(self.nano.flags & FL_LANE_PIN), 2.0, "LANE_PIN")
        self.arm_permit()
        self.lane_y = lane_y

    def st_touchoff(self):
        c = self.cfg
        self.enter("TOUCHOFF")
        self.move_th(c.TH_ATTACK)
        self.move_x(c.X_LANE[0], kind=TRAVEL)              # Z ≥ Z_SAFE에서만 통과
        z_est = c.SURFACE_EST + B_ATTACK                    # 표면에 rim 최저점이 닿을 때의 C 높이
        self.move_z(z_est + c.PROBE_MARGIN, c.V_Z_FAST)
        self.sync()
        self.send("G4 P0.3")                                # 베드 진동 정착
        self.nano.cmd("T")                                  # 영점(팬 무게 포함)
        self.nano.cmd("P1")                                 # 프로브 출력 무장
        self.pump(); self.pump()
        self.grbl.prb = None
        self.send(f"G38.2 Z{z_est - c.PROBE_OVERTRAVEL:.3f} F{c.V_PROBE*60:.0f}")   # 트리거 순간 위치 → PRB
        self.nano.cmd("P0")
        if not self.grbl.prb or not self.grbl.prb[3]:
            raise Fault("NO_SURFACE")
        z_trig = self.grbl.prb[2]
        self.z0 = z_trig + c.DELTA_BIAS                     # 늦은 트리거만큼 실제 접촉은 위
        st = self.fresh_status()
        self.plan["z"] = st["z"]
        surf = self.z0 - B_ATTACK
        self._log("TOUCH", f"Z_trig={z_trig:.3f} Z0={self.z0:.3f} 표면={surf:.2f} (추정 {c.SURFACE_EST:.1f})")
        print(f"             Z0 = {self.z0:.3f} mm → 표면 {surf:.2f} mm (추정 {c.SURFACE_EST:.1f}, 차 {surf - c.SURFACE_EST:+.2f})")

    # ---- 깊이 계획
    def _comp(self, fx, fz):
        return self.cfg.C_ZZ * fz + self.cfg.C_ZX * fx      # 스쿱이 위로 밀리는 양 [mm]

    def st_dive_drag(self):
        c = self.cfg
        self.enter("DIVE")
        d0 = max(c.D_MIN, min(D_PORTION, depth_for_force(c.F_TARGET, self.u_hat), c.D_MAX))
        f_pred = self.u_hat * 1e-3 * swept_area(d0)
        comp0 = self._comp(f_pred, c.K_VERT_PRIOR * f_pred)
        x0 = c.X_LANE[0]
        x_dive = x0 + d0 / math.tan(math.radians(c.BETA_DIVE))
        self.d_cmd = d0
        self.move_x(x_dive, z=self.z0 - d0 - comp0, kind=SCOOP)
        vol_est = 0.5 * swept_area(d0) * (x_dive - x0)
        segs = []
        self._seg_done = 0
        u_meas_all = []

        def queue(d, fx_pred, kvert, x_from, last_hint=False):
            nonlocal vol_pred
            comp = self._comp(fx_pred, kvert * fx_pred)
            length = min(c.SEG, c.X_LANE[1] - x_from)
            need = V_TARGET - c.CLOSE_CAPTURE * cap_volume(d) - vol_pred
            last = False
            if need <= swept_area(d) * length:
                length = max(1.0, need / max(swept_area(d), 1e-6))
                last = True
            if c.X_LANE[1] - (x_from + length) < 1.0:
                last = True
            seg = {"x0": x_from, "x1": x_from + length, "d": d, "comp": comp, "fx": [], "fz": [], "last": last}
            self.move_x(seg["x1"], z=self.z0 - d - comp, kind=SCOOP)
            vol_pred += swept_area(d) * length
            segs.append(seg)
            self._log("SEG_QUEUED", f"x {seg['x0']:.1f}→{seg['x1']:.1f} d={d:.2f} comp={comp:.3f}{' LAST' if last else ''}")
            return seg

        def on_sample(t, fx, fz):
            st = self.grbl.st
            x = st["x"] + st["v"] * (t - self.grbl.st_t) if st["state"] == "Run" else st["x"]
            if self.state == "DIVE" and x >= x_dive:
                self.enter("DRAG")
            if x < x_dive + c.INERTIA_MASK:
                return                                        # 가속·정착 구간은 평균에서 제외
            for s in segs:
                if s["x0"] <= x < s["x1"] and "done" not in s:
                    s["fx"].append(fx); s["fz"].append(fz)

        vol_pred = vol_est
        self.sample_hook = on_sample
        queue(d0, f_pred, c.K_VERT_PRIOR, x_dive)
        for _ in range(c.LOOKAHEAD):
            if not segs[-1]["last"]:
                queue(d0, f_pred, c.K_VERT_PRIOR, segs[-1]["x1"])
        # 구간 완료 → 측정 → 다음 구간 결정 (bounded lookahead)
        idx = 0
        while idx < len(segs):
            s = segs[idx]
            self.wait_until(lambda: self.grbl.st["x"] >= s["x1"] - 0.05 or
                            (self.grbl.st["state"] == "Idle" and self.grbl.st_t > self.t_last_tx + 0.02), 30.0, "SEG")
            s["done"] = True
            self._seg_done += 1
            if s["fx"]:
                fx = sum(s["fx"]) / len(s["fx"]); fz = sum(s["fz"]) / len(s["fz"])
            else:
                fx = self.u_hat * 1e-3 * swept_area(s["d"]); fz = c.K_VERT_PRIOR * fx
            d_est = s["d"] + s["comp"] - self._comp(fx, fz)
            vol_est += swept_area(max(d_est, 0.0)) * (s["x1"] - s["x0"])
            u_meas = fx / max(swept_area(max(d_est, 0.1)), 1e-6) * 1e3
            kvert = fz / fx if fx > 5.0 else c.K_VERT_PRIOR
            u_meas_all.append(u_meas)
            self._log("SEG_DONE", f"x1={s['x1']:.1f} n={len(s['fx'])} Fx={fx:.1f} Fz={fz:.1f} d_est={d_est:.2f} "
                                  f"u={u_meas:.0f}kPa V={vol_est/1e3:.1f}cm3")
            if s["last"]:
                break
            if not segs[-1]["last"]:
                d_prev = segs[-1]["d"]
                d_new = min(D_PORTION, depth_for_force(c.F_TARGET, u_meas), c.D_MAX)
                d_new = max(c.D_MIN, d_prev - c.DD_MAX, min(d_new, d_prev + c.DD_MAX))
                self.d_cmd = d_new
                queue(d_new, u_meas * 1e-3 * swept_area(d_new), kvert, segs[-1]["x1"])
            idx += 1
        self.sample_hook = None
        self.sync()
        cap = c.CLOSE_CAPTURE * cap_volume(segs[-1]["d"])
        self.result.update(vol_est_cm3=(vol_est + cap) / 1e3, n_seg=len(segs), fmax=self.fmax,
                           u_meas=sum(u_meas_all) / len(u_meas_all) if u_meas_all else None,
                           d_list=[round(s["d"], 2) for s in segs])
        if u_meas_all:
            u_bar = sum(u_meas_all) / len(u_meas_all)
            self.u_hat += c.LAMBDA_U * (u_bar - self.u_hat)   # 스트로크 간 적응 (다음 주문 첫 깊이)

    def st_close(self):
        self.enter("CLOSE")
        self.move_th(self.cfg.TH_CAPTURE, duration=self.cfg.T_CLOSE)
        self.sync()

    def st_lift(self):
        self.enter("LIFT")
        self.move_z(self.cfg.Z_SAFE + 10.0, self.cfg.V_Z_FAST)
        self.sync()
        st = self.fresh_status()
        if st["z"] < self.cfg.Z_SAFE:
            raise Fault("LIFT_INCOMPLETE")

    def st_to_cup(self):
        self.enter("TO_CUP")
        self.move_x(self.cfg.X_CUP, kind=TRAVEL)
        self.sync()

    def st_eject(self):
        c = self.cfg
        self.enter("EJECT")
        self.move_th(c.TH_EJECT)
        for k in range(3):                                   # 흔들기 ±8°
            self.move_th(c.TH_EJECT + 8.0)
            self.move_th(c.TH_EJECT - 8.0)
        self.move_th(c.TH_EJECT)
        self.sync()
        self.send("G4 P0.3")
        self.move_th(c.TH_ATTACK)
        self.sync()

    def st_weigh(self):
        c = self.cfg
        self.enter("WEIGH")
        self.send(f"G4 P{c.T_SETTLE:.1f}")                   # 베드 정지 후 정착
        buf = []
        self.sample_hook = lambda t, fx, fz: buf.append(fz)
        t_end = self.now() + 0.5
        self.wait_until(lambda: self.now() >= t_end, 1.0, "WEIGH")
        self.sample_hook = None
        m = -sum(buf) / len(buf) / G * 1e3                  # g, 영점 이후 팬에서 빠진 질량
        m_pred = self.result.get("vol_est_cm3", 0.0) * c.RHO
        if m_pred > 0:
            self.k_f += c.LAMBDA_K * (m / m_pred - self.k_f)
        verdict = "OK" if -0.05 <= m / c.M_TARGET - 1 <= 0.10 else "범위 밖(A35 −5/+10 %)"
        self.result.update(m_meas=m, m_pred=m_pred, verdict=verdict)
        self._log("PORTION", f"m={m:.1f} g pred={m_pred:.1f} g k_f={self.k_f:.3f} {verdict}")

    # ---------------------------------------------------------------- 결함 처리
    def handle_fault(self, f):
        self.in_fault = True
        self.sample_hook = None
        self.enter("FAULT", event=f"FAULT_{f.code}", note=f.note)
        self.grbl.realtime(b"!")                              # (이미 멈췄으면 무시됨)
        try:
            self.wait_until(lambda: self.grbl.st["state"] in STOPPED, 3.0, "STOP")
            st = self.fresh_status()
            self._log("STOPPED", f"GRBL={st['state']} X={st['x']:.2f} Z={st['z']:.2f}")
            self.result["stop_x"], self.result["stop_z"] = st["x"], st["z"]
            self.nano.cmd("E0")
            self.grbl.banner = False
            self.grbl.realtime(b"\x18")                      # 큐 비우기. 정지 상태면 위치 유지(확인 필요)
            self.wait_until(lambda: self.grbl.banner, 2.0, "RESET")
            st = self.fresh_status()
            lost = bool(self.grbl.alarms) or st["state"] == "Alarm"
            self.grbl.alarms.clear()
            self._log("RESET", "위치 상실 → 재원점 필요" if lost else "위치 유지")
            if not self.confirm(f"결함 {f.code}: 원인 확인 후 복구"):
                return
            self.nano.cmd("R")                                # 래치 해제는 작업자 확인 뒤에만
            self.pump(); self.pump()
            if not self.nano.flags & (FL_Z_HIGH | FL_X_WIN):
                # Z가 낮은 채 창 밖에 멈춤 → 하드웨어 R7 인터록이 모든 축을 막고 있다
                if not self.confirm("R7 인터록 1회 우회: 데드맨을 누른 채 Z 상승만 허용"):
                    return
                self.nano.cmd("U")
                self._log("R7_OVERRIDE", "Z 상승 복구용 1회 우회")
            if lost:
                self.in_fault = False
                self.st_home()                                # Z 먼저 올라간다
            else:
                self.send("G21 G90 G94")
                self.arm_permit()
                st = self.fresh_status()
                self.plan = {"x": st["x"], "z": st["z"], "th": st["th"]}
                if st["z"] < self.cfg.Z_SAFE:
                    self.move_z(self.cfg.Z_SAFE + 10.0, self.cfg.V_Z_FAST)   # Z 단독 상승은 R7과 무관
                    self.sync()
                self.in_fault = False
            self.pump(); self.pump()
            self.enter("IDLE", note="복구 완료 → 재원점 권장")
            self.result["recovered"] = True
        except Fault as f2:
            self._log("RECOVERY_FAILED", f2.code)
            print(f"  복구 실패: {f2.code} — 작업자 수동 조치 필요")
            self.result["recovered"] = False

    # ---------------------------------------------------------------- 한 사이클
    def run(self, lane_y=0.0):
        try:
            self.st_home()
            self.enter("IDLE", note="주문: 싱글 115 g")
            self.st_select_lane(lane_y)
            if self.scenario == "gate":
                self.gate_test_high()
            self.st_touchoff()
            if self.scenario == "gate":
                self.gate_test_low()
            self.st_dive_drag()
            self.st_close()
            self.st_lift()
            self.st_to_cup()
            self.st_eject()
            self.st_weigh()
            self.enter("IDLE", note="사이클 완료")
            self.result["ok"] = True
        except Fault as f:
            self.result["fault"] = f.code
            self.handle_fault(f)

    def gate_test_high(self):
        ok = self.request_xy_move(self.cfg.X_LANE[0], kind=TRAVEL)
        self.sync()
        self.result["gate_high_ok"] = ok

    def gate_test_low(self):
        """(1) 호스트 게이트: Z < Z_SAFE에서 컵으로 이송 요청 → 거부되어야 함.
        (2) 게이트 우회(버그 모의): 원시 G-code를 직접 보냄 → Nano X-창/Z-높이 인터록이 도어를 끊어야 함."""
        ok = self.request_xy_move(self.cfg.X_CUP, kind=TRAVEL)
        self.result["gate_low_rejected"] = not ok
        self._log("GATE_BYPASS_TEST", "게이트를 거치지 않고 G1 X430 전송(버그 모의)")
        print("             게이트 우회 시험: G1 X430 직접 전송 (버그 모의)")
        self.grbl.send_line(f"G1 X{self.cfg.X_CUP:.1f} F{self.cfg.V_TRAVEL*60:.0f}")
        self.wait_until(lambda: False, 30.0, "BYPASS_NOT_STOPPED")


# =====================================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description="V1-L host skeleton")
    ap.add_argument("--mock", action="store_true", help="가짜 GRBL·Nano로 실행")
    ap.add_argument("--scenario", default="normal", choices=["normal", "overload", "hang", "gate"])
    ap.add_argument("--grbl", help="GRBL 포트 (예: COM3, /dev/ttyACM0)")
    ap.add_argument("--nano", help="Nano 포트")
    ap.add_argument("--lane", type=float, default=0.0, help="레인 y [mm]: -38 / 0 / 38")
    ap.add_argument("--log", help="CSV 로그 경로")
    a = ap.parse_args(argv)
    cfg = Cfg()
    here = os.path.dirname(os.path.abspath(__file__))
    log_path = a.log or os.path.join(here, "logs", f"host_{'mock' if a.mock else 'real'}_{a.scenario}.csv")
    if a.mock:
        rig = MockRig(cfg, a.scenario)
    else:
        if not (a.grbl and a.nano):
            ap.error("실제 모드는 --grbl 과 --nano 가 필요하다 (또는 --mock)")
        if a.scenario != "normal":
            ap.error("시나리오는 mock 전용")
        rig = RealRig(a.grbl, a.nano)
    log = Logger(log_path)
    print(f"V1-L host skeleton  mode={'mock' if a.mock else 'real'}  scenario={a.scenario}")
    print(f"  d_portion={D_PORTION:.2f} mm (레인 {cfg.X_LANE[1]-cfg.X_LANE[0]:.0f} mm), V_target={V_TARGET/1e3:.1f} cm³")
    host = Host(rig, cfg, log, a.scenario)
    t_wall = time.time()
    host.run(a.lane)
    log.close()
    r, s = host.result, rig.summary()
    print("-" * 72)
    print(f"결과: {'사이클 완료' if r.get('ok') else 'FAULT ' + r.get('fault', '?')}   "
          f"(모의 시간 {host.now():.1f} s, 실행 {time.time()-t_wall:.1f} s)")
    if "d_list" in r:
        print(f"  구간 {r['n_seg']}개, 명령 깊이 {r['d_list']} mm, 드래그 |Fx|max {r['fmax']:.1f} N, "
              f"측정 u {r['u_meas']:.0f} kPa → 다음 û {host.u_hat:.0f} kPa")
    if "m_meas" in r:
        print(f"  portion: 팬 질량 차 {r['m_meas']:.1f} g (예측 {r['m_pred']:.1f} g, 목표 {cfg.M_TARGET:.0f} g) {r['verdict']}, "
              f"k_f → {host.k_f:.3f}")
    if s:
        print(f"  mock 참값: 컵 {s['cup_g']:.1f} g, 팬에서 빠짐 {s['removed_g']:.1f} g, 팬 벽 침범 {s['collision_mm']:.2f} mm")
        for (t, ev, pos) in s["grbl_events"]:
            print(f"  GRBL 이벤트 t={t:.3f} s {ev} X={pos[0]:.2f} Z={pos[2]:.2f}")
    if "stop_x" in r:
        print(f"  정지 위치 X={r['stop_x']:.2f} mm")
    print(f"  R7 게이트 거부 {host.blocked}회" + (f", 저Z 이송 거부={r.get('gate_low_rejected')}" if a.scenario == "gate" else ""))
    print(f"  로그: {log_path} ({len(log.events)} 이벤트)")
    return 0 if (r.get("ok") or a.scenario != "normal") else 1


if __name__ == "__main__":
    sys.exit(main())
