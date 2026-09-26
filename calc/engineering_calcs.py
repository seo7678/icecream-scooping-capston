"""Parametric sizing for the scoop head and the V1 research rig.

Run:  python3 calc/engineering_calcs.py
Out:  calc/output/engineering_calcs.md

Every load in here is an ASSUMPTION CASE (50/100/150/200 N), not a measurement.
The script exists so the team can re-run the whole sizing chain in one command
once P0/P1 experiments give real numbers (edit the CASES block only).
"""

import math
import os

import scoop_model as sm

OUT = os.path.join(os.path.dirname(__file__), "output", "engineering_calcs.md")

# ---------------------------------------------------------------- CASES ------
F_CASES_N = (50.0, 100.0, 150.0, 200.0)   # ASSUMPTION: drag force at the rim
V_CASES_MM_S = (50.0, 100.0)               # feed speed
STROKE_MM = 200.0                          # sweep length (tub chord limited)
DTHETA_CLOSE_DEG = 120.0                   # closing rotation
T_CLOSE_S = 0.8                            # closing time target
SF_MOTOR = 1.5                             # torque margin on catalogue rating

E_STEEL = 193e3      # MPa, 304 stainless
SY_304 = 205.0       # MPa, 304 annealed min yield (ASTM A276 class) -- check cert
E_AL = 69e3          # MPa, 6063-T5 profile
I_4040_MM4 = 8.0e4   # ASSUMPTION: 40x40 profile Ix ~ 8 cm^4; take from vendor catalogue
DESIGN_STRESS_FATIGUE = 50.0  # MPa nominal target for >1e6 cycles with Kf~2 (see docs/13)

lines = []


def p(s=""):
    lines.append(s)


def table(header, rows):
    p("| " + " | ".join(header) + " |")
    p("|" + "|".join("---" for _ in header) + "|")
    for r in rows:
        p("| " + " | ".join(r) + " |")
    p()


def screw_efficiency(lead_mm, d_mean_mm, mu, half_angle_deg=15.0):
    """Driving efficiency of a trapezoidal / ACME screw (standard power-screw formula)."""
    lam = math.atan(lead_mm / (math.pi * d_mean_mm))
    mu_eff = mu / math.cos(math.radians(half_angle_deg))
    phi = math.atan(mu_eff)
    eta = math.tan(lam) / math.tan(lam + phi)
    self_locking = lam <= phi
    return eta, math.degrees(lam), math.degrees(phi), self_locking


p("# Engineering calculations — generated output")
p()
p("> 생성: `python3 calc/engineering_calcs.py`. 모든 하중은 **ASSUMPTION CASE**이며 측정값이 아니다.")
p("> 측정값(P0-3, P1-1)이 나오면 스크립트 상단 CASES 블록만 바꾸고 다시 실행한다.")
p()

# ------------------------------------------------------------ 1. portion -----
p("## 1. Portion geometry → required stroke")
p()
p("V_ball = m/ρ, A(d) = 원형 segment 면적 (θ=0, 깊이 d = R − h), L = V/A (packing factor 1.0 가정)")
p()
rows = []
for rho in sm.DENSITY_CASES_G_CM3:
    v_mm3 = sm.ball_volume_cm3(sm.TARGET_MASS_G, rho) * 1e3
    for d in (15.0, 20.0, 25.0, 30.0):
        a = sm.swept_area_mm2(0.0, sm.R_BOWL_MM - d)
        rows.append([f"{rho:.2f}", f"{v_mm3/1e3:.0f}", f"{d:.0f}", f"{a:.0f}", f"{v_mm3/a:.0f}"])
table(["ρ [g/cm³]", "V for 115 g [cm³]", "depth d [mm]", "A [mm²]", "stroke L [mm]"], rows)
p("해석: 내경 ~230 mm 통에서 입구/출구 여유를 빼면 쓸 수 있는 직선 stroke는 ~180–200 mm이다(ASSUMPTION, 통 실측 필요).")
p("따라서 R=35 mm 스쿱으로 115 g을 한 번에 뜨려면 d ≥ 20 mm 수준이 필요하다.")
p()

# ------------------------------------------------------------ 2. feed axis ---
p("## 2. Feed (sweep) axis — drive options")
p()
p("리드스크류 토크: T = F·p / (2π·η). 벨트/피니언: T = F·r / η. 기계출력 P = F·v.")
p()
drives = []
eta_t88, lam, phi, lock = screw_efficiency(8.0, 7.0, 0.15)
drives.append(("Tr8x8 (4-start) + POM nut", 8.0, eta_t88, f"λ={lam:.1f}°, φ'={phi:.1f}°, self-lock={'yes' if lock else 'no'}"))
eta_t102, lam, phi, lock = screw_efficiency(2.0, 9.0, 0.15)
drives.append(("Tr10x2 (1-start) + POM nut", 2.0, eta_t102, f"λ={lam:.1f}°, φ'={phi:.1f}°, self-lock={'yes' if lock else 'no'}"))
drives.append(("Ball screw SFU1605", 5.0, 0.90, "η 0.9 (catalogue typical)"))
drives.append(("Ball screw SFU1610", 10.0, 0.90, "η 0.9 (catalogue typical)"))
p("| Drive | lead [mm] | η | note |")
p("|---|---|---|---|")
for name, lead, eta, note in drives:
    p(f"| {name} | {lead:.0f} | {eta:.2f} | {note} |")
p("| GT2/HTD belt, pulley r = 9.55 mm | – | 0.95 | backdrivable |")
p()
rows = []
for F in F_CASES_N:
    for v in V_CASES_MM_S:
        row = [f"{F:.0f}", f"{v:.0f}", f"{F*v/1e3:.1f}"]
        for name, lead, eta, _ in drives:
            T = F * lead / (2 * math.pi * eta) / 1e3  # N·m
            n = v / lead * 60.0
            row.append(f"{T:.3f} N·m @ {n:.0f} rpm")
        T_belt = F * 9.55 / 0.95 / 1e3
        n_belt = v / (2 * math.pi * 9.55) * 60.0
        row.append(f"{T_belt:.2f} N·m @ {n_belt:.0f} rpm")
        rows.append(row)
table(["F [N]", "v [mm/s]", "P_mech [W]"] + [d[0] for d in drives] + ["belt r=9.55"], rows)
p("선정 논리:")
p("- 수평 sweep 축은 **backdrivable**해야 한다(전원 차단 시 작업자가 손으로 밀어 뺄 수 있어야 함 → FMEA). Tr10x2는 self-locking이므로 sweep 축에는 부적합, Z축 후보.")
p("- SFU1610 볼스크류: 200 N·100 mm/s에서도 ~0.35 N·m @ 600 rpm → 24 V 60 W급 DC/BLDC 모터로 충분(여유 ≥ 2배).")
p("- 스테퍼 + 볼스크류는 힘 한계가 모터 전류가 아니라 탈조 토크로 정해지고 그 값이 1 kN을 넘을 수 있다(아래 3절) → **V1 연구 rig에서만 사용하고 기계식 과부하 요소 필수**.")
p()

# ------------------------------------------------ 3. stepper force ceiling ---
p("## 3. 안전 관점 — 구동계가 낼 수 있는 최대 힘")
p()
rows = []
for name, lead, eta in (("SFU1605 + NEMA23 (1.2 N·m usable)", 5.0, 0.9),
                        ("SFU1610 + NEMA23 (1.2 N·m usable)", 10.0, 0.9),
                        ("SFU1610 + DC 60 W, current-limited 0.354 N·m", 10.0, 0.9)):
    T = 1.2 if "NEMA" in name else 0.354
    Fmax = 2 * math.pi * eta * T / (lead / 1e3)
    rows.append([name, f"{Fmax:.0f}"])
table(["Drive", "F_max at nut [N]"], rows)
p("ISO/TS 15066의 손/손가락 준정적 접촉 한계는 140 N이다(research/standards.md). 스테퍼+볼스크류는 이 값의 수 배를 낼 수 있으므로,")
p("제품에서는 **전류제한된 DC/BLDC**로 힘 상한을 물리적으로 묶고(F_max ≈ 200 N, 절삭 요구치 기준), 사람 손이 작업영역에 없도록 hold-to-run + 가드로 분리한다.")
p("200 N은 140 N을 넘기 때문에 '힘 제한만으로 안전'이라는 주장은 **불가**하다 → 분리(guarding) 전략이 1차 방호다(docs/19).")
p()

# ------------------------------------------------------------ 4. pitch axis --
p("## 4. Pitch (θ) axis torque")
p()
p("Drag 중 유지토크: τ_hold = F_x · z_c (z_c = 절삭 segment 도심의 회전축 아래 거리) + F_z · x_c(θ=0에서 ≈0).")
p("Closing 토크(상한 추정): τ_close ≈ F_close · R, F_close = F_x (ASSUMPTION: 같은 크기).")
p()
rows = []
for R in sm.R_BOWL_CASES_MM:
    d = 20.0
    h = R - d
    zc = sm.segment_centroid_below_centre_mm(0.0, h, R)
    for F in F_CASES_N:
        t_hold = F * zc / 1e3
        t_close = F * R / 1e3
        rows.append([f"{R:.0f}", f"{d:.0f}", f"{zc:.1f}", f"{F:.0f}", f"{t_hold:.2f}", f"{t_close:.2f}",
                     f"{SF_MOTOR*max(t_hold, t_close):.1f}"])
table(["R [mm]", "d [mm]", "z_c [mm]", "F [N]", "τ_hold [N·m]", "τ_close [N·m]", "rated ≥ [N·m]"], rows)
omega = math.radians(DTHETA_CLOSE_DEG) / T_CLOSE_S
p(f"속도: {DTHETA_CLOSE_DEG:.0f}°를 {T_CLOSE_S} s에 → ω = {omega:.2f} rad/s = {omega*60/(2*math.pi):.0f} rpm.")
p(f"R=35 mm, F=150 N → τ ≈ 5.3 N·m, P = τω ≈ {5.25*omega:.0f} W → 정격 ≥ 8 N·m @ ≥ 30 rpm 기어드모터(24 V, 40–60 W급).")
p()

# ------------------------------------------------ 5. push rod (stem drive) ---
p("## 5. Pitch 전달: 평행링크 push-rod (스템 외부 노출)")
p()
p("θ 모터는 스템 상단(비식품영역), 스쿱 피벗은 스템 하단. 상/하 레버 길이 l = 25 mm인 평행사변형 링크(push-rod는 스템 밖에 노출).")
p("rod 축력 = τ / l, 좌굴(양단 핀) P_cr = π²EI/L².")
p()
rows = []
for tau in (3.0, 5.25, 8.0):
    Frod = tau / 0.025
    for d_rod in (6.0, 8.0):
        I = math.pi * d_rod ** 4 / 64
        for L in (300.0,):
            Pcr = math.pi ** 2 * E_STEEL * I / L ** 2
            rows.append([f"{tau:.2f}", f"{Frod:.0f}", f"{d_rod:.0f}", f"{L:.0f}", f"{Pcr:.0f}", f"{Pcr/Frod:.1f}"])
table(["τ [N·m]", "F_rod [N]", "rod Ø [mm]", "L [mm]", "P_cr [N]", "SF_buckling"], rows)
p("→ 8 mm 304 봉이면 좌굴 여유 ≥ 10. 6 mm는 8 N·m에서 SF≈4로 가능하지만 핀/구멍 마모를 고려해 8 mm 권장.")
p()

# ------------------------------------------------------------ 6. stem --------
p("## 6. Stem (캐리지 → 스쿱) 굽힘·처짐")
p()
p("캔틸레버 길이 L_s = 300 mm(통 깊이 250 mm + 여유), 끝단 합력 F_r = F·√(1+k_v²), k_v = 0.5 (ASSUMPTION).")
p()
rows = []
stems = (("304 tube Ø20×2", 20.0, 16.0), ("304 tube Ø25×2", 25.0, 21.0), ("304 tube Ø25×3", 25.0, 19.0), ("304 rod Ø16", 16.0, 0.0))
for name, do, di in stems:
    I = math.pi * (do ** 4 - di ** 4) / 64
    for F in (100.0, 200.0):
        Fr = F * math.sqrt(1 + sm.K_VERTICAL ** 2)
        M = Fr * 300.0
        sigma = M * (do / 2) / I
        delta = Fr * 300.0 ** 3 / (3 * E_STEEL * I)
        rows.append([name, f"{F:.0f}", f"{Fr:.0f}", f"{sigma:.0f}", f"{SY_304/sigma:.1f}", f"{delta:.2f}"])
table(["Stem", "F [N]", "F_r [N]", "σ_b [MPa]", "SF_yield", "δ_tip [mm]"], rows)
p(f"피로: 하루 ~10³ 회 × 5년이면 ~2×10⁶ 사이클. 노치계수 Kf≈2를 고려해 공칭응력 ≤ {DESIGN_STRESS_FATIGUE:.0f} MPa 목표 → 운용 F ≤ 100 N이면 **Ø25×2**, F = 150 N이면 **Ø25×3**(σ ≈ 49 MPa) 필요.")
p("δ_tip < 0.5 mm 목표(깊이 20 mm 대비 2.5%) → 100 N에서 Ø25×2 만족. push-rod는 튜브 **밖**에 평행 노출하고 튜브 양끝은 밀봉한다(튜브 내부 push-rod는 세척 불가 공동 → docs/19 FMEA #12).")
p()

# ------------------------------------------------------------ 7. pins / bearings
p("## 7. 스쿱 피벗 핀 & 캐리지 가이드")
p()
rows = []
for F in (100.0, 200.0):
    Fr = F * math.sqrt(1 + sm.K_VERTICAL ** 2)
    for dp in (6.0, 8.0):
        A = math.pi * dp ** 2 / 4
        tau_ds = Fr / (2 * A)
        rows.append([f"{F:.0f}", f"{Fr:.0f}", f"{dp:.0f}", f"{tau_ds:.1f}", f"{0.577*SY_304/tau_ds:.0f}"])
table(["F [N]", "F_r [N]", "pin Ø [mm]", "τ double shear [MPa]", "SF (τ_y≈0.577σ_y)"], rows)
p("캐리지 모멘트: M = F_r × 300 mm. F = 200 N → M ≈ 67 N·m. 블록 간격 s = 80 mm인 2블록 구성이면 블록당 ~0.84 kN 짝힘.")
p("→ 15 mm급 LM 가이드(HGR15/MGN15 등) 2블록이면 정격 대비 충분할 것으로 보이나, **벤더 카탈로그의 정적 모멘트 정격으로 반드시 확인**한다(값 미검증).")
p()

# ------------------------------------------------------------ 8. frame --------
p("## 8. Rig 프레임 (4040 알루미늄 프로파일)")
p()
rows = []
for span in (600.0, 800.0):
    for F in (200.0, 300.0):
        delta = F * span ** 3 / (48 * E_AL * I_4040_MM4)
        rows.append([f"{span:.0f}", f"{F:.0f}", f"{delta:.2f}"])
table(["span [mm]", "F at mid [N]", "δ simply supported [mm]"], rows)
p("I_4040 = 8 cm⁴는 ASSUMPTION(카탈로그 값으로 교체). 2020 프로파일(I≈0.7 cm⁴)은 같은 조건에서 10배 이상 처지므로 주 빔에 사용 금지.")
p()

# ------------------------------------------------------------ 9. energy -------
p("## 9. 사이클 에너지")
p()
rows = []
for F in F_CASES_N:
    W_drag = F * STROKE_MM / 1e3
    W_close = F * sm.R_BOWL_MM / 1e3 * math.radians(DTHETA_CLOSE_DEG)
    W = W_drag + W_close
    E_el = W / 0.35  # ASSUMPTION: electro-mechanical efficiency 35 %
    rows.append([f"{F:.0f}", f"{W_drag:.1f}", f"{W_close:.1f}", f"{W:.1f}", f"{E_el:.0f}", f"{E_el*1000/3600:.1f}"])
table(["F [N]", "W_drag [J]", "W_close [J]", "W_mech [J]", "E_elec [J/scoop]", "E per 1000 scoops [Wh]"], rows)
p("결론: 에너지 비용은 무시할 수준이다. 설계를 지배하는 것은 **peak force/torque**, 위생, 사이클타임이지 에너지가 아니다.")
p()

# ------------------------------------------------------------ 10. current proxy
p("## 10. 전류 → 힘 proxy (제품용 DC 구동)")
p()
p("F_est = 2π·η_s·N·η_g·K_t·(I − I_0) / p. 예시 모터: K_t = 0.05 N·m/A, I_0 = 0.4 A (ASSUMPTION), SFU1610 직결(N = 1, η_g = 1), η_s = 0.9.")
p()
rows = []
Kt, I0, eta_s, lead = 0.05, 0.4, 0.9, 10.0
for F in F_CASES_N:
    T = F * lead / (2 * math.pi * eta_s) / 1e3
    I = T / Kt + I0
    rows.append([f"{F:.0f}", f"{T:.3f}", f"{I:.2f}", f"±{0.15*F:.0f}"])
table(["F [N]", "T_motor [N·m]", "I [A]", "F_est error if η off by 15 % [N]"], rows)
p("I_0(무부하 전류)와 η는 **온도(냉동고 근처 그리스 점도)와 마모에 따라 변한다** → 매 기동 시 공회전 구간에서 I_0를 재측정(auto-tare)하고,")
p("proxy 오차는 P1-2(모터 dyno 교정)에서 실측한다. proxy는 제어용 추정치이며 안전 기능(힘 상한)은 하드웨어 전류제한으로 따로 구현한다.")
p()

# ------------------------------------------------------------ 11. docking ------
p("## 11. 헤드 도킹(캐비닛 상판) 반력 — 제품 구조(docs/12)")
p()
p("반력 경로: 스쿱 → 스템 → z 잠금 → 헤드 프레임 → 웰 둘레 도킹 플레이트(핀 2 + 래치). 스쿱은 프레임 아래 L_s에 있다.")
p()
rows = []
m_head = 3.0  # kg, ASSUMPTION A21
for F in (150.0, 200.0):
    for Ls in (150.0, 300.0):   # scoop depth below frame: full tub vs near-empty tub
        M_tip = F * Ls / 1e3                      # N·m about the front pin line
        M_weight = m_head * 9.81 * 0.15           # frame weight, 150 mm lever
        for s_pin in (250.0,):
            uplift = max(0.0, (M_tip - M_weight) / (s_pin / 1e3))
            shear = F / 2
            rows.append([f"{F:.0f}", f"{Ls:.0f}", f"{M_tip:.1f}", f"{M_weight:.1f}", f"{uplift:.0f}", f"{2*uplift:.0f}", f"{shear:.0f}"])
table(["F [N]", "스쿱 깊이 L_s [mm]", "전복 모멘트 [N·m]", "자중 복원 [N·m]", "래치 인장 [N]", "래치 설계하중 SF2 [N]", "핀당 전단 [N]"], rows)
p("→ 헤드 자중(3 kg 가정)만으로는 전복을 막지 못한다(복원 4.4 N·m ≪ 전복 22–60 N·m). **웰마다 고정 도킹 플레이트 + 래치(설계하중 ≥ 0.5 kN)** 가 필요하다.")
p("대안은 위치이동 암 자체를 강성 있게 만들고 브레이크로 잠그는 것인데, 이 경우 암·브레이크 비용이 커진다 → 도킹 플레이트 방식을 기준안으로 한다(캐비닛 상판 호환성은 P0-4에서 확인).")
p()

# ------------------------------------------------------------ 12. z lock -------
p("## 12. z 잠금·counterbalance")
p()
rows = []
for F in (100.0, 150.0, 200.0):
    Fz = sm.K_VERTICAL * F
    m_stem = 1.2   # kg, stem + scoop + push-rod + carriage share (ASSUMPTION)
    hold = (Fz + m_stem * 9.81) * 1.5
    n_clamp = hold / 0.3       # friction clamp, mu = 0.3 (ASSUMPTION)
    rows.append([f"{F:.0f}", f"{Fz:.0f}", f"{hold:.0f}", f"{n_clamp:.0f}"])
table(["F [N]", "F_z = k_v·F [N]", "잠금 유지력 SF1.5 [N]", "마찰 클램프 수직력 (μ=0.3) [N]"], rows)
p("→ 캠 레버 레일 클램프(수직력 ~1 kN급) 또는 스프링 작동-전자 해제식 브레이크로 충분. counterbalance는 가동질량 ~1.2 kg → 정하중 스프링 ~12 N(약간 모자라게 설정해 스쿱이 표면에 스스로 내려앉게 함).")
p()

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))
print(f"wrote {OUT}")
