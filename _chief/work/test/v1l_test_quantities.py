"""V1-L 시험 계획 계산 — 아이스크림 소요량, E0 표본 크기와 검출력, 불확실도, 단열 홀더 1D 열 모델.

실행:  python3 _chief/work/test/v1l_test_quantities.py > /tmp/out.md   (출력은 stdout 마크다운)
문서:  _chief/work/test/v1l_test_plan.md (이 출력의 표를 옮겨 씀)

규칙(R3): 이 파일의 입력은 모두 ASSUMPTION 이거나 레포의 가정값(A07 등)·검색 요약(snippet) 가격이다.
실측값은 하나도 없다. 결과는 "계획용 추정"이며 성능 주장이 아니다.
의존: numpy (scipy 없음 → t·이항 분포는 직접 계산). 레포 calc/의 기하 함수를 그대로 가져다 쓴다.
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "calc"))

import cartesian_model as cm       # noqa: E402  swept_area, R_SCOOP, WALL_MARGIN
import tub_lane_planner as tlp     # noqa: E402  stroke_volume, depth_for_portion, V_TARGET

# ============================================================== ASSUMPTIONS
RHO = 0.65                  # kg/L, A07 (0.55 / 0.75)                         ASSUMPTION
PORTION_KG = 0.115          # kg, 1 portion (repo 기준)
V_PORTION_CM3 = tlp.V_TARGET / 1e3          # 177 cm3 = 115 g / 0.65
TUB_L = 4.0                 # L, 라벨리 4 L 벌크 (BOM L69)
YIELD = 0.59                # 기계가 portion 으로 뜨는 비율 (tub_lane_planner §3)    ASSUMPTION
TUB_KG = TUB_L * RHO                        # 2.6 kg
TUB_USABLE_KG = TUB_KG * YIELD              # 1.534 kg
PORTIONS_PER_TUB = TUB_USABLE_KG / PORTION_KG
PAN_L = 0.360 * 0.165 * 0.120 * 1e3         # L, 젤라토 팬 360x165x120 = 7.13 L (BOM L37)

E0_TRAVEL = 150.0           # mm, 4 L 통 안 E0 스트로크 길이(dive 포함)          ASSUMPTION (통 실측으로 교체)
CAP_CAPTURE = tlp.CLOSE_CAPTURE             # 닫기 때 담기는 구 캡 비율 0.5       ASSUMPTION
REUSE_LOSS = (0.05, 0.10, 0.20)             # 재포장 1회당 손실(컵·스쿱 잔량, 녹은 물, 취급)  ASSUMPTION
E0_HANDLING_LOSS = 0.05                     # E0 후 통 잔량·칩 회수 손실               ASSUMPTION
CV_CASES = (0.10, 0.15, 0.20, 0.25)         # 스트로크 간 힘 변동계수(손으로 끄는 E0)  ASSUMPTION

lines = []
p = lines.append


def table(h, rows):
    p("| " + " | ".join(h) + " |")
    p("|" + "|".join("---" for _ in h) + "|")
    for r in rows:
        p("| " + " | ".join(str(c) for c in r) + " |")
    p("")


# ============================================================== small stats library (no scipy)
def t_pdf(x, v):
    c = math.exp(math.lgamma((v + 1) / 2) - math.lgamma(v / 2)) / math.sqrt(v * math.pi)
    return c * (1 + x * x / v) ** (-(v + 1) / 2)


def t_cdf(x, v, n=4000):
    a = abs(x)
    h = a / n
    s = t_pdf(0, v) + t_pdf(a, v)
    for i in range(1, n):
        s += (4 if i % 2 else 2) * t_pdf(i * h, v)
    area = s * h / 3
    return 0.5 + area if x >= 0 else 0.5 - area


def t_ppf(pr, v):
    lo, hi = 0.0, 60.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if t_cdf(mid, v) < pr:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def z_ppf(pr):
    from statistics import NormalDist
    return NormalDist().inv_cdf(pr)


def chi2_ppf(pr, v):
    """Wilson–Hilferty 근사."""
    z = z_ppf(pr)
    return v * (1 - 2 / (9 * v) + z * math.sqrt(2 / (9 * v))) ** 3


def binom_sf(k, n, q):
    """P(X >= k)."""
    return sum(math.comb(n, i) * q ** i * (1 - q) ** (n - i) for i in range(k, n + 1))


def cp_lower(k, n, alpha=0.05):
    """Clopper–Pearson 양측 95 % 하한."""
    if k == 0:
        return 0.0
    lo, hi = 0.0, 1.0
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if binom_sf(k, n, mid) < alpha / 2:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def mde_sigma(n1, n2, df, alpha=0.05, power=0.80):
    """두 평균 차이의 최소 검출 효과(σ 단위). 중심 t 분위수 근사."""
    se = math.sqrt(1 / n1 + 1 / n2)
    return (t_ppf(1 - alpha / 2, df) + t_ppf(power, df)) * se


# ============================================================== geometry helpers
def stroke_cm3(theta_deg, depth, travel):
    """dive 경사(30°) + 일정 깊이 드래그 + 닫기 캡(비율 0.5). tlp.stroke_volume 과 같은 식을 θ 에 일반화."""
    a = cm.swept_area(theta_deg, depth)
    l_dive = depth / math.tan(math.radians(tlp.BETA_DIVE))
    l_drag = max(0.0, travel - l_dive)
    b = cm.R_SCOOP * math.cos(math.radians(theta_deg))
    h = b - depth
    d_cap = cm.R_SCOOP - h
    v_cap = math.pi * d_cap ** 2 * (3 * cm.R_SCOOP - d_cap) / 3.0
    v = 0.5 * a * min(l_dive, travel) + a * l_drag + CAP_CAPTURE * v_cap
    return v / 1e3, a


def eq_from_cm3(v_cm3):
    return v_cm3 / V_PORTION_CM3


def kg_from_eq(eq):
    return eq * PORTION_KG


def tubs_from_eq(eq):
    return kg_from_eq(eq) / TUB_USABLE_KG


# ============================================================== header
p("# V1-L 시험 계획 계산 출력")
p("")
p(f"> `python3 _chief/work/test/v1l_test_quantities.py`. 모든 입력은 ASSUMPTION 또는 검색 요약(snippet)이다. 실측 없음.")
p(f"> 환산 사슬: portion 115 g(= {V_PORTION_CM3:.0f} cm³, ρ {RHO} kg/L, A07) → kg → 4 L 통(2.6 kg, 수율 {YIELD:.0%} → 쓸 수 있는 {TUB_USABLE_KG:.3f} kg = **{PORTIONS_PER_TUB:.1f} portion/통**).")
p(f"> 젤라토 팬 360×165×120 = {PAN_L:.2f} L. 80 % 채움 {0.8 * PAN_L:.2f} L = {0.8 * PAN_L / TUB_L:.2f}통, 93 % 채움 {0.93 * PAN_L:.2f} L = {0.93 * PAN_L / TUB_L:.2f}통.")
p("")

# ============================================================== 1. E0 stroke volumes
p("## 1. E0 스트로크 한 번이 쓰는 양 (4 L 통 안, 스트로크 길이 150 mm 가정)")
p("")
rows = []
E0V = {}
for th in (0.0, -30.0):
    for d in (8.0, 10.0, 15.0, 20.0, 25.0):
        v, a = stroke_cm3(th, d, E0_TRAVEL)
        E0V[(th, d)] = v
        rows.append([f"{th:.0f}°", f"{d:.0f}", f"{a:.0f}", f"{v:.0f}", f"{eq_from_cm3(v):.2f}", f"{kg_from_eq(eq_from_cm3(v)) * 1000:.0f}"])
table(["자세 θ", "깊이 d [mm]", "절삭단면 A [mm²]", "스트로크 부피 [cm³]", "portion-eq", "g"], rows)
dA15 = (cm.swept_area(-30, 15.2) - cm.swept_area(-30, 14.8)) / 0.4 / cm.swept_area(-30, 15)
dA10 = (cm.swept_area(-30, 10.2) - cm.swept_area(-30, 9.8)) / 0.4 / cm.swept_area(-30, 10)
dAR = (cm.swept_area(-30, 15, 35.1) - cm.swept_area(-30, 15, 34.9)) / 0.2 / cm.swept_area(-30, 15)
p(f"- 깊이 민감도 (1/A)·dA/dd: d 15 mm에서 {dA15 * 100:.1f} %/mm, d 10 mm에서 {dA10 * 100:.1f} %/mm. 스쿱 반경 민감도 (1/A)·dA/dR = {dAR * 100:.1f} %/mm.")
p("")

# ============================================================== 2. E0 design options, MDE
p("## 2. E0 설계안 비교 — 소요량과 검출 가능한 효과 크기")
p("")
p("검출 가능한 효과(MDE): 양측 α 0.05, 검정력 0.80, 두 수준 평균 차이, 잔차 자유도 df(중심 t 근사). log(u)의 σ ≈ CV로 보고 %로 환산(exp(MDE·CV) − 1).")
p("")


def design_row(name, strokes_spec, n1, n2, df, n_ref, extra_note):
    v = sum(cnt * E0V[(th, d)] for (th, d, cnt) in strokes_spec)
    n_str = sum(cnt for (_, _, cnt) in strokes_spec)
    eq = eq_from_cm3(v)
    m = mde_sigma(n1, n2, df)
    pct = [f"{(math.exp(m * cv) - 1) * 100:.0f}" for cv in CV_CASES]
    ci = t_ppf(0.975, df) / math.sqrt(n_ref)
    return [name, n_str, f"{eq:.1f}", f"{kg_from_eq(eq):.2f}", f"{tubs_from_eq(eq):.2f}",
            f"{n1} vs {n2}, df {df}", f"{m:.2f}σ", " / ".join(pct), f"±{ci:.2f}σ", extra_note]


designs = [
    ("P0-3 원안(3T×3d×2제품×5, θ 0°)",
     [(0.0, 15.0, 30), (0.0, 20.0, 30), (0.0, 25.0, 30)], 45, 45, 72, 5, "제품 45 vs 45 / 온도 쌍 30 vs 30 → 0.73σ"),
    ("P0-3 §7(2θ×3d×2제품×5, −14 °C)",
     [(0.0, 20.0, 10), (0.0, 25.0, 10), (0.0, 25.0, 10), (-30.0, 20.0, 10), (-30.0, 25.0, 10), (-30.0, 25.0, 10)], 30, 30, 48, 5, "d 20/25/28 → 28은 25로 근사"),
    ("Red Team 축소(1제품×3T×3d×3)",
     [(0.0, 15.0, 9), (0.0, 20.0, 9), (0.0, 25.0, 9)], 9, 9, 18, 3, "θ 비교 없음"),
    ("**E0-min 제안 (2θ×2T×n4 @d15 + d10×4)**",
     [(0.0, 15.0, 8), (-30.0, 15.0, 8), (-30.0, 10.0, 4)], 8, 8, 12, 4, "θ·T 주효과 각 8 vs 8"),
    ("E0-min, n3",
     [(0.0, 15.0, 6), (-30.0, 15.0, 6), (-30.0, 10.0, 3)], 6, 6, 8, 3, ""),
    ("E0-min, n5",
     [(0.0, 15.0, 10), (-30.0, 15.0, 10), (-30.0, 10.0, 5)], 10, 10, 16, 5, ""),
]
rows = [design_row(*d) for d in designs]
table(["설계", "스트로크", "portion-eq", "kg", "통", "비교 표본", "MDE", "MDE % @CV 10/15/20/25 %", "기준 셀 평균 95 % CI 반폭", "비고"], rows)
m_min = mde_sigma(8, 8, 12)
m_full = mde_sigma(30, 30, 72)
p(f"- **줄인 대가**: θ·온도 주효과의 검출 한계가 {m_full:.2f}σ(원안, 30 vs 30) → **{m_min:.2f}σ**(E0-min, 8 vs 8)로 커진다. "
  f"CV 15 %면 약 {(math.exp(m_full * 0.15) - 1) * 100:.0f} % → **{(math.exp(m_min * 0.15) - 1) * 100:.0f} %** 차이부터 검출된다. "
  f"교호작용(θ×T)은 그 2배({2 * m_min:.2f}σ)라 사실상 검출하지 못한다. 제품 간 차이는 재지 않는다(1제품).")
p(f"- 깊이 스케일 점검(F = u·A(d)): d 10 vs d 15(각 4, 기준 조건) → u 비율 차이 {mde_sigma(4, 4, 12):.2f}σ(CV 15 %면 {(math.exp(mde_sigma(4, 4, 12) * 0.15) - 1) * 100:.0f} %)부터 검출.")
# prep strokes and stage 2
v_prep = 2 * E0V[(-30.0, 8.0)]
v_stage2 = 4 * E0V[(-30.0, 15.0)]
v_min = 8 * E0V[(0.0, 15.0)] + 8 * E0V[(-30.0, 15.0)] + 4 * E0V[(-30.0, 10.0)]
eq_e0 = eq_from_cm3(v_min + v_prep)
eq_e0_s2 = eq_from_cm3(v_min + v_prep + v_stage2)
p(f"- 표면 평탄화 2회(d 8 mm) 포함 E0-min: **{eq_e0:.1f} portion-eq = {kg_from_eq(eq_e0):.2f} kg = {tubs_from_eq(eq_e0):.2f}통**. "
  f"조건부 2단계(기준 셀 +4, d 15) 포함: **{eq_e0_s2:.1f} eq = {tubs_from_eq(eq_e0_s2):.2f}통** → 4 L 1통 안.")
p("")

# prediction bound factor for single-stroke design load / theta gate
rows = []
for n_ref, df in ((4, 12), (8, 16)):
    for cv in (0.10, 0.15, 0.20, 0.25):
        f_ci = math.exp(t_ppf(0.975, df) * cv / math.sqrt(n_ref))
        f_pb = math.exp(t_ppf(0.95, df) * cv * math.sqrt(1 + 1 / n_ref))
        rows.append([n_ref, df, f"{cv:.0%}", f"×{f_ci:.2f}", f"×{f_pb:.2f}"])
p("### 2.1 게이트 판정에 쓰는 배수 (기준 셀 평균 → 상한)")
p("")
table(["기준 셀 n", "df(합동)", "CV", "평균의 95 % 상한 배수", "한 스트로크 95 % 예측 상한(P95) 배수"], rows)
p("- C-Gate 0·θ-Gate는 **한 스트로크 P95**(= 평균 × 배수)로 판정한다. 과부하·기어 과토크는 스트로크 하나에서 생기기 때문이다.")
p("- 예: 평균 τ_close 2.0 N·m, CV 20 %, n 4면 P95 ≈ 2.0 × " + f"{math.exp(t_ppf(0.95, 12) * 0.20 * math.sqrt(1.25)):.2f}" + " N·m → 3 N·m 경계. 2단계(n 8)로 좁힌다.")
p("")

# failure mode detection
p("### 2.2 드문 현상(칩 부서짐·걸림 등)을 한 번이라도 볼 확률")
p("")
rows = []
for n in (4, 8, 16):
    rows.append([n] + [f"{(1 - (1 - q) ** n) * 100:.0f} %" for q in (0.1, 0.2, 0.3)])
table(["스트로크 수", "발생률 10 %", "20 %", "30 %"], rows)

# ============================================================== 3. consumption per test
p("## 3. 시험별 아이스크림 소요량")
p("")
p("‘새’ = 공장 포장 그대로 보관, 옮겨 담기·재동결 이력 없음(판정용). ‘재사용’ = 재포장 + 재템퍼링(Chief 결정, 판정 제외 시험만).")
p("")
N_CG2 = 22     # 2 warm-up + 20 judged
N_DBG = 20     # 2 passes x ~10 (false bottom)
N_CC = 40      # 2 runs x 20
N_RETEST = 20  # 1 pass reserve
tests = [
    ("E0 본시험(16 d15 + 4 d10) + 평탄화 2", "새", eq_from_cm3(v_min + v_prep), "통 1(원포장 그대로)"),
    ("E0 2단계(조건부, 기준 셀 +4)", "새", eq_from_cm3(v_stage2), "통 1 잔량 → 부족하면 통 3"),
    ("E0-C 모사재(점토) 6", "–", 0.0, "점토(L70), 재사용"),
    ("TH-1 냉동고 설정 범위 48 h / TH-2 홀더 온도 곡선", "–", 0.0, "E0 통·DBG 팬에 열전대(공유)"),
    ("CAL-F·CAL-K·T-Z·T0·T1·T4–T8·T-M4", "–", 0.0, "물병·알루미늄 블록·점토"),
    ("T2 터치오프(아이스크림 10점)", "공유", 0.0, "DBG 팬 표면(접촉만)"),
    ("DBG 실제 아이스크림 디버깅 2 pass × ~10", "재사용", float(N_DBG), "E0 통 잔량 + 칩, 가짜 바닥"),
    ("CG2 드래그 힘·portion 판정(예열 2 + 판정 20)", "새", float(N_CG2), "통 2 + 통 3(팬 93 % 채움)"),
    ("K2 배출 50회", "공유", 0.0, "CG2 22회 + CC 앞 28회"),
    ("CC 연속 사이클 2 run × 20", "재사용", float(N_CC), "재사용 풀"),
    ("재시험 예비 1 pass", "재사용", float(N_RETEST), "재사용 풀"),
]
rows = []
tot_new = tot_reuse = 0.0
for name, kind, eq, src in tests:
    rows.append([name, kind, f"{eq:.1f}", f"{kg_from_eq(eq):.2f}", f"{tubs_from_eq(eq):.2f}", src])
    if kind == "새":
        tot_new += eq
    elif kind == "재사용":
        tot_reuse += eq
table(["시험", "재료", "portion-eq", "kg", "통(4 L·59 %)", "재료 출처"], rows)
no_share = eq_e0_s2 + N_DBG + N_CG2 + 28 + N_CC + N_RETEST
share_only = eq_e0_s2 + N_DBG + N_CG2 + N_CC + N_RETEST
p(f"- 새 제품 합계 **{tot_new:.1f} eq = {kg_from_eq(tot_new):.2f} kg = {tubs_from_eq(tot_new):.2f}통** → 구매 **{math.ceil(tubs_from_eq(tot_new))}통**(여유 {math.ceil(tubs_from_eq(tot_new)) - tubs_from_eq(tot_new):.2f}통 = {(math.ceil(tubs_from_eq(tot_new)) - tubs_from_eq(tot_new)) * TUB_KG:.2f} kg).")
p(f"- 재사용 합계 {tot_reuse:.0f} eq({tubs_from_eq(tot_reuse):.2f}통 분량)는 새로 사지 않고 판정 후 회수한 재료로 돈다(아래 3.1).")
p(f"- 비교: 재사용·공유 없이 모두 새로 사면 {no_share:.0f} eq = **{tubs_from_eq(no_share):.1f}통**, 공유만 하면 {share_only:.0f} eq = {tubs_from_eq(share_only):.1f}통.")
fill_L = N_CG2 * V_PORTION_CM3 / 1e3 / YIELD
p(f"- CG2 한 번 채움 부피 = {N_CG2} × {V_PORTION_CM3:.0f} cm³ ÷ {YIELD} = {fill_L:.2f} L = 팬의 {fill_L / PAN_L:.0%} = {fill_L / TUB_L:.2f}통.")
for rho in (0.55, 0.75):
    v_p = PORTION_KG / rho * 1e3
    n_fit = PAN_L * 0.93 * 1e3 * YIELD / v_p
    p(f"  - ρ {rho}: portion 부피 {v_p:.0f} cm³ → 93 % 채움에서 {n_fit:.1f} portion(예열 2 빼면 판정 {n_fit - 2:.0f}).")
p("")

# reuse pool balance
p("### 3.1 재사용 풀 질량 수지 (kg, 재포장 1회당 손실률 가정별)")
p("")
FILL_KG = 0.93 * PAN_L * RHO          # 4.31 kg
rows = []
for loss in REUSE_LOSS:
    pool = TUB_KG * (1 - E0_HANDLING_LOSS)               # E0 통 전부(칩 포함) 회수
    seq = [("E0 후(3주)", pool)]
    for lab in ("DBG 1(13주)", "DBG 2(14주)"):
        pool *= (1 - loss)                               # 가짜 바닥: 풀 전체를 한 번에 씀
        seq.append((lab, pool))
    new_fill = min(FILL_KG, 2 * TUB_KG)
    spare_new = 2 * TUB_KG - new_fill
    pool += new_fill * (1 - loss)                        # CG2 판정 뒤 회수
    seq.append(("CG2 후(15주)", pool))
    ok = True
    for lab in ("CC 1(18주)", "CC 2(20주)", "재시험(22주)"):
        if pool < FILL_KG:
            ok = False
        pool -= FILL_KG * loss
        seq.append((lab, pool))
    rows.append([f"{loss:.0%}"] + [f"{v:.2f}" for _, v in seq] + [f"{spare_new:.2f}", "●" if ok else "✘ 채움 부족"])
table(["손실/회"] + [lab for lab, _ in seq] + ["통 3 새 여유 kg", f"매 pass ≥ {FILL_KG:.2f} kg"], rows)
p("- DBG는 가짜 바닥(XPS 블록 + 식품용 랩)으로 채움 두께를 줄여 E0 통 잔량만으로 돈다. CC·재시험은 93 % 채움.")
p("- 재포장할수록 공기가 빠져 밀도가 오른다(과팽창률 감소). 매 채움 때 팬 질량 ÷ 채움 부피로 밀도를 기록한다(재료 이력 지표).")
p("")

# ============================================================== 4. cost
p("## 4. 아이스크림 비용 (가격은 모두 검색 요약 snippet, 2026-09-28, 구매 전 상품 페이지 확인)")
p("")
BOM_L69 = 46780
SUMB = 895721               # elec/v1l_bom.csv SUM-B after mechanical rework RT1 (commit 902c136)
R8_BUY_CAP = 900000         # DECISIONS: 구매+배송 ≤ 900,000 (예비비 ≥ 100,000 별도)
COND_SAVINGS = [("C13 필라멘트 학교 지급", 21700), ("C16 18T 공작실 재단", 16000),
                ("C14 4080 kg단가 견적", 8410), ("C18 주문 묶음 배송", 10000)]   # SUM-E 목록(조건부)
prices = [
    ("라벨리 프리미엄 바닐라 4 L", 4.0, 21800, "다나와 최저가 snippet"),
    ("라벨리 프리미엄 바닐라 4 L", 4.0, 23390, "SSG snippet (= BOM L69)"),
    ("라벨리 프리미엄 바닐라 4 L", 4.0, 15400, "클리앙 특가 게시물 snippet, 날짜 미확인"),
    ("롯데 조안나 바닐라 5 L", 5.0, 12900, "삼양 서브큐몰 snippet, 식품유형 미확인"),
    ("롯데 조안나 바닐라 5 L", 5.0, 14900, "다나와 snippet"),
    ("노브랜드 향긋한 바닐라향 아이스 5 L", 5.0, 13480, "다나와 snippet, '아이스'(식품유형 미확인)"),
]
rows = []
need_L = tubs_from_eq(tot_new) * TUB_L
for name, vol, pr, src in prices:
    n_units = math.ceil(need_L / vol - 1e-9)
    cost = n_units * pr
    rows.append([name, src, f"{pr:,}", f"{pr / vol:,.0f}", n_units, f"{cost:,}", f"{cost - BOM_L69:+,}", f"{SUMB + cost - BOM_L69:,}"])
table(["제품", "근거", "단가 [원]", "원/L", "필요 개수", "합계 [원]", "BOM L69(46,780) 대비", f"SUM-B({SUMB:,}) 재계산"], rows)
p(f"- 필요 부피 = 새 제품 {need_L:.1f} L(수율 59 % 기준). 5 L 제품은 같은 부피에 개수가 같거나 적다.")
p(f"- R8 기준(구매+배송 ≤ {R8_BUY_CAP:,}원): 현재 SUM-B {SUMB:,}원(기계 RT1 반영)의 여유는 {R8_BUY_CAP - SUMB:,}원뿐이라 "
  "라벨리 3통은 어느 가격이든 넘는다(특가 15,400원 제외). 5 L 업소용 3개는 여유 안이다.")
p("- 냉동 배송비는 L73(배송 50,000원)에 들어 있다고 가정했다. 3통을 **한 번에** 주문해야 이 가정이 맞는다.")
p("")

# ============================================================== 5. added test consumables
p("## 5. 시험용 추가 소모품 (BOM에 없음, 모두 추정)")
p("")
extras = [
    ("디지털 행잉 저울 50 kg / 10 g, peak hold (푸시풀 게이지를 빌리면 제외)", 8000, "추정", "E0 τ_close, 나사 마찰 T-μ"),
    ("속도 게이트 마이크로스위치 2개", 2400, "L48 단가", "E0 평균 속도"),
    ("지퍼백·밀폐용기·라벨(재사용 풀)", 5000, "추정", "재료 이력 관리"),
    ("식품용 랩(가짜 바닥 포장)", 2000, "추정", "DBG 채움 절감"),
]
optional = [("(선택) 볼베어링 서랍 레일 450–500 mm 1조 — 기본은 L92 남는 면적의 합판 채널 가이드(0원)", 10000, "추정", "E0 캐리지 상하 구속")]
rows = [[a, f"{b:,}", c, d] for a, b, c, d in extras]
tot_x = sum(b for _, b, _, _ in extras)
rows.append(["**합계(기본)**", f"**{tot_x:,}**", "", ""])
rows += [[a, f"{b:,}", c, d] for a, b, c, d in optional]
table(["품목", "원", "근거", "쓰임"], rows)
p("- 교정 물병은 기계 RT1에서 L72가 7병(133 N)으로 늘어 추가하지 않는다. 수직 C_zz도 7병(약 14 kg)까지만 건다.")
p("")
p("### 5.1 R8 재계산 — 구매+배송 상한과 닫는 조건")
p("")
rows = []
scen = [("라벨리 4 L × 3 (21,800)", 3 * 21800), ("라벨리 4 L × 3 (23,390, SSG)", 3 * 23390),
        ("5 L 업소용 × 3 (12,900, 유형 확인 조건)", 3 * 12900), ("5 L 업소용 × 3 (14,900)", 3 * 14900)]
for lab, ic in scen:
    add = ic - BOM_L69 + tot_x
    sb = SUMB + add
    over = sb - R8_BUY_CAP
    need = []
    acc = 0
    if over > 0:
        for nm, sv in COND_SAVINGS:
            acc += sv
            need.append(nm.split()[0])
            if acc >= over:
                break
    rows.append([lab, f"{add:+,}", f"{sb:,}", f"{over:+,}", " + ".join(need) + (f" (−{acc:,})" if need else "") if over > 0 else "불필요"])
table(["아이스크림 선택", "BOM 대비 증감(소모품 포함)", "SUM-B", "900,000 대비", "필요한 조건부 절감(순서대로)"], rows)
p(f"- 조건부 절감 합계(SUM-E 목록) = {sum(s for _, s in COND_SAVINGS):,}원. 예비비 100,000원은 건드리지 않는다.")
p("")

# ============================================================== 6. rectangular container as pan
p("## 6. 벌크 용기를 그대로 팬으로 쓸 때 — 내부 길이별 1 portion 깊이와 힘 (θ −30°, 벽 여유 10 mm)")
p("")
rows = []
for l_in in (250, 280, 300, 330, 360):
    travel = l_in - 2 * (cm.R_SCOOP + cm.WALL_MARGIN)
    d, a, v = tlp.depth_for_portion(travel)
    shrink = 2 * 80 * math.tan(math.radians(3.0))
    d2, a2, _ = tlp.depth_for_portion(travel - shrink)
    if d is None:
        rows.append([l_in, f"{travel:.0f}", "불가", "–", "–", "–", "–"])
        continue
    f90, f160 = 0.090 * a, 0.160 * a
    rows.append([l_in, f"{travel:.0f}", f"{d:.1f}", f"{a:.0f}", f"{f90:.0f}", f"{f160:.0f}",
                 f"{d2:.1f} / {0.160 * a2:.0f} N" if d2 else "불가"])
table(["내부 길이 [mm]", "C 이동 [mm]", "1 portion 깊이 [mm]", "A [mm²]", "F @ 90 kPa [N]", "F @ 160 kPa [N]", "벽 기울기 3°, 80 mm 아래: 깊이 / F@160"], rows)
p("- 설계 하중 F_d 100 N(u 160 kPa)을 1스트로크로 지키려면 C 이동 ≥ 약 270 mm, 곧 **내부 길이 ≥ 약 360 mm**가 필요하다(젤라토 팬과 같음).")
p("- 더 짧은 용기는 F_TARGET 82.5 N 깊이 적응으로 **2스트로크**가 늘어난다(파손이 아니라 사이클 증가). 판정 조건이 팬과 달라지므로 CG2에는 쓰지 않고, DBG·배출·CC에만 쓸 수 있다.")
p("")

# ============================================================== 7. thermal 1D model
p("## 7. 단열 홀더·냉동고 1D 열 모델 (위 표면 기준 깊이 방향, 옆면은 체적 손실항)")
p("")
K_XPS = 0.034
H_OPEN = 10.0             # W/m2K, repo A43
U_LID = 1 / (1 / H_OPEN + 0.03 / K_XPS)
U_BOT = 1 / (0.05 / K_XPS + 1 / H_OPEN)
P_OVER_A = 2 * (0.36 + 0.165) / (0.36 * 0.165)
SIDE = U_BOT * P_OVER_A   # W/m3K
H_FRZ = 5.0               # W/m2K, 냉동고 안 정체 공기(ASSUMPTION)
CASES = [("빠름(k 0.45, c 3.0)", 0.45, 3000.0), ("중간(k 0.30, c 4.5)", 0.30, 4500.0), ("느림(k 0.15, c 6.0)", 0.15, 6000.0)]
RHO_SI = RHO * 1000


def simulate(k, c, H, T0, u_top, t_top, u_bot, t_bot, side, t_side, t_end, n=60):
    dx = H / n
    x = (np.arange(n) + 0.5) * dx
    T = np.full(n, float(T0))
    alpha = k / (RHO_SI * c)
    dt = 0.35 * dx * dx / alpha
    steps = int(t_end / dt) + 1
    r_top = 1 / u_top + dx / (2 * k) if u_top > 0 else None
    r_bot = 1 / u_bot + dx / (2 * k) if u_bot > 0 else None
    rec_t, rec = [], []
    every = max(1, int(60.0 / dt))
    for s in range(steps):
        q = np.zeros(n)
        flux = k * (T[1:] - T[:-1]) / dx
        q[:-1] += flux
        q[1:] -= flux
        if r_top:
            q[0] += (t_top - T[0]) / r_top
        if r_bot:
            q[-1] += (t_bot - T[-1]) / r_bot
        q += side * dx * (t_side - T)
        T = T + dt * q / (RHO_SI * c * dx)
        if s % every == 0:
            rec_t.append(s * dt)
            rec.append(T.copy())
    return x, np.array(rec_t), np.array(rec)


def layer_mean(x, T, z0, z1):
    m = (x >= z0) & (x <= z1)
    return T[..., m].mean(axis=-1)


def first_time(t, y, thr, rising=True):
    idx = np.where(y >= thr)[0] if rising else np.where(y <= thr)[0]
    return t[idx[0]] / 60 if len(idx) else float("nan")


H_FILL = 0.10
p(f"가정: ρ {RHO_SI:.0f} kg/m³, 겉보기 비열 c(잠열 포함) 3.0–6.0 kJ/kg·K(repo §4와 같은 범위), 열전도율 k 0.15–0.45 W/m·K(값 미확인). "
  f"채움 두께 {H_FILL * 1000:.0f} mm, 실내 22 °C. 열린 위 h {H_OPEN:.0f} W/m²K, 뚜껑(XPS 30) U {U_LID:.2f}, 바닥·옆 XPS 50 U {U_BOT:.2f} W/m²K, 냉동고 안 h {H_FRZ:.0f} W/m²K. "
  "절삭층 = 표면 0–15 mm 평균(1 portion 깊이 14.7 mm). **모두 ASSUMPTION — TH-2 실측 곡선으로 k·c를 맞춘다.**")
p("")
p("### 7.1 냉동고(−18 °C)에서 꺼내 단열 홀더에서 데우기 (‘−18 → −14 데우기’ 방법)")
p("")
rows = []
for lid_lab, u_top, t_end in (("뚜껑 열림", H_OPEN, 4 * 3600), ("뚜껑 닫음", U_LID, 12 * 3600)):
    for lab, k, c in CASES:
        x, t, R = simulate(k, c, H_FILL, -18.0, u_top, 22.0, U_BOT, 22.0, SIDE, 22.0, t_end)
        lay = layer_mean(x, R, 0.0, 0.015)
        t15, t14, t13 = (first_time(t, lay, th) for th in (-15.0, -14.0, -13.0))
        if math.isnan(t14):
            rows.append([lid_lab, lab, "–", "> " + str(t_end // 60), "–", "–", "–", "–", "–", "–"])
            continue
        i14 = int(np.argmin(np.abs(t / 60 - t14)))
        Ts = R[i14]
        g = (Ts[0] - np.interp(0.015, x, Ts))
        core = np.interp(0.05, x, Ts)
        rows.append([lid_lab, lab, f"{t15:.0f}", f"{t14:.0f}", f"{t13:.0f}", f"{t13 - t15:.0f}", f"{Ts[0]:.1f}",
                     f"{np.interp(0.015, x, Ts):.1f}", f"{g / 15:.2f}", f"{core:.1f}"])
table(["상부", "경우", "절삭층 −15 °C [분]", "−14 °C [분]", "−13 °C [분]", "−15→−13 창 [분]", "그때 표면 [°C]", "15 mm [°C]", "절삭층 기울기 [K/mm]", "50 mm 깊이 [°C]"], rows)
p("- 뚜껑을 열고 데우면 절삭층 평균이 −14 °C가 되는 순간에도 **표면 −7 ~ −10 °C, 15 mm 깊이 −16 ~ −18 °C**다. 한 스트로크 안에서 6–10 K 차이가 나므로 u를 ‘−14 °C 값’이라고 할 수 없다.")
p("- 뚜껑을 닫고 데우면 기울기는 작아지지만 몇 시간이 걸리고, 그동안 코어는 여전히 차갑다(아래 50 mm 열).")
p("")

p("### 7.2 −14 °C로 템퍼링된 팬을 홀더에 넣은 뒤 절삭층(0–15 mm 평균)이 −13 °C를 넘기까지 [분]")
p("")
rows = []
for h_open in (H_OPEN, 5.0):
    u_lid = 1 / (1 / h_open + 0.03 / K_XPS)
    for lab, k, c in CASES:
        out = [f"{h_open:.0f}", lab]
        for u in (h_open, 0.3 * h_open + 0.7 * u_lid, u_lid):
            x, t, R = simulate(k, c, H_FILL, -14.0, u, 22.0, U_BOT, 22.0, SIDE, 22.0, 6 * 3600)
            lay = layer_mean(x, R, 0.0, 0.015)
            tt = first_time(t, lay, -13.0)
            out.append("> 360" if math.isnan(tt) else f"{tt:.0f}")
        rows.append(out)
table(["윗면 h [W/m²K]", "경우", "뚜껑 열림", "30 % 열림(시험 중 평균)", "뚜껑 닫음"], rows)
p("- repo §4(덩어리 모델, 전체 평균 +2 K)는 14–29분(열림)이었다. **절삭층만 보면 몇 분 만에** 벗어난다. 윗면 h는 오목한 팬 안 자연대류 + 복사라 5–10 W/m²K 범위로 보았다(확인 필요).")
p("")


def simulate_layers(k, c, T0, u_top, t_layer_s, n_cells=70, H=0.105, cut_cells=10, floor_cells=7):
    """층마다 위 15 mm를 깎아 내며 온도 이력을 따라감. 깎기 직전 절삭층 평균을 기록."""
    dx = H / n_cells
    T = np.full(n_cells, float(T0))
    alpha = k / (RHO_SI * c)
    dt = 0.35 * dx * dx / alpha
    out = []
    while len(T) >= cut_cells + floor_cells:
        steps = int(t_layer_s / dt)
        for _ in range(steps):
            n = len(T)
            q = np.zeros(n)
            flux = k * (T[1:] - T[:-1]) / dx
            q[:-1] += flux
            q[1:] -= flux
            q[0] += (22.0 - T[0]) / (1 / u_top + dx / (2 * k))
            q[-1] += (22.0 - T[-1]) / (1 / U_BOT + dx / (2 * k))
            q += SIDE * dx * (22.0 - T)
            T = T + dt * q / (RHO_SI * c * dx)
        out.append((T[:cut_cells].mean(), T[0], T[cut_cells - 1]))
        T = T[cut_cells:]
    return out


p("### 7.3 층을 깎아 내려가며 — 깎는 순간 절삭층 평균 온도 (−14 °C 템퍼링 시작, 뚜껑 30 % 열림, h 10)")
p("")
p("한 층(15 mm)을 레인 3개로 깎는 데 걸리는 시간별. E0는 스트로크당 약 1.5분 → 층당 약 4.5분, 기계(CC)는 사이클 약 47 s(ctrl mock) → 층당 약 2.4분. 1D라 레인 겹침은 무시했다.")
p("")
rows = []
u30 = 0.3 * H_OPEN + 0.7 * U_LID
for lab, k, c in CASES:
    for t_layer in (2.4, 4.5, 9.0):
        seq = simulate_layers(k, c, -14.0, u30, t_layer * 60)
        n_ok = 0
        for mean, _, _ in seq:
            if mean <= -13.0:
                n_ok += 1
            else:
                break
        rows.append([lab, f"{t_layer:.1f}", " / ".join(f"{m:.1f}" for m, _, _ in seq[:6]), n_ok,
                     f"{seq[0][1] - seq[0][2]:.1f}"])
table(["경우", "층당 시간 [분]", "층 1…6 절삭층 평균 [°C]", "−13 °C 이하인 층 수(연속)", "층 1 표면−15 mm 차 [K]"], rows)
p("- 깎을 때마다 데워진 윗층이 없어지고 찬 속이 드러나므로, **빨리 깎을수록 절삭층이 목표 근처에 머문다**. 느리게(층당 9분) 가면 몇 층 만에 창을 벗어난다.")
p("- 그래도 층 안에서 표면과 15 mm 깊이는 수 K 차이가 난다 → E0 기록 온도는 **절삭 중간 깊이(7–8 mm) 열전대**값으로 정의하고, 표면 IR 값을 함께 적는다.")
p("")

p("### 7.4 냉동고 안 템퍼링 시간 — 공기 −14 °C, 위·아래 h 5 (팬을 선반에 둠)")
p("")
rows = []
for lab, k, c in CASES:
    out = [lab]
    for T0 in (-18.0, -8.0):
        x, t, R = simulate(k, c, H_FILL, T0, H_FRZ, -14.0, H_FRZ, -14.0, 0.0, -14.0, 96 * 3600)
        dev = np.abs(R + 14.0).max(axis=1)
        tt = first_time(t, dev, 0.5, rising=False)
        out.append("> 96" if math.isnan(tt) else f"{tt / 60:.0f}")
    rows.append(out)
table(["경우", "−18 → −14 ± 0.5 (새 통·팬) [h]", "재포장 −8 → −14 ± 0.5 [h]"], rows)
p("- −8 °C 시작은 비열을 상수로 둔 값이라 **과소 추정**이다(−8 °C 근처는 녹는 잠열이 커서 겉보기 비열이 훨씬 크다, 확인 필요).")
p("- 결론: '24 h 재템퍼링'은 느린 쪽 물성이면 모자랄 수 있다 → **시간 규칙이 아니라 코어(50 mm) 열전대가 목표 ±0.5 K 안에 든 뒤 사용**으로 바꾼다.")
p("")

# ============================================================== 8. uncertainty budgets
p("## 8. 불확실도 예산 (표준불확도 u, 확장 U = 2u; Type B는 가정)")
p("")
p("### 8.1 u = F̄ / A(d) — 스트로크 한 번 (기준 조건 d 15 mm)")
p("")
comp = [
    ("X 셀 현장 교정(도르래 up/down 평균, 마찰 잔차)", 1.0, "ASSUMPTION, CAL-F로 대체"),
    ("등속 구간 평균의 잡음(0.005 N rms, 240샘플)", 0.01, "elec §2.3"),
    ("홈 깊이 측정 ±0.2 mm(버니어 깊이바 3점 평균)", 0.2 * dA15 * 100, "(1/A)dA/dd × 0.2 mm"),
    ("스쿱 반경 R 측정 ±0.1 mm", 0.1 * dAR * 100, "(1/A)dA/dR × 0.1 mm"),
    ("온도 ±0.5 K × 민감도 3–10 %/K", 0.5 * 6.5, "민감도는 E0가 잰다(중간값 6.5 %/K 가정)"),
    ("손 속도 ±20 % × 속도 지수 n 0–0.3", 100 * (1.2 ** 0.15 - 1), "n 0.15 가정(A03)"),
]
rows = []
ss = 0
for name, val, note in comp:
    rows.append([name, f"{val:.2f}", note])
    ss += val * val
uB = math.sqrt(ss)
rows.append(["**합성 Type B (스트로크 하나)**", f"**{uB:.1f}**", "RSS"])
table(["요인", "u [%]", "근거"], rows)
for cv in (0.10, 0.15, 0.20):
    uA = cv * 100 / math.sqrt(4)
    k = t_ppf(0.975, 12)
    p(f"- 기준 셀 평균(n 4, CV {cv:.0%}): Type A = CV/√4 = {uA:.1f} %, 계통 성분(교정·R) {math.sqrt(1 + (0.1 * dAR * 100) ** 2):.1f} %는 평균해도 줄지 않음 → "
      f"U95 ≈ {k:.2f}×{uA:.1f} ⊕ 2×{math.sqrt(1 + (0.1 * dAR * 100) ** 2):.1f} = **±{math.sqrt((k * uA) ** 2 + (2 * math.sqrt(1 + (0.1 * dAR * 100) ** 2)) ** 2):.0f} %**. "
      "(깊이·온도·속도 오차는 스트로크마다 달라 Type A에 이미 들어 있음)")
p("")
p("### 8.2 portion 질량(수직 20 kg 셀, 스쿱 직전 영점 → 직후 30 s 안)")
p("")
comp = [
    ("감도 교정(물병 스텝 6 kg 부근, 주방 저울 1 g) 0.5 % × 115 g", 0.575),
    ("크리프·열 드리프트 30 s(단열 스페이서)", 0.5),
    ("잡음 0.21 g rms, 1 s 평균(80샘플)", 0.21 / math.sqrt(80)),
    ("스쿱·팬 서리·물방울", 0.3),
]
rows = [[a, f"{b:.2f}"] for a, b in comp]
um = math.sqrt(sum(b * b for _, b in comp))
rows.append(["**합성 u / U95**", f"**{um:.2f} / {2 * um:.1f} g**"])
table(["요인(모두 가정)", "u [g]"], rows)
band = 115 * 0.15
p(f"- 허용 폭 −5 %/+10 % = {band:.2f} g. 시험 불확도비 TUR = {band:.2f} / (2 × {2 * um:.1f}) = **{band / (4 * um):.1f}** (≥ 4면 판정 가능). 컵 쪽 주방 저울(1 g)로 5개 교차 확인.")
p("")
p("### 8.3 스쿱 끝 강성 δ(100 N) (CAL-K)")
p("")
comp = [
    ("다이얼 0.01 mm 분해능·읽기", 0.01 / math.sqrt(12) * 1.4),
    ("다이얼 축 오정렬 5° 코사인 × 0.9 mm", 0.9 * (1 - math.cos(math.radians(5)))),
    ("힘(셀 교정 1 %) × 0.9 mm", 0.009),
    ("마그네틱 베이스·홀더 판 처짐(미확인)", 0.01),
]
rows = [[a, f"{b:.4f}"] for a, b in comp]
us = math.sqrt(sum(b * b for _, b in comp))
rows.append(["**합성 u / U95**", f"**{us:.3f} / {2 * us:.3f} mm**"])
table(["요인(가정)", "u [mm]"], rows)
p(f"- 판정선 1.0 mm, 히스테리시스 0.10 mm, 영점 복귀 0.05 mm 대비 U95 {2 * us:.3f} mm. 영점 복귀 판정은 U95와 같은 크기라 **0.01 mm 게이지로는 ‘0.05 mm 이하’를 겨우 판정** → 0.001 mm 게이지를 빌리면 좋다.")
p("")
p("### 8.4 반복 측정 σ의 신뢰구간 배수 (터치오프 T2, 정지 거리 T1)")
p("")
rows = []
for n in (10, 20, 30):
    lo = math.sqrt((n - 1) / chi2_ppf(0.975, n - 1))
    hi = math.sqrt((n - 1) / chi2_ppf(0.025, n - 1))
    rows.append([n, f"×{lo:.2f}", f"×{hi:.2f}"])
table(["n", "σ 95 % 하한", "σ 95 % 상한"], rows)
p("")

# ============================================================== 9. pass/fail operating characteristics
p("## 9. 합격 규칙의 판정력 (이항)")
p("")
rows = []
for (k, n, lab) in ((48, 50, "K2 배출 ≥ 48/50"), (50, 50, "K2 50/50"), (18, 20, "CG2 portion ≥ 18/20"), (20, 20, "CG2 20/20")):
    rows.append([lab, f"{cp_lower(k, n) * 100:.1f} %"])
table(["결과", "참 성공률 95 % 하한(Clopper–Pearson)"], rows)
rows = []
for q in (0.80, 0.90, 0.95, 0.98):
    rows.append([f"{q:.0%}", f"{binom_sf(48, 50, q) * 100:.0f} %", f"{binom_sf(18, 20, q) * 100:.0f} %"])
table(["참 성공률", "K2(≥ 48/50) 합격 확률", "CG2(≥ 18/20) 합격 확률"], rows)
p("- K2 규칙은 참 성공률 90 %짜리를 거의 떨어뜨리지만(합격 확률 낮음) 98 %짜리도 가끔 떨어뜨린다. 20회 portion은 80 %와 95 %를 잘 구분하지 못한다 → **portion은 합격 비율과 함께 평균 편차·SD(연속값)를 같이 보고**한다.")
p("")

print("\n".join(lines))
