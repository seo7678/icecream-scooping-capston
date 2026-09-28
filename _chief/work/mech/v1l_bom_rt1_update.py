#!/usr/bin/env python3
"""Red Team round 1 (mechanical share) → elec/v1l_bom.csv. Idempotent; run after v1l_bom_plywood_update.py.

Run:  python3 _chief/work/mech/v1l_bom_rt1_update.py

Rows set by ID (all marked "[기계 RT1]" in Notes):
  L01 4080 Z column 0.5 → 0.55 m (column extends 50 mm into the head bracket, mech calc §4)
  L17 MGN12 rail 450 → 500 (blocks stayed on the rail only 13 mm short, mech calc §4)
  L20 Al plate parts → 10T 6061: head parts + frame/drive brackets (Z block clamps, KFL08 supports,
      X motor mount, X nut bracket, Z motor mount, X-cell bracket, stop blocks, platform cross bar)
  L88 wood fastening + more epoxy (bedding) + stop-cage set screws
  L89 Al angle 40×40×4 → not bought (replaced by 10T brackets)
  L63 acrylic 900×600 sheet → cut piece 470×520 (window bottom raised to rim + 130)
  L48 interlock microswitches 3 → 5 (three lane-position switches, DECISIONS 2026-09-28)
  L72 calibration water bottles 6 → 7 (132 N incl. 5 % pulley loss, mech calc §6.8)
  L37 notes only: C15 (GN 1/3 pan) rejected (mech calc §6.9) → removed from the conditional sum
  L92 new: 6T plywood sheet #2 → force-platform top plate (2 × 6T laminated)
  L93 new: X-cell link (2 rod ends M6 + threaded rod)
Price status: search summaries are written "snippet" (DECISIONS 2026-09-28), guesses "estimate".
SUM rows are recomputed with the same rules as v1l_bom_plywood_update.py; the R8 rule is now
purchase + shipping ≤ 900,000 KRW and contingency ≥ 100,000 KRW (DECISIONS 2026-09-28).
"""

import csv
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
BOM = os.path.abspath(os.path.join(HERE, "..", "elec", "v1l_bom.csv"))
TAG = "[기계 RT1]"
SNIP = "snippet 2026-09-28 (구매 전 상품 페이지 확인)"
LIMIT_BUY = 900000

with open(BOM, encoding="utf-8", newline="") as fh:
    rows = list(csv.reader(fh))
hdr, body = rows[0], [r for r in rows[1:] if not r[0].startswith("SUM")]
idx = {r[0]: i for i, r in enumerate(body)}
before = {r[0]: list(r) for r in body}


def put(ID, **kw):
    """set columns of an existing row (by header name fragments)."""
    r = body[idx[ID]]
    cols = {"part": 2, "spec": 3, "qty": 4, "purpose": 5, "unit": 6, "total": 7, "src": 8, "url": 9, "status": 10, "notes": 14}
    for k, v in kw.items():
        r[cols[k]] = str(v)
    q, u = float(r[4] or 0), float(r[6] or 0)
    r[7] = str(round(q * u))



# ---------------------------------------------------------------- existing rows
put("L01", qty="0.55", spec="Z 이동 기둥 550 ×1 = 0.55 m (헤드 브래킷 안으로 50 mm 연장, 교차빔은 합판 교차판 L86)",
    status=SNIP,
    notes=f"{TAG} 0.5 → 0.55 m: 레일 500을 기둥 끝에 맞춰 두 블록이 행정 ±10 mm 여유까지 레일 위에 남게(기계 출력 §4). "
          "[기계 합판 하이브리드] 교차빔을 합판으로 바꾸고 Z 기둥만 남김. "
          "[조건부 절감 C14: [조건부] 4080 판매처 kg단가 견적 (약 25,000원/m 가정), −8,410원 — 총액 미반영] "
          "검색 결과 DF4080(중량) 40,290원/m (기계 부품 M01)")
for ID in ("L02", "L03", "L86"):
    put(ID, status=SNIP)
put("L87", status="snippet 2026-09-28 (합판 재단 1컷 1,000원, 판매처 미특정)")
put("L17", part="MGN12 500 mm + MGN12H", spec="레일 1 + 블록 1 (450 mm US$24.87 검색 결과 + 50 mm 추정)", unit=35075,
    status="estimate (snippet 450 mm 가격 + 길이 증가분)",
    notes=f"{TAG} 450 → 500: 초판은 C_max + 여유에서 하부 블록이 레일 끝을 13 mm 벗어남(기계 출력 §4). 블록 2개는 교차판 ㄷ자 클램프에 1개씩")
put("L20", part="알루미늄판 10T (6061) 절단품 — 헤드 + 프레임·구동 브래킷",
    spec="약 0.11 m²(10T 300×400 1장 분량) 블랭크 절단: 헤드 브래킷, θ 모터 마운트, 상부 크랭크 ×2, 하부 레버판 / "
         "Z 블록 ㄷ자 클램프 2세트(웹 100×80 + 플랜지 100×40 ×2), Z 너트·Z KFL08 브래킷, Z 모터 마운트, "
         "X KFL08 받침 2(직립 70×45 + 발 70×50), X 모터 마운트, X 너트 브래킷, X 셀 받침, 스톱 블록 8, 플랫폼 가로대 300×40",
    unit=40000, status="estimate (무료 정밀절단 판매처 사례: metalmarket 10T 판재 snippet, 가격 미확인)",
    purpose="헤드·Z 캐리지·프레임 체결부 (3D 프린트 금지 부위)",
    notes=f"{TAG} Red Team M3·H2: 앵글 40×40×4(L89) 대신 10T 브래킷, 누락 품목(X 모터 마운트, X 너트 브래킷, Z 모터 마운트, 플랫폼 가로대) 포함. "
          "구멍은 레이저컷 MDF 템플릿으로 드릴, M5·M3 손 탭. 판매처 절단이 유료면 공작실 절단(원래 C17) 검토 (기계 부품 M11)")
put("L88", spec="M6 관통볼트·너트·큰 와셔 ×30(Z 클램프 4×2, 받침·브래킷, 셀 마운트), M6 세트스크루 + 잠금너트 ×8(스톱 케이지), "
                "M5 인서트 너트 ×30(SBR16), M5·M3 볼트 소량, 목공 나사 4×40 1갑, 내수 목공 본드(D3) 500 g, 2액 에폭시 약 200 g(베딩)",
    unit=25000, status="estimate",
    notes=f"{TAG} 에폭시 베딩(구멍 여유 + 에폭시 충전, 조립 순서는 기계 문서 §9)으로 양 증가, 스톱 케이지 세트스크루 추가. "
          "[기계 합판 하이브리드] 나무–나무는 본드 + 나사")
put("L89", part="[RT1·미포함] 알루미늄 앵글 40×40×4 × 1 m", qty="0", status="estimate",
    notes=f"{TAG} 다리 외팔 굽힘(+0.05–0.65 mm @ 100 N, Red Team M3) → L20 10T 브래킷으로 대체, 미구매 (원안 7,000원)")
put("L63", part="가드 정면 창 투명 아크릴 3T 재단 470×520", spec="1장, 경첩 (옆면은 합판 측판 L86이 가드 겸용)", qty="1", unit=15000,
    status="estimate (snippet 900×600 27,600원의 면적 비례 + 재단비)",
    notes=f"{TAG} 창 하단을 팬 테두리 + 130으로 올려(홀더 상면과 끼임 간격 ≥ 100 mm, 기계 문서 §8) 창이 462×510으로 줄어듦. "
          "X 나사 끝·Z 커플러 덮개는 3D 프린트(L66). [빼지 않음] 학교 레이저커터로 가공 (기계 부품 M18)")
put("L48", spec="엔드스탑 모듈 또는 레버 마이크로스위치; 플래그는 3D 프린트", qty="5",
    purpose="Z-높이(A0), X-창(A1), 레인 위치 3개(−38 / 0 / +38) — Nano 입력 배정은 전자·제어 확인",
    status=SNIP,
    notes=f"{TAG} 레인 스위치 1 → 3(어느 구멍인지 확인, DECISIONS 2026-09-28 수동 인덱스 절차). [빼지 않음] 실제 위치를 보는 R7 인터록")
put("L72", spec="2 L 생수 ×7 (≈14.3 kg, 빌린 주방 저울로 개별 계량) + 소형 도르래 + 끈 + S고리", unit=9000, status="estimate",
    notes=f"{TAG} 6 → 7병: 1.1 × F_STOP 132 N + 도르래 마찰 5 %(가정) → 14.2 kg 필요(기계 출력 §6.8). 푸시풀 게이지 대신. X 셀은 도르래로 수평 하중")
r37 = body[idx["L37"]]
if TAG not in r37[14]:
    r37[14] = re.sub(r"\[조건부 절감 C15:.*?총액 미반영\]\s*", "", r37[14])
    r37[14] = f"{TAG} C15(GN 1/3 전환) 기각 — 채움 ≤ 120 mm면 레버 547 mm·Z 행정 +30, 거짓 바닥을 넣어도 F_d 114 N에서 1 mm 초과(기계 출력 §6.9). " + r37[14]

# ---------------------------------------------------------------- new rows (after L33)
NEW = [
    ["L92", "Sensors", "합판 6T 600×1200 (2장째) — 힘 플랫폼 상판", "상판 350×300을 6T 2겹 접착(12 mm), 레이저컷(판스프링·스톱·링크 구멍); 남는 면적은 조립 지그",
     "1", "힘 플랫폼 상판 + 조립 지그", "10000", "10000", "목재 재단몰", "", "estimate", "국내 1–5일 (추정)", "No", "No",
     f"{TAG} Red Team H2 누락 품목(플랫폼 상판). 가로대는 L20 Al 10T. 방수 도장(L90)"],
    ["L93", "Sensors", "X 셀 링크 — 로드엔드 M6 ×2 + M6 전산볼트 100 + 너트", "강 로드엔드(축방향 힘만 전달)", "1",
     "상판 ↔ X 셀(데크 고정) — 수직 셀에 F_x가 가지 않게", "6000", "6000", "알리/국내", "", "estimate", "알리 7–20일 (추정·확인 필요)", "No", "No",
     f"{TAG} Red Team M7: 병렬 플랫폼(수직 셀은 판스프링 경유, X는 링크로 데크 직결), 스톱 케이지는 데크에"],
]
for nr in NEW:
    if nr[0] in idx:
        body[idx[nr[0]]] = nr
    else:
        body.insert(idx["L33"] + 1 + (1 if nr[0] == "L93" and "L92" in [b[0] for b in body] else 0), nr)
        idx = {r[0]: i for i, r in enumerate(body)}

# ---------------------------------------------------------------- sums
items = [r for r in body if r[0].startswith("L")]
buy = [r for r in items if r[0] != "L74" and not r[1].startswith("Borrowed")]
for r in buy:
    assert abs(round(float(r[4] or 0) * float(r[6] or 0)) - int(r[7])) <= 1, r[0]
sum_b = sum(int(r[7]) for r in buy)
applied = sum(int(m.group(1).replace(",", "")) for r in items for m in re.finditer(r"\[절감 적용 C\d+:[^\]]*?−([\d,]+)원", r[14]))
cond_rows = [(r[0], m.group(1), int(m.group(2).replace(",", ""))) for r in items
             for m in re.finditer(r"\[조건부 절감 (C\d+):.*?−([\d,]+)원 — 총액 미반영\]", r[14])]
COND_EXTRA = [("L73", "C18", 10000, "주문 묶음 배송 — L73 재산정 필요")]   # C17 dropped: L20 now assumes seller cutting
cond = sum(c[2] for c in cond_rows) + sum(c[2] for c in COND_EXTRA)
cont = int(next(r for r in items if r[0] == "L74")[7])
cond_txt = "; ".join(f"{c} −{v:,} ({i})" for i, c, v in cond_rows) + "; " + "; ".join(f"{c} −{v:,} ({i}, 행 미수정: {d})" for i, c, v, d in COND_EXTRA)


def sumrow(ID, part, val, note):
    r = [""] * 15
    r[0], r[1], r[2], r[7], r[14] = ID, "합계", part, str(val), note
    return r


body += [
    sumrow("SUM-A", "기계 확정안(H2R) 구매+배송 (절감 C1–C12 전)", sum_b + applied, f"Chief 채택 절감 3·5 포함, [기계 합판 하이브리드] {TAG} 반영"),
    sumrow("SUM-B", "적용 절감 C1–C12 후 구매+배송", sum_b,
           f"R8 회계(DECISIONS 2026-09-28): 구매+배송 ≤ 900,000 {'충족' if sum_b <= LIMIT_BUY else '초과'} (여유 {LIMIT_BUY - sum_b:,}원) {TAG}"),
    sumrow("SUM-C", "예비비", cont, "≥ 100,000 유지 조건"),
    sumrow("SUM-D", "총액 (SUM-B + 예비비)", sum_b + cont, f"R8 100만 원 {'이내' if sum_b + cont <= 1_000_000 else '초과'}"),
    sumrow("SUM-E", "조건부 절감까지 실현 시 구매+배송", sum_b - cond, "조건부: " + cond_txt),
    sumrow("SUM-F", "SUM-E + 예비비", sum_b - cond + cont, ""),
]

# ---------------------------------------------------------------- scope check: only the listed rows changed
allowed = {"L01", "L02", "L03", "L86", "L87", "L17", "L20", "L88", "L89", "L63", "L48", "L72", "L37", "L92", "L93"}
for r in body:
    if r[0] in before and r[0] not in allowed:
        assert r == before[r[0]], f"unexpected change {r[0]}"
with open(BOM, "w", encoding="utf-8", newline="") as fh:
    csv.writer(fh, lineterminator="\r\n").writerows([hdr] + body)
delta = {ID: int(body[idx[ID]][7]) - int(before[ID][7]) for ID in allowed if ID in before and int(body[idx[ID]][7]) != int(before[ID][7])}
print("changed totals:", {k: f"{v:+,}" for k, v in sorted(delta.items())}, "| new rows L92 10,000, L93 6,000" if "L92" not in before else "")
print(f"SUM-A {sum_b + applied:,} | SUM-B {sum_b:,} | SUM-C {cont:,} | SUM-D {sum_b + cont:,} | SUM-E {sum_b - cond:,} | SUM-F {sum_b - cond + cont:,}")
print("conditional:", cond_txt)
