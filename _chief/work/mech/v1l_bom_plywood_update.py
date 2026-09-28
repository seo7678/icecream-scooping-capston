#!/usr/bin/env python3
"""Apply the mechanical plywood-hybrid frame (v1l_mech.md §13) to the V1-L BOM.

Run:  python3 _chief/work/mech/v1l_bom_plywood_update.py
Edits _chief/work/elec/v1l_bom.csv in place (idempotent):
  - Frame rows L01-L08 rewritten, new Frame rows L86-L90 inserted after L08
  - Safety rows L63 (guard size) and L64 (notes only)
  - SUM-A..SUM-F recomputed from the item rows
Other categories are copied through unchanged (checked at the end).

SUM rules (same as the electronics sheet, v1l_elec.md §7):
  SUM-B = Σ Total of purchase rows (all L rows except L74 contingency and 'Borrowed' rows)
  SUM-A = SUM-B + Σ "[절감 적용 Cn: …, −N원]" amounts in Notes (value before the applied savings)
  SUM-C = contingency (L74)
  SUM-D = SUM-B + SUM-C
  SUM-E = SUM-B − Σ "[조건부 절감 Cn: … −N원 — 총액 미반영]" amounts in Notes
          − conditional items that belong to rows this script may not edit (C17 L20, C18 L73; listed in the SUM-E note)
  SUM-F = SUM-E + SUM-C
"""

import csv
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
BOM = os.path.abspath(os.path.join(HERE, "..", "elec", "v1l_bom.csv"))
TAG = "[기계 합판 하이브리드]"

with open(BOM, encoding="utf-8", newline="") as fh:
    rows = list(csv.reader(fh))
hdr, body = rows[0], rows[1:]
assert len(hdr) == 15
orig = {r[0]: list(r) for r in body}


def row(ID, sub, part, spec, qty, purpose, unit, total, src, url, status, lead, borrow, food, notes):
    return [ID, sub, part, spec, qty, purpose, str(unit), str(total), src, url, status, lead, borrow, food, notes]


o = orig
VER = "verified 2026-09-28 (search snippet)"
DOM = "국내 1–5일 (추정)"
new_frame = [
    row("L01", "Frame", "알루미늄 프로파일 4080 (슬롯8, 중량) 절단 — Z 이동 기둥만",
        "Z 이동 기둥 500 ×1 = 0.5 m (교차빔 480 ×2는 합판 교차판 L86으로 대체)", "0.5",
        "Z 이동 기둥 (MGN12 레일 장착 — 금속 유지)", 40290, round(0.5 * 40290), o["L01"][8], o["L01"][9], o["L01"][10], o["L01"][11], "No", "No",
        f"{TAG} 교차빔을 합판으로 바꾸고 Z 기둥만 남김(원안 1.46 m 58,823원). "
        "[조건부 절감 C14: [조건부] 4080 판매처 kg단가 견적 (약 25,000원/m 가정), −7,645원 — 총액 미반영] "
        "검색 결과 DF4080(중량) 40,290원/m (기계 부품 M01)"),
    row("L02", "Frame", "[원안·미포함] 알루미늄 프로파일 2040 (슬롯6) 절단",
        "원안: 베이스·타워 다리·수평 타이·베드 틀 9.72 m", "0", "합판 18T(L86)로 대체",
        8800, 0, o["L02"][8], o["L02"][9], o["L02"][10], o["L02"][11], "No", "No",
        f"{TAG} 베이스·A-프레임 타워·베드 틀을 합판 18T로 대체 → 미구매 (원안 85,536원). C14 조건부 절감 중 2040 몫(32,076원)도 소멸"),
    row("L03", "Frame", "[원안·미포함] 알루미늄 프로파일 2020 (슬롯6) 절단",
        "원안: 베드 가로대·무릎 가새·가드 틀 2.82 m", "0", "합판 베드·뒷벽·측판으로 대체",
        2870, 0, o["L03"][8], o["L03"][9], o["L03"][10], o["L03"][11], "No", "No",
        f"{TAG} 옆힘은 합판 뒷벽(격막), 가드 틀은 합판 측판 모서리에 경첩 → 미구매 (원안 8,093원)"),
    row("L04", "Frame", "[원안·미포함] 20시리즈 연결 세트 (슬롯6)",
        "원안: L브라켓 20 + M5 T너트 40 + M5 볼트 40 / 세트 ×2", "0", "목재 체결 자재(L88)로 대체",
        12000, 0, o["L04"][8], o["L04"][9], o["L04"][10], o["L04"][11], "No", "No",
        f"{TAG} 20시리즈 프로파일이 없어짐 → 미구매 (원안 24,000원)"),
    row("L05", "Frame", "40시리즈 연결 부품 (슬롯8) — Z 기둥용",
        "M3 슬롯8 T너트 ×10(MGN12 레일), M8 끝단 볼트 ×2(헤드 브래킷, 중심구멍 탭), M5 T너트·볼트 ×4(Z 모터 브래킷)", "1",
        "4080 Z 기둥 체결", 5000, 5000, o["L05"][8], o["L05"][9], "estimate", o["L05"][11], "No", "No",
        f"{TAG} 교차빔–다리 체결 삭제, Z 기둥 체결만 (원안 12,000원) (기계 부품 M04)"),
    row("L06", "Frame", "[원안·미포함] 각도(피벗) 조인트", "원안: 20시리즈용 ×8", "0", "경사 다리 없음",
        2000, 0, o["L06"][8], o["L06"][9], o["L06"][10], o["L06"][11], "No", "No",
        f"{TAG} 합판 측판(직사각)이라 각도 조인트 불필요 → 미구매 (원안 16,000원)"),
    row("L07", "Frame", o["L07"][2], "4조 — 합판 베이스 리브 끝 4곳", o["L07"][4], "베이스 수평", o["L07"][6], o["L07"][7],
        o["L07"][8], o["L07"][9], o["L07"][10], o["L07"][11], "No", "No",
        f"{TAG} 위치만 변경(리브 끝). " + o["L07"][14]),
    row("L08", "Frame", "합판 6T (레이저컷용)",
        "600×1200 ×1 — 홀더 외피·팬 턱 받침·뚜껑, 알루미늄판 드릴 템플릿 (베드 데크는 18T L86으로 이동, X 스톱 블록은 18T 자투리)", "1",
        "홀더·템플릿", 10000, 10000, o["L08"][8], o["L08"][9], o["L08"][10], o["L08"][11], o["L08"][12], "No",
        f"{TAG} 2장 → 1장 (원안 20,000원). 레이저커터 작업 크기에 맞춰 재단 (기계 부품 M16)"),
    row("L86", "Frame", "구조용 내수 합판 18T 1220×2440 (CP)",
        "1장 전부 사용(재단도: 기계 출력 §10.6): 측판 500×939 ×2, 베이스 1000×560 + 리브 80×1000 ×3, 베드 데크 650×400, "
        "뒷벽 462×580, 교차판 462×270 ×2 + 앞 립 462×90 ×2, 클리트 60폭", "1",
        "브리지(측판·교차판·뒷벽)·베이스·베드 — 2040·2020·4080 교차빔 대체",
        28500, 28500, "대림목재 단가표 (CP합판 구조용 내수·방수 18T)",
        "https://daelimwood.com/category/%ED%95%A9%ED%8C%90%EB%B3%B4%EB%93%9C/331/", VER, DOM, "No", "No",
        f"{TAG} 검색 결과 CP 구조용 내수·방수 합판 18T 28,500원(1220×2440). 재단 여유 없음 → 판매처 재단(L87). "
        "대형 재단품 배송은 L73 안에서 프로파일 판매처 2곳(L02·L03)을 대체한다고 가정 — 판매처 배송비 확인 필요. 전 면 방수 도장(L90)"),
    row("L87", "Frame", "합판 재단비 (판매처 패널쏘)", "직선 16컷 (대각 없음, 재단도: 기계 출력 §10.6)", "16",
        "L86 재단", 1000, 16000, "국내 목재 재단 서비스 (검색 요약, 판매처 미특정)", "",
        "estimate (search snippet: 합판 재단 1컷 1,000원)", DOM, "No", "No",
        f"{TAG} [조건부 절감 C16: [조건부] 공작실 원형톱·테이블쏘로 직접 재단, −16,000원 — 총액 미반영] 판매처별 재단비 확인 필요"),
    row("L88", "Frame", "목재 체결 자재",
        "M6 관통볼트·너트·큰 와셔 ×24(Z 블록 마운트 6×2, 셀 마운트 4, 예비), M5 인서트 너트 ×30(SBR16·KFL08·너트 브래킷), "
        "목공 나사 4×40 1갑, 내수 목공 본드(D3) 500 g, 2액 에폭시 소형", "1",
        "금속–나무·나무–나무 체결", 20000, 20000, "국내 철물", "", "estimate", DOM, "No", "No",
        f"{TAG} 볼트 구멍은 볼트 지름과 같게(여유 0) — 여유가 있으면 미끄럼(히스테리시스)이 §4.2 보상 조건을 깬다. 나무–나무는 본드 + 나사"),
    row("L89", "Frame", "알루미늄 앵글 40×40×4 × 1 m",
        "Z 블록 마운트 2곳(교차판 앞 가장자리, M6 6개씩), Z 너트 브래킷 받침, KFL08 받침", "1",
        "금속–나무 경계 부품", 7000, 7000, "국내 금속 재단몰", "", "estimate", DOM, "No", "No",
        f"{TAG} 에폭시 + 볼트로 합판에 고정 권장. L20의 'Z 블록 마운트판'은 이 앵글로 대체 가능(C17)"),
    row("L90", "Frame", "방수 도장 — 수성 우레탄 바니시 0.5 L + 붓", "전 합판 2회, 절단면(마구리) 먼저", "1",
        "결로·물방울 대책", 9000, 9000, "국내", "", "estimate", DOM, "No", "No",
        f"{TAG} 차가운 팬·스쿱 주변 물방울 → 베드 데크·베이스 상면·측판 안쪽 우선"),
]
new_safety = {
    "L63": row("L63", "Safety", "가드 정면 창 투명 아크릴 3T 900×600",
               "1장 → 정면 창 462×580(경첩) + 나머지 438×600에서 X 나사 끝·Z 너트 덮개. 옆면은 합판 측판(L86)이 가드 겸용", "1",
               "브리지 정면 관찰창 (측면은 합판 측판)", 27600, 27600, o["L63"][8], o["L63"][9], o["L63"][10], o["L63"][11], "No", "No",
               f"{TAG} 2장 → 1장(원안 55,200원). 브리지 옆 핀치 구간은 18T 측판 500×939가 막음, 베드–측판 틈 31 mm. "
               "[빼지 않음] 학교 레이저커터로 절단(폴리카보네이트는 레이저 불가) (기계 부품 M18)"),
    "L64": row("L64", "Safety", o["L64"][2], "정면 창 경첩 1조 + 손잡이 + 자석 캐치", o["L64"][4], "정면 창 설치", o["L64"][6], o["L64"][7],
               o["L64"][8], o["L64"][9], o["L64"][10], o["L64"][11], "No", "No", f"{TAG} 정면 창용. [빼지 않음] (기계 부품 M18)"),
}

# ---------------------------------------------------------------- rebuild the body
frame_ids = {r[0] for r in new_frame}
out = []
inserted = False
for r in body:
    ID = r[0]
    if ID.startswith("SUM"):
        continue
    if ID in frame_ids:
        if not inserted:
            out.extend(new_frame)
            inserted = True
        continue
    if ID in new_safety:
        out.append(new_safety[ID])
        continue
    out.append(r)
assert inserted

# ---------------------------------------------------------------- recompute sums
items = [r for r in out if r[0].startswith("L")]
buy = [r for r in items if r[0] != "L74" and not r[1].startswith("Borrowed")]
for r in buy:   # qty × unit consistency (rounded) for every purchase row
    q, u, t = float(r[4] or 0), float(r[6] or 0), int(r[7])
    assert abs(round(q * u) - t) <= 1, (r[0], q, u, t)
sum_b = sum(int(r[7]) for r in buy)
applied = sum(int(m.group(1).replace(",", "")) for r in items
              for m in re.finditer(r"\[절감 적용 C\d+:[^\]]*?−([\d,]+)원", r[14]))
cond_rows = [(r[0], m.group(1), int(m.group(2).replace(",", ""))) for r in items
             for m in re.finditer(r"\[조건부 절감 (C\d+):.*?−([\d,]+)원 — 총액 미반영\]", r[14])]
COND_EXTRA = [("L20", "C17", 18000, "알루미늄판 공작실 재단·드릴(판재만 구매 약 12,000원 가정)"),
              ("L73", "C18", 10000, "주문 묶음 배송(프로파일 판매처 3→1곳, 알리 묶음) — L73 재산정 필요")]
cond = sum(c[2] for c in cond_rows) + sum(c[2] for c in COND_EXTRA)
contingency = int(next(r for r in items if r[0] == "L74")[7])
sum_a = sum_b + applied
sum_e = sum_b - cond
cond_txt = "; ".join(f"{c} −{v:,} ({i})" for i, c, v in cond_rows) + "; " + \
    "; ".join(f"{c} −{v:,} ({i}, 행 미수정: {d})" for i, c, v, d in COND_EXTRA)
blank = [""] * 15


def sumrow(ID, part, val, note):
    r = list(blank)
    r[0], r[1], r[2], r[7], r[14] = ID, "합계", part, str(val), note
    return r


out += [
    sumrow("SUM-A", "기계 확정안(합판 하이브리드) 구매+배송 (절감 C1–C12 전)", sum_a, f"Chief 채택 절감 3·5 포함, {TAG} 반영"),
    sumrow("SUM-B", "적용 절감 C1–C12 후 구매+배송", sum_b, f"목표 88만 원 {'충족' if sum_b <= 880000 else '초과'} (여유 {880000 - sum_b:,}원) {TAG}"),
    sumrow("SUM-C", "예비비", contingency, "하한 10만 원"),
    sumrow("SUM-D", "총액 (SUM-B + 예비비)", sum_b + contingency, f"R8 100만 원 {'이내' if sum_b + contingency <= 1_000_000 else '초과'}"),
    sumrow("SUM-E", "조건부 절감 C13–C18까지 실현 시 구매+배송", sum_e, "조건부: " + cond_txt),
    sumrow("SUM-F", "SUM-E + 예비비", sum_e + contingency, ""),
]

# ---------------------------------------------------------------- untouched-row check
touched = frame_ids | set(new_safety) | {"SUM-A", "SUM-B", "SUM-C", "SUM-D", "SUM-E", "SUM-F"}
for r in out:
    if r[0] not in touched:
        assert r == orig[r[0]], f"row {r[0]} changed unexpectedly"
for ID in orig:
    if ID not in touched:
        assert any(r[0] == ID for r in out), f"row {ID} lost"

with open(BOM, "w", encoding="utf-8", newline="") as fh:
    csv.writer(fh, lineterminator="\r\n").writerows([hdr] + out)

frame = sum(int(r[7]) for r in items if r[1] == "Frame")
guard = sum(int(r[7]) for r in items if r[0] in ("L63", "L64"))
print(f"Frame {frame:,} | guard L63+L64 {guard:,} | SUM-A {sum_a:,} | SUM-B {sum_b:,} | SUM-C {contingency:,} | "
      f"SUM-D {sum_b + contingency:,} | SUM-E {sum_e:,} | SUM-F {sum_e + contingency:,}")
print("conditional:", cond_txt)
