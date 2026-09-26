# research/components.md — 갠트리 부품 카탈로그 값

> 검색결과 snippet 기반(원문 PDF 미열람, 2026-09 세션). 태그: `[SNIPPET]` 한 출처 · `[MULTI]` 복수 출처 일치 · `[UNCERTAIN]` 열/의미 불확실 · `[DERIVED]` 태그된 값에서 계산 · `[NOT FOUND]`
> 이 값들은 `calc/scoop_load_path.py`, `calc/gantry_motor_sizing.py`, `calc/z_axis_sizing.py`에 반영되어 있다. 부품을 확정할 때 제조사 PDF로 다시 확인한다.

## 1. 프로파일 레일 블록 (정적 정격)

| 블록 | C0 | M0R | M0P | M0Y | 태그 / 출처 |
|---|---|---|---|---|---|
| MGN12H | 5.88 kN | 38.22 N·m | 36.26 N·m | 36.26 N·m | `[SNIPPET]` amazon.com/dp/B0DJNM5BVZ (HIWIN 표 순서로 해석) |
| MGN15H | 9.11 kN? | – | – | – | `[UNCERTAIN]` dold-mechatronik MGN 데이터시트 |
| HGH15CA | 16.97 kN (C = 11.38 kN) | 0.12 kN·m | ~0.10 kN·m | ~0.10 kN·m | C, C0, M0R `[SNIPPET]` hiwin.de, lm76.com HG 카탈로그 / M0P·M0Y `[UNCERTAIN]` |
| HGH20CA | 27.76 kN (C = 17.75 kN) | 0.27 kN·m | 0.20 kN·m | 0.20 kN·m | C, C0 `[SNIPPET]` / 모멘트 `[UNCERTAIN]` |

블록 **선형 강성**(N/µm)은 `[NOT FOUND]` → 계산에서는 가정값(MGN12 60 N/µm, HG15 200 N/µm)을 쓴다.

## 2. 타이밍 벨트

| 벨트 | 값 | 태그 / 출처 |
|---|---|---|
| Gates 2M GT2, 25.4 mm 폭 | 권장 작업장력 111 N, 극한강도 5,338 N, 인장계수 18,000 lbf/(in/in) | `[SNIPPET]` MIT BeltTensileProperties.pdf |
| GT2 6 mm | 작업장력 ~28 N, 파단 ~550 N | `[MULTI]` shapeoko wiki, handsontec |
| GT2 6 mm 강성 | ≈ 18.9 kN / 단위변형률 → 1 m 스팬 ≈ 19 N/mm | `[DERIVED]` Gates 값 폭 비례 |
| GT2 10 mm 강성 | ≈ 31.5 kN / 단위변형률 | `[DERIVED]` |
| HTD 5M, 25.4 mm 폭 | 454 N (작업장력 추정) → 15 mm 폭 ≈ 268 N | `[UNCERTAIN]` |
| HTD 8M, 25.4 mm 폭 | 792 N → 20/30 mm 폭 ≈ 624/935 N | `[UNCERTAIN]` |
| HTD·AT10 강성 | – | `[NOT FOUND]` (가정 150 kN/단위변형률 사용) |

**결론:** GT2는 드래그 200 N을 **강도로도 강성으로도** 받을 수 없다(작업장력 ~28 N, 1 m 스팬에서 200 N이면 ~10 mm 신장).

## 3. 볼스크류

| 항목 | 값 | 태그 |
|---|---|---|
| 임계속도 | N1 = λ2 · d_r / L² · 10⁷ [rpm], λ2: 고정-자유 3.4, 지지-지지 9.7, **고정-지지 15.1**, 고정-고정 21.9 — **λ2에 안전계수 0.8이 이미 포함** | `[SNIPPET]` THK A15-32 (재계산으로 확인 `[DERIVED]`) |
| DN 한계 | N2 = 70,000 / D (D = 볼 중심 지름) | `[SNIPPET]` THK |
| SFU1605/1610/2010 골지름 | – | `[NOT FOUND]` → 호칭경 − 볼 지름으로 추정(12.8 / 16.8 mm) |

## 4. 알루미늄 프로파일 단면

| 프로파일 | Ix [cm⁴] | Iy [cm⁴] | It [cm⁴] | 기타 | 태그 / 출처 |
|---|---|---|---|---|---|
| Misumi HFS8-4040 | 10.4 | 10.4 | – | A 640 mm², 1.73 kg/m | `[SNIPPET]` misumi p2319/p2315 |
| Bosch Rexroth 40×40L | 9.7 | 9.0 | 2.6 | – | `[SNIPPET]` esd.equipment |
| Misumi HFS8-4080 | 19.8 | 71.9 | – | A 1,109 mm² | `[SNIPPET]` |
| Misumi HFS8-8080 | 129.1 | 129.1 | – | A 1,691 mm², 4.57 kg/m | `[SNIPPET]` |
| 4080 / 8080 비틀림 상수 | – | – | – | – | `[NOT FOUND]` → 가정 6 / 40 cm⁴ |

## 5. 모터·브레이크

| 품목 | 값 | 태그 |
|---|---|---|
| Leadshine CS-M22331 (NEMA23 폐루프) | 3.1 N·m, 5 A, 0–1,900 rpm | `[SNIPPET]` |
| 57HSE3N + HBS57 | 3 N·m, 정격 1,000 rpm | `[SNIPPET]` |
| NEMA23 속도별 토크 | – | `[NOT FOUND]` → 계산은 일반 곡선 가정(1,500 rpm에서 35 %) |
| StepperOnline NEMA23 "-B280" 무여자 브레이크 | 2.8 N·m, 24 V, 4.5 W | `[SNIPPET]` |
| StepperOnline SWB-01 (NEMA17 브레이크) | 0.25 N·m, 24 V | `[SNIPPET]` |

## 6. 로봇팔 가격 기준점 (Cartesian 비용 비교용)

| 로봇 | 가격 | 태그 |
|---|---|---|
| UR5e | USD 35,000–48,000 (미국 판매처, 2026) | `[SNIPPET]` robotsourced, standardbots |
| Doosan M0609 | 44,770,000원 (DC 컨트롤러 포함 46,948,000원) | `[SNIPPET]` hskorea7 |
| Rainbow RB5-850 | 44,000,000원 (한 판매처) | `[SNIPPET]` |
| XYZ ARIS 3.0 (완제품) | 2천만 원대 | `[MULTI]` research/products.md |

로봇 본체만 4천만 원대이고, 시스템 통합비는 본체 가격만큼 더 든다는 판매처 설명이 있다 `[SNIPPET]`.
