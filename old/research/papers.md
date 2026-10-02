# research/papers.md — 논문·기술문헌 근거 대장

## 0. 이 문서를 읽는 법 (반드시 먼저)

**조사 환경의 한계 (2026-09-26 세션):**
- 클라우드 세션의 네트워크 정책 때문에 **원문 페이지 열람(WebFetch)이 전면 차단**되었다. PubMed, PMC, ScienceDirect, ResearchGate, Google Patents, Wikipedia 모두 차단.
- 사용 가능한 것은 검색엔진 결과(제목·요약 snippet)뿐이었고, 그것도 세션당 **200회 검색 한도**에 도달해 중단되었다.
- 따라서 아래 "확인"은 **원문 확인이 아니라 검색결과 요약 수준의 확인**이다. 숫자를 발표자료에 쓰기 전에 학교 도서관에서 PDF를 받아 대조해야 한다(→ `docs/24_next_actions.md` P0-0).

**증거 태그**
| 태그 | 의미 |
|---|---|
| `[MULTI]` | 2개 이상 독립 검색결과에서 같은 내용 확인 |
| `[SNIPPET]` | 1개 검색결과 요약/발췌에서 확인 (URL 기재) |
| `[UNCERTAIN]` | 부분적·모호·요약 도구가 다른 문헌과 섞었을 가능성 |
| `[미검증]` | 이전 저장소 기록에만 있고 이번 세션에서 확인 못 함 |
| `[NOT FOUND]` | 검색했으나 찾지 못함 |
| `[사전지식-검증필요]` | 조사자의 배경지식. 출처 확인 전 인용 금지 |

---

## A. 작업부하·인간공학 근거

### A1. Dempsey et al. (2000) — 핵심 근거
| 항목 | 내용 |
|---|---|
| Title | Ergonomics investigation of retail ice cream operations `[MULTI]` |
| Authors | Dempsey PG, McGorry R(W?), Cotnam J, Braun TW `[SNIPPET]` — McGorry 중간 이니셜은 출처마다 R.R./R.W.로 다름 `[UNCERTAIN]` |
| Year / Journal | 2000, *Applied Ergonomics* 31, 121–130. 호(issue) 2는 직접 확인 못 함 `[UNCERTAIN]` |
| DOI | 10.1016/S0003-6870(99)00043-5 (PII S0003687099000435와 일치) `[MULTI]` |
| URL | https://www.sciencedirect.com/science/article/abs/pii/S0003687099000435 · PubMed 10711974 |
| 소속 | Liberty Mutual Research Center 추정 `[UNCERTAIN]` |
| Study type | 현장 + 실험실 인간공학 평가("human:workplace model"), 보상청구자료 분석 병행 |
| Sample | 현장: 1개 매장. 인원 수는 `[NOT FOUND]`(기존 기록 "3명"은 미검증). 실험실: mock-up 아이스크림 chest, 9개 품목(아이스크림·요거트·소르베) `[SNIPPET]` |
| 측정장비 | 맞춤 제작 **instrumented ice cream scoop** — 계측 손잡이 + 계측 스쿱헤드, **grip force와 손잡이에 대한 moment** 측정 `[SNIPPET]` |
| Key findings (확인됨) | ① 스쿠핑 작업요구는 "문헌에 기록된 적 없음"이라 직접 계측 `[MULTI]` ② 주요 결함은 **스쿠핑에 수반되는 나쁜 자세와 높은 근력** `[SNIPPET]` ③ 온도가 **아이스크림 1 g당 적분 grip force**에 유의한 영향, 1개 품목 제외 모두 2차항 유의 `[SNIPPET]` ④ 보상청구: 원인별로 **동일평면 넘어짐이 비용 36%(건수 21%)로 1위**, 스쿠핑은 비용 ~15%(건수 ~13%), 신체부위별 wrist 비용 ~18%(건수 ~13%) `[UNCERTAIN]`(요약 1건, "비용(건수)" 순서는 해석) |
| Key findings (미검증) | 현장 3명 평균 grip 43/49/52% MGF, 실험실 36–69% MGF, 400건 이상 청구, 증상 설문 ~8명, "peak가 아니라 시간이 늘어 누적부하 증가" — **모두 이번 세션에서 확인 못 함** `[미검증]` |
| Limitations | 1개 매장, 소표본, 청구자료는 분모(노출 인원)가 없어 발생률 아님, 2000년 이전 도구·캐비닛 |
| How we use it | (1) 스쿠핑이 **측정 가능한 높은 근력 작업**이라는 근거. (2) 온도 ↓ → g당 누적 grip ↑ → 우리 `W = u·V` 모델(docs/07)과 일치하는 방향. (3) **"스쿠핑이 청구의 주원인"이라고 쓰면 안 된다** — 1위는 넘어짐. (4) 계측 스쿱 설계를 우리 P0 계측의 참고로 사용 |

### A2. Dempsey et al. (1998) — 동반 학회논문
| 항목 | 내용 |
|---|---|
| Title | Comprehensive Ergonomics Evaluation of Retail Ice Cream Shops `[MULTI]` |
| Venue | Proc. Human Factors and Ergonomics Society Annual Meeting, 1998 (42) |
| DOI | 10.1177/154193129804201220 `[SNIPPET]` |
| Content | 심리물리 데이터 기반 MMH 분석, 능동/수동 감시, **스쿠핑 힘 정량화**, 인체측정·작업장 설계 평가 `[MULTI]` |
| Caution | 한 요약에 "100명, 42% 증상"이 나왔으나 **다른 연구(계산대)와 섞인 것으로 보임 → 사용 금지** |
| How we use it | A1과 같은 연구 프로그램. 원문 확보 시 스쿠핑 힘의 절대값(N) 확인용 |

### A3. Howard & Conrad (1992) — 사례보고
| 항목 | 내용 |
|---|---|
| Title | Ice-cream scooper's hand: report of an occupationally related stress fracture of the hand `[SNIPPET]` |
| Journal | *Clinical Nuclear Medicine* 17:721–723 (1992년 9월호) `[SNIPPET]` |
| Sample | 17세 여성 스쿠퍼 1명 |
| Finding | 스쿠핑하는 오른손 **제2중수골 피로골절**. 저자들은 딱딱한 아이스크림을 뜰 때 손잡이의 받침점(fulcrum)으로 그 뼈를 반복 사용한 것이 원인이라고 추정 |
| Limitation | 단일 사례, 인과 아님 |
| How we use it | **손잡이-손 접촉응력**이라는 구체적 기전의 예시. "손잡이 모멘트를 손으로 받치는 구조" 자체가 위험요인일 수 있음 → 반력을 프레임으로 접지(grounding)하는 설계 논리의 보조 근거. 질병 예방 주장에는 사용 금지 |

### A4. Shin (2019) — 국내 질적 사례
| 항목 | 내용 |
|---|---|
| Title | Mystery Shopping and Well-Being of Service Workers in South Korea |
| Journal | *Safety and Health at Work* 10(4):476–481, PMC6933262 `[미검증]` |
| Content | Case 15: 대형 아이스크림 프랜차이즈 파트타임 종사자가 정해진 형태를 만드는 반복 스쿠핑에서 손·팔·손목 부담을 언급 `[미검증]` |
| Status | **이번 세션에서 검색 예산 소진으로 확인 못 함.** 저장소 기존 기록을 그대로 옮김 |
| How we use it | "국내에도 같은 호소가 있다"는 **질적 사례** 수준. 빈도·인과 근거로 쓰지 않음. 발표 전 원문의 해당 문단 인용 확인 필수 |

### A5. 보상청구·뉴스 (일화적)
| 출처 | 내용 | 태그 |
|---|---|---|
| AOL News, 2010-11-17 "Scooping Ice Cream Now Considered a Potential Job Hazard" | 캐나다 앨버타 편의점 직원의 반복 스쿠핑 어깨 재손상 WCB 청구가 항소에서 승인, 회전근개 수술 | `[SNIPPET]` |
| Workers Comp Insider (2010-11) "The Scoop on Scooping" | URL만 확인 | `[UNCERTAIN]` |
| namu.wiki "배스킨라빈스 아르바이트" | 매 스쿱 후 물에 헹굼, 저울로 중량 확인, 둥근 모양이 숙련 기준 | `[UNCERTAIN]` 커뮤니티 위키 |
| Threads @yunina_kr | 중량 미달이면 스티커가 출력 안 됨(1인 게시글) | `[UNCERTAIN]` |
| BR Korea 보도자료 seq=1264, 한국경제 2024-11 | '스쿱 마스터' 우수 아르바이트 분기 포상(2024-11~) | `[MULTI]` |

### A6. 한국 데이터 — 공백
- 국내 아이스크림 판매 종사자의 근골격계 질환 발생률: `[NOT FOUND]` (검색 예산 소진으로 KOSHA·산재통계 미검색)
- 매장당 하루 스쿱 수, 피크시간 스쿱 수: `[NOT FOUND]` → **현장조사 필요** (docs/24 P0-1)
- 한국 매장의 디핑 캐비닛 설정온도: `[NOT FOUND]`

### A7. 손/손목 노출평가 프레임워크 — `[사전지식-검증필요]`
실험 결과를 "위험 수준"으로 해석할 때 쓸 후보. **원문 확인 전에는 수식·임계값을 발표에 쓰지 않는다.**
| 도구 | 입력 | 우리 용도 |
|---|---|---|
| ACGIH TLV for Hand Activity (HAL) | HAL(0–10), 정규화 peak force(NPF, 0–10) | 수동 vs 장치 조건을 HAL–NPF 평면에 찍어 TLV/Action Limit 대비 위치 비교 |
| Strain Index (Moore & Garg 1995) / Revised SI (Garg et al. 2017) | %MVC 강도, 노력시간, 분당 횟수, 자세, 속도, 일일시간 | 동일 작업의 조건 간 상대비교 |
| OCRA (ISO 11228-3) | 기술동작 수/분 | 반복성 비교 |

세 도구 모두 힘을 **%MVC**로 받으므로, P0 계측에서 참가자 최대악력(MGF/MVC)을 반드시 함께 측정해야 한다.

---

## B. 아이스크림 물성·온도 근거

### B1. 권장 디핑 온도
| 출처 | 내용 | 태그 |
|---|---|---|
| IDFA "Tips on storing & handling ice cream" | **6–10 °F(−14.4 ~ −12.2 °C)에서 뜨기 쉬움**(이상적 서빙 범위), 보관 −5 ~ 0 °F(−20.6 ~ −17.8 °C), 10 °F 초과 시 body/texture/flavor 악화 | `[SNIPPET]` https://www.idfa.org/news-views/media-kits/ice-cream/tips-on-storing-handling-ice-cream |
| 장비 판매사 가이드 (gofoodservice, webstaurantstore) | 디핑 캐비닛 6–10 °F, 유지방·당 함량에 따라 조정 | `[MULTI]` |
| Goff, *Ice Cream Technology e-Book* (U. Guelph) | 전형적 서빙온도 −16 °C | `[SNIPPET]` https://books.lib.uoguelph.ca/icecreamtechnologyebook/ |
| Tetra Pak *Dairy Processing Handbook* | 경화 −18 °C 이하, 보관 −25 °C 이하 | `[SNIPPET]` https://dairyprocessinghandbook.tetrapak.com/chapter/ice-cream |

**함의:** 저장(−18 ~ −25 °C)과 디핑(−12 ~ −14 °C)은 다른 온도대다. 매장이 디핑 온도를 지키지 못하는 이유(회전율, 품목 간 경도 차이, 녹음 방지)가 현장에서 확인되어야 "온도관리만으로 충분한가"에 답할 수 있다.

### B2. 빙결률(frozen water fraction) vs 온도
| 온도 | 얼어 있는 물 | 출처 | 태그 |
|---|---|---|---|
| 믹스 어는점 약 −2.5 °C | 0 % | PMC7913915 등 | `[SNIPPET]` |
| 프리저 토출 −5 ~ −7 °C | ~50 % | Goff e-Book | `[MULTI]` |
| 서빙 −16 °C | ~72 % | Goff e-Book | `[SNIPPET]` |
| 서빙 적정 | "65–70 % 이하면 뜰 수 있음" | Mullan, *Food Science and Technology* (IFST), DOI 10.1002/fsat.3510_3.x (연도 2018/2021 불일치 `[UNCERTAIN]`) | `[MULTI]` |
| −18 °C | "~80 %" | 출처 불명확 | `[UNCERTAIN]` |
| 업계 경험칙 | <55 % 형태 유지 불가, >78 % 템퍼링 없이 뜨기 어려움 | icecreamcalc.com (비동료심사) | `[SNIPPET]` |

**함의:** 경도는 빙결률에 대해 **지수적으로** 증가한다(Wilbey et al. 1998, 2차 인용 `[SNIPPET]`). 몇 °C 차이가 절삭저항 u를 배수로 바꿀 수 있다 → 우리 실험의 온도 통제 요구(±0.5 °C 수준)의 근거.

### B3. 경도 측정 문헌 (프로브 시험 — 스쿱 시험 아님)
| 문헌 | 방법/결과 | 태그 |
|---|---|---|
| Muse & Hartel (2004) *J Dairy Sci* 87(1):1–10 "Ice cream structural elements that affect melting rate and hardness" | 감미료·유화제·토출온도 요인설계, 경도-구조 회귀. **N 값 미확보** | `[SNIPPET]` |
| Sofjan & Hartel (2004) *Int Dairy J* 14(3):255–262 | overrun ↑ → 경도 ↓. N 값 미확보 | `[SNIPPET]` |
| Inoue et al. (2009) *J Dairy Sci* 92(12), PubMed 19923588 | **−5, −10, −15 °C 경도**를 penetrometer로 측정, overrun·토출온도 유의. 수치 미확보. 공개 PDF 추정 위치: didatticagelato.it | `[SNIPPET]` — **최우선 원문 확보 대상** |
| Briggs, Steffe & Ustunol (1996) *J Dairy Sci* 79(4):527–531 | vane법 **항복응력 2.5–8.0 kPa**, 온도 ↑ → 감소, 초콜릿 > 바닐라, scoopability와 상관 보고. **시험온도 미확보** | `[SNIPPET]` |
| US 8,609,174 / US20090263556A1 (특허, 방법 정의) | 6.35 mm 프로브, −20 °C, "spoonable"을 **< 30 N**으로 정의 | `[SNIPPET]` |
| WO2017001372A1 | −18 °C Vickers 경도 0.042–0.36 (단위 미확인) | `[UNCERTAIN]` |
| Unilever EP1158863B1 계열 | Vickers 시험, −18 °C 캐비닛, 2 mm/min, 최대 95 N | `[SNIPPET]` |
| Stable Micro Systems "Ice Cream Scoop Rig" | 텍스처 분석기에 스쿱을 달아 긁는 힘을 scoopability 지표로 사용. **공개 수치 없음** | `[MULTI]` |

**함의:**
- 공개 문헌에서 **"스쿱으로 뜰 때의 힘(N)·토크(N·m)"를 온도 조건과 함께 제시한 자료를 찾지 못했다.** → 이것 자체가 우리 P0/P1 실험의 기여점이다.
- 프로브 압력 추정(30 N / 31.7 mm² ≈ 0.95 MPa, −20 °C, "부드러운" 제품)은 측면마찰을 포함하므로 재료상수가 아니다. 스쿱 rim의 날 두께가 1 mm라면 날 끝 압입만으로 수십~100 N 규모가 나올 수 있다는 **자릿수 점검용**으로만 쓴다(docs/07).

### B4. 절삭저항 저감 문헌
| 문헌 | 내용 | 태그 |
|---|---|---|
| US 5,819,615 "Cutting process"(추정) | 초음파 ON 시 **냉동과자 6 mm/s 절단 peak force가 OFF 대비 ~16 %(≈84 % 감소)**, 냉동어 블록 ~20 % | `[UNCERTAIN]` 출처 귀속 불확실 — 인용 전 원문 확인 |
| Schneider, Zahn, Rohm 등 "Ultrasonic cutting of foods…" (IFSET, J Food Eng) | 감소폭은 "제품에 크게 의존" | `[SNIPPET]` |
| 산업용 초음파 케이크 커터 (Cheersonic 등) | 판매사 마케팅, 데이터 없음 | `[SNIPPET]` |
| 가열 날, 와이어 절단, 날 각도·예리도, 스테인리스/PTFE와의 마찰계수 | 검색 예산 소진 | `[NOT FOUND]` |

### B5. 로봇·자동 스쿠핑 연구
| 문헌 | 내용 | 태그 |
|---|---|---|
| Columbia MECE E4602 학생 프로젝트 "Scoop" (jonathanblutinger.com/img/Group8-Scoop.pdf) | **7-DOF 로봇팔**이 특수 설계 통에서 하드아이스크림을 뜨고, 물에 헹굼. 동기: 파트타임 스쿠퍼의 건초염·손목터널 위험 | `[SNIPPET]` (비동료심사) |
| DiVA thesis "Design of Automatic Scoop System in Ice-cream Shop" | 제목만 확인 | `[SNIPPET]` |
| Columbia "Automating the Ice Cream Scooping Process"(사람 궤적 분석) | 제목·요약만 | `[SNIPPET]` |
| RIT Imagine "ice cream scooping robot", Dexai "Alfred" | 데모 | `[SNIPPET]` |
| 스쿠핑 힘/토크를 보고한 로봇 논문 | 검색 예산 소진 | `[NOT FOUND]` |

---

## C. 우리 주장에 대한 근거 강도 요약

| 주장 | 근거 강도 | 비고 |
|---|---|---|
| 하드아이스크림 스쿠핑은 높은 근력·나쁜 자세를 수반한다 | **중** | A1(초록 수준 확인), A2, A3 |
| 온도가 낮을수록 g당 누적 grip force가 커진다 | **중** | A1 Fig.5 요약 |
| 적정 디핑온도는 −12 ~ −14 °C 부근이다 | **중** | B1 복수 출처 |
| 한국 매장에서도 손목 부담 호소가 있다 | **약** | A4 미검증, A5 커뮤니티 |
| 스쿠핑이 산재 청구의 주원인이다 | **틀림** | A1: 넘어짐이 1위 |
| 스쿠퍼의 손목질환 발생률이 높다 | **근거 없음** | 분모 있는 자료 `[NOT FOUND]` |
| 스쿱 절삭력은 50–200 N 범위다 | **근거 없음(가정)** | 자릿수 점검만 가능 → P1 실측 |
| 초음파는 절삭력을 크게 줄인다 | **약–중** | 직선 날 기준, 스쿱 형상 미검증 |
