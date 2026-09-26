# research/products.md — 기존 제품·로봇·시장 맥락

> 모든 항목은 검색결과 snippet 기반(원문 페이지 미열람). 태그: `[SNIPPET]` `[MULTI]` `[UNCERTAIN]` `[NOT FOUND]` `(분석)`
> 가격은 **확인된 것만** 적었다. 확인 못 한 가격은 비워 두었다.

## A. 수공구·매장 설비

| Product | Company | URL | Mechanism | Price (확인된 것) | Automation | 남는 작업부하 | Weakness |
|---|---|---|---|---|---|---|---|
| 상업용 디셔(트리거/스퀴즈) | Hamilton Beach, Vollrath | vollrathfoodservice.com (color-coded dishers) | 고정 부피 볼 + 스프링 스윕 블레이드. Vollrath #8=4 oz, #12=2.66 oz, #16=2 oz `[SNIPPET]` | – | 없음 | 절입·전단·손목 회전 전부 + **트리거 악력 추가**(분석) | 딱딱한 제품에서 볼·트리거가 약점(분석) |
| **Zeroll Original** | Zeroll (The Legacy Companies) | zeroll.com, centralrestaurant.com | 손잡이에 밀봉된 **열전도 유체**가 손의 열을 볼로 전달, 가동부 없음. 제조사 주장: 압축 제거, 갤런당 최대 20 % 더 많은 스쿱 `[MULTI]` | **#20: US$36.05** `[SNIPPET]` | 없음 | 절입력·손목 동작 | **손세척만, 식기세척기 금지, 140 °F 초과 금지** `[MULTI]` |
| BR 브랜드 푸시버튼 스쿱 (Norpro 685) | Norpro(라이선스) | amazon B00005EBHA | 버튼으로 진공 해제, 2인치 공 `[SNIPPET]` | – | 없음 | 절삭력 전부 | 소비자용. **한국 BR 매장 사용 여부 미확인** |
| Midnight Scoop | Midnight Kitchen Tools | newatlas.com/midnight-scoop… | 손바닥에 얹는 두꺼운 곡선 손잡이, 손목 대신 팔·가슴으로 밀기, 뾰족한 끝 `[MULTI]` | – | 없음 | **총 힘은 그대로, 경로만 바뀜** | 독립 연구 없음(제조사 주장) |
| OXO / Tovolo / Spring Chef | 각 사 | oxo.com, tovolo.com, springchef.com | 뾰족 끝, 열 보유 아연 헤드 등 `[SNIPPET]` | – | 없음 | 전부 | 소비자용 |
| **충전식 가열 스쿱** (GVODE, Salton, Numhew 등) | 다수 | amazon, bestbuy, salton.com | 헤드 **~70 °C(158 °F)** 가열, 30 s–1 min 예열, 배터리 2,000–5,000 mAh, **~30회 또는 70–100분/충전**, 일부 113/140 °F 2단, IP67/68 `[MULTI]` | – | 가열만 | 절입력(일부만 감소), 손목, 정량 | **전부 소비자용**, NSF 표시 미확인, 교대근무 지속성 미검증(분석) |
| 저유량/가열 dipper well | Server Products ConserveWell 87740/87750/87760/87770 | gofoodservice.com, katom.com | 스쿱을 물속에 보관, 연속유수 대체(연 ~600 gal 주장), 87770은 가열+타이머 `[SNIPPET]` | **US$490–689** `[SNIPPET]` | 없음 | 스쿠핑 힘 전부 | 매장 표준 인프라 |
| 연속유수 dipper well | – | – | 흐르는 물 | – | 없음 | – | 물 낭비(2008 Starbucks ~2,340만 L/일 논란 `[MULTI]`) |
| **동력(모터 운동) 스쿱** | – | scoopcollector.com/electric-scoops | **현재 시판품 없음.** "electric ice cream scoop" 검색결과는 모두 가열 스쿱. 1926년 전기기계식 스쿱 특허, 초기 전기 스쿱은 가열제어·세척 문제 `[SNIPPET]` | – | – | – | **90년간 특허는 반복, 상용 생존 제품 없음**(분석, docs/03) |
| 진동/오실레이팅 스쿱, 젤라토 스페이드 | – | – | `[NOT FOUND]` (예산 소진) | – | – | – | – |

부가 정보:
- 디셔 번호 ≈ 32 fl oz ÷ N (예: #8≈118 mL, #16≈59 mL). **그램은 밀도(overrun) 의존이라 계산하지 않음.** Zeroll의 "oz"는 이 규칙과 맞지 않음 → 명목값으로만 취급(분석).
- 한국산 304 스테인리스 "배스킨라빈스 스쿱" 판매 확인(horeca.co.kr, 제목만) `[UNCERTAIN]`.

## B. 자동화·로봇 (15)

| # | Product | Company | 하드/소프트 | Mechanism | Cycle | Price | 사람이 하는 일 | Weakness | 우리 구조와 유사도 | 출처 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | **ARIS 3.0** | XYZ(엑스와이지), 한국 | `[UNCERTAIN]` 10여 종 + 토핑. 미국 특허 US12343859B2는 **캡슐형** | 로봇팔 + 비전 AI. 3.0은 50 % 경량, 20 % 고속 | **~1분/개** | **2천만 원대(기존 ~5천만 원)** `[MULTI]` | 보충·세척(분석) | 스쿠핑 방식 미확인, 속도 | L–M | heraldcorp 3138223, zdnet 20230601163849 |
| 2 | Doosan E-series (E0509) | 두산로보틱스 | F&B 일반, 아이스크림 서빙 마케팅 | 6축 협동로봇, 5 kg, 900 mm, NSF, IP66. 아이스크림은 PoC 단계, "people-centric automation" | – | – (벤더 ROI 14개월 주장) | – | 하드 스쿱 사이클 비공개 | M | kedglobal, foodindustryexecutive |
| 3 | RB 아이스크림 로봇 | 레인보우로보틱스 | 미특정 | RB 협동로봇, **"매장 직원과 협업 운영"** | – | – | 협업(분담 미공개) | 세부 없음 | **개념상 M–H**(인간-로봇 분담) | rainbow-robotics.com |
| 4 | Soft Cream Robot | Connected Robotics(일본) | **소프트** | 소프트서브 레버 아래 로봇팔 | 30–40 s, 90개/h | – | 보충 | 스쿠핑 문제를 회피 | L | connected-robotics.com |
| 5 | Reis & Irvy's | 미국 | **소프트 froyo** | 로봇 자판기 | <60 s | – | 보충 | 소프트만 | L | vendingmarketwatch |
| 6 | Sweet Robo | – | **소프트** | 자동 소프트서브 | – | "~$4,000부터"(벤더) `[UNCERTAIN]` | 보충 | 소프트만 | L | sweetrobo.com |
| 7 | Dice Cream | – | 스쿱 제품을 **정육면체 portion**으로 | 로봇팔 + 전용 큐브 스쿠퍼 | ~40 s, 90명/h | – | – | 초기 단계 | M(전용 헤드 참고) | unlimitventures.com |
| 8 | BonBot | 스톡홀름 | 미특정 | 로봇팔 카페 | – | – | – | – | L–M | robotics247.com |
| 9 | Flexiv + Noematrix 데모 | Flexiv | **하드, 냉동고에서** | 힘제어 적응형 로봇팔이 문 열고 "적절한 힘으로" 스쿠핑 | – | – | – | 데모 | **기능상 H**: 힘제어 스쿠핑이 핵심 난제임을 보여줌 | Flexiv LinkedIn/X |
| 10 | Columbia "Scoop" | 학생 프로젝트 | **하드** | 7-DOF 팔, 전용 통, 물 헹굼 | – | – | – | 전용 통 필요 | **H**(가장 가까운 학술 사례) | jonathanblutinger.com |
| 11 | KUKA ice cream robot | KUKA | 스쿱 | 산업용 팔 데모 | – | – | – | 데모 | M | YouTube |
| 12 | Anno Robots 밀폐형 | 중국 | 소프트+토핑 추정 | 6축 팔 키오스크 | – | – | 보충 | – | L | annorobots.com |
| 13 | VLT XBot | – | 미특정 | – | – | – | – | – | ? | vendinglab.tech |
| 14 | Blendid (스무디, 비교용) | – | – | 로봇팔 키오스크 | – | 음료 $5–6 | 보충 | – | L | grocerydive |
| 15 | Dexai Alfred | Dexai Robotics | 스쿱 데모 | 로봇 부주방장, 아이스크림 스쿠핑 데모 | – | – | – | – | M | thespoon.tech |

`[NOT FOUND]`(예산 소진): BR 코리아 로봇 스쿠핑 매장, Makr Shakr, Cafe X, Moley, Chef Robotics, Carpigiani pozzetti, Coldsnap/Ninja Creami/Solo Gelato(1인분 제조형, 스쿠핑 대체 아님 — 분석), Tetra Pak Hoyer/Gram/중국 볼 성형기(공장용 — 분석).

**구조적 유사성 요약 (분석):**
1. **열만 보태는 도구**(가열 스쿱, Zeroll, dipper well): 사람이 모든 힘·회전을 제공
2. **힘 경로만 바꾸는 도구**(Midnight Scoop, 전완 부착 스쿱 US5368465): 총 힘 불변
3. **인간-로봇 협업 개념**(레인보우, 두산): 분담·사이클·툴링 비공개
4. **완전자동 하드 스쿠핑**(Columbia, Flexiv, Dice Cream, KUKA, 특허 D7): 기술적 핵심은 **힘제어 절입-sweep stroke** — 우리가 남기려는 바로 그 부분
5. 상용 "아이스크림 로봇"의 다수는 **소프트서브**로 스쿠핑 자체를 회피 → 디핑 캐비닛 매장의 직접 경쟁자 아님
6. **사람이 헤드를 놓고 기계가 절입·sweep·회전을 수행하는 상용 제품은 찾지 못했다**(41회 검색 기준, 부존재 증명 아님)

## C. 한국 시장·정량 맥락

| 항목 | 값 | 태그/출처 |
|---|---|---|
| 배스킨라빈스 가맹점 수(공정위) | 2022: 1,653 · 2023: 1,687 · 2024: 1,706 | `[SNIPPET]` news.nate.com 20260125n11257 |
| 전체 점포(직영 포함) | 2021: 1,626 · 2022: 1,720 · 2023: 1,752 | `[SNIPPET]` businesspost num=406004 |
| 제품 중량 | 싱글레귤러 115 g · 파인트 320 g(3맛) · 쿼터 620 g(4맛) · 패밀리 960 g(5맛) · 하프갤런 1,200 g(6맛) | `[MULTI]` |
| 용기 포함 목표중량 | 싱글레귤러 120 · 싱글킹 150 · 더블주니어 160 · 더블레귤러 240 · 파인트 336 · 쿼터 643 · 패밀리 989 · 하프갤런 1,237 g | `[UNCERTAIN]` namu.wiki |
| 저울 확인 관행 | 용기 아래 저울을 보며 목표 이상으로 채움, 미달 시 스티커 미출력 | `[UNCERTAIN]` 커뮤니티 |
| 스쿱 헹굼 | 매 스쿱 후 물에 헹굼, 주기적 물 교체 | `[UNCERTAIN]` namu.wiki |
| 스쿱 마스터 | 2024-11부터 우수 아르바이트 분기 포상 → **스쿠핑 숙련·품질이 관리 대상** | `[MULTI]` |
| 소비자가 | 싱글레귤러 3,900 · 파인트 9,800 · 쿼터 18,500 · 패밀리 26,000 · 하프갤런 31,500원(시점 불명) | `[UNCERTAIN]` |
| 최저임금 | 2025: 10,030원 · **2026: 10,320원**(월 2,156,880원) · 2027: 10,700원 | `[MULTI]` moel.go.kr |
| 매장당 인원·일일 스쿱 수 | – | `[NOT FOUND]` |

**중요한 관찰(분석):**
- 파인트 이상은 **공 모양이 아니라 용기에 눌러 담는** 작업이다. 이 경우 공 형상보다 **중량**이 중요하고, 눌러 담는 힘(pressing)은 우리 장치가 줄이지 못하는 잔여 부하다.
- 저울 기반 최저중량 관리가 사실이라면, 정량 가치는 CV보다 **추가 스쿱(top-up) 횟수와 과다 g**로 측정해야 한다.

## D. 가격 기준점

| 항목 | 가격 | 태그 |
|---|---|---|
| Zeroll #20 | US$36.05 | `[SNIPPET]` |
| ConserveWell dipper well | US$490–689 | `[SNIPPET]` |
| XYZ ARIS 3.0 | 2천만 원대(기존 ~5천만 원) | `[MULTI]` |
| Sweet Robo 소프트 자판기 | "$4,000부터"(벤더) | `[UNCERTAIN]` |
| "로봇 아이스크림 기계" 일반 | $12,000–60,000+ | `[SNIPPET]` 애그리게이터 |
| 디핑 캐비닛, 협동로봇 정가, 가열 스쿱 소매가 | – | `[NOT FOUND]` |

## E. "좋은 스쿱 + 적정 온도로 충분한가?" — 증거 정리

| 방향 | 증거 | 강도 |
|---|---|---|
| 충분할 수 있다 | 온도가 g당 누적 grip에 유의(Dempsey 2000) · 권장 디핑온도 존재(IDFA) · Zeroll/가열 스쿱/Midnight Scoop의 **제조사 주장** | 중(온도), 약(도구) |
| 부족할 수 있다 | 전용 스쿱·헹굼·디핑 캐비닛을 쓰는 매장에서도 부담 호소(Shin 2019 `[미검증]`) · Dempsey 현장 grip 43–52 % MGF `[미검증]` · 저울 최저중량 → top-up · 가열 스쿱은 소비자용·배터리 한계 · Zeroll 세척 제약 | 약–중 |
| **판정** | **어느 쪽도 증명할 통제 비교 연구를 찾지 못했다.** 스쿱 종류 × 온도 × 목표중량 통제 비교가 없다 → 우리 P0 실험이 이 공백을 직접 채운다 | – |
