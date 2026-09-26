# 24. Next Actions — 이번 주부터

우선순위: **P0** = 1–2주 안에, 이것 없이는 방향을 못 정함 · **P1** = G0/G1 통과 후 · **P2** = 개선

## P0 (1–2주차)

### P0-0 원문 확보 (도서관)
- **Purpose:** 발표 근거를 snippet 수준에서 원문 수준으로 올린다(Red Team: 안 하면 Evidence FAIL).
- **Procedure:** 학교 도서관 DB(ScienceDirect, PubMed, Google Scholar, KIPRIS)에서 다운로드 → `references/pdf/`(gitignore 권장, 저작권) → `research/papers.md`의 `[미검증]`·`[UNCERTAIN]` 항목을 하나씩 대조해 태그 갱신.
  - 논문: Dempsey 2000(Appl Ergon 31), Dempsey 1998(HFES), Shin 2019(SHAW 10(4)), Inoue 2009(JDS 92(12)), Briggs 1996(JDS 79(4)), Howard & Conrad 1992
  - 특허 청구항: US12343859B2, US20240245265A1, US4758150, US6840754, KR20210015444A, KR100450044B1
  - 추가 검색: Dempsey 2000 인용 논문(Google Scholar "cited by"), KOSHA 식품서비스 근골격계, 로봇 스쿠핑 힘 측정 논문
- **Equipment:** 학교 계정
- **Expected output:** 갱신된 papers.md/patents.md, 확정 수치표
- **Decision criterion:** 43/49/52 % MGF 등이 원문과 다르면 docs/02·23 수정. US12343859B2 청구항이 "경도 기반 동작 조정"을 넓게 청구하면 P1-4 FTO 우선.

### P0-1 매장 관찰 (동의 필수)
- **Purpose:** 반복 노출량, 사이클타임, top-up, 캐비닛 온도 — 경제성과 G0의 입력.
- **Procedure:** 점주 허락 → 피크 1시간 + 비피크 1시간 관찰(직원 동의 시 손 위주 촬영, 고객 얼굴 제외). 기록: 시간당 스쿱 수(싱글/더블/포장별), 스쿱당 사이클(큐 → 컵), top-up 횟수, 헹굼 빈도, 캐비닛 표시온도·IR 표면온도, 스쿱 모델, 통 치수, 캐비닛 상판·뚜껑 구조 사진. 직원 5분 인터뷰(가장 힘든 맛·시간대·동작).
- **Equipment:** 스톱워치 앱, IR 온도계(B18), 줄자, 관찰 기록지(`experiments/templates/store_observation.csv`)
- **Expected output:** 시간당 스쿱 수 분포, 사이클 중앙값, top-up 비율, 온도
- **Decision criterion:** 캐비닛이 이미 −12 ~ −14 °C이고 직원이 "힘들지 않다"면 G0 위험 신호 → P0-2 결과와 함께 판정.

### P0-2 수동 baseline (실험실, 저비용 버전)
- **Purpose:** 최선 조건에서 남는 부하(H1)와 사람 CV·과다 g.
- **Procedure:** `experiments/protocols/P0-2_manual_baseline.md`. 참가자 4–6명 × 온도 2(−14, −18 °C) × 스쿱 2(매장형, 열전도형) × 8회. 측정: 악력계 MVC, 손잡이 FSR grip(교정), 폰 측면 영상(손목 각), 공 질량, 사이클, Borg CR-10.
- **Equipment:** 스쿱 2종(B26), FSR + 아두이노, 악력계(B33), 저울(B35), 폰 삼각대, 4 L 통(B36)
- **Expected output:** peak grip %MVC, 질량 mean/SD/CV, 목표 115 g 대비 과다 g, 사이클
- **Decision criterion (G0 일부):** −14 °C·최선 스쿱 peak grip < 20 %MVC → G0 실패 후보(E0와 함께 판정).

### P0-3 Drag force test ★ 가장 중요
- **Purpose:** 문헌이 주지 못한 **스쿱 절삭력 F(d, T)의 첫 실측** → G1, 모든 사이징의 입력.
- **Procedure:** `experiments/protocols/P0-3_drag_force_test.md`. 온도 3 × 깊이 3 × 제품 2 × 5회 = 90 stroke.
- **Equipment:** 푸시풀 게이지 500 N + 수동 슬라이드 + 스쿱 고정구(B34), 스페이서, 템퍼링 냉동고(B19), 열전대(B17)
- **Expected output:** F_peak·F_mean 표, u(T) = F/A, n 대략값(속도 2수준 추가 시)
- **Decision criterion (G1):** −14 °C, d = 20 mm 평균 F < 60 N → C(무동력) 우선 / 60–200 N → A·B / > 200 N → 온도관리 전제. 결과를 `calc/scoop_model.py`의 U_CASES에 넣고 calc 전부 재실행.

### P0-4 형상 측정 (스쿱·통·캐비닛)
- **Purpose:** 가정 A06, A10, A11 교체 → stroke·스템·도킹 설계.
- **Procedure:** 매장과 같은 스쿱·통을 구입(또는 매장에서 실측): 볼 내경·깊이·rim 두께·날끝 반경(확대 사진), 손잡이 길이·오프셋, 통 내경·깊이·테이퍼, 캐비닛 웰 개구부·상판 치수.
- **Equipment:** 버니어 캘리퍼스, 반경 게이지, 폰 매크로 사진
- **Expected output:** 치수표 → `data/assumptions.md` 갱신
- **Decision criterion:** 통 chord < 170 mm면 stroke 부족 → 깊이↑(힘↑) 또는 스쿱 R 재검토.

### P0-5 온도·밀도
- **Purpose:** 온도 통제 절차 확정, ρ(A07) 측정.
- **Procedure:** 템퍼링 냉동고 설정 −14 °C, 통 코어(5 cm)·표면 온도를 24 h 로깅해 평형시간 확인. 꺼낸 뒤 표면온도 상승률(°C/min) 측정 → 시험 창 결정. 밀도: 얼린 통에서 코어드릴/칼로 원통 절단 → 질량/부피(3회).
- **Equipment:** 냉동고 + 온도조절기, 열전대 3, IR, 저울, 자
- **Expected output:** 평형시간, 허용 시험 창(표면 +1 °C 이내), ρ
- **Decision criterion:** 표면이 3분 안에 +1 °C 넘게 오르면 rig를 냉동고 위 설치 방식으로 확정(docs/16 §7).

### P0-6 사람 스쿠핑 궤적 촬영
- **Purpose:** docs/06 가설 검증, V1 기준 궤적(x, z, θ) 확보.
- **Procedure:** 투명 측면이 없는 통이므로 스쿱 손잡이 끝에 색 마커 2개, 측면 60–240 fps 촬영, 숙련자(가능하면 매장 경력자) 10회. Tracker(OSP) 또는 Kinovea로 x, z, θ(t) 추출. 손목 roll 회전 횟수 계수.
- **Equipment:** 폰, 삼각대, 마커 스티커
- **Expected output:** 궤적 곡선, dive 길이, drag 길이, closing 각도·시간
- **Decision criterion:** closing이 x 정지 상태에서 일어나면 A의 표준 궤적 채택, 이동 중이면 rolling close(B) 가능성↑.

### P0-7 IRB·동의서
- **Purpose:** 인간 대상 측정의 윤리·일정 리스크 제거.
- **Procedure:** 학교 IRB에 심의 필요 여부 문의(최소위험 연구 면제 가능성), 동의서·정보문 초안 작성.
- **Expected output:** 심의 결과 또는 면제 확인
- **Decision criterion:** 심의가 3주 이상 걸리면 E1 일정을 뒤로 미루고 E0·E3(사람 없는 실험)를 먼저.

### P0-8 사전등록
- **Purpose:** 사후 기준 조정 방지.
- **Procedure:** docs/01 §6 가설·판정선과 docs/17을 확정 → 커밋 → `git tag prereg-v1`.
- **Decision criterion:** 태그 이후 판정선을 바꾸려면 변경 이유를 문서화하고 별도 태그.

## P1 (3–6주차, G0/G1 통과 후)

| ID | Task | Purpose | Procedure | Equipment | Expected output | Decision criterion |
|---|---|---|---|---|---|---|
| P1-1 | V1 3축 rig 제작 | E1–E9 수행 | docs/16 일정, 안전 체인 먼저 | BOM B01–B33 | 동작 rig, 점검표 | 드라이런 20회 무고장 |
| P1-2 | 모터 dyno 교정 | 전류 proxy 오차(A14) | 모터+스크류로 로드셀을 5단계 × 3회, 상온/냉동고 옆 | B06, B09, B16 | F = a·I + b, 잔차 σ | 오차 ≤ ±15 % 아니면 로드셀 유지(제품 비용↑) |
| P1-3 | 통 포스 플랫폼 제작·교정 | M1/M2/M3 공통 기준 | 추 0.5–10 kg 3축 교정, 온도 드리프트 | B11, B12, B14 | 교정 행렬 | 교차감도 < 5 % |
| P1-4 | FTO 확인 | US12343859B2 등 | 산학협력단/변리사 상담 | P0-0 청구항 | 위험 메모 | 충돌 시 설계 회피안 |
| P1-5 | 워크플로 목업 | 도킹·이동·헹굼 시간(H4 사전 점검) | 합판·3D 프린트 헤드(3 kg 추) + 툴 밸런서 + 웰 2개 목업, 10회 × 3명 | 목업 재료 | 사이클 분해 시간 | 도킹·이동만으로 수동 사이클 초과 시 제품 비전 재설계 |
| P1-6 | 계측 수동 스쿱 | M3 손잡이 모멘트·grip | 스트레인게이지 3브리지 + FSR, 추로 교정 | B31 | 교정된 계측 스쿱 | 교정 오차 ≤ 5 % |

## P2 (7주차 이후)

| ID | Task | Purpose |
|---|---|---|
| P2-1 | ejector 시험(과회전 + 스트리퍼 vs 헹굼만) | 모듈 F 존폐 |
| P2-2 | θ 디더링, 온수 예열(E8, E9) | u 저감 수단 |
| P2-3 | 세척 시간·잔여물(E7) | H3 검증 |
| P2-4 | 내구 운전 1,000 사이클 | 핀·스템·proxy 드리프트 |
| P2-5 | 가맹본부/점주 인터뷰 | 구매 주체·가치(Q18) |
| P2-6 | 마찰·부착 경사판 시험 | μ, 표면 상태 효과 |
