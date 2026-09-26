# Ice Cream Scooping Capstone

**하드아이스크림 스쿠핑 작업부하의 정량화와 반력접지형 2자유도 동력 스쿠핑 모듈의 설계·검증**

> 요약: [`docs/00_executive_summary.md`](docs/00_executive_summary.md) · 교수 제출용: [`docs/23_final_capstone_proposal.md`](docs/23_final_capstone_proposal.md) · 이번 주 할 일: [`docs/24_next_actions.md`](docs/24_next_actions.md)

## 연구질문

> 적정 디핑온도와 최선의 수공구를 쓰는 조건에서도 남는 스쿠핑 부하 가운데, 반력(grip·손목 토크)과 손목 회전을 **프레임에 접지된 저자유도(≤ 2 actuated DOF) 헤드**로 옮기면, 수동 대비 작업자 손 힘과 손목 동작을 얼마나 줄이면서 사이클타임과 portion consistency를 유지할 수 있는가?

## 설계 철학 (재검증 후)

- **판단은 사람, 반력과 반복 궤적은 프레임에 접지된 저자유도 기구**
- 손에 드는 동력 공구는 반토크가 손으로 돌아온다 → 반력을 **웰 도킹 프레임**으로 보낸다
- 동력화는 전제가 아니다. **결정 게이트**로 고른다:
  - G0: 최선 조건에서도 부하가 작으면 기계 개발 중단(측정연구로 전환)
  - G1: 절삭력 < 60 N이면 **무동력 C**, 60–200 N이면 **동력 A/B**
  - G2: 속도민감도·rolling close 결과로 **A(2 모터) vs B(1 모터 캠)**
- 가열·비전·AI·헤드 무게센서는 **삭제**, ejector는 시험 후 결정

## 동결된 주 설계안 (A)

1. 작업자가 헤드를 원하는 웰에 도킹(핀 2 + 래치) → 프레임이 웰 개구부를 덮어 가드 역할
2. rim을 표면에 닿게 해 기준을 잡고 z 잠금 (d = R·cos θ − h)
3. hold-to-run 트리거 → **dive → drag(x) → closing(θ)**, 구동 2 DOF
4. 모터 전류로 부하 추정 → 힘 상한 안에서 깊이 적응 + 부피 적분으로 목표량 채움, 과부하 시 후퇴
5. 헤드를 들어 컵 위에서 배출, 맛 교체 시 헹굼 도크

## 현재 상태

**문제 재검증 · 선행조사(특허 45건, 제품·로봇 25종) · 개념 21개 → 3개 · 계산 · 제어 시뮬레이션 · 실험계획 · FMEA 완료. 실측 데이터 0건.**
가장 먼저 할 일은 스쿱 절삭력의 첫 실측([`experiments/protocols/P0-3_drag_force_test.md`](experiments/protocols/P0-3_drag_force_test.md))이다.

⚠️ 이번 조사는 원문 열람이 차단된 환경에서 검색 요약만으로 이루어졌다. 근거마다 태그(`[MULTI]`, `[SNIPPET]`, `[UNCERTAIN]`, `[미검증]`)가 있고, 원문 대조가 P0-0이다.

## 폴더

| 폴더 | 내용 |
|---|---|
| `docs/` | 00–24 설계 문서, `archive/`에 v0 가설 문서 보존 |
| `research/` | 근거 원장: papers, patents, products, standards, open_questions |
| `data/` | `assumptions.md` — 모든 가정값 대장(ID, 민감도, 측정 방법) |
| `calc/` | 재현 가능한 계산·시뮬레이션(`python3 calc/*.py`), 출력은 `calc/output/` |
| `BOM/` | `prototype_v1.csv` — V1 rig 37개 항목, 추정가 |
| `experiments/` | 프로토콜(P0-2, P0-3), 기록 템플릿 |
| `cad/`, `firmware/` | 제작 단계 산출물 자리(사양은 docs/11–14) |
| `prompts/`, `references/` | 이전 작업 프롬프트, 참고문헌 색인 |

## 핵심 KPI

peak / mean hand force [N] · force–time integral [N·s] · grip [%MVC] · wrist excursion [°] · 부하 상태 손목 회전 수 · 재료가 받은 힘 [N] · scoop torque [N·m] · cycle time [s] · mass, CV [%] · success rate [%] · motor current [A] · energy/cycle [J] · 세척 시간 [s]
