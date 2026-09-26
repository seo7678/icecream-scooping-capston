# Ice Cream Scooping Capstone

**하드아이스크림 스쿠핑의 자동화: Cartesian 자동 맛 선택 스쿠퍼 (현재 기준안)**

> **한 문서로 보기(문제 → 기존 해결책·특허와 한계 → 아이디어 → 구조와 메커니즘):** [`docs/concept_brief.md`](docs/concept_brief.md)
>
> **구조도 · 메커니즘 설명 · 작동 동영상:** [`docs/mechanism_explained.md`](docs/mechanism_explained.md) (그림·영상 원본은 `media/`)
>
> **현재 기준안 요약:** [`docs/final_system_concept.md`](docs/final_system_concept.md) — 권장 구조, 다시 판단한 결정과 근거, 위험 Top 5, 첫 시제품(V1a), 결정 게이트, 15개 항목 요약
>
> 구조: [`system_architecture_cartesian.md`](docs/system_architecture_cartesian.md) · 비교: [`architecture_comparison.md`](docs/architecture_comparison.md) · 선행기술 재검색: [`updated_prior_art.md`](docs/updated_prior_art.md)

## 현재 기준안 (2026-09)

![작동 동영상](media/operation.gif)

![전체 구조](media/fig1_overall_axonometric.png)

작업자가 맛을 고르면 **직교 3축(X_M, Y_M, Z_M)** 이 헤드를 그 통으로 옮기고, 헤드는 **스쿱 피치 θ 하나**만 따로 움직인다. 이 기계를 "5축 로봇"이라 부르지 않는다. 구성은 전역 위치결정 3축과 로컬 스쿠핑 1축, **모터 4개**다.

```
맛 선택 → 좌표 조회 → (Z ≥ Z_SAFE) XY 이동 → Z 접근 → 로드셀 표면 검출(3 N)
→ dive → X 드래그(힘 기반 깊이 적응) → θ 닫기 → Z 상승 → XY → 컵 스테이션
→ 배출(빗 걸기) → 저울 계량 → 맛별 보정 → 후퇴
```

| 결정 | 내용 | 근거 |
|---|---|---|
| 스쿠핑 원리 | 끌기-말기(클램셸·압입·회전·코어와 비교) | `calc/output/scoop_mechanism_compare.md` |
| 드래그 축 | 전역 X 볼스크류, 드래그 방향 = 빔 축. 로컬 x축 없음 | `calc/output/scoop_load_path.md` |
| 구조 | 강관 기둥·빔 + HG15 + Ø30 스템 → 200 N에서 스쿱 끝 ~1.0 mm | `docs/gantry_load_path.md` |
| 표면 | Z 볼너트 로드셀 터치오프 + 절삭 이력 지도, 비전 없음 | `calc/output/z_axis_sizing.md` |
| 사이클 | 기계 14.5 s(11.1–22.7, 추정) + 작업자 ~3 s | `calc/output/cycle_time_estimate.md` |
| 비용 | V1 BOM ~524만 원(추정) | `BOM/cartesian_prototype_v1.csv` |
| 안전 | 인클로저로 분리(모든 축 > 140 N), 도어·컵 베이 → STO, Z_SAFE 규칙 | `docs/collision_and_safety.md` |
| 위생 | 갠트리와 통 사이 연속 드립 차단층, 식품 모듈 공구 없이 분리 | `docs/sanitation_architecture.md` |

**판단:** 수동과 로봇 팔 사이의 중간 자동화로는 **조건부로 실용적**이다(병렬 업무, 고정 맛 세트, 저울 정량). **손목 부하만이 목적이면 과도하다.** 그 목적에는 아래 Legacy M이 더 싸고 충분하다.

**다음 할 일:** 기계 없이 절삭력부터 잰다(E0 확장, [`P0-3` §7](experiments/protocols/P0-3_drag_force_test.md)). 그다음 한 통짜리 X-Z-θ 장치(V1a)를 만들고, 여러 통 자동 사이클(V1b)로 넓힌다. **실측 데이터 0건. 모든 하중은 가정이다.**

---

## Legacy Architecture M — 반력 접지 헤드 + 사람 위치지정 (이전 기준안)

> 아래는 이전 기준안이다. 헤드 기구·식품 모듈·부하 적응 제어·실험계획은 새 구조에서도 그대로 쓰인다. 무엇이 유효하고 무엇이 바뀌었는지는 각 문서 첫머리의 배너에 있다.
> 요약: [`docs/00_executive_summary.md`](docs/00_executive_summary.md) · 제안서: [`docs/23_final_capstone_proposal.md`](docs/23_final_capstone_proposal.md) · 한 문서 요약의 이전 판: `docs/concept_brief.md` 커밋 `32daeaa`

### 연구질문 (Legacy)

> 적정 디핑온도와 최선의 수공구를 쓰는 조건에서도 남는 스쿠핑 부하 가운데, 반력(grip·손목 토크)과 손목 회전을 **프레임에 접지된 저자유도(≤ 2 actuated DOF) 헤드**로 옮기면, 수동 대비 작업자 손 힘과 손목 동작을 얼마나 줄이면서 사이클타임과 portion consistency를 유지할 수 있는가?

### 설계 철학 (재검증 후)

- **판단은 사람, 반력과 반복 궤적은 프레임에 접지된 저자유도 기구**
- 손에 드는 동력 공구는 반토크가 손으로 돌아온다 → 반력을 **웰 도킹 프레임**으로 보낸다
- 동력화는 전제가 아니다. **결정 게이트**로 고른다:
  - G0: 최선 조건에서도 부하가 작으면 기계 개발 중단(측정연구로 전환)
  - G1: 절삭력 < 60 N이면 **무동력 C**, 60–200 N이면 **동력 A/B**
  - G2: 속도민감도·rolling close 결과로 **A(2 모터) vs B(1 모터 캠)**
- 가열·비전·AI·헤드 무게센서는 **삭제**, ejector는 시험 후 결정

### 동결된 주 설계안 (A)

1. 작업자가 헤드를 원하는 웰에 도킹(핀 2 + 래치) → 프레임이 웰 개구부를 덮어 가드 역할
2. rim을 표면에 닿게 해 기준을 잡고 z 잠금 (d = R·cos θ − h)
3. hold-to-run 트리거 → **dive → drag(x) → closing(θ)**, 구동 2 DOF
4. 모터 전류로 부하 추정 → 힘 상한 안에서 깊이 적응 + 부피 적분으로 목표량 채움, 과부하 시 후퇴
5. 헤드를 들어 컵 위에서 배출, 맛 교체 시 헹굼 도크

### 상태

**문제 재검증 · 선행조사(특허 45건, 제품·로봇 25종) · 개념 21개 → 3개 · 계산 · 제어 시뮬레이션 · 실험계획 · FMEA 완료. 실측 데이터 0건.**
가장 먼저 할 일은 스쿱 절삭력의 첫 실측([`experiments/protocols/P0-3_drag_force_test.md`](experiments/protocols/P0-3_drag_force_test.md))이다.

⚠️ 이번 조사는 원문 열람이 차단된 환경에서 검색 요약만으로 이루어졌다. 근거마다 태그(`[MULTI]`, `[SNIPPET]`, `[UNCERTAIN]`, `[미검증]`)가 있고, 원문 대조가 P0-0이다.

## 폴더

| 폴더 | 내용 |
|---|---|
| `docs/` | Cartesian 기준안 문서 14개(`final_system_concept.md`부터), Legacy 00–24, `archive/`에 v0 가설 문서 |
| `research/` | 근거 원장: papers, patents(§5 Cartesian 재검색), products, standards, components(카탈로그 값), open_questions |
| `data/` | `assumptions.md` — 모든 가정값 대장(ID, 민감도, 측정 방법) |
| `calc/` | 재현 가능한 계산·시뮬레이션(`python3 calc/*.py`), 출력은 `calc/output/` |
| `BOM/` | `cartesian_prototype_v1.csv` — Cartesian V1 44개 항목, 추정가 / `prototype_v1.csv` — Legacy V1 rig |
| `experiments/` | 프로토콜(P0-2, P0-3), 기록 템플릿 |
| `cad/` | `cartesian_layout.md`, `scooping_head_layout.md` — 배치·높이·치수 |
| `media/` | 구조도(축측·3면도), 메커니즘 그림, 작동 동영상(MP4·GIF)과 생성 스크립트 |
| `firmware/` | `state_machine.md`(전이표·가드·Z_SAFE 게이트), `coordinate_map_example.md`(맛 좌표 JSON·레인 선택) |
| `prompts/`, `references/` | 이전 작업 프롬프트, 참고문헌 색인 |

## 핵심 KPI

Cartesian: 기계 사이클 [s] · 작업자 점유 시간/스쿱 [s] · 드래그 힘 F_x, F_z [N] · 표면 검출 반복성 [mm] · portion 질량·허용범위 내 비율 [%] · 배출 성공률 [%] · 통당 기계 portion 수 · 가드 거부 횟수(로그)

Legacy: 
peak / mean hand force [N] · force–time integral [N·s] · grip [%MVC] · wrist excursion [°] · 부하 상태 손목 회전 수 · 재료가 받은 힘 [N] · scoop torque [N·m] · cycle time [s] · mass, CV [%] · success rate [%] · motor current [A] · energy/cycle [J] · 세척 시간 [s]
