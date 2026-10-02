# calc/ — 재현 가능한 공학 계산

모든 스크립트는 표준 Python 3 + numpy/matplotlib(그림만)으로 돈다.

```bash
pip install matplotlib        # numpy 포함
python3 calc/engineering_calcs.py   # 구동축·피치축·스템·핀·프레임·에너지·전류 proxy
python3 calc/portion_model.py       # 깊이-부피-힘 trade-off, 정량 오차예산
python3 calc/load_adaptive_sim.py   # S0/S1/S2 부하적응 전략 비교 시뮬레이션
python3 calc/cam_feasibility.py     # Architecture B(1모터 트랙캠) 압력각 한계

# Cartesian 기준안 (공통 가정은 cartesian_model.py 한 곳)
python3 calc/scoop_load_path.py         # 반력 경로, 구성별 스쿱 끝 변위, 스템 응력
python3 calc/gantry_motor_sizing.py     # X/Y 볼스크류 임계속도, 토크, 벨트 한계
python3 calc/z_axis_sizing.py           # Z 행정·counterbalance·브레이크, 표면 검출 방식 비교
python3 calc/tub_lane_planner.py        # 통 벽 keep-out, 레인 길이, 1 portion 깊이, 통당 portion
python3 calc/cycle_time_estimate.py     # 사이클타임 분해와 시나리오 범위
python3 calc/scoop_mechanism_compare.py # 끌기-말기 vs 클램셸: 통을 얼마나 쓰는가(5 mm 격자)
python3 calc/capstone_alternatives.py   # 학부 제작형 V1-S: 팬 vs 통, 레버·강성, 이중 push-rod, 단열 홀더, 힘 플랫폼, 펌웨어 지연
```

결과는 `calc/output/`에 markdown/PNG로 생성된다. Legacy 결과는 docs/11, 13, 14, 15에서, Cartesian 결과는 `docs/final_system_concept.md`와 각 Cartesian 문서에서 인용한다.

## 규칙

- 재료 물성(u, n, ρ, k_p)과 절삭력은 **전부 ASSUMPTION**이다(`scoop_model.py` 상단, `data/assumptions.md`).
- 측정값이 나오면 `scoop_model.py`의 상수와 각 스크립트 상단 CASES 블록만 교체하고 다시 실행한다.
- 출력 표를 손으로 고치지 않는다. 스크립트를 고치고 다시 생성한다.

| 파일 | 답하는 질문 | 교체해야 할 측정값 |
|---|---|---|
| `scoop_model.py` | 스쿱 기하(절삭단면 A, 깊이, 도심), 재료 부하 모델 | R(스쿱 실측), ρ, u(T), n |
| `engineering_calcs.py` | 모터·스크류·스템·핀·프레임 사이징 | F_x, F_z/F_x, τ_close |
| `portion_model.py` | 기계식 정량의 한계, 깊이 vs 힘 trade-off | ρ·k_p 편차, 사람 CV |
| `load_adaptive_sim.py` | 속도적응(B) vs 깊이적응(A) 중 무엇이 필요한가 | u, n, proxy 오차 |
| `cam_feasibility.py` | 1모터 트랙캠(B)이 closing을 만들 수 있는가 | τ_close, 허용 closing 거리 |
| `cartesian_model.py` | Cartesian 공통 가정(스쿱, 통, 하중 케이스, 배치, Z_SAFE, 질량) | R, 통 치수, F_x, 질량(칭량) |
| `scoop_load_path.py` | 반력이 어느 요소를 얼마나 휘게 하나, 아키텍처 1/2/3 비교 | F_x, F_y, F_z, 부품 강성 |
| `gantry_motor_sizing.py` | X/Y 구동계가 드래그와 이송을 감당하나 | F_x, 가동질량, 속도 목표 |
| `z_axis_sizing.py` | Z 모터·브레이크·counterbalance, 표면 검출 임계와 압입 오차 | Z 질량, 압입압력 p, 로드셀 노이즈 |
| `tub_lane_planner.py` | 통 안에서 레인이 얼마나 길고, 1 portion에 몇 mm가 필요한가 | 통 치수, u(T) |
| `cycle_time_estimate.py` | 사이클 시간은 어디에 쓰이나 | 축 속도·가속도, 체류시간 |
| `scoop_mechanism_compare.py` | 스쿠핑 원리별로 통을 바닥까지 쓸 수 있나 | 통 치수, 힘 예산(깊이 한계), 턱 발자국 |
| `capstone_alternatives.py` | 학부 팀이 만들 수 있게 바꾸면 무엇이 쉬워지나 | 팬 치수, 모듈 강성, 겉보기 비열, 로드셀 분해능, 호스트 지연 |
