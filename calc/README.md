# calc/ — 재현 가능한 공학 계산

모든 스크립트는 표준 Python 3 + numpy/matplotlib(그림만)으로 돈다.

```bash
pip install matplotlib        # numpy 포함
python3 calc/engineering_calcs.py   # 구동축·피치축·스템·핀·프레임·에너지·전류 proxy
python3 calc/portion_model.py       # 깊이-부피-힘 trade-off, 정량 오차예산
python3 calc/load_adaptive_sim.py   # S0/S1/S2 부하적응 전략 비교 시뮬레이션
python3 calc/cam_feasibility.py     # Architecture B(1모터 트랙캠) 압력각 한계
```

결과는 `calc/output/`에 markdown/PNG로 생성되며 docs/13, 14, 15, 11에서 인용한다.

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
