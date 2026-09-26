# Load-adaptive drag simulation — generated output

> 생성: `python3 calc/load_adaptive_sim.py`. 재료(u, n)·구동계·proxy 오차는 모두 **ASSUMPTION**.
> 이 표는 절대값이 아니라 **전략 간 순위가 어떤 조건에서 뒤집히는지**를 보기 위한 것이다.

- 고정경로(S0/S1): 깊이 22 mm, 진입 30 mm dive, stroke 182 mm (medium 제품 기준 설계)
- S2: 최대깊이 28 mm, 힘 목표 110 N(추정치)로 pitch를 조절, 목표부피 177 cm³ 도달 또는 chord 190 mm에서 종료
- 하드웨어 힘 상한 201 N(전류제한), proxy gain 0.9, 과부하 판정 161 N(추정치), 시간예산 8 s

셀 = peak force / drag 시간 / 목표부피 달성률, ✔ = 부피 ≥ 95 % 이고 중단 없음.

| u case | n | inclusion | S0 fixed | S1 speed-adapt | S2 depth-adapt |
|---|---|---|---|---|---|
| soft (40 kPa) | 0.05 | none | 41 N / 2.3 s / 100 % ✔ | 41 N / 2.3 s / 100 % ✔ | 58 N / 1.7 s / 100 % ✔ |
| soft (40 kPa) | 0.05 | chunk | 104 N / 2.3 s / 100 % ✔ | 104 N / 2.3 s / 100 % ✔ | 144 N / 1.7 s / 100 % ✔ |
| soft (40 kPa) | 0.15 | none | 41 N / 2.3 s / 100 % ✔ | 41 N / 2.3 s / 100 % ✔ | 58 N / 1.7 s / 100 % ✔ |
| soft (40 kPa) | 0.15 | chunk | 104 N / 2.3 s / 100 % ✔ | 104 N / 2.3 s / 100 % ✔ | 144 N / 1.7 s / 100 % ✔ |
| soft (40 kPa) | 0.3 | none | 41 N / 2.3 s / 100 % ✔ | 41 N / 2.3 s / 100 % ✔ | 58 N / 1.7 s / 100 % ✔ |
| soft (40 kPa) | 0.3 | chunk | 104 N / 2.3 s / 100 % ✔ | 104 N / 2.3 s / 100 % ✔ | 144 N / 1.7 s / 100 % ✔ |
| medium (90 kPa) | 0.05 | none | 93 N / 2.3 s / 100 % ✔ | 93 N / 2.3 s / 100 % ✔ | 129 N / 1.7 s / 100 % ✔ |
| medium (90 kPa) | 0.05 | chunk | 201 N / 5.8 s / 100 % ✔ | 201 N / 2.0 s / 64 % ✘ overload,JAM-abort | 201 N / 2.2 s / 100 % ✔ |
| medium (90 kPa) | 0.05 | jam | 201 N / 2.1 s / 75 % ✘ STALL | 201 N / 2.4 s / 81 % ✘ overload,JAM-abort | 129 N / 1.7 s / 100 % ✔ |
| medium (90 kPa) | 0.15 | none | 93 N / 2.3 s / 100 % ✔ | 93 N / 2.3 s / 100 % ✔ | 129 N / 1.7 s / 100 % ✔ |
| medium (90 kPa) | 0.15 | chunk | 201 N / 2.6 s / 100 % ✔ | 201 N / 3.6 s / 100 % ✔ | 201 N / 2.1 s / 100 % ✔ |
| medium (90 kPa) | 0.15 | jam | 201 N / 2.1 s / 75 % ✘ STALL | 201 N / 2.4 s / 81 % ✘ overload,JAM-abort | 129 N / 1.7 s / 100 % ✔ |
| medium (90 kPa) | 0.3 | none | 93 N / 2.3 s / 100 % ✔ | 93 N / 2.3 s / 100 % ✔ | 129 N / 1.7 s / 100 % ✔ |
| medium (90 kPa) | 0.3 | chunk | 201 N / 2.4 s / 100 % ✔ | 201 N / 2.9 s / 100 % ✔ | 201 N / 2.1 s / 100 % ✔ |
| medium (90 kPa) | 0.3 | jam | 201 N / 2.1 s / 75 % ✘ STALL | 201 N / 2.4 s / 81 % ✘ overload,JAM-abort | 129 N / 1.7 s / 100 % ✔ |
| hard (160 kPa) | 0.05 | none | 166 N / 2.3 s / 100 % ✔ | 152 N / 8.0 s / 74 % ✘ TIMEOUT | 141 N / 2.4 s / 83 % ✘ SHORT |
| hard (160 kPa) | 0.05 | chunk | 201 N / 1.7 s / 58 % ✘ STALL | 201 N / 7.0 s / 64 % ✘ overload,JAM-abort | 201 N / 2.4 s / 78 % ✘ SHORT |
| hard (160 kPa) | 0.15 | none | 166 N / 2.3 s / 100 % ✔ | 142 N / 6.0 s / 100 % ✔ | 141 N / 2.4 s / 83 % ✘ SHORT |
| hard (160 kPa) | 0.15 | chunk | 201 N / 1.7 s / 58 % ✘ STALL | 201 N / 4.1 s / 64 % ✘ overload,JAM-abort | 201 N / 2.4 s / 78 % ✘ SHORT |
| hard (160 kPa) | 0.3 | none | 166 N / 2.3 s / 100 % ✔ | 133 N / 4.4 s / 100 % ✔ | 141 N / 2.4 s / 83 % ✘ SHORT |
| hard (160 kPa) | 0.3 | chunk | 201 N / 4.2 s / 100 % ✔ | 201 N / 3.2 s / 64 % ✘ overload,JAM-abort | 201 N / 2.4 s / 78 % ✘ SHORT |
| very_hard (250 kPa) | 0.05 | none | 201 N / 1.5 s / 7 % ✘ STALL | 184 N / 1.4 s / 8 % ✘ overload,JAM-abort | 137 N / 2.4 s / 53 % ✘ SHORT |
| very_hard (250 kPa) | 0.05 | chunk | 201 N / 1.5 s / 7 % ✘ STALL | 184 N / 1.4 s / 8 % ✘ overload,JAM-abort | 201 N / 2.4 s / 50 % ✘ SHORT |
| very_hard (250 kPa) | 0.15 | none | 201 N / 8.0 s / 73 % ✘ TIMEOUT | 201 N / 2.2 s / 13 % ✘ overload,JAM-abort | 137 N / 2.4 s / 53 % ✘ SHORT |
| very_hard (250 kPa) | 0.15 | chunk | 201 N / 6.5 s / 58 % ✘ STALL | 201 N / 2.2 s / 13 % ✘ overload,JAM-abort | 201 N / 2.4 s / 50 % ✘ SHORT |
| very_hard (250 kPa) | 0.3 | none | 201 N / 4.9 s / 100 % ✔ | 153 N / 8.0 s / 67 % ✘ TIMEOUT | 137 N / 2.4 s / 53 % ✘ SHORT |
| very_hard (250 kPa) | 0.3 | chunk | 201 N / 3.1 s / 58 % ✘ STALL | 201 N / 7.9 s / 64 % ✘ overload,JAM-abort | 201 N / 2.4 s / 50 % ✘ SHORT |

## 요약

| Strategy | 성공 / 전체 | peak force > 150 N 발생 |
|---|---|---|
| S0 | 17 / 27 | 18 / 27 |
| S1 | 13 / 27 | 16 / 27 |
| S2 | 15 / 27 | 9 / 27 |

