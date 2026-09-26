# Firmware

사양: `docs/14_control_strategy.md` (FSM, 전류 proxy, 깊이 적응 + 부피 적분, 안전 기능).
제어 로직은 `calc/load_adaptive_sim.py`에서 먼저 시뮬레이션했다(같은 파라미터 이름 사용).

## 구현 원칙

- 안전 기능(E-stop, hold-to-run, 인터록, 전류 상한)은 **하드웨어**. 펌웨어는 보조.
- 과부하 임계는 추정치 기준 ≤ 0.8 × 하드웨어 한계.
- 매 사이클 TARE(I₀ 재측정), 리셋 후 자동 재기동 금지.
- 1 kHz 제어 루프, V1은 로드셀 ≥ 500 Hz 로깅(ADS1256).

## 예정 구조

```
firmware/
  v1_rig/        ESP32: 3축(z 스테퍼, x DC, θ DC) + DAQ + 로그
  common/        FSM, proxy, 적응 제어(시뮬레이션과 같은 식)
```
