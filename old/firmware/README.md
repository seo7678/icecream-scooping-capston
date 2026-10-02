# Firmware

**현재 아키텍처(Cartesian 자동 맛 선택):**
- `state_machine.md` — 전이표, 타이머, 전역 가드, Z_SAFE 게이트, 드래그 루프, 로그 형식
- `coordinate_map_example.md` — 맛 좌표표(JSON), 조회, 레인 선택, keep-out, 계획 단계 Z_SAFE 검사
- 상위 사양: `docs/control_state_machine.md`, `docs/coordinate_and_flavor_mapping.md`

Legacy(수동 위치결정, 모드 M1) 사양: `docs/14_control_strategy.md`. 깊이 적응 로직은 `calc/load_adaptive_sim.py`에서 먼저 시뮬레이션했고, Cartesian 드래그 루프도 같은 파라미터 이름을 쓴다.

## 구현 원칙

- 안전 기능(E-stop, hold-to-run, 인터록, 전류 상한)은 **하드웨어**. 펌웨어는 보조.
- 과부하 임계는 추정치 기준 ≤ 0.8 × 하드웨어 한계.
- 매 사이클 TARE(I₀ 재측정), 리셋 후 자동 재기동 금지.
- 1 kHz 제어 루프, V1은 로드셀 ≥ 500 Hz 로깅(ADS1256).
- **XY 이동 명령은 `request_xy_move()` 한 곳으로만** 나간다(Z < Z_SAFE면 거부, 예외 두 가지만).

## 예정 구조

```
firmware/
  v1_rig/        (Legacy) ESP32: 3축(z 스테퍼, x DC, θ DC) + DAQ + 로그
  cartesian_v1/  Teensy 4.1급: X·Y·Z 폐루프 스테퍼 + θ 기어드모터 + 로드셀 2 + 저울 + 베이 빔
  common/        FSM, proxy, 적응 제어(시뮬레이션과 같은 식)
```
