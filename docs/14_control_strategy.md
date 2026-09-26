# 14. Control Strategy

> 원칙: 복잡한 AI 금지. **모터 전류 → 힘/토크 추정 → 규칙 기반 적응.** 안전 기능은 MCU 소프트웨어에 맡기지 않는다.
> 시뮬레이션: `calc/load_adaptive_sim.py` → `calc/output/load_adaptive_sim.md`, `fig_load_adaptive.png`

## 1. 하드웨어 구조

```
 24 V SMPS ──[E-stop NC]──[안전 릴레이]──┬── 드라이버 x (H-bridge, 전류제한, 전류센스) ── DC 모터 x + 엔코더
                                        └── 드라이버 θ (같음)                       ── 기어드모터 θ + 엔코더
 손잡이 hold-to-run 스위치 ──(하드와이어로 드라이버 ENABLE 직렬)──┘
 가드/도킹 인터록 스위치 ────(같은 ENABLE 체인)─────────────────┘

 MCU (ESP32 또는 STM32) : 전류 ADC, 엔코더 카운터, 리밋 스위치, FSM, 로그
 V1 추가: ADS1256 24-bit ADC ← 로드셀(x 인라인, θ 반력암, 통 플랫폼), 열전대(MAX31856)
```

- **하드웨어 전류제한**(드라이버 설정/분류 저항)이 힘 상한 200 N·토크 상한 10 N·m를 만든다. 소프트웨어가 멈춰도 이 한계는 유지된다.
- **hold-to-run과 인터록은 드라이버 ENABLE에 하드와이어**로 들어간다. 손을 떼면 MCU와 무관하게 정지(정지범주 0).

## 2. 상태기계 (FSM)

| 상태 | 동작 | 다음 상태 조건 |
|---|---|---|
| POWER_ON | 자체 진단(엔코더, 전류센서 offset, 리밋) | 정상 → IDLE, 이상 → FAULT |
| IDLE | 모터 비활성. **리셋 후 자동 재기동 금지** | 도킹 인터록 ON + 트리거 → TARE |
| TARE | 두 축 무부하 저속 이동으로 I₀ 측정(auto-tare) | 완료 → ARM |
| ARM | θ → θ₀(rim 접지 자세). 제품: z 잠금 해제, 스템 하강. V1: z 모터 하강 | θ 전류 또는 z 로드셀이 접촉 임계 초과 → z 잠금 → DIVE |
| DIVE | x 전진 0 → 30 mm, θ: θ₀ → 0(dive 램프 하한) | x ≥ 30 mm → DRAG |
| DRAG | x 전진, 적응 제어(§4), 부피 V 적분 | V ≥ V_target 또는 x ≥ L_max → CLOSE |
| CLOSE | x 정지(A), θ → 90–120° | θ 도달 → HOLD |
| HOLD | θ 유지, 작업자가 헤드를 들어 옮김 | 배출 버튼 → EJECT |
| EJECT | θ 과회전(스트리퍼 채택 시) 후 복귀 | 완료 → HOME |
| HOME | x 원점, θ 원점, (제품) 헹굼 도크 담금 | → IDLE |
| RETRACT | x 5 mm 후퇴, 재시도 카운트 +1 | 재시도 ≤ 2 → DRAG, 초과 → ABORT |
| ABORT | θ를 올려 rim을 빼고 x 원점, "통 템퍼링 필요/재시도" 표시 | 트리거 해제 → IDLE |
| FAULT | 드라이버 비활성, 오류 코드 | 수동 리셋 |

모든 상태에서: 트리거 해제 → 즉시 정지(하드웨어) → 소프트웨어는 IDLE로 동기화.

## 3. 부하 추정 — 전류를 힘으로

```
τ_motor = K_t · (I − I₀)
F_est   = 2π · η_s · N · η_g · τ_motor / p          (x축)
τ_est   = N · η_g · τ_motor                          (θ축)
F̂      = 1차 저역통과(τ_f = 20 ms)
```

| 오차원 | 영향 | 대책 |
|---|---|---|
| K_t 편차(온도·개체) | 이득 오차 | 모터 dyno 교정(P1-2): 로드셀 대비 F = a·I + b 회귀 |
| η_s, η_g 변동(부하·온도·그리스 점도) | 이득 오차 ±10–15 %(A14) | 교정 + **임계값을 보수적으로**(§5 규칙) |
| I₀(마찰) 변동 — 냉동고 근처 저온 | offset | 매 사이클 TARE |
| 가속 전류 | 과도 오차 | 가속 구간 판정 마스킹 |
| PWM 리플 | 노이즈 | 동기 샘플링 + LPF |

**proxy는 제어용 추정치일 뿐 안전 기능이 아니다.** 안전은 §1의 하드웨어 전류제한이 담당한다.

## 4. 적응 전략 — 시뮬레이션 비교

| 전략 | 내용 | 적용 |
|---|---|---|
| S0 | 고정 경로·고정 속도, 하드웨어 전류제한만 | 기준 |
| S1 | 고정 경로, F̂에 따라 속도 감속(90 → 141 N 구간 선형), 과부하 시 후퇴·재시도 | **B가 할 수 있는 전부** |
| S2 | F̂가 목표(110 N)를 넘으면 θ를 올려 깊이를 줄이고, 부피 V를 적분해 목표에 도달할 때까지 stroke 연장. 과부하 시 먼저 pitch-up, 더 못 올리면 후퇴 | **A 전용** |

**결과(27개 조건: 경도 4 × 속도민감도 3 × 이물 3, 모두 ASSUMPTION)**

| 전략 | 성공(부피 ≥ 95 %, 중단 없음) | peak > 150 N 발생 |
|---|---|---|
| S0 | 17 / 27 | 18 / 27 |
| S1 | 13 / 27 | 16 / 27 |
| S2 | 15 / 27 | **9 / 27** |

**해석 — 시뮬레이션이 설계에 준 결론**
1. **속도 적응(S1)은 힘을 거의 줄이지 못한다.** n이 작으면 속도를 절반으로 줄여도 힘은 수 % 준다. 전류제한에 걸린 DC 모터는 이미 스스로 느려지므로(S0) S1은 사이클만 늘리고, 보수적 정지 임계 때문에 일부 완주를 중단으로 바꾼다. → **B(1 모터)의 적응 수단은 약하다.** n을 P1에서 측정해 판정(G2).
2. **깊이 적응(S2)은 150 N 초과를 절반으로 줄인다.** 대가는 단단한 제품에서 부피 미달(78–84 %) → 두 번째 stroke 또는 "템퍼링 필요" 표시가 필요하다.
3. **"매우 단단함(250 kPa 가정)"에서는 어떤 전략도 200 N 안에서 115 g을 한 번에 못 뜬다.** → 장치는 온도관리를 대체하지 못한다. **운용 범위(예: −16 ~ −12 °C)를 제품 사양으로 명시**한다.
4. **임계값 규칙:** 과부하 판정 임계 ≤ (1 − 최대 proxy 오차) × 하드웨어 한계. 초안(185 N 임계, proxy 10 % 저독)에서는 과부하 판정이 한 번도 작동하지 않고 모두 하드웨어 stall로 끝났다. 현재 0.8 × F_HW = 161 N.
5. S2가 medium + jam 조건을 "통과"한 것은 깊게 잘라 stroke가 짧아져 장애물(x = 140 mm)에 닿기 전에 끝났기 때문이다(**시나리오 배치의 산물** — 일반화 금지).

## 5. 파라미터 초기값 (V1에서 조정)

| 파라미터 | 초기값 | 근거 |
|---|---|---|
| 하드웨어 힘 상한 F_HW | 200 N (I_lim ≈ 7.5 A) | calc §3 |
| 과부하 임계(추정치) | 0.8 · F_HW = 161 N, 50 ms 지속 | §4 규칙 4 |
| S2 힘 목표 | 0.55 · F_HW ≈ 110 N | sim |
| S2 PI 이득 | K_p = 0.03 rad/s/N, K_i = 0.05 rad/s/(N·s), θ̇ ≤ 3 rad/s | sim(튜닝 필요) |
| 공칭 이송 | 80 mm/s | 사이클 예산 |
| dive 길이 | 30 mm | 가정 |
| 후퇴 거리 / 재시도 | 5 mm / 2회 | – |
| stall 판정 | v = 0 이고 pitch 여유 없음 300 ms | sim |
| drag 시간 예산 | 8 s(시뮬), 제품 목표 ≤ 3 s | P6 |
| 접촉 판정(ARM) | θ 전류 상승 > I₀ + 0.3 A 또는 z 로드셀 > 3 N | V1에서 결정 |

## 6. 안전 기능 (ISO 13849-1 PLr은 docs/19에서 산정)

| SF | 기능 | 구현 | MCU 의존 |
|---|---|---|---|
| SF1 | 비상정지(정지범주 0) | E-stop NC → 안전 릴레이 → 드라이버 전원 | 없음 |
| SF2 | hold-to-run | 손잡이 스위치 → 드라이버 ENABLE 직렬 | 없음 |
| SF3 | 힘·토크 상한 | 드라이버 하드웨어 전류제한 | 없음 |
| SF4 | 가드/도킹 인터록 | 인터록 스위치 → ENABLE 직렬 | 없음 |
| SF5 | 과부하 후퇴 | FSM(§2) | 있음(보조) |
| SF6 | 워치독 | MCU 워치독 → ENABLE 해제 | 부분 |
| SF7 | 리밋 | 하드 리밋 스위치(ENABLE 체인) + 소프트 리밋 | 부분 |
| SF8 | 재기동 방지 | 리셋 후 IDLE 대기, 트리거 재입력 필요 | 있음 |

## 7. 제어 루프 의사코드 (A, 1 kHz)

```text
loop every 1 ms:
    I_x, I_th   = adc.read_currents()
    F_hat       = lpf(Fx_from_current(I_x - I0_x))
    tau_hat     = lpf(tau_from_current(I_th - I0_th))
    if not enable_chain_ok():            # hold-to-run, E-stop, interlock (read-back only)
        state = IDLE; drivers.off(); continue
    switch state:
      DIVE:  x_cmd += v_nom*dt ; th_cmd = max(th_cmd, entry_floor(x)) ; if x >= X_ENTRY: state = DRAG
      DRAG:  x_cmd += v_nom*dt
             e = F_hat - F_TARGET ; integ = clamp(integ + e*dt)
             dth = (F_hat > F_STOP) ? TH_RATE_MAX : clamp(Kp*e + Ki*integ)
             th_cmd = clamp(th_cmd + dth*dt, entry_floor(x), TH_MAX)
             V += area(th_cmd, h) * v * dt
             if F_hat > F_STOP and th_cmd >= TH_MAX: over_t += dt else over_t = 0
             if over_t > 50 ms: state = RETRACT
             if V >= V_TARGET or x >= L_MAX: state = CLOSE
      CLOSE: th_cmd = ramp_to(TH_HOLD) ; if at(TH_HOLD): state = HOLD
      ...
    pos_ctrl_x(x_cmd) ; pos_ctrl_th(th_cmd)
    log(t, x, th, I_x, I_th, F_hat, tau_hat, V, state)      # V1: + load cells @ 500 Hz
```

## 8. 교정 절차

| 절차 | 방법 | 결과 |
|---|---|---|
| C1 모터 dyno(P1-2) | 모터+스크류로 로드셀을 밀어 F를 5단계 × 3회, 냉동고 옆 온도에서 반복 | F = a·I + b, 잔차 σ, 온도 영향 |
| C2 θ 토크 | 반력암 로드셀로 τ vs I | τ = c·I + e |
| C3 I₀ 드리프트 | 30분 연속 운전 중 I₀ 기록 | TARE 주기 결정 |
| C4 접촉 임계 | 표면 접촉 10회 | ARM 판정값 |

## 9. 선택적 아이디어 — 예압 스프링 pitch 컴플라이언스 (가설)

B·C처럼 θ 구동이 없는 구조에서 캠과 스쿱 사이에 예압 토션 스프링을 넣어, drag 모멘트 F·z_c가 예압을 넘으면 스쿱이 기울어 깊이가 줄게 하는 **수동적 힘 제한**. 주의: θ = 0이 최대 깊이 자세라 θ = 0 부근에서는 깊이 변화가 2차로 작고, drag 모멘트는 개구부를 아래로 돌리는 방향이다. 기울어지는 방향이 깊이를 줄이는지 늘리는지는 **drag 기준 자세 θ_nom의 부호에 달려 있다**(불안정 가능). V1에서 θ 축을 토크 모드로 돌려 모사한 뒤 채택 여부를 정한다.
