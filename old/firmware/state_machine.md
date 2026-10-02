# Firmware State Machine (구현 수준)

> 상태의 의미·안전 조건은 `docs/control_state_machine.md`, 좌표·조회표는 `firmware/coordinate_map_example.md`.
> 대상: V1 Cartesian 시연기(Teensy 4.1급 MCU, 1 kHz 루프). 숫자는 모두 **ASSUMPTION**이며 V1 시험 후 `config.h` 한 곳에서 바꾼다.
> 아직 코드는 없다. 이 문서는 코드를 쓰기 전에 고정할 **전이표·타이머·가드**다.

## 1. 계층

```
┌──────────────────────────────────────────────────────────────┐
│ 하드웨어 안전 체인 (펌웨어와 무관하게 동작)                    │
│ E-stop ─┬─ 도어 인터록 ─┬─ 컵 베이 2빔(전 축) ─▶ 안전 릴레이 ─▶ 드라이버 STO │
│         └─ Z 스프링 브레이크(무전원 = 잠김)                    │
├──────────────────────────────────────────────────────────────┤
│ 1 kHz 실시간 루프   : 궤적 보간, 스텝 발생, 로드셀 필터, 전역 가드 │
│ 100 Hz 상태기계     : 아래 전이표                              │
│ 10 Hz  HMI·로그     : 버튼/터치, SD 로그, 저울 표시             │
└──────────────────────────────────────────────────────────────┘
```

펌웨어의 가드는 **하드웨어 체인을 대신하지 않는다.** 펌웨어가 멈춰도 E-stop, 도어, Z 브레이크는 동작해야 한다.

## 2. 파라미터 (config.h 초안)

| 이름 | 값 | 단위 | 출처 |
|---|---|---|---|
| `Z_SAFE` | 60 (컵 매립) / 140 (컵 올림) | mm | cartesian_model.z_safe |
| `Z_MIN` | −205 | mm | 통 바닥 + 10 + R |
| `Z_TOP` | 160 | mm | Z 행정 상단 |
| `V_XY_TRAVEL` | 250 | mm/s | gantry_motor_sizing |
| `A_XY` | 1000 | mm/s² | 〃 |
| `V_Z_FAST` / `V_Z_PROBE` | 150 / 10 | mm/s | z_axis_sizing |
| `Z_est` | 표면 추정 + R·cos 30° (= +30.3) | mm | 표면에 rim 최저점이 닿을 때의 **C 높이**(Z_M은 항상 C 높이) |
| `PROBE_MARGIN` | 10 | mm | Z_est 위에서 저속 전환 |
| `PROBE_OVERTRAVEL` | 25 | mm | Z_est 아래로 더 내려가도 무접촉 → NO_TUB |
| `F_TOUCH` / `T_TOUCH` | 3 / 20 | N / ms | 로드셀 임계 + 지속시간 |
| `F_LIMIT` | 200 | N | 구조 설계 하중(S30) |
| `F_TARGET` | 0.55 × F_LIMIT = 110 | N | 깊이 적응 목표 |
| `F_STOP` / `T_STOP` | 0.80 × F_LIMIT = 160 / 50 | N / ms | 과부하 |
| `F_TRAVEL` | 30 | N | 이송 중 예상 밖 힘 = 충돌 의심 |
| `THETA_ATTACK` / `THETA_CAPTURE` | −30 / +90 | ° | scooping_head_mechanism §1 |
| `THETA_MIN` / `THETA_MAX` | −32 / +92 | ° | 평행링크 ±60° + 2° 여유 |
| `V_DRAG` / `T_CLOSE` | 80 / 800 | mm/s / ms | cycle_time_estimate |
| `DIVE_ANGLE` | 30 | ° | dive 경로 경사 |
| `D_MAX` | 28 | mm | 힘 예산(맛별 덮어쓰기 가능) |
| `CUP_PRESENT_G` | 3 | g | 저울 |
| `T_CUP_WAIT` / `T_BAY_WAIT` | 10 / 10 | s | |
| `T_SETTLE_SCALE` | 1.0 | s | |
| `T_IDLE_HOME` | 60 | s | |
| `POS_ERR_MAX` | 0.2 | mm | 폐루프 드라이버 편차 |
| `EMA_LAMBDA` | 0.2 | – | 맛별 portion 보정 |

## 3. 전역 가드 (1 kHz, 모든 상태)

```c
// 매 주기 가장 먼저 실행. 하나라도 걸리면 상태기계보다 우선한다.
void global_guards(void) {
    if (!hw_chain_ok())               { enter(E_STOP_OR_INTERLOCK); return; } // 드라이버는 이미 STO
    if (!homed && state_needs_home()) { enter(POSITION_ERROR); return; }
    if (max_following_error() > POS_ERR_MAX) { stop_all(); enter(POSITION_ERROR); return; }
    if (in_travel_move() && (fabs(Fx) > F_TRAVEL || fabs(Fz) > F_TRAVEL)) {
        stop_all(); enter(OVERLOAD); return;                 // 이송 중 충돌 의심
    }
    if (bay_beam_blocked()) {                                // 하드웨어가 이미 전 축 STO
        enter(HAND_IN_BAY); return;                          // 빔 해제 + 계속 버튼 → Z 상승 후 재개
    }
}
```

> **선행기술 주의:** 이송 중 30 N / 스쿠핑 중 160 N처럼 상태에 따라 힘 한계를 바꾸는 구조는 Dexai US11597084B2(등록)의 "상황별 토크 한계"와 겹칠 수 있다. 청구항을 확인하기 전까지는 시제품·연구용으로만 쓴다. 회피안(위치 기준 전환, 추종 오차 기반 충돌 감지)은 `docs/updated_prior_art.md` §5 R1.

### 3.1 Z_SAFE 가드 — XY 명령은 이 함수로만 나간다

```c
// XY 이동 요청은 모두 이 게이트를 통과한다. 상태기계 코드가 motion_xy()를 직접 부르지 않는다.
MoveResult request_xy_move(float x, float y, MoveKind kind) {
    if (Z >= Z_SAFE) return motion_xy(x, y, V_XY_TRAVEL);           // 일반 이송

    // Z < Z_SAFE 에서 허용되는 XY 이동은 두 가지뿐
    if (kind == MOVE_DRAG && state == SCOOP_DRAG
        && inside_tub_workspace(active_tub, x, y, Z))               // 현재 통의 safe workspace 안
        return motion_xy(x, y, V_DRAG);
    if (kind == MOVE_EJECT && state == EJECT
        && inside_cup_bay(x, y) && !bay_beam_blocked())             // 컵 베이 안 빗 걸기 후퇴
        return motion_xy(x, y, V_EJECT);

    log_event(EV_XY_BLOCKED_LOW_Z, x, y, Z);
    return MOVE_REJECTED;                                            // → POSITION_ERROR
}
```

`inside_tub_workspace()`는 C가 반경 `r_c = R_tub(표면 깊이 + d) − R − 10` 원 안에 있는지 본다(`coordinate_map_example.md` §4).

## 4. 전이표

| # | 현재 상태 | 이벤트 / 조건 | 가드 | 동작 | 다음 상태 |
|---|---|---|---|---|---|
| 1 | BOOT | 자체진단 OK | 좌표표 CRC OK, 로드셀 응답, 드라이버 enable | – | HOMING |
| 2 | BOOT | 진단 실패 | – | 화면 코드 | FAULT |
| 3 | HOMING | 시작 | 인터록 OK | **Z 위로 먼저**(25 mm/s) → X → Y, 스위치 후 2 mm 백오프·5 mm/s 재접근 | – |
| 4 | HOMING | 3축 원점 OK | 편차 0 | `homed = true`, HOME(0,0,Z_TOP) | IDLE |
| 5 | HOMING | 스위치 미검출(행정 + 10 mm) | – | 정지 | POSITION_ERROR |
| 6 | IDLE | 맛 버튼 | homed | `order = {flavor, size}` | LOOKUP |
| 7 | IDLE | 유휴 > T_IDLE_HOME | – | – | RETURN_HOME |
| 8 | LOOKUP | 조회 OK | status ∈ {OK, LOW}, map_valid | 레인·Z_est·파라미터 계산 | CHECK_CUP |
| 9 | LOOKUP | status ∈ {EMPTY, DISABLED} | – | "통 교체/점검" 안내 | IDLE |
| 10 | LOOKUP | map_valid = false | – | 3점 재프로브 예약 | REPROBE |
| 11 | CHECK_CUP | 저울 > CUP_PRESENT_G, 안정 | **베이 빔 0.5 s 연속 비어 있음** | 저울 tare(컵 포함) | XY_TO_TUB |
| 12 | CHECK_CUP | T_CUP_WAIT 초과 | – | "컵을 놓으세요" | NO_CUP |
| 13 | XY_TO_TUB | 진입 | **Z ≥ Z_SAFE** (아니면 Z 먼저 상승) | θ → THETA_ATTACK, `request_xy_move(lane_start, TRAVEL)` | – |
| 14 | XY_TO_TUB | 도착, 편차 < POS_ERR_MAX | – | – | Z_APPROACH |
| 15 | Z_APPROACH | 진입 | – | Z → Z_est + PROBE_MARGIN @ V_Z_FAST | – |
| 16 | Z_APPROACH | 목표 도달 | – | 로드셀 tare | SURFACE_DETECT |
| 17 | Z_APPROACH | 하강 중 ΔF_z ≥ F_TOUCH | – | 즉시 정지(예상보다 높은 표면) | SURFACE_DETECT |
| 18 | SURFACE_DETECT | ΔF_z ≥ F_TOUCH, T_TOUCH 지속 | – | `Z0 = Z`, 지도 셀 보정 | SCOOP_DIVE |
| 19 | SURFACE_DETECT | Z < Z_est − PROBE_OVERTRAVEL, 무접촉 | – | Z 상승 | NO_TUB |
| 20 | SCOOP_DIVE | 진입 | – | X·Z 보간(경사 DIVE_ANGLE), 깊이 d까지 | – |
| 21 | SCOOP_DIVE | 깊이 d 도달 | – | 부피 적분 시작 | SCOOP_DRAG |
| 22 | SCOOP_DIVE | F > F_STOP, T_STOP | – | 정지 → Z 상승 | OVERLOAD |
| 23 | SCOOP_DRAG | 매 주기 | `inside_tub_workspace` | X 전진 V_DRAG, F_x > F_TARGET이면 Z 상승(깊이 적응), 부피 적분 | – |
| 24 | SCOOP_DRAG | V ≥ V_target 또는 레인 끝 | – | X 정지 | SCOOP_CLOSE |
| 25 | SCOOP_DRAG | F_x > F_STOP, T_STOP, 깊이 여유 없음 | – | X 정지 → Z 상승 | OVERLOAD |
| 26 | SCOOP_DRAG | X 명령 대비 진행 0, 300 ms | – | 정지 | STALL |
| 27 | SCOOP_CLOSE | 진입 | – | θ → THETA_CAPTURE, T_CLOSE | – |
| 28 | SCOOP_CLOSE | θ 도달 | – | – | Z_LIFT |
| 29 | SCOOP_CLOSE | θ 전류 한계 > 500 ms | – | 정지 | STALL |
| 30 | Z_LIFT | 진입 | – | Z → Z_SAFE @ V_Z_FAST | – |
| 31 | Z_LIFT | **Z ≥ Z_SAFE** | – | 절삭 이력 지도 갱신 | XY_TO_CUP |
| 32 | XY_TO_CUP | 진입 | Z ≥ Z_SAFE | `request_xy_move(cup, TRAVEL)` | – |
| 33 | XY_TO_CUP | 도착 | – | – | BAY_CHECK |
| 34 | BAY_CHECK | 빔 비어 있음 | – | – | Z_DISPENSE |
| 35 | BAY_CHECK | 빔 차단 > T_BAY_WAIT | – | "손을 빼세요" | HAND_IN_BAY |
| 36 | Z_DISPENSE | 진입 | 빔 비어 있음 | Z → Z_C + R + 15 @ 50 mm/s | – |
| 37 | Z_DISPENSE | 도달 | – | – | EJECT |
| 38 | EJECT | 진입 | – | θ → THETA_ATTACK, 흔들기(±5°, 10 Hz, 0.5 s), `request_xy_move(comb, EJECT)` | – |
| 39 | EJECT | 완료 | – | – | Z_RETRACT |
| 40 | Z_RETRACT | Z ≥ Z_SAFE | – | – | PORTION_CHECK |
| 41 | PORTION_CHECK | 저울 안정 T_SETTLE_SCALE | – | 질량 기록·판정, k_f 갱신(EMA), 로그 | IDLE 또는 RINSE |
| 42 | PORTION_CHECK | 저울 증가 < 20 g | – | "배출 실패" | EJECT_FAIL |
| 43 | RINSE | 다음 맛 ≠ 현재 맛, 알레르겐, 또는 N 스쿱마다 | Z ≥ Z_SAFE로 이동 | 헹굼 스테이션 담금·흔들기 | IDLE |
| 44 | RETURN_HOME | 도착 | – | – | IDLE |
| 45 | OVERLOAD / STALL / NO_TUB | 진입 | – | X·θ 정지, Z 상승 가능하면 Z_SAFE까지, 원인 코드 | RECOVER |
| 46 | RECOVER | 작업자 확인 버튼 | 인터록 OK | 재원점 | HOMING |
| 47 | POSITION_ERROR | 진입 | – | 전 축 정지, `homed = false` | (작업자 리셋) → HOMING |
| 48 | E_STOP_OR_INTERLOCK | 해제 + **리셋 버튼** | – | 자동 재기동 금지 | HOMING |
| 49 | NO_CUP / HAND_IN_BAY / EJECT_FAIL | 해소 + 버튼 | – | – | 직전 단계 재개 또는 IDLE |

## 5. 타이머

| 타이머 | 시작 | 만료 시 |
|---|---|---|
| `t_cup_wait` 10 s | CHECK_CUP 진입 | NO_CUP |
| `t_bay_wait` 10 s | BAY_CHECK 진입 | HAND_IN_BAY |
| `t_touch` 20 ms | ΔF_z ≥ F_TOUCH 첫 샘플 | 접촉 확정 |
| `t_stop` 50 ms | F > F_STOP 첫 샘플 | OVERLOAD |
| `t_stall_x` 300 ms | X 진행 없음 | STALL |
| `t_stall_theta` 500 ms | θ 전류 한계 | STALL |
| `t_settle` 1 s | 헤드가 베이를 떠남 | 질량 확정 |
| `t_idle` 60 s | IDLE 진입 | RETURN_HOME |
| `t_state_watchdog` 상태별(예: 이송 5 s, 드래그 4 s) | 상태 진입 | FAULT(상태 멈춤) |

## 6. 드래그 루프 (1 kHz)

```c
void scoop_drag_tick(void) {
    float fx = lp_filter(loadcell_x());                 // 볼너트 로드셀, 200 Hz 저역통과
    // 깊이 적응: 목표보다 힘이 크면 Z를 올려 절삭단면을 줄인다(공격각 유지)
    if (fx > F_TARGET)       dz_cmd += K_DEPTH * (fx - F_TARGET) * DT;
    if (fx < 0.8 * F_TARGET && depth() < d_plan) dz_cmd -= K_DEPTH_DOWN * DT;
    dz_cmd = clamp(dz_cmd, 0, d_plan);                  // 계획 깊이보다 깊게는 안 간다
    z_set(Z0 - d_plan + dz_cmd);

    x_set(x + V_DRAG * DT);                             // request_xy_move(MOVE_DRAG) 경유
    vol += swept_area(THETA_ATTACK, depth()) * V_DRAG * DT;

    if (fx > F_STOP) { if (++n_stop > T_STOP) fault(OVERLOAD); } else n_stop = 0;
    if (vol >= V_target(flavor) || x >= lane_end) next(SCOOP_CLOSE);
}
```

`V_target = M_target / (ρ_nominal · k_f)` (맛별 보정, `docs/cup_dispensing_station.md` §5). `swept_area()`는 `calc/cartesian_model.py`와 같은 식이다.

## 7. 로그 형식 (CSV, 1 kHz 중 상태 전이·100 Hz 요약)

```
t_ms, state, event, X, Y, Z, theta, Fx, Fz, I_theta, vol_mm3, scale_g, flavor, lane, layer, Z0, fault_code
```

- 모든 전이와 거부된 XY 요청(`EV_XY_BLOCKED_LOW_Z`)을 기록한다. V1 시험(E2)에서 가드가 실제로 몇 번 막았는지가 안전 근거 자료가 된다.

## 8. 시험 전 확인 목록 (펌웨어 단위)

| 시험 | 방법 | 통과 기준 |
|---|---|---|
| Z_SAFE 게이트 | Z = Z_SAFE − 1 mm에서 XY 이송 명령 주입 | 100 % 거부, 로그 기록 |
| 베이 빔 | 이송·Z_DISPENSE 중 빔 차단 | 전 축 STO ≤ 50 ms, 버튼 없이는 재개 안 됨 |
| 이송 충돌 | 이송 경로에 스펀지 블록 | F_TRAVEL에서 정지, 블록 변형 ≤ 5 mm |
| 과부하 | 냉동 왁스 블록(F > F_STOP) | T_STOP 안에 정지 + Z 상승 |
| 정전 | 드래그 중 전원 차단 | Z 브레이크 잠김, 헤드가 천천히 위로(counterbalance) |
| 재기동 | E-stop 해제 | 리셋 버튼 없이는 움직이지 않음 |
