# Coordinate Map Example (V1)

> 개념·좌표계 정의는 `docs/coordinate_and_flavor_mapping.md`. 이 문서는 **저장 형식과 조회·레인 선택 코드**의 예시다.
> 좌표값은 `calc/cartesian_model.py`의 `LAYOUT_V1` 예시값이다. 실제 값은 Calibration Mode에서 등록한다.

## 1. 저장 형식 (flash의 `map.json`, CRC 포함)

```json
{
  "version": 3,
  "units": "mm",
  "machine": { "z_safe": 60.0, "z_min": -205.0, "z_top": 160.0, "cup_rim_z": 0.0 },
  "stations": {
    "home":  { "x": 0.0,    "y": 0.0 },
    "rinse": { "x": 120.0,  "y": 0.0,  "dip_z": -60.0 },
    "cup":   { "x": 1120.0, "y": 0.0,  "dispense_z": 50.0, "comb_dx": -40.0 }
  },
  "tub_model": { "d_top": 230.0, "d_bottom": 210.0, "depth": 250.0 },
  "flavors": [
    {
      "flavor_id": "VAN", "name": "Vanilla", "tub_slot": 1,
      "tub_center": [340.0, 0.0], "rim_z": 0.0,
      "surface_z_est": -62.0, "lane_set": [-40.0, 0.0, 40.0],
      "layer": 3, "lane_idx": 1,
      "scoop": { "attack_deg": -30.0, "d_max": 28.0, "f_target": 110.0 },
      "k_portion": 1.00, "allergen": false,
      "status": "OK", "map_valid": true
    },
    {
      "flavor_id": "CHO", "name": "Chocolate", "tub_slot": 2,
      "tub_center": [600.0, 0.0], "rim_z": 0.5,
      "surface_z_est": -118.0, "lane_set": [-40.0, 0.0, 40.0],
      "layer": 6, "lane_idx": 0,
      "scoop": { "attack_deg": -30.0, "d_max": 24.0, "f_target": 95.0 },
      "k_portion": 0.96, "allergen": false,
      "status": "OK", "map_valid": true
    },
    {
      "flavor_id": "PNB", "name": "Peanut Butter", "tub_slot": 3,
      "tub_center": [860.0, 0.0], "rim_z": -0.3,
      "surface_z_est": -214.0, "lane_set": [0.0],
      "layer": 11, "lane_idx": 0,
      "scoop": { "attack_deg": -30.0, "d_max": 28.0, "f_target": 110.0 },
      "k_portion": 1.03, "allergen": true,
      "status": "LOW", "map_valid": true
    }
  ],
  "crc32": "0x5A3C19E2"
}
```

- 절삭 이력 **높이 지도**(10 mm 격자, 통당 ~530셀, int16 0.1 mm 단위 ≈ 1 KB)는 맛마다 별도 파일 `surf_<slot>.bin`에 둔다.
- `PNB`는 거의 빈 통이라 옆 레인에서 1 portion이 안 나와(`tub_lane_planner.md` §1) `lane_set`이 중앙 한 줄로 줄었다. `allergen: true`라 전용 식품 모듈을 요구한다.

## 2. 조회

```python
def lookup(flavor_id, order_size):
    f = MAP.flavor(flavor_id)
    if f is None or f.status in ("EMPTY", "DISABLED"):
        return Reject("통 교체/점검")
    if not f.map_valid:
        return Reprobe(f.tub_slot)                 # 3점 프로브 후 지도 재초기화
    if f.allergen and HEAD.food_module != "ALLERGEN":
        return Reject("알레르겐 전용 식품 모듈로 교체")
    lane_y, z_surf = select_lane(f)                # §3, 표면(rim 최저점 기준) 높이
    x0, y0 = f.tub_center
    x_start, x_end = lane_limits(f, lane_y, z_surf) # §4
    return Plan(
        start=(x0 + x_start, y0 + lane_y), end_x=x0 + x_end,
        z_est=z_surf + f.rim_z + B_ATTACK,         # 펌웨어의 Z_est = 표면에 닿을 때의 C 높이
        d=f.scoop.d_max, f_target=f.scoop.f_target,
        v_target=M_TARGET[order_size] / (RHO_NOMINAL * f.k_portion),
    )
```

## 3. 레인 선택 — 한 자리만 파지 않기

```python
def select_lane(f):
    """층별 평삭: 현재 층에서 아직 층 바닥까지 안 깎인 레인 중, 표면이 가장 높은 레인."""
    surf = SURFACE_MAP[f.tub_slot]
    lanes = [(y, surf.lane_median(y)) for y in f.lane_set]
    layer_floor = max(h for _, h in lanes) - f.scoop.d_max
    open_lanes = [(y, h) for y, h in lanes if h > layer_floor + 3.0]   # 3 mm 이하 차이는 같은 층
    y, h = max(open_lanes, key=lambda t: t[1])
    return y, h
```

- 스쿱이 끝나면 `surf.cut(lane_y, x_start, x_end, depth_profile)`로 셀을 깎는다. 드래그 중 깊이 적응으로 실제로 깎인 깊이(로그의 Z)를 쓴다.
- 다음 스쿱의 터치오프로 얻은 표면(Z0 − 30.3)과 지도 예측의 차이가 **> 8 mm**이면 사람이 통을 건드린 것으로 보고 `map_valid = false`로 둔다(다음 주문 전 3점 재프로브).
- 이 규칙으로 통 전체를 소진하는 시뮬레이션: `calc/output/scoop_mechanism_compare.md`(바닥까지, 통 부피의 약 59 %).

## 4. Keep-out 계산

```python
from math import cos, radians
R_SCOOP, WALL_MARGIN, FLOOR_MARGIN = 35.0, 10.0, 10.0

def tub_radius(f, depth_below_rim):
    t = min(1.0, max(0.0, depth_below_rim / MAP.tub_model.depth))
    return 0.5 * (MAP.tub_model.d_top + (MAP.tub_model.d_bottom - MAP.tub_model.d_top) * t)

def workspace_radius(f, z_c):
    """C가 머물 수 있는 원의 반경. 스쿱 바닥(z_c - R)이 있는 깊이의 통 반경으로 계산(테이퍼 → 보수적)."""
    return tub_radius(f, -(z_c - R_SCOOP - f.rim_z)) - R_SCOOP - WALL_MARGIN

B_ATTACK = R_SCOOP * cos(radians(30.0))         # rim 최저점이 C보다 30.3 mm 아래(공격 자세)

def lane_limits(f, lane_y, z_surface):
    z_c = z_surface - f.scoop.d_max + B_ATTACK       # 드래그 중 C 높이
    r = workspace_radius(f, z_c)
    if abs(lane_y) >= r:
        raise LaneUnreachable
    half = (r * r - lane_y * lane_y) ** 0.5
    return -half, +half                           # 드래그는 항상 +X_M 방향

def inside_tub_workspace(f, x, y, z_c):
    x0, y0 = f.tub_center
    if z_c < MAP.machine.z_min:                   # 바닥 keep-out: C >= -250 + 10 + R
        return False
    return (x - x0) ** 2 + (y - y0) ** 2 <= workspace_radius(f, z_c) ** 2
```

## 5. Z_SAFE 가드 (호스트 측 계획 단계에서 한 번 더)

```python
def plan_xy(path, z_now):
    """계획 단계 검사. 실시간 게이트(firmware/state_machine.md §3.1)와 별도로, 계획된 모든 XY 구간을 검사한다."""
    for seg in path:
        if seg.kind == "TRAVEL" and z_now < MAP.machine.z_safe:
            raise PlanError(f"XY travel at Z={z_now:.1f} < Z_SAFE={MAP.machine.z_safe}")
        if seg.kind == "DRAG" and not all(inside_tub_workspace(ACTIVE, x, y, z_now) for x, y in seg.samples(5.0)):
            raise PlanError("drag leaves tub workspace")
        if seg.kind == "EJECT" and not inside_cup_bay(seg.end):
            raise PlanError("eject move outside cup bay")
        z_now = seg.z_end
```

계획 단계 검사와 실시간 게이트를 **둘 다** 둔다. 앞의 것은 좌표표 오류를 동작 전에 잡고, 뒤의 것은 실행 중 상태가 어긋난 경우를 잡는다.

## 6. 예시: "Chocolate, 싱글" 한 번

| 단계 | 값 |
|---|---|
| 조회 | slot 2, 중심 (600, 0), 표면 추정 −118 + 0.5 = −117.5 mm |
| 레인 | 층 6, 레인 y = 0 (표면이 가장 높음) |
| 레인 범위 | 드래그 중 C = −117.5 − 24 + 30.3 = −111.2 → 볼 최저점 깊이 ~146 mm에서 통 반경 ≈ 109 mm → r_c ≈ 64 mm → x ∈ [536, 664] (길이 128 mm) |
| 시작점 | (536, 0), θ = −30°, Z_SAFE 60 mm에서 도착 |
| 프로브 | Z_est(C) = −117.5 + 30.3 = −87.2 → C를 −77.2까지 빠르게 → 10 mm/s → 접촉 Z0 = −88.9 (표면 −119.2, 지도와 −1.7 mm 차이, 정상) |
| 목표 부피 | 115 g / (0.65 g/cm³ × 0.96) = 184 cm³ |
| 드래그 | d_max 24 mm, F_target 95 N(단단한 맛 설정) → 부피 도달 또는 x = 664 |
| 배출 | 컵 (1120, 0), Z 50 mm → 빗 걸기 −40 mm |
| 기록 | 적분 부피 184 cm³ → 예측 0.65 × 184 = 119.6 g, 저울 112 g → k_f ← 0.96 + 0.2 × (112/119.6 − 0.96) = 0.955, 다음 목표 185 cm³. 지도 셀 갱신 |
