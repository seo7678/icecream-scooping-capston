"""Reaction-force path and compliance chain of the Cartesian machine.

Run:  python3 calc/scoop_load_path.py
Out:  calc/output/scoop_load_path.md

Question answered: when the scoop is dragged through hard ice cream, how far
does the scoop tip move away from where the machine thinks it is, and which
element is the weak link? Compared for:
  L  "3D-printer-like"      : MGN12 blocks, 4040 column, 4080 beam, GT2 belt X drive
  M  "reinforced profile"   : HGR15 blocks, 4080 column, 8080 beam, HTD5M-15 belt
  S  "machine-like"         : HGR15 blocks, steel 60x60x4 column, steel 100x50x4 beam, ball-screw X
and for the three architectures (docs/architecture_comparison.md):
  Arch 1  global X drags (drive in the loop)
  Arch 2  local x_scoop drags, global XY locked (global drive out of the loop)
  Arch 3  head docks on the tub deck (gantry out of the loop)

All section properties marked ASSUMPTION must be replaced with catalogue values.
Linear-elastic superposition; the load is quasi-static.
"""

import os

import cartesian_model as cm

OUT = os.path.join(os.path.dirname(__file__), "output", "scoop_load_path.md")

E_AL, G_AL = 69e3, 26e3        # MPa
E_ST, G_ST = 200e3, 79e3       # MPa (structural steel)
E_SS = 193e3                   # MPa (304 stem)


def rect_tube(b, h, t):
    """I about the axis parallel to b (bending in the h direction), thin-wall J."""
    I = (b * h ** 3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12.0
    a_m, b_m = h - t, b - t
    J = 2.0 * t * a_m ** 2 * b_m ** 2 / (a_m + b_m)
    return I, J


def round_tube(do, t):
    di = do - 2 * t
    return 3.14159265 * (do ** 4 - di ** 4) / 64.0


st_col_I, _ = rect_tube(60, 60, 4)
st_beam_I, st_beam_J = rect_tube(50, 100, 4)       # 100 mm vertical
st_beam_Iy, _ = rect_tube(100, 50, 4)              # lateral bending
STEM_I = {"25x3": round_tube(25, 3), "30x3": round_tube(30, 3)}

# Section properties of aluminium profiles: ASSUMPTION until catalogue values are in.
# Aluminium profile properties: Misumi HFS8 catalogue (research/components.md):
#   4040 Ix = Iy = 10.4 cm^4; 4080 Ix = 19.8 / Iy = 71.9 cm^4; 8080 Ix = Iy = 129.1 cm^4.
# Torsion constants: Bosch 40x40 It = 2.6 cm^4; 4080/8080 It NOT FOUND -> ASSUMPTION.
# Rail-block linear stiffness and belt/nut stiffness other than GT2 are ASSUMPTIONS.
CONFIGS = {
    "L": {
        "name": "3D-printer-like (MGN12, 4040 col, 4080 beam, GT2)",
        "col_EI": E_AL * 10.4e4, "beam_EI": E_AL * 71.9e4, "beam_EIy": E_AL * 19.8e4, "beam_GJ": G_AL * 6.0e4,
        "block_k": 60e3, "block_s": 60.0,            # N/mm per block, block spacing mm (ASSUMPTION)
        "drive": "belt", "belt_ksp": 18.9e3,         # GT2-6: ~18.9 kN per unit strain (derived from Gates 2M data)
    },
    "M": {
        "name": "reinforced profile (HGR15, 4080 col, 8080 beam, HTD5M-15)",
        "col_EI": E_AL * 71.9e4, "beam_EI": E_AL * 129.1e4, "beam_EIy": E_AL * 129.1e4, "beam_GJ": G_AL * 40.0e4,
        "block_k": 200e3, "block_s": 150.0,
        "drive": "belt", "belt_ksp": 150e3,          # HTD 5M 15 mm stiffness NOT FOUND (ASSUMPTION)
    },
    "S": {
        "name": "machine-like (HGR15, steel 60x60x4 col, steel 100x50x4 beam, ball screw)",
        "col_EI": E_ST * st_col_I, "beam_EI": E_ST * st_beam_I, "beam_EIy": E_ST * st_beam_Iy, "beam_GJ": G_ST * st_beam_J,
        "block_k": 200e3, "block_s": 150.0,
        "drive": "screw", "screw_dr": 16.8, "screw_L": 1100.0,   # SFU2010 root dia (estimated)
    },
}
CONFIGS["S30"] = dict(CONFIGS["S"], name="machine-like + stem 30x3", stem="30x3")
for _k in ("L", "M", "S"):
    CONFIGS[_k]["stem"] = "25x3"

BEAM_SPAN = 1450.0       # mm between bridge supports (V1 layout + carriage width)
H_BLOCK = 580.0          # mm, lower Z guide block above the deck (derived in docs/gantry_load_path.md)


def geometry(c_height):
    """Levers for a pivot height C [mm above deck]."""
    p_z = c_height - cm.Z_C_LEVER                    # force point
    col_bottom = c_height + cm.STEM_LEN + cm.HEAD_LEN
    overhang = H_BLOCK - col_bottom                  # column below the lower block
    below = cm.HEAD_LEN + cm.STEM_LEN + cm.Z_C_LEVER # column bottom -> force point
    lever = H_BLOCK - p_z
    return overhang, below, lever


def drive_k(cfg, x_pos=560.0, length=1150.0):
    if cfg["drive"] == "belt":
        l1, l2 = x_pos, length - x_pos
        return cfg["belt_ksp"] * (1.0 / l1 + 1.0 / l2)
    a = 3.14159265 * cfg["screw_dr"] ** 2 / 4.0
    k_screw = E_ST * a / cfg["screw_L"]
    k_nut, k_brg = 150e3, 150e3                      # N/mm (ASSUMPTION, preloaded class)
    return 1.0 / (1.0 / k_screw + 1.0 / k_nut + 1.0 / k_brg)


def tip_dx(cfg, F, c_height, arch):
    """Drag-direction tip displacement contributions [mm] for drag force F."""
    ov, below, lever = geometry(c_height)
    out = {}
    # stem: cantilever from head clamp to force point
    ls = cm.STEM_LEN + cm.Z_C_LEVER
    out["stem"] = F * ls ** 3 / (3 * E_SS * STEM_I[cfg["stem"]])
    if arch == 3:
        return out                                   # head docked at deck level: gantry not in the loop
    # Z column: cantilever of length ov with tip force F and tip moment F*below
    M = F * below
    EI = cfg["col_EI"]
    d_end = F * ov ** 3 / (3 * EI) + M * ov ** 2 / (2 * EI)
    rot = F * ov ** 2 / (2 * EI) + M * ov / EI
    out["column"] = d_end + rot * below
    # Z guide block pair and X carriage block pair (rotational springs)
    K_rot = cfg["block_k"] * cfg["block_s"] ** 2 / 2.0
    out["Z blocks"] = F * lever / K_rot * lever
    out["X blocks"] = F * lever / K_rot * lever
    # X beam bending in the vertical plane under the pitch moment at mid-span
    rot_beam = F * lever * BEAM_SPAN / (12 * cfg["beam_EI"])
    out["beam"] = rot_beam * lever
    # global drive in the loop only for Arch 1
    if arch == 1:
        out["X drive"] = F / drive_k(cfg)
    return out


def tip_dy(cfg, Fy, c_height, arch):
    """Side-direction tip displacement [mm] for side force Fy (beam torsion + lateral bending)."""
    ov, below, lever = geometry(c_height)
    if arch == 3:
        ls = cm.STEM_LEN + cm.Z_C_LEVER
        return Fy * ls ** 3 / (3 * E_SS * STEM_I[cfg["stem"]])
    T = Fy * lever
    phi = T * BEAM_SPAN / (4 * cfg["beam_GJ"])       # torsion, both ends restrained
    lat = Fy * BEAM_SPAN ** 3 / (48 * cfg["beam_EIy"])
    return phi * lever + lat


lines = []
p = lines.append
p("# Scoop reaction load path & compliance — generated")
p("")
p("> `python3 calc/scoop_load_path.py`. 절삭력은 **ASSUMPTION CASE**, 알루미늄 프로파일 단면값·블록 강성·벨트 강성도 **ASSUMPTION**(카탈로그로 교체).")
p(f"> 기하: 스템 {cm.STEM_LEN:.0f} mm, 헤드 {cm.HEAD_LEN:.0f} mm, Z 하부 블록 높이 {H_BLOCK:.0f} mm(데크 기준), 빔 스팬 {BEAM_SPAN:.0f} mm.")
p("")
for c_height, label in ((cm.z_min(), "가장 깊은 스쿱(통 바닥 근처)"), (-80.0, "중간 수위")):
    ov, below, lever = geometry(c_height)
    p(f"## 드래그 방향 팁 변위 — {label}: C = {c_height:.0f} mm, 모멘트 레버 {lever:.0f} mm")
    p("")
    p("| 구성 | 아키텍처 | F_x [N] | stem | column | Z blocks | X blocks | beam | X drive | **합계 [mm]** | 강성 [N/mm] |")
    p("|---|---|---|---|---|---|---|---|---|---|---|")
    for key, cfg in CONFIGS.items():
        for arch in (1, 2, 3):
            if arch == 3 and key not in ("S", "S30"):
                continue
            F = 200.0
            d = tip_dx(cfg, F, c_height, arch)
            tot = sum(d.values())
            cols = [f"{d.get(k, 0.0):.2f}" if k in d else "–" for k in ("stem", "column", "Z blocks", "X blocks", "beam", "X drive")]
            p(f"| {key} | {arch} | {F:.0f} | " + " | ".join(cols) + f" | **{tot:.2f}** | {F/tot:.0f} |")
    p("")

p("## 옆방향(F_y = 0.3·F_x = 60 N) 팁 변위 — 가장 깊은 스쿱")
p("")
p("| 구성 | 아키텍처 1/2 [mm] | 아키텍처 3 [mm] |")
p("|---|---|---|")
for key, cfg in CONFIGS.items():
    a12 = tip_dy(cfg, 60.0, cm.z_min(), 1)
    a3 = tip_dy(cfg, 60.0, cm.z_min(), 3)
    p(f"| {key} | {a12:.2f} | {a3:.2f} |")
p("")

p("## 강도·블록 하중 (F_x = 200 N, 가장 깊은 스쿱)")
p("")
ov, below, lever = geometry(cm.z_min())
M = 200.0 * lever
p(f"- Z 하부 블록 위치 굽힘모멘트 M = 200 N × {lever:.0f} mm = **{M/1e3:.0f} N·m**")
p(f"- 강관 60×60×4 기둥 응력 σ = M/(I/c) = {M/(st_col_I/30):.1f} MPa (여유 큼) · 4040 알루미늄(I = 8 cm⁴ 가정): {M/(8.0e4/20):.1f} MPa")
p(f"- 스템 Ø25×3 클램프부 σ = {200*(cm.STEM_LEN+cm.Z_C_LEVER)/(STEM_I['25x3']/12.5):.0f} MPa (피로 목표 50 MPa 초과) · Ø30×3: {200*(cm.STEM_LEN+cm.Z_C_LEVER)/(STEM_I['30x3']/15.0):.0f} MPa")
for s in (60.0, 150.0):
    p(f"- X 캐리지 블록 간격 {s:.0f} mm, 블록 4개: 블록당 짝힘 ≈ M/(2·s) = {M/(2*s):.0f} N")
p(f"- 브리지 양끝(Y 캐리지)에 걸리는 수직 짝힘 ≈ M / 스팬 = {M/BEAM_SPAN:.0f} N")
p("")
p("카탈로그 정적 정격(research/components.md, snippet 기반):")
p("")
p("| 블록 | C0 [kN] | M0R / M0P [N·m] | 161 N·m를 블록 하나로? | 짝힘(간격 150 mm, 4블록) 대비 C0 |")
p("|---|---|---|---|---|")
for name, c0, m0r, m0p in (("MGN12H", 5.88, 38.2, 36.3), ("HGH15CA", 16.97, 120.0, 100.0), ("HGH20CA", 27.76, 270.0, 200.0)):
    ok = "가능" if m0p >= M / 1e3 else "불가"
    p(f"| {name} | {c0} | {m0r:.0f} / {m0p:.0f} | {ok} | {M/(2*150.0)/1e3:.2f} kN → 여유 {c0/(M/(2*150.0)/1e3):.0f}배 |")
p("")
p("HGH15/20의 M0P 값은 열 매핑이 UNCERTAIN. 결론은 같다: **용량은 충분하고, 강성(변위)이 설계를 지배한다.** MGN12는 모멘트를 블록 하나로 받을 수 없으므로 간격을 벌린 짝힘 배치가 필수이고, 그래도 강성 때문에 탈락.")
p("")
p("**해석 규칙:** 레일·블록의 *하중 용량*은 kN급이라 문제가 아니고, **강성(변위)이 설계를 지배한다.** 목표: 드래그 방향 팁 변위 ≤ 1 mm @ 200 N(강성 ≥ 200 N/mm).")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {OUT}")
