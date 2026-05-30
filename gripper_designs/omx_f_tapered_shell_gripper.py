"""
OMX_F Tapered Hollow-Shell Gripper Finger (L side).

좌표계 (원본 STL × 1000 후 mm):
  Y = 핑거 길이축 (모터 피벗 = Y0, 블레이드는 +Y 로 뻗음)
  Z = 세로 높이 (위 +Z, 아래 -Z)
  X = 두께 / 그립 닫힘 방향 (-X 가 짝 핑거 방향 = 안쪽 / 그리핑 면)

설계 개요:
  - 하단 (Y=cut_y) 은 원본 STL 의 모터 마운트/하우징을 그대로 사용.
  - 그 위에 새 테이퍼드 hollow shell 블레이드를 union (살짝 overlap 하여 빈틈 없이).
  - 베이스 단면(Y=cut_y) X·Z 는 원본 STL 의 cut 위치 단면과 매칭.
  - 팁(Y=cut_y+blade_length) 단면은 X(두께)·Z(높이) 양쪽 모두 줄어듦 → 진짜 테이퍼.
  - 안쪽 그리핑면(-X) 과 바닥면(-Z) 은 길이 방향으로 평면 유지 (그립/안정성).
  - 바깥면(+X) 과 윗면(+Z) 만 안쪽으로 슬로프 → 직각삼각형에 가까운 단면.
  - 외벽만 남기고 안을 비운 쉘 (boolean cut, loft 인셋).
  - 내부에 일정 간격 가로 지지대(ribs) — 사다리 모양.
  - 안쪽 (-X) 면에 작은 사각뿔 teeth 패턴 → 그립력 향상.
"""

import cadquery as cq                               # CAD 코드 메인 라이브러리
import trimesh                                      # STL boolean union 용
import numpy as np                                  # 수치 연산용
from pathlib import Path                            # 파일 경로 처리용

# ============================================================
# PARAMETERS  ── 여기 숫자만 바꾸면 모델이 다시 생성됨
# ============================================================

# ── 원본 STL (모터 마운트) ─────────────────────────────────
SRC_STL = "../model/omx_f_mesh/follower_gripper_l.stl"  # 원본 L 핑거 STL 경로
STL_UNIT_SCALE = 1000.0                              # STL 이 미터 단위 → mm 로 환산
cut_y = 22.0                                         # 이 Y 값 이후를 잘라내고 새 블레이드로 교체 (mm) — 살짝 더 일찍 자름
joint_overlap = 4.0                                  # 블레이드 베이스가 하우징 안쪽으로 파고드는 깊이 (mm) — 깊을수록 빈틈 없음

# ── 베이스 단면 (Y = cut_y, 하우징 외형보다 약간 크게 → 완전 envelope) ─
# 원본 STL Y∈[18..23] 부근 단면 계측: X 최대 [-9.96, 2.40], Z 최대 [-36, 4]
# 블레이드 베이스는 그보다 0.1~0.3mm 씩 크게 → 어느 단면에서도 하우징을 감싸 union 매끈
base_x_inner = -10.2                                 # 베이스 안쪽 X (-10 보다 살짝 -X 로)
base_x_outer =   2.5                                 # 베이스 바깥 X (하우징 최대 +X 2.4 보다 큼)
base_z_bottom = -36.0                                # 베이스 바닥 Z (하우징 바닥과 동일)
base_z_top    =   4.0                                # 베이스 윗변 Z (하우징 윗변과 동일)

# ── 팁 단면 (Y = cut_y + blade_length, 양방향 축소) ────
blade_length    = 100.0                              # 블레이드 길이 (Y 방향, mm)
tip_x_inner     = -10.2                              # 팁 안쪽 X — base 와 동일하게 두면 -X 면이 평면 유지
tip_x_outer     =  -4.0                              # 팁 바깥 X — base 보다 더 -X 로 → X 두께 감소 (base 12.7mm → tip 6.2mm)
tip_z_bottom    = -36.0                              # 팁 바닥 Z — base 와 동일하게 두면 -Z 면이 평면 유지
tip_z_top       = -26.0                              # 팁 윗변 Z — base 보다 낮춰 Z 높이 감소 (base 40mm → tip 10mm)

# 베이스/팁 두께·높이를 별도 변수로도 확인 가능 (편의)
blade_corner_r   =  1.0                              # 블레이드 외곽 코너 라운딩 반경 (mm)

# ── 쉘 (외벽 두께) ────────────────────────────────────────
wall_thickness   = 2.0                               # 외벽 두께 (mm) — FDM 권장 ≥ 1.2
end_wall_thickness = 2.0                             # 팁 끝(닫힌 면) 두께 (mm)

# ── 가로 지지대 (사다리 ribs) ────────────────────────────
num_ribs         = 5                                 # 사다리 가로대 개수 (변수)
rib_thickness    = 2.0                               # 각 가로대 Y 방향 두께 (mm)
rib_start_margin = 6.0                               # 첫 가로대 ~ 베이스 사이 여유 (mm)
rib_end_margin   = 6.0                               # 마지막 가로대 ~ 팁 사이 여유 (mm)

# ── Lightening 구멍 (rib 사이 -X 면 트러스 컷아웃) ──────
cutout_enabled    = True                             # 컷아웃 ON/OFF
cutout_face       = "-X"                             # 뚫을 면: "-X" 또는 "+X" (ridge 면 반대 추천)
cutout_y_margin   = 2.5                              # 좌우 rib 으로부터의 Y 여유 (mm)
cutout_z_margin   = 4.0                              # 위/아래 외벽으로부터의 Z 여유 (mm)
cutout_corner_r   = 1.5                              # 컷아웃 모서리 라운딩 반경 (mm)
cutout_min_size   = 4.0                              # 너무 작으면 (Y 또는 Z 한 변 < 이값) 그 bay 는 skip

# ── 그립 ridge (세로 길쭉한 삼각형 돌기) ─────────────────
ridge_enabled    = True                              # ridge 패턴 ON/OFF
ridge_face       = "+X"                              # 돌출 면: "+X" (바깥) 또는 "-X" (안쪽)
ridge_pitch_y    = 7.0                               # ridge 간 Y 방향 간격 (mm)
ridge_width      = 2.5                               # ridge 베이스 Y 방향 폭 (mm) — 얇을수록 날카로움
ridge_height     = 1.5                               # ridge 돌출 높이 (X 방향, mm)
ridge_y_start_margin = 6.0                           # 베이스에서부터 ridge 시작 여유 (mm)
ridge_y_end_margin   = 6.0                           # 팁까지 ridge 끝 여유 (mm)
ridge_z_margin   = 3.0                               # 위/아래 가장자리에서 ridge 떨어뜨리는 여유 (mm)

# ── 출력 ─────────────────────────────────────────────────
OUT_BLADE_STL = "omx_f_tapered_shell_blade_only.stl" # 블레이드만 (디버그용)
OUT_FINAL_STL = "omx_f_tapered_shell_gripper_L.stl"  # 최종 union 결과
OUT_DIR = Path(__file__).parent                      # 출력 디렉토리 = 이 파일 폴더

# 검증
assert wall_thickness * 2 < (tip_x_outer - tip_x_inner), "wall 이 너무 두꺼움 — 팁에서 내부 공간이 없어짐"  # 안전 체크

# ============================================================
# 유도 변수
# ============================================================
y_base  = cut_y - joint_overlap                      # 실제 블레이드 시작 Y (하우징 안쪽으로 살짝 파고듦)
y_tip   = cut_y + blade_length                       # 블레이드 팁 Y
blade_total_y = y_tip - y_base                       # 블레이드 Y 길이 (overlap 포함)

# 보조 함수: t∈[0,1] 에서 단면 사각형 4 점 반환 (loft 용)
def cross_section_at(t):                             # t=0 베이스, t=1 팁
    xi = base_x_inner + (tip_x_inner - base_x_inner) * t   # 안쪽 X 보간
    xo = base_x_outer + (tip_x_outer - base_x_outer) * t   # 바깥 X 보간
    zb = base_z_bottom + (tip_z_bottom - base_z_bottom) * t  # 바닥 Z 보간
    zt = base_z_top    + (tip_z_top    - base_z_top)    * t  # 윗 Z 보간
    return xi, xo, zb, zt                            # 4 값 반환

# ============================================================
# 1) 외곽 (Outer Shell)  — base→tip 직사각 단면 loft
# ============================================================
def make_loft(y0, y1, get_section):
    """y0, y1 사이를 사각 단면 loft 로 채움.
    get_section(t) → (xi, xo, zb, zt) 반환. Workplane("XZ") 평면 노멀=Y."""
    xi0, xo0, zb0, zt0 = get_section(0.0)            # 베이스 단면
    xi1, xo1, zb1, zt1 = get_section(1.0)            # 팁 단면
    # CadQuery "XZ" 평면 노멀은 -Y → offset 부호 뒤집어서 +Y 로 가도록
    return (
        cq.Workplane("XZ")                           # XZ 평면 (노멀 -Y, offset 부호 반전 필요)
        .workplane(offset=-y0)                       # 평면을 Y=y0 으로 (offset 부호 뒤집기)
        .polyline([(xi0, zb0), (xo0, zb0), (xo0, zt0), (xi0, zt0)])  # 베이스 4 점
        .close()                                     # 닫기
        .workplane(offset=-(y1 - y0))                # Y=y1 평면으로 추가 이동 (-부호)
        .polyline([(xi1, zb1), (xo1, zb1), (xo1, zt1), (xi1, zt1)])  # 팁 4 점
        .close()                                     # 닫기
        .loft(combine=True, ruled=True)              # ruled loft
    )

outer_solid = make_loft(y_base, y_tip, cross_section_at)  # 외곽 loft
# 외곽 코너 라운딩 — Y 방향 4 모서리만 살짝 부드럽게
try:
    outer_solid = outer_solid.edges("|Y").fillet(blade_corner_r)  # Y 평행 모서리 fillet
except Exception as _e:                              # 일부 모서리에서 fillet 실패 시 무시
    print(f"[warn] outer fillet skipped: {_e}")     # 경고

# ============================================================
# 2) 안쪽 캐비티 (Inner Cavity) — 각 변마다 wall 만큼 인셋한 loft
# ============================================================
# 베이스 끝벽: y_base + 0 (overlap 영역) → 캐비티는 cut_y 부터 시작해도 무방
#   여기서는 단순화: Y 양 끝도 wall 만큼 들여서 막힌 박스 형태로 cut.
inner_y0 = y_base + wall_thickness                   # 캐비티 Y 시작
inner_y1 = y_tip  - end_wall_thickness               # 캐비티 Y 끝

def inner_section_at(t):                             # 캐비티 단면 — 외곽에서 wall 인셋
    xi, xo, zb, zt = cross_section_at(t)             # 외곽 단면
    return (xi + wall_thickness,                     # 안쪽 X +wall (캐비티 안쪽)
            xo - wall_thickness,                     # 바깥 X -wall
            zb + wall_thickness,                     # 바닥 +wall
            zt - wall_thickness)                     # 윗변 -wall

# 캐비티의 t 매핑: 외곽 좌표계 (y_base..y_tip) 의 inner_y0..inner_y1 구간을
# 외곽 단면 보간기에 매핑 → 캐비티가 외곽 슬로프를 그대로 따라감
def cavity_section_at_y(y):                          # 임의 y 에서 캐비티 단면
    t_outer = (y - y_base) / blade_total_y           # 외곽 t
    xi, xo, zb, zt = cross_section_at(t_outer)       # 외곽 단면
    return (xi + wall_thickness, xo - wall_thickness,
            zb + wall_thickness, zt - wall_thickness)

xi0, xo0, zb0, zt0 = cavity_section_at_y(inner_y0)   # 캐비티 베이스 단면
xi1, xo1, zb1, zt1 = cavity_section_at_y(inner_y1)   # 캐비티 팁 단면

# 캐비티 두께/높이가 양수인지 검증
assert xo0 - xi0 > 0.1, f"베이스 캐비티 X 두께 부족: {xo0-xi0}"
assert xo1 - xi1 > 0.1, f"팁 캐비티 X 두께 부족: {xo1-xi1} — wall 줄이거나 tip 두께 키워라"
assert zt0 - zb0 > 0.1, f"베이스 캐비티 Z 높이 부족"
assert zt1 - zb1 > 0.1, f"팁 캐비티 Z 높이 부족"

inner_cavity = (
    cq.Workplane("XZ")                               # XZ 평면 (노멀 -Y)
    .workplane(offset=-inner_y0)                     # offset 부호 반전 → Y=inner_y0
    .polyline([(xi0, zb0), (xo0, zb0), (xo0, zt0), (xi0, zt0)])
    .close()
    .workplane(offset=-(inner_y1 - inner_y0))        # Y=inner_y1 까지 추가 이동
    .polyline([(xi1, zb1), (xo1, zb1), (xo1, zt1), (xi1, zt1)])
    .close()
    .loft(combine=True, ruled=True)                  # ruled loft
)

shell = outer_solid.cut(inner_cavity)                # 외곽 − 캐비티 = 쉘

# ============================================================
# 3) 가로 지지대 (Horizontal Ribs) — 사다리 모양
# ============================================================
rib_zone_y_start = inner_y0 + rib_start_margin       # rib 배치 시작 Y
rib_zone_y_end   = inner_y1 - rib_end_margin         # rib 배치 끝 Y
if num_ribs >= 2:                                    # 2 개 이상 → 균등
    rib_step = (rib_zone_y_end - rib_zone_y_start) / (num_ribs - 1)
    rib_ys = [rib_zone_y_start + i*rib_step for i in range(num_ribs)]
elif num_ribs == 1:                                  # 1 개 → 중앙
    rib_ys = [(rib_zone_y_start + rib_zone_y_end)/2]
else:                                                # 0 → 없음
    rib_ys = []

def rib_at(y_pos):                                   # 그 Y 의 캐비티 단면 꽉 채운 슬랩
    xi, xo, zb, zt = cavity_section_at_y(y_pos)      # 캐비티 단면 보간
    w_x = xo - xi                                    # X 폭
    h_z = zt - zb                                    # Z 높이
    cx  = (xi + xo) / 2                              # X 중심
    cz  = (zb + zt) / 2                              # Z 중심
    return (
        cq.Workplane("XY")                           # 기본 평면
        .box(w_x, rib_thickness, h_z,                # X·Y·Z 박스
             centered=(True, True, True))            # 중심 정렬
        .translate((cx, y_pos, cz))                  # 위치
    )

ribs_solid = None                                    # 누적
for y in rib_ys:                                     # 각 rib 위치
    one = rib_at(y)                                  # rib 1 개
    ribs_solid = one if ribs_solid is None else ribs_solid.union(one)

shell_with_ribs = shell                              # 시작
if ribs_solid is not None:                           # 있으면 union
    shell_with_ribs = shell_with_ribs.union(ribs_solid)

# ============================================================
# 4) 그립 Ridge — 세로(Z 방향) 길쭉한 삼각형 돌기
# ============================================================
# 각 ridge = 외벽 한 면에서 ±X 방향으로 돌출되는 얇은 삼각형 프리즘.
# XY 평면에서 삼각형 (base 두 점이 외벽 위, peak 한 점이 더 바깥) 그리고
# Z 축으로 길이 = (local_zt - local_zb - 2*margin) 만큼 extrude.

def build_ridges():                                  # 모든 ridge 를 하나의 solid 로 반환
    if not ridge_enabled:                            # OFF 면 None
        return None                                  # 종료
    ridges_union = None                              # 누적 solid
    yy = cut_y + ridge_y_start_margin                # 시작 Y (cut_y 이후만 → 하우징 침범 방지)
    while yy <= y_tip - ridge_y_end_margin:          # 팁 여유까지
        t = (yy - y_base) / blade_total_y            # 외곽 단면 t
        xi_local, xo_local, zb_local, zt_local = cross_section_at(t)  # 그 Y 의 외곽 단면
        # 그 Y 에서 ridge 가 차지할 Z 범위 (위/아래 여유 빼고)
        z_bot = zb_local + ridge_z_margin            # ridge 바닥 Z
        z_top = zt_local - ridge_z_margin            # ridge 상단 Z
        z_len = z_top - z_bot                        # ridge 세로 길이
        if z_len < 1.0:                              # 너무 짧으면 skip (팁 근처)
            yy += ridge_pitch_y                      # 다음 Y
            continue                                 # 다음 루프
        # 돌출 면 선택 — "+X" 면은 슬로프(테이퍼)지만 그 Y 의 외벽 X 좌표를 그대로 사용
        if ridge_face == "+X":                       # 바깥 면
            base_x = xo_local                        # 외벽 X
            peak_x = xo_local + ridge_height         # peak (바깥)
        else:                                        # 안쪽 면
            base_x = xi_local                        # 외벽 X
            peak_x = xi_local - ridge_height         # peak (안쪽)
        # XY 평면 (노멀 +Z) 에 삼각형 단면 → Z 방향으로 extrude
        # 삼각형 3 점: base 두 점 (Y±width/2, X=base_x) + peak 점 (Y=yy, X=peak_x)
        ridge = (
            cq.Workplane("XY")                       # 기본 XY 평면
            .polyline([                              # 삼각형 폴리라인
                (base_x, yy - ridge_width/2),        # base 1
                (base_x, yy + ridge_width/2),        # base 2
                (peak_x, yy),                        # peak (날카로운 끝)
            ])
            .close()                                 # 닫기
            .extrude(z_len)                          # Z 방향 extrude = ridge 세로 길이
            .translate((0, 0, z_bot))                # Z = z_bot 위치로 이동
        )
        ridges_union = ridge if ridges_union is None else ridges_union.union(ridge)  # 누적
        yy += ridge_pitch_y                          # 다음 Y

    return ridges_union                              # 결과

ridges_solid = build_ridges()                        # ridge 생성
blade_with_ridges = shell_with_ribs                  # 시작
if ridges_solid is not None:                         # 있으면 union
    blade_with_ridges = blade_with_ridges.union(ridges_solid)

# ============================================================
# 5) Lightening 컷아웃 — rib 사이 외벽에 둥근 사각 구멍
# ============================================================
# rib 사이 Y 구간(bay)마다 -X (또는 +X) 외벽을 관통하는 둥근 사각 prism 을 cut.
# 외벽 두께만큼만 뚫으면 캐비티와 연결됨 → 구조 강도는 rib + 양쪽 벽이 유지.

def build_cutouts():                                 # 모든 cutout solid (cut 용) 반환
    if not cutout_enabled or not rib_ys:             # OFF 또는 rib 없으면 None
        return None                                  # 종료
    # bay 경계 Y 좌표: 처음 = inner_y0, rib_ys, 마지막 = inner_y1
    edges_y = [inner_y0] + rib_ys + [inner_y1]       # bay 분할 경계 (mm)
    cuts_union = None                                # 누적
    for i in range(len(edges_y) - 1):                # 인접 경계 쌍 = 한 bay
        y_lo = edges_y[i] + cutout_y_margin          # bay 내 컷아웃 Y 시작
        y_hi = edges_y[i+1] - cutout_y_margin        # bay 내 컷아웃 Y 끝
        if y_hi - y_lo < cutout_min_size:            # 너무 좁으면 skip
            continue                                 # 다음 bay
        # bay 중심 Y 에서의 외곽 단면 → Z 범위 결정
        y_mid = (y_lo + y_hi) / 2                    # bay 중심 Y
        t = (y_mid - y_base) / blade_total_y         # 외곽 t
        xi_local, xo_local, zb_local, zt_local = cross_section_at(t)  # 단면
        z_lo = zb_local + cutout_z_margin            # 컷 Z 하단
        z_hi = zt_local - cutout_z_margin            # 컷 Z 상단
        if z_hi - z_lo < cutout_min_size:            # 너무 짧으면 skip
            continue                                 # 다음 bay
        # 둥근 사각 단면 (YZ 평면), X 방향으로 외벽 두께만큼 관통
        if cutout_face == "-X":                      # 안쪽 면 관통
            x_start = xi_local - 0.5                 # 외벽 바깥쪽에서 시작 (0.5mm 마진)
            x_end   = xi_local + wall_thickness + 1.0  # 캐비티 안쪽까지 (wall + 여유)
        else:                                        # 바깥 면 관통
            x_start = xo_local - wall_thickness - 1.0  # 캐비티 안쪽
            x_end   = xo_local + 0.5                 # 외벽 바깥쪽
        cut_x_len = x_end - x_start                  # cut prism X 길이
        cy = (y_lo + y_hi) / 2                       # 컷 Y 중심
        cz = (z_lo + z_hi) / 2                       # 컷 Z 중심
        w_y = y_hi - y_lo                            # Y 폭
        h_z = z_hi - z_lo                            # Z 높이
        # YZ 평면 (노멀 +X) 에서 둥근 사각 sketch → X 방향 extrude
        cut_prism = (
            cq.Workplane("YZ")                       # YZ 평면 (노멀 +X)
            .workplane(offset=x_start)               # X = x_start
            .moveTo(cy, cz)                          # 중심
            .rect(w_y, h_z)                          # 사각
            .extrude(cut_x_len)                      # +X 방향 prism
        )
        # 모서리 라운딩 (Y 평행 모서리 4 개 — cut prism 의 Z·Y 모서리는 ⊥X 평면 위)
        try:
            cut_prism = cut_prism.edges("|X").fillet(cutout_corner_r)  # X 평행 모서리 라운딩
        except Exception:                            # 실패 시 그대로
            pass                                     # skip
        cuts_union = cut_prism if cuts_union is None else cuts_union.union(cut_prism)  # 누적
    return cuts_union                                # 결과

cutouts_solid = build_cutouts()                      # 컷아웃 prism 들
if cutouts_solid is not None:                        # 있으면 blade 에서 cut
    blade_with_ridges = blade_with_ridges.cut(cutouts_solid)  # boolean cut

# ============================================================
# 5) 블레이드 STL export
# ============================================================
blade_path = OUT_DIR / OUT_BLADE_STL                 # 블레이드 STL 경로
cq.exporters.export(                                 # CadQuery → STL
    blade_with_ridges,                               # 블레이드 솔리드
    str(blade_path),                                 # 파일명
    tolerance=0.02,                                  # 곡선 tolerance
    angularTolerance=0.2,                            # 각도 tolerance
)
print(f"[blade] exported: {blade_path}")             # 진행 표시

# ============================================================
# 6) 원본 STL 의 모터 마운트 부분과 union  (trimesh)
# ============================================================
src_stl_path = (OUT_DIR / SRC_STL).resolve()         # 원본 STL 절대경로
mount_mesh = trimesh.load(str(src_stl_path))         # 원본 STL 로드 (미터 단위)
mount_mesh.apply_scale(STL_UNIT_SCALE)               # mm 로 환산 (×1000)

# Y >= cut_y 인 부분 잘라내기 — 평면 슬라이스
mount_cut = mount_mesh.slice_plane(                  # 평면 자르기
    plane_origin=[0, cut_y, 0],                      # 잘리는 평면 한 점
    plane_normal=[0, -1, 0],                         # 노멀 = -Y → -Y 쪽만 유지 = Y<=cut_y
    cap=True,                                        # 잘린 단면 막기
)
if mount_cut is None or mount_cut.is_empty:          # 잘린 결과 검증
    raise RuntimeError("mount 자르기 실패")          # 에러

blade_mesh = trimesh.load(str(blade_path))           # 블레이드 STL 로드

# Boolean union (trimesh manifold engine)
try:
    final_mesh = trimesh.boolean.union(              # union
        [mount_cut, blade_mesh],                     # 두 메시
        engine="manifold",                           # manifold 엔진
    )
except Exception as e:                               # 실패시 fallback
    print(f"[union] manifold 실패: {e}, concatenate fallback")  # 로그
    final_mesh = trimesh.util.concatenate([mount_cut, blade_mesh])  # 단순 합치기

final_path = OUT_DIR / OUT_FINAL_STL                 # 최종 경로
final_mesh.export(str(final_path))                   # 저장
print(f"[final] exported: {final_path}  watertight={final_mesh.is_watertight}")  # 진행 표시
print(f"        bbox: {final_mesh.bounds.tolist()}")  # bbox 확인
