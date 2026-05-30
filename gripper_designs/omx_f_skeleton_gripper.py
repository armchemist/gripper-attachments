"""
OMX_F Skeleton Gripper Finger (L side).

기존 omx_f_tapered_shell_gripper 의 후속 버전.
블레이드를 hollow shell 이 아니라 "갈비뼈(skeleton)" 구조로 변경:
  - 외벽 평면 영역은 완전 관통 (반대쪽이 그대로 보임)
  - +X 방향으로 튀어나온 ridge (수직 삼각 프리즘) 들만 남김
  - ridge 들을 위(top) + 아래(bottom) + 중앙(center) 가로 spine 3 개로 연결
  - 모터 마운트 ↔ skeleton 사이 짧은 solid transition 으로 강도 확보

좌표계 (원본 STL × 1000 후 mm):
  Y = 길이축, Z = 세로, X = 두께/그립 방향 (-X 가 안쪽)
"""

import cadquery as cq                               # 메인 CAD 라이브러리
import trimesh                                      # STL boolean / IO
from pathlib import Path                            # 파일 경로

# ============================================================
# PARAMETERS  (편집해서 모양 조절)
# ============================================================

# ── 원본 STL (모터 마운트) ─────────────────────────────────
SRC_STL = "../model/omx_f_mesh/follower_gripper_l.stl"  # 원본 L 핑거 STL
STL_UNIT_SCALE = 1000.0                              # m → mm
cut_y = 22.0                                         # 이 Y 이후 잘라내고 새 블레이드 union
joint_overlap = 4.0                                  # 블레이드 베이스가 하우징으로 파고드는 깊이

# ── 베이스 단면 (하우징 envelope) ─────────────────────────
base_x_inner = -10.2                                 # 베이스 안쪽 X
base_x_outer =   2.5                                 # 베이스 바깥 X
base_z_bottom = -36.0                                # 베이스 바닥 Z
base_z_top    =   4.0                                # 베이스 윗변 Z

# ── 팁 단면 (X·Z 모두 축소) ──────────────────────────────
blade_length    = 100.0                              # 블레이드 길이 (Y)
tip_x_inner     = -10.2                              # 팁 안쪽 X (베이스와 동일 → -X 면 평면)
tip_x_outer     =  -4.0                              # 팁 바깥 X
tip_z_bottom    = -36.0                              # 팁 바닥 Z (베이스와 동일 → -Z 면 평면)
tip_z_top       = -26.0                              # 팁 윗변 Z

# ── 솔리드 transition (모터→skeleton 사이) ───────────────
solid_base_len  = 10.0                               # 베이스 쪽 솔리드 구간 Y 길이 (mm) — 강도용
solid_tip_len   =  6.0                               # 팁 쪽 솔리드 캡 길이 (mm) — 끝 마무리

# ── 가로 spine (3 개: top / middle / bottom) ─────────────
spine_top_z_thick    = 3.0                           # top spine 의 Z 두께 (mm)
spine_bottom_z_thick = 3.0                           # bottom spine 의 Z 두께 (mm)
spine_middle_z_thick = 3.0                           # 중앙 spine Z 두께 (mm) — "가운데 선 하나"
spine_x_inset        = 0.0                           # spine X 안쪽 인셋 (0 = 외벽까지 풀)

# ── 세로 ridge (튀어나온 삼각 프리즘) ────────────────────
num_ridges       = 11                                # ridge 개수 — skeleton 가로 칸 수 = N-1
ridge_width      = 3.5                               # ridge 베이스 Y 폭 (mm)
ridge_height     = 1.8                               # +X 방향 추가 돌출 높이 (mm)
ridge_face       = "+X"                              # 돌출 방향 ("+X" or "-X")

# ── 출력 ─────────────────────────────────────────────────
OUT_BLADE_STL = "omx_f_skeleton_blade_only.stl"      # 블레이드만 (디버그)
OUT_FINAL_STL = "omx_f_skeleton_gripper_L.stl"       # 최종 union
OUT_DIR = Path(__file__).parent                      # 출력 폴더

# ============================================================
# 유도 변수 / 단면 보간기
# ============================================================
y_base = cut_y - joint_overlap                       # 블레이드 실제 시작 Y
y_tip  = cut_y + blade_length                        # 블레이드 끝 Y
blade_total_y = y_tip - y_base                       # 블레이드 총 Y 길이

def cross_section_at(t):                             # t∈[0,1] 외곽 단면 (xi, xo, zb, zt)
    xi = base_x_inner  + (tip_x_inner  - base_x_inner)  * t  # 안쪽 X
    xo = base_x_outer  + (tip_x_outer  - base_x_outer)  * t  # 바깥 X
    zb = base_z_bottom + (tip_z_bottom - base_z_bottom) * t  # 바닥 Z
    zt = base_z_top    + (tip_z_top    - base_z_top)    * t  # 윗 Z
    return xi, xo, zb, zt                            # 반환

def section_at_y(y):                                 # 임의 Y 에서 외곽 단면
    t = (y - y_base) / blade_total_y                 # t 계산
    return cross_section_at(t)                       # 단면

# ============================================================
# 1) 외곽 솔리드 구간 — base 와 tip 양 끝의 짧은 솔리드 캡
# ============================================================
def make_loft_segment(y0, y1):                       # y0..y1 사이 사각 단면 loft
    xi0, xo0, zb0, zt0 = section_at_y(y0)            # y0 단면
    xi1, xo1, zb1, zt1 = section_at_y(y1)            # y1 단면
    return (
        cq.Workplane("XZ")                           # XZ 평면 (노멀 -Y)
        .workplane(offset=-y0)                       # offset 부호 반전 → Y=y0
        .polyline([(xi0, zb0), (xo0, zb0), (xo0, zt0), (xi0, zt0)])  # 베이스 사각
        .close()                                     # 닫기
        .workplane(offset=-(y1 - y0))                # Y=y1 평면
        .polyline([(xi1, zb1), (xo1, zb1), (xo1, zt1), (xi1, zt1)])  # 팁 사각
        .close()                                     # 닫기
        .loft(combine=True, ruled=True)              # ruled loft
    )

y_skel_start = y_base + solid_base_len               # skeleton 구간 시작
y_skel_end   = y_tip  - solid_tip_len                # skeleton 구간 끝

base_solid = make_loft_segment(y_base, y_skel_start) # 베이스 솔리드 wedge
tip_solid  = make_loft_segment(y_skel_end, y_tip)    # 팁 솔리드 캡

# ============================================================
# 2) 가로 SPINE 3 개 (top / middle / bottom) — y_skel_start..y_skel_end
# ============================================================
# 각 spine = Y 길이방향 빔. XZ 단면은 그 Y 의 외곽 단면을 따라 인터폴레이션 (외벽 평면 유지).
# top spine: Z 범위 = [z_top - thick, z_top]
# bottom spine: Z 범위 = [z_bot, z_bot + thick]
# middle spine: Z 범위 = [mid - thick/2, mid + thick/2]
# X 범위: [xi + inset, xo - inset] (외벽까지 풀로)

def spine_loft(y0, y1, kind):                        # kind: "top" / "mid" / "bot"
    def sec(y):                                      # 그 Y 의 spine 단면 4 꼭짓점
        xi, xo, zb, zt = section_at_y(y)             # 외곽 단면
        xl = xi + spine_x_inset                      # spine X 좌측
        xr = xo - spine_x_inset                      # spine X 우측
        if kind == "top":                            # 윗 spine
            zL = zt - spine_top_z_thick              # 아래
            zH = zt                                  # 위
        elif kind == "bot":                          # 아래 spine
            zL = zb                                  # 바닥
            zH = zb + spine_bottom_z_thick           # 위
        else:                                        # 중앙 spine
            zc = (zb + zt) / 2.0                     # 중앙 Z
            zL = zc - spine_middle_z_thick / 2.0     # 아래
            zH = zc + spine_middle_z_thick / 2.0     # 위
        return xl, xr, zL, zH                        # 4 값
    xl0, xr0, zL0, zH0 = sec(y0)                     # y0 spine 단면
    xl1, xr1, zL1, zH1 = sec(y1)                     # y1 spine 단면
    return (
        cq.Workplane("XZ")                           # XZ 평면
        .workplane(offset=-y0)                       # Y=y0
        .polyline([(xl0, zL0), (xr0, zL0), (xr0, zH0), (xl0, zH0)])
        .close()                                     # 닫기
        .workplane(offset=-(y1 - y0))                # Y=y1
        .polyline([(xl1, zL1), (xr1, zL1), (xr1, zH1), (xl1, zH1)])
        .close()                                     # 닫기
        .loft(combine=True, ruled=True)              # ruled loft
    )

spine_top    = spine_loft(y_skel_start, y_skel_end, "top")     # 윗 spine
spine_bot    = spine_loft(y_skel_start, y_skel_end, "bot")     # 아래 spine
spine_middle = spine_loft(y_skel_start, y_skel_end, "mid")     # 중앙 보강 spine ← "가운데 선"

# ============================================================
# 3) 세로 RIDGE 들 (튀어나온 삼각 프리즘) — skeleton 구간에 균등 분포
# ============================================================
# 각 ridge = XY 평면 (노멀 +Z) 에서 삼각형 단면 → Z 로 extrude.
# 삼각형 단면: base 2 점 (외벽 위, ±width/2 Y), peak 1 점 (외벽 + ridge_height 만큼 더 +X).
# Z 길이 = 그 Y 의 외곽 Z 높이 (top spine + bottom spine 잇기 위해 풀 길이로).

def build_ridges():                                  # ridge solid 누적
    if num_ridges < 1:                               # 0 개면 None
        return None                                  # 종료
    if num_ridges == 1:                              # 1 개면 중앙
        ys = [(y_skel_start + y_skel_end) / 2.0]     # 중앙 Y
    else:                                            # ≥2 개 균등
        step = (y_skel_end - y_skel_start) / (num_ridges - 1)  # 간격
        ys = [y_skel_start + i*step for i in range(num_ridges)]  # 위치 리스트
    ridges = None                                    # 누적
    for yy in ys:                                    # 각 ridge Y
        xi, xo, zb, zt = section_at_y(yy)            # 그 Y 의 외곽 단면
        # 돌출 면 선택
        if ridge_face == "+X":                       # 바깥 돌출
            base_x = xo                              # 베이스 X (외벽)
            peak_x = xo + ridge_height               # peak X
        else:                                        # 안쪽 돌출
            base_x = xi                              # 안쪽 외벽
            peak_x = xi - ridge_height               # peak (-X)
        # 삼각형 단면 (XY 평면, Z 로 extrude)
        ridge = (
            cq.Workplane("XY")                       # XY 평면 (노멀 +Z)
            .polyline([                              # 삼각형 3 점
                (base_x, yy - ridge_width/2),        # base 1
                (base_x, yy + ridge_width/2),        # base 2
                (peak_x, yy),                        # peak
            ])
            .close()                                 # 닫기
            .extrude(zt - zb)                        # 외곽 Z 높이만큼 extrude
            .translate((0, 0, zb))                   # Z=zb 로 이동
        )
        ridges = ridge if ridges is None else ridges.union(ridge)  # 누적
    return ridges                                    # 결과

ridges_solid = build_ridges()                        # ridge 들

# ============================================================
# 4) 추가: 각 ridge 의 본체 (외벽 평면 → 돌출 peak 사이) — 사다리 가로대 효과
# ============================================================
# ridge 만으로는 외벽 평면 영역이 완전히 비어 있어서 한쪽 spine 끼리 살짝 분리 가능.
# 각 ridge 의 base 가 외벽 사각 (xi..xo) 까지 채우도록 작은 직사각 슬랩을 union.
def build_ridge_webs():                              # ridge 위치마다 얇은 web (Y×X 사각, Z 풀)
    if num_ridges < 1:                               # 없음
        return None                                  # 종료
    if num_ridges == 1:                              # 1 개
        ys = [(y_skel_start + y_skel_end) / 2.0]     # 중앙
    else:                                            # ≥2
        step = (y_skel_end - y_skel_start) / (num_ridges - 1)
        ys = [y_skel_start + i*step for i in range(num_ridges)]
    webs = None                                      # 누적
    for yy in ys:                                    # 각 위치
        xi, xo, zb, zt = section_at_y(yy)            # 외곽 단면
        # 사각 슬랩: X=xi..xo, Y=yy±width/2, Z=zb..zt — 외벽 평면을 ridge 폭만큼만 채움
        w_x = xo - xi                                # X 폭
        cx  = (xi + xo) / 2                          # X 중심
        web = (
            cq.Workplane("XY")                       # XY 평면
            .box(w_x, ridge_width, zt - zb,          # X·Y·Z 박스
                 centered=(True, True, False))      # 바닥 Z=0 정렬
            .translate((cx, yy, zb))                 # 위치 이동
        )
        webs = web if webs is None else webs.union(web)  # 누적
    return webs                                      # 결과

ridge_webs = build_ridge_webs()                      # ridge web 들

# ============================================================
# 5) Skeleton 합치기  (base solid + 3 spines + ridges + webs + tip solid)
# ============================================================
skeleton = base_solid                                # 시작: 베이스 솔리드
skeleton = skeleton.union(spine_top)                 # + top spine
skeleton = skeleton.union(spine_bot)                 # + bottom spine
skeleton = skeleton.union(spine_middle)              # + 중앙 spine (보강선)
if ridge_webs is not None:                           # ridge web (외벽 평면 + 돌출 받침)
    skeleton = skeleton.union(ridge_webs)            # union
if ridges_solid is not None:                         # ridge (튀어나온 삼각)
    skeleton = skeleton.union(ridges_solid)          # union
skeleton = skeleton.union(tip_solid)                 # + 팁 솔리드 캡

# ============================================================
# 6) STL export (blade only)
# ============================================================
blade_path = OUT_DIR / OUT_BLADE_STL                 # 블레이드 STL 경로
cq.exporters.export(                                 # CadQuery → STL
    skeleton,                                        # 솔리드
    str(blade_path),                                 # 파일명
    tolerance=0.02,                                  # tol
    angularTolerance=0.2,                            # ang tol
)
print(f"[blade] exported: {blade_path}")             # 진행

# ============================================================
# 7) 원본 모터 마운트와 trimesh union
# ============================================================
src_stl_path = (OUT_DIR / SRC_STL).resolve()         # 원본 STL 경로
mount_mesh = trimesh.load(str(src_stl_path))         # 로드 (미터)
mount_mesh.apply_scale(STL_UNIT_SCALE)               # mm 환산

mount_cut = mount_mesh.slice_plane(                  # 평면 자르기
    plane_origin=[0, cut_y, 0],                      # Y=cut_y
    plane_normal=[0, -1, 0],                         # -Y → Y<=cut_y 유지
    cap=True,                                        # 단면 캡
)
if mount_cut is None or mount_cut.is_empty:          # 검증
    raise RuntimeError("mount 자르기 실패")          # 에러

blade_mesh = trimesh.load(str(blade_path))           # 블레이드 STL 로드

try:                                                 # manifold union
    final_mesh = trimesh.boolean.union(              # boolean union
        [mount_cut, blade_mesh],                     # 두 메시
        engine="manifold",                           # 엔진
    )
except Exception as e:                               # 실패 fallback
    print(f"[union] manifold 실패: {e}, concat fallback")
    final_mesh = trimesh.util.concatenate([mount_cut, blade_mesh])

final_path = OUT_DIR / OUT_FINAL_STL                 # 최종 경로
final_mesh.export(str(final_path))                   # 저장
print(f"[final] exported: {final_path}  watertight={final_mesh.is_watertight}")
print(f"        bbox: {final_mesh.bounds.tolist()}")  # bbox
print(f"        volume mm^3: {final_mesh.volume:.1f}")  # 부피
