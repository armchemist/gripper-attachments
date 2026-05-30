"""
OMX_F Tapered Hollow-Shell Gripper Finger (R side, 이등변 측면).

L 측 블레이드 (omx_f_tapered_shell_iso_blade_only.stl) 를 그대로 활용:
  - L frame: length=+Y, inner=-X
  - R frame: length=+X, inner=-Y
  - 변환: (x, y, z) → (y, x, z)   (= -90° around Z + Y mirror)
  - 결과: L's +Y 길이축 → R's +X, L's -X 안쪽면 → R's -Y 안쪽면

그 후 R 모터 마운트 STL (follower_gripper_r.stl) 를 X<=cut_x 영역만 남기고
변환된 블레이드와 trimesh boolean union 으로 합침.
"""

import trimesh                                       # 메시 IO / boolean
import numpy as np                                   # 행렬 변환
from pathlib import Path                             # 경로 처리

# ============================================================
# PARAMETERS
# ============================================================
HERE = Path(__file__).parent                         # 현재 폴더
SRC_R_MOUNT = HERE / "../model/omx_f_mesh/follower_gripper_r.stl"  # R 마운트 원본
SRC_L_BLADE = HERE / "omx_f_tapered_shell_iso_blade_only.stl"      # L 블레이드 STL (재사용)
OUT_STL     = HERE / "omx_f_tapered_shell_iso_gripper_R.stl"       # 최종 R 그리퍼

STL_UNIT_SCALE = 1000.0                              # m → mm
cut_x = 22.0                                         # R 마운트 자르는 X 위치 (mm, L 의 cut_y 와 동일 값)

# ============================================================
# 1) R 마운트 로드 + 자르기
# ============================================================
mount = trimesh.load(str(SRC_R_MOUNT.resolve()))     # R STL 로드 (m 단위)
mount.apply_scale(STL_UNIT_SCALE)                    # mm 환산
mount_cut = mount.slice_plane(                       # 평면 자르기
    plane_origin=[cut_x, 0, 0],                      # 평면 한 점
    plane_normal=[-1, 0, 0],                         # 노멀 -X → -X 쪽만 유지 = X<=cut_x
    cap=True,                                        # 단면 캡
)
if mount_cut is None or mount_cut.is_empty:          # 검증
    raise RuntimeError("R mount 자르기 실패")        # 에러
print(f"[mount-R] cut bbox: {mount_cut.bounds.tolist()}")  # 진행

# ============================================================
# 2) L 블레이드 로드 + 변환 (L frame → R frame)
# ============================================================
blade = trimesh.load(str(SRC_L_BLADE.resolve()))     # L 블레이드 mm 단위 (이미 mm)
# 변환 행렬: (x, y, z) → (y, x, z)
#   - L 의 +Y 길이축 → R 의 +X
#   - L 의 -X 안쪽 면 → R 의 -Y 안쪽 면 (R STL 의 inner 방향과 일치)
T = np.array([                                       # 4x4 동차좌표 변환
    [0, 1, 0, 0],                                    # X_new = Y_old
    [1, 0, 0, 0],                                    # Y_new = X_old
    [0, 0, 1, 0],                                    # Z 유지
    [0, 0, 0, 1],                                    # 동차
])
blade.apply_transform(T)                             # 변환 적용
# determinant = -1 (반사 포함) → 면 normal 방향 뒤집힘 → 수정
blade.invert()                                       # 면 winding 반전 → 노멀 정상화
blade.process(validate=True)                         # 메시 정리
print(f"[blade-R] transformed bbox: {blade.bounds.tolist()}")  # 변환 결과 확인

# ============================================================
# 3) Mount + Blade union
# ============================================================
try:
    final = trimesh.boolean.union(                   # manifold union
        [mount_cut, blade],                          # 두 메시
        engine="manifold",                           # 엔진
    )
except Exception as e:                               # 실패 fallback
    print(f"[union] manifold 실패: {e}, concat fallback")
    final = trimesh.util.concatenate([mount_cut, blade])

final.export(str(OUT_STL))                           # 저장
print(f"[final] exported: {OUT_STL}")                # 진행
print(f"        watertight={final.is_watertight}  volume={final.volume:.1f} mm^3")
print(f"        bbox: {final.bounds.tolist()}")      # bbox
