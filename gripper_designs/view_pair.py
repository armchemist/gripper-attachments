"""두 그리퍼 STL을 함께 시각화하여 맞물림 확인."""
import trimesh
import numpy as np
from pathlib import Path

HERE = Path(__file__).parent

L = trimesh.load(str((HERE / "omx_f_tapered_shell_iso_gripper_L.stl").resolve()))
R = trimesh.load(str((HERE / "omx_f_tapered_shell_iso_gripper_R.stl").resolve()))

print(f"L bbox: {L.bounds.tolist()}")
print(f"R bbox: {R.bounds.tolist()}")

# 두 메시를 다른 색으로 표시
L.visual.face_colors = [100, 180, 255, 200]   # 파란색
R.visual.face_colors = [255, 140,  80, 200]   # 주황색

scene = trimesh.Scene([L, R])
scene.show()
