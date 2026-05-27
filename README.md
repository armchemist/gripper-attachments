# OMX-F 그리퍼 어태치먼트

OMX-F 로봇 팔 그리퍼에 끼워 쓰는 3D 프린팅 부품들 모음이다.

## 파일 구성

```
gripper-attachments/
├── GRIPPER_DIMENSIONS.md       핑거 치수 측정값 정리본
├── analyze_gripper.py          STL에서 단면 치수 뽑는 스크립트
├── gripper_sleeve.scad         범용 슬리브 (파라미터 조절용)
├── omx_f_gripper_sleeve.scad   OMX-F 핑거에 맞게 튜닝된 슬리브
├── omx_f_tube_insert.scad      시험관 잡는 팁 인서트
├── omx_f_tube_insert2~5.scad   튜브 인서트 개선 버전들
└── model/
    └── omx_f_mesh/
        └── follower_07_gripper_motorized.stl   원본 그리퍼 핑거 STL
```

## 새 부품 설계할 때

STL 파일 직접 열어서 치수 잴 필요 없이, `GRIPPER_DIMENSIONS.md`에 블레이드 단면 치수, 피벗 위치, 방향 기준이 다 정리되어 있다. 새 어태치먼트 만들 때는 여기 수치부터 보면 된다.

설계 전에 알아두면 좋은 것:

- **X 0~22 mm** 구간은 모터/기어 하우징이라 건드리면 안 된다
- **X 22~65 mm** 블레이드 구간이 부품 붙이는 영역
- 벽 두께 1.8 mm, 클리어런스 0.3 mm 정도가 FDM 출력 기준점

## OpenSCAD에서 끼움새 확인하는 법

`.scad` 파일 열면 위쪽에 `show_phantom` 파라미터가 있는데, `true`로 바꾸면 핑거 블레이드 형태가 반투명하게 같이 뜬다. 부품이 어디에 걸리는지 대충 보기엔 충분하다.

더 정확하게 원본 메시 위에 얹어서 보고 싶으면 파일 맨 아래에 이거 한 줄 추가하면 된다:

```scad
// 내보내기 전에 꼭 지울 것
%import("model/omx_f_mesh/follower_07_gripper_motorized.stl");
```

`%`를 붙이면 F5 프리뷰에서는 반투명 고스트로 보이고, F6 렌더링이나 STL 내보내기에는 포함이 안 된다. 출력 파일 걱정 없이 끼움새만 눈으로 확인할 수 있다.

## 출력 워크플로우

1. OpenSCAD에서 `.scad` 파일 열기
2. `show_phantom = true` 또는 `%import(...)` 추가하고 **F5**로 끼움새 확인
3. STL 내보낼 때는 다시 `false`로 돌리고 `%import` 줄 지우기
4. **F6** 렌더 → **F7** STL 내보내기
5. 출력 방향: X축이 베드 길이 방향이 되도록 눕혀서
6. 재료: PETG나 TPU 95A 추천, 벽 3~4겹, 인필 20%
