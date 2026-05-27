# OMX-F 그리퍼 어태치먼트

OpenMANIPULATOR-X 팔로워 암(OMX-F) 그리퍼용 3D 프린팅 어태치먼트 모음.

## 파일 구성

```
gripper-attachments/
├── GRIPPER_DIMENSIONS.md       그리퍼 핑거 치수 측정값 — 모델링 전 반드시 읽기
├── analyze_gripper.py          STL 단면 분석 스크립트 (치수 재측정 시 사용)
├── gripper_sleeve.scad         범용 파라메트릭 슬리브
├── omx_f_gripper_sleeve.scad   OMX-F 블레이드 단면에 맞춘 슬리브
├── omx_f_tube_insert.scad      시험관 파지용 팁 인서트
├── omx_f_tube_insert2~5.scad   튜브 인서트 반복 개선 버전들
└── model/
    └── omx_f_mesh/
        └── follower_07_gripper_motorized.stl   원본 link6 핑거 메시
```

## 새 어태치먼트 설계하기

먼저 `GRIPPER_DIMENSIONS.md`를 열어 블레이드 단면 테이블, 피벗 위치, 내/외면 방향 기준을 확인한다. STL을 직접 분석하는 대신 여기 기록된 수치를 그대로 사용한다.

설계 시 기준이 되는 주요 치수:

| 구역 | X 범위 | 비고 |
|---|---|---|
| 하우징 블록 | 0 – 22 mm | 덮지 말 것 (모터/기어 본체) |
| 그립 블레이드 | 22 – 65 mm | 어태치먼트 부착 영역 |
| 팁 | 65 mm | 블레이드 끝단 |

벽 두께 `1.8 mm` + 클리어런스 `0.3 mm`가 블레이드 테이퍼 구간 FDM 출력의 기본 시작값이다.

## OpenSCAD에서 원본 그리퍼와 끼움새 확인하기

모든 `.scad` 파일에는 `show_phantom` 파라미터가 있다. `true`로 설정하면 핑거 블레이드의 반투명 헐(hull)이 어태치먼트와 함께 렌더링되어 단면 끼움새를 빠르게 확인할 수 있다.

실제 원본 메시로 정확한 끼움새를 확인하려면 `.scad` 파일 맨 아래에 `%import()`를 추가한다:

```scad
// STL 내보내기 전에 반드시 이 줄 삭제할 것
%import("model/omx_f_mesh/follower_07_gripper_motorized.stl");
```

`%` 수식자는 프리뷰(F5)에서 STL을 투명한 고스트로 표시하되, 최종 렌더(F6)와 내보내기(F7)에서는 제외된다. 출력 파일에 원본 메시가 포함될 걱정 없이 끼움새만 확인할 수 있다.

## 출력 워크플로우

1. OpenSCAD에서 대상 `.scad` 파일 열기
2. `show_phantom = true` 설정 (또는 `%import(...)` 추가) 후 **F5**로 끼움새 확인
3. STL 내보내기 전 `show_phantom = false` 로 되돌리고 `%import` 줄 제거
4. **F6**으로 렌더링 → **F7**로 STL 내보내기
5. 출력 방향: X축을 프린팅 베드 길이 방향으로 눕혀서 출력
6. 권장 재료: PETG 또는 TPU 95A, 벽 3–4겹, 인필 20%
