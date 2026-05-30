// ============================================================
//  OMX_F Flat Tube Gripper — Direct Motor-Mount Replacement
//  ─────────────────────────────────────────────────────────
//  follower_gripper_l.stl 의 모터 마운트/하우징부 (y = 0..cut_y) 만 그대로
//  살리고, 그 너머의 블레이드를 평평한 긴 핑거로 교체.
//  안쪽 면 (-X) 에 세로 리지를 일정 간격 배치 → 15 mL 팔콘 (Φ~17 mm)
//  같은 시험관을 미끄러짐 없이 잡도록 설계.
//
//  좌표계 (L STL 원본 그대로):
//    Y = 핑거 길이축 (모터에서 바깥)
//    Z = 세로 높이
//    X = 두께 / 그립 방향 (-X 가 짝 핑거 방향 = 안쪽)
//
//  L 버전: which_side = "L"
//  R 버전: which_side = "R"  → X 미러 (짝 핑거)
// ============================================================

// ── USER PARAMETERS ─────────────────────────────────────────

// 어디서 원본을 자를지 (mm, Y 축). 22 = 하우징 끝.
// 더 크게 = 원본 블레이드 더 많이 남김 (보강)
// 더 작게 = 더 빨리 새 블레이드로 전환
cut_y           = 23.0;

// 새 블레이드 (이등변 삼각형 측면 프로파일 — 베이스 wide, 팁 sharp point)
// 두께(X) 는 일정, 높이(Z) 만 베이스→팁 으로 균등 수축 (상하 동시 좁힘)
blade_length    = 110.0;   // cut_y 부터 새 블레이드 끝까지 길이 (mm)
blade_thick     = 5.0;     // X 두께 (mm) — 베이스/팁 일정
blade_base_height = 38.0;  // 베이스 Z 세로 높이 (mm) — 하우징 크기와 비슷
blade_tip_height  = 4.0;   // 팁 Z 세로 높이 (mm) — 뾰족
blade_x_center  = 4.0;     // 블레이드 중심 X 좌표 (원본 블레이드 위치와 일치)
blade_z_center  = -16.0;   // 블레이드 중심 Z 좌표 (원본 블레이드 중심)
blade_corner_r  = 1.0;     // 블레이드 코너 라운딩 (mm)

// 베이스 접합 보강 — 모터 하우징과 블레이드 사이를 두꺼운 넥(neck)으로 연결
// 넥 베이스(하우징 쪽)는 하우징 외형 (X=-10..10, Z=-36..4) 을 그대로 덮어
// 단차 없이 매끈하게 이어지도록 한다.
joint_length_y  = 18.0;    // 접합 넥 Y 길이 (mm)
joint_overlap_y = 4.0;     // 넥이 하우징 안으로 파고드는 깊이 (mm) — 강한 union
joint_base_xmin = -10.0;   // 넥 베이스 X 최소 (하우징 외형과 일치)
joint_base_xmax =  10.0;   // 넥 베이스 X 최대
joint_base_zmin = -36.0;   // 넥 베이스 Z 최소 (하우징 외형과 일치)
joint_base_zmax =   4.0;   // 넥 베이스 Z 최대

// 그립 리지 (안쪽 면 = -X 방향 돌출)
ridge_pitch     = 9.0;     // 리지 Y 방향 간격 (mm)
ridge_width     = 3.0;     // 리지 베이스 Y 폭 (mm)
ridge_height    = 2.0;     // 리지 -X 방향 돌출 깊이 (mm)
ridge_z_margin  = 2.0;     // 리지가 블레이드 Z 가장자리에서 떨어지는 거리 (mm)
ridge_start_y   = 45.0;    // 첫 리지 Y 위치 (mm) — 두꺼운 넥 끝나는 지점부터

// 출력 / 미리보기
which_side      = "L";     // "L" | "R" | "both"
show_phantom    = true;    // 원본 STL 반투명 표시 (STL export 전엔 false)

// ── DERIVED ─────────────────────────────────────────────────
STL_L = "../model/omx_f_mesh/follower_gripper_l.stl";
STL_R = "../model/omx_f_mesh/follower_gripper_r.stl";

// 원본 STL 은 미터 단위. OpenSCAD 에서 mm 로 다루려면 1000 배 스케일.
STL_SCALE = 1000;

EPS = 0.01;
$fn = 32;

// Y 좌표
// 넥은 하우징 안쪽으로 joint_overlap_y 만큼 파고들어 union 강도 확보
joint_y_start = cut_y - joint_overlap_y;
joint_y_end   = cut_y + joint_length_y;
blade_y_start = joint_y_end;
blade_y_end   = blade_y_start + blade_length;

// 임의 Y 에서의 블레이드 단면 (linear interp, t=0 at base, t=1 at tip)
function blade_t(y) = max(0, min(1, (y - blade_y_start)/blade_length));
function blade_height_at(y) = blade_base_height + (blade_tip_height - blade_base_height)*blade_t(y);

// 두께는 일정 — 안쪽 면 X 좌표 고정
blade_inner_x = blade_x_center - blade_thick/2;
blade_outer_x = blade_x_center + blade_thick/2;

// ── MODULES ─────────────────────────────────────────────────

// 원본 STL 에서 y >= cut_y 부분을 잘라낸 마운트
module mount_l() {
    difference() {
        scale([STL_SCALE, STL_SCALE, STL_SCALE])
            import(STL_L, convexity = 10);
        // 큰 박스로 y >= cut_y 영역 모두 제거
        translate([-100, cut_y, -100])
            cube([200, 200, 200]);
    }
}

// 테이퍼 블레이드 — 이등변 형태로 베이스 → 팁 hull
// 안쪽 (-X) / 바깥 (+X) / 위 (+Z) / 아래 (-Z) 모두 중심으로 균등 수축
module tapered_blade() {
    r = blade_corner_r;
    // 베이스 (Y = blade_y_start): X 두께 = blade_thick, Z = blade_base_height
    base_z_min = blade_z_center - blade_base_height/2;
    base_z_max = blade_z_center + blade_base_height/2;
    // 팁 (Y = blade_y_end): X 두께 동일, Z = blade_tip_height (이등변 상하 대칭 수축)
    tip_z_min  = blade_z_center - blade_tip_height/2;
    tip_z_max  = blade_z_center + blade_tip_height/2;

    hull() {
        for (x = [blade_inner_x + r, blade_outer_x - r])
        for (z = [base_z_min + r, base_z_max - r])
            translate([x, blade_y_start, z]) sphere(r = r);
        for (x = [blade_inner_x + r, blade_outer_x - r])
        for (z = [tip_z_min + r, tip_z_max - r])
            translate([x, blade_y_end, z]) sphere(r = r);
    }
}

// 두꺼운 넥 — 하우징과 블레이드 사이 굵은 연결부
// 하우징 쪽 단면 (Y = cut_y) 은 하우징 크기에 맞춰 큼직하게,
// 블레이드 베이스 쪽 단면 (Y = joint_y_end) 은 블레이드 베이스와 일치
module thick_neck() {
    r = blade_corner_r;
    // 하우징 쪽 단면 (Y = joint_y_start) — 하우징 외형 그대로 (-10..10, -36..4)
    // 블레이드 베이스 쪽 단면 (Y = joint_y_end) — 블레이드 베이스와 매칭
    base_x_min = blade_inner_x;
    base_x_max = blade_outer_x;
    base_z_min = blade_z_center - blade_base_height/2;
    base_z_max = blade_z_center + blade_base_height/2;

    hull() {
        // 하우징 매칭 단면 (Y=joint_y_start, 하우징 내부로 살짝 파고듬)
        for (x = [joint_base_xmin + r, joint_base_xmax - r])
        for (z = [joint_base_zmin + r, joint_base_zmax - r])
            translate([x, joint_y_start, z]) sphere(r = r);
        // 블레이드 베이스 단면 (Y=joint_y_end)
        for (x = [base_x_min + r, base_x_max - r])
        for (z = [base_z_min + r, base_z_max - r])
            translate([x, joint_y_end, z]) sphere(r = r);
    }
}

// 단일 리지 (Y = y_pos 위치, 안쪽 -X 면에서 돌출)
// 안쪽 면이 이등변 테이퍼라 Y 마다 위치/높이 달라짐 → 함수로 계산
module ridge_at(y_pos) {
    local_h = blade_height_at(y_pos);
    tooth_h = max(2.0, local_h - 2*ridge_z_margin);
    base_x  = blade_inner_x + 0.4;
    peak_x  = blade_inner_x - ridge_height;
    translate([0, y_pos, blade_z_center])
    linear_extrude(height = tooth_h, center = true)
        polygon([
            [base_x, -ridge_width/2],
            [peak_x, 0],
            [base_x,  ridge_width/2],
        ]);
}

module all_ridges() {
    n = floor((blade_y_end - blade_corner_r - ridge_start_y) / ridge_pitch);
    for (i = [0 : n]) {
        y = ridge_start_y + i * ridge_pitch;
        if (y + ridge_width/2 < blade_y_end - blade_corner_r)
            ridge_at(y);
    }
}

// 완성된 L 핑거 = 잘라낸 마운트 + 두꺼운 넥 + 테이퍼 블레이드 + 리지
module finger_l() {
    union() {
        mount_l();
        thick_neck();
        tapered_blade();
        all_ridges();
    }
}

// R = L 의 X-미러 (그립 방향이 +X 가 됨)
module finger_r() {
    mirror([1, 0, 0]) finger_l();
}

// 원본 STL 팬텀 (정렬 확인)
module phantom_l() { color("cyan", 0.25) scale([STL_SCALE, STL_SCALE, STL_SCALE]) import(STL_L, convexity = 10); }
module phantom_r() { color("cyan", 0.25) scale([STL_SCALE, STL_SCALE, STL_SCALE]) import(STL_R, convexity = 10); }

// ── RENDER ──────────────────────────────────────────────────
SEP_X = 50;  // both 모드 시각 분리

if (which_side == "L") {
    finger_l();
    if (show_phantom) phantom_l();
} else if (which_side == "R") {
    finger_r();
    if (show_phantom) phantom_r();
} else {  // both
    translate([-SEP_X/2, 0, 0]) {
        finger_l();
        if (show_phantom) phantom_l();
    }
    translate([ SEP_X/2, 0, 0]) {
        finger_r();
        if (show_phantom) phantom_r();
    }
}

// ============================================================
//  출력 가이드
//  ─────────────────────────────────────────────────────────
//  재료     : PETG 권장 (강성 + 약간의 탄성). PLA 가능.
//  방향     : 블레이드 평평한 바깥면 (+X 면) 을 베드에 깔고 출력
//             → 리지가 위로, 서포트 없이 모든 면 클린
//             OpenSCAD 에서 export 전 rotate([0, -90, 0]) 권장
//  레이어   : 0.2 mm
//  벽       : 3-4 perimeters, 인필 20-25 %
//
//  STL 내보내기 전
//  ─────────────────────────────────────────────────────────
//  1. show_phantom = false
//  2. which_side  = "L" 또는 "R"
//  3. F6 렌더 (원본 STL 부울 때문에 수십 초 걸릴 수 있음)
//  4. F7 STL export
// ============================================================
