// ============================================================
//  OMX_F Gripper — Long Blade Finger Extension (toothed)
//  ─────────────────────────────────────────────────────────
//  스타일: 닫힌 외피 + 그립 면 (-Y) 에서 일정 간격으로
//         바깥쪽 (-Y 방향) 으로 튀어나오는 톱니 리지
//
//  소켓  : 기존 핑거 블레이드 (x=22..65) 위에 슬라이드 끼움
//  확장부: x=65 부터 ext_length 만큼 뻗는 테이퍼 웨지
//          외피는 사방 (±Y, ±Z) 닫힌 셸
//          내측 (-Y) 면에 ridge_pitch 간격으로 톱니 돌출
//          소켓 구간도 같은 톱니 패턴 적용
//
//  show_phantom 로 실제 핑거 STL 을 cyan 반투명으로 함께 표시
// ============================================================

// ── USER PARAMETERS ─────────────────────────────────────────

// 소켓 끼움 (기존 블레이드 x=22..65)
clearance       = 0.30;   // 블레이드 - 소켓 사이 갭 (mm)
wall            = 1.8;    // 소켓 외벽 두께 (mm)
clip_lip        = 0.6;    // 외측 (+Y) 스냅 클립 깊이 (mm)

// 확장부 외형 envelope (x = 65 → 65 + ext_length)
ext_length      = 150;    // 확장부 길이 (mm)
ext_base_hy     = 5.5;    // 베이스 half-Y (mm)
ext_base_hz     = 20.0;   // 베이스 half-Z (mm)
ext_tip_hy      = 1.0;    // 팁 half-Y (mm)
ext_tip_hz      = 2.0;    // 팁 half-Z (mm)
inward_offset   = 0;      // 팁 안쪽 휨 (-Y, mm). 0 = 직선.

// 외피 두께
skin_y_thick    = 1.8;    // ±Y 면 두께 (mm)
skin_z_thick    = 1.4;    // ±Z 면 두께 (mm)
tip_cap_thick   = 1.5;    // 팁 끝 마감 (mm)
base_thick      = 5.0;    // 소켓 접합부 솔리드 (mm)

// 그립 톱니 (-Y 면 바깥쪽으로 돌출)
ridge_pitch     = 13;     // 톱니 간격 (mm, X 방향)
ridge_width     = 3.5;    // 톱니 베이스 폭 (mm, X 방향)
ridge_height    = 2.5;    // 톱니 돌출 높이 (mm, -Y 방향)
ridge_start_x   = 28;     // 톱니 시작 위치 (mm, 소켓 안쪽부터)
ridge_z_margin  = 1.5;    // 톱니가 ±Z 가장자리에서 떨어지는 거리 (mm)

// 표시
show_phantom    = true;   // 실제 핑거 STL phantom (STL export 전엔 false)
which_finger    = "both"; // "link6" | "link7" | "both"

// ── BLADE CROSS-SECTION (GRIPPER_DIMENSIONS.md) ─────────────
blade_pts = [
    [ 22, 5.7, 19.4 ],
    [ 28, 4.2, 17.7 ],
    [ 36, 3.1, 15.5 ],
    [ 46, 2.8, 12.7 ],
    [ 55, 3.5, 10.3 ],
    [ 62, 3.5,  8.3 ],
    [ 65, 3.0,  7.6 ],
];

X_SOCKET_START = 22;
X_SOCKET_END   = 65;
X_EXT_START    = X_SOCKET_END;
X_EXT_END      = X_SOCKET_END + ext_length;

// ── HELPERS ──────────────────────────────────────────────────
function lerp(a, b, t) = a + (b-a)*t;

function seg(x, i=0) =
    (i >= len(blade_pts)-2) ? i :
    (x <= blade_pts[i+1][0]) ? i : seg(x, i+1);

function bhy(x) =
    let(i = seg(x), x0 = blade_pts[i][0], x1 = blade_pts[i+1][0])
    lerp(blade_pts[i][1], blade_pts[i+1][1], (x-x0)/(x1-x0));

function bhz(x) =
    let(i = seg(x), x0 = blade_pts[i][0], x1 = blade_pts[i+1][0])
    lerp(blade_pts[i][2], blade_pts[i+1][2], (x-x0)/(x1-x0));

// 확장부 cross-section (t = 0..1)
function ext_hy(t) = lerp(ext_base_hy, ext_tip_hy, t);
function ext_hz(t) = lerp(ext_base_hz, ext_tip_hz, t);
function ext_cy(t) = -inward_offset * t;

// 임의 X 에서의 -Y 외측 면 위치와 Z 반높이 (소켓/확장부 통합)
function face_y_at(x) =
    x < X_EXT_START
        ? -(bhy(x) + wall + clearance)
        : ext_cy((x - X_EXT_START)/ext_length)
          - ext_hy((x - X_EXT_START)/ext_length);

function face_hz_at(x) =
    x < X_EXT_START
        ? bhz(x) + wall + clearance
        : ext_hz((x - X_EXT_START)/ext_length);

N_HULL = 16;

// ── SOCKET ──────────────────────────────────────────────────
module socket_outer() {
    hull()
        for (i = [0:N_HULL]) {
            x = X_SOCKET_START + i * (X_SOCKET_END - X_SOCKET_START) / N_HULL;
            hy = bhy(x) + wall + clearance;
            hz = bhz(x) + wall + clearance;
            translate([x, 0, 0])
                cube([0.01, hy*2, hz*2], center=true);
        }
}

module socket_cavity() {
    translate([-0.1, 0, 0])
    hull()
        for (i = [0:N_HULL]) {
            x = X_SOCKET_START + i * (X_SOCKET_END - X_SOCKET_START) / N_HULL;
            hy = bhy(x) + clearance;
            hz = bhz(x) + clearance;
            translate([x, 0, 0])
                cube([0.01, hy*2, hz*2], center=true);
        }
}

module socket_clip() {
    hull()
        for (i = [0:N_HULL]) {
            x = X_SOCKET_START + i * (X_SOCKET_END - X_SOCKET_START) / N_HULL;
            hy_out = bhy(x) + wall + clearance;
            hz_out = bhz(x) + wall + clearance;
            translate([x, hy_out, 0])
                cube([0.01, clip_lip*2, hz_out*1.2], center=true);
        }
}

module socket_body() {
    difference() {
        union() {
            socket_outer();
            socket_clip();
        }
        socket_cavity();
    }
}

// ── EXTENSION WEDGE ENVELOPE ────────────────────────────────
module wedge_envelope() {
    hull()
        for (i = [0:N_HULL]) {
            t = i / N_HULL;
            x = X_EXT_START + t * ext_length;
            translate([x, ext_cy(t), 0])
                cube([0.01, ext_hy(t)*2, ext_hz(t)*2], center=true);
        }
}

// Envelope 내부 캐비티 (벽두께만큼 축소된 envelope)
module wedge_inner_cavity() {
    eps = 0.01;
    hull()
        for (i = [0:N_HULL]) {
            t = i / N_HULL;
            x = X_EXT_START + base_thick + t * (ext_length - base_thick - tip_cap_thick);
            hy = max(eps, ext_hy(t) - skin_y_thick);
            hz = max(eps, ext_hz(t) - skin_z_thick);
            translate([x, ext_cy(t), 0])
                cube([0.01, hy*2, hz*2], center=true);
        }
}

// 닫힌 hollow shell: envelope 에서 내부 캐비티만 제거
module extension_shell() {
    difference() {
        wedge_envelope();
        wedge_inner_cavity();
    }
}

// ── GRIP TOOTH RIDGES (바깥쪽 -Y 방향 돌출) ──────────────────
// X=x_pos 위치에 톱니 한 개 — 닫힌 외피 위에 얹힘
// 베이스 폭 = ridge_width, 돌출 높이 = ridge_height
// Z 방향 길이 = 그 위치의 envelope full Z height - 2*margin
module grip_tooth_at(x_pos) {
    fy = face_y_at(x_pos);
    hz = face_hz_at(x_pos);
    tooth_hz = max(0.5, hz - ridge_z_margin);

    // 톱니 폴리곤 (X-Y 평면): 베이스를 면 안쪽으로 0.5mm 깊이 box 처리
    // → 닫힌 외피와 깔끔하게 union
    linear_extrude(height = 2*tooth_hz, center = true)
        polygon(points = [
            [x_pos - ridge_width/2, fy + 0.5],   // 베이스 좌 (외피 안쪽)
            [x_pos,                 fy - ridge_height],  // 피크 (외측 돌출)
            [x_pos + ridge_width/2, fy + 0.5],   // 베이스 우
        ]);
}

// 소켓 시작부터 확장부 팁까지 일정 간격으로 톱니 배치
module all_grip_teeth() {
    n = floor((X_EXT_END - tip_cap_thick - ridge_start_x) / ridge_pitch);
    for (i = [0:n]) {
        x = ridge_start_x + i * ridge_pitch;
        if (x + ridge_width/2 < X_EXT_END - tip_cap_thick + 0.5)
            grip_tooth_at(x);
    }
}

// ── EXTENSION BODY ──────────────────────────────────────────
module extension_body() {
    extension_shell();
    // 솔리드 보강부: 소켓 접합과 팁 끝
    intersection() {
        wedge_envelope();
        union() {
            translate([X_EXT_START + base_thick/2, 0, 0])
                cube([base_thick + 0.1,
                      ext_base_hy*2 + 20,
                      ext_base_hz*2 + 20], center=true);
            translate([X_EXT_END - tip_cap_thick/2, 0, 0])
                cube([tip_cap_thick + 0.1,
                      ext_base_hy*2 + 20,
                      ext_base_hz*2 + 20], center=true);
        }
    }
}

// ── COMPLETE FINGER ─────────────────────────────────────────
// link6: 내측 = -Y → 톱니가 -Y 방향으로 돌출
module finger_link6() {
    socket_body();
    extension_body();
    all_grip_teeth();
}

// link7: 내측 = +Y → Y 미러 (톱니가 +Y 방향으로 돌출됨)
module finger_link7() {
    mirror([0, 1, 0]) finger_link6();
}

// ── PHANTOM (실제 핑거 메쉬, alignment 확인용) ──────────────
STL_LINK6 = "model/omx_f_mesh/follower_07_gripper_motorized.stl";

module finger_phantom_link6() {
    color("cyan", 0.25)
        import(STL_LINK6, convexity = 10);
}

module finger_phantom_link7() {
    mirror([0, 1, 0]) finger_phantom_link6();
}

// ── RENDER ──────────────────────────────────────────────────
SEP = 60;  // "both" 모드 시각 분리 (실제 그리퍼 간격 아님)

if (which_finger == "link6") {
    finger_link6();
    if (show_phantom) finger_phantom_link6();

} else if (which_finger == "link7") {
    finger_link7();
    if (show_phantom) finger_phantom_link7();

} else {  // "both"
    translate([0,  SEP/2, 0]) {
        finger_link6();
        if (show_phantom) finger_phantom_link6();
    }
    translate([0, -SEP/2, 0]) {
        finger_link7();
        if (show_phantom) finger_phantom_link7();
    }
}

// ============================================================
//  출력 가이드
// ─────────────────────────────────────────────────────────────
//  재료     : PETG 또는 PLA (강성 필요).
//  방향     : +Y 외피 (톱니 없는 면) 를 베드에 깔고 출력
//             → 톱니 면 (-Y) 이 위로 향함, 서포트 최소화
//             OpenSCAD export 전 rotate([-90, 0, 0]) 권장
//  레이어   : 0.2 mm
//  벽       : 3 perimeters, 인필 15-25 %
//
//  톱니 튜닝
//  ───────────
//  더 촘촘하게        → ridge_pitch 줄임 (8-10 mm)
//  더 깊게/공격적으로 → ridge_height 늘림 (3-4 mm)
//  더 넓게            → ridge_width 늘림 (5 mm)
//  잡는 면 줄이려면   → ridge_start_x 늘림 (40 → 소켓 끝부터 시작)
//
//  핏 튜닝
//  ───────────
//  헐겁다       → clearance 줄임 (0.20)
//  빡빡함       → clearance 늘림 (0.40)
//  클립 안 걸림 → clip_lip 늘림 (0.9)
//
//  STL 내보내기 전
//  ───────────────────────────────────────────────────────────
//  1. show_phantom = false
//  2. which_finger = "link6" 또는 "link7" (개별 출력)
//  3. F6 렌더 → F7 STL
// ============================================================
