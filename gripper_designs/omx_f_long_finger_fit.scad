// ============================================================
//  OMX_F Gripper — Finger Grip Attachment
//  소켓(x=22..65) 슬라이드 끼움 + 바깥 돌출 리지
// ============================================================

// ── USER PARAMETERS ─────────────────────────────────────────

// 소켓 피팅
clearance  = 0.30;  // 블레이드-소켓 갭 (mm) — 빡빡하면 0.40
wall       = 1.8;   // 소켓 외벽 두께 (mm)
clip_lip   = 0.6;   // +Y 스냅 클립 깊이 (mm)

// 팁 연장 (블레이드 끝 x=65 이후 솔리드 캡)
tip_ext    = 10;    // 팁 돌출 길이 (mm) — 0 이면 소켓만

// 바깥 돌출 리지 (그립 면 -Y 바깥쪽)
ridge_pitch  = 10;  // 리지 간격 (mm)
ridge_width  = 3.0; // 리지 베이스 폭 (mm)
ridge_height = 2.0; // 리지 돌출 높이 (mm)
ridge_z_gap  = 2.0; // ±Z 끝단 여백 (mm)

// 표시
show_phantom = false;
which_finger = "both"; // "link6" | "link7" | "both"

// ── BLADE CROSS-SECTION ──────────────────────────────────────
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
X_TIP_END      = X_SOCKET_END + tip_ext;

// ── HELPERS ──────────────────────────────────────────────────
function lerp(a, b, t) = a + (b-a)*t;

function seg(x, i=0) =
    (i >= len(blade_pts)-2) ? i :
    (x <= blade_pts[i+1][0]) ? i : seg(x, i+1);

function bhy(x) =
    let(i=seg(x), x0=blade_pts[i][0], x1=blade_pts[i+1][0])
    lerp(blade_pts[i][1], blade_pts[i+1][1], (x-x0)/(x1-x0));

function bhz(x) =
    let(i=seg(x), x0=blade_pts[i][0], x1=blade_pts[i+1][0])
    lerp(blade_pts[i][2], blade_pts[i+1][2], (x-x0)/(x1-x0));

// 소켓 외벽 -Y 면 Y 좌표 및 Z 반높이
function outer_neg_y(x) = -(bhy(x) + wall + clearance);
function outer_hz(x)    =   bhz(x) + wall + clearance;

N = 16;
STL_LINK6 = "model/omx_f_mesh/follower_07_gripper_motorized.stl";

// ── SOCKET ───────────────────────────────────────────────────
module socket_outer() {
    hull()
        for (i = [0:N]) {
            x  = X_SOCKET_START + i*(X_SOCKET_END-X_SOCKET_START)/N;
            hy = bhy(x) + wall + clearance;
            hz = bhz(x) + wall + clearance;
            translate([x, 0, 0])
                cube([0.01, hy*2, hz*2], center=true);
        }
}

// 캐비티: 둥근 모서리 단면으로 blade 곡률 근사
module socket_cavity() {
    translate([-0.1, 0, 0])
    hull()
        for (i = [0:N]) {
            x  = X_SOCKET_START + i*(X_SOCKET_END-X_SOCKET_START)/N;
            hy = bhy(x) + clearance;
            hz = bhz(x) + clearance;
            r  = min(hy * 0.40, 3.0);
            translate([x, 0, 0])
            rotate([0, 90, 0])
            linear_extrude(height=0.01, center=true)
                offset(r=r, $fn=12)
                    square([(hz-r)*2, (hy-r)*2], center=true);
        }
}

module socket_clip() {
    hull()
        for (i = [0:N]) {
            x      = X_SOCKET_START + i*(X_SOCKET_END-X_SOCKET_START)/N;
            hy_out = bhy(x) + wall + clearance;
            hz_out = bhz(x) + wall + clearance;
            translate([x, hy_out, 0])
                cube([0.01, clip_lip*2, hz_out*1.2], center=true);
        }
}

module socket_body() {
    difference() {
        union() { socket_outer(); socket_clip(); }
        socket_cavity();
    }
}

// ── TIP CAP (블레이드 끝 이후 솔리드) ───────────────────────
module tip_cap() {
    if (tip_ext > 0) {
        // x=65 단면에서 팁으로 테이퍼 (half-Y/Z 점점 줄어듦)
        tip_hy = max(1.0, bhy(X_SOCKET_END) * 0.5);
        tip_hz = max(2.0, bhz(X_SOCKET_END) * 0.4);
        hull() {
            translate([X_SOCKET_END, 0, 0])
                cube([0.01,
                      (bhy(X_SOCKET_END)+wall+clearance)*2,
                      (bhz(X_SOCKET_END)+wall+clearance)*2],
                     center=true);
            translate([X_TIP_END, 0, 0])
                cube([0.01, tip_hy*2, tip_hz*2], center=true);
        }
    }
}

// ── 바깥 돌출 리지 ───────────────────────────────────────────
// 소켓 + 팁 전 구간에 걸쳐 -Y 면 바깥으로 삼각 리지 돌출
module ridge_at(x_pos) {
    // 소켓 구간은 blade_pts 보간, 팁 구간은 끝 단면 유지
    x_clamped = min(x_pos, X_SOCKET_END - 0.1);
    fy = outer_neg_y(x_clamped);   // -Y 면 위치
    hz = outer_hz(x_clamped);      // Z 반높이
    tooth_hz = max(1.0, hz - ridge_z_gap);

    linear_extrude(height=tooth_hz*2, center=true)
        polygon([
            [x_pos - ridge_width/2, fy + 0.5],
            [x_pos,                 fy - ridge_height],
            [x_pos + ridge_width/2, fy + 0.5],
        ]);
}

module all_ridges() {
    x_end = X_TIP_END - ridge_width/2;
    for (x = [X_SOCKET_START : ridge_pitch : x_end])
        ridge_at(x);
}

// ── FINGER ───────────────────────────────────────────────────
module finger_link6() {
    socket_body();
    tip_cap();
    all_ridges();
}

module finger_link7() {
    mirror([0, 1, 0]) finger_link6();
}

// ── PHANTOM ──────────────────────────────────────────────────
module finger_phantom_link6() {
    color("cyan", 0.25) import(STL_LINK6, convexity=10);
}
module finger_phantom_link7() {
    mirror([0, 1, 0]) finger_phantom_link6();
}

// ── RENDER ───────────────────────────────────────────────────
SEP = 60;

if (which_finger == "link6") {
    finger_link6();
    if (show_phantom) finger_phantom_link6();
} else if (which_finger == "link7") {
    finger_link7();
    if (show_phantom) finger_phantom_link7();
} else {
    translate([0,  SEP/2, 0]) { finger_link6(); if (show_phantom) finger_phantom_link6(); }
    translate([0, -SEP/2, 0]) { finger_link7(); if (show_phantom) finger_phantom_link7(); }
}

// ============================================================
//  튜닝
//  전체 길이  : tip_ext (팁 연장, 0 = 소켓만, 현재 10 mm)
//               → 소켓 43 mm + tip_ext = 전체 길이
//  리지 간격  : ridge_pitch (현재 10 mm)
//  리지 높이  : ridge_height (현재 2 mm)
//  클리어런스 : clearance (헐거우면 0.20, 빡빡하면 0.40)
//
//  STL 내보내기 전: show_phantom=false, which_finger="link6"/"link7"
// ============================================================
