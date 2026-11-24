#!/usr/bin/env python3
"""MG996R 1개 시저 리프트 — 3D 프린트 부품 STL 생성기
좌표계(조립 기준): X = 길이(0 = 라즈베리파이 쪽 끝), Y = 폭(중앙 0), Z = 높이(바닥 0), 단위 mm
"""
import math
import os
import numpy as np
from meshlib import (Part, extrude, circle, rect, stadium, hull, check_shell, rot_x)

# ======================================================================
# 설계 치수
# ======================================================================
BASE_L, BASE_W = 160.0, 100.0
WALL = 4.0
FLOOR = 3.0
WALL_H = 66.0

XF = 8.0          # 고정 핀 X
ZP = 58.0         # 아래쪽 핀/슬라이더 축 높이
ARM_L = 147.0     # 시저 팔 핀 간격
ARM_W, ARM_T = 14.0, 6.0
PIN_D = 6.0       # 회전축 핀 지름 (프린트)
PIN_S = 5.0       # 크랭크·푸시로드·트레이 핀 지름
HOLE_M4 = PIN_D + 0.4   # 핀 구멍 (이름은 호환용)
HOLE_M3 = PIN_S + 0.4

HORN_R = 37.0     # 크랭크 반경 (서보축 ↔ 핀)
ROD = 56.0        # 푸시로드 핀 간격
DTAB = 14.0       # 슬라이더 축 ↔ 푸시로드 핀 (탭 길이)
XS = 46.0         # 서보 출력축 X
SX = XS + DTAB - XF   # 기구학용: 고정핀 ↔ (서보축 + 탭) = 52
ANG_UP, ANG_DOWN = 32.0, 172.0

Z_CRANK = 50.0    # 크랭크 아랫면 = 서보 혼 윗면
CRANK_T = 4.0
Z_ROD0 = Z_CRANK + CRANK_T + 1.0      # 푸시로드 아랫면 55 (와셔 1mm)
ROD_T = 4.0
TAB_Z0 = Z_ROD0 + ROD_T + 1.0         # 탭 아랫면 60
TAB_T = 4.0

BAR = 12.0                 # 슬라이더 바 단면
BAR_HALF_Y = 31.5
SPACER_L = 7.5

# MG996R
SERVO_L, SERVO_W = 40.7, 19.7
SHAFT_OFF = 10.15          # 몸체 중심 ↔ 출력축
EAR_HOLE_HALF = 24.6       # 장착 구멍 간격 49.2 / 2
EAR_HOLE_Y = 5.0
Z_EAR = 32.5               # 서보 귀(플랜지) 아랫면 = 받침 기둥 윗면
EAR_TO_SPLINE = 16.3       # 귀 아랫면 ↔ 출력축(스플라인) 윗면 (MG996R 추정, 실측 권장)
SPLINE_DS = (5.7, 5.9, 6.1)  # 크랭크 스플라인 외경 후보 (25T)

# 상판
TOP_T = 5.0
TOP_ROD_BELOW = 8.0        # 상판 아랫면 ↔ 위쪽 핀 축
FLANGE_D = 16.0

# 턴테이블 캐리어 고정 핀 위치 (하판 좌표)
CARRIER_PINS = [(12.0, 0.0), (148.0, 0.0), (80.0, 34.0), (80.0, -34.0)]

# 레이어 (|y|)
A_Y = (39.0, 45.0)         # 바깥 팔
B_Y = (32.0, 38.0)         # 안쪽 팔


def s_of(a):
    a = math.radians(a)
    return -HORN_R * math.cos(a) + math.sqrt(ROD ** 2 - (HORN_R * math.sin(a)) ** 2)


def x_rel(a):           # 고정핀 ↔ 슬라이더 축
    return SX + s_of(a)


def h_of(a):            # 아래핀 ↔ 위핀 높이
    return math.sqrt(ARM_L ** 2 - x_rel(a) ** 2)


X_MIN, X_MAX = x_rel(ANG_UP), x_rel(ANG_DOWN)
SLOT0, SLOT1 = XF + X_MIN - 1.0, XF + X_MAX + 1.0


# ======================================================================
# 부품
# ======================================================================
def side_profile_holes():
    return [circle(XF, ZP, HOLE_M4 / 2), stadium(SLOT0, ZP, SLOT1, ZP, HOLE_M4 / 2 + 0.1)]


def make_base():
    p = Part("base")
    xc = XS + SHAFT_OFF
    cut = rect(xc - SERVO_L / 2 - 0.3, -SERVO_W / 2 - 0.7, xc + SERVO_L / 2 + 0.3, SERVO_W / 2 + 0.7)
    p.add(extrude(rect(0, -BASE_W / 2, BASE_L, BASE_W / 2),
                  [cut] + [circle(x, y, HOLE_M3 / 2, 32) for x, y in CARRIER_PINS], 0, FLOOR))
    # 긴 벽 (핀 구멍 + 슬롯)
    for y0, y1 in ((BASE_W / 2 - WALL, BASE_W / 2), (-BASE_W / 2, -BASE_W / 2 + WALL)):
        p.add(extrude(rect(0, 0, BASE_L, WALL_H), side_profile_holes(), y0, y1, "xz"))
    # 앞 벽 (파이 쪽) — 팔 끝이 지나가는 자리는 낮게
    yi = BASE_W / 2 - WALL + 0.5
    front = [(-yi, 0), (yi, 0), (yi, 50), (32, 50), (32, WALL_H), (-32, WALL_H), (-32, 50), (-yi, 50)]
    p.add(extrude(front, [rect(-8, 6, 8, 18), circle(-35, 25, HOLE_M3 / 2), circle(35, 25, HOLE_M3 / 2)],
                  0, WALL, "yz"))
    # 뒷 벽 (슬라이더 바가 지나가므로 낮게)
    p.add(extrude(rect(-yi, 0, yi, 50), [], BASE_L - WALL, BASE_L, "yz"))
    # 서보 받침 기둥 2개
    for hx, (x0, x1) in ((xc - EAR_HOLE_HALF, (xc - SERVO_L / 2 - 10.5, xc - SERVO_L / 2 - 0.5)),
                         (xc + EAR_HOLE_HALF, (xc + SERVO_L / 2 + 0.5, xc + SERVO_L / 2 + 10.5))):
        holes = [stadium(hx - 0.6, sy, hx + 0.6, sy, 1.4, 20) for sy in (-EAR_HOLE_Y, EAR_HOLE_Y)]
        p.add(extrude(rect(x0, -9.5, x1, 9.5), holes, FLOOR - 1, Z_EAR))
    return p


def make_top_plate():
    """조립 좌표, 위쪽 핀 축 z = 0 기준"""
    p = Part("top_plate")
    z0 = TOP_ROD_BELOW
    p.add(extrude(rect(0, -BASE_W / 2, BASE_L, BASE_W / 2), [], z0, z0 + TOP_T))
    prof = rect(0, z0 - FLANGE_D, BASE_L, z0 + 0.5)
    holes = [circle(XF, 0, HOLE_M4 / 2), stadium(SLOT0, 0, SLOT1, 0, HOLE_M4 / 2 + 0.1)]
    for y0, y1 in ((BASE_W / 2 - WALL, BASE_W / 2), (-BASE_W / 2, -BASE_W / 2 + WALL)):
        p.add(extrude(prof, holes, y0, y1, "xz"))
    return p


def make_arm():
    p = Part("arm")
    p.add(extrude(stadium(0, 0, ARM_L, 0, ARM_W / 2),
                  [circle(0, 0, HOLE_M4 / 2), circle(ARM_L / 2, 0, HOLE_M4 / 2), circle(ARM_L, 0, HOLE_M4 / 2)],
                  0, ARM_T))
    return p


def make_slider_bar():
    """조립 좌표, 바 축 = (x 0, z 0)"""
    p = Part("slider_bar")
    h = BAR / 2
    p.add(extrude(rect(-h, -h, h, h), [circle(0, 0, HOLE_M4 / 2)], -BAR_HALF_Y, BAR_HALF_Y, "xz"))
    tz0 = TAB_Z0 - ZP
    tab = hull(circle(-DTAB, 0, 5.5) + rect(-h - 0.01, -5.5, -h + 1.5, 5.5))
    p.add(extrude(tab, [circle(-DTAB, 0, HOLE_M3 / 2)], tz0, tz0 + TAB_T))
    return p


def spline_hole(d_out, teeth=25, depth=0.3, clear=0.05):
    """암 스플라인(구멍) 윤곽: 축의 이(齒)가 들어갈 홈"""
    ro, ri = d_out / 2 + clear, d_out / 2 - depth + clear
    pts = []
    for i in range(teeth):
        a0 = 2 * math.pi * i / teeth
        w = 2 * math.pi / teeth
        for f, r in ((0.0, ri), (0.2, ro), (0.5, ro), (0.7, ri)):
            a = a0 + f * w
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def make_crank(d_spline=5.9):
    """서보 출력축(25T)에 직접 끼우는 크랭크. 혼 나사(서보 동봉) 1개로 고정.
    조립 좌표: z=0 이 크랭크 판 아랫면(Z_CRANK)."""
    p = Part(f"crank_spline{d_spline:.1f}")
    boss_r = 7.0
    eng = 4.0                                   # 스플라인 물림 길이
    zb = -(CRANK_T + 1.0)                       # 보스 아랫면 (= 판 아래 5 mm)
    outline = hull(circle(0, 0, 13, 64) + circle(HORN_R, 0, 5.5, 32))
    tip = circle(HORN_R, 0, HOLE_M3 / 2, 32)
    # 보스: 스플라인 구간 + 나사 구간
    p.add(extrude(circle(0, 0, boss_r, 64), [spline_hole(d_spline)], zb, zb + eng))
    p.add(extrude(circle(0, 0, boss_r, 64), [circle(0, 0, 1.7, 24)], zb + eng - 0.2, 0.2))
    # 판: 아래 0.5 mm 는 나사 구멍, 위는 나사머리 자리(Ø6.2)
    p.add(extrude(outline, [circle(0, 0, 1.7, 24), tip], 0, 0.7))
    p.add(extrude(outline, [circle(0, 0, 3.1, 32), tip], 0.5, CRANK_T))
    return p


def make_pushrod():
    p = Part("pushrod")
    p.add(extrude(stadium(0, 0, ROD, 0, 5.5), [circle(0, 0, HOLE_M3 / 2), circle(ROD, 0, HOLE_M3 / 2)], 0, ROD_T))
    return p


def make_spacer():
    return Part("spacer").add(extrude(circle(0, 0, 5.5, 48), [circle(0, 0, HOLE_M4 / 2, 40)], 0, SPACER_L))


def make_washer(d_pin, name):
    return Part(name).add(extrude(circle(0, 0, d_pin / 2 + 2.5, 48), [circle(0, 0, d_pin / 2 + 0.25, 40)], 0, 1.0))


# ---------------- 프린트 핀 + C클립 ----------------
CLIP_T = 1.5


def _dcircle(r, zf, n=48):
    """바닥이 평평한(D형) 원 단면: 눕혀서 출력, 바닥 z = zf"""
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n
        y, z = r * math.cos(a), r * math.sin(a)
        pts.append((y, max(z, zf)))
    out = []
    for q in pts:                      # 중복 제거
        if not out or (abs(q[0] - out[-1][0]) > 1e-6 or abs(q[1] - out[-1][1]) > 1e-6):
            out.append(q)
    if abs(out[0][0] - out[-1][0]) < 1e-6 and abs(out[0][1] - out[-1][1]) < 1e-6:
        out.pop()
    # 바닥 평면 위 연속점 정리 (직선 위 중간점 제거)
    clean = []
    n = len(out)
    for i in range(n):
        a, b, c = out[i - 1], out[i], out[(i + 1) % n]
        if abs(a[1] - zf) < 1e-9 and abs(b[1] - zf) < 1e-9 and abs(c[1] - zf) < 1e-9:
            continue
        clean.append(b)
    return clean


def make_pin(d, grip, name, head=True):
    """머리 + 축(grip) + 클립홈 + 끝. X 방향으로 눕혀 출력 (적층 방향이 축과 나란해 튼튼함)."""
    r = d / 2
    rg = r - 0.75                      # 홈 반지름
    zf = -(rg - 0.1)                   # 공통 바닥면
    rh = r + 2.0
    p = Part(name)
    x = 0.0
    if head:
        p.add(extrude(_dcircle(rh, zf), [], 0, 2.5, "yz"))
        x = 2.5
    x_shaft_end = x + grip + 0.3
    p.add(extrude(_dcircle(r, zf), [], x - (0.2 if head else 0), x_shaft_end, "yz"))
    p.add(extrude(_dcircle(rg, zf), [], x_shaft_end - 0.2, x_shaft_end + CLIP_T + 0.2 + 0.2, "yz"))
    p.add(extrude(_dcircle(r, zf), [], x_shaft_end + CLIP_T + 0.2, x_shaft_end + CLIP_T + 0.2 + 1.5, "yz"))
    return p


def make_clip(d, name):
    """C 클립: 핀 홈에 옆에서 끼움"""
    rg = d / 2 - 0.75
    ri, ro = rg + 0.05, d / 2 + 2.6
    w = rg * 2 * 0.8                   # 입구 폭 (홈보다 좁아 '딸깍' 걸림)
    ao, ai = math.asin(w / 2 / ro), math.asin(w / 2 / ri)
    n = 40
    outer = [(ro * math.cos(ao + (2 * math.pi - 2 * ao) * i / n), ro * math.sin(ao + (2 * math.pi - 2 * ao) * i / n)) for i in range(n + 1)]
    inner = [(ri * math.cos(2 * math.pi - ai - (2 * math.pi - 2 * ai) * i / n), ri * math.sin(2 * math.pi - ai - (2 * math.pi - 2 * ai) * i / n)) for i in range(n + 1)]
    # 입구 바깥쪽 모따기용 손잡이
    return Part(name).add(extrude(outer + inner, [], 0, CLIP_T))


def make_shim(t):
    holes = [stadium(-0.6, sy, 0.6, sy, 1.6, 20) for sy in (-EAR_HOLE_Y, EAR_HOLE_Y)]
    return Part(f"shim_{t:g}mm").add(extrude(rect(-4.5, -9.5, 4.5, 9.5), holes, 0, t))


def make_pi_tray():
    """라즈베리파이 4/5 (85x56, 구멍 58x49) 옆 받침. 조립 좌표 x<0"""
    p = Part("pi_tray")
    D = 72.0
    p.add(extrude(rect(-D, -BASE_W / 2, 0, BASE_W / 2), [rect(-60, -46, -50, -40), rect(-60, 40, -50, 46)], 0, FLOOR))
    p.add(extrude(rect(-BASE_W / 2, 0, BASE_W / 2, 40),
                  [rect(-8, 6, 8, 18), circle(-35, 25, HOLE_M3 / 2), circle(35, 25, HOLE_M3 / 2)],
                  -WALL, 0, "yz"))
    # 보강 삼각 리브
    for yr in (-47.0, 47.0):
        p.add(extrude([(-30, FLOOR - 0.5), (-WALL + 0.5, FLOOR - 0.5), (-WALL + 0.5, 30)], [], yr - 1.5, yr + 1.5, "xz"))
    # 스탠드오프: 보드 긴 변(85)을 Y 방향, GPIO 헤더 쪽 긴 변을 베이스 쪽(x=-10)으로
    for a in (3.5, 61.5):
        for c in (3.5, 52.5):
            x = -10 - (56 - c)
            y = -42.5 + a
            p.add(extrude(circle(x, y, 3.2, 32), [circle(x, y, 1.3, 24)], FLOOR - 0.5, FLOOR + 5))
    return p


# ======================================================================
# 내보내기
# ======================================================================
def export(out="stl"):
    os.makedirs(out, exist_ok=True)
    G6 = {"pin_bottom_fixed": WALL + 1 + ARM_T,           # 벽 + 와셔 + 팔A
          "pin_middle": ARM_T + 1 + ARM_T,                  # 팔A + 와셔 + 팔B
          "pin_top_fixed": WALL + SPACER_L + 0.5 + ARM_T,   # 플랜지 + 스페이서 + 팔B
          "rod_slider": BASE_W + 2}                         # 양쪽 벽 + 바깥 와셔 2
    parts = [
        (make_base(), 1, None),
        (make_top_plate(), 1, rot_x(180)),
        (make_arm(), 4, None),
        (make_slider_bar(), 2, rot_x(180)),
        (make_pushrod(), 1, None),
        (make_spacer(), 6, None),
        (make_shim(1), 2, None),
        (make_shim(2), 2, None),
        (make_pin(PIN_D, G6["pin_bottom_fixed"], "pin6_bottom_fixed"), 2, None),
        (make_pin(PIN_D, G6["pin_middle"], "pin6_middle"), 2, None),
        (make_pin(PIN_D, G6["pin_top_fixed"], "pin6_top_fixed"), 2, None),
        (make_pin(PIN_D, G6["rod_slider"], "rod6_slider"), 2, None),
        (make_pin(PIN_S, ROD_T + 1 + CRANK_T, "pin5_link"), 4, None),
        (make_clip(PIN_D, "clip6"), 10, None),
        (make_clip(PIN_S, "clip5"), 10, None),
        (make_washer(PIN_D, "washer6"), 10, None),
        (make_washer(PIN_S, "washer5"), 4, None),
    ] + [(make_crank(d), 1, rot_x(180)) for d in SPLINE_DS]
    report = []
    for part, qty, R in parts:
        if R is not None:
            part = part.transformed(R)
        part = part.to_floor()
        for s in part.shells:
            bad, vol = check_shell(s)
            assert bad == 0 and vol > 0, (part.name, bad, vol)
        lo, hi = part.bounds()
        fn = f"{part.name}_x{qty}.stl"
        part.save(os.path.join(out, fn))
        report.append((fn, qty, hi - lo))
    return report


if __name__ == "__main__":
    print(f"Slider range x={XF + X_MIN:.1f} ~ {XF + X_MAX:.1f}, slot {SLOT0:.1f} ~ {SLOT1:.1f}")
    print(f"Scissor height {h_of(ANG_DOWN):.1f} → {h_of(ANG_UP):.1f}  (rise {h_of(ANG_UP) - h_of(ANG_DOWN):.1f} mm)")
    for fn, q, size in export():
        print(f"{fn:28s} {size[0]:6.1f} x {size[1]:6.1f} x {size[2]:5.1f}")
