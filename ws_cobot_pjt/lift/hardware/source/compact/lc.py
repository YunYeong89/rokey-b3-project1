#!/usr/bin/env python3
"""소형 저상 리프트 — 상판·하판 120 x 80 mm, 상승 약 62 mm, MG996R 눕힘(크랭크 세로 회전)
좌표: X 길이(0 = 앞벽), Y 폭(중앙 0), Z 높이(바닥 0), 단위 mm
"""
import math
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lift3d"))
import gen as G                                     # noqa: E402  (핀·클립·스플라인 함수 재사용)
from meshlib import (Part, extrude, circle, rect, stadium, hull, check_shell, rot_x, rot_y)  # noqa: E402

# ---------------- 크기 ----------------
BL, BW = 120.0, 80.0       # 하판·상판 크기
WALL, FLOOR = 4.0, 3.0
XF = 7.0                   # 고정핀 X
ZP = 14.0                  # 아래 핀·슬라이더 축 높이
WALL_H = 21.5
ARM_L, ARM_W, ARM_T = 110.0, 14.0, 4.0
PIN_D, PIN_S = 6.0, 5.0
H6, H5 = PIN_D + 0.4, PIN_S + 0.4
GAP = 0.5
Y_WALL = BW / 2 - WALL                     # 36 벽 안쪽면
A_Y = (Y_WALL - GAP - ARM_T, Y_WALL - GAP)  # 31.5 ~ 35.5  바깥 팔
B_Y = (A_Y[0] - GAP - ARM_T, A_Y[0] - GAP)  # 27 ~ 31     안쪽 팔
BAR_Y = B_Y[0] - GAP                        # 26.5 슬라이더 바 끝
SPACER_L = ARM_T + GAP                      # 4.5
BAR = 12.0

# ---------------- 구동부 ----------------
SERVO_L, SERVO_W, SERVO_H = 40.7, 19.7, 42.9
SHAFT_OFF, EAR_HALF, EAR_HOLE_HALF, EAR_HOLE_Y = 10.15, 27.25, 24.6, 5.0
EAR_TO_SPLINE = 16.3
ZC = FLOOR + 0.5 + SERVO_W / 2             # 13.35 서보 축 높이
XS = 42.0                                   # 서보 축 X
R = 24.0                                    # 크랭크 반경
ROD = 47.0                                  # 푸시로드 핀 간격
TH_DOWN, TH_SPAN = 3.0, 140.0
SERVO_DOWN, SERVO_UP = 172, 32
CRANK_T, ROD_T = 4.0, 4.0
Y_SERVO_BOTTOM = -26.4
Y_SHAFT_END = Y_SERVO_BOTTOM + SERVO_H      # 16.5
Y_EAR = Y_SHAFT_END - EAR_TO_SPLINE         # 0.2
CR_BOSS0 = Y_SHAFT_END - 4.0
CR_PLATE0 = CR_BOSS0 + 5.0                  # 17.5
CR_PLATE1 = CR_PLATE0 + CRANK_T             # 21.5
PR_Y0 = CR_PLATE1 + GAP                     # 22
PR_Y1 = PR_Y0 + ROD_T                       # 26
BAR_A = (-BAR_Y, PR_Y0 - GAP)               # 아래 슬라이더 바 (-26.5 ~ 21.5)

# 상판
TOP_T, TOP_ROD_BELOW, FLANGE_D = 5.0, 8.0, 16.0


def crank_tip(th):
    t = math.radians(th)
    return XS + R * math.cos(t), ZC + R * math.sin(t)


def rod_x(th):
    tx, tz = crank_tip(th)
    return tx + math.sqrt(ROD ** 2 - (ZP - tz) ** 2)


def h_of(th):
    return math.sqrt(ARM_L ** 2 - (rod_x(th) - XF) ** 2)


X_MIN, X_MAX = rod_x(TH_DOWN + TH_SPAN), rod_x(TH_DOWN)
SLOT0, SLOT1 = X_MIN - 1.0, X_MAX + 1.0


# ---------------- 부품 ----------------
def side_holes(z):
    return [circle(XF, z, H6 / 2), stadium(SLOT0, z, SLOT1, z, H6 / 2 + 0.1)]


def make_base():
    p = Part("base")
    p.add(extrude(rect(0, -BW / 2, BL, BW / 2), [], 0, FLOOR))
    for y0, y1 in ((Y_WALL, BW / 2), (-BW / 2, -Y_WALL)):
        p.add(extrude(rect(0, 0, BL, WALL_H), side_holes(ZP), y0, y1, "xz"))
    yi = Y_WALL + 0.5
    yn = B_Y[0]                                     # 27: 이보다 바깥은 팔·클립 자리라 낮게
    front = [(-yi, 0), (yi, 0), (yi, 6), (yn, 6), (yn, WALL_H), (-yn, WALL_H), (-yn, 6), (-yi, 6)]
    p.add(extrude(front, [rect(-24, 5, -12, 15)], 0, WALL, "yz"))
    p.add(extrude(rect(-yi, 0, yi, 6), [], BL - WALL, BL, "yz"))
    # 서보 고정 칸막이
    xc = XS - SHAFT_OFF
    x0, x1 = xc - SERVO_L / 2 - 0.3, xc + SERVO_L / 2 + 0.3
    z0, z1 = ZC - SERVO_W / 2 - 0.5, ZC + SERVO_W / 2 + 0.3
    wall = rect(WALL - 0.5, 0, xc + EAR_HALF + 1.5, z1 + 3.5)
    holes = [rect(x0, z0, x1, z1)]
    holes += [circle(xc + sx * EAR_HOLE_HALF, ZC + sz * EAR_HOLE_Y, 1.3, 20) for sx in (-1, 1) for sz in (-1, 1)]
    p.add(extrude(wall, holes, Y_EAR - 4.0, Y_EAR, "xz"))
    # 서보 바닥 끝 받침
    p.add(extrude(rect(x0 + 3, Y_SERVO_BOTTOM - 2.5, x1 - 3, Y_SERVO_BOTTOM - 0.5), [], FLOOR - 0.3, ZC + 4))
    return p


def make_top_plate():
    """조립 좌표, 위 핀 축 z=0"""
    p = Part("top_plate")
    z0 = TOP_ROD_BELOW
    p.add(extrude(rect(0, -BW / 2, BL, BW / 2), [], z0, z0 + TOP_T))
    for y0, y1 in ((Y_WALL, BW / 2), (-BW / 2, -Y_WALL)):
        p.add(extrude(rect(0, z0 - FLANGE_D, BL, z0 + 0.5), side_holes(0), y0, y1, "xz"))
    return p


def make_arm(inner=False):
    """inner=True: 안쪽 팔(B) — 가운데 구멍을 Ø5.9 로 좁혀 가운데 핀을 눌러 끼움 (클립 없이 고정)"""
    mid = 5.9 / 2 if inner else H6 / 2
    return Part("arm").add(extrude(stadium(0, 0, ARM_L, 0, ARM_W / 2),
                                   [circle(0, 0, H6 / 2), circle(ARM_L / 2, 0, mid), circle(ARM_L, 0, H6 / 2)],
                                   0, ARM_T))


def make_bar(y0, y1, name):
    h = BAR / 2
    return Part(name).add(extrude(rect(-h, -h, h, h), [circle(0, 0, H6 / 2)], y0, y1, "xz"))


def make_crank(d_spline=5.9):
    p = Part(f"crank_spline{d_spline:.1f}")
    outline = hull(circle(0, 0, 9.5, 64) + circle(R, 0, 5.5, 32))
    tip = circle(R, 0, H5 / 2, 32)
    p.add(extrude(circle(0, 0, 7.0, 64), [G.spline_hole(d_spline)], -5.0, -1.0))
    p.add(extrude(circle(0, 0, 7.0, 64), [circle(0, 0, 1.7, 24)], -1.2, 0.2))
    p.add(extrude(outline, [circle(0, 0, 1.7, 24), tip], 0, 0.7))
    p.add(extrude(outline, [circle(0, 0, 3.1, 32), tip], 0.5, CRANK_T))
    return p


def make_pushrod():
    """크랭크 쪽 구멍은 핀이 꽉 끼는 Ø4.9 (핀이 푸시로드에 고정, 크랭크에서 회전)"""
    return Part("pushrod").add(extrude(hull(circle(0, 0, 5.5, 40) + circle(ROD, 0, 5.8, 40)),
                                       [circle(0, 0, 4.9 / 2, 32), circle(ROD, 0, H6 / 2, 32)], 0, ROD_T))


def make_press_pin(d, grip, name):
    """머리 + 축 (홈·클립 없음). 좁은 구멍에 눌러 끼워 고정"""
    r = d / 2
    zf = -(r - 0.85)
    p = Part(name)
    p.add(extrude(G._dcircle(r + 2.0, zf), [], 0, 2.5, "yz"))
    p.add(extrude(G._dcircle(r, zf), [], 2.3, 2.5 + grip, "yz"))
    return p


def make_spacer():
    return Part("spacer").add(extrude(circle(0, 0, 5.5, 48), [circle(0, 0, H6 / 2, 40)], 0, SPACER_L))


GRIP = {"pin6_bottom_fixed": WALL + GAP + ARM_T,
        "pin6_middle": ARM_T + GAP + ARM_T,
        "pin6_top_fixed": WALL + SPACER_L + ARM_T + 0.5,
        "rod6_slider": BW + 0.6}


def export(out="stl"):
    os.makedirs(out, exist_ok=True)
    parts = [
        ("01_하판", make_base(), 1, None),
        ("02_상판", make_top_plate(), 1, rot_x(180)),
        ("03_시저팔_바깥A", make_arm(False), 2, None),
        ("03_시저팔_안쪽B", make_arm(True), 2, None),
        ("04_아래슬라이더바", make_bar(*BAR_A, "bar_bottom"), 1, rot_x(90)),
        ("05_위슬라이더바", make_bar(-BAR_Y, BAR_Y, "bar_top"), 1, rot_x(90)),
        ("06_푸시로드", make_pushrod(), 1, None),
    ] + [(f"07_크랭크_스플라인{d:.1f}", make_crank(d), 1, rot_x(180)) for d in G.SPLINE_DS] + [
        ("08_슬라이더축", G.make_pin(PIN_D, GRIP["rod6_slider"], "rod"), 2, None),
        ("09_스페이서", make_spacer(), 6, None),
        ("10_핀6_아래고정", G.make_pin(PIN_D, GRIP["pin6_bottom_fixed"], "p"), 2, None),
        ("11_핀6_가운데", make_press_pin(PIN_D, GRIP["pin6_middle"], "p"), 2, None),
        ("12_핀6_위고정", G.make_pin(PIN_D, GRIP["pin6_top_fixed"], "p"), 2, None),
        ("13_핀5_크랭크", make_press_pin(PIN_S, CRANK_T + GAP + ROD_T, "p"), 2, None),
        ("14_클립6", G.make_clip(PIN_D, "c"), 8, None),
    ]
    rep = []
    for name, part, q, Rm in parts:
        if Rm is not None:
            part = part.transformed(Rm)
        part = part.to_floor()
        for s in part.shells:
            bad, vol = check_shell(s)
            assert bad == 0 and vol > 0, (name, bad, vol)
        lo, hi = part.bounds()
        fn = f"{name}_x{q}.stl"
        part.save(os.path.join(out, fn))
        rep.append((fn, q, hi - lo))
    return rep


if __name__ == "__main__":
    hs = [h_of(TH_DOWN + TH_SPAN * k / 140) for k in range(141)]
    print(f"슬라이더 X {X_MAX:.1f} → {X_MIN:.1f}, 슬롯 {SLOT0:.1f}~{SLOT1:.1f} (판 끝 {BL})")
    print(f"시저 높이 {hs[0]:.1f} → {hs[-1]:.1f}  상승 {hs[-1] - hs[0]:.1f} mm, 단조 {all(b >= a for a, b in zip(hs, hs[1:]))}")
    pk = max(9.81 * (b - a) / 1000 / math.radians(TH_SPAN / 140) / 0.0981 for a, b in zip(hs, hs[1:]))
    top0 = ZP + hs[0] + TOP_ROD_BELOW + TOP_T
    print(f"1 kg 최대 토크 {pk:.2f} kg·cm | 하판 벽 {WALL_H} | 상판 윗면 {top0:.1f} → {top0 + hs[-1] - hs[0]:.1f}")
    print(f"Y 배치: 팔A {A_Y}, 팔B {B_Y}, 바 ±{BAR_Y}, 서보 {Y_SERVO_BOTTOM}~{Y_SHAFT_END}, 크랭크판 {CR_PLATE0}~{CR_PLATE1}, 푸시로드 {PR_Y0}~{PR_Y1}")
    for fn, q, s in export():
        print(f"{fn:34s} {s[0]:6.1f} x {s[1]:6.1f} x {s[2]:5.1f}")
