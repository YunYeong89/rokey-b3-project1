"""간섭 검사: 움직이는 부품을 점으로 샘플링해 다른 부품 안에 들어가는지 각도별로 확인"""
import math
import numpy as np
import gen as G

STEP = 0.8


class Box:
    def __init__(s, x0, x1, y0, y1, z0, z1, name):
        s.b = np.array([[x0, y0, z0], [x1, y1, z1]], float); s.name = name
    def inside(s, P, tol):
        return np.all((P > s.b[0] + tol) & (P < s.b[1] - tol), axis=1)
    def sample(s):
        ax = [np.arange(s.b[0, i], s.b[1, i] + 1e-9, STEP) for i in range(3)]
        return np.stack(np.meshgrid(*ax, indexing="ij"), -1).reshape(-1, 3)


class SlabCapsule:
    """2D capsule (segment p0-p1, radius r) in plane (u,v) extruded along w.
    plane 'xz' (w=y) or 'xy' (w=z)"""
    def __init__(s, p0, p1, r, w0, w1, plane, name, mirror=False):
        s.p0, s.p1, s.r, s.w0, s.w1, s.plane, s.name, s.mirror = np.array(p0, float), np.array(p1, float), r, w0, w1, plane, name, mirror
    def _uvw(s, P):
        if s.plane == "xz":
            return P[:, [0, 2]], P[:, 1]
        return P[:, [0, 1]], P[:, 2]
    def inside(s, P, tol):
        uv, w = s._uvw(P)
        if s.mirror:
            w = np.abs(w)
        d = s.p1 - s.p0
        t = np.clip(((uv - s.p0) @ d) / max(d @ d, 1e-12), 0, 1)
        dist = np.linalg.norm(uv - (s.p0 + t[:, None] * d), axis=1)
        return (dist < s.r - tol) & (w > s.w0 + tol) & (w < s.w1 - tol)
    def sample(s):
        lo = np.minimum(s.p0, s.p1) - s.r; hi = np.maximum(s.p0, s.p1) + s.r
        us = np.arange(lo[0], hi[0] + 1e-9, STEP); vs = np.arange(lo[1], hi[1] + 1e-9, STEP)
        ws = np.arange(s.w0, s.w1 + 1e-9, STEP)
        ws = np.concatenate([ws, -ws]) if s.mirror else ws
        U, V, W = np.meshgrid(us, vs, ws, indexing="ij")
        if s.plane == "xz":
            P = np.stack([U, W, V], -1).reshape(-1, 3)
        else:
            P = np.stack([U, V, W], -1).reshape(-1, 3)
        return P[s.inside(P, -1e-6)]


def cyl_z(x, y, r, z0, z1, name):
    return SlabCapsule((x, y), (x, y), r, z0, z1, "xy", name)


def cyl_y(x, z, r, y0, y1, name, mirror=True):
    return SlabCapsule((x, z), (x, z), r, y0, y1, "xz", name, mirror)


def bodies(a):
    xr = G.XF + G.x_rel(a)            # 아래 슬라이더 축 X
    h = G.h_of(a)
    zt = G.ZP + h                      # 위 핀 축 Z
    xs, r = G.XS, G.HORN_R
    tip = (xs - r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
    tab = (xr - G.DTAB, 0.0)
    mid = (G.XF + G.x_rel(a) / 2, G.ZP + h / 2)
    B = {}
    # --- 움직이는 그룹 ---
    B["crank_hub"] = cyl_z(xs, 0, 13, G.Z_CRANK, G.Z_CRANK + G.CRANK_T, "crank")
    B["crank_boss"] = cyl_z(xs, 0, 7, G.Z_CRANK - 5, G.Z_CRANK, "crank boss")
    B["crank_arm"] = SlabCapsule((xs, 0), tip, 5.5, G.Z_CRANK, G.Z_CRANK + G.CRANK_T, "xy", "crank")
    B["tip_bolt"] = cyl_z(*tip, 4.6, G.Z_CRANK - 2.5, G.Z_CRANK, "tip pin head")
    B["tip_clip"] = cyl_z(*tip, 5.7, G.Z_ROD0 + G.ROD_T, G.Z_ROD0 + G.ROD_T + 3.4, "tip clip")
    B["pushrod"] = SlabCapsule(tip, tab, 5.5, G.Z_ROD0, G.Z_ROD0 + G.ROD_T, "xy", "pushrod")
    B["tab_bolt"] = cyl_z(*tab, 4.6, G.Z_ROD0 - 2.5, G.Z_ROD0, "tab pin head")
    B["tab_clip"] = cyl_z(*tab, 5.7, G.TAB_Z0 + G.TAB_T, G.TAB_Z0 + G.TAB_T + 3.4, "tab clip")
    B["bar"] = Box(xr - 6, xr + 6, -31.5, 31.5, G.ZP - 6, G.ZP + 6, "bottom bar")
    B["tab"] = SlabCapsule((xr - 6, 0), tab, 5.5, G.TAB_Z0, G.TAB_Z0 + G.TAB_T, "xy", "tab")
    B["b_spacer"] = cyl_y(xr, G.ZP, 5.5, 38.3, 45.7, "bottom spacer")
    B["b_rod_nut"] = cyl_y(xr, G.ZP, 5.7, 50.0, 56.0, "bottom rod head/clip")
    B["A"] = SlabCapsule((G.XF, G.ZP), (xr, zt), 7, *G.A_Y, "xz", "arm A", True)
    B["B"] = SlabCapsule((xr, G.ZP), (G.XF, zt), 7, *G.B_Y, "xz", "arm B", True)
    B["mid_bolt"] = cyl_y(*mid, 5.1, 45.0, 48.0, "middle pin head", True)
    B["mid_clip"] = cyl_y(*mid, 5.7, 27.0, 32.0, "middle clip", True)
    B["fix_nut_b"] = cyl_y(G.XF, G.ZP, 5.7, 34.5, 39.0, "bottom fixed clip")
    B["fix_nut_t"] = cyl_y(G.XF, zt, 5.7, 27.5, 32.0, "top fixed clip")
    B["fix_head_t"] = cyl_y(G.XF, zt, 5.1, 50.0, 52.6, "top fixed head")
    B["fix_head_b"] = cyl_y(G.XF, G.ZP, 5.1, 50.0, 52.6, "bottom fixed head")
    B["t_rod_end"] = cyl_y(xr, zt, 5.7, 50.0, 56.0, "top rod head/clip")
    B["t_spacer"] = cyl_y(xr, zt, 5.5, 31.6, 38.9, "top spacer")
    B["tbar"] = Box(xr - 6, xr + 6, -31.5, 31.5, zt - 6, zt + 6, "top bar")
    B["ttab"] = SlabCapsule((xr - 6, 0), tab, 5.5, zt + G.TAB_Z0 - G.ZP, zt + G.TAB_Z0 - G.ZP + 4, "xy", "top tab")
    zu = zt + G.TOP_ROD_BELOW
    B["plate"] = Box(0, 160, -50, 50, zu, zu + G.TOP_T, "top plate")
    B["flangeL"] = Box(0, 160, 46, 50, zu - G.FLANGE_D, zu, "flange")
    B["flangeR"] = Box(0, 160, -50, -46, zu - G.FLANGE_D, zu, "flange")
    # --- 고정 ---
    xc = G.XS + G.SHAFT_OFF
    B["servo"] = Box(xc - 20.35, xc + 20.35, -9.85, 9.85, G.Z_EAR - 26.6, G.Z_EAR + 10.6, "servo body")
    B["servo_ears"] = Box(xc - 27.25, xc + 27.25, -9.85, 9.85, G.Z_EAR, G.Z_EAR + 2.5, "servo ears")
    B["spline"] = cyl_z(G.XS, 0, 3.0, G.Z_EAR + 10.6, G.Z_CRANK - 1, "spline")
    B["pillar1"] = Box(xc - 30.85, xc - 20.85, -9.5, 9.5, 0, G.Z_EAR, "pillar")
    B["pillar2"] = Box(xc + 20.85, xc + 30.85, -9.5, 9.5, 0, G.Z_EAR, "pillar")
    B["floor"] = Box(0, 160, -50, 50, 0, G.FLOOR, "floor")
    B["wallL"] = Box(0, 160, 46, 50, 0, G.WALL_H, "side wall")
    B["wallR"] = Box(0, 160, -50, -46, 0, G.WALL_H, "side wall")
    B["front_low"] = Box(0, 4, -46, 46, 0, 50, "front wall")
    B["front_mid"] = Box(0, 4, -32, 32, 0, G.WALL_H, "front wall")
    B["rear"] = Box(156, 160, -46, 46, 0, 50, "rear wall")
    return B


# 서로 붙어 있는(체결된) 쌍은 제외
JOINED = {
    frozenset(p) for p in [
        ("crank_hub", "crank_arm"), ("crank_hub", "spline"), ("crank_boss", "spline"), ("crank_boss", "crank_hub"), ("crank_boss", "crank_arm"), ("crank_boss","servo"),
        ("tip_clip","pushrod"), ("tab_clip","tab"), ("tip_clip","tip_bolt"), ("mid_clip","B"), ("mid_bolt","A"), ("mid_clip","mid_bolt"),
        ("fix_head_t","flangeL"), ("fix_head_t","flangeR"), ("fix_head_b","wallL"), ("fix_head_b","wallR"), ("t_rod_end","flangeL"), ("t_rod_end","flangeR"), ("t_rod_end","t_spacer"),
        ("fix_nut_t","B"), ("fix_nut_b","A"), ("tab_bolt","pushrod"), ("tab_clip","ttab"), ("crank_arm", "tip_bolt"), ("pushrod", "tip_bolt"),
        ("crank_hub", "tip_bolt"), ("pushrod", "tab_bolt"), ("tab", "tab_bolt"), ("bar", "tab"),
        ("bar", "B"), ("b_spacer", "B"), ("b_spacer", "wallL"), ("b_spacer", "wallR"), ("b_spacer", "b_rod_nut"),
        ("b_rod_nut", "wallL"), ("b_rod_nut", "wallR"),
        ("A", "B"), ("A", "mid_bolt"), ("B", "mid_bolt"), ("A", "fix_nut_b"),
        ("B", "fix_nut_t"), ("t_spacer", "B"), ("t_spacer", "A"), ("t_spacer", "flangeL"), ("t_spacer", "flangeR"),
        ("tbar", "A"), ("tbar", "t_spacer"), ("tbar", "ttab"), ("plate", "flangeL"), ("plate", "flangeR"),
        ("servo", "servo_ears"), ("servo", "spline"), ("servo_ears", "pillar1"), ("servo_ears", "pillar2"),
        ("pillar1", "floor"), ("pillar2", "floor"), ("servo", "floor"),
        ("floor", "wallL"), ("floor", "wallR"), ("floor", "front_low"), ("floor", "front_mid"), ("floor", "rear"),
        ("wallL", "front_low"), ("wallR", "front_low"), ("wallL", "rear"), ("wallR", "rear"),
        ("front_low", "front_mid"), ("b_spacer", "A"),
        ("A", "wallL"), ("A", "wallR"), ("B", "flangeL"), ("B", "flangeR"),  # 핀으로 연결(와셔 간격 1mm)
        ("A", "flangeL"), ("A", "flangeR"),
    ]
}
MOVING = ["crank_hub", "crank_boss", "crank_arm", "tip_bolt", "tip_clip", "tab_clip", "mid_clip", "fix_head_t", "t_rod_end", "pushrod", "tab_bolt", "bar", "tab", "b_spacer", "b_rod_nut",
          "A", "B", "mid_bolt", "fix_nut_b", "fix_nut_t", "t_spacer", "tbar", "ttab", "plate", "flangeL", "flangeR"]


def run(tol=0.3):
    worst = {}
    for a in np.arange(G.ANG_UP, G.ANG_DOWN + 0.01, 4.0):
        B = bodies(a)
        for m in MOVING:
            P = B[m].sample()
            for k, o in B.items():
                if k == m or frozenset((k, m)) in JOINED:
                    continue
                n = int(B[o].inside(P, tol).sum()) if False else int(o.inside(P, tol).sum())
                if n:
                    key = tuple(sorted((m, k)))
                    if key not in worst or n > worst[key][1]:
                        worst[key] = (a, n)
    return worst


if __name__ == "__main__":
    w = run()
    if not w:
        print("간섭 없음 ✅")
    for (p, q), (a, n) in sorted(w.items()):
        print(f"간섭: {p:10s} ↔ {q:10s}  각도 {a:5.1f}°  점 {n}")
