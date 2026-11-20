"""Tiny mesh toolkit: 2D polygons with holes -> extruded watertight prisms -> STL.
Parts are unions of overlapping shells (slicers merge overlapping volumes)."""
import math
import struct
import numpy as np

SEG = 48  # circle resolution


# ---------------------------------------------------------------- 2D shapes
def circle(cx, cy, r, n=None):
    n = n or max(16, int(SEG * min(1.0, r / 6.0 + 0.4)))
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def stadium(x0, y0, x1, y1, r, n=40):
    """slot between two centres"""
    ang = math.atan2(y1 - y0, x1 - x0)
    pts = []
    for i in range(n // 2 + 1):
        a = ang - math.pi / 2 + math.pi * i / (n // 2)
        pts.append((x1 + r * math.cos(a), y1 + r * math.sin(a)))
    for i in range(n // 2 + 1):
        a = ang + math.pi / 2 + math.pi * i / (n // 2)
        pts.append((x0 + r * math.cos(a), y0 + r * math.sin(a)))
    return pts


def hull(points):
    pts = sorted(set((round(x, 9), round(y, 9)) for x, y in points))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def area(p):
    return 0.5 * sum(p[i][0] * p[(i + 1) % len(p)][1] - p[(i + 1) % len(p)][0] * p[i][1] for i in range(len(p)))


def ccw(p):
    return p if area(p) > 0 else p[::-1]


def cw(p):
    return p if area(p) < 0 else p[::-1]


# ---------------------------------------------------------------- triangulation
def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _in_tri(p, a, b, c, eps=1e-9):
    return _cross(a, b, p) >= -eps and _cross(b, c, p) >= -eps and _cross(c, a, p) >= -eps


def _seg_cross(p1, p2, q1, q2):
    """proper intersection (touching at endpoints ignored)"""
    d1, d2 = _cross(q1, q2, p1), _cross(q1, q2, p2)
    d3, d4 = _cross(p1, p2, q1), _cross(p1, p2, q2)
    return ((d1 > 1e-12 and d2 < -1e-12) or (d1 < -1e-12 and d2 > 1e-12)) and \
           ((d3 > 1e-12 and d4 < -1e-12) or (d3 < -1e-12 and d4 > 1e-12))


def _inside(pt, poly):
    x, y = pt
    c = False
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (a[1] > y) != (b[1] > y):
            if x < a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
                c = not c
    return c


def _bridge(poly, hole, others):
    """merge CW hole into CCW poly via the nearest mutually visible vertex pair."""
    mi = max(range(len(hole)), key=lambda i: (hole[i][0], -hole[i][1]))
    M = hole[mi]
    n = len(poly)
    order = sorted(range(n), key=lambda i: (poly[i][0] - M[0]) ** 2 + (poly[i][1] - M[1]) ** 2)
    edges = [(poly[i], poly[(i + 1) % n]) for i in range(n)]
    for h in [hole] + others:
        edges += [(h[i], h[(i + 1) % len(h)]) for i in range(len(h))]
    for bi in order:
        P = poly[bi]
        if any(_seg_cross(M, P, a, b) for a, b in edges):
            continue
        # segment must not pass through other vertices
        bad = False
        for q in poly + hole:
            if q == P or q == M:
                continue
            if abs(_cross(M, P, q)) < 1e-9 and min(M[0], P[0]) - 1e-9 <= q[0] <= max(M[0], P[0]) + 1e-9 \
                    and min(M[1], P[1]) - 1e-9 <= q[1] <= max(M[1], P[1]) + 1e-9:
                bad = True
                break
        if bad:
            continue
        mid = ((M[0] + P[0]) / 2, (M[1] + P[1]) / 2)
        if not _inside(mid, poly) or any(_inside(mid, h) for h in [hole] + others):
            continue
        hole_seq = hole[mi:] + hole[:mi + 1]
        return poly[:bi + 1] + hole_seq + [poly[bi]] + poly[bi + 1:]
    raise ValueError("bridge failed")


def triangulate(outer, holes=()):
    poly = list(ccw(outer))
    hs = sorted([list(cw(h)) for h in holes], key=lambda h: -max(p[0] for p in h))
    for k, h in enumerate(hs):
        poly = _bridge(poly, h, hs[k + 1:])
    idx = list(range(len(poly)))
    tris = []
    guard = 0
    while len(idx) > 3:
        guard += 1
        if guard > 100000:
            raise RuntimeError("triangulation stuck")
        n = len(idx)
        found = False
        for k in range(n):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
            a, b, c = poly[i0], poly[i1], poly[i2]
            cr = _cross(a, b, c)
            if cr <= 1e-12:
                continue
            ok = True
            for j in idx:
                if j in (i0, i1, i2):
                    continue
                q = poly[j]
                if q == a or q == b or q == c:
                    continue
                if _in_tri(q, a, b, c):
                    ok = False
                    break
            if ok:
                tris.append((i0, i1, i2))
                idx.pop(k)
                found = True
                break
        if not found:
            # remove a degenerate (collinear / zero-area) vertex
            for k in range(n):
                a, b, c = poly[idx[k - 1]], poly[idx[k]], poly[idx[(k + 1) % n]]
                if abs(_cross(a, b, c)) <= 1e-9:
                    tris.append((idx[k - 1], idx[k], idx[(k + 1) % n]))
                    idx.pop(k)
                    found = True
                    break
        if not found:
            raise RuntimeError("no ear found")
    tris.append(tuple(idx))
    return poly, tris


# ---------------------------------------------------------------- 3D
BASIS = {
    "xy": (np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])),   # extrude z
    "yz": (np.array([0, 1, 0]), np.array([0, 0, 1]), np.array([1, 0, 0])),   # extrude x
    "xz": (np.array([1, 0, 0]), np.array([0, 0, 1]), np.array([0, -1, 0])),  # extrude -y
}


def extrude(outer, holes=(), w0=0.0, w1=1.0, plane="xy"):
    """returns (N,3,3) triangles of a closed prism, outward normals.
    plane 'xz': give y range as w0,w1 (handled internally)."""
    eu, ev, ew = BASIS[plane]
    if plane == "xz":
        w0, w1 = -w1, -w0
    poly, tris = triangulate(outer, holes)
    P = np.array(poly, float)
    def pt(i, w):
        return eu * P[i, 0] + ev * P[i, 1] + ew * w
    out = []
    for a, b, c in tris:
        out.append([pt(a, w1), pt(b, w1), pt(c, w1)])     # top (normal +w)
        out.append([pt(a, w0), pt(c, w0), pt(b, w0)])     # bottom
    loops = [ccw(outer)] + [cw(h) for h in holes]
    for loop in loops:
        L = np.array(loop, float)
        n = len(L)
        for i in range(n):
            p, q = L[i], L[(i + 1) % n]
            A = eu * p[0] + ev * p[1] + ew * w0
            B = eu * q[0] + ev * q[1] + ew * w0
            C = eu * q[0] + ev * q[1] + ew * w1
            D = eu * p[0] + ev * p[1] + ew * w1
            out.append([A, B, C])
            out.append([A, C, D])
    return np.array(out)


class Part:
    def __init__(self, name):
        self.name = name
        self.shells = []

    def add(self, tris):
        self.shells.append(np.asarray(tris, float))
        return self

    def transformed(self, R=np.eye(3), t=(0, 0, 0)):
        p = Part(self.name)
        for s in self.shells:
            p.shells.append(s @ np.asarray(R).T + np.asarray(t))
        return p

    def tris(self):
        return np.concatenate(self.shells)

    def bounds(self):
        t = self.tris().reshape(-1, 3)
        return t.min(0), t.max(0)

    def to_floor(self):
        lo, hi = self.bounds()
        return self.transformed(t=(-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]))

    def save(self, path):
        T = self.tris()
        with open(path, "wb") as f:
            f.write(self.name.encode()[:80].ljust(80, b" "))
            f.write(struct.pack("<I", len(T)))
            for tri in T:
                n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                ln = np.linalg.norm(n)
                n = n / ln if ln > 0 else n
                f.write(struct.pack("<12fH", *n, *tri[0], *tri[1], *tri[2], 0))


def check_shell(tris):
    """watertight + consistently oriented + positive volume"""
    from collections import Counter
    key = lambda p: tuple(np.round(p, 6))
    edges = Counter()
    for t in tris:
        for i in range(3):
            edges[(key(t[i]), key(t[(i + 1) % 3]))] += 1
    bad = 0
    for (a, b), c in edges.items():
        if edges.get((b, a), 0) != c or c != 1:
            bad += 1
    vol = np.sum(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2]))) / 6
    return bad, vol


def rot_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def rot_y(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def rot_z(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
