#!/usr/bin/env python3
"""자동차 조립 STL → 웹 3D 용 car.glb

  STL 하나 → 떨어진 덩어리(부품)로 나누기 → 방향 맞추기 → 원점·크기 맞추기 → 부품 이름 붙이기
  → 삼각형 줄이기 → GLB 저장

좌표 (viewer3d.js 와 약속, 단위 mm)
  +X = 차 앞쪽, +Y = 위, +Z = 차 오른쪽, 원점 = 차 뒤·왼쪽·바닥 모서리

방향 찾는 법
  1. 방향 기준 상자(OBB) 축으로 돌림
  2. 삼각형 수·크기가 같은 큰 덩어리 4개 = 바퀴
     바퀴가 얇은 축 = 좌우, 바퀴 중심이 멀리 퍼진 나머지 축 = 앞뒤, 남은 축 = 위아래 (바퀴가 아래)
  3. 바퀴 중심 4개로 바닥면을 다시 맞춤 (OBB 가 살짝 기울어 있어도 바르게)
  4. 앞쪽: bonnet 으로 이름 붙인 덩어리가 있는 쪽. 없으면 OBB 방향 그대로 (틀리면 --flip-front)

사용법 (web/ 에서)
  python3 -m venv tools/.venv
  tools/.venv/bin/pip install trimesh numpy scipy networkx fast_simplification
  tools/.venv/bin/python tools/car_to_glb.py static/models/assembly.stl --list
  tools/.venv/bin/python tools/car_to_glb.py static/models/assembly.stl -o static/models/car.glb \\
      --length-mm 260 --name 8=bonnet --name 9=roof

부품 이름 규칙: viewer3d.js 가 찾는 이름 = bonnet, mirror_l, mirror_r, door_fl, door_fr, door_rl, door_rr
  (roof 는 스캔 구역 계산에 씀). 바퀴는 자동으로 wheels, 이름 없는 덩어리는 body 로 합쳐진다.
  같은 이름을 여러 번 주면 (--name 8=bonnet --name 11=bonnet) 한 메쉬로 합쳐진다.
"""
import argparse
import sys

import numpy as np
import trimesh

COLORS = {  # viewer3d.js COLORS 와 맞춤 (RGB 0~255)
    "body": (0x9F, 0xB3, 0xC8), "part": (0xC3, 0xD0, 0xDD), "wheels": (0x2A, 0x2F, 0x36),
}
VIEWER_PARTS = {"bonnet", "mirror_l", "mirror_r", "door_fl", "door_fr", "door_rl", "door_rr"}


# ---------------------------------------------------------------- 읽기·나누기
def load_parts(path):
    mesh = trimesh.load(path, force="mesh")
    parts = mesh.split(only_watertight=False)
    parts = sorted(parts, key=lambda p: (-len(p.faces), tuple(np.round(p.bounds.mean(0), 3))))
    return mesh, parts


def find_wheels(parts, extents):
    """삼각형 수가 같고 크기가 비슷한 덩어리 4개 중 가장 큰 묶음 → 바퀴 인덱스"""
    for i in range(min(len(parts), 30)):
        same = [j for j in range(len(parts))
                if abs(len(parts[j].faces) - len(parts[i].faces)) <= max(2, len(parts[i].faces) * 0.001)
                and np.allclose(np.sort(extents[j]), np.sort(extents[i]), rtol=0.02)]
        if len(same) == 4:
            return same
    return None


# ---------------------------------------------------------------- 방향
def orient(mesh, parts, flip_front=False, front_parts=()):
    """원본 → 차량 좌표 (X 앞, Y 위, Z 오른쪽) 회전·이동 행렬(4x4, 크기 조정 전)과 바퀴 인덱스"""
    T_obb, _ = trimesh.bounds.oriented_bounds(mesh)
    pts = lambda p: trimesh.transform_points(p.vertices, T_obb)
    ext = [np.ptp(pts(p), axis=0) for p in parts]
    wheels = find_wheels(parts, ext)
    if wheels is None:
        sys.exit("바퀴(똑같은 큰 덩어리 4개)를 못 찾음 — --list 로 확인")
    wc = np.array([(lambda q: (q.min(0) + q.max(0)) / 2)(pts(parts[i])) for i in wheels])   # OBB 좌표의 바퀴 중심
    width_ax = int(np.argmin(ext[wheels[0]]))                         # 바퀴가 얇은 축 = 좌우
    rest = [a for a in range(3) if a != width_ax]
    len_ax = max(rest, key=lambda a: np.ptp(wc[:, a]))                # 바퀴 중심이 퍼진 축 = 앞뒤
    up_ax = [a for a in rest if a != len_ax][0]

    allp = trimesh.transform_points(mesh.vertices, T_obb)
    up = np.eye(3)[up_ax] * (1 if wc[:, up_ax].mean() < allp[:, up_ax].mean() else -1)
    # 바퀴 중심 4개로 바닥면 맞추기 (평면 법선 = 위)
    c = wc - wc.mean(0)
    n = np.linalg.svd(c)[2][-1]
    up = n if n @ up > 0 else -n
    # 앞: 앞바퀴 둘 - 뒷바퀴 둘 (일단 OBB 축 + 방향), bonnet 이 있으면 그쪽
    side = wc[:, len_ax] > np.median(wc[:, len_ax])
    fwd = wc[side].mean(0) - wc[~side].mean(0)
    fwd -= up * (fwd @ up); fwd /= np.linalg.norm(fwd)
    if front_parts:
        fc = np.mean([pts(parts[i]).mean(0) for i in front_parts], axis=0)
        if (fc - wc.mean(0)) @ fwd < 0:
            fwd = -fwd
    if flip_front:
        fwd = -fwd
    right = np.cross(fwd, up)                                         # X × Y = Z (오른손 좌표, 거울상 없음)
    R = np.eye(4); R[:3, :3] = np.vstack([fwd, up, right])           # OBB → 차량 (행 = 새 축)
    return R @ T_obb, wheels


def contained(inner, outer, tol=0.02):
    pad = (outer[1] - outer[0]) * tol
    return np.all(inner[0] >= outer[0] - pad) and np.all(inner[1] <= outer[1] + pad)


# ---------------------------------------------------------------- 메인
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl")
    ap.add_argument("-o", "--out", default="static/models/car.glb")
    ap.add_argument("--length-mm", type=float, default=260.0, help="실제 모형 전체 길이 (앞뒤) [mm], 기본 260")
    ap.add_argument("--name", action="append", default=[], metavar="IDX=NAME", help="덩어리 이름 (예: 8=bonnet)")
    ap.add_argument("--target-faces", type=int, default=150_000, help="전체 삼각형 목표 수 (기본 15만)")
    ap.add_argument("--flip-front", action="store_true", help="앞뒤가 거꾸로 나오면")
    ap.add_argument("--list", action="store_true", help="부품 번호·크기·중심 표만 출력")
    a = ap.parse_args()

    names = {}
    for s in a.name:
        i, _, nm = s.partition("=")
        if not i.isdigit() or not nm:
            sys.exit(f"--name 형식: 인덱스=이름 (받은 값 {s!r})")
        names[int(i)] = nm.strip()

    print(f"읽는 중: {a.stl}")
    mesh, parts = load_parts(a.stl)
    print(f"  삼각형 {len(mesh.faces):,}개, 덩어리 {len(parts)}개")
    front = [i for i, n in names.items() if n == "bonnet" and i < len(parts)]
    M, wheels = orient(mesh, parts, a.flip_front, front)
    for p in parts:
        p.apply_transform(M)
    lo = np.min([p.bounds[0] for p in parts], axis=0)
    hi = np.max([p.bounds[1] for p in parts], axis=0)
    k = a.length_mm / (hi[0] - lo[0])
    S = np.eye(4) * k; S[3, 3] = 1; S[:3, 3] = -lo * k                 # 원점 = 뒤·왼쪽·바닥
    for p in parts:
        p.apply_transform(S)
    L, H, W = (hi - lo) * k
    print(f"  크기 L(앞뒤) {L:.1f} × W(좌우) {W:.1f} × H(높이) {H:.1f} mm  (원본 단위 × {k:.5f})")

    # 바퀴: 4개 + 바퀴 상자 안에 들어가는 덩어리 (휠·캡 등)
    wheel_set = set(wheels)
    for i, p in enumerate(parts):
        if i not in wheel_set and i not in names and any(contained(p.bounds, parts[w].bounds) for w in wheels):
            wheel_set.add(i)

    if a.list:
        print(f"\n{'번호':>4} {'삼각형':>7}  {'크기 L×H×W [mm]':<22} {'중심 x,y(높이),z [mm]':<24} 추정/이름")
        for i, p in enumerate(parts):
            e, c = p.extents, p.bounds.mean(0)
            tag = names.get(i, "wheels" if i in wheel_set else "")
            print(f"{i:>4} {len(p.faces):>7}  {e[0]:6.1f} ×{e[1]:6.1f} ×{e[2]:6.1f}   "
                  f"{c[0]:6.1f},{c[1]:6.1f},{c[2]:6.1f}     {tag}")
        print("\n좌표: x 0=뒤 → 앞, y 0=바닥 → 위, z 0=왼쪽 → 오른쪽")
        return

    # 이름별로 묶기
    groups = {}
    for i, p in enumerate(parts):
        nm = names.get(i) or ("wheels" if i in wheel_set else "body")
        groups.setdefault(nm, []).append(p)

    # 삼각형 줄이기: 덩어리마다 같은 비율 (바퀴는 WHEEL_WEIGHT 배로 더 줄임, 작은 덩어리는 그대로)
    import fast_simplification
    WHEEL_WEIGHT = 0.4
    n_wheel = sum(len(parts[i].faces) for i in wheel_set)
    n_other = sum(len(p.faces) for p in parts) - n_wheel
    base_ratio = min(1.0, a.target_faces / (n_other + WHEEL_WEIGHT * n_wheel))
    scene = trimesh.Scene()
    out_faces = 0
    for nm, ps in groups.items():
        keep = []
        ratio = base_ratio * (WHEEL_WEIGHT if nm == "wheels" else 1.0)
        for p in ps:
            if ratio < 1 and len(p.faces) > 400:
                v, f = fast_simplification.simplify(p.vertices.astype(np.float32), p.faces, target_reduction=1 - ratio)
                p = trimesh.Trimesh(v, f, process=True)
            keep.append(p)
        g = trimesh.util.concatenate(keep)
        g.merge_vertices()
        g = g.smooth_shaded                                          # 날카로운 모서리는 법선을 나눔 (CAD 느낌 유지)
        col = COLORS.get(nm, COLORS["part"])
        g.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
            name=nm, baseColorFactor=[*(c / 255 for c in col), 1.0],
            metallicFactor=0.15 if nm != "wheels" else 0.0, roughnessFactor=0.55))
        scene.add_geometry(g, node_name=nm, geom_name=nm)
        out_faces += len(g.faces)
        print(f"  {nm:<10} 덩어리 {len(ps):>2}개  삼각형 {len(g.faces):>7,}")
    missing = sorted(VIEWER_PARTS - groups.keys())
    if missing:
        print(f"  (모델에 없는 부품: {', '.join(missing)} → 뷰어가 반투명 상자로 대신 표시)")
    layout = make_layout(parts, names, wheels, wheel_set, (L, W, H))
    glb = scene.export(file_type="glb")
    glb = set_scene_extras(glb, {"car_layout": layout})
    with open(a.out, "wb") as f:
        f.write(glb)
    print(f"저장: {a.out}  삼각형 {out_faces:,}개, {len(glb) / 1e6:.1f} MB")
    print(f"  배치(extras.car_layout): 문 x {layout['doors']['door_rl']['x']} / {layout['doors']['door_fl']['x']}, "
          f"높이 {layout['doors']['door_fl']['z']}, 옆면 y {layout['body_y']}")


# ---------------------------------------------------------------- 배치 정보 (GLB scene extras)
def make_layout(parts, names, wheels, wheel_set, dims):
    """웹(viewer3d.js, admin.js)과 서버(db.py 샘플)가 같이 쓰는 차량 좌표계 배치 [mm].
    차량 좌표계: x 앞, y 오른쪽, z 높이 (three.js 로는 X=x, Y=z, Z=y)
    모델에 없는 문·미러는 바퀴·보닛·차체 옆면에서 추정 → 다른 차로 바꾸면 웹에서 확인하고 필요하면 이 함수만 손보기"""
    L, W, H = dims
    r1 = lambda v: round(float(v), 1)
    box = lambda b: {"x": [r1(b[0][0]), r1(b[1][0])], "y": [r1(b[0][2]), r1(b[1][2])], "z": [r1(b[0][1]), r1(b[1][1])]}
    named = {}
    for i, nm in names.items():
        b = parts[i].bounds
        named[nm] = b if nm not in named else np.array([np.minimum(named[nm][0], b[0]), np.maximum(named[nm][1], b[1])])
    # 차체 옆면: 바퀴가 아닌 큰 덩어리의 좌우 끝
    big = [p for i, p in enumerate(parts) if i not in wheel_set and len(p.faces) > 1000 and p.extents[2] < W * 0.95]
    body_y = [r1(min(p.bounds[0][2] for p in big)), r1(max(p.bounds[1][2] for p in big))]
    sill = min(p.bounds[0][1] for p in big if p.extents[0] > L * 0.5)  # 긴 옆 차체의 아래 끝
    wb = [parts[w].bounds for w in wheels]
    wx = sorted({round(float((b[0][0] + b[1][0]) / 2)) for b in wb})
    rear_x, front_x = wx[0], wx[-1]
    R = float(np.mean([(b[1][1] - b[0][1]) / 2 for b in wb]))
    x0 = rear_x + R + 2
    x1 = named["bonnet"][0][0] - 2 if "bonnet" in named else front_x - R - 2
    belt = named["bonnet"][1][1] if "bonnet" in named else H * 0.65
    z = [r1(sill + 4), r1(belt - 4)]
    xm = (x0 + x1) / 2
    doors = {}
    for side, y in (("l", body_y[0]), ("r", body_y[1])):
        doors[f"door_r{side}"] = {"x": [r1(x0), r1(xm - 1)], "y": y, "z": z}
        doors[f"door_f{side}"] = {"x": [r1(xm + 1), r1(x1)], "y": y, "z": z}
    mirrors = {f"mirror_{s}": {"x": r1(x1 - 6), "y": r1(y), "z": r1(belt + 6), "size": [16, 9, 14]}
               for s, y in (("l", body_y[0] - 7), ("r", body_y[1] + 7))}
    return {"units": "mm", "L": r1(L), "W": r1(W), "H": r1(H), "body_y": body_y,
            "parts": {nm: box(b) for nm, b in named.items()}, "doors": doors, "mirrors": mirrors}


def set_scene_extras(glb, extras):
    """GLB 의 JSON 덩어리에 scenes[0].extras 추가 (three.js GLTFLoader → gltf.scene.userData)"""
    import json
    import struct
    jlen = struct.unpack_from("<I", glb, 12)[0]
    j = json.loads(glb[20:20 + jlen])
    j["scenes"][j.get("scene", 0)].setdefault("extras", {}).update(extras)
    js = json.dumps(j, separators=(",", ":"), ensure_ascii=False).encode()
    js += b" " * (-len(js) % 4)
    rest = glb[20 + jlen:]
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + len(rest)) + struct.pack("<II", len(js), 0x4E4F534A) + js + rest
    return out


if __name__ == "__main__":
    main()
