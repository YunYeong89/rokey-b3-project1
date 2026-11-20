/* 관리자·고객 공용 3D 차량 뷰어 (three.js)
 *
 * 3D 모델
 *  static/models/car.glb (실제 차량, tools/car_to_glb.py) 를 먼저 불러오고, 없으면 차량 모델 코드(예: sedan_1_18)의
 *  앞부분으로 static/models/<sedan|suv|sports>.glb (로봇 쪽 rokey 차량 모델, --web 시작 때 다시 생성),
 *  그것도 없으면 상자로 만든 임시 차를 그린다. 차량을 바꾸면 다시 불러오고, 화면 크기는 불러온 모델에 맞춘다.
 *  GLB 준비 규칙
 *   - 단위 mm, 위쪽이 +Y (glTF 기본)
 *   - 원점 = 차량 좌표계 원점(뒤 왼쪽 모서리 바닥), +X = 차 앞쪽, +Z = 차 오른쪽
 *   - 교체 부품은 따로 분리된 메쉬로, 이름을 bonnet / mirror_l / mirror_r / door_fl / door_fr / door_rl / door_rr 로
 *     (STL 은 tools/car_to_glb.py 로 변환 → README "실제 3D 모델 바꾸는 법")
 *  → 로봇이 찍은 손상 좌표 (x, y) 를 그대로 3D 위치 (X=x, Z=y) 에 올릴 수 있다.
 *    높이 z 가 있으면 (문처럼 옆면) 그 높이에서 옆으로, 없으면 (보닛) 위에서 표면을 찾아 붙인다.
 *
 * 차 크기·배치 (layout)
 *  L·W·H 는 GLB 의 bounding box 에서 읽는다 → 모델을 바꿔도 코드 수정 없음.
 *  부품·문·미러 위치는 GLB scene extras 의 car_layout (car_to_glb.py 가 기록) → 없으면 이름 붙은 메쉬 범위 + 임시 모델 비율.
 *  모델에 없는 교체 부품(문·미러 등)은 layout 위치에 반투명 상자로 대신 표시 (상태가 있을 때만 보임).
 */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

/** 임시 모델(buildPlaceholderCar)의 배치. 차량 좌표계 mm: x 앞, y 오른쪽, z 높이 (three.js 로는 X=x, Y=z, Z=y) */
const PLACEHOLDER_LAYOUT = {
  L: 260, W: 110, H: 69, body_y: [0, 110],
  parts: {
    trunk: { x: [4, 66], y: [6, 104], z: [40, 40] },
    roof: { x: [76, 160], y: [16, 94], z: [67, 70] },
    bonnet: { x: [176, 252], y: [8, 102], z: [39, 44] },
  },
  doors: {
    door_rl: { x: [72, 118], y: 0, z: [13, 38] }, door_fl: { x: [120, 165], y: 0, z: [13, 38] },
    door_rr: { x: [72, 118], y: 110, z: [13, 38] }, door_fr: { x: [120, 165], y: 110, z: [13, 38] },
  },
  mirrors: { mirror_l: { x: 158, y: -7, z: 46, size: [16, 9, 14] }, mirror_r: { x: 158, y: 117, z: 46, size: [16, 9, 14] } },
};
const CAR = { L: 260, W: 110, H: 69 };          // load() 에서 모델 bounding box 로 바뀜
const MODEL_DIR = "/static/models/";            // car.glb (실제 차량), 없으면 <sedan|suv|sports>.glb (rokey 차종)
const PART_CODES = ["bonnet", "mirror_l", "mirror_r", "door_fl", "door_fr", "door_rl", "door_rr"];
export const COLORS = {
  body: 0x9fb3c8, part: 0xc3d0dd, glass: 0x3a4a5c, tire: 0x2a2f36,
  ok: 0x2e9d6a, dent: 0xd64545, crack: 0xd64545, failed: 0xd64545, missing: 0x7c4dbd, requested: 0xe0a526, suspect: 0xe0a526, replaced: 0x2e9d6a,
};
const VIEWS = { iso: [260, 205, 300], top: [0, 535, 1], front: [480, 95, 0], side: [0, 115, 480] };   // 바라보는 점에서 카메라까지
// 로봇팔을 켜면 (관리자 화면) 로봇까지 보이게 넓힘: 바라보는 점(월드), 거리 배율, 최대 줌아웃
// target(C) = 바라보는 점 (C = 차 크기 {L, W, H}, 화면 중앙 = 차 가운데 바닥), scale = VIEWS 거리 배율
// robot: 차를 크게 보이게 차 위쪽(로봇 쪽으로 조금)을 가깝게 봄. 로봇 전체는 휠로 축소 (maxDist 까지)
const EXTENT = { car: { target: (C) => [0, C.H * 0.36, 0], scale: 1, maxDist: 900, grid: 1 },
                 robot: { target: (C) => [0, C.H * 1.3, -C.W * 0.9], scale: 2.0, maxDist: 2400, grid: 2.5 } };

/**
 * @param {HTMLElement} host   뷰어를 넣을 요소
 * @param {object} opt
 *   msgEl        안내 문구 요소 (모델 로딩 상태)
 *   onPartClick  (code) => void — 부품을 클릭(드래그 아님)했을 때
 */
export function createCarViewer(host, { msgEl = null, onPartClick = null } = {}) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setClearColor(0xf3f6f9);
  host.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(38, 1, 1, 5000);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(0, 25, 0);
  controls.maxPolarAngle = Math.PI * 0.49;
  controls.minDistance = 150; controls.maxDistance = 900;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(200, 400, 250);
  scene.add(sun);
  const grid = new THREE.GridHelper(600, 30, 0xc9d3dd, 0xdde4ea);
  scene.add(grid);
  let extent = EXTENT.car;

  const carRoot = new THREE.Group();            // 차량 좌표계 → 화면 중앙으로
  scene.add(carRoot);
  const proxies = new THREE.Group();            // 모델에 없는 부품 대신 보여 줄 반투명 상자
  carRoot.add(proxies);
  const markers = new THREE.Group();            // 손상 표시
  const pointsGroup = new THREE.Group();        // 실시간 측정 점
  carRoot.add(markers, pointsGroup);
  let model = null;
  let layout = PLACEHOLDER_LAYOUT;
  let sizeK = 1;                                // 임시 모델(260×110×69) 대비 크기 배율 (대각선 기준) → 카메라·그리드
  const parts = {};                             // PART_CODES → Mesh(또는 Group)
  const baseMaterial = new Map();

  function setView(name) {
    const [x, y, z] = VIEWS[name], k = extent.scale * sizeK;   // 시점 거리는 차 크기에 비례 (fitToCar)
    const [tx, ty, tz] = extent.target(CAR);
    controls.target.set(tx, ty, tz);
    camera.position.set(tx + x * k, ty + y * k, tz + z * k);
    controls.update();
  }
  /** 로봇팔 표시에 맞춰 시야 넓히기/되돌리기 */
  function setRobotExtent(on) {
    extent = on ? EXTENT.robot : EXTENT.car;
    controls.minDistance = 150 * sizeK;
    controls.maxDistance = extent.maxDist * sizeK;
    grid.scale.setScalar(extent.grid * sizeK);
    setView("iso");
  }
  /** 차 크기가 정해지면 (load) 화면 배치 맞추기 */
  function fitToCar() {
    carRoot.position.set(-CAR.L / 2, 0, -CAR.W / 2);
    sizeK = Math.hypot(CAR.L, CAR.W, CAR.H) / Math.hypot(260, 110, 69);
    sun.position.set(200 * sizeK, 400 * sizeK, 250 * sizeK);
    camera.far = 5000 * sizeK; camera.updateProjectionMatrix();
    setRobotExtent(extent === EXTENT.robot);
  }

  function resize() {
    const w = host.clientWidth, h = host.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(host);

  const t0 = performance.now();
  const frameHooks = [];                        // 매 프레임 호출 (dt [s]) — 로봇팔 등
  let tPrev = t0;
  renderer.setAnimationLoop(() => {
    const now = performance.now();
    const t = (now - t0) / 1000, dt = (now - tPrev) / 1000;
    tPrev = now;
    for (const fn of frameHooks) {
      try { fn(dt); } catch (e) { console.error(e); }
    }
    // 진행 중이면 교체 대상 부품을 은은하게 깜빡임
    for (const obj of Object.values(parts)) {
      const pulse = obj.userData.pulse ? 0.35 + 0.35 * Math.sin(t * 4) : 0;
      eachMesh(obj, (m) => { if (m.material.emissive) m.material.emissiveIntensity = pulse; });
    }
    if (!host.clientWidth) return;              // 숨긴 뷰어 (관리자 화면): 계산만 쓰고 그리지 않음
    controls.update();
    renderer.render(scene, camera);
  });

  // ---------------------------------------------------------------- 모델
  let loaded = null;
  async function load(vehicleModel = "sedan") {
    const type = vehicleType(vehicleModel);
    if (model && loaded === type) return;
    loaded = type;
    let glb = null;
    for (const url of [`${MODEL_DIR}car.glb`, `${MODEL_DIR}${type}.glb`]) {
      try {
        const head = await fetch(url, { method: "HEAD" });
        if (head.ok) { glb = (await new GLTFLoader().loadAsync(url)).scene; break; }
      } catch (e) { console.warn(`${url} 불러오기 실패`, e); }
    }
    if (model) carRoot.remove(model);           // 차량을 바꾸면 다시 불러옴
    for (const code of Object.keys(parts)) delete parts[code];
    baseMaterial.clear();
    proxies.clear();
    model = glb || buildPlaceholderCar();
    if (!glb) Object.assign(CAR, { L: PLACEHOLDER_LAYOUT.L, W: PLACEHOLDER_LAYOUT.W, H: PLACEHOLDER_LAYOUT.H });
    carRoot.add(model);
    readLayout(glb);
    fitToCar();
    const absent = [];
    for (const code of PART_CODES) {
      let obj = model.getObjectByName(code);
      if (!obj) { absent.push(code); obj = buildProxy(code); if (!obj) continue; }
      eachMesh(obj, (m) => {
        m.material = m.material.clone();          // 부품마다 색을 따로 바꾸려고 복제
        baseMaterial.set(m, { color: m.material.color.clone(), opacity: m.material.opacity, transparent: m.material.transparent });
        m.material.emissive = new THREE.Color(0x000000);
      });
      obj.userData.partCode = code;
      parts[code] = obj;
    }
    if (absent.length) console.warn(`[viewer3d] 모델에 없는 부품: ${absent.join(", ")} → 손상·교체는 반투명 상자로 대신 표시`);
    if (msgEl) {
      msgEl.textContent = glb ? "" : `임시 모델 (static/models/car.glb · ${type}.glb 가 없음)`;
      msgEl.classList.toggle("small-msg", !glb);
    }
    resize();
    setView("iso");
  }

  /** 차 크기 (bounding box) + 배치 (GLB extras.car_layout → 없으면 메쉬 이름·임시 모델 비율) */
  function readLayout(glb) {
    model.updateMatrixWorld(true);
    const bb = new THREE.Box3().setFromObject(model);
    if (glb) {
      Object.assign(CAR, { L: bb.max.x, W: bb.max.z, H: bb.max.y });
      if (Math.max(...[bb.min.x, bb.min.y, bb.min.z].map(Math.abs)) > 0.02 * CAR.L)
        console.warn("[viewer3d] car.glb 원점이 뒤·왼쪽·바닥 모서리가 아님", bb.min);
    }
    const ex = glb && glb.userData.car_layout;
    if (ex) { layout = { ...ex, L: CAR.L, W: CAR.W, H: CAR.H }; return; }
    if (!glb) { layout = PLACEHOLDER_LAYOUT; return; }
    // extras 없는 GLB: 임시 모델 배치를 크기 비율로 늘리고, 이름 붙은 메쉬가 있으면 그 범위
    const P = PLACEHOLDER_LAYOUT, sx = CAR.L / P.L, sy = CAR.W / P.W, sz = CAR.H / P.H;
    const sc = (b) => ({ x: b.x.map((v) => v * sx), y: typeof b.y === "number" ? b.y * sy : b.y.map((v) => v * sy), z: Array.isArray(b.z) ? b.z.map((v) => v * sz) : b.z * sz });
    layout = {
      L: CAR.L, W: CAR.W, H: CAR.H, body_y: [0, CAR.W],
      parts: Object.fromEntries(Object.entries(P.parts).map(([k, b]) => [k, sc(b)])),
      doors: Object.fromEntries(Object.entries(P.doors).map(([k, b]) => [k, sc(b)])),
      mirrors: Object.fromEntries(Object.entries(P.mirrors).map(([k, m]) => [k, { ...m, x: m.x * sx, y: m.y * sy, z: m.z * sz }])),
    };
    for (const name of ["bonnet", "roof", "trunk"]) {
      const o = model.getObjectByName(name);
      if (!o) continue;
      const b = new THREE.Box3().setFromObject(o);
      layout.parts[name] = { x: [b.min.x, b.max.x], y: [b.min.z, b.max.z], z: [b.min.y, b.max.y] };
    }
  }

  /** 모델에 없는 부품 자리에 반투명 상자 (평소엔 숨김, paint 에서 상태가 있으면 보임) */
  function buildProxy(code) {
    let g;
    const T = 4;                                // 문 상자 두께 [mm] (옆면 바깥쪽으로)
    const d = layout.doors?.[code], mi = layout.mirrors?.[code];
    if (d) {
      const out = d.y < CAR.W / 2 ? -1 : 1;
      g = new THREE.BoxGeometry(d.x[1] - d.x[0], d.z[1] - d.z[0], T);
      g.translate((d.x[0] + d.x[1]) / 2, (d.z[0] + d.z[1]) / 2, d.y + out * T / 2);
    } else if (mi) {
      const [sx, sz, sy] = mi.size || [16, 9, 14];
      g = new THREE.BoxGeometry(sx, sz, sy);
      g.translate(mi.x, mi.z, mi.y);
    } else return null;
    const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: COLORS.part, transparent: true, opacity: 0.45, depthWrite: false, roughness: 0.6 }));
    const edge = new THREE.LineSegments(new THREE.EdgesGeometry(g), new THREE.LineBasicMaterial({ color: COLORS.part }));
    const obj = new THREE.Group();
    obj.add(m, edge);
    obj.name = code; obj.visible = false;
    obj.userData.proxy = true;
    proxies.add(obj);
    return obj;
  }

  // ---------------------------------------------------------------- 표면 높이
  const ray = new THREE.Raycaster();
  /** 측정 점·손상 위치 → 표면 위 점과 바깥 법선. 높이 z 가 있으면 옆면(y 로 왼쪽/오른쪽), 없으면 윗면 */
  function onSurface(p) {
    if (p.z == null) return surfaceAt("top", p.x, p.y);
    const s = surfaceAt(p.y < CAR.W / 2 ? "left" : "right", p.x, p.z);
    if (!s.hit) s.pos.set(p.x, p.z, p.y);          // 표면을 못 찾으면 보낸 좌표 그대로
    return s;
  }

  // ---------------------------------------------------------------- 상태 칠하기
  /** partStates: {code: "dent"|"missing"|...}, findings: job.findings, running: 진행 중이면 깜빡임 */
  function paint(partStates = {}, findings = [], running = false) {
    for (const [code, obj] of Object.entries(parts)) {
      const s = partStates[code];
      const has = s && COLORS[s] !== undefined;
      if (obj.userData.proxy) {                   // 모델에 없는 부품: 상태가 있을 때만 반투명 상자
        obj.visible = has;
        if (has) obj.traverse((o) => { if (o.material) o.material.color.set(COLORS[s]); });
        if (has) eachMesh(obj, (m) => m.material.emissive.set(COLORS[s]));
        obj.userData.pulse = running && has && s !== "replaced";
        continue;
      }
      eachMesh(obj, (m) => {
        const base = baseMaterial.get(m);
        m.material.color.copy(has ? new THREE.Color(COLORS[s]) : base.color);
        m.material.emissive.set(has ? COLORS[s] : 0x000000);
        const ghost = s === "missing";
        m.material.transparent = ghost || base.transparent;
        m.material.opacity = ghost ? 0.35 : base.opacity;
      });
      obj.userData.pulse = running && !!s && s !== "replaced";
    }
    clearGroup(markers);
    for (const f of findings) {
      if (f.decision !== "dent" || f.x == null) continue;
      const dot = new THREE.Mesh(new THREE.SphereGeometry(4.5, 20, 14), new THREE.MeshBasicMaterial({ color: COLORS.dent }));
      const ring = new THREE.Mesh(new THREE.RingGeometry(9, 11, 40), new THREE.MeshBasicMaterial({ color: COLORS.dent, side: THREE.DoubleSide }));
      const s = onSurface(f);                     // 표면 위, 링은 표면 방향으로 눕힘
      dot.position.copy(s.pos).addScaledVector(s.normal, 2);
      ring.position.copy(s.pos).addScaledVector(s.normal, 1.5);
      ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), s.normal);
      markers.add(dot, ring);
    }
  }

  // ---------------------------------------------------------------- 측정 점
  const pointGeo = { coarse: new THREE.SphereGeometry(2.2, 10, 8), fine: new THREE.SphereGeometry(1.4, 8, 6) };
  const pointMat = Object.fromEntries(["ok", "suspect", "dent"].map((k) => [k, new THREE.MeshBasicMaterial({ color: COLORS[k] })]));
  function addPoint(p) {
    const m = new THREE.Mesh(p.fine ? pointGeo.fine : pointGeo.coarse, pointMat[p.state] || pointMat.ok);
    const s = onSurface(p);
    m.position.copy(s.pos).addScaledVector(s.normal, p.fine ? 1.2 : 1.8);
    pointsGroup.add(m);
  }
  function clearPoints() { pointsGroup.clear(); }

  // ---------------------------------------------------------------- 표면 찾기 (연속 스캔용)
  /** 차량 표면 점과 바깥 법선 (차량 로컬 = carRoot 좌표, Y 위쪽)
   *  face "top": (u, v) = 차량 (x, y), 위에서 아래로 / "left"·"right": (u, v) = 차량 (x, z), 옆에서 차 쪽으로 */
  const FACE_RAY = { top: [0, -1, 0], left: [0, 0, 1], right: [0, 0, -1] };
  function surfaceAt(face, u, v) {
    const d = new THREE.Vector3(...FACE_RAY[face]);
    const local = face === "top" ? new THREE.Vector3(u, 1000, v) : new THREE.Vector3(u, v, face === "left" ? -1000 : CAR.W + 1000);
    const fallback = face === "top" ? new THREE.Vector3(u, CAR.H * 0.65, v) : new THREE.Vector3(u, v, face === "left" ? 0 : CAR.W);
    if (model) {
      carRoot.updateMatrixWorld();
      ray.set(carRoot.localToWorld(local.clone()), d);
      const hit = ray.intersectObject(model, true)[0];
      if (hit) {
        const n = hit.face.normal.clone().transformDirection(hit.object.matrixWorld);
        if (n.dot(d) > 0) n.negate();
        return { pos: carRoot.worldToLocal(hit.point.clone()), normal: n, hit: true };
      }
    }
    return { pos: fallback, normal: d.clone().negate(), hit: false };
  }

  // ---------------------------------------------------------------- 부품 클릭
  if (onPartClick) {
    let down = null;
    const el = renderer.domElement;
    el.addEventListener("pointerdown", (e) => { down = { x: e.clientX, y: e.clientY }; });
    el.addEventListener("pointerup", (e) => {
      if (!down || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 5) return;   // 드래그(회전)는 무시
      const r = el.getBoundingClientRect();
      const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
      ray.setFromCamera(ndc, camera);
      const hit = ray.intersectObjects(Object.values(parts), true)[0];
      let o = hit && hit.object;
      while (o && !o.userData.partCode) o = o.parent;
      if (o) onPartClick(o.userData.partCode);
    });
  }

  return {
    load, paint, addPoint, clearPoints, setView, resize,
    scene, carRoot, surfaceAt, setRobotExtent, onFrame: (fn) => frameHooks.push(fn),
    /** 차 크기 {L, W, H} [mm] (load 후 모델 기준) */
    dims: () => ({ ...CAR }),
    size: () => ({ ...CAR }),                   // dims 와 같음 (admin.js 데모 배율)
    /** 차량 좌표계 배치 (PLACEHOLDER_LAYOUT 과 같은 모양) */
    layout: () => layout,
  };
}

/** 차량 모델 코드 → 3D 모델 종류: "sedan_1_18" → "sedan" */
export function vehicleType(model) { return String(model || "sedan").split("_")[0]; }

// ================================================================ 유틸
function eachMesh(obj, fn) { obj.traverse((o) => { if (o.isMesh) fn(o); }); }
function clearGroup(g) {
  for (const c of g.children) { c.geometry?.dispose(); c.material?.dispose(); }
  g.clear();
}
function mat(color, opts = {}) { return new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.15, ...opts }); }
function box(w, h, d, color, x, y, z, name) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat(color));
  m.position.set(x, y, z); if (name) m.name = name;
  return m;
}

function buildPlaceholderCar() {
  const g = new THREE.Group();
  const lift = 10;
  // 차체 (뒤 x=0 → 앞 x=260), 폭 z=0..110
  g.add(box(260, 30, 110, COLORS.body, 130, lift + 15, 55));
  // 지붕·유리
  g.add(box(95, 26, 84, COLORS.glass, 118, lift + 30 + 13, 55));
  g.add(box(85, 3, 76, COLORS.body, 118, lift + 30 + 27, 55));
  // 교체 부품
  const bonnet = box(77, 3, 88, COLORS.part, 213.5, lift + 31.5, 55, "bonnet");
  bonnet.rotation.z = -0.06;                    // 앞쪽으로 살짝 내려감
  g.add(bonnet);
  g.add(box(16, 9, 14, COLORS.part, 158, lift + 36, -7, "mirror_l"));
  g.add(box(16, 9, 14, COLORS.part, 158, lift + 36, 117, "mirror_r"));
  // 문 4개: 옆면에 붙은 얇은 판 (뒷문 x 72~118, 앞문 x 120~165, 높이 13~38, 왼쪽 y=0 / 오른쪽 y=110)
  for (const [name, x, z] of [["door_rl", 95, -1], ["door_fl", 142.5, -1], ["door_rr", 95, 111], ["door_fr", 142.5, 111]]) {
    g.add(box(name.startsWith("door_r") ? 46 : 45, 25, 2, COLORS.part, x, lift + 15.5, z, name));
  }
  // 바퀴
  const wheelGeo = new THREE.CylinderGeometry(14, 14, 12, 24);
  wheelGeo.rotateX(Math.PI / 2);
  for (const x of [50, 210]) for (const z of [-2, 112]) {
    const w = new THREE.Mesh(wheelGeo, mat(COLORS.tire)); w.position.set(x, 14, z); g.add(w);
  }
  return g;
}
