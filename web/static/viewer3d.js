/* 관리자·고객 공용 3D 차량 뷰어 (three.js)
 *
 * 3D 모델
 *  static/models/car.glb 가 있으면 그걸 불러오고, 없으면 상자로 만든 임시 차를 그린다.
 *  GLB 준비 규칙 (모델링 담당에게 전달)
 *   - 단위 mm, 위쪽이 +Y (glTF 기본)
 *   - 원점 = 차량 좌표계 원점(뒤 왼쪽 모서리 바닥), +X = 차 앞쪽, +Z = 차 오른쪽
 *   - 교체 부품은 따로 분리된 메쉬로, 이름을 bonnet / mirror_l / mirror_r / door_fl / door_fr / door_rl / door_rr 로
 *  → 로봇이 찍은 손상 좌표 (x, y) 를 그대로 3D 위치 (X=x, Z=y) 에 올릴 수 있다.
 *    높이 z 가 있으면 (문처럼 옆면) 3D 높이 Y=z 에 그대로, 없으면 (보닛) 위에서 표면을 찾아 올린다.
 */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const CAR = { L: 260, W: 110 };
const MODEL_URL = "/static/models/car.glb";
const PART_CODES = ["bonnet", "mirror_l", "mirror_r", "door_fl", "door_fr", "door_rl", "door_rr"];
export const COLORS = {
  body: 0x9fb3c8, part: 0xc3d0dd, glass: 0x3a4a5c, tire: 0x2a2f36,
  ok: 0x2e9d6a, dent: 0xd64545, failed: 0xd64545, missing: 0x7c4dbd, requested: 0xe0a526, suspect: 0xe0a526, replaced: 0x2e9d6a,
};
const VIEWS = { iso: [260, 205, 300], top: [0, 535, 1], front: [480, 95, 0], side: [0, 115, 480] };   // 바라보는 점에서 카메라까지
// 로봇팔을 켜면 (관리자 화면) 로봇까지 보이게 넓힘: 바라보는 점(월드), 거리 배율, 최대 줌아웃
const EXTENT = { car: { target: [0, 25, 0], scale: 1, maxDist: 900, grid: 1 },
                 robot: { target: [0, 230, -210], scale: 3.0, maxDist: 2400, grid: 2.5 } };

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
  carRoot.position.set(-CAR.L / 2, 0, -CAR.W / 2);
  scene.add(carRoot);
  const markers = new THREE.Group();            // 손상 표시
  const pointsGroup = new THREE.Group();        // 실시간 측정 점
  carRoot.add(markers, pointsGroup);
  let model = null;
  const parts = {};                             // PART_CODES → Mesh(또는 Group)
  const baseMaterial = new Map();

  function setView(name) {
    const [x, y, z] = VIEWS[name], [tx, ty, tz] = extent.target, k = extent.scale;
    controls.target.set(tx, ty, tz);
    camera.position.set(tx + x * k, ty + y * k, tz + z * k);
    controls.update();
  }
  /** 로봇팔 표시에 맞춰 시야 넓히기/되돌리기 */
  function setRobotExtent(on) {
    extent = on ? EXTENT.robot : EXTENT.car;
    controls.maxDistance = extent.maxDist;
    grid.scale.setScalar(extent.grid);
    setView("iso");
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
    controls.update();
    renderer.render(scene, camera);
  });

  // ---------------------------------------------------------------- 모델
  async function load() {
    let glb = null;
    try {
      const head = await fetch(MODEL_URL, { method: "HEAD" });
      if (head.ok) glb = (await new GLTFLoader().loadAsync(MODEL_URL)).scene;
    } catch (e) { console.warn("car.glb 불러오기 실패, 임시 모델 사용", e); }
    model = glb || buildPlaceholderCar();
    carRoot.add(model);
    for (const code of PART_CODES) {
      const obj = model.getObjectByName(code);
      if (!obj) { console.warn(`모델에 ${code} 메쉬가 없음`); continue; }
      eachMesh(obj, (m) => {
        m.material = m.material.clone();          // 부품마다 색을 따로 바꾸려고 복제
        baseMaterial.set(m, { color: m.material.color.clone(), opacity: m.material.opacity, transparent: m.material.transparent });
        m.material.emissive = new THREE.Color(0x000000);
      });
      obj.userData.partCode = code;
      parts[code] = obj;
    }
    if (msgEl) {
      msgEl.textContent = glb ? "" : "임시 모델 (static/models/car.glb 를 넣으면 실제 모델로 바뀜)";
      msgEl.classList.toggle("small-msg", !glb);
    }
    resize();
    setView("iso");
  }

  // ---------------------------------------------------------------- 표면 높이
  const ray = new THREE.Raycaster();
  function surfaceY(x, z) {                     // 차량 좌표 (x, z) 위에서 아래로 쏴서 표면 높이 찾기
    if (!model) return 45;
    ray.set(new THREE.Vector3(x - CAR.L / 2, 1000, z - CAR.W / 2), new THREE.Vector3(0, -1, 0));
    const hit = ray.intersectObject(model, true)[0];
    return hit ? hit.point.y : 45;
  }

  // ---------------------------------------------------------------- 상태 칠하기
  /** partStates: {code: "dent"|"missing"|...}, findings: job.findings, running: 진행 중이면 깜빡임 */
  function paint(partStates = {}, findings = [], running = false) {
    for (const [code, obj] of Object.entries(parts)) {
      const s = partStates[code];
      const has = s && COLORS[s] !== undefined;
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
      if (f.z != null) {                          // 옆면(문): 링을 세워서 바깥쪽에
        const out = f.y < CAR.W / 2 ? -1 : 1;
        dot.position.set(f.x, f.z, f.y + out * 3);
        ring.position.set(f.x, f.z, f.y + out * 2.5);
      } else {                                    // 윗면(보닛): 표면 위에 눕혀서
        const y = surfaceY(f.x, f.y);
        dot.position.set(f.x, y + 2, f.y);
        ring.rotation.x = -Math.PI / 2; ring.position.set(f.x, y + 1.5, f.y);
      }
      markers.add(dot, ring);
    }
  }

  // ---------------------------------------------------------------- 측정 점
  const pointGeo = { coarse: new THREE.SphereGeometry(2.2, 10, 8), fine: new THREE.SphereGeometry(1.4, 8, 6) };
  const pointMat = Object.fromEntries(["ok", "suspect", "dent"].map((k) => [k, new THREE.MeshBasicMaterial({ color: COLORS[k] })]));
  function addPoint(p) {
    const m = new THREE.Mesh(p.fine ? pointGeo.fine : pointGeo.coarse, pointMat[p.state] || pointMat.ok);
    if (p.z != null) m.position.set(p.x, p.z, p.y + (p.y < CAR.W / 2 ? -2.5 : 2.5));   // 옆면 점
    else m.position.set(p.x, surfaceY(p.x, p.y) + 1.5, p.y);
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
    const fallback = face === "top" ? new THREE.Vector3(u, 45, v) : new THREE.Vector3(u, v, face === "left" ? 0 : CAR.W);
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
  };
}

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
