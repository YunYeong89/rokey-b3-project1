/* 관리자·고객 공용 3D 차량 뷰어 (three.js)
 *
 * 3D 모델
 *  static/models/car.glb 가 있으면 그걸 불러오고, 없으면 상자로 만든 임시 차를 그린다.
 *  GLB 준비 규칙 (모델링 담당에게 전달)
 *   - 단위 mm, 위쪽이 +Y (glTF 기본)
 *   - 원점 = 차량 좌표계 원점(뒤 왼쪽 모서리 바닥), +X = 차 앞쪽, +Z = 차 오른쪽
 *   - 교체 부품은 따로 분리된 메쉬로, 이름을 bonnet / mirror_l / mirror_r 로
 *  → 로봇이 찍은 손상 좌표 (x, y) 를 그대로 3D 위치 (X=x, Z=y) 에 올릴 수 있다.
 */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const CAR = { L: 260, W: 110 };
const MODEL_URL = "/static/models/car.glb";
const PART_CODES = ["bonnet", "mirror_l", "mirror_r"];
export const COLORS = {
  body: 0x9fb3c8, part: 0xc3d0dd, glass: 0x3a4a5c, tire: 0x2a2f36,
  ok: 0x2e9d6a, dent: 0xd64545, failed: 0xd64545, missing: 0x7c4dbd, requested: 0xe0a526, suspect: 0xe0a526, replaced: 0x2e9d6a,
};
const VIEWS = { iso: [260, 230, 300], top: [0, 560, 1], front: [480, 120, 0], side: [0, 140, 480] };

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
  scene.add(new THREE.GridHelper(600, 30, 0xc9d3dd, 0xdde4ea));

  const carRoot = new THREE.Group();            // 차량 좌표계 → 화면 중앙으로
  carRoot.position.set(-CAR.L / 2, 0, -CAR.W / 2);
  scene.add(carRoot);
  const markers = new THREE.Group();            // 손상 표시
  const pointsGroup = new THREE.Group();        // 실시간 측정 점
  carRoot.add(markers, pointsGroup);
  let model = null;
  const parts = {};                             // bonnet / mirror_l / mirror_r → Mesh(또는 Group)
  const baseMaterial = new Map();

  function setView(name) {
    const [x, y, z] = VIEWS[name];
    camera.position.set(x, y, z);
    controls.target.set(0, 25, 0);
    controls.update();
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
  renderer.setAnimationLoop(() => {
    const t = (performance.now() - t0) / 1000;
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
      const y = surfaceY(f.x, f.y);
      const dot = new THREE.Mesh(new THREE.SphereGeometry(4.5, 20, 14), new THREE.MeshBasicMaterial({ color: COLORS.dent }));
      dot.position.set(f.x, y + 2, f.y);
      const ring = new THREE.Mesh(new THREE.RingGeometry(9, 11, 40), new THREE.MeshBasicMaterial({ color: COLORS.dent, side: THREE.DoubleSide }));
      ring.rotation.x = -Math.PI / 2; ring.position.set(f.x, y + 1.5, f.y);
      markers.add(dot, ring);
    }
  }

  // ---------------------------------------------------------------- 측정 점
  const pointGeo = { coarse: new THREE.SphereGeometry(2.2, 10, 8), fine: new THREE.SphereGeometry(1.4, 8, 6) };
  const pointMat = Object.fromEntries(["ok", "suspect", "dent"].map((k) => [k, new THREE.MeshBasicMaterial({ color: COLORS[k] })]));
  function addPoint(p) {
    const m = new THREE.Mesh(p.fine ? pointGeo.fine : pointGeo.coarse, pointMat[p.state] || pointMat.ok);
    m.position.set(p.x, surfaceY(p.x, p.y) + 1.5, p.y);
    pointsGroup.add(m);
  }
  function clearPoints() { pointsGroup.clear(); }

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

  return { load, paint, addPoint, clearPoints, setView, resize };
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
  // 바퀴
  const wheelGeo = new THREE.CylinderGeometry(14, 14, 12, 24);
  wheelGeo.rotateX(Math.PI / 2);
  for (const x of [50, 210]) for (const z of [-2, 112]) {
    const w = new THREE.Mesh(wheelGeo, mat(COLORS.tire)); w.position.set(x, 14, z); g.add(w);
  }
  return g;
}
