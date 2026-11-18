/* 관리자 3D 뷰어용 두산 M0609 로봇팔 (고객 화면에서는 쓰지 않음)
 *
 * 모델: static/models/m0609/ (white 메쉬 + m0609.json, 출처는 json 의 _source)
 *       static/models/rg2/ (OnRobot RG2 그리퍼, 출처는 SOURCE.md) — 손가락이 탐침 막대를 물고 있음
 * 좌표: 이 파일에서 위치는 "차량 로컬" = viewer3d 의 carRoot 좌표 (three.js, Y 위쪽)
 *        차량 좌표계 (x 앞, y 오른쪽, z 높이) 와는 (x, z, y) 로 바뀜 → carToLocal()
 * 실제 모드: rosbridge 로 /dsr01/joint_states 를 구독해서 관절 각도를 그대로 그린다.
 * 데모 모드: 시뮬레이터가 transit / followPath 로 탐침 끝(볼 중심) 경로를 주면 IK(robot_ik.js, DLS)로 따라간다.
 */
import * as THREE from "three";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { createKinematics } from "./robot_ik.js";

// ================================================================ 설정 상수
/** 로봇 베이스의 위치 (차량 좌표계 mm). yawDeg = 로봇 정면(+X)이 향하는 방향: 0 = 차 앞(+x), 90 = 차 오른쪽(+y)
 *  x, y 가 null 이면 차 크기에서 계산: x = 차 길이 가운데, y = -(차 폭 가운데에서 centerDist 만큼 왼쪽)
 *  centerDist 400 = RG2 + 탐침(툴 약 283 mm), 팔꿈치 위 자세(DEMO_JOINT_LIMITS_DEG)로 보닛·지붕·양쪽 문·미러 모두 닿고
 *  팔·그리퍼가 차에 안 부딪히는 거리 (375~425 에서 확인. 멀면 건너편 문이 안 닿거나 그리퍼가 지붕 모서리에 걸림)
 *  실제 설치 위치를 재면 x, y 에 숫자로 넣기 */
export const ROBOT_BASE_IN_CAR = { x: null, y: null, z: 0, yawDeg: 90, centerDist: 400 };
/** 툴: 플랜지에 OnRobot RG2 그리퍼 (models/rg2/, 출처는 SOURCE.md), 손가락이 탐침 막대를 물고 있음.
 *  fingerRad = finger_joint 각도 (지름 5 mm 막대를 물었을 때, 0.78 = 완전히 닫힘),
 *  tipZ = 그 각도에서 플랜지 면 ~ 손가락 끝 [mm] (URDF 관절식으로 계산) */
export const GRIPPER = { fingerRad: 0.73, tipZ: 233 };
/** 탐침: 손가락 끝에서 stickOut 만큼 튀어나온 막대 + 끝 볼. gripDepth = 손가락 안에 물린 길이 [mm]
 *  → 플랜지 면 ~ 볼 중심 = GRIPPER.tipZ + stickOut (IK 기준점) */
export const PROBE = { stickOut: 50, gripDepth: 22, rodRadius: 2.5, ballRadius: 4 };
/** 실제 로봇 관절 토픽 */
export const ROS_JOINTS = { topic: "/dsr01/joint_states", type: "sensor_msgs/JointState", rateHz: 25, staleMs: 1500 };
/** 홈 자세 [deg] (탐침이 아래를 봄) */
export const HOME_DEG = [0, 0, 90, 0, 90, 0];
/** 데모 IK 가 쓰는 관절 범위 [deg] (J1~J6). URDF 는 J3 말고 ±360° 라 IK 가 팔꿈치 아래·손목 뒤집힌 해로 튀어
 *  불가능해 보이는 자세가 나옴 → 팔꿈치 위(J3 > 0), 어깨 앞쪽, 손목 안 뒤집힘으로 좁힘. 실제 모드(ROS 관절값)에는 안 씀 */
export const DEMO_JOINT_LIMITS_DEG = [[-120, 120], [-60, 75], [10, 150], [-150, 150], [0, 180], [-180, 180]];
/** 데모 동작 속도 등 */
export const MOTION = {
  airSpeed: 520,        // 공중 이동 [mm/s]
  approachSpeed: 150,   // 표면에 붙기 직전 [mm/s]
  turnSec: 0.25,        // 탐침 방향 1 rad 바꾸는 데 [s]
  minMove: 0.15,        // 이동 한 번 최소 시간 [s]
  lift: 22,             // 표면에서 들어 올리는 높이 [mm]
  safeAbove: 66,        // 다른 면으로 옮길 때 지나가는 높이 = 차 높이 + 이 값 [mm]
};
/** 옆면(문)에서 탐침을 수평에서 아래로 기울이는 각도 [deg]. near = 로봇 쪽 면, far = 차 건너편 면.
 *  시뮬레이션으로 고른 값 (툴 약 283 mm, 베이스 centerDist 400, DEMO_JOINT_LIMITS_DEG 기준):
 *  문·미러·지붕·보닛 대표 점 모두 IK 오차 < 1 mm, 팔·그리퍼가 차에 안 부딪힘. near 55~60 / far 61~64 (베이스 ±25 mm) 에서 확인.
 *  near 65 이상은 그리퍼가 차에 걸리고, far 를 크게(수직에 가깝게) 하면 그리퍼가 지붕 모서리에 걸림 */
export const SIDE_TILT_DEG = { near: 57, far: 62 };

const MODEL_DIR = "/static/models/m0609/";
const RG2_DIR = "/static/models/rg2/";
const DEG = Math.PI / 180;
const HOME = HOME_DEG.map((d) => d * DEG);
const DOWN = new THREE.Vector3(0, -1, 0);
export const carToLocal = (x, y, z) => new THREE.Vector3(x, z, y);

/** 툴 모양 (RG2 + 물린 탐침). 플랜지(link_6) 좌표계 (Z = 플랜지 바깥 방향) 에서 만든다.
 *  반환: object(그릴 것), tip(볼 중심), dir(누르는 방향), materials(반투명 전환용), ready(그리퍼 메쉬 로딩 Promise).
 *  ㄱ자 탐침 등으로 바꿀 때는 이 함수만 바꾸면 된다. */
export function buildProbe(cfg = PROBE, grip = GRIPPER) {
  const g = new THREE.Group();
  const rodMat = new THREE.MeshStandardMaterial({ color: 0x4a5560, roughness: 0.4, metalness: 0.5 });
  const ballMat = new THREE.MeshStandardMaterial({ color: 0xf08c2e, roughness: 0.35 });
  const length = grip.tipZ + cfg.stickOut;
  const z0 = grip.tipZ - cfg.gripDepth, rodLen = length - cfg.ballRadius - z0;
  const rod = new THREE.Mesh(new THREE.CylinderGeometry(cfg.rodRadius, cfg.rodRadius, rodLen, 12), rodMat);
  rod.rotation.x = Math.PI / 2; rod.position.z = z0 + rodLen / 2;
  const ball = new THREE.Mesh(new THREE.SphereGeometry(cfg.ballRadius, 16, 12), ballMat);
  ball.position.z = length;
  const rg2 = buildRG2(grip);
  g.add(rg2.object, rod, ball);
  return { object: g, tip: [0, 0, length], dir: [0, 0, 1], materials: [rodMat, ballMat, ...rg2.materials], ready: rg2.ready };
}

/** OnRobot RG2 (URDF 관절 그대로, 손가락은 grip.fingerRad 로 고정). 메쉬 단위 m → ×1000 */
function buildRG2(grip) {
  const M = 1000;
  const frame = (xyz, rpy = [0, 0, 0]) => {
    const f = new THREE.Group();
    f.position.set(xyz[0] * M, xyz[1] * M, xyz[2] * M);
    f.rotation.set(rpy[0], rpy[1], rpy[2], "ZYX");                   // URDF rpy = Rz·Ry·Rx
    return f;
  };
  const turnX = (parent, a) => { const r = new THREE.Group(); r.rotation.x = a; parent.add(r); return r; };
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0xd4d8dc, roughness: 0.5, metalness: 0.1 });
  const fingerMat = new THREE.MeshStandardMaterial({ color: 0x2b2f33, roughness: 0.6 });
  const tool0 = frame([0, 0, 0], [Math.PI, -Math.PI / 2, 0]);        // link_6 → tool0
  const base = frame([0, 0, 0], [Math.PI / 2, 0, Math.PI / 2]);      // tool0 → rg2_base_link
  tool0.add(base);
  const th = grip.fingerRad, slots = [[base, "base_link", bodyMat]];
  for (const [s, rz] of [[-1, 0], [1, Math.PI]]) {                 // 왼쪽 / 오른쪽 손가락 (mimic 관절)
    const outer = frame([0, s * 0.017178, 0.125797], [0, 0, rz]); base.add(outer);
    const inner = frame([0, s * 0.007678, 0.142297], [0, 0, rz ? -Math.PI : 0]); base.add(inner);
    const outerR = turnX(outer, -th), innerR = turnX(inner, -th);
    const tip = frame([0, -0.039592, 0.038177]); outerR.add(tip);
    slots.push([outerR, "outer_knuckle", bodyMat], [innerR, "inner_knuckle", bodyMat], [turnX(tip, th), "inner_finger", fingerMat]);
  }
  const loader = new STLLoader(), geo = {};
  const ready = Promise.all([...new Set(slots.map((x) => x[1]))].map(async (n) => {
    geo[n] = await loader.loadAsync(`${RG2_DIR}${n}.stl`);
    geo[n].scale(M, M, M); geo[n].computeVertexNormals();
  })).then(() => { for (const [parent, n, mat] of slots) parent.add(new THREE.Mesh(geo[n], mat)); });
  return { object: tool0, materials: [bodyMat, fingerMat], ready };
}

/**
 * @param viewer  createCarViewer() 결과 (carRoot, onFrame 사용)
 * @param opt.msgEl 로딩 문구 표시 요소
 */
export function createRobotArm(viewer, { msgEl = null } = {}) {
  const root = new THREE.Group();               // 차량 로컬에서 베이스 위치·방향
  const zup = new THREE.Group();                // ROS(Z 위) → three(Y 위)
  zup.rotation.x = -Math.PI / 2;
  root.add(zup);
  root.visible = false;
  viewer.carRoot.add(root);
  let baseToLocal, localToBase, safeY;          // placeBase() 에서 차 크기로 정함
  const basePos = new THREE.Vector3();
  /** 차 크기(viewer.dims)에 맞춰 베이스 위치·지나가는 높이 정하기 (차 모델을 불러온 뒤) */
  function placeBase() {
    const { L, W, H } = viewer.dims(), B = ROBOT_BASE_IN_CAR;
    const bx = B.x ?? L / 2, by = B.y ?? W / 2 - B.centerDist;
    root.position.copy(carToLocal(bx, by, B.z));
    root.rotation.y = -B.yawDeg * DEG;
    root.updateMatrix(); zup.updateMatrix();
    baseToLocal = root.matrix.clone().multiply(zup.matrix);           // 베이스 좌표 → 차량 로컬
    localToBase = baseToLocal.clone().invert();
    basePos.copy(root.position);
    safeY = H + MOTION.safeAbove;
  }
  placeBase();
  const toBase = (v) => v.clone().applyMatrix4(localToBase).toArray();
  const dirToBase = (v) => v.clone().transformDirection(localToBase).toArray();
  const fromBase = (a) => new THREE.Vector3(...a).applyMatrix4(baseToLocal);
  const dirFromBase = (a) => new THREE.Vector3(...a).transformDirection(baseToLocal);

  const armMat = new THREE.MeshStandardMaterial({ color: 0xeef1f4, roughness: 0.45, metalness: 0.05 });
  let probeMats = [];
  let kin = null, loading = null;
  const rotGroups = [];
  const q = HOME.slice();
  let motion = null;                            // { duration, t, step(t), resolve }
  let source = "ros";                           // "demo" | "ros"
  let lastNormal = null;                        // 지금 붙어 있는 면의 바깥 법선 (공중이면 null)
  const tiltChecked = new Set();
  let ghost = null;

  // ------------------------------------------------------------ 로딩
  function load() {
    if (!loading) loading = (async () => {
      const spec = await (await fetch(MODEL_DIR + "m0609.json")).json();
      const probe = buildProbe(PROBE);
      probeMats = probe.materials;
      placeBase();
      kin = createKinematics(spec, probe, { limits: DEMO_JOINT_LIMITS_DEG.map(([a, b]) => [a * DEG, b * DEG]) });
      const links = { base_link: zup };
      for (const j of spec.joints) {
        const origin = new THREE.Group();
        origin.position.set(...j.xyz);
        origin.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2], "ZYX");     // URDF rpy = Rz·Ry·Rx
        const rot = new THREE.Group();
        origin.add(rot);
        links[j.parent].add(origin);
        links[j.child] = rot;
        rotGroups.push(rot);
      }
      links[spec.flange.link].add(probe.object);
      const loader = new ColladaLoader();
      const gripperReady = probe.ready;
      await Promise.all(Object.entries(spec.links).flatMap(([link, files]) => files.map(async (f) => {
        const s = (await loader.loadAsync(MODEL_DIR + f)).scene;
        s.quaternion.identity();                  // ColladaLoader 의 Z_UP 자동 회전 취소 (zup 그룹에서 한 번에 돌림)
        const extra = [];
        s.traverse((o) => {
          if (o.isLight || o.isCamera) extra.push(o);   // DAE 에 들어 있는 Blender Camera·Lamp
          if (o.isMesh) {
            [].concat(o.material).forEach((m) => m.dispose());
            o.material = armMat; o.castShadow = o.receiveShadow = false;
          }
        });
        extra.forEach((o) => o.parent.remove(o));
        links[link].add(s);
      })));
      await gripperReady;
      applyQ();
    })();
    return loading;
  }

  function applyQ() { rotGroups.forEach((g, i) => (g.rotation.z = q[i])); }

  function setGhost(on) {
    if (ghost === on) return;
    ghost = on;
    for (const m of [armMat, ...probeMats]) {
      m.transparent = on; m.opacity = on ? 0.35 : 1; m.depthWrite = !on; m.needsUpdate = true;
    }
  }

  // ------------------------------------------------------------ 자세
  /** 지금 탐침 끝 위치·방향 (차량 로컬) */
  function tipPose() {
    const { p, d } = kin.fk(q);
    return { pos: fromBase(p), dir: dirFromBase(d) };
  }
  /** IK 로 한 프레임 따라가기 (웜 스타트) */
  function track(pose) {
    const r = kin.solve(q, toBase(pose.pos), dirToBase(pose.dir));
    for (let i = 0; i < 6; i++) q[i] = r.q[i];
    applyQ();
    return r;
  }
  /** 이 자세에 닿을 수 있는지 (홈과 지금 자세에서 각각 풀어 봄) → 위치 오차 [mm] */
  function reachErr(pose) {
    let best = Infinity;
    for (const seed of [q, HOME]) {
      let s = seed.slice(), r;
      for (let k = 0; k < 6; k++) { r = kin.solve(s, toBase(pose.pos), dirToBase(pose.dir), { iters: 40 }); s = r.q; }
      best = Math.min(best, r.posErr + (r.angErr > 0.02 ? 50 : 0));
    }
    return best;
  }

  /** 표면 접촉 자세: 표면 점 + 바깥 법선 → 볼 중심은 법선 쪽으로 볼 반지름만큼, 탐침은 표면을 누르는 방향.
   *  옆면은 SIDE_TILT_DEG 만큼 아래로 기울임. 처음 쓰는 면에서 IK 가 안 닿으면 경고 (표면 근처까지만 감) */
  function contactPose(surfacePos, normal) {
    const n = normal.clone().normalize();
    const pos = surfacePos.clone().addScaledVector(n, PROBE.ballRadius);
    const press = n.clone().negate();
    if (n.y > 0.7) return { pos, dir: press };                    // 윗면: 그대로 아래로
    const toBase = basePos.clone().sub(pos).setY(0);
    const near = n.dot(toBase) > 0;
    const deg = near ? SIDE_TILT_DEG.near : SIDE_TILT_DEG.far;
    const pose = { pos, dir: press.multiplyScalar(Math.cos(deg * DEG)).addScaledVector(DOWN, Math.sin(deg * DEG)).normalize() };
    const key = [n.x, n.y, n.z].map((v) => Math.round(v * 2)).join(",");
    if (kin && !tiltChecked.has(key)) {
      tiltChecked.add(key);
      const err = reachErr(pose);
      if (err > 2) console.warn(`[robot3d] 옆면(${near ? "로봇 쪽" : "건너편"}) IK 오차 ${err.toFixed(1)} mm → 표면 근처까지만 접근`);
    }
    return pose;
  }

  // ------------------------------------------------------------ 동작 (데모)
  function startMotion(duration, step) {
    cancel();
    return new Promise((resolve) => { motion = { duration: Math.max(duration, 1e-3), t: 0, step, resolve }; });
  }
  /** 진행 중인 동작을 그 자리에서 멈춤 (await 하던 쪽은 false 를 받음) */
  function cancel() {
    if (!motion) return;
    const r = motion.resolve; motion = null; r(false);
  }
  const ease = (s) => (s < 0.5 ? 2 * s * s : 1 - (-2 * s + 2) ** 2 / 2);
  function slerpDir(a, b, s) {
    const ang = Math.acos(THREE.MathUtils.clamp(a.dot(b), -1, 1));
    if (ang < 1e-4) return b.clone();
    let axis = new THREE.Vector3().crossVectors(a, b);
    if (axis.lengthSq() < 1e-10) axis = new THREE.Vector3(1, 0, 0).cross(a);
    return a.clone().applyAxisAngle(axis.normalize(), ang * s);
  }

  /** 지금 위치에서 pose 까지 직선(탐침 방향은 회전 보간)으로 */
  function moveTo(pose, { speed = MOTION.airSpeed } = {}) {
    const from = tipPose();
    const dist = from.pos.distanceTo(pose.pos);
    const ang = Math.acos(THREE.MathUtils.clamp(from.dir.dot(pose.dir), -1, 1));
    const dur = Math.max(MOTION.minMove, dist / speed + ang * MOTION.turnSec);
    return startMotion(dur, (t, T) => {
      const s = ease(t / T);
      track({ pos: from.pos.clone().lerp(pose.pos, s), dir: slerpDir(from.dir, pose.dir, s) });
    });
  }

  /** 관절 공간 보간 (홈 복귀 등) */
  function moveJoints(target, dur = 1.0) {
    const from = q.slice();
    return startMotion(dur, (t, T) => {
      const s = ease(t / T);
      for (let i = 0; i < 6; i++) q[i] = from[i] + (target[i] - from[i]) * s;
      applyQ();
    });
  }

  /** 표면에서 떼고 → (다른 면/먼 곳이면 safeY 높이로 넘어가서) → 목표 위로 → 붙이기.
   *  to = contactPose(), normal = 목표 면의 바깥 법선. 같은 면 가까운 곳이면 높이 넘기 없이 들었다 놓기만. */
  async function transit(to, normal) {
    const cur = tipPose();
    const legs = [];
    let p = cur.pos.clone();
    if (lastNormal) { p.addScaledVector(lastNormal, MOTION.lift); legs.push({ pos: p.clone(), dir: cur.dir }); }
    const approach = to.pos.clone().addScaledVector(normal, MOTION.lift);
    const sameFace = lastNormal && lastNormal.dot(normal) > 0.95 && p.distanceTo(approach) < 70;
    if (!sameFace) {
      const up = Math.max(safeY, p.y, approach.y);
      legs.push({ pos: new THREE.Vector3(p.x, up, p.z), dir: DOWN.clone() });
      legs.push({ pos: new THREE.Vector3(approach.x, up, approach.z), dir: DOWN.clone() });
    }
    legs.push({ pos: approach, dir: to.dir });
    for (const l of legs) if (!(await moveTo(l))) return false;
    if (!(await moveTo(to, { speed: MOTION.approachSpeed }))) return false;
    lastNormal = normal.clone().normalize();
    return true;
  }

  /** 표면에 붙인 채 경로 따라가기. poseAt(s) → contactPose, s = 0..length [mm].
   *  onProgress(s) 는 매 프레임 호출 (측정 점 간격 판단용). */
  async function followPath(poseAt, length, { speed, onProgress = null } = {}) {
    const start = poseAt(0);
    if (tipPose().pos.distanceTo(start.pos) > 1) {
      if (!(await moveTo(start, { speed: MOTION.approachSpeed }))) return false;
    }
    return startMotion(length / speed, (t, T) => {
      const s = (t / T) * length;
      track(poseAt(s));
      if (onProgress) onProgress(s);
    });
  }

  /** 공중으로 들어 올린 뒤 홈 자세로 */
  async function home() {
    if (lastNormal) {
      const cur = tipPose();
      const up = cur.pos.clone().addScaledVector(lastNormal, MOTION.lift);
      if (!(await moveTo({ pos: up, dir: cur.dir }))) return false;
      if (!(await moveTo({ pos: new THREE.Vector3(up.x, Math.max(safeY, up.y), up.z), dir: DOWN.clone() }))) return false;
    }
    lastNormal = null;
    return moveJoints(HOME, 1.0);
  }

  // ------------------------------------------------------------ 매 프레임
  function update(dt) {
    if (motion) {
      motion.t = Math.min(motion.duration, motion.t + Math.min(dt, 0.1));
      motion.step(motion.t, motion.duration);
      if (motion.t >= motion.duration) { const r = motion.resolve; motion = null; r(true); }
    }
    if (source === "ros") setGhost(!rosLive());
  }
  viewer.onFrame(update);

  // ------------------------------------------------------------ 실제 모드 (rosbridge)
  let topic = null, rosConnected = false, lastJointAt = 0;
  const rosLive = () => rosConnected && performance.now() - lastJointAt < ROS_JOINTS.staleMs;
  function attachRos(ros) {
    if (topic) topic.unsubscribe();
    rosConnected = true;
    topic = new ROSLIB.Topic({
      ros, name: ROS_JOINTS.topic, messageType: ROS_JOINTS.type,
      throttle_rate: Math.round(1000 / ROS_JOINTS.rateHz), queue_length: 1,
    });
    topic.subscribe((m) => {
      const now = performance.now();
      if (now - lastJointAt < 1000 / ROS_JOINTS.rateHz - 2) return;   // 서버 throttle 이 안 먹어도 여기서 한 번 더
      lastJointAt = now;
      if (source !== "ros" || !m.name) return;
      m.name.forEach((name, k) => {
        const i = /joint_?([1-6])$/.exec(name);        // "joint_1" (네임스페이스 접두어가 붙어도 매칭)
        if (i && Number.isFinite(m.position[k])) q[Number(i[1]) - 1] = m.position[k];
      });
      applyQ();
    });
  }
  function onRosClose() { rosConnected = false; topic = null; }       // 마지막 자세 유지 + 반투명 (update 에서)
  function setSource(s) {
    source = s;
    if (s === "demo") { setGhost(false); }
    else cancel();
  }

  // ------------------------------------------------------------ 표시
  async function setVisible(on) {
    if (on && !kin) {
      const prev = msgEl ? [msgEl.textContent, msgEl.className] : null;
      if (msgEl) { msgEl.textContent = "로봇팔 불러오는 중…"; msgEl.className = "viewer-msg small-msg"; }
      try { await load(); } finally { if (msgEl) [msgEl.textContent, msgEl.className] = prev; }
    }
    root.visible = on;
    if (!on) cancel();
  }

  return {
    load, setVisible, setSource, attachRos, onRosClose, cancel,
    transit, followPath, home, contactPose, tipPose,
    get visible() { return root.visible && !!kin; },
    get busy() { return !!motion; },
  };
}
