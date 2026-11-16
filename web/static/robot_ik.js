/* 로봇팔 기구학 (FK + 수치 IK) — three.js 에 의존하지 않는 순수 계산
 *
 * 좌표계: 로봇 베이스(base_link), Z 위쪽, 길이 mm, 각도 rad. 관절 정보는 static/models/m0609/m0609.json.
 * IK: damped least squares (DLS). 목표 = 탐침 끝(볼 중심) 위치 + 탐침이 누르는 방향(벡터 하나).
 *     탐침 축을 중심으로 한 회전은 자유로 둔다(구형 볼이라 상관없음). 관절 한계는 매 반복마다 clamp.
 *     호출할 때마다 바로 전 자세에서 시작하므로(웜 스타트) 연속 동작이 튀지 않는다.
 */

// ---- 4x4 행렬 (행 우선 배열 16개)
function mul(a, b) {
  const o = new Array(16);
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
    o[r * 4 + c] = a[r * 4] * b[c] + a[r * 4 + 1] * b[4 + c] + a[r * 4 + 2] * b[8 + c] + a[r * 4 + 3] * b[12 + c];
  }
  return o;
}
function fromXyzRpy([x, y, z], [r, p, w]) {        // URDF: R = Rz(yaw)·Ry(pitch)·Rx(roll)
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(w), sy = Math.sin(w);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y,
    -sp, cp * sr, cp * cr, z,
    0, 0, 0, 1,
  ];
}
function rotZ(q) { const c = Math.cos(q), s = Math.sin(q); return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]; }
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);

// 6x6 선형 방정식 A x = b (가우스 소거, 부분 피벗)
function solve6(A, b) {
  const n = 6, M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

/**
 * @param spec  m0609.json 내용
 * @param tool  { tip: [x,y,z], dir: [x,y,z] } — 플랜지(link_6) 좌표계에서 탐침 끝 위치와 누르는 방향
 * @param opt.limits  [[하한, 상한] | null] × 6 [rad] — IK 가 쓸 관절 범위 (URDF 한계보다 좁게: 자세 고르기용). null 이면 URDF 그대로
 */
export function createKinematics(spec, tool, { limits = null } = {}) {
  const joints = spec.joints;
  const origins = joints.map((j) => fromXyzRpy(j.xyz, j.rpy));
  const lower = joints.map((j, i) => Math.max(j.lower, limits?.[i]?.[0] ?? -Infinity));
  const upper = joints.map((j, i) => Math.min(j.upper, limits?.[i]?.[1] ?? Infinity));
  let tip = tool.tip, tdir = tool.dir;

  /** q → { p: 탐침 끝, d: 누르는 방향, axes: [{o, z}] (관절마다 원점·회전축) } */
  function fk(q) {
    let T = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    const axes = [];
    for (let i = 0; i < 6; i++) {
      T = mul(T, origins[i]);
      axes.push({ o: [T[3], T[7], T[11]], z: [T[2], T[6], T[10]] });
      T = mul(T, rotZ(q[i]));
    }
    const p = [0, 1, 2].map((r) => T[r * 4] * tip[0] + T[r * 4 + 1] * tip[1] + T[r * 4 + 2] * tip[2] + T[r * 4 + 3]);
    const d = [0, 1, 2].map((r) => T[r * 4] * tdir[0] + T[r * 4 + 1] * tdir[1] + T[r * 4 + 2] * tdir[2]);
    return { p, d, axes };
  }

  /**
   * DLS IK.  target: 탐침 끝 [mm], dir: 누르는 방향(단위벡터)
   * 반환 { q, posErr [mm], angErr [rad] }
   */
  function solve(q0, target, dir, { iters = 30, lambda = 20, w = 200, maxStep = 40 } = {}) {
    const q = q0.slice();
    let posErr = Infinity, angErr = Infinity;
    for (let it = 0; it <= iters; it++) {
      const { p, d, axes } = fk(q);
      let ep = [target[0] - p[0], target[1] - p[1], target[2] - p[2]];
      const eo = cross(d, dir);                       // d 를 dir 쪽으로 돌리는 회전 (크기 ≈ sin 각도)
      posErr = norm(ep);
      angErr = Math.acos(Math.max(-1, Math.min(1, dot(d, dir))));
      if ((posErr < 0.2 && angErr < 0.003) || it === iters) break;
      if (posErr > maxStep) ep = ep.map((v) => (v * maxStep) / posErr);
      const e = [...ep, w * eo[0], w * eo[1], w * eo[2]];
      const J = [[], [], [], [], [], []];             // 6x6, 열 = 관절
      for (let i = 0; i < 6; i++) {
        const { o, z } = axes[i];
        const v = cross(z, [p[0] - o[0], p[1] - o[1], p[2] - o[2]]);
        for (let r = 0; r < 3; r++) { J[r][i] = v[r]; J[r + 3][i] = w * z[r]; }
      }
      // dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
      const A = [0, 1, 2, 3, 4, 5].map((r) => [0, 1, 2, 3, 4, 5].map((c) => {
        let s = r === c ? lambda * lambda : 0;
        for (let k = 0; k < 6; k++) s += J[r][k] * J[c][k];
        return s;
      }));
      const y = solve6(A, e);
      for (let i = 0; i < 6; i++) {
        let dq = 0;
        for (let r = 0; r < 6; r++) dq += J[r][i] * y[r];
        q[i] = Math.min(upper[i], Math.max(lower[i], q[i] + dq));
      }
    }
    return { q, posErr, angErr };
  }

  return { fk, solve, lower, upper, setTool(t) { tip = t.tip; tdir = t.dir; } };
}
