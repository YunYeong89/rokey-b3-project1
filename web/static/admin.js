/* 관리자 화면 (SCR-101 ~ SCR-104)
 *
 * 흐름
 *  [수리 시작] → POST /api/jobs {vehicle_id, mode, parts} → rosbridge /repair/command 로
 *     {"cmd":"start","job_id":123} 발행 (모드·부품은 DB에서 조회)
 *  로봇 쪽 공정 관리 노드가 단계마다 API 로 DB 에 기록하고 /repair/status 로 실시간 값을 보낸다.
 *  화면은 (1) /repair/status 로 측정 점·힘을 실시간 표시, (2) /api/jobs/<id> 를 주기적으로 읽어 단계·결과 표시.
 *  데모 모드에서는 브라우저 안 시뮬레이터가 로봇 대신 같은 API 를 호출한다.
 *
 * /repair/status (std_msgs/String, JSON)
 *  {"type":"point","x":120,"y":40,"dz":-0.8,"state":"dent"}  {"type":"wrench","fz":-4.2}  {"type":"flush","mm":0.18}
 *  point 는 연속 접촉 스캔 중 일정 간격(기본 5 mm)마다 온다.
 * 로봇팔 3D (robot3d.js): 실제 모드는 /dsr01/joint_states 구독, 데모 모드는 시뮬레이터가 탐침 경로를 준다.
 */
import { createCarViewer } from "./viewer3d.js";
import { robotControl } from "./robot_control.js";
import { createRobotArm } from "./robot3d.js";

const state = {
  job: null, pollTimer: null, points: [], rosOk: false,
  demo: false, demoCancel: false, mode: "auto",
  running: false, fz: [], robotMsg: null, alarms: [], alarmSeen: 0,
};
const FZ_MAX = 200;

// ------------------------------------------------------------ 탭
$$(".tab").forEach((b) => b.addEventListener("click", () => {
  $$(".tab").forEach((x) => x.classList.toggle("active", x === b));
  $$(".panel").forEach((p) => p.classList.toggle("active", p.id === "tab-" + b.dataset.tab));
  ({ stock: loadParts, history: loadJobs, members: loadMembers, alarms: openAlarms }[b.dataset.tab] || (() => {}))();
}));

// ------------------------------------------------------------ 모드 / 부품 선택
$$("#modeSeg .seg-btn").forEach((b) => b.addEventListener("click", () => {
  if (b.disabled) return;
  state.mode = b.dataset.mode;
  $$("#modeSeg .seg-btn").forEach((x) => x.classList.toggle("active", x === b));
  $("#partPick").classList.toggle("hidden", state.mode !== "manual");
  $("[data-inspection]").hidden = state.mode !== "manual";   // 부품 탐색 창은 수동 모드에서만 (로봇 3D 아래)
  repaint();
}));
$("#partPickList").addEventListener("change", () => repaint());
const pickedParts = () => $$("#partPickList input:checked").map((c) => c.value);

// 수동 모드 부품 목록: robot_ok=1 부품만. 재고 탭에서 바꾸면 다시 그림 (고른 것은 유지)
function renderPartPick(ps) {
  const picked = new Set(pickedParts());
  $("#partPickList").innerHTML = ps.filter((p) => p.robot_ok).map((p) =>
    `<label class="chk"><input type="checkbox" value="${esc(p.code)}"${picked.has(p.code) ? " checked" : ""}> ${esc(p.name)}</label>`).join("")
    || `<p class="note">로봇 교체 가능 부품이 없습니다. 재고 탭에서 켜 주세요.</p>`;
  $$("#partPickList input").forEach((c) => (c.disabled = $("#startBtn").disabled));
  repaint();
}

// ------------------------------------------------------------ 데이터 로드
async function loadVehicles() {
  const vs = await api("GET", "/api/vehicles");
  $("#vehicleSel").innerHTML = vs.map((v) =>
    `<option value="${v.id}" data-model="${esc(v.model)}">${esc(v.name)} · ${esc(v.owner_name || "차주 없음")}</option>`).join("");
  return vs;
}

async function loadParts() {
  const ps = await api("GET", "/api/parts");          // 서버가 분류 순(외판·미러·램프·기타)으로 정렬
  let cat = null;
  $("#partsBody").innerHTML = ps.map((p) => {
    const st = p.stock === 0 ? ["out", "없음"] : p.stock <= p.min_stock ? ["low", "부족"] : ["ok", "충분"];
    const head = p.category !== cat ? `<tr class="group"><td colspan="8">${esc((cat = p.category) || "미분류")}</td></tr>` : "";
    return head + `<tr><td>${esc(p.code)}</td><td>${esc(p.name)}</td><td>${esc(p.location || "–")}</td><td><b>${p.stock}</b></td><td>${p.min_stock}</td>
      <td><span class="badge ${st[0]}">${st[1]}</span></td>
      <td><label class="switch" title="로봇 교체 켜기/끄기"><input type="checkbox" data-robot="${esc(p.code)}"${p.robot_ok ? " checked" : ""}
        aria-label="${esc(p.name)} 로봇 교체"><span></span></label>${p.robot_ok ? `<span class="badge robot">로봇 교체 가능</span>` : ""}</td>
      <td><input type="number" min="0" value="${p.stock}" data-code="${esc(p.code)}"><button class="btn primary small" data-save="${esc(p.code)}">저장</button></td></tr>`;
  }).join("");
  renderPartPick(ps);
  $$("[data-robot]").forEach((c) => c.addEventListener("change", async () => {
    c.disabled = true;
    try { await api("PATCH", `/api/parts/${c.dataset.robot}`, { robot_ok: c.checked ? 1 : 0 }); }
    catch (e) { alert("저장 실패: " + e.message); }
    loadParts();
  }));
  $$("[data-save]").forEach((b) => b.addEventListener("click", async () => {
    const v = parseInt($(`input[data-code="${b.dataset.save}"]`).value, 10);
    try { await api("PATCH", `/api/parts/${b.dataset.save}`, { stock: v }); loadParts(); }
    catch (e) { alert("저장 실패: " + e.message); }
  }));
}

async function loadJobs() {
  const js = await api("GET", "/api/jobs?limit=200");
  const vs = await api("GET", "/api/vehicles");
  const owner = Object.fromEntries(vs.map((v) => [v.id, v.owner_name || "–"]));
  $("#jobsBody").innerHTML = js.map((j) => `<tr class="click" data-id="${j.id}">
    <td>${j.id}</td><td>${esc(j.vehicle_name)}</td><td>${esc(owner[j.vehicle_id])}</td><td>${MODE_TEXT[j.mode]}</td>
    <td>${esc(j.operator_name)}</td><td>${esc(j.started_at || "")}</td>
    <td><span class="badge ${j.status}">${STATUS_TEXT[j.status] || j.status}</span></td>
    <td>${(j.replaced || "").split(",").filter(Boolean).map(partName).join(", ") || "–"}</td>
    <td>${esc(j.summary || "")}</td></tr>`).join("") || `<tr><td colspan="9" class="note">아직 수리 기록이 없습니다.</td></tr>`;
  $$("#jobsBody tr.click").forEach((tr) => tr.addEventListener("click", async () => {
    const box = $("#jobDetail");
    box.innerHTML = jobDetailHtml(await api("GET", `/api/jobs/${tr.dataset.id}`));
    box.classList.remove("hidden");
    box.scrollIntoView({ behavior: "smooth" });
  }));
}

async function loadMembers() {
  const [us, vs] = await Promise.all([api("GET", "/api/users"), api("GET", "/api/vehicles")]);
  $("#usersBody").innerHTML = us.map((u) =>
    `<tr><td>${esc(u.username)}</td><td>${esc(u.name)}</td><td><span class="badge ${u.role === "admin" ? "mode" : "ok"}">${u.role === "admin" ? "관리자" : "사용자"}</span></td>
    <td><input type="email" class="inline" data-uid="${u.id}" value="${esc(u.email)}" placeholder="없음"
      style="width: 100%; min-width: 150px; margin: 0; padding: 6px 8px; font-size: 14px"></td></tr>`).join("");
  $$("#usersBody input[data-uid]").forEach((inp) => inp.addEventListener("change", async () => {   // 칸에서 나오거나 Enter 면 저장
    const msg = $("#emailMsg");
    try {
      const r = await api("PATCH", `/api/users/${inp.dataset.uid}`, { email: inp.value });
      inp.value = r.email || ""; msg.className = "form-msg ok"; msg.textContent = "이메일을 저장했습니다.";
    } catch (e) { msg.className = "form-msg err"; msg.textContent = e.message; }
  }));
  const owners = us.filter((u) => u.role === "user");
  const opts = (sel) => `<option value="">차주 없음</option>` + owners.map((u) =>
    `<option value="${u.id}" ${u.id === sel ? "selected" : ""}>${esc(u.name)} (${esc(u.username)})</option>`).join("");
  $("#ownerSel").innerHTML = opts(null);
  $("#vehiclesBody").innerHTML = vs.map((v) => `<tr><td>${esc(v.name)}</td><td>${esc(v.model)}</td>
    <td><select class="inline" data-vid="${v.id}">${opts(v.owner_id)}</select></td></tr>`).join("");
  $$("#vehiclesBody select").forEach((s) => s.addEventListener("change", async () => {
    try { await api("PATCH", `/api/vehicles/${s.dataset.vid}`, { owner_id: s.value ? Number(s.value) : null }); loadVehicles(); }
    catch (e) { alert(e.message); }
  }));
}

function formHandler(formSel, msgSel, path, toBody) {
  $(formSel).addEventListener("submit", async (ev) => {
    ev.preventDefault();
    const f = Object.fromEntries(new FormData(ev.target).entries());
    const msg = $(msgSel);
    try {
      await api("POST", path, toBody(f));
      ev.target.reset(); msg.className = "form-msg ok"; msg.textContent = "등록했습니다.";
      loadMembers(); loadVehicles();
    } catch (e) { msg.className = "form-msg err"; msg.textContent = e.message; }
  });
}
formHandler("#userForm", "#userMsg", "/api/users", (f) => f);
formHandler("#vehicleForm", "#vehicleMsg", "/api/vehicles",
  (f) => ({ name: f.name, model: f.model, owner_id: f.owner_id ? Number(f.owner_id) : null }));

// ------------------------------------------------------------ 3D 차량 뷰어 (고객 화면과 공용)
// 관리자 화면에서는 숨겨 둔다 (#viewer hidden): 데모 모드의 차 형상(스캔 구역·표면 위치) 계산용
const viewer = createCarViewer($("#viewer"), {
  msgEl: $("#viewerMsg"),
  onPartClick: (code) => {                     // 수동 모드: 3D 화면에서 부품을 눌러 선택
    if (state.mode !== "manual" || $("#startBtn").disabled) return;
    const c = $(`#partPickList input[value="${code}"]`);   // 로봇 교체 가능 부품만 목록에 있음
    if (c) { c.checked = !c.checked; repaint(); }
  },
});
$$("[data-view]").forEach((b) => b.addEventListener("click", () => viewer.setView(b.dataset.view)));

// ------------------------------------------------------------ 로봇팔 (관리자 화면만)
const robot = createRobotArm(viewer, { msgEl: $("#viewerMsg") });
const ROBOT_KEY = "repair.robotArm";
async function setRobotShown(on) {
  const b = $("#robotToggle");
  if (!b) return;                              // 모형 차 3D 를 숨긴 화면: 로봇팔 표시 없음
  b.disabled = true;
  try {
    await robot.setVisible(on);
    viewer.setRobotExtent(on);
    b.setAttribute("aria-pressed", String(on));
    try { localStorage.setItem(ROBOT_KEY, on ? "1" : "0"); } catch (e) { }
  } catch (e) {
    console.error(e); alert("로봇팔 모델을 불러오지 못했습니다: " + e.message);
  } finally { b.disabled = false; }
}
$("#robotToggle")?.addEventListener("click", () => setRobotShown(!robot.visible));

function repaint() {
  const job = state.job;
  const running = !!job && job.status === "running";
  let pst = partStates(job);
  if (!running && state.mode === "manual") {   // 시작 전: 고른 부품만 미리보기
    pst = Object.fromEntries(pickedParts().map((p) => [p, "requested"]));
  }
  viewer.paint(pst, job ? job.findings : [], running);
}

// ------------------------------------------------------------ 실시간 상태 (ROS 또는 데모)
function onStatus(msg) {
  if (msg.type === "point") {
    state.points.push(msg);
    viewer.addPoint(msg);
    $("#statPts").textContent = state.points.length;
    const d = Math.min(...state.points.map((p) => p.dz));
    $("#statDepth").textContent = d < -0.05 ? Math.abs(d).toFixed(2) : "–";
  } else if (msg.type === "wrench") {
    const fz = Number(msg.fz);
    if (!Number.isFinite(fz)) return;
    $("#statFz").textContent = fz.toFixed(1);
    state.fz.push(fz);
    if (state.fz.length > FZ_MAX) state.fz.splice(0, state.fz.length - FZ_MAX);
    scheduleFz();
  } else if (msg.type === "flush") {
    $("#statFlush").textContent = Number(msg.mm).toFixed(2);
  } else if (msg.type === "robot") {
    if (["idle", "moving", "error"].includes(msg.state)) { state.robotMsg = msg; updateRobotState(); }
  }
}

// 측정 점·힘 그래프·통계 초기화 (새 수리 시작 / 차량 변경 시)
function resetLive() {
  state.points = []; viewer.clearPoints();
  state.fz = []; drawFz();
  ["#statPts", "#statFz", "#statDepth", "#statFlush"].forEach((s) => ($(s).textContent = s === "#statPts" ? "0" : "–"));
}

// ------------------------------------------------------------ 힘(Fz) 실시간 선 그래프 (canvas, 라이브러리 없음)
let fzQueued = false;
function scheduleFz() {                        // 메시지가 몰려도 화면 갱신은 프레임당 1번
  if (fzQueued) return;
  fzQueued = true;
  requestAnimationFrame(() => { fzQueued = false; drawFz(); });
}
function niceStep(range) {
  const raw = range / 4, p = 10 ** Math.floor(Math.log10(raw)), m = raw / p;
  return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p;
}
function drawFz() {
  const cv = $("#fzCanvas");
  if (!cv) return;                             // Fz 그래프는 로봇 탭 실시간 힘 그래프로 대체 (현재 Fz 는 요약 칸)
  const w = cv.clientWidth, h = cv.clientHeight;
  if (!w || !h) return;                        // 다른 탭에 있으면 그리지 않음 (돌아올 때 다시 그림)
  const dpr = window.devicePixelRatio || 1;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  }
  const g = cv.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const data = state.fz;
  const L = 40, R = 8, T = 6, B = 6, pw = w - L - R, ph = h - T - B;
  let lo = Math.min(0, ...data), hi = Math.max(0, ...data);
  if (hi - lo < 2) { const c = (hi + lo) / 2; lo = c - 1; hi = c + 1; }
  const step = niceStep(hi - lo);
  lo = Math.floor(lo / step) * step; hi = Math.ceil(hi / step) * step;
  const X = (i) => L + (i / (FZ_MAX - 1)) * pw;
  const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;

  g.font = "11px system-ui, sans-serif"; g.textAlign = "right"; g.textBaseline = "middle";
  for (let v = lo; v <= hi + step / 2; v += step) {
    const y = Math.round(Y(v)) + 0.5;
    g.strokeStyle = Math.abs(v) < step / 2 ? "#9aa4b0" : "#dde2e8"; g.lineWidth = 1;
    g.beginPath(); g.moveTo(L, y); g.lineTo(w - R, y); g.stroke();
    g.fillStyle = "#6b7684"; g.fillText((Math.abs(v) < 1e-9 ? 0 : v).toFixed(step < 1 ? 1 : 0), L - 6, y);
  }
  if (!data.length) {
    g.textAlign = "center"; g.fillStyle = "#9aa4b0"; g.fillText("힘 데이터 없음", L + pw / 2, T + ph / 2);
    $("#fzRange").textContent = "";
    return;
  }
  const off = FZ_MAX - data.length;            // 최신 값이 항상 오른쪽 끝
  g.strokeStyle = "#0f5c8c"; g.lineWidth = 2; g.lineJoin = "round";
  g.beginPath();
  data.forEach((v, i) => (i ? g.lineTo(X(off + i), Y(v)) : g.moveTo(X(off + i), Y(v))));
  g.stroke();
  const last = data[data.length - 1];
  g.fillStyle = "#0f5c8c"; g.beginPath(); g.arc(X(FZ_MAX - 1), Y(last), 3.5, 0, Math.PI * 2); g.fill();
  $("#fzRange").textContent = `최소 ${Math.min(...data).toFixed(1)} · 최대 ${Math.max(...data).toFixed(1)} · ${data.length}개`;
}
window.addEventListener("resize", () => drawFz());

// ------------------------------------------------------------ 로봇 상태 (상단 바)
// /repair/status 의 {"type":"robot","state":...} 가 오면 그것을 우선, 없으면 연결 여부 + 진행 중 작업으로 판단
const ROBOT_TEXT = { off: "연결 끊김", idle: "대기", moving: "동작 중", error: "에러" };
function updateRobotState() {
  const connected = state.demo || state.rosOk;
  let st, title = "";
  if (!connected) st = "off";
  else if (state.robotMsg) { st = state.robotMsg.state; title = state.robotMsg.message || ""; }
  else st = state.running ? "moving" : "idle";
  const el = $("#robotState");
  el.className = "robot-state " + st;
  el.querySelector("span").textContent = ROBOT_TEXT[st] + (st === "error" && title ? ` · ${title}` : "");
  el.title = title || "로봇 상태";
}

// ------------------------------------------------------------ 작업 진행 표시 (DB 주기 조회)
function showJob(j) {                         // 화면에 작업 표시만 (진행 제어는 안 건드림)
  state.job = j;
  $("#progBar").style.width = paintSteps("#steps", j) + "%";
  renderStatusCard(j);
  const rf = j ? j.replacements.filter((r) => r.flush_mm != null).pop() : null;
  if (rf) $("#statFlush").textContent = Number(rf.flush_mm).toFixed(2);
  repaint();
}
function renderJob(j) {
  showJob(j);
  if (j.status !== "running") { setRunning(false); stopPolling(); loadParts(); }
}

// 대기 중 차량을 바꾸면 그 차량의 진행 중/마지막 수리를 보여 줌 (고객 화면과 동일)
async function showVehicle() {
  const sel = $("#vehicleSel");
  const title = $("#viewerTitle");
  if (title) title.textContent = sel.selectedOptions[0] ? sel.selectedOptions[0].textContent : "차량";
  if ($("#startBtn").disabled || !sel.value) return;
  await viewer.load(sel.selectedOptions[0].dataset.model);    // 차종에 맞는 모델 (car.glb 우선)
  computeScanGeometry(viewer.layout());                       // 다른 모델이면 스캔 구역도 그 배치로
  try {
    let job = await api("GET", `/api/jobs/current?vehicle_id=${sel.value}`);
    if (!job) {
      const last = await api("GET", `/api/jobs?vehicle_id=${sel.value}&limit=1`);
      job = last.length ? await api("GET", `/api/jobs/${last[0].id}`) : null;
    }
    resetLive();
    showJob(job);
  } catch (e) { console.warn(e); }
}
$("#vehicleSel").addEventListener("change", showVehicle);
function startPolling(id) {
  stopPolling();
  const tick = async () => { try { renderJob(await api("GET", `/api/jobs/${id}`)); } catch (e) { console.warn(e); } };
  tick(); state.pollTimer = setInterval(tick, 700);
}
function stopPolling() { if (state.pollTimer) clearInterval(state.pollTimer); state.pollTimer = null; }
function setRunning(on) {
  state.running = on;
  $("#startBtn").disabled = on; $("#stopBtn").disabled = !on;
  $("#vehicleSel").disabled = on; $("#demoMode").disabled = on;
  $$("#modeSeg .seg-btn, #partPick input").forEach((x) => (x.disabled = on));
  updateRobotState();
}

// ------------------------------------------------------------ 시작 / 정지
$("#startBtn").addEventListener("click", async () => {
  if (!state.demo && (!robotControl.ready || robotControl.status?.busy)) {
    alert("ROS 제어 노드와 로봇이 준비되어야 하며, 다른 기능 실행 중에는 수리를 시작할 수 없습니다."); return;
  }
  const scene = robotControl.status?.scene || "";
  if (!state.demo && robotControl.status?.mode === "virtual"
      && !["sedan", "suv", "sports", "real"].includes(scene) && !scene.startsWith("demo")) {
    alert("수리 공정은 sedan/suv/sports 또는 실차(real, demo) 장면에서 실행하세요."); return;
  }
  // 실차 자동 모드는 탐침으로 보닛을 스캔(real_scan): 그리퍼 피드백이 없으면 사람이 확인 (보닛 교체 뒤에는 탐침이 거치대에 있음)
  const realCar = robotControl.status?.mode === "real" || scene === "real" || scene.startsWith("demo");
  if (!state.demo && state.mode === "auto" && realCar && robotControl.status?.probe_attached !== true
      && !confirm("그리퍼 상태를 확인할 수 없습니다.\n로봇이 탐침을 잡은 상태인지 직접 확인했나요?\n(확인을 누르면 보닛 스캔을 시작합니다)")) return;
  const parts = state.mode === "manual" ? pickedParts() : [];
  if (state.mode === "manual" && !parts.length) { alert("교체할 부품을 1개 이상 고르세요."); return; }
  resetLive();
  $("#resultBox").className = "result hidden";
  let job;
  try { job = await api("POST", "/api/jobs", { vehicle_id: Number($("#vehicleSel").value), mode: state.mode, parts }); }
  catch (e) {
    if (e.status === 409) {
      const cur = await api("GET", "/api/jobs/current");
      if (cur && confirm(`진행 중인 수리(#${cur.id})가 남아 있습니다. 정지 처리할까요?`))
        await api("POST", `/api/jobs/${cur.id}/finish`, { status: "stopped", summary: "관리자가 정리함" });
      return;
    }
    alert(e.message); return;
  }
  setRunning(true); renderJob(job); startPolling(job.id);
  if (state.demo) runDemo(job);
  else {
    try { robotControl.repair({ cmd: "start", job_id: job.id }); }
    catch (e) {
      await api("POST", `/api/jobs/${job.id}/finish`, { status: "failed", summary: e.message });
      alert(e.message);
    }
  }
});

// 수리 카드 정지 버튼 (부품 탐색의 로봇 정지도 같이 씀): 로봇 정지(web_control) + 진행 중 작업 정지
async function stopJob() {
  const running = state.job && state.job.status === "running";
  if (state.demo) {
    if (!running) return;
    state.demoCancel = true; robot.cancel();
  } else {
    try { robotControl.stop(); }              // 웹에서 시작한 작업 중지 + 로봇 move_stop (web_control)
    catch (e) { alert(e.message); }
    if (running) try { robotControl.repair({ cmd: "stop", job_id: state.job.id }); } catch (e) { }
  }
  if (running) try { await api("POST", `/api/jobs/${state.job.id}/finish`, { status: "stopped", summary: "관리자가 정지 버튼을 누름" }); } catch (e) { }
}
$("#stopBtn").addEventListener("click", stopJob);
// 로봇 탭의 '로봇 정지' 버튼도 진행 중인 수리를 같이 멈춘다 (로봇 정지 명령은 robot_control.js 가 보냄)
$$("[data-task-stop]").forEach((b) => b.addEventListener("click", () => { if (state.job?.status === "running") stopJob(); }));

// ------------------------------------------------------------ rosbridge
// robot_control.js 의 연결 하나를 함께 쓴다: web_control 상태·명령, /repair/status, 로봇팔 관절값 (http/https·LAN 주소 공통)
function connectRos() {
  let jointTopic;
  const attachArm = () => {
    jointTopic = robotControl.jointTopic();
    robot.attachRos(robotControl.ros, jointTopic);
  };
  robotControl.addEventListener("connection", () => {
    state.robotMsg = null; attachArm(); updateRobotState();
  });
  robotControl.addEventListener("close", () => {
    state.robotMsg = null; robot.onRosClose(); updateRobotState();               // 마지막 자세 유지 + 반투명
  });
  if (robotControl.connected) attachArm();
  robotControl.addEventListener("state", async () => {
    if (robotControl.connected && jointTopic !== robotControl.jointTopic()) {
      state.robotMsg = null;
      attachArm();
    }
    state.rosOk = robotControl.ready;
    if (!state.demo) setRosPill(state.rosOk ? "on" : "off", !robotControl.live ? "ROS 제어 노드 응답 대기"
      : !state.rosOk ? "로봇 준비 대기" : robotControl.status.busy ? "로봇 실행 중" : "로봇 제어 준비됨");
    updateRobotState();
    const status = robotControl.status;
    if (!state.demo && status?.event === "rejected" && status.rejected_job === state.job?.id)
      console.warn(status.error);
  });
  robotControl.addEventListener("repair", (event) => { if (!state.demo) onStatus(event.detail); });
}
function setRosPill(cls, text) { const p = $("#rosState"); p.className = "pill " + cls; p.textContent = text; }
$("#demoMode").addEventListener("change", (e) => {
  state.demo = e.target.checked;
  robot.setSource(state.demo ? "demo" : "ros");
  if (state.demo) setRosPill("demo", "데모 모드 (로봇 없음)");
  else setRosPill(state.rosOk ? "on" : "off", state.rosOk ? "로봇 연결됨" : "로봇 연결 안 됨");
  state.robotMsg = null; updateRobotState();
});

// ------------------------------------------------------------ 데모 시뮬레이터 (로봇 대신 같은 API 호출)
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const randn = () => Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());

async function runDemo(job) {
  const jobId = job.id;
  state.demoCancel = false;
  const ev = (stage, message, level = "info") => api("POST", `/api/jobs/${jobId}/events`, { stage, message, level });
  const check = () => { if (state.demoCancel) throw new Error("stopped"); };
  let errored = false;
  onStatus({ type: "robot", state: "moving", message: "데모 수리 중" });
  try {
    await sleep(300); check();          // 차량은 고정 위치: 기준점 터치 없이 바로 시작

    let toReplace;
    if (job.mode === "manual") {
      toReplace = job.requested;
      await ev("stock", `수동 모드: 지정 부품 ${toReplace.map(partName).join(", ")} 교체 (스캔·판정 건너뜀)`);
    } else {
      const robotOk = new Set((await api("GET", "/api/parts")).filter((p) => p.robot_ok).map((p) => p.code));
      const found = await demoScan(jobId, ev, check, robotOk);
      if (!found.length) {
        await ev("detect", "손상 없음");
        await api("POST", `/api/jobs/${jobId}/finish`, { status: "no_damage", summary: "모든 측정 점이 임계값 이내" });
        return;
      }
      toReplace = found.filter((c) => robotOk.has(c));        // 로봇은 robot_ok=1 부품만 교체
      const manualOnly = found.filter((c) => !robotOk.has(c)).map(partName);
      if (manualOnly.length) await ev("detect", `로봇 교체 불가 부품: ${manualOnly.join(", ")} (수작업 필요)`, "warn");
      if (!toReplace.length) {
        await api("POST", `/api/jobs/${jobId}/finish`, { status: "on_hold", summary: `로봇 교체 불가: ${manualOnly.join(", ")}` });
        return;
      }
    }

    const lacking = [];
    for (const c of toReplace) {
      check();
      const p = await api("GET", `/api/parts/${c}/check`);
      await ev("stock", `${p.name} 재고 ${p.stock}개`, p.available ? "info" : "warn");
      if (!p.available) lacking.push(p.name);
      await sleep(300);
    }
    if (lacking.length) {
      await api("POST", `/api/jobs/${jobId}/finish`, { status: "on_hold", summary: `재고 없음: ${lacking.join(", ")}` });
      return;
    }

    const results = [];
    for (const c of toReplace) {
      check();
      await ev("replace", `${partName(c)} 분리`);
      if (robot.visible) await moveToPart(c); else await sleep(700);   // 떼고 붙이는 동작은 다음 단계
      check();
      await ev("replace", `${partName(c)} 새 부품 장착 (힘 제어)`);
      let peak = 0;
      for (let i = 0; i < 12; i++) { const fz = -(4 + i * 1.1 + randn() * 0.5); peak = Math.max(peak, -fz); onStatus({ type: "wrench", fz }); await sleep(60); }
      check();
      await ev("inspect", `${partName(c)} 단차 측정 (4점)`); await sleep(500);
      const flush = Math.abs(0.15 + randn() * 0.12);
      onStatus({ type: "flush", mm: flush });
      const ok = flush < 0.5;
      await api("POST", `/api/jobs/${jobId}/replacements`, { part_code: c, success: ok, retries: 0, peak_force: peak, flush_mm: flush });
      if (ok) await api("POST", `/api/parts/${c}/consume`);
      await ev("inspect", `${partName(c)} 단차 ${flush.toFixed(2)} mm → ${ok ? "합격" : "불합격"}`, ok ? "info" : "error");
      results.push(ok);
    }
    if (robot.visible) { await robot.home(); check(); }
    await ev("done", "작업 종료, 홈 위치 복귀");
    const allOk = results.every(Boolean);
    await api("POST", `/api/jobs/${jobId}/finish`, {
      status: allOk ? "done" : "failed",
      summary: allOk ? `${toReplace.map(partName).join(", ")} 교체 완료` : "재검사 불합격 부품 있음",
    });
  } catch (e) {
    if (e.message !== "stopped") {
      console.error(e);
      errored = true;
      onStatus({ type: "robot", state: "error", message: e.message });
      try { await ev("done", "데모 오류: " + e.message, "error"); await api("POST", `/api/jobs/${jobId}/finish`, { status: "failed", summary: e.message }); } catch (_) { }
    }
  } finally {
    if (!errored) onStatus({ type: "robot", state: "idle", message: "" });   // 완료·보류·정지 모두 대기로
    if (!state.demoCancel && robot.visible) robot.home();   // 손상 없음·보류로 끝나도 홈으로 (정지면 그 자리에)
  }
}

// ------------------------------------------------------------ 연속 접촉 스캔 (데모)
// 탐침을 표면에 붙인 채 지그재그(래스터) 라인을 따라가며 일정 간격마다 측정 점을 만든다.
// 부품 경계(트렁크 → 지붕 → 보닛, 문 → 문)를 넘거나 옆면으로 바꿀 때만 들어 올렸다 다시 붙인다.
// 로봇팔이 켜져 있으면 로봇 동작이 끝나기를 기다리고(실제 속도), 꺼져 있으면 점당 짧게만 쉰다(예전 속도).
const SCAN = {
  sampleStep: 5,          // 측정 점 간격 (이동 거리 기준) [mm]
  fineStep: 3,            // 상세 스캔 점 간격 [mm]
  lineGap: 15,            // 윗면 라인 간격 [mm]
  doorLineGap: 8,         // 문 라인 간격 [mm]
  detailGap: 4,           // 상세 스캔 라인 간격 [mm]
  detailMargin: 5,        // 의심 점 묶음 바깥으로 더 볼 폭 [mm]
  speed: 80,              // 윗면 접촉 이동 속도 [mm/s] → 라인(약 90 mm) 하나 1.1 s
  doorSpeed: 36,          // 문 접촉 이동 속도 [mm/s] → 라인(약 36 mm) 하나 1 s
  detailSpeed: 50,        // 상세 스캔 속도 [mm/s]
  hiddenMs: 12,           // 로봇팔이 꺼져 있을 때 점당 대기 [ms]
  hiddenFineMs: 8,
};
const DOOR_ORDER = ["door_rl", "door_fl", "door_fr", "door_rr"];   // 로봇 쪽(왼쪽) 먼저, 차 위로 넘어가서 오른쪽
const TOP_NAMES = { trunk: "트렁크", roof: "지붕", bonnet: "보닛" };   // 윗면 스캔 구역 (뒤 → 앞 순서, 배치에 있는 것만)
const INSET = { top: 4, doorX: 5, doorZ: 3 };                      // 부품 가장자리에서 안쪽으로 [mm]
/** 스캔 구역·부품 위치 (차량 좌표계 mm) — viewer.load() 후 차 배치(viewer.layout())로 계산 → 모델을 바꿔도 그대로.
 *  TOP_AREAS: 윗면 구역 (구역 사이는 들어 올려서 넘음) / DOOR_AREA: 문 (옆면 y, 높이 z)
 *  PART_SPOT: 교체 단계에서 로봇이 가 볼 부품 위치 [면, u, v] (top: x, y / left·right: x, 높이 z) */
let TOP_AREAS = [], DOOR_AREA = {}, PART_SPOT = {};
function computeScanGeometry(lay) {
  const inset = ([a, b], m) => [a + m, b - m];
  const mid = ([a, b]) => (a + b) / 2;
  TOP_AREAS = Object.entries(TOP_NAMES).filter(([k]) => lay.parts?.[k])
    .map(([k, name]) => ({ code: k, name, x: inset(lay.parts[k].x, INSET.top), y: inset(lay.parts[k].y, INSET.top) }));
  DOOR_AREA = Object.fromEntries(Object.entries(lay.doors || {}).map(([k, d]) =>
    [k, { x: inset(d.x, INSET.doorX), y: d.y, z: inset(d.z, INSET.doorZ), face: d.y < lay.W / 2 ? "left" : "right" }]));
  PART_SPOT = {};
  const b = lay.parts?.bonnet;
  if (b) PART_SPOT.bonnet = ["top", mid(b.x), mid(b.y)];
  for (const [k, d] of Object.entries(DOOR_AREA)) PART_SPOT[k] = [d.face, mid(d.x), mid(d.z)];
  for (const [k, m] of Object.entries(lay.mirrors || {})) PART_SPOT[k] = [m.y < lay.W / 2 ? "left" : "right", m.x, m.z];
}

/** 지그재그 경로. alongU = 라인이 u 방향 (문), 아니면 v 방향 (윗면: 차 폭 방향) */
function raster([u0, u1], [v0, v1], gap, alongU) {
  const pts = [];
  if (alongU) {
    const n = Math.max(1, Math.round((v1 - v0) / gap));
    for (let i = 0; i <= n; i++) { const v = v0 + ((v1 - v0) * i) / n; pts.push(...(i % 2 ? [[u1, v], [u0, v]] : [[u0, v], [u1, v]])); }
  } else {
    const n = Math.max(1, Math.round((u1 - u0) / gap));
    for (let i = 0; i <= n; i++) { const u = u0 + ((u1 - u0) * i) / n; pts.push(...(i % 2 ? [[u, v1], [u, v0]] : [[u, v0], [u, v1]])); }
  }
  return pts;
}

/** 의심 점들을 가까운 것끼리 묶어 상세 스캔 구역(사각형)으로. bounds 안으로 자름 */
function zonesOf(pts, bounds, join = 12) {
  const zones = [];
  for (const p of pts) {
    const z = zones.find((z) => p.u >= z.u[0] - join && p.u <= z.u[1] + join && p.v >= z.v[0] - join && p.v <= z.v[1] + join);
    if (z) { z.u = [Math.min(z.u[0], p.u), Math.max(z.u[1], p.u)]; z.v = [Math.min(z.v[0], p.v), Math.max(z.v[1], p.v)]; }
    else zones.push({ u: [p.u, p.u], v: [p.v, p.v] });
  }
  const m = SCAN.detailMargin, clip = (r, b) => [Math.max(b[0], r[0] - m), Math.min(b[1], r[1] + m)];
  return zones.map((z) => ({ u: clip(z.u, bounds.u), v: clip(z.v, bounds.v) }));
}

/** 표면 위 (u, v) 에 탐침을 대는 자세.
 *  탐침 방향은 메쉬의 국소 법선이 아니라 스캔하는 면으로 정함 (윗면 = 수직 아래, 옆면 = 정해진 기울기).
 *  실제 모델은 지붕 가장자리 곡면·문 손잡이 등에서 법선이 갑자기 바뀌어, 법선을 따르면 IK 가 팔을 다른 자세로 휙 돌림 */
const FACE_NORMAL = { top: [0, 1, 0], left: [0, 0, -1], right: [0, 0, 1] };   // carRoot 로컬 (Y 위, Z 오른쪽)
function contactAt(face, u, v) {
  const h = viewer.surfaceAt(face, u, v);
  const n = h.normal.clone().set(...FACE_NORMAL[face]);
  return { pose: robot.contactPose(h.pos, n), normal: n };
}
/** 들어 올려 옮긴 뒤 표면에 대기 (로봇팔이 꺼져 있으면 아무것도 안 함) */
async function touch(face, u, v) {
  if (!robot.visible) return;
  const c = contactAt(face, u, v);
  await robot.transit(c.pose, c.normal);
}
async function moveToPart(code) {
  const spot = PART_SPOT[code];
  if (spot) await touch(...spot);
}

/** 연속 접촉 스캔 한 번: 경로 시작점에 붙이고 → 떼지 않고 polyline 을 따라감.
 *  onSample(u, v) 를 이동 거리 step 마다 호출 (→ /repair/status point 와 같은 처리) */
async function contactScan(face, pts, { step, speed, fine = false, check, onSample }) {
  const segs = [];
  let L = 0;
  for (let i = 1; i < pts.length; i++) {
    const [a, b] = [pts[i - 1], pts[i]], len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (len > 0) { segs.push({ a, b, len, s0: L }); L += len; }
  }
  const at = (s) => {
    const g = segs.find((g) => s <= g.s0 + g.len) || segs[segs.length - 1];
    const k = Math.min(1, Math.max(0, (s - g.s0) / g.len));
    return [g.a[0] + (g.b[0] - g.a[0]) * k, g.a[1] + (g.b[1] - g.a[1]) * k];
  };
  const n = Math.floor(L / step + 1e-6) + 1;
  let next = 0;
  const emitUpTo = (s) => { while (next < n && next * step <= s + 1e-6) onSample(...at(next++ * step)); };

  if (robot.visible) {
    const [u0, v0] = pts[0];
    const c = contactAt(face, u0, v0);
    await robot.transit(c.pose, c.normal); check();
    await robot.followPath((s) => contactAt(face, ...at(s)).pose, L, { speed, onProgress: emitUpTo }); check();
    emitUpTo(L);                                // 도중에 로봇팔을 끈 경우 남은 점
  } else {
    while (next < n) { check(); emitUpTo(next * step); await sleep(fine ? SCAN.hiddenFineMs : SCAN.hiddenMs); }
  }
}

async function demoScan(jobId, ev, check, robotOk) {
  const r = Math.random();
  const bon = TOP_AREAS.find((a) => a.code === "bonnet");           // 데모 손상은 보닛 가운데 쪽에
  const lerp = ([a, b], t) => a + (b - a) * t;
  const dents = bon && r < 0.55 ? [{ x: lerp(bon.x, 0.3 + Math.random() * 0.45), y: lerp(bon.y, 0.2 + Math.random() * 0.6), depth: 0.8 + Math.random() * 1.6 }] : [];
  const missing = r > 0.35 && r < 0.85 ? [Math.random() < 0.5 ? "mirror_l" : "mirror_r"] : [];
  // 가끔(30%) 문 찌그러짐 — 로봇 교체 가능으로 켜진 문 중에서
  const doors = Object.keys(DOOR_AREA).filter((d) => robotOk.has(d));
  const doorDent = doors.length && Math.random() < 0.3 ? (() => {
    const code = doors[Math.floor(Math.random() * doors.length)], a = DOOR_AREA[code];
    return { code, x: a.x[0] + Math.random() * (a.x[1] - a.x[0]), y: a.y, z: a.z[0] + Math.random() * (a.z[1] - a.z[0]), depth: 0.9 + Math.random() * 1.6 };
  })() : null;
  const SIGMA = 0.08, SUS = 1.5 * SIGMA * 3, DMG = 3 * SIGMA * 3;   // 데모용 임계값 [mm]
  const heightAt = (x, y) => {
    let dz = SIGMA * randn();
    for (const d of dents) { const dist = Math.hypot(x - d.x, y - d.y); if (dist < 14) dz -= d.depth * (1 - (dist / 14) ** 2); }
    return dz;
  };
  const sideDz = (code, x, z) => {
    let dz = SIGMA * randn();
    if (doorDent && doorDent.code === code) { const dist = Math.hypot(x - doorDent.x, z - doorDent.z); if (dist < 10) dz -= doorDent.depth * (1 - (dist / 10) ** 2); }
    return dz;
  };
  const classify = (dz) => (dz < -DMG ? "dent" : dz < -SUS ? "suspect" : "ok");
  const r1 = (v) => Math.round(v * 10) / 10;
  const contactFz = () => onStatus({ type: "wrench", fz: -5 + randn() * 0.3 });   // 접촉 유지 힘

  // ---- 윗면: 구역마다 지그재그, 구역 사이만 들어 올림
  await ev("scan", `표면 스캔(연속 접촉) 시작 — 윗면 ${TOP_AREAS.length}구역, 라인 간격 ${SCAN.lineGap} mm, 점 간격 ${SCAN.sampleStep} mm`);
  const suspicious = TOP_AREAS.map(() => []);
  for (const [k, a] of TOP_AREAS.entries()) {
    await contactScan("top", raster(a.x, a.y, SCAN.lineGap, false), {
      step: SCAN.sampleStep, speed: SCAN.speed, check,
      onSample: (x, y) => {
        const dz = heightAt(x, y), st = classify(dz);
        onStatus({ type: "point", x: r1(x), y: r1(y), dz, state: st });
        contactFz();
        if (st !== "ok") suspicious[k].push({ u: x, v: y });
      },
    });
  }
  const topZones = TOP_AREAS.flatMap((a, k) => zonesOf(suspicious[k], { u: a.x, v: a.y }));
  if (topZones.length) {
    await ev("scan", `의심 구역 ${topZones.length}곳 상세 스캔(연속 접촉, 라인 간격 ${SCAN.detailGap} mm)`);
    for (const z of topZones) {
      await contactScan("top", raster(z.u, z.v, SCAN.detailGap, false), {
        step: SCAN.fineStep, speed: SCAN.detailSpeed, fine: true, check,
        onSample: (x, y) => { const dz = heightAt(x, y); onStatus({ type: "point", x: r1(x), y: r1(y), dz, state: classify(dz), fine: true }); contactFz(); },
      });
    }
  }

  // ---- 옆면(문): 탐침이 옆을 보고 문 표면을 따라감. 점에 z 가 있으면 화면이 옆면에 표시
  await ev("scan", "표면 스캔(연속 접촉) — 옆면 문 4곳");
  const doorSus = {};
  for (const code of DOOR_ORDER.filter((c) => DOOR_AREA[c])) {
    const a = DOOR_AREA[code];
    await contactScan(a.face, raster(a.x, [a.z[0] + 2, a.z[1]], SCAN.doorLineGap, true), {
      step: SCAN.sampleStep, speed: SCAN.doorSpeed, check,
      onSample: (x, z) => {
        const dz = sideDz(code, x, z), st = classify(dz);
        onStatus({ type: "point", x: r1(x), y: a.y, z: r1(z), dz, state: st });
        contactFz();
        if (st !== "ok") (doorSus[code] ||= []).push({ u: x, v: z });
      },
    });
  }
  for (const [code, pts] of Object.entries(doorSus)) {
    const a = DOOR_AREA[code];
    await ev("scan", `${partName(code)} 의심 구역 상세 스캔(연속 접촉)`, "warn");
    for (const z of zonesOf(pts, { u: a.x, v: a.z })) {
      await contactScan(a.face, raster(z.u, z.v, SCAN.detailGap, true), {
        step: SCAN.fineStep, speed: SCAN.detailSpeed, fine: true, check,
        onSample: (x, zz) => { const dz = sideDz(code, x, zz); onStatus({ type: "point", x: r1(x), y: a.y, z: r1(zz), dz, state: classify(dz), fine: true }); contactFz(); },
      });
    }
  }

  for (const m of ["mirror_l", "mirror_r"]) {
    check();
    if (robot.visible) { await moveToPart(m); check(); }
    await ev("scan", `${partName(m)} 접촉 확인: ${missing.includes(m) ? "접촉 없음" : "접촉 있음"}`, missing.includes(m) ? "warn" : "info");
    await sleep(250);
  }
  await ev("detect", "정상 모델과 비교 중"); await sleep(500); check();
  const findings = [
    ...dents.map((d) => ({ part_code: "bonnet", x: d.x, y: d.y, depth: -d.depth, decision: "dent" })),
    ...(doorDent ? [{ part_code: doorDent.code, x: doorDent.x, y: doorDent.y, z: doorDent.z, depth: -doorDent.depth, decision: "dent" }] : []),
    ...missing.map((m) => ({ part_code: m, decision: "missing" })),
  ];
  if (findings.length) await api("POST", `/api/jobs/${jobId}/findings`, findings);
  const parts = [...new Set(findings.map((f) => f.part_code))];
  if (parts.length) await ev("detect", `교체 대상: ${parts.map(partName).join(", ")}`, "warn");
  return parts;
}

// ------------------------------------------------------------ 알람 (warn/error 이벤트)
const ALARM_KEY = "repair.alarmSeen";
try { state.alarmSeen = Number(localStorage.getItem(ALARM_KEY)) || 0; } catch (e) { }
const alarmTabOpen = () => $("#tab-alarms").classList.contains("active");

async function pollAlarms() {
  try { state.alarms = await api("GET", "/api/alarms?limit=100"); } catch (e) { console.warn(e); return; }
  if (alarmTabOpen()) { renderAlarms(); markAlarmsSeen(); }
  const n = state.alarms.filter((a) => a.id > state.alarmSeen).length;
  const c = $("#alarmCount");
  c.textContent = n >= 100 ? "99+" : n;
  c.classList.toggle("hidden", n === 0);
}
function renderAlarms() {
  $("#alarmsBody").innerHTML = state.alarms.map((a) => `<tr class="${a.id > state.alarmSeen ? "new" : ""}">
    <td>${esc(a.ts)}</td><td><span class="lvl ${esc(a.level)}">${a.level === "error" ? "오류" : "경고"}</span></td>
    <td>#${a.job_id}</td><td>${esc(a.vehicle_name)}</td><td>${esc(a.stage)}</td><td>${esc(a.message)}</td></tr>`).join("")
    || `<tr><td colspan="6" class="note">알람이 없습니다.</td></tr>`;
}
function markAlarmsSeen() {
  const max = state.alarms.reduce((m, a) => Math.max(m, a.id), state.alarmSeen);
  if (max === state.alarmSeen) return;
  state.alarmSeen = max;
  try { localStorage.setItem(ALARM_KEY, String(max)); } catch (e) { }
}
async function openAlarms() {
  renderAlarms();                              // 새 알람 강조를 먼저 보여 준 뒤 확인 처리
  await pollAlarms();
  markAlarmsSeen();
  $("#alarmCount").classList.add("hidden");
}
$("#alarmRefresh").addEventListener("click", openAlarms);

// ------------------------------------------------------------ 시작
(async function init() {
  await viewer.load();
  computeScanGeometry(viewer.layout());
  try { if (localStorage.getItem(ROBOT_KEY) === "1") setRobotShown(true); } catch (e) { }
  repaint();
  await loadVehicles();
  loadParts();
  updateRobotState(); drawFz();
  connectRos();
  pollAlarms(); setInterval(pollAlarms, 3000);
  const cur = await api("GET", "/api/jobs/current");   // 새로고침해도 진행 중인 작업 이어서 표시
  if (cur) {
    $("#vehicleSel").value = cur.vehicle_id;
    const title = $("#viewerTitle");
    if (title) title.textContent = $("#vehicleSel").selectedOptions[0]?.textContent || cur.vehicle_name;
    await viewer.load(cur.model);
    computeScanGeometry(viewer.layout());
    setRunning(true); renderJob(cur); startPolling(cur.id);
  } else {
    await showVehicle();
  }
})();
