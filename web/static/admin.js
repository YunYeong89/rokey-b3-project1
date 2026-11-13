/* 관리자 화면 (SCR-101 ~ SCR-104)
 *
 * 흐름
 *  [수리 시작] → POST /api/jobs {vehicle_id, mode, parts} → rosbridge /repair/command 로
 *     {"cmd":"start","job_id","vehicle_id","mode":"auto|manual","parts":[...]} 발행
 *  로봇 쪽 공정 관리 노드가 단계마다 API 로 DB 에 기록하고 /repair/status 로 실시간 값을 보낸다.
 *  화면은 (1) /repair/status 로 측정 점·힘을 실시간 표시, (2) /api/jobs/<id> 를 주기적으로 읽어 단계·결과 표시.
 *  데모 모드에서는 브라우저 안 시뮬레이터가 로봇 대신 같은 API 를 호출한다.
 *
 * /repair/status (std_msgs/String, JSON)
 *  {"type":"point","x":120,"y":40,"dz":-0.8,"state":"dent"}  {"type":"wrench","fz":-4.2}  {"type":"flush","mm":0.18}
 */
import { createCarViewer } from "./viewer3d.js";

const state = {
  job: null, pollTimer: null, points: [], ros: null, cmdTopic: null, rosOk: false,
  demo: false, demoCancel: false, mode: "auto",
};

// ------------------------------------------------------------ 탭
$$(".tab").forEach((b) => b.addEventListener("click", () => {
  $$(".tab").forEach((x) => x.classList.toggle("active", x === b));
  $$(".panel").forEach((p) => p.classList.toggle("active", p.id === "tab-" + b.dataset.tab));
  ({ stock: loadParts, history: loadJobs, members: loadMembers }[b.dataset.tab] || (() => {}))();
  if (b.dataset.tab === "repair") requestAnimationFrame(viewer.resize);
}));

// ------------------------------------------------------------ 모드 / 부품 선택
$$("#modeSeg .seg-btn").forEach((b) => b.addEventListener("click", () => {
  if (b.disabled) return;
  state.mode = b.dataset.mode;
  $$("#modeSeg .seg-btn").forEach((x) => x.classList.toggle("active", x === b));
  $("#partPick").classList.toggle("hidden", state.mode !== "manual");
  repaint();
}));
$$("#partPick input").forEach((c) => c.addEventListener("change", () => repaint()));
const pickedParts = () => $$("#partPick input:checked").map((c) => c.value);

// ------------------------------------------------------------ 데이터 로드
async function loadVehicles() {
  const vs = await api("GET", "/api/vehicles");
  $("#vehicleSel").innerHTML = vs.map((v) =>
    `<option value="${v.id}">${esc(v.name)} · ${esc(v.owner_name || "차주 없음")}</option>`).join("");
  return vs;
}

async function loadParts() {
  const ps = await api("GET", "/api/parts");
  $("#partsBody").innerHTML = ps.map((p) => {
    const st = p.stock === 0 ? ["out", "없음"] : p.stock <= p.min_stock ? ["low", "부족"] : ["ok", "충분"];
    return `<tr><td>${esc(p.code)}</td><td>${esc(p.name)}</td><td><b>${p.stock}</b></td><td>${p.min_stock}</td>
      <td><span class="badge ${st[0]}">${st[1]}</span></td>
      <td><input type="number" min="0" value="${p.stock}" data-code="${esc(p.code)}"><button class="btn primary small" data-save="${esc(p.code)}">저장</button></td></tr>`;
  }).join("");
  $$("[data-save]").forEach((b) => b.addEventListener("click", async () => {
    const v = parseInt($(`input[data-code="${b.dataset.save}"]`).value, 10);
    try { await api("PATCH", `/api/parts/${b.dataset.save}`, { stock: v }); loadParts(); }
    catch (e) { alert("저장 실패: " + e.message); }
  }));
}

async function loadJobs() {
  const js = await api("GET", "/api/jobs");
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
    `<tr><td>${esc(u.username)}</td><td>${esc(u.name)}</td><td><span class="badge ${u.role === "admin" ? "mode" : "ok"}">${u.role === "admin" ? "관리자" : "사용자"}</span></td></tr>`).join("");
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
const viewer = createCarViewer($("#viewer"), {
  msgEl: $("#viewerMsg"),
  onPartClick: (code) => {                     // 수동 모드: 3D 화면에서 부품을 눌러 선택
    if (state.mode !== "manual" || $("#startBtn").disabled) return;
    const c = $(`#partPick input[value="${code}"]`);
    if (c) { c.checked = !c.checked; repaint(); }
  },
});
$$("[data-view]").forEach((b) => b.addEventListener("click", () => viewer.setView(b.dataset.view)));

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
    $("#statFz").textContent = Number(msg.fz).toFixed(1);
  } else if (msg.type === "flush") {
    $("#statFlush").textContent = Number(msg.mm).toFixed(2);
  }
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
  $("#viewerTitle").textContent = sel.selectedOptions[0] ? sel.selectedOptions[0].textContent : "차량";
  if ($("#startBtn").disabled || !sel.value) return;
  try {
    let job = await api("GET", `/api/jobs/current?vehicle_id=${sel.value}`);
    if (!job) {
      const last = await api("GET", `/api/jobs?vehicle_id=${sel.value}&limit=1`);
      job = last.length ? await api("GET", `/api/jobs/${last[0].id}`) : null;
    }
    state.points = []; viewer.clearPoints();
    ["#statPts", "#statFz", "#statDepth", "#statFlush"].forEach((s) => ($(s).textContent = s === "#statPts" ? "0" : "–"));
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
  $("#startBtn").disabled = on; $("#stopBtn").disabled = !on;
  $("#vehicleSel").disabled = on; $("#demoMode").disabled = on;
  $$("#modeSeg .seg-btn, #partPick input").forEach((x) => (x.disabled = on));
}

// ------------------------------------------------------------ 시작 / 정지
$("#startBtn").addEventListener("click", async () => {
  if (!state.demo && !state.rosOk) {
    alert("로봇(rosbridge)에 연결되지 않았습니다.\n로봇 없이 확인하려면 '데모 모드'를 켜세요."); return;
  }
  const parts = state.mode === "manual" ? pickedParts() : [];
  if (state.mode === "manual" && !parts.length) { alert("교체할 부품을 1개 이상 고르세요."); return; }
  state.points = []; viewer.clearPoints();
  ["#statPts", "#statFz", "#statDepth", "#statFlush"].forEach((s) => ($(s).textContent = s === "#statPts" ? "0" : "–"));
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
  else state.cmdTopic.publish(new ROSLIB.Message({ data: JSON.stringify(
    { cmd: "start", job_id: job.id, vehicle_id: job.vehicle_id, mode: job.mode, parts: job.requested }) }));
});

$("#stopBtn").addEventListener("click", async () => {
  if (!state.job) return;
  if (state.demo) state.demoCancel = true;
  else if (state.rosOk) state.cmdTopic.publish(new ROSLIB.Message({ data: JSON.stringify({ cmd: "stop", job_id: state.job.id }) }));
  try { await api("POST", `/api/jobs/${state.job.id}/finish`, { status: "stopped", summary: "관리자가 정지 버튼을 누름" }); } catch (e) { }
});

// ------------------------------------------------------------ rosbridge
function connectRos() {
  if (typeof ROSLIB === "undefined") { if (!state.demo) setRosPill("off", "roslib 없음 (인터넷 확인)"); return; }
  const ros = new ROSLIB.Ros({ url: APP.rosbridgeUrl || `ws://${location.hostname}:9090` });
  state.ros = ros;
  ros.on("connection", () => {
    state.rosOk = true; if (!state.demo) setRosPill("on", "로봇 연결됨");
    state.cmdTopic = new ROSLIB.Topic({ ros, name: "/repair/command", messageType: "std_msgs/String" });
    state.cmdTopic.advertise();   // 미리 광고해 두지 않으면 첫 명령이 유실될 수 있음
    new ROSLIB.Topic({ ros, name: "/repair/status", messageType: "std_msgs/String", throttle_rate: 50 })
      .subscribe((m) => { if (!state.demo) { try { onStatus(JSON.parse(m.data)); } catch (e) { } } });
  });
  ros.on("error", () => { });
  ros.on("close", () => { state.rosOk = false; if (!state.demo) setRosPill("off", "로봇 연결 안 됨"); setTimeout(connectRos, 3000); });
}
function setRosPill(cls, text) { const p = $("#rosState"); p.className = "pill " + cls; p.textContent = text; }
$("#demoMode").addEventListener("change", (e) => {
  state.demo = e.target.checked;
  if (state.demo) setRosPill("demo", "데모 모드 (로봇 없음)");
  else setRosPill(state.rosOk ? "on" : "off", state.rosOk ? "로봇 연결됨" : "로봇 연결 안 됨");
});

// ------------------------------------------------------------ 데모 시뮬레이터 (로봇 대신 같은 API 호출)
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const randn = () => Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());

async function runDemo(job) {
  const jobId = job.id;
  state.demoCancel = false;
  const ev = (stage, message, level = "info") => api("POST", `/api/jobs/${jobId}/events`, { stage, message, level });
  const check = () => { if (state.demoCancel) throw new Error("stopped"); };
  try {
    await ev("calibrate", "차량 기준점 3곳 터치"); await sleep(600); check();
    onStatus({ type: "wrench", fz: -5.1 });
    await ev("calibrate", "차량 좌표계 설정 완료"); await sleep(300); check();

    let toReplace;
    if (job.mode === "manual") {
      toReplace = job.requested;
      await ev("stock", `수동 모드: 지정 부품 ${toReplace.map(partName).join(", ")} 교체 (스캔·판정 건너뜀)`);
    } else {
      toReplace = await demoScan(jobId, ev, check);
      if (!toReplace.length) {
        await ev("detect", "손상 없음");
        await api("POST", `/api/jobs/${jobId}/finish`, { status: "no_damage", summary: "모든 측정 점이 임계값 이내" });
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
      await ev("replace", `${partName(c)} 분리`); await sleep(700); check();
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
    await ev("done", "작업 종료, 홈 위치 복귀");
    const allOk = results.every(Boolean);
    await api("POST", `/api/jobs/${jobId}/finish`, {
      status: allOk ? "done" : "failed",
      summary: allOk ? `${toReplace.map(partName).join(", ")} 교체 완료` : "재검사 불합격 부품 있음",
    });
  } catch (e) {
    if (e.message !== "stopped") {
      console.error(e);
      try { await ev("done", "데모 오류: " + e.message, "error"); await api("POST", `/api/jobs/${jobId}/finish`, { status: "failed", summary: e.message }); } catch (_) { }
    }
  }
}

async function demoScan(jobId, ev, check) {
  const r = Math.random();
  const dents = r < 0.55 ? [{ x: 200 + Math.random() * 30, y: 30 + Math.random() * 50, depth: 0.8 + Math.random() * 1.6 }] : [];
  const missing = r > 0.35 && r < 0.85 ? [Math.random() < 0.5 ? "mirror_l" : "mirror_r"] : [];
  const SIGMA = 0.08, SUS = 1.5 * SIGMA * 3, DMG = 3 * SIGMA * 3;   // 데모용 임계값 [mm]
  const heightAt = (x, y) => {
    let dz = SIGMA * randn();
    for (const d of dents) { const dist = Math.hypot(x - d.x, y - d.y); if (dist < 14) dz -= d.depth * (1 - (dist / 14) ** 2); }
    return dz;
  };
  const classify = (dz) => (dz < -DMG ? "dent" : dz < -SUS ? "suspect" : "ok");

  await ev("scan", "격자 스캔 시작 (20 mm 간격)");
  const suspicious = [];
  for (let x = 10; x <= 250; x += 20) for (let y = 10; y <= 100; y += 15) {
    check();
    const dz = heightAt(x, y), st = classify(dz);
    onStatus({ type: "point", x, y, dz, state: st });
    onStatus({ type: "wrench", fz: -5 + randn() * 0.3 });
    if (st !== "ok") suspicious.push({ x, y });
    await sleep(25);
  }
  if (suspicious.length) {
    await ev("scan", `의심 구역 ${suspicious.length}곳 상세 스캔 (5 mm 간격)`);
    for (const s of suspicious) for (let dx = -10; dx <= 10; dx += 5) for (let dy = -10; dy <= 10; dy += 5) {
      check();
      const x = s.x + dx, y = s.y + dy, dz = heightAt(x, y);
      onStatus({ type: "point", x, y, dz, state: classify(dz), fine: true });
      await sleep(12);
    }
  }
  for (const m of ["mirror_l", "mirror_r"]) {
    check();
    await ev("scan", `${partName(m)} 접촉 확인: ${missing.includes(m) ? "접촉 없음" : "접촉 있음"}`, missing.includes(m) ? "warn" : "info");
    await sleep(250);
  }
  await ev("detect", "정상 모델과 비교 중"); await sleep(500); check();
  const findings = [
    ...dents.map((d) => ({ part_code: "bonnet", x: d.x, y: d.y, depth: -d.depth, decision: "dent" })),
    ...missing.map((m) => ({ part_code: m, decision: "missing" })),
  ];
  if (findings.length) await api("POST", `/api/jobs/${jobId}/findings`, findings);
  const parts = [...new Set(findings.map((f) => f.part_code))];
  if (parts.length) await ev("detect", `교체 대상: ${parts.map(partName).join(", ")}`, "warn");
  return parts;
}

// ------------------------------------------------------------ 시작
(async function init() {
  await viewer.load();
  repaint();
  await loadVehicles();
  loadParts();
  connectRos();
  const cur = await api("GET", "/api/jobs/current");   // 새로고침해도 진행 중인 작업 이어서 표시
  if (cur) {
    $("#vehicleSel").value = cur.vehicle_id;
    $("#viewerTitle").textContent = $("#vehicleSel").selectedOptions[0]?.textContent || cur.vehicle_name;
    setRunning(true); renderJob(cur); startPolling(cur.id);
  } else {
    await showVehicle();
  }
})();
