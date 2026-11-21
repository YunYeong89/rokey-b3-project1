/* 고객(차주) 화면 (SCR-201, SCR-202) — 3D 뷰어는 viewer3d.js (관리자 화면과 공용) */
import { createCarViewer } from "./viewer3d.js";

const ui = { vehicles: [], vehicleId: null, job: null, timer: null };

const viewer = createCarViewer($("#viewer"), { msgEl: $("#viewerMsg") });
$$("[data-view]").forEach((b) => b.addEventListener("click", () => viewer.setView(b.dataset.view)));

// ================================================================ 화면 데이터
function renderStatus(job) {
  ui.job = job;
  pollCloud();
  viewer.paint(partStates(job), job ? job.findings : [], !!job && job.status === "running");
  $("#progBar").style.width = paintSteps("#steps", job) + "%";
  renderStatusCard(job);
}

// 진행 중인 수리의 실시간 측정점 (로봇이 보닛을 훑은 자리, 2초마다). 끝난 작업이거나 체크를 끄면 지움
const cloud = { job: null, tag: null, at: 0, busy: false };
async function pollCloud() {
  const job = ui.job, show = $("#showScan")?.checked !== false;
  if (!job || job.status !== "running" || !show) {
    viewer.setScanCloud(null); cloud.job = cloud.tag = null; return;
  }
  if (cloud.busy || (cloud.job === job.id && performance.now() - cloud.at < 2000)) return;
  cloud.busy = true;
  try {
    if (cloud.job !== job.id) cloud.tag = null;
    const response = await fetch(`/api/jobs/${job.id}/cloud`, { headers: cloud.tag ? { "If-None-Match": cloud.tag } : {} });
    cloud.job = job.id; cloud.at = performance.now();
    if (response.status === 304) return;
    if (!response.ok || response.status === 204) { viewer.setScanCloud(null); return; }
    const data = await response.json();
    cloud.tag = response.headers.get("ETag");
    viewer.setScanCloud(data.points, data.size);
  } catch (e) { console.warn(e); }
  finally { cloud.busy = false; }
}
$("#showScan")?.addEventListener("change", () => { cloud.tag = null; cloud.at = 0; pollCloud(); });

async function refresh() {
  if (!ui.vehicleId) return;
  try {
    await viewer.load(ui.vehicles.find((v) => v.id === ui.vehicleId)?.model);   // 차종에 맞는 축소 모델
    let job = await api("GET", `/api/jobs/current?vehicle_id=${ui.vehicleId}`);
    if (!job) {
      const last = await api("GET", `/api/jobs?vehicle_id=${ui.vehicleId}&limit=1`);
      job = last.length ? await api("GET", `/api/jobs/${last[0].id}`) : null;
    }
    renderStatus(job);
  } catch (e) { console.warn(e); }
}

async function loadHistory() {
  const js = await api("GET", "/api/jobs");
  $("#jobsBody").innerHTML = js.map((j) => `<tr class="click" data-id="${j.id}">
    <td>${j.id}</td><td>${esc(j.vehicle_name)}</td><td>${MODE_TEXT[j.mode]}</td>
    <td>${esc(j.started_at)}</td><td>${esc(j.finished_at || "–")}</td>
    <td><span class="badge ${j.status}">${STATUS_TEXT[j.status]}</span></td>
    <td>${(j.replaced || "").split(",").filter(Boolean).map(partName).join(", ") || "–"}</td></tr>`).join("")
    || `<tr><td colspan="7" class="note">아직 수리 기록이 없습니다.</td></tr>`;
  $$("#jobsBody tr.click").forEach((tr) => tr.addEventListener("click", async () => {
    const box = $("#jobDetail");
    box.innerHTML = jobDetailHtml(await api("GET", `/api/jobs/${tr.dataset.id}`));
    box.classList.remove("hidden");
    box.scrollIntoView({ behavior: "smooth" });
  }));
}

$$(".tab").forEach((b) => b.addEventListener("click", () => {
  $$(".tab").forEach((x) => x.classList.toggle("active", x === b));
  $$(".panel").forEach((p) => p.classList.toggle("active", p.id === "tab-" + b.dataset.tab));
  if (b.dataset.tab === "history") loadHistory();
  if (b.dataset.tab === "status") requestAnimationFrame(viewer.resize);
}));

$("#myVehicle").addEventListener("change", (e) => {
  ui.vehicleId = Number(e.target.value);
  $("#viewerTitle").textContent = e.target.selectedOptions[0].textContent;
  refresh();
});

(async function init() {
  await viewer.load();
  ui.vehicles = await api("GET", "/api/vehicles");
  if (!ui.vehicles.length) {
    $("#myVehicle").innerHTML = `<option>등록된 차량 없음</option>`;
    $("#viewerTitle").textContent = "등록된 차량이 없습니다 (정비소에 문의)";
    renderStatus(null);
    return;
  }
  $("#myVehicle").innerHTML = ui.vehicles.map((v) => `<option value="${v.id}">${esc(v.name)}</option>`).join("");
  ui.vehicleId = ui.vehicles[0].id;
  $("#viewerTitle").textContent = ui.vehicles[0].name;
  await refresh();
  ui.timer = setInterval(refresh, 1500);
})();
