/* 관리자·사용자 화면 공통 함수 */
"use strict";

const STAGES = ["calibrate", "scan", "detect", "stock", "replace", "inspect", "done"];
const PART_NAMES = { bonnet: "보닛", mirror_l: "사이드미러(좌)", mirror_r: "사이드미러(우)" };
const STATUS_TEXT = { running: "진행 중", done: "수리 완료", no_damage: "손상 없음", on_hold: "수리 보류", failed: "실패", stopped: "정지됨" };
const MODE_TEXT = { auto: "자동", manual: "수동" };
const DECISION_TEXT = { dent: "손상(찌그러짐)", missing: "부품 없음", suspect: "의심", requested: "교체 지정(수동)" };

const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));
const num = (v, d = 0) => (v === null || v === undefined ? "–" : Number(v).toFixed(d));
const partName = (c) => PART_NAMES[c] || c;
function esc(s) { return String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c])); }

async function api(method, path, body) {
  const res = await fetch(path, {
    method, headers: { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  if (res.status === 401) { location.href = "/login"; throw new Error("login"); }
  const data = await res.json().catch(() => ({}));
  if (!res.ok) { const e = new Error(data.error || res.statusText); e.status = res.status; throw e; }
  return data;
}

function table(head, rows, empty = "없음") {
  if (!rows.length) return `<p class="note">${empty}</p>`;
  return `<table class="tbl"><thead><tr>${head.map((h) => `<th>${h}</th>`).join("")}</tr></thead>
    <tbody>${rows.map((r) => `<tr>${r.map((c) => `<td>${esc(c)}</td>`).join("")}</tr>`).join("")}</tbody></table>`;
}

/* 진행 단계 목록(<ol id=steps> 의 li[data-stage])을 작업 상태에 맞춰 칠한다 */
function paintSteps(listSel, job) {
  const seen = job ? job.events.map((e) => e.stage).filter((s) => STAGES.includes(s)) : [];
  const lastIdx = seen.length ? STAGES.indexOf(seen[seen.length - 1]) : -1;
  const errStage = job ? job.events.filter((e) => e.level === "error").map((e) => e.stage).pop() : null;
  $$(listSel + " li").forEach((li, i) => {
    const s = li.dataset.stage;
    li.className = "";
    if (!job) return;
    if (job.status === "running") {
      if (i < lastIdx) li.classList.add(seen.includes(s) ? "done" : "skip");
      else if (i === lastIdx) li.classList.add("active");
    } else {
      if (seen.includes(s)) li.classList.add("done");
      else if (i < lastIdx || ["no_damage", "on_hold", "stopped", "failed"].includes(job.status)) li.classList.add("skip");
      if (s === errStage) { li.classList.remove("done"); li.classList.add("err"); }
    }
  });
  if (!job) return 0;
  return job.status === "running" ? Math.max(0, lastIdx) / (STAGES.length - 1) * 100 : 100;
}

/* 작업의 부품별 상태: replaced > missing > dent > requested */
function partStates(job) {
  const st = {};
  if (!job) return st;
  for (const f of job.findings) {
    const rank = { requested: 1, suspect: 1, dent: 2, missing: 3 }[f.decision] || 0;
    if (!st[f.part_code] || rank > st[f.part_code].rank) st[f.part_code] = { state: f.decision, rank };
  }
  for (const r of job.replacements) st[r.part_code] = { state: r.success ? "replaced" : "failed", rank: 9 };
  return Object.fromEntries(Object.entries(st).map(([k, v]) => [k, v.state]));
}

function logHtml(job) {
  return job.events.slice().reverse().map((e) =>
    `<li class="${e.level}"><span class="t">${esc((e.ts || "").slice(11))}</span><span class="s">${esc(e.stage)}</span>${esc(e.message)}</li>`).join("");
}

function jobDetailHtml(j) {
  return `<h2>수리 #${j.id} · ${esc(j.vehicle_name)} <span class="badge ${j.status}">${STATUS_TEXT[j.status]}</span>
      <span class="badge mode">${MODE_TEXT[j.mode]} 모드</span></h2>
    <p class="note">시작 ${esc(j.started_at)} · 종료 ${esc(j.finished_at || "–")} · 담당 ${esc(j.operator_name)}</p>
    <h3>손상 판정 / 교체 지정</h3>${table(["부품", "x [mm]", "y [mm]", "깊이 [mm]", "판정"], j.findings.map((f) =>
      [partName(f.part_code), num(f.x), num(f.y), num(f.depth, 2), DECISION_TEXT[f.decision] || f.decision]))}
    <h3>부품 교체</h3>${table(["부품", "결과", "재시도", "최대 힘 [N]", "단차 [mm]", "시각"], j.replacements.map((r) =>
      [partName(r.part_code), r.success ? "성공" : "실패", r.retries, num(r.peak_force, 1), num(r.flush_mm, 2), r.ts]))}
    <h3>로그</h3>${table(["시각", "단계", "수준", "내용"], j.events.map((e) => [e.ts, e.stage, e.level, e.message]))}`;
}

/* 수리 현황 카드 (관리자·고객 공용): 상태 배지·시간·결과·교체 부품 목록 */
function renderStatusCard(job) {
  const badge = $("#jobBadge"), modeB = $("#jobMode"), box = $("#resultBox");
  if (!job) {
    badge.className = "badge"; badge.textContent = "수리 기록 없음";
    modeB.classList.add("hidden"); $("#jobTime").textContent = "";
    box.className = "result hidden"; $("#log").innerHTML = ""; $("#partsList").innerHTML = `<p class="note">–</p>`;
    return;
  }
  badge.className = "badge " + job.status; badge.textContent = STATUS_TEXT[job.status];
  modeB.classList.remove("hidden"); modeB.textContent = `${MODE_TEXT[job.mode]} 모드`;
  $("#jobTime").textContent = `수리 #${job.id} · 시작 ${job.started_at}${job.finished_at ? " · 종료 " + job.finished_at : ""}`;
  if (job.status === "running") box.className = "result hidden";
  else { box.className = "result " + job.status; box.textContent = `${STATUS_TEXT[job.status]}${job.summary ? " — " + job.summary : ""}`; }
  $("#log").innerHTML = logHtml(job);
  $("#partsList").innerHTML = partsListHtml(job);
}

function partsListHtml(job) {
  const st = partStates(job);
  const rep = Object.fromEntries(job.replacements.map((r) => [r.part_code, r]));
  const codes = Object.keys(st);
  if (!codes.length) return `<p class="note">${job.status === "running" ? "검사 중입니다." : "교체한 부품이 없습니다."}</p>`;
  return codes.map((c) => {
    const r = rep[c];
    const txt = r ? (r.success ? `교체 완료 · 단차 ${num(r.flush_mm, 2)} mm 합격` : `재검사 불합격 (단차 ${num(r.flush_mm, 2)} mm)`)
                  : { dent: "찌그러짐 발견 → 교체 예정", missing: "부품 없음 → 교체 예정", requested: "교체 예정", suspect: "확인 중" }[st[c]] || "";
    return `<div class="part-row ${st[c]}"><b>${partName(c)}</b><span>${esc(txt)}</span></div>`;
  }).join("");
}
