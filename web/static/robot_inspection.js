import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { robotControl } from "./robot_control.js";

const PARTS = { bonnet: "보닛", left_door: "왼쪽 도어", right_door: "오른쪽 도어" };
const PHASE = { waiting_pose: "차량 배치 확인 대기", scanning: "탐색 중" };

class InspectionCanvas {
  constructor(canvas, add) {
    this.canvas = canvas;
    this.add = add;
    this.meshes = new Map();
    new ResizeObserver(() => this.draw()).observe(canvas);
    for (const type of ["pointerdown", "pointermove", "pointerup", "pointercancel"])
      canvas.addEventListener(type, (event) => this[type](event));
  }

  async show(part) {
    if (this.part === part && this.model) return;
    this.part = part;
    this.model = this.stroke = null;
    this.draw();
    const door = part !== "bonnet", name = door ? "door" : "bonnet";
    if (!this.meshes.has(name)) {
      const load = new STLLoader().loadAsync(`/api/robot/scene/real_${name}.stl`)
        .then((geometry) => {
          const vertices = geometry.getAttribute("position").array;
          geometry.dispose();
          return vertices;
        })
        .catch((error) => { this.meshes.delete(name); throw error; });
      this.meshes.set(name, load);
    }
    const vertices = await this.meshes.get(name);
    if (part !== this.part) return;
    const triangles = [], bounds = [Infinity, Infinity, -Infinity, -Infinity];
    for (let i = 0; i < vertices.length; i += 9) {
      if (door && (vertices[i+1] + vertices[i+4] + vertices[i+7]) * (part === "left_door" ? 1 : -1) <= 0)
        continue;
      const a = door ? 2 : 1;
      const points = [0, 3, 6].map((j) => [vertices[i+j], vertices[i+j+a]]);
      triangles.push(points);
      for (const [x, y] of points) {
        bounds[0] = Math.min(bounds[0], x); bounds[2] = Math.max(bounds[2], x);
        bounds[1] = Math.min(bounds[1], y); bounds[3] = Math.max(bounds[3], y);
      }
    }
    if (!triangles.length) throw new Error("부품 CAD가 비어 있습니다.");
    const scanTop = door ? (bounds[1] + bounds[3]) / 2 : bounds[3];
    this.model = { triangles, bounds, scanTop };
    this.draw();
  }

  draw() {
    const c = this.canvas, w = c.clientWidth, h = c.clientHeight;
    if (!w || !h) return;
    const dpr = Math.min(devicePixelRatio, 2);
    if (c.width !== Math.round(w*dpr) || c.height !== Math.round(h*dpr)) {
      c.width = Math.round(w*dpr); c.height = Math.round(h*dpr);
    }
    const ctx = c.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    if (!this.model) return;

    const { bounds: [xmin, ymin, xmax, ymax], triangles } = this.model;
    const scale = Math.min((w-24)/(xmax-xmin), (h-24)/(ymax-ymin));
    const tx = w/2 - (xmin+xmax)*scale/2, ty = h/2 + (ymin+ymax)*scale/2;
    const px = ([x,y]) => [tx + x*scale, ty - y*scale];
    this.projection = { scale, tx, ty };

    if (this.model.layer?.width !== c.width || this.model.layer.height !== c.height) {
      const layer = document.createElement("canvas");
      layer.width = c.width; layer.height = c.height;
      const cad = layer.getContext("2d");
      cad.scale(dpr, dpr);
      cad.fillStyle = "#4b6177";
      cad.strokeStyle = "#4b6177";
      for (const tri of triangles) {
        cad.beginPath();
        cad.moveTo(...px(tri[0]));
        cad.lineTo(...px(tri[1]));
        cad.lineTo(...px(tri[2]));
        cad.closePath();
        cad.fill();
      }
      this.model.layer = layer;
    }
    ctx.drawImage(this.model.layer, 0, 0, w, h);
    if (this.model.scanTop < ymax) {
      const [left, top] = px([xmin, ymax]), [right, bottom] = px([xmax, this.model.scanTop]);
      ctx.fillStyle = "#f2f6fac4";
      ctx.fillRect(left, top, right-left, bottom-top);
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = "#4b6177";
      ctx.beginPath(); ctx.moveTo(left, bottom); ctx.lineTo(right, bottom); ctx.stroke();
      ctx.setLineDash([]);
    }

    const box = ([x0, y0, x1, y1], label, preview=false) => {
      const [x,y] = px([x0,y1]), width = (x1-x0)*scale, height = (y1-y0)*scale;
      ctx.fillStyle = preview ? "#38bdf84a" : "#007faf56";
      ctx.strokeStyle = preview ? "#0d739a" : "#006480";
      ctx.lineWidth = 1.5;
      ctx.fillRect(x, y, width, height);
      ctx.strokeRect(x, y, width, height);
      if (label) {
        ctx.font = "600 12px system-ui";
        ctx.textAlign = "center";
        ctx.fillStyle = "#063044";
        ctx.fillText(label, x+width/2, y+height/2+4);
      }
    };
    this.regions?.forEach((region, i) => box(region, String(i+1)));
    if (this.stroke) {
      box(this.rectangle(this.stroke), "", true);
      ctx.beginPath();
      ctx.moveTo(...px(this.stroke[0]));
      this.stroke.slice(1).forEach((p) => ctx.lineTo(...px(p)));
      ctx.strokeStyle = "#e0f7ff";
      ctx.stroke();
    }
  }

  point(event, maxY = this.model.scanTop) {
    const r = this.canvas.getBoundingClientRect(), { scale, tx, ty } = this.projection;
    const [x0,y0,x1] = this.model.bounds;
    return [
      Math.min(x1, Math.max(x0, (event.clientX-r.left-tx)/scale)),
      Math.min(maxY, Math.max(y0, (ty-event.clientY+r.top)/scale)),
    ];
  }

  rectangle(points) {
    const [x0,y0,x1] = this.model.bounds;
    const interval = (coord, lo, hi) => {
      const min = Math.min(...points.map((p) => p[coord])), max = Math.max(...points.map((p) => p[coord]));
      const size = Math.min(hi-lo, Math.max(4, max-min));
      const first = Math.max(lo, Math.min(hi-size, (min+max-size)/2));
      return [first, first+size];
    };
    const [left,right] = interval(0, x0, x1), [bottom,top] = interval(1, y0, this.model.scanTop);
    return [left,bottom,right,top];
  }

  pointerdown(event) {
    if (!this.model || !this.projection || this.disabled || this.stroke || event.button !== 0) return;
    const point = this.point(event, this.model.bounds[3]);
    if (point[1] > this.model.scanTop) return;
    this.canvas.setPointerCapture(event.pointerId);
    this.pointer = event.pointerId;
    this.stroke = [point];
    this.draw();
  }

  pointermove(event) {
    if (!this.stroke || event.pointerId !== this.pointer) return;
    this.stroke.push(this.point(event));
    this.draw();
  }

  pointerup(event) {
    if (!this.stroke || event.pointerId !== this.pointer) return;
    this.stroke.push(this.point(event));
    const box = this.rectangle(this.stroke);
    this.stroke = null;
    this.add(box);
  }

  pointercancel(event) {
    if (event.pointerId !== this.pointer) return;
    this.stroke = null;
    this.draw();
  }
}

export class RobotInspection {
  constructor(root) {
    this.root = root;
    this.el = (name) => root.querySelector(`[data-inspection-${name}]`);
    this.plan = [];
    this.error = this.el("error");
    this.canvas = new InspectionCanvas(this.el("draw"), (box) => {
      this.current.regions.push(box);
      this.current.mode = "partial";
      this.render();
    });
    this.el("part").onchange = () => this.selectPart();
    this.el("robot").onchange = (event) => robotControl.select(event.target.value);
    this.el("plan-only").onchange = () => this.refresh();
    this.el("full").onclick = () => this.send([{ part: this.current.part, regions: [] }]);
    this.el("partial").onclick = () => this.send([{ part: this.current.part, regions: this.current.regions }]);
    this.el("run").onclick = () => this.runPlan();
    this.el("save").onclick = () => this.save();
    this.el("continue").onclick = () => {
      this.error.textContent = "";
      try { robotControl.continueInspection(robotControl.status.request_id); }
      catch (error) { this.error.textContent = error.message; }
    };
    robotControl.addEventListener("state", () => this.refresh());
    this.refresh();
  }

  get current() { return this.plan.find((item) => item.part === this.el("part").value); }

  async load() {
    if (this.loaded) return;
    this.loaded = true;
    try {
      const [plan, vehicles] = await Promise.all([api("GET", "/api/robot/inspection-plan"), api("GET", "/api/vehicles")]);
      this.plan = plan;
      this.el("vehicle").append(...vehicles.map((v) => new Option(`${v.name} · ${v.model}`, v.id)));
      this.selectPart();
    } catch (error) {
      this.error.textContent = error.message;
      this.loaded = false;
    }
  }

  async selectPart() {
    const part = this.current?.part;
    if (!part) return;
    this.el("draw-note").textContent = part === "bonnet" ? "보닛 평면에서 검사할 부분을 그리세요."
      : `${PARTS[part]} 하단에서 검사할 부분을 그리세요.`;
    this.render();
    try { await this.canvas.show(part); }
    catch (error) {
      if (part === this.current?.part) this.el("draw-note").textContent = "부품 CAD를 불러오지 못했습니다.";
    }
    this.refresh();
  }

  render() {
    const item = this.current;
    this.canvas.regions = item?.regions || [];
    this.canvas.draw();
    this.el("regions").replaceChildren(...(item?.regions || []).map(([x0,y0,x1,y1], i) => {
      const row = document.createElement("li");
      const size = `${(x1-x0).toFixed(1)} × ${(y1-y0).toFixed(1)} mm`;
      row.append(document.createTextNode(size), this.button("삭제", () => {
        item.regions.splice(i, 1);
        this.render();
      }, `영역 ${i+1} 삭제`));
      return row;
    }));
    this.el("order").replaceChildren(...this.plan.map((entry, i) => {
      const row = document.createElement("li");
      const info = document.createElement("div");
      const label = document.createElement("label");
      const enabled = Object.assign(document.createElement("input"), { type: "checkbox", checked: entry.enabled });
      enabled.onchange = () => { entry.enabled = enabled.checked; this.render(); };
      label.append(enabled, document.createTextNode(PARTS[entry.part]));
      const move = (j) => () => {
        [this.plan[i], this.plan[j]] = [this.plan[j], this.plan[i]];
        this.render();
      };
      const up = this.button("↑", move(i-1), `${PARTS[entry.part]} 순서 앞으로`);
      const down = this.button("↓", move(i+1), `${PARTS[entry.part]} 순서 뒤로`);
      up.dataset.fixed = i === 0 ? "true" : "";
      down.dataset.fixed = i === this.plan.length-1 ? "true" : "";
      info.append(label, up, down);
      row.append(info);
      const mode = Object.assign(document.createElement("select"), { ariaLabel: `${PARTS[entry.part]} 탐색 방식` });
      mode.append(new Option("전체", "full"), new Option(`부분 (${entry.regions.length}개)`, "partial"));
      mode.value = entry.mode;
      mode.onchange = () => { entry.mode = mode.value; this.render(); };
      row.append(mode);
      if (entry.mode === "partial") {
        const regions = document.createElement("ol");
        for (const [n] of entry.regions.entries()) {
          const region = document.createElement("li");
          region.textContent = `영역 ${n+1}`;
          regions.append(region);
        }
        row.append(regions);
      }
      return row;
    }));
    this.refresh();
  }

  button(text, action, label) {
    const button = Object.assign(document.createElement("button"), {
      type: "button", textContent: text, className: "btn ghost-dark small", ariaLabel: label,
    });
    button.onclick = action;
    return button;
  }

  async save() {
    this.error.textContent = "";
    const snapshot = JSON.stringify(this.plan);
    try {
      await api("PUT", "/api/robot/inspection-plan", JSON.parse(snapshot));
      this.saved = performance.now();
      this.refresh();
    } catch (error) { this.error.textContent = error.message; }
  }

  runPlan() {
    const enabled = this.plan.filter((item) => item.enabled);
    const missing = enabled.find((item) => item.mode === "partial" && !item.regions.length);
    if (missing) {
      this.error.textContent = `${PARTS[missing.part]}의 부분 탐색 영역을 그리세요.`;
      this.el("part").value = missing.part;
      this.selectPart();
      return;
    }
    this.send(enabled.map((item) => ({ part: item.part, regions: item.mode === "partial" ? item.regions : [] })));
  }

  send(parts) {
    if (!parts.length || !this.el("vehicle").reportValidity()) return;
    this.error.textContent = "";
    try {
      const id = `${Date.now()}-${Math.random().toString(16).slice(2)}`;
      this.pending = { id, until: performance.now()+6000 };
      robotControl.inspect(parts, this.el("plan-only").checked, id, Number(this.el("vehicle").value));
    } catch (error) { this.pending = null; this.error.textContent = error.message; }
    this.refresh();
  }

  refresh() {
    const state = robotControl.status, inspection = robotControl.live ? state?.inspection : null;
    const robotIds = [...new Set([robotControl.selected, ...robotControl.available])].sort();
    if (this.robotKey !== robotIds.join()) {
      this.robotKey = robotIds.join();
      this.el("robot").replaceChildren(...robotIds.map((id) => new Option(id === "dsr01" ? "DSR01" : id, id)));
    }
    this.el("robot").value = robotControl.selected;

    if (state?.event === "rejected" && state.rejected_request &&
        (state.rejected_request === this.pending?.id || state.rejected_request === state.request_id)) {
      this.error.textContent = state.error;
      this.pending = null;
    }
    if (this.pending && this.pending.id === state?.request_id) this.pending = null;
    if (this.pending && performance.now() > this.pending.until) {
      this.error.textContent = "검사 명령 접수를 확인하지 못했습니다.";
      this.pending = null;
    }

    const busy = !!(robotControl.live && state?.busy || this.pending);
    const canMove = robotControl.ready && state?.probe_attached === true;
    const canRun = !!this.current && robotControl.live && !busy &&
      (canMove || this.el("plan-only").checked);
    const confirm = inspection?.phase === "waiting_pose";
    this.el("continue").hidden = !confirm || state?.state === "stopping";
    this.el("continue").disabled = !canMove;
    this.el("full").disabled = !canRun;
    this.el("partial").disabled = !canRun || !this.current.regions.length;
    this.el("run").disabled = !canRun || !this.plan.some((item) => item.enabled);
    this.el("save").disabled = busy || !this.plan.length;
    this.el("robot").disabled = !!(this.pending || robotControl.live && state?.managed);
    this.canvas.disabled = busy;
    for (const name of ["vehicle", "part", "plan-only"]) this.el(name).disabled = busy;
    for (const control of this.root.querySelectorAll("[data-inspection-order] input, [data-inspection-order] select, "
        + "[data-inspection-order] button, [data-inspection-regions] button"))
      control.disabled = busy || control.dataset.fixed === "true";
    const width = state?.gripper_width_mm;
    this.el("probe").textContent = robotControl.live ? (
      (state.probe_attached === true ? "탐침 파지" : state.probe_attached === false ? "탐침 미파지" : "탐침 상태 확인 중")
      + (Number.isFinite(width) ? ` · RG2 ${width.toFixed(1)} mm` : "")
    ) : "";
    const prefix = inspection
      ? `${inspection.step}/${inspection.total} · ${inspection.label}${inspection.region ? ` · 영역 ${inspection.region}` : ""}`
      : "";
    const progress = state?.scan && inspection ? ` · ${Math.round(state.scan.progress*100)}%` : "";
    this.el("status").textContent = !robotControl.live ? "로봇 연결 대기" : this.pending ? "명령 접수 대기"
      : state.state === "stopping" ? state.message : confirm
        ? `${prefix} · 검사할 면이 로봇을 향하게 차량을 배치하세요.`
        : inspection ? `${prefix} · ${state.plan_only ? "경로 계획 중" : PHASE[inspection.phase]}${progress}`
        : state.task === "real_scan" && ["failed", "succeeded"].includes(state.state) ? state.message
        : performance.now() - (this.saved ?? -Infinity) < 3000 ? "탐색 순서를 저장했습니다."
        : state.probe_attached === false && !this.el("plan-only").checked ? "탐색 전에 탐침을 수동으로 파지하세요."
        : state.task === "real_scan" ? state.message : "탐색 대기";
  }
}
