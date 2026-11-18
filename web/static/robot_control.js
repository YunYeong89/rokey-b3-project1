/** rosbridge on the page's host: ws for an http page, wss for an https one (run_web.sh --lan relays both). */
export function rosbridgeUrl() {
  if (window.APP.rosbridgeUrl) return window.APP.rosbridgeUrl;
  const secure = location.protocol === "https:";
  const port = secure ? window.APP.rosbridgeTlsPort || 9443 : window.APP.rosbridgePort || 9090;
  return `${secure ? "wss" : "ws"}://${location.hostname}:${port}`;
}

/** Shared command connection: readiness comes from web_control's heartbeat, not the WebSocket alone. */
class RobotControlClient extends EventTarget {
  constructor() {
    super();
    this.received = 0;
    this.connected = false;
    this.connect();
    setInterval(() => this.emit(), 500);
  }

  connect() {
    this.ros = new ROSLIB.Ros({ url: rosbridgeUrl() });
    this.ros.on("connection", () => {
      this.connected = true;
      this.received = 0;
      this.commands = new ROSLIB.Topic({ ros: this.ros, name: "/robot/command", messageType: "std_msgs/msg/String" });
      this.repairs = new ROSLIB.Topic({ ros: this.ros, name: "/repair/command", messageType: "std_msgs/msg/String" });
      this.commands.advertise();
      this.repairs.advertise();
      new ROSLIB.Topic({ ros: this.ros, name: "/robot/status", messageType: "std_msgs/msg/String" })
        .subscribe((message) => {
          this.status = JSON.parse(message.data);
          this.received = performance.now();
          this.emit();
        });
      new ROSLIB.Topic({ ros: this.ros, name: "/repair/status", messageType: "std_msgs/msg/String", throttle_rate: 50 })
        .subscribe((message) => this.dispatchEvent(new CustomEvent("repair", { detail: JSON.parse(message.data) })));
      this.dispatchEvent(new CustomEvent("connection", { detail: this.ros }));  // 같은 연결을 쓰는 화면 (로봇팔 3D)
      this.emit();
    });
    this.ros.on("error", () => {});
    this.ros.on("close", () => {
      this.connected = false;
      this.received = 0;
      this.dispatchEvent(new Event("close"));
      this.emit();
      this.retry = setTimeout(() => this.connect(), 2000);
    });
  }

  get live() { return this.connected && performance.now() - this.received < 2500 && this.received > 0; }
  get ready() { return this.live && this.status?.ready; }
  emit() { this.dispatchEvent(new CustomEvent("state", { detail: this.status })); }

  start(task, parameters, requestId) {
    if (!this.live) throw new Error("ROS 제어 노드에 연결되지 않았습니다");
    this.commands.publish(new ROSLIB.Message({ data: JSON.stringify({ cmd: "start", task, parameters, request_id: requestId }) }));
  }

  /** Robot stop: move_stop, and the program started from the web is interrupted (web_control). */
  stop() {
    if (!this.connected) throw new Error("rosbridge 연결을 확인하세요");
    this.commands.publish(new ROSLIB.Message({ data: JSON.stringify({ cmd: "stop" }) }));
  }

  /** Live force graph (draw_graph) beside any program. */
  graph(on, compensated) {
    if (!this.live) throw new Error("ROS 제어 노드에 연결되지 않았습니다");
    this.commands.publish(new ROSLIB.Message({ data: JSON.stringify({ cmd: "graph", on, compensated }) }));
  }

  repair(command) {
    if (!this.live) throw new Error("ROS 제어 노드에 연결되지 않았습니다");
    this.repairs.publish(new ROSLIB.Message({ data: JSON.stringify(command) }));
  }
}

export const robotControl = new RobotControlClient();

export class RobotTaskPanel {
  constructor(panel) {
    this.panel = panel;
    this.select = panel.querySelector("[data-task-select]");
    this.fields = panel.querySelector("[data-task-fields]");
    this.startButton = panel.querySelector("[data-task-start]");
    this.stopButton = panel.querySelector("[data-task-stop]");
    this.status = panel.querySelector("[data-task-status]");
    this.log = panel.querySelector("[data-task-log]");
    this.error = panel.querySelector("[data-task-error]");
    this.select.addEventListener("change", () => this.renderFields());
    this.fields.addEventListener("change", () => this.refresh());
    this.startButton.addEventListener("click", () => this.start());
    this.stopButton.addEventListener("click", () => {
      try { robotControl.stop(); } catch (error) { this.error.textContent = error.message; }
    });
    robotControl.addEventListener("state", () => this.refresh());
    this.refresh();
  }

  renderFields() {
    const command = this.catalog.find((item) => item.id === this.select.value);
    this.fields.replaceChildren();
    for (const [key, field] of Object.entries(command?.options || {})) {
      const label = document.createElement("label");
      label.textContent = field.label;
      let input;
      if (field.type === "select") {
        input = document.createElement("select");
        input.replaceChildren(...(field.empty ? [new Option(field.empty, "")] : []), ...field.choices.map((value) =>
          new Option(field.virtual?.includes(value) ? `${value} (실기: 계획만)` : value, value)));
      } else {
        input = document.createElement("input");
        input.type = field.type === "boolean" ? "checkbox" : "number";
        if (input.type === "number") {
          input.min = field.min; input.max = field.max;
          input.step = field.type === "integer" ? "1" : "any";
          input.placeholder = "설정 파일 기본값";
        } else label.className = "robot-check";
      }
      input.dataset.parameter = key;
      label.append(input);
      this.fields.append(label);
    }
    this.refresh();
  }

  start() {
    const parameters = {};
    for (const input of this.fields.querySelectorAll("[data-parameter]")) {
      if (!input.reportValidity()) return;
      if ((input.type === "number" || input.tagName === "SELECT") && !input.value) continue;
      parameters[input.dataset.parameter] = input.type === "checkbox" ? input.checked
        : input.type === "number" ? Number(input.value) : input.value;
    }
    this.error.textContent = "";
    try {
      this.pending = { id: `${Date.now()}-${Math.random().toString(16).slice(2)}`, until: performance.now() + 6000 };
      robotControl.start(this.select.value, parameters, this.pending.id);
    } catch (error) { this.pending = null; this.error.textContent = error.message; }
    this.refresh();
  }

  refresh() {
    const state = robotControl.status;
    if (state?.catalog && !this.catalog) {
      this.catalog = state.catalog;
      this.select.replaceChildren(...this.catalog.map((item) =>
        new Option(item.virtual ? `${item.label} (시뮬레이션 전용)` : item.label, item.id)));
      this.renderFields();
    }
    if (this.pending && state?.event === "rejected" && state.rejected_request === this.pending.id) {
      this.error.textContent = state.error;
      this.pending = null;
    }
    if (this.pending && state?.request_id === this.pending.id) this.pending = null;
    if (this.pending && performance.now() > this.pending.until) {
      this.error.textContent = "실행 접수를 확인하지 못했습니다. 상태와 로그를 확인하세요.";
      this.pending = null;
    }
    const busy = state?.busy && robotControl.live;
    const robot = { virtual: "가상 로봇", real: "실제 로봇" }[state?.mode] || "로봇";
    const planOnly = this.fields.querySelector('[data-parameter="plan_only"]')?.checked;
    this.startButton.disabled = !robotControl.live || (!robotControl.ready && !planOnly) || busy || !!this.pending;
    this.stopButton.disabled = !robotControl.live || state.state === "stopping";
    this.select.disabled = !!busy || !!this.pending;
    this.fields.querySelectorAll("input, select").forEach((input) => { input.disabled = !!busy || !!this.pending; });
    this.status.textContent = !robotControl.connected ? "rosbridge 연결 대기"
      : !robotControl.live ? "ROS 제어 노드 응답 대기"
      : state.external_tasks?.length ? `터미널 작업 실행 중: ${state.external_tasks.join(", ")}`
      : this.pending ? "실행 접수 대기"
      : `${state.message} · ${robot} ${robotControl.ready ? "준비됨" : "준비 대기"}`
        + `${state.probe_attached ? "" : state.mode === "real" ? ` · 탐침 TCP(${state.probe_tcp}) 미선택` : " · 탐침 미파지"}`
        + `${state.task ? ` · ${state.task}` : ""}`;
    if (state?.event === "robot_stopped") this.error.textContent = `로봇 정지 명령 전송 (${state.stop_sent}건)`;
    if (state?.log !== this.log.textContent) this.log.textContent = state?.log || "";
  }
}
