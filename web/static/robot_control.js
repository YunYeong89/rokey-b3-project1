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
    const url = window.APP.rosbridgeUrl || `${location.protocol === "https:" ? "wss" : "ws"}://${location.hostname}:9090`;
    this.ros = new ROSLIB.Ros({ url });
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
      this.emit();
    });
    this.ros.on("error", () => {});
    this.ros.on("close", () => {
      this.connected = false;
      this.received = 0;
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

  stop() {
    if (!this.connected) throw new Error("rosbridge 연결을 확인하세요");
    this.commands.publish(new ROSLIB.Message({ data: JSON.stringify({ cmd: "stop" }) }));
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
        input.replaceChildren(...field.choices.map((value) => new Option(value, value)));
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
      if (input.type === "number" && !input.value) continue;
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
      this.select.replaceChildren(...this.catalog.map((item) => new Option(item.label, item.id)));
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
    const planOnly = this.fields.querySelector('[data-parameter="plan_only"]')?.checked;
    this.startButton.disabled = !robotControl.live || (!robotControl.ready && !planOnly) || busy || !!this.pending;
    this.stopButton.disabled = !robotControl.connected || !state?.managed || state.state === "stopping";
    this.select.disabled = !!busy || !!this.pending;
    this.fields.querySelectorAll("input, select").forEach((input) => { input.disabled = !!busy || !!this.pending; });
    this.status.textContent = !robotControl.connected ? "rosbridge 연결 대기"
      : !robotControl.live ? "ROS 제어 노드 응답 대기"
      : state.external_tasks?.length ? `터미널 작업 실행 중: ${state.external_tasks.join(", ")}`
      : this.pending ? "실행 접수 대기"
      : `${state.message} · ${robotControl.ready ? "가상 로봇 준비됨" : "가상 로봇 준비 대기"}${state.task ? ` · ${state.task}` : ""}`;
    if (state?.log !== this.log.textContent) this.log.textContent = state?.log || "";
  }
}
