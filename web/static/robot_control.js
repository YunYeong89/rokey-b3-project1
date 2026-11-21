/** rosbridge on the page's host: ws for an http page, wss for an https one (run_web.sh --lan relays both). */
export function rosbridgeUrl() {
  if (window.APP.rosbridgeUrl) return window.APP.rosbridgeUrl;
  const secure = location.protocol === "https:";
  const port = secure ? window.APP.rosbridgeTlsPort || 9443 : window.APP.rosbridgePort || 9090;
  return `${secure ? "wss" : "ws"}://${location.hostname}:${port}`;
}

const COMMAND_TOPICS = ["/robot/command", "/repair/command", "/robot/teleop"];   // web -> web_control / teleop

/** Shared command connection: readiness comes from web_control's heartbeat, not the WebSocket alone. */
class RobotControlClient extends EventTarget {
  constructor() {
    super();
    this.selected = "dsr01";
    this.connected = false;
    this.robots = new Map();
    this.connect();
    setInterval(() => this.emit(), 500);
    setInterval(() => this.discover(), 4000);
  }

  connect() {
    const ros = this.ros = new ROSLIB.Ros({ url: rosbridgeUrl() });
    this.ros.on("connection", () => {
      if (ros !== this.ros) return;
      this.connected = true;
      this.outputs = new Map();
      this.watch("dsr01");
      if (this.selected !== "dsr01") this.watch(this.selected);
      this.discover();
      this.dispatchEvent(new CustomEvent("connection", { detail: ros }));
      this.emit();
    });
    this.ros.on("error", () => {});
    this.ros.on("close", () => {
      if (ros !== this.ros) return;
      this.connected = false;
      this.robots.clear();
      this.dispatchEvent(new Event("close"));
      this.emit();
      this.retry = setTimeout(() => this.connect(), 2000);
    });
  }

  reconnect() {
    clearTimeout(this.retry);
    const old = this.ros;
    this.connected = false;
    this.robots.clear();
    this.dispatchEvent(new Event("close"));
    this.connect();
    old?.close();
    this.emit();
  }

  topic(path, robot = this.selected) { return robot === "dsr01" ? path : `/${robot}${path}`; }

  jointTopic() {
    if (this.selected === "dsr01" && window.APP.robotJointTopic) return window.APP.robotJointTopic;
    return `/${this.selected}/${this.status?.mode === "real" ? "joint_states" : "sim_joint_states"}`;
  }

  watch(robot) {
    if (this.robots.has(robot)) return;
    const ros = this.ros, record = { received: -Infinity };
    this.robots.set(robot, record);
    for (const path of COMMAND_TOPICS) this.output(this.topic(path, robot));   // matched long before the first command
    new ROSLIB.Topic({ ros, name: this.topic("/robot/status", robot), messageType: "std_msgs/msg/String" })
      .subscribe((message) => {
        if (ros !== this.ros) return;
        record.status = JSON.parse(message.data);
        record.received = performance.now();
        this.emit();
      });
    new ROSLIB.Topic({ ros, name: this.topic("/repair/status", robot), messageType: "std_msgs/msg/String", throttle_rate: 50 })
      .subscribe((message) => {
        if (ros === this.ros && robot === this.selected)
          this.dispatchEvent(new CustomEvent("repair", { detail: JSON.parse(message.data) }));
      });
  }

  discover() {
    if (!this.connected) return;
    const ros = this.ros;
    ros.getTopics(({ topics }) => {
      if (ros !== this.ros || !this.connected) return;
      for (const name of topics) {
        const match = name.match(/^\/(robot_[1-9][0-9]*)\/robot\/status$/);
        if (match) this.watch(match[1]);
      }
    }, () => {});
  }

  get available() {
    return [...this.robots].filter(([, r]) => performance.now() - r.received < 2500).map(([id]) => id);
  }

  select(robot) {
    if (robot === this.selected) return;
    this.selected = robot;
    this.dispatchEvent(new Event("robot"));
    this.emit();
  }

  get status() { return this.robots.get(this.selected)?.status; }
  get live() { return this.connected && this.available.includes(this.selected); }
  get ready() { return this.live && this.status?.ready; }
  emit() { this.dispatchEvent(new CustomEvent("state", { detail: this.status })); }

  /** Advertised once and kept: rosbridge creates a publisher on the first publish and sends at once, before the
   *  subscriber (web_control, VOLATILE) is matched, so a first message on a new topic was lost (수리 시작 did nothing). */
  output(name) {
    if (!this.outputs.has(name)) {
      const topic = new ROSLIB.Topic({ ros: this.ros, name, messageType: "std_msgs/msg/String" });
      topic.advertise();
      this.outputs.set(name, topic);
    }
    return this.outputs.get(name);
  }

  publish(path, value) {
    this.output(this.topic(path)).publish(new ROSLIB.Message({ data: JSON.stringify(value) }));
  }

  start(task, parameters, requestId, vehicleId) {
    if (!this.live) throw new Error("ROS 제어 노드에 연결되지 않았습니다");
    this.publish("/robot/command", {
      cmd: "start", task, parameters, request_id: requestId, vehicle_id: vehicleId
    });
  }

  inspect(parts, planOnly, requestId, vehicleId) {
    if (!this.live) throw new Error("ROS 제어 노드에 연결되지 않았습니다");
    this.publish("/robot/command", { cmd: "inspect", parts, plan_only: planOnly, request_id: requestId, vehicle_id: vehicleId });
  }

  continueInspection(requestId) {
    if (!this.ready) throw new Error("로봇 준비 상태를 확인하세요");
    this.publish("/robot/command", { cmd: "inspection_continue", request_id: requestId });
  }

  /** Robot stop: move_stop, and the program started from the web is interrupted (web_control). */
  stop() {
    if (!this.connected) throw new Error("rosbridge 연결을 확인하세요");
    this.publish("/robot/command", { cmd: "stop" });
  }

  /** Live force graph (draw_graph) beside any program. */
  graph(on, compensated) {
    if (!this.live) throw new Error("ROS 제어 노드에 연결되지 않았습니다");
    this.publish("/robot/command", { cmd: "graph", on, compensated });
  }

  /** One key of the web teleop (rokey teleop --web): a held button repeats it, "" releases (stops at once). */
  teleop(key) {
    if (!this.connected) throw new Error("rosbridge 연결을 확인하세요");
    this.publish("/robot/teleop", { key });
  }

  repair(command) {
    if (!this.live) throw new Error("ROS 제어 노드에 연결되지 않았습니다");
    this.publish("/repair/command", command);
  }
}

export const robotControl = new RobotControlClient();

const SCAN_STAGES = { planning: "계획", moving: "이동", scanning: "측정", reporting: "결과 저장" };

/** real_scan progress (/scan/progress via web_control's status): stage, path and segment, share, points, force. */
function scanText(scan) {
  return ` · 스캔 ${SCAN_STAGES[scan.stage]} 경로 ${scan.path}/${scan.paths} 구간 ${scan.segment}/${scan.segments}`
    + ` · ${Math.round(scan.progress * 100)}% · 점 ${scan.points} · ${scan.force_n.toFixed(2)} N`
    + ` · 남은 약 ${Math.ceil(scan.remaining_s / 60)}분`;
}

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
    // "ROS 기능 제어" 카드는 화면에서 뺐음: 기능 선택·실행 칸이 없으면 로봇 정지 버튼과 실행 로그만 다룸
    this.select?.addEventListener("change", () => this.renderFields());
    this.fields?.addEventListener("change", () => this.refresh());
    this.startButton?.addEventListener("click", () => this.start());
    this.stopButton.addEventListener("click", () => {
      try { robotControl.stop(); } catch (error) { if (this.error) this.error.textContent = error.message; else alert(error.message); }
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
    if (!this.select) {   // 기능 실행 칸 없음: 정지 버튼 상태와 실행 로그만
      this.stopButton.disabled = !robotControl.live || state?.state === "stopping";
      if (this.log && state?.log !== this.log.textContent) this.log.textContent = state?.log || "";
      return;
    }
    if (state?.catalog && !this.catalog) {
      this.catalog = state.catalog.filter((item) => !item.panel && !item.virtual &&
        !["choose_car", "scan_surface", "real_scan", "box_scan", "defect_scan", "pick_probe"].includes(item.id));
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
        + `${state.probe_attached === true ? " · 탐침 파지" : state.probe_attached === false ? " · 탐침 미파지" : " · 탐침 상태 확인 중"}`
        + `${state.task ? ` · ${state.task}` : ""}`;
    if (robotControl.live && state.scan) this.status.textContent += scanText(state.scan);
    if (state?.event === "robot_stopped") this.error.textContent = `로봇 정지 명령 전송 (${state.stop_sent}건)`;
    if (state?.log !== this.log.textContent) this.log.textContent = state?.log || "";
  }
}
