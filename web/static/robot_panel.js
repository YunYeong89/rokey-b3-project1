import { RobotViewer } from "./robot_viewer.js";
import { RobotRosClient } from "./robot_ros.js";
import { RobotTaskPanel, robotControl, rosbridgeUrl } from "./robot_control.js";
import { RobotResults } from "./robot_results.js";
import { CloudFeed } from "./robot_clouds.js";

export class RobotPanel {
  constructor(panel) {
    this.panel = panel;
    this.tasks = new RobotTaskPanel(panel);
    this.status = panel.querySelector("[data-robot-status]");
    this.message = panel.querySelector("[data-robot-message]");
    this.viewer = new RobotViewer(panel.querySelector("[data-robot-viewer]"));
    this.results = new RobotResults(panel.querySelector("[data-robot-results]"));
    this.clouds = new CloudFeed(this.viewer);
    panel.querySelectorAll("[data-robot-cloud]").forEach((input) =>
      input.addEventListener("change", () => this.viewer.setCloudVisible(input.dataset.robotCloud, input.checked)));
    this.url = panel.querySelector("[data-robot-url]");
    this.topic = panel.querySelector("[data-robot-topic]");
    this.url.value = rosbridgeUrl();
    robotControl.addEventListener("state", () => this.followMode());
    this.topic.value = window.APP.robotJointTopic || "/dsr01/sim_joint_states";
    panel.querySelector("[data-robot-connect]").addEventListener("click", () => this.connect());
    panel.querySelectorAll("[data-robot-view]").forEach((button) =>
      button.addEventListener("click", () => this.viewer.view(button.dataset.robotView)));
    panel.querySelector("[data-robot-fit]").addEventListener("click", () => this.viewer.fit());
    panel.querySelectorAll("[data-robot-display]").forEach((input) =>
      input.addEventListener("change", () => this.viewer.setDisplay({ [input.dataset.robotDisplay]: input.checked })));
    panel.querySelector("[data-robot-frame]").addEventListener("change", (event) => this.viewer.setTcpFrame(event.target.value));
    document.querySelector('[data-tab="robot"]').addEventListener("click", () => {
      if (!this.started) {
        this.started = true;
        this.load(); this.connect(); this.results.start(); this.clouds.start();
      }
      requestAnimationFrame(() => this.viewer.resize());
    });
    setInterval(() => this.refresh(), 200);
    window.addEventListener("beforeunload", () => this.client?.close());
  }

  async load() {
    try {
      const robot = await this.viewer.load((loaded, total) => {
        this.message.textContent = `URDF / 메시 로딩 ${loaded} / ${total}`;
      });
      const frame = this.panel.querySelector("[data-robot-frame]");
      frame.replaceChildren(...Object.keys(robot.links).map((name) => new Option(name, name)));
      frame.value = this.viewer.tcpFrame;
      const names = Object.keys(robot.joints).filter((name) => robot.joints[name].jointType !== "fixed");
      const rows = this.panel.querySelector("[data-robot-joints]");
      rows.replaceChildren(...names.map((name) => {
        const row = document.createElement("tr");
        const label = document.createElement("td"), value = document.createElement("td");
        label.textContent = name; value.textContent = "—"; value.dataset.joint = name;
        row.append(label, value); return row;
      }));
      this.message.textContent = "";
      this.message.hidden = true;
      this.panel.dataset.modelLoaded = "true";
    } catch (error) {
      this.message.textContent = `로봇 모델 로딩 실패: ${error.message}`;
      this.message.classList.add("error");
      console.error(error);
    }
  }

  /** Joint topic of the connected robot, unless the server sets one: simulation feedback or the real controller. */
  followMode() {
    const mode = robotControl.status?.mode;
    if (window.APP.robotJointTopic || !mode || !this.started) return;
    const topic = mode === "real" ? "/dsr01/joint_states" : "/dsr01/sim_joint_states";
    if (topic === this.modeTopic) return;
    this.modeTopic = topic;
    if (this.topic.value !== topic) { this.topic.value = topic; this.connect(); }
  }

  connect() {
    this.client?.close();
    this.client = new RobotRosClient({ url: this.url.value, topic: this.topic.value,
      sceneTopic: window.APP.robotSceneTopic || "/dsr01/sim_scene",
      onScene: (message) => this.viewer.applyScene(message),
      onState: (state) => { this.connection = state; this.refresh(); },
      onMessage: (message) => { this.viewer.applyJointState(message); this.refresh(); } });
    this.client.connect();
  }

  refresh() {
    const age = this.client?.lastReceived ? (performance.now() - this.client.lastReceived)/1000 : Infinity;
    const receiving = this.connection !== "disconnected" && age < 1;
    const states = { connecting: "연결 중", waiting: "연결됨 · 관절값 대기", error: "연결 오류", disconnected: "연결 끊김" };
    this.status.textContent = receiving ? "관절값 수신 중" : (this.connection === "waiting" && Number.isFinite(age)
      ? "관절 데이터 수신 중단" : states[this.connection] || "연결 대기");
    this.status.className = `pill ${receiving ? "on" : "off"}`;
    this.panel.querySelector("[data-robot-rate]").textContent = receiving ? `${this.client.rate.toFixed(1)} Hz` : "—";
    this.panel.querySelector("[data-robot-age]").textContent = Number.isFinite(age) ? `${age.toFixed(1)} s` : "—";
    if (!this.viewer.tcpFrame) return;
    for (const cell of this.panel.querySelectorAll("[data-joint]")) {
      const value = this.viewer.robot.joints[cell.dataset.joint].angle;
      cell.textContent = `${(value * 180 / Math.PI).toFixed(2)}°`;
    }
    const pose = this.viewer.pose();
    this.panel.querySelector("[data-robot-position]").textContent = pose.position.map((x) => x.toFixed(4)).join("  ");
    this.panel.querySelector("[data-robot-orientation]").textContent = pose.quaternion.map((x) => x.toFixed(4)).join("  ");
  }
}

const panel = new RobotPanel(document.querySelector("#tab-robot"));
export default panel;
window.addEventListener("DOMContentLoaded", () => {
  if (new URLSearchParams(location.search).get("tab") === "robot") document.querySelector('[data-tab="robot"]').click();
});
