import { RobotViewer } from "./robot_viewer.js";
import { RobotRosClient } from "./robot_ros.js";
import { RobotTaskPanel, robotControl, rosbridgeUrl } from "./robot_control.js";
import { RobotTeleop } from "./robot_teleop.js";
import { RobotResults } from "./robot_results.js";
import { CloudFeed } from "./robot_clouds.js";
import { RobotInspection } from "./robot_inspection.js";

export class RobotPanel {
  constructor(panel) {
    this.panel = panel;
    this.tasks = new RobotTaskPanel(panel);
    this.inspection = new RobotInspection(panel.querySelector("[data-inspection]"));
    this.teleop = new RobotTeleop(panel);
    this.status = panel.querySelector("[data-robot-status]");
    this.message = panel.querySelector("[data-robot-message]");
    this.viewer = new RobotViewer(panel.querySelector("[data-robot-viewer]"));
    this.results = new RobotResults(panel.querySelector("[data-robot-results]"));
    this.clouds = new CloudFeed(this.viewer);
    panel.querySelector("[data-robot-objects]").addEventListener("change", (event) =>
      this.viewer.setObjectsVisible(event.target.checked));
    panel.querySelectorAll("[data-robot-cloud]").forEach((input) =>
      input.addEventListener("change", () => this.viewer.setCloudVisible(input.dataset.robotCloud, input.checked)));
    this.rosStatus = panel.querySelector("[data-ros-connection]");
    robotControl.addEventListener("state", () => this.followMode());
    panel.querySelector("[data-robot-connect]").addEventListener("click", () => {
      robotControl.reconnect();
      this.connect();
    });
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
      this.inspection.load();
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

  followMode() {
    if (this.started && this.client?.topic !== robotControl.jointTopic()) this.connect();
    this.refresh();
  }

  connect() {
    this.client?.close();
    this.client = new RobotRosClient({ url: rosbridgeUrl(), topic: robotControl.jointTopic(),
      sceneTopic: (robotControl.selected === "dsr01" && window.APP.robotSceneTopic) || `/${robotControl.selected}/sim_scene`,
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
    this.rosStatus.textContent = robotControl.live ? "연결됨" : robotControl.connected ? "로봇 응답 대기" : "연결 대기";
    this.rosStatus.className = `pill ${robotControl.live ? "on" : "off"}`;
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
