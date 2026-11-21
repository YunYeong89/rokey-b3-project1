import { robotControl } from "./robot_control.js";

/** Web teleop: the Teleop button starts `rokey teleop --web` (web_control task "teleop") and opens a control popup.
 *  Height on the left, the BASE x-y arrows on the right, speed, gripper, stop below. The robot moves only while a
 *  button is held: the held key repeats on /robot/teleop every REPEAT_MS (teleop jogs on while it keeps arriving), and
 *  the release (pointer up, cancel or lost, window blur, hidden tab, teleop ended) sends "" at once (move_stop). */
const REPEAT_MS = 80;
const PAD = [  // [key, label, grid area]: BASE as seen from the robot (front: toward the vehicle)
  ["w", "▲<small>앞 +X</small>", "up"], ["a", "◀<small>왼쪽 +Y</small>", "left"],
  ["d", "▶<small>오른쪽 −Y</small>", "right"], ["s", "▼<small>뒤 −X</small>", "down"],
];
const HEIGHT = [["r", "▲<small>위 +Z</small>"], ["f", "▼<small>아래 −Z</small>"]];
const SPEEDS = [["1", 2], ["2", 5], ["3", 15], ["4", 30], ["5", 50]];  // mm/s (teleop SPEEDS)

export class RobotTeleop {
  constructor(panel) {
    this.toggle = Object.assign(document.createElement("button"), { className: "btn small", textContent: "Teleop" });
    panel.querySelector(".robot-task-buttons").append(this.toggle);
    this.toggle.addEventListener("click", () => (this.pop.hidden ? this.open() : this.close()));
    this.pop = this.build();
    document.body.append(this.pop);
    robotControl.addEventListener("state", () => this.refresh());
    window.addEventListener("blur", () => this.release());
    document.addEventListener("visibilitychange", () => { if (document.hidden) this.release(); });
  }

  build() {
    const pop = document.createElement("section");
    pop.className = "teleop-pop";
    pop.hidden = true;
    pop.innerHTML = `
      <header class="teleop-head"><b>Teleop</b><span class="note" data-teleop-status></span>
        <button class="teleop-close" data-teleop-close aria-label="닫기">×</button></header>
      <div class="teleop-body">
        <div class="teleop-height" data-teleop-height></div>
        <div class="teleop-pad" data-teleop-pad><span class="teleop-centre">BASE</span></div>
      </div>
      <div class="teleop-row"><span class="note">속도</span><span data-teleop-steps></span></div>
      <div class="teleop-row"><span class="note">그리퍼</span><span data-teleop-grip></span></div>
      <div class="teleop-row">
        <button class="btn danger small" data-teleop-stop>로봇 정지</button>
        <button class="btn small" data-teleop-off>Teleop 끄기</button></div>
      <p class="form-msg err" data-teleop-error role="alert"></p>
      <p class="note">버튼을 누르고 있는 동안만 움직이고, 떼면 바로 멈춥니다.</p>`;
    const key = (name, html, area) => {
      const button = Object.assign(document.createElement("button"), { className: "teleop-key", innerHTML: html });
      if (area) button.style.gridArea = area;
      this.hold(button, name);
      return button;
    };
    pop.querySelector("[data-teleop-pad]").append(...PAD.map(([name, html, area]) => key(name, html, area)));
    pop.querySelector("[data-teleop-height]").append(...HEIGHT.map(([name, html]) => key(name, html)));
    pop.querySelector("[data-teleop-grip]").append(key("o", "열기"), key("c", "닫기"));
    this.steps = SPEEDS.map(([name, speed]) => {
      const button = Object.assign(document.createElement("button"), { className: "teleop-step", textContent: `${speed} mm/s` });
      button.dataset.step = name;
      button.addEventListener("click", () => this.send(name) && this.mark(name));
      return button;
    });
    pop.querySelector("[data-teleop-steps]").append(...this.steps);
    this.mark("2");  // teleop starts at 5 mm/s
    this.status = pop.querySelector("[data-teleop-status]");
    this.error = pop.querySelector("[data-teleop-error]");
    pop.querySelector("[data-teleop-close]").addEventListener("click", () => this.close());
    pop.querySelector("[data-teleop-off]").addEventListener("click", () => this.close());
    pop.querySelector("[data-teleop-stop]").addEventListener("click", () => {
      this.release();
      try { robotControl.stop(); } catch (error) { this.error.textContent = error.message; }
    });
    return pop;
  }

  /** Moves only while held: the key at once and every REPEAT_MS, "" on any way the press can end. */
  hold(button, name) {
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      if (!this.active || event.button > 0) return;
      this.release();
      button.setPointerCapture(event.pointerId);
      if (!this.send(name)) return;
      this.held = button;
      button.classList.add("held");
      this.timer = setInterval(() => this.send(name) || this.release(), REPEAT_MS);
    });
    for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
      button.addEventListener(type, () => { if (this.held === button) this.release(); });
    }
    button.addEventListener("contextmenu", (event) => event.preventDefault());
  }

  release() {
    if (!this.held) return;
    clearInterval(this.timer);
    this.held.classList.remove("held");
    this.held = this.timer = null;
    this.send("");
  }

  send(key) {
    try {
      robotControl.teleop(key);
      return true;
    } catch (error) {
      this.error.textContent = error.message;
      return false;
    }
  }

  mark(name) {
    this.steps.forEach((button) => button.classList.toggle("selected", button.dataset.step === name));
  }

  get active() {
    const state = robotControl.status;
    return robotControl.live && state?.task === "teleop" && state.state === "running";
  }

  open() {
    this.pop.hidden = false;
    this.error.textContent = "";
    if (!this.active) {
      const state = robotControl.status;
      if (state?.busy) this.error.textContent = "다른 작업이 실행 중입니다. 끝난 뒤 다시 켜세요.";
      else {
        try {
          this.request = `${Date.now()}-${Math.random().toString(16).slice(2)}`;
          robotControl.start("teleop", {}, this.request);
        } catch (error) { this.error.textContent = error.message; }
      }
    }
    this.refresh();
  }

  close() {
    this.release();
    if (this.active) {
      try { robotControl.stop(); } catch (error) { this.error.textContent = error.message; }
    }
    this.pop.hidden = true;
    this.refresh();
  }

  refresh() {
    const state = robotControl.status, active = this.active;
    if (!active) this.release();
    if (active && !this.wasActive) this.mark("2");  // a new teleop starts at 5 mm/s
    this.wasActive = active;
    if (state?.event === "rejected" && state.rejected_request === this.request) {
      this.error.textContent = state.error;
      this.request = null;
    }
    this.toggle.classList.toggle("primary", active);
    this.toggle.textContent = active ? "Teleop 켜짐" : "Teleop";
    this.pop.querySelectorAll(".teleop-key, .teleop-step").forEach((button) => { button.disabled = !active; });
    this.status.textContent = active ? "조작 가능" : !robotControl.live ? "ROS 제어 노드 응답 대기"
      : state?.task === "teleop" ? state.message : "Teleop 시작 대기";
  }
}
