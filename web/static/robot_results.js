import { robotControl } from "./robot_control.js";

/** Report files of the robot programs by run (/api/robot/runs: plan, surface, force and ROI graphs, CSV / JSON / PLY)
 * and the live force graph (draw_graph started beside any program, images/force_live.png redrawn every second). */
export class RobotResults {
  constructor(card) {
    this.select = card.querySelector("[data-run-select]");
    this.images = card.querySelector("[data-run-images]");
    this.files = card.querySelector("[data-run-files]");
    this.graphButton = card.querySelector("[data-graph-toggle]");
    this.compensated = card.querySelector("[data-graph-compensated]");
    this.live = card.querySelector("[data-graph-live]");
    this.graphError = card.querySelector("[data-graph-error]");
    this.select.addEventListener("change", () => {
      this.pinned = this.select.value !== this.runs?.[0]?.run;  // the newest run is followed unless another is chosen
      this.render();
    });
    this.graphButton.addEventListener("click", () => {
      this.graphError.textContent = "";
      try { robotControl.graph(!robotControl.status?.graph, this.compensated.checked); }
      catch (error) { this.graphError.textContent = error.message; }
    });
    robotControl.addEventListener("state", () => this.refreshGraph());
  }

  start() {
    if (this.timer) return;
    this.load();
    this.timer = setInterval(() => this.load(), 3000);
  }

  async load() {
    const response = await fetch("/api/robot/runs");
    if (!response.ok) return;
    const runs = await response.json();
    const key = runs.map((run) => `${run.run}:${run.mtime}`).join();
    if (key === this.key) return;
    Object.assign(this, { key, runs });
    const selected = this.pinned ? this.select.value : runs[0]?.run;
    this.select.replaceChildren(...runs.map((run) => new Option(`${run.run} · 파일 ${run.files.length}개`, run.run)));
    if (selected) this.select.value = selected;
    this.render();
  }

  render() {
    const run = this.runs?.find((item) => item.run === this.select.value);
    const files = run?.files || [];
    const url = (file) => `/api/robot/files/${file.path}?v=${file.mtime}`;
    const label = (file) => file.name.replace(run.run, "").replace(/^[_/]+/, "") || file.name;
    this.images.replaceChildren(...files.filter((file) => file.kind === "image").map((file) => {
      const link = Object.assign(document.createElement("a"), { href: url(file), target: "_blank", title: file.name });
      const image = Object.assign(document.createElement("img"), { src: url(file), alt: file.name, loading: "lazy" });
      const caption = Object.assign(document.createElement("span"), { textContent: label(file) });
      link.append(image, caption);
      return link;
    }));
    this.files.replaceChildren(...files.filter((file) => file.kind !== "image").map((file) =>
      Object.assign(document.createElement("a"), { href: url(file), download: file.name.split("/").pop(),
        textContent: `${label(file)} (${Math.max(1, Math.round(file.size / 1024))} KB)` })));
    if (!files.length) this.images.textContent = "아직 결과 파일이 없습니다. 기능을 실행하면 여기에 그래프가 표시됩니다.";
  }

  refreshGraph() {
    const status = robotControl.status, on = !!status?.graph;
    this.graphButton.textContent = on ? "힘 그래프 종료 (PNG·CSV 저장)" : "실시간 힘 그래프 시작";
    this.graphButton.disabled = !robotControl.live;
    this.compensated.disabled = on;
    this.live.hidden = !on;
    if (on && performance.now() - (this.drawn || 0) > 1000) {
      this.drawn = performance.now();
      this.live.src = `/api/robot/files/images/force_live.png?t=${Date.now()}`;
    }
    if (!on && status?.graph_log) this.graphError.textContent = status.graph_log;
  }
}
