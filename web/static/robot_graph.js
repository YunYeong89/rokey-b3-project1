import { robotControl } from "./robot_control.js";

/** Live force graph drawn in the browser: the four panels of draw_graph's figure (BASE external force, force magnitude
 * and contact force with the touch threshold, TCP position, TCP path coloured by |F|) from its stream
 * /robot/force_graph (std_msgs/String JSON batches, rokey draw_graph.py GraphStream). Every animation frame the time
 * axis scrolls with the clock and the y ranges ease toward the data, so 20 Hz samples read as continuous motion. */
const WINDOW_S = 30;   // shown span, as draw_graph --window
const DELAY_S = 0.25;  // drawn this far behind the newest sample, so batch arrival jitter never shows
const EASE_S = 0.25;   // time constant of the axis-range easing
const HEIGHT = 210;    // plot height [CSS px]
// x / y / z keep the 3D view's axis colours; red-green sits in the colour-vision warn band (validator), so every line
// also carries a direct label at its end. |F| / Fn: palette slots 1-2. Path: one-hue blue ramp (steps 250-700).
const XYZ = ["#e34948", "#008300", "#2a78d6"];
const MAGNITUDE = "#2a78d6", NORMAL = "#eb6834";
const RAMP = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"].map((hex) =>
  [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16)));

const css = (name, fallback) => getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
const element = (tag, className, text) => Object.assign(document.createElement(tag), { className, textContent: text ?? "" });

function ticks(lo, hi, count) {
  const raw = (hi - lo || 1) / Math.max(count, 1), power = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 5, 10].map((m) => m * power).find((s) => s >= raw), values = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-6; v += step) values.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  return { values, digits: Math.max(0, -Math.floor(Math.log10(step) + 1e-9)) };
}

function lower(array, value) {  // first index with array[i] >= value
  let lo = 0, hi = array.length;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (array[mid] < value) lo = mid + 1; else hi = mid; }
  return lo;
}

function ramp(u) {
  const x = Math.min(Math.max(u, 0), 1) * (RAMP.length - 1), i = Math.min(Math.floor(x), RAMP.length - 2), f = x - i;
  return `rgb(${RAMP[i].map((c, k) => Math.round(c + (RAMP[i + 1][k] - c) * f)).join(",")})`;
}

/** Samples of the current draw_graph run (the window and a little more), and the clock that maps them to now. */
class Store {
  constructor() { this.clear(); }

  clear() {
    Object.assign(this, { t: [], force: [[], [], []], magnitude: [], fn: [], calibrated: null, fnCalibrated: [],
                          tcp: [[], [], []], offset: null, arrived: 0, meta: {} });
  }

  add(batch) {
    if (this.t.length && batch.t[0] < this.t[this.t.length - 1] - 1) this.clear();  // a new run starts at 0
    const { t, force, fn, position } = batch;
    if (batch.force_calibrated && !this.calibrated) this.calibrated = [[], [], []];
    t.forEach((time, i) => {
      this.t.push(time);
      force[i].forEach((v, k) => this.force[k].push(v));
      this.magnitude.push(Math.hypot(...force[i]));
      this.fn.push(fn[i]);
      position[i].forEach((v, k) => this.tcp[k].push(v));
      if (this.calibrated) {
        batch.force_calibrated[i].forEach((v, k) => this.calibrated[k].push(v));
        this.fnCalibrated.push(batch.fn_calibrated[i]);
      }
    });
    this.meta = batch;
    const wall = performance.now() / 1000, sample = t[t.length - 1] - wall;
    this.offset = this.offset === null || Math.abs(sample - this.offset) > 1 ? sample : this.offset + .1 * (sample - this.offset);
    this.arrived = wall;
    const cut = lower(this.t, this.t[this.t.length - 1] - WINDOW_S - 5);
    if (cut > 2000) for (const series of [this.t, ...this.force, this.magnitude, this.fn, ...this.tcp,
                                          ...(this.calibrated || []), this.fnCalibrated]) series.splice(0, cut);
  }

  /** Shown time: the sample clock, scrolling with wall time; held at the last sample once the stream stops. */
  now() {
    const wall = performance.now() / 1000, last = this.t[this.t.length - 1];
    const now = wall + this.offset - DELAY_S;
    return wall - this.arrived > 1 ? Math.min(now, last) : now;
  }
}

class Panel {
  constructor(parent, title, unit) {
    const root = element("div", "force-panel"), head = element("div", "force-head");
    head.append(element("b", "", title), element("span", "force-unit", unit));
    this.legend = element("div", "force-legend");
    this.box = element("div", "force-plot");
    this.canvas = document.createElement("canvas");
    this.tip = element("div", "force-tip");
    this.tip.hidden = true;
    this.box.append(this.canvas, this.tip);
    root.append(head, this.legend, this.box);
    parent.append(root);
    this.canvas.addEventListener("pointermove", (event) => { this.hover = { x: event.offsetX, y: event.offsetY }; });
    this.canvas.addEventListener("pointerleave", () => { this.hover = null; this.tip.hidden = true; });
  }

  /** Canvas at the box width and HEIGHT, in device pixels; the context draws in CSS pixels. */
  context() {
    const w = this.box.clientWidth, ratio = devicePixelRatio || 1;
    if (this.canvas.width !== Math.round(w * ratio) || this.canvas.height !== Math.round(HEIGHT * ratio)) {
      Object.assign(this.canvas, { width: Math.round(w * ratio), height: Math.round(HEIGHT * ratio) });
      this.canvas.style.height = `${HEIGHT}px`;
    }
    const ctx = this.canvas.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, w, HEIGHT);
    ctx.font = "11px system-ui, -apple-system, sans-serif";
    return { ctx, w };
  }

  setLegend(items) {
    const key = items.map((item) => item.label + item.dashed).join();
    if (key === this.legendKey) return;
    this.legendKey = key;
    this.legend.replaceChildren(...items.map(({ label, color, dashed }) => {
      const item = element("span", "force-key");
      const swatch = element("i", dashed ? "dashed" : "");
      swatch.style.borderColor = color;
      item.append(swatch, label);
      return item;
    }));
  }

  grid(ctx, theme, area, x, y) {
    const { L, T, pw, ph } = area;
    ctx.lineWidth = 1;
    ctx.strokeStyle = theme.grid;
    ctx.fillStyle = theme.muted;
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    for (const v of y.ticks.values) {
      const py = Math.round(y.at(v)) + .5;
      if (py < T - .5 || py > T + ph + .5) continue;
      ctx.beginPath(); ctx.moveTo(L, py); ctx.lineTo(L + pw, py); ctx.stroke();
      ctx.fillText(v.toFixed(y.ticks.digits), L - 6, py);
    }
    ctx.textAlign = "center"; ctx.textBaseline = "top";
    for (const v of x.ticks.values) {
      const px = Math.round(x.at(v)) + .5;
      if (px < L - .5 || px > L + pw + .5) continue;
      ctx.beginPath(); ctx.moveTo(px, T); ctx.lineTo(px, T + ph); ctx.stroke();
      ctx.fillText(v.toFixed(x.ticks.digits) + (x.suffix || ""), px, T + ph + 5);
    }
    ctx.strokeStyle = theme.axis;
    ctx.strokeRect(L + .5, T + .5, pw - 1, ph - 1);
  }

  showTip(rows, x, y) {
    this.tip.replaceChildren(...rows.map(([label, value, color]) => {
      const row = element("div", "");
      if (color) { const dot = element("i", ""); dot.style.background = color; row.append(dot); }
      row.append(element("span", "", label), element("b", "", value));
      return row;
    }));
    this.tip.hidden = false;
    const left = x + 14 + this.tip.offsetWidth > this.box.clientWidth ? x - 14 - this.tip.offsetWidth : x + 14;
    this.tip.style.transform = `translate(${Math.max(0, left)}px, ${Math.max(0, y - 10)}px)`;
  }

  ease(name, target, k) {
    this[name] = this[name] ? this[name].map((v, i) => v + (target[i] - v) * k) : target;
    return this[name];
  }
}

/** Time series panel: lines over the last WINDOW_S seconds, a reference line, direct end labels, hover readout. */
class LinePanel extends Panel {
  constructor(parent, title, unit, series, reference = () => null) {
    super(parent, title, unit);
    Object.assign(this, { series, reference });
  }

  draw(store, now, k, theme) {
    const { ctx, w } = this.context(), area = { L: 48, T: 8, pw: w - 48 - 40, ph: HEIGHT - 8 - 22 };
    const { L, T, pw, ph } = area, t = store.t, from = now - WINDOW_S;
    const i0 = Math.max(0, lower(t, from) - 1), i1 = Math.min(t.length, lower(t, now) + 1);
    const series = this.series(store), reference = this.reference(store);
    this.setLegend(series);
    let lo = Infinity, hi = -Infinity;
    for (const { values } of series) for (let i = i0; i < i1; i++) { lo = Math.min(lo, values[i]); hi = Math.max(hi, values[i]); }
    if (reference) { lo = Math.min(lo, reference.value); hi = Math.max(hi, reference.value); }
    if (!(hi >= lo)) return;
    const pad = Math.max((hi - lo) * .1, .2);
    const [ylo, yhi] = this.ease("range", [lo - pad, hi + pad], k);
    const x = { at: (v) => L + (v - from) / WINDOW_S * pw, ticks: ticks(from, now, Math.max(3, Math.round(pw / 90))), suffix: " s" };
    const y = { at: (v) => T + (yhi - v) / (yhi - ylo) * ph, ticks: ticks(ylo, yhi, Math.max(3, Math.round(ph / 42))) };
    this.grid(ctx, theme, area, x, y);
    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    if (reference) {
      const py = y.at(reference.value);
      ctx.strokeStyle = theme.muted; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(L, py); ctx.lineTo(L + pw, py); ctx.stroke();
      ctx.setLineDash([]); ctx.fillStyle = theme.ink; ctx.textAlign = "left"; ctx.textBaseline = "bottom";
      ctx.fillText(reference.label, L + 6, py - 3);
    }
    ctx.lineWidth = 2; ctx.lineJoin = "round"; ctx.lineCap = "round";
    for (const { values, color, dashed } of series) {
      ctx.strokeStyle = color; ctx.setLineDash(dashed ? [6, 4] : []);
      ctx.beginPath();
      for (let i = i0; i < i1; i++) (i === i0 ? ctx.moveTo : ctx.lineTo).call(ctx, x.at(t[i]), y.at(values[i]));
      ctx.stroke();
    }
    ctx.restore();
    ctx.setLineDash([]);
    const ends = series.filter((s) => !s.dashed && i1 > i0).map((s) => ({ s, py: y.at(s.values[i1 - 1]) }))
      .sort((a, b) => a.py - b.py);
    ends.forEach((end, j) => { if (j) end.py = Math.max(end.py, ends[j - 1].py + 12); });
    ctx.textAlign = "left"; ctx.textBaseline = "middle";
    for (const { s, py } of ends) {
      const yy = Math.min(Math.max(py, T + 5), T + ph + 8);
      ctx.fillStyle = s.color; ctx.fillRect(L + pw + 3, yy - 1, 7, 2);
      ctx.fillStyle = theme.ink; ctx.fillText(s.short || s.label, L + pw + 13, yy);
    }
    if (!this.hover || this.hover.x < L || this.hover.x > L + pw || i1 <= i0) { this.tip.hidden = true; return; }
    const at = from + (this.hover.x - L) / pw * WINDOW_S;
    let i = Math.min(Math.max(lower(t, at), i0), i1 - 1);
    if (i > i0 && at - t[i - 1] < t[i] - at) i -= 1;
    const px = x.at(t[i]);
    ctx.strokeStyle = theme.ink; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(px + .5, T); ctx.lineTo(px + .5, T + ph); ctx.stroke();
    for (const { values, color } of series) {
      ctx.fillStyle = color; ctx.beginPath(); ctx.arc(px, y.at(values[i]), 4, 0, 2 * Math.PI); ctx.fill();
    }
    this.showTip([[`t ${t[i].toFixed(2)} s`, ""], ...series.map((s) => [s.label, s.values[i].toFixed(2), s.color])],
                 px, this.hover.y);
  }
}

/** TCP path in the x-y plane over the last WINDOW_S seconds, equal axes, coloured by |F| with its colour bar. */
class PathPanel extends Panel {
  draw(store, now, k, theme) {
    const { ctx, w } = this.context(), area = { L: 52, T: 8, pw: w - 52 - 62, ph: HEIGHT - 8 - 22 };
    const { L, T, pw, ph } = area, t = store.t, [xs, ys] = store.tcp, force = store.magnitude;
    const i0 = lower(t, now - WINDOW_S), i1 = Math.min(t.length, lower(t, now) + 1);
    if (i1 <= i0) return;
    let xlo = Infinity, xhi = -Infinity, ylo = Infinity, yhi = -Infinity, flo = Infinity, fhi = -Infinity;
    for (let i = i0; i < i1; i++) {
      xlo = Math.min(xlo, xs[i]); xhi = Math.max(xhi, xs[i]); ylo = Math.min(ylo, ys[i]); yhi = Math.max(yhi, ys[i]);
      flo = Math.min(flo, force[i]); fhi = Math.max(fhi, force[i]);
    }
    const scale = Math.max((xhi - xlo) / pw, (yhi - ylo) / ph, .05) * 1.15;  // mm per px, the same on both axes
    const [cx, cy, s] = this.ease("view", [(xlo + xhi) / 2, (ylo + yhi) / 2, scale], k);
    const [fmin, fmax] = this.ease("forces", [flo, Math.max(fhi, flo + .5)], k);
    const x = { at: (v) => L + pw / 2 + (v - cx) / s, ticks: ticks(cx - s * pw / 2, cx + s * pw / 2, Math.max(3, Math.round(pw / 90))) };
    const y = { at: (v) => T + ph / 2 - (v - cy) / s, ticks: ticks(cy - s * ph / 2, cy + s * ph / 2, Math.max(3, Math.round(ph / 42))) };
    this.grid(ctx, theme, area, x, y);
    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    for (let i = i0; i < i1; i++) {
      ctx.fillStyle = ramp((force[i] - fmin) / (fmax - fmin));
      ctx.beginPath(); ctx.arc(x.at(xs[i]), y.at(ys[i]), 2.5, 0, 2 * Math.PI); ctx.fill();
    }
    const last = i1 - 1;
    ctx.strokeStyle = theme.ink; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(x.at(xs[last]), y.at(ys[last]), 6, 0, 2 * Math.PI); ctx.stroke();
    ctx.restore();
    const bx = L + pw + 12, gradient = ctx.createLinearGradient(0, T + ph, 0, T);
    RAMP.forEach((_, j) => gradient.addColorStop(j / (RAMP.length - 1), ramp(j / (RAMP.length - 1))));
    ctx.fillStyle = gradient; ctx.fillRect(bx, T, 10, ph);
    ctx.fillStyle = theme.muted; ctx.textAlign = "left"; ctx.textBaseline = "middle";
    const bar = ticks(fmin, fmax, 4);
    for (const v of bar.values) ctx.fillText(v.toFixed(bar.digits), bx + 14, T + ph - (v - fmin) / (fmax - fmin) * ph);
    if (!this.hover) { this.tip.hidden = true; return; }
    let best = -1, distance = 144;
    for (let i = i0; i < i1; i++) {
      const d = (x.at(xs[i]) - this.hover.x) ** 2 + (y.at(ys[i]) - this.hover.y) ** 2;
      if (d < distance) { distance = d; best = i; }
    }
    if (best < 0) { this.tip.hidden = true; return; }
    ctx.strokeStyle = theme.ink; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(x.at(xs[best]), y.at(ys[best]), 5, 0, 2 * Math.PI); ctx.stroke();
    this.showTip([[`t ${t[best].toFixed(2)} s`, ""], ["x", `${xs[best].toFixed(1)} mm`], ["y", `${ys[best].toFixed(1)} mm`],
                  ["|F|", `${force[best].toFixed(2)} N`, ramp((force[best] - fmin) / (fmax - fmin))]],
                 x.at(xs[best]), y.at(ys[best]));
  }
}

export class ForceChart {
  constructor(root) {
    this.root = root;
    this.store = new Store();
    this.status = element("p", "note force-status", "힘 그래프 수신 대기");
    const grid = element("div", "force-grid");
    root.append(this.status, grid);
    const xyz = (values, names, dashed = false) => values.map((v, k) =>
      ({ label: names[k], color: XYZ[k], values: v, dashed }));
    this.panels = [
      new LinePanel(grid, "BASE 외력", "F [N]", (s) => [...xyz(s.force, ["Fx", "Fy", "Fz"]),
        ...(s.calibrated ? xyz(s.calibrated, ["Fx 보정", "Fy 보정", "Fz 보정"], true) : [])]),
      new LinePanel(grid, "외력 크기 · 접촉력", "F [N]", (s) => [
        { label: "|F|", color: MAGNITUDE, values: s.magnitude },
        { label: "Fn (TOOL −Z)", short: "Fn", color: NORMAL, values: s.fn },
        ...(s.calibrated ? [{ label: "Fn 보정", color: NORMAL, values: s.fnCalibrated, dashed: true }] : [])],
        (s) => s.meta.threshold == null ? null
          : { value: s.meta.threshold, label: `접촉 임계 ${Number(s.meta.threshold).toFixed(2)} N` }),
      new LinePanel(grid, "TCP 위치 (BASE)", "mm", (s) => xyz(s.tcp, ["x", "y", "z"])),
      new PathPanel(grid, "TCP 경로 (|F| 색)", "x, y [mm] · |F| [N]"),
    ];
    robotControl.addEventListener("connection", (event) => this.subscribe(event.detail));
    robotControl.addEventListener("robot", () => {
      this.store = new Store();
      if (robotControl.connected) this.subscribe(robotControl.ros);
    });
    if (robotControl.connected) this.subscribe(robotControl.ros);
    this.previous = performance.now();
    requestAnimationFrame((ms) => this.frame(ms));
  }

  get fresh() { return this.store.t.length > 0 && performance.now() / 1000 - this.store.arrived < 3; }

  subscribe(ros) {
    this.topic?.unsubscribe();
    this.topic = new ROSLIB.Topic({ ros, name: robotControl.topic("/robot/force_graph"),
      messageType: "std_msgs/msg/String", queue_length: 10 });
    this.topic.subscribe((message) => this.store.add(JSON.parse(message.data)));
  }

  frame(ms) {
    requestAnimationFrame((next) => this.frame(next));
    const k = 1 - Math.exp(-(ms - this.previous) / 1000 / EASE_S);
    this.previous = ms;
    if (!this.store.t.length || this.root.hidden || !this.root.offsetParent) return;
    const theme = { ink: css("--ink", "#1c2530"), muted: css("--muted", "#6b7684"), grid: "#eef1f4", axis: css("--line", "#dde2e8") };
    const now = this.store.now(), meta = this.store.meta;
    const count = this.store.t.length - lower(this.store.t, now - WINDOW_S);
    this.status.textContent = `외력 (${meta.mode}, tool=${meta.tool}, tcp=${meta.tcp}, 샘플 ${count}개, 최근 ${WINDOW_S} s)`
      + (this.fresh ? " · 수신 중" : " · 수신 멈춤");
    for (const panel of this.panels) panel.draw(this.store, now, k, theme);
  }
}
