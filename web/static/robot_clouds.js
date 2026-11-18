/** The running program's clouds (session/clouds/<name>.npy via /api/robot/clouds) in the robot viewer: reference
 * (the part, then its measurable points after planning) and measured (contact points, rewritten every second). */
export class CloudFeed {
  constructor(viewer, names = ["reference", "measured"]) {
    Object.assign(this, { viewer, names, tags: {} });
  }

  start() {
    if (this.timer) return;
    this.poll();
    this.timer = setInterval(() => this.poll(), 1500);
  }

  async poll() {
    for (const name of this.names) {
      const response = await fetch(`/api/robot/clouds/${name}`, { cache: "no-cache" });
      const tag = response.headers.get("ETag");
      if (!response.ok || tag === this.tags[name]) continue;  // none yet (404) or unchanged
      this.tags[name] = tag;
      this.viewer.setCloud(name, CloudFeed.parse(await response.arrayBuffer()));
    }
  }

  /** NumPy .npy (format 1 or 2, little-endian float32, shape (n, 3)) -> Float32Array x, y, z. */
  static parse(buffer) {
    const view = new DataView(buffer), major = view.getUint8(6);
    const header = major === 1 ? 10 + view.getUint16(8, true) : 12 + view.getUint32(8, true);
    return new Float32Array(buffer.slice(header));
  }
}
