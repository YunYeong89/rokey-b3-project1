/** Read-only rosbridge subscriptions: JointState, plus the simulation scene (MarkerArray) if given. */
export class RobotRosClient {
  constructor({ url, topic, onState, onMessage, sceneTopic, onScene }) {
    Object.assign(this, { url, topic, onState, onMessage, sceneTopic, onScene });
    this.closed = false;
    this.lastReceived = 0;
    this.count = 0;
    this.rate = 0;
    this.rateStarted = performance.now();
  }

  connect() {
    this.onState("connecting");
    this.socket = new WebSocket(this.url);
    this.socket.onopen = () => {
      this.onState("waiting");
      this.socket.send(JSON.stringify({ op: "subscribe", id: "robot-joints",
        topic: this.topic, type: "sensor_msgs/msg/JointState", throttle_rate: 50, queue_length: 1 }));
      if (this.sceneTopic) this.socket.send(JSON.stringify({ op: "subscribe", id: "robot-scene",
        topic: this.sceneTopic, type: "visualization_msgs/msg/MarkerArray", throttle_rate: 200, queue_length: 1 }));
    };
    this.socket.onmessage = ({ data }) => {
      const packet = JSON.parse(data);
      if (packet.op === "publish" && packet.topic === this.sceneTopic) return this.onScene?.(packet.msg);
      if (packet.op !== "publish" || packet.topic !== this.topic) return;
      const now = performance.now();
      this.lastReceived = now;
      this.count++;
      if (now - this.rateStarted >= 1000) {
        this.rate = this.count * 1000 / (now - this.rateStarted);
        this.count = 0;
        this.rateStarted = now;
      }
      this.onMessage(packet.msg);
    };
    this.socket.onerror = () => this.onState("error");
    this.socket.onclose = () => {
      if (this.closed) return;
      this.onState("disconnected");
      this.retry = setTimeout(() => this.connect(), 2000);
    };
  }

  close() {
    this.closed = true;
    clearTimeout(this.retry);
    this.socket?.close();
  }
}
