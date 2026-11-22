import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import URDFLoader from "./vendor/urdf-loader/URDFLoader.js";

const CLOUD_COLORS = { reference: 0x5a96ff, measured: 0xff6e1e };
// 장면 마커 중 "차량·시편 메시" 표시로 숨길 것 (native/scene.py). 탐침·거치대·바닥은 위치 확인용으로 그대로 둔다
const OBJECT_MESHES = new Set(["vehicle", "holder", "contact_block", "hemisphere", "fixed_vehicle"]);
// 시뮬레이터 없이 그리는 고정 실차 (robot_files.py /api/robot/fixed-scene). 시뮬레이션 장면이 오면 그것으로 바꾼다
const FIXED_NS = "fixed_vehicle";
// RG2 finger joint at the 28.6 mm the probe is held at (onrobot_rg_control jointValueToWidth inverted):
// width = 2000 (0.055 cos(a + 0.76794) - 0.0144 + 0.108505 cos(1.41371)) [mm]
const HELD_FINGER_RAD = Math.acos((28.6 / 2000 + .0144 - .108505 * Math.cos(1.41371)) / .055) - .76794;

/** RViz-style robot display. All model transforms use ROS meters/radians and Z-up. */
export class RobotViewer {
  constructor(host) {
    this.host = host;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x18212c);
    this.camera = new THREE.PerspectiveCamera(42, 1, .001, 100)   // near 1 mm: close zoom on the probe tip;
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.6, -1.8, 1.3);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    host.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.set(.1, 0, .45);
    this.controls.minDistance = .01;   // 1 cm from the target: scan points up close
    this.controls.zoomToCursor = true;  // the wheel zooms toward the point under the mouse
    this.controls.maxDistance = 15;
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x596a80, 2.3));
    for (const [position, intensity] of [[[2, -3, 4], 2.4], [[-2, 2, 1], 1.2]]) {
      const light = new THREE.DirectionalLight(0xffffff, intensity);
      light.position.fromArray(position);
      this.scene.add(light);
    }
    this.grid = new THREE.GridHelper(3, 30, 0x718297, 0x344354);
    this.grid.rotation.x = Math.PI/2;
    this.scene.add(this.grid);
    this.baseAxes = new THREE.AxesHelper(.2);
    this.baseAxes.visible = false;   // base and TCP axes follow the 좌표축 checkbox (off by default)
    this.scene.add(this.baseAxes);
    this.frames = [];
    this.showFrames = false;
    this.showProbe = true;
    this.probeTcpMm = 282.5;  // flange -> probe tip bottom along link_6 z (real_scene.yaml probe.tcp_mm_deg, fixed-scene)
    this.showVisual = true;
    this.showCollision = false;
    this.collisionMaterial = new THREE.MeshBasicMaterial({ color: 0x5ce0bc, wireframe: true,
      transparent: true, opacity: .4, depthTest: false });
    new ResizeObserver(() => this.resize()).observe(host);
    this.renderer.setAnimationLoop(() => {
      if (!host.clientHeight || !host.clientWidth) return;
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  async load(onProgress = () => {}) {
    this.metadata = await (await fetch("/static/robot/model.json")).json();
    const manager = new THREE.LoadingManager();
    manager.onProgress = (url, loaded, total) => onProgress(loaded, total);
    const loader = new URDFLoader(manager);
    loader.parseCollision = true;
    const assetsLoaded = new Promise((resolve, reject) => {
      manager.onLoad = resolve;
      manager.onError = (url) => reject(new Error(`Model asset could not be loaded: ${url}`));
    });
    this.robot = await loader.loadAsync(`${this.metadata.model_url}?v=${this.metadata.version}`);
    await assetsLoaded;
    this.scene.add(this.robot);
    const extras = [];  // the DSR DAE meshes carry Blender lamps and cameras: ~20 point lights lit the whole scene
    this.robot.traverse((object) => {
      if (object.isLight || object.isCamera) extras.push(object);
      if (object.isURDFCollider) object.traverse((child) => {
        if (child.isMesh) child.material = this.collisionMaterial;
      });
    });
    extras.forEach((object) => object.removeFromParent());
    for (const joint of Object.values(this.robot.joints)) joint.ignoreLimits = true;
    for (const link of Object.values(this.robot.links)) {
      const axes = new THREE.AxesHelper(.045);
      axes.visible = this.showFrames;
      link.add(axes);
      this.frames.push(axes);
    }
    this.tcpFrame = this.metadata.tcp_frame;
    this.tcpAxes = new THREE.AxesHelper(.12);
    this.robot.links[this.tcpFrame].add(this.tcpAxes);
    this.loadProbe();
    this.setDisplay();
    if (this.latest) this.applyJointState(this.latest);
    this.fit();
    return this.robot;
  }

  applyJointState(message) {
    this.latest = message;
    if (!this.robot) return;
    // Master joints update mimic joints first; explicit measured children override them.
    const values = Object.fromEntries(message.name.map((name, i) => [name, message.position[i]]));
    const joints = Object.entries(values).sort(([a], [b]) =>
      Number(!!this.robot.joints[a]?.mimicJoint) - Number(!!this.robot.joints[b]?.mimicJoint));
    const holding = this.probe?.visible;
    for (const [name, value] of joints) {
      if (holding && name.startsWith("rg2_")) continue;   // drawn closed on the probe below
      if (Number.isFinite(value)) this.robot.joints[name]?.setJointValue(value);
    }
    // The real RG2 reports no finger angle (its joints stay 0: open): with the probe shown, the fingers close on it
    // at the width it is held at (real_scene.yaml held_width_mm, RG2 28.6 mm), mimic joints following.
    if (holding) this.robot.joints.rg2_finger_joint?.setJointValue(HELD_FINGER_RAD);
    this.robot.updateMatrixWorld(true);
  }

  /** Simulation scene (visualization_msgs/MarkerArray in base_link): the selected map, none without one.
   *  fixed: the real cell's vehicle at its measured pose, drawn until a simulation scene arrives. */
  applyScene(message, { fixed = false, real = false } = {}) {
    this.sceneObjects ??= new Map();
    if (fixed && this.simScene) {
      if (!real) return;
      for (const object of this.sceneObjects.values()) this.scene.remove(object);
      this.sceneObjects.clear();
      this.simScene = false;
      this.placeProbe();
    }
    if (!fixed && !this.simScene) {
      this.simScene = true;  // its own probe markers: the held probe below is hidden
      for (const [key, object] of this.sceneObjects)
        if (object.userData.ns === FIXED_NS) { this.scene.remove(object); this.sceneObjects.delete(key); }
      this.placeProbe();
    }
    for (const marker of message.markers) {
      const key = `${marker.ns}/${marker.id}`;
      if (!this.sceneObjects.has(key)) {
        const object = this.createMarker(marker);
        object.userData.ns = marker.ns;
        object.visible = !OBJECT_MESHES.has(marker.ns) || this.showObjects !== false;
        this.sceneObjects.set(key, object);
        this.scene.add(object);
      }
      const object = this.sceneObjects.get(key), { position: p, orientation: q } = marker.pose;
      object.position.set(p.x, p.y, p.z);
      object.quaternion.set(q.x, q.y, q.z, q.w);
    }
  }

  /** Point cloud in base_link (Float32Array x, y, z in metres), coloured as in the native RViz. */
  setCloud(name, positions) {
    this.clouds ??= new Map();
    let points = this.clouds.get(name);
    if (!points) {
      points = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({
        color: CLOUD_COLORS[name] ?? 0xffffff, size: 1.2, sizeAttenuation: false }));
      points.visible = this.cloudVisible?.[name] ?? true;
      this.clouds.set(name, points);
      this.scene.add(points);
    }
    points.geometry.dispose();
    points.geometry = new THREE.BufferGeometry().setAttribute("position", new THREE.BufferAttribute(positions, 3));
  }

  /** Vehicle / coupon meshes of the scene on or off, to see the measured cloud on its own. */
  setObjectsVisible(visible) {
    this.showObjects = visible;
    for (const object of this.sceneObjects?.values() || [])
      if (OBJECT_MESHES.has(object.userData.ns)) object.visible = visible;
  }

  setCloudVisible(name, visible) {
    this.cloudVisible = { ...this.cloudVisible, [name]: visible };
    const points = this.clouds?.get(name);
    if (points) points.visible = visible;
  }

  createMarker({ type, scale: { x, y, z }, color, mesh_resource }) {
    const material = new THREE.MeshStandardMaterial({ color: new THREE.Color(color.r, color.g, color.b),
      opacity: color.a, transparent: color.a < 1, side: THREE.DoubleSide });
    const group = new THREE.Group();
    const add = (geometry) => group.add(new THREE.Mesh(geometry, material));
    if (type === 1) add(new THREE.BoxGeometry(x, y, z));
    else if (type === 2) add(new THREE.SphereGeometry(x/2, 24, 16));
    else if (type === 3) add(new THREE.CylinderGeometry(x/2, x/2, z, 24).rotateX(Math.PI/2));  // ROS axis is Z
    else if (type === 10) new STLLoader().load(`/api/robot/${mesh_resource}`,  // scene/<name>.stl (robot_files.py)
      (geometry) => add(geometry.scale(x, y, z)));
    return group;
  }

  setDisplay({ visual = this.showVisual, collision = this.showCollision,
               frames = this.showFrames, grid = this.grid.visible, probe = this.showProbe } = {}) {
    Object.assign(this, { showVisual: visual, showCollision: collision, showFrames: frames, showProbe: probe });
    this.placeProbe();
    this.grid.visible = grid;
    for (const axes of [...this.frames, this.baseAxes, this.tcpAxes]) if (axes) axes.visible = frames;
    this.robot?.traverse((object) => {
      if (object.isURDFVisual) object.visible = visual;
      if (object.isURDFCollider) object.visible = collision;
    });
  }

  /** The probe held in the RG2 (meshes/probe_v3_5mm.stl, mm, tip frame: +z to its top, the ball at the bottom) on
   *  link_6 as native/probe_tool.py mounts it (grip axis x: tip +z opposite link_6 z), its ball bottom at the TCP. */
  async loadProbe() {
    try {
      const geometry = await new STLLoader().loadAsync("/static/robot/probe_v3_5mm.stl");
      geometry.scale(.001, .001, .001);
      geometry.computeBoundingBox();
      this.probe = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0xd9dee5, metalness: .25, roughness: .45 }));
      this.probe.rotation.x = Math.PI;
      this.robot.links.link_6.add(this.probe);
      this.placeProbe();
    } catch (error) { console.warn("탐침 모델 로딩 실패", error); }
  }

  /** Flange-to-tip distance (mm) of the real cell's probe TCP. */
  setProbeTcp(mm) {
    if (Number.isFinite(mm) && mm > 0) this.probeTcpMm = mm;
    this.placeProbe();
  }

  placeProbe() {
    if (!this.probe) return;
    this.probe.position.set(0, 0, this.probeTcpMm / 1000 + this.probe.geometry.boundingBox.min.z);
    this.probe.visible = this.showProbe && !this.simScene;
    if (this.latest) this.applyJointState(this.latest);   // fingers closed on it, or back to the reported angle
  }

  setTcpFrame(name) {
    if (!this.robot?.links[name]) return;
    this.tcpFrame = name;
    this.robot.links[name].add(this.tcpAxes);
  }

  pose() {
    if (!this.robot) return null;
    this.robot.updateMatrixWorld(true);
    const base = this.robot.links[this.metadata.fixed_frame];
    const link = this.robot.links[this.tcpFrame];
    const transform = base.matrixWorld.clone().invert().multiply(link.matrixWorld);
    const position = new THREE.Vector3(), quaternion = new THREE.Quaternion(), scale = new THREE.Vector3();
    transform.decompose(position, quaternion, scale);
    return { position: position.toArray(), quaternion: quaternion.toArray() };
  }

  fit() {
    if (!this.robot) return;
    const box = new THREE.Box3().setFromObject(this.robot);
    const center = box.getCenter(new THREE.Vector3());
    const distance = Math.max(box.getSize(new THREE.Vector3()).length()*1.15, .8);
    this.controls.target.copy(center);
    this.camera.position.copy(center).add(new THREE.Vector3(1, -1.3, .8).normalize().multiplyScalar(distance));
    this.controls.update();
    this.resize();
  }

  view(name) {
    const distance = this.camera.position.distanceTo(this.controls.target);
    const directions = { iso: [1, -1.3, .8], front: [1, 0, .1], side: [0, -1, .1], top: [.001, -.001, 1] };
    this.camera.position.copy(this.controls.target).add(new THREE.Vector3(...directions[name]).normalize().multiplyScalar(distance));
    this.controls.update();
  }

  resize() {
    const width = this.host.clientWidth, height = this.host.clientHeight;
    if (!width || !height) return;
    this.renderer.setSize(width, height);
    this.camera.aspect = width/height;
    this.camera.updateProjectionMatrix();
  }
}
