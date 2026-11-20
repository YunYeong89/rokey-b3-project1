"""Native MuJoCo physics and optional passive viewer; all data is protected by the backend lock."""
import threading
import time
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from ruckig import InputParameter, OutputParameter, Ruckig, Synchronization


class Scalar:
    def __init__(self, data):
        self.data = data

    def SerializeToString(self):
        return self.data


class TorqueChannel:
    def __init__(self, engine, name):
        self.engine, self.name = engine, name

    def publish_raw(self, value, _type):
        self.engine.gravity[self.name] = float(value)


class JointReference:
    """Predict moving targets and generate position/velocity/acceleration without a fixed low-pass delay."""
    def __init__(self, names, mimics, initial, dt):
        self.names, self.mimics = list(names), mimics
        self.indices = {name: i for i, name in enumerate(names)}
        self.position, self.target = initial.copy(), initial.copy()
        self.velocity = np.zeros(len(names))
        self.acceleration = np.zeros(len(names))
        self.target_velocity, self.target_acceleration = self.velocity.copy(), self.acceleration.copy()
        self.time = self.updated_at = 0.
        self.servo = False
        arm = np.array([name.startswith('joint_') for name in names])
        self.speed = np.where(arm, np.deg2rad(90), 1.5)
        self.max_acceleration = np.where(arm, np.deg2rad(180), 6.)
        self.jerk = np.where(arm, np.deg2rad(2000), 60.)
        self.generator = Ruckig(len(names), dt)
        self.input, self.output = InputParameter(len(names)), OutputParameter(len(names))
        self.input.current_position = initial.tolist()
        self.input.max_velocity = self.speed.tolist()
        self.input.max_acceleration = self.max_acceleration.tolist()
        self.input.max_jerk = self.jerk.tolist()
        self.input.synchronization = Synchronization.No
        self.lock = threading.Lock()

    def set(self, positions, velocity=None, acceleration=None, age=0.):
        with self.lock:
            if self.servo:
                self.input.current_position = self.position.tolist()
                self.input.current_velocity = self.velocity.tolist()
                self.input.current_acceleration = self.acceleration.tolist()
            self.servo = False
            target = self.target.copy()
            for name, value in positions.items():
                if name in self.indices:
                    target[self.indices[name]] = value
            elapsed = max(self.time - self.updated_at, .002)
            speed = ((target - self.target) / elapsed if velocity is None else
                     np.array([velocity.get(name, 0.) for name in self.names]))
            # A discontinuous position command is a point-to-point move with a stationary endpoint.
            speed = np.where(np.abs(speed) > self.speed, 0., speed)
            accel = ((speed - self.target_velocity) / elapsed if acceleration is None else
                     np.array([acceleration.get(name, 0.) for name in self.names]))
            accel = np.clip(accel, -self.max_acceleration, self.max_acceleration)
            # Ruckig requires enough remaining speed margin for a nonzero endpoint acceleration.
            margin = .9 * np.sqrt(2 * self.jerk * np.maximum(0., self.speed - np.abs(speed)))
            accel = np.clip(accel, -margin, margin)
            for name, (source, multiplier, offset) in self.mimics.items():
                i, j = self.indices[name], self.indices[source]
                target[i], speed[i], accel[i] = target[j] * multiplier + offset, speed[j] * multiplier, accel[j] * multiplier
            self.target = target + speed * age + .5 * accel * age**2
            self.target_velocity, self.target_acceleration = speed, accel
            self.updated_at = self.time

    def set_servo(self, positions, period):
        """The Cartesian controller already limits its increments: interpolate each tick without replanning it."""
        with self.lock:
            self.servo = True
            self.servo_start = self.position.copy()
            self.target = np.array([positions.get(name, self.position[i]) for i, name in enumerate(self.names)])
            for name, (source, multiplier, offset) in self.mimics.items():
                self.target[self.indices[name]] = self.target[self.indices[source]] * multiplier + offset
            self.servo_period, self.updated_at = period, self.time

    def advance(self, actual, dt):
        with self.lock:
            if self.servo:
                elapsed = self.time - self.updated_at + dt
                self.position = self.servo_start + min(elapsed / self.servo_period, 1.) * (self.target - self.servo_start)
                self.velocity = ((self.target - self.servo_start) / self.servo_period
                                 if elapsed <= self.servo_period + 1e-9 else np.zeros(len(self.names)))
                self.acceleration[:] = 0.
                self.time += dt
                return self.position
            ahead = min(self.time - self.updated_at + .02, .04)
            self.input.target_position = (self.target + self.target_velocity * ahead
                                          + .5 * self.target_acceleration * ahead**2).tolist()
            self.input.target_velocity = self.target_velocity.tolist()
            self.input.target_acceleration = self.target_acceleration.tolist()
            if self.generator.update(self.input, self.output) < 0:
                raise RuntimeError('Joint trajectory could not satisfy its velocity/acceleration/jerk limits')
            self.output.pass_to_input(self.input)
            self.position = np.asarray(self.output.new_position)
            self.velocity = np.asarray(self.output.new_velocity)
            self.acceleration = np.asarray(self.output.new_acceleration)
            self.time += dt
            return self.position

    def close(self):
        pass  # No wall-clock interpolation thread.


class Engine:
    def __init__(self, robot):
        self.model = getattr(robot, 'mujoco_model', None)
        if self.model is None:
            self.model = mujoco.MjModel.from_xml_path(str(robot.mujoco_path))
        self.data = mujoco.MjData(self.model)
        self.joints = {name: self.model.joint(name).id for name in robot.joints}
        self.gravity, self.viewer = {}, None
        self.model_path = robot.mujoco_path
        self.render_due = 0.
        self.actuators = {name: self.model.actuator(name).id for name in robot.joints}
        self.qpos_indices = np.array([self.model.jnt_qposadr[j] for j in self.joints.values()])
        self.dof_indices = np.array([self.model.jnt_dofadr[j] for j in self.joints.values()])
        self.reference = JointReference(robot.joints, robot.mimics, self.data.qpos[self.qpos_indices], self.model.opt.timestep)
        self.inertial_torque = np.zeros(self.model.nv)
        self.reference_acceleration = np.zeros(self.model.nv)
        # Same home configuration as the existing Gazebo world (all dynamic joints initially zero).
        mujoco.mj_forward(self.model, self.data)

    def set_targets(self, positions):
        self.reference.set(positions)

    def step(self):
        targets = self.reference.advance(self.data.qpos[self.qpos_indices], self.model.opt.timestep)
        self.reference_acceleration[self.dof_indices] = self.reference.acceleration
        mujoco.mj_mulM(self.model, self.data, self.inertial_torque, self.reference_acceleration)
        self.data.qfrc_applied[:] = 0
        for index, (name, jid) in enumerate(self.joints.items()):
            aid, dof = self.actuators[name], self.dof_indices[index]
            feedforward = (self.model.dof_damping[dof] * self.reference.velocity[index] + self.inertial_torque[dof])
            self.data.ctrl[aid] = targets[index] + feedforward / self.model.actuator_gainprm[aid, 0]
            self.data.qfrc_applied[self.model.jnt_dofadr[jid]] = self.gravity.get(name, 0.)
        mujoco.mj_step(self.model, self.data)
        positions = {name: float(self.data.qpos[self.model.jnt_qposadr[jid]]) for name, jid in self.joints.items()}
        velocity = {name: float(self.data.qvel[self.model.jnt_dofadr[jid]]) for name, jid in self.joints.items()}
        # MuJoCo force sensors measure the parent-on-child interaction; bridge convention is child-on-parent.
        wrench = -np.r_[self.data.sensor('flange_force').data, self.data.sensor('flange_torque').data]
        return positions, velocity, wrench

    def view(self):
        import mujoco.viewer
        # Keep the precise collider in physics; the GUI only needs the original visible meshes.
        root = ET.parse(self.model_path).getroot()
        for parent in root.iter():
            for child in list(parent):
                if child.tag in ('geom', 'mesh') and child.get('name', '').startswith('vehicle_collision_'):
                    parent.remove(child)
        for geom in root.iter('geom'):
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
        self.render_model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.render_data = mujoco.MjData(self.render_model)
        self.viewer = mujoco.viewer.launch_passive(self.render_model, self.render_data)
        with self.viewer.lock():
            self.viewer.cam.lookat[:] = [.3, 0, .3]
            self.viewer.cam.distance = 1.8
            self.viewer.cam.elevation = -25

    def sync_viewer(self, physics_lock):
        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            if time.monotonic() < self.render_due:
                return
            self.render_due = time.monotonic() + 1 / 30
            with self.viewer.lock(), physics_lock:
                self.render_data.qpos[:] = self.data.qpos
                self.render_data.qvel[:] = self.data.qvel
                self.render_data.eq_active[:] = self.data.eq_active
                self.render_data.time = self.data.time
            self.viewer.sync(state_only=True)

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
