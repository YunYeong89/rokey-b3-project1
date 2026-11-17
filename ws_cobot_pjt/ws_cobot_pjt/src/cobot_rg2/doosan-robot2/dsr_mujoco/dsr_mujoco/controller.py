"""Emulator replacement for the accelerated MuJoCo session (run_virtual.sh --acc): the DRCF services the robot
programs use, answered in simulation time.

movej: the joints on one synchronised trapezoidal profile (vel / acc of the joint moving most, or `time`). movel: the
TCP on a straight line and its orientation about one fixed axis, on the profile of whichever is slower (vel / acc
[mm/s, deg/s], or `time`), every point by IK from the last; the path is checked for reach, joint limits,
singularities and joint speeds before it starts and refused with an alarm otherwise (no motion). A move sent during
another one runs after it; move_stop brakes along the path. FK / IK of the active TCP (profiles are flange relative,
ZYZ deg), solution spaces, TCP / tool profiles (changed in manual mode only), robot mode, the RG2 at 1 rad/s.
Requests and responses in mm / deg as on the DSR services, SI inside."""
import numpy as np

from kinematics import KinematicChain, rotation, rotation_error, skew
from rokey.rotations import rotation_zyz

GRIPPER = {'o': -0.558505, 'c': 0.785398}                        # rg2_finger_joint open / closed
GRIPPER_SPEED = 1.0                                               # rad/s, as gripper_virtual_node
JOINT_SPEED = np.deg2rad([150., 150., 180., 225., 225., 225.])   # M0609 joint speed limits
IDLE, BUSY = 0, 2                                                 # check_motion: DR_STATE_IDLE / DR_STATE_BUSY
MANUAL, AUTONOMOUS = 0, 1


class Rejected(Exception):
    """A refused request: no motion and a failed response; with `alarm` also a controller alarm, as DRCF raises."""

    def __init__(self, text, alarm=True):
        super().__init__(text)
        self.alarm = alarm


def matrix(pose):
    """DSR pose (mm, ZYZ deg) -> 4x4 (m)."""
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation_zyz(pose[3:]), np.asarray(pose[:3], float) / 1000
    return result


def posx(pose):
    """4x4 (m) -> DSR pose (mm, ZYZ deg with B in [0, 180]; A = 0 at B = 0 / 180)."""
    r = pose[:3, :3]
    b = np.arccos(np.clip(r[2, 2], -1, 1))
    if np.sin(b) > 1e-7:
        a, c = np.arctan2(r[1, 2], r[0, 2]), np.arctan2(r[2, 1], -r[2, 0])
    else:
        a, c = 0., np.arctan2(r[1, 0], r[1, 1] if r[2, 2] < 0 else r[0, 0])
    return [*(pose[:3, 3] * 1000).tolist(), *np.rad2deg([a, b, c]).tolist()]


class Kinematics:
    """Flange (link_6) chain of the session URDF with the active TCP offset; IK by damped least squares."""

    def __init__(self, robot):
        self.chain = KinematicChain(robot, tcp_link='link_6')
        self.names = self.chain.names
        self.low, self.high = np.array([self.chain.limits[name] for name in self.names]).T
        self.tcp = np.eye(4)

    def forward(self, q):
        """TCP pose (4x4, m) and its BASE Jacobian at joints q (rad)."""
        flange, jacobian = self.chain.pose_jacobian(dict(zip(self.names, q)))
        pose = flange @ self.tcp
        lever = skew(pose[:3, 3] - flange[:3, 3])
        return pose, np.r_[jacobian[:3] - lever @ jacobian[3:], jacobian[3:]]

    def solve(self, target, q, iterations=30):
        """Joints reaching the TCP `target` from q; None when they do not converge or leave the joint limits."""
        q = np.array(q, float)
        for _ in range(iterations):
            pose, jacobian = self.forward(q)
            error = np.r_[target[:3, 3] - pose[:3, 3], rotation_error(target[:3, :3], pose[:3, :3])]
            if np.linalg.norm(error[:3]) < 1e-6 and np.linalg.norm(error[3:]) < 1e-5:
                return q if np.all((q >= self.low) & (q <= self.high)) else None
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-8 * np.eye(6), error)
            q = q + np.clip(step, -.2, .2)
        return None

    def singular(self, q, limit=1e-3):
        return np.linalg.svd(self.forward(q)[1], compute_uv=False)[-1] < limit

    def space(self, q):
        """DRCF solution space: + 4 wrist centre behind the base axis (shoulder), + 2 joint 3 > 0 (elbow),
        + 1 joint 5 < 0 (wrist flip); HOME (0, 0, 90, 0, 90, 0) is 2."""
        _, axes = self.chain.transforms(dict(zip(self.names, q)))
        ahead = np.array([np.cos(q[0]), np.sin(q[0]), 0.])
        return 4 * int(axes['joint_5'][0] @ ahead < 0) + 2 * int(q[2] > 0) + int(q[4] < 0)

    def inverse(self, target, space, current):
        """Joints at the TCP `target` in solution space `space`: IK from the current joints, then from seeds of
        every space (base turned towards the target or away, elbow and wrist either way)."""
        yaw = np.arctan2(target[1, 3], target[0, 3])
        seeds = [current] + [np.array([yaw + shoulder, lean, elbow, 0., wrist, 0.])
                             for shoulder in (0., np.pi) for lean in (0., -.6, .6)
                             for elbow in (np.pi / 2, -np.pi / 2) for wrist in (np.pi / 2, -np.pi / 2)]
        for seed in seeds:
            q = self.solve(target, seed, 100)
            if q is not None and self.space(q) == space:
                return q
        return None


class Trapezoid:
    """Normalised path 0 -> 1 on a trapezoidal speed profile: `length` at `speed` / `acc` (triangular when too short),
    or lasting `duration` (a quarter speeding up, a quarter braking) when that is given, as the movej / movel `time`
    overrides vel / acc."""

    def __init__(self, length, speed, acc, duration=0.):
        if duration > 0:
            self.ramp, self.peak, self.duration = duration / 4, 4 / (3 * duration), duration
        elif length <= 1e-9:
            self.ramp, self.peak, self.duration = 0., 0., 0.
        else:
            if speed <= 0 or acc <= 0:
                raise Rejected('motion speed and acceleration must be positive')
            ramp = speed / acc
            if speed * ramp > length:
                ramp = np.sqrt(length / acc)
                speed = acc * ramp
            self.ramp, self.peak, self.duration = ramp, speed / length, ramp + length / speed

    def sample(self, t):
        """Path parameter and its rate at time t."""
        r, v, d = self.ramp, self.peak, self.duration
        if t >= d:
            return 1., 0.
        a = v / r
        if t < r:
            return .5 * a * t * t, a * t
        if t < d - r:
            return .5 * v * r + v * (t - r), v
        return 1. - .5 * a * (d - t) ** 2, a * (d - t)


class Motion:
    def __init__(self, profile):
        self.profile, self.time, self.braking = profile, 0., None

    def advance(self, dt):
        """Path parameter after dt, and whether the motion is over."""
        self.time += dt
        if self.braking is None:
            return self.profile.sample(self.time)[0], self.time >= self.profile.duration
        start, s, speed, rate = self.braking
        elapsed = min(self.time - start, speed / rate)
        return min(1., s + speed * elapsed - .5 * rate * elapsed ** 2), elapsed >= speed / rate

    def stop(self, scale):
        """Brake along the path at `scale` times the profile's acceleration."""
        if self.braking is None:
            s, speed = self.profile.sample(self.time)
            self.braking = (self.time, s, speed, max(scale * self.profile.peak / max(self.profile.ramp, 1e-3), 1e-9))


class JointMotion(Motion):
    def __init__(self, start, goal, profile):
        super().__init__(profile)
        self.start, self.delta = start, goal - start

    def at(self, s):
        return self.start + s * self.delta


class LineMotion(Motion):
    def __init__(self, kinematics, q, start, goal, profile, length, angle):
        super().__init__(profile)
        self.kinematics, self.q, self.start, self.goal = kinematics, q, start, goal
        self.turn = rotation_error(goal[:3, :3], start[:3, :3])
        self.length, self.angle = length, angle   # mm, deg

    def pose(self, s):
        pose = np.eye(4)
        pose[:3, 3] = self.start[:3, 3] + s * (self.goal[:3, 3] - self.start[:3, 3])
        pose[:3, :3] = rotation(s * self.turn) @ self.start[:3, :3]
        return pose

    def at(self, s):
        q = self.kinematics.solve(self.pose(s), self.q)
        if q is None:
            raise Rejected('movel stopped: inverse kinematics failed on the path')
        self.q = q
        return q

    def check(self):
        """Refuse a path that leaves the reach or the joint limits, passes a singularity or needs too fast a joint
        (samples every 5 mm / 2 deg)."""
        count = int(np.ceil(max(self.length / 5., self.angle / 2.))) + 1
        q, step = self.q, 1. / count
        for s in np.linspace(step, 1., count):
            following = self.kinematics.solve(self.pose(s), q)
            if following is None:
                raise Rejected('movel target or path out of reach / joint limits')
            if self.kinematics.singular(following):
                raise Rejected('movel path passes a singularity')
            if np.any(np.abs(following - q) * self.profile.peak / step > JOINT_SPEED):
                raise Rejected('movel path exceeds the joint speed limits')
            q = following


class EmulatedController:
    """DRCF state and motions. `command(name, request, actual)`: the response values of a DSR request (`actual`:
    MuJoCo joints), Rejected when refused; `advance(dt)` once per control period: the nominal joints with the RG2,
    their velocities and accelerations."""

    def __init__(self, robot, profiles, initial):
        self.kinematics = Kinematics(robot)
        self.names = self.kinematics.names
        self.tcps = {name: [float(v) for v in value['pos']] for name, value in profiles['tcps'].items()}
        self.tools = {name: dict(value) for name, value in profiles['tools'].items()}
        self.tool = profiles['active']['tool']
        self.select_tcp(profiles['active']['tcp'])
        self.mode, self.reference = AUTONOMOUS, 0
        self.q = np.array([initial.get(name, 0.) for name in self.names])
        self.velocity = np.zeros(6)
        self.gripper = self.gripper_target = float(initial.get('rg2_finger_joint', 0.))
        self.motion, self.queue = None, []
        self.alarm = {'id': 0}
        self.handlers = {
            'system/get_robot_system': lambda r, q: {'robot_system': 1},   # virtual
            'system/get_robot_mode': lambda r, q: {'robot_mode': self.mode},
            'system/set_robot_mode': self.set_mode,
            'motion/move_joint': self.move_joint,
            'motion/move_line': self.move_line,
            'motion/check_motion': lambda r, q: {'status': BUSY if self.busy() else IDLE},
            'motion/move_stop': self.stop,
            'motion/fkin': self.fkin,
            'motion/ikin': self.ikin,
            'motion/set_ref_coord': self.set_reference,
            'tool/get_current_tool': lambda r, q: {'info': self.tool},
            'tool/set_current_tool': self.set_tool,
            'tool/config_create_tool': self.create_tool,
            'tcp/get_current_tcp': lambda r, q: {'info': self.tcp},
            'tcp/set_current_tcp': self.set_tcp,
            'tcp/config_create_tcp': self.create_tcp,
            'aux_control/get_current_posx': self.current_posx,
            'aux_control/get_current_posj': lambda r, q: {'pos': np.rad2deg(q).tolist()},
            'gripper': self.command_gripper,
        }

    def command(self, name, request, actual):
        q = np.array([actual.get(joint, value) for joint, value in zip(self.names, self.q)])
        try:
            return self.handlers[name](request, q)
        except Rejected as error:
            if error.alarm:
                self.raise_alarm(f'{name}: {error}')
            raise

    def busy(self):
        return self.motion is not None or bool(self.queue)

    def raise_alarm(self, text):
        self.alarm = {'id': self.alarm['id'] + 1, 'level': 2, 'group': 2, 'code': 9000, 'text': text}

    def state(self):
        return {'motion': BUSY if self.busy() else IDLE, 'gripper': self.gripper, 'alarm': self.alarm}

    def advance(self, dt):
        """Nominal joints after dt (with the RG2), their velocities and accelerations, by joint name."""
        previous, velocity = self.q.copy(), self.velocity
        if self.motion is None and self.queue:
            try:
                self.motion = self.queue.pop(0)(self.q)
            except Rejected as error:
                self.raise_alarm(str(error))
                self.queue.clear()
        if self.motion is not None:
            try:
                s, done = self.motion.advance(dt)
                self.q = self.motion.at(s)
            except Rejected as error:
                self.raise_alarm(str(error))
                self.queue.clear()
                done = True
            if done:
                self.motion = None
        self.velocity = (self.q - previous) / dt
        acceleration = (self.velocity - velocity) / dt
        step = float(np.clip(self.gripper_target - self.gripper, -GRIPPER_SPEED * dt, GRIPPER_SPEED * dt))
        self.gripper += step
        # Python floats: the backend's snapshot compares them (numpy bools are not JSON for the bridge)
        names = [*self.names, 'rg2_finger_joint']
        return (dict(zip(names, map(float, [*self.q, self.gripper]))),
                dict(zip(names, map(float, [*self.velocity, step / dt]))),
                dict(zip(names, map(float, [*acceleration, 0.]))))

    def reset(self, positions):
        """The nominal joints jump to where the robot is held (after compliance / the surface stream)."""
        self.q = np.array([positions[name] for name in self.names])
        self.velocity[:] = 0
        self.motion, self.queue = None, []

    def start(self, plan):
        """Run plan(start joints) now, or after the running motions."""
        if self.busy():
            self.queue.append(plan)
        else:
            self.motion = plan(self.q)
        return {}

    def move_joint(self, request, _):
        def plan(q):
            goal = np.deg2rad(request['pos']) + (q if request['mode'] == 1 else 0.)
            if np.any(goal < self.kinematics.low) or np.any(goal > self.kinematics.high):
                raise Rejected('movej target outside the joint limits')
            delta = np.rad2deg(np.abs(goal - q)).max()
            return JointMotion(q, goal, Trapezoid(delta, request['vel'], request['acc'], request['time']))
        return self.start(plan)

    def move_line(self, request, _):
        def plan(q):
            start = self.kinematics.forward(q)[0]
            goal = self.target(request['pos'], request['ref'], request['mode'], start)
            length = np.linalg.norm(goal[:3, 3] - start[:3, 3]) * 1000
            angle = np.rad2deg(np.linalg.norm(rotation_error(goal[:3, :3], start[:3, :3])))
            profile = max(Trapezoid(length, request['vel'][0], request['acc'][0], request['time']),
                          Trapezoid(angle, request['vel'][1], request['acc'][1], request['time']),
                          key=lambda p: p.duration)
            motion = LineMotion(self.kinematics, q, start, goal, profile, length, angle)
            motion.check()
            return motion
        return self.start(plan)

    @staticmethod
    def target(pos, reference, mode, start):
        """movel goal: BASE / WORLD absolute, BASE relative (offset and turn in BASE), or in the TOOL frame at the
        start (absolute and relative alike)."""
        pose = matrix(pos)
        if reference == 1:
            return start @ pose
        if reference not in (0, 2):
            raise Rejected(f'unsupported reference {reference}')
        if mode == 1:
            pose[:3, 3] += start[:3, 3]
            pose[:3, :3] = pose[:3, :3] @ start[:3, :3]
        return pose

    def stop(self, request, _):
        self.queue.clear()
        if self.motion is not None:
            self.motion.stop(1. if request['stop_mode'] == 2 else 4.)   # DR_SSTOP slow, the others quick
        return {}

    @staticmethod
    def base(request):
        if request['ref'] not in (0, 2):
            raise Rejected('only the BASE / WORLD reference is supported', alarm=False)

    def fkin(self, request, _):
        self.base(request)
        return {'conv_posx': posx(self.kinematics.forward(np.deg2rad(request['pos']))[0])}

    def ikin(self, request, q):
        self.base(request)
        joints = self.kinematics.inverse(matrix(request['pos']), request['sol_space'], q)
        if joints is None:
            raise Rejected(f'no IK solution in solution space {request["sol_space"]}', alarm=False)
        return {'conv_posj': np.rad2deg(joints).tolist()}

    def current_posx(self, request, q):
        self.base(request)
        return {'posx': [*posx(self.kinematics.forward(q)[0]), float(self.kinematics.space(q))]}

    def set_mode(self, request, _):
        if request['robot_mode'] not in (MANUAL, AUTONOMOUS):
            raise Rejected(f'unsupported robot mode {request["robot_mode"]}')
        self.mode = request['robot_mode']
        return {}

    def set_reference(self, request, _):
        if request['coord'] not in (0, 1, 2):
            raise Rejected(f'unsupported reference {request["coord"]}')
        self.reference = request['coord']
        return {}

    def manual(self, kind, name, known):
        if self.mode != MANUAL:
            raise Rejected(f'{kind} profiles change in manual mode only')
        if known is not None and name not in known:
            raise Rejected(f'unknown {kind} {name!r}')

    def set_tool(self, request, _):
        self.manual('tool', request['name'], self.tools)
        self.tool = request['name']
        return {}

    def create_tool(self, request, _):
        self.manual('tool', request['name'], None)
        self.tools[request['name']] = {key: request[key] for key in ('weight', 'cog', 'inertia')}
        return {}

    def select_tcp(self, name):
        self.tcp, self.kinematics.tcp = name, matrix(self.tcps[name])

    def set_tcp(self, request, _):
        self.manual('TCP', request['name'], self.tcps)
        self.select_tcp(request['name'])
        return {}

    def create_tcp(self, request, _):
        self.manual('TCP', request['name'], None)
        self.tcps[request['name']] = [float(v) for v in request['pos']]
        return {}

    def command_gripper(self, request, _):
        command = request['command']
        try:
            target = GRIPPER[command] if command in GRIPPER else float(command)
        except ValueError:
            raise Rejected(f'unknown gripper command {command!r}', alarm=False) from None
        self.gripper_target = float(np.clip(target, GRIPPER['o'], GRIPPER['c']))
        return {'target': self.gripper_target}
