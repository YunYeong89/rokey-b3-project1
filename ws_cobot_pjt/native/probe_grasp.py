"""Virtual-only, pose-checked handoff between rack and RG2 fixed grasp constraints."""
import subprocess
import time

import numpy as np


class ProbeGrasp:
    def __init__(self, backend, design):
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import AdvertiseMessageOptions, SubscribeOptions
        self.backend, self.design, self.Empty, self.StringMsg = backend, design, Empty, StringMsg
        from probe_mount import ProbeMount
        self.mount = ProbeMount(backend.node)
        self.states = {}
        self.publishers = {}
        for group in ['dock', 'grip']:
            for action in ['attach', 'detach']:
                self.publishers[group, action] = backend.node.advertise(
                    f'/cobot/probe/{group}/{action}', 'gz.msgs.Empty', AdvertiseMessageOptions())
            def receive(data, info, name=group):
                value = StringMsg(); value.ParseFromString(data); self.states[name] = value.data
            backend.node.subscribe_raw(f'/cobot/probe/{group}/state', receive,
                                       'gz.msgs.StringMsg', SubscribeOptions())

    def pause(self, enabled):
        subprocess.run(['gz', 'service', '-s', '/world/cobot_world/control',
                        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '3000', '--req', f'pause: {str(enabled).lower()}'],
                       check=True, capture_output=True, text=True)

    def switch(self, group, attached):
        expected = 'attached' if attached else 'detached'
        deadline = time.monotonic() + 3
        while self.states.get(group) != expected:
            self.publishers[group, 'attach' if attached else 'detach'].publish_raw(
                self.Empty().SerializeToString(), 'gz.msgs.Empty')
            if time.monotonic() > deadline:
                raise TimeoutError(f'Probe {group} did not become {expected}')
            time.sleep(.05)

    def initialize(self):
        # Gazebo 8 creates detachable joints attached initially. Keep physics paused
        # until the robot constraint is detached and the rack constraint is confirmed.
        time.sleep(2)
        self.pause(True)
        self.switch('grip', False)
        self.switch('dock', False)
        self.switch('dock', True)
        self.pause(False)

    def attach(self, positions):
        transforms, _ = self.design.chain.transforms(positions)
        flange = np.linalg.inv(self.design.world_base) @ transforms['link_6']
        error = np.linalg.norm(flange[:3, 3]-self.design.park_flange[:3, 3])
        rotation = flange[:3, :3].T @ self.design.park_flange[:3, :3]
        angle = np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1))
        finger = positions.get(self.design.joint, -1)
        if error > .002 or angle > np.deg2rad(2) or abs(finger-self.design.close_angle) > .03:
            return {'success':False, 'message':f'Grasp alignment rejected: {error*1000:.2f} mm, '
                    f'{np.rad2deg(angle):.2f} deg, finger={finger:.3f} rad'}
        fit_error = self.design.grasp_error(positions)
        if fit_error:
            return {'success': False, 'message': fit_error}
        self.pause(True)
        try:
            flange_body = self.mount.transform()
            self.switch('dock', False)
            try:
                self.switch('grip', True)
            except Exception:
                self.switch('dock', True)
                raise
            self.design.set_mount(flange_body)
            self.design.attached = True
        finally:
            self.pause(False)
        return {'success':True, 'message':'Probe grasp fixed; rack released'}

    def gravity_wrench(self, sensor_pose, positions):
        if not self.design.attached:
            return np.zeros(6)
        transforms, _ = self.design.chain.transforms(positions)
        center = (transforms['link_6'] @ self.design.flange_body @ np.r_[self.design.com, 1])[:3]
        force = np.array([0., 0., -9.81*self.design.mass])
        return np.r_[force, np.cross(center-sensor_pose[:3, 3], force)]
