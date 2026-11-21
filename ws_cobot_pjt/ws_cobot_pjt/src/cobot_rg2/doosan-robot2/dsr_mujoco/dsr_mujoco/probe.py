"""Pose-verified MuJoCo weld handoff using the existing pickup protocol."""
from probe_grasp import ProbeGrasp


class MuJoCoProbeGrasp(ProbeGrasp):
    def __init__(self, backend, design):
        self.backend, self.design = backend, design

    def initialize(self):
        pass  # The exported model starts with dock weld active and grip weld inactive.

    def pause(self, enabled):
        if enabled:
            self.backend.lock.acquire()
        else:
            self.backend.lock.release()

    def switch(self, group, attached):
        engine = self.backend.engine
        index = engine.model.equality('probe_' + group).id
        engine.data.eq_active[index] = attached
