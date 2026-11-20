"""A robot program run by web_control: its own process group, its output kept for the web log."""
import os
import signal
import subprocess
import tempfile
import time


class ManagedProcess:
    def __init__(self, arguments, env=None):
        self.output = tempfile.TemporaryFile(mode='w+b')
        self.process = subprocess.Popen(arguments, stdout=self.output, stderr=subprocess.STDOUT,
                                        start_new_session=True, env=env)
        self.started_at = time.time()
        self.stop_due, self.killed = 0., False

    def log(self, size=6000):
        end = os.fstat(self.output.fileno()).st_size
        return os.pread(self.output.fileno(), size, max(0, end - size)).decode(errors='replace')

    def interrupt(self, grace):
        """SIGINT: the program stops the robot and releases force and compliance in its finally; SIGKILL from poll()
        once `grace` seconds have passed."""
        if not self.stop_due:
            os.killpg(self.process.pid, signal.SIGINT)
            self.stop_due = time.monotonic() + grace

    def poll(self):
        code = self.process.poll()
        if code is None and self.stop_due and not self.killed and time.monotonic() > self.stop_due:
            os.killpg(self.process.pid, signal.SIGKILL)
            self.killed = True
        return code

    def close(self):
        self.output.close()
