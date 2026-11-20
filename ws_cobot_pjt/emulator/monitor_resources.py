"""Record Docker resource snapshots during the emulator soak run."""
import argparse
import json
from pathlib import Path
import subprocess
import time


class ResourceMonitor:
    def run(self, container, seconds, output):
        start = time.monotonic()
        with output.open('w') as file:
            while time.monotonic() - start < seconds:
                result = subprocess.run(
                    ['docker', 'stats', '--no-stream', '--format', '{{json .}}', container],
                    capture_output=True, text=True, check=True)
                row = json.loads(result.stdout)
                row['elapsed_s'] = time.monotonic() - start
                file.write(json.dumps(row) + '\n')
                file.flush()
                print(f"{row['elapsed_s']:.0f}s CPU={row['CPUPerc']} memory={row['MemUsage']}", flush=True)
                time.sleep(30)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('container')
    parser.add_argument('seconds', type=int)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    ResourceMonitor().run(args.container, args.seconds, args.output)
