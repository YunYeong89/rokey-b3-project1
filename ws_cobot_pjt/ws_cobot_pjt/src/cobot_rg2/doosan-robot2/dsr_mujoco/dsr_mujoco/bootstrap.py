"""Locate the existing native workspace without modifying its packages."""
from pathlib import Path
import os
import sys

def workspace():
    candidate = os.environ.get('COBOT_WORKSPACE')
    roots = [Path(candidate)] if candidate else Path(__file__).resolve().parents
    for root in roots:
        if (root / 'native/session.py').is_file():
            sys.path.insert(0, str(root / 'native'))
            sys.path.insert(0, str(root / 'ws_cobot_pjt/src/rokey'))
            return root
    raise RuntimeError('Set COBOT_WORKSPACE to the host ws_cobot_pjt directory containing native/session.py')

def main():
    workspace()
    from .session import main as run
    run()

if __name__ == '__main__':
    main()
