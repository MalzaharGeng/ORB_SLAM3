#!/usr/bin/env python3
"""Run the private Agent 2.4.3; restart it when the USB device is re-enumerated."""
import argparse
import os
import pathlib
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', type=pathlib.Path, required=True, help='/dev/serial/by-id/...')
    parser.add_argument('--agent', type=pathlib.Path,
                        default=pathlib.Path.home()/'.cache/orbslam3-px4/agent_install/bin/MicroXRCEAgent')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    binary = args.agent.resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error(f'Agent missing: {binary}; run setup_px4_dependencies.py --agent')
    command = [str(binary), 'serial', '--dev', str(args.device), '-b', '921600']
    print('Agent command:', command, flush=True)
    if args.check:
        print('USB present:', args.device.exists())
        return
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    # Agent's FastDDS/FastCDR stay private; do not inherit ROS or MVS library paths.
    environment = dict(os.environ)
    environment['LD_LIBRARY_PATH'] = str(binary.parent.parent/'lib')
    child = None
    def identity():
        try:
            st = args.device.stat()
            return st.st_rdev, st.st_ino
        except FileNotFoundError:
            return None
    try:
        while running:
            current = identity()
            if current is None:
                time.sleep(.25)
                continue
            if not os.access(args.device, os.R_OK | os.W_OK):
                raise PermissionError(f'No read/write permission for {args.device}')
            child = subprocess.Popen(command, env=environment)
            while running and child.poll() is None and identity() == current:
                time.sleep(.25)
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            child = None
            if running:
                print('USB/Agent disconnected; waiting to reconnect', flush=True)
                time.sleep(1)
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    main()
