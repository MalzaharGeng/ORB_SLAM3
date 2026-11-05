"""Archive authoritative source without exposing connection credentials."""
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nano_ssh import connect, PW

ROOT = Path(__file__).resolve().parent
FILES = ['src/Px4Vio.cc', 'include/Px4Vio.h', 'src/Px4Bridge.cc',
         'config/px4_vio_local.yaml', 'config/px4_vio.yaml',
         'tests/px4_vio_regression.cc', 'CMakeLists.txt']
SYSTEM = ['/usr/local/sbin/vio-clock-prepare', '/etc/systemd/system/px4-vio.service']

def main():
    c = connect(PW)
    try:
        _, out, err = c.exec_command('for p in /AGENTS.md /home/AGENTS.md /home/nano/AGENTS.md /home/nano/ORB_SLAM3/AGENTS.md; do if [ -f "$p" ]; then cat "$p"; fi; done; systemctl is-active px4-vio.service; pgrep -af "stereo_inertial|cmake --build"; cd /home/nano/ORB_SLAM3 && find src include config tests -name AGENTS.md -print', timeout=20)
        print(out.read().decode(errors='replace'), err.read().decode(errors='replace'))
        s = c.open_sftp()
        hashes = {}
        for remote, local in [(f'/home/nano/ORB_SLAM3/{f}', f) for f in FILES] + [(f, 'system/' + Path(f).name) for f in SYSTEM]:
            dst = ROOT / 'baseline' / local
            if dst.exists():
                raise RuntimeError(f'Baseline already exists: {dst}')
            dst.parent.mkdir(parents=True, exist_ok=True)
            s.get(remote, str(dst))
            hashes[remote] = hashlib.sha256(dst.read_bytes()).hexdigest()
        (ROOT / 'baseline_hashes.json').write_text(json.dumps(hashes, indent=2), encoding='utf-8')
        s.close()
        print('BASELINE_ARCHIVED', len(hashes))
    finally:
        c.close()

if __name__ == '__main__':
    main()
