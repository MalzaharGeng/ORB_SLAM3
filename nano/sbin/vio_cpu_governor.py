#!/usr/bin/python3
"""Apply a temporary CPU governor while VIO runs, without changing caps."""
import json,os,sys
from pathlib import Path
ALLOWED={'schedutil','performance','ondemand','userspace','powersave','conservative'}
POLICY_ROOT=Path('/sys/devices/system/cpu/cpufreq');STATE_PATH=Path('/run/vio-cpu-governors.json')
def configure(action,root=POLICY_ROOT,state_path=STATE_PATH):
    policies=sorted(root.glob('policy*'))
    if not policies:raise ValueError('CPU frequency policies unavailable')
    current={p.name:(p/'scaling_governor').read_text().strip() for p in policies}
    if any(v not in ALLOWED for v in current.values()):raise ValueError('Unsupported current CPU governor')
    if action=='restore':
        if not state_path.exists():return
        saved=json.loads(state_path.read_text());
        if set(saved)!={p.name for p in policies} or any(v not in ALLOWED for v in saved.values()):raise ValueError('Invalid saved CPU governor state')
        for p in policies:(p/'scaling_governor').write_text(saved[p.name])
        state_path.unlink();return
    if action!='set':raise ValueError('Expected set or restore')
    if any('performance' not in (p/'scaling_available_governors').read_text().split() for p in policies):raise ValueError('performance governor unavailable')
    if state_path.exists():
        saved=json.loads(state_path.read_text())
        if set(saved)!={p.name for p in policies}:raise ValueError('Invalid saved CPU governor state')
        if all(v=='performance' for v in current.values()):return
        raise ValueError('Existing CPU governor state is active')
    fd=os.open(state_path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    try:
        with os.fdopen(fd,'w') as f:json.dump(current,f);f.flush();os.fsync(f.fileno())
        for p in policies:(p/'scaling_governor').write_text('performance')
    except BaseException:
        for p in policies:
            try:(p/'scaling_governor').write_text(current[p.name])
            except OSError:pass
        state_path.unlink(missing_ok=True);raise
    print(json.dumps({'cpu_governor':'performance','power_mode_and_frequency_caps':'unchanged'}),flush=True)
if __name__=='__main__':
    if os.geteuid()!=0 or len(sys.argv)!=2:raise SystemExit('Usage as root: vio_cpu_governor.py set|restore')
    configure(sys.argv[1])
