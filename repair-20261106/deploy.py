"""Deploy only on an idle Nano with no attached flight-controller USB serial."""
import datetime
import hashlib
import json
from pathlib import Path
import shlex
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nano_ssh import connect,PW

ROOT=Path(__file__).resolve().parent
REMOTE='/home/nano/vio_repair_20261106'
FILES=['src/Px4Vio.cc','include/Px4Vio.h','src/Px4Bridge.cc','config/px4_vio_local.yaml','CMakeLists.txt','tests/px4_recording_gate_regression.cc']

def main():
    c=connect(PW)
    try:
        s=c.open_sftp()
        # Reuse a unique audited installation directory, never overwrite a prior backup.
        try:s.stat(REMOTE)
        except OSError:s.mkdir(REMOTE)
        else:raise RuntimeError('Remote deployment directory already exists')
        staged_hashes={}
        for f in FILES+['system/vio-clock-prepare']:
            local=ROOT/'staged'/f
            remote=REMOTE+'/'+f.replace('/','__')
            s.put(str(local),remote)
            staged_hashes[f]=hashlib.sha256(local.read_bytes()).hexdigest()
        s.put(str(ROOT/'baseline_hashes.json'),REMOTE+'/baseline_hashes.json')
        s.put(str(ROOT/'test_clock.py'),REMOTE+'/test_clock.py')
        code=r'''
import datetime,glob,hashlib,json,os,pathlib,shutil,subprocess,sys,time
root=pathlib.Path('/home/nano/ORB_SLAM3')
work=pathlib.Path('/home/nano/vio_repair_20261106')
files=FILES_PLACEHOLDER
hashes=json.loads((work/'baseline_hashes.json').read_text())
for path,expected in hashes.items():
    assert hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()==expected,'Baseline changed: '+path
assert not glob.glob('/dev/ttyACM*') and not glob.glob('/dev/ttyUSB*'),'Controller serial attached; refusing maintenance'
def assert_idle():
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:name=(p/'comm').read_text().strip()
        except OSError:continue
        assert not name.startswith('stereo_inertial'),'VIO running; refusing maintenance'
assert_idle()
active=subprocess.run(['systemctl','is-active','px4-vio.service'],capture_output=True,text=True).stdout.strip()=='active'
(work/'service_was_active').write_text(str(active))
subprocess.run(['systemctl','stop','px4-vio.service'],check=True,timeout=55)
assert_idle()
backup=work/'backup'
backup.mkdir()
for f in files:
    source=root/f
    if source.exists():
        dst=backup/f;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dst)
for f in ['build/lib/libORB_SLAM3.so','build/liborbslam3_px4.a','build/liborbslam3_px4_core.a','build/Examples/Stereo-Inertial/stereo_inertial_realsense_D435i','build/tests/px4_bridge_fixture','build/tests/d435i_recovery_replay']:
    source=root/f
    if source.exists():
        dst=backup/f;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dst)
shutil.copy2('/usr/local/sbin/vio-clock-prepare',backup/'vio-clock-prepare')
state=pathlib.Path('/var/lib/vio-clock');state.mkdir(parents=True,exist_ok=True)
for f in ['last-good.json','clock-origin.json']:
    if (state/f).exists():shutil.copy2(state/f,backup/('clock_'+f))
stamp=HOST_TIMESTAMP_PLACEHOLDER
assert 2024<=datetime.datetime.fromtimestamp(stamp,datetime.timezone.utc).year<2100,'Invalid host timestamp'
before=time.time()
subprocess.run(['/bin/date','--set','@%.6f'%stamp],check=True,timeout=10)
(state/'last-good.json').write_text(json.dumps({'seconds':stamp,'source':'maintenance_host_utc'}))
(state/'host-sync.json').write_text(json.dumps({'source':'maintenance_host_utc','before_seconds':before,'host_seconds':stamp,'system_seconds':time.time()}))
for f in files:
    target=root/f;target.parent.mkdir(parents=True,exist_ok=True)
    temp=target.with_name(target.name+'.repair_tmp')
    shutil.copyfile(work/f.replace('/','__'),temp);os.chown(temp,1000,1000);os.replace(temp,target)
clock=pathlib.Path('/usr/local/sbin/vio-clock-prepare')
shutil.copyfile(work/'system__vio-clock-prepare',clock);os.chown(clock,0,0);os.chmod(clock,0o755)
subprocess.run([str(clock)],check=True,timeout=20)
print('DEPLOYED_AND_CLOCK_RESTORED; service remains stopped for build')
'''.replace('FILES_PLACEHOLDER',repr(FILES)).replace('HOST_TIMESTAMP_PLACEHOLDER',repr(time.time()))
        stdin,out,err=c.exec_command('sudo -S -p "" /usr/bin/python3 -c '+shlex.quote(code),timeout=90)
        stdin.write(PW+'\n');stdin.flush();stdin.channel.shutdown_write()
        a=out.read().decode(errors='replace');b=err.read().decode(errors='replace');rc=out.channel.recv_exit_status()
        (ROOT/'deploy.log').write_text(a+'\n'+b,encoding='utf-8')
        print(a,b)
        if rc:raise SystemExit(rc)
        (ROOT/'staged_hashes.json').write_text(json.dumps(staged_hashes,indent=2),encoding='utf-8')
        s.close()
    finally:c.close()

if __name__=='__main__':main()
