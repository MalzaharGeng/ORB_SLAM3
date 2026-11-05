"""Verify installed artifacts, collect evidence, restore the prior waiting service."""
import hashlib
import json
from pathlib import Path
import shlex
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nano_ssh import connect,PW

ROOT=Path(__file__).resolve().parent
REMOTE='/home/nano/vio_repair_20261106'

def main():
    c=connect(PW)
    try:
        s=c.open_sftp()
        with s.open(REMOTE+'/exit_code') as f:
            code=f.read().decode().strip()
        if code!='0':raise RuntimeError('Build/tests did not succeed: '+code)
        hashes=json.loads((ROOT/'staged_hashes.json').read_text())
        for relative,expected in hashes.items():
            target='/usr/local/sbin/vio-clock-prepare' if relative.startswith('system/') else '/home/nano/ORB_SLAM3/'+relative
            with s.open(target,'rb') as f:actual=hashlib.sha256(f.read()).hexdigest()
            if actual!=expected:raise RuntimeError('Installed hash mismatch: '+target)
        logs=ROOT/'remote_logs';logs.mkdir(exist_ok=True)
        for name in ['configure.log','build.log','ctest.log','ned_dds.log','ned_air_dds.log','diff_check.log','job.log','test_update.log','clock_test.log']:
            s.get(REMOTE+'/'+name,str(logs/name))
        for name in ['last-good.json','clock-origin.json','host-sync.json']:
            s.get('/var/lib/vio-clock/'+name,str(logs/name))
        code=r'''
import glob,json,pathlib,subprocess
work=pathlib.Path('/home/nano/vio_repair_20261106')
assert (work/'exit_code').read_text().strip()=='0'
assert not glob.glob('/dev/ttyACM*') and not glob.glob('/dev/ttyUSB*'),'Controller attached; do not start live VIO for verification'
if (work/'service_was_active').read_text()=='True':
    subprocess.run(['systemctl','start','px4-vio.service'],check=True,timeout=55)
state=subprocess.run(['systemctl','is-active','px4-vio.service'],capture_output=True,text=True).stdout.strip()
report={'service_state':state,'date':subprocess.check_output(['date','-Iseconds'],text=True).strip(),
        'compiled_and_installed':True,'source_hashes_verified':True,
        'physical_flight_test':False,'ground_fault_injection':False,
        'controller_usb_serial_present':bool(glob.glob('/dev/ttyACM*')+glob.glob('/dev/ttyUSB*')),
        'backup':'/home/nano/vio_repair_20261106/backup'}
(work/'verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
'''
        stdin,out,err=c.exec_command('sudo -S -p "" /usr/bin/python3 -c '+shlex.quote(code),timeout=75)
        stdin.write(PW+'\n');stdin.flush();stdin.channel.shutdown_write()
        a=out.read().decode(errors='replace');b=err.read().decode(errors='replace');rc=out.channel.recv_exit_status()
        print(a,b)
        (ROOT/'finalize.log').write_text(a+'\n'+b,encoding='utf-8')
        if rc:raise SystemExit(rc)
        s.get(REMOTE+'/verification.json',str(ROOT/'verification.json'))
        _,out,err=c.exec_command('systemctl status px4-vio.service --no-pager; tail -8 /home/nano/vio_daemon.log',timeout=15)
        a=out.read().decode(errors='replace');b=err.read().decode(errors='replace')
        (logs/'service_final.log').write_text(a+'\n'+b,encoding='utf-8');print(a,b)
        s.close()
    finally:c.close()

if __name__=='__main__':main()
