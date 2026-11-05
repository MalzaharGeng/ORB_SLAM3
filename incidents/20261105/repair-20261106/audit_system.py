"""Read-only system inventory for a targeted VIO simplification proposal."""
import json
from pathlib import Path
import shlex
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nano_ssh import connect,PW

CODE=r'''
import json,pathlib,subprocess,datetime
def run(args):
    p=subprocess.run(args,capture_output=True,text=True,timeout=12)
    return {'rc':p.returncode,'stdout':p.stdout,'stderr':p.stderr}
r={'time':datetime.datetime.now(datetime.timezone.utc).isoformat()}
for name,args in {
 'running_services':['systemctl','list-units','--type=service','--state=running','--no-pager','--no-legend'],
 'enabled_services':['systemctl','list-unit-files','--type=service','--state=enabled','--no-pager','--no-legend'],
 'memory':['free','-m'],
 'processes':['ps','-eo','pid,comm,pcpu,pmem,rss','--sort=-rss'],
 'disk':['df','-h','/','/home/nano'],
 'power_mode':['/usr/sbin/nvpmodel','-q'],
 'vio_service':['systemctl','show','px4-vio.service','-p','ActiveState','-p','SubState','-p','MemoryCurrent','-p','CPUUsageNSec'],
 'display_target':['systemctl','get-default'],
 'swap':['cat','/proc/swaps'],
}.items():
    try:r[name]=run(args)
    except (OSError,subprocess.TimeoutExpired) as e:r[name]={'error':str(e)}
r['cpu_governors']={str(p):p.read_text().strip() for p in pathlib.Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_governor')}
r['cpu_max_freq']={str(p):p.read_text().strip() for p in pathlib.Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_max_freq')}
print(json.dumps(r,indent=2))
'''

def main():
    c=connect(PW)
    try:
        _,out,err=c.exec_command('python3 -c '+shlex.quote(CODE),timeout=100)
        a=out.read().decode(errors='replace');b=err.read().decode(errors='replace')
        if out.channel.recv_exit_status():raise RuntimeError(b)
        result=json.loads(a)
        (Path(__file__).resolve().parent/'system_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        for key in ['memory','processes','running_services','disk','power_mode','display_target','swap','cpu_governors','cpu_max_freq']:
            value=result[key]
            if key=='processes':value={'stdout':'\n'.join(value['stdout'].splitlines()[:22])}
            print(key,json.dumps(value,ensure_ascii=False))
    finally:c.close()

if __name__=='__main__':main()
