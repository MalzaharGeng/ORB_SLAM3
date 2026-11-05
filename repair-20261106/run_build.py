"""Start a detached logged build; does not launch live VIO or flight commands."""
from pathlib import Path
import shlex
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nano_ssh import connect,PW
ROOT=Path(__file__).resolve().parent
REMOTE='/home/nano/vio_repair_20261106'
c=connect(PW)
try:
    s=c.open_sftp();s.put(str(ROOT/'build_and_test.sh'),REMOTE+'/build_and_test.sh');s.close()
    script='bash '+REMOTE+'/build_and_test.sh > '+REMOTE+'/job.log 2>&1; echo $? > '+REMOTE+'/exit_code'
    _,out,err=c.exec_command('nohup bash -c '+shlex.quote(script)+' </dev/null >/dev/null 2>&1 & echo $!',timeout=10)
    print('BUILD_PID',out.read().decode().strip(),err.read().decode())
finally:c.close()
