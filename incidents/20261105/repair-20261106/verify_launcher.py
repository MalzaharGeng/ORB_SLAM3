from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nano_ssh import connect,PW
c=connect(PW)
try:
    command='''tail -4 /home/nano/vio_daemon.log
grep -n -E 'stereo_inertial|BIN=|BINARY=' /home/nano/vio_daemon.sh | head -10
find /home/nano/vio_repair_20261106/backup/build -type f -printf '%P %s\\n'
find /home/nano/ORB_SLAM3/build -type f -name stereo_inertial_realsense_D435i -printf '%P %s\\n'
'''
    _,out,err=c.exec_command(command,timeout=15)
    a=out.read().decode(errors='replace');b=err.read().decode(errors='replace')
    print(a,b)
    (Path(__file__).resolve().parent/'remote_logs/launcher_verified.log').write_text(a+'\n'+b,encoding='utf-8')
finally:c.close()
