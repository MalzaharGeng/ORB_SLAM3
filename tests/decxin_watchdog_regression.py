"""Test installed capture watchdog with mocked process commands and virtual time."""
from pathlib import Path
import subprocess,tempfile
def main():
    text=(Path(__file__).resolve().parents[1] / "nano/bin/vio_daemon.sh").read_text()
    fn=text[text.index("watch_stall() {"):text.index('\nsay "===',text.index("watch_stall() {"))]
    with tempfile.TemporaryDirectory() as d:
        root=Path(d)
        for mode in ["live","stalled"]:
            script="""
set -u
H=TASKDIR
SLAM_COMM=stereo_uvc
STALL_LIMIT=90
MODE=TASKMODE
ticks=0
say() { echo "$*"; }
ps() { echo 00:01; }
pkill() { echo "$*" >> "$H/signals"; }
touch() { :; }
sleep() {
 if [ "$1" = 15 ]; then
  ticks=$((ticks+1))
  if [ "$ticks" -gt 7 ]; then exit 77; fi
  if [ "$MODE" = live ]; then echo "[STAT] 10 fps" >> "$H/run.log"; fi
 fi
}
""".replace("TASKDIR",str(root)).replace("TASKMODE",mode)
            (root/"run.log").write_text("")
            (root/"signals").unlink(missing_ok=True)
            result=subprocess.run(["bash","-c",script+fn+'\nwatch_stall "$H/run.log"\n'],
                text=True,capture_output=True,timeout=5)
            signal=(root/"signals").read_text() if (root/"signals").exists() else ""
            if mode=="live":
                assert result.returncode==77 and not signal,(result,signal)
            else:
                assert result.returncode==0 and "-INT -x stereo_uvc" in signal and "-9 -x stereo_uvc" in signal,(result,signal)
        print("PASS UVC heartbeat survives >45s, stalled UVC gets targeted INT/KILL; no real signals sent")
if __name__=="__main__":main()
