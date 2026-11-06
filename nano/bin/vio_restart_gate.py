#!/usr/bin/env python3
"""Only fresh disarmed, landed telemetry permits starting a new VIO process."""
import json,sys,time
from pathlib import Path
def reason(summary,steady,wall):
    if summary.get("closed_cleanly") or summary.get("error"):return "recorder_unavailable"
    v=summary.get("vehicle",{})
    if type(v.get("armed")) is not int or v["armed"]!=0:return "waiting_disarmed"
    if v.get("landed") is not True:return "waiting_landed"
    for stream,limit in (("status",2.),("land",2.5)):
        received=v.get(stream+"_receipt_steady");stamp=v.get(stream+"_timestamp_us")
        if not isinstance(received,(float,int)) or not 0<=steady-received<=limit:return "stale_"+stream
        if not isinstance(stamp,int) or not -.05<=wall-stamp*1e-6<=limit:return "stale_"+stream+"_sample"
    return ""
def main():
    try:r=reason(json.loads(Path(sys.argv[1]).read_text()),time.monotonic(),time.time())
    except (OSError,ValueError,KeyError,IndexError,TypeError):r="restart_evidence_unavailable"
    print("VIO_START_GATE "+(r or "disarmed_landed"))
    return 1 if r else 0
if __name__=="__main__":raise SystemExit(main())
