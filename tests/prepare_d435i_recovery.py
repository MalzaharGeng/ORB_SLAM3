#!/usr/bin/env python3
"""Build a portable replay timeline from the two verified D435i captures.
Raw sensor timestamps in the source DB3/exports are left unchanged.
"""
import csv
import json
import os
import sys
from pathlib import Path
base=Path(sys.argv[1]).resolve()
out=base/'combined';out.mkdir(exist_ok=True)
parts=[(base/'export',base/'slam_events.csv','first_saved'),
       (base/'second_capture/export',base/'second_capture/slam_events.csv','second_saved')]
streams={s:[] for s in ['stereo','gyro','accel']}
shift=1.0;events=[]
for index,(directory,event_file,event_name) in enumerate(parts):
    images=list(csv.DictReader((directory/'stereo.csv').open()))
    start=float(images[0]['timestamp'])
    event_rows=list(csv.DictReader(event_file.open()))
    saved=float(next(r['sensor_time'] for r in event_rows if r['event']==event_name))
    end=saved if index==0 else float(images[-1]['timestamp'])
    for stream in streams:
        for row in csv.DictReader((directory/(stream+'.csv')).open()):
            t=float(row['timestamp'])
            if t<start-0.1 or t>end:continue
            row['timestamp']=format(t-start+shift,'.17g')
            if stream=='stereo':
                for side in ['left','right']:
                    row[side]=os.path.relpath(directory/row[side],out)
            streams[stream].append(row)
    events.append(dict(part=index+1,start=shift,saved_at=saved-start+shift,end=end-start+shift))
    shift+=end-start+1.0
for stream,rows in streams.items():
    with (out/(stream+'.csv')).open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
(out/'events.json').write_text(json.dumps(events,indent=2)+'\n')
(out/'reset_at.txt').write_text(str(events[1]['start']-events[0]['start']-0.001)+'\n')
print('REPLAY_PREPARED',events)
