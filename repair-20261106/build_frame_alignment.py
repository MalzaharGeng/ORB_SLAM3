"""Make a reviewable frame-to-ULog index using stored clock origin and matched packets."""
import csv
import json
from pathlib import Path
import numpy as np
from pyulog import ULog

ROOT=Path(__file__).resolve().parent
meta=json.loads((ROOT/'historical_frame_metadata.json').read_text(encoding='utf-8'))
alignment=json.loads((ROOT/'flight_nano_alignment.json').read_text(encoding='utf-8'))['14_07_14.ulg']
origin=float(next(v for k,v in meta.items() if k.endswith('/event_1603810252_3003_6/clock_origin.txt')).splitlines()[0].split('=')[1])
with (ROOT/'14_07_14_message_matches.csv').open() as f:matches=list(csv.DictReader(f))
px=np.array([int(r['px4_us']) for r in matches],dtype=float)
delta=np.array([int(r['nano_msg_us'])-int(r['px4_us']) for r in matches],dtype=float)
order=np.argsort(px);px=px[order];delta=delta[order]
start=alignment['ulog_start_us'];median_delta=float(np.median(delta))
def px4time(nano):
    initial=nano-median_delta
    return nano-float(np.interp(initial,px,delta))
ulog=ULog(str(ROOT.parent/'14_07_14.ulg'),message_name_filter_list=['manual_control_setpoint','vehicle_status','vehicle_local_position'])
rc=ulog.get_dataset('manual_control_setpoint').data
pos=ulog.get_dataset('vehicle_local_position').data
def latest(d,time):return int(np.clip(np.searchsorted(d['timestamp'],time,side='right')-1,0,len(d['timestamp'])-1))
publish={}
telemetry=ROOT.parent/'2026.11.5/runs/19700101_080351_live_Tm7tjC/px4_telemetry.csv'
with telemetry.open(encoding='utf-8',errors='replace',newline='') as f:
    for r in csv.DictReader(f):
        if r.get('event')!='publish':continue
        try:publish[int(r['sample_us'])]=r
        except (ValueError,TypeError):pass
frames=[]
for kind,items in [('full_rate',[(k,v) for k,v in meta.items() if k.startswith('frame_')]),('overview',meta['overview_metadata'].items())]:
    for directory,text in items:
        frame_id,t,steady,imu=text.split(',');nano=(float(t)+origin)*1e6;native=px4time(nano)
        ri=latest(rc,native);pi=latest(pos,native)
        sample_key=round(nano)
        publication=next((publish[k] for k in [sample_key,sample_key-1,sample_key+1] if k in publish),None)
        row=dict(kind=kind,frame_directory=directory,frame_id=int(frame_id),image_seconds=float(t),
                 nano_sample_us=sample_key,ulog_seconds=(native-start)/1e6,frame_enqueue_steady_seconds=float(steady),imu_count=int(imu),
                 published_sample_match=publication is not None,
                 matching_publish_system_us=publication['system_us'] if publication else '',
                 rc_source_age_ms=(native-int(rc['timestamp'][ri]))/1000,
                 position_source_age_ms=(native-int(pos['timestamp'][pi]))/1000)
        row.update({k:float(rc[k][ri]) for k in ['roll','pitch','yaw','throttle']})
        row.update({k:float(pos[k][pi]) for k in ['x','y','z','vx','vy','vz','heading']})
        frames.append(row)
frames.sort(key=lambda r:r['ulog_seconds'])
with (ROOT/'last_flight_frame_alignment.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.DictWriter(f,fieldnames=list(frames[0]));w.writeheader();w.writerows(frames)
summary=dict(sensor_time_origin_system_seconds=origin,median_nano_minus_px4_seconds=median_delta/1e6,
    approximate_image_seconds_at_ulog_zero=start/1e6+median_delta/1e6-origin,
    full_rate={},overview={},key_events=[])
for kind in ['full_rate','overview']:
    a=[r for r in frames if r['kind']==kind]
    summary[kind]=dict(frames=len(a),ulog_start=a[0]['ulog_seconds'],ulog_end=a[-1]['ulog_seconds'],
                        published_sample_matches=sum(r['published_sample_match'] for r in a))
for target in [16.48,17.48,21.88,22.68,23.08,30,38,41,43]:
    row=min(frames,key=lambda r:abs(r['ulog_seconds']-target))
    summary['key_events'].append(dict(requested_ulog_seconds=target,**{k:row[k] for k in ['kind','frame_directory','image_seconds','ulog_seconds']}))
(ROOT/'last_flight_alignment_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2))
