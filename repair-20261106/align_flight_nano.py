"""Align PX4 native ULogs to Nano recordings by identical message payloads."""
import csv
import json
from pathlib import Path
import numpy as np
from pyulog import ULog

ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent
ulog_names=['13_47_11.ulg','14_07_14.ulg']
logs=[ULog(str(PROJECT/name),message_name_filter_list=['vehicle_attitude','manual_control_setpoint','vehicle_status']) for name in ulog_names]
topics={'attitude':('vehicle_attitude',[f'q[{i}]' for i in range(4)]),
        'manual_control':('manual_control_setpoint',['roll','pitch','yaw','throttle'])}
indexes={}
for name,u in zip(ulog_names,logs):
    for src,(topic,keys) in topics.items():
        d=u.get_dataset(topic).data
        indexes[(name,src)]={}
        for i,stamp in enumerate(d['timestamp']):
            sig=tuple(round(float(d[k][i]),7) for k in keys)
            indexes[(name,src)].setdefault(sig,[]).append(int(stamp))

matches={(name,src):[] for name in ulog_names for src in topics}
statuses=[]
path=next((PROJECT/'2026.11.5/inrec').rglob('*Tm7tjC.csv'))
bad=0
with path.open(encoding='utf-8',errors='replace',newline='') as f:
    for row in csv.DictReader(f):
        src=row.get('src')
        if src not in topics and src!='status':continue
        try:
            p=json.loads(row['payload_json'])
            received=float(row['wall_t']);steady=float(row['steady_t']);up=float(row['uptime_s'])
            stamp=int(row['t_msg_us'])
        except (ValueError,TypeError,KeyError,json.JSONDecodeError):bad+=1;continue
        if src=='status':
            statuses.append({'received':received,'stamp':stamp,'steady':steady,'uptime':up,'armed':int(row['armed']),'nav':int(row['nav_state'])})
            continue
        sig=tuple(round(float(x),7) for x in p['q']) if src=='attitude' else tuple(round(float(p[k]),7) for k in topics[src][1])
        for name,u in zip(ulog_names,logs):
            choices=indexes[(name,src)].get(sig,[])
            # Timestamp narrows duplicate neutral-stick payloads to the same flight.
            choices=[t for t in choices if abs(stamp-t)<2000000]
            if not choices:continue
            if src=='manual_control':
                attitude_matches=matches[(name,'attitude')]
                if len(attitude_matches)<10:continue
                delta=float(np.median([r['nano_msg_us']-r['px4_us'] for r in attitude_matches[-100:]]))
                choices=[t for t in choices if abs(stamp-t-delta)<2000]
                if len(choices)!=1:continue
            exact=min(choices,key=lambda t:abs(stamp-t))
            matches[(name,src)].append(dict(px4_us=exact,nano_msg_us=stamp,receive_us=round(received*1e6),
                                          steady=steady,uptime=up,unique_payload=len(choices)==1))

def stats(values):
    a=np.array(values,dtype=float)
    return {'count':len(a),'min':float(np.min(a)),'p50':float(np.median(a)),
            'p95':float(np.percentile(a,95)),'max':float(np.max(a))}
summary={}
for name,u in zip(ulog_names,logs):
    result={'ulog_start_us':u.start_timestamp,'topics':{}}
    for src in topics:
        rows=matches[(name,src)]
        if not rows:continue
        result['topics'][src]={
            'matched_rows':len(rows),'distinct_px4_messages':len(set(r['px4_us'] for r in rows)),
            'nano_msg_minus_px4_ms':stats([(r['nano_msg_us']-r['px4_us'])/1000 for r in rows]),
            'nano_receive_minus_px4_ms':stats([(r['receive_us']-r['px4_us'])/1000 for r in rows]),
            'nano_receive_minus_msg_ms':stats([(r['receive_us']-r['nano_msg_us'])/1000 for r in rows]),
            'nano_uptime_at_ulog_zero_seconds':stats([r['uptime']-(r['px4_us']-u.start_timestamp)/1e6 for r in rows]),
            'nano_steady_at_ulog_zero_seconds':stats([r['steady']-(r['px4_us']-u.start_timestamp)/1e6 for r in rows]),
        }
    within=[r for r in statuses if u.start_timestamp-2000000<r['stamp']<u.start_timestamp+int(1e6*(50 if name.startswith('14') else 290))]
    result['status_edges']=[]
    clock_delta=np.median([r['nano_msg_us']-r['px4_us'] for r in matches[(name,'attitude')]])
    for i,r in enumerate(within):
        if i==0 or (r['armed'],r['nav'])!=(within[i-1]['armed'],within[i-1]['nav']):
            result['status_edges'].append({**r,'ulog_seconds_after_clock_conversion':(r['stamp']-clock_delta-u.start_timestamp)/1e6})
    summary[name]=result
    rows=matches[(name,'attitude')]
    with (ROOT/(name.replace('.ulg','')+'_message_matches.csv')).open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
(ROOT/'flight_nano_alignment.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2))
