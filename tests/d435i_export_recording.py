#!/usr/bin/env python3
"""Export RealSense 2.58 ROS2 DB3 recordings using original sensor metadata times.
Run after sourcing /opt/ros/humble/setup.bash. Never use bag arrival time as IMU time.
"""
import csv
import json
import sqlite3
import sys
from pathlib import Path
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

source, target = Path(sys.argv[1]), Path(sys.argv[2])
target.mkdir(parents=True, exist_ok=True)
db = sqlite3.connect(f'file:{source}?mode=ro', uri=True)
assert db.execute('PRAGMA quick_check').fetchone()[0] == 'ok'
topics = {name: tid for tid, name in db.execute('select id,name from topics')}

def metadata(prefix, suffix):
    tid = topics[prefix + '/' + suffix + '/metadata']
    result = {}
    for t, blob in db.execute('select timestamp,data from messages where topic_id=? order by id',(tid,)):
        text = deserialize_message(blob, String).data
        result[t] = dict(item.split('=', 1) for item in text.split(';') if '=' in item)
    return result

image_streams=[]
for side, index in [('left',1),('right',2)]:
    (target/side).mkdir(exist_ok=True)
    prefix=f'/device_0/sensor_0/Infrared_{index}'
    meta=metadata(prefix,'image'); rows=[]
    for t, blob in db.execute('select timestamp,data from messages where topic_id=? order by id',(topics[prefix+'/image/data'],)):
        m=deserialize_message(blob, Image); md=meta[t]
        number=int(md['Frame number']); name=f'{side}/{number}.png'
        array=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width]
        assert cv2.imwrite(str(target/name),array,[cv2.IMWRITE_PNG_COMPRESSION,1])
        rows.append((number,float(md['timestamp'])*0.001,name,md['timestamp_domain']))
    image_streams.append(rows)
    print(side,len(rows),flush=True)
right={r[0]:r for r in image_streams[1]}
with (target/'stereo.csv').open('w') as f:
    out=csv.writer(f);out.writerow(['timestamp','left','right','left_frame','right_frame','skew_ms','domain'])
    for n,t,name,domain in image_streams[0]:
        r=right[n];out.writerow([format(t,'.17g'),name,r[2],n,n,1000*(t-r[1]),domain])
counts={'stereo':len(image_streams[0])}
for stream,field in [('Gyro','angular_velocity'),('Accel','linear_acceleration')]:
    prefix=f'/device_0/sensor_2/{stream}_0';meta=metadata(prefix,'imu');count=0
    with (target/(stream.lower()+'.csv')).open('w') as f:
        out=csv.writer(f);out.writerow(['timestamp','x','y','z','domain'])
        for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by id',(topics[prefix+'/imu/data'],)):
            m=deserialize_message(blob,Imu);md=meta[t];v=getattr(m,field)
            out.writerow([format(float(md['timestamp'])*.001,'.17g'),v.x,v.y,v.z,md['timestamp_domain']]);count+=1
    counts[stream.lower()]=count
(target/'export_summary.json').write_text(json.dumps(counts,indent=2)+'\n')
print('EXPORT_OK',counts,flush=True)
