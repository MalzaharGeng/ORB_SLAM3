"""Fetch only historical recording clock/frame metadata, never image payloads."""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nano_ssh import connect,PW
ROOT=Path(__file__).resolve().parent
c=connect(PW)
try:
    s=c.open_sftp()
    prior=ROOT/'historical_frame_metadata.json'
    result=json.loads(prior.read_text(encoding='utf-8')) if prior.exists() else {}
    run='/home/nano/ORB_SLAM3/recordings/test_runs/19700101_080351_live_Tm7tjC'
    for path in [run+'/image_ring/clock_origin.txt',run+'/image_ring/camera_calibration.json']:
        try:
            with s.open(path,'r') as f:result[path]=f.read().decode()
        except OSError as e:result[path]={'unavailable':str(e)}
    event='/home/nano/vio_image_events/event_1603810252_3003_6'
    entries=s.listdir(event)
    frames=sorted(x for x in entries if x.startswith('frame_'))
    result['event_entries']=[x for x in entries if not x.startswith('frame_')]
    for name in ['clock_origin.txt','rolling_event.txt','format.txt']:
        with s.open(event+'/'+name,'r') as f:result[event+'/'+name]=f.read().decode()
    for frame in frames:
        if frame in result:continue
        with s.open(event+'/'+frame+'/frame.csv','r') as f:result[frame]=f.read().decode().strip()
    overview=event+'/overview'
    result.setdefault('overview_metadata',{})
    for frame in sorted(x for x in s.listdir(overview) if x.startswith('frame_')):
        if frame in result['overview_metadata']:continue
        with s.open(overview+'/'+frame+'/frame.csv','r') as f:result['overview_metadata'][frame]=f.read().decode().strip()
    s.close()
    (ROOT/'historical_frame_metadata.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if not k.startswith('frame_') and k!='overview_metadata'},indent=2))
    print('FRAME_METADATA_COUNT',len(frames),'FIRST',result[frames[0]],'LAST',result[frames[-1]])
    print('OVERVIEW_FRAME_COUNT',len(result['overview_metadata']))
finally:c.close()
