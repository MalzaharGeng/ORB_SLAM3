#!/usr/bin/env python3
"""Export committed rolling inputs to the existing lossless replay format."""
import argparse,csv,json,math,os,pathlib,shutil

def committed_frames(source):
    if (source/'format.txt').read_text().strip()!='ORB_SLAM3_ROLLING_INPUT_V1':
        raise ValueError('Unsupported ring format')
    result=[]
    for path in source.glob('frame_*'):
        if not path.is_dir() or path.name.endswith('.partial'):
            continue
        try:
            with (path/'frame.csv').open(newline='') as f:row=next(csv.reader(f))
            index,stamp,steady,count=int(row[0]),float(row[1]),float(row[2]),int(row[3])
            if not math.isfinite(stamp) or not math.isfinite(steady):continue
            if not all((path/name).is_file() for name in ('left.png','right.png','imu.csv')):continue
            imu=list(csv.reader((path/'imu.csv').open(newline='')))
            if len(imu)!=count or any(len(r)!=7 for r in imu):continue
            result.append((index,stamp,steady,path,imu))
        except (OSError,ValueError,StopIteration,IndexError):continue
    result.sort()
    return result

def export(source,destination):
    frames=committed_frames(source)
    if not frames:raise ValueError('No committed stereo/IMU frames')
    destination.mkdir(parents=True,exist_ok=False)
    (destination/'left').mkdir();(destination/'right').mkdir()
    shutil.copy2(source/'settings.yaml',destination/'settings.yaml')
    for name in ('clock_origin.txt','camera_calibration.json','overview.txt'):
        metadata_source=source/name
        if not metadata_source.is_file() and source.name=='overview':metadata_source=source.parent/name
        if metadata_source.is_file():shutil.copy2(metadata_source,destination/name)
    (destination/'format.txt').write_text('ORB_SLAM3_STEREO_INPUT_V1\n')
    (destination/'events.csv').write_text('frame,action\n')
    with (destination/'stereo.csv').open('w',newline='') as f,(destination/'imu.csv').open('w',newline='') as imu:
        w=csv.writer(f);v=csv.writer(imu)
        w.writerow(['index','timestamp','left','right','imu_count']);v.writerow(['frame','timestamp','ax','ay','az','gx','gy','gz'])
        for i,(_,stamp,_,path,batch) in enumerate(frames):
            for side in ('left','right'):
                target=destination/side/(str(i)+'.png')
                try:os.link(path/(side+'.png'),target)
                except OSError:shutil.copy2(path/(side+'.png'),target)
            for row in batch:v.writerow([i]+row)
            w.writerow([i,repr(stamp),'left/'+str(i)+'.png','right/'+str(i)+'.png',len(batch)])
    (destination/'complete.txt').write_text(str(len(frames))+'\n')
    sparse=(source/'overview.txt').is_file()
    gaps=[] if sparse else [{'previous':a[0],'next':b[0]} for a,b in zip(frames,frames[1:]) if b[0]!=a[0]+1]
    metadata={'source':str(source.resolve()),'frames':len(frames),'steady_span_seconds':frames[-1][2]-frames[0][2],
              'sparse_context':sparse,'queue_gaps':gaps,'warning':('Sparse context has intentionally discontinuous images/IMU; use for diagnosis, not continuous inertial replay.' if sparse else 'Last 20 seconds alone may not initialize inertial SLAM. A matching map/state or excitation is required for full estimator replay.')}
    (destination/'export_metadata.json').write_text(json.dumps(metadata,indent=2))
    return metadata

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=pathlib.Path);p.add_argument('destination',type=pathlib.Path)
    a=p.parse_args();print(json.dumps(export(a.source,a.destination),indent=2))
if __name__=='__main__':main()
