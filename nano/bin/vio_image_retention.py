#!/usr/bin/env python3
"""Bound old image rings across sessions. Only explicitly marked recordings qualify."""
import pathlib,shutil,os,time

def preserve_interrupted(base,events):
    events=pathlib.Path(events);events.mkdir(parents=True,exist_ok=True)
    snapshots=[]
    for marker in pathlib.Path(base).glob('*/image_ring/format.txt'):
        ring=marker.parent
        if ((ring/'closed.txt').exists() and not (ring/'error.txt').exists()) or (ring/'recovered_event.txt').exists():continue
        if marker.read_text().strip()!='ORB_SLAM3_ROLLING_INPUT_V1':continue
        frames=sorted(p for p in ring.glob('frame_*') if p.is_dir() and not p.name.endswith('.partial'))
        if not frames:continue
        target=events/('event_unclean_'+str(time.time_ns()))
        target.mkdir()
        for name in ('format.txt','settings.yaml'):shutil.copy2(ring/name,target/name)
        for name in ('clock_origin.txt','camera_calibration.json'):
            if (ring/name).is_file():shutil.copy2(ring/name,target/name)
        if (ring/'overview').is_dir():
            def link_or_copy(source,dest):
                try:os.link(source,dest)
                except OSError:shutil.copy2(source,dest)
                return dest
            shutil.copytree(ring/'overview',target/'overview',copy_function=link_or_copy)
            for name in ('clock_origin.txt','camera_calibration.json'):
                if (ring/name).is_file():shutil.copy2(ring/name,target/'overview'/name)
        for frame in frames:
            dest=target/frame.name;dest.mkdir()
            for item in frame.iterdir():
                try:os.link(item,dest/item.name)
                except OSError:shutil.copy2(item,dest/item.name)
        reason='recording_failure' if (ring/'error.txt').exists() else 'unclean_exit'
        (target/'rolling_event.txt').write_text('reason='+reason+'\nsource='+str(ring)+'\n')
        (ring/'recovered_event.txt').write_text(str(target)+'\n');snapshots.append(str(target))
    return snapshots

def limit_events(events,max_count=24,max_bytes=4*1024**3):
    root=pathlib.Path(events).resolve()
    own=[p for p in root.iterdir() if p.is_dir() and not p.is_symlink() and (p/'rolling_event.txt').is_file()]
    own.sort(key=lambda p:p.stat().st_mtime)
    sizes={p:sum(f.stat().st_size for f in p.rglob('*') if f.is_file()) for p in own}
    total=sum(sizes.values())
    while own and (len(own)>max_count or total>max_bytes):
        p=own.pop(0)
        if p.resolve().parent!=root:raise ValueError('Unexpected event path')
        total-=sizes[p];shutil.rmtree(p)

def prune(base,keep=3):
    base=pathlib.Path(base).resolve()
    owned=[]
    for marker in base.glob('*/image_ring/format.txt'):
        try:
            if marker.read_text().strip()=='ORB_SLAM3_ROLLING_INPUT_V1':
                owned.append(marker.parent)
        except OSError:continue
    owned.sort(key=lambda p:p.stat().st_mtime,reverse=True)
    removed=[]
    for path in owned[keep:]:
        resolved=path.resolve()
        if resolved.parent.parent!=base or resolved.name!='image_ring' or path.is_symlink():
            raise ValueError('Unexpected recording path')
        shutil.rmtree(resolved);removed.append(str(resolved))
    return removed

if __name__=='__main__':
    base=pathlib.Path.home()/'ORB_SLAM3/recordings/test_runs'
    events=pathlib.Path.home()/'vio_image_events'
    print('INTERRUPTED_IMAGE_RINGS_PRESERVED',preserve_interrupted(base,events))
    limit_events(events)
    print('OLD_IMAGE_RINGS_REMOVED',prune(base))
