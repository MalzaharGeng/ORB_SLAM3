#!/usr/bin/env python3
import pathlib,tempfile,importlib.util,json,shutil
from unittest.mock import patch
def module(name):
    spec=importlib.util.spec_from_file_location(name,pathlib.Path(__file__).with_name(name+'.py'))
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
exporter=module('export_vio_image_ring');retention=module('vio_image_retention')
with tempfile.TemporaryDirectory() as d:
    root=pathlib.Path(d);base=root/'runs';events=root/'events';base.mkdir();events.mkdir()
    for i in range(5):
        ring=base/str(i)/'image_ring';ring.mkdir(parents=True)
        (ring/'format.txt').write_text('ORB_SLAM3_ROLLING_INPUT_V1\n');(ring/'settings.yaml').write_text('%YAML:1.0\n')
        frame=ring/'frame_000000000001';frame.mkdir()
        (frame/'frame.csv').write_text('1,10.5,100.0,1\n');(frame/'imu.csv').write_text('10.5,1,2,3,4,5,6\n')
        for side in ('left','right'):(frame/(side+'.png')).write_bytes(b'test-image-payload')
    ring=base/'4/image_ring'
    (ring/'clock_origin.txt').write_text('origin=123\n')
    (ring/'camera_calibration.json').write_text('{"serial":"test"}')
    overview=ring/'overview';shutil.copytree(ring/'frame_000000000001',overview/'frame_000000000001')
    for name in ('format.txt','settings.yaml'):shutil.copy2(ring/name,overview/name)
    (overview/'overview.txt').write_text('Sparse context\n')
    shutil.copytree(overview/'frame_000000000001',overview/'frame_000000000016')
    (overview/'frame_000000000016/frame.csv').write_text('16,11.5,101.0,1\n')
    context=exporter.export(overview,root/'overview_export')
    assert context['sparse_context'] and not context['queue_gaps'] and context['frames']==2
    assert (root/'overview_export/camera_calibration.json').is_file()
    (ring/'frame_000000000002.partial').mkdir()
    meta=exporter.export(ring,root/'export')
    assert meta['frames']==1 and not meta['queue_gaps']
    assert (root/'export/imu.csv').read_text().splitlines()[1]=='0,10.5,1,2,3,4,5,6'
    snapshots=retention.preserve_interrupted(base,events)
    assert len(snapshots)==5 and len(retention.preserve_interrupted(base,events))==0
    restored_context=next(pathlib.Path(p) for p in snapshots if (pathlib.Path(p)/'overview').exists())
    assert (restored_context/'overview/camera_calibration.json').is_file()
    unrelated=base/'unrelated/image_ring';unrelated.mkdir(parents=True);(unrelated/'precious.txt').write_text('keep')
    removed=retention.prune(base,3);assert len(removed)==2 and (unrelated/'precious.txt').exists()
    retention.limit_events(events,max_count=2)
    assert len(list(events.glob('*/rolling_event.txt')))==2
    assert all(pathlib.Path(s).parent==events for s in snapshots)
    failed=base/'failed/image_ring'
    shutil.copytree(base/'4/image_ring',failed)
    (failed/'recovered_event.txt').unlink(missing_ok=True)
    (failed/'closed.txt').write_text('error=disk reserve\n')
    (failed/'error.txt').write_text('disk reserve\n')
    restored=retention.preserve_interrupted(base,events)
    assert len(restored)==1 and 'recording_failure' in (pathlib.Path(restored[0])/'rolling_event.txt').read_text()
print('VIO_IMAGE_TOOLS_REGRESSION_PASS')
