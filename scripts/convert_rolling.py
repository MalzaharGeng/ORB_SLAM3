#!/usr/bin/env python3
"""把 RollingInputRecording 的 event 目录转成 replay 能吃的标准 recording 格式。"""
import os
import sys
import glob
import shutil

E = sys.argv[1] if len(sys.argv) > 1 else "/home/nano/vio_image_events/event_1603810252_3003_6"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/replay_conv_6"

os.makedirs(OUT, exist_ok=True)

frames = sorted(d for d in glob.glob(os.path.join(E, "frame_*")) if os.path.isdir(d))
if not frames:
    raise SystemExit("没有 frame 目录")

stereo = ["index,timestamp,left,right,imu_count"]
imu = ["frame,timestamp,ax,ay,az,gx,gy,gz"]

n_imu = 0
seq = 0          # replay 要求 index 从 0 连续递增
for f in frames:
    name = os.path.basename(f)
    with open(os.path.join(f, "frame.csv")) as fh:
        parts = fh.read().strip().split(",")
    if len(parts) < 4:
        continue
    # frame.csv 四列: id, timestamp(图像), steady_clock, imu_count
    _orig, ts, _steady, cnt = parts[0], parts[1], parts[2], parts[3]
    idx = seq
    seq += 1

    stereo.append("%s,%s,%s/left.png,%s/right.png,%s" % (idx, ts, name, name, cnt))

    p = os.path.join(f, "imu.csv")
    if os.path.exists(p):
        with open(p) as fh:
            for ln in fh:
                v = ln.strip().split(",")
                if len(v) != 7:
                    continue
                # 源码 RollingInputRecording.cc:159 写入顺序为
                #   m.t, a.x, a.y, a.z, w.x, w.y, w.z
                # 即 原始: ts,ax,ay,az,gx,gy,gz —— 与目标列顺序完全一致，只需补 frame 列
                imu.append("%s,%s,%s,%s,%s,%s,%s,%s" % (idx, v[0], v[1], v[2], v[3], v[4], v[5], v[6]))
                n_imu += 1

with open(os.path.join(OUT, "stereo.csv"), "w") as fh:
    fh.write("\n".join(stereo) + "\n")
with open(os.path.join(OUT, "imu.csv"), "w") as fh:
    fh.write("\n".join(imu) + "\n")
with open(os.path.join(OUT, "events.csv"), "w") as fh:
    fh.write("frame,action\n")
with open(os.path.join(OUT, "complete.txt"), "w") as fh:
    fh.write(str(len(frames)) + "\n")
with open(os.path.join(OUT, "format.txt"), "w") as fh:
    fh.write("ORB_SLAM3_STEREO_INPUT_V1\n")

for extra in ("settings.yaml", "camera_calibration.json", "clock_origin.txt"):
    src = os.path.join(E, extra)
    if os.path.exists(src):
        shutil.copy(src, os.path.join(OUT, extra))

# 图像用软链接，避免复制 100+ MB
for f in frames:
    dst = os.path.join(OUT, os.path.basename(f))
    if os.path.islink(dst) or os.path.exists(dst):
        continue
    os.symlink(f, dst)

print("frames=%d  imu=%d  out=%s" % (len(frames), n_imu, OUT))
print("first=%s  last=%s" % (os.path.basename(frames[0]), os.path.basename(frames[-1])))
