#!/usr/bin/env python3
"""把多个 RollingInputRecording event 目录按时间拼成一份 replay 输入。

- 帧按图像时间戳排序，重叠区（< 半帧周期）只保留一帧；
- 被丢弃帧携带的 IMU 样本追加给前一个保留帧（时间戳去重），避免 IMU 空洞；
- 每个保留帧的 IMU 批只保留时间戳严格大于前一帧末样本的部分，保证全局单调；
- index 从 0 连续重排（replay 要求）。
用法: replay_splice_rolling.py OUT_DIR EVENT_DIR [EVENT_DIR ...]
"""
import os
import sys
import glob
import shutil

OUT = sys.argv[1]
EVENTS = sys.argv[2:]
if not EVENTS:
    raise SystemExit("need at least one event dir")

os.makedirs(OUT, exist_ok=True)

frames = []  # (ts, dir, imu_list[(t,ax,ay,az,gx,gy,gz)])
for e in EVENTS:
    for d in sorted(glob.glob(os.path.join(e, "frame_*"))):
        if not os.path.isdir(d):
            continue
        fp = os.path.join(d, "frame.csv")
        if not os.path.exists(fp):
            continue
        parts = open(fp).read().strip().split(",")
        if len(parts) < 4:
            continue
        ts = float(parts[1])
        imu = []
        ip = os.path.join(d, "imu.csv")
        if os.path.exists(ip):
            for ln in open(ip):
                v = ln.strip().split(",")
                if len(v) != 7:
                    continue
                try:
                    imu.append((float(v[0]), float(v[1]), float(v[2]), float(v[3]),
                                float(v[4]), float(v[5]), float(v[6])))
                except ValueError:
                    continue
        frames.append((ts, d, imu))

frames.sort(key=lambda x: x[0])
if not frames:
    raise SystemExit("no frames")

dts = [frames[i + 1][0] - frames[i][0] for i in range(len(frames) - 1)]
dts = sorted(d for d in dts if d > 1e-4)
frame_dt = dts[len(dts) // 2] if dts else 0.0667
dup_gap = 0.5 * frame_dt

kept = []          # [ts, dir, imu_list]
dropped = 0
for ts, d, imu in frames:
    if kept and (ts - kept[-1][0]) < dup_gap:
        # 重复帧：把它的新 IMU 补到上一保留帧
        prev = kept[-1]
        last_t = prev[2][-1][0] if prev[2] else -1e18
        for m in imu:
            if m[0] > last_t:
                prev[2].append(m)
                last_t = m[0]
        dropped += 1
        continue
    kept.append([ts, d, list(imu)])

stereo = ["index,timestamp,left,right,imu_count"]
imulines = ["frame,timestamp,ax,ay,az,gx,gy,gz"]
n_imu = 0
prev_last = -1e18
for idx, (ts, d, imu) in enumerate(kept):
    name = os.path.basename(d)
    batch = [m for m in imu if m[0] > prev_last]
    if batch:
        prev_last = batch[-1][0]
    stereo.append("%d,%.17g,%s/left.png,%s/right.png,%d" % (idx, ts, name, name, len(batch)))
    for m in batch:
        imulines.append("%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g" % ((idx,) + m))
        n_imu += 1
    dst = os.path.join(OUT, name)
    if not (os.path.islink(dst) or os.path.exists(dst)):
        os.symlink(d, dst)

open(os.path.join(OUT, "stereo.csv"), "w").write("\n".join(stereo) + "\n")
open(os.path.join(OUT, "imu.csv"), "w").write("\n".join(imulines) + "\n")
open(os.path.join(OUT, "events.csv"), "w").write("frame,action\n")
open(os.path.join(OUT, "complete.txt"), "w").write(str(len(kept)) + "\n")
open(os.path.join(OUT, "format.txt"), "w").write("ORB_SLAM3_STEREO_INPUT_V1\n")
for extra in ("settings.yaml", "camera_calibration.json", "clock_origin.txt"):
    src = os.path.join(EVENTS[0], extra)
    if os.path.exists(src):
        shutil.copy(src, os.path.join(OUT, extra))

span = kept[-1][0] - kept[0][0]
print("events=%d frames_in=%d kept=%d dropped_dup=%d imu=%d span=%.3fs out=%s"
      % (len(EVENTS), len(frames), len(kept), dropped, n_imu, span, OUT))
print("first=%.6f last=%.6f" % (kept[0][0], kept[-1][0]))
