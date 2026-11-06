#!/usr/bin/env python3
"""Bounded raw-MJPEG + raw-IMU calibration capture; no control publishers."""
import argparse,csv,json,math,os,queue,signal,threading,time,traceback
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import SensorCombined,VehicleStatus,TimesyncStatus
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--out",required=True)
    ap.add_argument("--seconds",type=float,default=100)
    ap.add_argument("--hz",type=float,default=10)
    args=ap.parse_args()
    if not 1<=args.seconds<=600 or not 1<=args.hz<=15:ap.error("invalid duration/rate")
    out=Path(args.out);out.mkdir(exist_ok=False,parents=True);(out/"frames").mkdir()
    cv2.setNumThreads(1)
    done=threading.Event();errors=[];status={"received":False,"armed":True,"last":0.}
    counts={"imu":0,"frames":0,"bad":0,"imu_invalid":0};intervals=[];previous_imu=None
    rclpy.init();node=Node("decxin_calibration_capture")
    imu_file=(out/"imu.csv").open("w",buffering=65536);iw=csv.writer(imu_file)
    iw.writerow(["timestamp_us","receive_system_us","receive_steady_s","ax","ay","az","gx","gy","gz","accel_relative_us","accel_dt_us","gyro_dt_us","clip","accel_cal","gyro_cal"])
    sf=(out/"sync.csv").open("w",buffering=65536);sw=csv.writer(sf);sw.writerow(["receive_system_us","timestamp","rtt_us","observed_offset","estimated_offset"])
    def on_imu(m):
        nonlocal previous_imu
        received=time.time_ns()//1000
        clip=int(m.accelerometer_clipping)|int(m.gyro_clipping)
        if clip or m.accelerometer_timestamp_relative==2147483647:counts["imu_invalid"]+=1
        iw.writerow([int(m.timestamp),received,time.monotonic(),*[float(x) for x in m.accelerometer_m_s2],*[float(x) for x in m.gyro_rad],int(m.accelerometer_timestamp_relative),int(m.accelerometer_integral_dt),int(m.gyro_integral_dt),clip,int(m.accel_calibration_count),int(m.gyro_calibration_count)])
        if previous_imu is not None:intervals.append((int(m.timestamp)-previous_imu)*1e-6)
        previous_imu=int(m.timestamp);counts["imu"]+=1
    def on_status(m):status.update(received=True,armed=int(m.arming_state)==2,last=time.monotonic())
    def on_sync(m):
        if m.source_protocol==2:sw.writerow([time.time_ns()//1000,int(m.timestamp),int(m.round_trip_time),int(m.observed_offset),int(m.estimated_offset)])
    subs=[node.create_subscription(SensorCombined,"/fmu/out/sensor_combined",on_imu,qos_profile_sensor_data),
          node.create_subscription(VehicleStatus,"/fmu/out/vehicle_status_v1",on_status,qos_profile_sensor_data),
          node.create_subscription(TimesyncStatus,"/fmu/out/timesync_status",on_sync,qos_profile_sensor_data)]
    def spin():
        try:
            while not done.is_set():rclpy.spin_once(node,timeout_sec=.01)
        except Exception as e:errors.append("ROS: "+str(e));done.set()
    ros=threading.Thread(target=spin);ros.start()
    q=queue.Queue(maxsize=12);ff=(out/"frames.csv").open("w",buffering=65536);fw=csv.writer(ff)
    fw.writerow(["index","timestamp_us","steady_s","file","bytes","read_start_us","capture_duration_ms"])
    def writer():
        try:
            while True:
                item=q.get()
                try:
                    if item is None:return
                    idx,stamp,mono,raw,start=item;name=f"frames/{idx:06d}.jpg"
                    (out/name).write_bytes(raw)
                    fw.writerow([idx,stamp,mono,name,len(raw),start,(stamp-start)*.001]);counts["frames"]+=1
                finally:q.task_done()
        except Exception as e:errors.append("writer: "+str(e));done.set()
    disk=threading.Thread(target=writer);disk.start()
    cap=None;start=time.monotonic();last_report=0.;index=0;wall0=time.time();mono0=start
    def stop(*_):done.set()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        deadline=time.monotonic()+8
        while not status["received"] and time.monotonic()<deadline:time.sleep(.05)
        if not status["received"] or status["armed"]:raise RuntimeError("Need fresh DISARMED vehicle status")
        cap=cv2.VideoCapture("/dev/video0",cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,2720);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,960)
        cap.set(cv2.CAP_PROP_FPS,30);cap.set(cv2.CAP_PROP_BUFFERSIZE,1);cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
        if not cap.isOpened():raise RuntimeError("DECXIN Camera not open")
        start=time.monotonic();wall0=time.time();mono0=start;next_frame=start
        print("CAPTURE_READY ground/disarmed; raw JPEG + PX4 IMU",flush=True)
        while not done.is_set() and time.monotonic()-start<args.seconds:
            if status["armed"] or time.monotonic()-status["last"]>2:raise RuntimeError("Disarmed status lost")
            before=time.time_ns()//1000
            ok,raw=cap.read();stamp=time.time_ns()//1000;mono=time.monotonic()
            if abs((time.time()-wall0)-(mono-mono0))>.02:raise RuntimeError("System clock jumped")
            if not ok or raw is None or raw.size<100:counts["bad"]+=1;continue
            if mono>=next_frame:
                if index==0:
                    gray=cv2.imdecode(raw,cv2.IMREAD_GRAYSCALE)
                    if gray is None or gray.shape!=(960,2720):raise RuntimeError("Raw JPEG shape invalid")
                payload=raw.tobytes()
                if not payload.startswith(b"\xff\xd8"):
                    counts["bad"]+=1;continue
                try:q.put_nowait((index,stamp,mono,payload,before))
                except queue.Full:raise RuntimeError("Raw writer queue full: calibration recording incomplete")
                index+=1;next_frame=mono+1./args.hz
            elapsed=mono-start
            if elapsed-last_report>=2:
                print(json.dumps({"elapsed":round(elapsed,1),**counts,"queue":q.qsize()}),flush=True);last_report=elapsed
        if done.is_set() and time.monotonic()-start<args.seconds-1:errors.append("Capture interrupted")
    except Exception as e:errors.append(str(e));traceback.print_exc()
    finally:
        if cap is not None:cap.release()
        done.set();ros.join(timeout=3)
        # Writer failures terminate capture; avoid waiting forever on an abandoned queue.
        if disk.is_alive():q.put(None,timeout=3);disk.join(timeout=10)
        if disk.is_alive():errors.append("Writer did not flush")
        ff.close();imu_file.close();sf.close();node.destroy_node();rclpy.shutdown()
        summary={"duration":time.monotonic()-start,"counts":counts,"errors":errors,
            "image_timestamp":"software immediately after V4L2 read; same convention as stereo_uvc",
            "imu_domain":"DDS timestamp, must match host system microseconds",
            "imu_gap_max_s":max(intervals) if intervals else None,
            "imu_nonpositive":sum(x<=0 for x in intervals),"complete":not errors}
        (out/"summary.json").write_text(json.dumps(summary,indent=2))
        print(json.dumps(summary),flush=True)
    return 0 if not errors else 1
if __name__=="__main__":raise SystemExit(main())
