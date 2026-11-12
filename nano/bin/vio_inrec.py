#!/usr/bin/env python3
"""Record VIO, PX4 state, attitude and manual input without sending commands."""
import argparse
import csv
import json
import math
import os
import re
import queue
import signal
import threading
import time
from datetime import datetime
from pathlib import Path
from vio_csv_store import RollingCsvStore

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from px4_msgs.msg import (VehicleOdometry, VehicleLocalPosition, VehicleStatus,
                          EstimatorStatusFlags, VehicleAttitude, VehicleAttitudeSetpoint,
                          ManualControlSetpoint, TimesyncStatus, SensorCombined, VehicleLandDetected,
                          ManualControlSwitches, BatteryStatus, FailsafeFlags)
from std_msgs.msg import String

COLS = ["wall_t", "src", "t_msg_us", "x", "y", "z", "vx", "vy", "vz",
        "qw", "qx", "qy", "qz", "quality", "armed", "nav_state",
        "cs_ev_pos", "cs_ev_hgt", "cs_ev_yaw", "note", "uptime_s",
        "schema_version", "steady_t", "t_sample_us", "pose_frame", "velocity_frame",
        "reset_counter", "position_variance", "velocity_variance", "orientation_variance",
        "status_age_s", "flags_age_s", "bridge_hz", "bridge_age_ms", "bridge_sent",
        "bridge_rejected", "payload_json"]

RE_BRIDGE = re.compile(r"\[PX4\]\s+(?P<state>\w+)\s+hz=(?P<hz>[-\d.e+]+)\s+"
                       r"age_ms=(?P<age>[-\d.e+]+)\s+sent=(?P<sent>\d+)\s+rejected=(?P<rej>\d+)")

def topic(base, cls):
    version = getattr(cls, "MESSAGE_VERSION", 0)
    return base + ("_v%d" % version if version else "")

def payload(msg):
    return json.dumps(message_to_ordereddict(msg), default=list, separators=(",", ":"))

class Inrec(Node):
    def __init__(self, out_path, bridge_log=None):
        super().__init__("vio_inrec")
        self.t0 = time.monotonic()
        self.out_path = str(out_path)
        self.storage = RollingCsvStore(out_path, COLS,
            segment_bytes=int(os.environ.get('VIO_DATA_SEGMENT_MIB','128'))*1024**2,
            budget_bytes=int(os.environ.get('VIO_DATA_BUDGET_MIB','8192'))*1024**2,
            min_free_bytes=int(os.environ.get('VIO_DATA_MIN_FREE_MIB','2048'))*1024**2)
        self.fh = self.storage
        self.w = self.storage
        self.w.writeheader()
        self.fh.flush()
        self.write_queue=queue.Queue(maxsize=8192)
        self.writer_error=""
        self.written=0
        self.written_by_source={}
        self.received={}
        self.last_source={}
        self.last_message_us={}
        self.landed=None
        self.last_stamp={}
        self.gaps={}
        self.nav_state={}
        self.writer=threading.Thread(target=self._write_rows,name="inrec-writer")
        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.closed = False
        self.n = 0
        self.armed = self.nav = -1
        self.cev = (-1, -1, -1)
        self.status_t = self.flags_t = None
        self.last_flush = self.t0
        self.cnt = {}
        self.subscriptions_kept = []
        self.writer.start()
        for base, cls, callback in (
            ("/fmu/in/vehicle_visual_odometry", VehicleOdometry, self.on_vio),
            ("/fmu/out/vehicle_local_position", VehicleLocalPosition, self.on_lpos),
            ("/fmu/out/vehicle_status", VehicleStatus, self.on_status),
            ("/fmu/out/estimator_status_flags", EstimatorStatusFlags, self.on_flags),
            ("/fmu/out/vehicle_attitude", VehicleAttitude, lambda m: self.on_other("attitude", m)),
            ("/fmu/out/vehicle_attitude_setpoint", VehicleAttitudeSetpoint, lambda m: self.on_other("attitude_setpoint", m)),
            ("/fmu/out/manual_control_setpoint", ManualControlSetpoint, lambda m: self.on_other("manual_control", m)),
            ("/fmu/out/timesync_status", TimesyncStatus, lambda m: self.on_other("timesync", m)),
            ("/fmu/out/sensor_combined", SensorCombined, lambda m: self.on_other("imu_raw",m)),
            ("/fmu/out/vehicle_land_detected", VehicleLandDetected, lambda m: self.on_other("land",m)),
            ("/fmu/out/manual_control_switches", ManualControlSwitches, lambda m: self.on_other("manual_switches",m)),
            ("/fmu/out/battery_status", BatteryStatus, lambda m: self.on_other("battery",m)),
            ("/fmu/out/failsafe_flags", FailsafeFlags, lambda m: self.on_other("failsafe",m)),
        ):
            qos=qos_profile_sensor_data
            if cls in (SensorCombined,VehicleAttitude):
                qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST)
            self.subscriptions_kept.append(self.create_subscription(cls, topic(base, cls), callback, qos))
        self.subscriptions_kept.append(self.create_subscription(String,"/vio/navigation_health",self.on_health,10))
        self.br_thread = None
        if bridge_log:
            self.br_thread = threading.Thread(target=self._tail_bridge, args=(bridge_log,), daemon=True)
            self.br_thread.start()
        for name in ("task_status","task_event"):
            self.subscriptions_kept.append(self.create_subscription(String,
                "/navigation/"+name,lambda m,n=name:self.on_task(n,m),10))
        self.create_timer(5., self.heartbeat)
        self.create_timer(1., self.verify)
        self.row("recording_start",payload_json=json.dumps({"schema_version":3,"images_recorded":False}))
        self.write_manifest(False)
        print("INREC_READY",self.out_path,flush=True)

    def row(self, src, **fields):
        now = time.monotonic()
        with self.lock:
            if self.closed:
                return
            row = dict(wall_t="%.6f" % time.time(), steady_t="%.6f" % now, src=src,
                       schema_version=3, uptime_s="%.3f" % (now-self.t0),
                       armed=self.armed, nav_state=self.nav,
                       cs_ev_pos=self.cev[0], cs_ev_hgt=self.cev[1], cs_ev_yaw=self.cev[2],
                       status_age_s=now-self.status_t if self.status_t is not None else "",
                       flags_age_s=now-self.flags_t if self.flags_t is not None else "")
            row.update(fields)
            self.received[src]=self.received.get(src,0)+1
            self.last_source[src]=now
            stamp=fields.get("t_sample_us") or fields.get("t_msg_us")
            if stamp:self.last_message_us[src]=int(stamp)
            if src=="imu_raw" and stamp:
                stamp=int(stamp)
                previous=self.last_stamp.get(src)
                if previous is not None:
                    gap=stamp-previous
                    if gap<=0 or gap>30000:
                        g=self.gaps.setdefault(src,{"count":0,"nonpositive":0,"max_us":0})
                        g["count"]+=1;g["nonpositive"]+=int(gap<=0);g["max_us"]=max(g["max_us"],gap)
                self.last_stamp[src]=stamp
            try:self.write_queue.put_nowait(row)
            except queue.Full:
                self.writer_error="recording queue full; data incomplete"
                self.stop_flag.set()
                return
            self.n += 1
            self.cnt[src] = self.cnt.get(src, 0) + 1

    def _write_rows(self):
        try:
            while True:
                row=self.write_queue.get()
                try:
                    if row is None:break
                    self.w.writerow(row);self.written+=1
                    src=row["src"];self.written_by_source[src]=self.written_by_source.get(src,0)+1
                    now=time.monotonic()
                    if self.written%200==0 or now-self.last_flush>=1.:
                        self.fh.flush()
                        if now-self.last_flush>=1.:
                            os.fsync(self.fh.fileno());self.last_flush=now
                finally:self.write_queue.task_done()
        except Exception as error:
            self.writer_error=str(error)
            self.stop_flag.set()

    def on_health(self,msg):
        try:self.nav_state=json.loads(msg.data)
        except ValueError:self.nav_state={"invalid_json":True}
        self.row("navigation_health",payload_json=msg.data)

    def on_task(self,name,msg):
        try:
            data=json.loads(msg.data)
            stamp=data.get("timestamp_us","") if isinstance(data,dict) else ""
        except ValueError:stamp=""
        self.row(name,t_msg_us=stamp,payload_json=msg.data)

    def verify(self):
        now=time.monotonic()
        required={"imu_raw":.25,"status":2.,"lpos":.5,"timesync":2.}
        wall=time.time()
        checks={name:(0<=now-self.last_source.get(name,-1e20)<=limit and
            -.05<=wall-self.last_message_us.get(name,0)*1e-6<=limit) for name,limit in required.items()}
        self.row("verification",payload_json=json.dumps({
            "input_fresh":all(checks.values()),"streams":checks,
            "writer_error":self.writer_error,"queue":self.write_queue.qsize(),
            "navigation_ready":bool(self.nav_state.get("navigation_ready",False)),
            "imu_gaps":self.gaps},separators=(",",":")))
        self.write_manifest(False)
        if self.writer_error:self.stop_flag.set()

    def write_manifest(self,closed):
        with self.lock:
            result={"schema_version":3,"path":self.out_path,"closed_cleanly":bool(closed),
                "writer_complete":bool(closed and not self.writer_error and self.written==self.n),
                "error":self.writer_error,"accepted_rows":self.n,"written_rows":self.written,
                "received_by_source":dict(self.received),"accepted_by_source":dict(self.cnt),"recorded_by_source":dict(self.written_by_source),
                "queue":self.write_queue.qsize(),"imu_gaps":dict(self.gaps),
                "vehicle":{"armed":self.armed,"landed":self.landed,
                    "status_receipt_steady":self.last_source.get("status"),
                    "status_timestamp_us":self.last_message_us.get("status"),
                    "land_receipt_steady":self.last_source.get("land"),
                    "land_timestamp_us":self.last_message_us.get("land")},
                "images_recorded":False,"storage":self.storage.status(),
                "acquisition_scope":"all DDS messages delivered to this process; gaps reported; older sealed segments rotate within declared storage bounds"}
        path=Path(self.out_path+".summary.json")
        temporary=path.with_suffix(path.suffix+".tmp")
        try:
            temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2))
            os.replace(temporary,path)
        except OSError as error:
            self.writer_error=str(error);self.stop_flag.set()

    def on_vio(self, msg):
        p, v, q = msg.position, msg.velocity, msg.q
        self.row("vio", t_msg_us=msg.timestamp, t_sample_us=msg.timestamp_sample,
                 x=p[0], y=p[1], z=p[2], vx=v[0], vy=v[1], vz=v[2],
                 qw=q[0], qx=q[1], qy=q[2], qz=q[3], quality=msg.quality,
                 pose_frame=msg.pose_frame, velocity_frame=msg.velocity_frame,
                 reset_counter=msg.reset_counter,
                 position_variance=json.dumps([float(v) for v in msg.position_variance]),
                 velocity_variance=json.dumps([float(v) for v in msg.velocity_variance]),
                 orientation_variance=json.dumps([float(v) for v in msg.orientation_variance]),
                 payload_json=payload(msg))

    def on_lpos(self, msg):
        self.row("lpos", t_msg_us=msg.timestamp, t_sample_us=getattr(msg,"timestamp_sample", ""),
                 x=msg.x, y=msg.y, z=msg.z, vx=msg.vx, vy=msg.vy, vz=msg.vz,
                 note="xy_valid=%s z_valid=%s heading=%s" % (msg.xy_valid,msg.z_valid,msg.heading),
                 payload_json=payload(msg))

    def on_status(self, msg):
        with self.lock:
            self.status_t = time.monotonic()
            self.armed = int(msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.nav = msg.nav_state
        self.row("status", t_msg_us=msg.timestamp, payload_json=payload(msg))

    def on_flags(self, msg):
        with self.lock:
            self.flags_t = time.monotonic()
            self.cev = (int(msg.cs_ev_pos),int(msg.cs_ev_hgt),int(msg.cs_ev_yaw))
        self.row("flags", t_msg_us=msg.timestamp, payload_json=payload(msg))

    def on_other(self, name, msg):
        if name=="land":self.landed=bool(msg.landed)
        self.row(name, t_msg_us=msg.timestamp, payload_json=payload(msg))

    def _tail_bridge(self, path):
        fh = None
        partial = ""
        try:
            while not self.stop_flag.is_set():
                try:
                    stat = os.stat(path)
                    if fh is None or os.fstat(fh.fileno()).st_ino != stat.st_ino or fh.tell() > stat.st_size:
                        if fh:
                            fh.close()
                        fh = open(path, encoding="utf-8", errors="replace")
                        partial = ""
                    line = fh.readline()
                    if not line:
                        self.stop_flag.wait(.2)
                        continue
                    partial += line
                    if not partial.endswith("\n"):
                        continue
                    self.parse_bridge(partial)
                    partial = ""
                except OSError:
                    self.stop_flag.wait(.5)
        finally:
            if fh:
                fh.close()

    def parse_bridge(self, line):
        m = RE_BRIDGE.search(line)
        if m:
            self.row("bridge", note=m.group("state"), bridge_hz=m.group("hz"),
                     bridge_age_ms=m.group("age"), bridge_sent=m.group("sent"),
                     bridge_rejected=m.group("rej"), payload_json=json.dumps({"raw":line.strip()}))
        if "[STAT]" in line or "[NAV]" in line or "[IMU]" in line or "VIBA" in line or "状态 ->" in line:
            self.row("slam_event",payload_json=json.dumps({"raw":line.strip()}))

    def heartbeat(self):
        self.get_logger().info("inrec %.1fs rows=%d sources=%s" % (time.monotonic()-self.t0,self.n,self.cnt))

    def close(self):
        if self.closed:return
        self.stop_flag.set()
        if self.br_thread:self.br_thread.join(timeout=2.)
        with self.lock:self.closed=True
        if self.writer.is_alive():
            try:self.write_queue.put(None,timeout=5.)
            except queue.Full:self.writer_error="writer did not drain"
            self.writer.join(timeout=8.)
        if self.writer.is_alive():self.writer_error="writer shutdown timeout"
        try:
            self.fh.flush();os.fsync(self.fh.fileno());self.fh.close()
        except OSError as error:self.writer_error=str(error)
        self.write_manifest(True)


def find_latest_run_log():
    paths = list(Path.home().glob("ORB_SLAM3/recordings/test_runs/*live*/run.log"))
    return str(max(paths, key=lambda p:p.stat().st_mtime)) if paths else None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out")
    ap.add_argument("--duration", type=float, default=0.)
    ap.add_argument("--bridge-log")
    ap.add_argument("--no-bridge", action="store_true")
    args = ap.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        ap.error("duration must be finite and nonnegative")
    directory = Path.home()/"inrec"
    directory.mkdir(exist_ok=True)
    output = args.out or str(directory/("inrec_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".csv"))
    rclpy.init()
    node = Inrec(output, None if args.no_bridge else args.bridge_log or find_latest_run_log())
    def stop(*_):node.stop_flag.set()
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    try:
        while rclpy.ok() and not node.stop_flag.is_set():
            rclpy.spin_once(node,timeout_sec=.2)
            if args.duration and time.monotonic()-node.t0 >= args.duration:
                break
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print("INREC_COMPLETE" if not node.writer_error else "INREC_INCOMPLETE",output,"rows",node.n,"sources",node.cnt)
    return 0 if not node.writer_error else 2

if __name__ == "__main__":raise SystemExit(main())
