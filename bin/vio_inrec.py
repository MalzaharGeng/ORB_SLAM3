#!/usr/bin/env python3
"""Record VIO, PX4 state, attitude and manual input without sending commands."""
import argparse
import csv
import json
import math
import os
import re
import signal
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from px4_msgs.msg import (VehicleOdometry, VehicleLocalPosition, VehicleStatus,
                          EstimatorStatusFlags, VehicleAttitude, VehicleAttitudeSetpoint,
                          ManualControlSetpoint, TimesyncStatus)

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
        self.fh = open(out_path, "w", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.fh, COLS)
        self.w.writeheader()
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
        for base, cls, callback in (
            ("/fmu/in/vehicle_visual_odometry", VehicleOdometry, self.on_vio),
            ("/fmu/out/vehicle_local_position", VehicleLocalPosition, self.on_lpos),
            ("/fmu/out/vehicle_status", VehicleStatus, self.on_status),
            ("/fmu/out/estimator_status_flags", EstimatorStatusFlags, self.on_flags),
            ("/fmu/out/vehicle_attitude", VehicleAttitude, lambda m: self.on_other("attitude", m)),
            ("/fmu/out/vehicle_attitude_setpoint", VehicleAttitudeSetpoint, lambda m: self.on_other("attitude_setpoint", m)),
            ("/fmu/out/manual_control_setpoint", ManualControlSetpoint, lambda m: self.on_other("manual_control", m)),
            ("/fmu/out/timesync_status", TimesyncStatus, lambda m: self.on_other("timesync", m)),
        ):
            self.subscriptions_kept.append(self.create_subscription(cls, topic(base, cls), callback, qos_profile_sensor_data))
        self.br_thread = None
        if bridge_log:
            self.br_thread = threading.Thread(target=self._tail_bridge, args=(bridge_log,), daemon=True)
            self.br_thread.start()
        self.create_timer(5., self.heartbeat)

    def row(self, src, **fields):
        now = time.monotonic()
        with self.lock:
            if self.closed:
                return
            row = dict(wall_t="%.6f" % time.time(), steady_t="%.6f" % now, src=src,
                       schema_version=2, uptime_s="%.3f" % (now-self.t0),
                       armed=self.armed, nav_state=self.nav,
                       cs_ev_pos=self.cev[0], cs_ev_hgt=self.cev[1], cs_ev_yaw=self.cev[2],
                       status_age_s=now-self.status_t if self.status_t is not None else "",
                       flags_age_s=now-self.flags_t if self.flags_t is not None else "")
            row.update(fields)
            self.w.writerow(row)
            self.n += 1
            self.cnt[src] = self.cnt.get(src, 0) + 1
            if self.n % 100 == 0 or now-self.last_flush > 2.:
                self.fh.flush()
                self.last_flush = now

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

    def heartbeat(self):
        self.get_logger().info("inrec %.1fs rows=%d sources=%s" % (time.monotonic()-self.t0,self.n,self.cnt))

    def close(self):
        self.stop_flag.set()
        if self.br_thread:
            self.br_thread.join(timeout=2.)
        with self.lock:
            if not self.closed:
                self.closed = True
                self.fh.flush()
                os.fsync(self.fh.fileno())
                self.fh.close()

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
    print("INREC_COMPLETE",output,"rows",node.n,"sources",node.cnt)

if __name__ == "__main__":main()
