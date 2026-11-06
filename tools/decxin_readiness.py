#!/usr/bin/env python3
"""Read-only hardware readiness capture. No command or setpoint publishers."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import SensorCombined,VehicleStatus,VehicleLocalPosition,EstimatorStatusFlags,VehicleOdometry
from std_msgs.msg import String
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--seconds",type=float,default=30.)
    ap.add_argument("--out",required=True)
    args=ap.parse_args()
    if not 1<=args.seconds<=3600:ap.error("seconds must be 1..3600")
    rclpy.init()
    node=Node("decxin_readiness_observer")
    counts={};latest={};last={};intervals=[];imu_previous=None
    def receive(name,m):
        nonlocal imu_previous
        counts[name]=counts.get(name,0)+1;last[name]=time.monotonic()
        if name=="imu":
            if imu_previous is not None:intervals.append((m.timestamp-imu_previous)*1e-6)
            imu_previous=m.timestamp
            latest[name]={"timestamp":m.timestamp,"accel":[float(v) for v in m.accelerometer_m_s2],"gyro":[float(v) for v in m.gyro_rad],
                "relative_accel_us":m.accelerometer_timestamp_relative,
                "clipping":int(m.accelerometer_clipping)|int(m.gyro_clipping)}
        elif name=="status":latest[name]={"arming_state":m.arming_state,"nav_state":m.nav_state}
        elif name=="position":latest[name]={"xy_valid":m.xy_valid,"z_valid":m.z_valid,"v_xy_valid":m.v_xy_valid,
            "v_z_valid":m.v_z_valid,"xyz":[m.x,m.y,m.z]}
        elif name=="fusion":latest[name]={"ev_pos":m.cs_ev_pos,"ev_vel":m.cs_ev_vel,"ev_hgt":m.cs_ev_hgt}
        elif name=="health":
            try:latest[name]=json.loads(m.data)
            except ValueError:latest[name]={"parse_error":True}
        elif name=="odometry":latest[name]={"timestamp":m.timestamp,"timestamp_sample":m.timestamp_sample}
    subs=[]
    for name,typ,topic in [
        ("imu",SensorCombined,"/fmu/out/sensor_combined"),
        ("status",VehicleStatus,"/fmu/out/vehicle_status_v1"),
        ("position",VehicleLocalPosition,"/fmu/out/vehicle_local_position_v1"),
        ("fusion",EstimatorStatusFlags,"/fmu/out/estimator_status_flags"),
        ("odometry",VehicleOdometry,"/fmu/in/vehicle_visual_odometry")]:
        subs.append(node.create_subscription(typ,topic,lambda m,n=name:receive(n,m),qos_profile_sensor_data))
    subs.append(node.create_subscription(String,"/vio/navigation_health",lambda m:receive("health",m),10))
    start=time.monotonic()
    while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.05)
    end=time.monotonic();duration=end-start
    result={"duration_seconds":duration,"counts":counts,"hz":{k:v/duration for k,v in counts.items()},
        "last_age_seconds":{k:end-v for k,v in last.items()},"latest":latest,
        "imu_intervals":{"samples":len(intervals),"min_seconds":min(intervals) if intervals else None,
            "max_seconds":max(intervals) if intervals else None,"nonpositive":sum(x<=0 for x in intervals),
            "over_30ms":sum(x>.03 for x in intervals)}}
    result["navigation_ready"]=bool(latest.get("health",{}).get("navigation_ready",False))
    Path(args.out).write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))
    node.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
