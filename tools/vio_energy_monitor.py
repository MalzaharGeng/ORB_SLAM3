#!/usr/bin/env python3
"""Read-only ROS energy monitor; no flight commands or arming actions."""
import argparse
import json
from pathlib import Path
import time
from vio_energy import EnergyAccumulator

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--battery-id',required=True,help='Explicit pack/charge-session identifier; never reset just because landing occurred')
    parser.add_argument('--state-file',type=Path,required=True)
    parser.add_argument('--source-id',type=int,required=True,help='Explicit BatteryStatus.id; ignore other sources')
    parser.add_argument('--topic-prefix',default='')
    parser.add_argument('--initial-usable-wh',type=float)
    parser.add_argument('--next-task-wh',type=float)
    parser.add_argument('--reserve-wh',type=float)
    args=parser.parse_args()
    if args.source_id<0:parser.error('source-id must be nonnegative')
    if args.topic_prefix and (not args.topic_prefix.startswith('/') or args.topic_prefix.endswith('/')):
        parser.error('topic-prefix must be empty or /namespace without trailing slash')
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    energy=EnergyAccumulator.load(args.state_file,args.battery_id,boot_id) if args.state_file.exists() else EnergyAccumulator(args.battery_id)
    energy.snapshot(time.monotonic(),args.initial_usable_wh,args.next_task_wh,args.reserve_wh)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import BatteryStatus
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('vio_energy_monitor')
    publisher=node.create_publisher(String,args.topic_prefix+'/vio/energy_status',10)
    selected=[args.source_id]
    def receive(message):
        if message.id!=selected[0]:return
        source_age=(time.time_ns()//1000-int(message.timestamp))*1e-6
        source_fresh=-.1<=source_age<=energy.max_gap_seconds
        energy.update(int(message.timestamp),time.monotonic(),float(message.voltage_v),float(message.current_a),bool(message.connected) and source_fresh)
        # Persist every accepted packet so an ordinary monitor restart cannot
        # silently reset the consumption accumulated across prior flight legs.
        energy.save(args.state_file,boot_id)
    version=getattr(BatteryStatus,'MESSAGE_VERSION',0)
    topic=args.topic_prefix+'/fmu/out/battery_status'+(f'_v{version}' if version else '')
    node.create_subscription(BatteryStatus,topic,receive,qos_profile_sensor_data)
    def report():
        status=energy.snapshot(time.monotonic(),args.initial_usable_wh,args.next_task_wh,args.reserve_wh)
        status.update(timestamp_us=time.time_ns()//1000,source_id=selected[0])
        msg=String();msg.data=json.dumps(status,allow_nan=False);publisher.publish(msg)
    node.create_timer(.5,report)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        energy.save(args.state_file,boot_id);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
