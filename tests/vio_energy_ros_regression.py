"""Battery transport and unknown-capacity behavior in DDS domain45 only."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
os.environ['ROS_DOMAIN_ID']='45'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import BatteryStatus
from std_msgs.msg import String

def main():
    rclpy.init();node=rclpy.create_node('energy_test_peer');received=[];stream=[True]
    ver=getattr(BatteryStatus,'MESSAGE_VERSION',0)
    topic='/vio_energy_test/fmu/out/battery_status'+(f'_v{ver}' if ver else '')
    publisher=node.create_publisher(BatteryStatus,topic,qos_profile_sensor_data)
    node.create_subscription(String,'/vio_energy_test/vio/energy_status',lambda m:received.append(json.loads(m.data)),10)
    def emit():
        if stream[0]:publisher.publish(BatteryStatus(timestamp=time.time_ns()//1000,id=1,connected=True,voltage_v=24.,current_a=10.,remaining=0.))
    node.create_timer(.1,emit)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def until(predicate,seconds=6):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert predicate(),'Energy DDS test timed out'
    with tempfile.TemporaryDirectory(prefix='vio-energy-') as directory:
        with open(Path(directory)/'node.log','w+') as log:
            args=[sys.executable,str(Path(__file__).resolve().parents[1]/'tools/vio_energy_monitor.py'),
                  '--battery-id','test-pack','--source-id','1','--state-file',str(Path(directory)/'state.json'),'--topic-prefix','/vio_energy_test']
            proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
            try:
                until(lambda:received and received[-1]['consumed_wh']>.03)
                assert received[-1]['measurement_valid'] and received[-1]['coverage_complete']
                assert received[-1]['energy_sufficient'] is None and received[-1]['remaining_usable_wh'] is None
                profile=received[-1]['battery_profile']
                assert abs(profile['nominal_reference_wh']-126.54)<1e-8 and not profile['nameplate_consistent']
                before=received[-1]['consumed_wh'];stream[0]=False;spin(3.7)
                assert not received[-1]['measurement_valid'],'stale battery remained valid'
                stream[0]=True;until(lambda:received[-1]['measurement_valid'])
                assert not received[-1]['coverage_complete'],'missing energy interval hidden'
                assert received[-1]['consumed_wh']>=before and received[-1]['energy_sufficient'] is None
                print('VIO_ENERGY_ROS_PASS consumed_wh='+str(received[-1]['consumed_wh'])+' unknown_capacity_blocks_permission=1 gap_detected=1')
            except Exception:
                log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
            finally:
                proc.terminate();proc.wait(timeout=5)
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
