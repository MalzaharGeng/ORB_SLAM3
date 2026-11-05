"""Real DDS lifecycle peer in domain43; never publishes vehicle commands."""
import json
import math
import os
import pathlib
import subprocess
import sys
import tempfile
import time
os.environ['ROS_DOMAIN_ID']='43'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleLandDetected, TimesyncStatus, EstimatorStatusFlags, VehicleOdometry

def topic(base,cls):
    v=getattr(cls,'MESSAGE_VERSION',0)
    return '/vio_test/fmu/'+base+(f'_v{v}' if v else '')

def main():
    rclpy.init();node=rclpy.create_node('vio_sortie_peer')
    state={'armed':False,'landed':True,'land_stream':True,'fusion':True}
    health=[];received=[]
    pubs={cls:node.create_publisher(cls,topic('out/'+name,cls),qos_profile_sensor_data)
          for cls,name in [(VehicleStatus,'vehicle_status'),(VehicleLocalPosition,'vehicle_local_position'),
                           (VehicleLandDetected,'vehicle_land_detected'),(TimesyncStatus,'timesync_status'),(EstimatorStatusFlags,'estimator_status_flags')]}
    node.create_subscription(String,'/vio_test/vio/navigation_health',lambda m:health.append((time.monotonic(),json.loads(m.data))),10)
    node.create_subscription(VehicleOdometry,topic('in/vehicle_visual_odometry',VehicleOdometry),received.append,qos_profile_sensor_data)
    def emit():
        now=time.time_ns()//1000
        st=VehicleStatus(timestamp=now);st.arming_state=VehicleStatus.ARMING_STATE_ARMED if state['armed'] else VehicleStatus.ARMING_STATE_DISARMED;st.nav_state=3
        pubs[VehicleStatus].publish(st)
        pubs[VehicleLocalPosition].publish(VehicleLocalPosition(timestamp=now,xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,x=2.,y=3.,z=-.5,heading=.4))
        pubs[TimesyncStatus].publish(TimesyncStatus(timestamp=now,source_protocol=TimesyncStatus.SOURCE_PROTOCOL_DDS,round_trip_time=1000))
        pubs[EstimatorStatusFlags].publish(EstimatorStatusFlags(timestamp=now,cs_ev_pos=state['fusion']))
        if state['land_stream']:pubs[VehicleLandDetected].publish(VehicleLandDetected(timestamp=now,landed=state['landed']))
    node.create_timer(.03,emit)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def until(predicate,seconds=8):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert predicate(),'sortie condition timed out'
    def latest():return health[-1][1] if health else {}
    with tempfile.TemporaryDirectory(prefix='vio-sortie-') as directory:
        with open(pathlib.Path(directory)/'bridge.log','w+') as log:
            proc=subprocess.Popen([sys.argv[1],'--sortie'],stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,text=True,cwd=directory)
            def command(s):proc.stdin.write(s+'\n');proc.stdin.flush()
            try:
                until(lambda:latest().get('ground_ready',False))
                first=received[-1];token=(latest()['instance_id'],latest()['reset_counter'])
                assert latest()['schema_version']==1 and latest()['navigation_ready']
                assert all(k in latest() for k in ['px4_xy_reset_counter','px4_z_reset_counter','px4_heading_reset_counter'])
                state['armed']=True;state['landed']=False;spin(.6)
                assert latest()['navigation_ready'] and not latest()['ground_ready']
                command('motion_fault');until(lambda:latest().get('airborne_fault',False))
                assert not latest()['requires_disarm'] and latest()['fault_reason']=='velocity_out_of_bounds'
                count=len(received);spin(.6);assert len(received)==count,'in-flight motion fault auto-recovered'
                state['landed']=True;state['land_stream']=False;spin(.7)
                assert latest()['airborne_fault'],'missing land stream recovered fault'
                state['land_stream']=True;until(lambda:latest().get('ground_ready',False),seconds=9)
                assert latest()['ground_recoveries']==1 and (latest()['instance_id'],latest()['reset_counter'])==token
                assert max(abs(a-b) for a,b in zip(received[-1].position,first.position))<.25,'recovery moved origin'
                state['landed']=False;spin(.6);assert latest()['navigation_ready'] and not latest()['ground_ready']
                state['fusion']=False;spin(.6);assert not latest()['navigation_ready'],'fusion loss kept readiness'
                state['fusion']=True;until(lambda:latest().get('navigation_ready',False))
                command('clock');until(lambda:latest().get('requires_disarm',False))
                state['landed']=True;spin(3.5);assert latest()['airborne_fault'] and not latest()['ground_ready']
                command('quit');proc.wait(timeout=5);assert proc.returncode==0
                print('PX4_SORTIE_ROS_PASS two_armed_sorties=1 ground_recoveries=1 hard_fault_blocked=1 health_messages='+str(len(health)))
            except Exception:
                log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
            finally:
                if proc.poll() is None:proc.terminate();proc.wait(timeout=5)
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
