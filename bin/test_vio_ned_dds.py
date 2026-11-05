#!/usr/bin/env python3
"""Real DDS NED delivery test on domain 43 /vio_test, isolated from hardware."""
import math,os,subprocess,tempfile,time
os.environ['ROS_DOMAIN_ID']='43'
import rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from px4_msgs.msg import VehicleOdometry,VehicleStatus,VehicleLocalPosition,TimesyncStatus

def topic(base,cls):
    v=getattr(cls,'MESSAGE_VERSION',0)
    return '/vio_test/fmu/'+base+(('_v%d'%v) if v else '')

rclpy.init();node=rclpy.create_node('repair_ned_dds_peer');received=[]
pubs={cls:node.create_publisher(cls,topic('out/'+base,cls),qos)
      for base,cls in [('vehicle_status',VehicleStatus),('vehicle_local_position',VehicleLocalPosition),('timesync_status',TimesyncStatus)]}
node.create_subscription(VehicleOdometry,topic('in/vehicle_visual_odometry',VehicleOdometry),received.append,qos)
airborne=os.environ.get("VIO_TEST_AIRBORNE")=="1"
heading=.9
def emit():
    now=time.time_ns()//1000
    m=VehicleStatus();m.timestamp=now;m.arming_state=VehicleStatus.ARMING_STATE_ARMED if airborne else VehicleStatus.ARMING_STATE_DISARMED;m.nav_state=1;pubs[VehicleStatus].publish(m)
    p=VehicleLocalPosition();p.timestamp=now;p.heading=heading;p.x=2.;p.y=3.;p.z=-.5;p.xy_valid=p.z_valid=True
    pubs[VehicleLocalPosition].publish(p)
    s=TimesyncStatus();s.timestamp=now;s.source_protocol=TimesyncStatus.SOURCE_PROTOCOL_DDS
    s.round_trip_time=500;pubs[TimesyncStatus].publish(s)
node.create_timer(.03,emit)
def until(condition,seconds=10):
    end=time.monotonic()+seconds
    while not condition() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    assert condition(),'NED DDS timeout'
with tempfile.TemporaryDirectory() as d:
    with open(d+'/fixture.log','w+') as log:
        proc=subprocess.Popen(['/home/nano/ORB_SLAM3/build/tests/px4_bridge_fixture','--ned-air' if airborne else '--ned'],stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,text=True,cwd=d)
        try:
            until(lambda:len(received)>10)
            for m in received:
                assert m.pose_frame==VehicleOdometry.POSE_FRAME_NED and m.velocity_frame==VehicleOdometry.VELOCITY_FRAME_NED
                assert max(abs(a-b) for a,b in zip(m.position,(2.,3.,-.5)))<1e-5
                assert abs(2*math.atan2(m.q[3],m.q[0])-.9)<1e-5
            heading=1.3
            proc.stdin.write('velocity\n');proc.stdin.flush()
            until(lambda:any(abs(m.velocity[0])>.01 for m in received))
            m=received[-1]
            assert abs(m.velocity[0]-.25*math.cos(.9))<1e-5
            assert abs(m.velocity[1]-.25*math.sin(.9))<1e-5
            print('VIO_NED_DDS_REGRESSION_PASS airborne=%s observations=%d'%(airborne,len(received)))
        except Exception:
            log.flush();log.seek(0);print(log.read());raise
        finally:
            proc.stdin.write('quit\n');proc.stdin.flush();proc.wait(timeout=5)
node.destroy_node();rclpy.shutdown()
