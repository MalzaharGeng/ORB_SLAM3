"""Exercise real ROS delivery and IMU fault rejection in isolated domain 45."""
import os,re,subprocess,time,sys
from pathlib import Path
os.environ["ROS_DOMAIN_ID"]="45"
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import SensorCombined
def main():
    rclpy.init()
    node=Node("decxin_imu_test_source")
    pub=node.create_publisher(SensorCombined,"/decxin_imu_test/fmu/out/sensor_combined",qos_profile_sensor_data)
    probe=subprocess.Popen([sys.argv[1] if len(sys.argv)>1 else str(Path(__file__).resolve().parents[1]/"build/tests/px4_imu_probe"),"45","/decxin_imu_test"],
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    start=time.monotonic();next_send=start
    while probe.poll() is None and time.monotonic()-start<20:
        elapsed=time.monotonic()-start
        if time.monotonic()>=next_send:
            m=SensorCombined()
            m.timestamp=time.time_ns()//1000
            m.gyro_integral_dt=m.accelerometer_integral_dt=5000
            m.accelerometer_m_s2=[0.,0.,-9.81]
            if 4<elapsed<5:m.gyro_clipping=1
            if 8<elapsed<8.1:m.timestamp-=100000
            pub.publish(m);next_send+=.005
        rclpy.spin_once(node,timeout_sec=.001)
    if probe.poll() is None:probe.kill()
    output=probe.communicate(timeout=5)[0]
    print(output)
    node.destroy_node();rclpy.shutdown()
    assert output.count("interval=ok")>=6,output
    rejected=[int(v) for v in re.findall(r"rejected=(\d+)",output)]
    epochs=[int(v) for v in re.findall(r"epoch=(\d+)",output)]
    assert rejected and max(rejected)>=100,output
    assert epochs and max(epochs)>=2,output
    assert "imu_interval_uncovered" in output,output
    print("PASS isolated DDS: normal intervals, clipped samples, backward clock, fail-closed coverage, receive recovery")
if __name__=="__main__":main()
