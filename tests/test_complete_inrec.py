"""Recorder completeness and gap visibility, using isolated ROS domain."""
import csv,importlib.util,json,os,tempfile,unittest,time,sys
from pathlib import Path
os.environ["ROS_DOMAIN_ID"]="57"
import rclpy
from px4_msgs.msg import SensorCombined,VehicleStatus,VehicleLandDetected
spec=importlib.util.spec_from_file_location("vio_inrec",str(Path(__file__).resolve().parents[1]/"nano/bin/vio_inrec.py"))
if not Path(spec.origin).exists():
    spec=importlib.util.spec_from_file_location("vio_inrec","/home/nano/vio_inrec.py")
sys.path.insert(0,str(Path(spec.origin).parent))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class RecordingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def test_raw_imu_every_received_sample_flushed_and_gap_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"complete.csv";node=module.Inrec(path)
            try:
                status=VehicleStatus();status.arming_state=VehicleStatus.ARMING_STATE_DISARMED;node.on_status(status)
                for i in range(1000):
                    m=SensorCombined();m.timestamp=1000000+i*10000+(100000 if i>=500 else 0)
                    m.gyro_integral_dt=m.accelerometer_integral_dt=5000
                    m.accelerometer_m_s2=[.1,.2,-9.81];m.gyro_rad=[.01,.02,.03]
                    node.on_other("imu_raw",m)
                node.verify();node.close()
                with path.open() as recorded:
                    rows=list(csv.DictReader(recorded))
                imu=[x for x in rows if x["src"]=="imu_raw"]
                self.assertEqual(len(imu),1000)
                self.assertEqual(len({x["t_msg_us"] for x in imu}),1000)
                for row in imu:
                    data=json.loads(row["payload_json"]);self.assertIn("accelerometer_m_s2",data)
                    self.assertIn("gyro_rad",data)
                summary=json.loads(Path(str(path)+".summary.json").read_text())
                self.assertTrue(summary["writer_complete"]);self.assertEqual(summary["accepted_rows"],len(rows))
                self.assertEqual(summary["written_rows"],len(rows));self.assertEqual(summary["imu_gaps"]["imu_raw"]["count"],1)
                self.assertGreater(summary["imu_gaps"]["imu_raw"]["max_us"],100000)
            finally:node.close();node.destroy_node()
    def test_stale_source_not_renewed_and_restart_evidence_persisted(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"freshness.csv";node=module.Inrec(path)
            try:
                old=int((time.time()-10)*1e6)
                for name in ("imu_raw","status","lpos","timesync"):
                    node.row(name,t_msg_us=old)
                node.verify();node.close()
                with path.open() as recorded:rows=list(csv.DictReader(recorded))
                verify=json.loads(next(r["payload_json"] for r in rows if r["src"]=="verification"))
                self.assertFalse(verify["input_fresh"])
                self.assertFalse(any(verify["streams"].values()))
            finally:node.close();node.destroy_node()
            path=Path(directory)/"restart.csv";node=module.Inrec(path)
            try:
                stamp=time.time_ns()//1000
                status=VehicleStatus();status.timestamp=stamp
                status.arming_state=VehicleStatus.ARMING_STATE_DISARMED;node.on_status(status)
                land=VehicleLandDetected();land.timestamp=stamp;land.landed=True;node.on_other("land",land)
                node.close()
                summary=json.loads(Path(str(path)+".summary.json").read_text())
                self.assertEqual(summary["vehicle"]["armed"],0)
                self.assertTrue(summary["vehicle"]["landed"])
                self.assertEqual(summary["vehicle"]["status_timestamp_us"],stamp)
            finally:node.close();node.destroy_node()

    def test_writer_failure_is_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            node=module.Inrec(Path(directory)/"failed.csv")
            node.writer_error="injected writer failure"
            node.close();node.destroy_node()
            summary=json.loads(Path(directory,"failed.csv.summary.json").read_text())
            self.assertFalse(summary["writer_complete"]);self.assertTrue(summary["error"])
if __name__=="__main__":unittest.main()
