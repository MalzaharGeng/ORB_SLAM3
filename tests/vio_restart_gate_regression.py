import importlib.util,unittest,copy
from pathlib import Path
path=Path(__file__).resolve().parents[1]/"nano/bin/vio_restart_gate.py"
spec=importlib.util.spec_from_file_location("restart_gate",path)
gate=importlib.util.module_from_spec(spec);spec.loader.exec_module(gate)
class RestartGateTest(unittest.TestCase):
    def setUp(self):
        self.steady=100.;self.wall=1800000000.
        self.base=dict(closed_cleanly=False,error="",vehicle=dict(armed=0,landed=True,
            status_receipt_steady=99.5,status_timestamp_us=1799999999500000,
            land_receipt_steady=99.,land_timestamp_us=1799999999000000))
    def test_fresh_disarmed_and_landed(self):self.assertEqual(gate.reason(self.base,self.steady,self.wall),"")
    def test_airborne_and_armed_never_restart(self):
        for update in [dict(armed=1),dict(landed=False),dict(armed=-1),dict(landed=None)]:
            value=copy.deepcopy(self.base);value["vehicle"].update(update)
            self.assertTrue(gate.reason(value,self.steady,self.wall))
    def test_receipt_cannot_hide_old_flight_status(self):
        value=copy.deepcopy(self.base);value["vehicle"]["status_timestamp_us"]-=10000000
        self.assertEqual(gate.reason(value,self.steady,self.wall),"stale_status_sample")
    def test_stale_or_previous_boot_evidence(self):
        for key,value in [("status_receipt_steady",90.),("land_receipt_steady",101.)]:
            data=copy.deepcopy(self.base);data["vehicle"][key]=value
            self.assertTrue(gate.reason(data,self.steady,self.wall))
    def test_stopped_failed_or_missing_recorder(self):
        for update in [dict(error="disk error"),dict(closed_cleanly=True),dict(vehicle={})]:
            data=copy.deepcopy(self.base);data.update(update)
            self.assertTrue(gate.reason(data,self.steady,self.wall))
if __name__=="__main__":unittest.main()
