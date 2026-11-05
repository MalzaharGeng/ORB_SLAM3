import math
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vio_energy import EnergyAccumulator, load_battery_profile

class EnergyTest(unittest.TestCase):
    def sample(self,e,second,voltage=24.,current=10.):e.update(int((100+second)*1e6),1000+second,voltage,current)
    def test_constant_power_and_coulombs(self):
        e=EnergyAccumulator('pack');[self.sample(e,t) for t in range(61)]
        self.assertAlmostEqual(e.consumed_wh,4.);self.assertAlmostEqual(e.discharged_mah,166.66666667)
    def test_trapezoid_voltage_current_changes(self):
        e=EnergyAccumulator('pack');self.sample(e,0,24,10);self.sample(e,1,22,20)
        self.assertAlmostEqual(e.consumed_wh,(240+440)/2/3600)
    def test_unknown_capacity_has_no_launch_permission(self):
        e=EnergyAccumulator('pack');self.sample(e,0);self.sample(e,1)
        s=e.snapshot(1001);self.assertIsNone(s['remaining_usable_wh']);self.assertIsNone(s['energy_sufficient'])
    def test_landing_does_not_reset_consumption(self):
        e=EnergyAccumulator('pack');[self.sample(e,t,current=0 if 5<=t<10 else 10) for t in range(16)]
        before=e.consumed_wh;self.sample(e,16);self.assertGreater(e.consumed_wh,before)
    def test_budget_reserves_and_insufficiency(self):
        e=EnergyAccumulator('pack');self.sample(e,0);self.sample(e,1)
        self.assertTrue(e.snapshot(1001,10,4,2)['energy_sufficient']);self.assertFalse(e.snapshot(1001,5,4,2)['energy_sufficient'])
    def test_gap_is_not_interpolated_into_fake_certainty(self):
        e=EnergyAccumulator('pack');self.sample(e,0);self.sample(e,10)
        self.assertEqual(e.consumed_wh,0);self.assertFalse(e.coverage_complete);self.assertIsNone(e.snapshot(1010,10,1,1)['energy_sufficient'])
    def test_invalid_current_and_nan_break_coverage(self):
        for current in [-1.,float('nan')]:
            e=EnergyAccumulator('pack');self.sample(e,0);self.sample(e,1,current=current);self.sample(e,2)
            self.assertFalse(e.coverage_complete);self.assertEqual(e.consumed_wh,0)
    def test_duplicates_do_not_keep_battery_fresh(self):
        e=EnergyAccumulator('pack');self.sample(e,0);e.update(100000000,1005,24,10)
        self.assertFalse(e.snapshot(1005)['measurement_valid'])
    def test_clock_jump_or_source_regression_invalidates_coverage(self):
        e=EnergyAccumulator('pack');self.sample(e,0);e.update(99000000,1001,24,10)
        self.assertFalse(e.coverage_complete)
    def test_restart_and_pack_identity(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'journal.json';e=EnergyAccumulator('pack');self.sample(e,0);self.sample(e,1);e.save(path,'boot')
            restored=EnergyAccumulator.load(path,'pack','boot');self.sample(restored,2)
            self.assertAlmostEqual(restored.consumed_wh,2*240/3600)
            with self.assertRaises(ValueError):EnergyAccumulator.load(path,'new_pack','boot')
            rebooted=EnergyAccumulator.load(path,'pack','new_boot');self.assertFalse(rebooted.coverage_complete)
    def test_missing_reserve_or_task_is_not_permission(self):
        e=EnergyAccumulator('pack');self.sample(e,0);self.assertIsNone(e.snapshot(1000,10,None,2)['energy_sufficient'])
    def test_stale_measurement_revokes_budget(self):
        e=EnergyAccumulator('pack');self.sample(e,0);self.assertFalse(e.snapshot(1005,10,1,1)['measurement_valid'])
    def test_supplied_nameplate_preserves_conflicting_values(self):
        p=load_battery_profile(Path(__file__).resolve().parents[1]/'config/vio_battery.json')
        self.assertEqual(p['label_energy_wh'],129.96)
        self.assertAlmostEqual(p['derived_nominal_energy_wh'],126.54)
        self.assertAlmostEqual(p['nominal_reference_wh'],126.54)
        self.assertAlmostEqual(p['nameplate_difference_wh'],3.42)
        self.assertFalse(p['nameplate_consistent']);self.assertFalse(p['usable_capacity_verified'])
        e=EnergyAccumulator('pack');self.sample(e,0)
        self.assertIsNone(e.snapshot(1000)['energy_sufficient'])
    def test_profile_rejects_invalid_values(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'profile.json'
            for value in [None,True,-1,0,float('inf'),'5700']:
                path.write_text(json.dumps(dict(schema_version=1,nominal_voltage_v=22.2,nominal_capacity_mah=value,label_energy_wh=129.96)))
                with self.assertRaises(ValueError):load_battery_profile(path)

if __name__=='__main__':unittest.main()
