import importlib.machinery
import math
from pathlib import Path
import types
import unittest

m=types.ModuleType('clock_policy')
importlib.machinery.SourceFileLoader('clock_policy',str(Path(__file__).resolve().parent/'staged/system/vio-clock-prepare')).exec_module(m)
GOOD=1793894400.

class ClockTest(unittest.TestCase):
    def test_dead_rtc_uses_saved_bound(self):
        self.assertEqual(m.decision(100, 100, GOOD, False), ('saved_lower_bound',GOOD))
    def test_invalid_without_source_fails_closed(self):
        self.assertEqual(m.decision(100,None,None,False),('no_trusted_clock',None))
    def test_running_vio_prevents_repair(self):
        self.assertEqual(m.decision(100,GOOD,None,True),('refuse_running_clock_step',None))
    def test_rtc_rollback_never_rewinds_valid_system(self):
        self.assertEqual(m.decision(GOOD+100,GOOD,None,False),('system_clock',GOOD+100))
    def test_rtc_can_restore(self):
        self.assertEqual(m.decision(100,GOOD,None,False),('rtc',GOOD))
    def test_trusted_system_kept_without_rtc(self):
        self.assertEqual(m.decision(GOOD,None,None,False),('system_clock',GOOD))
    def test_nonfinite_and_corrupt_sources(self):
        for x in [float('nan'),float('inf'),-1,'2026',None,1e20]:
            self.assertEqual(m.decision(100,x,x,False),('no_trusted_clock',None))
    def test_running_consistent_clock_does_not_step(self):
        self.assertEqual(m.decision(GOOD,GOOD+1,None,True),('system_clock',GOOD))

if __name__=='__main__':unittest.main()
