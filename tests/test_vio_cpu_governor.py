import importlib.util,tempfile,unittest
from pathlib import Path
source=Path(__file__).resolve().parents[1]/'nano/sbin/vio_cpu_governor.py';spec=importlib.util.spec_from_file_location('m',source);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class CpuGovernorTest(unittest.TestCase):
 def fixture(self,root):
  for name,g in (('policy0','schedutil'),('policy4','ondemand')):
   p=root/name;p.mkdir();(p/'scaling_governor').write_text(g);(p/'scaling_available_governors').write_text('schedutil ondemand performance');(p/'scaling_min_freq').write_text('729600');(p/'scaling_max_freq').write_text('1344000')
 def test_set_restore(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);self.fixture(root);state=root/'run.json';m.configure('set',root,state);m.configure('set',root,state)
   for p in root.glob('policy*'):self.assertEqual((p/'scaling_governor').read_text(),'performance');self.assertEqual((p/'scaling_max_freq').read_text(),'1344000')
   m.configure('restore',root,state);self.assertEqual((root/'policy0/scaling_governor').read_text(),'schedutil');self.assertEqual((root/'policy4/scaling_governor').read_text(),'ondemand')
 def test_missing_governor(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);self.fixture(root);(root/'policy4/scaling_available_governors').write_text('schedutil')
   with self.assertRaisesRegex(ValueError,'unavailable'):m.configure('set',root,root/'run.json')
if __name__=='__main__':unittest.main()
