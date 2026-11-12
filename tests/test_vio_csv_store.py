"""Check raw CSV rotation, active-path stability, retention receipts and isolation."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

source=Path(__file__).resolve().parents[1]/'nano/bin/vio_csv_store.py'
spec=importlib.util.spec_from_file_location('vio_csv_store',source)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class CsvStoreTest(unittest.TestCase):
    def test_rotation_keeps_current_path_and_complete_rows(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'state.csv'
            store=module.RollingCsvStore(path,['index','payload'],segment_bytes=200,budget_bytes=100000,min_free_bytes=0)
            for i in range(620):store.writerow({'index':i,'payload':'raw,quoted\nvalue'})
            store.flush();store.close()
            files=sorted(store.parts.glob('segment_*.csv'))+[path]
            rows=[]
            for p in files:
                with p.open(newline='') as f:rows.extend(csv.DictReader(f))
            self.assertEqual([int(r['index']) for r in rows],list(range(620)))
            self.assertTrue(all(r['payload']=='raw,quoted\nvalue' for r in rows))
            self.assertEqual(store.status()['evicted_rows_this_process'],0)

    def test_budget_only_removes_managed_sealed_files(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);unrelated=root/'old.csv';unrelated.write_text('precious')
            store=module.RollingCsvStore(root/'state.csv',['index','payload'],segment_bytes=200,budget_bytes=1000,min_free_bytes=0)
            for i in range(620):store.writerow({'index':i,'payload':'x'*12})
            store.flush();store.close()
            self.assertTrue(store.path.exists())
            self.assertEqual(unrelated.read_text(),'precious')
            self.assertGreater(store.status()['evicted_rows_this_process'],0)
            self.assertTrue((store.parts/'evicted.jsonl').is_file())
            self.assertLessEqual(sum(p.stat().st_size for p in store.parts.glob('segment_*.csv')),1000)

    def test_pin_prevents_eviction_and_budget_error_is_explicit(self):
        with tempfile.TemporaryDirectory() as temp:
            store=module.RollingCsvStore(Path(temp)/'state.csv',['index','payload'],segment_bytes=200,budget_bytes=1000,min_free_bytes=0)
            (store.parts/'PIN').write_text('retain fault evidence')
            try:
                with self.assertRaisesRegex(OSError,'pinned'):
                    for i in range(620):store.writerow({'index':i,'payload':'x'*12})
                self.assertTrue(next(store.parts.glob('segment_*.csv')).exists())
            finally:store.close()

    def test_closed_head_from_previous_session_is_managed(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            first=module.RollingCsvStore(root/'old.csv',['index','payload'],segment_bytes=200,budget_bytes=1000,min_free_bytes=0)
            for i in range(30):first.writerow({'index':i,'payload':'x'*100})
            first.close()
            second=module.RollingCsvStore(root/'new.csv',['index','payload'],segment_bytes=200,budget_bytes=1000,min_free_bytes=0)
            try:
                self.assertFalse((root/'old.csv').exists())
                for i in range(220):second.writerow({'index':i,'payload':'y'*12})
                self.assertFalse((root/'old.csv').exists())
                self.assertTrue((first.parts/'evicted.jsonl').exists())
                self.assertTrue(second.path.exists())
            finally:second.close()


if __name__=='__main__':unittest.main()
