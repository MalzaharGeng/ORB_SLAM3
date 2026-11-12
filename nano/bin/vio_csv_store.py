"""Bounded CSV segments with explicit retention loss; no ROS or control actions."""
import csv
import json
import os
from pathlib import Path
import re
import shutil
import time

MARKER = 'VIO_ROLLING_CSV_V1'
PART = re.compile(r'^segment_[0-9]{9}\.csv$')


class RollingCsvStore:
    def __init__(self, path, columns, segment_bytes=128*1024**2,
                 budget_bytes=8*1024**3, min_free_bytes=2*1024**3):
        if segment_bytes <= 0 or budget_bytes < 2*segment_bytes or min_free_bytes < 0:
            raise ValueError('Invalid CSV storage bounds')
        self.path = Path(path).absolute()
        self.columns = columns
        self.segment_bytes = segment_bytes
        self.budget_bytes = budget_bytes
        self.min_free_bytes = min_free_bytes
        self.parts = self.path.with_name(self.path.name+'.parts')
        self.parts.mkdir(exist_ok=False)
        (self.parts/'format.txt').write_text(MARKER+'\n')
        self.sequence = self.rows = self.sealed_rows = self.evicted_rows = 0
        self.sealed = []
        self.closed = False
        # Also enforce the shared budget on short sessions that never rotate.
        self._prune(reserve=self.segment_bytes)
        self._open()

    def _open(self):
        self.file = self.path.open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, self.columns)
        self.writer.writeheader()
        self.file.flush()

    def writeheader(self):
        # Inrec's existing constructor calls writeheader; it is already present.
        pass

    def writerow(self, row):
        if self.rows % 200 == 0:
            if shutil.disk_usage(self.path.parent).free < self.min_free_bytes:
                self._prune(reserve=self.segment_bytes)
                if shutil.disk_usage(self.path.parent).free < self.min_free_bytes:
                    raise OSError('State recording free-space reserve reached')
            if self.file.tell() >= self.segment_bytes:
                self._rotate()
        self.writer.writerow(row)
        self.rows += 1

    def _rotate(self):
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        self.sequence += 1
        target = self.parts/f'segment_{self.sequence:09d}.csv'
        self.path.rename(target)
        metadata = target.with_suffix('.json')
        metadata.write_text(json.dumps({'rows':self.rows, 'bytes':target.stat().st_size,
                                       'sealed_time_ns':time.time_ns()}))
        self.sealed.append((target.name, self.rows))
        self.sealed_rows += self.rows
        self.rows = 0
        self._open()
        self._prune(reserve=self.segment_bytes)

    def _prune(self, reserve):
        # Only marked, immediate child stores and sealed chunks can be evicted.
        # The stable current CSV path, unmarked old data, and pinned stores stay.
        candidates = []
        total = 0
        root = self.path.parent.resolve()
        for directory in root.glob('*.csv.parts'):
            if directory.is_symlink() or not directory.is_dir():
                continue
            marker = directory/'format.txt'
            if marker.is_symlink() or not marker.is_file() or marker.read_text().strip() != MARKER:
                continue
            pinned = (directory/'PIN').exists()
            for part in directory.iterdir():
                if not PART.fullmatch(part.name) or part.is_symlink() or not part.is_file():
                    continue
                meta = part.with_suffix('.json')
                if meta.is_symlink() or not meta.is_file():
                    continue
                size = part.stat().st_size
                total += size
                if not pinned:
                    candidates.append((part.stat().st_mtime_ns,part,meta,size,directory))
            closed_meta=directory/'closed_head.json'
            head=root/directory.name[:-6]
            if (closed_meta.is_file() and not closed_meta.is_symlink()
                    and head.is_file() and not head.is_symlink()):
                size=head.stat().st_size;total+=size
                if not pinned:candidates.append((head.stat().st_mtime_ns,head,closed_meta,size,directory))
        for _,part,meta,size,owner in sorted(candidates):
            enough_space = shutil.disk_usage(root).free >= self.min_free_bytes
            if total+reserve <= self.budget_bytes and enough_space:
                break
            if owner.resolve().parent != root or part.parent.resolve() not in (owner.resolve(),root):
                raise ValueError('Invalid rolling CSV eviction path')
            info = json.loads(meta.read_text())
            # Store an eviction receipt before releasing any recorded bytes.
            receipt = owner/'evicted.jsonl'
            with receipt.open('a',encoding='utf-8') as log:
                log.write(json.dumps({'file':part.name,'rows':info['rows'],'bytes':size,
                                      'reason':'rolling_budget','time_ns':time.time_ns()})+'\n')
                log.flush();os.fsync(log.fileno())
            part.unlink();meta.unlink();total -= size
            if owner == self.parts:
                self.evicted_rows += info['rows']
                self.sealed = [(name,rows) for name,rows in self.sealed if name != part.name]
        if total+reserve > self.budget_bytes:
            raise OSError('State recording budget exhausted by pinned data')

    def status(self):
        return {'mode':'rolling','current_path':str(self.path),'segments_directory':str(self.parts),
                'segment_bytes':self.segment_bytes,'managed_sealed_budget_bytes':self.budget_bytes,
                'min_free_bytes':self.min_free_bytes,'current_rows':self.rows,
                'retained_sealed_rows':sum(rows for _,rows in self.sealed),
                'evicted_rows_this_process':self.evicted_rows,
                'retention_notice':'writer_complete covers successful writes; old sealed segments may be evicted. See per-store evicted.jsonl. Historical unmarked data is not deleted.'}

    def flush(self):
        self.file.flush()

    def fileno(self):
        return self.file.fileno()

    def close(self):
        if self.closed:
            return
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        (self.parts/'closed_head.json').write_text(json.dumps({'rows':self.rows,
                    'bytes':self.path.stat().st_size,'closed_time_ns':time.time_ns()}))
        self.closed = True
