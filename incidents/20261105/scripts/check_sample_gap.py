#!/usr/bin/env python3
"""检查 px4_telemetry.csv 里 VIO 样本间隔是否出现突变。

15 fps 的帧间隔应为 66.7 ms。若出现数百毫秒的空洞，
会触发 observation_gap（门限 300 ms），进而永久锁存。

用法:
    python check_sample_gap.py <px4_telemetry.csv> [阈值ms]
"""
import sys
import csv
import math


def main(path, thresh_ms=300.0):
    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8", errors="replace")))

    def g(v):
        try:
            x = float(v)
            return x if math.isfinite(x) else None
        except Exception:
            return None

    vals = [(i, g(r["system_us"])) for i, r in enumerate(rows) if g(r["system_us"])]
    t0 = vals[0][1]

    prev_su = None
    gaps = []
    for i, v in vals:
        t = (v - t0) / 1e6
        su = g(rows[i]["sample_us"])
        if su is None or su == prev_su:
            continue
        if prev_su is not None:
            dt = (su - prev_su) / 1e3
            if dt > thresh_ms:
                gaps.append((t, dt))
        prev_su = su

    print(f"阈值 {thresh_ms} ms，检出 {len(gaps)} 处超限间隔")
    for t, dt in gaps[:50]:
        print(f"  t={t:8.2f}s  间隔 {dt:8.1f} ms")
    if len(gaps) > 50:
        print(f"  ... 还有 {len(gaps)-50} 处")

    if gaps:
        print("\n注意：单次超限即触发 motionFault，会置 airborneFault_ 并永久锁存。")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 300.0)
