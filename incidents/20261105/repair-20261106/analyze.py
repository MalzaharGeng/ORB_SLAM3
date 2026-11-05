"""Read-only reproduction of the recorded fault; tolerate and count torn CSV tails."""
import csv
from collections import Counter
import io
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent

def main():
    path = PROJECT / '2026.11.5/runs/19700101_080351_live_Tm7tjC/px4_telemetry.csv'
    raw = path.read_bytes()
    rows, invalid = [], 0
    for r in csv.DictReader(io.StringIO(raw.decode('utf-8', errors='replace'))):
        try:
            assert None not in r and all(v is not None for v in r.values())
            assert math.isfinite(float(r['system_us'])) and r['reason']
        except (ValueError, TypeError, AssertionError):
            invalid += 1
            continue
        rows.append(r)
    t0 = float(rows[0]['system_us'])
    def t(r): return (float(r['system_us']) - t0) / 1e6
    armed = [r for r in rows if r['armed'] == '1']
    def stats(key):
        a = sorted(abs(float(r[key])) for r in armed if math.isfinite(float(r[key])))
        return dict(p50=a[len(a)//2], p99=a[int((len(a)-1)*.99)], maximum=a[-1])
    transitions = [{**{k:r[k] for k in ['reason','sample_us','sent','rejected','armed','sync_rtt_us','sync_residual_us']}, 't_seconds': t(r)}
                   for i,r in enumerate(rows) if (i==0 or r['reason']!=rows[i-1]['reason']) and 302<t(r)<306]
    gap = next(i for i,r in enumerate(rows) if r['reason']=='observation_gap')
    last_pub = next(r for r in reversed(rows[:gap]) if r['event']=='publish')
    result = dict(valid_rows=len(rows), malformed_rows=invalid, nul_bytes=raw.count(b'\0'),
                  armed_reasons=dict(Counter(r['reason'] for r in armed)),
                  sync_metrics={k:stats(k) for k in ['sync_rtt_us','sync_residual_us','sync_age_ms']},
                  fault_transitions=transitions,
                  accepted_sample_gap_seconds=(int(rows[gap]['sample_us'])-int(last_pub['sample_us']))/1e6,
                  old_sync_rejections=sum(r['reason']=='time_sync_unready' for r in armed),
                  new_sync_quality_rejections=sum(float(r['sync_age_ms'])>2000 or float(r['sync_age_ms'])<0 or float(r['sync_rtt_us'])>50000 or abs(float(r['sync_residual_us']))>10000 for r in armed))
    # Inspect raw ULogs rather than infer battery cause from the written narrative.
    from pyulog import ULog
    batteries = {}
    for name in ['13_47_11.ulg','14_07_14.ulg']:
        u = ULog(str(PROJECT/name), message_name_filter_list=['battery_status'])
        b = u.get_dataset('battery_status').data
        batteries[name] = {k:dict(min=float(min(b[k])), max=float(max(b[k])))
                           for k in ['voltage_v','remaining','warning'] if k in b}
        batteries[name]['parameters'] = {k:v for k,v in u.initial_parameters.items() if k in ['BAT_LOW_THR','BAT_CRIT_THR','BAT_EMERGEN_THR','COM_LOW_BAT_ACT','BAT1_N_CELLS','BAT1_V_EMPTY','BAT1_V_CHARGED']}
    result['ulog_battery'] = batteries
    (ROOT/'analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))

if __name__ == '__main__': main()
