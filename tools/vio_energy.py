"""Measured discharge energy; independent of PX4's voltage-based remaining field.

Capacity and initial charge are explicit inputs, never inferred from a short flight.
Missing intervals invalidate a complete budget but retain measured consumption.
"""
import json
import math
from pathlib import Path

def load_battery_profile(path):
    """Nameplate metadata is not a measurement of usable or remaining energy."""
    profile=json.loads(Path(path).read_text(encoding='utf-8'))
    if profile.get('schema_version')!=1:raise ValueError('Unsupported battery profile schema')
    keys=['nominal_voltage_v','nominal_capacity_mah','label_energy_wh']
    for key in keys:
        value=profile.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid battery nameplate field: '+key)
    derived=profile['nominal_voltage_v']*profile['nominal_capacity_mah']/1000.
    return dict(schema_version=1,**{k:profile[k] for k in keys},
                derived_nominal_energy_wh=derived,
                nominal_reference_wh=min(derived,profile['label_energy_wh']),
                nameplate_difference_wh=profile['label_energy_wh']-derived,
                nameplate_consistent=math.isclose(derived,profile['label_energy_wh'],rel_tol=.01),
                usable_capacity_verified=False)

class EnergyAccumulator:
    def __init__(self, battery_id, max_gap_seconds=3.):
        if not battery_id or not math.isfinite(max_gap_seconds) or max_gap_seconds<=0:
            raise ValueError('battery_id and a positive gap limit are required')
        self.battery_id=battery_id
        self.max_gap_seconds=float(max_gap_seconds)
        self.consumed_wh=0.
        self.discharged_mah=0.
        self.coverage_complete=True
        self.unmeasured_intervals=0
        self.samples=0
        self.last_source_us=None
        self.last_received_s=None
        self.last_power_w=None
        self.last_current_a=None
        self.latest_valid=False

    def update(self, source_us, received_s, voltage_v, current_a, connected=True):
        if not isinstance(source_us,int) or source_us<=0 or not math.isfinite(received_s):
            self._missing();return
        if source_us==self.last_source_us:
            # A replayed packet does not extend the freshness deadline.
            return
        valid=connected and all(math.isfinite(x) for x in [voltage_v,current_a]) and voltage_v>0 and current_a>=0
        power=voltage_v*current_a if valid else None
        if valid and not math.isfinite(power):valid=False
        if self.last_received_s is not None and received_s<=self.last_received_s:valid=False
        dt_source=None if self.last_source_us is None else (source_us-self.last_source_us)/1e6
        dt_receive=None if self.last_received_s is None else received_s-self.last_received_s
        continuous=valid and self.last_power_w is not None and dt_source is not None and dt_receive is not None
        if continuous:
            continuous=0<dt_source<=self.max_gap_seconds and 0<dt_receive<=self.max_gap_seconds and abs(dt_source-dt_receive)<=.5
        if continuous:
            self.consumed_wh+=(self.last_power_w+power)*.5*dt_source/3600.
            self.discharged_mah+=(self.last_current_a+current_a)*.5*dt_source/3.6
        elif self.last_source_us is not None or not valid:
            self.coverage_complete=False;self.unmeasured_intervals+=1
        self.last_source_us=source_us;self.last_received_s=received_s
        self.last_power_w=power if valid else None;self.last_current_a=current_a if valid else None
        self.latest_valid=valid;self.samples+=1

    def _missing(self):
        self.coverage_complete=False;self.unmeasured_intervals+=1;self.latest_valid=False
        self.last_power_w=None;self.last_current_a=None

    def snapshot(self, now_s, initial_usable_wh=None, next_task_wh=None, reserve_wh=None):
        for value in [initial_usable_wh,next_task_wh,reserve_wh]:
            if value is not None and (not math.isfinite(value) or value<0):raise ValueError('Energy budgets must be finite and nonnegative')
        fresh=self.last_received_s is not None and 0<=now_s-self.last_received_s<=self.max_gap_seconds
        measured_valid=self.latest_valid and fresh
        remaining=None
        sufficient=None
        if not measured_valid:reason='battery_measurement_unavailable'
        elif not self.coverage_complete:reason='incomplete_energy_coverage'
        elif initial_usable_wh is None:reason='initial_usable_energy_unknown'
        else:
            remaining=max(0.,initial_usable_wh-self.consumed_wh)
            if next_task_wh is None or reserve_wh is None:reason='task_or_reserve_budget_unknown'
            else:
                sufficient=remaining>=next_task_wh+reserve_wh
                reason='energy_budget_sufficient' if sufficient else 'insufficient_energy_budget'
        return dict(schema_version=1,battery_id=self.battery_id,consumed_wh=self.consumed_wh,
                    discharged_mah=self.discharged_mah,power_w=self.last_power_w if measured_valid else None,
                    coverage_complete=self.coverage_complete,measurement_valid=measured_valid,
                    unmeasured_intervals=self.unmeasured_intervals,samples=self.samples,
                    remaining_usable_wh=remaining,next_task_wh=next_task_wh,reserve_wh=reserve_wh,
                    energy_sufficient=sufficient,reason=reason)

    def save(self, path, boot_id):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        temp=path.with_name(path.name+'.tmp')
        temp.write_text(json.dumps(dict(version=1,boot_id=boot_id,state=self.__dict__),allow_nan=False),encoding='utf-8')
        temp.replace(path)

    @classmethod
    def load(cls, path, battery_id, boot_id):
        saved=json.loads(Path(path).read_text(encoding='utf-8'))
        if saved['version']!=1 or saved['state']['battery_id']!=battery_id:
            raise ValueError('Journal does not belong to this battery/session; choose a new journal explicitly')
        state=saved['state']
        result=cls(battery_id,state['max_gap_seconds'])
        for key in result.__dict__:
            if key in state:setattr(result,key,state[key])
        if not math.isfinite(result.consumed_wh) or result.consumed_wh<0 or not math.isfinite(result.discharged_mah) or result.discharged_mah<0:
            raise ValueError('Corrupt energy journal')
        if saved['boot_id']!=boot_id:
            result._missing();result.last_received_s=None
        return result
