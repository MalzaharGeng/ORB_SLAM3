"""Reconstruct the last flight's manual -> setpoint -> attitude/control response."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog

ROOT=Path(__file__).resolve().parent
u=ULog(str(ROOT.parent/'14_07_14.ulg'))
def data(name):return u.get_dataset(name).data
def seconds(d):return (d['timestamp'].astype('int64')-u.start_timestamp)/1e6
def euler(d,prefix):
    w,x,y,z=[d[f'{prefix}[{i}]'].astype(float) for i in range(4)]
    return np.rad2deg(np.column_stack((np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))))
manual=data('manual_control_setpoint');att=data('vehicle_attitude');sp=data('vehicle_attitude_setpoint')
angles=euler(att,'q');targets=euler(sp,'q_d')
datasets={
 'rc':(manual,np.column_stack([manual[k] for k in ['roll','pitch','yaw','throttle']])),
 'att_deg':(att,angles), 'target_deg':(sp,targets),
 'rates_sp_deg_s':(data('vehicle_rates_setpoint'),np.rad2deg(np.column_stack([data('vehicle_rates_setpoint')[k] for k in ['roll','pitch','yaw']]))),
 'rates_deg_s':(data('vehicle_angular_velocity'),np.rad2deg(np.column_stack([data('vehicle_angular_velocity')[f'xyz[{i}]'] for i in range(3)]))),
 'motor':(data('actuator_motors'),np.column_stack([data('actuator_motors')[f'control[{i}]'] for i in range(4)])),
 'lpos':(data('vehicle_local_position'),np.column_stack([data('vehicle_local_position')[k] for k in ['x','y','z','vx','vy','vz','heading']])),
 'lpos_sp':(data('vehicle_local_position_setpoint'),np.column_stack([data('vehicle_local_position_setpoint')[k] for k in ['x','y','z','vx','vy','vz']])),
}
def sample(t):
    row={'ulog_seconds':float(t)}
    for name,(d,a) in datasets.items():
        i=int(np.clip(np.searchsorted(seconds(d),t,side='right')-1,0,len(a)-1))
        row[name]=[round(float(x),4) if np.isfinite(x) else None for x in a[i]]
    return row
grid=[sample(t) for t in np.arange(3,42,.05)]
cm=data('vehicle_control_mode');st=data('vehicle_status');ca=data('control_allocator_status')
result={
 'control_flags':{k:np.unique(v).tolist() for k,v in cm.items() if k.startswith('flag_')},
 'nav_states':np.unique(st['nav_state']).tolist(),
 'rc_range':{k:[float(np.min(manual[k])),float(np.max(manual[k]))] for k in ['roll','pitch','yaw','throttle']},
 'yaw_net_change_3_41_deg':float(np.rad2deg(np.interp(41,seconds(att),np.unwrap(np.deg2rad(angles[:,2])))-np.interp(3,seconds(att),np.unwrap(np.deg2rad(angles[:,2]))))),
 'attitude_resets':np.unique(att['quat_reset_counter']).tolist(),
 'allocation':{k:np.unique(ca[k]).tolist() for k in ['torque_setpoint_achieved','thrust_setpoint_achieved','handled_motor_failure_mask']},
 'snapshots':[sample(t) for t in [3,5,10,15,20,25,30,32,34,35,36,37,38,39,40,41,41.5,42,43]],
}
for axis,i in [('roll',0),('pitch',1)]:
    errors=np.array([abs(r['att_deg'][i]-r['target_deg'][i]) for r in grid])
    result[axis+'_error_deg']={'p50':float(np.percentile(errors,50)),'p95':float(np.percentile(errors,95)),'max':float(np.max(errors))}
    act=[r for r in grid if abs(r['rc'][i])>.15]
    result[axis+'_active_examples']=act[::max(1,len(act)//8)][:8]
(ROOT/'last_control_response.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ['snapshots','roll_active_examples','pitch_active_examples']},indent=2))
for r in result['snapshots']:
    print('t',r['ulog_seconds'],'rc',r['rc'],'target',r['target_deg'],'att',r['att_deg'],'rate_sp',r['rates_sp_deg_s'],'motor',r['motor'])
