"""Native-sample reconstruction; nearest samples, no interpolation of discrete states."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog

ROOT=Path(__file__).resolve().parent
u=ULog(str(ROOT.parent/'14_07_14.ulg'))
keys={
    'battery_status':['voltage_v','current_a','remaining','warning'],
    'vehicle_local_position':['z','vz','v_z_valid','xy_valid'],
    'actuator_motors':['control[0]','control[1]','control[2]','control[3]'],
    'vehicle_thrust_setpoint':['xyz[2]'],
    'manual_control_setpoint':['throttle','roll','pitch','yaw'],
    'vehicle_land_detected':['landed','ground_contact','freefall'],
    'vehicle_status':['arming_state','nav_state','failsafe'],
}
data={name:u.get_dataset(name).data for name in keys}
out=[]
for second in [0,2,5,6,7,10,20,30,35,38,39,40,41,42,43,44]:
    row={'ulog_seconds':second}
    for name,d in data.items():
        times=(d['timestamp'].astype('int64')-u.start_timestamp)/1e6
        i=int(np.argmin(abs(times-second)))
        row[name]={k:round(float(d[k][i]),4) for k in keys[name] if k in d}
        row[name]['sample_time_seconds']=round(float(times[i]),6)
    out.append(row)
(ROOT/'final_flight_timeline.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
warnings=data['battery_status']
events={
    'warning_changes':[dict(ulog_seconds=(int(warnings['timestamp'][i])-u.start_timestamp)/1e6,
                            warning=int(warnings['warning'][i]),voltage_v=float(warnings['voltage_v'][i]))
                       for i in range(len(warnings['timestamp'])) if i==0 or warnings['warning'][i]!=warnings['warning'][i-1]],
    'logged_messages':[dict(ulog_seconds=(int(m.timestamp)-u.start_timestamp)/1e6,message=m.message) for m in u.logged_messages],
}
(ROOT/'final_flight_events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
print('FINAL_FLIGHT_TIMELINE_SAVED',len(out))
