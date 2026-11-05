"""Find sustained manual inputs, compare body-relative velocity and acceleration."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog

ROOT=Path(__file__).resolve().parent
u=ULog(str(ROOT.parent/'14_07_14.ulg'))
def data(n):return u.get_dataset(n).data
def times(d):return (d['timestamp'].astype('int64')-u.start_timestamp)/1e6
grid=np.arange(3.,41.5,.02)
def at(d,k):return np.interp(grid,times(d),d[k].astype(float))
def hold(d,k):return d[k][np.clip(np.searchsorted(times(d),grid,side='right')-1,0,len(d[k])-1)].astype(float)
a=data('vehicle_attitude');p=data('vehicle_local_position');m=data('manual_control_setpoint');sp=data('vehicle_attitude_setpoint')
def quat_matrix(d,prefix):
    q=np.column_stack([at(d,f'{prefix}[{i}]') for i in range(4)])
    q/=np.linalg.norm(q,axis=1)[:,None]
    w,x,y,z=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
                     2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
                     2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)
R=quat_matrix(a,'q');Rd=quat_matrix(sp,'q_d')
v=np.column_stack([at(p,k) for k in ['vx','vy','vz']])
heading=np.arctan2(R[:,1,0],R[:,0,0])
H=np.zeros_like(R);H[:,0,0]=H[:,1,1]=np.cos(heading)
H[:,1,0]=np.sin(heading);H[:,0,1]=-np.sin(heading);H[:,2,2]=1.
# Project horizontal navigation velocity onto heading-forward/right axes.
# Do not mix vertical descent into the pilot's horizontal left/right motion.
vb=np.einsum('nji,nj->ni',H,v)
acc=np.column_stack([at(p,k) for k in ['ax','ay','az']])
ab=np.einsum('nji,nj->ni',H,acc)
implied=-9.80665*R[:,:,2]/R[:,2,2,None];implied[:,2]+=9.80665
accel_expected_body=np.einsum('nji,nj->ni',H,implied)
roll=np.rad2deg(np.arctan2(R[:,2,1],R[:,2,2]));pitch=np.rad2deg(np.arcsin(np.clip(-R[:,2,0],-1,1)))
rollsp=np.rad2deg(np.arctan2(Rd[:,2,1],Rd[:,2,2]));pitchsp=np.rad2deg(np.arcsin(np.clip(-Rd[:,2,0],-1,1)))
rc=np.column_stack([hold(m,k) for k in ['roll','pitch','yaw','throttle']])
segments=[]
for channel,axis,angles,targets in [(0,1,roll,rollsp),(1,0,-pitch,-pitchsp)]:
    signs=np.where(np.abs(rc[:,channel])>=.15,np.sign(rc[:,channel]),0)
    edges=np.r_[0,np.where(np.diff(signs)!=0)[0]+1,len(signs)]
    for start,end in zip(edges[:-1],edges[1:]):
        if not signs[start] or (end-start)*.02<.3:continue
        ids=np.arange(start,end);sgn=signs[start]
        segments.append(dict(channel=['roll','pitch'][channel],start=round(float(grid[start]),3),end=round(float(grid[end-1]+.02),3),
            duration=round((end-start)*.02,3),rc_mean=float(np.mean(rc[ids,channel])),
            actual_tilt_mean_deg=float(np.mean(angles[ids])),target_tilt_mean_deg=float(np.mean(targets[ids])),
            velocity_body_start=float(vb[start,axis]),velocity_body_end=float(vb[end-1,axis]),
            fraction_velocity_opposite=float(np.mean(sgn*vb[ids,axis]<-.1)),
            delta_velocity_toward_command=float(sgn*(vb[end-1,axis]-vb[start,axis])),
            measured_acceleration_toward_command=float(np.mean(sgn*ab[ids,axis])),
            tilt_implied_acceleration_toward_command=float(np.mean(sgn*accel_expected_body[ids,axis]))))
result={'note':'roll + is body right; pitch + requests forward tilt in this log. Velocity is EKF estimate, not independent truth.',
        'segments':sorted(segments,key=lambda s:s['start'])}
(ROOT/'opposite_motion_segments.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
for s in result['segments']:
    print(s['channel'],s['start'],s['end'],'rc',round(s['rc_mean'],2),'tilt/sp',round(s['actual_tilt_mean_deg'],1),round(s['target_tilt_mean_deg'],1),
          'v start/end',round(s['velocity_body_start'],2),round(s['velocity_body_end'],2),'opposite%',round(s['fraction_velocity_opposite']*100),
          'dv toward',round(s['delta_velocity_toward_command'],2),'a toward',round(s['measured_acceleration_toward_command'],2))
print('longest',sorted(segments,key=lambda s:-s['duration'])[:3])
