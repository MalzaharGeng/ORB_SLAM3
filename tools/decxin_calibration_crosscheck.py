#!/usr/bin/env python3
"""Independent dataset cross checks. Reports candidate quality; never edits live settings."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
def cross_metrics(source,target):
    s=json.loads((source/"calibration_candidate.json").read_text())
    a=np.genfromtxt(target/"imu.csv",delimiter=",",names=True)
    t=a["timestamp_us"]*1e-6;t-=t[0]
    g=np.column_stack([a["gx"],a["gy"],a["gz"]])
    integ=np.vstack([np.zeros(3),np.cumsum((g[1:]+g[:-1])*.5*np.diff(t)[:,None],axis=0)])
    p=np.array(json.loads((target/"visual_pairs.json").read_text()),float)
    offset=s["image_time_offset_seconds"]
    p=p[(p[:,0]+offset>t[0])&(p[:,1]+offset<t[-1])]
    for i in np.where(np.diff(t)>.035)[0]:
        p=p[~((p[:,0]+offset<t[i+1])&(p[:,1]+offset>t[i]))]
    dt=p[:,1]-p[:,0]
    v=Rotation.from_matrix(s["R_body_rectified_camera"]).apply(p[:,2:5])
    gy=np.column_stack([np.interp(p[:,1]+offset,t,integ[:,j])-np.interp(p[:,0]+offset,t,integ[:,j]) for j in range(3)])
    residual=(gy-v)/dt[:,None]-np.array(s["gyro_bias_rad_s"])
    moving=np.linalg.norm(p[:,2:5]/dt[:,None],axis=1)>.05
    return {"intervals":len(p),"moving_intervals":int(moving.sum()),
        "rate_rmse_rad_s":float(np.sqrt(np.mean(residual**2))),
        "moving_rate_rmse_rad_s":float(np.sqrt(np.mean(residual[moving]**2))),
        "angle_rms_deg":float(np.rad2deg(np.sqrt(np.mean(np.sum((residual*dt[:,None])**2,axis=1))))),
        "angle_p95_deg":float(np.rad2deg(np.percentile(np.linalg.norm(residual*dt[:,None],axis=1),95)))}
def main():
    ap=argparse.ArgumentParser();ap.add_argument("first");ap.add_argument("second");ap.add_argument("--out",required=True);a=ap.parse_args()
    first,second=Path(a.first),Path(a.second)
    x=json.loads((first/"calibration_candidate.json").read_text());y=json.loads((second/"calibration_candidate.json").read_text())
    delta=Rotation.from_matrix(x["R_body_rectified_camera"]).inv()*Rotation.from_matrix(y["R_body_rectified_camera"])
    result={"datasets":[str(first),str(second)],
        "offset_difference_ms":abs(x["image_time_offset_seconds"]-y["image_time_offset_seconds"])*1000,
        "rotation_difference_deg":float(np.rad2deg(delta.magnitude())),
        "first_on_second":cross_metrics(first,second),"second_on_first":cross_metrics(second,first),
        "selected_dataset":str(second),"translation_body_frd_m":[.150,.0205,.0495],
        "translation_source":"user measured; forward/right/down relative to PX4 IMU",
        "capture_profile":{"format":"MJPG","fps":30,"buffers":1,"timestamp":"host immediately after read"},
        "scope":"rotation/time independent cross-validation; translation is measured, not optimized"}
    result["passed"]=bool(x["candidate_checks_pass"] and y["candidate_checks_pass"] and
        x.get("capture_complete",True) and y.get("capture_complete",True) and
        result["offset_difference_ms"]<10 and result["rotation_difference_deg"]<1.5 and
        max(result["first_on_second"]["moving_rate_rmse_rad_s"],result["second_on_first"]["moving_rate_rmse_rad_s"])<.03 and
        max(result["first_on_second"]["angle_p95_deg"],result["second_on_first"]["angle_p95_deg"])<.3)
    Path(a.out).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return 0 if result["passed"] else 1
if __name__=="__main__":raise SystemExit(main())
