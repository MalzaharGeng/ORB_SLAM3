"""Synthetic independent visual/gyro measurements check delay sign and rotation recovery."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
import numpy as np
from scipy.spatial.transform import Rotation
from decxin_calibration_fit import fit_rotation_delay
def main():
    rng=np.random.default_rng(43)
    t=np.arange(0,90,.005)
    g=np.column_stack([.35*np.sin(t*1.3)+.1*np.cos(t*.51),.3*np.sin(t*.83)+.12*np.cos(t*1.7),.4*np.sin(t*1.07)+.08*np.cos(t*.61)])
    integral=np.vstack([np.zeros(3),np.cumsum((g[1:]+g[:-1])*.5*np.diff(t)[:,None],axis=0)])
    nominal=np.array([[0,0,1],[1,0,0],[0,1,0]],float)
    exact=Rotation.from_rotvec([.012,-.018,.023])*Rotation.from_matrix(nominal)
    offset=-.047;bias=np.array([.001,-.002,.0015])
    pairs=[]
    for a in np.arange(1,88,.11):
        b=a+.11
        v=np.array([np.interp(b+offset,t,integral[:,j])-np.interp(a+offset,t,integral[:,j]) for j in range(3)])
        visual=exact.inv().apply(v)
        visual+=rng.normal(0,.00005,3)
        pairs.append([a,b,*visual])
    result=fit_rotation_delay(pairs,np.column_stack([t,g+bias]),nominal)
    assert abs(result["image_time_offset_seconds"]-offset)<.003,result
    delta=(Rotation.from_matrix(result["R_body_rectified_camera"]).inv()*exact).magnitude()
    assert np.rad2deg(delta)<.15,result
    assert result["candidate_checks_pass"] and not result["calibration_verified"],result
    print("PASS known -47ms offset sign, 3-axis mounting rotation, gyro bias, temporal holdout, candidate remains unverified")
if __name__=="__main__":main()
