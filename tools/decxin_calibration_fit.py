#!/usr/bin/env python3
"""Estimate body-from-camera rotation and gyro/image delay using independent stereo PnP.
Candidate calibration only; never changes live configs or authorizes flight.
"""
import argparse,csv,json,time
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
def fit_rotation_delay(pairs,imu,nominal):
    pairs=np.asarray(pairs,float);imu=np.asarray(imu,float)
    times=imu[:,0];gyro=imu[:,1:4]
    integ=np.vstack([np.zeros(3),np.cumsum((gyro[1:]+gyro[:-1])*.5*np.diff(times)[:,None],axis=0)])
    def vectors(offset,sel):
        a=pairs[sel,0]+offset;b=pairs[sel,1]+offset
        if a.min()<times[0] or b.max()>times[-1]:raise ValueError("IMU coverage")
        return np.column_stack([np.interp(b,times,integ[:,j])-np.interp(a,times,integ[:,j]) for j in range(3)])
    dt=pairs[:,1]-pairs[:,0];visual=pairs[:,2:5]
    # Hold out whole temporal blocks, not random adjacent samples.
    block=np.floor((pairs[:,0]-pairs[0,0])/2).astype(int);hold=(block%4)==3;train=~hold
    if hold.sum()<10 or train.sum()<30:raise ValueError("Too few independent temporal validation samples")
    def solve(mask):
        grid=[]
        for offset in np.linspace(-.2,.2,161):
            g=vectors(offset,mask)
            r,_=Rotation.align_vectors(g,visual[mask])
            residual=(g-r.apply(visual[mask]))/dt[mask,None]
            grid.append((float(np.sqrt(np.mean(residual**2))),float(offset),r.as_rotvec()))
        _,offset,rv=min(grid,key=lambda x:x[0])
        def residual(x):
            r=Rotation.from_rotvec(x[:3]);pred=r.apply(visual[mask])
            return ((vectors(x[3],mask)-pred)/dt[mask,None]-x[4:7]).ravel()
        solved=least_squares(residual,np.r_[rv,offset,np.zeros(3)],
            bounds=(np.r_[[-np.inf]*3,-.2,[-.05]*3],np.r_[[np.inf]*3,.2,[.05]*3]),
            loss="soft_l1",f_scale=.03,max_nfev=100)
        return solved,grid
    solved,grid=solve(train)
    r=Rotation.from_rotvec(solved.x[:3]);offset=float(solved.x[3]);bias=solved.x[4:]
    def metrics(mask):
        g=vectors(offset,mask);v=r.apply(visual[mask]);res=(g-v)/dt[mask,None]-bias
        return {"pairs":int(mask.sum()),"rate_rmse_rad_s":float(np.sqrt(np.mean(res**2))),
                "angle_rms_deg":float(np.rad2deg(np.sqrt(np.mean(np.sum((res*dt[mask,None])**2,axis=1))))),
                "speed_rms_rad_s":float(np.sqrt(np.mean(np.sum((visual[mask]/dt[mask,None])**2,axis=1))))}
    other,_=solve(hold)
    differences=Rotation.from_rotvec(other.x[:3]).inv()*r
    covariance=np.linalg.eigvalsh((visual/dt[:,None]).T@(visual/dt[:,None])/len(visual))
    nominal_delta=float(np.rad2deg((Rotation.from_matrix(nominal).inv()*r).magnitude()))
    result={"image_time_offset_seconds":offset,"R_body_rectified_camera":r.as_matrix().tolist(),
        "gyro_bias_rad_s":bias.tolist(),"train":metrics(train),"holdout":metrics(hold),
        "independent_offset_seconds":float(other.x[3]),
        "offset_block_disagreement_ms":float(abs(other.x[3]-offset)*1000),
        "rotation_block_disagreement_deg":float(np.rad2deg(differences.magnitude())),
        "rotation_from_nominal_deg":nominal_delta,
        "excitation_rate_covariance_eigenvalues":covariance.tolist(),
        "grid":[{"offset":x[1],"rmse":x[0]} for x in grid]}
    result["candidate_checks_pass"]=bool(abs(offset)<.18 and nominal_delta<10 and
        result["holdout"]["angle_rms_deg"]<.5 and result["holdout"]["rate_rmse_rad_s"]<.08 and
        result["offset_block_disagreement_ms"]<15 and result["rotation_block_disagreement_deg"]<3 and
        covariance[0]>.003)
    result["calibration_verified"]=False
    return result
def main():
    ap=argparse.ArgumentParser();ap.add_argument("directory");ap.add_argument("--runtime",required=True)
    ap.add_argument("--nominal",required=True);ap.add_argument("--max-pairs",type=int,default=700)
    ap.add_argument("--analyze-interrupted-prefix",action="store_true")
    args=ap.parse_args();root=Path(args.directory);cv2.setNumThreads(2)
    summary=json.loads((root/"summary.json").read_text())
    if not summary["complete"]:
        if not args.analyze_interrupted_prefix or summary["errors"]!=["Disarmed status lost"]:
            raise ValueError("Incomplete capture")
        print("DIAGNOSTIC ONLY: interrupted capture, retain only complete IMU-covered intervals",flush=True)
    if summary["imu_nonpositive"] or summary["counts"]["imu_invalid"]:raise ValueError("Invalid IMU samples")
    frame=list(csv.DictReader((root/"frames.csv").open()))
    imu=np.genfromtxt(root/"imu.csv",delimiter=",",names=True)
    origin=float(imu["timestamp_us"][0])*1e-6
    sample=np.column_stack([imu["timestamp_us"]*1e-6-origin,imu["gx"],imu["gy"],imu["gz"]])
    gap_indices=np.where(np.diff(sample[:,0])>.035)[0]
    gaps=[(sample[i,0],sample[i+1,0]) for i in gap_indices]
    fs=cv2.FileStorage(args.runtime,cv2.FILE_STORAGE_READ)
    k=fs.getNode("P1").mat()[:3,:3];p2=fs.getNode("P2").mat();baseline=-p2[0,3]/p2[0,0]
    size=(1280,960)
    maps=[]
    for eye in [1,2]:
        maps.append(cv2.initUndistortRectifyMap(fs.getNode(f"K{eye}").mat(),fs.getNode(f"D{eye}").mat(),
            fs.getNode(f"R{eye}").mat(),fs.getNode(f"P{eye}").mat(),size,cv2.CV_16SC2))
    orb=cv2.ORB_create(nfeatures=2500,fastThreshold=12)
    matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
    def match(a,b,ratio=.72):
        if a is None or b is None or min(len(a),len(b))<2:return []
        return [x[0] for x in matcher.knnMatch(a,b,k=2) if len(x)==2 and x[0].distance<ratio*x[1].distance]
    def features(row):
        im=cv2.imread(str(root/row["file"]),cv2.IMREAD_GRAYSCALE)
        if im is None or im.shape!=(960,2720):return None
        left=cv2.remap(im[:,160:1440],*maps[0],cv2.INTER_LINEAR)
        right=cv2.remap(im[:,1440:2720],*maps[1],cv2.INTER_LINEAR)
        kl,dl=orb.detectAndCompute(left,None);kr,dr=orb.detectAndCompute(right,None)
        points={}
        for m in match(dl,dr,.75):
            u,v=kl[m.queryIdx].pt;ur,vr=kr[m.trainIdx].pt;disparity=u-ur
            if abs(v-vr)>1.5 or not 2<disparity<150:continue
            z=k[0,0]*baseline/disparity
            if not .3<z<12:continue
            points[m.queryIdx]=[(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z]
        return kl,dl,points
    cache=root/"visual_pairs.json"
    if cache.exists():pairs=json.loads(cache.read_text())
    else:
        pairs=[];previous=None
        for idx,row in enumerate(frame):
            current=features(row)
            if current is None:
                print("SKIP_BAD_JPEG",row["index"],flush=True);previous=None;continue
            if previous is not None:
                prevrow,(kp,desc,pts)=previous
                obj=[];img=[]
                for m in match(desc,current[1]):
                    if m.queryIdx in pts:obj.append(pts[m.queryIdx]);img.append(current[0][m.trainIdx].pt)
                dt=(int(row["timestamp_us"])-int(prevrow["timestamp_us"]))*1e-6
                if len(obj)>=35 and .04<dt<.3:
                    obj=np.array(obj,np.float64);img=np.array(img,np.float64)
                    ok,rv,tv,inliers=cv2.solvePnPRansac(obj,img,k,None,iterationsCount=100,reprojectionError=2.,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
                    if ok and inliers is not None and len(inliers)>=25 and len(inliers)/len(obj)>.5:
                        ix=inliers[:,0];rv,tv=cv2.solvePnPRefineLM(obj[ix],img[ix],k,None,rv,tv)
                        proj=cv2.projectPoints(obj[ix],rv,tv,k,None)[0][:,0]
                        reproj=float(np.sqrt(np.mean(np.sum((proj-img[ix])**2,axis=1))))
                        # PnP is current-from-previous; camera orientation evolution is its inverse.
                        visual=-rv[:,0]
                        if reproj<1.2 and np.linalg.norm(visual)<.25:
                            pairs.append([int(prevrow["timestamp_us"])*1e-6-origin,int(row["timestamp_us"])*1e-6-origin,*visual.tolist(),reproj,int(len(ix))])
            previous=row,current
            if idx%25==0:print("STEREO_PNP",idx,"valid_pairs",len(pairs),flush=True)
            if len(pairs)>=args.max_pairs:break
        cache.write_text(json.dumps(pairs))
    data=np.array(pairs)
    if len(data)<80:raise ValueError(f"Too few valid stereo PnP intervals: {len(data)}")
    # Offset search needs 0.2 seconds of IMU margin around each visual interval.
    data=data[(data[:,0]>.25)&(data[:,1]<sample[-1,0]-.25)]
    before_gap_filter=len(data)
    for a,b in gaps:
        data=data[~((data[:,0]-.2<b)&(data[:,1]+.2>a))]
    if len(data)<80:raise ValueError("Too few intervals after IMU gap exclusion")
    nominal=np.array(json.loads(Path(args.nominal).read_text())["T_body_rectified_camera"])[:3,:3]
    result=fit_rotation_delay(data[:,:5],sample,nominal)
    result["capture_complete"]=summary["complete"]
    result["diagnostic_only"]=not summary["complete"]
    result["imu_gaps_seconds"]=[list(g) for g in gaps];result["discarded_gap_intervals"]=before_gap_filter-len(data)
    result["stereo_pairs"]=len(data);result["median_pnp_reprojection_px"]=float(np.median(data[:,5]))
    (root/"calibration_candidate.json").write_text(json.dumps(result,indent=2))
    display=dict(result);display.pop("grid");print(json.dumps(display,indent=2),flush=True)
if __name__=="__main__":main()
