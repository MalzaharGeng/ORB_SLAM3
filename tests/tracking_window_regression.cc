#include "System.h"
#include "Tracking.h"
#include "Atlas.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "Pinhole.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <cstdlib>
namespace ORB_SLAM3 {
struct FlightMapRegression {
    static void Run(System& slam,bool enforce) {
        auto* tracker=slam.mpTracker;auto* map=slam.mpAtlas->GetCurrentMap();
        cv::Mat left(480,640,CV_8UC1),right(480,640,CV_8UC1,cv::Scalar(0));
        cv::randu(left,0,255);left(cv::Rect(8,0,632,480)).copyTo(right(cv::Rect(0,0,632,480)));
        cv::Mat K=(cv::Mat_<float>(3,3)<<400,0,320,0,400,240,0,0,1),dist=cv::Mat::zeros(4,1,CV_32F);
        ORBextractor le(800,1.2,8,20,7),re(800,1.2,8,20,7);
        auto* camera=slam.mpAtlas->AddCamera(new Pinhole({400,400,320,240}));
        IMU::Calib calib(Sophus::SE3f(),.001,.01,.0001,.001);IMU::Preintegrated pre(IMU::Bias(),calib);
        Frame frame(left,right,1.,&le,&re,slam.mpVocabulary,K,dist,20,40,camera,nullptr,calib);
        frame.SetPose(Sophus::SE3f());frame.mpImuPreintegrated=&pre;frame.mnDataset=0;
        std::vector<KeyFrame*> keys;std::vector<MapPoint*> points;
        for(int count:{32,128,512}) {
            while(int(keys.size())<count) {
                frame.mnId++;frame.mTimeStamp+=.5;
                auto* key=new KeyFrame(frame,map,slam.mpKeyFrameDatabase);map->AddKeyFrame(key);
                if(!keys.empty()) {key->mPrevKF=keys.back();keys.back()->mNextKF=key;}
                keys.push_back(key);
                if(points.empty())for(int i=0;i<300;++i) {
                    auto* p=new MapPoint(Eigen::Vector3f(.001*i,0,2),key,map);map->AddMapPoint(p);points.push_back(p);
                }
                for(size_t i=0;i<points.size();++i) {key->AddMapPoint(points[i],i);points[i]->AddObservation(key,i);}
            }
            map->SetInertialSensor();map->SetImuInitialized();map->SetIniertialBA2();
            tracker->mCurrentFrame=frame;tracker->mLastFrame=frame;
            tracker->mCurrentFrame.mnId=100000+count;tracker->mnLastRelocFrameId=0;
            tracker->mCurrentFrame.mpLastKeyFrame=keys.back();tracker->mpLastKeyFrame=keys.back();tracker->mpReferenceKF=keys.back();
            tracker->mLastFrame.mvpMapPoints.assign(frame.N,nullptr);
            for(size_t i=0;i<points.size();++i)tracker->mLastFrame.mvpMapPoints[i]=points[i];
            const auto start=std::chrono::steady_clock::now();
            for(int repeat=0;repeat<20;++repeat) {
                ++tracker->mCurrentFrame.mnId;tracker->UpdateLocalKeyFrames();tracker->UpdateLocalPoints();
            }
            const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/20;
            std::cout<<"TRACKING_WINDOW_BENCH total_kfs="<<count<<" local_kfs="<<tracker->mvpLocalKeyFrames.size()<<" points="<<tracker->mvpLocalMapPoints.size()<<" mean_ms="<<ms<<std::endl;
            if(enforce && tracker->mvpLocalKeyFrames.size()>80)throw std::runtime_error("mature local keyframe window exceeded 80");
            if(tracker->mvpLocalMapPoints.size()!=points.size())throw std::runtime_error("bounded window lost shared map points");
            if(map->KeyFramesInMap()!=keys.size())throw std::runtime_error("window restriction erased persistent map");
        }
        // Exercise a dense covisibility graph as well as the temporal-only
        // fixture above. The persistent Atlas still contains all 512 KFs.
        keys.back()->UpdateConnections();tracker->mpReferenceKF=keys.back();
        ++tracker->mCurrentFrame.mnId;
        const auto denseStart=std::chrono::steady_clock::now();
        tracker->UpdateLocalKeyFrames();tracker->UpdateLocalPoints();
        std::cout<<"TRACKING_WINDOW_DENSE total_kfs="<<keys.size()<<" local_kfs="<<tracker->mvpLocalKeyFrames.size()
                 <<" points="<<tracker->mvpLocalMapPoints.size()<<" ms="
                 <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-denseStart).count()<<std::endl;
        if(enforce && tracker->mvpLocalKeyFrames.size()>80)throw std::runtime_error("dense covisibility escaped window bound");
        if(tracker->mvpLocalMapPoints.size()!=points.size())throw std::runtime_error("dense window lost tracked points");

        // After initialization/reset, a geometrically tracked previous frame can
        // have no marginalized inertial prior. Exercise the actual TrackLocalMap
        // dispatch, not a duplicate of its selection condition.
        for(int i=0;i<20;++i)pre.IntegrateNewMeasurement(Eigen::Vector3f(0,0,9.81f),Eigen::Vector3f::Zero(),.005f);
        tracker->mCurrentFrame=frame;tracker->mLastFrame=frame;
        tracker->mCurrentFrame.mnId=200000;tracker->mLastFrame.mnId=199999;
        tracker->mCurrentFrame.mpPrevFrame=&tracker->mLastFrame;
        tracker->mCurrentFrame.mpLastKeyFrame=keys.back();
        keys.back()->SetVelocity(Eigen::Vector3f::Zero());
        tracker->mCurrentFrame.mpImuPreintegrated=&pre;
        tracker->mCurrentFrame.mpImuPreintegratedFrame=&pre;
        tracker->mCurrentFrame.SetImuPoseVelocity(Eigen::Matrix3f::Identity(),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero());
        tracker->mLastFrame.SetImuPoseVelocity(Eigen::Matrix3f::Identity(),Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero());
        tracker->mLastFrame.mpcpi=nullptr;tracker->mCurrentFrame.mpcpi=nullptr;
        tracker->mnLastRelocFrameId=0;tracker->mbMapUpdated=false;
        tracker->mState=Tracking::OK;
        for(size_t i=0;i<points.size();++i) {
            tracker->mCurrentFrame.mvpMapPoints[i]=points[i];
            tracker->mLastFrame.mvpMapPoints[i]=points[i];
        }
        tracker->TrackLocalMap();
        if(!tracker->mCurrentFrame.mpcpi)throw std::runtime_error("missing-prior bootstrap did not establish an inertial constraint");
        std::cout<<"MISSING_INERTIAL_PRIOR_BOOTSTRAP_PASS"<<std::endl;
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc<3)return 2;
        ORB_SLAM3::System slam(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,false);
        ORB_SLAM3::FlightMapRegression::Run(slam,argc>3);slam.Shutdown();
        std::cout<<"TRACKING_WINDOW_REGRESSION_PASS\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
