#include "System.h"
#include "Tracking.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "ORBextractor.h"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
namespace ORB_SLAM3 {
struct LoopProbe : LoopClosing {
    using LoopClosing::LoopClosing;
    bool Pending(){return CheckNewKeyFrames();}
};
struct FlightMapRegression {
    static void Run(System& slam) {
        auto check=[](bool c,const char* m){if(!c)throw std::runtime_error(m);};
        auto* tracker=slam.mpTracker;auto* atlas=slam.mpAtlas;auto* first=atlas->GetCurrentMap();
        // Disabling loop/GBA must retain database insertion and drain its queue.
        cv::Mat left(480,640,CV_8UC1),right(480,640,CV_8UC1,cv::Scalar(0));
        cv::randu(left,0,255);left(cv::Rect(8,0,632,480)).copyTo(right(cv::Rect(0,0,632,480)));
        cv::Mat K=(cv::Mat_<float>(3,3)<<400,0,320,0,400,240,0,0,1),dist=cv::Mat::zeros(4,1,CV_32F);
        ORBextractor le(800,1.2,8,20,7),re(800,1.2,8,20,7);
        auto* camera=atlas->AddCamera(new Pinhole({400,400,320,240}));
        IMU::Calib calib(Sophus::SE3f(),.001,.01,.0001,.001);IMU::Preintegrated pre(IMU::Bias(),calib);
        Frame frame(left,right,1.,&le,&re,slam.mpVocabulary,K,dist,20,40,camera,nullptr,calib);
        frame.SetPose(Sophus::SE3f());frame.mpImuPreintegrated=&pre;frame.mnDataset=0;
        auto* origin=new KeyFrame(frame,first,slam.mpKeyFrameDatabase);first->AddKeyFrame(origin);
        auto* indexed=new KeyFrame(frame,first,slam.mpKeyFrameDatabase);first->AddKeyFrame(indexed);
        LoopProbe noLoop(atlas,slam.mpKeyFrameDatabase,slam.mpVocabulary,true,false);
        noLoop.InsertKeyFrame(indexed);
        std::thread worker([&]{noLoop.Run();});
        for(int i=0;i<100 && noLoop.Pending();++i)usleep(10000);
        noLoop.RequestFinish();worker.join();
        check(!noLoop.Pending(),"no-loop mode retained an unbounded keyframe queue");
        frame.mnId+=1000;frame.ComputeBoW();
        const auto candidates=slam.mpKeyFrameDatabase->DetectRelocalizationCandidates(&frame,first);
        check(!candidates.empty(),"no-loop mode failed to index keyframes for relocalization");
        first->SetInertialSensor();first->SetImuInitialized();first->SetIniertialBA2();
        tracker->mCurrentFrame=frame;
        tracker->mCurrentFrame.mTimeStamp=100.;
        check(tracker->RetainMatureMap(first),"mature map immediately discarded");
        tracker->mCurrentFrame.mTimeStamp=109.9;
        check(tracker->RetainMatureMap(first),"short loss discarded mature map");
        check(atlas->GetCurrentMap()==first && !slam.GetNavigationState().valid,"retention changed origin or published lost pose");
        tracker->mCurrentFrame.mTimeStamp=110.1;
        check(!tracker->RetainMatureMap(first),"complete-loss timeout never expires");
        const auto firstId=first->GetId();
        check(slam.StartNewMap(),"reset required a prior save under flight map policy");
        check(!slam.StartNewMap(),"duplicate queued map reset accepted");
        cv::Mat blank(480,640,CV_8UC1,cv::Scalar(0));
        slam.TrackStereo(blank,blank,111.,{});
        check(atlas->GetCurrentMap()->GetId()!=firstId,"explicit reset did not create a second map");
        bool retained=false;for(auto* map:atlas->GetAllMaps())if(map==first)retained=true;
        check(retained,"explicit reset erased the first Atlas map");
        check(!slam.GetNavigationState().valid,"new uninitialized map published a valid pose");
    }
};
}
int main(int argc,char** argv) {
    if(argc!=3)return 2;
    const auto path=std::filesystem::temp_directory_path()/("flight-map-test-"+std::to_string(getpid())+".yaml");
    try {
        std::ifstream input(argv[2]);std::string text((std::istreambuf_iterator<char>(input)),{});
        const std::string key="Tracking.preserve_mature_map: 0";
        const auto pos=text.find(key);
        if(pos!=std::string::npos)text.replace(pos,key.size(),"Tracking.preserve_mature_map: 1");
        else text+="\nTracking.preserve_mature_map: 1\n";
        std::ofstream(path)<<text;
        ORB_SLAM3::System slam(argv[1],path.string(),ORB_SLAM3::System::IMU_STEREO,false);
        ORB_SLAM3::FlightMapRegression::Run(slam);slam.Shutdown();
        std::filesystem::remove(path);std::cout<<"FLIGHT_MAP_POLICY_REGRESSION_PASS\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
