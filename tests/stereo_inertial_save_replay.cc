// A deterministic stereo + IMU scene drives the real Tracking/LocalMapping/
// LoopClosing threads. No camera, GUI, downloads or prebuilt maps are needed.
#include "System.h"
#include <opencv2/imgproc.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace ORB_SLAM3;
namespace fs = std::filesystem;
static void require(bool condition, const char* message) {
    if(!condition) throw std::runtime_error(message);
}
static Eigen::Vector3f position(double t) {
    return {float(0.6*sin(1.3*t)+0.015*exp(-t)*sin(18*t)),float(0.3*sin(1.8*t)),float(0.25*sin(0.9*t))};
}
static Eigen::Vector3f acceleration(double t) {
    return {float(-0.6*1.3*1.3*sin(1.3*t)+0.015*exp(-t)*(-323*sin(18*t)-36*cos(18*t))),
            float(-0.3*1.8*1.8*sin(1.8*t)),float(-0.25*0.9*0.9*sin(0.9*t))};
}
static Sophus::SO3f orientation(double t) {
    return Sophus::SO3f::exp(Eigen::Vector3f(float(0.08*sin(1.1*t)),0,0)) *
           Sophus::SO3f::exp(Eigen::Vector3f(0,float(0.15*sin(0.8*t)),0));
}
struct Plane {
    cv::Mat texture;
    std::vector<Eigen::Vector3f> corners;
};
static cv::Mat render(const std::vector<Plane>& planes, double t, bool right) {
    cv::Mat im(480,640,CV_8U,cv::Scalar(40));
    const Eigen::Matrix3f Rwc=orientation(t).matrix();
    const Eigen::Vector3f center=position(t)+Rwc*Eigen::Vector3f(right?0.05f:0,0,0);
    const std::vector<cv::Point2f> source{{0,0},{511,0},{511,511},{0,511}};
    for(const auto& plane:planes) {
        std::vector<cv::Point2f> target;
        for(const auto& point:plane.corners) {
            const Eigen::Vector3f c=Rwc.transpose()*(point-center);
            target.emplace_back(400*c.x()/c.z()+320,400*c.y()/c.z()+240);
        }
        cv::Mat H=cv::getPerspectiveTransform(source,target), warped, mask;
        cv::warpPerspective(plane.texture,warped,H,im.size());
        cv::warpPerspective(cv::Mat(512,512,CV_8U,cv::Scalar(255)),mask,H,im.size(),cv::INTER_NEAREST);
        warped.copyTo(im,mask);
    }
    return im;
}
static std::string bytes(const fs::path& path) {
    std::ifstream in(path,std::ios::binary);
    require(bool(in),"Cannot read saved archive");
    return std::string(std::istreambuf_iterator<char>(in),{});
}
int main(int argc,char** argv) {
    if(argc!=3) { std::cerr<<"Usage: stereo_inertial_save_replay vocabulary output_directory\n"; return 2; }
    cv::setNumThreads(2);
    const fs::path directory=argv[2]; fs::create_directories(directory);
    const auto settings=directory/"synthetic.yaml";
    std::ofstream(settings) << R"(%YAML:1.0
File.version: "1.0"
Camera.type: "Rectified"
Camera1.fx: 400.0
Camera1.fy: 400.0
Camera1.cx: 320.0
Camera1.cy: 240.0
Camera.width: 640
Camera.height: 480
Camera.fps: 30
Camera.RGB: 0
Stereo.b: 0.05
Stereo.ThDepth: 40.0
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
IMU.InsertKFsWhenLost: 0
IMU.NoiseGyro: 0.001
IMU.NoiseAcc: 0.01
IMU.GyroWalk: 0.000001
IMU.AccWalk: 0.0001
IMU.Frequency: 200.0
ORBextractor.nFeatures: 1500
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
)";
    cv::RNG rng(4321);
    std::vector<Plane> planes;
    const std::vector<std::vector<Eigen::Vector3f>> corners{
        {{-4,-3,5},{4,-3,5},{4,3,5},{-4,3,5}},
        {{-2,-1.4,3},{-0.25,-1.4,3},{-0.25,1.2,3},{-2,1.2,3}},
        {{0.3,-0.9,2.6},{1.7,-0.9,2.6},{1.7,1.5,2.6},{0.3,1.5,2.6}}
    };
    for(const auto& c:corners) {
        cv::Mat low(128,128,CV_8U), texture;
        rng.fill(low,cv::RNG::UNIFORM,0,255);
        cv::resize(low,texture,cv::Size(512,512),0,0,cv::INTER_LINEAR);
        planes.push_back({texture,c});
    }
    System slam(argv[1],settings.string(),System::IMU_STEREO,false);
    int saves=0, lastRequest=-100, imuIndex=0, failures=0;
    bool pending=false, testingFailure=false, failureTested=false;
    const std::string saveBase=(directory/"first_map").string();
    System::AtlasSaveStatus finalSave;
    int navigationSamples=0, velocitySamples=0;
    uint64_t lastNavigationEpoch=0;
    for(int frame=0;frame<1500;++frame) {
        const auto started=std::chrono::steady_clock::now();
        double t=frame/30.0;
        std::vector<IMU::Point> imu;
        while(imuIndex/200.0<=t) {
            const double ti=imuIndex/200.0;
            auto R=orientation(ti);
            auto a=R.inverse()*(acceleration(ti)-Eigen::Vector3f(0,9.81f,0));
            const Eigen::Vector3f w=(orientation(ti-0.0001).inverse()*orientation(ti+0.0001)).log()/0.0002f;
            imu.emplace_back(a.x(),a.y(),a.z(),w.x(),w.y(),w.z(),ti+1.0); ++imuIndex;
        }
        slam.TrackStereo(render(planes,t,false),render(planes,t,true),t+1.0,imu);
        const auto nav=slam.GetNavigationState();
        if(nav.valid) {
            ++navigationSamples;
            require(nav.timestamp==t+1.0 && nav.trackingState==Tracking::OK,"Navigation snapshot is stale or not tracking");
            require(nav.referenceEpoch>0 && nav.referenceEpoch>=lastNavigationEpoch,"Navigation reference epoch regressed");
            lastNavigationEpoch=nav.referenceEpoch;
            require(nav.cameraToWorld.matrix().allFinite() && nav.imuToWorld.matrix().allFinite(),"Navigation pose is not finite");
            require(nav.initialImuToWorld.log().norm()<1e-4f,"Initial IMU frame drifted under optimization");
            require((nav.cameraToWorld.inverse()*nav.imuToWorld).log().norm()<1e-4f,"Identity synthetic camera/IMU extrinsics changed direction");
            require((slam.GetCurrentImuPose().inverse()*nav.imuToWorld).log().norm()<1e-5f,"Navigation pose getter disagrees with snapshot");
            if(nav.inertialInitialized) {
                require(nav.gravityWorld.allFinite() && std::abs(nav.gravityWorld.norm()-IMU::GRAVITY_VALUE)<1e-3f,"Navigation gravity rotation changed its norm");
                if(nav.velocityValid) {
                    require(nav.velocityWorld.allFinite(),"Navigation velocity is not finite");
                    ++velocitySamples;
                }
            }
        }
        if(pending) {
            const auto status=slam.GetAtlasSaveStatus();
            if(status.state==System::AtlasSaveState::SUCCEEDED || status.state==System::AtlasSaveState::FAILED) {
                pending=false;
                if(testingFailure) {
                    require(status.state==System::AtlasSaveState::FAILED,"Invalid path reported success");
                    failureTested=true; testingFailure=false;
                } else if(status.state==System::AtlasSaveState::SUCCEEDED) {
                    ++saves; finalSave=status;
                    std::cout<<"REPLAY_SAVE_OK "<<saves<<" map="<<status.mapId<<" KFs="<<status.keyFrames<<" MPs="<<status.mapPoints<<std::endl;
                } else { ++failures; }
            }
        }
        if(frame>=600 && !pending && frame-lastRequest>=30 && saves<5) {
            testingFailure=(saves>=1 && !failureTested);
            pending=slam.RequestAtlasSave(testingFailure?(directory/"missing"/"archive").string():saveBase);
            require(pending,"Save request was rejected unexpectedly");
            require(!slam.RequestAtlasSave(saveBase),"Duplicate save request must be rejected");
            lastRequest=frame;
        }
        if(saves>=5 && frame-lastRequest>60) break;
        std::this_thread::sleep_until(started+std::chrono::milliseconds(33));
    }
    const auto saved= saves>0 ? bytes(saveBase+".osa") : std::string();
    require(navigationSamples>100 && velocitySamples>100,"Navigation fields were not exercised after inertial initialization");
    require(slam.GetRecoveryStatus().stage==System::RecoveryStage::FIRST_SAVED,"Successful online save did not advance recovery");
    require(slam.StartNewMap(),"Saved map could not start recovery reset");
    require(slam.GetRecoveryStatus().stage==System::RecoveryStage::RESETTING && !slam.StartNewMap(),"Reset queue accepted duplicate action");
    const cv::Mat blank(480,640,CV_8U,cv::Scalar(0));
    slam.TrackStereo(blank,blank,60.0);
    require(slam.GetRecoveryStatus().stage==System::RecoveryStage::SECOND_MAPPING,"Full reset did not enter second mapping");
    require(!slam.GetNavigationState().valid,"Full reset retained the previous navigation pose");
    require(!slam.RequestLoadFirstMap(),"Uninitialized second map accepted reference load");
    require(slam.ConfirmSecondInitialization(),"Second-map initialization check could not be queued");
    slam.TrackStereo(blank,blank,60.033333333);
    require(slam.GetRecoveryStatus().stage==System::RecoveryStage::SECOND_MAPPING,"Uninitialized second map incorrectly became ready");
    slam.Shutdown();
    std::cout<<"NAVIGATION_RECOVERY_PASS snapshots="<<navigationSamples<<" velocities="<<velocitySamples<<std::endl;
    // Shutdown's documented contract is to wait for all map writers.
    require(saves==5,"Replay failed to complete five post-VIBA saves");
    require(failureTested,"Write-failure path was not exercised");
    require(saved==bytes(saveBase+".osa"),"Shutdown overwrote the first map archive");
    std::ifstream in(saveBase+".osa",std::ios::binary);
    Atlas* loaded=nullptr; std::string vocName,checksum;
    { boost::archive::binary_iarchive archive(in); archive>>vocName>>checksum>>loaded; }
    ORBVocabulary voc; require(voc.loadFromTextFile(argv[1]),"Vocabulary load failed");
    KeyFrameDatabase db(voc);
    loaded->SetKeyFrameDababase(&db); loaded->SetORBVocabulary(&voc); loaded->PostLoad();
    auto maps=loaded->GetAllMaps();
    auto it=std::find_if(maps.begin(),maps.end(),[&](Map* map){return map->GetId()==finalSave.mapId;});
    require(it!=maps.end(),"Saved active map missing on readback");
    require((*it)->KeyFramesInMap()==finalSave.keyFrames,"Readback keyframe count mismatch");
    require((*it)->MapPointsInMap()==finalSave.mapPoints,"Readback point count mismatch");
    require((*it)->GetIniertialBA2(),"Readback lost VIBA 2 state");
    for(auto* key:(*it)->GetAllKeyFrames()) {
        require(key->GetPose().matrix().allFinite(),"Readback pose invalid");
        require(key->mpImuPreintegrated!=nullptr,"Readback IMU data missing");
    }
    delete loaded;
    std::cout<<"PASS: five online saves after VIBA 2, failure/retry, tracking resumed, archive readback; early attempts="<<failures<<std::endl;
}
