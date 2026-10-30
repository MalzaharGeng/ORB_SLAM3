// Replay the two recorded initialization segments without requiring the camera.
#include "System.h"
#include "Examples/Utils/RecoveryConsole.h"
#ifdef ORB_SLAM3_WITH_PX4
#include "Px4Bridge.h"
#endif
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace ORB_SLAM3;
namespace fs=std::filesystem;
struct Motion {double t;Eigen::Vector3f v;};
struct ImagePair {double t;std::string left,right;};
static std::vector<std::string> split(std::string s){std::vector<std::string> r;std::stringstream in(s);while(std::getline(in,s,','))r.push_back(s);return r;}
static std::vector<Motion> motion(fs::path file){std::ifstream in(file);std::string line;std::getline(in,line);std::vector<Motion> rows;while(std::getline(in,line)){auto r=split(line);rows.push_back({std::stod(r[0]),{std::stof(r[1]),std::stof(r[2]),std::stof(r[3])}});}return rows;}
int main(int argc,char** argv){
 if(argc!=5 && !(argc==7 && std::string(argv[5])=="--px4-config")){std::cerr<<"Usage: d435i_recovery_replay vocabulary settings export_directory output_directory [--px4-config FILE]\n";return 2;}
#ifdef ORB_SLAM3_WITH_PX4
 std::unique_ptr<px4::Bridge> px4Bridge;
 if(argc==7) {
  auto config=px4::Config::Load(argv[6]);
  // Replay can never write to the hardware namespace. Its peer is the test subscriber.
  config.topicPrefix="/vio_test";config.domainId=42;
  px4Bridge=std::make_unique<px4::Bridge>(config);
 }
#else
 if(argc==7) {std::cerr<<"PX4 support was not built\n";return 2;}
#endif
 cv::setNumThreads(2);fs::path dir=argv[3],output=argv[4];fs::create_directories(output);
 auto gyros=motion(dir/"gyro.csv"),accels=motion(dir/"accel.csv");
 std::ifstream in(dir/"stereo.csv");std::string line;std::getline(in,line);std::vector<ImagePair> frames;
 while(std::getline(in,line)){auto r=split(line);frames.push_back({std::stod(r[0]),r[1],r[2]});}
 if(frames.empty()||gyros.empty()||accels.empty())throw std::runtime_error("Empty capture");
 const double origin=frames.front().t;
 const auto startupBegin=std::chrono::steady_clock::now();
 System slam(argv[1],argv[2],System::IMU_STEREO,std::getenv("ORB_REPLAY_VIEWER")!=nullptr);
 const auto startupEnd=std::chrono::steady_clock::now();
 std::ofstream metadata(output/"timing_metadata.csv");
 metadata<<"startup_ms,origin_sensor_time,viewer,unpaced\n"<<std::setprecision(17)
         <<std::chrono::duration<double,std::milli>(startupEnd-startupBegin).count()<<","<<origin<<","
         <<(std::getenv("ORB_REPLAY_VIEWER")!=nullptr)<<","<<(std::getenv("ORB_REPLAY_UNPACED")!=nullptr)<<"\n";
 std::ofstream timing(output/"timing.csv");
 timing<<"index,t,wall_start_s,wall_end_s,read_ms,track_ms,processing_ms,wait_ms,cycle_ms,state,map,anchor,imu,ba1,ba2,save_state_before,recovery_stage_before,recovery_stage_after\n";
 timing<<std::setprecision(12);
 RecoveryConsole console(slam,(output/"first").string(),false);
 const bool terminal=std::getenv("ORB_REPLAY_CONSOLE")!=nullptr;
 if(terminal) console.Help();
 std::ofstream trace(output/"trace.csv");trace<<"t,state,map,keyframes,points,imu,ba2,save_stage\n";
 std::ofstream navigationTrace(output/"navigation.csv");
 navigationTrace<<"t,map,anchor,first_map_frame,imu_x,imu_y,imu_z,imu_qw,imu_qx,imu_qy,imu_qz,camera_x,camera_y,camera_z,raw_imu_x,raw_imu_y,raw_imu_z,origin_translation_norm,origin_rotation_norm\n";
 size_t navigationSamples=0;
 double resetAt=85.0; std::ifstream(dir/"reset_at.txt")>>resetAt;
 size_t gi=0,ai=0;bool firstSaved=false,reset=false,secondReady=false,loaded=false,pending=false,reportWritten=false;
 double lastRequest=-1;int stage=0;
 const int stride=std::getenv("ORB_REPLAY_STRIDE")?std::max(1,std::atoi(std::getenv("ORB_REPLAY_STRIDE"))):1;
 std::cout<<"REPLAY_FRAME_STRIDE "<<stride<<" (all intervening IMU retained)"<<std::endl;
 for(size_t i=0;i<frames.size();i+=stride){
  if(terminal && !console.Poll()) break;
  auto start=std::chrono::steady_clock::now();const auto& frame=frames[i];double t=frame.t-origin;
  if(t>=resetAt && !firstSaved){
   std::cerr<<"REPLAY_FIRST_INIT_INCOMPLETE: VIBA 2 and save did not finish within the first capture; refusing to use the second capture for first initialization"<<std::endl;
   slam.Shutdown();return 1;
  }
  if(t>=resetAt && firstSaved && !reset){std::cout<<"REQUEST_RESET t="<<t<<std::endl;reset=console.Execute("reset");stage=2;}
  std::vector<IMU::Point> measures;
  while(gi<gyros.size() && gyros[gi].t<=frame.t){
   auto g=gyros[gi++];if(g.t<origin-0.1)continue;while(ai+1<accels.size()&&accels[ai+1].t<=g.t)++ai;
   const Eigen::Vector3f a=accels[ai].v;
   measures.emplace_back(a.x(),a.y(),a.z(),g.v.x(),g.v.y(),g.v.z(),g.t-origin+1);
  }
  const auto saveBefore=slam.GetAtlasSaveStatus();const auto recoveryBefore=slam.GetRecoveryStatus();
  const auto readStart=std::chrono::steady_clock::now();
  const auto left=cv::imread((dir/frame.left).string(),0),right=cv::imread((dir/frame.right).string(),0);
  if(left.empty() || right.empty()) throw std::runtime_error("Cannot decode replay stereo pair");
  const auto trackStart=std::chrono::steady_clock::now();
#ifdef ORB_SLAM3_WITH_PX4
  // Rebase each playback frame to its actual injection time; do not send historical epoch timestamps.
  const auto px4SampleTime=px4::SystemMicros();
#endif
  slam.TrackStereo(left,right,t+1,measures);
  const auto trackEnd=std::chrono::steady_clock::now();
  const auto nav=slam.GetNavigationState();
#ifdef ORB_SLAM3_WITH_PX4
  if(px4Bridge)px4Bridge->Submit(nav,px4SampleTime);
#endif
  if(nav.valid){
   if(!nav.inFirstMap && nav.initialImuToWorld.log().norm()>1e-4f)
    throw std::runtime_error("Initial IMU world origin changed during optimization");
   const auto p=nav.imuToWorld.translation();const auto q=nav.imuToWorld.unit_quaternion();
   const auto c=nav.cameraToWorld.translation();const auto raw=(nav.mapToWorld.inverse()*nav.imuToWorld).translation();
   navigationTrace<<std::setprecision(10)<<t<<","<<nav.mapId<<","<<nav.initialKeyFrameId<<","<<nav.inFirstMap<<","<<p.x()<<","<<p.y()<<","<<p.z()<<","<<q.w()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<c.x()<<","<<c.y()<<","<<c.z()<<","<<raw.x()<<","<<raw.y()<<","<<raw.z()<<","<<nav.initialImuToWorld.translation().norm()<<","<<nav.initialImuToWorld.so3().log().norm()<<"\n";
   ++navigationSamples;
  }
  Map* map=nullptr;for(auto* point:slam.GetTrackedMapPoints())if(point&&!point->isBad()){map=point->GetMap();if(map)break;}
  if(pending){auto status=slam.GetAtlasSaveStatus();if(status.state==System::AtlasSaveState::SUCCEEDED){pending=false;if(!firstSaved){firstSaved=true;stage=1;std::cout<<"FIRST_SAVED t="<<t<<std::endl;}else {secondReady=true;stage=3;std::cout<<"SECOND_READY t="<<t<<std::endl;}}else if(status.state==System::AtlasSaveState::FAILED)pending=false;}
  if(t>=42 && !firstSaved && !pending && t-lastRequest>=1 && map && map->GetIniertialBA2()){pending=console.Execute("first");lastRequest=t;}
  if(t>=100 && reset && !secondReady && t-lastRequest>=1 && map && map->GetIniertialBA2()) {
   console.Execute("second");lastRequest=t;
  }
  if(slam.GetRecoveryStatus().stage==System::RecoveryStage::SECOND_READY){secondReady=true;stage=3;}
  if(secondReady && !loaded){std::cout<<"REQUEST_READ t="<<t<<std::endl;loaded=console.Execute("load");stage=4;}
  if(terminal) console.PrintChanges();
  const auto recovery=slam.GetRecoveryStatus();
  if(recovery.stage==System::RecoveryStage::ALIGNED){
   stage=5;
   if(!reportWritten){
    const auto& r=recovery.report;
    if(!r.valid || !std::isfinite(r.confidence) || r.confidence<0 || r.confidence>100 ||
       (slam.GetCurrentPose().translation()-r.cameraInFirst).norm()>1e-4f ||
       (slam.GetCurrentImuPose().translation()-r.imuInFirst).norm()>1e-4f)
     throw std::runtime_error("Invalid recovery report or camera offset differs from public pose");
    std::ofstream out(output/"recovery_report.csv");
    out<<"timestamp,camera_x_m,camera_y_m,camera_z_m,camera_distance_m,map2_origin_x_m,map2_origin_y_m,map2_origin_z_m,map2_origin_distance_m,confidence_0_100,pnp_inliers,pairs,candidate_pairs,pair_ratio,rmse_m,translation_delta_m,rotation_delta_rad,confirmations,imu_x_m,imu_y_m,imu_z_m,imu_distance_m\n";
    out<<std::setprecision(10)<<r.timestamp<<","<<r.cameraInFirst.x()<<","<<r.cameraInFirst.y()<<","<<r.cameraInFirst.z()<<","<<r.cameraDistance<<","<<r.secondOriginInFirst.x()<<","<<r.secondOriginInFirst.y()<<","<<r.secondOriginInFirst.z()<<","<<r.originDistance<<","<<r.confidence<<","<<r.match.inliers<<","<<r.match.pairs<<","<<r.match.candidatePairs<<","<<r.pairRatio<<","<<r.match.rmse<<","<<r.translationDisagreement<<","<<r.rotationDisagreement<<","<<r.confirmations<<","<<r.imuInFirst.x()<<","<<r.imuInFirst.y()<<","<<r.imuInFirst.z()<<","<<r.imuDistance<<"\n";
    out.close();if(!out)throw std::runtime_error("Cannot write recovery report CSV");
    reportWritten=true;
   }
  }
  trace<<t<<","<<slam.GetTrackingState()<<","<<(map?long(map->GetId()):-1)<<","<<(map?map->KeyFramesInMap():0)<<","<<(map?map->MapPointsInMap():0)<<","<<(map&&map->isImuInitialized())<<","<<(map&&map->GetIniertialBA2())<<","<<stage<<"\n";trace.flush();
  if(i%150==0)std::cout<<"REPLAY t="<<t<<" tracking="<<slam.GetTrackingState()<<" stage="<<stage<<std::endl;
  const bool imuReady=map&&map->isImuInitialized(),ba1Ready=map&&map->GetIniertialBA1(),ba2Ready=map&&map->GetIniertialBA2();
  const auto workEnd=std::chrono::steady_clock::now();
  // No frame skipping: preserve all intervening IMU. Pace by the recorded
  // sensor interval, without catching up by dropping frames after slow work.
  if(!std::getenv("ORB_REPLAY_UNPACED") && i+stride<frames.size())
   std::this_thread::sleep_until(start+std::chrono::duration<double>(frames[i+stride].t-frame.t));
  const auto cycleEnd=std::chrono::steady_clock::now();
  const auto ms=[](auto duration){return std::chrono::duration<double,std::milli>(duration).count();};
  timing<<i<<","<<t<<","<<std::chrono::duration<double>(start.time_since_epoch()).count()<<","<<std::chrono::duration<double>(cycleEnd.time_since_epoch()).count()
        <<","<<ms(trackStart-readStart)<<","<<ms(trackEnd-trackStart)<<","<<ms(workEnd-start)<<","<<ms(cycleEnd-workEnd)<<","<<ms(cycleEnd-start)
        <<","<<slam.GetTrackingState()<<","<<(map?long(map->GetId()):-1)<<","<<(nav.valid?long(nav.initialKeyFrameId):-1)
        <<","<<imuReady<<","<<ba1Ready<<","<<ba2Ready<<","<<int(saveBefore.state)<<","<<int(recoveryBefore.stage)<<","<<int(recovery.stage)<<"\n";
 }
 std::cout<<"REPLAY_END first="<<firstSaved<<" second="<<secondReady<<" read_requested="<<loaded<<std::endl;
 const auto recovered=slam.GetRecoveryStatus();
 slam.Shutdown();
 if(console.QuitRequested()) {std::cout<<"REPLAY_STOPPED_BY_USER"<<std::endl;return 0;}
 if(recovered.stage!=System::RecoveryStage::ALIGNED) {std::cerr<<"RECOVERY_NOT_ALIGNED: "<<recovered.message<<std::endl;return 1;}
 if(!reportWritten || !recovered.report.valid)throw std::runtime_error("No verified recovery report");
 if(!navigationSamples)throw std::runtime_error("No navigation pose snapshots");
 std::cout<<"NAVIGATION_FRAME_PASS samples="<<navigationSamples<<std::endl;
 std::cout<<"RECOVERY_REPLAY_PASS inliers="<<recovered.inliers<<" pairs="<<recovered.mapPairs<<" residual="<<recovered.residual<<std::endl;
}
