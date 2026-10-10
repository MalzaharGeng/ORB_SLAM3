// Replay the two recorded initialization segments without requiring the camera.
#include "System.h"
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
 if(argc!=5){std::cerr<<"Usage: d435i_recovery_replay vocabulary settings export_directory output_directory\n";return 2;}
 cv::setNumThreads(2);fs::path dir=argv[3],output=argv[4];fs::create_directories(output);
 auto gyros=motion(dir/"gyro.csv"),accels=motion(dir/"accel.csv");
 std::ifstream in(dir/"stereo.csv");std::string line;std::getline(in,line);std::vector<ImagePair> frames;
 while(std::getline(in,line)){auto r=split(line);frames.push_back({std::stod(r[0]),r[1],r[2]});}
 if(frames.empty()||gyros.empty()||accels.empty())throw std::runtime_error("Empty capture");
 const double origin=frames.front().t;
 System slam(argv[1],argv[2],System::IMU_STEREO,std::getenv("ORB_REPLAY_VIEWER")!=nullptr);
 std::ofstream trace(output/"trace.csv");trace<<"t,state,map,keyframes,points,imu,ba2,save_stage\n";
 double resetAt=85.0; std::ifstream(dir/"reset_at.txt")>>resetAt;
 size_t gi=0,ai=0;bool firstSaved=false,reset=false,secondReady=false,loaded=false,pending=false,reportWritten=false;
 double lastRequest=-1;int stage=0;
 const int stride=std::getenv("ORB_REPLAY_STRIDE")?std::max(1,std::atoi(std::getenv("ORB_REPLAY_STRIDE"))):1;
 std::cout<<"REPLAY_FRAME_STRIDE "<<stride<<" (all intervening IMU retained)"<<std::endl;
 for(size_t i=0;i<frames.size();i+=stride){
  auto start=std::chrono::steady_clock::now();const auto& frame=frames[i];double t=frame.t-origin;
  if(t>=resetAt && !firstSaved){
   std::cerr<<"REPLAY_FIRST_INIT_INCOMPLETE: VIBA 2 and save did not finish within the first capture; refusing to use the second capture for first initialization"<<std::endl;
   slam.Shutdown();return 1;
  }
  if(t>=resetAt && firstSaved && !reset){std::cout<<"REQUEST_RESET t="<<t<<std::endl;reset=slam.StartNewMap();stage=2;}
  std::vector<IMU::Point> measures;
  while(gi<gyros.size() && gyros[gi].t<=frame.t){
   auto g=gyros[gi++];if(g.t<origin-0.1)continue;while(ai+1<accels.size()&&accels[ai+1].t<=g.t)++ai;
   const Eigen::Vector3f a=accels[ai].v;
   measures.emplace_back(a.x(),a.y(),a.z(),g.v.x(),g.v.y(),g.v.z(),g.t-origin+1);
  }
  slam.TrackStereo(cv::imread((dir/frame.left).string(),0),cv::imread((dir/frame.right).string(),0),t+1,measures);
  Map* map=nullptr;for(auto* point:slam.GetTrackedMapPoints())if(point&&!point->isBad()){map=point->GetMap();if(map)break;}
  if(pending){auto status=slam.GetAtlasSaveStatus();if(status.state==System::AtlasSaveState::SUCCEEDED){pending=false;if(!firstSaved){firstSaved=true;stage=1;std::cout<<"FIRST_SAVED t="<<t<<std::endl;}else {secondReady=true;stage=3;std::cout<<"SECOND_READY t="<<t<<std::endl;}}else if(status.state==System::AtlasSaveState::FAILED)pending=false;}
  if(t>=42 && !firstSaved && !pending && t-lastRequest>=1 && map && map->GetIniertialBA2()){pending=slam.RequestAtlasSave((output/"first").string());lastRequest=t;}
  if(t>=100 && reset && !secondReady && t-lastRequest>=1 && map && map->GetIniertialBA2()) {
   slam.ConfirmSecondInitialization();lastRequest=t;
  }
  if(slam.GetRecoveryStatus().stage==System::RecoveryStage::SECOND_READY){secondReady=true;stage=3;}
  if(secondReady && !loaded){std::cout<<"REQUEST_READ t="<<t<<std::endl;loaded=slam.RequestLoadFirstMap();stage=4;}
  const auto recovery=slam.GetRecoveryStatus();
  if(recovery.stage==System::RecoveryStage::ALIGNED){
   stage=5;
   if(!reportWritten){
    const auto& r=recovery.report;
    if(!r.valid || !std::isfinite(r.confidence) || r.confidence<0 || r.confidence>100 ||
       (slam.GetCurrentPose().translation()-r.cameraInFirst).norm()>1e-4f)
     throw std::runtime_error("Invalid recovery report or camera offset differs from public pose");
    std::ofstream out(output/"recovery_report.csv");
    out<<"timestamp,camera_x_m,camera_y_m,camera_z_m,camera_distance_m,map2_origin_x_m,map2_origin_y_m,map2_origin_z_m,map2_origin_distance_m,confidence_0_100,pnp_inliers,pairs,candidate_pairs,pair_ratio,rmse_m,translation_delta_m,rotation_delta_rad,confirmations\n";
    out<<std::setprecision(10)<<r.timestamp<<","<<r.cameraInFirst.x()<<","<<r.cameraInFirst.y()<<","<<r.cameraInFirst.z()<<","<<r.cameraDistance<<","<<r.secondOriginInFirst.x()<<","<<r.secondOriginInFirst.y()<<","<<r.secondOriginInFirst.z()<<","<<r.originDistance<<","<<r.confidence<<","<<r.match.inliers<<","<<r.match.pairs<<","<<r.match.candidatePairs<<","<<r.pairRatio<<","<<r.match.rmse<<","<<r.translationDisagreement<<","<<r.rotationDisagreement<<","<<r.confirmations<<"\n";
    out.close();if(!out)throw std::runtime_error("Cannot write recovery report CSV");
    reportWritten=true;
   }
  }
  trace<<t<<","<<slam.GetTrackingState()<<","<<(map?long(map->GetId()):-1)<<","<<(map?map->KeyFramesInMap():0)<<","<<(map?map->MapPointsInMap():0)<<","<<(map&&map->isImuInitialized())<<","<<(map&&map->GetIniertialBA2())<<","<<stage<<"\n";trace.flush();
  if(i%150==0)std::cout<<"REPLAY t="<<t<<" tracking="<<slam.GetTrackingState()<<" stage="<<stage<<std::endl;
  std::this_thread::sleep_until(start+std::chrono::milliseconds(33*stride));
 }
 std::cout<<"REPLAY_END first="<<firstSaved<<" second="<<secondReady<<" read_requested="<<loaded<<std::endl;
 const auto recovered=slam.GetRecoveryStatus();
 slam.Shutdown();
 if(recovered.stage!=System::RecoveryStage::ALIGNED) {std::cerr<<"RECOVERY_NOT_ALIGNED: "<<recovered.message<<std::endl;return 1;}
 if(!reportWritten || !recovered.report.valid)throw std::runtime_error("No verified recovery report");
 std::cout<<"RECOVERY_REPLAY_PASS inliers="<<recovered.inliers<<" pairs="<<recovered.mapPairs<<" residual="<<recovered.residual<<std::endl;
}
