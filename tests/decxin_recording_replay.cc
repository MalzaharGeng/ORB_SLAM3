#include "System.h"
#include "Px4ImuBuffer.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <memory>
using namespace ORB_SLAM3;
namespace fs=std::filesystem;
static std::vector<std::string> split(const std::string& line) {
    std::stringstream s(line);std::vector<std::string> result;std::string item;
    while(std::getline(s,item,','))result.push_back(item);
    return result;
}
int main(int argc,char** argv) {
    if(argc!=7) {std::cerr<<"usage: vocab settings runtime_rectify imu_config capture_dir trajectory_output\n";return 2;}
    std::unique_ptr<System> slam;
    try {
        const fs::path root(argv[5]);
        cv::FileStorage rect(argv[3],cv::FileStorage::READ),config(argv[4],cv::FileStorage::READ);
        if(!rect.isOpened() || !config.isOpened())throw std::runtime_error("config missing");
        double offset=double(config["image_time_offset_seconds"]);
        std::vector<px4::ImuReading> readings;
        std::ifstream imu(root/"imu.csv");std::string line;std::getline(imu,line);
        while(std::getline(imu,line)) {
            if(line.empty() || line=="\r")continue;
            auto v=split(line);if(v.size()<15)throw std::runtime_error("bad IMU row");
            px4::ImuReading r;r.time=std::stod(v[0])*1e-6;
            for(int i=0;i<3;++i) {r.accel[i]=std::stof(v[3+i]);r.gyro[i]=std::stof(v[6+i]);}
            if(std::stoi(v[12]))throw std::runtime_error("clipped IMU");
            readings.push_back(r);
        }
        if(readings.size()<100)throw std::runtime_error("not enough IMU");
        const double origin=readings.front().time;
        for(auto& r:readings)r.time-=origin;
        cv::Mat mapX[2],mapY[2];
        for(int i=0;i<2;++i) {
            std::string eye=std::to_string(i+1);cv::Mat k,d,r,p;
            rect["K"+eye]>>k;rect["D"+eye]>>d;rect["R"+eye]>>r;rect["P"+eye]>>p;
            cv::initUndistortRectifyMap(k,d,r,p,cv::Size(1280,960),CV_16SC2,mapX[i],mapY[i]);
        }
        slam=std::make_unique<System>(argv[1],argv[2],System::IMU_STEREO,false);
        px4::ImuBuffer buffer;size_t at=0;double previous=0;uint64_t epoch=0;
        bool everBa2=false;size_t processed=0,bad=0;int previousState=-1;
        std::ifstream frames(root/"frames.csv");std::getline(frames,line);
        std::ofstream nav(fs::path(argv[6]).string()+".nav.csv");
        nav<<"frame,time,valid,imu,ba2,epoch,x,y,z\n";
        while(std::getline(frames,line)) {
            if(line.empty() || line=="\r")continue;
            auto v=split(line);if(v.size()<4)throw std::runtime_error("bad frame row");
            double t=std::stod(v[1])*1e-6-origin+offset;
            cv::Mat image=cv::imread((root/v[3]).string(),cv::IMREAD_GRAYSCALE);
            if(image.empty() || image.size()!=cv::Size(2720,960)) {++bad;continue;}
            while(at<readings.size() && (buffer.data.empty() || buffer.data.back().time<t))buffer.Add(readings[at++]);
            std::vector<px4::ImuReading> batch;std::string reason;
            if(!buffer.Slice(previous>0?previous:t-.01,t,batch,reason)) {
                if(!processed)continue;
                // The recording may stop its IMU a few ms before the final camera frame.
                if(t>readings.back().time)break;
                throw std::runtime_error("incomplete interval: "+reason);
            }
            if(processed && buffer.epoch!=epoch)throw std::runtime_error("IMU discontinuity");
            epoch=buffer.epoch;
            std::vector<IMU::Point> measurements;
            for(size_t i=processed?1:0;i<batch.size();++i) {
                auto& r=batch[i];measurements.emplace_back(r.accel[0],r.accel[1],r.accel[2],r.gyro[0],r.gyro[1],r.gyro[2],r.time);
            }
            cv::Mat left,right;
            cv::remap(image(cv::Rect(160,0,1280,960)),left,mapX[0],mapY[0],cv::INTER_LINEAR);
            cv::remap(image(cv::Rect(1440,0,1280,960)),right,mapX[1],mapY[1],cv::INTER_LINEAR);
            slam->TrackStereo(left,right,t,measurements);previous=t;++processed;
            const auto n=slam->GetNavigationState();const auto p=n.imuToWorld.translation();
            nav<<processed<<","<<t<<","<<n.valid<<","<<n.inertialInitialized<<","<<n.inertialBA2<<","<<n.referenceEpoch<<","<<p.x()<<","<<p.y()<<","<<p.z()<<"\n";
            if(n.valid && n.inertialInitialized && n.inertialBA2)everBa2=true;
            if(processed%25==0 || n.trackingState!=previousState) {
                std::cout<<"[REPLAY] frames="<<processed<<" t="<<t<<" tracking="<<n.trackingState
                    <<" valid="<<n.valid<<" imu="<<n.inertialInitialized<<" ba2="<<n.inertialBA2
                    <<" epoch="<<n.referenceEpoch<<" bad_images="<<bad<<std::endl;
                previousState=n.trackingState;
            }
        }
        const auto last=slam->GetNavigationState();slam->Shutdown();
        slam->SaveTrajectoryEuRoC(argv[6]);
        std::cout<<"[REPLAY_RESULT] frames="<<processed<<" ever_ba2="<<everBa2
            <<" final_valid="<<last.valid<<" final_ba2="<<last.inertialBA2<<std::endl;
        return everBa2 && last.valid && last.inertialBA2?0:1;
    } catch(const std::exception& e) {
        std::cerr<<"[REPLAY_ERROR] "<<e.what()<<std::endl;
        if(slam)slam->Shutdown();return 2;
    }
}
