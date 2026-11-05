#include "RollingInputRecording.h"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>
using namespace ORB_SLAM3;
namespace fs=std::filesystem;
void check(bool c,const char* m){if(!c)throw std::runtime_error(m);}
int main(){
    char pattern[]="/tmp/orb-ring-XXXXXX";const fs::path root=mkdtemp(pattern);
    try {
        std::ofstream(root/"settings.yaml")<<"%YAML:1.0\nCamera.fps: 15\n";
        cv::Mat image(24,32,CV_8UC1);cv::randu(image,0,255);const auto expected=image.clone();
        RollingInputRecording::Options o;o.seconds=20;o.minFreeBytes=0;o.maxQueue=60;o.triggerCooldownSeconds=0;
        o.overviewSeconds=24;o.overviewPeriod=2;
        std::vector<IMU::Point> imu{{1,2,3,.1,.2,.3,10}};
        {
            RollingInputRecording w(root/"ring",root/"events",root/"settings.yaml",o);
            for(int i=0;i<31;++i)w.Frame(image,image,10+i,imu,double(i));
            w.Trigger("tracking_lost");
            w.Frame(image,image,41,imu,31.);
            check(w.Close(),"ring writer failed");check(w.Close(),"ring Close not idempotent");
        }
        size_t ringCount=0;for(auto& d:fs::directory_iterator(root/"ring"))if(d.is_directory() && d.path().filename().string().find("frame_")==0)++ringCount;
        check(ringCount==21,"20-second ring bound incorrect");
        fs::path event;for(auto& d:fs::directory_iterator(root/"events"))if(d.is_directory())event=d.path();
        check(!event.empty(),"trigger did not preserve event");
        size_t n=0;for(auto& d:fs::directory_iterator(event))if(d.is_directory() && d.path().filename().string().find("frame_")==0)++n;
        check(n==21,"snapshot did not preserve pretrigger window");
        auto old=event/"frame_000000000010";
        check(fs::exists(old/"left.png")&&!fs::exists(root/"ring/frame_000000000010"),"frozen frame was removed with ring");
        check(cv::norm(cv::imread((old/"left.png").string(),0),expected,cv::NORM_INF)==0,"recorded pixels differ");
        check(!fs::exists(event/"frame_000000000031"),"posttrigger input leaked into pretrigger snapshot");
        size_t context=0;for(auto& d:fs::directory_iterator(event/"overview"))if(d.is_directory())++context;
        check(context==13,"sparse context time/period bound incorrect");
        check(fs::exists(event/"overview/frame_000000000006/left.png"),"context before full-rate window lost");
        check(!fs::exists(root/"ring/overview/frame_000000000006"),"sparse ring did not evict expired context");
        check(fs::exists(event/"overview/overview.txt"),"sparse inputs not identified");
        std::ifstream sample(old/"imu.csv");std::string text;std::getline(sample,text);check(text.find("10,1,2,3,")==0,"IMU batch lost");
        // Only marked events are eligible for quota deletion. Unrelated data survives.
        fs::create_directory(root/"events/unrelated");
        o.maxEvents=1;
        {
            RollingInputRecording w(root/"ring2",root/"events",root/"settings.yaml",o);
            w.Frame(image,image,100,imu,100);w.Trigger("second");check(w.Close(),"second recording failed");
        }
        check(fs::exists(root/"events/unrelated"),"quota removed unrelated data");
        size_t own=0;for(auto& d:fs::directory_iterator(root/"events"))if(fs::exists(d.path()/"rolling_event.txt"))++own;
        check(own==1,"event quota not enforced");
        {
            o.minFreeBytes=UINT64_MAX;
            RollingInputRecording w(root/"disk_full",root/"events",root/"settings.yaml",o);
            w.Frame(image,image,1,{},1);check(!w.Close(),"free-space reserve ignored");
            check(fs::exists(root/"disk_full/error.txt"),"recording failure not exposed");
        }
        fs::remove_all(root);std::cout<<"ROLLING_INPUT_REGRESSION_PASS\n";return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<" artifacts="<<root<<std::endl;return 1;}
}
