#include "InputRecording.h"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>
using namespace ORB_SLAM3;
namespace fs=std::filesystem;
static void require(bool ok,const char* text) {if(!ok)throw std::runtime_error(text);}
template<class F> void rejects(F f,const char* text) {
    bool failed=false;try {f();}catch(const std::exception&){failed=true;}require(failed,text);
}
int main() {
    char pattern[]="/tmp/orb-recording-XXXXXX";const char* temp=mkdtemp(pattern);
    require(temp,"mkdtemp failed");const fs::path root=temp;
    try {
        std::ofstream(root/"settings.yaml")<<"%YAML:1.0\nIMU.fastInit: 0\n";
        const fs::path recording=root/"recording with spaces";
        cv::Mat left(24,32,CV_8UC1),right(24,32,CV_8UC1);cv::randu(left,0,255);cv::randu(right,0,255);
        const auto expectedLeft=left.clone(),expectedRight=right.clone();
        std::vector<IMU::Point> imu{{1.25f,-2.f,9.8f,.01f,.02f,-.03f,1000.1001},
                                   {2.f,3.f,9.f,-.01f,.04f,.03f,1000.1999}};
        {
            InputRecording writer(recording,root/"settings.yaml");
            writer.Frame(left,right,1000.15,imu); // batch includes a sample after image time
            left.setTo(0);right.setTo(0); // queue must own pixels
            writer.Frame(left,right,1000.18,{});writer.Event("first");writer.Event("reset");
            writer.Frame(left,right,1.0,imu);writer.Event("reset_active"); // clock discontinuity
            require(writer.Close(),"recording failed");require(writer.Close(),"Close not idempotent");
        }
        const auto data=ReadInputRecording(recording);
        require(data.complete && data.frames.size()==3,"frame roundtrip failed");
        require(data.frames[0].timestamp==1000.15 && data.frames[2].timestamp==1.,"timestamps rebased/lost");
        require(data.frames[1].imu.empty() && data.frames[0].imu.size()==2,"IMU frame boundaries lost");
        for(size_t i=0;i<imu.size();++i) {
            const auto& read=data.frames[0].imu[i];
            require(read.t==imu[i].t && read.a==imu[i].a && read.w==imu[i].w,"IMU values changed");
        }
        require(cv::norm(cv::imread((recording/data.frames[0].left).string(),0),expectedLeft,cv::NORM_INF)==0 &&
                cv::norm(cv::imread((recording/data.frames[0].right).string(),0),expectedRight,cv::NORM_INF)==0,"image pixels changed");
        require(data.frames[1].actions==std::vector<std::string>({"first","reset"}) &&
                data.frames[2].actions==std::vector<std::string>({"reset_active"}),"event order/boundary lost");
        rejects([&]{InputRecording overwrite(recording,root/"settings.yaml");},"existing recording overwritten");
        std::ofstream(recording/"complete.txt")<<"4\n";
        rejects([&]{ReadInputRecording(recording);},"truncated finalized file accepted");
        fs::remove(recording/"complete.txt");
        std::ofstream(recording/"stereo.csv",std::ios::app)<<"3,100"; // interrupted index row
        std::ofstream(recording/"imu.csv",std::ios::app)<<"3,1001,0,0,9,0,0,0\n";
        std::ofstream(recording/"events.csv",std::ios::app)<<"3,load\n";
        require(ReadInputRecording(recording).frames.size()==3,"committed prefix not recoverable");
        fs::remove(recording/data.frames[0].left);
        rejects([&]{ReadInputRecording(recording);},"missing committed image accepted");
        {
            InputRecording writer(root/"disk_failure",root/"settings.yaml");
            fs::remove(root/"disk_failure/left");
            writer.Frame(left,right,1.,{});
            require(!writer.Close(),"disk failure reported complete");
            require(!fs::exists(root/"disk_failure/complete.txt"),"failure wrote completion marker");
            rejects([&]{ReadInputRecording(root/"disk_failure");},"failed recording accepted silently");
        }
        fs::remove_all(root);
        std::cout<<"INPUT_RECORDING_PASS: exact pixels/IMU/batches, events, clock reset, overwrite refusal, interrupted prefix, write failures\n";
        return 0;
    }catch(const std::exception& e) {std::cerr<<e.what()<<"; artifacts: "<<root<<std::endl;return 1;}
}
