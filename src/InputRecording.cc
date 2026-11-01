#include "InputRecording.h"
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace ORB_SLAM3 {
namespace fs=std::filesystem;
namespace {
const char* format="ORB_SLAM3_STEREO_INPUT_V1";
std::vector<std::string> split(const std::string& line) {
    std::istringstream in(line);std::vector<std::string> r;std::string s;
    while(std::getline(in,s,','))r.push_back(s);
    return r;
}
void require(bool ok,const std::string& text) {if(!ok)throw std::runtime_error("Recording: "+text);}
size_t count(const std::string& s) {
    require(!s.empty() && s.find_first_not_of("0123456789")==std::string::npos,"invalid index/count");
    return std::stoull(s);
}
double number(const std::string& s) {
    size_t used=0;const double v=std::stod(s,&used);
    require(used==s.size() && std::isfinite(v),"nonfinite/invalid number");return v;
}
bool action(const std::string& s) {
    return s=="first" || s=="reset" || s=="second" || s=="load" || s=="reset_active" || s=="reset_full";
}
void imagePath(const fs::path& directory,const std::string& name) {
    const fs::path p=name;
    require(!p.empty() && !p.is_absolute(),"image path must be relative");
    for(const auto& part:p)require(part!="..","image path leaves recording");
    require(fs::is_regular_file(directory/p),"missing image: "+name);
}
}

InputRecording::InputRecording(const fs::path& directory,const fs::path& settings):directory_(fs::absolute(directory)) {
    require(fs::is_regular_file(settings),"settings file missing");
    fs::create_directories(directory_.parent_path());
    require(fs::create_directory(directory_),"output already exists: "+directory_.string());
    fs::copy_file(settings,directory_/"settings.yaml");
    fs::create_directory(directory_/"left");fs::create_directory(directory_/"right");
    std::ofstream marker(directory_/"format.txt");marker<<format<<"\n";marker.close();
    frames_.open(directory_/"stereo.csv");imu_.open(directory_/"imu.csv");events_.open(directory_/"events.csv");
    frames_<<"index,timestamp,left,right,imu_count\n"<<std::setprecision(17);frames_.flush();
    imu_<<"frame,timestamp,ax,ay,az,gx,gy,gz\n"<<std::setprecision(17);imu_.flush();
    events_<<"frame,action\n";events_.flush();
    require(bool(marker)&&bool(frames_)&&bool(imu_)&&bool(events_),"cannot create recording files");
    worker_=std::thread(&InputRecording::Write,this);
    std::cout<<"[Recording] started: "<<directory_<<" (stereo PNG + synchronized IMU + recovery actions)"<<std::endl;
}
InputRecording::~InputRecording() {Close();}
void InputRecording::Fail(const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!error_.empty())return;
    error_=message;queue_.clear();
    std::ofstream(directory_/"error.txt")<<message<<"\n";
    std::cerr<<"[Recording] FAILED: "<<message<<"; SLAM continues, recording is incomplete"<<std::endl;
    wake_.notify_all();
}
void InputRecording::Frame(const cv::Mat& left,const cv::Mat& right,double timestamp,const std::vector<IMU::Point>& imu) {
    try {
        std::unique_lock<std::mutex> lock(mutex_);
        if(stopping_ || !error_.empty())return;
        if(queue_.size()>=60) {lock.unlock();Fail("disk writer queue exceeded 60 frames");return;}
        require(!left.empty() && !right.empty() && left.type()==CV_8UC1 && right.type()==CV_8UC1,
                "expected nonempty 8-bit stereo infrared images");
        require(std::isfinite(timestamp),"invalid frame timestamp");
        queue_.push_back({submitted_,timestamp,left.clone(),right.clone(),imu});
        ++submitted_;wake_.notify_one();
    } catch(const std::exception& e) {Fail(e.what());}
}
void InputRecording::Event(const std::string& command) {
    try {
        std::unique_lock<std::mutex> lock(mutex_);
        if(stopping_ || !error_.empty() || !submitted_)return;
        require(action(command),"unknown recovery action");
        events_<<submitted_-1<<","<<command<<"\n";events_.flush();
        require(bool(events_),"cannot write events");
    } catch(const std::exception& e) {Fail(e.what());}
}
void InputRecording::Write() {
    try {
        for(;;) {
            Packet p;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock,[&]{return stopping_ || !error_.empty() || !queue_.empty();});
                if(!error_.empty() || (stopping_ && queue_.empty()))break;
                p=std::move(queue_.front());queue_.pop_front();
            }
            const auto left="left/"+std::to_string(p.index)+".png",right="right/"+std::to_string(p.index)+".png";
            require(cv::imwrite((directory_/left).string(),p.left,{cv::IMWRITE_PNG_COMPRESSION,1}) &&
                    cv::imwrite((directory_/right).string(),p.right,{cv::IMWRITE_PNG_COMPRESSION,1}),"image write failed");
            for(const auto& m:p.imu) {
                require(std::isfinite(m.t)&&m.a.allFinite()&&m.w.allFinite(),"invalid IMU sample");
                imu_<<p.index<<","<<m.t<<","<<m.a.x()<<","<<m.a.y()<<","<<m.a.z()
                    <<","<<m.w.x()<<","<<m.w.y()<<","<<m.w.z()<<"\n";
            }
            imu_.flush();require(bool(imu_),"IMU write failed");
            // Publish a frame index only after both images and its entire IMU
            // batch exist. An interrupted file can replay the committed prefix.
            frames_<<p.index<<","<<p.timestamp<<","<<left<<","<<right<<","<<p.imu.size()<<"\n";
            frames_.flush();require(bool(frames_),"frame index write failed");++written_;
        }
    } catch(const std::exception& e) {Fail(e.what());}
}
bool InputRecording::Close() {
    {std::lock_guard<std::mutex> lock(mutex_);if(closed_)return error_.empty();stopping_=true;wake_.notify_all();}
    if(worker_.joinable())worker_.join();
    frames_.close();imu_.close();events_.close();
    if(error_.empty() && (!frames_ || !imu_ || !events_))Fail("closing recording files failed");
    if(error_.empty()) {
        std::ofstream end(directory_/"complete.txt");end<<written_<<"\n";end.close();
        if(!end)Fail("cannot finalize recording");
    }
    closed_=true;
    std::cout<<"[Recording] "<<(error_.empty()?"COMPLETE":"INCOMPLETE")<<" frames="<<written_<<" directory="<<directory_<<std::endl;
    return error_.empty();
}

RecordedInput ReadInputRecording(const fs::path& directory) {
    std::string line;std::ifstream marker(directory/"format.txt");std::getline(marker,line);
    require(line==format,"unsupported format");
    RecordedInput result;result.complete=fs::exists(directory/"complete.txt");
    require(!fs::exists(directory/"error.txt"),"writer reported failure; inspect error.txt");
    std::ifstream frames(directory/"stereo.csv"),imu(directory/"imu.csv"),events(directory/"events.csv");
    require(bool(frames)&&bool(imu)&&bool(events),"missing input CSV");
    std::getline(frames,line);require(line=="index,timestamp,left,right,imu_count","invalid stereo header");
    std::vector<size_t> counts;
    while(std::getline(frames,line)) {
        if(frames.eof() && !result.complete)break; // interrupted final CSV row
        const auto r=split(line);require(r.size()==5,"invalid stereo row");
        require(count(r[0])==result.frames.size(),"frame index gap");
        imagePath(directory,r[2]);imagePath(directory,r[3]);
        result.frames.push_back({number(r[1]),r[2],r[3],{}, {}});counts.push_back(count(r[4]));
    }
    require(!result.frames.empty(),"no recorded stereo frames");
    if(result.complete) {
        std::ifstream end(directory/"complete.txt");std::getline(end,line);
        require(count(line)==result.frames.size(),"completed recording frame count differs");
    }
    std::getline(imu,line);require(line=="frame,timestamp,ax,ay,az,gx,gy,gz","invalid IMU header");
    size_t previous=0;
    while(std::getline(imu,line)) {
        if(imu.eof() && !result.complete)break;
        const auto r=split(line);require(r.size()==8,"invalid IMU row");
        const auto i=count(r[0]);
        if(!result.complete && i>=result.frames.size())break;
        require(i<result.frames.size() && i>=previous,"IMU frame index invalid");previous=i;
        result.frames[i].imu.emplace_back(number(r[2]),number(r[3]),number(r[4]),number(r[5]),number(r[6]),number(r[7]),number(r[1]));
    }
    for(size_t i=0;i<counts.size();++i)require(result.frames[i].imu.size()==counts[i],"IMU batch truncated");
    std::getline(events,line);require(line=="frame,action","invalid event header");previous=0;
    while(std::getline(events,line)) {
        if(events.eof() && !result.complete)break;
        const auto r=split(line);require(r.size()==2,"invalid event row");const auto i=count(r[0]);
        if(!result.complete && i>=result.frames.size())break;
        require(i<result.frames.size() && i>=previous && action(r[1]),"invalid event");previous=i;
        result.frames[i].actions.push_back(r[1]);
    }
    if(!result.complete)std::cerr<<"[Replay] recording was not finalized; replaying "<<result.frames.size()<<" committed frames only"<<std::endl;
    return result;
}
}
