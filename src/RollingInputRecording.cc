#include "RollingInputRecording.h"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>

namespace ORB_SLAM3 {
namespace fs=std::filesystem;
namespace {
void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
void syncFile(const fs::path& path) {
    const int fd=::open(path.c_str(),O_RDONLY);
    check(fd>=0,"cannot open committed recording for fsync");
    const int rc=::fsync(fd);::close(fd);check(rc==0,"recording fsync failed");
}
uintmax_t bytes(const fs::path& path) {
    uintmax_t n=0;for(const auto& f:fs::recursive_directory_iterator(path))if(f.is_regular_file())n+=f.file_size();return n;
}
std::string safeReason(std::string s) {
    if(s.size()>120)s.resize(120);
    for(char& c:s)if(!std::isalnum(static_cast<unsigned char>(c)) && c!='_' && c!='-')c='_';
    return s.empty()?"manual":s;
}
void linkFrame(const fs::path& source,const fs::path& target) {
    fs::create_directory(target);
    for(const auto& file:fs::directory_iterator(source)) {
        std::error_code ec;fs::create_hard_link(file.path(),target/file.path().filename(),ec);
        if(ec)fs::copy_file(file.path(),target/file.path().filename());
    }
}
void copyMetadata(const fs::path& source,const fs::path& target) {
    for(const char* name:{"format.txt","settings.yaml","clock_origin.txt","camera_calibration.json"})
        if(fs::is_regular_file(source/name))fs::copy_file(source/name,target/name,fs::copy_options::overwrite_existing);
}
}
double RollingInputRecording::Now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
RollingInputRecording::RollingInputRecording(const fs::path& ring,const fs::path& events,
                                             const fs::path& settings,Options o):
    ring_(fs::absolute(ring)),events_(fs::absolute(events)),options_(o) {
    check(std::isfinite(o.seconds) && o.seconds>0 && o.seconds<=60 && o.maxEvents>0 && o.maxQueue>0 && o.maxQueue<=60,
          "invalid rolling recording bounds");
    check(std::isfinite(o.overviewSeconds)&&o.overviewSeconds>=0&&o.overviewSeconds<=600 &&
          std::isfinite(o.overviewPeriod)&&o.overviewPeriod>=1&&o.overviewPeriod<=60,"invalid overview bounds");
    check(fs::is_regular_file(settings),"rolling settings missing");
    fs::create_directories(ring_.parent_path());
    check(fs::create_directory(ring_),"rolling output already exists");
    fs::create_directories(events_);
    std::ofstream(ring_/"format.txt")<<"ORB_SLAM3_ROLLING_INPUT_V1\n";
    fs::copy_file(settings,ring_/"settings.yaml");
    std::ofstream(ring_/"limits.txt")<<"retention_seconds="<<o.seconds<<"\nmax_queue="<<o.maxQueue
        <<"\noverview_seconds="<<o.overviewSeconds<<"\noverview_period="<<o.overviewPeriod
        <<"\nmax_event_bytes="<<o.maxEventBytes<<"\nmax_events="<<o.maxEvents<<"\n";
    if(o.overviewSeconds>0) {
        fs::create_directory(ring_/"overview");copyMetadata(ring_,ring_/"overview");
        std::ofstream(ring_/"overview/overview.txt")<<"Sparse visual context; IMU batches are not continuous.\nperiod_seconds="<<o.overviewPeriod<<"\n";
    }
    worker_=std::thread(&RollingInputRecording::Run,this);
    std::cout<<"[RollingInput] started "<<ring_<<" retention="<<o.seconds<<"s events="<<events_<<std::endl;
}
RollingInputRecording::~RollingInputRecording(){Close();}
void RollingInputRecording::Frame(const cv::Mat& left,const cv::Mat& right,double stamp,
                                  const std::vector<IMU::Point>& imu,double steady) {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_ || !error_.empty())return;
        const size_t id=sequence_++;
        if(queue_.size()>=options_.maxQueue) {++dropped_;return;}
        check(!left.empty() && !right.empty() && left.type()==CV_8UC1 && right.type()==CV_8UC1,"invalid rolling stereo image");
        check(std::isfinite(stamp),"nonfinite image timestamp");
        for(const auto& m:imu)check(std::isfinite(m.t)&&m.a.allFinite()&&m.w.allFinite(),"nonfinite rolling IMU");
        Packet p;p.id=id;p.timestamp=stamp;p.steady=steady<0?Now():steady;
        p.left=left.clone();p.right=right.clone();p.imu=imu;
        queue_.push_back(std::move(p));wake_.notify_one();
    } catch(const std::exception& e) {Fail(e.what());}
}
void RollingInputRecording::Trigger(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now=Now();
    if(stopping_ || !error_.empty() || now-lastTrigger_<options_.triggerCooldownSeconds)return;
    // A bounded control packet sits behind inputs already submitted at trigger time.
    // Future frames cannot leak into this pre-event snapshot.
    if(queue_.size()>options_.maxQueue)return;
    Packet p;p.trigger=safeReason(reason);queue_.push_back(std::move(p));
    lastTrigger_=now;wake_.notify_one();
}
void RollingInputRecording::Fail(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(error_.empty()) {
        error_=reason;queue_.clear();
        std::ofstream(ring_/"error.txt")<<reason<<"\n";
        std::cerr<<"[RollingInput] FAILED "<<reason<<"; VIO continues"<<std::endl;
    }
    wake_.notify_all();
}
void RollingInputRecording::LimitEvents() {
    std::vector<fs::path> own;
    for(const auto& d:fs::directory_iterator(events_))
        if(d.is_directory() && fs::is_regular_file(d.path()/"rolling_event.txt"))own.push_back(d.path());
    std::sort(own.begin(),own.end(),[](const auto& a,const auto& b){return fs::last_write_time(a)<fs::last_write_time(b);});
    uintmax_t total=0;for(const auto& p:own)total+=bytes(p);
    while(!own.empty() && (own.size()>options_.maxEvents || total>options_.maxEventBytes)) {
        const auto p=own.front();const auto amount=bytes(p);
        // Only marked, immediate child directories in our dedicated event root.
        check(p.parent_path()==events_ && fs::is_regular_file(p/"rolling_event.txt"),"invalid event eviction target");
        fs::remove_all(p);total-=amount;own.erase(own.begin());
    }
}
void RollingInputRecording::Freeze(const std::string& reason) {
    if(retained_.empty())return;
    const auto wall=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    const std::string name="event_"+std::to_string(wall)+"_"+std::to_string(::getpid())+"_"+std::to_string(eventSequence_++);
    const fs::path tmp=events_/(name+".partial"),final=events_/name;
    check(fs::create_directory(tmp),"event name collision");
    copyMetadata(ring_,tmp);
    for(const auto& e:retained_) {
        linkFrame(e.directory,tmp/e.directory.filename());
    }
    if(!overview_.empty()) {
        fs::create_directory(tmp/"overview");copyMetadata(ring_,tmp/"overview");
        fs::copy_file(ring_/"overview/overview.txt",tmp/"overview/overview.txt");
        for(const auto& e:overview_)linkFrame(e.directory,tmp/"overview"/e.directory.filename());
    }
    {std::ofstream marker(tmp/"rolling_event.txt");marker<<"reason="<<reason<<"\nframes="<<retained_.size()
        <<"\nspan_seconds="<<std::setprecision(12)<<retained_.back().steady-retained_.front().steady
        <<"\noverview_frames="<<overview_.size()<<"\n";
        marker.close();check(bool(marker),"cannot commit event marker");}
    syncFile(tmp/"rolling_event.txt");fs::rename(tmp,final);syncFile(events_);
    LimitEvents();
    std::cout<<"[RollingInput] EVENT "<<reason<<" frames="<<retained_.size()<<" path="<<final<<std::endl;
}
void RollingInputRecording::Run() {
    try {
        for(;;) {
            Packet p;
            {std::unique_lock<std::mutex> lock(mutex_);
             wake_.wait(lock,[&]{return stopping_||!error_.empty()||!queue_.empty();});
             if(!error_.empty() || (stopping_&&queue_.empty()))break;
             p=std::move(queue_.front());queue_.pop_front();}
            if(!p.trigger.empty()) {Freeze(p.trigger);continue;}
            if(p.id%15==0)check(fs::space(ring_).available>=options_.minFreeBytes,"recording free space reserve reached");
            std::ostringstream id;id<<"frame_"<<std::setw(12)<<std::setfill('0')<<p.id;
            const auto target=ring_/id.str(),tmp=ring_/(id.str()+".partial");
            check(fs::create_directory(tmp),"frame output already exists");
            check(cv::imwrite((tmp/"left.png").string(),p.left,{cv::IMWRITE_PNG_COMPRESSION,1}) &&
                  cv::imwrite((tmp/"right.png").string(),p.right,{cv::IMWRITE_PNG_COMPRESSION,1}),"rolling PNG write failed");
            {std::ofstream meta(tmp/"frame.csv");meta<<std::setprecision(17)<<p.id<<","<<p.timestamp<<","<<p.steady<<","<<p.imu.size()<<"\n";
             meta.close();check(bool(meta),"rolling frame metadata write failed");}
            {std::ofstream imu(tmp/"imu.csv");imu<<std::setprecision(17);
             for(const auto& m:p.imu)imu<<m.t<<","<<m.a.x()<<","<<m.a.y()<<","<<m.a.z()<<","<<m.w.x()<<","<<m.w.y()<<","<<m.w.z()<<"\n";
             imu.close();check(bool(imu),"rolling IMU write failed");}
            // Per-frame data precedes atomic directory commit. Periodically
            // synchronize a complete checkpoint frame. Filesystem/device caches
            // can still lose other recently committed frames on sudden power loss.
            if(p.id%15==0) {
                for(const auto& f:fs::directory_iterator(tmp))syncFile(f.path());
                syncFile(tmp);
            }
            fs::rename(tmp,target);
            if(p.id%15==0)syncFile(ring_);
            retained_.push_back({target,p.steady});
            if(options_.overviewSeconds>0) {
                if(p.steady-lastOverview_>=options_.overviewPeriod) {
                    const auto overviewTarget=ring_/"overview"/target.filename();
                    linkFrame(target,overviewTarget);overview_.push_back({overviewTarget,p.steady});lastOverview_=p.steady;
                }
                while(!overview_.empty() && p.steady-overview_.front().steady>options_.overviewSeconds) {
                    check(overview_.front().directory.parent_path()==ring_/"overview","invalid overview eviction target");
                    fs::remove_all(overview_.front().directory);overview_.pop_front();
                }
            }
            while(!retained_.empty() && p.steady-retained_.front().steady>options_.seconds) {
                const auto old=retained_.front().directory;
                check(old.parent_path()==ring_,"invalid ring eviction target");
                fs::remove_all(old);retained_.pop_front();
            }
        }
    } catch(const std::exception& e) {Fail(e.what());}
}
bool RollingInputRecording::Close() {
    {std::lock_guard<std::mutex> lock(mutex_);if(closed_)return error_.empty();stopping_=true;wake_.notify_all();}
    if(worker_.joinable())worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);closed_=true;
    std::ofstream(ring_/"closed.txt")<<"error="<<error_<<"\ndropped="<<dropped_<<"\n";
    return error_.empty();
}
std::string RollingInputRecording::Error() const {std::lock_guard<std::mutex> lock(mutex_);return error_;}
size_t RollingInputRecording::Dropped() const {std::lock_guard<std::mutex> lock(mutex_);return dropped_;}
}
