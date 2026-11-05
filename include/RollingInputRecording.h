#pragma once
#include "ImuTypes.h"
#include <filesystem>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>

namespace ORB_SLAM3 {
// Lossless, bounded, crash-recoverable inputs actually passed to TrackStereo.
// No encoding or filesystem operations on the capture/tracking thread.
class RollingInputRecording {
public:
    struct Options {
        double seconds=20.;
        double overviewSeconds=120., overviewPeriod=1.;
        uintmax_t minFreeBytes=2ULL*1024*1024*1024;
        uintmax_t maxEventBytes=4ULL*1024*1024*1024;
        size_t maxEvents=24, maxQueue=6;
        double triggerCooldownSeconds=5.;
    };
    RollingInputRecording(const std::filesystem::path& ring,
                          const std::filesystem::path& events,
                          const std::filesystem::path& settings, Options options);
    ~RollingInputRecording();
    void Frame(const cv::Mat&,const cv::Mat&,double timestamp,const std::vector<IMU::Point>&,
               double steadySeconds=-1.);
    void Trigger(const std::string& reason);
    bool Close();
    std::string Error() const;
    size_t Dropped() const;
private:
    struct Packet {
        size_t id=0;double timestamp=0,steady=0;
        cv::Mat left,right;std::vector<IMU::Point> imu;
        std::string trigger;
    };
    struct Entry {std::filesystem::path directory;double steady;};
    void Run();
    void Freeze(const std::string& reason);
    void LimitEvents();
    void Fail(const std::string& reason);
    static double Now();
    const std::filesystem::path ring_,events_;
    Options options_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<Packet> queue_;
    std::deque<Entry> retained_;
    std::deque<Entry> overview_;
    double lastOverview_=-1e20;
    std::thread worker_;
    bool stopping_=false,closed_=false;
    size_t sequence_=0,dropped_=0,eventSequence_=0;
    double lastTrigger_=-1e20;
    std::string error_;
};
}
