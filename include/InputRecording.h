#pragma once

#include "ImuTypes.h"
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

namespace ORB_SLAM3 {

// Records the exact stereo/IMU batches passed to TrackStereo, before its own
// rectification/resizing. PNG work runs off the tracking thread. A bounded
// queue fails explicitly instead of silently dropping input or blocking VIO.
class InputRecording {
public:
    InputRecording(const std::filesystem::path& directory, const std::filesystem::path& settings);
    ~InputRecording();
    void Frame(const cv::Mat& left, const cv::Mat& right, double timestamp,
               const std::vector<IMU::Point>& imu);
    // Successful recovery actions and explicit resets, before the current frame.
    // Called only on the TrackStereo thread, never from Viewer.
    void Event(const std::string& action);
    bool Close(); // Call after acquisition stops; flushes and joins the writer.
private:
    struct Packet {
        size_t index;
        double timestamp;
        cv::Mat left, right;
        std::vector<IMU::Point> imu;
    };
    void Write();
    void Fail(const std::string& message);
    const std::filesystem::path directory_;
    std::ofstream frames_, imu_, events_;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<Packet> queue_;
    std::thread worker_;
    size_t submitted_=0, written_=0;
    bool stopping_=false, closed_=false;
    std::string error_;
};

struct RecordedFrame {
    double timestamp;
    std::string left, right;
    std::vector<IMU::Point> imu;
    std::vector<std::string> actions;
};
struct RecordedInput {
    std::vector<RecordedFrame> frames;
    bool complete=false;
};
RecordedInput ReadInputRecording(const std::filesystem::path& directory);
}
