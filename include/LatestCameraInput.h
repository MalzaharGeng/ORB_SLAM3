#pragma once
#include <opencv2/core.hpp>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <utility>
namespace ORB_SLAM3 {
struct TimedCameraInput {
    cv::Mat image; // Owned image or encoded-frame storage; callers must not mutate it after Put.
    double timestamp=0,grabMs=0;
};
// A single waiting frame, with its timestamp. Replacements never form a backlog.
// Stop wakes a waiting consumer and permits the final waiting frame to drain.
class LatestCameraInput {
public:
    bool Put(TimedCameraInput input) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopped_)return false;
        if(has_)++dropped_;
        value_=std::move(input);has_=true;wake_.notify_one();return true;
    }
    bool Take(TimedCameraInput& input) {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock,[this]{return has_ || stopped_;});
        if(!has_)return false;
        input=std::move(value_);has_=false;return true;
    }
    void Stop() {
        std::lock_guard<std::mutex> lock(mutex_);stopped_=true;wake_.notify_all();
    }
    std::uint64_t Dropped()const {
        std::lock_guard<std::mutex> lock(mutex_);return dropped_;
    }
private:
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    TimedCameraInput value_;
    bool has_=false,stopped_=false;
    std::uint64_t dropped_=0;
};
}
