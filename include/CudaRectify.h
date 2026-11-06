#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <memory>
namespace ORB_SLAM3 {
// Same fixed-point maps, interpolation and zero-border policy as CPU remap.
// One instance belongs to one capture/replay thread. Maps must remain unchanged.
class CudaStereoRectifier {
public:
    CudaStereoRectifier();
    ~CudaStereoRectifier();
    CudaStereoRectifier(const CudaStereoRectifier&)=delete;
    CudaStereoRectifier& operator=(const CudaStereoRectifier&)=delete;
    bool Remap(const cv::Mat& gray,const cv::Rect& leftRoi,const cv::Rect& rightRoi,
               const cv::Mat& leftMap,const cv::Mat& leftFraction,
               const cv::Mat& rightMap,const cv::Mat& rightFraction,
               cv::Mat& left,cv::Mat& right);
    std::uint64_t Calls()const;
private:
    struct State;
    std::unique_ptr<State> state_;
};
}
