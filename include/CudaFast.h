#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <vector>
namespace ORB_SLAM3 {
bool CudaFastScores(const std::vector<cv::Mat>& pyramid,std::vector<cv::Mat>& scores,int minimumThreshold);
void CudaFastCell(const cv::Mat& scores,const cv::Rect& roi,int threshold,std::vector<cv::KeyPoint>& keypoints);
std::uint64_t CudaFastCalls();
}
