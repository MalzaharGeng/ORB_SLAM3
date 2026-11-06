#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
namespace ORB_SLAM3 {
// Preserve luma directly. No BGR conversion or chroma JPEG decode is required.
inline cv::Mat UvcGray(const cv::Mat& raw,const std::string& format,int width,int height) {
    if(raw.empty())return {};
    cv::Mat gray;
    if(format=="YUYV") {
        if(raw.type()!=CV_8UC2 || raw.cols!=width || raw.rows!=height)return {};
        cv::extractChannel(raw,gray,0);
    } else if(format=="MJPG") {
        if(raw.type()!=CV_8UC1 || !raw.isContinuous() || raw.total()<4 ||
           raw.ptr<unsigned char>()[0]!=0xff || raw.ptr<unsigned char>()[1]!=0xd8)return {};
        try {gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);}
        catch(const cv::Exception&) {return {};}
    } else throw std::runtime_error("Unsupported pixel format: "+format);
    if(gray.type()!=CV_8UC1 || gray.cols!=width || gray.rows!=height)return {};
    return gray;
}
// A transient bad first MJPEG buffer must not abort an otherwise healthy stream.
// Stop and attempt bounds also apply when every supplied frame is invalid.
template<class Read,class Stop>
cv::Mat UvcStartupGray(Read read,Stop stop,int width,int height,int& skipped,int attempts=20) {
    skipped=0;
    for(int i=0;i<attempts && !stop();++i) {
        cv::Mat raw;read(raw);
        cv::Mat gray=UvcGray(raw,"MJPG",width,height);
        if(!gray.empty())return gray;
        ++skipped;
    }
    return {};
}

}
