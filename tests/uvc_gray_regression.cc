#include "Examples/Utils/UvcGray.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdexcept>
static void require(bool b) {if(!b)throw std::runtime_error("UVC gray regression failed");}
int main() {
    cv::Mat yuyv(3,6,CV_8UC2);
    for(int y=0;y<3;++y)for(int x=0;x<6;++x)yuyv.at<cv::Vec2b>(y,x)=cv::Vec2b(y*10+x,127+x);
    auto gray=ORB_SLAM3::UvcGray(yuyv,"YUYV",6,3);
    for(int y=0;y<3;++y)for(int x=0;x<6;++x)require(gray.at<unsigned char>(y,x)==y*10+x);
    // Honor stride: cropped rows must not assume tightly packed input.
    auto roi=yuyv(cv::Rect(1,0,4,3));
    gray=ORB_SLAM3::UvcGray(roi,"YUYV",4,3);require(gray.at<unsigned char>(2,3)==24);
    require(ORB_SLAM3::UvcGray(yuyv,"YUYV",5,3).empty());
    require(ORB_SLAM3::UvcGray(cv::Mat(3,6,CV_8UC3),"YUYV",6,3).empty());
    cv::Mat source(64,96,CV_8UC1);cv::randu(source,0,255);
    std::vector<unsigned char> encoded;cv::imencode(".jpg",source,encoded);
    gray=ORB_SLAM3::UvcGray(cv::Mat(encoded),"MJPG",96,64);
    require(gray.size()==source.size());
    encoded[0]=0;require(ORB_SLAM3::UvcGray(cv::Mat(encoded),"MJPG",96,64).empty());
    require(ORB_SLAM3::UvcGray(cv::Mat(),"YUYV",6,3).empty());
    std::vector<unsigned char> good;cv::imencode(".jpg",source,good);
    const cv::Mat expected=cv::imdecode(good,cv::IMREAD_GRAYSCALE);
    int reads=0,skipped=0;
    auto decoded=ORB_SLAM3::UvcStartupGray([&](cv::Mat& raw) {
        ++reads;
        if(reads==1)raw=cv::Mat(1,8,CV_8UC1,cv::Scalar(0));
        else if(reads==2) {
            cv::Mat wrong(32,48,CV_8UC1,cv::Scalar(127));std::vector<unsigned char> bytes;
            cv::imencode(".jpg",wrong,bytes);raw=cv::Mat(bytes).clone();
        } else raw=cv::Mat(good).clone();
    },[]{return false;},96,64,skipped);
    require(reads==3 && skipped==2 && cv::countNonZero(decoded!=expected)==0);
    reads=0;
    decoded=ORB_SLAM3::UvcStartupGray([&](cv::Mat& raw){++reads;raw=cv::Mat();},
        []{return false;},96,64,skipped);
    require(decoded.empty() && reads==20 && skipped==20);
    reads=0;
    decoded=ORB_SLAM3::UvcStartupGray([&](cv::Mat&){++reads;},[]{return true;},96,64,skipped);
    require(decoded.empty() && reads==0 && skipped==0);
    std::cout<<"PASS exact YUYV luma/stride, frame layout guards, JPEG dimensions/signature, bounded startup retry/recovery/cancellation\n";
}
