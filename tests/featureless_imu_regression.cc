// A frame without features still participates in IMU integration after reset.
#include "Frame.h"
#include "ORBextractor.h"
#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"
#include <iostream>
#include <stdexcept>

using namespace ORB_SLAM3;
int main() {
    cv::Mat blank(480,640,CV_8U,cv::Scalar(0));
    cv::Mat depth(480,640,CV_32F,cv::Scalar(1));
    cv::Mat K=(cv::Mat_<float>(3,3)<<400,0,320,0,400,240,0,0,1);
    cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F);
    ORBVocabulary vocabulary;
    ORBextractor left(1000,1.2,8,20,7),right(1000,1.2,8,20,7);
    Pinhole camera(std::vector<float>{400,400,320,240});
    auto check=[](Frame& frame) {
        if(frame.N!=0) throw std::runtime_error("Fixture unexpectedly has features");
        frame.setIntegrated();
        if(!frame.imuIsPreintegrated()) throw std::runtime_error("Empty frame cannot record IMU integration");
    };
    std::cout<<"Checking featureless stereo frame"<<std::endl;
    Frame stereo(blank,blank,1.,&left,&right,&vocabulary,K,distortion,20.f,40.f,&camera);
    check(stereo);
    Frame rgbd(blank,depth,1.,&left,&vocabulary,K,distortion,20.f,40.f,&camera);
    check(rgbd);
    Frame mono(blank,1.,&left,&vocabulary,&camera,distortion,20.f,40.f);
    check(mono);
    Sophus::SE3f baseline(Sophus::SO3f(),Eigen::Vector3f(-.05,0,0));
    KannalaBrandt8 fisheye(std::vector<float>{400,400,320,240,0,0,0,0});
    fisheye.mvLappingArea={0,640};
    Frame stereoTwoCameras(blank,blank,1.,&left,&right,&vocabulary,K,distortion,20.f,40.f,&fisheye,&fisheye,baseline);
    check(stereoTwoCameras);
    std::cout<<"FEATURELESS_IMU_PASS: all image constructors retain a valid IMU mutex"<<std::endl;
}
