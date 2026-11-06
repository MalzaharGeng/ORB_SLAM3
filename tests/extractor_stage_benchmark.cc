#include "ORBextractor.h"
#include "Frame.h"
#include "Pinhole.h"
#include "ORBVocabulary.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
#include <vector>
using namespace ORB_SLAM3;
using Clock=std::chrono::steady_clock;
class ExposedExtractor:public ORBextractor {
public:
    using ORBextractor::ORBextractor;
    using ORBextractor::ComputePyramid;
    using ORBextractor::ComputeKeyPointsOctTree;
};
template<class L,class R> double pairTime(L left,R right) {
    auto start=Clock::now();std::thread a(left),b(right);a.join();b.join();
    return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
static double median(std::vector<double> v) {std::sort(v.begin(),v.end());return v[v.size()/2];}
int main(int argc,char** argv) {
    if(argc<4)return 2;
    if(argc>4)cv::setNumThreads(std::stoi(argv[4]));
    cv::FileStorage fs(argv[2],cv::FileStorage::READ);
    cv::Mat mx[2],my[2],k,p2;
    for(int i=0;i<2;++i) {
        std::string eye=std::to_string(i+1);cv::Mat a,d,r,p;
        fs["K"+eye]>>a;fs["D"+eye]>>d;fs["R"+eye]>>r;fs["P"+eye]>>p;
        cv::initUndistortRectifyMap(a,d,r,p,cv::Size(1280,960),CV_16SC2,mx[i],my[i]);
        if(!i)k=p(cv::Rect(0,0,3,3)).clone();else p2=p;
    }
    k.convertTo(k,CV_32F);const float bf=float(-p2.at<double>(0,3)),baseline=bf/k.at<float>(0,0);
    cv::Mat dist=cv::Mat::zeros(4,1,CV_32F);
    Pinhole camera({k.at<float>(0,0),k.at<float>(1,1),k.at<float>(0,2),k.at<float>(1,2)});
    ORBVocabulary voc;if(!voc.loadFromTextFile(argv[1]))return 3;
    ExposedExtractor le(1000,1.2,8,20,7),re(1000,1.2,8,20,7);
    std::vector<std::filesystem::path> paths;
    for(auto& entry:std::filesystem::directory_iterator(std::filesystem::path(argv[3])/"frames"))if(entry.path().extension()==".jpg")paths.push_back(entry.path());
    std::sort(paths.begin(),paths.end());
    std::vector<double> preprocess,full,pyramid,keypoints,frame,stereo;int samples=0;
    for(size_t idx=0;idx<paths.size() && samples<18;idx+=std::max(size_t(1),paths.size()/18)) {
        cv::Mat image=cv::imread(paths[idx].string(),0);if(image.size()!=cv::Size(2720,960))continue;
        cv::Mat left,right;auto prep=Clock::now();
        cv::remap(image(cv::Rect(160,0,1280,960)),left,mx[0],my[0],cv::INTER_LINEAR);
        cv::remap(image(cv::Rect(1440,0,1280,960)),right,mx[1],my[1],cv::INTER_LINEAR);
        preprocess.push_back(std::chrono::duration<double,std::milli>(Clock::now()-prep).count());
        std::vector<cv::KeyPoint> lk,rk;cv::Mat ld,rd;std::vector<int> overlap{0,0},overlapR{0,0};
        full.push_back(pairTime([&]{le(left,cv::Mat(),lk,ld,overlap);},[&]{re(right,cv::Mat(),rk,rd,overlapR);}));
        pyramid.push_back(pairTime([&]{le.ComputePyramid(left);},[&]{re.ComputePyramid(right);}));
        std::vector<std::vector<cv::KeyPoint>> kl,kr;
        keypoints.push_back(pairTime([&]{le.ComputeKeyPointsOctTree(kl);},[&]{re.ComputeKeyPointsOctTree(kr);}));
        auto start=Clock::now();
        Frame f(left,right,idx*.15,&le,&re,&voc,k,dist,bf,baseline*40,&camera);
        frame.push_back(std::chrono::duration<double,std::milli>(Clock::now()-start).count());
        start=Clock::now();f.ComputeStereoMatches();
        stereo.push_back(std::chrono::duration<double,std::milli>(Clock::now()-start).count());
        int depths=std::count_if(f.mvDepth.begin(),f.mvDepth.end(),[](float z){return z>0;});
        std::cout<<"PROFILE sample="<<samples<<" pair_features_ms="<<full.back()<<" pyramid_ms="<<pyramid.back()
            <<" keypoints_ms="<<keypoints.back()<<" frame_ms="<<frame.back()<<" stereo_ms="<<stereo.back()
            <<" features="<<lk.size()<<" depths="<<depths<<std::endl;++samples;
    }
    if(samples<5)return 4;
    std::cout<<"PROFILE_SUMMARY samples="<<samples<<" cv_threads="<<cv::getNumThreads()
        <<" preprocess_ms="<<median(preprocess)<<" pair_features_ms="<<median(full)
        <<" pyramid_ms="<<median(pyramid)<<" keypoints_orientation_ms="<<median(keypoints)
        <<" frame_ms="<<median(frame)<<" stereo_ms="<<median(stereo)<<std::endl;
}
