#include "ORBextractor.h"
#include "CudaFast.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace ORB_SLAM3;
static void require(bool b,const char* text){if(!b)throw std::runtime_error(text);}
static void compare(const cv::Mat& image,int threshold,int minimum,std::vector<double>& cpuMs,std::vector<double>& gpuMs) {
    ORBextractor cpu(1000,1.2,8,threshold,minimum),gpu(1000,1.2,8,threshold,minimum);
    std::vector<cv::KeyPoint> a,b;cv::Mat da,db;std::vector<int> overlap{0,0};
    const char* parallelValue=std::getenv("ORB_SLAM3_PARALLEL_LEVELS");
    const bool parallel=parallelValue && std::strcmp(parallelValue,"1")==0;
    unsetenv("ORB_SLAM3_PARALLEL_LEVELS");
    setenv("ORB_SLAM3_FAST_BACKEND","cpu",1);auto start=std::chrono::steady_clock::now();
    const int countA=cpu(image,cv::Mat(),a,da,overlap);
    cpuMs.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
    if(parallel)setenv("ORB_SLAM3_PARALLEL_LEVELS","1",1);
    setenv("ORB_SLAM3_FAST_BACKEND","cuda",1);const auto calls=CudaFastCalls();start=std::chrono::steady_clock::now();
    const int countB=gpu(image,cv::Mat(),b,db,overlap);
    gpuMs.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
    require(CudaFastCalls()==calls+1,"CUDA path not executed");
    require(countA==countB && a.size()==b.size(),"GPU FAST changed feature count/order");
    for(size_t i=0;i<a.size();++i) {
        require(a[i].pt==b[i].pt && a[i].response==b[i].response && a[i].angle==b[i].angle &&
                a[i].octave==b[i].octave && a[i].size==b[i].size,"GPU FAST changed keypoint");
    }
    require(da.size()==db.size(),"GPU FAST descriptor count changed");
    if(!da.empty())require(std::memcmp(da.data,db.data,da.total())==0,"GPU FAST descriptor bytes changed");
}
int main(int argc,char** argv) {
    try {
        cv::setNumThreads(6);cv::RNG rng(8923);
        std::vector<double> cpu,gpu;int synthetic=0,recorded=0;
        for(int kind=0;kind<4;++kind) {
            cv::Mat im(960,1280,CV_8UC1);rng.fill(im,cv::RNG::UNIFORM,0,256);
            if(kind==1)cv::GaussianBlur(im,im,cv::Size(5,5),1.2);
            if(kind==2)im.setTo(127);
            if(kind==3){im.setTo(0);cv::rectangle(im,cv::Rect(200,200,600,400),cv::Scalar(255),-1);}
            compare(im,20,7,cpu,gpu);++synthetic;
        }
        if(argc==3) {
            cv::FileStorage fs(argv[1],cv::FileStorage::READ);
            cv::Mat mx[2],my[2];
            for(int eye=0;eye<2;++eye) {
                std::string n=std::to_string(eye+1);cv::Mat k,d,r,p;
                fs["K"+n]>>k;fs["D"+n]>>d;fs["R"+n]>>r;fs["P"+n]>>p;
                cv::initUndistortRectifyMap(k,d,r,p,cv::Size(1280,960),CV_16SC2,mx[eye],my[eye]);
            }
            std::vector<std::filesystem::path> files;
            for(auto& f:std::filesystem::directory_iterator(std::filesystem::path(argv[2])/"frames"))files.push_back(f.path());
            std::sort(files.begin(),files.end());
            for(size_t idx=0;idx<files.size() && recorded<20;idx+=std::max(size_t(1),files.size()/10)) {
                auto raw=cv::imread(files[idx].string(),0);if(raw.size()!=cv::Size(2720,960))continue;
                for(int eye=0;eye<2;++eye) {
                    cv::Mat im;cv::remap(raw(cv::Rect(eye?1440:160,0,1280,960)),im,mx[eye],my[eye],cv::INTER_LINEAR);
                    compare(im,20,7,cpu,gpu);++recorded;
                }
            }
            require(recorded>=16,"too few recorded image samples");
        }
        // Concurrent feature threads reuse a bounded pool of CUDA streams/storage.
        cv::Mat l(960,1280,CV_8UC1),r(960,1280,CV_8UC1);rng.fill(l,cv::RNG::UNIFORM,0,256);rng.fill(r,cv::RNG::UNIFORM,0,256);
        ORBextractor le(1000,1.2,8,20,7),re(1000,1.2,8,20,7);
        std::vector<cv::KeyPoint> lk,rk;cv::Mat ld,rd;std::vector<int> ov{0,0},ov2{0,0};
        const auto before=CudaFastCalls();
        std::thread left([&]{le(l,cv::Mat(),lk,ld,ov);}),right([&]{re(r,cv::Mat(),rk,rd,ov2);});left.join();right.join();
        require(CudaFastCalls()==before+2 && !ld.empty() && !rd.empty(),"concurrent CUDA extraction failed");
        std::sort(cpu.begin(),cpu.end());std::sort(gpu.begin(),gpu.end());
        std::cout<<"CUDA_EXTRACTOR_EQUIVALENCE_PASS synthetic="<<synthetic<<" recorded="<<recorded
            <<" cpu_median_ms="<<cpu[cpu.size()/2]<<" cuda_median_ms="<<gpu[gpu.size()/2]
            <<" cuda_calls="<<CudaFastCalls()<<std::endl;return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
