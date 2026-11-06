#include "CudaRectify.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>
using ORB_SLAM3::CudaStereoRectifier;
using Clock=std::chrono::steady_clock;
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static double median(std::vector<double> values) {
    std::sort(values.begin(),values.end());return values[values.size()/2];
}
static void compare(CudaStereoRectifier& gpu,const cv::Mat& image,cv::Rect l,cv::Rect r,
                    cv::Mat* mx,cv::Mat* my,std::vector<double>& cpu,std::vector<double>& cuda) {
    cv::Mat ref[2],out[2];auto start=Clock::now();
    cv::remap(image(l),ref[0],mx[0],my[0],cv::INTER_LINEAR);
    cv::remap(image(r),ref[1],mx[1],my[1],cv::INTER_LINEAR);
    cpu.push_back(std::chrono::duration<double,std::milli>(Clock::now()-start).count());
    start=Clock::now();const auto before=gpu.Calls();
    require(gpu.Remap(image,l,r,mx[0],my[0],mx[1],my[1],out[0],out[1]),"GPU remap fallback during equivalence test");
    cuda.push_back(std::chrono::duration<double,std::milli>(Clock::now()-start).count());
    require(gpu.Calls()==before+1,"CUDA invocation missing");
    for(int eye=0;eye<2;++eye) {
        const int changed=cv::countNonZero(ref[eye]!=out[eye]);
        if(changed)std::cerr<<"different_pixels="<<changed<<" max="<<cv::norm(ref[eye],out[eye],cv::NORM_INF)<<std::endl;
        require(!changed,"GPU changed rectified image bytes");
    }
}
int main(int argc,char** argv) {
    try {
        setenv("ORB_SLAM3_RECTIFY_BACKEND","cuda",1);cv::setNumThreads(6);
        std::vector<double> cpu,cuda;cv::RNG rng(54913);int synthetic=0,recorded=0;
        for(int kind=0;kind<4;++kind) {
            cv::Mat backing(103,269,CV_8UC1);rng.fill(backing,cv::RNG::UNIFORM,0,256);
            cv::Mat image=backing(cv::Rect(3,2,260,99)); // Non-contiguous input.
            if(kind==1)image.setTo(127);
            if(kind==2)image.setTo(255);
            if(kind==3)image.setTo(0);
            cv::Mat mx[2],my[2];
            for(int eye=0;eye<2;++eye) {
                cv::Mat x(71,113,CV_32F),y(71,113,CV_32F);
                rng.fill(x,cv::RNG::UNIFORM,-3.,132.);rng.fill(y,cv::RNG::UNIFORM,-3.,102.);
                // Exercise every one of the 1024 fractional coefficients.
                for(int i=0;i<1024;++i){x.at<float>(i/113,i%113)=10+(i%32)/32.f;y.at<float>(i/113,i%113)=20+(i/32)/32.f;}
                cv::Mat a,b;cv::convertMaps(x,y,a,b,CV_16SC2);
                cv::Mat paddedA(73,117,CV_16SC2),paddedB(73,117,CV_16UC1);
                mx[eye]=paddedA(cv::Rect(2,1,113,71));my[eye]=paddedB(cv::Rect(2,1,113,71));
                a.copyTo(mx[eye]);b.copyTo(my[eye]);
            }
            CudaStereoRectifier gpu;
            compare(gpu,image,cv::Rect(0,0,130,99),cv::Rect(130,0,130,99),mx,my,cpu,cuda);
            compare(gpu,image,cv::Rect(0,0,130,99),cv::Rect(130,0,130,99),mx,my,cpu,cuda);
            unsetenv("ORB_SLAM3_RECTIFY_BACKEND");cv::Mat l,r;
            require(!gpu.Remap(image,cv::Rect(0,0,130,99),cv::Rect(130,0,130,99),mx[0],my[0],mx[1],my[1],l,r),"CPU backend unexpectedly uses CUDA");
            setenv("ORB_SLAM3_RECTIFY_BACKEND","cuda",1);
            // Geometry/map changes must explicitly fall back instead of using stale maps.
            cv::Mat changed=mx[0].clone();
            require(!gpu.Remap(image,cv::Rect(0,0,130,99),cv::Rect(130,0,130,99),changed,my[0],mx[1],my[1],l,r),"changed maps accepted");
            ++synthetic;
        }
        std::vector<double> realCpu,realCuda;
        if(argc>=3) {
            const int maximum=argc>=4?std::stoi(argv[3]):40;
            require(maximum>0,"invalid sample limit");
            cv::FileStorage fs(argv[1],cv::FileStorage::READ);cv::Mat mx[2],my[2];
            for(int eye=0;eye<2;++eye) {
                const auto n=std::to_string(eye+1);cv::Mat k,d,r,p;
                fs["K"+n]>>k;fs["D"+n]>>d;fs["R"+n]>>r;fs["P"+n]>>p;
                cv::initUndistortRectifyMap(k,d,r,p,cv::Size(1280,960),CV_16SC2,mx[eye],my[eye]);
            }
            std::vector<std::filesystem::path> files;
            for(auto& f:std::filesystem::directory_iterator(std::filesystem::path(argv[2])/"frames"))
                if(f.path().extension()==".jpg")files.push_back(f.path());
            std::sort(files.begin(),files.end());CudaStereoRectifier gpu;
            for(size_t i=0;i<files.size() && recorded<maximum;i+=std::max(size_t(1),files.size()/maximum)) {
                auto image=cv::imread(files[i].string(),0);if(image.size()!=cv::Size(2720,960))continue;
                compare(gpu,image,cv::Rect(160,0,1280,960),cv::Rect(1440,0,1280,960),mx,my,realCpu,realCuda);++recorded;
            }
            require(recorded>=20,"too few real samples");
        }
        std::cout<<"CUDA_RECTIFY_EQUIVALENCE_PASS synthetic="<<synthetic<<" repeated=2 recorded="<<recorded
            <<" cpu_median_ms="<<(realCpu.empty()?median(cpu):median(realCpu))
            <<" cuda_median_ms="<<(realCuda.empty()?median(cuda):median(realCuda))
            <<" changed_pixels=0 border_and_stride=1 changed_maps_fallback=1"<<std::endl;
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
