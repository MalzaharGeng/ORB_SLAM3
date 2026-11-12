#include "ORBextractor.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

struct Result {int mono=0;std::vector<cv::KeyPoint> keys;cv::Mat desc;};
static Result extract(ORB_SLAM3::ORBextractor& extractor,const cv::Mat& image) {
    Result result;std::vector<int> overlap{0,0};
    result.mono=extractor(image,cv::Mat(),result.keys,result.desc,overlap);return result;
}
static void compare(const Result& a,const Result& b) {
    if(a.mono!=b.mono || a.keys.size()!=b.keys.size() || a.desc.size()!=b.desc.size() || a.desc.type()!=b.desc.type())
        throw std::runtime_error("Descriptor mode changed feature count/layout");
    if(!a.desc.empty() && cv::norm(a.desc,b.desc,cv::NORM_INF)!=0)
        throw std::runtime_error("Descriptor bytes differ");
    for(size_t i=0;i<a.keys.size();++i) {
        const auto& x=a.keys[i];const auto& y=b.keys[i];
        if(x.pt!=y.pt || x.response!=y.response || x.angle!=y.angle || x.size!=y.size || x.octave!=y.octave || x.class_id!=y.class_id)
            throw std::runtime_error("Descriptor mode changed keypoint/order");
    }
}
int main(int argc,char** argv) {
    try {
        cv::RNG rng(29183);std::vector<cv::Mat> images;
        unsetenv("ORB_SLAM3_DESCRIPTOR_CONTROL_FILE");
        for(int kind=0;kind<5;++kind) {
            cv::Mat backing(740,984,CV_8UC1);rng.fill(backing,cv::RNG::UNIFORM,0,256);
            cv::Mat image=backing(cv::Rect(5,7,960,720));
            if(kind==1)cv::GaussianBlur(image,image,cv::Size(7,7),2);
            if(kind==2)image.setTo(128);
            if(kind==3) {image.setTo(0);cv::rectangle(image,{150,100,600,450},cv::Scalar(255),-1);}
            if(kind==4)cv::normalize(image,image,100,140,cv::NORM_MINMAX);
            images.push_back(image);
        }
        if(argc>1)for(const auto& file:std::filesystem::directory_iterator(argv[1])) {
            if(file.path().extension()!=".png")continue;
            auto image=cv::imread(file.path().string(),0);
            if(image.size()!=cv::Size(960,720))throw std::runtime_error("Expected active resolution");
            images.push_back(image);
        }
        size_t comparisons=0;
        for(const char* backend:{"cpu","cuda"})for(int threads:{1,3,6}) {
            setenv("ORB_SLAM3_FAST_BACKEND",backend,1);setenv("ORB_SLAM3_PARALLEL_LEVELS","1",1);cv::setNumThreads(threads);
            ORB_SLAM3::ORBextractor left(1000,1.2,8,20,7),right(1000,1.2,8,20,7);
            for(size_t i=0;i<images.size();++i) {
                const auto& l=images[i];const auto& r=images[(i+1)%images.size()];Result a,b,c,d;
                setenv("ORB_SLAM3_PARALLEL_DESCRIPTORS","0",1);
                std::thread ls([&]{a=extract(left,l);}),rs([&]{b=extract(right,r);});ls.join();rs.join();
                setenv("ORB_SLAM3_PARALLEL_DESCRIPTORS","1",1);
                std::thread lp([&]{c=extract(left,l);}),rp([&]{d=extract(right,r);});lp.join();rp.join();
                compare(a,c);compare(b,d);comparisons+=2;
            }
        }
        std::cout<<"PASS descriptor serial/parallel equivalence: "<<comparisons
                 <<" exact image comparisons, 1000 features, synthetic/real/noncontiguous/reused buffers, CPU/CUDA, 1/3/6 threads\n";
    }catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
