#include "ORBextractor.h"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <iostream>
using namespace ORB_SLAM3;
int main(int argc,char** argv) {
    if(argc!=2)return 2;
    std::vector<std::filesystem::path> paths;
    for(auto& f:std::filesystem::directory_iterator(std::filesystem::path(argv[1])/"left"))if(f.path().extension()==".png")paths.push_back(f.path());
    std::sort(paths.begin(),paths.end());
    if(paths.empty())return 3;
    std::cout<<"nfeatures,fast_initial,fast_min,pair_median_ms,pair_p95_ms,mean_left_features\n";
    for(int count:{800,1000,1200})for(int threshold:{20,15}) {
        ORBextractor le(count,1.2,8,threshold,threshold==20?7:5),re(count,1.2,8,threshold,threshold==20?7:5);
        std::vector<double> timing;double features=0;
        for(size_t i=0;i<std::min(size_t(25),paths.size());++i) {
            auto left=cv::imread(paths[i].string(),0),right=cv::imread((paths[i].parent_path().parent_path()/"right"/paths[i].filename()).string(),0);
            std::vector<cv::KeyPoint> lk,rk;cv::Mat ld,rd;std::vector<int> overlap{0,0},overlapR{0,0};
            auto start=std::chrono::steady_clock::now();
            std::thread l([&]{le(left,cv::Mat(),lk,ld,overlap);});std::thread r([&]{re(right,cv::Mat(),rk,rd,overlapR);});l.join();r.join();
            timing.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());features+=lk.size();
        }
        std::sort(timing.begin(),timing.end());
        std::cout<<count<<","<<threshold<<","<<(threshold==20?7:5)<<","<<timing[timing.size()/2]<<","<<timing[size_t((timing.size()-1)*.95)]<<","<<features/timing.size()<<"\n";
    }
}
