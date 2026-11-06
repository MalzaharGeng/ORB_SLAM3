#include "LatestCameraInput.h"
#include <atomic>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
static void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
int main() {
    try {
        ORB_SLAM3::LatestCameraInput single;ORB_SLAM3::TimedCameraInput item;
        for(int i=1;i<=3;++i)require(single.Put({cv::Mat(2,2,CV_8UC1,cv::Scalar(i)),double(i),double(i)}),"put rejected");
        require(single.Dropped()==2,"waiting frames not replaced");
        require(single.Take(item) && item.timestamp==3 && item.image.at<unsigned char>(0,0)==3,"latest timestamp/image separated");
        single.Stop();require(!single.Take(item),"stop did not wake empty consumer");
        require(!single.Put({}),"closed mailbox accepted input");
        ORB_SLAM3::LatestCameraInput queue;std::atomic<int> accepted{0},consumed{0};std::atomic<bool> good{true};
        std::thread consumer([&] {
            ORB_SLAM3::TimedCameraInput input;int previous=0;
            while(queue.Take(input)) {
                const int n=int(input.timestamp);
                if(n<=previous || input.grabMs!=input.timestamp ||
                   input.image.at<unsigned char>(0,0)!=(n%256))good=false;
                previous=n;++consumed;
            }
        });
        for(int i=1;i<=10000;++i) {
            if(queue.Put({cv::Mat(3,3,CV_8UC1,cv::Scalar(i%256)),double(i),double(i)}))++accepted;
        }
        queue.Stop();consumer.join();
        require(good,"threaded image/timestamp mismatch or reordering");
        require(consumed>0 && consumed+queue.Dropped()==accepted,"unaccounted capture loss");
        ORB_SLAM3::LatestCameraInput empty;
        auto waiter=std::async(std::launch::async,[&]{ORB_SLAM3::TimedCameraInput x;return empty.Take(x);});
        empty.Stop();require(!waiter.get(),"empty shutdown deadlock");
        std::cout<<"LATEST_CAMERA_INPUT_PASS threaded_inputs="<<accepted<<" consumed="<<consumed
            <<" overwritten="<<queue.Dropped()<<" paired_timestamps=1 bounded_waiting_frames=1 stop_wakeup=1\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
