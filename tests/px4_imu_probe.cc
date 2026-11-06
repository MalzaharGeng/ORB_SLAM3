#include "Px4Imu.h"
#include "Px4Vio.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
int main(int argc,char** argv) {
    ORB_SLAM3::px4::ImuReceiver receiver(argc>1?std::stoi(argv[1]):0,argc>2?argv[2]:"");
    int valid=0;double previous=0;uint64_t priorEpoch=0;
    for(int i=0;i<15;++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const double end=ORB_SLAM3::px4::SystemMicros()*1e-6-.02;
        std::vector<ORB_SLAM3::px4::ImuReading> out;uint64_t epoch;std::string reason;
        const bool ok=receiver.Interval(previous?previous:end-.02,end,out,epoch,reason);
        double acceleration=0;
        if(ok) {for(float a:out.back().accel)acceleration+=a*a;if(!previous || epoch==priorEpoch)++valid;}
        std::cout<<receiver.Status()<<" interval="<<reason<<" samples="<<out.size()<<" accel_norm="<<std::sqrt(acceleration)<<std::endl;
        previous=end;priorEpoch=epoch;
    }
    return valid>=10?0:1;
}
