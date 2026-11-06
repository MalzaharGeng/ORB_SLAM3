#include "Px4ImuBuffer.h"
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3::px4;
static void require(bool value) {if(!value)throw std::runtime_error("IMU regression failed");}
int main() {
    ImuBuffer b;std::vector<ImuReading> out;std::string reason;
    for(int i=0;i<=10;++i) {ImuReading s;s.time=1.+i*.005;s.accel[0]=i;b.Add(s);}
    require(b.Slice(1.0025,1.0275,out,reason));
    require(std::abs(out.front().accel[0]-.5)<1e-5 && std::abs(out.back().accel[0]-5.5)<1e-5);
    require(out.size()==7 && out.front().time==1.0025 && out.back().time==1.0275);
    require(!b.Slice(1.0275,1.2,out,reason) && reason=="waiting_imu" && out.empty());
    require(!b.Slice(.5,1.03,out,reason) && reason=="imu_interval_uncovered");
    ImuReading s;s.time=2.;b.Add(s);require(b.epoch==1 && b.data.size()==1);
    s.time=1.9;b.Add(s);require(b.epoch==2 && b.data.size()==1);
    s.time=NAN;require(!b.Add(s) && b.epoch==3 && b.data.empty());
    for(int i=0;i<10000;++i) {s.time=3.+i*.005;b.Add(s);}
    require(b.data.size()==2048);
    require(!b.Slice(52.99,52.98,out,reason));
    std::cout<<"PASS interpolation, full coverage, future wait, gap, clock reset, finite input, bounded retention\n";
}
