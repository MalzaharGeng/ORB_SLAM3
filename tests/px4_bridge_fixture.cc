#include "Px4Bridge.h"
#include <chrono>
#include <iostream>
#include <poll.h>
#include <thread>
#include <unistd.h>
using namespace ORB_SLAM3::px4;
int main(int argc,char** argv) {
    try {
        Config c;c.extrinsicsValid=true;c.domainId=43;c.topicPrefix="/vio_test";
        if(argc==2 && std::string(argv[1])=="--sitl") {c.domainId=44;c.topicPrefix="/vio_sitl";}
        else if(argc!=1)throw std::runtime_error("Usage: px4_bridge_fixture [--sitl]");
        c.logPath="px4_fixture.csv";
        Bridge bridge(c);
        Sample s;s.valid=s.imuInitialized=s.ba2=s.velocityValid=true;s.gravity={0,0,-9.81};
        s.imuToWorld=Eigen::Vector3d(1,-1,-1).asDiagonal();
        bool stale=false;std::string input;
        for(;;) {
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::microseconds(33333);
            pollfd fd{STDIN_FILENO,POLLIN,0};
            if(poll(&fd,1,0)>0 && (fd.revents&(POLLIN|POLLHUP))) {
                char bytes[256];const auto count=read(STDIN_FILENO,bytes,sizeof(bytes));
                if(count<=0)break;
                input.append(bytes,count);size_t newline;
                while((newline=input.find('\n'))!=std::string::npos) {
                    const auto command=input.substr(0,newline);input.erase(0,newline+1);
                    if(command=="quit")return 0;
                    if(command=="lost")s.valid=false;
                    else if(command=="resume") {s.valid=true;stale=false;}
                    else if(command=="reset") {++s.referenceEpoch;s.position.x()+=3.;}
                    else if(command=="clock") {++s.clockEpoch;bridge.InvalidateClock(s.clockEpoch);}
                    else if(command=="stale")stale=true;
                    else if(command=="velocity")s.velocity.x()=.25;
                    else if(command=="stationary")s.velocity.setZero();
                }
            }
            s.position+=s.velocity/30.;
            s.sampleUs=SystemMicros()-(stale?500000:10000);bridge.Submit(s);
            std::this_thread::sleep_until(deadline);
        }
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
