#include "Px4Vio.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>
using namespace ORB_SLAM3::px4;
static void require(bool ok,const char* text) {if(!ok)throw std::runtime_error(text);}
static void near(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const char* what) {require((a-b).norm()<1e-8,what);}
struct Run {
    uint64_t wall=1700000000000000ULL,mono=1000000;
    Sample s;Telemetry t;Gate gate;
    explicit Run(Config c):gate(c) {
        s.valid=s.imuInitialized=s.ba2=s.velocityValid=true;
        s.gravity={0,0,-9.81};
        t.haveStatus=t.haveSync=t.haveReader=true;
    }
    std::optional<Observation> step(uint64_t dt=33333) {
        wall+=dt;mono+=dt;++s.sequence;s.sampleUs=wall-10000;
        t.statusSteadyUs=t.syncSteadyUs=mono;
        return gate.Evaluate(s,t,wall,mono);
    }
    Observation ready() {
        for(int i=0;i<40;++i)if(auto o=step())return *o;
        throw std::runtime_error("never ready: "+gate.Reason());
    }
};
int main(int argc,char** argv) {
try {
    require(argc==2,"repo path missing");
    auto placeholder=Config::Load(std::string(argv[1])+"/config/px4_vio.yaml");
    require(!placeholder.extrinsicsValid,"example must not enable uncalibrated observations");
    Run missing(placeholder);for(int i=0;i<60;++i)require(!missing.step(),"uncalibrated publication");
    Config c;c.extrinsicsValid=true;
    Run nominal(c);auto first=nominal.ready();near(first.position,Eigen::Vector3d::Zero(),"zero initial centre");
    nominal.s.position={1,2,3};nominal.s.velocity={4,5,6};auto moved=nominal.step();require(bool(moved),"motion missing");
    near(moved->position,{1,-2,-3},"FLU world to FRD axes");near(moved->velocity,{4,-5,-6},"velocity axes");
    require(!nominal.gate.Evaluate(nominal.s,nominal.t,nominal.wall,nominal.mono),"duplicate publication");
    // Tilted arbitrary navigation coordinates plus nontrivial IMU mounting.
    const Eigen::Matrix3d worldFromBody=(Eigen::AngleAxisd(.8,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.3,Eigen::Vector3d::UnitX())).toRotationMatrix();
    c.bodyFromImu=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    const Eigen::Vector3d imuPositionBody(.15,-.04,.07);Run mounted(c);
    mounted.s.imuToWorld=worldFromBody*c.bodyFromImu;
    mounted.s.gravity=worldFromBody*Eigen::Vector3d(0,0,9.81);
    mounted.s.position=Eigen::Vector3d(2,3,4)+worldFromBody*imuPositionBody;
    auto base=mounted.ready();near(base.position,Eigen::Vector3d::Zero(),"origin must be the initial IMU centre");
    require(base.attitude.angularDistance(Eigen::Quaterniond::Identity())<1e-8,"body quaternion / mounting direction");
    mounted.s.position+=worldFromBody*Eigen::Vector3d(1,2,-3);
    mounted.s.velocity=worldFromBody*Eigen::Vector3d(2,-1,4);
    auto mo=mounted.step();near(mo->position,{1,2,-3},"tilted frame displacement");
    near(mo->velocity,{2,-1,4},"tilted frame velocity");
    // An ordinary motion or reconnect keeps the reference; true discontinuities require disarmed.
    mounted.t.armed=true;require(bool(mounted.step()),"existing reference must work armed");
    ++mounted.s.referenceEpoch;require(!mounted.step(),"map reset published while armed");
    for(int i=0;i<40;++i)require(!mounted.step(),"reanchor while armed");
    mounted.t.armed=false;require(mounted.ready().resetCounter==1,"reference reset counter");
    mounted.t.haveReader=false;require(!mounted.step(),"link loss publication");
    mounted.t.haveReader=true;require(mounted.ready().resetCounter==1,"reconnect should preserve reference");
    mounted.s.valid=false;require(!mounted.step(),"lost tracking publication");
    mounted.s.valid=true;++mounted.s.continuity;require(!mounted.step(),"tracking resume must stabilize");mounted.ready();
    mounted.s.velocity.x()=std::numeric_limits<double>::quiet_NaN();require(!mounted.step(),"NaN velocity publication");
    mounted.s.velocity.setZero();mounted.ready();
    mounted.t.armed=true;++mounted.s.clockEpoch;require(!mounted.step(),"clock transition must invalidate reference");
    mounted.t.armed=false;require(mounted.ready().resetCounter==2,"clock reset counter");
    auto old=mounted.s;old.sequence++;old.sampleUs=mounted.wall-500000;
    require(!mounted.gate.Evaluate(old,mounted.t,mounted.wall,mounted.mono),"historical timestamp published");
    Run init(c);init.s.ba2=false;for(int i=0;i<60;++i)require(!init.step(),"pre-VIBA2 publication");
    init.s.ba2=true;init.t.observedOffsetUs=100000;require(!init.step(),"unconverged timesync");
    init.t.observedOffsetUs=0;init.ready();
    init.wall+=5000000;init.t.armed=true;require(!init.step(),"host realtime jump publication");
    // Per-stream clock checks cover mixed domains, normal interleaving and regression.
    SensorClock clock;const uint64_t w=1700000000000000ULL,m=1000000;
    for(unsigned i=0;i<4;++i)require(clock.Accept(i,true,w*.001,w,m),"initial global timestamp");
    require(clock.Ready(),"all four domains ready");
    require(clock.Accept(2,true,(w+5000)*.001,w+6000,m+6000),"normal IMU sample");
    require(!clock.Accept(3,false,(w+5000)*.001,w+6000,m+6000),"hardware clock admitted");
    require(!clock.Ready() && clock.Epoch()==1,"domain transition invalidates all streams");
    require(clock.Accept(2,true,(w+7000)*.001,w+7000,m+7000),"clock recovery");
    require(!clock.Accept(2,true,(w+6000)*.001,w+8000,m+8000),"backward sensor timestamp");
    for(unsigned i=0;i<4;++i)require(clock.Accept(i,true,(w+9000)*.001,w+9000,m+9000),"clock reestablishment");
    const auto clockEpoch=clock.Epoch();
    require(!clock.Accept(2,true,(w+300000)*.001,w+300000,m+300000),"stalled sensor stream admitted");
    require(clock.Epoch()>clockEpoch && !clock.Ready(),"sensor stall must invalidate all buffers");
    require(TopicName("","/fmu/out/vehicle_status",1)=="/fmu/out/vehicle_status_v1","versioned topic");
    // Invalid calibration is rejected before any transport is started.
    c.bodyFromImu(0,0)=7;bool rejected=false;try{c.Validate();}catch(...){rejected=true;}
    require(rejected,"non-rigid calibration accepted");
    std::cout<<"PX4_VIO_REGRESSION_PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
