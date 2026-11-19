#include "Px4Vio.h"
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace ORB_SLAM3::px4;
// Run this identical driver against both old and new cores. Compare actual
// decisions, health, reset counters and published numeric values byte-for-byte.
int main(){
    std::cout<<std::setprecision(17);
    for(int scenario=0;scenario<24;++scenario){
        Config c;c.extrinsicsValid=true;c.stableSeconds=.1;c.maxAgeSeconds=.25;c.maxSampleGap=.25;
        c.allowArmedLandedRecovery=scenario%2;
        Gate gate(c);Sample s;Telemetry t;
        s.valid=s.imuInitialized=s.ba2=s.velocityValid=true;s.gravity={0,0,-9.81};
        t.haveStatus=t.haveSync=t.haveReader=t.haveNavigationStatus=t.haveFusionStatus=true;
        t.navigationPositionValid=t.navigationVelocityValid=t.evPosition=true;t.navState=1;
        uint64_t wall=1700000000000000ULL,mono=1000000;
        for(int i=0;i<160;++i){
            uint64_t dt=(scenario==5 && i==70)?400000:20000;wall+=dt;mono+=dt;
            ++s.sequence;s.sampleUs=wall-10000;s.position.x()+=.0002;s.velocity={.01,0,0};
            t.statusSteadyUs=t.syncSteadyUs=t.localSteadyUs=t.fusionSteadyUs=mono;
            t.armed=i>=30 && i<110;t.haveReader=!(scenario==2 && i>=60 && i<65);
            if(i==70){
                switch(scenario){
                case 0:s.position.x()+=2.;break;
                case 1:s.imuToWorld=Eigen::AngleAxisd(1.,Eigen::Vector3d::UnitX()).toRotationMatrix();break;
                case 3:++s.referenceEpoch;break;
                case 4:++s.clockEpoch;break;
                case 6:s.sampleUs=wall-300000;break;
                case 7:s.velocity={10,0,0};break;
                case 8:s.valid=false;break;
                case 9:s.ba2=false;break;
                case 10:s.velocityValid=false;break;
                case 11:s.sampleUs=wall+10000;break;
                case 12:s.position.y()=NAN;break;
                default:++s.continuity;break;
                }
            }
            if(i==80){s.valid=s.ba2=s.velocityValid=true;s.position={.016,0,0};s.imuToWorld=Eigen::Matrix3d::Identity();}
            const auto o=gate.Evaluate(s,t,wall,mono);const auto h=gate.Health(t,mono);
            std::cout<<scenario<<","<<i<<","<<gate.Reason()<<","<<gate.Sent()<<","<<gate.Rejected()
                <<","<<h.airborneFault<<","<<h.requiresDisarm<<","<<h.reason<<","<<h.faultReason<<","<<bool(o);
            if(o){std::cout<<","<<o->timestampUs<<","<<o->sampleUs<<","<<int(o->resetCounter);
                for(int j=0;j<3;++j)std::cout<<","<<o->position[j]<<","<<o->velocity[j];
                std::cout<<","<<o->attitude.w()<<","<<o->attitude.x()<<","<<o->attitude.y()<<","<<o->attitude.z();}
            std::cout<<"\n";
        }
    }
}
