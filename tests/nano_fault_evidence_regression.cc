#include "Px4Vio.h"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace ORB_SLAM3::px4;
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
struct Run {
    Config config;Gate gate;Sample s;Telemetry t;
    uint64_t wall=1700000000000000ULL,mono=1000000;
    explicit Run(Config c):config(c),gate(c){s.valid=s.imuInitialized=s.ba2=s.velocityValid=true;s.gravity={0,0,-9.81};t.haveStatus=t.haveSync=t.haveReader=true;}
    bool step(uint64_t dt=33333){wall+=dt;mono+=dt;s.sampleUs=wall-10000;++s.sequence;t.statusSteadyUs=t.syncSteadyUs=mono;return bool(gate.Evaluate(s,t,wall,mono));}
    void ready(){for(int i=0;i<60;++i)if(step())return;throw std::runtime_error("not ready");}
};
int main(){
    Config c;c.extrinsicsValid=true;
    Run start(c);start.s.valid=false;start.step();check(!start.gate.FirstFault(),"startup wait must not fill first-fault evidence");
    Run before(c);before.step();auto prev=before.s;before.s.position.x()=2.;before.step();
    check(before.gate.Reason()=="position_velocity_inconsistent","prepublication decision changed");
    check(before.gate.FirstFault().has_value(),"missing prepublication evidence");
    const auto& a=*before.gate.FirstFault();
    check(a.havePrevious && a.previous.sequence==prev.sequence && a.previous.sampleUs==prev.sampleUs,"comparison was captured after clearing");
    check(a.current.position.x()==2. && a.positionResidual>1.99 && !before.gate.FirstLatchedFault(),"incorrect prepublication snapshot");
    Run armed(c);armed.ready();armed.t.armed=true;armed.s.mapId=7;armed.s.frameId=30;armed.s.position.y()=3.;
    check(!armed.step(),"corrupt position published");
    check(armed.gate.FirstFault() && armed.gate.FirstLatchedFault(),"missing first/latch evidence");
    const auto original=armed.gate.FirstFault()->Json();
    check(armed.gate.FirstLatchedFault()->telemetry.armed && armed.gate.FirstLatchedFault()->current.mapId==7,"live context lost");
    armed.s.position.z()=20.;++armed.s.referenceEpoch;armed.step();
    check(armed.gate.FirstFault()->Json()==original,"later faults overwrote original cause");
    Run frame(c);frame.ready();frame.t.armed=true;const auto old=frame.s;++frame.s.referenceEpoch;frame.step();
    check(frame.gate.FirstLatchedFault()->havePrevious && frame.gate.FirstLatchedFault()->previous.referenceEpoch==old.referenceEpoch,"reanchor destroyed comparison");
    check(frame.gate.FirstLatchedFault()->current.referenceEpoch!=old.referenceEpoch,"new epoch missing");
    Run gap(c);gap.ready();gap.t.armed=true;gap.step(400000);
    check(gap.gate.FirstLatchedFault()->reason=="observation_gap","gap reason changed");
    FaultSnapshot nonfinite;nonfinite.current.position.x()=std::numeric_limits<double>::quiet_NaN();
    nonfinite.telemetry.localVelocity.y()=std::numeric_limits<double>::infinity();
    std::cout<<"JSON "<<nonfinite.Json()<<"\nJSON "<<original<<"\n";
    std::cout<<"PASS first cause, pre-clear sample pair, latch, epoch reset, outage, immutable evidence, JSON nonfinite values\n";
}
