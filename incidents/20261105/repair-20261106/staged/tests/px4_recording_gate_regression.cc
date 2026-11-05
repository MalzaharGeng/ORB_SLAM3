#include "Px4Vio.h"
#include <iostream>
#include <stdexcept>
#include <functional>
using namespace ORB_SLAM3::px4;
static void check(bool ok,const char* why) { if(!ok)throw std::runtime_error(why); }
struct Run {
    uint64_t wall=1793894400000000ULL,mono=1000000;
    Config config; Sample s; Telemetry t; Gate gate;
    explicit Run(Config c):config(c),gate(c) {
        s.valid=s.imuInitialized=s.ba2=s.velocityValid=true;s.gravity={0,0,9.81};
        t.haveReader=t.haveStatus=t.haveSync=true;
    }
    std::optional<Observation> step(uint64_t dt=66667) {
        wall+=dt;mono+=dt;++s.sequence;s.sampleUs=wall-60000;
        t.statusSteadyUs=t.syncSteadyUs=mono;
        return gate.Evaluate(s,t,wall,mono);
    }
    Observation ready() { for(int i=0;i<40;++i)if(auto o=step())return *o;throw std::runtime_error(gate.Reason()); }
};
int main(int argc,char**argv) {
    int failures=0;
    auto test=[&](const char*name,const std::function<void()>&f){try{f();std::cout<<"PASS "<<name<<"\n";}catch(const std::exception&e){++failures;std::cout<<"FAIL "<<name<<": "<<e.what()<<"\n";}};
    Config c;c.extrinsicsValid=true;
    test("recorded_RTT_20387us",[&]{Run r(c);r.ready();r.t.armed=true;r.t.roundTripUs=20387;check(bool(r.step()),"recorded RTT stopped output");});
    test("recorded_residual_6400us",[&]{Run r(c);r.ready();r.t.armed=true;r.t.observedOffsetUs=-6400;check(bool(r.step()),"recorded residual stopped output");});
    test("deployed_gap_1_07s_keeps_frame",[&]{
        check(argc==2,"config argument required");auto deployed=Config::Load(argv[1]);deployed.alignToPx4Ned=false;
        deployed.bodyFromImu.setIdentity();Run r(deployed);auto before=r.ready();r.t.armed=true;r.s.position.x()=.05;
        auto after=r.step(1067747);check(bool(after),r.gate.Reason().c_str());
        check(after->resetCounter==before.resetCounter,"gap changed reset counter");
        check((after->position-before.position-Eigen::Vector3d(.05,0,0)).norm()<1e-8,"gap changed origin");
    });
    test("bad_pose_across_gap_stays_blocked",[&]{
        auto d=Config::Load(argv[1]);d.alignToPx4Ned=false;d.bodyFromImu.setIdentity();Run r(d);r.ready();r.t.armed=true;r.s.position.x()=2.;
        check(!r.step(1067747),"inconsistent gap pose published");check(r.gate.Reason()=="position_velocity_inconsistent","wrong pose diagnostic");
        r.s.position.setZero();check(!r.step(),"hard airborne latch cleared");
    });
    test("sync_faults_remain_blocked",[&]{Run r(c);r.ready();r.t.armed=true;r.t.roundTripUs=100001;check(!r.step(),"bad RTT accepted");r.t.roundTripUs=0;r.t.observedOffsetUs=50001;check(!r.step(),"bad residual accepted");r.t.observedOffsetUs=0;r.t.haveSync=false;check(!r.step(),"missing sync accepted");});
    test("sync_age_over_2s_blocked",[&]{Run r(c);r.ready();r.wall+=2100000;r.mono+=2100000;++r.s.sequence;r.s.sampleUs=r.wall-10000;r.t.statusSteadyUs=r.mono;check(!r.gate.Evaluate(r.s,r.t,r.wall,r.mono),"stale sync accepted");check(r.gate.Reason()=="time_sync_unready","wrong sync age reason");});
    test("short_sync_outage_recovers_same_reference",[&]{auto d=Config::Load(argv[1]);d.alignToPx4Ned=false;d.bodyFromImu.setIdentity();Run r(d);auto before=r.ready();r.t.armed=true;r.t.haveSync=false;for(int i=0;i<15;++i)check(!r.step(),"outage published");r.t.haveSync=true;auto after=r.ready();check(after.resetCounter==before.resetCounter,"recovery reanchored");});
    test("clock_jump_remains_latched",[&]{Run r(c);r.ready();r.t.armed=true;r.wall+=500000;check(!r.step(),"clock jump published");check(!r.step(),"clock latch cleared in flight");check(r.gate.Reason()=="airborne_fault_latched","clock latch missing");});
    test("stale_sample_remains_blocked",[&]{Run r(c);r.ready();r.wall+=400000;r.mono+=400000;++r.s.sequence;r.s.sampleUs=r.wall-300000;r.t.statusSteadyUs=r.t.syncSteadyUs=r.mono;check(!r.gate.Evaluate(r.s,r.t,r.wall,r.mono),"stale input accepted");check(r.gate.Reason()=="sample_stale","late new sample did not reach age gate");});
    test("future_sample_remains_blocked",[&]{Run r(c);r.ready();++r.s.sequence;r.s.sampleUs=r.wall+1000;check(!r.gate.Evaluate(r.s,r.t,r.wall,r.mono),"future input accepted");check(r.gate.Reason()=="sample_in_future","future diagnostic missing");});
    std::cout<<"RECORDING_GATE_FAILURES="<<failures<<"\n";return failures?1:0;
}
