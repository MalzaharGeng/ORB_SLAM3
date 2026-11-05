#include "Px4Vio.h"
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace ORB_SLAM3::px4;
static void check(bool v,const char* why) {if(!v)throw std::runtime_error(why);}
struct Run {
    uint64_t wall=1793900000000000ULL,mono=1000000;
    Sample s;Telemetry t;Gate gate;
    explicit Run(bool opt=true):gate(config(opt)) {
        s.valid=s.imuInitialized=s.ba2=s.velocityValid=true;s.gravity={0,0,9.81};
        t.haveStatus=t.haveSync=t.haveReader=true;t.haveLand=true;t.landed=true;
        t.haveNavigationStatus=t.navigationPositionValid=t.navigationVelocityValid=true;
        t.haveFusionStatus=t.evPosition=true;
    }
    static Config config(bool opt) {Config c;c.extrinsicsValid=true;c.allowArmedLandedRecovery=opt;return c;}
    std::optional<Observation> step(uint64_t dt=66667,bool updateLand=true) {
        wall+=dt;mono+=dt;++s.sequence;s.sampleUs=wall-20000;
        t.statusSteadyUs=t.syncSteadyUs=t.localSteadyUs=t.fusionSteadyUs=mono;
        if(updateLand) {t.landSteadyUs=mono;t.landSampleUs=wall;}
        return gate.Evaluate(s,t,wall,mono);
    }
    Observation ready() {for(int i=0;i<80;++i)if(auto o=step())return *o;throw std::runtime_error(gate.Reason());}
    void ground() {for(int i=0;i<60;++i)step();check(gate.Health(t,mono).groundReady,"ground never ready");}
    void airborneFault() {ready();t.armed=true;t.landed=false;s.position.x()=1.6;check(!step(),"bad pose published");s.position.setZero();}
};
int main() {
    int failures=0;auto test=[&](const char*n,const std::function<void()>&f){try{f();std::cout<<"PASS "<<n<<"\n";}catch(const std::exception&e){++failures;std::cerr<<"FAIL "<<n<<": "<<e.what()<<"\n";}};
    test("normal_two_armed_sorties_preserve_frame",[]{Run r;auto first=r.ready();r.ground();r.t.armed=true;r.t.landed=false;
        check(bool(r.step()),"first takeoff stopped");r.s.position.x()=.2;auto moved=r.step();check(bool(moved),"flight stopped");
        r.t.landed=true;r.ground();r.t.landed=false;auto second=r.step();check(bool(second),"second takeoff stopped");
        check(second->resetCounter==first.resetCounter,"landing reset frame");check((second->position-Eigen::Vector3d(.2,0,0)).norm()<1e-8,"landing changed origin");});
    test("motion_fault_landed_recovery_without_reanchor",[]{Run r;r.airborneFault();r.t.landed=true;
        for(int i=0;i<25;++i)check(!r.step(),"recovered before landed dwell");r.ground();check(r.gate.Health(r.t,r.mono).groundRecoveries==1,"recovery not counted");
        r.t.landed=false;auto next=r.step();check(bool(next)&&next->resetCounter==0,"recovery reset or blocked next takeoff");});
    test("airborne_fault_never_auto_recovers",[]{Run r;r.airborneFault();for(int i=0;i<90;++i)check(!r.step(),"in-flight recovery");});
    test("recovery_is_opt_in",[]{Run r(false);r.airborneFault();r.t.landed=true;for(int i=0;i<90;++i)check(!r.step(),"disabled recovery bypassed");});
    test("partial_or_missing_landing_does_not_recover",[]{Run r;r.airborneFault();r.t.haveLand=false;r.t.landed=true;for(int i=0;i<60;++i)check(!r.step(),"missing landing accepted");r.t.haveLand=true;r.t.landed=false;for(int i=0;i<60;++i)check(!r.step(),"contact without landed accepted");});
    test("stale_landed_does_not_recover",[]{Run r;r.airborneFault();r.t.landed=true;for(int i=0;i<80;++i)check(!r.step(66667,false),"stale landed accepted");});
    test("moving_or_invalid_samples_reset_evidence",[]{Run r;r.airborneFault();r.t.landed=true;r.s.velocity.x()=.3;
        for(int i=0;i<50;++i)check(!r.step(),"moving recovery");r.s.velocity.setZero();r.t.localVelocity.y()=.3;
        for(int i=0;i<50;++i)check(!r.step(),"PX4 moving recovery");r.t.localVelocity.setZero();r.s.valid=false;
        for(int i=0;i<50;++i)check(!r.step(),"invalid recovery");r.s.valid=true;for(int i=0;i<20;++i)check(!r.step(),"invalid evidence retained");r.ground();});
    test("landed_pulse_then_airborne_does_not_recover",[]{Run r;r.airborneFault();r.t.landed=true;for(int i=0;i<20;++i)check(!r.step(),"pulse recovered");r.t.landed=false;for(int i=0;i<50;++i)check(!r.step(),"bounce recovered");});
    test("frame_and_clock_faults_require_disarm",[]{for(bool clock:{false,true}){Run r;r.ready();r.t.armed=true;
        if(clock)++r.s.clockEpoch;else ++r.s.referenceEpoch;check(!r.step(),"frame reset accepted");r.t.landed=true;
        for(int i=0;i<90;++i)check(!r.step(),"hard reset recovered while armed");check(r.gate.Health(r.t,r.mono).requiresDisarm,"hard cause lost");
        r.t.armed=false;auto o=r.ready();check(o.resetCounter==1,"disarmed reanchor missing");}});
    test("duplicate_polling_cannot_confirm_landing",[]{Run r;r.airborneFault();r.t.landed=true;r.step();
        const auto wall=r.wall,mono=r.mono;for(int i=1;i<500;++i){r.t.landSteadyUs=r.t.statusSteadyUs=r.t.syncSteadyUs=r.t.localSteadyUs=mono+i*5000;
            check(!r.gate.Evaluate(r.s,r.t,wall+i*5000,mono+i*5000),"duplicate evidence recovered");}
        check(!r.gate.Health(r.t,mono+2500000).groundReady,"duplicate ground readiness");});
    test("navigation_readiness_requires_fusion_and_expires",[]{Run r;r.ground();check(r.gate.Health(r.t,r.mono).navigationReady,"healthy navigation not ready");
        check(!r.gate.Health(r.t,r.mono+300000).navigationReady,"stale output ready");r.t.evPosition=false;check(!r.gate.Health(r.t,r.mono).navigationReady,"missing fusion ready");
        r.t.evPosition=true;r.t.navigationVelocityValid=false;check(!r.gate.Health(r.t,r.mono).navigationReady,"invalid PX4 velocity ready");});
    test("stationary_pose_jump_restarts_ground_evidence",[]{Run r;r.airborneFault();r.t.landed=true;for(int i=0;i<20;++i)r.step();r.s.position.x()=.2;
        for(int i=0;i<20;++i)check(!r.step(),"jump retained ground timer");r.ground();});
    test("ground_recovery_requires_finite_px4_velocity",[]{Run r;r.airborneFault();r.t.landed=true;r.t.localVelocity.x()=std::numeric_limits<double>::quiet_NaN();for(int i=0;i<70;++i)check(!r.step(),"NaN ground velocity accepted");});
    test("permanently_displaced_pose_does_not_recover",[]{Run r;r.airborneFault();r.t.landed=true;r.s.position.x()=1.6;for(int i=0;i<90;++i)check(!r.step(),"stable but displaced frame accepted");});
    test("replayed_landed_source_timestamp_does_not_recover",[]{Run r;r.airborneFault();r.t.landed=true;r.t.landSampleUs=r.wall;
        for(int i=0;i<90;++i){r.t.landSteadyUs=r.mono+66667;check(!r.step(66667,false),"replayed land source accepted");}});
    test("landed_motion_flag_prevents_recovery",[]{Run r;r.airborneFault();r.t.landed=true;r.t.landMoving=true;for(int i=0;i<90;++i)check(!r.step(),"land movement ignored");});
    test("configuration_rejects_short_dwell",[]{Config c=Run::config(true);c.landedStableSeconds=.1;bool threw=false;try{c.Validate();}catch(...){threw=true;}check(threw,"unsafe dwell accepted");});
    std::cout<<"SORTIE_REGRESSION_FAILURES="<<failures<<"\n";return failures?1:0;
}
