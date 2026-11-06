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
        if(t.haveLocalFrame) {t.localSteadyUs=mono;t.localSampleUs=s.sampleUs;}
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
    Config c;c.extrinsicsValid=true;c.maxSpeed=20.;c.maxPositionResidual=5.;c.consistencyWindowSeconds=10.;
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
    // PX4-local NED: the same fixed transform applies to position, velocity and attitude.
    for(double heading : {0., 1.5707963267948966, -1.5707963267948966, 2.8}) {
        Config n;n.extrinsicsValid=true;n.alignToPx4Ned=true;n.maxSampleGap=.6;
        Run r(n);r.s.gravity={0,0,9.81};
        const Eigen::Matrix3d initial=Eigen::AngleAxisd(.73,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d ned=Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        r.s.imuToWorld=initial;r.s.position={1,2,3};
        r.t.haveLocalFrame=r.t.localXYValid=r.t.localZValid=true;
        r.t.localHeading=heading;r.t.localPosition={4,-2,-.8};
        auto b=r.ready();require(b.nedFrame,"NED flag missing");
        near(b.position,r.t.localPosition,"NED local origin");
        require(b.attitude.angularDistance(Eigen::Quaterniond(ned))<1e-8,"NED initial yaw");
        ++r.t.localHeadingReset;
        auto adopted=r.step();
        require(bool(adopted) && adopted->resetCounter==0,"EKF adoption caused an EV reset feedback loop");
        r.t.armed=true;
        const Eigen::Matrix3d turn=Eigen::AngleAxisd(1.5707963267948966,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        r.s.imuToWorld=initial*turn;
        r.s.position+=initial*turn*Eigen::Vector3d(.02,.01,0);
        r.s.velocity=initial*turn*Eigen::Vector3d(.6,.3,0);
        // Future PX4 heading changes must not rotate the established observation frame.
        r.t.localHeading+=.4;
        // A physical 90-degree turn takes time; do not model it as a one-frame reset.
        auto after=r.step(500000);require(bool(after),"yawed observation absent");
        near(after->position,b.position+ned*turn*Eigen::Vector3d(.02,.01,0),"NED yawed forward/right axes");
        near(after->velocity,ned*turn*Eigen::Vector3d(.6,.3,0),"NED yawed velocity axes");
        require(after->attitude.angularDistance(Eigen::Quaterniond(ned*turn))<1e-8,"NED yawed quaternion");
        const Eigen::Matrix3d tilt=(Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitY())*
                                   Eigen::AngleAxisd(-.1,Eigen::Vector3d::UnitX())).toRotationMatrix();
        r.s.imuToWorld=initial*turn*tilt;
        auto tilted=r.step();require(bool(tilted),"tilted observation absent");
        require(tilted->attitude.angularDistance(Eigen::Quaterniond(ned*turn*tilt))<1e-8,"NED pitch/roll signs");
        ++r.s.referenceEpoch;require(!r.step(),"airborne map reset accepted");
        for(int i=0;i<80;++i)require(!r.step(),"airborne latch auto-recovered");
        require(r.gate.Reason()=="airborne_fault_latched","missing airborne fault reason");
        r.t.armed=false;r.s.velocity.setZero();
        require(r.ready().resetCounter==1,"disarmed recovery reset count");
    }
    Config strict;strict.extrinsicsValid=true;
    Config localConfig=strict;localConfig.alignToPx4Ned=true;
    Run localReset(localConfig);localReset.s.gravity={0,0,9.81};
    localReset.t.haveLocalFrame=true;localReset.ready();localReset.t.armed=true;
    ++localReset.t.localHeadingReset;
    require(!localReset.step(),"airborne PX4 heading reset accepted");
    require(localReset.gate.Reason()=="airborne_fault_latched","PX4 reset did not latch");
    // Pure attitude jumps at an unchanged position used to escape the motion gate.
    for(const auto& axis : {Eigen::Vector3d::UnitX().eval(),Eigen::Vector3d::UnitZ().eval()}) {
        Config angular=strict;angular.allowArmedLandedRecovery=true;
        Run r(angular);r.ready();r.t.armed=true;
        r.s.imuToWorld=Eigen::AngleAxisd(1.5707963267948966,axis).toRotationMatrix();
        require(!r.step(),"attitude-only jump was published");
        require(r.gate.Reason()=="attitude_discontinuity","angular jump diagnostic absent");
        require(r.gate.Health(r.t,r.mono).requiresDisarm,"angular frame fault allowed landed recovery");
        r.t.haveLand=r.t.landed=r.t.haveNavigationStatus=true;
        for(int i=0;i<120;++i) {
            r.t.landSteadyUs=r.mono+33333;r.t.landSampleUs=r.wall+33333;
            require(!r.step(),"angular frame fault recovered while armed");
        }
        r.t.armed=false;r.ready();
    }
    Run normalTurn(strict);normalTurn.ready();normalTurn.t.armed=true;
    for(int i=1;i<=60;++i) {
        normalTurn.s.imuToWorld=Eigen::AngleAxisd(i*.03,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        require(bool(normalTurn.step()),"ordinary continuous turn was rejected");
    }
    for(double invalid : {0.,-1.,std::numeric_limits<double>::quiet_NaN(),721.}) {
        auto bad=strict;bad.maxAttitudeRateDeg=invalid;bool rejectedAngular=false;
        try {bad.Validate();}catch(...) {rejectedAngular=true;}
        require(rejectedAngular,"invalid angular rate accepted");
    }
    Run jump(strict);jump.ready();jump.t.armed=true;jump.s.position.x()+=1.635;
    require(!jump.step(),"recorded 1.635m inconsistency was published");
    require(jump.gate.Reason()=="position_velocity_inconsistent","jump diagnostic absent");
    jump.s.position.setZero();for(int i=0;i<80;++i)require(!jump.step(),"motion fault latch bypassed");
    jump.t.armed=false;jump.ready();
    Run gap(strict);gap.ready();gap.t.armed=true;
    require(!gap.step(400000),"airborne sample gap was published");
    require(gap.gate.Reason()=="observation_gap","sample gap diagnostic absent");
    Run fast(strict);fast.ready();fast.s.velocity.x()=6.;
    require(!fast.step(),"unbounded velocity was published");
    Run noGround(strict);noGround.t.armed=true;
    for(int i=0;i<60;++i)require(!noGround.step(),"airborne first reference accepted");
    Config nd=strict;nd.alignToPx4Ned=true;Run noLocal(nd);
    for(int i=0;i<60;++i)require(!noLocal.step(),"NED without PX4 local frame accepted");
    require(noLocal.gate.Reason()=="px4_local_frame_unready","local frame diagnostic absent");
    Config air;air.extrinsicsValid=true;air.alignToPx4Ned=true;air.allowAirborneInitialReference=true;
    Run airInit(air);airInit.s.gravity={0,0,9.81};airInit.t.armed=true;airInit.t.navState=1;
    airInit.t.haveLocalFrame=airInit.t.localZValid=true;airInit.t.localPosition.z()=-.7;
    auto airFirst=airInit.ready();require(std::abs(airFirst.position.z()+.7)<1e-8,"airborne altitude origin lost");
    ++airInit.t.localXYReset;++airInit.t.localHeadingReset;
    require(bool(airInit.step()),"initial ALTCTL EV adoption reset prevented airborne initialization");
    ++airInit.s.referenceEpoch;for(int i=0;i<80;++i)require(!airInit.step(),"airborne initial option permitted reanchor");
    Run airControl(air);airControl.s.gravity={0,0,9.81};airControl.t.armed=true;airControl.t.navState=1;airControl.t.haveLocalFrame=true;
    airControl.ready();airControl.t.navState=2;++airControl.t.localXYReset;
    require(!airControl.step(),"handover exemption continued after POSCTL took control");
    Run airHealth(air);airHealth.s.gravity={0,0,9.81};airHealth.t.armed=true;
    airHealth.t.navState=1;airHealth.t.haveLocalFrame=true;airHealth.ready();
    airHealth.t.haveNavigationStatus=airHealth.t.navigationPositionValid=airHealth.t.navigationVelocityValid=true;
    airHealth.t.haveFusionStatus=airHealth.t.evPosition=true;airHealth.t.localVelocity.setZero();
    for(int i=0;i<85;++i) {
        require(bool(airHealth.step()),"initial handover stopped valid observations");
        airHealth.t.fusionSteadyUs=airHealth.mono;
        auto h=airHealth.gate.Health(airHealth.t,airHealth.mono);
        require(!h.navigationReady && h.reason=="initial_ev_handover","airborne handover prematurely authorized navigation");
    }
    for(int i=0;i<10;++i){airHealth.step();airHealth.t.fusionSteadyUs=airHealth.mono;}
    require(airHealth.gate.Health(airHealth.t,airHealth.mono).navigationReady,"stable airborne handover never became ready");
    Run unsafeAir(air);unsafeAir.t.armed=true;unsafeAir.t.navState=2;unsafeAir.t.haveLocalFrame=true;
    for(int i=0;i<60;++i)require(!unsafeAir.step(),"airborne POSCTL reference accepted");
    require(unsafeAir.gate.Reason()=="airborne_initial_mode_unsafe","unsafe airborne mode diagnostic absent");
    Config drift;drift.extrinsicsValid=true;Run inconsistent(drift);inconsistent.ready();inconsistent.t.armed=true;
    inconsistent.s.velocity={.2,-.17,0};bool stopped=false;
    for(int i=0;i<90;++i)if(!inconsistent.step() && inconsistent.gate.Reason()=="cumulative_position_velocity_inconsistent") {stopped=true;break;}
    require(stopped,"recorded slow position/velocity disagreement escaped cumulative gate");
    for(int i=0;i<20;++i)require(!inconsistent.step(),"cumulative airborne fault auto-recovered");
    // Realistic low-rate delivery: 180ms computation age plus 182/250ms
    // arrival period. Poll every 5ms exactly as the bridge does; no duplicates.
    for(uint64_t period : {181818ULL,250000ULL}) {
        Config low;low.extrinsicsValid=true;low.maxAgeSeconds=.25;
        Run r(low);size_t delivered=0;
        for(int frame=0;frame<25;++frame) {
            r.wall+=period;r.mono+=period;++r.s.sequence;r.s.sampleUs=r.wall-180000;
            r.t.statusSteadyUs=r.t.syncSteadyUs=r.mono;
            if(r.gate.Evaluate(r.s,r.t,r.wall,r.mono))++delivered;
            const auto count=r.gate.Sent();
            for(uint64_t poll=5000;poll<period;poll+=5000)
                require(!r.gate.Evaluate(r.s,r.t,r.wall+poll,r.mono+poll),"low-rate duplicate republished");
            require(r.gate.Sent()==count,"duplicate advanced publication count");
        }
        require(delivered>=18,"low-rate polling repeatedly cleared startup stability");
        std::cout<<"LOW_RATE_GATE_PASS period_us="<<period<<" age_us=180000 delivered="<<delivered<<"/25\n";
        // An actual receipt stall still stops output and requires stability.
        require(!r.gate.Evaluate(r.s,r.t,r.wall+310000,r.mono+310000),"real stall admitted");
        require(r.gate.Reason()=="sample_stale","real stall diagnostic absent");
        r.wall+=350000;r.mono+=350000;r.t.statusSteadyUs=r.t.syncSteadyUs=r.mono;
        ++r.s.sequence;r.s.sampleUs=r.wall-180000;
        require(!r.gate.Evaluate(r.s,r.t,r.wall,r.mono),"stall bypassed recovery stability");
        r.wall+=100000;r.mono+=100000;r.t.statusSteadyUs=r.t.syncSteadyUs=r.mono;
        ++r.s.sequence;r.s.sampleUs=r.wall-260000;
        require(!r.gate.Evaluate(r.s,r.t,r.wall,r.mono),"new late observation was accepted");
        require(r.gate.Reason()=="sample_stale","new late observation diagnostic absent");
    }
    Run residualXY(air);residualXY.s.gravity={0,0,9.81};
    residualXY.t.armed=true;residualXY.t.navState=1;residualXY.t.haveLocalFrame=true;
    residualXY.t.localXYValid=false;residualXY.t.localZValid=true;
    residualXY.t.localPosition={12.5,-7.25,-1.1};
    auto anchored=residualXY.ready();
    near(anchored.position,residualXY.t.localPosition,"invalid nonzero EKF XY was silently zeroed");
    ++residualXY.t.localXYReset;residualXY.t.localPosition.x()+=.1;
    auto adoption=residualXY.step();require(bool(adoption),"initial XY adoption rejected");
    near(adoption->position,anchored.position,"EKF adoption fed back into fixed EV origin");
    std::cout<<"INVALID_NONZERO_XY_PASS origin="<<anchored.position.transpose()<<"\n";
    // Invalid calibration is rejected before any transport is started.
    c.bodyFromImu(0,0)=7;bool rejected=false;try{c.Validate();}catch(...){rejected=true;}
    require(rejected,"non-rigid calibration accepted");
    std::cout<<"PX4_VIO_REGRESSION_PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
