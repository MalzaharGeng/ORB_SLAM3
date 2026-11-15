#include "Px4Bridge.h"
#include "Px4GateSnapshot.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>
#include <deque>
#include <cmath>
#include <sstream>
#include <cstdlib>

namespace ORB_SLAM3 { namespace px4 {
namespace {
template<class T,class=void> struct Version {static constexpr uint32_t value=0;};
template<class T> struct Version<T,std::void_t<decltype(T::MESSAGE_VERSION)>> {
    static constexpr uint32_t value=T::MESSAGE_VERSION;
};
using Odom=px4_msgs::msg::VehicleOdometry;
using Status=px4_msgs::msg::VehicleStatus;
using Position=px4_msgs::msg::VehicleLocalPosition;
using Flags=px4_msgs::msg::EstimatorStatusFlags;
using Sync=px4_msgs::msg::TimesyncStatus;
using Land=px4_msgs::msg::VehicleLandDetected;
using AttSetpoint=px4_msgs::msg::VehicleAttitudeSetpoint;
using RateSetpoint=px4_msgs::msg::VehicleRatesSetpoint;
static_assert(Odom::MESSAGE_VERSION==0 && Status::MESSAGE_VERSION==1 && Position::MESSAGE_VERSION==1,
              "Build with PX4 1.17 px4_msgs definitions");
}
namespace {
// All submit-side values are copied here, at the Bridge's data owner, before
// the Sample is moved into the queue. Nothing here feeds a Gate decision.
GateSnapshotRecord MakeSubmitRecord(const Sample& s) {
    GateSnapshotRecord r;
    r.seq=s.sequence;r.pair=s.pairId;
    r.submitWallUs=SystemMicros();r.submitMonoUs=SteadyMicros();
    r.srcSimUs=s.srcSimUs;r.mapOffsetUs=s.mapOffsetUs;r.sampleUs=s.sampleUs;
    r.clockEpoch=s.clockEpoch;r.refEpoch=s.referenceEpoch;r.continuity=s.continuity;
    r.mapId=s.mapId;r.initialKfId=s.initialKeyFrameId;r.frameId=s.frameId;
    const Eigen::Quaterniond sq(s.imuToWorld);
    r.srcQ=SnapQuat{sq.w(),sq.x(),sq.y(),sq.z()};
    r.srcP=SnapVec3{s.position.x(),s.position.y(),s.position.z()};
    r.srcV=SnapVec3{s.velocity.x(),s.velocity.y(),s.velocity.z()};
    const Eigen::Quaterniond rq(s.rawCameraToWorld);
    r.rawQ=SnapQuat{rq.w(),rq.x(),rq.y(),rq.z()};
    r.rawP=SnapVec3{s.rawCameraPosition.x(),s.rawCameraPosition.y(),s.rawCameraPosition.z()};
    const Eigen::Quaterniond mq(s.mapToWorld);
    r.mapQ=SnapQuat{mq.w(),mq.x(),mq.y(),mq.z()};
    r.mapT=SnapVec3{s.mapTranslation.x(),s.mapTranslation.y(),s.mapTranslation.z()};
    const Eigen::Quaterniond iq(s.initialImuToWorld);
    r.initQ=SnapQuat{iq.w(),iq.x(),iq.y(),iq.z()};
    r.initT=SnapVec3{s.initialImuTranslation.x(),s.initialImuTranslation.y(),s.initialImuTranslation.z()};
    r.valid=s.valid;r.imuInit=s.imuInitialized;r.ba2=s.ba2;r.velValid=s.velocityValid;
    r.gyroIntegral=SnapVec3{s.gyroIntegral.x(),s.gyroIntegral.y(),s.gyroIntegral.z()};
    r.gyroSpanS=s.gyroSpanS;r.imuCount=s.imuCount;
    return r;
}
}
struct Bridge::Impl {
    Config config;
    Gate gate;
    std::shared_ptr<rclcpp::Context> context;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Publisher<Odom>::SharedPtr publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr healthPublisher;
    rclcpp::Subscription<Land>::SharedPtr landSub;
    rclcpp::Subscription<Status>::SharedPtr statusSub;
    rclcpp::Subscription<Position>::SharedPtr positionSub;
    rclcpp::Subscription<Flags>::SharedPtr flagsSub;
    rclcpp::Subscription<Sync>::SharedPtr syncSub;
    // [NP-01] Bounded observation of the ACTUAL controller targets. Only the
    // latest sample is kept (no history, no unbounded buffer); subscription
    // callbacks run on the same single-threaded executor as the other subs.
    rclcpp::Subscription<AttSetpoint>::SharedPtr attSpSub;
    rclcpp::Subscription<RateSetpoint>::SharedPtr rateSpSub;
    struct SetpointSnapshot {
        bool valid=false;
        double qw=0,qx=0,qy=0,qz=0,t0=0,t1=0,t2=0,extra=0;
        double roll=0,pitch=0,yaw=0;
    };
    SetpointSnapshot attSp{},rateSp{};
    Telemetry telemetry;
    Position position{};
    std::deque<Position> localHistory;
    Flags flags{};
    uint8_t navMode=0;
    uint64_t positionReceived=0,flagsReceived=0;
    std::mutex mutex;
    std::mutex faultMutex;
    std::string faultReason;
    bool publishedOnce=false;
    std::condition_variable wake;
    // 待评估样本队列（旧→新）。单槽 mailbox 会在 20 ms 内被下一帧覆盖，
    // 零延迟源（真值回灌）永远等不到 local 历史越过样本时间戳。
    std::deque<Sample> queue;
    bool pending=false;
    uint64_t sequence=0,continuity=0,overwritten=0;
    std::atomic<bool> stop{false};
    std::thread worker;
    std::ofstream log;
    std::unique_ptr<GateSnapshotTrace> snapshot;
    const std::string instanceId=std::to_string(SystemMicros())+"-"+std::to_string(SteadyMicros());

    explicit Impl(const Config& c):config(c),gate(c) {
        if(!config.logPath.empty()) {
            log.open(config.logPath);
            if(!log)throw std::runtime_error("Cannot open PX4 telemetry log: "+config.logPath);
            log<<"system_us,event,reason,sample_us,age_ms,sent,rejected,overwritten,armed,nav_mode,ev_pos,ev_hgt,ev_vel,reject_pos,reject_vel,local_xy_valid,local_z_valid,px,py,pz,vx,vy,vz,reset_counter,ekf_x,ekf_y,ekf_z,ekf_vx,ekf_vy,ekf_vz,ekf_vxy_valid,ekf_vz_valid,status_age_ms,sync_age_ms,flags_age_ms,position_age_ms,sync_rtt_us,sync_residual_us"
              // [NP-01] bounded diagnostic columns: what was ACTUALLY serialized
              // into VehicleOdometry, the ORB sample it came from, EKF reset /
              // rejection flags, and the latest PX4 control targets.
              ",qw,qx,qy,qz,pose_frame,velocity_frame,ang_vel_nan,pos_var,vel_var,ori_var,quality"
              ",src_px,src_py,src_pz,src_vx,src_vy,src_vz,src_qw,src_qx,src_qy,src_qz"
              ",xy_reset,z_reset,heading_reset,rej_hpos,rej_vpos,rej_hvel,rej_vvel,ev_yaw"
              ",att_sp_qw,att_sp_qx,att_sp_qy,att_sp_qz,att_sp_t0,att_sp_t1,att_sp_t2,att_sp_yawrate"
              ",rate_sp_roll,rate_sp_pitch,rate_sp_yaw,rate_sp_t0,rate_sp_t1,rate_sp_t2"
              ",att_sp_valid,rate_sp_valid\n";
        }
        {
            // Opt-in: with NP01_GATE_SNAPSHOT unset the production path is
            // byte-identical apart from one getenv in this constructor.
            const char* p=std::getenv("NP01_GATE_SNAPSHOT");
            if(p && *p) {
                snapshot=std::make_unique<GateSnapshotTrace>(p);
                std::cerr<<"[NP01_FIX11] gate snapshot: "<<p<<std::endl;
            }
        }
        context=std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions init;init.set_domain_id(config.domainId);init.shutdown_on_signal=false;
        context->init(0,nullptr,init);
        rclcpp::NodeOptions options;options.context(context).use_global_arguments(false);
        node=std::make_shared<rclcpp::Node>("orb_slam3_px4_bridge",options);
        auto topic=[&](const char* name,uint32_t version){return TopicName(config.topicPrefix,name,version);};
        publisher=node->create_publisher<Odom>(topic("/fmu/in/vehicle_visual_odometry",Version<Odom>::value),
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
        auto qos=rclcpp::SensorDataQoS().keep_last(5);
        healthPublisher=node->create_publisher<std_msgs::msg::String>(
            TopicName(config.topicPrefix,"/vio/navigation_health",0),rclcpp::QoS(1).reliable().durability_volatile());
        landSub=node->create_subscription<Land>(topic("/fmu/out/vehicle_land_detected",Version<Land>::value),qos,
            [this](Land::ConstSharedPtr m){telemetry.haveLand=true;telemetry.landSteadyUs=SteadyMicros();telemetry.landSampleUs=m->timestamp;
                telemetry.landed=m->landed;telemetry.freefall=m->freefall;
                telemetry.landMoving=m->vertical_movement || m->horizontal_movement || m->rotational_movement;});
        statusSub=node->create_subscription<Status>(topic("/fmu/out/vehicle_status",Version<Status>::value),qos,
            [this](Status::ConstSharedPtr m){telemetry.haveStatus=true;telemetry.statusSteadyUs=SteadyMicros();
                telemetry.armed=m->arming_state==Status::ARMING_STATE_ARMED;navMode=m->nav_state;telemetry.navState=m->nav_state;});
        syncSub=node->create_subscription<Sync>(topic("/fmu/out/timesync_status",Version<Sync>::value),qos,
            [this](Sync::ConstSharedPtr m){if(m->source_protocol!=Sync::SOURCE_PROTOCOL_DDS)return;
                telemetry.haveSync=true;telemetry.syncSteadyUs=SteadyMicros();telemetry.roundTripUs=m->round_trip_time;
                telemetry.observedOffsetUs=m->observed_offset;telemetry.estimatedOffsetUs=m->estimated_offset;});
        positionSub=node->create_subscription<Position>(topic("/fmu/out/vehicle_local_position",Version<Position>::value),qos,
            [this](Position::ConstSharedPtr m){position=*m;positionReceived=SteadyMicros();
                if(!localHistory.empty() && m->timestamp<=localHistory.back().timestamp)localHistory.clear();
                localHistory.push_back(*m);while(localHistory.size()>100)localHistory.pop_front();
                telemetry.localSteadyUs=positionReceived;
                telemetry.haveNavigationStatus=true;
                telemetry.navigationPositionValid=m->xy_valid && m->z_valid &&
                    std::isfinite(m->x) && std::isfinite(m->y) && std::isfinite(m->z);
                telemetry.navigationVelocityValid=m->v_xy_valid && m->v_z_valid;
                telemetry.localVelocity={m->vx,m->vy,m->vz};
                telemetry.localXYReset=m->xy_reset_counter;telemetry.localZReset=m->z_reset_counter;
                telemetry.localHeadingReset=m->heading_reset_counter;});
        flagsSub=node->create_subscription<Flags>(topic("/fmu/out/estimator_status_flags",Version<Flags>::value),qos,
            [this](Flags::ConstSharedPtr m){flags=*m;flagsReceived=SteadyMicros();
                telemetry.haveFusionStatus=true;telemetry.fusionSteadyUs=flagsReceived;
                telemetry.evPosition=m->cs_ev_pos;telemetry.evVelocity=m->cs_ev_vel;});
        attSpSub=node->create_subscription<AttSetpoint>(
            topic("/fmu/out/vehicle_attitude_setpoint",Version<AttSetpoint>::value),qos,
            [this](AttSetpoint::ConstSharedPtr m){
                attSp.valid=true;attSp.qw=m->q_d[0];attSp.qx=m->q_d[1];
                attSp.qy=m->q_d[2];attSp.qz=m->q_d[3];
                attSp.t0=m->thrust_body[0];attSp.t1=m->thrust_body[1];attSp.t2=m->thrust_body[2];
                attSp.extra=m->yaw_sp_move_rate;});
        rateSpSub=node->create_subscription<RateSetpoint>(
            topic("/fmu/out/vehicle_rates_setpoint",Version<RateSetpoint>::value),qos,
            [this](RateSetpoint::ConstSharedPtr m){
                rateSp.valid=true;rateSp.roll=m->roll;rateSp.pitch=m->pitch;rateSp.yaw=m->yaw;
                rateSp.t0=m->thrust_body[0];rateSp.t1=m->thrust_body[1];rateSp.t2=m->thrust_body[2];});
        rclcpp::ExecutorOptions eo;eo.context=context;
        executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);executor->add_node(node);
        std::cout<<"[PX4] domain="<<config.domainId<<" topic="<<publisher->get_topic_name()
            <<" extrinsics="<<(config.extrinsicsValid?"configured":"MISSING (receive only)")<<std::endl;
        if(config.extrinsicsValid)std::cout<<"[PX4] R_BI configured; IMU lever arm is configured only in PX4 EKF2_EV_POS_X/Y/Z"<<std::endl;
        worker=std::thread([this]{Run();});
    }
    ~Impl() {
        stop=true;wake.notify_all();if(worker.joinable())worker.join();
        if(snapshot) {
            snapshot->Flush(true);
            std::string c;snapshot->Counters(&c);
            std::cerr<<"[NP01_FIX11] "<<c<<std::endl;
        }
        executor.reset();node.reset();context->shutdown("VIO bridge stopped");
    }
    void Submit(Sample s) {
        std::lock_guard<std::mutex> lock(mutex);
        if(!s.valid)++continuity;
        s.continuity=continuity;s.sequence=++sequence;
        if(snapshot)snapshot->Add(MakeSubmitRecord(s));
        queue.push_back(std::move(s));
        // 有界：溢出时丢最旧的并如实计数，避免延迟无限增长。
        while(queue.size()>32) {queue.pop_front();++overwritten;}
        pending=true;wake.notify_one();
    }
    void Record(const Sample& sample,const Observation* o,uint64_t dropped) {
        if(!log)return;
        const double nan=std::numeric_limits<double>::quiet_NaN();
        log<<std::setprecision(12)<<SystemMicros()<<","<<(o?"publish":"status")<<","<<gate.Reason()<<","<<sample.sampleUs
           <<","<<gate.AgeSeconds()*1000<<","<<gate.Sent()<<","<<gate.Rejected()<<","<<dropped<<","<<telemetry.armed
           <<","<<int(navMode)<<","<<flags.cs_ev_pos<<","<<flags.cs_ev_hgt<<","<<flags.cs_ev_vel
           <<","<<(flags.reject_hor_pos||flags.reject_ver_pos)<<","<<(flags.reject_hor_vel||flags.reject_ver_vel)
           <<","<<position.xy_valid<<","<<position.z_valid;
        for(int i=0;i<3;++i)log<<","<<(o?o->position(i):nan);
        for(int i=0;i<3;++i)log<<","<<(o?o->velocity(i):nan);
        const auto steady=SteadyMicros();
        const auto age=[steady](uint64_t stamp){return stamp?double(steady-stamp)*.001:-1.;};
        log<<","<<(o?int(o->resetCounter):-1)
           <<","<<position.x<<","<<position.y<<","<<position.z<<","<<position.vx<<","<<position.vy<<","<<position.vz
           <<","<<position.v_xy_valid<<","<<position.v_z_valid
           <<","<<age(telemetry.statusSteadyUs)<<","<<age(telemetry.syncSteadyUs)
           <<","<<age(flagsReceived)<<","<<age(positionReceived)<<","<<telemetry.roundTripUs
           <<","<<(telemetry.observedOffsetUs-telemetry.estimatedOffsetUs);
        // [NP-01] Bounded diagnostic: publish rows record the EXACT serialized
        // attitude/frame/variance, the ORB sample it was derived from, the EKF
        // reset/rejection flags and the latest PX4 attitude/rate targets.
        // Status rows keep the column count with empty fields (no growth).
        if(o) {
            const Eigen::Quaterniond srcQ(sample.imuToWorld);
            log<<","<<o->attitude.w()<<","<<o->attitude.x()<<","<<o->attitude.y()<<","<<o->attitude.z()
               <<","<<(o->nedFrame?int(Odom::POSE_FRAME_NED):int(Odom::POSE_FRAME_FRD))
               <<","<<(o->nedFrame?int(Odom::VELOCITY_FRAME_NED):int(Odom::VELOCITY_FRAME_FRD))
               <<",1"  // angular_velocity is intentionally NaN (not faked)
               <<","<<o->positionVariance<<","<<o->velocityVariance<<","<<o->orientationVariance
               <<",0"  // msg.quality is always 0 in this bridge
               <<","<<sample.position.x()<<","<<sample.position.y()<<","<<sample.position.z()
               <<","<<sample.velocity.x()<<","<<sample.velocity.y()<<","<<sample.velocity.z()
               <<","<<srcQ.w()<<","<<srcQ.x()<<","<<srcQ.y()<<","<<srcQ.z()
               <<","<<int(position.xy_reset_counter)<<","<<int(position.z_reset_counter)
               <<","<<int(position.heading_reset_counter)
               <<","<<int(flags.reject_hor_pos)<<","<<int(flags.reject_ver_pos)
               <<","<<int(flags.reject_hor_vel)<<","<<int(flags.reject_ver_vel)
               <<","<<int(flags.cs_ev_yaw)
               <<","<<attSp.qw<<","<<attSp.qx<<","<<attSp.qy<<","<<attSp.qz
               <<","<<attSp.t0<<","<<attSp.t1<<","<<attSp.t2<<","<<attSp.extra
               <<","<<rateSp.roll<<","<<rateSp.pitch<<","<<rateSp.yaw
               <<","<<rateSp.t0<<","<<rateSp.t1<<","<<rateSp.t2
               <<","<<int(attSp.valid)<<","<<int(rateSp.valid);
        } else {
            for(int i=0;i<45;++i)log<<",";
        }
        log<<"\n";
    }
    void AlignLocalFrame(const Sample& sample) {
        telemetry.haveLocalFrame=false;
        // 调试：对齐失败时把图像时间戳与 local 历史的实际区间打出来，
        // 用于区分「基座不同」「窗口不覆盖」「reset 计数变化」三类原因。
        if(!localHistory.empty()) {
            static uint64_t lastDbg=0;const uint64_t nowS=SteadyMicros();
            if(nowS-lastDbg>1000000) {lastDbg=nowS;
                std::fprintf(stderr,"[ALIGN] sample=%lld first=%lld last=%lld n=%zu d_first=%.1fms d_last=%.1fms\n",
                    (long long)sample.sampleUs,(long long)localHistory.front().timestamp,
                    (long long)localHistory.back().timestamp,localHistory.size(),
                    (double)((long long)sample.sampleUs-(long long)localHistory.front().timestamp)*1e-3,
                    (double)((long long)sample.sampleUs-(long long)localHistory.back().timestamp)*1e-3);
            }
        }
        // Match PX4 heading to the image sample, not the later SLAM completion time.
        for(size_t i=1;i<localHistory.size();++i) {
            const auto& a=localHistory[i-1];const auto& b=localHistory[i];
            if(a.timestamp>sample.sampleUs || b.timestamp<sample.sampleUs)continue;
            if(b.timestamp-a.timestamp>150000 || a.heading_reset_counter!=b.heading_reset_counter ||
               a.xy_reset_counter!=b.xy_reset_counter || a.z_reset_counter!=b.z_reset_counter)return;
            const double u=double(sample.sampleUs-a.timestamp)/double(b.timestamp-a.timestamp);
            const double delta=std::atan2(std::sin(double(b.heading-a.heading)),std::cos(double(b.heading-a.heading)));
            telemetry.localHeading=a.heading+u*delta;
            telemetry.localPosition=(1.-u)*Eigen::Vector3d(a.x,a.y,a.z)+u*Eigen::Vector3d(b.x,b.y,b.z);
            telemetry.localSampleUs=sample.sampleUs;telemetry.haveLocalFrame=true;
            telemetry.localXYValid=a.xy_valid && b.xy_valid;telemetry.localZValid=a.z_valid && b.z_valid;
            return;
        }
    }
    void Run() noexcept {
        try {
            Sample sample;uint64_t lastReport=0,lastRate=SteadyMicros(),lastSent=0,dropped=0;
            double hz=0;std::string lastReason;
            uint64_t noOutputSince=0,lastWarning=0,lastHealth=0;
            bool haveSample=false;
            while(!stop) {
                executor->spin_some(std::chrono::milliseconds(2));
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    if(!queue.empty()) {sample=queue.front();haveSample=true;}
                    dropped=overwritten;pending=false;
                }
                telemetry.haveReader=publisher->get_subscription_count()>0;
                const auto now=SystemMicros(),steady=SteadyMicros();
                // 等待「能包住样本时间戳的 PX4 local 包」：
                // 真实 VIO 输出天然带处理延迟，local 历史早已越过样本时间；真值回灌
                // 是零延迟源，而 SITL 的 local 时间戳又比主机表慢约 80 ms（timesync
                // 伪影），样本会一直比 local 历史新。这里只推迟提交，绝不修改样本
                // 时间戳；等不到（超过 max_age）就照常交给 Gate 判 stale。
                const bool bracketable=!localHistory.empty() && sample.sampleUs &&
                    sample.sampleUs<=localHistory.back().timestamp;
                const bool waitLocal=config.alignToPx4Ned && haveSample && !bracketable &&
                    sample.sampleUs && now<=sample.sampleUs+uint64_t(config.maxAgeSeconds*1e6);
                if(snapshot && haveSample && waitLocal) {
                    GateSnapshotRecord w;w.decision=4;w.reason="wait_px4_local_frame";
                    w.evalWallUs=now;w.evalMonoUs=steady;
                    snapshot->Mark(sample.sequence,w);
                }
                std::optional<Observation> output;
                if(!waitLocal && haveSample) {
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        if(!queue.empty() && queue.front().sequence==sample.sequence)queue.pop_front();
                    }
                    if(config.alignToPx4Ned)AlignLocalFrame(sample);
                    // 每秒一次的入口探针：确认配置开关、local 历史长度、融合状态。
                    {
                        static uint64_t lastProbe=0;
                        if(!lastProbe || steady-lastProbe>=1000000) {
                            lastProbe=steady;
                            std::fprintf(stderr,"[PROBE] align=%d hist=%zu haveLocal=%d ev=%d/%d hz=%.1f reason=%s\n",
                                (int)config.alignToPx4Ned,localHistory.size(),(int)telemetry.haveLocalFrame,
                                (int)telemetry.evPosition,(int)telemetry.evVelocity,hz,gate.Reason().c_str());
                        }
                    }
                    output=gate.Evaluate(sample,telemetry,now,steady);
                    if(snapshot) {
                        const GateDiagnostic gd=gate.Diagnostic();
                        const GateComparison& cm=gd.comparison;
                        const std::string why=gate.Reason();
                        GateSnapshotRecord d;
                        d.decision=output?1:
                            ((why=="stabilizing"||why=="waiting_new_frame"||why=="px4_local_frame_unready")?3:2);
                        d.reason=why;d.faultReason=gd.faultReason;
                        d.evalWallUs=now;d.evalMonoUs=steady;d.ageMs=gate.AgeSeconds()*1000;
                        d.sent=gate.Sent();d.rejected=gate.Rejected();
                        d.armed=telemetry.armed;d.navState=int(navMode);
                        d.landed=(telemetry.haveLand && telemetry.landed);
                        d.havePrevious=cm.havePair;d.comparable=cm.comparable;
                        d.dt=cm.dt;d.angleDeg=cm.angleDeg;d.limitDeg=cm.limitDeg;
                        d.posResidual=cm.posResidual;
                        d.prevSeq=cm.prevSeq;d.prevSampleUs=cm.prevSampleUs;
                        d.prevRefEpoch=cm.prevRefEpoch;d.prevClockEpoch=cm.prevClockEpoch;
                        d.prevMapId=cm.prevMapId;d.prevInitialKfId=cm.prevInitialKeyFrameId;
                        d.prevFrameId=cm.prevFrameId;
                        const Eigen::Quaterniond pq(cm.prevRotation);
                        d.prevQ=SnapQuat{pq.w(),pq.x(),pq.y(),pq.z()};
                        d.prevP=SnapVec3{cm.prevPosition.x(),cm.prevPosition.y(),cm.prevPosition.z()};
                        d.prevV=SnapVec3{cm.prevVelocity.x(),cm.prevVelocity.y(),cm.prevVelocity.z()};
                        const Eigen::Quaterniond pmq(cm.prevMapToWorld);
                        d.prevMapQ=SnapQuat{pmq.w(),pmq.x(),pmq.y(),pmq.z()};
                        d.prevMapT=SnapVec3{cm.prevMapTranslation.x(),cm.prevMapTranslation.y(),cm.prevMapTranslation.z()};
                        d.haveReference=gd.haveReference;d.airborneFault=gd.airborneFault;
                        d.resetCounter=gd.resetCounter;
                        const Eigen::Quaterniond fq(gd.flightFromWorld);
                        d.flightQ=SnapQuat{fq.w(),fq.x(),fq.y(),fq.z()};
                        d.origin=SnapVec3{gd.origin.x(),gd.origin.y(),gd.origin.z()};
                        d.outputOrigin=SnapVec3{gd.outputOrigin.x(),gd.outputOrigin.y(),gd.outputOrigin.z()};
                        if(output) {
                            d.published=1;
                            d.outP=SnapVec3{output->position.x(),output->position.y(),output->position.z()};
                            d.outV=SnapVec3{output->velocity.x(),output->velocity.y(),output->velocity.z()};
                            d.outQ=SnapQuat{output->attitude.w(),output->attitude.x(),output->attitude.y(),output->attitude.z()};
                            d.poseFrame=output->nedFrame?1:0;
                        }
                        GateSnapshotRecord cur=MakeSubmitRecord(sample);
                        cur.Overlay(d);
                        d.crossTransform=GateSnapshotCrossTransform(cur)?1:0;
                        snapshot->Mark(sample.sequence,d);
                    }
                    {
                        std::lock_guard<std::mutex> lock(faultMutex);
                        if(output) {publishedOnce=true;faultReason.clear();}
                        else if(publishedOnce && gate.Reason()!="waiting_new_frame" && gate.Reason()!="stabilizing")faultReason=gate.Reason();
                    }
                    if(output || !publishedOnce) {noOutputSince=0;lastWarning=0;}
                    else {
                        if(!noOutputSince)noOutputSince=steady;
                        if(steady-noOutputSince>=10000000 && (!lastWarning || steady-lastWarning>=10000000)) {
                            std::cerr<<"[PX4] WARN no accepted observation for "<<(steady-noOutputSince)*1e-6
                                <<" s; reason="<<gate.Reason()<<" armed="<<telemetry.armed<<std::endl;
                            lastWarning=steady;
                        }
                    }
                    if(output) {
                        const auto& o=*output;Odom msg{};
                        msg.timestamp=o.timestampUs;msg.timestamp_sample=o.sampleUs;
                        msg.pose_frame=o.nedFrame?Odom::POSE_FRAME_NED:Odom::POSE_FRAME_FRD;
                        msg.velocity_frame=o.nedFrame?Odom::VELOCITY_FRAME_NED:Odom::VELOCITY_FRAME_FRD;
                        msg.q={float(o.attitude.w()),float(o.attitude.x()),float(o.attitude.y()),float(o.attitude.z())};
                        for(int i=0;i<3;++i) {
                            msg.position[i]=o.position(i);msg.velocity[i]=o.velocity(i);
                            msg.angular_velocity[i]=std::numeric_limits<float>::quiet_NaN();
                            msg.position_variance[i]=o.positionVariance;msg.velocity_variance[i]=o.velocityVariance;
                            msg.orientation_variance[i]=o.orientationVariance;
                        }
                        msg.reset_counter=o.resetCounter;msg.quality=0;publisher->publish(msg);Record(sample,&o,dropped);
                    }
                }
                if(!lastHealth || steady-lastHealth>=200000) {
                    const auto h=gate.Health(telemetry,steady);
                    std::ostringstream json;json<<std::boolalpha
                        <<"{\"schema_version\":1,\"instance_id\":\""<<instanceId
                        <<"\",\"timestamp_us\":"<<now<<",\"steady_us\":"<<steady
                        <<",\"navigation_ready\":"<<h.navigationReady<<",\"ground_ready\":"<<h.groundReady
                        <<",\"ground_confirmed\":"<<h.groundConfirmed<<",\"armed\":"<<telemetry.armed
                        <<",\"landed\":"<<(telemetry.haveLand && telemetry.landed)
                        <<",\"airborne_fault\":"<<h.airborneFault<<",\"requires_disarm\":"<<h.requiresDisarm
                        <<",\"ground_recoveries\":"<<h.groundRecoveries<<",\"reset_counter\":"<<int(h.resetCounter)
                        <<",\"reference_epoch\":"<<sample.referenceEpoch<<",\"clock_epoch\":"<<sample.clockEpoch
                        <<",\"px4_xy_reset_counter\":"<<int(telemetry.localXYReset)
                        <<",\"px4_z_reset_counter\":"<<int(telemetry.localZReset)
                        <<",\"px4_heading_reset_counter\":"<<int(telemetry.localHeadingReset)
                        <<",\"reason\":\""<<h.reason<<"\",\"gate_reason\":\""<<gate.Reason()
                        <<"\",\"fault_reason\":\""<<h.faultReason<<"\"}";
                    std_msgs::msg::String message;message.data=json.str();healthPublisher->publish(message);lastHealth=steady;
                }
                // Treat normal per-frame waiting/publishing as one status to avoid 60 log lines/s.
                const std::string reason=gate.Reason()=="waiting_new_frame"?"publishing":gate.Reason();
                if(steady-lastRate>=1000000) {
                    hz=(gate.Sent()-lastSent)*1e6/double(steady-lastRate);
                    lastSent=gate.Sent();lastRate=steady;
                }
                if(steady-lastReport>=1000000 || reason!=lastReason) {
                    const bool flagsFresh=flagsReceived && steady-flagsReceived<1000000;
                    const bool localFresh=positionReceived && steady-positionReceived<1000000;
                    std::cout<<"[PX4] "<<reason<<" hz="<<hz<<" age_ms="<<gate.AgeSeconds()*1000
                        <<" sent="<<gate.Sent()<<" rejected="<<gate.Rejected()<<" overwritten="<<dropped
                        <<" armed="<<telemetry.armed<<" mode="<<int(navMode)
                        <<" EV(pos/hgt/vel)="<<(flagsFresh?int(flags.cs_ev_pos):-1)<<"/"
                        <<(flagsFresh?int(flags.cs_ev_hgt):-1)<<"/"<<(flagsFresh?int(flags.cs_ev_vel):-1)
                        <<" local_valid="<<(localFresh?int(position.xy_valid&&position.z_valid):-1)<<std::endl;
                    Record(sample,nullptr,dropped);if(log)log.flush();
                    lastReport=steady;lastReason=reason;
                }
                if(snapshot)snapshot->Flush(false);
                std::unique_lock<std::mutex> lock(mutex);
                wake.wait_for(lock,std::chrono::milliseconds(5),[this]{return stop||pending;});
            }
        } catch(const std::exception& e) {
            std::cerr<<"[PX4] BRIDGE_FAILED: "<<e.what()<<"; publication stopped"<<std::endl;
        }
    }
};
Bridge::Bridge(const Config& config):impl_(std::make_unique<Impl>(config)) {}
Bridge::~Bridge()=default;
std::string Bridge::FaultReason() const {
    std::lock_guard<std::mutex> lock(impl_->faultMutex);return impl_->faultReason;
}
void Bridge::Submit(Sample s) {impl_->Submit(std::move(s));}
void Bridge::InvalidateClock(uint64_t epoch) {Sample s;s.clockEpoch=epoch;Submit(std::move(s));}
}}
