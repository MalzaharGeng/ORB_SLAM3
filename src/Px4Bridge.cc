#include "Px4Bridge.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>

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
static_assert(Odom::MESSAGE_VERSION==0 && Status::MESSAGE_VERSION==1 && Position::MESSAGE_VERSION==1,
              "Build with PX4 1.17 px4_msgs definitions");
}
struct Bridge::Impl {
    Config config;
    Gate gate;
    std::shared_ptr<rclcpp::Context> context;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Publisher<Odom>::SharedPtr publisher;
    rclcpp::Subscription<Status>::SharedPtr statusSub;
    rclcpp::Subscription<Position>::SharedPtr positionSub;
    rclcpp::Subscription<Flags>::SharedPtr flagsSub;
    rclcpp::Subscription<Sync>::SharedPtr syncSub;
    Telemetry telemetry;
    Position position{};
    Flags flags{};
    uint8_t navMode=0;
    uint64_t positionReceived=0,flagsReceived=0;
    std::mutex mutex;
    std::condition_variable wake;
    Sample mailbox;
    bool pending=false;
    uint64_t sequence=0,continuity=0,overwritten=0;
    std::atomic<bool> stop{false};
    std::thread worker;
    std::ofstream log;

    explicit Impl(const Config& c):config(c),gate(c) {
        if(!config.logPath.empty()) {
            log.open(config.logPath);
            if(!log)throw std::runtime_error("Cannot open PX4 telemetry log: "+config.logPath);
            log<<"system_us,event,reason,sample_us,age_ms,sent,rejected,overwritten,armed,nav_mode,ev_pos,ev_hgt,ev_vel,reject_pos,reject_vel,local_xy_valid,local_z_valid,px,py,pz,vx,vy,vz,reset_counter,ekf_x,ekf_y,ekf_z,ekf_vx,ekf_vy,ekf_vz,ekf_vxy_valid,ekf_vz_valid,status_age_ms,sync_age_ms,flags_age_ms,position_age_ms,sync_rtt_us,sync_residual_us\n";
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
        statusSub=node->create_subscription<Status>(topic("/fmu/out/vehicle_status",Version<Status>::value),qos,
            [this](Status::ConstSharedPtr m){telemetry.haveStatus=true;telemetry.statusSteadyUs=SteadyMicros();
                telemetry.armed=m->arming_state!=Status::ARMING_STATE_DISARMED;navMode=m->nav_state;});
        syncSub=node->create_subscription<Sync>(topic("/fmu/out/timesync_status",Version<Sync>::value),qos,
            [this](Sync::ConstSharedPtr m){if(m->source_protocol!=Sync::SOURCE_PROTOCOL_DDS)return;
                telemetry.haveSync=true;telemetry.syncSteadyUs=SteadyMicros();telemetry.roundTripUs=m->round_trip_time;
                telemetry.observedOffsetUs=m->observed_offset;telemetry.estimatedOffsetUs=m->estimated_offset;});
        positionSub=node->create_subscription<Position>(topic("/fmu/out/vehicle_local_position",Version<Position>::value),qos,
            [this](Position::ConstSharedPtr m){position=*m;positionReceived=SteadyMicros();});
        flagsSub=node->create_subscription<Flags>(topic("/fmu/out/estimator_status_flags",Version<Flags>::value),qos,
            [this](Flags::ConstSharedPtr m){flags=*m;flagsReceived=SteadyMicros();});
        rclcpp::ExecutorOptions eo;eo.context=context;
        executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);executor->add_node(node);
        std::cout<<"[PX4] domain="<<config.domainId<<" topic="<<publisher->get_topic_name()
            <<" extrinsics="<<(config.extrinsicsValid?"configured":"MISSING (receive only)")<<std::endl;
        if(config.extrinsicsValid)std::cout<<"[PX4] R_BI configured; IMU lever arm is configured only in PX4 EKF2_EV_POS_X/Y/Z"<<std::endl;
        worker=std::thread([this]{Run();});
    }
    ~Impl() {
        stop=true;wake.notify_all();if(worker.joinable())worker.join();
        executor.reset();node.reset();context->shutdown("VIO bridge stopped");
    }
    void Submit(Sample s) {
        std::lock_guard<std::mutex> lock(mutex);
        if(!s.valid)++continuity;
        s.continuity=continuity;s.sequence=++sequence;
        if(pending)++overwritten;
        mailbox=std::move(s);pending=true;wake.notify_one();
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
           <<","<<(telemetry.observedOffsetUs-telemetry.estimatedOffsetUs)<<"\n";
    }
    void Run() noexcept {
        try {
            Sample sample;uint64_t lastReport=0,lastRate=SteadyMicros(),lastSent=0,dropped=0;
            double hz=0;std::string lastReason;
            while(!stop) {
                executor->spin_some(std::chrono::milliseconds(2));
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    if(pending) {sample=mailbox;pending=false;}
                    dropped=overwritten;
                }
                telemetry.haveReader=publisher->get_subscription_count()>0;
                const auto now=SystemMicros(),steady=SteadyMicros();
                auto output=gate.Evaluate(sample,telemetry,now,steady);
                if(output) {
                    const auto& o=*output;Odom msg{};
                    msg.timestamp=o.timestampUs;msg.timestamp_sample=o.sampleUs;
                    msg.pose_frame=Odom::POSE_FRAME_FRD;msg.velocity_frame=Odom::VELOCITY_FRAME_FRD;
                    msg.q={float(o.attitude.w()),float(o.attitude.x()),float(o.attitude.y()),float(o.attitude.z())};
                    for(int i=0;i<3;++i) {
                        msg.position[i]=o.position(i);msg.velocity[i]=o.velocity(i);
                        msg.angular_velocity[i]=std::numeric_limits<float>::quiet_NaN();
                        msg.position_variance[i]=o.positionVariance;msg.velocity_variance[i]=o.velocityVariance;
                        msg.orientation_variance[i]=o.orientationVariance;
                    }
                    msg.reset_counter=o.resetCounter;msg.quality=0;publisher->publish(msg);Record(sample,&o,dropped);
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
void Bridge::Submit(Sample s) {impl_->Submit(std::move(s));}
void Bridge::InvalidateClock(uint64_t epoch) {Sample s;s.clockEpoch=epoch;Submit(std::move(s));}
}}
