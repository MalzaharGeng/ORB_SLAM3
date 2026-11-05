#include "Px4Vio.h"
#include <opencv2/core.hpp>
#include <chrono>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <algorithm>

namespace ORB_SLAM3 { namespace px4 {
namespace {
bool rotation(const Eigen::Matrix3d& r) {
    return r.allFinite() && (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-4 &&
           std::abs(r.determinant()-1.)<1e-4;
}
bool fresh(uint64_t now, uint64_t then, uint64_t limit) { return then && now>=then && now-then<=limit; }
double number(const cv::FileStorage& fs, const char* key, double fallback) {
    const auto n=fs[key];
    if(n.empty()) return fallback;
    if(!n.isInt() && !n.isReal()) throw std::runtime_error(std::string(key)+" must be numeric");
    return double(n);
}
}

uint64_t SystemMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
uint64_t SteadyMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string TopicName(const std::string& prefix, const std::string& base, uint32_t version) {
    return prefix+base+(version ? "_v"+std::to_string(version) : "");
}

Config Config::Load(const std::string& path) {
    cv::FileStorage fs(path,cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::runtime_error("Cannot read PX4 config: "+path);
    Config c;
    const double calibrated=number(fs,"extrinsics_valid",0);
    if(calibrated!=0 && calibrated!=1) throw std::runtime_error("extrinsics_valid must be 0 or 1");
    c.extrinsicsValid=calibrated==1;
    if(!fs["T_BI"].empty())
        throw std::runtime_error("Replace T_BI with its 3x3 rotation R_BI; configure translation only in PX4 EKF2_EV_POS_X/Y/Z");
    if(!fs["R_BI"].empty()) {
        cv::Mat r;fs["R_BI"]>>r;
        if(r.rows!=3 || r.cols!=3 || r.channels()!=1 || (r.depth()!=CV_32F && r.depth()!=CV_64F))
            throw std::runtime_error("R_BI must be a floating-point 3x3 rotation (IMU to body FRD)");
        r.convertTo(r,CV_64F);
        for(int row=0;row<3;++row)
            for(int col=0;col<3;++col)c.bodyFromImu(row,col)=r.at<double>(row,col);
    } else if(c.extrinsicsValid) throw std::runtime_error("extrinsics_valid=1 requires R_BI");
    c.positionStd=number(fs,"position_std_m",c.positionStd);
    c.velocityStd=number(fs,"velocity_std_m_s",c.velocityStd);
    c.orientationStd=number(fs,"orientation_std_rad",c.orientationStd);
    c.stableSeconds=number(fs,"stable_seconds",c.stableSeconds);
    c.maxAgeSeconds=number(fs,"max_age_seconds",c.maxAgeSeconds);
    if(number(fs,"allow_airborne_reference",0)!=0)
        throw std::runtime_error("Airborne reference creation is unsupported; set allow_airborne_reference: 0");
    const double airborne=number(fs,"allow_airborne_initial_reference",0);
    if(airborne!=0 && airborne!=1)throw std::runtime_error("allow_airborne_initial_reference must be 0 or 1");
    c.allowAirborneInitialReference=airborne==1;
    c.consistencyWindowSeconds=number(fs,"consistency_window_seconds",c.consistencyWindowSeconds);
    c.maxWindowResidual=number(fs,"max_window_residual_m",c.maxWindowResidual);
    const double align=number(fs,"align_to_px4_ned",0);
    if(align!=0 && align!=1)throw std::runtime_error("align_to_px4_ned must be 0 or 1");
    c.alignToPx4Ned=align==1;
    c.maxSpeed=number(fs,"max_speed_m_s",c.maxSpeed);
    c.maxPositionResidual=number(fs,"max_position_residual_m",c.maxPositionResidual);
    c.maxSampleGap=number(fs,"max_sample_gap_seconds",c.maxSampleGap);
    c.maxSyncRttSeconds=number(fs,"max_sync_rtt_seconds",c.maxSyncRttSeconds);
    c.maxSyncResidualSeconds=number(fs,"max_sync_residual_seconds",c.maxSyncResidualSeconds);
    c.maxSyncAgeSeconds=number(fs,"max_sync_age_seconds",c.maxSyncAgeSeconds);
    const double landedRecovery=number(fs,"allow_armed_landed_recovery",0);
    if(landedRecovery!=0 && landedRecovery!=1)throw std::runtime_error("allow_armed_landed_recovery must be 0 or 1");
    c.allowArmedLandedRecovery=landedRecovery==1;
    c.landedStableSeconds=number(fs,"landed_stable_seconds",c.landedStableSeconds);
    c.landedMaxSpeed=number(fs,"landed_max_speed_m_s",c.landedMaxSpeed);
    const double domain=number(fs,"domain_id",0);
    if(!std::isfinite(domain) || domain<0 || domain>232 || domain!=std::floor(domain))
        throw std::runtime_error("domain_id must be an integer in [0,232]");
    c.domainId=static_cast<int>(domain);
    if(!fs["topic_prefix"].empty())fs["topic_prefix"]>>c.topicPrefix;
    if(!fs["log_path"].empty())fs["log_path"]>>c.logPath;
    c.Validate();return c;
}
void Config::Validate() const {
    if(!rotation(bodyFromImu))throw std::runtime_error("Invalid R_BI rotation");
    for(double v:{positionStd,velocityStd,orientationStd,stableSeconds,maxAgeSeconds})
        if(!std::isfinite(v) || v<=0)throw std::runtime_error("PX4 noise/timing values must be finite and positive");
    for(double v:{positionStd,velocityStd,orientationStd})
        if(v<1e-9 || v>1e9)throw std::runtime_error("PX4 standard deviations must be in [1e-9,1e9]");
    if(maxAgeSeconds>1 || stableSeconds>60)throw std::runtime_error("PX4 timing limits: max_age <= 1 s, stability <= 60 s");
    if(!std::isfinite(maxSpeed) || maxSpeed<=0 || maxSpeed>50 ||
       !std::isfinite(maxPositionResidual) || maxPositionResidual<=0 || maxPositionResidual>5 ||
       !std::isfinite(maxSampleGap) || maxSampleGap<=0 || maxSampleGap>2)
        throw std::runtime_error("Invalid PX4 motion gate limits");
    if(!std::isfinite(consistencyWindowSeconds) || consistencyWindowSeconds<.5 || consistencyWindowSeconds>10 ||
       !std::isfinite(maxWindowResidual) || maxWindowResidual<=0 || maxWindowResidual>5)
        throw std::runtime_error("Invalid cumulative consistency limits");
    if(!std::isfinite(maxSyncRttSeconds) || maxSyncRttSeconds<=0 || maxSyncRttSeconds>.1 ||
       !std::isfinite(maxSyncResidualSeconds) || maxSyncResidualSeconds<=0 || maxSyncResidualSeconds>.05 ||
       !std::isfinite(maxSyncAgeSeconds) || maxSyncAgeSeconds<=0 || maxSyncAgeSeconds>2)
        throw std::runtime_error("Invalid PX4 time-sync quality limits");
    if(!std::isfinite(landedStableSeconds) || landedStableSeconds<2 || landedStableSeconds>30 ||
       !std::isfinite(landedMaxSpeed) || landedMaxSpeed<=0 || landedMaxSpeed>.3)
        throw std::runtime_error("Invalid stationary landed recovery limits");
    if(domainId<0 || domainId>232)throw std::runtime_error("Invalid DDS domain");
    if(!topicPrefix.empty()) {
        if(topicPrefix.front()!='/' || topicPrefix.back()=='/' || topicPrefix.find("//")!=std::string::npos)
            throw std::runtime_error("topic_prefix must be empty or /namespace without trailing slash");
        for(char ch:topicPrefix) if(!(std::isalnum(static_cast<unsigned char>(ch)) || ch=='/' || ch=='_'))
            throw std::runtime_error("Invalid topic_prefix character");
    }
}
Gate::Gate(Config config):config_(std::move(config)) { config_.Validate(); }
void Gate::Reanchor() {
    ClearGround();
    haveReference_=false;havePrevious_=false;windowStartUs_=0;integratedVelocity_.setZero();stableSinceUs_=0;lastValidUs_=0;
    if(everPublished_)pendingReset_=true;
}
void Gate::ClearGround() {
    groundSinceUs_=0;groundLastUs_=0;groundConfirmed_=false;
}
void Gate::LatchFault(const char* why,bool requiresDisarm) {
    // A later motion reject must never downgrade a clock/map-frame failure.
    airborneFault_=true;requiresDisarm_=requiresDisarm_ || requiresDisarm;
    faultReason_=why;ClearGround();
    if(requiresDisarm_)Reanchor();
}
void Gate::UpdateGround(const Sample& s,const Telemetry& t,uint64_t steady,bool isNew) {
    const bool evidence=t.haveLand && fresh(steady,t.landSteadyUs,500000) &&
        fresh(lastSystemUs_,t.landSampleUs,500000) && t.landed &&
        !t.freefall && !t.landMoving && t.haveNavigationStatus && fresh(steady,t.localSteadyUs,200000) &&
        t.localVelocity.allFinite() && t.localVelocity.norm()<=config_.landedMaxSpeed &&
        s.velocity.norm()<=config_.landedMaxSpeed;
    if(!evidence) {ClearGround();return;}
    if(!isNew) {
        if(!fresh(steady,groundLastUs_,200000))ClearGround();
        return;
    }
    // A stable but permanently displaced pose does not prove frame continuity.
    // Refuse automatic recovery if it cannot return near the last checked pose.
    if(airborneFault_ && havePrevious_ && (s.position-previous_.position).norm()>config_.maxPositionResidual) {
        ClearGround();return;
    }
    if(!fresh(steady,groundLastUs_,200000) ||
       (s.position-groundPosition_).norm()>std::min(.1,config_.maxPositionResidual) ||
       Eigen::Quaterniond(s.imuToWorld).angularDistance(Eigen::Quaterniond(groundAttitude_))>.0872664626)
        ClearGround();
    if(!groundSinceUs_) {groundSinceUs_=steady;groundPosition_=s.position;groundAttitude_=s.imuToWorld;}
    groundLastUs_=steady;
    groundConfirmed_=steady>=groundSinceUs_ && steady-groundSinceUs_>=uint64_t(config_.landedStableSeconds*1e6);
}
NavigationHealth Gate::Health(const Telemetry& t,uint64_t steady) const {
    NavigationHealth h;h.airborneFault=airborneFault_;h.requiresDisarm=requiresDisarm_;
    h.groundRecoveries=groundRecoveries_;h.resetCounter=resetCounter_;h.faultReason=faultReason_;
    h.groundConfirmed=groundConfirmed_ && fresh(steady,groundLastUs_,200000) &&
        t.haveLand && fresh(steady,t.landSteadyUs,500000) && fresh(lastSystemUs_,t.landSampleUs,500000) &&
        t.landed && !t.freefall && !t.landMoving;
    const bool observationFresh=lastPublishedSteadyUs_ && steady>=lastPublishedSteadyUs_ &&
        lastPublishedAgeSeconds_+(steady-lastPublishedSteadyUs_)*1e-6<=config_.maxAgeSeconds;
    if(airborneFault_)h.reason="airborne_fault_latched";
    else if(!haveReference_ || !observationFresh || (reason_!="publishing" && reason_!="waiting_new_frame"))h.reason="vio_unready";
    else if(!t.haveStatus || !fresh(steady,t.statusSteadyUs,1000000) || !t.haveReader)h.reason="link_unavailable";
    else if(!t.haveSync || !fresh(steady,t.syncSteadyUs,uint64_t(config_.maxSyncAgeSeconds*1e6)) ||
            t.roundTripUs>config_.maxSyncRttSeconds*1e6 ||
            std::abs(double(t.observedOffsetUs)-double(t.estimatedOffsetUs))>config_.maxSyncResidualSeconds*1e6)h.reason="time_sync_unready";
    else if(!t.haveNavigationStatus || !fresh(steady,t.localSteadyUs,200000) ||
            !t.navigationPositionValid || !t.navigationVelocityValid || !t.localVelocity.allFinite() ||
            !t.haveFusionStatus || !fresh(steady,t.fusionSteadyUs,500000) || !t.evPosition)h.reason="px4_navigation_unready";
    else {h.navigationReady=true;h.reason="navigation_ready";}
    h.groundReady=h.navigationReady && h.groundConfirmed;
    return h;
}
bool Gate::Reject(const char* reason,bool newSample) {
    reason_=reason;stableSinceUs_=0;lastValidUs_=0;if(newSample)++rejected_;return false;
}
std::optional<Observation> Gate::Evaluate(const Sample& s,const Telemetry& t,uint64_t now,uint64_t steady) {
    // A fresh image may arrive just before the PX4 packet needed to bracket its
    // timestamp. Wait for that packet without consuming the sequence or clearing
    // a valid stabilization window. An expired image still takes the normal gate.
    const bool mayWaitForInitialLocal=!t.armed || (!everPublished_ && config_.allowAirborneInitialReference &&
        (t.navState==0 || t.navState==1 || t.navState==15));
    if(config_.alignToPx4Ned && !haveReference_ && !airborneFault_ && mayWaitForInitialLocal &&
       s.sampleUs && now>=s.sampleUs && now-s.sampleUs<=uint64_t(config_.maxAgeSeconds*1e6) &&
       (!t.haveLocalFrame || !fresh(steady,t.localSteadyUs,200000) || !t.localSampleUs ||
        std::abs(double(t.localSampleUs)-double(s.sampleUs))>150000 ||
        !std::isfinite(t.localHeading) || !t.localPosition.allFinite())) {
        reason_="px4_local_frame_unready";return std::nullopt;
    }
    const bool isNew=!seen_ || s.sequence!=lastSequence_;
    auto fail=[&](const char* why)->std::optional<Observation>{ClearGround();Reject(why,isNew);return std::nullopt;};
    bool clockFault=false;
    if(lastSteadyUs_) {
        const int64_t wallDelta=int64_t(now)-int64_t(lastSystemUs_);
        const int64_t monoDelta=int64_t(steady)-int64_t(lastSteadyUs_);
        clockFault=monoDelta<0 || std::abs(wallDelta-monoDelta)>50000;
    }
    lastSystemUs_=now;lastSteadyUs_=steady;
    if(config_.alignToPx4Ned && t.haveLocalFrame && fresh(steady,t.localSteadyUs,200000)) {
        if(seenLocalReset_ && (localXYReset_!=t.localXYReset || localZReset_!=t.localZReset ||
                              localHeadingReset_!=t.localHeadingReset)) {
            // On ground, EKF may reset as it first adopts this very EV frame.
            // Reanchoring in response would send another EV reset and create a
            // reset-feedback loop. Keep the established EV frame fixed there.
            const bool initialAdoption=initialAirborne_ && steady<=initialHandoverUntilUs_ &&
                (t.navState==0 || t.navState==1 || t.navState==15);
            if(t.armed && everPublished_ && !initialAdoption)LatchFault("px4_local_frame_reset",true);
        }
        seenLocalReset_=true;localXYReset_=t.localXYReset;localZReset_=t.localZReset;
        localHeadingReset_=t.localHeadingReset;
    }
    const bool epochChanged=seen_ && (s.referenceEpoch!=epoch_ || s.clockEpoch!=clockEpoch_);
    if(epochChanged) {if(t.armed && everPublished_)LatchFault("reference_or_clock_epoch_changed",true);else Reanchor();}
    if(seen_ && s.continuity!=continuity_) {stableSinceUs_=0;lastValidUs_=0;}
    if(isNew && seen_ && s.sampleUs && lastSampleUs_ && s.sampleUs<=lastSampleUs_)clockFault=true;
    epoch_=s.referenceEpoch;clockEpoch_=s.clockEpoch;continuity_=s.continuity;
    if(isNew && s.sampleUs)lastSampleUs_=s.sampleUs;
    lastSequence_=s.sequence;seen_=true;
    age_=(int64_t(now)-int64_t(s.sampleUs))*1e-6;
    if(clockFault) {if(t.armed && everPublished_)LatchFault("clock_discontinuity",true);else Reanchor();return fail("clock_discontinuity");}
    if(!config_.extrinsicsValid)return fail("extrinsics_missing");
    if(!t.haveStatus || !fresh(steady,t.statusSteadyUs,1000000) || !t.haveReader)return fail("link_unavailable");
    if(!t.haveSync || !fresh(steady,t.syncSteadyUs,uint64_t(config_.maxSyncAgeSeconds*1e6)) ||
       t.roundTripUs>config_.maxSyncRttSeconds*1e6 ||
       std::abs(double(t.observedOffsetUs)-double(t.estimatedOffsetUs))>config_.maxSyncResidualSeconds*1e6)
        return fail("time_sync_unready");
    if(airborneFault_ && !t.armed) {
        airborneFault_=false;requiresDisarm_=false;faultReason_.clear();Reanchor();
    }
    if(airborneFault_ && (requiresDisarm_ || !config_.allowArmedLandedRecovery))return fail("airborne_fault_latched");
    if(!s.sampleUs || age_<-.005) {
        if(t.armed && everPublished_)LatchFault("sample_clock_invalid",true);
        Reanchor();return fail("sample_clock_invalid");
    }
    if(age_<0)return fail("sample_in_future");
    if(age_>config_.maxAgeSeconds) {
        // Polling the last observation is not a new late measurement. Its age
        // grows by both processing latency and the next frame's arrival period.
        // Keep the startup window through an allowed receipt gap, without ever
        // republishing that old observation or accepting a late NEW observation.
        if(!isNew && lastValidUs_ && steady>=lastValidUs_ &&
           steady-lastValidUs_<=uint64_t(config_.maxSampleGap*1e6)) {
            reason_="waiting_new_frame";return std::nullopt;
        }
        return fail("sample_stale");
    }
    if(!s.valid)return fail("tracking_invalid");
    if(!s.imuInitialized || !s.ba2)return fail("inertial_initializing");
    if(!s.velocityValid || !s.position.allFinite() || !s.velocity.allFinite() ||
       !rotation(s.imuToWorld) || !s.gravity.allFinite() || s.gravity.norm()<1.)return fail("observation_invalid");
    UpdateGround(s,t,steady,isNew);
    if(airborneFault_) {
        if(!groundConfirmed_) {
            // Preserve independent ground evidence while the motion latch is held.
            Reject("airborne_fault_latched",isNew);return std::nullopt;
        }
        airborneFault_=false;faultReason_.clear();++groundRecoveries_;
        havePrevious_=false;windowStartUs_=0;integratedVelocity_.setZero();
        stableSinceUs_=0;lastValidUs_=0;
        // Keep haveReference_, transform, origin and reset_counter unchanged.
    }
    if(!haveReference_ && t.armed) {
        if(everPublished_ || !config_.allowAirborneInitialReference)return fail("reference_requires_disarmed");
        // PX4 MANUAL=0, ALTCTL=1, STAB=15. Never first-enable EV while
        // POSCTL/Offboard/auto control already relies on an existing position.
        if(t.navState!=0 && t.navState!=1 && t.navState!=15)return fail("airborne_initial_mode_unsafe");
        if(!config_.alignToPx4Ned)return fail("airborne_initial_requires_ned");
    }
    if(config_.alignToPx4Ned && !haveReference_ &&
       (!t.haveLocalFrame || !fresh(steady,t.localSteadyUs,200000) ||
        !t.localSampleUs || std::abs(double(t.localSampleUs)-double(s.sampleUs))>150000 ||
        !std::isfinite(t.localHeading) || !t.localPosition.allFinite()))
        return fail("px4_local_frame_unready");
    if(!isNew) {reason_=lastSentSequence_==s.sequence ? "waiting_new_frame" : reason_;return std::nullopt;}
    // Check sensor-time increments before either stabilization or publication.
    // Never clamp a corrupt pose into a plausible-looking measurement.
    auto motionFault=[&](const char* why)->std::optional<Observation>{
        if(t.armed && everPublished_)LatchFault(why,false);
        else {havePrevious_=false;windowStartUs_=0;integratedVelocity_.setZero();}
        return fail(why);
    };
    if(s.velocity.norm()>config_.maxSpeed)return motionFault("velocity_out_of_bounds");
    if(havePrevious_ && previous_.referenceEpoch==s.referenceEpoch && previous_.clockEpoch==s.clockEpoch) {
        const double dt=(double(s.sampleUs)-double(previous_.sampleUs))*1e-6;
        if(dt>config_.maxSampleGap && t.armed && everPublished_)return motionFault("observation_gap");
        if(dt>0 && dt<=config_.maxSampleGap) {
            const Eigen::Vector3d predicted=.5*(s.velocity+previous_.velocity)*dt;
            if((s.position-previous_.position-predicted).norm()>config_.maxPositionResidual)
                return motionFault("position_velocity_inconsistent");
            if(windowStartUs_)integratedVelocity_+=predicted;
        }
    }
    if(!windowStartUs_ || (havePrevious_ && s.sampleUs-previous_.sampleUs>uint64_t(config_.maxSampleGap*1e6))) {
        windowStartUs_=s.sampleUs;windowPosition_=s.position;integratedVelocity_.setZero();
    } else if(s.sampleUs-windowStartUs_>=uint64_t(config_.consistencyWindowSeconds*1e6)) {
        if((s.position-windowPosition_-integratedVelocity_).norm()>config_.maxWindowResidual)
            return motionFault("cumulative_position_velocity_inconsistent");
        windowStartUs_=s.sampleUs;windowPosition_=s.position;integratedVelocity_.setZero();
    }
    previous_=s;havePrevious_=true;
    const uint64_t prevValidUs=lastValidUs_;
    if(lastValidUs_ && (steady<lastValidUs_ || steady-lastValidUs_>uint64_t(config_.maxSampleGap*1e6)))stableSinceUs_=0;
    lastValidUs_=steady;
    if(!stableSinceUs_)stableSinceUs_=steady;
    if(steady-stableSinceUs_<uint64_t(config_.stableSeconds*1e6)) {reason_="stabilizing";return std::nullopt;}
    const Eigen::Matrix3d worldFromBody=s.imuToWorld*config_.bodyFromImu.transpose();
    if(!haveReference_) {
        initialAirborne_=t.armed && !everPublished_;
        initialHandoverUntilUs_=initialAirborne_?steady+3000000:0;
        const Eigen::Vector3d down=s.gravity.normalized();
        Eigen::Vector3d forward=worldFromBody.col(0)-down*down.dot(worldFromBody.col(0));
        if(forward.norm()<.1)return fail("heading_projection_invalid");
        forward.normalize();
        flightFromWorld_.row(0)=forward.transpose();
        flightFromWorld_.row(1)=down.cross(forward).transpose();
        flightFromWorld_.row(2)=down.transpose();
        outputOrigin_.setZero();
        if(config_.alignToPx4Ned) {
            // F's initial heading is zero; align once to the fresh PX4 local heading.
            // Do not feed fused PX4 yaw back into subsequent VIO observations.
            flightFromWorld_=Eigen::AngleAxisd(t.localHeading,Eigen::Vector3d::UnitZ()).toRotationMatrix()*flightFromWorld_;
            // Validity describes navigation confidence, not the numeric frame
            // origin. Invalid XY may still contain nonzero prior EKF state.
            // Preserve the finite, time-matched coordinates; never replace them
            // with zero and manufacture a first-observation position jump.
            outputOrigin_=t.localPosition;
        }
        // Observations stay at the IMU centre. PX4 alone compensates the lever arm.
        origin_=s.position;
        haveReference_=true;
        if(pendingReset_) {++resetCounter_;pendingReset_=false;}
    }
    // 动态协方差：连续性好时收紧（0.6x），跟踪中断/重定位时放宽（最高 20x）
    double quality=1.;
    if(prevValidUs && steady>prevValidUs) {
        const double gapSec=double(steady-prevValidUs)/1e6;
        if(gapSec>.10)quality*=1.0+(gapSec-.10)*20.0;
    }
    if(s.continuity!=varianceContinuity_) {varianceContinuity_=s.continuity;boostSamples_=10;}
    if(boostSamples_>0) {quality*=4.0;--boostSamples_;}
    if(quality>20.0)quality=20.0;
    if(quality<.25)quality=.25;
    Observation o;
    o.timestampUs=now;o.sampleUs=s.sampleUs;
    o.position=outputOrigin_+flightFromWorld_*(s.position-origin_);
    o.velocity=flightFromWorld_*s.velocity;
    o.attitude=Eigen::Quaterniond(flightFromWorld_*worldFromBody).normalized();
    o.positionVariance=config_.positionStd*config_.positionStd*quality;
    o.velocityVariance=config_.velocityStd*config_.velocityStd*quality;
    o.orientationVariance=config_.orientationStd*config_.orientationStd*quality;
    o.resetCounter=resetCounter_;o.nedFrame=config_.alignToPx4Ned;
    ++sent_;everPublished_=true;lastSentSequence_=s.sequence;reason_="publishing";
    lastPublishedSteadyUs_=steady;lastPublishedAgeSeconds_=age_;
    return o;
}

void SensorClock::Reset() { for(auto& s:streams_)s=Stream{};++epoch_; }
bool SensorClock::Ready() const { for(const auto& s:streams_)if(!s.ready)return false;return true; }
bool SensorClock::Accept(unsigned stream,bool global,double ms,uint64_t wall,uint64_t mono) {
    if(stream>=4)return false;
    bool bad=!global || !std::isfinite(ms) || std::abs(ms*1000.-double(wall))>200000.;
    if(steadyUs_)bad=bad || mono<steadyUs_ ||
        std::abs((int64_t(wall)-int64_t(systemUs_))-(int64_t(mono)-int64_t(steadyUs_)))>50000;
    systemUs_=wall;steadyUs_=mono;
    for(const auto& streamState:streams_)
        if(streamState.ready && (mono<streamState.steadyUs || mono-streamState.steadyUs>200000))bad=true;
    auto& s=streams_[stream];
    if(s.ready)bad=bad || ms<=s.timestampMs || ms-s.timestampMs>1000.;
    if(bad) {Reset();return false;}
    s={true,ms,mono};return true;
}
}} // namespace ORB_SLAM3::px4
