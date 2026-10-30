#include "Px4Vio.h"
#include <opencv2/core.hpp>
#include <chrono>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

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
    haveReference_=false;stableSinceUs_=0;lastValidUs_=0;
    if(everPublished_)pendingReset_=true;
}
bool Gate::Reject(const char* reason,bool newSample) {
    reason_=reason;stableSinceUs_=0;lastValidUs_=0;if(newSample)++rejected_;return false;
}
std::optional<Observation> Gate::Evaluate(const Sample& s,const Telemetry& t,uint64_t now,uint64_t steady) {
    const bool isNew=!seen_ || s.sequence!=lastSequence_;
    auto fail=[&](const char* why)->std::optional<Observation>{Reject(why,isNew);return std::nullopt;};
    bool clockFault=false;
    if(lastSteadyUs_) {
        const int64_t wallDelta=int64_t(now)-int64_t(lastSystemUs_);
        const int64_t monoDelta=int64_t(steady)-int64_t(lastSteadyUs_);
        clockFault=monoDelta<0 || std::abs(wallDelta-monoDelta)>50000;
    }
    lastSystemUs_=now;lastSteadyUs_=steady;
    if(seen_ && (s.referenceEpoch!=epoch_ || s.clockEpoch!=clockEpoch_)) Reanchor();
    if(seen_ && s.continuity!=continuity_) {stableSinceUs_=0;lastValidUs_=0;}
    if(isNew && seen_ && s.sampleUs && lastSampleUs_ && s.sampleUs<=lastSampleUs_)clockFault=true;
    epoch_=s.referenceEpoch;clockEpoch_=s.clockEpoch;continuity_=s.continuity;
    if(isNew && s.sampleUs)lastSampleUs_=s.sampleUs;
    lastSequence_=s.sequence;seen_=true;
    age_=(int64_t(now)-int64_t(s.sampleUs))*1e-6;
    if(clockFault) {Reanchor();return fail("clock_discontinuity");}
    if(!config_.extrinsicsValid)return fail("extrinsics_missing");
    if(!t.haveStatus || !fresh(steady,t.statusSteadyUs,1000000) || !t.haveReader)return fail("link_unavailable");
    if(!t.haveSync || !fresh(steady,t.syncSteadyUs,2000000) || t.roundTripUs>20000 ||
       std::abs(double(t.observedOffsetUs)-double(t.estimatedOffsetUs))>5000)return fail("time_sync_unready");
    if(!s.sampleUs || age_<-.005) {Reanchor();return fail("sample_clock_invalid");}
    if(age_<0)return fail("sample_in_future");
    if(age_>config_.maxAgeSeconds)return fail("sample_stale");
    if(!s.valid)return fail("tracking_invalid");
    if(!s.imuInitialized || !s.ba2)return fail("inertial_initializing");
    if(!s.velocityValid || !s.position.allFinite() || !s.velocity.allFinite() ||
       !rotation(s.imuToWorld) || !s.gravity.allFinite() || s.gravity.norm()<1.)return fail("observation_invalid");
    if(!haveReference_ && t.armed)return fail("reference_requires_disarmed");
    if(!isNew) {reason_=lastSentSequence_==s.sequence ? "waiting_new_frame" : reason_;return std::nullopt;}
    if(lastValidUs_ && (steady<lastValidUs_ || steady-lastValidUs_>uint64_t(config_.maxAgeSeconds*1e6)))stableSinceUs_=0;
    lastValidUs_=steady;
    if(!stableSinceUs_)stableSinceUs_=steady;
    if(steady-stableSinceUs_<uint64_t(config_.stableSeconds*1e6)) {reason_="stabilizing";return std::nullopt;}
    const Eigen::Matrix3d worldFromBody=s.imuToWorld*config_.bodyFromImu.transpose();
    if(!haveReference_) {
        const Eigen::Vector3d down=s.gravity.normalized();
        Eigen::Vector3d forward=worldFromBody.col(0)-down*down.dot(worldFromBody.col(0));
        if(forward.norm()<.1)return fail("heading_projection_invalid");
        forward.normalize();
        flightFromWorld_.row(0)=forward.transpose();
        flightFromWorld_.row(1)=down.cross(forward).transpose();
        flightFromWorld_.row(2)=down.transpose();
        // Observations stay at the IMU centre. PX4 alone compensates the lever arm.
        origin_=s.position;
        haveReference_=true;
        if(pendingReset_) {++resetCounter_;pendingReset_=false;}
    }
    Observation o;
    o.timestampUs=now;o.sampleUs=s.sampleUs;
    o.position=flightFromWorld_*(s.position-origin_);
    o.velocity=flightFromWorld_*s.velocity;
    o.attitude=Eigen::Quaterniond(flightFromWorld_*worldFromBody).normalized();
    o.positionVariance=config_.positionStd*config_.positionStd;
    o.velocityVariance=config_.velocityStd*config_.velocityStd;
    o.orientationVariance=config_.orientationStd*config_.orientationStd;
    o.resetCounter=resetCounter_;
    ++sent_;everPublished_=true;lastSentSequence_=s.sequence;reason_="publishing";
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
