#pragma once

// Transport-independent observation conversion and publication gate.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string>

namespace ORB_SLAM3 { namespace px4 {

struct Config {
    bool extrinsicsValid = false;
    // Subsequent reference creation is ground-only; guarded first airborne
    // activation may be explicitly enabled in a manual/altitude mode.
    bool alignToPx4Ned = false;
    bool allowAirborneInitialReference = false;
    double consistencyWindowSeconds = 2., maxWindowResidual = .25;
    double maxSpeed = 5., maxPositionResidual = .5, maxSampleGap = .3;
    // Angular envelope catches attitude-only frame jumps (position may stay fixed).
    double maxAttitudeRateDeg = 180., maxAttitudeStepDeg = 10.;
    Eigen::Matrix3d bodyFromImu = Eigen::Matrix3d::Identity();
    double positionStd = .1, velocityStd = .2, orientationStd = .08726646259971647;
    double stableSeconds = 1., maxAgeSeconds = .2;
    // DDS jitter limits; freshness and timestamp age are independent gates.
    double maxSyncRttSeconds = .05, maxSyncResidualSeconds = .01, maxSyncAgeSeconds = 2.;
    // Opt-in: only motion faults may recover at a confirmed stationary landing.
    bool allowArmedLandedRecovery = false;
    double landedStableSeconds = 2., landedMaxSpeed = .2;
    int domainId = 0;
    std::string topicPrefix; // "" for hardware, "/vio_test" for replay
    std::string logPath;
    static Config Load(const std::string& path);
    void Validate() const;
};

struct Sample {
    uint64_t sequence = 0, continuity = 0, referenceEpoch = 0, clockEpoch = 0;
    uint64_t sampleUs = 0;
    bool valid = false, imuInitialized = false, ba2 = false, velocityValid = false;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d imuToWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(), gravity = Eigen::Vector3d::Zero();
    // [NP-01 FIX11][OBSERVATION-ONLY] Consistent value-copy chain snapshot.
    // These fields are metadata only: no Gate decision reads them.
    uint64_t pairId = 0;
    int64_t srcSimUs = 0, mapOffsetUs = 0;
    unsigned long mapId = 0, initialKeyFrameId = 0, frameId = 0;
    Eigen::Matrix3d mapToWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d mapTranslation = Eigen::Vector3d::Zero();
    Eigen::Matrix3d initialImuToWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d initialImuTranslation = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rawCameraToWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d rawCameraPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroIntegral = Eigen::Vector3d::Zero();  // rad over this frame's IMU interval
    double gyroSpanS = 0;
    uint64_t imuCount = 0;
};

struct Telemetry {
    bool haveStatus = false, armed = false, haveSync = false, haveReader = false;
    uint64_t statusSteadyUs = 0, syncSteadyUs = 0;
    int64_t observedOffsetUs = 0, estimatedOffsetUs = 0;
    uint32_t roundTripUs = 0;
    uint8_t navState = 255;
    bool haveLocalFrame = false, localXYValid = false, localZValid = false;
    uint64_t localSteadyUs = 0, localSampleUs = 0;
    Eigen::Vector3d localPosition = Eigen::Vector3d::Zero();
    double localHeading = 0.;
    uint8_t localXYReset = 0, localZReset = 0, localHeadingReset = 0;
    bool haveLand = false, landed = false, freefall = false, landMoving = false;
    uint64_t landSteadyUs = 0, landSampleUs = 0;
    bool haveNavigationStatus = false, navigationPositionValid = false, navigationVelocityValid = false;
    Eigen::Vector3d localVelocity = Eigen::Vector3d::Zero();
    bool haveFusionStatus = false, evPosition = false, evVelocity = false;
    uint64_t fusionSteadyUs = 0;
};

struct Observation {
    uint64_t timestampUs = 0, sampleUs = 0;
    Eigen::Vector3d position, velocity;
    Eigen::Quaterniond attitude;
    double positionVariance = 0, velocityVariance = 0, orientationVariance = 0;
    uint8_t resetCounter = 0;
    bool nedFrame = false;
};

struct NavigationHealth {
    bool navigationReady = false, groundReady = false, groundConfirmed = false;
    bool airborneFault = false, requiresDisarm = false;
    uint64_t groundRecoveries = 0;
    uint8_t resetCounter = 0;
    std::string reason, faultReason;
};

// [NP-01 FIX11] Value-copy of the exact operands Gate::Evaluate compared, taken
// BEFORE previous_ is reassigned at the end of a successful evaluation. This is
// the only way to replay a REJECTED pair: after Evaluate returns, previous_ may
// already hold the current sample (accept) or a stale one (reject).
struct GateComparison {
    bool havePair = false, comparable = false;
    double dt = 0, angleDeg = 0, limitDeg = 0, posResidual = 0;
    uint64_t prevSeq = 0, prevSampleUs = 0, prevRefEpoch = 0, prevClockEpoch = 0;
    unsigned long prevMapId = 0, prevInitialKeyFrameId = 0, prevFrameId = 0;
    Eigen::Vector3d prevPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d prevVelocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d prevRotation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d prevMapToWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d prevMapTranslation = Eigen::Vector3d::Zero();
};

struct GateDiagnostic {
    GateComparison comparison;
    bool haveReference = false, airborneFault = false;
    Eigen::Matrix3d flightFromWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d outputOrigin = Eigen::Vector3d::Zero();
    uint64_t resetCounter = 0;
    std::string reason, faultReason;
    uint64_t sent = 0, rejected = 0;
};

class Gate {
public:
    explicit Gate(Config config);
    std::optional<Observation> Evaluate(const Sample&, const Telemetry&, uint64_t systemUs, uint64_t steadyUs);
    const std::string& Reason() const { return reason_; }
    uint64_t Sent() const { return sent_; }
    uint64_t Rejected() const { return rejected_; }
    double AgeSeconds() const { return age_; }
    NavigationHealth Health(const Telemetry&, uint64_t steadyUs) const;
    // Observation only: returns value copies, never references or pointers.
    GateDiagnostic Diagnostic() const;
private:
    void Reanchor();
    void ClearGround();
    void UpdateGround(const Sample&, const Telemetry&, uint64_t steadyUs, bool newSample);
    void LatchFault(const char* reason, bool requiresDisarm);
    bool Reject(const char* reason, bool newSample);
    Config config_;
    std::string reason_ = "waiting_sample";
    bool haveReference_ = false, everPublished_ = false, pendingReset_ = false, seen_ = false;
    Eigen::Matrix3d flightFromWorld_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d outputOrigin_ = Eigen::Vector3d::Zero();
    bool airborneFault_ = false, havePrevious_ = false;
    bool requiresDisarm_ = false;
    std::string faultReason_;
    uint64_t groundSinceUs_ = 0, groundLastUs_ = 0, groundRecoveries_ = 0;
    uint64_t lastPublishedSteadyUs_ = 0;
    double lastPublishedAgeSeconds_ = 0;
    bool groundConfirmed_ = false;
    Eigen::Vector3d groundPosition_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d groundAttitude_ = Eigen::Matrix3d::Identity();
    Sample previous_;
    GateComparison lastComparison_;
    Eigen::Vector3d windowPosition_ = Eigen::Vector3d::Zero(), integratedVelocity_ = Eigen::Vector3d::Zero();
    uint64_t windowStartUs_ = 0;
    bool initialAirborne_ = false;
    uint64_t initialHandoverUntilUs_ = 0;
    bool seenLocalReset_ = false;
    uint8_t localXYReset_ = 0, localZReset_ = 0, localHeadingReset_ = 0;
    uint64_t epoch_ = 0, clockEpoch_ = 0, continuity_ = 0, lastSequence_ = 0;
    uint64_t lastSampleUs_ = 0, lastSentSequence_ = 0, stableSinceUs_ = 0, lastValidUs_ = 0;
    uint64_t lastSystemUs_ = 0, lastSteadyUs_ = 0, sent_ = 0, rejected_ = 0;
    uint8_t resetCounter_ = 0;
    double age_ = 0;
    // 动态协方差用：跟踪连续性变化计数 + 变化后放大窗口
    uint64_t varianceContinuity_ = 0;
    int boostSamples_ = 0;
};

// [NP01-FIX15] 诊断用：把"为什么拒绝"拆成互斥的布尔原因，并指出触发停更的流。
// 纯只读诊断，不参与任何门限计算，不改变 Accept 的返回值语义。
// 背景：只打印 age/dt/gap 无法区分原因——"本路 gap=4 ms"并不代表别的流没停更，
// 一次 200 ms 以上的调度饥饿会让 stalled 命中并升级为 domainReset，日志看起来
// 却像是本路时间戳有问题。
struct SensorClockReject {
    bool rejected = false;
    bool domainFault = false;   // 非全局时间 / 时间戳非有限
    bool staleSample = false;   // 本样本 epoch 年龄超出 200 ms
    bool hostArtefact = false;  // 主机 system/steady 时钟关系异常
    bool stalled = false;       // 某一路已就绪的流停更 > 200 ms
    bool srcFault = false;      // 本流源时间未前进 / 跳变 > 1 s
    bool domainReset = false;   // 拒绝升级为时钟域重置（清空全部流 + epoch++）
    int  stalledStream = -1;    // 触发 stalled 的流编号；-1 表示无
};

// The capture callback uses one clock per selected stream (left/right/gyro/accel).
// A time-domain transition or clock jump invalidates the entire sample bundle.
class SensorClock {
public:
    bool Accept(unsigned stream, bool globalTime, double timestampMs, uint64_t systemUs, uint64_t steadyUs);
    // 同上，但把拒绝原因写进 out（接受时 out->rejected=false）。返回值语义不变。
    bool Accept(unsigned stream, bool globalTime, double timestampMs, uint64_t systemUs, uint64_t steadyUs, SensorClockReject * out);
    bool Ready() const;
    uint64_t Epoch() const { return epoch_; }
    const SensorClockReject & LastReject() const { return lastReject_; }
private:
    struct Stream { bool ready = false; double timestampMs = 0; uint64_t steadyUs = 0; } streams_[4];
    uint64_t epoch_ = 0, systemUs_ = 0, steadyUs_ = 0;
    SensorClockReject lastReject_;
    void Reset();
};

uint64_t SystemMicros();
uint64_t SteadyMicros();
std::string TopicName(const std::string& prefix, const std::string& base, uint32_t version);

}} // namespace ORB_SLAM3::px4
