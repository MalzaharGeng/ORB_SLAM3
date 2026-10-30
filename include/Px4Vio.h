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
    Eigen::Matrix3d bodyFromImu = Eigen::Matrix3d::Identity();
    double positionStd = .1, velocityStd = .2, orientationStd = .08726646259971647;
    double stableSeconds = 1., maxAgeSeconds = .2;
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
};

struct Telemetry {
    bool haveStatus = false, armed = false, haveSync = false, haveReader = false;
    uint64_t statusSteadyUs = 0, syncSteadyUs = 0;
    int64_t observedOffsetUs = 0, estimatedOffsetUs = 0;
    uint32_t roundTripUs = 0;
};

struct Observation {
    uint64_t timestampUs = 0, sampleUs = 0;
    Eigen::Vector3d position, velocity;
    Eigen::Quaterniond attitude;
    double positionVariance = 0, velocityVariance = 0, orientationVariance = 0;
    uint8_t resetCounter = 0;
};

class Gate {
public:
    explicit Gate(Config config);
    std::optional<Observation> Evaluate(const Sample&, const Telemetry&, uint64_t systemUs, uint64_t steadyUs);
    const std::string& Reason() const { return reason_; }
    uint64_t Sent() const { return sent_; }
    uint64_t Rejected() const { return rejected_; }
    double AgeSeconds() const { return age_; }
private:
    void Reanchor();
    bool Reject(const char* reason, bool newSample);
    Config config_;
    std::string reason_ = "waiting_sample";
    bool haveReference_ = false, everPublished_ = false, pendingReset_ = false, seen_ = false;
    Eigen::Matrix3d flightFromWorld_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    uint64_t epoch_ = 0, clockEpoch_ = 0, continuity_ = 0, lastSequence_ = 0;
    uint64_t lastSampleUs_ = 0, lastSentSequence_ = 0, stableSinceUs_ = 0, lastValidUs_ = 0;
    uint64_t lastSystemUs_ = 0, lastSteadyUs_ = 0, sent_ = 0, rejected_ = 0;
    uint8_t resetCounter_ = 0;
    double age_ = 0;
};

// The capture callback uses one clock per selected stream (left/right/gyro/accel).
// A time-domain transition or clock jump invalidates the entire sample bundle.
class SensorClock {
public:
    bool Accept(unsigned stream, bool globalTime, double timestampMs, uint64_t systemUs, uint64_t steadyUs);
    bool Ready() const;
    uint64_t Epoch() const { return epoch_; }
private:
    struct Stream { bool ready = false; double timestampMs = 0; uint64_t steadyUs = 0; } streams_[4];
    uint64_t epoch_ = 0, systemUs_ = 0, steadyUs_ = 0;
    void Reset();
};

uint64_t SystemMicros();
uint64_t SteadyMicros();
std::string TopicName(const std::string& prefix, const std::string& base, uint32_t version);

}} // namespace ORB_SLAM3::px4
