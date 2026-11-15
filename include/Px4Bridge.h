#pragma once
#include "Px4Vio.h"
#include <memory>

namespace ORB_SLAM3 { namespace px4 {
// [NP-01 FIX11][OBSERVATION-ONLY] Metadata captured at the data owner (the
// node's value copy of GetNavigationState plus the IMU bundle consumed by
// TrackStereo). Default-constructed for every other caller, so truth_odom_node
// and the recovery replay tools keep compiling unchanged.
struct SubmitMeta {
    uint64_t pairId = 0;
    int64_t srcSimUs = 0, mapOffsetUs = 0;
    unsigned long mapId = 0, initialKeyFrameId = 0, frameId = 0;
    Eigen::Matrix3d mapToWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d mapTranslation = Eigen::Vector3d::Zero();
    Eigen::Matrix3d initialImuToWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d initialImuTranslation = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rawCameraToWorld = Eigen::Matrix3d::Identity();
    Eigen::Vector3d rawCameraPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroIntegral = Eigen::Vector3d::Zero();
    double gyroSpanS = 0;
    uint64_t imuCount = 0;
};

class Bridge {
public:
    explicit Bridge(const Config& config);
    ~Bridge();
    Bridge(const Bridge&)=delete;
    Bridge& operator=(const Bridge&)=delete;
    // No ROS calls on the tracking/capture thread. The mailbox has capacity one.
    // Kept templated so the transport library need not include the SLAM/viewer headers.
    template<class Navigation> void Submit(const Navigation& n,uint64_t stamp,uint64_t clockEpoch=0,
                                          const SubmitMeta& meta=SubmitMeta{}) {
        Sample s;s.sampleUs=stamp;s.clockEpoch=clockEpoch;s.referenceEpoch=n.referenceEpoch;
        s.valid=n.valid;s.imuInitialized=n.inertialInitialized;s.ba2=n.inertialBA2;s.velocityValid=n.velocityValid;
        s.position=n.imuToWorld.translation().template cast<double>();
        s.imuToWorld=n.imuToWorld.rotationMatrix().template cast<double>();
        s.velocity=n.velocityWorld.template cast<double>();s.gravity=n.gravityWorld.template cast<double>();
        s.pairId=meta.pairId;s.srcSimUs=meta.srcSimUs;s.mapOffsetUs=meta.mapOffsetUs;
        s.mapId=meta.mapId;s.initialKeyFrameId=meta.initialKeyFrameId;s.frameId=meta.frameId;
        s.mapToWorld=meta.mapToWorld;s.mapTranslation=meta.mapTranslation;
        s.initialImuToWorld=meta.initialImuToWorld;s.initialImuTranslation=meta.initialImuTranslation;
        s.rawCameraToWorld=meta.rawCameraToWorld;s.rawCameraPosition=meta.rawCameraPosition;
        s.gyroIntegral=meta.gyroIntegral;s.gyroSpanS=meta.gyroSpanS;s.imuCount=meta.imuCount;
        Submit(std::move(s));
    }
    void InvalidateClock(uint64_t clockEpoch);
    std::string FaultReason() const;
    // Also used by the integration fixture without a camera/SLAM instance.
    void Submit(Sample sample);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}}
