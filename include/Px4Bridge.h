#pragma once
#include "Px4Vio.h"
#include <memory>

namespace ORB_SLAM3 { namespace px4 {
class Bridge {
public:
    explicit Bridge(const Config& config);
    ~Bridge();
    Bridge(const Bridge&)=delete;
    Bridge& operator=(const Bridge&)=delete;
    // No ROS calls on the tracking/capture thread. The mailbox has capacity one.
    // Kept templated so the transport library need not include the SLAM/viewer headers.
    template<class Navigation> void Submit(const Navigation& n,uint64_t stamp,uint64_t clockEpoch=0) {
        Sample s;s.sampleUs=stamp;s.clockEpoch=clockEpoch;s.referenceEpoch=n.referenceEpoch;
        s.valid=n.valid;s.imuInitialized=n.inertialInitialized;s.ba2=n.inertialBA2;s.velocityValid=n.velocityValid;
        s.position=n.imuToWorld.translation().template cast<double>();
        s.imuToWorld=n.imuToWorld.rotationMatrix().template cast<double>();
        s.velocity=n.velocityWorld.template cast<double>();s.gravity=n.gravityWorld.template cast<double>();
        Submit(std::move(s));
    }
    void InvalidateClock(uint64_t clockEpoch);
    // Also used by the integration fixture without a camera/SLAM instance.
    void Submit(Sample sample);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}}
