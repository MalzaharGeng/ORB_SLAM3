#pragma once
#include "Px4ImuBuffer.h"
#include <memory>
namespace ORB_SLAM3 { namespace px4 {
// Raw body-FRD IMU; never uses fused PX4 attitude/velocity as an inertial measurement.
class ImuReceiver {
public:
    ImuReceiver(int domain,const std::string& prefix);
    ~ImuReceiver();
    bool Interval(double start,double end,std::vector<ImuReading>& out,uint64_t& epoch,std::string& reason);
    std::string Status() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}}
