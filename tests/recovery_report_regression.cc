#include "RecoveryReport.h"
#include <iostream>
#include <stdexcept>

using namespace ORB_SLAM3;
static void require(bool condition, const char* message)
{
    if(!condition) throw std::runtime_error(message);
}

int main()
{
    const Sophus::SE3f firstFromSecond(Sophus::SO3f::exp(Eigen::Vector3f(0,0,1.57079632679f)),
                                       Eigen::Vector3f(1,2,3));
    const Sophus::SE3f secondFromCamera(Sophus::SO3f(),Eigen::Vector3f(2,0,0));
    RecoveryMatchMetrics good{150,60,60,0.02f};
    auto make=[&](RecoveryMatchMetrics metrics,float delta=0.02f,int count=2) {
        return MakeRecoveryReport(firstFromSecond,secondFromCamera,metrics,delta,0.01f,count,123.5);
    };
    const auto report=make(good);
    require(report.valid,"Strong geometry must produce a report");
    require((report.secondOriginInFirst-Eigen::Vector3f(1,2,3)).norm()<1e-5f,
            "Map origin offset uses the wrong coordinate frame");
    require((report.cameraInFirst-Eigen::Vector3f(1,4,3)).norm()<1e-5f,
            "Camera offset must compose rotation and translation, not use the map origin");
    require(std::abs(report.cameraDistance-std::sqrt(26.f))<1e-5f,
            "Camera distance is not relative to the first map origin");
    require(std::abs(report.originDistance-std::sqrt(14.f))<1e-5f,"Incorrect map-origin distance");
    require(report.confidence>90 && report.confidence<=100,"Strong geometry should score highly");
    const Sophus::SE3f cameraFromImu(Sophus::SO3f(),Eigen::Vector3f(0.02f,0,0));
    const auto bodyReport=MakeRecoveryReport(firstFromSecond,secondFromCamera,good,0.02f,0.01f,2,123.5,cameraFromImu);
    require((bodyReport.imuInFirst-Eigen::Vector3f(1,4.02f,3)).norm()<1e-5f,
            "Recovery omitted the camera-to-IMU mounting transform");
    require((bodyReport.imuInFirst-bodyReport.cameraInFirst).norm()>0.019f,
            "Camera and IMU centers must not be conflated");

    auto contaminated=good;contaminated.candidatePairs=120;
    require(make(contaminated).confidence<report.confidence,
            "Rejected pairs must lower confidence even when filtered RMSE stays identical");
    auto noisy=good;noisy.rmse=0.14f;
    require(make(noisy).confidence<report.confidence,"Larger residual must lower confidence");
    require(make(good,0.14f).confidence<report.confidence,"Less stable transforms must lower confidence");
    require(!make(good,0.02f,1).valid,"One geometric estimate is insufficient");
    require(!make(good,0.15f).valid,"Inconsistent transforms must not be accepted");
    auto invalid=good;invalid.pairs=19;
    require(!make(invalid).valid,"Insufficient 3D support must be rejected");
    invalid=good;invalid.candidatePairs=0;
    require(!make(invalid).valid,"Empty denominator must be rejected");
    invalid=good;invalid.rmse=std::numeric_limits<float>::quiet_NaN();
    require(!make(invalid).valid && make(invalid).confidence==0,"NaN must not produce confidence");
    require(!RecoveryReport().valid,"An unloaded map must not have a valid zero offset");
    std::cout<<"PASS: coordinate composition, quality degradation, unavailable/invalid reports\n";
}
