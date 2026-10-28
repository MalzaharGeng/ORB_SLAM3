#ifndef ORB_SLAM3_RECOVERY_REPORT_H
#define ORB_SLAM3_RECOVERY_REPORT_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

struct RecoveryMatchMetrics
{
    int inliers = 0;
    int pairs = 0;
    int candidatePairs = 0; // Before the 0.25 m residual filter.
    float rmse = std::numeric_limits<float>::infinity();
};

// Snapshot at geometric alignment, not a live tracking probability.
// Offsets use the FIRST saved map's INITIAL IMU axes and origin, in meters.
struct RecoveryReport
{
    bool valid = false;
    double timestamp = 0;
    Eigen::Vector3f secondOriginInFirst = Eigen::Vector3f::Zero();
    Eigen::Vector3f cameraInFirst = Eigen::Vector3f::Zero();
    Eigen::Vector3f imuInFirst = Eigen::Vector3f::Zero();
    float originDistance = 0;
    float cameraDistance = 0;
    float imuDistance = 0;
    float confidence = 0; // Heuristic quality score, 0..100; not a probability.
    float pairRatio = 0;
    float translationDisagreement = 0;
    float rotationDisagreement = 0; // Radians.
    int confirmations = 0;
    RecoveryMatchMetrics match;
};

inline RecoveryReport MakeRecoveryReport(const Sophus::SE3f& firstFromSecond,
                                         const Sophus::SE3f& secondFromCamera,
                                         const RecoveryMatchMetrics& metrics,
                                         float translationDelta, float rotationDelta,
                                         int confirmations, double timestamp,
                                         const Sophus::SE3f& cameraFromImu = Sophus::SE3f())
{
    RecoveryReport report;
    report.match = metrics;
    report.timestamp = timestamp;
    report.translationDisagreement = translationDelta;
    report.rotationDisagreement = rotationDelta;
    report.confirmations = confirmations;
    // Unknown/invalid geometry must not look like a high-confidence zero offset.
    if(confirmations < 2 || metrics.inliers < 50 || metrics.pairs < 20 ||
       metrics.candidatePairs < metrics.pairs || metrics.candidatePairs > metrics.inliers ||
       !std::isfinite(metrics.rmse) || metrics.rmse < 0 || metrics.rmse >= 0.15f ||
       !std::isfinite(translationDelta) || translationDelta < 0 || translationDelta >= 0.15f ||
       !std::isfinite(rotationDelta) || rotationDelta < 0 || rotationDelta >= 0.10f ||
       !std::isfinite(timestamp) || !firstFromSecond.matrix().allFinite() ||
       !secondFromCamera.matrix().allFinite() || !cameraFromImu.matrix().allFinite())
        return report;

    report.secondOriginInFirst = firstFromSecond.translation();
    report.cameraInFirst = (firstFromSecond * secondFromCamera).translation();
    report.imuInFirst = (firstFromSecond * secondFromCamera * cameraFromImu).translation();
    report.originDistance = report.secondOriginInFirst.norm();
    report.cameraDistance = report.cameraInFirst.norm();
    report.imuDistance = report.imuInFirst.norm();
    report.pairRatio = float(metrics.pairs) / metrics.candidatePairs;

    const float inlierSupport = std::min(metrics.inliers / 150.f, 1.f);
    const float pairSupport = std::min(metrics.pairs / 60.f, 1.f);
    const float residualQuality = 1.f - std::pow(metrics.rmse / 0.15f, 2);
    const float stability = std::exp(-0.5f * (std::pow(translationDelta / 0.15f, 2) +
                                             std::pow(rotationDelta / 0.10f, 2)));
    // Explicit engineering weights; no claim of statistical calibration.
    report.confidence = std::clamp(100.f * (0.20f * inlierSupport + 0.15f * pairSupport +
        0.25f * report.pairRatio + 0.25f * residualQuality + 0.15f * stability), 0.f, 100.f);
    report.valid = true;
    return report;
}

} // namespace ORB_SLAM3
#endif
