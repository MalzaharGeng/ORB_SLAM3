// [NP-01 FIX5 / FIX5b] Bounded retention of the VIBA2 inertial-initialisation window.
//
// Violated contract (FIX5 original finding): LocalMapping owns the cumulative
// mTinit window and the in-flight InitializeIMU() result for the CURRENT Map.
// Tracking.cc used the condition (!isImuInitialized() || !GetIniertialBA2()) to
// ResetActiveMap() after a single TrackLocalMap miss. During the
// VIBA1-done / VIBA2-pending interval that condition is ALWAYS true, so one
// ordinary tracking miss destroyed the map, zeroed mTinit and discarded the
// VIBA2 result, restarting the 10 s window forever (NP-01 1 m FAIL: 33 resets,
// 38 new maps, VIBA2 completed twice but both were invalidated before any EV
// observation was published).
//
// FIX5b corrections to the FIX5 implementation (reviewed 2026-11-15):
//   * The window is now BOUND TO A MAP IDENTITY. The FIX5 version timed a bare
//     `mViba2ProtectSince` double, so a window opened on map A could be applied
//     to a different map B that never earned it.
//   * The elapsed time is supplied by the CALLER from a MONOTONIC HOST CLOCK.
//     The FIX5 version used `mCurrentFrame.mTimeStamp` (sensor time), which is
//     the one clock the NP-01 SensorClock analysis identifies as able to
//     regress / restart; a regression silently restarted the window, so the
//     bound was not enforceable in the clock domain it was written in.
//   * Configuration is validated BEFORE any window state is examined. The FIX5
//     version returned "retain" for `protectedSince < 0` even when the policy
//     was disabled, i.e. the function contract and its tests disagreed with the
//     production call order. A disabled policy now cannot return Retain for any
//     input combination.
//   * There is no "restart" outcome. With a monotonic clock a non-finite or
//     negative elapsed value means the bound cannot be enforced, so the policy
//     fails CLOSED to native ORB-SLAM3 behaviour instead of opening a new
//     window (a restart is what made the FIX5 bound extendable).
//
// This header is dependency-free so the decision can be unit tested without
// Atlas, OpenCV or ROS. It ONLY answers "may this map be kept a bit longer".
// It never marks a pose valid, never changes a Gate / LIMIT / age / acceptance
// threshold, never enables never_reset_map, and never lowers any initialisation
// requirement: VIBA2 must still succeed in InitializeIMU() before the map
// reports ba2=true and the Gate can publish.
#pragma once
#include <cmath>

namespace ORB_SLAM3 {

enum class Viba2WindowDecision {
    // Native ORB-SLAM3 behaviour. The caller MUST clear its window state.
    Drop,
    // First eligible loss for THIS map: open the bounded window, then retain.
    StartWindow,
    // Inside the bounded window for THIS map: retain.
    Retain
};

// imuStereo        : sensor is IMU_STEREO (the only mode this policy touches)
// imuInitialized   : map->isImuInitialized()   (VIBA1 done)
// inertialBA2      : map->GetIniertialBA2()    (VIBA2 done)
// windowSeconds    : Tracking.viba2_recovery_seconds; <=0 or non-finite = disabled
// windowActive     : the caller currently has a window open
// sameMap          : the open window belongs to the map being evaluated
// elapsedSeconds   : monotonic host seconds since the window opened;
//                    ignored unless windowActive && sameMap
inline Viba2WindowDecision DecideViba2Window(
    bool imuStereo, bool imuInitialized, bool inertialBA2,
    double windowSeconds,
    bool windowActive, bool sameMap, double elapsedSeconds)
{
    // 1. Applicability: only the IMU_STEREO window between VIBA1 and VIBA2.
    if (!imuStereo || !imuInitialized || inertialBA2) return Viba2WindowDecision::Drop;

    // 2. Configuration first, so a disabled policy can never retain anything.
    if (!std::isfinite(windowSeconds) || windowSeconds <= 0.0)
        return Viba2WindowDecision::Drop;

    // 3. The window belongs to one map. A different map never inherits it.
    if (!windowActive || !sameMap) return Viba2WindowDecision::StartWindow;

    // 4. An unenforceable bound fails closed, it never opens a new window.
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0)
        return Viba2WindowDecision::Drop;

    // 5. Bounded: after windowSeconds the native reset / new-map path resumes.
    if (elapsedSeconds < windowSeconds) return Viba2WindowDecision::Retain;
    return Viba2WindowDecision::Drop;
}

inline const char* Viba2WindowDecisionName(Viba2WindowDecision d)
{
    switch (d) {
    case Viba2WindowDecision::Drop: return "drop";
    case Viba2WindowDecision::StartWindow: return "start_window";
    case Viba2WindowDecision::Retain: return "retain";
    }
    return "unknown";
}

} // namespace ORB_SLAM3
