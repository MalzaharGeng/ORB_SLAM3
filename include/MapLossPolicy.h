// VIBA2 初始化期间的限时地图保留策略（FIX5b）。
// VIBA1 完成而 VIBA2 未完成时，单次跟踪失败曾触发地图重置，清空 mTinit。
// NP-01 1 m 失败轮记录到 33 次重置、38 张新地图；两次 VIBA2 结果均在 EV 发布前失效。
// 保留窗口绑定地图身份，按主机单调钟计时，避免传感器时钟回退重启 10 s 窗口。
// 先检查配置；禁用、非有限或负耗时均退回原有重置路径，不重新开窗。
// 本函数只决定是否暂时保留地图，不设置位姿有效性，不改变 Gate/LIMIT/age 门限，
// 不启用 never_reset_map，也不降低初始化要求；发布仍需 InitializeIMU 完成 VIBA2。
// 无 Atlas、OpenCV 或 ROS 依赖，可单独测试。
#pragma once
#include <cmath>

namespace ORB_SLAM3 {

enum class Viba2WindowDecision {
    // 恢复原有处理，调用方清除窗口状态。
    Drop,
    // 当前地图首次满足保留条件，建立限时窗口。
    StartWindow,
    // 当前地图仍在保留窗口内。
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
    // 仅处理 IMU_STEREO 的 VIBA1 至 VIBA2 阶段。
    if (!imuStereo || !imuInitialized || inertialBA2) return Viba2WindowDecision::Drop;

    // 禁用配置不能保留地图。
    if (!std::isfinite(windowSeconds) || windowSeconds <= 0.0)
        return Viba2WindowDecision::Drop;

    // 新地图不继承旧地图的窗口。
    if (!windowActive || !sameMap) return Viba2WindowDecision::StartWindow;

    // 无法确认耗时时，结束保留，不重新开窗。
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0)
        return Viba2WindowDecision::Drop;

    // 窗口超时后恢复重置或建图。
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
