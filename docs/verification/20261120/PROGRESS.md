# 仿真阶段记录（2026-11-20）

## 当前仿真进展（2026-11-20）

原场地已接入 Drone-Harrier，完成双目/IMU → ORB-SLAM3 → PX4 EV → Position/Offboard → 5 m → 接回降落的联调。最新冻结候选执行东向 5 m 正式组：**T1、T2 通过，T3 失败，整组未通过**。T3 未进入 NAVIGATE，交接前位置保持未满足 `at_home` 条件。该组与 11-16 宽场地冷启动 3/3 使用不同条件，分开记录；历史中断专项保持 B / UNVERIFIED。

| 代码 / 配置 | 作用 |
|---|---|
| `include/System.h`、`src/System.cc` | 一致导航快照、参考身份与首次有效参考 |
| `include/MapLossPolicy.h`、`src/Tracking.cc`、`src/LocalMapping.cc` | 有界 VIBA2 初始化地图保留及优化提交的地图身份检查 |
| `include/Px4Vio.h`、`src/Px4Vio.cc` | 源时间、跟踪与有效性检查，发布参考及故障锁存 |
| `src/Px4Bridge.cc`、`include/Px4GateSnapshot.h` | ROS/PX4 桥接与可选比较快照 |
| `config/RM27_Harrier_IMU_Stereo_baseline.yaml` | 当前机体的仿真相机/IMU 几何、噪声及初始化参数 |
| `tests/` | 参数、地图策略、参考变换、时钟与桥接回归 |

本次补齐 Harrier 配置与当前联调说明，核心算法沿用已有 master 实现。ROS 输入节点、默认关闭的短时失锁诊断、飞手、行为树、场景和六次运行证据由 [RM27_drone_sim](https://github.com/MalzaharGeng/RM27_drone_sim) 维护。算法侧的有效性与发布契约见 [Harrier 联调说明](../../../docs/HARRIER_SIM_20261120.md)，历史变更见 [NP-01 更新记录](../../../docs/NP01_SIM_STATUS_20261116.md)。

东向任务的通过不能代表原北向任务或长期可靠率；`transport` 的时间补偿限仿真使用。Nano 实飞、碰撞回正、空中跨地图导航恢复及完整延迟性能仍未验收。
