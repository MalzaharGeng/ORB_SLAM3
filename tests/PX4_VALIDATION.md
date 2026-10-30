# PX4 通信实现记录（2026-10-30）

> 以下为 Avi 源仓库的历史验证记录，非本分支本轮实测。本机检查与未验证项目见 [同步报告](../docs/AVI_SYNC_2026-10-30.md)。

实现入口与部署说明见 `tests/PX4_VIO.md`。

后续接口简化：桥接仅配置 `R_BI` 旋转，平移只由飞控配置；输出原点改为初始 IMU 中心。此调整完成编译检查，下面的回放和仿真数字保留为调整前的运行记录。

| 检查 | 结果与边界 |
|---|---|
| PX4 开关 ON | 实机入口、回放、桥接及测试程序编译通过；6/6 CTest 通过 |
| PX4 开关 OFF | 不需要 ROS 的构建通过；5/5 CTest 通过 |
| 录制恢复回放 | 1686 个有效导航快照；订阅端收到 183 条消息，时间戳不重复、数值有限，重置计数 0/1/2；stride=2 约 15 Hz |
| PX4 1.17 SIH + Agent 2.4.3 | 实际 EKF 位置、高度、速度融合、视觉速度输入响应、断流/延迟/参考重置/Agent 重启恢复通过；合成观测，不代表 VIO 定位精度 |
| PX4 ULog | 238 条视觉里程计记录，PX4 内采样时龄 10.014–12.227 ms；位置、高度各 227 次 fused=true；速度依据融合标志、EKF 速度响应及创新日志验证 |
| D435i 实机 | Global Time 采集成功、headless 命令退出正常；静止状态未完成惯性初始化，未发送定位 |
| 飞控原生 USB | 当前没有连接飞控，USB 启动扩展、拔插及真实安装外参尚未验收 |

记录位于 `recordings/test_runs/px4_implementation_20261030/`：`ctest.log`、`off_final_ctest.log`、`replay/summary.json`、`replay_peer.json`、`sitl_acceptance/summary.json`、`sitl_acceptance/ulog_audit.json`、`live/run.log`。Ulog 原件在 `sitl_acceptance/runtime/log/`。

PX4 仿真使用上游 1.17 提交 `f65ddce53985f13cd4fa24e7c0d78345b467bf1f`。该快照的 `ExternalVisionVel::fuseVelocity` 将 `aid_src.timestamp` 传给局部速度融合，导致速度 aid-source 的 `timestamp_sample` 保持 0，诊断发布条件不满足。因此不能把缺少 `estimator_aid_src_ev_vel` 日志误判为通信未接收；本实现未修改飞控融合代码。位置/高度日志及速度响应另行验证，原始证据保留在上述目录。

原工作区已有的初始化参数与配置修改已保留。`preexisting.patch` 和 `task_existing_files.diff` 分别记录本轮前的修改及本轮对已有文件的增量。
