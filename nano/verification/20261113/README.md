# DECXIN YUYV 部署与验证 — 2026-11-13

Jetson Orin Nano / JetPack 6.2，CPU 实际上限 1.344 GHz，每目 960×720、1000 ORB 特征。

- 新相机入口在 Nano 原生编译通过，默认 YUYV，保留显式 MJPG 回退。
- 三项相机/队列/启动器回归通过；五项新时间标定合成测试通过。
- YUYV 主配置与 MJPG 成套回退的配置检查通过，格式/标定混用返回错误码 2。
- 双目对照各使用 45 秒进程预算、单缓冲、相同 GPU/CPU 后端；处理 468 / 478 帧，全为正常跟踪状态，均正常退出。
- YUYV 灰度开销 2.62 ms/帧，MJPG 21.20 ms/帧；跟踪帧率 14.56 / 14.46 fps。
- 时间验证保留原已验证的相机/飞控刚性变换，仅重新估计格式相关时间偏移。三段独立、不重叠的运动采集通过六组双向检查；采用第一段拟合的 -52.280271 ms。
- 原完整空间/时间重估部分候选没有通过；本次不采用失败的空间候选，也不放宽原误差阈值。更换安装时不能使用当前时间单独验证流程。
- 已部署 `VIO_CAPTURE_FORMAT=YUYV`、单缓冲、关闭额外采集流水线；保留原标定 YAML 和服务备份。
- 在线已观测图像坏帧 0、约 15.5 fps、IMU 约 100 Hz、`imu=1, ba2=0, valid=0`。静止运行未完成 VIBA2；此记录不证明飞行就绪或长期导航连续性。
- 没有发送解锁、模式或飞控参数命令。

| 文件 | 内容 |
|---|---|
| [temporal_validation.json](temporal_validation.json) | 固定原安装变换的独立时间验证、来源哈希及原验收阈值 |
| [slam_comparison.json](slam_comparison.json) | 两种格式静态双目对照的结构化结果 |
| [regressions.log](regressions.log) | 相关回归及配置/混用拒绝检查 |
| [live_status.log](live_status.log) | YUYV 服务环境及在线状态采样 |

原始图像、IMU CSV、构建和二进制留在 Nano `/home/nano/camera_link_20261113/`，不随代码分支上传。设备时钟为 2026-11-06，本目录使用用户端日期 2026-11-13。

配置、复现命令和 MJPG 回退见 [链路说明](../../../docs/DECXIN_YUYV_20261113.md)。
