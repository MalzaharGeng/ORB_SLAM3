# 特征前端性能记录

## 测试条件

Jetson Orin Nano，aarch64，ROS 2 Humble，C++ OpenCV 4.8.0。
每目 1280×960，ORB 配置 1000 特征，8 层金字塔，FAST 阈值 20 / 7。
CPU 上限 1.344 GHz，未修改功耗模式。输入采用已保存的相机图像。

相机输入 30/60 fps 与 SLAM 实际处理帧率分别报告，关闭录制的剖析不能代替常驻服务验收。

## 双目匹配优化

行候选表改为稳定次序的连续索引，11×11 灰度块的 L1 距离使用精确整数计算；
aarch64 利用 NEON，其他平台保留标量路径。原有阈值、亚像素拟合和特征参数保持。

| 同源数据 18 组，中位耗时 | 优化前 | 优化后 |
|---|---:|---:|
| 左右目特征提取 | 71.35 ms | 70.96 ms |
| 金字塔构建 | 9.30 ms | 10.22 ms |
| 角点、分布与方向 | 46.51 ms | 44.30 ms |
| 双目匹配 | 5.86 ms | 2.25 ms |
| Frame 前端 | 80.79 ms | 76.38 ms |

单独阶段测试会受缓存、调度与负载影响，不能把各阶段中位数直接相加。
特征提取代码未在本阶段修改，其波动不计为匹配优化收益。

## 正确性检查

- 3000 个随机、带步长图像块：整数 L1 与 OpenCV NORM_L1 相同。
- 16 组合成双目：深度和右目亚像素坐标逐字节一致，包含 2133 个有效匹配。
- 20 组真实相机图像：深度和右目亚像素坐标逐字节一致。
- 左目有特征而右目无匹配时，全部深度为无效值，不访问空中位数数组。
- CTest 的无特征帧、双目匹配、IMU 缓冲 3 项通过。

## 验证边界

本阶段未达到实际处理 30 fps。未进行实飞、长时 IMU 连续性或实际 EKF 融合导航验收。
原始图像、构建产物和本机性能日志保留在设备，不进入代码分支。
GPU 和并行特征计算须分别通过完整输出对照及整段回放后再决定是否启用。

## Optional CUDA FAST backend

Jetson Orin Nano has one CUDA device (CUDA 12.6). An optional ORB_SLAM3_WITH_CUDA_FAST=ON
backend computes the FAST-9 response on the GPU and reuses device allocations across frames.
ORB_SLAM3_FAST_BACKEND=cuda selects it; an unset variable keeps the CPU path. Initialization or
runtime failure falls back to CPU.

Equivalence coverage includes four synthetic image classes and 20 recorded image samples.
Keypoint count, coordinate, order, response, orientation and descriptor bytes match the CPU path.
CUDA single-image extraction median was 47.25 ms versus 58.52 ms for the CPU comparison.
With both eyes and the existing pipeline, the measured front end was about 69.5 ms, so the target
of 30 processed fps remains unmet. This section describes the earlier standalone benchmark;
the current Nano service uses CUDA correction and FAST. Long-duration thermal and VIO
trajectory validation remains open.

## 2026-11-13：成熟 VIO 描述子并行对照

保持每目 960×720、1000 配置特征、8 层、scale 1.2、FAST 20/7，以及原标定、
IMU 连续性检查和 VIBA 门控。GPU 执行校正与 FAST，CPU 将 GaussianBlur 和描述子
按金字塔层并行计算，再按原顺序拼装关键点与描述子；没有减少图像或特征。

生产 ORBextractor 的 252 次图像对照覆盖真实、合成、低纹理、非连续 ROI、缓冲复用、
双目并发，以及 CPU/CUDA 和 1/3/6 线程。关键点字段、顺序、描述子字节全部一致。

手动初始化完成后，在同一进程、同一成熟地图中按串行/并行/并行/串行测量，
每段预留 5 秒后采样 35 秒，共 1642 帧、140.34 秒。CPU 上限 1.344 GHz、25W、
schedutil；状态内录开启、图像录制关闭，没有同时运行离线基准。

| 指标 | 串行 | 分层并行 |
|---|---:|---:|
| 实际处理帧率（总帧数/总时间） | 11.55 fps | 11.85 fps |
| SLAM 平均耗时（日志区间均值） | 85.21 ms | 83.68 ms |
| 各段帧率 | 11.46 / 11.65 | 11.87 / 11.82 |

本轮帧率提高约 2.5%，每帧节约约 1.54 ms，小于离线前端的 5–7 ms。
所有采样帧均 tracking=2、valid=imu=ba2=1，无 epoch 变化、报告坏帧或观察到的 IMU 停止。
这是单一静态环境短测，不能证明轨迹精度不变或长期稳定性；没有实飞或地面真值对照。

代码默认串行，Nano 服务配置显式启用 `ORB_SLAM3_PARALLEL_DESCRIPTORS=1`；可选
`ORB_SLAM3_DESCRIPTOR_CONTROL_FILE` 支持同进程对照，文件只有单个 `1` 时启用，
`0`、丢失或格式错误均回退串行。Nano 本次已选并行，隔离构建保留于
`/home/nano/camera_link_20261113/optimized_descriptors`，服务 drop-in 为
`65-descriptor-optimization.conf`，控制文件为
`/home/nano/camera_link_20261113/descriptor_mode.txt`。写入 `0` 可不重启恢复串行。
原 `build` 库和程序保留；移除该 drop-in 并重启服务才会恢复原程序，随后需重新初始化。

后续优先剖析角点候选扫描、GPU 分数图回传及临时内存分配。完整定位的收益必须重测，
不以 GPU 使用率或未完成 VIBA2 的帧率代替成熟 VIO 帧率。
