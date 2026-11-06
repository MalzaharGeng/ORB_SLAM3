# DECXIN Camera 与 PX4 IMU 接入

运行平台为 Jetson Orin Nano，Ubuntu 22.04 / aarch64 / ROS 2 Humble。
每目图像 1280×960，ORB 配置 1000 特征点。相机输入帧率与 SLAM 处理帧率分别统计。

## 配置与数据路径

| 文件 | 用途 |
|---|---|
| Examples/Stereo-Inertial/DECXIN_Camera.yaml | 默认双目配置，包含当前相机和机体参数 |
| Examples/Stereo-Inertial/DECXIN_PX4_Bench.yaml | 惯性台架配置，关闭环形图像写盘 |
| Examples/Stereo-Inertial/DECXIN_rectify.yaml | 原始双目标定、裁剪布局与输入帧率 |
| config/decxin_px4_imu.yaml | 相机到机体变换、图像时间偏移及适用采集设置 |
| config/decxin_px4.yaml | 默认桥接接收配置，extrinsics_valid=0 |
| config/decxin_px4_bench.yaml | 台架接收配置，R_BI 为单位阵 |

默认入口 stereo_uvc 使用纯双目。设置 VIO_IMU_CONFIG 后可接收 PX4 SensorCombined 原始加速度和角速度；
运行时仍须通过标定匹配、惯性初始化、VIBA2、跟踪、时间和飞控状态门控。
当前默认服务不发布导航观测。

相机到机体的旋转与时间偏移经独立采集验证，平移采用尺量值。参数及误差见
[外参标定记录](DECXIN_EXTRINSICS_20261110.md)。

## 图像与时间

DECXIN Camera 输出 2720×960：前 160 列为填充，之后为左右各 1280 列。
左右图像按运行时 stereoRectify 的 R1/R2/P1/P2 进行校正。

C++ OpenCV 4.8.0 的校正参数：
fx=fy=694.006880918，cx=649.014984131，cy=444.565422058。
Python OpenCV 4.5.4 的 alpha=0 结果与之有差异，分析工具使用 C++ 导出的矩阵。

图像时间戳在 V4L2 读出后、解码和校正前生成，经过图像时间偏移补偿后与 IMU 对齐。
标定适用于 MJPG 2720×960 @30 fps、单 V4L2 缓冲和读出后主机时间戳。
改变输入格式、缓冲或帧率需重新验证时间偏移；配置不匹配时程序退出。

## IMU 接入

SensorCombined 已采用机体 FRD 坐标，因此 R_BI 为单位阵。
使用原始角速度与加速度，不使用 PX4 融合位姿代替惯性测量。

接收缓冲最多保留 2048 条采样，要求图像区间具备完整 IMU 覆盖，并对端点插值。
削顶、非有限值、过期采样、时钟回退、校准计数变化与间断都会触发拒绝或重新初始化。
ORB-SLAM3 会保留上一批次末点，后续批次不重复提交公共端点，避免零时间间隔。

DDS 实际送达约 100 Hz，传感器积分周期约 5 ms；台架配置采用 100 Hz。
已有记录中出现过约 1 秒接收缺口，长时连续性尚未通过。

## 稳定性与退出

控制台支持 status、pose、freeze_images 请求和正常退出。
守护进程按 stereo_uvc / stereo_inertial 实际进程名称执行监控、恢复及关闭，
同时识别图像处理统计心跳，直接读取实际会话目录。

缺少上一帧边缘化惯性先验时，跟踪器使用最后关键帧重建惯性约束，
避免直接构造依赖空先验的优化边。回归测试与录制回放覆盖该路径。

整段 PNG 录制与环形录制分别运行。整段写盘队列有界，溢出时报告不完整并保留数据；
已观测到写盘跟不上处理速度，不能将该录制标为完整回放输入。

## 已验证范围

- Nano 上构建、启动配置检查与标定拟合回归通过。
- 两段完整运动数据的旋转/时间估计相差 0.309 度、0.276 ms。
- 缺先验测试在修复前段错误，修复后通过；557 帧真实记录回放无崩溃。
- 标定后的实机 VIO 完成 VIBA1/VIBA2；1086 帧中未出现视觉跟踪丢失，
  后因 waiting_imu 退出，轨迹已保存。
- 既有 44 项行为树测试以及桥接、录制门控、IMU 缓冲与启动器检查通过。
- 未进行新硬件实飞；未验证长时间 IMU 连续性和实际 EKF 融合导航。

## USB 与帧率

相机协商为 5000M SuperSpeed，位于 USB3 Hub 下。

| 输入格式 | 请求帧率 | 实测驱动取帧 | 有效图像数据量 |
|---|---|---|---|
| MJPEG | 30 | 29.62 fps | 7.73 MB/s |
| YUYV | 30 | 30.41 fps | 158.79 MB/s |
| MJPEG | 60 | 60.03 fps | 17.73 MB/s |
| YUYV | 60 | 59.90 fps | 312.82 MB/s |

YUYV 30 fps 测试无序号缺口；60 fps 测试有 9 个缺口。MJPEG 两档均出现异常 JPEG 缓冲。
这些短时结果证明链路处于 USB3 模式，不能据此保证所有线材和运动条件下稳定。

SLAM 实际约 8～11 fps，受场景、算法与并发负载影响。相机输入设为 60 fps
未在常驻服务中改善处理速度，默认保持 30 fps；性能实验与已验证标定配置分开。

## 运行检查

先加载 ROS 2 Humble 和匹配的 px4_msgs 环境，在仓库根目录执行：

~~~bash
build/Examples/Stereo/stereo_uvc Vocabulary/ORBvoc.txt   Examples/Stereo-Inertial/DECXIN_PX4_Bench.yaml   Examples/Stereo-Inertial/DECXIN_rectify.yaml   --imu-config config/decxin_px4_imu.yaml --check-config
~~~

台架运行需停止占用相机的服务，保留一个 DDS Agent，飞控处于地面未解锁状态：

~~~bash
build/Examples/Stereo/stereo_uvc Vocabulary/ORBvoc.txt   Examples/Stereo-Inertial/DECXIN_PX4_Bench.yaml   Examples/Stereo-Inertial/DECXIN_rectify.yaml   --headless --console --imu-config config/decxin_px4_imu.yaml   --imu-bench --px4-config config/decxin_px4_bench.yaml
~~~

台架模式禁止导航观测发布。初始化需要有效平移激励；静态启动不代表能够完成 VIBA2。
