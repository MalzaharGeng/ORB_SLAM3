# DECXIN YUYV 图像链路与验证 — 2026-11-13

当前配置使用 **USB 3 / YUYV 2720×960 @30 fps → 亮度提取 → 双目裁剪与校正 → 每目 960×720 → ORB-SLAM3 + PX4 原始 IMU**。相机缓冲为 1，额外取流流水线关闭，图像时间戳仍在读出后产生。时间偏移更新为 **−52.280271 ms**，相机/飞控空间外参保持原验证值。

## 硬件与格式选择

实机为 `1bcf:2d52` DECXIN / Sunplus UVC 模组，协商 USB 3 5000 Mbit/s。`/dev/video0` 为视频，`/dev/video1` 为 UVC 元数据；CSI 媒体拓扑没有已接入相机，USB 描述符也未暴露标准 IMU 接口。当前继续使用 PX4 `SensorCombined`。

用户提供的 SC133HGS / LSM6DSV 型号不能仅凭 USB 描述符独立确认。MIPI 是接口，MJPG 是压缩格式；传感器芯片接口不等于成品模组的对外接口。切 MIPI 还需要模组引脚、接线、双目同步和 Jetson 驱动支持。

设备声明 MJPG 30/60/120 fps、YUYV 30/60 fps。本次只使用 30 fps，以避免增加 IMU 对齐和处理负载。YUYV 的字节亮度直接送入单通道校正，不经 JPEG 解码或 BGR 转换。保留显式 MJPG 回退。

## 实测

先独占相机，直接 V4L2、4 缓冲，各运行约 15 秒。这些是采集指标：

| 指标 | MJPG | YUYV |
|---|---:|---:|
| 采集帧率 | 28.82 fps | 30.42 fps |
| 载荷 | 7.51 MB/s | 158.85 MB/s |
| JPEG 头异常 | 37 | 不适用 |
| 驱动序号缺口 | 160 | 0 |
| 驱动错误标志 | 0 | 0 |

随后用同一个新二进制、每目 960×720、1000 特征、单缓冲、GPU 校正与 FAST、相同 CPU 层级并行设置做静态双目对照。每种格式进程预算 45 秒，包含词典加载及退出；排除首个性能窗口：

| 指标 | MJPG | YUYV |
|---|---:|---:|
| 跟踪帧率 | 14.46 fps | 14.56 fps |
| 解码 / 亮度提取 | 21.20 ms/帧 | 2.62 ms/帧 |
| V4L2 读取等待 | 41.55 ms/帧 | 57.21 ms/帧 |
| SLAM 跟踪计算 | 67.46 ms/帧 | 68.46 ms/帧 |
| 处理帧数 | 468 | 478 |
| 正常跟踪状态比例 | 100% | 100% |
| 报告的坏帧 | 1 | 0 |
| JPEG 警告 | 4 | 0 |
| 退出码 | 0 | 0 |

YUYV 转换成本下降约 87.6%，并避免此次 JPEG 异常；帧率基本相同，因为主要限制仍是 SLAM 计算。读取耗时含等待下一帧，不能直接把解码节省量当作端到端延迟收益。当前 USB 带宽可以承载这次载荷，未观察到跟踪性能受损。短时测试不能证明长期零坏帧，也不是带真值的轨迹精度或飞行验收。

## 时间偏移验证

原 MJPG 标定为 −44.534 ms，不能沿用到 YUYV。初始完整空间/时间重估的部分候选因三轴激励不足或角误差略超过原阈值而未通过；未放宽阈值，也未把失败的空间候选投入运行。

本次只改变像素格式，没有改变刚性安装，因此使用已独立验证的原 `T_b_c1`，只估计新时间偏移和用于残差评估的陀螺偏置。空间外参不再作为本轮自由变量。换装或改变安装时，仍必须重新走完整空间/时间标定，不能使用这个流程。

三段完整且时间不重叠的运动数据分别独立估计出 −52.280、−52.026、−51.144 ms，最大差 1.137 ms。第一段为主拟合集，第二、第三段均完整用于独立验证，没有按结果截取区间。六组双向检查的最大移动角速度 RMSE 为 0.01475 rad/s，最大角误差 95 分位数为 0.29901°，保留原有 0.03 rad/s、0.3°、10 ms 阈值。第一段对两个独立验证集的角误差 95 分位数为 0.22759°、0.12675°。

拟合排除跨越真实 IMU 缺口的视觉区间，不补造 IMU。另有恒定转速可观测性检查，避免低误差但无法确定时间偏移的伪通过。合成测试覆盖已知延迟、偏置、独立信号不一致、恒定转速、IMU 缺口和非法空间变换。

证据见 [temporal_validation.json](../nano/verification/20261113/temporal_validation.json)。`flight_ready=false` 表示时间验证不授予飞行就绪；运行仍需正常 VIBA2、时间同步和发布门控。

复现命令（原始数据保留在 Nano，未上传 GitHub）：

```bash
python3 tools/decxin_temporal_calibration.py \
  /home/nano/camera_link_20261113/yuyv_motion1_active \
  /home/nano/camera_link_20261113/yuyv_motion2_active \
  /home/nano/camera_link_20261113/yuyv_motion3_active \
  --mount-config config/decxin_px4_imu_mjpg.yaml --mount-unchanged \
  --out /tmp/yuyv_temporal_validation.json
python3 tests/decxin_temporal_calibration_regression.py
```

需要 Python OpenCV、NumPy、SciPy。视觉区间由原 `decxin_calibration_fit.py` 的独立双目 PnP 流程提取。

## 当前配置与回退

`stereo_uvc` 默认 YUYV，也可设置 `VIO_CAPTURE_FORMAT=MJPG|YUYV`。实际协商格式、尺寸、帧率不匹配时拒绝启动；惯性模式还要求采集格式匹配 IMU 配置。默认服务显式设置：

```ini
Environment=VIO_CAPTURE_FORMAT=YUYV
Environment=VIO_CAPTURE_BUFFERS=1
Environment=ORB_SLAM3_CAPTURE_PIPELINE=0
Environment=VIO_IMU_CONFIG=/home/nano/ORB_SLAM3/config/decxin_px4_imu.yaml
```

回退必须成套使用 MJPG 和旧时间标定，空间变换不变：

```ini
[Service]
Environment=VIO_CAPTURE_FORMAT=MJPG
Environment=VIO_CAPTURE_BUFFERS=1
Environment=ORB_SLAM3_CAPTURE_PIPELINE=0
Environment=VIO_IMU_CONFIG=/home/nano/ORB_SLAM3/config/decxin_px4_imu_mjpg.yaml
```

上述回退可写入 `sudo systemctl edit px4-vio`；只有确认整机未解锁、着地后才重启定位服务。无飞控参数、解锁、模式或任务命令属于本次相机部署。

设备本次时钟为 2026-11-06，报告使用用户端日期 2026-11-13。原始运行文件名保留设备时间，未为整理报告而修改系统时钟。
