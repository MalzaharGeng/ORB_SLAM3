# 守护脚本与自愈

px4-vio.service 运行 ~/vio_daemon.sh，负责 DDS Agent、状态内录和 headless SLAM。
pilot-navigation.service 独立运行导航验证与飞手授权逻辑。
两项服务均需 enable 才能在下次开机启动；enable 不等同于完成一次重启验收。

## 启动顺序

1. 等待飞控串口及相机。DECXIN UVC 使用两个 video 节点，无需 D435i 的 hidraw。
2. 启动 MicroXRCE-DDS Agent，检查串口握手，失败后重试。
3. 加载 ROS 环境，启动连续状态内录，等待 INREC_READY。
4. 通过 run_recovery_test.sh --headless --px4-config "$VIO_PX4_CONFIG" 启动 SLAM。
5. 将本会话日志链接到内录；控制台 stdin 由 FIFO 保持打开。

连续内录跨 SLAM 会话运行。每两秒检查内录进程，退出后新建 part 分段；
恢复失败时结束当前 SLAM。正常停止先终止采集、排空内录，再退出 Agent。
详细数据范围与完整性判定见 [录制说明](recording-and-maps.md)。

当前服务设置 VIO_IMU_CONFIG=config/decxin_px4_imu.yaml 和 VIO_PX4_CONFIG=config/decxin_px4_flight.yaml，启用受门控的首次空中初始化。VIO 新进程启动前从完整内录清单核对新鲜的未解锁、着地状态；空中退出后不会自动重建参考系。见 [五米任务流程](../../docs/DECXIN_FIVE_METRE_FLIGHT.md)。

## 判活

| 证据 | 阈值 | 动作 |
|---|---|---|
| UVC [STAT] 或 D435i [VIO_PERF] 无增长 | 45 s | 标记采集停滞并结束当前 SLAM |
| 日志行数和进程 CPU 时间同时不变 | STALL_LIMIT，默认 90 s | 结束会话并重试 |
| 内录进程退出 | 检查周期 2 s | 保留旧分段，新开分段；失败则停止 SLAM |

从 SLAM 命令行读取实际会话目录，避免主机时钟回退后监控到旧日志。
Linux 进程名按入口匹配，UVC 使用 stereo_uvc，D435i 使用截断后的 stereo_inertial。
DECXIN 的 USB 复位不会冒用仅匹配 D435i 的恢复路径。

## 配置

| 变量 | 默认值 | 用途 |
|---|---|---|
| RESTART_DELAY | 10 s | SLAM 退出后等待 |
| STALL_LIMIT | 90 s | 日志和 CPU 同时停滞阈值 |
| CAM_SETTLE | 5 s | 相机枚举后的等待 |
| BAUD | 921600 | 飞控串口 |
| VIO_PROCESSING_PROFILE | full（服务显式使用 light） | 选择 full/light 处理配置 |
| ORB_SLAM3_RECTIFY_BACKEND | 服务为 cuda | GPU 双目校正 |
| ORB_SLAM3_FAST_BACKEND | 服务为 cuda | GPU FAST，其他 ORB 步骤仍由 CPU 执行 |
| ORB_SLAM3_CAPTURE_PIPELINE | 服务为 1 | 纯双目独立取流，惯性入口强制关闭 |
| VIO_CAPTURE_BUFFERS | 服务为 2 | 纯双目双缓冲，惯性入口强制为 1 |
| VIO_IMAGE_RECORDING | 0 | 日常服务关闭环形图像内录 |
| VIO_IMU_CONFIG | 未设置 | 显式启用标定后的 PX4 原始 IMU 入口 |

~~~bash
systemctl is-enabled px4-vio pilot-navigation
systemctl status px4-vio pilot-navigation
tail -f ~/vio_daemon.log
~~~

vio-clock-prepare 只在服务启动阶段恢复持久化时钟边界。
运行中不要跳变主机时间；消息时间、单调时间和时间同步状态均应保留用于诊断。
