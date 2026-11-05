# 守护脚本与自愈

`vio_daemon.sh` 由 `px4-vio.service` 拉起，负责把 DDS Agent、同步内录和 SLAM 按顺序
带起来，并在运行期看着它们。路径约定：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `RESTART_DELAY` | 10 s | SLAM 退出后等多久再进下一轮 |
| `STALL_LIMIT` | 90 s | 日志行数与 CPU 时间同时不变多久判死 |
| `CAM_SETTLE` | 5 s | 相机节点齐后的稳定窗口 |
| `BAUD` | 921600 | 串口波特率 |

## 启动顺序

1. **终止残留**。上一轮 SLAM 还在退出时 usbfs 接口仍被它占着，video 只剩 2、hidraw
   消失——这是「占用态」不是「故障态」，必须先 `pkill -9` 等接口释放再判断，否则会
   误判成故障去做 USB 复位，把正常工作的相机打断（实测踩过两次）。
2. **等相机**。`video ≥ 6` 且 `hidraw ≥ 1` 算齐；齐了再等 `CAM_SETTLE`，因为节点出现
   不等于 UVC 控制通道就绪，刚枚举完立刻开流会 `get_xu(EAGAIN)` 直接退出。等 12 轮
   （约 24 秒）没齐且确认无 SLAM 占用，做 USB 复位，最多 2 次；再不行就照常启动，
   让上层重试。
3. **起 Agent**。上电初期飞控自身还没启动完，串口没有数据，握手会静默失败，而 Agent
   自己不重试——所以握手失败就整只重启，最多 3 次，以日志出现
   `session established` 为准。失败不阻塞启动循环，回来重试。
4. **起内录**。`vio_inrec.py` 订阅 PX4 话题并解析 SLAM 日志，写 `~/inrec/`。
5. **起 SLAM**。`./run_recovery_test.sh --headless --px4-config config/px4_vio_local.yaml`（两者都在 master：[启动器](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/run_recovery_test.sh)、[桥接配置](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/config/px4_vio_local.yaml)），
   stdin 由一个 FIFO 顶住：daemon 自己持着 FIFO 写端，读端永不 EOF，控制台命令
   （`first`、`reset`、`status`…）随时可以 `printf > ~/.vio_stdin.fifo` 送进去。

退出走 `trap`：先停采集、收尾内录、再退 Agent。旧版本只删 pidfile，子进程还在跑，
`systemctl stop` 会等到超时。

## 判活

`watch_stall` 每 15 秒看一次，两路证据：

| 证据 | 阈值 | 动作 |
|---|---|---|
| `[VIO_PERF]` 计数不增长 | 45 s | 冻结最后有效输入 → 标记 `.vio_camera_recover` → 重启采集 |
| 日志行数 + 进程 CPU 时间同时不变 | 90 s | 判死，结束本次会话，systemd 20 s 后重启 |

第二路必须存在：等 IMU 初始化时日志是节流打印的，几十秒不增长是正常的，只数行数会
把「正在正常等待」误判成卡死。CPU 时间第二路也拦得住「还在写日志但已不算图」的
假活。

## 相机恢复

`vio-camera-prepare`（sbin/，root 所有）只处理 VID:PID `8086:0b3a`，unbind/bind 让
内核重新枚举，不重载驱动。两个保护：

- 目标设备正被 VIO 使用时返回 `skipped_running_vio`。复位会把正常工作的相机打断，
  形成「复位 → SLAM 崩 → 再复位」循环，必须挡住。
- 早期 `usbfix.sh` 在普通账户下写 sysfs 被拒，所以改成 root 所有的独立程序，由
  systemd 的 `ExecStartPre`（root 身份）和 sudoers 单条授权两条路调用。sudoers 只
  放行这一条无参数命令。

上一轮会话留下 `.vio_camera_recover` 时，守护脚本在等相机前先做一次定向恢复。

## 诊断信号

| 行 | 含义 |
|---|---|
| `[VIO_PERF]` | 每帧一行：跟踪耗时、特征数、地图号、ring_dropped。图像真的处理过才有它 |
| `[D435i_CAPTURE]` | 等不到图像时打印 SDK 各流回调数、时戳域和时龄，区分「传感器停流」和「时戳异常」 |
| `[PX4]` | 桥接状态行：state、hz、age_ms、sent、rejected |
| `[MapPolicy] explicit reset` | 控制台 reset 命令确实生效的标志 |

判读顺序：先看 `[VIO_PERF]` 有没有在走，再看 `[PX4]`；`[D435i_CAPTURE]` 只在怀疑
采集时看。

## 故障对照

| 现象 | 先查 |
|---|---|
| 服务反复重启 | `~/vio_service.log` 是否 `get_xu(EAGAIN)`；是则把 `CAM_SETTLE` 调大 |
| Agent 起不来 | `~/vio_agent.log` 有无 `session established`；串口设备号是否变了 |
| 长时间无 `[PX4]` | `run.log` 是否停在 `start VIBA`：初始化激励不足，动一动就行 |
| video 掉到 2 | 上一轮是否正常退出；手动 `sudo /usr/local/sbin/vio-camera-prepare` |
| 时间戳告警刷屏 | RTC 1970 未解决（见 [hardware.md](hardware.md)），相对时间判读不受影响 |
