# 录制、地图与现场命令

服务启动后自动运行 DDS Agent、VIO、遥测 CSV、双目环形图像和同步相机 IMU。
自主飞行的行为树不随服务启动，仍需单独触发。

```bash
systemctl status px4-vio.service
tail -f ~/vio_daemon.log
```

## 运行目录

```
~/ORB_SLAM3/recordings/test_runs/<run>/
├── run.log                  SLAM 控制台全量输出
├── px4_telemetry.csv        桥接逐帧遥测
├── camera_calibration.json  本次实际打开 profile 的标定与序列号
└── image_ring/
    ├── 000001/ ...          最近 20 秒全帧率双目 + 同批次 IMU
    ├── overview/            每约 1 秒一组的最近 120 秒背景
    └── clock_origin.txt     图像时戳到主机时间的换算
```

`~/inrec/*.csv` 是同步内录，由 `vio_inrec.py` 写，与图像缓冲相互独立。

## 环形图像

写盘在独立线程：编码、提交、淘汰都不在主循环上，待写队列最多 6 帧，溢出只加
计数（`ring_dropped`），不等磁盘。

- 全帧率区保留最近 20 秒，记录的是实际送进 `TrackStereo` 的双目和同步 IMU 批次。
  IMU 的加速度按陀螺仪时刻插值，不是 SDK 未加工的原始流。
- 背景区每约 1 秒留一组，复用已编码 PNG 的硬链接，不重复压缩，也不把这部分原图
  堆进内存。背景里的 IMU 批次是断续的，只能看场景，不能连续复跑惯性。
- 磁盘剩余低于 2 GiB 时停止新增图像，写告警，VIO 和遥测继续。
- 正常只保留最近 20 秒；异常会冻结成事件，冻结内容走硬链接，后续淘汰删不掉。

## 事件

冻结触发条件：跟踪或惯性未就绪、地图参考变化、桥接故障原因变化、手动标记。
同一短时故障按 5 秒去重。

事件目录 `~/vio_image_events/event_*`，上限 24 份且总量 4 GiB，超出按时间淘汰
较旧的事件。历史会话的环形目录保留最多 3 份，另加当前一份。遥测 CSV 不参与这
套清理。

手动冻结当前 20 秒：

```bash
touch ~/ORB_SLAM3/recordings/test_runs/<run>/freeze_images
```

导出成回放格式（目标目录必须不存在）：

```bash
python3 ~/export_vio_image_ring.py ~/vio_image_events/<event> ~/vio_export_<name>
python3 ~/export_vio_image_ring.py ~/vio_image_events/<event>/overview ~/vio_context_<name>
```

导出按已提交的帧走，半写入帧跳过。

## 地图

飞行配置默认关闭回环、GBA 和合图，只保留局部 BA 与原图重定位，目的是避免控制
过程中坐标突变。需要全局一致的地图时走地面建图 profile：先停服务腾出相机，
建完再恢复。

```bash
sudo systemctl stop px4-vio.service
cd ~/ORB_SLAM3
./run_recovery_test.sh --mapping --headless
# 正常退出后
sudo systemctl start px4-vio.service
```

`--mapping` 会用 master 的 [config/px4_vio.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/config/px4_vio.yaml)（外参无效）和只收不发的桥接，拒绝自定义
硬件配置；回环开着的配置里再去发硬件 EV，实机入口会直接报错退出。原因很简单：
全局回环会改地图坐标，控制端不能同时吃两种坐标系。

`~/maps.sh`（[maps.sh](../bin/maps.sh)）管理保存下来的 Atlas，存档在 `~/maps/<名字>.osa`：

```bash
~/maps.sh list
~/maps.sh use <名字>
~/maps.sh new <名字>
~/maps.sh status
```

注意：启动器加载 Atlas 之后仍会新建本次活动地图。文件加载成功不等于已经定位回
历史地图，跨启动自动恢复历史原点这条没有验证过。

## 控制台命令

前台模式下（`--console`，服务里默认开）可用：

| 命令 | 作用 |
|---|---|
| `first` | 首次 VIBA2 完成后存图 |
| `reset` | 飞行配置下直接建新图，旧图留在 Atlas；不再要求先存图 |
| `second` | 确认第二次初始化 |
| `load` | 读取首次地图并匹配 |
| `status` | 阶段、存图状态、最近位姿 |

服务运行时通过 FIFO 送命令：

```bash
printf 'reset\n' > ~/.vio_stdin.fifo
```

写完在 `run.log` 里确认 `[MapPolicy] explicit reset`，没有这行说明命令没进去。

## 定位健康

- 空中可以完成 IMU 激励和首次 VIO 接入，但只允许 ALTCTL、MANUAL、STAB 三种模式；
  POSCTL、Offboard 和自动任务中拒绝首次接入。
- 首次接入后有一个 3 秒采纳窗口，这期间 EKF 因为采纳 EV 而重置不会反过来重锚
  EV 框架。窗口结束后再出现地图或时钟异常，走故障锁存。
- 视觉失效时 PX4 自身的姿态环通常还能撑住，但水平定点不保证。地图恢复是后台
  行为，不是「飞机可以安全等 10 秒」的承诺。
