# 第二次炸机（2026-11-05 · run Tm7tjC）

2026-11-05 夜间的这次会话含两次独立起飞：第一次 VIO 观测被门控永久锁存，失去位置
反馈后纯 IMU 积分漂移 102 m；第二次在漂移点由飞手低油门迫降。工程侧有两个独立问题
叠加，逐帧证据都在本分支。第二次飞行的低电压告警为伴随遥测记录，未触发飞控动作，
不列入事故原因（见结论下方标注）。

一份可复现的事故分析数据集。含飞控原始日志、nano 侧 VIO 遥测、离线重算结果与逐帧
门控证据。不含源码，与 `master` 没有共同历史；第一次炸机的数据在
[data/first-crash-20261103](https://github.com/MalzaharGeng/ORB_SLAM3/tree/data/first-crash-20261103)。

---

## 结论

**这一夜是两个独立工程问题叠加**，不是单一故障：

| # | 问题 | 性质 | 证据强度 |
|---|---|---|---|
| 1 | **VIO 观测被门控拦截**（RTT / 帧间隔阈值过紧） | 工程实现 | ★★★★★ 逐帧可查 |
| 2 | **故障锁存无降级路径**（一次抖动判死整场） | 设计缺陷 | ★★★★★ 源码 + 日志 |

> **低电压告警标注（2026-11-06）**：第二次飞行 `battery_status.warning` 于 5.7 s 起置位、6.7 s 达 3，但 `COM_LOW_BAT_ACT=0`，飞控未因此触发任何动作；38–41 s 的下降与遥控油门（−0.63~−0.67，约 0.52–0.59 m/s）一致。该告警属**伴随遥测记录，对本次事故没有影响，不作为故障或原因列入结论**。电压曲线作为原始数据保留在 §3.2 与 `repair-20261106/final_flight_timeline.json`，证据边界见 `analysis/07_修复与复盘.md`。

**已记录窗口内 VIO 可稳定跟踪**（证据边界见 §3.3 与 analysis/07）—— 用出事瞬间的原始图像离线重算，286 帧里 285 帧正常跟踪（`tracking=2`）；事故前 127.7 秒的六段录制拼起来重算，1091 帧里 1060 帧有效、`tracking` 一次都没丢，锁存那一帧前后 VIO 位姿连续无跳变。重放只能证明被记录的窗口可以跟踪，不能证明整场无漂移、绝对精度正确或实时闭环必然安全。

**唯一例外是"纯悬停冷启动"**：单段悬停 20 秒从头初始化时，视觉能跟住但 IMU 初始化反复回退，`ba2` 始终没完成（见 §3.5）。这是 VI-SLAM 的运动激励前提，不是故障，但项目门控设计没有考虑这种状态。

---

## 一、事故概要

| 项 | 值 |
|---|---|
| 时间 | 2026-11-05 约 22:00（无人机日志时钟为 1970，见下） |
| 会话 | `19700101_080351_live_Tm7tjC` |
| 载机 | Jetson Orin Nano Super（25 W，6 核 A78AE）+ D435i 双目 IR 640×480@15fps |
| 飞控 | PX4 1.17，uXRCE-DDS over CP2102 @ 921600 |
| SLAM | ORB-SLAM3 Stereo-Inertial，nFeatures 1000 |
| 结果 | **漂移 102 m**（第一次飞行）→ 在漂移点迫降（第二次飞行，即最后一次炸机） |

### 两次飞行

`Tm7tjC` 的遥测里有两段 `armed`，是**两次独立起飞**：

| | 第一次 | 第二次（最后一次炸机） |
|---|---|---|
| 遥测时间 | t = 126 ~ 412 s（286 秒） | t = 1329 ~ 1373 s（44 秒） |
| 模式 | ALTCTL → POSCTL → ALTCTL | 全程 ALTCTL |
| 轨迹 | x −28.2~20.3，**y −4.8~101.2** | 位置几乎不动（0.84 m） |
| 结果 | 漂移 102 m 后上锁 | 飞手低油门迫降（下降与遥控一致） |
| 对应本仓库 | `data/flight_log/13_47_11.ulg` + `data/nanovio/Tm7tjC/` | `data/flight_log/14_07_14.ulg` + `replay/` |

两次飞行各有一份完整 ULog（`13_47_11.ulg` 覆盖第一次全程 287 s，`14_07_14.ulg` 覆盖第二次全程 44 s）。对齐基准：`13_47_11` 的 ULog 时钟 t=0 = nano 遥测 t=126.2 s，由四个独立事件交叉验证（解锁、EV 融合开始、模式切换、上锁位置，终点位置差 0.06 m）；第二次同理（t=0 = 遥测 1329.2 s）。

> 两次解锁的飞控 boot 间隔 916 s 与 nano 遥测两段 armed 的间隔 916 s 精确相等，确认两份 ULog 与两次飞行一一对应。

---

## 二、数据清单

```
data/
├── flight_log/13_47_11.ulg            12 MB    飞控 ULog（第一次飞行 287 s 全程）
├── flight_log/14_07_14.ulg            2.0 MB   飞控 ULog（第二次飞行 44.2 s）
└── nanovio/Tm7tjC/
    ├── px4_telemetry.csv              6.1 MB   15 Hz × 20657 行，38 列
    ├── run.log                        872 KB   SLAM 控制台全量日志
    ├── px4.yaml                       1.6 KB   桥接配置快照
    ├── settings.yaml                  3.6 KB   SLAM 参数快照
    └── launch.txt                     1.1 KB   启动命令行
```

### 关键数据源对照

| 数据源 | 覆盖 | 采样 | 内容 |
|---|---|---|---|
| `14_07_14.ulg` | 第二次飞行 44.2 s | 混合 | 姿态、遥控、电池、电机、控制分配 |
| `px4_telemetry.csv` | 两次飞行全覆盖 | 15 Hz | VIO 位姿 + EKF 位姿 + EV 融合位 + 门控原因 + 时钟质量 |
| `run.log` | 全时段 | 事件驱动 | `[PX4]` 门控判据、`VIO_PERF`、滚动录制事件 |
| `replay/` | 4 次离线重算 | — | 事故段 + 事故前六段（单段与拼接）用同一批图像重跑 VIO |

字段说明见 [`analysis/04_数据字典.md`](analysis/04_数据字典.md)。

---

## 相关位置

| 内容 | 位置 |
|---|---|
| 回放动画（浏览器直接看，炸机演示保留） | [replay_tm7tjc](https://malzahargeng.github.io/ORB_SLAM3/replay/replay_tm7tjc.html) |
| 机载滚动录制图像（1.4 GB，不进仓库） | `/home/nano/vio_image_events/`，7 个事件清单见 §5.4 |
| 第一次炸机（2026-11-03 · run 6nQrcY） | [data/first-crash-20261103](https://github.com/MalzaharGeng/ORB_SLAM3/tree/data/first-crash-20261103) |
| 源码与门控实现 | [master: src/Px4Vio.cc](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/src/Px4Vio.cc) |
| 桥接参数来源 | [master: config/px4_vio_local.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/config/px4_vio_local.yaml) |
| 试验机部署、守护脚本与飞控参数 | [nano 分支](https://github.com/MalzaharGeng/ORB_SLAM3/tree/nano) |
| 本分支快照 | [tag second-crash-20261105](https://github.com/MalzaharGeng/ORB_SLAM3/releases/tag/second-crash-20261105) |

## 三、核心发现

### 3.1 第一次飞行：漂移 102 米

| t (s) | 事件 | 状态 |
|---|---|---|
| 126.1 | 解锁 | `px4_local_frame_unready` |
| 135~145 | 手摇初始化 | `inertial_initializing` |
| **147.5** | **VIO 开始发布** | `publishing`，`EV=1/1` |
| 165.7 | 切 POSCTL | 定点悬停，dH 0.00~0.15 |
| 214~228 | 真实机动 | vx 1.26 m/s，**不是跳变** |
| 228~305 | VIO 帧率走低 | 大量 `waiting_new_frame` |
| **305.2** | **永久锁存** | `time_sync_unready` → `observation_gap` → `airborne_fault_latched` |
| 308.2 | 降级 ALTCTL | PX4 失去位置 |
| 308~412 | 漂移 | y 从 −0.01 涨到 98.79 |
| 412.9 | 上锁 | 位于 (−19.24, **102.25**) |

锁存瞬间原始日志：

```
[PX4] publishing        hz=14.96  sent=2176  EV=1/1/0
[PX4] time_sync_unready hz=14.99  sent=2186  EV=1/1/0   ← RTT 20387us > 20000 门限
[PX4] time_sync_unready hz=0.999  sent=2186  EV=0/0/0
[PX4] observation_gap   hz=0.999  sent=2186  EV=0/0/0   ← 帧间隔 1001ms > 300ms 门限
[PX4] airborne_fault_latched      sent=2186             ← 永久锁存，sent 冻结
```

`sent` 停在 2186，之后 1976 次全部 `rejected`。

### 3.2 第二次飞行：迫降过程与电压记录（伴随，非因果）

| t (s) | 电压 | 电流 | 剩余 |
|---|---|---|---|
| 0.34 | 22.96 V | 0.8 A | 46.9% |
| 3.54 | 22.07 V | 11.4 A | 33.6% |
| 6.74 | 20.76 V | 25.6 A | **4.4%**（紧急告警） |
| 8.34 | 20.25 V | 31.3 A | **0%** |
| 9.94 | **19.99 V** | **34.9 A** | 0% |
| 43.54（落地） | 22.31 V | 0.9 A | 22% |

6S 电池负载下最低 19.99 V（3.33 V/cell），卸载后回升至 22.31 V。以上仅为原始遥测记录，不作硬件判定；该告警未触发飞控动作，与迫降无因果关系（见结论下方标注）。

飞行中控制链正常（`torque_ok=1`、`thrust_ok=1`、`sat=0` 全程），姿态跟随摇杆一比一。

### 3.3 VIO 离线重算：算法正常

| 测试 | 数据 | 有效位姿 | tracking | ba2 |
|---|---|---|---|---|
| A | 锁存瞬间 286 帧 | **285（99.7%）** | **`2`×285** | 完成 135 帧 |
| B | 前段六段拼接 1091 帧 / 127.7 s | **1060（97.2%）** | **`2`×1090，掉出 0 次** | 完成 618 帧 |
| C | 事故前悬停段 285 帧 / 20 s | 269（94.4%） | `2`×284 | **完成 0 帧** |

**同一套算法、同一批图像，重算几乎全程正常跟踪。**

> **修订**：初版重算得到 105/286 和 43/1091，是格式转换时把 `imu.csv` 的加速度轴与角速度轴对调所致（`RollingInputRecording.cc:159` 的顺序是 `t, ax, ay, az, gx, gy, gz`）。修正后 98% 以上帧有效，`tracking` 甚至一次都没丢。详见 `analysis/02_VIO能力边界.md`。
>
> **B 行二次修订**：改用 `scripts/replay_splice_rolling.py`（重叠帧去重 + IMU 连续化，而不是简单按时间截断）后复算，1091 帧里 1060 帧有效、618 帧完成 VIBA2，比初版拼接的 1071/657 略低但数据链更干净：重叠区不再有重复 IMU 样本。两次结果同量级，结论不变。

→ 详见 [`analysis/02_VIO能力边界.md`](analysis/02_VIO能力边界.md)

### 3.4 门控阈值与实测的错配

| 判据 | 代码门限 | 实测最大值 | 超出 |
|---|---|---|---|
| `roundTripUs` | 20 000 us | **23 794 us** | +19% |
| `\|observed−estimated\|` | 5 000 us | **6 378 us** | +28% |
| `observation_gap` | 300 ms | **1 067 ms** | +256% |
| `max_age_seconds` | 250 ms | 190 ms | 未超 |
| `track_ms` vs 帧间隔 | 66.7 ms | **77 ms** | 处理慢于采集 |

→ 详见 [`analysis/03_门控缺陷清单.md`](analysis/03_门控缺陷清单.md)

### 3.5 事故前"正常段"基线（补做）

把事故前被滚动录制下来的六段（284.15–411.82 s）全部回放，与事故段对照：

| 指标 | N0 悬停段 284–304 s | NA 前段拼接 284–412 s | XF 事故段 1352–1373 s |
|---|---|---|---|
| 帧数 | 285 | 1091 | 286 |
| `valid` 帧 | 94.4 % | 97.2 % | **99.7 %** |
| `tracking=OK` | 99.6 % | 99.9 % | 99.7 % |
| `ba2` 完成 | **0 %** | 56.6 % | 47.2 % |
| 首次 `ba2` | **从未** | 18.93 s | 11.00 s |
| `track_ms` p50 / p95 | 51.6 / 59.1 ms | 53.6 / 61.8 ms | 52.3 / 56.0 ms |
| TRACK LOST | 0 | 0 | 0 |

三条关键结论：

1. **锁存时刻 VIO 是健康的。** 305.2 s（`ev_pos` 1→0、`sent` 冻结 2186）前后逐帧 `valid=1 / tracking=OK / ba2=1`，位置连续（−0.01, −0.02, 0.05 → −0.02, −0.02, 0.05），无跳变。卡的是程序侧时间同步判定。
2. **`time_sync_unready` 不是突发，是持续劣化。** 六段录制里有五段的触发原因就是它；280 s 时 `rejected` 已累计 1961 次，305.2 s 只是越过锁存阈值。
3. **纯悬停冷启动是真实弱点。** N0 中 IMU 初始化被"运动不足"打断 2 次、地图重建 3 次、VIBA2 一次没到；而飞机在动的段落 11 s 就完成 VIBA2。VIO 悬停时的水平漂移量级为 5–8 cm/s，与"漂 102 m"无关（那是锁存后 PX4 纯 IMU 积分约 100 s 的结果）。

→ 详见 [`analysis/05_正常段回放与基线.md`](analysis/05_正常段回放与基线.md)（数据表见本节与 `replay/replay_summary.json`，逐帧明细见 `replay/norm0_*.csv`、`replay/norma_*.csv`。）

---

## 四、可交互回放

```
replay/replay_tm7tjc.html        251 KB  事故段回放，单文件双击即开
replay/norm0_navigation.csv              正常悬停段 284–304 s 逐帧输出（285 帧）
replay/norm0_timing.csv                  同上，逐帧耗时
replay/norma_navigation.csv              前段六段拼接 284–412 s 逐帧输出（1091 帧）
replay/norma_timing.csv                  同上，逐帧耗时
replay/navigation_latch286.csv           事故段 1352.9–1372.8 s 逐帧输出
replay/navigation_splice1091.csv         旧版拼接输出（保留作对照）
replay/replay_summary.json               三组指标汇总
```

- `replay_tm7tjc.html`：左水平轨迹（绿=EKF / 蓝=VIO / 红点=锁存位置），右仪表，下三曲线，可拖时间轴

`replay/*.csv` 是离线重算的逐帧输出（index, timestamp, valid, tracking, imu, ba2, 位置, 四元数）。

---

## 五、复现方法

### 5.1 复现门控判定结论

```bash
# 看锁存前的完整判据
grep -a '\[PX4\]' data/nanovio/Tm7tjC/run.log | tail -400 | grep -aE "time_sync_unready|observation_gap|airborne_fault|publishing"

# 看 VIO 处理耗时（是否跟得上 15 fps）
grep -a 'VIO_PERF' data/nanovio/Tm7tjC/run.log | awk -F'track_ms=' '{split($2,a," "); if(a[1]>66) print}' | head
```

### 5.2 复现 ULog 分析

```bash
pip install pyulog
python scripts/analyze_ulg.py data/flight_log/14_07_14.ulg
```

输出：模式序列、电池曲线、电机输出、控制分配状态、姿态与摇杆对照。

### 5.3 复现 VIO 重算

需要 ORB-SLAM3 可执行文件 `d435i_recovery_replay` 与原始滚动录制（本仓库未包含，见 5.4）。

```bash
python scripts/convert_rolling.py <event_dir> <output_dir>
d435i_recovery_replay Vocabulary/ORBvoc.txt <output>/settings.yaml <output> <replay_out>
```

`convert_rolling.py` 把滚动录制格式（`frame_xxx/`）转成 replay 要求的 `stereo.csv` + `imu.csv`。

多段拼接（事故前正常段）用第二个脚本：

```bash
python scripts/replay_splice_rolling.py <output_dir> <event_1> <event_2> ... <event_6>
d435i_recovery_replay Vocabulary/ORBvoc.txt <output_dir>/settings.yaml <output_dir> <replay_out>
```

它按图像时间戳排序、重叠帧去重、被丢弃帧的 IMU 补进前一保留帧、每帧 IMU 批保证全局单调。汇总统计用 `python scripts/analyze_replay_segments.py`。

**关键约束**：replay 要求 `index` 从 0 连续递增，原始帧号（如 19613）会报 `frame index gap`。

### 5.4 原始图像数据获取

滚动录制的全帧双目图像未包含在本仓库（7 个事件 × 20 s × 290 KB ≈ 1.4 GB，超过 Git 仓库合理体积）。

存储在机载设备：

```
/home/nano/vio_image_events/
├── event_535187059_3003_0   285 帧  time_sync_unready
├── event_545232749_3003_1   293 帧  time_sync_unready
├── event_554376806_3003_2   284 帧  time_sync_unready
├── event_621951414_3003_3   279 帧  time_sync_unready
├── event_636011137_3003_4   280 帧  time_sync_unready
├── event_645986425_3003_5   247 帧  px4_local_frame_unready
└── event_1603810252_3003_6  286 帧  airborne_fault_latched  ← 锁存瞬间
```

每个事件目录含 `frame_xxx/{left.png,right.png,frame.csv,imu.csv}` + `settings.yaml` + `camera_calibration.json` + `overview/`（120 帧稀疏上下文）。

---

## 六、目录结构

```
.
├── README.md                        本文件
├── data/
│   ├── flight_log/                  两份 ULog（13_47_11 第一次 / 14_07_14 第二次）
├── repair-20261106/             门控修复实施记录（staged/baseline 源码、patch、验证日志）
│   ├── nanovio/Tm7tjC/              nano 侧遥测与配置
│   └── px4_windows/                 两段遥测窗口切片（284–412 s / 1352–1373 s）
├── analysis/
│   ├── 01_事故复盘.md               两次飞行的完整时间线
│   ├── 02_VIO能力边界.md            算法能与不能（含重算证据）
│   ├── 03_门控缺陷清单.md           5 处阈值错配 + 锁存设计问题
│   ├── 04_数据字典.md               字段含义与坑
│   ├── 05_正常段回放与基线.md       事故前六段回放、三组对照与四条结论
│   └── 06_综合结论.md               ULog × 内录三段联合分析（文字为主）
├── replay/
│   ├── replay_tm7tjc.html           事故段可交互回放
│   ├── norm0_navigation.csv         悬停段 285 帧（269 有效，ba2 0）
│   ├── norm0_timing.csv
│   ├── norma_navigation.csv         前段拼接 1091 帧（1060 有效，ba2 618）
│   ├── norma_timing.csv
│   ├── navigation_latch286.csv      锁存瞬间重算（285 有效）
│   ├── navigation_splice1091.csv    旧版拼接重算（保留对照）
│   ├── replay_xf_navigation.csv     事故段重放（285/286 有效）
│   ├── ulog_flight1.csv             第一次飞行 ULog 重采样（t 折算到遥测轴）
│   ├── ulog_flight2.csv             第二次飞行 ULog 重采样
│   └── replay_summary.json          三组指标汇总
├── evidence/
│   └── 关键日志.txt                 锁存前后原文摘录
└── scripts/
    ├── analyze_ulg.py               ULog 分析
    ├── convert_rolling.py           滚动录制 → replay 格式（单段）
    ├── replay_splice_rolling.py     多段拼接 → replay 格式（去重 + IMU 连续化）
    ├── analyze_replay_segments.py   三组 replay 指标汇总
    └── check_sample_gap.py          逐帧门控空洞统计
```

---

## 七、已知环境问题

| 问题 | 表现 | 影响 |
|---|---|---|
| nano 系统时间 1970 | RTC 无后备电池，每次断电复位 | timesync / 文件时间戳 / SSL |
| VIO 处理速度临界 | `track_ms` 77 ms vs 帧间隔 66.7 ms | 周期性 1 秒跳帧 |
| 滚动录制仅 20 s | 事件触发只能回溯 20 秒 | 无法复现初始化阶段 |

---

## 八、许可与引用

数据为真实飞行记录，用于本机 VIO 系统的问题定位。

如果引用，请说明这是 2026-11-05 夜间 `Tm7tjC` 会话的复盘数据。
