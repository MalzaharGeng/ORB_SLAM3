# 飞控侧配置

PX4 1.17（FMUv5 / STM32F76）。这里只记与 VIO 接入直接相关的项，其余保持飞控原有
配置。所有参数都按「先拆桨地面验证，再上飞行」的顺序推进。

## 视觉融合

`EKF2_EV_CTRL = 11`：打开位置和航向，没有打开速度。

EV 不提供速度时，EKF 只能用位置观测去推速度。观测是 15 Hz、带时延的，速度越高
推算误差越大，残差累积到门限会触发位置重对齐，表现成一帧跳变。桥接侧现在加了两
秒窗口的位置-速度一致性检查，能拦掉这类输出，但不改变「缺速度观测」这个前提。
11-03 那次的逐帧遥测在 [data/first-crash-20261103](https://github.com/MalzaharGeng/ORB_SLAM3/tree/data/first-crash-20261103) 分支。

如果以后要让桥接发 `twist`，再考虑打开速度位；在那之前不要先开，速度项没有可信
输入。

## 时延

`EKF2_EV_DELAY` 当前 120 ms。实测把 Nano 的 `timestamp_sample` 换到飞控时钟后，与
EKF 记录的 EV 采样时间差中位数约 125 ms，其中约 5 ms 是 EKF 半积分周期。

Nano 发的已经是图像采样时刻，处理耗时体现在「发送时间减采样时间」上，不是时间戳
本身。再把 120 ms 叠上去，等于把观测往过去多移了一次。建议从 0 起算，在地面用
创新序列和运动数据校剩余时延。

`EKF2_EV_DELAY` 的调整会影响所有 EV 观测，改完必须重新看创新均值和拒绝计数。

## 杆臂

相机到 D435i IMU 的变换在 SLAM 配置里（`IMU.T_b_c1`，见 [RealSense_D435i.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/Examples/Stereo-Inertial/DECXIN_Camera.yaml)），机体杆臂只在飞控配：

```
EKF2_EV_POS_X = 0.15
EKF2_EV_POS_Y = 0
EKF2_EV_POS_Z = -0.02
```

按 FRD（前 +X、右 +Y、下 +Z）粗测，D435i 的 IMU 在重心前 15 cm、上 2 cm。这是卷尺
量级，需要精测。当前飞控里三项都是 0，尚未补偿。

两个变换不要混用：`IMU.T_b_c1` 是相机到 D435i 内部 IMU，`EKF2_EV_POS_*` 是那个 IMU
到机体重心的距离。

## 高度参考

`EKF2_HGT_REF = 0`（气压）。评估视觉垂直漂移和掉高时用这个，避免把视觉高度误差
和气压基准混在一起看。

## 接管

`COM_RC_OVERRIDE = 3`：自动模式和 Offboard 都允许摇杆接管。只开自动模式那一位时，
Offboard 期间打杆不会直接夺回控制权。

模式开关本身是独立的人工接管路径，不依赖这个参数。

## 日志

`SDLOG_MODE = 2`：上电即开始写 ULog，直到断电。默认值 0 是解锁后才记，落地后
断电前的那段会丢。这个参数写入后需要重启飞控生效。

历史日志里出现过 CPU 97.4% 的告警，末段约 87%。持续开机记录会增加 SD 写入，
是否影响飞控负载要单独核实。

## 判读注意

`cs_yaw_align = 0` 且 `cs_ev_yaw = 1` 是可能同时成立的：PX4 1.17 在 EV 航向以 FRD
接入时，这个对齐标志本来就可能保持 0。不能只凭 `cs_yaw_align` 判断航向有没有
融合，要看 `cs_ev_yaw` 和实际航向跟踪。11-03 的内录里 `cs_ev_yaw = 1` 占 3400 行
中的 2673 行，见 [data/first-crash-20261103](https://github.com/MalzaharGeng/ORB_SLAM3/tree/data/first-crash-20261103)。
