# NP-01 算法主线同步与验证边界（2026-11-16）

本次把 WSL 已使用的算法源码、仿真配置和离线回归纳入 master。仿真输入适配器、飞手、场景、监督器与审计报告归 [RM27_drone_sim](https://github.com/MalzaharGeng/RM27_drone_sim)，Nano 分支及设备未同步。

## 源码改动

- `SensorClock` 区分单帧陈旧/主机时钟异常与需要清空全部流的传感器域故障；真停更仍以 200 ms 判定，增加拒绝原因、停更流和 late-attach 回归。
- `Gate`/`Bridge` 增加按值保存的比较快照、源时间映射和帧变换诊断，能回放实际被比较的前后样本。
- 首次建立有效导航参考与已有参考改变分开计 epoch；地图初始化提交前后核对地图身份；可选 VIBA2 保留窗口使用单调时间绑定同一地图，默认关闭。
- 空地图、空参考和祖先链异常时，轨迹导出写出可解释的空输出，避免退出时崩溃。
- 新增序列化 EV 帧契约、地图保留策略、Gate 快照、late-attach 时钟和空地图导出测试；保留本地已有的启动器测试入口修正提交。

相对上一次远端 master，本批次还包含既有 Gate 行为调整，不能统称“只加日志、门限全未改”：fusion status 新鲜度 0.5→1.5 s；负样本年龄异常界 −5→−50 ms 且该分支不再永久锁存；主机 system/steady 偏差界 50→200 ms；配置校验允许 time-sync residual 到 1.0 s，以加载单独的 transport 配置。运动包络与参考变化故障路径仍分别约束。这些变更不代表 Nano 或实机验收。

## 配置

`RM27_Sim_IMU_Stereo_baseline.yaml` 关闭 `never_reset_map` / `retain_without_ba2` 两个实验开关；实验配置单独保留。仿真基线仍有自己的 VIBA2 窗口等设置，不能理解成与 Nano 配置逐字相同。相机内外参来自 Gazebo 几何，禁止当作实机标定。

`px4_vio_sim.yaml` 为严格配置，`px4_vio_sim_transport.yaml` 为标记 [SIM-ONLY] 的传输补偿配置；后者的 time-sync / 样本年龄结果不能登记为实机同源验收。

## 历史飞行证据

FIX20 的高度参考修复属于仿真飞手：`ref_z += delta_z`，只按单个可确认 uint8 counter 事件平移一次，异常失败关闭。修复后正常 5 m 回归和独立冷启动 3/3 已通过；旧 ACC03 不改写为新版本结果。

2026-11-16 中断轮已实际 SIGTERM 唯一节点，观察到约 0.619 s 授权撤销、0.623 s 接管、9.126 s 落地及 11.127 s DONE/disarm。专项 B，严格验收 UNVERIFIED：C1–C8/C10 PASS，C9 缺共享 MicroXRCEAgent 运行后独立证据；不是已证实响应失败。普通评估器 FAIL 14/2/1 原样保留。完整报告见配套仓库 `docs/verification/20261116/`。

本次 Git 同步不新增飞行。原场地可靠性、延迟性能、Nano 实机和其他故障类型仍未验证；新源码构建也不能冒充原冻结二进制。

## 本次 Git 同步检查

2026-11-16，在 WSL 复用已有构建目录完成 ORB_SLAM3、PX4 core 和新增回归目标增量构建；20/20 CTest 全 PASS。重新链接的帧契约/Gate 快照两项再次 PASS。检出目录与原构建输入的源码、CMake、测试和配置逐字节核对，覆盖 121 个已跟踪输入及本次新增文件，来源见 [文件清单](NP01_SOURCE_MANIFEST_20261116.json)。

配套仿真仓库的 3 个 ROS 节点完成独立目录编译；生产 reset 回放 11/11、其他离线回归 117/117。未启动新的仿真或飞行；未在 aarch64/Humble 编译，未操作 Nano。未声明新构建产物与历史冻结二进制逐字节相同。
