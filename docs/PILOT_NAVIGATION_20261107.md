# 遥控器优先导航审计（2026-11-07）

本仓库只提供 SLAM/VIO 定位，不含行为树。用户指出的实机行为树在同一 Ubuntu-26.04 的 `~/drone_sim/scripts/px4_square_bt.py`，本次已同步修改该工程。完整说明和命令见 [drone_sim 权限/十米/重定位说明](../../drone_sim/docs/PILOT_NAVIGATION_20261107.md)。两处源码都要保留，单独复制 ORB_SLAM3 不会带上行为树。

本仓库改动：保留已有地图/时钟故障锁存与位置-速度一致性门控，新增 `max_attitude_step_deg + max_attitude_rate_deg_s * dt` 的姿态跳变检测；解锁后的姿态跳变必须锁机重建参考，不能靠 landed recovery 释放。默认 10° + 180°/s × dt，属于待调参数。扩展了 `px4_vio_regression`，验证 yaw/roll 突变与连续旋转。原有初始 IMU 导航坐标与跨地图对齐未改动。

控制权限与定位质量是两条独立链路：Position 模式保留 VIO 给 PX4 的定位，行为树只在物理遥控器授权和实际 Offboard 下发送轨迹；切 Altitude/Position 后撤销。行为树不再发解锁/上锁、切模式、起飞、降落命令。重定位参考改变会撤销任务，即使两张地图的数值原点看起来相同。

本次保留原工作区未提交改动，修改前的完整 tracked diff 与关键文件快照在 `~/flight_control_review_20261107/`。未推送 GitHub，未部署到 Nano，未刷飞控，未进行实体飞行。

## 当前实机入口限制

此 WSL 构建未发现 RealSense SDK，`build_local` 无 `stereo_inertial_realsense_D435i` 目标；桥接 core/ROS fixture 可编译。已有 [新相机记录](CAMERA_MODULE_20261106.md) 明确：实际相机已换为 USB3 模组，但项目仍缺该模组的采集入口和实测标定。此次权限/跳变修改不补造相机参数，也不据软件 mock 测试宣称可直接实飞。

PX4 默认 DDS 表还需导出物理开关消息，准备好的补丁在 `~/drone_sim/docs/rc_switch_dds.patch`；本次未应用或刷机。缺少该话题时行为树保持等待，不放宽授权条件。
