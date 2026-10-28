# 初始 IMU 导航坐标系

导航世界系定义为当前地图首个关键帧对应的 IMU 坐标系：原点在该时刻的 IMU 中心，
X/Y/Z 轴与该时刻的 IMU 三轴一致。初始 IMU 位姿为零平移、单位旋转；初始化运动后的位置不会被强制归零。
“初始”是 SLAM 建立该张地图的首关键帧时刻，不是启动程序或点击 First Init Done 的时刻。
首次地图保存后，读档并完成几何匹配，会把第二次运行的输出转换到第一张地图的初始 IMU 坐标系。
重置后尚未匹配成功时，界面明确显示 Current map initial IMU，不能当作已经恢复首次位置。

## 刚体变换的消除

内部视觉惯性优化仍使用原有坐标系，重力计算不变。设内部地图坐标系为 M，
首关键帧的 IMU 坐标系为 B0，当前 IMU 为 B，左相机为 C；T_AB 表示 B 到 A 的变换：

```text
T_B0_M = inverse(T_M_B0)
T_B0_B = T_B0_M * T_M_B
T_B0_C = T_B0_M * T_M_C
P_B0   = T_B0_M * P_M
```

每次输出使用同一地图更新锁内的首关键帧和当前帧位姿。如果优化给全图施加刚体变换 G：
`inverse(G*T_M_B0) * (G*T_M_B) = inverse(T_M_B0) * T_M_B`。
因此平移和旋转都会消除；这并不掩盖相机的真实相对运动，也不消除局部估计误差或尺度错误。
地图点、关键帧、相机图标及 GUI 数值使用相同的转换。

首关键帧的位姿会随优化更新，但导航参考始终是同一物理时刻的 IMU。
此做法不冻结整个首帧的优化变量，不改变内部重力方向假设。
初始 IMU 的 Z 轴不一定与重力方向重合，也不自动等同于 ENU/NED。

## 读档及坐标转换

原始 `.osa` 仍保存内部地图坐标和首关键帧、IMU 外参，文件格式不变。
加载后从首关键帧的 `GetImuPose()` 恢复导航参考，因此此前包含约 2 m 原点偏移的旧存档也能使用。
不要把外部读取的原始地图点直接当作导航坐标；使用 `Map::GetInitialImuFrame()` 转换。
同时读取其他地图位姿/点时，调用方应持有 `Map::mMutexMapUpdate`，保证快照一致。

跨地图匹配仍在内部坐标中进行，保持原来的 PnP 和 3D 检验。设 A1/A2 为各地图到自身初始 IMU 的变换，
T_M1_M2 为匹配求得的原始跨地图变换，则存储：

```text
T_B01_B02 = A1 * T_M1_M2 * inverse(A2)
当前 IMU 在首次参考系：T_B01_B = T_B01_B02 * A2 * T_M2_B
```

连续两次匹配的一致性也在初始 IMU 坐标系中检查。后续第二张地图发生整体刚体调整时，
重新计算 A2，从而不会把调整泄漏到恢复后的导航输出。局部形变和跟踪漂移仍不由此消除。
此前的两次 VIBA 2 与人工确认流程保留。

## 接口与界面

- `GetNavigationState()`：一次读取相机、IMU、原点和地图转换的完整快照；先检查 `valid`。
- `GetCurrentImuPose()`：IMU 到导航世界系的位姿；使用 IMU 中心计算当前位置和距离。
- `GetCurrentPose()`：左相机到同一导航世界系的位姿。相机与 IMU 有安装偏移，初始相机位置不必为零。
- `TrackStereo()`：保留原有返回约定，返回内部地图坐标中的 Tcw；导航接入使用上述导航接口。
- GUI 的 `IMU XYZ`、`Distance`、`IMU q(wxyz)` 显示当前 IMU 位姿；红/绿/蓝坐标轴分别是初始 IMU 的 X/Y/Z。
- 读档报告增加 `imuInFirst` / `imuDistance`。`secondOriginInFirst` 表示第二次初始 IMU 在第一次初始 IMU 世界中的位置。
- 回放的 `navigation.csv` 记录归一化位姿、内部原始位姿及原点误差，`recovery_report.csv` 追加读档时的 IMU 位置/距离。

## 验证

`atlas_save_regression` 对实际 Map/KeyFrame/MapPoint 施加包含三轴旋转和数米平移的刚体变换，
检查 IMU 位姿与地图点的导航坐标保持不变、原点始终为单位位姿；然后重复存取 5 次，检查坐标参考不丢失。
`recovery_report_regression` 检查相机与 IMU 安装偏移的方向和组合关系。
真实录制回放逐帧检查原点单位位姿，且读档当帧的报告必须与导航 API 的相机/IMU 输出一致。

```bash
cmake --build build --target atlas_save_regression recovery_report_regression d435i_recovery_replay stereo_inertial_realsense_D435i -j2
ctest --test-dir build --output-on-failure -R 'atlas_save_regression|recovery_report_regression'
./run_recovery_test.sh --replay --debug
```

2026-10-23 验证结果：全工程编译通过、2/2 回归测试通过，带 GUI 的真实数据回放完整通过。
1684 帧有效导航快照中，局部初始 IMU 原点的最大平移残差约 4.77e-7 m，旋转残差约 3.02e-8 rad。
同一段回放在 t=81.379 s 的内部原始 IMU 位置跳变 4.668 m，而转换后的相邻帧位移为 0.0408 m。
这反映了整体坐标跳变被消除，不代表该帧绝对定位误差为 0.0408 m。
用户此前约两米偏移的存档经转换后，首帧 IMU 为零位置/单位姿态，最后一帧距起点约 0.0202 m。
证据在 `recordings/test_runs/initial_imu_frame_validation_20261023/`。
