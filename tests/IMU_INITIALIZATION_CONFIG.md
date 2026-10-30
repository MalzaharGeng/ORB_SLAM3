# IMU 初始化参数

实机默认文件：`Examples/Stereo-Inertial/RealSense_D435i.yaml`。

```yaml
IMU.fastInit: 0
IMU.VIBA1Time: 5.0
IMU.VIBA2Time: 15.0
```

`IMU.fastInit: 0` 保留首次建图时的加速度变化检查；设置为 1 仅跳过这一项检查，
不跳过 VIBA、最少关键帧数、惯性优化或后续运动不足检查。当前默认明确为 0。

两项 VIBA 时间是同一个 `mTinit` 计时的累计触发阈值，单位秒，不是优化耗时、
程序运行时长，也不是第二段在第一段结束后额外等待的时间。
例如 3/10 表示 `mTinit > 3` 时尝试 VIBA 1，第一段成功后且 `mTinit > 10` 时尝试 VIBA 2。
初次 IMU 初始化以关键帧时间跨度设置 mTinit，之后按原有位移条件累计；运动不足时不一定继续计时。
这两项参数对首次地图和重置后第二张地图都生效。

要求：`0 < IMU.VIBA1Time < IMU.VIBA2Time`，数值必须有限。整数和小数写法均支持。
没有这两个键的旧文件自动使用 5/15；大于 50 秒的设置也能触发，已移除原来对待执行 VIBA 的 50 秒外层限制。
原有单目尺度精化窗口仍保持原样。

初始化条件尚未满足、优化提前返回时，不设置 VIBA 完成标志，会在后续关键帧重试。
启动会输出实际的 fastInit、时间阈值及 IMU 重定位恢复帧数；触发 VIBA 时输出当时的 mTinit 和阈值。
`end VIBA 2` 表示本次调用完成，但不等同于经过真值验证的定位精度。

## 配置读取修正

- File.version=1.0 与旧版配置共用 IMU 读取/校验，fastInit 在两条路径都生效。
- 新版补齐 `mnFramesToResetIMU = mMaxFrames`，与旧版一致，不再使用未初始化的恢复窗口。
- `IMU.Frequency: 200` 和 `200.0` 均支持；浮点参数可写整数，Camera.fps 支持 30 / 30.0。
- IMU 频率和测量噪声必须大于零，偏置随机游走非负；外参支持 f/d 矩阵，并检查有限数值、4×4 形状及刚体旋转。
- 同时支持 `IMU.T_b_c1` / `Tbc`、`IMU.InsertKFsWhenLost` / `InsertKFsWhenLost`；同时出现且值冲突则报错，不静默忽略。
- 非惯性传感器模式不再读取未设置的 IMU 参数来创建预积分器。
- 右目针孔畸变 getter 和右目鱼眼参数读取修正为各自的 Camera2 数据；D435i 的 Rectified 配置不受这两处修正影响。
- `mImuPer=0.001` 是现有 IMU 时间边界容差，本次没有把它改成采样周期，以免混淆含义。

配置解析错误会在启动 SLAM 工作线程之前报出具体键名。
这是一轮初始化及相机参数读取路径修正，不代表所有传感器与所有标定组合都已实机验证。

## 使用

代码首次更新后编译：

```bash
cmake -S . -B build -DORB_SLAM3_BUILD_TESTS=ON
cmake --build build --target stereo_inertial_realsense_D435i d435i_recovery_replay imu_settings_regression -j2
```

以后仅修改 YAML 的 fastInit/VIBA 时间，无需重新编译；重启程序生效：

```bash
./run_recovery_test.sh --headless
```

启动脚本的 `config/recovery_*.conf` 控制 GUI/SSH/终端输入，不是相机或 IMU 参数文件。
每次运行会复制实际使用的 YAML 到输出目录，方便检查。
`--replay` 默认使用录制目录的 `settings.yaml`，不会自动用实机 YAML 覆盖历史录制配置。
如需用不同时间阈值回放，复制录制的 settings.yaml 到新文件、修改阈值，再直接传给回放程序：

```bash
ORB_REPLAY_STRIDE=2 ./build/tests/d435i_recovery_replay \
  Vocabulary/ORBvoc.txt /path/to/replay-settings.yaml \
  recordings/d435i_recovery_verified_20260918_1654/combined /tmp/imu-timing-replay
```

测试：`ctest --test-dir build --output-on-failure`。
`imu_settings_regression` 会构造新旧配置，检查参数实际传给 Tracking/LocalMapping、默认值、非法输入、
右目独立畸变参数及纯视觉路径，不需要相机。

2026-10-30 验证：全工程编译通过、4/4 CTest 回归通过。使用独立的 3/10 秒配置进行真实录制回放，
两张地图均按所配阈值完成 VIBA，完整存图/重置/读档/对齐流程通过，GDB 正常退出。
实机默认仍是 fastInit=0、5/15 秒。验证日志在 `recordings/test_runs/imu_configuration_validation_20261030/`。
