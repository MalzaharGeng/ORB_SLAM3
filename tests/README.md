# Atlas 保存回归测试

> 本文来自 [Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3/tree/d4e700981e3cf43ec323606fe4a7282b9d03590b)，保留 avi 的历史测试记录。当前分支采用 C++17 和构建树输出；启动脚本支持 `BUILD_DIR`（默认 build/）及 `RECORDING_DIR`。下面直接调用 `./Examples/...` 的旧命令应改用 `./build/Examples/...`；2026-10-30 本轮实际验证范围见 [同步报告](../docs/AVI_SYNC_2026-10-30.md)；[上一轮报告](../docs/AVI_SYNC_REPORT.md) 保留作为历史。

IMU 初始化参数现在可在相机 YAML 中配置：`IMU.fastInit: 0`、`IMU.VIBA1Time: 5.0`、`IMU.VIBA2Time: 15.0`。
新旧读取路径修正、参数含义与验证见 [IMU_INITIALIZATION_CONFIG.md](IMU_INITIALIZATION_CONFIG.md)。
初始化时间和处理帧率的重复回放测试见 [REPLAY_BENCHMARK.md](REPLAY_BENCHMARK.md)。

当前导航坐标系已改为首关键帧时刻的 IMU 坐标系，位姿、地图显示和读档偏移统一消除整体刚体变换。
接口、公式和初始点定义见 [NAVIGATION_FRAME.md](NAVIGATION_FRAME.md)。下文 2026-09-18 的数值是修改前的历史记录。

## 一键启动测试

新增实机无 GUI 和终端阶段指令：`./run_recovery_test.sh --headless`。
Windows SSH 图形转发使用 `--config config/recovery_ssh_windows.conf`。
命令、配置与连接步骤见 [REMOTE_AND_CONSOLE.md](REMOTE_AND_CONSOLE.md)。

在桌面终端执行，默认连接 D435i 并打开交互 GUI：

```bash
~/ORB_SLAM3/run_recovery_test.sh
```

按 `First Init Done → Start Second Map → Second Init Done → Read First Map` 操作。
两次初始化都要实际看到 `end VIBA 2`；首次存图成功后再重置。
读档后检查 `FILE LOAD SUCCESS`、`ALIGNMENT SUCCESS`、相对原点偏移和可信度，结束时点击 `Stop`。

```bash
~/ORB_SLAM3/run_recovery_test.sh --debug           # GDB 启动，保留异常回溯
~/ORB_SLAM3/run_recovery_test.sh --headless        # 实机无 GUI；first/reset/second/load
~/ORB_SLAM3/run_recovery_test.sh --replay          # 已录制数据 + GUI，自动走完各阶段
~/ORB_SLAM3/run_recovery_test.sh --replay --headless # 无 GUI 的离线回放
~/ORB_SLAM3/run_recovery_test.sh --check           # 仅检查文件/动态库，不启动相机
~/ORB_SLAM3/run_recovery_test.sh --build           # 先编译再启动
```

脚本可从任意目录执行；每次在 `recordings/test_runs/` 创建独立目录，包含 `run.log`、
`launch.txt`、配置副本和地图存档。实机按钮存档为该目录中的 `orbslam3_first_map.osa`，
离线回放还会生成 `trace.csv` 和对齐成功时的 `recovery_report.csv`。
此脚本不录制新的传感器包，也不设置实机测试的结束时长。
它仅对自身进程排除 `/opt/MVS/` 动态库搜索路径，避免海康 SDK 的 libusb 与 RealSense 冲突。
`--check` 不代表相机连接、初始化或恢复流程已经通过；离线测试仍可能因输入运动不足而失败退出。

2026-10-23 启动脚本验证：从 `/tmp` 执行 `--replay --headless --debug`，完成两次 VIBA 2、
存图、重置、磁盘读档与对齐，输出 `RECOVERY_REPLAY_PASS`，脚本退出码为 0。
日志保存在 `recordings/test_runs/20261023_153042_replay_pCfhCL/run.log`。此验证没有启动实机相机。

`First Init Done` 的保存路径曾在清理过期观测时触发段错误。
离线复现的 GDB 回溯为：

```text
SIGSEGV in std::_Rb_tree_increment
ORB_SLAM3::Map::PreSave
ORB_SLAM3::Atlas::PreSave
```

`EraseObservation()` 可能调用 `SetBadFlag()`，从正在遍历的地图点集合中删除当前元素。
`MapPoint::PreSave()` 也曾在遍历观测集合时删除当前观测。
修复改为遍历副本，并处理删除最后一个观测后引用关键帧为空的情况。

在线保存由 `RequestAtlasSave()` 排队，在 `TrackStereo()` 的帧边界执行。
先暂停回环线程并等待已有 GBA 结束，再暂停局部建图。
保存结束或失败后恢复线程，恢复时保留排队的关键帧。
Viewer 轮询状态，不阻塞等待，因此仍可响应停止和重置。
只有跟踪正常且 VIBA 2 完成的地图才能确认保存；失败后可重新点击。
文件先写入同目录临时文件，写入和关闭成功后替换目标 `.osa`，避免失败时毁掉旧档。
按钮保存的路径不会成为 Shutdown 自动保存路径。

## 编译与测试

在仓库根目录执行：

```bash
cmake -S . -B build -DORB_SLAM3_BUILD_TESTS=ON
cmake --build build --target stereo_inertial_realsense_D435i atlas_save_regression stereo_inertial_save_replay -j2
ctest --test-dir build --output-on-failure -R atlas_save_regression
./build/tests/stereo_inertial_save_replay Vocabulary/ORBvoc.txt /tmp/orb-atlas-save-test
```

`atlas_save_regression` 验证：

- 重复存取 5 次后关键帧、地图点、描述子、观测关系和 IMU 预积分保持一致。
- 清理过期观测不会使地图点迭代器失效。
- 清理最后一个观测不会解引用空集合。
- 空的当前地图不会因保存而被标为坏地图。

`stereo_inertial_save_replay` 生成 640×480 双目图像（30 Hz、基线 5 cm）及相应的 200 Hz IMU 测量，
使用真实 System、Tracking、LocalMapping 和 LoopClosing 线程，不使用相机或外部数据集。
它验证实际完成 VIBA 1/2 后连续保存 5 次，拒绝重复排队请求，写入失败后可重试，
继续跟踪和建图后退出不会覆盖已保存文件，并反序列化检查地图、位姿和 IMU 数据。
运行需要词典，通常需要约一分钟；地图点数量会随线程调度变化。
这是合成场景测试，不能替代 D435i 实机验证。

2026-09-18 验证结果：上述原崩溃用例修复后正常退出，回归测试和完整回放均通过。
最终版本的回放日志摘录（数量只表示该次运行）：

```text
end VIBA 1
end VIBA 2
REPLAY_SAVE_OK 1 map=0 KFs=54 MPs=1983
Atlas save failed: Cannot create archive: No such file or directory
REPLAY_SAVE_OK 2 map=0 KFs=58 MPs=2058
REPLAY_SAVE_OK 3 map=0 KFs=60 MPs=2108
REPLAY_SAVE_OK 4 map=0 KFs=62 MPs=2136
REPLAY_SAVE_OK 5 map=0 KFs=64 MPs=2151
Shutdown
PASS: five online saves after VIBA 2, failure/retry, tracking resumed, archive readback; early attempts=0
```

## D435i 实机验证记录（2026-09-18）

使用已重新编译的 `stereo_inertial_realsense_D435i` 和 `RealSense_D435i.yaml`，
在 GDB 下接收真实 IR1/IR2 和 IMU 数据。用户提供运动激励后，日志出现 `end VIBA 1`、
`end VIBA 2`；随后用户点击了 `First Init Done`。

```text
Atlas save succeeded: orbslam3_first_map.osa
First initialization accepted: map 0, 54 KFs, 1249 MPs. Atlas save succeeded: orbslam3_first_map.osa
```

存档大小为 13,420,539 字节。独立进程从磁盘反序列化并执行 `PostLoad()` 后得到：

```text
Map 0: KFs=54 MPs=1249 IMU=1 VIBA2=1
ARCHIVE_READBACK_OK maps=1 KFs=54 MPs=1249 vocabulary=ORBvoc.txt checksum=5420bad0713bc97034dd2a9b2f0cc387
```

检查了关键帧位姿、描述子、地图点坐标、引用关键帧、观测和 IMU 预积分；
词典校验和与本地 `ORBvoc.txt` 一致。保存后程序继续运行，观察期间未出现新的段错误或跟踪失败日志。
本次运行在 `/tmp/orb-d435i-live/run.log` 留下日志，读回结果在 `/tmp/orb-d435i-live/readback.log`。
当时的独立读回检查尚未验证 Viewer 的 `Read First Map`；下面记录后续的磁盘恢复实现与测试。

## 实机复测

```bash
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i \
  Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/RealSense_D435i.yaml d435i_test
```

运动激励完成，看到 `end VIBA 2` 且跟踪正常后点击 `First Init Done`。
`Save Status` 显示保存状态，终端应输出 `Atlas save succeeded: orbslam3_first_map.osa`。
成功后才可进入 `Start Second Map`；失败会显示原因并允许再次点击。

如果还有崩溃，可用下面的命令保留所有线程的回溯：

```bash
gdb -q -batch -ex 'set pagination off' -ex run -ex 'thread apply all bt 12' \
  --args ./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i \
  Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/RealSense_D435i.yaml d435i_test \
  > /tmp/orb-d435i-save-gdb.log 2>&1
```

后续恢复实现已改为独立读取首次存档并进行几何验证，详见下面的离线恢复测试。
此前“存档读回成功”仅验证序列化完整性，不能作为恢复定位成功的证据。

## 录制两次真实初始化的输入

`d435i_capture_slam.cc` 用于录制 IR1/IR2、200 Hz 陀螺仪和加速度计，同时运行真实 SLAM。
参数为 `vocabulary settings output_directory`，输出目录中不能已有 `session.db3`。
目前本机 librealsense 2.58 使用 ROS2 `.db3` 格式。
录制开始须人工确认已准备好运动激励；结束条件取决于 SLAM 结果：

1. 实际完成第一次 VIBA 2，保存 `first.osa` 成功。
2. 调用全量 `System::Reset()`，清空当前 Atlas 后重新初始化。
3. 实际完成第二次 VIBA 2，保存 `second.osa` 成功。
4. 继续保留约 10 秒场景重叠，然后正常关闭录制文件。

`slam_events.csv` 记录传感器时间和真实存图/Reset事件，`CAPTURE_COMPLETE` 表示上述条件完成。
若需提前停止，在输出目录创建 `STOP` 文件；这将标记为 `CAPTURE_INCOMPLETE`。
这份输入用于离线复现 GUI 恢复问题；录制完成本身不代表 GUI 读档已成功。

`d435i_export_recording.py` 使用已安装的 ROS2 消息库从原始 `.db3` 导出 stereo/gyro/accel CSV 和 PNG：

```bash
source /opt/ros/humble/setup.bash
python3 tests/d435i_export_recording.py /path/to/session.db3 /path/to/export
```

导出使用 SDK 元数据里的传感器时间，不将数据库写入时间当成 IMU 时间。
原始文件完整保留；初始化时 SDK 可能输出早于第一帧的旧陀螺仪时间戳，回放时应舍弃不覆盖图像时间的前导样本。
`recordings/` 不加入 Git，以免意外提交数 GB 的原始图像数据。

## 离线恢复测试（真实 D435i 输入）

有效数据位于 `recordings/d435i_recovery_verified_20260918_1654/`。
第一次录制已实际完成 VIBA 1/2 并保存，但随后的全量 Reset 崩溃；第二次使用新进程补录，
也实际完成 VIBA 1/2 和保存。`manifest.json` 保留这一事实，不将两段录制描述成一次成功的恢复过程。
`combined/` 按传感器时间重放两段输入，移除图像开始之前的无效前导 gyro 样本，保留原始 `.db3`。

```bash
cmake -S . -B build -DORB_SLAM3_BUILD_TESTS=ON
cmake --build build --target d435i_recovery_replay stereo_inertial_realsense_D435i -j2
ORB_REPLAY_VIEWER=1 ORB_REPLAY_STRIDE=2 \
  ./build/tests/d435i_recovery_replay Vocabulary/ORBvoc.txt \
  recordings/d435i_recovery_verified_20260918_1654/settings.yaml \
  recordings/d435i_recovery_verified_20260918_1654/combined \
  /tmp/orb-recovery-replay
```

设置 `ORB_REPLAY_VIEWER=1` 可同时运行 GUI；测试通过按钮使用的同一 System API 自动提交各阶段操作，无需相机或人工按键。
`ORB_REPLAY_STRIDE=2` 每两对图像使用一对，保留期间全部 IMU 样本；原始数据不变。
不设置这两个环境变量则逐帧运行无 GUI 回放。
回放日程对应本次录制：先等待 VIBA 2 存图，在两段输入衔接时执行全量重置，再确认第二次初始化并读档。
若第一段结束时尚未完成初始化并存图，测试直接失败，不能借第二段的运动激励完成第一次初始化。

2026-09-18 真实数据回放结果：

- 无 GUI、逐帧回放通过：首次存图在 79.278 s；重置后 KFs/MPs 均为 0；第二次 VIBA 2 后磁盘读取和对齐成功。
  对齐时 52 个 PnP 内点、23 个 3D 点对、点对 RMSE 0.096 m；后续约 9.8 s 保持正常跟踪。
- GUI、图像步长 2 的回放通过：首次存图在 82.447 s；重置、第二次 VIBA 2、磁盘读取和对齐依次完成。
  对齐时 172 个 PnP 内点、85 个 3D 点对、点对 RMSE 0.108 m；后续 140 帧（约 9.3 s）跟踪状态均为 OK。
  GDB 输出 `RECOVERY_REPLAY_PASS`、`exited normally`，退出时未发生 Viewer 卡死。
- 一次 GUI 逐帧回放未能在第一段结束前完成 VIBA 2，不能算完整流程通过；失败日志同样保留。
  这份录制的初始化对运动量和线程调度敏感，不能保证每次回放都成功初始化。

点对残差不是绝对定位精度，录制没有地面真值。离线测试覆盖 GUI 线程及按钮调用的 API，
不等于重新进行一次人工点击的实机验收。详细输入、原始崩溃回溯和通过/失败日志见录制目录的 `README.md` 与 `validation/`。

实现约定：

- `Start Second Map` 执行全量重置，原子替换 Atlas 当前地图并清除旧运动模型和 IMU 积分状态。
- `Second Init Done` 仅在第二次跟踪正常、VIBA 2 完成时确认，输出第二张地图的信息。
- `Read First Map` 真正读取 `.osa`，核对词典校验和、地图 ID 与保存时的关键帧/点数；失败保留第二张地图。
- 首次地图保持只读。通过旧地图的 PnP 重定位和第二张稀疏图的 3D 点配对，估计第二坐标系到首次坐标系的 SE(3)。
- 至少 50 个 PnP 内点、20 个有效 3D 点对、点对 RMSE 小于 0.15 m，且连续两次变换相近，才输出 `ALIGNMENT SUCCESS`。
- 恢复后仍在第二张地图进行双目惯性跟踪；`GetCurrentPose()` 和 Viewer 显示变换到首次地图坐标系的位姿，
  蓝色点是首次参考地图。当前输出参考系已更新为首次初始 IMU，详见 NAVIGATION_FRAME.md。
  `TrackStereo()` 的原始返回值仍是当前 Atlas 坐标系的 `Tcw`，两者不要混用。
- 文件读取成功和坐标对齐成功是不同状态；不把点数相同当作几何匹配成功。
- 这是参考地图坐标恢复，不执行两张地图的联合 BA 或合并优化。首次对齐后仍可能出现跟踪漂移。

最终 CMake 可执行文件也以 GUI、图像步长 2 再次回放通过，日志为录制目录的 `validation/final-gui-pass.log`：
184 个 PnP 内点、86 个 3D 点对、点对 RMSE 0.081 m；对齐后 138 帧（约 9.1 s）持续跟踪，正常退出。

## 读档偏移与可信度

`Read First Map` 完成文件读取后，还需要几何对齐通过，才会输出偏移与可信度。
等待匹配、读取失败或地图重置时显示 `N/A`，不会把未知偏移显示成有效的零距离。
对齐成功时终端输出三行，同时显示在 Pangolin 菜单和 Current Frame 图像上：

- `CAMERA OFFSET FROM FIRST ORIGIN` / `At load in FIRST`：对齐完成瞬间，相机相对于首次地图原点的 X/Y/Z 和直线距离，单位米。
- `SECOND ORIGIN OFFSET FROM FIRST ORIGIN` / `Map2 origin in FIRST`：第二张地图原点在首次地图中的 X/Y/Z 和距离。
- `LOAD CONFIDENCE`：0–100 分的匹配质量评分，以及 PnP 内点数、3D 点对数/候选数、点对 RMSE、连续两次变换的平移/旋转差。

令 `T12` 为第二地图到第一地图的变换，`T2c` 为相机到第二地图的变换：

```text
第二地图原点偏移 = T12.translation
读档时相机偏移   = (T12 * T2c).translation
距离             = sqrt(x*x + y*y + z*z)
```

坐标轴当前使用首次地图首关键帧时刻的 IMU 三轴，原点为该 IMU 的位置，不自动解释为东/北/天。
`IMU at Load`、`Camera at Load` 和可信度是对齐时的快照；`Current Pose` / `IMU XYZ` 则继续更新当前 IMU 位置。
文件刚读完但尚未匹配到旧地图时，无法给出有效的首次地图坐标。
通过 `GetRecoveryStatus().report` 可读取完整快照；只在 `stage == ALIGNED && report.valid` 时使用。
`GetCurrentPose()` 仍用于读取当前相机在对齐后的坐标系中的位姿。

可信度采用明确的工程评分，并非经过真值标定的成功概率，也不是后续跟踪的实时置信度。
令 `I` 为 PnP 内点数，`P` 为距离残差小于 0.25 m 的有效 3D 点对数，`C` 为过滤前的候选点对数，
`E` 为有效点对 RMSE，`dt/dr` 为连续两次估计的平移差（米）和旋转差（弧度）：

```text
score = 100 * (
    0.20 * min(I / 150, 1)
  + 0.15 * min(P / 60, 1)
  + 0.25 * (P / C)
  + 0.25 * (1 - (E / 0.15)^2)
  + 0.15 * exp(-0.5 * ((dt / 0.15)^2 + (dr / 0.10)^2))
)
```

只在原有对齐门槛通过时评分：至少 50 个 PnP 内点、20 个有效点对、RMSE < 0.15 m，
连续两次估计的平移差 < 0.15 m、旋转差 < 0.10 rad；所有数值必须有限，候选点对数必须有效。
分数限制在 0–100。过滤前的候选数也参与评分，避免大量点被剔除之后仅凭较小残差得到虚高分数。
这些权重和归一化常量是启发式参数，尚未用不同环境及地面真值校准。

验证命令：

```bash
cmake --build build --target recovery_report_regression d435i_recovery_replay stereo_inertial_realsense_D435i -j2
ctest --test-dir build --output-on-failure -R 'recovery_report_regression|atlas_save_regression'
```

`recovery_report_regression` 检查旋转和平移组合后的相机坐标、地图原点坐标、距离、无效输入拒绝，
以及残差/异常点比例/变换不一致增加时评分下降。
真实数据回放还检查报告中的相机偏移与对齐帧的 `GetCurrentPose()` 一致，并输出 `recovery_report.csv`。

2026-09-18 实测：新增回归测试与存档测试 2/2 通过，全工程编译通过。带 GUI 的真实录制回放输出相机偏移 `(0.089, 0.139, 0.036) m`、距离 `0.169 m`，可信度评分 `84.8/100`；
报告与同帧公开位姿一致，对齐后 121 帧持续正常跟踪并退出。第一轮初始化未完成的失败记录也保留。
详细结果在 `recordings/d435i_recovery_verified_20260918_1654/validation/recovery-offset-confidence/README.md`。

## PX4 1.17 通信

ROS 2 / uXRCE-DDS / 原生 USB 接入见 [PX4_VIO.md](PX4_VIO.md)，已有验证记录见 [PX4_VALIDATION.md](PX4_VALIDATION.md)。
