# avi 功能合并与现代构建适配（2026-10-28）

## 来源

- 原始上游：[UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)，基准 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。
- 功能来源：[Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3)，`master` 固定为 `d4e700981e3cf43ec323606fe4a7282b9d03590b`。
- 用户分支：[MalzaharGeng/ORB_SLAM3](https://github.com/MalzaharGeng/ORB_SLAM3)，在既有本地构建适配提交上做真实 Git 合并，保留 avi 提交与作者。

## 合并结果

接入 avi 的在线地图保存、双次初始化恢复流程、独立旧图加载、几何对齐、初始 IMU 导航输出、D435i 终端/无 GUI 控制及其测试工具。GPLv3、官方作者与第三方许可证保留。

同时保留本地根 CMake 3.13、DBoW2 与 g2o 3.10、标准属性、目标式依赖链接、统一构建图、构建树产物、可配置并行数、Linux RPATH 兼容与 Viewer 收尾。

| 适配点 | 最终行为 |
| --- | --- |
| C++ | C++17、标准必须满足、禁用 GNU 扩展；保留 `COMPILEDWITHC11` 宏 |
| Shutdown | 请求 Viewer 结束并避免自身 join；join 建图/回环、等待 GBA；未处理的排队保存标为失败；静止后保存 Atlas |
| Viewer | 暂停时检查结束；退出线程显式销毁 OpenCV 与 Pangolin 窗口 |
| 启动脚本 | 可执行文件与核心库从 `BUILD_DIR` 查找，默认 build/；`BUILD_JOBS` 默认 2；回放输入由 `RECORDING_DIR` 指定 |
| 测试 | 保留 avi 三个 CTest 与两个离线回放目标；新增启动路径回归 |

恢复报告里的 confidence 是工程评分，不是概率；`FILE LOAD SUCCESS` 不等于 `ALIGNMENT SUCCESS`。导航 API 与 `TrackStereo()` 的 Tcw 返回值坐标约定不同，见 [导航说明](../tests/NAVIGATION_FRAME.md)。

## 验证

在独立构建目录中配置，使用现有 Pangolin 安装前缀、两路并行，不覆盖既有构建目录。

| 检查 | 结果 | 范围 |
| --- | --- | --- |
| 配置及全量编译链接 | 通过，退出 0 | 核心、DBoW2/g2o、24 个数据集示例、5 个测试/回放可执行文件 |
| C++ 标准 | 通过 | compile_commands 中 92 个 C++ 编译单元全部为 `-std=c++17` |
| CTest | 4/4 | 地图序列化、恢复报告、终端控制、启动路径 |
| 合成双目惯性回放 | 退出 0 | 完成 VIBA 1/2；连续 5 次在线保存；错误路径失败后重试；恢复跟踪；存档读回 |
| RGB-D + Viewer | 退出 0 | 40 帧本地合成图像，40 个有限位姿、3 个关键帧，窗口正常退出 |
| 脚本与文档 | 通过 | bash 语法、含空格与自定义目录的路径、相对链接、来源历史核对 |

合成 RGB-D 最大平移误差约 0.0059 m，仅用于该合成平面输入的冒烟断言。双目惯性用例中的无效路径写入失败是设计的一部分，其后成功完成重试。

```bash
BUILD_DIR="$PWD/build_sync" BUILD_JOBS=2 bash ./build.sh \
  -DCMAKE_PREFIX_PATH="$HOME/local/pangolin-0.9.6" \
  -DORB_SLAM3_BUILD_TESTS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
env -u LD_LIBRARY_PATH ctest --test-dir build_sync --output-on-failure --timeout 60
env -u LD_LIBRARY_PATH timeout 180 ./build_sync/tests/stereo_inertial_save_replay \
  "$PWD/Vocabulary/ORBvoc.txt" "$PWD/build_sync/stereo_save"
```

构建日志与位姿检查结果留在本地构建目录，不入库。

## 边界

- 未安装 RealSense SDK，D435i 实机与录制程序未构建、未运行。
- 缺 avi 文档中的真实录制输入，完整的「重置—第二次初始化—旧图加载—几何对齐」真实数据回放未执行。
- SSH X11 跨主机显示、ROS1 与真实场景精度未验证。
- GUI 正常退出后仍有 Qt 跨线程计时器警告，其他图形后端与菜单并发退出的情况未覆盖。
- 启动路径回归用只读检查与替代可执行文件验证路径发现，不代表传感器或 SLAM 已运行。
- `tests/README.md` 中 avi 的历史实机数据与结果保留并标注来源，不当作本机结果。

## 工作区与远端

合并前未提交的本地文件保存为独立备份目录与命名 Git stash，原 CMake 备份文件保留。本地 `origin` 指向自己的仓库，`upstream` 指向官方，`avi` 指向功能参考仓库。未改动系统依赖，未上传构建产物、日志、录制输入或认证凭证。
