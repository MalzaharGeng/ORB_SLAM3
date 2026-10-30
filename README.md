# ORB-SLAM3 — 现代构建与地图恢复

本仓库是 [MalzaharGeng](https://github.com/MalzaharGeng) 维护的 ORB-SLAM3 学习与适配分支，直接基于 [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)。保留官方算法、示例、作者信息、提交历史及 GPLv3 许可证，保留本地现代 CMake 构建和线程退出修复，并同步 avi 的地图恢复、导航位姿和终端控制功能。

> **上游来源**：UZ-SLAMLab/ORB_SLAM3，`master`，基准提交 [`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`](https://github.com/UZ-SLAMLab/ORB_SLAM3/commit/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4)。
>
> **原作者**：Carlos Campos、Richard Elvira、Juan J. Gómez Rodríguez、José M. M. Montiel、Juan D. Tardós。ORB-SLAM3 的算法及论文成果归原作者所有。
>
> **功能参考与合并来源（avi）**：[Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3)，同步提交 [`6b5d18ed0e17755a2b51c4d0538e79cb73233aa3`](https://github.com/Kuroneko-avi/ORB_SLAM3/commit/6b5d18ed0e17755a2b51c4d0538e79cb73233aa3)。保留 avi 原始提交及作者，集成其在线存图、重置后恢复、初始 IMU 导航、D435i 终端/无 GUI 控制和回归测试。

## 本分支改动

| 方面 | 改动 |
| --- | --- |
| 工具链 | C++17，关闭 GNU 扩展；根 CMake 最低 3.16，DBoW2/g2o 最低 3.10 |
| 构建依赖 | 将 DBoW2 纳入根 CMake 构建图，按目标链接 DBoW2/g2o |
| 输出目录 | 核心库、第三方库和示例程序放入构建目录 |
| 构建脚本 | 严格错误处理；支持 `BUILD_DIR`、`BUILD_JOBS` 和额外 CMake 参数 |
| Linux 动态库 | 对 Pangolin 间接依赖采用传统 RPATH 链接兼容方式 |
| 退出流程 | 等待 Viewer、建图、回环线程及 GBA，允许暂停中的 Viewer 响应结束请求并销毁窗口 |
| avi 功能 | 在线存图、独立加载旧图、几何对齐、导航 API、GUI/终端控制和测试 |
| 文档与编辑器 | 上游说明、本地变更说明、历史构建验证报告，以及 VS Code C++17 配置 |

本轮新增 IMU 初始化时间配置、导航速度/重力/参考变化字段、可选 PX4 VIO 桥接和离线转换测试。PX4 默认关闭；启用需兼容的 ROS 2、PX4 1.17 `px4_msgs` 和设备外参。部署脚本仅在显式调用时运行。本机验证范围见 [2026-10-30 同步报告](docs/AVI_SYNC_2026-10-30.md)。

具体文件变化见 [本地修改说明](docs/LOCAL_CHANGES.md)。官方功能、算法介绍、论文和历史使用说明见 [上游 README 原文](docs/UPSTREAM_README.md)。

## 依赖

- Linux，支持 C++17 的 GCC/Clang。
- CMake **3.16 或更新版本**。
- OpenCV **4.4 或更新版本**，Eigen3 **3.1 或更新版本**。
- Pangolin；本地历史验证使用 0.9.6。所安装 Pangolin 的具体版本仍需兼容 C++17。
- Boost.Serialization 和 OpenSSL 开发库。
- DBoW2、g2o、Sophus 源码已包含在 `Thirdparty/`。
- RealSense SDK、ROS1 仅供相应可选示例使用，普通数据集示例无需安装它们。

依赖的来源及许可证见 [Dependencies.md](Dependencies.md)。可选 PX4 用户目录依赖构建脚本见 `scripts/setup_px4_dependencies.py`；普通构建不会调用它。

## 编译

```bash
git clone https://github.com/MalzaharGeng/ORB_SLAM3.git
cd ORB_SLAM3
BUILD_JOBS=2 bash ./build.sh -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Pangolin 安装在自定义前缀时，显式指定路径：

```bash
BUILD_JOBS=2 bash ./build.sh \
  -DCMAKE_PREFIX_PATH="$HOME/local/pangolin-0.9.6" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

可选择独立构建目录：

```bash
BUILD_DIR="$PWD/build_local" BUILD_JOBS=2 bash ./build.sh \
  -DCMAKE_PREFIX_PATH="$HOME/local/pangolin-0.9.6"
```

`BUILD_JOBS` 默认 2；`BUILD_DIR` 默认仓库内 `build/`。脚本会在词典文本尚未存在时解压仓库提供的词典，不会下载数据集。使用其他构建目录后，请同步调整下面示例中的 `build` 前缀。

主要产物：

```text
build/lib/libORB_SLAM3.so
build/Thirdparty/DBoW2/lib/libDBoW2.so
build/Thirdparty/g2o/lib/libg2o.so
build/Examples/Monocular/mono_tum
build/Examples/RGB-D/rgbd_tum
build/Examples/Stereo-Inertial/stereo_inertial_euroc
```

## 运行示例

从仓库根目录运行，先把占位路径替换为实际数据集路径：

```bash
# TUM RGB-D；关联文件需要与该序列对应。
./build/Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml \
  /path/to/tum_sequence /path/to/associations.txt

# TUM 单目。
./build/Examples/Monocular/mono_tum \
  Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml /path/to/tum_sequence
```

数据集说明和更多传感器示例见 [上游文档](docs/UPSTREAM_README.md)。上游示例命令使用源码树中的程序路径；**本分支程序位于构建树中**，应相应加上 `build/` 等实际构建目录前缀。RealSense 示例只有在 CMake 找到 SDK 后才会生成。

## 地图恢复与测试

先启用测试并编译（自定义 Pangolin 路径按实际情况填写）：

```bash
BUILD_DIR="$PWD/build_sync" BUILD_JOBS=2 bash ./build.sh \
  -DCMAKE_PREFIX_PATH="$HOME/local/pangolin-0.9.6" \
  -DORB_SLAM3_BUILD_TESTS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ctest --test-dir build_sync --output-on-failure
./build_sync/tests/stereo_inertial_save_replay Vocabulary/ORBvoc.txt /tmp/orb-save-smoke
```

实机 D435i 恢复流程（需要已安装 SDK，并已编译对应示例）：

```bash
BUILD_DIR="$PWD/build_sync" ./run_recovery_test.sh --headless
# 依次完成：first → reset → second → load；status/pose 查看结果，quit 退出。
```

启动脚本和构建脚本共用 `BUILD_DIR`（默认 `build/`），不再从源码树寻找可执行文件。离线回放可用 `RECORDING_DIR` 选择外部录制目录；录制数据不随 Git 分发。`--headless` 关闭窗口，仍需要 Pangolin 构建依赖。

位姿和坐标约定见 [导航说明](tests/NAVIGATION_FRAME.md)，操作流程见 [avi 测试说明](tests/README.md) 及 [终端/SSH 文档](tests/REMOTE_AND_CONSOLE.md)。`FILE LOAD SUCCESS` 仅表示读入文件，`ALIGNMENT SUCCESS` 才表示几何对齐通过；评分是工程指标，不是统计概率。

2026-10-30 的合并、保留内容和验证结果见 [本轮 avi 同步报告](docs/AVI_SYNC_2026-10-30.md)；[上一轮报告](docs/AVI_SYNC_REPORT.md) 保留作为历史。以下报告为同步前的历史记录：

## 同步前验证记录（C++14 历史）

[2026-10-27 本地构建与运行报告](ORB_SLAM3_MODERN_BUILD_REPORT.md) 记录了以下历史结果：

- 核心库、DBoW2/g2o 和 24 个普通数据集示例编译、链接通过。
- 核心库与示例的动态依赖、示例 Usage 启动检查通过。
- 原 `rgbd_tum` 配合 40 帧合成 RGB-D 和 Viewer 运行，正常退出。

这些是同步前的本机结果，不是 GitHub CI 结果，也不构成真实传感器数据集精度验证。当时尚未验证 IMU、地图保存恢复等路径；2026-10-30 本轮的实际测试结果以 [同步报告](docs/AVI_SYNC_2026-10-30.md) 为准。RealSense 实机、ROS1、真实回环和复杂并发关闭仍需单独验证。

该 C++14 历史报告中 `build_codex/audit/` 下的日志、生成器、检查器和合成数据保存在原本机工作区，未作为源码上传。该报告中的审计/回滚命令依赖本机资料，不应在全新克隆中直接照搬。

## 目录

```text
Examples/                         官方示例与配置
Examples_old/                     官方旧版示例
Thirdparty/                       随上游提供的依赖源码
Vocabulary/                       原始压缩词典
include/、src/                    ORB-SLAM3 接口与实现
tests/、config/、run_recovery_test.sh avi 恢复测试与启动配置
docs/AVI_SYNC_REPORT.md             2026-10-28 同步历史
docs/AVI_SYNC_2026-10-30.md          本轮同步与验证
docs/LOCAL_CHANGES.md              本分支修改范围及边界
docs/UPSTREAM_README.md            原始上游 README
ORB_SLAM3_MODERN_BUILD_REPORT.md   本地历史构建验证报告
.vscode/c_cpp_properties.json      可调整的编辑器配置
```

不上传构建目录、生成库、解压词典、运行日志、数据集和 CMake 备份。VS Code 配置中的 Pangolin 路径以 `${env:HOME}/local/pangolin-0.9.6/include` 为示例，按实际安装位置调整。

## 许可证与引用

本仓库遵循上游 [GNU GPL v3](LICENSE)，各第三方组件沿用其自身许可证。请保留原作者及第三方版权声明。

在学术工作中使用 ORB-SLAM3 时，请引用原论文：

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map {SLAM}},
  author={Campos, Carlos and Elvira, Richard and G\'omez, Juan J. and Montiel, Jos\'e M. M. and Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={37},
  number={6},
  pages={1874--1890},
  year={2021}
}
```

更多相关论文见 [原始上游 README](docs/UPSTREAM_README.md)。本分支的构建适配不代表原作者对修改内容提供维护或背书。
