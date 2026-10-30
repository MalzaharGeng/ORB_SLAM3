# 本地修改说明

本分支直接基于 [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) 的 `master` 提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，由 [MalzaharGeng](https://github.com/MalzaharGeng) 整理发布。

功能参考与合并来源：[Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3)，最新同步提交 `6b5d18ed0e17755a2b51c4d0538e79cb73233aa3`，保留 avi 原始提交及作者信息。2026-10-28 的历史基线为 `d4e7009`。

## 源码与构建修改

| 文件 | 修改及原因 |
| --- | --- |
| `CMakeLists.txt` | CMake 最低 3.16；C++17 标准属性；保留示例使用的 `COMPILEDWITHC11` 宏；DBoW2 纳入依赖图；目标方式链接 DBoW2/g2o；库与示例移到构建树；Linux 添加 `--disable-new-dtags` 以兼容 Pangolin 间接依赖搜索 |
| `Thirdparty/DBoW2/CMakeLists.txt` | CMake 最低 3.10，库输出到构建目录 |
| `Thirdparty/g2o/CMakeLists.txt` | CMake 最低 3.10，非 Windows 的默认库输出到构建目录 |
| `build.sh` | Bash 严格模式；按脚本位置定位源码；单一 CMake 构建图；按需解压词典；支持构建目录、并行数和额外 CMake 参数 |
| `src/System.cc` | 请求 Viewer 结束，外部调用时 join；join 局部建图和回环线程；等待活动 GBA |
| `src/Viewer.cc` | 暂停期间也检查结束请求；在 Viewer 线程内销毁 OpenCV/Pangolin 窗口 |
| `.gitignore` | 排除 `build_codex/`、本地备份和常用的额外构建目录 |

ORB 提取器、匹配器、优化器、相机模型及第三方算法源码未因本地适配而修改。公开接口已增加 avi 的在线地图保存、恢复状态和导航位姿 API。

## 构建与使用边界

- 保留 `COMPILEDWITHC11` 是为了兼容现有示例中的计时分支，不表示仍采用 C++11。
- 核心库、依赖库和示例的输出路径位于构建树；原版文档、外部脚本及调用方应调整程序和库路径。
- C++17 对原本机 Pangolin 安装有效；若所安装依赖要求更高标准，需要单独适配，不能假定所有 Pangolin 构建都兼容。
- Linux 传统 RPATH 兼容会改变动态库搜索优先级，适用于本机遇到的间接依赖问题。
- g2o 仍可能按上游方式在源码树生成被忽略的 `config.h`，未验证不同工具链同时生成该文件。
- 退出等待没有统一超时，不将它等同于已验证所有 GBA/菜单并发退出情形。

## 验证记录

完整历史记录见 [本地构建报告](../ORB_SLAM3_MODERN_BUILD_REPORT.md)。报告记录的是 2026-10-27 的本机实验，原始审计资料没有加入版本控制；GitHub 仓库当前未配置 CI。

已记录的运行输入是 40 帧合成 RGB-D，不能用于宣称真实数据集精度、真实回环、IMU 或 RealSense 实机流程通过。Qt 退出和 WSLg 软件渲染相关警告仍可能出现。

## 发布时的整理

- 保留官方提交历史、许可证、第三方声明、示例和压缩词典。
- 首页更新为本分支说明；原始首页保存在 [UPSTREAM_README.md](UPSTREAM_README.md)。
- 录入既有构建报告；VS Code Pangolin 路径由本机绝对路径改为 `${env:HOME}` 示例。
- 不录入二进制、构建缓存、审计日志、数据集、解压词典和 `CMakeLists.txt.backup_cpp11`。
- [Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3) 作为功能参考与合并来源；代码基点是官方仓库，本次已合并其功能提交并适配本地构建路径。

## 本次 avi 同步

- C++ 标准从本地 C++14 升至 C++17，保留 CMake 标准属性和历史 `COMPILEDWITHC11` 宏。
- 完整接入地图保存/恢复、初始 IMU 导航、地图及序列化修复、D435i 终端控制和测试源码。
- 保留本地第三方 CMake 最低版本、依赖目标链接、构建树输出和 Linux RPATH 兼容。
- 合并 Shutdown 取消排队保存请求的逻辑，保留 Viewer 暂停退出和窗口销毁。
- 启动脚本采用 `BUILD_DIR`、`BUILD_JOBS`；用 `RECORDING_DIR` 指定外部回放数据。
- 新增启动路径回归测试，覆盖空格目录、任意工作目录、自定义构建/录制目录和只读检查。

本次实测结果与尚未验证范围见 [AVI_SYNC_REPORT.md](AVI_SYNC_REPORT.md)。

## 2026-10-30 增量同步

合并 Avi 的可配置 IMU 初始化、导航速度/重力/参考 epoch、默认关闭的 PX4 bridge、回放计时与工具。
保留上述本地现代构建、关闭和路径修改。新增 benchmark 已适配构建树核心库与自定义构建/录制目录。
修复本轮实际复现的无特征 Frame 提前返回时 IMU 互斥量未初始化问题；仅提前现有分配位置，保留计算路径。
本轮 7/7 CTest、合成惯性保存/读回/导航/重置及 RGB-D Viewer 冒烟结果与边界见 [报告](AVI_SYNC_2026-10-30.md)。
