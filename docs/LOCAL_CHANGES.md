# 本地修改说明

本分支直接基于 [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) 的 `master` 提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，由 [MalzaharGeng](https://github.com/MalzaharGeng) 整理发布。

功能参考与合并来源：[Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3)，最新同步提交 `b9ba5ecf0a490e906ad8cd968d9dd42b39aa8d10`，保留 avi 原始提交及作者信息。2026-10-28 的历史基线为 `d4e7009`。

## 源码与构建修改

| 文件 | 修改及原因 |
| --- | --- |
| `CMakeLists.txt` | CMake 最低 3.16；C++17 标准属性；保留示例使用的 `COMPILEDWITHC11` 宏；DBoW2 纳入依赖图；目标方式链接 DBoW2/g2o；库与示例移到构建树；Linux 添加 `--disable-new-dtags` 以兼容 Pangolin 间接依赖搜索 |
| `Thirdparty/DBoW2/CMakeLists.txt` | CMake 最低 3.10，库输出到构建目录 |
| `Thirdparty/g2o/CMakeLists.txt` | CMake 最低 3.10，非 Windows 的默认库输出到构建目录 |
| `build.sh` | Bash 严格模式；按脚本位置定位源码；单一 CMake 构建图；按需解压词典；支持构建目录、并行数和额外 CMake 参数 |
| `src/System.cc` | 请求 Viewer 结束，外部调用时 join；join 局部建图和回环线程；等待活动 GBA |
| `src/Viewer.cc` | 暂停期间也检查结束请求；在 Viewer 线程内销毁 OpenCV/Pangolin 窗口 |
| `.gitignore` | 排除额外构建目录与本地备份文件 |

ORB 提取器、匹配器、优化器、相机模型及第三方算法源码未因本地适配而修改。公开接口已增加 avi 的在线地图保存、恢复状态和导航位姿 API。

## 构建与使用边界

- 保留 `COMPILEDWITHC11` 是为了兼容现有示例中的计时分支，不表示仍采用 C++11。
- 核心库、依赖库和示例的输出路径位于构建树；原版文档、外部脚本及调用方应调整程序和库路径。
- C++17 对原本机 Pangolin 安装有效；若所安装依赖要求更高标准，需要单独适配，不能假定所有 Pangolin 构建都兼容。
- Linux 传统 RPATH 兼容会改变动态库搜索优先级，适用于本机遇到的间接依赖问题。
- g2o 仍可能按上游方式在源码树生成被忽略的 `config.h`，未验证不同工具链同时生成该文件。
- 退出等待没有统一超时，GBA 与菜单并发退出的情形未逐一覆盖。

## 验证记录

2026-10-27 在本机做过一次从零配置的完整构建与合成 RGB-D 冒烟，结论是 24 个示例全部构建通过、核心库可被示例正确加载；审计日志当时留在本机，未纳入版本控制。GitHub 仓库未配置 CI。

2026-10-28 与 2026-10-30 两轮同步的实测范围见 [AVI_SYNC_REPORT.md](AVI_SYNC_REPORT.md) 与 [AVI_SYNC_2026-10-30.md](AVI_SYNC_2026-10-30.md)。已记录的运行输入都是短合成序列，不能用于宣称真实数据集精度、真实回环、IMU 或 RealSense 实机流程通过。Qt 退出与 WSLg 软件渲染相关警告仍可能出现。

## 发布时的整理

- 保留官方提交历史、许可证、第三方声明、示例和压缩词典。
- 首页更新为本分支说明；原始首页保存在 [UPSTREAM_README.md](UPSTREAM_README.md)。
- VS Code 中的 Pangolin 路径由本机绝对路径改为 `${env:HOME}` 示例。
- 不录入二进制、构建缓存、审计日志、数据集、解压词典和 `CMakeLists.txt.backup_cpp11`。
- [Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3) 作为功能参考与合并来源；代码基点是官方仓库，其功能提交已合入并适配本地构建路径。

## 后续增量同步

**2026-10-30**：合并 Avi 的可配置 IMU 初始化、导航速度/重力/参考 epoch、默认关闭的 PX4 bridge、回放计时与工具。保留本地现代构建、关闭和路径修改；benchmark 适配构建树核心库与自定义构建/录制目录。修复同步过程中复现的无特征 Frame 提前返回时 IMU 互斥量未初始化问题，只提前现有分配位置。详见 [记录](AVI_SYNC_2026-10-30.md)。

**2026-10-31**：Avi 只改 tests/README.md 并新增 tests/TEST_COMMANDS.md，无算法或构建源码变更，详见 [记录](AVI_SYNC_2026-10-31.md)。
