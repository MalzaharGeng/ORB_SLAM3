# avi 功能同步与现代构建保留记录

执行日期：2026-10-28（Asia/Shanghai）。

## 来源

- 原始上游：[UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)，基准 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。
- 本次参考与合并来源：[Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3)，`master` 固定为 `d4e700981e3cf43ec323606fe4a7282b9d03590b`。
- 用户分支：[MalzaharGeng/ORB_SLAM3](https://github.com/MalzaharGeng/ORB_SLAM3)，在既有本地构建适配提交上进行真实 Git 合并，保留 avi 提交和作者。

## 合并结果

已接入 avi 的在线地图保存、双次初始化恢复流程、独立旧图加载、几何对齐、初始 IMU 导航输出、D435i 终端/无 GUI 控制及其测试工具。GPLv3、官方作者和第三方许可证保留。

同时保留本地根 CMake 3.13 / DBoW2 和 g2o 3.10、标准属性、目标式依赖链接、统一构建图、构建树产物、可配置并行数、Linux RPATH 兼容和 Viewer 收尾。

| 适配点 | 最终行为 |
| --- | --- |
| C++ | C++17、标准必须满足、禁用 GNU 扩展；保留 `COMPILEDWITHC11` 宏 |
| Shutdown | 请求 Viewer 结束并避免自身 join；join 建图/回环、等待 GBA；未处理的排队保存标为失败；静止后保存 Atlas |
| Viewer | 暂停时检查结束；退出线程显式销毁 OpenCV 和 Pangolin 窗口 |
| 启动脚本 | 可执行文件和核心库从 `BUILD_DIR` 查找，默认 build/；`BUILD_JOBS` 默认 2；回放输入可由 `RECORDING_DIR` 指定 |
| 测试 | 保留 avi 三个 CTest 和两个离线回放目标；新增启动路径回归 |

恢复报告的 confidence 是工程评分，不是概率。`FILE LOAD SUCCESS` 不等于 `ALIGNMENT SUCCESS`。导航 API 与 `TrackStereo()` 原有 Tcw 返回值采用不同的坐标约定，见 [导航说明](../tests/NAVIGATION_FRAME.md)。

## 验证

本节在实际构建和测试后记录最终结果；不把 avi 的历史测试文档当作当前机器的测试结果。
