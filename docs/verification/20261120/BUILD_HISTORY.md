# 构建与历史验证记录

## 验证

以下为 **2026-10-30 本分支在 Ubuntu WSL 中的检查结果**。Avi 文档中的历史实机与仿真记录单独保留，未计入本表。2026-11-06 本地 WSL 另有 `build/` 14/14（关闭 PX4 桥接）与 `build_local/` 17/17（开启 PX4 桥接，由 `~/px4_ws_201` 提供与 nano 同版的消息定义）两次 CTest 记录；两次 WSL 结果都不能替代设备端构建与实飞验收，环境差异见[运行环境与同步边界](#运行环境与同步边界)。

| 检查 | 结果 |
|---|---|
| 核心与示例构建 | 核心库、DBoW2/g2o、24 个普通数据集 Examples 构建通过；96 个 C++ 编译单元使用 C++17 |
| CTest | 7/7 通过，覆盖参数读取、Atlas 序列化、变换组合、终端控制、启动路径、无特征帧与离线 PX4 转换 |
| 合成双目惯性回放 | 完成 VIBA 1/2、5 次在线保存、写入失败重试、存档读回及重置状态检查 |
| 导航状态 | 检查 810 个有效快照及 736 个含速度快照的时间、参考系与数值一致性 |
| 合成 RGB-D + Viewer | 40 帧正常运行并退出；仍有 Qt 跨线程计时器警告 |

```bash
ctest --test-dir build --output-on-failure
./build/tests/stereo_inertial_save_replay \
  Vocabulary/ORBvoc.txt /tmp/orb-save-smoke
```
