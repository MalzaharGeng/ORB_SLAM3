# 回放初始化时间与帧率测试

```bash
cmake --build build --target d435i_recovery_replay -j2
python3 tests/benchmark_recovery.py recordings/test_runs/my_benchmark \
  --profiles 5:15,3:10,2:5 --repeats 3 --stride 1
python3 tests/report_recovery_benchmark.py recordings/test_runs/my_benchmark
```

三组配置依次交错运行，每组重复三次；不会并行运行 SLAM。默认无 GUI、无 GDB，
640×480 完整双目输入，保留全部 IMU，fastInit=0，按原始传感器帧间隔回放。
程序计算慢时不会丢帧追赶；实际回放帧率可能低于录制帧率。
`--viewer` 开启 GUI；`--unpaced` 不等待帧间隔，用于额外吞吐压力测试。
无等待测试会改变建图线程获得的处理时间和初始化行为，不能直接当作正常实时 SLAM 的可靠帧率。
使用新输出目录，避免覆盖已有试验。

结果分别统计：

- 从每段输入开始到 VIBA 2 完成的墙钟时间，包含中途重试。
- 最终成功地图从首关键帧到 VIBA 1/2 完成的墙钟时间，以及触发时的 mTinit。
- VIBA 调用内部耗时，包含优化、地图更新和内部等待。
- 图像 PNG 读取、TrackStereo、整帧处理、回放等待和周期耗时。
- 整段实际 FPS，以及初始化后正常跟踪帧的平均/P95/P99 耗时。
- 失败、提前退出、初始化重置及几何匹配结果。

`TrackStereo` 计时围绕真实函数调用，包含其中的锁等待；内部异步工作线程的总 CPU 时间不等于这个耗时。
正常跟踪样本剔除与初始化调用重叠的帧，以及存图/重置/加载/匹配阶段。
实际 FPS 排除词典加载和进程关闭时间；词典/系统构造时间单独保存在 timing_metadata.csv。
保存和读档的停顿仍会体现在整段实际 FPS 与全帧延迟中。

`ORB_INIT_METRICS=/absolute/path/initialization.csv` 开启初始化诊断，正常实机未设置时不写该文件。
回放器额外输出 timing.csv、timing_metadata.csv；原有导航/恢复结果文件保留。
输入为两次真实录制拼接的测试时间线，不能描述成一次原始成功坠机恢复录制。

这里只能测试代码完成初始化及后续跟踪/匹配的情况；没有地面真值，不能证明物理状态已经准确收敛，
也不能仅凭两段 VIBA 结束或匹配评分就断言某组阈值适合实际飞行。

2026-10-30 的三组各三次实测及 GUI/无等待补测保存于
[完整报告](../recordings/test_runs/replay_benchmark_20261030/REPORT.md)。
逐帧数据和每次设置均保留；GUI 未在第一段内完成初始化的试验也计入结果，不删除失败样本。
