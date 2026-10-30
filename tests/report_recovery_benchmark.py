#!/usr/bin/env python3
"""Summarize replay benchmarks without interpreting completion as accuracy."""
import csv
import json
from pathlib import Path
import statistics
import sys


def median(values):
    return statistics.median(values) if values else None


def fmt(value, digits=2):
    return "—" if value is None else f"{value:.{digits}f}"


root = Path(sys.argv[1]).resolve()
results = json.loads((root / "results.json").read_text())
groups = {}
for r in results:
    key = (r["first_threshold"], r["second_threshold"])
    groups.setdefault(key, []).append(r)
table = []
for (first, second), trials in groups.items():
    phases = [p for r in trials for m in r["map_initializations"] for p in m["phases"]]
    first_phase = [p for p in phases if p["stage"] == 1]
    second_phase = [p for p in phases if p["stage"] == 2]
    regular = [r["regular_initialized_tracking"] for r in trials if r["regular_initialized_tracking"]["frames"]]
    table.append(dict(first=first, second=second, attempts=len(trials), passes=sum(r["pass"] for r in trials),
        completed_maps=sum(len(r["map_initializations"]) for r in trials),
        viba1_wall_from_anchor_median_s=median([p["wall_from_anchor_observed_s"] for p in first_phase if p["wall_from_anchor_observed_s"] is not None]),
        viba2_wall_from_anchor_median_s=median([p["wall_from_anchor_observed_s"] for p in second_phase if p["wall_from_anchor_observed_s"] is not None]),
        viba2_wall_from_anchor_min_s=min((p["wall_from_anchor_observed_s"] for p in second_phase if p["wall_from_anchor_observed_s"] is not None), default=None),
        viba2_wall_from_anchor_max_s=max((p["wall_from_anchor_observed_s"] for p in second_phase if p["wall_from_anchor_observed_s"] is not None), default=None),
        viba1_compute_median_ms=median([p["compute_ms"] for p in first_phase]),
        viba2_compute_median_ms=median([p["compute_ms"] for p in second_phase]),
        replay_fps_median=median([r.get("observed_replay_fps") for r in trials if r.get("observed_replay_fps")]),
        tracking_mean_ms_median=median([r["track_mean_ms"] for r in regular]),
        tracking_p95_ms_median=median([r["track_p95_ms"] for r in regular]),
        processing_mean_ms_median=median([r["processing_mean_ms"] for r in regular]),
        read_mean_ms_median=median([r["read_mean_ms"] for r in regular])))
(root / "aggregate.json").write_text(json.dumps(table, indent=2) + "\n")
with (root / "aggregate.csv").open("w") as stream:
    writer = csv.DictWriter(stream, fieldnames=list(table[0]));writer.writeheader();writer.writerows(table)

lines = ["# D435i 回放初始化时间与帧率", "",
         "本结果使用已有双目红外 + IMU 录制，fastInit=0，640×480，OpenCV 线程数为 2。各次试验顺序运行；配置、日志和逐帧计时均保留。",
         "VIBA 完成表示 InitializeIMU 返回成功；没有地面真值或独立收敛判据，因此不把完成时间表述为已验证的准确定位收敛时间。", "",
         "## 初始化时间", "",
         "以下时间以最终完成 VIBA 2 的有效地图首关键帧为起点，统计两张地图及重复试验的中位数。失败/重置的尝试不混入这些成功地图的时长，但完整流程成功率保留。",
         "VIBA 1/2 的完成时间都从同一个首关键帧起算。VIBA 调用耗时不包含此前等待运动和关键帧的时间，但包含调用内部的优化、地图更新和锁等待，非纯 CPU 运算时间。", "",
         "| VIBA 阈值（秒） | 完整恢复通过 | 完成 VIBA 2 的地图数 | VIBA 1 完成（秒） | VIBA 2 完成（秒） | VIBA 1 调用（ms） | VIBA 2 调用（ms） |",
         "|---|---:|---:|---:|---:|---:|---:|"]
for r in table:
    lines.append(f"| {r['first']:g}/{r['second']:g} | {r['passes']}/{r['attempts']} | {r['completed_maps']} | {fmt(r['viba1_wall_from_anchor_median_s'])} | {fmt(r['viba2_wall_from_anchor_median_s'])} | {fmt(r['viba1_compute_median_ms'])} | {fmt(r['viba2_compute_median_ms'])} |")
lines += ["", "VIBA 2 完成时间范围：" + "；".join(f"{r['first']:g}/{r['second']:g} 设置为 {fmt(r['viba2_wall_from_anchor_min_s'])}–{fmt(r['viba2_wall_from_anchor_max_s'])} 秒" for r in table) + "。",
          "阈值比较的是 mTinit：初始惯性求解后，只有满足平移运动条件的关键帧间隔才继续累计；它不是启动后的墙钟计时器。", ""]
lines += ["", "## 帧率", "", "每帧是一对双目图像及对应 IMU。实际回放 FPS 包括图像读取、TrackStereo、状态记录与回放等待，按整个处理区间统计。",
          "正常跟踪耗时只选 Tracking::OK、VIBA 2 完成的帧，排除初始化优化、存图、重置、读档和跨地图匹配；TrackStereo 内部等待工作线程的时间仍包含在内。",
          "1 / 平均 TrackStereo 耗时仅为该调用的等效处理率，不是已验证的整机实时上限。PNG 读取是离线测试开销，不代表相机采集开销。", "",
          "| VIBA 阈值 | 整段实际 FPS（各次中位数） | TrackStereo 平均 ms（各次中位数） | TrackStereo P95 ms（各次中位数） | 含图像读取的处理 ms | PNG 读取 ms |",
          "|---|---:|---:|---:|---:|---:|"]
for r in table:
    lines.append(f"| {r['first']:g}/{r['second']:g} | {fmt(r['replay_fps_median'])} | {fmt(r['tracking_mean_ms_median'])} | {fmt(r['tracking_p95_ms_median'])} | {fmt(r['processing_mean_ms_median'])} | {fmt(r['read_mean_ms_median'])} |")
lines += ["", "## 各次试验及完整等待时间", "",
          "原始输入由两次物理录制组成，第一段存图后原程序崩溃，第二段由新进程补录；回放在两段之间执行全量重置。",
          "第一段包含较长的运动不足阶段。下面从各段开始到 VIBA 2 真正完成的墙钟时间包含该段此前的重试，不能与仅从有效首关键帧计时混用。", "",
          "| 试验 | 退出码 | 完整恢复 | 第一段等待（秒） | 第二段等待（秒） | 实际 FPS |",
          "|---|---:|---|---:|---:|---:|"]
for r in results:
    segments = {m['segment']: m['wall_from_segment_start_s'] for m in r['map_initializations']}
    lines.append(f"| {r['first_threshold']:g}/{r['second_threshold']:g} 第 {r['repeat']} 次 | {r['exit_code']} | {'通过' if r['pass'] else '未通过'} | {fmt(segments.get(1))} | {fmt(segments.get(2))} | {fmt(r.get('observed_replay_fps'))} |")
lines += ["", "## 文件与适用范围", "",
          "- environment.json：CPU、构建选项、二进制哈希、GUI/限速/帧步长条件。",
          "- results.json / aggregate.csv：所有试验与汇总。",
          "- 每次目录的 initialization.csv：实际优化阶段、地图/首关键帧、传感器时间、单调时钟与耗时。",
          "- timing.csv：图像读取、TrackStereo、处理、等待、每帧状态。",
          "- navigation.csv：初始 IMU 坐标系的位姿；不含地面真值。",
          "- 对齐内点和点对 RMSE 仅衡量内部匹配一致性，不能用来证明哪组初始化具有更高绝对精度。",
          "- 缩短阈值可能更早结束 VIBA，也会更早退出部分运动不足检查。是否可靠需更多场景和真值/已知轨迹验证。",
          "- 每段首次 VIBA 2 截止时限固定。提前失败的试验只统计已处理帧，不能假装完成了全部输入。", ""]
(root / "REPORT.md").write_text("\n".join(lines))
print(root / "REPORT.md")
