# -*- coding: utf-8 -*-
"""从坠机 telemetry 截取关键数据，生成 markdown 摘录"""
import csv, math, os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SRC = os.path.join(ROOT, "data", "px4_telemetry.csv")
OUT = os.path.join(ROOT, "analysis", "坠机数据摘录_6nQrcY.md")


def g(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except Exception:
        return None


rows = list(csv.DictReader(open(SRC, newline="", encoding="utf-8", errors="replace")))
t0 = g(rows[0]["system_us"])

recs = []
for r in rows:
    t = (g(r["system_us"]) - t0) / 1e6
    ex, ey, ez = g(r["ekf_x"]), g(r["ekf_y"]), g(r["ekf_z"])
    px, py, pz = g(r["px"]), g(r["py"]), g(r["pz"])
    vx, vy, vz = g(r["vx"]), g(r["vy"]), g(r["vz"])
    dh = math.hypot(px - ex, py - ey) if None not in (px, py, ex, ey) else None
    recs.append(dict(
        t=t,
        ex=ex, ey=ey, ez=ez,
        px=px, py=py, pz=pz,
        vx=vx, vy=vy, vz=vz, dh=dh,
        arm=int(r["armed"]) if r.get("armed", "").lstrip("-").isdigit() else -1,
        md=int(r["nav_mode"]) if r.get("nav_mode", "").lstrip("-").isdigit() else -1,
        evp=int(r["ev_pos"]) if r.get("ev_pos", "").lstrip("-").isdigit() else -1,
        evh=int(r["ev_hgt"]) if r.get("ev_hgt", "").lstrip("-").isdigit() else -1,
        evv=int(r["ev_vel"]) if r.get("ev_vel", "").lstrip("-").isdigit() else -1,
        rsn=(r.get("reason") or "").strip(),
    ))

flight = [r for r in recs if r["t"] >= 680]
last = recs[-1]

# ---------- 关键节点 ----------
MODE = {0: "Manual", 1: "ALTCTL", 2: "POSCTL", 4: "Auto", 14: "Offboard"}
events = []
prev = None
for r in flight:
    # 只跟踪 armed / mode / EV 融合位
    sig = (r["arm"], r["md"], r["evp"], r["evh"], r["evv"])
    if sig != prev:
        events.append(r)
        prev = sig

# 补充：速度/位置差阈值事件
def add_if(pred, label, used):
    for r in flight:
        if pred(r):
            if not any(abs(r["t"] - u["t"]) < 1.0 for u in used):
                used.append(dict(t=r["t"], label=label, r=r))
            break

marks = [dict(t=r["t"], label=f"armed={r['arm']} mode={MODE.get(r['md'],r['md'])} EV={r['evp']}/{r['evh']}/{r['evv']} · {r['rsn']}", r=r) for r in events]
add_if(lambda r: r["dh"] is not None and r["dh"] > 0.2, "dH 首次 > 0.2 m", marks)
add_if(lambda r: r["dh"] is not None and r["dh"] > 0.5, "dH 首次 > 0.5 m", marks)
add_if(lambda r: r["vx"] is not None and r["vx"] > 1.0, "vx 首次 > 1.0 m/s", marks)
add_if(lambda r: r["vx"] is not None and r["vx"] > 2.0, "vx 首次 > 2.0 m/s", marks)
add_if(lambda r: r["dh"] is not None and r["dh"] > 1.5, "dH 首次 > 1.5 m", marks)
marks.sort(key=lambda m: m["t"])

# 跳变检测
jumps = []
for a, b in zip(recs, recs[1:]):
    if a["ex"] is None or b["ex"] is None:
        continue
    d = math.hypot(b["ex"] - a["ex"], b["ey"] - a["ey"])
    if d > 0.4:
        jumps.append((b["t"], a["ex"], a["ey"], b["ex"], b["ey"], d, a["dh"], b["dh"]))

# ---------- 生成 md ----------
L = []
L.append("# 11-03 坠机数据摘录")
L.append("")
L.append("**来源**：本目录 `data/px4_telemetry.csv`（原始位置 `/home/nano/ORB_SLAM3/recordings/test_runs/19700101_080117_live_6nQrcY/px4_telemetry.csv`）  ")
L.append(f"**原始行数**：{len(rows)} 行　**采样**：约 15 Hz　**提取区间**：t ≥ 680 s（共 {len(flight)} 行）  ")
L.append("**时间基准**：t = 相对 SLAM 启动秒数（会话内 08:01:15 起算，对应 11-03 约 23:09）")
L.append("")
L.append("**坐标约定**：PX4 local NED，x 向北、y 向东、z 向下为正（负值 = 高于原点）。dH = VIO 与 EKF 的水平位置差。")
L.append("")
L.append("---")
L.append("")
L.append("## 一、会话总览")
L.append("")
L.append("| 项 | 值 |")
L.append("|---|---|")
first_armed = next((r for r in recs if r["arm"] == 1), None)
L.append(f"| 首次 armed=1 | t = {first_armed['t']:.1f} s |" if first_armed else "| 首次 armed=1 | — |")
L.append(f"| 最后一帧 | t = {last['t']:.1f} s，armed={last['arm']}，mode={MODE.get(last['md'],last['md'])} |")
L.append(f"| 飞行时长 | {last['t'] - first_armed['t']:.1f} s（{last['t']-first_armed['t']:.0f} 秒） |" if first_armed else "")
L.append(f"| 结束方式 | 日志突然中断，无 Shutdown（VIO 当时 hz≈15.98、sent=2488 正常） |")
L.append(f"| 最大 dH | {max((r['dh'] for r in flight if r['dh'] is not None), default=0):.2f} m |")
L.append(f"| 最大 vx | {max((r['vx'] for r in flight if r['vx'] is not None), default=0):.2f} m/s |")
L.append(f"| EKF x 行程 | {min((r['ex'] for r in flight if r['ex'] is not None)):.2f} → {max((r['ex'] for r in flight if r['ex'] is not None)):.2f} m |")
L.append(f"| EKF y 行程 | {min((r['ey'] for r in flight if r['ey'] is not None)):.2f} → {max((r['ey'] for r in flight if r['ey'] is not None)):.2f} m |")
L.append(f"| EKF z 范围 | {min((r['ez'] for r in flight if r['ez'] is not None)):.2f} → {max((r['ez'] for r in flight if r['ez'] is not None)):.2f} m |")
L.append("")
L.append("---")
L.append("")
L.append("## 二、状态跃迁时间线")
L.append("")
L.append("| t (s) | 事件 | ekf_x | ekf_y | ekf_z | vx | dH |")
L.append("|---|---|---|---|---|---|---|")
for m in marks:
    r = m["r"]
    f = lambda v, d=2: ("—" if v is None else f"{v:.{d}f}")
    L.append(f"| {m['t']:.2f} | {m['label']} | {f(r['ex'])} | {f(r['ey'])} | {f(r['ez'],3)} | {f(r['vx'])} | {f(r['dh'])} |")
L.append("")
L.append("---")
L.append("")
L.append("## 三、位置跳变（单帧水平位移 > 0.4 m）")
L.append("")
if jumps:
    L.append("| t (s) | 跳前 (x,y) | 跳后 (x,y) | 位移 | dH 跳前 → 跳后 |")
    L.append("|---|---|---|---|---|")
    for t, x0, y0, x1, y1, d, dh0, dh1 in jumps:
        L.append(f"| {t:.2f} | ({x0:.2f}, {y0:.2f}) | ({x1:.2f}, {y1:.2f}) | **{d:.2f} m** | "
                 f"{'—' if dh0 is None else f'{dh0:.2f}'} → {'—' if dh1 is None else f'{dh1:.2f}'} |")
else:
    L.append("（无）")
L.append("")
L.append("---")
L.append("")
L.append("## 四、坠机前 5 秒逐帧（已剔除无 VIO 数据的帧）")
L.append("")
L.append("| t (s) | ekf_x | ekf_y | ekf_z | VIO x | VIO y | vx | vy | vz | dH | mode |")
L.append("|---|---|---|---|---|---|---|---|---|---|---|")
tail = [r for r in flight if r["t"] >= last["t"] - 5 and r["px"] is not None]
for i, r in enumerate(tail):
    f = lambda v, d=2: ("—" if v is None else f"{v:.{d}f}")
    hl = ""
    if r["dh"] is not None and r["dh"] > 1.3:
        hl = " style=\"background:#fef2f2\""
    L.append(f"| {r['t']:.2f} | {f(r['ex'])} | {f(r['ey'])} | {f(r['ez'],3)} | "
             f"{f(r['px'])} | {f(r['py'])} | {f(r['vx'])} | {f(r['vy'])} | {f(r['vz'])} | "
             f"{f(r['dh'])} | {MODE.get(r['md'],r['md'])} |")
L.append("")
L.append("---")
L.append("")
L.append("## 五、结论")
L.append("")
L.append("1. **悬停阶段 dH = 0.00 m**，VIO 与 EKF 完全一致 → 静态标定、外参、坐标对齐在静止时没有缺陷。")
L.append("2. **一旦运动 dH 随速度单向增长**：0.1 m/s 档均值 0.03 m，1.6 m/s 档 0.23 m，到 2.8 m/s 时 1.93 m。实测观测时延约 105 ms，在 2.8 m/s 下只能解释 0.3 m，其余无法用时延解释。")
L.append("3. **t≈876.93 出现单帧 1.79 m 水平跳变**，dH 从 1.61 骤降到 0.16 → EKF 把位置强制拉回 VIO。")
L.append("4. 该跳变对位置控制器等效于一次 1.8 m 阶跃给定，控制器大幅修正姿态 → 表现为持续前倾俯冲。")
L.append("5. 高度同步下沉：ekf_z 从 −0.22 降到 −0.32，vz 全程为小负值。")
L.append("6. **同一形态出现过两次**：t≈698.56（EV 首次接上）也有 1.67 m 跳变，dH 从 1.47 降到 0.15；坠机那次 1.79 m，dH 从 1.61 降到 0.16。量级与形态一致，说明 EKF 在反复对 VIO 做位置重对齐。")
L.append("7. 数据在 t≈877.4 中断，armed 仍为 1 —— 坠地/断电。")
L.append("")
L.append("---")
L.append("")
L.append("## 六、根因")
L.append("")
L.append("`EKF2_EV_CTRL = 11`，只打开位置与航向，速度位没有开。遥测 3345 帧的 `ev_vel` 全为 0，run.log 末行 `EV(pos/hgt/vel)=1/1/0` 与此一致。")
L.append("")
L.append("EV 不提供速度，EKF 只能用 15 Hz、约 105 ms 时延的位置观测去推速度。速度越高，推算误差越大，这正是 dH 随速度单向增长的原因；残差累积到门限后 EKF 执行位置重对齐，表现为单帧大跳变，也就是第 3、6 条那两次。")
L.append("")
L.append("触发条件因此是「高速 + 只有位置观测」：速度到 2.8 m/s 时重对齐量 1.79 m，足以让位置控制器输出失控的姿态修正。")
L.append("")
L.append("---")
L.append("")
L.append("## 七、更正")
L.append("")
L.append("此前把原因归到「`cs_yaw_align` 恒为 0，位置错系」，本份数据不支持：内录 3400 行里 `cs_ev_yaw = 1` 有 2673 行（`cs_ev_pos`、`cs_ev_hgt` 同为 2673 行），yaw 对齐是成功的。跳变源于融合配置缺速度观测，不是坐标系映射错，原结论作废。")

open(OUT, "w", encoding="utf-8").write("\n".join(L))
print(f"written {OUT}  {os.path.getsize(OUT)} B")
print(f"flight rows={len(flight)}  marks={len(marks)}  jumps={len(jumps)}")
