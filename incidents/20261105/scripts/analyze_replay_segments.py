# -*- coding: utf-8 -*-
"""三组 replay 的对比统计：正常悬停段 / 前段拼接 / 事故段。"""
import csv
import json
import os
import statistics as st

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CASES = [
    ("N0", "正常悬停段 284.15-304.13 s", "replay/norm0_navigation.csv", "replay/norm0_timing.csv"),
    ("NA", "前段拼接 284.15-411.82 s", "replay/norma_navigation.csv", "replay/norma_timing.csv"),
    ("XF", "事故段 1352.87-1372.81 s", "replay/replay_xf_navigation.csv", "replay/replay_xf_timing.csv"),
]


def load(p):
    with open(p, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def pct(v):
    v2 = sorted(v)
    n = len(v2)
    return v2[int(0.95 * (n - 1))] if n else 0


def summarize(name, desc, navp, timp):
    nav = load(os.path.join(ROOT, navp))
    tim = load(os.path.join(ROOT, timp))
    n = len(nav)
    t0 = float(nav[0]["timestamp"])
    t1 = float(nav[-1]["timestamp"])
    valid = sum(1 for r in nav if r["valid"] == "1")
    ok = sum(1 for r in nav if r["tracking"] in ("2", "5"))
    ba2 = [r for r in nav if r["ba2"] == "1"]
    imu = sum(1 for r in nav if r["imu"] == "1")
    first_ba2 = float(ba2[0]["timestamp"]) - t0 if ba2 else None
    first_imu = None
    for r in nav:
        if r["imu"] == "1":
            first_imu = float(r["timestamp"]) - t0
            break
    tr = [float(r["track_ms"]) for r in tim if r.get("track_ms")]
    rd = [float(r["read_ms"]) for r in tim if r.get("read_ms")]
    cy = [float(r["cycle_ms"]) for r in tim if r.get("cycle_ms")]
    # 位置：按 ba2 段切分，统计相邻帧位移速率
    pos = [(float(r["timestamp"]), float(r["imu_x"]), float(r["imu_y"]), float(r["imu_z"]), r["valid"])
           for r in nav]
    vmax = 0.0
    jump = []
    for i in range(1, len(pos)):
        dt = pos[i][0] - pos[i - 1][0]
        if dt <= 0 or pos[i][4] != "1" or pos[i - 1][4] != "1":
            continue
        d = ((pos[i][1] - pos[i - 1][1]) ** 2 + (pos[i][2] - pos[i - 1][2]) ** 2 + (pos[i][3] - pos[i - 1][3]) ** 2) ** 0.5
        v = d / dt
        if d > 0.5:
            jump.append((pos[i][0], d))
        vmax = max(vmax, v)
    return {
        "id": name, "desc": desc,
        "frames": n, "span_s": round(t1 - t0, 2),
        "valid_pct": round(100.0 * valid / n, 1),
        "tracking_ok_pct": round(100.0 * ok / n, 1),
        "imu_init_pct": round(100.0 * imu / n, 1),
        "ba2_pct": round(100.0 * len(ba2) / n, 1),
        "first_imu_s": None if first_imu is None else round(first_imu, 2),
        "first_ba2_s": None if first_ba2 is None else round(first_ba2, 2),
        "read_ms_p50": round(st.median(rd), 1),
        "track_ms_p50": round(st.median(tr), 1),
        "track_ms_p95": round(pct(tr), 1),
        "track_ms_max": round(max(tr), 1),
        "track_over_frame_pct": round(100.0 * sum(1 for x in tr if x > 66.7) / len(tr), 1),
        "cycle_ms_p50": round(st.median(cy), 1),
        "pos_jumps": len(jump),
        "pos_jump_max_m": round(max((d for _, d in jump), default=0.0), 2),
        "max_step_speed_m_s": round(vmax, 2),
    }


def main():
    out = [summarize(*c) for c in CASES]
    print("%-4s %-30s %5s %7s %7s %7s %7s %8s %8s %8s %7s %7s" % (
        "id", "desc", "帧", "valid%", "ok%", "imu%", "ba2%", "首imu", "首ba2", "trk p50", "p95", ">66.7%"))
    for r in out:
        print("%-4s %-30s %5d %7.1f %7.1f %7.1f %7.1f %8s %8s %8.1f %7.1f %7.1f" % (
            r["id"], r["desc"], r["frames"], r["valid_pct"], r["tracking_ok_pct"], r["imu_init_pct"],
            r["ba2_pct"], r["first_imu_s"], r["first_ba2_s"], r["track_ms_p50"], r["track_ms_p95"],
            r["track_over_frame_pct"]))
        print("     位置跳变>0.5m: %d 次 (最大 %.2f m) | 最大帧间速度 %.2f m/s | cycle p50 %.1f ms" % (
            r["pos_jumps"], r["pos_jump_max_m"], r["max_step_speed_m_s"], r["cycle_ms_p50"]))
    with open(os.path.join(ROOT, "replay", "replay_summary.json"), "w", encoding="utf-8") as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1)
    print("\nwrote replay_summary.json")


if __name__ == "__main__":
    main()
