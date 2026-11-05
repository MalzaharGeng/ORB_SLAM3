#!/usr/bin/env python3
"""分析飞控 ULog：模式序列、电池曲线、电机输出、控制分配、姿态与摇杆对照。

用法:
    pip install pyulog
    python analyze_ulg.py data/flight_log/14_07_14.ulg
"""
import sys
import math
import numpy as np
from pyulog import ULog

MODE = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "HOLD",
        5: "RTL", 6: "ACRO", 14: "OFFBOARD", 17: "TAKEOFF", 18: "LAND"}


def load(path):
    u = ULog(path)
    data = {}
    for d in u.data_list:
        data.setdefault(d.name, []).append(d.data)
    return u, data


def get(data, name, idx=0):
    ds = data.get(name)
    return ds[min(idx, len(ds) - 1)] if ds else None


def main(path):
    u, data = load(path)
    armed = get(data, "actuator_armed")
    on = np.where(armed["armed"] == 1)[0]
    if len(on) == 0:
        raise SystemExit("日志里没有 armed=1，无法定位起点")
    T0 = int(armed["timestamp"][on[0]])

    def rel(a):
        return (np.asarray(a, dtype=np.int64) - T0) / 1e6

    print(f"文件: {path}")
    print(f"解锁时刻: boot {T0/1e6:.2f}s   日志时长: {rel(u.last_timestamp):.2f}s")

    # ---- 模式 ----
    print("\n=== 模式 / 解锁状态变化 ===")
    vs = get(data, "vehicle_status")
    prev = None
    for i in range(len(vs["timestamp"])):
        k = (int(vs["nav_state"][i]), int(vs["arming_state"][i]))
        if k != prev:
            print(f"  t={float(rel(vs['timestamp'][i])):7.2f}s  "
                  f"nav_state={k[0]} ({MODE.get(k[0],'?')})  arming_state={k[1]}")
            prev = k

    # ---- 控制模式 flag ----
    print("\n=== 控制模式标志变化 ===")
    vcm = get(data, "vehicle_control_mode")
    keys = [k for k in vcm if k.startswith("flag_control_")
            and any(s in k for s in ("offboard", "position", "altitude", "attitude", "manual"))]
    prev = None
    for i in range(len(vcm["timestamp"])):
        cur = tuple(int(vcm[k][i]) for k in keys)
        if cur != prev:
            print(f"  t={float(rel(vcm['timestamp'][i])):7.2f}s  "
                  + " ".join(f"{k.replace('flag_control_','')}={v}" for k, v in zip(keys, cur)))
            prev = cur

    # ---- 电池 ----
    print("\n=== 电池 ===")
    b = get(data, "battery_status")
    last = -9
    for i in range(len(b["timestamp"])):
        t = float(rel(b["timestamp"][i]))
        if t - last < 1.5:
            continue
        last = t
        warn = int(b["warning"][i]) if "warning" in b else 0
        cells = int(b["cell_count"][i]) if "cell_count" in b else 0
        vpc = b["voltage_v"][i] / cells if cells else 0
        print(f"  t={t:6.2f}s  V={b['voltage_v'][i]:5.2f}  A={b['current_a'][i]:6.2f}  "
              f"rem={b['remaining'][i]:.3f}  {vpc:.2f}V/cell  warn={warn}")

    # ---- 控制分配 ----
    print("\n=== 控制分配（异常才打印）===")
    ca = get(data, "control_allocator_status")
    last = -9
    bad = 0
    for i in range(len(ca["timestamp"])):
        t = float(rel(ca["timestamp"][i]))
        if t - last < 1.0:
            continue
        last = t
        sat = [int(ca[f"actuator_saturation[{k}]"][i]) for k in range(4)]
        tq = int(ca["torque_setpoint_achieved"][i])
        th = int(ca["thrust_setpoint_achieved"][i])
        if tq != 1 or th != 1 or max(sat) != 0:
            bad += 1
            print(f"  t={t:6.2f}s  torque_ok={tq} thrust_ok={th} sat={sat}")
    if bad == 0:
        print("  （全程正常：torque_ok=1, thrust_ok=1, sat=0）")

    # ---- 姿态 vs 摇杆 ----
    print("\n=== 姿态 vs 遥控（每 2 秒）===")
    att = get(data, "vehicle_attitude")
    mc = get(data, "manual_control_setpoint")

    def q2e(q0, q1, q2, q3):
        r = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        p = math.asin(max(-1, min(1, 2 * (q0 * q2 - q3 * q1))))
        return math.degrees(r), math.degrees(p)

    ta = rel(att["timestamp"])
    rolls, pitches = [], []
    for i in range(len(ta)):
        r, p = q2e(att["q[0]"][i], att["q[1]"][i], att["q[2]"][i], att["q[3]"][i])
        rolls.append(r)
        pitches.append(p)
    tm = rel(mc["timestamp"])
    print(f"{'t':>7s} {'roll':>7s} {'pitch':>7s} | {'rc_r':>6s} {'rc_p':>6s} {'rc_t':>6s}")
    for q in np.arange(0, float(ta[-1]), 2.0):
        r = float(np.interp(q, ta, rolls))
        p = float(np.interp(q, ta, pitches))
        rr = float(np.interp(q, tm, mc["roll"]))
        rp = float(np.interp(q, tm, mc["pitch"]))
        rt = float(np.interp(q, tm, mc["throttle"]))
        print(f"{q:7.1f} {r:7.1f} {p:7.1f} | {rr:+6.2f} {rp:+6.2f} {rt:+6.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
