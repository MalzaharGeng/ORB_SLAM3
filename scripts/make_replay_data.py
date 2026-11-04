# -*- coding: utf-8 -*-
"""从坠机 telemetry 提炼回放数据，生成紧凑 JSON"""
import csv, math, json, os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

P = os.path.join(ROOT, "data", "px4_telemetry.csv")
OUT = os.path.join(ROOT, "replay", "replay_data.js")


def g(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except Exception:
        return None


rows = list(csv.DictReader(open(P, newline="", encoding="utf-8", errors="replace")))
t0 = g(rows[0]["system_us"])

# 只取解锁前 5 秒之后（地面爬升段不要）
T_FROM = 680.0

T, EX, EY, EZ, PX, PY, PZ, VX, VY, VZ, DH, MD, EVP, EVH, AR, RS = ([] for _ in range(16))
rs_map, rs_list = {}, []
for r in rows:
    t = (g(r["system_us"]) - t0) / 1e6
    if t < T_FROM:
        continue
    ex, ey, ez = g(r["ekf_x"]), g(r["ekf_y"]), g(r["ekf_z"])
    px, py, pz = g(r["px"]), g(r["py"]), g(r["pz"])
    vx, vy, vz = g(r["vx"]), g(r["vy"]), g(r["vz"])
    dh = math.hypot(px - ex, py - ey) if None not in (px, py, ex, ey) else None

    T.append(round(t, 2))
    EX.append(round(ex, 2) if ex is not None else None)
    EY.append(round(ey, 2) if ey is not None else None)
    EZ.append(round(ez, 3) if ez is not None else None)
    PX.append(round(px, 2) if px is not None else None)
    PY.append(round(py, 2) if py is not None else None)
    PZ.append(round(pz, 3) if pz is not None else None)
    VX.append(round(vx, 2) if vx is not None else None)
    VY.append(round(vy, 2) if vy is not None else None)
    VZ.append(round(vz, 2) if vz is not None else None)
    DH.append(round(dh, 2) if dh is not None else None)
    MD.append(int(r["nav_mode"]) if r.get("nav_mode", "").isdigit() else -1)
    EVP.append(int(r["ev_pos"]) if r.get("ev_pos", "").lstrip("-").isdigit() else -1)
    EVH.append(int(r["ev_hgt"]) if r.get("ev_hgt", "").lstrip("-").isdigit() else -1)
    AR.append(int(r["armed"]) if r.get("armed", "").lstrip("-").isdigit() else -1)
    rsn = (r.get("reason") or "").strip()
    if rsn not in rs_map:
        rs_map[rsn] = len(rs_list)
        rs_list.append(rsn)
    RS.append(rs_map[rsn])

data = {
    "from": T_FROM,
    "n": len(T),
    "reasons": rs_list,
    "t": T, "ex": EX, "ey": EY, "ez": EZ,
    "px": PX, "py": PY, "pz": PZ,
    "vx": VX, "vy": VY, "vz": VZ,
    "dh": DH, "md": MD, "evp": EVP, "evh": EVH, "ar": AR, "rs": RS,
}
js = "window.REPLAY=" + json.dumps(data, separators=(",", ":"), ensure_ascii=False) + ";"
open(OUT, "w", encoding="utf-8").write(js)
print(f"rows={len(T)}  t={T[0]}~{T[-1]}  size={os.path.getsize(OUT)/1024:.0f} KB")
print("reasons:", rs_list)
