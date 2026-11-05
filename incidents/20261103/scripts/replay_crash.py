# -*- coding: utf-8 -*-
"""坠机那次（19700101_080117_6nQrcY）的数据回放：逐段打印"""
import csv, math, os, sys, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

P = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "data", "px4_telemetry.csv")


def g(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except Exception:
        return None


rows = list(csv.DictReader(open(P, newline="", encoding="utf-8", errors="replace")))
print(f"总行数 {len(rows)}")

t0 = g(rows[0]["system_us"])
armed_rows = [r for r in rows if r.get("armed") == "1"]
print(f"armed=1 行数 {len(armed_rows)}")
if not armed_rows:
    raise SystemExit

print(f"首次 armed=1 @ t={ (g(armed_rows[0]['system_us'])-t0)/1e6:.1f}s")
print(f"末行         @ t={ (g(rows[-1]['system_us'])-t0)/1e6:.1f}s")
print(f"末行 armed/mode = {rows[-1]['armed']}/{rows[-1]['nav_mode']}")

print(f"\n{'t(s)':>7s} {'arm':>4s} {'md':>3s} {'ev':>5s} "
      f"{'ekf_x':>8s} {'ekf_y':>8s} {'ekf_z':>8s} {'vx':>7s} {'vy':>7s} {'vz':>7s} "
      f"{'px':>7s} {'py':>7s} {'pz':>7s} {'dH':>6s}")
print("-" * 118)

last = None
for r in rows:
    t = (g(r["system_us"]) - t0) / 1e6
    if last is not None and t - last < 2.0:
        continue
    last = t
    ex, ey, ez = g(r["ekf_x"]), g(r["ekf_y"]), g(r["ekf_z"])
    px, py, pz = g(r["px"]), g(r["py"]), g(r["pz"])
    vx, vy, vz = g(r["vx"]), g(r["vy"]), g(r["vz"])
    dh = math.hypot(px - ex, py - ey) if None not in (px, py, ex, ey) else None
    f = lambda v, w=8, d=2: (f"{v:{w}.{d}f}" if v is not None else " " * (w - 1) + "-")
    print(f"{t:7.1f} {r['armed']:>4s} {r['nav_mode']:>3s} "
          f"{r['ev_pos']}/{r['ev_hgt']}/{r['ev_vel']:>1s} "
          f"{f(ex)} {f(ey)} {f(ez)} {f(vx,7)} {f(vy,7)} {f(vz,7)} "
          f"{f(px,7)} {f(py,7)} {f(pz,7)} {f(dh,6)}")

print("\n=== 最后 15 行原始关键列 ===")
for r in rows[-15:]:
    t = (g(r["system_us"]) - t0) / 1e6
    print(f"t={t:7.1f} armed={r['armed']} mode={r['nav_mode']} ev={r['ev_pos']}/{r['ev_hgt']}/{r['ev_vel']} "
          f"ekf=({r['ekf_x']},{r['ekf_y']},{r['ekf_z']}) v=({r['vx']},{r['vy']},{r['vz']}) "
          f"px=({r['px']},{r['py']},{r['pz']}) rsn={r['reason']}")
