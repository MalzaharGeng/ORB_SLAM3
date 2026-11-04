# -*- coding: utf-8 -*-
"""生成 GitHub README 用的 SVG 图（无 matplotlib 依赖）"""
import csv, math, os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PKG = ROOT
SRC = os.path.join(PKG, "data", "px4_telemetry.csv")
FIG = os.path.join(PKG, "figures")
os.makedirs(FIG, exist_ok=True)

INK = "#1f2328"
INK2 = "#57606a"
INK3 = "#8b949e"
GRID = "#eaeef2"
GREEN = "#1a7f37"
BLUE = "#0969da"
RED = "#cf222e"
AMBER = "#bc4c00"


def g(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except Exception:
        return None


rows = list(csv.DictReader(open(SRC, newline="", encoding="utf-8", errors="replace")))
t0 = g(rows[0]["system_us"])
D = []
for r in rows:
    t = (g(r["system_us"]) - t0) / 1e6
    ex, ey, ez = g(r["ekf_x"]), g(r["ekf_y"]), g(r["ekf_z"])
    px, py, pz = g(r["px"]), g(r["py"]), g(r["pz"])
    vx = g(r["vx"])
    dh = math.hypot(px - ex, py - ey) if None not in (px, py, ex, ey) else None
    D.append(dict(t=t, ex=ex, ey=ey, ez=ez, px=px, py=py, pz=pz, vx=vx, dh=dh))
F = [d for d in D if d["t"] >= 680]


# ---------------- 图1：水平轨迹 ----------------
def traj_svg():
    W, H = 920, 430
    L, R, T, B = 62, 130, 26, 42
    pw, ph = W - L - R, H - T - B
    xs = [v for d in F for v in (d["ex"], d["px"]) if v is not None]
    ys = [v for d in F for v in (d["ey"], d["py"]) if v is not None]
    x0, x1 = min(xs) - 0.4, max(xs) + 0.4
    y0, y1 = min(ys) - 0.4, max(ys) + 0.4
    k = min(pw / (x1 - x0), ph / (y1 - y0))
    ox = L + (pw - (x1 - x0) * k) / 2
    oy = T + (ph - (y1 - y0) * k) / 2
    X = lambda v: ox + (v - x0) * k
    Y = lambda v: oy + (y1 - v) * k

    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">']
    s.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')

    # 网格
    for gx in range(int(math.ceil(x0)), int(x1) + 1):
        s.append(f'<line x1="{X(gx):.1f}" y1="{oy:.1f}" x2="{X(gx):.1f}" y2="{oy+(y1-y0)*k:.1f}" stroke="{GRID}"/>')
        s.append(f'<text x="{X(gx):.1f}" y="{oy+(y1-y0)*k+15:.1f}" font-size="10" fill="{INK3}" text-anchor="middle">{gx}</text>')
    for gy in range(int(math.ceil(y0)), int(y1) + 1):
        s.append(f'<line x1="{ox:.1f}" y1="{Y(gy):.1f}" x2="{ox+(x1-x0)*k:.1f}" y2="{Y(gy):.1f}" stroke="{GRID}"/>')
        s.append(f'<text x="{ox-7:.1f}" y="{Y(gy)+3.5:.1f}" font-size="10" fill="{INK3}" text-anchor="end">{gy}</text>')

    s.append(f'<text x="{ox+(x1-x0)*k/2:.0f}" y="{H-10}" font-size="11" fill="{INK2}" text-anchor="middle">x (m, 北)</text>')
    s.append(f'<text x="16" y="{T+8}" font-size="11" fill="{INK2}">y (m, 东)</text>')

    # VIO 轨迹
    seg = []
    for d in F:
        if d["px"] is None or d["py"] is None:
            if len(seg) > 1:
                s.append(f'<polyline points="{" ".join(seg)}" fill="none" stroke="{BLUE}" stroke-width="1.3" opacity="0.55"/>')
            seg = []
        else:
            seg.append(f"{X(d['px']):.1f},{Y(d['py']):.1f}")
    if len(seg) > 1:
        s.append(f'<polyline points="{" ".join(seg)}" fill="none" stroke="{BLUE}" stroke-width="1.3" opacity="0.55"/>')

    # EKF 轨迹
    seg = []
    for d in F:
        if d["ex"] is None or d["ey"] is None:
            if len(seg) > 1:
                s.append(f'<polyline points="{" ".join(seg)}" fill="none" stroke="{GREEN}" stroke-width="2"/>')
            seg = []
        else:
            seg.append(f"{X(d['ex']):.1f},{Y(d['ey']):.1f}")
    if len(seg) > 1:
        s.append(f'<polyline points="{" ".join(seg)}" fill="none" stroke="{GREEN}" stroke-width="2"/>')

    # 起点
    for d in F:
        if d["ex"] is not None:
            s.append(f'<circle cx="{X(d["ex"]):.1f}" cy="{Y(d["ey"]):.1f}" r="4" fill="{INK3}"/>')
            s.append(f'<text x="{X(d["ex"])+8:.1f}" y="{Y(d["ey"])+4:.1f}" font-size="10.5" fill="{INK3}">起飞点</text>')
            break

    # 跳变点
    idx = next((i for i, d in enumerate(D) if d["t"] >= 876.93), None)
    if idx is not None:
        a, b = D[idx - 1], D[idx]
        if a["ex"] is not None and b["ex"] is not None:
            s.append(f'<line x1="{X(a["ex"]):.1f}" y1="{Y(a["ey"]):.1f}" x2="{X(b["ex"]):.1f}" y2="{Y(b["ey"]):.1f}" '
                     f'stroke="{RED}" stroke-width="2.5" stroke-dasharray="5 3"/>')
            s.append(f'<circle cx="{X(b["ex"]):.1f}" cy="{Y(b["ey"]):.1f}" r="6" fill="{RED}" opacity="0.18"/>')
            s.append(f'<circle cx="{X(b["ex"]):.1f}" cy="{Y(b["ey"]):.1f}" r="3.5" fill="{RED}"/>')
            s.append(f'<text x="{X(b["ex"])-6:.1f}" y="{Y(b["ey"])-14:.1f}" font-size="11" font-weight="600" '
                     f'fill="{RED}" text-anchor="end">t=876.93s 跳变 1.79 m</text>')

    # 图例
    lx, ly = W - R + 16, T + 6
    items = [(GREEN, "EKF 估计"), (BLUE, "VIO 原始"), (RED, "位置跳变")]
    for i, (c, lab) in enumerate(items):
        yy = ly + i * 20
        s.append(f'<line x1="{lx}" y1="{yy}" x2="{lx+18}" y2="{yy}" stroke="{c}" stroke-width="2.4"/>')
        s.append(f'<text x="{lx+24}" y="{yy+4}" font-size="11" fill="{INK2}">{lab}</text>')
    s.append('</svg>')
    open(os.path.join(FIG, "trajectory.svg"), "w", encoding="utf-8").write("".join(s))
    print("trajectory.svg")


# ---------------- 图2：三行曲线 ----------------
def series_svg():
    W = 920
    rowH, gap, T, L, R = 108, 30, 22, 54, 22
    H = T + rowH * 3 + gap * 2 + 30
    tmin, tmax = 860, 878
    S = [d for d in F if tmin <= d["t"] <= tmax]

    def x(v): return L + (W - L - R) * (v - tmin) / (tmax - tmin)

    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">']
    s.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')

    panels = [
        ("dh", "dH  VIO 与 EKF 水平差 (m)", 0, 2.2, RED),
        ("ez", "ekf_z  高度 (m, NED)", -0.6, 0.1, "#0969da"),
        ("vx", "vx  前向速度 (m/s)", -0.5, 3.2, "#8250df"),
    ]

    for pi, (key, title, vmin, vmax, color) in enumerate(panels):
        ytop = T + pi * (rowH + gap)
        ybot = ytop + rowH
        Y = lambda v: ybot - rowH * (v - vmin) / (vmax - vmin)

        s.append(f'<text x="{L}" y="{ytop-6}" font-size="11.5" font-weight="600" fill="{INK}">{title}</text>')
        for i in range(4):
            v = vmin + (vmax - vmin) * i / 3
            s.append(f'<line x1="{L}" y1="{Y(v):.1f}" x2="{W-R}" y2="{Y(v):.1f}" stroke="{GRID}"/>')
            s.append(f'<text x="{L-7}" y="{Y(v)+3.5:.1f}" font-size="9.5" fill="{INK3}" text-anchor="end">{v:g}</text>')

        # 关键竖线
        for tt, lab, col in [(873.76, "扰动", AMBER), (875.94, "加速", AMBER), (876.93, "跳变", RED)]:
            s.append(f'<line x1="{x(tt):.1f}" y1="{ytop}" x2="{x(tt):.1f}" y2="{ybot}" stroke="{col}" '
                     f'stroke-width="1" stroke-dasharray="3 3" opacity="0.65"/>')
            if pi == 0:
                s.append(f'<text x="{x(tt):.1f}" y="{ytop-6}" font-size="9.5" fill="{col}" text-anchor="middle">{lab}</text>')

        seg = []
        for d in S:
            v = d[key]
            if v is None:
                if len(seg) > 1:
                    s.append(f'<polyline points="{" ".join(seg)}" fill="none" stroke="{color}" stroke-width="1.8"/>')
                seg = []
            else:
                seg.append(f"{x(d['t']):.1f},{Y(v):.1f}")
        if len(seg) > 1:
            s.append(f'<polyline points="{" ".join(seg)}" fill="none" stroke="{color}" stroke-width="1.8"/>')

        if pi == 2:
            for tt in range(tmin, tmax + 1, 2):
                s.append(f'<text x="{x(tt):.1f}" y="{ybot+16}" font-size="10" fill="{INK3}" text-anchor="middle">{tt}</text>')
            s.append(f'<text x="{(L+W-R)/2:.0f}" y="{H-8}" font-size="11" fill="{INK2}" text-anchor="middle">t (s, 相对 SLAM 启动)</text>')

    s.append('</svg>')
    open(os.path.join(FIG, "timeline.svg"), "w", encoding="utf-8").write("".join(s))
    print("timeline.svg")


traj_svg()
series_svg()
for f in ["trajectory.svg", "timeline.svg"]:
    p = os.path.join(FIG, f)
    print(f"  {f}  {os.path.getsize(p)/1024:.1f} KB")
