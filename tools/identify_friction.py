#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台单轴摩擦参数辨识（纯标准库，不需要 numpy/scipy）。

模型（关节侧，抬头/逆时针为正，与 trd_gimbal.cpp 约定一致）:

    J * dw/dt = tau - b * w - tauc * sign(w)

    J    转动惯量       (kg·m^2)         —— 已知
    b    粘滞摩擦系数   (N·m·s/rad)      —— 本脚本要辨识的目标
    tauc 库仑摩擦       (N·m)            —— 顺带辨识
    tau  电机力矩       (N·m)            —— 命令力矩或反馈力矩
    w    角速度         (rad/s)          —— 关节角速度（电机侧 / 减速比）

辨识方法（三种，可分别或全部运行）:

  steady  稳态转矩-转速。稳态 d w/dt ~ 0，于是 tau = b*w + tauc，是一条直线，
          斜率直接就是 b，连 J 都不需要。最稳，推荐先用它。
  ode     整段 ODE 回归。tau = J*dw/dt + b*w + tauc*sign(w) 对
          [dw/dt, w, sign(w)] 做线性最小二乘；J 已知就固定 J 只解 b、tauc。
          对闭环/手扳数据也能用，缺点是 dw/dt 对噪声敏感。
  decay   自由衰减。找 tau ~ 0 的段，w 近似指数衰减，ln|w| 的斜率 = -b/J，
          于是 b = -斜率 * J。用来交叉验证 steady。

数据来源（二选一）:
  · 文本/CSV：每行 "t,tau,w" 或 "tau,w"（逗号/分号/空白都行，# 是注释）
  · VOFA+ JustFloat 原始二进制：n 个 float(小端) + 帧尾 00 00 80 7F

用法示例:
  tools/identify_friction.py --demo
  tools/identify_friction.py run.csv --J 0.012 --time-col 0 --tau-col 1 --omega-col 2
  tools/identify_friction.py run.bin --format vofa --channels 2 --tau-col 0 --omega-col 1 --dt 0.001 --J 0.012
  tools/identify_friction.py run.csv --J 0.012 --omega-deg-per-s --plot fit.svg

采集建议：跑开环恒力矩台阶（如 ±0.5/1/1.5/2 N·m，每档 1~2s）最好，数据里同时
记 tau 和 w（把 trd_gimbal.cpp 的 VofaPublish 里 yaw_out_torque / yaw_snap.omega
两条放开即可）。稳态法对反馈低通不敏感（DC 增益不受影响）；ode 法会被 100Hz
角速度低通拖相位，尽量以稳态法为准。
"""

from __future__ import annotations

import argparse
import math
import struct
import sys

# VOFA+ JustFloat 帧尾：0x7F800000 小端
VOFA_TAIL = b"\x00\x00\x80\x7f"


# =====================================================================
# 数据加载
# =====================================================================

def _looks_like_text(data: bytes) -> bool:
    if not data:
        return False
    sample = data[:4096]
    ok = sum(1 for b in sample if b in (9, 10, 11, 12, 13) or 32 <= b < 127)
    return ok / len(sample) > 0.9


def parse_vofa(data: bytes, channels: int):
    """把 VOFA+ JustFloat 原始流解成 rows（每行 channels 个 float）。"""
    frame = channels * 4 + 4
    tails = []
    start = 0
    while True:
        j = data.find(VOFA_TAIL, start)
        if j < 0:
            break
        tails.append(j)
        start = j + 4

    rows = []
    seen = set()
    for j in tails:
        fstart = j + 4 - frame
        if fstart < 0 or fstart in seen:
            continue
        seen.add(fstart)
        rows.append(list(struct.unpack_from("<%df" % channels, data, fstart)))
    return rows


def parse_text(text: str):
    rows = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        line = line.replace(",", " ").replace(";", " ").replace("\t", " ")
        parts = [p for p in line.split() if p]
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue
        if not all(math.isfinite(v) for v in vals):   # 丢掉 nan/inf 行（VOFA 起停时常有）
            continue
        rows.append(vals)
    return rows


def load_rows(path: str, fmt: str, channels: int):
    with open(path, "rb") as fh:
        data = fh.read()
    if fmt == "vofa" or (fmt == "auto" and not _looks_like_text(data)):
        if channels <= 0:
            raise SystemExit("VOFA 二进制需要 --channels（每帧几个 float）")
        rows = parse_vofa(data, channels)
        if not rows:
            raise SystemExit("没解析出任何 VOFA 帧：确认 --channels 与帧尾格式")
        return rows
    return parse_text(data.decode("utf-8", errors="replace"))


def pick_columns(rows, args):
    ncol = min(len(r) for r in rows)
    tc, tauc, wc = args.time_col, args.tau_col, args.omega_col
    if tauc is None or wc is None:
        if ncol >= 3:
            tc = 0 if tc is None else tc
            tauc = 1 if tauc is None else tauc
            wc = 2 if wc is None else wc
        elif ncol == 2:
            tauc = 0 if tauc is None else tauc
            wc = 1 if wc is None else wc
        else:
            raise SystemExit("列数不足：至少需要 tau、w 两列（用 --tau-col/--omega-col 指定）")
    return tc, tauc, wc


def extract(rows, tc, tauc, wc, dt, omega_scale):
    tau, omega, t = [], [], []
    for i, r in enumerate(rows):
        if tauc >= len(r) or wc >= len(r):
            continue
        tau.append(r[tauc])
        omega.append(r[wc] * omega_scale)
        t.append(r[tc] if (tc is not None and tc < len(r)) else i * dt)
    return t, tau, omega


# =====================================================================
# 线性最小二乘（标准库实现）
# =====================================================================

def solve_linear(A, bvec):
    n = len(A)
    M = [row[:] + [bvec[i]] for i, row in enumerate(A)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-15:
            return None
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        for r in range(n):
            if r == col:
                continue
            f = M[r][col] / pv
            if f == 0.0:
                continue
            for c in range(col, n + 1):
                M[r][c] -= f * M[col][c]
    return [M[i][n] / M[i][i] for i in range(n)]


def lstsq(X, y):
    k = len(X[0])
    XtX = [[sum(X[i][a] * X[i][b] for i in range(len(X))) for b in range(k)] for a in range(k)]
    Xty = [sum(X[i][a] * y[i] for i in range(len(X))) for a in range(k)]
    beta = solve_linear(XtX, Xty)
    if beta is None:
        return None, float("nan")
    ymean = sum(y) / len(y)
    ss_res = sum((y[i] - sum(beta[a] * X[i][a] for a in range(k))) ** 2 for i in range(len(y)))
    ss_tot = sum((v - ymean) ** 2 for v in y)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return beta, r2


def moving_average(x, win):
    if win <= 1:
        return x[:]
    out = []
    half = win // 2
    n = len(x)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out.append(sum(x[lo:hi]) / (hi - lo))
    return out


def median(x):
    s = sorted(x)
    n = len(s)
    if n == 0:
        return float("nan")
    return s[n // 2] if (n % 2) else 0.5 * (s[n // 2 - 1] + s[n // 2])


def derivative(t, x):
    n = len(x)
    d = [0.0] * n
    for i in range(n):
        if i == 0:
            dt = t[1] - t[0] if n > 1 else 1.0
            d[i] = (x[1] - x[0]) / dt if (n > 1 and dt) else 0.0
        elif i == n - 1:
            dt = t[i] - t[i - 1]
            d[i] = (x[i] - x[i - 1]) / dt if dt else 0.0
        else:
            dt = t[i + 1] - t[i - 1]
            d[i] = (x[i + 1] - x[i - 1]) / dt if dt else 0.0
    return d


def sign(v):
    return 1.0 if v > 0 else (-1.0 if v < 0 else 0.0)


# =====================================================================
# 三种辨识方法
# =====================================================================

def method_steady(t, tau, omega, args):
    """稳态 tau = b*w + tauc：按力矩台阶分段，取每段后半段均值再拟合。"""
    tol = args.plateau_tol
    min_n = args.plateau_min
    segs = []
    start = 0
    n = len(tau)
    for i in range(1, n):
        if abs(tau[i] - tau[start]) > tol:
            segs.append((start, i))
            start = i
    segs.append((start, n))

    pts = []
    for a, b in segs:
        if b - a < min_n:
            continue
        if max(tau[a:b]) - min(tau[a:b]) > tol:
            continue
        half = a + (b - a) // 2
        cnt = b - half
        pts.append((sum(omega[half:b]) / cnt, sum(tau[half:b]) / cnt))

    if len(pts) < 2:
        return None, ("steady: 没找到 >=2 个稳态力矩台阶（需要开环恒力矩数据；"
                      "或调小 --plateau-tol / --plateau-min）")
    beta, r2 = lstsq([[w, 1.0] for w, _ in pts], [tt for _, tt in pts])
    if beta is None:
        return None, "steady: 拟合奇异（转速变化太小？多给几个不同转速的台阶）"
    return {"b": beta[0], "tau_c": beta[1], "r2": r2, "points": pts}, None


def method_ode(t, tau, omega, args):
    """整段 ODE：tau = J*dw/dt + b*w + tauc*sign(w)。"""
    w = moving_average(omega, args.smooth)
    dw = derivative(t, w)
    if args.J is not None:
        resid = [tau[i] - args.J * dw[i] for i in range(len(tau))]
        beta, r2 = lstsq([[w[i], sign(w[i])] for i in range(len(tau))], resid)
        if beta is None:
            return None, "ode: 拟合奇异"
        return {"b": beta[0], "tau_c": beta[1], "r2": r2, "J": args.J}, None
    beta, r2 = lstsq([[dw[i], w[i], sign(w[i])] for i in range(len(tau))], tau)
    if beta is None:
        return None, "ode: 拟合奇异（数据激励不足？）"
    return {"b": beta[1], "tau_c": beta[2], "r2": r2, "J": beta[0]}, None


def method_vsweep(t, tau, omega, args):
    """速度扫描：按 ω 平台分段（速度环会把 ω 稳在各档给定），取每档后半段均值，
    用 τ = b·ω + τc·sign(ω) 做最小二乘 → 斜率就是 b，截距就是库仑摩擦。"""
    tol = args.vs_tol
    min_n = args.plateau_min
    segs = []
    start = 0
    n = len(omega)
    for i in range(1, n):
        if abs(omega[i] - omega[start]) > tol:
            segs.append((start, i))
            start = i
    segs.append((start, n))

    pts = []
    for a, b in segs:
        if b - a < min_n:
            continue
        half = a + (b - a) // 2
        # 取平台后半段的中位数：换向瞬态/尖峰不会把均值带偏
        pts.append((median(omega[half:b]), median(tau[half:b])))

    # 丢掉接近 0 的档：sign(ω) 在那里没意义
    pts = [(w, tt) for w, tt in pts if abs(w) > args.vs_zero]
    if len(pts) < 2:
        return None, ("vsweep: 没找到足够的速度平台（速度扫描数据才有；"
                      "或调大 --vs-tol / 调小 --plateau-min）")
    mags = sorted(abs(w) for w, _ in pts)
    if mags[-1] - mags[0] < 0.3 * (sum(mags) / len(mags)):
        return None, ("vsweep: 所有平台都在同一个 |ω| 上 —— τ=b·ω+τc·sign(ω) 只剩一个"
                      "独立方程，b 和 τc 分不开（会给出负 b 这种假结果）。"
                      "要用不同的速度幅值各测一段再一起拟合。")
    beta, r2 = lstsq([[w, sign(w)] for w, _ in pts], [tt for _, tt in pts])
    if beta is None:
        return None, "vsweep: 拟合奇异（正负速度都要扫到）"
    return {"b": beta[0], "tau_c": beta[1], "r2": r2, "points": pts}, None


def method_integ(t, tau, omega, args):
    """积分法：把 J·dw/dt = tau - b·w - tauc·sign(w) 两边积分，
    ∫tau dt = J·Δw + b·∫w dt + tauc·∫sign dt。不放大噪声，瞬态和稳态信息一起用。"""
    n = len(t)
    A = [0.0] * n; C = [0.0] * n; D = [0.0] * n
    for i in range(1, n):
        dt = t[i] - t[i - 1]
        A[i] = A[i - 1] + 0.5 * (tau[i] + tau[i - 1]) * dt
        C[i] = C[i - 1] + 0.5 * (omega[i] + omega[i - 1]) * dt
        D[i] = D[i - 1] + 0.5 * (sign(omega[i]) + sign(omega[i - 1])) * dt
    if n < 10:
        return None, "integ: 样本太少"
    if args.J is not None:
        y = [A[i] - args.J * (omega[i] - omega[0]) for i in range(1, n)]
        X = [[C[i], D[i]] for i in range(1, n)]
    else:
        y = [A[i] for i in range(1, n)]
        X = [[omega[i] - omega[0], C[i], D[i]] for i in range(1, n)]
    beta, r2 = lstsq(X, y)
    if beta is None:
        return None, "integ: 拟合奇异"
    if args.J is not None:
        return {"b": beta[0], "tau_c": beta[1], "r2": r2, "J": args.J}, None
    return {"b": beta[1], "tau_c": beta[2], "r2": r2, "J": beta[0]}, None


def method_decay(t, tau, omega, args):
    """自由衰减：找 tau~0 段，拟合 J*dw/dt = -b*w - tauc*sign(w)。

    同时给出"纯指数"估计（ln|w| 斜率）作为对照：纯指数会忽略库仑摩擦，
    库仑显著时它会把 b 估大，所以以含库仑的 ODE 拟合为准。
    """
    if args.J is None:
        return None, "decay: 需要 --J（已知转动惯量）"
    segs = []
    start = None
    for i in range(len(tau)):
        if abs(tau[i]) <= args.zero_tol and start is None:
            start = i
        elif abs(tau[i]) > args.zero_tol and start is not None:
            segs.append((start, i))
            start = None
    if start is not None:
        segs.append((start, len(tau)))

    w = moving_average(omega, args.smooth)
    dw = derivative(t, w)
    X, y, used = [], [], 0
    logX, logY = [], []
    for a, b in segs:
        if b - a < 10:
            continue
        used += 1
        for i in range(a, b):
            if abs(w[i]) > args.omega_floor:
                X.append([w[i], sign(w[i])])           # dw = -(b/J)w - (tauc/J)sign(w)
                y.append(dw[i])
                logX.append([t[i], 1.0])
                logY.append(math.log(abs(w[i])))
    if len(X) < 10:
        return None, "decay: 没找到足够长的 tau~0 段（或 |w| 一直低于 --omega-floor）"
    beta, r2 = lstsq(X, y)
    if beta is None:
        return None, "decay: 拟合奇异"
    b = -beta[0] * args.J
    tau_c = -beta[1] * args.J
    b_log = None
    blog_fit, _ = lstsq(logX, logY)
    if blog_fit is not None:
        b_log = -blog_fit[0] * args.J
    return {"b": b, "tau_c": tau_c, "r2": r2, "segments": used, "b_logdecay": b_log}, None


# =====================================================================
# SVG 拟合图（标准库）
# =====================================================================

def write_svg(path, t, tau, omega, steady_points, b, tau_c):
    W, H = 900, 720
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
             'viewBox="0 0 %d %d" font-family="sans-serif" font-size="12">' % (W, H, W, H),
             '<rect width="%d" height="%d" fill="white"/>' % (W, H)]

    def frame_(x0, y0, x1, y1, title, xl, yl):
        parts.append('<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#888"/>'
                     % (x0, y0, x1 - x0, y1 - y0))
        parts.append('<text x="%d" y="%d" fill="#333">%s</text>' % (x0, y0 - 6, title))
        parts.append('<text x="%d" y="%d" text-anchor="middle" fill="#333">%s</text>'
                     % ((x0 + x1) // 2, y1 + 30, xl))
        parts.append('<text x="%d" y="%d" fill="#333" transform="rotate(-90 %d %d)" '
                     'text-anchor="middle">%s</text>'
                     % (x0 - 40, (y0 + y1) // 2, x0 - 40, (y0 + y1) // 2, yl))

    pxs = [p[0] for p in steady_points] or omega
    pys = [p[1] for p in steady_points] or tau
    xmin, xmax = min(pxs), max(pxs)
    ymin, ymax = min(pys), max(pys)
    xmin -= (xmax - xmin) * 0.1 + 0.1
    xmax += (xmax - xmin) * 0.1 + 0.1
    ymin -= (ymax - ymin) * 0.1 + 0.01
    ymax += (ymax - ymin) * 0.1 + 0.01
    X0, X1, Y0, Y1 = 70, 870, 40, 300
    sx = lambda v: X0 + (v - xmin) / (xmax - xmin) * (X1 - X0)
    sy = lambda v: Y1 - (v - ymin) / (ymax - ymin) * (Y1 - Y0)
    frame_(X0, Y0, X1, Y1, "Tau vs omega (steady-state fit)", "omega [rad/s]", "tau [N.m]")
    for p in steady_points:
        parts.append('<circle cx="%.1f" cy="%.1f" r="3" fill="#bbb"/>' % (sx(p[0]), sy(p[1])))
    parts.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#c33" stroke-width="2"/>'
                 % (sx(xmin), sy(b * xmin + tau_c), sx(xmax), sy(b * xmax + tau_c)))
    for p in steady_points:
        parts.append('<circle cx="%.1f" cy="%.1f" r="4" fill="#06c"/>' % (sx(p[0]), sy(p[1])))
    parts.append('<text x="%d" y="%d" fill="#c33">tau = %.4f*omega + %.4f</text>'
                 % (X0 + 8, Y0 + 16, b, tau_c))

    X0, X1, Y0, Y1 = 70, 870, 370, 660
    tmin, tmax = t[0], t[-1]
    frame_(X0, Y0, X1, Y1, "tau & omega vs time", "t [s]",
           "tau[N.m] blue / omega[rad/s] green")
    omin, omax = min(omega), max(omega)
    tmn, tmx = min(tau), max(tau)
    tx = (lambda v: X0 + (v - tmin) / (tmax - tmin) * (X1 - X0)) if tmax > tmin else (lambda v: X0)
    ty_t = (lambda v: Y1 - (v - tmn) / (tmx - tmn) * (Y1 - Y0)) if tmx > tmn else (lambda v: Y0)
    ty_o = (lambda v: Y1 - (v - omin) / (omax - omin) * (Y1 - Y0)) if omax > omin else (lambda v: Y0)
    step = max(1, len(t) // 2000)
    idx = range(0, len(t), step)
    parts.append('<polyline fill="none" stroke="#06c" stroke-width="1.5" points="' +
                 " ".join("%.1f,%.1f" % (tx(t[i]), ty_t(tau[i])) for i in idx) + '"/>')
    parts.append('<polyline fill="none" stroke="#090" stroke-width="1.5" points="' +
                 " ".join("%.1f,%.1f" % (tx(t[i]), ty_o(omega[i])) for i in idx) + '"/>')
    parts.append('</svg>')
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(parts))


# =====================================================================
# 合成数据自检
# =====================================================================

def make_demo():
    import random
    rng = random.Random(7)
    J, b, tc = 0.012, 0.045, 0.06
    dt = 0.001
    profile = ([0.0] * 500 + [0.5] * 1500 + [1.0] * 1500 + [1.6] * 1500
               + [2.0] * 1500 + [1.0] * 1000 + [0.0] * 2500)
    t, tau, omega = [], [], []
    w = 0.0
    for k, T in enumerate(profile):
        tau.append(T)
        t.append(k * dt)
        omega.append(w)
        for _ in range(10):
            w += ((T - b * w - tc * sign(w)) / J) * (dt / 10.0)
        w += rng.gauss(0.0, 0.02)
    return t, tau, omega, (J, b, tc)


# =====================================================================
# main
# =====================================================================

def main() -> int:
    p = argparse.ArgumentParser(description="云台单轴摩擦辨识（b=粘滞, tauc=库仑）",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("data", nargs="?", help="数据文件（CSV/文本 或 VOFA 二进制）")
    p.add_argument("--J", type=float, default=None, help="已知转动惯量 kg·m^2（decay 必需）")
    p.add_argument("--format", choices=["auto", "text", "vofa"], default="auto")
    p.add_argument("--channels", type=int, default=0, help="VOFA 每帧 float 个数")
    p.add_argument("--time-col", type=int, default=None, help="时间列号(0基)")
    p.add_argument("--tau-col", type=int, default=None, help="力矩列号")
    p.add_argument("--omega-col", type=int, default=None, help="角速度列号")
    p.add_argument("--dt", type=float, default=0.001, help="无时间列时的采样周期 s")
    p.add_argument("--skip", type=int, default=0, help="跳过前 N 个数据行（丢掉旧记录/垃圾段）")
    p.add_argument("--take", type=int, default=0, help="最多取 N 个数据行（0 = 到结尾）")
    p.add_argument("--omega-deg-per-s", action="store_true", help="角速度数据是 deg/s")
    p.add_argument("--methods", default="vsweep,integ,steady,ode,decay", help="要跑的方法，逗号分隔")
    p.add_argument("--smooth", type=int, default=5, help="ode 求导前移动平均窗口")
    p.add_argument("--plateau-tol", type=float, default=0.02, help="steady: 力矩台阶容差 N·m")
    p.add_argument("--plateau-min", type=int, default=100, help="steady: 台阶最少样本数")
    p.add_argument("--zero-tol", type=float, default=0.05, help="decay: 判 tau~0 的阈值 N·m")
    p.add_argument("--omega-floor", type=float, default=0.3, help="decay: |w| 下限 rad/s")
    p.add_argument("--vs-tol", type=float, default=0.2, help="vsweep: w 平台容差 rad/s")
    p.add_argument("--vs-zero", type=float, default=0.2, help="vsweep: 忽略 |w| 低于此值的档")
    p.add_argument("--plot", default=None, help="把拟合图写成 SVG")
    p.add_argument("--demo", action="store_true", help="用合成数据自检，不用数据文件")
    args = p.parse_args()

    if args.demo:
        t, tau, omega, truth = make_demo()
        args.J = truth[0]
        print("[demo] 真值: J=%.4f  b=%.4f  tauc=%.4f  (%d 采样, %.1fs)"
              % (truth[0], truth[1], truth[2], len(t), t[-1]))
    else:
        if not args.data:
            p.error("要么给数据文件，要么加 --demo")
        rows = load_rows(args.data, args.format, args.channels)
        if args.skip:
            rows = rows[args.skip:]
        if args.take:
            rows = rows[:args.take]
        tc, tauc, wc = pick_columns(rows, args)
        scale = math.pi / 180.0 if args.omega_deg_per_s else 1.0
        t, tau, omega = extract(rows, tc, tauc, wc, args.dt, scale)
        if len(t) < 10:
            print("有效样本太少，检查列号 / 数据格式", file=sys.stderr)
            return 2
        print("载入 %d 个样本，时长 %.3fs，tau in [%.3f, %.3f]，w in [%.3f, %.3f]"
              % (len(t), t[-1] - t[0], min(tau), max(tau), min(omega), max(omega)))

    wanted = [m.strip() for m in args.methods.split(",") if m.strip()]
    results = {}
    for name in wanted:
        if name == "steady":
            res, err = method_steady(t, tau, omega, args)
        elif name == "vsweep":
            res, err = method_vsweep(t, tau, omega, args)
        elif name == "integ":
            res, err = method_integ(t, tau, omega, args)
        elif name == "ode":
            res, err = method_ode(t, tau, omega, args)
        elif name == "decay":
            res, err = method_decay(t, tau, omega, args)
        else:
            res, err = None, "未知方法 %s" % name
        if res:
            results[name] = res
        else:
            print("%-6s: 跳过 —— %s" % (name, err))

    print("\n===== 辨识结果 =====")
    for name in wanted:
        if name not in results:
            continue
        r = results[name]
        extra = ("  J=%.5f" % r["J"]) if r.get("J") is not None else ""
        tc_s = "n/a" if math.isnan(r["tau_c"]) else ("%.5f" % r["tau_c"])
        print("%-6s: b = %.6f N·m·s/rad   tauc = %s N·m   R2=%.4f%s"
              % (name, r["b"], tc_s, r["r2"], extra))
        if r.get("b_logdecay") is not None:
            print("        （对照：纯指数拟合 b = %.6f；库仑显著时它会偏大）"
                  % r["b_logdecay"])
    primary = "vsweep" if "vsweep" in results else ("steady" if "steady" in results else None)
    if primary and "ode" in results:
        bs, bo = results[primary]["b"], results["ode"]["b"]
        if abs(bs) > 1e-9 and abs(bs - bo) / abs(bs) > 0.3:
            print("提示: %s 与 ode 的 b 差得较多，检查库仑摩擦/低速段/线束拖拽。" % primary)
    if primary:
        print("\n建议采用 %s 的 b = %.6f N·m·s/rad" % (primary, results[primary]["b"]))

    if args.plot and results:
        base = primary or next(iter(results))
        b = results[base]["b"]
        tau_c = 0.0 if math.isnan(results[base]["tau_c"]) else results[base]["tau_c"]
        pts = results[base].get("points", [])
        write_svg(args.plot, t, tau, omega, pts, b, tau_c)
        print("拟合图已写出: %s" % args.plot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
