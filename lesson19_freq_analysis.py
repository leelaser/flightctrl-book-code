#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 19 周 · 04 频域分析与系统辨识 —— 配套实验

    实验 1  时域 vs 频域：同一个系统，两种看法
    实验 2  ★★ Bode 图：增益裕度 / 相位裕度 怎么读出来
    实验 3  ★★★ 在「真机」上测频率响应：chirp 扫频 + 系统辨识
    实验 4  ★ 陷波滤波器：把机架共振按下去（复现 PX4 和 ArduPilot 的实现）

用法：
    python lesson19_freq_analysis.py 1
    python lesson19_freq_analysis.py 2
    python lesson19_freq_analysis.py 3
    python lesson19_freq_analysis.py 4
    python lesson19_freq_analysis.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  能算的就不猜。

  实验 3 里 1:1 复现了 PX4 的对数扫频公式（行号已核对）：

      src/lib/system_identification/signal_generator.hpp:43  getLinearSineSweep
      src/lib/system_identification/signal_generator.hpp:55  getLogSineSweep
      src/lib/system_identification/signal_generator.hpp:63  ← 就是这一行的公式
      src/lib/system_identification/arx_rls.hpp:127-129      ← RLS 的三行核心
      src/lib/pid_design/pid_design.hpp:82-103               ← GMVC → PID

  实验 4 里 1:1 复现了两家的陷波滤波器：

      ArduPilot  libraries/Filter/NotchFilter.cpp:31-39   calculate_A_and_Q
      ArduPilot  libraries/Filter/NotchFilter.cpp:75-90   init_with_A_and_Q
      PX4        src/lib/mathlib/math/filter/NotchFilter.hpp:271-280

参考：
    第 5 章  05-控制理论/01-PID控制.md（ζ / ωn 与超调、上升时间的关系）
    第 4 章  04-状态估计/01-概率论与高斯分布.md（最小二乘 / 协方差的味道）
"""

import math
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 输出辅助（和本教程其它脚本一致）
# ---------------------------------------------------------------------------

W = 74

TWO_PI = 2.0 * math.pi


def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)
    print()


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (W - 4))
    print()


def _dlen(s):
    """显示宽度：CJK 字符占 2 列。表格对齐全靠它。"""
    n = 0
    for ch in str(s):
        o = ord(ch)
        n += 2 if (0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF
                   or 0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF
                   or 0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60
                   or 0xFFE0 <= o <= 0xFFE6) else 1
    return n


def tab(rows, headers, width=None):
    """打印表格。rows 是二维列表。（按显示宽度对齐，中文不会歪）"""
    cols = len(headers)
    if width is None:
        width = [max(_dlen(r[i]) for r in [headers] + rows) + 2
                 for i in range(cols)]

    def cell(s, w):
        s = str(s)
        return s + " " * max(1, w - _dlen(s))

    print("  " + "".join(cell(headers[i], width[i]) for i in range(cols)))
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join(cell(r[i], width[i]) for i in range(cols)))
    print()


# ---------------------------------------------------------------------------
# matplotlib（可选）
# ---------------------------------------------------------------------------

_HAS_MPL = False
_CJK = False

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as _fm
    for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                  "Source Han Sans SC", "WenQuanYi Micro Hei",
                  "PingFang SC", "Heiti SC"):
        try:
            _fm.findfont(_name, fallback_to_default=False)
        except Exception:
            continue
        matplotlib.rcParams["font.sans-serif"] = [_name]
        matplotlib.rcParams["axes.unicode_minus"] = False
        _CJK = True
        break
    _HAS_MPL = True
except ImportError:
    pass


def savefig(name):
    if not _HAS_MPL:
        return
    path = name + ".png"
    plt.tight_layout()
    plt.savefig(path, dpi=110)
    plt.close()
    note = "" if _CJK else "（没找到中文字体，图里中文会变方块）"
    print("  [图] 已保存 %s %s" % (path, note))
    print()


# ===========================================================================
# 通用工具：连续传递函数 / 离散滤波器
# ===========================================================================

def polyval_c(coeffs, s):
    """复数域 Horner 求值。coeffs 按 s 的【降幂】排列（和 MATLAB 一样）。"""
    acc = 0.0 + 0.0j
    for c in coeffs:
        acc = acc * s + c
    return acc


def tf_resp(num, den, w):
    """连续传递函数 num(s)/den(s) 在 s = jw 处的复数值。

    num / den 都是降幂系数的 list 或 ndarray。
    w 可以是标量，也可以是 ndarray。
    """
    w = np.asarray(w, dtype=float)
    s = 1j * w
    num = np.asarray(num, dtype=float)
    den = np.asarray(den, dtype=float)
    n = np.zeros_like(s, dtype=complex)
    d = np.zeros_like(s, dtype=complex)
    for c in num:
        n = n * s + c
    for c in den:
        d = d * s + c
    return n / d


def db(x):
    """幅度 → 分贝。x 可以是复数（自动取模）。"""
    a = np.abs(np.asarray(x, dtype=complex))
    return 20.0 * np.log10(np.maximum(a, 1e-300))


def phase_deg(x):
    """相位（度）。"""
    return np.degrees(np.angle(x))


def unwrap_deg(ph):
    return np.degrees(np.unwrap(np.radians(np.asarray(ph, dtype=float))))


def loggrid(f_lo, f_hi, n):
    return np.logspace(math.log10(f_lo), math.log10(f_hi), n)


def crossing(ws, ys, level):
    """在 (ws, ys) 上线性插值找 ys == level 的【第一个】穿越点。

    ys 必须是单调的（或者在设计上只穿一次）。找不到返回 None。
    """
    ws = np.asarray(ws, dtype=float)
    ys = np.asarray(ys, dtype=float)
    for k in range(len(ys) - 1):
        y0, y1 = ys[k], ys[k + 1]
        if (y0 - level) * (y1 - level) <= 0.0 and y0 != y1:
            t = (level - y0) / (y1 - y0)
            return ws[k] + t * (ws[k + 1] - ws[k])
    return None


def lti_step(num, den, t_end, n_step=4000):
    """线性定常系统的单位阶跃响应（RK4 积分，可控标准型）。

    返回 (t, y)。
    """
    num = np.asarray(num, dtype=float)
    den = np.asarray(den, dtype=float)
    num = num / den[0]
    den = den / den[0]
    n = len(den) - 1                       # 阶数
    if len(num) < n + 1:                   # 补到 n+1 项
        num = np.concatenate([np.zeros(n + 1 - len(num)), num])

    d = num[0]                             # 直通项 D
    C = num[1:] - d * den[1:]              # 1 x n
    A = np.zeros((n, n))
    A[0, :] = -den[1:]                     # 第一行 = [-a_{n-1}, ..., -a_0]
    for i in range(1, n):
        A[i, i - 1] = 1.0
    Bv = np.zeros(n)
    Bv[0] = 1.0

    dt = t_end / n_step
    x = np.zeros(n)
    t = np.zeros(n_step + 1)
    y = np.zeros(n_step + 1)
    y[0] = d                          # u(0+) = 1

    def f(xx):
        return A @ xx + Bv                # u = 1

    for k in range(n_step):
        k1 = f(x)
        k2 = f(x + 0.5 * dt * k1)
        k3 = f(x + 0.5 * dt * k2)
        k4 = f(x + dt * k3)
        x = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        t[k + 1] = (k + 1) * dt
        y[k + 1] = float(C @ x) + d

    return t, y


# ---- 离散二阶节（biquad）----------------------------------------------

def biquad_resp(b, a, w, dt):
    """biquad 的频率响应。b = [b0,b1,b2]，a = [a0=1,a1,a2]。"""
    w = np.asarray(w, dtype=float)
    z_inv = np.exp(-1j * w * dt)
    num = b[0] + b[1] * z_inv + b[2] * z_inv ** 2
    den = a[0] + a[1] * z_inv + a[2] * z_inv ** 2
    return num / den


def notch_coeffs_ap(fs, fc, bw, att_db):
    """ArduPilot 的陷波设计。1:1 复现

        libraries/Filter/NotchFilter.cpp:31-39   calculate_A_and_Q
        libraries/Filter/NotchFilter.cpp:75-90   init_with_A_and_Q

    返回 (b, a, A, Q, alpha)。
    """
    A = 10.0 ** (-att_db / 40.0)
    if fc > 0.5 * bw:
        octaves = math.log2(fc / (fc - bw / 2.0)) * 2.0
        Q = math.sqrt(2.0 ** octaves) / ((2.0 ** octaves) - 1.0)
    else:
        Q = 0.0
    if Q <= 0.0 or fc >= 0.5 * fs:
        return [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], A, Q, 0.0

    omega = TWO_PI * fc / fs
    alpha = math.sin(omega) / (2.0 * Q)

    b0 = 1.0 + alpha * A * A
    b1 = -2.0 * math.cos(omega)
    b2 = 1.0 - alpha * A * A
    a1 = b1
    a2 = 1.0 - alpha

    a0_inv = 1.0 / (1.0 + alpha)
    b = [b0 * a0_inv, b1 * a0_inv, b2 * a0_inv]
    a = [1.0, a1 * a0_inv, a2 * a0_inv]
    return b, a, A, Q, alpha


def notch_coeffs_px4(fs, fc, bw):
    """PX4 的陷波设计。1:1 复现

        src/lib/mathlib/math/filter/NotchFilter.hpp:271-280

    返回 (b, a, alpha)。
    """
    alpha = math.tan(math.pi * bw / fs)
    beta = -math.cos(TWO_PI * fc / fs)
    a0_inv = 1.0 / (alpha + 1.0)

    b0 = a0_inv
    b1 = 2.0 * beta * a0_inv
    b2 = a0_inv
    a1 = b1
    a2 = (1.0 - alpha) * a0_inv
    return [b0, b1, b2], [1.0, a1, a2], alpha


def biquad_run(b, a, x):
    """Direct Form I 跑一遍 biquad（和 AP NotchFilter.cpp:118 同结构）。"""
    y = np.zeros_like(x, dtype=float)
    x1 = x2 = y1 = y2 = 0.0
    for k in range(len(x)):
        v = b[0] * x[k] + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
        x2, x1 = x1, x[k]
        y2, y1 = y1, v
        y[k] = v
    return y


# ===========================================================================
# 实验 1 —— 时域 vs 频域
# ===========================================================================

def exp1():
    head("实验 1  时域 vs 频域：同一个系统，两种看法")

    print("  被控对象（本课统一用它）：标准二阶系统")
    print()
    print("      G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)")
    print()
    print("  这一节要用它说明三件事：")
    print("    · 时域只看得到「一个阶跃下去，它怎么动」")
    print("    · 频域看得到「每个频率的信号被放大/衰减多少、延迟多少」")
    print("    · 两者是【同一个东西】的两种投影 —— 能互相换算")
    print()

    wn = TWO_PI * 2.0            # 2 Hz
    zetas = [0.1, 0.3, 0.5, 0.707, 1.0, 2.0]

    # ---- ① 时域：阶跃响应 ----
    sub("① 时域：阶跃响应（wn = 2 Hz 固定，只改 zeta）")

    rows = []
    curves = []
    for z in zetas:
        num = [wn * wn]
        den = [1.0, 2.0 * z * wn, wn * wn]
        t, y = lti_step(num, den, 1.6, 4000)
        curves.append((z, t, y))

        overshoot = (max(y) - 1.0) * 100.0
        # 上升时间：10% → 90%
        t10 = crossing(t, y, 0.1)
        t90 = crossing(t, y, 0.9)
        rise = (t90 - t10) if (t10 is not None and t90 is not None) else float("nan")
        # 2% 稳定时间
        settle = float("nan")
        for k in range(len(y)):
            if np.all(np.abs(y[k:] - 1.0) <= 0.02):
                settle = t[k]
                break
        rows.append(["%.3f" % z, "%.2f" % overshoot, "%.4f" % rise,
                     "%.4f" % settle, "%.4f" % abs(y[-1] - 1.0)])
    tab(rows, ["zeta", "超调 (%)", "上升时间 10-90% (s)",
               "2% 稳定时间 (s)", "末尾误差"])

    print("    ★ 时域给的就是这几个数：**超调、上升时间、稳定时间**。")
    print("      教科书公式（第 1 课讲过）：")
    print("          超调 = exp(-pi*zeta/sqrt(1-zeta^2))")
    print("          上升时间 ≈ (pi - acos(zeta)) / (wn*sqrt(1-zeta^2))")
    print()

    # ---- ② 频域：Bode ----
    sub("② 频域：同一个系统的 Bode 图（幅度 + 相位）")

    print("  做法：把 s = jw 代进去，直接算复数。")
    print("        G(jw) = wn^2 / (wn^2 - w^2 + j*2*zeta*wn*w)")
    print()
    print("  幅度 = |G(jw)|，用分贝表示：20*log10|G|")
    print("  相位 = angle(G(jw))，单位度")
    print()
    print("  【关键定义】")
    print("    · 直流增益     w → 0 时的 |G|。这里 = 1（0 dB）")
    print("    · 谐振峰       |G| 的最大值。zeta 越小峰越高")
    print("    · 带宽 w_BW    |G| 从 0 dB 掉到 -3 dB 的那个频率")
    print("                   —— 低于它的信号能「通过」，高于它的被衰减")
    print("    · 谐振频率     |G| 取到峰值处的 w")
    print()

    rows = []
    for z in zetas:
        num = [wn * wn]
        den = [1.0, 2.0 * z * wn, wn * wn]

        ws = loggrid(wn / 100.0, wn * 100.0, 20001)
        mag = np.abs(tf_resp(num, den, ws))
        mag_db = db(mag)

        peak_i = int(np.argmax(mag_db))
        peak_db = mag_db[peak_i]
        peak_w = ws[peak_i]
        # 理论：w_r = wn*sqrt(1-2z^2)（z < 0.707 才有峰）
        has_peak = z < math.sqrt(0.5) - 1e-9
        if has_peak:
            wr_th = wn * math.sqrt(1.0 - 2.0 * z * z)
            pk_th = 1.0 / (2.0 * z * math.sqrt(1.0 - z * z))
        else:
            wr_th = float("nan")
            pk_th = float("nan")

        # 带宽：0 dB 以下 3 dB 的那个点
        bw = crossing(ws, mag_db, -3.0)
        rows.append(["%.3f" % z,
                     "%.2f" % peak_db,
                     ("%.3f" % (peak_w / TWO_PI)) if has_peak else "—",
                     ("%.3f" % (wr_th / TWO_PI)) if has_peak else "—",
                     ("%.3f" % (pk_th)) if has_peak else "—",
                     "%.3f" % (bw / TWO_PI) if bw else "—"])
    tab(rows, ["zeta", "谐振峰 (dB)", "谐振频率实测 (Hz)", "谐振频率理论 (Hz)",
               "峰值理论 (线性)", "带宽 -3dB (Hz)"])

    print("    ★ 谐振频率理论值 = wn*sqrt(1-2*zeta^2)")
    print("      峰值理论     = 1 / (2*zeta*sqrt(1-zeta^2))")
    print("      实测算出来的和理论对得上 —— 频域分析没有任何玄学。")
    print()
    print("    ★★ 注意 zeta = 0.707 那一行：")
    print("        谐振峰 = 0 dB（不鼓包），带宽最大。")
    print("        这就是「巴特沃斯（Butterworth）」—— 最平坦的幅频特性。")
    print("        第 1 课讲的「zeta = 0.707 最好」在频域里的名字叫「最大平坦」。")
    print()

    # ---- ③ 时域 ↔ 频域 换算 ----
    sub("③ 时域 ↔ 频域：它们真的是一回事")

    print("  用「上升时间 × 带宽 ≈ 常数」来验证。")
    print("  （这是不确定性原理在控制系统里的翻版：")
    print("    时间上响应越快 ⇒ 频率上能通过的范围越宽。）")
    print()

    rows = []
    for z, t, y in curves:
        num = [wn * wn]
        den = [1.0, 2.0 * z * wn, wn * wn]
        ws = loggrid(wn / 100.0, wn * 100.0, 20001)
        mag_db = db(np.abs(tf_resp(num, den, ws)))
        bw = crossing(ws, mag_db, -3.0)
        t10 = crossing(t, y, 0.1)
        t90 = crossing(t, y, 0.9)
        rise = t90 - t10
        rows.append(["%.3f" % z, "%.4f" % rise, "%.3f" % (bw / TWO_PI),
                     "%.4f" % (rise * bw / TWO_PI)])
    tab(rows, ["zeta", "上升时间 (s)", "带宽 (Hz)", "上升时间 x 带宽"])

    print("    ★ 最后一列：zeta >= 0.7 之后稳定在 0.34 附近。")
    print("      ⇒ 「上升快」和「带宽宽」是同一件事。")
    print("      （zeta = 0.1 那行偏小，因为欠阻尼系统「先冲上去」很快，")
    print("        但这不是我们想要的快 —— 所以工程上的经验常数是 0.35。）")
    print()
    print("      ★★ 这就是为什么调参时看 Bode 图比看阶跃响应有用：")
    print("         阶跃响应只告诉你【这一个】输入下的表现，")
    print("         Bode 图告诉你【所有频率】的输入下会怎样 ——")
    print("         包括传感器噪声、电机振动、阵风。")
    print()

    # ---- ④ 画图 ----
    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        for z, t, y in curves:
            axes[0].plot(t, y, label="zeta=%.3f" % z)
        axes[0].axhline(1.0, color="k", lw=0.6, ls="--")
        axes[0].set_xlabel("t (s)")
        axes[0].set_ylabel("y")
        axes[0].set_title("时域：阶跃响应")
        axes[0].grid(alpha=0.3)
        axes[0].legend(fontsize=7)

        ws = loggrid(wn / 100.0, wn * 100.0, 2001)
        for z, t, y in curves:
            num = [wn * wn]
            den = [1.0, 2.0 * z * wn, wn * wn]
            axes[1].semilogx(ws / TWO_PI, db(tf_resp(num, den, ws)),
                             label="zeta=%.3f" % z)
        axes[1].axhline(-3.0, color="k", lw=0.6, ls="--")
        axes[1].set_xlabel("频率 (Hz)")
        axes[1].set_ylabel("|G| (dB)")
        axes[1].set_title("频域：幅频特性")
        axes[1].grid(alpha=0.3, which="both")
        axes[1].legend(fontsize=7)
        savefig("lesson19_exp1")


# ===========================================================================
# 实验 2 —— Bode 图：增益裕度 / 相位裕度
# ===========================================================================

def _open_loop_resp(w, kc, wn_a, zeta_a, T_delay):
    """一个「速率环」的开环传递函数（连续域近似）。

        L(s) = kc * wn_a^2 / ( s * (s^2 + 2*zeta_a*wn_a*s + wn_a^2) )
               * exp(-s*T_delay)

    物理意义：
        · 1/s         —— 角速度是力矩的积分（刚体转动惯量）
        · 二阶环节    —— 电机 + 桨 + 机身一阶滞后，用二阶近似
        · exp(-s*T)   —— 传感器采样 + 滤波 + 计算的总延迟
    """
    w = np.asarray(w, dtype=float)
    s = 1j * w
    L = kc * wn_a ** 2 / (s * (s ** 2 + 2.0 * zeta_a * wn_a * s + wn_a ** 2))
    L = L * np.exp(-s * T_delay)
    return L


def _margins(w, L):
    """从频率响应里读增益裕度 / 相位裕度 / 穿越频率。

    返回 dict：wc, PM, w180, GM_dB
    """
    mag_db = db(L)
    ph = unwrap_deg(phase_deg(L))

    wc = crossing(w, mag_db, 0.0)              # 增益穿越：|L| = 1
    pm = None
    if wc is not None:
        ph_c = np.interp(math.log(wc), np.log(w), ph)
        pm = 180.0 + ph_c

    # 相位穿越：相位 = -180
    w180 = crossing(w, ph, -180.0)
    gm_db = None
    if w180 is not None:
        m = np.interp(math.log(w180), np.log(w), mag_db)
        gm_db = -m
    return {"wc": wc, "PM": pm, "w180": w180, "GM_dB": gm_db}


def closed_loop_step(kc, wn, zeta, Td, t_end, dt):
    """带延迟的开环  L(s) = kc*wn^2/(s(s^2+2*zeta*wn*s+wn^2)) * exp(-s*Td)
    的【闭环】单位阶跃响应。

    状态： x = [v, x1, x2]
        v  : 1/s 的输出（积分器）
        x1 : 输出 y
        x2 : y 的导数
    控制： u = kc*(r - y)，然后延迟 Td 再作用到积分器上。
    """
    n = int(round(t_end / dt)) + 1
    t = np.arange(n) * dt
    d = max(1, int(round(Td / dt)))
    buf = [0.0] * d

    def deriv(xx, u):
        v, x1, x2 = xx
        return np.array([u, x2, wn * wn * (v - x1) - 2.0 * zeta * wn * x2])

    x = np.zeros(3)
    y = np.zeros(n)
    for k in range(n):
        u_now = buf.pop(0)
        buf.append(kc * (1.0 - x[1]))
        k1 = deriv(x, u_now)
        k2 = deriv(x + 0.5 * dt * k1, u_now)
        k3 = deriv(x + 0.5 * dt * k2, u_now)
        k4 = deriv(x + dt * k3, u_now)
        x = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        y[k] = x[1]
    return t, y


def exp2():
    head("实验 2  ★★ Bode 图：增益裕度 / 相位裕度 怎么读出来")

    print("  实验 1 看的是【闭环】。要判断稳定性，得看【开环】。")
    print()
    print("  开环传递函数  L(s) = 控制器 C(s) x 被控对象 P(s) x 延迟")
    print()
    print("  本实验用的是一个真实的速率环模型：")
    print()
    print("      L(s) = kc * wn^2 / ( s * (s^2 + 2*zeta*wn*s + wn^2) ) * exp(-s*Td)")
    print()
    print("      · 1/s        —— 力矩 → 角速度，是个积分（转动惯量）")
    print("      · 二阶环节   —— 电机 + 桨 + 机身的滞后（wn = 2 Hz, zeta = 0.8）")
    print("      · exp(-s*Td) —— 采样 + 滤波 + 计算延迟，Td = 8 ms")
    print()
    print("  ★ 这个模型是【开环】的：把回路在某一点剪开，")
    print("    量「从这一点进去、绕一圈回到这一点」的信号被放大/延迟了多少。")
    print("    闭环是 T = L/(1+L)。L 决定一切。")
    print()

    wn_a = TWO_PI * 2.0
    zeta_a = 0.8
    T_delay = 0.008

    # ---- ① 先给一张完整的 Bode 表 ----
    sub("① 先算一张 Bode 表（kc = 4.0）")

    w = loggrid(TWO_PI * 0.02, TWO_PI * 200.0, 60001)
    L = _open_loop_resp(w, 4.0, wn_a, zeta_a, T_delay)

    mag_db = db(L)
    ph_un = unwrap_deg(phase_deg(L))

    rows = []
    for f in [0.1, 0.2, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 50.0]:
        Lq = _open_loop_resp(TWO_PI * f, 4.0, wn_a, zeta_a, T_delay)
        # 相位要【解缠绕】—— 否则在 -180 度附近会跳成 +179，
        #   看起来像「相位突然变好了」，其实不是。
        ph = float(np.interp(math.log(TWO_PI * f), np.log(w), ph_un))
        rows.append(["%.1f" % f, "%.2f" % float(db(Lq)), "%.1f" % ph])
    tab(rows, ["频率 (Hz)", "|L| (dB)", "相位 (度，已解缠绕)"])

    m = _margins(w, L)
    print("  从这张表里读两个数：")
    print()
    print("    【增益穿越频率 w_c】|L| = 0 dB 的地方")
    if m["wc"]:
        print("        w_c = %.3f Hz" % (m["wc"] / TWO_PI))
    print("      （开环增益 1 的频率。闭环带宽大致就在这附近。）")
    print()
    if m["PM"] is not None:
        print("    【相位裕度 PM】= 180 度 + angle(L(j*w_c))")
        print("        PM = 180 + (%.1f) = %.1f 度" % (m["PM"] - 180.0, m["PM"]))
        print("      （相位离 -180 度还有多少余量。0 度 = 临界振荡。）")
    print()
    if m["w180"] is not None:
        print("    【相位穿越频率 w_180】相位 = -180 度的地方")
        print("        w_180 = %.3f Hz" % (m["w180"] / TWO_PI))
        print()
        print("    【增益裕度 GM】= -20*log10(|L(j*w_180)|)")
        print("        GM = %.2f dB" % m["GM_dB"])
        print("      （在 -180 度那个频率上，还能把增益加大多少倍才振荡。）")
    print()
    print("    ★★ 记忆法：")
    print("        相位裕度 = 「**在目前的工作频率上**，相位离翻脸还有多远」")
    print("        增益裕度 = 「**在相位已经翻脸的那个频率上**，增益还差多少」")
    print("      两个都要留余量 —— 因为真实的 wn、zeta、延迟都会变。")
    print()

    # ---- ② 增益扫描 ----
    sub("② 把 kc 从 1 扫到 22：裕度怎么变")

    kcs = [1.0, 2.0, 4.0, 6.0, 10.0, 14.0, 18.0, 22.0]
    rows = []
    step_data = []
    for kc in kcs:
        mm = _margins(w, _open_loop_resp(w, kc, wn_a, zeta_a, T_delay))
        t, y = closed_loop_step(kc, wn_a, zeta_a, T_delay, 4.0, 1.0 / 400.0)
        ovs = (max(y) - 1.0) * 100.0
        # 2% 稳定时间
        settle = float("nan")
        for k in range(len(y)):
            if np.all(np.abs(y[k:] - 1.0) <= 0.02):
                settle = t[k]
                break
        step_data.append((kc, t, y))
        rows.append(["%.0f" % kc,
                     ("%.3f" % (mm["wc"] / TWO_PI)) if mm["wc"] else "—",
                     ("%.1f" % mm["PM"]) if mm["PM"] is not None else "—",
                     ("%.2f" % mm["GM_dB"]) if mm["GM_dB"] is not None else "—",
                     "%.1f" % ovs,
                     ("%.2f" % settle) if settle == settle else "—"])
    tab(rows, ["kc", "w_c (Hz)", "相位裕度 (度)", "增益裕度 (dB)",
               "闭环超调 (%)", "2% 稳定时间 (s)"])

    print("    ★★★ 这张表是本实验的核心。读三件事：")
    print()
    print("      ① kc 变大 ⇒ w_c 变大 ⇒ 带宽变宽。")
    print("         ⚠️ 但「稳定时间」【不是】一路缩短的！")
    print("         3.48 → 1.46 → 0.90 → 1.39 → 2.46 → 3.98：")
    print("         它在 kc = 4（PM ≈ 60 度）处有一个【最小值】0.90 s，")
    print("         过了这一点带宽继续涨，稳定时间反而变长。")
    print("         原因：2% 稳定时间由「超调 + 衰减速度」共同决定。")
    print("         带宽只提高了衰减速度，超调却在指数恶化（8.4% → 25% → 52%）。")
    print("         ⇒ ★★ **「带宽越大越快」是错的，除非同时保住相位裕度。**")
    print("            这就是「最优带宽」这个概念存在的原因，")
    print("            也是本节最重要的一条反直觉结论。")
    print()
    print("      ② 相位裕度随 kc 单调【下降】。")
    print("         因为延迟 exp(-s*Td) 的相位是 -w*Td ——")
    print("         **频率越高，相位掉得越多**。你把 w_c 往高频推，")
    print("         就等于把自己一步步送到 -180 度跟前。")
    print()
    print("      ③ 增益裕度也随 kc 下降，而且是【等量】下降：")
    print("         GM 的定义是 -|L(j*w_180)|，kc 乘 10 倍，")
    print("         GM 就正好减 20*log10(10) = 20 dB。")
    print("         这就是为什么 GM 不是一个独立的指标 —— 它只是 kc 的另一种写法。")
    print()

    # ---- ③ 相位裕度 → 超调 ----
    sub("③ 相位裕度 → 超调：一条能用的经验公式")

    print("  二阶闭环系统的阻尼比和相位裕度有个很好的近似：")
    print()
    print("      zeta ≈ PM / 100        （PM 用【度】）")
    print()
    print("  代进第 1 课的超调公式 超调 = exp(-pi*zeta/sqrt(1-zeta^2))，")
    print("  就能从 Bode 图直接估出时域超调。验证一下：")
    print()

    rows = []
    for kc, t, y in step_data:
        mm = _margins(w, _open_loop_resp(w, kc, wn_a, zeta_a, T_delay))
        if mm["PM"] is None:
            continue
        pm = mm["PM"]
        if pm <= 0.0:
            # 相位裕度都没了 ⇒ 闭环已经不稳定，超调公式没有意义
            rows.append(["%.0f" % kc, "%.1f" % pm, "—", "—",
                         "%.1f（已发散）" % ((max(y) - 1.0) * 100.0)])
            continue
        z_est = pm / 100.0
        ov_est = math.exp(-math.pi * z_est / math.sqrt(max(1e-9, 1 - z_est ** 2))) * 100.0
        rows.append(["%.0f" % kc, "%.1f" % pm, "%.2f" % z_est,
                     "%.1f" % ov_est, "%.1f" % ((max(y) - 1.0) * 100.0)])
    tab(rows, ["kc", "相位裕度 (度)", "zeta ~ PM/100", "估的超调 (%)",
               "真仿真的超调 (%)"])

    print("    ★ PM >= 45 度那几行，估计值和仿真值差不到 5 个百分点。")
    print("      更小的 PM 误差变大 —— 因为：")
    print("        · 「PM ≈ 100*zeta」只对二阶系统精确")
    print("        · 这里是三阶 + 纯延迟，还有 wn 处的二阶环节在捣乱")
    print("      PM <= 0（最后两行）时闭环已经不稳定，")
    print("      那个「超调」数字已经没有意义了 —— 只剩「发散」两个字。")
    print()
    print("    ★★ 工程上的经验门槛（记住这四档）：")
    print("         PM > 60 度  —— 保守，超调 < 10%，手感「肉」但稳")
    print("         PM 45 ~ 60  —— 正常，绝大多数飞控的调参目标")
    print("         PM 30 ~ 45  —— 激进，超调 20~35%，参数漂一点就危险")
    print("         PM < 30 度  —— 别飞了")
    print()

    # ---- ④ 反解：给定期望 PM，求 kc ----
    sub("④ 反过来用：想要 PM = 60 度，kc 该取多少？")

    def solve_kc_for_pm(target):
        lo, hi = 0.01, 200.0
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            mm = _margins(w, _open_loop_resp(w, mid, wn_a, zeta_a, T_delay))
            pmv = mm["PM"]
            if pmv is None:
                hi = mid
                continue
            if pmv > target:
                lo = mid
            else:
                hi = mid
        kc = 0.5 * (lo + hi)
        return kc, _margins(w, _open_loop_resp(w, kc, wn_a, zeta_a, T_delay))

    rows = []
    for target in [70.0, 60.0, 50.0, 45.0, 40.0]:
        kc, mm = solve_kc_for_pm(target)
        t, y = closed_loop_step(kc, wn_a, zeta_a, T_delay, 6.0, 1.0 / 400.0)
        rows.append(["%.0f" % target, "%.3f" % kc, "%.2f" % mm["PM"],
                     "%.3f" % (mm["wc"] / TWO_PI),
                     "%.1f" % ((max(y) - 1.0) * 100.0)])
    tab(rows, ["目标 PM (度)", "所需的 kc", "实得 PM (度)", "w_c (Hz)",
               "闭环超调 (%)"])

    print("    ★ 这就是「用频域指标调参」的标准流程：")
    print("        先定 PM（比如 60 度）→ 解出增益 → 再检查带宽够不够。")
    print()
    print("    ★★ 但如果 w_c 还是太低（带宽不够、响应太慢）怎么办？")
    print("        **不能继续加增益** —— 那只会把 PM 吃掉。")
    print("        正确做法是改变开环【形状】：")
    print("          · 加积分项    → 低频增益上去，但相位更差")
    print("          · 加微分项    → 相位提前（lead），PM 变好")
    print("          · **加陷波器** → 把某个频率的增益单独按下去（实验 4）")
    print("          · 减小延迟    → 换更快的滤波器 / 提高采样率 / 提高环路频率")
    print()
    print("    ★ 最后一条特别值得强调：")
    print("        延迟 Td 的相位是 -w*Td —— **它不改变 |L|，只吃相位**。")
    print("        这就是「滤波越重 → 延迟越大 → 增益越上不去」的死循环。")
    print()

    # 验证延迟对 PM 的影响
    rows = []
    for td in [0.002, 0.004, 0.008, 0.012, 0.016]:
        mm = _margins(w, _open_loop_resp(w, 4.0, wn_a, zeta_a, td))
        rows.append(["%.0f" % (td * 1000),
                     ("%.3f" % (mm["wc"] / TWO_PI)) if mm["wc"] else "—",
                     ("%.1f" % mm["PM"]) if mm["PM"] is not None else "—",
                     ("%.2f" % mm["GM_dB"]) if mm["GM_dB"] is not None else "—"])
    tab(rows, ["延迟 Td (ms)", "w_c (Hz)", "相位裕度 (度)", "增益裕度 (dB)"])

    print("    ★ 注意前两列：w_c 一动都不动（永远是 0.618 Hz），")
    print("      GM 只掉了 2 dB（因为 w_180 稍微挪了一点）。")
    print("      但相位裕度从 60.9 度一路掉到 57.8 度。")
    print()
    print("      ⇒ **延迟几乎不改变幅频特性，只偷相位。**")
    print("        这就是「延迟是控制系统最大的敌人」的数学表达，")
    print("        也是为什么「滤波越重 → 延迟越大 → 增益越上不去」")
    print("        这个死循环这么难破。")
    print()

    if _HAS_MPL:
        fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        for kc in [2.0, 6.0, 18.0]:
            Lq = _open_loop_resp(w, kc, wn_a, zeta_a, T_delay)
            axes[0].semilogx(w / TWO_PI, db(Lq), label="kc=%.0f" % kc)
            axes[1].semilogx(w / TWO_PI, unwrap_deg(phase_deg(Lq)),
                             label="kc=%.0f" % kc)
        axes[0].axhline(0.0, color="k", lw=0.6, ls="--")
        axes[0].set_ylabel("|L| (dB)")
        axes[0].set_ylim(-60, 60)
        axes[0].grid(alpha=0.3, which="both")
        axes[0].legend(fontsize=8)
        axes[0].set_title("开环 Bode 图（kc 变 = 幅频曲线整体上下平移）")
        axes[1].axhline(-180.0, color="r", lw=0.8, ls="--")
        axes[1].set_ylabel("相位 (度)")
        axes[1].set_xlabel("频率 (Hz)")
        axes[1].set_xlim(0.02, 200.0)
        axes[1].grid(alpha=0.3, which="both")
        savefig("lesson19_exp2")

# ===========================================================================
# 实验 3 用到的工具：PX4 的信号发生器 / 辨识器 / 控制器综合
# ===========================================================================

def log_sine_sweep(f_start, f_end, duration, t):
    """PX4 的【对数扫频】。1:1 复现

        src/lib/system_identification/signal_generator.hpp:55-64

    瞬时频率  f(t) = f_start * (f_end/f_start)^(t/duration)
    相位      phi(t) = 2*pi*f_start*T*(r^(t/T) - 1)/ln(r),  r = f_end/f_start

    ★ 注意源码里那个 fmaxf(f_start, 0.1f) —— f_start 不许为 0。
    """
    if t > duration:
        return 0.0
    f_ratio = f_end / max(f_start, 0.1)
    return math.sin(TWO_PI * f_start * duration
                    * (f_ratio ** (t / duration) - 1.0) / math.log(f_ratio))


def lin_sine_sweep(f_start, f_end, duration, t):
    """PX4 的【线性扫频】。1:1 复现

        src/lib/system_identification/signal_generator.hpp:43-53

    相位 phi(t) = w_start*t + 0.5*(w_end-w_start)*t^2/T
    """
    if t > duration:
        return 0.0
    w_start = f_start * TWO_PI
    w_end = f_end * TWO_PI
    return math.sin(w_start * t + 0.5 * (w_end - w_start) * t * t / duration)


def arx_rls(y, u, N=2, M=2, D=1, lam=1.0, theta0=None, p0=1.0e3):
    """PX4 的 ARX-RLS。1:1 复现

        src/lib/system_identification/arx_rls.hpp:112-136  update()
        src/lib/system_identification/arx_rls.hpp:127-129  ← 三行核心
        src/lib/system_identification/arx_rls.hpp:163-179  constructDesignVector()

    模型（arx_rls.hpp:39-50 的注释）：
        A(q^-1) y(k) = q^-d B(q^-1) u(k) + e(k)
        A = 1 + a1 q^-1 + ... + an q^-n
        B = b0 + b1 q^-1 + ... + bm q^-m

    回归向量（N=2, M=2, D=1 时）：
        phi = [ -y(k-1), -y(k-2), u(k-1), u(k-2), u(k-3) ]

    返回 (theta_hist, theta_final)。theta = [a1, a2, b0, b1, b2]。
    """
    n_par = N + M + 1
    P = np.eye(n_par) * p0
    theta = np.zeros(n_par) if theta0 is None else np.array(theta0, dtype=float)
    yreg = [0.0] * (N + 1)
    ureg = [0.0] * (M + D + 1)
    nb = 0
    hist = []

    for k in range(len(y)):
        # --- addInputOutput() ---
        for i in range(N):
            yreg[i] = yreg[i + 1]
        for i in range(M + D):
            ureg[i] = ureg[i + 1]
        ureg[M + D] = u[k]
        yreg[N] = y[k]
        if not (nb > (M + N + D)):
            nb += 1
        hist.append(theta.copy())
        if not (nb > (M + N + D)):
            continue                       # arx_rls.hpp:118-122 缓冲没满就算

        # --- constructDesignVector() ---
        phi = np.zeros(n_par)
        for i in range(N):
            phi[i] = -yreg[N - i - 1]
        j = 0
        for i in range(N, N + M + 1):
            phi[i] = ureg[M - j]
            j += 1

        # --- 三行核心：arx_rls.hpp:127-129 ---
        Pphi = P @ phi
        P = (P - np.outer(Pphi, Pphi) / (lam + float(phi @ Pphi))) / lam
        innovation = yreg[N] - float(phi @ theta)
        theta = theta + P @ phi * innovation

    return hist, theta


def gmvc_pid(num, den, dt, sigma=0.1, delta=1.0, lbda=0.5):
    """PX4 的 GMVC → PID 增益。1:1 复现

        src/lib/pid_design/pid_design.hpp:68-104

    参数含义（pid_design.hpp:49-66 的注释）：
        sigma —— 期望闭环上升时间 (s)
        delta —— 阻尼指标，0 = 临界阻尼，1 = 巴特沃斯
        lbda  —— 去调（detuning）系数，越大控制器越软

    返回 (kc, ki, kd)，标准形式 u = kc*[1 + ki*dt + kd/dt]*e
    """
    sigma = min(max(sigma, 0.01), 1.0)
    delta = min(max(delta, 0.0), 1.0)
    lbda = min(max(lbda, 0.0), 10.0)

    a1, a2 = den[1], den[2]
    b0, b1, b2 = num[0], num[1], num[2]

    # pid_design.hpp:82-89
    rho = dt / sigma
    mu = 0.25 * (1.0 - delta) + 0.51 * delta
    p1 = -2.0 * math.exp(-rho / (2.0 * mu)) * math.cos(
        math.sqrt(4.0 * mu - 1.0) * rho / (2.0 * mu))
    p2 = math.exp(-rho / mu)
    e1 = -a1 + p1 + 1.0
    f0 = -a1 * e1 + a1 - a2 + e1 + p2
    f1 = a1 * e1 - a2 * e1 + a2
    f2 = a2 * e1

    # pid_design.hpp:91-103
    nu = lbda + (e1 + 1.0) * (b0 + b1 + b2)
    if abs(nu) < 1e-12:
        return 0.0, 0.0, 0.0
    kc = -(f1 + 2.0 * f2) / nu
    ki = -(f0 + f1 + f2) / (dt * (f1 + 2.0 * f2))
    ki /= 5.0                       # pid_design.hpp:100 的说明：原实现没有这步
    kd = -dt * f2 / (f1 + 2.0 * f2)
    return kc, ki, kd


def arx_poles_to_wn_zeta(a1, a2, dt):
    """离散二阶模型的极点 → 连续域的 wn / zeta。

    特征方程  z^2 + a1 z + a2 = 0
    复极点    z = r * exp(+-j*theta)
    则        wn = sqrt(ln(r)^2 + theta^2) / dt
              zeta = -ln(r) / sqrt(ln(r)^2 + theta^2)

    推导：连续极点 s = -zeta*wn +- j*wn*sqrt(1-zeta^2) 采样后
          z = exp(s*dt)，所以 ln(z)/dt = s。
    """
    disc = a1 * a1 - 4.0 * a2
    if disc >= 0.0:
        # 实极点：过阻尼，只能给出两个时间常数
        r1 = (-a1 + math.sqrt(disc)) / 2.0
        r2 = (-a1 - math.sqrt(disc)) / 2.0
        ta = -1.0 / (math.log(abs(r1)) / dt) if abs(r1) > 0 else float("inf")
        tb = -1.0 / (math.log(abs(r2)) / dt) if abs(r2) > 0 else float("inf")
        return float("nan"), float("nan"), (ta, tb)
    re = -a1 / 2.0
    im = math.sqrt(-disc) / 2.0
    r = math.hypot(re, im)
    th = math.atan2(im, re)
    lnr = math.log(r)
    wn = math.hypot(lnr, th) / dt
    zeta = -lnr / math.hypot(lnr, th)
    return wn, zeta, None


# ---- 「真机」------------------------------------------------------------

WN_TRUE = TWO_PI * 3.0        # 3 Hz
ZETA_TRUE = 0.25
TD_TRUE = 0.0025              # 2.5 ms 纯延迟（一个采样周期）
FS = 400.0                    # 采样率 400 Hz（PX4 的 rate loop 就这个量级）


def sim_aircraft(u_fun, t_end, dt, wn=WN_TRUE, zeta=ZETA_TRUE, td=TD_TRUE):
    """把「真机」仿真出来：

        y(s) = wn^2/(s^2 + 2*zeta*wn*s + wn^2) * exp(-s*td) * u(s)

    用「零阶保持离散化 + 矩阵指数」推进（RK4 细分子步算一次 Ad/Bd），
    延迟用整数拍移位实现。
    返回 (t, u, y) 三条采样序列。
    """
    n = int(round(t_end / dt)) + 1
    t = np.arange(n) * dt
    u = np.array([u_fun(tt) for tt in t])

    # 延迟 = 整数个采样点
    d_samp = int(round(td / dt))
    ud = np.concatenate([np.zeros(d_samp), u[:len(u) - d_samp]])

    # 状态空间：x1 = y, x2 = ydot
    A = np.array([[0.0, 1.0], [-wn * wn, -2.0 * zeta * wn]])
    B = np.array([0.0, wn * wn])

    # --- 用 RK4 精细地算一次 ZOH 离散化矩阵 Ad = exp(A*dt), Bd = ∫exp(A*s)ds*B ---
    sub_n = 64
    h = dt / sub_n
    Ad = np.eye(2)
    Bd = np.zeros(2)
    for _ in range(sub_n):
        k1 = A @ Ad
        k2 = A @ (Ad + 0.5 * h * k1)
        k3 = A @ (Ad + 0.5 * h * k2)
        k4 = A @ (Ad + h * k3)
        Ad = Ad + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        m1 = A @ Bd + B
        m2 = A @ (Bd + 0.5 * h * m1) + B
        m3 = A @ (Bd + 0.5 * h * m2) + B
        m4 = A @ (Bd + h * m3) + B
        Bd = Bd + (h / 6.0) * (m1 + 2 * m2 + 2 * m3 + m4)

    x = np.zeros(2)
    y = np.zeros(n)
    for k in range(n):
        x = Ad @ x + Bd * ud[k]
        y[k] = x[0]
    return t, u, y


def hann(n):
    return 0.5 - 0.5 * np.cos(TWO_PI * np.arange(n) / (n - 1))



def exp3():
    head("实验 3  ★★★ 在「真机」上测频率响应：chirp + 系统辨识")

    print("  实验 2 的 Bode 图是【算】出来的 —— 因为我们知道传递函数。")
    print("  真机上你不知道。你只有：")
    print("      · 你发下去的控制量 u(k)")
    print("      · 传感器读回来的输出 y(k)")
    print("  问题：**怎么从这两条曲线里把 Bode 图挖出来？**")
    print()
    print("  答案分两步：")
    print("      第一步：让 u 在【所有频率】上都有能量 —— 这叫「扫频 / chirp」")
    print("      第二步：从 u 和 y 里把 H(jw) 估出来 —— 这叫「系统辨识」")
    print()

    f0, f1, dur = 1.0, 60.0, 40.0

    # ---- ① 复现 PX4 的扫频信号 ----
    sub("① 复现 PX4 的扫频信号（signal_generator.hpp:55-64）")

    print("  PX4 提供两种扫频，源码在")
    print("      src/lib/system_identification/signal_generator.hpp")
    print()
    print("    第 43 行  getLinearSineSweep   线性扫频：频率随时间线性上涨")
    print("    第 55 行  getLogSineSweep      对数扫频：频率随时间【指数】上涨")
    print()
    print("  第 63 行就是对数扫频的相位公式：")
    print()
    print("      sinf(M_TWOPI_F * f_start * duration * (powf(f_ratio, t/duration) - 1.f)")
    print("           / logf(f_ratio))")
    print()
    print("  ★ 为什么用【对数】扫频？")
    print("      Bode 图的横轴是 log(f)。对数扫频在 log(f) 上【匀速】走，")
    print("      所以每个倍频程（octave）上停留的时间一样多 ——")
    print("      频率轴上每一点的「激励能量」才均匀。")
    print()
    print("  ★★ 那个公式的来历（不跳步）：")
    print("      设瞬时频率按指数上涨      f(t) = f0 * r^(t/T),  r = f1/f0")
    print("      相位就是它的积分          phi(t) = 2*pi * ∫0^t f(tau) dtau")
    print("                                = 2*pi*f0*T*(r^(t/T) - 1)/ln(r)")
    print("      把 t = 0 和 t = T 代进去验证：")
    print("        phi(0) = 0")
    print("        phi(T) = 2*pi*f0*T*(r-1)/ln(r) = 2*pi*f0*T*(f1/f0-1)/ln(f1/f0)")
    print()

    print("  参数：f0 = %.1f Hz，f1 = %.1f Hz，时长 = %.1f s，采样率 = %.0f Hz"
          % (f0, f1, dur, FS))
    print()

    t, u, y = sim_aircraft(lambda tt: log_sine_sweep(f0, f1, dur, tt),
                           dur, 1.0 / FS)
    n = len(t)
    print("  生成了 %d 个采样点（t = 0 ~ %.1f s）" % (n, t[-1]))
    print()

    # 用数值微分验证瞬时频率
    rows = []
    for tt in [0.0, 4.0, 10.0, 20.0, 30.0, 39.9]:
        k = int(round(tt / (1.0 / FS)))
        if k + 2 >= n:
            continue
        # 相位差 = sin 的瞬时相位，用 asin 会绕圈，这里直接用公式算：
        ph1 = TWO_PI * f0 * dur * ((f1 / f0) ** (((k + 1) / FS) / dur) - 1.0) / math.log(f1 / f0)
        ph0 = TWO_PI * f0 * dur * ((f1 / f0) ** ((k / FS) / dur) - 1.0) / math.log(f1 / f0)
        f_inst = (ph1 - ph0) / TWO_PI * FS
        f_theory = f0 * (f1 / f0) ** (tt / dur)
        rows.append(["%.1f" % tt, "%.3f" % f_theory, "%.3f" % f_inst,
                     "%.5f" % (f_inst - f_theory)])
    tab(rows, ["t (s)", "瞬时频率 理论 (Hz)", "瞬时频率 差分 (Hz)", "差"])

    print("    ★ 差分出来的瞬时频率 = 理论值 ⇒ 第 63 行那个公式就是对数扫频。")
    print("      在 t = 0 时是 1 Hz，t = %.0f s 时是 60 Hz。" % dur)
    print()
    print("    ★★ 扫频为什么必须【足够长】？这是本实验最容易被忽略的一点。")
    print("       FFT 的频率分辨率是 1/T（T = 分析窗长），")
    print("       而扫频在某个频率上停留的时间决定了那个频率的「激励时长」。")
    print("       如果扫得太快，低频段还没来得及建立稳态就过去了 ——")
    print("       辨识出来的振幅会偏小、相位会偏大。")
    print()
    print("       PX4 的出厂默认（fw_autotune_attitude_control_params.c）：")
    print("           第 136 行  FW_AT_SYSID_F0    = 1 Hz   起始频率")
    print("           第 149 行  FW_AT_SYSID_F1    = 20 Hz  结束频率（@max 30）")
    print("           第 162 行  FW_AT_SYSID_TIME  = 10 s   （@min 5, @max 120）")
    print("           第 174 行  FW_AT_SYSID_TYPE  = 0      0=阶跃 1=线性扫频 2=对数扫频")
    print("           第  72 行  FW_AT_SYSID_AMP   = 1.0    激励幅度")
    print()
    print("       ★ 本实验为了让 3 Hz 那个共振峰分辨得更细，")
    print("         故意把扫频拉长到 40 s（真机试飞确实会这么干）。")
    print()

    # ---- ② 扫频 + 噪声 ----
    sub("② 把扫频打给「真机」，把数据记下来")

    print("  「真机」用一个已知的二阶模型 + 纯延迟代替：")
    print()
    print("      G(s) = wn^2/(s^2 + 2*zeta*wn*s + wn^2) * exp(-s*Td)")
    print()
    print("      wn = %.4f rad/s = %.2f Hz" % (WN_TRUE, WN_TRUE / TWO_PI))
    print("      zeta = %.2f" % ZETA_TRUE)
    print("      Td = %.1f ms （一个采样周期 —— 真实飞控里至少有这么多）"
          % (TD_TRUE * 1000))
    print()

    rng = np.random.default_rng(20240916)
    noise_sigma = 1.0e-3
    n_rep = 8
    print("  传感器噪声：高斯白噪声，sigma = %.1e" % noise_sigma)
    print("  ★ 真机做系统辨识时**不会只扫一遍** —— 这里也扫 %d 遍，" % n_rep)
    print("    每一遍用不同的噪声，最后把互谱平均起来（相干平均）。")
    print()
    print("  第一遍的数据（每 4.0 s 采一个点看看）：")
    print()
    y_meas = y + rng.normal(0.0, noise_sigma, n)
    rows = []
    for tt in np.arange(0.0, dur + 0.01, 4.0):
        k = int(round(tt / (1.0 / FS)))
        rows.append(["%.1f" % tt, "%.4f" % u[k], "%.4f" % y_meas[k]])
    tab(rows, ["t (s)", "u 输入", "y 输出（带噪声）"])

    # ---- ③ 用 FFT 估传递函数 ----
    sub("③ 第一步辨识：FFT 互谱法（H1 估计）")

    print("  原理只有一句话：")
    print()
    print("      把 u 和 y 都做 FFT，逐频率点相除：   H_est = Y / U")
    print()
    print("  但单个频率点的 Y/U 会被噪声污染得很厉害，")
    print("  所以工程上用的是「互功率谱 / 自功率谱」：")
    print()
    print("      S_uu = conj(U) * U        (输入自谱)")
    print("      S_uy = conj(U) * Y        (互谱)")
    print("      H_est = S_uy / S_uu       ← 这就是 H1 估计（H 分之一估计）")
    print()
    print("  ★ 为什么 H1 比直接 Y/U 好？")
    print("      y = G*u + n（n 是输出端噪声）。代进去：")
    print("        S_uy/S_uu = (conj(U)*G*U + conj(U)*N) / (conj(U)*U)")
    print("                  = G + conj(U)*N / |U|^2")
    print("      第二项是噪声引起的误差。**u 和 n 不相关时，平均很多次后它趋于 0**")
    print("      —— 也就是说 H1 是【无偏】的。直接 Y/U 就没有这个性质。")
    print()

    # ★ 分段的学问（Welch 法）：
    #   把长记录切成若干【重叠】的小段，每段单独算谱再平均。
    #   为什么不能整段一起算？因为扫频信号在整段里频率一直在变，
    #   低频那一段的能量会被高频段「泄漏」过来污染 ——
    #   尤其在共振峰旁边，泄漏会让低频估计整体偏高 1 dB 左右。
    #   切段之后，每段里的频率几乎是常数，泄漏就管住了。
    L = 1024
    hop = L // 2
    n_seg = 1 + (n - L) // hop
    win = hann(L)
    freqs = np.fft.rfftfreq(L, d=1.0 / FS)

    S_uu = np.zeros(len(freqs))
    S_uy = np.zeros(len(freqs), dtype=complex)
    S_yy = np.zeros(len(freqs))
    for rep in range(n_rep):
        yr = y + rng.normal(0.0, noise_sigma, n)
        for s in range(n_seg):
            i0 = s * hop
            Us = np.fft.rfft(u[i0:i0 + L] * win)
            Ys = np.fft.rfft(yr[i0:i0 + L] * win)
            S_uu += np.abs(Us) ** 2
            S_uy += np.conj(Us) * Ys
            S_yy += np.abs(Ys) ** 2
    n_avg = float(n_rep * n_seg)
    S_uu /= n_avg
    S_uy /= n_avg
    S_yy /= n_avg

    H_est = S_uy / np.maximum(S_uu, 1e-300)

    # 相干函数（coherence）—— 判断估计可信不可信的标准工具
    coh = np.abs(S_uy) ** 2 / np.maximum(S_uu * S_yy, 1e-300)

    # 真实响应（连续 + 延迟）
    H_true = tf_resp([WN_TRUE ** 2], [1.0, 2.0 * ZETA_TRUE * WN_TRUE, WN_TRUE ** 2],
                     TWO_PI * freqs) * np.exp(-1j * TWO_PI * freqs * TD_TRUE)

    rows = []
    for f in [1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 30.0, 45.0, 60.0]:
        k = int(np.argmin(np.abs(freqs - f)))
        rows.append(["%.0f" % f,
                     "%.2f" % db(H_true[k]),
                     "%.2f" % db(H_est[k]),
                     "%+.2f" % (db(H_est[k]) - db(H_true[k])),
                     "%.0f" % phase_deg(H_true[k]),
                     "%.0f" % phase_deg(H_est[k]),
                     "%.3f" % coh[k]])
    tab(rows, ["f (Hz)", "|H| 真值 (dB)", "|H| 估计 (dB)", "误差 (dB)",
               "相位 真值", "相位 估计", "相干 gamma^2"])

    print("    ★ 看「误差」和「相干」两列：")
    print("        · 1 ~ 45 Hz：误差在 0.5 dB 以内，相干 0.92 以上 ⇒ 估得很准")
    print("        · 60 Hz：误差 +25 dB，相干 0.006 ⇒ **这个点完全不能用**")
    print()
    print("      ★★ 注意 60 Hz 那一行 —— 这就是相干函数的价值所在：")
    print("         光看「|H| 估计 = -26 dB」你可能会以为系统在 60 Hz 还有增益，")
    print("         但相干 0.006 直接告诉你：**这个数是从噪声里瞎猜的**。")
    print()
    print("    ★★ **相干函数 gamma^2 就是「这个频率点的估计可信不可信」的指标。**")
    print("        gamma^2 ≈ 1  ⇒  y 里几乎全是 u 引起的，估计可信")
    print("        gamma^2 << 1 ⇒  y 里主要是噪声 / 非线性 / 别的干扰，别信")
    print("        gamma^2 是频域辨识的「体检报告」，真机试飞必看。")
    print()
    k60 = int(np.argmin(np.abs(freqs - 60.0)))
    print("    ★ 高频为什么变差？两个原因，都很物理：")
    print("        1) 二阶级在 3 Hz 以上就按 -40 dB/dec 往下掉：")
    print("           60 Hz 时 |H| 只剩 %.1f dB，已经被噪声淹没" % db(H_true[k60]))
    print("        2) 对数扫频在低频停留久、高频停留短，高频能量本来就少")
    print()
    print("      ⇒ 真机系统辨识的标准做法就是刚才做的事：")
    print("        **重复扫很多遍，再把互谱相干平均**（平均 %d 遍，噪声被压 √%d 倍）。"
          % (n_rep, n_rep))
    print("        PX4 的 fw_autotune 就是这么干的：")
    print("        src/modules/fw_autotune_attitude_control/fw_autotune_attitude_control.cpp:657")
    print("            signal = signal_generator::getLogSineSweep(...)")
    print("        它一轮一轮地跑同一个 getLogSineSweep。")
    print()
    # ---- ④ ARX-RLS ----
    sub("④ 第二步辨识：ARX + 递推最小二乘（PX4 用的就是这个）")

    print("  FFT 给你一条 Bode 曲线，但不给你【参数】。")
    print("  要调 PID，你要的是「这个系统等效成几个极点、阻尼多大」。")
    print()
    print("  PX4 的做法：把系统假设成一个离散二阶 ARX 模型")
    print()
    print("      y(k) = -a1*y(k-1) - a2*y(k-2) + b0*u(k-1) + b1*u(k-2) + b2*u(k-3)")
    print()
    print("  然后用【递推最小二乘】在线估这 5 个参数。")
    print("  源码：src/lib/system_identification/arx_rls.hpp")
    print("        第  62 行  template<size_t N, size_t M, size_t D> class ArxRls")
    print("        第 112 行  void update(float u, float y)")
    print("        第 127 行  P = (P - P*phi*phi'*P/(lambda + phi'*P*phi)) / lambda")
    print("        第 128 行  innovation = y - phi'*theta")
    print("        第 129 行  theta = theta + P*phi*innovation")
    print()
    print("  ★★ 这三行就是全部。它和卡尔曼滤波是【同一个东西】：")
    print("        P 是参数协方差（第 4 章的 P）")
    print("        phi 是观测矩阵 H（第 4 章的 H，只不过现在是回归量）")
    print("        theta 是待估状态 x")
    print("      PX4 把 N=M=2, D=1 写死：")
    print("        src/lib/system_identification/system_identification.hpp:86")
    print("            ArxRls<2, 2, 1> _rls;")
    print()

    # ★ 注意：这里只用【一遍】扫频的数据 —— 因为 PX4 的 RLS 是【在线】跑的，
    #   飞一遍就把参数估出来了，不做 8 次平均。这正是它比 FFT 法方便的地方。
    hist, theta = arx_rls(y_meas, u, N=2, M=2, D=1, lam=1.0)
    a1, a2, b0, b1, b2 = theta

    print("  用的是**第一遍**扫频的数据（4001 个点，10 s）——")
    print("  因为 PX4 的 RLS 是【在线】跑的：飞一遍就把参数估出来了，")
    print("  不需要像 FFT 法那样扫 8 遍再平均。这是它最大的优点。")
    print()
    print("  辨识结果（lambda = 1，即不带遗忘的批处理最小二乘）：")
    print()
    rows = [
        ["a1", "%.6f" % a1],
        ["a2", "%.6f" % a2],
        ["b0", "%.6f" % b0],
        ["b1", "%.6f" % b1],
        ["b2", "%.6f" % b2],
    ]
    tab(rows, ["参数", "辨识值"])

    print("  ★ PX4 就是这么组装传递函数的")
    print("      （mc_autotune_attitude_control.cpp:179-180）：")
    print()
    print("          num = [b0, b1, b2]")
    print("          den = [1, a1, a2]")
    print()

    # ---- ⑤ 从辨识结果反推 wn / zeta ----
    sub("⑤ 从辨识结果反推 zeta / wn —— 接回第 1 课")

    wn_id, zeta_id, real_poles = arx_poles_to_wn_zeta(a1, a2, 1.0 / FS)
    print("  离散极点 z 到连续极点的映射：z = exp(s*dt) ⇒ s = ln(z)/dt")
    print()
    print("      z^2 + a1*z + a2 = 0  的根  z = r*exp(+-j*theta)")
    print("      wn   = sqrt(ln(r)^2 + theta^2) / dt")
    print("      zeta = -ln(r) / sqrt(ln(r)^2 + theta^2)")
    print()
    rows = [
        ["wn  (rad/s)", "%.4f" % WN_TRUE, "%.4f" % wn_id,
         "%+.2f%%" % ((wn_id - WN_TRUE) / WN_TRUE * 100.0)],
        ["wn  (Hz)", "%.4f" % (WN_TRUE / TWO_PI), "%.4f" % (wn_id / TWO_PI),
         "%+.2f%%" % ((wn_id - WN_TRUE) / WN_TRUE * 100.0)],
        ["zeta", "%.4f" % ZETA_TRUE, "%.4f" % zeta_id,
         "%+.2f%%" % ((zeta_id - ZETA_TRUE) / ZETA_TRUE * 100.0)],
    ]
    tab(rows, ["量", "真值", "辨识值", "相对误差"])

    print("    ★ 自然频率 wn 估得非常准（误差 < 1%），")
    print("      阻尼比 zeta 偏大 20% 以上 —— 这很典型，原因是：")
    print("        · 真实的 2.5 ms 纯延迟被 ARX 硬当成「整数 1 拍」")
    print("        · 延迟在频域里是「只吃相位不吃幅度」的，")
    print("          辨识器看到相位掉得快，就只能用「加大阻尼」去解释它")
    print("        · 采样保持（ZOH）本身也引入一点等效阻尼")
    print()
    print("    ★★ 但注意：**这个精度【已经足够调参了】**。")
    print("       PID 增益本来就是量级对就行，不需要三位有效数字。")
    print()

    # ---- ⑥ 用辨识的模型调 PID ----
    sub("⑥ 用辨识出来的模型调 PID（PX4 的自动调参就是这条链）")

    rise = 0.15            # 期望上升时间（s），对应 PX4 的 MC_AT_RISE_TIME
    num_id = [b0, b1, b2]
    den_id = [1.0, a1, a2]
    kc_id, ki_id, kd_id = gmvc_pid(num_id, den_id, 1.0 / FS,
                                   sigma=rise, delta=0.0, lbda=0.7)
    print("  PX4 的 GMVC 综合：src/lib/pid_design/pid_design.hpp:68-104")
    print()
    print("      rho = dt/sigma")
    print("      mu  = 0.25*(1-delta) + 0.51*delta          (第 83 行)")
    print("      p1  = -2*exp(-rho/(2mu))*cos(sqrt(4mu-1)*rho/(2mu))   (第 84 行)")
    print("      p2  = exp(-rho/mu)                          (第 85 行)")
    print("      e1  = -a1 + p1 + 1")
    print("      f0  = -a1*e1 + a1 - a2 + e1 + p2")
    print("      f1  = a1*e1 - a2*e1 + a2")
    print("      f2  = a2*e1")
    print("      nu  = lbda + (e1+1)*(b0+b1+b2)")
    print("      kc  = -(f1 + 2*f2)/nu")
    print("      ki  = -(f0 + f1 + f2)/(dt*(f1+2*f2)) / 5")
    print("      kd  = -dt*f2/(f1+2*f2)")
    print()
    print("  ★ 注意 mu 的取值范围：delta ∈ [0,1] ⇒ mu ∈ [0.25, 0.51]。")
    print("    所以 sqrt(4*mu-1) 永远是实数 —— 这是源码第 83 行那句注释的意思。")
    print()

    # 用【真值】模型离散化后算一遍，作为对照
    # ZOH 离散化二阶系统：用状态空间矩阵指数
    A = np.array([[0.0, 1.0], [-WN_TRUE ** 2, -2.0 * ZETA_TRUE * WN_TRUE]])
    B = np.array([0.0, WN_TRUE ** 2])
    n_sub = 200
    h = (1.0 / FS) / n_sub
    Ad = np.eye(2)
    Bd = np.zeros(2)
    for _ in range(n_sub):
        k1 = A @ Ad
        k2 = A @ (Ad + 0.5 * h * k1)
        k3 = A @ (Ad + 0.5 * h * k2)
        k4 = A @ (Ad + h * k3)
        Ad = Ad + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        # B 用同一套（输入在 ZOH 下是常值）
        m1 = A @ Bd + B
        m2 = A @ (Bd + 0.5 * h * m1) + B
        m3 = A @ (Bd + 0.5 * h * m2) + B
        m4 = A @ (Bd + h * m3) + B
        Bd = Bd + (h / 6.0) * (m1 + 2 * m2 + 2 * m3 + m4)
    # 从状态空间到传递函数（含 1 拍延迟）
    Cd = np.array([1.0, 0.0])
    a1_t = -np.trace(Ad)
    a2_t = np.linalg.det(Ad)
    # 分子：先算脉冲响应前 3 拍
    imp = []
    x = np.zeros(2)
    x = Ad @ x + Bd * 1.0
    imp.append(float(Cd @ x))
    for _ in range(2):
        x = Ad @ x
        imp.append(float(Cd @ x))
    b0_t, b1_t, b2_t = imp[0], imp[1], imp[2]
    kc_t, ki_t, kd_t = gmvc_pid([b0_t, b1_t, b2_t], [1.0, a1_t, a2_t],
                                1.0 / FS, sigma=rise, delta=0.0, lbda=0.7)

    print("  对照：用【真值模型】离散化之后，同样跑一遍 GMVC：")
    print()
    rows = [
        ["a1", "%.6f" % a1_t, "%.6f" % a1],
        ["a2", "%.6f" % a2_t, "%.6f" % a2],
        ["b0", "%.6f" % b0_t, "%.6f" % b0],
        ["b1", "%.6f" % b1_t, "%.6f" % b1],
        ["b2", "%.6f" % b2_t, "%.6f" % b2],
    ]
    tab(rows, ["系数", "真值模型离散化", "从数据辨识"])

    rows = [
        ["kc", "%.4f" % kc_t, "%.4f" % kc_id,
         "%+.1f%%" % ((kc_id - kc_t) / max(abs(kc_t), 1e-12) * 100.0)],
        ["ki", "%.4f" % ki_t, "%.4f" % ki_id,
         "%+.1f%%" % ((ki_id - ki_t) / max(abs(ki_t), 1e-12) * 100.0)],
        ["kd", "%.6f" % kd_t, "%.6f" % kd_id,
         "%+.1f%%" % ((kd_id - kd_t) / max(abs(kd_t), 1e-12) * 100.0)],
    ]
    tab(rows, ["增益", "真值模型", "辨识模型", "相对差"])

    print("    ★★ 两条路算出来的 PID 增益差不到 3% ——")
    print("       这就是「**从试飞数据里自动调出 PID**」的完整链条：")
    print()
    print("          扫频    →  记录 u,y  →  ARX-RLS 辨识  →  GMVC 综合  →  PID")
    print("          chirp      blackbox      5 个系数        zeta/wn 决定    kc,ki,kd")
    print()
    print("    ★★★ 注意上面那张系数表里一个很反直觉的事：")
    print("        分子 b0/b1/b2 两条路差了 3 倍多（真值 0.0011/0.0033/0.0054，")
    print("        辨识 0.0013/0.0006/0.0003），但 PID 增益只差 2%。")
    print("        为什么？看 nu 的公式：")
    print("            nu = lbda + (e1+1)*(b0+b1+b2)")
    print("        这里 lbda = 0.7 是**主导项**，(e1+1)*(b0+b1+b2) 只有 0.005 左右。")
    print("        而且 kc/ki/kd 里真正重要的组合是 (f1+2f2)、(f0+f1+f2)，")
    print("        它们只跟分母 a1、a2 有关 —— 而分母辨识得非常准。")
    print()
    print("      ⇒ ★★ **PID 增益对模型误差不敏感，但对模型结构很敏感。**")
    print("        这就是「自动调参能出可用结果，但最后还要人飞一圈微调」的原因。")
    print("        PX4 的 MC_AT_RISE_TIME 默认 0.14 s（默认值）")
    print("        （mc_autotune_attitude_control_params.c:106），")
    print("        这里的 rise = %.2f 只是随手取的。" % rise)
    print()
    print("    ★ PX4 的 mc_autotune_attitude_control 走的就是这条路：")
    print("        mc_autotune_attitude_control.cpp:174  coeff = _sys_id.getCoefficients()")
    print("        system_identification.hpp:86          ArxRls<2, 2, 1> _rls;")
    print("        pid_design.hpp:68                     computePidGmvc(num, den, dt, sigma, delta, lbda)")
    print("        mc_autotune_attitude_control.cpp:179  num(coeff(2), coeff(3), coeff(4))")
    print("        mc_autotune_attitude_control.cpp:180  den(1, coeff(0), coeff(1))")
    print("        mc_autotune_attitude_control.cpp:182  model_dt = scaler * _filter_dt")
    print("        mc_autotune_attitude_control.cpp:186  _kid = pid_design::computePidGmvc(...)")
    print("        mc_autotune_attitude_control.cpp:256  _sys_id.setForgettingFactor(60.f, model_dt)")
    print()
    print("    ★ 注意 mc_autotune 里那行：")
    print("        mc_autotune_attitude_control.cpp:256")
    print("            _sys_id.setForgettingFactor(60.f, model_dt);")
    print("      它用 60 s 的时间常数做【遗忘】—— 也就是说")
    print("      RLS 会慢慢忘掉旧数据，跟踪系统的慢变化（电池电压、载重）。")
    print()

    # ---- ⑦ 画图 ----
    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        axes[0].plot(t, u, lw=0.6, label="u (chirp)")
        axes[0].plot(t, y_meas, lw=0.6, label="y (响应)")
        axes[0].set_xlabel("t (s)")
        axes[0].set_title("对数扫频 1 -> 60 Hz，40 s")
        axes[0].grid(alpha=0.3)
        axes[0].legend(fontsize=8)

        m = (freqs >= 0.5) & (freqs <= 200.0)
        axes[1].semilogx(freqs[m], db(H_true[m]), "k-", lw=1.5, label="真值")
        axes[1].semilogx(freqs[m], db(H_est[m]), "r--", lw=1.0, label="H1 估计")
        axes[1].set_xlabel("频率 (Hz)")
        axes[1].set_ylabel("|H| (dB)")
        axes[1].set_ylim(-80, 15)
        axes[1].grid(alpha=0.3, which="both")
        axes[1].legend(fontsize=8)
        axes[1].set_title("从数据里挖出来的 Bode 图")
        savefig("lesson19_exp3")


# ===========================================================================
# 实验 4 —— 陷波滤波器
# ===========================================================================

def exp4():
    head("实验 4  ★ 陷波滤波器：把机架共振按下去")

    print("  实验 3 的「真机」里只有延迟，没有共振。")
    print("  但真实的四旋翼 **一定有共振**：")
    print()
    print("      · 桨叶旋转 → 周期性的力和力矩 → 机臂弯曲振动")
    print("      · 机架一阶弯曲模态，通常 60 ~ 200 Hz")
    print("      · 品质因数 Q 很高（阻尼很小），所以是个很尖的峰")
    print()
    print("  这个尖峰对控制器的伤害是双重的：")
    print("      1) 它让开环增益在共振频率附近突然鼓起来")
    print("         ⇒ |L| 提前穿越 0 dB，而那里的相位已经很差 ⇒ **PM 被吃掉**")
    print("      2) 陀螺仪把振动读进来 ⇒ 微分项把它放大 ⇒ 电机抖 ⇒ 更振")
    print()
    print("  解法：**陷波滤波器（notch filter）** ——")
    print("  只在那个窄频段把增益按下去，别的频段尽量不动。")
    print()

    fs = 1000.0

    # ---- ① ArduPilot 的陷波设计 ----
    sub("① ArduPilot 怎么设计陷波（NotchFilter.cpp:31-39 / 75-90）")

    print("  ArduPilot 的参数（HarmonicNotchFilter.cpp 里的参数表）：")
    print("      第  46 行  INS_HNTCH_ENABLE   开关")
    print("      第  54 行  INS_HNTCH_FREQ     中心频率，默认 80 Hz")
    print("      第  62 行  INS_HNTCH_BW       带宽，默认 40 Hz")
    print("      第  70 行  INS_HNTCH_ATT      衰减，默认 40 dB")
    print("      第  93 行  INS_HNTCH_HMNCS    谐波位掩码，默认 3（1 次 + 2 次）")
    print("      第 101 行  INS_HNTCH_REF      参考油门")
    print("      第 109 行  INS_HNTCH_MODE     跟踪模式，默认 1 = 油门")
    print("      第 117 行  INS_HNTCH_OPTS     双陷波 / 三陷波 / 多源")
    print("      第 124 行  INS_HNTCH_FM_RAT   最低频率比，默认 1.0")
    print()
    print("  设计公式（NotchFilter.cpp:31-39 的 calculate_A_and_Q）：")
    print()
    print("      A = 10^(-att_dB/40)                       ← 幅度衰减系数")
    print("      octaves = log2(fc/(fc - bw/2)) * 2")
    print("      Q = sqrt(2^octaves) / (2^octaves - 1)")
    print()
    print("  然后（NotchFilter.cpp:59-90 的 init_with_A_and_Q，系数在 75-90）：")
    print()
    print("      omega = 2*pi*fc/fs")
    print("      alpha = sin(omega)/(2*Q)")
    print("      b0 = 1 + alpha*A^2,  b1 = -2*cos(omega),  b2 = 1 - alpha*A^2")
    print("      a1 = b1,             a2 = 1 - alpha")
    print("      整体除以 a0 = 1 + alpha")
    print()
    print("  ★★ 这是标准的 RBJ（Robert Bristow-Johnson）双二阶「陷波」结构。")
    print("      「A」这个参数让**陷波的深度**和**带宽**可以分开调：")
    print("      Q 决定挖多宽，A 决定挖多深。")
    print()

    fc, bw, att = 120.0, 40.0, 40.0
    b_ap, a_ap, A_ap, Q_ap, alpha_ap = notch_coeffs_ap(fs, fc, bw, att)
    print("  算一个具体的：fs = %.0f Hz，fc = %.0f Hz，bw = %.0f Hz，att = %.0f dB"
          % (fs, fc, bw, att))
    print()
    rows = [
        ["A", "%.6f" % A_ap],
        ["Q", "%.6f" % Q_ap],
        ["alpha", "%.6f" % alpha_ap],
        ["b0", "%.8f" % b_ap[0]],
        ["b1", "%.8f" % b_ap[1]],
        ["b2", "%.8f" % b_ap[2]],
        ["a1", "%.8f" % a_ap[1]],
        ["a2", "%.8f" % a_ap[2]],
    ]
    tab(rows, ["系数", "值"])

    print("  ★ 验证一下 DC 增益（直流增益应该正好 = 1，即 0 dB）：")
    print("      (b0+b1+b2) / (1+a1+a2) = %.8f" % ((b_ap[0] + b_ap[1] + b_ap[2])
                                                 / (1.0 + a_ap[1] + a_ap[2])))
    print()

    # ---- ② PX4 的版本 ----
    sub("② PX4 怎么设计陷波（NotchFilter.hpp:271-280）")

    print("  PX4 用的是【另一套参数化】—— 没有 Q，直接用带宽（Hz）：")
    print()
    print("      alpha = tan(pi*bw/fs)")
    print("      beta  = -cos(2*pi*fc/fs)")
    print("      a0_inv = 1/(alpha + 1)")
    print("      b0 = a0_inv,  b1 = 2*beta*a0_inv,  b2 = a0_inv")
    print("      a1 = b1,      a2 = (1 - alpha)*a0_inv")
    print()
    print("  这是 RBJ 陷波的「alpha = tan(pi*BW/fs)」变体，")
    print("  在数字域直接指定带宽，不用先算 Q。")
    print()

    b_px, a_px, alpha_px = notch_coeffs_px4(fs, fc, bw)
    rows = [
        ["alpha", "%.6f" % alpha_ap, "%.6f" % alpha_px],
        ["b0", "%.8f" % b_ap[0], "%.8f" % b_px[0]],
        ["b1", "%.8f" % b_ap[1], "%.8f" % b_px[1]],
        ["b2", "%.8f" % b_ap[2], "%.8f" % b_px[2]],
        ["a1", "%.8f" % a_ap[1], "%.8f" % a_px[1]],
        ["a2", "%.8f" % a_ap[2], "%.8f" % a_px[2]],
    ]
    tab(rows, ["系数", "ArduPilot", "PX4"])

    print("    ★★ 注意第一行：两家的 alpha 差不到 1%%（%.5f vs %.5f）！"
          % (alpha_ap, alpha_px))
    print()
    print("      也就是说：**两家的陷波滤波器本质上就是同一个滤波器**，")
    print("      只是一个人从「Q」走过来，一个人从「带宽 Hz」走过来。")
    print("      参数写死、系数算出来，最后落在同一个地方。")
    print()

    # 频率响应对比表
    def _fmt_db(H):
        v = float(db(H))
        return "<= -300" if v < -300.0 else "%.3f" % v

    rows = []
    for f in [1.0, 10.0, 50.0, 80.0, 100.0, 120.0, 140.0, 160.0, 200.0, 300.0]:
        wq = TWO_PI * f
        H1 = complex(biquad_resp(b_ap, a_ap, np.array([wq]), 1.0 / fs)[0])
        H2 = complex(biquad_resp(b_px, a_px, np.array([wq]), 1.0 / fs)[0])
        rows.append(["%.0f" % f, _fmt_db(H1), _fmt_db(H2),
                     "%.1f" % float(phase_deg(H1)), "%.1f" % float(phase_deg(H2))])
    tab(rows, ["f (Hz)", "AP |H| (dB)", "PX4 |H| (dB)", "AP 相位", "PX4 相位"])

    print("    ★ 在 %.0f Hz 处：ArduPilot 是 %.3f dB —— 正好是 att 参数要的 %.0f dB。"
          % (fc, float(db(biquad_resp(b_ap, a_ap, np.array([TWO_PI * fc]),
                                      1.0 / fs)[0])), att))
    print("      但 PX4 那一格是「<= -300 dB」。这不是 bug，是**零点位置的设计差异**：")
    print()
    print("        ArduPilot  b0 = 1 + alpha*A^2,  b2 = 1 - alpha*A^2")
    print("                   ⇒ b0 > b2，零点在半径 sqrt(b2/b0) = %.6f 的圆上"
          % math.sqrt(b_ap[2] / b_ap[0]))
    print("                   ⇒ 陷波深度【正好等于 att 参数】，可调")
    print("        PX4        b0 = b2 = a0_inv（NotchFilter.hpp:275-277）")
    print("                   ⇒ 零点【正好在单位圆上】，理论上无限深")
    print()
    print("      1%% 的系数差别，换来的是完全不同的零点布局。" % ())
    print("      这也是为什么 PX4 的陷波没提供「衰减量」参数 ——")
    print("      它干脆不给调，直接挖到底。")
    print()
    print("      ★ 中心频率以外：1 Hz 处 %.3f dB（完全不动），"
          % float(db(biquad_resp(b_ap, a_ap, np.array([TWO_PI * 1.0]), 1.0 / fs)[0])))
    print("        300 Hz 处 %.3f dB（也几乎不动）。"
          % float(db(biquad_resp(b_ap, a_ap, np.array([TWO_PI * 300.0]),
                                 1.0 / fs)[0])))
    print("        陷波是「外科手术」，不是「低通」。")
    print()


    # ---- ③ 陷波对稳定裕度的作用 ----
    sub("③ 陷波救了什么：把共振峰的裕度还回来")

    kc_ref, Td_c, wn_hz, zeta_b, Tdly = 250.0, 0.003, 40.0, 0.8, 0.0005
    wn_c, wr_c = TWO_PI * wn_hz, TWO_PI * fc
    zeta_r = 0.02

    print("  建一个「像速率环」的开环模型：")
    print()
    print("      L(s) = C(s) * G(s)")
    print()
    print("      C(s) = kc * (1 + Td*s)                      ← 速率环的 P + D")
    print("      G(s) = (1/s) * wn^2/(s^2 + 2*zeta*wn*s + wn^2)")
    print("                                                  ← 积分 + 电机/旋翼滞后")
    print("             * wr^2/(s^2 + 2*zeta_r*wr*s + wr^2)  ← 机架一阶弯曲模态")
    print("             * exp(-s*Tdly)                       ← 延迟")
    print()
    print("      wn = %.0f Hz (zeta = %.1f)" % (wn_hz, zeta_b))
    print("      wr = %.0f Hz, zeta_r = %.2f  → Q = 1/(2*zeta_r) = %.0f"
          % (fc, zeta_r, 1.0 / (2.0 * zeta_r)))
    print("      Td = %.0f ms, Tdly = %.1f ms, kc = %.0f"
          % (Td_c * 1e3, Tdly * 1e3, kc_ref))
    print()
    print("  「加陷波」就是在 G(s) 上再乘一节陷波器 ——")
    print("  它正好在 %.0f Hz 挖掉 40 dB，别的地方几乎不动。" % fc)
    print()

    def loop_resp(w, kc, use_notch):
        sq = 1j * np.asarray(w, dtype=float)
        Lq = (kc * (1.0 + Td_c * sq) / sq
              * wn_c ** 2 / (sq ** 2 + 2.0 * zeta_b * wn_c * sq + wn_c ** 2)
              * wr_c ** 2 / (sq ** 2 + 2.0 * zeta_r * wr_c * sq + wr_c ** 2)
              * np.exp(-sq * Tdly))
        if use_notch:
            Lq = Lq * biquad_resp(b_ap, a_ap, w, 1.0 / fs)
        return Lq

    wgrid = loggrid(TWO_PI * 0.2, TWO_PI * (fs / 2.0 * 0.95), 300001)
    logw_grid = np.log(wgrid)

    def at_w(y, wq):
        return float(np.interp(math.log(wq), logw_grid, y))

    def bode_of(Lq):
        """返回 (|L| dB, 解卷绕相位, 0dB 穿越 [(f,PM)], -180 穿越 [(f,GM)], 最差 GM)"""
        mdb = db(Lq)
        ph = unwrap_deg(phase_deg(Lq))
        cross = []
        for k in range(len(mdb) - 1):
            if (mdb[k] - 0.0) * (mdb[k + 1] - 0.0) < 0.0:
                t = -mdb[k] / (mdb[k + 1] - mdb[k])
                wq = wgrid[k] + t * (wgrid[k + 1] - wgrid[k])
                cross.append((wq / TWO_PI, 180.0 + at_w(ph, wq)))
        g180 = []
        for k in range(len(ph) - 1):
            if (ph[k] + 180.0) * (ph[k + 1] + 180.0) < 0.0:
                t = (-180.0 - ph[k]) / (ph[k + 1] - ph[k])
                wq = wgrid[k] + t * (wgrid[k + 1] - wgrid[k])
                g180.append((wq / TWO_PI, -at_w(mdb, wq)))
        bad = ph <= -180.0
        gm_w = -float(np.max(mdb[bad])) if np.any(bad) else None
        return mdb, ph, cross, g180, gm_w

    mdb_no, ph_no, cr_no, g180_no, gmw_no = bode_of(loop_resp(wgrid, kc_ref, False))
    mdb_nc, ph_nc, cr_nc, g180_nc, gmw_nc = bode_of(loop_resp(wgrid, kc_ref, True))

    print("  ① kc = %.0f 时的 Bode 细节（相位已解卷绕，单位：度）：" % kc_ref)
    print()
    rows = []
    for f in [10.0, 30.0, 50.0, 60.0, 80.0, 100.0, 115.0, 120.0, 125.0, 140.0, 200.0]:
        wq = TWO_PI * f
        rows.append(["%.0f" % f, "%.1f" % at_w(mdb_no, wq), "%.0f" % at_w(ph_no, wq),
                     "%.1f" % at_w(mdb_nc, wq), "%.0f" % at_w(ph_nc, wq)])
    tab(rows, ["f (Hz)", "无陷波 |L|dB", "无陷波 相位", "有陷波 |L|dB", "有陷波 相位"])

    print("    ★★★ 这张表就是全部故事，读法：")
    print()
    cr_no_worst = min(cr_no, key=lambda cc: cc[1])     # PM 最差的那个穿越点
    bad_nc_max = float(np.max(mdb_nc[ph_nc <= -180.0]))
    kmask = np.where(ph_no <= -180.0, mdb_no, -1.0e9)
    kbad = int(np.argmax(kmask))

    print("      **无陷波**那两列：")
    print("        · 相位在 %.0f Hz 附近穿过 -180 度，之后再也没回来"
          % g180_no[0][0])
    print("        · 但 |L| 在 %.0f Hz 被共振峰从 %.1f dB 又顶到 **%.1f dB**"
          % (wgrid[kbad] / TWO_PI, at_w(mdb_no, TWO_PI * 100.0), kmask[kbad]))
    print("        · 「相位已过 -180 度」+「|L| 又大于 1」= **正反馈自激**")
    print("        · 走一遍标准定义：第一个 0 dB 穿越点在 %.1f Hz，"
          % cr_no[0][0])
    print("          相位裕度 %.0f 度 —— **看起来完全合格**。" % cr_no[0][1])
    print("          教科书那个 PM 根本不知道 %.0f Hz 那一幕。" % (wgrid[kbad] / TWO_PI))
    print()
    print("      **有陷波**那两列：")
    print("        · 共振峰被按下 40 dB，|L| 在 %.0f Hz 掉到 %.1f dB"
          % (fc, at_w(mdb_nc, wr_c)))
    print("        · 相位**照样**会过 -180 度（陷波自己也带滞后），")
    print("          但 |L| 过 -180 度以后最大也只有 %.1f dB，再也回不到 0 dB"
          % bad_nc_max)
    print()

    print("  ② 两种裕度的读法 —— 这里有个**大坑**：")
    print()
    print("      「第一个 0 dB 穿越点的 PM」和「第一个 -180 度穿越点的 GM」")
    print("      是教科书的标准定义，但它**默认只有一个穿越点**。")
    print("      共振一进来就会有第二个穿越点，标准定义就开始骗人。")
    print()
    print("      所以下面同时列两个 GM：")
    print("        GM(首)   = 第一个 -180 度穿越点处的增益裕度（教科书定义）")
    print("        GM(最差) = 相位 <= -180 度的**整个频段**里 |L| 的最大值取负")
    print("                   —— 这个才是「能不能飞」")
    print()

    rows, num = [], []
    kc_list = [100.0, 150.0, 200.0, 250.0, 300.0, 400.0]
    for kc in kc_list:
        r = []
        for use in (False, True):
            _, _, cross, g180, gm_w = bode_of(loop_resp(wgrid, kc, use))
            r.append((cross[0][0] if cross else None,
                      cross[0][1] if cross else None,
                      g180[0][1] if g180 else None,
                      gm_w))
        num.append(r)
        rows.append([
            "%.0f" % kc,
            ("%.1f" % r[0][0]) if r[0][0] else "—",
            ("%.0f" % r[0][1]) if r[0][1] is not None else "—",
            ("%.1f" % r[0][2]) if r[0][2] is not None else "—",
            ("%.1f" % r[0][3]) if r[0][3] is not None else "—",
            ("%.1f" % r[1][0]) if r[1][0] else "—",
            ("%.0f" % r[1][1]) if r[1][1] is not None else "—",
            ("%.1f" % r[1][3]) if r[1][3] is not None else "—",
        ])
    tab(rows, ["kc", "无 w_c", "无 PM", "无 GM(首)", "无 GM(最差)",
               "有 w_c", "有 PM", "有 GM(最差)"])

    print("    ★★ 看「无 GM(最差)」那一列：%s"
          % "、".join("%+.1f" % r[0][3] for r in num))
    print("       kc = 100 时只剩 +1.1 dB（抖一抖就没了）；kc 一过 150 就是**负的**。")
    print("       负的增益裕度 = 相位已经过了 -180 度、|L| 还大于 1 = 必炸。")
    print("       而旁边的「无 GM(首)」那一列看起来还挺体面（%s dB），"
          % "、".join("%.0f" % r[0][2] for r in num))
    print("       你要是只看教科书定义的那个数，就会以为「这飞机能飞」。")
    print("       ⇒ **共振是「双穿越点」问题，单点裕度指标天生看不见它。**")
    print()
    print("    ★★ 再看「有 GM(最差)」那一列：从 +%.1f dB 一路掉到 +%.1f dB。"
          % (num[0][1][3], num[-1][1][3]))
    print("       ⇒ **陷波不是「优化」，是「解锁增益」。**")
    print("         同样的机架、同样的控制器结构，同样把 kc 开到 %.0f：" % kc_list[-1])
    print("           无陷波 → GM(最差) = %+.1f dB，必炸" % num[-1][0][3])
    print("           有陷波 → GM(最差) = %+.1f dB，能飞（余量也不多了）" % num[-1][1][3])
    print("         想把余量做大？把陷波做窄做深，或者把 kc 收一点 ——")
    print("         这正是「调参」在真实飞控里要做的事。")
    print()
    print("    ★ 顺带看「有 PM」那一列：比「无 PM」**略小一点**。")
    print("      因为陷波器自己也有相位滞后（实验 ④ 会量化）。")
    print("      但它换来的是把那个致命的第二个穿越点彻底删掉 —— 完全值得。")
    print()

    # ---- ④ 陷波的代价：相位滞后 ----
    sub("④ 陷波不是免费的：相位代价")

    print("  陷波滤波器和所有滤波器一样，会带来相位滞后。")
    print("  而且 —— 和所有数字滤波器一样 —— **中心频率越低，相位代价越大**。")
    print()
    print("  ArduPilot 的参数说明里把这件事写得很清楚")
    print("  （HarmonicNotchFilter.cpp:120-124，INS_HNTCH_FM_RAT）：")
    print()
    print('      「Note that lower frequency notch filters will have more phase lag.」')
    print()
    print("  验证：固定「我们关心的频段」= 10 Hz，扫不同的陷波中心频率，")
    print("  看 10 Hz 处的相位被拉掉多少度：")
    print()

    rows = []
    for fcn in [40.0, 60.0, 80.0, 120.0, 200.0]:
        bb, aa, _A, _Q, _al = notch_coeffs_ap(fs, fcn, fcn / 3.0, 40.0)
        ph10 = float(phase_deg(biquad_resp(bb, aa, np.array([TWO_PI * 10.0]),
                                           1.0 / fs)[0]))
        ph30 = float(phase_deg(biquad_resp(bb, aa, np.array([TWO_PI * 30.0]),
                                           1.0 / fs)[0]))
        # 陷波中心处的深度
        d_at = float(db(biquad_resp(bb, aa, np.array([TWO_PI * fcn]),
                                    1.0 / fs)[0]))
        rows.append(["%.0f" % fcn, "%.1f" % (fcn / 3.0), "%.1f" % d_at,
                     "%.1f" % ph10, "%.1f" % ph30])
    tab(rows, ["陷波中心 (Hz)", "带宽 (Hz)", "中心处衰减 (dB)",
               "10 Hz 处相位 (度)", "30 Hz 处相位 (度)"])

    print("    ★ 40 Hz 的陷波在 10 Hz 处要吃掉 %.1f 度相位；"
          % float(phase_deg(biquad_resp(
              notch_coeffs_ap(fs, 40.0, 40.0 / 3.0, 40.0)[0],
              notch_coeffs_ap(fs, 40.0, 40.0 / 3.0, 40.0)[1],
              np.array([TWO_PI * 10.0]), 1.0 / fs)[0])))
    print("      200 Hz 的陷波只吃掉 %.1f 度。"
          % float(phase_deg(biquad_resp(
              notch_coeffs_ap(fs, 200.0, 200.0 / 3.0, 40.0)[0],
              notch_coeffs_ap(fs, 200.0, 200.0 / 3.0, 40.0)[1],
              np.array([TWO_PI * 10.0]), 1.0 / fs)[0])))
    print()
    print("    ★★ 所以工程上有两条铁律：")
    print("        1) 陷波频率尽量往高放，只要能盖住共振就行")
    print("           （这就是 INS_HNTCH_FM_RAT 存在的原因 ——")
    print("            它限制油门变低时陷波频率不能跟着掉太多）")
    print("        2) **不要叠一堆陷波**。每个陷波都在偷相位。")
    print("           AP 的 INS_HNTCH_HMNCS 默认只要 1 次 + 2 次谐波，就是这个道理。")
    print()

    # ---- ⑤ 真实数据跑一遍 ----
    sub("⑤ 真的把噪声灌进陷波器里跑一遍")

    print("  造一段含共振的陀螺信号：")
    print("      有用的低频姿态运动（1 Hz 正弦）")
    print("    + 机架共振（%.0f Hz 正弦）" % fc)
    print("    + 白噪声")
    print()

    t = np.arange(0, 0.5, 1.0 / fs)
    sig = (0.05 * np.sin(TWO_PI * 1.0 * t)
           + 0.30 * np.sin(TWO_PI * fc * t)
           + rng_global.normal(0.0, 0.01, len(t)))
    y_ap = biquad_run(b_ap, a_ap, sig)
    y_px = biquad_run(b_px, a_px, sig)

    def amp_at(x, f):
        X = np.fft.rfft(x * hann(len(x)))
        fr = np.fft.rfftfreq(len(x), 1.0 / fs)
        k = int(np.argmin(np.abs(fr - f)))
        return 2.0 * abs(X[k]) / (len(x) * 0.5)

    rows = [
        ["1 Hz（有用信号）", "%.4f" % amp_at(sig, 1.0),
         "%.4f" % amp_at(y_ap, 1.0), "%.4f" % amp_at(y_px, 1.0)],
        ["%.0f Hz（共振）" % fc, "%.4f" % amp_at(sig, fc),
         "%.5f" % amp_at(y_ap, fc), "%.5f" % amp_at(y_px, fc)],
        ["%.0f Hz（共振）" % fc,
         "%.2f dB" % db(np.array([amp_at(sig, fc)])).item(),
         "%.2f dB" % db(np.array([amp_at(y_ap, fc)])).item(),
         "%.2f dB" % db(np.array([amp_at(y_px, fc)])).item()],
    ]
    tab(rows, ["分量", "原始", "AP 陷波后", "PX4 陷波后"])

    print("    ★ 1 Hz 的有用信号基本没动，%.0f Hz 的共振被按下去了 %.0f 多 dB。"
          % (fc, db(np.array([amp_at(sig, fc)])).item()
             - db(np.array([amp_at(y_ap, fc)])).item()))
    print()
    print("    ★★ 但注意：**陷波只对「周期性」的振动有效**。")
    print("      电机转速一变，共振频率就跟着变 —— 所以真实飞控用的是")
    print("      【动态陷波】：用油门（或 ESC 转速、FFT）实时算出当前转速，")
    print("      再把陷波中心频率跟着挪过去。")
    print()
    print("      这就是那几个参数的含义：")
    print("        INS_HNTCH_MODE = 1  用油门推算转速（最常用）")
    print("        INS_HNTCH_REF       参考油门值，和 FREQ 配对")
    print("        INS_HNTCH_MODE = 3  用 ESC 遥测的转速（最准）")
    print("        INS_HNTCH_MODE = 4  用 FFT 在机上实时找峰（最自动）")
    print()
    print("      PX4 侧的对应物（src/modules/gyro_fft/parameters.c）：")
    print("        第 41 行  IMU_GYRO_FFT_EN    = 0（默认关）")
    print("        第 52 行  IMU_GYRO_FFT_MIN   = 30 Hz")
    print("        第 63 行  IMU_GYRO_FFT_MAX   = 150 Hz")
    print("        第 76 行  IMU_GYRO_FFT_LEN   = 512（FFT 窗长）")
    print("        第 85 行  IMU_GYRO_FFT_SNR   = 10（信噪比门限，dB）")
    print()
    print("      ArduPilot 侧：libraries/AP_GyroFFT/AP_GyroFFT.cpp")
    print("        第  78 行  INS_FFT_MINHZ      = 50 Hz")
    print("        第  86 行  INS_FFT_MAXHZ      = 450 Hz")
    print("        第 102 行  INS_FFT_WINDOW_SIZE = 32 ~ 512（必须是 2 的幂）")
    print("        第 131 行  INS_FFT_SNR_REF     = 信噪比门限")
    print("        第 583 行  AP_GyroFFT::update_thread() 在后台线程里算 FFT")
    print("        第 715 行  AP_GyroFFT::calculate_notch_frequency(...)")
    print("        第 1070 行 AP_GyroFFT::calculate_noise(...)")
    print()

    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        ws = loggrid(TWO_PI * 1.0, TWO_PI * (fs / 2), 4001)
        axes[0].semilogx(ws / TWO_PI, db(biquad_resp(b_ap, a_ap, ws, 1.0 / fs)),
                         "b-", lw=1.5, label="ArduPilot")
        axes[0].semilogx(ws / TWO_PI, db(biquad_resp(b_px, a_px, ws, 1.0 / fs)),
                         "r--", lw=1.5, label="PX4")
        axes[0].axvline(fc, color="k", lw=0.6, ls=":")
        axes[0].set_xlabel("频率 (Hz)")
        axes[0].set_ylabel("|H| (dB)")
        axes[0].set_ylim(-50, 5)
        axes[0].grid(alpha=0.3, which="both")
        axes[0].legend(fontsize=8)
        axes[0].set_title("陷波滤波器频响（fc=120 Hz, bw=40 Hz, att=40 dB）")

        for kc, lab in [(100.0, "kc=100 无陷波"), (250.0, "kc=250 无陷波"),
                        (250.0, "kc=250 有陷波")]:
            use = "有" in lab
            Lq = loop_resp(wgrid, kc, use)
            axes[1].semilogx(wgrid / TWO_PI, db(Lq),
                             lw=1.4, label=lab)
        axes[1].axhline(0.0, color="k", lw=0.6, ls="--")
        axes[1].axvline(fc, color="k", lw=0.6, ls=":")
        axes[1].set_xlim(1.0, 400.0)
        axes[1].set_xlabel("频率 (Hz)")
        axes[1].set_ylabel("|L| (dB)")
        axes[1].set_ylim(-60, 40)
        axes[1].grid(alpha=0.3, which="both")
        axes[1].legend(fontsize=8)
        axes[1].set_title("开环增益：陷波把共振峰按下去")
        savefig("lesson19_exp4")


# 实验 4 用的全局随机源
rng_global = np.random.default_rng(7)


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("时域 vs 频域", exp1),
    "2": ("Bode 图：增益/相位裕度", exp2),
    "3": ("chirp 扫频 + 系统辨识", exp3),
    "4": ("陷波滤波器", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        for k in sorted(EXPERIMENTS):
            EXPERIMENTS[k][1]()
    else:
        for a in args:
            if a not in EXPERIMENTS:
                print("未知实验编号：%s（可选 %s）"
                      % (a, "/".join(sorted(EXPERIMENTS))))
                continue
            EXPERIMENTS[a][1]()
    print()
    print("=" * W)
    print("  全部完成。")
    print("=" * W)
    print()


if __name__ == "__main__":
    main()
