#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 11 周 · 03 扩展卡尔曼滤波 EKF —— 配套实验

    实验 1  非线性怎么「破坏」高斯 —— 以及一阶线性化的误差有多大
    实验 2  ★★ 雅可比怎么求：解析 / 前向差分 / 中心差分 / 复步
    实验 3  ★★ 完整的 EKF：极坐标（距离-方位）目标跟踪
    实验 4  ★★★ EKF 的四个坑：线性化误差 / 雅可比写错 / 协方差失正定 / 初始 P 过大

用法：
    python lesson11_ekf.py 1
    python lesson11_ekf.py 2
    python lesson11_ekf.py 3
    python lesson11_ekf.py 4
    python lesson11_ekf.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  和本教程其它脚本一样：**能推的就不背，能算的就不猜。**

  这一课只有一件事要证明：

      卡尔曼滤波 → 扩展卡尔曼滤波，**唯一的变化就是 F 和 H 变成雅可比**。

  所以本脚本用四种**互相独立**的方式算同一组雅可比（实验 2），
  再用它跑一个真正的 EKF（实验 3），最后把四个坑**逐个复现**出来（实验 4）。

  ★ 实验 4 的四个坑全部是**可复现的数值现象**，不是口头警告 ——
    每一个都打印出「滤波器看起来正常，但某个指标已经坏了」的证据。

参考：
    EKF/Joan Sola, Quaternion kinematics for the error-state Kalman filter, 2017.pdf
        §4.3 Useful, and very useful, Jacobians of the rotation
        §6.1.1 Jacobian computation for the filter correction
    EKF/Kalman_and_Bayesian_Filters_in_Python.pdf   第 11 章（EKF）
    EKF/GNSS与惯性及多传感器组合导航系统原理(第二版).pdf   第 4 章
"""

import math
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 输出辅助（和本教程其它脚本一致）
# ---------------------------------------------------------------------------

W = 74


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


def tab(rows, headers, width=None):
    """打印表格。rows 是二维列表。"""
    cols = len(headers)
    if width is None:
        width = [max(len(str(r[i])) for r in [headers] + rows) + 2
                 for i in range(cols)]
    line = "  " + "".join("%-*s" % (width[i], headers[i]) for i in range(cols))
    print(line)
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join("%-*s" % (width[i], str(r[i])) for i in range(cols)))
    print()


# ---------------------------------------------------------------------------
# 通用的小工具
# ---------------------------------------------------------------------------

def skew(v):
    """3x3 反对称矩阵 [v]x，满足 [v]x @ u == cross(v, u)。"""
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def quat_to_R(q):
    """四元数（w, x, y, z，Hamilton）→ 旋转矩阵（body → NED）。"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_mult(a, b):
    """四元数乘法（w, x, y, z）。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis_angle(axis, angle):
    """轴角 → 四元数。angle 可以是小量，此时用 sin(a/2) ≈ a/2。"""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    h = 0.5 * angle
    return np.concatenate(([math.cos(h)], axis * math.sin(h)))


def quat_normalize(q):
    return np.asarray(q, dtype=float) / np.linalg.norm(q)


def trapz(y, x):
    """梯形积分（不依赖 scipy）。"""
    return float(np.sum((y[1:] + y[:-1]) * 0.5 * np.diff(x)))


def _fmt(x):
    """紧凑地打印一个数：大数用科学计数法，避免把表格撑爆。"""
    if not np.isfinite(x):
        return "inf" if x > 0 else "-inf"
    ax = abs(x)
    if ax < 1e-3:
        return "%.6f" % x
    if ax < 1e4:
        return "%.2f" % x
    if ax < 1e7:
        return "%.3g" % x
    return "%.1e" % x


def wrap_pi(a):
    """把角度折到 (-pi, pi]。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# 实验 1：非线性怎么「破坏」高斯
# ---------------------------------------------------------------------------

def exp1():
    head("实验 1  非线性怎么「破坏」高斯 —— 一阶线性化的误差有多大")

    print("""
  飞控里最典型的非线性观测就是【加速度计】：

      静止时机体测到的比力 = 重力在机体轴上的投影

      设只有俯仰角 θ 不确定（其余姿态已知），那么
          a_y = g · sin(θ)          ← 观测量
          a_z = -g · cos(θ)

  ★ θ 是【高斯】的，但 g·sin(θ) 是【非高斯】的 —— 这就是问题的起点。

  本实验要回答三个问题：
      ① 一阶线性化（EKF 的做法）到底错多少？
      ② 误差随 σ 怎么增长（是一次？还是二次）？
      ③ 输出分布到底偏成什么样？
""")

    g = 9.81
    n = 2000001                     # 数值积分的格点数

    def moments(mu_deg, sig_deg):
        """把 N(μ, σ²) 推过 h(θ)=g·sin(θ)，数值求 h 的均值/方差/偏度。

        返回 (均值, 方差, 偏度, 超额峰度)
        """
        mu = math.radians(mu_deg)
        sig = math.radians(sig_deg)
        lo, hi = mu - 8 * sig, mu + 8 * sig
        th = np.linspace(lo, hi, n)
        p = np.exp(-0.5 * (th - mu) ** 2 / sig ** 2) / (math.sqrt(2 * math.pi) * sig)
        y = g * np.sin(th)
        Z = trapz(p, th)
        pn = p / Z
        m = trapz(y * pn, th)
        v = trapz((y - m) ** 2 * pn, th)
        sk = trapz((y - m) ** 3 * pn, th) / v ** 1.5
        ku = trapz((y - m) ** 4 * pn, th) / (v * v) - 3.0
        return m, v, sk, ku

    def ekf_moments(mu_deg, sig_deg):
        """EKF 的一阶线性化预言：均值 = h(μ)，方差 = H(μ)²σ²。"""
        mu = math.radians(mu_deg)
        sig = math.radians(sig_deg)
        return g * math.sin(mu), (g * math.cos(mu)) ** 2 * sig * sig

    sub("① 同一个函数，两个工作点 —— 一个把误差藏起来，一个把它暴露出来")

    print("""    ★★ 这里有个非常值得注意的现象（很多教材不讲）：

        在 μ = 0（水平）附近，sin(θ) ≈ θ 是【奇函数】，
        二阶项 h''(μ)(θ-μ)²/2 里的 h''(0) = -g·sin(0) = 0
        ⇒ **均值偏差恰好被对称性抵消了，看起来「线性化很准」。**

        换个工作点 μ = 60°，这个巧合就没了。
""")

    rows = []
    for sig_deg in (0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 45.0, 60.0):
        m0_t, _, _, _ = moments(0.0, sig_deg)
        m0_e, _ = ekf_moments(0.0, sig_deg)
        m6_t, v6_t, _, _ = moments(60.0, sig_deg)
        m6_e, v6_e = ekf_moments(60.0, sig_deg)
        rows.append(["%.1f°" % sig_deg,
                     "%.2e" % abs(m0_t - m0_e),
                     "%.4f" % m6_t, "%.4f" % m6_e,
                     "%.4f" % abs(m6_t - m6_e),
                     "%.2f%%" % (100 * abs(v6_t - v6_e) / max(v6_t, 1e-30))])

    tab(rows, ["σ", "μ=0 的 ΔE", "μ=60° 真值 E", "μ=60° EKF E",
               "μ=60° ΔE", "μ=60° Var 误差"])
    print("""    ★ 读这张表：

        第 2 列（μ=0）  ：ΔE 恒等于 0 —— **误差被对称性藏起来了**
        第 5 列（μ=60°）：
            σ=1°   偏差 0.0013 m/s²   （相对误差 0.02%）
            σ=10°  偏差 0.128  m/s²   （1.5%）
            σ=30°  偏差 1.09   m/s²   （13%）
            σ=60°  真值只剩 4.91（EKF 说 8.50）—— 差了 1.7 倍

        第 6 列（方差误差）：在 σ=45° 附近达到最大 8% 左右，
            **然后反而下降了** —— 因为 sin 有界，分布已经「绕回来」了。

    ⚠ ⚠ 这两列合起来是一个真实的工程陷阱：
        **你在水平状态（μ=0）标定和测试滤波器，觉得「线性化没问题」；
          一旦飞机倾斜到 60°，同一个滤波器就开始有偏了。**
        ⇒ 「在什么工作点上验证」和「雅可比对不对」同样重要。

    ⚠ 也要诚实地说清边界：σ=60° 那几行已经超出线性化理论的适用范围了。
      「偏差 ∝ σ²」这个规律只在 σ 不太大时成立（见下面 ②）。
""")

    sub("② 误差是 O(σ²) 还是 O(σ)？—— 用比值验证阶数（工作点 μ=60°）")

    print("    理论（精确闭式解，可以手推）：")
    print("        θ ~ N(μ, σ²) ⇒ E[sin θ] = sin(μ)·e^(-σ²/2)")
    print("        ⇒ ΔE = g·sin(μ)·(1 - e^(-σ²/2)) ≈ g·sin(μ)·σ²/2   ← O(σ²)")
    print()

    rows = []
    prev = None
    for sig_deg in (2.0, 4.0, 8.0, 16.0, 32.0):
        m_t, _, _, _ = moments(60.0, sig_deg)
        m_e, _ = ekf_moments(60.0, sig_deg)
        err = abs(m_t - m_e)
        # 闭式解（用来交叉验证数值积分）
        sig = math.radians(sig_deg)
        mu = math.radians(60.0)
        err_exact = abs(g * math.sin(mu) * (1.0 - math.exp(-0.5 * sig * sig)))
        ratio = "—" if prev is None else "%.3f" % (err / prev)
        rows.append(["%.1f°" % sig_deg, "%.6e" % err, "%.6e" % err_exact, ratio])
        prev = err

    tab(rows, ["σ", "数值积分 ΔE", "闭式解 ΔE", "本步/上一步"])
    print("""    ★ σ 每翻一倍，误差变成 4 倍 ⇒ 「本步/上一步」≈ 4 ⇒ **误差是 O(σ²)**。

      （数值积分和闭式解在 12 位有效数字上一致 —— 两条独立路径互验。）

      为什么？泰勒展开：

          h(θ) = h(μ) + h'(μ)(θ-μ) + ½h''(μ)(θ-μ)² + …

          EKF 只保留前两项，求期望时丢掉的那一项是
              E[½h''(μ)(θ-μ)²] = ½h''(μ)·σ²          ← 【二阶，且非零】

      ⇒ ★★ 这就是 EKF「线性化误差」的本质：
            **它不是随机误差，是系统性的偏差，而且随不确定性平方增长。
              你没法靠「多测几次」消掉它 —— 这是模型层面的错。**
""")

    sub("③ 输出分布偏成什么样（偏度 / 峰度，工作点 μ=30°）")

    rows = []
    for sig_deg in (1.0, 5.0, 15.0, 30.0, 60.0):
        _, _, sk, ku = moments(30.0, sig_deg)
        rows.append(["%.1f°" % sig_deg, "%.6f" % sk, "%.6f" % ku])

    tab(rows, ["σ", "偏度（高斯应为 0）", "超额峰度（高斯应为 0）"])
    print("""    ★ 输入是标准高斯（偏度 0、峰度 0），输出【两个都不是 0】——
      **高斯经过非线性函数之后不再是高斯。**

    ⚠ 工作点故意选 μ=30° 而不是 μ=0：
      在 μ=0 时 sin 是奇函数，输出分布【仍然对称】（偏度恰好为 0），
      只有峰度不为 0。**换个工作点两个都会露出来。**
      ⇒ 「用对称的工作点做测试」会漏掉一半的问题。

    ⇒ 那为什么 KF 还能用？
      因为「高斯 + 线性」才封闭。EKF 的做法是**假装**输出还是高斯，
      只是把它的均值和方差按线性化算一遍。

      这个「假装」在 σ ≤ 1° 时几乎无害（偏度 0.03，已经能看出来），
      在 σ ≥ 30° 时就是灾难。
""")
    print("""    ★★ 工程结论（记住这一条）：
        EKF 的精度【只取决于「误差有多大」】，不取决于「非线性有多强」。

        同一个 g·sin(θ)，σ=0.5° 时 EKF 完美，σ=45° 时 EKF 是垃圾。
        ⇒ **这就是为什么飞控启动时要先「对准（align）」**：
          先把不确定度压到几度以内，再开滤波器。
          （PX4 的 EKF2 就是这个流程：静止 → 对准 → 才开始融合）
""")
    print("        配套代码实验 1 结束。")


# ---------------------------------------------------------------------------
# 实验 2：★ 雅可比怎么求
# ---------------------------------------------------------------------------

def _num_jac_forward(f, x, eps):
    """前向差分雅可比。"""
    y0 = f(x)
    m = len(y0)
    n = len(x)
    J = np.zeros((m, n))
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps
        J[:, i] = (f(x + dx) - y0) / eps
    return J


def _num_jac_central(f, x, eps):
    """中心差分雅可比。"""
    n = len(x)
    y0 = f(x)
    J = np.zeros((len(y0), n))
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps
        J[:, i] = (f(x + dx) - f(x - dx)) / (2 * eps)
    return J


def _num_jac_complex(f, x, eps):
    """复步微分：J ≈ Im f(x + i·eps) / eps。

    ★ 它【没有减法相消】，所以 eps 可以取到 1e-200，
      精度直接到机器精度。代价是 f 必须在复域解析
      （不能有 abs / max / atan2 这类分支）。
    """
    n = len(x)
    y0 = f(x)
    J = np.zeros((len(y0), n))
    for i in range(n):
        dx = np.zeros(n, dtype=complex)
        dx[i] = 1j * eps
        J[:, i] = np.imag(f(x + dx)) / eps
    return J


def _jac_Rv_analytic(q, v):
    """h(q, v) = R(q)·v 关于 q 的解析雅可比（3x4），Solà 2017 式 (17)-(19)。

    R(q)·v 对 q 的导数是个 3x4。用 Solà 的结论：
        ∂(R v)/∂q = 2·[ q_w·v + q_v×v ,  q_v·vᵀ + q_w[v]× + q_v vᵀ - v q_vᵀ ... ]
    这里直接用【列向量形式】写出（见课文 §11.4.2 的推导）。
    """
    w, x, y, z = q
    # 依次对 w, x, y, z 求导
    dRdw = 2 * np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]])
    dRdx = 2 * np.array([
        [0, y, z],
        [y, -2 * x, -w],
        [z, w, -2 * x]])
    dRdy = 2 * np.array([
        [-2 * y, x, w],
        [x, 0, z],
        [-w, z, -2 * y]])
    dRdz = 2 * np.array([
        [-2 * z, -w, x],
        [w, -2 * z, y],
        [x, y, 0]])
    return np.column_stack((dRdw @ v, dRdx @ v, dRdy @ v, dRdz @ v))


def _h_range_bearing(x):
    """极坐标观测：z = [r, θ]，x = [px, py, vx, vy]。"""
    px, py = x[0], x[1]
    return np.array([math.hypot(px, py), math.atan2(py, px)])


def _H_range_bearing_analytic(x):
    """极坐标观测的解析雅可比（2x4）。"""
    px, py = x[0], x[1]
    r2 = px * px + py * py
    r = math.sqrt(r2)
    H = np.zeros((2, 4))
    H[0, 0] = px / r
    H[0, 1] = py / r
    H[1, 0] = -py / r2
    H[1, 1] = px / r2
    return H


def exp2():
    head("实验 2  ★★ 雅可比怎么求：解析 / 前向差分 / 中心差分 / 复步")

    print("""
  课文 §11.4 说：EKF 里最容易出错的就是雅可比。
  本实验用【四种独立方法】算同一组雅可比，看它们能不能对上，
  再看它们在什么情况下会互相偏离。

  两个被求导的函数都是飞控里真实出现的：
      (a) h(q, v) = R(q)·v          ← 姿态 → 机体轴上的重力/磁场投影
      (b) h(x)   = [r, θ]（极坐标）  ← 距离-方位观测
""")

    sub("① 函数 (a)：h(q, v) = R(q)·v —— 3 个输出、7 个输入")

    q = quat_normalize(np.array([0.9, 0.3, -0.2, 0.15]))
    v = np.array([0.0, 0.0, -9.81])

    def hqv(x):
        return quat_to_R(x[0:4]) @ x[4:7]

    x0 = np.concatenate((q, v))
    J_ana = _jac_Rv_analytic(q, v)                    # 解析（3x4，只对 q）
    J_fwd = _num_jac_forward(hqv, x0, 1e-6)[:, 0:4]   # 前向差分
    J_ctr = _num_jac_central(hqv, x0, 1e-6)[:, 0:4]   # 中心差分
    J_cpx = _num_jac_complex(hqv, x0, 1e-20)[:, 0:4]  # 复步

    print("    q = [%.4f, %.4f, %.4f, %.4f]（已归一化）" % tuple(q))
    print("    v = [%.2f, %.2f, %.2f]" % tuple(v))
    print("    h = R·v = [%.6f, %.6f, %.6f]" % tuple(hqv(x0)))
    print()

    rows = [
        ["解析", "%.3e" % np.abs(J_ana - J_ana).max()],
        ["前向差分 (eps=1e-6)", "%.3e" % np.abs(J_fwd - J_ana).max()],
        ["中心差分 (eps=1e-6)", "%.3e" % np.abs(J_ctr - J_ana).max()],
        ["复步   (eps=1e-20)", "%.3e" % np.abs(J_cpx - J_ana).max()],
    ]
    tab(rows, ["方法", "与解析解的最大偏差"])

    print("""    ★ 三个结论：
        · 前向差分误差 O(eps)，所以 1e-6 只能给到 1e-6 精度
        · 中心差分误差 O(eps²)，同样 eps 精度高得多
        · **复步微分给到机器精度**（因为它没有减法相消）

    ⚠ 但复步的适用面很窄：函数必须【复解析】——
      有 atan2 / abs / max / if 的地方都不能用。
      飞控代码里到处是这些，所以工程上主力是【解析】和【中心差分】。
""")

    print("""    ⚠ 注意上表里「中心差分」那一行的小异常：
      它在 eps=1e-6 时误差 7.9e-10，**比前向差分的 2e-5 好了 5 个数量级**，
      但仍然没到复步的 3.6e-15。

      原因：R(q)·v 对 q 是【二次多项式】。中心差分对二次函数是
      数学上【精确】的（三阶导数 = 0 ⇒ 截断误差 = 0），
      所以它剩下的 7.9e-10 全是【舍入误差】。

      ⇒ 想看清「截断 vs 舍入」的完整取舍，要换一个非多项式函数。
""")

    sub("② 有限差分步长 eps 的取舍（用非多项式的 r = √(px²+py²) 看完整曲线）")

    xr = np.array([120.0, -80.0, 0.0, 0.0])
    Hr = _H_range_bearing_analytic(xr)[0:1, :]     # 只看距离那一行

    rows = []
    for e in (1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10,
              1e-12, 1e-14):
        ef = np.abs(_num_jac_forward(_h_range_bearing, xr, e)[0:1, :] - Hr).max()
        ec = np.abs(_num_jac_central(_h_range_bearing, xr, e)[0:1, :] - Hr).max()
        rows.append(["%.0e" % e, "%.3e" % ef, "%.3e" % ec])
    tab(rows, ["eps", "前向差分误差", "中心差分误差"])

    print("""    ★ 这一张才是完整的形状 —— 两边都会变差，中间有「甜点区」：

        eps 太大  ⇒ 【截断误差】主导 :
                      前向 O(eps)       （泰勒展开只丢了一阶项之后的东西）
                      中心 O(eps²)
        eps 太 小 ⇒ 【舍入误差】主导 :
                      两个几乎相等的数相减，有效位被吃光，误差 ∝ 1/eps

        最优 eps 的经典估计（令两项相等）：
            前向： eps* ≈ √(机器精度·|f|/|f'|)
            中心： eps* ≈ (机器精度·|f|/|f'''|)^(1/3)

        ⚠ 注意 eps* 和【函数自身的尺度】有关，不是固定的数：
          表里前向差分在 1e-6 附近最好（误差 4e-9），
          中心差分在 1e-3 附近最好（误差 7e-12）。
          「1e-8 / 6e-6」那个常见说法是 f ~ O(1) 时的特例。

    ★★ 由此得到「数值雅可比」的天然精度上限：**大约 1e-8 ~ 1e-11，
        永远到不了机器精度**。

        · 调试时它够用 —— 符号写错、漏项、系数差 2 倍，它一眼就能抓出来
        · 但不能拿它当「真值」去把解析雅可比验到最后一位
        · 想要机器精度的数值雅可比，只有【复步微分】这一条路
          （前提是函数复解析）

    ⚠⚠ 而飞控代码里最多的恰恰是 atan2 / sqrt / 归一化 ——
        **复步用不了**。所以工程上的组合拳是：

            符号工具（SymPy）推解析雅可比
                ↓ 生成代码（或打印出来抄）
            再用【中心差分】做单元测试（容差 1e-6 量级）
                ↓
            上线后用【新息 / NIS】做在线体检（实验 3、4）

        ★ 这正是 PX4 和 ArduPilot 的真实做法，见课文 §11.4.3。
""")

    sub("③ 函数 (b)：极坐标观测的雅可比（实验 3 要用）")

    x = np.array([120.0, -80.0, -5.0, 3.0])
    H_ana = _H_range_bearing_analytic(x)
    H_num = _num_jac_central(_h_range_bearing, x, 1e-7)

    print("    状态 x = [px, py, vx, vy] = [%.1f, %.1f, %.1f, %.1f]" % tuple(x))
    print("    r = %.6f m,  θ = %.6f rad (%.3f°)"
          % (math.hypot(x[0], x[1]), math.atan2(x[1], x[0]),
             math.degrees(math.atan2(x[1], x[0]))))
    print()
    print("    H(解析) =")
    for row in H_ana:
        print("        [% .8f  % .8f  % .8f  % .8f]" % tuple(row))
    print("    H(中心差分) =")
    for row in H_num:
        print("        [% .8f  % .8f  % .8f  % .8f]" % tuple(row))
    print()
    print("    最大偏差 = %.3e" % np.abs(H_ana - H_num).max())
    print()

    print("""    ★★ 手工推一遍（课文 §11.6.2 会详细写）：

        h₁ = r = √(px² + py²)
            ∂r/∂px = px/r      ∂r/∂py = py/r      ∂r/∂vx = ∂r/∂vy = 0

        h₂ = θ = atan2(py, px)
            ∂θ/∂px = -py/(px²+py²) = -py/r²
            ∂θ/∂py = +px/(px²+py²) = +px/r²

        ⇒ H = [ px/r   py/r   0   0 ]
              [-py/r²  px/r²  0   0 ]

    ⚠ 最容易写错的三个地方：
        · 第二行的分母是 r² 不是 r（因为 atan2 求导后分母是 px²+py²）
        · ∂θ/∂px 的【负号】—— 目标往 +x 走，方位角确实变小
        · 后面两列【必须是 0】—— 极坐标观测里没有速度信息
          （忘了写 0 反而没事，写成非 0 就是 bug）
""")
    print("        配套代码实验 2 结束。")


# ---------------------------------------------------------------------------
# 实验 3：★ 完整的 EKF —— 极坐标目标跟踪
# ---------------------------------------------------------------------------

def _run_ekf_polar(truth, zs, x0, P0, R, dt, Q):
    """标准 EKF。返回 (估计序列, P 序列, 新息序列)。"""
    F = np.array([[1.0, 0.0, dt, 0.0],
                  [0.0, 1.0, 0.0, dt],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    x = np.array(x0, dtype=float)
    P = np.array(P0, dtype=float)
    xs, Ps, innovs = [], [], []
    for k in range(len(zs)):
        # --- 预测 ---
        x = F @ x
        P = F @ P @ F.T + Q
        # --- 更新 ---
        z = zs[k]
        if z is not None:
            h = _h_range_bearing(x)
            H = _H_range_bearing_analytic(x)
            y = z - h
            y[1] = wrap_pi(y[1])                   # ★ 方位角的新息要折到 (-π, π]
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x + K @ y
            I = np.eye(4)
            P = (I - K @ H) @ P
            P = 0.5 * (P + P.T)                    # 保持对称
        xs.append(x.copy())
        Ps.append(P.copy())
        innovs.append(None if z is None
                      else np.array([z[0] - _h_range_bearing(x)[0],
                                     wrap_pi(z[1] - _h_range_bearing(x)[1])]))
    return np.array(xs), np.array(Ps), innovs


def _run_linear_kf_cart(truth, zs, x0, P0, R, dt, Q, jac_convert=True):
    """「把极坐标观测转成笛卡尔再用线性 KF」—— 工程上极常见的偷懒做法。

    jac_convert=True  : 用雅可比把极坐标的 R 传到笛卡尔（勉强无偏）
    jac_convert=False : 直接把极坐标的 R 当笛卡尔噪声用（★ 过度自信）

    ★ 这两种都是【近似】，都会引入偏差 —— 只是大小不同（见课文 §11.5.1）。

    返回 (xs, Ps, Rcs)：状态序列、协方差序列、以及每一步实际用到的 R_c。
    """
    F = np.array([[1.0, 0.0, dt, 0.0],
                  [0.0, 1.0, 0.0, dt],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    H = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0]])
    x = np.array(x0, dtype=float)
    P = np.array(P0, dtype=float)
    xs, Ps, Rcs = [], [], []
    for k in range(len(zs)):
        x = F @ x
        P = F @ P @ F.T + Q
        r, th = zs[k]
        zc = np.array([r * math.cos(th), r * math.sin(th)])
        if jac_convert:
            # 把极坐标的 R 通过雅可比 Jc = ∂(x,y)/∂(r,θ) 传到笛卡尔
            Jc = np.array([[math.cos(th), -r * math.sin(th)],
                           [math.sin(th), r * math.cos(th)]])
            Rc = Jc @ R @ Jc.T
        else:
            Rc = R.copy()                    # ★ 直接把极坐标噪声当笛卡尔噪声
        S = H @ P @ H.T + Rc
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ (zc - H @ x)
        P = (np.eye(4) - K @ H) @ P
        P = 0.5 * (P + P.T)                  # 对称化（见课文 §11.6.3）
        xs.append(x.copy())
        Ps.append(P.copy())
        Rcs.append(Rc.copy())
    return np.array(xs), np.array(Ps), np.array(Rcs)


def _make_track(dt, n_steps, p0, v0, seed, sigma_r, sigma_th):
    """生成一条匀速直线轨迹和它的极坐标观测。"""
    rng = np.random.default_rng(seed)
    truth = np.zeros((n_steps, 4))
    for k in range(n_steps):
        truth[k, 0:2] = p0 + v0 * (k * dt)
        truth[k, 2:4] = v0
    zs = []
    for k in range(n_steps):
        r = math.hypot(truth[k, 0], truth[k, 1]) + rng.normal(0, sigma_r)
        th = math.atan2(truth[k, 1], truth[k, 0]) + rng.normal(0, sigma_th)
        zs.append(np.array([r, th]))
    return truth, zs


def exp3():
    head("实验 3  ★★ 完整的 EKF：极坐标（距离-方位）目标跟踪")

    print("""
  这是本课的「主实验」：一个真正的非线性观测系统。

      状态（4 维）  x = [px, py, vx, vy]        目标在 2D 平面里的位置和速度
      运动（线性）  x' = F x + w                F 是匀速直线（CV）模型
      观测（非线性）z = [ r, θ ] = [ √(px²+py²), atan2(py, px) ] + v

  ★ 注意：**状态转移是线性的，观测是非线性的**。
    这是「EKF 只需处理一半非线性」的最常见情形。
    （另一半 —— 状态转移也非线性 —— 见实验 4 的坑 1。）

  对比三个滤波器：

      A. EKF              对 h(x) 求雅可比，直接在极坐标里做观测更新
      B. 线性 KF（雅可比）  先把 (r,θ) 转成 (x,y)，用雅可比把 R 传过去
      C. 线性 KF（naive）   直接把极坐标的 R 当笛卡尔噪声用
                           ★ 这是工程上最常见的偷懒写法，也是最常见的错误来源

  ⚠ 单次仿真的随机性很大，一次跑出来的 RMSE 差 2 倍说明不了问题。
    所以本实验做【60 次蒙特卡洛】，报【平均 RMSE 和平均偏差】。
""")

    dt = 0.1
    n_steps = 320
    p0 = np.array([180.0, 95.0])
    v0 = np.array([-7.0, -3.2])
    sigma_r = 5.0
    sigma_th = math.radians(0.5)
    R = np.diag([sigma_r ** 2, sigma_th ** 2])

    sigma_a = 0.5
    Q = (sigma_a ** 2) * np.array([[dt ** 4 / 4, 0, dt ** 3 / 2, 0],
                                   [0, dt ** 4 / 4, 0, dt ** 3 / 2],
                                   [dt ** 3 / 2, 0, dt ** 2, 0],
                                   [0, dt ** 3 / 2, 0, dt ** 2]])

    x0 = np.array([170.0, 110.0, -5.0, -1.0])
    P0 = np.diag([100.0 ** 2, 100.0 ** 2, 10.0 ** 2, 10.0 ** 2])

    # 2D 叉积（numpy 2.x 的 np.cross 不再支持 2 维向量）
    def cross2(a, b):
        return a[0] * b[1] - a[1] * b[0]

    v_hat = v0 / np.linalg.norm(v0)
    t_cpa = -float(np.dot(p0, v0) / np.dot(v0, v0))
    d_cpa = abs(cross2(p0, v_hat))

    sub("① 真值轨迹：让目标【穿过原点附近】")

    print("    采样 %.0f Hz，共 %.1f s，起点 (%.0f, %.0f)，速度 (%.1f, %.1f) m/s"
          % (1 / dt, n_steps * dt, p0[0], p0[1], v0[0], v0[1]))
    print("    ⇒ 最近点距离 ≈ %.1f m（在第 %.1f s 处通过）" % (d_cpa, t_cpa))
    print()
    print("""    ★ 为什么故意让它靠近原点？
      **距离-方位观测的非线性强度 ∝ 1/r**：
        · r 大（几百米）⇒ 方位角随手一算就线性得很好
        · r 小（十几米）⇒ 目标横移一点，方位角就转很多 ⇒ 强非线性

      ⇒ 同一个滤波器，前半段「看起来完美」，后半段才会露馅。
        这正是「雅可比写错了但滤波器看起来正常」的温床。
""")

    sub("② 蒙特卡洛：60 次独立仿真，分段统计位置 RMSE")

    m1 = n_steps // 3
    m2 = 2 * n_steps // 3
    modes = [("EKF（极坐标 + 雅可比）", "ekf"),
             ("线性 KF（雅可比转换）", "lin_jac"),
             ("线性 KF（naive 转换）", "lin_naive")]

    stats = {m: {"all": [], "a": [], "b": [], "c": [], "bias": []}
             for _, m in modes}

    n_mc = 60
    for seed in range(n_mc):
        truth, zs = _make_track(dt, n_steps, p0, v0, seed, sigma_r, sigma_th)
        for _, mode in modes:
            if mode == "ekf":
                xs, _, _ = _run_ekf_polar(truth, zs, x0, P0, R, dt, Q)
            else:
                xs, _, _ = _run_linear_kf_cart(truth, zs, x0, P0, R, dt, Q,
                                               jac_convert=(mode == "lin_jac"))
            e = xs[:, 0:2] - truth[:, 0:2]
            stats[mode]["all"].append(math.sqrt(float(np.mean(np.sum(e ** 2, 1)))))
            stats[mode]["a"].append(math.sqrt(float(np.mean(np.sum(e[0:m1] ** 2, 1)))))
            stats[mode]["b"].append(math.sqrt(float(np.mean(np.sum(e[m1:m2] ** 2, 1)))))
            stats[mode]["c"].append(math.sqrt(float(np.mean(np.sum(e[m2:] ** 2, 1)))))
            stats[mode]["bias"].append(e[m2:].mean(0))

    rows = []
    for name, m in modes:
        st = stats[m]
        rows.append([name,
                     "%.3f" % np.mean(st["all"]),
                     "%.3f" % np.mean(st["a"]),
                     "%.3f" % np.mean(st["b"]),
                     "%.3f" % np.mean(st["c"]),
                     "(%+.3f, %+.3f)" % (np.mean([x[0] for x in st["bias"]]),
                                         np.mean([x[1] for x in st["bias"]]))])
    tab(rows, ["滤波器", "全程 RMSE", "远距段", "中距段", "近距段",
               "近距段偏差 (x, y)"])

    print("""    ★★ 读这张表（三段分别对应 r 大 / 中 / 小）：

        ① **远距段**：三个滤波器差不多
           ⇒ 因为 r 大时 h(x) 几乎是线性的，转换误差很小。
              **如果你只在这里测过，你会以为偷懒的做法没问题。**

        ② **近距段**：差距拉开
           · EKF 最好
           · 「雅可比转换」次之，但已经出现【明显偏差】（见最后一列）
           · 「naive 转换」最差 —— 近距段 RMSE 是 EKF 的 4 倍

        ③ **最后一列是本节的关键**，它把误差拆成了两份：
           · EKF 的近距段偏差基本为 0（厘米级，在噪声里）
           · 「雅可比转换」的偏差是【分米级】的系统性偏移 —— 多测消不掉
           · 「naive 转换」的偏差反而很小 —— 但它的 RMSE 最大

    ⚠ 一定要分清这两件事（它们是误差的两个完全不同的来源）：
        · **偏差（bias）**     = 系统性偏移，多测【不能】消掉  ← 表格最后一列
        · **方差（variance）** = 随机抖动，多测【能】平均掉    ← RMSE 里的另一部分

      「雅可比转换」的问题主要在【偏差】（非线性转换把噪声搬到了错误的均值上）；
      「naive 转换」的问题主要在【方差】——
      它把 (r,θ) 的噪声当成各向同性的笛卡尔噪声，
      而实际上切向噪声只有 r·σθ ≈ 0.1 m，比径向的 5 m 小 50 倍。
      ⇒ 它以为切向「测得极准」，于是几乎完全无视预测、跟着带噪观测跑。
""")

    sub("③ 新息统计 —— 判断滤波器「自洽」的体检指标")

    print("""    ★★ 工程上判断「滤波器是否诚实」，看的通常不是 RMSE（真值你一般不知道），
      而是【新息 e = z - h(x̂)】的统计特性。

      两个指标：
        · 新息的【均值】必须 ≈ 0 —— 不为 0 就说明滤波器有系统偏差
        · 新息的【NIS】= eᵀS⁻¹e，S = HPHᵀ + R
          模型正确时 NIS ~ χ²(2)（自由度 = 观测维数 2）
          理论均值 = 2.0，理论 95% 分位数 = 5.991
""")

    rows = []
    nis_rows = []
    for name, mode in modes:
        rs_all, ts_all, nis_all = [], [], []
        for seed in range(20):
            truth, zs = _make_track(dt, n_steps, p0, v0, seed, sigma_r, sigma_th)
            if mode == "ekf":
                xs, Ps, _ = _run_ekf_polar(truth, zs, x0, P0, R, dt, Q)
                for k in range(1, n_steps):
                    Hk = _H_range_bearing_analytic(xs[k])
                    S = Hk @ Ps[k] @ Hk.T + R
                    rp = math.hypot(xs[k, 0], xs[k, 1])
                    tp = math.atan2(xs[k, 1], xs[k, 0])
                    e = np.array([zs[k][0] - rp, wrap_pi(zs[k][1] - tp)])
                    rs_all.append(e[0])
                    ts_all.append(e[1])
                    nis_all.append(float(e @ np.linalg.solve(S, e)))
            else:
                # 线性 KF 的 NIS 只能在【它自己的坐标系】里算：
                #   e_c = z_c - H x̂,   S_c = H P Hᵀ + R_c
                # ★ 这正是滤波器自认为的「新息协方差」——
                #   如果 R_c 是错的，NIS 会立刻把这件事捅出来。
                H = np.array([[1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0]])
                xs, Ps, Rcs = _run_linear_kf_cart(
                    truth, zs, x0, P0, R, dt, Q,
                    jac_convert=(mode == "lin_jac"))
                for k in range(n_steps):
                    r, th = zs[k]
                    zc = np.array([r * math.cos(th), r * math.sin(th)])
                    ec = zc - H @ xs[k]
                    Sc = H @ Ps[k] @ H.T + Rcs[k]
                    nis_all.append(float(ec @ np.linalg.solve(Sc, ec)))
                    rp = math.hypot(xs[k, 0], xs[k, 1])
                    tp = math.atan2(xs[k, 1], xs[k, 0])
                    rs_all.append(zs[k][0] - rp)
                    ts_all.append(wrap_pi(zs[k][1] - tp))
        rows.append([name,
                     "%+.4f" % np.mean(rs_all), "%.3f" % np.std(rs_all),
                     "%+.5f" % np.mean(ts_all), "%.5f" % np.std(ts_all)])
        nis_rows.append([name, "%.3f" % np.mean(nis_all),
                         "%.3f" % np.percentile(nis_all, 95)])

    tab(rows, ["滤波器", "距离新息均值", "距离新息 σ", "方位新息均值", "方位新息 σ"])
    print("""    （新息一律换算回【极坐标】比较，单位：米 / 弧度。观测噪声是 σ_r = 5 m、σ_θ = 0.5° = 0.0087 rad。）

    ★ 先看 σ 那两列 —— 它们应该和观测噪声是【同一个量级】：
        · EKF 与「雅可比转换」：σ_r ≈ 4.8 m、σ_θ ≈ 0.0078 rad，
          和 5 m / 0.0087 rad 基本吻合
          （略小一点是正常的：状态估计本身「吃掉」了一部分观测噪声）。
        · 「naive 转换」：距离 σ 反而更小，但方位 σ ≈ 0.049 rad ≈ 2.8°，
          是 EKF 的【6 倍以上】。
          ⇒ 它自以为方位测得很准，于是几乎完全无视预测、跟着带噪观测跑。

    ★★ 关键在【均值】那两列：它们必须 ≈ 0。
        · EKF：两个均值都在噪声水平内，干净。
        · 「雅可比转换」：方位均值干净，但【距离均值是显著的正偏】——
          这就是表格最后一列那个分米级位置偏差在观测空间里的投影。
""")

    tab(nis_rows, ["滤波器", "NIS 均值", "NIS 95% 分位"])
    print("""    ★ 理论值：χ²(2) 的均值 = 2.0，95% 分位数 = 5.991。

      ⇒ EKF 和「雅可比转换」的 NIS 都在理论值同一量级（略偏小 = 略微保守，
        这是正常的：过程噪声 Q 只是真随机游走的粗略模型）；
        「naive 转换」的 NIS 则大得离谱 ——
        **滤波器自己都不知道自己错得这么厉害**，这是最危险的一类 bug。

      ★★ 但请注意一个【反例】：
        「雅可比转换」的 NIS 看起来非常健康（和 EKF 几乎一样），
        可它在表格里明明是【有偏】的。
        ⇒ **NIS 只能证明「协方差自洽」，不能证明「估计无偏」。**
          滤波器可以非常自信地错着 —— 这就是坑 2 最难抓的地方。

      ★ 所以工程上从不「只看 RMSE」，而是盯着 NIS / 新息均值做在线监控
        （PX4 和 ArduPilot 都有一套类似的创新门限和滤波器健康度检查）。
""")

    print("        配套代码实验 3 结束。")
# ---------------------------------------------------------------------------
# 实验 4：★★★ EKF 的四个坑
# ---------------------------------------------------------------------------

def _setup_exp4():
    """复用一个统一的非线性系统设定：单摆式的角度观测。

    状态 x = [θ, ω]，θ 是角度、ω 是角速度。
    运动（非线性！）：θ' = θ + ω·dt,  ω' = ω - (g/L)·sin(θ)·dt
    观测（非线性）：  z = g·sin(θ)      ← 加速度计的 y 轴
    """
    g = 9.81
    L = 1.0
    dt = 0.02
    n = 500
    rng = np.random.default_rng(7)

    # 真值
    th, om = math.radians(30.0), 0.0
    truth = np.zeros((n, 2))
    for k in range(n):
        truth[k] = [th, om]
        om += -(g / L) * math.sin(th) * dt
        th += om * dt

    sigma_z = 0.05
    zs = g * np.sin(truth[:, 0]) + rng.normal(0, sigma_z, n)
    return g, L, dt, truth, zs, sigma_z


def _exp4_pitfall1(g, L, dt, truth, zs, sigma_z):
    sub("坑 1：线性化误差 —— 离工作点越远越差")

    print("""    线性化是「在 x̂ 附近」的近似。如果 x̂ 离真值很远，
    雅可比 F、H 都是【在错误的点上】算的 —— 预测和修正的方向本身就是错的。

    本实验的单摆还有一个额外的陷阱：观测是 h(θ) = g·sin(θ)，
    而 sin 有一个【镜像对称】：

        g·sin(θ) ≡ g·sin(180° - θ)

    ⇒ 观测本身分不清 θ 和 180°-θ。真值在 +30° 附近摆动时，
      「镜像解」在 180°-30° = 150° 附近摆动，
      **它和真值相差 142°（平均），却给出几乎一样的观测！**

    做法：同一个滤波器，只改【初始角度猜测】，看它最后停在哪个解上。
""")

    # 「镜像解」：θ_m = 180° - θ_true，它满足同一个观测方程
    mirror = math.pi - truth[:, 0]
    print("    （参考：镜像解到真值的平均距离 = %.1f°）"
          % math.degrees(np.mean(np.abs(wrap_pi(mirror - truth[:, 0])))))
    print()

    n = len(zs)
    rows = []
    for dth_deg in (1.0, 5.0, 20.0, 45.0, 90.0, 150.0):
        x = np.array([math.radians(30.0) + math.radians(dth_deg), 0.0])
        P = np.diag([math.radians(5.0) ** 2, 0.5 ** 2])
        R = np.array([[sigma_z ** 2]])
        Q = np.diag([1e-6, 1e-4])
        ths = []
        for k in range(n):
            th, om = x[0], x[1]
            x_pred = np.array([th + om * dt, om - (g / L) * math.sin(th) * dt])
            F = np.array([[1.0, dt],
                          [-(g / L) * math.cos(th) * dt, 1.0]])
            P = F @ P @ F.T + Q
            x = x_pred
            h = g * math.sin(x[0])
            H = np.array([[g * math.cos(x[0]), 0.0]])
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x + (K @ np.array([[zs[k] - h]]))[:, 0]
            P = (np.eye(2) - K @ H) @ P
            P = 0.5 * (P + P.T)
            ths.append(x[0])
        ths = np.array(ths)
        errs = np.abs(np.array([wrap_pi(ths[k] - truth[k, 0]) for k in range(n)]))
        err_mir = np.abs(np.array([wrap_pi(ths[k] - mirror[k]) for k in range(n)]))
        tail = float(np.degrees(np.mean(errs[n // 2:])))
        tail_mir = float(np.degrees(np.mean(err_mir[n // 2:])))
        rows.append(["%+.0f°" % dth_deg,
                     "%.3f°" % math.degrees(errs[0]),
                     "%.3f°" % math.degrees(errs.max()),
                     "%.3f°" % tail,
                     "%.3f°" % tail_mir,
                     "收敛到真解" if tail < 1.0 else "★ 锁死在镜像解"])

    tab(rows, ["初始 θ 偏差", "首步更新后误差", "最大误差", "末段误差",
               "末段离镜像解", "结论"])
    print("""    ★★ 逐行读这张表，它比「发散」更阴险：

        · 偏差 ≤ 20°：能拉回来，末段误差 0.139° = 稳态噪声水平。
        · 偏差 45°：**第一次更新就把估计值推过了头**（首步误差 53.2°），
          注意「最大误差 = 首步误差」⇒ 之后是一路收敛的。
          原因：H = g·cos(75°) 很小，但新息 z - h 很大，
          一阶近似算出来的修正量【方向和大小都错了】。
        · 偏差 ≥ 90°：**滤波器稳稳地停在了镜像解上** ——
          末段离真值 139° 远，但离镜像解只有 3.25°。

    ★★ 这是比「发散」危险得多的失败模式：
        输出是一条**光滑、漂亮、看起来完全正常的正弦曲线**，
        协方差也没有爆炸，代码更不会报错。
        只有当你拿真值去比时，才发现它从一开始就错了 142°。

    ⇒ ★★ 这一条直接解释了飞控的三个设计：
        ① **为什么必须先「对准」再飞** —— 把初始不确定性压到足够小，
           否则滤波器可能收敛到「另一个姿态解」上去
        ② **为什么观测必须有【互补性】** ——
           单靠加速度计测重力方向，姿态永远有 180° 的镜像模糊；
           所以必须有磁力计（或 GPS 航向）来打破这个对称性
           （这正是第 12 课 ESKF 要解决的问题）
        ③ **为什么 EKF 常有「重置（reset）」逻辑** —— 发现误差太大就重来
           （PX4 的 `resetQuatStateYaw`、AP 的 `ResetQuatStateYawOnly` 都是这个）
""")
    return


def _exp4_pitfall2():
    sub("坑 2：雅可比写错 —— 最隐蔽的 bug")

    print("""    ★★★ 这是本课最重要的一个坑。

    雅可比写错时，滤波器**不会报错、不会崩、不会 NaN，
    协方差也一直是正定的**。
    它会输出一条看起来很像样的曲线 —— 然后在几十秒里慢慢偏掉。

    本实验用一个飞控里最典型的系统：

        状态  x = [θ, b]      θ = 姿态角，b = 陀螺零偏（rad/s）
        预测  θ' = θ + (ω_测量 - b)·dt ,  b' = b
              ⇒ F = [ 1  -dt ]        ← ★ 偏置项的符号是【最容易写错】的一处
                    [ 0   1  ]
        观测  z  = θ                （姿态参考：加速度计/磁力计解算出的姿态）
              ⇒ H = [ 1  0 ]

    ★ 为什么偏置项的符号最容易错？
      因为它取决于你用【局部误差】还是【全局误差】约定 ——
      这两种约定下同一个式子的符号正好相反。
      （第 12 课会讲清楚这件事；PX4 的代码生成脚本
        `derivation.py` 里就明确写着它用的是哪一种。）

    真值：θ(t) = 40°·sin(1.0·t)，陀螺零偏 b = 0.005 rad/s = 0.29°/s
    采样 50 Hz，共 50.0 s；姿态参考噪声 σ = 0.5°
""")

    dt2 = 0.02
    n = 2500
    A = math.radians(40.0)
    OM = 1.0
    b_true = 0.005
    sigma_z = math.radians(0.5)
    sigma_w = 0.002

    rng = np.random.default_rng(11)
    th_true = np.zeros(n)
    wm = np.zeros(n)
    for k in range(n):
        t = k * dt2
        th_true[k] = A * math.sin(OM * t)
        wm[k] = A * OM * math.cos(OM * t) + b_true + rng.normal(0, sigma_w)
    zs = th_true + rng.normal(0, sigma_z, n)

    q = n // 4

    def run(f_mode, h_mode):
        x = np.array([0.0, 0.0])
        P = np.diag([math.radians(5.0) ** 2, 0.1 ** 2])
        R = np.array([[sigma_z ** 2]])
        Q = np.diag([math.radians(0.05) ** 2, 1e-10])
        et, eb, ivs, nis, minlam = [], [], [], [], []
        for k in range(n):
            th, b = x[0], x[1]
            x_pred = np.array([th + (wm[k] - b) * dt2, b])
            F = np.array([[1.0, (-dt2 if f_mode == "correct" else +dt2)],
                          [0.0, 1.0]])
            P = F @ P @ F.T + Q
            x = x_pred
            H = np.array([[1.0 if h_mode == "correct" else -1.0, 0.0]])
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            innov = zs[k] - x[0]      # 新息 = 真观测 - 预测观测（与 H 怎么写无关）
            x = x + (K @ np.array([[innov]]))[:, 0]
            P = (np.eye(2) - K @ H) @ P
            P = 0.5 * (P + P.T)
            et.append(abs(wrap_pi(x[0] - th_true[k])))
            eb.append(abs(x[1] - b_true))
            ivs.append(innov)
            nis.append(float(innov * innov / S[0, 0]))
            minlam.append(np.linalg.eigvalsh(P).min())
        return (np.array(et), np.array(eb), np.array(ivs),
                np.array(nis), np.array(minlam))

    modes = [("correct", "correct", "全部正确"),
             ("wrong", "correct", "★ F 的偏置项符号写反"),
             ("correct", "wrong", "★★ H 的符号写反（对照）")]
    res = {}
    rows = []
    for fm, hm, name in modes:
        et, eb, ivs, nis, ml = run(fm, hm)
        res[name] = (et, eb, ivs, nis, ml)
        rows.append([name,
                     "%.3f°" % math.degrees(np.mean(et[:q])),
                     "%.3f°" % math.degrees(np.mean(et[q:2 * q])),
                     "%.3f°" % math.degrees(np.mean(et[2 * q:3 * q])),
                     "%.3f°" % math.degrees(np.mean(et[3 * q:])),
                     _fmt(np.mean(eb[3 * q:]))])
    tab(rows, ["雅可比", "θ误差 0-25%", "25-50%", "50-75%", "75-100%",
               "零偏误差(末段)"])

    print("""    ★★★ 这就是「最隐蔽」的确切含义 —— 请只看「F 符号写反」那一行：

        θ 误差：  1.191°  →  3.835°  →  6.512°  →  9.334°

      **它是单调增长的，但增长得很慢**：
      第一段只有 1.2°，比姿态参考噪声（0.5°）大不了多少，
      你会以为是「今天振动有点大」；
      到 25 s 时 9.3°，你才觉得不对 —— 而这时已经来不及了。

      ⇒ 同时注意【零偏误差】：末段已经错到 0.87 rad/s，
        是真实零偏 0.005 rad/s 的【170 倍】——
        滤波器不但没估出零偏，还朝反方向跑掉了。

    ★★ 对照组（H 的符号写反）反而是「好」错误：它当场就崩得离谱。
      **能让你一眼看出来的 bug 不可怕，可怕的是看不出来的那个。**
""")

    rows = []
    for fm, hm, name in modes:
        et, eb, ivs, nis, ml = res[name]
        rows.append([name,
                     _fmt(np.mean(ivs)),
                     _fmt(np.mean(nis[:q])),
                     _fmt(np.mean(nis[q:2 * q])),
                     _fmt(np.mean(nis[2 * q:3 * q])),
                     _fmt(np.mean(nis[3 * q:])),
                     "%.2e" % ml.min()])
    tab(rows, ["雅可比", "新息均值", "NIS 0-25%", "NIS 25-50%", "NIS 50-75%",
               "NIS 75-100%", "min λ(P)"])

    print("""    ★★ 这张表回答「什么时候能发现它」：

        · **新息均值**：θ 的误差 1.2° 时，新息均值已经明显偏离 0。
          ⇒ 这是最早、最便宜的报警指标（一个滑动平均就够了）。

        · **NIS**（理论均值 = 1.0，自由度 1）：
          正确滤波器全程 ≈ 1.0；
          符号写反的滤波器 9.82 → 68.19 → 191.18 → 389.32，
          和 θ 误差同步增长。
          ⇒ ★★ NIS 的真正价值是它【自带门槛】：
            χ²(1) 的 95% 分位数是 3.84，
            所以在线算 NIS，超过 3.84 就能【自动】报警 ——
            不用等你看出来误差变大了。

        · **min λ(P)**：三个滤波器的最小特征值**完全一样**（8.46e-07），
          都是正数 —— 也就是说，**协方差从头到尾都是「健康」的**。
          ⇒ ★★★ 这是本课最该记住的一句话：
             P 只描述【滤波器自己以为的不确定性】，
             它对自己写错的雅可比【一无所知】。
             所以「协方差没崩」完全不能当作「滤波器是对的」的证据。
""")

    print("""    ★★ 三个可操作的纪律（本教程一直在强调这个）：

        ① 写完雅可比，**必须用数值差分验一遍**（实验 2 就是在教这个工具）
        ② 上线前看**新息均值**：它必须在 0 附近
        ③ 上线后看 **NIS**：它必须接近自由度
           （PX4 的 `test_ratio`、AP 的 `innovGate` 就是在做这件事）

    ⚠ 第 ① 条要在【每一个工作点】验，不能只验一个。
      因为雅可比是 x 的函数 —— 在悬停点对，在 60° 倾斜可能就错了。
""")
    return
def _exp4_pitfall3():
    sub("坑 3：协方差矩阵失去正定性 —— 纯数值问题")

    print("""    协方差必须【对称正定】（第 1 课讲过：vᵀPv = Var[vᵀx] ≥ 0）。
    否则会出现「负的方差」，滤波器立刻失去意义。

    但教科书上的那条更新式

        P ← (I - K H) P

    **在浮点运算下既不保证对称，也不保证正定** ——
    Solà 2017 的脚注 26 专门警告过这一点：

        「(I-KH)P … is known to have poor numerical stability,
          as its outcome is not guaranteed to be symmetric nor
          positive definite.」

    本实验真的把它跑出来。配置要同时满足三个条件才会翻车：

        · float32（飞控 MCU 上大量用单精度）
        · 初始 P₀ 的条件数很大（本例 1e8 —— 位置方差 1 m² 和
          某个 1e-8 量级的状态方差放在同一个矩阵里）
        · R 极小（测量「极准」，1e-6）⇒ (I-KH) 里发生减法相消

    做法：6 维状态、2 维观测、连续做 800 次【纯更新】（不做预测），
    每个时刻记录 λ_min(P)。四种写法对比。
""")

    n = 6
    rng = np.random.default_rng(5)
    U, _ = np.linalg.qr(rng.normal(0, 1, (n, n)))
    P0 = U @ np.diag(np.logspace(0, -8, n)) @ U.T
    H = np.zeros((2, n))
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    R = 1e-6 * np.eye(2)

    print("    P₀ 的条件数 = %.3e，λ(P₀) 从 %.1e 到 %.1e"
          % (np.linalg.cond(P0), np.linalg.eigvalsh(P0).min(),
             np.linalg.eigvalsh(P0).max()))
    print()

    def run(form, dtype, iters=800):
        P = P0.astype(dtype)
        Hd = H.astype(dtype)
        Rd = R.astype(dtype)
        I = np.eye(n, dtype=dtype)
        worst_lam = np.inf
        worst_asym = 0.0
        for _ in range(iters):
            S = (Hd @ P @ Hd.T + Rd).astype(np.float64).astype(dtype)
            K = P @ Hd.T @ np.linalg.inv(S).astype(dtype)
            if form == "naive":
                P = (I - K @ Hd) @ P          # ★ 教科书那一行
            elif form == "naive_sym":
                P = (I - K @ Hd) @ P
                P = 0.5 * (P + P.T)           # ★ 再做一次强制对称（ArduPilot 的 ForceSymmetry）
            elif form == "sym":
                P = P - K @ (Hd @ P)          # ★ 另一种常见的展开写法
            else:
                P = (I - K @ Hd) @ P @ (I - K @ Hd).T + K @ Rd @ K.T   # Joseph
            P = P.astype(dtype)
            Pf = P.astype(np.float64)
            worst_lam = min(worst_lam, np.linalg.eigvalsh(Pf).min())
            worst_asym = max(worst_asym, np.abs(Pf - Pf.T).max())
        return worst_lam, worst_asym

    rows = []
    for dtype, dname in ((np.float32, "float32"), (np.float64, "float64")):
        for form, fname in (("naive", "(I-KH)P"),
                            ("naive_sym", "(I-KH)P + 强制对称"),
                            ("sym", "P - K(HP)"),
                            ("joseph", "Joseph 形式")):
            # float64 下四种写法结果完全相同，只跑一次避免表格冗余
            lam, asym = run(form, dtype)
            rows.append([dname, fname,
                         "%.4e" % lam,
                         "%.3e" % asym,
                         "❌ 失正定" if lam < 0 else "OK"])
    tab(rows, ["精度", "更新形式", "最小特征值 (全程最小)", "最大不对称度", "结论"])

    print("""    ★★ 逐行读 —— 这个结果是本课最干净的一个「反例」：

        float32 + (I-KH)P            λ_min = -9.93e-06   ❌
        float32 + (I-KH)P + 强制对称   λ_min = -9.89e-06   ❌   ← 不对称度已经 = 0 了！
        float32 + P-K(HP)            λ_min = -9.93e-06   ❌
        float32 + Joseph             λ_min = +1.23e-09   OK
        float64 + 四种写法             λ_min = +1.21e-09   OK

      ⇒ 三个结论：
        ① **同样的公式，float32 会翻车、float64 不会** —— 这是纯粹的数值问题
        ② 「强制对称 P ← (P+Pᵀ)/2」**只能治对称，治不了正定**：
           加了这一句之后最大不对称度确实降到了 0，
           但 λ_min 依然是 -9.89e-06。
           ★★ 这正是 ArduPilot 要【两个】函数的原因 ——
             `ForceSymmetry()` 管对称，
             `ConstrainVariances()` 另外管方差上下限。
        ③ **Joseph 形式在浮点下依然保住了正定性**

    ★ 为什么 (I-KH)P 会翻车？三步误差叠加：

        ① 求逆 S⁻¹ 引入舍入误差
        ② 两个大矩阵相乘把误差放大
        ③ (I - KH) 这个【减法】发生相消 ——
           当 KH 的元素接近 1 时，有效位数被吃掉大半

      P 的最小特征值本来就小（1e-8 量级），
      它最先被这些误差淹没，于是变成负数。

    ★★ 工程上的三种对策（按推荐度）：

        ① **Joseph 形式**（最常用）
              P ← (I-KH) P (I-KH)ᵀ + K R Kᵀ
           它【天然对称】（两个对称项相加），
           而且可以证明在精确运算下恒正定。
           ⚠ 代价是计算量约为 (I-KH)P 的两倍。

        ② **强制对称 + 方差上下限**（便宜、工程上最普遍）
              P ← (P + Pᵀ)/2
              再把每个对角元夹到 [v_min, v_max]
           ★ ArduPilot 里就有两个专门的函数干这个：
             `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1871-1882`（原文）：
             ```c
             // force symmetry on the covariance matrix to prevent ill-conditioning
             void NavEKF3_core::ForceSymmetry()
             {
                 for (uint8_t i=1; i<=stateIndexLim; i++)
                 {
                     for (uint8_t j=0; j<=i-1; j++)
                     {
                         ftype temp = 0.5f*(P[i][j] + P[j][i]);
                         P[i][j] = temp;
                         P[j][i] = temp;
                     }
                 }
             }
             ```
             紧接着 `AP_NavEKF3_core.cpp:1887` 的 `ConstrainVariances()`
             给每个状态的方差设上下限，例如姿态误差那一行是
             `P[i][i] = constrain_ftype(P[i][i], 0.0, 1.0);`。

        ③ **平方根滤波 / UD 分解**
           直接维护 P 的 Cholesky 因子（`P = L Lᵀ`），只更新 L。
           L 恒有效 ⇒ P 恒正定，由【构造】保证。
           ★ PX4 的 EKF2 就是**平方根形式**的：
             状态协方差在内部以「方差 + 下三角」的形式保存和更新。
           代价是计算量大、代码复杂、可读性差。

    ⚠ 还有一个更隐蔽的来源：**Q 和 R 的量纲差异太大**。
      位置方差可以到 1e6（公里级），姿态误差方差只有 1e-12（微弧度级），
      放在同一个 23x23 矩阵里就差了 18 个数量级 ⇒ 条件数爆炸。
      ⇒ ★★ 这就是为什么工程实现里**必须给每个状态单独设方差上下限**：
        PX4 EKF2 的 `src/modules/ekf2/EKF/covariance.cpp:212-238`
        里有一个 `Ekf::constrainStateVariances()`，逐状态调用
        `constrainStateVar(State::quat_nominal, 1e-9f, 1.f);`
        —— 四元数那 3 个误差状态被夹在 [1e-9, 1] 之间。
        ❗ 注意：**强制对称 + 夹上下限是本脚本里的做法，
          它不是数学上严格的**，只是工程上足够好用；
          要严格就得上 ③。
""")
    return
def _exp4_pitfall4(g, L, dt, truth, zs, sigma_z):
    sub("坑 4：P₀ 报大了不等于保险 —— 「越谦虚，第一跳越猛」")

    print("""    很多人的直觉是：

        「初值我随便填一个，把 P₀ 设得很大就行了 ——
          反正滤波器知道自己不知道，会自己收敛过去。」

    ★★ 这个直觉是【错的】，而且错得很危险。本实验用一张网格表把它拆开。

    做法：把【初始猜测偏差】和【P₀ 的大小】两个因素分开扫。
    注意 P₀ 只描述「我以为自己有多不准」，
    而【初始猜测偏差】才是「我真的有多不准」——
    这两件事可以完全不一致，本实验就是要看谁说了算。
""")

    n = len(zs)

    def run(dth_deg, p0_deg):
        x = np.array([math.radians(30.0 + dth_deg), 0.0])
        P = np.diag([math.radians(p0_deg) ** 2, 0.5 ** 2])
        R = np.array([[sigma_z ** 2]])
        Q = np.diag([1e-6, 1e-4])
        ths = []
        for k in range(n):
            a, b = x[0], x[1]
            x = np.array([a + b * dt, b - (g / L) * math.sin(a) * dt])
            F = np.array([[1.0, dt],
                          [-(g / L) * math.cos(a) * dt, 1.0]])
            P = F @ P @ F.T + Q
            h = g * math.sin(x[0])
            H = np.array([[g * math.cos(x[0]), 0.0]])
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x + (K @ np.array([[zs[k] - h]]))[:, 0]
            P = (np.eye(2) - K @ H) @ P
            P = 0.5 * (P + P.T)
            ths.append(x[0])
        ths = np.array(ths)
        e = np.abs(np.array([wrap_pi(ths[k] - truth[k, 0]) for k in range(n)]))
        return math.degrees(e[0]), math.degrees(np.mean(e[n // 2:]))

    p0s = (0.5, 5.0, 90.0, 360.0)
    dths = (1.0, 20.0, 45.0, 60.0, 75.0, 90.0, 120.0, 150.0)

    print("  表 A：末段（后 5 s）平均误差，单位度")
    print()
    rows = []
    for d in dths:
        row = ["%+.0f°" % d]
        for p in p0s:
            _, tail = run(d, p)
            row.append("%.2f" % tail if tail < 10 else "★ %.0f" % tail)
        rows.append(row)
    tab(rows, ["初始猜测偏差"] + ["P₀ σ=%.1f°" % p for p in p0s])

    print("""    ★★ 表 A 的读法 —— 请【竖着】读，再【横着】读：

        · **横着读**（同一行，P₀ 从 0.5° 变到 360°）：
          **整行几乎不变！**
          ⇒ P₀ 报大 700 倍，最后的精度【一点没变好】。

        · **竖着读**（同一列，初始偏差从 +1° 变到 +150°）：
          在 +60° 和 +75° 之间有一个【陡峭的悬崖】：
            偏差 ≤ 60°  ⇒ 末段误差 0.14°（= 稳态噪声水平）
            偏差 ≥ 75°  ⇒ 末段误差 139° —— 锁死在镜像解上

          （细分扫描显示：偏差超过 60° 后开始【时好时坏】——
            因为初始猜测越过了 90°，落到了 sin 的另一侧；
            超过约 72° 之后就【必然】锁死。）

    ⇒ ★★★ 结论：**决定成败的是【初值】，不是【P₀】。**
      P₀ 只是你对不确定性的【声明】，它改变不了你真的离真值有多远。
""")

    print("  表 B：第一次观测更新【之后】的误差，单位度")
    print()
    rows = []
    for d in dths:
        row = ["%+.0f°" % d]
        for p in p0s:
            e0, _ = run(d, p)
            row.append("%.2f" % e0)
        rows.append(row)
    tab(rows, ["初始猜测偏差"] + ["P₀ σ=%.1f°" % p for p in p0s])

    print("""    ★★ 但 P₀ 并非毫无作用 —— 它决定【第一跳有多猛】。
      请在表 B 里横着读 +45° 那一行：

          P₀ σ=0.5°  →  首步误差 12.66°
          P₀ σ=5°    →  首步误差 53.21°
          P₀ σ=90°   →  首步误差 58.13°
          P₀ σ=360°  →  首步误差 58.14°

      **P₀ 报得越大，第一跳越猛**，而且很快「饱和」不再增加。

    ★ 为什么饱和？把卡尔曼增益写开就明白了：
         K = P Hᵀ (H P Hᵀ + R)⁻¹
      当 P → ∞ 时，R 可以忽略，于是
         K → P Hᵀ (H P Hᵀ)⁻¹ = H⁺   （H 的伪逆）
      对本实验的标量观测，H⁺ = 1/H，所以状态被**一步扔到 h(x) = z 的那个点上**
      （一阶意义下）。这正是「第一跳」的极限大小。

    ★★ 而这一步恰恰是最危险的：
      「一步跳到 h(x) = z 的点」在一阶意义下是对的，
      **但 sin(θ) 有两个点满足 h = z** ——
      跳到哪一个，取决于你从哪边出发。
      跳得越猛，越没有机会在半路上「发现不对、慢慢纠正」。

    ⇒ ★★ 所以「P₀ 设大一点更保险」这句话要反过来读：
        · P₀ 报【小】了 ⇒ 滤波器过度自信 ⇒ 无视观测、慢慢偏掉
        · P₀ 报【大】了 ⇒ 滤波器过度谦虚 ⇒ 第一跳跳到极限位置，
          如果那里线性化已经不成立，就【一步到位地错下去】
        **两个方向都有代价。P₀ 应该等于你【真实的】初始不确定性。**
""")

    print("""    ⇒ 工程上的对策：

        ① **初值必须用别的办法拿到**，不能指望 P₀ 兜底：
           · 飞控上电时的「对准（alignment）」就是在做这件事
           · 或者先做一次批处理最小二乘 / 平均值，再交给 EKF 递推
        ② **开滤波后的前几个观测要设门限**（innovation gating / 卡方检验）
           ⇒ 新息太大的观测先丢掉，不要让 K 一次把它全吃进去
           （PX4 的 `test_ratio` 门限、AP 的 `innovGate` 参数就是这个）
        ③ **P₀ 要诚实** —— 它应该反映你【真实的】初始不确定度，
           既不是「填个 0 表示我知道」，也不是「填个 1e6 表示我不知道」
        ④ 用【NIS / 新息均值】在线盯着，一发现不诚实就重置
""")
    return
def exp4():
    head("实验 4  ★★★ EKF 的四个坑（每一个都现场复现）")

    print("""
  课文 §11.5 列了四个坑。这一节把它们**全部跑出来**，
  并且每一坑都给出「能提前发现它的指标」。

      坑 1  线性化误差        —— 离工作点越远越差
      坑 2  雅可比写错        —— 滤波器看起来正常，慢慢发散  ★★★
      坑 3  协方差失正定      —— 数值问题
      坑 4  P₀ 报大≠保险     —— 「越谦虚，第一跳越猛」

  统一用同一个系统：单摆式的角度观测

      状态  x = [θ, ω]
      f(x)   θ' = θ + ω·dt,   ω' = ω - (g/L)·sin(θ)·dt     ← 非线性
      h(x)   z  = g·sin(θ)                                  ← 非线性
""")

    g, L, dt, truth, zs, sigma_z = _setup_exp4()
    print("    真值：θ₀ = 30°，ω₀ = 0；采样 %.0f Hz，共 %.1f s"
          % (1 / dt, len(zs) * dt))
    print("    观测噪声 σ_z = %.3f m/s²" % sigma_z)

    _exp4_pitfall1(g, L, dt, truth, zs, sigma_z)
    _exp4_pitfall2()
    _exp4_pitfall3()
    _exp4_pitfall4(g, L, dt, truth, zs, sigma_z)

    print("""
  ==========================================================================
    实验 4 小结：四个坑的「体检指标」
  ==========================================================================

    坑                    早期症状            能提前发现的指标
    --------------------  ------------------  --------------------------
    1 线性化误差          误差偏大            误差随「离工作点多远」增长
    2 雅可比写错          什么都看不出 ★★★    新息均值 ≠ 0 ；NIS 偏离自由度
    3 协方差失正定        P 的最小特征值 < 0  λ_min(P) ；P 的不对称度
    4 P₀ 与初值不匹配     第一次跳变过猛      首步跳变量；NIS / 新息均值

  ★★ 这四个指标里，**【新息 + NIS】是唯一能在「还看不出问题」时报警的**。
     这正是 PX4/AP 的 EKF 里那一大堆「创新门限 / 卡方检验」存在的原因
     （第 14、15 课会详细讲）。
""")
    print("        配套代码实验 4 结束。")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("非线性怎么破坏高斯 + 线性化误差", exp1),
    "2": ("★★ 雅可比怎么求：四种方法对比", exp2),
    "3": ("★★ 完整的 EKF：极坐标目标跟踪", exp3),
    "4": ("★★★ EKF 的四个坑（现场复现）", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        for k in sorted(EXPERIMENTS):
            print("\n" + "#" * W)
            print("#  实验 %s：%s" % (k, EXPERIMENTS[k][0]))
            print("#" * W)
            EXPERIMENTS[k][1]()
        print()
        print("=" * W)
        print("  完成。")
        print("=" * W)
        return 0

    for a in args:
        if a not in EXPERIMENTS:
            print("  未知实验：%s（可选 %s）" % (a, "/".join(sorted(EXPERIMENTS))))
            return 1
        EXPERIMENTS[a][1]()

    print()
    print("=" * W)
    print("  完成。")
    print("=" * W)
    return 0


if __name__ == "__main__":
    sys.exit(main())
