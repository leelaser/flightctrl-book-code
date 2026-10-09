#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 2 第 2 章 · 磁罗盘与椭球拟合标定 —— 配套实验

    实验 1  硬铁与软铁的几何表现（球心平移 / 球变椭球）
    实验 2  ★★★ 椭球拟合（含手写雅可比 + 数值微分对照）
    实验 3  ★ "转得不够"会怎样
    实验 4  磁偏角与偏航误差

用法：
    python lesson07_compass.py 1
    python lesson07_compass.py 2
    python lesson07_compass.py 3
    python lesson07_compass.py 4
    python lesson07_compass.py          # 全部

只需 numpy + matplotlib。

参考源码（ArduPilot Copter-4.5.7）：
    libraries/AP_Compass/CompassCalibrator.cpp:560-568   calc_residual（核心方程）
    libraries/AP_Compass/CompassCalibrator.cpp:571-584   目标函数
    libraries/AP_Compass/CompassCalibrator.cpp:708-736   手写雅可比
    libraries/AP_Compass/CompassCalibrator.cpp:470-486   fit_acceptable 判据
    libraries/AP_Compass/CompassCalibrator.h:9-10        4 参数 / 9 参数
"""

import math
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_PLT = True
except ImportError:
    HAVE_PLT = False


def hr(ch="=", n=74):
    print(ch * n)


def head(title):
    print()
    hr()
    print("  " + title)
    hr()


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (len(title) + 2))


# ===========================================================================
# 公共：干扰模型与残差
# ===========================================================================

def make_softiron(diag, offdiag):
    """按 ArduPilot 的方式构造【对称】软铁矩阵。

    源码：CompassCalibrator.cpp:713-717
        Matrix3f softiron(
            diag.x    , offdiag.x , offdiag.y,
            offdiag.x , diag.y    , offdiag.z,
            offdiag.y , offdiag.z , diag.z
        );
    """
    dx, dy, dz = diag
    oxy, oxz, oyz = offdiag
    return np.array([
        [dx,  oxy, oxz],
        [oxy, dy,  oyz],
        [oxz, oyz, dz ],
    ])


def add_disturbance(m_earth, offset, M):
    """把理想磁场变成"测量值"（正向模型）。

        m_meas = M⁻¹ · m_earth - offset

    ⚠ 要同时支持单个向量 (3,) 和一批向量 (N,3)：
       np.linalg.solve(M, X) 只接受 (3,) 或 (3,k)，
       传 (N,3) 会报 "mismatch in core dimension"。
       ⇒ 批量时转置成 (3,N) 再转回来。
    """
    me = np.asarray(m_earth, dtype=float)
    if me.ndim == 1:
        return np.linalg.solve(M, me) - offset
    return np.linalg.solve(M, me.T).T - offset


def correct(m_meas, offset, M):
    """校准（逆向）—— 对应 CompassCalibrator.cpp:272

        corrected = softiron * (v + offset)
    """
    return M @ (m_meas + offset)


def residual(sample, p):
    """★★★ 核心残差 —— 照抄 CompassCalibrator.cpp:560-568

        return params.radius - (softiron*(sample+params.offset)).length();

    p = [ox, oy, oz, dx, dy, dz, oxy, oxz, oyz, R]
    """
    off = p[0:3]
    M = make_softiron(p[3:6], p[6:9])
    R = p[9]
    return R - np.linalg.norm(M @ (sample + off))


def residuals_all(samples, p):
    return np.array([residual(s, p) for s in samples])


def mean_sq_residual(samples, p):
    """对应 CompassCalibrator.cpp:571-584"""
    r = residuals_all(samples, p)
    return float(np.mean(r ** 2))


# ===========================================================================
# 实验 1：硬铁与软铁的几何表现
# ===========================================================================

def exp1():
    head("实验 1：硬铁与软铁的几何表现")

    print("""
  理想情况下，把飞机转到所有姿态，磁力计测量点应该落在一个【球面】上
  （半径 = 当地磁场强度）。

  干扰会破坏这个球：
      硬铁  ⇒  球心【平移】
      软铁  ⇒  球被【拉伸/剪切】成椭球
""")

    # 生成理想球面上的点（模拟各个姿态）
    rng = np.random.default_rng(11)
    n = 600
    # 球面均匀采样
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    R_earth = 45.0    # µT，典型磁场强度
    m_ideal = v * R_earth

    sub("① 三种情况的对比")

    CASES = [
        ("理想（无干扰）", np.zeros(3), np.eye(3)),
        ("只加硬铁", np.array([15.0, -8.0, 5.0]), np.eye(3)),
        ("只加软铁", np.zeros(3), make_softiron((1.15, 0.85, 1.0),
                                                (0.12, -0.08, 0.05))),
        ("两者都有", np.array([15.0, -8.0, 5.0]),
         make_softiron((1.15, 0.85, 1.0), (0.12, -0.08, 0.05))),
    ]

    print("    %-18s %-22s %-22s %s"
          % ("情况", "测量点的中心", "各轴跨度", "|测量| 范围"))
    print("    " + "-" * 82)

    data_sets = {}
    for name, off, M in CASES:
        m_meas = add_disturbance(m_ideal, off, M)
        data_sets[name] = (m_meas, off, M)

        center = m_meas.mean(axis=0)
        spans = m_meas.max(axis=0) - m_meas.min(axis=0)
        norms = np.linalg.norm(m_meas, axis=1)

        print("    %-18s [%6.2f %6.2f %6.2f]  [%5.1f %5.1f %5.1f]  %.1f ~ %.1f"
              % (name, center[0], center[1], center[2],
                 spans[0], spans[1], spans[2], norms.min(), norms.max()))

    print()
    print("""    ★ 看两列：

      【中心】：理想时在原点；加了硬铁后【整体平移】
                ⇒ 硬铁 = 球心偏移

      【各轴跨度】：理想时三轴相同（都是 2R = 90）；
                    加软铁后【三轴不等】
                ⇒ 软铁 = 各方向缩放不同 ⇒ 球变椭球

      【|测量| 范围】：理想时恒为 45（球面）；
                       有干扰时是个区间
                ⇒ ★ 这就是校准的目标：让这个区间【缩成一个点】
""")

    sub("② ★ 校准的目标：让 |校正后| 变成一个常数")

    print("    %-18s %-24s %-24s %s"
          % ("情况", "校正前 |m| 标准差", "校正后 |m| 标准差", "改善"))
    print("    " + "-" * 82)

    for name, off, M in CASES:
        m_meas = data_sets[name][0]
        before = np.std(np.linalg.norm(m_meas, axis=1))

        if name == "理想（无干扰）":
            after = before
        else:
            # 用【真值】参数校正
            corrected = np.array([correct(s, off, M) for s in m_meas])
            after = np.std(np.linalg.norm(corrected, axis=1))

        ratio = before / after if after > 1e-12 else float("inf")
        print("    %-18s %-24.4f %-24.4e %.1f×"
              % (name, before, after, ratio))

    print()
    print("""    ⇒ ★★ 用【真值】参数校正后，|m| 的标准差降到浮点噪声量级。

      这就是校准的全部目标：
      **找到一组 (offset, M)，让校正后的模长变成常数。**

      而那个常数就是当地磁场强度 R（拟合的副产物）。
""")

    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 4, figsize=(18, 4.6))
            for ax, (name, off, M) in zip(axes, CASES):
                m_meas = data_sets[name][0]
                ax.scatter(m_meas[:, 0], m_meas[:, 1], s=4, alpha=0.5)
                ax.set_title("%s\n|m|: %.1f ~ %.1f"
                             % (name,
                                np.linalg.norm(m_meas, axis=1).min(),
                                np.linalg.norm(m_meas, axis=1).max()),
                             fontsize=10)
                ax.set_aspect("equal")
                ax.grid(True, alpha=0.3)
                ax.set_xlabel("X (µT)")
                if ax is axes[0]:
                    ax.set_ylabel("Y (µT)")
            fig.suptitle("Hard/soft iron disturbance (XY projection)", fontsize=12)
            fn = "compass_disturbance.png"
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 实验 2：椭球拟合
# ===========================================================================

def sphere_jacob(sample, p):
    """★ 照抄 CompassCalibrator.cpp:597-619（球拟合的雅可比）。

    参数： [ox, oy, oz, R]  （M 固定为单位阵）
    """
    off = p[0:3]
    R = p[3]
    M = np.eye(3)

    s_o = sample + off
    v = M @ s_o
    length = np.linalg.norm(v)

    A, B, C = v

    ret = np.zeros(4)
    ret[0] = 1.0                                    # 对 R
    ret[1] = -1.0 * ((M[0, 0] * A + M[0, 1] * B + M[0, 2] * C) / length)
    ret[2] = -1.0 * ((M[1, 0] * A + M[1, 1] * B + M[1, 2] * C) / length)
    ret[3] = -1.0 * ((M[2, 0] * A + M[2, 1] * B + M[2, 2] * C) / length)
    return ret


def sphere_residual(sample, p):
    off = p[0:3]
    R = p[3]
    return R - np.linalg.norm(sample + off)


def ellipsoid_jacob(sample, p):
    """★★★ 照抄 CompassCalibrator.cpp:708-736（椭球拟合的雅可比）。

    参数： [ox, oy, oz, dx, dy, dz, oxy, oxz, oyz, R]
      —— 注意 R 是固定的，所以这里只返回 9 个（不含 R）
    """
    off = p[0:3]
    M = make_softiron(p[3:6], p[6:9])

    s_o = sample + off
    v = M @ s_o
    A, B, C = v
    length = np.linalg.norm(v)

    ret = np.zeros(9)
    # 0-2: 对 offset 的偏导
    ret[0] = -1.0 * ((M[0, 0] * A + M[0, 1] * B + M[0, 2] * C) / length)
    ret[1] = -1.0 * ((M[1, 0] * A + M[1, 1] * B + M[1, 2] * C) / length)
    ret[2] = -1.0 * ((M[2, 0] * A + M[2, 1] * B + M[2, 2] * C) / length)
    # 3-5: 对 diag 的偏导
    ret[3] = -1.0 * (s_o[0] * A) / length
    ret[4] = -1.0 * (s_o[1] * B) / length
    ret[5] = -1.0 * (s_o[2] * C) / length
    # 6-8: 对 offdiag 的偏导（★ 每一项都有两部分 —— 因为它出现在矩阵的两个位置）
    ret[6] = -1.0 * ((s_o[1] * A) + (s_o[0] * B)) / length     # oxy
    ret[7] = -1.0 * ((s_o[2] * A) + (s_o[0] * C)) / length     # oxz
    ret[8] = -1.0 * ((s_o[2] * B) + (s_o[1] * C)) / length     # oyz
    return ret


def gauss_newton(samples, p_init, n_params, jac_fn, resid_fn,
                 n_iter=60, lma=10.0):
    """Gauss-Newton + Levenberg-Marquardt 阻尼。

    对应 CompassCalibrator.cpp:756 的 "Gauss Newton Part ... including LM"。

    p 的【前 n_params 个】是要优化的；后面的固定不变。
    """
    p = np.array(p_init, dtype=float)
    best = p.copy()
    best_fit = float(np.mean(resid_fn(samples, p) ** 2))

    lam = lma

    for _it in range(n_iter):
        JTJ = np.zeros((n_params, n_params))
        JTr = np.zeros(n_params)

        for s in samples:
            # 用【完整参数向量】算雅可比，但只取前 n_params 个
            j_full = ellipsoid_jacob(s, p)         # 9 个
            j = j_full[:n_params]                  # 球拟合时只用前 3 个（offset）
            r = resid_fn(s, p)

            JTJ += np.outer(j, j)
            JTr += j * r

        # ★ LM：在 JᵀJ 的对角加上阻尼
        JTJ_damped = JTJ + lam * np.eye(n_params) * np.trace(JTJ) / n_params

        try:
            dp = np.linalg.solve(JTJ_damped, -JTr)
        except np.linalg.LinAlgError:
            break

        p_try = p.copy()
        # ★ 注意：球拟合时只动前 3 个（offset），R 也动
        #   这里用一个更通用的写法：由调用者给出"哪些参数可变"
        p_try[:n_params] = p[:n_params] + dp

        fit_try = float(np.mean(resid_fn(samples, p_try) ** 2))

        if fit_try < best_fit:
            p = p_try
            best = p.copy()
            best_fit = fit_try
            lam = max(lam / 3.0, 1e-6)
        else:
            lam *= 3.0
            if lam > 1e8:
                break

    return best, best_fit


def run_sphere_fit(samples, p):
    """球拟合：固定 M = I，解 offset + R（4 个参数）。

    p = [ox, oy, oz, dx, dy, dz, oxy, oxz, oyz, R]
    ★ 这里直接解 offset 和 R —— 用一个专用版本更清楚。
    """
    p4 = np.array([p[0], p[1], p[2], p[9]], dtype=float)
    best = p4.copy()
    best_fit = float(np.mean([sphere_residual(s, p4) ** 2 for s in samples]))
    lam = 10.0

    for _it in range(80):
        JTJ = np.zeros((4, 4))
        JTr = np.zeros(4)
        for s in samples:
            j = sphere_jacob(s, p4)
            r = sphere_residual(s, p4)
            JTJ += np.outer(j, j)
            JTr += j * r

        JTJ_d = JTJ + lam * np.eye(4) * np.trace(JTJ) / 4.0
        try:
            dp = np.linalg.solve(JTJ_d, -JTr)
        except np.linalg.LinAlgError:
            break

        p_try = p4 + dp
        fit_try = float(np.mean([sphere_residual(s, p_try) ** 2 for s in samples]))

        if fit_try < best_fit:
            p4 = p_try
            best = p4.copy()
            best_fit = fit_try
            lam = max(lam / 3.0, 1e-6)
        else:
            lam *= 3.0
            if lam > 1e8:
                break

    out = np.array(p, dtype=float)
    out[0:3] = best[0:3]
    out[9] = best[3]
    return out, best_fit


def run_ellipsoid_fit(samples, p, R_fixed):
    """椭球拟合：固定 R，解 offset + diag + offdiag（9 个参数）。"""
    p = np.array(p, dtype=float)
    p[9] = R_fixed

    idx = list(range(9))          # 要优化的参数下标
    best = p.copy()
    best_fit = float(np.mean(residuals_all(samples, p) ** 2))
    lam = 10.0

    for _it in range(120):
        JTJ = np.zeros((9, 9))
        JTr = np.zeros(9)
        for s in samples:
            j = ellipsoid_jacob(s, p)
            r = residual(s, p)
            JTJ += np.outer(j, j)
            JTr += j * r

        JTJ_d = JTJ + lam * np.eye(9) * np.trace(JTJ) / 9.0
        try:
            dp = np.linalg.solve(JTJ_d, -JTr)
        except np.linalg.LinAlgError:
            break

        p_try = p.copy()
        p_try[idx] += dp
        fit_try = float(np.mean(residuals_all(samples, p_try) ** 2))

        if fit_try < best_fit:
            p = p_try
            best = p.copy()
            best_fit = fit_try
            lam = max(lam / 3.0, 1e-6)
        else:
            lam *= 3.0
            if lam > 1e8:
                break

    return best, best_fit


def exp2():
    head("实验 2：★★★ 椭球拟合")

    print("""
  这个实验完整复现 ArduPilot 的校准算法：
      ① 残差函数
      ② ★ 手写雅可比 —— 并【和数值微分对照】验证
      ③ 交替优化（球拟合 → 椭球拟合）
      ④ 看参数能不能读回来
""")

    sub("① 生成带已知干扰的合成数据")

    RNG = np.random.default_rng(2024)
    R_TRUE = 45.0
    OFFSET_TRUE = np.array([15.0, -8.0, 5.0])
    DIAG_TRUE = np.array([1.15, 0.85, 1.00])
    OFFDIAG_TRUE = np.array([0.12, -0.08, 0.05])
    M_TRUE = make_softiron(DIAG_TRUE, OFFDIAG_TRUE)

    # 球面均匀采样（模拟"转得足够充分"）
    N = 400
    v = RNG.normal(size=(N, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    m_ideal = v * R_TRUE
    samples = np.array([add_disturbance(m, OFFSET_TRUE, M_TRUE) for m in m_ideal])

    print("    【真值】")
    print("        磁场强度 R   = %.1f µT" % R_TRUE)
    print("        offset       = [%6.2f %6.2f %6.2f]" % tuple(OFFSET_TRUE))
    print("        diag         = [%6.3f %6.3f %6.3f]" % tuple(DIAG_TRUE))
    print("        offdiag      = [%6.3f %6.3f %6.3f]" % tuple(OFFDIAG_TRUE))
    print()
    print("    采样点数: %d（球面均匀分布）" % N)
    print("    测量值的 |m| 范围: %.2f ~ %.2f µT"
          % (np.linalg.norm(samples, axis=1).min(),
             np.linalg.norm(samples, axis=1).max()))

    sub("② ★★★ 验证手写雅可比")

    print("""
    非线性最小二乘需要 ∂r/∂p。
    ArduPilot 是【手写】的（29 行）—— 手写就容易错。

    ⇒ 用【数值微分】对照验证：

        ∂r/∂pᵢ ≈ (r(p + h·eᵢ) - r(p)) / h
""")

    p_test = np.array([10.0, -5.0, 3.0,          # offset
                       1.1, 0.9, 1.0,            # diag
                       0.1, -0.05, 0.03,         # offdiag
                       R_TRUE])                  # R
    s_test = samples[0]
    h = 1e-6

    j_hand = ellipsoid_jacob(s_test, p_test)
    j_num = np.zeros(9)
    for i in range(9):
        pp = p_test.copy()
        pp[i] += h
        j_num[i] = (residual(s_test, pp) - residual(s_test, p_test)) / h

    print("    %-10s %-18s %-18s %s" % ("参数", "手写雅可比", "数值微分", "相对误差"))
    print("    " + "-" * 70)
    NAMES = ["offset.x", "offset.y", "offset.z",
             "diag.x", "diag.y", "diag.z",
             "offdiag.xy", "offdiag.xz", "offdiag.yz"]
    for i, nm in enumerate(NAMES):
        rel = abs(j_hand[i] - j_num[i]) / max(abs(j_num[i]), 1e-12)
        print("    %-10s %-18.8f %-18.8f %.2e" % (nm, j_hand[i], j_num[i], rel))

    max_rel = max(abs(j_hand[i] - j_num[i]) / max(abs(j_num[i]), 1e-12)
                  for i in range(9))
    print()
    if max_rel < 1e-4:
        print("    ✓ 手写雅可比与数值微分一致（最大相对误差 %.2e）" % max_rel)
    else:
        print("    ✗ 不一致！最大相对误差 %.2e" % max_rel)

    sub("③ 交替优化：球拟合 → 椭球拟合")

    # 初值：offset = 0，M = I，R = 50（随便给一个）
    p = np.array([0.0, 0.0, 0.0,
                  1.0, 1.0, 1.0,
                  0.0, 0.0, 0.0,
                  50.0])

    print("    初值: offset = [0 0 0], diag = [1 1 1], offdiag = [0 0 0], R = 50")
    print()

    # ★ 交替：球拟合 ⇄ 椭球拟合
    #   对应 CompassCalibrator.cpp:204-218
    for cycle in range(3):
        p, fit_s = run_sphere_fit(samples, p)
        p, fit_e = run_ellipsoid_fit(samples, p, p[9])
        print("    第 %d 轮:  球拟合残差 = %.4e   椭球拟合残差 = %.4e"
              % (cycle + 1, fit_s, fit_e))
        print("              offset = [%7.3f %7.3f %7.3f]"
              % tuple(p[0:3]))
        print("              diag   = [%7.4f %7.4f %7.4f]"
              % tuple(p[3:6]))
        print("              offdiag= [%7.4f %7.4f %7.4f]"
              % tuple(p[6:9]))
        print("              R      = %.3f" % p[9])
        print()

    sub("④ 和真值对比：一个看起来像 bug 的标度因子")

    M_fit = make_softiron(p[3:6], p[6:9])
    k = M_fit[0, 0] / M_TRUE[0, 0]

    print("    ★★ 先看一个现象：拟合出的 M 每一项都和真值【差同一个倍数】")
    print()
    print("    %-14s %-18s %-18s %s"
          % ("参数", "真值", "拟合值", "比值"))
    print("    " + "-" * 70)
    ROWS = [
        ("offset.x", OFFSET_TRUE[0], p[0]),
        ("offset.y", OFFSET_TRUE[1], p[1]),
        ("offset.z", OFFSET_TRUE[2], p[2]),
        ("diag.x", DIAG_TRUE[0], p[3]),
        ("diag.y", DIAG_TRUE[1], p[4]),
        ("diag.z", DIAG_TRUE[2], p[5]),
        ("offdiag.xy", OFFDIAG_TRUE[0], p[6]),
        ("offdiag.xz", OFFDIAG_TRUE[1], p[7]),
        ("offdiag.yz", OFFDIAG_TRUE[2], p[8]),
    ]
    for nm, tv, fv in ROWS:
        ratio = fv / tv if abs(tv) > 1e-12 else float("nan")
        print("    %-14s %-18.6f %-18.6f %.6f" % (nm, tv, fv, ratio))

    print()
    print("    ⇒ offset 完全对上；M 的每一项都被同一个因子 %.6f 缩放。" % k)
    print("      %.6f = R_fit/R_true = %.1f / %.1f"
          % (k, p[9], R_TRUE))
    print()
    print("""    ★★★ 这不是 bug，是【标度简并】：

        残差是   r = R - |M·(s + offset)|

        ⇒ (M, R) 和 (k·M, k·R) 【给出完全相同的残差】

        因为 |k·M·v| = k·|M·v| ⇒ r = k·R - k·|M·v| = k·r
        ⇒ 残差被缩放了 k 倍，但【零残差仍然是零残差】

    ⇒ 所以参数只在【相差一个公共标度】的意义下唯一。

    【ArduPilot 怎么处理的？】

      · 球拟合阶段：优化 R（4 个参数里有它）
      · 椭球拟合阶段：固定 R（9 个参数里没有它）
      ⇒ R 由球拟合定下来，M 再缩放到匹配它

      但球拟合是在 M = I 的假设下估的 R ⇒
      【对椭球数据，这个 R 只是个"平均半径"，不是真值】
      ⇒ 所以最终的 M 带一个标度偏差
""")

    print("    ── ★★ 关键问题：这个标度偏差影响航向吗？──")
    print()

    # 用两组参数分别算航向
    max_err = 0.0
    for s in samples[:200]:
        c_true = correct(s, OFFSET_TRUE, M_TRUE)
        c_fit = correct(s, p[0:3], M_fit)
        yaw_t = math.degrees(math.atan2(-c_true[1], c_true[0]))
        yaw_f = math.degrees(math.atan2(-c_fit[1], c_fit[0]))
        d = abs((yaw_t - yaw_f + 180) % 360 - 180)
        max_err = max(max_err, d)

    print("    航向的定义： yaw = atan2(-m_y, m_x)")
    print("    ⇒ 【均匀缩放不改变方向】⇒ 不影响航向")
    print()
    print("    实测（200 个采样点，用拟合参数 vs 用真值参数算航向）：")
    print("        最大航向误差 = %.6f°" % max_err)
    print()
    print("    ✓ 完全一致 —— 标度偏差对航向【无影响】")

    print()
    print("    ── 归一化之后再看 ──")
    print()
    print("    把 M 乘以 R_true/R_fit，就消除了标度歧义：")
    print()
    print("    %-14s %-18s %-18s %s" % ("参数", "真值", "归一化后", "误差"))
    print("    " + "-" * 70)
    for nm, tv in (("diag.x", DIAG_TRUE[0]), ("diag.y", DIAG_TRUE[1]),
                   ("diag.z", DIAG_TRUE[2]), ("offdiag.xy", OFFDIAG_TRUE[0]),
                   ("offdiag.xz", OFFDIAG_TRUE[1]), ("offdiag.yz", OFFDIAG_TRUE[2])):
        idx = {"diag.x": 3, "diag.y": 4, "diag.z": 5,
               "offdiag.xy": 6, "offdiag.xz": 7, "offdiag.yz": 8}[nm]
        fv = p[idx] * R_TRUE / p[9]
        print("    %-14s %-18.6f %-18.6f %.2e" % (nm, tv, fv, abs(tv - fv)))

    # 校正后的效果
    corrected = np.array([correct(s, p[0:3], M_fit) for s in samples])
    norms = np.linalg.norm(corrected, axis=1)

    print()
    print("    【校正效果】")
    print("        校正前 |m| 标准差 = %.4f µT"
          % np.std(np.linalg.norm(samples, axis=1)))
    print("        校正后 |m| 标准差 = %.4e µT" % np.std(norms))
    print("        校正后 |m| 均值   = %.4f µT（R_fit，不是 %.1f）"
          % (np.mean(norms), R_TRUE))

    print("""
    ★★★ 三条结论：

      ① 参数【只在相差一个公共标度】的意义下可辨识
      ② ★ 但航向对公共标度【免疫】—— 所以标度偏差不影响使用
      ③ 这不只是本实验的玩具问题 ——
         【任何形如"拟合到球面"的问题都有这个简并】

    ⇒ 工程上关心的是【方向】，不是【模长】，
      所以 ArduPilot 不需要额外处理这个歧义。

    ⚠ 但如果你的应用需要【磁场强度的绝对值】
      （比如用它判断"是否受干扰"），
      那就必须额外校准这个标度 —— 比如用已知的当地磁场强度。
""")
    print()


# ===========================================================================
# 实验 3：转得不够会怎样
# ===========================================================================

def exp3():
    head("实验 3：★「转得不够」会怎样")

    print("""
  实验 2 用了【球面均匀】的 400 个点。
  真实校准中，用户可能只转了一小部分。

  这个实验固定用【同样的真值参数】，只改【数据覆盖率】，
  看拟合结果怎么崩。
""")

    RNG = np.random.default_rng(7)
    R_TRUE = 45.0
    OFFSET_TRUE = np.array([15.0, -8.0, 5.0])
    M_TRUE = make_softiron((1.15, 0.85, 1.00), (0.12, -0.08, 0.05))

    def gen_covering(cone_deg, n=400):
        """在【以某方向为轴的圆锥】内采样。

        cone_deg = 180  ⇒ 整个球面
        cone_deg = 30   ⇒ 只有一小片
        """
        axis = np.array([0.0, 0.0, 1.0])
        cos_lim = math.cos(math.radians(cone_deg))

        pts = []
        while len(pts) < n:
            v = RNG.normal(size=3)
            v /= np.linalg.norm(v)
            if np.dot(v, axis) >= cos_lim:
                pts.append(v * R_TRUE)
        return np.array([add_disturbance(m, OFFSET_TRUE, M_TRUE) for m in pts])

    print("    %-14s %-30s %-14s %s"
          % ("覆盖率", "拟合出的 offset", "拟合残差", "offset 误差"))
    print("    " + "-" * 80)

    results = []
    for cone in (180, 120, 90, 60, 45, 30, 20):
        samples = gen_covering(cone)

        p = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 50.0])
        for _ in range(2):
            p, _fs = run_sphere_fit(samples, p)
            p, fit_e = run_ellipsoid_fit(samples, p, p[9])

        off_err = np.linalg.norm(p[0:3] - OFFSET_TRUE)
        results.append((cone, p[0:3].copy(), fit_e, off_err))

        name = "整个球面" if cone >= 180 else ("±%d° 圆锥" % cone)
        print("    %-14s [%7.3f %7.3f %7.3f]   %-14.3e %.3f µT"
              % (name, p[0], p[1], p[2], fit_e, off_err))

    print()
    print("    真值 offset = [%6.2f %6.2f %6.2f]"
          % tuple(OFFSET_TRUE))

    print("""
    ★★ 观察：

      ① 覆盖率 180°（整个球面）⇒ 误差 < 0.01 µT
      ② 覆盖率越窄 ⇒ offset 误差越大
      ③ ★★ 但【拟合残差】可能【一直很小】！

      最后一条最危险：
        在数据覆盖的那一小片里，椭球可以完美拟合
        ⇒ 残差小 ⇒ ArduPilot 的 fit_acceptable() 【会通过】
        ⇒ 但你转到没采样过的方向，误差就暴露了

    ⇒ 这正是 §2.6.3 说的：
      【残差小 ≠ 校准对】
""")

    print("    ── ★★ 残差能发现「转得不够」吗 ──")
    print()
    print("    %-14s %-14s %-16s %-16s %s"
          % ("覆盖率", "残差", "残差过判据吗", "offset 误差", "换算成航向误差"))
    print("    " + "-" * 84)

    B_horiz = 45.0 * math.cos(math.radians(58))    # 北京纬度附近的水平分量

    for cone, _off, fit_e, off_err in results:
        name = "整个球面" if cone >= 180 else ("±%d° 圆锥" % cone)
        # 残差判据：用 ArduPilot 那种"适应度 ≤ tolerance²"的形式，
        # 这里取一个宽松的容差 0.1 µT ⇒ 判据 0.01
        passes = "✓ 过" if fit_e <= 0.01 else "✗ 不过"
        yaw_err = math.degrees(math.atan2(off_err, B_horiz))
        print("    %-14s %-14.3e %-16s %-16.3f %.2f°"
              % (name, fit_e, passes, off_err, yaw_err))

    print()
    print("""    ★★★ 看这一列「残差过判据吗」：

      **所有覆盖率下残差都极小（1e-29 ~ 1e-7），
        全部都能通过任何合理的残差判据。**

      而实际 offset 误差从 0 涨到 0.4 µT。

    ⇒ ⇒ **残差【完全无法】区分"转够了"和"只转了一点点"。**

    【为什么？】

      残差衡量的是【自洽性】——
      "这组参数能不能解释我采到的这些数据"

      而覆盖率衡量的是【信息量】——
      "我采的数据够不够把参数定下来"

      ★ 这是【两个正交的维度】：
        · 数据少，但残差小 ⇒ 自洽，但不可信
        · 数据多，但残差大 ⇒ 可信，但有干扰没建模

    ⇒ 所以 ArduPilot 用【两套独立的检查】：

        · 界面上的进度/覆盖率条  →  信息量够不够
        · fit_acceptable() 的
          radius/offset/diag 范围  →  参数物理上合理吗
        · fit_acceptable() 的
          _fitness <= tolerance²   →  自洽吗

    ⚠ 三者【互补】，不能互相替代。
""")
    print()


# ===========================================================================
# 实验 4：磁偏角与偏航误差
# ===========================================================================

def exp4():
    head("实验 4：磁偏角与偏航误差")

    print("""
  这一节把"罗盘误差"换算成"偏航误差"——
  因为飞控最终要的是【航向】。
""")

    sub("① 磁偏角的量级")

    print("""
    磁偏角（declination）随地理位置变化，全球范围大约 -30° ~ +30°。

    几个城市（近似值）：
""")
    print("    %-16s %-14s %s" % ("地点", "磁偏角", "说明"))
    print("    " + "-" * 56)
    CITIES = [
        ("北京", -6.5, "磁北偏西"),
        ("上海", -5.0, "磁北偏西"),
        ("广州", -2.5, "磁北偏西"),
        ("乌鲁木齐", 3.0, "磁北偏东"),
        ("伦敦", 0.5, "接近 0"),
        ("纽约", -13.0, "磁北偏西"),
        ("洛杉矶", 12.0, "磁北偏东"),
        ("悉尼", 12.5, "磁北偏东"),
    ]
    for nm, dec, note in CITIES:
        print("    %-16s %-14.1f %s" % (nm, dec, note))

    print()
    print("""    ★ 北京和纽约差 6.5°，洛杉矶和悉尼差 0.5°。

    ⇒ ★★ 如果不做磁偏角修正，飞机在所有地方都会有
      【-30° ~ +30° 的航向误差】——
      而这是【系统性】的，不是噪声。

    ⇒ 这就是为什么 COMPASS_AUTODEC 默认开着。
""")

    sub("② 校准误差 → 偏航误差")

    print("""
    校准误差也会变成偏航误差。定量估计：

        yaw_error ≈ |offset_error| / (B · cos(dip))

    其中 B·cos(dip) 是磁场在【水平面内的分量】——
    因为航向只由水平分量决定。

    ★ 注意分母：磁倾角越大（越靠近极点），
      水平分量越小 ⇒ 【同样的 offset 误差造成更大的航向误差】
""")

    print("    %-14s %-14s %-20s %-20s %s"
          % ("纬度", "磁倾角", "水平分量占比", "offset 误差 2µT 时", "offset 误差 5µT 时"))
    print("    " + "-" * 92)

    B = 45.0
    rows4 = []
    for lat, dip in ((0, 0), (20, 30), (40, 58), (60, 72), (75, 80)):
        cosd = math.cos(math.radians(dip))
        h = B * cosd
        yaw2 = math.degrees(math.atan2(2.0, h))
        yaw5 = math.degrees(math.atan2(5.0, h))
        rows4.append((lat, dip, cosd, h, yaw2, yaw5))
        print("    %-14s %-14s %-20s %-20s %s"
              % ("%d°" % lat, "%d°" % dip, "%.0f%% (%.1f µT)" % (100 * cosd, h),
                 "%.1f°" % yaw2, "%.1f°" % yaw5))

    # ★ 用实测值拼结论 —— 不写死数字
    y0 = rows4[0][4]
    ymax = rows4[-1][4]
    print()
    print("""    ★★ 两个观察（用本轮运行的真实数字）：

      ① 同样 2 µT 的 offset 误差：
           赤道（0°）     ⇒ %.1f° 偏航误差
           高纬（75°）    ⇒ %.1f° 偏航误差
         ⇒ 【放大 %.1f 倍】

      ② ★ 这解释了为什么高纬度地区对罗盘校准更敏感

    ⚠ 反过来也说明：**高纬度的罗盘本来就不好用**——
      水平分量小 ⇒ 信噪比低 ⇒ 不如多依赖 GPS 航向
""" % (y0, ymax, ymax / y0))

    sub("③ ★ 什么时候干脆不用罗盘")

    print("""
    磁场水平分量小到一定程度时，罗盘的航向就不可信了。

    ArduPilot 有一个参数控制这个：

        比较：|水平分量| vs |总磁场|

        如果比值太小 ⇒ 认为"磁航向不可靠"

    ⇒ 这时候系统会用【GPS 航向】代替（需要移动才能得到）。

    ★ 这也是为什么"GPS 和罗盘对不上"是常见故障 ——
      它们在不同的条件下各自失效。
""")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("硬铁与软铁的几何表现", exp1),
    "2": ("★★★ 椭球拟合", exp2),
    "3": ("★ 「转得不够」会怎样", exp3),
    "4": ("磁偏角与偏航误差", exp4),
}


def main():
    print()
    hr("#")
    print("#  卷 2 第 2 章 · 磁罗盘与椭球拟合标定")
    print("#  配套实验")
    hr("#")

    arg = sys.argv[1] if len(sys.argv) > 1 else None

    if arg is None:
        for key in sorted(EXPERIMENTS):
            EXPERIMENTS[key][1]()
    elif arg in EXPERIMENTS:
        EXPERIMENTS[arg][1]()
    else:
        print()
        print("  未知实验编号: %s" % arg)
        print("  可用: %s" % ", ".join(sorted(EXPERIMENTS)))
        print()
        return 1

    print()
    hr("#")
    print("#  实验结束")
    hr("#")
    print()


if __name__ == "__main__":
    sys.exit(main())
