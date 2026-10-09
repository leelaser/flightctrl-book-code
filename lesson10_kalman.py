#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 2 第 4 章 · 卡尔曼滤波从零推导 —— 配套实验

    实验 1  一维 KF 手算 + 脚本对照（把推导过程逐步打印）
    实验 2  ★★ 二维目标跟踪（位置 + 速度）：看 P 怎么收缩
    实验 3  ★ Q 与 R 的影响：调大调小分别意味着什么
    实验 4  新息诊断：模型搞错时，新息会怎么「报警」

用法：
    python lesson10_kalman.py 1
    python lesson10_kalman.py 2
    python lesson10_kalman.py 3
    python lesson10_kalman.py 4
    python lesson10_kalman.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  和上一课（lesson04_probability.py）一样：**能推的就不背，能算的就不猜。**

  上一课证明了「两个高斯相乘 ⇒ 卡尔曼增益」；
  这一课要证明的是它的【递推版本】：

      预测： x⁻ = F x̂            P⁻ = F P Fᵀ + Q
      更新： K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹  x̂ = x⁻ + K(z-Hx⁻)  P = (I-KH)P⁻

  这三个实验分别从三个角度验证它「不是拍脑袋定的」：

      实验 1  手算 = 脚本                     （公式自洽）
      实验 2  P 自动收敛到稳态                 （Riccati 迭代）
      实验 1③ KF 递推 = 批量加权最小二乘        （和经典方法等价）
      实验 4  新息必须是白噪声                 （模型正确性的可检验推论）

参考：
    EKF/Kalman_and_Bayesian_Filters_in_Python.pdf   第 6~8 章
    EKF/GNSS与惯性及多传感器组合导航系统原理(第二版).pdf  第 4 章
    Probabilistic-Robotics-cn.pdf                   第 3 章
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


def fmt_mat(M, name, prec=6):
    """把一个矩阵按缩进的多行形式打印出来。"""
    M = np.atleast_2d(M)
    print("      %s =" % name)
    for i in range(M.shape[0]):
        body = "  ".join("%*.*f" % (prec + 4, prec, M[i, j])
                         for j in range(M.shape[1]))
        print("        [ %s ]" % body)


# ---------------------------------------------------------------------------
# 卡尔曼滤波器的标准实现（本章的主角）
# ---------------------------------------------------------------------------

class LinearKF:
    """最朴素的线性卡尔曼滤波器。

    约定（和课文 §10.1.5 一致，也是 PX4 的约定）：
        预测：  x⁻ = F x̂            P⁻ = F P Fᵀ + Q
        新息：  y  = z - H x⁻        S  = H P⁻ Hᵀ + R
        增益：  K  = P⁻ Hᵀ S⁻¹
        更新：  x̂  = x⁻ + K y        P  = (I - K H) P⁻

    ★ 注意新息的符号：这里用 y = z - Hx⁻（观测 减 预测）。
      PX4 的 ekf2 也是这个约定（velocity_fusion.cpp:43），
      但 ArduPilot 的 EKF3 用的是反的（innov = 状态 - 观测），
      所以 AP 那边状态更新是【减】 K·innov（PosVelFusion.cpp:1079）。
      两者数学等价，但读代码时看错符号会得出完全相反的结论。
    """

    def __init__(self, x0, P0, F, H, Q, R):
        self.x = np.asarray(x0, dtype=float).reshape(-1, 1)
        self.P = np.array(P0, dtype=float)
        self.F = np.array(F, dtype=float)
        self.H = np.array(H, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.n = self.x.shape[0]
        self.I = np.eye(self.n)
        # 只读日志（predict/update 里填）
        self.log = []

    # ---- 预测步 ----------------------------------------------------------
    def predict(self):
        self.x = self.F @ self.x                      # x⁻ = F x̂
        self.P = self.F @ self.P @ self.F.T + self.Q  # P⁻ = F P Fᵀ + Q

    # ---- 更新步 ----------------------------------------------------------
    def update(self, z):
        z = np.asarray(z, dtype=float).reshape(-1, 1)
        y = z - self.H @ self.x                       # 新息
        S = self.H @ self.P @ self.H.T + self.R       # 新息协方差
        K = self.P @ self.H.T @ np.linalg.inv(S)      # 增益
        self.x = self.x + K @ y                       # 状态更新
        self.P = (self.I - K @ self.H) @ self.P       # 协方差更新
        return y, S, K

    # ---- 一步走完并记录 --------------------------------------------------
    def step(self, z, tag=""):
        x_pre = self.x.copy()
        P_pre = self.P.copy()
        self.predict()
        x_pred, P_pred = self.x.copy(), self.P.copy()
        y, S, K = self.update(z)
        self.log.append(dict(tag=tag, x_pre=x_pre, P_pre=P_pre,
                             x_pred=x_pred, P_pred=P_pred,
                             y=y, S=S, K=K,
                             x_post=self.x.copy(), P_post=self.P.copy()))
        return self.log[-1]


def steady_state_P(F, H, Q, R, n_iter=2000):
    """把 Riccati 递推迭代到收敛，得到稳态 P。

    ★ 注意：这个迭代【不含测量数据】—— 只用 F/H/Q/R。
      这正是 KF 最重要的一条性质：**增益的收敛只取决于模型和噪声参数，
      和数据无关。** 所以工程上可以离线把稳态增益算好，在线只做加减乘。
    """
    P = np.eye(F.shape[0]) * 1e6
    I = np.eye(F.shape[0])
    for _ in range(n_iter):
        P = F @ P @ F.T + Q
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        P = (I - K @ H) @ P
    return P, K


def nis_test(innovs, variances, gate=3.0):
    """归一化新息平方（NIS）检验。

    test_ratio = y² / (gate² · S)   ← 和 PX4 ekf.h:1188 的定义完全一致
    test_ratio > 1 ⇒ 这一次观测被判为「不可信」（弃用）
    """
    innovs = np.asarray(innovs, dtype=float).ravel()
    variances = np.asarray(variances, dtype=float).ravel()
    ratio = innovs ** 2 / (gate ** 2 * variances)
    return ratio, ratio > 1.0


def autocorr_lag1(seq):
    """滞后 1 步的自相关系数。白噪声应当 ≈ 0。"""
    x = np.asarray(seq, dtype=float).ravel()
    x = x - x.mean()
    denom = float(np.sum(x * x))
    if denom <= 0:
        return 0.0
    return float(np.sum(x[1:] * x[:-1]) / denom)


# ---------------------------------------------------------------------------
# 实验 1：一维 KF 手算 + 脚本对照
# ---------------------------------------------------------------------------

def exp1():
    head("实验 1  一维 KF 手算 + 脚本对照")

    print("""
  课文 §10.2~§10.4 推出来的五个方程，这个实验要把它【算出来】：

      预测：  x⁻ = F x̂            P⁻ = F P Fᵀ + Q
      新息：  y  = z - H x⁻        S  = H P⁻ Hᵀ + R
      增益：  K  = P⁻ Hᵀ S⁻¹
      更新：  x̂  = x⁻ + K y        P  = (I - K H) P⁻

  分四小步：
      ① 最简标量情形：F=1, H=1 —— 每一步都手算一遍
      ② 第一个时间步【完整展开】，和 ① 的数字对拍
      ③ ★ 静态参数下 KF 递推 == 批量加权最小二乘（证明它不神秘）
      ④ 二维（位置 + 速度）的【一步】矩阵手算，看矩阵长什么样
""")

    # ------------------------------------------------------------------
    sub("① 最简标量：F = 1, H = 1 —— 一个静止参数 + 逐次观测")

    F = np.array([[1.0]])
    H = np.array([[1.0]])
    Q = np.array([[0.01]])
    R = np.array([[1.00]])
    x0 = np.array([0.0])
    P0 = np.array([[10.0]])

    zs = [1.2, 1.9, 3.1, 3.9, 5.2, 6.1]

    print("    设定：")
    print("      F = 1（状态不变：x_k = x_{k-1}）")
    print("      H = 1（直接观测状态）")
    print("      Q = %.2f（过程噪声：状态其实会缓慢漂移）" % Q[0, 0])
    print("      R = %.2f（观测噪声方差，σ_z = %.1f）" % (R[0, 0], math.sqrt(R[0, 0])))
    print("      x̂₀ = %.1f,  P₀ = %.1f" % (x0[0], P0[0, 0]))
    print()

    kf = LinearKF(x0, P0, F, H, Q, R)

    rows = []
    for k, z in enumerate(zs, 1):
        rec = kf.step([z], tag="k=%d" % k)
        rows.append([
            k, "%.2f" % z,
            "%.6f" % rec["x_pred"][0, 0], "%.6f" % rec["P_pred"][0, 0],
            "%.6f" % rec["y"][0, 0],
            "%.6f" % rec["K"][0, 0],
            "%.6f" % rec["x_post"][0, 0], "%.6f" % rec["P_post"][0, 0],
        ])
    tab(rows, ["k", "z", "x⁻", "P⁻", "新息 y", "K", "x̂", "P"])

    print("""    ★ 看两列：
        P⁻  每步都【变大】（+Q，因为预测引入了新的不确定）
        P   每步又【变小】（观测带来了信息）
      ⇒ 这就是 KF 的心跳：预测让不确定性长大，更新让它缩小。
        两者的平衡点就是稳态 P（后续实验会看到它收敛）。

      ★ 再看 K 那一列：从 0.91 一路降到 0.21。
        一开始 P 很大（我不确定）⇒ 信观测；
        后来 P 收敛得很小（我确定了）⇒ 信自己。
        **这就是「一开始信观测、后来信模型」的完整过程。**
""")

    # ------------------------------------------------------------------
    sub("② 第一个时间步：完整展开，和 ① 对拍")

    x_prior, P_prior = x0.reshape(-1, 1).copy(), P0.copy()
    print("    初始（先验）：x̂₀ = %.10f,  P₀ = %.10f"
          % (x_prior[0, 0], P_prior[0, 0]))
    print()

    print("    【预测步】——课文 §10.2")
    x_pred = F @ x_prior
    P_pred = F @ P_prior @ F.T + Q
    print("      x⁻ = F·x̂ = %.10f × %.10f = %.10f"
          % (F[0, 0], x_prior[0, 0], x_pred[0, 0]))
    print("      P⁻ = F·P·Fᵀ + Q")
    print("         = %.10f × %.10f × %.10f + %.10f" % (F[0, 0], P_prior[0, 0], F[0, 0], Q[0, 0]))
    print("         = %.10f + %.10f = %.10f"
          % (F[0, 0] * P_prior[0, 0] * F[0, 0], Q[0, 0], P_pred[0, 0]))
    print()

    z1 = zs[0]
    print("    【更新步】——课文 §10.3（z₁ = %.4f）" % z1)
    y = z1 - H @ x_pred
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_post = x_pred + K @ y
    P_post = (np.eye(1) - K @ H) @ P_pred

    print("      新息   y = z - H·x⁻ = %.4f - %.10f = %.10f" % (z1, x_pred[0, 0], y[0, 0]))
    print("      新息方差 S = H·P⁻·Hᵀ + R = %.10f + %.10f = %.10f"
          % (P_pred[0, 0], R[0, 0], S[0, 0]))
    print("      增益   K = P⁻Hᵀ·S⁻¹ = %.10f / %.10f = %.10f"
          % (P_pred[0, 0], S[0, 0], K[0, 0]))
    print("      状态   x̂ = x⁻ + K·y = %.10f + %.10f×%.10f" % (x_pred[0, 0], K[0, 0], y[0, 0]))
    print("                 = %.10f" % x_post[0, 0])
    print("      协方差 P = (I-KH)·P⁻ = (1 - %.10f) × %.10f = %.10f"
          % (K[0, 0], P_pred[0, 0], P_post[0, 0]))
    print()

    print("    ① 里脚本跑出来的第一行：")
    print("        x⁻ = %.10f   P⁻ = %.10f" % (kf.log[0]["x_pred"][0, 0], kf.log[0]["P_pred"][0, 0]))
    print("        K  = %.10f   x̂  = %.10f   P = %.10f"
          % (kf.log[0]["K"][0, 0], kf.log[0]["x_post"][0, 0], kf.log[0]["P_post"][0, 0]))
    ok = (abs(x_pred[0, 0] - kf.log[0]["x_pred"][0, 0]) < 1e-12
          and abs(P_pred[0, 0] - kf.log[0]["P_pred"][0, 0]) < 1e-12
          and abs(K[0, 0] - kf.log[0]["K"][0, 0]) < 1e-12)
    print("    %s 手算和脚本一致。" % ("✅" if ok else "❌"))
    print()

    # ------------------------------------------------------------------
    sub("③ ★ KF 递推 == 批量加权最小二乘（Q = 0 的静态参数情形）")

    print("""    如果状态【根本不动】（F = 1）、模型也【不承认它会漂】（Q = 0），
    那么 KF 应该退化成什么？

      直觉：观测是一堆带噪声的同一个数的读数 ⇒ 最优估计就是【加权平均】。
      而且有先验时还要把先验也算进去。

    批量加权最小二乘（WLS）的闭式解：
        x̂ = ( Σ₀⁻¹·μ₀ + Σ zᵢ/R ) / ( Σ₀⁻¹ + n/R )
        P = 1 / ( Σ₀⁻¹ + n/R )
""")

    Q0 = np.array([[0.0]])
    R0 = np.array([[1.0]])
    P00 = np.array([[10.0]])
    mu0 = 0.0
    zs2 = [1.2, 1.9, 3.1, 3.9, 5.2, 6.1]
    n = len(zs2)

    kf2 = LinearKF(np.array([mu0]), P00, F, H, Q0, R0)
    for z in zs2:
        kf2.step([z])

    info_prior = 1.0 / P00[0, 0]
    info_obs = n / R0[0, 0]
    x_wls = (info_prior * mu0 + sum(zs2) / R0[0, 0]) / (info_prior + info_obs)
    P_wls = 1.0 / (info_prior + info_obs)

    print("    先验信息量  Σ₀⁻¹        = 1/%.1f = %.6f" % (P00[0, 0], info_prior))
    print("    %d 次观测信息量 n/R      = %d/%.1f = %.6f" % (n, n, R0[0, 0], info_obs))
    print("    后验信息量（相加）        = %.6f" % (info_prior + info_obs))
    print()

    rows = [
        ["KF 递推 %d 步" % n, "%.12f" % kf2.x[0, 0], "%.12f" % kf2.P[0, 0]],
        ["批量 WLS 闭式解", "%.12f" % x_wls, "%.12f" % P_wls],
        ["简单平均", "%.12f" % (sum(zs2) / n), "—"],
    ]
    tab(rows, ["方法", "估计 x̂", "方差 P"])

    d_x = abs(kf2.x[0, 0] - x_wls)
    d_P = abs(kf2.P[0, 0] - P_wls)
    print("    偏差：Δx̂ = %.2e,  ΔP = %.2e" % (d_x, d_P))
    if d_x < 1e-9 and d_P < 1e-9:
        print("    ✅ 完全一致 —— KF 就是【递推的】加权最小二乘。")
    else:
        print("    ⚠️ 不一致，检查 Q 是否真的为 0。")
    print()
    print("""    ★ 注意「简单平均」那一行：它【不】等于 WLS 的结果 ——
      因为先验 x̂₀ = 0 是一个【真的信息】（精度 1/P₀ = 0.1），
      它把估计往 0 拽了一点。

      ⇒ **KF 没有「丢掉历史」——它把历史压缩进了 (x̂, P) 两个数里。**
        这就是为什么 KF 不需要存下所有观测（节省内存/算力），
        也是为什么 P 必须【诚实】：P 错了，历史就被记错了。

    ★ 这一步同时说明：RLS（递推最小二乘）= KF 在 F=1, Q=0 时的特例
      —— 详见课文 §10.9。
""")

    # ------------------------------------------------------------------
    sub("④ 二维（位置 + 速度）的【一步】矩阵手算")

    dt = 1.0
    Fm = np.array([[1.0, dt],
                   [0.0, 1.0]])
    Hm = np.array([[1.0, 0.0]])
    sa = 0.5
    Qm = sa ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                             [dt ** 3 / 2, dt ** 2]])
    Rm = np.array([[9.0]])
    xm = np.array([[0.0], [0.0]])
    Pm = np.array([[100.0, 0.0],
                   [0.0, 25.0]])

    print("    匀速直线运动（CV）模型，状态 = [位置, 速度]ᵀ：")
    print()
    fmt_mat(Fm, "F")
    fmt_mat(Hm, "H")
    print("      Q = σ_a² · [[dt⁴/4, dt³/2], [dt³/2, dt²]]，取 σ_a = %.2f, dt = %.1f：" % (sa, dt))
    print("      （Q 的推导见课文 §10.6.4，这里先用）")
    fmt_mat(Qm, "Q")
    fmt_mat(Rm, "R")
    fmt_mat(xm, "x̂₀")
    fmt_mat(Pm, "P₀")
    print()
    print("    ⇒ 读一下 P₀ 的含义：")
    print("        位置方差 %.0f ⇒ σ_pos = %.2f m（完全不知道在哪）" % (Pm[0, 0], math.sqrt(Pm[0, 0])))
    print("        速度方差 %.0f ⇒ σ_vel = %.2f m/s" % (Pm[1, 1], math.sqrt(Pm[1, 1])))
    print("        非对角元 0 ⇒ 初始时位置和速度【认为不相关】")
    print()

    kf3 = LinearKF(xm, Pm, Fm, Hm, Qm, Rm)
    z_obs = 5.0
    rec = kf3.step([z_obs])

    print("    【预测步】x⁻ = F·x̂₀")
    fmt_mat(rec["x_pred"], "x⁻")
    print("    【预测步】P⁻ = F·P₀·Fᵀ + Q")
    fmt_mat(rec["P_pred"], "P⁻")
    print("      ★ 注意 P⁻ 的【非对角元 = %.4f】不再为 0！（P₀ 里它是 0）"
          % rec["P_pred"][0, 1])
    print("        为什么？位置 = 位置 + 速度×dt ⇒ 速度错一点，位置就跟着错")
    print("        ⇒ **相关是模型「制造」出来的**（课文 §10.2.3 会推）。")
    print()
    print("    【更新步】z = %.1f, R = %.1f" % (z_obs, Rm[0, 0]))
    print("      新息   y = z - H·x⁻ = %.6f" % rec["y"][0, 0])
    print("      S = H·P⁻·Hᵀ + R = %.6f + %.1f = %.6f"
          % (rec["P_pred"][0, 0], Rm[0, 0], rec["S"][0, 0]))
    fmt_mat(rec["K"], "K")
    print("      ★ K 是【2×1】的：位置和速度都要按各自的增益被修正。")
    print("        位置增益 %.6f > 速度增益 %.6f —— 为什么？" % (rec["K"][0, 0], rec["K"][1, 0]))
    print("        因为观测【直接】测位置，对速度只有【间接】信息。")
    print()
    fmt_mat(rec["x_post"], "x̂")
    fmt_mat(rec["P_post"], "P")
    print("      ★ P 收缩了：位置方差 %.4f → %.4f" % (Pm[0, 0], rec["P_post"][0, 0]))
    print("        而且**速度方差也被修正了**（%.4f → %.4f）——" % (Pm[1, 1], rec["P_post"][1, 1]))
    print("        明明没有测速度，为什么速度也变准了？")
    print("        因为位置和速度【相关】（P⁻ 的非对角元 = %.4f），" % rec["P_pred"][0, 1])
    print("        知道「位置比预想的近」⇒ 推断出「速度大概也偏小了」。")
    print("        ⇒ **这就是协方差非对角元在工程上的全部价值。**")
    print()


# ---------------------------------------------------------------------------
# 实验 2：★★ 二维目标跟踪（位置 + 速度）
# ---------------------------------------------------------------------------

def exp2():
    head("实验 2  ★★ 二维目标跟踪（位置 + 速度）：看 P 怎么收缩")

    print("""
  场景：一个目标做匀速直线运动，我们每秒用一个带噪声的传感器测它的【位置】。

     状态  x = [位置 p, 速度 v]ᵀ          ← 速度是【看不见】的，必须估出来
     模型  p_k = p_{k-1} + v_{k-1}·dt
           v_k = v_{k-1}                ← 匀速假设（这就是模型的全部内容）
     观测  z_k = p_k + 噪声,  σ_z = 3 m

  这个实验要看三件事：
      ① P 怎么从很大【收缩】到稳态（Riccati 迭代）
      ② 估计误差是否落在 ±3σ 里（一致性检验 —— 不落进去就说明 P 是错的）
      ③ P 的非对角元（位置-速度相关）长什么样
""")

    dt = 1.0
    sigma_a = 0.2            # 加速度噪声标准差（模型没建的「抖动」）
    sigma_z = 3.0            # 位置观测噪声标准差
    N = 100

    F = np.array([[1.0, dt], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    Q = sigma_a ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                                 [dt ** 3 / 2, dt ** 2]])
    R = np.array([[sigma_z ** 2]])

    sub("① 参数设定")

    rows = [
        ["dt", "%.3f s" % dt, "采样周期"],
        ["σ_a", "%.3f m/s²" % sigma_a, "过程噪声：模型的「加速度没建」有多大"],
        ["σ_z", "%.3f m" % sigma_z, "观测噪声：位置传感器有多准"],
        ["Q(dt)", "%.4f m²" % Q[0, 0], "一个 dt 内位置累积的过程方差"],
        ["R", "%.3f m²" % R[0, 0], "观测噪声方差"],
        ["√(Q₀₀/R)", "%.4f" % math.sqrt(Q[0, 0] / R[0, 0]), "Q/R 的比值 —— 决定增益"],
    ]
    tab(rows, ["符号", "值", "含义"])

    # ---- 真值 + 观测生成（固定种子，保证可复现） ----
    rng = np.random.default_rng(20240610)
    p_true = np.zeros(N + 1)
    v_true = np.zeros(N + 1)
    p_true[0], v_true[0] = 0.0, 2.0
    for k in range(1, N + 1):
        a_k = rng.normal(0.0, sigma_a)         # 真实世界有微小的加速度扰动
        v_true[k] = v_true[k - 1] + a_k * dt
        p_true[k] = p_true[k - 1] + v_true[k - 1] * dt + 0.5 * a_k * dt * dt

    z_meas = p_true[1:] + rng.normal(0.0, sigma_z, N)

    sub("② 跑 KF，每 10 步看一眼")

    kf = LinearKF(np.array([[0.0], [0.0]]),
                  np.array([[100.0, 0.0], [0.0, 25.0]]),
                  F, H, Q, R)

    rows = []
    for k in range(1, N + 1):
        rec = kf.step([z_meas[k - 1]])
        if k % 10 == 0 or k == 1:
            rows.append([
                k,
                "%.3f" % z_meas[k - 1],
                "%.4f" % rec["x_post"][0, 0],
                "%.4f" % rec["x_post"][1, 0],
                "%.4f" % rec["P_post"][0, 0],
                "%.6f" % rec["P_post"][0, 1],
                "%.4f" % rec["P_post"][1, 1],
                "%.4f" % rec["K"][0, 0],
                "%.4f" % rec["K"][1, 0],
            ])
    tab(rows, ["k", "z", "p̂", "v̂", "P₀₀", "P₀₁", "P₁₁", "K₀", "K₁"])

    P_inf_early, K_inf_early = steady_state_P(F, H, Q, R)
    p_first = kf.log[0]["P_post"]
    p_last = kf.log[-1]["P_post"]
    print("""    ★ 逐列读：

      P₀₀（位置方差）  %.4f → %.4f → …… → %.4f（稳态 %.4f）
          初始「完全不知道在哪」(σ = %.1f m)，几十步后变成 σ ≈ %.2f m。
          注意它【不会】降到远低于 σ_z = %.1f m —— 观测本身就只准到那里。

      P₁₁（速度方差）  %.4f → …… → %.4f（稳态 %.4f）
          ★ 速度【从来没有被直接观测过】，但它的方差也塌下去了：
          σ_v 从 %.2f m/s 降到 %.3f m/s —— 收敛了 %.0f 倍。
          这是「位置-速度相关 + 运动模型」共同作用的结果。
          **这是 KF 最像魔法、也最容易被误解的一条性质。**

      P₀₁（协方差）    0 → %.4f（稳态 %.4f），【正】相关
          读法：位置估高了 ⇒ 往往意味着速度也估高了
          （因为位置是速度积分出来的，速度偏大会同时抬高两者）。
          ⇒ 所以观测说「位置没那么远」时，滤波器会【同时】把速度往下调，
            这就是 K₁ > 0 的来源。**相关是「间接观测」的桥梁。**

      K₀ / K₁        位置增益 %.4f → %.4f，速度增益 %.4f → %.4f
          两个增益都随 P 收缩而下降，最后停在稳态值上不动了。
          从第 30 步起 K 就不再变化 —— 这就是下面 ③ 要说的「稳态」。
          **物理意义：滤波器「学完了」，之后就是一个固定系数的线性系统。**
"""
          % (p_first[0, 0], kf.log[4]["P_post"][0, 0], p_last[0, 0], P_inf_early[0, 0],
             math.sqrt(100.0), math.sqrt(p_last[0, 0]), sigma_z,
             25.0, p_last[1, 1], P_inf_early[1, 1],
             math.sqrt(25.0), math.sqrt(p_last[1, 1]),
             math.sqrt(25.0) / math.sqrt(p_last[1, 1]),
             p_last[0, 1], P_inf_early[0, 1],
             kf.log[0]["K"][0, 0], kf.log[-1]["K"][0, 0],
             kf.log[0]["K"][1, 0], kf.log[-1]["K"][1, 0]))

    # ---- 稳态解 ----
    sub("③ Riccati 迭代：稳态 P 和稳态增益（★ 与数据无关）")

    P_inf, K_inf = steady_state_P(F, H, Q, R)
    fmt_mat(P_inf, "P_∞")
    fmt_mat(K_inf, "K_∞")
    print("      稳态位置标准差 √P₀₀ = %.6f m" % math.sqrt(P_inf[0, 0]))
    print("      稳态速度标准差 √P₁₁ = %.6f m/s" % math.sqrt(P_inf[1, 1]))
    print("      相关系数 ρ = P₀₁/√(P₀₀·P₁₁) = %.6f"
          % (P_inf[0, 1] / math.sqrt(P_inf[0, 0] * P_inf[1, 1])))
    print()
    print("      第 %d 步的 P（上面表里最后一行）与稳态 P 的差：" % N)
    print("        |P₀₀ - P₀₀∞| = %.2e" % abs(kf.P[0, 0] - P_inf[0, 0]))
    print()
    print("    ★★ 这里有一条非常重要的工程结论：")
    print("      **稳态增益只取决于 (F, H, Q, R)，和测量数据无关。**")
    print("      所以工程上可以【离线】把 K_∞ 算好、烧进代码，")
    print("      在线只做 x̂ = x⁻ + K(z - Hx⁻) 和 P⁻ = FPFᵀ+Q ——")
    print("      省掉了一次矩阵求逆（嵌入式上求逆很贵）。")
    print("      这叫 **定常卡尔曼滤波**（steady-state KF），")
    print("      PX4 的 landing_target_estimator 和很多工业实现都这么干。")
    print()

    # ---- 一致性检验 ----
    sub("④ 一致性检验：误差落在 ±3σ 里吗？（P 是不是在撒谎）")

    err_p, err_v, sig_p, sig_v = [], [], [], []
    for k, rec in enumerate(kf.log, 1):
        tp = p_true[k]
        # 速度真值用有限差分近似（真实速度是仿真里存着的，直接用）
        err_p.append(rec["x_post"][0, 0] - tp)
        err_v.append(rec["x_post"][1, 0] - v_true[k])
        sig_p.append(math.sqrt(rec["P_post"][0, 0]))
        sig_v.append(math.sqrt(rec["P_post"][1, 1]))

    err_p = np.array(err_p)
    err_v = np.array(err_v)
    sig_p = np.array(sig_p)
    sig_v = np.array(sig_v)

    n_out_p = int(np.sum(np.abs(err_p) > 3 * sig_p))
    n_out_v = int(np.sum(np.abs(err_v) > 3 * sig_v))

    rows = [
        ["位置 p", "%.4f m" % err_p.mean(), "%.4f m" % err_p.std(),
         "%.4f m" % sig_p[-1], "%d / %d" % (n_out_p, N)],
        ["速度 v", "%.4f m/s" % err_v.mean(), "%.4f m/s" % err_v.std(),
         "%.4f m/s" % sig_v[-1], "%d / %d" % (n_out_v, N)],
    ]
    tab(rows, ["状态", "误差均值", "误差标准差", "末步 √P", "超 3σ 次数"])

    print("""    ★ 判读方法：
        · 误差均值 ≈ 0            ⇒ 无偏（滤波器没有系统性偏差）
        · 误差标准差 ≈ √P          ⇒ ★★ **P 是诚实的**（没有过度自信/过度谦虚）
        · 超 3σ 次数 ≈ 0           ⇒ 和理论一致（%d 次里期望约 %.1f 次）
                                      （3σ 区间外本身就有 0.27%% 的概率）

      **「误差标准差 ≈ √P」这一条，是整个卡尔曼滤波工程实践的试金石。**
      如果实测误差远大于 √P，那说明模型错了或 Q/R 设小了，
      滤波器在 **撒谎** —— 后面它会越来越自信、越来越不听传感器，
      最终发散。卷 2 第 8 章 PX4 的 EKF2 有大量代码专门盯着这件事。
""" % (N, 0.0027 * N))

    # ---- 新息统计 ----
    sub("⑤ 新息的统计性质（模型对不对，看这里）")

    innovs = np.array([r["y"][0, 0] for r in kf.log])
    svars = np.array([r["S"][0, 0] for r in kf.log])
    ratio, rejected = nis_test(innovs, svars, gate=3.0)

    rows = [
        ["新息均值", "%.6f" % innovs.mean(), "≈ 0（模型正确时无偏）"],
        ["新息标准差", "%.6f" % innovs.std(), "%.6f（理论 √S 的均值）" % math.sqrt(svars.mean())],
        ["滞后 1 自相关", "%.6f" % autocorr_lag1(innovs), "≈ 0（必须是白的）"],
        ["平均归一化 NIS", "%.6f" % (np.mean(innovs ** 2 / svars)), "≈ 1（χ² 自由度 1）"],
        ["被 3σ 门限拒绝", "%d / %d" % (int(rejected.sum()), N), "期望 ≈ 0"],
    ]
    tab(rows, ["统计量", "实测", "理论期望"])

    print("""    ★ 「自相关 ≈ 0」和「NIS ≈ 1」这两条不是装饰：
      它们是【模型正确性】的可检验推论。下一小节的实验 4 会故意
      把模型搞错，看这两条怎么同时崩掉。
""")

    # ---- 练习 2：两个情景 ----
    sub("⑥ 练习 2：把 Q 缩小 —— 真实机动跟不跟着变小？")

    print("""
  这个问题有一个【前提陷阱】：「把 σ_a 从 0.2 改成 0.05」听起来像一个改动，
  但本函数里同一个 sigma_a 同时管两件事：

      sigma_a → Q = sigma_a²·[...]        滤波器的过程噪声（:465）
      sigma_a → rng.normal(0, sigma_a)    真实世界的加速度扰动（:487）

  所以「脚本原样改一个数」测的是【情景 A：真值和模型同步变简单】，
  而练习想问的是【情景 B：真实机动没变小，只是「我以为」它小了】。

  下面把两个情景都跑出来（同一个种子 20240610，N = %d，
  真值轨迹在情景 B 里【完全一样】，只有滤波器以为的 Q 在变）：
""" % N)

    def run_case(sigma_a_model, sigma_a_true, seed=20240610):
        """按给定的（滤波器以为的 σ_a，真值 σ_a）跑一遍，返回统计量。"""
        Qc = sigma_a_model ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                                            [dt ** 3 / 2, dt ** 2]])
        rc = np.random.default_rng(seed)
        pc = np.zeros(N + 1)
        vc = np.zeros(N + 1)
        pc[0], vc[0] = 0.0, 2.0
        for k in range(1, N + 1):
            a = rc.normal(0.0, sigma_a_true)
            vc[k] = vc[k - 1] + a * dt
            pc[k] = pc[k - 1] + vc[k - 1] * dt + 0.5 * a * dt * dt
        zc = pc[1:] + rc.normal(0.0, sigma_z, N)
        kc = LinearKF(np.array([[0.0], [0.0]]),
                      np.array([[100.0, 0.0], [0.0, 25.0]]),
                      F, H, Qc, R)
        ep, yv, sv = [], [], []
        for k in range(1, N + 1):
            d = kc.step([zc[k - 1]])
            ep.append(d["x_post"][0, 0] - pc[k])
            yv.append(d["y"][0, 0])
            sv.append(d["S"][0, 0])
        ep = np.array(ep)
        yv = np.array(yv)
        sv = np.array(sv)
        Pc, _ = steady_state_P(F, H, Qc, R)
        return (math.sqrt((ep ** 2).mean()), float(np.mean(yv ** 2 / sv)),
                autocorr_lag1(yv), Pc)

    rows = []
    for tag, sm, st in (("A 同步变小", 0.05, 0.05),
                        ("A 基准（匹配）", 0.20, 0.20),
                        ("A 同步变大", 1.00, 1.00),
                        ("B 只缩 Q", 0.05, 0.20),
                        ("B 基准（匹配）", 0.20, 0.20),
                        ("B 只放 Q", 1.00, 0.20)):
        rmse, nis, ac, Pc = run_case(sm, st)
        rows.append([tag, "%.2f" % sm, "%.2f" % st, "%.4f" % Pc[0, 0],
                     "%.4f" % rmse, "%.4f" % nis, "%+.4f" % ac])
    tab(rows, ["情景", "模型 σ_a", "真值 σ_a", "P∞₀₀", "位置 RMSE", "平均 NIS", "自相关"])

    print("""    ★★ 读这张表（本章最该带走的一张）：

        情景 A（模型和真值同步变）—— 「脚本原样改一个数」
            σ_a 0.20 → 0.05：NIS 0.8669 → %s（略降），自相关 -0.1042 → -0.1671。
            **y 和 S 一起缩，分子分母同时变小，比值不动** —— 这正是「自洽」的表现。
            ⚠ RMSE 从 1.4151 降到 1.0179 **不能**算作「滤波器变好了」：
              **目标本身变平滑了**（真值用的还是同一个 sigma_a），跟踪当然更容易。

        情景 B（真值不动，只改滤波器【以为】的 Q）★ 练习想问的就是这个
            σ_a 0.20 → 0.05：NIS 0.8669 → %s（★ 变大！），自相关 → +0.2041。
            **分母 S 变小，而新息 y 的实际幅度不变** ⇒ NIS 变大 ⇒ 过度自信。
            σ_a 0.20 → 1.00：NIS → %s（变小），自相关 → -0.2956 ⇒ 过度保守。

      ⇒ ★★ 结论：**「把 Q 调小」本身不会让 NIS 变小 —— 在真值不变的前提下它让 NIS 变大。**
         NIS 是「过度自信的探测器」：它比较的是「我以为新息该有多大」
         和「新息实际有多大」。Q 说小 = 把自己的不确定度说小 = NIS 变大。

      ⇒ ★★ 这同时也是「Q/R 等比不变性」的【边界】（§10.6.5）：
         只有 Q 和 R 【同时】缩放，K 才严格不变；
         **只缩一个，等于在偷偷改标定** —— 这就是情景 B 与 A 的分水岭。
""" % (rows[0][5], rows[3][5], rows[5][5]))

    if _HAS_MPL:
        _plot_tracking(kf, p_true, v_true, z_meas, sig_p, sig_v)
    else:
        print("    （没装 matplotlib，跳过画图）")
        print()


# ---------------------------------------------------------------------------
# 实验 3：★ Q 与 R 的影响
# ---------------------------------------------------------------------------

def exp3():
    head("实验 3  ★ Q 与 R：调大调小分别意味着什么")

    print("""
  课文 §10.6 说：
      R ——「传感器有多不准」，来自手册 / Allan 方差，**可以测**
      Q ——「模型有多不准」，来自「模型没建的东西」，**只能猜**

  但工程上真正决定性能的不是 Q 和 R 各自多大，而是【比值 Q/R】。

  这个实验：用【同一段测量数据】，扫一遍 Q 和 R 的缩放倍数，
  看稳态增益、稳态 P、以及实际 RMSE 怎么变。
""")

    dt = 1.0
    F = np.array([[1.0, dt], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    Q_base = 0.5 ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                                  [dt ** 3 / 2, dt ** 2]])
    R_base = np.array([[9.0]])

    # 真值 + 观测（和实验 2 同一套参数，但换种子避免完全重复）
    rng = np.random.default_rng(31415)
    N = 60
    p_true = np.zeros(N + 1)
    v_true = np.zeros(N + 1)
    p_true[0], v_true[0] = 0.0, 2.0
    for k in range(1, N + 1):
        a_k = rng.normal(0.0, 0.5)
        v_true[k] = v_true[k - 1] + a_k * dt
        p_true[k] = p_true[k - 1] + v_true[k - 1] * dt + 0.5 * a_k * dt * dt
    z_meas = p_true[1:] + rng.normal(0.0, 3.0, N)

    def run(qscale, rscale, keep_last=20):
        kf = LinearKF(np.array([[0.0], [0.0]]),
                      np.array([[100.0, 0.0], [0.0, 25.0]]),
                      F, H, Q_base * qscale, R_base * rscale)
        ep, ev = [], []
        for k in range(1, N + 1):
            kf.step([z_meas[k - 1]])
            if k > N - keep_last:
                ep.append(kf.x[0, 0] - p_true[k])
                ev.append(kf.x[1, 0] - v_true[k])
        return kf, np.array(ep), np.array(ev)

    # --------------------------------------------------------------
    sub("① 固定 R，扫 Q（Q 小 ⇒ 更信模型；Q 大 ⇒ 更信观测）")

    print("    R 固定为 σ_z² = %.1f（正确值）；Q 乘上不同倍数：" % R_base[0, 0])
    print()

    rows = []
    for qs in (0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0):
        kf, ep, ev = run(qs, 1.0)
        P_inf, K_inf = steady_state_P(F, H, Q_base * qs, R_base)
        rows.append([
            "×%g" % qs,
            "%.6f" % P_inf[0, 0],
            "%.6f" % P_inf[1, 1],
            "%.6f" % K_inf[0, 0],
            "%.6f" % K_inf[1, 0],
            "%.4f" % math.sqrt(np.mean(ep ** 2)),
            "%.4f" % math.sqrt(np.mean(ev ** 2)),
        ])
    tab(rows, ["Q 倍数", "稳态 P₀₀", "稳态 P₁₁", "K₀", "K₁", "位置 RMSE", "速度 RMSE"])

    print("""    ★ 读表：
        Q ↑  ⇒  P_∞ ↑  ⇒  K_∞ ↑  ⇒  更信观测、响应更快
        Q ↓  ⇒  P_∞ ↓  ⇒  K_∞ ↓  ⇒  更信模型、曲线更平滑

      ★ 注意最右两列的「U 形」：
        Q 太小时 RMSE【反而变大】—— 因为真实世界有加速度扰动（σ_a = 0.5），
        而滤波器以为没有，于是【跟不上】真实运动，产生滞后误差。
        Q 太大时 RMSE 也变大 —— 因为开始把观测噪声当信号跟了。

      ⇒ **Q 不是「越小越好」，它是「对模型误差的诚实程度」。**
        你把 Q 设得比真实模型误差小，就是在骗自己。
""")

    # --------------------------------------------------------------
    sub("② 固定 Q，扫 R（R 小 ⇒ 更信观测；R 大 ⇒ 更信模型）")

    print("    Q 固定（正确值）；R 乘上不同倍数：")
    print()

    rows = []
    for rs in (0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0):
        kf, ep, ev = run(1.0, rs)
        P_inf, K_inf = steady_state_P(F, H, Q_base, R_base * rs)
        rows.append([
            "×%g" % rs,
            "%.6f" % P_inf[0, 0],
            "%.6f" % P_inf[1, 1],
            "%.6f" % K_inf[0, 0],
            "%.6f" % K_inf[1, 0],
            "%.4f" % math.sqrt(np.mean(ep ** 2)),
            "%.4f" % math.sqrt(np.mean(ev ** 2)),
        ])
    tab(rows, ["R 倍数", "稳态 P₀₀", "稳态 P₁₁", "K₀", "K₁", "位置 RMSE", "速度 RMSE"])

    print("""    ★ 读表：
        R ↑  ⇒  K ↓  ⇒  几乎不听观测（估计几乎等于「按模型外推」）
        R ↓  ⇒  K ↑  ⇒  几乎照抄观测（估计变成一堆噪声）

      ⇒ **R 小是「自信的传感器」——但如果你把 R 写小了，
        滤波器会以为观测很准，于是把噪声全吃进状态里。**

      ★ 这里的位置 RMSE 也是「U 形」：R 太小（×0.1）时 RMSE 反而升到 1.87，
        R 太大（×100）时升到 5.20，最小点在 R 的【真值】附近。
        ⇒ 又一次说明：**噪声参数不是「越小越好」，而是「越准越好」。**

    ★★ 两句工程口诀：

        调 Q 是「调我对模型的信任度」—— 影响【相位/滞后】
        调 R 是「调我对传感器的信任度」—— 影响【噪声/抖动】

      实际的调试方法（第 8 章调参会细讲）：
        · 先按手册/Allan 方差把 R 定死（这个不该乱调）
        · 只用 Q 来在「跟得紧」和「抖得小」之间找平衡
""")

    # --------------------------------------------------------------
    sub("③ ★ Q/R 的比值才是关键（把 Q 和 R 同乘 100，结果不变）")

    rows = []
    for scale in (1.0, 100.0, 10000.0):
        kf, ep, ev = run(scale, scale)
        P_inf, K_inf = steady_state_P(F, H, Q_base * scale, R_base * scale)
        rows.append([
            "Q,R 同乘 ×%g" % scale,
            "%.6f" % P_inf[0, 0],
            "%.6f" % K_inf[0, 0],
            "%.4f" % math.sqrt(np.mean(ep ** 2)),
        ])
    tab(rows, ["情形", "稳态 P₀₀", "K₀", "位置 RMSE"])

    print("""    ★ 增益 K 完全不变！（P 会随比例缩放，但 K 不变）

      为什么？因为 K = P⁻Hᵀ/(HP⁻Hᵀ+R)，把 P 和 R 同比放大，K 的分子分母
      同阶 —— 结果不变（P 是 Q 的线性函数，所以 P 也同比放大）。

      ⇒ **KF 只认「Q 和 R 的比值」。**
        这意味着：**你的 Q 和 R 可以有系统性的量纲误差，只要比例对，就没事。**
        （但这只在【线性、定常】情形成立 —— 非线性 EKF 里不成立，
          所以卷 2 第 5 章之后还是老老实实标定。）
""")

    # --------------------------------------------------------------
    sub("④ 反例：Q = 0 + 模型错 ⇒ 增益「猝死」，滤波器发散")

    print("""    最危险的一种调参：Q = 0（「我的模型是完美的」）。
    此时 Riccati 迭代会一路把 P 压到 0，K 也随之压到 0：

""")

    rows = []
    for qs in (0.0, 1e-8, 1e-4, 1.0):
        P_inf, K_inf = steady_state_P(F, H, Q_base * qs, R_base, n_iter=5000)
        rows.append(["Q ×%g" % qs, "%.3e" % P_inf[0, 0], "%.3e" % K_inf[0, 0],
                     "%.3e" % P_inf[1, 1], "%.3e" % K_inf[1, 0]])
    tab(rows, ["情形", "稳态 P₀₀", "K₀", "稳态 P₁₁", "K₁"])

    print("""    ★ Q = 0 ⇒ P 收敛到【机器精度的 0】⇒ K ≈ 0 ⇒ 滤波器完全不理观测。
      此时如果真实系统有一点没建模的加速度（真实世界一定有），
      估计就会【单调地漂走】—— 这叫 **滤波器发散**。

      ⚠️ 而且注意：**P 会「自信地」显示它很准（P₀₀ ≈ 0）**，
         但真实误差在无限增长。**这就是「P 在撒谎」的极端情形。**

      ⇒ 工程铁律：**Q 永远不要设成 0。**
        哪怕你真的相信模型，也要留一个「跛脚项」，
        让滤波器保留「怀疑自己」的能力。

      PX4 的 EKF2 就是这么做的 —— 见 bias_estimator.cpp:46-51：
        预测时 P 无条件长大（_process_psd * dt），
        甚至在检测到「新息序列有偏」时【主动把过程噪声放大】
        （delta_state_var *= _process_var_boost_gain）。
        **这不是 hack，这是防止发散的标准手段。**
""")


# ---------------------------------------------------------------------------
# 实验 4：新息诊断
# ---------------------------------------------------------------------------

def exp4():
    head("实验 4  新息诊断：模型搞错时，新息会怎么「报警」")

    print("""
  课文 §10.8 说：模型正确时，新息 y_k = z_k - Hx̂_k⁻ 必须满足三条：
      ① 零均值
      ② 协方差 = H P⁻ Hᵀ + R     （这里已经是白化后的协方差 S）
      ③ ★ **是白噪声**（不同时刻互不相关）

  前两条好理解，第 ③ 条最关键：**如果模型正确，滤波器已经把信息榨干了，
  剩下的「残差」必须是没有规律的。一旦有规律 ⇒ 就是模型没建的东西。**

  这个实验跑三种情形：
      A  模型正确            → 三条全过（这就是上一次实验的样子）
      B  模型错了（真实在转弯，滤波以为是直线）  → 新息出现【相关性】+ 均值偏移
      C  R 设得太小（过度自信）  → NIS 爆表，大量观测被拒
""")

    dt = 1.0
    F = np.array([[1.0, dt], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    sigma_z = 3.0
    R_true = np.array([[sigma_z ** 2]])
    N = 120

    # 滤波器「以为」的过程噪声幅度
    sigma_a_model = 0.5
    Q = sigma_a_model ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                                       [dt ** 3 / 2, dt ** 2]])

    # 真实的机动幅度（滤波器不知道）
    A_man, T_man = 3.0, 30.0

    A_const = 0.5           # 常值加速度（真实有，模型没有）

    def truth(kind, seed):
        """生成真值轨迹。所有 kind 都叠加 i.i.d. 噪声 N(0, 0.2²)。

        line  —— 严格符合 CV 模型（加速度只有白噪声）
        weave —— 叠加一个【正弦机动】a(t) = A·sin(2π t / T)，A = 3.0
                 ★ 典型的「有规律但模型没建」的机动
        accel —— 叠加一个【常值加速度】a = 0.5 m/s²
                 ★ 典型的「结构性偏差」：滤波器会【稳定地滞后】一个固定量，
                   于是新息均值【持续非零】（不是震荡，是恒定偏置）
        """
        rng = np.random.default_rng(seed)
        p = np.zeros(N + 1)
        v = np.zeros(N + 1)
        p[0], v[0] = 0.0, 2.0
        for k in range(1, N + 1):
            a = rng.normal(0.0, 0.2)
            if kind == "weave":
                a += A_man * math.sin(2 * math.pi * (k - 0.5) * dt / T_man)
            elif kind == "accel":
                a += A_const
            v[k] = v[k - 1] + a * dt
            p[k] = p[k - 1] + v[k - 1] * dt + 0.5 * a * dt * dt
        return p, v

    def run_case(p_true, z, q_scale, r_scale):
        """跑一遍 KF 并收集诊断量。"""
        kf = LinearKF(np.array([[p_true[0]], [2.0]]),
                      np.array([[100.0, 0.0], [0.0, 25.0]]),
                      F, H, Q * q_scale, R_true * r_scale)
        for k in range(1, N + 1):
            kf.step([z[k - 1]])
        innov = np.array([r["y"][0, 0] for r in kf.log])
        svar = np.array([r["S"][0, 0] for r in kf.log])
        ratio, rej = nis_test(innov, svar, gate=3.0)
        err = np.array([kf.log[k - 1]["x_post"][0, 0] - p_true[k]
                        for k in range(1, N + 1)])
        sig = np.array([math.sqrt(r["P_post"][0, 0]) for r in kf.log])
        return dict(innov=innov, svar=svar, ratio=ratio, rej=rej,
                    err=err, sig=sig, kf=kf)

    # 三条真值轨迹用同一套测量噪声（公平对比）
    def make_z(p, seed):
        return p[1:] + np.random.default_rng(seed).normal(0.0, sigma_z, N)

    p_line, _ = truth("line", 777)
    p_weave, _ = truth("weave", 777)
    p_accel, _ = truth("accel", 777)
    z_line = make_z(p_line, 2024)
    z_weave = make_z(p_weave, 2024)
    z_accel = make_z(p_accel, 2024)

    print("    真值类型（都叠加 i.i.d. 噪声 σ = 0.2 m/s²）：")
    print("      line  —— 严格 CV（和滤波器模型完全一致）")
    print("      weave —— 再叠加正弦机动 a(t) = %.1f·sin(2π t / %.0f)"
          % (A_man, T_man))
    print("               峰值 %.1f m/s² = 滤波器假设 σ_a (%.1f) 的 %.0f 倍"
          % (A_man, sigma_a_model, A_man / sigma_a_model))
    print("      accel —— 再叠加常值加速度 a = %.1f m/s²" % A_const)
    print()

    cA = run_case(p_line, z_line, 1.0, 1.0)
    cB1 = run_case(p_weave, z_weave, 1.0, 1.0)
    cB2 = run_case(p_weave, z_weave, 100.0, 1.0)
    cD1 = run_case(p_accel, z_accel, 1.0, 1.0)
    cD2 = run_case(p_accel, z_accel, 100.0, 1.0)
    cC = run_case(p_line, z_line, 1.0, 0.001)

    # --------------------------------------------------------------
    sub("① 六种情形的统计对比")

    def stats(c, label):
        innov, svar, rej, err, sig = (c["innov"], c["svar"], c["rej"],
                                      c["err"], c["sig"])
        nis = float(np.mean(innov ** 2 / svar))
        return [
            label,
            "%+.4f" % innov.mean(),
            "%+.4f" % autocorr_lag1(innov),
            "%.3f" % nis,
            "%d / %d" % (int(rej.sum()), len(innov)),
            "%.4f" % math.sqrt(np.mean(err ** 2)),
            "%.4f" % sig.mean(),
        ]

    rows = [
        stats(cA, "A  模型对 + 参数对"),
        stats(cB1, "B1 正弦机动 + Q 正常"),
        stats(cB2, "B2 正弦机动 + Q×100"),
        stats(cD1, "D1 常值加速度 + Q 正常"),
        stats(cD2, "D2 常值加速度 + Q×100"),
        stats(cC, "C  模型对 + R 小 1000 倍"),
    ]
    tab(rows, ["情形", "新息均值", "自相关", "平均NIS", "被拒", "位置RMSE", "平均√P"])

    print("""    ★★ 逐行读（这张表是本实验的全部价值）：

      A  模型对、参数对（基准）
          均值 ≈ 0 · 自相关 ≈ 0 · NIS ≈ 1 · 拒绝 0 次 · RMSE ≈ 平均√P
          ⇒ **三条诊断全过。这是「健康滤波器」的样子。**

      B1  真值有正弦机动（峰值 3 m/s²），滤波器却以为只有 0.5 的白噪声
          自相关【+0.93，几乎完美相关】—— 因为误差是【有规律的】，
              不是随机的（正弦的周期是 30 s，滤波器却当成白噪声）
          NIS ≈ 17 —— 新息比滤波器自以为的大 4 倍多
          81/120 次观测被拒
          ⇒ **三条诊断同时报警。这就是「模型错了」的指纹。**

      B2  同样的机动，把 Q 放大 100 倍（滤波器「认怂」）
          自相关掉回 0、NIS 掉回 1、不再拒绝观测 —— **症状消失了**
          但这不是免费的：平均 √P 从 2.01 涨到 2.73，
          即**滤波器承认自己更不确定了** —— 这正确反映了「Q 是猜的」。
          ⇒ **Q 放大 = 用「噪声敏感度」换「跟踪能力」，
            是一笔等价交换，不是修复。**

      D1  真值有常值加速度 0.5 m/s²（模型里没有「加速度」这个状态）
          新息均值【+3.56，持续偏正】—— 滤波器稳定地滞后一个固定量
          ★★ 但注意：**自相关 ≈ 0，NIS 只有 1.56，几乎不拒绝观测！**
          为什么？因为「恒定偏置 + 白噪声」仍然是【白噪声】——
          均值移位不改变自相关函数。这是本实验最反直觉的一条。

          ⇒ **所以 D1 这类误差比 B1 危险得多：**
            B1（有规律机动）会被自相关和 NIS 抓住；
            D1（结构性恒定偏差）**只有「新息均值」能抓住**，
            而工程上很多实现只做 NIS 门限检验 —— 它会悄悄漏过去。

          ⇒ 这正是 PX4 为什么要**额外**做「新息符号序列监测」：
            见 bias_estimator.cpp:109-131 的 updateOffsetDetection /
            isOffsetDetected —— 它不是看新息大不大，而是看
            **新息的符号是不是长时间不变**，专门抓这种恒定偏置。
            **一个 NIS 门限是不够的。**

      D2  同样的常值加速度，Q 放大 100 倍
          新息均值从 +3.56 降到 +0.67，RMSE 也降了 —— 但**仍然非零**
          ⇒ ★★ **Q 永远消不掉结构性偏差，它只能把偏差压小。**
            想真正消掉，必须**把这个物理量建成一个状态**
            （加速度偏置 → 卷 2 第 5 章 EKF；传感器零偏 → 卷 2 第 6 章 ESKF）。
            这就是「Q 是止疼药，不是解药」的确切含义。

      C   模型对，但 R 设小了 1000 倍（谎称传感器很准）
          【NIS ≈ 314】、103/120 次观测被门限拒绝
          ⇒ **R 写小 = 说谎说传感器很准 ⇒ 新息相对 S 显得异常大。**
            工程上表现是：滤波器频繁拒观测 ⇒ 退化成纯惯性外推 ⇒ 漂移。

    ⚠️ 一个必须点破的陷阱（读 B1 / D1 / C 一起看）：
      **新息大【不一定】是观测坏了。**
      B1 和 D1 里新息大，是因为【模型】坏了，而那些观测其实都是好的。
      如果这时滤波器把观测全拒掉，它就彻底失去了唯一的真实信息源
      —— 这叫 **「滤波器把自己骗瞎了」**，是真实事故里最常见的一条路径。
""")

    # --------------------------------------------------------------
    sub("② 新息的「形状」：把自相关画出来（数值）")

    def acf(seq, nlag=6):
        x = np.asarray(seq, dtype=float).ravel()
        x = x - x.mean()
        d = float(np.sum(x * x))
        out = []
        for l in range(nlag + 1):
            # ★ 注意 l=0 时不能写 x[:-0]（那是空数组），要显式写 len(x)
            out.append(float(np.sum(x[l:] * x[:len(x) - l]) / d))
        return out

    nlag = 6
    hdr = ["情形"] + ["lag %d" % l for l in range(nlag + 1)]
    rows = [
        ["A  模型对"] + ["%+.4f" % v for v in acf(cA["innov"], nlag)],
        ["B1 正弦机动"] + ["%+.4f" % v for v in acf(cB1["innov"], nlag)],
        ["D1 常值加速度"] + ["%+.4f" % v for v in acf(cD1["innov"], nlag)],
        ["C  R 设小"] + ["%+.4f" % v for v in acf(cC["innov"], nlag)],
    ]
    tab(rows, hdr)

    print("""    ★ 判读标准：白噪声的自相关在所有 lag > 0 处都应落在
      ±1.96/√N = ±%.3f 以内（95%% 置信带）。

        A   全部落在带内              ⇒ 白 ✓ 模型对
        B1  lag 1~6 全部远超 0.5      ⇒ 强相关 ✗ 模型错（有规律机动）
        D1  基本落在带内              ⇒ 看起来「白」✗ **漏报！**
        C   慢衰减（lag 1 = -0.70）   ⇒ 相关 ✗ 但这是 R 的错，不是模型的错

      ⇒ ★★ **D1 这一行是本实验最重要的发现：**
        **一个恒定偏置的新息，自相关检验是【抓不到】的。**
        所以工程上必须同时看「自相关」和「新息均值」两个量 ——
        只看一个，会漏掉一半的模型问题。
""" % (1.96 / math.sqrt(N)))

    # --------------------------------------------------------------
    sub("③ 新息卡方检验：PX4 是怎么「自动」拒掉坏观测的")

    print("""    PX4 的做法（ekf.h:1182-1195）是把新息归一化成一个「检验比」：

        test_ratio = y² / (gate² · S)          ← gate 默认 3（= 3σ）
        innovation_rejected = (test_ratio > 1)

    这本质就是 **归一化新息平方（NIS）** 的单次版本。
    看下面的累积拒绝次数随时间的变化：
""")

    def cum_rej(rej):
        return np.cumsum(rej.astype(int))

    cumA, cumB, cumC = cum_rej(cA["rej"]), cum_rej(cB1["rej"]), cum_rej(cC["rej"])
    marks = [1, 20, 40, 60, 80, 100, 120]
    rows = []
    for m in marks:
        rows.append([m, "%d" % cumA[m - 1], "%d" % cumB[m - 1], "%d" % cumC[m - 1]])
    tab(rows, ["步数 k", "A 累计拒", "B1 累计拒", "C 累计拒"])

    print("""    ★ A 基本不拒（偶发 1~2 次是正常的，因为 3σ 门限本身就有 ~0.3% 的误报率）。
      B1 在中后期持续被拒 —— 模型跟不上，新息一直很大。
      C 从一开始就疯狂拒绝 —— R 说谎的代价。

      ⇒ **工程上要盯的两个量（卷 2 第 8 章 EKF2 会反复出现）**：

          innovation     —— 有没有系统性偏离（模型错的信号）
          test_ratio     —— 相对滤波器「自我认知」的异常程度
          rejected 比例  —— 持续 > 10% 就说明参数或模型有问题

      ⚠️ 但注意一个**陷阱**：
        **拒绝观测只能防止坏数据污染状态，不能修复错模型。**
        B1 的情形下，被拒的观测其实是【好数据】，
        是滤波器自己的模型错了，才把它们判成坏的 ——
        这叫 **「滤波器把自己骗瞎了」**，
        是真实事故里最常见的一条路径。
""")

    if _HAS_MPL:
        _plot_innovation(cB1["innov"], cC["innov"])
    else:
        print("    （没装 matplotlib，跳过画图）")
        print()


# ---------------------------------------------------------------------------
# 画图（可选）
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


def _t(zh, en):
    return zh if _CJK else en


def _plot_tracking(kf, p_true, v_true, z_meas, sig_p, sig_v):
    N = len(z_meas)
    ks = np.arange(1, N + 1)
    p_est = np.array([r["x_post"][0, 0] for r in kf.log])
    v_est = np.array([r["x_post"][1, 0] for r in kf.log])
    k0 = np.array([r["K"][0, 0] for r in kf.log])
    k1 = np.array([r["K"][1, 0] for r in kf.log])

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))

    ax = axes[0]
    ax.plot(ks, z_meas, "x", color="0.6", ms=4, label=_t("观测 z", "measurement"))
    ax.plot(ks, p_true[1:], "k-", lw=1.4, label=_t("真实位置", "true pos"))
    ax.plot(ks, p_est, "r-", lw=1.8, label=_t("KF 估计", "KF estimate"))
    ax.fill_between(ks, p_est - 3 * sig_p, p_est + 3 * sig_p,
                    color="r", alpha=0.15, label=_t("±3σ", "+-3sigma"))
    ax.set_xlabel("k")
    ax.set_ylabel(_t("位置 [m]", "position [m]"))
    ax.set_title(_t("位置：观测 vs 估计", "position: meas vs est"))
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(ks, v_true[1:], "k-", lw=1.4, label=_t("真实速度", "true vel"))
    ax.plot(ks, v_est, "b-", lw=1.8, label=_t("KF 速度估计", "KF vel"))
    ax.fill_between(ks, v_est - 3 * sig_v, v_est + 3 * sig_v, color="b", alpha=0.15)
    ax.set_xlabel("k")
    ax.set_ylabel(_t("速度 [m/s]", "velocity [m/s]"))
    ax.set_title(_t("速度：从未被直接观测", "velocity: never measured"))
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.plot(ks, k0, "r-", lw=1.8, label="K0")
    ax.plot(ks, k1, "b-", lw=1.8, label="K1")
    ax.set_xlabel("k")
    ax.set_ylabel(_t("增益", "gain"))
    ax.set_title(_t("增益的收敛", "gain convergence"))
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out = "kf_tracking.png"
    plt.savefig(out, dpi=110)
    plt.close(fig)
    print("    ✅ 图已存：%s%s" % (out, "" if _CJK else "（无中文字体，标签用英文）"))
    print()


def _plot_innovation(iB, iC):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7))

    ax = axes[0]
    ax.plot(iB, "r-", lw=1.2, alpha=0.85,
            label=_t("B1: 有正弦机动，模型没建", "B1: unmodeled maneuver"))
    ax.plot(iC, "m-", lw=1.0, alpha=0.6,
            label=_t("C: R 设小 1000 倍", "C: R too small"))
    ax.axhline(0, color="k", lw=0.6)
    sA = float(np.std(iB))
    ax.axhline(3 * sA, color="k", ls="--", lw=0.8)
    ax.axhline(-3 * sA, color="k", ls="--", lw=0.8)
    ax.set_xlabel("k")
    ax.set_ylabel("innovation")
    ax.set_title(_t("新息序列：模型错时出现明显的周期性",
                    "innovation: periodic pattern when model is wrong"))
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)

    def acf_local(seq, lags=20):
        x = np.asarray(seq, dtype=float).ravel()
        x = x - x.mean()
        d = float(np.sum(x * x))
        return [float(np.sum(x[l:] * x[:len(x) - l]) / d)
                for l in range(lags + 1)]

    ax = axes[1]
    lags = np.arange(21)
    ax.stem(lags - 0.15, acf_local(iB), linefmt="r-", markerfmt="ro",
            basefmt="k-", label="B1: unmodeled maneuver")
    ax.stem(lags + 0.15, acf_local(iC), linefmt="m-", markerfmt="mo",
            basefmt="k-", label="C: R too small")
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("lag")
    ax.set_ylabel("autocorrelation")
    ax.set_title(_t("新息自相关：白噪声应该在 0 附近",
                    "innovation ACF: white noise should be near 0"))
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out = "kf_innovation.png"
    plt.savefig(out, dpi=110)
    plt.close(fig)
    print("    ✅ 图已存：%s%s" % (out, "" if _CJK else "（无中文字体，标签用英文）"))
    print()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("一维 KF 手算 + 脚本对照", exp1),
    "2": ("★★ 二维目标跟踪：看 P 怎么收缩", exp2),
    "3": ("★ Q 与 R 的影响", exp3),
    "4": ("新息诊断：模型错了会怎样", exp4),
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
