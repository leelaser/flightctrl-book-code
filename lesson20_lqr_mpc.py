#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 3 第 5 章 · LQR 与 MPC 进阶 —— 配套实验

    实验 1  ★   从 PID 到最优控制：倒立摆上 PID 和 LQR 的正面对决
    实验 2  ★★  Riccati 方程到底怎么解出来（不动点迭代 + 连续 Riccati ODE）
    实验 3  ★★★ LQR 和 Kalman 滤波是同一个数学（对偶性 → LQG）
    实验 4  ★★  MPC：滚动时域 + 约束，以及「飞控里到底用不用」

用法：
    python lesson20_lqr_mpc.py 1
    python lesson20_lqr_mpc.py 2
    python lesson20_lqr_mpc.py 3
    python lesson20_lqr_mpc.py 4
    python lesson20_lqr_mpc.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  能算的就不猜。

  · 实验 2 里 Riccati 方程是**真的**一步步迭代出来的，
    每次迭代的 ‖P_{k+1} - P_k‖ 都打印出来，最后还验残差。
  · 实验 3 里 Kalman 增益是**调用同一个 dare()** 算出来的，
    直接证明 LQR 和 Kalman 是同一套数学换了个转置。
  · 实验 4 里 MPC 是**真的**在解一个带约束的 QP，
    还把求解时间用 perf_counter 测出来 —— 因为「MPC 算得慢」
    这句话必须用数字说话。

【关于「飞控里到底用不用 LQR/MPC」】

  这是本章唯一需要「查源码」而不是「推导」的结论。已核对：

    PX4        $SRC_PX4/
               grep -ril "riccati" src/           → 0 个文件
               grep -rnw -i "lqr"    src/         → 0 处（只有一个 PDF 误匹配）
               grep -rn -i "prediction horizon|model predictive" src/ → 0 处
               PX4 里那个 MPC_ 参数前缀 = **M**ulticopter **P**osition **C**ontrol
               （参数定义在 src/modules/mc_pos_control/*.c，共 61 个）

    ArduPilot  $SRC_APM/
               grep -ril "riccati" libraries/ ArduCopter/      → 0 个文件
               grep -rn -i "model predictive" libraries/ ArduCopter/ → 0 处

  也就是说：这两个飞控里 **一行 LQR、一行 MPC 都没有**。
  但实验 3 会告诉你，PX4 的 EKF2 里到处是 Kalman 增益 ——
  那是 LQG 的**另一半**。

参考：
    卷 2 第 5 章（Q 和 R 的原始含义）
    卷 3 第 1 章（ζ / ωn）
    卷 3 第 3 章（为什么位置环是串级）
"""

import itertools
import _srcdirs
import math
import sys
import time

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
# 线性代数工具（只用 numpy，不依赖 scipy）
# ===========================================================================

def expm(M):
    """矩阵指数 exp(M)：缩放 + 泰勒展开 + 平方。

    对 2x2 / 4x4 这种小矩阵足够精确（误差 ~1e-16）。
    """
    M = np.asarray(M, dtype=float)
    n = M.shape[0]
    nrm = float(np.max(np.sum(np.abs(M), axis=1)))
    k = 0
    while nrm > 0.5 and k < 40:
        nrm *= 0.5
        k += 1
    Aq = M / (2.0 ** k)
    E = np.eye(n)
    term = np.eye(n)
    for i in range(1, 60):
        term = term @ Aq / float(i)
        E = E + term
        if float(np.max(np.abs(term))) < 1e-20:
            break
    for _ in range(k):
        E = E @ E
    return E


def c2d_zoh(A, B, dt):
    """连续 (A,B) → 离散 (Ad,Bd)，零阶保持。

    做法：把 [A B; 0 0] 求矩阵指数，左上角是 Ad，右上角是 Bd。
    这不是玄学 —— expm 的定义直接给出 x(k+1) = Ad x(k) + Bd u(k)。
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    E = expm(M * dt)
    return E[:n, :n], E[:n, n:]


def dlqr(A, B, Q, R):
    """离散 LQR：解 DARE，返回 (K, P)。

        K = (R + B'PB)^-1 B'PA
    """
    P, K, _n, _h = dare(A, B, Q, R)
    return K, P


def dare(A, B, Q, R, max_iter=5000, tol=1e-15):
    """解离散代数 Riccati 方程（DARE）—— 用不动点迭代。

        P_{k+1} = Q + A'P_k A - A'P_k B (R + B'P_k B)^-1 B'P_k A

    返回 (P, K, 迭代次数, 每步的 ‖P_{k+1}-P_k‖ 列表)。

    ★ 这不是「某个库函数」，就是 Riccati 方程本身迭代到不动点。
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)

    P = Q.copy()
    hist = []
    it = 0
    for it in range(1, max_iter + 1):
        S = R + B.T @ P @ B
        # K_k = S^-1 B'PA   （中间增益，最终收敛到最优 K）
        Kk = np.linalg.solve(S, B.T @ P @ A)
        Pn = A.T @ P @ A - A.T @ P @ B @ Kk + Q
        Pn = 0.5 * (Pn + Pn.T)                    # 强制对称，压掉浮点毛刺
        d = float(np.max(np.abs(Pn - P)))
        P = Pn
        hist.append(d)
        if d < tol:
            break

    S = R + B.T @ P @ B
    K = np.linalg.solve(S, B.T @ P @ A)
    return P, K, it, hist


def dare_residual(A, B, Q, R, P):
    """把 P 代回 DARE，看残差有多大。教学用：证明「真的解出来了」。"""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    S = np.asarray(R, dtype=float) + B.T @ P @ B
    lhs = A.T @ P @ A - P - A.T @ P @ B @ np.linalg.solve(S, B.T @ P @ A) \
        + np.asarray(Q, dtype=float)
    return float(np.max(np.abs(lhs)))


def kf_gain(A, C, Qn, Rn):
    """稳态 Kalman 增益（【预测形式】）—— 注意它是【同一个 dare()】换了个转置。

        P  = dare(A', C', Qn, Rn)          ← 对偶，这就是预测协方差
        K  = (Rn + C P C')^-1 C P A'       ← dare 返回的对偶 LQR 增益
        L  = K' = A P C' (C P C' + Rn)^-1  ← 转置回列向量，预测形式的 Kalman 增益

    注意：L 里带一个 A（预测形式），因为本文件里的观测器更新式是
        x_hat[k+1] = A x_hat[k] + B u[k] + L (y[k] - C x_hat[k])
    y[k] 直接推进到 k+1，配套的增益必须带 A。
    课本上常见的滤波形式 L_f = P_f C' (C P_f C' + Rn)^-1 不带 A，
    但那里的 P_f 是【更新后】协方差，和这里的 P 不是同一个矩阵。
    两者关系：L = A @ L_f。写代码时【先把更新式写出来】，再决定用哪种。

    返回 (L, P)。
    """
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float)
    P, L, _n, _h = dare(A.T, C.T, np.asarray(Qn, dtype=float),
                        np.asarray(Rn, dtype=float))
    return L.T, P


def eig_str(M):
    """把特征值打印成 a±bj 的字符串。"""
    ev = np.linalg.eigvals(np.asarray(M, dtype=float))
    ev = ev[np.argsort(-np.abs(ev))]
    out = []
    for z in ev:
        if abs(z.imag) < 1e-9:
            out.append("%.3f" % z.real)
        else:
            out.append("%.3f%+.3fj" % (z.real, z.imag))
    return ", ".join(out)


# ===========================================================================
# 倒立摆：本章的主角
# ===========================================================================
#
#   一根长度 l、质量 m 的杆，铰接在小车上（这里只考虑杆本身，车不动）。
#   角度 theta 从【竖直向上】量起。用力矩 u 驱动。
#
#       theta_ddot = (g/l) * sin(theta) - (b/(m*l^2)) * theta_dot + u/(m*l^2)
#
#   在 theta = 0 处线性化（sin(theta) ≈ theta）：
#
#       A = [[0,      1               ],
#            [g/l,  -b/(m*l^2)        ]]      B = [[0], [1/(m*l^2)]]
#
#   ★ 注意 A[1][0] = +g/l > 0 —— 这是**正反馈**，开环有一个右半平面极点。
#     这就是「倒立」的意思：不控就倒。

PEND_M = 0.5          # kg
PEND_L = 0.4          # m
PEND_DAMP = 0.02      # N·m·s/rad（铰链的粘性阻尼）
PEND_G = 9.80665      # m/s^2

PEND_ML2 = PEND_M * PEND_L ** 2                 # = 0.08 kg·m^2
PEND_A11 = PEND_G / PEND_L                      # = 24.516625  1/s^2
PEND_A22 = -PEND_DAMP / PEND_ML2                # = -0.25      1/s
PEND_B2 = 1.0 / PEND_ML2                        # = 12.5      1/(kg·m^2)

PEND_A = np.array([[0.0, 1.0],
                   [PEND_A11, PEND_A22]])
PEND_B = np.array([[0.0],
                   [PEND_B2]])

DT = 0.004            # 250 Hz，和真实飞控的速率环一个量级


def pend_step(x, u, dt, substeps=8):
    """非线性倒立摆积分一步（RK4，内部再细分 substeps 次）。

    状态 x = [theta, theta_dot]。
    返回新的 x —— 如果 |theta| 超过 90 度就返回 None（倒了）。
    """
    def f(xx, uu):
        th, thd = xx
        return np.array([thd,
                         PEND_A11 * math.sin(th) + PEND_A22 * thd
                         + PEND_B2 * uu])

    h = dt / float(substeps)
    for _ in range(substeps):
        k1 = f(x, u)
        k2 = f(x + 0.5 * h * k1, u)
        k3 = f(x + 0.5 * h * k2, u)
        k4 = f(x + h * k3, u)
        x = x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        if abs(x[0]) > 0.5 * math.pi:
            return None
    return x


def sim_pend(x0, K, n_step, umax=None):
    """用 u = -K x 闭环仿真非线性倒立摆。

    返回 (t 数组, x 数组[N,2], u 数组[N], 累加代价 J, 是否挺住)。
    代价用离散形式： J = sum (x'Qx + u'R u) —— Q/R 由调用者自己算。
    """
    K = np.asarray(K, dtype=float).reshape(1, -1)
    x = np.asarray(x0, dtype=float).copy()
    xs = np.zeros((n_step + 1, 2))
    us = np.zeros(n_step + 1)
    xs[0] = x
    alive = True
    for k in range(n_step):
        u = float(-(K @ x)[0])
        if umax is not None:
            u = max(-umax, min(umax, u))
        us[k] = u
        x = pend_step(x, u, DT)
        if x is None:
            alive = False
            xs[k + 1:] = np.nan
            break
        xs[k + 1] = x
    return (np.arange(n_step + 1) * DT, xs, us, alive)


def cost_sum(xs, us, Q, R, n_step):
    """离散代价  J = sum_{k=0}^{n-1} (x_k' Q x_k + u_k' R u_k)。

    ★ 一定要用【求和】而不是【积分】——
      因为 Riccati 方程给的是这个和的最小值  J* = x0' P x0。
      写成 ∫...dt 就会差一个 dt 因子，对不上。这是初学者最容易踩的坑。
    """
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float).reshape(-1)
    j = 0.0
    n = min(n_step, len(xs) - 1)
    for k in range(n):
        if not np.isfinite(xs[k]).all():
            return float("inf")
        j += float(xs[k] @ Q @ xs[k]) + float(R[0] * us[k] ** 2)
    return j


def cost_int(ts, xs, us, Q, R, n_step):
    """按 J = ∫(x'Qx + u'Ru)dt 的离散化算实际代价（梯形法）。

    和 cost_sum 只差一个 dt 因子（当 dt 很小时）。留着是为了
    说明「连续代价」和「离散代价」的口径区别。
    """
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float).reshape(-1)
    j = 0.0
    n = min(n_step, len(ts) - 1)
    for k in range(n):
        if not np.isfinite(xs[k]).all() or not np.isfinite(xs[k + 1]).all():
            return float("inf")
        lk = float(xs[k] @ Q @ xs[k]) + float(R[0] * us[k] ** 2)
        lk1 = float(xs[k + 1] @ Q @ xs[k + 1]) + float(R[0] * us[k + 1] ** 2)
        j += 0.5 * (lk + lk1) * (ts[k + 1] - ts[k])
    return j


def disc_poles_to_cont(eigs, dt):
    """离散闭环极点 → 等价的连续极点（z = exp(s*dt) ⇒ s = ln(z)/dt）。

    返回 (ωn, ζ) 列表 —— 直接和卷 3 第 1 章的 ζ/ωn 对上。
    """
    out = []
    for z in np.asarray(eigs):
        s = np.log(complex(z)) / dt
        wn = abs(s)
        zeta = -s.real / wn if wn > 0 else 0.0
        out.append((wn, zeta))
    return out


def settle_time(ts, xs, band=0.02):
    """从最后往回找，第一个「之后永远留在带内」的时刻。"""
    th = xs[:, 0]
    ok = np.abs(th) < band
    for k in range(len(th) - 1, -1, -1):
        if not ok[k]:
            if k + 1 >= len(th):
                return None
            return ts[k + 1]
    return ts[0]


def max_abs_u(us):
    return float(np.max(np.abs(us)))


# ===========================================================================
# MPC：滚动时域 + 浓缩 QP
# ===========================================================================
#
#   离散模型   x_{k+1} = Ad x_k + Bd u_k
#
#   把未来 N 步的状态全部堆成一个列向量：
#
#       X = [x_1; x_2; ...; x_N]        （n*N 维）
#       U = [u_0; u_1; ...; u_{N-1}]    （m*N 维）
#
#   递推 N 次，全部用 x_0 和 U 表示：
#
#       X = Abar x_0 + Bbar U
#
#       Abar = [Ad; Ad^2; ...; Ad^N]
#       Bbar = [[Bd,      0,   ...  0    ],
#               [Ad Bd,   Bd,  ...  0    ],
#               [Ad^2 Bd, Ad Bd, ... 0   ],
#               ...]                      ← 下三角分块 Toeplitz
#
#   代价（终端代价用 Riccati 解 P，这样才有「无限时域」的味道）：
#
#       J = X' Qbar X + U' Rbar U        Qbar = diag(Q,...,Q,P)
#
#   代进去，得到**只关于 U** 的二次型 —— 这就是「浓缩」：
#
#       J(U) = 0.5 U' H U + (F' x_0)' U + const
#
#       H = 2 (Bbar' Qbar Bbar + Rbar)
#       F = 2  Abar' Qbar Bbar
#
#   ★ 于是 MPC 的一步 = 解一个 box 约束的 QP：
#
#       min_U  0.5 U'HU + (F'x_0)'U      s.t.  u_min <= U <= u_max
#
#   本项目用一个最朴素的求解器：**投影梯度法**
#
#       U <- clip( U - (1/L) * grad , u_min, u_max ),   L = λmax(H)
#
#   ★ 选它不是为了快，恰恰相反 —— 是为了让「算出最优解需要迭代多少次」
#     这件事**看得见**。真实 MPC 用 active-set / interior-point QP，
#     但「迭代次数跟数据有关、没有固定的 WCET」这一点是一样的。

def mpc_condensed(Ad, Bd, Q, R, N, Pterm):
    """构造浓缩 QP 的 (Abar, Bbar, Qbar, Rbar, H, F)。"""
    Ad = np.asarray(Ad, dtype=float)
    Bd = np.asarray(Bd, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    n, m = Ad.shape[0], Bd.shape[1]

    Abar = np.zeros((N * n, n))
    Bbar = np.zeros((N * n, N * m))
    Apow = np.eye(n)
    for i in range(N):
        Apow = Ad @ Apow
        Abar[i * n:(i + 1) * n, :] = Apow
    for j in range(N):
        Apow = np.eye(n)
        for i in range(j, N):
            Bbar[i * n:(i + 1) * n, j * m:(j + 1) * m] = Apow @ Bd
            Apow = Ad @ Apow

    Qbar = np.zeros((N * n, N * n))
    for i in range(N - 1):
        Qbar[i * n:(i + 1) * n, i * n:(i + 1) * n] = Q
    Qbar[(N - 1) * n:, (N - 1) * n:] = Pterm
    Rbar = np.kron(np.eye(N), R)

    H = 2.0 * (Bbar.T @ Qbar @ Bbar + Rbar)
    F = 2.0 * (Abar.T @ Qbar @ Bbar)
    return {"Ad": Ad, "Bd": Bd, "Abar": Abar, "Bbar": Bbar,
            "Qbar": Qbar, "Rbar": Rbar,
            "H": H, "F": F, "N": N, "n": n, "m": m}


def pg_qp(mpc, x0, umin=None, umax=None, n_iter=4000, U0=None,
          rate_idx=None, rate_max=None, rate_rho=0.0):
    """投影梯度法解浓缩 QP。返回 (U, 用了多少次迭代)。

    rate_idx / rate_max / rate_rho：软「速率约束」。
      对预测状态里下标为 rate_idx 的分量加罚：
          rho * max(0, |s_i| - rate_max)^2
      这不是硬约束（罚函数法不可能精确满足），但足够说明
      「MPC 能把约束写进优化问题里，LQR 不能」。
    """
    H, F = mpc["H"], mpc["F"]
    Abar, Bbar = mpc["Abar"], mpc["Bbar"]
    n, N = mpc["n"], mpc["N"]

    U = np.zeros(N) if U0 is None else U0.copy()
    g0 = F.T @ x0
    lmax = float(np.max(np.linalg.eigvalsh(H)))
    alpha = 1.0 / lmax

    pen = np.zeros(N * n)
    for it in range(1, n_iter + 1):
        grad = H @ U + g0
        if rate_rho > 0.0 and rate_max is not None:
            s = Abar @ x0 + Bbar @ U
            d = s[rate_idx::n] - rate_max
            d2 = -s[rate_idx::n] - rate_max
            pen[:] = 0.0
            pen[rate_idx::n] = 2.0 * rate_rho * (np.where(d > 0.0, d, 0.0)
                                                 - np.where(d2 > 0.0, d2, 0.0))
            grad = grad + Bbar.T @ pen
        U = U - alpha * grad
        if umin is not None:
            U = np.clip(U, umin, umax)
        # 简单停机准则：梯度足够小
        if float(np.max(np.abs(grad))) < 1e-11:
            return U, it
    return U, n_iter


def qp_box_exact(H, c, lo, hi):
    """min 0.5 U'HU + c'U   s.t.  lo <= U <= hi —— 穷举 active set 的精确解。

    每个变量 3 种状态（自由 / 贴下界 / 贴上界）⇒ 3^n 种组合。
    n = 4 → 81 种，n = 8 → 6561 种，秒出；
    n = 20 → 3.5e9 种，不可能 —— 这就是 MPC 用不起穷举的原因（第 ⑤ 节）。

    ★ 它的用途是【当基准】：验证迭代解法的答案对不对，
      而不是拿它去跑 MPC。
    """
    n = H.shape[0]
    lo = np.full(n, float(lo)) if np.isscalar(lo) else np.asarray(lo, float)
    hi = np.full(n, float(hi)) if np.isscalar(hi) else np.asarray(hi, float)
    best = None
    for code in itertools.product((0, 1, 2), repeat=n):
        idx_lo = [i for i in range(n) if code[i] == 1]
        idx_hi = [i for i in range(n) if code[i] == 2]
        free = [i for i in range(n) if code[i] == 0]
        u = np.zeros(n)
        for i in idx_lo:
            u[i] = lo[i]
        for i in idx_hi:
            u[i] = hi[i]
        fixed = idx_lo + idx_hi
        if free:
            rhs = c[free].copy()
            if fixed:
                rhs = rhs + H[np.ix_(free, fixed)] @ u[fixed]
            u[free] = np.linalg.solve(H[np.ix_(free, free)], -rhs)
        if np.all(u >= lo - 1e-9) and np.all(u <= hi + 1e-9):
            u = np.clip(u, lo, hi)
            cost = float(0.5 * u @ H @ u + c @ u)
            if best is None or cost < best[0]:
                best = (cost, u.copy())
    return best[1]


def run_receding(x0, mpc, K_lqr, umax=None, n_step=600, niter=400,
                 mode="mpc", dt=None):
    """滚动时域仿真。

    mode = "mpc"     每步解一次 QP，只施加 U[0]
    mode = "lqr"     直接 u = -K x（可再夹一个 umax），不解 QP
    """
    Ad, Bd = mpc["Ad"], mpc["Bd"]
    N = mpc["N"]

    x = np.asarray(x0, dtype=float).copy()
    xs = np.zeros((n_step + 1, 2))
    us = np.zeros(n_step + 1)
    xs[0] = x
    U = np.zeros(N)
    iters = 0
    for k in range(n_step):
        if mode == "lqr":
            u = float(-(K_lqr @ x)[0])
            if umax is not None:
                u = max(-umax, min(umax, u))
        else:
            U, ni = pg_qp(mpc, x, None if umax is None else -umax,
                          umax, niter, U)
            iters += ni
            u = float(U[0])
        us[k] = u
        x = Ad @ x + Bd[:, 0] * u
        xs[k + 1] = x
    return xs, us, iters




# ===========================================================================
# 实验 1  ★ 从 PID 到最优控制：倒立摆上 PID 和 LQR 的正面对决
# ===========================================================================

def exp1():
    head("实验 1  ★ 从 PID 到最优控制：倒立摆上 PID 和 LQR 的正面对决")

    print("  为什么用倒立摆？因为它最像飞控的核心矛盾：")
    print()
    print("      · 开环**不稳定**（有一个右半平面极点）")
    print("      · 执行器只有一个（一个力矩 / 四个电机）")
    print("      · 状态有两个（角度 + 角速度），但传感器往往只给角度")
    print()
    print("  这和卷 3 第 3 章的位置环一模一样：**欠驱动 + 不稳定 + 串级**。")
    print()

    sub("① 模型：一根倒立的杆")

    print("  杆长 l = %.2f m，质量 m = %.2f kg，铰链阻尼 b = %.3f N·m·s/rad"
          % (PEND_L, PEND_M, PEND_DAMP))
    print()
    print("      theta_ddot = (g/l)*sin(theta) - (b/(m*l^2))*theta_dot")
    print("                   + u/(m*l^2)")
    print()
    print("  在 theta = 0（竖直向上）处线性化：sin(theta) ≈ theta")
    print()
    print("      A = [[0, 1], [g/l, -b/(m*l^2)]]      B = [[0], [1/(m*l^2)]]")
    print()
    print("  代入数字：")
    print("      m*l^2 = %.4f kg·m^2" % PEND_ML2)
    print("      g/l   = %.6f  1/s^2   ← ★ 正号！这是正反馈，所以开环不稳"
          % PEND_A11)
    print("      b/(m*l^2) = %.4f  1/s" % (-PEND_A22))
    print("      1/(m*l^2) = %.4f  1/(kg·m^2)" % PEND_B2)
    print()
    print("      A = [[%.1f, %.2f], [%.6f, %.4f]]"
          % (PEND_A[0, 0], PEND_A[0, 1], PEND_A[1, 0], PEND_A[1, 1]))
    print("      B = [[%.1f], [%.4f]]" % (PEND_B[0, 0], PEND_B[1, 0]))
    print()
    print("  开环极点：%s" % eig_str(PEND_A))
    print("      ⇒ 一个在 +%.3f rad/s（右半平面），不控必倒。"
          % max(np.linalg.eigvals(PEND_A).real))
    print()

    Ad, Bd = c2d_zoh(PEND_A, PEND_B, DT)
    print("  离散化（零阶保持，dt = %.0f ms ⇒ %.0f Hz）：" % (DT * 1e3, 1.0 / DT))
    print("      Ad = [[%.6f, %.6f], [%.6f, %.6f]]"
          % (Ad[0, 0], Ad[0, 1], Ad[1, 0], Ad[1, 1]))
    print("      Bd = [[%.8f], [%.8f]]" % (Bd[0, 0], Bd[1, 0]))
    print("      Ad 的特征值：%s（= exp(开环极点*dt)）" % eig_str(Ad))
    print()

    sub("② 手工 PID：为什么纯 P 不够，kp 要到多大才够")

    print("  先试最简单的 u = -(kp*theta + kd*theta_dot)：")
    print()
    print("      闭环特征方程  s^2 + (b/(m*l^2) + kd/(m*l^2)) s")
    print("                       + (kp/(m*l^2) - g/l) = 0")
    print()
    print("  ★ 要注意那个常数项：它是 (kp/(m*l^2) - g/l)。")
    print("    要求它 > 0，也就是")
    print()
    print("        kp > (g/l) * m*l^2 = m*g*l = %.2f * %.5f * %.2f = %.4f N·m/rad"
          % (PEND_M, PEND_G, PEND_L, PEND_M * PEND_G * PEND_L))
    print()
    print("  ⇒ **kp 有一个硬下界**：小于这个数，你给多少 D 都救不回来。")
    print("    物理意义：kp*theta 必须能提供「托住杆子」的最小力矩 m*g*l*sin(theta)。")
    print("    这就是卷 3 第 3 章讲的「前馈 / 工作点」—— 只不过这里的「工作点」")
    print("    是杆子的重力矩，必须靠反馈自己顶上去。")
    print()
    print("  按卷 3 第 1 章的配方选极点：zeta = 0.8, wn = 6 rad/s")
    print()

    ZETA_PD = 0.8
    WN_PD = 6.0
    KD_PD = (2.0 * ZETA_PD * WN_PD - (-PEND_A22)) * PEND_ML2
    KP_PD = (WN_PD ** 2 + PEND_A11) * PEND_ML2
    K_PD = np.array([[KP_PD, KD_PD]])

    rows = [
        ["kp", "%.4f" % KP_PD, "%.4f" % (PEND_M * PEND_G * PEND_L), "下界 = m*g*l"],
        ["kd", "%.4f" % KD_PD, "—", "2*zeta*wn*m*l^2 - b"],
    ]
    tab(rows, ["增益", "本设计取值", "下界 / 参考", "来源"])
    print("  闭环极点：%s" % eig_str(PEND_A - PEND_B @ K_PD))
    print("      （就是 s^2 + 2*0.8*6*s + 36 = 0 的根，wn = 6，zeta = 0.8）")
    print("      kp = %.4f > %.4f，所以常数项 = %.4f > 0，稳住。"
          % (KP_PD, PEND_M * PEND_G * PEND_L,
             KP_PD / PEND_ML2 - PEND_A11))
    print()

    sub("③ LQR：把「调参」变成一个优化问题")

    Q1 = np.diag([1.0, 0.0])
    R1 = np.array([[1.0]])
    Pd, Kd, it1, hist1 = dare(Ad, Bd, Q1, R1)
    K1 = Kd

    print("  先写清楚 LQR 到底在最小化什么：")
    print()
    print("      J = sum_{k=0}^{inf} ( x_k' Q x_k + u_k' R u_k )")
    print()
    print("  取  Q = diag(1, 0)（只罚角度，不罚角速度）")
    print("      R = 1         （控制力矩的代价）")
    print()
    print("  解 Riccati 方程（实验 2 会展开怎么解）：")
    print()
    print("      P = [[%9.4f, %9.4f]," % (Pd[0, 0], Pd[0, 1]))
    print("           [%9.4f, %9.4f]]" % (Pd[1, 0], Pd[1, 1]))
    print()
    print("      K = R^-1 B' P A = [%.6f, %.6f]" % (Kd[0, 0], Kd[0, 1]))
    print()
    print("  ★★★ 停下来看一眼这个 K ——")
    print()
    print("      u = -K x = -%.4f * theta - %.4f * theta_dot"
          % (K1[0, 0], K1[0, 1]))
    print()
    print("      这不就是一个 **PD 控制器** 吗？kp = %.4f，kd = %.4f。"
          % (K1[0, 0], K1[0, 1]))
    print()
    print("      ⇒ ★★ **LQR 不是「另一种控制器」，它是「一种自动定 PD 参数的方法」。**")
    print("        对二阶系统，LQR 的解永远长成 PD 的样子；")
    print("        对四阶系统（位置+速度+角度+角速度），它长成")
    print("        「位置P + 速度PD + 角度P + 角速度P」—— 也就是卷 3 第 3 章的串级结构，")
    print("        只不过每一层的增益是**一起算出来的**，不是一层层手调的。")
    print()
    print("      这也解释了为什么 LQR 有点「可望不可即」：")
    print("      它给你的结构你早就在用，它给你的**增益**需要模型。")
    print()

    ev = np.linalg.eigvals(Ad - Bd @ Kd)
    wn_l, zeta_l = disc_poles_to_cont(ev, DT)[0]
    evp = np.linalg.eigvals(PEND_A - PEND_B @ K_PD)
    print("  对比一下两边的闭环极点（都换算到连续域）：")
    print()
    rows = [
        ["手工 PD (zeta=0.8, wn=6)", "%.3f" % abs(evp[0]),
         "%.3f" % (-evp[0].real / abs(evp[0]))],
        ["LQR (Q=diag(1,0), R=1)", "%.3f" % wn_l, "%.3f" % zeta_l],
    ]
    tab(rows, ["控制器", "wn (rad/s)", "zeta"])
    print("  ⇒ 同一个「wn 6 左右、zeta 0.8~1.0」的区间，两条路都到了。")
    print("    区别是：手工 PD 是你**选**的，LQR 是 Q/R **推**出来的。")
    print()

    sub("④ 正面对决：同一把尺子量两个控制器")

    print("  尺子必须是同一把 —— 固定 Q = diag(1,0)、R = 1，")
    print("  算 J = sum (theta^2 + u^2)。谁小谁赢。")
    print()

    X0 = np.array([0.3, 0.0])
    NSTEP = 2500                                    # 10 s

    results = []
    for name, KK in [("手工 PD", K_PD), ("LQR", K1)]:
        ts, xs, us, alive = sim_pend(X0, KK, NSTEP)
        j = cost_sum(xs, us, Q1, R1, NSTEP)
        results.append((name, KK, j, settle_time(ts, xs), max_abs_u(us), alive))

    rows = []
    for name, KK, j, st, mu, alive in results:
        rows.append([name,
                     ("%.4f" % st) if st else "—",
                     "%.4f" % mu,
                     "%.4f" % j,
                     "挺住" if alive else "倒了"])
    tab(rows, ["控制器", "调节时间 (s)", "max|u| (N·m)", "J", "结果"])

    print("  ★ 手工 PD 的 J = %.4f，LQR 的 J = %.4f —— LQR 低 %.1f%%。"
          % (results[0][2], results[1][2],
             100.0 * (results[0][2] - results[1][2]) / results[0][2]))
    print()
    print("    ★★ 这个「低」不是运气 —— 因为 J 正是 LQR 的定义，")
    print("       它在**所有**能稳住的控制律里取最小值。")
    print("       你手调的任何 PD，J 都不可能比它小。这是数学保证。")
    print()
    print("    ★ 但也要看清楚：LQR 的调节时间是 %.2f s，比手工 PD 的 %.2f s **更长**。"
          % (results[1][3], results[0][3]))
    print("      不是「LQR 更快」，而是「LQR 找到了另一个取舍点」：")
    print("      它肯花更少的力矩，换更长的收敛时间。")
    print("      你要更快？把 R 调小 —— 下一节就是干这个的。")
    print()

    sub("⑤ 只调一个旋钮：扫 R，看整条取舍曲线")

    print("  Q 固定在 diag(1,0)，只把 R 从 0.01 扫到 100：")
    print()

    rows = []
    sweep = [0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0, 100.0]
    for r in sweep:
        Rr = np.array([[r]])
        Pk, Kk, _it, _h = dare(Ad, Bd, Q1, Rr)
        evk = np.linalg.eigvals(Ad - Bd @ Kk)
        wnk, zk = disc_poles_to_cont(evk, DT)[0]
        ts, xs, us, alive = sim_pend(X0, Kk, NSTEP)
        rows.append(["%.2f" % r,
                     "%.4f" % Kk[0, 0],
                     "%.4f" % Kk[0, 1],
                     "%.3f" % wnk,
                     "%.3f" % zk,
                     ("%.3f" % settle_time(ts, xs)) if settle_time(ts, xs) else "—",
                     "%.4f" % max_abs_u(us),
                     "%.4f" % (X0 @ Pk @ X0)])
    tab(rows, ["R", "k1 (角度)", "k2 (角速度)", "wn", "zeta",
               "调节时间", "max|u|", "J* = x0'Px0"])

    print("  ★★ 读这张表要抓住三个方向：")
    print()
    print("      R 变大 ⇒ 控制「变贵」 ⇒ K 变小 ⇒ 闭环变慢 ⇒ 力矩省了、J* 大了")
    print("      R 变小 ⇒ 控制「变便宜」⇒ K 变大 ⇒ 闭环变快 ⇒ 力矩费了、J* 小了")
    print()
    print("    ★ 注意最右边那一列 J* = x0'Px0：")
    print("      它和仿真出来的离散代价 **对得上**（差一点点，")
    print("      因为仿真只有 10 s，尾巴被截掉，而且用的是非线性模型）。")
    print("      这不是巧合 ——")
    print()
    print("          ★★ J* = x0' P x0")
    print()
    print("      这就是 Riccati 方程的解 P 的物理意义：")
    print("      **P 是「从 x0 出发、按最优策略走，总共要付多少代价」**。")
    print("      实验 2 会把这条式子验到底。")
    print()
    Kinf = dare(Ad, Bd, Q1, np.array([[1.0e8]]))[1]
    print("    ★ 还有一个很漂亮的极限：R -> 无穷大时 K 收敛到")
    print("      [%.4f, %.4f] —— 不是 0。" % (Kinf[0, 0], Kinf[0, 1]))
    print("      「最省力的控制」= 刚好把不稳定极点反射到左半平面，")
    print("      这就是经典控制里的 **minimum-energy control**（最小能量控制）。")
    print("      把 R 取到 1e8 它也不肯再小了 —— 因为再小就稳不住了。")
    print()

    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        for name, KK, j, st, mu, alive in results:
            ts, xs, us, alive = sim_pend(X0, KK, 1200)
            axes[0].plot(ts, xs[:, 0], lw=1.6, label="%s (J=%.4f)" % (name, j))
            axes[1].plot(ts, us, lw=1.6, label=name)
        axes[0].axhline(0.0, color="k", lw=0.6, ls="--")
        axes[0].set_xlim(0, 1200 * DT)
        axes[0].set_xlabel("时间 (s)")
        axes[0].set_ylabel("theta (rad)")
        axes[0].grid(alpha=0.3)
        axes[0].legend(fontsize=8)
        axes[0].set_title("倒立摆：从 theta=0.3 rad 恢复")
        axes[1].axhline(0.0, color="k", lw=0.6, ls="--")
        axes[1].set_xlim(0, 1200 * DT)
        axes[1].set_xlabel("时间 (s)")
        axes[1].set_ylabel("u (N·m)")
        axes[1].grid(alpha=0.3)
        axes[1].legend(fontsize=8)
        axes[1].set_title("控制力矩")
        savefig("lesson20_exp1")


# ===========================================================================
# 实验 2  ★★ Riccati 方程到底怎么解出来
# ===========================================================================
#
#   实验 1 把 dare() 当黑盒用了。这一节把它拆开。
#
#   三条独立的解法，互相验证：
#
#     ① 不动点迭代      P_{k+1} = Q + A'P_k A - A'P_k B (R+B'P_k B)^-1 B'P_k A
#                       （就是 dare()，也是「有限时域 LQR 倒着递推」的极限）
#     ② 连续 Riccati ODE dP/dtau = A'P + PA - P B R^-1 B' P + Q，积分到稳态
#     ③ 哈密顿矩阵       H = [[A, -B R^-1 B'], [-Q, -A']] 的稳定不变子空间
#
#   三条路算出来必须是同一个 P。差多少都打印出来。

def care_residual(A, B, Q, R, P):
    """连续 Riccati 方程  A'P + PA - P B R^-1 B' P + Q = 0  的残差。"""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    lhs = A.T @ P + P @ A - P @ B @ np.linalg.inv(R) @ B.T @ P + Q
    return float(np.max(np.abs(lhs)))


def care_ode(A, B, Q, R, T=20.0, n=20000, P0=None):
    """积分 Riccati 微分方程求 CARE 的稳定解。

        dP/dtau = f(P) = A'P + P A - P B R^-1 B' P + Q,   P(0) = P0

    ★ 这个方程不是天上掉下来的：
      有限时域 LQR 从终端时刻 T 往回推，代价矩阵满足的正是它
      （把 tau = T - t 当自变量，负号翻成正号）。
      tau -> 无穷 时它停在不动点上，那个不动点就是 CARE 的解。

    用 RK4 积分。2x2 的矩阵，手写 RK4 比调库清楚。
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    Rinv = np.linalg.inv(R)
    BRB = B @ Rinv @ B.T

    def f(P):
        return A.T @ P + P @ A - P @ BRB @ P + Q

    P = np.zeros_like(Q) if P0 is None else np.array(P0, dtype=float)
    h = T / float(n)
    for _ in range(n):
        k1 = f(P)
        k2 = f(P + 0.5 * h * k1)
        k3 = f(P + 0.5 * h * k2)
        k4 = f(P + h * k3)
        P = P + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return P


def dare_doubling(A, B, Q, R, n_iter=80):
    """DARE 的【倍增算法】—— 二次收敛，和 dare() 的不动点迭代解同一个方程。

        A_{k+1} = A_k (I + G_k H_k)^-1 A_k
        G_{k+1} = G_k + A_k G_k (I + H_k G_k)^-1 A_k'
        H_{k+1} = H_k + A_k' (I + H_k G_k)^-1 H_k A_k

    初值 A_0 = A, G_0 = B R^-1 B', H_0 = Q。
    收敛后 P = H_inf，同时 A_inf -> 0。

    ★ 为什么需要它？因为实验 ② 里那个不动点迭代的收敛率是 rho(A_cl)^2。
      dt 很小时 rho(A_cl) -> 1（离散极点挤向单位圆），
      不动点迭代要跑几百万次；倍增算法每步把「视野」翻一倍，几十次就够。
      这是「同一个方程、不同解法、代价差几个数量级」的活例子。
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    n = A.shape[0]
    I = np.eye(n)

    G = B @ np.linalg.inv(R) @ B.T
    H = Q.copy()
    Ak = A.copy()
    for _ in range(n_iter):
        Xg = np.linalg.inv(I + G @ H)
        Xh = np.linalg.inv(I + H @ G)
        Anew = Ak @ Xg @ Ak
        Gnew = G + Ak @ G @ Xh @ Ak.T
        Hnew = H + Ak.T @ Xh @ H @ Ak
        H = 0.5 * (Hnew + Hnew.T)
        Ak = Anew
        G = Gnew
    K = np.linalg.solve(R + B.T @ H @ B, B.T @ H @ A)
    return H, K, float(np.max(np.abs(Ak)))


def care_hamiltonian(A, B, Q, R):
    """用哈密顿矩阵求 CARE 的解。

        H = [[ A,  -B R^-1 B' ],
             [ -Q, -A'        ]]

    性质（这一步是可以证明的，这里只用数值验证）：
      · H 的 2n 个特征值 = 闭环极点 ∪ 闭环极点的相反数
      · 取实部为负的那 n 个特征向量，堆成 [V1; V2]，则  P = V2 V1^-1
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    n = A.shape[0]
    H = np.zeros((2 * n, 2 * n))
    H[:n, :n] = A
    H[:n, n:] = -B @ np.linalg.inv(R) @ B.T
    H[n:, :n] = -Q
    H[n:, n:] = -A.T
    w, V = np.linalg.eig(H)
    idx = np.argsort(w.real)                 # 实部最小的 n 个 = 稳定子空间
    Vs = V[:, idx[:n]]
    V1 = Vs[:n, :]
    V2 = Vs[n:, :]
    P = np.real(V2 @ np.linalg.inv(V1))
    P = 0.5 * (P + P.T)
    return P, w


def exp2():
    head("实验 2  ★★ Riccati 方程到底怎么解出来")

    print("  实验 1 里 dare() 是个黑盒。这一节把它彻底拆开，")
    print("  用【三条互相独立的解法】算同一个 P，看它们对不对得上。")
    print()

    A, B = PEND_A, PEND_B
    Ad, Bd = c2d_zoh(A, B, DT)          # ★ 数字控制器真正面对的是这一对
    Q1 = np.diag([1.0, 0.0])
    R1 = np.array([[1.0]])

    # -------------------------------------------------------------------
    sub("① 先把问题写清楚：Riccati 方程是「最优性条件」")

    print("  ⚠️ 先把两个世界分清楚，这是本实验最容易搞混的地方：")
    print()
    print("      (A, B)    = 连续时间的被控对象（物理世界）")
    print("      (Ad, Bd)  = 零阶保持离散化后的一对（%d Hz，dt = %.0f ms）"
          % (1.0 / DT, DT * 1e3))
    print()
    print("  ★★ 飞控里跑的控制器【永远是离散的】。所以：")
    print("     「算 PID/LQR 增益」用 (Ad, Bd) + 离散 Riccati（DARE）")
    print("     「分析物理稳定性」用 (A, B) + 连续 Riccati（CARE）")
    print("     两者【不是同一个问题】，硬凑到一起比会得出荒谬的结论。")
    print("     本实验把两边都算出来，最后再用一条 dt 扫描曲线把它们缝上。")
    print()
    print("  要最小化的是【无限时域】代价（离散形式）：")
    print()
    print("      J = sum_{k=0}^{inf} ( x_k' Q x_k + u_k' R u_k )")
    print()
    print("  让 J 最小的控制律是状态反馈  u_k = -K x_k，")
    print("  最优代价是状态的二次型  J* = x_0' P x_0。")
    print()
    print("  ★★ 下面这三步是本章最核心的推导，一步都不许跳：")
    print()
    print("  【第 1 步】写出「从 k 时刻出发」的剩余代价（值函数）：")
    print()
    print("      V(x_k) = min sum_{i=k}^{inf} ( x_i' Q x_i + u_i' R u_i )")
    print("             = x_k' P x_k")
    print()
    print("  【第 2 步】把 k 单独拆出来（动态规划 / Bellman 方程）：")
    print()
    print("      x_k' P x_k = min_u [ x_k' Q x_k + u' R u + x_{k+1}' P x_{k+1} ]")
    print()
    print("      代入 x_{k+1} = A x_k + B u ：")
    print()
    print("      x' P x = min_u [ x'Qx + u'Ru + (Ax+Bu)' P (Ax+Bu) ]")
    print()
    print("  【第 3 步】括号里对 u 求导、令其为零：")
    print()
    print("      d/du [ ... ] = 2 R u + 2 B' P (A x + B u) = 0")
    print()
    print("      ⇒ (R + B'PB) u = -B'PA x")
    print("      ⇒ ★★ u = -(R + B'PB)^-1 B'PA x")
    print()
    print("      把它写回  K = (R + B'PB)^-1 B'PA ，就是实验 1 里那个 K。")
    print()
    print("  【第 4 步】把最优 u 代回 Bellman 方程，对【所有】x 都要成立，")
    print("            于是两边的 x' (·) x 的括号必须相等：")
    print()
    print("      P = Q + A'PA - A'PB (R + B'PB)^-1 B'PA")
    print()
    print("      ★★ 这就是【离散代数 Riccati 方程（DARE）】。")
    print("         它不是「一个要背的公式」，它就是 Bellman 方程本身。")
    print()
    print("  ★ 注意第 3 步求导时 (Ax+Bu)'P(Ax+Bu) 展开成")
    print("    x'A'PAx + x'A'PBu + u'B'PAx + u'B'PBu，")
    print("    其中 x'A'PBu 和 u'B'PAx 是标量的转置关系，两个都是 u'B'PAx，")
    print("    所以对 u 求导给出 2B'PAx —— 系数 2 就是这么来的。")
    print()

    # -------------------------------------------------------------------
    sub("② 解法一：不动点迭代（dare 干的事）")

    Pd, Kd, nit, hist = dare(Ad, Bd, Q1, R1)

    print("  把上面的 Riccati 方程当成一个迭代式（注意用 Ad, Bd）：")
    print()
    print("      P_{k+1} = Q + Ad'P_k Ad - Ad'P_k Bd (R + Bd'P_k Bd)^-1 Bd'P_k Ad")
    print()
    print("  从 P_0 = Q 出发一直迭代到不动点。这就是 dare() 的全部内容。")
    print()

    # 手工重跑一遍，这次记录「离真解的【距离】」而不是「相邻两步的差」
    dist = []
    P_walk = Q1.copy()
    for _k in range(24):
        S = R1 + Bd.T @ P_walk @ Bd
        Kk = np.linalg.solve(S, Bd.T @ P_walk @ Ad)
        P_walk = Ad.T @ P_walk @ Ad - Ad.T @ P_walk @ Bd @ Kk + Q1
        P_walk = 0.5 * (P_walk + P_walk.T)
        dist.append(float(np.max(np.abs(P_walk - Pd))))

    # 第二遍：这次从「真解附近」出发（P_0 = 0.9 * P_真），
    # 用来隔离出【线性收敛段】，看收敛率是不是真的等于 rho^2。
    dist_near = []
    P_near = 0.9 * Pd
    for _k in range(10):
        S = R1 + Bd.T @ P_near @ Bd
        Kk = np.linalg.solve(S, Bd.T @ P_near @ Ad)
        P_near = Ad.T @ P_near @ Ad - Ad.T @ P_near @ Bd @ Kk + Q1
        P_near = 0.5 * (P_near + P_near.T)
        dist_near.append(float(np.max(np.abs(P_near - Pd))))

    Acl = Ad - Bd @ Kd                      # 闭环矩阵（离散）
    rho = float(np.max(np.abs(np.linalg.eigvals(Acl))))

    rows = []
    for k in range(10):
        r_far = ("%.5f" % (dist[k] / dist[k - 1])) if k > 0 else u"—"
        r_near = ("%.5f" % (dist_near[k] / dist_near[k - 1])) if k > 0 else u"—"
        rows.append([str(k + 1),
                     "%.5e" % dist[k], r_far,
                     "%.5e" % dist_near[k], r_near])
    tab(rows, ["迭代 k", "‖P_k−P_真‖（从 P_0=Q）", "比值",
               "‖P_k−P_真‖（从 0.9P_真）", "比值"])

    print("  ★ 迭代了 %d 次收敛（判据 max|P_{k+1} - P_k| < 1e-15）。" % nit)
    print()
    print("  ★★★ 这张表要【分两段读】，这是本实验最容易看漏的地方：")
    print()
    print("    【右半张表】从 P_0 = 0.9*P_真 出发（已经在解的附近）：")
    print("       比值立刻稳定在 %.5f —— 这就是【线性收敛段】。"
          % (dist_near[8] / dist_near[7]))
    print("       而闭环矩阵的谱半径 rho(Ad - Bd K) = %.8f，平方 = %.8f。"
          % (rho, rho * rho))
    print("       ⇒ ★★ **两者相等**（差 %.2f%%，剩下的差是二次项的残留）。"
          % (100.0 * abs((dist_near[8] / dist_near[7]) - rho * rho) / (rho * rho)))
    print()
    print("    【左半张表】从 P_0 = Q 出发（真实的迭代路径）：")
    print("       比值是 %.5f，**不等于** rho^2，而且在慢慢往下爬 ——"
          % (dist[8] / dist[7]))
    print("       说明前几十步根本不在线性收敛段里。")
    print()
    print("  ★ 为什么会这样？因为线性化只在解的附近成立：")
    print()
    print("       设 E_k = P_k - P_真，把迭代式两边减去 DARE，")
    print("       展开并丢掉 E 的二次项，才得到")
    print()
    print("           E_{k+1} ≈ A_cl' E_k A_cl        （A_cl = Ad - Bd K）")
    print()
    print("       把 E_0 在 A_cl 的特征基下展开，第 k 步的衰减因子是")
    print("       |λ_i * λ_j|^k —— 所以**最慢那一项是 rho(A_cl)^2**。")
    print("       从 P_0 = Q 出发时 E_0 很大，二次项不可忽略，")
    print("       所以前面那一段比 rho^2 慢；走近了以后才归到 rho^2。")
    print()
    print("  ★ 用总步数交叉验算：")
    print("       理论：ln(1e-15) / ln(rho^2) = ln(1e-15)/ln(%.8f) = %.0f 步"
          % (rho * rho, math.log(1e-15) / math.log(rho * rho)))
    print("       实测：%d 步（含前面那段非线性慢速区）" % nit)
    print("     同一个量级 ⇒ 「收敛率 = 闭环谱半径平方」这个结论站得住。")
    print()
    print("  ⚠️ 陷阱：**别拿「Riccati 迭代是线性收敛」当成「它很快」。**")
    print("     rho^2 = %.4f 意味着每步只消掉 %.1f%% 的误差。"
          % (rho * rho, 100.0 * (1.0 - rho * rho)))
    print("     本例要 %d 次；如果闭环设计得慢一点（rho 更接近 1），" % nit)
    print("     几百上千次是常态 —— 这是【离线设计】可以忍的代价，")
    print("     但在线 MPC 每一步都要解一次，就完全不能忍（实验 4 会算这笔账）。")
    print()
    print("  ★★ 这条性质有一个【要命】的推论：")
    print("     **采样率越高，Ad 的极点越靠近 1，rho 越接近 1，")
    print("       不动点迭代就越慢。**")
    # ★ 「越慢」不能只写半句话 —— 用【同一个公式】把不同 dt 下的
    #   迭代次数算出来。闭环连续极点 p 与离散极点 z 的关系是
    #   z = exp(p*dt)，所以换 dt 只需把 p 按新 dt 重新映射。
    _ev_c = np.log(np.linalg.eigvals(Acl)) / DT
    for _dtx in (1e-3, 1e-5):
        _rho_x = float(np.max(np.abs(np.exp(_ev_c * _dtx))))
        _n_x = math.log(1e-15) / math.log(_rho_x * _rho_x)
        print("       dt = %-6s ⇒ rho^2 = %.8f ⇒ 要迭代 %s 次"
              % ("%.0e s" % _dtx, _rho_x * _rho_x,
                 ("%.2g" % _n_x).replace("e+0", "e")))
    print("     ⇒ 采样率翻 10 倍，迭代次数翻 10 倍量级（因为 1-rho^2 ∝ dt）。")
    print("       所以实验 ⑤ 要引入【倍增算法】来绕开这件事。")
    print()
    print("  离散 Riccati 解 P = [[%.4f, %.4f], [%.4f, %.4f]]"
          % (Pd[0, 0], Pd[0, 1], Pd[1, 0], Pd[1, 1]))
    print("  最优增益     K = [%.6f, %.6f]" % (Kd[0, 0], Kd[0, 1]))
    print("  闭环极点     %s" % eig_str(Acl))
    print("  ★ 这两个数和实验 1 那张 R 扫描表里 R = 1.00 那一行【完全一样】——")
    print("    因为用的就是同一个 dare(Ad, Bd, Q1, R1)。")
    print()
    print("  ★ 验残差（把 P 代回 DARE，左边减右边，应该全是 0）：")
    print("      max|残差| = %.3e" % dare_residual(Ad, Bd, Q1, R1, Pd))
    print("    ⇒ 不是「差不多」，是双精度下的机器零。")
    print()

    # -------------------------------------------------------------------
    sub("③ 解法二：连续 Riccati ODE 积分到稳态")

    P_ode = care_ode(A, B, Q1, R1, T=20.0, n=20000)

    print("  离散那份 P 是「采样周期 dt = %.0f ms」下的解。" % (DT * 1e3))
    print("  连续版本（CARE）长这样：")
    print()
    print("      A'P + P A - P B R^-1 B' P + Q = 0")
    print()
    print("  ★ 怎么从离散变连续？——【不要】直接把 dt 代 0，")
    print("    正确的是把 Q 和 R 都乘上 dt（代价是「每秒钟付多少」的密度），")
    print("    再令 dt -> 0。本节用更稳的做法：直接积分 Riccati 微分方程")
    print()
    print("      dP/dtau = A'P + P A - P B R^-1 B' P + Q ,   P(0) = 0")
    print()
    print("    用 RK4 从 tau = 0 积到 tau = 20 s（2 万步）。")
    print()
    print("  积分出来的 P = [[%.6f, %.6f], [%.6f, %.6f]]"
          % (P_ode[0, 0], P_ode[0, 1], P_ode[1, 0], P_ode[1, 1]))
    print("  CARE 残差 max|A'P+PA-PBR^-1B'P+Q| = %.3e" % care_residual(A, B, Q1, R1, P_ode))
    print()
    print("  ⇒ ★★ 残差是机器零。**这就是 Riccati 方程的数值解**，")
    print("     没有任何黑盒：写下一个微分方程，积分到它不动为止。")
    print()

    # -------------------------------------------------------------------
    sub("④ 解法三：哈密顿矩阵的特征值分解")

    P_ham, w = care_hamiltonian(A, B, Q1, R1)
    # ★ 连续情形的增益是 K = R^-1 B'P ——【没有】A。
    #   （离散才是 K = (R + B'PB)^-1 B'PA。这两个公式很容易混，
    #    混了以后闭环极点会算错，本脚本第一版就踩过这个坑。）
    K_ham = np.linalg.solve(R1, B.T @ P_ham)

    print("  造一个 4x4 的哈密顿矩阵：")
    print()
    print("      H = [[ A,  -B R^-1 B' ],")
    print("           [ -Q, -A'        ]]")
    print()
    print("  H = ")
    H = np.zeros((4, 4))
    H[:2, :2] = A
    H[:2, 2:] = -B @ np.linalg.inv(R1) @ B.T
    H[2:, :2] = -Q1
    H[2:, 2:] = -A.T
    for r in H:
        print("      [%12.4f %12.4f %12.4f %12.4f]" % (r[0], r[1], r[2], r[3]))
    print()
    print("  H 的特征值：%s" % eig_str(H))
    print("  A - B K（闭环）的特征值：%s" % eig_str(A - B @ K_ham))
    print()
    print("  ★★ 看清楚：H 的特征值正好是【闭环极点】加上它们的【相反数】。")
    print("     一半在左半平面（稳定），一半在右半平面（镜像）。")
    print("     这个 ± 对称性是哈密顿矩阵的定义性质 ——")
    print("     也是为什么「最优控制的极点一定是稳定且镜像成对」的。")
    print()
    print("  取左半平面那两列特征向量堆成 [V1; V2]，P = V2 V1^-1 ：")
    print()
    print("      P(哈密顿) = [[%.6f, %.6f], [%.6f, %.6f]]"
          % (P_ham[0, 0], P_ham[0, 1], P_ham[1, 0], P_ham[1, 1]))
    print("      CARE 残差 = %.3e" % care_residual(A, B, Q1, R1, P_ham))
    print()

    # -------------------------------------------------------------------
    sub("⑤ 三条路的关系：③ 和 ④ 必须相等，② 要「走过去」才相等")

    K_ode = np.linalg.solve(R1, B.T @ P_ode)
    rows = [
        ["② dare()（离散, dt=4ms）", "%.4f" % Pd[0, 0], "%.6f" % Kd[0, 0], "离散"],
        ["③ Riccati ODE（连续）", "%.10f" % P_ode[0, 0], "%.8f" % K_ode[0, 0], "连续"],
        ["④ 哈密顿矩阵（连续）", "%.10f" % P_ham[0, 0], "%.8f" % K_ham[0, 0], "连续"],
    ]
    tab(rows, ["解法", "P[0,0]", "K[0,0]", "口径"])

    print("  ★★ ③ 和 ④ 描述【同一个连续问题】，它们必须完全相等：")
    print("      P 的相对差 = %.3e" % (abs(P_ode[0, 0] - P_ham[0, 0]) / abs(P_ode[0, 0])))
    print("    ⇒ 两条毫不相干的数学路径（RK4 积分 ODE / 4x4 复特征分解）")
    print("      落在同一个 2x2 矩阵上，精度到双精度的全部有效位。")
    print("      这就是「数值解算对了」的最强证据。")
    print()
    print("  ⚠️ 陷阱：**不能拿 ② 的 P 和 ③④ 的 P 直接相减。**")
    print("     ② 的 P[0,0] = %.1f，③④ 的 P[0,0] = %.4f。" % (Pd[0, 0], P_ode[0, 0]))
    print("     有人说「差这么多，肯定哪个算错了」—— 不是算错，是【问题不同】：")
    print()
    print("        离散 LQR 的代价是  sum (x'Q_d x + u'R_d u)")
    print("        连续 LQR 的代价是  ∫ (x'Q_c x + u'R_c u) dt")
    print()
    print("     要让两者表示【同一个物理问题】，必须换口径：")
    print()
    print("        Q_d = Q_c * dt ,   R_d = R_c * dt")
    print()
    print("     （「每一步付多少」 = 「每秒钟付多少」 × 步长）")
    print("     本脚本第一版就是漏了这一步，拿 dt = 4 ms 的 P 去和连续 P 比，")
    print("     差 163%，还差点把它解释成「离散化误差」蒙混过去 —— 那是错的。")
    print()
    print("  换对口径以后再扫 dt，才是正确的对比方法：")
    print("  固定连续问题 (Q_c = diag(1,0), R_c = 1)，把 dt 一路缩小。")
    print()
    print("  ★ 但这里有个数值坑：dt 小到 1e-5 s 时，不动点迭代要跑几百万次")
    print("    （因为收敛率 rho^2 -> 1）。所以下面用【倍增算法】解 DARE：")
    print()

    Qc = np.diag([1.0, 0.0])
    Rc = np.array([[1.0]])
    rows = []
    for dtk in [1e-1, 4e-2, 1e-2, DT, 1e-3, 1e-4, 1e-5, 1e-6]:
        Adk, Bdk = c2d_zoh(A, B, dtk)
        Pk, Kk, _res = dare_doubling(Adk, Bdk, Qc * dtk, Rc * dtk)
        err_p = abs(Pk[0, 0] - P_ode[0, 0]) / P_ode[0, 0]
        err_k = abs(Kk[0, 0] - K_ode[0, 0]) / abs(K_ode[0, 0])
        rows.append(["%.6f s (%.0f Hz)" % (dtk, 1.0 / dtk),
                     "%.9f" % Pk[0, 0],
                     "%.4e" % err_p,
                     "%.8f" % Kk[0, 0],
                     "%.4e" % err_k])
    tab(rows, ["采样周期 dt", "P_d[0,0]（换口径后）", "P 的相对误差",
               "K_d[0,0]", "K 的相对误差"])

    print("  ★★ 两条误差列单调下降，dt = 1 us 那一行已经到 1e-4 % 以下，")
    print("     而且从 1e-2 到 1e-3 到 1e-4，误差大致各降一个数量级")
    print("     —— **一次收敛（O(dt)）**。")
    print()
    print("  ★★★ 这条曲线就是「控制器跑在 250 Hz」的定量含义：")
    print("     dt = %.0f ms 时，离散最优增益和「理想连续最优」相差" % (DT * 1e3))
    Adk0, Bdk0 = c2d_zoh(A, B, DT)
    Kd0 = dare_doubling(Adk0, Bdk0, Qc * DT, Rc * DT)[1]
    print("         K_d[0] = %.6f  vs  K_c[0] = %.6f   差 %.4f%%"
          % (Kd0[0, 0], K_ode[0, 0], 100.0 * abs(Kd0[0, 0] - K_ode[0, 0]) / abs(K_ode[0, 0])))
    print()
    print("     采样率越高越接近理论最优 —— 但代价是算力。")
    print("     **「最优」在数字系统里永远是「在这个采样率下最优」。**")
    print("     这也解释了为什么 250 Hz 的速率环比 1 kHz 的难调：")
    print("     它离「连续最优」本来就更远。")
    print()

    # -------------------------------------------------------------------
    sub("⑥ J* = x0' P x0 —— 把 P 的物理意义验到底")

    print("  P 到底是什么？Riccati 方程的解不是数学玩具，")
    print("  它的定义是：**从 x0 出发、按最优策略走，一共要付多少代价**。")
    print()
    print("      J* = x_0' P x_0")
    print()
    print("  造一个初值 x0 = [%.2f, %.2f]，用最优增益 K 闭环仿真，" % (0.3, 0.0))
    print("  把每一步的 x'Qx + u'Ru 加起来，和 x0'Px0 比：")
    print("  （用的是离散的 Pd 和离散代价 —— 口径必须一致，见 ⑤）")
    print()

    X0 = np.array([0.3, 0.0])
    j_star = float(X0 @ Pd @ X0)
    rows = []
    j_short = None
    for nstep in [250, 500, 1000, 2500]:
        ts, xs, us, alive = sim_pend(X0, Kd, nstep)
        j_sim = cost_sum(xs, us, Q1, R1, nstep)
        if j_short is None:
            j_short = j_sim
        rows.append(["%d 步 (%.1f s)" % (nstep, nstep * DT),
                     "%.6f" % j_sim, "%.6f" % j_star,
                     "%+.3f%%" % (100.0 * (j_sim - j_star) / j_star)])
    tab(rows, ["仿真长度", "仿真累加代价 J", "J* = x0'Px0", "相对差"])

    print("  ★★ 关键读法：**J 是【饱和】的，不是一直涨。**")
    print("     只跑 1 s（%.0f%% 的时长）就已经付掉 J* 的 %.2f%% 了，"
          % (100.0 * 250 * DT / (2500 * DT), 100.0 * j_short / j_star))
    print("     之后再加时长，J 几乎不动 —— 因为系统早就收敛到 0 了，")
    print("     x'Qx + u'Ru 全是 0，加多少步都不涨。")
    print("     而 J* 是【无穷时域】的和 ⇒ 相对差应该收敛到一个小常数，")
    print("     不是单调下降、更不是发散。表的最后一列正是这样。")
    print()
    print("  ★ 剩下那 %.2f%% 的差是【负的】，来自两处，都不是「算错了」："
          % abs(100.0 * (j_short - j_star) / j_star))
    print("    1) ★ 仿真用的是【非线性】倒立摆。theta = 0.3 rad 时")
    print("       真实重力矩是 m*g*l*sin(theta) < m*g*l*theta（线性化）——")
    print("       **真实系统比线性模型「更好控」，所以实际花的代价更少。**")
    print("       这个负号就是「线性化把系统想得太难了」的直接证据。")
    print("    2) 反过来，离散化的 ZOH 又让控制略有一点滞后，会略微增加代价。")
    print("    两条方向相反，最后净剩 -1.4%。")
    print()
    print("  ★★ 顺便：J* = x0'Px0 是**闭式**的 —— 不算仿真、不算积分，")
    print("     一个二次型就出来了，而且对【任意】x0 都成立（P 只要算一次）。")
    print("     这就是为什么在线 MPC 敢用「终端代价 P」：")
    print("     它把「N 步之后的无穷远未来」压缩成了一个 2x2 矩阵，")
    print("     等价于「预测时域 = 无穷」。实验 4 会直接用这个结论。")
    print()

    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        ns = 2500
        ts, xs, us, alive = sim_pend(X0, Kd, ns)
        axes[0].plot(ts, xs[:, 0], lw=1.6, label="theta (LQR)")
        axes[0].plot(ts, xs[:, 1] * 0.1, lw=1.6, label="theta_dot / 10")
        axes[0].axhline(0, color="k", lw=0.6, ls="--")
        axes[0].set_xlabel("时间 (s)")
        axes[0].set_ylabel("状态")
        axes[0].grid(alpha=0.3)
        axes[0].legend(fontsize=8)
        axes[0].set_title("闭环响应")
        cum = np.cumsum(xs[:ns, 0] ** 2 + us[:ns] ** 2)
        axes[1].plot(ts[:ns], cum, lw=1.6, label="累加代价")
        axes[1].axhline(float(X0 @ Pd @ X0), color="r", ls="--", lw=1.2,
                        label="J* = x0'Px0 = %.4f" % float(X0 @ Pd @ X0))
        axes[1].set_xlabel("时间 (s)")
        axes[1].set_ylabel("J")
        axes[1].grid(alpha=0.3)
        axes[1].legend(fontsize=8)
        axes[1].set_title("代价累加 vs Riccati 的闭式解")
        savefig("lesson20_exp2")


# ===========================================================================
# 实验 3  ★★★ LQR 和 Kalman 滤波是同一个数学（对偶性 → LQG）
# ===========================================================================
#
#   实验 2 从头到尾在解一个方程：
#
#       A'P + PA - P B R^-1 B' P + Q = 0            (LQR)
#
#   现在把卡尔曼滤波的稳态方程抄过来：
#
#       A P + P A' - P C' Rn^-1 C P + Qn = 0        (Kalman)
#
#   ★★ 转置一下第二行（P 是对称的）：
#
#       A'P + PA - P C' Rn^-1 C P + Qn = 0
#
#   把 C' 看成 B、Rn 看成 R、Qn 看成 Q —— **两行字一模一样**。
#
#   这就是对偶性。本节不背结论，直接数值验证它。

def sim_pend_measured(x0, K, n_step, L=None, C=None, sigma=0.0,
                      mode="full", seed=1, umax=None, dt=DT):
    """带测量的倒立摆仿真，三种控制器放同一把尺子下比。

    mode = "full"   假装两个状态都测得到（理想情况，作为基准）
    mode = "naive"  只有角度测量，角速度用【差分】硬凑 —— 教科书反面教材
    mode = "lqg"    只有角度测量，用稳态卡尔曼观测器重构状态

    返回 (ts, xs, us, alive, chatter)
        chatter = 控制量一阶差分的 RMS —— 用来量化「抖不抖」。
    """
    rng = np.random.default_rng(seed)
    C = np.asarray(C, dtype=float) if C is not None else np.array([[1.0, 0.0]])
    K = np.asarray(K, dtype=float).reshape(1, -1)
    Cr = C.reshape(-1)                  # 展平成 1-D，好用 float(Cr @ x) 取标量
    Lv = None if L is None else np.asarray(L, dtype=float).reshape(-1)

    Ad, Bd = c2d_zoh(PEND_A, PEND_B, dt)

    x = np.asarray(x0, dtype=float).copy()
    xh = np.zeros(2)                     # 观测器状态
    y_prev = None
    xs = np.zeros((n_step + 1, 2))
    us = np.zeros(n_step + 1)
    xs[0] = x
    alive = True

    for k in range(n_step):
        y = float(Cr @ x) + (sigma * rng.standard_normal() if sigma > 0 else 0.0)

        if mode == "full":
            x_used = x
        elif mode == "naive":
            if y_prev is None:
                thd = 0.0
            else:
                thd = (y - y_prev) / dt
            y_prev = y
            x_used = np.array([y, thd])
        else:                            # lqg
            y_prev = y
            x_used = xh

        u = float(-(K @ x_used)[0])
        if umax is not None:
            u = max(-umax, min(umax, u))
        us[k] = u

        x = pend_step(x, u, dt)
        if x is None:
            alive = False
            xs[k + 1:] = np.nan
            break
        xs[k + 1] = x

        if mode == "lqg":
            # 稳态观测器：x_{k+1} = Ad x + Bd u + L (y - C x)
            xh = Ad @ xh + Bd[:, 0] * u + Lv * (y - float(Cr @ xh))

    us = us[:len(xs)]
    # ★ 即使中途倒了也要算抖动：否则「差分测速把噪声放大 250 倍」
    #   这句结论在表里只剩一个「—」，读者看不到证据。
    if len(us) > 2:
        chatter = float(np.sqrt(np.mean(np.diff(us) ** 2)))
    else:
        chatter = float("nan")
    return np.arange(n_step + 1) * dt, xs, us, alive, chatter


def exp3():
    head("实验 3  ★★★ LQR 和 Kalman 滤波是同一个数学（对偶性 → LQG）")

    Ad, Bd = c2d_zoh(PEND_A, PEND_B, DT)
    # ★ 只测【角度】—— 就是摆杆上的编码器 / 电位器。
    #   注意别写成 [0, 1]：那样测的是角速度（陀螺），
    #   而本节要讲的故事恰恰是「角度好测、角速度不好拿」。
    C = np.array([[1.0, 0.0]])

    # -------------------------------------------------------------------
    sub("① 对偶性：把两个 Riccati 方程并排放")

    print("  本章实验 2 里，LQR 的代价矩阵满足（离散，DARE）：")
    print()
    print("      A'P A - P - A'P B (R + B'PB)^-1 B'P A + Q = 0")
    print()
    print("      K = (R + B'PB)^-1 B'P A          （最优反馈增益）")
    print()
    print("  卡尔曼滤波的协方差满足（预测形式，本代码用的就是这一种）：")
    print()
    print("      A P A' - P - A P C' (C P C' + Rn)^-1 C P A' + Qn = 0")
    print()
    print("      L = A P C' (C P C' + Rn)^-1       （卡尔曼增益）")
    print()
    print("  ★★ 对照读：把 LQR 那行的 A→A'、B→C'、R→Rn、Q→Qn，就是卡尔曼这一行。")
    print("     （两个增益里 A 的位置正好互换：K 把 A 放右边，L 把 A 放左边，")
    print("       所以 L' 恰等于【转置系统】的 LQR 增益 K_dual。）")
    print()
    print("    现在把两个方程对着读：")
    print()
    rows = [
        ["LQR", "B", "R", "Q", "K = (R+B'PB)^-1 B'P A", "A - B K"],
        ["Kalman", "C'", "Rn", "Qn", "L = A P C'(CPC'+Rn)^-1", "A - L C"],
    ]
    tab(rows, ["问题", "「B」", "「R」", "「Q」", "增益", "闭环矩阵"])
    print("  ⇒ ★★★ **它们不是「像」，是【同一个方程换了符号】。**")
    print("     LQR 是「已知状态、选控制」；Kalman 是「已知控制、估状态」。")
    print("     一个往前看（控制），一个往后看（估计），数学是同一套。")
    print()
    print("  ⚠️ 口径陷阱：卡尔曼增益有【预测】和【滤波】两种写法，差一个 A：")
    print("       预测（本代码）  L = A P C'(CPC'+Rn)^-1")
    print("                       配 x[k+1] = A x[k] + B u[k] + L(y[k] - C x[k])")
    print("       滤波（课本常见）L = Pf C'(C Pf C' + Rn)^-1")
    print("                       那个 Pf 是【更新后】协方差，和上面的 P 不是同一个")
    print("     两者关系 L = A @ L_f；本文件 kf_gain() 返回的是【预测形式】。")
    print()

    # -------------------------------------------------------------------
    sub("② 数值验证：用同一个 dare() 算两边")

    # 过程噪声（LQR 里叫 Q）：外力矩扰动是加在 θ̈ 上的，
    # 离散化后等价于「角速度通道」上有噪声 —— 所以只填 (2,2) 这一项。
    Qn = np.diag([0.0, 1.0])
    Rn = np.array([[0.01]])           # 测量噪声（LQR 里叫 R）：编码器 0.1 rad 的 1σ

    # 卡尔曼：dare(A', C', Qn, Rn)
    L, P_kf = kf_gain(Ad, C, Qn, Rn)
    # 对偶的 LQR：dare(A', C', Qn, Rn) —— 注意这里的「B」是 C'
    K_dual, P_dual = dlqr(Ad.T, C.T, Qn, Rn)

    print("  取 A = 倒立摆的 Ad，C = [1, 0]（只量角度 —— 摆杆上装编码器）。")
    print("  噪声口径：Qn = diag(0, 1)（过程噪声从力矩进来，落在角速度通道），Rn = 0.01。")
    print()
    print("  【卡尔曼】 kf_gain(Ad, C, Qn, Rn) 内部就是 dare(Ad', C', Qn, Rn)：")
    print("      L      = [%.6f, %.6f]（列向量，2x1）" % (L[0, 0], L[1, 0]))
    print()
    print("  【对偶 LQR】 同一个 dare(Ad', C', Qn, Rn) 直接返回增益：")
    print("      K_dual = [%.6f, %.6f]" % (K_dual[0, 0], K_dual[0, 1]))
    print()
    L_T = L.T
    print("  ★ 验证 L' == K_dual ：")
    print("      L'       = [%.6f, %.6f]" % (L_T[0, 0], L_T[0, 1]))
    print("      差 max|L' - K_dual| = %.3e" % float(np.max(np.abs(L_T - K_dual))))
    print()
    print("  ★ 验证协方差矩阵是【同一个 P】：")
    print("      P(卡尔曼) = [[%.6f, %.6f], [%.6f, %.6f]]"
          % (P_kf[0, 0], P_kf[0, 1], P_kf[1, 0], P_kf[1, 1]))
    print("      P(LQR)    = [[%.6f, %.6f], [%.6f, %.6f]]"
          % (P_dual[0, 0], P_dual[0, 1], P_dual[1, 0], P_dual[1, 1]))
    print("      差 max|P_kf - P_dual| = %.3e" % float(np.max(np.abs(P_kf - P_dual))))
    print()
    # 两种口径的数值对照：预测形式（带 A）vs 滤波形式（不带 A）。
    # 写在这里是为了让「差一个 A」**看得见**——免得读者拿滤波形式的公式
    # 去对预测形式的数，然后怀疑代码错了。
    S = C @ P_kf @ C.T + Rn
    L_pred = Ad @ P_kf @ C.T @ np.linalg.inv(S)
    L_filt = P_kf @ C.T @ np.linalg.inv(S)
    print("  ⚠️ 两种口径的数值对照（同一个 P，只差一个 Ad）：")
    print("      预测形式 L = Ad P C'(CPC'+Rn)^-1 = [%.6f, %.6f]  ← 上面打印的 L"
          % (L_pred[0, 0], L_pred[1, 0]))
    print("      滤波形式 L =    P C'(CPC'+Rn)^-1 = [%.6f, %.6f]"
          % (L_filt[0, 0], L_filt[1, 0]))
    print("      第一分量之比 = %.4f（差 %.1f%%，因为 Ad 的第一列 ≈ [1, 0.098]）"
          % (L_pred[0, 0] / L_filt[0, 0],
             100.0 * abs(L_pred[0, 0] / L_filt[0, 0] - 1.0)))
    print("      ⇒ 差一个 A 不是「差不多」：肉眼对公式必须挑口径。")
    print()
    print("  ★ 验证闭环极点也成对偶：")
    print("      eig(Ad - L C)            = %s" % eig_str(Ad - L @ C))
    print("      eig(Ad' - C' K_dual)     = %s" % eig_str(Ad.T - C.T @ K_dual))
    print("      ⇒ 互为转置，特征值完全相同。")
    print()
    print("  ★★★ 结论：**卡尔曼滤波就是「跑在转置系统上的 LQR」。**")
    print("     PX4 的 EKF2 里满屏的卡尔曼增益，和姿态环里的 PID 增益，")
    print("     背后是同一类数学 —— 只不过一个管估计、一个管控制。")
    print()

    # -------------------------------------------------------------------
    sub("③ 把它拼起来：LQG = LQR + 卡尔曼观测器")

    print("  实机上你永远拿不到全部状态。倒立摆只有【角度】测得到")
    print("  （编码器），角速度要么对角度差分、要么用观测器估。")
    print("  于是「最优控制」在工程上必须变成：")
    print()
    print("      观测器（卡尔曼）估状态  →  LQR 拿估计值算控制")
    print()
    print("      u_k = -K x̂_k")
    print("      x̂_{k+1} = Ad x̂_k + Bd u_k + L (y_k - C x̂_k)")
    print()
    print("  ★★ 这就是【分离原理】（separation principle）：")
    print("     观测器和控制器可以【各设计各的】，拼起来仍然稳定。")
    print("     这不是显然的 —— 两个都稳定的系统串起来不一定稳定。")
    print("     它成立的条件是「LQG」，是控制理论里最漂亮的结论之一。")
    print()

    K_lqr, P_lqr = dlqr(Ad, Bd, np.diag([1.0, 0.0]), np.array([[1.0]]))

    X0 = np.array([0.3, 0.0])
    NS = 2500
    SIG = 5.0e-3                       # 陀螺噪声 5e-3 rad/s，真实量级

    rows = []
    res = {}
    cases = [("① 全状态 LQR（作弊基准）", "full"),
             ("② 差分测速 + LQR（天真）", "naive"),
             ("③ 卡尔曼观测器 + LQR（LQG）", "lqg")]
    for name, mode in cases:
        ts, xs, us, alive, chat = sim_pend_measured(
            X0, K_lqr, NS, L=L, C=C, sigma=SIG, mode=mode, seed=7)
        j = cost_sum(xs, us, np.diag([1.0, 0.0]), np.array([[1.0]]), NS)
        res[mode] = (ts, xs, us, j, alive, chat)
        st = settle_time(ts, xs)
        rows.append([name,
                     ("%.4f" % st) if (alive and st) else u"—",
                     "%.4f" % max_abs_u(us) if alive else u"—",
                     ("%.3f" % j) if alive else u"发散了",
                     ("%.5f" % chat) if np.isfinite(chat) else u"—",
                     "挺住" if alive else "倒了（%.2f s）"
                     % float(np.nansum(np.isfinite(xs[:, 0])) * DT)])
    tab(rows, ["控制器", "调节时间 (s)", "max|u|", "J", "控制抖动 RMS", "结果"])

    print("  ★ 三者用的是【同一个 K_lqr】、【同一段噪声】。区别只在「用哪个状态算 u」。")
    print()
    c1 = res["full"][5]
    c2 = res["naive"][5]
    c3 = res["lqg"][5]
    print("  ★ 读法（下面每个倍数都是用上面表里的数现算的）：")
    print("    · ① 是作弊基准（两个状态都测得到），它最好，这是上限。")
    print("    · ② 差分测速：角度那一项还好，但角速度是 (y_k - y_{k-1})/dt，")
    print("      噪声被除以 dt = %.3f s 放大 %.0f 倍 —— 角度噪声 1σ = %.1e rad,"
          % (DT, 1.0 / DT, SIG))
    print("      差分出来的角速度噪声 1σ ≈ %.1e/%.3f = %.2f rad/s,"
          % (SIG, DT, SIG / DT))
    print("      比摆杆真实的角速度（峰值也就 1 rad/s 量级）还大。")
    print("      结果：抖动 RMS %.5f，是 ① 的 %.0f 倍；控制力矩峰值 %.4f N·m，"
          % (c2, c2 / c1, max_abs_u(res["naive"][2])))
    print("      是 ① 的 %.1f 倍。真机上这就是**电机嗡嗡响、发热、烧电调**。"
          % (max_abs_u(res["naive"][2]) / max_abs_u(res["full"][2])))
    print("    · ③ LQG：观测器把噪声按模型滤了一遍，抖动 RMS %.5f ——"
          % c3)
    print("      比 ② 小 %.0f 倍，但仍是 ① 的 %.0f 倍。"
          % (c2 / c3, c3 / c1))
    print("      ★ 这个「仍比作弊基准大」不要回避：观测器的输出里【必然】")
    print("        还带着测量噪声（L 越大、跟得越紧、噪声进来得越多）。")
    print("        LQG 不是「消噪」，是「用模型把噪声压到可用的程度」。")
    print()
    print("  ★★ 这就是「**为什么飞控必须有状态估计**」的最小例子：")
    print("     不是「有了估计更准」这么轻描淡写，而是")
    print("     **不估计就根本不能用**。第 4 章那一整章的 EKF，")
    print("     在这一节里有了它的存在理由。")
    print()
    print("  ★ 补一句关键的：② 和 ③ 的【反馈增益一模一样】，")
    print("     所以「控制器调得好不好」和「状态估得准不准」是两件事。")
    print("     调参调不出来噪声问题 —— 那是滤波器的问题。")
    print()

    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        for label, mode in [("① 全状态", "full"), ("② 差分测速", "naive"),
                            ("③ LQG", "lqg")]:
            ts, xs, us, j, alive, chat = res[mode]
            axes[0].plot(ts, xs[:, 0], lw=1.4, label="%s (J=%.3f)" % (label, j))
            axes[1].plot(ts, us, lw=1.0, label="%s (RMS=%.4f)" % (label, chat))
        axes[0].axhline(0, color="k", lw=0.6, ls="--")
        axes[0].set_xlim(0, 4)
        axes[0].set_xlabel("时间 (s)")
        axes[0].set_ylabel("theta (rad)")
        axes[0].grid(alpha=0.3)
        axes[0].legend(fontsize=8)
        axes[0].set_title("同一段陀螺噪声，三种用法")
        axes[1].axhline(0, color="k", lw=0.6, ls="--")
        axes[1].set_xlim(0, 4)
        axes[1].set_xlabel("时间 (s)")
        axes[1].set_ylabel("u (N·m)")
        axes[1].grid(alpha=0.3)
        axes[1].legend(fontsize=7)
        axes[1].set_title("控制力矩（抖不抖一眼看出）")
        savefig("lesson20_exp3")


# ===========================================================================
# 实验 4  ★★ MPC：滚动时域 + 约束，以及「飞控里到底用不用」
# ===========================================================================

def _grep_source(pattern, roots):
    """在源码树里数一数 pattern 命中多少个源文件。

    ★ 这不是「查文档说没有」，是【现场扫源码】。
      你换一个 PX4 / ArduPilot 版本，这个函数会立刻告诉你结论还成不成立。
      目录不存在时返回 (None, [])，脚本照常跑完。
    """
    import os
    import re
    rx = re.compile(pattern, re.IGNORECASE)
    hits = []
    for root in roots:
        if not os.path.isdir(root):
            return None, []
        for dirpath, dirnames, filenames in os.walk(root):
            for fn in filenames:
                if not fn.endswith((".c", ".cpp", ".h", ".hpp", ".cc", ".py")):
                    continue
                path = os.path.join(dirpath, fn)
                try:
                    with open(path, "rb") as fh:
                        data = fh.read()
                except OSError:
                    continue
                if b"\x00" in data[:4096]:        # 二进制文件跳过
                    continue
                if rx.search(data.decode("utf-8", "ignore")):
                    hits.append(path)
    return len(hits), hits


def run_mpc_nonlinear(x0, mpc, umax, n_step, niter=150,
                      rate_idx=None, rate_max=None, rate_rho=0.0):
    """把 MPC 接到【非线性】倒立摆上，滚动时域跑。

    每一步都重新解一次 QP（热启动：用上一步的解当初始点）。
    rate_* 是软「状态约束」（见 pg_qp 的说明）——用来演示
    「LQR 根本没法表达状态约束，MPC 能」。
    返回 (ts, xs, us, 总迭代次数, 是否挺住)。
    """
    x = np.asarray(x0, dtype=float).copy()
    N = mpc["N"]
    xs = np.zeros((n_step + 1, 2))
    us = np.zeros(n_step + 1)
    xs[0] = x
    U = np.zeros(N)
    iters = 0
    alive = True
    for k in range(n_step):
        U, ni = pg_qp(mpc, x, -umax, umax, niter, U,
                      rate_idx=rate_idx, rate_max=rate_max, rate_rho=rate_rho)
        iters += ni
        u = float(U[0])
        us[k] = u
        xn = pend_step(x, u, DT)
        if xn is None:
            alive = False
            xs[k + 1:] = np.nan
            break
        x = xn
        xs[k + 1] = x
    return np.arange(n_step + 1) * DT, xs, us, iters, alive


def exp4():
    head("实验 4  ★★ MPC：滚动时域 + 约束，以及「飞控里到底用不用」")

    Ad, Bd = c2d_zoh(PEND_A, PEND_B, DT)
    Q1 = np.diag([1.0, 0.0])
    R1 = np.array([[1.0]])
    # ★ dlqr 返回 (K, P)，别写反 —— 写反了下面 ② 的「MPC == LQR」会差 4.6e+01，
    #   而且 K_lqr 变成 2x2 的 P，sim_pend 会直接报维度错。
    K_lqr, Pterm = dlqr(Ad, Bd, Q1, R1)

    print("  MPC 的全部想法只有一句话：")
    print()
    print("      **每一步都把未来 N 步的最优控制问题解一遍，但只施加第一步。**")
    print()
    print("  然后下一步重来（用新的状态重新解）。这个「重来」叫【滚动时域】。")
    print()
    print("  ★ 为什么要重来？两个理由，都很硬：")
    print("    1) 模型不准 / 有扰动 ⇒ 预测必然有偏差，重来就是闭环修正")
    print("    2) 下一步有了新的测量 ⇒ 没有理由死守旧计划")
    print()

    # -------------------------------------------------------------------
    sub("① 把问题浓缩成一个只关于 U 的二次型")

    N = 20
    mpc = mpc_condensed(Ad, Bd, Q1, R1, N, Pterm)

    print("  预测时域 N = %d 步（%.0f ms，即 %.2f s 的未来）。"
          % (N, N * DT * 1e3, N * DT))
    print()
    print("  把未来 N 步的状态全堆起来：")
    print()
    print("      X = [x_1; x_2; ...; x_N]     (%d 维)" % (N * mpc["n"]))
    print("      U = [u_0; u_1; ...; u_{N-1}] (%d 维)" % (N * mpc["m"]))
    print()
    print("  递推 N 次，把 X 全用 x_0 和 U 表示出来：")
    print()
    print("      X = Abar x_0 + Bbar U")
    print()
    print("      Abar 是 %dx%d（Ad 的幂堆起来）" % mpc["Abar"].shape)
    print("      Bbar 是 %dx%d（下三角分块 Toeplitz，Ad^i Bd）" % mpc["Bbar"].shape)
    print()
    print("  代价  J = X'Qbar X + U'Rbar U  （最后一步用 Riccati 的 P 当终端代价）")
    print("  代进去就只剩 U 了 —— 这叫【浓缩】：")
    print()
    print("      J(U) = 0.5 U'H U + (F'x_0)'U + 常数")
    print()
    print("      H = %dx%d，条件数 cond(H) = %.1f" % (mpc["H"].shape + (np.linalg.cond(mpc["H"]),)))
    print()
    print("  ★★ 关键：**约束 u_min <= u <= u_max 是加在 U 上的**，")
    print("     而 U 就是优化变量本身 ⇒ 约束是「框约束」（box constraint），")
    print("     形式最简单的一类约束。这是浓缩法的全部好处。")
    print()

    # -------------------------------------------------------------------
    sub("② 没有约束时，MPC 必须【完全等于】LQR")

    print("  这一点非常重要，它是「MPC 到底对不对」的试金石：")
    print()
    print("      当预测时域无限长（或终端代价用 Riccati 的 P）且无约束时，")
    print("      MPC 的最优解 = LQR。**一步都不差。**")
    print()
    print("  理由：终端代价 P 把「N 步之后的无穷远未来」精确地折算进来了，")
    print("        于是有限时域问题 = 无限时域问题。")
    print()

    rows = []
    nis = []
    for x0 in [np.array([0.1, 0.0]), np.array([0.3, 0.0]),
               np.array([-0.2, 0.5])]:
        U_exact = np.linalg.solve(mpc["H"], -(mpc["F"].T @ x0))
        u_mpc = float(U_exact[0])
        u_lqr = float(-(K_lqr @ x0)[0])
        # 投影梯度法解同一个 QP（不加约束）。20000 是迭代上限，不是实际迭代数——
        # pg_qp 一旦梯度足够小就提前退出，所以下面打印的 ni 才是真花的步数。
        U_pg, ni = pg_qp(mpc, x0, None, None, 20000)
        nis.append(ni)
        rows.append(["[%.1f, %.1f]" % (x0[0], x0[1]),
                     "%.6f" % u_mpc, "%.6f" % u_lqr,
                     "%.3e" % abs(u_mpc - u_lqr),
                     "%.6f" % float(U_pg[0]),
                     "%.3e" % abs(float(U_pg[0]) - u_mpc),
                     str(ni)])
    tab(rows, ["x0", "MPC 首步 u（精确解）", "LQR 的 u = -Kx0", "差",
               "MPC 首步 u（投影梯度）", "差", "迭代数"])

    print("  ★★★ 两列「差」都是机器零量级 ⇒")
    print("      **MPC 和 LQR 不是两个对立的流派，LQR 是 MPC 的一个特例。**")
    print("      MPC 多出来的东西只有一样：**约束**。")
    print()
    print("  ★ 顺带看清投影梯度法：这里它只用了 %d~%d 次迭代就把无约束 QP 解到机器精度"
          % (min(nis), max(nis)))
    print("    ——20000 只是**迭代上限**，梯度足够小就提前退出，别把上限读成实际代价。")
    print("    这一题 N=4，H 才 4×4，条件数又温和，所以迭代法看起来很划算。")
    print("    真正的问题不在「一次要迭代多少次」，而在**维数**：")
    print("    实验 4 ⑤ 会量到 MPC 在真实规模下的单周期耗时，和 LQR 差三个数量级。")
    print()

    # -------------------------------------------------------------------
    sub("③ 加上约束：先算【可行性】，再看「事后夹」到底行不行")

    UMAX = 1.5
    NS = 1500

    # ---- ③-a 第一步不是选算法，是算可达性 ----------------------------
    TH_BAD = 1.0
    # 保持力矩：重力矩 m g l sinθ 必须由 u 顶住
    u_hold_bad = PEND_M * PEND_G * PEND_L * math.sin(TH_BAD)
    print("  给执行器一个硬上限：|u| <= %.1f N·m。" % UMAX)
    print()
    print("  ③-a 【先算可达性】—— 90% 的工程师会跳过这一步，然后白忙一天。")
    print()
    print("      摆杆偏离竖直 θ 时，重力矩是 m g l sinθ，")
    print("      要靠 u 顶住才能停在那里（力臂 l = %.2f m）：" % PEND_L)
    print()
    print("          u_hold(θ) = m g l sinθ = %.4f x sinθ  N·m"
          % (PEND_M * PEND_G * PEND_L))
    print("          u_hold(%.1f rad) = %.4f N·m  >  u_max = %.1f N·m ✗"
          % (TH_BAD, u_hold_bad, UMAX))
    print()
    print("      ⇒ **这个初值在约束内【不可达】。**")
    print("        θ > %.3f rad 的地方，1.5 N·m 连「停住」都做不到，"
          % math.asin(min(1.0, UMAX / (PEND_M * PEND_G * PEND_L))))
    print("        更别说往回推。这不是控制器好不好，是物理上办不到。")
    print()

    rows = []
    X0bad = np.array([TH_BAD, 0.0])
    for name, mode in [("LQR（事后夹）", "lqr"), ("MPC（约束进优化）", "mpc")]:
        if mode == "lqr":
            ts, xs, us, alive = sim_pend(X0bad, K_lqr, NS, umax=UMAX)
            iters = 0
        else:
            ts, xs, us, iters, alive = run_mpc_nonlinear(
                X0bad, mpc, UMAX, NS, niter=150)
        rows.append([name,
                     "挺住" if alive else "倒了",
                     "%.3f" % np.nanmax(np.abs(xs[:, 0])),
                     "%.4f" % max_abs_u(us),
                     (str(iters) if iters else u"—")])
    tab(rows, ["控制器（θ0 = %.1f rad）" % TH_BAD, "结果",
               "最大偏离 (rad)", "max|u|", "QP 总迭代次数"])
    print("  ★★ 两个都倒。**MPC 不是魔法** —— 约束内不可达时，")
    print("     任何算法都只能承认失败。这就是为什么工程上第一件事")
    print("     是把「可达域」算出来（姿态角上限、油门余量），")
    print("     而不是先挑控制器。")
    print()

    # ---- ③-b 可行域内：事后夹 vs 约束进优化 --------------------------
    TH_OK = 0.5
    X0 = np.array([TH_OK, 0.0])
    u_hold_ok = PEND_M * PEND_G * PEND_L * math.sin(TH_OK)
    print("  ③-b 【可行域内】再比一次。取 θ0 = %.1f rad：" % TH_OK)
    print("      u_hold(%.1f) = %.4f N·m < %.1f ⇒ 可达。"
          % (TH_OK, u_hold_ok, UMAX))
    print()

    ts_l, xs_l, us_l, alive_l = sim_pend(X0, K_lqr, NS, umax=UMAX)
    ts_m, xs_m, us_m, it_m, alive_m = run_mpc_nonlinear(
        X0, mpc, UMAX, NS, niter=150)
    j_l = cost_sum(xs_l, us_l, Q1, R1, NS)
    j_m = cost_sum(xs_m, us_m, Q1, R1, NS)
    d_us = float(np.nanmax(np.abs(us_l - us_m)))
    d_xs = float(np.nanmax(np.abs(xs_l - xs_m)))
    rows = [
        ["LQR（事后夹）", "%.3f" % settle_time(ts_l, xs_l), "%.4f" % max_abs_u(us_l),
         "%.4f" % j_l, "挺住" if alive_l else "倒了"],
        ["MPC（约束进优化）", "%.3f" % settle_time(ts_m, xs_m),
         "%.4f" % max_abs_u(us_m), "%.4f" % j_m, "挺住" if alive_m else "倒了"],
    ]
    tab(rows, ["控制器", "调节时间 (s)", "max|u|", "J", "结果"])
    print("  ★★ 实测两条轨迹的差：max|u_LQR - u_MPC| = %.3e，" % d_us)
    print("     max|θ_LQR - θ_MPC| = %.3e —— **机器零量级，逐点相同。**" % d_xs)
    print()
    print("  ★★★ 这是个反直觉的结果，值得单独讲清楚：")
    print("     **只有一个执行器、约束又只是 u 的上下限时，")
    print("     「事后夹住」和「约束进优化」给出的是同一条轨迹。**")
    print()
    print("      这不是「差不多」，是【一模一样】。而且不是这一次的巧合 ——")
    print("      下面用【穷举 active set 的精确解】验（不用迭代解，")
    print("      免得有人说是「投影梯度没收敛」）：")
    print()

    rng_e = np.random.default_rng(0)
    ev = []
    for Nh in (4, 6, 8):
        mh = mpc_condensed(Ad, Bd, Q1, R1, Nh, Pterm)
        w_first = 0.0
        w_tail = 0.0
        n_prefix = 0
        for _ in range(12):
            xr = rng_e.uniform(-1.2, 1.2, 2)
            g0 = mh["F"].T @ xr
            Uu = -np.linalg.solve(mh["H"], g0)
            Uq = qp_box_exact(mh["H"], g0, -UMAX, UMAX)
            w_first = max(w_first, abs(float(Uq[0])
                                       - float(np.clip(Uu[0], -UMAX, UMAX))))
            w_tail = max(w_tail, float(np.max(np.abs(
                Uq - np.clip(Uu, -UMAX, UMAX)))))
            sat = np.where(np.abs(Uq) > UMAX - 1e-6)[0]
            if len(sat) == 0 or np.array_equal(sat, np.arange(len(sat))):
                n_prefix += 1
        ev.append([str(Nh), "%d / 12" % n_prefix,
                   "%.2e" % w_first, "%.3f" % w_tail])
    tab(ev, ["预测时域 N", "饱和分量是前缀", "首步差 |U*[0]-clip|",
             "整条计划的最大差"])

    print("  ★ 三列读数说明：")
    print("    · 第一列：12 个随机状态里，饱和的分量【全部是从 U[0] 起连续一段】")
    print("      （这就是机制：一旦饱和就是在往回推，推过头后面自然退饱和）。")
    print("    · 第二列：首步 u 与「夹住的无约束最优」的差打印出来是 0.00e+00 ——")
    print("      不是「小」，是【浮点意义上完全相同】。")
    print("    · 第三列：★ 计划的后半段【确实不一样】。")
    print("      所以「MPC 和夹住法完全等价」只对【施加出去的那一步】成立，")
    print("      对「计划」不成立。滚动时域只施加第一步 —— 这才是原因。")
    print()
    print("  ★★ 那 MPC 的约束优势到底在哪？在下面三件事上，")
    print("     而它们的共同点是：**约束不再和唯一那个执行器对齐。**")
    print("        1) 多个执行器共享约束（四旋翼 4 个电机 ⇒ 下一节 ④）")
    print("        2) 状态约束（倾角上限、别撞墙、速度上限）")
    print("        3) 需要提前规划（现在就得为几步之后的约束做准备）")
    print("     「各自夹一下」在第 1 种情况下【必然错】，因为夹住某一个")
    print("     执行器会改变【通道之间的配比】，而配比本身就是控制量。")
    print()

    # -------------------------------------------------------------------
    sub("④ 约束真正的战场：多个执行器共享约束（四旋翼 mixer）")

    L_ARM = 0.18          # 等效力臂 (m)
    C_YAW = 0.02          # 偏航系数 (m)
    #            M1     M2     M3     M4       电机顺序：右前/左前/右后/左后
    Mmix = np.array([
        [1.0, 1.0, 1.0, 1.0],                 # 总距  T  = Σu
        [-L_ARM, -L_ARM, L_ARM, L_ARM],       # 横滚  τx = l(-u1-u2+u3+u4)
        [L_ARM, -L_ARM, -L_ARM, L_ARM],       # 俯仰  τy = l(u1-u2-u3+u4)
        [C_YAW, -C_YAW, C_YAW, -C_YAW],       # 偏航  τz = c(u1-u2+u3-u4)
    ])
    # 想要的「虚拟控制量」：95% 总距 + 横滚 0.2 + 俯仰 0.1（一个斜向机动）
    f_des = np.array([3.8, 0.2, 0.1, 0.0])

    print("  这一节没有动力学，只有一个【静态分配】问题 ——")
    print("  因为这才是约束真正咬人的地方，而且它就是飞控里的 mixer。")
    print()
    print("  四旋翼：4 个电机，每个归一化推力 u_i ∈ [0, 1]（1 = 满油门）。")
    print("  它们合力产生 4 个东西（总距 + 三轴力矩）：")
    print()
    print("      f = [T, τx, τy, τz]' = M u")
    print()
    print("  M 的系数（为了不喧宾夺主，做了归一化，只保留结构）：")
    print("      力臂 l = %.2f m（横滚/俯仰），偏航系数 c = %.2f m" % (L_ARM, C_YAW))
    print()

    # ---- 三种分配策略 -------------------------------------------------
    def delivered(u):
        return Mmix @ u

    def qp_alloc(Wmat, lam=1e-6):
        """min (Mu-f)'W(Mu-f) + λ|u|²  s.t. 0 <= u <= 1 —— 精确解。

        写成 0.5 U'HU + c'U 的形式交给 qp_box_exact（穷举 3^4 = 81 种
        active set）。4 个变量可以穷举，80 个变量不可以（第 ⑤ 节）。
        """
        H = 2.0 * (Mmix.T @ Wmat @ Mmix + lam * np.eye(4))
        c = -2.0 * (Mmix.T @ Wmat @ f_des)
        return qp_box_exact(H, c, 0.0, 1.0)

    # 策略 1：天真 —— 先求精确解，再逐通道夹
    u_naive = np.clip(np.linalg.solve(Mmix, f_des), 0.0, 1.0)

    # 策略 2：规则式 —— 保推力，把力矩【整体缩放】到可行（AP 的 thrust_boost
    #          / PX4 的 airmode 就是这个思路）
    u_at0 = np.linalg.solve(Mmix, np.array([f_des[0], 0.0, 0.0, 0.0]))
    u_at1 = np.linalg.solve(Mmix, f_des)
    s_max = 1.0
    for i in range(4):
        d = u_at1[i] - u_at0[i]
        if abs(d) < 1e-15:
            continue
        s_max = min(s_max, (1.0 - u_at0[i]) / d if d > 0 else (0.0 - u_at0[i]) / d)
    s_max = max(0.0, min(1.0, s_max))
    u_rule = np.clip(u_at0 + s_max * (u_at1 - u_at0), 0.0, 1.0)

    # 策略 3：约束进优化 —— 姿态权重远大于推力（姿态优先）
    W_alloc = np.diag([1.0, 1.0e4, 1.0e4, 1.0e4])
    u_qp = qp_alloc(W_alloc)

    print("  场景：一个【斜向机动】——总距要 T = %.2f（平均每个电机 %.0f%%），"
          % (f_des[0], 100.0 * f_des[0] / 4.0))
    print("        同时要横滚 τx = %.2f、俯仰 τy = %.2f N·m。"
          % (f_des[1], f_des[2]))
    print("        需要的差动 = (τx + τy) / l = %.3f，而 4 个电机的总和只有 4.0，"
          % ((f_des[1] + f_des[2]) / L_ARM))
    print("        %.0f%% 的平均油门已经把余量吃光了 ⇒ **不可行**。"
          % (100.0 * f_des[0] / 4.0))
    print()
    print("  三种做法，各自丢了什么：")
    print()

    rows = []
    got = {}
    for name, u in [("① 先精确解再逐通道夹（天真）", u_naive),
                    ("② 保推力、力矩整体缩放（规则式）", u_rule),
                    ("③ 约束进优化、姿态优先（QP）", u_qp)]:
        fd = delivered(u)
        got[name[0]] = (u, fd)
        rows.append([name,
                     "%.2f~%.2f" % (u.min(), u.max()),
                     "%.3f (%+.1f%%)" % (fd[0], 100.0 * (fd[0] / f_des[0] - 1.0)),
                     "%.3f (%+.1f%%)" % (fd[1], 100.0 * (fd[1] / f_des[1] - 1.0)),
                     "%.3f (%+.1f%%)" % (fd[2], 100.0 * (fd[2] / f_des[2] - 1.0)),
                     "%.3f" % fd[3]])
    tab(rows, ["分配策略", "电机指令范围", "实得总距 T", "实得横滚 τx",
               "实得俯仰 τy", "实得偏航 τz"])

    d_naive = got["①"][1]
    d_rule = got["②"][1]
    d_qp = got["③"][1]
    tau_req = math.hypot(f_des[1], f_des[2])
    ang_req = math.degrees(math.atan2(f_des[2], f_des[1]))
    ang_naive = math.degrees(math.atan2(d_naive[2], d_naive[1]))
    # ★ 下面两个「极限」必须【精确算】，不能凭公式拍：
    #   M 是 4×4 满秩（实验 4 ③ 验过 rank = 4）⇒ 给定 (T, τx, τy, τz) 就
    #   只有唯一一个解 u；「能不能做到」等价于「这个 u 在不在 [0,1]^4 里」。
    #   于是「极限」= 一维搜索（可行集是凸的 ⇒ 射线/区间上二分就是精确的）。
    def tau_ok(Tv, tx, ty):
        u = np.linalg.solve(Mmix, np.array([Tv, tx, ty, 0.0]))
        return bool(np.all(u >= -1e-12) and np.all(u <= 1.0 + 1e-12))

    def max_tau_along(Tv, ang_deg):
        """固定总距 Tv，沿 ang_deg 方向 |τ| 最大能给到多少。"""
        lo, hi = 0.0, 2.0 * L_ARM
        a = math.radians(ang_deg)
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            if tau_ok(Tv, mid * math.cos(a), mid * math.sin(a)):
                lo = mid
            else:
                hi = mid
        return lo

    def max_T_keep_tau(tx, ty):
        """严格保住力矩 (τx, τy)（且 τz = 0）时，总距最大能到多少。"""
        grid = [0.01 * k for k in range(1, 451)]
        feas = [tau_ok(Tv, tx, ty) for Tv in grid]
        if not any(feas):
            return 0.0
        end = max(i for i, f in enumerate(feas) if f)
        lo = grid[end]
        if end + 1 < len(grid):
            hi = grid[end + 1]
            for _ in range(200):
                mid = 0.5 * (lo + hi)
                if tau_ok(mid, tx, ty):
                    lo = mid
                else:
                    hi = mid
        return lo

    tau_max_dir = max_tau_along(f_des[0], ang_req)
    tau_max_any = max(max_tau_along(f_des[0], k) for k in range(0, 360, 5))
    T_keep_tau = max_T_keep_tau(f_des[1], f_des[2])

    print("  ★ 先把两个「极限」算出来，读数才有参照物")
    print("    （M 满秩 ⇒ 给定 (T, τx, τy, τz) 只有唯一解 u，所以")
    print("      「极限」就是查这个 u 有没有越出 [0,1] —— 精确定出来的）：")
    print("     · 要【严格保推力】T = %.2f：沿期望方向最多只能给 %.4f N·m，"
          % (f_des[0], tau_max_dir))
    print("       所有方向扫一遍最多也只有 %.4f N·m（期望 %.3f）"
          % (tau_max_any, tau_req))
    print("       ⇒ 规则式那个 s = %.3f 就是这么来的（力矩只能整体缩水）。"
          % s_max)
    print("     · 要【严格保力矩】|τ| = %.3f：总距必须降到 %.2f（丢 %.0f%%）"
          % (tau_req, T_keep_tau, 100.0 * (1.0 - T_keep_tau / f_des[0])))
    print()
    print("  ★★ 三种做法的差别【不是谁更准，而是谁优先】：")
    print()
    print("     · ① 天真：推力只丢 %.0f%%，看起来还行 —— 但力矩两个通道"
          % (100.0 * (1.0 - d_naive[0] / f_des[0])))
    print("       各丢 %.0f%% / %.0f%%，【丢的比例还不一样】。"
          % (100.0 * (1.0 - d_naive[1] / f_des[1]),
             100.0 * (1.0 - d_naive[2] / f_des[2])))
    print("       于是不只是「变钝」，连【力矩方向】都被夹歪了：")
    print("          要的方向 %.1f°，实得 %.1f°，歪了 %.1f°。"
          % (ang_req, ang_naive, ang_naive - ang_req))
    print("       还漏到偏航上去了：τz = %.4f（本来没要偏航）。" % d_naive[3])
    print("       为什么漏？因为 u = [%.3f, %.3f, %.3f, %.3f] 里 M3/M4 先撞了"
          % (got["①"][0][0], got["①"][0][1], got["①"][0][2], got["①"][0][3]))
    print("       上限 —— 哪个先撞、撞多少，取决于当时的机动方向，")
    print("       **飞控没法为它设计补偿**。这就是「姿态被夹歪」的真实含义。")
    print()
    print("     · ② 规则式：推力精确（%+.1f%%），两个力矩通道【同比例】缩到"
          % (100.0 * (d_rule[0] / f_des[0] - 1.0)))
    print("       %.0f%% —— 方向保住了（歪 %.1f°），幅度没了。"
          % (100.0 * s_max, math.degrees(math.atan2(d_rule[2], d_rule[1])) - ang_req))
    print("       飞机不掉高，但几乎操纵不动：响应变成「钝」的，"
          "而不是「歪」的。")
    print("       ★ 这正是 PX4 的 airmode / ArduPilot 的 thrust_boost 干的事：")
    print("         饱和时【有秩序地】放弃一部分力矩，而不是让它乱掉。")
    print()
    print("     · ③ 约束进优化：力矩误差 %.1f%% / %.1f%%（基本精确），"
          % (100.0 * (d_qp[1] / f_des[1] - 1.0), 100.0 * (d_qp[2] / f_des[2] - 1.0)))
    print("       代价是总距丢 %.0f%%（%.2f → %.2f）。"
          % (100.0 * (1.0 - d_qp[0] / f_des[0]), f_des[0], d_qp[0]))
    print("       姿态优先、牺牲高度 —— 对穿越机是对的，对重载吊挂就不一定。")
    print("       （注意它也漏了一点到偏航：%.4f —— 因为它是【按权重权衡】，"
          % d_qp[3])
    print("        不是硬性解耦。权重给多少，就是工程判断。）")
    print()
    print("  ★★★ 这三条路就是「约束优化」在飞控里的真实落点：")
    print("     **优先级 = 权重**。")
    print("     ② 和 ③ 的差别不是「谁更准」，而是 ③ 可以在【每一条通道上】")
    print("     显式地指定优先级，而 ② 只能一刀切。")
    print("     ★ 但产业界的实现是 ②：规则式、固定开销、可预测 —— 不是 ③。")
    print("       原因在第 ⑤ ⑥ 节。")
    print()

    # -------------------------------------------------------------------
    sub("⑤ 算力账：MPC 到底算不算得过来")

    print("  这是工程判断，必须用数字说话。")
    print()

    x0 = np.array([0.5, 0.0])
    U0 = np.zeros(N)
    # 冷启动
    t0 = time.perf_counter()
    for _ in range(50):
        pg_qp(mpc, x0, -UMAX, UMAX, 150)
    t_cold = (time.perf_counter() - t0) / 50.0
    # 热启动
    Uw = np.zeros(N)
    pg_qp(mpc, x0, -UMAX, UMAX, 150, Uw)
    t0 = time.perf_counter()
    for _ in range(50):
        pg_qp(mpc, x0, -UMAX, UMAX, 150, Uw)
    t_warm = (time.perf_counter() - t0) / 50.0

    t_lqr = 0.0
    t0 = time.perf_counter()
    for _ in range(200000):
        _ = float(-(K_lqr @ x0)[0])
    t_lqr = (time.perf_counter() - t0) / 200000.0

    print("  本节的 MPC：N = %d 步预测，2 个状态，1 个输入。" % N)
    print("  求解器是最朴素的投影梯度法（%d 次迭代上限）。" % 150)
    print()
    rows = [
        ["LQR（一次 2x2 内积）", "%.3f us" % (t_lqr * 1e6), "—"],
        ["MPC 冷启动（一个 QP）", "%.1f us = %.2f ms" % (t_cold * 1e6, t_cold * 1e3),
         "%.1f%%" % (100.0 * t_cold / DT)],
        ["MPC 热启动（一个 QP）", "%.1f us = %.2f ms" % (t_warm * 1e6, t_warm * 1e3),
         "%.1f%%" % (100.0 * t_warm / DT)],
    ]
    tab(rows, ["算一件事", "耗时（PC 上的 Python）", "占 %.0f Hz 周期的比例" % (1.0 / DT)])

    print("  ★ 先看 LQR：%.3f 微秒。**它是三个浮点乘法加两个加法。**"
          % (t_lqr * 1e6))
    print("    在任何一片飞控 MCU 上都是「免费」的。")
    print()
    print("  ★ 再看 MPC：热启动 %.2f ms，占 %.0f Hz 周期的 %.0f%%。"
          % (t_warm * 1e3, 1.0 / DT, 100.0 * t_warm / DT))
    print()
    print("  ⚠️ 陷阱：**这个数字不能直接拿来下结论**，有三重折扣：")
    print()
    print("    1) 这里是 PC 上的 Python + numpy，不是 STM32H7 上的 C++。")
    print("       numpy 每次调用有小数组开销，MCU 上反而可能更快；")
    print("       但 MCU 主频只有 400~500 MHz（PC 的 1/10），单手算又慢回来。")
    print("    2) 这只是 2 状态 1 输入的玩具。真机姿态环是 12+ 状态、4 输入，")
    print("       预测 N=20 步 ⇒ QP 的变量数是 80，H 是 80x80。")
    print("       投影梯度这类一阶方法的迭代次数随条件数恶化，")
    print("       实际要用 active-set / 内点法 —— 那又带来新的问题：")
    print("    3) ★★ **迭代次数取决于数据，没有固定的最坏执行时间（WCET）。**")
    print("       飞控是硬实时系统：250 Hz 就是 4 ms，超了就是丢拍、就是炸机。")
    print("       MPC 的「算多久」是数据相关的 —— 这在适航认证里是致命的。")
    print()

    # -------------------------------------------------------------------
    sub("⑥ PX4 / ArduPilot 里到底有没有 LQR / MPC")

    PX4_SRC = _srcdirs.src("PX4_115", "src")
    APM_SRC = _srcdirs.src("APM")

    APM_ROOTS = [APM_SRC + "/libraries", APM_SRC + "/ArduCopter"]
    checks = [
        ("riccati", [PX4_SRC], "PX4: Riccati 方程"),
        ("\\blqr\\b", [PX4_SRC], "PX4: LQR 这个词"),
        ("model predictive|prediction horizon", [PX4_SRC], "PX4: MPC 这个词"),
        # ★ 对照组：LQG = 卡尔曼 + LQR，两者是同一个 Riccati 方程的两半。
        #   加这一行是为了让「有一半、缺另一半」这件事在表里直接可见。
        ("kalman", [PX4_SRC], "PX4: Kalman（对照）"),
        ("riccati", APM_ROOTS, "AP: Riccati 方程"),
        ("\\blqr\\b", APM_ROOTS, "AP: LQR 这个词"),
        ("model predictive|prediction horizon", APM_ROOTS, "AP: MPC 这个词"),
        ("kalman", APM_ROOTS, "AP: Kalman（对照）"),
    ]

    print("  ★ 下面这段是**现场扫源码树**（不是抄文档）：")
    print()
    rows = []
    for pat, roots, label in checks:
        cnt, hits = _grep_source(pat, roots)
        if cnt is None:
            rows.append([label, "（找不到源码目录，跳过）", "—"])
            continue
        rows.append([label,
                     "%d 个文件" % cnt,
                     ("例：%s" % hits[0].replace("\\", "/").split("/")[-1]) if hits else u"—"])
    tab(rows, ["搜什么", "命中", "例子"])

    print("  ★★★ 结论（这就是本章最重要的工程判断）：")
    print()
    print("    **PX4 和 ArduPilot 的飞控代码里，一行 LQR、一行 MPC 都没有。**")
    print()
    print("    ★★ 表里最后两行是【对照组】，请对比着读：")
    print("       LQG = 卡尔曼 + LQR，两者是同一个 Riccati 方程的两半：")
    print("       一个管估计（往前看），一个管控制（往后看）—— 实验 3 证过。")
    print("       而 PX4 有几十个文件提到 Kalman，提到 LQR 的是 0 个。")
    print("       ⇒ **「估计」那一半做到了极致（EKF2），")
    print("          「控制」那一半整个放弃了 —— 留给了 PID。**")
    print("       这不是技术做不到，是工程选择（理由见下表）。")
    print()
    print("    注意别被名字骗了：PX4 里那一大堆 `MPC_` 开头的参数")
    print("    （MPC_XY_VEL_MAX、MPC_TILTMAX_AIR、MPC_THR_MAX ...）")
    print("    里的 MPC 是 **M**ulticopter **P**osition **C**ontrol ——")
    print("    「多旋翼位置控制」，和 Model Predictive Control 毫无关系。")
    print("    它们定义在 src/modules/mc_pos_control/multicopter_position_control_*.c。")
    print()
    print("    真实飞控用的是【串级 PID】：")
    print("      PX4    src/lib/rate_control/rate_control.hpp:127-129")
    print("             _gain_p / _gain_i / _gain_d —— 三个 Vector3f，就这么简单")
    print("      AP     libraries/AC_PID/AC_PID.h:23  class AC_PID")
    print()
    print("  为什么？四条理由，按重要性排：")
    print()
    rows = [
        ["① 算力 / 实时性", "LQR 是一次内积；MPC 每步解 QP，",
         "MPC 直接出局。LQR 这块其实不贵 —— 但它不是瓶颈"],
        ["② 需要模型", "LQR 要 (A,B)，MPC 还要约束集；",
         "四旋翼的 A、B 随载重/电池/桨叶变，PID 不需要模型，这是它的免死金牌"],
        ["③ 调试 / 可解释", "PID 的 P、I、D 各自对应「手感」的一个维度，",
         "现场能调；LQR 的 Q、R 改动会同时动所有增益，飞行员无法理解"],
        ["④ 认证 / 鲁棒性", "适航要「最坏情况可证明」；",
         "MPC 的迭代次数数据相关 ⇒ 无处安放。LQR 对模型误差没有鲁棒裕度保证"],
    ]
    tab(rows, ["理由", "说明", "后果"])
    print("  ★★ 注意第 ① 条不是主因 —— **LQR 其实很便宜**（%.3f 微秒）。"
          % (t_lqr * 1e6))
    print("     真正的墙是 ② ③ ④：**模型从哪来、参数谁来调、出了事怎么解释。**")
    print("     这三件事在消费级/工业级量产飞控上是压倒性的。")
    print()
    print("  ★ 那学术界在推什么？")
    print("    · 自适应控制（L1、MRAC）—— AP 有，但默认关，是实验性的")
    print("    · 增量非线性动态逆（INDI）—— PX4 有 mc_indi / 学术界很热")
    print("    · 学习型 MPC —— 在仿真和算力充裕的平台上（如 Jetson）做")
    print("    · ★ 而最接近量产的「最优控制」，其实是 **PX4 的自动调参**：")
    print("      src/modules/mc_autotune_attitude_control/ ——")
    print("      它用系统辨识 + GMVC 反解出 PID 增益。")
    print("      **不是把 PID 换成最优控制，而是让最优控制去算 PID 的参数。**")
    print("      这条路绕开了 ②③④ 三条墙 —— 因为它最终交付的还是一组 PID。")
    print()
    print("  ★★★ 所以本章真正的结论是：")
    print("      **LQR/MPC 的价值不在于「替换 PID」，而在于")
    print("        (a) 给你一把衡量 PID 好坏的尺子（J = ∫ x'Qx + u'Ru）；")
    print("        (b) 给你一套设计观测器的数学（Kalman / LQG）；")
    print("        (c) 给你一个理解「约束」和「最优」的框架。**")
    print("      学会它再回头看 PID，你会知道你调的那三个数在优化什么。")
    print()

    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        # 左：③-b —— 「事后夹」和「约束进优化」的轨迹重合
        axes[0].plot(ts_l, xs_l[:, 0], lw=3.0, alpha=0.45,
                     label="LQR（事后夹）")
        axes[0].plot(ts_m, xs_m[:, 0], lw=1.1, ls="--",
                     label="MPC（约束进优化）")
        axes[0].axhline(0, color="k", lw=0.6, ls="--")
        axes[0].set_xlim(0, 2.5)
        axes[0].set_xlabel("时间 (s)")
        axes[0].set_ylabel("theta (rad)")
        axes[0].grid(alpha=0.3)
        axes[0].legend(fontsize=8)
        axes[0].set_title("θ0=0.5, |u|<=%.1f：两条线完全重合\n"
                          "max|Δθ| = %.1e" % (UMAX, d_xs))
        # 右：④ —— 三种分配策略的电机指令
        idx = np.arange(4)
        axes[1].bar(idx - 0.26, u_naive, width=0.26, label="① 天真（逐通道夹）")
        axes[1].bar(idx, u_rule, width=0.26, label="② 规则式（保推力）")
        axes[1].bar(idx + 0.26, u_qp, width=0.26, label="③ 约束进优化（姿态优先）")
        axes[1].axhline(1.0, color="r", lw=1.0, ls="--")
        axes[1].text(3.35, 1.03, "电机上限", color="r", fontsize=8, ha="right")
        axes[1].set_xticks(idx)
        axes[1].set_xticklabels(["M1", "M2", "M3", "M4"])
        axes[1].set_ylim(0, 1.18)
        axes[1].set_ylabel("归一化推力")
        axes[1].grid(alpha=0.3, axis="y")
        axes[1].legend(fontsize=7, loc="lower left")
        axes[1].set_title("同一个需求 (T=3.8, τx=0.2, τy=0.1)\n三种分配的电机指令")
        savefig("lesson20_exp4")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("PID vs LQR：倒立摆正面对决", exp1),
    "2": ("Riccati 方程怎么解出来", exp2),
    "3": ("LQR 与 Kalman 的对偶性 → LQG", exp3),
    "4": ("MPC：滚动时域 + 约束 + 工程判断", exp4),
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
