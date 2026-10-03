#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 4 周 · 05 姿态解算：互补滤波与 Mahony —— 配套实验

    实验 1  ★  频域互补恒等式：低通 + 高通 = 全通（「互补」这个名字的由来）
    实验 2      一维标量互补滤波：陀螺漂移 vs 加计噪声，τ 怎么选
    实验 3  ★★  Mahony 三维姿态解算：陀螺 + 加计 + 磁力计（含零偏估计）
    实验 4  ★   Mahony 和 EKF 什么关系 —— 它是「固定增益的 EKF」

用法：
    python lesson13_mahony.py 1
    python lesson13_mahony.py 2
    python lesson13_mahony.py 3
    python lesson13_mahony.py 4
    python lesson13_mahony.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  实验 1 是本课的「地基」：它用数值证明
        H_LP(s) + H_HP(s) = 1
  这个恒等式就是「互补滤波」这个名字的全部来源 ——
  **不是两个滤波器拼起来，而是两个不完整的答案加起来正好是完整答案**。

  实验 3 是**照着 PX4 的 attitude_estimator_q_main.cpp 逐行复刻**的：
        src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp:463-571
  所以本脚本里所有四元数运算的**约定必须和 PX4 的 matrix 库一致**：
        q        : body → earth（NED）的四元数，q = [w, x, y, z]
        q.rotateVector(v)        = q ⊗ v ⊗ q*      （body → earth）
        q.rotateVectorInverse(v) = q* ⊗ v ⊗ q      （earth → body）
        q.derivative1(w)         = ½ q ⊗ (0, w)    （w 在 body 系）
  （见 src/lib/matrix/matrix/Quaternion.hpp:319-325）

  实验 4 回答本课最关键的那个问题：
        Mahony 不是卡尔曼滤波，但为什么它长得那么像 EKF？
  答案：**它就是「卡尔曼增益不更新」的 EKF** —— 也就是一个固定增益观测器。

参考：
    PX4-Autopilot/src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp
    EKF/attitude_estimator_q_main（better）.pdf
    Mahony, Hamel, Pflimlin, "Nonlinear Complementary Filters on the
        Special Orthogonal Group", IEEE TAC 53(5), 2008
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
# 四元数工具 —— 约定与 PX4 的 matrix 库逐字对齐
#
#   q = [w, x, y, z]，把 body 系的向量旋到 earth(NED) 系：
#       v_earth = q ⊗ v_body ⊗ q*
#   运动学（w 在 body 系）：
#       q̇ = ½ q ⊗ (0, w)
# ---------------------------------------------------------------------------

def q_mul(a, b):
    """四元数乘法 a ⊗ b（Hamilton 约定，与 PX4 matrix::Quaternion::operator* 一致）。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def q_rot(q, v):
    """q ⊗ v ⊗ q* —— body → earth。对应 PX4 Quatf::rotateVector。"""
    p = np.array([0.0, v[0], v[1], v[2]])
    return q_mul(q_mul(q, p), q_conj(q))[1:]


def q_rot_inv(q, v):
    """q* ⊗ v ⊗ q —— earth → body。对应 PX4 Quatf::rotateVectorInverse。"""
    p = np.array([0.0, v[0], v[1], v[2]])
    return q_mul(q_mul(q_conj(q), p), q)[1:]


def q_deriv1(q, w):
    """½ q ⊗ (0, w)。对应 PX4 matrix::Quaternion::derivative1。

    见 src/lib/matrix/matrix/Quaternion.hpp:319-325
    """
    return 0.5 * q_mul(q, np.array([0.0, w[0], w[1], w[2]]))


def q_to_dcm(q):
    """四元数 → body-to-earth 旋转矩阵。"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def dcm_to_euler(R):
    """body-to-earth 旋转矩阵 → (roll, pitch, yaw)，ZYX 顺序。

    提取公式与 PX4 matrix::Eulerf(Dcmf) 完全一致
    （见 src/lib/matrix/matrix/Euler.hpp:84-100）：
        theta = asin(-R[2,0])
        phi   = atan2(R[2,1], R[2,2])
        psi   = atan2(R[1,0], R[0,0])
    返回弧度。
    """
    theta = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(theta - math.pi / 2) < 1e-3:
        phi = 0.0
        psi = math.atan2(R[1, 2], R[0, 2])
    elif abs(theta + math.pi / 2) < 1e-3:
        phi = 0.0
        psi = math.atan2(-R[1, 2], -R[0, 2])
    else:
        phi = math.atan2(R[2, 1], R[2, 2])
        psi = math.atan2(R[1, 0], R[0, 0])
    return phi, theta, psi


def q_to_euler(q):
    return dcm_to_euler(q_to_dcm(q))


def q_from_euler(phi, theta, psi):
    """ZYX（yaw-pitch-roll）欧拉角 → body-to-earth 四元数。"""
    cp, sp = math.cos(phi * 0.5), math.sin(phi * 0.5)
    ct, st = math.cos(theta * 0.5), math.sin(theta * 0.5)
    cy, sy = math.cos(psi * 0.5), math.sin(psi * 0.5)
    return q_norm(np.array([
        cp * ct * cy + sp * st * sy,
        sp * ct * cy - cp * st * sy,
        cp * st * cy + sp * ct * sy,
        cp * ct * sy - sp * st * cy,
    ]))


def q_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    s = math.sin(angle * 0.5)
    return np.array([math.cos(angle * 0.5), axis[0] * s, axis[1] * s, axis[2] * s])


def wrap_pi(a):
    """把角度折到 (-π, π]，对应 PX4 的 math::wrap_pi。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def angle_err_deg(a, b):
    return math.degrees(abs(wrap_pi(a - b)))


# ---------------------------------------------------------------------------
# 实验 1：★ 频域互补恒等式
# ---------------------------------------------------------------------------

def exp1():
    head("实验 1  ★ 频域互补恒等式 —— 「互补」这个名字的由来")

    print("""
  问题：陀螺积分得到角度 θ_gyro，它有【低频漂移】；
        加速度计解算得到角度 θ_acc，它有【高频噪声】。
        两个都不完美，但它们的误差在不同频段。

  互补滤波的想法：**把 θ_gyro 的高频留下、θ_acc 的低频留下，加起来。**

  写成频域就是：

        θ̂(s) = H_LP(s)·θ_acc(s) + H_HP(s)·θ_gyro(s)

  其中一阶形式是：

        H_LP(s) = 1/(1+τs)          ← 低通（让加计的低频通过）
        H_HP(s) = τs/(1+τs)         ← 高通（让陀螺的高频通过）

  ★ 名字的来源就是下面这一行 —— 本节要验证的恒等式：
""")

    print("        H_LP(s) + H_HP(s) = 1/(1+τs) + τs/(1+τs) = (1+τs)/(1+τs) = 1")
    print()
    print("""  它的含义非常强：

      **如果两个传感器都完全准确（θ_acc = θ_gyro = 真值），
        那么它们的加权和【恒等于真值】—— 一个「全通」滤波器。**

  也就是说互补滤波【不是】在真值里掺了一点近似，
  而是「两个不完整的答案加起来正好是完整答案」。
  这是它和「随便做个低通滤波」的本质区别。
""")

    sub("① 数值验证：H_LP + H_HP = 1（复数域）")

    tau = 0.5
    print("    取 τ = %.2fs  ⇒ 截止频率 f_c = 1/(2πτ) = %.4f Hz"
          % (tau, 1.0 / (2 * math.pi * tau)))
    print()
    print("    %-12s %-14s %-14s %-14s %-10s" %
          ("f (Hz)", "|H_LP|", "|H_HP|", "H_LP+H_HP", "相位差°"))
    print("    " + "-" * 68)

    for f in (0.01, 0.05, 0.1, 0.1592, 0.3183, 1.0, 5.0, 50.0):
        s = 1j * 2 * math.pi * f
        h_lp = 1.0 / (1.0 + tau * s)
        h_hp = tau * s / (1.0 + tau * s)
        ssum = h_lp + h_hp
        dphase = math.degrees(abs(np.angle(h_lp) - np.angle(h_hp)))
        print("    %-12.4f %-14.6f %-14.6f %-14.6f %-10.2f"
              % (f, abs(h_lp), abs(h_hp), abs(ssum), dphase))
    print()
    print("""    ★ 两个观察：

        (1) 第 4 列恒等于 1.000000 —— 这就是「全通」，与频率无关。
        (2) 第 5 列恒等于 90° —— 两个滤波器【正交】。
            正因为正交，它们的功率也是互补的：
                |H_LP|² + |H_HP|² = 1
            （幅值互补 + 正交 ⇒ 功率互补。数值见下一节。）
""")

    sub("② 功率互补与 -3dB 交叉点")

    rows = []
    for f_ratio in (0.01, 0.1, 0.5, 1.0, 2.0, 10.0, 100.0):
        w = f_ratio                       # 归一化 ωτ
        p_lp = 1.0 / (1.0 + w * w)
        p_hp = w * w / (1.0 + w * w)
        rows.append(["ωτ = %.2f" % w, "%.6f" % p_lp, "%.6f" % p_hp,
                     "%.6f" % (p_lp + p_hp)])
    tab(rows, ["归一化频率", "|H_LP|²", "|H_HP|²", "两者之和"])
    print("""    ★ 注意 ωτ = 1（即 f = f_c）那一行：两者都是 0.5。

      这就是「互补」的几何图像：

          功率
           1 │＼                          ／
             │  ＼      H_LP          ／     H_HP
           ½ │    ＼______________／
             │        ↑
             └────────┼──────────────────────► 频率
                    f_c = 1/(2πτ)

      **两边各占一半，加起来永远是 1。**
      这个 f_c 就是你要调的「分界线」：
          低于 f_c 的误差由加速度计负责（陀螺漂移在这里）
          高于 f_c 的误差由陀螺负责（加计噪声在这里）
""")

    sub("③ 为什么不能「只用一个」：三个滤波器的对比")

    print("""    设 真值 θ、陀螺漂移 b（低频）、加计噪声 n（高频）：

        θ_gyro = θ + b        （漂移是低频的，因为它来自积分）
        θ_acc  = θ + n        （噪声是高频的）

    三种做法的误差（形式化，不是数值）：

        %-22s %-26s %s""" % ("做法", "留下的低频误差", "留下的高频误差"))
    print("    " + "-" * 68)
    print("        %-22s %-26s %s" % ("只用陀螺 θ_gyro", "b（漂移，会发散）", "0"))
    print("        %-22s %-26s %s" % ("只用加计 θ_acc", "0", "n（抖动）"))
    print("        %-22s %-26s %s" % ("互补滤波", "b 被高通滤掉", "n 被低通滤掉"))
    print()
    print("""    ★★ 一句话总结本实验：

      **互补滤波不是「折中」，而是「各取所长」——
        它利用的不是两个传感器的平均值，而是它们误差的频段不重叠。**
""")


# ---------------------------------------------------------------------------
# 实验 2：一维标量互补滤波
# ---------------------------------------------------------------------------

def comp_filter_1d(gyro, acc, dt, tau):
    """一阶互补滤波（离散实现）。

    对应连续形式     θ̂̇ = ω + (θ_acc - θ̂)/τ
    离散化（后向欧拉） θ̂ ← θ̂ + dt·(ω + (θ_acc - θ̂)/τ)

    注意 dt/τ 就是「每步信加速度计多少」，见实验 4 与增益 K 的对应。
    """
    n = len(gyro)
    th = np.zeros(n)
    k = dt / tau
    for i in range(1, n):
        th[i] = th[i - 1] + dt * gyro[i - 1] + k * (acc[i] - th[i - 1])
    return th


def exp2():
    head("实验 2  一维标量互补滤波：陀螺漂移 vs 加计噪声")

    rng = np.random.default_rng(20240913)

    dt = 0.005                 # 200 Hz，和真实飞控的 IMU 率同量级
    T = 20.0
    n = int(T / dt)
    t = np.arange(n) * dt

    # 真值：一个缓慢摆动的倾角
    A = math.radians(30.0)
    f0 = 0.2
    theta_true = A * np.sin(2 * math.pi * f0 * t)
    omega_true = A * 2 * math.pi * f0 * np.cos(2 * math.pi * f0 * t)

    # 传感器
    gyro_bias = math.radians(2.0)             # 2 °/s 的陀螺零偏
    gyro_noise_std = math.radians(0.15)       # 陀螺噪声
    acc_noise_std = math.radians(3.0)         # 加计解算的倾角噪声（含运动加速度干扰）

    gyro = omega_true + gyro_bias + rng.normal(0, gyro_noise_std, n)
    acc = theta_true + rng.normal(0, acc_noise_std, n)

    print("""
  场景设定（和真实四旋翼同量级）：
""")
    rows = [
        ["采样率", "%.0f Hz" % (1 / dt), "dt = %.4f s" % dt],
        ["真值", "%.0f° @ %.1f Hz 的正弦" % (math.degrees(A), f0), ""],
        ["陀螺零偏", "%.2f °/s" % math.degrees(gyro_bias),
         "20 秒会漂 %.1f°" % math.degrees(gyro_bias * T)],
        ["陀螺噪声", "%.3f °/s" % math.degrees(gyro_noise_std), "高频"],
        ["加计倾角噪声", "%.1f°" % math.degrees(acc_noise_std), "高频（+运动加速度）"],
    ]
    tab(rows, ["项", "值", "说明"])

    sub("① 三种估计的误差对比")

    # 纯陀螺积分
    th_gyro = np.zeros(n)
    for i in range(1, n):
        th_gyro[i] = th_gyro[i - 1] + dt * gyro[i - 1]

    # 纯加计
    th_acc = acc.copy()

    # 互补滤波
    tau = 1.0
    th_comp = comp_filter_1d(gyro, acc, dt, tau)

    def rms(e):
        return math.degrees(math.sqrt(float(np.mean(e ** 2))))

    win = slice(int(2.0 / dt), None)          # 跳过前 2 秒（避免初值影响）
    rows = [
        ["纯陀螺积分", "%.3f°" % rms(th_gyro[win] - theta_true[win]),
         "%.3f°" % math.degrees(abs(th_gyro[-1] - theta_true[-1])),
         "漂移不收敛"],
        ["纯加速度计", "%.3f°" % rms(th_acc[win] - theta_true[win]),
         "%.3f°" % math.degrees(abs(np.mean(th_acc[win] - theta_true[win]))),
         "抖 but 无偏"],
        ["互补滤波 τ=%.1fs" % tau, "%.3f°" % rms(th_comp[win] - theta_true[win]),
         "%.3f°" % math.degrees(abs(th_comp[-1] - theta_true[-1])),
         "★ 两者都小"],
    ]
    tab(rows, ["估计器", "RMS 误差", "末端误差*", "特点"])
    print("""      * 加速度计那一行填的是【平均误差】而不是末端误差 ——
        它没有漂移，用平均值更能说明「无偏」。

    ★ 读表要点（别读反了）：

        · 纯陀螺积分：RMS 24.3°、末端 39.9° —— 【两项都烂】，因为漂移是发散的。
        · 纯加速度计：RMS 3.0° 最大的一档，但平均误差只有 0.1°
          —— 【无偏，只是抖】。它的 RMS 全是高频噪声贡献的。
        · 互补滤波：RMS 1.9°，比纯加计小 1.6 倍 ——
          把「陀螺的低频准」和「加计的无偏」各取了一半。

      **衡量一个姿态估计器要看两个数：RMS（抖不抖）和均值（偏不偏）。
        陀螺赢在均值、输在 RMS；加计正好相反。**
""")

    sub("② τ 怎么选：扫描")

    rows = []
    for tau_s in (0.1, 0.3, 1.0, 3.0, 10.0, 30.0):
        e = comp_filter_1d(gyro, acc, dt, tau_s)
        rows.append([
            "%.1f s" % tau_s,
            "%.3f Hz" % (1.0 / (2 * math.pi * tau_s)),
            "%.3f°" % rms(e[win] - theta_true[win]),
            "%.3f°" % math.degrees(abs(e[-1] - theta_true[-1])),
        ])
    tab(rows, ["τ (s)", "f_c", "RMS 误差", "末端误差（漂移残留）"])
    print("""    ★ 这张表就是调参的全部内容：

        τ 太小（0.1s）⇒ 截止频率高 ⇒ 加速度计噪声全进来了
                          ⇒ RMS 大
        τ 太大（30s）⇒ 截止频率低 ⇒ 陀螺漂移没被滤掉
                          ⇒ 末端误差大

    **所以 τ 的最优值由两个因素共同决定：
        · 陀螺漂移的大小（决定 f_c 要高于多少）
        · 加计噪声的大小（决定 f_c 要低于多少）**

      真实参数（PX4 的 attitude_estimator_q 没有直接的 τ，
      它用 ATT_W_ACC 这个「权重」间接设定 —— 见实验 4 的换算）：

        ATT_W_ACC 默认 0.2  ⇒ 等效 τ ≈ 1/0.2 = 5 s
        （见 src/modules/attitude_estimator_q/attitude_estimator_q_params.c:60）
""")


# ---------------------------------------------------------------------------
# 实验 3：★★ Mahony 三维姿态解算（照抄 PX4 attitude_estimator_q 的结构）
# ---------------------------------------------------------------------------

CONSTANTS_ONE_G = 9.80665


FIFTY_DPS = 0.873          # PX4 attitude_estimator_q_main.cpp:506
SPIN_GATE = 0.175          # PX4 attitude_estimator_q_main.cpp:542


def mahony_update(q, gyro_bias, gyro, accel, mag, dt,
                  w_acc, w_mag, w_gyro_bias, bias_max,
                  acc_norm_ok=True, use_mag=True, use_acc=True,
                  mag_decl=0.0, spin_gate=True, mag_spin_gain=True,
                  pos_acc=None):
    """一步 Mahony 修正 —— 结构与 PX4 attitude_estimator_q_main.cpp:463-571 一一对应。

    参数与 PX4 的对应关系：

        PX4 变量               本函数变量       PX4 行号
        ─────────────────────  ──────────────  ─────────
        spinRate = _gyro.length()  spinRate    478
        corr                    corr            477
        _param_att_w_mag        w_mag           513
        _param_att_w_acc        w_acc           538
        _param_att_w_gyro_bias  w_gyro_bias     543
        _param_att_bias_max     bias_max        546
        _gyro_bias              gyro_bias       ★ PX4 里是【加】上去的

    use_mag / use_acc 是本脚本为了做「可观测性」对比加的开关，
    PX4 里没有 —— 真实固件靠 ATT_EXT_HDG_M 参数切换航向源。

    返回 (q, gyro_bias, corr)。
    """
    corr = np.zeros(3)

    # PX4 478 行：spinRate 在整步里只算一次，磁力计增益和零偏门限共用
    spin_rate = float(np.linalg.norm(gyro))

    # ---- (a) 磁力计修正（PX4 500-514 行）----
    if use_mag:
        mag_earth = q_rot(q, mag)
        mag_err = wrap_pi(math.atan2(mag_earth[1], mag_earth[0]) - mag_decl)
        # PX4 505-510 行：自旋越快，磁力计的增益越大。直觉：自旋时
        # 磁力计误差变化快，不改增益的话 yaw 会跟不上。
        gain_mult = 1.0
        if mag_spin_gain and spin_rate > FIFTY_DPS:
            gain_mult = min(spin_rate / FIFTY_DPS, 10.0)
        corr += (q_rot_inv(q, np.array([0.0, 0.0, -mag_err]))
                 * w_mag * gain_mult)

    # ---- (b) 加速度计修正（PX4 519-539 行）----
    # k 是【预测的】地球 Z 轴（Down）单位向量在 body 系中的表示。
    # PX4 523-527 行直接用四元数分量把它展开写出来（省掉两次乘法），
    # 这里保留 rotateVectorInverse 的写法以便和推导对应。
    k = q_rot_inv(q, np.array([0.0, 0.0, 1.0]))
    if use_acc and acc_norm_ok:
        a = accel if pos_acc is None else (accel - pos_acc)
        a_hat = a / np.linalg.norm(a)
        # 叉积 k × â：小角度下它正比于「把预测转 1 步得到测量」的旋转向量
        # （详细推导见课文 §13.4.2）
        corr += np.cross(k, a_hat) * w_acc

    # ---- (c) 陀螺零偏的积分项（PX4 541-549 行）----
    # ★★ 两个要命的细节：
    #   1) 门限 spinRate < 0.175 rad/s —— 【只在几乎静止时才学分零偏】。
    #      飞行中角速度大，此时 corr 里混着真实机动的成分，
    #      拿去积分会把机动当成零偏，所以干脆不学。
    #   2) 用的是【还没加陀螺读数】的 corr —— 也就是纯修正量。
    if (not spin_gate) or spin_rate < SPIN_GATE:
        gyro_bias = gyro_bias + corr * (w_gyro_bias * dt)
        gyro_bias = np.clip(gyro_bias, -bias_max, bias_max)

    # ---- (d) 加上陀螺（PX4 551-554 行）----
    rates = gyro + gyro_bias
    corr = corr + rates

    # ---- (e) 积分并归一化（PX4 556-560 行）----
    q = q_norm(q + q_deriv1(q, corr) * dt)
    return q, gyro_bias, corr


MAG_EARTH = np.array([0.25, 0.0, 0.35])     # Gauss：北向 0.25、向下 0.35


def simulate_attitude(dt, T, seed=7, static=False,
                      gyro_bias_dps=(1.5, -2.0, 0.8),
                      gyro_noise_dps=0.3, accel_noise=0.15, mag_noise=0.004):
    """生成一段「陀螺 + 加计 + 磁力计」的仿真数据。

    约定与 PX4 一致：
        q            : body → earth(NED)
        静止时加计读数 a_body = R^T · (0, 0, -g)   （比力朝上）
        地磁在 NED 中    mag_earth = (Bn, 0, Bd)，Bn>0 北、Bd>0 下

    static=True 时姿态恒定在 (20°, -15°, 35°)、真值角速度恒为 0 ——
    这是 Mahony 论文里最经典的收敛测试场景（静置在桌上看它收敛）。
    """
    rng = np.random.default_rng(seed)
    n = int(T / dt)
    t = np.arange(n) * dt

    if static:
        phis = np.full(n, math.radians(20.0))
        thetas = np.full(n, math.radians(-15.0))
        psis = np.full(n, math.radians(35.0))
    else:
        # 真值姿态：绕三个轴各来一点正弦，模拟手持晃动
        phis = np.radians(20.0) * np.sin(2 * math.pi * 0.25 * t)
        thetas = np.radians(15.0) * np.sin(2 * math.pi * 0.17 * t + 0.7)
        psis = np.radians(40.0) * np.sin(2 * math.pi * 0.11 * t + 1.3)

    q_true = np.zeros((n, 4))
    for i in range(n):
        q_true[i] = q_from_euler(phis[i], thetas[i], psis[i])

    # 真值角速度：用有限差分从真值姿态反算（足够精确）
    omega_true = np.zeros((n, 3))
    for i in range(1, n - 1):
        dq = q_mul(q_conj(q_true[i]), q_true[i + 1])
        if dq[0] < 0:
            dq = -dq
        v = dq[1:]
        nv = np.linalg.norm(v)
        if nv < 1e-12:
            omega_true[i] = 0.0
        else:
            omega_true[i] = v / nv * (2.0 * math.asin(min(1.0, nv)) / dt)
    omega_true[0] = omega_true[1]
    omega_true[-1] = omega_true[-2]

    # 传感器噪声 / 零偏
    gyro_bias_true = np.radians(np.array(gyro_bias_dps))     # °/s → rad/s
    gyro = (omega_true
            + gyro_bias_true
            + rng.normal(0, math.radians(gyro_noise_dps), (n, 3)))

    accel = np.zeros((n, 3))
    for i in range(n):
        a = q_rot_inv(q_true[i], np.array([0.0, 0.0, -CONSTANTS_ONE_G]))
        accel[i] = a + rng.normal(0, accel_noise, 3)

    mag = np.zeros((n, 3))
    for i in range(n):
        m = q_rot_inv(q_true[i], MAG_EARTH)
        mag[i] = m + rng.normal(0, mag_noise, 3)

    return t, q_true, gyro, accel, mag, gyro_bias_true


# PX4 默认参数（attitude_estimator_q_params.c:60,72,91,146）
PX4_W_ACC = 0.2          # ATT_W_ACC
PX4_W_MAG = 0.1          # ATT_W_MAG
PX4_W_GYRO_BIAS = 0.1    # ATT_W_GYRO_BIAS
PX4_BIAS_MAX = 0.05      # ATT_BIAS_MAX  [rad/s]


def run_mahony(gyro, accel, mag, dt, w_acc=PX4_W_ACC, w_mag=PX4_W_MAG,
               w_gyro_bias=PX4_W_GYRO_BIAS, q_init=None,
               bias_max=PX4_BIAS_MAX, use_mag=True, use_acc=True,
               acc_gate=True, spin_gate=True, mag_spin_gain=True,
               mag_decl=0.0, pos_acc=None):
    """跑完整的 Mahony 滤波，返回 (q 历史, 零偏历史)。"""
    n = len(gyro)
    q = np.array([1.0, 0.0, 0.0, 0.0]) if q_init is None else q_init.copy()
    bias = np.zeros(3)
    qs = np.zeros((n, 4))
    bs = np.zeros((n, 3))

    for i in range(n):
        # 对应 PX4 529-539 行：不在用 GPS 速度补偿加速度时，
        # 只在加计模长接近 1g 时才做修正（减少机动加速度带来的漂移）
        acc_norm_ok = True
        if acc_gate:
            a2 = float(np.dot(accel[i], accel[i]))
            lo = CONSTANTS_ONE_G * 0.9
            hi = CONSTANTS_ONE_G * 1.1
            acc_norm_ok = lo * lo < a2 < hi * hi

        pa = None if pos_acc is None else pos_acc[i]
        q, bias, _ = mahony_update(q, bias, gyro[i], accel[i], mag[i], dt,
                                   w_acc, w_mag, w_gyro_bias, bias_max,
                                   acc_norm_ok=acc_norm_ok, use_mag=use_mag,
                                   use_acc=use_acc, spin_gate=spin_gate,
                                   mag_spin_gain=mag_spin_gain,
                                   mag_decl=mag_decl, pos_acc=pa)
        qs[i] = q
        bs[i] = bias

    return qs, bs


def att_errors_deg(qs, q_true):
    """逐点姿态误差（度量：两个 body→earth 旋转之间的夹角）。"""
    e = np.zeros(len(qs))
    for i in range(len(qs)):
        dq = q_mul(q_conj(qs[i]), q_true[i])
        w = min(1.0, abs(dq[0]))
        e[i] = math.degrees(2.0 * math.acos(w))
    return e


def tilt_yaw_errors_deg(qs, q_true):
    """把姿态误差拆成【倾角误差】和【偏航误差】两部分。

    倾角误差 = 两个 DCM 第三行（地球 Z 轴在 body 系里的表示）的夹角。
       —— 这一项才是加计能观测到的量。
    偏航误差 = 由 DCM 第一行算出的 yaw 之差。
       —— 这一项只有磁力计 / 外部航向能观测到。
    """
    n = len(qs)
    tilt = np.zeros(n)
    yaw = np.zeros(n)
    for i in range(n):
        r_e = q_to_dcm(qs[i])[2]
        r_t = q_to_dcm(q_true[i])[2]
        tilt[i] = math.degrees(math.acos(
            max(-1.0, min(1.0, float(np.dot(r_e, r_t))))))
        d_e = q_to_dcm(qs[i])
        d_t = q_to_dcm(q_true[i])
        yaw[i] = angle_err_deg(math.atan2(d_e[1][0], d_e[0][0]),
                               math.atan2(d_t[1][0], d_t[0][0]))
    return tilt, yaw


def exp3():
    dt = 0.004                     # 250 Hz
    T = 120.0                      # 120 s：PX4 的增益很低，收敛本来就慢
    t, q_true, gyro, accel, mag, bias_true = simulate_attitude(dt, T, static=True)

    print("    场景：静置在桌面上（真值角速度恒为 0），真实姿态 =")
    print("          滚转 20°、俯仰 -15°、偏航 35°，陀螺零偏 = [%.2f, %.2f, %.2f] °/s"
          % tuple(np.degrees(bias_true)))
    print("          采样 250 Hz，时长 %.0f s" % T)
    print("          噪声：陀螺 0.3 °/s，加计 0.15 m/s²，磁力计 0.004 G")
    print()

    q_init = q_from_euler(math.radians(60.0), math.radians(-40.0),
                          math.radians(100.0))

    # ---- 0 叉积的方向（本课最核心的一步推导，先验证符号）----
    sub("0 先把符号钉死：叉积 k × â 到底指向哪边？")

    print("""  课文 §13.4.2 有一条推导：加速度计的修正项

        corr_acc = k × â        （k = 估计的地球 Z 轴在 body 系，â = 归一化加计读数）

    恰好等于「把估计姿态拧向真值」所需要的旋转向量（的倾角部分）。
    这条推导【符号最容易错】，所以先做数值验证 ——
    构造一个已知的估计误差 δ，看看 k × â 到底等于 +δ 还是 -δ。

    构造方式：设真值 R = I（机体与地球重合），
    估计姿态 R_hat = R · exp(δ^)，即比真值多转了一个 δ。
    那么：

        â  = 真实加计读数 / |·| = -R^T e_z = (0, 0, -1)   （指向「上」）
        k  = R_hat^T e_z                                  （估计的地球 Down）
""")

    rows = []
    for dlt in (np.array([0.1, 0.0, 0.0]),
                np.array([0.0, 0.1, 0.0]),
                np.array([0.0, 0.0, 0.1]),
                np.array([0.1, 0.05, -0.08])):
        q_true_test = np.array([1.0, 0.0, 0.0, 0.0])      # R = I
        q_est_test = q_from_euler(dlt[0], dlt[1], dlt[2])  # ≈ exp(δ^)
        a_hat = np.array([0.0, 0.0, -1.0])                 # 加计指向「上」
        k_v = q_rot_inv(q_est_test, np.array([0.0, 0.0, 1.0]))
        e = np.cross(k_v, a_hat)
        rows.append(["[%.2f %.2f %.2f]" % tuple(dlt),
                     "[%7.4f %7.4f %7.4f]" % tuple(k_v),
                     "[%7.4f %7.4f %7.4f]" % tuple(e),
                     "%7.4f" % np.linalg.norm(e - (-dlt))])
    tab(rows, ["δ", "k = R_hat^T e_z", "k × â", "‖(k×â) - (-δ)‖"])

    print("""    ★★ 看最后两列：k × â ≈ -δ（最后一列接近 0 就是证据）。

      **所以 corr_acc = k × â 指向的是「-δ」方向 ——
        而把估计姿态转回真值，需要的正是绕 -δ 转。
        方向正确，不需要再加负号。**

      ★ 那个负号是哪来的？是【物理】提供的，不是数学：
        加计读数静止时指向【上】(-R^T e_z)，而 k 指向【下】(R^T e_z)，
        两者本来就反号。如果你自己实现时把 â 定义成「测量的 Down」，
        就必须手动补一个负号，否则滤波器会【发散】。
        这是初学者最常踩的坑。

      ★ 注意 δ 里【沿重力方向的分量】不起作用：
        δ = [0, 0, 0.1] 那一行（绕地球 Z 轴转，即纯偏航误差）
        算出来的 k × â 几乎为 0 —— **加计看不见偏航**。
        这就是第 2 节可观测性表里「只用加计，偏航 129°」的原因。
""")

    print("""  ── 顺手把磁力计修正的符号也钉死 ──

    磁力计的误差项是（attitude_estimator_q_main.cpp:503-504, 513）：

        mag_err = wrap_pi(atan2(mag_earth(1), mag_earth(0)) - _mag_decl)
        corr   += q.rotateVectorInverse((0, 0, -mag_err)) * ATT_W_MAG

    注意那个【负号】。下面让估计姿态带一个已知的偏航误差，
    看 mag_err 和修正方向各是什么符号。
""")

    rows = []
    for yaw_err_deg in (5.0, -5.0, 30.0, -30.0, 120.0):
        q_e = q_from_euler(0.0, 0.0, math.radians(yaw_err_deg))
        mag_e = q_rot(q_e, MAG_EARTH)
        m_err = wrap_pi(math.atan2(mag_e[1], mag_e[0]))
        rows.append(["%+7.2f" % yaw_err_deg,
                     "[%7.4f %7.4f %7.4f]" % tuple(mag_e),
                     "%+8.3f" % math.degrees(m_err),
                     "%+8.3f" % math.degrees(-m_err)])
    tab(rows, ["偏航误差[°]", "mag_earth (q_est 转出来的)",
               "mag_err[°]", "修正绕 earth Z [°]"])

    print("""    ★★ 读表：**mag_err 精确等于偏航误差**（+5.00 -> +5.000），
      而修正量取的是 -mag_err —— **两者反号，所以是负反馈。**

      这就是为什么源码里要写 (0, 0, -mag_err) 那个负号，
      而加计那一路（上一张表）【不需要】负号 ——
      两个负号被各自的物理约定自动提供了。

      ★ 注意最后一行 ±120°：mag_err 仍然精确等于 120°。
        这正是 wrap_pi 的作用 —— atan2 的结果本来就在 (-π, π]，
        再 wrap 一次是防止 _mag_decl 相减后溢出。
        如果不 wrap，179° 和 -179° 会被算成 358° 的误差，滤波器直接飞掉。
""")

    # ---- 1 从错误的初值出发，看收敛 ----
    sub("1 从错误的初始姿态出发：Mahony 会收敛吗？")

    qs, bs = run_mahony(gyro, accel, mag, dt, q_init=q_init)
    err = att_errors_deg(qs, q_true)
    tilt, yaw = tilt_yaw_errors_deg(qs, q_true)

    rows = []
    for tt in (0.0, 2.0, 5.0, 10.0, 20.0, 40.0, 80.0, 119.9):
        i = min(len(t) - 1, int(tt / dt))
        rows.append(["%.1f" % t[i], "%.1f" % err[i],
                     "%.2f" % tilt[i], "%.2f" % yaw[i]])
    tab(rows, ["时刻[s]", "总姿态误差[°]", "倾角误差[°]", "偏航误差[°]"])

    print("""    ★ 初值误差是 60°/-40°/100°（非常大），最终收敛到 1° 以内。

      两个现象值得说清楚：

      (1) 倾角先回来、偏航后回来。
          因为它们由两个【独立的修正项】负责，各自乘自己的权重
          （attitude_estimator_q_main.cpp:513 和 538 行）：

            倾角：加计观测，ATT_W_ACC = 0.2       ⇒ 等效 tau ≈ 1/0.2 = 5 s
            偏航：磁力计观测，ATT_W_MAG = 0.1     ⇒ 等效 tau ≈ 1/0.1 = 10 s

      (2) ★★ 偏航【不是单调收敛的，它会冲过头】。
          看表里的偏航列：66° -> 36.9° -> 0.93°（20 s 处过零）
          -> 又被推到 11.6°（40 s）-> 最后才衰减下来。

          这是 PI 环的典型超调，可以解析地算出来。
          把偏航误差记作 delta，线性化后闭环是

              delta'' + K_p·delta' + K_i·delta = 0

          其中 K_p = ATT_W_MAG = 0.1，K_i = ATT_W_GYRO_BIAS = 0.1，于是

              omega_n = sqrt(K_i) = sqrt(0.1) = 0.316 rad/s
              zeta    = K_p / (2·omega_n) = 0.1 / 0.632 = 0.158

          **zeta 只有 0.16，是严重欠阻尼** —— 理论超调量约 60%。
          所以源码里 ATT_W_MAG 和 ATT_W_GYRO_BIAS 【都是 0.1 是个巧合，
          不是一对好搭档】：P 项太小、I 项太大。

      ★ 那 PX4 为什么不管这个？因为真实机型上根本不会出现 100° 的偏航初值：
        init_attitude_q()（attitude_estimator_q_main.cpp:424-461）
        在启动时就用磁力计把偏航设好了，之后磁力计只要维持就行。
        **这个滤波器的设计前提是「初值已经基本正确」。**

      ★★ 最后，Mahony 是（几乎）全局渐近稳定的非线性观测器 ——
      它不依赖「误差很小」这个线性化前提，所以能从 100° 的偏航误差拉回来。
      严格证明见 Mahony 2008 论文的 Lyapunov 分析，前提条件是：

          **重力方向与地磁方向不能平行。**

      这正是纯加计定不了偏航的原因 —— 见下一节的可观测性表。
    """)

    # ---- 2 可观测性 ----
    sub("2 三路修正各自能修什么：可观测性表")

    cases = [
        ("只用陀螺（纯积分）", dict(use_mag=False, use_acc=False)),
        ("只用加计", dict(use_mag=False)),
        ("只用磁力计", dict(use_acc=False)),
        ("加计 + 磁力计（PX4 默认）", dict()),
    ]
    rows = []
    for name, kw in cases:
        qs_c, _ = run_mahony(gyro, accel, mag, dt, q_init=q_init, **kw)
        tl, yw = tilt_yaw_errors_deg(qs_c, q_true)
        e = att_errors_deg(qs_c, q_true)
        rows.append([name, "%.1f" % e[-1], "%.1f" % tl[-1], "%.1f" % yw[-1]])
    tab(rows, ["修正源", "120s 总误差[°]", "120s 倾角[°]", "120s 偏航[°]"])

    print("""    ★★ 这张表就是 Mahony 论文那句前提的数值版：

      · 只用陀螺    -> 倾角、偏航全部发散（零偏 + 噪声，纯积分必漂）
      · 只用加计    -> 倾角几乎完美（0.1°），**偏航完全不收敛**
      · 只用磁力计  -> 倾角也发散！偏航也收敛不了
      · 两个都用    -> 全部收敛

      ★ 读表提醒：前 3 行的数字看着「没那么大」，是假象。
        姿态误差是个旋转角，超过 180° 会自动绕回来，
        所以发散的系统在表里反而可能显示出「中等」的数值。
        判断是否收敛的可靠办法是【看它随时间稳不稳定】，而不是看某一时刻的值。

        本脚本实测：把仿真时长从 120 s 延长到 240 s，

            只用陀螺      81.57°  ->  89.58°     （乱跳 = 发散）
            只用加计     129.30°  -> 174.29°     （乱跳 = 发散）
            只用磁力计   173.44°  -> 114.24°     （乱跳 = 发散）
            加计+磁力计    0.21°  ->   0.05°     （稳定 = 收敛）

      ★ 注意「只用加计」这一行：它的【倾角是收敛的】（0.1°），
        但【总误差在乱跳】—— 因为总误差里含偏航，而偏航在发散。
        一个小技巧：把一个系统的误差拆成可观测/不可观测两部分看，
        比盯一个标量总数有用得多。

      ★ 为什么加计定不了偏航？
      加计测的是【比力】。静止时它只告诉你
      「地球 Z 轴（Down）在 body 系里指哪个方向」。
      绕地球 Z 轴转多少度，这个方向都【不变】。
      所以 **偏航落在加计观测的零空间（null space）里**，
      无论看多久都定不出来。

      ★ 为什么只用磁力计也不行？
      因为磁力计提供的量是 atan2(mag_earth(1), mag_earth(0)) ——
      「把磁力计读数转到 earth 系，再看它的水平分量指哪」。
      这个计算【本身就要用到当前的姿态估计】。
      倾角一歪，投出来的水平分量就是歪的，航向也就没有意义了。

      ★★ 所以两者是【分工】而不是【冗余】：
        加计给 2 个自由度（倾角），磁力计给 1 个自由度（航向），
        合起来正好 3 个 —— 刚好把姿态定死。
        这也解释了为什么必须要求「重力方向和地磁方向不平行」：
        两者平行时，磁力计给的那 1 个自由度就落进加计的零空间，失效。
    """)

    # ---- 3 零偏积分项 + 自旋门限 ----
    sub("3 I 项（ATT_W_GYRO_BIAS）与自旋门限（spinRate < 0.175）")

    def fmt_bias(b):
        return "[%7.3f,%7.3f,%7.3f]" % (math.degrees(b[0]),
                                        math.degrees(b[1]),
                                        math.degrees(b[2]))

    win = slice(int(100.0 / dt), None)
    qs_p, bs_p = run_mahony(gyro, accel, mag, dt, q_init=q_init)
    qs_n, bs_n = run_mahony(gyro, accel, mag, dt, q_init=q_init,
                            w_gyro_bias=0.0)

    tl_p, yw_p = tilt_yaw_errors_deg(qs_p, q_true)
    tl_n, yw_n = tilt_yaw_errors_deg(qs_n, q_true)

    rows = [
        ["有 I 项 (ATT_W_GYRO_BIAS=0.1)",
         "%.3f" % float(np.mean(att_errors_deg(qs_p, q_true)[win])),
         "%.3f" % float(np.mean(tl_p[win])),
         fmt_bias(bs_p[-1])],
        ["无 I 项 (ATT_W_GYRO_BIAS=0)",
         "%.3f" % float(np.mean(att_errors_deg(qs_n, q_true)[win])),
         "%.3f" % float(np.mean(tl_n[win])),
         "恒为 0（不估计）"],
        ["真值（估计应收敛到 -bias）", "—", "—", fmt_bias(-bias_true)],
    ]
    tab(rows, ["配置（100~120s 平均）", "总误差[°]", "倾角误差[°]",
               "零偏估计 [°/s]"])

    print("""    ★★ 这是 Mahony 最有价值的部分。

      **加计/磁力计修正项不但修正姿态，还通过 I 项「学」到了陀螺零偏。**

      物理直觉（一步步来）：

        1. 零偏是 +1.5 °/s ⇒ 光积分陀螺，姿态会一直朝一个方向偏；
        2. 姿态一偏，加计/磁力计的修正项 corr 就出现了；
        3. corr 不停地把姿态往回推，于是它【持续存在】；
        4. 一个持续存在的 corr，物理含义就是「角速度有一份恒定偏差」；
        5. 把它积分进 _gyro_bias，下次就自动扣掉了。

      PX4 源码里这个「学」就是一行（attitude_estimator_q_main.cpp:543）：

          _gyro_bias += corr * (_param_att_w_gyro_bias.get() * dt);

      ★ 注意第 3 行写的是 -bias_true。因为 PX4 是

          _rates = _gyro + _gyro_bias;        // 第 551 行

      也就是 _gyro_bias 是【加】上去的，所以它学到的是真零偏的相反数。
      表中三个轴：-1.493 vs -1.500（0.5%）、1.999 vs 2.000（0.05%）、
      -0.787 vs -0.800（1.6%）—— 都在 2% 以内。

      ★ 没有 I 项时（第 2 行），姿态误差会稳定在
        「修正项刚好抵消零偏」的那个角度上 —— 一个固定的稳态误差。
        这正是纯 P 控制的宿命：**有静差**。

        这个静差可以解析地算出来：平衡时加计修正项要抵消掉零偏的倾角分量，
        即 K_p·delta = bias_tilt，于是

            delta_ss = bias_tilt / ATT_W_ACC = 2.5 °/s / 0.2 = 12.5°

        本脚本实测 100~120 s 的平均倾角误差是 13.0° —— 吻合。
        （这里 bias_tilt = sqrt(1.5² + 2.0²) = 2.5 °/s 是 x/y 轴的合成零偏，
          因为加计只能观测到倾角方向的零偏。偏航方向的零偏
          由磁力计负责，量级是 0.8/0.1 = 8°，但它会混进倾角误差里一起体现，
          所以上表里的「总误差」不是一个干净的分解。）

      ★ 这也正是 EKF 把 gyro_bias 放进状态向量所做的事
        （PX4 的 State::gyro_bias 在状态向量第 9..11 位）——
        区别只是 EKF 用协方差 P 自动算出「积分速度」，
        而 Mahony 要用手调的 ATT_W_GYRO_BIAS。
    """)

    # 自旋门限
    t2, q_true2, gyro2, accel2, mag2, _ = simulate_attitude(dt, 20.0,
                                                            static=False)
    spin = np.linalg.norm(gyro2, axis=1)
    frac = float(np.mean(spin < SPIN_GATE))
    rows = [
        ["门限 SPIN_GATE", "%.3f rad/s（attitude_estimator_q_main.cpp:542）"
         % SPIN_GATE],
        ["本段仿真的角速度", "最小 %.3f / 中位 %.3f / 最大 %.3f rad/s"
         % (float(np.min(spin)), float(np.median(spin)), float(np.max(spin)))],
        ["低于门限的采样比例", "%.1f%%  <-- 只有这些时刻才更新零偏" % (100 * frac)],
    ]
    tab(rows, ["项目", "数值"])
    print("""    ★★ 这是一个【论文里没有、只有读源码才知道】的设计：

      PX4 把零偏积分套在了一个门限里（attitude_estimator_q_main.cpp:542）：

          if (spinRate < 0.175f) {
              _gyro_bias += corr * (_param_att_w_gyro_bias.get() * dt);
              ... 限幅到 +-ATT_BIAS_MAX ...
          }

      **只在角速度很小时才学分零偏。** 为什么？

        飞行中 corr 里混着「真实机动」的成分 ——
        你正在快速滚转，加计修正项会拼命往回拉，
        这个 corr 又大又持久，但它【不是零偏】，
        而是天经地义的姿态跟踪修正。
        拿它去积分零偏，就会把机动误学成零偏，然后姿态真的歪掉。

      代价也很直白：**飞行中几乎不学零偏**（见上表）。
      所以零偏必须在起飞前（静止时）就学好 ——
      这就是 PX4 要求在【静止、水平】状态下解锁的原因之一。
    """)

    # ---- 4 三个角上的保护 ----
    sub("4 三个角上的保护：加计模长门限 / 自旋增益 / 磁偏角")

    # 4a 加计模长门限
    t4, qt4, g4, a4, mg4, _ = simulate_attitude(dt, 120.0, static=True)
    bad = a4.copy()
    j0, j1 = int(60.0 / dt), int(64.0 / dt)
    for i in range(j0, j1):
        bad[i] = bad[i] + np.array([6.0, 5.0, 3.0])

    print("    (a) 加计模长门限 —— 在 60~64 s 注入 (6,5,3) m/s² 的机动加速度。")
    print("        此时 |a| = %.2f m/s²，越过 1.1g = %.2f 的上界，"
          % (math.sqrt(CONSTANTS_ONE_G ** 2 + 36.0 + 25.0 + 9.0),
             1.1 * CONSTANTS_ONE_G))
    print("        加计读数不再指向重力方向。")
    print()
    rows = []
    for name, acc, gat in [("干净加计，门限开", a4, True),
                           ("污染加计，门限开", bad, True),
                           ("污染加计，门限关", bad, False)]:
        qs_x, bs_x = run_mahony(g4, acc, mg4, dt, q_init=qt4[0], acc_gate=gat)
        tl_x, _ = tilt_yaw_errors_deg(qs_x, qt4)
        rows.append([name,
                     "%.3f" % float(np.max(tl_x[j0:j1])),
                     "%.3f" % float(np.max(tl_x[int(66.0 / dt):])),
                     "%.3f" % math.degrees(bs_x[-1][2])])
    tab(rows, ["配置", "注入段最大倾角[°]", "注入后最大倾角[°]", "末段 z 零偏[°/s]"])

    print("""      ★★ 第 2 行与第 1 行完全一样 —— 门限把污染数据整段扔掉了。
        第 3 行（关掉门限）倾角被拽偏 33°，而且零偏估计也被污染
        （z 轴从 -0.80 变成 -1.04 °/s），久久回不来。

      PX4 的做法非常朴素（attitude_estimator_q_main.cpp:531-536）：

          const float accel_norm_sq = _accel.norm_squared();
          if (ATT_ACC_COMP || ((0.9g)² < accel_norm_sq < (1.1g)²)) {
              corr += (k % (_accel - _pos_acc).normalized()) * ATT_W_ACC;
          }

      **模长不像 1g ⇒ 现在不是「只受重力」⇒ 这次加计不可信 ⇒ 跳过。**

      ★ 注意它是【跳过这一步的修正】，不是复位、不是报错、不是降级。
        非常简单，也非常有效 —— 这就是「工程实现」与「论文公式」的差别。

      ★ 那 ATT_ACC_COMP 是什么？用 GPS 速度算出的运动加速度去补偿
        （attitude_estimator_q_main.cpp:332-363 的
         update_vehicle_local_position()），补偿之后即使机动也没关系，
        所以可以绕过这个门限。默认是关的（ATT_ACC_COMP = 0），
        因为大多数机型没有可信的 GPS 速度。
    """)

    # 4b 自旋增益
    n = int(30.0 / dt)
    psi = np.radians(160.0) * (np.arange(n) * dt) + math.radians(80.0)
    qt5 = np.array([q_from_euler(0.0, 0.0, psi[i]) for i in range(n)])
    om = np.zeros((n, 3))
    om[:, 2] = math.radians(160.0)
    rng = np.random.default_rng(3)
    g5 = om + rng.normal(0, math.radians(0.3), (n, 3))
    a5 = np.array([q_rot_inv(qt5[i], np.array([0.0, 0.0, -CONSTANTS_ONE_G]))
                   for i in range(n)]) + rng.normal(0, 0.15, (n, 3))
    m5 = np.array([q_rot_inv(qt5[i], MAG_EARTH)
                   for i in range(n)]) + rng.normal(0, 0.004, (n, 3))

    print("    (b) 自旋增益 —— 让机体以 160 °/s 匀速自旋")
    print("        （spinRate 中位 %.2f rad/s，超过 0.873 rad/s 的阈值），"
          % float(np.median(np.linalg.norm(g5, axis=1))))
    print("        初始偏航误差 80°。")
    print()
    rows = []
    for name, kw in [("开（PX4 默认）", dict(mag_spin_gain=True)),
                     ("关（gainMult 恒为 1）", dict(mag_spin_gain=False))]:
        qs_y, _ = run_mahony(g5, a5, m5, dt,
                             q_init=np.array([1.0, 0.0, 0.0, 0.0]), **kw)
        tl_y, yw_y = tilt_yaw_errors_deg(qs_y, qt5)
        rows.append([name, "%.2f" % yw_y[int(1.0 / dt)],
                     "%.2f" % yw_y[int(5.0 / dt)],
                     "%.3f" % float(np.mean(yw_y[-int(5.0 / dt):]))])
    tab(rows, ["自旋增益", "1s 偏航误差[°]", "5s 偏航误差[°]", "末段[°]"])

    print("""      ★ 对应源码（attitude_estimator_q_main.cpp:505-513）：

          float gainMult = 1.0f;
          const float fifty_dps = 0.873f;          // 50 °/s
          if (spinRate > fifty_dps) {
              gainMult = math::min(spinRate / fifty_dps, 10.0f);
          }
          corr += _q.rotateVectorInverse(Vector3f(0, 0, -mag_err))
                  * ATT_W_MAG * gainMult;

      **转得越快，磁力计的权重越大，最多放大 10 倍。**

      为什么？磁力计修正是「比例修正」，转得快时误差积累得快，
      固定权重就追不上了。放大增益相当于把等效带宽按转速抬高。
      上表：开增益时 5 s 就压到 15°，关掉则慢 3 倍以上，稳态也差 7 倍。

      ★ 上限 10 倍是个保护 —— 再大就等于把磁力计噪声直接灌进姿态。
    """)

    # 4c 磁偏角
    print("    (c) 磁偏角 —— 在已经收敛的静态场景里，故意把 MAG_DECL 填错：")
    print()
    rows = []
    for dec in (0.0, 8.0, -15.0, 25.0):
        qs_d, _ = run_mahony(g4, a4, mg4, dt, q_init=qt4[0],
                             mag_decl=math.radians(dec))
        tl_d, yw_d = tilt_yaw_errors_deg(qs_d, qt4)
        rows.append(["%+.2f" % dec,
                     "%.3f" % float(np.mean(tl_d[-int(20.0 / dt):])),
                     "%.3f" % float(np.mean(yw_d[-int(20.0 / dt):]))])
    tab(rows, ["填错的 MAG_DECL[°]", "末段倾角误差[°]", "末段偏航误差[°]"])

    print("""      ★★ 偏航误差【精确等于】填错的磁偏角，倾角完全不受影响。

      源码（attitude_estimator_q_main.cpp:500-504）：

          Vector3f mag_earth = _q.rotateVector(_mag);
          float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);

      磁力计指向的是【磁北】，飞控要的是【真北】，
      两者之差就是磁偏角。_mag_decl 填错，整个偏航就是固定偏置。

      ★ 现实中的做法是别手填：ATT_MAG_DECL_A = 1（默认开）
        会用 GPS 位置去查 WMM 世界地磁模型表自动算。
        只有没有 GPS 的室内场景才需要手填 ATT_MAG_DECL。
    """)
# ---------------------------------------------------------------------------
# 实验 4：★ Mahony 和 EKF 什么关系
# ---------------------------------------------------------------------------

def ekf_scalar_runs(N, dt, q_theta, q_bias, R, P0):
    """标量姿态 EKF 的 Riccati 递推，返回每步的卡尔曼增益历史。

    状态 x = [θ, b]：
        θ_{k+1} = θ_k + (ω_m,k - b_k)·dt
        b_{k+1} = b_k
    观测：
        z_k = θ_k + v,   v ~ N(0, R)
    所以
        F = [[1, -dt], [0, 1]],  H = [1, 0]
    """
    F = np.array([[1.0, -dt], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    Q = np.diag([q_theta, q_bias])
    P = P0.copy()
    Ks = np.zeros((N, 2, 1))

    for k in range(N):
        P = F @ P @ F.T + Q
        S = float((H @ P @ H.T)[0, 0]) + R
        K = (P @ H.T) / S
        Ks[k] = K
        # 用 Joseph 形式保证对称正定（和 PX4 的 fuseDirectStateMeasurement 一致）
        I_KH = np.eye(2) - K @ H
        P = I_KH @ P @ I_KH.T + K * R @ K.T

    return Ks


def ekf_scalar_run(gyro, acc, dt, q_theta, q_bias, R, P0, freeze_K_at=None):
    """跑标量 EKF。freeze_K_at=K 时用固定增益（= Mahony 的等价形式）。"""
    N = len(gyro)
    F = np.array([[1.0, -dt], [0.0, 1.0]])
    H = np.array([[1.0, 0.0]])
    Q = np.diag([q_theta, q_bias])
    P = P0.copy()
    x = np.zeros(2)

    th = np.zeros(N)
    Ks = np.zeros((N, 2, 1))

    for k in range(1, N):
        # --- 预测 ---
        th_before = x[0]
        x[0] = x[0] + (gyro[k - 1] - x[1]) * dt
        P = F @ P @ F.T + Q

        # --- 增益 ---
        if freeze_K_at is not None:
            K = freeze_K_at
        else:
            S = float((H @ P @ H.T)[0, 0]) + R
            K = (P @ H.T) / S
        Ks[k] = K

        # --- 更新 ---
        innov = acc[k] - x[0]
        x = x + (K * innov).ravel()
        I_KH = np.eye(2) - K @ H
        P = I_KH @ P @ I_KH.T + K * R @ K.T

        th[k] = x[0]

    return th, Ks


def exp4():
    head("实验 4  ★ Mahony 和 EKF 什么关系 —— 它是「固定增益的 EKF」")

    print("""
  这是本课最该带走的一个认识：

      **Mahony 不是「卡尔曼滤波的近似」，
        它是「卡尔曼增益不随时间更新」的卡尔曼滤波。**

  下面分三步把这个等号做实：

      ① 标量 EKF 的增益会【收敛到一个常数】（Riccati 递推收敛）
      ② 把那个常数增益冻结 —— ★ 稳态结果【完全一样】，
         差别只在启动的几十秒（这一步要分窗口看，别被总 RMS 骗了）
      ③ 那个冻结的增益，就是互补滤波里的 1/τ
         —— 也就是 Mahony 的 K_p
""")

    dt = 0.005
    T = 20.0
    n = int(T / dt)
    t = np.arange(n) * dt
    rng = np.random.default_rng(424242)

    A = math.radians(30.0)
    f0 = 0.2
    theta_true = A * np.sin(2 * math.pi * f0 * t)
    omega_true = A * 2 * math.pi * f0 * np.cos(2 * math.pi * f0 * t)

    gyro_bias_true = math.radians(2.0)
    gyro = omega_true + gyro_bias_true + rng.normal(0, math.radians(0.15), n)
    acc = theta_true + rng.normal(0, math.radians(3.0), n)

    # ---- ① Riccati 收敛 ----
    sub("① 标量 EKF 的增益会收敛到常数")

    # 噪声参数按「真实噪声」设 —— 这样才是"最优"的增益
    q_theta = (math.radians(0.15) ** 2) * dt      # 陀螺噪声驱动 θ
    q_bias = (math.radians(0.02) ** 2) * dt       # 零偏随机游走（很小）
    R = math.radians(3.0) ** 2                    # 加计观测噪声
    P0 = np.diag([math.radians(30.0) ** 2, math.radians(3.0) ** 2])

    Ks = ekf_scalar_runs(n, dt, q_theta, q_bias, R, P0)

    rows = []
    for k in (0, 10, 50, 200, 1000, 2000, 3999):
        rows.append(["%.2f s" % t[k], "%.6f" % Ks[k][0, 0], "%.6f" % Ks[k][1, 0]])
    tab(rows, ["时刻", "K[0]（姿态增益）", "K[1]（零偏增益）"])
    K_inf = Ks[-1].copy()
    print("    ★ 稳态增益：K[0] = %.6f, K[1] = %.6f" % (K_inf[0, 0], K_inf[1, 0]))
    print("""
      注意 K[1] 是【负的】—— 这不是 bug：
      零偏和姿态角是【负相关】的（零偏估大了，姿态角就被低估），
      所以最优估计里 b̂ 的修正方向和 θ̂ 相反。
""")

    # ---- ② 冻结增益 ----
    sub("② 把增益冻结成常数：稳态完全一样，差别全在启动阶段")

    th_full, _ = ekf_scalar_run(gyro, acc, dt, q_theta, q_bias, R, P0)
    th_frozen, _ = ekf_scalar_run(gyro, acc, dt, q_theta, q_bias, R, P0,
                                  freeze_K_at=K_inf)
    th_comp = comp_filter_1d(gyro, acc, dt, 1.0)

    def rms(e):
        return math.degrees(math.sqrt(float(np.mean(e ** 2))))

    win = slice(int(2.0 / dt), None)          # 跳过前 2 秒

    rows = [
        ["完整 EKF（增益每步算）", "%.4f°" % rms(th_full[win] - theta_true[win])],
        ["冻结增益 EKF", "%.4f°" % rms(th_frozen[win] - theta_true[win])],
        ["互补滤波 τ=1s", "%.4f°" % rms(th_comp[win] - theta_true[win])],
    ]
    tab(rows, ["估计器（2~20 s 窗口）", "RMS 误差"])
    print("""
    ⚠️ 先别下结论：冻结增益在这里【明显更差】。这不是 bug，
       而是因为「2~20 s」这一段里完整 EKF 还在【启动阶段】——
       它一开始的增益极大（K[0] 从 0.99 起步），收敛得快，
       而冻结增益从第一拍起就只有 0.0042，起步慢得多。

    ★★ 所以正确的检验是【分窗口看】：把两者的差随时间画出来，
       看它是不是会消失。下面就是把时长拉长到 120 s 的结果。
""")

    # ②b 分窗口对比 —— 证明「稳态一致、差别只在启动」
    T2 = 120.0
    n2 = int(T2 / dt)
    t2 = np.arange(n2) * dt
    rng2 = np.random.default_rng(424242)
    th2 = A * np.sin(2 * math.pi * f0 * t2)
    om2 = A * 2 * math.pi * f0 * np.cos(2 * math.pi * f0 * t2)
    gyro2 = om2 + gyro_bias_true + rng2.normal(0, math.radians(0.15), n2)
    acc2 = th2 + rng2.normal(0, math.radians(3.0), n2)

    tf2, _ = ekf_scalar_run(gyro2, acc2, dt, q_theta, q_bias, R, P0)
    tz2, _ = ekf_scalar_run(gyro2, acc2, dt, q_theta, q_bias, R, P0,
                            freeze_K_at=K_inf)

    rows = []
    ratios = []
    for lo, hi in ((0.0, 2.0), (2.0, 10.0), (10.0, 30.0), (30.0, 60.0), (60.0, 120.0)):
        w = slice(int(lo / dt), int(hi / dt))
        e_f = rms(tf2[w] - th2[w])
        e_z = rms(tz2[w] - th2[w])
        ratios.append(e_z / e_f)
        rows.append(["%5.0f ~ %5.0f s" % (lo, hi), "%.4f°" % e_f,
                     "%.4f°" % e_z, "%.2f×" % (e_z / e_f)])
    tab(rows, ["窗口", "完整 EKF", "冻结增益 EKF", "冻结/完整"])

    print("""    ★★ 看最后一列：%s

       **30 s 之后两者几乎完全一样（比值 1.00）。**
       差别【只存在于启动的那几十秒】—— 冻结增益丢掉了
       「一开始大增益快速收敛」的能力。

    ★ 这一条同时解释了两件事：

       (1) 「Mahony = 固定增益 EKF」这个等号，
           **在稳态意义上成立得很精确**（比值 1.00）；
       (2) 但它【启动慢】—— 这正是实验 3 第 1 节里
           Mahony 要花 20~100 s 才收敛到 1° 以内的原因。
           固定增益是有代价的，代价就在启动阶段。

      ★★ 一句话总结这一节：

          **Riccati 递推收敛之后，"最优"增益就是一个常数 ——
            所以稳态的 EKF 和一个常数增益观测器是同一个东西。
            但 EKF 真正的价值在【到达稳态之前】：它用大增益起步，
            在头几秒就把误差压下去，而固定增益要走几十秒才能走完同一条路。**
""" % " -> ".join("%.2f×" % r for r in ratios))

    # ---- ③ 常数增益 ↔ 1/τ ----
    sub("③ 常数增益就是 1/τ —— 把两个世界接起来")

    print("""    标量 EKF 的姿态通道（不含零偏）退化成：

        θ̂ ← θ̂ + (ω_m - b̂)·dt + K[0]·(θ_acc - θ̂)

    互补滤波（实验 2 用的）是：

        θ̂ ← θ̂ + ω_m·dt + (dt/τ)·(θ_acc - θ̂)

    **两项一比，立刻得到：**

        ┌─────────────────────────────────────┐
        │   K[0]  =  dt / τ      ⇔   τ = dt/K[0]  │
        └─────────────────────────────────────┘

    也就是说 —— **互补滤波里的 1/τ 就是卡尔曼增益。**
""")

    tau_from_K = dt / K_inf[0, 0]
    print("    用上表的稳态 K[0] = %.6f 反算：" % K_inf[0, 0])
    print("        τ = dt/K[0] = %.4f / %.6f = %.4f s" % (dt, K_inf[0, 0], tau_from_K))
    print("        等效 w_acc = 1/τ = %.4f" % (1.0 / tau_from_K))
    print()
    print("""    ★ 对照 PX4 的默认参数：

        ATT_W_ACC = 0.2（attitude_estimator_q_params.c:60）
        ⇒ 等效 τ = 1/0.2 = 5 s

      而本实验按「真实噪声统计」算出来的最优 τ = %.1f s，
      PX4 的默认值比它「慢」约 %.1f 倍。

      这个差距是【有意的】：本实验假设加计误差是纯白噪声，
      而真实机身上加计的误差主要来自【机动加速度】——
      那不是白噪声，是低频的、有结构的干扰。
      对这种干扰，唯一能做的就是【把权重调小、让陀螺多说话】。
      这也正是 PX4 要额外加「加计模长门限」的原因
      （见实验 3 第 4 节 (a)，以及课文 §13.7.2）。

      ⚠️ 但注意：**Mahony 用的是常数增益，EKF 用的是每步重算的增益。**
         区别在哪里？看下面。
""" % (tau_from_K, 5.0 / tau_from_K))

    # ---- ④ 什么时候 Mahony 不够用 ----
    sub("④ 常数增益什么时候会输：加计噪声变了，增益却改不了")

    n_mc = 30
    mults = (1.0, 3.0, 10.0)
    k_opts = {}
    for nm in mults:
        k_opts[nm] = ekf_scalar_runs(n, dt, q_theta, q_bias,
                                     R * nm * nm, P0)[-1].copy()

    print("    每个噪声水平跑 %d 次蒙特卡洛（不同随机种子），取 RMS 平均。"
          % n_mc)
    print("    固定增益那一列用的是【按噪声 ×1 调好的 K[0] = %.6f】，"
          % K_inf[0, 0])
    print("    也就是 Mahony 的做法：出厂标定一次，之后再也不改。")
    print()
    res4 = []
    for nm in mults:
        R_big = R * nm * nm
        err_ad = 0.0
        err_fx = 0.0
        for s in range(n_mc):
            rng_g = np.random.default_rng(2000 + s)
            rng_a = np.random.default_rng(1000 + s)
            gyro_s = (omega_true + gyro_bias_true
                      + rng_g.normal(0, math.radians(0.15), n))
            acc_s = theta_true + rng_a.normal(0, math.radians(3.0) * nm, n)
            th_a, _ = ekf_scalar_run(gyro_s, acc_s, dt, q_theta, q_bias,
                                     R_big, P0)
            th_f, _ = ekf_scalar_run(gyro_s, acc_s, dt, q_theta, q_bias,
                                     R_big, P0, freeze_K_at=K_inf)
            err_ad += rms(th_a[win] - theta_true[win])
            err_fx += rms(th_f[win] - theta_true[win])
        res4.append((nm, k_opts[nm][0, 0], err_ad / n_mc, err_fx / n_mc))

    rows4 = [["×%.0f" % r[0], "%.6f" % r[1], "%.4f°" % r[2], "%.4f°" % r[3],
              "%.2f×" % (r[3] / r[2])] for r in res4]
    tab(rows4, ["加计噪声", "该噪声下的最优 K[0]", "自适应 EKF", "固定增益 EKF",
                "固定/自适应"])

    print("""    ★ 两个规律都在这张表里：

      (1) 【最优增益随传感器质量变化】：
          加计噪声 ×1 -> ×10，最优 K[0] 从 %.6f 掉到 %.6f，小了 %.1f 倍。
          —— 观测越不可信，就越不该听它的。

      (2) 【自适应 EKF 在每个噪声水平下都赢】：
          最后一列「固定/自适应」始终大于 1（%.2f× / %.2f× / %.2f×）。

      ★★ 但【别急着下结论】—— 这个窗口是 2~20 s，
         **里面混着启动阶段**。上一节已经证明：
         固定增益的全部损失都集中在启动的那几十秒。
         所以这里的差距，有多少是"增益选错了"、
         有多少只是"起步慢"？必须把窗口挪到稳态再看一遍。

""" % (res4[0][1], res4[-1][1], res4[0][1] / res4[-1][1],
            res4[0][3] / res4[0][2], res4[1][3] / res4[1][2],
            res4[2][3] / res4[2][2]))

    # ---- ④b 稳态窗口（40~60 s）再看一遍 ----
    print("    ── 稳态窗口（40~60 s，已经充分收敛）──────────────────\n")

    T3 = 60.0
    n3 = int(T3 / dt)
    t3 = np.arange(n3) * dt
    theta3 = A * np.sin(2 * math.pi * f0 * t3)
    omega3 = A * 2 * math.pi * f0 * np.cos(2 * math.pi * f0 * t3)

    n_mc2 = 10
    res5 = []
    for nm in mults:
        R_big = R * nm * nm
        k_opt = ekf_scalar_runs(4000, dt, q_theta, q_bias, R_big, P0)[-1].copy()
        err_ad = 0.0
        err_fx = 0.0
        for s in range(n_mc2):
            rng_g = np.random.default_rng(2000 + s)
            rng_a = np.random.default_rng(1000 + s)
            gyro_s = (omega3 + gyro_bias_true
                      + rng_g.normal(0, math.radians(0.15), n3))
            acc_s = theta3 + rng_a.normal(0, math.radians(3.0) * nm, n3)
            th_a, _ = ekf_scalar_run(gyro_s, acc_s, dt, q_theta, q_bias,
                                     R_big, P0)
            th_f, _ = ekf_scalar_run(gyro_s, acc_s, dt, q_theta, q_bias,
                                     R_big, P0, freeze_K_at=K_inf)
            w3 = slice(int(40.0 / dt), None)
            err_ad += rms(th_a[w3] - theta3[w3])
            err_fx += rms(th_f[w3] - theta3[w3])
        res5.append((nm, k_opt[0, 0], err_ad / n_mc2, err_fx / n_mc2))

    rows5 = [["×%.0f" % r[0], "%.6f" % r[1], "%.4f°" % r[2], "%.4f°" % r[3],
              "%.2f×" % (r[3] / r[2])] for r in res5]
    tab(rows5, ["加计噪声", "该噪声下的最优 K[0]", "自适应 EKF", "固定增益 EKF",
                "固定/自适应"])

    print("""    ★★ 两个窗口的最后一列，趋势【正好相反】：

        加计噪声     2~20 s 窗口（含启动）   40~60 s 窗口（稳态）
        --------------------------------------------------
        ×1          %.2f×                  %.2f×
        ×3          %.2f×                  %.2f×
        ×10         %.2f×                  %.2f×
                   ↓ 递减                 ↑ 递增

      ★ 把这两行合起来读，才是完整的答案：

        【启动阶段】自适应优势在 ×1 时最大（%.2f×）——
          因为噪声小时，全 EKF 的大增益起步把误差压得又快又准，
          而固定增益只能慢慢爬。
          噪声越大、优势越小（降到 %.2f×）：
          反正大家都被噪声淹了，起步快也没用。

        【稳态阶段】×1 时比值是 %.2f×（%.4f° vs %.4f°）—— 几乎完全一样。
          这不奇怪：固定增益用的就是按 ×1 调出来的最优值，
          在它自己的主场当然打平。★ 这个 ≈1.00× 就是
          「Mahony = 固定增益的 EKF」最干净的一个数值证据。

          而噪声一旦和标定时【不一致】（×3、×10），
          固定增益就开始付出代价，比值升到 %.2f×、%.2f× ——
          稳态也有差距了。

      ★★ 所以"Mahony 什么时候输"有两个完全不同的答案：

        (1) **启动慢** —— 固定增益丢掉了"大步收敛"的能力（实验 3 里
            Mahony 要 20~100 s 才到 1° 以内，就是这个原因）；
        (2) **传感器实际水平和标定时不符** —— 增益改不了，
            稳态也要付代价。

        ★ 而 (2) 在真实飞机上【一定会发生】：出厂标定用的是
          台架上的噪声水平，真机上还有振动、机动、温度。
          只不过 PX4 的对策不是"在线调增益"，而是 §13.7 那一圈门限
          —— 用"直接把坏数据扔掉"代替"降低增益"。
""" % (res4[0][3] / res4[0][2], res5[0][3] / res5[0][2],
            res4[1][3] / res4[1][2], res5[1][3] / res5[1][2],
            res4[2][3] / res4[2][2], res5[2][3] / res5[2][2],
            res4[0][3] / res4[0][2], res4[2][3] / res4[2][2],
            res5[0][3] / res5[0][2], res5[0][2], res5[0][3],
            res5[1][3] / res5[1][2], res5[2][3] / res5[2][2]))
    print("""

    ★★ 这就是 Mahony 和 EKF 的**本质分工**：

        ┌──────────────────┬──────────────────────────────┐
        │ Mahony（固定增益）│ EKF（自适应增益）             │
        ├──────────────────┼──────────────────────────────┤
        │ 不需要 P，不算矩阵│ 维护 23×23 的 P，每步 Riccati │
        │ 常数 K_p / K_i    │ K 由 P 和 R 实时决定          │
        │ 传感器质量稳定时  │ 传感器质量变化时【自动调权】   │
        │ 【够用且极省算力】│ 但算力/内存/调参成本高得多     │
        │ 不给不确定性度量  │ ★ 输出 P ⇒ 能做故障检测/融合   │
        └──────────────────┴──────────────────────────────┘

      PX4 里两个都有，用途不同：

        attitude_estimator_q   ← Mahony 变体，只出姿态，给不需要位置的场合
        ekf2                   ← 23 维 ESKF，出姿态+位置+速度+零偏+风

      详见课文 §13.5 和 §13.6。
""")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("★ 频域互补恒等式（低通+高通=全通）", exp1),
    "2": ("一维标量互补滤波与 τ 的选取", exp2),
    "3": ("★★ Mahony 三维姿态解算（复刻 PX4）", exp3),
    "4": ("★ Mahony 与 EKF 的关系（固定增益 EKF）", exp4),
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
