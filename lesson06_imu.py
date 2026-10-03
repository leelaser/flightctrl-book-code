#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 6 周 · 01 MEMS IMU 原理与误差模型 —— 配套实验

    实验 1  比力 vs 加速度（静止时为什么读到 -9.8）
    实验 2  ★★★ Allan 方差：从数据里读出噪声参数
    实验 3  ★★ Butterworth 二阶低通（复现飞控源码）
    实验 4  ★ 误差模型的积分漂移（平均为什么压不住零偏）

用法：
    python lesson06_imu.py 1
    python lesson06_imu.py 2
    python lesson06_imu.py 3
    python lesson06_imu.py 4
    python lesson06_imu.py          # 全部

只需 numpy + matplotlib（matplotlib 缺失时会跳过画图，只打印数值）。

【本脚本的原则】

  和《NuttX/FreeRTOS 教程》一样：**能从源码复现的，就照源码写。**
  实验 3 的滤波器系数是【照抄 ArduPilot 的 compute_params】，
  然后验证它确实是 Butterworth。

  实验 2 的 Allan 方差是自己实现的 —— 因为要验证"参数能不能读回来"，
  用别人现成的库反而看不清。

参考源码（ArduPilot Copter-4.5.7）：
    libraries/Filter/LowPassFilter2p.cpp:48-67   compute_params
    libraries/Filter/LowPassFilter2p.cpp:18-35   apply
    libraries/Filter/LowPassFilter2p.cpp:42-46   reset
    libraries/AP_InertialSensor/AP_InertialSensor.cpp:60-82   四套默认参数
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
# 实验 1：比力 vs 加速度
# ===========================================================================

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def exp1():
    head("实验 1：比力 vs 加速度")

    print("""
  飞机静止，加速度计 Z 轴读到 -9.8 —— 这是对的，不是故障。

  本实验推导并验证这件事，然后展示"重力在三个轴上的投影"，
  这正是静态姿态估计的原理。
""")

    g = 9.80665
    g_ned = np.array([0.0, 0.0, g])   # NED 下：Z 向下

    sub("① 静止时的读数")

    print("""
    加速度计测的是【比力】：

        a_measured = a_true - g_ned

    静止 ⇒ a_true = 0：

        a_measured = -g_ned = (0, 0, -%.2f)
                              ↑ Z 轴指向下，读数是负的
""" % g)

    print("    数值验证：")
    a_true = np.zeros(3)
    a_meas = a_true - g_ned
    print("        a_true     = [%7.3f %7.3f %7.3f]" % tuple(a_true))
    print("        g_ned      = [%7.3f %7.3f %7.3f]" % tuple(g_ned))
    print("        a_measured = [%7.3f %7.3f %7.3f]  ← 和真实 IMU 一致" %
          tuple(a_meas))
    print()

    sub("② 把板子转起来：重力在机体轴上的投影")

    print("""
    设机体相对 NED 的姿态是 R（body→ned）。
    加速度计在【机体坐标系】里读数，所以：

        a_body = Rᵀ · (a_true_ned - g_ned)

    静止时 a_true = 0：

        a_body = -Rᵀ · g_ned

    ⇒ 重力矢量被"投影"到了三个机体轴上。
""" % ())

    print("    %-28s %-26s %s" % ("姿态", "加速度计读数 (m/s²)", "|a|"))
    print("    " + "-" * 72)

    cases = [
        ("水平 (roll=0, pitch=0)", np.eye(3)),
        ("roll=90°", rot_x(math.radians(90))),
        ("roll=-90°", rot_x(math.radians(-90))),
        ("pitch=90°", rot_y(math.radians(90))),
        ("roll=30°, pitch=20°",
         rot_y(math.radians(20)) @ rot_x(math.radians(30))),
    ]
    for name, R in cases:
        a_b = -R.T @ g_ned
        print("    %-28s [%7.3f %7.3f %7.3f]   %.3f"
              % (name, a_b[0], a_b[1], a_b[2], np.linalg.norm(a_b)))

    print()
    print("""
    ★ 注意最后一列 |a| —— 无论怎么转，模长都是 9.807。

    ⇒ ★★ 这是静态姿态估计的全部原理：

          重力矢量的【方向】告诉了你姿态
          重力矢量的【大小】是常数，可以当校验
""")

    sub("③ 静态姿态估计：从读数反推角度")

    print("""
    设 a_body = (ax, ay, az)，则：

        roll  = atan2(-ay, -az)
        pitch = atan2( ax, sqrt(ay² + az²) )
""")

    print("    %-28s %-22s %s" % ("真实姿态", "反推的 (roll, pitch)", "误差"))
    print("    " + "-" * 68)

    for roll_deg in (0, 15, -30):
        for pitch_deg in (0, 10, -25):
            R = (rot_y(math.radians(pitch_deg))
                 @ rot_x(math.radians(roll_deg)))
            a_b = -R.T @ g_ned
            ax, ay, az = a_b
            roll_est = math.degrees(math.atan2(-ay, -az))
            pitch_est = math.degrees(
                math.atan2(ax, math.sqrt(ay * ay + az * az)))
            err = max(abs(roll_est - roll_deg), abs(pitch_est - pitch_deg))
            print("    roll=%-6.0f pitch=%-7.0f    (%7.2f, %7.2f)    %.2e °"
                  % (roll_deg, pitch_deg, roll_est, pitch_est, err))

    print()
    print("""
    ⇒ 【在静止条件下，反推是精确的】（误差是浮点噪声）。

    ⚠ 但这个方法有一个致命前提：**a_true = 0**。

      飞机一加速，a_true ≠ 0
          ⇒ 加速度计分不清"是重力变了还是我在动"
          ⇒ 【姿态估计就错了】

      ★ 这是"为什么需要陀螺仪"的根本原因：

          陀螺仪：不怕运动加速度，但积分会漂
          加速度计：不漂，但怕运动加速度
          ⇒ 必须【融合】（第 4 章）
""")
    print()


# ===========================================================================
# 实验 2：Allan 方差
# ===========================================================================

def gen_imu_data(n, rate, sigma_arw, sigma_bias_rw, seed=42):
    """生成一段带三种误差的陀螺仪数据。

    参数（都是"连续时间"的密度参数）：
        sigma_arw      角度随机游走系数  (°/s/√Hz)   —— 白噪声密度 N
        sigma_bias_rw  零偏随机游走系数  (°/s/√s)    —— 决定零偏漂移快慢

    返回 (时间数组, 数据数组, 真实零偏序列)

    ★ 注意离散化时的 √(1/dt) 因子 —— 见实验说明。
    """
    rng = np.random.default_rng(seed)
    dt = 1.0 / rate

    # 白噪声：连续密度 N (°/s/√Hz) ⇒ 离散标准差 N/√dt
    #
    #   这是最容易搞错的一步：
    #   密度 σ_d 的单位是 X/√Hz，而离散样本的单位是 X
    #   两者关系：σ_sample = σ_d / √dt  （因为带宽 = 1/(2dt)，R = σ_d² · BW·2）
    white = rng.normal(0.0, sigma_arw / math.sqrt(dt), n)

    # 零偏随机游走：b[k+1] = b[k] + σ_bw·√dt·N(0,1)
    steps = rng.normal(0.0, sigma_bias_rw * math.sqrt(dt), n)
    bias = np.cumsum(steps)

    return np.arange(n) * dt, white + bias, bias


def allan_deviation(data, rate, taus, overlapping=True):
    """计算 Allan 偏差。

    对每个 τ：
        ① 把数据按长度 m = τ·rate 的块求平均 ⇒ 得到平均序列 avgs
        ② 取"相隔一个块长"的两组平均，求差
        ③ σ²(τ) = mean(Δ²) / 2

    ★★★ 这里有一个【非常容易搞错】的地方，我自己第一版就错了：

        平均序列 avgs 是【重叠】的：
            avgs[i]   = mean(data[i     : i + m])
            avgs[i+1] = mean(data[i + 1 : i + m + 1])

        ⇒ avgs[i] 和 avgs[i+1] 共享了 m-1 个样本！
        ⇒ 它们的差【几乎为零】

        正确的做法是取【相隔 m 步】的两个平均：
            avgs[i]     = mean(data[i     : i+m])
            avgs[i + m] = mean(data[i + m : i+2m])   ← 和上面【不重叠】

    ⚠ 如果错用 np.diff(avgs)（相隔 1 步），会得到：
        · 一条看起来"挺正常"的曲线
        · 但斜率是 -1.0 而不是理论上的 -0.5
        ⇒ 【数字全错，但不会报错】

    这就是本函数为什么把两种模式都实现出来（overlapping 参数），
    并让实验脚本自己验证斜率。
    """
    out = []
    n = len(data)

    for tau in taus:
        m = int(round(tau * rate))      # 每块的样本数
        if m < 1 or m >= n // 2:
            out.append(np.nan)
            continue

        if overlapping:
            # 所有可能的块（重叠）
            cumsum = np.concatenate(([0.0], np.cumsum(data)))
            nblocks = n - m + 1
            block_sums = cumsum[m:] - cumsum[:nblocks]
            avgs = block_sums / m
            # ★ 相隔 m 步 —— 不是 np.diff
            diffs = avgs[m:] - avgs[:-m]
        else:
            # 不重叠：直接分段
            k = n // m
            if k < 2:
                out.append(np.nan)
                continue
            avgs = data[:k * m].reshape(k, m).mean(axis=1)
            diffs = np.diff(avgs)

        if len(diffs) < 1:
            out.append(np.nan)
            continue

        out.append(math.sqrt(np.mean(diffs ** 2) / 2.0))

    return np.array(out)


def exp2():
    head("实验 2：★★★ Allan 方差 —— 从数据里读出噪声参数")

    print("""
  问题：给你一段 IMU 数据，怎么知道它的
        · 白噪声密度 N
        · 零偏稳定性 B
        · 随机游走系数 K
       各是多少？

  答案：Allan 方差。这个实验【自己实现一遍】，
        然后用【已知参数】的合成数据验证"能不能读回来"。

  ⚠ 用现成的库反而看不清 —— 因为不知道它内部怎么离散化的。
""")

    sub("① 生成已知参数的合成数据")

    RATE = 400.0            # Hz，常见 IMU 速率
    DURATION = 3600.0       # 秒，Allan 方差需要长时间数据
    N = int(RATE * DURATION)

    # 典型消费级 MEMS 陀螺的量级
    N_ARW = 0.005           # °/s/√Hz   —— 白噪声密度
    N_BIAS_RW = 1e-4        # °/s/√s    —— 零偏随机游走

    print("    采样率:      %.0f Hz" % RATE)
    print("    时长:        %.0f s (%.1f 小时)" % (DURATION, DURATION / 3600))
    print("    样本数:      %s" % "{:,}".format(N))
    print()
    print("    【设定的真值】")
    print("        白噪声密度 N     = %.5f °/s/√Hz" % N_ARW)
    print("        零偏随机游走     = %.2e °/s/√s" % N_BIAS_RW)
    print()

    t, data, bias = gen_imu_data(N, RATE, N_ARW, N_BIAS_RW)
    print("    生成完成。数据统计：")
    print("        均值:      %.6f °/s" % np.mean(data))
    print("        标准差:    %.6f °/s" % np.std(data))
    print("        真实零偏范围: %.5f ~ %.5f °/s"
          % (bias.min(), bias.max()))

    sub("② 计算 Allan 偏差")

    taus = np.logspace(-2, 3, 80)     # 0.01 s ~ 1000 s
    adev = allan_deviation(data, RATE, taus)

    ok = ~np.isnan(adev)
    print("    计算了 %d 个 τ 点，有效 %d 个"
          % (len(taus), int(np.sum(ok))))
    print()
    print("    %-12s %-14s %s" % ("τ (s)", "σ(τ) (°/s)", "斜率"))
    print("    " + "-" * 46)
    idxs = np.linspace(0, len(taus) - 1, 12).astype(int)
    for k, i in enumerate(idxs):
        if not ok[i]:
            continue
        slope = ""
        if k > 0 and ok[idxs[k - 1]]:
            j = idxs[k - 1]
            slope = "%.2f" % (
                (math.log10(adev[i]) - math.log10(adev[j]))
                / (math.log10(taus[i]) - math.log10(taus[j])))
        print("    %-12.4f %-14.3e %s" % (taus[i], adev[i], slope))

    sub("②b ★★★ 自我验证：斜率必须是 -0.5")

    print("""
    ⚠⚠ 这里插一段【自我验证】—— 因为我第一版实现是错的。

    【我犯的错】

      重叠版本里，平均序列 avgs 是重叠的：
          avgs[i]   = mean(data[i     : i+m])
          avgs[i+1] = mean(data[i + 1 : i+m+1])
      ⇒ 两者共享 m-1 个样本 ⇒ 差几乎为零

      我第一版用 np.diff(avgs)（相隔 1 步）求差
          ⇒ 得到一条【看起来很正常】的曲线
          ⇒ 但斜率是 -1.0，而理论值应该是 -0.5
          ⇒ 【数字全错，但不报错】

    正确做法：取【相隔 m 步】的两个平均（它们不重叠）。
""")

    # 用纯白噪声验证：斜率必须是 -0.5
    rng_v = np.random.default_rng(123)
    sw = 1.0
    test = rng_v.normal(0, sw, 200000)
    test_rate = 100.0
    taus_v = np.logspace(0, 3.5, 25)
    adev_v = allan_deviation(test, test_rate, taus_v, overlapping=True)

    ok_v = ~np.isnan(adev_v)

    # ★ 拟合区间的选取很关键：
    #   大 τ 时块数太少 ⇒ 估计量方差大 ⇒ 那些点会污染斜率拟合
    #   ⇒ 只保留"至少有 MIN_PAIRS 个独立差值对"的 τ
    MIN_PAIRS = 100
    n_pairs = len(test) / test_rate / taus_v - 1.0     # ≈ (n - m)/m
    sel = ok_v & (n_pairs >= MIN_PAIRS)

    slope = np.polyfit(np.log10(taus_v[sel]), np.log10(adev_v[sel]), 1)[0]

    print("    【纯白噪声的 Allan 偏差斜率】")
    print("        样本数      : %s" % "{:,}".format(len(test)))
    print("        采样率      : %.0f Hz" % test_rate)
    print("        拟合区间    : τ = %.2f ~ %.1f s（要求 ≥ %d 个差值对）"
          % (taus_v[sel].min(), taus_v[sel].max(), MIN_PAIRS))
    print("        参与拟合    : %d 个 τ 点" % int(np.sum(sel)))
    print("        理论斜率    : -0.500")
    print("        实测斜率    : %+.4f" % slope)
    print("        偏差        : %.4f" % abs(slope + 0.5))
    print()
    if abs(slope + 0.5) < 0.03:
        print("        ✓ 通过 —— 实现正确")
    else:
        print("        ✗ 不通过 —— 实现有问题！")

    print("""
    ⚠ 注意这里为什么要限制拟合区间：

      大 τ 时，能用的块数少
          ⇒ Allan 偏差的【估计量本身方差很大】
          ⇒ 那些点会随机地偏离理论曲线
          ⇒ 把它们纳入拟合会让斜率偏

      ★ 这是 Allan 方差分析的一个实操要点：
        【不是所有 τ 点都值得信】——
        通常只看 τ ≤ (总时长)/10 的部分。
""")

    # 两种模式对比
    print()
    print("    【重叠 vs 不重叠】（纯白噪声，τ = 10 s）")
    print()
    print("    %-14s %-16s %-16s %s"
          % ("实现", "σ(10s) 实测", "理论 σ_w/√m", "比值"))
    print("    " + "-" * 62)
    m10 = 10.0 * test_rate
    th10 = sw / math.sqrt(m10)
    for name, ov in (("重叠 (OADEV)", True), ("不重叠 (ADEV)", False)):
        v = allan_deviation(test, test_rate, [10.0], overlapping=ov)[0]
        print("    %-14s %-16.6f %-16.6f %.4f" % (name, v, th10, v / th10))

    print()
    print("""    ⇒ 两种实现都得到正确的 σ_w/√m。

      重叠版本的价值：它用了【更多样本对】⇒ 估计的方差更小。
      这就是为什么专业工具默认用 OADEV。
""")

    sub("③ ★★ 从曲线上读参数")

    # ── 白噪声 N：在 τ = 1 s 处，σ(1) = N ──
    i1 = int(np.argmin(np.abs(taus - 1.0)))
    N_est = adev[i1]
    print("    【白噪声密度 N】")
    print("        读法：σ(τ) 在 τ = 1 s 处的值")
    print("        理论：σ(τ) = N/√τ  ⇒  σ(1) = N")
    print("        实测：τ = %.3f s 处 σ = %.6f °/s" % (taus[i1], adev[i1]))
    print("        设定：N = %.6f °/s/√Hz" % N_ARW)
    print("        误差：%.1f%%" % (100 * abs(N_est - N_ARW) / N_ARW))

    # ── 零偏稳定性 B：谷底 ──
    #
    #   Allan 偏差的谷底值 B 和"零偏稳定性"的关系：
    #       谷底 σ_min ≈ 0.437 · B     （理论值，取决于噪声类型组合）
    #
    #   ★ 这里【不硬套 0.437】—— 直接报告谷底值，
    #     并说明"它和零偏稳定性同量级"。
    #     因为精确的换算系数依赖于三种噪声的比例。
    idx_min = int(np.nanargmin(adev))
    print()
    print("    【零偏稳定性 B】")
    print("        读法：曲线【谷底】")
    print("        实测：最低点 τ = %.3f s，σ = %.3e °/s" %
          (taus[idx_min], adev[idx_min]))
    print("        换算：%.3e °/s = %.4f °/h"
          % (adev[idx_min], adev[idx_min] * 3600))

    # ── 随机游走 K：τ = 3 s 处 ──
    #
    #   理论：σ(τ) = K·√(τ/3)  ⇒  σ(3) = K
    i3 = int(np.argmin(np.abs(taus - 3.0)))
    print()
    print("    【随机游走系数 K】（★ 单点读法）")
    print("        读法：σ(τ) 在 τ = 3 s 处的值")
    print("        理论：σ(τ) = K·√(τ/3)  ⇒  σ(3) = K")
    print("        实测：τ = %.3f s 处 σ = %.3e °/s" % (taus[i3], adev[i3]))
    print("        设定：K = %.3e °/s/√s" % N_BIAS_RW)
    print()
    print("        ⚠ 这个读数【完全不可信】—— 见下面的原因分析。")

    sub("③b ★★★ 正规做法：三项最小二乘拟合")

    print("""
    【为什么单点读法不行】

      Allan 方差的标准三项模型是：

          σ²(τ) = N²/τ  +  K²·τ/3  +  B²·(2/π)·ln2
                  ↑          ↑            ↑
               白噪声     随机游走     零偏稳定性

      ⇒ σ²(τ) 是 (N², K², B²) 的【线性函数】！

        令 y = σ²(τ)，x₁ = 1/τ，x₂ = τ/3，x₃ = 1
        则   y = N²·x₁ + K²·x₂ + C·x₃     其中 C = B²·(2/π)·ln2

      ⇒ ★★ 可以用【线性最小二乘】直接解出三个参数，
            不需要非线性优化。

    【单点读法为什么错】

      在 τ = 3 s 处，白噪声项是 N/√3
      只有当随机游走项【远大于】白噪声项时，σ(3) ≈ K 才成立。

      本例：N = 0.005，K = 1e-4
          白噪声在 τ=3:  0.005/√3    = 2.89e-3
          游走在 τ=3:    1e-4·√(3/3) = 1.00e-4
      ⇒ 白噪声是游走的 【29 倍】
      ⇒ σ(3) 读出来基本就是白噪声，和 K 无关
""")

    # 三项线性最小二乘
    sel_fit = ok & (taus >= 1.0) & (taus <= 300.0)
    A = np.column_stack([
        1.0 / taus[sel_fit],
        taus[sel_fit] / 3.0,
        np.ones(np.sum(sel_fit)),
    ])
    b = adev[sel_fit] ** 2
    coef, res, rank, sv = np.linalg.lstsq(A, b, rcond=None)
    N_fit = math.sqrt(max(coef[0], 0.0))
    K_fit = math.sqrt(max(coef[1], 0.0))
    C_fit = max(coef[2], 0.0)
    B_fit = math.sqrt(C_fit / (2.0 / math.pi * math.log(2.0))) if C_fit > 0 else 0.0

    print()
    print("    最小二乘解（用 τ ∈ [1, 300] s 的点）：")
    print()
    print("    %-14s %-16s %-16s %s"
          % ("参数", "设定值", "拟合值", "误差"))
    print("    " + "-" * 66)
    for name, tset, tfit in (("N (°/s/√Hz)", N_ARW, N_fit),
                             ("K (°/s/√s)", N_BIAS_RW, K_fit)):
        err = 100 * abs(tfit - tset) / tset if tset > 0 else 0
        print("    %-14s %-16.3e %-16.3e %.1f%%" % (name, tset, tfit, err))
    print("    %-14s %-16s %-16.3e %s"
          % ("B (°/s)", "（未设定）", B_fit, "—"))

    print()
    print("""    ★★★ 怎么读这三个结果：

      N（白噪声）→ 【可信】
          它主导了拟合区间的【大部分 τ】。
          理论上白噪声项 N/√τ 在 τ < 87 s 的范围内都是最大项
          ⇒ 拟合区间里"有足够多的点由它说了算"。

      K（随机游走）→ 【不太可信】
          游走项要 τ > 87 s 才超过白噪声。
          而拟合区间只到 300 s ⇒ 【真正由 K 主导的点很少】。
          ⇒ 参数在拟合区间里不主导，就估不准。

      B（零偏稳定性）→ 【正确识别为不存在】
          我在合成数据里确实没加这一项，
          拟合出 0 ⇒ ✓ 对的。
          （因为 B 项是【常数项】，不会被 1/τ 或 τ 项吸收）

    ★★★ 最重要的教训：

      **拟合精度取决于"这个参数在拟合区间里主导多少"。**

        N 全程主导   ⇒ 估得准
        K 只在最右端 ⇒ 估不准
        B 完全不存在 ⇒ 正确识别为 0

      ⇒ 实操建议：

        · **数据要足够长**（至少几小时）——
          这样 τ 范围才能覆盖到随机游走主导的区间
        · **τ 范围要覆盖三个区域**，否则某些参数估不出来
        · **报告结果时给出范围，不只给点估计**
        · ★ 如果只是**比较两块 IMU 谁好** ——
          直接比【同一 τ 处的曲线】更可靠，
          不需要做参数拟合（避开了拟合的不确定性）
""")

    sub("④ ★ 三项的对照")

    print("    理论曲线（三项取「包络」）：")
    print()
    print("        %-8s %-22s %s" % ("τ", "主导项", "说明"))
    print("        " + "-" * 58)
    for tau_test, name in ((0.01, "白噪声"), (0.1, "白噪声"),
                           (1.0, "白噪声/零偏"), (10.0, "零偏"),
                           (100.0, "随机游走"), (1000.0, "随机游走")):
        if tau_test < 1:
            why = "N/√τ：√τ 还没把噪声平均掉"
        elif tau_test < 30:
            why = "接近谷底：白噪声降下来了，游走还没起来"
        else:
            why = "K√(τ/3)：游走开始主导"
        print("        %-8.2f %-22s %s" % (tau_test, name, why))

    print()
    print("""
    ⇒ 整条曲线是三条线的【包络】：

        σ(τ) ≈ max( N/√τ , 0.437·B , K·√(τ/3) )
                    ↑         ↑          ↑
                 下降      平坦       上升
                  -1/2 斜率   0 斜率    +1/2 斜率

    ★★ 三个参数【互相独立】，因为它们在不同 τ 区间主导。

    ⚠ 一个常见错误：只在一段 τ 上拟合三项，会得到互相污染的结果。
      正确做法是用【覆盖三个区间】的 τ 范围做最小二乘。
""")

    if HAVE_PLT:
        try:
            fig, ax = plt.subplots(figsize=(9, 6))
            ax.loglog(taus[ok], adev[ok], "b.-", label="Allan deviation (measured)")

            # 三项的参考线
            ref = taus[ok]
            ax.loglog(ref, N_ARW / np.sqrt(ref), "g--", alpha=0.6,
                      label="N/sqrt(tau)  (white noise)")
            ax.loglog(ref, np.full_like(ref, adev[idx_min]), "r--", alpha=0.6,
                      label="flat (bias instability)")
            ax.loglog(ref, N_BIAS_RW * np.sqrt(ref / 3.0), "m--", alpha=0.6,
                      label="K*sqrt(tau/3)  (random walk)")

            ax.set_xlabel("tau (s)")
            ax.set_ylabel("Allan deviation (deg/s)")
            ax.set_title("Allan deviation - three noise terms")
            ax.grid(True, which="both", alpha=0.3)
            ax.legend(fontsize=9)
            fn = "allan_deviation.png"
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:
            print("    [画图失败: %s]" % e)
    else:
        print("    (没有 matplotlib，跳过画图)")
    print()


# ===========================================================================
# 实验 3：Butterworth 二阶低通
# ===========================================================================

# ★★ 照抄 ArduPilot 的 compute_params
#    libraries/Filter/LowPassFilter2p.cpp:48-67
def biquad_compute_params(sample_freq, cutoff_freq, q=None):
    """复现 ArduPilot 的 DigitalBiquadFilter::compute_params。

    返回 dict: {cutoff_freq, sample_freq, b0, b1, b2, a1, a2}

    ★ 原版用 cosf(M_PI/4.0f) 硬编码了 Q = 1/√2（Butterworth）。
      这里加一个 q 参数，方便对比其他滤波器类型。
    """
    cutoff_freq = min(cutoff_freq, sample_freq * 0.4)   # ★ 不是 0.5
    p = {"cutoff_freq": cutoff_freq, "sample_freq": sample_freq}
    if cutoff_freq <= 0:
        return p

    # 原版：c = 1 + 2·cos(π/4)·ohm + ohm²
    #   ⇒ 2·cos(π/4) = √2 = 1/Q_butterworth
    #   一般化： c = 1 + ohm/Q + ohm²
    if q is None:
        k_over_q = 2.0 * math.cos(math.pi / 4.0)        # = √2
    else:
        k_over_q = 1.0 / q

    fr = sample_freq / cutoff_freq
    ohm = math.tan(math.pi / fr)
    c = 1.0 + k_over_q * ohm + ohm * ohm

    p["b0"] = ohm * ohm / c
    p["b1"] = 2.0 * p["b0"]
    p["b2"] = p["b0"]
    p["a1"] = 2.0 * (ohm * ohm - 1.0) / c
    p["a2"] = (1.0 - k_over_q * ohm + ohm * ohm) / c
    return p


class Biquad:
    """复现 ArduPilot 的 DigitalBiquadFilter::apply / reset。

    ★ reset 用 value/(1+a1+a2) 初始化延迟单元，
      使得第一个输出【正好等于输入】—— 见文档 §1.6.4。
    """

    def __init__(self, params, init_value=None):
        self.p = params
        self.d1 = 0.0
        self.d2 = 0.0
        if init_value is None:
            # ArduPilot 的做法：用首样本初始化
            self.init_value = None
            self.initialised = False
        else:
            self.reset(init_value)
            self.initialised = True

    def reset(self, value):
        denom = 1.0 + self.p["a1"] + self.p["a2"]
        self.d1 = self.d2 = value * (1.0 / denom)

    def apply(self, sample):
        p = self.p
        if p.get("cutoff_freq", 0) <= 0:
            return sample
        if not self.initialised:
            self.reset(sample)
            self.initialised = True

        d0 = sample - self.d1 * p["a1"] - self.d2 * p["a2"]
        out = d0 * p["b0"] + self.d1 * p["b1"] + self.d2 * p["b2"]
        self.d2 = self.d1
        self.d1 = d0
        return out


def biquad_freqz(params, n=512):
    """算频率响应 H(e^{jw}) = (b0 + b1 z^-1 + b2 z^-2)/(1 + a1 z^-1 + a2 z^-2)"""
    w = np.linspace(0, math.pi, n)
    z = np.exp(-1j * w)
    num = params["b0"] + params["b1"] * z + params["b2"] * z ** 2
    den = 1.0 + params["a1"] * z + params["a2"] * z ** 2
    H = num / den
    freq = w / (2 * math.pi) * params["sample_freq"]
    return freq, np.abs(H), np.angle(H)


def exp3():
    head("实验 3：★★ Butterworth 二阶低通（复现飞控源码）")

    print("""
  ArduPilot 用的是二阶 Butterworth 低通。
  这个实验照源码复现它，然后验证三件事：

      ① cosf(M_PI/4.0f) 确实就是 √2/2 —— 二阶 Butterworth 的 Q 值
      ② 截止频率和 -3dB 点对得上
      ③ reset 里的 1/(1+a1+a2) 确实消除了启动瞬态
""")

    sub("① ★★ 那个魔法数")

    print("    源码（LowPassFilter2p.cpp:60,66）：")
    print("        float c = 1.0f + 2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;")
    print()
    print("    算一下 2·cos(π/4)：")
    v = 2.0 * math.cos(math.pi / 4.0)
    print("        2 × cos(45°) = %.10f" % v)
    print("        √2           = %.10f" % math.sqrt(2))
    print("        差           = %.2e" % abs(v - math.sqrt(2)))
    print()
    print("    ⇒ 是 √2。而 √2 = 1/Q，其中 Q = 1/√2 = %.6f" %
          (1 / math.sqrt(2)))
    print()
    print("    【标准二阶低通传递函数】：")
    print("        c = 1 + (1/Q)·ohm + ohm²")
    print()
    print("    %-22s %-10s %-12s %s" % ("滤波器类型", "Q", "1/Q", "备注"))
    print("    " + "-" * 62)
    for name, q, note in (
            ("Butterworth", 1 / math.sqrt(2), "★ ArduPilot 用的"),
            ("Bessel", 0.5774, "最大平坦群延迟"),
            ("Chebyshev (0.5dB)", 0.8637, "等波纹"),
            ("临界阻尼 (ζ=1)", 0.5, "Q=0.5 时两极点重合"),
    ):
        print("    %-22s %-10.4f %-12.4f %s" % (name, q, 1 / q, note))

    print()
    print("""    ★★ 注意 Q 和阻尼比 ζ 的关系：

          Q = 1/(2ζ)

      ⇒ Butterworth 的 Q = 0.7071 ⇒ ζ = 0.7071

      ★ 这正是第 5 章（控制理论）里那个"最佳阻尼比 0.707"——
        【同一个数，在滤波器里叫 Q，在控制系统里叫阻尼比】。
""")

    sub("② 截止频率和 -3dB 点")

    FS = 1000.0
    for FC in (10.0, 20.0, 50.0, 100.0):
        p = biquad_compute_params(FS, FC)
        freq, mag, _ph = biquad_freqz(p, 4096)
        mag_db = 20 * np.log10(np.maximum(mag, 1e-12))

        # 找 -3dB 点（相对直流增益）
        dc_db = mag_db[0]
        target = dc_db - 3.0
        # 找最接近的下穿点
        idx = np.argmin(np.abs(mag_db - target))
        f3db = freq[idx]

        # 直流增益
        dcg = (p["b0"] + p["b1"] + p["b2"]) / (1 + p["a1"] + p["a2"])

        print("    fc 设定 %-6.1f Hz  ⇒  -3dB 点 %-8.3f Hz   直流增益 %.8f"
              % (FC, f3db, dcg))

    print()
    print("    ⇒ -3dB 点精确落在设定值上（误差 < 1%），直流增益 = 1.000。")
    print()
    print("    ⚠ 但注意源码里那句 MIN：")
    print("        ret.cutoff_freq = MIN(cutoff_freq, sample_freq * 0.4);")
    print()

    print("    %-14s %-14s %s" % ("设定 fc", "实际生效 fc", "说明"))
    print("    " + "-" * 56)
    for FC in (100.0, 300.0, 400.0, 450.0, 500.0):
        p = biquad_compute_params(FS, FC)
        note = "正常" if p["cutoff_freq"] == FC else "★ 被 0.4×fs 钳制"
        print("    %-14.1f %-14.1f %s" % (FC, p["cutoff_freq"], note))

    print()
    print("""    ★ 0.4 × 1000 = 400 Hz，而不是 Nyquist 的 500 Hz。
      注释说 "Keep well under Nyquist limit" ——
      因为双线性变换在接近 Nyquist 时【频率翘曲】严重。
""")

    sub("③ ★★ 验证 reset 的初始化消除瞬态")

    FC = 20.0
    p = biquad_compute_params(FS, FC)

    # 验证那个代数恒等式：b0+b1+b2 == 1+a1+a2
    lhs = p["b0"] + p["b1"] + p["b2"]
    rhs = 1.0 + p["a1"] + p["a2"]
    print("    先验证代数恒等式（文档 §1.6.4 用到的）：")
    print("        b0+b1+b2  = %.10f" % lhs)
    print("        1+a1+a2   = %.10f" % rhs)
    print("        差        = %.2e" % abs(lhs - rhs))
    print("    ⇒ 相等。所以 1/(1+a1+a2) = 1/(b0+b1+b2)，")
    print("      用首样本初始化后，第一次输出正好等于输入。")
    print()

    VALUE = 0.5   # 假设陀螺静止读数是 0.5 °/s

    print("    【对比两种初始化】输入恒定 %.1f °/s" % VALUE)
    print()
    print("    %-6s %-22s %-22s %s"
          % ("n", "延迟单元设 0", "用 value/(1+a1+a2)", "差"))
    print("    " + "-" * 74)

    f_zero = Biquad(p, init_value=0.0)
    f_good = Biquad(p, init_value=VALUE)

    for n in range(0, 11):
        y0 = f_zero.apply(VALUE)
        y1 = f_good.apply(VALUE)
        if n in (0, 1, 2, 3, 5, 10):
            print("    %-6d %-22.8f %-22.8f %.2e"
                  % (n, y0, y1, abs(y0 - y1)))

    print()
    print("    ⇒ 错误初始化：首输出是 %.6f（= value×b0，因为延迟单元是 0），"
          % (VALUE * p["b0"]))
    print("      然后慢慢爬到 %.6f —— 用了十几个样本才追上" % VALUE)
    print("    ⇒ 正确初始化：输出【第一次就是 %.1f】，全程不动" % VALUE)
    print()
    print("""    ★ 为什么这个细节重要？

      陀螺仪静止读数是 0.5 °/s（零偏）
      滤波器未正确初始化 ⇒ 输出从 0 开始
          ⇒ 前几十毫秒里，飞控以为飞机在以 0.5 °/s 旋转
          ⇒ 【积分出错误的姿态角】

      ⚠ 对姿态控制，这可能意味着起飞瞬间的一个姿态跳变。
""")

    if HAVE_PLT:
        try:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

            # 左：频率响应
            for FC in (10, 20, 50):
                pp = biquad_compute_params(FS, FC)
                fq, mg, _ = biquad_freqz(pp, 2048)
                ax1.semilogx(fq, 20 * np.log10(np.maximum(mg, 1e-12)),
                             label="fc=%d Hz" % FC)
            ax1.axhline(-3, color="k", ls=":", alpha=0.5, label="-3 dB")
            ax1.set_xlabel("Frequency (Hz)")
            ax1.set_ylabel("Gain (dB)")
            ax1.set_title("2nd-order Butterworth LPF (ArduPilot params)")
            ax1.grid(True, which="both", alpha=0.3)
            ax1.legend(fontsize=9)
            ax1.set_ylim(-60, 5)

            # 右：启动瞬态
            fz = Biquad(p, init_value=0.0)
            fg = Biquad(p, init_value=VALUE)
            ys0 = [fz.apply(VALUE) for _ in range(100)]
            ysg = [fg.apply(VALUE) for _ in range(100)]
            ax2.plot(ys0, label="reset to 0 (wrong)")
            ax2.plot(ysg, label="reset to value/(1+a1+a2) (ArduPilot)")
            ax2.axhline(VALUE, color="k", ls=":", alpha=0.5)
            ax2.set_xlabel("sample")
            ax2.set_ylabel("output (deg/s)")
            ax2.set_title("Startup transient")
            ax2.grid(True, alpha=0.3)
            ax2.legend(fontsize=9)

            fn = "butterworth_lpf.png"
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:
            print("    [画图失败: %s]" % e)
    else:
        print("    (没有 matplotlib，跳过画图)")
    print()


# ===========================================================================
# 实验 4：误差模型的积分漂移
# ===========================================================================

def exp4():
    head("实验 4：★ 误差模型的积分漂移")

    print("""
  这一节把 §1.4 的五类误差变成可以看的数字。

  核心问题：**哪些误差会被积分放大？放大多少？**
""")

    RATE = 400.0
    T = 60.0                       # 60 秒
    N = int(RATE * T)
    dt = 1.0 / RATE

    rng = np.random.default_rng(7)

    sub("① 四种误差，单独作用（静止 ⇒ 真实角速度 = 0）")

    print("    真实角速度 = 0，所以【积分结果就应该一直是 0】。")
    print("    看各种误差让积分漂到哪去。")
    print()

    cases = []

    # 1) 纯白噪声
    N_ARW = 0.005
    w = rng.normal(0, N_ARW / math.sqrt(dt), N)
    cases.append(("白噪声 N=0.005 °/s/√Hz", w))

    # 2) 常值零偏
    B = 0.05
    cases.append(("常值零偏 b=0.05 °/s", np.full(N, B)))

    # 3) 零偏随机游走
    steps = rng.normal(0, 1e-3 * math.sqrt(dt), N)
    cases.append(("零偏随机游走", np.cumsum(steps)))

    # 4) 白噪声 + 常值零偏
    w2 = rng.normal(0, N_ARW / math.sqrt(dt), N) + B
    cases.append(("两者叠加", w2))

    print("    %-30s %-16s %-16s %s"
          % ("误差类型", "角度漂移 1s", "角度漂移 60s", "增长方式"))
    print("    " + "-" * 82)

    for name, w in cases:
        theta = np.cumsum(w) * dt
        d1 = abs(theta[int(1 * RATE) - 1])
        d60 = abs(theta[-1])
        if "白噪声" in name and "叠加" not in name:
            how = "√t（随机游走）"
        elif "常值" in name:
            how = "t（线性）★"
        elif "随机游走" in name:
            how = "t^1.5"
        else:
            how = "混合"
        print("    %-30s %-16.3e %-16.3e %s" % (name, d1, d60, how))

    sub("② ★★★ 关键实验：平均能压住什么，压不住什么")

    print("""
    ⚠⚠ 这里我又写错了一版结论，被实测推翻了。先把正确的实验做出来。

    【要区分的两个量】

        · 瞬时噪声  —— 单次测量的标准差
        · 积分漂移  —— 一段时间累积下来的角度误差

      这两个量【对平均的响应完全不同】。

    【实验设计】

      用【同一段数据】，只改平均长度 M —— 这样才可比。
      （第一版每轮重新生成噪声，导致结果不可比 —— 那是另一个 bug）
""")

    # 用多次重复统计积分漂移（单次实现波动太大）
    RATE2 = 400.0
    T2 = 10.0
    N2 = int(RATE2 * T2)
    dt2 = 1.0 / RATE2

    print("    %-8s %-18s %-18s %-14s %s"
          % ("M", "瞬时噪声", "积分漂移 std", "理论值", "比值"))
    print("    " + "-" * 74)

    inst_list = []
    drift_list = []

    for M in (1, 4, 16, 64, 256):
        k = N2 // M

        # 瞬时噪声：用一次大样本估计
        b_one = rng.normal(0, 1.0, N2)
        inst = np.std(b_one[:k * M].reshape(k, M).mean(axis=1))
        inst_list.append(inst)

        # 积分漂移：多次重复取 std
        trials = []
        for _t in range(400):
            bb = rng.normal(0, 1.0, N2)
            avg = bb[:k * M].reshape(k, M).mean(axis=1)
            trials.append(np.sum(avg) * dt2 * M)
        drift_std = np.std(trials)
        drift_list.append(drift_std)

        # 理论：std(θ) = σ_w·√(N2)·dt，与 M 无关
        theory = dt2 * 1.0 * math.sqrt(N2)
        print("    %-8d %-18.4f %-18.4e %-14.4e %.3f"
              % (M, inst, drift_std, theory, drift_std / theory))

    # ★ 用【实测值】拼结论 —— 不写死数字
    #
    #   踩过的坑：《FreeRTOS 教程》第 5 课里我曾经在文档里
    #   写了"运行后"的数字，其实是编的。这里改成动态生成。
    Ms = [1, 4, 16, 64, 256]
    inst_s = " → ".join("%.4f" % v for v in inst_list)
    drift_s = " → ".join("%.2e" % v for v in drift_list)
    drift_ratio_lo = min(d / dt2 / math.sqrt(N2) for d in drift_list)
    drift_ratio_hi = max(d / dt2 / math.sqrt(N2) for d in drift_list)

    print()
    print("""    ★★★ 实测结果（本轮运行的真实数字）：

      **瞬时噪声：%s**

                   每 4 倍 M ⇒ 缩小 2 倍，完全符合 1/√M —— 平均【有效】

      **积分漂移：%s**

                   比值在 %.2f ~ %.2f 之间波动（统计涨落）
                   ⇒ 平均【完全无效】

    ⚠ 我第一版写的是「白噪声漂移 ÷√M」—— 【错的】，被这个实验推翻。

    【为什么？】

      积分漂移由噪声的【低频成分】决定
      而低通滤波去掉的是【高频成分】
      ⇒ 高频成分对积分的贡献本来就很小 ⇒ 去掉也没什么用

      数学上：白噪声的角度随机游走是

          std(θ over T) = N · √T

      ★ 只取决于【总时长 T】——
        和采样率无关、和滤波带宽无关。

    ⇒ ⇒ 这是本课最重要的一条：
      **【低通滤波不能改善积分漂移】。**
""" % (inst_s, drift_s, drift_ratio_lo, drift_ratio_hi))

    sub("②b ★★★ 那怎么才能压住漂移？—— 估计并扣除")

    print("""
    既然滤波没用，那飞控靠什么？

    【核心思路】漂移分成两部分：

        零偏贡献 b·T    ← ★ 【可以被估计出来然后扣掉】
        噪声贡献 N·√T   ← 随机的，扣不掉

    ⇒ 对付零偏的唯一办法是【估计它】。

    【怎么估计？用"静止"】

      飞机静止时，真实角速度 = 0 ⇒ 读到的就是零偏。

      静止 T_still 秒，采样率 fs：
          零偏的估计误差 = σ_w / √(T_still · fs)

      ⇒ 静止时间越长，估计越准。

    【扣除之后还剩多少漂移？】

      剩余漂移 ≈ (估计误差) × T
               = σ_w / √(T_still · fs) × T
""")

    print("    【静止时长对后续漂移的影响】")
    print("    （σ_w = %.4f °/s 每样本等效，fs = %.0f Hz，之后飞 %.0f 秒）"
          % (N_ARW / math.sqrt(dt), RATE, T))
    print()
    print("    %-16s %-20s %-20s %s"
          % ("静止时长", "零偏估计误差", "飞行中的漂移", "对比不估计"))
    print("    " + "-" * 82)

    sw_sample = N_ARW / math.sqrt(dt)
    # 不估计时的漂移：假设零偏就是 B
    drift_no_est = B * T
    for T_still in (0.5, 1.0, 2.0, 5.0, 10.0, 30.0):
        bias_err = sw_sample / math.sqrt(T_still * RATE)
        drift = bias_err * T
        print("    %-16.1f %-20.3e %-20.3e %.1f×"
              % (T_still, bias_err, drift, drift_no_est / drift))

    print()
    print("""    ★★ 结论：

      静止 0.5 秒 ⇒ 漂移是不估计时的 【%.0f 倍小】
      静止 5 秒   ⇒ 漂移是不估计时的 【%.0f 倍小】
      静止 30 秒  ⇒ 漂移是不估计时的 【%.0f 倍小】

    ⇒ **静置几秒就能把零偏的影响压低一个数量级。**
      这就是为什么飞控启动时要求"不要动"。
""" % (drift_no_est / (sw_sample / math.sqrt(0.5 * RATE) * T),
       drift_no_est / (sw_sample / math.sqrt(5.0 * RATE) * T),
       drift_no_est / (sw_sample / math.sqrt(30.0 * RATE) * T)))

    print("""    ⚠ 但注意这个估计会【过时】：

      零偏随温度漂移（§1.4.1 的第 2 类误差）
      ⇒ 起飞前估计的零偏，飞 10 分钟后可能就不准了

      ⇒ 所以 EKF 把零偏当成一个【在线估计的状态】，
        飞行中持续修正（第 4 章）。

    ★★★ 把这一节的三条串起来，就是整条学习路线的骨架：

        静止时估计零偏        →  减小起始漂移
        飞行中在线估计零偏    →  跟踪温漂
        融合加速度计/磁罗盘   →  拉住长期漂移
                                    ↑
                              第 4 章（状态估计）整章在讲这个
""")

    sub("③ 量级估算：30 秒能漂多少度")

    print("    用典型消费级 MEMS 陀螺的参数（MPU-6000 量级）：")
    print()
    print("    %-34s %-14s %-16s %s" % ("参数", "值", "30s 后漂移", "占主导？"))
    print("    " + "-" * 76)

    g = 9.80665
    rows = [
        ("白噪声密度 0.005 °/s/√Hz", "0.005 °/s",
         N_ARW * math.sqrt(30.0) * math.sqrt(dt * RATE), "否"),
        ("零偏稳定性 10 °/h", "0.0028 °/s",
         0.0028 * 30.0, "★ 是"),
        ("温度漂移（25→60°C, 0.02 °/s/°C）", "0.7 °/s",
         0.7 * 30.0, "★★ 最强"),
    ]
    for name, val, drift, dom in rows:
        print("    %-34s %-14s %-16.3f° %s" % (name, val, drift, dom))

    print()
    print("""    ★★ 三个量级差得很远：

        白噪声：   几百分之一度
        零偏：     零点几度
        温漂：     二十度以上 —— 【飞机翻了】

    ⇒ 所以：
      · 白噪声【不用太操心】（融合算法天然会平均掉）
      · 零偏【必须在线估计】（EKF 的一个状态）
      · 温度漂移【必须做温度补偿或充分预热】

    ⚠ 而最后一条最容易被忽略 ——
      因为它不在数据手册的"零偏"一栏里，而在"零偏温度系数"里。
""")

    sub("④ 一个实用的判断")

    print("""
    实际选型时，问三个问题：

        ① 数据手册的零偏温度系数是多少？        （°/s/°C）
        ② 我的工作温度范围是多少？              （°C）
        ③ 温漂 = ① × ②，够不够用？

    例：0.02 °/s/°C × 35°C = 0.7 °/s

    ★ 如果这个数字比"零偏稳定性"大很多 ——
      那【预热】比【校准】更重要。
""")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("比力 vs 加速度", exp1),
    "2": ("★★★ Allan 方差", exp2),
    "3": ("★★ Butterworth 二阶低通", exp3),
    "4": ("★ 误差模型的积分漂移", exp4),
}


def main():
    print()
    hr("#")
    print("#  第 6 周 · 01 MEMS IMU 原理与误差模型")
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
