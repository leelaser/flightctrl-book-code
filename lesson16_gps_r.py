#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 2 第 10 章 · 观测噪声 R 的工程取值 —— 配套实验

    实验 1  ★★ 瑞利分布换算常数的独立验算：1.1774 / 1.4142 / 2.8284 / 2.4477
    实验 2  ★  手册指标 → R → PX4 参数（含 RMS / CEP / 2DRMS / R95 的歧义判别）
    实验 3  ★★ 静置实测 → R：样本量够不够、时间相关性有什么后果
    实验 4  ★★ DOP × UERE 实时 R，以及新息协方差匹配（Sage-Husa）在线估 R

用法：
    python lesson16_gps_r.py 1
    python lesson16_gps_r.py 2
    python lesson16_gps_r.py 3
    python lesson16_gps_r.py 4
    python lesson16_gps_r.py            # 全部 4 个实验

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  这一课的主题是「把手册上的一个数字，变成滤波器里能填的方差」。
  所以脚本里**每一个常数都用两种方法算**：解析式 + 蒙特卡洛。
  两者对上，才敢往课文里写。

  实验 1 回答课文的核心问题：CEP / DRMS / 2DRMS / R95 与 sigma 的换算系数
      到底是多少？—— 素材里写的 0.59 和 2.448 都是错的，这里用 400 万个
      随机点把正确值钉死。

  实验 2 把「手册 → R → PX4 参数」整条链路走一遍，包括
      **PX4 的参数是标准差、单位是米、而且是个 floor（只抬不降）** 这件事。

  实验 3 演示静置实测法最大的坑：**你测出来的标准差里，
      有一部分是慢变的（有色）误差，把它当白噪声塞进 R，
      会让新息看起来「合格」但其实是相关的** —— 这会骗过 NIS 检验。

  实验 4 是教程正文里缺的那条方法（DOP × UERE），并且用蒙特卡洛验证
      sigma_H = UERE × HDOP 和 sigma_E ≈ sigma_N ≈ sigma_H / sqrt(2)。

参考（本机源码，全部实读）：
    src/modules/ekf2/module.yaml           EKF2_GPS_P_NOISE / V_NOISE / P_GATE 的参数定义
    src/modules/ekf2/EKF/common.h          gnssSample 的 hacc / vacc / sacc 字段注释
    src/modules/ekf2/EKF/gps_control.cpp   max(hacc, gps_pos_noise) → sq() → 观测方差
    src/modules/ekf2/EKF/covariance.cpp    初始协方差里的 sq(gps_pos_noise)
    src/modules/ekf2/EKF/gnss_height_control.cpp   垂直通道的 1.5 倍关系
    src/modules/ekf2/EKF/gps_checks.cpp    pdop 只做门限，不做 R 的缩放
    msg/SensorGps.msg                      eph / epv 的上游来源
"""

import math
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 输出辅助（和本教程其它脚本一致）
# ---------------------------------------------------------------------------


def hr(title):
    print()
    print("=" * 74)
    print("  " + title)
    print("=" * 74)


def sub(title):
    print()
    print("-- " + title + " " + "-" * max(0, 68 - len(title)))


# ---------------------------------------------------------------------------
# 实验 1：瑞利分布换算常数的独立验算
# ---------------------------------------------------------------------------

# 各向同性二维高斯：E ~ N(0, s^2)，N ~ N(0, s^2) 且独立
#   => 径向误差 r = sqrt(E^2 + N^2) 服从瑞利分布，尺度参数 = s
#   => CDF: F(r) = 1 - exp(-r^2 / (2 s^2))
#   => 半径使 P(r <= R_p) = p 时：  R_p = s * sqrt(-2 ln(1 - p))
#
# 所以「50% 圆」CEP = s*sqrt(2 ln 2)，「95% 圆」R95 = s*sqrt(2 ln 20)。


def rayleigh_radius(p):
    """P(r <= R) = p 时的半径系数 R/sigma（解析式）。"""
    return math.sqrt(-2.0 * math.log(1.0 - p))


# sigma = 1 时各量的解析值
ANALYTIC = {
    "CEP  (50% 圆)": rayleigh_radius(0.50),
    "DRMS (均方根)": math.sqrt(2.0),
    "2DRMS (2xDRMS)": 2.0 * math.sqrt(2.0),
    "R90  (90% 圆)": rayleigh_radius(0.90),
    "R95  (95% 圆)": rayleigh_radius(0.95),
    "R99  (99% 圆)": rayleigh_radius(0.99),
    "平均径向误差": math.sqrt(math.pi / 2.0),
}


def exp1(n=4_000_000, seed=20260916):
    hr("实验 1：瑞利分布换算常数 —— 解析式 vs 400 万点蒙特卡洛")

    print()
    print("  模型：E、N 独立同分布 N(0, sigma^2)，r = sqrt(E^2 + N^2) ~ 瑞利(sigma)")
    print("  解析式：P(r <= R) = 1 - exp(-R^2 / (2 sigma^2))")
    print("          =>  R_p = sigma * sqrt(-2 ln(1-p))")
    print()
    print("  取 sigma = 1，抽 %d 个点：" % n)

    rng = np.random.default_rng(seed)
    e = rng.normal(0.0, 1.0, n)
    nn = rng.normal(0.0, 1.0, n)
    r = np.sqrt(e * e + nn * nn)
    r.sort()

    def mc_quantile(p):
        return float(r[min(n - 1, int(p * n))])

    mc = {
        "CEP  (50% 圆)": mc_quantile(0.50),
        "DRMS (均方根)": float(np.sqrt(np.mean(r * r))),
        "2DRMS (2xDRMS)": 2.0 * float(np.sqrt(np.mean(r * r))),
        "R90  (90% 圆)": mc_quantile(0.90),
        "R95  (95% 圆)": mc_quantile(0.95),
        "R99  (99% 圆)": mc_quantile(0.99),
        "平均径向误差": float(np.mean(r)),
    }

    print()
    print("  %-16s %12s %12s %10s" % ("量", "解析值", "蒙特卡洛", "相对差"))
    print("  " + "-" * 54)
    for k in ANALYTIC:
        a, m = ANALYTIC[k], mc[k]
        print("  %-16s %12.5f %12.5f %9.3f%%" % (k, a, m, 100.0 * (m - a) / a))

    sub("反过来：sigma = 系数 x 指标")
    print()
    print("  %-16s %14s %16s" % ("指标", "sigma = k x 指标", "k"))
    print("  " + "-" * 52)
    for k in ANALYTIC:
        if k == "平均径向误差":       # 平均径向误差不能反解 sigma 用于 R
            continue
        print("  %-16s %14.6f %16s" % (k, 1.0 / ANALYTIC[k], "-"))

    sub("★ 素材里两个常数的错误，用数值钉死")
    k_cep = 1.0 / ANALYTIC["CEP  (50% 圆)"]
    k_2drms = 1.0 / ANALYTIC["2DRMS (2xDRMS)"]
    k_r95 = 1.0 / ANALYTIC["R95  (95% 圆)"]
    print()
    print("  (a) 素材写 sigma = 0.59 * CEP")
    print("      正解 k_CEP = 1 / %.5f = %.6f" % (ANALYTIC["CEP  (50% 圆)"], k_cep))
    print("      0.59 偏低 %.1f%%  =>  方差 R 只有正确值的 %.4f 倍（小 %.1f%%）"
          % (100.0 * (1 - 0.59 / k_cep), (0.59 / k_cep) ** 2,
             100.0 * (1 - (0.59 / k_cep) ** 2)))
    print("      0.59 = 1/1.7，即「CEP = 1.7 sigma」这个民间口径；")
    print("      各向同性二维模型下的精确值是 CEP = %.4f sigma，"
          % ANALYTIC["CEP  (50% 圆)"])
    print("      1.7 比精确值大了 %.1f%% => 反解出的 sigma 自然就小 %.1f%%。"
          % (100.0 * (1.7 / ANALYTIC["CEP  (50% 圆)"] - 1.0),
             100.0 * (1.0 - 0.59 / k_cep)))
    print()
    print("  (b) 素材写 sigma = 2DRMS / 2.448")
    print("      2.448 是 R95 的系数，不是 2DRMS 的！")
    print("      k_2DRMS = 1 / %.5f = %.6f" % (ANALYTIC["2DRMS (2xDRMS)"], k_2drms))
    print("      k_R95   = 1 / %.5f = %.6f" % (ANALYTIC["R95  (95% 圆)"], k_r95))
    print("      用 2.448 除真正的 2DRMS => sigma 偏低 %.2f%%，R 偏低 %.1f%%"
          % (100.0 * (1 - 2.448 / ANALYTIC["2DRMS (2xDRMS)"]),
             100.0 * (1 - (2.448 / ANALYTIC["2DRMS (2xDRMS)"]) ** 2)))
    print()
    print("  ★ 两个错误的方向相同：sigma 偏小 => R 偏小 => 滤波器过度信任 GPS。")
    print("    这是危险的方向（抖动 + 新息爆表），不是安全的方向。")

    sub("★ 为什么『手册写 3 m』有四种可能，sigma 差 2.4 倍")
    print()
    coefs = [("CEP（或 50%）", k_cep), ("2DRMS（或 95%）", k_2drms),
             ("R95（95% 圆）", k_r95), ("RMS（单轴 1-sigma）", 1.0)]
    print("  手册「水平精度 3 m」：")
    print()
    print("  %-24s %12s %14s" % ("指标定义", "sigma (m)", "R = sigma^2"))
    print("  " + "-" * 52)
    for name, k in coefs:
        s = 3.0 * k
        print("  %-24s %12.4f %14.4f" % (name, s, s * s))
    print()
    spread = max(k for _, k in coefs) / min(k for _, k in coefs)
    print("  sigma 最大/最小 = %.3f 倍；R 最大/最小 = %.2f 倍" % (spread, spread * spread))
    print("  ⚠ 手册不写清是哪个，R 就能填错 %.1f 倍（若『RMS』其实是二维合成，"
          % (spread * spread))
    print("    那是 0.849/0.354 = %.2f 倍的 sigma、%.1f 倍的 R）。"
          % (0.849322 / 0.353553, (0.849322 / 0.353553) ** 2))

    # 可选画图
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        rr = np.linspace(0.0, 4.0, 400)
        pdf = rr / 1.0 ** 2 * np.exp(-rr * rr / (2.0 * 1.0 ** 2))
        fig, ax = plt.subplots(figsize=(8, 4.6))
        ax.plot(rr, pdf, lw=1.8, label="Rayleigh PDF (sigma=1)")
        for name, key, color in [
                ("CEP = %.4f" % ANALYTIC["CEP  (50% 圆)"], "CEP  (50% 圆)", "tab:blue"),
                ("DRMS = %.4f" % ANALYTIC["DRMS (均方根)"], "DRMS (均方根)", "tab:green"),
                ("2DRMS = %.4f" % ANALYTIC["2DRMS (2xDRMS)"], "2DRMS (2xDRMS)", "tab:orange"),
                ("R95 = %.4f" % ANALYTIC["R95  (95% 圆)"], "R95  (95% 圆)", "tab:red")]:
            ax.axvline(ANALYTIC[key], color=color, ls="--", lw=1.2, label=name)
        ax.set_xlabel("radial error r / sigma")
        ax.set_ylabel("probability density")
        ax.set_title("2D isotropic Gaussian -> Rayleigh radial error")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig("gps_r_rayleigh.png", dpi=110)
        print()
        print("  ✅ 图已存：code/gps_r_rayleigh.png")
    except Exception:
        pass


# ---------------------------------------------------------------------------
# 实验 2：手册指标 → R → PX4 参数
# ---------------------------------------------------------------------------

# 换算系数：sigma = k * 指标
K_SIGMA = {
    "cep": 1.0 / rayleigh_radius(0.50),      # 0.849322
    "drms": 1.0 / math.sqrt(2.0),            # 0.707107
    "2drms": 1.0 / (2.0 * math.sqrt(2.0)),   # 0.353553
    "r95": 1.0 / rayleigh_radius(0.95),      # 0.408539
    "rms": 1.0,                              # 单轴 1-sigma，直接用
}


def sigma_from_manual(value, metric):
    """手册指标 → 单轴 1-sigma（米）。metric ∈ K_SIGMA 的键。"""
    if metric not in K_SIGMA:
        raise ValueError("未知指标：%s" % metric)
    return value * K_SIGMA[metric]


def exp2():
    hr("实验 2：手册指标 → R → PX4 参数（一条完整链路）")

    sub("(a) 先把换算系数摆出来")
    print()
    print("  %-10s %12s   %s" % ("指标", "k", "说明"))
    print("  " + "-" * 62)
    for key, note in [
            ("cep", "sigma = 0.8493 x CEP（手册写 CEP 或 50%）"),
            ("drms", "sigma = 0.7071 x DRMS（= sqrt(sigma_E^2+sigma_N^2)）"),
            ("2drms", "sigma = 0.3536 x 2DRMS（= 2 x DRMS）"),
            ("r95", "sigma = 0.4085 x R95（手册写 95%）"),
            ("rms", "sigma = 1.0000 x RMS（★ 仅当它是单轴 1-sigma）")]:
        print("  %-10s %12.6f   %s" % (key, K_SIGMA[key], note))

    sub("(b) 数值例子：水平 CEP = 1.5 m，垂直 RMS = 3.0 m")
    cep_h, rms_v = 1.5, 3.0
    s_h = sigma_from_manual(cep_h, "cep")
    s_v = sigma_from_manual(rms_v, "rms")
    print()
    print("  水平：sigma_H（单轴） = %.4f x %.1f = %.6f m" % (K_SIGMA["cep"], cep_h, s_h))
    print("  垂直：sigma_U        = %.4f x %.1f = %.4f m" % (K_SIGMA["rms"], rms_v, s_v))
    print()
    print("  R = diag(sigma_H^2, sigma_H^2, sigma_U^2)")
    print("    = diag(%.6f, %.6f, %.6f)   [m^2]" % (s_h * s_h, s_h * s_h, s_v * s_v))
    print("    ≈ diag(%.4f, %.4f, %.4f)" % (s_h * s_h, s_h * s_h, s_v * s_v))
    print()
    print("  用这个 R 去描述这台接收机，等价于说：")
    print("    CEP  = %.4f m   （回代校验，应等于 %.1f）" % (s_h * rayleigh_radius(0.50), cep_h))
    print("    DRMS = %.4f m" % (s_h * math.sqrt(2.0)))
    print("    2DRMS= %.4f m" % (s_h * 2.0 * math.sqrt(2.0)))
    print("    R95  = %.4f m" % (s_h * rayleigh_radius(0.95)))

    sub("(c) 映射到 PX4：参数是标准差、单位米、而且是 floor")
    print()
    print("  源码事实（逐条实读，见课文 §15.5.10）：")
    print("    module.yaml:289-296   EKF2_GPS_P_NOISE  unit: m      default: 0.5")
    print("    module.yaml:280-287   EKF2_GPS_V_NOISE  unit: m/s    default: 0.3")
    print("    common.h:177          hacc = 1-std horizontal position error (m)")
    print("    gps_control.cpp:203   pos_noise = max(hacc, gps_pos_noise)   <-- 是 floor！")
    print("    gps_control.cpp:212   pos_var   = sq(pos_noise)             <-- 才是方差")
    print("    gnss_height_control.cpp:53   vertical = max(vacc, 1.5 * gps_pos_noise)")
    print()
    print("  ⇒ 参数是【标准差】，单位【米】。填进滤波器时 PX4 自己平方成方差。")
    print()
    print("  参数只抬不降：R_used = max(接收机上报的 hacc, 参数)")
    print("    · hacc > 参数  →  用 hacc（参数不起作用）")
    print("    · hacc < 参数  →  用参数（保护滤波器不被吹牛的接收机骗）")
    print()
    print("  本例：算出的水平 1-sigma = %.4f m => 建议填参数 = %.2f" % (s_h, s_h))
    print("    接收机如实上报 hacc = %.4f  →  max(%.4f, %.2f) = %.4f  正确"
          % (s_h, s_h, s_h, s_h))
    print("    接收机吹牛（上报 0.3）      →  max(0.30, %.2f) = %.2f   被参数兜住"
          % (s_h, s_h))
    print("    ★ 两种情况的参数值【是同一个】—— 因为参数就该填你算出的真值。")
    print()
    print("  ⚠ 但反方向无解：若接收机【过度保守】上报 hacc = 5.0（真值 %.4f），" % s_h)
    print("    max(5.0, %.2f) = 5.0 —— 参数压不下去。PX4 的参数是个【下界】，" % s_h)
    print("    不是可以双向纠正的标定值。这一条决定了它的正确用法：")
    print("    『用它挡住吹牛的接收机』，而不是『用它声明真实精度』。")

    sub("(d) ★ 垂直方向：PX4 写死了 1.5 倍，本例需要 2.35 倍")
    need = s_v / s_h
    print()
    print("  本例 vacc / hacc = %.4f / %.4f = %.4f 倍" % (s_v, s_h, need))
    print("  但 PX4 的垂直噪声 = 1.5 x EKF2_GPS_P_NOISE（gnss_height_control.cpp:53）")
    print("  => 无论参数填多少，PX4 眼里的垂直/水平比恒为 1.5，无法表达 %.2f 倍" % need)
    print()
    print("  后果（填 参数 = %.2f）：" % s_h)
    print("    水平 sigma = %.4f m  （= 真实值，对）" % max(s_h, s_h))
    print("    垂直 sigma = %.4f m  （真实值 %.1f m，偏乐观 %.0f%%）"
          % (1.5 * s_h, s_v, 100.0 * (s_v - 1.5 * s_h) / s_v))
    print()
    print("  ★ 放行条件：接收机如实上报 vacc = %.1f。" % s_v)
    print("    垂直用的是 max(vacc, 1.5 x 参数) = max(%.1f, %.4f) = %.1f  →  正确！"
          % (s_v, 1.5 * s_h, s_v))
    print("  ★ 所以这条链路的正确读法是：**R 的真实值来自接收机的 eph/epv，")
    print("    参数只是「不许比它更乐观」的地板**，不是「我认为的精度」。")

    sub("(e) ★ RMS 的歧义：用水平/垂直比值判别")
    print()
    print("  很多手册写 RMS，但它可能指【单轴 1-sigma】(用 K=1.0)，")
    print("  也可能指【二维合成的 DRMS】(用 K=0.7071) —— 差 1.414 倍。")
    print()
    print("  ★ 判别的抓手：**垂直只有一个轴，不存在『合成』问题**。")
    print("    所以【垂直/水平】这个比值会随手册口径变化：")
    print()
    print("  设某接收机真实 sigma_h = 1.50 m（单轴）、sigma_v = 3.00 m（比值 2.0）：")
    print()
    s_h_true, s_v_true = 1.50, 3.00
    for label, printed_h in [("手册印【单轴 sigma】", s_h_true),
                             ("手册印【DRMS】（二维合成）", s_h_true * math.sqrt(2.0))]:
        print("    %-26s 印出来：水平 %.2f : 垂直 %.2f  = 1 : %.2f"
              % (label, printed_h, s_v_true, s_v_true / printed_h))
    print()
    print("    真比值是 2.00；若手册把它印成 1.41，就说明水平那个数是二维合成值。")
    print("    经验上 GNSS 接收机的 垂直/水平 单轴比值落在 1.5~3。")
    print("    看到印出来的比值 < 1.5，就该怀疑水平是 DRMS，先乘 0.7071。")
    print("    **这个判别不需要任何仪器，只要看手册上的两个数。**")
    print()
    print("    反过来：如果两个数的比值落在 1.5~3 之间，它多半就是单轴 sigma，")
    print("    直接用，不要再乘任何系数 —— 乘了就是白送一个 0.71 倍。")


# ---------------------------------------------------------------------------
# 实验 3：静置实测 → R
# ---------------------------------------------------------------------------


def exp3(n_sec=3600, rate=1.0, s_white=0.8, s_colored=1.2,
         tau_c=60.0, seed=7):
    """合成一段「静置实测」数据：白噪声 + 一阶马尔可夫有色噪声。

    真实接收机的定位误差 = 高频白噪声（热噪声、接收机噪声）
                        + 低频有色误差（多径、电离层、星历残差）
    前者可以被滤波器平均掉，后者不能 —— 这就是本实验要暴露的坑。
    """
    hr("实验 3：静置实测 → R（样本量够不够、有色误差有什么后果）")

    dt = 1.0 / rate
    n = int(n_sec * rate)
    rng = np.random.default_rng(seed)

    rho = math.exp(-dt / tau_c)          # AR(1) 系数：相关时间 tau_c
    white = rng.normal(0.0, s_white, n)
    colored = np.zeros(n)
    w = rng.normal(0.0, s_colored * math.sqrt(1.0 - rho * rho), n)
    for i in range(1, n):
        colored[i] = rho * colored[i - 1] + w[i]

    y = white + colored
    s_total_true = math.sqrt(s_white ** 2 + s_colored ** 2)

    print()
    print("  合成一段静置数据（已知真值）：")
    print("    时长 %d s @ %.1f Hz  =>  %d 个历元" % (n_sec, rate, n))
    print("    白噪声分量      sigma_w = %.2f m" % s_white)
    print("    有色分量        sigma_c = %.2f m，相关时间 tau_c = %.0f s"
          % (s_colored, tau_c))
    print("    总标准差（真值）sigma   = %.4f m" % s_total_true)

    sub("(a) 长窗口统计 —— 素材的方法二")
    s_hat = float(np.std(y, ddof=1))
    print()
    print("    样本标准差 s = %.4f m   （真值 %.4f，偏 %.1f%%）"
          % (s_hat, s_total_true, 100.0 * (s_hat - s_total_true) / s_total_true))
    print("    => R = s^2 = %.4f m^2" % (s_hat * s_hat))
    print("    这是【总误差】的方差，含慢变部分。")
    print()
    print("    ⚠ 先别急着说「测得挺准」：偏差 %.1f%% 看起来小，但这段数据的 sigma"
          % (100.0 * (s_hat - s_total_true) / s_total_true))
    print("      估计本身就有 ~%.0f%% 的抽样不确定度（理由见 (d)）。"
          % (100.0 / math.sqrt(2.0 * n_sec / (2.0 * tau_c))))
    print("      换句话说：**静置 1 小时，你的 sigma 只能定到 10% 量级。**")

    sub("(b) 短窗口 / 差分统计 —— 白噪声那部分有多大")
    d = np.diff(y)
    s_white_hat = float(np.std(d, ddof=1)) / math.sqrt(2.0)
    print()
    print("    相邻差分 std(y_k - y_k-1) = %.4f m" % float(np.std(d, ddof=1)))
    print("    若只有白噪声，差分的方差 = 2 sigma_w^2；有色部分在 dt 内变化很小")
    print("    => sigma_w 估计 = %.4f m   （真值 %.2f）" % (s_white_hat, s_white))
    print()
    print("    白噪声方差 sigma_w^2 = %.4f m^2" % (s_white_hat ** 2))
    print("    总方差     sigma^2   = %.4f m^2" % (s_hat * s_hat))
    print("    慢变部分占比 = %.1f%%"
          % (100.0 * (s_hat ** 2 - s_white_hat ** 2) / (s_hat ** 2)))

    sub("(c) ★ 关键：把总方差当 R，新息看起来合格但其实是相关的")
    for rho_lag in (1, 5, 30, 120):
        ac = float(np.corrcoef(y[:-rho_lag], y[rho_lag:])[0, 1])
        print("    自相关 lag %3d (%5.0f s) = %+.4f" % (rho_lag, rho_lag * dt, ac))
    print()
    print("    KF 的观测噪声按定义是【白】的：E[v_k v_j] = 0 (k != j)。")
    print("    这段数据的自相关到 lag 120 还有 %.2f —— 明显不是白噪声。"
          % float(np.corrcoef(y[:-120], y[120:])[0, 1]))
    print("    把它当白噪声塞进 R，滤波器的 P 会在短时间尺度上过度乐观。")

    sub("(d) 有效样本数：为什么『采了几万个点』不等于『几万个独立样本』")
    print()
    print("    相关时间 tau_c = %.0f s，总时长 T = %d s" % (tau_c, n_sec))
    print("    有效独立样本数  N_eff ≈ T / (2 tau_c) = %.1f" % (n_sec / (2.0 * tau_c)))
    print()
    n_eff = n_sec / (2.0 * tau_c)
    print("    用正态样本估计 sigma 时，sigma_hat 的相对标准差 ≈ 1/sqrt(2 N_eff)")
    print("    => 本段数据的 sigma 估计本身就有 %.1f%% 的不确定度"
          % (100.0 / math.sqrt(2.0 * n_eff)))
    print()
    print("    要多少时长才能把 sigma 定到指定精度：")
    print("    %-12s %12s %14s" % ("目标精度", "需要 N_eff", "需要时长"))
    print("    " + "-" * 42)
    for target in (0.20, 0.10, 0.05):
        need_eff = 1.0 / (2.0 * target * target)
        print("    %-12s %12.0f %11.0f s" % ("±%.0f%%" % (100 * target),
                                            need_eff, need_eff * 2.0 * tau_c))

    sub("(e) 两种 R 的选择，各自意味着什么")
    print()
    print("    R = sigma_w^2 = %8.4f m^2   （只把白噪声当观测噪声）" % (s_white_hat ** 2))
    print("      优点：符合 KF 的白噪声假设，新息更接近白，NIS 检验可信")
    print("      代价：慢变误差不进 R，滤波器会跟着多径/电离层慢漂")
    print()
    print("    R = sigma^2   = %8.4f m^2   （总误差当观测噪声）" % (s_hat * s_hat))
    print("      优点：保守，长时段的定位误差不会被低估")
    print("      代价：新息一定相关（NIS 可能仍 ≈1，但白噪声检验会失败）；")
    print("            见 §10.8.2 的三条性质 —— 这一条过不了。")
    print()
    print("    ★ 工程折中：GNSS 位置 R 用【总方差】（因为慢变误差确实限制精度），")
    print("      但要知道新息的相关性是【正常】的，不要为了追求『白』去乱调 R。")
    print("      这也是 PX4 直接用接收机上报的 hacc（长窗口 1-sigma）的原因。")


# ---------------------------------------------------------------------------
# 实验 4：DOP × UERE，以及新息协方差匹配
# ---------------------------------------------------------------------------


def dop_from_geometry(az_deg, el_deg):
    """由卫星方位/高度角算 DOP。

    线性化的伪距观测方程：  delta_rho = G @ delta_x + eps
        delta_x = [dE, dN, dU, c*dt]^T
        G 的第 i 行 = [-cos(el)sin(az), -cos(el)cos(az), -sin(el), 1]
    （视线单位矢量由接收机指向卫星，取负号表示卫星在观测者上方）

    最小二乘解 delta_x = (G^T G)^-1 G^T delta_rho
    若 Cov(eps) = sigma_URE^2 I，则 Cov(delta_x) = sigma_URE^2 (G^T G)^-1
    记 Q = (G^T G)^-1，则 sigma_x = sigma_URE * sqrt(Q_xx)
    """
    az = np.radians(np.asarray(az_deg, dtype=float))
    el = np.radians(np.asarray(el_deg, dtype=float))
    g = np.column_stack([-np.cos(el) * np.sin(az),
                         -np.cos(el) * np.cos(az),
                         -np.sin(el),
                         np.ones_like(az)])
    q = np.linalg.inv(g.T @ g)            # cofactor 矩阵
    d = np.sqrt(np.diag(q))
    return q, d


def exp4(sigma_ure=2.0, seed=11):
    hr("实验 4：DOP × UERE 实时估 R + 新息协方差匹配（Sage-Husa）")

    sub("(a) 从几何矩阵推 DOP，并验证 sigma_H = UERE × HDOP")
    print()
    # 两组天空图：一组几何好，一组几何差（卫星挤在一起 / 高度角低）
    geometry = {
        "好几何（12 颗，方位均匀，仰角 10~80 度）":
            ([0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330],
             [80, 60, 40, 20, 10, 35, 65, 50, 30, 55, 45, 25]),
        "差几何（8 颗全挤在东北一个扇区）":
            ([30, 40, 50, 60, 70, 45, 55, 35],
             [12, 18, 15, 20, 14, 25, 16, 22]),
    }
    for label, (az, el) in geometry.items():
        q, d = dop_from_geometry(az, el)
        # GDOP 是【四维】的：位置三维 + 接收机钟差，= sqrt(trace(Q))
        gdop = math.sqrt(float(np.trace(q)))
        pdop = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
        hdop = math.sqrt(d[0] ** 2 + d[1] ** 2)
        vdop, tdop = d[2], d[3]
        print("  %s" % label)
        print("    GDOP=%.3f  PDOP=%.3f  HDOP=%.3f  VDOP=%.3f  TDOP=%.3f"
              % (gdop, pdop, hdop, vdop, tdop))
        print("    Q_EE=%.5f  Q_NN=%.5f  （两者相等吗？%s）"
              % (q[0, 0], q[1, 1], "相等" if abs(q[0, 0] - q[1, 1]) < 1e-12 else "不等"))
        s_h = sigma_ure * hdop
        s_e = sigma_ure * math.sqrt(q[0, 0])
        s_n = sigma_ure * math.sqrt(q[1, 1])
        s_v = sigma_ure * vdop
        print("    UERE = %.2f m  =>  sigma_H = UERE x HDOP = %.4f m" % (sigma_ure, s_h))
        print("                       sigma_E = %.4f  sigma_N = %.4f  sigma_U = %.4f"
              % (s_e, s_n, s_v))
        print("    R = diag(%.4f, %.4f, %.4f)  [m^2]" % (s_e * s_e, s_n * s_n, s_v * s_v))
        # 各向同性假设的校验
        print("    ★ sigma_H/sqrt(2) = %.4f  vs  sigma_E = %.4f  （差 %.2f%%）"
              % (s_h / math.sqrt(2.0), s_e,
                 100.0 * abs(s_h / math.sqrt(2.0) - s_e) / s_e))
        print()

    print("  ⇒ HDOP^2 = Q_EE + Q_NN 是【定义】；正因为 sigma_H^2 = sigma_E^2 + sigma_N^2，")
    print("    所以【只有 Q_EE = Q_NN 时】才有 sigma_E ≈ sigma_N ≈ sigma_H/sqrt(2)。")
    print()
    print("  ★ 两个算例的 Q_EE 都不等于 Q_NN —— 连 12 颗均匀分布的『好几何』")
    print("    也有 10% 量级的差距。所以：")
    print("      · 只要 HDOP 这一个数（比如手册只给 HDOP）→ 只能套 sqrt(2) 近似；")
    print("      · 手上有完整几何 → 直接用 Q 的分块，sigma_E = UERE*sqrt(Q_EE)，")
    print("        不要为了省事把 sigma_H 对半分。")
    print("    素材里的『sigma_E ≈ sigma_N ≈ sigma_H/sqrt(2)』是个【近似】，")
    print("    加上 ≈ 号是对的；当成恒等式就会引入 10% 量级的系统性偏差。")

    sub("(b) PX4 有没有用它？—— 没有")
    print()
    print("  源码事实：")
    print("    common.h:182          gnssSample 里有 float pdop{};   ← 收得到")
    print("    gps_checks.cpp:149    pdop > req_pdop 就置失败标志     ← 只做门限")
    print("    gps_control.cpp:203   算 R 时用的是 hacc，不是 pdop   ← 不缩放 R")
    print("  ⇒ PX4 只把 DOP 当【合格/不合格】的开关（EKF2_GPS_PDOP 风格的门限），")
    print("    不用 DOP × UERE 去实时缩放 R。要做这条，得自己写。")

    sub("(c) 新息协方差匹配：从一整段新息里反解 R")
    print()
    print("  理论新息协方差  S = H P^- H^T + R")
    print("  实测新息协方差  S_obs = mean(y_k^2)   （标量情形，窗口平均）")
    print("  =>  R_hat = S_obs - mean(H P^- H^T)   （即 §10.6.1 的『残差法』）")
    print()
    n = 3000
    rng = np.random.default_rng(seed)
    r_true = 4.0                      # m^2，即 sigma = 2 m
    p_pred = 1.5                      # m^2，滤波器的先验位置方差（假设已知）
    y = rng.normal(0.0, math.sqrt(p_pred + r_true), n)
    s_obs = float(np.mean(y * y))
    r_hat = s_obs - p_pred
    print("  仿真：R 真值 = %.4f m^2 (sigma = %.2f m)，P^- = %.4f m^2，n = %d"
          % (r_true, math.sqrt(r_true), p_pred, n))
    print("  实测 mean(y^2) = %.4f m^2" % s_obs)
    print("  R_hat = %.4f - %.4f = %.4f m^2   （真值 %.4f，偏 %.1f%%）"
          % (s_obs, p_pred, r_hat, r_true, 100.0 * (r_hat - r_true) / r_true))
    print()
    print("  ★ 两点必须知道：")
    print("    ① 窗口越长估计越准，但跟踪突变的能力越差 —— 这是自适应滤波的固有矛盾。")
    print("    ② R_hat = S_obs - P^- 在 P^- 被低估时会【把误差全算到 R 头上】，")
    print("       表现为 R 越估越大、滤波器越来越不信观测（经典的 Sage-Husa 发散）。")
    print("       所以工程上要给 R_hat 加下界和上界，并且只在门限通过时更新。")
    print("       这正是 PX4 把 gps_pos_noise 做成【下界】的同一个理由。")


# ---------------------------------------------------------------------------
# 调度
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("瑞利分布换算常数的独立验算（解析 vs 400 万点 MC）", exp1),
    "2": ("手册指标 → R → PX4 参数（完整链路 + 歧义判别）", exp2),
    "3": ("静置实测 → R（样本量、时间相关性、两种取舍）", exp3),
    "4": ("DOP × UERE 实时 R + 新息协方差匹配", exp4),
}


def main(argv):
    if len(argv) > 1 and argv[1] in EXPERIMENTS:
        EXPERIMENTS[argv[1]][1]()
    else:
        for k in sorted(EXPERIMENTS):
            EXPERIMENTS[k][1]()
    print()
    print("=" * 74)
    print("  完成。本章结论一句话：")
    print("  R 的单位是方差，PX4 的参数是标准差（米）；")
    print("  手册 → sigma 的换算系数必须选对（CEP 0.849 / 2DRMS 0.354 / R95 0.409），")
    print("  选错一个方向就是把 R 填小一半，滤波器过度自信。")
    print("=" * 74)


if __name__ == "__main__":
    main(sys.argv)
