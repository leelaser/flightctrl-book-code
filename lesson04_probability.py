#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 4 周 · 01 概率论与高斯分布 —— 配套实验

    实验 1  一元高斯与归一化（数值积分验证"密度 ≠ 概率"）
    实验 2  ★★ 两个高斯相乘 —— 卡尔曼滤波的种子（本课核心）
    实验 3  ★ 协方差椭球：特征值分解告诉你椭球的轴
    实验 4  最大熵：为什么"只知道均值和方差"时该选高斯

用法：
    python lesson04_probability.py 1
    python lesson04_probability.py 2
    python lesson04_probability.py 3
    python lesson04_probability.py 4
    python lesson04_probability.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  和《NuttX/FreeRTOS 教程》一样：**能推的就不背，能算的就不猜。**

  实验 2 是本脚本的重点：它把课文 §6 那三行公式
  （K = Σ₀Hᵀ(HΣ₀Hᵀ+R)⁻¹ / μ = μ₀ + K(z-Hμ₀) / Σ = (I-KH)Σ₀）

  **用两种完全独立的方法算出来**，然后对比：
      方法 A：直接套公式
      方法 B：在网格上把两个密度乘起来，再数值求均值和方差
      方法 C：从"信息形式"（Σ⁻¹ 相加）出发算

  **三条路必须给出同一个答案** —— 这就是"KF 不是发明出来的"的证明。

  实验 4 的"最大熵"用的是变分法 + 拉格朗日乘子的数值验证：
  在"均值和方差固定"的约束下，用随机扰动去搜索，
  看熵最大的分布是不是收敛到高斯。

参考：
    EKF/Kalman_and_Bayesian_Filters_in_Python.pdf   第 4~5 章
    Probabilistic-Robotics-cn.pdf                   第 2 章
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
# 高斯的基本工具
# ---------------------------------------------------------------------------

def gauss1d(x, mu, var):
    """一维高斯密度。var 是方差（不是标准差）。"""
    return np.exp(-0.5 * (x - mu) ** 2 / var) / np.sqrt(2 * np.pi * var)


def trapz(y, x):
    """梯形积分（不依赖 scipy）。"""
    return float(np.sum((y[1:] + y[:-1]) * 0.5 * np.diff(x)))


def entropy1d(mu, var, lo, hi, n=200001):
    """数值计算一维高斯微分熵 ∫ -p·ln p dx。

    注意：这里用【自然对数】。最大熵原理里用什么底不影响"谁最大"，
    只影响数值（差一个常数因子）。
    """
    x = np.linspace(lo, hi, n)
    p = gauss1d(x, mu, var)
    p = np.maximum(p, 1e-300)          # 防止 log(0)
    return -trapz(p * np.log(p), x)


# ---------------------------------------------------------------------------
# 实验 1：一元高斯与归一化
# ---------------------------------------------------------------------------

def exp1():
    head("实验 1  一元高斯与归一化 —— 「密度 ≠ 概率」")

    print("""
  ★ 先明确一件事：p(x) 是【密度】，不是概率。
    概率是密度在区间上的【积分】。

    所以：
      · p(x) 的【值】可以大于 1
      · 概率永远是 [0,1] 之间的数
""")

    sub("① 一个 p(x) > 1 的高斯")

    mu, var = 0.0, 0.01          # σ = 0.1
    print("    取 μ = %.1f, σ² = %.4f  ⇒ σ = %.2f" % (mu, var, math.sqrt(var)))
    print()
    print("    p(μ) = 1/√(2πσ²) = %.4f" % (1.0 / math.sqrt(2 * math.pi * var)))
    print()
    print("    ★ 这个值 = %.2f > 1 —— 但它是【密度】，不是概率。"
          % (1.0 / math.sqrt(2 * math.pi * var)))
    print()

    sub("② 用数值积分验证：它仍然是合法的概率分布")

    x = np.linspace(-1.0, 1.0, 400001)
    p = gauss1d(x, mu, var)
    total = trapz(p, x)
    print("    ∫ p(x) dx  在 [-1, 1] 上 = %.8f    ← 应该是 1" % total)
    print("    （取 ±10σ 的范围，误差来自梯形法本身：%.2e）" % abs(total - 1.0))
    print()

    sub("③ 3σ 法则：概率不是密度")

    rows = []
    p1sigma = None
    for k in (1, 2, 3):
        lo, hi = mu - k * math.sqrt(var), mu + k * math.sqrt(var)
        xx = np.linspace(lo, hi, 200001)
        prob = trapz(gauss1d(xx, mu, var), xx)
        if k == 1:
            p1sigma = prob
        rows.append(["±%dσ" % k, "%.4f" % (2 * k * math.sqrt(var)),
                     "%.6f" % prob, "%.2f%%" % (100 * prob)])
    tab(rows, ["范围", "区间宽度", "积分 = 概率", "百分比"])

    print("""    （对照理论值：68.27% / 95.45% / 99.73%）""")
    print()
    print("  ⇒ 对比一下同一个分布的两个数：")
    print("        密度 p(μ)   = %.4f      ← 可以 > 1" %
          (1.0 / math.sqrt(2 * math.pi * var)))
    print("        概率 P(±1σ) = %.4f      ← 永远 ≤ 1" % p1sigma)
    print()
    print("""    ★ 这一条在后面很重要：卡尔曼滤波里写「最大化 p(x)」，
      找的是【密度的峰】（众数），不是「概率最大的点」
      —— 连续分布里每个具体点的概率都是 0。""")


# ---------------------------------------------------------------------------
# 实验 2：★★ 两个高斯相乘 —— 卡尔曼滤波的种子
# ---------------------------------------------------------------------------

def exp2():
    head("实验 2  ★★ 两个高斯相乘 = 卡尔曼滤波的全部")

    print("""
  课文 §6 说：高斯先验 × 高斯似然 = 高斯后验，而且有闭式解。

  这个实验用【三种独立方法】算同一个融合问题，看它们是否一致：

      方法 A：直接套卡尔曼公式   K = Σ₀Hᵀ(HΣ₀Hᵀ+R)⁻¹
      方法 B：把两个密度乘起来再【数值】求均值方差
      方法 C：从【信息形式】出发（Σ⁻¹ 相加）

  如果三者一致，就说明 KF 的公式不是"发明"的，是"推"出来的。
""")

    # ---- 标量情形：先验 N(10, 4)，观测 z=12 噪声 R=1 ----
    mu0, S0 = 10.0, 4.0
    z, R = 12.0, 1.0
    H = 1.0

    sub("① 问题设定（课文 §5.4 那个例子）")
    print("    先验:  x ~ N(μ₀=%.1f, Σ₀=%.1f)   （σ₀=%.2f）"
          % (mu0, S0, math.sqrt(S0)))
    print("    观测:  z = %.1f,  R = %.1f        （σ_R=%.2f）"
          % (z, R, math.sqrt(R)))
    print("    模型:  z = Hx + v,  H = %.1f" % H)

    # ---- 方法 A：卡尔曼公式 ----
    K = S0 * H / (H * S0 * H + R)
    mu_A = mu0 + K * (z - H * mu0)
    S_A = (1 - K * H) * S0

    sub("② 方法 A —— 直接套卡尔曼公式")
    print("    K  = Σ₀H/(HΣ₀H+R) = %.1f×%.1f/(%.1f×%.1f×%.1f+%.1f) = %.6f"
          % (S0, H, H, S0, H, R, K))
    print("    μ  = μ₀ + K(z-Hμ₀) = %.1f + %.6f×(%.1f-%.1f) = %.10f"
          % (mu0, K, z, H * mu0, mu_A))
    print("    Σ  = (1-KH)Σ₀ = (1-%.6f)×%.1f = %.10f" % (K, S0, S_A))

    # ---- 方法 B：数值相乘 ----
    sub("③ 方法 B —— 把两个密度乘起来，数值求矩")

    lo = min(mu0, z) - 12 * max(math.sqrt(S0), math.sqrt(R))
    hi = max(mu0, z) + 12 * max(math.sqrt(S0), math.sqrt(R))
    x = np.linspace(lo, hi, 2000001)

    prior = gauss1d(x, mu0, S0)
    # 似然 p(z|x)：把 z 看成"观测到的常数"，x 是变量
    lik = gauss1d(z, H * x, R)
    post = prior * lik                      # ★ 未归一化的后验

    Z = trapz(post, x)                      # 归一化常数
    post_n = post / Z
    mu_B = trapz(x * post_n, x)
    S_B = trapz((x - mu_B) ** 2 * post_n, x)

    print("    在 [%.2f, %.2f] 上取 %d 个格点" % (lo, hi, len(x)))
    print("    先验 × 似然 ⇒ 未归一化后验，积分 Z = %.10f" % Z)
    print("    归一化后：")
    print("      μ = ∫ x·p(x) dx     = %.10f" % mu_B)
    print("      Σ = ∫ (x-μ)²·p dx   = %.10f" % S_B)

    # ---- 方法 C：信息形式 ----
    sub("④ 方法 C —— 信息形式（Σ⁻¹ 相加）")
    info0 = 1.0 / S0
    info_lik = H * H / R
    info_post = info0 + info_lik
    S_C = 1.0 / info_post
    mu_C = S_C * (info0 * mu0 + H * z / R)

    print("    先验信息量   Σ₀⁻¹        = 1/%.1f = %.6f" % (S0, info0))
    print("    观测信息量   H²/R        = %.1f²/%.1f = %.6f" % (H, R, info_lik))
    print("    ─────────────────────────────────────────────")
    print("    后验信息量 = 两者【相加】= %.6f  ⇒ Σ = %.10f" % (info_post, S_C))
    print("    μ = Σ·(Σ₀⁻¹μ₀ + Hz/R)   = %.10f" % mu_C)
    print()
    print("    ★ 注意信息形式【全是加法】—— 这比「加权平均」更本质：")
    print("      **融合 = 把两边的信息量加起来**。")

    # ---- 对比 ----
    sub("⑤ 三种方法对比")
    rows = [
        ["A：卡尔曼公式", "%.12f" % mu_A, "%.12f" % S_A],
        ["B：数值相乘",   "%.12f" % mu_B, "%.12f" % S_B],
        ["C：信息形式",   "%.12f" % mu_C, "%.12f" % S_C],
    ]
    tab(rows, ["方法", "后验均值 μ", "后验方差 Σ"])

    d_mu = max(abs(mu_A - mu_B), abs(mu_A - mu_C))
    d_S = max(abs(S_A - S_B), abs(S_A - S_C))
    print("    最大偏差：Δμ = %.2e,  ΔΣ = %.2e" % (d_mu, d_S))
    print()
    if d_mu < 1e-6 and d_S < 1e-6:
        print("    ✅ 三者一致 —— 卡尔曼公式确实等价于「两个高斯相乘」。")
    else:
        print("    ⚠️ 偏差偏大，检查格点密度或积分范围。")

    # ---- 增益的直觉 ----
    sub("⑥ 增益 K 的直觉：把三个极端情况跑一遍")

    rows = []
    for name, s0, r in [("我的先验很准 (Σ₀≪R)", 0.01, 1.0),
                        ("两边一样准 (Σ₀=R)", 1.0, 1.0),
                        ("观测很准 (R≪Σ₀)", 1.0, 0.01)]:
        k = s0 / (s0 + r)
        rows.append([name, "%.4f" % s0, "%.4f" % r, "%.6f" % k,
                     "%.6f" % (1 - k)])
    tab(rows, ["情况", "Σ₀", "R", "K", "1-K"])
    print("""    ★ K = "我的不确定" / ("我的不确定" + "观测的不确定")
       —— 也就是【倒方差加权】的矩阵版本。

        Σ₀→0 ⇒ K→0 ⇒ 几乎不听观测
        R→0  ⇒ K→1 ⇒ 几乎全听观测
        相等 ⇒ K=0.5 ⇒ 各听一半

    **这解释了 KF 为什么"自动"调权重 —— 它不需要你调参。**
""")

    # ---- H ≠ 1 的情况（练习 4 的答案） ----
    sub("⑦ 练习 4 的答案：H = 2 时 K 是多少？")

    print("    先验 N(%.0f, %.1f)（Σ₀ = %.4f），观测 R = %.4f" % (mu0, S0, S0, R))
    print("    K(H) = Σ₀·H / (H²·Σ₀ + R)")
    print()
    rows = []
    for Hh in (0.25, 0.5, 1.0, 2.0, 4.0):
        h2s = Hh * Hh * S0
        k = S0 * Hh / (h2s + R)
        rows.append(["%.2f" % Hh, "%.4f" % h2s, "%.4f" % (Hh * S0),
                     "%.4f" % (h2s + R), "%.6f" % k, "%.6f" % (1 - k)])
    tab(rows, ["H", "H²Σ₀", "HΣ₀", "S", "K", "1-K"])

    H_star = math.sqrt(R / S0)
    K_star = S0 * H_star / (H_star * S0 * H_star + R)
    print("    ★ K 对 H【不是】单调下降的 —— 它在 H* = √(R/Σ₀) = %.4f 处取最大值 %.6f，"
          % (H_star, K_star))
    print("      两侧都往下掉。所以「H 变大 K 反而变小」只在 H > H* 那一侧成立。")
    print()
    print("""    ★ 两侧各说一遍（别记反）：

      H > H*（本题 H=2 ⇒ K=0.470588，确实比 H=1 的 0.800000 小）：
        H=2 意味着「x 变动 1，观测就变动 2」⇒ 观测对 x 的【敏感度】翻倍。
        等效方差 R/H² = 0.25 看起来「更准」，但 S = H²Σ₀ + R 里
        H²Σ₀ 涨得更快（×4）⇒ 净效果 K ≈ Σ₀H/(H²Σ₀) = 1/H → 0。
        直觉：观测太敏感 ⇒ x 的一点点误差都被放大成巨大的新息
        ⇒ 滤波器分不清「新息大」是 x 错了还是观测噪声 ⇒ 反而不敢信它。

      H < H*（H=0.5 恰好落在峰值上 ⇒ K=1.000000）：
        H 越小，观测对 x 越「麻木」⇒ 里面根本没有 x 的信息 ⇒ K→0。

    ★ 为什么峰值在 H*？用信息形式一眼看出来：
        S = H²Σ₀ + R 在 H* 处满足 H²/R = 1/Σ₀
        ⇒ 【观测带来的信息量 = 先验的信息量】（Σ = Σ₀/2，后验方差砍一半）
        ⇒ 这时新息被最充分地利用，K 取到最大值。
      ⚠ 注意 K 在这里不是「权重」——H ≠ 1 时 K 带量纲（K = Σ₀H/S），
        甚至可以超过 1。要看「听先验还是听观测」的比例，
        应该看 KH = H²Σ₀/S ∈ [0,1)，而不是 K 本身。
        本题 H=0.5 时 KH = 0.5，正好「一人一半」✓。
""")


# ---------------------------------------------------------------------------
# 实验 3：协方差椭球
# ---------------------------------------------------------------------------

def exp3():
    head("实验 3  ★ 协方差椭球：特征值分解告诉你椭球的轴")

    print("""
  协方差矩阵 Σ 的几何意义是【不确定性椭球】：

      Σ = V Λ Vᵀ
        V 的列  → 椭圆的轴【方向】（特征向量）
        Λ 的对角 → 轴的【长度平方】（特征值）

    本实验用三种 Σ 验证这件事。
""")

    cases = [
        ("各向同性（正圆）", np.array([[4.0, 0.0], [0.0, 4.0]])),
        ("轴对齐（不相关但不等）", np.array([[4.0, 0.0], [0.0, 1.0]])),
        ("相关（椭球被拧了）", np.array([[2.0, 1.5], [1.5, 2.0]])),
    ]

    for name, S in cases:
        sub(name)
        print("    Σ = [[%.2f, %.2f]," % (S[0, 0], S[0, 1]))
        print("         [%.2f, %.2f]]" % (S[1, 0], S[1, 1]))
        print()

        # 相关系数
        rho = S[0, 1] / math.sqrt(S[0, 0] * S[1, 1])
        print("    相关系数 ρ = %.4f" % rho)

        # 特征值分解（对称矩阵用 eigh，保证实特征值）
        lam, V = np.linalg.eigh(S)
        # eigh 返回升序，倒过来让 λ₁ 是最大的
        lam = lam[::-1]
        V = V[:, ::-1]

        print("    特征值 λ = [%.4f, %.4f]" % (lam[0], lam[1]))
        print("    轴长 √λ  = [%.4f, %.4f]" % (math.sqrt(lam[0]), math.sqrt(lam[1])))
        print("    特征向量（列）:")
        print("        v₁ = [%.4f, %.4f]   倾角 %.2f°"
              % (V[0, 0], V[1, 0], math.degrees(math.atan2(V[1, 0], V[0, 0]))))
        print("        v₂ = [%.4f, %.4f]" % (V[0, 1], V[1, 1]))
        print()
        print("    对角元（方差）= [%.4f, %.4f]" % (S[0, 0], S[1, 1]))
        print()

        if abs(S[0, 1]) < 1e-12:
            print("    ⇒ 非对角元为 0 ⇒ 不相关 ⇒ 椭球的轴【对齐坐标轴】")
        else:
            print("    ⇒ 非对角元非 0 ⇒ 相关 ⇒ 椭球【被拧了】%.1f°"
                  % math.degrees(math.atan2(V[1, 0], V[0, 0])))
        print()

    sub("练习 5：构造一个「倾斜 45°」的 Σ")

    # 45° ⇒ 长轴方向 (1,1)/√2，短轴 (1,-1)/√2
    # 取 λ = [4, 1]，按 Σ = V Λ Vᵀ 反算
    ang = math.radians(45)
    V = np.array([[math.cos(ang), -math.sin(ang)],
                  [math.sin(ang), math.cos(ang)]])
    lam = np.diag([4.0, 1.0])
    S = V @ lam @ V.T

    print("    按 Σ = VΛVᵀ 反算，取 λ = [4, 1]、旋转 45°：")
    print("        Σ = [[%.4f, %.4f]," % (S[0, 0], S[0, 1]))
    print("             [%.4f, %.4f]]" % (S[1, 0], S[1, 1]))
    print()
    print("    ★ 注意这个矩阵长什么样：")
    print("        对角元都是 %.2f（相等）—— 但椭球【不是】正圆！" % S[0, 0])
    print("        非对角元 %.2f —— 就是这个把它拧过去了" % S[0, 1])
    print()
    print("    ⇒ **「对角元相等」不等于「各向同性」** —— ")

    # 验证分解回来
    lam2, V2 = np.linalg.eigh(S)
    lam2 = lam2[::-1]
    V2 = V2[:, ::-1]
    print("    特征值分解验证：λ = [%.4f, %.4f]  （应该回到 [4, 1]）"
          % (lam2[0], lam2[1]))
    print("    轴方向 = [%.4f, %.4f]，倾角 %.2f°  （应该是 45°）"
          % (V2[0, 0], V2[1, 0],
             math.degrees(math.atan2(V2[1, 0], V2[0, 0]))))
    print()

    if _HAS_MPL:
        _plot_ellipses(cases)
    else:
        print("    （没装 matplotlib，跳过画图）")
        print()


_HAS_MPL = False
_CJK = False
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as _fm

    # ★ 让 matplotlib 能画中文。找不到中文字体就退回英文标签
    #   （不静默画出一堆方框 —— 那比报错还难看）。
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
    """有中文字体就用中文，否则用英文。"""
    return zh if _CJK else en


def _plot_ellipses(cases):
    """把三个椭球画在一张图上（存成 png）。"""
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.4))
    t = np.linspace(0, 2 * np.pi, 400)
    unit = np.vstack([np.cos(t), np.sin(t)])

    titles = [
        _t("各向同性（正圆）", "isotropic (circle)"),
        _t("轴对齐（不相关）", "axis-aligned (uncorrelated)"),
        _t("相关（椭球被拧了）", "correlated (tilted)"),
    ]

    for ax, (name, S), ti in zip(axes, cases, titles):
        lam, V = np.linalg.eigh(S)
        lam = lam[::-1]
        V = V[:, ::-1]
        # 1σ 椭圆
        pts = V @ np.diag(np.sqrt(lam)) @ unit
        ax.plot(pts[0], pts[1], "b-", lw=2, label="1σ")
        # 2σ
        pts2 = V @ np.diag(np.sqrt(lam)) @ unit * 2
        ax.plot(pts2[0], pts2[1], "b--", lw=1, alpha=0.6, label="2σ")

        # 画轴
        for i in range(2):
            v = V[:, i] * math.sqrt(lam[i])
            ax.arrow(0, 0, v[0], v[1], head_width=0.08, head_length=0.12,
                     fc="r", ec="r", length_includes_head=True)

        ax.set_title(ti, fontsize=11)
        ax.set_xlabel("x₁")
        ax.set_ylabel("x₂")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.axhline(0, color="k", lw=0.5)
        ax.axvline(0, color="k", lw=0.5)
        lim = 4.5
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)

    plt.tight_layout()
    out = "covariance_ellipses.png"
    plt.savefig(out, dpi=110)
    plt.close(fig)
    print("    ✅ 图已存：%s%s" % (out, "" if _CJK else "（无中文字体，标签用英文）"))
    print()


# ---------------------------------------------------------------------------
# 实验 4：最大熵 —— 为什么选高斯
# ---------------------------------------------------------------------------

def exp4():
    head("实验 4  最大熵：为什么「只知道均值和方差」时该选高斯")

    print("""
  课文 §3.2 说高斯无处不在有三个原因，第三个是【最大熵原理】：

      在"只知道均值和方差"的前提下，高斯是熵最大的分布
      —— 也就是"最不假设"的那个。

  这个实验分两步验证：
      ① 用离散分布做【数值搜索】，看约束下熵最大的是不是"像高斯"
      ② 对比几个同均值同方差的分布，看谁的熵最大
""")

    sub("① 同均值同方差，比熵")

    mu, var = 0.0, 1.0

    # 用大范围数值积分算微分熵
    lo, hi, n = -30.0, 30.0, 3000001

    def num_entropy_from_pdf(fn):
        x = np.linspace(lo, hi, n)
        p = fn(x)
        p = np.maximum(p, 1e-300)
        return -trapz(p * np.log(p), x)

    # 分布 1：高斯（理论上熵最大）
    h_gauss = num_entropy_from_pdf(lambda x: gauss1d(x, mu, var))

    # 分布 2：同方差的均匀分布  U(-a, a)，方差 = a²/3 ⇒ a = √(3var)
    a = math.sqrt(3 * var)

    def unif(x):
        p = np.where(np.abs(x) <= a, 1.0 / (2 * a), 0.0)
        return p

    h_unif = num_entropy_from_pdf(unif)

    # 分布 3：同方差的双峰（两个高斯各半）
    #   混合 N(-d, v) 和 N(+d, v)，各 0.5 权重
    #   总方差 = v + d²  ⇒ 取 v 使总方差 = var
    d = 0.8
    v = var - d * d
    if v > 0:
        def bimodal(x):
            return 0.5 * gauss1d(x, -d, v) + 0.5 * gauss1d(x, d, v)
        h_bi = num_entropy_from_pdf(bimodal)
    else:
        h_bi = None

    # 分布 4：拉普拉斯，同方差 ⇒ b = √(var/2)
    b = math.sqrt(var / 2.0)

    def laplace(x):
        return np.exp(-np.abs(x) / b) / (2 * b)

    h_lap = num_entropy_from_pdf(laplace)

    print("    理论值：高斯 N(0,1) 的微分熵 = ½ln(2πeσ²) = %.6f"
          % (0.5 * math.log(2 * math.pi * math.e * var)))
    print()

    rows = [
        ["高斯 N(0,1)", "%.6f" % h_gauss, "—"],
        ["均匀 U(-√3,√3)", "%.6f" % h_unif,
         "%.4f%%" % (100 * (h_gauss - h_unif) / h_gauss)],
        ["拉普拉斯(同方差)", "%.6f" % h_lap,
         "%.4f%%" % (100 * (h_gauss - h_lap) / h_gauss)],
    ]
    if h_bi is not None:
        rows.append(["双峰（同方差）", "%.6f" % h_bi,
                     "%.4f%%" % (100 * (h_gauss - h_bi) / h_gauss)])

    tab(rows, ["分布", "微分熵 ∫-p·ln p", "比高斯低多少"])

    print("""    ★ 结论：**同均值同方差下，高斯的熵最大。**
      （数值误差范围内 —— 上表的差都应该是正的）

      含义：**用高斯 = 做了最少的额外假设。**
      如果你在"只知道均值和方差"时选了一个非高斯分布，
      你其实偷偷假设了更多的信息 —— 而那些信息你并没有。
""")

    sub("② 反过来：如果知道的是「取值范围」呢？")

    print("""    均匀分布是"只知道上下界"时的最大熵分布。

        已知              最大熵分布
        ──────────────    ──────────
        均值和方差        高斯          ← 飞控里的情形
        取值上下界        均匀分布
        x > 0 且均值已知   指数分布
        取值 {1..n}       离散均匀

    ⇒ **"该选哪个分布"不是喜好问题，是"你知道什么约束"的问题。**

      飞控里为什么是高斯？因为：
        · 噪声是很多微小独立扰动之和（CLT ⇒ 确实接近高斯）
        · 我们通常只知道它的均值和方差（不知道上下界）
      两条都对上了。
""")

    sub("③ 熵的单位与「信息量」的直觉")

    print("""    熵 H = -∫ p ln p dx 的单位是【奈特】（nat，用自然对数）。

    ★ 一个有用的换算：
        1 比特 = ln 2 ≈ 0.693 奈特
        高斯 N(0,σ²) 的熵 = ½ln(2πeσ²)
        σ 每缩小为 1/2，熵减少 ½ln 4 ≈ 0.693 奈特 ≈ 1 比特

    ⇒ **"观测让不确定性减半" ≈ "获得了 1 比特信息"**。
      这是信息论和估计理论之间的那座桥（卡尔曼滤波的互信息解释）。

    ⚠️ 注意：微分熵可以为【负】—— 因为密度可以 > 1。
       （离散熵永远 ≥ 0，微分熵没有这个性质，别混。）
""")

    # 验证 σ 减半 ⇒ 熵减 ln2
    h1 = entropy1d(0, 1.0, -30, 30, 1000001)
    h2 = entropy1d(0, 0.25, -30, 30, 1000001)     # σ 减半 ⇒ 方差 1/4
    print("    数值验证：")
    print("        H(N(0,1))    = %.6f" % h1)
    print("        H(N(0,0.25)) = %.6f   （σ 减半）" % h2)
    print("        差          = %.6f   （ln 2 = %.6f）"
          % (h1 - h2, math.log(2)))
    print()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("一元高斯与归一化", exp1),
    "2": ("★★ 两个高斯相乘 = KF 的种子", exp2),
    "3": ("★ 协方差椭球与特征值分解", exp3),
    "4": ("最大熵：为什么选高斯", exp4),
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
