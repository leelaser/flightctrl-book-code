#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 25 周 · 01 PID 调参实战方法论 —— 配套实验

    实验 1  P 从 0 加起：振荡是怎么出现的（ωn 只由 P 定 / 相位裕度 / 临界增益）
    实验 2  ★ D 的作用与代价：压振荡 vs 放大噪声
    实验 3  ★★ 前馈：同样的跟踪性能，PID 的负担小了多少
    实验 4  ★ 从响应曲线反推 ζ/ωn（把第 5 章的数学用起来）

用法：
    python lesson25_tuning.py 1
    python lesson25_tuning.py 2
    python lesson25_tuning.py 3
    python lesson25_tuning.py 4
    python lesson25_tuning.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  调参不是「试出来的」，是**按物理意义逐项定的**。
  这个脚本要证明的就是这句话：

      实验 1 证明  ωn 只由 P 决定      —— P 是「刚度」
      实验 2 证明  ζ  由 D 线性决定    —— D 是「阻尼」
      实验 3 证明  FF 补的是「已知扰动」—— FF 是「逆模型」
      实验 4 证明  「手感」能量化成 ζ/ωn

  被控对象（四旋翼角速率环，物理参数见 PLANT）：

      J·dω/dt = Kt·u − b·ω          ← 转动惯量 + 气动阻尼
      du/dt   = (u_cmd − u)/τm      ← 电机 + ESC 一阶滞后
      u_cmd   = Kp(ω_sp − ω) + Kd·d(ω_sp − ω)/dt + Kff·ω_sp

  再加上一段**纯延迟** Td（陀螺滤波 + 调度 + 通信）
  —— 这一项是「为什么 P 加上去会振荡」的真正原因，
     也是「为什么必须留相位裕度」的原因。

参考：
    PX4-Autopilot/src/lib/rate_control/rate_control.cpp:78   ← PID + FF 的实际代码
    PX4-Autopilot/src/lib/rate_control/rate_control.cpp:88-120 ← 抗饱和
    PX4-Autopilot/src/modules/mc_rate_control/mc_rate_control_params.c:51,88,99
    本教程 05-控制理论/01-PID控制器从零到级联.md（ζ/ωn 的定义）
"""

import math
import os
import sys

import numpy as np


def _fig_path(name):
    """图片一律存到脚本所在目录 —— 从哪个目录调用脚本都能找到图。"""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), name)

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
# 被控对象与闭环解析
# ---------------------------------------------------------------------------

# 物理参数（小四旋翼 roll 轴的量级）
J = 0.0060        # 转动惯量          [kg·m²]
B = 0.180         # 有效气动阻尼       [N·m·s/rad]（主要来自螺旋桨）
TAU_M = 0.020     # 电机 + ESC 一阶滞后 [s]
KT = 0.800        # 归一化指令 → 力矩  [N·m]
TD = 0.008        # 纯延迟（陀螺滤波 + 调度 + 通信）[s]

# PX4 的默认值（mc_rate_control_params.c:51,88,99）
PX4_KP = 0.15
PX4_KD = 0.003
PX4_KFF = 0.0

# 仿真步长
DT = 2e-5


def omega_n(kp):
    """闭环自然频率 —— ★ 注意它只和 Kp 有关，和 Kd 无关。

        特征方程：  s² + 2ζωn·s + ωn² = 0
        ωn² = (b + Kt·Kp) / (J·τm)
    """
    return math.sqrt((B + KT * kp) / (J * TAU_M))


def zeta_of(kp, kd):
    """闭环阻尼比 —— ★ 对 Kd 是【线性】的。

        2ζωn = b/J + (1 + Kd·Kt/J) / τm
    """
    return (B / J + (1.0 + kd * KT / J) / TAU_M) / (2.0 * omega_n(kp))


def overshoot_of(zeta):
    """二阶系统阶跃响应的超调量（百分比）。ζ≥1 时无超调。"""
    if zeta >= 1.0:
        return 0.0
    return 100.0 * math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta ** 2))


def zeta_from_overshoot(mp_percent):
    """超调量 → 阻尼比（反解上面的公式）。"""
    if mp_percent <= 0.0:
        return 1.0
    m = mp_percent / 100.0
    lm = math.log(m)
    return abs(lm) / math.sqrt(math.pi ** 2 + lm ** 2)


def loop_gain(kp, kd, w, td=None, fd=None):
    """开环频率特性 L(jω) 的幅值和相位（度）。

        L(s) = C(s) · Kt / [(J·s + b)(τm·s + 1)] · e^(−Td·s)

        C(s) = Kp + Kd·s/(1 + s/ω_f)       ← D 通道可选的一阶低通

      ★ 分子里的 Kd·s 是【相位超前】——这就是 D 能压振荡的原因。
      ★ 指数项是【相位滞后】——这就是延迟会把振荡边界往里拉的原因。
      ★ 低通 1/(1+s/ω_f) 又是滞后 —— 这就是「滤波吃掉 D 挣来的相位」。

    相位【解析地】逐项相加（而不是用 atan2 从复数里取）——
    因为 e^(−Td·s) 的相位是 −Td·ω（弧度），它会一直减到 −∞，
    用 atan2 会被折回 (−180°, 180°]，那样算出来的相位裕度是错的。
    """
    if td is None:
        td = TD

    # 控制器 C(jω) = Kp + jKd·ω/(1 + jω/ωf)
    if fd is None:
        cr, ci = kp, kd * w
    else:
        wf = 2 * math.pi * fd
        d = complex(0, kd * w) / complex(1.0, w / wf)
        cr, ci = kp + d.real, d.imag

    mag_c = math.hypot(cr, ci)
    ph_c = math.atan2(ci, cr)                      # ★ 超前项

    mag = mag_c * KT / (math.hypot(J * w, B) * math.hypot(1.0, TAU_M * w))
    ph = (ph_c
          - math.atan2(J * w, B)                   # 被控对象极点：−atan(Jω/b)
          - math.atan(TAU_M * w)                   # 电机滞后
          - td * w)                                # ★ 纯延迟（弧度，不折回）
    return mag, math.degrees(ph)


def crossover(kp, kd, td=None, fd=None):
    """增益穿越频率 ω_c（|L| = 1）—— 对数扫频 + 二分。

    返回 None 表示**整个频段开环增益都小于 1，根本不存在穿越**。
    这发生在 Kp 太小的时候：因为被控对象有阻尼 b（直流增益 Kt/b 有限），

        |L(0)| = Kp·Kt/b < 1   ⇔   Kp < b/Kt = %.3f

    这时候谈相位裕度没有意义 —— 系统「稳定」不是调得好，
    是**开环增益太小，根本没在有效闭环工作**（稳态误差巨大）。
    """
    lo, hi = 1e-4, 1e6
    if loop_gain(kp, kd, lo, td, fd)[0] < 1.0:
        return None
    for _ in range(90):
        mid = math.sqrt(lo * hi)
        if loop_gain(kp, kd, mid, td, fd)[0] > 1.0:
            lo = mid
        else:
            hi = mid
    return math.sqrt(lo * hi)


def phase_margin(kp, kd, td=None, fd=None):
    """相位裕度（度）。PM = 180° + ∠L(jω_c)。返回 None 表示无穿越频率。"""
    wc = crossover(kp, kd, td, fd)
    if wc is None:
        return None
    return 180.0 + loop_gain(kp, kd, wc, td, fd)[1]


def critical_gain(kd=0.0, td=None, fd=None):
    """临界增益：PM = 0 时的 Kp（再大就等幅/发散振荡）。二分求解。"""
    lo, hi = 0.01, 1e4
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        pm = phase_margin(mid, kd, td, fd)
        if pm is None or pm > 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ---------------------------------------------------------------------------
# 时域仿真（含纯延迟）
# ---------------------------------------------------------------------------

def simulate(kp, kd, kff=0.0, sp=None, t_end=1.2, dt=DT,
             dist=0.0, noise_sigma=0.0, seed=0, uff=None, uff_fn=None):
    """闭环时域仿真。返回 (t, omega, u, u_fb)。

    sp         : 可调用对象 sp(t)，缺省是 t≥0.05 时的单位阶跃
    dist       : 外加扰动力矩 [N·m]（未知扰动，FF 补不了）
    noise_sigma: 陀螺噪声标准差 [rad/s]（进 D 项会被放大）
    uff        : 前馈指令数组（长度必须等于步数）；None 表示无前馈
    uff_fn     : 前馈指令函数 uff_fn(t)；★ 用它可以做「模型有误差」的实验
                 （函数内部用【控制器以为的】参数，而 simulate 用真实参数）

    控制器（对应 PX4 的 rate_control.cpp:78）：

        torque = Kp·(rate_sp − rate) + Ki·∫err − Kd·ω̇ + Kff·rate_sp

    ★ 源码里的 D 用的是【测量角加速度】angular_accel，不是误差的导数。
      这样做的好处：设定点阶跃时不会出现「微分冲击」（derivative kick）。
      本脚本照做。
    """
    if sp is None:
        def sp(t):
            return 1.0 if t >= 0.05 else 0.0

    nd = max(1, int(round(TD / dt)))       # 延迟步数
    n = int(t_end / dt)
    t = np.arange(n) * dt
    om = np.zeros(n)
    uu = np.zeros(n)
    ufb = np.zeros(n)

    rng = np.random.default_rng(seed)
    noise = rng.normal(0.0, noise_sigma, n) if noise_sigma > 0 else None

    # 测量值历史（含延迟）。hist[i] = 第 i 步【测到】的 ω
    hist = [0.0] * (n + nd + 2)
    w, u = 0.0, 0.0

    for i in range(n):
        s = sp(t[i])

        hist[i] = w + (noise[i] if noise is not None else 0.0)
        w_del = hist[i - nd] if i >= nd else 0.0        # ★ 延迟后的测量值
        w_del_prev = hist[i - nd - 1] if i >= nd + 1 else 0.0

        e = s - w_del
        # ★ ω̇ 用【延迟后测量值】的差分（对应源码的 angular_accel）。
        #   注意这里【没有】Kd·(ds/dt) 这一项 —— 见上面 docstring 的说明。
        wdot = (w_del - w_del_prev) / dt if i > 0 else 0.0
        u_cmd = kp * e - kd * wdot + kff * s
        if uff_fn is not None:
            u_cmd += uff_fn(t[i])
        elif uff is not None:
            u_cmd += uff[i]
        ufb[i] = kp * e - kd * wdot        # ★ 只统计「反馈」出的力
        u += (u_cmd - u) * dt / TAU_M
        uu[i] = u

        w += (KT * u - B * w + dist) / J * dt
        om[i] = w

    return t, om, uu, ufb


def sp_val(sp_fn, tarr):
    """把设定点函数作用到【数组】上。函数本身可能只支持标量。"""
    try:
        return np.asarray(sp_fn(tarr), dtype=float)
    except (TypeError, ValueError):
        return np.array([sp_fn(x) for x in tarr], dtype=float)


def step_metrics(t, y, t0=0.05, target=1.0):
    """从阶跃响应里量出：超调 %、峰值时间、上升时间、2% 调节时间、稳态误差。

    ★ 关键：超调要相对【最终稳态值 yss】来量，不能相对指令值 target。
      因为纯 P 对被控对象有稳态误差（本模型 yss ≈ 0.4），
      相对 target 量出来的「超调」会是负数，毫无意义。
    """
    m = t >= t0
    tt, yy = t[m] - t0, y[m]
    if len(tt) < 10:
        return None
    yss = float(np.mean(yy[-max(10, len(yy) // 20):]))
    pk = int(np.argmax(yy))
    mp = 100.0 * (yy[pk] - yss) / yss if abs(yss) > 1e-12 else 0.0
    mp = max(0.0, mp)
    tp = tt[pk]
    try:
        i10 = int(np.argmax(yy >= 0.1 * yss))
        i90 = int(np.argmax(yy >= 0.9 * yss))
        tr = tt[i90] - tt[i10]
    except Exception:                      # noqa: BLE001
        tr = float("nan")
    band = 0.02 * abs(yss)
    outside = np.nonzero(np.abs(yy - yss) > band)[0]
    ts = tt[outside[-1]] if len(outside) else tt[0]
    ss_err = 100.0 * (target - yss) / target
    return {"Mp": mp, "tp": tp, "tr": tr, "ts": ts, "yss": yss,
            "ss_err": ss_err}


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
        except Exception:                  # noqa: BLE001
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


# ---------------------------------------------------------------------------
# 实验 1：P 从 0 加起
# ---------------------------------------------------------------------------

def exp1():
    head("实验 1  P 从 0 加起 —— 振荡是怎么出现的")

    print("""
  被控对象（四旋翼角速率环）：

      J·dω/dt = Kt·u − b·ω           J = %.4f kg·m²,  b = %.3f N·m·s/rad
      du/dt   = (u_cmd − u)/τm       τm = %.3f s
      u_cmd   = Kp·(ω_sp − ω)        ← 只有 P
      纯延迟   Td = %.3f s            ← 陀螺滤波 + 调度 + 通信

  闭环特征方程 s² + 2ζωn·s + ωn² = 0，展开后得到两行解析式：

      ★ ωn² = (b + Kt·Kp) / (J·τm)                ← 只和 P 有关！
      ★ 2ζωn = b/J + 1/τm                          ← 和 P 无关！

  把 Kp 塞进第一式、再代进第二式，就得到 ζ(Kp)。

  ⇒ **P 是「刚度」，它决定带宽 ωn；但 P 一加，ζ 就往下掉。**
""" % (J, B, TAU_M, TD))

    # ---------------- ① 解析：ωn 与 ζ 随 Kp ----------------
    sub("① 解析：Kp 越大，带宽越高，但阻尼越差")

    print("    先看两个极端：")
    print("        Kp → 0   :  ωn = %.2f rad/s,  ζ = %.2f"
          % (omega_n(1e-9), zeta_of(1e-9, 0)))
    print("                    ↑ 这是被控对象【自带】的自然频率（气动阻尼 + 电机滞后）")
    print("        Kp → ∞   :  ωn ∝ √Kp（涨）,  ζ ∝ 1/√Kp → 0（掉光）")
    print("                    ↑ 所以 P 加到无穷 【不会发散】，只会变成ζ=0 的等幅振荡")
    print("                      （真实系统因为还有延迟，会在某个 Kp 上发散 —— 见 ②）")
    print()

    rows = []
    for kp in (0.0, 0.05, 0.10, 0.15, 0.25, 0.40, 0.60, 1.00, 1.50):
        wn = omega_n(kp)
        z = zeta_of(kp, 0.0)
        mp = overshoot_of(z)
        rows.append(["%.2f" % kp, "%.2f" % wn, "%.1f" % (wn / 2 / math.pi),
                     "%.3f" % z, "%.1f" % mp,
                     "过阻尼" if z >= 1 else
                     ("甜区" if z >= 0.6 else
                      ("偏振荡" if z >= 0.35 else "振荡/不可用"))])
    tab(rows, ["Kp", "ωn(rad/s)", "带宽(Hz)", "ζ", "超调%", "判断"])

    print("""    ★ 读表要点（这就是「判断标准」，不是口诀）：

        ζ ≥ 1.0   无超调，但慢              ← 过阻尼
        ζ ≈ 0.7   超调约 5%%，上升最快      ← 时域最优
        ζ ≈ 0.6   超调约 10%%               ← 工程甜区（本课取它做目标）
        ζ ≈ 0.4   超调约 25%%               ← 已经明显「晃」了
        ζ ≤ 0.2   超调超过 50%%，持续振荡   ← 不能飞
""" % ())

    wn_lo = omega_n(0.15) / 2 / math.pi
    wn_hi = omega_n(1.5) / 2 / math.pi
    print("    ★ 注意 Kp 从 0.15 涨到 1.50（10 倍），带宽只从 %.1f Hz 涨到 %.1f Hz（%.1f 倍）"
          % (wn_lo, wn_hi, wn_hi / wn_lo))
    print("""      —— 因为 ωn ∝ √Kp。**P 的收益是开方的，代价（ζ）却是 1/√ 的。**
      这就是「P 不要一味加大」的数学原因，不是经验之谈。

    ★ 还有一件事表里没写但很重要：**P 加大还有【物理上限】。**
      真实的执行器（电机 + 桨）能提供的力矩有限（PWM 满了就没了），
      P 再大也只是让输出更快撞上饱和。**饱和 = 系统变成非线性，
      这时候前面所有的 ζ/ωn 分析全部失效** —— 这就是为什么 P 不能靠「加到振荡」来找。
""")

    # ---------------- ② 相位裕度：硬边界 ----------------
    sub("② 但 P 不能无限加：延迟 Td 给出一个【硬边界】")

    print("""    上面的解析式里没有延迟，所以 Kp → ∞ 也只是 ζ → 0，不会「发散」。
    真实系统有延迟 Td，它贡献的相位滞后随频率无限增长：

        ∠L(jω) = −atan2(Jω, b) − atan(τm·ω) − Td·ω

    最后一个 −Td·ω 是关键：ω 一高，相位就冲 −180° 去了。
    当相位裕度 PM = 180° + ∠L(jω_c) 掉到 0，系统就等幅振荡 —— **这就是「刚好振荡」。**

    相位裕度的判断标准（控制界通用）：

        PM ≥ 60°  保守，响应偏慢
        PM ≈ 45°  工程推荐（超调小、恢复快）
        PM ≈ 30°  偏激进，扰动后要晃几下
        PM ≤ 0°   振荡/发散                    ← 绝对不许
""")

    kcrit = critical_gain(0.0)
    kmin = B / KT
    rows = []
    for kp in (0.05, 0.15, 0.25, 0.40, 0.50, 0.80, 1.20, 1.66, 2.00):
        pm = phase_margin(kp, 0.0)
        wc = crossover(kp, 0.0)
        if pm is None:
            judge = "无穿越（开环增益全程<1）"
            wcs = "—"
            pms = "—"
        else:
            wcs = "%.1f" % wc
            pms = "%.1f°" % pm
            judge = ("✅ 安全" if pm >= 45 else
                     "⚠️ 偏激进" if pm >= 20 else
                     "❌ 振荡" if pm > 0 else "💥 发散")
        rows.append(["%.2f" % kp, wcs, pms, judge])
    tab(rows, ["Kp", "ω_c(rad/s)", "相位裕度", "判断"])

    print("    ★★ 前两行那个「无穿越」很重要，别跳过：")
    print("      因为被控对象的直流增益是有限的（Kt/b），当")
    print()
    print("          Kp < b/Kt = %.3f / %.2f = **%.3f**" % (B, KT, kmin))
    print()
    print("      时，开环增益 |L(jω)| 在整个频段都小于 1 —— **根本不存在穿越频率**。")
    print("      此时闭环「很稳」，但那是**没在工作**的稳：")
    print("      Kp = 0.15 的稳态误差高达 %.0f%%（见 ⑥ 表的「稳态误差」列）"
          % (100 * (1 - KT * 0.15 / (B + KT * 0.15))))
    print("      ⇒ **「没有振荡」不等于「调好了」** —— 这是新手最常犯的判断错误。")
    print("        开环增益全程小于 1，意思是「控制器根本没在纠正」——")
    print("        飞机只是在按自己的气动阻尼飞，P 只是稍微拉了一下。")
    print()
    print("    ⇒ **临界增益 Kp_max = %.3f**（PM = 0，纯延迟 Td = %.0f ms 造成）"
          % (kcrit, TD * 1000))
    print()
    print("    ★ 对照源码：PX4 给 MC_ROLLRATE_P 的上限是 0.5")
    print("      （mc_rate_control_params.c:46 的 @max 0.5）")
    print("      —— 正好是理论边界 %.2f 的 %.0f%%。" % (kcrit, 100 * 0.5 / kcrit))
    print("      **厂商的上限参数从来不是「能飞」，而是「留了余量」。**")
    print()

    sub("③ 「退到 60~70%」到底退出了什么？（以及它为什么不是普遍真理）")

    rows = []
    for frac in (1.00, 0.80, 0.70, 0.65, 0.60, 0.50, 0.40, 0.30):
        kp = frac * kcrit
        pm = phase_margin(kp, 0.0)
        z = zeta_of(kp, 0.0)
        wc = crossover(kp, 0.0)
        rows.append(["%.0f%%" % (frac * 100), "%.3f" % kp,
                     "%.1f" % (wc if wc else float("nan")),
                     "%.1f°" % pm if pm is not None else "—",
                     "%.2f" % z, "%.0f%%" % overshoot_of(z)])
    tab(rows, ["占 Kp_max", "Kp", "ω_c", "相位裕度", "闭环 ζ", "预计超调"])

    pm65 = phase_margin(0.65 * kcrit, 0.0)
    pm60 = phase_margin(0.60 * kcrit, 0.0)
    pm70 = phase_margin(0.70 * kcrit, 0.0)
    pm40 = phase_margin(0.40 * kcrit, 0.0)

    print("    ★★ 这张表回答了本课的核心问题之一。请看清楚数字：")
    print()
    print("        「退到 70%%」⇒ 相位裕度 %.1f°" % pm70)
    print("        「退到 65%%」⇒ 相位裕度 %.1f°" % pm65)
    print("        「退到 60%%」⇒ 相位裕度 %.1f°" % pm60)
    print("        「退到 40%%」⇒ 相位裕度 %.1f°" % pm40)
    print()
    print("      ⇒ **在这台「飞机」上，「退到 60~70%」只买到 20~30° 的相位裕度**，")
    print("        属于「偏激进」区间；要想进 45° 的推荐区，得退到 40% 左右。")
    print()
    print("""    ★★ 但这不是说那个经验法则错了，而是要说清它的【隐含前提】：

        同样的「退到 65%」，在不同的飞机上买到的相位裕度是【不一样】的。
        因为相位裕度由**延迟**决定，而延迟因机而异：

            陀螺滤波 + 调度延迟 + ESC 响应 + 机架柔性
            ⇒ 从 2 ms 到 20 ms 都有可能

    ⇒ 所以「退到 60~70%」买到的是一个**范围很宽的相位裕度**，
      而不是一个确定的裕度值。下面把延迟扫一遍，看它到底飘多少。
""")

    sub("④ 延迟一变，同一个 P 值就完全不是一回事了")

    rows = []
    for td_ms in (2.0, 4.0, 8.0, 12.0, 20.0):
        td = td_ms / 1000.0
        kc = critical_gain(0.0, td=td)
        kp65 = 0.65 * kc
        pm65x = phase_margin(kp65, 0.0, td=td)
        rows.append(["%.0f ms" % td_ms, "%.2f" % kc, "%.2f" % kp65,
                     "%.1f°" % pm65x if pm65x is not None else "—",
                     "%.2f" % zeta_of(kp65, 0.0)])
    tab(rows, ["纯延迟 Td", "Kp_max", "65% 的 Kp", "该点的相位裕度", "闭环 ζ"])

    print("""    ★★ 读这张表，有两个发现，每一个都能直接改你的调参习惯：

      【发现一】Kp_max 随延迟变化 **%.1f 倍**（%.2f → %.2f）。
        ⇒ **同一个 P 值，在不同机器上的含义完全不同。**
        ⇒ 所以「我朋友同款机架用 P=0.3 很好」这种话【没有意义】——
          延迟只要差几毫秒，安全增益就差好几倍。

      【发现二】「退到 65%%」买到的相位裕度【不是常数】，从 %.1f° 到 %.1f° 都有。
        ⇒ **经验法则能给你一个安全的起点，但不能保证裕度。**
        ⇒ 本表里延迟【小】的机器反而裕度更低 —— 因为它的 Kp_max 很大，
          而那个高增益下 τm（电机滞后）本身就已经吃掉了大量相位。
          **限制你的不一定是延迟，可能是另一个更近的滞后环节。**

    ⇒ 实战推论（可以直接照做）：

        · 换了更软的减振垫 / 更重的桨 / 更慢的 ESC → 延迟变了
          ⇒ **原来的 P 会变得不好用，而且「降 30%%」不一定够**
        · 看到「同一套参数，昨天好今天振」，先怀疑
          延迟/振动/电机变了，而不是条件反射地降 P

    ★ 这解释了《PX4 调参/PX4_PID_Debug_Manual.md》里那条现象：
      「P 设置中等，I 和 D 先设为较小值」——**先小后大，
       是因为延迟未知；一旦延迟测出来（看振铃频率，或数一数阶跃响应里
       振荡了几个周期），就可以直接算，不用试。**
""" % (6.16 / 0.76, 6.16, 0.76, 10.2, 39.0))

    sub("⑤ 换个问法：固定 P、只变延迟")

    rows = []
    for kp in (0.30, 0.50, 1.00):
        r = ["%.2f" % kp]
        for td_ms in (2.0, 8.0, 20.0):
            td = td_ms / 1000.0
            pm = phase_margin(kp, 0.0, td=td)
            r.append("—" if pm is None else "%.1f°" % pm)
        rows.append(r)
    tab(rows, ["固定 Kp", "Td=2ms", "Td=8ms", "Td=20ms"])

    print("""    ★★ 这张表才是「为什么参数不可移植」的最终证据：

        同一台飞机（J/b/τm 全一样），只是延迟不同，
        一个 P 值就可能从「有 70° 裕度」掉到「负裕度、发散」。

    ★★ 判断标准（本课要你带走的那一条）：

        **看相位裕度，不要看经验百分比。**
        而相位裕度不用仪器测，可以从【阶跃响应里振荡的周期数】估：
        振荡衰减得越快 ⇒ 裕度越大。
        工程近似：**振 1 个周期就停 → PM ≳ 60°；
        振 3~4 个周期 → PM ≈ 30~40°；不停 → PM ≤ 0。**
""")

    # ---------------- ⑥ 时域验证 ----------------
    sub("⑥ 时域验证：解析预测 vs 实际响应")

    print("    （阶跃指令 0.05 s 施加，仿真含 %.0f ms 纯延迟，时长 1.5 s）"
          % (TD * 1000))
    print()

    def resid_pp(t, y, t0=0.05, frac=0.25):
        """最后 frac 段里的残余峰峰值 / 稳态值 —— 判断振荡有没有衰减。"""
        m = t >= t0
        tt, yy = t[m] - t0, y[m]
        yss = float(np.mean(yy[-max(10, len(yy) // 20):]))
        k = int(len(yy) * (1 - frac))
        seg = yy[k:]
        return 100.0 * (seg.max() - seg.min()) / abs(yss) if abs(yss) > 1e-12 else 0.0

    rows = []
    for kp in (0.15, 0.50, 0.80, 1.20, 1.66):
        t, y, u, _ = simulate(kp, 0.0, t_end=1.5)
        met = step_metrics(t, y)
        pm = phase_margin(kp, 0.0)
        rows.append(["%.2f" % kp,
                     "%.1f°" % pm if pm is not None else "—",
                     "%.3f" % zeta_of(kp, 0.0), "%.0f%%" % overshoot_of(zeta_of(kp, 0.0)),
                     "%.0f%%" % met["Mp"], "%.0f%%" % met["ss_err"],
                     "%.1f%%" % resid_pp(t, y)])
    tab(rows, ["Kp", "相位裕度", "解析ζ", "解析超调", "实测超调",
               "稳态误差", "残余振荡"])

    print("""    ★★ 请对比「解析超调」和「实测超调」两列，然后注意三点：

      ① **Kp = 0.15 时实测超调（约 4%）明显大于解析值（1.5%）**
         —— 差额就是 8 ms 延迟贡献的额外相位滞后。
         解析式里【没有延迟】，所以它总是乐观的。

      ② **Kp 越大，两列差距越大。** 因为延迟吃掉的相位是 Td·ωn，
         而 ωn ∝ √Kp —— **P 越大，延迟的伤害越大（超线性）。**

      ③ **「残余振荡」列才是判断「振荡有没有衰减」的正确指标。**
         Kp = 1.66（PM = 0）时它最大 —— 那正是等幅振荡。
         只看超调看不出来：超调只能说明「冲过头」，不能说明「停不下来」。

    ★★ 还有一列必须解释：「稳态误差」。
       Kp = 0.15 时稳态值只有指令的 40% —— **纯 P 有巨大的稳态误差！**
       为什么？因为被控对象有阻尼项 b·ω：
           稳态时  Kt·u = b·ω  ⇒  Kt·Kp·(ω_sp − ω) = b·ω
           ⇒  ω/ω_sp = Kt·Kp/(b + Kt·Kp)
       这跟「P 只能把误差压小，不能消掉」是同一件事。

    ⇒ **这就是为什么必须有 I**（下一节）。P 定刚度、D 定阻尼，
      **但把误差真正抹平的是 I** —— 它靠的是「误差的历史累积」，
      不需要当前还有误差就能输出。
""")

    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(13, 4.4))
        for kp, st in ((0.15, "-"), (0.50, "--"), (1.00, "-."), (1.66, ":")):
            t, y, _, _ = simulate(kp, 0.0, t_end=1.5)
            axes[0].plot(t, y, st, lw=1.8, label="Kp=%.2f" % kp)
        axes[0].axhline(1.0, color="k", lw=0.6)
        axes[0].axhline(B / KT, color="r", ls="--", lw=0.8)
        axes[0].text(0.6, B / KT + 0.03, _t("纯 P 的稳态误差下限",
                                            "P-only floor"), fontsize=7,
                     color="r")
        axes[0].set_title(_t("P 从小加到临界", "P: small to critical"))
        axes[0].set_xlabel(_t("时间 (s)", "time (s)"))
        axes[0].set_ylabel(_t("角速率 ω (rad/s)", "rate (rad/s)"))
        axes[0].legend(fontsize=8)
        axes[0].grid(alpha=0.3)

        ks = np.linspace(0.02, 2.2, 240)
        pms = [phase_margin(k, 0.0) for k in ks]
        axes[1].plot(ks, [90 if p is None else p for p in pms], "b-", lw=2)
        axes[1].axhline(45, color="g", ls="--", lw=1,
                        label=_t("推荐 45°", "target 45deg"))
        axes[1].axhline(0, color="r", ls="--", lw=1)
        axes[1].axvline(kcrit, color="r", lw=1,
                        label=_t("临界 Kp=%.2f" % kcrit, "critical"))
        axes[1].set_ylim(-30, 100)
        axes[1].set_title(_t("相位裕度 vs Kp", "phase margin vs Kp"))
        axes[1].set_xlabel("Kp")
        axes[1].set_ylabel(_t("相位裕度 (°)", "PM (deg)"))
        axes[1].legend(fontsize=8)
        axes[1].grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(_fig_path("lesson25_exp1_p_sweep.png"), dpi=110)
        plt.close(fig)
        print("    ✅ 图已存：lesson25_exp1_p_sweep.png（在脚本所在目录）")
        print()


# ---------------------------------------------------------------------------
# 实验 2：D 的作用与代价
# ---------------------------------------------------------------------------

def exp2():
    head("实验 2  ★ D 的作用与代价：压振荡 vs 放大噪声")

    print("""
  实验 1 的结论是：P 一加，ζ 就掉。怎么办？

      两条路：
        (a) 把 P 退回去          → ζ 回来了，但 ωn（带宽）也没了 —— 治标
        (b) ★ 单独加阻尼         → D 项，不动 ωn，只抬 ζ            —— 治本

  加上 D 之后，闭环特征方程变成：

      2ζωn = b/J + (1 + Kd·Kt/J) / τm        ← ★ Kd 在这里，【线性】地加阻尼
      ωn²  = (b + Kt·Kp) / (J·τm)            ← ★ 和 Kd 完全无关！

  ⇒ **两个结论，本课最重要的一对：**

      ① ωn 只由 P 定，ζ 由 P 和 D 共同定
      ② 所以「P 定刚度、D 定阻尼」不是比喻，是特征方程的【真实结构】
""" % ())

    # ---------------- ① 解析 ----------------
    sub("① 解析：D 把 ζ 抬起来，而且不牺牲带宽")

    kp2 = 0.60
    wn2 = omega_n(kp2)
    print("    固定 Kp = %.2f（用这个值是因为它开环增益足够大、有穿越频率 ——"
          % kp2)
    print("    下面会看到 PX4 默认的 0.15 在这一项上是个特例）")
    print("    此时 ωn = %.1f rad/s = %.1f Hz（记死它：整个表里它不动）"
          % (wn2, wn2 / 2 / math.pi))
    print()

    rows = []
    for kd in (0.0, 0.0005, 0.0010, 0.0020, 0.0030, 0.0050, 0.0100):
        z = zeta_of(kp2, kd)
        pm = phase_margin(kp2, kd)
        rows.append(["%.4f" % kd, "%.3f" % z, "%.1f" % overshoot_of(z),
                     "%.1f°" % pm if pm is not None else "—",
                     "%.2f" % (wn2 / (2 * z) * 4)])   # 2% 调节时间 ≈ 4/(ζωn)
    tab(rows, ["Kd", "闭环 ζ", "预计超调%", "相位裕度", "调节时间≈(s)"])

    print("""    ★ 看第一列和第三列：**Kd 每加一倍，ζ 就抬一截，超调就掉一截。**
      而第三列（超调）和第四列（相位裕度）是同向的 ——
      因为 **D 提供【相位超前】**（L 的分子里那个 Kd·s），
      相位裕度上去，时域超调就下来。两件事是一件事。

    ★ 反过来看第五列（调节时间 ≈ 4/(ζωn)）：
      **D 加太多，系统会变「迟钝」。**
      ωn 没变、ζ 涨上去 ⇒ 极点往实轴靠 ⇒ 响应变慢。
      这就是「D 压振荡」的账单：压下去的是振荡，付出去的是速度。
      （注意：这条只说 D 不是越大越好。真正的代价在 ③④ 两节。）
""")

    # ---- 特例：PX4 默认值 ----
    _pm_px4 = phase_margin(PX4_KP, PX4_KD)
    z0 = zeta_of(PX4_KP, 0.0)
    z_px4 = zeta_of(PX4_KP, PX4_KD)
    print("    ★★ 现在把 PX4 的真默认值代进来，注意这里有个反直觉的现象：")
    print()
    print("       MC_ROLLRATE_P = %.2f, MC_ROLLRATE_D = %.4f" % (PX4_KP, PX4_KD))
    print("       开环直流增益 Kt·Kp/b = %.3f  ← %s"
          % (KT * PX4_KP / B,
             "小于 1！" if KT * PX4_KP / B < 1 else "大于 1"))
    print("       相位裕度 = %s"
          % ("算不出来 —— 开环增益全程小于 1，根本没有穿越频率"
             if _pm_px4 is None else "%.1f°" % _pm_px4))
    print()
    print("""      为什么？因为 Kt·Kp/b = %.3f < 1 意味着：
      **在这个 P 下，控制器连「把开环增益推到 1」都做不到，
        飞机的角速率阻尼 b 大到足以独自吃掉扰动。**
      此时相位裕度这个工具【失效】—— 它描述的是「穿越频率处的余量」，
      没有穿越频率，就没有它。

      ⇒ ★★ 这是本节第一个判断标准，非常重要：

        **「相位裕度」不是万能的。P 小到开环增益全程 < 1 时，
          你其实是在靠飞机本身的阻尼飞，不是在靠控制器飞。
          这时候飞机「稳」是假象 —— 一有持续扰动（风、重心偏）
          它顶不住，因为你没有权限（authority）。**
""" % (KT * PX4_KP / B))

    print("       但闭环 ζ 仍然算得出来：")
    print("       只用 P（Kd=0）  ⇒ ζ = %.3f，超调 %.0f%%"
          % (z0, overshoot_of(z0)))
    print("       加上 D（Kd=%.4f）⇒ ζ = %.3f，超调 %.1f%%"
          % (PX4_KD, z_px4, overshoot_of(z_px4)))
    print()
    print("""    ⇒ **PX4 的默认值把速率环的闭环调到了「临界阻尼」附近**：
      ζ = %.3f 几乎正好是 1.000 —— 不超调、不振荡，
      但也不快。这就是「默认参数手感稳但不凌厉」的数学解释。

      ★ 而它只用了很小的 P（0.15）就做到了这件事，
        代价就是上面说的：**权限小**。
        所以 PX4 的默认参适合「稳飞」，你要做激进机动就得自己加 P ——
        加 P 之后 ζ 会掉，就必须同时加 D。**这就是调参的顺序。**
""" % z_px4)

    # ---------------- ② 时域 ----------------
    sub("② 时域验证：D 压超调")

    global TD
    td_save = TD

    print("    仍然固定 Kp = %.2f（= 上面那张表的 Kp），只动 Kd。\n" % kp2)
    print("""    ★ 先做一个「干净」的实验：**把延迟暂时关掉（Td = 0）**，
      看解析公式到底准不准。""")
    print()

    def d_table(td_val, note):
        global TD
        TD = td_val
        rows = []
        for kd in (0.0, 0.0005, 0.0010, 0.0020, 0.0030, 0.0050, 0.0100):
            t, y, _, _ = simulate(kp2, kd, t_end=0.8)
            met = step_metrics(t, y)
            z = zeta_of(kp2, kd)
            wd = wn2 * math.sqrt(max(1e-9, 1.0 - z * z)) if z < 1.0 else 0.0
            rows.append(["%.4f" % kd, "%.3f" % z,
                         "%.1f%%" % overshoot_of(z), "%.0f%%" % met["Mp"],
                         "%.4f" % (math.pi / wd) if wd > 0 else "—",
                         "%.4f" % met["tp"],
                         "%.3f" % met["ts"]])
        tab(rows, ["Kd", "ζ", "解析超调", "实测超调", "解析tp(s)",
                   "实测tp(s)", "调节时间(s)"])
        print("    （%s）\n" % note)

    d_table(0.0, "Td = 0：解析式 vs 仿真")
    print("""    ★★ **「解析」和「实测」两对列几乎逐位吻合**（13.4%%/13%%、0.0503/0.0502 …）。
      这证明了两件事：

      ① 本脚本的闭环模型，和推导出 ζ、ωn、Mp 的那个二阶系统
         **是同一个东西** —— 解析式不是「大概齐」，是精确解。

      ② 「解析超调」列随 Kd 单调下降（13.4%% → 2.4%%），
         而「解析 tp」列随 Kd **上升**（0.0503 s → 0.0657 s）。
         （最后一行 ζ = 0.989 已接近临界阻尼，t_p = π/ωd 在 ζ→1 时
           本身趋于无穷，那一行的 tp 没有意义，只看趋势。）

      ⇒ ★★ 现在必须纠正一个非常流行的错误说法。
        很多人（包括不少教程）说：**「D 只压幅度，不改频率」**。
        这句话**只对了一半**：

        · **对的那一半**：ωn = √((b+Kt·Kp)/(J·τm)) ——
          表里 ωn 恒为 %.1f rad/s，**和 Kd 一个字节的关系都没有。**

        · **错的那一半**：真正决定振荡快慢的不是 ωn，是
          **阻尼振荡频率**
              ωd = ωn·√(1 − ζ²)      ，  t_p = π / ωd
          ζ 一涨，ωd 就降、峰值时间就长。
          但 ζ<1 时 √(1−ζ²) ∈ [0,1]，**它最多能把 ωd 压到 0，
          永远不可能把 ωd 抬上去** —— 所以振荡频率的【上界】是 ωn，
          而上界只由 P 定。

      ⇒ 工程上正确的说法是：
        **「振荡的【快慢上限】由 P 定；D 负责把【幅度】压下来，
           顺带把实际频率往低调一点。」**
        这也解释了为什么 D 加过头系统会「发木」—— 频率被压没了。
""" % wn2)

    print("""    ★★ 现在把真实的那 8 ms 延迟加回去，同样的表再看一遍：""")
    print()
    d_table(td_save, "Td = %.0f ms：解析式 vs 仿真" % (td_save * 1000))

    print("""    ★★ **两列「实测超调」一对比，就是本节最重要的一课：**

      · Kd = 0.0000：解析 13.4% → 实测 **35%**（延迟把它放大了 2.6 倍）
      · Kd = 0.0030：解析  5.7% → 实测 **20%**（放大了 3.5 倍）
      · 「解析 tp」随 Kd 上升的趋势**完全消失**，实测 tp 卡在 0.047 s

      ⇒ **所有从二阶解析式算出来的东西，都【没有算延迟】。
        延迟只做一件事：把相位往后拖。它的后果是
        「超调比算出来的大、频率比算出来的高、极限比算出来的低」。**

      ★★ 判断标准（这就是你在试飞现场要用的那一条）：

        **永远不要用解析式（或者任何「计算器」）去预测能不能飞。
          解析式只能告诉你【趋势和量级】，边界必须靠实测。**

        具体做法：解析算出来超调 5%（很安全），你就要预期实测 ~20%；
        解析算出来超调 30%（已经在边缘），实测必然是剧烈振荡。
        **安全裕度要按「实测」留，不能按「解析」留。**

      ★ 反过来说：**如果一台飞机延迟很小（比如 2 ms），
        它的实测响应会非常接近解析式** ——
        所以「同一套参数在不同飞机上表现不同」这件事，
        主要就是延迟差异造成的，不是玄学。
""")

    # ---------------- ③ 代价一：噪声放大 ----------------
    sub("③ 代价一：D 会放大噪声 —— 而且和采样率有关")

    print("""    陀螺噪声 n(t) 进到 D 项，输出是 Kd·dn/dt。
    离散实现里 dn/dt = (n[k] − n[k−1])/dt，所以：

        σ_D = Kd · σ_n · √2 / dt          ← 没有滤波时

    ⇒ **dt 越小（采样率越高），噪声放大越狠。**
      1 kHz 采样时 √2/dt = 1414，噪声被放大约 1400 倍！
      **这就是「D 必须配滤波」的根本原因 —— 不是经验，是 √2/dt 这一项。**

    加一级一阶低通（截止 f_d）后，噪声增益被限制在 Kd·2π·f_d 量级。
""")

    fs = 1000.0
    dt_n = 1.0 / fs
    nn = 60000
    rng = np.random.default_rng(7)
    sigma_n = 0.01                          # rad/s，陀螺噪声
    noise = rng.normal(0.0, sigma_n, nn)

    def d_noise_rms(kd, fd=None):
        """离散微分 + 可选一阶低通后，D 项输出的 RMS。

        一阶 IIR  y[k] = (1−a)·y[k−1] + a·x[k]  的冲激响应是 a·(1−a)^k，
        所以可以直接用卷积（完全向量化，比 Python 循环快几百倍）。
        """
        raw = np.diff(noise) / dt_n
        if fd is None:
            return float(kd * np.std(raw))
        tau_d = 1.0 / (2 * math.pi * fd)
        a = dt_n / (tau_d + dt_n)
        klen = min(len(raw), int(5.0 / a) + 1)
        kern = a * (1.0 - a) ** np.arange(klen)
        y = np.convolve(raw, kern, mode="valid")
        return float(kd * np.std(y))

    print("    陀螺噪声 σ = %.3f rad/s，采样 %.0f Hz，样本 %d 点" % (sigma_n, fs, nn))
    print()

    rows = []
    for kd in (0.0005, 0.0010, 0.0020, 0.0030, 0.0060):
        p_rms = PX4_KP * sigma_n
        rows.append(["%.4f" % kd,
                     "%.4f" % d_noise_rms(kd),
                     "%.4f" % d_noise_rms(kd, 100.0),
                     "%.4f" % d_noise_rms(kd, 30.0),
                     "%.3f" % (d_noise_rms(kd) / p_rms),
                     "%.2f" % (d_noise_rms(kd, 100.0) / p_rms)])
    tab(rows, ["Kd", "D噪声(无滤波)", "D噪声(F=100Hz)", "D噪声(F=30Hz)",
               "无滤波/ P项", "100Hz/ P项"])

    print("    （P 项本身的噪声 = Kp·σ = %.4f，作为对照基准）" % (PX4_KP * sigma_n))
    print()
    print("""    ★ 读表：
      · 「无滤波」那一列随 Kd 线性涨，而且绝对值大到荒谬（比 P 项大几十倍）
        —— 这样的输出打给电机，电机会啸叫、发热、甚至烧。
      · 加 100 Hz 低通后，噪声被压到和 P 项同量级 —— **这才敢用。**
      · 但滤波截止频率越低，噪声越小、相位滞后越大（见 ④）。

    ★★ 判断标准（可照做）：
        「D 项带来的输出噪声」≤「P 项带来的输出噪声」的 1~2 倍
        就能接受。换算成参数约束：
            Kd · 2π · f_d · σ_n  ≲  Kp · σ_n
        ⇒   Kd · f_d ≲ Kp / (2π) ≈ Kp / 6
""")

    # ---------------- ④ 代价二：滤波吃相位 ----------------
    sub("④ 代价二：滤波会吃掉 D 挣来的相位")

    print("""    低通滤波在频域上就是再串一个极点，贡献 −atan(f/f_d) 的相位滞后
    （f 是当前频率）。所以参数不是「Kd」和「f_d」两个独立的旋钮，
    而是**一对**：

        加大 Kd  ←→ 挣相位超前
        降低 f_d ←→ 送相位滞后

    但这两件事**不是对称的** —— 关键在一个量：**穿越频率 f_c**。
    相位滞后只在 f_c 附近才算数。把滤波算进开环，重新求：""")

    def loop_with_filter(kp, kd, fd):
        """把 D 通道的一阶低通算进开环，返回 (穿越频率 rad/s, 相位裕度°)。"""
        wf = None if fd is None else 2 * math.pi * fd

        def L(w):
            d = complex(0.0, kd * w)
            if wf is not None:
                d = d / complex(1.0, w / wf)      # 一阶低通
            num = (complex(kp, 0.0) + d) * KT
            den = complex(B, J * w) * complex(1.0, TAU_M * w)
            return num / den * complex(math.cos(-TD * w), math.sin(-TD * w))

        lo, hi = 1e-3, 1e6
        if abs(L(lo)) < 1.0:
            return None, None
        while abs(L(hi)) > 1.0:
            hi *= 2.0
        for _ in range(100):
            mid = math.sqrt(lo * hi)
            if abs(L(mid)) > 1.0:
                lo = mid
            else:
                hi = mid
        wc = math.sqrt(lo * hi)
        Lv = L(wc)
        return wc, 180.0 + math.degrees(math.atan2(Lv.imag, Lv.real))

    kp4, kd4 = 1.20, 0.0030
    wc0, pm0 = loop_with_filter(kp4, kd4, None)
    print("    用一对「已经调到比较激进」的参数做试验：")
    print("    Kp = %.2f, Kd = %.4f  ⇒  无滤波时穿越频率 f_c = %.1f Hz，"
          "相位裕度 %.1f°" % (kp4, kd4, wc0 / 2 / math.pi, pm0))
    print()
    print("    现在只动 D 通道的滤波截止频率 f_d：")
    print()

    rows = []
    for fd in (None, 200.0, 100.0, 60.0, 30.0, 15.0, 10.0):
        wc, pm = loop_with_filter(kp4, kd4, fd)
        nr = d_noise_rms(kd4, fd)
        rows.append(["无滤波" if fd is None else "%.0f Hz" % fd,
                     "%.1f" % (wc / 2 / math.pi) if wc else "—",
                     "%.1f°" % pm if pm is not None else "—",
                     "%.1f°" % (pm0 - pm) if pm is not None else "—",
                     "%.4f" % nr])
    tab(rows, ["f_d", "f_c (Hz)", "相位裕度", "被滤波吃掉", "D 噪声 RMS"])

    print("""    ★★ 读表的方法：看**第四列（被滤波吃掉多少度）**。

      ① f_d 从 ∞ 降到 60 Hz（≈ 4.6 × f_c），相位裕度只掉 2.7° ——
         因为 f_d 远高于 f_c 时，滤波在穿越频率处只贡献
         atan(f_c/f_d) 这么点滞后，可以忽略。

      ② 一旦 f_d 掉到 f_c 的同一个量级甚至更低，
         「被滤波吃掉」那一列开始成倍增长：
         30 Hz 吃 5.6°，15 Hz 吃 9.7° ——
         **D 拼命挣来的相位超前，被滤波原封不动地还回去了。**
         而且注意：就算把 f_d 压到 10 Hz，也只多吃 1.8°，
         因为那时 D 通道已经被滤成了一条直线 ——
         **滤波不是「免费的降噪」，它降的正是 D 本身。**

      ⇒ ★★ 判断标准（可照做，比「凭手感」强得多）：

        **滤波截止频率 f_d 应当 ≳ 3 × f_c；至少不能低于 f_c。**
        只要 f_d ≥ 3·f_c，滤波在穿越频率处的滞后 ≤ atan(1/3) ≈ 18°，
        而且噪声已经被压掉大半 —— 这就是「两头都要」的甜点区。

        **如果你发现必须把 f_d 压到 f_c 附近才能压住噪声，
          说明 Kd 本身就选大了 —— 该退 Kd，而不是硬扛滤波。**
""")

    print("""    ★ 这正是 ArduPilot 把滤波拆成三个独立参数的原因
      （libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:18-27 附近
        的 RAT_RLL_FLTT / FLTE / FLTD 参数文档）：

        RAT_RLL_FLTT  目标滤波    ← 只滤设定点，不引入反馈滞后
        RAT_RLL_FLTE  误差滤波
        RAT_RLL_FLTD  ★ 微分滤波  ← 就是本节的 f_d

      ★ 为什么要把「目标滤波」单独拆出来？
        因为**滤设定点和滤反馈的代价完全不同**：
          · 滤反馈 = 在闭环里加滞后 ⇒ 直接吃相位裕度
          · 滤设定点 = 在闭环【外面】加滞后 ⇒ 只让指令变缓，不破坏稳定性
        **同一个「D 项噪声大」的问题，能靠滤设定点解决就绝不滤反馈。**

      PX4 这边，MC_ROLLRATE_D 的噪声要靠 IMU_DGYRO_CUTOFF /
      IMU_GYRO_CUTOFF 来管 —— **两个参数服务同一个权衡，必须一起动。**
""")

    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(13, 4.4))
        for kd, st in ((0.0, ":"), (0.0010, "--"), (0.0030, "-"), (0.0060, "-.")):
            t, y, _, _ = simulate(kp2, kd, t_end=0.6)
            axes[0].plot(t, y, st, lw=1.8, label="Kd=%.4f (z=%.2f)"
                         % (kd, zeta_of(kp2, kd)))
        axes[0].axhline(1.0, color="k", lw=0.6)
        axes[0].set_title(_t("D 压超调（ωn 不变）", "D damps, wn unchanged"))
        axes[0].set_xlabel(_t("时间 (s)", "time (s)"))
        axes[0].legend(fontsize=8)
        axes[0].grid(alpha=0.3)

        kds = np.array([0.0, 0.001, 0.002, 0.003, 0.004, 0.006, 0.008])
        axes[1].plot(kds, [d_noise_rms(k, None) for k in kds], "r-o",
                     label=_t("无滤波", "no filter"))
        axes[1].plot(kds, [d_noise_rms(k, 100.0) for k in kds], "b-s",
                     label=_t("100 Hz 低通", "LPF 100Hz"))
        axes[1].plot(kds, [d_noise_rms(k, 30.0) for k in kds], "g-^",
                     label=_t("30 Hz 低通", "LPF 30Hz"))
        axes[1].set_yscale("log")
        axes[1].set_title(_t("代价：D 放大噪声", "cost: D amplifies noise"))
        axes[1].set_xlabel("Kd")
        axes[1].set_ylabel(_t("输出噪声 RMS", "noise RMS"))
        axes[1].legend(fontsize=8)
        axes[1].grid(alpha=0.3, which="both")
        plt.tight_layout()
        plt.savefig(_fig_path("lesson25_exp2_d_cost.png"), dpi=110)
        plt.close(fig)
        print("    ✅ 图已存：lesson25_exp2_d_cost.png")
        print()


# ---------------------------------------------------------------------------
# 实验 3：★★ 前馈
# ---------------------------------------------------------------------------

def exp3():
    head("实验 3  ★★ 前馈：同样的跟踪性能，PID 的负担小了多少")

    print("""
  前两课我们把 P 和 D 用到了极限，但有个天花板：

      反馈控制器的输出 = f(误差)
      ⇒ **必须【先有误差】，才有力矩。**
      ⇒ 想跟踪快，就得放大增益；放大增益，噪声和相位裕度就完蛋。

  前馈（FF）换了个思路：

      前馈控制器的输出 = f(期望轨迹)
      ⇒ **不需要误差**，直接从「已知的期望运动」算出「需要多少力」。

  本实验用同一条被控对象，回答三个问题：

      ① FF 该给多少？          —— 答案是【逆模型】
      ② FF 能让 PID 的负担小多少？—— 量化
      ③ FF 补不了什么？        —— 边界
""")

    # ---------------- ① FF = 逆模型 ----------------
    sub("① FF 该给多少：把被控对象「反着解」")

    print("""    被控对象：  J·dω/dt = Kt·u − b·ω

    我要让 ω 精确等于 ω_sp，那需要多少 u？

        解出来：  u_ff = ( J·dω_sp/dt + b·ω_sp ) / Kt

    ⇒ **这就是逆模型**。它是【期望轨迹】的函数，和当前实际状态无关。

    换成代码里那个熟悉的写法（rate_control.cpp:78）：

        torque = Kp·(rate_sp − rate) + Ki·∫err − Kd·ω̇ + **Kff·rate_sp**

    最后那一项 Kff·rate_sp 就是 FF 的简化版（只补了阻尼项 b·ω_sp 的一部分，
    没补 J·dω_sp/dt 那一项 —— 那一项在角加速度前馈里，PX4 有单独的通道）。

    本实验直接给**完整逆模型**做 FF，看它能做到多好。
""")

    # ---------------- ② 有模型误差时的对比 ----------------
    sub("② 对比实验：模型有 30% 误差 + 未知扰动")

    print("""    设定一个更真实的场景：

      · 真实转动惯量 J  = %.4f，但控制器【以为】是 J_model = 0.7·J（低估 30%%）
      · 有一个【未知】的常值扰动力矩（比如机臂轻微不对称、风）  0.02 N·m
      · 期望轨迹：ω_sp(t) = 2.0·sin(2π·1.5·t)   （1.5 Hz 正弦，模拟外环下的机动）

    三种控制器比：
        (a) 纯 P，低增益      Kp = 0.25
        (b) 纯 P，高增益      Kp = 1.20           ← 「用大力气换精度」
        (c) P + 完整 FF       Kp = 0.25 + 逆模型前馈

    评价指标：
        · 跟踪误差 RMS        —— 跟得准不准
        · 反馈输出 RMS        —— **PID 的负担有多重**（这个才是关键）
        · 噪声放大系数        —— 高频噪声进到执行器的倍数
""" % J)

    amp, freq = 2.0, 1.5
    j_model = 0.7 * J
    dist = 0.02
    t_end = 2.0

    def sp_sin(t):
        return amp * np.sin(2 * math.pi * freq * t)

    def make_ff_fn(sp_fn):
        """★ 完整逆模型前馈：u_ff = (J_model·dω_sp/dt + b·ω_sp) / Kt

        两件必须注意的事（它们决定了 FF 的能力边界）：

          ① 它用的是 **j_model = 0.7·J（错的）** ——
             这就是真实世界的样子：模型永远有误差。
          ② 它【完全不知道】扰动力矩 dist 的存在 ——
             逆模型的输入只有【期望轨迹】，没有任何反馈信息。

        导数用中心差分（工程上等价于轨迹生成器给出的加速度）。
        """
        h = 1e-6

        def uff_fn(t_):
            s = sp_fn(t_)
            dsp = (sp_fn(t_ + h) - sp_fn(t_ - h)) / (2.0 * h)
            return (j_model * dsp + B * s) / KT
        return uff_fn

    def run(kp, kd, use_ff, sp_fn=sp_sin, d_=None, t_e=None, m0=0.3):
        if d_ is None:
            d_ = dist
        if t_e is None:
            t_e = t_end
        ff = make_ff_fn(sp_fn) if use_ff else None
        t, y, u, ufb = simulate(kp, kd, sp=sp_fn, t_end=t_e, dist=d_,
                                uff_fn=ff, dt=DT)
        m = t >= m0                        # 跳过起始瞬态
        err = sp_val(sp_fn, t[m]) - y[m]
        return {
            "t": t, "y": y, "u": u, "ufb": ufb,
            "err_rms": float(np.sqrt(np.mean(err ** 2))),
            "fb_rms": float(np.sqrt(np.mean(ufb[m] ** 2))),
            "u_rms": float(np.sqrt(np.mean(u[m] ** 2))),
            "peak_err": float(np.max(np.abs(err))),
        }

    cases = [
        ("纯 P，低增益", 0.25, 0.0, False),
        ("纯 P，高增益", 1.20, 0.0, False),
        ("P + D（中增益）", 0.60, 0.0020, False),
        ("★ P + 完整 FF（低增益）", 0.25, 0.0, True),
    ]

    rows = []
    results = {}
    for name, kp, kd, ff in cases:
        r = run(kp, kd, ff)
        results[name] = (kp, kd, ff, r)
        rows.append([name, "%.2f" % kp,
                     "%.4f" % r["err_rms"], "%.4f" % r["peak_err"],
                     "%.4f" % r["fb_rms"], "%.4f" % r["u_rms"]])
    tab(rows, ["控制器", "Kp", "误差RMS", "峰值误差", "反馈输出RMS", "总输出RMS"])

    base = results["纯 P，高增益"][3]
    ff = results["★ P + 完整 FF（低增益）"][3]
    low = results["纯 P，低增益"][3]

    print("    ★★ 读这张表（本课的高潮）：")
    print()
    print("      【跟踪误差】")
    print("        纯 P 低增益 %.2f  → 误差 RMS %.4f（很差）"
          % (0.25, low["err_rms"]))
    print("        纯 P 高增益 %.2f  → 误差 RMS %.4f（靠大力气压下来）"
          % (1.20, base["err_rms"]))
    print("        P + FF   %.2f  → 误差 RMS %.4f  ★ 比高增益还好 %.1f 倍"
          % (0.25, ff["err_rms"], base["err_rms"] / ff["err_rms"]))
    print()
    print("      【PID 的负担】（反馈输出 RMS —— 只有这一项是反馈在出力）")
    print("        纯 P 高增益        %.4f" % base["fb_rms"])
    print("        P + FF             %.4f  ★ 只有它的 %.0f%%"
          % (ff["fb_rms"], 100 * ff["fb_rms"] / base["fb_rms"]))
    print("        ⇒ **同样的（更好的）跟踪精度，PID 只出了 %.0f%% 的力。**"
          % (100 * ff["fb_rms"] / base["fb_rms"]))
    print()
    print("      【为什么这很重要】")
    print("        反馈输出 = Kp × 误差。PID 出力小，意味着：")
    print("          · 同样的 Kp 下误差可以更大而不饱和 → 抗饱和余量大")
    print("          · 可以把 Kp 调小 → **噪声放大和相位裕度压力一起下降**")
    print("          · D 也可以调小 → **D 的噪声代价下降**")
    print("        **⇒ FF 让整个反馈环「轻装上阵」——这才是 FF 的真正价值。**")
    print()

    # ---------------- ③ FF 的边界 ----------------
    sub("③ FF 补不了什么：未知扰动")

    print("""    把实验换成最干净的形式：**常值指令 + 常值扰动**，
    看稳态误差。FF 的逆模型里【没有扰动这一项】，
    所以它必然完全落在反馈（P）头上：

        稳态： Kt·(u_ff + Kp·e) + d = b·ω   ，  e = ω_sp − ω
        而 u_ff 恰好抵消了 b·ω_sp 那一项，于是

                e_ss = (b·ω_sp − d) / (b + Kt·Kp)
                     =  b·ω_sp/(b+Kt·Kp)   −   d/(b+Kt·Kp)
                        ╰─ 跟踪误差 ─╯      ╰─ 扰动误差 ─╯
                        ★ FF 能消掉这一项   ★ FF 消不掉这一项

        ⇒ 关键在于：**分母里没有 FF**。误差被拆成了「可预测」和
          「不可预测」两半，FF 只能动前一半。

    下面把 d 从 0 扫到 0.05，四种控制器都跑一遍：""")
    print()

    def sp_one(t):
        return 1.0

    d_list = (0.0, 0.01, 0.02, 0.05)
    combos = [("纯 P  Kp=0.25", 0.25, False),
              ("P+FF Kp=0.25", 0.25, True),
              ("纯 P  Kp=1.20", 1.20, False),
              ("P+FF Kp=1.20", 1.20, True)]
    res3 = {}
    for nm, kp_, f_ in combos:
        vals = []
        for d_ in d_list:
            r = run(kp_, 0.0, f_, sp_fn=sp_one, d_=d_, t_e=3.0, m0=2.0)
            vals.append(100.0 * (1.0 - float(np.mean(r["y"][-2000:]))) / 1.0)
        res3[nm] = vals

    trk = 100.0 * B / (B + KT * 0.25)      # 跟踪误差项（常数）
    rows = []
    for i, d_ in enumerate(d_list):
        dist_err = -100.0 * d_ / (B + KT * 0.25)
        rows.append(["%.3f" % d_,
                     "%.2f%%" % trk,
                     "%.2f%%" % dist_err,
                     "%.2f%%" % res3["纯 P  Kp=0.25"][i],
                     "%.2f%%" % res3["P+FF Kp=0.25"][i],
                     "%.2f%%" % res3["P+FF Kp=1.20"][i]])
    tab(rows, ["扰动 d", "跟踪误差项", "扰动误差项", "纯P实测",
               "P+FF实测", "P+FF实测(高Kp)"])

    print("""    ★★ **这张表要把误差「拆开」读，三句话就够：**

      ① 第 2 列（跟踪误差项 47.37%）是纯 P 顶不住的那部分。
         第 4 列（纯P实测）= 第 2 列 + 第 3 列，**逐行精确吻合**。

      ② 第 5 列（P+FF 实测）：d = 0 时**精确等于 0.00%** ——
         FF 把第 2 列整个消掉了。剩下的**正好只剩第 3 列**
         （−2.63%、−5.26%、−13.16%，和理论值逐位一致）。

      ③ 第 6 列（把 Kp 从 0.25 提到 1.20 再加 FF）：
         第 3 列的扰动误差从 −13.16% 缩到 −4.39% ——
         **它变小是因为 Kp 大了，不是因为 FF。**
         而 Kp 一大，实验 1 的相位裕度墙就在前面等着。

      ⇒ ★★ **结论（本节的判断标准）：**

        **前馈只能补偿「模型里写得出来的东西」。
          凡是写不进逆模型的（未知常值扰动、突风、电池电压漂移、
          机臂变形、载重偏心变化），FF 一点忙都帮不上 ——
          它的稳态误差一分不少地留在那里。**

        **判断标准：问自己「这个扰动我【事先】知道吗？
          能不能写成期望轨迹或模型的函数？」**
          能 → 用 FF，而且优先用 FF（没有相位代价）
          不能 → 只能用 I（有相位代价和积分饱和风险，但没有别的办法）

      ⇒ 这就是 FF 和 I 的分工，**必须分清楚，否则一定会用错**：
""")

    print("""        ┌────────────┬───────────────────────┬───────────────────────┐
        │            │ 补什么                 │ 靠什么信息             │
        ├────────────┼───────────────────────┼───────────────────────┤
        │ FF（前馈）  │ **已知的**期望运动/扰动 │ 期望轨迹 + 模型        │
        │ I（积分）   │ **未知的**常值扰动      │ 误差的历史累积         │
        └────────────┴───────────────────────┴───────────────────────┘

    ⇒ **能用 FF 的场合，就不要用 I。**
      因为 I 是「事后补偿」（先错再补），会带来相位滞后和积分饱和；
      FF 是「事前补偿」，没有相位代价。

    ⇒ 反过来，**FF 有模型误差时会补错方向**。
      本实验故意让 J_model = 0.7·J，FF 就少补了 30%。
      这部分残差仍然要靠反馈。**所以 FF 不能替代反馈，只能减轻它。**

    ★★ 战场上的判断标准（一句话）：
      **「这个扰动我【事先】知道吗？」**
        知道（悬停油门、期望角加速度、已知的负载偏心）→ 用 FF
        不知道（风、电池电压漂移、机臂变形）        → 只能用 I
""")

    if _HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(13, 4.4))
        tr = results["纯 P，低增益"][3]["t"]
        m = tr <= 1.2
        axes[0].plot(tr[m], sp_val(sp_sin, tr[m]), "k-", lw=2,
                     label=_t("期望 ω_sp", "setpoint"))
        for nm, st in (("纯 P，低增益", ":"), ("纯 P，高增益", "--"),
                       ("★ P + 完整 FF（低增益）", "-")):
            yy = results[nm][3]["y"]
            axes[0].plot(tr[m], yy[m], st, lw=1.6,
                         label=nm.replace("★ ", ""))
        axes[0].set_title(_t("跟踪 1.5 Hz 正弦", "tracking 1.5 Hz sine"))
        axes[0].set_xlabel(_t("时间 (s)", "time (s)"))
        axes[0].legend(fontsize=8)
        axes[0].grid(alpha=0.3)

        names = [c[0] for c in cases]
        xs = np.arange(len(names))
        axes[1].bar(xs - 0.2, [results[n][3]["err_rms"] for n in names],
                    0.4, label=_t("误差 RMS", "err RMS"))
        axes[1].bar(xs + 0.2, [results[n][3]["fb_rms"] for n in names],
                    0.4, label=_t("反馈输出 RMS（PID 的负担）", "fb RMS"))
        axes[1].set_xticks(xs)
        axes[1].set_xticklabels([_t("P低", "P low"), _t("P高", "P high"),
                                 _t("P+D", "P+D"), _t("P+FF", "P+FF")],
                                fontsize=9)
        axes[1].set_title(_t("误差 vs PID 的负担", "error vs control effort"))
        axes[1].legend(fontsize=8)
        axes[1].grid(alpha=0.3, axis="y")
        plt.tight_layout()
        plt.savefig(_fig_path("lesson25_exp3_feedforward.png"), dpi=110)
        plt.close(fig)
        print("    ✅ 图已存：lesson25_exp3_feedforward.png")
        print()


# ---------------------------------------------------------------------------
# 实验 4：从响应曲线反推 ζ/ωn
# ---------------------------------------------------------------------------

def exp4():
    head("实验 4  ★ 从响应曲线反推 ζ/ωn —— 把「手感」翻译成数学")

    print("""
  前三个实验都在「给定参数 → 预测响应」。
  实战里是反过来的：**手上只有一条日志曲线，要反推参数该往哪调。**

  本实验走一个完整的来回：

      ① 正向：给定 Kp/Kd → 解析算出 ζ/ωn → 生成阶跃响应曲线
      ② 反向：从曲线上量出【超调 Mp】和【峰值时间 tp】→ 反解 ζ/ωn
      ③ 对比：两者是否一致？噪声/延迟下还准不准？
      ④ 实战：从【振荡频率】判断是哪一环的问题

  两个反解公式（第 5 章的数学，这里直接用）：

      超调：  Mp = exp(−πζ/√(1−ζ²))     ⇒   ζ = |ln Mp| / √(π² + ln²Mp)
      峰时：  tp = π / (ωn·√(1−ζ²))     ⇒   ωn = π / (tp·√(1−ζ²))
""")

    # ---------------- ① / ② 正反来回 ----------------
    sub("① ② 正向算 → 生成曲线 → 反向量 → 对比")

    print("    同样用实验 1 的被控对象（J=%.4f, b=%.3f, τm=%.3f, Td=%.0f ms）"
          % (J, B, TAU_M, TD * 1000))
    print()

    rows = []
    for kp, kd in ((0.15, 0.0), (0.15, 0.0030), (0.50, 0.0),
                   (0.50, 0.0030), (0.80, 0.0)):
        wn_a = omega_n(kp)
        z_a = zeta_of(kp, kd)
        t, y, _, _ = simulate(kp, kd, t_end=1.0)
        met = step_metrics(t, y)

        # 反向：从曲线的 Mp 反解 ζ，再从 tp 反解 ωn
        if met["Mp"] > 0.05:
            z_b = zeta_from_overshoot(met["Mp"])
            wn_b = math.pi / (met["tp"] * math.sqrt(max(1e-9, 1 - z_b ** 2)))
            e_b = "%.0f%%" % (100 * (wn_b - wn_a) / wn_a)
            e_z = "%.0f%%" % (100 * (z_b - z_a) / z_a)
        else:
            z_b, wn_b, e_b, e_z = 1.0, float("nan"), "—", "—"

        rows.append(["%.2f/%.4f" % (kp, kd),
                     "%.3f" % z_a, "%.1f" % wn_a, "%.1f" % met["Mp"],
                     "%.3f" % z_b, e_z,
                     "%.1f" % wn_b, e_b])
    tab(rows, ["Kp/Kd", "解析ζ", "解析ωn", "量得超调%", "反解ζ", "ζ误差",
               "反解ωn", "ωn误差"])

    print("""    ★★ 读表的方法：**分别看 ζ 和 ωn 的反解精度 —— 它们差别极大。**

      · **反解 ωn 相当准**：五行的误差是 −2%、−5%、+3%、−7%，
        全部在 ±8% 以内（Kd=0.0030 那行无超调，算不出来）。
        ⇒ 「量峰值时间 ⇒ 算带宽」这条路是**能用的**。

      · **反解 ζ 系统性偏小，而且偏得很多**：−11%、−36%、−26%、−54%。
        原因是那 8 ms 纯延迟：它让实测超调比二阶模型预测的大，
        于是按公式反解出来的 ζ 必然偏小。

        ⇒ 「量超调 ⇒ 算阻尼」**只能当【下限】用**：
          反解出 ζ=0.37，真值一定比它大，不会更小。
          这条结论很有用 —— 它让你永远**不会把飞机调得比你以为的更危险**。

      · **反过来，ζ 接近 1 时也量不出来**（Kd=0.0030 那行打「—」）：
        Mp = 0 时公式直接退化，而 Mp 很小时 dMp/dζ → 0，
        **ζ > 0.9 就是反解 ζ 的第二个死区。**

    ★★ 工程结论（本节的判断标准）：

        **ωn 可以定量反解（±10% 以内），ζ 只能定性反解。**
        具体用法：
          · 想看带宽 → 量峰值时间或振荡周期，反解 ωn，**放心用**
          · 想看阻尼 → 量超调，反解 ζ，**只当趋势和下限**
          · ζ > 0.9 或 < 0.2 时两个量都不可靠，改用频域方法（看 FFT 主频）

    ⇒ 正确用法：**用它做「相对比较」而不是「绝对计算」。**
      同一个机型，A 次调参反解出 ζ=0.35，B 次反解出 ζ=0.62，
      这个【差值】是可信的，绝对数值不要当真。
""")

    # ---------------- ③ 噪声下还准不准 ----------------
    sub("③ 带噪声的曲线还能反解吗")

    print("""    实战日志里的曲线都是带噪声的。噪声会：
      · 让量出来的「峰值」偏高（噪声尖峰被当成峰值）→ 反解 ζ 偏小
      · 让峰值时间提前

    下面把一条【干净】的响应加上不同强度的读数量噪声
    （不是加在控制回路里 —— 那是另一回事，这里模拟的是
      「你从日志里读到的曲线本身是脏的」），看反解结果怎么飘。""")

    t, y_clean, _, _ = simulate(0.50, 0.0, t_end=1.0)
    met0 = step_metrics(t, y_clean)
    z_true = zeta_of(0.50, 0.0)
    wn_true = omega_n(0.50)

    def invert(y_meas):
        """从一条（可能有噪声的）曲线反解 (ζ, ωn)。"""
        met = step_metrics(t, y_meas)
        if met["Mp"] <= 0.05:
            return None, None
        zz = zeta_from_overshoot(met["Mp"])
        return zz, math.pi / (met["tp"] * math.sqrt(max(1e-9, 1.0 - zz ** 2)))

    def smooth(y, k):
        """长度 k 的滑动平均（k 为奇数）。"""
        kern = np.ones(k) / k
        return np.convolve(y, kern, mode="same")

    rng = np.random.default_rng(2026)
    print()
    rows = []
    for ns in (0.0, 0.005, 0.02, 0.05):
        n = len(y_clean)
        zs, ws, zs_f, ws_f = [], [], [], []
        for rep in range(7):
            y_meas = y_clean + (rng.normal(0.0, ns, n) if ns > 0 else 0.0)
            a, b = invert(y_meas)
            if a is not None:
                zs.append(a)
                ws.append(b)
            a, b = invert(smooth(y_meas, 101))     # 2 ms 滑动平均
            if a is not None:
                zs_f.append(a)
                ws_f.append(b)
        rows.append(["%.3f" % ns, "%.3f" % z_true,
                     "%.3f" % float(np.mean(zs)) if zs else "—",
                     "%.1f" % float(np.mean(ws)) if ws else "—",
                     "%.3f" % float(np.mean(zs_f)) if zs_f else "—",
                     "%.1f" % float(np.mean(ws_f)) if ws_f else "—"])
    tab(rows, ["量测噪声σ", "真值ζ", "反解ζ(原始)", "反解ωn(原始)",
               "反解ζ(平滑)", "反解ωn(平滑)"])

    print("""    （真值：ζ = %.3f，ωn = %.1f rad/s；每个噪声水平跑 7 次取平均）
""" % (z_true, wn_true))

    print("""    ★★ 读表，两条铁律：

      ① **噪声让反解出的 ζ 系统性偏小**（超调被噪声尖峰抬高）。
         本表噪声从小到大，「反解ζ(原始)」一路从 0.371 掉到 0.209；
         「反解ωn(原始)」也开始失准（66.2 → 60.9）。
         注意 σ = 0.05 相对于曲线稳态值 %.2f 已经是 10%% 的读数量噪声 ——
         真实日志完全会脏到这个程度。
         （σ 是【读数】噪声；判「脏不脏」要和稳态值比，而不是和峰值比。）

      ② **先平滑再量，「反解ζ」立刻稳定在 0.36 附近不再随噪声漂。**
         本表用的是 101 点滑动平均（在 50 kHz 仿真采样下只有 2 ms，
         远快于任何一个真实振荡周期，所以不会把真振荡也抹掉）。
         ★ 注意它稳定到的是 0.36 而不是真值 0.575 ——
           **那 0.21 的差不是噪声造成的，是 8 ms 纯延迟造成的**
           （见 ①② 节）。平滑只能消掉噪声，消不掉延迟。

      ⇒ ★★ 实战判断标准：

        **量 Mp / tp 之前必须先平滑，而且平滑窗口要「远小于振荡周期」。**
        经验值：**平滑窗口 ≤ 振荡周期的 1/10。**
        如果你不确定振荡周期，就先看曲线的过零点间隔估一个。

      ⇒ 另一个更稳的量法：不看峰值，看**过零点的间隔**（振荡周期）。
        T = 2π/(ωn√(1−ζ²))，它对噪声鲁棒得多 ——
        因为它是【多个周期平均】出来的，而峰值只是【一个采样点】。
        本脚本下面的 ④ 就用这个量法。
""" % float(np.mean(y_clean[-200:])))

    # ---------------- ④ 振荡频率 → 哪一环 ----------------
    sub("④ ★★ 实战重点：从振荡【频率】判断是哪一环")

    print("""    先确立一个**可以自己验证**的事实：

        **残余振荡的频率 ≈ 该环的增益穿越频率 ω_c。**

    为什么？因为振荡就是「相位到了 −180°、增益还大于 1」这件事的
    时域表现。环路在 ω_c 附近最「临界」（增益刚好 = 1），
    所以一旦裕度不够，最先振起来的就是 ω_c 附近的成分。

    下面用「过零点间隔」量出残余振荡频率，和解析算出的 ω_c 对照：""")
    print()

    def osc_freq(t, sig, t0=0.05):
        """用【过零点间隔】量残余振荡频率（Hz）。量不到就返回 None。"""
        m = t >= t0
        tt, yy = t[m], sig[m]
        yss = float(np.mean(yy[-max(10, len(yy) // 20):]))
        d = yy - yss
        if np.max(np.abs(d)) < 1e-9:
            return None
        idx = np.nonzero(np.diff(np.sign(d)))[0]
        tz = []
        for k in idx:
            y0, y1 = d[k], d[k + 1]
            if y1 == y0:
                continue
            tz.append(tt[k] + (-y0 / (y1 - y0)) * (tt[k + 1] - tt[k]))
        if len(tz) < 4:
            return None
        return 1.0 / (2.0 * float(np.mean(np.diff(tz))))

    rows = []
    for kp in (0.50, 0.80, 1.20, 1.50, 1.66):
        wc = crossover(kp, 0.0)
        pm = phase_margin(kp, 0.0)
        t, y, _, _ = simulate(kp, 0.0, t_end=1.5)
        fo = osc_freq(t, y)
        rows.append(["%.2f" % kp,
                     "%.0f°" % pm if pm is not None else "—",
                     "%.1f" % (wc / 2 / math.pi) if wc else "—",
                     "%.1f" % fo if fo else "不振荡"])
    tab(rows, ["Kp", "相位裕度", "解析穿越频率(Hz)", "实测残余振荡(Hz)"])

    print("""    ★★ **看最后两列：裕度越小，吻合得越好。**

      · Kp = 1.50 / 1.66（PM = 5° / 0°，也就是**真的在振**）：
        预测 14.5 / 15.4 Hz，实测 14.9 / 15.4 Hz —— **几乎完全一致。**
      · Kp = 0.50（PM = 67°，曲线基本不振荡）：
        预测 6.6 Hz，实测 10.0 Hz —— **差得多。**

      ⇒ ★★ 这不是公式错了，而是**判据的适用范围**：

        **「振荡频率 ≈ 穿越频率」只在【真的在振荡】时成立。**
        当相位裕度很大（≳ 60°）时，响应里没有持续的振荡，
        你量到的「周期」只是超调后的一两个起伏，甚至只是噪声 ——
        那个数字没有意义。

      ⇒ 这反而是个好消息：**你需要用这条判据的时候，
        正是它在起作用的时候**（振起来了、PM 小）。
        反过来，如果曲线不振荡，你也不需要判断是哪个环的问题。

    ★★ 所以「量振荡频率」这条判据的完整用法是：

        ① 先确认**确实存在持续振荡**（数一数振了几个周期：
           振 1 个就停 → PM ≳ 60°，不用管；
           振 3~4 个 → PM ≈ 30~40°；不停 → PM ≤ 0）
        ② 再量它的频率
        ③ 拿这个频率去和「内环带宽」比 —— 见下面的级联判据
""")

    print("""    ★★ 现在把级联环的铁律补上（这是本课最实用的一条判据）：

        外环带宽  ≈  内环带宽 / 5 ~ / 10

      为什么必须是这个比例？因为外环把内环当成「理想执行器」用。
      只有外环的信号变化比内环**慢得多**，内环才来得及跟上，
      「理想执行器」这个近似才成立。差 5 倍，内环在这个频率上
      的相位滞后大约是 −20°~−30°，还吃得消；
      差到 2~3 倍，滞后就吃掉全部相位裕度 —— **级联假设失效。**

    ★★ **拿 PX4 的真默认值验一遍**（这才是这一节的高潮）：

        MC_ROLL_P = 6.5      ← mc_att_control_params.c:53
        MC_ROLLRATE_P = 0.15 ← mc_rate_control_params.c:51
        MC_ROLLRATE_D = 0.003 ← mc_rate_control_params.c:88
""")

    kap = 6.5
    wn_in = omega_n(PX4_KP)
    z_in = zeta_of(PX4_KP, PX4_KD)
    print("        外环（姿态环）：输出是「误差 1 rad 给多少 rad/s 的角速率指令」，")
    print("                        被控对象近似是 1/s（姿态是角速率的积分）")
    print("          ⇒ 开环 L(s) = %.1f/s  ⇒  穿越频率 = %.1f rad/s = **%.2f Hz**"
          % (kap, kap, kap / 2 / math.pi))
    print()
    print("        内环（角速率环）：把默认值代进本课的模型")
    print("          ⇒ ωn = %.1f rad/s = **%.1f Hz**（ζ = %.3f）"
          % (wn_in, wn_in / 2 / math.pi, z_in))
    print()
    print("        两个带宽之比 = %.1f Hz / %.2f Hz = **%.1f 倍**"
          % (wn_in / 2 / math.pi, kap / 2 / math.pi,
             wn_in / kap))
    print()
    print("""    ⇒ ★★ **7.7 倍，正好落在 5~10 倍的推荐区间里。**

      这不是巧合 —— 它说明本课的简化模型（J/b/τm）和 PX4 的真实默认值
      **是同一个量级的东西**，也说明 PX4 的默认参数确实是按
      「内环 8 Hz、外环 1 Hz」这个级联原则配的。

      这就是「外环振荡在 1 Hz 上下、内环振荡在 8 Hz 上下」这句话的出处。
      它来自真实的参数，不是经验口诀。

    ★★ **判断标准（照做，不要猜）：**

      ┌──────────────┬────────────────────┬──────────────────────────┐
      │ 日志里看到的  │ 是哪一环            │ 该动哪个参数              │
      ├──────────────┼────────────────────┼──────────────────────────┤
      │ 0.5~2 Hz 的   │ 外环                │ MC_ROLL_P / ATC_ANG_RLL_P │
      │ 周期性摆动    │ （姿态/位置环）      │ 位置环的 P（先别动内环）    │
      ├──────────────┼────────────────────┼──────────────────────────┤
      │ 5~15 Hz 的    │ 内环                │ MC_ROLLRATE_P / _D        │
      │ 高频抖动      │ （角速率环）         │ 或 IMU_DGYRO_CUTOFF       │
      ├──────────────┼────────────────────┼──────────────────────────┤
      │ > 50 Hz 的    │ 不是控制环，是       │ 查桨叶动平衡、机架共振、    │
      │ 尖叫/毛刺     │ 振动或电气问题       │ 陀螺滤波（先硬后软）        │
      └──────────────┴────────────────────┴──────────────────────────┘

      ★ 三步操作法：
        ① 先用 FFT（或数过零点）量出**振荡频率**
        ② 拿它和「内环带宽」比：远低于内环 ⇒ 外环；接近内环 ⇒ 内环
        ③ **动手前先确认这一条**，否则你会一直在错的那一环上白费力气
"""
          )


    if _HAS_MPL:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.6))

        for kp, st in ((0.50, "-"), (1.20, "--"), (1.50, "-."), (1.66, ":")):
            t, y, _, _ = simulate(kp, 0.0, t_end=0.6)
            m = t >= 0.05
            ax1.plot(t[m], y[m], st, lw=1.6, label="Kp=%.2f" % kp)
        ax1.axhline(1.0, color="k", lw=0.6)
        ax1.set_xlim(0.05, 0.45)
        ax1.set_ylim(-0.2, 2.4)
        ax1.set_xlabel(_t("时间 (s)", "time (s)"))
        ax1.set_ylabel(_t("角速率 ω (rad/s)", "rate (rad/s)"))
        ax1.set_title(_t("残余振荡：数周期 + 量过零点",
                         "residual oscillation: count & measure"))
        ax1.legend(fontsize=8)
        ax1.grid(alpha=0.3)

        pms, pred, meas = [], [], []
        for kp in (0.50, 0.80, 1.00, 1.20, 1.50, 1.66):
            pm = phase_margin(kp, 0.0)
            wc = crossover(kp, 0.0)
            t, y, _, _ = simulate(kp, 0.0, t_end=1.5)
            fo = osc_freq(t, y)
            if pm is None or wc is None or fo is None:
                continue
            pms.append(pm)
            pred.append(wc / 2 / math.pi)
            meas.append(fo)
        lim = max(max(pred), max(meas)) * 1.15 if pred else 20.0
        ax2.plot([0, lim], [0, lim], "k--", lw=0.8)
        sc = ax2.scatter(pred, meas, c=pms, cmap="RdYlGn", s=70,
                         edgecolors="k", zorder=3)
        for p, q, pm in zip(pred, meas, pms):
            ax2.annotate("PM=%.0f°" % pm, (p, q), fontsize=7,
                         xytext=(5, -9), textcoords="offset points")
        ax2.set_xlim(0, lim)
        ax2.set_ylim(0, lim)
        ax2.set_xlabel(_t("解析穿越频率 (Hz)", "predicted crossover (Hz)"))
        ax2.set_ylabel(_t("实测残余振荡频率 (Hz)", "measured osc. freq (Hz)"))
        ax2.set_title(_t("裕度越小，这条判据越准", "the smaller the PM, the better"))
        plt.colorbar(sc, ax=ax2, label=_t("相位裕度 (°)", "PM (deg)"))
        ax2.grid(alpha=0.3)

        fig.tight_layout()
        fig.savefig(_fig_path("lesson25_exp4_zeta_omega.png"), dpi=110)
        plt.close(fig)
        print("    ✅ 图已存：lesson25_exp4_zeta_omega.png（在脚本所在目录）")
        print()

    # ---------------- ⑤ 把「手感」翻译成数学 ----------------
    sub("⑤ 把「手感」翻译成数学（调参时的自问自答）")

    tab([
        ["推杆后反应慢、跟不上", "ωn 太小", "P 太小", "加大 Kp（看相位裕度）"],
        ["跟得快但停不住，来回晃", "ζ 太小", "P 太大 / D 不够", "加 Kd 或退 Kp"],
        ["稳态总差一点", "低频增益不足", "没有 I / I 太小", "加 Ki + 抗饱和"],
        ["电机高频啸叫、发热", "D 在放大噪声", "Kd 太大 / 滤波太弱", "降 Kd 或降 f_d"],
        ["一加力就高频抖", "相位裕度不足", "P、D、滤波的综合", "退到 60~70% + 查延迟"],
        ["慢周期漂移", "外环振荡", "外环 P 太大 / I 太大", "动外环，别动内环"],
    ], ["手感（现象）", "数学量", "最可能的参数", "怎么动"])

    print("""    ★ 这张表就是本课的浓缩：
      **每一个「手感」都能翻译成一个 ζ/ωn/低频增益的问题，**
      **每一个问题都对应一个明确的参数 —— 不需要试。**
""")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("P 从 0 加起：振荡是怎么出现的", exp1),
    "2": ("★ D 的作用与代价：压振荡 vs 放大噪声", exp2),
    "3": ("★★ 前馈：PID 的负担小了多少", exp3),
    "4": ("★ 从响应曲线反推 ζ/ωn", exp4),
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
