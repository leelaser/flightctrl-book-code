#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 9.8 周 · 06 吊运与消摆动力学 —— 配套实验

    实验 1  ★★  吊挂系统的精确动力学 + 两个极限频率
    实验 2  ★★★ 模态靠拢：摆的频率取决于位置环带宽
    实验 3  ★★★ 输入整形（ZV）：为什么它能消摆
    实验 4  ★★  鲁棒整形（ZVD）：缆长估计有误差时
    实验 5  ★★  缆绳的单边约束：什么时候会松弛
    实验 6  ★★  端到端闭环：整形 vs 不整形

用法：
    python lesson09e_slung_load.py 1
    python lesson09e_slung_load.py 2
    ...
    python lesson09e_slung_load.py          # 全部

只需 numpy（画图需要 matplotlib）。

⚠️ 本课和 01~05 课有一个重要区别：
   **ArduPilot 和 PX4 都没有吊运/消摆支持**（全树搜 `slung`/`crane` 零命中），
   本地资料里也没有相关文献。所以本课【没有源码可以核对】，
   全部靠推导 + 数值仿真。文末 §8.9 会说明这件事，并给出
   「消摆补偿本该插在哪里」的那个源码锚点。

符号约定：
    x_d      无人机水平位置 (m)
    theta    缆绳与竖直方向的夹角 (rad)
    L        缆长 (m)
    m_d      无人机质量 (kg)
    m_p      载荷质量 (kg)
    mu       m_p / m_d
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

G = 9.80665


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
# 公共：吊挂系统的精确动力学
#
# 广义坐标 (x_d, theta)，拉格朗日推导（见课文 §8.2），**不做小角度近似**：
#
#   (I)  (m_d + m_p) x_d'' + m_p L (theta'' cos - theta'^2 sin) = F_x
#   (II) L theta'' = - x_d'' cos(theta) - g sin(theta)
#
# 由 (II) 解出 theta''，代回 (I) 解出 x_d''：
#   x_d'' = [F_x + m_p g sin.cos + m_p L theta'^2 sin] / (m_d + m_p sin^2)
# ===========================================================================

def deriv(s, Fx, m_d, m_p, L):
    xd, vd, th, om = s
    den = m_d + m_p * math.sin(th) ** 2
    add = (Fx + m_p * G * math.sin(th) * math.cos(th)
           + m_p * L * om * om * math.sin(th)) / den
    thdd = (-add * math.cos(th) - G * math.sin(th)) / L
    return np.array([vd, add, om, thdd])


def sim_rk4(s0, t_end, Fx_func, m_d, m_p, L, dt=1e-4):
    """RK4 积分。Fx_func(t, s) 或常数。"""
    n = int(t_end / dt)
    s = np.array(s0, float)
    out = np.zeros((n, 5))
    for k in range(n):
        t = k * dt
        u = Fx_func(t, s) if callable(Fx_func) else Fx_func
        out[k] = [s[0], s[1], s[2], s[3], u]
        k1 = deriv(s, u, m_d, m_p, L)
        k2 = deriv(s + dt / 2 * k1, u, m_d, m_p, L)
        k3 = deriv(s + dt / 2 * k2, u, m_d, m_p, L)
        k4 = deriv(s + dt * k3, u, m_d, m_p, L)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return np.arange(n) * dt, out


def sim_prescribed(a_func, t_end, L, th0=0.0, dt=1e-4):
    """支点加速度被【直接给定】—— 用来单独研究整形器，剥掉位置环的影响。

        L theta'' + g sin(theta) = - a(t) cos(theta)
    """
    n = int(t_end / dt)
    th, om = th0, 0.0
    hist = np.zeros((n, 2))
    for k in range(n):
        a = a_func(k * dt)
        hist[k] = [th, a]
        om += (-a * math.cos(th) - G * math.sin(th)) / L * dt
        th += om * dt
    return np.arange(n) * dt, hist


def rms_tail(t, th, t0):
    m = t >= t0
    return float(np.sqrt(np.mean(th[m] ** 2)))


def measure_freq(t, th, t0=2.0):
    """峰值计数法测频率（小阻尼时够用）。"""
    m = t >= t0
    tt, hh = t[m], th[m]
    pk = [k for k in range(1, len(hh) - 1)
          if hh[k] > hh[k - 1] and hh[k] >= hh[k + 1]]
    if len(pk) < 3:
        return None
    return 2 * math.pi / ((pk[-1] - pk[0]) * (tt[1] - tt[0]) / (len(pk) - 1))


# ===========================================================================
# 实验 1  ★★ 精确动力学 + 两个极限频率
# ===========================================================================

def exp1():
    head("实验 1  ★★ 吊挂系统的精确动力学 + 两个极限频率")

    sub("(a) 推导结果（课文 §8.2 逐步推）")

    print("""
  广义坐标取 (x_d, theta)。拉格朗日量
      T = 1/2 m_d xd'^2 + 1/2 m_p [ (xd' + L th' cos)^2 + (L th' sin)^2 ]
      V = m_p g L (1 - cos theta)

  得到两个方程（★ 全程不做小角度近似）：

      (I)  (m_d+m_p) xd'' + m_p L (th'' cos - th'^2 sin) = F_x
      (II) L th'' = - xd'' cos(theta) - g sin(theta)

  ★★★ 注意 (II) 是一个**精确**的摆方程：右端第一项说明
      **【驱动摆的是支点的加速度 xd''，不是速度、也不是位置】。**
""")

    sub("(b) 线性化：一个方程、两个频率")

    print("""  小角度下 cos->1, sin->theta，消去 xd''：

      (I)  =>  m_d xd'' = F_x + m_p g theta
      (II) =>  L th'' + g theta = - xd''

  ★ 把 (I) 代进 (II)：

      L th'' + g theta (1 + m_p/m_d) = - F_x / m_d
""")
    print("      ⇒ 以【力】为输入：  weff = sqrt( g(1 + mu) / L )   mu = m_p/m_d")
    print("      ⇒ 以【支点加速度】为输入：wn = sqrt( g / L )      ★ 与质量无关")
    print()

    sub("(c) 数值验证：两个极限")

    L = 1.0
    m_d = 1.0
    wn = math.sqrt(G / L)
    print("  L=%.2f m  ->  wn = sqrt(g/L) = %.4f rad/s" % (L, wn))
    print()
    print("      mu=m_p/m_d   支点【钉死】实测   理论 wn    支点【自由】实测   理论 weff")
    print("      " + "-" * 76)
    for mu in [0.0, 0.5, 1.0, 2.0]:
        mp = mu if mu > 0 else 1e-6
        # 钉死：m_d 取极大，推力推不动它
        t1, o1 = sim_rk4([0, 0, 0.2, 0], 30.0, 0.0, 1e6, mp, L)
        f1 = measure_freq(t1, o1[:, 2])
        # 自由：不受控
        t2, o2 = sim_rk4([0, 0, 0.2, 0], 30.0, 0.0, m_d, mp, L)
        f2 = measure_freq(t2, o2[:, 2])
        we = math.sqrt(G * (1 + mu) / L)
        print("      %-12.2f %16.4f %12.4f %16.4f %12.4f"
              % (mu, f1, wn, f2, we))
    print("""
  ★★ 两条结论：
      · 支点【钉死】时频率恒为 wn = %.4f，与载荷质量【无关】
      · 支点【自由】时频率是 weff，随 mu 增大而【变大】
        => 载荷越重，摆得【越快】 —— 有点反直觉，但两式都这么写

  ★ 实测比理论低约 0.25%%：那是【大摆角】修正（theta0=0.2 rad）：
        wn_真实 / wn_小角 ≈ 1 - theta0^2/16 = 1 - 0.0025  ->  -0.25%%  ✓
    数字严丝合缝对上了 —— 这反过来验证了仿真是对的。
""" % wn)

    sub("(d) 为什么支点自由时更快？")

    print("""  物理图像：无人机是【漂浮的支点】。

    · 载荷往右摆 -> 缆绳把无人机往右拽 -> 无人机右移 -> 相对位移减小
    · 也就是说，无人机的"让步"相当于【增强了恢复力】
    · 让步得越多（m_p/m_d 越大），恢复力越强 -> 频率越高

  ★ 极限 mu -> 无穷：weff -> 无穷（线性模型失效，需要考虑无人机被甩飞）


  ★★ 这两条极限的工程意义（下一节展开）：

       吊车（支点固定）  ->  wn = sqrt(g/L)
       多旋翼（支点自由）->  weff = sqrt(g(1+mu)/L)

     **而真实的多旋翼吊运，支点【既不是固定的也不是自由的】——
       它由位置环控制。所以真实频率落在这两者之间。**
""")


# ===========================================================================
# 实验 2  ★★★ 模态靠拢
# ===========================================================================

def exp2():
    head("实验 2  ★★★ 模态靠拢：摆的频率取决于位置环带宽")

    sub("(a) 无人机加一个 PD 位置环")

    print("""  给无人机加位置环（这正是 ArduPilot / PX4 位置环在做的事）：

        F_x = - kp * x_d - kd * x_d'

  取 kd = 2*sqrt(kp*m_d)（对无人机单独而言临界阻尼）。
  然后做【特征值分析】：把 4 状态线性化，看系统有几个模态、各在什么频率。
""")

    m_d, m_p, L = 1.0, 1.0, 1.0
    wn = math.sqrt(G / L)
    we = math.sqrt(G * (1 + m_p / m_d) / L)
    print("  md=%.1f, mp=%.1f, L=%.1f" % (m_d, m_p, L))
    print("      wn = %.4f（支点钉死）    weff = %.4f（支点自由）" % (wn, we))
    print()
    print("      %-10s %-12s %-13s %-13s" % ("kp", "环带宽~", "低模态", "高模态（摆）"))
    print("      " + "-" * 56)
    for kp in [0.0, 0.5, 2.0, 8.0, 13.8, 35.6, 91.5, 605.0, 1557.0]:
        kd = 2 * math.sqrt(max(kp, 1e-9) * m_d) if kp > 0 else 0.0
        A = np.array([
            [0, 1, 0, 0],
            [-kp / m_d, -kd / m_d, m_p * G / m_d, 0],
            [0, 0, 0, 1],
            [kp / (m_d * L), kd / (m_d * L), -(m_p * G / m_d + G) / L, 0]])
        ev = np.linalg.eigvals(A)
        om = sorted(abs(e.imag) for e in ev if abs(e.imag) > 1e-6)
        uniq = []
        for w in om:
            if not uniq or abs(w - uniq[-1]) > 1e-6:
                uniq.append(w)
        if kp == 0.0:
            lo, hi = "—", "%.4f" % uniq[0] if uniq else "—"
        elif len(uniq) >= 2:
            lo, hi = "%.4f" % uniq[0], "%.4f" % uniq[-1]
        else:
            lo, hi = "（合并）%.4f" % uniq[0], "（合并）%.4f" % uniq[0]
        print("      %-10.2f %-12.3f %-13s %-13s"
              % (kp, math.sqrt(kp / m_d), lo, hi))

    sub("(b) ★★★ 三条读数")

    print("""  ① **kp = 0（位置环关掉）**：只有一个模态 = weff = 4.4287
     —— 这就是【支点自由】的极限。

  ② **kp -> 无穷（位置环极硬）**：摆模态 -> wn = 3.1316
     —— 这就是【支点钉死】的极限。

  ③ ★★★ 中间不是单调过渡，而是【模态靠拢】(mode veering)：
     低模态往上升、高模态往下降，在 kp ≈ 14 附近【合并】，
     合并处的频率 2.76 甚至【低于 wn】。

  ★ 为什么会靠拢：两个子系统的频率接近时（摆频 wn 与位置环带宽 sqrt(kp/m_d)），
    它们会【互相推开】而不是交叉 —— 这是耦合振子的普遍现象。
""")

    sub("(c) ★★★ 但还有一个更狠的：位置环【阻尼】不足会冒出寄生模态")

    print("""  ★ 上面的扫描里 kd 一直取 2*sqrt(kp*m_d)（对无人机单独而言【临界阻尼】）。
    把这个约束松开，固定 kp = 100，只看 kd：
""")
    kp_fix = 100.0
    print("      %-10s %-12s %-12s %-12s %s"
          % ("kd", "drone环 ζ", "低模态", "高模态", "高模态/wn"))
    print("      " + "-" * 62)
    for kd in [1.0, 5.0, 10.0, 14.142, 20.0, 30.0, 50.0, 100.0]:
        A = np.array([
            [0, 1, 0, 0],
            [-kp_fix / m_d, -kd / m_d, m_p * G / m_d, 0],
            [0, 0, 0, 1],
            [kp_fix / (m_d * L), kd / (m_d * L),
             -(m_p * G / m_d + G) / L, 0]])
        ev = np.linalg.eigvals(A)
        om = sorted(abs(e.imag) for e in ev if abs(e.imag) > 1e-9)
        lo, hi = om[0], om[-1]
        crit = 2 * math.sqrt(kp_fix * m_d)
        mark = "   <- 临界阻尼" if abs(kd - crit) < 1e-6 else ""
        print("      %-10.2f %-12.3f %-12.4f %-12.4f %.2fx%s"
              % (kd, kd / (2 * math.sqrt(kp_fix * 2)), lo, hi, hi / wn, mark))
    print("""
  ★★★ 两条完全不同的行为，分界线就是【临界阻尼 kd = 2*sqrt(kp*m_d)】：

      kd < 临界：两个模态【频率不同】，且高模态可以【远高于】weff
                 （kd=1 时高模态 10.51，是 wn 的 3.36 倍！）
      kd >= 临界：两个模态【频率相同】，只差阻尼

  ==> **位置环的【阻尼】比【带宽】对摆动频率的影响更大。**

  ⚠️ 这是个陷阱：一段"看起来很正常"的位置环（kp=100, kd=1，等效 ζ=0.035）
     会让摆动的主频跑到 10.5 rad/s —— 而你按 wn=3.13 设计整形器，
     完全对不上，整形器【一点用都没有】。

  ★★ 所以完整的结论是：

      wn <= w_实际 <= weff      【仅在位置环阻尼足够时成立】
      阻尼不足 -> 出现远高于 weff 的寄生模态
""")

    sub("(c) ★★★ 对消摆的直接后果")

    print("""  摆的频率【不是常数】，它随位置环带宽从 weff 连续变到 wn，
  中间还可能被模态靠拢拉到更低。

  => **消摆整形器该按哪个频率设计？**

      · 位置环很硬（带宽 >> 摆频）  ->  用 wn = sqrt(g/L)
      · 位置环很软                  ->  用 weff
      · 飞控的位置环带宽通常在 1~3 rad/s，**与摆频同量级**
        => 正好落在最麻烦的中间区域

  ★★★ 这就是「吊运难调」的根本原因：
     **被控对象的自然频率，本身取决于你自己的控制器参数。**
     这在 01~05 课里从来没有出现过 —— 那些课里的 J、B、m 都是飞机的属性，
     和你怎么调参无关。

  ★ 也是为什么整形器必须【鲁棒】（实验 4）：
     你不能精确知道该用哪个频率。
""")


# ===========================================================================
# 实验 3  ★★★ 输入整形 ZV
# ===========================================================================

def exp3():
    head("实验 3  ★★★ 输入整形（ZV）：为什么它能消摆")

    sub("(a) 核心思想：把阶跃拆成两半")

    print("""  ★★★ 关键：摆的方程是**线性的**（小角度下）

        L th'' + g th = - a(t)

  => **叠加原理成立**。一个加速度脉冲 a(t)=A*delta(t) 产生的响应是

        th(t) = -(A/L) * sin(wn t) / wn      （t > 0，幅度 A）

     ★ 也就是一个【纯正弦】，从 t=0 开始，幅度不衰减（无阻尼时）。

  ★★ 于是：如果再在【半个周期后】加第二个脉冲，它产生的正弦
     与第一个**正好反相** -> 两个正弦相加 = 0 -> **残余摆动为零！**

        t2 = T/2 = pi / wn

  ★ 而为了让总冲量不变（总位移不变），两个脉冲各取一半幅度：

        A1 = A2 = A/2      （无阻尼时 K = 1）
""")

    sub("(b) 数值验证")

    L = 1.0
    wn = math.sqrt(G / L)
    Tp = 2 * math.pi / wn
    print("  L=%.2f m  ->  wn=%.4f rad/s, 周期 T=%.4f s, 半周期 T/2=%.4f s"
          % (L, wn, Tp, Tp / 2))
    print()
    A, duty = 2.0, 0.5

    def naive(t):
        return A if t < duty else 0.0

    def shaped(t, dt2=Tp / 2):
        s = 0.0
        if t < duty:
            s += A / 2
        if dt2 <= t < dt2 + duty:
            s += A / 2
        return s
    for name, fn in [("未整形", naive), ("ZV 整形", shaped)]:
        t, h = sim_prescribed(fn, 12.0, L)
        r = rms_tail(t, h[:, 0], 4.0)
        print("      %-8s  残余摆角 RMS = %.4e rad（%.2f 度）, 最大摆角 %.4f rad"
              % (name, r, math.degrees(r), np.abs(h[:, 0]).max()))

    sub("(c) 扫描：间隔取多少最好")

    print("      间隔(s)   间隔/(T/2)     残余 RMS")
    print("      " + "-" * 44)
    for frac in [0.0, 0.25, 0.5, 0.75, 0.9, 1.0, 1.1, 1.25, 1.5]:
        dt2 = frac * Tp / 2
        t, h = sim_prescribed(lambda tt: shaped(tt, dt2), 12.0, L)
        r = rms_tail(t, h[:, 0], 4.0)
        mark = "   <-- 最优（=T/2）" if abs(frac - 1.0) < 1e-9 else ""
        print("      %8.4f  %10.2f   %.4e%s" % (dt2, frac, r, mark))
    print("""
  ★★ 最小值精确落在间隔 = T/2 处（不是 0.5*T/2=0.25，也不是 0.75）——
     这就是理论说的「半个周期」。
""")

    sub("(d) ★★ 但整形是有代价的")

    print("""  ★ 看 ZV 那一行的【最大摆角】：0.1438 rad，
    比未整形的 0.2875 rad 【小了一半】—— 变好了。

  ⚠️ 但代价在【时间】上：两段加速度之间要等 T/2，
     所以从一个位置搬到另一个位置的总时间【变长了】。

  ★ 一般规律：**整形器抑制摆动的代价是"变慢"**。
     摆的周期越长（缆越长），等待越久。
     => 长缆吊运注定是【慢工】。

  ★ 还有第二个代价：整形后的指令包含【阶跃】，
     而真实无人机的加速度是有限带宽的（姿态环跟不上突变）。
     => 实际用的时候还要对整形后的指令做【平滑】，
        平滑又会重新引入一点摆动。这是个工程折中。
""")


# ===========================================================================
# 实验 4  ★★ 鲁棒整形 ZVD
# ===========================================================================

def exp4():
    head("实验 4  ★★ 鲁棒整形（ZVD）：缆长估计有误差时")

    sub("(a) 问题：整形器要知道 wn，而 wn 依赖 L")

    print("""      wn = sqrt(g/L)

  ★ 而缆长 L 在实际系统里是【会变的】：
      · 卷收机构放缆/收缆
      · 挂钩后载荷被抬起，有效缆长变化
      · 摆动大时，缆的等效长度也变

  => 整形器按某个 L_design 设计，但真实 L 不一样。看会发生什么。
""")

    sub("(b) ZV vs ZVD")

    print("""      ZV  : 2 个脉冲，幅度 (1/(1+K), K/(1+K))，间隔 T_d/2
      ZVD : 3 个脉冲，幅度按 (1, 2K, K^2) 归一化，间隔各 T_d/2
            无阻尼时 K=1 -> 幅度 (0.25, 0.5, 0.25)

  ★ ZVD 多花半个周期，换来对【频率误差】的容忍度。
""")

    Ld = 1.0
    wn = math.sqrt(G / Ld)
    t2 = math.pi / wn
    print("  设计 L = %.2f m  ->  wn = %.4f, 间隔 T/2 = %.4f s" % (Ld, wn, t2))
    print()

    def make_shaper(kind, dt2):
        seq = [(1.0, 0.0)] if kind == "none" else []
        if kind == "zv":
            seq = [(0.5, 0.0), (0.5, dt2)]
        elif kind == "zvd":
            seq = [(0.25, 0.0), (0.5, dt2), (0.25, 2 * dt2)]
        def a(t, seq=seq, duty=0.6, A=2.0):
            return sum(A * amp for amp, tau in seq if tau <= t < tau + duty)
        return a

    print("      %-8s %-9s %-9s %-12s %-12s %-12s" %
          ("真实L", "真实wn", "频率误差", "未整形", "ZV", "ZVD"))
    print("      " + "-" * 66)
    rows = []
    for Lt in [0.7, 0.85, 1.0, 1.2, 1.5]:
        vals = []
        for kind in ["none", "zv", "zvd"]:
            t, h = sim_prescribed(make_shaper(kind, t2), 16.0, Lt)
            vals.append(rms_tail(t, h[:, 0], 6.0))
        rows.append((Lt, vals))
        print("      %-8.2f %-9.4f %+8.1f%% %-12.4e %-12.4e %-12.4e"
              % (Lt, math.sqrt(G / Lt),
                 100 * (math.sqrt(G / Lt) / wn - 1), vals[0], vals[1], vals[2]))
    print("""
  ★ 设计点（L=1.0）：ZV 残余 2.8e-4 —— 几乎完美
  ★ 缆长偏 +-20%%：ZV 涨到 3e-2 ~ 8e-2，**ZVD 只有它的 1/3 ~ 1/7**

  ==> **【鲁棒整形】的含义：拿一点点标称性能，换很宽的工作范围。**
     这正是吊运场景需要的 —— 因为你不知道真实 L。
""")

    if HAVE_PLT:
        sub("(c) 画图")
        Ls = np.linspace(0.6, 1.6, 60)
        curves = {k: [] for k in ["none", "zv", "zvd"]}
        for Lt in Ls:
            for k in curves:
                t, h = sim_prescribed(make_shaper(k, t2), 16.0, Lt)
                curves[k].append(rms_tail(t, h[:, 0], 6.0))
        plt.figure(figsize=(6.4, 4.2))
        for k, lab, st in [("none", "unshaped", ":"),
                           ("zv", "ZV", "-"), ("zvd", "ZVD", "--")]:
            plt.semilogy(Ls, np.maximum(curves[k], 1e-6), st, label=lab)
        plt.axvline(1.0, c="gray", ls=":", lw=1)
        plt.xlabel("true cable length L (m)")
        plt.ylabel("residual swing RMS (rad)")
        plt.title("Shapers vs cable-length uncertainty")
        plt.legend(fontsize=8)
        plt.grid(alpha=0.3, which="both")
        plt.tight_layout()
        out = "slung_shaper_robust.png"
        plt.savefig(out, dpi=110)
        print("  已保存 %s" % out)


# ===========================================================================
# 实验 5  ★★ 缆绳的单边约束
# ===========================================================================

def exp5():
    head("实验 5  ★★ 缆绳的单边约束：什么时候会松弛")

    sub("(a) 缆绳只能拉，不能推")

    print("""  ★★★ 前三课假设的执行器都是【双向】的（推力可以减小到 0，力矩可以正负）。
     但缆绳是一个【单边约束】：它只能产生 **T >= 0** 的拉力。

  ★ 载荷竖直方向方程（NED 向下为正）：

        m_p * z_p'' = m_p g - T cos(theta)

    准静态（theta 小、载荷跟着无人机动）：  z_p'' ~= a_z（无人机的竖直加速度）

        =>   T = m_p (g - a_z)
""")
    print("      %-24s %-12s %-12s" % ("无人机 a_z (m/s², 向下正)", "张力 T (N)", "状态"))
    print("      " + "-" * 52)
    for az in [-9.8, -5.0, 0.0, 5.0, 9.0, 9.8, 12.0]:
        T = 1.0 * (G - az)
        st = "松弛！" if T <= 0 else ("临界" if T < 0.98 else "绷紧")
        note = "   <- 自由落体" if abs(az - G) < 1e-6 else ""
        print("      %-24.1f %-12.3f %-12s%s" % (az, T, st, note))
    print("""
  ★★ 判据：**a_z > g 时缆绳松弛** —— 无人机下坠得比自由落体还快，
     载荷"追上"了无人机，缆绳失去张力。
""")

    sub("(b) ★★ 松弛意味着什么：模型切换")

    print("""  缆绳一松，系统从【1 个约束、5 个自由度】变成【2 个独立刚体、6 个自由度】：

      绷紧时：x_p = x_d + L sin(theta)     <- 约束成立，两体耦合
      松弛时：载荷自由落体，无人机独立     <- 约束消失，两体解耦

  ★★★ 这叫**混合系统**（hybrid system）：
     连续动力学 + 离散的"事件"（绷紧/松弛切换）。

  ★ 对控制的两个直接后果：

      ① 【增益跳变】绷紧/松弛的动力学完全不同，
         同一个控制器在两种模式下表现可能天差地别
      ② 【冲击】松弛后再绷紧的瞬间，缆绳被"突然拉直"，
         产生一个冲击载荷 —— 可能扯断缆绳或拉翻无人机

  ⚠️ 所以吊运**必须避免松弛**，这就给出了加速度的约束：

        a_z <= g        （向下）
        a_x <= ?        （水平方向，见下）
""")

    sub("(c) 水平方向的约束")

    print("""  ★ 水平方向同样有约束：拽得太猛，载荷会"荡"过头，
    等效于水平方向出现了很大的相对运动。

  ★ 用本课的模型可以直接算：无人机以 a_x 加速时，
    稳态摆角（如果来得及稳定）满足

        tan(theta_ss) = a_x / g

    => 想要摆角不超过 theta_max，水平加速度必须满足

        a_x <= g * tan(theta_max)
""")
    print("      %-18s %-20s %s" % ("允许摆角", "允许水平加速度", "说明"))
    print("      " + "-" * 58)
    for deg in [5, 10, 15, 20, 30]:
        ax = G * math.tan(math.radians(deg))
        print("      %-18s %-20.3f %s" % ("%d°" % deg, ax, "保守" if deg <= 10 else
                                          ("可用" if deg <= 20 else "激进")))
    print("""
  ★ 对比一下：01 课 §3.5 说四旋翼悬停时最大倾角通常允许 30~45°，
    对应的水平加速度是 5.7~9.8 m/s²。
    **而吊运只允许 5~10° => 1~1.7 m/s²。**
    ==> 吊运把无人机的水平机动能力【压缩了 5 倍以上】。
""")


# ===========================================================================
# 实验 6  ★★ 端到端闭环
# ===========================================================================

def exp6():
    head("实验 6  ★★ 端到端闭环：整形 vs 不整形")

    sub("(a) 完整的闭环：位置环 + 吊挂动力学")

    print("""  实验 3/4 是"给定支点加速度"的开环研究。
  这里接上真实的闭环：无人机有位置环，载荷通过缆绳反作用。

  ★ 位置环：【也】用 ZV 整形 —— 做法是把位置指令拆成两段，
    让位置环自己产生整形后的加速度。
    这比"整形力指令"更实际，因为位置环是非线性的（有限幅）。
""")
    L, m_d, m_p, kp, kd = 1.0, 1.0, 1.0, 8.0, 5.657
    wn = math.sqrt(G / L)
    t2 = math.pi / wn
    D = 2.0                        # 目标位移
    T_FADE = 1.5                   # 平滑过渡时间

    def run(kind, T=20.0, dt=1e-4):
        n = int(T / dt)
        s = np.zeros(4)
        out = np.zeros((n, 5))
        for k in range(n):
            t = k * dt
            if kind == "step":
                ref = D
            elif kind == "smooth":        # 正弦过渡：指令本身连续可导
                ref = D * (0.5 - 0.5 * math.cos(math.pi * min(t / T_FADE, 1.0)))
            elif kind == "zv":            # 指令拆成两段，间隔半个摆动周期
                ref = 0.0 if t < t2 else (D / 2 if t < 2 * t2 else D)
            Fx = -kp * (s[0] - ref) - kd * s[1]
            out[k] = [s[0], s[1], s[2], s[3], ref]
            k1 = deriv(s, Fx, m_d, m_p, L)
            k2 = deriv(s + dt / 2 * k1, Fx, m_d, m_p, L)
            k3 = deriv(s + dt / 2 * k2, Fx, m_d, m_p, L)
            k4 = deriv(s + dt * k3, Fx, m_d, m_p, L)
            s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return np.arange(n) * dt, out

    print()
    print("  目标位移 D = %.1f m。看【每秒内的最大摆角】——" % D)
    print("  它比【稳态值】更有意义，因为位置环最终总能把摆耗掉，")
    print("  真正伤人的是过程中的【瞬态幅值】。")
    print()
    print("      %-10s %s" % ("方式", "每秒最大 |theta| (rad)"))
    print("      " + "-" * 62)
    env_all = {}
    for name, kind in [("阶跃指令", "step"), ("平滑指令", "smooth"), ("ZV 整形", "zv")]:
        t, o = run(kind)
        env = []
        for tt in [1, 2, 3, 4, 5, 6]:
            seg = (t >= tt) & (t < tt + 1.0)
            env.append(float(np.abs(o[seg, 2]).max()))
        env_all[kind] = env
        print("      %-10s %s" % (name, "  ".join("%.3f" % v for v in env)))
    print("      %-10s %s" % ("", "  ".join("%-5s" % ("%ds" % s) for s in [1, 2, 3, 4, 5, 6])))
    pk = {k: max(v) for k, v in env_all.items()}
    print("""
  ★★ 三条读数：

    ① **阶跃**：摆角峰值出现在【第 1 秒】（%.3f）—— 指令一上去就猛拽一下。

    ② **平滑**：峰值推迟到【第 2 秒】，且降到 %.3f
       —— 说明【问题出在指令的高频成分】，不是出在反馈。

    ③ ★★★ **ZV**：峰值 %.3f，出现在【两个时刻】——
       第 1 秒（对应第一段指令）和第 3 秒（对应第二段，2.0 s 才发出）。

  ★★★ 但请【注意 ZV 的形状】：它不是【没有摆动】，而是
      **把一个大摆动，换成了两个小摆动**：

        阶跃：  0.490                 —— 一个峰
        平滑：  0.255  0.304          —— 两个峰，但都够大
        ZV  ：  0.255  0.095  0.188   —— 两个峰，中间被压到 0.095

  ==> **整形不【消除】摆动 —— 它把摆动【摊开】到更长的时间里，
      峰值因此降低。**

  ★ 看 ZV 第 2 秒那个 0.095：**那正是"反相抵消"在起作用** ——
    第一段引起的摆动刚长到一半，第二段就把它压回去了。
    这也是为什么第 3 秒又会起来：抵消只对【第一段产生的那半个周期】有效，
    第二段自己产生的摆动没人抵消。
""" % (pk["step"], pk["smooth"], pk["zv"]))

    sub("(c) ★★★ 为什么闭环里 ZV 不像开环那么神")

    print("""  ★ 对比一下实验 3（开环）和这里（闭环）的 ZV 效果：

      开环（直接给定支点加速度）：残余 2.8e-4 vs 未整形 2.0e-1
                                  => 降低 **776 倍**，几乎是"消除"
      闭环（整形【参考位置】）    ：峰值 %.3f vs %.3f
                                  => 只降低 **约 %.1f 倍**

  ★★ 差别在哪？**开环时我们能精确控制支点加速度 a(t)；
     闭环时我们只能整形【参考位置】，而位置环会把整形后的指令"抹圆"**：
     无人机的实际加速度是 kp*(ref - x_d) - kd*v_d，它【不是】ref 的二阶导数。

  ==> 整形器设计时假设的 a(t)，和无人机实际给出的 a(t) 不是一回事。

  ★★★ 这就是「输入整形在实验室效果很好、上真机打折扣」的根本原因。
     工程上有两条补救：
        · 把整形器放在【加速度指令】这一层（而不是位置指令层）
        · 或者用【闭环整形器】（把位置环的动态也纳入设计）

  ⚠️ 本课的仿真没有做这两件事 —— 所以上面那个 %.1f 倍是"朴素做法"的结果，
     真实的系统可以做得更好，也可以更差。
""" % (pk["zv"], pk["step"], pk["step"] / pk["zv"], pk["step"] / pk["zv"]))

    if HAVE_PLT:
        sub("(b) 画图")
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        for name, sh, st in [("step", "step", ":"), ("smooth", "smooth", "--"), ("ZV", "zv", "-")]:
            t, o = run(sh)
            ax[0].plot(t, o[:, 2], st, label=name)
            ax[1].plot(t, o[:, 0], st, label=name)
            ax[1].plot(t, o[:, 4], "--", lw=0.8, alpha=0.5)
        ax[0].set_xlabel("t (s)")
        ax[0].set_ylabel("swing angle theta (rad)")
        ax[0].set_title("Payload swing")
        ax[0].legend(fontsize=8)
        ax[0].grid(alpha=0.3)
        ax[1].set_xlabel("t (s)")
        ax[1].set_ylabel("x_d (m)")
        ax[1].set_title("Drone position (dashed = reference)")
        ax[1].legend(fontsize=8)
        ax[1].grid(alpha=0.3)
        plt.tight_layout()
        out = "slung_closedloop.png"
        plt.savefig(out, dpi=110)
        print("  已保存 %s" % out)


# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 精确动力学 + 两个极限频率", exp1),
    "2": ("★★★ 模态靠拢：频率 vs 位置环带宽", exp2),
    "3": ("★★★ 输入整形 ZV", exp3),
    "4": ("★★ 鲁棒整形 ZVD", exp4),
    "5": ("★★ 缆绳的单边约束", exp5),
    "6": ("★★ 端到端闭环", exp6),
}


def main():
    print()
    hr("#")
    print("#  第 9.8 周 · 06 吊运与消摆动力学")
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
