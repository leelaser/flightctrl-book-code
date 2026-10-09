#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lesson09h_vtol_transition.py —— 卷 1 第 12 章《VTOL 与倾转旋翼》配套仿真

    实验 1  倾转旋翼的推力矢量：cos 惩罚有多大
    实验 2  过渡走廊：机翼升力和旋翼推力谁接管
    实验 3  过渡轨迹：从悬停加速到巡航的时间历程
    实验 4  尾坐式的控制权限交接
    实验 5  三种 VTOL 架构的权衡
    实验 6  推力预算：过设计的代价

只用 numpy，不依赖 scipy。
运行： python lesson09h_vtol_transition.py [1-6]
"""

import sys
import math
import numpy as np

G = 9.81

# ─────────────────────────────────────────────────────────────────────
# 一个典型的「四旋翼 + 固定翼」复合式 VTOL（quadplane）
# ─────────────────────────────────────────────────────────────────────
VEH = dict(
    name       = '典型 25 kg 复合式 VTOL',
    m          = 25.0,        # kg
    S          = 0.60,        # m^2  机翼面积
    CL_max     = 1.40,        # 最大升力系数（失速）
    CL_cruise  = 0.80,        # 巡航升力系数（有余量）
    rho        = 1.225,       # kg/m^3
    Tmax_rotor = 480.0,       # N    四个升力旋翼的最大总推力
    D_cruise   = 40.0,        # N    巡航阻力
)
VEH['W'] = VEH['m'] * G


def banner(title):
    print()
    print('═' * 74)
    print('  ' + title)
    print('═' * 74)


def sub(title):
    print()
    print('── ' + title)
    print('─' * 74)


# ═════════════════════════════════════════════════════════════════════
# 实验 1 —— 倾转旋翼的推力矢量
# ═════════════════════════════════════════════════════════════════════

def exp1_tilt_vector():
    banner('实验 1 · 倾转旋翼的推力矢量：cos 惩罚有多大')

    print("""
   倾转旋翼的推力矢量随倾转角 delta 旋转：

        T_z = T * cos(delta)      <- 向上的分量（抗重力）
        T_x = T * sin(delta)      <- 向前的分量（加速）

   delta = 0   旋翼朝上，纯悬停
   delta = 90  旋翼朝前，纯平飞

   ★ 关键：悬停需要的【垂直分量】不变（还是 W），
     但旋翼产生的【总推力】必须更大。
     => 悬停所需推力  T(delta) = W / cos(delta)
     => 旋翼功率  P ~ T^1.5  （动量理论）
     => 功率比    P(delta)/P(0) = (1/cos delta)^1.5
     """)

    W = VEH['W']
    print('   机重 W = %.2f N' % W)
    print()
    print('   %-10s %12s %12s %12s %12s' %
          ('delta(°)', 'T_z/W', 'T_需(N)', 'T_需/W', '功率比'))
    print('   ' + '─' * 62)

    rows = []
    for d in (0, 15, 30, 45, 60, 75, 80, 85, 90):
        r = math.radians(d)
        c = math.cos(r)
        if c < 1e-9:
            rows.append((d, 0.0, float('inf'), float('inf'), float('inf')))
        else:
            Tneed = W / c
            Pratio = (1.0 / c) ** 1.5
            rows.append((d, c, Tneed, Tneed / W, Pratio))

    for d, c, Tneed, ratio, P in rows:
        if not math.isfinite(P):
            print('   %-10d %12s %12s %12s %12s' %
                  (d, '0.000', 'inf', 'inf', 'inf'))
        else:
            flag = ''
            if P > 2.0:
                flag = '  <== 代价爆炸'
            print('   %-10d %12.3f %12.2f %12.3f %12.4f%s' %
                  (d, c, Tneed, ratio, P, flag))

    sub('(a) 45 度那个点')
    r45 = math.radians(45)
    print('   cos(45°)      = %.6f' % math.cos(r45))
    print('   悬停推力      = W / cos45 = %.4f W  (多了 %.1f%%)' %
          (1 / math.cos(r45), (1 / math.cos(r45) - 1) * 100))
    print('   功率比        = (1/cos45)^1.5 = %.6f  (多了 %.1f%%)' %
          ((1 / math.cos(r45)) ** 1.5, ((1 / math.cos(r45)) ** 1.5 - 1) * 100))
    print()
    print('   ★★★ 读法：倾转到 45 度做「半悬停半平飞」时，')
    print('       推力只需要多 41%，但【功率要多 68%】——')
    print('       因为功率正比于推力的 1.5 次方。')
    print()
    print('   => 这就是为什么 VTOL 的过渡段是【最费电】的阶段，')
    print('      也是为什么过渡要【尽快】穿过 30~60 度这个区间。')

    sub('(b) 反解：给定推力余量，最多能倾转到多少度')
    print('   悬停时（V=0，机翼不产生升力）能倾转的最大角度：')
    print()
    print('        cos(delta_max) = W / T_max')
    print('        delta_max = arccos(1 / (T/W))')
    print()
    print('   %-12s %12s %14s' % ('T/W', 'delta_max(°)', '说明'))
    print('   ' + '─' * 42)
    for tw in (1.2, 1.5, 1.957, 2.0, 2.5, 3.0):
        if tw <= 1.0:
            continue
        dm = math.degrees(math.acos(1.0 / tw))
        note = ''
        if tw == 1.957:
            note = '<- 本章算例'
        if tw <= 1.5:
            note = '推力很紧，几乎不能倾转'
        print('   %-12.3f %12.2f %14s' % (tw, dm, note))

    tw = VEH['Tmax_rotor'] / VEH['W']
    print()
    print('   ★ 本章算例：T/W = %.3f -> 零速下最多倾转到 %.1f°' % (tw, math.degrees(math.acos(1 / tw))))
    print('   ★★ 想要零速就倾转到 90°，需要 T/W = 无穷大 —— 不可能。')
    print('   => 所以【过渡必须靠空速帮忙】：速度起来之后机翼接力。')
    return rows


# ═════════════════════════════════════════════════════════════════════
# 实验 2 —— 过渡走廊
# ═════════════════════════════════════════════════════════════════════

def wing_lift(V, CL, VEH=VEH):
    """机翼升力 L = 0.5 * rho * V^2 * S * CL"""
    return 0.5 * VEH['rho'] * V * V * VEH['S'] * CL


def exp2_corridor():
    banner('实验 2 · 过渡走廊：机翼升力和旋翼推力谁接管')

    W = VEH['W']
    Tm = VEH['Tmax_rotor']
    S = VEH['S']
    rho = VEH['rho']
    CLmax = VEH['CL_max']
    CLc = VEH['CL_cruise']

    print('''
   VTOL 平飞的力平衡：

        T * cos(delta) + L(V) = W        （垂直）
        T * sin(delta)        = D(V) + m*a    （水平）

   其中 L(V) 是机翼升力，正比于 V^2。
   所以【速度越大，旋翼要出的力越少，能倾转的角度越大】。

   ★★★ 这就定义了一条「过渡走廊」：
       对每个空速 V，存在一个【最大可倾转角】delta_max(V)。

        T * cos(delta) = W - L(V)   取 T = T_max
     => cos(delta_max) = (W - L(V)) / T_max
     => delta_max = arccos((W - L(V)) / T_max)
    ''')

    V_stall = math.sqrt(2 * W / (rho * S * CLmax))
    V_level = math.sqrt(2 * W / (rho * S * CLc))

    print('   机重 W        = %.2f N' % W)
    print('   旋翼最大推力  = %.2f N  (T/W = %.3f)' % (Tm, Tm / W))
    print('   机翼面积 S    = %.3f m^2' % S)
    print('   CL_max        = %.2f' % CLmax)
    print()
    print('   ★ 失速速度 V_stall = sqrt(2W/(rho*S*CLmax)) = %.2f m/s' % V_stall)
    print('   ★ 平飞速度 V_level = sqrt(2W/(rho*S*CL))    = %.2f m/s  (CL=%.2f)' %
          (V_level, CLc))
    print()
    print('   ★★ 注意这两个速度差 %.1f m/s —— 这就是【失速裕度】的代价。' %
          (V_level - V_stall))

    sub('(a) 过渡走廊表')
    print('   %-8s %10s %10s %14s %14s %s' %
          ('V(m/s)', 'L_max(N)', 'L/L_W', 'd_max(°)', 'd_max安全(°)', '谁在托着'))
    print('   ' + '─' * 74)

    Vs = np.array([0, 5, 10, 15, 18, V_stall, 24, V_level, 35, 40])
    for V in Vs:
        Lmax = wing_lift(V, CLmax)
        Lc = wing_lift(V, CLc)
        num = W - Lmax
        if num <= 0:
            dmax = 90.0
        else:
            dmax = math.degrees(math.acos(min(1.0, num / Tm)))
        num2 = W - Lc
        if num2 <= 0:
            d2 = 90.0
        else:
            d2 = math.degrees(math.acos(min(1.0, num2 / Tm)))
        holder = '旋翼' if Lmax < 0.15 * W else ('交接' if Lmax < 0.85 * W else '机翼')
        print('   %-8.1f %10.2f %10.3f %14.2f %14.2f %s' %
              (V, Lmax, Lmax / W, dmax, d2, holder))

    sub('(b) 走廊的形状')
    print('   把 delta_max(V) 画出来（横轴空速，纵轴可倾转角）')
    print('   ★ 纵轴只画 50~90 度这一段 —— 因为走廊的下边界恰好是 59.3 度，')
    print('     画 0~90 的话下面全是实心的，看不出形状。')
    print()
    grid = np.linspace(0, 40, 81)
    dms = []
    for V in grid:
        L = wing_lift(V, CLmax)
        num = W - L
        dms.append(90.0 if num <= 0
                   else math.degrees(math.acos(min(1.0, num / Tm))))
    for dline in range(90, 49, -5):
        row = ''.join('#' if dm >= dline else ' ' for dm in dms)
        tag = ''
        if dline == 90:
            tag = '  <- 触顶：机翼独立托住飞机'
        if dline == 55:
            tag = '  <- 左端之后就一直在这条线以上'
        print('   %3d° |%s%s' % (dline, row, tag))
    print('        +' + '─' * len(grid))
    print('         0        10        20        30        40 m/s')
    print()
    print('   ★ 曲线【向右上】走：空速越大，越敢倾转。')
    print('   ★★ 曲线【左端】不归零，而是停在 %.1f° ——' %
          math.degrees(math.acos(1 / (Tm / W))))
    print('      那就是实验 1(b) 的 arccos(W/T_max)。')
    print('   ★★★ 曲线右侧在 V = %.1f m/s 处【触顶到 90°】——' % V_stall)
    print('      那就是失速速度：机翼能独立托住飞机的最低空速。')
    print('   ★ 曲线【向右上】走：空速越大，越敢倾转。')
    print('   ★★ 曲线【左端】不归零，而是停在 %.1f° ——' %
          math.degrees(math.acos(1 / (Tm / W))))
    print('      那就是实验 1(b) 的 arccos(W/T_max)。')
    print('   ★★★ 曲线右侧在 V = %.1f m/s 处【触顶到 90°】——' % V_stall)
    print('      那就是失速速度：机翼能独立托住飞机的最低空速。')

    sub('(c) 走廊的「安全带」')
    print('   ★ 上面的 delta_max 用的是 CL_max —— 意味着机翼【正好在失速点】。')
    print('     实际飞行要有裕度，用 CL_cruise = %.2f 重算：' % CLc)
    print()
    print('   %-16s %14s %14s' % ('判据', 'V 触顶(m/s)', '相对失速速度'))
    print('   ' + '─' * 46)
    print('   %-16s %14.2f %14.2f' % ('CL_max (失速)', V_stall, 1.0))
    print('   %-16s %14.2f %14.2f' % ('CL_cruise (裕度)', V_level,
                                      V_level / V_stall))
    print()
    print('   ★★★ 过渡的目标速度必须取【后者】—— 如果按失速速度设计过渡，')
    print('       任何扰动都会让机翼失速，而失速的机翼升力【掉得比涨得快】。')
    print('   ★ 这就是「过渡走廊」真正的边界：不是物理极限，是【安全裕度】。')
    return V_stall, V_level


# ═════════════════════════════════════════════════════════════════════
# 实验 3 —— 过渡轨迹
# ═════════════════════════════════════════════════════════════════════

def drag_polar(V, VEH=VEH):
    """简化阻力：巡航点 D=40N@V=25，其余按 V^2 缩放（寄生） + 诱导项"""
    V0 = 25.0
    return VEH['D_cruise'] * (V / V0) ** 2


def _sim_transition(tilt_rate_deg, VEH, protect_corridor=True,
                    thrust_margin=0.85, dt=0.002, t_max=60.0):
    """跑一遍过渡。

    protect_corridor=True  -> 倾转角受走廊约束（不会掉高，但可能很慢）
    protect_corridor=False -> 按时间表硬倾（会掉高，但过渡快）

    返回 dict(t_end, h_drop, x_dist, V_end, trail)
    """
    W, m, Tm = VEH['W'], VEH['m'], VEH['Tmax_rotor']
    rate = math.radians(tilt_rate_deg)
    # 定高用的推力上限：留出余量去加速
    T_hold = thrust_margin * Tm
    V_level = math.sqrt(2 * W / (VEH['rho'] * VEH['S'] * VEH['CL_cruise']))

    V, h, x, delta, t = 0.5, 0.0, 0.0, 0.0, 0.0
    h_min, V_max_ever = 0.0, V
    t_end = None
    trail = []

    while t < t_max:
        L = wing_lift(V, VEH['CL_cruise'])
        num = W - L

        # 走廊上限（用 T_hold 而不是 T_max）
        if num <= 0:
            d_corridor = math.radians(90.0)
        else:
            d_corridor = math.acos(min(1.0, num / T_hold))

        # 倾转：按速率上升，可选地被走廊夹住
        delta_next = delta + rate * dt
        if protect_corridor:
            delta_next = min(delta_next, d_corridor)
        delta = min(delta_next, math.radians(90.0))

        # 定高控制器：解出保持 dh/dt=0 的推力
        c = math.cos(delta)
        if num <= 0:
            T = 0.0
        elif c < 1e-9:
            T = float('inf')
        else:
            T = num / c

        T_act = min(T, Tm)
        Vdot = (T_act * math.sin(delta) - drag_polar(V, VEH)) / m
        hdot = (T_act * math.cos(delta) + L - W) / m

        trail.append((t, V, h, math.degrees(delta), T, T <= Tm))

        V += Vdot * dt
        h += hdot * dt
        if V < 0.0:
            V = 0.0
        x += V * dt
        h_min = min(h_min, h)
        V_max_ever = max(V_max_ever, V)
        t += dt

        # 过渡完成 = 机翼能独立托住飞机（V >= V_level）
        if V >= V_level and t_end is None:
            t_end = t
            break

    return dict(t_end=(t_end if t_end else t), h_drop=-h_min,
                x_dist=x, V_end=V_max_ever, trail=trail,
                V_level=V_level)


def exp3_trajectory():
    banner('实验 3 · 过渡轨迹：从悬停加速到巡航的时间历程')

    W = VEH['W']
    Tm = VEH['Tmax_rotor']
    V_level = math.sqrt(2 * W / (VEH['rho'] * VEH['S'] * VEH['CL_cruise']))

    print('''
   把过渡当成一个【轨迹跟踪问题】来做仿真：

        状态: V (空速), h (高度)
        控制: delta (倾转角), T (总推力)

   运动方程：

        m dV/dt = T*sin(delta) - D(V)          （水平：加速）
        m dh/dt = T*cos(delta) + L(V) - W      （垂直：定高）

   ★ 定高控制器：每个时刻解出保持 dh/dt = 0 所需的推力
         T = (W - L(V)) / cos(delta)
     水平加速则靠同一个 T 的 sin 分量。

   ★★★ 这里有一个【致命的耦合】：
       定高控制器要的 T 在 delta 变大时会【暴涨】（cos 惩罚）。
       一旦 T 超过 T_max，高度就守不住了 —— 这就是掉高的来源。
    ''')

    print('   本章算例：')
    print('     目标过渡速度 V_level = %.2f m/s  (CL=%.2f)' %
          (V_level, VEH['CL_cruise']))
    print('     定高推力上限 T_hold   = %.1f N  (T_max 的 85%%，留 15%% 加速)' %
          (0.85 * Tm))
    print()

    THRUST_MARGIN = 0.85
    trail = None
    for rate in (12.0,):
        res = _sim_transition(rate, VEH, protect_corridor=True, thrust_margin=THRUST_MARGIN)
        trail = res['trail']

    sub('(a) 走廊保护下的时间历程（倾转 12°/s，每 1 秒采样）')
    print('   %-8s %10s %10s %10s %12s %10s %s' %
          ('t(s)', 'V(m/s)', 'h(m)', 'delta(°)', 'T_需(N)', 'T/T_max', '走廊'))
    print('   ' + '─' * 76)
    step = int(1.0 / 0.002)
    for k, (t_, V_, h_, d_, T_, ok) in enumerate(trail):
        if k % step != 0:
            continue
        Tstr = 'inf' if not math.isfinite(T_) else '%.1f' % T_
        rstr = 'inf' if not math.isfinite(T_) else '%.3f' % (T_ / Tm)
        print('   %-8.2f %10.2f %10.2f %10.1f %12s %10s %s' %
              (t_, V_, h_, d_, Tstr, rstr, 'OK' if ok else '★ 出界'))

    res = _sim_transition(12.0, VEH, protect_corridor=True, thrust_margin=THRUST_MARGIN)
    sub('(b) 读数')
    print('   过渡耗时      = %.2f s' % res['t_end'])
    print('   最终空速      = %.2f m/s  (目标 %.2f)' % (res['V_end'], V_level))
    print('   最大掉高      = %.3f m' % res['h_drop'])
    print('   水平距离      = %.1f m' % res['x_dist'])
    print()
    print('   ★ 走廊保护起作用了：整个过渡【不掉高】。')
    print('   ★★ 代价是：倾转角被走廊【夹住】，不能想倾多快就倾多快。')

    sub('(c) ★★★ 走廊保护 vs 硬倾转')
    print('   同一个过渡，两种策略：')
    print('     A 保护走廊：倾转不超过 delta_max(V)，靠余量慢慢加速')
    print('     B 硬倾转：  按时间表一直倾，不管走廊')
    print()
    print('   ★ 但先说一个【反直觉的观察】：在本章这台 T/W=1.96 的飞机上，')
    print('     两种策略几乎没差别 —— 因为推力余量足够大，')
    print('     走廊从一开始就很宽（V=0 时就有 %.1f°）。' %
          math.degrees(math.acos(VEH['W'] / (THRUST_MARGIN * VEH['Tmax_rotor']))))
    print()
    print('   ★★ 真正会出问题的是【推力余量小】的飞机。')
    print('      下面用两台飞机比：')
    print()

    VEH_HI = VEH
    VEH_LO = dict(VEH, name='低推力余量 VTOL', Tmax_rotor=318.0)
    for tag, V_ in (('高余量', VEH_HI), ('低余量', VEH_LO)):
        tw = V_['Tmax_rotor'] / V_['W']
        dc0 = math.degrees(math.acos(min(1.0, V_['W'] / (THRUST_MARGIN * V_['Tmax_rotor']))))
        print('     %s：T/W = %.2f,  V=0 时走廊上限 = %.1f°' % (tag, tw, dc0))
    print()

    for tag, V_ in (('高余量 (T/W=%.2f)' % (VEH_HI['Tmax_rotor'] / VEH_HI['W']), VEH_HI),
                    ('低余量 (T/W=%.2f)' % (VEH_LO['Tmax_rotor'] / VEH_LO['W']), VEH_LO)):
        print('   【%s】' % tag)
        print('   %-14s %12s %12s %12s %12s' %
              ('倾转速率(°/s)', '耗时A(s)', '掉高A(m)', '耗时B(s)', '掉高B(m)'))
        print('   ' + '─' * 64)
        for rate in (5.0, 10.0, 20.0, 40.0):
            ra = _sim_transition(rate, V_, protect_corridor=True, thrust_margin=THRUST_MARGIN)
            rb = _sim_transition(rate, V_, protect_corridor=False, thrust_margin=THRUST_MARGIN)
            print('   %-14.1f %12.2f %12.3f %12.2f %12.3f' %
                  (rate, ra['t_end'], abs(ra['h_drop']),
                   rb['t_end'], abs(rb['h_drop'])))
        print()

    print('   ★★★ 读表：')
    print('     · 高余量那台：A、B 两列几乎一样 —— 推力够，怎么倾都不掉。')
    print('     · 低余量那台：B 列（硬倾）掉高明显，而且【倾转越快掉得越多】；')
    print('       A 列（保护走廊）掉高恒为 0，代价是【多花时间】。')
    print()
    print('   ★★ 为什么 B 会掉高？一旦 delta 超过走廊，')
    print('      定高控制器要求的 T = (W-L)/cos(delta) 就超过了 T_max，')
    print('      推力饱和 → 垂直分量不够 → 掉高。')
    print()
    print('   ★ 为什么掉高【随倾转速率上升】？')
    print('     倾转越快，飞机在「低速 + 大倾角」这个最糟的组合里待得越久 ——')
    print('     空速还没起来（机翼不帮忙），倾角却已经很大（cos 惩罚拉满）。')
    print()
    print('   ★★★ 一个直接推论：')
    print('     「过渡速率」这个参数【不能独立调】——')
    print('     它的安全上限由【推力余量】决定。')
    print('     推力余量大 => 可以快速过渡；推力余量小 => 必须慢慢过渡。')
    print('   ★ 这就是实验 6 要算的那笔账：')
    print('     多装推力 = 多花重量，但换来【更短更安全的过渡】。')
    return trail


# ═════════════════════════════════════════════════════════════════════
# 实验 4 —— 尾坐式的控制权限交接
# ═════════════════════════════════════════════════════════════════════

def exp4_tailsitter():
    banner('实验 4 · 尾坐式的控制权限交接')

    print('''
   尾坐式（tailsitter）是 VTOL 里最"极端"的一种：

        【没有倾转机构，也没有独立的升力旋翼】
        同一套旋翼，既做悬停也做巡航
        切换方式 = 【整个飞机转过来】

   悬停时机头朝上（攻角 90°），平飞时机头朝前（攻角 0°）。
   中间的过程，飞机一边转、一边加速。

   ★★★ 关键问题：控制在【谁】手里？

        悬停时：靠【旋翼推力差】产生力矩   （和多旋翼一样）
        平飞时：靠【气动舵面】产生力矩     （和固定翼一样）
     => 中间必然有一个【交接点】：两种手段都不够强的区间。
    ''')

    W = VEH['W']

    # 尾坐式通常是个飞翼，舵面是升降副翼（elevon）
    b = 2.0            # m    翼展
    CL_delta = 0.35    # 1/rad  滚转舵效
    d_motor = 1.0      # m    单个旋翼到机身中线的距离（半翼展）
    Tmax_prop = 300.0  # N    两个旋翼总推力
    delta_max = math.radians(20.0)

    print('   算例：飞翼尾坐式')
    print('     翼展 b          = %.2f m' % b)
    print('     滚转舵效 CL_d   = %.2f /rad' % CL_delta)
    print('     旋翼到中线距离  = %.2f m' % d_motor)
    print('     旋翼最大总推力  = %.2f N   (T/W = %.3f)' %
          (Tmax_prop, Tmax_prop / W))

    sub('(a) ★★★ 尾坐式的控制分配：和四旋翼完全不同')
    print('''
   先看 AP_MotorsTailsitter 怎么分配四个通道：

        // AP_MotorsTailsitter.cpp:173-175
        _thrust_left  = throttle_thrust + roll_thrust * 0.5f;
        _thrust_right = throttle_thrust - roll_thrust * 0.5f;

        // AP_MotorsTailsitter.cpp:212-214
        _tilt_left  = pitch_thrust - yaw_thrust;
        _tilt_right = pitch_thrust + yaw_thrust;

   ★★★ 读出来：

        通道        手段
        ─────────────────────────────────────────
        throttle    两个电机【同增】
        roll        两个电机【差动】       <- 抢油门！
        pitch       两个倾转舵机【同向】
        yaw         两个倾转舵机【差动】

   ★ 对比四旋翼：四个电机【各自】分担 roll/pitch/yaw/throttle。
     尾坐式只有两个电机，而且它们还要负责【全部升力】。
   ★★ 所以 【roll 没有自己的执行器】—— 它是从油门里"抢"出来的。
    ''')

    hover_throttle = W / Tmax_prop
    print('   本章算例的悬停油门 h = W/T_max = %.4f' % hover_throttle)
    print()

    sub('(b) ★★★ 抢出来的滚转权限：正好等于「推力余量 × 力臂」')
    print('   归一化推力：left = h + roll*0.5,  right = h - roll*0.5')
    print('   约束：left <= 1  且  right >= 0')
    print()
    print('        roll <= 2*(1 - h)      （左边不能超满）')
    print('        roll <= 2*h            （右边不能变负）')
    print()
    roll_max = 2.0 * min(1.0 - hover_throttle, hover_throttle)
    print('   h = %.4f  =>  roll_max = 2*min(1-h, h) = %.4f' %
          (hover_throttle, roll_max))
    print()
    print('   实际的推力差 = roll_max * (T_max/2) = %.2f N' % (roll_max * Tmax_prop / 2))
    print('   滚转力矩     = 推力差 * d = %.2f N·m' %
          (roll_max * Tmax_prop / 2 * d_motor))
    print()
    tau_prop = min(W, Tmax_prop - W) * d_motor
    print('   ★★★ 化简一下（注意本算例 h > 0.5，所以取的是 1-h 那一边）：')
    print('        roll_max * (T_max/2) * d')
    print('      = 2*(1-h) * (T_max/2) * d')
    print('      = (1-h) * T_max * d')
    print('      = (T_max - W) * d')
    print('      = (%.1f - %.1f) * %.1f = %.2f N·m  ✓' %
          (Tmax_prop, W, d_motor, (Tmax_prop - W) * d_motor))
    print()
    print('   ★★★★★ 但是要小心：约束有【两条】，哪条先咬取决于 h')
    print()
    print('        h > 0.5  (T/W < 2)：左边先满  ->  (T_max - W) * d')
    print('        h < 0.5  (T/W > 2)：右边先空转 ->  W * d')
    print()
    print('   => 合起来：')
    print()
    print('            tau_roll_max = d * min( W , T_max - W )')
    print()
    print('   ★★★★★ 这个式子是【对称】的 —— 它关于 T_max = 2W 对称！')
    print()
    print('   ★ 两个【可怕】的推论：')
    print('       ① T_max = W（推力刚好等于重力）=> 权限 = 0')
    print('          飞机能悬停，但【完全不能滚转】。')
    print('       ② T_max 做得【太大】也会降低权限！')
    print('          因为悬停油门 h 变小，右边那个电机很快就会掉到 0。')
    print()
    print('   ★★ 所以尾坐式的 T/W 【不是越大越好】—— 它有一个【最优点】：')
    print('          T/W = 2，此时 tau_roll_max = W * d')
    print('      推力在 2 倍重力时，滚转权限最大。')

    sub('(c) 悬停滚转权限 vs 推力余量')
    print('   %-10s %12s %12s %14s %14s' %
          ('T/W', '悬停油门h', 'roll_max', 'tau(d·(T-W))', 'tau(d·min)'))
    print('   ' + '─' * 66)
    for tw in (1.05, 1.2, 1.5, 2.0, 3.0, 5.0):
        Tm_ = W * tw
        h_ = 1.0 / tw
        rm = 2.0 * min(1.0 - h_, h_)
        naive = (Tm_ - W) * d_motor
        exact = min(W, Tm_ - W) * d_motor
        tag = ''
        if abs(tw - 2.0) < 1e-9:
            tag = '  <- 峰值'
        if tw > 2.0:
            tag = '  <- 天真的式子【高估】了'
        print('   %-10.2f %12.4f %12.4f %14.2f %14.2f%s' %
              (tw, h_, rm, naive, exact, tag))
    print()
    print('   ★★★ 看最后两列：T/W = 5 时，')
    print('       「(T_max-W)*d」算出 %.1f N·m，' % ((W * 5 - W) * d_motor))
    print('       但真实的 min 形式只有 %.1f N·m —— 【差了一倍】。' % (W * d_motor))

    sub('(c2) 用第 02 课的语言说：这是一条【可达集边界】')
    print('   两个电机各有 [0, T_max/2] 的推力：')
    print()
    print('        L = h*(T_max/2) + Δ        R = h*(T_max/2) - Δ')
    print('        约束:  0 <= L <= T_max/2   且   0 <= R <= T_max/2')
    print('   => |Δ| <= (T_max/2) * min(h, 1-h)')
    print('   => tau = 2Δ*d <= T_max*d*min(h,1-h) = d*min(W, T_max-W)  ✓')
    print()
    print('   ★★ 这正是 05 课 §7.4「可达力旋量集」的形状：')
    print('      两个电机的 [0, T_max/2] 是个【方盒子】，')
    print('      你在盒子里找一个点，使 (L-R) 最大。')
    print('      => 最优点【永远在盒子的角上】，所以是个 min 形式。')
    print('   ★★★ 一模一样的分析方法，只是这次盒子的维度是 2。')

    sub('(d) ★★★ 和舵面的交接：一个是常数，一个是 V^2')
    print('   气动滚转力矩（一对舵面差动）：')
    print('        tau_aero = 0.5*rho*V^2*S*b*CL_delta*delta_max')
    print()
    print('   %-10s %16s %16s %10s' %
          ('V(m/s)', 'tau_aero(N·m)', 'tau_prop(N·m)', '谁强'))
    print('   ' + '─' * 58)
    Vx = None
    for V in np.arange(0, 30, 2.0):
        tau_a = 0.5 * VEH['rho'] * V * V * VEH['S'] * b * CL_delta * delta_max
        who = '舵面' if tau_a > tau_prop else '旋翼'
        if Vx is None and tau_a > tau_prop:
            Vx = V
        print('   %-10.1f %16.2f %16.2f %10s' % (V, tau_a, tau_prop, who))

    Vx_exact = math.sqrt(2.0 * tau_prop /
                         (VEH['rho'] * VEH['S'] * b * CL_delta * delta_max))
    print()
    print('   精确交接空速 V_cross = %.2f m/s' % Vx_exact)
    print()
    print('   ★ 注意 tau_prop 是【常数】—— 它不随空速变化；')
    print('     而 tau_aero 正比于 V^2，所以两者必然【只交一次】。')
    print('   ★★ 低于 %.1f m/s：舵面几乎没用，全靠旋翼差动' % Vx_exact)
    print('   ★★ 高于 %.1f m/s：舵面接管，旋翼可以歇了' % Vx_exact)
    print()
    print('   ★★★ 但这里是【成对】的比例关系 —— ')
    print('       舵面力矩每提高一倍，交接空速只要降低到 1/sqrt(2)。')
    print('       换句话说：想到降低交接空速，靠加舵面面积【很不划算】。')
    print('       更有效的办法是把 T/W 做大（增加 tau_prop）。')

    sub('(e) ★★★ 双重困难：cos 惩罚 + 权限交接同时发生')
    print('   尾坐式在过渡中，机身【一边转一边加速】：')
    print()
    print('        旋翼的向上分量 = T * cos(theta)')
    print('        （theta = 机身相对垂直的倾角）')
    print()
    print('   ★ 这和倾转旋翼的 cos 惩罚【一模一样】。')
    print()
    print('   ★★★ 所以尾坐式的过渡控制是【双重的难】：')
    print('         ① cos 惩罚：推力不够，而推力还要分出来做滚转')
    print('         ② 权限交接：力矩手段在换手（旋翼 -> 舵面）')
    print('       而这两件事发生在【同一段时间】里。')
    print()
    print('   ★ 更糟的是 ① 和 ② 会【互相放大】：')
    print('     cos 惩罚 => T 要变大 => 悬停油门 h 变大 =>')
    print('     滚转余量 2*(1-h) 变小 => 旋翼的滚转权限【反而下降】')
    print()
    print('   ★★ 这就是为什么实战里尾坐式比倾转旋翼少见 ——')
    print('      它的过渡走廊更窄，而且没有独立的升力系统兜底。')
    return Vx_exact


# ═════════════════════════════════════════════════════════════════════
# 实验 5 —— 三种 VTOL 架构的权衡
# ═════════════════════════════════════════════════════════════════════

def exp5_architectures():
    banner('实验 5 · 三种 VTOL 架构的权衡')

    W = VEH['W']
    print('''
   把三种主流 VTOL 架构放在同一架 25 kg 飞机上比：

      A. 复合式（Lift + Cruise）   独立的升力旋翼 + 独立的巡航推进
      B. 倾转旋翼（Tiltrotor）     同一套旋翼，靠舵机倾转
      C. 尾坐式（Tailsitter）      同一套旋翼，靠整个飞机转
    ''')

    # 公共：巡航需要推力 D_cruise，悬停需要 W
    D = VEH['D_cruise']
    print('   共同需求：')
    print('     悬停推力  >= %.1f N  (W)' % W)
    print('     巡航推力  >= %.1f N  (D_cruise)' % D)
    print('     推力比    = %.2f' % (W / D))
    print()

    sub('(a) 装机功率与重量')
    print('   %-14s %14s %14s %14s' %
          ('架构', '升力系统', '巡航系统', '总推力上限(N)'))
    print('   ' + '─' * 58)
    # (name, 升力系统, 巡航系统, 总推力上限, 悬停时工作的桨数, 全机桨数)
    arch = [
        ('A 复合式', '4 旋翼 480N', '1 推进 60N', 480 + 60, 4, 5),
        ('B 倾转旋翼', '4 旋翼 480N', '同一套（倾转）', 480, 4, 4),
        ('C 尾坐式', '2 旋翼 300N', '同一套（机体转）', 300, 2, 2),
    ]
    for name, lift_, cruise_, tot, nh, nt in arch:
        print('   %-14s %14s %14s %14.0f' % (name, lift_, cruise_, tot))

    print()
    print('   ★ A 的重量最大（两套系统），但【过渡最简单】。')
    print('   ★ C 的重量最轻（只有两个旋翼），但【过渡最难】。')

    sub('(b) 悬停效率对比（同样 25 kg，同样桨盘面积假设）')
    print('   悬停功率（动量理论）：P = T^1.5 / sqrt(2*rho*A)')
    print()
    A_per_rotor = 0.28        # m^2  单个旋翼桨盘面积
    print('   假设单旋翼桨盘面积 A = %.2f m^2' % A_per_rotor)
    print('   ★ 注意：悬停时只有【升力旋翼】在转，')
    print('     A 的巡航推进桨这时是停的，不贡献桨盘面积。')
    print()
    print('   %-14s %10s %14s %14s %14s' %
          ('架构', '悬停桨数', '桨盘(m^2)', '悬停功率(W)', '相对'))
    print('   ' + '─' * 68)
    P0 = None
    for name, lift_, cruise_, tot, nh, nt in arch:
        A = A_per_rotor * nh
        P = (W ** 1.5) / math.sqrt(2 * VEH['rho'] * A)
        if P0 is None:
            P0 = P
        print('   %-14s %10d %14.2f %14.1f %14.3f' %
              (name, nh, A, P, P / P0))
    print()
    print('   ★★ 桨盘面积越大，悬停越省功率 —— 这就是「大桨慢转」的原理。')
    print('   ★ A 和 B 悬停时都是 4 个桨盘，所以悬停效率【完全相同】——')
    print('     尽管 A 多背了一个巡航推进系统。')
    print('   ★★ C 只有 2 个旋翼，桨盘面积【正好一半】 => 悬停功率多 41.4%。')
    ratio = math.sqrt(2.0)
    print('     sqrt(2) = %.4f  <- 正好是 %.1f%%' % (ratio, (ratio - 1) * 100))
    print()
    print('   ★★★ 这一条就是尾坐式的【根本代价】：')
    print('       它的两个旋翼【既要悬停又要巡航】，桨盘面积被两头拉扯。')
    print('       桨小 => 悬停费电；桨大 => 巡航阻力大。')
    print('       而复合式可以【各配各的】：升力桨大而慢，推进桨小而快。')

    sub('(c) 过渡难度：三个指标')
    print('   %-14s %14s %14s %14s' %
          ('架构', '倾转机构', '过渡走廊', '失效模式数'))
    print('   ' + '─' * 58)
    tbl = [
        ('A 复合式', '无（升力旋翼只开关）', '宽', '3'),
        ('B 倾转旋翼', '有（舵机倾转）', '中', '5'),
        ('C 尾坐式', '无（机体整体转）', '窄', '4'),
    ]
    for row in tbl:
        print('   %-14s %14s %14s %14s' % row)
    print()
    print('   ★ 说明：')
    print('     · 「过渡走廊」宽窄 = 走廊内允许的空速/倾转角组合有多少')
    print('     · 「失效模式数」= 需要单独处理的特情的种类（估计值）')
    print('     · A 的升力旋翼只需要【开/关】（或小范围转速调节），')
    print('       所以它的过渡本质上是「推力交接」而不是「矢量旋转」')
    print()
    print('   ★★★ 一句话总结三条路线：')
    print('       A：用【重量】换【简单】')
    print('       B：用【机构复杂度】换【效率】')
    print('       C：用【控制复杂度】换【重量】')

    sub('(d) 巡航效率')
    print('   VTOL 平飞时，若旋翼还在转（哪怕不出力），也有【废阻力】：')
    print()
    print('   %-14s %14s %14s %14s' %
          ('架构', '巡航时旋翼', '废阻力(N)', '巡航功率(W)'))
    print('   ' + '─' * 58)
    # 停转的旋翼阻力小，转着的旋翼阻力大
    for name, spinning, drag in (('A 复合式', '停转', 6.0),
                                 ('B 倾转旋翼', '停转（或顺桨）', 6.0),
                                 ('C 尾坐式', '★ 必须继续转', 22.0)):
        P = (D + drag) * 25.0     # 巡航速度 25 m/s
        print('   %-14s %14s %14.2f %14.1f' % (name, spinning, drag, P))
    print()
    print('   ★★★ 尾坐式的巡航必须让旋翼继续转（它还要出推力），')
    print('       而且转速不低 —— 这带来额外阻力/功率。')
    print('   ★ 这是尾坐式「重量最轻但效率不高」的根源：')
    print('     悬停要小桨（省重量）=> 巡航效率差；')
    print('     巡航要大桨（高效率）=> 悬停桨盘不够。')
    print('   ★★ 尾坐式的旋翼必须【同时】满足两个互相矛盾的需求。')
    return arch


# ═════════════════════════════════════════════════════════════════════
# 实验 6 —— 推力预算：过设计的代价
# ═════════════════════════════════════════════════════════════════════

def exp6_thrust_budget():
    banner('实验 6 · 推力预算：过设计的代价')

    W = VEH['W']
    D = VEH['D_cruise']
    print('''
   VTOL 有一个所有机型都绕不开的矛盾：

        悬停要 T/W >= 1     （最好 1.5~2，因为要有控制余量）
        巡航要 T/W >= 0.16  （D/W = %.2f 就够了）

   => 装机推力必须是巡航需求的【%d 倍】左右！
   => 而多出来的推力，在巡航时全是【死重和废阻力】。
   ''' % (D / W, int(W / D)))

    sub('(a) 推力需求谱')
    print('   %-18s %14s %14s' % ('工况', '需要的推力(N)', 'T/W'))
    print('   ' + '─' * 48)
    needs = [
        ('纯悬停（无余量）', W, W / W),
        ('悬停 + 30% 控制余量', W * 1.3, 1.3),
        ('悬停 + 100% 控制余量', W * 2.0, 2.0),
        ('过渡段峰值（45°）', W / math.cos(math.radians(45)), 1 / math.cos(math.radians(45))),
        ('巡航（D=%.0fN）' % D, D, D / W),
    ]
    for name, T, ratio in needs:
        print('   %-18s %14.2f %14.3f' % (name, T, ratio))
    print()
    print('   ★★★ 看最后两行：过渡段需要的推力【比悬停还大】！')
    print('       因为过渡时机翼还没托住飞机，而旋翼已经倾斜了。')
    print('   => 设计推力上限时，【决定因素是过渡段，不是悬停】。')

    sub('(b) 过设计的重量代价')
    print('   假设每个升力旋翼的「比推力」是 8 N 推力 / N 重量（含电机+桨+电调）：')
    print()
    print('   %-12s %14s %14s %14s' %
          ('T/W 设计值', '总推力(N)', '动力系统重(kg)', '占机重'))
    print('   ' + '─' * 58)
    for tw in (1.2, 1.5, 1.957, 2.5, 3.0):
        T = W * tw
        mass = T / 8.0 / G
        print('   %-12.2f %14.1f %14.2f %14.1f%%' %
              (tw, T, mass, mass / VEH['m'] * 100))
    print()
    print('   ★★ 推力从 T/W=2 加到 3，动力系统重量多了一大截 —— ')
    print('      而这些重量【在巡航时完全是负担】。')
    print('   ★ 所以 VTOL 设计的一个核心权衡是：')
    print('      「给多少悬停/过渡余量」 vs 「巡航能带多少载荷」')

    sub('(c) ★★★ 真正的设计点：过渡段的功率峰值')
    print('   前两节算的是【静态需求】，看不出问题。真正的设计点是动态的：')
    print('   过渡时机翼升力是【渐增】的，而推力的 cos 惩罚是【立刻】起作用的。')
    print()
    print('   => 过渡段存在一个【功率峰值】，它才是动力系统的选型依据。')
    print()

    TMARG = 0.85
    Tm = VEH['Tmax_rotor']

    def peak_power(rate, V_):
        r = _sim_transition(rate, V_, protect_corridor=True, thrust_margin=TMARG)
        Ts = [x[4] for x in r['trail'] if math.isfinite(x[4])]
        return max(Ts), r['t_end']

    print('   %-14s %14s %16s %16s' %
          ('倾转速率(°/s)', '峰值推力(N)', '相对悬停', '相对巡航'))
    print('   ' + '─' * 62)
    P_hover = W ** 1.5
    P_cruise = D ** 1.5
    for rate in (5.0, 12.0, 30.0, 60.0):
        Tpk, tend = peak_power(rate, VEH)
        Ppk = Tpk ** 1.5
        print('   %-14.1f %14.2f %16.3f %16.3f' %
              (rate, Tpk, Ppk / P_hover, Ppk / P_cruise))
    print()
    Tpk0, _ = peak_power(12.0, VEH)
    print('   参照点：')
    print('     悬停（T=W=%.0f N）   功率指数 = %.1f' % (W, P_hover))
    print('     巡航（T=D=%.0f N）   功率指数 = %.1f' % (D, P_cruise))
    print('     过渡峰值(T=%.0f N)    功率指数 = %.1f' % (Tpk0, Tpk0 ** 1.5))
    print()
    print('   ★★★ 关键读数：')
    print('     · 过渡峰值功率是【巡航的 %.0f 倍】——' % ((Tpk0 ** 1.5) / P_cruise))
    print('       所以电调/电池/电机都要按【过渡】选型，不是按巡航。')
    print('     · 但过渡只持续几秒，而巡航要飞几十分钟 =>')
    print('       ★ 电调要按峰值选，电池要按【能量】选 —— 两个不同的指标。')
    print()
    print('   ★ 这就是为什么 VTOL 的电调常常比同重量的多旋翼【更大】：')
    print('     多旋翼的峰值和巡航差别不大，VTOL 的差别是几倍。')

    sub('(d) 一笔总账：三种工况对动力系统的三种要求')
    print('   %-14s %16s %16s' % ('工况', '决定什么', '占比/特征'))
    print('   ' + '─' * 50)
    rows = [
        ('悬停', '桨盘面积（效率）', '持续，能量大头'),
        ('过渡', '★ 电调/电机峰值', '几秒，功率大头'),
        ('巡航', '★ 电池能量', '持续，能量大头'),
    ]
    for r in rows:
        print('   %-14s %16s %16s' % r)
    print()
    print('   ★★★ 三个工况要求【完全不同】的东西 —— ')
    print('       这就是 VTOL 动力系统设计比多旋翼难的根本原因：')
    print('       多旋翼只有「悬停」一个主导工况，VTOL 有三个。')
    print()
    print('   ★★ 回到实验 3(c) 那张表：')
    print('      「过渡速率」这个参数同时影响 峰值功率 和 过渡时间，')
    print('      而它们又是相互制约的 —— 这就是 VTOL 飞控里')
    print('      为什么有一整块单独的【过渡策略】逻辑。')
    return needs


# ═════════════════════════════════════════════════════════════════════

EXPS = {
    '1': ('倾转旋翼的推力矢量：cos 惩罚有多大', exp1_tilt_vector),
    '2': ('过渡走廊：机翼升力和旋翼推力谁接管', exp2_corridor),
    '3': ('过渡轨迹：从悬停加速到巡航的时间历程', exp3_trajectory),
    '4': ('尾坐式的控制权限交接', exp4_tailsitter),
    '5': ('三种 VTOL 架构的权衡', exp5_architectures),
    '6': ('推力预算：过设计的代价', exp6_thrust_budget),
}


def main():
    print()
    print('╔' + '═' * 72 + '╗')
    print('║' + '  lesson09h · VTOL 与倾转旋翼 · 配套仿真'.ljust(62) + '║')
    print('╚' + '═' * 72 + '╝')
    print()
    print('  算例：%s' % VEH['name'])
    print('    质量 %.1f kg   机翼面积 %.2f m^2   旋翼最大推力 %.0f N' %
          (VEH['m'], VEH['S'], VEH['Tmax_rotor']))
    print('    T/W = %.3f   失速速度 = %.1f m/s' %
          (VEH['Tmax_rotor'] / VEH['W'],
           math.sqrt(2 * VEH['W'] / (VEH['rho'] * VEH['S'] * VEH['CL_max']))))
    print()
    print('  实验列表：')
    for k in sorted(EXPS):
        print('    %s. %s' % (k, EXPS[k][0]))
    print()

    args = sys.argv[1:]
    if not args:
        args = sorted(EXPS)
    for a in args:
        if a in EXPS:
            EXPS[a][1]()
        else:
            print('  !! 未知实验编号：%s' % a)
    print()
    print('═' * 74)
    print('  完成。')
    print('═' * 74)
    print()


if __name__ == '__main__':
    main()
