#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lesson09k_delta_wing.py —— 03b 章 · 小展弦比三角翼（巡飞弹类）气动建模

    实验 1  几何：AR / MAC / 后掠角 —— 以及它为什么不是"普通固定翼"
    实验 2  升力线斜率：Helmbold 公式 vs 2π —— 低 AR 到底损失多少
    实验 3  涡升力 (Polhamus 吸力类比) —— 三角翼真正区别于普通机翼的地方
    实验 4  阻力与升阻比：两种阻力模型，差一倍
    实验 5  纵向导数：静稳定裕度决定生死
    实验 6  横航向导数：为什么偏航是弱轴
    实验 7  导出 ArduPilot SIM_Plane 的 JSON 模型
    实验 8  自检：量纲、符号、极限工况（**先跑这个**）

只用 numpy，不依赖 scipy。
运行： python lesson09k_delta_wing.py [1-8]

────────────────────────────────────────────────────────────────────────
⚠️ 公式可信度分级（本脚本对每条都标注，请勿混用）：
     [A] 已确立   —— 教科书/文献公式，可直接用
     [B] 工程估算 —— 合理近似，量级可信，绝对值需风洞/CFD 校核
     [C] 占位     —— 需实测替换，脚本给出的是保守猜测
   默认几何是一架"代表性"小展弦比三角翼巡飞弹的**估计值**，
   不是任何具体型号的实测数据 —— 用你自己的实测/CFD 结果替换。
"""

import sys
import json
import math
import numpy as np

# ═════════════════════════════════════════════════════════════════════
#  输入：几何与质量（改这里）
# ═════════════════════════════════════════════════════════════════════

class Geom:
    """小展弦比三角翼几何。单位：米、千克、秒。"""
    # ── 平面形状 ──
    b        = 2.50      # 翼展
    c_root   = 1.80      # 根弦
    c_tip    = 0.25      # 尖弦（纯三角翼填 0）
    # ── 质量特性 ──
    mass     = 180.0     # 起飞质量
    Ixx      = 12.0      # 滚转惯量
    Iyy      = 95.0      # 俯仰惯量
    Izz      = 105.0     # 偏航惯量
    # ── 重心与气动中心：以【平均气动弦 MAC 的比例】给出 ──
    #    ⚠️ 这是标准做法，且是本脚本最容易出错的地方：
    #       用"根弦比例"和"MAC 比例"混算会让静稳定裕度算错符号。
    #       x_cg_mac = 0.40 表示"重心位于 MAC 前缘之后 40% MAC 处"。
    x_cg_mac = 0.40      # 重心
    x_ac_mac = 0.48      # 气动中心（低 AR 三角翼因涡升力，AC 比常规 0.25 靠后）
    # ── 升降副翼弦向位置（根弦比例，靠后缘）──
    x_elevon  = 0.90     # 升降副翼压心（根弦比例）
    elevon_span_frac = 0.55   # 升降副翼占半展长的比例（0~1）
    # ── 翼尖小翼（唯一提供航向稳定的部件）──
    fin_height = 0.28
    fin_area_each = 0.075
    fin_arm  = 1.55      # 小翼压心到重心的力臂（沿 x）
    # ── 阻力 ──
    Cd0      = 0.020     # 零升阻力（含机身/起落架/桨）
    # ── 飞行条件 ──
    V_cruise = 50.0      # 巡航速度 m/s
    rho      = 1.225
    # ── 舵面限幅（弧度）──
    dmax     = math.radians(25.0)


# ═════════════════════════════════════════════════════════════════════
#  几何
# ═════════════════════════════════════════════════════════════════════

def geometry(g=Geom):
    """[A] 平面形状的严格计算。"""
    lam = g.c_tip / g.c_root                       # 尖削比
    S   = 0.5 * (g.c_root + g.c_tip) * g.b         # 梯形面积
    AR  = g.b ** 2 / S
    # 平均气动弦长 MAC（梯形通用式）
    mac = (2.0 / 3.0) * g.c_root * (1 + lam + lam ** 2) / (1 + lam)
    # 前缘后掠角：从根弦前缘到尖弦前缘
    #   ⚠️ 是 Δx/Δy 不是 Δy/Δx —— 初版写反了，算出 34.7°，
    #      而本几何真实值是 1.8/1.25 = 1.44 → 55.2°。三角翼明显该是大后掠。
    tan_le = (g.c_root - g.c_tip) / (g.b / 2.0) if g.c_root > g.c_tip else 0.0
    le_deg = math.degrees(math.atan(tan_le))
    # MAC 前缘在"根弦前缘坐标系"中的位置（梯形翼通用式，[A]）：
    #   x_mac_le = x_le_root + (b/6)·((1+2λ)/(1+λ))·tan(Λ_LE)
    #   纯矩形(λ=1)：tanΛ=0 → 0；纯三角(λ=0)：b/6·tanΛ = 0.6 = c_root/3
    mac_le = (g.b / 6.0) * ((1.0 + 2.0 * lam) / (1.0 + lam)) * tan_le
    return dict(lam=lam, S=S, AR=AR, mac=mac, le_deg=le_deg, mac_le=mac_le,
                c_root=g.c_root, c_tip=g.c_tip, b=g.b,
                # 绝对位置（从根弦前缘量起，米）——稳定性和条带积分都用这两个
                x_cg_abs=mac_le + g.x_cg_mac * mac,
                x_ac_abs=mac_le + g.x_ac_mac * mac)


# ═════════════════════════════════════════════════════════════════════
#  升力
# ═════════════════════════════════════════════════════════════════════

def lift_slope(AR, M=0.0):
    """
    [A] 低展弦比升力线斜率，Helmbold 方程 / DATCOM：

             2π·AR
    a = ─────────────────────        [每弧度]
         2 + sqrt(AR²·(1-M²) + 4)

    极限行为（这是它可信的原因）：
        AR → ∞ :  a → 2π            （二维机翼，正确）
        AR → 0 :  a → π·AR/2        （细长体理论，正确）
    两个极限都对，所以中间值可信。
    """
    beta2 = max(1.0 - M * M, 1e-6)
    return 2.0 * math.pi * AR / (2.0 + math.sqrt(AR * AR * beta2 + 4.0))


def polhamus_CL(alpha, AR, a):
    """
    [A] Polhamus 吸力类比 (NASA TN D-3767, 1966)：

        C_N = K_p·sinα·cos²α + K_v·sin²α·cosα
        C_L = C_N·cosα
        K_p = a  （势流升力线斜率）
        K_v = π·AR / (2·sqrt(AR²+4))   （涡升力系数）

    这是三角翼区别于普通机翼的核心：
    尖前缘在大迎角卷起前缘涡，涡心低压额外产生升力，
    使失速推迟到 30°~40°（普通翼型 ~15°）。
    """
    Kp = a
    Kv = math.pi * AR / (2.0 * math.sqrt(AR * AR + 4.0))
    sa, ca = math.sin(alpha), math.cos(alpha)
    CN = Kp * sa * ca * ca + Kv * sa * sa * ca
    return CN * ca, CN


def alpha_stall_delta(AR):
    """
    [B] 三角翼失速迎角估算。
    低 AR 三角翼的失速迎角随 AR 减小而增大（涡升力推迟失速）。
    经验拟合：AR=1 → ~40°, AR=2 → ~32°, AR=3 → ~26°, AR=6 → ~18°
    普通矩形翼约 15°。
    """
    return math.radians(15.0 + 25.0 / (1.0 + 0.55 * AR))


# ═════════════════════════════════════════════════════════════════════
#  阻力
# ═════════════════════════════════════════════════════════════════════

def drag_polar_simple(CL, AR, Cd0, oswald=0.80):
    """[A] 经典极曲线：寄生 + 诱导。假设前缘有吸力（圆前缘）。"""
    return Cd0 + CL * CL / (math.pi * AR * oswald)


def drag_polar_sharp_le(CL, alpha, Cd0):
    """
    [B] 尖前缘极曲线（无前缘吸力）：
        C_D = C_D0 + C_L·tanα

    三角翼的涡升力【不是免费的】—— 它来自前缘分离，
    分离后前缘吸力消失，诱导阻力按 C_L·tanα 计（远大于 C_L²/(πARe)）。
    两种模型在同一点可以差一倍，这是选型时最容易被忽略的事。
    """
    return Cd0 + CL * math.tan(alpha)


# ═════════════════════════════════════════════════════════════════════
#  纵向导数
# ═════════════════════════════════════════════════════════════════════

def cm_alpha(a, SM):
    """
    [A] 俯仰静稳定性导数：
        C_m,α = -a · SM        SM = 静稳定裕度（占 MAC 的比例）

    对无尾布局这是【生死线】：
        SM > 0  静稳定
        SM = 0  中立稳定（不可飞，无配平余度）
        SM < 0  静不稳定（必须靠增稳，飞控失效即坠机）
    常规带尾翼飞机 SM 取 8%~15%；
    无尾三角翼通常只有 3%~8%，且气动中心随迎角前移（涡升力），
    低速大迎角时 SM 会进一步减小 —— 这就是俯仰上仰的根源。
    """
    return -a * SM


def cm_q_strip(g, geo, a, n_strip=400):
    """
    [A] 俯仰阻尼导数，展向条带积分（wing-alone）：

        C_m,q = -2·a·I / (S·c̄²),    I = ∬ (x - x_cg)² dS

    推导：俯仰角速度 q 在弦向站位 x 处附加迎角 Δα = q(x-x_cg)/V，
    该处升力增量对重心取矩后对面积积分，再按 q·c̄/(2V) 无量纲化。

    对无尾布局的物理含义：I 只有机翼自身的贡献，
    没有平尾那段长力臂 —— 所以 C_m,q 比常规飞机小得多，
    俯仰阻尼弱、更容易振荡。这是模型必须反映的。
    """
    b, c_root, c_tip = geo['b'], geo['c_root'], geo['c_tip']
    # 坐标：x 从根弦前缘量起，向右为后
    x_cg = geo['x_cg_abs']
    I = 0.0
    S_check = 0.0
    dy = (b / 2.0) / n_strip
    for i in range(n_strip):
        y = (i + 0.5) * dy                     # 0 ~ b/2
        eta = 2.0 * y / b                      # 0~1
        c_y = c_root + (c_tip - c_root) * eta  # 该展向站位的弦长
        x_le = (c_root - c_tip) * eta          # 前缘位置（后掠）
        x_te = x_le + c_y                      # 后缘位置
        # 沿弦向再积分（梯形法，取 8 点够用）
        n_c = 8
        dx = c_y / n_c
        for k in range(n_c):
            x = x_le + (k + 0.5) * dx
            I += (x - x_cg) ** 2 * dx * dy * 2.0   # ×2 因为上下对称两半
        S_check += c_y * dy * 2.0
    return -2.0 * a * I / (geo['S'] * geo['mac'] ** 2), I, S_check


def tau_flap(cf_c):
    """
    [B] 后缘襟翼效率因子 τ 的工程拟合。
    cf_c = 襟翼弦长 / 当地弦长。τ 随 cf/c 增大而增大但饱和。
    经典曲线（Abbott & von Doenhoff）拟合：cf/c=0.2→0.35, 0.3→0.45,
    0.4→0.53, 0.5→0.60。低 AR 三角翼取下限。
    """
    cf_c = min(max(cf_c, 0.0), 0.6)
    return 0.90 * cf_c ** 0.72


def cm_deltae(g, geo, a, cf_c=0.25):
    """
    [B] 升降副翼俯仰操纵导数：

        C_m,δe = -a·τ·(x_flap - x_cg)/c̄

    符号约定与 SIM_Plane 一致（δe 正 = 后缘下偏）。
    ⚠️ 无尾布局的俯仰权限通常只有常规飞机的 1/3~1/2
       —— 没有平尾的长力臂。所以 PITCH 限幅和失速改出都要重新评估。
    """
    tau = tau_flap(cf_c)
    x_cg = geo['x_cg_abs']
    x_flap = g.x_elevon * geo['c_root']
    return -a * tau * (x_flap - x_cg) / geo['mac'], tau


# ═════════════════════════════════════════════════════════════════════
#  横航向导数
# ═════════════════════════════════════════════════════════════════════

def cn_beta_fins(g, geo, V):
    """
    [B] 航向静稳定导数，来自翼尖小翼：

        C_n,β = -q·S_fin·l_fin·a_fin / (q·S·b)   （每弧度）

    小翼效率 a_fin 用 [A] 的 Helmbold 公式（视为小展弦比面）。
    ⚠️ 这是本布局最弱的一环：没有垂尾，全部航向稳定性靠小翼。
       C_n,β 偏小 → 荷兰滚阻尼不足、侧滑角大、转弯协调性差。
       ArduPilot/PX4 都默认有方向舵，无舵机型要特别验证这一项。
    """
    AR_fin = g.fin_height ** 2 / g.fin_area_each
    a_fin = lift_slope(AR_fin)
    Cn_b = -2.0 * g.fin_area_each * g.fin_arm * a_fin / (geo['S'] * geo['b'])
    return Cn_b, a_fin, AR_fin


def cl_p_roll_damping(geo, a):
    """
    [A] 滚转阻尼，展向条带积分：

        C_l,p = -2·a·I / (S·b²),    I = ∫ y²·c(y) dy   （全展长）

    推导：滚转角速度 p 在展向站位 y 处附加迎角 Δα = -p·y/V，
    该处升力增量对 x 轴取矩，沿展向积分后按 p·b/(2V) 无量纲化。
    """
    b, c_root, c_tip = geo['b'], geo['c_root'], geo['c_tip']
    n = 400
    dy = (b / 2.0) / n
    I = 0.0
    for i in range(n):
        y = (i + 0.5) * dy
        eta = 2.0 * y / b
        c_y = c_root + (c_tip - c_root) * eta
        I += y * y * c_y * dy * 2.0          # ×2：左右两半对称
    return -2.0 * a * I / (geo['S'] * b * b)


def cl_deltaa(g, geo, a, cf_c=0.25):
    """
    [B] 升降副翼滚转操纵导数（差动）：
        C_l,δa = a·τ·(y_cp/b)·(S_flap/S)
    y_cp 取副翼展向压心，约 0.65 倍副翼外端。
    """
    tau = tau_flap(cf_c)
    y_cp = 0.65 * (g.elevon_span_frac * geo['b'] / 2.0)
    S_flap = 2.0 * (g.elevon_span_frac * geo['b'] / 2.0) * 0.20 * geo['mac']
    return a * tau * (y_cp / geo['b']) * (S_flap / geo['S'])


def cn_deltaa(g, geo, a, cf_c=0.25):
    """
    [B] 差动舵的偏航导数（不利偏航 adverse yaw）：

        C_n,δa = -a·τ·(y_cp·x_arm/S/b) ...

    负号表示副翼下偏一侧阻力增大，机头偏向【下偏】那侧，与期望转弯反向。
    无垂尾时没有方向舵去配平它 —— 转弯时会持续带侧滑。
    """
    tau = tau_flap(cf_c)
    y_cp = 0.65 * (g.elevon_span_frac * geo['b'] / 2.0)
    x_cg = geo['x_cg_abs']
    x_flap = g.x_elevon * geo['c_root']
    S_flap = 2.0 * (g.elevon_span_frac * geo['b'] / 2.0) * 0.20 * geo['mac']
    return -a * tau * (S_flap / geo['S']) * (y_cp / geo['b']) * ((x_flap - x_cg) / geo['mac']) * 0.5


def cn_r_damping(geo, g, a):
    """[A] 航向阻尼，主要来自小翼力臂：C_n,r ≈ -2·C_n,β·(l_fin/b)"""
    return 0.0  # 由 cn_beta_fins 的结果在 compute_all 里推算


# ═════════════════════════════════════════════════════════════════════
#  汇总
# ═════════════════════════════════════════════════════════════════════

def compute_all(g=Geom):
    geo = geometry(g)
    AR, S, mac = geo['AR'], geo['S'], geo['mac']
    a = lift_slope(AR)
    a_2pi = 2.0 * math.pi
    a_stall = alpha_stall_delta(AR)

    # 静稳定裕度：全部用 MAC 比例算 —— 这是唯一不会算错符号的写法
    #   SM = x_ac_mac - x_cg_mac
    # [B] x_ac_mac = 0.48 是低 AR 三角翼的估计值（常规翼型 0.25），
    #     涡升力会把气动中心显著后移，且随迎角变化 —— 务必 CFD/风洞替换
    x_ac = geo['x_ac_abs']
    SM = g.x_ac_mac - g.x_cg_mac

    # ── 巡航配平：先算，因为下面的 C_l,β 要用到 C_L ──
    q_dyn = 0.5 * g.rho * g.V_cruise ** 2
    W = g.mass * 9.80665
    CL_cruise = W / (q_dyn * S)
    alpha_trim = CL_cruise / a

    Cm_a = cm_alpha(a, SM)
    Cm_q, I_int, S_chk = cm_q_strip(g, geo, a)
    Cm_de, tau = cm_deltae(g, geo, a)
    Cn_b, a_fin, AR_fin = cn_beta_fins(g, geo, g.V_cruise)
    # 航向阻尼：偏航角速度 r 在小翼处产生侧滑 Δβ = r·l_fin/V，
    # 小翼侧力对该侧滑的反抗构成阻尼矩。推导得
    #     C_n,r = C_n,β · (l_fin / b)
    # 因 C_n,β < 0，结果必为【负】(阻尼反对偏航)。
    # ⚠️ 初版写成 -2·C_n,β·(l/b)·0.5，多一个负号 → 得到 +0.035，
    #    那会让偏航轴在仿真里【发散】。自检已加符号断言。
    Cn_r = Cn_b * (g.fin_arm / geo['b'])
    Cl_p = cl_p_roll_damping(geo, a)
    # 后掠翼的二面角效应（上反效应）：C_l,β|sweep ≈ -C_L·tan(Λ)/(π·AR)
    # [B] 只含后掠项；翼尖小翼的贡献未建模，符号相反且量级不小 —— 需 CFD 定。
    # ⚠️ 初版写 +0.25·(−C_n,β) 得到【正】值，符号错（应稳定侧倾 → 负）。
    Cl_b = -CL_cruise * math.tan(math.radians(geo['le_deg'])) / (math.pi * geo['AR'])
    Cl_da = cl_deltaa(g, geo, a)
    Cn_da = cn_deltaa(g, geo, a)

    CD_simple = drag_polar_simple(CL_cruise, AR, g.Cd0)
    CD_sharp = drag_polar_sharp_le(CL_cruise, alpha_trim, g.Cd0)

    return dict(
        geo=geo, a=a, a_2pi=a_2pi, a_stall=a_stall, alpha_trim=alpha_trim,
        CL_cruise=CL_cruise, SM=SM, x_ac=x_ac,
        Cm_a=Cm_a, Cm_q=Cm_q, Cm_de=Cm_de, tau=tau,
        Cn_b=Cn_b, Cn_r=Cn_r, Cl_p=Cl_p, Cl_b=Cl_b, Cl_da=Cl_da, Cn_da=Cn_da,
        a_fin=a_fin, AR_fin=AR_fin,
        CD_simple=CD_simple, CD_sharp=CD_sharp,
        LD_simple=CL_cruise / CD_simple, LD_sharp=CL_cruise / CD_sharp,
        I_int=I_int, S_chk=S_chk,
    )


# ═════════════════════════════════════════════════════════════════════
#  输出
# ═════════════════════════════════════════════════════════════════════

def banner(t):
    print()
    print('═' * 74)
    print('  ' + t)
    print('═' * 74)


def sub(t):
    print()
    print('── ' + t)
    print('─' * 74)


# ═════════════════════════════════════════════════════════════════════
#  实验
# ═════════════════════════════════════════════════════════════════════

def exp1_geometry():
    banner('实验 1 · 几何：它为什么不是"普通固定翼"')
    r = compute_all()
    geo = r['geo']
    print('''
   平面形状：
      翼展 b        = %.2f m        根弦 = %.2f m   尖弦 = %.2f m
      面积 S        = %.3f m²       尖削比 λ = %.2f
      展弦比 AR     = %.2f
      平均气动弦 c̄  = %.3f m
      前缘后掠角    = %.1f°''' % (geo['b'], geo['c_root'], geo['c_tip'],
                                  geo['S'], geo['lam'], geo['AR'],
                                  geo['mac'], geo['le_deg']))
    print('''
   ▶ 关键：AR = %.2f。

     常规轻型飞机 AR ≈ 6~9，滑翔机 15~25，而这里是 %.2f。
     低 AR 的代价（下面几节逐条量化）：
       · 升力线斜率低  → 同样迎角产生更少升力，配平迎角大
       · 诱导阻力大    → 升阻比低，滑翔距离短
       · 前缘涡        → 大迎角有额外升力（唯一的好处）
       · 无平尾/无垂尾 → 两个控制轴都变弱
''' % (geo['AR'], geo['AR']))
    print('   ▶ 前缘后掠 %.1f° 是本布局的签名特征 —— 它正是涡升力的来源。'
          % geo['le_deg'])


def exp2_lift_slope():
    banner('实验 2 · 升力线斜率：低 AR 到底损失多少')
    r = compute_all()
    AR = r['geo']['AR']
    print('''
   Helmbold 方程 [A]：

              2π·AR
      a = ─────────────────────       [每弧度]
           2 + sqrt(AR²·(1-M²) + 4)

   它可信是因为两个极限都对：
       AR → ∞ :  a → 2π        （二维机翼）
       AR → 0 :  a → π·AR/2    （细长体理论）
''')
    header = '     AR     a [1/rad]   a/a_2π     a [1/deg]'
    print(header)
    print('   ' + '─' * (len(header) - 3))
    for ar in [1.0, 2.0, 3.0, 4.0, 6.0, 9.0, 15.0, 100.0]:
        av = lift_slope(ar)
        print('   %6.1f   %8.3f   %7.3f    %8.5f'
              % (ar, av, av / (2 * math.pi), av * math.pi / 180.0))
    print()
    print('   ▶ 本机 AR=%.2f → a = %.3f /rad，仅为二维值的 %.1f%%'
          % (AR, r['a'], 100 * r['a'] / r['a_2pi']))
    print('   ▶ 含义：配平迎角比常规机翼大得多。')
    print('     ArduPilot 的 PTCH_TRIM_DEG 和失速保护都要按实际 a 重算，')
    print('     照抄常规固定翼参数会导致低速段升力不足。')


def exp3_vortex_lift():
    banner('实验 3 · 涡升力：三角翼真正区别于普通机翼的地方')
    r = compute_all()
    AR, a = r['geo']['AR'], r['a']
    print('''
   Polhamus 吸力类比 [A] (NASA TN D-3767, 1966)：

      C_N = K_p·sinα·cos²α + K_v·sin²α·cosα
      K_p = a                              ← 势流部分
      K_v = π·AR / (2·sqrt(AR²+4))         ← 涡升力部分

   尖前缘在大迎角卷起前缘涡，涡心低压产生额外升力。
   结果是失速大幅推迟 —— 这就是三角翼存在的理由。
''')
    print('     α[°]     C_L(纯线性)   C_L(含涡)     增益       C_L 实测参考区间')
    print('   ' + '─' * 68)
    for adeg in [0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40]:
        al = math.radians(adeg)
        cl_lin = a * al
        cl_v, _ = polhamus_CL(al, AR, a)
        gain = (cl_v / cl_lin - 1.0) * 100 if cl_lin > 1e-9 else 0.0
        print('   %6.0f   %10.3f   %10.3f   %+7.1f%%' % (adeg, cl_lin, cl_v, gain))
    a_stall = math.degrees(r['a_stall'])
    Kv = math.pi * AR / (2.0 * math.sqrt(AR * AR + 4.0))
    print()
    print('   ▶ 本机估算失速迎角 α_stall ≈ %.0f°（普通翼型约 15°）' % a_stall)
    print()
    print('   ▶ 涡升力项占总升力的比例（Polhamus 分解，实测）：')
    for adeg in (10, 15, 20, 25, 30, 35, 40):
        al = math.radians(adeg)
        s, c = math.sin(al), math.cos(al)
        pot = a * s * c ** 3
        vor = Kv * s * s * c * c
        print('        α=%-3d   势流 %.4f   涡升力 %.4f   涡占 %4.1f%%'
              % (adeg, pot, vor, 100 * vor / (pot + vor)))
    print('''
   ⚠️ 注意量级：涡升力【不是】升力的主要来源。
      它在 α=30° 时也只占约 19%，峰值附近（33°）约 21%。
      它的真正价值不在"提供多少升力"，而在【把失速推迟到 33° 附近】——
      常规翼型 15° 就失速了，本机可以飞到 33° 才到 $C_L$ 峰值，
      可用升力系数因此从 ~1.0 提到 ~1.2。
      把涡升力说成"能大幅增升"是常见的夸大。''')
    print('''
   ⚠️⚠️ 对建模的直接后果 —— 这是本章最重要的一条：

      ArduPilot 的 SIM_Plane 气动模型是【纯线性 + 单一 alpha_stall】
      （源码注释自称 "very simple ... Not aerodynamically accurate"）。
      它【无法表达】上面这张表里的涡升力。

      所以：拿 SIM_Plane 做 SITL，能验证导航、混控、参数逻辑，
      但大迎角段的气动行为是【错的】。低速/大迎角故障（失速改出、
      发射段、动力失效后滑翔）的仿真结论【不可信】。
      这些工况要么用 JSBSim 写分段气动表，要么 CFD 单独算。
''')


def exp4_drag():
    banner('实验 4 · 阻力与升阻比：两种模型差一倍')
    r = compute_all()
    geo, AR = r['geo'], r['geo']['AR']
    print('''
   两种阻力模型 —— 差别不在精度，在【物理假设】：

     [A] 圆前缘（有吸力）：C_D = C_D0 + C_L²/(π·AR·e)
     [B] 尖前缘（无吸力）：C_D = C_D0 + C_L·tanα

   涡升力不是免费的：它来自前缘分离，
   分离后前缘吸力消失，诱导阻力按 C_L·tanα 计。
''')
    print('     模型                              C_D        L/D    滑翔比')
    print('   ' + '─' * 66)
    print('   圆前缘 (经典极曲线)            %9.4f   %6.2f    %5.1f:1'
          % (r['CD_simple'], r['LD_simple'], r['LD_simple']))
    print('   尖前缘 (无吸力)                %9.4f   %6.2f    %5.1f:1'
          % (r['CD_sharp'], r['LD_sharp'], r['LD_sharp']))
    print()
    print('   ▶ 巡航 CL = %.3f，配平迎角 %.2f°'
          % (r['CL_cruise'], math.degrees(r['alpha_trim'])))
    print('   ▶ 升阻比落在 %.1f~%.1f 区间。'
          % (min(r['LD_simple'], r['LD_sharp']), max(r['LD_simple'], r['LD_sharp'])))
    print('''
   ▶ 工程含义：这个 L/D 意味着动力失效后滑翔距离很短。
     ArduPilot 的 TECS 参数（TECS_CLMB_MAX / TECS_SINK_MAX /
     TECS_TIME_CONST）必须按实际 L/D 重调，
     默认值是按 L/D≈10 的常规机型给的，套用会导致爬升/下滑不可控。
''')


def exp5_longitudinal():
    banner('实验 5 · 纵向导数：静稳定裕度决定生死')
    r = compute_all()
    geo = r['geo']
    print('''
   两个位置都以【平均气动弦 MAC 的比例】给出（标准做法）：
       重心     x_cg = %.2f MAC   → 绝对位置 %.3f m（从根弦前缘量起）
       气动中心 x_ac = %.2f MAC   → 绝对位置 %.3f m
               （MAC 前缘在 %.3f m 处）
''' % (Geom.x_cg_mac, geo['x_cg_abs'], Geom.x_ac_mac, geo['x_ac_abs'], geo['mac_le']))
    print('   ▶ 静稳定裕度 SM = x_ac − x_cg = %+.4f  (%.1f%% MAC)'
          % (r['SM'], 100 * r['SM']))
    print('   ▶ C_m,α = -a·SM = %+.4f   %s'
          % (r['Cm_a'], '✓ 静稳定' if r['SM'] > 0 else '✗ 静不稳定！'))
    print()
    print('   俯仰阻尼 C_m,q = %+.3f   （条带积分，I = %.5f m⁴）'
          % (r['Cm_q'], r['I_int']))
    print('   升降副翼 C_m,δe = %+.4f   （τ = %.3f）' % (r['Cm_de'], r['tau']))
    print('''
   ▶ 两个必须注意的对比（对常规带尾翼飞机）：

     1) C_m,q 明显偏小 —— 没有平尾那段长力臂，
        俯仰阻尼只有机翼自身贡献。后果：俯仰容易振荡，
        ArduPilot 的 PTCH_RATE_D/I 需要重新整定，不能照抄。
        （对照：ArduPilot 默认模型 skywalker_2013 的 C_m,q = -20，
          那是有平尾的常规布局。）

     2) C_m,δe 俯仰权限小 —— 同理由。
        失速改出、大俯仰指令可能【打不满】。
        必须在 SITL 里先验证：给满 δe 能否在俯仰极限内拉起。

   ▶ 无尾布局的 CG 范围极窄：SM 变化 ±3% MAC 就可能从静稳定
     变成静不稳定。所以【重心位置的准确性】比气动系数精度更重要 ——
     装机上称重测重心，别用估算值。
''')


def exp6_lateral():
    banner('实验 6 · 横航向导数：为什么偏航是弱轴')
    r = compute_all()
    print('''
   全部航向稳定性来自翼尖小翼 —— 没有垂尾。
   （小翼高度 %.2f m，单侧面积 %.4f m²，力臂 %.2f m）
''' % (Geom.fin_height, Geom.fin_area_each, Geom.fin_arm))
    print('   小翼等效 AR = %.2f  →  小翼升力线斜率 a_fin = %.3f /rad'
          % (r['AR_fin'], r['a_fin']))
    print()
    print('   C_n,β  = %+.5f /rad     ← 航向静稳定（负=稳定）' % r['Cn_b'])
    print('   C_n,r  = %+.5f /rad     ← 航向阻尼' % r['Cn_r'])
    print('   C_l,β  = (上反/后掠效应，本脚本未建模)' )
    print('   C_l,p  = %+.5f /rad     ← 滚转阻尼' % r['Cl_p'])
    print('   C_l,δa = %+.5f /rad     ← 副翼滚转操纵' % r['Cl_da'])
    print('   C_n,δa = %+.5f /rad     ← 差动舵不利偏航' % r['Cn_da'])
    print('''
   ▶ 对比：常规带垂尾飞机 C_n,β 约 -0.08 ~ -0.15。
     本机 %.4f —— 约为常规值的 %.0f%%~%.0f%%（偏弱，但没到量级差距）。
''' % (r['Cn_b'], 100 * abs(r['Cn_b']) / 0.15, 100 * abs(r['Cn_b']) / 0.08))
    print('''   ▶ 后果链条（这是本项目最大的风险点）：

       C_n,β 小
         → 侧滑后回正力矩弱
         → 荷兰滚模态频率低、阻尼差
         → 转弯时侧滑角大，且【无法用方向舵配平】（没有方向舵）
         → 只能靠差动升降副翼（C_n,δa = %.4f，效率低且耦合滚转）
         → 协调转弯做不到，横侧向控制精度受限

   ▶ 建模时必须做的一件事：把 C_n,β 当作【不确定度最大的参数】，
     在 SITL 里做 ±50%% 的扫掠，看控制律还稳不稳。
     气动估算在这一项上误差最大 —— 小翼处于机翼自身的下洗流里，
     实际效率可能只有孤立翼估算值的 50%%~70%%。
''' % r['Cn_da'])


def exp7_export_json(path=None):
    banner('实验 7 · 导出 ArduPilot SIM_Plane 的 JSON 模型')
    r = compute_all()
    geo = r['geo']
    g = Geom

    model = {
        # ── 几何 ──
        "s": round(geo['S'], 4),
        "b": round(geo['b'], 4),
        "c": round(geo['mac'], 4),
        # ── 升力 ──
        "c_lift_0": 0.0,
        "c_lift_a": round(r['a'], 4),
        "c_lift_q": 0.0,
        "c_lift_deltae": 0.0,      # 由 c_m_deltae 体现俯仰，升力增量并入
        "mcoeff": 50.0,
        "oswald": 0.80,
        "alpha_stall": round(r['a_stall'], 4),
        # ── 阻力 ──
        "c_drag_p": round(g.Cd0, 5),
        "c_drag_q": 0.0,
        "c_drag_deltae": 0.0,
        # ── 侧力 ──
        "c_y_0": 0.0,
        "c_y_b": round(r['Cn_b'] * 0.6, 5),   # 侧力近似与 C_n,β 同源
        "c_y_p": 0.0,
        "c_y_r": 0.0,
        "c_y_deltaa": 0.0,
        "c_y_deltar": 0.0,
        # ── 滚转 ──
        "c_l_0": 0.0,
        "c_l_p": round(r['Cl_p'], 5),
        "c_l_b": round(r['Cl_b'], 5),           # 后掠上反效应，[B]
        "c_l_r": round(r['Cl_p'] * 0.12, 5),    # [C] 占位
        "c_l_deltaa": round(r['Cl_da'], 5),
        "c_l_deltar": 0.0,
        # ── 俯仰 ──
        "c_m_0": 0.0,
        "c_m_a": round(r['Cm_a'], 5),
        "c_m_q": round(r['Cm_q'], 4),
        "c_m_deltae": round(r['Cm_de'], 5),
        # ── 偏航 ──
        "c_n_0": 0.0,
        "c_n_b": round(r['Cn_b'], 5),
        "c_n_p": round(-r['Cn_b'] * 0.1, 5),    # [C] 占位
        "c_n_r": round(r['Cn_r'], 5),
        "c_n_deltaa": round(r['Cn_da'], 5),
        "c_n_deltar": 0.0,
        # ── 舵限 ──
        "deltaa_max": round(g.dmax, 4),
        "deltae_max": round(g.dmax, 4),
        "deltar_max": round(g.dmax, 4),
        # ── 重心偏差（SIM_Plane 用它做配平调整）──
        "CGOffset": [0.0, 0.0, 0.0],
    }

    txt = json.dumps(model, indent=4, ensure_ascii=False)
    print('   生成的模型（对照 tools/autotest/models/skywalker_2013.json 的字段）:')
    print()
    for line in txt.split('\n'):
        print('     ' + line)
    print()
    print('   ⚠️ 三个字段必须你自己确认，不要用脚本猜测值：')
    print('      · c_m_0   —— 巡航配平需要的零升力矩，取决于实际安装角/扭转')
    print('      · CGOffset—— 直接决定配平，本脚本填 0 表示"重心就在 x_cg 处"')
    print('      · alpha_stall —— 估算值 %.1f°，真值靠风洞/CFD'
          % math.degrees(r['a_stall']))
    if path:
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(model, f, indent=4, ensure_ascii=False)
        print()
        print('   ✓ 已写入: %s' % path)
    return model


def exp8_selfcheck():
    """
    自检：证明这把"量具"本身在工作。
    按项目惯例 —— 验证脚本比被测对象更易错，必须自证。
    """
    banner('实验 8 · 自检：量具是否在工作')
    ok = 0
    fail = 0

    def chk(cond, msg):
        nonlocal ok, fail
        if cond:
            ok += 1
            print('   ✓ %s' % msg)
        else:
            fail += 1
            print('   ✗ %s   <<< 失败' % msg)

    geo = geometry()
    a = lift_slope(geo['AR'])

    sub('A. 几何量纲与恒等式')
    chk(abs(geo['S'] - 0.5 * (Geom.c_root + Geom.c_tip) * Geom.b) < 1e-12,
        '面积 = 梯形公式 (S=%.4f)' % geo['S'])
    chk(geo['AR'] > 1.0, 'AR = %.2f > 1 (低展弦比前提成立)' % geo['AR'])
    chk(Geom.c_root * 0.3 < geo['mac'] < Geom.c_root,
        'c̄ = %.3f 落在 (0.3·root, root) 内' % geo['mac'])
    chk(abs(geo['mac'] - Geom.c_root) < 1e-9 if abs(Geom.c_tip - Geom.c_root) < 1e-9
        else True, '等弦时 c̄ = root (退化检查)')
    # 纯矩形翼 MAC 必须等于 c_root
    class Rect(Geom):
        c_tip = Geom.c_root
    chk(abs(geometry(Rect)['mac'] - Geom.c_root) < 1e-9,
        '矩形翼 (λ=1) 的 c̄ 精确等于根弦')
    # MAC 前缘位置的两个极限（参考系正确性的根据）
    class Delta(Geom):
        c_tip = 0.0
    chk(abs(geometry(Delta)['mac_le'] - Geom.c_root / 3.0) < 1e-9,
        '纯三角 (λ=0) MAC 前缘 = 根弦/3 = %.4f' % (Geom.c_root / 3.0))
    chk(abs(geometry(Rect)['mac_le']) < 1e-9,
        '矩形 (λ=1) MAC 前缘 = 0 (无后掠，正确)')
    # ★ 闭式 vs 数值积分交叉验证 —— 前缘后掠角写反这类错误专用防线。
    #   直接对平面形状积分 x_mac_le = ∫x_le·c dy / ∫c dy，与闭式比较。
    def mac_le_numeric(g=Geom, n=200000):
        b, c_r, c_t = g.b, g.c_root, g.c_tip
        dy = (b / 2.0) / n
        num = den = 0.0
        for i in range(n):
            y = (i + 0.5) * dy
            eta = 2.0 * y / b
            c_y = c_r + (c_t - c_r) * eta
            x_le = (c_r - c_t) * eta
            num += x_le * c_y * dy
            den += c_y * dy
        return num / den
    for cls, nm in ((Geom, '本机'), (Rect, '矩形'), (Delta, '纯三角')):
        cl = geometry(cls)['mac_le']
        cn = mac_le_numeric(cls)
        chk(abs(cl - cn) < 1e-6,
            '%s: 闭式 MAC前缘 %.6f 与数值积分 %.6f 一致' % (nm, cl, cn))
    chk(30.0 < geometry()['le_deg'] < 70.0,
        '前缘后掠角 %.1f° 落在三角翼合理区间 [30°, 70°] (防 tan/cot 写反)'
        % geometry()['le_deg'])
    chk(geo['x_cg_abs'] > geo['mac_le'],
        '重心 (%.3f) 位于 MAC 前缘 (%.3f) 之后' % (geo['x_cg_abs'], geo['mac_le']))

    sub('B. 升力线斜率的极限行为（公式可信度的根据）')
    a_inf = lift_slope(1e6)
    chk(abs(a_inf - 2 * math.pi) < 1e-4,
        'AR→∞ 时 a→2π = %.4f (实得 %.4f)' % (2 * math.pi, a_inf))
    a_small = lift_slope(1e-4)
    chk(abs(a_small - math.pi * 1e-4 / 2.0) < 1e-9,
        'AR→0 时 a→π·AR/2 (细长体理论)')
    chk(lift_slope(2.0) < lift_slope(6.0) < lift_slope(20.0),
        'a 随 AR 单调递增')
    chk(0.0 < a < 2 * math.pi, '本机 a = %.3f 落在 (0, 2π) 内' % a)

    sub('C. Polhamus 涡升力的硬性性质')
    r = compute_all()
    AR = geo['AR']
    # 性质 1：α=0 时涡升力项必须为 0
    cl0, cn0 = polhamus_CL(0.0, AR, a)
    chk(abs(cl0) < 1e-12, 'α=0 时 C_L=0 (无涡升力残留)')

    # 性质 2：小迎角趋近线性，且【收敛阶数可解析预测】。
    #   展开 C_L = K_p·sinα·cos³α + K_v·sin²α·cos²α 到最低阶：
    #       C_L ≈ K_p·α + K_v·α² − (K_p/2)·α³
    #   减掉线性项 K_p·α 后【除以 K_p·α】，相对偏差 ≈ (K_v/K_p)·α
    #   —— 是 O(α) 而不是 O(α²)（初版误判成二阶，断言了错误的阈值）。
    #   因此正确的判据是"α 减半 → 偏差减半"，而不是某个固定绝对值。
    errs = []
    for adeg in [2.0, 1.0, 0.5, 0.25]:
        al = math.radians(adeg)
        clv, _ = polhamus_CL(al, AR, a)
        errs.append(abs(clv - a * al) / (a * al))
    ratios = [errs[i + 1] / errs[i] for i in range(len(errs) - 1)]
    chk(all(0.40 < rt < 0.60 for rt in ratios),
        'α 每减半、相对偏差随之减半 → O(α) 收敛 (比率 %s)'
        % ', '.join('%.3f' % rt for rt in ratios))
    # 再用解析斜率交叉验证：偏差/α 应趋于常数 K_v/K_p
    Kv_over_Kp = (math.pi * AR / (2 * math.sqrt(AR * AR + 4))) / a
    slope = errs[-1] / math.radians(0.25)
    chk(abs(slope - Kv_over_Kp) / Kv_over_Kp < 0.05,
        '偏差/α = %.4f 与解析值 K_v/K_p = %.4f 相符 (差 %.1f%%)'
        % (slope, Kv_over_Kp, 100 * abs(slope - Kv_over_Kp) / Kv_over_Kp))

    # 性质 3：C_L(α) 必须有【极大值】—— 这是与线性模型的本质区别。
    #   线性外推 a·α 无上界；Polhamus 因 cos³α 衰减会饱和并下降。
    alphas = np.radians(np.arange(0.5, 60.0, 0.05))
    cls = np.array([polhamus_CL(al, AR, a)[0] for al in alphas])
    i_pk = int(np.argmax(cls))
    a_pk = math.degrees(alphas[i_pk])
    chk(5.0 < a_pk < 50.0 and cls[i_pk] > 0.5,
        'C_L(α) 在 α ≈ %.1f° 处取极大值 C_L = %.3f (线性模型无极大值)' % (a_pk, cls[i_pk]))
    chk(cls[i_pk + 5] < cls[i_pk], '极大值之后 C_L 确实下降 (非数值噪声)')

    # 性质 4：涡升力增益【由正转负】—— 低迎角 Polhamus 高于线性外推，
    #   高迎角因 cos³α 衰减反而低于线性外推。交叉点随 AR 变化，本机应在 10~20°。
    #   （初版断言"4~40° 全为正贡献"是错的：实际只有低迎角段成立）
    prev, cross = None, None
    # 网格 0.05°：0.25° 网格会把交叉点报成 13.75°，细网格是 13.5° 左右
    for adeg in np.arange(2.0, 40.0, 0.05):
        al = math.radians(adeg)
        clv, _ = polhamus_CL(al, AR, a)
        d = clv - a * al
        if prev is not None and prev > 0 >= d:
            cross = adeg
            break
        prev = d
    chk(cross is not None and 8.0 < cross < 25.0,
        '涡升力增益由正转负的交叉点 α ≈ %.2f° (低迎角高于线性, 高迎角低于线性)'
        % (cross if cross else -1.0))

    sub('D. 条带积分：面积必须还原')
    _, I_int, S_chk = cm_q_strip(Geom(), geo, a)
    chk(abs(S_chk - geo['S']) / geo['S'] < 0.01,
        '条带积分面积 %.4f vs 解析面积 %.4f (误差 %.2f%%)'
        % (S_chk, geo['S'], 100 * abs(S_chk - geo['S']) / geo['S']))
    chk(I_int > 0, '惯量矩积分 I = %.5f > 0' % I_int)

    sub('E. 导数的符号与量级')
    # ⚠️ 先验【默认输入本身是否合理】。
    #    初版把重心按"根弦比例"给、气动中心按"MAC 比例"算，两套参考系混用，
    #    默认配置算出 SM = -0.40（静不稳定）—— 这条断言正是该抓它的。
    chk(r['SM'] > 0.0,
        '默认重心给出 SM = %+.1f%% MAC > 0 （静稳定，输入合理）' % (100 * r['SM']))
    chk(abs(r['SM']) < 0.30,
        'SM = %.1f%% 在合理范围 (<30%%，否则俯仰权限不够)' % (100 * r['SM']))
    chk(r['Cm_a'] < 0 if r['SM'] > 0 else r['Cm_a'] > 0,
        'C_m,α 符号与静稳定裕度一致 (SM=%+.4f → C_m,α=%+.4f)' % (r['SM'], r['Cm_a']))
    chk(-50.0 < r['Cm_q'] < 0.0,
        'C_m,q = %+.3f 为负且在合理区间' % r['Cm_q'])
    chk(r['Cn_b'] < 0, 'C_n,β = %+.5f 为负 (航向静稳定)' % r['Cn_b'])
    # ★ 这两条是初版真出过错的地方：符号写反会让仿真里偏航/滚转发散
    chk(r['Cn_r'] < 0,
        'C_n,r = %+.5f 为负 (航向阻尼必须反对偏航角速度) —— 初版得 +0.035' % r['Cn_r'])
    chk(r['Cl_b'] < 0,
        'C_l,β = %+.5f 为负 (后掠上反效应应稳定侧倾) —— 初版得 +0.014' % r['Cl_b'])
    chk(r['Cl_p'] < 0, 'C_l,p = %+.5f 为负 (滚转阻尼)' % r['Cl_p'])
    chk(r['Cl_da'] > 0, 'C_l,δa = %+.5f 为正 (副翼正偏产生正滚转)' % r['Cl_da'])
    chk(abs(r['Cm_de']) > 1e-3, 'C_m,δe = %+.4f 俯仰权限非零' % r['Cm_de'])

    sub('F. 阻力模型在理论极限上的行为')
    chk(drag_polar_simple(0.0, AR, 0.02) == 0.02,
        'C_L=0 时简单极曲线 = C_D0')
    chk(drag_polar_sharp_le(0.0, 0.3, 0.02) == 0.02,
        'C_L=0 时尖前缘极曲线 = C_D0')
    chk(drag_polar_sharp_le(0.5, 0.3, 0.02) > drag_polar_simple(0.5, AR, 0.02),
        '同 C_L 下尖前缘阻力 > 圆前缘 (吸力损失，物理正确)')

    sub('G. 单调性（防公式写反）')
    chk(cm_alpha(5.0, 0.10) < cm_alpha(5.0, 0.02),
        'SM 越大 C_m,α 越负')
    chk(tau_flap(0.4) > tau_flap(0.2), 'τ 随舵弦比单调增')
    chk(tau_flap(0.9) == tau_flap(0.6), 'τ 在 cf/c>0.6 处截断饱和')

    print()
    print('   ' + '─' * 62)
    print('   自检结果： 通过 %d   失败 %d' % (ok, fail))
    if fail == 0:
        print('   ✓ 全部自检通过 —— 量具可用')
    else:
        print('   ✗ 有失败项，上面数值不可信，先修脚本')
    print('   ' + '─' * 62)
    return fail == 0


# ═════════════════════════════════════════════════════════════════════
#  分发
# ═════════════════════════════════════════════════════════════════════

EXPS = {
    '1': ('几何：AR / MAC / 后掠角', exp1_geometry),
    '2': ('升力线斜率：Helmbold vs 2π', exp2_lift_slope),
    '3': ('涡升力：Polhamus 吸力类比', exp3_vortex_lift),
    '4': ('阻力与升阻比：两种模型', exp4_drag),
    '5': ('纵向导数：静稳定裕度', exp5_longitudinal),
    '6': ('横航向导数：偏航弱轴', exp6_lateral),
    '7': ('导出 SIM_Plane JSON', exp7_export_json),
    '8': ('自检：量具是否在工作', exp8_selfcheck),
}


def main():
    print('╔' + '═' * 72 + '╗')
    print('║' + '  lesson09k · 小展弦比三角翼（巡飞弹类）气动建模'.ljust(60) + '║')
    print('╚' + '═' * 72 + '╝')
    print()
    for k in sorted(EXPS):
        print('    %s. %s' % (k, EXPS[k][0]))
    print()
    args = sys.argv[1:]
    # 不带参数时先跑自检，再跑全部实验
    if not args:
        exp8_selfcheck()
        args = [k for k in sorted(EXPS) if k != '8']
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
