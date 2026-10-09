#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lesson09i_control_surfaces.py —— 卷 1 第 10 章《控制面与固定翼变体》配套仿真

    实验 1  channel_function_mixer 的矩阵：精确解耦与条件数
    实验 2  MIXING_GAIN 的范围：0.5~1.2 是怎么定的
    实验 3  MIXING_OFFSET：不对称协调
    实验 4  elevon 与 vtail 是同一个函数
    实验 5  共轴双桨：舵效随推力【反比】缩放
    实验 6  Lua 自定义混控：脚本能改什么

只用 numpy，不依赖 scipy。
运行： python lesson09i_control_surfaces.py [1-6]
"""

import sys
import math
import numpy as np


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
# 实验 1 —— channel_function_mixer 的矩阵
# ═════════════════════════════════════════════════════════════════════

def exp1_mixer_matrix():
    banner('实验 1 · channel_function_mixer 的矩阵：精确解耦与条件数')

    print('''
   ★★★ 先看源码（ArduPlane/servos.cpp:167-187，逐字）：

        void Plane::channel_function_mixer(func1_in, func2_in, func1_out, func2_out) const
        {
            float in1 = SRV_Channels::get_output_scaled(func1_in);
            float in2 = SRV_Channels::get_output_scaled(func2_in);

            // apply MIXING_OFFSET to input channels
            if (g.mixing_offset < 0) {
                in2 *= (100 - g.mixing_offset) * 0.01;
            } else if (g.mixing_offset > 0) {
                in1 *= (100 + g.mixing_offset) * 0.01;
            }

            float out1 = constrain_float((in2 - in1) * g.mixing_gain, -4500, 4500);
            float out2 = constrain_float((in2 + in1) * g.mixing_gain, -4500, 4500);
            SRV_Channels::set_output_scaled(func1_out, out1);
            SRV_Channels::set_output_scaled(func2_out, out2);
        }

   ★ 忽略 MIXING_OFFSET（下节讲），核心就两行：

        out1 = (in2 - in1) * gain
        out2 = (in2 + in1) * gain
   ''')

    sub('(a) 写成矩阵')
    print('''
        [out1]       [ -1   1 ] [in1]
        [out2] = g * [  1   1 ] [in2]

   ★ 令 M = g * [[-1,1],[1,1]]
   ★★ 注意：这是个【蝶形】结构（butterfly）—— 一个减法、一个加法。
   ''')

    g = 0.5
    M = g * np.array([[-1.0, 1.0], [1.0, 1.0]])
    print('   gain = %.2f 时：' % g)
    print('        M = [[%+.4f, %+.4f],' % (M[0, 0], M[0, 1]))
    print('             [%+.4f, %+.4f]]' % (M[1, 0], M[1, 1]))

    sub('(b) 奇异值：条件数与 gain 无关')
    print('   %-12s %14s %14s %14s' %
          ('gain', 'sv[0]', 'sv[1]', '条件数'))
    print('   ' + '─' * 58)
    for gg in (0.2, 0.5, 1.0, 1.2, 2.0):
        Mg = gg * np.array([[-1.0, 1.0], [1.0, 1.0]])
        sv = np.linalg.svd(Mg, compute_uv=False)
        print('   %-12.2f %14.6f %14.6f %14.6f' %
              (gg, sv[0], sv[1], sv[0] / sv[1]))
    print()
    print('   ★★ 两个奇异值【相等】=> 条件数恒等于 1.000')
    print('   ★★★ 也就是说：这个蝶形矩阵是一个【纯旋转 + 缩放】——')
    print('       它在两个方向上等量放大，没有任何一个轴被削弱。')
    print()
    print('   ★ 验证：M^T M = g^2 * [[2,0],[0,2]]')
    MTM = M.T @ M
    print('        M^T M = [[%.4f, %.4f],' % (MTM[0, 0], MTM[0, 1]))
    print('                 [%.4f, %.4f]]' % (MTM[1, 0], MTM[1, 1]))
    print('      对角 => 两个输入【正交】，互不串扰 ✓')

    sub('(c) ★★★ 加进气动矩阵：为什么它和 V 尾推导对得上')
    print('''
   卷 1 第 10 章推导过 V 尾：

        两个舵面各偏转 delta_L, delta_R
        每个舵面的升力在【垂直】和【水平】两个方向分解
        垂直分量 -> 俯仰，水平分量 -> 偏航

        [M_pitch]         [ cos(G)   cos(G) ] [delta_L]
        [N_yaw  ] = kappa [ sin(G)  -sin(G) ] [delta_R]

   ★ 而混控器给出的是：

        [delta_L]        [ 1   1 ] [elevator]
        [delta_R] = g *  [ 1  -1 ] [rudder  ]

   ★★★ 两者相乘：
   ''')

    for Gamma_deg in (30.0, 40.0, 45.0, 55.0):
        G = math.radians(Gamma_deg)
        A = np.array([[math.cos(G), math.cos(G)],
                      [math.sin(G), -math.sin(G)]])
        Mmix = g * np.array([[1.0, 1.0], [1.0, -1.0]])
        C = A @ Mmix
        print('   Γ = %4.1f° 时，合成矩阵 C = A @ M_mix =' % Gamma_deg)
        print('        [[%+.4f, %+.4f],' % (C[0, 0], C[0, 1]))
        print('         [%+.4f, %+.4f]]' % (C[1, 0], C[1, 1]))
        sv = np.linalg.svd(C, compute_uv=False)
        cond = sv[0] / sv[1] if sv[1] > 1e-12 else float('inf')
        theo = max(math.tan(G), 1.0 / math.tan(G))
        print('        条件数 = %.4f   (理论 max(tanΓ, cotΓ) = %.4f)' %
              (cond, theo))
        print()

    print('   ★★★★ 对手算的结果：合成矩阵是【对角】的（右上/左下都是 0）！')
    print('        => 俯仰只由 elevator 决定，偏航只由 rudder 决定')
    print('        => 混控是【精确解耦】的，没有任何交叉耦合')
    print()
    print('   ★★★★★ 而条件数正好是 max(tanΓ, cotΓ) ——')
    print('        这正是 卷 1 第 10 章手推出来的那个式子！')
    print()
    print('   ★★ 所以源码和手推完全一致：')
    print('        混控器负责【解耦】（蝶形矩阵是正交的）')
    print('        几何 Γ 负责【增益】（cosΓ 给俯仰，sinΓ 给偏航）')
    print('      => 两件事被干净地分开了')
    return M


def _vtail_combined(gain, Gamma_deg):
    G = math.radians(Gamma_deg)
    A = np.array([[math.cos(G), math.cos(G)],
                  [math.sin(G), -math.sin(G)]])
    Mmix = gain * np.array([[1.0, 1.0], [1.0, -1.0]])
    return A @ Mmix


# ═════════════════════════════════════════════════════════════════════
# 实验 2 —— MIXING_GAIN 的范围
# ═════════════════════════════════════════════════════════════════════

def exp2_mixing_gain():
    banner('实验 2 · MIXING_GAIN 的范围：0.5~1.2 是怎么定的')

    print('''
   ★★ 源码（ArduPlane/Parameters.cpp:594-599）：

        // @Param: MIXING_GAIN
        // @DisplayName: Mixing Gain
        // @Description: The gain for the Vtail and elevon output mixers
        // @Range: 0.5 1.2
        AP_GROUPINFO("MIXING_GAIN", 34, Plane, mixing_gain, 0.5f),

   ★★★ 注意三件事：
        ① 默认值 0.5
        ② 范围【下限就是 0.5】—— 不能更小
        ③ 上限 1.2 —— 可以更大
   ''')

    sub('(a) 为什么下限是 0.5：行程约束')
    print('   混控输出：out1 = g*(in2 - in1),  out2 = g*(in2 + in1)')
    print('   输入范围（归一化到 ±1）：|in1| <= 1, |in2| <= 1')
    print()
    print('   最坏情况 |in2 - in1| = 2 且 |in2 + in1| = 2')
    print('   （即 in1 = ±1, in2 = ±1 且符号使和/差最大）')
    print()
    print('   => |out| <= 2g')
    print('   => 要保证不出行程（|out| <= 1），需要 2g <= 1')
    print('   => g <= 0.5')
    print()
    print('   ★★★ 所以 0.5 是【两个输入都打满时不出界】的临界值。')
    print()
    print('   %-10s %14s %14s %16s' %
          ('gain', 'out1范围', 'out2范围', '同时打满时'))
    print('   ' + '─' * 60)
    for gg in (0.3, 0.5, 0.8, 1.0, 1.2):
        # 穷举四个角点
        outs = []
        for i1 in (-1.0, 1.0):
            for i2 in (-1.0, 1.0):
                outs.append((gg * (i2 - i1), gg * (i2 + i1)))
        m = max(max(abs(a), abs(b)) for a, b in outs)
        tag = '✓ 恰好到界' if abs(gg - 0.5) < 1e-9 else (
            '出界 %.2f 倍' % m if m > 1.0 else '未用满')
        print('   %-10.2f %14.2f %14.2f %16s' %
              (gg, max(abs(a) for a, b in outs),
               max(abs(b) for a, b in outs), tag))

    sub('(b) ★★★ g 到底是什么：它 = 单轴指令用掉多少行程')
    print('''
   ⚠️ 先纠正一个容易搞错的地方：

   上一节的"|out| <= 2g"是【两个通道同时打满】的最坏情况。
   平时飞的时候，很少两个通道同时满 —— 所以真正要问的是：

        一个【纯俯仰】指令（rudder = 0）会用到多少行程？

             in1 = 0, in2 = elevator

             out1 = g * elevator      （左舵面）
             out2 = g * elevator      （右舵面）

   ★★★ 两个舵面【同向、同幅】，幅度就是 g * elevator。

   => 所以 g 的物理含义非常直白：

        g = 【满偏的纯俯仰指令用掉舵机行程的比例】

        g = 0.5  ->  纯俯仰只用一半行程，两个轴同时满时刚好不截幅
        g = 1.0  ->  纯俯仰用满行程，但两轴同时满时【必定截幅】
        g = 1.2  ->  ★ 连纯俯仰都超出行程了
   ''')

    print('   %-8s %18s %18s %18s' %
          ('gain', '纯俯仰用行程', '两轴同时满', '截幅风险'))
    print('   ' + '─' * 68)
    for gg in (0.3, 0.5, 0.8, 1.0, 1.2):
        single = gg
        both = 2 * gg
        if both <= 1.0:
            risk = '无（有裕量 %.0f%%）' % ((1 - both) * 100)
        elif single <= 1.0:
            risk = '仅在组合机动时'
        else:
            risk = '★ 连单轴都截'
        print('   %-8.2f %18s %18s %18s' %
              (gg, '%.2f 行程' % single, '%.2f 行程' % both, risk))

    sub('(c) ★★★ 那上限 1.2 是干什么的：补偿小 Γ 的增益损失')
    print('   由实验 1，合成后的有效增益是：')
    print()
    print('        俯仰增益 = 2g * cos(Gamma)')
    print('        偏航增益 = 2g * sin(Gamma)')
    print()
    print('   ★ 如果 Gamma 很小，sin(Gamma) 接近 0 => 偏航几乎没权限')
    print('   ★★ 这时候就要把 g 调大来补 —— 代价是【牺牲截幅裕量】。')
    print()
    print('   本章算例：Γ = 40°（cos = %.4f, sin = %.4f）' %
          (math.cos(math.radians(40)), math.sin(math.radians(40))))
    print()
    print('   %-10s %16s %16s %14s' %
          ('gain', '俯仰增益', '偏航增益', '截幅裕量'))
    print('   ' + '─' * 60)
    c40 = math.cos(math.radians(40))
    s40 = math.sin(math.radians(40))
    for gg in (0.5, 0.7, 0.9, 1.0, 1.2):
        margin = 1.0 - 2 * gg
        mtxt = '%.0f%%' % (margin * 100) if margin >= 0 else '★ %.0f%%' % (margin * 100)
        print('   %-10.2f %16.4f %16.4f %14s' %
              (gg, 2 * gg * c40, 2 * gg * s40, mtxt))
    print()
    print('   ★★ 读表：g 从 0.5 加到 1.2，')
    print('       俯仰增益从 %.3f 涨到 %.3f（+140%%），' % (c40, 2.4 * c40))
    print('       偏航增益从 %.3f 涨到 %.3f（+140%%），' % (s40, 2.4 * s40))
    print('       但截幅裕量从 +0% 变成 -140%。')
    print()
    print('   ★★★ 所以调大 MIXING_GAIN 是一个【明确的交换】：')
    print('        用「组合机动时的截幅」换「单轴的舵面权限」')
    print()
    print('   ★ 源码注释说它的用途是 "increase the aileron response" ——')
    print('     对飞翼来说，滚转全靠升降副翼差动，权限本来就不够，')
    print('     所以宁可牺牲一点组合机动的裕量，也要把滚转拉起来。')

    sub('(d) ★★★ 一个 gain 补不了两个轴')
    print('   想让俯仰增益 = 1.0：需要 g = 1/(2cosΓ) = %.4f' % (1.0 / (2 * c40)))
    print('   想让偏航增益 = 1.0：需要 g = 1/(2sinΓ) = %.4f' % (1.0 / (2 * s40)))
    print()
    print('   ★★★ 两个值【不一样】—— 除非 Γ = 45°。')
    print()
    for Gd in (30.0, 40.0, 45.0, 55.0):
        cc = math.cos(math.radians(Gd))
        ss = math.sin(math.radians(Gd))
        need_p = 1.0 / (2 * cc)
        need_y = 1.0 / (2 * ss)
        print('   Γ = %4.1f°:  俯仰要 g=%.4f，偏航要 g=%.4f，差 %.3f 倍' %
              (Gd, need_p, need_y, max(need_p, need_y) / min(need_p, need_y)))
    print()
    print('   ★★ 差值在 Γ = 45° 时为 1.000（完全一致），')
    print('      偏离 45° 越多，两个轴需要的 g 差得越远。')
    print()
    print('   => 所以 MIXING_GAIN 只能【折中】：')
    print('      调大补了弱轴，但强轴就【过增益】了。')
    print()
    print('   ★★★★★ 第四个「分一半」：')
    print('        四旋翼  ±0.5      （姿态 + 油门平分行程）')
    print('        V 尾    g=0.5     （俯仰 + 偏航对半）')
    print('        直升机  0.45/1.0  （总距优先，周期分 0.45）')
    print('        倾转旋翼 0.5      （为了和多旋翼增益一致，见 03 课 §3.8.4）')
    print()
    print('   ★★★ 而【只有这一个是可以调的】—— 其余三个是写死的常数。')

    sub('(e) ★★ 参数范围透露的设计意图')
    print('   %-18s %22s %s' % ('范围', '含义', '典型机型'))
    print('   ' + '─' * 68)
    for rng, mean, who in (
            ('0.5（默认，下限）', '两轴各拿一半行程', '绝大多数常规 V 尾/飞翼'),
            ('0.5 ~ 0.8', '轻微加大单轴权限', '舵面偏小的机型'),
            ('0.8 ~ 1.2', '补偿小 Γ 的增益损失', 'Γ 偏小的 V 尾'),
            ('> 1.2', '（参数不接受）', '—')):
        print('   %-18s %22s %s' % (rng, mean, who))
    print()
    print('   ★★★ 参数【范围】本身就是一份设计文档 ——')
    print('       它告诉你在什么区间内调是有意义的。')
    print('   ★★★ 而下限恰好是默认值（0.5）这一点很说明问题：')
    print('       设计者认为【低于 0.5 没有意义】——')
    print('       因为那只是白白削弱两个轴，换不来任何好处。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 3 —— MIXING_OFFSET
# ═════════════════════════════════════════════════════════════════════

def exp3_mixing_offset():
    banner('实验 3 · MIXING_OFFSET：不对称协调')

    print('''
   ★★ 源码（ArduPlane/Parameters.cpp:608-614）：

        // @Param: MIXING_OFFSET
        // @DisplayName: Mixing Offset
        // @Description: This is used to adjust the relationship between the roll and pitch
        //               on a flying wing. It is used to increase the aileron response...
        // @Range: -1000 1000
        AP_GROUPINFO("MIXING_OFFSET", 35, Plane, mixing_offset, 0),

   ★ 而混控器里的用法（servos.cpp:175-181）：

        if (g.mixing_offset < 0) {
            in2 *= (100 - g.mixing_offset) * 0.01;
        } else if (g.mixing_offset > 0) {
            in1 *= (100 + g.mixing_offset) * 0.01;
        }

   ★★★ 注意这是【在混合之前】把某一个输入缩放。
   ''')

    sub('(a) 效果：把蝶形变成"斜"的')
    print('   不加 offset 时：out1 = g(in2 - in1),  out2 = g(in2 + in1)')
    print('   加 offset 后：某个输入先乘一个系数 k = (100±offset)/100')
    print()
    print('   ★ 效果是：两个输出的幅度变得【不对称】。')
    print()
    g = 0.5
    print('   %-10s %-12s %14s %14s %14s' %
          ('offset', '被缩放的', 'out1', 'out2', '|out1|-|out2|'))
    print('   ' + '─' * 68)
    for off in (-400, -200, 0, 200, 400):
        # elevator=1, aileron=1
        in1, in2 = 1.0, 1.0
        if off < 0:
            in2 *= (100 - off) * 0.01
            who = 'in2'
        elif off > 0:
            in1 *= (100 + off) * 0.01
            who = 'in1'
        else:
            who = '—'
        o1 = g * (in2 - in1)
        o2 = g * (in2 + in1)
        print('   %-10d %-12s %14.4f %14.4f %14.4f' %
              (off, who, o1, o2, abs(o1) - abs(o2)))

    print()
    print('   ★★ 读表：offset 越正，out1 和 out2 越不均衡。')
    print('   ★★★ 物理含义：V 尾或飞翼的两个舵面【一边偏得多、一边偏得少】。')
    print()
    print('   ★ 源码注释说的用途是 "increase the aileron response" ——')
    print('     飞翼没有独立的方向舵，滚转全靠升降副翼差动，')
    print('     所以需要额外的自由度来榨出滚转权限。')
    print()
    print('   ★★ 代价是：')
    print('       俯仰权限和滚转权限【不再正交】——')
    print('       打俯仰会带出滚转，反之亦然。')
    print('   ★ 这是 卷 1 第 10 章讲的「条件数」在【人为变差】。')

    sub('(b) ★★ 和 MIXING_GAIN 的分工')
    print('   %-18s %16s %16s' % ('', 'MIXING_GAIN', 'MIXING_OFFSET'))
    print('   ' + '─' * 52)
    for a, b, c in (
            ('作用', '整体放大', '单边缩放'),
            ('影响', '两个轴同时', '破坏正交性'),
            ('范围', '0.5 ~ 1.2', '-1000 ~ 1000'),
            ('默认', '0.5', '0'),
            ('什么时候用', '权限不够', '要额外的协调')):
        print('   %-18s %16s %16s' % (a, b, c))
    print()
    print('   ★ 一个改【增益】，一个改【几何关系】—— 两者相乘不冲突。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 4 —— elevon 与 vtail 是同一个函数
# ═════════════════════════════════════════════════════════════════════

def exp4_elevon_vtail():
    banner('实验 4 · elevon 与 vtail 是同一个函数')

    print('''
   ★★★★ 本章最值得记住的一处（ArduPlane/servos.cpp:1009-1011，逐字）：

        // run vtail and elevon mixers
        channel_function_mixer(SRV_Channel::k_aileron, SRV_Channel::k_elevator,
                               SRV_Channel::k_elevon_left, SRV_Channel::k_elevon_right);
        channel_function_mixer(SRV_Channel::k_rudder, SRV_Channel::k_elevator,
                               SRV_Channel::k_vtail_right, SRV_Channel::k_vtail_left);

   ★★★ 同一个函数，两次调用 —— 一次做 elevon，一次做 vtail。
   ''')

    sub('(a) 对照表')
    print('   %-12s %-16s %-16s %-16s %-16s' %
          ('用途', 'in1', 'in2', 'out1', 'out2'))
    print('   ' + '─' * 78)
    rows = [
        ('elevon', 'k_aileron', 'k_elevator', 'k_elevon_left', 'k_elevon_right'),
        ('vtail', 'k_rudder', 'k_elevator', 'k_vtail_right', 'k_vtail_left'),
    ]
    for r in rows:
        print('   %-12s %-16s %-16s %-16s %-16s' % r)
    print()
    print('   ★★★ 完全同构 —— 只有【通道名】不同，公式一模一样。')
    print()
    print('   elevon:  out1 = g*(elevator - aileron)   -> 左升降副翼')
    print('            out2 = g*(elevator + aileron)   -> 右升降副翼')
    print()
    print('   vtail:   out1 = g*(elevator - rudder)    -> 右 V 尾')
    print('            out2 = g*(elevator + rudder)    -> 左 V 尾')

    sub('(b) ★★ 为什么可以同构：物理上是同一件事')
    print('''
   飞翼（elevon）：
        两个舵面在【同一个水平面】上，左右分列
        -> 一起偏转 => 俯仰
        -> 差动偏转 => 滚转

   V 尾（vtail）：
        两个舵面在【同一个 V 形面】上，左右分列
        -> 一起偏转 => 俯仰
        -> 差动偏转 => 偏航

   ★★★ 两者的【混控结构】完全相同：
        一个"和"通道 + 一个"差"通道
        只是那两个通道【对应哪两个力矩】不同
   ''')
    print('   %-12s %16s %16s' % ('布局', '"和"通道', '"差"通道'))
    print('   ' + '─' * 46)
    print('   %-12s %16s %16s' % ('飞翼 elevon', '俯仰', '滚转'))
    print('   %-12s %16s %16s' % ('V 尾 vtail', '俯仰', '偏航'))
    print()
    print('   ★ 所以源码用一个函数就够了 —— 差别只在【传哪些通道进来】。')
    print('   ★★ 这是一个很漂亮的设计：把【数学】和【物理分配】分开。')
    print('      函数只管数学（和/差），通道名决定物理（哪个力矩）。')

    sub('(c) ★★ 顺带一说：多旋翼也有 VTAIL 机架')
    print('''
   ★ 别混淆：AP_MotorsMatrix 里也有一个叫 VTAIL 的机架
     （libraries/AP_Motors/AP_MotorsMatrix.cpp:716-720）：

        _frame_type_string = "VTAIL";
        add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
        add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
        add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
        add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);

   ★★★ 这是【四旋翼的 V 形布局】—— 四个电机排成 V 字，
       不是固定翼的 V 尾！
   ★ 枚举值 MOTOR_FRAME_TYPE_VTAIL = 4
     （libraries/AP_Motors/AP_Motors_Class.h:88）
   ★★ 这就是卷 1 第 9 章提到过的 VTAIL 机型，
       它的源码注释里那个 sin(radians(40)) 就是卷 1 第 10 章的 Γ。
   ''')
    print('   ★★★ 命名撞车的教训：')
    print('        「V 尾」在固定翼里指【气动舵面布局】')
    print('        「VTAIL」在多旋翼里指【电机位置布局】')
    print('      ★ 两者数学上都是卷 1 第 10 章那个 cos/sin 分解，')
    print('        但物理上毫无关系 —— 一个是舵面，一个是电机。')
    print('   ★★ 这正是 00 课 §0.6「机型是代码」要提醒的事：')
    print('      名字相同的两个东西，可能完全不是一回事。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 5 —— 共轴双桨：舵效随推力反比缩放
# ═════════════════════════════════════════════════════════════════════

def exp5_coax():
    banner('实验 5 · 共轴双桨：舵效随推力【反比】缩放')

    print('''
   共轴双桨（Coax）是多旋翼里的"异类"：

        上下两个桨【共轴】，反转
        上下桨的【推力差】做偏航（反扭矩）
        四个【舵面】（在旋翼下洗流里）做俯仰和滚转
        ★ 而不是像普通多旋翼那样，靠电机差动做姿态
   ''')

    sub('(a) ★★ 偏航：上下桨的推力差')
    print('''
   ★ 源码（libraries/AP_Motors/AP_MotorsCoax.cpp:190-191，逐字）：

        _thrust_yt_ccw = thrust_out + 0.5f * yaw_thrust;
        _thrust_yt_cw  = thrust_out - 0.5f * yaw_thrust;

   ★★★ 读出来：
        「总距 ± 半个偏航量」—— 又是【蝶形】结构！
        和 §实验 1 的混控器、V 尾、差速车是同一个数学。
   ''')

    sub('(b) ★★★ 俯仰/滚转：舵效 ∝ 1/推力')
    print('''
   ★★★ 源码（libraries/AP_Motors/AP_MotorsCoax.cpp:194-215，逐字节选）：

        // limit thrust out for calculation of actuator gains
        float thrust_out_actuator = constrain_float(MAX(_throttle_hover * 0.5f, thrust_out), 0.5f, 1.0f);
        ...
        // force of a lifting surface is approximately equal to the angle of attack
        // times the airflow velocity squared
        // static thrust is proportional to the airflow velocity squared
        // therefore the torque of the roll and pitch actuators should be approximately
        // proportional to the angle of attack multiplied by the static thrust.
        _actuator_out[0] = roll_thrust / thrust_out_actuator;
        _actuator_out[1] = pitch_thrust / thrust_out_actuator;
        ...
        _actuator_out[2] = -_actuator_out[0];
        _actuator_out[3] = -_actuator_out[1];

   ★★★ 注意那个【除法】：roll_thrust **除以** thrust_out_actuator。
   ''')

    print('   源码注释把理由讲得很清楚，本章把它写成公式：')
    print()
    print('        舵面升力  F  ∝  alpha * v^2        （alpha = 攻角，v = 气流速度）')
    print('        旋翼静推力 T ∝  v^2                （动量理论：v 是下洗速度）')
    print('        => F ∝ alpha * T')
    print('        => 要产生【固定】的力矩，alpha ∝ 1/T')
    print()
    print('   ★★★ 所以代码里的【除法】是在做「攻角补偿」：')
    print('       推力大 -> 下洗强 -> 同样攻角出的力大 -> 攻角要【减小】')
    print('       推力小 -> 下洗弱 -> 同样攻角出的力小 -> 攻角要【增大】')
    print()

    sub('(c) 数值：补偿前后的差别')
    print('   归一化的舵面命令 = roll_thrust / thrust_out_actuator')
    print()
    print('   %-14s %14s %16s %16s %14s' %
          ('悬停油门', 'thrust_out', '不补偿的输出', '补偿后的输出', '放大倍数'))
    print('   ' + '─' * 74)
    for hov in (0.30, 0.40, 0.50, 0.60, 0.80):
        thrust_out = hov
        act = max(hov * 0.5, thrust_out)
        act = min(max(act, 0.5), 1.0)
        raw = 0.5
        comp = raw / act
        print('   %-14.2f %14.2f %16.4f %16.4f %14.3f' %
              (hov, thrust_out, raw, comp, comp / raw))

    print()
    print('   ★★ 读表：')
    print('     · thrust_out <= 0.5 时，分母被 clamp 到 0.5 => 倍数恒为 2.0')
    print('     · thrust_out > 0.5 之后，分母就是它本身 => 倍数 = 1/thrust_out')
    print()
    print('   ★★★ 所以「推力越大，舵面命令越小」——')
    print('       这保证了【同样的姿态指令，产生同样的力矩】，')
    print('       不管飞机是在轻载悬停还是重载爬升。')
    print()
    print('   ★ 源码那个 MAX(_throttle_hover * 0.5f, thrust_out) 和 clamp 到 0.5：')
    print('     是在防止【低推力时分母太小 -> 命令爆表】。')
    print('   ★★ 又是一个「补偿系数会发散 -> 必须截断」的例子 ——')
    print('      和 03 课 §3.5.1 的 1/cos(delta) 截断在 0.98 是同一类。')

    sub('(d) ★★ 共轴和普通多旋翼的根本区别')
    print('   %-18s %22s %22s' % ('', '普通多旋翼', '共轴双桨'))
    print('   ' + '─' * 66)
    for a, b, c in (
            ('姿态手段', '电机差动', '★ 舵面（在下洗流里）'),
            ('偏航手段', '反扭矩差', '上下桨【推力差】'),
            ('执行器数', 'N 个电机', '2 个电机 + 4 个舵机'),
            ('舵效与推力', '无关', '★ 反比（要补偿）'),
            ('低推力时', '没问题', '★ 舵效【变强】，要限制'),
            ('高推力时', '没问题', '★ 舵效【变弱】，要放大')):
        print('   %-18s %22s %22s' % (a, b, c))
    print()
    print('   ★★★ 共轴的麻烦在于：它的姿态执行器【寄生在推力上】。')
    print('       推力变了，舵效跟着变 => 必须动态补偿。')
    print('   ★★ 而卷 1 第 9 章的单旋翼（AP_MotorsSingle）是同一个问题：')
    print('       「滑流舵效 ∝ 推力」—— 只是那边没有做除法补偿。')
    print()
    print('   ★ 这就引出一个对照：')
    print('        单旋翼：滑流舵效 ∝ 推力，但【没有补偿】')
    print('        共轴：  滑流舵效 ∝ 推力，【做了除法补偿】')
    print('      ★ 本章不判断哪个更好，只指出：')
    print('        同一个物理效应，两处代码的处理方式不同。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 6 —— Lua 自定义混控
# ═════════════════════════════════════════════════════════════════════

def exp6_lua_mixing():
    banner('实验 6 · Lua 自定义混控：脚本能改什么')

    print('''
   00 课 §0.6 说过一个趋势：【机型正在从代码变成数据】。
   这一节看它的具体形态 —— ArduPilot 暴露给 Lua 脚本的两个混控接口。
   ''')

    sub('(a) ★★ 接口一：Motors_6DoF（6 个自由度）')
    print('''
   ★ 绑定声明（libraries/AP_Scripting/generator/description/bindings.desc:562-566）：

        singleton AP_MotorsMatrix_6DoF_Scripting rename Motors_6DoF
        singleton AP_MotorsMatrix_6DoF_Scripting method init boolean uint8_t 0 AP_MOTORS_MAX_NUM_MOTORS
        singleton AP_MotorsMatrix_6DoF_Scripting method add_motor void int8_t 0 (AP_MOTORS_MAX_NUM_MOTORS-1) \\
            float'skip_check float'skip_check float'skip_check float'skip_check float'skip_check \\
            float'skip_check boolean uint8_t 0 AP_MOTORS_MAX_NUM_MOTORS

   ★ 而 C++ 侧的签名
     （libraries/AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h:35）：

        add_motor(int8_t motor_num,
                  float roll_factor, float pitch_factor, float yaw_factor,
                  float throttle_factor, float forward_factor, float right_factor,
                  bool reversible, uint8_t testing_order)

   ★★★ 数一下：**6 个混控因子** + 可逆标志 + 测试顺序。
   ''')
    cols = ['roll', 'pitch', 'yaw', 'throttle', 'forward', 'right']
    print('   %-12s %-10s %s' % ('参数', '位置', '含义'))
    print('   ' + '─' * 52)
    for i, c in enumerate(cols):
        extra = '  <- 普通多旋翼没有' if c in ('forward', 'right') else ''
        print('   %-12s %-10d %s%s' % (c + '_factor', i + 2, c, extra))
    print()
    print('   ★★★ 前 4 个（roll/pitch/yaw/throttle）是普通多旋翼就有的；')
    print('       后 2 个（forward/right）是【水平推力】—— 普通多旋翼没有。')
    print()
    print('   ★★ 这就是为什么它叫 "6DoF"：')
    print('       3 个力矩 + 3 个力 = 6 个自由度')
    print('       而普通多旋翼是 4 通道（3 力矩 + 1 个垂直力）')
    print()
    print('   ★ 源码里对应的输出公式')
    print('     （libraries/AP_Motors/AP_Motors6DOF.cpp:335-359 附近的同构形式）：')
    print()
    print('        rpy_out[i]    = roll*_roll[i] + pitch*_pitch[i] + yaw*_yaw[i]')
    print('        linear_out[i] = throttle*_thr[i] + forward*_fwd[i] + lateral*_lat[i]')
    print('        out[i] = constrain(rpy_out[i] + linear_out[i], -1, 1)')
    print()
    print('   ★★★ 注意是【两组分别求，再相加】—— 这是 6DOF 的一个特点。')

    sub('(b) ★★ 接口二：Motors_dynamic（4 自由度，可飞行中改）')
    print('''
   ★ 因子表结构（libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:24-29）：

        struct factor_table {
            float roll[AP_MOTORS_MAX_NUM_MOTORS];
            float pitch[AP_MOTORS_MAX_NUM_MOTORS];
            float yaw[AP_MOTORS_MAX_NUM_MOTORS];
            float throttle[AP_MOTORS_MAX_NUM_MOTORS];
        };

   ★★★ 4 个自由度（不是 6 个），最多 12 个电机。

   ★ 两个方法：
        load_factors(const factor_table &table)   // .h:41
        init(uint8_t expected_num_motors)         // .h:35

   ★★ 而 load_factors 的说明里有一句话很关键（.cpp:71-73）：
      「必须先 load_factors 才允许 init」。
   ★★★ 也就是【先给矩阵，再给电机数】—— 顺序不能反。
   ''')

    sub('(c) ★★★ 两个接口的对照')
    print('   %-22s %20s %22s' % ('', 'Motors_6DoF', 'Motors_dynamic'))
    print('   ' + '─' * 68)
    for a, b, c in (
            ('自由度', '6（3 力矩 + 3 力）', '4（3 力矩 + 油门）'),
            ('最大电机数', '12', '12'),
            ('能否运行时改', '否（init 后固定）', '★ 能（load_factors）'),
            ('哪些机型能用', '★ 仅 ArduCopter', '★ ArduPlane 也能用'),
            ('带不带反向', '带（reversible）', '不带'),
            ('绑定位置', 'bindings.desc:562-566', 'bindings.desc:617-629')):
        print('   %-22s %20s %22s' % (a, b, c))
    print()
    print('   ★★ 两个关键差异：')
    print('     ① Motors_dynamic 【可以飞行中改几何】——')
    print('        这意味着可以做一个【会变形的飞机】')
    print('     ② Motors_6DoF 【只给 Copter】，Motors_dynamic 【Plane 也能用】')
    print()
    print('   ★★★ 回到 00 课 §0.6 的那个判断：')
    print('       「机型正在从编译期常量变成运行期参数」')
    print('       而这两个接口是它的**下一站**：')
    print('       从【参数】再进一步，变成【脚本】。')
    print()
    print('   ★★★ 三级演化：')
    print('       ① 编译期枚举（FRAME_CLASS 决定实例化哪个类）')
    print('       ② 运行期参数（系数存在参数表里）')
    print('       ③ ★ 运行期脚本（系数由 Lua 现场算）')
    print()
    print('   ★ 而 ③ 的代价是【不可预测】：')
    print('       飞控再也无法在编译期验证混控矩阵是否合理。')
    print('   ★★ 这是一个很典型的权衡：')
    print('       灵活性 ↑  <->  可验证性 ↓')
    return None


# ═════════════════════════════════════════════════════════════════════

EXPS = {
    '1': ('channel_function_mixer 的矩阵：精确解耦与条件数', exp1_mixer_matrix),
    '2': ('MIXING_GAIN 的范围：0.5~1.2 是怎么定的', exp2_mixing_gain),
    '3': ('MIXING_OFFSET：不对称协调', exp3_mixing_offset),
    '4': ('elevon 与 vtail 是同一个函数', exp4_elevon_vtail),
    '5': ('共轴双桨：舵效随推力【反比】缩放', exp5_coax),
    '6': ('Lua 自定义混控：脚本能改什么', exp6_lua_mixing),
}


def main():
    print()
    print('╔' + '═' * 72 + '╗')
    print('║' + '  lesson09i · 控制面与固定翼变体 · 配套仿真'.ljust(60) + '║')
    print('╚' + '═' * 72 + '╝')
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
