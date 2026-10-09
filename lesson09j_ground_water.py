#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lesson09j_ground_water.py —— 卷 1 第 13 章《地面与水下载具》配套仿真

    实验 1  差速转向：第六次出现蝶形结构
    实验 2  转弯半径 -> 角速度：为什么分两步算
    实验 3  阿克曼 vs 差速：几何决定的转弯能力
    实验 4  水下 6DOF：两组分开归一化
    实验 5  浮力与重心/浮心分离：一个被动的扶正力矩
    实验 6  介质差异：水和空气差了 836 倍

只用 numpy，不依赖 scipy。
运行： python lesson09j_ground_water.py [1-6]

⚠️ 注意：本脚本以【打印演示】为主，只有少数小节挂了自检（见 check()）。
   退出码 1 只在【挂了自检的小节】失败时出现；"打印完成"不等于"验证通过"。
"""

import sys
import math
import numpy as np

G = 9.81


def banner(title):
    print()
    print('═' * 74)
    print('  ' + title)
    print('═' * 74)


def sub(title):
    print()
    print('── ' + title)
    print('─' * 74)


# ── 自检 ──────────────────────────────────────────────────────────────
# ⚠️ 本脚本【绝大多数小节是纯打印演示，没有断言】—— 也就是说跑完打印"完成。"
#   并不代表任何结论被验证过。下面这套 check() 只服务于【挂了自检的小节】；
#   一旦有 [!!]，末尾会以退出码 1 结束，让调用方（人和检查器）看得见。
_n_ok = 0
_n_bad = 0


def check(cond, msg):
    global _n_ok, _n_bad
    if cond:
        _n_ok += 1
        print('    [OK] %s' % msg)
    else:
        _n_bad += 1
        print('    [!!] %s' % msg)


# ═════════════════════════════════════════════════════════════════════
# 实验 1 —— 差速转向：第六次出现蝶形结构
# ═════════════════════════════════════════════════════════════════════

def exp1_differential():
    banner('实验 1 · 差速转向：第六次出现蝶形结构')

    print('''
   ★★★ 源码（libraries/AR_Motors/AP_MotorsUGV.cpp:892-893，逐字）：

        float motor_left = throttle_scaled + steering_scaled;
        float motor_right = throttle_scaled - steering_scaled;

   ★★ 归一化在 :809-810（逐字）：

        float steering_scaled = steering / 4500.0f; // steering scaled -1 to +1
        float throttle_scaled = throttle * 0.01f;  // throttle scaled -1 to +1

   ★★★★ 又是【蝶形】结构：

        [left ]       [ 1   1 ] [throttle]
        [right] =     [ 1  -1 ] [steering]

   ★ 和下面这些是【同一个数学】：
   ''')

    tbl = [
        ('卷 1 第 8 章', '多旋翼 ±0.5', 'normalise_rpy_factors'),
        ('卷 1 第 10 章', 'V 尾 / 飞翼混控', 'channel_function_mixer'),
        ('卷 1 第 11 章', '直升机斜盘', 'add_servo_angle'),
        ('卷 1 第 12 章', '尾坐式 roll', 'throttle ± roll*0.5'),
        ('卷 1 第 12 章', '共轴偏航', 'thrust ± 0.5*yaw'),
        ('★ 本章', '差速车左右轮', 'throttle ± steering'),
    ]
    print('   %-16s %-26s %s' % ('在哪', '做什么', '源码形式'))
    print('   ' + '─' * 70)
    for a, b, c in tbl:
        print('   %-16s %-26s %s' % (a, b, c))
    print()
    print('   ★★★ 六次 —— 而且这是本章群最后一次见到它。')
    print('       每一次的物理都完全不同：')
    print('         电机力矩 / 气动舵面 / 斜盘位移 / 推力矢量 / 反扭矩 / 轮速')

    sub('(a) 为什么甲虫形（蝶形）无处不在')
    print('''
   把两件事想清楚：

        ① 你有【两个执行器】，但想要【两个独立的控制量】
        ② 两个控制量是"共性 + 差性"：
             油门 = 两个轮子一起转多快   ->  和
             转向 = 两个轮子差多少       ->  差

   ★★★ 所以"和/差"不是巧合 —— 它是【从两个执行器里榨出两个独立自由度】
       的唯一办法（在只允许线性叠加的前提下）。

   ★★ 数学上：这是个 2x2 矩阵，只要它可逆，就能把 (左,右) 和 (和,差) 互相转换。
       蝶形矩阵 [1,1; 1,-1] 是最简单的可逆对称选择。
   ''')

    sub('(b) ★★★ 反过来看：从左右轮速反推油门和转向')
    print('   这正好是实验 1 的【逆问题】—— 而 PX4 就是那么写的。')
    print()
    print('   ★ PX4 的差速运动学'
          '（src/modules/differential_drive/DifferentialDriveKinematics/')
    print('     DifferentialDriveKinematics.cpp:95-96，逐字）：')
    print()
    print('        return Vector2f(linear_velocity_x - rotational_velocity,')
    print('                linear_velocity_x + rotational_velocity) / _max_speed;')
    print()
    print('   其中 :79 是：')
    print('        const float rotational_velocity = (_wheel_base / 2.f) * yaw_rate;')
    print()
    print('   ★★★ 写成公式：')
    print()
    print('        v_L = ( v - omega*L/2 ) / v_max')
    print('        v_R = ( v + omega*L/2 ) / v_max')
    print()
    print('   ★★★★ 和 ArduPilot 的混控对比：')
    print()
    print('        ArduPilot:  left = throttle + steering      (无 L !)')
    print('        PX4:        v_L  = v - omega*L/2             (含 L)')
    print()
    print('   ★★ 两者结构一样，但 PX4 多了一个【轴距 L】。')

    sub('(c) ★★★ 那 ArduPilot 的 L 去哪了')
    print('   ★ 答案：它不在这条式子里，而在【上一层】。')
    print()
    print('   看 Rover/mode_steering.cpp:24（逐字）：')
    print()
    print('        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);')
    print()
    print('   ★★★ 也就是说：')
    print()
    print('        desired_steering  ->  target_turn_rate   【在 mode_steering 里完成】')
    print('        target_turn_rate  ->  left/right         【在 AP_MotorsUGV 里完成】')
    print()
    print('   ★★ 而中间那一步【已经隐含了车辆几何】——')
    print('      它是由姿态控制器（写角速度）负责的。')
    print()
    print('   ★ 所以两条路线的分工不同：')
    print('     · ArduPilot：混控器【纯归一化】，几何在控制器侧')
    print('     · PX4：      运动学【显式含 L】，混控器吃到的是轮速')
    print()
    print('   ★★★ 本章不判断哪个更好，只指出：')
    print('       同一个物理关系，被放在了管线的不同位置。')

    sub('(d) 数值对照')
    print('   本章算例：轴距 L = 0.6 m，最大速度 v_max = 2.0 m/s')
    print()
    L = 0.6
    vmax = 2.0
    print('   %-12s %12s %14s %14s %14s' %
          ('v(m/s)', 'omega(rad/s)', 'v_L(m/s)', 'v_R(m/s)', '归一化差'))
    print('   ' + '─' * 66)
    for v, om in ((2.0, 0.0), (2.0, 1.0), (2.0, 2.0),
                  (1.0, 2.0), (0.5, 2.0), (0.0, 2.0)):
        vl = v - om * L / 2
        vr = v + om * L / 2
        diff = (vr - vl) / vmax
        print('   %-12.2f %12.2f %14.3f %14.3f %14.3f' %
              (v, om, vl, vr, diff))
    print()
    print('   ★★ 注意 v = 0 那一行：v_L = -0.6, v_R = +0.6 ——')
    print('      两边【等速反向】=> 原地旋转（pivot turn）。')
    print('   ★★★ 这就是差速车独有的能力：阿克曼车做不到原地转。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 2 —— 转弯半径 -> 角速度
# ═════════════════════════════════════════════════════════════════════

def exp2_turn_radius():
    banner('实验 2 · 转弯半径 -> 角速度：为什么分两步算')

    print('''
   ★★★ 源码里有【两个】函数，分两步：

   第一步（Rover/mode_steering.cpp:37，逐字）：

        float max_g_force = speed * speed / MAX(g2.turn_radius, 0.1f);

   第二步（libraries/APM_Control/AR_AttitudeControl.cpp:735-747，逐字）：

        float AR_AttitudeControl::get_turn_rate_from_lat_accel(float lat_accel, float speed) const
        {
            // enforce minimum speed to stop oscillations when first starting to move
            if (fabsf(speed) < AR_ATTCONTROL_STEER_SPEED_MIN) {
                ...
            }

            return lat_accel / speed;
        }

   ★★★ 两步合起来：
   ''')

    print('        第一步：  a_lat = v^2 / R          （向心加速度）')
    print('        第二步：  omega = a_lat / v        （角速度）')
    print('        ─────────────────────────────────')
    print('        合起来：  omega = v / R            （自行车模型）')
    print()

    sub('(a) ★★★ 为什么不在一步里直接算 omega = v/R')
    print('''
   ★ 因为【限幅的位置不同】：

        车辆的物理限制是【转弯半径 R】（几何/机械）
        而不是【角速度 omega】（运动学）

   ★★★ 看两个量对速度的依赖：

        omega_max = v / R       <- 正比于 v，高速时可以很大
        a_lat_max = v^2 / R     <- 正比于 v^2
   ''')
    print()
    print('   ★★ 如果按 omega 限幅：')
    print('       低速时 omega 上限很小 => 转弯能力被人为压低')
    print('       （其实低速时完全可以转得更快）')
    print()
    print('   ★★ 如果按 a_lat 限幅：')
    print('       a_lat 有明确的物理意义（侧向加速度、乘客舒适度、翻车极限）')
    print('       => 低速时自动放宽 omega，高速时自动收紧 ✓')
    print()
    print('   ★★★ 所以两步算的【真正目的】是把"几何限制"翻译成"运动学限制"，')
    print('       而翻译过程必须【每一步都过一遍物理限幅】。')

    sub('(b) 数值：两种限幅方式的差别')
    R = 0.9          # Rover/Parameters.cpp:453-460 的 TURN_RADIUS 默认值
    A_MAX = 3.0      # 侧向加速度上限 m/s^2
    print('   本章算例：R = %.1f m（ArduPilot 默认），a_lat 上限 = %.1f m/s^2'
          % (R, A_MAX))
    print('   ★ R 的默认值来自 Rover/Parameters.cpp:453-460（@Range: 0 10）')
    print()
    print('   %-10s %16s %16s %16s %s' %
          ('v(m/s)', 'w_max(R)', 'w_max(a)', '实际取', '谁在限制'))
    print('   ' + '─' * 74)
    v_crit_tbl = math.sqrt(A_MAX * R)
    tbl = []
    for v in (0.5, 1.0, 2.0, 3.0, 5.0, 10.0):
        w_by_R = v / R                    # 几何限幅：ω = v / R
        w_by_A = A_MAX / v                # 加速度限幅：a_lat = ω·v ⇒ ω = a_max / v
        # 由 a_lat <= A_MAX 得 v²/R <= A_MAX ⇒ v <= sqrt(A_MAX·R)
        if v <= v_crit_tbl:
            w, who = w_by_R, '几何 R'
        else:
            w, who = w_by_A, '★ 侧向加速度'
        tbl.append((v, w_by_R, w_by_A, w, who))
        print('   %-10.1f %16.3f %16.3f %16.3f %s' % (v, w_by_R, w_by_A, w, who))

    # ★★★ 这张表必须【自洽】：实际取值 = 两者中的小者，"谁在限制"要与之相符。
    #   光看表发现不了整列写错 —— 曾把 w_max(a) 写成常数 sqrt(a·R)/R
    #   （那是 A_MAX/v 在 v_crit 处的取值），整列塌成 1.826，
    #   与旁边"谁在限制"自相矛盾，却没有任何断言会喊。
    # ⚠️ 只查"表自洽"【不够】—— w 是从 w_by_A 取的，把 w_by_A 改成常数后
    #   两边一起变，min() 那条会跟着一起自洽（实测：注入常数后依旧全绿）。
    #   那是拿代码校代码自己。下面两条走【独立的物理定义】：
    for (v, wr, wa, w, who) in tbl:
        check(abs(w - min(wr, wa)) < 1e-12,
              'v=%.1f：实际取 %.3f = min(几何 %.3f, 加速度 %.3f)' % (v, w, wr, wa))
        # 几何限幅的定义式：ω·R = v
        check(abs(wr * R - v) < 1e-12,
              '几何列满足 ω·R = v（%.3f × %.1f = %.3f）' % (wr, R, wr * R))
        # 加速度限幅的定义式：ω·v = a_max（这才是"侧向加速度上限"的含义）
        check(abs(wa * v - A_MAX) < 1e-12,
              '加速度列满足 ω·v = a_max（%.3f × %.1f = %.3f）' % (wa, v, wa * v))
    check(all(r[4] == ('几何 R' if r[1] <= r[2] else '★ 侧向加速度') for r in tbl),
          '"谁在限制"标签与 min() 取值一致 ⇒ 表自洽（%d 档）' % len(tbl))

    print()
    print('   ★★★ 分界速度：')
    v_crit = math.sqrt(A_MAX * R)
    print('        v_crit = sqrt(a_max * R) = sqrt(%.1f * %.1f) = %.2f m/s' %
          (A_MAX, R, v_crit))
    print()
    print('   ★★ 低于 v_crit：按【几何】限制（转得动就转）')
    print('      高于 v_crit：按【加速度】限制（怕翻车/怕甩出去）')
    print()
    print('   ★★★ 这就是"分两步"的完整含义：')
    print('       它不是两步算术，而是【两个 regime 的自动切换】。')

    sub('(c) ★★ 一个特例：差速车可以原地转')
    print('   看 libraries/AR_WPNav/AR_WPNav.cpp:528-532（逐字节选）：')
    print()
    print('        _turn_radius = pivot_possible ? 0.0 : turn_radius;')
    print()
    print('   ★★★ 如果车辆能做原地旋转（pivot），转弯半径就设为【0】。')
    print('   ★ 而"能不能 pivot"由什么决定（AP_MotorsUGV.cpp:301-304，逐字）：')
    print()
    print('        bool AP_MotorsUGV::have_skid_steering() const')
    print('        {')
    print('            return (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) &&')
    print('                    SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) || is_omni();')
    print('        }')
    print()
    print('   ★★★ 所以判据是【通道绑定】—— 左右各有一个油门通道 => 差速车。')
    print('   ★★ 注意这里【没有】任何车型枚举参与 ——')
    print('      是"你绑了什么通道"决定了飞控认为你是什么车。')
    print('   ★★★ 又是 00 课 §0.6「机型从代码变成数据」的一个实例。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 3 —— 阿克曼 vs 差速
# ═════════════════════════════════════════════════════════════════════

def exp3_ackermann():
    banner('实验 3 · 阿克曼 vs 差速：几何决定的转弯能力')

    print('''
   ⚠️ 先说一个发现：ArduPilot 里【没有】Ackermann 类，
       也【没有】SkidSteer / Omni / MotorBoat 类
       —— 全部实现在 AP_MotorsUGV 这一个类里。

   ★ 源码里的四个输出函数：

        AP_MotorsUGV.cpp:695   output_regular      （阿克曼 / 普通转向）
        AP_MotorsUGV.cpp:784   output_skid_steering（差速 / 履带）
        AP_MotorsUGV.cpp:909   output_omni         （全向轮）
        ─────────────────────────────────────────────
        AP_MotorsUGV.cpp:301-304  have_skid_steering（判据）
   ''')

    sub('(a) 阿克曼的几何')
    print('   ★★ SITL 里有真正的阿克曼几何（libraries/SITL/SIM_Rover.cpp:52-58，逐字）：')
    print()
    print('        return turning_circle * sinf(radians(max_wheel_turn)) /')
    print('               sinf(radians(steering * max_wheel_turn));')
    print()
    print('   ★ 常量（libraries/SITL/SIM_Rover.h:44-46）：')
    print()
    print('        max_wheel_turn = 35.0f    (度)')
    print('        turning_circle = 1.8f     (m)')
    print('        skid_turn_rate = 140.0f   (度/s)')
    print()
    print('   ★★★ 注意那个式子：R 反比于 sin(转向角)。')
    print()
    print('        steering = 0      ->  R = 无穷大（直线）')
    print('        steering 最大     ->  R = turning_circle（最小转弯半径）')
    print('        ★ 而且 R 和 steering 是【非线性】关系（sin）')

    sub('(b) 数值：两种车的转弯能力')
    max_wheel_turn = 35.0
    turning_circle = 1.8
    print('   阿克曼：R(steering) = %.1f * sin(%.0f°) / sin(steering * %.0f°)' %
          (turning_circle, max_wheel_turn, max_wheel_turn))
    print()
    print('   %-14s %16s %16s %16s' %
          ('转向输入', '转向角(°)', 'R(m)', 'w@2m/s'))
    print('   ' + '─' * 64)
    for s in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        ang = s * max_wheel_turn
        if ang < 1e-6:
            Rv = float('inf')
            w = 0.0
        else:
            Rv = turning_circle * math.sin(math.radians(max_wheel_turn)) / \
                 math.sin(math.radians(ang))
            w = 2.0 / Rv
        print('   %-14.2f %16.1f %16s %16.4f' %
              (s, ang, 'inf' if not math.isfinite(Rv) else '%.3f' % Rv, w))

    print()
    print('   ★★ 差速车（对照）：')
    print('        skid_turn_rate = 140 °/s = %.4f rad/s（固定！）' %
          math.radians(140.0))
    print('        ★ 注意它【和速度无关】—— 差速车的转弯速率是定值')
    print('        => 转弯半径 R = v / omega = v / %.4f' % math.radians(140.0))
    print()
    print('   %-12s %18s %18s' % ('v(m/s)', '差速 R(m)', '阿克曼最小 R(m)'))
    print('   ' + '─' * 52)
    for v in (0.5, 1.0, 2.0, 4.0):
        print('   %-12.1f %18.3f %18.3f' %
              (v, v / math.radians(140.0), turning_circle))

    sub('(c) ★★★ 两者的本质区别')
    print('   %-16s %22s %22s' % ('', '阿克曼', '差速'))
    print('   ' + '─' * 62)
    for a, b, c in (
            ('转向方式', '前轮转角', '左右轮速差'),
            ('最小转弯半径', '★ 固定（几何决定）', '★ 0（可原地转）'),
            ('角速度上限', '随速度变化', '★ 固定（140°/s）'),
            ('低速机动性', '差', '★ 好'),
            ('高速稳定性', '★ 好（轮胎不侧滑）', '差（轮胎要侧滑）'),
            ('结构', '一个转向舵机', '两个电机/两套驱动')):
        print('   %-16s %22s %22s' % (a, b, c))
    print()
    print('   ★★★ 一句话：')
    print('        阿克曼是【几何受限】——最小半径写在机械上')
    print('        差速是【速率受限】——转弯快慢写在驱动上')
    print()
    print('   ★★ 而这就解释了实验 2 的"分两步"：')
    print('       阿克曼车的 R 是几何常数 => 用 v^2/R 限幅最自然')
    print('       差速车的 omega 是常数   => 用 omega 限幅最自然')
    print('     ★ ArduPilot 用的是【前者】，因为它是通用框架。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 4 —— 水下 6DOF：两组分开归一化
# ═════════════════════════════════════════════════════════════════════

def exp4_6dof():
    banner('实验 4 · 水下 6DOF：两组分开归一化')

    print('''
   ★★ 机型枚举（libraries/AP_Motors/AP_Motors6DOF.h:21-30，逐字）：

        typedef enum {
            SUB_FRAME_BLUEROV1,
            SUB_FRAME_VECTORED,
            SUB_FRAME_VECTORED_6DOF,
            SUB_FRAME_VECTORED_6DOF_90DEG,
            SUB_FRAME_SIMPLEROV_3,
            SUB_FRAME_SIMPLEROV_4,
            SUB_FRAME_SIMPLEROV_5,
            SUB_FRAME_CUSTOM
        } sub_frame_t;

   ★ 参数绑定（ArduSub/Parameters.cpp:281-287）：
        @Values: 0:BlueROV1, 1:Vectored, 2:Vectored_6DOF, 3:Vectored_6DOF_90,
                 4:SimpleROV-3, 5:SimpleROV-4, 6:SimpleROV-5, 7:Custom
   ''')

    sub('(a) ★★★ 一个 8 推进器的例子')
    print('   VECTORED_6DOF 的混控矩阵'
          '（libraries/AP_Motors/AP_Motors6DOF.cpp:156-166，逐字）：')
    print()
    print('        add_motor_raw_6dof(MOT_1,  0,     0,    1.0f,  0,    -1.0f,  1.0f,  1);')
    print('        add_motor_raw_6dof(MOT_2,  0,     0,   -1.0f,  0,    -1.0f, -1.0f,  2);')
    print('        add_motor_raw_6dof(MOT_3,  0,     0,   -1.0f,  0,     1.0f,  1.0f,  3);')
    print('        add_motor_raw_6dof(MOT_4,  0,     0,    1.0f,  0,     1.0f, -1.0f,  4);')
    print('        add_motor_raw_6dof(MOT_5,  1.0f, -1.0f, 0,    -1.0f,  0,      0,     5);')
    print('        add_motor_raw_6dof(MOT_6, -1.0f, -1.0f, 0,    -1.0f,  0,      0,     6);')
    print('        add_motor_raw_6dof(MOT_7,  1.0f,  1.0f, 0,    -1.0f,  0,      0,     7);')
    print('        add_motor_raw_6dof(MOT_8, -1.0f,  1.0f, 0,    -1.0f,  0,      0,     8);')
    print()
    print('   ★★ 列序：Roll / Pitch / Yaw / Throttle / Forward / Lateral / 测试顺序')
    print()
    print('   ★★★ 注意前 4 个和后 4 个的【结构完全不同】：')
    print()
    print('   %-8s %8s %8s %8s %8s %8s %8s %s' %
          ('电机', 'Roll', 'Pitch', 'Yaw', 'Thr', 'Fwd', 'Lat', '它在管什么'))
    print('   ' + '─' * 74)
    rows = [
        (1, 0.0, 0.0, 1.0, 0.0, -1.0, 1.0, '水平'),
        (2, 0.0, 0.0, -1.0, 0.0, -1.0, -1.0, '水平'),
        (3, 0.0, 0.0, -1.0, 0.0, 1.0, 1.0, '水平'),
        (4, 0.0, 0.0, 1.0, 0.0, 1.0, -1.0, '水平'),
        (5, 1.0, -1.0, 0.0, -1.0, 0.0, 0.0, '★ 垂直'),
        (6, -1.0, -1.0, 0.0, -1.0, 0.0, 0.0, '★ 垂直'),
        (7, 1.0, 1.0, 0.0, -1.0, 0.0, 0.0, '★ 垂直'),
        (8, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0, '★ 垂直'),
    ]
    for n, r, p, y, t, f, l, tag in rows:
        print('   %-8d %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f %s' %
              (n, r, p, y, t, f, l, tag))
    print()
    print('   ★★★ 读出来：')
    print('        电机 1~4：Yaw + Forward + Lateral 非零，Throttle = 0  -> 水平推进器')
    print('        电机 5~8：Roll + Pitch + Throttle 非零，Fwd/Lat = 0    -> 垂直推进器')
    print()
    print('   ★★ 所以这是一台【4 水平 + 4 垂直】的 ROV。')

    sub('(b) ★★★ 源码就是这么归一化的：两组分开')
    print('   libraries/AP_Motors/AP_Motors6DOF.cpp 里，垂推那组（:530-532，逐字）：')
    print()
    print('            rpt_out[i] = roll_thrust * _roll_factor[i] +')
    print('                         pitch_thrust * _pitch_factor[i] +')
    print('                         throttle_thrust * _throttle_factor[i];')
    print()
    print('   平推那组（:544-546，逐字）：')
    print()
    print('            yfl_out[i] = yaw_thrust * _yaw_factor[i] +')
    print('                         forward_thrust * _forward_factor[i] +')
    print('                         lateral_thrust * _lateral_factor[i];')
    print()
    print('   合成（:556，逐字）：')
    print()
    print('            _thrust_rpyt_out[i] = constrain_float(')
    print('                _motor_reverse[i]*(rpt_out[i]/rpt_max + yfl_out[i]/yfl_max),-1.0f,1.0f);')
    print()
    print('   ★★★★ 关键在最后一行：')
    print()
    print('            rpt_out / rpt_max  +  yfl_out / yfl_max')
    print('            └──── 第一组归一化 ───┘   └── 第二组归一化 ──┘')
    print()
    print('   ★★★ 两组【各自归一化到 1】，然后【相加】。')

    sub('(c) ★★★ 为什么不能像多旋翼那样统一归一化')
    print('   对比一下：')
    print()
    print('   %-20s %26s %26s' % ('', '多旋翼', 'ROV (VECTORED_6DOF)'))
    print('   ' + '─' * 74)
    for a, b, c in (
            ('归一化组数', '1 组', '★ 2 组'),
            ('每组覆盖', '全部电机（全轴）', '★ 不同的电机子集'),
            ('物理原因', '每个电机都出力矩+推力', '★ 水平/垂直是两套物理硬件'),
            ('饱和时的行为', '全机一起降额', '★ 两组各自饱和')):
        print('   %-20s %26s %26s' % (a, b, c))
    print()
    print('   ★★★ 根本原因：')
    print()
    print('        多旋翼：所有电机【同一个方向】装 => 可以统一缩放')
    print('        ROV：   水平/垂直推进器【互相垂直】=> 推力不可互换')
    print()
    print('   ★★ 水平推进器【再怎么出力也顶不了一分升力】；')
    print('      垂直推进器【再怎么出力也顶不了一分前进推力】。')
    print('   => 所以两组必须【各自归一化、各自饱和】')
    print()
    print('   ★★★ 这是本章群里【第一次】看到"执行器分成不可互换的组"。')
    print('       直升机（03b-02）是【单组】—— 虽然通道多，但都作用在桨盘上。')
    print('       VTOL（03b-03）是【两组但可切换】—— 升力组和推力组。')
    print('       而 ROV 是【两组并存且必须同时用】。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 5 —— 浮力与重心/浮心分离
# ═════════════════════════════════════════════════════════════════════

def exp5_buoyancy():
    banner('实验 5 · 浮力与重心/浮心分离：一个被动的扶正力矩')

    print('''
   ★★★ 先看一个很有代表性的细节
     （ArduSub/Parameters.cpp:773-774，逐字）：

        // We should ignore this parameter since ROVs are neutral buoyancy
        AP_Param::set_by_name("MOT_THST_HOVER", 0.5);

   ★★★★ 飞控【强制】把悬停油门设成 0.5，理由是"ROV 是中性浮力"。

   ★ 对比一下空中载具：
   ''')

    print('   %-16s %28s %28s' % ('', '多旋翼', 'ROV'))
    print('   ' + '─' * 74)
    for a, b, c in (
            ('悬停油门', 'W / T_max（算出来的）', '★ 0.5（强行写死的）'),
            ('物理含义', '"挡住重力所需推力"', '★ "不出力"'),
            ('为什么', '有重量要抗', '★ 浮力已经抵消了重量'),
            ('悬停油门调参', '很重要', '★ 无意义')):
        print('   %-16s %28s %28s' % (a, b, c))
    print()
    print('   ★★★ 所以 ROV 的"油门"和空中载具的"油门"【不是一回事】：')
    print('        空中：油门 = 抗重力的量      => 有明确的物理基准')
    print('        ROV： 油门 = 净浮力之外的余量 => 基准是"零"')

    sub('(a) ★★★ 第二个空中载具没有的东西：浮心-重心分离')
    print('   ★ 浮力作用在【浮心】（CB），重力作用在【重心】（CG）。')
    print('   ★ 如果两者不重合，就产生一个力矩。')
    print()
    print('   ★★★ 源码（libraries/SITL/SIM_Submarine.cpp:125-134，逐字）：')
    print()
    print('        void Submarine::calculate_buoyancy_torque(Vector3f &torque)')
    print('        {')
    print('            // Let\'s assume 2 Liters water displacement at the top, and ~ 2kg of weight at the bottom.')
    print('            const Vector3f force_up(0,0,-40); // 40 N upwards')
    print('            const Vector3f force_position = dcm.transposed() * Vector3f(0, 0, 0.15); // offset in meters')
    print('            torque = force_position % force_up;')
    print('        }')
    print()
    print('   ★★ 三行读出来：')
    print('        ① force_up = (0,0,-40)  —— NED 里 z 向下，所以 -40 是【向上 40 N】')
    print('        ② force_position  —— 浮力作用点相对重心的偏移，在机体里是 (0,0,0.15)')
    print('        ③ torque = force_position % force_up  —— ★ 叉乘！')

    sub('(b) 算一下：这是一个什么形式的力矩')
    print('   ★ 用叉乘公式，把偏移写成 r，浮力写成 F = (0,0,-F_b)：')
    print()
    print('        tau = r x F')
    print()
    print('   ★★ 当机体【水平】时，r 也在 z 轴上 => 叉乘为 0 => 没有力矩 ✓')
    print('   ★★★ 当机体【倾斜 theta】时，r 在 NED 里跟着转：')
    print()
    print('        r_NED = R(theta) * (0, 0, d)')
    print()
    print('   ★ 以俯仰为例，R 是绕 y 轴的旋转：')
    print()
    print('        r_NED = (d*sin(theta), 0, d*cos(theta))')
    print()
    print('   叉乘：')
    print()
    print('        tau = r_NED x (0,0,-F_b)')
    print('            = ( 0*(-F_b) - d*cos(theta)*0,')
    print('                d*cos(theta)*0 - d*sin(theta)*(-F_b),')
    print('                d*sin(theta)*0 - 0 )')
    print('            = (0, d*F_b*sin(theta), 0)')
    print()
    print('   ★★★ 所以力矩大小是：')
    print()
    print('        |tau| = F_b * d * sin(theta)')
    print()

    Fb = 40.0
    d = 0.15
    print('   ★ 本章算例 F_b = %.0f N, d = %.2f m => 刚度 k = F_b*d = %.2f N·m/rad'
          % (Fb, d, Fb * d))
    print()
    print('   %-14s %18s %18s' % ('倾斜角(°)', 'sin(theta)', '|tau| (N·m)'))
    print('   ' + '─' * 52)
    for th in (0, 5, 10, 20, 30, 45, 90):
        s = math.sin(math.radians(th))
        print('   %-14d %18.4f %18.3f' % (th, s, Fb * d * s))

    sub('(c) ★★★ 这个力矩意味着什么')
    print('   ★★★★ 它是一个【被动】的扶正力矩：')
    print()
    print('        正比于 sin(theta) —— 和【单摆】完全一样')
    print('        倾斜越大，回正力矩越大')
    print('        ★ 而且它【不需要飞控做任何事】—— 是物理自带的')
    print()
    print('   ★★ 所以 ROV 有一个"想要回到水平"的自然姿态。')
    print()
    print('   ★ 而符号（扶正 还是 加剧倾斜）取决于'
          '浮心在重心【上方】还是【下方】：')
    print()
    print('        浮心在上（CB 高于 CG）=> ★ 扶正（像单摆）—— 【稳定】')
    print('        浮心在下（CB 低于 CG）=> ★ 加剧倾斜      —— 【不稳定】')
    print()
    print('   ★★★ 这正是 PX4 那个模块的出处。')
    print('       看 src/modules/uuv_att_control/uuv_att_control.cpp:137（注释）：')
    print()
    print('        （引用 Mellinger/Kumar 与 Duecker "Submerged Furuta Pendulum"）')
    print()
    print('   ★★ "Furuta 摆"就是一个【倒立摆】——')
    print('       说明水下姿态控制问题被归到了"摆"这一类里。')
    print()
    print('   ⚠️ 本章不判断上面那段代码里 (0,0,0.15) 的符号'
          '对应"浮心在上"还是"在下"——')
    print('      那需要确定 dcm 的转置约定，源码里没有注释说明。')
    print('      ★ 但无论符号如何，【力矩形式】是一样的：k*sin(theta)。')

    sub('(d) ★★ 对照：空中载具没有这一项')
    print('   %-18s %22s %22s' % ('', '空中载具', '水下载具'))
    print('   ' + '─' * 62)
    for a, b, c in (
            ('重力作用点', '重心 CG', '重心 CG'),
            ('浮力作用点', '—（没有）', '★ 浮心 CB'),
            ('CG/CB 分离', '—', '★ 产生扶正力矩'),
            ('净效果', '纯重力', '重力 - 浮力'),
            ('自然姿态', '★ 无', '★ 有（水平）')):
        print('   %-18s %22s %22s' % (a, b, c))
    print()
    print('   ★★★ 这是水下和空中【最本质的差别】：')
    print()
    print('        空中：姿态【完全】靠飞控维持 —— 没有外力帮你')
    print('        水下：姿态有一半靠【物理】维持 —— 飞控只是"微调"')
    print()
    print('   ★★ 这带来一个实践后果：')
    print('        ROV 失去动力时通常能【自己扶正并浮上来】')
    print('        多旋翼失去动力时【直接掉下去】')
    print('   ★ 这就是为什么水下载具的失效模式完全不同。')
    return None


# ═════════════════════════════════════════════════════════════════════
# 实验 6 —— 介质差异
# ═════════════════════════════════════════════════════════════════════

def exp6_medium():
    banner('实验 6 · 介质差异：水和空气差了 836 倍')

    print('''
   ★ 源码里的水密度（libraries/SITL/SIM_Submarine.h:59，逐字）：

        const float water_density = 1023.6; // (kg/m^3)

   ★ 空气密度（标准大气）：
        rho_air = 1.225 kg/m^3
   ''')

    rho_w = 1023.6
    rho_a = 1.225
    print('   %-16s %16s' % ('介质', '密度 (kg/m^3)'))
    print('   ' + '─' * 34)
    print('   %-16s %16.1f' % ('空气', rho_a))
    print('   %-16s %16.1f' % ('水', rho_w))
    print('   %-16s %16.1f' % ('比值', rho_w / rho_a))

    sub('(a) ★★ 惯性和阻尼的关系完全反了')
    print('   ★ 运动方程（简化）：')
    print()
    print('        m * dv/dt = F_thrust - 0.5 * rho * v^2 * A * C_D')
    print()
    print('   ★ 定义【特征速度】—— 推力刚好等于阻力的速度：')
    print()
    print('        v_char = sqrt( 2*F / (rho * A * C_D) )')
    print()
    print('   ★★ 它和 rho 的平方根成反比：')
    print()
    print('        v_char(水) / v_char(空气) = sqrt(rho_air / rho_water) = %.4f' %
          math.sqrt(rho_a / rho_w))
    print()
    print('   ★★★ 也就是说：')
    print('        同样的推力、同样的迎流面积')
    print('        水下载具的最高速度只有空中的 %.1f%%' %
          (math.sqrt(rho_a / rho_w) * 100))
    print()
    print('   ★ 数值对照（假设 F = 50 N, A = 0.05 m^2, C_D = 1.0）：')
    print()
    F, A, CD = 50.0, 0.05, 1.0
    for name, rho in (('空气', rho_a), ('水', rho_w)):
        v = math.sqrt(2 * F / (rho * A * CD))
        print('     %-6s: v_char = sqrt(2*%.0f/(%.3f*%.2f*%.1f)) = %6.2f m/s' %
              (name, F, rho, A, CD, v))

    sub('(b) ★★★ 阻力系数是【各向异性】的')
    print('   ★ 源码（libraries/SITL/SIM_Submarine.h:78-80，逐字）：')
    print()
    print('        const Vector3f linear_drag_coefficient = Vector3f(1.4, 1.8, 2.0);')
    print('        const Vector3f angular_drag_coefficient = Vector3f(1.05, 1.05, 1.05);')
    print()
    print('   ★★★ 注意线阻【三个轴不一样】：1.4 / 1.8 / 2.0')
    print()
    print('   ★ 而角阻【三个轴一样】：1.05 / 1.05 / 1.05')
    print()
    print('   ★★★ 这两个"一样/不一样"各自说明了什么？')
    print()
    print('   %-14s %18s %s' % ('', '系数', '物理含义'))
    print('   ' + '─' * 62)
    for a, b, c in (
            ('x（前后）', '1.4', '★ 最流线 —— 主要运动方向'),
            ('y（左右）', '1.8', '中等'),
            ('z（上下）', '2.0', '★ 最钝 —— 横截面最大'),
            ('roll', '1.05', '—'),
            ('pitch', '1.05', '—'),
            ('yaw', '1.05', '—')):
        print('   %-14s %18s %s' % (a, b, c))
    print()
    print('   ★★★ 读法：')
    print('        线阻各向异性 => ROV 是【鱼形】的，前后细、上下粗')
    print('        角阻各向同性 => 旋转阻力来自整体尺寸，不分方向')
    print()
    print('   ★★ 对照空中载具（03b-03 的 VTOL 算例）：')
    print('        阻力只用【一个】系数（按 V^2 缩放），不分轴')
    print('        ★ 因为固定翼的迎流方向基本固定（机头朝前）')
    print('        ★★ 而 ROV 会【横着走、竖着走】—— 三个方向都得建模')

    sub('(c) ★★★ 一张总表：为什么 ROV 是"另一种载具"')
    print('   %-20s %24s %24s' % ('', '空中载具', '水下载具'))
    print('   ' + '─' * 70)
    for a, b, c in (
            ('重力的对手', '—（只有重力）', '★ 浮力'),
            ('悬停油门', '算出来的', '★ 强制 0.5'),
            ('自然姿态', '无', '★ 有（CG/CB 分离）'),
            ('阻力方向性', '单一（机头朝前）', '★ 三个轴不同'),
            ('执行器分组', '1 组或 2 组可切换', '★ 2 组并存'),
            ('自由度', '4 或 6', '★ 6'),
            ('断电后果', '掉下来', '★ 自己浮上来')):
        print('   %-20s %24s %24s' % (a, b, c))
    print()
    print('   ★★★ 最后一行的对照最值得记：')
    print('        空中的失效模式是【坠毁】—— 必须立刻处置')
    print('        水下的失效模式是【漂浮】—— 可以慢慢来')
    print()
    print('   ★★ 这就是为什么 05 课（03 章）那套"失效检测-降级-降落"逻辑')
    print('      在水下【没有对应物】—— 该保护的资产不一样。')
    print()
    print('   ★★★ 而本章群到这里【第一次】遇到一个')
    print('       "物理会帮你"的载具 —— 前面五种都是"物理在跟你作对"。')
    return None


# ═════════════════════════════════════════════════════════════════════

EXPS = {
    '1': ('差速转向：第六次出现蝶形结构', exp1_differential),
    '2': ('转弯半径 -> 角速度：为什么分两步算', exp2_turn_radius),
    '3': ('阿克曼 vs 差速：几何决定的转弯能力', exp3_ackermann),
    '4': ('水下 6DOF：两组分开归一化', exp4_6dof),
    '5': ('浮力与重心/浮心分离', exp5_buoyancy),
    '6': ('介质差异：水和空气差了 836 倍', exp6_medium),
}


def main():
    print()
    print('╔' + '═' * 72 + '╗')
    print('║' + '  lesson09j · 地面与水下载具 · 配套仿真'.ljust(60) + '║')
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
    if _n_bad:
        print('  [!!] %d 项自检失败 / %d 项通过' % (_n_bad, _n_ok))
        print('═' * 74)
        print()
        sys.exit(1)
    if _n_ok:
        print('  [OK] 已挂自检的 %d 项全部通过' % _n_ok)
        print('  ⚠️ 其余小节仍是纯打印演示（无断言），见文件头与 check() 处说明。')
    else:
        print('  完成（纯打印演示，无自检）。')
    print('═' * 74)
    print()


if __name__ == '__main__':
    main()
