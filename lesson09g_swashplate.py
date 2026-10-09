#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 1 第 11 章 · 直升机：斜盘动力学 —— 配套实验

    实验 1  ★★★ 斜盘与混控矩阵是【同一个数学】
    实验 2  ★★★ 6 种斜盘类型的几何对照
    实验 3  ★★★ 周期变距：力矩和推力【不共用约束】
    实验 4  ★★  0.45 缩放、斜盘行程与舵机线性化
    实验 5  ★★  Tandem / Transverse / Intermeshing 的差动
    实验 6  ★★  RSC：转速控制与悬停功率

用法：
    python lesson09g_swashplate.py 1
    ...
    python lesson09g_swashplate.py      # 全部

只需 numpy（画图需要 matplotlib）。

参考源码（ArduPilot Copter-4.5.7）：
    libraries/AP_Motors/AP_MotorsHeli_Swash.cpp:113-180   ★★★ 6 种斜盘类型
    libraries/AP_Motors/AP_MotorsHeli_Swash.cpp:182-188   ★★★ add_servo_angle（与混控同源）
    libraries/AP_Motors/AP_MotorsHeli_Swash.cpp:197-200   ★★★ 0.45 缩放
    libraries/AP_Motors/AP_MotorsHeli_Swash.cpp:215-241   ★★★ calculate() 混合与重标定
    libraries/AP_Motors/AP_MotorsHeli_Swash.cpp:244-252   ★★  舵机线性化（50 度臂）
    libraries/AP_Motors/AP_MotorsHeli_Single.cpp:442-446  总距 -> 斜盘
    libraries/AP_Motors/AP_MotorsHeli_Single.cpp:486      ★ 旋翼反扭矩前馈 ∝ W^1.5
    libraries/AP_Motors/AP_MotorsHeli_Dual.cpp:306-360    ★★ 三种双旋翼构型
    libraries/AP_Motors/AP_MotorsHeli_Quad.cpp:126-145    四旋翼直升机的系数
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
# 公共：斜盘
#
# ★ 复现 AP_MotorsHeli_Swash 的两层：
#     ① calculate_roll_pitch_collective_factors()  按类型生成系数
#     ② calculate()                                混合 + 重标定
# ===========================================================================

SWASH_TYPES = ["H1", "H3", "H3-140", "H3-120", "H4-90", "H4-45"]


def _raw(num, roll, pitch, coll):
    return (num, roll, pitch, coll)


def _angle(num, angle, coll):
    """源码 :182-188 —— 与 AP_MotorsMatrix 的 add_motor **同一个公式**"""
    return (num, math.cos(math.radians(angle + 90)), math.cos(math.radians(angle)), coll)


def swash_factors(swash_type, servo_pos=(-60.0, 60.0, 180.0), phase=0.0):
    """返回 [(ch, roll_fac, pitch_fac, coll_fac), ...]，已含 0.45 缩放（源码 :197-200）"""
    raw = []
    if swash_type == "H1":
        raw = [_raw(1, 1.0, 0.0, 0.0), _raw(2, 0.0, 1.0, 0.0), _raw(3, 0.0, 0.0, 1.0)]
    elif swash_type == "H3":
        raw = [_angle(1, servo_pos[0] - phase, 1.0),
               _angle(2, servo_pos[1] - phase, 1.0),
               _angle(3, servo_pos[2] - phase, 1.0)]
    elif swash_type == "H3-140":
        raw = [_raw(1, 1.0, 1.0, 1.0), _raw(2, -1.0, 1.0, 1.0), _raw(3, 0.0, -1.0, 1.0)]
    elif swash_type == "H3-120":
        raw = [_angle(1, -60.0, 1.0), _angle(2, 60.0, 1.0), _angle(3, 180.0, 1.0)]
    elif swash_type == "H4-90":
        raw = [_angle(1, -90.0, 1.0), _angle(2, 90.0, 1.0),
               _angle(3, 180.0, 1.0), _angle(4, 0.0, 1.0)]
    elif swash_type == "H4-45":
        raw = [_angle(1, -45.0, 1.0), _angle(2, 45.0, 1.0),
               _angle(3, -135.0, 1.0), _angle(4, 135.0, 1.0)]
    # :198-200
    return [(n, r * 0.45, p * 0.45, c) for (n, r, p, c) in raw]


def swash_mix(swash_type, roll, pitch, coll, servo_pos=(-60.0, 60.0, 180.0), phase=0.0):
    """复现 calculate()（源码 :215-241）。输入 roll/pitch/coll 都在 [0,1]"""
    out = []
    for (n, rf, pf, cf) in swash_factors(swash_type, servo_pos, phase):
        o = rf * roll + pf * pitch + cf * coll
        if swash_type == "H1" and n in (1, 2):
            o += 0.5
        out.append(2.0 * o - 1.0)
    return out


def linear_servo_output(x):
    """源码 :244-252 —— safe_asin(0.766044f * input) * 1.145916f"""
    x = max(-1.0, min(1.0, x))
    return math.asin(0.766044 * x) * 1.145916


# ===========================================================================
# 实验 1  ★★★ 斜盘与混控矩阵是同一个数学
# ===========================================================================

def exp1():
    head("实验 1  ★★★ 斜盘与混控矩阵是【同一个数学】")

    sub("(a) 两个函数并排")

    print("""
  AP_MotorsHeli_Swash.cpp:182-188（原文）：

      void AP_MotorsHeli_Swash::add_servo_angle(uint8_t num, float angle, float collective)
      {
          add_servo_raw(num,
                        cosf(radians(angle + 90)),
                        cosf(radians(angle)),
                        collective);
      }

  AP_MotorsMatrix.cpp:536-544（原文，01 课 §3.4 引过）：

      void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees,
                                      float pitch_factor_in_degrees, float yaw_factor, ...)
      {
          add_motor_raw(
              motor_num,
              cosf(radians(roll_factor_in_degrees + 90)),
              cosf(radians(pitch_factor_in_degrees)),
              yaw_factor, ...);
      }

  ★★★ 把两段对齐看：

      cosf(radians(angle   + 90))   cosf(radians(angle))
      cosf(radians(roll_deg + 90))  cosf(radians(pitch_deg))

  ==> **完全同一个三角函数、同一个角度约定。**
""")

    sub("(b) ★★★ 但物理意义完全不同")

    print("      %-12s %-22s %-24s %s" % ("", "角度 theta 是什么", "cos/sin 分量是什么", "结果喂给谁"))
    print("      " + "-" * 80)
    print("      %-12s %-22s %-24s %s" % ("多旋翼", "**电机的位置**", "电机产生的**力矩**", "混控矩阵"))
    print("      %-12s %-22s %-24s %s" % ("直升机", "**舵机的安装方位**", "舵机推斜盘的**位移**", "斜盘三个舵机"))
    print("""
  ★★★ 一句话：

      多旋翼：theta 是「力从哪里来」   -> 圆周上的力源分布
      直升机：theta 是「舵机在哪里」   -> 圆周上的推杆分布

  ★★ 两者都在做同一件事：**把一个圆周上的执行器，
     按方位角分解成两个正交分量（roll / pitch）。**

  ⇒ 这是 03b 章里"同一个数学、两个物理"的第二个例子
    （第一个是 01 课 §1.7 的 V 尾 Gamma）。
""")

    sub("(c) ★ 数值验证：两者共用同一个恒等式")

    print("      角度       cos(a+90)      -sin(a)        cos(a)        校核")
    print("      " + "-" * 66)
    for a in [-135, -90, -60, -45, 0, 45, 60, 90, 180]:
        c90 = math.cos(math.radians(a + 90))
        ns = -math.sin(math.radians(a))
        ca = math.cos(math.radians(a))
        print("      %-10.1f %-14.6f %-14.6f %-14.6f %s"
              % (a, c90, ns, ca, "✅" if abs(c90 - ns) < 1e-12 else "❌"))
    print("""
  ★ 01 课 §3.4.2 说过的那个恒等式 `cos(theta+90) = -sin(theta)`
    —— **直升机这边用的是同一个**。
""")


# ===========================================================================
# 实验 2  ★★★ 6 种斜盘类型的几何
# ===========================================================================

def exp2():
    head("实验 2  ★★★ 6 种斜盘类型的几何对照")

    sub("(a) 源码里的 6 个 case")

    print("""  AP_MotorsHeli_Swash.cpp:123-178（原文，逐类型）：
""")
    for t in SWASH_TYPES:
        print("  ── %s" % t)
        for (n, rf, pf, cf) in swash_factors(t):
            print("       CH_%d :  roll %+7.4f   pitch %+7.4f   coll %+.2f" % (n, rf, pf, cf))
        print()
    print("""  ★★★ 注意两类写法的区别（源码注释自己说了）：

      **用 `add_servo_angle` 的**（H3 / H3-120 / H4-90 / H4-45）：
          系数从【角度】算，物理意义清晰

      **用 `add_servo_raw` 的**（H1 / H3-140）：
          系数【直接给】

  ★ 而源码注释把这件事说得更细（:125-126 / :143 / :152）：

      H3      : "phase angle is adjustable"        <- 相位角可调
      H3-120  : "not a pure mixing swashplate, phase angle is adjustable"
      H3-140  : "1:1 pure input style, phase angle not adjustable"
      H4-90   : "1:1 pure input style, phase angle not adjustable"
""")

    sub("(b) ★★★ 只有 H3 一个类型真的可调相位")

    print("""  ★ 看代码：只有 `SWASHPLATE_TYPE_H3` 用了 `_phase_angle`：

      :127-129   add_servo_angle(CH_1, _servo1_pos - _phase_angle, 1.0);
                 add_servo_angle(CH_2, _servo2_pos - _phase_angle, 1.0);
                 add_servo_angle(CH_3, _servo3_pos - _phase_angle, 1.0);

  ★ 其余类型的角度是**硬编码**的（-60/60/180、-90/90/180/0、-45/45/-135/135）。

  ★★ 相位角是什么：斜盘推杆的**安装方位**和桨叶挥舞之间的固有相位差 ——
     它是**机械安装**决定的，调相位角就是在补偿安装误差。
""")

    sub("(c) 几何：为什么 H3-120 叫 120")

    print("  ★ 把 H3-120 的三个方位角排开，看间隔：\n")
    angs = [-60.0, 60.0, 180.0]
    print("      舵机方位角： %s" % "  ".join("%+.0f°" % a for a in angs))
    gaps = []
    for i in range(3):
        d = (angs[(i + 1) % 3] - angs[i]) % 360
        gaps.append(d)
    print("      相邻间隔：   %s" % "  ".join("%.0f°" % g for g in gaps))
    print("""
  ★★ 三个都是 120° —— **这就是名字里 "120" 的含义**（等边三角形布置）。

  ★ 同理：
      H3-140  ->  140° 的等腰布置（源码用 raw 系数，角度不在代码里）
      H4-90   ->  四个舵机间隔 90°（十字）
      H4-45   ->  四个舵机间隔 90° 但整体转 45°（X 形）

  ★ 验证 H4-90 / H4-45 的间隔：
""")
    for t in ["H4-90", "H4-45"]:
        fac = swash_factors(t)
        angs = []
        print("      %-8s 系数：" % t, end="")
        for (n, rf, pf, cf) in fac:
            # 反解角度：roll=-sin, pitch=cos -> angle=atan2(-roll, pitch)
            a = math.degrees(math.atan2(-rf, pf))
            angs.append(a)
            print(" CH_%d=%+.0f°" % (n, a), end="")
        g = [(angs[(i + 1) % 4] - angs[i]) % 360 for i in range(4)]
        print("   间隔 %s" % "/".join("%.0f" % x for x in g))
    print("""
  ★★ 注意 H4-90 反解出来是 -90/90/180/0（间隔 180/90/180/90）——
     它是**十字形**（对径成对）。
     ★ 而 H4-45 是 -45/45/-135/135，间隔都是 90°（X 形）。
""")


# ===========================================================================
# 实验 3  ★★★ 周期变距：力矩和推力不共用约束
# ===========================================================================

def exp3():
    head("实验 3  ★★★ 周期变距：力矩和推力【不共用约束】")

    sub("(a) 多旋翼的力矩从哪来")

    print("""  ★ 01~03 课的结论（01 课 §3.3.2）：

      tau_roll = -sum_i y_i * T_i

  ⇒ **力矩来自【推力差】。**

  ★★ 于是「力矩上限」和「推力上限」是**同一个约束**：

      每个电机： 0 <= T_i <= T_max
      要 roll 力矩 tau  =>  需要左右推力差
      => tau_max = R * sum_{右半} |sin(theta_i)| * T_max
      => ★ **tau_max 正比于 T_max** —— 想要更大力矩就要更大的电机

  ★ 这是 05 课 §7.4「可达力旋量集」的结论：
     力矩和推力在**同一个箱子**里争空间。
""")

    sub("(b) ★★★ 直升机的力矩从哪来")

    print("""  ★ 直升机的 roll 力矩来自【桨盘倾斜】—— 而桨盘倾斜来自**周期变距**：

      斜盘倾斜 -> 桨叶在旋转中周期性改变桨距
               -> 桨盘平面的升力不对称
               -> **桨盘平面整个倾斜**
               -> 拉力矢量随之倾斜 -> 产生滚转力矩

  ★★★ 关键：**这个过程几乎不改变旋翼的总拉力。**
     改变的是拉力的**方向**，不是**大小**。

  ★★ 所以约束是分开的：

      %-22s %-30s %s
      %-22s %-30s %s
      %-22s %-30s %s
""" % ("量", "由什么决定", "上限来自哪",
       "总拉力 T", "**总距**（collective）", "桨叶能承受的最大攻角（失速）",
       "滚转力矩 tau", "**周期变距**（cyclic）", "斜盘最大倾角"))
    print("""
  ⇒ **两个物理量、两个上限、互不直接相关。**
""")

    sub("(c) ★★ 但它们共享【舵机行程】—— 这才是真相")

    print("""  ⚠️ 别高兴太早。看源码 :228：

      _output[i] = (_rollFactor[i] * roll)
                 + (_pitchFactor[i] * pitch)
                 + _collectiveFactor[i] * collective;

  ★★★ **roll、pitch、collective 三个量【加在同一个舵机上】。**
     => 它们**共享舵机行程**，和四旋翼共享电机行程是同一回事。

  ★ 区别在于**怎么分**：

      %-10s %-14s %-30s %s
      %-10s %-14s %-30s %s
      %-10s %-14s %-30s %s
      %-10s %-14s %-30s %s
""" % ("载具", "缩放", "分配方式", "源码",
       "四旋翼", "各 +-0.5", "姿态三轴 + 油门**平分**", "normalise_rpy_factors（02 课 §4.2.5）",
       "V 尾固定翼", "x 0.5", "俯仰和偏航**对半**", "MIXING_GAIN（04 课 §6.5.4）",
       "直升机", "* 0.45 / 1.0", "★ **总距占满 1.0，周期变距分 0.45**", "AP_MotorsHeli_Swash.cpp:198-200"))
    print("""
  ★★★ 直升机这里**总距不让步**（系数 1.0），周期变距只拿 0.45。
     **这是有物理原因的**：

        总距直接决定升力 -> 升力不够飞机就掉 -> **不能让**
        周期变距决定力矩 -> 力矩不够只是"转得慢" -> 可以让

     ★ 而四旋翼的"油门"和姿态在**同一个电机**上，
       物理上分不开，只能对半分。

  ⇒ **同一个设计约束（共享行程），因为物理结构不同，
     给出了三个不同的分配方案（0.5 / 0.5 / 0.45）。**
""")

    sub("(d) 数值：0.45 对应的行程预算")

    print("      %-18s %-14s %-14s %s" % ("输入组合", "舵机 _output 范围", "重标定后", "是否出界"))
    print("      " + "-" * 68)
    cases = [
        ("纯总距 coll=1", 1.0, 0.0, 1.0),
        ("纯周期 roll=1", 1.0, 0.0, 0.0),
        ("总距 + 周期满", 1.0, 1.0, 1.0),
        ("总距 0.5 + 周期 1", 1.0, 1.0, 0.5),
    ]
    for name, r, p, c in cases:
        o = swash_mix("H3-120", r, p, c)
        lo, hi = min(o), max(o)
        print("      %-18s [%+.3f, %+.3f]  %-14s %s"
              % (name, lo, hi, "出界!" if (hi > 1.001 or lo < -1.001) else "在界内",
                 "⚠" if (hi > 1.001 or lo < -1.001) else ""))
    print("""
  ★★ 读法：**纯总距**和**纯周期**都在界内；
     但**总距打满 + 周期也打满**会**出界** ——

        最大值 = 1.0（总距）+ 0.45（roll）+ 0.45（pitch）= 1.9
        重标定后 = 2*1.9 - 1 = 2.8  => **远超 [-1, 1]**

  ⇒ 这正是「共享行程」的字面含义：**三个通道抢同一个舵机。**
     0.45 不是保证不出界的魔数，而是**给典型工况留的余量**。
""")


# ===========================================================================
# 实验 4  ★★ 舵机线性化
# ===========================================================================

def exp4():
    head("实验 4  ★★ 斜盘行程与舵机线性化")

    sub("(a) 源码")

    print("""
  AP_MotorsHeli_Swash.cpp:244-252（原文）：

      float AP_MotorsHeli_Swash::get_linear_servo_output(float input) const
      {
          input = constrain_float(input, -1.0f, 1.0f);
          // the output is scaled to the range -1 to 1
          return safe_asin(0.766044f * input) * 1.145916;
      }
""")

    sub("(b) ★★★ 那两个魔数是什么")

    print("""  ★★★ 解一下：

      0.766044 = cos(40 deg) = **sin(50 deg)**
      1.145916 = 1 / 0.872665
      0.872665 = 50 deg 的弧度值（50 * pi/180 = 0.872665）

  ★★ 所以整个式子是：

      output = asin( sin(50 deg) * input ) / (50 deg 的弧度)

  ★ 验证满行程（input = 1）：

      asin(sin(50 deg)) = 50 deg = 0.872665 rad
      0.872665 / 0.872665 = **1.0**  ✓

  ★★★ 物理含义：**舵机臂转 50 度对应斜盘满行程。**

      斜盘的实际位移 ∝ sin(舵机臂转角)
      => 要让【位移】线性，舵机角度必须是 asin(位移) 的形式

  ⇒ **这就是「线性化」三个字的含义**：把执行器输入从"位移"翻译成"角度"。
""")
    print("      %-12s %-16s %-16s %s" % ("输入(位移)", "线性化前(角度)", "线性化后", "比值"))
    print("      " + "-" * 62)
    for x in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]:
        raw = x
        lin = linear_servo_output(x)
        print("      %-12.2f %-16.3f %-16.4f %.3f"
              % (x, raw, lin, (lin / raw) if raw > 1e-9 else float("nan")))
    print("""
  ★★ 读法：**在不线性化的情况下**，0.2 的输入只给 0.2 的"角度"（近似），
     但真实位移是 sin(0.2) = 0.199 —— 差得不多。
     **在大输入时差别才明显**：1.0 的输入如果按"角度=位移"处理，
     实际位移是 sin(1 rad) = 0.841，**少了 16%**。

  ⇒ 线性化的收益在大舵量时最大 —— 而大舵量正是机动的时候。
""")


# ===========================================================================
# 实验 5  ★★ 双旋翼直升机的差动
# ===========================================================================

def exp5():
    head("实验 5  ★★ Tandem / Transverse / Intermeshing 的差动")

    sub("(a) 三种构型")

    print("""  AP_MotorsHeli_Dual.h:16-18（原文）：

      #define HELI_DUAL_FRAME_TANDEM       0    // 纵列（像 Chinook）
      #define HELI_DUAL_FRAME_TRANSVERSE   1    // 横列（像 V-22 但不倾转）
      #define HELI_DUAL_FRAME_INTERMESHING 2    // 交叉啮合（像 K-MAX）

  ★ 三种构型的**差动策略完全不同** —— 见下面三段源码。
""")

    sub("(b) Tandem（纵列）：差动【总距】做俯仰，差动【周期】做偏航")

    print("""  AP_MotorsHeli_Dual.cpp:309-317（原文）：

      const float swash1_roll = roll_input + _yaw_scaler * yaw_input;
      const float swash2_roll = roll_input - _yaw_scaler * yaw_input;
      const float swash1_coll =  0.45 * _dcp_scaler * (pitch_input + ...) + collective1_input;
      const float swash2_coll = -0.45 * _dcp_scaler * (pitch_input + ...) + collective2_input;

  ★★ 读两条：

      ① **前后两个斜盘的总距【反号】**（+0.45 和 -0.45）=> 产生**俯仰力矩**
         前旋翼拉大、后旋翼拉小 => 机头下压 ✓

      ② **前后两个斜盘的周期变距【反号】**（+yaw 和 -yaw）=> 产生**偏航力矩**
         ★★ 因为两个旋翼的周期变距反向 => 拉力矢量朝相反方向倾斜
            => 两个倾斜的拉力形成一个**力偶** => 就是偏航力矩

  ★★★ ② 是 Tandem 最反直觉的地方：
     **偏航不是靠反扭矩，而是靠两个旋翼的"周期变距反向"。**
""")

    sub("(c) Transverse（横列）：反过来")

    print("""  AP_MotorsHeli_Dual.cpp:331-336（原文节选）：

      const float swash1_pitch = pitch_input - _yaw_scaler * yaw_input;
      const float swash2_pitch = pitch_input + _yaw_scaler * yaw_input;
      const float swash1_coll =  0.45 * _dcp_scaler * (roll_input + ...) + collective1_input;

  ★ 对照 Tandem：

      %-12s %-22s %-26s %s
      %-12s %-22s %-26s %s
      %-12s %-22s %-26s %s
""" % ("构型", "差动总距做", "差动周期做", "原因",
       "Tandem（纵列）", "**俯仰**", "**偏航**", "前后布局，总距差自然产生俯仰",
       "Transverse（横列）", "**滚转**", "**偏航**", "左右布局，总距差自然产生滚转"))
    print("""
  ★★★ 规律很清楚：**总距差产生的力矩方向，由两个旋翼的【连线方向】决定。**

      连线沿 x（纵列）  -> 总距差 -> 俯仰力矩
      连线沿 y（横列）  -> 总距差 -> 滚转力矩
      而**周期变距差的力偶**总是垂直于连线 => 两次都做偏航
""")

    sub("(d) Intermeshing（交叉啮合）：差动总距【也】做偏航")

    print("""  AP_MotorsHeli_Dual.cpp:350-355（原文节选）：

      const float swash1_pitch = pitch_input - _yaw_scaler * yaw_input;
      const float swash2_pitch = pitch_input + _yaw_scaler * yaw_input;
      const float swash1_coll =   0.45 * _dcp_scaler * yaw_input + collective1_input;
      const float swash2_coll =  -0.45 * _dcp_scaler * yaw_input + collective2_input;

  ★★★ 请看最后两行：**差动总距的输入是 `yaw_input`，不是 `pitch` 或 `roll`。**

  ★ 为什么？因为交叉啮合的两个旋翼**几乎是并排的**（像 K-MAX），
     但它们的**转向相反且桨盘重叠** ——

      两个旋翼的总距差 => 两个旋翼的【反扭矩差】
                        => **净偏航力矩**

  ⇒ **交叉啮合是唯一一个"差动总距做偏航"的构型。**

  ★★ 所以三种构型的偏航机理各不相同：

      Tandem      : 周期变距差 -> 力偶
      Transverse  : 周期变距差 -> 力偶
      Intermeshing: **反扭矩差**（和单旋翼尾桨同一个机理）
""")

    sub("(e) ★ 一个额外的前馈：反扭矩随总距变化")

    print("""  AP_MotorsHeli_Single.cpp:486（原文）：

      float yaw_offset = _collective_yaw_scale
                       * powf(fabsf(collective - _collective_zero_thrust_pct), 1.5f);

  ★★★ 注意指数 **1.5** —— 这不是随便取的：

      旋翼的【悬停功率】 P ∝ W^1.5     （W = 重量/拉力）
      而反扭矩 ∝ 功率 / 转速
      => **反扭矩 ∝ 总距^1.5**（转速不变时）

  ⇒ 所以这个前馈项是把「总距变化引起的反扭矩变化」提前补掉，
     **不用等尾桨的反馈环去修。**

  ★ 这是 03b 章第一个「**物理定律直接写进前馈项**」的例子 ——
     和 04 课 §6.2.1 的 qbar ∝ V² 是同一类（物理量 → 前馈）。
""")


# ===========================================================================
# 实验 6  ★★ RSC
# ===========================================================================

def exp6():
    head("实验 6  ★★ RSC：转速控制")

    sub("(a) 为什么直升机需要一个独立的转速控制器")

    print("""  ★ 多旋翼的"转速"就是油门 —— 飞控直接给，不需要额外的东西。

  ★★ 直升机不是：**主旋翼的转速必须【基本恒定】。**

      周期变距和总距的**气动效率都正比于转速**：
          T ∝ (Omega * pitch) 量级
      转速一变，同样的总距就给出不同的拉力
      => 控制和转速【强耦合】

  ⇒ 所以直升机把转速**单独拿出来**，用一个 RSC（Rotor Speed Control）闭环管理。
""")

    sub("(b) 源码：5 种模式、3 个状态")

    print("""  AP_MotorsHeli_RSC.h:34-40（原文）：

      enum RotorControlMode {
          DISABLED = 0,
          PASSTHROUGH,      // 直通（手动油门）
          SETPOINT,         // 定速（用一个油门曲线维持目标转速）
          THROUGHCURVE,     // 油门曲线（随总距查表）
          AUTOTHROTTLE      // 自动油门（调速器）
      };

  AP_MotorsHeli_RSC.h:57-61（原文）：

      enum class RotorControlState {
          STOP = 0,
          IDLE,
          ACTIVE
      };

  ★★ 注意 **PASSTHROUGH 到 AUTOTHROTTLE 是一条"自动化程度递增"的谱**：

      PASSTHROUGH   : 飞手直接控油门（像固定翼）
      SETPOINT      : 维持一个固定转速（最简单）
      THROUGHCURVE  : 按总距查表给油门（**开环前馈**）
      AUTOTHROTTLE  : 闭环调速器（**反馈**）

  ⇒ **这四种正好覆盖了「开环 -> 前馈 -> 反馈」的全部控制手段。**
""")

    sub("(c) ★★ 转速斜坡：上升慢、下降快")

    print("""  AP_MotorsHeli_RSC.cpp:390-428（原文节选）：

      _rotor_ramp_output += (dt / bailout_time);   // 逃逸时（:415）
      _rotor_ramp_output += (dt / ramp_time);      // 正常（:417）
      ...
      } else {
          // ramping down happens instantly
          _rotor_ramp_output = rotor_ramp_input;   // :426
      }

  ★★★ 注意这个**不对称**：

      加速：线性斜坡，用 `ramp_time`（默认 1 s）或 `bailout_time`
      减速：**瞬时**

  ★ 物理原因：**加速受发动机/电机功率限制**（有惯性、有扭矩上限），
     而**减速不受限** —— 关油门就行了。

  ★★ 这也是「执行器有方向性」的又一个例子
     （05 课 §7.7 的缆绳、本章 §1.3 的舵机，都是同一类）。
""")

    sub("(d) ★ 调速器的三个输入")

    print("""  AP_MotorsHeli_RSC.cpp:548-556（原文节选）：

      float governor_droop = ((float)_governor_rpm - _rotor_rpm)
                           * _governor_droop_response * 0.0001f;
      _governor_output = governor_droop
                       + ((throttlecurve - _governor_torque_reference)
                          * _governor_ff * 0.01);
      ...
      _control_output = constrain_float((_governor_torque_reference + _governor_output),
                                        (get_idle_output() * 1.5f), 1.0f);

  ★★ 拆成三项（这是标准的**调速器结构**）：

      %-26s %-34s %s
      %-26s %-34s %s
      %-26s %-34s %s
      %-26s %-34s %s
""" % ("项", "公式", "作用",
       "droop（下垂）", "(目标转速 - 实测转速) x 增益", "**反馈**：转速掉了就加油门",
       "torque_reference", "随旋翼负载缓慢调整的基准", "**积分**：补偿持续的负载变化",
       "前馈", "throttlecurve x 前馈增益", "**前馈**：按总距提前给油门"))
    print("""
  ★★★ **比例 + 积分 + 前馈** —— 这和 05 课 §7.9 那套没有本质区别，
     只是被控对象从"飞机姿态"换成了"旋翼转速"。

  ⇒ 控制理论的通用性在这里又出现了一次。
""")


# ===========================================================================

EXPERIMENTS = {
    "1": ("★★★ 斜盘与混控矩阵是同一个数学", exp1),
    "2": ("★★★ 6 种斜盘类型的几何", exp2),
    "3": ("★★★ 周期变距：力矩与推力", exp3),
    "4": ("★★ 斜盘行程与舵机线性化", exp4),
    "5": ("★★ 双旋翼的差动策略", exp5),
    "6": ("★★ RSC 转速控制", exp6),
}


def main():
    print()
    hr("#")
    print("#  卷 1 第 11 章 · 直升机：斜盘动力学")
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
