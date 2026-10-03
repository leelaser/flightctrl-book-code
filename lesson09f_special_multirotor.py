#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 9.10 周 · 01 特殊多旋翼：三旋翼 / 单旋翼 / V 尾多旋翼 —— 配套实验

    实验 1  ★★★ 三旋翼：yaw 通道【不占用】推力行程
    实验 2  ★★★ 倾转的代价：cos 补偿 + 一个没被建模的侧向力
    实验 3  ★★  三旋翼的力与力矩（含倾转）
    实验 4  ★★  单旋翼：4 片滑流舵面的混控
    实验 5  ★★★ 舵效的两种速度源：滑流 vs 来流
    实验 6  ★★  VTAIL / ATAIL / Y4 的矩阵

用法：
    python lesson09f_special_multirotor.py 1
    ...
    python lesson09f_special_multirotor.py      # 全部

只需 numpy（画图需要 matplotlib）。

参考源码（ArduPilot Copter-4.5.7）：
    libraries/AP_Motors/AP_MotorsTri.cpp:141-281   ★★★ 三旋翼的混控（硬编码 3 行）
    libraries/AP_Motors/AP_MotorsTri.cpp:163       yaw_thrust = yaw*sin(max_deg)
    libraries/AP_Motors/AP_MotorsTri.cpp:177       _pivot_angle = asin(yaw_thrust)
    libraries/AP_Motors/AP_MotorsTri.cpp:184       pivot_thrust_max = cos(pivot)
    libraries/AP_Motors/AP_MotorsTri.cpp:274       _thrust_rear /= cos(pivot)
    libraries/AP_Motors/AP_MotorsTri.cpp:345-361   get_roll_factor（只给 boost 用）
    libraries/AP_Motors/AP_MotorsTri.h:12-13       尾舵机角度上下限 5~80 度
    libraries/AP_Motors/AP_MotorsSingle.cpp:177-183  4 片舵面的混控
    libraries/AP_Motors/AP_MotorsSingle.cpp:226-232  滑流舵效模型
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
# 公共
#
# ★ 三旋翼的执行器与多旋翼【不同】：
#     3 个电机（MOT_1 右 / MOT_2 左 / MOT_4 后）
#     + 1 个尾舵机（CH_7），把【后电机整个转过去】
#
#   所以它的"混控"不是一个矩阵，而是：
#     ① roll/pitch 决定 3 个电机的推力（硬编码 3 行）
#     ② yaw 决定 1 个舵机角度
# ===========================================================================

TRI_SERVO_MAX_DEG = 80.0      # 源码 AP_MotorsTri.h:13 的上限
TRI_SERVO_MIN_DEG = 5.0       # 源码 AP_MotorsTri.h:12 的下限


def tri_pivot_angle(yaw_in, max_deg=TRI_SERVO_MAX_DEG, have_servo=True):
    """复现 AP_MotorsTri.cpp:163-182

        yaw_thrust = yaw_in * sin(max_deg)
        pivot_angle = asin(yaw_thrust)          （再按 ±max_deg 限幅）
    """
    if not have_servo:
        return 0.0
    yaw_thrust = yaw_in * math.sin(math.radians(max_deg))
    yaw_thrust = max(-1.0, min(1.0, yaw_thrust))
    ang = math.asin(yaw_thrust)
    lim = math.radians(max_deg)
    return max(-lim, min(lim, ang))


def tri_mix(roll, pitch, yaw, max_deg=TRI_SERVO_MAX_DEG, have_servo=True):
    """复现 AP_MotorsTri.cpp:163-206 + :274，返回 (右, 左, 后, 舵角, 补偿后推力)"""
    ph = tri_pivot_angle(yaw, max_deg, have_servo)
    # :204-206
    tr = roll * -0.5 + pitch * 0.5
    tl = roll * 0.5 + pitch * 0.5
    tre = pitch * -0.5
    # :274  cos 补偿（只在有舵机时）
    tre_comp = tre / math.cos(ph) if abs(math.cos(ph)) > 1e-9 else tre
    return tr, tl, tre, ph, tre_comp


# ===========================================================================
# 实验 1  ★★★ 三旋翼：yaw 不占推力行程
# ===========================================================================

def exp1():
    head("实验 1  ★★★ 三旋翼：yaw 通道【不占用】推力行程")

    sub("(a) 源码：yaw 只进舵机，不占推力")

    print("""
  AP_MotorsTri.cpp:163-182（原文节选）：

      yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain
                   * sinf(radians(_yaw_servo_angle_max_deg));
      ...
      _pivot_angle = safe_asin(yaw_thrust);
      if (fabsf(_pivot_angle) > radians(_yaw_servo_angle_max_deg)) {
          limit.yaw = true;
          _pivot_angle = constrain_float(_pivot_angle, ...);
      }

  AP_MotorsTri.cpp:204-206（原文）—— 三个电机的推力分配：

      _thrust_right = roll_thrust * -0.5f + pitch_thrust * 0.5f;
      _thrust_left  = roll_thrust *  0.5f + pitch_thrust * 0.5f;
      _thrust_rear  = pitch_thrust * -0.5f;

  ★★★ 请仔细看第 206 行：**yaw 根本没出现在这三个式子里。**
""")

    sub("(b) ★★★ 与四旋翼的根本区别")

    print("""  02 课 §4.4.2 讲过一个事实：四旋翼的 yaw 通道【要和油门/roll/pitch
  抢同一个 ±0.5 行程】—— 这就是 MOT_YAW_HEADROOM 参数的由来。

  ★ 三旋翼没有这个问题，因为 **yaw 有自己的执行器（舵机）**：

      四旋翼： 4 个电机  ->  4 个量（推力 + 3 轴力矩）
                yaw 靠【反扭矩】，而反扭矩和推力出自同一个电机
                => yaw 必须和别的通道【抢行程】

      三旋翼： 3 个电机 + 1 个舵机  ->  4 个量
                yaw 靠【舵机倾转】，舵机行程和电机行程【互不挤占】
                => yaw 有独立的行程预算
""")
    print("      %-14s %-16s %-16s %s" % ("机型", "yaw 执行器", "yaw 占推力行程吗", "代价"))
    print("      " + "-" * 68)
    print("      %-14s %-16s %-16s %s" % ("四旋翼", "电机反扭矩", "★ 占（要留 headroom）", "推力/力矩折中"))
    print("      %-14s %-16s %-16s %s" % ("三旋翼", "尾舵机倾转", "不占", "★ 尾电机推力损失"))

    sub("(c) 定量：倾转角与 yaw 指令的关系")

    print("  ★ 源码用 `yaw_thrust = yaw*sin(max)`，再 `pivot = asin(yaw_thrust)`。")
    print("    这不是线性映射 —— 看表（max = %.0f 度）：\n" % TRI_SERVO_MAX_DEG)
    print("      %-12s %-16s %-16s %s" % ("yaw 指令", "yaw_thrust", "倾转角", "倾转角/最大角"))
    print("      " + "-" * 62)
    for y in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]:
        yt = y * math.sin(math.radians(TRI_SERVO_MAX_DEG))
        ph = tri_pivot_angle(y)
        print("      %-12.2f %-16.4f %-16.2f %.3f"
              % (y, yt, math.degrees(ph), math.degrees(ph) / TRI_SERVO_MAX_DEG))
    print("""
  ★★ 关键设计意图（源码 :163 的注释说得很清楚）：

      「we scale this so a thrust request of 1.0f will ask for
        full servo deflection at full rear throttle」

  => **yaw 指令 1.0 对应满舵**，而 yaw_thrust 这个中间量
     本来就叫「推力」—— 因为它【就是】尾电机的侧向分量。
""")


# ===========================================================================
# 实验 2  ★★★ 倾转的代价
# ===========================================================================

def exp2():
    head("实验 2  ★★★ 倾转的代价：cos 补偿 + 一个没被建模的侧向力")

    sub("(a) 尾电机的力分解")

    print("""  后电机被舵机转过 phi 角后，它的推力方向是（FRD）：

        u_rear = (0, sin(phi), -cos(phi))

  ★ 分解成两个分量：

        垂直分量 = T_rear * cos(phi)     <- 这份才是"升力"
        侧向分量 = T_rear * sin(phi)     <- ★ 这份【产生 yaw 力矩】（力臂 = 后臂长）
""")

    sub("(b) 源码的补偿：只补了垂直分量")

    print("""  AP_MotorsTri.cpp:184  （原文）
      float pivot_thrust_max = cosf(_pivot_angle);

  AP_MotorsTri.cpp:274  （原文）
      _thrust_rear = _thrust_rear / cosf(_pivot_angle);

  ★ 意图：把推力【放大 1/cos】，让垂直分量保持原来的值。

  ★★★ 但请注意：**补偿之后的侧向分量也一起被放大了**：

        补偿前  侧向 = T_rear * sin(phi)
        补偿后  侧向 = (T_rear/cos(phi)) * sin(phi) = T_rear * tan(phi)

  => 侧向力 ∝ **tan(phi)**，而不是 sin(phi)。
""")

    sub("(c) ★★★ 那张表：侧向力有多大")

    T = 1.0
    print("  设后电机【补偿前】的推力指令 T = %.1f：\n" % T)
    print("      %-11s %-11s %-13s %-13s %-13s %s"
          % ("yaw 指令", "倾转角", "推力放大", "垂直分量", "侧向分量", "侧向/垂直"))
    print("      " + "-" * 76)
    for y in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]:
        ph = tri_pivot_angle(y)
        c, s = math.cos(ph), math.sin(ph)
        tcomp = T / c
        vert = tcomp * c
        lat = tcomp * s
        print("      %-11.2f %-11.2f %-13.4f %-13.4f %-13.4f %.3f"
              % (y, math.degrees(ph), 1.0 / c, vert, lat, lat / vert))
    print("""
  ★★ 读法：
      · **垂直分量恒等于 1.0** —— 说明 cos 补偿是**有效**的 ✓
      · **侧向分量随 yaw 指令增长** —— yaw = 1.0 时达到 0.985（与垂直分量同量级！）

  ★★★ 而这个侧向力**在混控里完全没有被处理**：
     它既不进 roll 行，也不进 pitch 行。
""")

    sub("(d) 那它会怎样？")

    print("""  ★ 侧向力作用在机体上，等效于一个**横向扰动**.

  ⚠️ 但**不会失稳** —— 因为姿态环是闭环的：

      侧向力 -> 飞机被推偏 -> 姿态环检测到 -> 用 roll/pitch 修正

  ★ 所以这是一个**性能问题**（姿态环要额外干活），
    不是**稳定性问题** —— 和 02 课 §4.3.4 对 OCTA V 串扰的定位完全一样。

  ★★ 真实的后果有两个：
      ① 打 yaw 会伴生一点横移（闭环能修，但会滞后）
      ② ★ 大 yaw 时尾电机推力被放大 1/cos(80°) = 5.76 倍 ——
         **尾电机可能先饱和**

  ⚠️ 第 ② 条更实际：源码 :184 算了 `pivot_thrust_max`，
     后面用它做了饱和检查（本课不展开）。
""")


# ===========================================================================
# 实验 3  ★★ 三旋翼的力与力矩
# ===========================================================================

def exp3():
    head("实验 3  ★★ 三旋翼的力与力矩（含倾转）")

    sub("(a) 源码隐含的几何")

    print("""  ★ 把 :204-206 的三个式子对照 01 课 §3.4 的约定
    （roll_i = -y_i/R, pitch_i = x_i/R），反推电机位置：
""")
    print("      %-10s %-14s %-14s %-16s %s" % ("电机", "roll 系数", "pitch 系数", "推出 (x/R, y/R)", "方位角"))
    print("      " + "-" * 76)
    for name, r, p in [("MOT_1 右", -0.5, 0.5), ("MOT_2 左", 0.5, 0.5), ("MOT_4 后", 0.0, -0.5)]:
        xr, yr = p, -r
        az = math.degrees(math.atan2(yr, xr))
        print("      %-10s %-14.1f %-14.1f (%+.2f, %+.2f)      %+.0f°"
              % (name, r, p, xr, yr, az))
    print("""
  ★★★ 读出来是一个**前臂张角 90 度**（±45°）的 Y 形布局，
     而不是经典三旋翼的 120°（±30°）。
""")

    sub("(b) ★ 一个值得注意的差异（不是 bug，是约定）")

    print("""  ★ 同一条约定下，**经典 120° 三旋翼**的系数应该是：

        前右（方位 -30° 侧）：roll = -sin(-30°)... 按 120° 张角算：
            前右 (x,y) = (0.5R, 0.866R) -> roll = -0.866, pitch = +0.5
            前左 (x,y) = (0.5R,-0.866R) -> roll = +0.866, pitch = +0.5
            后   (x,y) = (-R, 0)        -> roll =  0,     pitch = -1.0

    而 AP 的代码是：

        roll = ∓0.5        （vs 120° 的 ∓0.866，比值 0.577）
        pitch = 0.5/0.5/-0.5（vs 120° 的 0.5/0.5/-1.0）

  => **roll/pitch 的【比值】不同：AP 是 1.0，120° 布局是 1.732。**

  ⚠️ 本课**不判断**这是"源码简化"还是"AP 支持的确实是 90° 布局"——
     因为这需要一份权威的机型文档才能定论。
     ★ 这里只把**事实**摆出来：**代码里的系数对应 90° 张角。**

  ★★ 而这件事的**教材价值**在于：

      它说明「三旋翼」不是一个单一定义的机型。
      多旋翼有 X/+/V/H 等明确的 frame type 枚举，
      而 **Tri 只有一种**（`MOTOR_FRAME_TYPE_X` 等对 Tri 无意义）
      —— 它的几何是【写死在代码里的】。
""")

    sub("(c) 力矩：三个电机 + 一个舵机")

    print("""  ★ 汇总（FRD，机体系）：

        推力   F_z = -(T_右 + T_左 + T_后_补偿)
        滚转   τ_x = -Σ y_i T_i        （后电机在 y=0，不参与）
        俯仰   τ_y = +Σ x_i T_i
        偏航   τ_z = ★ **尾舵机倾转的侧向力 × 后臂长**
                  = T_后_补偿 * sin(phi) * R_后

  ★★★ 注意 τ_z 这一项的**结构**和前三课【完全不同】：

      多旋翼：τ_z = Σ k_m ω_i² σ_i        （反扭矩，与位置无关）
      三旋翼：τ_z = 侧向力 × 力臂          （气动力，与位置【有关】）

  => 这是 03b 章的第一个"不同的 yaw 来源"。02 课（直升机）会有第三个。
""")


# ===========================================================================
# 实验 4  ★★ 单旋翼：4 片滑流舵面
# ===========================================================================

def exp4():
    head("实验 4  ★★ 单旋翼：4 片滑流舵面的混控")

    sub("(a) 源码（AP_MotorsSingle.cpp:177-183，原文）")

    print("""
      actuator[0] =  rp_scale * roll_thrust  - yaw_thrust;   // front servo
      actuator[1] =  rp_scale * pitch_thrust - yaw_thrust;   // right servo
      actuator[2] = -rp_scale * roll_thrust  - yaw_thrust;   // rear servo
      actuator[3] = -rp_scale * pitch_thrust - yaw_thrust;   // left servo

  ★★★ 这是 03b 章里最干净的一个混控矩阵。写成矩阵（行 = 舵面）：

      [front]   [ +1   0  -1 ] [roll ]
      [right] = [  0  +1  -1 ] [pitch]
      [rear ]   [ -1   0  -1 ] [yaw  ]
      [left ]   [  0  -1  -1 ]
""")

    sub("(b) ★★★ 三条读数")

    print("""  ① **俯仰/滚转用【相对的两片】**：
       front/rear 做 roll，right/left 做 pitch —— 差动。

  ② ★★★ **yaw 用【全部四片同向偏转】**：
       四片的 yaw 系数【都是 -1】。

     ★ 为什么？因为它们是**舵面**，不是桨！

       桨  ：靠【反扭矩】产生 yaw —— 需要旋向不同
       舵面：靠【滑流偏转】产生 yaw —— 四片同向偏转，滑流被整体扭转
             反作用力矩就是 yaw 力矩

     => **同一组四个执行器，yaw 的产生机理和 pitch/roll 完全不同。**

  ③ **四个舵面的力矩都正比于同一个量**（滑流速度² = 推力），
     所以混控里它们可以【共用一个缩放】。
""")

    sub("(c) ★★ 滑流舵效：源码的注释说得很清楚")

    print("""  AP_MotorsSingle.cpp:226-232（原文注释）：

      // force of a lifting surface is approximately equal to the angle of
      // attack times the airflow velocity squared
      // static thrust is proportional to the airflow velocity squared
      // therefore the torque of the roll and pitch actuators should be
      // approximately proportional to the angle of attack multiplied by
      // the static thrust.
      for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
          _actuator_out[i] = constrain_float(rp_scale * actuator[i] / thrust_out_actuator, -1.0f, 1.0f);
      }

  ★ 翻译一下这个推导链：

      升力面受力  ∝ 攻角 × 气流速度²
      静推力      ∝ 气流速度²            <- ★ 这是动量理论（01 课 §3.7.1）
      ------------------------------------------
      => 舵面力矩 ∝ 攻角 × 静推力

  ★★★ 所以**归一化要除以推力** —— 这就是 `/ thrust_out_actuator` 的含义。

  => **舵面偏角 = 期望力矩 / 当前推力**
     推力越大，同样力矩需要的舵面偏角越**小**。
""")

    sub("(d) 数值：舵效随油门变化")

    print("      %-12s %-16s %-16s %s" % ("油门(相对)", "滑流速度(相对)", "舵效(相对)", "同样力矩需舵面偏角"))
    print("      " + "-" * 68)
    for thr in [0.2, 0.4, 0.6, 0.8, 1.0]:
        v = math.sqrt(thr)
        eff = thr
        print("      %-12.2f %-16.3f %-16.3f %.3f"
              % (thr, v, eff, 1.0 / eff))
    print("""
  ★★ 对比 04 课（固定翼 V 尾）：

      固定翼：舵效 ∝ V_来流²       <- 来流是【外部】的，飞控管不了
      单旋翼：舵效 ∝ 推力（转速²） <- 滑流是【自己】产生的，飞控知道

  ★★★ **结构完全相同，速度源不同。**
     这个对比是 03b 章的一条主线：**「舵效从哪里来」决定了能不能前馈补偿。**
""")


# ===========================================================================
# 实验 5  ★★★ 舵效的两种速度源
# ===========================================================================

def exp5():
    head("实验 5  ★★★ 舵效的两种速度源：滑流 vs 来流")

    sub("(a) 同一个公式，两个不同的输入")

    print("""  04 课 §6.2.1 说固定翼的气动力是：

        L = qbar * S * C_L(alpha, ...),    qbar = 0.5 * rho * V²

  实验 4 说单旋翼的舵面力矩是：

        M ∝ 攻角 × 推力

  ★★ 把两式并排：

      %-22s %-26s %s
      %-22s %-26s %s
""" % ("固定翼（V 尾）", "0.5*rho*V_inf^2 * S", "★ 来流 —— 外部，飞控测不到",
       "单旋翼（SINGLE）", "推力 = k_f * omega^2", "★ 滑流 —— 自己产生，飞控知道"))
    print("""
  ★★★ 这个差别有**直接的工程后果**：

      固定翼：舵效随空速平方变 —— 飞控**必须**用空速传感器测 V，才能前馈补偿
      单旋翼：舵效正比于推力   —— 飞控**本来就知道**推力（它自己下的指令）
                                 => 可以直接除，不需要额外传感器
""")


# ===========================================================================
# 实验 6  ★★ VTAIL / ATAIL / Y4
# ===========================================================================

def exp6():
    head("实验 6  ★★ VTAIL / ATAIL / Y4 的矩阵")

    sub("(a) 这三个机型藏在 QUAD class 里")

    print("""  00 课 §0.4.3 说过：frame class 按【执行器数量/类型】分，
  不按几何形状分。所以这三个都在 `setup_quad_matrix` 里：

      AP_MotorsMatrix.cpp:700   MOTOR_FRAME_TYPE_VTAIL
      AP_MotorsMatrix.cpp:723   MOTOR_FRAME_TYPE_ATAIL
      AP_MotorsMatrix.cpp:755   MOTOR_FRAME_TYPE_Y4
""")

    sub("(b) ★★★ VTAIL 的电机表：4 个电机，两个角度分开给")

    print("""  ★ 源码用的是 `add_motor(motor_num, roll_deg, pitch_deg, yaw_factor, order)`
    —— **roll 和 pitch 两个角度分开给**，这是矩阵型里少见的写法。

  AP_MotorsMatrix.cpp:716-720（VTAIL 原文，完整 4 行）：

      _frame_type_string = "VTAIL";
      add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
      add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
      add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
      add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);

  ★★★ 逐行展开成系数（roll = cos(度+90)，pitch = cos(度)，见 01 课 §3.4）：
""")
    print("      %-10s %-10s %-10s %-13s %-13s %-12s %s"
          % ("电机", "roll 度", "pitch 度", "roll 系数", "pitch 系数", "yaw 系数", "角色"))
    print("      " + "-" * 82)
    rows = [
        ("MOT_1", 60, 60, 0, "前（参与 roll/pitch）"),
        ("MOT_2", 0, -160, -1, "★ 后（只参与 yaw）"),
        ("MOT_3", -60, -60, 0, "前（参与 roll/pitch）"),
        ("MOT_4", 0, 160, +1, "★ 后（只参与 yaw）"),
    ]
    for nm, rd, pd, yf, role in rows:
        rf = math.cos(math.radians(rd + 90))
        pf = math.cos(math.radians(pd))
        ys = {0: "0", -1: "CW(-1)", 1: "CCW(+1)"}[yf]
        print("      %-10s %-10d %-10d %-13.4f %-13.4f %-12s %s"
              % (nm, rd, pd, rf, pf, ys, role))
    print("""
  ★★★ 三条关键读数：

    ① **前两个电机**（MOT_1/MOT_3）在 (±0.866, 0.5)，负责 **roll 和 pitch**
       —— 它们的 yaw 系数是 **0**。

    ② ★★★ **后两个电机**（MOT_2/MOT_4）的 roll 系数是 **0**、pitch 系数是 **-0.940**
       —— 也就是它们**在中心线上**（y = 0，所以不产生 roll）。

    ③ ★★★ 那它们怎么产生 yaw？**靠 yaw 系数 ±1，而不是靠位置。**

  ⇒ 因为后两个电机是【上反】的 —— 它们的推力方向被掰过，
     侧向分量 × 力臂 = yaw 力矩。
     **ArduPilot 把这个几何效应【编码成了一个 yaw 系数】，
     而不是真的去建模推力方向。**
""")

    sub("(c) ★★★ 源码注释里的 40° 就是 04 课的 Gamma")

    print("""  AP_MotorsMatrix.cpp:702-714（原文注释，节选）：

      Tested with: Lynxmotion Hunter Vtail 400
      - inverted rear outward blowing motors (at a **40 degree angle**)
      ...
      - no roll in rear motors
      - no yaw in front motors
      ...
      Roll control comes only from the front motors, Yaw control only from the rear motors.
      ...
      Note: if we want the front motors to help with yaw,
          motors 1's yaw factor should be changed to **sin(radians(40))**.
          Where "40" is the vtail angle

  ★★★ 看最后那两行 —— **`sin(radians(40))`，而注释明确说
     「40」就是 vtail angle。**

  ★★ 对照 04 课 §6.3.2（固定翼 V 尾）：

      M = -l_t * cos(Gamma) * (L_L + L_R)
      N = -l_t * sin(Gamma) * (L_L - L_R)

  ⇒ **同一个 Gamma，同一个 sin。**

  ★★★ 两处是**同一个几何**：
     「两个力源上反 Gamma 角」⇒ 俯仰用 cos、偏航用 sin。

      固定翼：力源是【气动面】 -> 力 = 气动力
      多旋翼：力源是【电机推力】 -> 力 = 推力

  ★ 差别只在于「力源是什么」，几何分解**一模一样**。

  ⚠️ 而 ArduPilot 在多旋翼这边**偷了懒**：它没有像固定翼那样
     写 cos/sin 分解，而是**直接把 sin(40°) 的效应塞进 yaw 系数**，
     并且**完全忽略**了 cos(40°) 对俯仰的影响（后电机的 pitch 系数
     用的是 cos(160°) = -0.940，而不是 cos(40°) 相关的量）。
""")

    sub("(d) ATAIL：几何镜像")

    print("""  AP_MotorsMatrix.cpp:723-742（原文注释节选）：

      The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
      - The Yaw factors are **reversed**, because the rear motors are facing different directions
      ...
      - Rear thrust on a V-Shaped V-Tail Quad is **outward**
      - Rear thrust on an A-Shaped V-Tail Quad is **inward**

  ★★ 对照代码（:738-741）：**只有两个 yaw 因子的符号反了**，其余完全相同。

      VTAIL: MOT_2 = CW(-1), MOT_4 = CCW(+1)
      ATAIL: MOT_2 = CCW(+1), MOT_4 = CW(-1)

  ⇒ 这正是 04 课 §6.3.2 的那句「两个面的差」的符号 ——
     **V 形和 A 形的区别就是 $(L_L - L_R)$ 的符号。**
""")


    print("""
  ★★★ 而且它们的**物理机理相反**：

      多旋翼 VTAIL：后电机【上反】-> 推力带横向分量 -> 产生 yaw
                     ★ 是**推力矢量**（04 课 §6.3 的 cos/sin 分解同样的几何）

      固定翼 V-Tail：舵面【上反】-> 气动力带横向分量 -> 产生 yaw
                     ★ 是**气动力**（04 课 §6.3 推导过）

  ★★ 几何完全一样（都是"两个面/推力源上反 Γ"），
     但一个在【推力域】、一个在【气动域】。

  => 这正是 04 课 §6.3.2 那两个公式
     （M ∝ cosΓ·(L_L+L_R)、N ∝ sinΓ·(L_L−L_R)）
     能【原样】用到多旋翼 VTAIL 上的原因 ——
     因为分的是"两个上反的力源"，与力源是什么无关。
""")


# ===========================================================================

EXPERIMENTS = {
    "1": ("★★★ 三旋翼：yaw 不占推力行程", exp1),
    "2": ("★★★ 倾转的代价与未建模的侧向力", exp2),
    "3": ("★★ 三旋翼的力与力矩", exp3),
    "4": ("★★ 单旋翼：4 片滑流舵面", exp4),
    "5": ("★★★ 舵效的两种速度源", exp5),
    "6": ("★★ VTAIL / ATAIL / Y4", exp6),
}


def main():
    print()
    hr("#")
    print("#  第 9.10 周 · 01 特殊多旋翼：三旋翼 / 单旋翼 / V 尾多旋翼")
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
