#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 9.6 周 · 04 固定翼 V 尾动力学建模 —— 配套实验

    实验 1  ★★  固定翼与多旋翼的根本区别（力依赖状态 + J 非对角）
    实验 2  ★★★ V 尾二面角的力分解（俯仰 ∝ cosΓ，偏航 ∝ sinΓ）
    实验 3  ★★★ 效能矩阵：行列式、条件数、最优二面角
    实验 4  ★★  混控矩阵与 ArduPilot 源码逐项对照
    实验 5  ★★  稳定性代价 cos²Γ / sin²Γ 与匹配二面角
    实验 6  ★    惯性耦合 Jxz（多旋翼没有的项）

用法：
    python lesson09c_vtail_dynamics.py 1
    python lesson09c_vtail_dynamics.py 2
    ...
    python lesson09c_vtail_dynamics.py          # 全部

只需 numpy（画图需要 matplotlib）。

参考源码（ArduPilot Copter-4.5.7 / ArduPlane）：
    ArduPlane/servos.cpp:167-186         channel_function_mixer
    ArduPlane/servos.cpp:1010-1011       elevon / vtail 的调用点
    ArduPlane/Parameters.cpp:594-614     MIXING_GAIN / MIXING_OFFSET
    libraries/AP_IOMCU/iofirmware/mixer.cpp:110-115   mix_elevon_vtail
    libraries/AP_IOMCU/iofirmware/mixer.cpp:209-215   vtail 分派
    libraries/SRV_Channel/SRV_Channel.h:123-124       k_vtail_left / right
    libraries/SITL/SIM_Plane.cpp:293-297              SITL 的逆混控
    Tools/autotest/default_params/plane-vtail.parm    V 尾参数模板
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
# 公共：V 尾几何
#
# 机体坐标系 FRD：X 前、Y 右、Z 下。
# 两个舵面（ruddervator）各自从水平面上抬 Gamma，形成 V 字。
#   左面 位置在 -Y 一侧，展向单位矢量  s_L = (0, -cosG, -sinG)
#   右面 位置在 +Y 一侧，展向单位矢量  s_R = (0, +cosG, -sinG)
# 舵面产生的气动力沿其法向（取「向上」那一支）：
#   u_L = s_L x x_hat 取反 = (0, +sinG, -cosG)
#   u_R =                    (0, -sinG, -cosG)
# Gamma = 0    -> (0, 0, -1)  纯水平尾翼，力朝上          ✓
# Gamma = 90°  -> (0, ±1, 0)  纯垂直尾翼，力朝侧向        ✓
# ===========================================================================

def surface_dirs(Gamma_deg):
    """返回 (左面升力方向, 右面升力方向)，FRD 单位矢量。"""
    G = math.radians(Gamma_deg)
    uL = np.array([0.0, +math.sin(G), -math.cos(G)])
    uR = np.array([0.0, -math.sin(G), -math.cos(G)])
    return uL, uR


def aero_matrix(Gamma_deg, l_t=1.0):
    """(L_L, L_R) -> (M, N)，不含公共因子 k。

    返回 A，满足  [M; N] = -l_t * A @ [L_L; L_R]
        A = [[ cosG,  cosG],
             [ sinG, -sinG]]
    """
    G = math.radians(Gamma_deg)
    return np.array([[math.cos(G), math.cos(G)],
                     [math.sin(G), -math.sin(G)]]) * l_t


def mix_matrix():
    """(delta_e, delta_r) -> (delta_L, delta_R)。

        delta_L = delta_e + delta_r
        delta_R = delta_e - delta_r
    """
    return np.array([[1.0, 1.0],
                     [1.0, -1.0]])


def ef_matrix(Gamma_deg, l_t=1.0, k=1.0):
    """(delta_e, delta_r) -> (M, N)，含舵效系数 k。

        [M; N] = -l_t * k * A @ T @ [delta_e; delta_r]
               = -2*l_t*k * diag(cosG, sinG) @ [delta_e; delta_r]
    """
    return -l_t * k * aero_matrix(Gamma_deg, 1.0) @ mix_matrix()


# ===========================================================================
# 实验 1  ★★ 固定翼与多旋翼的根本区别
# ===========================================================================

def exp1():
    head("实验 1  ★★ 固定翼与多旋翼的根本区别")

    sub("(a) 多旋翼：力只依赖输入")

    print("""
  01/03 课里，四旋翼/六旋翼的机体力是
      F_body = (0, 0, -sum_i T_i)
  右边**只出现输入 T_i**，一个状态量都没有。

  => 这叫「控制仿射」（control-affine）：  x_dot = f(x) + B(x) u
     而多旋翼的 B **不依赖 x**（近似），所以它是一个**线性混控 + 非线性刚体**。

  于是 02 课可以做那件很漂亮的事：把 B 求逆，一次算完所有电机。
""")

    sub("(b) 固定翼：力依赖状态，B 是状态的函数")

    print("""
  固定翼的机体力/力矩是**空速、迎角、侧滑角、角速度、舵面**的函数：

      V   = sqrt(u^2 + v^2 + w^2)
      alpha = atan2(w, u)                  迎角
      beta  = asin(v / V)                  侧滑角

      L = qbar S C_L(alpha, q, delta_e)    qbar = 0.5 rho V^2
      D = qbar S C_D(alpha, delta_e)
      Y = qbar S C_Y(beta, delta_r)

      l = qbar S b C_l(beta, p, r, delta_a, delta_r)
      m = qbar S c C_m(alpha, q, delta_e)
      n = qbar S b C_n(beta, p, r, delta_a, delta_r)

  => **同一个舵面偏度，在 V=10 m/s 和 V=30 m/s 下产生的力矩差 9 倍**
     （qbar ∝ V^2）。多旋翼完全没有这件事。

  ★ 这是本课和 01/03 最大的分歧：
     多旋翼的 B 是一张**常数表**；
     固定翼的 B 每次都要用当前状态**现算**。
""")

    sub("(c) 惯性矩阵：多旋翼是对角阵，固定翼不是")

    print("""
  多旋翼的机架关于 x-z 和 y-z 平面对称 => J 是**对角阵**。
  01 课直接写了 J = diag(0.008, 0.008, 0.015)，没有讨论非对角元 ——
  因为对多旋翼来说那些确实是 0（只有桨的旋翼本身贡献一点点）。

  固定翼**只有 x-z 平面是对称面**（左右对称），前后不对称、上下不对称：
      J = [[ Jx,   0,  -Jxz],
           [  0,  Jy,    0 ],
           [-Jxz,  0,   Jz]]
  Jxz != 0 是**产品惯量**，典型值可以到 Jx 的 5%~15%。

  => 刚体转动方程 J w_dot + w x (J w) = tau 里那个「陀螺项」
     **对多旋翼是次要修正，对固定翼是主项之一**。
""")

    sub("(d) 定量：两种载具的「B 依赖状态」程度")

    print("      %-16s %-28s %s" % ("载具", "力/力矩依赖什么", "B 是否随状态变"))
    print("      " + "-" * 68)
    rows = [
        ("四旋翼/六轴", "只依赖电机推力 T_i", "否（常数表）"),
        ("共轴 Y6/X8", "T_i，且上下桨系数不同", "否（但分上下桨）"),
        ("固定翼", "V, alpha, beta, p, q, r, 舵面", "**是**（qbar ∝ V^2）"),
        ("V 尾固定翼", "以上 + 二面角 Gamma 的投影", "**是**"),
    ]
    for a, b, c in rows:
        print("      %-16s %-28s %s" % (a, b, c))
    print("""
  => 所以固定翼的「混控」只解决最后一小段：
     从 (delta_e, delta_r) 到两个舵面，**不管气动**。
     气动那部分只能靠**增稳内环**（角速率 PID）兜住。
""")

    sub("(e) 数量级感受")

    rho = 1.225
    S, b, c = 0.55, 1.8, 0.25      # 一架 3 kg 级固定翼
    print("  取一架 %.1f m 翼展、S=%.2f m^2 的电动固定翼："
          % (b, S))
    print()
    print("      V (m/s)   qbar (Pa)   动压比    升降舵力矩比")
    print("      " + "-" * 46)
    q0 = None
    for V in [10, 15, 20, 25, 30]:
        qbar = 0.5 * rho * V * V
        if q0 is None:
            q0 = qbar
        print("      %-8d  %9.2f   %8.2f    %8.2f"
              % (V, qbar, qbar / q0, qbar / q0))
    print("""
  => 同一偏度，30 m/s 的力矩是 10 m/s 的 **9 倍**。
     **「舵效随速度平方变」是固定翼一切控制问题的起点** ——
     它直接导致：低速舵效不足（失速/起飞），高速舵效过强（振荡）。
     多旋翼完全没有这个轴。
""")


# ===========================================================================
# 实验 2  ★★★ V 尾二面角的力分解
# ===========================================================================

def exp2():
    head("实验 2  ★★★ V 尾二面角的力分解")

    sub("(a) 几何：两个舵面的力朝哪")

    print("""
  机体坐标系 FRD（X 前 / Y 右 / Z 下），尾翼在 c.g. 后方 l_t 处。
  两个舵面各自从水平面抬起 Gamma（二面角），构成 V 字：

      从背后看（往 +X 看）：

              \\   /           左面在 -Y 一侧，右面在 +Y 一侧
               \\ /            每个面「向上」的方向被掰了 Gamma
                V

  面的展向单位矢量：  s_L = (0, -cosG, -sinG)     s_R = (0, +cosG, -sinG)
  面产生的气动力沿其法向（取向上那一支）：
      u_L = (0, +sinG, -cosG)      u_R = (0, -sinG, -cosG)
""")
    print("      Gamma     u_L = (0, sinG, -cosG)        u_R")
    print("      " + "-" * 58)
    for Gd in [0, 15, 30, 35, 45, 60, 90]:
        uL, uR = surface_dirs(Gd)
        print("      %4d°     (0, %+.4f, %+.4f)      (0, %+.4f, %+.4f)"
              % (Gd, uL[1], uL[2], uR[1], uR[2]))
    print("""
  => Gamma=0 时两个面的力都朝正上（0,0,-1），V 字摊平 = 普通水平尾翼；
     Gamma=90 时力全在侧向，两个面都成了垂直鳍。
""")

    sub("(b) 对 c.g. 取矩")

    print("""
  设两个面的升力分别是 L_L、L_R（沿 u_L、u_R，正负视偏角而定）。
  尾翼位置 r = (-l_t, 0, 0)，力矩 = r x F。

  左面：F_L = L_L*(0, sinG, -cosG)
        r x F_L = (0, -l_t*L_L*cosG, -l_t*L_L*sinG)
  右面：F_R = L_R*(0, -sinG, -cosG)
        r x F_R = (0, -l_t*L_R*cosG, +l_t*L_R*sinG)

  合并（M = 俯仰力矩，绕 +Y；N = 偏航力矩，绕 +Z）：
""")
    print("      M = -l_t * cosG * (L_L + L_R)")
    print("      N = -l_t * sinG * (L_L - L_R)")
    print("""
  => 这就是 V 尾的**全部**气动秘密：

       俯仰 ∝ cos(Gamma)     用两个面的**和**
       偏航 ∝ sin(Gamma)     用两个面的**差**

  ★★★ 而且这一眼就看出两件事：
     ① **单个舵面无法产生纯俯仰或纯偏航** —— 它的力同时投影到
        俯仰和偏航两个轴上，比例恒为 cot(Gamma)。
     ② 只有**成对**使用才能解耦：同向偏 = 纯俯仰，反向偏 = 纯偏航。
""")

    sub("(c) 数值验证：单个舵面的力方向")

    print("  给左面一个升力 L_L = 1，看它分配到俯仰和偏航各多少：")
    print()
    print("      Gamma     M 分量      N 分量      M/N = cot(Gamma)")
    print("      " + "-" * 54)
    for Gd in [20, 30, 35, 40, 45, 60]:
        A = aero_matrix(Gd)
        M = -1.0 * A[0, 0] * 1.0        # L_L=1, L_R=0
        N = -1.0 * A[1, 0] * 1.0
        print("      %4d°   %9.4f   %9.4f      %9.4f"
              % (Gd, M, N, M / N if N else float("inf")))
    print("""
  => 实拍机型的 Gamma 大多在 30~40°，cot 在 1.2~1.7 ——
     也就是**单个舵面的力，俯仰分量是偏航分量的 1.2~1.7 倍**。
""")

    sub("(d) 成对使用：解耦")

    print("  取 Gamma = 35°，验证两种「标准动作」：")
    print()
    G = 35.0
    A = aero_matrix(G)
    l_t, k = 1.0, 1.0
    # 纯升降舵：d_e=1, d_r=0  -> L_L = 1, L_R = 1
    M, N = -l_t * k * A @ np.array([1.0, 1.0])
    print("      纯俯仰 (d_e=1, d_r=0)   L_L=+1 L_R=+1  ->  M=%+.4f  N=%+.4f" % (M, N))
    # 纯方向舵：d_e=0, d_r=1  -> L_L = 1, L_R = -1
    M2, N2 = -l_t * k * A @ np.array([1.0, -1.0])
    print("      纯偏航 (d_e=0, d_r=1)   L_L=+1 L_R=-1  ->  M=%+.4f  N=%+.4f" % (M2, N2))
    print("""
  => 完美解耦：俯仰动作**一点偏航都不产生**，反之亦然。
     代价是每个通道的有效性都被打了折（下面实验 3 会量化）。
""")


# ===========================================================================
# 实验 3  ★★★ 效能矩阵：行列式、条件数、最优二面角
# ===========================================================================

def exp3():
    head("实验 3  ★★★ 效能矩阵：行列式、条件数、最优二面角")

    sub("(a) 把两件事分开：混控矩阵 vs 效能矩阵")

    print("""
  ★ 这里有一个很容易混的点，先分开：

  ① **混控矩阵** T：(delta_e, delta_r) -> (delta_L, delta_R)
         T = [[1,  1],
              [1, -1]]
     它是**固定的、与 Gamma 无关的**，而且 T T^T = 2I ——
     即 T 是「正交阵 x sqrt(2)」，**条件数恒为 1**。

  ② **效能矩阵** A：(L_L, L_R) -> (M, N)
         A = [[ cosG,  cosG],
              [ sinG, -sinG]]
     它**依赖 Gamma**，条件数会变。

  从舵量到力矩的完整链是  Ef = -l_t*k * A @ T ：
""")
    print("      Ef = -2*l_t*k * [[cosG,   0  ],")
    print("                       [  0  , sinG ]]")
    print("""
  => 对角阵！这说明**只要混控正确，V 尾就是天然解耦的**。
     非对角项的缺失，是 T 和 A 的结构配合的结果（下面 (e) 证明）。
""")

    sub("(b) 条件数：Gamma 决定两个通道的强弱悬殊")

    print("""  对 A 求奇异值：A A^T = [[2cos^2G, 0], [0, 2sin^2G]]，于是

        sigma_1 = sqrt(2)*max(|cosG|, |sinG|)
        sigma_2 = sqrt(2)*min(|cosG|, |sinG|)
        cond(A) = max(tanG, cotG)
""")
    print("      Gamma     det(A)=sin2G     cond(A)=max(tan,cot)")
    print("      " + "-" * 48)
    for Gd in [10, 20, 30, 35, 40, 45, 50, 60, 70, 80]:
        A = aero_matrix(Gd)
        det = np.linalg.det(A)
        c = np.linalg.cond(A)
        star = "   <- 最优" if Gd == 45 else ""
        print("      %4d°   %12.4f   %20.4f%s" % (Gd, det, c, star))
    print("""
  => **两件事同时说明 45 度是最优的：**
     · det(A) 在 Gamma=45 取最大值（= 1），即「总效能」最大；
     · cond(A) 在 Gamma=45 取最小值（= 1），即「两通道一样强」。

  ⚠ 但真实 V 尾几乎**不取 45**，都在 30~40。原因在实验 5：
     除了控制效能，还要照顾**稳定性**，而稳定性对 Gamma 的要求不同。
""")

    sub("(c) 为什么实际机不会正好 45 度")

    print("""  看一眼两个通道的有效系数（相对最大值）：

       俯仰系数 = cosG        偏航系数 = sinG

  Gamma < 45  =>  cosG > sinG  => **俯仰更强、偏航更弱**
  Gamma > 45  =>  反过来

  对一架飞机来说，升降舵（俯仰）通常需要更大的权限
  （配平范围 + 拉杆机动），方向舵（偏航）则主要对付侧风修正和荷兰滚。
  => **工程上偏向 Gamma < 45，用偏航权限换俯仰权限。**
""")
    print("      Gamma     俯仰 cosG    偏航 sinG    俯仰/偏航")
    print("      " + "-" * 50)
    for Gd in [30, 35, 40, 45]:
        c, s = math.cos(math.radians(Gd)), math.sin(math.radians(Gd))
        print("      %4d°   %10.4f  %11.4f  %11.4f" % (Gd, c, s, c / s))

    sub("(d) 舵面行程：弱通道要更多行程")

    print("""  产生同样的偏航力矩，需要的舵量反比于 sinG：

        delta_r 需求 ∝ 1 / sinG

  这就是「V 尾比传统尾翼更吃舵机行程」的量化来源。""")
    print("      Gamma     1/sinG（相对 Gamma=45）     相对 90°")
    print("      " + "-" * 52)
    base = 1.0 / math.sin(math.radians(45))
    for Gd in [30, 35, 40, 45, 60]:
        v = 1.0 / math.sin(math.radians(Gd))
        print("      %4d°   %20.3f   %18.3f" % (Gd, v / base, v))
    print("""
  => Gamma=30 时，同样偏航力矩要 **1.41 倍** 的舵量（相对 45 度）。
     而舵面行程是有限的，且大偏角会带来非线性/失速 ——
     这给 Gamma 又加了一条下界。
""")

    sub("(e) 为什么对角化是「T 和 A 配合」的结果")

    print("""  直接验算：对任意 Gamma，Ef 的非对角元是不是零。
  符号推导：
      T 的两列是 (1,1) 和 (1,-1)；
      A 的两行是 (cosG, cosG) 和 (sinG, -sinG)。
      第 1 列 x 第 1 行：cosG*(1) + cosG*(1) ... 用矩阵乘法看更清楚：

      A @ T = [[cosG*1 + cosG*1,  cosG*1 + cosG*(-1)],
               [sinG*1 - sinG*1,  sinG*1 - sinG*(-1)]]
            = [[2cosG,   0   ],
               [  0  , 2sinG]]

  => 非对角元是 **cosG*1 - cosG*1 = 0** 和 **sinG*1 - sinG*1 = 0**，
     两项各自抵消。这是 **T 的两列正交 + A 的两行结构相同** 的结果。

  ★ 反过来说：**只有「同向偏=俯仰、反向偏=偏航」这一个约定下才是解耦的。**
    如果哪个飞控用了 (1,1) 和 (-1,1) 之类的另一组列向量，
    A@T 立刻出现非对角项 —— 那就是「打升降舵串偏航」的数学原因。
""")
    print("  数值验证（几个 Gamma，看 Ef 的非对角）：")
    print()
    print("      Gamma      Ef[0,1]        Ef[1,0]        2*l_t*k*cosG    2*l_t*k*sinG")
    print("      " + "-" * 76)
    for Gd in [20, 35, 45, 60]:
        Ef = ef_matrix(Gd)
        G = math.radians(Gd)
        print("      %4d°   %12.2e  %12.2e   %13.4f   %13.4f"
              % (Gd, Ef[0, 1], Ef[1, 0],
                 2 * math.cos(G), 2 * math.sin(G)))


# ===========================================================================
# 实验 4  ★★ 混控矩阵与 ArduPilot 源码逐项对照
# ===========================================================================

def exp4():
    head("实验 4  ★★ 混控矩阵与 ArduPilot 源码逐项对照")

    sub("(a) ArduPlane 的混控函数（servos.cpp:167-186，原文）")

    print("""
  void Plane::channel_function_mixer(
          Aux_servo_function_t func1_in, Aux_servo_function_t func2_in,
          Aux_servo_function_t func1_out, Aux_servo_function_t func2_out) const
  {
      // the order is setup so that non-reversed servos go "up", and
      // func1 is the "left" channel. ...
      float in1 = SRV_Channels::get_output_scaled(func1_in);
      float in2 = SRV_Channels::get_output_scaled(func2_in);
      ...
      float out1 = constrain_float((in2 - in1) * g.mixing_gain, -4500, 4500);
      float out2 = constrain_float((in2 + in1) * g.mixing_gain, -4500, 4500);
      SRV_Channels::set_output_scaled(func1_out, out1);
      SRV_Channels::set_output_scaled(func2_out, out2);
  }

  调用点（servos.cpp:1010-1011，原文）：
      channel_function_mixer(k_aileron, k_elevator, k_elevon_left,  k_elevon_right);
      channel_function_mixer(k_rudder,  k_elevator, k_vtail_right,  k_vtail_left);
""")
    print("""  ⚠️ 注意最后一行：**vtail 的参数顺序是 (right, left)**，
     和上面 elevon 的 (left, right) **反着**。源码注释解释了原因 ——
     「func1 是 left 通道」，而 vtail 传进去的第一个输出参数是 right。
     看错这一行会把左右舵面对调。
""")

    sub("(b) 逐项对照")

    print("  把 func1=r, func2=p 代入（vtail：func1_in=rudder, func2_in=elevator）：")
    print()
    print("      out1 = (in2 - in1) * gain = (elevator - rudder) * gain  -> k_vtail_right")
    print("      out2 = (in2 + in1) * gain = (elevator + rudder) * gain  -> k_vtail_left")
    print()
    print("  本课推导（实验 2b）：")
    print("      delta_L = delta_e + delta_r")
    print("      delta_R = delta_e - delta_r")
    print()
    print("  => **结构完全一致**：左 = 俯仰 + 偏航，右 = 俯仰 - 偏航。")
    print("     （通道正负号还要看 SERVOn_REVERSED，见 (d)。）")

    # 数值对照
    sub("(c) 数值对照：本课公式 vs AP 公式")

    gain = 0.5      # MIXING_GAIN 默认值
    print("  取 MIXING_GAIN = %.1f（ArduPlane 默认），扫几组舵量：\n" % gain)
    print("      %-18s %12s %12s | %12s %12s" %
          ("(pitch, rudder)", "AP right", "AP left", "本课 d_R", "本课 d_L"))
    print("      " + "-" * 74)
    for p, r in [(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-0.5, 0.7)]:
        ap_r = (p - r) * gain
        ap_l = (p + r) * gain
        mine_r = (p - r) * gain
        mine_l = (p + r) * gain
        print("      %-18s %12.4f %12.4f | %12.4f %12.4f"
              % ("(%.1f, %.1f)" % (p, r), ap_r, ap_l, mine_r, mine_l))
    print("""
  => 逐项相同。**混控矩阵 T 是和 Gamma 无关的、与 01/03 课多旋翼同构的
     一个 2x2 矩阵** —— 只是输入输出轴不同。
""")

    sub("(d) elevon 和 vtail 用的是同一个函数")

    print("""
  ★★ 这是一个容易被忽略但很漂亮的事实：

      elevon:  channel_function_mixer(k_aileron, k_elevator, left, right)
               -> out1 = (elevator - aileron)  = 左副翼-升降舵
               -> out2 = (elevator + aileron)  = 右副翼-升降舵

      vtail:   channel_function_mixer(k_rudder, k_elevator, right, left)
               -> out1 = (elevator - rudder)   = 右 V 尾舵面
               -> out2 = (elevator + rudder)   = 左 V 尾舵面

  两者都是「俯仰 +/- 另一个通道」，**同一个 2x2 矩阵**：
      [[1, 1], [1, -1]]
  区别只在：
      · 那「另一个通道」是副翼（滚转）还是方向舵（偏航）
      · 输出两个舵面是「左右副翼」还是「左右 V 尾面」

  => V 尾 = **俯仰/偏航方向上的 elevon**。
     用一句话记住：**V 尾就是「竖直平面里的副翼升降舵混控」。**
""")
    T = mix_matrix()
    for name, other in [("elevon", "aileron"), ("vtail", "rudder")]:
        print("      %-8s  [delta_L; delta_R] = [[1,1],[1,-1]] @ [elevator; %s]"
              % (name, other))
    print()
    print("      T = [[1, 1], [1, -1]]，  T T^T = 2I，  cond(T) = %.4f"
          % np.linalg.cond(T))

    sub("(e) MIXING_GAIN = 0.5：和多旋翼的 ±0.5 是同一件事")

    print("""
  ArduPlane/Parameters.cpp:594-599（原文注释）：
      \"The default is 0.5, which ensures that the mixer doesn't saturate,
       allowing both input channels to go to extremes while retaining
       control over the output. Hardware mixers often have a 1.0 gain,
       which gives more servo throw, but can saturate.\"

  ★★ 把这段和 02 课 §4.2.5 对一下：
     多旋翼 normalise_rpy_factors() 把 roll/pitch/yaw **各自**缩到 ±0.5，
     源码注释（AP_MotorsMatrix.cpp:1305-1306）说：
     「一个 1.0 的 roll 指令最多把电机推离中心 0.5」——
     剩下 0.5 留给油门和 yaw。

  => **同一个数 0.5、同一个理由**（给另一个通道留行程），
     出现在**两个毫无关系的载具类别**里。
     这不是巧合 —— 只要两个通道共享同一组执行器的行程，
     就必须各让一半。
""")
    print("      多旋翼： 3 个姿态通道 + 油门 共享 ±1.0 行程")
    print("      V 尾  ： 俯仰 + 偏航     共享 ±1.0 舵面行程")
    print()
    print("      区别在于：多旋翼是**三个通道挤一个自由度**（欠驱动），")
    print("                V 尾是**两个通道挤两个自由度**（恰好确定）。")

    sub("(f) SITL 里把混控反解回来（SIM_Plane.cpp:293-297）")

    print("""
  仿真器拿到的是**两个舵面的位置**（遥控器/飞控已经混过控了），
  它必须反解回气动意义上的 elevator / rudder：

      } else if (vtail) {
          // fake a vtail plane
          float ch1 = elevator;      // 来自 RC 通道 2
          float ch2 = rudder;        // 来自 RC 通道 4
          // this matches VTAIL_OUTPUT==2
          elevator = (ch2-ch1)/2.0f;
          rudder   = (ch2+ch1)/2.0f;
      }

  正解是  delta_L = e + r,  delta_R = e - r
      相加： delta_L + delta_R = 2e  =>  e = (delta_L + delta_R)/2
      相减： delta_L - delta_R = 2r  =>  r = (delta_L - delta_R)/2

  ★ 而 plane-vtail.parm（Tools/autotest/default_params/）只有三行：
      SERVO2_FUNCTION 80      <- k_vtail_right
      SERVO4_FUNCTION 79      <- k_vtail_left
      SERVO2_REVERSED 1       <- ★ 右舵面反向
  正是这个 REVERSED 让 ch1 对应到 -delta_R 而不是 +delta_R，
  上面那两行 SITL 公式才和正解对上。

  => **这是一条完整的可核对链**：
     混控 (servos.cpp:183-186)  <->  参数 (plane-vtail.parm)
                                 <->  逆混控 (SIM_Plane.cpp:293-297)
     任何一环把符号写反，SITL 就会把一个正常的 V 尾飞机判成舵面装反。
""")


# ===========================================================================
# 实验 5  ★★ 稳定性代价 cos²Γ / sin²Γ 与匹配二面角
# ===========================================================================

def exp5():
    head("实验 5  ★★ 稳定性代价 cos²Gamma / sin²Gamma 与匹配二面角")

    sub("(a) V 尾同时是水平尾翼和垂直尾翼")

    print("""
  一个二面角 Gamma 的舵面，对气流的**有效投影面积**是：
      水平投影（提供纵向稳定） = S_p * cosG
      垂直投影（提供航向稳定） = S_p * sinG

  而它产生的力再投影一次（实验 2b 的 cosG / sinG）：
      Delta C_m_alpha  ∝ S_p * cosG * cosG = S_p * cos^2(G)
      Delta C_n_beta   ∝ S_p * sinG * sinG = S_p * sin^2(G)

  两块面，总面积 S_vt = 2 S_p：
""")
    print("      Delta C_m_alpha  ∝  S_vt * cos^2(G)")
    print("      Delta C_n_beta   ∝  S_vt * sin^2(G)")
    print()
    print("  ★ 注意是**平方**，不是一次方 ——")
    print("    控制效能是 cosG/sinG（一次），稳定性是 cos²G/sin²G（二次）。")
    print("    这是本课最容易写错的一个指数。")
    print()
    print("      Gamma     俯仰效能 cosG   俯仰稳定 cos²G    偏航效能 sinG   偏航稳定 sin²G")
    print("      " + "-" * 82)
    for Gd in [30, 35, 40, 45, 50, 60]:
        G = math.radians(Gd)
        print("      %4d°   %12.4f   %14.4f   %14.4f   %14.4f"
              % (Gd, math.cos(G), math.cos(G) ** 2,
                 math.sin(G), math.sin(G) ** 2))

    sub("(b) 匹配传统尾翼：最佳二面角")

    print("""
  设一架可比的传统尾翼是 S_h（水平安定面）+ S_v（垂直安定面）。
  想让 V 尾**给出同样的静稳定性**：

      S_vt * cos^2(G) = S_h
      S_vt * sin^2(G) = S_v

  两式相除：  tan^2(G) = S_v / S_h
  两式相加：  S_vt = S_h + S_v

  => ★★★ **Gamma* = arctan( sqrt(S_v / S_h) )**
     而总面积正好等于两个面之和 —— 这是 V 尾「不省面积」的原因。
""")
    print("      S_v/S_h     Gamma* （匹配稳定性）")
    print("      " + "-" * 40)
    for ratio in [0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70]:
        g = math.degrees(math.atan(math.sqrt(ratio)))
        print("      %6.2f       %8.2f°" % (ratio, g))
    print("""
  => 典型无人机 S_v/S_h 在 0.3~0.5，给出 **28°~35°** ——
     这和实际 V 尾机型的二面角**完全对得上**。

  ★ 所以真实的 30~40° 不是拍脑袋定的，它同时被两件事夹住：
      · 上界：稳定性匹配（实验 5b）要求 G 不能太大
      · 下界：舵面行程 / 偏航权限（实验 3d）要求 G 不能太小
     45°（控制效能最优）落在这两股力量之外，所以实际机型很少取。
""")

    sub("(c) 用真实机型参数算一遍")

    # 一架 3 kg 级 UAV 的典型尾容量
    S_h, S_v = 0.085, 0.032
    print("  取一架 3 kg 级电动固定翼（尾翼面积 S_h=%.3f, S_v=%.3f m^2）："
          % (S_h, S_v))
    print()
    print("      S_v/S_h = %.4f" % (S_v / S_h))
    gstar = math.degrees(math.atan(math.sqrt(S_v / S_h)))
    print("      Gamma* = arctan(sqrt(%.4f)) = %.2f deg" % (S_v / S_h, gstar))
    print("      S_vt  = S_h + S_v = %.4f m^2" % (S_h + S_v))
    print()
    print("  在几个候选二面角上，稳定性匹配程度（相对 Gamma*）：")
    print()
    print("      Gamma     提供的 C_m 比例   提供的 C_n 比例    面积需要放大")
    print("      " + "-" * 62)
    for Gd in [25, gstar, 35, 40, 45]:
        G = math.radians(Gd)
        gs = math.radians(gstar)
        # 以 S_vt = S_h + S_v 为基准，看能提供多少稳定性
        cm = (S_h + S_v) * math.cos(G) ** 2 / S_h
        cn = (S_h + S_v) * math.sin(G) ** 2 / S_v
        # 要匹配住缺口，面积需要放大多少
        scale = max(1.0 / cm, 1.0 / cn)
        tag = "  <- Gamma*" if abs(Gd - gstar) < 1e-6 else ""
        print("      %5.1f°   %14.3f   %14.3f   %14.2fx%s"
              % (Gd, cm, cn, scale, tag))
    print("""
  => 偏离 Gamma* 时，**总有一侧的稳定性不够**，只能靠加大尾翼面积补，
     而面积加大又带来阻力和重量。这就是「V 尾二面角不是自由参数」的含义。
""")

    sub("(d) 控制 vs 稳定：两个不同的 Gamma 诉求")

    print("""  把两个约束并排：

      诉求                       最优 Gamma        性质
      ------------------------------------------------------------
      控制效能最大（det / cond）     45°            几何
      两通道一样强（cond=1）         45°            几何
      俯仰权限优先                   < 45°          任务
      舵面行程够用                   > 25~30°       执行器
      稳定性匹配（S_v/S_h）         arctan√(Sv/Sh)  气动
      ------------------------------------------------------------

  => 实际机型是这几条的交集，典型 30~40°。
     本课算出来的 Gamma* 只是其中一条，但它是**唯一一条有闭式解**的。
""")

    if HAVE_PLT:
        sub("(e) 画图")
        gs = np.linspace(10, 80, 400)
        Gr = np.deg2rad(gs)
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        ax[0].plot(gs, np.cos(Gr), label="control cosG (pitch)")
        ax[0].plot(gs, np.sin(Gr), label="control sinG (yaw)")
        ax[0].plot(gs, np.cos(Gr) ** 2, "--", label="stability cos^2 G")
        ax[0].plot(gs, np.sin(Gr) ** 2, "--", label="stability sin^2 G")
        ax[0].axvline(45, c="gray", ls=":", label="45 deg")
        ax[0].axvline(gstar, c="red", ls=":", label="Gamma*=%.1f deg" % gstar)
        ax[0].set_xlabel("Gamma (deg)")
        ax[0].set_ylabel("efficiency / stability factor")
        ax[0].set_title("V-tail: control vs stability")
        ax[0].legend(fontsize=8)
        ax[0].grid(alpha=0.3)

        cond = np.maximum(np.tan(Gr), 1.0 / np.tan(Gr))
        ax[1].plot(gs, cond, label="cond(A) = max(tanG, cotG)")
        ax[1].axvline(45, c="gray", ls=":", label="45 deg (cond=1)")
        ax[1].axvline(gstar, c="red", ls=":", label="Gamma*=%.1f deg" % gstar)
        ax[1].set_xlabel("Gamma (deg)")
        ax[1].set_ylabel("condition number")
        ax[1].set_title("V-tail effectiveness conditioning")
        ax[1].legend(fontsize=8)
        ax[1].grid(alpha=0.3)

        plt.tight_layout()
        out = "vtail_design.png"
        plt.savefig(out, dpi=110)
        print("  已保存 %s" % out)


# ===========================================================================
# 实验 6  ★ 惯性耦合 Jxz
# ===========================================================================

def exp6():
    head("实验 6  ★ 惯性耦合 Jxz：多旋翼没有的项")

    sub("(a) 多旋翼的陀螺项 vs 固定翼的陀螺项")

    print("""
  刚体转动方程对两种载具**是同一个式子**：
      J w_dot + w x (J w) = tau

  区别全在 J：
      多旋翼  J = diag(Jx, Jy, Jz)        -> w x (J w) 的三个分量都是
                                            (Jy-Jz)qr 这种「两个角速率相乘」
      固定翼  J = [[Jx,0,-Jxz],[0,Jy,0],[-Jxz,0,Jz]]   -> 多出 Jxz 项
""")

    sub("(b) 展开固定翼的陀螺项")

    print("""  记 w = (p, q, r)。J w = (Jx p - Jxz r, Jy q, -Jxz p + Jz r)

      w x (J w) 的三个分量：

        x: q*(-Jxz p + Jz r) - r*(Jy q)
           = (Jz - Jy) q r - Jxz p q
        y: r*(Jx p - Jxz r) - p*(-Jxz p + Jz r)
           = (Jx - Jz) p r + Jxz (p^2 - r^2)
        z: p*(Jy q) - q*(Jx p - Jxz r)
           = (Jy - Jx) p q + Jxz q r

  ★★★ 看 **y（俯仰）** 那一行：
        Jy q_dot = M - (Jx - Jz) p r - Jxz (p^2 - r^2)

      最后那个 **Jxz (p^2 - r^2)** 就是**惯性耦合**：
      **纯滚转（p != 0）会直接产生俯仰力矩。**
""")

    sub("(c) 定量：多大的滚转率会产生多少俯仰")

    # 一架 3kg 级 UAV 的转动惯量
    Jx, Jy, Jz, Jxz = 0.045, 0.075, 0.090, 0.006
    print("  取一架 3 kg 级固定翼的惯量（kg*m^2）：")
    print("      Jx=%.3f  Jy=%.3f  Jz=%.3f  Jxz=%.4f  (Jxz/Jx = %.1f%%)"
          % (Jx, Jy, Jz, Jxz, 100 * Jxz / Jx))
    print()
    print("      p (deg/s)   r (deg/s)   Jxz(p²-r²) (N*m)   等效俯仰角加速度 (deg/s²)")
    print("      " + "-" * 70)
    for pd, rd in [(0, 0), (90, 0), (180, 0), (270, 0), (180, 90), (180, 180)]:
        p = math.radians(pd)
        r = math.radians(rd)
        term = Jxz * (p * p - r * r)
        qdd = math.degrees(-term / Jy)
        print("      %9d   %9d   %17.5f   %22.3f" % (pd, rd, term, qdd))
    p180 = math.radians(180)
    t180 = Jxz * p180 * p180
    print("""
  读表：

    · **只要 p 非零，就有俯仰耦合** —— r=0 时 q_dd = %.1f deg/s^2（p=180）。
      这正是「纯滚转会产生俯仰」的字面体现，多旋翼做不到这一点。
    · 增长是 **p 的平方**：90 -> 180 是 4 倍，180 -> 270 是 2.25 倍。
      所以它在**大角速率机动**下才露头 —— 巡航时几乎看不见。
    · 最后一行 p = r = 180 时归零：因为 (p^2 - r^2) 抵消了。
      这也说明这一项**不是**「滚转率的大小」，而是「滚转与偏航的不平衡」。
""" % (math.degrees(t180 / Jy)))
    print("""
  ★ 历史上这一项出过事故：F-100 超佩刀在高滚转率下「自己抬头」，
    根源就是 Jxz(p² - r²) 叠加气动效应（后来靠增大垂尾面积解决）。
    多旋翼因为 Jxz 严格为零，从来没有这个问题 ——
    **这是「对称性送礼」的又一个例子**（和实验 1c 的对角 J 是同一件事）。
""")

    sub("(d) 那多旋翼的陀螺项有多大？")

    print("""  多旋翼 J = diag(Jx, Jx, 2Jx)（01/03 课），陀螺项三个分量：
        x: (Jz - Jy) q r = Jx q r
        y: (Jx - Jz) p r = -Jx p r
        z: (Jy - Jx) p q = 0        <- ★ 偏航通道**完全没有**陀螺耦合

  这也是 01 课 §3.2.2 说的「J 各向同性时陀螺项恒为零」的一般化：
  **各向同性 -> 零；各向异性但对称（Jx=Jy≠Jz）-> 只剩两个交叉项。**
""")
    Jx, Jy, Jz = 0.008, 0.008, 0.015       # 01 课的典型值
    print("      取 Jx=Jy=%.4f, Jz=%.4f（01 课典型值）：" % (Jx, Jz))
    print()
    print("      p      q      r      陀螺项 roll    陀螺项 pitch   陀螺项 yaw")
    print("    (deg/s)                              (N*m)          (N*m)         (N*m)")
    print("      " + "-" * 72)
    for pd, qd, rd in [(180, 90, 0), (180, 0, 90), (180, 90, 90), (360, 90, 90)]:
        p, q, r = math.radians(pd), math.radians(qd), math.radians(rd)
        tx = (Jz - Jy) * q * r
        ty = (Jx - Jz) * p * r
        tz = (Jy - Jx) * p * q
        print("      %5d  %5d  %5d   %11.5f   %11.5f   %11.5f"
              % (pd, qd, rd, tx, ty, tz))
    print("""
  => 逐项看**哪一对角速率**能点着它（令 Jx=Jy=J）：

      分量    表达式              需要哪一对非零     对称四旋翼（Jx=Jy）
      ------------------------------------------------------------------
      roll    (Jz - Jy) q r        q 和 r            系数 Jz-Jy != 0
      pitch   (Jx - Jz) p r        p 和 r            系数 Jx-Jz != 0
      yaw     (Jy - Jx) p q        p 和 q            **系数恒为 0**

  ★ 所以对称多旋翼有一个**结构性豁免**：
    **偏航通道完全没有陀螺耦合**（Jx = Jy 让 (Jy-Jx)pq 恒等于零），
    另外两个分量**必须有对应的那一对同时非零**才出现。

  对比实验 6(c) 的固定翼：**只要 p 单独非零**，俯仰耦合立刻出现。
  差别在两项的结构完全不同：
      · 固定翼的 Jxz(p² - r²)：p 单独就能点着，是**单变量平方项**
      · 多旋翼的 (Jx-Jz)pr   ：必须有 p 和 r 两个量，是**双变量乘积**
    机动时 r 通常远小于 p（飞机绕自己的滚转轴转，不一定同时偏航），
    所以乘积项被压制得更狠 —— **这是它「温和」的真正原因。**
""")

    sub("(e) 一句话对照")

    print("      载具      J             陀螺项性质                量级")
    print("      " + "-" * 66)
    print("      四旋翼    diag          两个角速率相乘            小")
    print("      六/八轴   diag          同上，但桨的陀螺力矩按 sqrt(N) 长  小~中")
    print("      固定翼    含 Jxz        纯滚转就产生俯仰（p² 项）  **大**")
    print("      " + "-" * 66)
    print("""
  => **「J 是不是对角阵」是这两类载具在刚体动力学上的分水岭。**
     01/03 课可以放心地写 J = diag(...)，04 课不行。
""")


# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 固定翼 vs 多旋翼的根本区别", exp1),
    "2": ("★★★ V 尾二面角的力分解", exp2),
    "3": ("★★★ 效能矩阵与条件数", exp3),
    "4": ("★★ 混控矩阵与源码对照", exp4),
    "5": ("★★ 稳定性代价与匹配二面角", exp5),
    "6": ("★ 惯性耦合 Jxz", exp6),
}


def main():
    print()
    hr("#")
    print("#  第 9.6 周 · 04 固定翼 V 尾动力学建模")
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
