#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 9.5 周 · 03 六轴八轴动力学建模 —— 配套实验

    实验 1  ★★ 通用 N 旋翼力与力矩（01 的四旋翼公式是 N=4 特例）
    实验 2  ★★★ 转动惯量：从几何推出来（01 只给了典型值）
    实验 3  ★★ 效能 vs 惯量：为什么 yaw 永远是瓶颈
    实验 4  ★★ 非均匀布局：正交性什么时候没了
    实验 5  ★★★ 共轴（Y6 / X8）：02 没碰过的机型
    实验 6  ★ 陀螺项：为什么邻桨反旋向能让它自动抵消

用法：
    python lesson09b_nrotor_dynamics.py 1
    python lesson09b_nrotor_dynamics.py 2
    ...
    python lesson09b_nrotor_dynamics.py          # 全部

只需 numpy（画图需要 matplotlib）。

参考源码（ArduPilot Copter-4.5.7）：
    libraries/AP_Motors/AP_MotorsMatrix.cpp:536-544   角度 → 系数的三角函数
    libraries/AP_Motors/AP_MotorsMatrix.cpp:774-844   setup_hexa_matrix（三张表）
    libraries/AP_Motors/AP_MotorsMatrix.cpp:853-968   setup_octa_matrix
    libraries/AP_Motors/AP_MotorsMatrix.cpp:889-901   OCTA V（MotorDefRaw，非均匀）
    libraries/AP_Motors/AP_MotorsMatrix.cpp:972-1070  setup_octaquad_matrix（共轴 X8）
    libraries/AP_Motors/AP_MotorsMatrix.cpp:1145-1196 setup_y6_matrix（共轴 Y6）
    libraries/AP_Motors/AP_MotorsMatrix.h:82-101      MotorDef / MotorDefRaw
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
# 公共：布局定义
#
# 全部照抄 ArduPilot 源码，字段顺序 = (roll_factor, pitch_factor, yaw_factor)
# 源码里 add_motor 是 add_motor_raw(cosf(radians(roll_deg+90)), cosf(radians(pitch_deg)), ...)
# 所以：  方位角 theta 的电机  ->  roll = -sin(theta),  pitch = +cos(theta)
# 见 03-动力学建模/01 课 §3.4。CCW = +1, CW = -1。
# ===========================================================================

# 用「方位角 + 旋向」描述的等分布布局（源码用 MotorDef）
CIRCULAR = {
    # 源码 :774-844  setup_hexa_matrix
    "HEXA_X":    ([90, -90, -30, 150, 30, -150], [-1, 1, -1, 1, 1, -1]),
    "HEXA_PLUS": ([0, 180, -120, 60, -60, 120],  [-1, 1, -1, 1, 1, -1]),
    # 源码 :853-887  setup_octa_matrix
    "OCTA_X":    ([22.5, -157.5, 67.5, 157.5, -22.5, -112.5, -67.5, 112.5],
                  [1, 1, -1, -1, -1, -1, 1, 1]),
    "OCTA_PLUS": ([0, 180, 45, 135, -45, -135, -90, 90],
                  [-1, -1, 1, 1, 1, 1, -1, -1]),
    # 源码 :972-1010  setup_octaquad_matrix（共轴 X8：同一角度出现两次）
    "OCTAQUAD_X": ([45, -45, -135, 135, 45, -45, -135, 135],
                   [1, -1, 1, -1, 1, -1, 1, -1]),
    # 对照用：等分布四旋翼
    "QUAD_X":    ([45, -135, -45, 135],          [1, 1, -1, -1]),
}

# 源码直接给 (roll, pitch, yaw) 的布局（MotorDefRaw）
RAW = {
    # 源码 :806-813  HEXA H（中间两个电机往里收）
    "HEXA_H": [(-1.0, 0.0, -1), (1.0, 0.0, 1), (1.0, 1.0, -1),
               (-1.0, -1.0, 1), (-1.0, 1.0, 1), (1.0, -1.0, -1)],
    # 源码 :889-901  OCTA V（非均匀、非圆周）
    "OCTA_V": [(0.83, 0.34, -1), (-0.67, -0.32, -1), (0.67, -0.32, 1),
               (-0.50, -1.00, 1), (1.00, 1.00, 1), (-0.83, 0.34, 1),
               (-1.00, 1.00, -1), (0.50, -1.00, -1)],
    # 源码 :1170-1176  Y6 default（3 条臂 × 2 共轴，力臂 0.666 / 1.333）
    "Y6": [(-1.0, 0.666, 1), (1.0, 0.666, -1), (1.0, 0.666, 1),
           (0.0, -1.333, -1), (-1.0, 0.666, -1), (0.0, -1.333, 1)],
}


def circular_matrix(angles_deg, spin):
    """等分布布局 -> 3xN 效能矩阵 [roll; pitch; yaw]（未归一化）。"""
    a = np.deg2rad(np.asarray(angles_deg, dtype=float))
    return np.vstack([-np.sin(a), np.cos(a), np.asarray(spin, dtype=float)])


def raw_matrix(rows):
    """(roll, pitch, yaw) 列表 -> 3xN 效能矩阵。"""
    return np.array(rows, dtype=float).T


def all_frames():
    """全部布局 -> {名字: (3xN 矩阵, 备注)}"""
    out = {}
    for name, (ang, sp) in CIRCULAR.items():
        out[name] = (circular_matrix(ang, sp), "等分布圆周")
    for name, rows in RAW.items():
        out[name] = (raw_matrix(rows), "MotorDefRaw")
    return out


def gram(B):
    """G = B B^T / (N/2)  —— 02 课 §4.5 用的同一套量。"""
    n = B.shape[1]
    return B @ B.T / (n / 2.0)


def offdiag_max(G):
    return float(np.abs(G - np.diag(np.diag(G))).max())


# ===========================================================================
# 实验 1  ★★ 通用 N 旋翼力与力矩
# ===========================================================================

def exp1():
    head("实验 1  ★★ 通用 N 旋翼力与力矩")

    sub("(a) 从通用式到等分布恒等式")

    print("""
  电机 i 位于机体 (x_i, y_i, 0)，推力 T_i 沿机体 -Z。
  对质心取矩：  r x F = (-y_i T_i, +x_i T_i, 0)

      tau_roll  = -sum_i y_i T_i
      tau_pitch = +sum_i x_i T_i
      tau_yaw   =  sum_i k_m n_i^2 sigma_i

  这组式和 01 课 §3.3.2 对四旋翼写的**完全一样** ——
  它本来就不含 N。N 只决定求和跑几项。

  换极坐标 x_i = R cos(theta_i), y_i = R sin(theta_i)，去掉公共因子 R：
      roll_i = -sin(theta_i)   pitch_i = +cos(theta_i)   yaw_i = sigma_i
""")

    sub("(b) 等分布恒等式（数值验证）")

    print("  沿圆周等角距分布时，下面三个和式是**恒等式**，与 N 无关：")
    print("      sum sin^2(theta_i) = N/2")
    print("      sum cos^2(theta_i) = N/2")
    print("      sum sin(theta_i)cos(theta_i) = 0")
    print()
    print("      N     sum sin^2    sum cos^2    sum sin*cos")
    print("      " + "-" * 48)
    for N in [3, 4, 5, 6, 8, 12, 16]:
        th = 2 * np.pi * np.arange(N) / N + 0.37   # 任意相位，验证与相位无关
        s2 = np.sum(np.sin(th) ** 2)
        c2 = np.sum(np.cos(th) ** 2)
        sc = np.sum(np.sin(th) * np.cos(th))
        print("      %-4d  %10.6f   %10.6f   %12.3e" % (N, s2, c2, sc))
    print()
    print("  => 前两个精确等于 N/2，第三个恒为 0（1e-16 量级是浮点噪声）。")

    sub("(c) 结论：等分布布局的 G 矩阵")

    for name in ["QUAD_X", "HEXA_X", "OCTA_X", "OCTAQUAD_X"]:
        B, _ = all_frames()[name]
        G = gram(B)
        print()
        print("  %-12s N=%d" % (name, B.shape[1]))
        print("      G = B B^T / (N/2) = (%.4f, %.4f, %.4f) 沿对角"
              % tuple(np.diag(G)))
        print("      非对角最大 = %.3e   条件数 = %.4f"
              % (offdiag_max(G), np.linalg.cond(G)))
    print("""
  => 所有「等半径 + 等角距 + 邻桨反旋向」的布局都给出
         G = (N/2) * diag(1, 1, 2)
     即归一化之后 **G = diag(1,1,2)，条件数恒为 2，与 N 无关**。

  ⚠ 注意 yaw 那一项是 2 而不是 1 —— 这一条会在实验 3 变成关键的伏笔。
""")


# ===========================================================================
# 实验 2  ★★★ 转动惯量：从几何推出来
# ===========================================================================

def exp2():
    head("实验 2  ★★★ 转动惯量：从几何推出来")

    sub("(a) 01 课只给了一个「典型值」")

    print("""
  01 课 §3.2.2 写着：
      典型 5 寸机： J = diag(0.008, 0.008, 0.015) kg*m^2
  但没解释**这三个数从哪来**。本实验把它推出来。

  逐电机的平行轴定理（电机质量 m_i，位置 (x_i, y_i, z_i)，
  自身绕质心的转动惯量先忽略 —— 它比 m R^2 小两个量级）：
      Delta Jxx = sum m_i (y_i^2 + z_i^2)
      Delta Jyy = sum m_i (x_i^2 + z_i^2)
      Delta Jzz = sum m_i (x_i^2 + y_i^2)
  机架是平的（z_i = 0），电机等质量 m：
      Delta Jxx = m sum y_i^2      Delta Jyy = m sum x_i^2
      Delta Jzz = m sum (x_i^2 + y_i^2)
""")

    sub("(b) 等分布布局：Jzz/Jxx = 2")

    print("      N     sum y^2/R^2   sum x^2/R^2   sum(x^2+y^2)/R^2   Jzz/Jxx")
    print("      " + "-" * 62)
    for N in [3, 4, 6, 8, 12]:
        th = 2 * np.pi * np.arange(N) / N
        syy = np.sum(np.sin(th) ** 2)
        sxx = np.sum(np.cos(th) ** 2)
        szz = np.sum(np.cos(th) ** 2 + np.sin(th) ** 2)
        print("      %-4d  %12.6f   %12.6f   %16.6f   %8.6f"
              % (N, syy, sxx, szz, szz / sxx))
    print("""
  => Jxx = Jyy = m R^2 * N/2 ,  Jzz = m R^2 * N
     **Jzz/Jxx = 2，精确地、与 N 无关。**

  和 01 给的典型值对一下：
""")
    print("      J = diag(0.008, 0.008, 0.015)  ->  Jzz/Jxx = %.4f" % (0.015 / 0.008))
    print("""
  1.875 vs 2 —— 差的那 6% 来自**机体/电池/载荷**趴在中心附近：
  它们对 Jxx 和 Jzz 的贡献接近相等（都离 z 轴不远），
  所以把比值从 2 往回拉。**几何项越主导，比值越接近 2。**
""")

    sub("(c) 把机体质量也算进去")

    R = 0.125          # 5 寸机臂长（m）
    m_rotor = 0.012    # 单电机+桨（kg）
    J_body = 2.0e-4    # 机体+电池（近似球状集中在中心）

    print("  取 R=%.3f m, 单电机+桨 %.3f kg, 机体+电池 %.1e kg*m^2:"
          % (R, m_rotor, J_body))
    print()
    print("      N     Jxx(几何)   Jzz(几何)   Jxx(总)     Jzz(总)    Jzz/Jxx")
    print("      " + "-" * 68)
    for N in [4, 6, 8]:
        Jxx_g = m_rotor * R * R * N / 2.0
        Jzz_g = m_rotor * R * R * N
        # 机体近似球状：对三个轴贡献相同
        Jxx = Jxx_g + J_body
        Jzz = Jzz_g + J_body
        print("      %-4d  %10.3e  %10.3e  %10.3e  %10.3e   %8.4f"
              % (N, Jxx_g, Jzz_g, Jxx, Jzz, Jzz / Jxx))
    print("""
  => N 越大，几何项越主导，Jzz/Jxx 越靠近 2。
     这解释了为什么**大机器比小机器更接近 2**，
     也解释了为什么六轴/八轴的 Jzz 相对更大 —— yaw 更笨重。
""")

    if HAVE_PLT:
        sub("(d) 画图")
        Ns = np.arange(3, 13)
        ratios = []
        for N in Ns:
            Jxx = m_rotor * R * R * N / 2.0 + J_body
            Jzz = m_rotor * R * R * N * 1.0 + J_body
            ratios.append(Jzz / Jxx)
        plt.figure(figsize=(6, 4))
        plt.plot(Ns, ratios, "o-", label="Jzz/Jxx")
        plt.axhline(2.0, ls="--", c="gray", label="几何极限 = 2")
        plt.axhline(0.015 / 0.008, ls=":", c="red", label="01 课典型值 1.875")
        plt.xlabel("N (rotors)")
        plt.ylabel("Jzz / Jxx")
        plt.title("Jzz/Jxx vs rotor count")
        plt.legend()
        plt.grid(alpha=0.3)
        plt.tight_layout()
        out = "nrotor_inertia.png"
        plt.savefig(out, dpi=110)
        print("  已保存 %s" % out)


# ===========================================================================
# 实验 3  ★★ 效能 vs 惯量：为什么 yaw 永远是瓶颈
# ===========================================================================

def exp3():
    head("实验 3  ★★ 效能 vs 惯量：为什么 yaw 永远是瓶颈")

    sub("(a) 「电机代价」怎么算")

    print("""
  02 课 §4.3.2 说过：冗余机型的混控解 = 伪逆 = 最小范数解
      u = B^T (B B^T)^{-1} y = B^T G^{-1} y
  所以「产生目标力旋量 y 所需的电机总代价」是
      ||u||^2 = y^T G^{-1} y

  等分布布局 G = (N/2) diag(1,1,2)，代入：
      ||u||^2 = (2/N) * ( tau_r^2 + tau_p^2 + tau_y^2 / 2 )

  => **同样的电机代价，能买到 sqrt(2) 倍的 yaw 力矩。**
     从「执行器便宜不便宜」的角度看，yaw 是便宜的。
""")

    sub("(b) 但惯量把这件事反过来")

    print("""
  角加速度 alpha = J^{-1} tau，而几何给出 J = diag(J, J, 2J)：
      alpha_r = tau_r / J      alpha_p = tau_p / J      alpha_y = tau_y / (2J)

  固定 ||u||（同样的电机代价预算），把全部预算投给某一轴：
      tau_r = sqrt(G_xx) * a = sqrt(N/2) * a   ->  alpha_r = sqrt(N/2) a / J
      tau_y = sqrt(G_zz) * a = sqrt(N)   * a   ->  alpha_y = sqrt(N) a / (2J)

  比值：
      alpha_y / alpha_r = sqrt(2)/2 = 0.7071
""")
    print("      N     alpha_r 系数    alpha_y 系数    alpha_y/alpha_r")
    print("      " + "-" * 56)
    for N in [4, 6, 8, 12]:
        J = 1.0                       # 只看比值，取 J=1
        ar = math.sqrt(N / 2.0) / J
        ay = math.sqrt(float(N)) / (2.0 * J)
        print("      %-4d  %13.6f    %13.6f    %15.6f"
              % (N, ar, ay, ay / ar))

    print("""
  => **单位电机代价换到的角加速度，roll : pitch : yaw = 1 : 1 : 0.7071，
     与 N 无关。**

  这是本课最重要的一个结论：
  · 加电机（四 -> 六 -> 八）**不会**改善这个比值；
  · yaw 永远比 roll/pitch 弱一个 sqrt(2)；
  · 而如果 Jzz/Jxx 因为机体质量掉到 2 以下，比值反而会回一点
    （见实验 2c：alpha_y/alpha_r = sqrt(2)/(Jzz/Jxx)）。
""")

    sub("(c) 顺带：条件数也搬不动")

    print("  02 课练习 5 量过「满机悬停条件数 20.0，失效后 21.3~28.6」。")
    print("  那是**分配矩阵在饱和约束下**的条件数。这里量的是另一件事 ——")
    print("  效能矩阵 G 本身的条件数：")
    print()
    for name in ["QUAD_X", "HEXA_X", "OCTA_X", "OCTAQUAD_X", "HEXA_H", "OCTA_V"]:
        B, _ = all_frames()[name]
        G = gram(B)
        mark = "" if offdiag_max(G) < 1e-12 else "   <- 非正交"
        print("      %-12s N=%-2d  cond(G) = %8.4f   max|offdiag| = %.3e%s"
              % (name, B.shape[1], np.linalg.cond(G), offdiag_max(G), mark))
    print("""
  => 等分布布局恒为 2。**再多的电机也不会让 G 变得更「好解」。**
     六轴/八轴买到的是冗余（02 课 §4.5.6 的秩），不是更好的条件数。

  ⚠️ 但注意最后两行 —— **HEXA_H 明明不正交（offdiag 0.6667），
     cond(G) 却还是 2.0000。** 这说明：
         cond(G) 量的是「各轴增益差多少」，**不是**「轴与轴正交不正交」。
     两个指标看的是不同的东西，谁也不能代替谁。
     （这和 02 课练习 3 的结论是同一个坑：Gram 矩阵对坐标翻转完全不变，
       所以「靠解耦/正交性做的自检」查不出某些错误。）
""")


# ===========================================================================
# 实验 4  ★★ 非均匀布局：正交性什么时候没了
# ===========================================================================

def exp4():
    head("实验 4  ★★ 非均匀布局：正交性什么时候没了")

    sub("(a) 全布局扫描")

    print("      %-12s %-3s %12s %10s   %s"
          % ("布局", "N", "max|offdiag|", "cond(G)", "roll·yaw"))
    print("      " + "-" * 62)
    for name in ["QUAD_X", "HEXA_X", "HEXA_PLUS", "OCTA_X", "OCTA_PLUS",
                 "OCTAQUAD_X", "HEXA_H", "OCTA_V", "Y6"]:
        B, _ = all_frames()[name]
        G = gram(B)
        print("      %-12s %-3d %12.3e %10.4f   %+9.4f"
              % (name, B.shape[1], offdiag_max(G),
                 np.linalg.cond(G), G[0, 2]))

    sub("(b) 规律一：镜像对称 => pitch 行自动正交")

    print("""
  机架左右对称（关于 x-z 平面）意味着：对每个电机 i，
  存在另一个电机 i' 满足  (x, y, sigma) -> (x, -y, -sigma)。

  看三个交叉项在这样一对上的贡献：
      roll_i * pitch_i = (-y)(x) = -yx
      roll_i' * pitch_i' = (+y)(x) = +yx          -> 抵消
      pitch_i * yaw_i = x * sigma
      pitch_i' * yaw_i' = x * (-sigma)            -> 抵消
      roll_i * yaw_i = (-y)(sigma) = -y*sigma
      roll_i' * yaw_i' = (+y)(-sigma) = -y*sigma  -> **相加，不抵消！**

  => **镜像对称保证 pitch 行与 roll、yaw 都正交；但管不了 roll·yaw。**
""")

    print("      %-12s %14s %14s %14s"
          % ("布局", "roll·pitch", "pitch·yaw", "roll·yaw"))
    print("      " + "-" * 58)
    for name in ["HEXA_X", "HEXA_H", "OCTA_X", "OCTA_V", "Y6"]:
        B, _ = all_frames()[name]
        G = gram(B)
        print("      %-12s %14.3e %14.3e %+14.4f"
              % (name, G[0, 1], G[1, 2], G[0, 2]))

    sub("(c) 规律二：roll·yaw = 0 需要额外的条件")

    print("""
  roll·yaw = sum_i (-y_i) sigma_i / (N/2) = -(R / (N/2)) * sum_i sin(theta_i) sigma_i

  镜像对称管不了它（上面刚证过）。让它归零的是**等角距**：
  等角距时 theta_k = 2*pi*k/N + phi，而邻桨反旋向给出 sigma_k = (-1)^k，
  于是这是一个**采样点上的交替序列**。写成指数形式：

      sigma_k = (-1)^k = exp(i*pi*k)
      sum_k sigma_k exp(i*theta_k)
          = exp(i*phi) * sum_k exp(i*(2*pi/N + pi)*k)
          = exp(i*phi) * (1 - r^N)/(1 - r),      r = exp(i*(2*pi/N + pi))
      r^N = exp(i*(2*pi + pi*N)) = exp(i*pi*N) = (-1)^N

  只要 **N 是偶数**，r^N = 1 => 分子 = 0 => **整个和式精确为零**。
  （取虚部就得到 sum sin(theta)*sigma = 0。）

  ★ 换个说法：交替序列 (-1)^k 与采样点上任何**整数频率**的正弦正交。
    这正是 02 课「正交性来自沿圆周均匀分布」的**离散版本** ——
    同一件事，一个用 Gram 矩阵说，一个用采样正交说。

  ⚠️ 两个前提缺一不可：**等角距**（否则 r^N != 1）和 **N 为偶数**
    （否则交替序列首尾接不上，sigma 自己也做不到交替）。
""")

    print("  直接验算「如果只有邻桨反旋向、但不是等角距」会怎样：")
    print()
    print("      %-26s %12s %12s" % ("布局", "roll·yaw", "max|offdiag|"))
    print("      " + "-" * 54)
    # 六旋翼等角距
    B, _ = all_frames()["HEXA_X"]
    print("      %-26s %+12.4f %12.3e"
          % ("HEXA_X 等角距 60°", gram(B)[0, 2], offdiag_max(gram(B))))
    # 同样旋向、但角度改成 60/60/60 抖一点
    for dev in [0.0, 5.0, 10.0, 20.0]:
        ang = [90, -90, -30 + dev, 150, 30, -150]
        B = circular_matrix(ang, [-1, 1, -1, 1, 1, -1])
        G = gram(B)
        print("      %-26s %+12.4f %12.3e"
              % ("HEXA_X 内圈偏 %.0f°" % dev, G[0, 2], offdiag_max(G)))
    print("""
  => 只要**破坏等角距**，roll·yaw 立刻离开 0。
     这是 HEXA_H（-0.6667）和 OCTA_V（+0.1700）不正交的同一个原因：
     机臂长度/角距不均匀。
""")

    sub("(d) ★ 但这和 02 课的「串扰」不是一回事")

    print("""
  ⚠ 必须分清两件事：

  (1) **02 课量的串扰**（OCTA V 打 yaw 串出 14.2% roll）
      是「ArduPilot 手写的固定系数表**不是** B 的伪逆」造成的 ——
      它是**实现误差**，换 PX4 的现算伪逆就没有。

  (2) **本实验量的 max|offdiag|**
      是 B 本身的性质（G = B B^T），**与用哪种求逆无关**。
      即使用完美的伪逆，G 不正交依然意味着：
        · 某些轴的解需要更多电机行程
        · 饱和时先牺牲的是弱轴

  换句话说：(1) 可以修，(2) 只能靠**改机架几何**。
""")
    # 证明 (2)：伪逆解严格达成目标，与非正交性无关
    B, _ = all_frames()["OCTA_V"]
    y = np.array([0.3, 0.0, 0.25])          # 期望 roll / pitch / yaw
    u = B.T @ np.linalg.pinv(B @ B.T) @ y
    print("  验算（OCTA V，期望 y = [%.2f, %.2f, %.2f]）：" % tuple(y))
    print("      伪逆解实得力旋量 = [%+.6f, %+.6f, %+.6f]" % tuple(B @ u))
    print("      误差 = %.3e  <- **精确达成**，与非正交性无关"
          % np.abs(B @ u - y).max())
    print("      最小范数代价 ||u|| = %.6f" % np.linalg.norm(u))
    # 对比等分布
    B2, _ = all_frames()["OCTA_X"]
    u2 = B2.T @ np.linalg.pinv(B2 @ B2.T) @ y
    print("   对照（OCTA X，同样目标）：")
    print("      ||u|| = %.6f  -> OCTA X 反而贵 %.1f%%"
          % (np.linalg.norm(u2),
             100 * (np.linalg.norm(u2) / np.linalg.norm(u) - 1)))
    Gv, Gx = gram(B), gram(B2)
    print("""
  ★ 这个方向出乎意料：**不正交的 OCTA V 反而更省电机**。
    原因看 G 的对角元 ——

        OCTA_X : G_xx = %.3f  G_yy = %.3f   roll*yaw = %+.3f
        OCTA_V : G_xx = %.3f  G_yy = %.3f   roll*yaw = %+.3f

    V 型角上那四个电机（系数 (1.00, 1.00)）同时离 x 轴和 y 轴都远，
    力臂更长 => 单位推力的力矩产出更大。

  => 所以「非均匀布局」**不是单纯的变差**，它是一次交换：
        更高的效能（同样的力旋量更省电机）
          vs
        roll*yaw 交叉项 %+.2f（两个通道绑在一起）

    02 课讲的 OCTA V 串扰，源头正是这个交叉项 ——
    只不过 02 课看到的是它在「AP 固定系数表」下的**实现后果**。
""" % (Gx[0, 0], Gx[1, 1], Gx[0, 2],
       Gv[0, 0], Gv[1, 1], Gv[0, 2],
       Gv[0, 2]))


# ===========================================================================
# 实验 5  ★★★ 共轴（Y6 / X8）
# ===========================================================================

def exp5():
    head("实验 5  ★★★ 共轴（Y6 / X8）：02 没碰过的机型")

    sub("(a) 结构：同一条臂上装两个桨，必然反向")

    print("""
  ArduPilot 里共轴机型有三个：
      Y6       源码 :1145-1196   3 条臂 x 2 = 6 电机
      X8 / OCTAQUAD  源码 :972-1070   4 条臂 x 2 = 8 电机
      （DECA 是 5 臂 x 2）

  它们和等分布布局的**唯一区别**：有电机的 (x, y) 完全重合。
  看源码里 Y6 default 的系数表（:1170-1176，字段 = roll, pitch, yaw）：
""")
    for i, (r, p, s) in enumerate(RAW["Y6"]):
        print("      m%d : roll=%+6.3f  pitch=%+6.3f  yaw=%+d"
              % (i, r, p, s))
    print("""
  m0/m4 同一位置（roll=-1, pitch=0.666）、m1/m2 同一位置、m3/m5 同一位置。
  **每一对的两个 yaw 系数都相反** —— 也就是两个桨反向转。

  验 sum(sigma) = %+d
""" % int(sum(r[2] for r in RAW["Y6"])))

    sub("(b) 共轴对：推力叠加，反扭矩差动")

    print("""
  设同一位置上下两桨推力 T_u / T_l，旋向相反（sigma_u = +1, sigma_l = -1）：
      这一对的总推力   = T_u + T_l
      这一对的 yaw 力矩 = k_m (T_u - T_l)      <- **差动**

  => 想让共轴对产生 yaw，必须让上下桨**转速不同**，
     而转速不同又改变总推力。**这是共轴机型特有的耦合。**

  对整机来说：
      yaw   = sum_pairs (T_u - T_l)
      推力  = sum_pairs (T_u + T_l)

  于是「打 yaw 会带推力变化」这件事在共轴上比平铺更严重 ——
  平铺机型的 yaw 靠**不同位置的桨**（力臂 x 反扭矩），
  符号成对抵消；共轴靠**同一位置的上下桨**，推力干扰无法靠几何抵消。
""")

    sub("(c) 效率损失：下桨泡在上桨的滑流里")

    print("  严格建模要用动量理论 + 滑流收缩。工程上常用一个一阶模型：")
    print("      T_l = eta * T_u        eta = 下桨推力折损系数")
    print("  实测 eta 典型 0.75~0.85（悬停），前飞时更接近 1。")
    print()
    eta = 0.80
    print("  取 eta = %.2f，看一对共轴在「等转速」和「要 yaw」两种情况："
          % eta)
    print()
    print("      工况                     推力     yaw力矩")
    print("      " + "-" * 46)
    for Tu in [1.0]:
        Tl = eta * Tu
        print("      上下等转速 T_u=%.2f          %6.3f   %+8.3f"
              % (Tu, Tu + Tl, Tu - Tl))
    # 要零 yaw 就得上下不等
    for dyaw in [0.0, 0.1, 0.2]:
        # T_u - T_l = dyaw, T_u + T_l = 2*T_avg
        Tu = 1.0 + dyaw / 2.0
        Tl = (1.0 - dyaw / 2.0) * eta
        print("      目标 yaw 差 %.2f              %6.3f   %+8.3f"
              % (dyaw, Tu + Tl, Tu - Tl))
    print("""
  => 想要 yaw，推力就变。上表第二列随 yaw 指令漂移 ——
     这就是共轴机型「打方向舵掉高度」的机理（和 01 课四旋翼掉高度
     是**两个不同的原因**：那个是推力矢量倾斜，这个是**反扭矩差动**）。
""")

    sub("(d) 共轴八轴的矩阵长什么样")

    B8, _ = all_frames()["OCTAQUAD_X"]
    print("  OCTAQUAD_X（源码 :972-1010），列 = m0..m7：")
    print()
    print("      %-8s" % "", end="")
    for i in range(8):
        print("%8s" % ("m%d" % i), end="")
    print()
    for r, lab in enumerate(["roll", "pitch", "yaw"]):
        print("      %-8s" % lab, end="")
        for i in range(8):
            print("%8.3f" % B8[r, i], end="")
        print()
    print("""
  看列：**m0 和 m4 的 roll、pitch 完全相同，只有 yaw 反号。**
  m1/m5、m2/m6、m3/m7 同理 —— 这就是「同一位置、上下两桨」。

  => ArduPilot 把上下桨当成**两个独立执行器**登记，
     所以静态混控矩阵**察觉不到共轴的存在**：
     G 和 OCTA_X 逐元素相同（实验 1c 已验），秩也相同。
""")
    G8, Gx8 = gram(B8), gram(all_frames()["OCTA_X"][0])
    print("      OCTAQUAD_X : G = (%.4f, %.4f, %.4f)  cond = %.4f"
          % (G8[0, 0], G8[1, 1], G8[2, 2], np.linalg.cond(G8)))
    print("      OCTA_X     : G = (%.4f, %.4f, %.4f)  cond = %.4f"
          % (Gx8[0, 0], Gx8[1, 1], Gx8[2, 2], np.linalg.cond(Gx8)))
    print("""
  => 所以「共轴的代价」**不在静态混控矩阵里**，而在三处：
     ① 实验 5(c) 的效率损失（推力不叠加，eta 折损）
     ② 实验 5(e) 的转动惯量（上下间距给 Jxx 加料）
     ③ 动态：下桨泡在滑流里，等效来流速度更高 =>
        同样的转速变化给出不同的推力变化，上下桨**动态不对称**

  ★ 这正是 02 课把 Y6/X8 归到「空白区」的原因 ——
    它们的问题不是矩阵问题，是**动力学**问题。
""")

    sub("(e) 共轴还改变了转动惯量")

    R = 0.20
    m = 0.05
    h = 0.08
    print("  上下两桨质量都是 m、水平位置相同 (x_i, y_i)，只在 z 上分开 +-h/2：")
    print("      Delta Jzz = sum m (x^2+y^2)     <- **与 z 无关**")
    print("      Delta Jxx = sum m (y^2+z^2)     <- z 有贡献")
    print()
    print("  设 R=%.2f m，上下桨间距 h=%.3f m，4 条臂 x 上下各一桨："
          % (R, h))
    ang = np.deg2rad([45, 135, 225, 315])
    Jzz_exact = sum(m * R * R for a in ang for _ in (0, 1))
    Jxx = sum(m * ((R * np.sin(a)) ** 2 + (h / 2.0) ** 2)
              for a in ang for _ in (0, 1))
    print("      Jzz(机臂项)    = %.4e" % Jzz_exact)
    print("      Jxx(含上下间距) = %.4e" % Jxx)
    print("      Jzz/Jxx = %.4f   <- **小于 2**，因为 z 给了 Jxx 额外贡献"
          % (Jzz_exact / Jxx))
    print("""
  作为对照，平铺八轴同样的 R、m（z 全为 0）：
      Jzz = %.4e   Jxx = %.4e   Jzz/Jxx = %.4f
""" % (Jzz_exact, sum(m * (R * np.sin(a)) ** 2 for a in ang for _ in (0, 1)),
       Jzz_exact / sum(m * (R * np.sin(a)) ** 2
                       for a in ang for _ in (0, 1))))
    print("""
  => 共轴把 Jzz/Jxx 从 2 往下拉。按实验 3 的公式
        alpha_y / alpha_r = sqrt(2) / (Jzz/Jxx)
     Jzz/Jxx 变小 => 比值变大 => **共轴的 yaw 响应反而比平铺快一点**。
     这是拿效率损失（实验 5c）换来的一个好处。
""")


# ===========================================================================
# 实验 6  ★ 陀螺项：邻桨反旋向让它自动抵消
# ===========================================================================

def exp6():
    head("实验 6  ★ 陀螺项：邻桨反旋向让它自动抵消")

    sub("(a) 01 课写了这一项，但没推后果")

    print("""
  01 课 §3.2.2 给出螺旋桨的陀螺力矩（这里写成量纲正确的形式）：
      tau_gyro = sum_i J_prop * omega_prop,i * sigma_i * (omega_body x z_hat)

  把与 i 无关的项提出来：
      tau_gyro = (omega_body x z_hat) * J_prop * sum_i ( omega_prop,i * sigma_i )
""")

    sub("(b) 悬停时它恒等于零")

    print("""
  悬停配平时所有桨转速相同（omega_prop,i = omega_prop），于是：
      sum_i omega_prop,i * sigma_i = omega_prop * sum_i sigma_i

  而「邻桨反旋向」恰好保证 sum(sigma) = 0 ——
  **这正是 02 课 §4.2.4 说的那件事。**

  => 悬停时陀螺力矩项**精确为零**，与 N 无关。
""")
    for name, (ang, sp) in CIRCULAR.items():
        if name in ("OCTAQUAD_X",):
            pass
        s = sum(sp)
        print("      %-12s  sum(sigma) = %+d" % (name, s))
    B, _ = all_frames()["Y6"]
    print("      %-12s  sum(sigma) = %+d" % ("Y6", int(B[2].sum())))

    sub("(c) 但差速时就不抵消了")

    print("""
  一旦电机转速不同（悬停微调、抗风、打 yaw），
  sum(omega_i * sigma_i) 不再为零，陀螺力矩出现。

  量化一下：设桨 i 的转速为 omega_i = omega_0 (1 + d_i)，d_i 是均值 0、
  标准差 s 的随机相对偏差（悬停微调 / 抗风 / 打 yaw 都会产生）。
      有效残余 / omega_0 = |sum_i sigma_i * d_i|

  由于 sum(sigma)=0，常数项没了，剩下的就是一个加权随机游走。
""")
    rng = np.random.default_rng(7)
    s = 0.15                       # 15% 的转速散布，工程上不算小
    print("  取转速相对散布 s = %.2f，每档模拟 20000 次：" % s)
    print()
    print("      %-10s %-3s %14s %14s %14s"
          % ("布局", "N", "实测均值", "理论 sqrt(2N/pi)*s", "相对 N=4"))
    print("      " + "-" * 62)
    base = None
    for name in ["QUAD_X", "HEXA_X", "OCTA_X"]:
        B, _ = all_frames()[name]
        sig = B[2]
        n = len(sig)
        vals = np.array([abs(np.sum(sig * rng.normal(0, s, n)))
                         for _ in range(20000)])
        theory = math.sqrt(2 * n / math.pi) * s
        if base is None:
            base = vals.mean()
        print("      %-10s %-3d %14.4f %14.4f %13.3fx"
              % (name, n, vals.mean(), theory, vals.mean() / base))
    print("""
  => 残余随 **sqrt(N)** 增长，不是常数。
     四轴 15%% 的转速散布 -> 残余约 %.3f；八轴涨到约 %.3f（%.2f 倍）。
""" % (math.sqrt(2 * 4 / math.pi) * s, math.sqrt(2 * 8 / math.pi) * s,
       math.sqrt(8 / 4.0)))
    print("""
  ⚠ 严格说这里还有一层：`sum(sigma_i d_i)` 只是**同一个 omega_body 下**
     各桨的角动量不平衡。真实的陀螺力矩还要乘 omega_body
     和 J_prop，所以「大机器翻跟头更费劲」的主因仍然是实验 3 的惯量，
     这一项是**次要修正**。

  ★ 但结论本身值得记住：**悬停时它严格为零，机动时按 sqrt(N) 长大。**
    零的那一半是「邻桨反旋向」白送的；长的这一半是 N 的代价。
""")

    sub("(d) 与 02 课的连接")

    print("""
  sum(sigma) = 0 这一个条件，同时保证了**两件完全不同的事**：

      ① 02 课 §4.2.4：yaw 行与 throttle 行不共线 => rank(B) = 4
      ② 本实验：悬停稳态下螺旋桨陀螺力矩自动抵消

  它们都是「邻桨反旋向」的推论，但一个是**代数**后果，
  一个是**动力学**后果 —— 同一句话管两件事。
""")


# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 通用 N 旋翼力与力矩", exp1),
    "2": ("★★★ 转动惯量：从几何推出来", exp2),
    "3": ("★★ 效能 vs 惯量：yaw 瓶颈", exp3),
    "4": ("★★ 非均匀布局的正交性", exp4),
    "5": ("★★★ 共轴（Y6 / X8）", exp5),
    "6": ("★ 陀螺项", exp6),
}


def main():
    print()
    hr("#")
    print("#  第 9.5 周 · 03 六轴八轴动力学建模")
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
