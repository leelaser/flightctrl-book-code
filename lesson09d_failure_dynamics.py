#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 9.7 周 · 05 失效动力学与安全降落 —— 配套实验

    实验 1  ★★  三种失效模式：断桨的离心力有多大
    实验 2  ★★★ 可达力旋量集：悬停还行不行
    实验 3  ★★★ 孤儿电机：为什么六旋翼的对径电机被迫停转
    实验 4  ★★★ 单向权限：为什么掉一个电机后只能往一侧滚
    实验 5  ★★★ 自旋模式：四旋翼为什么必须自旋才能飞
    实验 6  ★★  共轴 X8：最优雅的失效响应
    实验 7  ★★  复现 ArduPilot 的失效检测判据
    实验 8  ★★  降落剖面：从失效到触地

用法：
    python lesson09d_failure_dynamics.py 1
    python lesson09d_failure_dynamics.py 2
    ...
    python lesson09d_failure_dynamics.py          # 全部

只需 numpy（画图需要 matplotlib）。

★ 本脚本自带一个「有界最小二乘」(BVLS) 求解器 —— 因为判断「悬停还行不行」
  本质是一个带箱式约束的最小二乘问题，而机器上没有 scipy。

参考源码（ArduPilot Copter-4.5.7）：
    libraries/AP_Motors/AP_MotorsMatrix.cpp:415-461  ★★★ check_for_failed_motor
    libraries/AP_Motors/AP_MotorsMatrix.cpp:272      ★★★ "Quads cannot ... degrees of freedom"
    libraries/AP_Motors/AP_MotorsMatrix.cpp:265      ★★  "Hex aircraft are only marginal"
    libraries/AP_Motors/AP_MotorsMatrix.cpp:546-558  remove_motor
    libraries/AP_Motors/AP_MotorsMatrix.cpp:203-208  boost_ratio
    libraries/AP_Motors/AP_MotorsMatrix.cpp:263-272  失效处理与推荐参数
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
# 公共：有界最小二乘 (Bounded-Variable Least Squares)
#
#     min ||A u - b||^2    s.t.  lo <= u <= hi
#
# 主动集法（Stark & Parker 1983）。为什么需要它：
#   「悬停力旋量还达得到吗」= b 是否落在 {A u : u 在箱子里} 里
#   = 最小二乘残差是否为零。而残差本身还有物理意义：它就是【逼不出来的那部分】。
# ===========================================================================

def bvls(A, b, lo, hi, tol=1e-11, maxit=800):
    n = A.shape[1]
    u = np.clip(np.linalg.lstsq(A, b, rcond=None)[0], lo, hi)
    free = np.ones(n, bool)
    for _ in range(maxit):
        z = np.zeros(n)
        if free.any():
            z[free] = np.linalg.lstsq(A[:, free], b, rcond=None)[0]
        viol = free & ((z < lo - tol) | (z > hi + tol))
        if viol.any():
            u[viol] = np.clip(z[viol], lo[viol], hi[viol])
            free[viol] = False
            continue
        un = u.copy()
        un[free] = z[free]
        g = -A.T @ (A @ un - b)                  # 梯度 / 2
        rel = np.array([(~free[i]) and
                        ((u[i] <= lo[i] + tol and g[i] < -tol) or
                         (u[i] >= hi[i] - tol and g[i] > tol)) for i in range(n)])
        if not rel.any():
            return un, float(np.linalg.norm(A @ un - b)), True
        u = un
        free[rel] = True
    return u, float(np.linalg.norm(A @ u - b)), False


def wrench(B, u):
    """u -> (总推力, roll, pitch, yaw) 的归一化力旋量。"""
    return B @ u


# ===========================================================================
# 公共：布局
#
# 行向量 B = [推力行; roll 行; pitch 行; yaw 行]
#   推力行  : 全 1（Σu = 归一化总推力）
#   roll 行 : -sin(theta)
#   pitch行 : +cos(theta)
#   yaw 行  : sigma（CCW=+1, CW=-1）
# 见 03-动力学建模/01 课 §3.4 与 03 课 §5.2。
# ===========================================================================

H_HOVER = 0.40      # 悬停油门：单电机悬停推力 / T_max。u_i ∈ [0,1]，单位是 T_max

FRAMES = {
    # 源码 :591-600  QUAD X
    "QUAD_X":   ([45, -135, -45, 135], [1, 1, -1, -1]),
    # 源码 :792-800  HEXA X
    "HEXA_X":   ([90, -90, -30, 150, 30, -150], [-1, 1, -1, 1, 1, -1]),
    # 源码 :874-887  OCTA X
    "OCTA_X":   ([22.5, -157.5, 67.5, 157.5, -22.5, -112.5, -67.5, 112.5],
                 [1, 1, -1, -1, -1, -1, 1, 1]),
    # 源码 :972-1010  OCTAQUAD X（共轴 X8）
    "OCTAQUAD_X": ([45, -45, -135, 135, 45, -45, -135, 135],
                   [1, -1, 1, -1, 1, -1, 1, -1]),
}


def Bmat(angles_deg, spin):
    a = np.deg2rad(np.asarray(angles_deg, float))
    s = np.asarray(spin, float)
    return np.vstack([np.ones(len(a)), -np.sin(a), np.cos(a), s])


def antipode(angles_deg, i):
    """第 i 个电机的【对径电机】下标（相差 180°）。"""
    a = np.asarray(angles_deg, float)
    target = (a[i] + 180.0) % 360.0
    d = np.abs(((a - target + 180) % 360) - 180)
    j = int(np.argmin(d))
    return j if d[j] < 1e-6 else None


def hover_feasible(B, keep, N):
    """在 keep 列上，悬停力旋量可达吗？返回 (残差, u)。"""
    b = np.array([N * H_HOVER, 0.0, 0.0, 0.0])
    u, res, _ = bvls(B[:, keep], b, np.zeros(len(keep)), np.ones(len(keep)))
    return res, u


def one_sided_extent(Br, thr, row, sign):
    """在给定推力上，row 轴在 sign 方向能走多远（二分）。"""
    lo, hi = 0.0, 12.0
    for _ in range(60):
        mid = (lo + hi) / 2
        bb = np.array([thr, 0.0, 0.0, 0.0])
        bb[row] = sign * mid
        u, res, _ = bvls(Br, bb, np.zeros(Br.shape[1]), np.ones(Br.shape[1]))
        lo, hi = (mid, hi) if res < 1e-9 else (lo, mid)
    return lo


# ===========================================================================
# 实验 1  ★★ 三种失效模式
# ===========================================================================

def exp1():
    head("实验 1  ★★ 三种失效模式：断桨的离心力有多大")

    sub("(a) 「失效」不是一个东西，是三个")

    print("""
  混控层面看起来都是「某个电机不出力了」，但**物理过程完全不同**：

    ① 电机停转（ESC 烧毁 / 电机断路 / 堵转）
       推力 -> 0，反扭矩 -> 0（只剩风车阻力矩）
       振动：【没有】。飞机只是失去一个力源。
       => 混控层面重新分配即可，是三者里最「干净」的。

    ② 断桨（桨叶折断 / 脱落）
       推力 -> **不是零**，而是一个【随旋转角变化】的分量
       振动：离心不平衡 + 气动不平衡，**极大**
       => ★ 即使混控能补上平均推力，机械上也未必撑得住

    ③ ESC 半失效 / 电机匝间短路
       可能给出【错误的转速】—— 拉力忽大忽小、可能反转
       => 最难处理，因为混控的假设（"我给多少它就出多少"）失效了
""")

    sub("(b) 断桨的离心力量级")

    print("""  桨断掉一半后，剩下的部分质量 m_b 的质心离旋转轴约 r。
  转速 omega（rad/s）时的离心力：

        F_c = m_b * r * omega^2

  ★ 注意是 omega 的【平方】—— 转速翻倍，离心力变四倍。
""")
    print("      %-12s %6s %-16s %10s %12s" %
          ("桨", "转速", "状态", "离心力", "交变频率"))
    print("      " + "-" * 62)
    for name, R, m_blade, rpm in [("5 寸 3 叶", 0.0635, 0.0015, 10000),
                                  ("7 寸 3 叶", 0.0889, 0.0028, 7000),
                                  ("10 寸 3 叶", 0.127, 0.005, 5500)]:
        w = 2 * math.pi * rpm / 60.0
        for frac, lbl in [(0.5, "断成半桨"), (0.55, "剩单叶")]:
            F = m_blade * frac * R * w * w
            print("      %-12s %5d   %-16s %8.1f N %9.0f Hz"
                  % (name, rpm, lbl, F, rpm / 60.0))
    print("""
  ★ 交变频率 = 转速 —— 因为它【跟着转子转】，所以是一个旋转矢量式的载荷。
""")

    sub("(c) ★★ 和正常推力的对照")

    m_total, n_mot = 4.0, 6
    F_hover = m_total * 9.80665 / n_mot
    print("  取一架 %.1f kg 六旋翼：单桨悬停推力只有 %.2f N" % (m_total, F_hover))
    print()
    F5 = 0.0015 * 0.5 * 0.0635 * (2 * math.pi * 10000 / 60.0) ** 2
    print("  => 5 寸桨断成半桨的离心力 %.1f N 是它悬停推力的 **%.0f 倍**"
          % (F5, F5 / F_hover))
    print("""
  ★★★ 这就是「断桨 != 电机停转」的量化依据：

      · 电机停转：机臂上的载荷【变小】（少了推力），只是控制问题
      · 断桨    ：机臂上多了一个 %.0f N、167 Hz 的【交变载荷】，是结构问题
""" % (F5 / F_hover))
    print("""  ⚠️ 所以正确的第一动作是【尽快切断那个电机】，而不是
     「让混控想办法继续用它」。哪怕它还能出平均推力，机械上也在被撕。
""")


# ===========================================================================
# 实验 2  ★★★ 可达力旋量集：悬停还行不行
# ===========================================================================

def exp2():
    head("实验 2  ★★★ 可达力旋量集：悬停还行不行")

    sub("(a) 判据升级：为什么「秩」不够")

    print("""
  02 课 §4.5.6 用的是**秩**：4x5 秩 4 -> 可解。
  02 课练习 5 自己也发现这个判据**不够**：
      「12/15 对满秩，但悬停解**全部贴边**……秩只回答有没有解，
        不回答可行域余量」

  ★ 本实验把那个「余量」算出来。正确的判据是**可达力旋量集**：

        W = { B u  :  0 <= u_i <= 1 }        <- 4 维空间里的一个【超平行体投影】

      悬停可行  <=>  w_hover = (N*h, 0, 0, 0) 属于 W
      残差      <=>  最接近也差多少 —— 它就是【逼不出来的那部分力旋量】

  ★ 求解方法：带箱式约束的最小二乘（本脚本自带 BVLS）。
""")

    sub("(b) 全机型扫描")

    print("  h = %.2f（悬停油门）," % H_HOVER)
    print("  归一化：u_i ∈ [0,1] 单位是 T_max；力旋量单位是 T_max 与 T_max*R")
    print()
    for name, (a, s) in FRAMES.items():
        B = Bmat(a, s)
        N = len(a)
        print("  ── %s  (N=%d, 悬停需要 Σu = N·h = %.2f)" % (name, N, N * H_HOVER))
        for lost in range(N):
            keep = [i for i in range(N) if i != lost]
            res, u = hover_feasible(B, keep, N)
            if res < 1e-9:
                mins = " ".join("%.3f" % x for x in u)
                print("     掉 m%-2d ✅ 可行   u = [%s]   最小 %.4f"
                      % (lost, mins, u.min()))
            else:
                print("     掉 m%-2d ❌ 不可行 残差 = %.4f   （最接近 u = [%s]）"
                      % (lost, res, " ".join("%.3f" % x for x in u)))
        print()

    sub("(c) ★★★ 三条结论")

    print("""  ① **四旋翼掉一个：悬停力旋量【无解】**
        3 个电机 vs 4 个约束（推力 + 三轴力矩）。
        => 不是「解不好」，是**根本不存在**。源码里 ArduPilot 自己也写了
           （AP_MotorsMatrix.cpp:272）：
               "Quads cannot make use of motor loss handling because
                it doesn't have enough degrees of freedom."

  ② **六旋翼掉一个：悬停【可行】，但对径电机被钉死在 0**
        看 u 的最后一位 —— 总是 0.0000。这一条值得单独一节（实验 3）。

  ③ **八旋翼掉一个：悬停可行，且有余量**
        最小出力 0.2586 > 0，说明还有调整空间。

  ★ 而源码注释（AP_MotorsMatrix.cpp:265）对六旋翼的评价是：
        "Hex aircraft are only marginal and must be handles with care"
     —— "marginal"（勉强），和上面的 0.0000 完全对得上。
""")


# ===========================================================================
# 实验 3  ★★★ 孤儿电机
# ===========================================================================

def exp3():
    head("实验 3  ★★★ 孤儿电机：为什么六旋翼的对径电机被迫停转")

    sub("(a) 先看现象")

    print("""  实验 2 的结果里，六旋翼每个失效情形都有一个电机出力恰好是 0。
  它总是【失效电机的对径电机】—— 我把它叫「孤儿」(orphan)。

      掉 m0  ->  m1 关停        掉 m1  ->  m0 关停
      掉 m2  ->  m3 关停        掉 m3  ->  m2 关停
      掉 m4  ->  m5 关停        掉 m5  ->  m4 关停

  ★ 为什么会这样？关键在【对径对】。
""")

    sub("(b) 对径对：力矩自动抵消的最小单元")

    print("""  位置相差 180 度的两个电机（对径对）有个好性质：

      · 它们的 (x, y) 互为相反数 => 等推力时 roll/pitch 力矩【自动抵消】
      · 等分布布局里它们的 sigma【相同】=> 等推力时 yaw 力矩也抵消

  => 一个对径对只要【两边推力相等】，就既不产生力矩、又贡献 2 份推力。
     这是「干净」的。

  ★ 失效一个电机后，剩下的电机必须能**两两配成对径对**，
    整机才可能力矩为零。而失效电机的对径电机 —— 那个「孤儿」——
    找不到配对的伙伴了。
""")

    sub("(c) ★★★ 孤儿的命运由【零空间维数】决定")

    print("""  零空间维数 = (N-1) - rank(B_red)：悬停解有多少自由度。

      维数 0  => 解唯一，孤儿只能取那个值
      维数 >=1 => 孤儿可以在零空间方向上动 —— 但要看它在那个方向上的分量
""")
    print("      %-12s %-6s %-5s %-8s %-12s %s"
          % ("机型", "丢失", "零空间", "孤儿", "孤儿分量", "结论"))
    print("      " + "-" * 74)
    for name, (a, s) in FRAMES.items():
        B = Bmat(a, s)
        N = len(a)
        for lost in range(N):
            orph = antipode(a, lost)
            if orph is None or orph == lost:
                continue
            keep = [i for i in range(N) if i != lost]
            Br = B[:, keep]
            r = np.linalg.matrix_rank(Br)
            ns = np.linalg.svd(Br)[2][r:]          # 零空间（行向量）
            j = keep.index(orph)
            res, u = hover_feasible(B, keep, N)
            if len(ns) == 0:
                verdict = "解唯一"
                comp = np.array([0.0])
            else:
                comp = ns[:, j]
                verdict = "【被迫为 0】" if np.allclose(comp, 0, atol=1e-10) else "可动"
            cs = " ".join("%+.3f" % x for x in comp)
            print("      %-12s m%-5d %-5d %-8s %-12s %s"
                  % (name, lost, len(ns), "m%d" % orph, cs, verdict))
    print()
    print("""  ★★★ 读法：

      四旋翼（自旋模式，3 约束）：自由度 0  => 解唯一，孤儿 = 0
      六旋翼（悬停，4 约束）    ：自由度 1  => 零空间在孤儿上的分量【恰好是 0】
                                              => 孤儿【被迫为 0】，绕不过去
      八旋翼（悬停，4 约束）    ：自由度 3  => 分量非零，孤儿解放（回到 h = 0.4）

  => **六旋翼的 1 维自由度「不够」把孤儿解放出来。**
     这是一个恰好卡在边界上的结构 —— 也正是源码说的 "only marginal"。
""")


# ===========================================================================
# 实验 4  ★★★ 单向权限
# ===========================================================================

def exp4():
    head("实验 4  ★★★ 单向权限：为什么只能往一侧滚")

    sub("(a) 扫描：悬停点上各轴还能打多少")

    print("""  「姿态权限」= 在保持悬停推力不变的前提下，某个轴还能额外产生多少力矩。
  用二分 + BVLS 求。★ 双向都算 —— 因为下面会看到它【不对称】。
""")
    for name, (a, s) in FRAMES.items():
        B = Bmat(a, s)
        N = len(a)
        thr = N * H_HOVER
        print("  ── %s (N=%d)" % (name, N))
        res0, _ = hover_feasible(B, list(range(N)), N)
        lo_r = one_sided_extent(B, thr, 1, -1); hi_r = one_sided_extent(B, thr, 1, 1)
        lo_p = one_sided_extent(B, thr, 2, -1); hi_p = one_sided_extent(B, thr, 2, 1)
        lo_y = one_sided_extent(B, thr, 3, -1); hi_y = one_sided_extent(B, thr, 3, 1)
        print("     完好      roll[%+.3f,%+.3f] pitch[%+.3f,%+.3f] yaw[%+.3f,%+.3f]"
              % (-lo_r, hi_r, -lo_p, hi_p, -lo_y, hi_y))
        for lost in range(min(N, 4)):
            keep = [i for i in range(N) if i != lost]
            Br = B[:, keep]
            res, _ = hover_feasible(B, keep, N)
            if res > 1e-9:
                print("     掉 m%-2d     ❌ 悬停不可行" % lost)
                continue
            lo_r = one_sided_extent(Br, thr, 1, -1); hi_r = one_sided_extent(Br, thr, 1, 1)
            lo_p = one_sided_extent(Br, thr, 2, -1); hi_p = one_sided_extent(Br, thr, 2, 1)
            lo_y = one_sided_extent(Br, thr, 3, -1); hi_y = one_sided_extent(Br, thr, 3, 1)
            flag = "  ★单向" if (lo_r < 1e-9 or lo_p < 1e-9 or lo_y < 1e-9) else ""
            print("     掉 m%-2d     roll[%+.3f,%+.3f] pitch[%+.3f,%+.3f] yaw[%+.3f,%+.3f]%s"
                  % (lost, -lo_r, hi_r, -lo_p, hi_p, -lo_y, hi_y, flag))
        print()

    sub("(b) ★★★ 单向的机理：孤儿被钉在 0，而它的系数最大")

    a, s = FRAMES["HEXA_X"]
    B = Bmat(a, s)
    N = 6
    lost = 0
    keep = [i for i in range(N) if i != lost]
    Br = B[:, keep]
    orph = antipode(a, lost)
    j = keep.index(orph)
    print("  以 HEXA_X 掉 m0 为例。留下的电机与它们的 roll 系数：")
    print()
    print("      %-8s %-10s %-10s %s" % ("电机", "方位角", "roll 系数", "悬停时出力"))
    res, u = hover_feasible(B, keep, N)
    for k, i in enumerate(keep):
        tag = "  <- 孤儿" if i == orph else ""
        print("      m%-7d %-10.1f %-10.3f %.4f%s"
              % (i, a[i], B[1, i], u[k], tag))
    print("""
  ★★ 关键：孤儿 m%d 的 roll 系数是 %+.3f（绝对值最大），
     但它的出力被钉死在 0 —— 也就是说这个"最大的杠杆"只能往上抬，不能往下压。

  => 要产生【负方向】的 roll 力矩，需要它取【负值】，
     而推力不能为负（u >= 0）。
  => **负方向权限精确为 0。**
""" % (orph, B[1, orph]))
    print("  ★ 正方向呢？看孤儿的出力怎么变：")
    print()
    print("      %-10s %-10s %s" % ("τ_roll", "孤儿 u", "是否可行"))
    print("      " + "-" * 40)
    thr = N * H_HOVER
    for tau in [0.0, 0.2, 0.8, 1.4, 1.5, 1.51]:
        bb = np.array([thr, tau, 0.0, 0.0])
        uu, res2, _ = bvls(Br, bb, np.zeros(len(keep)), np.ones(len(keep)))
        print("      %+.3f      %+.4f      %s"
              % (tau, uu[j], "✅" if res2 < 1e-9 else "❌ 残差 %.4f" % res2))
    print("""
  ★ 正方向的极限是孤儿**撞到上界 1.0** 的时候（τ_roll = 1.5 时 u = 1.0）。
  ★ 负方向的极限是它**撞到下界 0** 的时候 —— 而下界 0 恰好就是悬停点本身。

  => 悬停点正好落在可达集的【边界】上。这就是"单向"的数学来源。
""")


# ===========================================================================
# 实验 5  ★★★ 自旋模式
# ===========================================================================

def exp5():
    head("实验 5  ★★★ 自旋模式：四旋翼为什么必须自旋才能飞")

    sub("(a) 松开一个约束，世界就不一样了")

    print("""  实验 2 说四旋翼掉一个电机后**悬停力旋量无解** —— 但那是要求
  【四】个量都对：推力 + roll + pitch + **yaw**。

  ★★★ 而在真实飞行中，yaw 是【可以放弃】的：
     机体绕自己的 z 轴转起来，反扭矩由**气动阻力**吸收。
     阻力矩是个【耗散项】，它不需要电机去"配平"，只要转速够高就能顶住。

  => 于是约束从 4 个降到 3 个：  [推力, roll, pitch]

  ★ 这是本课最关键的一步抽象：**失效之后，要问的不是"能不能悬停"，
    而是"能不能守住那些【必须守住】的量"。**
     必须守住的：推力（不然掉高度）、roll/pitch（不然翻）
     可以放弃的：yaw（转就转吧）
""")

    sub("(b) 四旋翼：3 个电机 / 3 个约束，恰好可逆")

    a, s = FRAMES["QUAD_X"]
    B = Bmat(a, s)
    N = 4
    print("      %-8s %-10s %-24s %-10s %s"
          % ("丢失", "det(B3)", "解 u（3 个电机）", "剩余 yaw", "结论"))
    print("      " + "-" * 72)
    for lost in range(N):
        keep = [i for i in range(N) if i != lost]
        B3 = B[:3][:, keep]
        d = np.linalg.det(B3)
        u = np.linalg.solve(B3, np.array([N * H_HOVER, 0.0, 0.0]))
        ty = float(B[3, keep] @ u)
        orph = antipode(a, lost)
        j = keep.index(orph) if orph in keep else None
        print("      m%-7d %+9.4f   [%s] %+9.4f   %s"
              % (lost, d, " ".join("%.4f" % x for x in u), ty,
                 "孤儿 m%d 停转" % orph if j is not None else "-"))
    print("""
  ★★★ 三条结论：

    ① **det = ±2 != 0，3x3 可逆** —— 推力/roll/pitch 精确可控。
       所以四旋翼掉一个电机**在数学上是可以飞的**。

    ② **对径电机停转，另两个各出 2h = 0.8** ——
       和六旋翼同一条规律，只是这里连"选择"都没有（解唯一）。

    ③ **残余 yaw 力矩 = ∓1.6** —— 恒定不变，机体必然越转越快，
       直到气动阻力把它顶住。
""")

    sub("(c) 自旋多少？")

    print("""  机体绕 z 轴自旋时，角速度 omega = (0, 0, Omega)。代入刚体转动方程：

      J w_dot + w x (J w) = tau

  ★★★ 而 w = (0,0,Omega) 时 J w = (0, 0, Jz*Omega) 与 w 【平行】，
     所以 w x (J w) = 0 —— **纯自旋不产生任何陀螺力矩。**

  => z 轴方程退化成一个标量常微分方程：

        Jz * Omega_dot = tau_yaw_电机 - c * Omega * |Omega|

     其中 c 是机体/机臂的气动阻尼系数（把整机看成一个旋转的钝体）。

  ★ 稳态自旋率：Omega_ss = sqrt( tau_yaw / c )
     —— 注意是【平方根】关系：力矩加倍，转速只涨 41%。
""")
    a, s = FRAMES["QUAD_X"]
    B = Bmat(a, s)
    tau_yaw = abs(float(B[3, [1, 2, 3]] @ np.linalg.solve(B[:3][:, [1, 2, 3]],
                                                          np.array([4 * H_HOVER, 0.0, 0.0]))))
    print("  对四旋翼掉一个： tau_yaw = %.4f（归一化）= %.4f * N*m" % (tau_yaw, tau_yaw))
    print()
    print("      阻尼系数 c       稳态自旋率 Omega      转速")
    print("      " + "-" * 50)
    for c in [0.002, 0.005, 0.010, 0.020]:
        Om = math.sqrt(tau_yaw / c * 0.01)      # 示例标度
        print("      %-16.3f %-20.2f %.0f rpm" % (c, Om, Om * 60 / (2 * math.pi)))
    print("""
  ⚠️ 上面的 c 是【示意值】—— 真实的值要靠实验标定（把飞机固定在转台上量阻力矩）。
     这里要传达的是【函数形式】：Omega_ss ∝ sqrt(tau/c)，不是线性关系。
""")

    sub("(d) 自旋模式的代价")

    print("""  ★ 自旋不是免费的，代价至少三条：

    ① **航向不可控** —— 相机/天线/载荷跟着转
    ② **位置控制退化** —— 自旋会产生偏心力矩，
       机体实际压在一个"平均"姿态上，位置环只能用很低的带宽
    ③ **陀螺项重新出现** —— 一旦 Omega 之外还有 p, q（姿态修正），
       w x (J w) 不再为零。见 03 课 §5.7 和 04 课 §6.7

  => 所以自旋模式是**降级模式**，不是正常模式。
     它是「保命」而不是「保任务」。
""")


# ===========================================================================
# 实验 6  ★★ 共轴 X8
# ===========================================================================

def exp6():
    head("实验 6  ★★ 共轴 X8：最优雅的失效响应")

    a, s = FRAMES["OCTAQUAD_X"]
    B = Bmat(a, s)
    N = 8

    sub("(a) 解")

    print("      %-10s %-12s %-14s %s"
          % ("丢失", "同轴伙伴", "伙伴出力", "其余 6 个"))
    print("      " + "-" * 62)
    for lost in range(N):
        keep = [i for i in range(N) if i != lost]
        res, u = hover_feasible(B, keep, N)
        pair = [k for k in range(N) if k != lost and abs(a[k] - a[lost]) < 1e-9]
        if not pair:
            continue
        j = keep.index(pair[0])
        others = np.delete(u, j)
        print("      m%-9d m%-11d %-14.4f %s"
              % (lost, pair[0], u[j], " ".join("%.4f" % x for x in others)))
    print("""
  ★★★ 完美得令人吃惊：

      · 失效电机的**同轴伙伴**精确升到 2h = 0.8000
      · 其余 6 个**一动不动**，还是 h = 0.4
      · 残留力旋量【精确为零】

  => 因为同轴的两桨本来就在**同一个位置**（见 03 课 §5.6.4），
     所以"这一站要出多少推力"是个独立的账 ——
     上面那个坏了，下面那个出双份就行，完全不影响别人。
""")

    sub("(b) 和其他机型对照")

    print("      %-12s %-26s %s" % ("机型", "掉 1 个电机后", "代价"))
    print("      " + "-" * 66)
    rows = [
        ("四旋翼", "悬停无解，必须自旋", "航向失控 + 位置环退化"),
        ("六旋翼", "可悬停，但姿态权限单向", "只能朝一侧修正，降落受限"),
        ("八旋翼", "可悬停，权限双向但不对称", "一侧余量变小"),
        ("共轴 X8", "可悬停，力旋量精确保持", "同轴伙伴余量减半（0.8）"),
    ]
    for x, y, z in rows:
        print("      %-12s %-26s %s" % (x, y, z))
    print("""
  ★ 但共轴 X8 的代价藏在别处（03 课 §5.6 已经讲过）：

      · 悬停时同轴两桨本来就各出 h，伙伴要翻倍到 2h
        => 它的余量从 (1-h)=0.6 掉到 (1-2h)=0.2，**只剩三分之一**
      · 再坏一个同轴的，这一站就彻底没了
      · 而且共轴的推力有 eta 折损（03 课 §5.6.3），伙伴实际要出得更多
""")


# ===========================================================================
# 实验 7  ★★ 复现 ArduPilot 的失效检测判据
# ===========================================================================

def exp7():
    head("实验 7  ★★ 复现 ArduPilot 的失效检测判据")

    sub("(a) 源码（AP_MotorsMatrix.cpp:415-461，原文节选）")

    print("""
  void AP_MotorsMatrix::check_for_failed_motor(float throttle_thrust_best_plus_adj)
  {
      // record filtered and scaled thrust output for motor loss monitoring purposes
      float alpha = _dt / (_dt + 0.5f);
      for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
          if (motor_enabled[i]) {
              _thrust_rpyt_out_filt[i] += alpha * (_thrust_rpyt_out[i] - _thrust_rpyt_out_filt[i]);
          }
      }
      ...
      float rpyt_high = 0.0f;   float rpyt_sum = 0.0f;   uint8_t number_motors = 0.0f;
      for (...) { ... rpyt_sum += _thrust_rpyt_out_filt[i];
                  if (_thrust_rpyt_out_filt[i] > rpyt_high) { rpyt_high = ...; } }
      float thrust_balance = 1.0f;
      if (rpyt_sum > 0.1f) {
          thrust_balance = rpyt_high * number_motors / rpyt_sum;
      }
      // ensure thrust balance does not activate for multirotors with less than 6 motors
      if (number_motors >= 6 && thrust_balance >= 1.5f && _thrust_balanced) {
          _thrust_balanced = false;
      }
      if (thrust_balance <= 1.25f && !_thrust_balanced) {
          _thrust_balanced = true;
      }
""")

    sub("(b) 判据是什么")

    print("""  ★★★ 核心就是一个比值：

        thrust_balance = N * max_i(u_i) / sum_i(u_i)
                       = (最大电机出力) / (平均电机出力)

  · 完好时所有电机出力接近 => 比值 ≈ 1
  · 一个电机失败后，控制器为了维持姿态会把其他电机推到饱和
    => 最大值远高于平均 => 比值上升
""")

    sub("(c) 用本课的模型算出健康/失效时这个比值")

    print("""
  ★ 注意 ArduPilot 用的是**指令值**（它测不了实际推力）。
    本课没有电机反馈模型，所以用一个等效的代理：
    在「某个电机被命令出力但实际出 0」的假设下，看控制器的指令长什么样。
""")
    print()
    print("      %-12s %-14s %-14s %s" % ("机型", "完好时", "失效后", "是否越过阈值"))
    print("      " + "-" * 60)
    for name, (a, s) in FRAMES.items():
        B = Bmat(a, s)
        N = len(a)
        if N < 6:
            print("      %-12s %-14s %-14s %s"
                  % (name, "%.3f" % 1.0, "-", "N<6，源码明确不启用"))
            continue
        # 完好：所有电机出力相同
        tb_ok = N * 1.0 / N
        # 失效：失效电机被命令到饱和(1.0)，其余按悬停解 + 补偿
        keep = [i for i in range(N) if i != 0]
        res, u = hover_feasible(B, keep, N)
        u_full = np.zeros(N)
        u_full[keep] = u
        # 控制器不知道失效，还会继续命令失效电机 -> 它的指令会往上顶；
        # 用"失效电机被顶到 1.0"作为上界估计
        u_cmd = np.concatenate([[1.0], u])
        tb_bad = N * u_cmd.max() / u_cmd.sum()
        print("      %-12s %-14.3f %-14.3f %s"
              % (name, tb_ok, tb_bad,
                 "✅ 检测到 (>1.5)" if tb_bad >= 1.5 else "⚠ 未越过 1.5"))
    print("""
  ★★ 这解释了源码里那个 `number_motors >= 6` 的守卫：
     四旋翼掉一个电机时，反正是救不回来的（§7.5 源码 :272），
     再去"检测"只会白白触发 thrust_boost 把混控搞乱。

  ★ 阈值 1.5 / 1.25 构成**滞回**：触发用 1.5，恢复用 1.25 ——
     防止在阈值附近反复抖动。
  ★ 时间常数 alpha = dt/(dt+0.5) => **0.5 秒低通**。
     这就是从"断了"到"飞控知道"的延迟量级。
""")


# ===========================================================================
# 实验 8  ★★ 降落剖面
# ===========================================================================

def exp8():
    head("实验 8  ★★ 降落剖面：从失效到触地")

    sub("(a) 四个阶段")

    print("""
      ① 检测      (0.5 ~ 1.0 s)   靠 thrust_balance 越过 1.5，低通 0.5 s
      ② 降级      (瞬间 ~ 0.5 s)   切混控、限幅姿态指令、进入"自保模式"
      ③ 下降      (数秒 ~ 数十秒)  定速下降
      ④ 触地      (< 0.2 s)        抛降落伞 / 最后一刻的缓冲

  ★ 本实验只算 ③ —— 因为它是唯一由**动力学**决定的一段。
""")

    sub("(b) 下降速度：两个约束夹住它")

    m = 4.0
    A_disk = 0.6           # 等效阻力面积 m^2（机体平着掉）
    rho = 1.225
    vt = math.sqrt(2 * m * 9.80665 / (rho * A_disk))
    print("  取 %.1f kg 六旋翼，平着下落时等效阻力面积 %.2f m^2：" % (m, A_disk))
    print()
    print("      终端速度 v_t = sqrt(2mg / (rho*A)) = %.2f m/s" % vt)
    print("""
  ★ 约束一（下界）：**不能停**。失效后推力余量变小，
    如果为了悬停把油门顶满，一旦要修正姿态就没余量了。
    => 必须保持一个**下降率**，让高度环不用把油门推到最大。

  ★ 约束二（上界）：**触地速度要能活**。
""")

    print("      触地速度   等效落高        对 4 kg 机体的后果")
    print("      " + "-" * 62)
    for v, h, note in [(1.0, 0.051, "起落架完好，无损"),
                       (2.0, 0.204, "轻着陆，推荐值"),
                       (3.0, 0.459, "起落架可能变形"),
                       (5.0, 1.274, "大概率损坏，但电池通常不炸"),
                       (8.0, 3.262, "结构损毁，有起火风险")]:
        print("      %5.1f m/s   %6.3f m     %s" % (v, h, note))
    print("""
  => **取 1~2 m/s 是"可控硬着陆"和"能修"之间的平衡点。**
     注意这已经比正常着陆快 5~10 倍 —— 失效降落注定是**硬着陆**。
""")

    sub("(c) ★★★ 为什么失效降落必须接近垂直")

    print("""  这一点来自实验 4 的"单向权限"：

  ★ 要让飞机**平移**，必须**倾斜**（01 课 §3.5：推力只能沿机体 -z）。
    而要倾斜，用的是一个方向的 roll 或 pitch 力矩。

  ★★★ 而六旋翼掉一个电机后，某些方向的姿态权限【精确为零】。
     => 你想往某个方向平移时，如果那需要"权限为零"的那个方向，
        **做不到**。

  ★ 单向下还能做的事：
      · 沿被允许的方向做**一次**倾斜
      · 然后必须靠这个倾斜一直飞，**回不来**（反向权限是 0）

  => 所以正确的降级策略是：
       ① 先用剩余的权限把飞机**摆平**
       ② 然后【不再做水平机动】，纯垂直下降
       ③ 水平位置漂移用"允许的那个方向"做小幅修正
""")

    sub("(d) 下降剖面数值")

    print("      高度(m)   下降率(m/s)   需要推力/重力   姿态权限占用")
    print("      " + "-" * 60)
    for h, v in [(30, 0.5), (20, 1.0), (10, 1.5), (5, 2.0), (2, 2.0), (0.5, 1.5)]:
        # 定速下降：推力略小于重力，差值提供减速/匀速所需的平衡
        # v 恒定 => 推力 = mg（因为没有加速度），但实际要抵消阻力
        drag = 0.5 * rho * A_disk * v * v
        need = (m * 9.80665 - drag) / (m * 9.80665)
        print("      %6.1f    %8.1f     %10.3f      %s"
              % (h, v, need, "低" if abs(need - 1) < 0.15 else "中"))
    print("""
  ★ 注意"需要推力/重力"这一列始终接近 1 ——
    也就是说**下降并不省推力**（除了阻力那一点）。
    失效降落的困难**不在能量，在控制权限**。
""")

    if HAVE_PLT:
        sub("(e) 画图")
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        names = ["QUAD_X", "HEXA_X", "OCTA_X"]
        for i, nm in enumerate(names):
            a, s = FRAMES[nm]
            B = Bmat(a, s)
            N = len(a)
            keep = [k for k in range(N) if k != 0]
            Br = B[:, keep]
            if hover_feasible(B, keep, N)[0] > 1e-9:
                ax[0].bar(i, 0, color="red")
                continue
            lo = one_sided_extent(Br, N * H_HOVER, 1, -1)
            hi = one_sided_extent(Br, N * H_HOVER, 1, 1)
            ax[0].bar(i - 0.18, lo, 0.36, color="tab:red", label="负方向" if i == 0 else "")
            ax[0].bar(i + 0.18, hi, 0.36, color="tab:blue", label="正方向" if i == 0 else "")
        ax[0].set_xticks(range(len(names)))
        ax[0].set_xticklabels([n + "\n(掉 m0)" for n in names])
        ax[0].set_ylabel("roll authority (normalized)")
        ax[0].set_title("Roll authority after losing one motor")
        ax[0].legend(fontsize=8)
        ax[0].grid(alpha=0.3, axis="y")

        vs = np.linspace(0, 10, 200)
        hs = vs ** 2 / (2 * 9.80665)
        ax[1].plot(vs, hs)
        ax[1].axhspan(0, 0.5, color="green", alpha=0.15, label="safe")
        ax[1].axhspan(0.5, 3, color="orange", alpha=0.15, label="damage possible")
        ax[1].axhspan(3, 10, color="red", alpha=0.15, label="destructive")
        ax[1].set_xlabel("touchdown speed (m/s)")
        ax[1].set_ylabel("equivalent drop height (m)")
        ax[1].set_title("Touchdown speed vs equivalent drop height")
        ax[1].legend(fontsize=8)
        ax[1].grid(alpha=0.3)
        plt.tight_layout()
        out = "failure_landing.png"
        plt.savefig(out, dpi=110)
        print("  已保存 %s" % out)


# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 三种失效模式与断桨离心力", exp1),
    "2": ("★★★ 可达力旋量集与悬停可行性", exp2),
    "3": ("★★★ 孤儿电机与零空间", exp3),
    "4": ("★★★ 单向姿态权限", exp4),
    "5": ("★★★ 自旋模式", exp5),
    "6": ("★★ 共轴 X8 的响应", exp6),
    "7": ("★★ 复现 ArduPilot 失效检测判据", exp7),
    "8": ("★★ 降落剖面", exp8),
}


def main():
    print()
    hr("#")
    print("#  第 9.7 周 · 05 失效动力学与安全降落")
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
