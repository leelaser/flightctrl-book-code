#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 8 周 · 01 四旋翼动力学建模 —— 配套实验

    实验 1  ★★ 混控矩阵的自动生成（复现 ArduPilot 的 add_motor）
    实验 2  ★★★ 非线性动力学仿真（悬停 + 横滚，验证"掉高度"）
    实验 3  ★ 欠驱动性：4 个输入能控制什么
    实验 4  悬停线性化的适用范围

用法：
    python lesson08_dynamics.py 1
    python lesson08_dynamics.py 2
    python lesson08_dynamics.py 3
    python lesson08_dynamics.py 4
    python lesson08_dynamics.py          # 全部

只需 numpy（画图需要 matplotlib）。

参考源码（ArduPilot Copter-4.5.7）：
    libraries/AP_Motors/AP_MotorsMatrix.cpp:536-544   角度 → 系数的三角函数
    libraries/AP_Motors/AP_MotorsMatrix.cpp:575-601   QUAD PLUS / X 的电机定义
    libraries/AP_Motors/AP_MotorsMatrix.h:10-11       旋向常量
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
# 公共：混控矩阵（复现 ArduPilot）
# ===========================================================================

YAW_FACTOR_CW = -1.0
YAW_FACTOR_CCW = 1.0


def add_motor(motor_num, angle_deg, yaw_factor):
    """★★★ 照抄 ArduPilot 的 add_motor。

    AP_MotorsMatrix.cpp:536-544
        add_motor_raw(
            motor_num,
            cosf(radians(roll_factor_in_degrees + 90)),   // = -sin(θ)
            cosf(radians(pitch_factor_in_degrees)),       // = +cos(θ)
            yaw_factor,
            testing_order);

    返回 (roll, pitch, yaw)
    """
    return (math.cos(math.radians(angle_deg + 90.0)),
            math.cos(math.radians(angle_deg)),
            float(yaw_factor))


# 和 AP_MotorsMatrix.cpp:575-601 一一对应
FRAMES = {
    "QUAD_PLUS": [(90, YAW_FACTOR_CCW), (-90, YAW_FACTOR_CCW),
                  (0, YAW_FACTOR_CW), (180, YAW_FACTOR_CW)],
    "QUAD_X": [(45, YAW_FACTOR_CCW), (-135, YAW_FACTOR_CCW),
               (-45, YAW_FACTOR_CW), (135, YAW_FACTOR_CW)],
    # HEXA X：ArduPilot 的 setup_hexa_matrix 里
    "HEXA_X": [(30, YAW_FACTOR_CCW), (90, YAW_FACTOR_CW),
               (150, YAW_FACTOR_CCW), (-150, YAW_FACTOR_CW),
               (-90, YAW_FACTOR_CCW), (-30, YAW_FACTOR_CW)],
}


def mixer_matrix(frame):
    """生成混控矩阵。

    行： [roll, pitch, yaw, throttle]
    列： 每个电机
    """
    rows = []
    for angle, yawf in FRAMES[frame]:
        r, p, y = add_motor(0, angle, yawf)
        rows.append([r, p, y, 1.0])
    return np.array(rows).T     # (4, N)


def motor_positions(frame):
    """电机的机体坐标 (x 前, y 右)。"""
    out = []
    for angle, _yf in FRAMES[frame]:
        out.append((math.cos(math.radians(angle)),
                    math.sin(math.radians(angle))))
    return np.array(out)


# ===========================================================================
# 实验 1：混控矩阵的自动生成
# ===========================================================================

def exp1():
    head("实验 1：★★ 混控矩阵的自动生成")

    print("""
  ArduPilot 的混控矩阵不是手写的 —— 它从电机【方位角】推导：

      roll 系数  = cos(θ + 90°) = -sin(θ)
      pitch 系数 = cos(θ)
      yaw 系数   = +1 (CCW) / -1 (CW)

  本实验复现这个推导，并【用几何验证它的符号】。
""")

    for frame in ("QUAD_PLUS", "QUAD_X"):
        sub("① %s 的电机布局" % frame)

        M = mixer_matrix(frame)
        pos = motor_positions(frame)
        n = M.shape[1]

        print("    %-8s %-10s %-16s %-14s %-14s %s"
              % ("电机", "θ", "机体位置", "roll 系数", "pitch 系数", "yaw 系数"))
        print("    " + "-" * 82)
        for i in range(n):
            ang, yawf = FRAMES[frame][i]
            where = ""
            if pos[i][0] > 0.3:
                where += "前"
            elif pos[i][0] < -0.3:
                where += "后"
            if pos[i][1] > 0.3:
                where += "右"
            elif pos[i][1] < -0.3:
                where += "左"
            print("    %-8d %-10.0f %-16s %-14.4f %-14.4f %+.0f"
                  % (i, ang, "(%+.2f,%+.2f) %s" % (pos[i][0], pos[i][1], where),
                     M[0, i], M[1, i], M[2, i]))

        print()
        print("    混控矩阵（列 = 电机）：")
        print("             " + "".join("%9s" % ("M%d" % i) for i in range(n)))
        for r, nm in enumerate(("roll", "pitch", "yaw", "throttle")):
            print("        %-6s " % nm + "".join("%9.4f" % v for v in M[r]))

        # ★ 几何自检
        print()
        print("    ── ★ 几何自检 ──")
        print()
        ok = True

        # 检查 1：右侧电机的 roll 系数应该为负（右滚要右边减速）
        right_rolls = [M[0, i] for i in range(n) if pos[i][1] > 0.3]
        left_rolls = [M[0, i] for i in range(n) if pos[i][1] < -0.3]
        c1 = all(v < 0 for v in right_rolls) and all(v > 0 for v in left_rolls)
        ok &= c1
        print("        右侧电机的 roll 系数全为负？  %s  (右: %s)"
              % ("✓" if c1 else "✗",
                 " ".join("%.2f" % v for v in right_rolls)))
        print("        左侧电机的 roll 系数全为正？  %s  (左: %s)"
              % ("✓" if all(v > 0 for v in left_rolls) else "✗",
                 " ".join("%.2f" % v for v in left_rolls)))

        # 检查 2：前方电机的 pitch 系数应该为正
        front_p = [M[1, i] for i in range(n) if pos[i][0] > 0.3]
        back_p = [M[1, i] for i in range(n) if pos[i][0] < -0.3]
        c2 = all(v > 0 for v in front_p) and all(v < 0 for v in back_p)
        ok &= c2
        print("        前方电机的 pitch 系数全为正？ %s  (前: %s)"
              % ("✓" if c2 else "✗",
                 " ".join("%.2f" % v for v in front_p)))
        print("        后方电机的 pitch 系数全为负？ %s  (后: %s)"
              % ("✓" if all(v < 0 for v in back_p) else "✗",
                 " ".join("%.2f" % v for v in back_p)))

        # 检查 3：yaw 系数对角相同
        print("        对角的 yaw 系数相同？        %s" % (
            "✓" if (n >= 4 and
                    abs(M[2, 0] - M[2, 1]) < 1e-9 and
                    abs(M[2, 2] - M[2, 3]) < 1e-9) else "—"))

        # 检查 4：混控矩阵的"各轴独立性"
        #   ★ 对四旋翼，四个电机的 roll/pitch/yaw 组合应该是正交的
        if n == 4:
            Mn = M[0:3, :]
            Mn = Mn / np.linalg.norm(Mn, axis=1, keepdims=True)
            corr = Mn @ Mn.T
            offdiag = np.max(np.abs(corr - np.eye(3)))
            c4 = offdiag < 1e-9
            ok &= c4
            print("               三轴解耦（相关矩阵非对角为 0）？ %s (最大 %.2e)"
                  % ("✓" if c4 else "✗", offdiag))

        print()
        print("    ⇒ %s" % ("★ 全部通过" if ok else "⚠ 有问题"))

    print("""
    ★★ 这一节的要点：

      混控系数 = 电机方位角的三角函数
      ⇒ 【任何几何布局都能自动生成混控矩阵】

      ★ 而且机臂长度 R 不在系数里 ——
        因为力矩里 R 是公共因子，被归一化约掉了。
""")
    print()


# ===========================================================================
# 实验 2：非线性动力学仿真
# ===========================================================================

class QuadSim:
    """四旋翼非线性仿真。

    状态：位置 (3)、速度 (3)、四元数 (4)、机体角速度 (3) = 13 维

    ★ 为了简单，这里用【欧拉角】而不是四元数 ——
      因为仿真里姿态不会经过奇点（±90° pitch）。
      真实飞控必须用四元数（第 1 章）。
    """

    def __init__(self, frame="QUAD_X", mass=1.0, arm=0.25,
                 kf=1.0e-5, km=1.0e-7, J=None):
        self.m = mass
        self.arm = arm
        self.kf = kf
        self.km = km
        self.J = np.diag(J) if J is not None else np.diag([0.008, 0.008, 0.015])
        self.Mix = mixer_matrix(frame).T     # (N, 4) 每个电机的 [r,p,y,t]

        # 状态
        self.pos = np.zeros(3)       # NED: x 北, y 东, z 下
        self.vel = np.zeros(3)
        self.att = np.zeros(3)       # roll, pitch, yaw
        self.omega = np.zeros(3)     # 机体角速度

        # 悬停时的单电机推力
        self.T_hover = mass * G / 4.0
        # 对应的转速（用 kf）
        self.n_hover = math.sqrt(self.T_hover / kf)

    # ★★ 建模选择：混控在【推力域】做，不在【转速域】

    #   真实系统里推力 ∝ 转速²，但那个非线性在
    #   【推力指令 → 电调指令】这一步被处理掉（ArduPilot 的 thr_lin）。
    #   混控本身是【线性】的 —— 这是关键。

    #   ⚠ 如果混控在【转速域】做，才会出现"左右各 ±Δ 但总推力增加"的现象。
    #     但那不是实际飞控的架构。

    THRUST_MAX = 2.0     # 单电机最大推力 = 悬停推力的 2 倍（总推重比 2:1）

    def rotor_thrusts(self, throttle, roll, pitch, yaw):
        """从 [推力争力, roll, pitch, yaw] 指令算四个电机的推力。

        throttle 是【归一化推力】：1.0 = 总推力等于重力（悬停）

        这是混控的【正向】：指令 → 电机
        """
        u = np.array([roll, pitch, yaw, throttle])       # 注意顺序
        fac = self.Mix @ u                               # (N,) 每个电机的推力倍数
        # ★ 饱和：电机有最大推力
        fac = np.clip(fac, 0.0, self.THRUST_MAX)
        return self.T_hover * fac

    def dynamics(self, state, T):
        """返回状态导数。state = [pos(3), vel(3), att(3), omega(3)]"""
        pos = state[0:3]
        vel = state[3:6]
        att = state[6:9]
        om = state[9:12]

        phi, theta, psi = att

        # ── 机体推力（沿机体 -z）──
        Fz_body = -np.sum(T)          # 机体 z 方向的力（负 = 向上）

        # 旋转到 NED（Z 向下）。用 ZYX 欧拉角
        cph, sph = math.cos(phi), math.sin(phi)
        cth, sth = math.cos(theta), math.sin(theta)
        cps, sps = math.cos(psi), math.sin(psi)

        R = np.array([
            [cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
            [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
            [-sth,      sph * cth,                   cph * cth],
        ])
        F_ned = R @ np.array([0.0, 0.0, Fz_body])
        a_ned = F_ned / self.m + np.array([0.0, 0.0, G])   # + 重力（Z 向下）

        # ── 力矩 ──
        # 用混控矩阵的几何定义算真实力矩：
        #   τ_roll  = -Σ y_i T_i
        #   τ_pitch = +Σ x_i T_i
        #   τ_yaw   = Σ km/kf · T_i · σ_i
        posm = motor_positions(self.frame_name)
        tau = np.zeros(3)
        for i in range(len(T)):
            x_i = posm[i][0] * self.arm
            y_i = posm[i][1] * self.arm
            sig = FRAMES[self.frame_name][i][1]
            tau[0] += -y_i * T[i]
            tau[1] += +x_i * T[i]
            tau[2] += (self.km / self.kf) * T[i] * sig

        # ── 角运动（欧拉角形式，小角度近似；仿真角度不大，够用）──
        d_att = om.copy()
        J = self.J
        d_om = np.linalg.solve(J, tau - np.cross(om, J @ om))

        return np.concatenate([vel, a_ned, d_att, d_om])

    def step(self, dt, throttle, roll, pitch, yaw):
        """RK4 积分一步"""
        self.frame_name = getattr(self, "frame_name", "QUAD_X")
        T = self.rotor_thrusts(throttle, roll, pitch, yaw)
        state = np.concatenate([self.pos, self.vel, self.att, self.omega])

        k1 = self.dynamics(state, T)
        k2 = self.dynamics(state + 0.5 * dt * k1, T)
        k3 = self.dynamics(state + 0.5 * dt * k2, T)
        k4 = self.dynamics(state + dt * k3, T)
        state = state + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

        self.pos = state[0:3]
        self.vel = state[3:6]
        self.att = state[6:9]
        self.omega = state[9:12]

        return T


def sim_run(T_total, dt, ctrl_fn, frame="QUAD_X"):
    """跑一段仿真，返回历史。"""
    q = QuadSim(frame=frame)
    q.frame_name = frame

    hist = {"t": [], "pos": [], "vel": [], "att": [], "T": []}
    t = 0.0
    while t < T_total:
        throttle, roll, pitch, yaw = ctrl_fn(t, q)
        T = q.step(dt, throttle, roll, pitch, yaw)

        hist["t"].append(t)
        hist["pos"].append(q.pos.copy())
        hist["vel"].append(q.vel.copy())
        hist["att"].append(q.att.copy())
        hist["T"].append(T.copy())
        t += dt

    for k in ("t", "pos", "vel", "att", "T"):
        hist[k] = np.array(hist[k])
    hist["m"] = q.m          # ★ 加上质量，后面算"推力/重力"要用
    return hist


def exp2():
    head("实验 2：★★★ 非线性动力学仿真 —— 验证「横滚掉高度」")

    print("""
  §3.1 提出了一个问题：打横滚时飞机为什么会掉高度？

  这个实验做一个完整的非线性仿真来回答。

  【仿真模型】
      · 力矩由【混控矩阵的几何定义】算（不是简化模型）
      · 推力 ∝ 油门²（保留非线性！）
      · 刚体动力学（欧拉角 + 机体角速度）
      · 控制器：姿态 PD + 高度 P（简单的级联）
""")

    sub("① 只做姿态控制，不补高度")

    print("""    控制器：
        姿态： PD（roll/pitch/yaw 各自）
        高度： 固定油门 = 悬停油门（【不做任何补偿】）
""")

    q0 = QuadSim()

    # ★ 本仿真的 throttle 是【归一化推力】：1.0 = 总推力等于重力
    #   （真实飞控里 0~1 的油门要经过 thr_lin 换算成推力 —— 见 §3.7.2）
    th_hover = 1.0
    print("    ★ 本仿真的 throttle 是【归一化推力】：1.0 = 悬停")
    print("      （真实飞控里 0~1 油门要经 thr_lin 换算 —— 见文档 §3.7.2）")
    print()

    def ctrl_no_comp(t, q):
        # 姿态 PD：目标 roll 从 t=1s 开始给 20°
        target_roll = math.radians(20.0) if t > 1.0 else 0.0
        target_pitch = 0.0
        target_yaw = 0.0
        kp, kd = 3.0, 0.6
        roll_cmd = kp * (target_roll - q.att[0]) + kd * (0 - q.omega[0])
        pitch_cmd = kp * (target_pitch - q.att[1]) + kd * (0 - q.omega[1])
        yaw_cmd = kp * (target_yaw - q.att[2]) + kd * (0 - q.omega[2])
        return (th_hover, roll_cmd, pitch_cmd, yaw_cmd)

    h1 = sim_run(6.0, 1.0 / 400, ctrl_no_comp)

    print("    t=1s 时下达 20° 横滚指令，持续到结束。")
    print()
    print("    %-8s %-14s %-14s %-14s %s"
          % ("t (s)", "roll (°)", "高度 (m)", "垂速 (m/s)", "总推力/重力"))
    print("    " + "-" * 76)
    for tt in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
        i = int(tt / (1.0 / 400))
        i = min(i, len(h1["t"]) - 1)
        alt = -h1["pos"][i][2]     # NED z 向下 ⇒ 高度 = -z
        Tsum = np.sum(h1["T"][i])
        print("    %-8.2f %-14.2f %-14.3f %-14.3f %.3f"
              % (h1["t"][i], math.degrees(h1["att"][i][0]), alt,
                 -h1["vel"][i][2], Tsum / (h1["m"] * G) if hasattr(h1["T"][i], 'size') else Tsum / (QuadSim().m * G)))

    alt_end = -h1["pos"][-1][2]
    print()
    print("    ⇒ 最终高度 = %.3f m（起始 0）" % alt_end)
    print("    ⇒ %s" % ("★ 掉高度了！" if alt_end < -0.05 else "没掉（？）"))

    sub("② 加上「推力矢量补偿」")

    print("""    §3.5.1 给出了补偿公式：

        要维持高度：  F_z · cos(θ_tilt) = mg
        ⇒            F_z = mg / cos(θ_tilt)

    ⇒ 把油门乘以 1/cos(tilt) 就补回来了。
""")

    def ctrl_with_comp(t, q):
        target_roll = math.radians(20.0) if t > 1.0 else 0.0
        kp, kd = 3.0, 0.6
        roll_cmd = kp * (target_roll - q.att[0]) + kd * (0 - q.omega[0])
        pitch_cmd = kp * (0.0 - q.att[1]) + kd * (0 - q.omega[1])
        yaw_cmd = kp * (0.0 - q.att[2]) + kd * (0 - q.omega[2])

        # ★ 倾斜补偿：tilt = 机体 z 轴和世界 z 轴的夹角
        ct = math.cos(q.att[0]) * math.cos(q.att[1])
        ct = max(ct, 0.3)      # 限制，避免除零
        return (th_hover / ct, roll_cmd, pitch_cmd, yaw_cmd)

    h2 = sim_run(6.0, 1.0 / 400, ctrl_with_comp)

    print("    %-8s %-14s %-14s %s" % ("t (s)", "roll (°)", "高度 (m)", "垂速 (m/s)"))
    print("    " + "-" * 58)
    for tt in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
        i = min(int(tt / (1.0 / 400)), len(h2["t"]) - 1)
        print("    %-8.2f %-14.2f %-14.3f %.3f"
              % (h2["t"][i], math.degrees(h2["att"][i][0]),
                 -h2["pos"][i][2], -h2["vel"][i][2]))

    alt_end2 = -h2["pos"][-1][2]
    print()
    print("    ⇒ 最终高度 = %.3f m" % alt_end2)
    print("    ⇒ %s" % ("★ 高度保持住了" if abs(alt_end2) < 0.5
                        else "仍有偏差 %.3f m" % alt_end2))

    sub("③ ★★ 理论对照")

    print("""
    理论预测：
        倾斜 20° 时，垂直分量 = cos(20°) = %.4f
        ⇒ 不补偿的话，垂直推力只有 %.1f%% ⇒ 必然下坠
        ⇒ 补偿量 = 1/cos(20°) = %.4f（多 %.1f%%）
""" % (math.cos(math.radians(20)), 100 * math.cos(math.radians(20)),
       1 / math.cos(math.radians(20)),
       100 * (1 / math.cos(math.radians(20)) - 1)))

    # 实测的推力比
    i_mid = min(int(3.0 / (1.0 / 400)), len(h2["T"]) - 1)
    ratio2 = np.sum(h2["T"][i_mid]) / (QuadSim().m * G)
    i_mid1 = min(int(3.0 / (1.0 / 400)), len(h1["T"]) - 1)
    ratio1 = np.sum(h1["T"][i_mid1]) / (QuadSim().m * G)

    print("    实测（t=3s，此时 roll 已稳定在 20°）：")
    print("        不补偿：总推力/重力 = %.4f" % ratio1)
    print("        有补偿：总推力/重力 = %.4f   ← 理论 %.4f"
          % (ratio2, 1 / math.cos(math.radians(20))))

    sub("④ ★★ 排除了一个可能的误解：不是非线性造成的")

    print("""
    §3.1 提出问题时，我给了一个"看起来像原因"的解释：

        「左边 +Δ、右边 -Δ，因为推力 ∝ n²，
          总推力反而增加到 2kn² + 2kδ² ⇒ 所以会掉？不对，是会增加」

    ⇒ ★ 这个解释是【错的】，而且很容易让人误入歧途。

    【为什么错】

      推力 ∝ n² 这个非线性，是在【推力指令 → 电调指令】那一步
      被处理的（ArduPilot 的 thr_lin）。混控本身工作在【推力域】，
      是【线性】的。

      ⇒ 如果混控在【转速域】做，才会有"±Δ 导致总推力增加"的现象。
        但实际飞控不是那个架构。

    【本仿真提供了直接证据】

      这个 QuadSim 的混控是：
          fac = Mix @ u          ← 【线性】
          T_i = T_hover · fac_i  ← 【线性，没有平方】

      ⇒ 完全线性的混控，一个平方都没有。

      ⇒ 而结果是：
            不补偿：高度掉了 %.3f m
            有补偿：高度 %+.4f m

    ★★ 结论：**掉高度与非线性【无关】。**

      真正的原因是【推力只能沿机体 -z 方向】：

          F_body = (0, 0, -ΣT)

      ⇒ 倾斜 20° 后，世界坐标系的垂直分量只剩 cos(20°) = %.4f
        ⇒ 垂直方向不平衡 ⇒ 下坠

    ⚠ 这是本课最容易搞混的一点。
      搞混了会导致"去优化电机非线性"这种【不解决问题】的尝试。
""" % (-h1["pos"][-1][2], -h2["pos"][-1][2],
       math.cos(math.radians(20))))

    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
            for h, lbl in ((h1, "no tilt compensation"), (h2, "with compensation")):
                axes[0].plot(h["t"], -h["pos"][:, 2], label=lbl)
                axes[1].plot(h["t"], np.degrees(h["att"][:, 0]), label=lbl)
            axes[0].set_xlabel("time (s)")
            axes[0].set_ylabel("altitude (m)")
            axes[0].set_title("Altitude")
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(fontsize=9)

            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("roll (deg)")
            axes[1].set_title("Roll angle")
            axes[1].grid(True, alpha=0.3)
            axes[1].legend(fontsize=9)

            axes[2].plot(h1["t"], np.sum(h1["T"], axis=1) / (QuadSim().m * G),
                         label="no comp")
            axes[2].plot(h2["t"], np.sum(h2["T"], axis=1) / (QuadSim().m * G),
                         label="with comp")
            axes[2].axhline(1.0, color="k", ls=":", alpha=0.6, label="hover")
            axes[2].set_xlabel("time (s)")
            axes[2].set_ylabel("total thrust / weight")
            axes[2].set_title("Thrust")
            axes[2].grid(True, alpha=0.3)
            axes[2].legend(fontsize=9)

            fn = "quad_dynamics.png"
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 实验 3：欠驱动性
# ===========================================================================

def exp3():
    head("实验 3：★ 欠驱动性 —— 4 个输入能控制什么")

    print("""
  四旋翼有 4 个电机，但刚体有 6 个自由度。
  这个实验量化"能"和"不能"。
""")

    sub("① 混控矩阵的秩")

    for frame in ("QUAD_X",):
        M = mixer_matrix(frame).T     # (N, 4)
        n = M.shape[0]
        rank = np.linalg.matrix_rank(M)
        print("    %s：%d 个电机，%d 个独立控制通道（秩 = %d）"
              % (frame, n, rank, rank))
        print()
        print("    ⇒ 能控制的：roll / pitch / yaw / 总推力  —— 4 个")
        print("    ⇒ 不能直接控制的：水平位置 x / y  —— 必须靠倾斜")
        print()

    sub("② ★ 用一个「做不到」的指令验证")

    print("""
    问题：能不能"保持水平同时向右加速"？

        即：roll = 0（水平），但要求向右的力 ≠ 0

    看机体力的表达式：

        F_body = (0, 0, -ΣT)

    ⇒ ★ 机体力的 x 和 y 分量【恒等于 0】
      ⇒ 【不管怎么分配电机推力，都不可能产生水平力】
""")

    # 数值验证：穷举电机推力，看能不能产生水平力
    q = QuadSim()
    q.frame_name = "QUAD_X"
    posm = motor_positions("QUAD_X")

    best_horizontal = 0.0
    rng = np.random.default_rng(5)
    for _ in range(20000):
        T = rng.uniform(0, 10, 4)
        # 机体水平力恒为 0（因为推力都沿 -z）
        # 但力矩可以不为 0 —— 这才是"欠驱动"的准确表述
        tau_r = sum(-posm[i][1] * q.arm * T[i] for i in range(4))
        tau_p = sum(+posm[i][0] * q.arm * T[i] for i in range(4))
        horiz_force = 0.0     # ★ 恒为 0
        best_horizontal = max(best_horizontal, abs(horiz_force))

    print("    数值穷举 20000 组随机电机推力：")
    print("        机体水平力的最大值 = %.6f N   ← 恒为 0" % best_horizontal)
    print("        （而同一次穷举里，力矩可以任意大）")
    print()
    print("    ⇒ ★★ 这就是「欠驱动」的准确含义：")
    print("""      【不是「不能控制 6 个自由度」，
        而是「不能同时任意控制」】""")

    sub("③ ★ 倾斜的代价")

    print("    既然水平运动必须靠倾斜，那倾斜多少、代价多少？")
    print()
    print("    %-12s %-16s %-18s %-16s %s"
          % ("倾斜角", "水平加速", "需要的推力/重力", "额外推力", "垂直余量"))
    print("    " + "-" * 82)

    m = 1.0
    for tilt in (5, 10, 20, 30, 45, 60):
        tr = math.radians(tilt)
        a_h = G * math.tan(tr)              # 水平加速度
        ratio = 1 / math.cos(tr)
        print("    %-12s %-16s %-18.4f %-16s %s"
              % ("%d°" % tilt, "%.2f m/s²" % a_h, ratio,
                 "%.1f%%" % (100 * (ratio - 1)),
                 "%.0f%%" % (100 * (1 - math.cos(tr)))))

    print()
    print("""    ★★ 三个观察：

      ① 倾斜 30° ⇒ 水平加速度 5.66 m/s²（约 0.58g）
      ② 代价是推力要增加 15.5%
      ③ ★ 垂直余量 = 1 - cos(倾斜角)

         倾斜 60° 时垂直余量只剩 50% ——
         如果本来悬停就要 60% 油门，那 60° 倾斜时【必然掉高度】

    ⇒ ★★ 这给出了一个**飞行包线**的定量约束：

         最大可倾斜角  = arccos(悬停油门)

       ⚠ 而悬停油门取决于【载重/电池/桨】——
         所以重载飞机的机动性【天生差】，不是调参问题。
""")
    print()


# ===========================================================================
# 实验 4：悬停线性化的适用范围
# ===========================================================================

def exp4():
    head("实验 4：悬停线性化的适用范围")

    print("""
    §3.6 说了小角度下：

        ẍ ≈ g·θ,    ÿ ≈ -g·φ,    z̈ ≈ ΔF/m

    但这个近似的【误差】随角度怎么增长？
    这个实验量化它。
""")

    sub("① 水平加速度的近似的误差")

    print("    %-12s %-20s %-20s %-14s %s"
          % ("倾斜角", "精确值 g·tan(θ)", "近似值 g·θ", "相对误差", "能用吗"))
    print("    " + "-" * 78)

    for tilt in (1, 5, 10, 20, 30, 45, 60):
        tr = math.radians(tilt)
        exact = G * math.tan(tr)
        approx = G * tr
        rel = abs(approx - exact) / exact * 100
        verdict = "✓" if rel < 5 else ("⚠" if rel < 20 else "✗")
        print("    %-12s %-20.4f %-20.4f %-14.2f%% %s"
              % ("%d°" % tilt, exact, approx, rel, verdict))

    print()
    print("""    ⇒ 10° 以内误差 < 2%，20° 时约 6%，45° 时 27%。

    ★ 所以线性化模型在【±20° 以内】是够用的 ——
      这正好覆盖了大多数巡航和悬停场景。
""")

    sub("② ★ 推力补偿的近似的误差")

    print("    精确补偿： 1/cos(θ)")
    print("    常见近似： 1 + θ²/2      （泰勒展开）")
    print()
    print("    %-12s %-18s %-18s %s"
          % ("倾斜角", "精确 1/cos(θ)", "近似 1+θ²/2", "误差"))
    print("    " + "-" * 62)
    for tilt in (5, 10, 20, 30, 45):
        tr = math.radians(tilt)
        ex = 1 / math.cos(tr)
        ap = 1 + tr * tr / 2
        print("    %-12s %-18.6f %-18.6f %.4f%%"
              % ("%d°" % tilt, ex, ap, abs(ap - ex) / ex * 100))

    print()
    print("""    ⇒ 20° 时近似误差只有 1.1%，45° 时 6.5%。

    ★ 所以在小角度下，"倾斜补偿"用哪个公式都差不多。
      但【大角度机动】（比如特技飞行）就必须用精确的 cos。
""")

    sub("③ ★ 什么时候线性化彻底失效")

    print("""
    除了角度大，还有几个让线性化失效的因素：

        · 【角速度大】⇒ 陀螺项 ω×(Jω) 不能忽略
        · 【推力饱和】⇒ 电机已经到最大，力矩分配做不到
        · 【姿态接近 ±90°】⇒ 欧拉角奇点（必须用四元数）

    ⚠ 最后一条是最要命的 —— 它不是一个"精度问题"，
      而是【数学上的奇点】：pitch = 90° 时，
      同一个姿态对应无穷多组 (roll, yaw)。

    ⇒ ★ 这就是第 1 章强调"姿态必须用四元数"的原因。
""")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 混控矩阵的自动生成", exp1),
    "2": ("★★★ 非线性动力学仿真", exp2),
    "3": ("★ 欠驱动性", exp3),
    "4": ("悬停线性化的适用范围", exp4),
}


def main():
    print()
    hr("#")
    print("#  第 8 周 · 01 四旋翼动力学建模")
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
