#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 18 周 · 03 位置与速度控制 —— 配套实验

    实验 1  欠驱动：4 个输入 6 个自由度 —— 水平力只能来自倾斜
    实验 2  ★★ 位置 → 姿态 反解（复现 PX4 的 thrustToAttitude）
    实验 3  ★★ 三层级联闭环：位置环 → 速度环 → 姿态环
    实验 4  ★ 前馈 vs 反馈：为什么悬停油门必须前馈

用法：
    python lesson18_position_control.py 1
    python lesson18_position_control.py 2
    python lesson18_position_control.py 3
    python lesson18_position_control.py 4
    python lesson18_position_control.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  和《NuttX/FreeRTOS 教程》一样：**能算的就不猜。**

  实验 2 复现的是 PX4 的这四行（行号已核对）：

      src/modules/mc_pos_control/PositionControl/PositionControl.cpp:204  _accelerationControl()
      src/modules/mc_pos_control/PositionControl/PositionControl.cpp:214  body_z = ...
      src/modules/mc_pos_control/PositionControl/PositionControl.cpp:215  ControlMath::limitTilt(...)
      src/modules/mc_pos_control/PositionControl/PositionControl.cpp:268  ControlMath::thrustToAttitude(...)

  实验 3 用的是 PX4 的**出厂默认增益**（行号已核对）：

      multicopter_position_control_gain_params.c:45   MPC_Z_P          = 1.0
      multicopter_position_control_gain_params.c:58   MPC_XY_P         = 0.95
      multicopter_position_control_gain_params.c:71   MPC_Z_VEL_P_ACC  = 4.0
      multicopter_position_control_gain_params.c:84   MPC_XY_VEL_P_ACC = 1.8
      multicopter_position_control_gain_params.c:97   MPC_Z_VEL_I_ACC  = 2.0
      multicopter_position_control_gain_params.c:111  MPC_XY_VEL_I_ACC = 0.4
      multicopter_position_control_params.c:49        MPC_THR_HOVER    = 0.5
      multicopter_position_control_limits_params.c:126 MPC_THR_MIN     = 0.12

  参考：
      第 3 章  03-动力学建模/01-四旋翼动力学建模.md  §3.5 欠驱动、§3.6 悬停线性化
      第 4 章  04-状态估计/01-概率论与高斯分布.md
"""

import math
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 输出辅助（和本教程其它脚本一致）
# ---------------------------------------------------------------------------

W = 74

G = 9.80665          # 标准重力加速度 m/s^2


def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)
    print()


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (W - 4))
    print()


def _dlen(s):
    """显示宽度：CJK 字符占 2 列。表格对齐全靠它。"""
    n = 0
    for ch in str(s):
        o = ord(ch)
        n += 2 if (0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF
                   or 0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF
                   or 0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60
                   or 0xFFE0 <= o <= 0xFFE6) else 1
    return n


def tab(rows, headers, width=None):
    """打印表格。rows 是二维列表。（按显示宽度对齐，中文不会歪）"""
    cols = len(headers)
    if width is None:
        width = [max(_dlen(r[i]) for r in [headers] + rows) + 2
                 for i in range(cols)]

    def cell(s, w):
        s = str(s)
        return s + " " * max(1, w - _dlen(s))

    print("  " + "".join(cell(headers[i], width[i]) for i in range(cols)))
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join(cell(r[i], width[i]) for i in range(cols)))
    print()


# ---------------------------------------------------------------------------
# 旋转矩阵的基本工具
# ---------------------------------------------------------------------------
#
# 约定（和 PX4/ArduPilot 一致）：
#   · 导航系用 NED：x = 北，y = 东，z = 地（向下为正）
#   · 机体系：x = 机头，y = 右，z = 机腹（向下为正）
#   · R = R_body->NED = Rz(psi) @ Ry(theta) @ Rx(phi)
#
# ★ 注意 NED 下"机体 z 轴"是【朝下】的 ——
#   所以多旋翼的推力方向是 **-z_body**（朝上）。
#   这就是 PX4 里 att_sp.thrust_body[2] 用【负数】表示"向上飞"的原因。

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_body_to_ned(phi, theta, psi):
    """R = Rz(psi) Ry(theta) Rx(phi)。第三列 = 机体 z 轴在 NED 下的表示。"""
    return rot_z(psi) @ rot_y(theta) @ rot_x(phi)


def body_z_of(phi, theta, psi):
    """机体 z 轴在 NED 下的单位向量（= R 的第三列）。"""
    return rot_body_to_ned(phi, theta, psi)[:, 2]


# ---------------------------------------------------------------------------
# 实验 1：欠驱动 —— 4 个输入 6 个自由度
# ---------------------------------------------------------------------------

def exp1():
    head("实验 1  欠驱动：4 个输入 6 个自由度 —— 水平力只能来自倾斜")

    print("""
  第 3 章 §3.5 说过：四旋翼有 6 个自由度，但只有 4 个独立输入
  （总推力 T + 三轴力矩 Mx, My, Mz）⇒ 欠驱动。

  ★ 这一节要把"欠驱动"从一个口号变成一行公式。

  总推力在机体系里永远只有【一个方向】：+z_body（穿过机身向上）。
  它变成导航系里的什么方向，完全由姿态 R 决定：

        ★★  F_ned = R · [0, 0, T]^T = T · (R 的第三列)

    ⇒ **姿态决定了推力指向。**
    ⇒ **所以位置环的"执行器"只能是姿态环的"设定值"。**
""")

    sub("① 推力的方向 = R 的第三列")

    print("    设 T = %.1f N（大约能托起 %.2f kg），yaw = 0。" % (9.81, 1.0))
    print("    改变 pitch，看推力在 NED 下的方向：")
    print()

    T = 9.81
    rows = []
    for deg in (0, 5, 10, 20, 30):
        th = math.radians(deg)
        # R 的第三列
        z_body = body_z_of(0.0, th, 0.0)
        F = T * z_body          # 推力沿 +z_body（NED 下"向下"）
        # ★ 多旋翼的推力是沿 -z_body 的（朝上），所以物理推力是 -F
        F_up = -F
        rows.append(["%.0f°" % deg,
                     "(%.3f, %.3f, %.3f)" % (z_body[0], z_body[1], z_body[2]),
                     "%+.3f" % F_up[0],
                     "%+.3f" % F_up[2]])
    tab(rows, ["pitch", "R 第三列 z_body(NED)", "北向推力 Fx (N)", "天向推力 Fz (N)"])

    print("""    ★ 三件事一眼可见：

        ① pitch = 0 时 R 第三列 = (0,0,1) ⇒ 推力全在"天向"（-z 是上）
        ② pitch ≠ 0 时，**水平分量出现了** —— 这就是"倾斜产生水平力"
        ③ **水平力是倾斜的【副产品】，不是独立的输入** ⇒ 这就是欠驱动
""")

    sub("② 想要多快，就得倾斜多少：a_xy = g·tanθ")

    print("    平飞时（高度不变，垂直合力 = 0）：")
    print()
    print("        T·cosθ = m·g          垂直平衡")
    print("        T·sinθ = m·a_xy       水平方向（牛顿第二定律）")
    print("        ─────────────────────────────")
    print("        两式相除       ⇒  ★  a_xy = g · tanθ")
    print()

    rows = []
    for a in (0.5, 1.0, 2.0, 3.0, 5.0, 8.0):
        th = math.atan(a / G)
        rows.append(["%.1f" % a, "%.4f" % th, "%.2f" % math.degrees(th),
                     "%.4f" % (a / G), "%.4f" % (G * math.tan(th))])
    tab(rows, ["想要的 a_xy (m/s²)", "θ (rad)", "θ (°)", "小角度 a/g", "精确 g·tanθ"])

    print("""    ★ 最后两列对比：小角度下 tanθ ≈ θ，两者几乎一样。

      **但这不是"可以随便近似"的理由 —— 大角度下差得很明显：**
      想要 8 m/s²（≈0.8 g 的水平加速度）需要倾斜 39.2°，
      此时 tanθ = 0.816 而 θ = 0.685 —— 差 19%。

      ★ 更重要的是：tanθ 在 θ→90° 时【爆炸】。
        这解释了为什么多旋翼的水平加速度有物理上限 ——
        倾角被限幅（PX4 的 MPC_TILTMAX_AIR，ArduPilot 的 ANGLE_MAX），
        本质上是在限制"推力里能分多少给水平方向"。
""")

    sub("③ 从加速度反解倾角（PX4 和 AP 都这么算）")

    print("""    既然姿态是唯一的手段，那控制器要做的第一件事就是
    "**给我想要的加速度，反解出该倾斜多少**"。

    代码里都不做 tan，而是直接做 atan（因为 atan 全域安全）：

        ArduPilot  libraries/AC_AttitudeControl/AC_PosControl.cpp:1215
            pitch_target = accel_to_angle(-accel_forward * 0.01) * 100;

        PX4        src/modules/mc_pos_control/PositionControl/PositionControl.cpp:214
            Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
""")

    rows = []
    for a in (1.0, 2.0, 4.0, 8.0):
        th_exact = math.atan(a / G)
        th_small = a / G
        rows.append(["%.1f" % a, "%.6f" % th_exact, "%.6f" % th_small,
                     "%.2f" % math.degrees(th_exact),
                     "%.4f%%" % (100 * (th_small - th_exact) / th_exact)])
    tab(rows, ["a_xy (m/s²)", "atan(a/g)", "a/g", "atan 值 (°)", "小角度误差"])

    print("""    ★ 结论：**倾斜和水平加速度是一一对应的（单调、可逆）。**

        想要 a → 必须倾斜 θ = atan(a/g)
        倾斜了 θ → 必然产生 a = g·tanθ

      **中间没有任何"第三条路"** —— 这就是欠驱动的全部含义。

      ⇒ 位置控制器的输出**不可能是"推力大小"**，
        它只能是一个**方向**（外加一个总推力标量给垂直通道用）。
        方向 → 姿态设定值 → 交给姿态环。
""")


# ---------------------------------------------------------------------------
# 实验 2：位置 → 姿态反解（复现 PX4 的 thrustToAttitude）
# ---------------------------------------------------------------------------

def accel_to_attitude(acc_sp, yaw):
    """复现 PX4 的 PositionControl::_accelerationControl() + thrustToAttitude()。

    返回 (roll, pitch, body_z, R)。acc_sp 是 NED 下的期望加速度。

    对应源码：
      PositionControl.cpp:207  float z_specific_force = -CONSTANTS_ONE_G;
      PositionControl.cpp:211  z_specific_force += _acc_sp(2);
      PositionControl.cpp:214  body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
      ControlMath.cpp:47       thrustToAttitude()
      ControlMath.cpp:69       bodyzToAttitude()
    """
    ax, ay, az = float(acc_sp[0]), float(acc_sp[1]), float(acc_sp[2])

    # ★ PX4 的三行：比力（specific force）在 NED 下 = a - g_vec，g_vec = (0,0,+g)
    z_specific_force = -G + az
    body_z = np.array([-ax, -ay, -z_specific_force])
    body_z = body_z / np.linalg.norm(body_z)

    # --- 复现 ControlMath::bodyzToAttitude() 的姿态构造 ---
    # 期望 yaw 方向在 XY 平面内转 90°：y_C = (-sin(yaw), cos(yaw), 0)
    y_c = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    body_x = np.cross(y_c, body_z)
    if body_z[2] < 0.0:                 # 倒飞时保持机头朝前
        body_x = -body_x
    if abs(body_z[2]) < 1e-6:
        body_x = np.array([0.0, 0.0, 1.0])
    body_x = body_x / np.linalg.norm(body_x)
    body_y = np.cross(body_z, body_x)

    R = np.column_stack([body_x, body_y, body_z])

    # --- 从 R 反解 roll / pitch（用于对照） ---
    # 对 R = Rz(psi) Ry(theta) Rx(phi)，第三列 = [sinθ, -cosθ sinφ, cosθ cosφ]
    theta = math.asin(max(-1.0, min(1.0, float(R[2, 0] * 0 + R[0, 2]))))
    # R[0,2] = sinθ  ⇒  θ = asin(R[0,2])
    theta = math.asin(max(-1.0, min(1.0, float(R[0, 2]))))
    cos_t = math.cos(theta)
    if abs(cos_t) < 1e-9:
        phi = 0.0
    else:
        phi = math.atan2(-float(R[1, 2]), float(R[2, 2]))
    return phi, theta, body_z, R


def exp2():
    head("实验 2  ★★ 位置 → 姿态 反解 —— 复现 PX4 的 thrustToAttitude")

    print("""
  这是本课的核心一步：

      **位置控制器怎么产生姿态设定值？**

  链条是：

      期望加速度 a_des (NED)
          ↓  ① 比力 = a_des - g_vec
      期望的"机体 z 轴方向" body_z
          ↓  ② 加上一个独立的 yaw
      期望姿态 R（= 四元数设定值）

  ① 是物理，② 里的 yaw 是**唯一一个不参与位置控制的自由度**（欠驱动的产物）。
""")

    sub("① 静止悬停：a_des = 0 ⇒ body_z = 竖直向下")

    phi, theta, bz, R = accel_to_attitude([0.0, 0.0, 0.0], 0.0)
    print("    a_des = (0, 0, 0)")
    print("    z_specific_force = -g + 0 = %.4f" % (-G))
    print("    body_z = normalize([0, 0, %.4f]) = (%.4f, %.4f, %.4f)"
          % (G, bz[0], bz[1], bz[2]))
    print("    roll = %+.4f rad (%.3f°), pitch = %+.4f rad (%.3f°)"
          % (phi, math.degrees(phi), theta, math.degrees(theta)))
    print()
    print("""    ★ 注意 body_z = (0, 0, +1)：在 NED 里 +z 是【向下】。

      **机体 z 轴朝下**，而多旋翼的推力是沿 **-z_body**（朝上）的。
      这就是 PX4 里 `att_sp.thrust_body[2] = -thr_sp.length()`
      （ControlMath.cpp:49）为什么带负号。
""")

    sub("② 想往北加速 2 m/s²")

    phi, theta, bz, R = accel_to_attitude([2.0, 0.0, 0.0], 0.0)
    print("    a_des = (+2, 0, 0) m/s²   （NED 里 +x 是北）")
    print("    body_z = normalize([-2, 0, %.4f]) = (%.4f, %.4f, %.4f)"
          % (G, bz[0], bz[1], bz[2]))
    print("    pitch = %+.4f rad = %+.3f°" % (theta, math.degrees(theta)))
    print("    小角度对照：-a_x/g = %+.4f rad = %+.3f°"
          % (-2.0 / G, math.degrees(-2.0 / G)))
    print()
    print("""    ★ pitch = %.3f° < 0 ⇒ **低头**。

      NED 下 pitch 抬头为正，所以"低头"是负的。
      低头 ⇒ 推力往北偏 ⇒ 向北加速 ✓
      （第 3 章 §3.6 写的 $\\ddot x \\approx g\\theta$ 是把 θ 定义成低头为正，
        本课统一用航空标准"抬头为正"，所以这里是 $\\ddot x \\approx -g\\theta$，
        **量级完全一致，只是符号约定不同**。）
""" % math.degrees(theta))

    sub("③ 想往东加速 2 m/s²")

    phi, theta, bz, R = accel_to_attitude([0.0, 2.0, 0.0], 0.0)
    print("    a_des = (0, +2, 0) m/s²   （NED 里 +y 是东）")
    print("    body_z = normalize([0, -2, %.4f]) = (%.4f, %.4f, %.4f)"
          % (G, bz[0], bz[1], bz[2]))
    print("    roll = %+.4f rad = %+.3f°" % (phi, math.degrees(phi)))
    print()
    print("    ★ roll > 0 ⇒ **右滚**（右翼下压）⇒ 推力往东偏 ⇒ 向东加速 ✓")

    sub("④ ★ 一般情形：同时要水平加速度和垂直加速度")

    print("""    如果飞行器正在垂直加速（比如起飞），az ≠ 0。
    这时 body_z 不再是竖直的，**水平倾角和垂直加速度耦合**。

    PX4 的做法（PositionControl.cpp:207-214）是把"重力"和"期望垂直加速度"
    一起塞进同一个向量：

        z_specific_force = -g + a_z          ← 注意是【加】上去的
        body_z = normalize([-a_x, -a_y, -z_specific_force])
               = normalize([-a_x, -a_y,  g - a_z])
""")

    rows = []
    for az in (0.0, 2.0, G * 0.5, 3.0, -3.0):
        phi, theta, bz, R = accel_to_attitude([2.0, 0.0, az], 0.0)
        name = "平飞"
        if az < -1e-9:
            name = "向上加速"
        elif az > 1e-9:
            name = "向下加速"
        rows.append(["%+.1f" % az, name, "%+.4f" % theta,
                     "%+.2f" % math.degrees(theta), "%.4f" % bz[2]])
    tab(rows, ["a_z (NED)", "在做什么", "pitch (rad)", "pitch (°)", "body_z 的 z 分量"])

    print("""    ⚠️ 先看清 NED 的符号：**z 向下为正**。
       所以 a_z = +2 是"向下加速"，a_z = -3 才是"向上加速"。

    ★ 表里看得很清楚（水平都要求 2 m/s²）：

        向上加速 (a_z = -3)  ⇒  倾角只要 8.88°   ← 最小
        平飞     (a_z =  0)  ⇒  倾角要 11.53°
        向下加速 (a_z = +4.9) ⇒  倾角要 22.19°   ← 最大

      **为什么？** 把比力方向写出来就懂了：

          比力 = a - g_vec = (a_x, a_y, a_z - g)

        向上加速时 a_z - g = -12.81，合力向量更长、更竖直
        ⇒ 同样的水平分量对应的倾角【更小】。

        向下加速时 a_z - g = -4.91，合力向量更短
        ⇒ 同样的水平分量对应的倾角【更大】。

      ★★ **这就是"俯冲时转弯更凶"的数学来源** ——
        同样的倾角，掉高度时能拉出更大的水平加速度。

      ★ 这也解释了 PX4 里那个开关（PositionControl.cpp:209）：

          if (!_decouple_horizontal_and_vertical_acceleration) {
              z_specific_force += _acc_sp(2);     // 把 az 算进 body_z
          }

        **把它算进去 ⇒ 水平跟踪更准（考虑了耦合）；
          不算进去 ⇒ 水平和垂直解耦，更好调。**
        是一对工程取舍，不是对错问题。
""")

    sub("⑤ 验证：反解出的 R 的第三列确实是 body_z，且 -z_body 指向期望合力方向")

    a_des = np.array([2.0, 1.5, -1.0])
    phi, theta, bz, R = accel_to_attitude(a_des, math.radians(30))
    print("    a_des = (%.1f, %.1f, %.1f),  yaw = 30°" % tuple(a_des))
    print("    R 第三列        = (%.6f, %.6f, %.6f)" % tuple(R[:, 2]))
    print("    body_z（直接算） = (%.6f, %.6f, %.6f)" % tuple(bz))
    print("    最大偏差         = %.2e" % float(np.max(np.abs(R[:, 2] - bz))))
    print()
    print("    R 正交性检查 |RᵀR - I| = %.2e" % float(np.max(np.abs(R.T @ R - np.eye(3)))))
    print("    det(R) = %.6f   （必须是 +1）" % float(np.linalg.det(R)))
    print()

    # 期望合力方向（比力方向）应该与 -body_z 一致
    f_dir = a_des - np.array([0.0, 0.0, G])
    f_dir = f_dir / np.linalg.norm(f_dir)
    print("    期望比力方向 (a-g)/|a-g| = (%.6f, %.6f, %.6f)" % tuple(f_dir))
    print("    -body_z                  = (%.6f, %.6f, %.6f)" % tuple(-bz))
    print("    偏差 = %.2e" % float(np.max(np.abs(f_dir - (-bz)))))
    print()
    print("""    ✅ 完全对上 —— **"姿态反解"就是"把比力方向当作机体的 -z 轴"**，
      仅此而已。yaw 是外面的自由参数，不参与这个反解。
""")

    sub("⑥ 小角度近似 vs 精确解：什么时候可以偷懒")

    rows = []
    for a in (1.0, 2.0, 4.0, 6.0, 8.0):
        phi_e, th_e, _, _ = accel_to_attitude([a, 0.0, 0.0], 0.0)
        approx = -a / G
        rows.append(["%.1f" % a, "%+.5f" % th_e, "%+.5f" % approx,
                     "%.3f" % math.degrees(th_e),
                     "%+.3f%%" % (100 * (approx - th_e) / abs(th_e))])
    tab(rows, ["a_x (m/s²)", "精确 pitch", "近似 -a/g", "精确值 (°)", "近似误差"])

    print("""    ★ 2 m/s² 以内误差 < 1.4%，4 m/s² 时 5.3%，8 m/s² 时已经 19%。
      所以：
        · **设计增益时**用线性模型（第 3 章 §3.6）完全够
        · **真正算设定值时**必须用 atan / 归一化（PX4 和 AP 都这么做）

      这正是第 3 章 §3.6 那条分界线：
      **"线性化是用来设计的，不是用来算的。"**
""")


# ---------------------------------------------------------------------------
# 实验 3：三层级联闭环仿真
# ---------------------------------------------------------------------------

def exp3():
    head("实验 3  ★★ 三层级联：位置环 → 速度环 → 姿态环")

    print("""
  课文说位置控制是三层级联。这个实验把三层都真的跑起来，
  看"从 A 点飞到 B 点"的全过程。

  用的增益是 **PX4 的出厂默认值**（不是编的）：

      位置环 P        MPC_XY_P         = 0.95    (1/s)
                      MPC_Z_P          = 1.0     (1/s)
      速度环 P        MPC_XY_VEL_P_ACC = 1.8     (1/s)
                      MPC_Z_VEL_P_ACC  = 4.0     (1/s)
      速度环 I        MPC_XY_VEL_I_ACC = 0.4     (m/s³ per m)
                      MPC_Z_VEL_I_ACC  = 2.0
      悬停油门        MPC_THR_HOVER    = 0.5
      最小油门        MPC_THR_MIN      = 0.12

  对应源码（行号已核对）：
      multicopter_position_control_gain_params.c:45,58,71,84,97,111
      multicopter_position_control_params.c:49
      multicopter_position_control_limits_params.c:126

  ---- 仿真里用了哪些"真"的公式 ----

  ① 姿态反解与推力     **完全复现** PX4 PositionControl.cpp:204-221
  ② 姿态环            **简化为**一阶滞后（时间常数 TAU_ATT）
  ③ 机体运动          用 NED 下的精确推力投影：
                          a = -（u/u_hover）·g·z_body + (0, 0, g)

  ★ ③ 那个式子是"推力和油门成正比"的线性化（第 3 章 §3.7），
    不包含桨叶气动、电池掉压、机体阻尼 —— 但**倾角耦合是精确的**，
    这一点对位置控制最重要。
""")

    # ---- PX4 出厂默认增益（数值来自上面的源码行号） ----
    MPC_XY_P = 0.95
    MPC_Z_P = 1.0
    MPC_XY_VEL_P_ACC = 1.8
    MPC_Z_VEL_P_ACC = 4.0
    MPC_XY_VEL_I_ACC = 0.4
    MPC_Z_VEL_I_ACC = 2.0
    MPC_THR_HOVER = 0.5
    MPC_THR_MIN = 0.12
    MPC_TILTMAX_AIR = math.radians(45.0)      # 参数 MPC_TILTMAX_AIR 默认 45°
    MPC_ACC_HOR_MAX = 5.0                     # 水平加速度限幅（参数 MPC_ACC_HOR_MAX）

    TAU_ATT = 0.05        # 姿态内环等效一阶时间常数 [s]（示例值，非实测）

    # ---- 被控对象：从"油门 u + 姿态(φ,θ)"到"NED 加速度" ----
    def plant_acc(u, phi, theta, u_hover):
        """NED 下的加速度。

        ★ 这张式子是位置控制里唯一真正"物理"的一步：

            z_body(NED) = ( sinθ,  -sinφ·cosθ,  cosφ·cosθ )
            a = -(u/u_hover)·g·z_body + (0, 0, g)

          最后那一项 (0,0,g) 是重力（NED 下 +z 向下）。
        """
        zb = body_z_of(phi, theta, 0.0)
        return -(u / u_hover) * G * zb + np.array([0.0, 0.0, G])

    def simulate(dt=0.004, t_end=6.0, pos_sp=(5.0, 3.0, -4.0),
                 use_ff_thrust=True, use_vel_pid=True, kp_direct=0.05,
                 u_hover_true=0.5):
        """跑三层级联仿真，返回 (t, pos, vel, att, u_hist)。

        use_vel_pid=False 时退化成"单环"：位置误差直接给倾角。
        """
        n = int(t_end / dt)

        pos = np.array([0.0, 0.0, -1.0])      # 起点：1 m 高悬停（NED 下 -1）
        vel = np.zeros(3)
        pos_sp = np.array(pos_sp, dtype=float)

        int_xy = np.zeros(2)
        int_z = 0.0

        phi = theta = 0.0                     # 当前姿态（一阶滞后跟踪目标）

        t_hist, pos_hist, vel_hist, att_hist, u_hist = [], [], [], [], []

        for k in range(n):
            t = k * dt

            if use_vel_pid:
                # ---- 第一层：位置环 P ---- PositionControl.cpp:124-138
                vel_sp = (pos_sp - pos) * np.array([MPC_XY_P, MPC_XY_P, MPC_Z_P])

                # ---- 第二层：速度环 PID ---- PositionControl.cpp:140-202
                vel_err = vel_sp - vel
                acc_sp = vel_err * np.array([MPC_XY_VEL_P_ACC,
                                             MPC_XY_VEL_P_ACC,
                                             MPC_Z_VEL_P_ACC])
                acc_sp[0] += int_xy[0]
                acc_sp[1] += int_xy[1]
                acc_sp[2] += int_z
                # 水平加速度限幅
                nh = float(np.linalg.norm(acc_sp[:2]))
                if nh > MPC_ACC_HOR_MAX:
                    acc_sp[:2] *= MPC_ACC_HOR_MAX / nh
            else:
                # ---- 单环：位置误差直接给倾角 ----
                e = pos_sp - pos
                theta_t = -kp_direct * e[0]
                phi_t = kp_direct * e[1]
                acc_sp = np.zeros(3)
                vel_err = np.zeros(3)

            # ---- 加速度 → 期望姿态 ---- PositionControl.cpp:204-221
            z_sf = -G + acc_sp[2]
            body_z = np.array([-acc_sp[0], -acc_sp[1], -z_sf])
            body_z = body_z / np.linalg.norm(body_z)

            # 倾角限幅ControlMath::limitTilt（ControlMath.cpp:58）
            cos_tilt = float(np.clip(body_z[2], -1.0, 1.0))
            if math.acos(cos_tilt) > MPC_TILTMAX_AIR:
                rej = body_z - cos_tilt * np.array([0.0, 0.0, 1.0])
                nr = float(np.linalg.norm(rej))
                if nr > 1e-9:
                    body_z = (math.cos(MPC_TILTMAX_AIR) * np.array([0.0, 0.0, 1.0])
                              + math.sin(MPC_TILTMAX_AIR) * (rej / nr))
                    body_z = body_z / np.linalg.norm(body_z)

            if use_vel_pid:
                theta_t = math.asin(max(-1.0, min(1.0, float(body_z[0]))))
                phi_t = math.atan2(-float(body_z[1]), float(body_z[2]))
            cos_tilt = float(np.clip(body_z[2], -1.0, 1.0))

            # ---- 姿态内环：一阶滞后 ----
            a_al = dt / (TAU_ATT + dt)
            phi += a_al * (phi_t - phi)
            theta += a_al * (theta_t - theta)

            # ---- 总距（collective）换算 ---- PositionControl.cpp:217-221
            #     thrust_ned_z 是 NED 下的推力 z 分量（向上为【负】）
            if use_vel_pid:
                if use_ff_thrust:
                    thrust_ned_z = acc_sp[2] * (MPC_THR_HOVER / G) - MPC_THR_HOVER
                else:
                    # ★ 没有悬停前馈：控制器不知道"托住自己"要多少油门
                    thrust_ned_z = acc_sp[2] * (MPC_THR_HOVER / G)
                coll = min(thrust_ned_z / max(cos_tilt, 1e-6), -MPC_THR_MIN)
                u = -coll
            else:
                u = MPC_THR_HOVER        # 单环：总距固定在悬停值

            u = max(0.0, min(1.0, u))

            # ---- 真正的运动学 ----
            acc = plant_acc(u, phi, theta, u_hover_true)
            vel = vel + acc * dt
            pos = pos + vel * dt

            # ---- 速度环积分（只在不饱和时积分）---- PositionControl.cpp:196-201
            if use_vel_pid:
                if MPC_THR_MIN < u < 1.0 - 1e-9:
                    int_z += vel_err[2] * MPC_Z_VEL_I_ACC * dt
                int_z = max(-G, min(G, int_z))
                int_xy += vel_err[:2] * MPC_XY_VEL_I_ACC * dt
                # 水平方向也限一下，防止演示跑飞
                int_xy = np.clip(int_xy, -MPC_ACC_HOR_MAX, MPC_ACC_HOR_MAX)

            t_hist.append(t)
            pos_hist.append(pos.copy())
            vel_hist.append(vel.copy())
            att_hist.append((phi, theta))
            u_hist.append(u)

        return (np.array(t_hist), np.array(pos_hist), np.array(vel_hist),
                np.array(att_hist), np.array(u_hist))

    sub("① 飞的全程：从 (0,0,-1) 到 (5,3,-4)")

    TARGET = np.array([5.0, 3.0, -4.0])
    t, pos, vel, att, u = simulate()

    print("    目标点 (5, 3, -4) m   （北 5 m、东 3 m、高 4 m）")
    print()
    print("    %-7s %-30s %-28s %s" % ("t (s)", "位置 NED (m)", "速度 (m/s)", "roll/pitch(°)"))
    print("    " + "-" * (W - 4))
    for i in range(0, len(t), max(1, len(t) // 14)):
        print("    %-7.2f (%7.3f,%7.3f,%7.3f)   (%7.3f,%7.3f,%7.3f)   %+7.2f/%+7.2f"
              % (t[i], pos[i, 0], pos[i, 1], pos[i, 2],
                 vel[i, 0], vel[i, 1], vel[i, 2],
                 math.degrees(att[i, 0]), math.degrees(att[i, 1])))
    print()
    print("    末端位置误差 = %.4f m" % float(np.linalg.norm(pos[-1] - TARGET)))
    print("    末端速度     = (%.4f, %.4f, %.4f) m/s"
          % (vel[-1, 0], vel[-1, 1], vel[-1, 2]))
    over_x = 100.0 * (float(np.max(pos[:, 0])) - 5.0) / 5.0
    print("    北向超调     = %+.1f%%   （峰值 %.3f m，目标 5.000 m）"
          % (over_x, float(np.max(pos[:, 0]))))
    print()
    print("""    ★ 注意那个 %.1f%% 的超调 —— **这不是 bug，是调出来的。**

      速度环的 P（1.8）和位置环的 P（0.95）只差 1.9 倍，
      所以速度环的相位滞后会吃掉位置环的一部分相位裕度，
      结果就是一个"略欠阻尼"的响应：快，但会冲过去一点。

      ★ 第 19 课会把这件事**定量**：把每一环的传递函数乘起来，
        读出相位裕度，就能算出超调应该是多少。
""" % over_x)

    sub("② 三层的「时间尺度」是分开的 —— 这是级联的全部意义")

    v_h = np.linalg.norm(vel[:, :2], axis=1)
    k_peak = int(np.argmax(v_h))
    print("    水平速度峰值 %.3f m/s，出现在 t = %.2f s" % (v_h[k_peak], t[k_peak]))

    d = np.linalg.norm(pos - TARGET, axis=1)
    band = np.where(d < 0.05 * float(np.linalg.norm(TARGET)))[0]
    if len(band):
        print("    位置进入 5%% 误差带：t = %.2f s" % t[band[0]])
    print("    末端误差 / 目标距离 = %.3f%%"
          % (100 * float(np.linalg.norm(pos[-1] - TARGET)) / float(np.linalg.norm(TARGET))))
    print()

    print("    用增益直接读出的【一阶等效带宽】：")
    print("        位置环  MPC_XY_P         = %.2f  ⇒ 带宽 ≈ %.2f rad/s"
          % (MPC_XY_P, MPC_XY_P))
    print("        速度环  MPC_XY_VEL_P_ACC = %.2f  ⇒ 带宽 ≈ %.2f rad/s"
          % (MPC_XY_VEL_P_ACC, MPC_XY_VEL_P_ACC))
    print("        姿态环  1/TAU_ATT        = %.2f  ⇒ 带宽 ≈ %.2f rad/s"
          % (1.0 / TAU_ATT, 1.0 / TAU_ATT))
    print("        ─────────────────────────────────────────────")
    print("        速度环 / 位置环     = %.1f ×" % (MPC_XY_VEL_P_ACC / MPC_XY_P))
    print("        姿态环 / 速度环     = %.1f ×" % ((1.0 / TAU_ATT) / MPC_XY_VEL_P_ACC))
    print()
    print("""    ★★ **这就是级联控制的黄金法则：内环要尽可能比外环快。**

      为什么？因为外环设计时"假装内环是理想的"。
      如果内环不够快，它带来的相位滞后会把外环的相位裕度吃掉 ——
      第 19 课要定量算这件事。

      PX4 的默认值是 **0.95 : 1.8 : 20**，每一级大约 2~10 倍。
      注意位置环和速度环只差 1.9 倍 —— **这是"不震荡"和"跟踪快"
      之间被推得很紧的一对**，也是为什么 PX4 的位置环增益
      改一点点手感就变很多（调参实战第 25 课会再遇到它）。
""")

    sub("③ 如果把速度环拿掉（位置误差直接给倾角）会怎样？")

    print("    单环设计：位置误差 → 倾角（rad），总距固定在悬停值。")
    print("    这等效于第 1 课那个**只有 P 的二阶系统**：")
    print()
    print("        θ = -kp·e_x   ⇒   ẍ = -g·θ = g·kp·e_x")
    print("        ⇒  e_x 的方程是  ë_x + g·kp·e_x = 0")
    print("        ⇒  ★ ω_n = √(g·kp)  且  **ζ = 0（无阻尼！）**")
    print()

    rows = []
    for kp in (0.02, 0.05, 0.10, 0.20):
        t2, pos2, vel2, att2, u2 = simulate(
            t_end=12.0, use_vel_pid=False, kp_direct=kp)
        vmax = float(np.max(np.linalg.norm(vel2[:, :2], axis=1)))
        over = 100.0 * (float(np.max(pos2[:, 0])) - 5.0) / 5.0
        # 数一数穿过目标线几次 —— 没有阻尼就应该一直穿来穿去
        sgn = np.sign(pos2[:, 0] - 5.0)
        cross = int(np.sum(np.abs(np.diff(sgn)) > 0))
        # 最后 2 秒内的水平残差幅值 —— 阻尼为 0 就不该衰减
        tail = pos2[t2 > (t2[-1] - 2.0), 0]
        z_last = float(pos2[-1, 2])
        rows.append(["%.2f" % kp, "%.3f" % math.sqrt(G * kp), "%.3f" % vmax,
                     "%+.1f%%" % over, "%d" % cross,
                     "%.3f" % float(np.max(np.abs(tail - 5.0))),
                     "%.1f" % z_last])
    tab(rows, ["kp", "ωn (rad/s)", "速度峰值 (m/s)", "超调",
               "穿越目标次数", "末段残差幅值 (m)", "末端 z (m)"])

    print("""    ★ 三件事：

        ① **ζ = 0 ⇒ 振幅不衰减。** 看"末段残差幅值"一列：
           12 s 后离目标还有那么多米，**而且这个数不会变小**
           —— 因为系统里没有任何耗散项，能量守恒。
           （真实系统还有气动阻力，但**那是"顺带的"阻尼，不可控；
             我们真正需要的是由速度反馈确定的、可设计的 ζ。**）
        ② **超调随 kp 增大而增大** —— 因为 ωn = √(g·kp) 变大，
           而 ζ 还是 0，振荡只会更激烈。
        ③ **垂直方向完全没救**：总距固定在悬停油门，
           一旦倾斜就掉（a_z = g·(1 - cosθ) > 0 ⇒ 往下加速），
           而且**没有任何机制把它拉回来**（连积分都没有）。
           看最后一列：z 从 -1 一路涨到 %+.1f（已经远远穿过地面 z = 0）。
           —— 单环连"不掉高度"都做不到。

      ⇒ 结论：**姿态角和位置之间必须有一层速度环，
        它提供的不只是"更快"，而是整个系统的阻尼 ζ。**
        这就是 `_velocityControl()`（PositionControl.cpp:140）
        存在的根本理由。
""" % z_last)

    # ---- 画图 ----
    if _HAS_MPL:
        _plot_cascade(t, pos, vel, att, u)
    else:
        print("    （没装 matplotlib，跳过画图）")
        print()
_HAS_MPL = False
_CJK = False
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as _fm

    for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                  "Source Han Sans SC", "WenQuanYi Micro Hei",
                  "PingFang SC", "Heiti SC"):
        try:
            _fm.findfont(_name, fallback_to_default=False)
        except Exception:
            continue
        matplotlib.rcParams["font.sans-serif"] = [_name]
        matplotlib.rcParams["axes.unicode_minus"] = False
        _CJK = True
        break

    _HAS_MPL = True
except ImportError:
    pass


def _t(zh, en):
    return zh if _CJK else en


def _plot_cascade(t, pos, vel, att, u):
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5))

    ax = axes[0][0]
    ax.plot(t, pos[:, 0], label=_t("北 (x)", "North x"))
    ax.plot(t, pos[:, 1], label=_t("东 (y)", "East y"))
    ax.axhline(5.0, ls=":", c="C0", lw=1)
    ax.axhline(3.0, ls=":", c="C1", lw=1)
    ax.set_title(_t("位置（水平）—— 外环", "position (horizontal) - outer"))
    ax.set_xlabel("t [s]")
    ax.set_ylabel("m")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = axes[0][1]
    ax.plot(t, -pos[:, 2], label=_t("高度", "altitude"))
    ax.axhline(4.0, ls=":", c="C0", lw=1)
    ax.set_title(_t("高度", "altitude"))
    ax.set_xlabel("t [s]")
    ax.set_ylabel("m")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = axes[1][0]
    ax.plot(t, vel[:, 0], label=_t("vN", "vN"))
    ax.plot(t, vel[:, 1], label=_t("vE", "vE"))
    ax.plot(t, vel[:, 2], label=_t("vD", "vD"))
    ax.set_title(_t("速度 —— 中环", "velocity - middle"))
    ax.set_xlabel("t [s]")
    ax.set_ylabel("m/s")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = axes[1][1]
    ax.plot(t, np.degrees(att[:, 0]), label=_t("roll", "roll"))
    ax.plot(t, np.degrees(att[:, 1]), label=_t("pitch", "pitch"))
    ax.set_title(_t("姿态设定值 —— 内环的目标", "attitude setpoint - inner target"))
    ax.set_xlabel("t [s]")
    ax.set_ylabel("deg")
    ax.grid(alpha=0.3)
    ax.legend()

    plt.tight_layout()
    out = "position_cascade.png"
    plt.savefig(out, dpi=110)
    plt.close(fig)
    print("    ✅ 图已存：%s%s" % (out, "" if _CJK else "（无中文字体，标签用英文）"))
    print()


# ---------------------------------------------------------------------------
# 实验 4：前馈 vs 反馈
# ---------------------------------------------------------------------------

def exp4():
    head("实验 4  ★ 前馈 vs 反馈：为什么悬停油门必须前馈")

    print("""
  先分清两件事（这是控制里最重要的一对概念）：

      反馈（feedback）  —— 看【误差】出力。误差为 0 ⇒ 输出为 0
      前馈（feed-forward）—— 看【指令】出力。跟误差无关

  ★ 位置控制里最典型的前馈就是**悬停油门**：

      PX4    MPC_THR_HOVER  = 0.5        multicopter_position_control_params.c:49
      AP     MOT_THST_HOVER = 0.35 左右  （由学习或参数给定）

  **"托住自己"这件事不需要反馈 —— 它是已知的、常值的。**
  把它交给积分项去找，是浪费积分项、也是延迟的来源。
""")

    sub("① 没有前馈：积分项要「爬」多久才找到悬停油门")

    # 一维高度通道。NED 下 z 向下为正，起飞就是让 z 变小。
    #   被控对象：  a_z = g·(1 - u/u_hover)
    #   u 是归一化油门（0~1），u_hover 是"刚好托住自己"的油门。
    #   控制器【不知道】u_hover —— 这正是问题所在。
    dt = 0.004
    t_end = 12.0
    n = int(t_end / dt)
    Z_SP = -3.0

    def run_height(u_ff, kp_pos=1.0, kp_vel=4.0, ki_vel=2.0,
                   u_hover_true=0.5, u_hover_nom=0.5, z_sp=Z_SP):
        """跑一维高度通道。

        u_ff = 控制器使用的悬停前馈（真值是 u_hover_true）。

        ★ 控制器结构**完全照抄 PX4**：

            位置环    vz_sp = (z_sp - z) * kp_pos
            速度环    a_z_cmd = integ + err * kp_vel        （NED 加速度）
            加速度→油门  u = u_ff - a_z_cmd * (u_hover_nom / G)
                         ↑ 对应 PositionControl.cpp:217
                           thrust_ned_z = _acc_sp(2)*(hover/g) - hover
                           （符号：NED 下"向上"是负的加速度 ⇒ 负得越多油门越大）

            被控对象  a_z = g·(1 - u/u_hover_true)
        """
        z = 0.0
        vz = 0.0
        integ = 0.0
        hist = []
        for k in range(n):
            vz_sp = (z_sp - z) * kp_pos          # 位置环 P
            # ★ 速度限幅 MPC_Z_VEL_MAX_UP = 3.0 / _DN = 1.5
            #   multicopter_position_control_limits_params.c:64,81
            vz_sp = max(-3.0, min(1.5, vz_sp))
            err = vz_sp - vz                     # 速度环误差
            a_z_cmd = integ + err * kp_vel       # NED 加速度设定值
            # ★ 加速度限幅 MPC_ACC_UP_MAX = 4.0 / MPC_ACC_DOWN_MAX = 3.0
            #   multicopter_altitude_mode_params.c:44,56
            a_z_cmd = max(-4.0, min(3.0, a_z_cmd))
            u_raw = u_ff - a_z_cmd * (u_hover_nom / G)
            u = max(0.12, min(1.0, u_raw))       # MPC_THR_MIN ~ 1
            a = G * (1.0 - u / u_hover_true)     # ★ 真正的被控对象
            vz += a * dt
            z += vz * dt
            # ★ 反饱和（PositionControl.cpp:155-158 的等价物）：
            #   只在"误差还想把输出往饱和里推"时才停积分
            sat_low = (u_raw <= 0.12 + 1e-12 and a_z_cmd > 0.0)
            sat_high = (u_raw >= 1.0 - 1e-12 and a_z_cmd < 0.0)
            if not (sat_low or sat_high):
                integ += err * ki_vel * dt
            # PositionControl.cpp:143 —— 垂直积分项限幅 ±g
            integ = max(-G, min(G, integ))
            hist.append((k * dt, z, vz, u, integ))
        return np.array(hist)

    print("    目标高度 3 m（NED 下 z = -3），悬停油门真值 u_hover = 0.5")
    print("    控制器：位置环 P=1.0，速度环 P=4.0 / I=2.0")
    print("    限幅：  速度 -3.0 ~ +1.5 m/s，加速度 -4.0 ~ +3.0 m/s²，油门 ≥ 0.12")
    print("              （全是 PX4 出厂默认值，见下面源码行号）")
    print("    被控对象：a_z = g·(1 - u/u_hover)")
    print()

    print("    ★★ 先做一道【静态】的算术，它比动态曲线更能说明问题。")
    print()
    print("    PX4 的 加速度→油门 转换是（PositionControl.cpp:217）：")
    print()
    print("        u = u_ff - a_z_cmd · (hover/g)")
    print()
    print("    加速度限幅 MPC_ACC_UP_MAX = 4.0，MPC_ACC_DOWN_MAX = 3.0，")
    print("    所以 **a_z_cmd 的可用范围是 [-4, +3]**（单位 m/s²，NED）。")
    print("    代进去，看油门能落在什么区间：")
    print()

    rows = []
    for uff in (0.0, 0.25, 0.40, 0.50, 0.60, 0.70):
        u_lo = uff - 3.0 * (0.5 / G)         # a_z_cmd = +3（向下加速）
        u_hi = uff + 4.0 * (0.5 / G)         # a_z_cmd = -4（向上加速）
        ok = (u_lo < 0.5 < u_hi) and u_hi <= 1.0 and u_lo >= 0.12
        rows.append(["%.2f" % uff,
                     "%.3f" % u_lo, "%.3f" % u_hi,
                     "%.3f" % (u_hi - u_lo),
                     "能" if ok else "★ 不能"])
    tab(rows, ["前馈 u_ff", "油门下限", "油门上限", "可用行程",
               "能托住悬停(0.5)吗？"])

    print("""    ★★★ **这张表是本实验最重要的一张。**

        ① **前馈就是执行器的"工作点"。**
           有前馈（u_ff = 0.5）时，油门只需要在 [0.347, 0.704] 里动
           —— 行程 0.357，两头都留着大把余量。

        ② **没有前馈（u_ff = 0）时，油门上限只有 0.204**
           —— 而托住自己需要 0.5。
           **⇒ 控制器连"飞起来"的权限都没有，无论积分怎么涨。**
           （因为积分项也受加速度限幅约束，涨不过 -4。）

        ③ 这就是"**前馈把执行器的动态范围省下来了**"这句话的定量版本：
           重力那个常值负载不由反馈去扛，反馈的行程才够用来做机动。

      ⚠️ 所以"去掉前馈看看会怎样"这种实验，在 PX4 的结构里会直接发散 ——
        下面真的跑一遍给你看。
""")

    h_noff = run_height(0.0)
    h_ff = run_height(0.5)

    def settle_time(h, target=Z_SP, band=0.05):
        err = np.abs(h[:, 1] - target)
        out = np.where(err >= band)[0]
        if len(out) == 0:
            return 0.0
        j = int(out[-1]) + 1
        return h[j, 0] if j < len(h) else float("nan")

    def rise_time(h, lo=0.1, hi=0.9, target=Z_SP):
        d = np.abs(h[:, 1])
        total = abs(target)
        t_lo = np.where(d >= lo * total)[0]
        t_hi = np.where(d >= hi * total)[0]
        if not len(t_lo) or not len(t_hi):
            return float("nan")
        return h[t_hi[0], 0] - h[t_lo[0], 0]

    sub("② 动态：真的跑一遍（无前馈 vs 有前馈）")

    print("    %-7s %-32s %s" % ("t (s)", "无前馈  高度/油门/积分项",
                                 "有前馈  高度/油门/积分项"))
    print("    " + "-" * (W - 4))
    for i in range(0, len(h_noff), max(1, len(h_noff) // 8)):
        print("    %-7.2f %8.3f / %6.3f / %8.4f       %7.3f / %6.3f / %8.4f"
              % (h_noff[i, 0], -h_noff[i, 1], h_noff[i, 3], h_noff[i, 4],
                 -h_ff[i, 1], h_ff[i, 3], h_ff[i, 4]))
    print()

    rows = []
    for name, h in (("无前馈", h_noff), ("有前馈", h_ff)):
        rows.append([name,
                     "%.2f" % rise_time(h),
                     "%.4f" % float(np.max(h[h[:, 0] < 2.0, 1])),
                     "%.4f" % h[-1, 1],
                     "%.4f" % h[-1, 4]])
    tab(rows, ["配置", "上升时间 (s)", "起步 2 s 内最低下沉 (m)",
               "末端 z (m)", "末端积分项"])

    print("""    ★ 对上了 —— **无前馈那一路是发散的**
      （z 一直增大 = 一直往下掉，2 s 内就下沉了 %.1f m）。

      而且注意它的积分项：**卡在 -9.8066 不动了**
      —— 那是加速度限幅 + 积分限幅（PositionControl.cpp:143 的 ±g）
      双重顶死的结果。**控制器已经尽力了，硬件权限不够。**

      ★ 所以"前馈"这件事在 PX4 里不是"优化"，是"必需"：
        `thrust_ned_z = acc*(hover/g) - hover` 里那个 **`- hover`**
        就是内建的前馈，它是整套公式能工作的前提。

      ⚠️ 反过来说：**如果 MPC_THR_HOVER 参数设错了，
        整个高度通道的工作点就偏了** —— 这正是为什么
        PX4 提供了悬停推力估计器（HTE）去自动修正它。
""" % (float(np.max(h_noff[h_noff[:, 0] < 2.0, 1]))))

    sub("③ 前馈只是「不准」（而不是「没有」）会怎样？")

    rows = []
    for uh in (0.35, 0.44, 0.50, 0.56, 0.65):
        h = run_height(uh, u_hover_true=0.5)
        rows.append(["%.2f" % uh,
                     "%+.1f%%" % (100 * (uh - 0.5) / 0.5),
                     "%.2f" % rise_time(h),
                     "%+.4f" % float(np.max(h[h[:, 0] < 2.0, 1])),
                     "%+.4f" % float(np.min(h[h[:, 0] < 2.0, 1])),
                     "%+.4f" % h[-1, 4],
                     "%.4f" % h[-1, 1]])
    tab(rows, ["前馈给的 u_ff", "前馈相对误差", "上升时间 (s)",
               "起步 2 s 内最低 (m)", "起步 2 s 内最高 (m)",
               "末端积分项", "末端 z (m)"])

    print("""    ★ 结论：**前馈偏离真值 ⇒ 多出来的部分必然由积分项补上。**

        前馈偏小 ⇒ 积分项是负的（继续往上补）
        前馈偏大 ⇒ 积分项是正的（往下压）

      **两种都能收敛** —— 因为积分项在兜底。但：

        · 前馈偏小 ⇒ 上升变慢（前馈不够，积分慢慢补）
        · 前馈偏大 ⇒ 一开始**上冲**（"最高"一列是正的），再被压回来
        · 偏到 +30% 时，末端 z = -30.3 ⇒ **已经跑飞了**
          （前馈给太多，油门工作点上移，加速度限幅的方向性不对称暴露出来）

      ⇒ ★ 工程经验：**悬停油门宁可给大一点点，但不能多太多。**
        真实安全边界由 MPC_THR_MIN/MPC_THR_MAX 和加速度限幅共同决定。

      ★ 这也解释了 PX4 为什么专门做了**悬停推力估计器**：

            src/modules/mc_hover_thrust_estimator/
                MulticopterHoverThrustEstimator.cpp
                zero_order_hover_thrust_ekf.cpp

        MulticopterPositionControl.cpp:263-265, 374-378
            if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
                _control.setHoverThrust(_param_mpc_thr_hover.get());
            ...
            if (_hover_thrust_estimate_sub.update(&hte)) {
                _control.updateHoverThrust(hte.hover_thrust);
            }

        以及 PositionControl.cpp:75 的 `updateHoverThrust()` ——
        **换前馈值时要同步补偿积分项**，否则飞机会"抖"一下。
        （源码在 PositionControl.cpp:84-88）
""")

    sub("④ 位置环也有前馈：速度前馈 / 加速度前馈")

    print("""    PX4 的 setpoint 里除了位置，还可以直接给速度和加速度：

        PositionControl.cpp:101-103
            _pos_sp = Vector3f(setpoint.position);
            _vel_sp = Vector3f(setpoint.velocity);
            _acc_sp = Vector3f(setpoint.acceleration);

        然后 PositionControl.cpp:129 把它们**相加**：
            ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);

    ★ 这就是"**前馈 + 反馈叠加**"的标准写法：

        最终速度设定值 = 【前馈速度】（轨迹规划给的）
                       + 【位置误差 × P】（反馈补的）

    **好处**：轨迹跟踪时，前馈已经给了大部分速度指令，
    位置误差只剩"模型不准那一小点"，反馈增益可以设得小、系统更稳。
    （第 19 课的相位裕度会告诉我们：反馈增益越大，裕度越小。）
""")

    print("""    ⚠️ 但注意 `addIfNotNanVector3f` 这个名字 ——
      PX4 用 **NaN 表示"这一项没有设定值"**。
      这是 uORB 消息里表达"可选字段"的一个工程技巧：
      NaN 参与加法会污染整个输出，所以专门写了 `addIfNotNan`。

      ⇒ **看飞控源码时看到一堆 `PX4_ISFINITE` 检查，不要以为是多余的** ——
        它们是在实现"前馈/反馈可独立开关"这件事。
""")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("欠驱动：4 个输入 6 个自由度", exp1),
    "2": ("★★ 位置 → 姿态反解（PX4 thrustToAttitude）", exp2),
    "3": ("★★ 三层级联闭环仿真", exp3),
    "4": ("★ 前馈 vs 反馈：悬停油门", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        for k in sorted(EXPERIMENTS):
            print("\n" + "#" * W)
            print("#  实验 %s：%s" % (k, EXPERIMENTS[k][0]))
            print("#" * W)
            EXPERIMENTS[k][1]()
        print()
        print("=" * W)
        print("  完成。")
        print("=" * W)
        return 0

    for a in args:
        if a not in EXPERIMENTS:
            print("  未知实验：%s（可选 %s）" % (a, "/".join(sorted(EXPERIMENTS))))
            return 1
        EXPERIMENTS[a][1]()

    print()
    print("=" * W)
    print("  完成。")
    print("=" * W)
    return 0


if __name__ == "__main__":
    sys.exit(main())
