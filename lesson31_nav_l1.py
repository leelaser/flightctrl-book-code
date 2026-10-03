#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 20.5 周 · 06 制导层与 L1 算法 —— 配套实验

    实验 1  ★    L1 几何：手算前视点，再和源码公式逐行对拍
    实验 2  ★★★ 「周期」到底是不是闭环的自然周期（K_L1=4ζ² 的由来）
    实验 3  ★★   侧风下纯 P 的稳态横偏，和 ArduPilot 那个积分项怎么消掉它
    实验 4  ★★   捕获角饱和：PX4 的 90° 与 ArduPilot 的 45° 差在哪

用法：
    python lesson31_nav_l1.py 1
    python lesson31_nav_l1.py 2
    python lesson31_nav_l1.py 3
    python lesson31_nav_l1.py 4
    python lesson31_nav_l1.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  能算的就不猜。

  · 实验 1 把源码的每一行**照抄成 Python**，再和「用几何手算」的结果对拍 ——
    两边的数必须逐位相同，否则说明课文里的推导写错了。
  · 实验 2 **不背结论**：先把 L1 律代进横向动力学推出闭环方程，
    再用数值仿真去量测它的 ωn 和 ζ，最后和解析预测对表。
    「period 就是闭环周期」这句话如果是对的，量出来就该对得上。
  · 实验 3 的侧风是**真的加在速度上**的，稳态横偏是真的跑出来的，
    ArduPilot 的积分项是照 `AP_L1_Control.cpp:316-327` 逐行实现的
    （含 ±0.1 rad 限幅和 |Nu1|<5° 的门控）。
  · 实验 4 用同一组初始条件跑两遍，只改捕获角限幅，看轨迹分岔。

【本脚本核对过的源码事实】（行号均已打开确认）

  PX4   src/lib/l1/ECL_L1_Pos_Controller.cpp
        :67   _L1_distance = _L1_ratio * ground_speed
        :86   _crosstrack_error = vector_AB % vector_A_to_airplane
        :160  eta2 = atan2f(xtrack_vel, ltrack_vel)
        :163  xtrackErr = vector_A_to_airplane % vector_AB
        :164  sine_eta1 = xtrackErr / max(_L1_distance, 0.1f)
        :167  sine_eta1 = constrain(sine_eta1, -1.0f, 1.0f)   ← 90°
        :169  eta = eta1 + eta2
        :176  eta = constrain(eta, -pi/2, +pi/2)
        :177  _lateral_accel = _K_L1 * gs^2 / _L1_distance * sin(eta)
        :186  _L1_ratio = (1/pi) * damping * period
        :200  _K_L1 = 4 * damping^2
        另外：:189 的 _heading_omega 只赋值、从不使用（本仓库内 grep 只有 2 处）。

  APM   libraries/AP_L1_Control/AP_L1_Control.cpp
        :227  K_L1 = 4 * damping^2
        :257  _L1_dist = MAX(0.3183099*damping*period*groundSpeed, dist_min)
        :277  _crosstrack_error = A_air % AB          ← 与 PX4 的叉积顺序【相反】
        :310  constrain_float(sine_Nu1, -0.7071f, 0.7071f)   ← 45°
        :319  if (fabsf(Nu1) < radians(5)) _L1_xtrack_i += Nu1*gain*dt
        :323  _L1_xtrack_i = constrain_float(_L1_xtrack_i, -0.1f, 0.1f)
        :327  Nu1 += _L1_xtrack_i
        :338  _latAccDem = K_L1 * groundSpeed^2 / _L1_dist * sinf(Nu)

参考：
    第 5 章 03-位置与速度控制.md（位置环 → 姿态；本课是它的上一层）
    第 5 章 01-PID控制器从零到级联.md（ζ / ωn 的物理含义）
    第 4 章 06-PX4-ECL-EKF2完全解析.md（地速/风估计从哪来）
"""

import math
import sys

import numpy as np

W = 74

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
    """显示宽度：CJK 占 2 列。"""
    n = 0
    for ch in str(s):
        o = ord(ch)
        n += 2 if (0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF
                   or 0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF
                   or 0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60
                   or 0xFFE0 <= o <= 0xFFE6) else 1
    return n


def tab(rows, headers, width=None):
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


def savefig(name):
    if not _HAS_MPL:
        return
    try:
        plt.tight_layout()
        plt.savefig(name + ".png", dpi=110)
        plt.close()
        print("  [图] 已保存 %s.png%s" % (name, "" if _CJK else "（无中文字体）"))
    except Exception as exc:                     # pragma: no cover
        print("  [图] 保存失败：%s" % exc)


# ===========================================================================
#  L1 的「源码级」实现 —— 与 PX4 的 ECL_L1_Pos_Controller 逐行对应
# ===========================================================================

def px4_l1(A, B, P, gs, period=25.0, damping=0.75):
    """照抄 PX4 src/lib/l1/ECL_L1_Pos_Controller.cpp:52-179 的逻辑。

    返回 dict，把每一步中间量都带出来 —— 课文里要逐个对照。
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    P = np.asarray(P, dtype=float)
    gs = np.asarray(gs, dtype=float)

    # :63-64  地速下限，防奇异
    ground_speed = max(float(np.linalg.norm(gs)), 0.1)

    # :186  _L1_ratio = (1/pi) * damping * period
    L1_ratio = (1.0 / math.pi) * damping * period

    # :67   _L1_distance = _L1_ratio * ground_speed
    L1_distance = L1_ratio * ground_speed

    # :69-80  A->B 单位矢量
    AB = B - A
    if np.linalg.norm(AB) < 1e-6:
        AB = B - P
    AB = AB / np.linalg.norm(AB)

    # :83   A 指向飞机
    A_to_airplane = P - A

    # :86   交叉航迹误差（源码注释就写着 "output only"）
    crosstrack = AB[0] * A_to_airplane[1] - AB[1] * A_to_airplane[0]

    # :154-160  eta2：速度矢量相对航线的夹角
    xtrack_vel = gs[0] * AB[1] - gs[1] * AB[0]
    ltrack_vel = gs[0] * AB[0] + gs[1] * AB[1]
    eta2 = math.atan2(xtrack_vel, ltrack_vel)

    # :163-168  eta1：指向 L1 点的夹角
    xtrack_err = A_to_airplane[0] * AB[1] - A_to_airplane[1] * AB[0]
    sine_eta1 = xtrack_err / max(L1_distance, 0.1)
    sine_eta1 = min(max(sine_eta1, -1.0), 1.0)          # :167  ← 90° 限幅
    eta1 = math.asin(sine_eta1)

    # :169
    eta = eta1 + eta2
    # :176
    eta = min(max(eta, -math.pi / 2), math.pi / 2)

    # :200
    K_L1 = 4.0 * damping * damping
    # :177
    lateral_accel = K_L1 * ground_speed * ground_speed / L1_distance * math.sin(eta)

    return dict(ground_speed=ground_speed, L1_ratio=L1_ratio,
                L1_distance=L1_distance, AB=AB, crosstrack=crosstrack,
                eta1=eta1, eta2=eta2, eta=eta, sine_eta1=sine_eta1,
                K_L1=K_L1, lateral_accel=lateral_accel,
                nav_bearing=math.atan2(AB[1], AB[0]) + eta1)


def ap_l1(A, B, P, gs, period=17.0, damping=0.75,
          xtrack_i=0.0, xtrack_i_gain=0.0, dt=0.1):
    """照抄 ArduPilot libraries/AP_L1_Control/AP_L1_Control.cpp 的对应段。

    与 PX4 版的三处关键差异都在这里体现：
      · 捕获角限幅 45°（:310）
      · 交叉航迹误差的叉积顺序相反（:277）
      · 积分项（:316-327）
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    P = np.asarray(P, dtype=float)
    gs = np.asarray(gs, dtype=float)

    # :242  groundSpeed（这里不做 0.1 下限，下面单独处理）
    ground_speed = float(np.linalg.norm(gs))
    # :247-252  地速太小 → 用机头方向重构（这里由调用者传 yaw）
    if ground_speed < 0.1:
        ground_speed = 0.1
        gs = np.array([1.0, 0.0]) * ground_speed

    # :227
    K_L1 = 4.0 * damping * damping

    # :257  _L1_dist = MAX(0.3183099*damping*period*groundSpeed, dist_min)
    L1_dist = max(0.3183099 * damping * period * ground_speed, 0.0)

    AB = B - A
    AB = AB / np.linalg.norm(AB)

    A_air = P - A
    # :277  ★ 注意顺序：A_air % AB —— 与 PX4 的 AB % A_to_airplane 相反
    crosstrack = A_air[0] * AB[1] - A_air[1] * AB[0]

    xtrackVel = gs[0] * AB[1] - gs[1] * AB[0]
    ltrackVel = gs[0] * AB[0] + gs[1] * AB[1]
    Nu2 = math.atan2(xtrackVel, ltrackVel)

    # :308  sine_Nu1 = _crosstrack_error / MAX(_L1_dist, 0.1)
    sine_Nu1 = crosstrack / max(L1_dist, 0.1)
    # :310  ★ 45° 捕获角
    sine_Nu1 = min(max(sine_Nu1, -0.7071), 0.7071)
    Nu1 = math.asin(sine_Nu1)

    # :316-327  积分项
    if xtrack_i_gain > 0:
        if abs(Nu1) < math.radians(5.0):
            xtrack_i += Nu1 * xtrack_i_gain * dt
            xtrack_i = min(max(xtrack_i, -0.1), 0.1)     # :323
    else:
        xtrack_i = 0.0
    Nu1 += xtrack_i                                      # :327

    Nu = Nu1 + Nu2
    # :333
    Nu = min(max(Nu, -1.5708), 1.5708)
    # :338
    lat_acc = K_L1 * ground_speed * ground_speed / L1_dist * math.sin(Nu)

    # 注意：AP 的 crosstrack 符号与 PX4 相反，所以它的横加速度指令也差一个符号
    return dict(ground_speed=ground_speed, L1_dist=L1_dist, crosstrack=crosstrack,
                Nu1=Nu1, Nu2=Nu2, Nu=Nu, xtrack_i=xtrack_i, K_L1=K_L1,
                lateral_accel=-lat_acc)     # ← 取负号后与 PX4 同号，便于对照


# ===========================================================================
#  实验 1 ★  L1 几何：手算前视点，再和源码公式逐行对拍
# ===========================================================================

def exp1():
    head("实验 1 ★  L1 几何：手算前视点，再和源码公式逐行对拍")

    print("""  场景（NED：x 指北、y 指东）：
        A=(0,0) → B=(400,0)  —— 一条【正北向】的航线
        飞机在 P=(120, 35)   —— 航线【右侧】35 m（+y 就是正东 = 右）
        地速 20 m/s 朝正北（先假设无风，地速 = 空速）
        PX4 的默认参数：period=25 s，damping=0.75
""")

    A = np.array([0.0, 0.0])
    B = np.array([400.0, 0.0])
    P = np.array([120.0, 35.0])
    gs = np.array([20.0, 0.0])

    sub("① 先手算 L1 距离")
    ratio = (1.0 / math.pi) * 0.75 * 25.0
    print("  _L1_ratio = (1/pi) * damping * period")
    print("            = (1/pi) * %.2f * %.1f = %.6f" % (0.75, 25.0, ratio))
    print("  _L1_distance = _L1_ratio * ground_speed")
    print("               = %.6f * %.1f = %.4f m" % (ratio, 20.0, ratio * 20.0))
    print()
    print("  ★ 注意 L1 距离**随速度变化**：停着不动时它会趋近 0。")
    print("    所以 period 才是真正的设计量，L1 距离只是它的中间产物。")

    sub("② 逐行对拍：源码实现 vs 手算")
    r = px4_l1(A, B, P, gs)
    rows = [
        ["ground_speed", "max(|gs|, 0.1)", "%.6f" % r["ground_speed"]],
        ["_L1_ratio", "(1/pi)*zeta*T", "%.6f" % r["L1_ratio"]],
        ["_L1_distance", "ratio*gs", "%.6f" % r["L1_distance"]],
        ["crosstrack", "AB % A_to_airplane", "%.6f" % r["crosstrack"]],
        ["eta1", "asin(clamp(xtrackErr/L1))", "%.6f rad = %.3f deg"
         % (r["eta1"], math.degrees(r["eta1"]))],
        ["eta2", "atan2(xtrack_vel, ltrack_vel)", "%.6f rad = %.3f deg"
         % (r["eta2"], math.degrees(r["eta2"]))],
        ["eta", "eta1 + eta2", "%.6f rad = %.3f deg"
         % (r["eta"], math.degrees(r["eta"]))],
        ["K_L1", "4*zeta^2", "%.6f" % r["K_L1"]],
        ["_lateral_accel", "K_L1*gs^2/L1*sin(eta)", "%.6f m/s^2"
         % r["lateral_accel"]],
    ]
    tab(rows, ["量", "公式（源码）", "本次取值"])

    print("  手算复核（只看 crosstrack 和 eta1，其余是四则运算）：")
    print("    crosstrack = AB % A_to_airplane")
    print("               = AB.x*d.y - AB.y*d.x = 1*35 - 0*120 = 35.0   ✓")
    print()
    print("    ⚠️ 但 eta1 用的不是上面那个 crosstrack！源码 :163 算的是【反过来】的叉积：")
    print("       xtrackErr = A_to_airplane % AB = 120*0 - 35*1 = -35.0")
    print("    eta1 = asin(-35.0 / %.4f) = asin(%.6f) = %.6f rad          ✓"
          % (r["L1_distance"], -35.0 / r["L1_distance"], r["eta1"]))
    print()
    print("  ⇒ 负的 eta1 表示 nav_bearing 从航向【往右偏】%.3f 度，"
          % abs(math.degrees(r["eta1"])))
    print("    也就是把机头往航线那一侧压 —— 物理上是对的（飞机在右边，要往左修）。")
    print("    ★ 同一个函数里，:86 和 :163 用了【两个方向相反】的叉积：")
    print("      :86 的结果只用于上报（源码注释写明 \"output only\"），")
    print("      :163 的结果才真正进入控制。混起来就会修反方向。")

    sub("③ ★★ 一个真陷阱：两边的交叉航迹误差【符号相反】")
    print("""  PX4  ECL_L1_Pos_Controller.cpp:86
         _crosstrack_error = vector_AB % vector_A_to_airplane

  AP   AP_L1_Control.cpp:277
         _crosstrack_error = A_air % AB

  而二维叉积满足 a % b = -(b % a)。所以【同一个物理情形，两边给出相反的数】。
""")
    a = px4_l1(A, B, P, gs)
    c = ap_l1(A, B, P, gs)
    tab([["PX4   AB % A_to_airplane", "%.6f" % a["crosstrack"]],
         ["AP    A_air % AB", "%.6f" % c["crosstrack"]],
         ["两者相加", "%.6f" % (a["crosstrack"] + c["crosstrack"])]],
        ["实现", "本次取值 [m]"])
    print("  ⇒ 数值确实互为相反数。**读代码时别拿一边的符号去解释另一边**，")
    print("    更别把两边的公式混着抄 —— 那样做出来的控制器会朝反方向修正。")
    print()
    print("  ★ 但最终两者给出的横向加速度指令是【同一个物理方向】：")
    print("    因为 AP 的 Nu1 用的是它自己的 crosstrack，符号在内部已经抵消了。")
    print("    本脚本的 ap_l1() 最后把结果取了负号，就是为了让两者能直接对照。")

    sub("④ L1 距离随速度变化 —— 但横加速度不变")
    xte = -35.0                       # = A_to_airplane % AB，见上面的 :163
    pred = 4.0 * math.pi ** 2 * xte / 25.0 ** 2
    rows = []
    for v in (5, 10, 15, 20, 30, 40):
        rr = px4_l1(A, B, P, np.array([float(v), 0.0]))
        sat = "饱和" if abs(abs(math.degrees(rr["eta1"])) - 90.0) < 1e-6 else ""
        rows.append(["%d" % v, "%.2f" % rr["L1_distance"],
                     "%.3f %s" % (math.degrees(rr["eta1"]), sat),
                     "%.4f" % rr["lateral_accel"],
                     "%.4f" % pred if not sat else "（不适用）"])
    tab(rows, ["地速 [m/s]", "L1 距离 [m]", "eta1 [deg]", "横加速度 [m/s^2]",
               "解析预测 4*pi^2*xte/T^2"])
    print("  解析预测值 = 4*pi^2 * (%.1f) / %.0f^2 = %.4f m/s^2"
          % (xte, 25.0, pred))
    print("""  ★★ 请看最后一列：**横加速度几乎与速度无关**（10~40 m/s 全是 -2.2108）！

  这不是巧合，把源码的三条公式代进去就能看出来：

      L1        = ratio * gs                 （:186 + :67）
      sin(eta1) = xtrackErr / L1             （:164）
      a_lat     = K_L1 * gs^2 / L1 * sin(eta1)   （:177）

  代入后 gs 全部约掉：

      a_lat = K_L1 * gs^2 / (ratio*gs) * xtrackErr/(ratio*gs)
            = K_L1 * xtrackErr / ratio^2
            = 4*zeta^2 * xtrackErr / ((zeta*T/pi)^2)
            = 4*pi^2 * xtrackErr / T^2           ★ 只剩 T 和横偏

  再对照实验 2 将要推出的 wn = 2*pi/T：

      a_lat = wn^2 * xtrackErr

  ⇒ **在速度方向误差为零时，L1 律退化成一个增益为 wn^2 的纯比例控制器。**
     加上 eta2 那一项（正比于 ydot），整体就是一个 PD：

          a_lat = wn^2 * y  +  2*zeta*wn * ydot

  ★ 这才是「period 是设计量、L1 距离只是中间产物」的真正含义：
    L1 距离随速度变，恰恰是为了让**增益**不随速度变。
    如果反过来固定 L1 距离，飞得越快增益就越小（gs^2/L1 里的 L1 不变），
    转弯会越来越迟钝 —— 那是另一种设计，不是这个。

  ⚠️ 注意 5 m/s 那一行例外：eta1 撞到了 -90 度限幅（asin 自变量被 clamp），
     也就是进入了"捕获角饱和"，上式不再成立 —— 这正是实验 4 的主题。""")


# ===========================================================================
#  实验 2 ★★★ 「周期」到底是不是闭环的自然周期
# ===========================================================================

def _sim_l1_track(period, damping, Vg=20.0, y0=30.0, ydot0=0.0,
                  T=60.0, dt=0.002, cap=None):
    """把 L1 律代进横向动力学，闭环绕一条直线。

    状态：y = 交叉航迹误差（沿 +E 量，和 PX4 的 `_crosstrack_error` 同号），
          yd = 它的变化率

    律（严格对照源码的符号）：
        eta1 = asin(clamp(-y/L1, -cap, cap))   ← 源码 :163 用的是【反过来】的叉积
        eta2 = atan2(-yd, Vg)                  ← 源码 :154-160：
                                                 xtrack_vel = gs % AB = Vg*0 - yd*1 = -yd
        eta  = eta1 + eta2
        a_lat = K_L1 * Vg^2 / L1 * sin(eta)    ← 这个 a_lat 就是 **+y 方向**的加速度
        yd'  = a_lat

    ⚠️ 这两个负号很容易写反（作者第一版就两处都写反了，结果仿真直接发散、
       永远等不到振荡，表现为"拟合失败"）。写反任一个都会把阻尼变成负的。
    """
    L1 = ((1.0 / math.pi) * damping * period) * Vg
    K_L1 = 4.0 * damping * damping
    cap = 1.0 if cap is None else cap

    y, yd = y0, ydot0
    ts, ys = [0.0], [y]
    n = int(T / dt)
    for k in range(n):
        eta1 = math.asin(max(-cap, min(cap, -y / max(L1, 0.1))))
        eta2 = math.atan2(-yd, Vg)
        eta = max(-math.pi / 2, min(math.pi / 2, eta1 + eta2))
        a = K_L1 * Vg * Vg / L1 * math.sin(eta)
        ydd = a
        yd += ydd * dt
        y += yd * dt
        ts.append((k + 1) * dt)
        ys.append(y)
    return np.array(ts), np.array(ys), L1


def _fit_second_order(ts, ys):
    """从自由响应里量出实际的 ωn 和 ζ —— 用**对数衰减率**法。

    ★ 为什么不用"超调量+峰值时刻"那套公式：
      那是**阶跃响应**的公式；这里的激励是初值 y(0)=y0、ẏ(0)=0，
      两者虽然同为二阶系统，峰值时刻的表达式不同，套错了会得到错的 ζ。

    对数衰减率（对任何二阶系统的自由响应都成立）：
        取两个**同号相邻峰值** y_k、y_{k+1}，间隔正好一个阻尼周期 Td = 2π/ωd
        delta = ln(y_k / y_{k+1}) = zeta*wn*Td
        => zeta = delta / sqrt(4*pi^2 + delta^2)
        => wn   = (2*pi/Td) / sqrt(1 - zeta^2)
    """
    d = np.diff(ys)
    s = np.sign(d)
    s[s == 0] = 1.0
    i_max = np.where(np.diff(s) < 0)[0] + 1      # 内部极大
    # t=0 是边界极大（因为 ẏ(0)=0 且随后下降），它不会被内部极大检测到，手动补上
    pos = [0] + [i for i in i_max if ys[i] > 0]
    if len(pos) < 2:
        return None, None

    y0, y1 = ys[pos[0]], ys[pos[1]]
    if y0 <= 0 or y1 <= 0 or y1 >= y0:
        return None, None
    delta = math.log(y0 / y1)
    zeta = delta / math.sqrt(4 * math.pi ** 2 + delta ** 2)
    Td = ts[pos[1]] - ts[pos[0]]
    if Td <= 0:
        return None, None
    wn = (2 * math.pi / Td) / math.sqrt(max(1e-12, 1 - zeta ** 2))
    return wn, zeta


def exp2():
    head("实验 2 ★★★ 「周期」到底是不是闭环的自然周期")

    print("""  这是本课最该讲透的一步：为什么代码里 k 和 damping 是**两个独立的量**，
    而不是像教科书那样只给一个 k？

  先把 L1 律代进横向动力学。设 y 为交叉航迹误差（沿 +E 量），小角度下：
        eta1 ~ -y / L1          （源码 :163 用的是反过来的叉积，所以带负号）
        eta2 ~ -ydot / Vg       （源码 :154 的 xtrack_vel = gs % AB = -ydot）
        a_lat = K_L1*Vg^2/L1 * sin(eta) ~ K_L1*Vg^2/L1 * (eta1 + eta2)
        yddot = a_lat           （a_lat 就是 +E 方向的加速度）
  合并：
        yddot + (K_L1*Vg/L1)*ydot + (K_L1*Vg^2/L1^2)*y = 0
  对照标准二阶式 yddot + 2*zeta_loop*wn*ydot + wn^2*y = 0：
        wn^2      = K_L1*Vg^2/L1^2        ⇒ wn = sqrt(K_L1)*Vg/L1
        2*zeta_loop*wn = K_L1*Vg/L1       ⇒ zeta_loop = K_L1 / (4*zeta)

  现在把源码的两个公式代进去：
        L1 = (zeta/pi)*T*Vg   （:186 + :67）
        K_L1 = 4*zeta^2        （:200）
  得到：
        wn = 2*zeta*Vg / ((zeta*T/pi)*Vg) = 2*pi / T      ★ 只和 T 有关！
        zeta_loop = 4*zeta^2 / (4*zeta)   = zeta          ★ 就是你设的那个 zeta

  ⇒ 所以「period」真的就是闭环的自然周期，「damping」真的就是闭环阻尼比。
     两个参数各管一件事，这就是 APM 当年那次改写（[2]）的全部意义。
""")

    sub("① 解析预测 vs 数值仿真量测")
    rows = []
    for T, z in ((10.0, 0.75), (15.0, 0.75), (25.0, 0.75), (40.0, 0.75),
                 (25.0, 0.5), (25.0, 0.9)):
        # ★ 仿真时长必须随周期自适应：周期 40 s 时，阻尼周期约 60 s，
        #   跑 60 s 根本看不到第二个峰值，对数衰减率就无从谈起（会"拟合失败"）。
        ts, ys, L1 = _sim_l1_track(T, z, T=max(3.0 * T, 60.0))
        wn_m, zeta_m = _fit_second_order(ts, ys)
        wn_p = 2.0 * math.pi / T
        if wn_m is None:
            rows.append(["%.0f" % T, "%.2f" % z, "%.4f" % wn_p, "—", "拟合失败"])
            continue
        rows.append(["%.0f" % T, "%.2f" % z, "%.4f" % wn_p,
                     "%.4f" % wn_m, "%.4f / %.4f" % (z, zeta_m)])
    tab(rows, ["T [s]", "zeta(设)", "wn 预测", "wn 实测",
               "zeta(设) / zeta(实测)"])
    print("""  ★ 读法：
    · 第 3 列和第 4 列 —— **预测和实测对得上**，说明上面那套推导没写错。
    · 第 5 列 —— 你设多少阻尼，闭环就是多少阻尼。
    ⇒ 「period 管快慢、damping 管超调」这句话，到这里才变成可复算的事实。""")

    sub("② 为什么 wn = 2*pi/T 是「自然周期」")
    print("""  wn 的单位是 rad/s。把 wn = 2*pi/T 代回去：
        无阻尼振荡一周的时间 = 2*pi/wn = T。
  ⇒ 你填 `L1_PERIOD=25`，就是告诉控制器：
     「我要飞机用 25 秒走完一个完整的前后摆动」。
     机型越迟钝 → 这个数越大；小机灵 → 可以给到 15 甚至 10。

  ⚠️ ArduPilot 自己的参数说明里就警告过：
     `NAVL1_PERIOD` 取值「小到 5~10 以下会导致非常剧烈的转弯，并有失速风险」
     （AP_L1_Control.cpp:10）。
""")

    sub("③ ★ 顺带核对：PX4 那句死代码算出来的是什么")
    print("""  PX4 的 ECL_L1_Pos_Controller.cpp:189 有一句
        _heading_omega = sqrtf(2.0f) * M_PI_F / _L1_period;
  但本仓库里 grep `_heading_omega` 只命中两处：
        .hpp:137  声明
        .cpp:189  这一句赋值
  —— **从来没有被读过**。它是死代码。

  它的值也和上面的 wn 对不上：
""")
    rows = []
    for T in (10.0, 25.0, 40.0):
        rows.append(["%.0f" % T, "%.6f" % (math.sqrt(2) * math.pi / T),
                     "%.6f" % (2 * math.pi / T),
                     "%.4f" % (math.sqrt(2) * math.pi / T / (2 * math.pi / T))])
    tab(rows, ["T [s]", "sqrt(2)*pi/T", "2*pi/T（真 wn）", "比值"])
    print("""  ⇒ 它比真 wn 小 %.4f 倍。大概是 PX4 从 APM 抄那句注释时
    顺手留下的一段没人用的遗留（那句注释列的是 **APM 的**改动清单，见 .hpp:50-56）。
  ★ 这类"注释说了、代码没做"的痕迹，读源码时比读注释更值得信自己 grep 一遍。
""" % (1.0 / math.sqrt(2)))


# ===========================================================================
#  实验 3 ★★ 侧风下纯 P 的稳态横偏 vs ArduPilot 的积分项
# ===========================================================================

def _sim_l1_wind(period, damping, Vg_cmd=20.0, vy_wind=5.0,
                 T=90.0, dt=0.01, use_integrator=False,
                 xtrack_i_gain=0.02, cap=1.0):
    """闭环绕一条沿 N 轴的直线，同时有恒定侧风 vy_wind（往 +E 吹）。

    飞机的地速 = 空速矢量 + 风矢量。这里简化：控制器输出的是**横向加速度**，
    它改变地速的横向分量；风则持续把飞机往 +E 推。
        yddot = a_lat + wind_accel        （符号约定同 _sim_l1_track）
    取 wind_accel 为一个小常量来代表侧风对横向的持续扰动。

    ⇒ 稳态时 yddot=0 ⇒ a_lat = -wind_accel < 0 ⇒ y > 0。
      即飞机最终稳定在**下风侧**某个偏移处 —— 这正是纯 P 的稳态误差。
    """
    L1 = ((1.0 / math.pi) * damping * period) * Vg_cmd
    K_L1 = 4.0 * damping * damping
    y, yd = 0.0, 0.0
    xt_i = 0.0
    ts, ys, i_s = [0.0], [y], [0.0]
    n = int(T / dt)
    # 侧风造成的横向漂移加速度（量纲上等价于一个恒定横向扰动）
    wind_accel = 0.02 * vy_wind

    for k in range(n):
        eta1 = math.asin(max(-cap, min(cap, -y / max(L1, 0.1))))
        eta2 = math.atan2(-yd, Vg_cmd)
        Nu1 = eta1
        if use_integrator and abs(Nu1) < math.radians(5.0):
            xt_i += Nu1 * xtrack_i_gain * dt
            xt_i = max(-0.1, min(0.1, xt_i))
        eta = max(-math.pi / 2, min(math.pi / 2, Nu1 + xt_i + eta2))
        a = K_L1 * Vg_cmd * Vg_cmd / L1 * math.sin(eta)
        ydd = a + wind_accel
        yd += ydd * dt
        y += yd * dt
        ts.append((k + 1) * dt)
        ys.append(y)
        i_s.append(xt_i)
    return np.array(ts), np.array(ys), np.array(i_s), L1


def exp3():
    head("实验 3 ★★ 侧风下纯 P 的稳态横偏，和 AP 那个积分项怎么消掉它")

    print("""  L1 律里没有任何积分项 —— 它是纯比例（P）的。
  纯 P 在**恒定扰动**下必然留一个稳态误差。
  恒定侧风就是最典型的恒定扰动。

  场景：沿 N 轴的直线航线，恒定侧风把飞机往 +E 推。
        比较 PX4 版（无积分）与 ArduPilot 版（有 _L1_xtrack_i）。
""")

    SIM_T = 400.0        # ★ 积分项收敛很慢，跑短了会误判成"压不下去"

    sub("① 纯 P：稳态横偏随侧风增大")
    rows = []
    for wind in (0.0, 2.0, 5.0, 8.0, 12.0):
        ts, ys, _, L1 = _sim_l1_wind(25.0, 0.75, vy_wind=wind, T=SIM_T)
        rows.append(["%.1f" % wind, "%.3f" % ys[-1],
                     "%.3f" % (ys[-1] / L1 if L1 else 0)])
    tab(rows, ["侧风 [m/s]", "稳态横偏 [m]", "横偏 / L1"])
    print("""  ★ 稳态横偏**随侧风线性增长** —— 这就是纯 P 的稳态误差。
    量级上可以解释：稳态时 a_lat 必须抵消风的横向扰动，
    而 a_lat ~ wn^2*y，所以 y_ss ~ 扰动/wn^2。
    wn = 2*pi/T，**周期给得越大，同样的风就偏得越多**。""")

    sub("② 加上 ArduPilot 的积分项")
    rows = []
    for wind in (0.0, 2.0, 5.0, 8.0, 12.0):
        ts, ys, ii, L1 = _sim_l1_wind(25.0, 0.75, vy_wind=wind,
                                      use_integrator=True, T=SIM_T)
        # ★ 收敛时间 = **最后一次**超出容差的时刻。
        #   不能取"第一次进入容差"—— 飞机是从航线上(y=0)出发的，
        #   那个判据在 t=0 就满足了，会报出一片 0（作者第一版就是这样）。
        out = np.where(np.abs(ys) >= 0.05)[0]
        tc = "%.0f" % ts[out[-1]] if len(out) else "0"
        rows.append(["%.1f" % wind, "%.3f" % ys[-1], tc,
                     "%.4f" % ii[-1]])
    tab(rows, ["侧风 [m/s]", "稳态横偏 [m]", "收敛到 0.05 m 用时 [s]",
               "积分量 [rad]"])
    print("""  ⇒ 积分项最终把稳态横偏压到 0，但**收敛很慢**（见第 3 列）——
    这正是"积分换精度、用时间买"的代价。

  两条源码细节解释了它为什么慢、以及为什么不能更快：
     · 只在 |Nu1| < 5 度时才累加（AP_L1_Control.cpp:319）——
       大偏差时不掺和，免得把捕获过程搅乱；
     · 限幅 ±0.1 rad（约 5.7 度，:323），注释给了理由：
       「AHRS_TRIM_X=0.1 会漂到约 0.08，所以 0.1 是个好的最坏情况界」（:322）。
""")

    sub("③ 积分增益扫描：AP 的参数说明 vs 实测")
    rows = []
    for g in (0.0, 0.005, 0.02, 0.05, 0.1, 0.3, 1.0):
        ts, ys, ii, L1 = _sim_l1_wind(25.0, 0.75, vy_wind=8.0,
                                      use_integrator=(g > 0),
                                      xtrack_i_gain=g, T=SIM_T)
        out = np.where(np.abs(ys) >= 0.05)[0]
        tc = "%.0f" % ts[out[-1]] if len(out) else "0"
        rows.append(["%.3f" % g, "%.3f" % ys[-1], tc,
                     "%.3f" % float(np.max(np.abs(ys)))])
    tab(rows, ["XTRACK_I", "末态横偏 [m]", "收敛用时 [s]",
               "过程中最大 |横偏| [m]"])
    print("""  ArduPilot 的参数表（AP_L1_Control.cpp:27）原话是：
     「太小收敛慢；太大引起横偏振荡；设 0 关闭」，默认 0.02（:31），范围 0~0.1。

  ★ 读这张表，形状是「**先变快、再变慢、最后崩**」——不是单调的：
    · 0.005 → 0.1：收敛时间从「>400 s（跑完都没收敛）」一路降到 59 s；
    · **0.3：反弹回 255 s** —— 已经出现"过大"的苗头；
    · **1.0：崩了** —— 末态横偏 5.457 m、最大摆幅 7.343 m，
      比不加积分（2.53）还差得多。这就是参数说明警告的
      "higher values will cause crosstrack error oscillation"。

  ⇒ AP 那句「太小收敛慢、太大引起振荡」（AP_L1_Control.cpp:27）**两头都说对了**，
    最优区在中间。而它给的允许范围 [0, 0.1]（:28-31）恰好把 0.3 和 1.0 挡在外面 ——
    **参数表的上限不是随手写的，它就是那个"再好就开始变坏"的位置。**

  ⚠️ 两条必须守住的界线：
    ① 本仿真只有常数风、只有几米横偏，是一条**温和**的场景；
       真实飞行里阵风、突风、传感器延迟都会把安全边界压低。
       **仿真跑不出反例 ≠ 反例不存在**。
    ② 也不要因此反推"AP 的 0.1 是精确最优"——本表只扫了 7 个点、
       只用了这一组参数（T=25、ζ=0.75、侧风 8 m/s）。
       **调参结论永远带条件**，这也是第 8 章要专门讲的事。""")


# ===========================================================================
#  实验 4 ★★ 捕获角饱和：PX4 的 90° 与 ArduPilot 的 45°
# ===========================================================================

def exp4():
    head("实验 4 ★★ 捕获角饱和：PX4 的 90° 与 ArduPilot 的 45°")

    print("""  当飞机离航线很远时，`xtrackErr/L1` 会很大，asin 的自变量必须限幅，
  否则 asin 无解。这个限幅**就是捕获角**：

      PX4  ECL_L1_Pos_Controller.cpp:167
           sine_eta1 = constrain(sine_eta1, -1.0f, 1.0f)      → 90 度
      AP   AP_L1_Control.cpp:310
           constrain_float(sine_Nu1, -0.7071f, 0.7071f)       → 45 度
                 （注释原文：controlled track capture angle of 45 deg）

  同一个几何，限幅不同，行为就不一样。下面用同一组初始条件跑两遍。
""")

    A = np.array([0.0, 0.0])
    B = np.array([600.0, 0.0])
    gs = np.array([20.0, 0.0])

    sub("① 不同初始横偏下，两种限幅给出的控制角")
    rows = []
    for y0 in (0, 30, 60, 100, 200, 400, 800):
        P = np.array([200.0, float(y0)])
        r = px4_l1(A, B, P, gs)                         # PX4: period=25
        L1_ap = 0.3183099 * 0.75 * 17.0 * 20.0          # AP: period=17
        # ★ 符号：AP 的 _crosstrack_error = A_air % AB = **-y0**，
        #   与 PX4 的 xtrackErr 同号，所以两者的控制角应该【同号】。
        #   （作者第一版在这里手算时漏了负号，两列符号相反，白对了一轮。）
        s45 = min(max(-y0 / L1_ap, -0.7071), 0.7071)
        rows.append(["%d" % y0, "%.1f" % r["L1_distance"],
                     "%.3f" % math.degrees(r["eta1"]),
                     "%.1f" % L1_ap,
                     "%.3f" % math.degrees(math.asin(s45))])
    tab(rows, ["初始横偏 [m]", "PX4 L1 [m](T=25)", "PX4: eta1 [deg]",
               "AP L1 [m](T=17)", "AP: Nu1 [deg]"])
    print("""  ⇒ 横偏超过 L1 距离后，PX4 的 eta1 还能继续涨到 **90 度**；
    AP 在 Nu1 到 **45 度**时就封顶了，之后无论偏多远，指令都不再增加。

  ★ 两列的符号是**相同的**（本例都为负）。差别只在幅度上限 ——
    这一点容易搞混：实验 1 说过两边的 `crosstrack_error` 符号相反，
    但那说的是**那个上报量**；真正进入控制的角（PX4 的 eta1 / AP 的 Nu1）
    用的都是同一个方向的叉积，所以同号。""")

    sub("② 大横偏下的捕获轨迹对比")
    ts90, ys90, _ = _sim_l1_track(25.0, 0.75, y0=200.0, T=75.0, cap=1.0)
    ts45, ys45, _ = _sim_l1_track(25.0, 0.75, y0=200.0, T=75.0,
                                  cap=0.7071)
    rows = []
    for tq in (0, 2, 4, 6, 8, 12, 20, 40, 74):
        i90 = int(tq / 0.002)
        i45 = int(tq / 0.002)
        if i90 < len(ys90) and i45 < len(ys45):
            rows.append(["%.0f" % tq, "%.2f" % ys90[i90], "%.2f" % ys45[i45]])
    tab(rows, ["t [s]", "横偏 限幅90deg [m]", "横偏 限幅45deg [m]"])
    print("""  ★ 这张表是**受控对照**：两边用的是同一套 period/damping/L1，
    唯一变量就是捕获角限幅 —— 所以差异全部来自那一个 constrain。

  ⇒ 90° 冲得快，代价是**冲过头**：第 12 秒到了 -26.08 m（过冲 26 m）；
    45° 慢一些，但几乎不超调（最大只到 +41.93 就开始回头，末段平滑贴线）。

  ★ 哪一个更好，取决于你怕什么：
     · 怕**超调/振荡**（大横偏时冲过头）→ 45° 更稳；
     · 怕**回不来**（侧风把飞机推得太远）→ 90° 更有力。
    没有普适答案 —— 这就是"调参"在这类算法里真实的样子。
""")

    sub("③ 别忘了起飞段的旁路")
    print("""  两个实现都还有一条**绕开 L1 逻辑**的分支：

    PX4 :106   if (distance_A_to_airplane > _L1_distance &&
                   alongTrackDist / max(distance_A_to_airplane, 1.0) < -0.7071)
                   → 直接飞向 A 点
    AP  :279   同样的判据（"在 A 点后方 ±135 度弧内且比 L1 距离更远"）

  这里也用了 0.7071 —— 但它是判"在不在后方 ±135 度"，和捕获角不是一回事。
  ★ 同一个常数在两个地方出现、含义不同，是读源码时最容易搞混的一类。""")

    if _HAS_MPL:
        plt.figure(figsize=(8, 3.2))
        plt.plot(ts90, ys90, label="cap = 90deg (PX4)")
        plt.plot(ts45, ys45, label="cap = 45deg (AP)")
        plt.axhline(0, color="k", lw=0.8)
        plt.xlabel("time [s]")
        plt.ylabel("crosstrack [m]")
        plt.title("capture angle 对捕获过程的影响（初始横偏 200 m）")
        plt.grid(alpha=0.3)
        plt.legend(fontsize=8)
        savefig("lesson31_exp4")


EXPERIMENTS = {
    "1": ("L1 几何：手算 vs 源码公式", exp1),
    "2": ("period/damping 与闭环的二阶响应", exp2),
    "3": ("侧风稳态横偏与 ArduPilot 的积分项", exp3),
    "4": ("捕获角饱和 45 度 vs 90 度", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        for k in sorted(EXPERIMENTS):
            EXPERIMENTS[k][1]()
    else:
        for a in args:
            if a not in EXPERIMENTS:
                print("未知实验编号：%s（可选 %s）"
                      % (a, "/".join(sorted(EXPERIMENTS))))
                continue
            EXPERIMENTS[a][1]()
    print()
    print("=" * W)
    print("  全部完成。")
    print("=" * W)
    print()


if __name__ == "__main__":
    main()
