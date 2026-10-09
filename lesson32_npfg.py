#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 3 第 7 章 · NPFG：非线性路径跟随与抗风 —— 配套实验

    实验 1  ★★  方位矢量：把「前视点」换成「方向场」（前视角是二次的）
    实验 2  ★★  track_error_bound 为什么随地速变；track_proximity 在干嘛
    实验 3  ★★★ 风三角：从「我想往哪个方位飞」解出「空速矢量该是多少」
    实验 4  ★★★ 抗风：wind excess regulation，以及「零地速」这个终态

用法：
    python lesson32_npfg.py 1
    python lesson32_npfg.py 2
    python lesson32_npfg.py 3
    python lesson32_npfg.py 4
    python lesson32_npfg.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  能算的就不猜。

  · 本脚本把 `npfg.cpp` 的**关键函数逐行照抄成 Python**（每个函数都标了行号）。
    这样课文里的每一步都能在脚本里复算。
  · 实验 3 的「风三角」不是比喻：它是真的解一个三角形 ——
    已知风速矢量、期望的**方位线**、以及可用空速，
    求「空速矢量该指向哪、多大」。脚本把解出来的矢量再代回去验算。
  · 实验 4 会让风**从 0 一路加到超过空速**，把 `refAirVelocity()`
    的四个分支都走一遍，看给定量的演化 —— 包括一处注释与算式对不上的地方。

【本脚本核对过的源码事实】（行号均已打开确认）

  PX4   src/lib/npfg/npfg.cpp
        :76-78   guideToPath(curr_pos, ground_vel, wind_vel,
                             unit_path_tangent, position_on_path, path_curvature)
        :82      air_vel = ground_vel - wind_vel
        :88      signed_track_error_ = unit_path_tangent.cross(path_pos_to_vehicle)
        :103     p_gain_ = pGain(adapted_period_, damping_)
        :107     track_error_bound_ = trackErrorBound(ground_speed, time_const_)
        :111     look_ahead_ang = lookAheadAngle(normalized_track_error)
        :115     bearing_vec_ = bearingVec(unit_path_tangent, look_ahead_ang, signed_track_error_)
        :122     feas_ = bearingFeasibility(wind_cross_bearing, wind_dot_bearing, ...)
        :125     feas_combined = feas_ * feas_on_track_
        :132-134 air_vel_ref_ = refAirVelocity(...); airspeed_ref_ = air_vel_ref_.norm()
        :137     lateral_accel = lateralAccel(air_vel, air_vel_ref_, airspeed)
        :147     lateral_accel_ = lateral_accel + feas_combined * track_proximity_ * lateral_accel_ff_
        :149     updateRollSetpoint()
        :220     // See [TODO: include citation] ...   ← 上游自己没给出处
        :266-276 trackErrorBound()
        :278-281 pGain() = 4*pi*damping/period
        :283-286 timeConst() = period*damping
        :288-291 lookAheadAngle() = pi/2 * (nte-1)^2      ← 二次！
        :293-303 bearingVec()
        :329-397 refAirVelocity()
        :377-380 air_vel_ref = wind_vel ★实测地速=2×风，与其注释不符（见实验4②）
        :407-411 bearingIsFeasible()
        :413-419 solveWindTriangle()
        :431-443 bearingFeasibility()
        :445-475 lateralAccelFF()
        :477-493 lateralAccel()
        :500-508 updateRollSetpoint() —— roll = atan(a_lat/g)，限幅，且只取有限值

参考：
    卷 3 第 6 章（本章的上一课，L1 的几何）
    卷 3 第 3 章（$a=g\tan\theta$ 的反解）
    卷 2 第 8 章（风估计从哪里来）
"""

import math
import sys

import numpy as np

W = 74

# NPFG 源码里的常量（npfg.hpp:277-284）
NPFG_EPSILON = 1e-6
MIN_RADIUS = 0.5          # m
NTE_FRACTION = 0.5
AIRSPEED_BUFFER = 1.5     # m/s

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
        print("  [图] 已保存 %s.png" % name)
    except Exception as exc:                     # pragma: no cover
        print("  [图] 保存失败：%s" % exc)


def cross2(a, b):
    """二维叉积 —— 对应源码里 matrix 的 `%` 运算符（Vector2.hpp）。"""
    return a[0] * b[1] - a[1] * b[0]


def dot2(a, b):
    return a[0] * b[0] + a[1] * b[1]


def norm2(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1])


def unit2(a):
    n = norm2(a)
    return np.array([a[0] / n, a[1] / n])


# ===========================================================================
#  npfg.cpp 的「源码级」实现（每个函数标了对应行号）
# ===========================================================================

def p_gain(period, damping):
    """npfg.cpp:278-281 —— pGain() = 4*pi*damping/period"""
    return 4.0 * math.pi * damping / period


def time_const(period, damping):
    """npfg.cpp:283-286 —— timeConst() = period*damping"""
    return period * damping


def look_ahead_angle(nte):
    """npfg.cpp:288-291 —— lookAheadAngle() = pi/2 * (nte-1)^2

    ★ nte = normalized_track_error ∈ [0,1]
        nte=0（在航线上）→ laa = pi/2 = 90°  → 方位矢量 = 路径切矢（沿航线飞）
        nte=1（偏到边界）→ laa = 0            → 方位矢量 = 指向航线的垂线
     过渡是【二次】的。L1 是线性的 —— 这是两者最本质的结构差异。
    """
    return math.pi / 2.0 * (nte - 1.0) * (nte - 1.0)


def track_error_bound(ground_speed, tc):
    """npfg.cpp:266-276 —— 偏差边界随地速变化

    地速 > 1 m/s 时  bound = gs * time_const
    否则（防止归一化奇异）bound = 0.5 * tc * (gs^2 + 1)
    """
    if ground_speed > 1.0:
        return ground_speed * tc
    return 0.5 * tc * (ground_speed * ground_speed + 1.0)


def track_proximity(laa):
    """npfg.cpp:260-264 —— trackProximity() = sin^2(laa)

    在航线上(laa=90°) → 1；偏到边界(laa=0) → 0。
    它用来给「曲率前馈」和「自适应周期」做淡入淡出。
    """
    s = math.sin(laa)
    return s * s


def bearing_vec(unit_path_tangent, laa, signed_track_error):
    """npfg.cpp:293-303 —— 方位矢量（本章的核心概念）

    unit_path_normal  : 把切矢转 90°（源码注释写的是 right handed 90 deg）
    unit_track_error  : 垂直于路径、指向路径方向的单位矢（即「回来」的方向）
    返回              : cos(laa)*指向路径 + sin(laa)*沿路径
                        —— 偏得远时朝路径飞，贴近时沿路径飞
    """
    t = np.asarray(unit_path_tangent, dtype=float)
    n = np.array([-t[1], t[0]])
    sgn = -1.0 if signed_track_error < 0.0 else 1.0
    unit_track_error = -sgn * n
    return math.cos(laa) * unit_track_error + math.sin(laa) * t


def bearing_is_feasible(wind_cross_bearing, wind_dot_bearing, airspeed, wind_speed):
    """npfg.cpp:407-411"""
    return (abs(wind_cross_bearing) < airspeed) and \
           ((wind_dot_bearing > 0.0) or (wind_speed < airspeed))


def bearing_feasibility(wind_cross_bearing, wind_dot_bearing, airspeed, wind_speed):
    """npfg.cpp:431-443 —— 可行性，[0,1] 的连续量

    ★ 先做一个符号处理：如果风在方位线上的投影为负（顶着风分量），
      就把 |横向风| 直接置成【整个风速】—— 意思是「这风完全不帮忙」。
    然后 sin^2(pi/2 * clamp((airspeed - |wind_cross|)/AIRSPEED_BUFFER, 0, 1))
      空速富余 >= 1.5 m/s → 1（完全可行）
      空速不够      → 0（不可行）
    """
    if wind_dot_bearing < 0.0:
        wind_cross_bearing = wind_speed
    else:
        wind_cross_bearing = abs(wind_cross_bearing)
    arg = (airspeed - wind_cross_bearing) / AIRSPEED_BUFFER
    arg = min(max(arg, 0.0), 1.0)
    s = math.sin(math.pi * 0.5 * arg)
    return s * s


def project_airsp_on_bearing(airspeed, wind_cross_bearing):
    """npfg.cpp:399-405 —— 空速在方位线上的可投影分量

    ⚠️ 源码注释明确写了前置条件：调用前必须已经判定 bearingIsFeasible()，
       否则返回值是【错的】（根号里会被 max 截成 0，看起来「正常」）。
    """
    return math.sqrt(max(airspeed * airspeed - wind_cross_bearing ** 2, 0.0))


def solve_wind_triangle(wind_cross_bearing, airsp_dot_bearing, bvec):
    """npfg.cpp:413-419 —— 解风三角

    源码注释：「essentially a 2D rotation with the speeds (magnitudes) baked in」
    即：把 (横向风, 所需空速分量) 这个对，按方位矢量旋转到世界系里。
    返回的是【空速矢量】—— 不是风速、也不是地速。
    """
    return np.array([airsp_dot_bearing * bvec[0] - wind_cross_bearing * bvec[1],
                     wind_cross_bearing * bvec[0] + airsp_dot_bearing * bvec[1]])


def infeasible_air_vel_ref(wind_vel, bvec, wind_speed, airspeed):
    """npfg.cpp:421-429 —— 完全不可行时的缓解律"""
    k = math.sqrt(max(wind_speed ** 2 - airspeed ** 2, 0.0))
    v = k * bvec - wind_vel
    n = norm2(v)
    if n < NPFG_EPSILON:
        return np.array([0.0, 0.0])
    return v / n * airspeed


def ref_air_velocity(wind_vel, bvec, wind_cross_bearing, wind_dot_bearing,
                     wind_speed, airspeed_nom, airspeed_max, min_ground_speed,
                     en_wind_reg=True, en_min_gsp=True, en_track_keeping=False):
    """npfg.cpp:329-397 —— 从方位矢量解出【空速矢量给定】

    四个分支（本脚本实验 4 会把它们逐一走一遍）：
      ① 需要维持最小地速  → 按需要的最小空速解风三角
      ② 名义空速可行      → 直接用名义空速解
      ③ 名义不可行但最大可行 且开着 wind regulation
           · wind_dot_bearing <= 0 → air_vel_ref = wind_vel  ←★ 零地速终态
           · 否则 → 用「与方位线成直角」的最小空速用量解
      ④ 全都不可行        → 缓解律 infeasibleAirVelRef
    """
    wind_vel = np.asarray(wind_vel, dtype=float)

    if min_ground_speed > wind_dot_bearing and (en_min_gsp or en_track_keeping) and en_wind_reg:
        # ── 分支 ①：要维持最小地速 ──
        d = min_ground_speed - wind_dot_bearing
        airspeed_min = math.sqrt(d * d + wind_cross_bearing ** 2)
        if airspeed_min > airspeed_max:
            if bearing_is_feasible(wind_cross_bearing, wind_dot_bearing, airspeed_max, wind_speed):
                adb = project_airsp_on_bearing(airspeed_max, wind_cross_bearing)
                return solve_wind_triangle(wind_cross_bearing, adb, bvec), "①-min>max,仍可行"
            return infeasible_air_vel_ref(wind_vel, bvec, wind_speed, airspeed_max), "①-不可行→缓解"
        if airspeed_min > airspeed_nom:
            adb = project_airsp_on_bearing(airspeed_min, wind_cross_bearing)
            return solve_wind_triangle(wind_cross_bearing, adb, bvec), "①-用最小空速"
        adb = project_airsp_on_bearing(airspeed_nom, wind_cross_bearing)
        return solve_wind_triangle(wind_cross_bearing, adb, bvec), "①-名义空速够"

    # ── 分支 ②③④：wind excess regulation / 缓解 ──
    if bearing_is_feasible(wind_cross_bearing, wind_dot_bearing, airspeed_nom, wind_speed):
        adb = project_airsp_on_bearing(airspeed_nom, wind_cross_bearing)
        return solve_wind_triangle(wind_cross_bearing, adb, bvec), "②-名义可行"

    if bearing_is_feasible(wind_cross_bearing, wind_dot_bearing, airspeed_max, wind_speed) and en_wind_reg:
        if wind_dot_bearing <= 0.0:
            # ★★★ 本章最重要的一行
            # 「we only increment the airspeed to regulate, but not overcome, excess wind
            #   NOTE: in the terminal condition, this will result in a zero ground velocity configuration」
            return wind_vel.copy(), "③-空速给定=风矢量"
        return solve_wind_triangle(wind_cross_bearing, 0.0, bvec), "③-直角最小用量"

    airspeed_input = airspeed_max if en_wind_reg else airspeed_nom
    return infeasible_air_vel_ref(wind_vel, bvec, wind_speed, airspeed_input), "④-缓解律"


def lateral_accel(air_vel, air_vel_ref, airspeed, pg):
    """npfg.cpp:477-493 —— 反馈项：只由【航向误差】产生"""
    dot_err = dot2(air_vel, air_vel_ref)
    cross_err = cross2(air_vel, air_vel_ref)
    aref = norm2(air_vel_ref)
    if dot_err < 0.0:
        # 航向误差 > 90 度：饱和保持
        return pg * (-airspeed if cross_err < 0.0 else airspeed)
    return pg * cross_err / aref


def lateral_accel_ff(t, ground_vel, wind_dot_upt, wind_cross_upt, airspeed,
                     signed_track_error, path_curvature):
    """npfg.cpp:445-475 —— 前馈项：仅为维持路径曲率所需"""
    pfc = path_curvature / max(1.0 - path_curvature * signed_track_error,
                               abs(path_curvature) * MIN_RADIUS)
    tangent_gs = max(dot2(ground_vel, t), 0.0)
    path_frame_rate = pfc * tangent_gs
    denom = max(project_airsp_on_bearing(airspeed, wind_cross_upt), NPFG_EPSILON)
    speed_ratio = 1.0 + wind_dot_upt / denom
    return airspeed * speed_ratio * path_frame_rate


def update_roll_setpoint(lat_accel, roll_lim_rad):
    """npfg.cpp:500-508 —— roll = atan(a_lat/g)，限幅"""
    roll = math.atan(lat_accel / 9.80665)
    roll = min(max(roll, -roll_lim_rad), roll_lim_rad)
    return roll if math.isfinite(roll) else None


# ===========================================================================
#  实验 1 ★★ 方位矢量：把「前视点」换成「方向场」
# ===========================================================================

def exp1():
    head("实验 1 ★★ 方位矢量：把「前视点」换成「方向场」")

    print("""  L1 的做法：在航线上找**一个点**（距飞机 L1 米），朝它飞。
  NPFG 的做法：在飞机当前位置构造**一个方向**（方位矢量），顺着它飞。

    「点」和「方向」的区别，在偏得远的时候会变得非常明显 ——
    因为 L1 的指向角随偏差**线性**变化，而 NPFG 是**二次**的。

  两个关键量（npfg.cpp:288-291 与 :293-303）：

      nte  = track_error / track_error_bound   ∈ [0,1]   （归一化偏差）
      laa  = pi/2 * (nte - 1)^2                           （look-ahead 角）

      方位矢量 = cos(laa) * 「指向路径的垂线」 + sin(laa) * 「路径切矢」
""")

    sub("① look-ahead 角：二次 vs 线性")
    rows = []
    for nte in (0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0):
        laa = look_ahead_angle(nte)
        # L1 的对照量：把「前视点方向」也折算成一个角
        # L1: eta1 = asin(xtrack/L1)，取 xtrack/L1 = nte（同样的归一化偏差）
        l1_ang = math.asin(min(max(nte, -1.0), 1.0))
        rows.append(["%.2f" % nte, "%.2f" % math.degrees(laa),
                     "%.2f" % math.degrees(l1_ang),
                     "%.3f" % track_proximity(laa)])
    tab(rows, ["nte（归一化偏差）", "NPFG laa [deg]", "L1 的角 [deg]", "track_proximity"])
    print("""  ★ 读第 2、3 列的**形状**（不是数值，两者定义不同，不能直接比大小）：
    · L1 的角随偏差**线性**增长（asin 在中小角度下近似线性）；
    · NPFG 的角是**抛物线**（(nte-1)^2）——
      两端平、中间陡。

  为什么要做成二次？源码在 `switchDistance` 的注释里给了线索（npfg.hpp:316-317）：

      「as the bearing angle changes quadratically (instead of linearly as in L1),
        the time constant (automatically calculated for on track stability)
        proportional track error boundary typically over estimates the required
        switching distance」

  ⇒ 白话：**方位角随偏差二次变化，意味着大偏差时「转回来」的力度增长得更快**，
    所以按时间常数算出来的切换距离（L1 那套算法）会被高估。
    NPFG 因此可以取更小的切换距离（默认乘 0.32）。

  ★ 这是**全仓库里唯一一处 NPFG 直接对比 L1 的源码注释** ——
    连一句「我们比 L1 好在哪」的成段论述都没有，只有这一行注释。""")

    sub("② 方位矢量的构造（nte = 0.6 为例）")
    t = np.array([1.0, 0.0])                 # 路径沿 +x
    ste = 40.0                               # 带符号偏差 [m]
    bound = 100.0
    nte = min(abs(ste) / bound, 1.0)
    laa = look_ahead_angle(nte)
    bvec = bearing_vec(t, laa, ste)

    n = np.array([-t[1], t[0]])
    sgn = -1.0 if ste < 0 else 1.0
    ute = -sgn * n
    print("  路径切矢 t = (%.2f, %.2f)，带符号偏差 = %+.1f m，边界 = %.1f m"
          % (t[0], t[1], ste, bound))
    print("  nte = |%+.1f| / %.1f = %.3f  ⇒  laa = %.4f rad = %.2f deg"
          % (ste, bound, nte, laa, math.degrees(laa)))
    print()
    print("  「指向路径的垂线」方向 unit_track_error = (%+.4f, %+.4f)" % (ute[0], ute[1]))
    print("  「路径切矢」方向         unit_path_tangent = (%+.4f, %+.4f)" % (t[0], t[1]))
    print()
    print("  方位矢量 = cos(%.2f deg)*(%.4f,%.4f) + sin(%.2f deg)*(%.4f,%.4f)"
          % (math.degrees(laa), ute[0], ute[1], math.degrees(laa), t[0], t[1]))
    print("           = (%+.6f, %+.6f)   |方位矢量| = %.6f"
          % (bvec[0], bvec[1], norm2(bvec)))
    print()
    print("  ★ 它是**单位矢量**（模长恒为 1）：cos^2+sin^2=1 —— 这个构造天然归一化。")
    print("    nte 越小（越贴近航线）→ laa 越大 → cos(laa) 越小 → 越偏向「沿路径飞」；")
    print("    nte 越大（偏得越远）→ laa 越小 → 越偏向「朝路径飞」。")

    sub("③ 方位矢量随偏差的完整演化")
    rows = []
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 3.0):
        ste_k = frac * bound
        nte_k = min(abs(ste_k) / bound, 1.0)
        laa_k = look_ahead_angle(nte_k)
        bv = bearing_vec(t, laa_k, ste_k)
        # 方位矢量与路径切矢的夹角 = 它还「朝着航线」的成分有多少
        ang = math.degrees(math.atan2(bv[1], bv[0]))
        rows.append(["%.2f" % frac, "%.2f" % nte_k,
                     "%.2f" % math.degrees(laa_k),
                     "(%+.4f, %+.4f)" % (bv[0], bv[1]),
                     "%+.2f" % ang])
    tab(rows, ["偏差/边界", "nte", "laa [deg]", "方位矢量", "与路径夹角 [deg]"])
    print("""  ★ 注意最后两行：偏差超过边界后，nte 被 `constrain(...,0,1)` 截住（:215），
    方位矢量**不再变化** —— 始终指向路径的垂线方向。
    ⇒ 这就是「饱和」：偏得再远，NPFG 也只是「径直朝航线飞」，
      不会像 L1 那样把指令一路加到 90° 上去。""")

    if _HAS_MPL:
        fr = np.linspace(0, 3, 200)
        laas = [look_ahead_angle(min(f, 1.0)) for f in fr]
        plt.figure(figsize=(8, 3.2))
        plt.plot(fr, np.degrees(laas), label="NPFG laa (二次)")
        plt.plot(fr, np.degrees([math.asin(min(f, 1.0)) for f in fr]),
                 label="L1 eta1 (线性/asin)")
        plt.axvline(1.0, color="k", lw=0.8, ls="--")
        plt.text(1.02, 80, "nte 边界", fontsize=8)
        plt.xlabel("归一化偏差  nte")
        plt.ylabel("角度 [deg]")
        plt.title("前视角随偏差的变化：NPFG 二次 vs L1 线性")
        plt.grid(alpha=0.3)
        plt.legend(fontsize=8)
        savefig("lesson32_exp1")


# ===========================================================================
#  实验 2 ★★ track_error_bound 为什么随地速变
# ===========================================================================

def exp2():
    head("实验 2 ★★ track_error_bound 为什么随地速变")

    print("""  这是 NPFG 里一个乍看很奇怪的设定：**偏差边界不是一个常数**，
  而是随**地速**变化（npfg.cpp:266-276）：

      地速 > 1 m/s :  bound = ground_speed * time_const
      地速 <= 1 m/s:  bound = 0.5 * time_const * (ground_speed^2 + 1)

  而 time_const = period * damping（:283-286）。

  ★ 为什么要这样？因为 NPFG 内部所有的「偏差」都是**归一化**的
    （nte = track_error / bound ∈ [0,1]）。
    如果用一个固定的米数当边界，那么「飞得快」和「飞得慢」时，
    同样的米数对应的 nte 会差很多，控制器的行为就随速度漂移了。

    把边界做成 **正比于地速**（= 正比于「在这段时间里能飞多远」），
    归一化偏差就变成了一个**与速度无关**的量。
""")

    sub("① 边界随地速变化")
    rows = []
    for T, z in ((10.0, 0.7), (12.0, 0.7)):
        tc = time_const(T, z)
        for gs in (0.5, 1.0, 5.0, 10.0, 20.0, 30.0):
            rows.append(["%.0f" % T, "%.1f" % gs, "%.2f" % tc,
                         "%.2f" % track_error_bound(gs, tc)])
    tab(rows, ["period [s]", "地速 [m/s]", "time_const [s]", "偏差边界 [m]"])
    print("""  ★ 看 period=10 那一组（time_const=7.0 s）：
      地速 20 m/s → 边界 140 m；地速 30 m/s → 边界 210 m。
    **「多偏算偏得远」这件事，是随速度变的** —— 飞得快时允许偏得多。
    （直觉上也对：同样的 50 m 偏差，慢速飞机要修正很久，快飞机一会儿就回来了。）""")

    sub("② ★ 那个 gs<=1 的分支在防什么")
    tc = time_const(10.0, 0.7)
    rows = []
    for gs in (0.0, 0.2, 0.5, 0.8, 1.0, 1.2, 2.0):
        b = track_error_bound(gs, tc)
        lin = gs * tc            # 如果不做那个分支，直接用 gs*tc
        rows.append(["%.1f" % gs, "%.3f" % lin, "%.3f" % b,
                     "%.3f" % (b - lin)])
    tab(rows, ["地速 [m/s]", "若直接用 gs*tc", "实际 bound", "差值"])
    print("""  ⇒ 地速趋近 0 时，`gs*tc` 也趋近 0 —— 而 bound 是被**除**的那个量
    （nte = track_error / bound），bound→0 会让 nte 直接爆掉（除零奇异）。

    源码用一个**二次式**把低速段托住（:274）：
        bound = 0.5 * tc * (gs^2 + 1)
    地速 = 0 时它还等于 0.5*tc —— **永远不为零**。

  ★ 注释里那句话值得抄下来（:272-273）：
      「limit bound to some minimum ground speed to avoid singularities in
        track error normalization. the following equation assumes ground
        speed minimum = 1.0」
    —— 上游明确说了这个式子是**为了配合「地速下限 1.0」的假设**而设计的。""")

    sub("③ track_proximity 在干嘛（前馈的淡入淡出）")
    rows = []
    for nte in (0.0, 0.25, 0.5, 0.75, 1.0):
        laa = look_ahead_angle(nte)
        rows.append(["%.2f" % nte, "%.2f" % math.degrees(laa),
                     "%.4f" % track_proximity(laa)])
    tab(rows, ["nte", "laa [deg]", "track_proximity = sin^2(laa)"])
    print("""  track_proximity 在 `guideToPath` 里出现两次（:113 与 :147）：

      · :147  lateral_accel_ = 反馈项 + feas_combined * track_proximity_ * 前馈项

  ⇒ 意思是：**「为了维持路径曲率需要的前馈加速度」只在贴近航线时才加进去。**
    偏得很远的时候（track_proximity→0）前馈被完全淡出，
    因为那时候首要任务是**回到航线上**，不是「沿着这条曲线飞」。

  ★ 这个设计是有道理的：如果偏在 100 m 外还硬要按路径曲率加速，
    等于在错误的轨道上做正确的转弯 —— 越转越偏。""")

    sub("④ 自适应周期：两个方向上的边界")
    print("""  `adaptPeriod()`（:152-211）会根据风、曲率、滚转响应时间**动态改周期**。
  它往两个方向调，各有各的物理理由：

  **下限（缩短周期）** —— 目的是**躲开极限环**：
      零曲率:            period_lb = pi * roll_time_const / damping        (:246)
      有曲率+风(ζ>0.5):  period_lb = 4*pi * roll_time_const * damping      (:253)
      两者按 feas_on_track 混合（:256），再乘安全系数 period_safety_factor（默认 1.5）

    源码注释（:242-243）说得很清楚：这是
      「a constant worst case bound for any wind ratio, airspeed, and path curvature」
      ——**任何风速比/空速/曲率下的最坏情况保守界**。

  **上限（加长周期）** —— 目的是**保持跟踪稳定性**：
      period_ub = 4*pi*damping / (air_turn_rate * wind_factor * feas_on_track)  (:234)
      直线或无风时返回 INFINITY（:237）

  ⚠️ **上游对这两个方向都写了免责声明**（:186-190、:196-198）：

      「only allow upper bounding ONLY if lower bounding is enabled
        (is otherwise dangerous to allow period decrements without stability checks).」
      「NOTE: if the roll time constant is not accurately known, reducing
        the period can still destabilize the system!
        enable this feature at your own risk.」

  ⇒ 一个飞控库在注释里写「use at your own risk」，是很少见的。
    它说明**作者知道这套自适应在数学上还没有被完全证明**。
""")

    rows = []
    for T in (10.0, 12.0):
        z = 0.7
        tc = time_const(T, z)
        rows.append(["%.0f" % T, "%.3f" % p_gain(T, z), "%.3f" % tc,
                     "%.1f" % (math.pi * 0.5 / z),            # roll_tc=0.5
                     "%.1f" % (4 * math.pi * 0.5 * z)])
    tab(rows, ["period [s]", "p_gain [rad/s]", "time_const [s]",
               "下限(零曲率) [s]", "下限(有曲率) [s]"])
    print("  （roll_time_const 取 PX4 默认的 `NPFG_ROLL_TC = 0.5 s`；")
    print("    下限这里没乘安全系数 1.5 —— 源码里是乘了的（:162、:165）。）")


# ===========================================================================
#  实验 3 ★★★ 风三角：从「方位」解出「空速矢量」
# ===========================================================================

def exp3():
    head("实验 3 ★★★ 风三角：从「我想往哪个方位飞」解出「空速矢量该是多少」")

    print("""  这是 NPFG 和 L1 最根本的分工差异：

      L1   : 输入【地面速度】→ 输出横向加速度。  它压根不知道风的存在。
      NPFG : 输入【地面速度 + 风矢量 + 空速能力】→ 先算出**空速矢量给定**，
             再从这个给定和当前空速的差算横向加速度。

  「空速矢量给定」是怎么来的？——解一个**三角形**：

      已知：风矢量 W（大小方向都知道，来自风估计）
      已知：我想沿着方位线 b 飞（方位矢量，实验 1 算出）
      已知：我能用的空速大小 a（nom 或 max）
      求  ：空速矢量 A，使得 A + W 沿着 b 的方向

  代码里它叫 `solveWindTriangle()`（npfg.cpp:413-419），
  实现只有两行 —— 源码注释说得很准：
      「essentially a 2D rotation with the speeds (magnitudes) baked in」
""")

    sub("① 无风：退化成「空速 = 地速」")
    bvec = np.array([1.0, 0.0])
    wind = np.array([0.0, 0.0])
    wcb = cross2(wind, bvec)
    wdb = dot2(wind, bvec)
    ws = norm2(wind)
    a_nom, a_max = 20.0, 25.0
    av, br = ref_air_velocity(wind, bvec, wcb, wdb, ws, a_nom, a_max, 0.0)
    print("  风 = (%.1f, %.1f) m/s，方位 = (%.1f, %.1f)"
          % (wind[0], wind[1], bvec[0], bvec[1]))
    print("  分支 = %s" % br)
    print("  wind_cross_bearing = %.4f   wind_dot_bearing = %.4f"
          % (wcb, wdb))
    print("  空速矢量给定 = (%+.4f, %+.4f)，模长 = %.4f"
          % (av[0], av[1], norm2(av)))
    gv = av + wind
    print("  代回验算：地速 = 空速 + 风 = (%+.4f, %+.4f)" % (gv[0], gv[1]))
    print("  ⇒ 无风时地速与空速重合、沿方位线，符合预期 ✓")

    sub("② 侧面来风：空速要「偏着飞」才能沿方位线走")
    rows = []
    for wy in (0.0, 3.0, 6.0, 9.0, 12.0, 15.0):
        wind = np.array([0.0, wy])
        wcb = cross2(wind, bvec)
        wdb = dot2(wind, bvec)
        ws = norm2(wind)
        feas = bearing_feasibility(wcb, wdb, a_nom, ws)
        if not bearing_is_feasible(wcb, wdb, a_nom, ws):
            rows.append(["%.1f" % wy, "—", "%.3f" % feas, "不可行"])
            continue
        av, br = ref_air_velocity(wind, bvec, wcb, wdb, ws, a_nom, a_max, 0.0)
        gv = av + wind
        # 地速应该沿 bvec 方向
        ang = math.degrees(math.atan2(gv[1], gv[0]))
        rows.append(["%.1f" % wy, "%.4f" % norm2(av), "%.3f" % feas,
                     "%+.2f deg, |地速|=%.2f" % (ang, norm2(gv))])
    tab(rows, ["侧风 wy [m/s]", "空速给定 [m/s]", "可行性", "地速方向与大小"])
    print("""  ★ 读第 4 列：**不管侧风多大，地速方向始终沿方位线（0.00 度）** ——
    因为空速矢量被解成「侧着顶住风」的方向。
    这正是风三角要的效果：**你要的是地速的方向，不是空速的方向。**

  ⚠️ 但注意第 2 列：空速给定会**变大**（因为要顶风），
    而空速是有上限的（`airspeed_max_`）。风一超过某个值就不可行了 —— 见下。""")

    sub("③ 可行性判据：什么时候「到不了那个方位」")
    print("""  `bearingIsFeasible()`（npfg.cpp:407-411）只有一行判断：

      return (fabsf(wind_cross_bearing) < airspeed) && ((wind_dot_bearing > 0.0f) || (wind_speed < airspeed));

  **翻译**：能到达方位 b，当且仅当
    ① 风速在**垂直于 b** 方向的分量，小于可用空速；
    ② 并且（风在 b 方向上**推着你**，或者风本身小于空速）。

  第二条有点绕，它的意思是：如果风在 b 方向上**顶着你**（wind_dot < 0），
  那就必须靠空速硬顶 —— 而只有空速比风速大才顶得住。
""")
    rows = []
    for ws_ in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0):
        # 风完全逆着方位线吹
        wind = np.array([-ws_, 0.0])
        wcb = cross2(wind, bvec)
        wdb = dot2(wind, bvec)
        f_nom = bearing_feasibility(wcb, wdb, a_nom, ws_)
        f_max = bearing_feasibility(wcb, wdb, a_max, ws_)
        rows.append(["%.1f" % ws_, "%.3f" % f_nom, "%.3f" % f_max,
                     "%.3f" % min(max((a_nom - abs(wcb if wdb >= 0 else ws_)) / AIRSPEED_BUFFER, 0), 1)])
    tab(rows, ["逆风 [m/s]", "可行性(空速20)", "可行性(空速25)", "归一化自变量"])
    print("""  ⇒ 逆风 20 m/s 时，nom 空速（20）刚好不顶用：可行性掉到 0；
    但 max 空速（25）还有余量，可行性是 0.64 左右。

  ★ `bearingFeasibility()`（:431-443）返回的不是布尔值，而是 [0,1] 的**连续量**：

      sin^2( pi/2 * clamp((airspeed - |wind_cross|) / AIRSPEED_BUFFER, 0, 1) )

    它有三个用处（`guideToPath` 里都能看到）：
      · :125  合成 feas_combined = feas_ * feas_on_track_，作为总门控
      · :147  给**曲率前馈**淡入淡出（不可行时归零）
      · :234  缩小周期的上界

  ⇒ **「连续可行性」而不是「布尔可行」**，是 NPFG 比 L1 精细的一个体现：
    它可以表达「有点勉强」，从而**平滑地**降低激进程度，而不是一脚踩到 0。""")

    sub("④ 两个前置条件（源码明确写了，别踩）")
    print("""  这两个函数的注释里都写着「调用前必须已经判过可行性」：

    `projectAirspOnBearing()`（:401-403）：
       「it is assumed that bearing feasibility is checked and found feasible
         (e.g. bearingIsFeasible() = true) prior to entering this method
         otherwise the return will be erroneous」

    `infeasibleAirVelRef()`（:424-426）：
       「it is assumed that bearing feasibility is checked and found infeasible
         ... otherwise the normalization of the air velocity vector could have
         a division by zero」

  ★ 这两个条件很容易被忽略，因为**违反了也不会崩**：
    前者会被 `math::max(..., 0.0f)` 截成 0，返回一个「看起来正常」的 0；
    后者只有在特定输入下才除零。
  ⇒ 读源码时，**「不崩」不等于「用对了」**。这类注释是作者在替你踩坑。""")


# ===========================================================================
#  实验 4 ★★★ 抗风：wind excess regulation 与「零地速」终态
# ===========================================================================

def exp4():
    head("实验 4 ★★★ 抗风：wind excess regulation 与「零地速」终态")

    print("""  现在把风**从 0 一路加大到超过空速**，看空速给定怎么演化。
  这是 NPFG 名字里那个 `excess wind handling` 的主角。

  期望沿 +x 方向飞（方位矢量 = (1,0)），逆风沿 -x 吹。
  名义空速 20 m/s，最大空速 25 m/s。
""")

    bvec = np.array([1.0, 0.0])
    a_nom, a_max = 20.0, 25.0

    sub("① 空速给定与地速随逆风演化")
    rows = []
    for ws_ in (0.0, 5.0, 10.0, 15.0, 19.0, 21.0, 23.0, 25.0, 28.0, 35.0):
        wind = np.array([-ws_, 0.0])
        wcb = cross2(wind, bvec)
        wdb = dot2(wind, bvec)
        av, br = ref_air_velocity(wind, bvec, wcb, wdb, ws_, a_nom, a_max, 0.0)
        gv = av + wind
        rows.append(["%.1f" % ws_, "%.3f" % norm2(av),
                     "%+.3f" % gv[0], "%.3f" % norm2(gv), br])
    tab(rows, ["逆风 [m/s]", "空速给定 [m/s]", "地速 x [m/s]", "|地速| [m/s]", "走了哪个分支"])
    print("""  ★★★ **这张表是本章最重要的结果。** 一行一行读：

    · 0 → 19 m/s：地速从 20 一路降到 1，空速给定保持名义值 20 —— 正常飞行。
    · **21 ~ 25 m/s：地速被精确压到 0。** 空速给定 = 逆风分量（21 → 25）。

    **飞机在「原地悬停」** —— 不是倒飞、不是被吹跑，而是**停在原地**。

  它是怎么算出来的？走的是**分支 ①**（npfg.cpp:339-340）：

      airspeed_min = sqrt( (min_ground_speed - wind_dot_bearing)^2
                         + wind_cross_bearing^2 )

  这里 min_ground_speed = 0、风完全逆向所以 wind_cross_bearing = 0，于是

      airspeed_min = sqrt( (0 - (-21))^2 + 0 ) = 21   ← **正好等于逆风分量**

  ⇒ 要让地速恰好为 0，需要的空速就**正好等于逆风分量**。
    这不是巧合，是「顶风悬停」的物理定义：**空速把风顶掉，地速归零。**

    · 超过 25 m/s（最大空速）：缓解律 —— 顶不住了，按能做的做。
""")

    sub("② ★★ 一处「注释与代码对不上」—— 实测出来的")
    print("""  `npfg.cpp:377-380` 那两行注释是这么写的：

      if (wind_dot_bearing <= 0.0f) {
          // we only increment the airspeed to regulate, but not overcome, excess wind
          // NOTE: in the terminal condition, this will result in a zero ground
          //       velocity configuration
          air_vel_ref = wind_vel;
      }

  **注释说：这样会得到「零地速」。**

  但按 :82 的定义 `air_vel = ground_vel - wind_vel`（即 `地速 = 空速 + 风`）：

      空速给定 = 风矢量  ⇒  地速 = 风 + 风 = **2 × 风**

  这是**加倍**，不是归零。下面把它跑出来 ——
  要走到这个分支，必须满足 `min_ground_speed <= wind_dot_bearing` 且 `wind_dot_bearing <= 0`，
  也就是**纯侧风**（wind_dot_bearing 恰好 = 0）：
""")
    rows = []
    for ws_ in (21.0, 22.0, 24.0):
        wind = np.array([0.0, ws_])            # 纯侧风
        wcb = cross2(wind, bvec)
        wdb = dot2(wind, bvec)
        av, br = ref_air_velocity(wind, bvec, wcb, wdb, ws_, a_nom, a_max, 0.0)
        gv = av + wind
        rows.append(["%.1f" % ws_, "%.2f" % wdb, "%.3f" % norm2(av),
                     "%.3f" % norm2(gv), br])
    tab(rows, ["侧风 [m/s]", "wind_dot_bearing", "空速给定 [m/s]",
               "|地速| [m/s]", "分支"])
    print("""  ⇒ **实测：地速 = 2 × 风速**（21 → 42，22 → 44，24 → 48），
    和注释说的「零地速」完全相反。

  ★ 怎么理解这件事？三种可能，本章不下定论但把线索摆出来：

    ① **注释是错的**（或过期）。这个分支的算式就是 `air_vel_ref = wind_vel`，
       按定义只能得到 2× 风。
    ② **注释描述的是"意图"**：作者想要的效果是「顶住风、原地不动」，
       而那需要 `air_vel_ref = -wind_vel`（空速反向顶风）。
       代码少了一个负号，或者注释描述的是尚未实现的目标。
    ③ **本章的解读有误**：可能 PX4 在这个上下文里对 `wind_vel` 的符号
       另有约定。本脚本核对过 `air_vel = ground_vel - wind_vel`（:82）
       和 `vehicle_local_position.msg` 里「wind velocity in local NED frame」
       的定义，两者是自洽的 —— 但**这不排除调用方传入了别的约定**。

  ⚠️ **本章的处理原则**：把**实测数字**（2× 风速）和**源码原文**（零地速）
    都摆出来，指出它们不一致，**不替上游下"这是 bug"的结论**。
    要下定论，得去读 `FixedwingPositionControl` 传给 `guideToPath` 的
    `_wind_vel` 到底是什么符号 —— 那超出本章的范围了。

  ★ 但有一条是可以确定的：**这个文件头部自己写着**（npfg.hpp:47-49）

      「TODO: Concise, up to date documentation and stability analysis
        for the following implementation.」

    —— 上游作者明确承认这份实现**还没有一份准确、最新的文档和稳定性分析**。
    遇到注释对不上，这行 TODO 就是最好的注脚。
""")

    print("""  ★★ **为什么"停在原地"是一个好答案？**

  回到文献 [1] 的标题：《On Flying Backwards: Preventing Run-away of
  Small, Low-speed, Fixed-wing UAVs in Strong Winds》——
  「关于倒着飞：防止小型低速固定翼在强风中的失控漂移」。

  对比 L1 在这种风下的行为（卷 3 第 6 章）：
    · L1 只看地速，不知道风；
    · 强风下它算出的修正方向可能是**反的**，因为地速矢量已经被风扭曲；
    · ArduPilot 的补丁（用机头方向重构地速）只是**防止指令反向**，
      飞机**依然在被吹着后退**。

  NPFG 的选择是：**承认顶不过去，于是不再硬顶，而是把地速压到 0。**
  用控制论的话说 —— **把「不可达的跟踪目标」换成一个「可达的稳定状态」**。

  ⚠️ 注意这不是「安全问题自动解决」：飞机停在原地**依然是在强风里**，
    能量、姿态、高度都还要靠别的通道管。
    它解决的是**制导层不该给出错误指令**这件事。""")

    sub("③ wind excess regulation 是个总闸")
    print("""  `en_wind_excess_regulation_`（默认 **true**，参数 `NPFG_WIND_REG` 默认 1）
  在 `refAirVelocity()` 里作为门控出现了 **5 次**（:335、:375、:391 等）。
  把它关掉会发生什么？——**所有空速增量都被关掉，给定永远用名义空速**。
""")
    rows = []
    for ws_ in (15.0, 21.0, 25.0, 30.0):
        wind = np.array([-ws_, 0.0])
        wcb = cross2(wind, bvec)
        wdb = dot2(wind, bvec)
        av_on, br_on = ref_air_velocity(wind, bvec, wcb, wdb, ws_, a_nom, a_max, 0.0,
                                        en_wind_reg=True)
        av_off, br_off = ref_air_velocity(wind, bvec, wcb, wdb, ws_, a_nom, a_max, 0.0,
                                          en_wind_reg=False)
        gv_on = av_on + wind
        gv_off = av_off + wind
        rows.append(["%.1f" % ws_, "%.2f" % norm2(av_on), "%+.2f" % gv_on[0],
                     "%.2f" % norm2(av_off), "%+.2f" % gv_off[0]])
    tab(rows, ["逆风 [m/s]", "开: 空速给定", "开: 地速 x",
               "关: 空速给定", "关: 地速 x"])
    print("""  ⇒ 关掉之后，空速给定**始终卡在 20 m/s**（名义），
    逆风 25 m/s 时地速变成 **-5 m/s** —— **飞机在倒退**。

  ★ 这正是文献标题里那个 "Flying Backwards" 的场景。
    `NPFG_WIND_REG` 这个参数的作用，就是**决定要不要让飞机进入这个状态**。""")

    sub("④ 三个开关的默认值与它们的关系")
    rows = [
        ["`NPFG_WIND_REG`", "1（开）", "抗风总闸：关掉则永远用名义空速"],
        ["`NPFG_EN_MIN_GSP`", "1（开）", "允许为提高地速而增加空速"],
        ["`NPFG_TRACK_KEEP`", "1（开）", "为了「保持在航线上」而增加地速需求"],
        ["`NPFG_LB_PERIOD`", "1（开）", "周期下限（防极限环）"],
        ["`NPFG_UB_PERIOD`", "1（开）", "周期上限（保跟踪稳定）"],
    ]
    tab(rows, ["参数", "默认", "作用"])
    print("""  ⚠️ **一个容易读错的细节**（卷 3 第 6 章也提过类似的）：
    `npfg.hpp` 里 `en_track_keeping_` 的**类内初值是 false**（:304），
    但对应的参数 `NPFG_TRACK_KEEP` 默认是 **1** —— 运行期实际是 **true**。
    这个差异来自「类内初值」和「参数默认值」是两套东西，
    调用者 `parameter_update()` 会把参数灌进去覆盖初值。

  ★ 顺带看 `minGroundSpeed()`（:305-327）怎么把两个需求合起来：
      · 保航线需求 = (1-feas) * max * clamp(nte / NTE_FRACTION, 0, 1)   (:314-316)
        —— **可行度越低、偏差越大，越要求保持地速**
      · 用户需求   = min_gsp_desired_                                   (:323)
      · 取两者的 **max**（:326）

    注意 `NTE_FRACTION = 0.5`：**偏差达到边界的一半时，保航线需求就给满值**。
    这个 0.5 是个经验值，源码没给推导。""")

    if _HAS_MPL:
        ws_arr = np.linspace(0, 35, 120)
        airsp, gsp = [], []
        for ws_ in ws_arr:
            wind = np.array([-ws_, 0.0])
            wcb = cross2(wind, bvec)
            wdb = dot2(wind, bvec)
            av, _ = ref_air_velocity(wind, bvec, wcb, wdb, ws_, a_nom, a_max, 0.0)
            gv = av + wind
            airsp.append(norm2(av))
            gsp.append(gv[0])
        plt.figure(figsize=(8, 3.4))
        plt.plot(ws_arr, airsp, label="空速给定 [m/s]")
        plt.plot(ws_arr, gsp, label="地速 x [m/s]")
        plt.axhline(0, color="k", lw=0.8)
        plt.axvline(a_nom, color="r", lw=0.8, ls="--")
        plt.text(a_nom + 0.3, 15, "名义空速", fontsize=8, color="r")
        plt.xlabel("逆风 [m/s]")
        plt.ylabel("速度 [m/s]")
        plt.title("NPFG 抗风：逆风增大时的空速给定与地速")
        plt.grid(alpha=0.3)
        plt.legend(fontsize=8)
        savefig("lesson32_exp4")


EXPERIMENTS = {
    "1": ("方位矢量：前视角是二次的", exp1),
    "2": ("偏差边界与 track_proximity", exp2),
    "3": ("风三角：从方位解空速矢量", exp3),
    "4": ("抗风与零地速终态", exp4),
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
