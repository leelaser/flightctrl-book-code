#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 16 周 · 01 PID 控制器：从零到级联 —— 配套实验

    实验 1  二阶系统 + P 控制：稳态误差从哪来（以及 D 为什么救不了它）
    实验 2  ★ PI / PID：积分怎么消掉稳态误差，以及积分饱和的三种处理
    实验 3  ★ PID：D 项在干什么，以及「对噪声敏感」的定量实测
    实验 4  ★★ 级联 PID：外环 + 内环，以及带宽分离原则

用法：
    python lesson16_pid.py 1
    python lesson16_pid.py 2
    python lesson16_pid.py 3
    python lesson16_pid.py 4
    python lesson16_pid.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的被控对象是什么】

  全脚本只用【一个】被控对象 —— 四旋翼姿态的【单轴】：

        J·ẍ = u + d          ← 双积分器 + 常值扰动力矩

  为什么用它？
    · 第 3 章讲过：J·ω̇ = M − ω×(Jω)，小角速率下就是 J·ω̇ ≈ M
    · 所以「角度环 + 力矩」的被控对象就是【纯双积分器】——
      没有阻尼、没有自稳性，这是最难控的一类对象
    · ★ 而它正好能把 PID 的每一项都逼出来：
        P 提供刚度、D 提供阻尼、I 消掉 d 造成的稳态误差

  J = 0.008 kg·m² 是第 3 章用的 5 寸机 roll 轴转动惯量（真实量级）。
  d 是常值扰动力矩（重心偏移、电机安装误差……）。

【本脚本的原则】

  和《NuttX/FreeRTOS 教程》一样：**能推的就不背，能算的就不猜。**

  每一个「课文里说的现象」，本脚本都给出：
      ① 解析预测（用公式算出来的数）
      ② 数值实测（仿真跑出来的数）
      ③ 两者的对比

  ★ 实验 2 的「积分饱和」、实验 3 的「D 项噪声」、实验 4 的「带宽分离」
    都会给出【可比较的数字】，而不是「看起来振荡了」。

参考源码（引用行号均已打开确认）：
  PX4-Autopilot/src/lib/pid/pid.cpp:147-162            积分限幅型抗饱和
  PX4-Autopilot/src/lib/pid/pid.cpp:125-138            微分作用于测量值（NO_SP）
  PX4-Autopilot/src/lib/rate_control/rate_control.cpp:78       D 用角加速度
  PX4-Autopilot/src/lib/rate_control/rate_control.cpp:88-118   饱和反馈型抗饱和
  PX4-Autopilot/src/modules/mc_rate_control/MulticopterRateControl.cpp:133
  ArduPilot/libraries/AC_PID/AC_PID.cpp:322-336        条件积分型抗饱和
  ArduPilot/libraries/AC_PID/AC_PID.cpp:248-253        微分 + 低通滤波
"""

import math
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_PLT = True
except ImportError:
    HAVE_PLT = False

W = 78
HERE = os.path.dirname(os.path.abspath(__file__))

# 5 寸机 roll 轴转动惯量（第 3 章用的值）
J_DEFAULT = 0.008


# ---------------------------------------------------------------------------
# 输出辅助
# ---------------------------------------------------------------------------

def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (len(title) + 2))
    print()


def _dw(s):
    """显示宽度：CJK 字符算 2 列。"""
    n = 0
    for ch in s:
        o = ord(ch)
        n += 2 if (0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF
                   or 0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF
                   or 0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60
                   or 0xFFE0 <= o <= 0xFFE6) else 1
    return n


def _pad(s, w):
    return s + " " * max(w - _dw(s), 0)


def tab(rows, headers):
    """打印表格（按显示宽度对齐，CJK 算 2 列）。"""
    rows = [[str(c) for c in r] for r in rows]
    headers = [str(h) for h in headers]
    widths = [max(_dw(r[i]) for r in [headers] + rows) + 2
              for i in range(len(headers))]
    print("  " + "".join(_pad(headers[i], widths[i])
                         for i in range(len(headers))))
    print("  " + "-" * (sum(widths) + 4))
    for r in rows:
        print("  " + "".join(_pad(r[i], widths[i]) for i in range(len(r))))
    print()


# ---------------------------------------------------------------------------
# 被控对象与仿真
# ---------------------------------------------------------------------------

class Plant:
    """单轴二阶对象：  J·ẍ = u + d − b·ẋ

    J : 转动惯量（kg·m²）
    d : 常值扰动力矩（N·m）—— 重心偏移、气动不对称……
    b : 阻尼系数（N·m·s/rad）—— 真机机身轴上几乎没有，默认 0
    u_max : 力矩上限（N·m）—— 电机能给出的最大差动力矩
    """

    def __init__(self, J=J_DEFAULT, d=0.0, b=0.0, u_max=None):
        self.J = J
        self.d = d
        self.b = b
        self.u_max = u_max

    def deriv(self, x, xd, u):
        return xd, (u + self.d - self.b * xd) / self.J


class PID:
    """一个「工程实现」的 PID —— 每个开关都对应一家飞控的真实做法。

    kp, ki, kd      : 三个增益（注意：ki 作用在误差积分上，kd 作用在导数上）
    u_max           : 输出限幅（对应 PX4 的 normalized torque 1.0 / AP 的 -1~1）
    i_limit         : 积分项限幅（AP 的 IMAX / PX4 的 MC_RR_INT_LIM）
    anti_windup     : None      —— 不抗饱和（自由积分，会 windup）
                      'clamp'   —— 输出/积分限幅，饱和时不更新积分
                                   （PX4 pid.cpp:147-158）
                      'freeze'  —— 饱和时只允许积分朝「减小误差」方向走
                                   （ArduPilot AC_PID.cpp:324-336）
                      'backcalc'—— 反算：把 (u_sat − u_unsat) 反馈回积分器
    d_source        : 'measured' —— ★ 速度是【直接测到】的（陀螺 / 速度计）：
                                    D = −kd·ẋ_meas
                                    真机上飞行器的角速率就是陀螺直读的，
                                    所以内环的「阻尼项」不需要微分。
                      'diff'     —— ★ 速度只能靠【微分】得到（后向差分）：
                                    D = −kd·(x[n] − x[n−1]) / dt
                                    这是 PX4 速率环 D 项的做法
                                    （VehicleAngularVelocity.cpp:776），
                                    ⚠ 它会带来 kd < J 的离散稳定上界（实验 1 ②）。
    d_on_measurement: ★ 只对 d_source='diff' 有意义
                      True  —— 微分【测量值】（不含设定值）
                                 （PX4 rate_control.cpp:78 就是这么做的）
                      False —— 微分【误差】（含设定值 ⇒ 微分冲击）
    d_lpf_hz        : D 项一阶低通截止频率（None = 不滤）
                      （ArduPilot AC_PID 的 FLTD / PX4 的 IMU_DGYRO_CUTOFF）
    Tt              : 反算时间常数（只在 anti_windup='backcalc' 时用）
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, u_max=None, i_limit=None,
                 anti_windup="clamp", d_source="measured",
                 d_on_measurement=True, d_lpf_hz=None, Tt=0.02):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.u_max = u_max
        self.i_limit = i_limit if i_limit is not None else (
            u_max if u_max is not None else float("inf"))
        self.anti_windup = anti_windup
        self.d_source = d_source
        self.d_on_measurement = d_on_measurement
        self.d_lpf_hz = d_lpf_hz
        self.Tt = Tt
        self.reset()

    def reset(self):
        self.I = 0.0            # 【积分项本身】（已经乘过 ki）—— 和 PX4 一致
        self.e_prev = None
        self.x_meas_prev = None
        self.xd_meas_prev = None
        self.d_filt = 0.0
        self.last = {"P": 0.0, "I": 0.0, "D": 0.0, "u": 0.0, "u_unsat": 0.0}

    def __call__(self, t, r, x, xd_meas, dt):
        e = r - x

        # ---- D 项 ----
        if self.d_source == "measured":
            # ★ 速度是直接测到的 ⇒ D = −kd·ẋ
            #   没有微分 ⇒ 不放大噪声，也没有 kd < J 的离散上界。
            #   真机上「速率环的阻尼」就是这种情况（陀螺直读角速率）。
            #   ⚠ 此时 d_on_measurement 开关【不起作用】—— 不微分就没有
            #     设定值的导数，也就没有微分冲击。
            d_raw = -xd_meas
        else:
            # ★ 速度只能靠微分得到 ⇒ D = −kd·(被微分的量/dt)
            #   d_on_measurement=True  → 微分【测量值】ẋ（PX4 风格）
            #   d_on_measurement=False → 微分【误差】e = r − x（含设定值，
            #                            设定值突变时会有微分冲击）
            src = xd_meas if self.d_on_measurement else e
            if self.xd_meas_prev is None:
                d_raw = 0.0
            else:
                d_raw = -(src - self.xd_meas_prev) / dt
            self.xd_meas_prev = src
        self.e_prev = e

        if self.d_lpf_hz is not None and self.d_lpf_hz > 0.0:
            tau = 1.0 / (2.0 * math.pi * self.d_lpf_hz)
            alpha = dt / (tau + dt)
            self.d_filt += alpha * (d_raw - self.d_filt)
            d_lpf = self.d_filt
        else:
            d_lpf = d_raw

        D = self.kd * d_lpf
        P = self.kp * e

        # ---- 先算「未限幅」的输出，抗饱和要用 ----
        u_unsat = P + self.I + D
        u_sat = u_unsat
        if self.u_max is not None:
            u_sat = min(max(u_unsat, -self.u_max), self.u_max)

        # ---- I 项更新 ----
        if self.anti_windup is None:
            self.I += self.ki * e * dt
            self.I = min(max(self.I, -self.i_limit), self.i_limit)

        elif self.anti_windup == "clamp":
            # PX4 pid.cpp:147-158：先试探性地积分，
            # 只有「积分后输出仍在范围内」且「积分本身未超限」才接受
            I_new = self.I + self.ki * e * dt
            if (abs(P + D + I_new) <= (self.u_max or float("inf"))
                    and abs(I_new) <= self.i_limit):
                self.I = I_new

        elif self.anti_windup == "freeze":
            # ArduPilot AC_PID.cpp:324-336：
            # 饱和时，积分只允许朝「能让输出离开饱和」的方向变
            saturated = (u_unsat != u_sat)
            same_sign = (self.I > 0 and e > 0) or (self.I < 0 and e < 0)
            if (not saturated) or (not same_sign):
                self.I += self.ki * e * dt
                self.I = min(max(self.I, -self.i_limit), self.i_limit)

        elif self.anti_windup == "backcalc":
            # 反算（back-calculation）：把超出量按 1/Tt 的速率灌回积分器
            self.I += (self.ki * e + (u_sat - u_unsat) / self.Tt) * dt
            self.I = min(max(self.I, -self.i_limit), self.i_limit)

        # ---- 用更新后的 I 重算输出 ----
        u_unsat = P + self.I + D
        u_sat = u_unsat
        if self.u_max is not None:
            u_sat = min(max(u_unsat, -self.u_max), self.u_max)

        self.last = {"P": P, "I": self.I, "D": D,
                     "u": u_sat, "u_unsat": u_unsat}
        return u_sat

    def terms(self):
        return self.last


def simulate(ctrl, T=4.0, dt=1.0 / 2000, plant=None, r_fn=None,
             x0=0.0, xd0=0.0, sensor_noise=0.0, seed=1):
    """闭环仿真（力矩在一步内零阶保持 —— 和真实飞控一样）。

    sensor_noise : 角速率测量的噪声标准差（rad/s），模拟陀螺噪声。
                   角度 x 假设是估计出来的（第 4 章），这里不额外加噪声。
    """
    plant = plant or Plant()
    r_fn = r_fn or (lambda t: 1.0)
    rng = np.random.default_rng(seed)

    x, xd = x0, xd0
    ctrl.reset()

    n = int(round(T / dt))
    log = {k: np.zeros(n) for k in
           ("t", "r", "x", "xd", "u", "u_unsat", "P", "I", "D")}

    for i in range(n):
        t = i * dt
        r = r_fn(t)
        xd_meas = xd + (sensor_noise * rng.standard_normal()
                        if sensor_noise > 0 else 0.0)

        # 控制器先用【上一时刻】的测量算一次，拿到 u（零阶保持）
        u = ctrl(t, r, x, xd_meas, dt)
        tm = ctrl.terms()

        log["t"][i] = t
        log["r"][i] = r
        log["x"][i] = x
        log["xd"][i] = xd
        log["u"][i] = u
        log["u_unsat"][i] = tm["u_unsat"]
        log["P"][i] = tm["P"]
        log["I"][i] = tm["I"]
        log["D"][i] = tm["D"]

        # RK4 推进一步（u 固定）
        k1x, k1v = plant.deriv(x, xd, u)
        k2x, k2v = plant.deriv(x + 0.5 * dt * k1x, xd + 0.5 * dt * k1v, u)
        k3x, k3v = plant.deriv(x + 0.5 * dt * k2x, xd + 0.5 * dt * k2v, u)
        k4x, k4v = plant.deriv(x + dt * k3x, xd + dt * k3v, u)
        x += dt / 6.0 * (k1x + 2 * k2x + 2 * k3x + k4x)
        xd += dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)

    return log


# ---------------------------------------------------------------------------
# 响应指标
# ---------------------------------------------------------------------------

def select(log, mask):
    """按布尔掩码取 log 的一个子段（返回新的 dict）。"""
    return {k: v[mask] for k, v in log.items()}


def metrics(log, tail_frac=0.1, ref=None):
    """返回 (稳态误差, 超调百分比, 上升时间, 2% 调节时间, 峰值)。

    ref : 超调/上升时间/调节时间所参照的【终值】。
          默认取设定值 r 的最后 5%（理想情况）；有常值扰动时应该传
          「实测终值」，否则「超调」里会混进稳态误差的偏移量。
    """
    t, r, x = log["t"], log["r"], log["x"]
    n_tail = max(int(len(x) * tail_frac), 10)
    ss_err = float(np.mean(r[-n_tail:] - x[-n_tail:]))

    r_final = float(np.mean(r[-max(int(len(r) * 0.05), 5):]))
    target = r_final if ref is None else float(ref)
    x0 = x[0]
    span = target - x0

    if abs(span) < 1e-12:
        return ss_err, 0.0, float("nan"), float("nan"), float(x[-1])

    peak = float(np.max(x)) if span > 0 else float(np.min(x))
    overshoot = (peak - target) / span * 100.0

    # 上升时间：第一次到达终值的 90%
    thr = x0 + 0.9 * span
    hit = (x >= thr) if span > 0 else (x <= thr)
    rise = float(t[np.argmax(hit)]) if hit.any() else float("nan")

    # 2% 调节时间：最后一次离开 ±2% 带
    band = 0.02 * abs(span)
    outside = np.abs(x - target) > band
    if outside.any():
        settle = float(t[np.max(np.nonzero(outside)[0])])
    else:
        settle = 0.0

    return ss_err, overshoot, rise, settle, peak


def final_value(log, frac=0.1):
    """实测终值（最后 frac 段的均值）。有常值扰动时用它当超调的参照。"""
    n = max(int(len(log["x"]) * frac), 10)
    return float(np.mean(log["x"][-n:]))


def fnum(x, fmt="%.3f", big=1.0e6):
    """格式化一个数；非有限或过大时返回「—」（防止发散值撑爆表格）。"""
    if x is None or not math.isfinite(float(x)) or abs(float(x)) > big:
        return "—"
    return fmt % float(x)


def env_peak(log, t_lo, t_hi, ref=None):
    """一段时间窗内 |x − ref| 的最大值 —— 振荡的包络。

    ref 默认是设定值 r；传「实测终值」可以区分「一个恒定的静差」（稳定）
    和「还在振荡」（不稳定）—— 这是判断收敛的关键。
    """
    seg = select(log, (log["t"] >= t_lo) & (log["t"] < t_hi))
    if len(seg["t"]) == 0:
        return 0.0
    ref = seg["r"] if ref is None else ref
    return float(np.max(np.abs(seg["x"] - ref)))


def _norm_torque_scale(mass=1.0, arm=0.25, twr=2.0):
    """「归一化力矩 1.0」对应多少 N·m —— 用第 8 课的混控矩阵算。

    复现 AP_MotorsMatrix.cpp:536-544 的 add_motor：
        roll 系数 = cos(θ+90°) = −sin θ
        pitch 系数 = cos θ
        yaw 系数  = σ (CCW=+1 / CW=−1)
    QUAD_X 的电机定义见 AP_MotorsMatrix.cpp:591-599。
    """
    g = 9.80665
    th = mass * g / 4.0
    t_max = twr * th                      # 单电机最大推力 = 悬停的 twr 倍
    frames = [(45, +1), (-135, +1), (-45, -1), (135, -1)]
    mix = []
    for ang, sig in frames:
        r = math.cos(math.radians(ang + 90.0))
        p = math.cos(math.radians(ang))
        mix.append([r, p, float(sig), 1.0])
    mix = np.array(mix).T                 # (4, 4) 行 = roll/pitch/yaw/thr

    # 指令 [roll=1, pitch=0, yaw=0, throttle=1] ⇒ 每个电机的推力倍数
    fac = mix @ np.array([1.0, 0.0, 0.0, 1.0])
    thrust = th * np.clip(fac, 0.0, t_max / th)
    tau = 0.0
    for i, (ang, _sig) in enumerate(frames):
        y_i = math.sin(math.radians(ang)) * arm
        tau += -y_i * thrust[i]
    return abs(tau)


def zeta_from_kd(kd, kp, J):
    """二阶系统  J·ẍ + kd·ẋ + kp·x = 0  的阻尼比。"""
    return kd / (2.0 * math.sqrt(J * kp))


def wn_from_kp(kp, J):
    return math.sqrt(kp / J)


def overshoot_theory(zeta):
    """二阶欠阻尼系统阶跃响应的超调量（%）。"""
    if zeta >= 1.0:
        return 0.0
    return 100.0 * math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta * zeta))


# ===========================================================================
# 实验 1：二阶系统 + P 控制
# ===========================================================================

def exp1():
    head("实验 1  二阶系统 + P 控制：稳态误差从哪来")

    print("""
  被控对象（四旋翼姿态单轴）：

        J·ẍ = u + d            J = %.4f kg·m²

  【第一部分】先只看 P，而且不加扰动（d = 0）：

        u = kp·(r − x)

  代入：   J·ẍ = kp·(r − x)
    ⇒      J·ẍ + kp·x = kp·r

  ⇒ 这是一个【无阻尼】二阶系统：

        ωn = sqrt(kp / J)          ζ = 0    ← ★ 阻尼比恒为零！

  ★★ 为什么？因为被控对象里【没有任何一项和 ẋ 有关】。
     P 是作用在 x 上的，它只能提供「弹簧」，提供不了「阻尼」。
""" % J_DEFAULT)

    J = J_DEFAULT
    r_step0 = math.radians(10.0)

    def amp_at(log, r_ref, t_lo, t_hi):
        seg = select(log, (log["t"] > t_lo) & (log["t"] <= t_hi))
        return float(np.max(np.abs(seg["x"] - r_ref))) if len(seg["t"]) else float("nan")

    print("    下面用两个不同的 kp 各跑一次，看 P 到底决定了什么：")
    print()

    rows = []
    for mult in (1.0, 4.0):
        kp = J * 20.0 ** 2 * mult
        wn = wn_from_kp(kp, J)
        c = PID(kp=kp, ki=0.0, kd=0.0, u_max=None, d_source="measured")
        log = simulate(c, T=3.0, dt=1.0 / 2000, plant=Plant(J=J, d=0.0),
                       r_fn=lambda t: r_step0 if t > 0.2 else 0.0, x0=0.0)

        # 实测周期：误差过零点之间的时间差（每个周期过零两次）
        seg = select(log, (log["t"] > 0.3) & (log["t"] < 1.9))
        cross = np.nonzero(np.diff(np.sign(seg["x"] - seg["r"])))[0]
        if len(cross) >= 3:
            period_meas = 2.0 * float(np.mean(np.diff(seg["t"][cross])))
        else:
            period_meas = float("nan")

        a1 = amp_at(log, r_step0, 0.2, 0.8)
        a2 = amp_at(log, r_step0, 2.4, 3.0)
        rows.append(["%.4f" % kp,
                     "%.3f" % wn,
                     "%.4f" % (2 * math.pi / wn),
                     "%.4f" % period_meas,
                     "%.5f" % a1,
                     "%.5f" % a2,
                     "%.4f" % (a2 / a1)])

    tab(rows, ["kp", "ωn", "周期 2π/ωn (s)", "实测周期 (s)",
               "0.2~0.8s 振幅 (rad)", "2.4~3.0s 振幅 (rad)", "振幅比"])
    print("""    ★★ 三个观察：

      ① 实测周期和解析周期 2π/ωn 【逐行对上】
         ⇒ 「P 管刚度（ωn = sqrt(kp/J)）」被证实
      ② kp 增大 4 倍 ⇒ ωn 增大 2 倍（sqrt 关系），周期减半
         ⇒ ★ 想让响应快一倍，P 要加 4 倍
      ③ ★★ 振幅【不衰减】—— 它是「振荡」，不是「收敛」。
         注意周期那两列和 kp 无关的部分：振幅比一列【大于 1】，
         也就是说它其实在【非常缓慢地发散】。见下面 ④。

      ⇒ P 增益只决定了振荡【快慢】，完全决定不了
        振荡【衰减不衰减】。这就是「P 管刚度」的准确含义。
      ⇒ 而「让振荡衰减」是 D 的活 —— 见 ①。
""")

    # ------------------------------------------------------------------
    sub("④ ★★ 一个容易被忽略的真相：数字 P 控制其实在【缓慢发散】")

    print("""    ζ = 0 的连续系统是「等幅振荡」（临界稳定）。但仿真里振幅是【变大】的。
    ★ 这不是数值误差，而是【数字控制器】本身的性质。推导（不跳步）：

        记 θ = ωn·dt（一个采样周期转过的相位）。
        零阶保持（ZOH，控制器每 dt 更新一次 u，两次之间不变）下，
        闭环的精确离散递推是「u 在这一个 dt 内是常数」：

            u = kp·(r − x[n])   ⇒   ẍ = (kp/J)·(r − x[n])   ← 与时间无关
            ⇒ x[n+1] = x[n] + dt·ẋ[n] + dt²·u/(2J)
               ẋ[n+1] = ẋ[n] + dt·u/J

        令误差 ε[n] = x[n] − r，u = −kp·ε[n]，dt²·kp/J = θ²：

            ε[n+1] = ε[n]·(1 − θ²/2) + dt·ẋ[n]
            ẋ[n+1] = −(θ/dt)·ε[n] + ẋ[n]

        ⇒ 状态矩阵  M = [ 1 − θ²/2 ,  dt ;
                         −θ/dt     ,   1 ]

        ⇒ ★★ det(M) = (1 − θ²/2)·1 + dt·θ/dt = **1 + θ²/2 > 1**

        两个特征值的模之积 = det(M) > 1 ⇒ 至少有一个 |λ| > 1
        ⇒ **ZOH 采样 + 纯 P 反馈的双积分器是【不稳定】的**，
           发散速率 |λ| ≈ sqrt(1 + θ²/2) ≈ 1 + θ²/4 （每步）

    ★★ 这是数字控制里最经典的坑之一，物理本质是：
        **零阶保持相当于给回路加了半个采样的纯延迟。**
        ζ = 0 的系统【相位裕度 = 0】，任何一点延迟都会把它推过临界。

    ★ 验证：让 dt 变小，θ 变小，发散应该变慢：
""")

    rows = []
    for dt in (1.0 / 2000, 1.0 / 1000, 1.0 / 500):
        kp = J * 20.0 ** 2
        wn = wn_from_kp(kp, J)
        c = PID(kp=kp, ki=0.0, kd=0.0, u_max=None, d_source="measured")
        log = simulate(c, T=3.0, dt=dt, plant=Plant(J=J, d=0.0),
                       r_fn=lambda t: r_step0 if t > 0.2 else 0.0, x0=0.0)
        a1 = amp_at(log, r_step0, 0.2, 0.5)
        a2 = amp_at(log, r_step0, 2.7, 3.0)
        theta = wn * dt
        pred = math.exp(wn * wn * dt / 4.0 * 2.5)     # 2.5 s 的解析增长倍数
        rows.append(["1/%d" % round(1.0 / dt), "%.4f" % theta,
                     "%.6f" % (1.0 + theta * theta / 4.0),
                     "%.4f" % (a2 / a1), "%.4f" % pred])

    tab(rows, ["dt (s)", "θ = ωn·dt", "每步 |λ| (解析)",
               "实测振幅比", "解析增长倍率"])
    print("""    ★★ 两个结论：

      ① dt 越小、θ 越小、|λ| 越接近 1 ⇒ 发散越慢
         （但【永远】> 1，采样率再高也消不掉）
      ② ★ 真机上 θ 通常是 1e-3~1e-2 量级，所以这个发散
         在几秒的尺度上看不出来；但如果你把 P 加到很大
         （ωn 大 ⇒ θ 大），或者把控制回路跑到很低的频率上，
         它就会变成真实的「慢飘」。

      我们的实测值略小于解析值，因为解析用的是「每步 × 2.5 s」，
      而实测比的是两个时间窗里的【最大振幅】—— 包络在窗内还在变。
      量级和趋势对上了就够了。

    ⇒ ★★ 所以「P-only 是等幅振荡」这句话，在【数字】控制器里
      要修正成：「P-only 是【缓慢发散】的振荡，ζ 严格小于 0」。
      这就是为什么真机的姿态环【必须】有阻尼项。
""")

    # ------------------------------------------------------------------
    sub("① 加上 D：ζ 从 0 变成可控")

    print("""    控制器：

        u = kp·(r − x) − kd·ẋ

    代入：   J·ẍ + kd·ẋ + kp·x = kp·r

    标准二阶形式    ẍ + 2ζωn·ẋ + ωn²·x = ωn²·r ：

        ωn = sqrt(kp/J)            ★ P 定【刚度】（多快）
        ζ  = kd / (2·sqrt(J·kp))   ★ D 定【阻尼】（多稳）

    ★★ 注意这里是【速度 ẋ 直接测到】的情形（真机上就是陀螺直读角速率），
       所以 D 项不需要做任何微分 —— 这一步很关键，请记住它，
       因为 ② 会告诉你「如果只能靠微分得到 ẋ」时代价有多大。
""")

    kp = J * 20.0 ** 2
    r_step = math.radians(10.0)
    rows = []
    for zt in (0.0, 0.2, 0.4, 0.7, 1.0, 1.4, 2.0):
        kd = 2.0 * zt * math.sqrt(J * kp)
        c = PID(kp=kp, ki=0.0, kd=kd, u_max=None, d_source="measured")
        log = simulate(c, T=3.0, dt=1.0 / 2000, plant=Plant(J=J),
                       r_fn=lambda t: r_step if t > 0.2 else 0.0)
        ss, ov, rise, settle, peak = metrics(log)
        rows.append(["%.4f" % kd, "%.2f" % zt,
                     "%.2f" % rise, "%.1f%%" % max(ov, 0.0),
                     "%.1f%%" % overshoot_theory(zt),
                     "%.3f" % settle if math.isfinite(settle) else "—"])

    print("    kp = %.4f（ωn = sqrt(kp/J) = 20 rad/s，无阻尼周期 %.4f s），"
          "kd 按 2ζ·sqrt(J·kp) 反算" % (kp, 2 * math.pi / 20))
    print()
    tab(rows, ["kd", "ζ（反算）", "上升时间 (s)", "超调（实测）",
               "超调（解析）", "2% 调节时间 (s)"])
    print("""    解析超调量公式（二阶欠阻尼阶跃响应）：

        Mp = exp( −πζ / sqrt(1−ζ²) )

    ★★ 从 ζ = 0.2 那一行开始，实测和解析【逐行对上】——
       这说明「P 管刚度、D 管阻尼」不是经验口诀，
       而是 J·ẍ + kd·ẋ + kp·x 这个方程的必然结果。

    ⚠ 第一行（ζ = 0）实测 114.3% > 解析 100.0%，为什么？
       就是上面 ④ 讲的 ZOH 半拍延迟 —— 无阻尼时它让系统缓慢发散，
       峰值比理想值高。★ 而只要 D 给出一点点阻尼（ζ ≥ 0.2），
       这个偏差就消失了。这也从另一个角度说明 P-only 有多脆弱。

    ★ 顺带得到一个工程结论：
        ζ = 0.7 时超调约 4.6%，这是最常用的目标；
        ζ = 1.0（临界阻尼）完全不超调但慢（调节时间反而变长）；
        ζ < 0.4 就开始明显振荡。

    ⚠ 还有一个反直觉的地方：kd 从 0 涨到 0.64（80 倍），
       而 ζ 只从 0 涨到 2.0。★ 因为 ζ 和 kd 【成正比】，
       而 ωn 和 sqrt(kp) 成正比 —— 「想把响应快一倍，P 要乘 4」，
       这在前面 P-only 的表格里已经验证过了。

    ★ 再看「上升时间」和「调节时间」两列：
       上升时间随 kd 单调变大（0.27 → 0.64 s）——
       ★ D 是【阻力】，它一定让响应变慢。
       而调节时间先变小（3.0 → 0.49 s）再变大（0.94 s）——
       ★★ 因为 ζ=1.0 之后系统过阻尼，虽然不超调，但「慢慢爬」。
       这就是「ζ 取 0.7 左右」这个经验值的来历：
       它是「不过冲」和「不磨蹭」之间的折中。
""")

    # ------------------------------------------------------------------
    sub("② ★★ D 的两种实现：微分型有一个硬上界 kd < J")

    print("""    ① 里我们假设 ẋ 是「白送」的。但 D 项的本质是【微分】，
    所以必须回答一个问题：**这个微分会付出什么代价？**

    ★ 答案取决于「ẋ 从哪来」，而工程上只有两种可能：

      ┌── 实现 A：ẋ 是【直接测到】的
      │     真机对应：陀螺直接输出角速率 ω；内环的阻尼用的就是它。
      │     控制器：  u = kp·e − kd·ẋ
      │     代价：    ★ 没有！不微分 ⇒ 不放大噪声、不受 J 限制。
      │
      └── 实现 B：ẋ 必须靠【微分】算出来（后向差分）
            真机对应：PX4 速率环的 D 项 —— 因为速率环已经是最内环，
                      没有比角速率更内层的可测量了，
                      所以只能对角速率再求一次导（角加速度）。
            控制器：  u = kp·e − kd·(ẋ[n] − ẋ[n−1]) / dt
            代价：    ★★ 两个：放大噪声（实验 3）
                      ★★ 撞上离散稳定性墙 kd < J（本节）

    ★★ 下面是实现 B 的稳定性墙，它是【精确可推】的。推导（不跳步）：

        对象：      J·ẍ[n] = u[n]
        控制器：    u[n] = kp·(r − x[n]) − kd·(ẋ[n] − ẋ[n−1]) / dt
        记 a = kd / J

        Step 1  把 u 代进对象，并把 J 除下去：
                ẍ[n] = (kp/J)·(r − x[n]) − (kd/J)·(ẋ[n] − ẋ[n−1]) / dt
                     = (kp/J)·(r − x[n]) − a·(ẋ[n] − ẋ[n−1]) / dt
                两边乘 dt：
                    dt·ẍ[n] = (dt·kp/J)·(r − x[n]) − a·(ẋ[n] − ẋ[n−1])

        Step 2  前向欧拉离散（和仿真里 RK4 的极限行为一致）：
                    ẋ[n+1] − ẋ[n] = dt·ẍ[n]
                ⇒   ẋ[n+1] = ẋ[n] + (dt·kp/J)·(r − x[n])
                                   − a·(ẋ[n] − ẋ[n−1])
                以及
                    x[n+1] = x[n] + dt·ẋ[n]

        Step 3  换成误差 ε[n] = x[n] − r（常数 r 不影响差分）：
                    dt·ẋ[n] = ε[n+1] − ε[n]
                ⇒   ẋ[n] = (ε[n+1] − ε[n]) / dt
                ⇒   ẋ[n] − ẋ[n−1] = (ε[n+1] − 2ε[n] + ε[n−1]) / dt

        Step 4  代回 Step 2 的第一式（两边同乘 dt 再整理）：
                    ε[n+2] − 2ε[n+1] + ε[n]
                        = −(dt²·kp/J)·ε[n] − a·(ε[n+1] − 2ε[n] + ε[n−1])
                令 K = dt²·kp/J，全部并到一边：
                    ε[n+2] − (2 − a)·ε[n+1] + (1 + 2a + K)·ε[n] − a·ε[n−1] = 0

        Step 5  z 变换（ε[n+k] → z^k·E(z)），特征多项式：
                    z³ − (2 − a)·z² + (1 + 2a + K)·z − a = 0

        Step 6  ★★ 韦达定理：三次多项式 z³ + c₂z² + c₁z + c₀ 的
                三根之积 = −c₀。这里 c₀ = −a，所以
                     z₁·z₂·z₃ = a
                三根全在单位圆内 ⇒ 必有 |z₁z₂z₃| < 1 ⇒ **a < 1**

                ★★ 即        kd < J          （J 是转动惯量！）

    ⇒ 这个结论有两个反直觉的地方：

      ① **D 增益的上限由【惯量】决定**，而不是由采样率、滤波器、
         或者「调参经验」决定。J 越小（越轻的机子），D 能开的越小。
      ② ★★ 它和 dt 【无关】—— 因为 dt 在差分（÷dt）和积分（×dt）
         里正好抵消了。**降采样率救不了你。**
""")

    print("    数值验证（kp 固定 %.4f，只改 kd，D 用裸后向差分、不加滤波）："
          % kp)
    print()
    rows = []
    for ratio in (0.5, 0.9, 1.0, 1.1, 2.0, 4.0):
        kd = ratio * J
        c = PID(kp=kp, ki=0.0, kd=kd, u_max=None, d_source="diff")
        log = simulate(c, T=1.0, dt=1.0 / 2000, plant=Plant(J=J),
                       r_fn=lambda t: r_step if t > 0.2 else 0.0)
        finite = np.all(np.isfinite(log["x"]))
        peak = float(np.max(np.abs(log["x"]))) if finite else float("inf")
        ok = finite and peak < 10.0
        rows.append(["%.2f" % ratio, "%.5f" % kd,
                     "%.3e" % peak if finite else "inf",
                     "稳定" if ok else "★ 发散"])
    tab(rows, ["kd/J", "kd", "max|x| (rad)", "结果（kd < J ?）"])

    print("""    ★★ 分界线精确落在 kd/J = 1 上，和 Step 6 的推导一致。

    ★ 再和 ① 对照一下：① 里 ζ = 0.7 用的 kd = %.4f，是 J 的 %.0f 倍 ——
      在「速度直测」（实现 A）下跑得好好的（超调 4.6%%），
      换成「微分型」（实现 B）就会立刻发散。
      ★★ 同一个 kd，两种实现，一个是标准答案，一个是灾难。
      这就是「D 项怎么实现」这件事的分量。
""" % (2 * 0.7 * math.sqrt(J * kp), 2 * 0.7 * math.sqrt(J * kp) / J))

    print("""    ★★ 现在看真实飞控是怎么做的。

      PX4 的角加速度（= 速率环 D 项用的那个量）是
      【先差分、再滤波】的（VehicleAngularVelocity.cpp:770-782 原文）：

          angular_acceleration = (data[n] − _angular_velocity_raw_prev(axis)) * inverse_dt_s;
          angular_acceleration_filtered = _lp_filter_acceleration[axis].update(angular_acceleration);

      滤波器的参数（imu_gyro_parameters.c:160）：

          PARAM_DEFINE_FLOAT(IMU_DGYRO_CUTOFF, 30.0f);

      参数文档里写得很直白（imu_gyro_parameters.c:143-151）：

          被控对象是「角速度的导数」，也就是速率环的 D 项。
          因为 D 项用的是角速度的导数，它对噪声最敏感。
          所以加这个滤波器可以让你把 IMU_GYRO_CUTOFF 调高，
          从而降低控制延迟、并允许把 P 增益调大。

      ★ 注意它没说出来的那半句：**滤波器把 kd < J 这个稳定性上界
        一起推高了**。下面用第 8 课的混控矩阵把这件事量化。
""")

    # 归一化力矩 1.0 对应多少 N·m —— 用第 8 课的混控矩阵算
    tau_scale = _norm_torque_scale()
    kd_def_param = 0.003        # MC_ROLLRATE_D 默认值
    kd_max_param = 0.01         # MC_ROLLRATE_D 的 @max
    limit_norm = J / tau_scale
    print("    用第 8 课的混控矩阵算「归一化力矩 1.0」≈ 多少 N·m：")
    print("        1 kg、臂长 0.25 m、推重比 2:1、J = %.4f kg·m²" % J)
    print("        ⇒ 归一化力矩 1.0 ≈ %.4f N·m（roll 轴，本脚本实测）"
          % tau_scale)
    print("        ⇒ 无滤波时的稳定上界 kd < J 折算成【归一化单位】：")
    print("              J / τ_scale = %.5f" % limit_norm)
    print()
    rows = [
        ["MC_ROLLRATE_D @default", "%.4f" % kd_def_param,
         "%.4f" % (kd_def_param * tau_scale),
         "%.3f" % (kd_def_param * tau_scale / J),
         "稳定" if kd_def_param * tau_scale < J else "★ 超界"],
        ["MC_ROLLRATE_D @max", "%.4f" % kd_max_param,
         "%.4f" % (kd_max_param * tau_scale),
         "%.3f" % (kd_max_param * tau_scale / J),
         "稳定" if kd_max_param * tau_scale < J else "★ 超界（必须靠滤波）"],
    ]
    tab(rows, ["PX4 参数", "参数值", "等效 kd (N·m·s/rad)",
               "kd/J", "无滤波时"])
    print("""    ★ 换算方向别搞反：D 增益的单位是【归一化力矩 / (rad/s²)】，
      而「归一化力矩 1.0 = %.4f N·m」，所以

          等效 kd [N·m·s/rad] = MC_ROLLRATE_D × τ_scale        （乘，不是除！）

    ★★★ 这就是本节最实用的一个结论：

      **PX4 允许把 MC_ROLLRATE_D 调到 0.01（参数上限），
        折算成物理单位是 %.4f N·m·s/rad，是 kd < J 上界的 %.2f 倍 ——
        它在无滤波时【已经越过了稳定边界】。
        它之所以能开这么大，正是因为 IMU_DGYRO_CUTOFF 在保它。**

    ⇒ ★ 「改参数之前先看它的物理含义」不是一句空话：
      D 的上限不是拍脑袋定的，它和【转动惯量】【力矩量级】
      【D 项滤波器】三个东西绑在一起。

      ★ 而 PX4 的【默认值】 0.003（等效 kd/J = %.3f）是安全的 ——
        默认参数不需要靠滤波器救，这是个刻意留出的余量。
        ★★ 第 17 周实验 2 ③ 还会用这个数：届时会把 K_d/J = %.3f
          和「D 项在串级结构里的最优值」放在一起讨论。

    ⚠ 反过来说：如果你在代码里【绕过】了角加速度滤波器
      （比如自己在线程里对陀螺做裸差分喂给 D），
      你会撞上一个和 J 有关的稳定性墙 —— 而现象是
      「一加油门就高频啸叫 / 翻车」，很难往 D 上想。

    ★ 顺带说明：这里分析的是【一阶】后向差分。
      PX4 真正用的是「一阶差分 + 二阶 Butterworth 低通」
      （imu_gyro_parameters.c:145 原文写了
      「2nd order butterworth filter」）。二阶滚降更陡、相位损失更小，
      所以能承受更大的 kd。本脚本用一阶差分只是为了让推导干净 ——
      结论（滤波把上界推高）是一样的。
""" % (tau_scale, kd_max_param * tau_scale,
       (kd_max_param * tau_scale) / J,
       kd_def_param * tau_scale / J,
       kd_def_param * tau_scale / J))

    # ------------------------------------------------------------------
    sub("③ 加上常值扰动 d：P 的稳态误差出现了")

    print("""    现在让 d ≠ 0（重心偏移、某个电机装歪了……）。

        u = kp·(r − x) − kd·ẋ            J·ẍ = u + d

    稳态时 ẍ = 0、ẋ = 0，代入：

        0 = kp·(r − x_ss) − 0 + d
      ⇒ x_ss = r + d / kp
      ⇒ ★ 稳态误差  e_ss = r − x_ss = −d / kp

    ★★ 注意两件事：

      ① 误差和 kp 成【反比】⇒ 加大 P 能减小误差，但【永远不是零】
      ② 式子里面【没有 kd】⇒ ★ D 对稳态误差【一点用都没有】！

         为什么？因为稳态时 ẋ = 0，D 项  −kd·ẋ  恒等于 0。
         D 是「速度的反对者」，飞机停下来之后它就下班了。
""")

    d = 0.05          # 0.05 N·m 的常值扰动力矩（约为最大力矩的 6%）
    kd_good = 2 * 0.7 * math.sqrt(J * kp)          # ζ = 0.7
    print("    扰动 d = %.3f N·m，每行都按 ζ = 0.7 反算 kd（速度直测，无微分）"
          % d)
    print("    照 Δ(kp) 时 kd 也随之变化：kd = 2·0.7·sqrt(J·kp)")
    print()

    rows = []
    devs, e_rel = [], []
    for mult in (0.5, 1.0, 2.0, 5.0, 10.0, 20.0):
        kp_i = kp * mult
        kd_i = 2 * 0.7 * math.sqrt(J * kp_i)
        c = PID(kp=kp_i, ki=0.0, kd=kd_i, u_max=None, d_source="measured")
        log = simulate(c, T=5.0, dt=1.0 / 2000, plant=Plant(J=J, d=d),
                       r_fn=lambda t: r_step if t > 0.2 else 0.0)
        # ★ 超调要相对【实测终值】算，否则会把 d/kp 的稳态偏移算进超调
        ss, ov, rise, settle, peak = metrics(log, ref=final_value(log))
        dev = abs(ss - (-d / kp_i)) / (d / kp_i) * 100.0
        devs.append(dev)
        e_rel.append(abs(ss) / r_step * 100.0)
        rows.append(["%.4f" % kp_i,
                     "%.4f" % kd_i,
                     "%+.6f" % ss,
                     "%+.6f" % (-d / kp_i),
                     "%.2f%%" % dev,
                     "%.1f%%" % max(ov, 0.0)])

    print("    目标角 r = %.2f° = %.4f rad，仿真 5 s（阶跃在 0.2 s）"
          % (math.degrees(r_step), r_step))
    print()
    tab(rows, ["kp", "kd (ζ=0.7)", "e_ss 实测 (rad)", "e_ss = −d/kp (rad)",
               "相对偏差", "超调（相对实测终值）"])
    print("""    ★★ 第 3、4 列【逐行对上】（相对偏差最大 %.2f%%，只是尾均值的
       离散误差）。所以：

        「P 单独用会有稳态误差」不是经验，而是 0 = kp·(r−x_ss) + d
        这个代数方程的直接结果。

        ★ 注意最后一列：这张表里【每一行都配了合适的 kd（ζ=0.7）】，
          所以超调都在 4~5%% 的解析值附近 —— 但稳态误差该多大还是多大。

        ⇒ ★★ D 能修「振」，不能修「偏」。这是两个正交的问题。

    ★★ 再看第 5 列（稳态误差相对于目标角 r 的百分比）：
       kp 从 %.2f 加到 %.2f（20 倍），相对误差才从 %.1f%% 降到 %.1f%%。
       「加大 P 来消稳态误差」是一条收敛极慢的路 —— 因为 e_ss ∝ 1/kp。
       ★ 真正能把 e_ss 打到【精确的零】的，只有 I（下一节）。
""" % (max(devs), kp * 0.5, kp * 20, e_rel[0], e_rel[-1]))

    # D 对稳态误差完全无效的直接对照
    rows = []
    for kd_v in (0.10, 0.15, 0.20, kd_good, 0.35, 0.60):
        c = PID(kp=kp, ki=0.0, kd=kd_v, u_max=None, d_source="measured")
        log = simulate(c, T=5.0, dt=1.0 / 2000, plant=Plant(J=J, d=d),
                       r_fn=lambda t: r_step if t > 0.2 else 0.0)
        ss, ov, rise, settle, peak = metrics(log, ref=final_value(log))
        rows.append(["%.4f" % kd_v,
                     "%.3f" % zeta_from_kd(kd_v, kp, J),
                     "%+.7f" % ss,
                     "%.1f%%" % max(ov, 0.0),
                     "%.3f" % settle if math.isfinite(settle) else "—"])

    print("    ★ 直接对照：kp 固定在 %.4f，只改 kd，看 e_ss 会不会变"
          % kp)
    print()
    tab(rows, ["kd", "ζ", "e_ss (rad)", "超调", "2% 调节时间 (s)"])
    settles = [float(r[4]) for r in rows if r[4] != "—"]
    i_min = int(np.argmin(settles))
    print("""        ⇒ ★★ e_ss 一列【在所有有效数字上都没变】，全部是
          −d/kp = %.7f；而超调从 %.1f%% 一路降到 %.1f%%。
          2%% 调节时间则先降后升：%.3f s（kd=%.2f）→ %.3f s（kd 最小）
          → %.3f s（kd 最大）—— ★ 这就是「ζ 太小振荡、ζ 太大磨蹭」。

        ⇒ 「D 只修动态（振），不修静态（偏）」，这里是它的直接证据。
""" % (-d / kp, float(rows[0][3].rstrip("%")), float(rows[-1][3].rstrip("%")),
       settles[i_min], float(rows[i_min][0]), settles[0], settles[-1]))

    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 3, figsize=(16, 4.4))

            l0 = simulate(PID(kp=kp, ki=0.0, kd=0.0), T=3.0, dt=1.0 / 2000,
                          plant=Plant(J=J),
                          r_fn=lambda t: r_step if t > 0.2 else 0.0)
            axes[0].plot(l0["t"], np.degrees(l0["x"]),
                         label="P only  (zeta = 0)")
            axes[0].plot(l0["t"], np.degrees(l0["r"]), "k:", label="setpoint")

            for zt, lbl in ((0.7, "P+D  zeta = 0.7"),
                            (0.2, "P+D  zeta = 0.2")):
                cc = PID(kp=kp, ki=0.0, kd=2 * zt * math.sqrt(J * kp),
                         d_source="measured")
                ll = simulate(cc, T=3.0, dt=1.0 / 2000, plant=Plant(J=J),
                              r_fn=lambda t: r_step if t > 0.2 else 0.0)
                axes[0].plot(ll["t"], np.degrees(ll["x"]), label=lbl)
            axes[0].set_title("P gives stiffness, D gives damping")
            axes[0].set_xlabel("time (s)")
            axes[0].set_ylabel("angle (deg)")
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(fontsize=8)

            # ★ kd = J 附近的离散稳定性边界（D 用裸差分）
            for ratio, style in ((0.9, "-"), (1.1, "--")):
                cc = PID(kp=kp, ki=0.0, kd=ratio * J, u_max=20.0,
                         d_source="diff")
                ll = simulate(cc, T=0.4, dt=1.0 / 2000, plant=Plant(J=J),
                              r_fn=lambda t: r_step if t > 0.2 else 0.0)
                fin = np.all(np.isfinite(ll["x"]))
                y = (np.degrees(ll["x"]) if fin
                     else np.full_like(ll["x"], np.nan))
                axes[1].plot(ll["t"], y, style,
                             label="kd/J = %.1f%s"
                                   % (ratio, "" if fin else "  (diverged)"))
            axes[1].set_title("differentiated D: the kd < J wall")
            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("angle (deg)")
            axes[1].set_ylim(-60, 60)
            axes[1].grid(True, alpha=0.3)
            axes[1].legend(fontsize=8)

            ms = np.array([0.5, 1, 2, 5, 10, 20])
            errs = []
            for mult in ms:
                cc = PID(kp=kp * mult, ki=0.0,
                         kd=2 * 0.7 * math.sqrt(J * kp * mult),
                         d_source="measured")
                ll = simulate(cc, T=4.0, dt=1.0 / 2000, plant=Plant(J=J, d=d),
                              r_fn=lambda t: r_step if t > 0.2 else 0.0)
                errs.append(float(np.mean(ll["x"][-200:] - ll["r"][-200:])))
            axes[2].loglog(ms, np.abs(errs) * 1e3, "o-", label="measured")
            axes[2].loglog(ms, d / (kp * ms) * 1e3, "x--", label="d/kp")
            axes[2].set_title("P up -> error down, never zero")
            axes[2].set_xlabel("kp multiplier (zeta always 0.7)")
            axes[2].set_ylabel("steady error (mrad)")
            axes[2].grid(True, alpha=0.3, which="both")
            axes[2].legend(fontsize=8)

            fig.tight_layout()
            fn = os.path.join(HERE, "pid_lesson16_exp1.png")
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:                      # noqa: BLE001
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 实验 2：PI / PID 与积分饱和
# ===========================================================================

def exp2():
    head("实验 2  ★ 积分：怎么消掉稳态误差，以及它带来的麻烦")

    J = J_DEFAULT
    d = 0.02
    r_step = math.radians(10.0)
    kp = J * 20.0 ** 2
    kd = 2 * 0.7 * math.sqrt(J * kp)

    print("""
    §实验 1 的结论：只要有常值扰动，P（哪怕配上 D）就一定留稳态误差。

    ★ 真正的原因是：稳态时控制器必须输出一个【非零】的力矩去顶住 d，
      但  u = kp·(r−x) − kd·ẋ  在 e = 0、ẋ = 0 时只能输出 0。

    ⇒ 我们需要一个「误差为零时仍然可以非零」的项。

    ★★ 这就是积分项：

        u = kp·e + ki·∫e dt − kd·ẋ

      稳态时 e = 0，但 ∫e dt 可以停在任意非零常数上 ——
      它【记住了】历史上积累的误差，并且把这个记忆当成输出使出去。
""")

    # ------------------------------------------------------------------
    sub("① 只在双积分器上用 PI —— 它会发散")

    print("""    Ｊ·ẍ = u + d，u = kp·e + ki·∫e dt       （e = r − x）

    对时间求导：  J·ẍ' = kp·ė + ki·e
      ⇒  J·x''' + kp·ẋ + ki·x = ki·r + (d 的导数项)

    特征方程：    J·s³ + 0·s² + kp·s + ki = 0

    ★ 注意 s² 的系数是【0】—— 用 Routh 判据：
      s³ 行：[ J ,  kp ]
      s² 行：[ 0 ,  ki ]   ← 首元为 0！
      ⇒ 用 ε 法，s¹ 行首元 = (ε·kp − J·ki)/ε → −∞
      ⇒ 符号变化 ⇒ ★★ 不稳定。

    ⇒ 结论：**PI 控制不了双积分器**。必须再加 D（或者外面套速率环）。
""")

    for ki in (30.0, 60.0):
        c = PID(kp=kp, ki=ki, kd=0.0, u_max=None)
        log = simulate(c, T=3.0, dt=1.0 / 2000, plant=Plant(J=J, d=d),
                       r_fn=lambda t: r_step if t > 0.2 else 0.0)
        print("    ki = %-6.1f ⇒ t=3s 时的角度 = %9.3f °，"
              "角速率 = %9.3f rad/s   %s"
              % (ki, math.degrees(log["x"][-1]), log["xd"][-1],
                 "★ 发散了" if abs(log["x"][-1]) > 1.0 else ""))

    print("""
    ★ 所以飞控里【不存在】「直接在角度上做 PI」这种设计。
      正确的做法有两条，本章两条都会讲：
        · 加 D → PID（本节 ②）
        · 外面套速率环 → 级联 PID（实验 4，也是真机的做法）
""")

    sub("② PID：D 把双积分器救回来")

    print("""    特征方程变成：  J·s³ + kd·s² + kp·s + ki = 0

    Routh 判据（a₃=kJ, a₂=kd, a₁=kp, a₀=ki > 0 且 a₂a₁ > a₃a₀）：

        ★★  kd·kp > J·ki        即  ki < kd·kp / J

    这是一个【精确的】稳定条件，不是经验。下面数值验证。
""")

    ki_limit_exact = kd * kp / J
    ki_good = kp * 20.0 / 5.0        # 「积分比比例慢 5 倍」的经验规则

    print("    kp = %.4f，kd = %.4f，J = %.4f" % (kp, kd, J))
    print("    ⇒ 稳定上界  ki < kd·kp/J = %.3f" % ki_limit_exact)
    print("    ⇒ 规则取    ki = kp·ωn/5 = %.3f  （是上界的 %.1f%%）"
          % (ki_good, 100 * ki_good / ki_limit_exact))
    print()

    rows = []
    amps = []
    n_bad = 0
    for ki in (0.0, ki_good, 2 * ki_good, 5 * ki_good,
               ki_limit_exact * 0.99, ki_limit_exact * 1.01):
        c = PID(kp=kp, ki=ki, kd=kd, u_max=None)
        log = simulate(c, T=6.0, dt=1.0 / 2000, plant=Plant(J=J, d=d),
                       r_fn=lambda t: r_step if t > 0.2 else 0.0)
        ss, ov, rise, settle, peak = metrics(log)
        ok_finite = bool(np.all(np.isfinite(log["x"])))
        xref = final_value(log) if ok_finite else 0.0
        amp_late = env_peak(log, 5.0, 6.0, xref) if ok_finite else float("inf")
        amp_mid = env_peak(log, 3.0, 4.0, xref) if ok_finite else float("inf")
        if not ok_finite:
            verdict, mark = "★ 发散", 0
        elif amp_late < 0.01 * r_step:
            verdict, mark = "稳定（已收敛）", 2
        elif amp_late > 1.2 * amp_mid:
            verdict, mark = "★ 不收敛（在长大）", 1
        else:
            verdict, mark = "★ 不收敛（等幅）", 1
        n_bad += (mark < 2)
        amps.append(amp_late)
        rows.append(["%.2f" % ki,
                     "%.1f%%" % (100 * ki / ki_limit_exact),
                     "%.5f" % ss if mark == 2 else "—",
                     "%.1f%%" % max(ov, 0.0) if mark == 2 else "—",
                     "%.2f" % settle if (math.isfinite(settle)
                                         and mark == 2) else "—",
                     "%.5f" % amp_late,
                     verdict])

    print("    Routh 边界 ki* = kd·kp/J = %.3f 的两侧：" % ki_limit_exact)
    print()
    tab(rows, ["ki", "ki / ki*", "e_ss (rad)", "超调", "2% 调节时间 (s)",
               "末段振幅 (rad)", "结果"])
    print("""    ★★ 三件事同时被证实：

      ① ki = 0 时 e_ss = %.5f rad（就是实验 1 的 −d/kp）
      ② 只要 ki 明显小于 ki*，e_ss 立刻掉到【机器精度】级别
         （1e-5 量级）—— 这就是积分的威力
      ③ ★★ 而 0.99×ki* 和 1.01×ki* 这一对【都】不收敛，
         末段振幅分别是 %.4f rad 和 %.4f rad（目标角的 %.0f%% 和 %.0f%%）。
         101%% 那个振幅更大，说明它确实在长大。

          这恰恰说明了 Routh 判据的价值：
          **离边界越近，收敛/发散越慢，仿真跑到天荒地老也看不出差别。
            但判据一行代数就告诉你边界在 ki* = kd·kp/J。**
          ★ 这也是「试凑调参」最危险的地方：边界附近看起来「还行」，
            实际上是临界稳定的，飞起来遇到一点延迟就炸。

    ★ 顺带看清「积分比比例慢」这条经验的来历：
      它其实是 Routh 条件 ki < kd·kp/J 的一个保守版本 ——
      工程上取 ki ≈ kp·ωn/5 = %.1f，只有边界的 %.0f%%，
      留了足够余量给参数误差、延迟、以及后面要讲的级联。
""" % (metrics(simulate(PID(kp=kp, ki=0.0, kd=kd, u_max=None), T=6.0,
                      dt=1.0 / 2000, plant=Plant(J=J, d=d),
                      r_fn=lambda t: r_step if t > 0.2 else 0.0))[0],
       amps[-2], amps[-1],
       100.0 * amps[-2] / r_step, 100.0 * amps[-1] / r_step,
       ki_good, 100.0 * ki_good / ki_limit_exact))

    # 积分器的终值
    c = PID(kp=kp, ki=ki_good, kd=kd, u_max=None)
    log = simulate(c, T=4.0, dt=1.0 / 2000, plant=Plant(J=J, d=d),
                   r_fn=lambda t: r_step if t > 0.2 else 0.0)
    print("    ★★ 再看积分项的终值：")
    print("        扰动 d            = %+.5f N·m" % d)
    print("        I 项终值实测      = %+.5f N·m   ← 符号相反、大小相等"
          % log["I"][-1])
    print("        稳态总输出 u      = %+.6f N·m" % log["u"][-1])
    print("""
        ⇒ ★ 终态时误差为零，但积分器停在一个【非零】值上，
             它输出的力矩【正好等于 −d】——
             积分器「学会」了顶住这个扰动。

        ★★ 这就是「I 项能消稳态误差」的机制：
           它不是把误差变小，而是【记住了一个能抵消扰动的偏置】。
""")

    # ------------------------------------------------------------------
    sub("③ ★ 代价：积分饱和（integral windup）")

    print("""    现在给执行机构加上【真实的限幅】：电机能给的差动力矩是有限的。

        |u| ≤ u_max

    ★ 一旦有 u_max，P 和 D 立刻「到顶」，但【积分器不会】——
      它还在傻乎乎地累加：I ← I + ki·e·dt。
      等误差终于反号了，I 里已经存了一大笔「欠账」，
      必须靠长时间的反向误差才能泄掉。

    ★★ 这就是 integral windup（积分饱和）：
       **执行机构饱和了，但积分器不知道。**

    看一个具体场景：

        · 目标从 0 跳到 10°，只保持 0.20 s，然后立刻回到 0°
        · u_max = 0.040 N·m（很弱的执行机构，为了把现象放大）
        · 回撤时飞机还【远没有】飞到目标 —— 正是这时候积分欠账最多

    比较五种做法：
""")

    umax = 0.040
    target_big = math.radians(10.0)
    ki_windup = ki_good
    t_rev = 0.20 + 0.20        # 目标回零的时刻

    def r_big(t):
        return target_big if 0.2 <= t < t_rev else 0.0

    variants = [
        ("① 自由积分 + 无 i_limit", None, 1.0e9),
        ("② 自由积分 + 有 i_limit", None, 2.0 * umax),
        ("③ clamp（PX4 pid.cpp）", "clamp", 2.0 * umax),
        ("④ freeze（AP AC_PID.cpp）", "freeze", 2.0 * umax),
        ("⑤ backcalc（反算）", "backcalc", 2.0 * umax),
    ]
    rows = []
    logs = {}
    over_pct, imax = [], []
    for name, aw, ilim in variants:
        c = PID(kp=kp, ki=ki_windup, kd=kd, u_max=umax,
                i_limit=ilim, anti_windup=aw, Tt=0.05, d_source="measured")
        log = simulate(c, T=2.5, dt=1.0 / 2000, plant=Plant(J=J, d=0.0),
                       r_fn=r_big)
        logs[name] = log
        # 回撤后的反向峰值：目标回零后飞机冲过头多少
        seg = select(log, log["t"] > t_rev)
        peak_back = float(np.max(seg["x"]))
        # 回撤后多久才重新回到 0 附近（±1°）
        outside = np.abs(seg["x"]) > math.radians(1.0)
        nz = np.nonzero(outside)[0]
        recov = float(seg["t"][nz[-1]]) - t_rev if len(nz) else 0.0
        rows.append([name,
                     "%.3f" % float(np.max(np.abs(log["I"]))),
                     "%.2f" % math.degrees(peak_back),
                     "%.0f%%" % (peak_back / target_big * 100.0),
                     "%.3f" % recov])
        over_pct.append(peak_back / target_big * 100.0)
        imax.append(float(np.max(np.abs(log["I"]))))

    print("    u_max = %.3f N·m，i_limit = %.3f N·m，目标 %.0f° 保持 0.20 s"
          % (umax, 2.0 * umax, math.degrees(target_big)))
    print()
    tab(rows, ["策略", "max|I| (N·m)", "回撤后最大角 (°)", "相对目标",
               "回撤后恢复时间 (s)"])

    print("""    ★★ 读表方法：

      · 「max|I|」= 积分器冲到多高。① 和 ② 的区别就是 i_limit 的作用：
        ① 冲到 %.1f N·m（是 u_max 的 %.0f 倍！），② 被钳在 %.3f
      · 「回撤后最大角 / 相对目标」= 目标回零后飞机反向冲过头多少 ——
        这就是积分饱和最典型的症状：**反向超调**，
        因为积分器里存的欠账必须靠反向误差慢慢泄掉
      · 「恢复时间」= 从回撤到重新稳定在 ±1° 内用了多久

    ★★ 三点结论：

      ① **i_limit 是必要的第一道防线**，但它【不够】：
         ② 的过冲 %.0f%% 仍然明显比 ③④⑤ 大一截 ——
         因为钳在限值上的积分依然是「满的」，退饱和还是要靠等。
      ② ③④⑤ 三种真正的抗饱和策略把过冲从 %.0f%% 压到 %.0f%%。
         ★ 但【压不到 0】—— 因为剩下那一部分里有一大半是
         **执行机构饱和本身**造成的（回撤瞬间飞机还有角速度，
         而减速能力只有 u_max），抗饱和只能消掉「积分欠账」那部分。
      ③ ⑤ backcalc 恢复最快，因为它【主动】把超出量灌回积分器，
         而不是被动地等误差反号。
""" % (imax[0], imax[0] / umax, umax * 2.0, over_pct[1],
       over_pct[0], over_pct[2]))

    print("""    ★ 三种抗饱和的取舍（源码里的差异）：

      ┌───────────┬──────────────────────────────┬──────────────────────┐
      │ clamp     │ 输出超出范围就【不积分】      │ 简单，但会「卡住」：  │
      │           │ PX4 pid.cpp:147-158          │ 需要积分出力时进不去  │
      ├───────────┼──────────────────────────────┼──────────────────────┤
      │ freeze    │ 饱和时只允许让 |u| 变小的方向 │ 保守，退饱和最快      │
      │           │ AP AC_PID.cpp:324-336        │ 但动态响应略慢        │
      ├───────────┼──────────────────────────────┼──────────────────────┤
      │ backcalc  │ (u_sat−u_unsat)/Tt 反馈回 I  │ 平滑，Tt 需调         │
      └───────────┴──────────────────────────────┴──────────────────────┘

    ★ PX4 的速率环用的其实是【第四种】，也是最有意思的一种
      （rate_control.cpp:88-118）：

        ① 控制分配器（Control Allocation）告诉它哪个轴饱和了
        ② 饱和方向上的误差被【钳到 0】：rate_error = min(rate_error, 0)
        ③ 再乘一个 i_factor = 1 − (e/400°)²

      ⇒ 它不只是「不积分」，而是【把误差本身做了方向性钳位】——
        这比单纯的积分限幅更贴近「为什么饱和」这个原因。
""")

    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
            for name, _aw, _il in variants:
                lg = logs[name]
                axes[0].plot(lg["t"], np.degrees(lg["x"]), label=name)
                axes[1].plot(lg["t"], lg["I"], label=name)
            _k0 = variants[0][0]           # 「① 自由积分 + 无 i_limit」
            axes[0].plot(logs[_k0]["t"],
                         np.degrees(logs[_k0]["r"]), "k:", label="setpoint")
            axes[0].set_title("integral windup: overshoot on reversal")
            axes[0].set_xlabel("time (s)")
            axes[0].set_ylabel("angle (deg)")
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(fontsize=8)
            axes[1].set_title("integral term")
            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("I term (Nm)")
            axes[1].grid(True, alpha=0.3)
            axes[1].legend(fontsize=8)
            fig.tight_layout()
            fn = os.path.join(HERE, "pid_lesson16_exp2.png")
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:                      # noqa: BLE001
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 实验 3：D 项与噪声
# ===========================================================================

def exp3():
    head("实验 3  ★ PID 的 D 项：它其实在预测，以及它为什么怕噪声")

    J = J_DEFAULT
    kp = J * 20.0 ** 2
    r_step = math.radians(10.0)

    print("""
    §实验 1 说 D 提供阻尼。这一节要说清两件事：

      ① D 项在数学上到底在干什么？（它是一次【预测】）
      ② 为什么真机里 D 项几乎总是「作用在测量值上」并且要滤波？
""")

    # ------------------------------------------------------------------
    sub("① D 项在预测什么")

    print("""    把 D 项单独看。控制器在 t 时刻算：

        u_D(t) = −kd·ẋ(t)

    如果只用 P：  u = kp·e(t) = kp·(r − x(t))
    问题是：它只看【现在】。现在是 0，不代表下一瞬间也是 0。

    ★ 一阶泰勒展开告诉我们「下一瞬间会在哪」：

        x(t + Δ) ≈ x(t) + ẋ(t)·Δ

    ⇒ 把 P 写成「预测的误差」：

        e(t+Δ) ≈ e(t) − ẋ(t)·Δ
        kp·e(t+Δ) = kp·e(t) − kp·Δ·ẋ(t)
                  = kp·e(t) + (−kp·Δ)·ẋ(t)
                                  └──────┬──────┘
                                   这就是 D 项！

    ★★ 所以 D 项 = **用 kd = kp·Δ 去抵消 Δ 秒之后的误差**。
       它不是「阻尼」这个词的字面意思，而是**一个提前量**。

    ⇒ 这就解释了实验 1 的表：kd 越大 → 提前量越大 → 越「刹得住」。
""")

    # ------------------------------------------------------------------
    sub("② D 项的噪声：实测")

    print("""    但 D 项要的是 ẋ 的【变化率】。真机上这个量从哪来？—— 陀螺仪。

    ★ 关键区别（实验 1 ② 已经说过）：
        · 如果 D 作用在【直测的 ẋ】上（d_source='measured'）——
          它就是把陀螺读数乘个系数，噪声只被【线性】放大。
        · 如果 D 作用在【微分出来的 ẋ】上（d_source='diff'）——
          也就是 PX4 速率环那种做法 —— 噪声会被【微分】放大。

    ★★ 微分为什麼放大噪声？把后向差分写成频域形式：

        D(z) = (1 − z⁻¹) / dt      ⇒     |D(jω)| = 2·sin(ω·dt/2) / dt

      在低频（ω·dt ≪ 1）：|D| ≈ ω            ← 正常的「求导」
      到奈奎斯特频率（ω = π/dt）：|D| = 2/dt  ← ★ 放大到 2/dt 倍！

      dt = 0.5 ms 时 2/dt = 4000。
      ★★ 也就是说：**陀螺上 0.01 rad/s 的噪声，裸差分后变成
         40 rad/s 的「角加速度」** —— 再乘 kd 就是从电机里吼出来。

    ★ 下面做一个「安静悬停」的仿真：
        · 目标角 = 0（就是不动）
        · 陀螺噪声：σ = 0.01 rad/s（约 0.57 °/s，常见 MEMS 陀螺的量级）
        · 看 D 项输出的 RMS 随 kd 怎么变
""")

    sigma = 0.01
    dt_sim = 1.0 / 2000
    filt_ref = 30.0        # 对应 PX4 的 IMU_DGYRO_CUTOFF 默认值

    def rms_d_of(kd, src, fc):
        c = PID(kp=kp, ki=0.0, kd=kd, u_max=None, d_source=src,
                d_lpf_hz=fc)
        lg = simulate(c, T=2.0, dt=dt_sim, plant=Plant(J=J),
                      r_fn=lambda t: 0.0, sensor_noise=sigma, seed=7)
        d = lg["D"][200:]
        if not np.all(np.isfinite(d)):
            return float("nan")
        return float(np.sqrt(np.mean(d ** 2)))

    rows = []
    r_raw_first = float("nan")
    r_filt_last = float("nan")
    kd_first = 0.005
    for kd in (0.005, 0.010, 0.020, 0.040, 0.080):
        r_meas = rms_d_of(kd, "measured", None)
        r_filt = rms_d_of(kd, "diff", filt_ref)
        r_raw = rms_d_of(kd, "diff", None)
        if kd == 0.005:
            r_raw_first = r_raw
        r_filt_last = r_filt
        rows.append(["%.3f" % kd, "%.3f" % (kd / J),
                     "%.5f" % r_meas,
                     "%.5f" % r_filt,
                     "%.4f" % r_raw if math.isfinite(r_raw)
                     else "★ 发散（nan）",
                     "%.1f" % (r_meas / kd),
                     "%.2f" % (r_filt / kd)])

    print("    陀螺噪声 σ = %.3f rad/s，采样 dt = 1/2000 s，"
          "D 项滤波器 %.0f Hz（= PX4 IMU_DGYRO_CUTOFF 默认值）"
          % (sigma, filt_ref))
    print()
    tab(rows, ["kd", "kd/J", "RMS(D) 速度直测 (N·m)",
               "RMS(D) 微分+滤波 (N·m)", "RMS(D) 微分不滤 (N·m)",
               "直测: RMS(D)/kd", "微分: RMS(D)/kd"])
    print("""    ★★ 读表（这张表信息量很大，逐列看）：

      · 第 6 列 = RMS(D)/kd = %.3f = 陀螺噪声 σ 本身。
        直测时 D 项就是「把陀螺读数乘 kd」，噪声【线性】进来。

      · ★★ 第 7 列 = RMS(D)/kd ≈ %.2f，是 σ 的 %.0f 倍！
        也就是说：**微分型 D 项看到的「等效角速度噪声」被放大了 %.0f 倍。**
        加了一个 30 Hz 低通之后仍然如此 —— 因为滤波器在高频段的
        衰减能力有限，而差分把噪声搬到了高频。

      · ★★★ 第 5 列：**不加滤波的微分会在 kd ≳ J 时直接发散**
        （第一行 kd = 0.005 < J = %.3f 还活着，RMS 是 %.2f ——
         注意这是 kd=0.005 时的值，比值约 %.0f，正好是 2/dt = %.0f
         这个理论上界的一个分数）。

        ★ 所以「D 项滤波器」不是可选项，它是【让微分型 D 项能用】
          的必要条件（实验 1 ② 的 kd < J 墙）。

    ★ 工程上的三个对策（两家飞控都做了）：

      对策 A：D 项加【低通滤波】—— 见 ③
              ArduPilot 的参数叫 ATC_RAT_RLL_FLTD（AC_PID 的 FLTD）
              PX4 有 IMU_DGYRO_CUTOFF（对角加速度专门再滤一次）
      对策 B：★ D 项作用在【测量值】而不是【误差】上 —— 见 ④
      对策 C：★ 干脆【不用】微分 —— 让外面套一层速率环来提供阻尼
              （实验 4 的级联 PID，也是真机的做法）

    下面分别量化 A 和 B。
""" % (sigma, r_filt_last / 0.080, (r_filt_last / 0.080) / sigma,
       (r_filt_last / 0.080) / sigma, J, r_raw_first,
       r_raw_first / kd_first, 2.0 / dt_sim))

    # ------------------------------------------------------------------
    sub("③ 对策 A：低通滤波的代价")

    kd_demo = 0.06
    wn_here = wn_from_kp(kp, J)
    rows = []
    for fc in (None, 500.0, 100.0, 30.0, 10.0, 5.0):
        # 噪声衰减：一阶低通对白噪声的 RMS 增益 = sqrt(α/(2−α))
        if fc is None:
            alpha, gain_n, phase, mag = 1.0, 1.0, 0.0, 1.0
        else:
            x = 2.0 * math.pi * fc * dt_sim
            alpha = x / (1.0 + x)
            gain_n = math.sqrt(alpha / (2.0 - alpha))
            # D 项在 ωn 处的相位滞后 / 幅值衰减
            m = 2.0 * math.pi * fc
            phase = math.degrees(math.atan(wn_here / m))
            mag = 1.0 / math.sqrt(1.0 + (wn_here / m) ** 2)
        c = PID(kp=kp, ki=0.0, kd=kd_demo, u_max=None, d_lpf_hz=fc,
                d_source="diff")
        lg = simulate(c, T=2.0, dt=dt_sim, plant=Plant(J=J),
                      r_fn=lambda t: 0.0, sensor_noise=sigma, seed=7)
        d = lg["D"][200:]
        rms_d = (float(np.sqrt(np.mean(d ** 2)))
                 if np.all(np.isfinite(d)) else float("nan"))
        rows.append(["不滤（kd>J 会发散）" if fc is None else "%.0f Hz" % fc,
                     "%.3f" % alpha,
                     fnum(rms_d, "%.5f"),
                     "%.3f" % gain_n,
                     "%.1f°" % phase,
                     "%.3f" % mag])

    print("    kd = %.3f（= %.1f·J，超过 kd < J ⇒ 必须靠滤波才稳定）"
          % (kd_demo, kd_demo / J))
    print("    陀螺噪声 σ = %.3f rad/s，dt = 1/2000 s，ωn = %.0f rad/s"
          % (sigma, wn_here))
    print()
    tab(rows, ["D 项低通截止", "α", "RMS(D) 实测 (N·m)",
               "白噪声 RMS 增益", "在 ωn 处的相位滞后", "在 ωn 处的幅值"])
    print("""    ★★ 读表（两个方向的趋势同时出现）：

      · 截止频率越低 → α 越小 → 白噪声 RMS 增益越小
        ⇒ **RMS(D) 实测和解析增益是同一个趋势** —— 噪声被压下去了。

      · ★ 但同时「在 ωn 处的相位滞后」越来越大：
        截止频率越低，D 项在它最该出力的频率上就越【晚】，
        它提供的阻尼就越打折扣（幅值一列也一起掉）。
        最下面那一行，D 项只剩 %.0f%% 的幅值 + %.0f° 的滞后。

    ⇒ ★★ 这就是控制里最经典的 **噪声 ↔ 相位（带宽）权衡**：
      **你不可能既让 D 项干净、又让它反应快。**
      滤波器每多压一点噪声，就要多交一份相位。

    ★ 定量地说：一阶低通在频率 f 处的滞后是 atan(f/fc)。
      让 D 项起作用的前提是 fc 【显著高于】回路带宽 ωn/(2π)，
      否则 D 项就在「拖后腿」。
      这就是 FLTD / IMU_DGYRO_CUTOFF 的整定原则：
      **在噪声可接受的前提下尽量高**，而不是「越低越干净越好」。
      第 4 课会用 Bode 图把这个权衡换算成相位裕度的数字。
""" % (100.0 * mag, phase))


    # ------------------------------------------------------------------
    sub("④ ★★ 对策 B：D 作用在测量值上（derivative on measurement）")

    print("""    这是本节最重要的一条工程实践。

    ★ 两种写法：

        作用在误差上：  D = kd · de/dt = kd · (dr/dt − dx/dt)
        作用在测量上：  D = kd · (−dx/dt)                  ← 少了 dr/dt

    ★ 平时两者几乎一样（因为 dr/dt 通常为 0）。
      ★★ 但当【设定值突变】时，两者天差地别：

        r 从 0 跳到 10° 的那一瞬间，dr/dt = ∞（理想阶跃）
        ⇒ 作用在误差上的 D 项会吐出一个【无穷大的尖峰】——
          工程上叫 **微分冲击（derivative kick）**。

      真实的飞控里，这个尖峰要么被限幅吃掉（浪费了控制量），
      要么真的冲到电机上一次 —— 结果就是「打杆瞬间抖一下」。

    ★ PX4 的做法（rate_control.cpp:78）：

        torque = P·rate_error + I − D·angular_accel + FF·rate_sp

      注意它减的是 **angular_accel（角加速度）**，
      也就是【测量值】的导数（MulticopterRateControl.cpp:133 里
      就是 vehicle_angular_velocity.xyz_derivative）。
      ⇒ rate_sp 完全不出现在 D 项里 ⇒ 设定值突变时 D 项【零反应】。

    ★ ArduPilot 走的是另一条路（AC_PID.cpp:248-253）：
      D 仍然作用在误差上，但【先把设定值低通滤波】（FLTT 参数）：

        _target += get_filt_T_alpha(dt) * (target − _target);
        derivative = (_error − error_last) / dt;

      ⇒ 设定值变成平滑斜坡 ⇒ dr/dt 有界 ⇒ 尖峰被压掉了。

    ★★ 两条路等价吗？不完全。PX4 的做法对测量噪声更友好；
      ArduPilot 的做法保留了「跟着设定值变化」的能力。
      但【两者的目的相同】：让 D 项在设定值跳变时闭嘴。

    下面量化这个尖峰。
""")

    rows = []
    peaks = []
    for name, dom in (("★ 作用在误差上", False), ("作用在测量上", True)):
        c = PID(kp=kp, ki=0.0, kd=kd_demo, u_max=5.0, d_source="diff",
                d_lpf_hz=30.0, d_on_measurement=dom)
        log = simulate(c, T=1.0, dt=1.0 / 2000, plant=Plant(J=J),
                       r_fn=lambda t: r_step if t > 0.5 else 0.0)
        peak_d = float(np.max(np.abs(log["D"])))
        peak_u = float(np.max(np.abs(log["u_unsat"])))
        peaks.append((peak_d, peak_u))
        # 第 4 列 = |D| 峰值 / kd = 「能造出这么大 D 的等效角速度」
        #           ★ 这一列和 |D| 峰值只差一个常数 kd，但它把
        #             「D 项到底看到了什么」直接翻译成了物理量
        rows.append([name, "%.4f" % peak_d, "%.4f" % peak_u,
                     "%.1f" % (peak_d / kd_demo)])

    print("    在 t = 0.5 s 给一个 10° 的设定值阶跃，"
          "kd = %.3f，D 项 30 Hz 低通，dt = 0.5 ms" % kd_demo)
    print()
    tab(rows, ["D 项作用在", "|D| 峰值 (N·m)", "|u_unsat| 峰值 (N·m)",
               "尖峰等效速率 (rad/s)"])
    print("""    ★★ 先看一个纯代数的数（不依赖仿真）：

        设定值在【一个采样周期】里跳了 Δr = %.4f rad，
        所以 dr/dt 的【瞬时值】= Δr/dt = %.0f rad/s。
        若不加滤波，D 项吐出的尖峰 = kd·(Δr/dt) = **%.2f N·m** ——
        是本实验里 u_max = 5 N·m 的 %.1f 倍。

      ★ 上表第 1 行实测 %.2f N·m 而不是 %.2f，是因为 30 Hz 低通
        把这个只有一拍宽的脉冲「摊平」到了好几个采样点上 ——
        ★★ 滤波器在这里【顺便】当了限幅器。这也是它的价值之一。

      · 作用在【误差】上：就算摊平了，尖峰仍有 %.2f N·m，
        等效速率 %.1f rad/s
        （= 只有「角速度瞬时跳到 %.1f rad/s」才能造出这么大的 D）
      · 作用在【测量】上：D 项只看得到真实的角速度，
        尖峰 %.3f N·m，等效速率 %.1f rad/s ——
        只有前者的 1/%.1f，
        而且这一份是「飞机真的在转」引起的，不是设定值跳变引起的。

    ★★ 所以「D 项作用在测量值上」不是一个风格问题，
      而是**设定值跳变时 D 项该不该有反应**这个物理问题。

      用教科书上的说法：微分冲击（derivative kick）
      = 控制器对「命令」而不是对「状态」做出的过激反应。

    ⇒ ★ 这条实践有多普遍？看 PX4 的老 PID（pid.cpp:125-138）就明白了：
      它专门定义了一个模式：

          PID_MODE_DERIVATIV_CALC_NO_SP     // NO_SP = no setpoint
              d = (−val − error_previous) / dt;

      「微分，但不要设定值」—— 这就是 derivative on measurement
      被写进枚举类型里的样子。

""" % (r_step, r_step / (1.0 / 2000),
       kd_demo * r_step / (1.0 / 2000),
       kd_demo * r_step / (1.0 / 2000) / 5.0,
       peaks[0][0], kd_demo * r_step / (1.0 / 2000),
       peaks[0][0], peaks[0][0] / kd_demo, peaks[0][0] / kd_demo,
       peaks[1][0], peaks[1][0] / kd_demo,
       peaks[0][0] / max(peaks[1][0], 1e-9)))
    print("""
    ⚠ 注意：作用在测量值上也有代价 ——
      设定值快速变化时，D 项【不提供】任何提前量，
      跟踪快速指令的能力会略差。所以 PX4 另外提供了 FF 项
      （MC_ROLLRATE_FF）来专门负责「跟着设定值走」。
      这就是「D 管阻尼、FF 管跟踪」的分工。
""")

    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 3, figsize=(16, 4.4))
            for dom, lbl in ((False, "D on error"), (True, "D on measurement")):
                c = PID(kp=kp, ki=0.0, kd=0.06, u_max=5.0,
                        d_source="diff", d_on_measurement=dom)
                lg = simulate(c, T=1.0, dt=1.0 / 2000, plant=Plant(J=J),
                              r_fn=lambda t: r_step if t > 0.5 else 0.0)
                axes[0].plot(lg["t"], lg["D"], label=lbl)
                axes[1].plot(lg["t"], np.degrees(lg["x"]), label=lbl)
            axes[0].set_title("D term at a setpoint step")
            axes[0].set_xlabel("time (s)")
            axes[0].set_ylabel("D term (Nm)")
            axes[0].set_ylim(-1.0, 1.0)
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(fontsize=8)

            axes[1].plot(lg["t"], np.degrees(lg["r"]), "k:", label="setpoint")
            axes[1].set_title("angle response")
            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("angle (deg)")
            axes[1].grid(True, alpha=0.3)
            axes[1].legend(fontsize=8)

            fcs = [200.0, 100.0, 50.0, 30.0, 10.0, 5.0]
            noise_rms, phases = [], []
            for fc in fcs:
                c = PID(kp=kp, ki=0.0, kd=0.06, d_lpf_hz=fc,
                        d_source="diff")
                lg = simulate(c, T=2.0, dt=1.0 / 2000, plant=Plant(J=J),
                              r_fn=lambda t: 0.0, sensor_noise=sigma, seed=7)
                d = lg["D"][200:]
                noise_rms.append(float(np.sqrt(np.mean(d ** 2)))
                                 if np.all(np.isfinite(d)) else np.nan)
                phases.append(math.degrees(math.atan(20.0 / (2 * math.pi * fc))))
            axes[2].semilogy(fcs, noise_rms, "o-", label="noise RMS (measured)")
            axes[2].set_xlabel("D filter cutoff (Hz)")
            axes[2].set_ylabel("RMS(D) (Nm)")
            axes[2].invert_xaxis()
            ax2 = axes[2].twinx()
            ax2.plot(fcs, phases, "s--", color="C3", label="phase lag at wn")
            ax2.set_ylabel("phase lag at wn (deg)")
            axes[2].set_title("filter: noise down, phase lag up")
            axes[2].grid(True, alpha=0.3)
            fig.tight_layout()
            fn = os.path.join(HERE, "pid_lesson16_exp3.png")
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:                      # noqa: BLE001
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 实验 4：级联 PID
# ===========================================================================

def exp4():
    head("实验 4  ★★ 级联 PID：外环 + 内环，以及带宽分离")

    J = J_DEFAULT
    r_step = math.radians(20.0)

    print("""
    【为什么一个 PID 不够】

    §实验 2 已经看到了：双积分器上做 PID，稳定条件是 ki < kd·kp/J。
    这只是【稳定性】。真正的问题是：

      ① 双积分器的两个极点都在原点，任何一点模型误差都会让相位裕度崩掉
      ② 更根本的：被控对象里【没有一个可测的中间量】被用起来

    但四旋翼有一个天然可测的中间量 ——

        J·ω̇ = M            （速率环的被控对象，见第 3 章）
        φ̇ = ω             （运动学，纯积分）

    ⇒ 把 ω 测出来、先把它控住，再让外面管角度。
      ★ 这就是级联，也是 PX4/ArduPilot 的实际架构：

        PX4:  mc_att_control（角度）→ mc_rate_control（角速率）→ 控制分配
        AP :  AC_AttitudeControl 的 attitude_controller_run_quat
              → rate_controller_run（AC_AttitudeControl_Multi.cpp:442）

    ★★ 为什么内环必须比外环快？（本实验的核心）

      把内环闭环看成一个近似的一阶环节（时间常数 1/λ）：

          ω(s)          λ
          ─────  =  ───────
          ω_sp(s)    s + λ

      外面套一个 P：  ω_sp = Ka·(φ_sp − φ)，且 φ̇ = ω

          φ(s)            λ·Ka
          ────── = ──────────────────
          φ_sp(s)   s² + λ·s + λ·Ka

      ⇒ 二阶系统！

          ωn_att = sqrt(λ·Ka)              ★ 外环闭环带宽
          ζ_att  = λ / (2·sqrt(λ·Ka))
                 = 0.5·sqrt(λ / Ka)        ★ 内环越快，外环越不容易振

      ⇒ ★★ 这就是「带宽分离」的数学形式：
         λ/Ka 越大，ζ_att 越大。λ = 2·Ka 时 ζ = 0.71；
         λ = 10·Ka 时 ζ = 1.58（过阻尼、不振荡）。

      ⚠ 但这个二阶形式【假设内环是一阶的】——
        只有 λ ≫ ωn_att 才成立。λ 不够大时，真实响应会和它偏离。
        下面把这两种情况都跑出来。
""")

    # ------------------------------------------------------------------
    sub("① 级联的实际响应（内环 P，外环 P）")

    def cascade_sim(Ka, lam, T=3.0, dt=1.0 / 2000, d=0.0,
                    ki_rate=0.0, tau_act=0.0):
        """外环 P（角度 → 速率设定值）+ 内环 PI（速率 → 力矩）。

        内环：J·ω̇ = u_act + d，u = Kp_r·(ω_sp − ω) + I_r
              无执行器滞后时闭环恰好是一阶，极点 λ = Kp_r / J
        执行器：u̇_act = (u − u_act) / tau_act   ← ★ 真实内环【不是】
              一阶的根源：电机升速、桨叶惯量、以及 D 项滤波器
              都会往内环里塞一个额外的滞后环节
        外环：ω_sp = Ka·(φ_sp − φ)，φ̇ = ω
        """
        Kp_r = lam * J
        ctrl = PID(kp=Kp_r, ki=ki_rate, kd=0.0, u_max=None)

        phi = [0.0]
        om = [0.0]
        ua = [0.0]                    # 实际作用到机体上的力矩
        ctrl.reset()

        n = int(round(T / dt))
        log = {k: np.zeros(n) for k in ("t", "phi", "om", "r", "wsp",
                                        "u", "I", "ua")}

        for i in range(n):
            t = i * dt
            r = r_step if t > 0.2 else 0.0
            w_sp = Ka * (r - phi[0])
            u = ctrl(t, w_sp, om[0], om[0], dt)
            tm = ctrl.terms()

            # ★ τ_a = 0 ⇒ 理想执行器：力矩立即到位（退化成原来的模型）
            if tau_act <= 0.0:
                ua[0] = u

            log["t"][i] = t
            log["phi"][i] = phi[0]
            log["om"][i] = om[0]
            log["r"][i] = r
            log["wsp"][i] = w_sp
            log["u"][i] = u
            log["I"][i] = tm["I"]
            log["ua"][i] = ua[0]

            def f(ph, w, u_a):
                du = 0.0 if tau_act <= 0.0 else (u - u_a) / tau_act
                return w, (u_a + d) / J, du
            k1a, k1b, k1c = f(phi[0], om[0], ua[0])
            k2a, k2b, k2c = f(phi[0] + 0.5 * dt * k1a,
                             om[0] + 0.5 * dt * k1b,
                             ua[0] + 0.5 * dt * k1c)
            k3a, k3b, k3c = f(phi[0] + 0.5 * dt * k2a,
                             om[0] + 0.5 * dt * k2b,
                             ua[0] + 0.5 * dt * k2c)
            k4a, k4b, k4c = f(phi[0] + dt * k3a, om[0] + dt * k3b,
                             ua[0] + dt * k3c)
            phi[0] += dt / 6.0 * (k1a + 2 * k2a + 2 * k3a + k4a)
            om[0] += dt / 6.0 * (k1b + 2 * k2b + 2 * k3b + k4b)
            ua[0] += dt / 6.0 * (k1c + 2 * k2c + 2 * k3c + k4c)

        return log

    Ka = 6.0
    print("    外环增益 Ka = %.1f（单位：rad/s 每 rad）" % Ka)
    print()
    rows = []
    settle_by_ratio = {}
    for ratio in (2.0, 5.0, 10.0, 20.0):
        lam = ratio * Ka
        log = cascade_sim(Ka, lam)
        _ss, ov, rise, settle, _pk = metrics(
            {"t": log["t"], "r": log["r"], "x": log["phi"]})
        wn = math.sqrt(lam * Ka)
        ze = 0.5 * math.sqrt(lam / Ka)
        if math.isfinite(settle):
            settle_by_ratio[ratio] = settle
        rows.append(["%.1f" % ratio, "%.1f" % lam, "%.2f" % wn,
                     "%.2f" % ze, "%.1f%%" % ov,
                     "%.0f%%" % overshoot_theory(ze),
                     "%.0f" % (settle * 1e3) if math.isfinite(settle) else "—"])

    print("    λ = 内环极点；ωn_att = sqrt(λ·Ka)；ζ_att = 0.5·sqrt(λ/Ka)")
    print()
    tab(rows, ["λ/Ka", "λ (rad/s)", "ωn_att (rad/s)", "ζ_att",
               "超调(实测)", "超调(二阶解析)", "2% 调节 (ms)"])
    print("""    ★★ 读表：

      · 比值 = 2 时 ζ_att = 0.71（二阶系统里最常见的「最佳阻尼」），
        实测超调 4.3%%，和二阶解析的 4%% 几乎一样 ——
        ★ 注意这里有个【陷阱】：本仿真里内环严格是一阶的，
          所以二阶解析式连比值 = 2 都预测得很准。**这不代表
          比值 2 在真机上也行**（原因见下面的执行器滞后）。
      · 比值 ≥ 5 时实测超调被测到 0%%（ζ_att ≥ 1.12，本来就不该超调）
      · ★ 调节时间在比值 = 5 时最短（%.0f ms），再往上反而【变长】——
        因为 ζ_att 太大（过阻尼），收敛被拖慢；
        而 ωn_att = sqrt(λ·Ka) 在 Ka 固定时增长得越来越慢
""" % (settle_by_ratio.get(5.0, 0.0) * 1e3,))

    # ---- ★★ 真正的理由：内环不是一阶的 -----------------------------
    print("""    ★★ 那为什么工程上还是要求「内环比外环快得多」？

      因为**真实内环从来不是一阶的**。它至少还有这些额外滞后：

        · 电机 + 桨叶的升速时间常数（几十 ms 量级）
        · 内环传感器（陀螺）和 D 项的滤波器（实验 3 的代价）
        · 计算与通信延迟（一整个采样周期到几个周期）

      ⇒ 这些滞后可以合并成一个等效时间常数 τ_a。
        把 τ_a 加进仿真，再看同一张表：
""")

    sub("①b ★★ 加上执行器滞后 τ_a：带宽分离的理由才真正出现")

    taus = (0.0, 0.02, 0.04, 0.06, 0.08, 0.10)
    ratios_b = (2.0, 5.0, 10.0, 20.0)
    ovs = {}                           # ovs[ratio][tau] -> 超调 %
    for ratio in ratios_b:
        for tau in taus:
            lg = cascade_sim(Ka, ratio * Ka, T=3.0, tau_act=tau)
            ovs[(ratio, tau)] = metrics(
                {"t": lg["t"], "r": lg["r"], "x": lg["phi"]})[1]

    rows = []
    for tau in taus:
        row = ["%.0f" % (tau * 1e3)]
        for ratio in ratios_b:
            row.append("%.1f%%" % ovs[(ratio, tau)])
        rows.append(row)

    print("    ★ 一阶执行器滞后 u̇_act = (u − u_act)/τ_a 串在内环里。")
    print("      τ_a 的量级：电机+桨升速 20~40 ms、D 项滤波器 5~10 ms、"
          "计算延迟 1~2 ms")
    print()
    tab(rows, ["τ_a (ms)"] + ["λ/Ka=%.0f" % r_ for r_ in ratios_b])
    print("""    ★★ 这张表是【单调的】，而且行与行之间差别巨大：

      · τ_a = 0（理想内环）：任何比值都不超调或轻微超调
        ⇒ ★ 这就是为什么只做纯数学仿真会得出「级联很容易」的错觉。
      · τ_a = 60 ms：比值 2 已经冲到 %.1f%% 超调，比值 20 还是 %.1f%%
      · τ_a = 100 ms：比值 2 是 %.1f%%，比值 20 只有 %.1f%%

      ⇒ ★★ 同样一个滞后，**对外环的伤害随 λ/Ka 增大而急剧减小**。

      ★ 注意这个结论【不能】用「比值大 ⇒ ωn_att 低 ⇒ τ_a 相移小」
        来解释 —— 恰恰相反，比值 20 的 ωn_att = 26.8 rad/s 比
        比值 2 的 8.5 rad/s 更【高】，τ_a 在它那里的相移反而更大：

          比值 2 ：−atan(8.49/12) − atan(8.49·τ_a)
          比值 20：−atan(26.8/120) − atan(26.8·τ_a)

      ★★ 真正的原因是**相位裕度**：
        比值 2 的 ζ_att = 0.71，外环【本来就用掉了】大部分裕度；
        比值 20 的 ζ_att = 2.24，外环是重过阻尼的，手里攒着一大把
        用不完的裕度 —— **它才有本钱去付 τ_a 这笔账**。

      ⇒ ★★ 所以「带宽分离」准确的说法是：
        **用外环的过阻尼（ζ_att > 1）去换「内环可以不必是理想一阶」
        这个自由度。** 代价就是第 ① 张表里看到的调节时间变长。

      ⇒ ★★ 这就是「带宽分离」四个字的全部含义：
        **只有内环的带宽远高于外环，外环才能把内环当成一个
        纯增益（无延迟）来设计。** 否则外环必须把内环的滞后
        算进自己的模型里，两个环就会【互相打架】。

    ⚠ 反过来说：**把外环调快，不能靠调高 Ka** ——
      调高 Ka 会同时抬高 ωn_att，把外环推向 τ_a 更敏感的频段。
      ★ 唯一正确的做法是把 λ（也就是内环的 P）提上去，
      而这又受实验 3 的噪声限制（P 大了，D 也得大，噪声就上来了）。
      ⇒ ★★ 整条链路是连在一起的：**噪声 → 带宽 → 级联的分离比**。

      ⇒ ★ 所以整定顺序永远是：**先内环、后外环**。
        内环调不好（有滞后、有振荡），外环再怎么调都是白费。
        这条规则在 PX4/ArduPilot 的手册里都是第一句话。
""" % (ovs[(2.0, 0.06)], ovs[(20.0, 0.06)],
       ovs[(2.0, 0.10)], ovs[(20.0, 0.10)]))

    # ------------------------------------------------------------------
    sub("② ★ 用真实数字算一遍（和源码参数对上）")

    print("""    把上面的 λ 换成 PX4 的参数，看它到底是多大。

    PX4 速率环（rate_control.cpp:78）：

        torque = Kp·(ω_sp − ω) + I − Kd·ω̇_accel + FF·ω_sp

    输出的 torque 是【归一化力矩】（[-1,1]，由控制分配器翻译成电机推力）。
    所以 PX4 的 MC_ROLLRATE_P = 0.15 的物理含义是：

        单位角速率误差（1 rad/s）对应 0.15 的归一化力矩

    ★ 要换算成 λ，需要知道「归一化力矩 1.0」等于多少 N·m ——
      这取决于飞机的推重比和力臂（第 3 章 §3.4.2 说过：
      力矩里 R 是公共因子，被混控归一化约掉了，剩下的量级由增益承担）。
      ★★ 本脚本【不估算】，直接用实验 1 ② 那个函数现算：
        同一台 1 kg / 0.25 m 力臂 / 推重比 2:1 的参考机，
        归一化力矩 1.0 = τ_scale = %.4f N·m。

      换算方向（和实验 1 ② 一致）：D 增益要【乘】τ_scale，
      P 增益（角速率误差 → 归一化力矩）同样要【乘】τ_scale。
""" % _norm_torque_scale())

    # PX4 真实默认值
    px4_p = 0.15               # src/modules/mc_rate_control/mc_rate_control_params.c:51
    px4_i = 0.2                # src/modules/mc_rate_control/mc_rate_control_params.c:63
    px4_d = 0.003              # src/modules/mc_rate_control/mc_rate_control_params.c:88
    px4_att_p = 6.5            # src/modules/mc_att_control/mc_att_control_params.c:53
    # ★★ 归一化力矩 1.0 对应多少 N·m —— 和实验 1 ② 用【同一个函数】算，
    #    这样本脚本里不会再出现第二个互相矛盾的数。
    tau_max_est = _norm_torque_scale()
    px4_att_p_ap = 4.5         # libraries/AC_AttitudeControl/AC_AttitudeControl.h:15
    ap_rat_p = 0.135           # libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h:11
    lam_px4 = px4_p * tau_max_est / J
    lam_ap = ap_rat_p * tau_max_est / J
    print("    取 MC_ROLLRATE_P = %.2f，归一化力矩 1.0 ≈ %.4f N·m"
          "（实验 1 ② 现算的 τ_scale），J = %.4f"
          % (px4_p, tau_max_est, J))
    print("    ⇒ 内环极点 λ = P·τ_scale/J = %.2f rad/s = %.2f Hz"
          % (lam_px4, lam_px4 / (2 * math.pi)))
    print()
    print("    再取 PX4 的角度环 MC_ROLL_P = %.1f（角度环 P，输出角速率）："
          % px4_att_p)
    wn = math.sqrt(lam_px4 * px4_att_p)
    ze = 0.5 * math.sqrt(lam_px4 / px4_att_p)
    print("    ⇒ ωn_att = sqrt(λ·Ka) = %.2f rad/s = %.2f Hz"
          % (wn, wn / (2 * math.pi)))
    print("    ⇒ ζ_att  = 0.5·sqrt(λ/Ka) = %.2f" % ze)
    print("    ⇒ 带宽比 λ/ωn_att = %.2f" % (lam_px4 / wn))
    print()
    print("    ArduPilot 的默认值（AC_AttitudeControl.h:15 的 ANG_RLL_P = %.1f，"
          % px4_att_p_ap)
    print("    AC_AttitudeControl_Multi.h:11 的 RAT_RLL_P = %.3f）用同样方法："
          % ap_rat_p)
    wn_a = math.sqrt(lam_ap * px4_att_p_ap)
    ze_a = 0.5 * math.sqrt(lam_ap / px4_att_p_ap)
    print("    ⇒ λ = %.2f rad/s，ωn_att = %.2f rad/s，ζ_att = %.2f，"
          "带宽比 = %.2f" % (lam_ap, wn_a, ze_a, lam_ap / wn_a))
    print("""
    ★★ 结论：两家默认参数给出的带宽比都在 1.6 ~ 1.8 倍，
       ζ_att 都在 0.8 ~ 0.9（略欠阻尼，二阶超调约 1~2%%）。
       ★ 两家在这个问题上的答案几乎一样，因为物理约束是一样的。
       ★★ 第 17 周实验 4 会把这两组默认值逐项反推、并跑仿真验证。

    ⚠ 这里有一个【隐含假设】：把「归一化力矩 1.0」等同于
      「τ_scale = %.4f N·m」。真实值随推重比、力臂、电池电压而变
      （τ_scale 大致与力臂、推重比成正比）。
      所以这两个 λ 是【本脚本这台 1 kg 参考机】上的值。
      真正要定量，必须做频域辨识（系统辨识 + Bode 图）。
""" % tau_max_est)

    # ------------------------------------------------------------------
    sub("③ ★★ 内环没有积分时：会出现「悬停倾斜」")

    print("""    这是级联结构里一个非常具体、非常真实的坑。

    设飞机有一个常值扰动力矩 d（重心偏了）。内环【只用 P】：

        稳态： J·ω̇ = 0 ⇒ u = −d
               u = Kp_r·(ω_sp − 0) = −d
         ⇒ ω_sp = −d / Kp_r     ← ★ 内环要一个【恒定的非零角速率指令】
         ⇒ 外环 Ka·(φ_sp − φ_ss) = −d/Kp_r
         ⇒ ★★ 稳态倾角误差  φ_ss − φ_sp = d / (Kp_r · Ka)

    ⇒ 现象：**飞机稳稳地停在了一个错的倾角上**，
      而且看起来「很稳」（不振荡、不漂移）—— 最难查的一类问题。

    ★ 内环加上积分（PI）就没这个问题了：
      稳态时 I = −d 顶住扰动，ω_sp 可以老老实实等于 0。

    ★★ 这就是「速率环必须有 I」的真正理由 ——
      不是为了消跟踪误差，而是为了让【外环的稳态】成立。
      两家飞控的速率环都有 I（MC_ROLLRATE_I / ATC_RAT_RLL_I），
      ★ 而角度环（外环）通常【没有 I】—— 因为没有必要，
        内环的 I 已经保证了「角速率指令 = 实际角速率」。
""")

    d = 0.05
    Kp_r = 5.0 * Ka * J          # λ = 5·Ka
    # 积分增益的取值不是随便拍的：PX4 的 MC_ROLLRATE_I = 0.2、
    # MC_ROLLRATE_P = 0.15（mc_rate_control_params.c:63/51），
    # 两者单位相同，所以 I/P = 0.2/0.15。这里照抄这个比值。
    ki_r_px4 = (0.2 / 0.15) * Kp_r
    print("    取 d = %.3f N·m，λ = 5·Ka（Ka = %.1f）" % (d, Ka))
    print("    ⇒ Kp_r = λ·J = %.5f N·m/(rad/s)" % Kp_r)
    print("    ⇒ 积分增益照抄 PX4 的 I/P 比（0.2/0.15）：ki = %.5f"
          % ki_r_px4)
    print("      积分时间 Kp_r/ki = %.3f s，远慢于内环闭环 λ = %.0f rad/s"
          % (Kp_r / ki_r_px4, 5.0 * Ka))
    print()

    rows = []
    res = {}
    for ki_r, name in ((0.0, "内环只用 P"), (ki_r_px4, "内环加 I")):
        # ★ 加 I 之后要跑到积分【自己】收敛为止：
        #   积分时间 Kp_r/ki = 1.33 s ⇒ 取 12 s（约 9 个积分时间常数）
        log = cascade_sim(Ka, 5.0 * Ka, T=12.0, d=d, ki_rate=ki_r)
        phi_ss = math.degrees(float(np.mean(log["phi"][-400:])))
        i_fin = float(log["I"][-1])
        res[name] = (phi_ss, i_fin)
        rows.append([name, "%.3f" % phi_ss,
                     "%.3f" % (phi_ss - math.degrees(r_step)),
                     "%.5f" % i_fin])

    tab(rows, ["配置", "稳态角 (°)", "稳态误差 (°)", "内环 I 终值 (N·m)"])

    phi0, i0 = res["内环只用 P"]
    phi1, i1 = res["内环加 I"]
    pred = math.degrees(d / (Kp_r * Ka))
    err0 = phi0 - math.degrees(r_step)
    err1 = phi1 - math.degrees(r_step)
    print("""    ★★ 读表：

      · 第 1 行（内环只用 P）：稳态角 = %.3f°，比设定值 %.1f° 差了
        %.3f°。这个差值和解析式 d/(Kp_r·Ka) = %.3f° 对得上
        （相对差 %.2f%%）。
        ★ 内环的 I 一直是 %.5f —— 没有积分，就没有人顶住那个常值扰动。

      · 第 2 行（内环加 I）：稳态角 = %.3f°，误差被压到 %.3f°
        （残差 %.4f°）。★★ 内环的 I 停在 %.5f N·m，
        和理论值 −d = %.3f N·m 只差 %.1f%% ——
        **积分器变成了一个「自动配平舵」，它记住的正是扰动力矩本身。**

      ⇒ ★ 注意这个坑有多阴险：第 1 行的飞机【不振荡、不漂移】，
        看上去「稳得很」，只是停在一个错的角度上。
        用「响应曲线好不好看」是查不出来的。

      ⇒ ★ 所以「飞机会自己歪着悬停」这件事，
        查的第一站就是**速率环的 I**（和重心）。
""" % (phi0, math.degrees(r_step), err0, pred,
       abs(err0 - pred) / abs(pred) * 100.0, i0,
       phi1, err1, abs(err1),
       i1, -d, abs(i1 + d) / d * 100.0))

    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 3, figsize=(16, 4.4))
            for ratio in (1.0, 2.0, 5.0, 20.0):
                lg = cascade_sim(Ka, ratio * Ka)
                axes[0].plot(lg["t"], np.degrees(lg["phi"]),
                             label="lambda/Ka = %.0f" % ratio)
            axes[0].plot(lg["t"], np.degrees(lg["r"]), "k:", label="setpoint")
            axes[0].set_title("cascade step response vs bandwidth ratio")
            axes[0].set_xlabel("time (s)")
            axes[0].set_ylabel("angle (deg)")
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(fontsize=8)

            for ki_r, lbl in ((0.0, "rate loop P only"),
                              (ki_r_px4, "rate loop PI")):
                lg = cascade_sim(Ka, 5.0 * Ka, T=12.0, d=d, ki_rate=ki_r)
                axes[1].plot(lg["t"], np.degrees(lg["phi"]), label=lbl)
            axes[1].plot(lg["t"], np.degrees(lg["r"]), "k:", label="setpoint")
            axes[1].set_title("hover tilt from a CG offset (d=%.2f Nm)" % d)
            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("angle (deg)")
            axes[1].grid(True, alpha=0.3)
            axes[1].legend(fontsize=8)

            ratios = np.linspace(1.0, 20.0, 40)
            ovs, zetas = [], []
            for r_ in ratios:
                lg = cascade_sim(Ka, r_ * Ka)
                ovs.append(metrics({"t": lg["t"], "r": lg["r"],
                                    "x": lg["phi"]})[1])
                zetas.append(0.5 * math.sqrt(r_))
            axes[2].plot(ratios, ovs, "o-", label="measured overshoot")
            axes[2].plot(ratios, [overshoot_theory(z) for z in zetas], "--",
                         label="2nd-order theory")
            axes[2].set_xlabel("lambda / Ka")
            axes[2].set_ylabel("overshoot (%)")
            axes[2].set_title("bandwidth separation vs overshoot")
            axes[2].grid(True, alpha=0.3)
            axes[2].legend(fontsize=8)

            fig.tight_layout()
            fn = os.path.join(HERE, "pid_lesson16_exp4.png")
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:                      # noqa: BLE001
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("二阶系统 + P 控制：稳态误差从哪来", exp1),
    "2": ("★ 积分与积分饱和", exp2),
    "3": ("★ D 项与噪声", exp3),
    "4": ("★★ 级联 PID 与带宽分离", exp4),
}


def main():
    print()
    print("#" * W)
    print("#  第 16 周 · 01 PID 控制器：从零到级联")
    print("#  配套实验")
    print("#" * W)

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
    print("#" * W)
    print("#  实验结束")
    print("#" * W)
    print()


if __name__ == "__main__":
    sys.exit(main())
