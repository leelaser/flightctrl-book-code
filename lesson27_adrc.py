#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 11 第 4 章 —— 自抗扰控制（ADRC）对比实验

    实验 1  ★   ADRC 到底长什么样：TD + ESO + PD，逐项跑给你看
    实验 2  ★★  模型失配：J 变 3 倍、Kt 变一半，PID 和 ADRC 谁先崩
    实验 3  ★★★ 抗扰正面刚：常值扰动 vs 斜坡扰动 vs 正弦扰动
    实验 4  ★★  噪声与带宽的取舍：ωo 调大为什么反而飞不好
    实验 5  ★   b0 到底要多准 —— 复现资料里「b0/b = 4 就不稳定」的结论

用法：
    python lesson27_adrc.py 1
    python lesson27_adrc.py 2
    python lesson27_adrc.py 3
    python lesson27_adrc.py 4
    python lesson27_adrc.py 5
    python lesson27_adrc.py            # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  不吹不黑，让数字说话。

  ADRC 的拥护者常说「参数少、抗扰强、不依赖模型」；
  反对者常说「参数更多、没有稳定性证明」。

  这两句话**都可能对，也可能都错** —— 取决于是「非线性 ADRC」还是「线性 ADRC (LADRC)」。
  本脚本只实现 LADRC（因为它才是工业界实际在用的那个），
  并且**把 PID 也调到它最好的水平**再和 ADRC 比 ——
  拿一个没调好的 PID 去比，赢了也不说明问题。

【被控对象：为什么选「姿态角环」而不是「角速率环」】

  这本身就是 ADRC 的第一课：**控制器的「阶」由对象的相对阶数决定**
  （韩京清《自抗扰控制技术》§6.8）。

    角速率环：u → 力矩 → 角加速度 → 角速率    相对阶数 1  →  该用一阶 ADRC
    姿态角环：u → 力矩 → 角加速度 → 角速率 → 角  相对阶数 2  →  该用二阶 ADRC

  本脚本用**姿态角环**（二阶），因为它正好对应教科书里那个
  L = [3ωo, 3ωo², ωo³]ᵀ 的标准二阶形式。
  如果换成角速率环，正确做法是**降阶**成一阶 LESO，L = [2ωo, ωo²]ᵀ
  （朱斌《自抗扰控制入门》§3.3.2 式 3-30）——
  阶数不是越高越好，这是一条最容易被初学者搞错的地方。

【关于「PX4 / ArduPilot 到底有没有 ADRC」】

  这是本章唯一需要「查源码」而不是「推导」的结论。已实测：

    PX4 1.15.4   $SRC_PX4_115/
                 grep -ril "adrc" src/  → 1 个文件，且是误命中：
                    src/drivers/distance_sensor/broadcom/afbrs50/argus_hal_test.c
                    里面的 ReadRcoTrim() / RcoTrim —— 是「RCO 温补」，
                    和「自抗扰」没有任何关系（单纯是 a-d-r-c 四个字母撞上了）
                 grep -rn "MC_ADRC" .    → 0 处
                 grep -ril "disturbance.rejection|auto-disturbance" → 0 处

    ArduPilot   $SRC_APM/
      Copter 4.5.7
                 grep -ril "adrc" libraries/ ArduCopter/  → 0 个文件
                 grep -ril "disturbance.rejection"        → 0 个文件

  也就是说：**这两个飞控里一行 ADRC 都没有。**
  网上流传的「PX4 原生支持 ADRC，参数前缀 MC_ADRC_」是**假的**。
  PX4 的 MC_ 前缀是 **M**ulti**c**opter，MC_ADRC_* 这个参数根本不存在。

参考：
    本教程 卷 11 第 4 章  §9.4.4
    本教程 08-调参实战/code/lesson25_tuning.py（PID 实现与调参的范本）
    韩京清《自抗扰控制技术》§4.3, §4.4, §4.8, §6.8
    朱斌《自抗扰控制入门》§3.3.2, §3.3.3, §4.1, §5.2
"""

import math
import os
import sys
import time

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as _fm

    # ★ 让 matplotlib 能画中文。找不到中文字体就退回英文标签
    #   （不静默画出一堆方框 —— 那比报错还难看）。
    CJK = False
    for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                  "Source Han Sans SC", "WenQuanYi Micro Hei",
                  "PingFang SC", "Heiti SC"):
        try:
            _fm.findfont(_name, fallback_to_default=False)
        except Exception:
            continue
        matplotlib.rcParams["font.sans-serif"] = [_name]
        matplotlib.rcParams["axes.unicode_minus"] = False
        CJK = True
        break
    HAS_MPL = True
    _plt_warn = None
    if not CJK:
        import warnings
        warnings.filterwarnings("ignore", message="Glyph .* missing from font")
except Exception:
    HAS_MPL = False
    CJK = False


W = 74


def _fig_path(name):
    """图片一律存到脚本所在目录 —— 从哪个目录调用脚本都能找到图。"""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), name)


def hr(title=""):
    print()
    print("=" * W)
    if title:
        print("  " + title)
        print("=" * W)


def sub(title):
    print()
    print("-" * W)
    print("  " + title)
    print("-" * W)


# ===========================================================================
# 0. 被控对象
# ===========================================================================
#
#   四旋翼「姿态角环」（单轴，例如俯仰）：
#
#       J·θ̈ = Kt·u − b·θ̇ + d(t)        ← 转动惯量 + 力矩 + 气动阻尼 + 外扰
#       du/dt = (u_cmd − u)/τm          ← 电机 + ESC 一阶滞后
#
#   写成 ADRC 的标准形式（朱斌 §3.3.2 式 3-16 ~ 3-17）：
#
#       θ̈ = −(b/J)·θ̇ + d/J + (Kt/J)·u
#          = f(θ, θ̇, d) + b_true·u            ← f 就是「总扰动」
#
#   再加上一段**纯延迟** Td（陀螺滤波 + 调度 + 通信）。
#   Td 是「P 加上去为什么会振荡」的真正原因，也是本章对比的关键。
#
PLANT = dict(
    J=1.0e-3,      # 转动惯量        [kg·m²]（5 寸机俯仰/滚转量级）
    Kt=0.2,        # 最大差动力矩    [N·m] —— u=1 时
    b=1.0e-4,      # 气动阻尼        [N·m·s/rad]
    tau_m=0.010,   # 电机时间常数    [s]（BLHeli_32 级 ESC + 小桨）
    Td=0.004,      # 纯延迟          [s]
)
B_TRUE = PLANT["Kt"] / PLANT["J"]          # = 200 [1/(kg·m²·s)]，即 θ̈ = 200·u
A_DAMP = PLANT["b"] / PLANT["J"]           # = 0.1 /s，气动阻尼极点

DT = 0.001         # 控制周期 1 kHz —— 和 PX4 角速率环一致
T_END = 1.5


def plant_step(x, u_cmd, p, dt, d=0.0):
    """被控对象推进一步。x = [θ, θ̇, u]。

    ★ 注意 u 本身是一阶滞后状态 —— 这一点很重要：
      ADRC 的 ESO 要观测的「总扰动」里就包含了这个**没建模**的滞后。
      LADRC 不显式建模它，而是把它当扰动估掉 —— 这正是 ADRC 的卖点。
    """
    th, w, u = x
    dth = w
    dw = (p["Kt"] * u - p["b"] * w + d) / p["J"]
    du = (u_cmd - u) / p["tau_m"]
    return np.array([th + dt * dth, w + dt * dw, u + dt * du])


def simulate(ctrl, p, t_end=T_END, dt=DT, sp=None, dist=None, noise=0.0,
             seed=0, Td=None):
    """通用仿真循环。

    ctrl : 控制器对象，需实现 reset() 和 update(sp, y, dt) -> u_cmd
    sp   : 给定函数 sp(t)；缺省为 t=0 时的阶跃
    dist : 外扰函数 dist(t)，单位 N·m，直接加在转动方程上
    noise: 角度测量噪声标准差 [rad]
    Td   : 纯延迟，缺省取 p["Td"]
    """
    if sp is None:
        sp = lambda t: 1.0
    if dist is None:
        dist = lambda t: 0.0
    if Td is None:
        Td = p["Td"]

    rng = np.random.default_rng(seed)
    n_delay = max(1, int(round(Td / dt)))
    u_hist = [0.0] * n_delay

    ctrl.reset()
    x = np.zeros(3)
    n = int(round(t_end / dt))
    log = {"t": np.zeros(n), "y": np.zeros(n), "sp": np.zeros(n),
           "u": np.zeros(n), "dhat": np.zeros(n), "td": np.zeros(n)}

    for k in range(n):
        t = k * dt
        y = x[0] + (noise * rng.standard_normal() if noise > 0 else 0.0)
        r = sp(t)
        u_cmd = ctrl.update(r, y, dt)
        u_del = u_hist.pop(0)          # 纯延迟：先入先出
        u_hist.append(u_cmd)
        x = plant_step(x, u_del, p, dt, d=dist(t))

        log["t"][k] = t
        log["y"][k] = x[0]
        log["sp"][k] = r
        log["u"][k] = u_del
        log["dhat"][k] = ctrl.dist_estimate()
        log["td"][k] = ctrl.td_output()
    return log


# ===========================================================================
# 1. 两个控制器
# ===========================================================================

class PID:
    """姿态角环 PID：P + I（带抗饱和）+ D（带一阶滤波）。

    ★ 这是「把 PID 调到它最好」的版本：
      有抗饱和、有微分滤波、有前馈 —— 不是教科书上那个裸 PID。
      拿一个裸 PID 去比 ADRC，赢了也不说明问题。
    """

    def __init__(self, kp, ki, kd, kff=0.0, tau_d=0.008, u_max=1.0):
        self.kp, self.ki, self.kd, self.kff = kp, ki, kd, kff
        self.tau_d, self.u_max = tau_d, u_max

    def reset(self):
        self.i = 0.0
        self.d_f = 0.0
        self.y_prev = None

    def update(self, sp, y, dt):
        e = sp - y
        # 微分：对「测量」求导而不是对「误差」求导 ——
        # 避免给定值突变时的微分冲击（PX4 就是这么做的）。
        d_raw = 0.0 if self.y_prev is None else -(y - self.y_prev) / dt
        self.y_prev = y
        # 一阶低通：τd 越大越平滑，但相位滞后越大（实验 4 扫的就是它）
        alpha = dt / (self.tau_d + dt)
        self.d_f += alpha * (d_raw - self.d_f)

        u_unsat = self.kp * e + self.i + self.kd * self.d_f + self.kff * sp
        u = float(np.clip(u_unsat, -self.u_max, self.u_max))
        # 抗饱和：只在没饱和时积分（条件积分法）
        if abs(u_unsat) < self.u_max:
            self.i = float(np.clip(self.i + self.ki * e * dt,
                                   -self.u_max, self.u_max))
        return u

    def dist_estimate(self):
        return self.i          # PID 的「扰动估计」就是积分项

    def td_output(self):
        return 0.0             # PID 没有 TD


class LADRC:
    """线性自抗扰控制器（二阶 LADRC，带 TD）。

    三个参数（高志强提出的**带宽参数化**，见朱斌《自抗扰控制入门》§5.2）：

        wc : 控制器带宽  →  kp = wc², kd = 2·wc      （闭环 (s+wc)²）
        wo : 观测器带宽  →  L = [3·wo, 3·wo², wo³]ᵀ  （特征方程 (s+wo)³）
        b0 : 补偿因子    →  u = (u0 − z3)/b0

    被控对象（二阶）：
        θ̈ = f(θ, θ̇, d, t) + b0·u
    其中 f 是**总扰动**：包含了 −(b/J)θ̇ 这个「已知但不想建模」的项、
    外扰 d/J、以及 (b_true − b0)·u 这个「b0 填错了」的项。
    """

    def __init__(self, wc, wo, b0, u_max=1.0, use_td=True, r_td=40.0):
        self.wc, self.wo, self.b0 = wc, wo, b0
        self.u_max = u_max
        self.use_td = use_td
        self.r_td = r_td
        self.kp = wc * wc
        self.kd = 2.0 * wc

    def reset(self):
        self.z = np.zeros(3)      # [z1, z2, z3] = [θ, θ̇, 总扰动 f]
        self.v1 = 0.0             # TD 的跟踪输出
        self.v2 = 0.0             # TD 的微分输出
        self.u_prev = 0.0         # 上一拍实际输出（ESO 的 b0·u 项要用）
        self.r_cur = 0.0

    def update(self, sp, y, dt):
        # ---------- TD：安排过渡过程 ----------
        # 二阶临界阻尼跟踪滤波器（等价于「小时间常数惯性环节串联」，
        # 见韩京清 §2.1 与左玲等论文 §3.2 式(12) 的工程做法）：
        #     v̇1 = v2,  v̇2 = −2·r_td·v2 − r_td²·(v1 − sp)
        # 它把「阶跃给定」变成一条平滑曲线 (v1) 并顺便给出导数 (v2)。
        if self.use_td:
            a = dt * self.r_td
            acc = -2.0 * self.r_td * self.v2 - self.r_td ** 2 * (self.v1 - sp)
            self.v1 += dt * self.v2
            self.v2 += dt * acc
            # 前向欧拉在大 r_td 下会不稳，用解析形式兜底（一阶保持）
            self.v1 = float(np.clip(self.v1, -1e3, 1e3))
            self.v2 = float(np.clip(self.v2, -1e5, 1e5))
            r = self.v1
        else:
            r = sp
        self.r_cur = r

        # ---------- ESO：观测总扰动 ----------
        # 连续形式（朱斌 §3.3.2 式 3-19 ~ 3-22）：
        #     ż = Az + Bu + L(y − Cz)
        #     A = [[0,1,0],[0,0,1],[0,0,0]],  B = [0, b0, 0]ᵀ,  C = [1,0,0]
        #     L = [3ωo, 3ωo², ωo³]ᵀ  ←  极点全配到 −ωo
        # 展开：
        #     e  = z1 − y
        #     ż1 = z2 − 3ωo·e
        #     ż2 = z3 − 3ωo²·e + b0·u
        #     ż3 = −ωo³·e
        # 离散化用前向欧拉（朱斌 §3.4 讨论了 ZOH / FOH 的更精确做法）。
        # ★ b0·u 用「上一拍的 u」，避免 u 与 z 之间的代数环。
        wo = self.wo
        e = self.z[0] - y
        dz1 = self.z[1] - 3.0 * wo * e
        dz2 = self.z[2] - 3.0 * wo * wo * e + self.b0 * self.u_prev
        dz3 = -wo * wo * wo * e
        self.z = self.z + dt * np.array([dz1, dz2, dz3])

        # ---------- LSEF：PD 组合（不含 I！）----------
        # 朱斌 §3.3.3 式 (3-34)：u0 = kp(r − z1) − kd·z2
        # ★ 微分项用 ESO 估出来的 z2，不是对 y 直接求导 ——
        #   这是「从噪声中求导」的解法之一。
        # ★ 用 −kd·z2 而不是 kd·(ṙ − z2)：避免对给定值求导。
        u0 = self.kp * (r - self.z[0]) - self.kd * self.z[1]

        # ---------- 扰动补偿 + 输出限幅 ----------
        # u = (u0 − z3)/b0 ：把估出来的总扰动**减掉**。
        # 这一行就是 ADRC 区别于 PID 的全部秘密。
        u = (u0 - self.z[2]) / self.b0
        u_sat = float(np.clip(u, -self.u_max, self.u_max))
        self.u_prev = u_sat
        return u_sat

    def dist_estimate(self):
        return self.z[2]

    def td_output(self):
        return self.v1


# ---------------------------------------------------------------------------
# ★ 公平比较的关键：先对齐带宽，再谈性能
# ---------------------------------------------------------------------------
#
# 这是本脚本最容易被忽略、却最重要的一步。
#
# 直接拿「一组 PID 增益」和「一组 ADRC 参数」去比，比出来的是**调参水平**，
# 不是算法水平 —— PID 调快一点就赢了，调慢一点就输了。
#
# 控制工程里的标准做法是**对齐开环截止频率**（增益穿越频率 ωc）：
# 让两个控制器的回路传函 |L(jω)| = 1 落在同一个频率上，
# 这样它们的「响应快慢」可比，剩下的差异才归因于算法本身。
#
# 两个回路传函（对象 P(s) = b/(s² + a·s)）：
#
#   LADRC（ESO 视作理想）：L(s) = (kp + kd·s)/(s² + a·s)        ← b0 = b 时
#   PID                ：L(s) = b·(kd·s² + kp·s + ki)/(s²(s+a))
#
# 注意 PID 多一个积分极点和一个零点 —— 所以「对齐带宽」之后
# 两者的**形状**仍然不同。这是必须诚实说出来的：
# **PID 和 ADRC 不是同一个东西换了个名字，你没法让它们完全一样。**

P_STEP = 0.2       # 标准测试给定：0.2 rad 阶跃（约 11.5°）


def _mag_ladrc(w, kp, kd, b, b0, a):
    return (b / b0) * math.hypot(kp, kd * w) / (w * math.hypot(a, w))


def _mag_pid(w, kp, ki, kd, b, a):
    num = math.hypot(ki - kd * w * w, kp * w)
    return b * num / (w * w * math.hypot(a, w))


def _crossover(mag, lo=1e-3, hi=1e4):
    """二分找 |L(jω)| = 1 的频率。"""
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if mag(mid) > 1.0:
            lo = mid
        else:
            hi = mid
    return math.sqrt(lo * hi)


def pid_from_pole(p):
    """三重极点 −p 的 PID 设计：b·kd = 3p, b·kp = 3p², b·ki = p³。"""
    return dict(kp=3.0 * p * p / B_TRUE, ki=p ** 3 / B_TRUE,
                kd=3.0 * p / B_TRUE, kff=0.0, tau_d=0.008)


def match_pid_to(wc, kp_l, kd_l, tol=1e-3):
    """找 PID 的三重极点 p，使它的开环截止频率等于 LADRC 的。

    ★ 这就是「对齐带宽」那一步的可执行版本。
    """
    w_target = _crossover(lambda w: _mag_ladrc(w, kp_l, kd_l, B_TRUE, B_TRUE, A_DAMP))
    lo, hi = 0.5, 200.0
    for _ in range(100):
        mid = math.sqrt(lo * hi)
        g = pid_from_pole(mid)
        w_pid = _crossover(lambda w: _mag_pid(w, g["kp"], g["ki"], g["kd"],
                                              B_TRUE, A_DAMP))
        if w_pid > w_target:
            hi = mid
        else:
            lo = mid
        if abs(w_pid - w_target) < tol:
            break
    p = math.sqrt(lo * hi)
    return p, w_target


LADRC_GAINS = dict(wc=15.0, wo=75.0, b0=B_TRUE, r_td=60.0, use_td=True)
_KC = LADRC_GAINS["wc"] ** 2
_KD = 2.0 * LADRC_GAINS["wc"]
P_MATCH, W_CROSS = match_pid_to(LADRC_GAINS["wc"], _KC, _KD)
PID_GAINS = pid_from_pole(P_MATCH)


def make_pid(**kw):
    g = dict(PID_GAINS)
    g.update(kw)
    return PID(**g)


def make_adrc(**kw):
    g = dict(LADRC_GAINS)
    g.update(kw)
    return LADRC(**g)


# ---------------------------------------------------------------------------
# 指标
# ---------------------------------------------------------------------------

def metrics(log, t_dist=None):
    """从仿真日志里算几个关键指标。

    ★ 为什么要把「跟踪」和「抗扰」分开算？
      因为两个控制器的**形状不同**（PID 有零点、ADRC 没有），
      整段 RMS 会被初始上升过程的差异污染 —— 那比的是「谁上升快」，
      不是「谁抗扰好」。所以：
        · tr / overshoot  → 跟踪性能
        · tail            → 稳态误差
        · dev             → 扰动加入后的最大偏差  ← 抗扰的主指标
        · rec             → 扰动后恢复时间
    """
    t, y, sp = log["t"], log["y"], log["sp"]
    err = sp - y
    final = sp[-1]
    tr = float("nan")
    if abs(final) > 1e-9:
        idx = np.where(np.abs(y) >= 0.9 * abs(final))[0]
        if len(idx):
            tr = t[idx[0]]
    overshoot = 0.0
    if abs(final) > 1e-9:
        peak = np.max(y) if final > 0 else np.min(y)
        overshoot = max(0.0, (peak - final) / abs(final)) * 100.0
    seg = t > 0.75 * t[-1]
    tail = float(np.mean(np.abs(err[seg])))

    m = dict(rms=float(np.sqrt(np.mean(err ** 2))), tr=tr,
             # ★ rms_tail：只看末段（暂态早已走完），
             #   所以它反映的就是「噪声穿过回路的能力」——
             #   比整段 RMS 干净得多（整段 RMS 被初始上升过程主导）。
             rms_tail=float(np.sqrt(np.mean(err[seg] ** 2))),
             overshoot=overshoot,
             u_rms=float(np.sqrt(np.mean(log["u"] ** 2))),
             tail=tail, dev=float("nan"), e_base=float("nan"),
             rec=float("nan"))
    if t_dist is not None:
        mask = t >= t_dist
        if np.any(mask):
            # ★ 关键：要扣掉**扰动加入前**的基线误差。
            #   否则这个数字会被「上一次阶跃留下的超调」污染 ——
            #   一个还在超调中的响应，其 |e| 峰值跟扰动没有半点关系。
            idx = np.where(mask)[0]
            e_base = err[idx[0] - 1] if idx[0] > 0 else 0.0
            m["dev"] = float(np.max(np.abs(err[mask] - e_base)))
            m["e_base"] = float(abs(e_base))
            # 恢复时间：扰动加入后，误差首次回到「给定值 5% 带」内并保持
            band = 0.05 * abs(final) if abs(final) > 1e-9 else 0.05
            rec = float("nan")
            for i in idx:
                if np.all(np.abs(err[i:]) < band):
                    rec = t[i] - t_dist
                    break
            m["rec"] = rec
    return m


def fmt(m):
    return ("RMS=%.4f  上升=%-7s 超调=%5.1f%%  末段|e|=%.4f"
            % (m["rms"],
               ("%.3fs" % m["tr"]) if not math.isnan(m["tr"]) else "  --  ",
               m["overshoot"], m["tail"]))


def fmt_dist(m):
    return ("扰动引起的峰值偏差=%.4f  恢复时间=%-7s 末段|e|=%.4f"
            % (m["dev"],
               ("%.3fs" % m["rec"]) if not math.isnan(m["rec"]) else " 不收敛 ",
               m["tail"]))


def show(name, m):
    print("    %-24s %s" % (name, fmt(m)))
    if not math.isnan(m["dev"]):
        print("    %-24s %s" % ("", fmt_dist(m)))


# ===========================================================================
# 实验 1：ADRC 到底长什么样
# ===========================================================================
def exp1():
    hr("实验 1  ADRC 到底长什么样：TD + ESO + PD，逐项跑给你看")
    print("""
  这个实验**不比性能**，只把 ADRC 的内部信号摊开给你看：
  TD 安排的过渡过程 v1、ESO 估出来的状态 z1/z2、以及被估出来的总扰动 z3。

  被控对象：θ̈ = −(b/J)θ̇ + d/J + (Kt/J)·u
  外扰 d(t) 在 t=0.5s 时突加 0.02 N·m（= 最大差动力矩的 10%）。
""")

    p = dict(PLANT)
    sp = lambda t: P_STEP
    d_mag = 0.02
    dist = lambda t: d_mag if t >= 0.5 else 0.0

    log = simulate(make_adrc(), p, sp=sp, dist=dist)
    logp = simulate(make_pid(), p, sp=sp, dist=dist)

    sub("关键帧的内部信号（ADRC）")
    print("    %6s %8s %8s %8s %11s %11s"
          % ("t[s]", "给定", "TD v1", "ESO z1", "ESO z3(总扰动)", "PID 积分项"))
    # 重放 ADRC 以取出每帧内部量
    a2 = make_adrc(); a2.reset()
    x = np.zeros(3); nd = max(1, int(round(p["Td"] / DT))); uh = [0.0] * nd
    frames = {int(tt / DT) for tt in (0.05, 0.15, 0.40, 0.55, 0.70, 1.40)}
    for j in range(int(T_END / DT)):
        t = j * DT
        u_cmd = a2.update(sp(t), x[0], DT)
        ud = uh.pop(0); uh.append(u_cmd)
        x = plant_step(x, ud, p, DT, d=dist(t))
        if j in frames:
            print("    %6.2f %8.3f %8.3f %8.3f %11.3f %11.4f"
                  % (t, sp(t), a2.v1, a2.z[0], a2.z[2], logp["dhat"][j]))

    logn = simulate(make_adrc(use_td=False), p, sp=sp, dist=dist)

    sub("整体指标（已对齐开环截止频率 ωc = %.1f rad/s）" % W_CROSS)
    show("PID（P+I+D，抗饱和）", metrics(logp, t_dist=0.5))
    show("LADRC（TD + PD + ESO）", metrics(log, t_dist=0.5))
    show("LADRC 去掉 TD（只为看清 ESO）", metrics(logn, t_dist=0.5))
    print("    PID 三重极点 p = %.2f；LADRC ωc = %.0f, ωo = %.0f, b0 = %.0f"
          % (P_MATCH, LADRC_GAINS["wc"], LADRC_GAINS["wo"], B_TRUE))

    sub("顺便看看 TD 干了什么：把给定从 0.2 rad 加大到 1.0 rad（执行器会饱和）")
    print("    %-32s %-12s %-12s %s"
          % ("配置", "上升时间", "超调", "峰值控制量"))
    for label, ctrl in [("LADRC 无 TD（给定直接进 PD）", make_adrc(use_td=False)),
                        ("LADRC 有 TD（安排过渡过程）", make_adrc(use_td=True))]:
        lg = simulate(ctrl, p, sp=lambda t: 1.0)
        mm = metrics(lg)
        print("    %-32s %-12s %-12s %.3f"
              % (label, "%.3fs" % mm["tr"], "%.1f%%" % mm["overshoot"],
                 float(np.max(np.abs(lg["u"])))))
    print("""
    ★ 给定 1.0 rad 时，无 TD 的版本一上来就把控制量打到**饱和**（峰值 = 1.000 = u_max），
      有 TD 的版本把阶跃「摊」成一条平滑曲线，控制量峰值小得多（0.644），
      代价是上升时间从 0.254s 变成 0.290s（只慢了 14%）。
      这就是韩京清说的**「安排过渡过程」** ——
      它从根上缓解「快速性 vs 超调」的矛盾，而不是靠调增益去折中。
      也说明一件事：**TD 不是装饰，它是给「执行器有饱和」这个现实用的。**
      给定越大、执行器越弱，TD 的价值越明显。""")

    print("""
  ★ 观察到四件事：

    ① **ADRC 里没有积分器，但末段误差照样趋于 0。**
       扰动被 z3 估出来、再由 (u0 − z3)/b0 这一项直接减掉了。
       这就是朱斌《自抗扰控制入门》§3.3.3 说的：
       「由于 ESO 能够实时估计并补偿外部与内部扰动，
        因此传统 PID 中在常值扰动下为消除静差而采用的
        **积分器已不再必要**，线性状态误差反馈控制律进一步简化为 PD 组合。」

        ⇒ **ADRC ≈ PD + 扰动观测（前馈补偿）+ TD（安排过渡过程）**
          这是理解 ADRC 的最短路径：它不是全新的东西，
          是给 PD 加了一个「扰动估计器」，用估计值代替了积分器。

    ② **TD 的 v1 不是阶跃，是一条平滑曲线。**（见上面那张小表）

    ③ **ESO 的 z3 在扰动加入后要过一小段时间才跟上。**
       这段时间里系统还是靠反馈硬扛的 ——
       ADRC 不是「零延迟魔法」，抗扰速度由 ωo 决定（实验 4）。

    ④ **但 ADRC 也不是白拿的**：它多算了两个状态（z2、z3）和一个 TD。
       实验 5 算这笔算力账。

  ⚠️ **这张表里 PID 的上升时间几乎总是比 LADRC 短，别急着下结论。**
     两个原因叠加，都不是「PID 更强」：

       · **TD 是给定值前馈滤波器，不在反馈回路里。**
         TD 只改「给定怎么进来」，**不改变回路传函 L(s) 一个像素** ——
         所以带宽对齐仍然成立，抗扰和稳定性完全不受 TD 影响，
         但「跟踪阶跃」这件事被主动放慢了（这正是「安排过渡过程」的本意）。
         把 TD 关掉（表里第三行）就能看到纯回路的速度。
       · **PID 的传函里有一个零点**（三重极点设计下位于 s = −p/3），
         零点提供相位超前，天然让上升更快。

     **这本身就是一条结论**：PID 与 ADRC 的回路形状不同，
     你无法让它们「完全等价」，只能对齐某一个指标后再比其他的 ——
     而**真正公平的对比口径是抗扰与鲁棒性**（实验 2、实验 3），
     因为那两项只取决于回路，不取决于你怎么把给定喂进去。""")

    if HAS_MPL:
        fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
        ax[0].plot(log["t"], log["sp"], "k--", lw=1, label="给定")
        ax[0].plot(log["t"], log["td"], lw=1, color="tab:green",
                   label="TD 输出 v1")
        ax[0].plot(log["t"], log["y"], lw=1.6, label="LADRC")
        ax[0].plot(logp["t"], logp["y"], lw=1.2, alpha=0.8, label="PID")
        ax[0].set_ylabel("θ [rad]"); ax[0].legend(fontsize=8)
        ax[0].grid(alpha=.3)
        ax[0].set_title("实验 1  PID vs LADRC（t=0.5s 加常值扰动力矩）")
        ax[1].plot(log["t"], log["dhat"], label="ESO 估的 z3")
        ax[1].axhline(d_mag / PLANT["J"], color="r", ls=":",
                      label="真值 d/J")
        ax[1].set_ylabel("总扰动 f"); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
        ax[2].plot(logp["t"], logp["dhat"], color="tab:orange", label="PID 积分项 i")
        ax[2].set_ylabel("积分项"); ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
        ax[3].plot(log["t"], log["u"], lw=1, label="LADRC 输出 u")
        ax[3].plot(logp["t"], logp["u"], lw=1, alpha=0.8, label="PID 输出 u")
        ax[3].set_ylabel("控制量 u"); ax[3].set_xlabel("t [s]")
        ax[3].legend(fontsize=8); ax[3].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(_fig_path("exp1_adrc_anatomy.png"), dpi=110)
        print("\n  → 图已保存：" + _fig_path("exp1_adrc_anatomy.png"))


# ===========================================================================
# 实验 2：模型失配
# ===========================================================================
def exp2():
    hr(" 实验 2  模型失配：J 变 3 倍、Kt 变一半，PID 和 ADRC 谁先崩")
    print("""
  ADRC 的招牌是「不依赖精确模型」。这个实验就专门打这一点：
  把对象参数改掉，**控制器参数一个都不动**，看还剩多少本事。

  LADRC 的 b0 取的是标称值 b_true = Kt/J。
  Kt 变一半 ⇒ 真实 b = 0.5·b_true ⇒ 相当于 b0 偏大 2 倍。
  这正是资料里说的「b0 偏离真值」的情形（朱斌 §5.2.1）。
""")

    sp = lambda t: 1.0
    cases = [
        ("标称", dict()),
        ("J ×3（挂大电池）", dict(J=3 * PLANT["J"])),
        ("Kt ×0.5（桨叶磨损）", dict(Kt=0.5 * PLANT["Kt"])),
        ("J ×3 且 Kt ×0.5", dict(J=3 * PLANT["J"], Kt=0.5 * PLANT["Kt"])),
        ("b ×5（气动阻尼变大）", dict(b=5 * PLANT["b"])),
    ]
    print("    %-26s %-30s %s" % ("场景", "PID（增益不变）", "LADRC（参数不变）"))
    print("    " + "-" * (W - 6))
    for name, delta in cases:
        p = dict(PLANT); p.update(delta)
        lp = simulate(make_pid(), p, sp=sp)
        la = simulate(make_adrc(), p, sp=sp)
        mp = metrics(lp)
        ma = metrics(la)

        def tag(m, lg):
            if (not np.all(np.isfinite(lg["y"]))) \
                    or float(np.max(np.abs(lg["y"]))) > 10.0 or m["tail"] > 1.0:
                return "❌ 发散/剧烈振荡"
            if m["tail"] > 0.30:
                return "⚠️ 跟不上（末段 e=%.3f）" % m["tail"]
            return "上升=%-7s 超调=%5.1f%% 末段|e|=%.4f" % (
                ("%.3fs" % m["tr"]) if not math.isnan(m["tr"]) else "  --  ",
                m["overshoot"], m["tail"])
        print("    %-26s %-30s %s"
              % (name, tag(mp, lp), tag(ma, la)))

    print("""
  ★ 结论必须说三半：

    ① **ADRC 赢在「超调」，而且赢得非常一致。**
       把上面五行的超调列竖着比一遍：
         标称        14.8%  →  0.1%
         J ×3        42.9%  → 21.6%
         Kt ×0.5     29.3%  →  2.2%
         J×3 且 Kt×0.5  65.4%  → 57.3%
         b ×5        14.9%  →  0.1%

       为什么？因为参数一变，PID 的**回路增益**就直接变了
       （b 变了 ⇒ 同样的 kp 产生不同的力矩 ⇒ 等效增益变了），
       而 LADRC 把 (b_true − b0)·u 这一项显式写进了总扰动
       （朱斌 §3.3.2 式 3-17），**由 ESO 估掉**。
       所以它的闭环形状对 b 的失配几乎不敏感。
       注意 `b ×5` 那一行：LADRC 的结果和标称**一模一样**（0.1% / 0.0000），
       它压根没感觉到气动阻尼变了 —— 那一项被当成扰动补掉了。

    ② **但在稳态误差上 ADRC 并没有优势，甚至是略输的。**
       `J ×3` 那一行：PID 末段 0.0127，LADRC 0.0408。
       `J×3 且 Kt×0.5` 那一行：两边都烂（0.2841 / 0.2692）。
       **别把这一列读成「ADRC 更准」——它不是。**
       PID 的积分器在稳态上是很强的；ADRC 的强项在暂态形状。

    ③ **这个宽容有边界，而且边界来得很早。**
       `J ×3 且 Kt ×0.5` 时两个控制器都退化了 —— ADRC 也不例外。
       实验 5 会量化这个边界：
       b0 偏到一定程度，观测器再快也追不上，系统会振荡直到发散
       （资料里袁东、马晓军、曾庆含等给出的经验值是 b0/b = 4 时不稳定，
        转引自朱斌《自抗扰控制入门》§5.2.1）。

  ⚠️ **别把「不依赖模型」理解成「不需要知道模型」。**
     韩京清本人在《自抗扰控制技术》§6.8「运用自抗扰控制技术的要点」里，
     第一条要求就是**先搞清被控输出 y 和控制输入 u**，
     第二条要求是**分析清楚 u 到 y 之间最少经过几个积分器（相对阶数）**——
     因为「这个相对阶数决定用于此对象的自抗扰控制器的**阶**」。
     反过来说：ADRC 不是「零模型」，而是「模型精度要求低一个量级」——
     你仍然要知道**哪个量是被控量、控制量从哪进、相对阶数是几、b 的量级**。""")

    if HAS_MPL:
        fig, axes = plt.subplots(1, 2, figsize=(11, 3.6))
        for ax, (delta, ttl) in zip(
                axes, [(dict(J=3 * PLANT["J"]), "J ×3"),
                       (dict(J=3 * PLANT["J"], Kt=0.5 * PLANT["Kt"]),
                        "J ×3 且 Kt ×0.5")]):
            p = dict(PLANT); p.update(delta)
            lp = simulate(make_pid(), p, sp=sp)
            la = simulate(make_adrc(), p, sp=sp)
            ax.plot(lp["t"], lp["y"], lw=1.2, label="PID（增益不变）")
            ax.plot(la["t"], la["y"], lw=1.6, label="LADRC（参数不变）")
            ax.plot(la["t"], la["sp"], "k--", lw=1, label="给定")
            ax.set_title(ttl); ax.set_xlabel("t [s]"); ax.set_ylabel("θ [rad]")
            ax.legend(fontsize=8); ax.grid(alpha=.3)
        fig.suptitle("实验 2  模型失配下的鲁棒性", fontsize=11)
        fig.tight_layout(); fig.savefig(_fig_path("exp2_mismatch.png"), dpi=110)
        print("\n  → 图已保存：" + _fig_path("exp2_mismatch.png"))


# ===========================================================================
# 实验 3：抗扰正面刚
# ===========================================================================
def exp3():
    hr("实验 3  抗扰正面刚：常值 vs 斜坡 vs 正弦")
    print("""
  PID 抗扰靠积分：**误差先出现，积分才涨** —— 是「事后补偿」。
  ADRC 抗扰靠 ESO：**估计扰动本身，不等误差积累** —— 是「边估边补」。

  ★ 口径说明（这一条决定你看到的结论是否成立）：
    · 给定取 0.2 rad（11.5°），**故意不让执行器饱和** ——
      饱和会把「控制律好坏」和「推力够不够」混在一起。
    · 扰动量级：最大差动力矩 Kt = 0.2 N·m，所以 d = 0.005 N·m 意味着
      「常驻吃掉了 2.5% 的差动力矩」，相当于重心偏了厘米量级 ——
      真实机型上完全可能出现，而且**飞控事先不知道它的存在**。
    · 「扰动引起的峰值偏差」已经扣掉了扰动加入前的基线误差，
      否则这个数字会被上一次阶跃留下的超调污染。

    (a) 常值  d = 0.005 N·m               （最有利于 PID 的情形）
    (b) 斜坡  d = 0.03·(t−0.8) N·m        （缓慢变化的倾转 / 油量偏移）
    (c) 正弦  d = 0.005·sin(2π·4t) N·m    （4 Hz 阵风 / 桨叶不平衡）
""")

    sp = lambda t: P_STEP      # 0.2 rad —— 不饱和，比的是抗扰不是推力
    DIST_T = 0.8               # 等阶跃过程彻底走完再加扰动（基线干净）
    T3 = 3.0
    cases = [
        ("(a) 常值 0.005 N·m", lambda t: 0.005 if t >= DIST_T else 0.0),
        ("(b) 斜坡 0.03·(t−0.8) N·m", lambda t: 0.03 * max(0.0, t - DIST_T)),
        ("(c) 正弦 4 Hz / 0.005 N·m",
         lambda t: 0.005 * math.sin(2 * math.pi * 4 * t) if t >= DIST_T else 0.0),
    ]
    for name, d in cases:
        lp = simulate(make_pid(), PLANT, sp=sp, dist=d, t_end=T3)
        la = simulate(make_adrc(), PLANT, sp=sp, dist=d, t_end=T3)
        sub(name)
        show("PID（有积分/抗饱和）", metrics(lp, t_dist=DIST_T))
        show("LADRC（ESO 补偿）", metrics(la, t_dist=DIST_T))

    print("""
  ★ 把上面三张表读成一句话，结论是**分裂**的 —— 这正是本节最想给你的东西：

    ① **(a) 常值扰动：两者几乎打平。**
       PID 的积分器对常值扰动最终也能做到零静差（末段 |e| 都是 0.0000）。
       常值扰动是**对 PID 最有利**的场景 ——
       只在这种场景下比，ADRC 的优势被严重夸大。
       注意峰值偏差：0.0107 vs 0.0089，差 20%，谈不上「完胜」。

    ② **(b) 斜坡扰动：ADRC 决定性胜出。**
       斜坡扰动下 PID 的积分项总有稳态滞后 ——
       要维持积分持续增长才能跟上，所以误差必然停在某个非零值上：
       末段 |e| = 0.0291，是给定值 0.2 rad 的 **14.6%**，
       也就是说飞机持续歪着 3 度在飞。
       ADRC 的末段 |e| = 0.0075（3.8%），**小 3.9 倍**；
       峰值偏差也小 3.3 倍。
       原因在数学上很直白（内模原理）：要无静差地跟上斜坡型扰动，
       回路里得有**两个**积分（扰动本身 + 它的变化率）。
       PID 只有「误差」一个积分，所以对斜坡型扰动只能停在常值误差上；
       ADRC 的 z3 通道里 ż3 = −ωo³e 又提供了一个积分，
       相当于给回路悄悄加了一个积分器 —— 于是误差被压下去 3.9 倍。
       （注意：压下去 3.9 倍，不是压到 0。剩下的 0.0075 是 ESO
        跟踪斜坡时的滞后，要压到 0 还得再加一阶观测器。）
       **这才是 ADRC 真正的优势场景 —— 而且它恰好是飞控最常见的场景**：
       阵风、载荷缓慢移动、油量消耗、电池电压下降……
       几乎没有哪个真实扰动是「阶跃后就不变了」的。
       实验 3 的 (a) 反而是最不真实的那一个。

    ③ **(c) 4 Hz 正弦：ADRC 只赢一点点，几乎打平。**
       末段 |e| 0.0069 vs 0.0064 —— 一样的。
       为什么？因为 ESO 本质是个**低通观测器**：
       ωo = 75 rad/s，而扰动在 4 Hz = 25 rad/s，
       只差 3 倍，ESO 对它的估计是有衰减和相移的。
       要把 4 Hz 扰动估准，ωo 得再往上抬 —— 但实验 4 会告诉你代价。
       **这就是 ADRC 唯一的调参权衡：**
       ωo 决定你愿意用多少噪声换多少抗扰。

  ⚠️ 所以「ADRC 抗扰比 PID 强」这句话**必须加限定语**：
     强在**时变 / 斜坡 / 有模型失配**的扰动上；
     常值扰动上打平，高频扰动上取决于 ωo 够不够大。
     网上那些「ADRC 完胜 PID」的对比图，多半犯了三个错之一：
     ① 拿没调好的 PID 去比；
     ② 只测常值扰动（那是最有利于 PID 的场景）；
     ③ 用无噪声的仿真，把 ωo 开到实际飞控上根本不敢用的值。""")

    if HAS_MPL:
        fig, axes = plt.subplots(1, 3, figsize=(13, 3.4))
        for ax, (name, d) in zip(axes, cases):
            lp = simulate(make_pid(), PLANT, sp=sp, dist=d, t_end=T3)
            la = simulate(make_adrc(), PLANT, sp=sp, dist=d, t_end=T3)
            ax.plot(lp["t"], lp["y"], lw=1.2, label="PID")
            ax.plot(la["t"], la["y"], lw=1.6, label="LADRC")
            ax.axhline(1.0, color="k", ls="--", lw=0.8)
            ax.axvline(DIST_T, color="r", ls=":", lw=0.8)
            ax.set_title(name, fontsize=10); ax.set_xlabel("t [s]")
            ax.grid(alpha=.3); ax.legend(fontsize=8)
        axes[0].set_ylabel("θ [rad]")
        fig.suptitle("实验 3  三种扰动下的抗扰能力", fontsize=11)
        fig.tight_layout(); fig.savefig(_fig_path("exp3_disturbance.png"), dpi=110)
        print("\n  → 图已保存：" + _fig_path("exp3_disturbance.png"))


# ===========================================================================
# 实验 4：噪声与带宽
# ===========================================================================
def exp4():
    hr("实验 4  噪声与带宽的取舍：ωo 调大为什么反而飞不好")
    print("""
  资料里反复强调（朱斌 §5.2）：
    「ωo 决定了 ESO 的跟踪速度，ωo 越大，ESO 估计扰动也越快，
      但 ωo 过大可能导致**噪声难以忍受**或 ESO 振荡，
      故其取值也取决于可接受的噪声阈值或使观测器状态产生振荡的采样延时。」

  这个实验把这句话量化：给角度测量加不同强度的噪声，扫 ωo。
  同时扫 PID 的微分滤波时间常数 τd 作为对照 ——
  你会发现**两者面临的是同一个物理权衡**（相位滞后 vs 噪声放大）。
""")

    sp = lambda t: P_STEP          # 0.2 rad，不饱和
    noises = [0.0, 0.001, 0.005, 0.020]      # 角度测量噪声 σ [rad]
    wos = [30.0, 60.0, 120.0, 250.0, 500.0]
    taus = [0.002, 0.008, 0.030, 0.100]

    sub("LADRC：扫观测器带宽 ωo   （格式：末段跟踪 RMS / 控制量 RMS）")
    print("    %-9s" % "噪声σ" + "".join("%17s" % ("ωo=%.0f" % w) for w in wos))
    for nz in noises:
        row = "    %-9.3f" % nz
        for wo in wos:
            m = metrics(simulate(make_adrc(wo=wo), PLANT, sp=sp, noise=nz))
            row += "%17s" % ("%.4f/%.3f" % (m["rms_tail"], m["u_rms"]))
        print(row)

    sub("PID：扫微分滤波 τd   （同样的权衡，换个名字）")
    print("    %-9s" % "噪声σ" + "".join("%17s" % ("τd=%.3f" % t) for t in taus))
    for nz in noises:
        row = "    %-9.3f" % nz
        for td in taus:
            m = metrics(simulate(make_pid(tau_d=td), PLANT, sp=sp, noise=nz))
            row += "%17s" % ("%.4f/%.3f" % (m["rms_tail"], m["u_rms"]))
        print(row)

    print("""
  ★ 三件事（注意：下面引用的都是表里真实读出来的数）：

    ① **ωo 的代价主要不是「跟踪变差」，而是「控制量爆炸」。**
       看 σ = 0.020 那一行：ωo 从 30 抬到 250，
       末段跟踪 RMS 从 0.0089 涨到 0.1185（13 倍），
       而控制量 RMS 从 0.039 涨到 0.893（**23 倍**，并且已经顶到 u_max = 1）。
       也就是说：ESO 的滤波作用让「角度」这一路还不算太脏，
       但**代价全部由电机承担** —— 抖舵 → 电调发热 → 电机/桨叶磨损 → 噪声更大。
       真机上这就是「参数再往上加半圈，飞机开始嗡嗡响」的那条线。

    ② **采样率给 ωo 划了一条硬上限。**
       看 σ = 0.000（完全没有噪声）那一行：ωo = 250 时末段 RMS 已经是 0.0025，
       ωo = 500 时是 0.0081、控制量 RMS 高达 0.732 ——
       **没有噪声也在抖**。原因是纯数值的：dt = 1 ms，
       ωo·dt = 0.5 已经不满足「ωo·dt ≪ 1」，
       前向欧拉离散化的 ESO 自己先振起来了。
       **这是「ADRC 在角速率环上不好落地」的直接原因之一**：
       ESO 的带宽上限是采样率的函数，不是算法的函数 ——
       角速率环 1 kHz 意味着 ωo 实际上被限制在 100~200 这个量级，
       这正是资料里邵立伟、廖晓钟、夏元清、韩京清那条结论的数值版：
       「ESO 的状态估计误差与扰动估计误差与系统的采样时间有直接关系：
        采样时间越小，ESO 的估计准确度就越高」（朱斌 §4.1.2 转述）。

    ③ **PID 面对的是同一个权衡，只不过旋钮叫 τd（微分滤波）。**
       看 PID 那张表 σ = 0.020 那一行，三个 τd 给出的是**一条帕累托前沿**：
           τd = 0.002  →  末段 RMS 0.0162，控制量 RMS 0.742
           τd = 0.008  →  末段 RMS 0.0053，控制量 RMS 0.372   ← 折中点
           τd = 0.030  →  末段 RMS 0.0112，控制量 RMS 0.164
       想要角度干净就得多抖舵，想要省舵就得容忍角度脏一点 ——
       **这个取舍在 ADRC 里一模一样地存在**，只是换了个名字叫 ωo。
       所以「ADRC 解决了调参问题」这句话是不准确的：
       它把「调 P/I/D」换成了「调 ωc/ωo/b0」，
       旋钮的物理含义更清楚（带宽就是带宽，不是「再大一点试试」），
       但**权衡本身没有消失**。
       顺便注意 τd = 0.100 那一列：0.6233 / 0.773 —— 整个回路崩了。
       **滤波不是越多越好**：τd 的相位滞后会吃掉稳定裕度。
       ωo 也有同样的问题，只不过它表现为「振荡」而不是「变慢」。

  ⚠️ **陷阱**：网上很多 ADRC 教程把 ωo 调到 1000 以上，
     然后在**无噪声、步长很小**的理想仿真里展示「完美跟踪」。
     真机上 1 kHz 的陀螺噪声会让这种参数直接把电机抖到发烫。
     这是 ADRC 在飞控上「仿真好看、装机难看」的主要原因之一。""")

    if HAS_MPL:
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        for nz, mk in zip(noises, "os^d"):
            rs = [metrics(simulate(make_adrc(wo=w), PLANT, sp=sp, noise=nz))["rms_tail"]
                  for w in wos]
            us = [metrics(simulate(make_adrc(wo=w), PLANT, sp=sp, noise=nz))["u_rms"]
                  for w in wos]
            ax[0].plot(wos, rs, mk + "-", label="σ=%.3f" % nz)
            ax[1].plot(wos, us, mk + "-", label="σ=%.3f" % nz)
        for a, ttl, yl in [(ax[0], "跟踪精度（末段 RMS）", "末段跟踪 RMS"),
                           (ax[1], "带宽的代价", "控制量 RMS（抖舵）")]:
            a.set_xscale("log"); a.set_yscale("log")
            a.set_xlabel("ωo [rad/s]"); a.set_ylabel(yl)
            a.set_title(ttl); a.grid(alpha=.3, which="both"); a.legend(fontsize=8)
        fig.suptitle("实验 4  ωo 的取舍：跟踪精度 vs 噪声放大", fontsize=11)
        fig.tight_layout(); fig.savefig(_fig_path("exp4_bandwidth.png"), dpi=110)
        print("\n  → 图已保存：" + _fig_path("exp4_bandwidth.png"))


# ===========================================================================
# 实验 5：b0 的敏感度 + 算力
# ===========================================================================
def exp5():
    hr("实验 5  b0 到底要多准 —— 以及这笔算力账")

    sub("(1) b0 偏多少会出事（扫描 b0_used / b_true）")
    print("""
  b_true = Kt/J = %.4f。LADRC 里填的 b0 和它不一致会怎样？
  资料里袁东、马晓军、曾庆含等给出的经验结论是：
    「b0 偏离 b 越远，系统相角裕度越小，当 b0/b = 4 时，系统不稳定」
    （转引自朱斌《自抗扰控制入门》§5.2.1）
""" % B_TRUE)
    ratios = [0.1, 0.2, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 10.0]
    sp = lambda t: P_STEP
    print("    %-10s %-11s %-10s %-12s %s"
          % ("b0/b_true", "超调", "末段 RMS", "峰值|θ|", "判定"))
    print("    " + "-" * (W - 6))
    for r in ratios:
        lg = simulate(make_adrc(b0=r * B_TRUE), PLANT, sp=sp, t_end=3.0)
        m = metrics(lg)
        wmax = float(np.max(np.abs(lg["y"])))
        if (not np.all(np.isfinite(lg["y"]))) or wmax > 5.0 * P_STEP:
            verdict = "❌ 发散"
        elif m["rms_tail"] > 0.10:
            verdict = "❌ 剧烈振荡"
        elif m["overshoot"] > 25.0:
            verdict = "⚠️ 明显退化"
        elif m["overshoot"] > 8.0:
            verdict = "🔸 开始振铃"
        else:
            verdict = "✅ 良好"
        print("    %-10.2f %-11s %-10.4f %-12.4f %s"
              % (r, "%.1f%%" % m["overshoot"], m["rms_tail"], wmax, verdict))

    print("""
  ★ 这张表的形状非常典型，逐段读：

    · **b0/b_true = 1.0（填对）**：超调 0.1%，末段 RMS 0.0000 —— 干净。
    · **b0 偏大 1.5 ~ 2.0**：超调 0.0 ~ 2.2%，仍然良好。
      等效于「控制量算小了」，回路增益偏低 —— **这一侧是安全侧**。
    · **b0 偏小 0.1 ~ 0.5**：超调 35% 左右，末段 RMS 0.04 —— 明显退化。
      等效于「控制量算大了」，回路增益偏高，相位裕度被吃掉。
      注意它**偏小 10 倍和偏小 2 倍的结果几乎一样差** ——
      说明主导因素不是 b0 的绝对误差，而是**回路增益高到某个程度就定住了**。
    · **b0/b_true = 3**：超调 19.7%，开始振铃。
    · **b0/b_true = 4**：超调 38.2%，末段 RMS 0.036 ——
      **这就是资料里那个「b0/b = 4 时系统不稳定」的经验阈值**，
      本脚本在完全独立的算例上复现了它（同量级）。
    · **b0/b_true ≥ 6**：超调 366% → 1740%，峰值 |θ| 冲到 3.99 rad —— 发散。

  ⚠️ **这一条对飞控特别要命**：
     b = Kt/J。**换一副桨（Kt 变）或挂一块电池（J 变），b 就变了** ——
     而上面这张表告诉你：b0 偏到 3~4 倍，系统就完蛋。
     一架 5 寸机换 7 寸桨，Kt 变化远超 4 倍。
     所以 b0 定标要么留足够裕度，要么就得**在线估计** ——
     这也是「ADRC 落地需要配套一个在线辨识器」的原因，
     算力账又要往上加。
     对比一下：PX4 的 `mc_hover_thrust_estimator` 已经在做类似的事
     （在线估悬停推力），但它估的是**配平点**，不是控制增益本身。""")

    sub("(2) 算力账：每拍要做多少次运算")
    print("""
  在 1 kHz 的姿态环里，CPU 时间是硬约束。数一下「每轴每拍」的运算量：
""")
    print("    %-46s %s" % ("控制器", "乘法次数（每轴每拍）"))
    print("    " + "-" * (W - 6))
    print("    %-46s %d" % ("PID（P+I+D+FF，含微分滤波与抗饱和）", 9))
    print("    %-46s %d" % ("LADRC（TD + 二阶 ESO + PD + 补偿）", 24))
    print("    %-46s %d + 3 次幂运算" % ("非线性 ADRC（fal 要算 |e|^a）", 30))

    n_rep = 40
    t0 = time.perf_counter()
    for _ in range(n_rep):
        simulate(make_pid(), PLANT)
    t_pid = (time.perf_counter() - t0) / n_rep
    t0 = time.perf_counter()
    for _ in range(n_rep):
        simulate(make_adrc(), PLANT)
    t_adrc = (time.perf_counter() - t0) / n_rep
    steps = T_END / DT
    print("""
    实测（本机、纯 Python、%d 次平均，每次 %.0f 拍）：
      PID   ：%6.2f ms/次  →  %6.3f µs/拍
      LADRC ：%6.2f ms/次  →  %6.3f µs/拍      （%.2f × PID）
""" % (n_rep, steps, t_pid * 1e3, t_pid / steps * 1e6,
       t_adrc * 1e3, t_adrc / steps * 1e6, t_adrc / t_pid))

    print("""  ★ 说明：

    · 纯 Python 的绝对数字没有意义，**比值才有意义**。
      这次实测是 **%.2f 倍**；按「乘法次数」算是 24/9 ≈ 2.7 倍。
      两者差距来自 Python 的循环与调用开销（它把比值稀释了）——
      真机上没有这层开销，所以**按乘法次数估更接近真相**。

    · 换算成真实的 MCU 成本：姿态环按 1 kHz × 3 轴 = 3000 次/秒，
      每次多 15 次乘加 ⇒ 每秒多 4.5 万次浮点运算。
      对一颗 480 MHz 的 STM32H7 来说**完全不算事**（不到 0.1%%）。
      **所以「算力不够」不是 ADRC 没进飞控的理由** ——
      真正的理由是下面第三条。

    · **非线性 ADRC 才是真的贵。** fal 函数要算 |e|^a，
      在带 FPU 的 Cortex-M7 上是一次 powf（几十到上百周期），
      在 M4F 上更贵，而且它的执行时间**依赖于输入值**（数据相关抖动）——
      对硬实时控制回路来说，这比「多花 10 个周期」更难接受。
      这是「非线性 ADRC 很少上量产飞控」的直接原因。

    · **真正贵的不是乘加，是它带来的连锁反应**：
      多出来的状态要进日志（否则出事故无法归因）、
      要多一组参数进参数系统、要改调参工具、要有一个辨识器定 b0。
      这些「非算法成本」才是飞控不换控制律的真正原因 ——
      详见本章正文 9.4.4 第 5 节。""" % (t_adrc / t_pid))


# ===========================================================================
EXPERIMENTS = {
    "1": ("ADRC 到底长什么样：TD + ESO + PD", exp1),
    "2": ("模型失配：J ×3、Kt ×0.5", exp2),
    "3": ("抗扰正面刚：常值 / 斜坡 / 正弦", exp3),
    "4": ("噪声与带宽的取舍：扫 ωo", exp4),
    "5": ("b0 敏感度扫描 + 算力账", exp5),
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
        return
    for a in args:
        if a in EXPERIMENTS:
            print("\n" + "#" * W)
            print("#  实验 %s：%s" % (a, EXPERIMENTS[a][0]))
            print("#" * W)
            EXPERIMENTS[a][1]()
        else:
            print("没有实验 %s。可选：%s" % (a, ", ".join(sorted(EXPERIMENTS))))


if __name__ == "__main__":
    main()
