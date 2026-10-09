#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 11 第 5 章 —— 创新实践：在线电机效率辨识（四步法完整走一遍）

    实验 1 ★   复现：把 PX4 的 zero_order_hover_thrust_ekf 逐行翻译成 Python
    实验 2 ★★  可辨识性：悬停 vs 机动，Fisher 信息 / 条件数 / CRLB
    实验 3 ★★  注入缺陷：电机效率下降 / 整机变重 / 桨叶损伤
    实验 4 ★★★ 改进方案 + 消融实验
    实验 5 ★★★ 诚实报告：改进方案在什么条件下失效

用法：
    python lesson28_innovation.py 1
    python lesson28_innovation.py 2
    python lesson28_innovation.py 3
    python lesson28_innovation.py 4
    python lesson28_innovation.py 5
    python lesson28_innovation.py            # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则：数字必须能对上源码】

  第 1 步「复现」不是「大概像」，而是**能过 PX4 自己的单元测试**。
  所以实验 1 的判据直接抄 PX4 的测试断言：

      src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf_test.cpp

  那几个 EXPECT_NEAR 就是验收标准。过了，才说明你读懂了；
  没过，先别往下走 —— 第 2 步之后的分析都建立在「我懂原算法」之上。

【本章的核心线索：可辨识性】

  贯穿全课的那个问题：

      **悬停时，你能区分「1 号电机坏了」和「整机变重了」吗？为什么？**

  答案是**不能**，而且这个「不能」是可以写成矩阵的：
  两者在垂直通道上作用在**同一个方向**上（回归量共线），
  Fisher 信息矩阵**奇异** → 不可辨识（§实验 2）。

  由此推出本章的全部结构：
    · 为什么需要激励（机动）        → 实验 2
    · 为什么 PX4 的估计器沉默地吸收故障 → 实验 3
    · 改进方案为什么能行、能行到哪一步 → 实验 4
    · 改进方案在哪里仍然失效        → 实验 5（本章最有价值的一节）

【关于「真实日志」】

  本脚本只依赖 numpy，因此**不读 .ulg**。真实日志的结论以实测常数的形式
  写在实验 1 的 L3 段（数据来源：PX4调参/ulog/20250922/.../log_13_*.ulg）。
  复现日志的完整过程在课文里，脚本只负责可复算的那部分。

参考：
    本教程 卷 11 第 4 章  §9.3, §9.8
    本教程 卷 2 第 10 章         §15.5.6, §15.5.7
    本教程 09-进阶与创新/code/lesson27_adrc.py           （脚本风格范本）
"""

import math
import os
import sys

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


W = 78
G = 9.80665


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


def table(headers, rows, widths=None, indent=2):
    """等宽中文表格。宽度按【显示宽度】算 —— 中文占 2 列。"""
    def dw(s):
        return sum(2 if ord(c) > 0x2E7F else 1 for c in str(s))

    def pad(s, w):
        return str(s) + " " * max(0, w - dw(s))

    cols = len(headers)
    if widths is None:
        widths = [0] * cols
        for j in range(cols):
            widths[j] = max(dw(headers[j]),
                            max(dw(r[j]) for r in rows) if rows else 0)
    pre = " " * indent
    print(pre + "  ".join(pad(h, widths[j]) for j, h in enumerate(headers)))
    print(pre + "  ".join("-" * widths[j] for j in range(cols)))
    for r in rows:
        print(pre + "  ".join(pad(c, widths[j]) for j, c in enumerate(r)))


# ===========================================================================
# 0. 被辨识的对象：一架 5 寸四旋翼
# ===========================================================================
#
#   坐标：x 前，y 右，z 下（NED）。桨在机体 z 方向出力，方向向上（-z）。
#
#   电机布局（quad X，PX4 编号）：
#
#           机头
#        4 (FL)  1 (FR)
#             \  /
#             /  \
#        3 (RL)  2 (RR)
#
#   位置： p1 = (+l, +l)   p2 = (-l, +l)   p3 = (-l, -l)   p4 = (+l, -l)
#   转向： σ = [-1, +1, -1, +1]（反扭矩符号，1 号与 3 号同向）
#
#   单个电机的静态推力：        F_i = c_i · u_i          （u_i ∈ [0,1] 归一化指令）
#   反扭矩：                    Q_i = κ · c_i · u_i      （κ 无量纲，桨的阻扭比）
#
#   于是四个「扳手」分量：
#
#       F_up = Σ F_i
#       M_roll  = -l·(F1 + F2 - F3 - F4)        ← 绕 +x（右滚为正）
#       M_pitch =  l·(F1 - F2 - F3 + F4)        ← 绕 +y（抬头为正）
#       M_yaw   =  κ·(σ1F1 + σ2F2 + σ3F3 + σ4F4)
#
#   ★ 这 4 行就是 §9.3 说的那个「效率矩阵 B」。
#     在线辨识的全部问题，就是这个 4×4 矩阵里的 c_i 会不会变 ——
#     而 PX4 / ArduPilot 都当它是**常数**。
#
L_ARM = 0.125          # 力臂（对角轴到电机），m
KAPPA = 0.016          # 桨的阻扭比 Q/F（5 寸桨约 0.015~0.02）
C_NOM = 8.0            # 单个电机满油门推力，N（4 个共 32 N）
M_NOM = 1.5            # 整机质量，kg
SIGMA = np.array([-1.0, 1.0, -1.0, 1.0])
POS = np.array([[1.0, 1.0], [-1.0, 1.0], [-1.0, -1.0], [1.0, -1.0]])

# 标称悬停油门：mg = 4·C·Th  →  Th = mg/(4C)
TH_HOVER = M_NOM * G / (4.0 * C_NOM)
# 单位指令 → 垂直加速度的标称增益（悬停线性化点附近）
ACC_PER_U = C_NOM / M_NOM


def wrench(u, c=None, kap=None):
    """输入归一化指令 u[4]，返回 (M_roll, M_pitch, M_yaw, F_up)（真实值）。

    c   为各电机的**真实**效率（N / 单位指令）；缺省用标称值。
    kap 为各电机的**真实**阻扭比相对系数（桨叶损伤会改这一项）。
    故障就等于改 c（和 kap）。
    """
    if c is None:
        c = np.full(4, C_NOM)
    if kap is None:
        kap = np.ones(4)
    F = np.asarray(c, float) * np.asarray(u, float)
    m_roll = -L_ARM * (F[0] + F[1] - F[2] - F[3])
    m_pitch = L_ARM * (F[0] - F[1] - F[2] + F[3])
    m_yaw = KAPPA * float(np.dot(SIGMA * np.asarray(kap, float), F))
    return m_roll, m_pitch, m_yaw, float(F.sum())


# PX4 里 B 矩阵是**离线标定常数**，用它求伪逆得到混控：
#     u = B⁺ · τ_desired
# 外面下发的力矩，实际产生多少，取决于真实 c —— 这就是「控制分配的无知」。
def _B(arm):
    """效率矩阵 B(arm)：行 = (roll, pitch, yaw, thrust)，列 = 4 个电机。

    ★ arm 是**混控器以为的**力臂。真机的力臂永远是 L_ARM
      （`wrench()` 里写死的那个）。实验 5.2 就是让 arm ≠ L_ARM。
    """
    return np.array([
        [-arm, -arm, arm, arm],             # roll
        [arm, -arm, -arm, arm],             # pitch
        [-KAPPA, KAPPA, -KAPPA, KAPPA],     # yaw
        [1.0, 1.0, 1.0, 1.0],               # thrust（已含 c 归一化）
    ]) * C_NOM


B_NOM = _B(L_ARM)
B_INV = np.linalg.pinv(B_NOM)
_BINV_CACHE = {}


def _binv(arm):
    key = round(float(arm), 9)
    if key not in _BINV_CACHE:
        _BINV_CACHE[key] = np.linalg.pinv(_B(arm))
    return _BINV_CACHE[key]


def mix(tau, arm=L_ARM):
    """标称混控：力矩+总推力 → 4 个归一化指令。

    ★ PX4 里 B 是**编译期常数**（ControlAllocationPseudoInverse 对
      _effectiveness 求伪逆），OEM 标定完就冻住了：既没有「力臂标定」
      这个可调参数，也不会随电池电压 / 桨磨损 / 空气密度变化。
      `arm` 这个入口就是把这件事暴露出来用的（实验 5.2）。
    """
    return _binv(arm) @ np.asarray(tau, float)


# ===========================================================================
# 1. 复现：PX4 zero_order_hover_thrust_ekf 的 Python 翻译
# ===========================================================================
#
#   逐行对应（行号取自 PX4-Autopilot-1.15.4）：
#
#     src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp
#       :44-50   predict()          —— 状态是常量，只推协方差：P += Q·dt²
#       :52-82   fuseAccZ()         —— 门限 → 更新 or 抬方差 → LPF → 学 R
#       :84-87   computeH()         —— H = -g·T/Th²
#       :89-94   computeInnovVar()  —— S = max(H·P·H + R, R)
#       :96-105  computeInnov / computePredictedAccZ
#       :107-110 computeKalmanGain()—— K = P·H/S
#       :112-115 computeInnovTestRatio() —— NIS: innov²/(gate²·S)
#       :122-130 updateState / updateStateCovariance
#       :132-140 isLargeOffsetDetected / bumpStateVariance
#       :142-148 updateLpf()
#       :150-156 updateMeasurementNoise()
#
#     src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.hpp
#       :97-106  默认初值：Th=0.5, P=0.01, Q=12.5e-6, R=5, gate=3,
#                Th∈[0.1,0.9], dt=0.02
#       :137-138 两个 LPF 时间常数：噪声学习 2 s，残差 LPF 1 s
#
#   ★ 量测模型为什么长这样：
#       h(u, Th) = g·T/Th - g
#     T = Th 时 h = 0（悬停，垂直加速度为 0）；T > Th 时向上加速。
#     这是把「推力—加速度」关系在悬停点做**仿射化**：
#         a_up = g·(T/Th - 1)   ⇔   T = Th·(1 + a_up/g)
#     和 mc_pos_control/PositionControl/PositionControl.cpp:217 那条
#     「thrust_ned_z = a_sp·(Th/g) - Th」是同一个式子的两面。
#     ⚠️ 陷阱：真的推力曲线是 T ∝ u²（或 THR_MDL_FAC 混合模型），
#        这里用的是**线性**模型。a 越大，模型误差越大 —— 这一条
#        在实验 5 里会变成改进方案的一个真实失效点。
#
class ZeroOrderHoverThrustEkf:
    """PX4 zero_order_hover_thrust_ekf 的逐行 Python 翻译（float64）。"""

    def __init__(self, dt=0.02, hover_thr=0.5, state_var=0.01,
                 process_var=12.5e-6, acc_var=5.0, gate=3.0,
                 hover_thr_min=0.1, hover_thr_max=0.9):
        self.th = hover_thr
        self.P = state_var
        self.Q = process_var
        self.R = acc_var
        self.R_scale = 1.0
        self.gate = gate
        self.th_min = hover_thr_min
        self.th_max = hover_thr_max
        self.dt = dt
        self.res_lpf = 0.0
        self.s_lpf = 0.0
        self.NOISE_TAU = 2.0
        self.LPF_TAU = 1.0
        # 供日志用
        self.innov = 0.0
        self.innov_var = 0.0
        self.innov_test_ratio = 0.0
        self.bumped = 0
        self.rejected = 0

    # ---- cpp:44-50 ----
    def predict(self, dt):
        self.P += self.Q * dt * dt
        self.dt = dt

    # ---- cpp:84-87 ----
    def _H(self, thrust):
        return -G * thrust / (self.th * self.th)

    # ---- cpp:52-82 ----
    def fuse_acc_z(self, acc_z, thrust):
        H = self._H(thrust)
        R = self.R * self.R_scale
        S = max(H * self.P * H + R, R)                    # cpp:89-94
        innov = acc_z - (G * thrust / self.th - G)        # cpp:96-105
        K = self.P * H / S                                # cpp:107-110
        ratio = innov * innov / (self.gate * self.gate * S)   # cpp:112-115

        residual = innov
        if ratio < 1.0:                                   # cpp:117-120
            self.th = min(max(self.th + K * innov, self.th_min), self.th_max)
            self.P = min(max((1.0 - K * H) * self.P, 1e-10), 1.0)
            # ★ residual 用**更新后**的 Th 重算（cpp:65）—— 这一点很容易漏，
            #   漏了 R 的学习轨迹就会偏（实验 1 的 L2 段会拿日志验证这条）
            residual = acc_z - (G * thrust / self.th - G)
        else:
            self.rejected += 1
            if abs(self.s_lpf) > 0.2:                     # cpp:132-140
                self.P += 1e3 * self.Q * self.dt * self.dt
                self.bumped += 1

        a = self.dt / (self.LPF_TAU + self.dt)            # cpp:142-148
        self.res_lpf = (1 - a) * self.res_lpf + a * residual
        sgn = math.copysign(1.0, innov) * ratio
        self.s_lpf = (1 - a) * self.s_lpf + a * min(max(sgn, -1.0), 1.0)

        a2 = self.dt / (self.NOISE_TAU + self.dt)         # cpp:150-156
        self.R = min(max((1 - a2) * self.R
                         + a2 * ((residual - self.res_lpf) ** 2 + H * self.P * H),
                         1.0), 400.0)

        self.innov = innov
        self.innov_var = S
        self.innov_test_ratio = ratio
        return K


def run_px4_ekf(ht_true, thrust_seq, dt=0.02, noise=0.0, seed=0,
                accel_bias=0.0, t_sim=None, thr_noise=0.0, ekf=None, **kw):
    """按 PX4 单元测试的方式跑：每步 predict(dt) 再 fuse(acc, thrust)。

    thrust_seq 标量时按 t_sim 秒循环（PX4 测试就是「跑 2 秒 / 10 秒」），
    也可以直接给一条逐点序列。
    ekf 传入已有实例时**接着它的状态往下跑** —— testHoverThrustJump
    就是这么用的（同一个 _ekf 成员连着跑两段）。
    """
    if ekf is None:
        ekf = ZeroOrderHoverThrustEkf(dt=dt, **kw)
    rng = np.random.default_rng(seed)
    if hasattr(thrust_seq, "__len__"):
        thr_arr = np.asarray(thrust_seq, float)
    else:
        assert t_sim is not None, "标量推力必须给 t_sim（仿真时长，秒）"
        n = int(round(t_sim / dt)) + 1
        thr_arr = np.full(n, float(thrust_seq))
    hist = np.zeros(len(thr_arr))
    for k in range(len(thr_arr)):
        ekf.predict(dt)
        T = thr_arr[k]
        if thr_noise:
            T = T + thr_noise * rng.standard_normal()
        acc_true = G * thr_arr[k] / ht_true - G
        acc_meas = acc_true + accel_bias
        if noise:
            acc_meas += noise * rng.standard_normal()
        ekf.fuse_acc_z(acc_meas, T)
        hist[k] = ekf.th
    return ekf, hist


def exp1():
    hr("实验 1  复现：把 zero_order_hover_thrust_ekf 翻译成 Python")

    print("""
  复现的判据不是「看起来像」，而是**能过 PX4 自己的单元测试**。
  验收标准直接抄源码里那几个 EXPECT_NEAR：
      src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf_test.cpp

  ⚠️ 下文所有 `cpp:行号` / `hpp:行号` 都是简写，指同目录下这两个文件：
      zero_order_hover_thrust_ekf.cpp
      zero_order_hover_thrust_ekf.hpp
""")

    # 六个用例的参数全部抄自测试文件，一个不漏
    # （前面那个 ZeroOrderHoverThrustEkf _ekf{} 是用**类默认值**构造的，
    #   所以上下限是 hpp:98-99 的 0.1/0.9，而不是 MPC_THR_HOVER±HTE_THR_RANGE）
    cases = []

    # ---- testStaticCase  (:108-122) ----
    ekf, h_case = run_px4_ekf(0.5, 0.5, dt=0.02, seed=42, t_sim=2.0)
    cases += [
        ("testStaticCase", "hover_thrust", "0.5000", "%.6f" % ekf.th, "1e-4",
         abs(ekf.th - 0.5) < 1e-4),
        ("testStaticCase", "hover_thrust_var", "0  (1e-3)", "%.3e" % ekf.P, "1e-3",
         abs(ekf.P) < 1e-3),
        ("testStaticCase", "accel_noise_var", "1.0 (1.0)", "%.4f" % ekf.R, "1.0",
         abs(ekf.R - 1.0) < 1.0),
    ]

    # ---- testStaticConvergence  (:124-138) ----
    ekf2, h2 = run_px4_ekf(0.72, 0.72, dt=0.02, seed=42, t_sim=2.0)
    cases += [
        ("testStaticConvergence", "hover_thrust", "0.7200 (1e-2)", "%.6f" % ekf2.th,
         "1e-2", abs(ekf2.th - 0.72) < 1e-2),
        ("testStaticConvergence", "hover_thrust_var", "0  (1e-3)", "%.3e" % ekf2.P,
         "1e-3", abs(ekf2.P) < 1e-3),
        ("testStaticConvergence", "accel_noise_var", "1.0 (1.0)", "%.4f" % ekf2.R,
         "1.0", abs(ekf2.R - 1.0) < 1.0),
    ]

    # ---- testStaticConvergenceWithNoise  (:140-156)  σ=3, 10 s ----
    ekf3, _ = run_px4_ekf(0.72, 0.72, dt=0.02, noise=3.0, seed=42, t_sim=10.0)
    cases += [
        ("testStaticConvergenceWithNoise", "hover_thrust", "0.7200 (5e-2)",
         "%.6f" % ekf3.th, "5e-2", abs(ekf3.th - 0.72) < 5e-2),
        ("testStaticConvergenceWithNoise", "accel_noise_var", "9.0 (1.8)",
         "%.4f" % ekf3.R, "1.8", abs(ekf3.R - 9.0) < 1.8),
    ]

    # ---- testLargeAccelNoiseAndBias  (:158-174)  σ=7, thrust=0.4(<悬停), 15 s ----
    ekf4, _ = run_px4_ekf(0.72, 0.40, dt=0.02, noise=7.0, seed=42, t_sim=15.0)
    cases += [
        ("testLargeAccelNoiseAndBias", "hover_thrust", "0.7200 (7e-2)",
         "%.6f" % ekf4.th, "7e-2", abs(ekf4.th - 0.72) < 7e-2),
        ("testLargeAccelNoiseAndBias", "accel_noise_var", "49.0 (9.8)",
         "%.4f" % ekf4.R, "9.8", abs(ekf4.R - 49.0) < 9.8),
    ]

    # ---- testThrustAndAccelNoise  (:176-195)  accel σ=2, thrust σ=0.01, 15 s ----
    ekf5, _ = run_px4_ekf(0.60, 0.72, dt=0.02, noise=2.0, thr_noise=0.01,
                          seed=42, t_sim=15.0)
    cases += [
        ("testThrustAndAccelNoise", "hover_thrust", "0.6000 (5e-2)",
         "%.6f" % ekf5.th, "5e-2", abs(ekf5.th - 0.60) < 5e-2),
        ("testThrustAndAccelNoise", "accel_noise_var", "4.0 (1.6)",
         "%.4f" % ekf5.R, "1.6", abs(ekf5.R - 4.0) < 1.6),
    ]

    # ---- testHoverThrustJump  (:197-222)  同一个 _ekf 连着跑两段 ----
    ekf6, _ = run_px4_ekf(0.80, 0.80, dt=0.02, noise=2.0, thr_noise=0.01,
                          seed=42, t_sim=10.0)
    ekf6, _ = run_px4_ekf(0.30, 0.30, dt=0.02, noise=2.0, thr_noise=0.01,
                          seed=43, t_sim=10.0, ekf=ekf6)
    cases += [
        ("testHoverThrustJump", "hover_thrust (跳变后)", "0.3000 (5e-2)",
         "%.6f" % ekf6.th, "5e-2", abs(ekf6.th - 0.30) < 5e-2),
        ("testHoverThrustJump", "accel_noise_var", "4.0 (8.0)",
         "%.4f" % ekf6.R, "8.0", abs(ekf6.R - 4.0) < 8.0),
    ]

    sub("L1  单元测试级：PX4 的 6 个用例逐个复算（RNG 换了种子，不比逐位）")
    rows = [[c[0], c[1], c[2], c[3], c[4], "通过" if c[5] else "失败"]
            for c in cases]
    table(["PX4 用例", "被检查的量", "PX4 断言值(容差)", "本脚本",
           "容差", "判定"], rows)
    npass = sum(1 for c in cases if c[5])
    print("  复算结果：%d / %d 条断言通过。" % (npass, len(cases)))

    err2 = abs(ekf2.th - 0.72)
    n1 = int((np.abs(h2 - 0.72) < 1e-2).argmax())
    print("""
  ★ 复现误差（用最严的那条确定性用例）：|%.6f − 0.72| = %.3e，
    相对误差 %.4f%% —— **比 §9.8 要求的「< 1%%」小一个数量级**。
    （这条用例之所以是「最严」的，是因为它不带噪声：
      任何公式抄写错误都会在这里暴露成 1e-2 以上的偏差。）

  ★ 但注意「2 秒」这个数字是怎么来的：从 0.5 爬到 0.72 的 1%% 带内要
    %d 步 ≈ %.2f s。这不是笔误，是**单状态滤波器的固有速度**：
    越接近目标，新息越小，每步只挪 K·innov，指数收敛。
    PX4 把测试时长取成 2 s 正是照着这个速度定的 ——
    ★ 而这个「慢」，就是后面实验 2 里「信息量不足」的另一种说法。
""" % (ekf2.th, err2, 100 * err2 / 0.72, n1, n1 * 0.02) + """
  ⚠️ 陷阱：很多人「复现」时把 R 写死成常数。这里必须让 R 自己学
     （cpp:150-156），否则 testStaticConvergenceWithNoise 的
     accel_noise_var 断言（9.0 ± 1.8）根本过不了。
     同理，漏掉 cpp:65 那个「用更新后的 Th 重算 residual」也会让
     R 的学习轨迹整体偏移 —— 这两处是复现时最容易漏的两行。
""")

    # ---- Monte Carlo 稳健性 ----
    sub("L1'  再加一层：把 PX4 的单次随机实现换成 200 次 Monte Carlo")
    print("  PX4 的断言是「固定 seed=42、单次实现、落在容差内」。")
    print("  单次通过不代表统计上稳。用 200 个不同种子重跑：")
    rows = []
    for tag, sig, thr, ht, t_sim, tol_ht, tol_R, thrn in [
            ("testStaticConvergenceWithNoise", 3.0, 0.72, 0.72, 10.0, 5e-2, 0.2, 0.0),
            ("testLargeAccelNoiseAndBias", 7.0, 0.40, 0.72, 15.0, 7e-2, 0.2, 0.0),
            ("testThrustAndAccelNoise", 2.0, 0.72, 0.60, 15.0, 5e-2, 0.4, 0.01),
    ]:
        n = 200
        ths = np.zeros(n)
        Rs = np.zeros(n)
        for s in range(n):
            ee, _ = run_px4_ekf(ht, thr, dt=0.02, noise=sig, thr_noise=thrn,
                                seed=1000 + s, t_sim=t_sim)
            ths[s] = ee.th
            Rs[s] = ee.R
        rows.append([tag, "%.4f" % ht,
                     "%.4f ± %.4f" % (ths.mean(), ths.std()),
                     "±%g" % tol_ht,
                     "%.1f%%" % (100 * (np.abs(ths - ht) < tol_ht).mean()),
                     "%.1f%%" % (100 * (np.abs(Rs - sig * sig)
                                        < tol_R * sig * sig).mean())])
    table(["用例", "真值", "hover_thrust (200 次)", "容差", "ht 覆盖率",
           "R 覆盖率"], rows)
    print("""
  ★ 复现的正确姿势：**先用确定性用例逐位对齐，再看统计用例**。
    确定性用例对不上 → 公式抄错了；
    统计用例对不上 → 收敛速度/R 学习律的理解有偏差。
    两者是完全不同的问题，混在一起查会很痛苦。

  ★ 顺带说明：第 3 行（同时有推力噪声）的 R 覆盖率只有 8 成左右 ——
    这不是我们写错了，是 PX4 自己在测试注释里写明的：
      「Because of the nonlinear measurment model and the thrust noise,
        the accel noise estimation is a bit worse」
    所以它把容差从 0.2 放宽到 0.4。**读源码的注释，能省你半天调试。**

  ★ 这里 R 的收敛值就是「R 学习律」在干活，公式见 cpp:150-156：
        acc_var ← (1−α)·acc_var + α·[(res − res_lpf)² + H·P·H] ,  α = dt/(2+dt)
    它本质是**新息协方差匹配**（Sage-Husa 型自适应）。
    本教程 卷 2 第 10 章 §15.5.7 已经把这个
    方法从原理到数值稳定性完整推导过一遍，这里**不再重推**，
    只把它当成「PX4 已经写好的现成组件」来调用。
""")

    # ---- 真实日志 ----
    sub("L2  结构级：拿真实飞行日志核对中间量（不需要任何内部状态）")
    print("""
  数据来源（本机真实日志，53 份 .ulg 里最长的一份）：
    PX4调参/ulog/20250922/2025_09_22_log_13_2025-9-22-16-59-20/log_13_*.ulg
    hover_thrust_estimate 共 28208 条，10 Hz，实测稳态 hover_thrust = 0.4802

  办法：日志里的 accel_innov / accel_innov_var / accel_noise_var /
  hover_thrust_var 都是滤波器的**中间量**，可以直接拿源码公式去核对 ——
  这一步不需要复现任何内部状态，所以判据是硬的。
""")
    rows = [
        ["innov_var = max(H·P·H + R, R)", "cpp:89-94",
         "反推 R/acc_var 中位数 1.0029",
         "p1 = 0.970，p99 = 1.005", "一致"],
        ["ratio = innov²/(gate²·innov_var)", "cpp:112-115",
         "逐点相对误差 < 1e-6", "28206 条全部成立", "一致"],
        ["P ← (1-K·H)·P", "cpp:127-130",
         "单步相对误差 ~1e-5", "float32 舍入量级", "一致"],
        ["R ← (1-α)R + α(res² + H·P·H)", "cpp:150-156",
         "隐含 P = 2.5e-4", "与从 0.01 缓慢衰减的理论值同阶", "一致"],
    ]
    table(["源码公式", "位置", "实测", "备注", "判定"], rows)

    sub("L3  闭环轨迹级：连续回放 24807 步")
    print("""
  把日志里每一步的 (a_z, T) 反推出来喂给本脚本的实现，连续跑：
""")
    rows = [
        ["单步对照（用固件上一步的日志状态做初值）", "1.53e-4", "0.032 %", "通过"],
        ["闭环回放（24807 步，自己带状态往前跑）", "6.90e-3", "1.44 %", "不通过"],
    ]
    table(["判据", "rms 误差", "相对 0.48", "结论"], rows)
    print("""
  ★★ 这里必须诚实：**闭环回放的 1.44% 说明「误差 < 1%」这条验收线没过。**

    但差在哪，是查得出来的 —— 而且这才是复现这一步真正的产出：

      1. zero_order_hover_thrust_ekf.hpp:112-113 的
         _residual_lpf / _signed_innov_test_ratio_lpf **两个内部状态不进日志**，
         而它们喂进 R 的学习律（cpp:150-156）。R 差 5/26 → K 差 → 缓漂。
      2. 更强的证据：日志里的 accel_innov_var 与**同序号**的 accel_noise_var
         相关系数 0.9984，而与上一步的只有 0.9724 —— 按 1.15.4 源码
         computeInnovVar 应该用上一步的 _acc_var。⇒ **这份日志的固件
         与手头这份 1.15.4 源码在更新顺序上不一致**（或字段语义不同）。

    ⇒ 第 1 步的正确产出是两条，不是一条：
        (a) 「我的方程是对的」—— L1 的 0.032% 证明了；
        (b) 「日志缺一个可观测性字段」—— 这条直接变成改进点：
            把 _residual_lpf 加进 hover_thrust_estimate 消息。

    ★ 这也是全课主题的第一次出现：**不可观测的内部状态，你既复现不出来，
      也辨识不出来**。后面实验 2 会用 Fisher 信息把这个直觉写成矩阵。
""")


# ===========================================================================
# 2. 可辨识性：悬停 vs 机动
# ===========================================================================
#
#   把「1 号电机效率下降 δ」和「整机变重 ε」写成同一个回归问题。
#
#   真实垂直加速度（悬停附近线性化，忽略二阶项）：
#
#       a_z = (Σ c_i u_i)/m - g
#           = (c̄/m̄)·(U - δ·u_1 - ε·U) - g          U = Σ u_i
#
#   于是「残差」y_k = a_z,k - (c̄/m̄)U_k + g 满足
#
#       y_k = φ_kᵀ θ + noise ,   φ_k = -(c̄/m̄)·[u_1,k , U_k] ,  θ = [δ, ε]
#
#   ★★★ 全部问题就在这个 φ_k 上。
#
#     · 悬停：u_1 = Th, U = 4Th  →  φ ∝ (1, 4)   ← **只有一个方向**
#     · 纯升降（四个电机一起加 Δ）：u_1 = Th+Δ, U = 4(Th+Δ) → φ ∝ (1,4)
#       ⇒ 方向没变！**升降不增加任何信息**（反直觉，但这就是线性代数）
#     · 滚转/俯仰/偏航机动：u_1 = Th+Δ, U = 4Th → φ ∝ (Th+Δ, 4Th)
#       ⇒ 方向变了 ⇒ 两个参数才可能分开
#
#   Fisher 信息矩阵（白噪声 σ²，N 个采样）：
#
#       F = (1/σ²)·Σ_k φ_k φ_kᵀ      CRLB:  Cov(θ̂) ⪰ F⁻¹
#
#   悬停时 F 秩 1 → 奇异 → **不可辨识**。
#   判别指标：条件数 κ(F)，以及 θ̂ 的相关系数 ρ(δ̂, ε̂)。
#
P_MASS = 1.5
SIG_A = 1.0            # 用于辨识的「有效」垂直加速度噪声，m/s²（见下）


def info_matrix(profiles, sigma=SIG_A):
    """profiles: [(u1, U), ...] 逐采样的回归量；返回 Fisher 信息矩阵。

    ★ 回归量里必须带上 -(c̄/m̄) = -ACC_PER_U 这个增益 ——
      漏了它，σ(δ̂) 就会整体偏大一个 ACC_PER_U 倍（这是复现类脚本的经典坑）。
    """
    F = np.zeros((2, 2))
    for u1, U in profiles:
        phi = -ACC_PER_U * np.array([u1, U])
        F += np.outer(phi, phi)
    return F / (sigma * sigma)


def ident_metrics(F):
    """由 Fisher 信息算条件数、CRLB 标准差、相关系数。

    ★ 秩判定必须用**相对**阈值：绝对阈值 1e-300 会把「数值上秩 1」
      误判成「病态但可逆」，于是 κ 报出一个 1e14 的天文数字而不是 ∞ ——
      看起来像「很难辨识」，实际上根本是「不可辨识」，性质完全不同。
    """
    ev = np.linalg.eigvalsh(F)
    if ev[1] <= 0 or ev[0] <= 1e-12 * ev[1]:
        return np.inf, np.inf, np.inf, 1.0
    kappa = ev[1] / ev[0]
    C = np.linalg.inv(F)
    sd = np.sqrt(np.diag(C))
    rho = C[0, 1] / math.sqrt(C[0, 0] * C[1, 1])
    return kappa, sd[0], sd[1], rho


def sigma_delta_ideal(n, sigma=SIG_A):
    """理想下限：假如 δ 只有一个回归量、且没有共线（'正交化' 后的最好情况）。

    σ(δ̂) ≥ σ_a / ( (c̄/m̄)·sqrt(Σ u_1²) )
    """
    return sigma / (ACC_PER_U * TH_HOVER * math.sqrt(n))


def make_profile(n, duty, amp, seed=0, collective_only=False):
    """造一段飞行剖面：悬停为主 + duty 比例的机动。

    duty=0       纯悬停
    duty=0.2     20% 时间做姿态机动（差动幅度 amp）
    collective_only=True 时用「纯升降」代替机动（对照用）
    """
    rng = np.random.default_rng(seed)
    prof = []
    for k in range(n):
        if rng.random() < duty:
            if collective_only:
                d = amp          # 四个电机一起动 → 共模，不增加信息
                u = TH_HOVER + d
                prof.append((u, 4.0 * u))
            else:
                # 差动：1 号与对角一个加一个减 → U 不变，但 u_1 变了
                # ★ 幅度取 ± 随机：真实的滚转是「左滚 + 右滚」，
                #   如果只往一个方向偏，那只是换了个工作点，不增加信息
                d = amp * (1.0 if rng.random() < 0.5 else -1.0)
                prof.append((TH_HOVER + d, 4.0 * TH_HOVER))
        else:
            prof.append((TH_HOVER, 4.0 * TH_HOVER))
    return prof


def exp2():
    hr("实验 2  可辨识性：悬停 vs 机动（Fisher 信息 / 条件数 / CRLB）")

    N = 3000
    print("""
  场景：要同时估两个量
         δ = 1 号电机效率相对下降（差模故障）
         ε = 整机质量相对增加（共模「故障」）
  观测：垂直加速度（等效噪声 σ = %.1f m/s²，见本实验末尾的说明）
  激励：N = %d 个采样，其中 duty 比例的时间在做机动

  ★ 先看一个反直觉的对照：把「机动」换成「四个电机一起升降」。
""" % (SIG_A, N))

    sub("2.1  条件数与 CRLB —— 数字说话")
    rows = []
    for tag, duty, amp, coll in [
            ("纯悬停", 0.00, 0.0, False),
            ("20% 纯升降（共模激励）", 0.20, 0.05, True),
            ("20% 姿态机动（差模激励）", 0.20, 0.05, False),
            ("50% 姿态机动", 0.50, 0.05, False),
            ("100% 姿态机动", 1.00, 0.05, False),
    ]:
        F = info_matrix(make_profile(N, duty, amp, seed=7, collective_only=coll))
        kappa, sd_d, sd_e, rho = ident_metrics(F)
        rows.append([tag,
                     "∞" if not np.isfinite(kappa) else "%.3e" % kappa,
                     "∞" if not np.isfinite(sd_d) else "%.4f" % sd_d,
                     "∞" if not np.isfinite(sd_e) else "%.4f" % sd_e,
                     "%.4f" % rho])
    table(["激励方式", "κ(F)", "σ(δ̂) [1σ]", "σ(ε̂) [1σ]", "ρ(δ̂,ε̂)"], rows)

    print("""
  ★★ 三行结论，每一行都重要：

    ① 纯悬停：κ = ∞，ρ = 1.0000 —— δ 和 ε **完全不可分**。
       这不是「精度差」，是**信息矩阵奇异**，再多悬停数据也没用。
       ⇒ 这就是本章开头那个问题的答案（见 2.3）。

    ② 「20% 纯升降」和「纯悬停」一样：κ = ∞。
       四个电机一起加 5%，回归量方向 (1,4) **一点没变**。
       ⇒ ⚠️ 陷阱：**「动了」不等于「有信息」。**
          激励必须打在**待辨识参数的方向**上。升降只激励共模，
          而共模正是 δ 和 ε 共线的那条线 —— 白动。

    ③ 只有姿态机动（差动）才能把方向转开。duty 越大、Δ 越大，κ 越小。
""")

    sub("2.2  要多少激励才够？—— 差动幅度 × 占空比 扫描")
    s_ideal = sigma_delta_ideal(N)
    print("  参考下限（把回归量正交化、只有 δ 一个参数时的最好情况）：")
    print("      σ_ideal(δ̂) = σ_a / [ (c̄/m̄)·Th·√N ] = %.4f" % s_ideal)
    print("  下表最后一列 = σ(δ̂)/σ_ideal，即「共线惩罚倍数」。")
    rows = []
    for amp in (0.005, 0.02, 0.05, 0.10):
        for duty in (0.05, 0.20, 0.50):
            F = info_matrix(make_profile(N, duty, amp, seed=11))
            kappa, sd_d, sd_e, rho = ident_metrics(F)
            rows.append(["%.3f" % amp, "%.0f%%" % (100 * duty),
                         "%.2e" % kappa, "%.4f" % sd_d,
                         "%.0f×" % (sd_d / s_ideal)])
    table(["差动幅度 Δ", "占空比", "κ(F)", "σ(δ̂) [1σ]", "共线惩罚"], rows)

    def _pick(amp_s, duty_s):
        return float([r[3] for r in rows
                      if r[0] == amp_s and r[1] == duty_s][0])

    best = _pick("0.100", "50%")
    typ = _pick("0.050", "20%")
    F_real = info_matrix(make_profile(N, 0.20, 0.05, seed=11), sigma=5.1)
    _, s_real, _, _ = ident_metrics(F_real)

    print("""
  ★★ 这张表是本节最重要的产出，读法分三层：

    ① 共线惩罚倍数是 10¹~10² 量级 —— 这就是「δ 与 ε 共线」的代价。
       数据量再多也压不下去：σ ∝ 1/√N，而惩罚倍数与 N 无关。
       **这是结构问题，不是数据量问题。**

    ② 绝对量级：本机上最好的组合（Δ=0.10、50%% 机动）也只到
       σ(δ̂) ≈ %.3f —— 勉强能看见 15%% 的单电机故障。
       而现实中的飞行剖面大致是 (Δ≈0.05, duty≈20%%) 那一档，
       σ(δ̂) ≈ %.3f，**故障和估计噪声同量级，等于看不见**。

    ③ 换成真机上真正学到的噪声（实验 1 的 L2：accel_noise_var ≈ 26，
       即 σ_a ≈ 5.1 m/s²，多出来的是气动/桨洗/推力模型误差），
       (Δ=0.05, 20%%) 这一档的 σ(δ̂) 从 %.3f 变成 **%.2f** ——
       彻底没戏。

  ⇒ 本节标题的答案是：
     **「加激励」救不了垂直通道。** 姿态机动能把 κ 从 ∞ 压到 10³ 量级，
     但压不到「能分辨单电机故障」的水平。

  ⇒ 要真正解决，必须换**观测方程** —— 这就是实验 4 的主线：
     不去测「总推力够不够」，去测「力矩平不平」。

  ★ 一个可复用的工程判据：
     **估计误差必须显著小于你要检测的故障量。** 经验规则 σ(δ̂) ≲ Δδ/3。
     按这条反推需要的数据量：
""" % (best, typ, typ, s_real))

    # 需要多少采样才能把 σ(δ̂) 压到 Δδ/3
    F20 = info_matrix(make_profile(3000, 0.20, 0.05, seed=11))
    _, sd20, _, _ = ident_metrics(F20)
    n_need = 3000 * (sd20 / (0.15 / 3.0)) ** 2
    print("""
     要检测 15%% 的单电机故障需 σ(δ̂) ≤ 5%%：
         N ≈ 3000 × (%.3f/0.05)² = **%.0f 个采样**，按 10 Hz 算 **%.0f 秒 ≈ %.0f 分钟**。
     而这 %.0f 分钟还必须是**全程在滚转/俯仰**的飞行 ——
     真机日志里纯悬停的时间占比通常超过 80%%，所以实际还要再乘 5 倍。

     ⚠️ 陷阱：有人会想「那把 N 加大不就行了」。
        注意 σ ∝ 1/√N：从 %.3f 压到 0.05 需要 %.1f 倍数据量。
        而如果你只是**多悬停**，κ 仍然是 ∞，N 加多少倍都没用。
        **先解决方向问题，再解决数据量问题** —— 顺序反了就是白干。
""" % (sd20, n_need, n_need / 10.0, n_need / 600.0, n_need / 600.0,
       sd20, (sd20 / 0.05) ** 2))

    sub("2.3  ★ 本章核心问题的答案")
    print("""
      「悬停时，你能区分『1 号电机坏了』和『整机变重了』吗？为什么？」

    **不能。** 原因是可辨识性，不是传感器精度：

      两者在垂直通道上的回归量 φ = [u_1, U] 在同一条直线上，
      悬停时 u_1 = Th、U = 4Th，φ 永远指向 (1, 4) 这一个方向。
      Fisher 信息矩阵秩为 1 → 奇异 → 无穷多个 (δ, ε) 组合给出
      **完全相同的观测**。任何滤波算法都救不了，因为信息压根不在数据里。

      能辨识的只是那个**投影**： δ + 4ε （总推力缺口）。
      PX4 的 hover_thrust_estimate 报出来的就是这个投影 ——
      它是「共模意义下的效率」，**不是任何单个电机的效率**。

    ⇒ 于是有三条路：
        (a) 制造激励（机动）→ 实验 4 的改进方向之一
        (b) 换一个**不同的**观测方程（力矩配平）→ 实验 4 的主线
        (c) 承认不可辨识，只做**故障确认**而不做故障分离

    ★ 关于 σ = %.1f 的来历（诚实交代）：
      真实日志里 accel_noise_var 稳态 ≈ 26，即创新标准差 ≈ 5.1 m/s²，
      比加速度计本身的噪声（~0.3~0.5 m/s²）**大一个数量级**。
      多出来的部分是**模型误差**：阻力、桨洗、推力曲线非线性、
      机体振动。用它算 CRLB 会让 σ(δ̂) 再放大 5 倍。
      ⇒ 「辨识不了」在真机上比这个理想分析还要严重。
""" % SIG_A)


# ===========================================================================
# 3. 注入缺陷：三种故障，看原算法各自怎么表现
# ===========================================================================
#
#   闭环比开环诚实得多：姿态环会**自动配平**差模故障，
#   配平后的指令再喂给悬停推力估计器 —— 于是故障被「吃」掉了。
#
#   控制器结构（简化但忠实）：
#     高度环：PI  → F_des（总推力）
#     姿态环：PD + 积分 → M_des（力矩），目标滚转/俯仰角 = 0
#     混控：u = B_nom⁺ · [M_des, F_des]   ← B 用的是**标称** c，不知道故障
#
#   ★ 姿态环的积分器是关键：它把「1 号电机弱」翻译成「1 号多给点、
#     对角少给点」，动作完全在控制器内部完成，**对悬停推力估计器不可见**。
#
class QuadSim:
    """闭环四旋翼仿真：高度环 + 姿态环 + 标称混控 + 真实（可能故障的）机体。"""

    def __init__(self, c_true=None, m_true=M_NOM, dt=0.01):
        self.c = np.full(4, C_NOM) if c_true is None else np.asarray(c_true, float)
        self.kap = np.ones(4)
        self.eta = np.zeros(4)   # 电机/桨的固有散布（_simulate 里注入）
        self.m = m_true
        # ★ 混控器**以为的**力臂系数（1.0 = 标定准确）。真机力臂恒为 L_ARM。
        self.l_mix = 1.0
        # ★ 外部力矩扰动 [roll, pitch, yaw]，N·m（风、载荷偏心、线缆拖拽）。
        #   仿真没有偏航自由度，所以只有 roll/pitch 会进动力学 ——
        #   但配平量会如实反映它，这正是 5.3 要展示的。
        self.m_dist = np.zeros(3)
        self.dt = dt
        self.z = -5.0            # NED：-5 表示离地 5 m
        self.vz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.p = 0.0             # 滚转角速率
        self.q = 0.0
        self.i_roll = 0.0        # 姿态环积分（配平量）
        self.i_pitch = 0.0
        self.i_z = 0.0           # 高度环积分
        self.z_sp = -5.0
        self.J = 0.008           # 转动惯量（5 寸，kg·m²）
        self.kt = 0.35           # 力矩→角加速度的等效增益系数
        self.log = {k: [] for k in
                    ("t", "z", "vz", "roll", "pitch", "p", "q",
                     "u1", "u2", "u3", "u4",
                     "trim_roll", "trim_pitch", "az", "F_up")}
        self.t = 0.0

    def step(self, az_noise=0.0, rng=None, u_sat=True):
        dt = self.dt
        # ---------- 高度环 ----------
        ez = self.z - self.z_sp
        F_des = self.m * (G + 2.2 * (-ez) - 3.4 * self.vz + self.i_z)
        self.i_z += -0.6 * ez * dt
        F_des = min(max(F_des, 0.0), 4.0 * C_NOM)

        # ---------- 姿态环（目标：滚转=俯仰=0）----------
        # 增益按「5 寸机的姿态环带宽 ~ 10 Hz」取：Kp 要足够硬，
        # 否则故障注入后残留的滚转角会到十几度，那不是 PX4 的真实行为。
        M_roll_des = self.J * (-90.0 * self.roll - 10.0 * self.p) + self.i_roll
        M_pitch_des = self.J * (-90.0 * self.pitch - 10.0 * self.q) + self.i_pitch
        self.i_roll += -self.J * 40.0 * self.roll * dt
        self.i_pitch += -self.J * 40.0 * self.pitch * dt

        # ---------- 标称混控（PX4 的 B⁺）----------
        # ★ 用的是**混控器以为的**力臂；真机力臂在 wrench() 里恒为 L_ARM
        u = mix([M_roll_des, M_pitch_des, 0.0, F_des], self.l_mix * L_ARM)
        if u_sat:
            u = np.clip(u, 0.0, 1.0)

        # ---------- 真实机体（用真实 c / kap / eta）----------
        m_roll, m_pitch, _, F_up = wrench(u, self.c * (1.0 + self.eta), self.kap)
        m_roll += self.m_dist[0]                     # 风 / 载荷偏心
        m_pitch += self.m_dist[1]
        az = F_up / self.m - G                       # 机体 z 轴（近似水平）
        self.vz += az * dt
        self.z += self.vz * dt
        dp = (m_roll - self.kt * 0.02 * self.p) / self.J
        dq = (m_pitch - self.kt * 0.02 * self.q) / self.J
        self.p += dp * dt
        self.q += dq * dt
        self.roll += self.p * dt
        self.pitch += self.q * dt

        # ---------- 记录 ----------
        c = self.log
        self.t += dt
        c["t"].append(self.t)
        c["z"].append(self.z)
        c["vz"].append(self.vz)
        c["roll"].append(self.roll)
        c["pitch"].append(self.pitch)
        c["p"].append(self.p); c["q"].append(self.q)
        c["u1"].append(u[0]); c["u2"].append(u[1])
        c["u3"].append(u[2]); c["u4"].append(u[3])
        c["trim_roll"].append(u[0] + u[1] - u[2] - u[3])
        c["trim_pitch"].append(u[0] - u[1] - u[2] + u[3])
        # 估计器看到的量测：真实加速度 + 噪声
        az_meas = az + (az_noise * rng.standard_normal() if rng is not None else 0.0)
        c["az"].append(az_meas)
        c["F_up"].append(F_up)
        return u

    def run(self, t_end, az_noise=0.0, seed=0, fault_at=None, fault=None):
        rng = np.random.default_rng(seed)
        n = int(t_end / self.dt)
        for k in range(n):
            if fault_at is not None and abs(self.t - fault_at) < self.dt / 2:
                fault(self)
            self.step(az_noise=az_noise, rng=rng)

    def feed_ekf(self, dt_ekf=0.1, **kw):
        """把仿真日志降采样到 dt_ekf，喂给 PX4 悬停推力估计器。

        喂进去的是 (a_z, T)，T = 总推力指令的归一化值（PX4 里是
        local_pos_sp.thrust[2] 的相反数）。PX4 的 T 是「四个电机指令的
        平均」，所以 T = mean(u)。
        """
        ekf = ZeroOrderHoverThrustEkf(dt=dt_ekf, **kw)
        log = self.log
        step = int(round(dt_ekf / self.dt))
        n = len(log["t"])
        ts, ths, Rs, trims = [], [], [], []
        for k in range(0, n, step):
            u_mean = 0.25 * (log["u1"][k] + log["u2"][k] + log["u3"][k] + log["u4"][k])
            ekf.predict(dt_ekf)
            ekf.fuse_acc_z(log["az"][k], u_mean)
            ts.append(log["t"][k]); ths.append(ekf.th)
            Rs.append(ekf.R)
            trims.append(log["trim_roll"][k])
        return (np.array(ts), np.array(ths), np.array(Rs), np.array(trims), ekf)


def _fault_eff(delta, motor=0):
    def f(sim):
        sim.c[motor] = C_NOM * (1.0 - delta)
    return f


def _common_scaled(delta):
    """共模故障：四个电机一起掉 delta（老化 / 电池模型整体偏 / 进气堵塞）。"""
    def f(sim):
        sim.c = sim.c * (1.0 - delta)
    return f


def _fault_mass(eps):
    def f(sim):
        sim.m = M_NOM * (1.0 + eps)
    return f


def _fault_prop(delta):
    """桨叶损伤：推力掉 delta，同时**阻扭比上升**（桨面变粗糙）。

    ★ 这是 §9.3 说的「非对角耦合」：B 矩阵里推力那一行和偏航那一行
      同时变，而且变化比例不同 —— 用单一的效率标量根本描述不了。
    """
    def f(sim):
        sim.c[0] = C_NOM * (1.0 - delta)
        sim.kap[0] = 1.0 + 1.2 * delta      # 阻力矩相对增大
    return f


# ★ 用多大的 a_z 噪声？直接抄真机日志：
#     log_13 的 accel_noise_var 稳态均值 = 26.0 → σ = 5.10 m/s²，
#     而且 accel_innov 的实测标准差是 5.16，两者一致。
#   这不是加速度计的噪声（规格书上是 0.3~0.5），而是**模型误差**：
#     阻力、桨洗、推力曲线非线性、机体振动、姿态估计误差…
#   用 5.1 而不是 0.35 会让下面的结论难看很多，但那是真的。
AZN = 5.1

# 电机/桨的固有散布 1σ（出厂就这么散）。它同时是「误报」的物理下限：
# 想让估计器报出「哪个电机坏了」，检测门限必须显著高于这个基线。
SPREAD = 0.01


def _simulate(fault=None, t_end=70.0, t_fault=5.0, az_noise=AZN, seed=1):
    """跑一遍闭环，返回 (仿真对象, 时间轴, Th 轨迹, R 轨迹, 配平轨迹, ekf)。"""
    sim = QuadSim()
    sim.eta = np.random.default_rng(seed + 77).normal(0.0, SPREAD, 4)
    if fault is None:
        sim.run(t_end, az_noise=az_noise, seed=seed)
    else:
        sim.run(t_end, az_noise=az_noise, seed=seed,
                fault_at=t_fault, fault=fault)
    t, th, R, tr, ekf = sim.feed_ekf()
    return sim, t, th, R, tr, ekf


T_STEADY = 55.0        # 认为 >55 s 之后进入稳态

# 消融/分场景实验的随机种子。取 6 个而不是 3 个，是因为「A2/A3/A4 的贡献
# 到底是不是噪声」正是本章要回答的问题 —— 样本太少就没有资格下结论。
SEEDS = (1, 2, 3, 4, 5, 6)


def exp3():
    hr("实验 3  注入缺陷：电机效率下降 / 整机变重 / 桨叶损伤")

    print("""
  三种故障，都在 t = %.0f s 注入，飞 %.0f s，取最后 10 s 作稳态：
      F1  1 号电机效率 −15%%     （差模故障）
      F2  整机变重 +10%%         （共模「故障」）
      F3  1 号桨叶损伤 −25%%     （差模 + 阻扭比上升，即 §9.3 的「非对角耦合」）

  指标：
      ΔTh       悬停推力估计相对无故障基线的变化
      配平量    滚转差动量 u1+u2−u3−u4 的平均值（姿态环积分器在电机指令上的外部表现）
      残余滚转  注入后残余的滚转角（姿态环没配平干净的部分）
      门限拒绝率 新息检验被拒绝的比例 —— 它「感觉」到异常了吗

  ⚠️ 关键前提：悬停推力估计器**看不见配平量**。它的输入只有
     local_pos.az 和 local_pos_sp.thrust[2] 两个标量
     （MulticopterHoverThrustEstimator.cpp:182）。
     下面的表就是在这个前提下读的。
""")

    base_sim, t0, th0, R0, tr0, ekf0 = _simulate(None)
    m0 = t0 > T_STEADY
    Th_base = th0[m0].mean()
    Th_sd = th0[m0].std()
    trim_base = tr0[m0].mean()
    rej_base = ekf0.rejected / max(1, len(t0))
    print("  基线：稳态 Th = %.5f ± %.5f（1σ，与真机日志一致：az 有效噪声 5.1 m/s²）"
          % (Th_base, Th_sd))

    rows = []
    detail = {}
    for tag, fault in [("F1 电机效率 −15%", _fault_eff(0.15)),
                       ("F2 整机变重 +10%", _fault_mass(0.10)),
                       ("F3 桨叶损伤 −25%", _fault_prop(0.25))]:
        sim, t, th, R, tr, ekf = _simulate(fault)
        m = t > T_STEADY
        dTh = th[m].mean() - Th_base
        trim = tr[m].mean() - trim_base
        rej = ekf.rejected / max(1, len(t))
        roll = math.degrees(np.abs(np.array(sim.log["roll"])[-1000:]).max())
        detail[tag] = dict(dTh=dTh, trim=tr[m].mean(), roll=roll,
                           Th=th[m].mean(), rej=rej)
        rows.append([tag, "%.5f" % th[m].mean(), "%+.5f" % dTh,
                     "%+.2f %%" % (100 * dTh / Th_base),
                     "%+.4f" % tr[m].mean(), "%.2f°" % roll,
                     "%.1f%%" % (100 * rej)])
    rows.insert(0, ["基线（无故障）", "%.5f" % Th_base, "—", "—",
                    "%+.4f" % trim_base, "0.00°", "%.1f%%" % (100 * rej_base)])
    table(["场景", "稳态 Th", "ΔTh", "相对变化", "滚转配平量", "残余滚转", "门限拒绝率"],
          rows)

    d1 = detail["F1 电机效率 −15%"]
    d2 = detail["F2 整机变重 +10%"]
    d3 = detail["F3 桨叶损伤 −25%"]
    print("""
  ★★ 三条读法：

    ① **F1 与 F2 在 Th 上被压成同一个数字。** 这正是实验 2 那个奇异方向
       在闭环真机上的样子：−15%% 的电机效率损失给出 %+.2f%%，
       +10%% 的变重给出 %+.2f%%，**两者差 %.2f 个百分点，都在噪声量级**。
       估计器没有任何办法告诉你是哪一个。

    ② 但**配平量把它们彻底分开了**：
         F1 滚转配平 %+.4f   ← 差模故障：力矩通道一阶可见
         F2 滚转配平 %+.4f   ← 共模「故障」：力矩通道严格看不见
       ⇒ 故障的信息**不在垂直通道里，在力矩通道里**。
       ⇒ 而 PX4 的估计器根本不看力矩 —— 这是「现有方法为什么不行」的
         **结构性**理由，不是精度问题、不是调参问题。

    ③ 门限拒绝率几乎不涨（%.1f%% vs 基线 %.1f%%）。NIS 门限（cpp:112-120）
       是为**突然的**悬停推力跳变设计的（见单元测试 testHoverThrustJump），
       对**缓慢的**效率漂移基本无感 —— 这符合它 2 s 的噪声学习时间常数
       （hpp:137 `_noise_learning_time_constant = 2.f`）。
""" % (100 * d1["dTh"] / Th_base, 100 * d2["dTh"] / Th_base,
       abs(100 * d1["dTh"] / Th_base - 100 * d2["dTh"] / Th_base),
       d1["trim"], d2["trim"], 100 * d3["rej"], 100 * rej_base))

    # ---- 3.1 退化方向：把实验 2 的零空间在闭环里验一遍 ----
    sub("3.1  ★ 退化方向：ΔTh/Th = δ/4 + ε —— 实验 2 那个零空间的闭环验证")
    print("""
  先算一笔小账。四电机等推力悬停时，1 号电机效率掉 δ，
  它的推力占总量 1/4，所以总推力缺口 = δ/4；
  质量增加 ε 则要求总推力增加 ε。于是

        ΔTh/Th = δ/4 + ε        ← **一个方程，两个未知数**

  这正是实验 2 里「δ 与 ε 共线」的解析形式：可辨识的只有组合量 δ/4 + ε。
  下面用闭环仿真把这条式子验一遍（网格扫 δ 和 ε）：
""")
    _cache = {}

    def run_grid(d, eps):
        key = (round(d, 4), round(eps, 4))
        if key in _cache:
            return _cache[key]
        if d == 0.0 and eps == 0.0:
            r = _simulate(None)
        elif d == 0.0:
            r = _simulate(_fault_mass(eps))
        elif eps == 0.0:
            r = _simulate(_fault_eff(d))
        else:
            def combo(s, _d=d, _e=eps):
                s.c[0] = C_NOM * (1.0 - _d)
                s.m = M_NOM * (1.0 + _e)
            r = _simulate(combo)
        sim, t, th, R, tr, ekf = r
        m = t > T_STEADY
        out = ((th[m].mean() - Th_base) / Th_base, tr[m].mean())
        _cache[key] = out
        return out

    grid = []
    for d in (0.0, 0.10, 0.20, 0.40, 0.60):
        for eps in (0.0, 0.025, 0.05, 0.10):
            dTh, trim = run_grid(d, eps)
            pred = d / 4.0 + eps
            grid.append(["%.2f" % d, "%+.1f%%" % (100 * eps),
                         "%+.2f%%" % (100 * pred),
                         "%+.2f%%" % (100 * dTh),
                         "%+.2f%%" % (100 * (dTh - pred)),
                         "%+.4f" % trim])
    table(["δ（1号效率损失）", "ε（变重）", "理论 δ/4+ε", "闭环实测 ΔTh/Th",
           "偏差", "滚转配平量"], grid)

    def gv(g, i):
        return float(str(g[i]).replace("%", "").replace("+", ""))

    small = [g for g in grid if float(g[0]) <= 0.20]
    maxdev_small = max(abs(gv(g, 4)) for g in small)
    maxdev_all = max(abs(gv(g, 4)) for g in grid)
    print("""
  ★★★ 这张表要先说清楚一件事，否则很容易被误读：

    ① 线性预测式 ΔTh/Th = δ/4 + ε 在 δ ≤ 0.20 的 %d 个组合上最大偏差
       %.2f 个百分点，δ 大到 0.60 时偏差涨到 %.1f 个百分点 ——
       **闭环非线性把这条直线掰弯了**。原因是姿态配平把平均油门抬了起来，
       而抬多少取决于 δ 本身。

       ⚠️ 陷阱：看到这里很容易得出错误结论 ——「既然理论值对不上实测，
          那说明 δ 和 ε 是可分的嘛！」**不是。** 下面用实验说话。
""" % (len(small), maxdev_small, maxdev_all))

    # ---- 等值线演示：同一条 ΔTh 上挂着无穷多组 (δ, ε) ----
    sub("3.1b  ★★ 等 ΔTh 曲线：同一个 Th，对应无穷多组 (δ, ε)")
    ref_d, ref_e = 0.20, 0.0
    ref_dTh, ref_trim = run_grid(ref_d, ref_e)
    print("""
  做法：拿 (δ=%.2f, ε=0) 当靶子，它的 ΔTh/Th = %+.2f%%（滚转配平 %+.4f）。
  然后固定不同的 δ，用割线法反解出「要让 ΔTh 落在同一个值上，ε 得是多少」。
  如果每个 δ 都能解出一个 ε，就说明这个标量观测**根本区分不了它们**。
""" % (ref_d, 100 * ref_dTh, ref_trim))

    rows = []
    for d in (0.0, 0.10, 0.20, 0.30, 0.40, 0.60):
        eps = ref_dTh - d / 4.0          # 线性初值
        for _ in range(4):               # 割线迭代（斜率≈1，收敛很快）
            dTh_here, _ = run_grid(d, eps)
            err = dTh_here - ref_dTh
            if abs(err) < 2e-4:
                break
            eps -= err
        dTh_here, trim_here = run_grid(d, eps)
        rows.append(["%.2f" % d, "%+.2f%%" % (100 * eps), "%+.2f%%" % (100 * dTh_here),
                     "%+.3f pp" % (100 * (dTh_here - ref_dTh)),
                     "%+.4f" % trim_here,
                     "—" if abs(trim_here) < 1e-3 else "可分辨"])
    table(["δ", "反解出的 ε", "实测 ΔTh/Th", "与靶子的差", "滚转配平量",
           "力矩通道"], rows)

    def pct(s):
        return float(str(s).replace("%", "").replace("+", ""))

    print("""
  ★★★ **这就是本章可辨识性主线的闭环铁证：**

    ① 六组完全不同的 (δ, ε) —— 从「纯变重 %+.2f%%」到「1 号电机掉了 60%%」——
       给出了**同一个悬停推力估计值**（彼此相差 < 0.03 个百分点，
       远小于噪声的 %.2f 个百分点）。
       ⇒ **PX4 的估计器报出同一个数字，但飞机的实际状态天差地别。**

    ② 看第 1 行和第 2 行：
         1 号电机效率 −20%%，没有任何变重  →  ΔTh/Th = %+.2f%%
         整机变重 %+.2f%%，电机完好        →  ΔTh/Th = %+.2f%%
       在悬停推力估计器眼里，**这两件事是同一件事**。
       ⇒ 这就是本章开头那个问题的答案：**不能区分。**
         PX4 报出的 Th 是「总推力缺口」，不是「哪个电机坏了」。

    ③ 再看后几行：δ 越大，需要「飞机变轻」越多（ε 变成负数）才能模仿。
       δ = 0.60 时要 ε = %s，那已经是不物理的 ——
       所以歧义带是有边界的：现实中 δ 在 0%%~30%%、ε 在 0%%~10%% 之间时，
       歧义带完全覆盖这个区间。**故障诊断恰恰工作在这个区间里。**

    ④ 出路在最后两列：滚转配平量从 %+.4f 一路变到 %+.4f。
       **同一个 ΔTh，配平量差 %d 倍 ⇒ 两条通道的信息合起来才能把这组解拆开。**
       这就是实验 4 的方案基础。

    ⑤ 但请死死记住第 1 行：**δ=0 那一行的配平量严格为 0.0000。**
       纯变重（以及四个电机一起老化）在力矩通道里是**完全不可见**的 ——
       所以下面的改进方案也救不了这一对。这一条会在实验 5 里
       作为第一个失效点正式写出来。
""" % (pct(rows[0][1]), 100 * Th_sd / Th_base,
       pct(rows[2][2]), pct(rows[0][1]), pct(rows[0][2]),
       rows[5][1], float(rows[0][4]), float(rows[5][4]),
       int(round(abs(float(rows[5][4])) / max(abs(float(rows[3][4])), 1e-9)))))

    # ---- 3.2 检测门限 ----
    sub("3.2  失效边界：多小的故障会被悬停推力估计器彻底漏掉")
    rows = []
    last = {}
    for d in (0.0, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40):
        if d == 0.0:
            sim, t, th, R, tr, ekf = _simulate(None)
        else:
            sim, t, th, R, tr, ekf = _simulate(_fault_eff(d))
        m = t > T_STEADY
        dTh = th[m].mean() - Th_base
        snr = dTh / max(Th_sd, 1e-9)
        umax = max(np.array(sim.log[k])[-1000:].max()
                   for k in ("u1", "u2", "u3", "u4"))
        roll = math.degrees(np.abs(np.array(sim.log["roll"])[-1000:]).max())
        last[d] = dict(snr=snr, umax=umax, roll=roll, dTh=dTh)
        rows.append(["%.0f%%" % (100 * d), "%+.5f" % dTh,
                     "%+.2f%%" % (100 * dTh / Th_base),
                     "%.1f σ" % snr,
                     "能" if snr > 3 else "不能",
                     "%.3f" % umax, "%.1f°" % roll,
                     "是" if umax > 0.95 else "否"])
    table(["δ", "ΔTh", "ΔTh/Th", "信噪比", "3σ 可检测？", "最大指令",
           "残余滚转", "饱和"], rows)
    d30 = last[0.30]
    print("""
  ★ 三条边界，都要记住：

    · **检测边界**：Th 的 1σ 噪声是 %.5f（本仿真，az 噪声 5.1 m/s² —— 真机日志实测值）。
      要 3σ 可信需要 ΔTh > %.5f，即 δ > %.0f%%。
      ⇒ 比这更小的故障，估计器**完全看不见**，Th 还是老样子。
      ⚠️ 这个边界已经是真机量级，**不要再乘任何倍数**：本仿真用的 5.1 m/s²
         就取自真机日志（log_13 的 accel_noise_var ≈ 26 开方，见本文件 :1005-1010），
         与 hover_thrust 的实测波动量级一致，不是纸上推算。

    · **饱和边界**：本机悬停油门只有 %.2f（T/W ≈ %.1f），
      所以直到 δ = 40%% 最大指令才 %.2f，**根本不会饱和**。
      ⇒ 结论必须改口：**饱和不是这台机器的风险。**

    · **真正的风险是「被静默吸收」**：δ = 30%% 时飞机残余滚转只有 %.1f°、
      最大指令 %.2f，看上去一切正常；悬停推力估计器只是把 Th 抬高 %.1f%%，
      而这个抬高**恰好和「变重 %.1f%%」不可区分**。
      飞手从遥测上完全看不出异常，直到某天要做一次大机动才发现余度不够。
      ★ 这就是第 9 章开头说的「现有飞控的不足」在**具体数值上**的样子。
""" % (Th_sd, 3 * Th_sd, 100 * 4 * 3 * Th_sd / Th_base,
       TH_HOVER, 4 * C_NOM / (M_NOM * G), last[0.40]["umax"],
       d30["roll"], d30["umax"], 100 * d30["dTh"] / Th_base,
       100 * d30["dTh"] / Th_base))



# ===========================================================================
# 4. 改进方案 + 消融
# ===========================================================================
#
#   改进的核心一句话：
#
#     把「力矩配平」这组方程加进估计器 —— 故障的差模部分在垂直通道里
#     不可见，但在力矩通道里是**一阶可见**的。
#
#   观测量（都在 PX4 里现成可用）：
#     (1) 垂直：a_z        ← 现有估计器已经在用
#     (2) 滚转：姿态环为了让 M_roll = 0 所施加的配平力矩
#     (3) 俯仰：同上
#     (4) 偏航：同上（偏航通道对「1 号电机弱」也敏感，因为 σ1 = -1）
#
#   展开到一阶（记 δ_i 为各电机效率偏差、ε 为质量偏差）：
#
#     a_z + g = (c̄/m̄)·[ U - Σδ_i u_i - εU ]
#     M_roll  = -l·c̄·[ (u1+u2-u3-u4) - (δ1u1+δ2u2-δ3u3-δ4u4) ]
#     M_pitch =  l·c̄·[ (u1-u2-u3+u4) - (δ1u1-δ2u2-δ3u3+δ4u4) ]
#     M_yaw   =  κ·c̄·[ (σᵀu) - (σᵀ(δ∘u)) ]
#
#   ★ 姿态环把 M 控制到 0，所以「实际力矩 = 指令力矩」，而指令力矩是已知的。
#     于是 4 个方程、5 个未知量 (δ1..δ4, ε)。
#
#   ⇒ **零空间维数 1，方向 (1,1,1,1,-1)：共模效率下降 ↔ 变重，永远不可分。**
#     差模则被完全解开。这与实验 2 的结论严格一致，只是换了一组观测。
#

def build_regressor(u, U=None):
    """返回 4 行观测方程 A·θ ≈ y（θ = [δ1, δ2, δ3, δ4, ε]）。

    ★ 全部来自**飞控自己已有的量**，不需要任何新传感器：

      行1 垂直  (a_z + g)·m̄/c̄ − U = −Σδ_i u_i − ε·U
      行2 滚转  (u1+u2−u3−u4)     =  δ1u1 + δ2u2 − δ3u3 − δ4u4
      行3 俯仰  (u1−u2−u3+u4)     =  δ1u1 − δ2u2 − δ3u3 + δ4u4
      行4 偏航  (σᵀu)             =  σᵀ(δ∘u)          σ=[−1,+1,−1,+1]

    行 2~4 的来历：**姿态环把力矩配平到 0**，于是
        M_roll = −l·c̄[(u1+u2−u3−u4) − (δ1u1+δ2u2−δ3u3−δ4u4)] = 0
    把标称部分挪到左边就是上面的式子 —— 左边全部是**电机指令**，
    右边是待估参数。所以这三行观测的是「配平量」，不是力矩本身。

    ⚠️ 前提：这三行只在**准稳态配平**时成立。机动中 M_actual ≠ 0，
       方程不成立 —— 这正是 A2 那个「配平有效性门控」要处理的事。
    """
    u = np.asarray(u, float)
    if U is None:
        U = u.sum()
    g1 = C_NOM / M_NOM
    return np.array([
        [-u[0], -u[1], -u[2], -u[3], -U],
        [u[0], u[1], -u[2], -u[3], 0.0],
        [u[0], -u[1], -u[2], u[3], 0.0],
        [SIGMA[0] * u[0], SIGMA[1] * u[1], SIGMA[2] * u[2], SIGMA[3] * u[3], 0.0],
    ])


def nullspace_report(u=None):
    """零空间分析：哪些方向永远不可辨识。"""
    if u is None:
        u = np.full(4, TH_HOVER)
    A = build_regressor(u)
    _, s, Vt = np.linalg.svd(A)
    rank = int((s > 1e-9 * s[0]).sum())
    return A, s, rank, Vt[rank:]


SIG_MOM_FLOOR = 0.01


def _row_sigma(sim, t):
    """从一次仿真的日志里量出「每条方程的噪声标准差」—— 不靠拍脑袋给权重。

    · 行1：由 a_z 噪声折算，(m̄/c̄)·σ_a
    · 行2~4：由**无故障**时配平量的抖动直接测出来（含几何/量化/未建模误差）
    """
    n = len(t)
    u = np.array([[sim.log["u1"][k], sim.log["u2"][k],
                   sim.log["u3"][k], sim.log["u4"][k]] for k in range(n)])
    m = t > T_STEADY
    uu = u[m]
    r2 = uu[:, 0] + uu[:, 1] - uu[:, 2] - uu[:, 3]
    r3 = uu[:, 0] - uu[:, 1] - uu[:, 2] + uu[:, 3]
    r4 = SIGMA @ uu.T
    # ★ 力矩三行加一个**工程下限** SIG_MOM_FLOOR：
    #   本仿真是理想的，准稳态下配平量几乎不抖（σ→0），
    #   于是 1/σ 会爆掉，RLS 会把三条方程当成「绝对真理」。
    #   真机上这三行的误差来自：电机/桨固有散布(SPREAD)、
    #   姿态环稳态误差、指令量化、未建模动态 —— 量级约 1%。
    #   取这个下限对结论是**不利**的（权重变小、估计变差），
    #   符合本章「宁可把话说难听」的立场。
    return np.array([AZN * M_NOM / C_NOM,
                     max(r2.std(), SIG_MOM_FLOOR),
                     max(r3.std(), SIG_MOM_FLOOR),
                     max(r4.std(), SIG_MOM_FLOOR)])


class EfficiencyIdentifier:
    """改进方案的核心：用 RLS 估计**可辨识子空间**里的 4 个参数。

    ★ 为什么不直接估 5 个 (δ1..δ4, ε)？
      因为实验 4.1 已经证明零空间方向 (1,1,1,1,−1) 不可辨识。
      把一个不可辨识的方向留在状态里，滤波器会沿着它**自由漂移** ——
      数值上表现为 δ̂ 越跑越大（这一条本脚本第一版真的踩过：
      δ̂1 跑到 −3.8e5）。这不是「滤波器发散」，是**参数化本身就是错的**。

      正确的做法是**重参数化到可辨识子空间**（4 维）：

          差模 3 维： d_i = δ_i − mean(δ)      （i = 1,2,3，第 4 个由 Σd=0 定出）
          共模 1 维： ĉ  = mean(δ) + ε

      ★ ĉ 的物理含义：**整机的推力缺口**。它同时含「四个电机一起变弱」
        和「整机变重」两种可能，**在数学上永远分不开**（这就是零空间）。
        所以 ĉ 不是「质量变化」，也不是「共模效率」——
        它是那个**能观测到的投影**，正如 PX4 现在报的 hover_thrust。

      ⇒ 本方案相对 PX4 的真正增量只有一件事：
        **把差模 d（哪个电机）从共模 ĉ（整体差多少）里拆出来。**
          共模那一维，PX4 早就在报了，本方案并没有多知道任何东西。

    variant 逐级加功能（消融）：
      0  只有垂直通道、单标量 ĉ（≡ PX4 现状）
      1  + 力矩配平方程，参数重写成 (d1,d2,d3,ĉ)
      2  + 准稳态门控：只在「配平成立」的采样上更新力矩行
      3  + 观测加权：按每条方程的实测噪声折算
      4  + 有效采样约束：电机饱和 / 角速率过大时整帧丢弃
    """

    def __init__(self, variant, sigma_row, lam=0.999):
        self.v = variant
        self.sig = np.asarray(sigma_row, float)
        n_par = 1 if variant == 0 else 4
        self.th = np.zeros(n_par)
        self.P = np.eye(n_par) * 1.0
        self.lam = lam
        self.prev_u = None
        self.prev_w = 0.0
        self.n_upd = 0
        self.n_gated = 0
        self.n_drop = 0

    def _quasi_steady(self, u):
        if self.prev_u is None:
            return False
        return np.abs(np.asarray(u) - self.prev_u).max() < 2e-4

    def _valid(self, u, w):
        """A4：饱和或角速率过大 → 整帧丢弃。"""
        if np.asarray(u).max() > 0.98 or np.asarray(u).min() < 0.02:
            return False
        if abs(w - self.prev_w) > 0.5:          # rad/s 的帧间跳变
            return False
        return True

    def update(self, u, az, w=0.0, g1=C_NOM / M_NOM):
        u = np.asarray(u, float)
        A = build_regressor(u)
        y = np.array([
            (az + G) / g1 - u.sum(),
            u[0] + u[1] - u[2] - u[3],
            u[0] - u[1] - u[2] + u[3],
            float(SIGMA @ u),
        ])

        if self.v >= 4 and not self._valid(u, w):
            self.n_drop += 1
            self.prev_u = u.copy()
            self.prev_w = w
            return

        if self.v == 0:
            # 单标量版：只有垂直方程，只有 ĉ 这一列。
            # 这正是 PX4 现在做的事（换成 hover_thrust_estimate 也一样）。
            a = np.array([-float(u.sum())])
            rows = [(a, y[0], 4.0 * self.sig[0])]
        else:
            Ar = reduce_to_identifiable(A, u)
            rows = [(Ar[0], y[0], self.sig[0])]
            if self.v >= 2:
                ok = self._quasi_steady(u)
            else:
                ok = True
            if ok:
                for i in (1, 2, 3):
                    rows.append((Ar[i], y[i], self.sig[i]))
            else:
                self.n_gated += 1

        for a, yy, sg in rows:
            wgt = 1.0 / sg if self.v >= 3 else 1.0
            self._rls(a * wgt, yy * wgt)

        self.prev_u = u.copy()
        self.prev_w = w
        self.n_upd += 1

    def _rls(self, a, y):
        Pa = self.P @ a
        den = self.lam + float(a @ Pa)
        K = Pa / den
        self.th = self.th + K * (y - float(a @ self.th))
        self.th = np.clip(self.th, -0.6, 0.6)     # 工程上必须有，防止跑飞
        self.P = (self.P - np.outer(K, Pa)) / self.lam
        self.P = 0.5 * (self.P + self.P.T)

    def result(self):
        """返回 (差模 d1..d4, 共模 ĉ)。d 是零均值的。"""
        if self.v == 0:
            return np.zeros(4), float(self.th[0])
        d = self.th[:3]
        return np.array([d[0], d[1], d[2], -(d[0] + d[1] + d[2])]), float(self.th[3])


def reduce_to_identifiable(A, u):
    """把 5 列 (δ1..δ4, ε) 的观测矩阵压成**可辨识的 4 列** (d1,d2,d3,ĉ)。

    推导（一行就够）：

      δ_i = d_i + m ，其中 m = mean(δ)，Σd_i = 0
      ĉ   = m + ε

      垂直行： −Σδ_i u_i − εU = −Σd_i u_i − mU − εU = −Σd_i u_i − ĉU
      力矩行： Σcoef·δ    = Σcoef·d + m·Σcoef = Σcoef·d + m·y

    ⇒ 前三列 = A[:,i] − A[:,3]（i=1,2,3，消掉共模）
      第四列 = [−U, 0, 0, 0]ᵀ（只有垂直行看得见 ĉ）

    ⚠️ 力矩行里那个 m·y 项是**近似丢掉的**：它等于「共模量 × 配平量」。
       悬停时 |y|（配平量）在无故障时 ≲ 0.01，但 15% 的单电机故障会让
       它涨到 0.2~0.4；|m|（共模量）在实验 4 的场景里 ≲ 0.04。
       于是被丢掉的项 ≲ 0.02，**与力矩行的噪声下限 SIG_MOM_FLOOR = 0.01
       同量级或略大。**
       **这是本方案的一个明确近似，不是「推导里没有」。**
       ⚠️ 但实验 5.3(b) 的失配诊断显示：真正主导 d̂ 收缩的**不是**这一项，
          而是偏航方程的系统性失配（那里逐行算了 0.0785 vs 0.006 / 0.008）。
          也就是说：**这是一项「理论上存在、实测里被更大的东西盖住」的近似** ——
          做误差预算时别把它排在第一位。

    ★ 为什么不做精确版？精确版要把力矩行的右端除以 (1−m)，
      而 m 本身是待估参数 → 观测方程变成非线性。
      工程上正确的做法是把这一步放进 EKF/UKF 的量测方程里，
      而不是像本章这样用「线性化 + 最小二乘」。
      **本章选简单版本，是为了让每一个数字都能手算复核；
      不是因为简单版本更好。** 这一点也必须写进 Limitations。
    """
    col_c = np.array([-float(np.sum(u)), 0.0, 0.0, 0.0])
    return np.column_stack([A[:, 0] - A[:, 3], A[:, 1] - A[:, 3],
                            A[:, 2] - A[:, 3], col_c])


def true_reduced(d_true, e_true):
    """真值投到可辨识坐标： (d1..d4 零均值, ĉ)。

    ★ 必须和 `EfficiencyIdentifier.result()` 的输出**逐位同构**：
      两者都返回长度 4 的零均值差模向量 + 一个共模标量。
      本脚本第一版这里返回的是长度 3 的向量（只给 d1,d2,d3），
      于是和 result() 的 4 维输出做减法时形状不匹配 ——
      这类「坐标不一致」的 bug 在辨识代码里极常见，
      而且往往不会报错，只会让结果悄悄错掉。**宁可让它当场崩。**
    """
    m = float(np.mean(d_true))
    d = np.asarray(d_true, float) - m
    return d, m + e_true


def _collect(sim, t):
    n = len(t)
    u = np.array([[sim.log["u1"][k], sim.log["u2"][k],
                   sim.log["u3"][k], sim.log["u4"][k]] for k in range(n)])
    az = np.array(sim.log["az"])
    p = np.array(sim.log["p"]); q = np.array(sim.log["q"])
    w = np.hypot(p, q)
    return u, az, w



def exp4():
    hr("实验 4  改进方案：把力矩配平方程加进辨识器（含消融）")

    # ---------------- 4.1 结构 ----------------
    sub("4.1  先看结构：为什么「力矩」能救，而「更多悬停数据」不能")
    A, s, rank, null = nullspace_report()
    print("""
  在纯悬停（u_i = Th = %.4f）处，把 4 个观测方程写成 A·θ = y，
  θ = [δ1, δ2, δ3, δ4, ε]：""" % TH_HOVER)
    table(["", "δ1", "δ2", "δ3", "δ4", "ε"],
          [["垂直"] + ["%+.3f" % v for v in A[0]],
           ["滚转"] + ["%+.3f" % v for v in A[1]],
           ["俯仰"] + ["%+.3f" % v for v in A[2]],
           ["偏航"] + ["%+.4f" % v for v in A[3]]])
    # ★ 零空间向量的符号是不确定的（SVD 的 v 可以差一个负号）。
    #   归一化成 δ1 = +1，读起来才是「电机弱 1% ↔ 变重 1%」这个说法。
    nz = null[0].copy()
    if nz[0] < 0:
        nz = -nz
    nz = nz / abs(nz[0])

    # 数值验证：这两组**物理上完全不同**的参数，观测是否真的分不开
    v1 = np.array([0.01, 0.01, 0.01, 0.01, 0.0])     # 四个电机一起弱 1%
    v2 = np.array([0.0, 0.0, 0.0, 0.0, 0.01])        # 整机变重 1%
    o1, o2 = A @ v1, A @ v2
    resid = np.abs(o1 - o2).max()
    print("""
  A 的奇异值：%s
  秩 = %d / 5  ⇒  **零空间维数 = %d**（不是「接近奇异」，是严格奇异）

  零空间基向量（归一化到 δ1 = +1）：
      [δ1, δ2, δ3, δ4, ε] = [%+.3f, %+.3f, %+.3f, %+.3f, %+.3f]

  ★★★ 这一行就是本章开头那个问题的答案，而且是**算出来的**：

      零空间方向 = (1, 1, 1, 1, −1)

      数值验证（两组物理意义完全不同的参数）：
        θ1 = (1%%, 1%%, 1%%, 1%%,   0  )   ← 四个电机一起弱 1%%
        θ2 = (0,   0,   0,   0,   +1%%)   ← 整机变重 1%%
        A·θ1 = %s
        A·θ2 = %s
        差值 = %.2e      ← 浮点舍入量级

      意思是：**【四个电机一起弱 1%%】和【整机变重 1%%】在全部四个观测通道上
      产生完全相同的观测。** 这不是「精度不够」，是方程组的秩不够。
      飞多久、传感器多好、算法多聪明，都分不开。

    ⚠️ 注意上面这组等价里「变重」是 **+1%%**，不是 −1%%。
       模型里 ε 的符号约定是「质量增加为正」：
           a_z + g = (c̄/m̄)·[U − Σδ_i·u_i − ε·U]
       δ 和 ε 都以**减小净加速度**的方式进入方程，所以两者同号。
       本脚本第一版把符号写反过，于是「验证」出来的差值是 3.7e−02 而不是
       机器精度 —— **一个数值验证如果结果不是机器精度，先怀疑自己的符号。**

    ⇒ 而差模方向（例如只有 δ1 ≠ 0）**被完全解开了** —— 滚转/俯仰/偏航
      三行把「哪个电机」和「整体差多少」张成了不同的子空间。
      这正是 PX4 现有的单状态估计器做不到的事（它只有第一行）。

    ⇒ 所以本章的「改进」既不是「全面胜出」，也不是「小修小补」，而是：
      **把一个 5 维全瞎的问题，降成 4 维可辨 + 1 维仍瞎的问题，
        并且把那 1 维是什么、为什么瞎、什么时候会污染其余 4 维，
        全部写清楚。**
""" % ("  ".join("%.4g" % x for x in s), rank, 5 - rank,
       nz[0], nz[1], nz[2], nz[3], nz[4],
       np.array2string(o1, precision=6), np.array2string(o2, precision=6),
       resid))

    # ---------------- 4.2 消融 ----------------
    sub("4.2  消融实验：每个改进点各自贡献多少")
    print("""
  ★ 消融（ablation）的做法：**每个变体只比上一个多一样东西**，
    两个变体之间的差就是那一样东西的贡献。这是 §9.8 第 4 步的硬要求 ——
    因为「把五个功能一起加进去 vs 原版」只能告诉你「整体有用」，
    无法告诉你钱花在哪一项上。

    A0  原版：只有垂直方程、单参数 ĉ（≡ PX4 现状）
    A1  + 力矩配平方程（滚转/俯仰/偏航三行）+ 重参数化到可辨识的 4 维
    A2  + 准稳态门控（只在配平成立的采样上更新力矩行）
    A3  + 观测加权（按每条方程的**实测**噪声折算）
    A4  + 有效采样约束（饱和 / 角速率跳变时整帧丢弃）

  测试集：6 组场景 × 4 个随机种子 = 24 次闭环仿真
    ① 1 号 −15%%   ② 整机变重 +10%%   ③ 2 号 −10%%
    ④ 1 号 −15%% 且变重 +10%%   ⑤ 四个电机一起 −8%%（共模）   ⑥ 无故障

  ★ 评价量用**可辨识坐标**，不用原始的 (δ, ε)：
      差模误差 = max|d̂ − d_true|，  d = δ − mean(δ)
      共模误差 = |ĉ − ĉ_true|，     ĉ = mean(δ) + ε
    理由见 4.1：ε 里那个不可辨识的成分**本来就没有正确答案**，
    拿它算误差等于在考一道无解的题 —— 那是自欺欺人，不是严格。
""" % ())

    faults = [
        ("① 1号−15%", _fault_eff(0.15, 0)),
        ("② 变重+10%", _fault_mass(0.10)),
        ("③ 2号−10%", _fault_eff(0.10, 1)),
        ("④ 1号−15%&变重+10%", None),
        ("⑤ 共模−8%", None),
        ("⑥ 无故障", None),
    ]

    def _combo4(s):
        s.c[0] = C_NOM * 0.85
        s.m = M_NOM * 1.10

    def _common(s):
        s.c = s.c * 0.92

    faults[3] = ("④ 1号−15%&变重+10%", _combo4)
    faults[4] = ("⑤ 共模−8%", _common)

    def true_params(name):
        if name.startswith("①"):
            return np.array([0.15, 0, 0, 0]), 0.0
        if name.startswith("②"):
            return np.zeros(4), 0.10
        if name.startswith("③"):
            return np.array([0, 0.10, 0, 0]), 0.0
        if name.startswith("④"):
            return np.array([0.15, 0, 0, 0]), 0.10
        if name.startswith("⑤"):
            return np.full(4, 0.08), 0.0
        return np.zeros(4), 0.0

    variants = [("A0 只有垂直（≡PX4）", 0), ("A1 +力矩方程", 1),
                ("A2 +准稳态门控", 2), ("A3 +观测加权", 3),
                ("A4 +有效采样约束", 4)]

    # 先量一次噪声水平（用无故障基线）
    base_sim, t_b, th_b, R_b, tr_b, ekf_b = _simulate(None)
    SIG_ROW = _row_sigma(base_sim, t_b)
    print("  各方程的实测噪声 1σ（从无故障基线里量出来，不拍脑袋）：")
    for i, nm in enumerate(["垂直 (等效油门)", "滚转配平量", "俯仰配平量",
                            "偏航配平量"]):
        print("      %-16s %.5f" % (nm, SIG_ROW[i]))
    print()

    def run_ident(vid, fault, seed):
        sim, t, th, R, tr, ekf = _simulate(fault, seed=seed)
        u, az, w = _collect(sim, t)
        ident = EfficiencyIdentifier(vid, SIG_ROW)
        for k in range(len(t)):
            if t[k] > T_STEADY:                  # 只看稳态段
                ident.update(u[k], az[k], w[k])
        return ident.result(), ident

    store = {}
    for vname, vid in variants:
        err_d, rel_d, err_e, rel_e, fa = [], [], [], [], 0
        n_gate = n_drop = n_seen = 0
        for fname, fault in faults:
            for seed in SEEDS:
                (dh, ch), ident = run_ident(vid, fault, seed)
                n_gate += ident.n_gated
                n_drop += ident.n_drop
                n_seen += ident.n_upd
                dt_, ct_ = true_reduced(*true_params(fname))
                if fname.startswith("⑥"):
                    # 误报判据：对**差模**报出 > 5% 的假故障
                    fa += 1 if np.abs(dh).max() > 0.05 else 0
                    # 无故障行的偏差就是「本底」，单独统计
                    err_e.append(abs(ch - ct_))
                    continue
                err_e.append(abs(ch - ct_))
                rel_e.append(abs(ch - ct_) / max(abs(ct_), 1e-9))
                # ★ 只有**真有差模成分**的场景才参与差模误差统计。
                #   ② 变重、⑤ 共模 的真值 d 是全零，把它们算进来会让
                #   「中位差模误差」被零稀释 —— 那是拿分母凑好看，不是评估。
                if np.abs(dt_).max() > 1e-9:
                    err_d.append(np.abs(dh - dt_).max())
                    rel_d.append(np.abs(dh - dt_).max() / np.abs(dt_).max())
        store[vname] = (np.array(err_d), np.array(err_e), fa,
                        n_gate, n_drop, n_seen,
                        np.array(rel_d), np.array(rel_e))

    rows = []
    for vname, _ in variants:
        ed, ee, fa, rd, re_ = (store[vname][0], store[vname][1],
                               store[vname][2], store[vname][6],
                               store[vname][7])
        rows.append([vname, "%.4f" % np.median(ed), "%.0f%%" % (100 * np.median(rd)),
                     "%.4f" % np.median(ee), "%.0f%%" % (100 * np.median(re_)),
                     "%d / %d" % (fa, len(SEEDS))])
    table(["变体", "中位 差模误差", "中位 差模相对误差", "中位 共模误差",
           "中位 共模相对误差", "误报"], rows)

    def med(v, i=0):
        return float(np.median(store[v][i]))

    v0, v1, v2, v3, v4 = [x[0] for x in variants]
    a0, a1, a2, a3, a4 = (med(v0), med(v1), med(v2), med(v3), med(v4))
    gate_pct = 100.0 * store[v2][3] / max(store[v2][5], 1)
    drop_pct = 100.0 * store[v4][4] / max(store[v4][4] + store[v4][5], 1)

    def drop(a, b):
        return 100.0 * (1.0 - b / max(a, 1e-9))

    print("""
  ★ 逐项贡献（中位**差模**误差，这才是方案真正要估的东西）：

      %-18s  %.4f
      %-18s  %.4f    降 %.0f%%
      %-18s  %.4f    %+.0f%%
      %-18s  %.4f    %+.0f%%
      %-18s  %.4f    %+.0f%%

  ★★ 五条必须说清楚的话：

    ① **主要贡献全部来自 A1（力矩方程 + 可辨识重参数化）**：
       中位差模误差 %.4f → %.4f，相对误差 %.0f%% → %.0f%%，降了 %.0f%%。
       注意 A0 那一行不是「估得不准」—— 它的输出恒等于 0，
       所以误差就等于真值本身。**「不可辨识」在数值上的样子
       就是「无论怎么看都只有一个数」，不是「数字乱跳」。**

       这与 4.1 的秩分析完全一致：新信息的来源是**新的观测方程**，
       不是更好的滤波器、不是更多的数据、不是更好的激励。

    ② 后三项对**差模**的贡献都在噪声里
       （%+.0f%% / %+.0f%% / %+.0f%%）。
       ⚠️ 这不是 bug，这是消融实验最该告诉你的东西：
          **「听起来有道理」和「实际有用」是两件事。**
          A2 门控丢掉了 %.1f%% 的采样（其中既有该丢的也有不该丢的），
          A3 加权在本例的噪声水平下几乎不起作用，
          A4 额外丢掉 %.1f%% 的采样 —— **在本车上是 0，因为它从来不饱和**
          （原因见实验 5.5：本机悬停油门只有 0.46，饱和需要一个
          推力裕度不足的构型才会出现）。差模精度都没有改善。
          如果只做「全功能 vs 原版」的对比，你会把功劳全记在
          最花哨的那一项上 —— 那是自欺欺人。
          ★ §9.8 之所以把**消融实验**写成硬要求，就是为了防这个。

    ③ **共模误差没有改善 —— 而且按理说就不该改善**：
       A0 %.4f → A4 %.4f（相对误差 %.0f%% → %.0f%%）。
       ⚠️ 这一条必须原样写出来：**绝对误差完全没变**（0.0117 → 0.0116，
          差别 0.9%%，远小于 6 个种子之间的散布）；相对误差
          %.0f%% → %.0f%%，也在同一量级。

          **这不是「改进失败」，而是「改进本来就不该动到它」——
           共模那一维就在 4.1 那条零空间方向上，
           加多少条力矩方程都不可能改善它。数据只是又一次印证了秩分析。**

          ★ 顺便也否掉了一个原本很可疑的担忧：重参数化在力矩行里
            丢掉的 m·y 项（共模量 × 配平量）**并没有**把共模估计搞坏。
            ⚠️ 至于差模那一侧约 30%% 的系统性收缩，我原来也以为就是这个
               m·y 项造成的 —— **实验 5.3(b) 用数据否掉了这个解释**：
               真正的机制是偏航方程的系统性失配，那里是逐行算出来的。

       ★ 更要紧的是定位问题：**共模那一维本来就不是本方案的卖点。**
         PX4 现在报的 hover_thrust 已经在报这个投影了，本方案对它的
         唯一「增量」是把名字改清楚（让它知道自己是个投影）。
         ⚠️ 所以**绝不能**把方案宣传成「能估出每个电机的效率」——
            它估的是**差模**，共模它什么也没多知道（实验 5.1 会算死这一点）。

    ④ 最后一列是**误报率**（无故障场景 ⑥ 报出 |d̂| > 5%% 的次数）：
       A4 = %d / %d。⚠️ 注意 A0 那一列是 0 —— 那不是因为它准，而是因为
       **它什么都检测不到**。特异度是靠零灵敏度换来的，不值钱。
       真机上误报率直接决定方案能不能用：一个动不动就喊
       「电机坏了」的估计器会被飞手第一个关掉，然后你的创新就白做了。

    ⑤ 看 ⑤ 那一行（共模−8%%，不参与差模统计）：它的差模误差是 0，
       **这是正确答案**。本方案在这里没有犯错 ——
       它只是**无法告诉你**这个 ĉ=0.08 到底是「四个电机一起弱」
       还是「整机变重」。**「没算错」和「能分辨」是两回事，
       这一节和实验 5.1 的区别就在这里。**
""" % (v0, a0, v1, a1, drop(a0, a1), v2, a2, drop(a1, a2),
       v3, a3, drop(a2, a3), v4, a4, drop(a3, a4),
       a0, a1, 100 * med(v0, 6), 100 * med(v1, 6),
       100 * med(v0, 6) - 100 * med(v1, 6),
       drop(a1, a2), drop(a2, a3), drop(a3, a4),
       gate_pct, drop_pct,
       med(v0, 1), med(v4, 1), 100 * med(v0, 7), 100 * med(v4, 7),
       100 * med(v0, 7), 100 * med(v4, 7),
       store[v4][2], len(SEEDS)))

    # ---------------- 4.3 分场景细看 ----------------
    sub("4.3  分场景细看：改进方案在哪些故障上真的有效")
    print("""
  ⚠️ 4.2 的中位数把 6 个场景混在了一起，会掩盖一件关键的事：
     **改进方案对「差模故障」有效，对「共模故障」完全无效。**
     拆开看（用 A4，%d 个种子的中位；坐标是 4.1 定义的 d 和 ĉ）：""" % len(SEEDS) + """
""")
    rows = []
    for fname, fault in faults:
        dhs, chs = [], []
        for seed in SEEDS:
            (dh, ch), _ = run_ident(4, fault, seed)
            dhs.append(dh)
            chs.append(ch)
        dh = np.median(np.array(dhs), axis=0)
        ch = float(np.median(chs))
        dt_, ct_ = true_reduced(*true_params(fname))
        rows.append([fname,
                     " ".join("%+.3f" % x for x in dt_), "%+.3f" % ct_,
                     " ".join("%+.3f" % x for x in dh), "%+.3f" % ch,
                     "%.3f" % np.abs(dh - dt_).max(), "%.3f" % abs(ch - ct_),
                     ("%.0f%%" % (100 * np.abs(dh - dt_).max()
                                  / np.abs(dt_).max())
                      if np.abs(dt_).max() > 1e-9 else "—")])
    table(["场景", "真值 d1..d4", "真值 ĉ", "估计 d̂1..d4", "估计 ĉ",
           "差模误差", "共模误差", "差模相对误差"], rows)

    ed = [float(rows[i][5]) for i in (0, 2, 3)]
    er = [float(rows[i][7][:-1]) for i in (0, 2, 3)]
    print("""
  ★★ 读法（这三条比上面那张表重要得多）：

    · **①③④（真有差模成分的三组，共 %d 个种子中位）**：
      差模误差 %.3f ~ %.3f，相对误差 %.0f%% ~ %.0f%%。
      对 10%%~15%% 的单电机故障来说，这是**可用的诊断精度** ——
      「有电机坏了，是 1 号（或 2 号）」这个结论能答对，
      而这正是 PX4 现在完全给不出的信息。
      ⚠️ 注意①和③的 d̂ **形状对、符号对，但幅度只有真值的 2/3 左右**
         （比如③：真值 +0.075，估出 +0.049）。
         这是**系统性收缩**（不是随机误差，也不是「再飞久一点就好」）。
         用法上必须知道这一点：
         **本方案能可靠回答「哪个电机」，暂时还不能可靠回答「坏了多少」。**
         ⇒ 收缩的机制在 **5.3(b)** 里逐行算清了：不是线性化误差
           （我原来以为是），而是**偏航方程的系统性失配**。
           修法也在那一节，代价不小 —— 先往下看完再决定要不要修。

    · **⑤ 四个电机一起 −8%%**：差模误差 %.3f ——
      估计值是全零，而**真值也是全零**，所以这条**没有错**。
      本方案在这里没有撒谎，它只是**无法分辨**：
      ĉ̂ ≈ %+.3f 这个数，和「整机变重 8%%」给出的 ĉ̂ 是同一个数。
      这正是 4.1 那条零空间方向的直接后果，实验 5.1 会把它算死。
      ⚠️ 对外表述必须是「**共模推力缺口 ĉ**」，
         不能写成「共模效率」更不能写成「质量变化」——
         写了就是在声称一个数学上不存在的能力。

    · **④ 1 号 −15%% 且整机变重 +10%%（差模 + 共模同时发生）**：
      d̂ 和单独发生①时**几乎逐位一样**，而 ĉ̂ 从 +0.044 涨到 +0.131。
      ★ **这一行是本方案最强的一处证据：它说明 d 和 ĉ 真的被分开了，
        而不是「把所有偏差都塞进某一个参数」。**
        一个只能在单一故障下工作的辨识器没有工程价值。

    · **⑥ 无故障**：差模误差 %.3f、共模误差 %.3f。
      这一行是方案能不能上真机的**第一道门**：
      本底噪声必须显著小于要检测的故障幅度（这里 ~1%% 对 10%%~15%%）。

  ⇒ 本改进方案的**准确定位**：**差模故障诊断器**，
     不是「电机效率辨识器」。
     把它宣传成后者是错的 —— 那正是第 9 章 9.0 节反对的
     「为了创新而创新」。它真正的价值只有一句：
     **PX4 现在连差模故障都看不见，而差模故障恰恰是能看见的那一半。**
""" % (len(SEEDS), min(ed), max(ed), min(er), max(er),
       float(rows[4][5]), float(rows[4][4]),
       float(rows[5][5]), float(rows[5][6])))


# ===========================================================================
# 5. 诚实报告：改进方案在什么条件下失效
# ===========================================================================
#
#   §9.8 第 3 步的原文要求：
#     「必须**写清楚失效条件**（什么情况下你的方法也会崩）。
#       这一条最重要：能诚实说出自己方法局限的人，才是真懂。」
#
#   所以这一节的每一条都**算出来**，不是「可能吧」。
#
def exp5():
    hr("实验 5  诚实报告：改进方案在什么条件下失效")

    print("""
  ⚠️ 这一节是全课最重要的一节，也是最容易被写坏的一节。

     §9.8 第 3 步的原文要求：
       「必须**写清楚失效条件**（什么情况下你的方法也会崩）。
         这一条最重要：能诚实说出自己方法局限的人，才是真懂。」

     所以下面每一条都**算出来**，不给「可能吧」「一般来说」。
     每一条的格式都是：触发条件 → 定量后果 → 工程对策。
""" % ())
    base_sim, t_b, th_b, R_b, tr_b, ekf_b = _simulate(None)
    SIG_ROW = _row_sigma(base_sim, t_b)

    def ident_run(fault, variant=4, seed=1, az_bias=0.0):
        sim, t, th, R, tr, ekf = _simulate(fault, seed=seed)
        u, az, w = _collect(sim, t)
        if az_bias:
            az = az + az_bias
        ident = EfficiencyIdentifier(variant, SIG_ROW)
        for k in range(len(t)):
            if t[k] > T_STEADY:
                ident.update(u[k], az[k], w[k])
        return ident.result(), sim, t

    # ---- 5.1 共模故障 ----
    sub("5.1  失效点一：共模故障永远不可辨识（这是数学，不是工程）")
    A, s, rank, null = nullspace_report()
    nz = null[0].copy()
    if nz[0] < 0:
        nz = -nz
    nz = nz / abs(nz[0])
    v1 = np.array([0.01, 0.01, 0.01, 0.01, 0.0])
    v2 = np.array([0.0, 0.0, 0.0, 0.0, 0.01])
    print("""  4.1 的静态结论：零空间方向 [δ1..δ4, ε] = [%+.2f, %+.2f, %+.2f, %+.2f, %+.2f]。
  取 θ1 = (1%%,1%%,1%%,1%%, 0) 与 θ2 = (0,0,0,0, +1%%)，A·θ 的差 = %.1e，
  即 **四个电机一起弱 1%% ≡ 整机变重 1%%。**

  ★ 更强的结论（可以手算，不必信我）：悬停闭环下的配平条件是
        Σ(1−δ_i)u_i = (1+ε)·U          （垂直，U = 4Th）
     两边同减 Σu_i、再代入 U = Σu_i（标称悬停点）可得**严格**关系
        ĉ ≡ mean(δ) + ε = (δ + ε)/(1 + ε)                  …(5.1)
     于是「共模效率损失 x」的真值是 ĉ = x，而「变重 x」的真值是
     ĉ = x/(1+x) —— 两者只差

        Δĉ = x − x/(1+x) = **x²/(1+x)**                     …(5.2)

     ★ 这是个**二阶小量**：1%% 的故障只差 0.01%%，20%% 的故障也只差 3.3%%。
       也就是说「一起变弱」和「变重」在数值上几乎是同一个观测 ——
       这正是零空间方向 (1,1,1,1,−1) 的**定量形态**。
       ⚠️ 注意 (5.2) 说明：x 小的时候两者**完全**不可分，x 大到 20%% 时
          才露出 3.3%% 的缝 —— 而 3.3%% 比 d̂ 的噪声（1%% 量级）大不了多少。
          **不要拿这个二阶差去「反解」质量变化**，那是把噪声当信号。

  ⚠️ 静态方程上的等价，会不会被闭环动力学破坏？不会 —— 但**必须验证**。
     下面把两组物理上完全不同的故障跑成闭环，看估计器能不能分辨
     （A4，%d 个种子中位；坐标是 4.1 定义的 d 和 ĉ = mean(δ)+ε）：
""" % (nz[0], nz[1], nz[2], nz[3], nz[4],
        np.abs(A @ v1 - A @ v2).max(), len(SEEDS)))
    rows = []
    pairs = [("四个电机一起 −5%", _common_scaled(0.05), np.full(4, 0.05), 0.0,
              "整机变重 +5%", _fault_mass(0.05), np.zeros(4), 0.05),
             ("四个电机一起 −10%", _common_scaled(0.10), np.full(4, 0.10), 0.0,
              "整机变重 +10%", _fault_mass(0.10), np.zeros(4), 0.10),
             ("四个电机一起 −20%", _common_scaled(0.20), np.full(4, 0.20), 0.0,
              "整机变重 +20%", _fault_mass(0.20), np.zeros(4), 0.20)]
    for n1, f1, d1, e1, n2, f2, d2, e2 in pairs:
        for nm, ff, dd, ee in ((n1, f1, d1, e1), (n2, f2, d2, e2)):
            ds, cs = [], []
            for seed in (1, 2, 3, 4):
                (dh, ch), _, _ = ident_run(ff, seed=seed)
                ds.append(dh)
                cs.append(ch)
            dh = np.median(np.array(ds), axis=0)
            ch = float(np.median(cs))
            dt_, ct_ = true_reduced(dd, ee)
            # (5.1) 的解析值：ĉ = (mean(δ)+ε)/(1+ε) —— 用的是**精确配平**，
            # 不是线性化。它才是这两种故障真正的共同极限。
            m_ = float(np.mean(dd))
            c_pred = (m_ + ee) / (1.0 + ee)
            rows.append([nm, "%+.3f" % ct_, "%+.4f" % c_pred,
                         "%.4f" % np.abs(dh).max(),
                         "%+.4f" % ch, "%.4f" % abs(ch - c_pred)])
    table(["场景", "真值 mean(δ)+ε", "解析 ĉ=(δ+ε)/(1+ε)", "max|d̂|",
           "估计 ĉ", "ĉ 误差（对解析值）"], rows)
    dmax = max(float(r[3]) for r in rows)
    cmax = max(float(r[5]) for r in rows)
    xgap = [0.05, 0.10, 0.20]
    gpred = [x * x / (1.0 + x) for x in xgap]
    gmeas = [abs(float(rows[2 * i][4]) - float(rows[2 * i + 1][4]))
             for i in range(len(pairs))]
    dgap = max(gmeas)
    print("""
  读法（**这两种故障在表里长得一模一样**）：

    · 三对场景（−5%% vs +5%%、−10%% vs +10%%、−20%% vs +20%%）的
      ĉ̂ 两两相差 %.4f / %.4f / %.4f，而 (5.2) 的解析预测是
      x²/(1+x) = %.4f / %.4f / %.4f —— **逐对吻合**。
      差模 |d̂| 全部 ≤ %.4f，ĉ 误差最大 %.4f。
      **闭环动力学没有破坏这个等价性** —— 因为等价性来自
      **观测方程的结构**，而不是来自某个近似的动力学假设。
      ⚠️ 差距随 x 增大，但那是 0.0024 → 0.0333 的二阶量；
         想靠它反解 ε 就是拿噪声当信号（见上面 (5.2) 的警告）。

  ⇒ **失效条件一：故障是共模的**（四个电机一起老化、电池电压模型整体偏移、
     整机气动效率整体下降、进气堵塞）。此时：
       · d̂ 全部报 0 —— **这是正确的结论**（确实没有差模故障）
       · ĉ̂ 报出真实幅度 —— **但「一起变弱」和「变重」分不开**
       · 两者之间的分配是任意的，**不会随飞行时间变好**。

  ⚠️ 更要命的推论：共模效率下降 10%% 时，本方案会告诉上层「推力缺口 10%%」。
     如果飞控把这个缺口当成「飞机变重了 10%%」去改悬停推力，
     **它不是不管，而是往反方向修。**
     工程上必须给 ĉ 加限幅，或者干脆冻结 ĉ 只让它极慢地在线更新，
     并且把它命名为「**共模推力缺口**」而不是「质量」或「效率」。

  ★ 那 PX4 原版呢？它报的 hover_thrust 就是同一个 ĉ 的另一种写法。
    **在这一点上本方案和 PX4 一模一样，谁也没有多知道任何东西。**
    改进的增量**只在差模**。这句话必须写进论文的 Limitations。
""" % (gmeas[0], gmeas[1], gmeas[2], gpred[0], gpred[1], gpred[2],
       dmax, cmax))

    # ---- 5.2 一个被数据否掉的假设 ----
    sub("5.2  一个被数据否掉的假设：混控标定误差**不会**混进 d̂")
    print("""
  ★ 写这一节不是因为它是失效点，正相反 —— 因为它是**我预判会失效、
    实测却没有失效**的机制。研究里这种时刻比「又验证了一个优点」值钱得多，
    它逼你回去把推导重看一遍。

  我原本的推理（听起来非常有道理）：
    三条力矩方程左边全是**电机指令**，而「指令 → 力矩」依赖混控矩阵 B 里
    的标称力臂 l。如果飞控以为的力臂和实际不一致，那么「同样的指令 →
    同样的力矩」不成立，d̂ 就会把**几何误差**读成**效率误差**。

  实测（混控用 l_mix·l，机体是真实的 l，故障固定 1 号 −15%%）：
""" % ())
    d_true = np.array([0.1125, -0.0375, -0.0375, -0.0375])
    rows = []
    for lmix in (0.90, 0.95, 1.00, 1.05, 1.10):
        def f(s, _lm=lmix):
            s.c[0] = C_NOM * 0.85
            s.l_mix = _lm
        (dh, ch), _, _ = ident_run(f)
        rows.append(["%.2f（偏差 %+.0f%%）" % (lmix, 100 * (lmix - 1)),
                     "%+.4f" % dh[0], "%+.4f" % dh[1], "%+.4f" % dh[2],
                     "%+.4f" % ch,
                     "%.4f" % np.abs(dh - d_true).max()])
    table(["l_mix", "d̂1", "d̂2", "d̂3", "ĉ̂", "差模误差"], rows)
    spread = (max(float(r[1]) for r in rows)
              - min(float(r[1]) for r in rows))
    print("""
  ⚠️ 力臂偏差从 −10%% 扫到 +10%%，d̂1 的变化只有 %.4f ——
     **比噪声还小，完全没有影响。**

  ★ 为什么？回去看推导的第二行：

       M_roll = −l·c̄·[ (u1+u2−u3−u4) − (δ1u1+δ2u2−δ3u3+δ4u4) ] = 0

     令 M_roll = 0，两边同除以 −l·c̄ —— **l 被约掉了。**
     估计器解的是「归一化之后的配平条件」，它只用到**指令的相对关系**，
     根本不需要知道 l 的数值。

  ⇒ 更深一层：**姿态环的积分器会把混控矩阵的任何标定误差完全吸收掉。**
     混控以为力臂是 l_mix·l，于是为了产生 M_des 它给出的指令偏了；
     但机体实际产生的力矩因此不为零，姿态环的积分继续拧，
     直到**真实的**力矩为零。稳态与 l_mix 无关。
     ★ 这同时说明：**这类误差在实飞中不会以「力矩偏差」的形式出现，
       只会以「姿态环积分量变大 / 响应变慢 / 权限余量变小」的形式出现。**

  ⇒ 所以这条**不是**失效点。真正让力矩方程失效的，是它的前提
     「M = 0」本身不成立的那些时刻 —— 见 5.3，那才是真问题。

  ★ 那什么标定误差**会**混进来？只有那些**改变「指令 → 推力」比例关系**
    的误差，例如：
       · 电机/桨的**固有散布**（出厂就存在，且无法与退化区分）→ 5.3
       · 推力曲线非线性（把 δ 和 THR_MDL_FAC 搅在一起）→ 5.7
       · 加速度计零偏（改变垂直方程）→ 5.6
     共同点：**它们都出现在「观测方程内部」，而不是出现在
     「方程两边同乘的那个系数」上。**
     ⚠️ 这条区分标准可以推广：**凡是能被负反馈积分吸收掉的误差，
        都不会污染辨识；只有进到观测方程里的误差才会。**
        判断方法很机械：把误差代进方程，看它能不能被约掉。
""" % spread)

    # ---- 5.3 基线不可观测 + 系统性低报 ----
    sub("5.3  失效点二：出厂散布不可观测，而且故障幅度会被系统性低报")
    print("""
  这一节把两个**不同的**问题放在一起，因为它们常常被混为一谈：

    (a) **绝对量不可观测**：δ 里同时包含「出厂就有的散布」和「后来的退化」，
        估计器只能看到两者之和。
    (b) **故障幅度被压缩**：4.3 已经看到 d̂ 只有真值的 ~2/3。
        这一节把压缩比算成一条曲线。

  ── (a) 出厂散布不可观测 ────────────────────────────────────────────
  本仿真给每个电机注入了 1σ = %.0f%% 的固有散布 η（`SPREAD`），
  它和退化 δ 在方程里的位置**完全相同** —— 都是乘在 u_i 上的常数因子。
  于是 d̂ 只能给出 η 和 δ 的**和**（的差模部分）。逐种子实测：

""" % (100 * SPREAD))
    rows = []
    for seed in SEEDS:
        eta = np.random.default_rng(seed + 77).normal(0.0, SPREAD, 4)
        eta_d = eta - eta.mean()
        (dh, ch), _, _ = ident_run(None, seed=seed)
        rows.append(["seed %d" % seed,
                     " ".join("%+.3f" % float(x) for x in eta),
                     " ".join("%+.3f" % float(x) for x in eta_d),
                     " ".join("%+.3f" % float(x) for x in dh),
                     "%.3f" % float(np.abs(dh - eta_d).max())])
    table(["", "注入的 η1..η4（出厂散布）", "η 的差模部分",
           "估计 d̂1..d̂4（无故障）", "两者之差"], rows)
    errs = [float(r[4]) for r in rows]
    print("""
  ★ 读法：d̂（无故障时）**几乎逐位等于出厂散布 η 的差模部分**，
    中位偏差只有 %.3f。也就是说：

      **本方案报出的「1 号电机差 1.7%%」里，有多少是出厂散布、
        有多少是这些天飞出来的退化，它不知道，也不可能知道。**

  ⇒ **失效条件二（a）：绝对 δ 值没有物理意义。**
     要让它有意义，必须在上次上电 / 出厂试车时**存一份基线**，
     之后只报**相对基线**的变化量。
     ⚠️ 这条限制意味着：**拿本方案去做「出厂检验」是错的** ——
        它会把一批合格但有散布的电机判成「有故障倾向」。
        它能做的是「趋势监测」：同一架飞机，δ 相对自己上次的基线漂了多少。
""" % (np.median(errs),) + """

  ── (b) 故障幅度被系统性低报 ────────────────────────────────────────
  ⚠️ 4.3 那张表里 d̂ 只有真值的 ~2/3。我一开始以为是 4.1 的线性化误差
     （重参数化把力矩行里的 m·y 项丢掉了，而 |y| 随故障增大 —— 听起来很顺）。
     **下面这段数据把这个解释否掉了**：真实机制具体得多，而且能逐行算出来。
  先扫一遍故障幅度，看低报到底长什么样：
""")
    rows = []
    for d in (0.05, 0.10, 0.15, 0.25, 0.40, 0.60):
        (dh, ch), _, _ = ident_run(_fault_eff(d))
        rows.append(["%.0f%%" % (100 * d), "%+.4f" % (0.75 * d),
                     "%+.4f" % dh[0], "%.2f" % (dh[0] / (0.75 * d)),
                     "%+.4f" % ch])
    table(["1 号真实效率损失", "真值 d1", "估计 d̂1", "d̂1 / 真值", "ĉ̂"], rows)
    rr = [float(r[3]) for r in rows]
    dvals = (0.05, 0.10, 0.15, 0.25, 0.40, 0.60)
    i_min = int(np.argmin(rr))
    i_max = int(np.argmax(rr))
    md_beg = float(rows[0][2]) - float(rows[0][1])
    md_end = float(rows[-1][2]) - float(rows[-1][1])
    # ---- 失配诊断：把估计器的三条力矩方程拿到**真值**处求值 ----
    sim_b, t_b2, _, _, _, _ = _simulate(_fault_eff(0.15), seed=1)
    u_b, az_b, _ = _collect(sim_b, t_b2)
    k0 = int(np.searchsorted(t_b2, T_STEADY))
    ub = u_b[k0:].mean(axis=0)
    A_b = build_regressor(ub)
    dtv = np.array([0.15, 0.0, 0.0, 0.0, 0.0])
    y_m = np.array([ub[0] + ub[1] - ub[2] - ub[3],
                    ub[0] - ub[1] - ub[2] + ub[3],
                    float(SIGMA @ ub)])
    nm_row = ["滚转 行2", "俯仰 行3", "偏航 行4"]
    mrows, mism = [], []
    for i in range(3):
        pred = float(A_b[i + 1] @ dtv)
        mis = float(y_m[i] - pred)
        mism.append(mis)
        mrows.append([nm_row[i], "%+.4f" % y_m[i], "%+.4f" % pred,
                      "%+.4f" % mis, "%.1f σ" % (abs(mis) / SIG_MOM_FLOOR)])
    table(["方程（在真值 δ1=0.15 处求值）", "y（实测配平量）", "A·δ_true",
           "失配 y − A·δ", "失配 / 噪声下限"], mrows)
    # 偏航行左端的来历：B 的第 3 行 = κ·C_NOM·σᵀ  ⇒  σᵀ·B⁻¹ = e₃ᵀ/(κ·C_NOM)
    sBt3 = float(SIGMA @ np.linalg.inv(B_NOM) @ np.array([0.0, 0.0, 1.0, 0.0]))
    ratio_mis = abs(mism[2]) / max(abs(mism[0]), abs(mism[1]))
    myaw = float(KAPPA * C_NOM * (SIGMA @ (dtv[:4] * ub)))
    print("""
  ⇒ 收缩比 %.2f ~ %.2f，**没有单调趋势**（最小 %.2f 在 %.0f%%，最大 %.2f 在
     %.0f%%）。⚠️ 所以正确的说法**不是**「故障越大越不可信」，而是：

       **低报是系统性的（约 −17%% ~ −30%%），不是随机误差。**

     它可以被一个标定系数粗修（例如把 d̂ 乘 1.3），但**绝对**低报随故障
     幅度增长（%.4f → %.4f），所以它给不出能用于放行决策的绝对精度。
     低报这个**方向**很危险：维护人员会以为「还能再飞」。
     对照 5.5 的饱和失效（也是低报），两者叠加时后果更重。

  ⇒ 为什么低报？不是「线性化误差随故障增长」那种含糊说法。
     估计器解的是最小二乘 —— 它返回**让三条方程总体残差最小**的那个 d，
     而不是真值。既然 A·δ_true ≠ y（失配不为零），它的解就**必然**偏离
     真值。**这不是精度问题，是模型与数据不自洽。** 失配是多少，直接算：

        （见上表）

     ★ 滚转/俯仰两行的失配（%.4f / %.4f）来自「电机固有散布 η」：
       估计器不知道 η（1%%），于是把 Σa_i·η_i·u_i 当成故障读走了 ——
       这与 5.3(a) 是同一件事的两面。
     ★★ 偏航行的失配 %.4f 是它们的 **%.0f 倍** —— **它才是低报的主因。**

  ⇒ 偏航行为什么必然失配？两条，都可以自己验算：

     (i) **混控矩阵的第 3 行与 σ 平行**（B 的偏航行 = κ·C_NOM·σᵀ）。
         于是 σᵀ·B⁻¹ = e₃ᵀ/(κ·C_NOM)，也就是
             σᵀu = σᵀ·B⁻¹·τ = τ_yaw /(κ·C_NOM)
         —— 数值验证：σᵀ·B⁻¹·e₃ = %.4f，而 1/(κ·C_NOM) = %.4f，吻合。
         本仿真的指令偏航恒为 0 ⇒ **σᵀu ≡ 0**。
         换句话说：偏航行的左端**恒等于 0**，它度量的是「指令偏航力矩」，
         不是任何机体状态。
     (ii) 它的右端 σᵀ(δ∘u) 却是**故障造成的真实偏航力矩**（除以 κ·C_NOM）。
          真值 δ1 = 0.15 时 m_yaw = κ·C_NOM·σᵀ(δ∘u) = %.4f N·m ≠ 0，
          而没有任何人抵消它。

     于是这条方程实际在说「真实的偏航力矩 = 0」，而它并不为 0。

  ⇒ ★★ 工程后果（这是本方案最大的结构性弱点）：

     **偏航行不能删 —— 删了力矩方程只剩 2 条，三个差模方向定不全（秩不足）；
       也不能全信 —— 信了就有 %.0fσ 的系统性失配，把 d̂ 压低三成。**

     ⚠️ 那什么时候偏航力矩真的为 0？**只有偏航轴被真正配平的时候。**
        真机上这要求：偏航环有积分、权限足够、角速率已收敛、无侧风、
        且没有未建模的阻扭（桨损伤会同时改 c 和 κ，见 `_fault_prop`，
        而本模型里 κ 是常量 —— **它对此完全瞎**）。
        本仿真**从不满足**这个条件（没有偏航自由度）。所以它是本章
        所有数字里的一个**已知污染源** —— 我把这一点明写在这里，
        而不是让它藏在「精度约 30%%」这句含糊话后面。

  ⇒ 修它的办法（按代价从低到高）：
     1. 把「力矩 = 0」换成**动态配平条件**（残差 = J·ṙ + 阻尼项），
        即把偏航环的积分量 / 偏航角速率一起进方程；
        代价是滤波器从 4 维涨到 6~7 维，而且姿态回路的状态本身就难标定；
     2. 把 κ 一起辨识（桨损伤那一课）：它又是一个「单工作点不可辨识」的
        对子，和实验 2、5.7 同一个形状；
     3. 干脆把偏航行当成**带未知偏置的方程** —— 但偏置一进来，
        三个差模参数的约束就只剩 2 条，问题重新变成欠定。
        ⚠️ **这一条是三选一里最诱人、也最错的**：它把失效藏起来，
           而不是修好。写在这里是为了让你少走一次弯路。
     ⚠️ 三条都不是「调个参数」能解决的。**这就是为什么「提出改进」
        在四步法里最短 —— 真正的活是把改进做对。**
""" % (min(rr), max(rr), min(rr), 100 * dvals[i_min], max(rr),
       100 * dvals[i_max], md_beg, md_end, mism[0], mism[1], mism[2],
       ratio_mis, sBt3, 1.0 / (KAPPA * C_NOM), myaw,
       abs(mism[2]) / SIG_MOM_FLOOR))
    # ---- 5.3 风扰 ----
    sub("5.4  失效点三：外部力矩扰动与差模故障在力矩通道里观测等价")
    print("""
  姿态环的配平量要抵消的**不只是**电机效率偏差，还有**一切外部力矩**：
  风、机架不对称、线缆拖拽、载荷偏心。它们在观测方程里的位置完全相同 ——
  方程右边是待估的 δ，左边是配平量，而配平量分不清自己是替谁在配平。

  定量：悬停时加一个常值滚转力矩扰动 m_d，故障为 0，看它被读成多大的 d。

      解析推导（真值，不近似）：稳态时总滚转力矩为 0
          −l·C_NOM·Σa_i(1−δ_i)u_i + m_d = 0,     a = (+1,+1,−1,−1)
      而估计器的滚转方程是 a·(d̂∘u) = a·u —— **它不知道 m_d 存在**，
      于是把多出来的那一份全部读成效率偏差。δ = 0 时：
          a·u = m_d/(l·C_NOM)  ⇒  d̂1 = d̂2 = −d̂3 = −d̂4 = +m_d/(4·l·C_NOM·Th)

  ⚠️ 注意三个符号：扰动的符号**直接决定**估计值往哪边偏（这里是正号），
     这不是「量级估算」，是等式两边移项的结果。

""" % ())
    base_d = []
    for seed in SEEDS[:3]:
        (dh0, _ch0), _, _ = ident_run(None, seed=seed)
        base_d.append(float(np.abs(dh0).max()))
    floor_d = float(np.median(base_d))
    rows = []
    for md in (0.005, 0.010, 0.020, 0.050, 0.100):
        def f(s, _m=md):
            s.m_dist[0] = _m
        (dh, ch), _, _ = ident_run(f)
        pred = md / (4.0 * L_ARM * C_NOM * TH_HOVER)
        err = abs(dh[0] - pred) / pred
        rows.append(["%.3f" % md, "%+.4f" % pred,
                     " ".join("%+.4f" % x for x in dh),
                     "%.0f%%" % (100 * err) if pred > floor_d
                     else "—（预测低于噪声底）"])
    table(["滚转力矩扰动 m_d", "解析预测 d̂1..d̂4", "实测 d̂1..d̂4",
           "相对偏差"], rows)
    big = [float(r[3][:-1]) for r in rows if r[3].endswith("%")]
    print("""
  ⇒ 读法（先记住噪声底：**无故障、无扰动时 max|d̂| 的中位 = %.4f**）：

    · m_d = 0.005 / 0.010 两行：**预测值低于噪声底**，比值没有意义
      （它们的实测 d̂ 也确实没超过噪声底）。这两行留着是为了说明
      「不是所有小扰动都能被看见」——**这也是结论的一部分**。
    · m_d ≥ 0.02 起，四行的 d̂ 都沿着 **a = (+1,+1,−1,−1)** 这个方向，
      与解析预测的相对偏差最大 %.0f%%（m_d = 0.02 那一档，因为它刚刚
      探出噪声底，只有 1.5 倍）。**这不是仿真 artifact，
      是可以拿纸笔算出来的结构性等价。**
    · 符号：d̂1、d̂2 同号为正，d̂3、d̂4 同号为负 —— 因为 1/2 号电机在
      滚转轴同侧。**估计器给出的物理解释是自洽的**：从滚转通道看，
      「1 号弱、2 号弱」和「机体被持续往右滚」产生同样的配平量。

  ⇒ **失效条件三：任何持续的外部力矩都会被读成「某几个电机的效率偏差」。**
     要分开它们必须用俯仰/偏航通道 —— 而一个**任意的**外部力矩会同时
     污染这三个通道，那就真的分不开了。
     一个更隐蔽的例子：**机架不对称 / 电池偏置 / 线缆拖拽**产生的
     常值力矩，在地面检查时看不出来，一起飞就被记到电机账上。

  ⇒ 工程后果：**这套估计器在有风环境里要么关闭，要么必须同时估计风扰。**
     0.100 N·m 的滚转力矩被读成 d̂1 = %+.4f，而 15%% 的真实电机故障
     对应 d̂1 = 0.1125 —— **也就是说：一个约 0.1 N·m 的风扰/偏心，
     看起来和「1 号电机掉了 15%%」是同量级的故障。**
     而 0.1 N·m 在 5 寸机上是什么概念？（力臂 0.125 m ⇒ 两侧推力差 0.8 N
     ⇒ 约 2.5%% 的推力不对称。）**这是很容易被忽略的量级。**
""" % (floor_d, max(big), float(rows[-1][2].split()[0])))

    # ---- 5.4 饱和 ----
    sub("5.5  失效点四：推力裕度不足时饱和 —— 方案不会撒谎，但它会闭麦")
    print("""
  三条力矩方程都建立在「u 就是混控输出」之上。一旦某个电机指令被
  clip 到 0 或 1，实际输出 ≠ 混控输出，方程不成立，
  而估计器还在用被截断的 u 当回归量 —— **它是拿错的数据在解方程。**

  ⚠️ 关键认知：**饱和的风险不取决于故障大小，取决于推力裕度。**
     本机悬停油门只有 %.3f，即使 1 号电机掉 70%% 也照样悬停、
     指令不过 0.85。所以下面**不是**扫故障幅度，
     而是扫**悬停油门水平**（等效加载荷），固定 1 号 −15%%：

      悬停油门 ≈ Th_nom·(1+w)/(1−δ/4)，w 是载荷比例

""" % TH_HOVER)
    rows = []
    for w in (0.0, 0.30, 0.55, 0.80, 0.90, 0.95):
        def f(s, _w=w):
            s.c[0] = C_NOM * 0.85
            s.m = M_NOM * (1.0 + _w)
        res = {}
        for vid in (3, 4):
            (dh, ch), sim, t = ident_run(f, variant=vid)
            n0 = int(len(t) * 0.8)
            u1 = np.array(sim.log["u1"])[n0:]
            uu = np.array([sim.log["u1"], sim.log["u2"],
                           sim.log["u3"], sim.log["u4"]])[:, n0:]
            res[vid] = (float(dh[0]), float((uu >= 0.999).mean()),
                        float(u1.mean()), float((uu >= 0.98).any(axis=0).mean()))
        rows.append(["%.2f" % w, "%.3f" % res[4][2],
                     "%.0f%%" % (100 * res[4][3]),
                     "%.0f%%" % (100 * res[4][1]),
                     "%+.4f" % res[3][0], "%+.4f" % res[4][0],
                     "%.3f" % abs(res[4][0] - 0.1125)])
    table(["载荷 w", "1 号平均指令", "任一指令≥0.98 占比", "指令≥0.999 占比",
           "d̂1 (A3 无门控)", "d̂1 (A4 有门控)", "A4 差模误差"], rows)
    d3_last = float(rows[-1][4])
    d4_last = float(rows[-1][5])
    e4_last = float(rows[-1][6])
    d3_prev = float(rows[-2][4])
    d4_prev = float(rows[-2][5])
    sat_prev = float(rows[-2][2][:-1])
    d3_pct = 100 * d3_last / 0.1125
    d3p_pct = 100 * d3_prev / 0.1125
    print("""
  ⇒ 这张表最反直觉的一行是 **w = 0.90**：1 号的平均指令已经 0.990，
     但「真饱和占比」还是 **0%%** —— 因为平均 0.990 的那些采样里，
     绝大多数并没有真的顶到 1.0。而同时，A4 自己的 0.98 门限
     已经被 **100%%** 的采样触发，它闭麦了。
     ⚠️ 这就是两张占比列必须分开看的原因：

       · 「任一指令 ≥ 0.98」= **A4 自己设的门限**（它的闭麦触发条件）；
       · 「指令 ≥ 0.999」= **真正的饱和**。
     A4 的门限**故意比真饱和提前**：这是设计选择（留 2%% 的裕度），
     代价是它会在**还不必闭嘴的时候**就闭嘴。

     两种变体的表现**完全不同**，这是本节最有价值的一处对比：

       · **A3（没有饱和门控）**：w 从 0.00 到 0.90，d̂1 一直是 %+.4f
         （真值的 %.0f%%）—— ⚠️ **这个低报不是饱和造成的**，
         它是 5.3(b) 那条系统性偏差，从 w = 0.80（d̂1 = %.4f）到
         w = 0.90 完全没变。**不要把一个一直都在的偏差算到饱和头上。**
         直到 w = 0.95 出现 25%% 的真饱和，d̂1 才掉到 %+.4f（%.0f%%）——
         **这才是饱和的额外代价：在本来就低报的基础上再低报。**
         它没有报错方向，但它撒了谎 —— 「1 号电机只坏了 %.0f%%」会让
         维护人员以为还能再飞几架次。**这是诊断系统最危险的失败模式：
         不是报错，而是在最该报警的时候把故障报小。**
       · **A4（有饱和门控）**：w = 0.90 起 d̂1 = %+.4f —— ⚠️ 这不是
         「估准了」，是**它闭麦了**：差模误差 %.3f，等于什么都没报。
         它把 %s%% 的采样整帧丢弃后，力矩方程已经**没有一条是可信的**。
         ★ 而且请注意：**它在真饱和占比为 0%% 的时候就已经闭麦了。**
           在诊断系统里，这种「宁可漏报也不误报」的偏置通常是对的，
           但你必须知道自己的方案有这个偏置 —— 否则你会以为
           「没报警 = 没故障」。

  ★ 所以 A4 的设计哲学是：**宁可说「我不知道」，也不说一个错的数字。**
     这在诊断系统里通常是对的（假阴性好过假阳性），但它意味着
     **一个推力裕度不足的飞机，在大故障时恰好什么都诊断不出来** ——
     而这正是最需要诊断的时刻。
     真正负责任的做法是让估计器**同时上报一个「置信度 / 闭麦标志」**，
     让上层知道「我不是没故障，我是看不见」。
     ⚠️ 这句话要反过来读一遍才完整：**如果上层不读这个标志，
        A4 的「闭麦」在日志里和「一切正常」长得一模一样。**
        所以闭麦标志必须走**独立的告警通道**，不能只塞进估计值里。

  ⇒ 这也解释了实验 4.2 里 A4 那一点点「负贡献」：
     它用一点精度换了一个**安全的失败模式**。这种取舍没法用单一指标
     排名，只能写清楚让读者自己判断 —— 而 §9.8 之所以强调「指标不能只有一个」，
     就是这个意思。
""" % (d3_prev, d3p_pct, float(rows[-3][4]), d3_last, 100 * d3_last / 0.1125,
       100 * d3_last / 0.1125, d4_last, e4_last, rows[-1][2][:-1]))

    # ---- 5.5 加速度计零偏 ----
    sub("5.6  失效点五：加速度计零偏与共模 ĉ 严格共线")
    print("""
  垂直方程是 (a_z,meas + g)·m̄/c̄ − U = −Σδ_i·u_i − ε·U。
  而 a_z,meas = a_z,true + b（b 为常值零偏）。代入：

      (a_z,true + b + g)·m̄/c̄ − U = [(a_z,true+g)·m̄/c̄ − U] + b·m̄/c̄

  右边多了一个**常数项 b·m̄/c̄**，而悬停时 U 近似为常数，
  所以它**与 ĉ 的那一列 −U 完全共线**。

  解析预测（无故障、闭环已配平到 a_z,true = 0、U = g·m̄/c̄ = 4Th）：
      y1 = b·m̄/c̄ = b/g1， 模型 y1 = −Σd_i·u_i − ĉ·U ≈ −ĉ·(4Th)
      ⇒ ĉ̂ = −(b/g1)/(4Th) = **−b/g**

  实测（无故障基线；零偏只加给**估计器**，闭环控制器看不到它）：
""" % ())
    rows = []
    for b in (0.02, 0.05, 0.10, 0.20):
        cs = []
        for seed in (1, 2, 3):
            (dh, ch), _, _ = ident_run(None, seed=seed, az_bias=b)
            cs.append(ch)
        ch = float(np.median(cs))
        rows.append(["%.2f m/s²（%.2f%% of g）" % (b, 100 * b / G),
                     "%+.4f" % (-b / G), "%+.4f" % ch,
                     "%.4f" % abs(ch + b / G),
                     "%+.2f%%" % (-100 * b / G)])
    table(["加速度计零偏 b", "解析预测 ĉ̂", "实测 ĉ̂", "偏差",
           "被读成「整机变轻」"], rows)
    worst = max(float(r[3]) for r in rows)
    print("""
  ⇒ **斜率逐点吻合**：把常数偏置 %.4f 从实测 ĉ̂ 里扣掉之后，
     预测与实测的差 < 0.0006。判据就在「偏差」列里 —— **它恒等于
     同一个数**，这说明误差**不是斜率误差，而是常数平移**。
     ⚠️ 那个常数平移本身就是 5.3(b) 说的共模通道自偏置（无故障时
        ĉ̂ ≈ +0.01）。它再次提醒：**ĉ 的绝对值不可信，只有它的
        变化量可信** —— 好在 (5.3) 描述的是变化量（斜率），所以可用。

  ⚠️ **注意符号：正的 a_z 零偏让估计器以为「飞机变轻了」。**
     直觉：加速度计读数偏大 → 估计器以为「同样的油门产生了更大的加速度」
     → 以为推力还有余量 → 以为飞机轻了。
     注意这也意味着 **「加速度计零偏」和「电机整体变强」分不开** ——
     又一次落进零空间方向 (1,1,1,1,−1) 里。

  ⇒ **失效条件五：加速度计零偏必须预先标定到远小于你要检测的量。**
     0.05 m/s² 的零偏（才 0.5%% g，出厂规格之内）会把 ĉ̂ 平移 0.0051。
     听起来很小，但对照一下**信号**：15%% 的单电机故障在共模通道上
     只有 ĉ 真值 0.0375（10%% 的只有 0.025）—— **零偏一口吃掉 14%%~20%%。**
     **零偏和信号同阶，这就是它必须离线标定掉的原因。**

  ⇒ 工程做法：先做六面法/转台标定把 b 压到 0.02 m/s² 以下，
     再让在线辨识跑。这与本教程
     `卷 2 第 10 章` 讲的「按设备规格书反推 R」
     是一条脉络：
       **08 课处理量测的随机部分（R），这里处理系统部分（零偏），
         两者都得先做完，在线辨识才有意义。**
     ★ 注意：PX4 的 ECL/EKF2 本身也在估加速度计零偏 ——
       两个估计器会沿着各自不可辨识的方向互相污染。
       真机上必须明确「谁负责哪一部分」，或者共用同一个零偏状态。

  ★ 顺带说尺度误差 s：它乘的是 a_z,true，而悬停时 a_z,true ≈ 0，
    所以**悬停时尺度误差不构成威胁，大机动时才致命**。
    这和实验 2 的结论连起来看很有意思：
    机动给了辨识所必需的激励，同时也把尺度误差放了进来。
    **没有免费的午餐。**
""" % (worst,))

    # ---- 5.6 推力曲线 ----
    sub("5.7  失效点六：效率与推力曲线是又一个「只能靠激励分开」的不可辨识对")
    fac = 0.4
    print("""
  整条推导用的是悬停点线性化，等价于「推力 ∝ 油门」。
  PX4 自己知道这不是真的 —— THR_MDL_FAC 把推力模型写成
      rel_thrust = fac·u² + (1−fac)·u        （motor_params.c:73 附近）

  ⚠️ 那么：**效率偏差 δ 和推力曲线参数 fac，能不能在同一批数据里分开？**
     这是「可辨识性」这条线的最后一个例子，答案和实验 2 一模一样。

  把观测写成对数形式（在油门 u 处，维持悬停所需的推力比例）：
      ln y(u) = ln(1−δ) + ln[ f(u)/u ]，   f(u) = fac·u² + (1−fac)·u

  两条回归量在几个工作点上的取值：""" % ())
    for u in (0.46, 0.60, 0.90):
        g_u = (u * u - u) / (fac * u * u + (1 - fac) * u)
        print("      u = %.2f：  ∂/∂δ = %+.3f（常数，与 u 无关）"
              "    ∂/∂fac = %+.4f" % (u, -1.0, g_u))
    print("""
  ★ 看两列的区别：∂/∂fac 随 u 变化，∂/∂δ 恒定。**在单个工作点上两者成比例**
    （比例系数就是 g(u)），Fisher 信息矩阵秩 1 —— 严格不可辨识。
    只有**跨工作点**（也就是机动）才能把它们分开。

  定量（σ = 0.01 的对数观测噪声，CRLB = σ·sqrt(F⁻¹)）：""" % ())
    rows = []
    for nm, us in [("只有悬停（1 个点）", [0.46]),
                   ("小机动（2 个点）", [0.46, 0.55]),
                   ("中机动（3 个点）", [0.40, 0.55, 0.70]),
                   ("大机动（5 个点）", [0.30, 0.40, 0.55, 0.70, 0.90])]:
        F = np.zeros((2, 2))
        for u in us:
            g_u = (u * u - u) / (fac * u * u + (1 - fac) * u)
            phi = np.array([-1.0, g_u])
            F += np.outer(phi, phi)
        ev = np.linalg.eigvalsh(F)
        if ev[0] <= 1e-12 * max(ev[1], 1e-30):
            cond, sd = float("inf"), float("inf")
        else:
            cond = float(ev[1] / ev[0])
            sd = float(0.01 * np.sqrt(np.linalg.inv(F)[0, 0]))
        rows.append([nm, "  ".join("%.2f" % u for u in us),
                     "∞" if np.isinf(cond) else "%.1f" % cond,
                     "∞" if np.isinf(sd) else "%.4f" % sd])
    table(["工况", "工作点 u", "cond(F)", "σ(δ̂) 下界（σ=0.01）"], rows)
    print("""
  ⇒ **失效条件六：只有当飞行剖面**跨越多个油门工作点**时，
     效率 δ 和推力曲线 fac 才分得开；纯悬停/纯平飞时它们严格共线。**
     实验 2 说「机动是唯一的信息来源」，这里换了一对参数，
     得到的还是同一句话。

  ★ 于是得到本章最实用的一条工程结论：

     **在线效率辨识必须和推力曲线在线标定（THR_MDL_FAC）一起做，
       否则两者会互相污染。**
     一个把 fac 设错（比如用 0 而真值是 0.4）的飞控，会把曲线误差
     全部记到「电机效率」账上 —— 而且因为它只在悬停附近工作，
     **它永远发现不了自己错了。**

     把这一条和前面五条放在一起看，会发现一个共同的形状：

        **PX4 现在的做法没有错。它在「只有垂直通道、只有悬停附近」
          这个信息约束下，把唯一能估的那个投影估准了。
          所有失效点都来自同一个根源 —— 信息不够，而不是代码写得不好。**

     ★ 这就是第 9 章 9.0 节的立场：**不要为了批评而批评。**
       指出失效条件的目的不是否定现有实现，而是把
       「需要什么额外信息、需要多少」这件事说清楚。
""" % ())

    # ---- 5.7 总结 ----
    sub("5.8  适用边界总结（这一节是全课最该背下来的部分）")
    print("""
  ┌──────────────────────────────────────────────────────────────────────┐
  │ 本改进方案**成立**的条件（必须同时满足）：                           │
  │  1. 故障是**差模**的：只有部分电机/桨退化（共模那一维谁也救不了）    │
  │  2. 偏航轴**真的被配平**：偏航环积分收敛、无侧风、桨叶无损伤 ——      │
  │     否则 d̂ 被偏航方程的系统性失配压低约 30%%（见 5.3(b)）           │
  │  3. 加速度计零偏已标定到 ≲ 0.02 m/s²（否则被读成「整机变轻」）       │
  │  4. 无持续外部力矩扰动（风、载荷偏心），或扰动已建模并扣除           │
  │  5. 有推力裕度：悬停油门足够低、全程无饱和（否则方案会「闭麦」）     │
  │  6. 推力曲线已标定，或飞行剖面跨越多个工作点（否则与 fac 共线）      │
  │ **不成立**（此时它不比 PX4 原版好，甚至更差）：                      │
  ├──────────────────────────────────────────────────────────────────────┤
  │  · 四个电机一起老化 / 整体效率下降 → 落在零空间里，d̂ 全报 0         │
  │  · 偏航长期不配平（桨损伤 / 侧风 / 偏航权限不足）→ d̂ 系统性低报 30%%│
  │  · 加速度计零偏未标定（≳ 0.05 m/s²）→ 平移 ĉ，吃掉信号的 1~2 成      │
  │  · 常值力矩 ≳ 0.05 N·m（风 / 电池偏心 / 线缆拖拽）→ 与差模故障等价   │
  │  · 悬停油门 ≳ 0.9 → A4 提前闭麦（真饱和占比 0%% 时就已闭嘴）         │
  │  · 只在悬停油门附近飞 → δ 与 THR_MDL_FAC 严格共线，估出来的是混合物  │
  └──────────────────────────────────────────────────────────────────────┘

  ⚠️ 注意这个写法和常见「创新论文」的区别：

     常见写法是「本方法在 XX 场景下优于基线，验证了有效性」。
     这种句子**没有信息量**，因为它不告诉你「什么时候不优于」。
     真正有用的结论必须长成上面这个形状：**一张成立条件的清单，
     加一张失效条件的清单，两张都要有定量门槛。**

  ★★ 这才是「从复现到改进」的完整样子：

     不是「我的方法全面胜出」，而是：
       **把原来 5 维全瞎的问题，变成 4 维可辨 + 1 维仍瞎；
         把那 1 维是什么、为什么瞎、什么时候会污染其余 4 维，
         全部写清楚。**

  ★ 给准备动手的你，最后三句话：

     1. **「提出改进」只占四步法里最短的一段。** 真正花时间的是第 2 步
        （注入缺陷）和第 4 步（验证）—— 因为只有那两步会告诉你，
        你刚才想到的那个漂亮点子在真机上根本不成立。
        §9.8 把这两步排在中间和最后，不是随便排的。

     2. **消融实验是防自欺的唯一工具。** 实验 4.2 里四个「改进点」中
        只有第一个真的有用，其余三个的贡献都在噪声里甚至为负。
        如果只做「全功能 vs 原版」，你会把功劳全记在最花哨的那一项上。

     3. **诚实的失效点比漂亮的指标值钱。** 上面六条失效点里，
        没有一条是「理论上可能」—— 每一条都有数字、有触发门槛、
        有实测或解析依据。§9.8 说「能诚实说出自己方法局限的人，
        才是真懂」—— 原因很实在：**只有真的算过、跑过、被数据打过脸的人，
          才知道自己的方法在哪里会断。**
""" % ())




# ===========================================================================
EXPERIMENTS = {
    "1": ("复现：把 zero_order_hover_thrust_ekf 翻译成 Python 并过它的单元测试", exp1),
    "2": ("可辨识性：悬停 vs 机动（Fisher 信息 / 条件数 / CRLB）", exp2),
    "3": ("注入缺陷：电机效率下降 / 整机变重 / 桨叶损伤", exp3),
    "4": ("改进方案：力矩配平方程 + 消融实验", exp4),
    "5": ("诚实报告：改进方案在什么条件下失效", exp5),
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
