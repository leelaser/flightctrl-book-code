#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 1 第 8 章 · 混控矩阵与电机模型 —— 配套实验

    实验 1 ★★★ 两条路走到同一个矩阵：PX4 的几何式 vs ArduPilot 的查表法
    实验 2 ★★★ 三种「逆」：方阵逆 / 伪逆 / ArduPilot 的正向混控
    实验 3 ★★★ 电机失效：正交性被破坏之后
    实验 4 ★★★ 饱和：ArduPilot 的 yaw_allowed vs PX4 的三档 Airmode
    实验 5 ★★  电机模型：推力曲线 / SPIN_MIN / 电压补偿

用法：
    python lesson09_mixer.py 1
    python lesson09_mixer.py 2
    ...
    python lesson09_mixer.py          # 全部

只需 numpy（画图需要 matplotlib）。

=======================================================================
参考源码
=======================================================================

ArduPilot Copter-4.5.7
    libraries/AP_Motors/AP_MotorsMatrix.cpp:502-527     add_motor_raw
    libraries/AP_Motors/AP_MotorsMatrix.cpp:536-544     ★ 角度 → 系数
    libraries/AP_Motors/AP_MotorsMatrix.h:79             add_motor_raw 默认参数
    libraries/AP_Motors/AP_MotorsMatrix.cpp:575-601     ★ QUAD PLUS / X
    libraries/AP_Motors/AP_MotorsMatrix.cpp:756-763     Y4（用 MotorDefRaw）
    libraries/AP_Motors/AP_MotorsMatrix.cpp:1307-1341   ★ normalise_rpy_factors
    libraries/AP_Motors/AP_MotorsMatrix.cpp:213-406     ★★ output_armed_stabilizing
    libraries/AP_Motors/AP_Motors_Class.h:200-206       AP_Motors_limit
    libraries/AP_Motors/AP_MotorsMulticopter.h:8-18     ★ 默认参数
    libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp:114-129  ★ 推力曲线
    libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:395-399  throttle_avg_max
    libraries/AC_AttitudeControl/AC_AttitudeControl.h:37,39            THR_MIX 默认值

PX4-Autopilot 1.15.4
    src/modules/control_allocator/ControlAllocation/ControlAllocation.hpp:87-94
                                                        ★ 6 个控制轴
    src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp:196-205
                                                        ★★ moment = p×a - km·a
    src/lib/matrix/matrix/PseudoInverse.hpp:18-63       ★★ Courrieu 快速伪逆
    src/lib/matrix/matrix/PseudoInverse.hpp:78-126      ★ 满秩 Cholesky
    src/modules/control_allocator/ControlAllocation/ControlAllocationPseudoInverse.cpp:73-140
                                                        ★ updateControlAllocationMatrixScale
    src/modules/control_allocator/ControlAllocation/ControlAllocationPseudoInverse.cpp:172-181
                                                        ★ allocate()
    src/modules/control_allocator/ControlAllocation/ControlAllocationSequentialDesaturation.cpp:67-228
                                                        ★★★ 三档 Airmode
    src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessMultirotor.hpp:52-55
                                                        normalize_rpy = true
    src/modules/control_allocator/module.yaml:209-223   CA_ROTORn_KM 符号约定
    src/modules/control_allocator/module.yaml:182-192   CA_ROTORn_AZ 默认 -1
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


def tab(rows, headers, fmt=None, indent="  "):
    """打印一张表。rows 是 list of list。"""
    cols = len(headers)
    if fmt is None:
        fmt = ["{}"] * cols
    widths = [len(h) for h in headers]
    for r in rows:
        for i, c in enumerate(r):
            widths[i] = max(widths[i], len(fmt[i].format(c)))
    print(indent + " | ".join(h.ljust(widths[i]) for i, h in enumerate(headers)))
    print(indent + "-+-".join("-" * w for w in widths))
    for r in rows:
        print(indent + " | ".join(fmt[i].format(c).ljust(widths[i])
                                  for i, c in enumerate(r)))


# ===========================================================================
# 公共：机型定义
# ===========================================================================
#
# 坐标约定（两家的约定【完全一致】，这一点很关键）：
#
#     机体坐标系 FRD（Forward-Right-Down）：
#         +x = 机头前方
#         +y = 机身右侧
#         +z = 机腹下方（向下！）
#
#     ArduPilot 的电机方位角 θ 从机头开始，向右侧增加
#         position = (cos θ, sin θ, 0) · L
#     PX4 的 CA_ROTOR n_PX/PY/PZ 直接就是 FRD 下的位置
#
#     螺旋桨转轴（向上推）在 FRD 里是 (0, 0, -1)
#
#     见 AP_MotorsMatrix.cpp:575-601（θ=0 的是 test order 1 = 机头）
#        ActuatorEffectivenessRotors.cpp:110-112（FixedUpwards = (0,0,-1)）

YAW_CCW = +1.0      # AP_MotorsMatrix.h:10-11  AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1
YAW_CW = -1.0       #                        AP_MOTORS_MATRIX_YAW_FACTOR_CW  = -1
#
# PX4 的约定（module.yaml:209-223）：
#     "Use a positive value for a rotor with CCW rotation."
#     "Use a negative value for a rotor with CW rotation."
# ⇒ 两家的符号约定【一致】。实验 1 会数值验证这一点。

ARM = 0.25          # 力臂，单位 m（250mm 轴距的四旋翼）

# (θ 角度, 旋向) —— 与 AP_MotorsMatrix.cpp 的电机表一一对应
#
# ★ 注意：ArduPilot 表里给的是【角度】，PX4 是【坐标】。
#   两者关系就是 position = (L cos θ, L sin θ, 0)。
FRAMES = {
    "QUAD_PLUS": [(90, YAW_CCW), (-90, YAW_CCW),
                  (0, YAW_CW), (180, YAW_CW)],
    "QUAD_X": [(45, YAW_CCW), (-135, YAW_CCW),
               (-45, YAW_CW), (135, YAW_CW)],
    "HEXA_X": [(30, YAW_CCW), (90, YAW_CW),
               (150, YAW_CCW), (-150, YAW_CW),
               (-90, YAW_CCW), (-30, YAW_CW)],
    # AP_MotorsMatrix.cpp:876-885  OCTA X（注意是 22.5°，不是 45°！）
    "OCTA_QUAD_X": [(22.5, YAW_CW), (-157.5, YAW_CW), (67.5, YAW_CCW), (157.5, YAW_CCW),
                    (-22.5, YAW_CCW), (-112.5, YAW_CCW), (-67.5, YAW_CW), (112.5, YAW_CW)],
}

# ★ ArduPilot 的六旋翼 X 有【三个变体】(AP_MotorsMatrix.cpp:774-844)：
#     MOTOR_FRAME_TYPE_X      :792-800   角度 90/-90/-30/150/30/-150
#     MOTOR_FRAME_TYPE_DJI_X  :819-831   角度 30/-30/-90/-150/150/90
#     MOTOR_FRAME_TYPE_CW_X   :832-844   角度 30/90/150/-150/-90/-30
#   三者电机排列（角度）完全一样，区别只在【编号顺序】和【旋向分配】。
#   本实验用最"正规"的 X 表，因为它是默认值。
FRAMES["HEXA_X"] = [(90, YAW_CW), (-90, YAW_CCW), (-30, YAW_CW),
                    (150, YAW_CCW), (30, YAW_CCW), (-150, YAW_CW)]

# ★ 有的机型 ArduPilot 不用角度，而是【直接给 roll/pitch 系数】(MotorDefRaw)，
#   见 AP_MotorsMatrix.cpp:889-901（OCTA V）。
#
#   由实验 1 的恒等式  roll = -y/L、pitch = +x/L
#   可以【反解】出这组系数对应的真实电机位置 —— 这些系数就是几何！
#
#   （系数表里的顺序是 7,3,6,4,8,2,1,5，这里按电机编号重排。）
RAW_FRAMES = {
    "OCTA_V": [
        # (x, y, 旋向)  —— 注意力臂长短不一，这是个 V 形不是圆环
        (0.34, -0.83, YAW_CW),
        (-0.32, 0.67, YAW_CW),
        (-0.32, -0.67, YAW_CCW),
        (-1.00, 0.50, YAW_CCW),
        (1.00, -1.00, YAW_CCW),
        (0.34, 0.83, YAW_CCW),
        (1.00, 1.00, YAW_CW),
        (-1.00, -0.50, YAW_CW),
    ],
}


def rotor_geometry(frame, arm=ARM):
    """返回 [(position(3,), axis(3,), yaw_sign), ...]（FRD 坐标）"""
    if frame in RAW_FRAMES:
        return [(np.array([x * arm, y * arm, 0.0]),
                 np.array([0.0, 0.0, -1.0]), yw)
                for x, y, yw in RAW_FRAMES[frame]]
    out = []
    for angle_deg, yaw_sign in FRAMES[frame]:
        th = math.radians(angle_deg)
        pos = np.array([arm * math.cos(th), arm * math.sin(th), 0.0])
        axis = np.array([0.0, 0.0, -1.0])
        out.append((pos, axis, yaw_sign))
    return out


def n_motors(frame):
    return len(RAW_FRAMES[frame]) if frame in RAW_FRAMES else len(FRAMES[frame])


# ===========================================================================
# 公共：ArduPilot 的混控表
# ===========================================================================

def ap_add_motor(angle_deg, yaw_factor):
    """★★★ 照抄 ArduPilot 的 add_motor。

    AP_MotorsMatrix.cpp:536-544
        add_motor_raw(
            motor_num,
            cosf(radians(roll_factor_in_degrees + 90)),   // = -sin(θ)
            cosf(radians(pitch_factor_in_degrees)),       // = +cos(θ)
            yaw_factor,
            testing_order);
    第六个参数 throttle_factor 走默认值 1.0（AP_MotorsMatrix.h:79）。
    """
    return (math.cos(math.radians(angle_deg + 90.0)),
            math.cos(math.radians(angle_deg)),
            float(yaw_factor),
            1.0)


def ap_mixer_raw(frame):
    """未归一化的 AP 混控表 M_ap，形状 (N, 4)，列 = [roll, pitch, yaw, throttle]"""
    if frame in RAW_FRAMES:
        # 由 (x, y) 反推：roll = -y, pitch = +x
        return np.array([(-y, x, yw, 1.0) for x, y, yw in RAW_FRAMES[frame]])
    return np.array([ap_add_motor(a, y) for a, y in FRAMES[frame]])


def ap_normalise(M):
    """★★ 照抄 AP_MotorsMatrix::normalise_rpy_factors。

    AP_MotorsMatrix.cpp:1307-1341
        「scale factors back to -0.5 to +0.5 for each axis」
        roll/pitch/yaw 每一列按【最大绝对值】缩放到 0.5；
        throttle 列按最大值缩放到 1.0，且负数截为 0。
    """
    M = M.copy()
    for col in range(3):
        f = np.max(np.abs(M[:, col]))
        if f != 0.0:
            M[:, col] = 0.5 * M[:, col] / f
    tf = np.max(np.maximum(0.0, M[:, 3]))
    if tf != 0.0:
        M[:, 3] = np.maximum(0.0, M[:, 3]) / tf
    return M


# ===========================================================================
# 公共：PX4 的效率矩阵
# ===========================================================================

CT = 1.0            # 推力系数（归一化掉，反正会被消元）
KM = 0.05           # PX4 的 CA_ROTORn_KM 默认值（module.yaml:223）


def px4_effectiveness(frame, arm=ARM, ct=CT, km=KM, up_positive=False):
    """★★★ 照抄 PX4 的效率矩阵构造。

    ActuatorEffectivenessRotors.cpp:196-205
        matrix::Vector3f thrust = ct * axis;
        matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;
        for (size_t j = 0; j < 3; j++) {
            effectiveness(j,     i) = moment(j);     // 力矩
            effectiveness(j + 3, i) = thrust(j);     // 力
        }

    返回 (4, N) 的矩阵，行 = [τ_roll, τ_pitch, τ_yaw, F_z]。

    ★★ 注意 F_z 的符号：向上的螺旋桨在 FRD 里 axis = (0,0,-1)，
       所以 thrust_z = ct · (-1) = **-ct**。这是 PX4 的真实约定。

    ★★ 这个符号【不是无关紧要的】：PX4 把效率矩阵的推力行用作
       去饱和方向（ControlAllocationSequentialDesaturation.cpp:196
       `desaturateActuators(_actuator_sp, thrust_z, true)`，
       注释写的是 "only reduce thrust"）。符号反了，
       `increase_only` 的语义就正好反过来（作者踩过一次，见 doc §4.4.6）。

    up_positive=True 时把推力行翻成 +ct，只为让**实验 2/3 的读数好看**；
    它不改变任何正交性 / 秩 / 伪逆结论（只是给一行乘 -1）。
    凡是用到去饱和语义的地方（实验 4）必须用默认的 up_positive=False。
    """
    B = np.zeros((4, n_motors(frame)))
    for i, (pos, axis, yaw_sign) in enumerate(rotor_geometry(frame, arm)):
        # ★ 转轴方向的单位化（源码 :162-170）
        axis = axis / np.linalg.norm(axis)
        # ★★★ 力矩 —— 一行代码概括了「混控矩阵从哪来」
        moment = ct * np.cross(pos, axis) - ct * (km * yaw_sign) * axis
        B[0:3, i] = moment
        B[3, i] = -ct                      # ← thrust_z，见 docstring
    if up_positive:
        B[3, :] = -B[3, :]
    return B


def up_of(w):
    """把 [τr, τp, τy, F_z] 里的 F_z 换成「向上的推力」。"""
    w = np.asarray(w, dtype=float)
    out = w.copy()
    out[3] = -w[3]
    return out


def px4_geninv(G):
    """★★★ 照抄 PX4 的 matrix::geninv（Courrieu 2008 快速伪逆）。

    PseudoInverse.hpp:18-63 —— 基于【满秩 Cholesky 分解】加速的
    Moore-Penrose 伪逆。比 SVD 快很多，代价是数值稳定性略差。

    返回 (伪逆, 秩)。
    """
    G = np.asarray(G, dtype=float)
    M, N = G.shape
    if M <= N:
        A = G @ G.T
        L, rank = _full_rank_cholesky(A)
        AtL = L.T @ L
        X = _inv_leading_block(AtL, rank)
        A2 = X @ X @ L.T
        res = G.T @ (L @ A2)
    else:
        A = G.T @ G
        L, rank = _full_rank_cholesky(A)
        AtL = L.T @ L
        X = _inv_leading_block(AtL, rank)
        A2 = X @ X @ L.T
        res = (L @ A2) @ G.T
    return res, rank


def _full_rank_cholesky(A):
    """照抄 PseudoInverse.hpp:78-126 的 fullRankCholesky。

    和普通 Cholesky 的区别：遇到「对角线不够大」的列就【跳过】，
    而不是开根号 —— 这样奇异矩阵也能走完，顺便数出秩。
    """
    n = A.shape[0]
    # PseudoInverse.hpp:83  const Type tol = N * typeEpsilon<Type>() * A.diag().max();
    tol = n * np.finfo(np.float32).eps * np.max(np.abs(np.diag(A)))
    L = np.zeros((n, n))
    r = 0
    for k in range(n):
        if r == 0:
            L[k:, r] = A[k:, k]
        else:
            LL = L[k:, :r] @ L[k, :r]
            L[k:, r] = A[k:, k] - LL
        if L[k, r] > tol:
            L[k, r] = math.sqrt(L[k, r])
            if k < n - 1:
                L[k + 1:, r] = L[k + 1:, r] / L[k, r]
            r += 1
    return L, r


def _inv_leading_block(A, rank):
    """对应 PseudoInverse.hpp:36 的 inv(A, X, rank) —— 只对前 rank×rank 求逆。

    SquareMatrix.hpp:378-423：LU 分解的循环上界是 rank 而不是 M，
    所以 A 的后半部分（全 0）根本不参与运算，得到的 X 也是分块对角。
    """
    n = A.shape[0]
    X = np.zeros((n, n))
    if rank > 0:
        X[:rank, :rank] = np.linalg.inv(A[:rank, :rank])
    return X


def px4_normalise_mix(mix, control_allocation_scale_in=None, normalize_rpy=True):
    """★★ 照抄 PseudoInverse.hpp 的 updateControlAllocationMatrixScale +
           normalizeControlAllocationMatrix。

    ControlAllocationPseudoInverse.cpp:73-140
        roll/pitch：scale = sqrt(‖col‖² / (n_nonzero/2))，两者取 max
        yaw       ：scale = col.max()
        thrust xyz：scale = mean(|col|)，逐轴回退到 Z 轴的值
    :143-169 再逐列除掉 scale，并把 |·|<1e-3 的元素清零。
    """
    mix = mix.copy()
    n_col = mix.shape[1]
    scale = np.ones(6)

    if normalize_rpy:
        n_roll = int(np.sum(np.abs(mix[:, 0]) > 1e-3))
        n_pitch = int(np.sum(np.abs(mix[:, 1]) > 1e-3))
        s_roll = math.sqrt(np.sum(mix[:, 0] ** 2) / (n_roll / 2.0)) if n_roll else 1.0
        s_pitch = math.sqrt(np.sum(mix[:, 1] ** 2) / (n_pitch / 2.0)) if n_pitch else 1.0
        scale[0] = max(s_roll, s_pitch)
        scale[1] = scale[0]
        scale[2] = mix[:, 2].max()

    scale[5] = 1.0
    for axis_idx in (2, 1, 0):          # 源码 :120  for (axis_idx = 2; axis_idx >= 0; --axis_idx)
        col = 3 + axis_idx
        if col >= n_col:
            continue
        norms = np.abs(mix[:, col])
        n_nonzero = int(np.sum(norms > np.finfo(np.float32).eps))
        scale[col] = norms.sum() / n_nonzero if n_nonzero else scale[5]

    if scale[0] > np.finfo(np.float32).eps:
        mix[:, 0] /= scale[0]
        mix[:, 1] /= scale[1]
    if n_col > 2 and scale[2] > np.finfo(np.float32).eps:
        mix[:, 2] /= scale[2]
    if n_col > 3 and scale[3] > np.finfo(np.float32).eps:
        for c in range(3, n_col):
            mix[:, c] /= scale[c]

    mix[np.abs(mix) < 1e-3] = 0.0
    return mix, scale


# ===========================================================================
# 公共：ArduPilot 的 output_armed_stabilizing（完整复现）
# ===========================================================================

class APLimits(object):
    def __init__(self):
        self.roll = False
        self.pitch = False
        self.yaw = False
        self.throttle_lower = False
        self.throttle_upper = False

    def __repr__(self):
        s = [k for k, v in self.__dict__.items() if v]
        return "{" + ",".join(s) + "}" if s else "{}"


YAW_HEADROOM = 200          # AP_MotorsMulticopter.h:8   AP_MOTORS_YAW_HEADROOM_DEFAULT
THRUST_MAX = 1.0            # 正常电池、不限流时的 _throttle_thrust_max
THST_HOVER = 0.35           # AP_MotorsMulticopter.h:10  AP_MOTORS_THST_HOVER_DEFAULT
THR_MIX_MIN = 0.10          # AC_AttitudeControl.h:37    AC_ATTITUDE_CONTROL_MIN_DEFAULT
THR_MIX_MAX = 0.50          # AC_AttitudeControl.h:39    AC_ATTITUDE_CONTROL_MAX_DEFAULT


def ap_throttle_avg_max(throttle_in, rpy_mix=THR_MIX_MIN, hover=THST_HOVER):
    """★★ 照抄 AC_AttitudeControl_Multi::get_throttle_avg_max。

    AC_AttitudeControl_Multi.cpp:395-399
        throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
        return MAX(throttle_in,
                   throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix)
                   + _motors.get_throttle_hover() * _throttle_rpy_mix);

    它回答的是：「**平均推力**最多能抬到多少？」

    ★ 关键在 MAX 的第二项：当飞行员给低油门时，这一项混入【悬停油门】，
      让分配器可以「悄悄抬高平均推力」到一个仍不足以让飞机上升的水平 ——
      这样就有余量给姿态控制用了。这就是注释里说的
      「Situation #2b allows us to raise the throttle above what the pilot
        commanded but not so far that it would actually cause the copter to rise」
                        —— AP_MotorsMatrix.cpp:259

    _throttle_rpy_mix 在 [_thr_mix_min, _thr_mix_max] 之间滑动
    （低油门→min=0.1，高油门→max=0.5，见 AC_AttitudeControl_Multi.cpp:366-370）。
    本实验取低油门端的 0.1，是姿态控制最「抠门」的工况。
    """
    thr = min(max(throttle_in, 0.0), 1.0)
    return max(thr, thr * max(0.0, 1.0 - rpy_mix) + hover * rpy_mix)


def ap_output_armed_stabilizing(M, roll_in, pitch_in, yaw_in, throttle_in,
                                throttle_avg_max, yaw_headroom=YAW_HEADROOM,
                                throttle_thrust_max=THRUST_MAX,
                                motor_enabled=None):
    """★★★ 逐行照抄 AP_MotorsMatrix::output_armed_stabilizing。

    AP_MotorsMatrix.cpp:213-406。行号标在注释里。
    M 是【已归一化】的混控表 (N,4)，列 = [roll, pitch, yaw, throttle]。
    返回 (motor_out, limits, yaw_used, rpy_scale)。
    """
    N = M.shape[0]
    if motor_enabled is None:
        motor_enabled = [True] * N
    rf, pf, yf, tf = M[:, 0], M[:, 1], M[:, 2], M[:, 3]
    limit = APLimits()

    # :216-234 电压/气压补偿增益 = 1（本实验不涉及）
    roll_thrust = roll_in
    pitch_thrust = pitch_in
    yaw_thrust = yaw_in
    throttle_thrust = throttle_in
    throttle_avg_max = min(max(throttle_avg_max, 0.0), 1.0)

    # :237-244 sanity check
    if throttle_thrust <= 0.0:
        throttle_thrust = 0.0
        limit.throttle_lower = True
    if throttle_thrust >= throttle_thrust_max:
        throttle_thrust = throttle_thrust_max
        limit.throttle_upper = True

    # :247 让 throttle_avg_max 落在 [throttle_thrust, throttle_thrust_max]
    throttle_avg_max = min(max(throttle_avg_max, throttle_thrust), throttle_thrust_max)

    # :251 ★ 给 rpy 留出最大控制范围的油门
    throttle_thrust_best_rpy = min(0.5, throttle_avg_max)

    # :276-298 计算能塞进多少 yaw
    out = np.zeros(N)
    yaw_allowed = 1.0
    for i in range(N):
        if not motor_enabled[i]:
            continue
        out[i] = roll_thrust * rf[i] + pitch_thrust * pf[i]
        if yf[i] != 0.0:
            thrust_rp_best_throttle = throttle_thrust_best_rpy + out[i]
            if yaw_thrust * yf[i] > 0.0:
                motor_room = 1.0 - thrust_rp_best_throttle
            else:
                motor_room = thrust_rp_best_throttle
            motor_yaw_allowed = max(motor_room, 0.0) / abs(yf[i])
            yaw_allowed = min(yaw_allowed, motor_yaw_allowed)

    # :302-308
    yaw_allowed_min = yaw_headroom * 0.001
    yaw_allowed = max(yaw_allowed, yaw_allowed_min)

    # :327-331
    if abs(yaw_thrust) > yaw_allowed:
        yaw_thrust = min(max(yaw_thrust, -yaw_allowed), yaw_allowed)
        limit.yaw = True

    # :334-350 把 yaw 加进去，同时找最高/最低
    rpy_low = 1.0
    rpy_high = -1.0
    for i in range(N):
        if not motor_enabled[i]:
            continue
        out[i] = out[i] + yaw_thrust * yf[i]
        rpy_low = min(rpy_low, out[i])
        rpy_high = max(rpy_high, out[i])

    # :359-366 缩放
    rpy_scale = 1.0
    if rpy_high - rpy_low > 1.0:
        rpy_scale = 1.0 / (rpy_high - rpy_low)
    if throttle_avg_max + rpy_low < 0.0:
        rpy_scale = min(rpy_scale, -throttle_avg_max / rpy_low)

    # :368-390
    rpy_high *= rpy_scale
    rpy_low *= rpy_scale
    throttle_thrust_best_rpy = -rpy_low          # ★ 把 rpy 带【重新居中】
    thr_adj = throttle_thrust - throttle_thrust_best_rpy
    if rpy_scale < 1.0:
        limit.roll = True
        limit.pitch = True
        limit.yaw = True
        if thr_adj > 0.0:
            limit.throttle_upper = True
        thr_adj = 0.0
    elif thr_adj < 0.0:
        thr_adj = 0.0
    elif thr_adj > 1.0 - (throttle_thrust_best_rpy + rpy_high):
        thr_adj = 1.0 - (throttle_thrust_best_rpy + rpy_high)
        limit.throttle_upper = True

    # :392-398
    best_plus_adj = throttle_thrust_best_rpy + thr_adj
    for i in range(N):
        if motor_enabled[i]:
            out[i] = best_plus_adj * tf[i] + rpy_scale * out[i]
    return out, limit, yaw_thrust, rpy_scale


# ===========================================================================
# 公共：PX4 的顺序去饱和（三档 Airmode）
# ===========================================================================

class PX4Allocator(object):
    """★★★ 照抄 ControlAllocationSequentialDesaturation。

    ControlAllocationSequentialDesaturation.cpp:44-234
        allocate() 按 MC_AIRMODE 分派：
            0 (默认) mixAirmodeDisabled
            1        mixAirmodeRP
            2        mixAirmodeRPY
    """

    def __init__(self, B, actuator_min=0.0, actuator_max=1.0,
                 control_trim=None, actuator_trim=None):
        self.B = np.asarray(B, dtype=float)
        self.n = self.B.shape[1]
        mix, self.scale = px4_geninv(self.B)
        mix, self.scale = px4_normalise_mix(mix, normalize_rpy=True)
        self.mix = mix                                   # (N, 4) —— 只取 4 个用到的轴
        self.amin = np.full(self.n, float(actuator_min))
        self.amax = np.full(self.n, float(actuator_max))
        self.trim = (np.zeros(self.n) if actuator_trim is None
                     else np.asarray(actuator_trim, dtype=float))
        self.ctrl_trim = (np.zeros(4) if control_trim is None
                          else np.asarray(control_trim, dtype=float))
        self.ctrl_sp = np.zeros(4)

    # ---- ControlAllocationPseudoInverse.cpp:172-181 ----------------------
    def allocate_pseudo_inverse(self):
        """最朴素的伪逆分配：u = trim + mix·(c - c_trim)，然后硬截断。"""
        u = self.trim + self.mix @ (self.ctrl_sp - self.ctrl_trim)
        return np.clip(u, self.amin, self.amax)

    # ---- ControlAllocationSequentialDesaturation.cpp:88-119 --------------
    @staticmethod
    def _compute_desaturation_gain(desat, u, amin, amax):
        """★★ 求「沿 desat 方向平移多少」能把最严重的饱和消掉。

        源码 :96 有一个很容易被忽略的细节：
            if (fabsf(desaturation_vector(i)) < 0.2f) continue;
        —— 效率太低的执行器不参与去饱和，否则增益会巨大。
        """
        k_min, k_max = 0.0, 0.0
        for i in range(len(u)):
            if abs(desat[i]) < 0.2:
                continue
            if u[i] < amin[i]:
                k = (amin[i] - u[i]) / desat[i]
                k_min = min(k_min, k)
                k_max = max(k_max, k)
            if u[i] > amax[i]:
                k = (amax[i] - u[i]) / desat[i]
                k_min = min(k_min, k)
                k_max = max(k_max, k)
        return k_min + k_max

    def _desaturate(self, u, desat, increase_only=False):
        """源码 :67-86 —— 注意【做两遍】：一次全增益，一次半增益。"""
        gain = self._compute_desaturation_gain(desat, u, self.amin, self.amax)
        if increase_only and gain < 0.0:
            return u, 0.0
        u = u + gain * desat
        gain2 = 0.5 * self._compute_desaturation_gain(desat, u, self.amin, self.amax)
        u = u + gain2 * desat
        return u, gain

    # ---- 三条分配路径 ----------------------------------------------------
    def mix_airmode_disabled(self):
        """:173-204 —— 永不靠【增加】推力来去饱和，宁可牺牲 rpy。"""
        u = self.trim + self.mix @ (self.ctrl_sp - self.ctrl_trim)
        thrust_z = self.mix[:, 3].copy()
        roll = self.mix[:, 0].copy()
        pitch = self.mix[:, 1].copy()
        u, _ = self._desaturate(u, thrust_z, increase_only=True)
        u, _ = self._desaturate(u, roll)
        u, _ = self._desaturate(u, pitch)
        return self._mix_yaw(u)

    def mix_airmode_rp(self):
        """:121-143 —— 允许增减推力来保住 roll/pitch，但 yaw 不行。"""
        u = (self.trim
             + self.mix[:, 0] * (self.ctrl_sp[0] - self.ctrl_trim[0])
             + self.mix[:, 1] * (self.ctrl_sp[1] - self.ctrl_trim[1])
             + self.mix[:, 3] * (self.ctrl_sp[3] - self.ctrl_trim[3]))
        u, _ = self._desaturate(u, self.mix[:, 3].copy())
        return self._mix_yaw(u)

    def mix_airmode_rpy(self):
        """:145-171 —— 全 airmode：先保 roll/pitch，再把 yaw 去饱和。"""
        u = self.trim + self.mix @ (self.ctrl_sp - self.ctrl_trim)
        u, _ = self._desaturate(u, self.mix[:, 3].copy())
        u, _ = self._desaturate(u, self.mix[:, 2].copy())
        return u

    def _mix_yaw(self, u):
        """:206-228 mixYaw。

        ★ 这里有一个非常「反直觉」的技巧（源码 :221-224）：
            把 _actuator_max 临时【抬高 15%】再去做 yaw 去饱和，
            做完再恢复。
        ⇒ 效果是「满油门时还能挤出一部分 yaw 权限」，
          代价是那 15% 的电机指令会超过正常上限。
        """
        u = u + self.mix[:, 2] * (self.ctrl_sp[2] - self.ctrl_trim[2])
        amax_backup = self.amax.copy()
        self.amax = self.amax + (self.amax - self.amin) * 0.15
        u, _ = self._desaturate(u, self.mix[:, 2].copy())
        self.amax = amax_backup
        u, _ = self._desaturate(u, self.mix[:, 3].copy(), increase_only=True)
        return u

    def allocate(self, airmode):
        if airmode == 1:
            return self.mix_airmode_rp()
        if airmode == 2:
            return self.mix_airmode_rpy()
        return self.mix_airmode_disabled()

    # ---- 工具：给定电机指令，反算实际产生的力矩 --------------------------
    def achieved(self, u):
        return self.B @ (u - self.trim)


# ===========================================================================
# 公共：ArduPilot 的推力曲线
# ===========================================================================

def thrust_curve(thrust, expo, lift_max=1.0, battery_scale=1.0):
    """★★ 照抄 Thrust_Linearization::apply_thrust_curve_and_volt_scaling。

    AP_Motors_Thrust_Linearization.cpp:114-129
        throttle_ratio =
            ((expo - 1) + sqrt((1-expo)² + 4·expo·lift_max·thrust)) / (2·expo)
        return throttle_ratio * battery_scale

    它其实是下面这个二次方程的正根解：
        lift_max · thrust = (1 - expo) · t + expo · t²
    即「推力 = 线性项 + 二次项」的混合。
        expo = 0  → 推力 = t          （线性）
        expo = 1  → 推力 = t²         （纯二次，物理上最常见）
        expo = 0.65（默认）→ 两者混合
    """
    expo = min(max(expo, -1.0), 1.0)
    if expo == 0.0:
        return lift_max * thrust * battery_scale
    inner = (1.0 - expo) ** 2 + 4.0 * expo * lift_max * thrust
    t = ((expo - 1.0) + math.sqrt(max(inner, 0.0))) / (2.0 * expo)
    return min(max(t * battery_scale, 0.0), 1.0)


def thrust_curve_inverse(throttle, expo, lift_max=1.0):
    """正向曲线的反函数（源码 :133- 的 remove_thrust_curve_and_volt_scaling）。

    直接由 lift_max · thrust = (1-expo)·t + expo·t² 解出 thrust。
    """
    expo = min(max(expo, -1.0), 1.0)
    return ((1.0 - expo) * throttle + expo * throttle * throttle) / lift_max


def thrust_to_actuator(thrust, expo, spin_min=0.15, spin_max=0.95):
    """照抄 Thrust_Linearization::thrust_to_actuator（:100-104）。"""
    return spin_min + (spin_max - spin_min) * thrust_curve(thrust, expo)


# ===========================================================================
# 实验 1：两条路走到同一个矩阵
# ===========================================================================

def exp1():
    head("实验 1  两条路走到同一个矩阵：PX4 的几何式 vs ArduPilot 的查表法")

    print("""
  ArduPilot 把混控矩阵【手工列成表】（AP_MotorsMatrix.cpp:575-601）：
      QUAD X: {45, CCW}, {-135, CCW}, {-45, CW}, {135, CW}
      然后 add_motor 里换算（:536-544）：
          roll  = cos(θ + 90°)
          pitch = cos(θ)
          yaw   = ±1

  PX4 不列表，它从【三维几何】现算（ActuatorEffectivenessRotors.cpp:196-205）：
          thrust = ct · axis
          moment = ct · (position × axis) - ct · km · axis

  本实验要验证：这两条路得到的是【同一个矩阵】。
""")

    for frame in ("QUAD_X", "HEXA_X", "OCTA_QUAD_X"):
        sub("机型 " + frame)
        B = px4_effectiveness(frame, up_positive=True)
        M_raw = ap_mixer_raw(frame)
        n = M_raw.shape[0]

        # PX4 的 B 行是 [τ_roll, τ_pitch, τ_yaw, F_up]，单位是 N·m 和 N
        # AP 的表是【无量纲】的系数，两者相差一个【逐行标量】：
        #     B = diag(ARM, ARM, KM, CT) @ M_raw
        # 我们把这个标量除掉再比。
        B_unscaled = B.copy()
        B_unscaled[0] /= ARM
        B_unscaled[1] /= ARM
        B_unscaled[2] /= KM
        B_unscaled[3] /= CT
        B_unscaled = B_unscaled.T          # (4,N) → (N,4)

        rows = []
        for i in range(n):
            rows.append(["m%d" % i,
                         "%.4f" % M_raw[i, 0], "%.4f" % B_unscaled[i, 0],
                         "%.4f" % M_raw[i, 1], "%.4f" % B_unscaled[i, 1],
                         "%.2f" % M_raw[i, 2], "%.2f" % B_unscaled[i, 2]])
        tab(rows,
            ["电机", "AP roll", "PX4 roll", "AP pitch", "PX4 pitch", "AP yaw", "PX4 yaw"])

        err = np.max(np.abs(B_unscaled - M_raw))
        print("\n  最大绝对偏差 = %.3e   ⇒ %s"
              % (err, "✅ 同一个矩阵" if err < 1e-12 else "❌ 不一致！"))

        # ★ 为什么 AP 的 roll 系数是 cos(θ+90) 而 PX4 算出来是 -sin(θ)？
        print("\n  ★ 关键恒等式：cos(θ + 90°) ≡ -sin(θ)")
        th = math.radians(FRAMES[frame][0][0])
        print("      θ = %.1f°" % FRAMES[frame][0][0])
        print("      cos(θ+90) = %+.12f    AP 表里写的" % math.cos(th + math.pi / 2))
        print("      -sin(θ)   = %+.12f    PX4 的 p×a 算出来的" % (-math.sin(th)))
        print("""
  ⇒ 也就是说：**「按角度查表」和「叉乘算力矩」是同一件事**。
     AP 的 cos(θ+90) 本质就是 -sin θ，就是 p×a 的 x 分量除以力臂。
""")

    # ---- 归一化 ----
    sub("归一化：AP 的 normalise_rpy_factors 把每一列缩到 ±0.5")
    M_raw = ap_mixer_raw("QUAD_X")
    M_norm = ap_normalise(M_raw)
    rows = []
    for i in range(4):
        rows.append(["m%d" % i,
                     "%+.4f" % M_raw[i, 0], "%+.4f" % M_norm[i, 0],
                     "%+.4f" % M_raw[i, 1], "%+.4f" % M_norm[i, 1],
                     "%+.1f" % M_raw[i, 2], "%+.1f" % M_norm[i, 2]])
    tab(rows, ["电机", "roll 原", "roll 归一", "pitch 原", "pitch 归一",
               "yaw 原", "yaw 归一"])
    print("""
  ★ 归一化的意义（AP_MotorsMatrix.cpp:1305-1306 的注释）：
      「normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5」
    这是在说：**一个 1.0 的 roll 指令，最多只能把电机推离中心 0.5**。
    剩下的 0.5 就是留给油门和 yaw 的裕度 —— 见实验 4。
""")

    # ---- 正交性 ----
    sub("★★ 更深一层：这些系数是【互相正交】的")
    for frame in ("QUAD_X", "HEXA_X"):
        M = ap_normalise(ap_mixer_raw(frame))
        print("\n  机型 %s：Gram 矩阵 Mᵀ M / (N/2)" % frame)
        G = M[:, :3].T @ M[:, :3]
        G = G / (M.shape[0] / 2.0)
        for r in G:
            print("      [" + "  ".join("%+.6f" % v for v in r) + "]")
    print("""
  ⇒ 非对角项全是 0。**这正是 AP 那套手写系数能work的原因**：
     roll / pitch / yaw 三个方向「占用的电机模式」互不重叠，
     ⇒ 打 roll 不会产生 pitch 和 yaw 的串扰，三个通道可以独立设计。

  ★★ 而 PX4 的伪逆**只有在行正交时才等于 AP 的正向混控**（实验 2 会算）。
     所以「正交」不是巧合，是这个设计能成立的【前提】。
     实验 3 会演示：一旦正交性被破坏（比如掉一个电机），会出什么事。
""")


# ===========================================================================
# 实验 2：三种「逆」
# ===========================================================================

def exp2():
    head("实验 2  三种「逆」：方阵逆 / 伪逆 / ArduPilot 的正向混控")

    print("""
  问题：我想要力矩 w = (τ_roll, τ_pitch, τ_yaw, F_up)，
        每个电机该出多少推力 u？

      w = B · u        B 是 4×N（N 个电机）

  N = 4 时 B 是方阵，u = B⁻¹ w —— 唯一解。
  N > 4 时 B 是「扁」的，u 有无穷多解 —— 得挑一个。

  三家做法：
      numpy       B⁻¹（方阵）/ pinv（SVD 伪逆）
      PX4         geninv（Courrieu 快速伪逆，满秩 Cholesky）
      ArduPilot   不算逆！它直接把系数表【正向】乘上去
""")

    B = px4_effectiveness("QUAD_X", up_positive=True)
    B4 = B[[0, 1, 2, 3], :]                 # 四旋翼用的 4 个轴

    sub("① N=4（四旋翼）：B 是方阵，逆是唯一的")
    Binv = np.linalg.inv(B4)
    ginv, rank = px4_geninv(B4)
    print("  B 的形状 = %s，秩 = %d" % (B4.shape, rank))
    print("  ‖B⁻¹(numpy) − geninv(PX4)‖∞ = %.3e" % np.max(np.abs(Binv - ginv)))
    print("  ‖B · B⁻¹ − I‖∞              = %.3e" % np.max(np.abs(B4 @ Binv - np.eye(4))))
    print("  ⇒ 满秩方阵时，PX4 的快速伪逆就是普通逆。")

    sub("② ★★ ArduPilot 的正向混控 = 伪逆吗？")
    M_ap = ap_normalise(ap_mixer_raw("QUAD_X"))
    # AP 的系数是无量纲的；B 的行有物理单位。先看「方向」对不对。
    # 比较：B · M_ap 应该是对角阵（每个指令只影响自己的轴）
    prod = B4 @ M_ap
    print("  B · M_ap =")
    for r in prod:
        print("      [" + "  ".join("%+10.6f" % v for v in r) + "]")
    off = prod - np.diag(np.diag(prod))
    print("\n  非对角元最大绝对值 = %.3e" % np.max(np.abs(off)))
    print("""
  ⇒ **是对角阵！** 这一件事同时说明了两点：

     (a) AP 的混控是【正确的】—— 打 roll 只产生 roll 力矩；
     (b) AP 的系数表就是【伪逆】，只是按轴各自缩放了不同倍数：
             M_ap = B⁺ · diag(增益)
         因为  B·M_ap = diag  ⟺  M_ap = B⁺ · diag  （B 行满秩时成立）
""")

    # 数值验证 (b)
    gain = np.diag(prod)
    M_from_pinv = Binv @ np.diag(gain)
    print("  ‖M_ap − B⁻¹·diag(B·M_ap)‖∞ = %.3e"
          % np.max(np.abs(M_ap - M_from_pinv)))
    print("  对角增益 = [" + ", ".join("%.4f" % g for g in gain) + "]")
    print("""
  ★ 增益的具体数值也有物理意义：
      roll  : %.4f N·m  —— 单位 roll 指令产生多少力矩
      pitch : %.4f N·m
      yaw   : %.4f N·m
      F_up  : %.4f N    —— 单位 throttle 产生多少推力
""" % tuple(gain))
    print("""  ★ 注意 gain[0] 是怎么来的：四旋翼的 roll 力臂是
    【臂长投影到 y 轴】= L·sin45° = %.4f（不是 L 本身！）。
    4 个电机 × 投影力臂 × 归一化系数 0.5 = %.4f ← 就是 gain[0]。

  ⇒ 这就是 PX4 用 p×a 而不是查表的直接好处：
     **投影、力臂差异、旋向，全都在叉乘里自动算掉了。**
     ArduPilot 要为每个角度单独准备系数，PX4 不需要。
""" % (ARM * math.sin(math.radians(45)),
       4 * ARM * math.sin(math.radians(45)) * 0.5))

    sub("③ N=6（六旋翼）：结果出乎意料 —— 两者【一模一样】")
    B6 = px4_effectiveness("HEXA_X", up_positive=True)[[0, 1, 2, 3], :]
    g6, rank6 = px4_geninv(B6)
    M6 = ap_normalise(ap_mixer_raw("HEXA_X"))
    prod6 = B6 @ M6
    print("  B 的形状 = %s，秩 = %d  （4 行、6 列 ⇒ 行满秩、列不满）"
          % (B6.shape, rank6))

    # ★ 关键：AP 的表是【无量纲系数】，PX4 的 B 是【物理量】。
    #   要公平比较，必须让两者产生【同一个力矩 w】。
    #   B·M_ap = diag(gain)  ⇒  AP 要产生 w，指令应该是 x = w / gain。
    w = np.array([0.15, -0.10, 0.05, 2.00])     # 期望 (τr, τp, τy, F_up)
    gain6 = np.diag(prod6)
    x_ap = w / gain6
    u_ap = M6 @ x_ap                            # AP 路径
    u_px = g6 @ w                               # PX4 路径

    print("\n  期望力矩 w = [" + ", ".join("%+.4f" % v for v in w) + "]")
    print("  （τ 单位 N·m，F 单位 N；2 N ≈ 一架 200 g 四旋翼的悬停推力）")
    print("\n  AP 要达到这个力矩，控制指令是 x = w / gain =")
    print("      [" + ", ".join("%+.4f" % v for v in x_ap) + "]\n")

    rows = []
    for i in range(6):
        rows.append(["m%d" % i, "%+.9f" % u_ap[i], "%+.9f" % u_px[i],
                     "%.2e" % abs(u_ap[i] - u_px[i])])
    tab(rows, ["电机", "AP 正向混控", "PX4 伪逆", "|差|"])

    print("\n  最大差异 = %.3e" % np.max(np.abs(u_ap - u_px)))
    print("""
  ★★★ **两者完全相同（到浮点精度）。**

  这不是巧合，可以证明：

      实验 1 已经验证 B·M_ap = diag(gain)，即对角阵。
      换句话说：M_ap 的第 k 列与 B 的第 j 行（j≠k）正交。
      再注意 M_ap 的第 k 列 ∝ B 的第 k 行 —— 因为 B 的第 k 行
      就是「各电机的第 k 个系数」，而 M_ap 的第 k 列就是这些系数本身。
      ⇒ **M_ap 的每一列都落在 B 的【行空间】里**。

      而 Moore-Penrose 伪逆的定义正是：
          B⁺w = 「满足 Bu=w、且落在 B 行空间里」的那个唯一解。
      ⇒ 两者必然相等。

  ⇒ 所以「ArduPilot 的手工混控表」和「PX4 的伪逆」在
     **对称机型上是数学等价的**。两家写的是同一个东西，
     只是一个查表、一个现算。
""")

    sub("④ 那 AP 的表什么时候会出问题？")
    Bv = px4_effectiveness("OCTA_V", up_positive=True)[[0, 1, 2, 3], :]
    Mv = ap_normalise(ap_mixer_raw("OCTA_V"))
    prodv = Bv @ Mv
    print("  用 OCTA_V（ArduPilot 的 V 型八旋翼，系数是【手工给的】MotorDefRaw，")
    print("  见 AP_MotorsMatrix.cpp:889-901）再算一遍 B·M_ap：\n")
    for r in prodv:
        print("      [" + "  ".join("%+9.5f" % v for v in r) + "]")
    offv = prodv - np.diag(np.diag(prodv))
    print("\n  非对角元最大绝对值 = %.3e   ← 不是 0！" % np.max(np.abs(offv)))
    ii, jj = np.unravel_index(np.argmax(np.abs(offv)), offv.shape)
    names = ["roll", "pitch", "yaw", "throttle"]
    print("  最大的串扰：打 %s → 串出 %s 力矩，%+.5f" % (names[jj], names[ii], offv[ii, jj]))
    print("  相对对角元 %.4f 是 **%.1f%%** 的串扰"
          % (prodv[ii, ii], 100 * abs(offv[ii, jj] / prodv[ii, ii])))
    print("""
  ⇒ **在 V 型机上，AP 的手工系数和伪逆不再等价。**

  为什么？回顾实验 1 的正交性来源（两个条件各管一件事）：
    ① 【沿圆周均匀分布】  ⇒ 三个姿态和式为 0（三轴互不串扰）
    ② 【邻桨反旋向】Σσ=0 ⇒ yaw 行与 throttle 行不共线
      Σ rollᵢ·pitchᵢ = 0,   Σ rollᵢ·yawᵢ = 0,   Σ pitchᵢ·yawᵢ = 0
  V 型机连①都不满足（力臂长短不一 1.00 / 0.83 / 0.67 / 0.50，
  电机也不成对径分布），于是姿态三轴正交性失效 ——
  **而正交性正是「查表法」能成立的前提。**

  ★★ 那 PX4 呢？伪逆是从效率矩阵【现算】的，
     它**根本不需要正交性这个前提** —— B 长什么样就适应什么样。
     下面验证。
""")

    sub("⑤ 同一个 V 型机：给同样的指令，看实得力矩")
    gv, _ = px4_geninv(Bv)
    gainv = np.diag(prodv)
    print("  模拟「遥控器打杆」：控制指令 x = [roll, pitch, yaw, throttle]")
    print("  AP 直接吃 x；PX4 吃换算成物理力矩的 x·gain（两者等价输入）\n")
    rows = []
    for xr, xp, xy in ((0.4, 0.0, 0.0), (0.0, 0.4, 0.0), (0.0, 0.0, 0.4),
                       (0.3, 0.3, 0.0), (0.3, 0.0, 0.3), (0.0, 0.3, 0.3)):
        x = np.array([xr, xp, xy, 0.5])
        wa = Bv @ (Mv @ x)
        wp = Bv @ (gv @ (x * gainv))
        rows.append(["%.1f/%.1f/%.1f" % (xr, xp, xy),
                     "%+.3f" % wa[0], "%+.3f" % wa[1], "%+.3f" % wa[2],
                     "%+.3f" % wp[0], "%+.3f" % wp[1], "%+.3f" % wp[2]])
    tab(rows, ["指令 r/p/y", "AP→roll", "AP→pitch", "AP→yaw",
               "PX4→roll", "PX4→pitch", "PX4→yaw"])
    # 给下面这段话算真实数字（不写死）
    x_r = np.array([0.4, 0.0, 0.0, 0.5])
    wa_r = Bv @ (Mv @ x_r)
    wp_r = Bv @ (gv @ (x_r * gainv))
    print("""
  ★ 看第 1 行：给**纯 roll** 指令（0.4/0/0），
     AP  实得 roll = %+.4f，同时串出 pitch = %+.4f、yaw = %+.4f；
     PX4 实得 roll = %+.4f，pitch = %+.4f、yaw = %+.4f —— 严格不串。
  ⇒ 因为伪逆的定义就是 Bu = w **精确成立**（B 行满秩时）。
     反过来说，AP 那 14%% 的 yaw→roll 串扰会一直存在，
     直到它专门为 V 型机再手调一组系数。
""" % (wa_r[0], wa_r[1], wa_r[2], wp_r[0], wp_r[1], wp_r[2]))
    print("""  ★★ 但要说清楚：这不代表「AP 飞不了 V 型机」。
     AP 的 V 型机是能飞的 —— 因为姿态控制器是**闭环**的，
     串扰会当作扰动被 PID 修掉，只是多消耗一点控制余量。
     这属于「性能损失」，不是「功能性错误」。

  ⇒ **这就是查表法和现算法的本质差别**：
     AP 的表是「人工挑的、依赖对称性的近似」，
     PX4 的伪逆是「由 B 唯一决定的、不依赖对称性的精确解」。

  ★★ 而真正的差别在【可维护性】上：
      · ArduPilot 每加一个新机型，都得有人手算或试凑一组系数
        （OCTA_V 那 8 组 (x, y, σ) 就是这么来的）
      · PX4 只要填 CA_ROTORn_PX/PY/PZ/AX/AZ/CT/KM，矩阵自动生成

     这也是为什么 PX4 能支持倾斜旋翼、矢量推力这类构型，
     而 ArduPilot 的倾斜旋翼要走 `AP_Motors6DOF` /
     `thrust_compensation_callback` 这种「特事特办」的接口
     （见 AP_MotorsMatrix.cpp:1344-1354）。

  ★ 但 ArduPilot 这条路也有它的好处，而且不小：
     **系数是显式常量 ⇒ 可预测、可审计、可手调。**
     出问题时你能一眼把混控表打印出来看；
     PX4 出问题时你看到的是一个 6×N 的浮点矩阵。
     这是「工程可控性」与「通用性」之间一个很典型的取舍，
     没有哪一边是纯粹更好的。
""")


# ===========================================================================
# 实验 3：正交性被破坏之后
# ===========================================================================

def exp3():
    head("实验 3  电机失效：正交性被破坏之后")

    print("""
  实验 1 我们看到：AP 的系数表之所以能work，是因为三个方向的系数【正交】。
  实验 2 我们看到：正交 ⟺ AP 的正向混控 = 伪逆。

  现在问一个致命的问题：
      **如果掉了一个电机呢？**
""")

    frame = "HEXA_X"
    B = px4_effectiveness(frame, up_positive=True)[[0, 1, 2, 3], :]
    M_raw = ap_mixer_raw(frame)
    N = M_raw.shape[0]
    M_full = ap_normalise(M_raw)

    sub("① ArduPilot 做了什么")
    print("""  libraries/AP_Motors/AP_MotorsMatrix.cpp:546-558
      void AP_MotorsMatrix::remove_motor(int8_t motor_num) {
          motor_enabled[motor_num] = false;
          _roll_factor[motor_num]  = 0.0f;
          _pitch_factor[motor_num] = 0.0f;
          _yaw_factor[motor_num]   = 0.0f;
          _throttle_factor[motor_num] = 0.0f;
      }

  ⇒ **只是把这个电机的系数清零，其它 5 个电机的系数一个字都不改。**
     `normalise_rpy_factors()` 也【不会】重跑 ——
     它只在 setup_motors() 的最后调用一次。

  ★ 先说清楚一件事：AP 其实【有】掉电机的机制，
    就是 `MOT_THRUST_BOOST` / `_thrust_boost`（见 :263-272 那段长注释）。
    但它不是「重新分配」，只是把失效电机的输出【从限幅计算里摘出去】。
    下面先看它不做重新分配时会怎样，再来评价这个补丁。
""")

    # ★ 先扫一遍掉哪个电机最糟，免得「挑了个最惨的例子」不好交代
    names3 = ["roll", "pitch", "yaw"]
    sub("①b 先扫一遍：掉哪个电机最糟？")
    print("  对每个可能的失效电机，算「三轴子块的最大串扰」：\n")
    rows = []
    lost, worst_val = 0, -1.0
    for k in range(N):
        en = [i != k for i in range(N)]
        Mk = M_full.copy()
        Mk[k, :] = 0.0
        Pk = B[:, en] @ Mk[en]
        Sk = Pk[:3, :3] / np.diag(Pk)[:3, None]
        v = np.max(np.abs(Sk - np.eye(3)))
        if v > worst_val:
            lost, worst_val = k, v
        rows.append(["m%d" % k,
                     "%+.2f" % M_full[k, 0], "%+.2f" % M_full[k, 1],
                     "%+.1f" % M_full[k, 2], "%.1f%%" % (100 * v)])
    tab(rows, ["失效电机", "roll 系数", "pitch 系数", "yaw 系数", "最大串扰"])
    print("""
  ⇒ 掉【任何一个】都会产生串扰，最糟的是掉 m%d（%.0f%%）。
     下面就用掉 m%d 这个最坏情况继续分析。
""" % (lost, 100 * worst_val, lost))

    enabled = [i != lost for i in range(N)]
    M_red = M_full.copy()
    M_red[lost, :] = 0.0

    sub("② 正交性没了：Gram 矩阵出现非对角项")
    Ms_ok = M_full[:, :3]
    Ms_bad = M_red[enabled, :3]

    def gram(Ms):
        # 用【同一套】归一化因子，只比较形状
        return Ms.T @ Ms / (Ms.shape[0] / 2.0)

    G_ok = gram(Ms_ok)
    G_bad = gram(Ms_bad)
    print("  完好时 MᵀM/(N/2)：")
    for r in G_ok:
        print("      [" + "  ".join("%+8.4f" % v for v in r) + "]")
    print("\n  掉 %d 号电机后（剩下 5 个电机，仍除以 5/2）：" % lost)
    for r in G_bad:
        print("      [" + "  ".join("%+8.4f" % v for v in r) + "]")
    print("\n  非对角元最大绝对值：完好 %.2e  →  失效 %.4f"
          % (np.max(np.abs(G_ok - np.diag(np.diag(G_ok)))),
             np.max(np.abs(G_bad - np.diag(np.diag(G_bad))))))
    print("""
  ⇒ 非对角项从 0 变成了 %.4f 量级。
     **正交性就是这个机制的全部基础，它没了。**

  ★ 为什么掉一个电机会破坏正交？因为正交性靠的是
     「所有电机一起」的对称性：
         Σ rollᵢ·pitchᵢ = 0
     这个和式里的每一项都参与了抵消，去掉一项就不再是 0。
     掉的那一项越大，剩下的偏差越大 ——
""" % np.max(np.abs(G_bad - np.diag(np.diag(G_bad)))))
    print("     本例掉的是 0 号电机：(roll=%+.3f, pitch=%+.3f, yaw=%+.1f)"
          % (M_full[lost, 0], M_full[lost, 1], M_full[lost, 2]))

    names3 = ["roll", "pitch", "yaw"]
    sub("③ 后果：打纯 roll，串出 pitch 和 yaw")
    print("  用「按对角元归一化后的 B·A」来看串扰结构（理想 = 单位阵）：\n")

    def structure(A):
        """S[i,j] = 「轴 j 的指令在轴 i 上产生多少响应，以轴 i 自己的量程为单位」

        ★ 这里必须按【行】归一化（除以 diag[i]）—— 作者第一版写成了
          `P / np.diag(P)`，那是按【列】除（numpy 的广播方向），
          于是同一件事在 ①b 和 ③ 里算出 50% 和 100% 两个数。
        """
        P = B[:, enabled] @ A
        return P / np.diag(P)[:, None]

    B_red = B[:, enabled]
    S_ap = structure(M_red[enabled])

    g_red, _ = px4_geninv(B_red)
    mix_red, _ = px4_normalise_mix(g_red, normalize_rpy=True)
    S_px = structure(mix_red)

    print("  ArduPilot（只把失效电机的系数清零）:")
    for r in S_ap:
        print("      [" + "  ".join("%+8.4f" % v for v in r) + "]")
    print("\n  PX4（对剩下的 5 个电机重算伪逆）:")
    for r in S_px:
        print("      [" + "  ".join("%+8.4f" % v for v in r) + "]")

    # ★ 只用 roll/pitch/yaw 这 3×3 子块来算串扰比例。
    #   为什么不用 throttle 那一行：throttle 的「对角元」在掉电机后
    #   本身就缩水了，除出来的比例会虚高、不好解读。
    A3_ap = S_ap[:3, :3]
    A3_px = S_px[:3, :3]
    off_ap = np.max(np.abs(A3_ap - np.eye(3)))
    off_px = np.max(np.abs(A3_px - np.eye(3)))
    # ★ 找出最大的那一处在哪两轴之间（不写死，免得改了机型就失配）
    D = np.abs(A3_ap - np.eye(3))
    ii, jj = np.unravel_index(np.argmax(D), D.shape)
    print("\n  roll/pitch/yaw 三轴子块的交叉耦合：")
    print("      AP  : 最大 %.4f  ⇒ 打一根杆会串出 %.1f%% 的另一个轴"
          % (off_ap, 100 * off_ap))
    print("      PX4 : 最大 %.2e  ⇒ 精确为 0" % off_px)
    print("""
  ⇒ ★★ **AP 的串扰是 %.0f%%，PX4 精确为 0。**

  ★ 读法：S[i,j] = 「轴 j 的指令，在轴 i 上产生多少响应（以轴 i 自己的满量程为单位）」。
     本例 AP 的最大值出现在 %s 行、%s 列 ——
     意思是「**打%s，会串出 %+.0f%% 的%s力矩**」。

  ★★ 而且注意：**这不是物理限制，是算法限制。**
""" % (100 * off_ap, names3[ii], names3[jj], names3[jj],
       100 * A3_ap[ii, jj], names3[ii]))
    print("""     六旋翼掉一个电机后秩仍然是 4（⑥ 会验证），
     说明「纯 yaw 的可行解」是存在的 ——
     PX4 的伪逆确实精确找到了它（S_px = 单位阵）。
     是 AP 的固定系数表找不到。

  原因清清楚楚：
      · PX4 的伪逆是对【剩下的 5 个电机】重新求解 B_red·u = w
      · AP 只是把失效电机的系数清零，剩下 5 个电机的输出【原封不动】

  ⇒ 前者是「重新分配」，后者是「少一个就不管了」。
""")

    sub("④ 定量：打一根杆，另一根轴上冒出多少")
    print("  控制器给「纯 roll」指令，看实得力矩里 roll 之外的分量占多少。")
    print("  （用比例而不是绝对值，这样两家的归一化差异不会干扰结论）\n")
    P_ap = B_red @ M_red[enabled]           # 4×4，和 ③ 里的 P 是同一个东西
    P_px = B_red @ mix_red
    rows = []
    worst = (0.0, "", "")
    for name, cmd in (("纯 roll", np.array([1.0, 0.0, 0.0])),
                      ("纯 pitch", np.array([0.0, 1.0, 0.0])),
                      ("纯 yaw", np.array([0.0, 0.0, 1.0]))):
        # ★ 油门取 0：这是个【纯矩阵探针】，不是飞行工况。
        #   带上油门的话，throttle 列本身的交叉项会混进来，读数就不干净了。
        x = np.array([cmd[0], cmd[1], cmd[2], 0.0])
        j = int(np.argmax(cmd))
        wa = P_ap @ x
        wp = P_px @ x
        # ★ 分母必须用【响应轴 k 自己】的量程 P[k,k]，
        #   再乘上同一大小的指令 —— 这样才和 ③ 的 S[i,j] = P[i,j]/P[i,i] 同口径。
        #   （作者第一版用的是 P[j,j]（指令轴的量程），
        #     于是同一件事在 ③ 算 50%、在 ④ 算 100%。）
        ra = [(wa[k] / (P_ap[k, k] * cmd[j]) if k != j else 0.0) for k in range(3)]
        rp = [(wp[k] / (P_px[k, k] * cmd[j]) if k != j else 0.0) for k in range(3)]
        for k in range(3):
            if k != j and abs(ra[k]) > abs(worst[0]):
                worst = (ra[k], names3[j], names3[k])
        rows.append([name,
                     "%+.1f%%" % (100 * ra[0]), "%+.1f%%" % (100 * ra[1]),
                     "%+.1f%%" % (100 * ra[2]),
                     "%+.1f%%" % (100 * rp[0]), "%+.1f%%" % (100 * rp[1]),
                     "%+.1f%%" % (100 * rp[2])])
    tab(rows, ["指令", "AP→r", "AP→p", "AP→y", "PX4→r", "PX4→p", "PX4→y"])
    print("""
  ★ 表里是「串到该轴上的力矩 ÷ 该轴本该产生的力矩」（和 ③ 的 S 矩阵同口径）。
     理想情况应该全是 0.0%% —— PX4 那一半确实全是 0.0%%（数值误差量级）。

  ★★ AP 最严重的一条：**打 %s，串出 %+.1f%% 的 %s 力矩。**
""" % (worst[1], 100 * worst[0], worst[2]))
    print("""  ★ 后果：姿态控制器是闭环的，这些串扰会被当成扰动修掉 ——
     所以飞机**不会掉下来**，但控制余量被白白吃掉了一大块。
     这正是 AP_MotorsMatrix.cpp:268 那句
     「Hex aircraft are only marginal and must be handles with care」
     背后的真实含义。

  ★★ 对比 PX4：它什么都不用调，因为伪逆本来就定义在
     「当前有效的 B」上。B 少一列，解自动变成 5 个电机的最优分配。
""")

    sub("⑤ ArduPilot 的对策：thrust_boost（以及它为什么只是补丁）")
    print("""  AP_MotorsMatrix.cpp:263-272 的注释原文（含一个拼写错误）：

    // Under the motor lost condition we remove the highest motor output from
    // our calculations and let that motor go greater than 1.0
    // ...
    // Hex              : MOT_YAW_HEADROOM = 0,   ATC_RAT_RLL_IMAX = 1.0,

  ★ 读懂这段话：AP 不是重新分配，而是**假装那个电机不存在**，
    让它「名义上超过 1.0」，好把剩余的限幅空间让给别的电机。

  代码里是这样落实的（:284, :346, :354-356）：
      if (!is_zero(_yaw_factor[i]) && (!_thrust_boost || i != _motor_lost_index))
      ...
      if (_thrust_boost) rpy_high = boost_ratio(rpy_high, _thrust_rpyt_out[_motor_lost_index]);

  ⇒ **这是工程补贴（workaround），不是解法。**
     它需要用户按机型手工调 MOT_YAW_HEADROOM / ATC_RAT_*_IMAX，
     注释里给了 5 种机型的推荐值。

  ★★ 对比之下 PX4 什么都不用调 ——
     因为伪逆本来就是「对当前有效的 B 重新求解」。
     B 少一列，解就自动变成 5 个电机的最优分配。

     **这就是「数学上通解」和「按情况打补丁」的区别。**

  ★ 当然，PX4 也有它自己的问题：失效瞬间 B 变了，geninv 重算，
     `_mix` 整个换一副面孔 ⇒ **所有电机的输出会突跳**。

     PX4 的对策是 `setHadActuatorFailure()`（ControlAllocation.hpp:108）：
         一旦检测到失效就【冻结归一化因子】不再更新（:62），
         避免 `_control_allocation_scale` 跟着变导致二次跳变。

     但它【不解决】分配矩阵本身的突变。
     ⇒ 这是一个可以改进的点（后面 §4.6.4 会讲）：
        对分配矩阵做在线插值 / 速率限制（类似操作空间控制里的
        null-space 平滑切换），让失效过渡连续。
""")

    sub("⑥ 顺带：四旋翼为什么没法做这件事")
    print("""  AP_MotorsMatrix.cpp:272 的注释：
      「Quads cannot make use of motor loss handling because it doesn't have
        enough degrees of freedom.」

  验算一下：四旋翼有 4 个电机、要对 4 个轴（roll/pitch/yaw/thrust）负责。
  掉一个 ⇒ 3 个电机要产生 4 个独立的力矩分量。
  **3 个执行器张成的空间最多 3 维，装不下 4 维的目标** ⇒ 数学上无解，
  不是算法不好。

  下面用秩来验证：
""")
    Bq = px4_effectiveness("QUAD_X", up_positive=True)[[0, 1, 2, 3], :]
    for nm, BB in (("四旋翼 4 电机", Bq), ("四旋翼掉 1 个（剩 3）", Bq[:, 1:]),
                   ("六旋翼 6 电机", B), ("六旋翼掉 1 个（剩 5）", B[:, 1:])):
        _, rk = px4_geninv(BB)
        print("      %-24s 形状 %-8s 秩 = %d  %s"
              % (nm, str(BB.shape), rk, "✅ 可解" if rk == 4 else "❌ 无解"))
    print("""
  ⇒ 掉电机后六旋翼的秩【仍然是 4】（还有冗余），四旋翼掉到 3。
     冗余度 = 电机数 − 轴数 = 6 − 4 = 2，掉 1 个还剩 1 的冗余。
     四旋翼 4 − 4 = 0，本来就一点冗余都没有。

  ★★★ 这句话就是整个「电机失效容错」的判据：
      **执行器数量必须严格大于目标维度，且剩下的执行器仍然张满整个空间。**
      六旋翼勉强够（1 冗余），八旋翼舒服得多，
      四旋翼**根本上做不到** —— 任何软件都救不了。
""")

    sub("⑦ 顺带说清「倒飞」：它由什么决定")
    print("""  ★ 先说一个容易搞混的点：**「姿态倒过来」和「倒飞」不是一回事。**
     姿态翻转（roll = 180°）只是坐标系转了 180°，
     推力方向仍然沿机体轴 —— 此时飞机被推向【地面】。
     要悬停，必须让推力反向，即 **推力可以取负值**。

  ⇒ 所以「能不能倒飞」在数学上就是一个问题：
     **执行器的取值范围里有没有负推力？**

  在 PX4 里这件事是【一行参数】：
     CA_ROTORn_AZ 默认 -1.0（螺旋桨朝上，module.yaml:182-192）
     改成 +1.0 就是「螺旋桨朝下」，效率矩阵的推力行整个翻号。
     `3D 电调` 再把执行器范围从 [0,1] 扩到 [-1,1] 即可。
""")
    B_flip = px4_effectiveness("QUAD_X", up_positive=True)
    B_inv = B_flip.copy()
    B_inv[3, :] = -B_inv[3, :]
    print("  同一个 QUAD_X，只把 AZ 从 -1 翻成 +1：")
    for name, BB in (("AZ = -1（正常）", B_flip), ("AZ = +1（倒装）", B_inv)):
        print("      %-18s 推力行 = [%s]"
              % (name, ", ".join("%+.2f" % v for v in BB[3])))
    print("""
  ★ 推力行只是翻了个号 —— 而斜杠那三行（roll/pitch/yaw）【一模一样】。
    ⇒ **倒飞不需要新的混控逻辑**，只需要：
         (a) 允许电机出负推力（3D 电调 / 可逆电调）
         (b) 把推力指令的符号放开
     换句话说，倒飞的障碍是**硬件**（电调不支持反向），不是算法。

  ★★★ 那 ArduPilot 呢？这里必须记一次【自我更正】。

     ★ 我第一版写的是「AP 4.5.7 搜不到 MOT_3D_ENABLE / 3D_MODE，
       所以不支持倒飞」。**这是错的 —— 我搜错了名字，而且搜错了层。**

     真相：AP 的 3D 参数叫 `SERVO_BLH_3DMASK`，
     它在 `libraries/AP_BLHeli/` 和 HAL 的输出层，【不在 AP_Motors 里】：

         libraries/AP_BLHeli/AP_BLHeli.cpp:127-135
             // @Param: 3DMASK
             // @Description: Mask of channels which are dynamically reversible.
             //   This is used to configure ESCs in '3D' mode, allowing for the
             //   motor to spin in either direction
             AP_GROUPINFO("3DMASK", 10, AP_BLHeli, channel_reversible_mask, 0),

     ★★ 这暴露了一个非常干净的架构对比：

         PX4  ：3D 做在【混控层】—— 效率矩阵的推力行翻号
         AP   ：3D 做在【输出层】—— DShot 的电平映射

     而 `AP_Motors` 对 3D 【一无所知】（grep `3D` / `revers` 零匹配）。

     ★ 可执行的推论（从映射公式直接读出，见 doc §4.3.5）：
       AP 开了 3D 后 "电机指令 0.5" = PWM 1500 = 停转，
       但 MOT_SPIN_MIN / MOT_SPIN_MAX 默认 0.15 / 0.95 【不围绕 0.5 居中】
       ⇒ 开 3D 必须重设 SPIN_MIN / SPIN_MAX（例如 0.25 / 0.75）。

     AP 里另外两个能做非平面推力的类是：
         AP_MotorsTailsitter   （尾坐式 VTOL，推力随姿态转）
         AP_Motors6DOF         （真正的 6 自由度）
     它们走的是【独立的混控子类】——
     这正是 §4.3.4 讲的「查表法要为新构型写新代码」的又一个例子。
""")


# ===========================================================================
# 实验 4：饱和
# ===========================================================================

def exp4():
    head("实验 4  饱和：ArduPilot 的 yaw_allowed vs PX4 的三档 Airmode")

    print("""
  电机只能出正推力，而且有上限。当「roll 需要左前加大、右后减小」的时候，
  小到 0 就减不下去了 —— 这就是【饱和】。

  两家用了完全不同的思路：
      ArduPilot：先算「yaw 能塞进多少」（yaw_allowed），再整体缩放 rpy
      PX4      ：顺序去饱和（sequential desaturation），
                 且按 MC_AIRMODE 分三档，可以选择「抬油门保姿态」
""")

    frame = "QUAD_X"
    B = px4_effectiveness(frame)[[0, 1, 2, 3], :]
    M_ap = ap_normalise(ap_mixer_raw(frame))

    sub("① ArduPilot：yaw 的权限由「电机还能往两边走多少」决定")
    print("""  AP_MotorsMatrix.cpp:276-308
      yaw_allowed = min over motors of  (电机到限位的余量 / |yaw_factor|)
      yaw_allowed = MAX(yaw_allowed, _yaw_headroom · 0.001)   // 默认 0.2

  `MOT_YAW_HEADROOM` 默认 200（AP_MotorsMulticopter.h:8）⇒ 下限 0.2。
  「headroom」= 裕度：**至少要给 yaw 留出 20% 的指令权限**。

  ★ 为什么「余量 / |yaw_factor|」？因为归一化后 yaw 系数最大只有 0.5，
    一个满 yaw 指令（±1.0）要把电机推离中心 ±0.5。
    所以「还有多少余量」除以 0.5 就是「还能塞进多少 yaw 指令」。

  ★★ 还有一层：油门本身也会占掉行程。AP 的做法是
     `get_throttle_avg_max()`（AC_AttitudeControl_Multi.cpp:395-399）——
     低油门时**悄悄把平均推力抬到「还不至于让飞机上升」的水平**，
     好腾出余量给姿态控制。下面把这个也算进去。
""")

    rows = []
    for thr in (0.0, 0.25, 0.5, 0.75, 1.0):
        r = []
        for roll in (0.0, 0.25, 0.5, 0.75, 1.0):
            _, lim, yaw_used, rs = ap_output_armed_stabilizing(
                M_ap, roll, 0.0, 1.0, thr,
                throttle_avg_max=ap_throttle_avg_max(thr))
            r.append(yaw_used)
        rows.append(["%.2f" % thr] + ["%+.3f" % v for v in r])
    tab(rows, ["油门＼roll", "0.00", "0.25", "0.50", "0.75", "1.00"])
    print("""
  ⇒ 表里是【实际生效的 yaw 指令】（要求 1.0，被 yaw_allowed 截到多少）。

  ★ 三个规律，一眼可见：
      1. **油门很低时 yaw 权限被压到下限 0.2**：
         电机都贴在 0 附近，往下没有余量，`yaw_allowed` 算出来接近 0，
         被 `_yaw_headroom` 兜住才留下 0.2
      2. 油门回到 0.5 以上后 yaw 权限恢复 —— 因为
         `throttle_thrust_best_rpy = MIN(0.5, throttle_avg_max)` 被**封顶在 0.5**，
         分配器的计算基准就不再随油门上升了
      3. roll 越大，电机越偏向一侧 ⇒ 余量越少 ⇒ yaw 权限越小

  ★★ 这就是那个「0.5 归一化」的另一面：
     roll/pitch/yaw 三个系数都只占 ±0.5，
     所以三个通道【共享】那 ±0.5 的行程 —— 谁用多了谁就把别人挤掉。

  ★ 高油门那一头是另一个故事。`throttle_thrust_best_rpy` 封顶 0.5 的代价是：
     油门超过 0.5 的部分只能靠 `thr_adj` 直接叠加（:372-390），
     一旦叠加后超出 1.0，就会被砍掉并置 `limit.throttle_upper`。
     ⇐ **AP 在「姿态权限」和「推力」之间，明确选择保姿态。**
     这是设计取舍，不是 bug；但它意味着满油门的实际可用推力
     会比飞行员的指令少一截，`limit.throttle_upper` 就是给上层控制器的通知。
""")

    sub("①b 超出范围时：rpy_scale 与 limit 标志")
    rows = []
    for roll in (0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0):
        _, lim, _, rs = ap_output_armed_stabilizing(
            M_ap, roll, 0.0, 0.0, 0.5, throttle_avg_max=0.5)
        rows.append(["%.2f" % roll, "%.4f" % rs, str(lim)])
    tab(rows, ["roll 指令", "rpy_scale", "limit 标志"])
    rs_big = ap_output_armed_stabilizing(M_ap, 2.0, 0.0, 0.0, 0.5,
                                         throttle_avg_max=0.5)[3]
    print("""
  ★ 读法：
      - roll ≤ **1.0** 时 rpy_scale = 1.0，指令原样通过
        （因为 0.5 的归一化 ⇒ 满 roll 只把电机推离中心 ±0.5，
          刚好占满 1.0 的行程 —— 这就是「归一化到 0.5」设计的目的）
      - roll > 0.5 后 rpy_scale 掉到 1.0 以下 ——
        **三个姿态轴一起被等比缩小**（`limit.roll/pitch/yaw` 同时置位）
      - 这时的理论极限是 1.0/(2×0.5×0.5) = %.1f... 但油门 0.5 时
        实测最低只到 %.4f（因为油门也在占行程，见源码 :364-366
        那第二个 rpy_scale 约束「throttle_avg_max + rpy_low < 0」）
""" % (1.0 / (2 * 0.5 * 0.5), rs_big))
    print("""  ★★ limit 标志会被【上抛给姿态控制器】：
     AC_AttitudeControl 读这些标志去抑制积分项、回退角度环增益，
     避免「控制器还在加大指令，但执行机构已经打满」的积分饱和。
     这是 AP 这套设计里很关键的一环 —— 分配器不只输出指令，还报告饱和度。
""")

    sub("② PX4：三档 Airmode，直接看结果")
    print("""  MC_AIRMODE（ControlAllocationSequentialDesaturation.cpp:52-64）
      0 = disabled : 永不为了姿态【增加】推力，宁可牺牲 rpy        ← 默认
      1 = RP       : 允许增减推力保住 roll/pitch，yaw 优先级最低
      2 = RPY      : 全 airmode，先保 roll/pitch 再保 yaw

  下面做一件事：**油门给 0，roll 指令给满**。
  「纯姿态控制、不要高度」—— 这正是特技飞行的场景。
""")
    alloc = PX4Allocator(B)
    alloc.ctrl_trim = np.array([0.0, 0.0, 0.0, 0.0])
    alloc.trim = np.zeros(4)

    # ★ 注意推力指令是【负的】：PX4 的 F_z 沿机体 -z（向下为正）。
    #   所以「向上飞」= 给一个负的 thrust_z 指令。
    ctrl = np.array([0.5, 0.0, 0.0, 0.0])      # roll 指令 0.5，油门 0
    alloc.ctrl_sp = ctrl
    rows = []
    ref_roll = None
    ref_up = None
    for mode, name in ((0, "disabled"), (1, "RP"), (2, "RPY")):
        u = alloc.allocate(mode)
        w = up_of(alloc.achieved(u))           # 换算成「向上的推力」方便读
        if ref_roll is None:
            ref_roll = B[0] @ alloc.mix[:, 0] * ctrl[0]   # 无饱和时该有的 roll
        rows.append([name,
                     "%.3f" % u.min(), "%.3f" % u.max(),
                     "%+.4f" % w[0], "%+.4f" % w[3]])
    tab(rows, ["Airmode", "电机 min", "电机 max", "实得 roll", "实得更升力"])
    print("""  （无饱和时 roll 指令 0.5 本该产生 %+.4f 的滚转力矩；"更升力"就是向上的总推力）
""" % ref_roll)
    print("""  ★★ **这就是 Airmode 的全部含义**：
      - disabled：油门 0 时电机已经贴地，**roll 被完全放弃**（实得 roll = 0）
      - RP / RPY：为了做出 roll，**自动把总推力抬起来**，
        代价是飞机会往上窜（"更升力" > 0）。

  ⇒ 所以 Airmode 不是「一个模式」，它是**力矩与推力之间的优先级声明**：
         我要姿态，高度你看着办  —— 这就是 airmode。
""")

    sub("③ 顺序去饱和的细节：为什么是「顺序」")
    print("""  ControlAllocationSequentialDesaturation.cpp:145-171  (mixAirmodeRPY)
      desaturateActuators(actuator_sp, thrust_z);   // 先沿推力轴平移
      desaturateActuators(actuator_sp, yaw);        // 再沿 yaw 方向平移

  「去饱和」= 沿某个方向平移整个电机向量，直到最严重的越界被消掉：
      u ← u + k · desat_vec
      k = argmin |越界量|
      源码实现见 :88-119 computeDesaturationGain

  ★ :96 有个很容易漏掉的门槛：
        if (fabsf(desaturation_vector(i)) < 0.2f) continue;
    **效率太低的执行器不参与去饱和**，否则算出来的增益会非常大。
""")
    # 演示两遍半增益
    u0 = np.array([1.3, 0.4, 0.1, -0.2])
    desat = np.array([1.0, 1.0, 1.0, 1.0])
    k = PX4Allocator._compute_desaturation_gain(desat, u0, np.zeros(4), np.ones(4))
    u1 = u0 + k * desat
    k2 = 0.5 * PX4Allocator._compute_desaturation_gain(desat, u1, np.zeros(4), np.ones(4))
    u2 = u1 + k2 * desat
    print("  例：u = [1.30, 0.40, 0.10, −0.20]，沿 [1,1,1,1] 平移")
    print("      第一遍增益 k  = %+.4f  →  u = [%s]"
          % (k, ", ".join("%+.3f" % v for v in u1)))
    print("      第二遍 k/2    = %+.4f  →  u = [%s]"
          % (k2, ", ".join("%+.3f" % v for v in u2)))
    print("""
  ⇒ 源码 :81-85 做了【两遍】，第二遍用一半增益。
     **第二遍不是为了「修正」，是为了「别修过头」** ——
     第一遍按最严重的越界量平移，往往会冲到另一侧去，
     所以第二遍只走一半。

  ★★ 而且源码自己承认这是启发式，不是最优解（:70-72 的头注释）：

        「Note that as we only slide along the given axis, in extreme cases
          outputs can still contain values outside of [min_output, max_output]」

     ⇒ 「只沿一个方向平移」意味着**它不可能同时消掉两侧的越界**。
        例子里最后那个 −0.25 依然是负的。

  ★ 这跟 ArduPilot 的做法对比很有意思：
     AP 用的是 `rpy_scale` —— **等比缩小所有姿态指令**，
     所以它能保证【一定】不越界（rpy_scale 小到足够为止），
     代价是姿态被整体削弱。
     PX4 用的是平移 —— 尽量保住指令幅值，代价是**可能仍然轻微越界**，
     最后靠执行器层的硬限幅兜住（此时该电机就真的饱和了）。
     两家在同一件事上又选了两个相反的方向。
""")

    sub("④ mixYaw 里那个「抬高 15%」的技巧")
    print("  源码 :221-224")
    print("      ActuatorVector max_prev = _actuator_max;")
    print("      _actuator_max += (_actuator_max - _actuator_min) * 0.15f;")
    print("      desaturateActuators(_actuator_sp, yaw);")
    print("      _actuator_max = max_prev;")
    print("""
  ⇒ 临时把上限抬高 15%，让 yaw 去饱和时【允许一部分电机超限】。
     注释（:220）写得很直白：
        「allow some yaw response at maximum thrust」
     —— 满油门时还能拧一点机头，代价是那一小段电机指令会过头。

  ★ 对比 ArduPilot：它是**反过来**的 ——
     用 MOT_YAW_HEADROOM 保证【任何情况下】都给 yaw 留 20% 权限，
     但代价是全油门的姿态权限永久性变差。
     两家在同一个取舍上选了相反的方向。
""")

    sub("⑤ 定量对比：随着油门升高，roll 权限怎么变")
    print("  给固定的 roll = 0.5，扫油门。两列数字都是【相对于该有的值】的百分比：")
    print("      roll 列 = 实得滚转力矩 ÷ 无饱和时该有的滚转力矩")
    print("      推力列 = 实得升力   ÷ 指令油门该产生的升力")
    print("  理想情况两列都是 100%。\n")
    rows = []
    for thr in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        # ★ 参考升力：4 个电机 × 油门（AP 的 throttle 系数全是 1.0）
        ref_lift = 4.0 * thr
        # AP（用真实的 throttle_avg_max）
        u_ap, lim, _, rs = ap_output_armed_stabilizing(
            M_ap, 0.5, 0.0, 0.0, thr, throttle_avg_max=ap_throttle_avg_max(thr))
        w_ap = up_of(B @ u_ap)
        r = []
        for mode in (0, 1, 2):
            w = up_of(alloc.achieved(alloc.allocate(mode)))
            r.append((w[0], w[3]))

        def cell(roll, lift):
            return "%5.0f%% / %s" % (
                100 * roll / ref_roll,
                "  --" if ref_lift == 0 else "%4.0f%%" % (100 * lift / ref_lift))

        alloc.ctrl_sp = np.array([0.5, 0.0, 0.0, -thr])
        r = []
        for mode in (0, 1, 2):
            w = up_of(alloc.achieved(alloc.allocate(mode)))
            r.append((w[0], w[3]))
        rows.append(["%.1f" % thr, cell(w_ap[0], w_ap[3]),
                     cell(r[0][0], r[0][1]), cell(r[1][0], r[1][1]),
                     cell(r[2][0], r[2][1])])
    tab(rows, ["油门指令", "AP", "PX4 am0", "PX4 am1", "PX4 am2"])
    print("""
  ★★ 三个看点：

    1. **PX4 am0（出厂默认，无 airmode）在低油门时直接放弃姿态**：
       油门 0 时 roll 权限 0%，油门 0.2 时只有 57%。
       这是「油门收到底就翻不了身」的数学原因。

    2. **am1/am2 全程 100% roll，代价是推力**：
       油门 0 时它为了做 roll，自己把升力抬到了指令的 —— 无穷倍
       （指令是 0，实得 1.41）。这就是 airmode 飞机「收油门反而往上窜」的来源。

    3. **AP 走的是第三条路**：低油门时靠 `get_throttle_avg_max()`
       把平均推力偷偷抬到 @AVG@ 换一点姿态权限（油门 0 时 roll 拿到 10%），
       高油门时则稳定在 71% —— 姿态权限被固定的「裕度」思维
       永久性削掉一截，但**实得推力从来不超过指令**
       （只有油门 0.2 那一行是 108%，那是 `throttle_avg_max` 的抬升余量）。
       这是全套逻辑里最「保守」的一个选择。
""".replace("@AVG@", "%.3f" % ap_throttle_avg_max(0.0)))


# ===========================================================================
# 实验 5：电机模型
# ===========================================================================

def exp5():
    head("实验 5  电机模型：推力曲线 / SPIN_MIN / 电压补偿")

    print("""
  混控矩阵算出的是【推力】。但电调要的是【油门】。
  这两者之间不是线性的 —— 因为 T ∝ n²，而 n ∝ (油门 − 死区)。

  ArduPilot 用一条【可调指数】的曲线描述它：
      AP_Motors_Thrust_Linearization.cpp:114-129
""")

    sub("① 反解一条二次曲线")
    print("""  源码 :127
      throttle_ratio = ((expo - 1) + sqrt((1-expo)² + 4·expo·lift_max·thrust))
                       / (2·expo)

  ★ 这个式子看起来莫名其妙，其实它就是下面这个二次方程的正根：
      lift_max · thrust = (1 - expo) · t + expo · t²
  整理成标准形式：
      expo·t² + (1-expo)·t − lift_max·thrust = 0
      t = [−(1−expo) + sqrt((1−expo)² + 4·expo·lift_max·thrust)] / (2·expo)
      t = [(expo−1) + sqrt((1−expo)² + 4·expo·lift_max·thrust)] / (2·expo)
                                              ↑ 和源码 :127 一模一样

  ⇒ 所以这不是什么黑魔法，就是**求根公式**。
""")
    # 数值验证
    print("  数值验证（round-trip 误差）：")
    rows = []
    for expo in (0.0, 0.3, 0.65, 1.0):
        errs = []
        for thrust in np.linspace(0.01, 0.99, 50):
            t = thrust_curve(thrust, expo)
            back = thrust_curve_inverse(t, expo)
            errs.append(abs(back - thrust))
        rows.append(["%.2f" % expo, "%.3e" % max(errs)])
    tab(rows, ["expo", "max |正向∘反向 − 1|"])
    print("""
  expo = 0 时源码走的是特殊分支（:123-126 直接返回 lift_max·thrust），
  但它和求根公式在 expo→0 的极限是【一致的】，所以 round-trip 仍然是 1。
""")

    sub("② 曲线长什么样")
    print("      expo = 0.00 → 推力与油门线性（教科书假设）")
    print("      expo = 0.65 → ArduPilot 默认（AP_MotorsMulticopter.h:9）")
    print("      expo = 1.00 → 推力 ∝ 油门²（电机的物理模型）")
    print()
    rows = []
    for thrust in (0.1, 0.25, 0.35, 0.5, 0.75, 1.0):
        rows.append(["%.2f" % thrust] +
                    ["%.4f" % thrust_curve(thrust, e) for e in (0.0, 0.3, 0.65, 1.0)])
    tab(rows, ["期望推力", "expo=0 油门", "expo=0.3", "expo=0.65", "expo=1.0"])
    print("""
  ★ 注意 0.35 那一行（AP 默认的悬停油门 MOT_THST_HOVER = 0.35，
    见 AP_MotorsMulticopter.h:10）：
      线性假设（expo=0）下悬停需要 35%% 油门；
      按 AP 默认的 expo=0.65，实际需要 %.1f%% ——
      **高出 15 个百分点**。
""" % (100 * thrust_curve(0.35, 0.65)))

    sub("③ 再套上 SPIN_MIN / SPIN_MAX")
    print("""  AP_Motors_Thrust_Linearization.cpp:100-104
      thrust_to_actuator(thrust) = spin_min + (spin_max - spin_min) · curve(thrust)

  默认 SPIN_MIN = 0.15、SPIN_MAX = 0.95（AP_MotorsMulticopter.h:14-15）。
  ⇒ 电调指令被映射到 [0.15, 0.95]：
      0.15 是「电机能稳定转起来的最小油门」（保证不堵转）
      0.95 是「留 5% 给电调/电池电压波动」
""")
    rows = []
    for thrust in np.linspace(0.0, 1.0, 6):
        rows.append(["%.2f" % thrust,
                     "%.4f" % thrust_curve(thrust, 0.65),
                     "%.4f" % thrust_to_actuator(thrust, 0.65)])
    tab(rows, ["推力指令", "曲线后", "→ 电调指令"])
    print("""
  ★★ 这一步是**分辨率损失**的来源：
     推力 [0, 1] 被压缩进电调 [0.15, 0.95]，只有 0.8 的行程。
     再叠上 expo 曲线的压缩，**低推力区的分辨率会非常差**。

     下面按【不同推力工作点】分别算一个「台阶」有多大 ——
     因为分辨率是【局部】的，越靠近曲线陡的地方越差。
""")
    NSTEP = 1024          # 常见的 10 bit 油门分辨率
    rows = []
    for expo in (0.0, 0.3, 0.65, 1.0):
        cells = []
        for tq in (0.05, 0.10, 0.35, 0.70):
            a = thrust_to_actuator(tq, expo)
            b = thrust_to_actuator(tq + 0.01, expo)
            dt = (b - a) / 0.01                       # 每单位推力的行程
            lsb = dt / NSTEP * 100.0                  # 一个档位 = 百分之几推力
            cells.append("%.3f%%" % lsb)
        rows.append(["%.2f" % expo] + cells)
    tab(rows, ["expo", "推力 0.05", "推力 0.10", "推力 0.35(悬停)", "推力 0.70"])
    print("""  表里是【一个电调解析度档位（1/%d）对应多少推力变化】。""" % NSTEP)
    print("""
  ★ 两个结论：

    1. **推力越低，分辨率越差。** 因为曲线把行程都堆在了低推力区 ——
       推力 0.05 附近一个档位能跳 %.3f%% 的推力，
       悬停点只有 %.3f%%，相差 %.1f 倍。

    2. **expo 越大，低推力区越糟。** expo=1.0（纯二次）时
       推力 0.05 附近一个档位跳 %.3f%%，是 expo=0（线性）的 %.1f 倍。

  ★★ 这两个数就是「低油门抖动」的量化来源：
     油门越低，同样的电调分辨率能表达的推力台阶越粗，
     于是姿态控制器的精细输出会被【量化】成跳变 ——
     表现出来就是低速下降时的抖动 / 高度振荡。

  ★ 这也解释了为什么 ArduPilot 推荐 DShot 而不是 PWM：
     DShot 有 2000 档（0~1999），普通 PWM 约 1000 档，
     OneShot125/42 只有 125~250 档 —— 分辨率差 4~16 倍。
     辅助判据见 `MOT_SPIN_MIN`：把它抬高一点（比如 0.15→0.2）
     等于把工作点从最差的分辨率区挪开，代价是损失一点推力范围。
""" % (100 * ((thrust_to_actuator(0.06, 0.65) - thrust_to_actuator(0.05, 0.65)) / 0.01) / NSTEP,
       100 * ((thrust_to_actuator(0.36, 0.65) - thrust_to_actuator(0.35, 0.65)) / 0.01) / NSTEP,
       ((thrust_to_actuator(0.06, 0.65) - thrust_to_actuator(0.05, 0.65)) /
        (thrust_to_actuator(0.36, 0.65) - thrust_to_actuator(0.35, 0.65))),
       100 * ((thrust_to_actuator(0.06, 1.0) - thrust_to_actuator(0.05, 1.0)) / 0.01) / NSTEP,
       ((thrust_to_actuator(0.06, 1.0) - thrust_to_actuator(0.05, 1.0)) /
        (thrust_to_actuator(0.06, 0.0) - thrust_to_actuator(0.05, 0.0)))))

    sub("④ 电压补偿：电池掉压时自动加油门")
    print("""  AP_Motors_Thrust_Linearization.cpp:117-120
      float battery_scale = 1.0;
      if (is_positive(batt_voltage_filt.get())) {
          battery_scale = 1.0 / batt_voltage_filt.get();
      }

  batt_voltage_filt 是【电池电压占满电电压的比例】，滤波后的值。
  ⇒ 电压掉到 80%，油门指令乘 1/0.8 = 1.25。

  ★ 为什么这在【推力域混控】下是对的？
    因为混控算出的是「我要多少牛顿」。
    电压掉了 ⇒ 同样的油门出的推力少了 ⇒ 那就给更多油门。
    如果混控在【转速域】做，这个补偿就得放到每个电机上分别做，
    而且没法在混控前统一处理 —— 这正是卷 1 第 7 章 讲的那件事。
""")
    rows = []
    for vol in (1.0, 0.9, 0.8, 0.7):
        print_ = None
        # 悬停需要 0.35 的推力
        need = thrust_curve(0.35, 0.65, battery_scale=1.0 / vol)
        rows.append(["%.2f" % vol, "%.4f" % (1.0 / vol),
                     "%.4f" % thrust_to_actuator(0.35, 0.65),
                     "%.4f" % (0.15 + 0.8 * need)])
    tab(rows, ["电压比", "补偿倍数", "满电时电调指令", "掉压后电调指令"])
    print("""
  ★ 但这里有个**陷阱**：补偿是【单向】的。
    电压比 > 1（比如满电 12.6V 而 BAT_VOLT_MAX 设成了 4S 的 16.8V）
    会让 battery_scale < 1，油门被【压低】。
    ⇒ `MOT_BAT_VOLT_MAX` 必须按【实际电池满电电压】设，
      设大了会出现「满电时反而没劲」。
      ArduPilot 的默认值是 0（= 关闭补偿），
      见 AP_MotorsMulticopter.h:17-18。
""")

    sub("⑤ 小结：从力矩到电调指令的完整链条")
    print("""
      ┌─────────────────────────────────────────────────────┐
      │ 期望力矩 / 推力   τ_roll, τ_pitch, τ_yaw, F_up       │
      └───────────────────────┬─────────────────────────────┘
                              │ ① 混控（实验 1-3）
                              │    AP : M_ap · [roll,pitch,yaw,thr]
                              │    PX4: trim + mix · (c − c_trim)
                              ▼
      ┌─────────────────────────────────────────────────────┐
      │ 每个电机的【推力指令】 u_i ∈ [0, 1]                   │
      └───────────────────────┬─────────────────────────────┘
                              │ ② 饱和处理（实验 4）
                              │    AP : yaw_allowed + rpy_scale
                              │    PX4: 顺序去饱和 × 三档 airmode
                              ▼
      ┌─────────────────────────────────────────────────────┐
      │ 限幅后的推力指令                                     │
      └───────────────────────┬─────────────────────────────┘
                              │ ③ 推力曲线 + 电压补偿（本实验）
                              │    t = spin_min + (spin_max−spin_min)·curve(u)
                              ▼
      ┌─────────────────────────────────────────────────────┐
      │ 电调油门指令 t ∈ [0.15, 0.95]                        │
      └─────────────────────────────────────────────────────┘

  ★ 注意 ② 和 ③ 的【顺序】：先饱和处理，后曲线映射。
    如果反过来的话，曲线映射会破坏「线性求和」的前提 ——
    混控矩阵的推导（卷 1 第 7 章）假定了推力可线性叠加。
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("两条路走到同一个矩阵（几何式 vs 查表法）", exp1),
    "2": ("三种「逆」：方阵逆 / 伪逆 / 正向混控", exp2),
    "3": ("电机失效：正交性被破坏之后", exp3),
    "4": ("饱和：yaw_allowed vs 三档 Airmode", exp4),
    "5": ("电机模型：推力曲线 / 电压补偿", exp5),
}


def main():
    args = sys.argv[1:]
    if not args:
        args = sorted(EXPERIMENTS.keys())
    print()
    hr("#")
    print("#  卷 1 第 8 章 · 混控矩阵与电机模型 —— 配套实验")
    hr("#")
    for a in args:
        if a not in EXPERIMENTS:
            print("  未知实验：%s（可选 %s）" % (a, "/".join(sorted(EXPERIMENTS))))
            return 1
        EXPERIMENTS[a][1]()
    print()
    hr("=")
    print("  完成。")
    hr("=")
    return 0


if __name__ == "__main__":
    sys.exit(main())
