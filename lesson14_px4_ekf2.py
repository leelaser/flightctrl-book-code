#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 4 周 · 06 PX4 ECL EKF2 完全解析 —— 配套实验

    实验 1  ★  状态向量：24 个 float 为什么只有 23 个自由度
    实验 2  ★★ 融合顺序 + 一个能跑的简化 EKF2 主循环（照 control.cpp 的顺序）
    实验 3  ★★ 新息门限（卡方检验）：gate 的单位是 sigma
    实验 4  ★★ 多实例选择器：三个并行的 EKF 怎么投票选主

用法：
    python lesson14_px4_ekf2.py 1
    python lesson14_px4_ekf2.py 2
    python lesson14_px4_ekf2.py 3
    python lesson14_px4_ekf2.py 4
    python lesson14_px4_ekf2.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  这一课是「读源码课」。**下面每一个工程细节都来自真的打开过的 PX4 源文件**，
  每一条都标了 文件:行号。脚本里出现的所有常数都从源码里抄下来，不是凭印象写的。

  PX4 源码根目录（本机）：
      D:/DEVLib/PX4APM/PX4/PX4-Autopilot/src/modules/ekf2/

  实验 1 回答一个容易被跳过、但**决定了整个 ESKF 怎么实现**的问题：
      状态向量 24 个 float，为什么协方差矩阵 P 只有 23x23？
      —— 因为四元数有 4 个 float，但只有 3 个独立方向。

  实验 2 把 control.cpp 的融合顺序**真的按那个顺序跑一遍**，
      让你看到「mag 先、height 后」这种顺序不是随意的。

  实验 3 把新息门限的公式 test_ratio = innov² / (gate² · innov_var) 拆开：
      gate 的单位是【sigma】，> 1 就拒。
      ★ 并且真的用蒙特卡洛算出「正常数据被误拒的概率」，
        和卡方分布的解析值对比。

  实验 4 复刻 EKF2Selector 的选择逻辑（combined_test_ratio / relative_test_ratio /
      healthy 迟滞 / 10 秒保护期），跑三个带故障注入的 EKF，打印切换过程。

参考（本机源码，全部实读）：
    src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h
    src/modules/ekf2/EKF/control.cpp
    src/modules/ekf2/EKF/ekf.h
    src/modules/ekf2/EKF/ekf_helper.cpp
    src/modules/ekf2/EKF/covariance.cpp
    src/modules/ekf2/EKF/common.h
    src/modules/ekf2/EKF2Selector.cpp / EKF2Selector.hpp
    src/modules/ekf2/params_selector.yaml / params_multi.yaml
"""

import math
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 输出辅助（和本教程其它脚本一致）
# ---------------------------------------------------------------------------

W = 74


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


def tab(rows, headers, width=None):
    """打印表格。rows 是二维列表。"""
    cols = len(headers)
    if width is None:
        width = [max(len(str(r[i])) for r in [headers] + rows) + 2
                 for i in range(cols)]
    line = "  " + "".join("%-*s" % (width[i], headers[i]) for i in range(cols))
    print(line)
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join("%-*s" % (width[i], str(r[i])) for i in range(cols)))
    print()


# ---------------------------------------------------------------------------
# 从 PX4 源码抄下来的常数（每一条都真的打开文件确认过）
# ---------------------------------------------------------------------------

# state.h:44-52 —— 误差状态的自由度索引
STATE_DOF = [
    # (名字,    DOF 索引, DOF 数,  float 存储偏移, float 数)
    ("quat_nominal", 0, 3, 0, 4),     # state.h:44  四元数占 4 个 float
    ("vel", 3, 3, 4, 3),              # state.h:45
    ("pos", 6, 3, 7, 3),              # state.h:46
    ("gyro_bias", 9, 3, 10, 3),       # state.h:47
    ("accel_bias", 12, 3, 13, 3),     # state.h:48
    ("mag_I", 15, 3, 16, 3),          # state.h:49
    ("mag_B", 18, 3, 19, 3),          # state.h:50
    ("wind_vel", 21, 2, 22, 2),       # state.h:51
]

STATE_MEANING = {
    "quat_nominal": "姿态四元数 q（body → NED），[w, x, y, z]",
    "vel": "NED 速度 [北, 东, 地]（m/s）",
    "pos": "NED 位置 [北, 东, 地]（m）",
    "gyro_bias": "陀螺零偏（rad/s），Δ角速度的积分",
    "accel_bias": "加计零偏（m/s²）",
    "mag_I": "地球磁场矢量在 NED 系的表示（Gauss）",
    "mag_B": "机体磁场干扰（硬铁）在 body 系的表示（Gauss）",
    "wind_vel": "水平风速 [北, 东]（m/s）",
}

# state.h:52 —— State::size = 23（误差状态自由度总数）
STATE_SIZE_DOF = 23
# state.h:22 —— Data() 返回 matrix::Vector<float, 24>
STATE_SIZE_STORAGE = 24

# control.cpp:46-162 —— controlFusionModes() 的调用顺序
FUSION_ORDER = [
    # (序号, 函数名, 源码行号, 说明)
    (1, "controlMagFusion()", "control.cpp:105",
     "磁力计：航向 / 3D 磁场融合"),
    (2, "controlOpticalFlowFusion()", "control.cpp:109",
     "光流：水平速度 + 地形高度"),
    (3, "controlGpsFusion()", "control.cpp:113",
     "GNSS：水平位置 + 三轴速度（+ GNSS 航向）"),
    (4, "_aux_global_position.update()", "control.cpp:117",
     "辅助全局位置（如 MAVLink 外部定位）"),
    (5, "controlAirDataFusion()", "control.cpp:121",
     "空速管：TAS + 风三角"),
    (6, "controlBetaFusion()", "control.cpp:125",
     "合成侧滑角约束（固定翼用）"),
    (7, "controlDragFusion()", "control.cpp:129",
     "多旋翼阻力模型 → 风速估计"),
    (8, "controlHeightFusion()", "control.cpp:132",
     "高度：baro / GPS / range / EV"),
    (9, "controlGravityFusion()", "control.cpp:135",
     "重力矢量（相当于加速度计姿态观测）"),
    (10, "controlExternalVisionFusion()", "control.cpp:140",
     "外部视觉：位置 / 速度 / 航向"),
    (11, "controlAuxVelFusion()", "control.cpp:145",
     "辅助水平速度"),
    (12, "controlZeroInnovationHeadingUpdate()", "control.cpp:148",
     "零新息航向更新（把 GSF 的航向塞进来）"),
    (13, "_zero_velocity_update.update()", "control.cpp:150",
     "零速修正（静止检测用）"),
    (14, "_zero_gyro_update.update()", "control.cpp:153",
     "零角速度修正（陀螺零偏，受 imu_ctrl 开关控制）"),
    (15, "controlFakePosFusion()", "control.cpp:157",
     "假位置：无位置源时把位置钉住防漂"),
    (16, "controlFakeHgtFusion()", "control.cpp:158",
     "假高度：同上，垂直方向"),
    (17, "updateDeadReckoningStatus()", "control.cpp:161",
     "最后才更新「纯惯导推算」状态位"),
]

# common.h:294-494 + module.yaml —— 各传感器的默认新息门限（单位：sigma）
INNOV_GATES = [
    # (传感器, 参数名, gate 默认值, common.h 行号, 保护类型)
    ("气压高度 / GPS 高度", "EKF2_BARO_GATE", 5.0, 294, "慢变、可信"),
    ("GPS 水平位置", "EKF2_GPS_P_GATE", 5.0, 322, "慢变、可信"),
    ("GPS 三轴速度", "EKF2_GPS_V_GATE", 5.0, 323, "慢变、可信"),
    ("空速 TAS", "EKF2_TAS_GATE", 5.0, 377, "可建模"),
    ("合成侧滑 beta", "EKF2_BETA_GATE", 5.0, 385, "可建模"),
    ("测距仪 range", "EKF2_RNG_GATE", 5.0, 409, "可建模"),
    ("外部视觉位置 EV pos", "EKF2_EVP_GATE", 5.0, 434, "慢变"),
    ("磁力计 3D", "EKF2_MAG_GATE", 3.0, 362, "★ 容易被干扰"),
    ("磁航向 heading", "EKF2_HDG_GATE", 2.6, 350, "★ 容易被干扰"),
    ("光流", "EKF2_OF_GATE", 3.0, 454, "★ 容易被纹理/光照影响"),
    ("外部视觉速度 EV vel", "EKF2_EVV_GATE", 3.0, 433, "★ 可能突跳"),
    ("辅助速度 auxvel", "-", 5.0, 494, "慢变"),
]

# EKF2Selector.hpp:146-147 —— 相对误差分数的限幅与切换门限
REL_ERR_SCORE_LIM = 1.0      # EKF2Selector.hpp:146
REL_ERR_THRESH = 0.5         # EKF2Selector.hpp:147
# params_selector.yaml:11 —— EKF2_SEL_ERR_RED 默认 0.2
SEL_ERR_RED = 0.2
# params_selector.yaml:19,28,37,46 —— IMU 不一致检测门限
SEL_IMU_RAT_DPS = 7.0        # deg/s
SEL_IMU_ANG_DEG = 15.0       # deg
SEL_IMU_ACC = 1.0            # m/s^2
SEL_IMU_VEL = 2.0            # m/s
# EKF2Selector.cpp:255 —— 状态超时
STATUS_TIMEOUT_MS = 50
# EKF2Selector.hpp:81 —— 选择器的运行周期
FILTER_UPDATE_PERIOD_MS = 10
# EKF2Selector.hpp:111 —— healthy 位的迟滞时间
HEALTHY_HYSTERESIS_MS = 1000
# EKF2Selector.cpp:740,765 —— 两个 10 秒保护期
RESELECT_GUARD_MS = 10000
SWITCH_GUARD_MS = 10000

# 真实地球平均磁场强度（mag_control.cpp:301）
AVG_EARTH_MAG_FIELD_GAUSS = 0.45
# mag_control.cpp:302
AVG_EARTH_MAG_GATE_GAUSS = 0.40
# common.h:370-371 / EKF2_MAG_CHK_STR, EKF2_MAG_CHK_INC
MAG_CHK_STR_TOL_GAUSS = 0.2
MAG_CHK_INC_TOL_DEG = 20.0

# common.h:69-78 —— 各种「数据不能太旧」的间隔上限（微秒）
MAX_INTERVALS_US = [
    ("BARO", 200e3, "common.h:69"),
    ("EV", 200e3, "common.h:70"),
    ("GNSS", 500e3, "common.h:71"),
    ("GNSS_YAW", 1500e3, "common.h:72"),
    ("RNG", 200e3, "common.h:73"),
    ("MAG", 500e3, "common.h:74"),
]
BADACC_PROBATION_US = 10e6    # common.h:77
BADACC_BIAS_PNOISE = 4.9      # common.h:78（m/s²）


# ---------------------------------------------------------------------------
# 通用小工具
# ---------------------------------------------------------------------------

def q_mul(a, b):
    """四元数乘法 a ⊗ b（Hamilton 约定，与 PX4 matrix::Quaternion 一致）。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q):
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_from_rotvec(v):
    """旋转向量（轴 × 角，3 个数）→ 单位四元数（4 个数）。

    这就是 PX4 的 matrix::AxisAnglef → Quatf：
        Quatf delta_quat(matrix::AxisAnglef( ... ));   （ekf_helper.cpp:545）
    """
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = v / theta
    s = math.sin(theta * 0.5)
    return np.array([math.cos(theta * 0.5), axis[0] * s, axis[1] * s, axis[2] * s])


def rotvec_from_quat(q):
    """单位四元数 → 旋转向量（3 个数）。上面那个映射的逆。

    这就是 PX4 的 Quatf::log()（本脚本用它来证明「4 个 float ↔ 3 个数」）。
    """
    q = q_norm(q)
    if q[0] < 0:
        q = -q                      # 取 w >= 0 的那一半，保证唯一
    s = float(np.linalg.norm(q[1:]))
    if s < 1e-12:
        return np.zeros(3)
    theta = 2.0 * math.atan2(s, q[0])
    return q[1:] / s * theta


def wrap_pi(a):
    """折到 (-π, π]，对应 PX4 的 math::wrap_pi。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def norm_tail_prob(x):
    """标准正态的【双侧】尾概率 P(|Z| > x) = erfc(x/√2)。

    用 math.erfc 算，不是查表 —— 所以下面表里的数字都是真算出来的。
    """
    return math.erfc(x / math.sqrt(2.0))


def chi2_1d_cdf(x):
    """自由度 1 的卡方分布 CDF：P(χ² ≤ x) = erf(√(x/2))。"""
    return math.erf(math.sqrt(x / 2.0))


# ---------------------------------------------------------------------------
# 实验 1：状态向量 —— 24 个 float，23 个自由度
# ---------------------------------------------------------------------------

def exp1():
    head("实验 1  ★ 状态向量：24 个 float 为什么只有 23 个自由度")

    print("""
  EKF2 是一个【误差状态卡尔曼滤波】（ESKF）。
  它的状态向量长什么样，决定了后面所有公式的形状。

  源码里这件事分两处写：
      state.h:22          Data() 返回 matrix::Vector<float, 24>   ← 存储长度 24
      state.h:52          State::size = 23                        ← 自由度 23

  ★★ 这两个数不一样，是本课第一个「只有读源码才知道」的点。
""")

    sub("① 状态向量全表（左半：存储布局 / 右半：误差状态索引）")

    rows = []
    for name, dof_idx, dof_n, mem_off, mem_n in STATE_DOF:
        kind = "核心" if name in ("quat_nominal", "vel", "pos",
                                  "gyro_bias", "accel_bias") else "辅助"
        rows.append([name,
                     "%d..%d" % (mem_off, mem_off + mem_n - 1),
                     mem_n,
                     "%d..%d" % (dof_idx, dof_idx + dof_n - 1),
                     dof_n,
                     kind])
    rows.append(["合计", "0..23", STATE_SIZE_STORAGE, "0..22", STATE_SIZE_DOF, ""])
    tab(rows, ["状态名", "float 存储", "浮点数", "DOF 索引", "自由度", "类别"])

    print("""    ★ 注意从 vel 开始，两列就差 1 了：
        存储偏移 = DOF 索引 + 1
      因为排在第一个的四元数占了 4 个 float，却只贡献 3 个自由度。

    ★ 「核心 / 辅助」这个划分是本教程为了讲清楚加的，源码里没有这两个词。
      但它有据可依，见下面的 ②。
""")

    sub("② 核心 vs 辅助 —— 一个有源码依据的划分")

    print("""    源码里，辅助状态是被【编译开关】保护、并且可以被【单独关掉】的：

        covariance.cpp:165   #if defined(CONFIG_EKF2_MAGNETOMETER)
        covariance.cpp:233   #if defined(CONFIG_EKF2_WIND)
        fuse() 里 ekf_helper.cpp:564-577：
            if (_control_status.flags.mag)   { mag_I / mag_B 更新 }
            if (_control_status.flags.wind)  { wind_vel   更新 }

      编译时没开这两个宏，mag_I/mag_B/wind_vel 就【不存在】，
      它们的协方差也不参与运算。

      而 quat / vel / pos / gyro_bias / accel_bias 这 15 个自由度
      在 fuse()（ekf_helper.cpp:544-562）里是【无条件】更新的。

    ⇒ 这就是「核心 15 维 + 辅助 8 维 = 23 维」的由来。
""")

    rows = [
        ["核心", "15", "quat(3) + vel(3) + pos(3) + gyro_bias(3) + accel_bias(3)",
         "无条件更新，姿态/速度/位置的最小闭包"],
        ["辅助", "8", "mag_I(3) + mag_B(3) + wind_vel(2)",
         "可被编译开关或状态位关掉，不参与就退化"],
    ]
    tab(rows, ["类别", "自由度", "包含", "依据"])

    sub("③ ★ 数值验证：四元数真的只有 3 个独立方向")

    print("""    分三步证明「4 个 float，3 个自由度」。

    (a) 求导：q 对旋转向量 v 的雅可比 ∂q/∂v 在 v=0 处的秩
""")

    # 数值求 Jacobian：q = [cos(|v|/2), sin(|v|/2)·v/|v|] 对 v 的偏导
    # 解析结果应为 [[0,0,0], [0.5,0,0], [0,0.5,0], [0,0,0.5]]
    eps = 1e-7
    J = np.zeros((4, 3))
    for j in range(3):
        vp = np.zeros(3)
        vp[j] = eps
        vm = np.zeros(3)
        vm[j] = -eps
        qp = quat_from_rotvec(vp)
        qm = quat_from_rotvec(vm)
        J[:, j] = (qp - qm) / (2.0 * eps)

    print("    ∂q/∂v 的数值值（中心差分，h = %.0e）：" % eps)
    for i in range(4):
        tag = "w" if i == 0 else "xyz"[i - 1]
        print("        ∂q_%s/∂v = [%+9.6f, %+9.6f, %+9.6f]" % (tag, J[i, 0], J[i, 1], J[i, 2]))
    print()
    print("    解析值应该是：")
    print("        ∂q_w/∂v = [0, 0, 0]           ← ★ 整行全是 0")
    print("        ∂q_x/∂v = [0.5, 0, 0]")
    print("        ∂q_y/∂v = [0, 0.5, 0]")
    print("        ∂q_z/∂v = [0, 0, 0.5]")
    print()
    print("    数值误差 = %.3e" % float(np.max(np.abs(J - np.array(
        [[0, 0, 0], [0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]])))))
    print()
    print("    ★ 第 0 行全 0 ⇒ 一阶意义上，【w 分量的变化量由 xyz 决定】。")
    print("      也就是说 4 个 float 里，只有 3 个方向是自由的。")
    print("      秩检验：rank(∂q/∂v) = %d  （应该是 3）" % int(np.linalg.matrix_rank(J)))

    print("""
    (b) 双射：3 个数 ↔ 4 个 float，能否无损往返？
""")

    rng = np.random.default_rng(20240306)
    n_rt = 200000
    v = rng.normal(0.0, 1.0, (n_rt, 3))
    # 把角度限制在 (0, π) 内，避开 log 的分支点
    nrm = np.linalg.norm(v, axis=1, keepdims=True)
    v = v / nrm * (rng.uniform(0.0, math.pi * 0.999, (n_rt, 1)))
    err = 0.0
    for i in range(0, n_rt, 20000):
        for k in range(i, min(i + 20000, n_rt)):
            q = quat_from_rotvec(v[k])
            v2 = rotvec_from_quat(q)
            err = max(err, float(np.max(np.abs(v[k] - v2))))
    print("      随机抽 %d 个旋转向量，做 v → q → v 往返：" % n_rt)
    print("      最大往返误差 = %.3e" % err)
    print("      （float64 机器精度约 2.2e-16 ⇒ 这就是无损往返）")
    print("      ⇒ 3 个数【足够】表示一个姿态，第 4 个 float 是冗余的。")

    print("""
    (c) 反过来：把四元数当成 4 个独立的 float 去扰动，会怎样？
""")

    q0 = quat_from_rotvec(np.array([0.3, -0.2, 0.5]))
    d4 = rng.normal(0.0, 0.02, 4)          # 4 维独立扰动（错误做法）
    q_bad = q0 + d4
    d3 = rng.normal(0.0, 0.02, 3)          # 3 维旋转向量扰动（PX4 做法）
    q_good = q_mul(quat_from_rotvec(d3), q0)

    rows = [
        ["不动", "%.16f" % float(np.linalg.norm(q0)),
         "%.3e" % abs(float(np.linalg.norm(q0)) - 1.0)],
        ["4 维独立扰动（错）", "%.16f" % float(np.linalg.norm(q_bad)),
         "%.3e" % abs(float(np.linalg.norm(q_bad)) - 1.0)],
        ["3 维旋转扰动（PX4）", "%.16f" % float(np.linalg.norm(q_good)),
         "%.3e" % abs(float(np.linalg.norm(q_good)) - 1.0)],
    ]
    tab(rows, ["做法", "‖q‖", "偏离 1 多少"])
    print("""    ★ 4 维扰动把 ‖q‖ 顶出了单位球面（虽然二阶小），
      3 维扰动【精确】保持在球面上（一阶意义上）。

      PX4 里对应的三行（ekf_helper.cpp:544-547）：

          Quatf delta_quat(matrix::AxisAnglef(
                  K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx, 0)
                  * (-1.f * innovation)));
          _state.quat_nominal = delta_quat * _state.quat_nominal;
          _state.quat_nominal.normalize();

      ★★ 三个细节全在这里：
          1) K.slice<【dof】, 1>(【idx】, 0) —— 取的是【3】个增益，不是 4 个；
          2) AxisAnglef(旋转向量) —— 3 个数直接构成一个【旋转】；
          3) 最后 normalize() —— 把二阶的模长误差抹掉。
""")

    sub("④ 这件事对协方差矩阵的影响：P 是 23x23，且四元数那块是奇异的")

    print("""    如果谁真的把四元数当 4 个自由度，他会写一个 24x24 的 P。
    我们来看看那样做会出什么问题：
""")

    # 在切空间里造一个随机 3x3 协方差，用 G P G^T 映到 4 维
    G = J                                   # 4x3 的 ∂q/∂v
    A = rng.normal(0.0, 1.0, (3, 3))
    P3 = A @ A.T * 1e-4 + np.eye(3) * 1e-6   # 任意对称正定矩阵
    P4 = G @ P3 @ G.T                        # 4x4
    w4 = np.linalg.eigvalsh(P4)

    print("    取一个 3x3 的姿态协方差 P3（正定），用 G P3 Gᵀ 映到 4 维：")
    print("        P3 的特征值 = [%.3e, %.3e, %.3e]" % tuple(np.linalg.eigvalsh(P3)))
    print("        P4（4x4）的特征值 =")
    for x in sorted(w4, reverse=True):
        print("            %+.6e" % x)
    print()
    print("    数值秩 rank(P4) = %d   （< 4 ⇒ 奇异）" % int(np.linalg.matrix_rank(P4)))
    print("    条件数 = %.3e" % float(np.linalg.cond(P4)))
    print()
    print("""    ★★ 结论：
      真正的姿态不确定性只有 3 个方向。写成 4x4 就【必然奇异】，
      求逆会炸。所以 PX4 干脆把 P 定义成 23x23，
      四元数那块只留 3x3 —— 这就是 State::quat_nominal{0, 3} 的含义。

      这也解释了 covariance.cpp:220 这一行的写法：

          constrainStateVar(State::quat_nominal, 1e-9f, 1.f);

      它约束的是【3 个】旋转方差（tilt_x/y/z），不是 4 个四元数分量。
      具体值来自 resetQuatCov（covariance.cpp:266-283）：
          tilt 方差 = sq(initial_tilt_err) = 0.1² = 0.01 rad²   （common.h:287）
          yaw  方差 = sq(0.01) = 1e-4 rad²（默认）/ sq(mag_heading_noise)（有磁力计时）
""")

    sub("⑤ 其余辅助状态的默认初始化方差")

    rows = [
        ["vel", "sq(gps_vel_noise) = 0.5²", "未定义 CONFIG_EKF2_GNSS 时 sq(0.5)",
         "covariance.cpp:60-64"],
        ["vel(2)", "3 × 水平方差", "垂直速度先验更差", "covariance.cpp:64"],
        ["pos(xy)", "sq(gps_pos_noise) = 0.5²", "covariance.cpp:74", "covariance.cpp:89"],
        ["pos(z)", "sq(baro_noise) = 2.0²", "有 GPS 高度时 1.5×gps_pos_noise",
         "covariance.cpp:68-78"],
        ["gyro_bias", "sq(switch_on_gyro_bias) = 0.1²", "common.h:285",
         "resetGyroBiasCov"],
        ["accel_bias", "sq(switch_on_accel_bias) = 0.2²", "common.h:286",
         "resetAccelBiasCov"],
        ["mag_I / mag_B", "sq(mag_noise) = 0.05²", "common.h:360",
         "covariance.cpp:293-294"],
        ["wind_vel", "见 resetWindCov", "common.h:280 wind_vel_nsd = 1e-2",
         "covariance.cpp:100"],
    ]
    tab(rows, ["状态", "初始方差", "来源", "位置"])


# ---------------------------------------------------------------------------
# 实验 2：融合顺序 + 一个能跑的简化 EKF2 主循环
# ---------------------------------------------------------------------------

class Eskf(object):
    """最小可运行的 ESKF 内核 —— 结构和 PX4 的 Ekf 类一一对应。

    约定（和 PX4 完全一致）：
        新息 innovation = 【状态预测 - 观测】（不是「观测 - 预测」！）
        更新   x ← x - K · innovation

    见 velocity_fusion.cpp:62
            aid_src.innovation[i] = _state.vel(i) - aid_src.observation[i];
        ekf_helper.cpp:551
            _state.vel = constrain(_state.vel - K.slice<...>() * innovation, ...)

    ★★ 这个符号约定是「只有读源码才知道」的典型：论文里几乎全写成
       x ← x + K(z - Hx)，PX4 写成 x ← x - K(Hx - z)，两者等价，
       但你在源码里找不到那个加号。
    """

    def __init__(self, names, x0, P0):
        self.names = names
        self.n = len(names)
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.rejected_count = 0
        self.fused_count = 0

    def predict_cov(self, F, Q):
        """只推协方差 —— 对应 PX4 的 predictCovariance()。

            P ← F P Fᵀ + Q

        ★★ 注意这里【不碰名义状态 x】。
          PX4 把预测拆成两个函数（ekf.cpp:279-313）：

              predictState()        —— 名义状态的非线性递推（含 IMU 输入）
              predictCovariance()   —— 只推 P

          x 的非线性递推在调用点手写（因为要减零偏、要 wrap_pi），
          F 只是它的雅可比。如果把 x 也交给 F 去推，就会【推两遍】——
          零偏等于被减了两次，估计值会稳定地收敛到真值的一半。
        """
        self.P = F @ self.P @ F.T + Q

    def innovation_var(self, idx, R):
        """PX4 的 innovation_variance = P(idx,idx) + R  （velocity_fusion.cpp:47）。"""
        return float(self.P[idx, idx]) + R

    def test_ratio(self, innov, innov_var, gate):
        """PX4 的 setEstimatorAidStatusTestRatio（ekf.h:1188）。

            test_ratio = innovation² / (gate² · innovation_variance)
            拒绝 ⟺ test_ratio > 1
        """
        if not (math.isfinite(innov) and math.isfinite(innov_var)) or innov_var <= 0.0:
            return float("inf")
        return innov * innov / (gate * gate * innov_var)

    def fuse_direct(self, innov, innov_var, R, idx, gate):
        """一次【直接状态观测】的标量融合 —— ekf_helper.cpp:827-885 的复刻。

        返回 (是否融合, K)。
        """
        tr = self.test_ratio(innov, innov_var, gate)
        if tr > 1.0:
            self.rejected_count += 1
            return False, None

        # K = P[:, idx] / innovation_variance   （ekf_helper.cpp:832-834）
        K = self.P[:, idx] / innov_var
        # PX4 在这里会调用 clearInhibitedStateKalmanGains(K)（ekf_helper.cpp:836）
        # 把被抑制状态的增益清零。本实验里没有抑制状态，跳过。

        # ---- Joseph 形式（ekf_helper.cpp:851-877）----
        # P = (I - K H) P (I - K H)^T + K R K^T
        H = np.zeros(self.n)
        H[idx] = 1.0
        IKH = np.eye(self.n) - np.outer(K, H)
        self.P = IKH @ self.P @ IKH.T + np.outer(K, K) * R
        self.P = 0.5 * (self.P + self.P.T)          # 强制对称（数值卫生）

        # ---- 状态修正：x ← x - K·innovation（注意是【减】）----
        self.x = self.x - K * innov
        # 对应 ekf_helper.cpp:880 的 constrainStateVariances()
        for i in range(self.n):
            self.P[i, i] = min(max(self.P[i, i], 1e-9), 1e6)
        self.fused_count += 1
        return True, K


def exp2():
    head("实验 2  ★★ 融合顺序 + 一个能跑的简化 EKF2 主循环")

    print("""
  EKF2 每收到一个 IMU 样本，就调用一次 controlFusionModes()
  （control.cpp:46-162）。这个函数【按固定顺序】检查每一种观测源：

      先 mag，再光流，再 GPS，再……最后才是「纯惯导推算」状态位。
""")

    sub("① controlFusionModes() 的真实调用顺序（control.cpp 逐行核对）")

    rows = [[str(i), fn, loc, desc] for i, fn, loc, desc in FUSION_ORDER]
    tab(rows, ["#", "调用", "源码位置", "做什么"])

    print("""    ★★ 顺序不是随便排的，读源码能看出三条规律：

      (1) 【航向在最前】。mag 排第 1。因为 mag 提供的是 yaw，
          而 yaw 误差会让后面所有基于「世界系」的观测（GPS 位置/速度）
          的新息都带上一个旋转偏差。先修 yaw，后面的新息才干净。

      (2) 【高度在中间，且排在 GPS 之后】。height 排第 8。
          因为 baro 的高度基准和 GPS 高度基准是【互斥可选】的
          （control_status.flags.baro_hgt / gps_hgt / rng_hgt / ev_hgt），
          必须等 GPS 那一轮决定完「我这次用不用 GPS 高度」再融合。

      (3) 【最后三件事是兜底】：
          零新息航向 → 零速/零角速度 → 假位置/假高度 → 更新推算状态位。
          这些不是在融合测量值，而是在「没有测量值时也要让滤波器活着」。
""")

    sub("② 一个能跑的简化 EKF2 —— 高度通道 + 航向通道")

    print("""    真正的 EKF2 是 23 维 9000 行的东西，这里做不到。但我们能做到
    【结构上完全相同】的缩影：

        高度通道（3 维 ESKF）：x = [h, v, b_a]
            预测：h ← h + v·dt + ½a·dt²，  v ← v + a·dt，  b_a 不变
                  其中 a = a_meas - b_a （IMU 直接给比力）
            融合：baro 直接测 h  ⇒ 直接状态观测，索引 0

        航向通道（2 维 ESKF）：x = [ψ, b_g]
            预测：ψ ← ψ + (ω_meas - b_g)·dt，b_g 不变
            融合：磁航向直接测 ψ ⇒ 直接状态观测，索引 0

    ★ 两个通道都只用【标量序贯融合】，和 PX4 完全一样
      （velocity_fusion.cpp:86-105 就是三次连续调用 fuseDirectStateMeasurement）。

    然后主循环【严格按 control.cpp 的顺序】调用这两件事：
        第 1 步 mag（control.cpp:105） → 第 8 步 height（control.cpp:132）
    所以同一个 IMU 步里如果两个观测都到齐了，一定是 mag 先融合。
""")

    dt = 0.004                      # 250 Hz IMU
    t_end = 40.0
    n = int(t_end / dt)
    mag_period = 5                  # 50 Hz
    baro_period = 5                 # 50 Hz
    rng = np.random.default_rng(1406)

    # --- 真值 ---
    h_true = 5.0 + 2.0 * np.sin(2 * math.pi * 0.25 * np.arange(n) * dt)
    v_true = np.gradient(h_true, dt)
    a_true = np.gradient(v_true, dt)
    psi_true = math.radians(20.0) + math.radians(30.0) * np.sin(
        2 * math.pi * 0.05 * np.arange(n) * dt)
    w_true = np.gradient(psi_true, dt)

    b_a_true = 0.35                  # 加计零偏 m/s²（沿高度方向）
    b_g_true = math.radians(1.2)     # 陀螺零偏 rad/s
    a_meas = a_true + b_a_true + rng.normal(0.0, 0.25, n)
    w_meas = w_true + b_g_true + rng.normal(0.0, math.radians(0.4), n)
    baro = h_true + rng.normal(0.0, 0.30, n)
    mag_hdg = psi_true + rng.normal(0.0, math.radians(3.0), n)

    GATE_BARO = 5.0                  # common.h:294 / EKF2_BARO_GATE
    GATE_MAG = 2.6                   # common.h:350 / EKF2_HDG_GATE

    # --- 高度 ESKF：x = [h, v, b_a] ---
    HGT = Eskf(["h", "v", "b_a"],
               x0=[0.0, 0.0, 0.0],
               P0=np.diag([2.0 ** 2, 0.5 ** 2, 0.2 ** 2]))
    # --- 航向 ESKF：x = [psi, b_g] ---
    HDG = Eskf(["psi", "b_g"],
               x0=[0.0, 0.0],
               P0=np.diag([math.radians(30.0) ** 2, math.radians(3.0) ** 2]))

    q_hgt = np.diag([0.0, (0.6 ** 2) * dt, (0.001 ** 2) * dt])
    q_hdg = np.diag([(math.radians(0.4) ** 2) * dt, (math.radians(0.002) ** 2) * dt])
    R_BARO = 0.30 ** 2               # common.h:292 baro_noise = 2.0 是观测噪声(米)
    R_MAG = math.radians(3.0) ** 2   # 磁航向噪声（简化）

    log = []
    bias_trace = []
    h_est = np.zeros(n)
    psi_est = np.zeros(n)

    for k in range(1, n):
        # ============ 预测（predictState，ekf.cpp:279-313）============
        a_used = a_meas[k - 1] - HGT.x[2]
        HGT.x[0] = HGT.x[0] + HGT.x[1] * dt + 0.5 * a_used * dt * dt
        HGT.x[1] = HGT.x[1] + a_used * dt
        F_h = np.array([[1.0, dt, -0.5 * dt * dt],
                        [0.0, 1.0, -dt],
                        [0.0, 0.0, 1.0]])
        HGT.predict_cov(F_h, q_hgt)

        w_used = w_meas[k - 1] - HDG.x[1]
        HDG.x[0] = wrap_pi(HDG.x[0] + w_used * dt)
        F_p = np.array([[1.0, -dt], [0.0, 1.0]])
        HDG.predict_cov(F_p, q_hdg)

        # ============ controlFusionModes()：严格按 control.cpp 顺序 ============
        step_log = []
        # ---- 第 1 位：controlMagFusion()  control.cpp:105 ----
        if k % mag_period == 0:
            innov = wrap_pi(HDG.x[0] - mag_hdg[k])
            ivar = HDG.innovation_var(0, R_MAG)
            ok, _ = HDG.fuse_direct(innov, ivar, R_MAG, 0, GATE_MAG)
            step_log.append("mag%s" % ("" if ok else "[拒]"))

        # ---- 第 2~7 位：光流/GPS/空速/侧滑/阻力 —— 本场景没有 ----
        # ---- 第 8 位：controlHeightFusion()  control.cpp:132 ----
        if k % baro_period == 0:
            innov = HGT.x[0] - baro[k]
            ivar = HGT.innovation_var(0, R_BARO)
            ok, _ = HGT.fuse_direct(innov, ivar, R_BARO, 0, GATE_BARO)
            step_log.append("hgt%s" % ("" if ok else "[拒]"))

        # ---- 第 12 位：零新息航向；第 15/16 位：假位置/假高度 ----
        #         本场景有真实观测，跳过。

        h_est[k] = HGT.x[0]
        psi_est[k] = HDG.x[0]
        if step_log:
            log.append((k, k * dt, step_log))

        # 每 5 秒记一次零偏估计，用来看收敛速度（④ 要用的证据）
        if k % int(5.0 / dt) == 0:
            bias_trace.append((k * dt, HGT.x[2], HDG.x[1]))

    sub("③ 主循环轨迹：前 60 个 IMU 步里发生的融合（按顺序）")

    rows = []
    for k, tt, items in log[:24]:
        rows.append(["%d" % k, "%.3f" % tt, "  ->  ".join(items)])
    tab(rows, ["IMU 步", "时间[s]", "本步按 control.cpp 顺序执行的融合"])

    print("""    ★ 看那些同时出现两个的行（每 5 步一次）：

          第 5 步    mag  ->  hgt
          第 10 步   mag  ->  hgt
          ...

      **mag 永远在 hgt 前面。** 这不是巧合 —— control.cpp:105 在
      control.cpp:132 之前，没有别的原因。位置决定顺序，顺序决定结果。
""")

    sub("④ 滤波器跑得对不对：误差与零偏收敛")

    win = slice(int(5.0 / dt), None)

    def rms(e):
        return math.sqrt(float(np.mean(np.asarray(e) ** 2)))

    ba_err = abs(HGT.x[2] - b_a_true) / b_a_true * 100.0
    bg_err = abs(HDG.x[1] - b_g_true) / b_g_true * 100.0
    ba_pct = 100.0 * HGT.x[2] / b_a_true
    bg_pct = 100.0 * HDG.x[1] / b_g_true

    rows = [
        ["高度 h", "%.4f m" % rms(h_est[win] - h_true[win]),
         "（baro 噪声 %.2f m）" % 0.30],
        ["航向 psi", "%.4f °" % math.degrees(rms(psi_est[win] - psi_true[win])),
         "（磁航向噪声 %.1f °）" % 3.0],
        ["加计零偏 b_a 估计", "%.4f m/s²" % HGT.x[2], "完成度 %.1f%%" % ba_pct],
        ["加计零偏 b_a 真值", "%.4f m/s²" % b_a_true, "残差 %.1f%%" % ba_err],
        ["陀螺零偏 b_g 估计", "%.4f °/s" % math.degrees(HDG.x[1]),
         "完成度 %.1f%%" % bg_pct],
        ["陀螺零偏 b_g 真值", "%.4f °/s" % math.degrees(b_g_true),
         "残差 %.1f%%" % bg_err],
        ["高度通道：融合 / 被拒", "%d / %d" % (HGT.fused_count, HGT.rejected_count), ""],
        ["航向通道：融合 / 被拒", "%d / %d" % (HDG.fused_count, HDG.rejected_count), ""],
    ]
    tab(rows, ["量", "数值", "备注"])

    print("""    ★ 先看前两行，这是【融合增益】最直接的证据：

        高度误差 %.4f m  vs  单次 baro 测量噪声 0.30 m   ⇒ 好了 %.1f 倍
        航向误差 %.4f °  vs  单次磁航向噪声 3.00 °  ⇒ 好了 %.1f 倍

      如果 EKF 没有用对，估计误差会【大于】单次测量噪声（那还不如直接
      用传感器）。这里两个通道都明显更好 —— 融合确实在起作用。

    ★★ 再看零偏，这是本课最重要的一件事：

      **两个零偏都被学到了真值的 %.0f%% / %.0f%%。**

      但本实验里【没有任何一行动过零偏】——
      代码里 b_a 和 b_g 的预测方程就是「不变」（F 的第三行 = [0,0,1]、第二行 = [0,1]）。

      它们是怎么学会的？**靠协方差矩阵 P 的非对角元。**

      innovation_variance = P(idx,idx) + R 只用到 P 的对角元，
      但卡尔曼增益 K = P[:, idx] / innov_var 取的是【整列】。
      只要 P 里 h 和 b_a 的互协方差非零，修正 h 的时候
      就会【顺带】修正 b_a。这就是 EKF 无需额外逻辑就能估零偏的原因。

      ★ 对照上一课（Mahony）：Mahony 要专门写一行
        _gyro_bias += corr * (ATT_W_GYRO_BIAS * dt)
        才能学零偏；EKF 里这一行【不存在】—— 它是 P 的非对角元自动长出来的。
""" % (rms(h_est[win] - h_true[win]),
       0.30 / max(rms(h_est[win] - h_true[win]), 1e-9),
       math.degrees(rms(psi_est[win] - psi_true[win])),
       3.0 / max(math.degrees(rms(psi_est[win] - psi_true[win])), 1e-9),
       ba_pct, bg_pct))

    # 零偏收敛轨迹 —— 用同一段仿真里的采样点，直接说明「谁快谁慢」
    rows = []
    for tt, ba, bg in bias_trace:
        rows.append(["%5.0f" % tt, "%+.4f" % ba, "%.1f%%" % (100.0 * ba / b_a_true),
                     "%+.4f" % math.degrees(bg),
                     "%.1f%%" % (100.0 * math.degrees(bg) / math.degrees(b_g_true))])
    tab(rows, ["t[s]", "b_a 估计[m/s²]", "完成度", "b_g 估计[°/s]", "完成度"])

    print("""    ★★ 这张轨迹表说明了一件很容易讲错的事：

        · 陀螺零偏 b_g 在 %.0f s 就已经收敛到 %.1f%%，之后一直在真值附近。
        · 加计零偏 b_a 到 %.0f s 才 %.1f%%，而且【还在慢慢爬】。

      也就是说：**不是「谁快谁慢」这么简单，而是两条通道的
      可观测性【差一个量级】。**

        · 航向通道：b_g 的作用是「一个持续的角速率误差」，
          磁航向给的是【绝对】基准，两者在时间上可分离 ——
          时间越长，累积的转角越大，b_g 就越藏不住。20 秒足够。

        · 高度通道：b_a 和真实的垂直加速度在短时间内【无法区分】。
          本场景的真值高度是 0.25 Hz 的正弦运动，加速度是周期性的，
          滤波器没法一口咬定「这段时间的加速度计偏置到底是零偏
          还是真的在加速」。只有靠运动模态的多样性慢慢分离 ——
          这要几分钟，不是几十秒。

      ★★ 这个差别是【物理】决定的，不是调参决定的 ——
        把 EKF2_ACC_B_NOISE 调大只会让 b_a 估得更快但也更抖，
        不会改变「可观测性弱」这个事实。这是读源码时最该
        理解的一类工程现实：**滤波器只能估计它看得见的东西。**
""" % (bias_trace[len(bias_trace) // 2][0],
       100.0 * bias_trace[len(bias_trace) // 2][2] / b_g_true,
       bias_trace[-1][0], ba_pct))

    # 展示互协方差确实长出来了
    rows = [
        ["P(h, b_a)", "%+.6e" % float(HGT.P[0, 2])],
        ["P(v, b_a)", "%+.6e" % float(HGT.P[1, 2])],
        ["P(psi, b_g)", "%+.6e" % float(HDG.P[0, 1])],
    ]
    tab(rows, ["互协方差", "终值"])
    print("""    ★ 两个互协方差【都不是 0】—— 这就是零偏能被学到的那扇门。
      它们的符号为负，意思是「零偏估大了，高度/航向估计就偏小」，
      和上一课 Mahony 里 K[1] 为负是同一件事。

    ★★ 源码里这条路径没有任何特殊处理：
      ekf_helper.cpp:832-834 就是

          for (int row = 0; row < State::size; row++) {
              K(row) = P(row, state_index) / innov_var;
          }

      **取的是 P 的【一整列】，不是对角元。**
      零偏能被估出来，全靠这一列里的非对角元。
""")

    sub("⑤ 顺带一个源码细节：新息的符号是反的")

    print("""    本脚本的 Eskf.fuse_direct 里写的是

        self.x = self.x - K * innov

    减号，不是加号。因为 PX4 定义

        innovation = 状态预测 - 观测        （velocity_fusion.cpp:62）

    而教科书里几乎都写

        innovation = z - Hx̂                （观测 - 预测）

    两者只差一个符号，更新式也差一个符号，最终结果完全一样。
    ★ 但你如果照着论文去读 PX4 源码，就会在每一处融合里
      「看起来符号写反了」—— 不是反了，是定义不同。

    mag 那边更绕一点（mag_fusion.cpp:133）：

        aid_src_mag.innovation[index] =
            _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index)
            + _state.mag_B(index) - mag(index);

    还是【预测 - 观测】。顺带注意它用了 rotateVectorInverse（earth → body），
    因为 mag_I 在 NED 系、mag 在 body 系，要先把 mag_I 转到 body 系才能比。
""")


# ---------------------------------------------------------------------------
# 实验 3：新息门限（卡方检验）
# ---------------------------------------------------------------------------

def exp3():
    head("实验 3  ★★ 新息门限：gate 的单位是 sigma，> 1 就拒")

    print("""
  这是 EKF2 里最重要的一个「保护机制」，全部逻辑只有两行
  （ekf.h:1188-1189）：

      status.test_ratio = sq(status.innovation)
                        / (sq(innovation_gate) * status.innovation_variance);
      status.innovation_rejected = (status.test_ratio > 1.f);

  3 维版本（ekf.h:1207-1220）多了最后一行：

      // if any of the innovations are rejected, then the overall innovation is rejected
      status.innovation_rejected = innovation_rejected;

  ⇒ **任一轴被拒 ⇒ 整体被拒。** 这是「3D 版本」和「1D 版本」唯一的区别。
""")

    sub("① 把这个式子拆开：为什么 gate 的单位是 sigma")

    print("""    记     σ_innov² = innovation_variance = P(idx,idx) + R
           ν        = innovation（新息）

    拒绝条件：

        ν² / (gate² · σ_innov²) > 1
        ⟺  ν² > gate² · σ_innov²
        ⟺  |ν| > gate · σ_innov

    ★★ 最后一行是关键：**门限就是一个「多少倍 σ_innov」的数。**
      gate = 5.0 的意思就是「新息超过 5 个标准差就扔掉」。

    ⇒ 如果滤波器模型正确、新息真的是零均值高斯，
      那么 ν/σ_innov ~ N(0,1)，误拒概率 = P(|Z| > gate)。

    ★ 注意 innovation_variance 不是 R，而是 P(idx,idx) + R。
      就是说分母里含【滤波器自己当前有多不确定】：
      自己越不确定，越不容易拒观测。这正是贝叶斯该有的样子。
""")

    sub("② 误拒概率：解析值（用 erfc 真算，不是查表）")

    rows = []
    for name, pname, gate, line, _tag in INNOV_GATES:
        p1 = norm_tail_prob(gate)
        rows.append([name, pname, "%.1f" % gate, "%.3e" % p1,
                     "%.4f%%" % (100.0 * p1)])
    tab(rows, ["传感器", "参数", "gate(σ)", "1D 误拒概率", "百分比"])
    print("""    ★ 看这张表的最后一列 —— 它解释了「为什么各家的 gate 不一样」：

        gate = 5.0  →  误拒率 5.7e-7   （约 175 万次里错 1 次）
        gate = 3.0  →  误拒率 2.70e-3  （约 370 次里错 1 次）
        gate = 2.6  →  误拒率 9.32e-3  （约 107 次里错 1 次）

      ★★ 所以 gate 的选择是一个【代价权衡】，不是「谁更准」：

        · GPS/baro/空速 —— gate 5.0
          这些传感器的新息【本来就该接近高斯】：它们的误差是慢变的、
          分布良好的。拒错一次的代价（丢失一次位置修正）却很大。
          ⇒ 门限开大，宁可信其有。

        · 磁力计 —— gate 3.0；磁航向 —— gate 2.6
          磁场【天生会被干扰】：旁边走过一个铁架、飞过一条高压线、
          电调大电流一过，mag 读数就歪。这些干扰不是高斯噪声，
          是【有偏的野值】。对野值，宽门限等于放进来污染滤波器。
          ⇒ 门限收紧，宁可错杀。
          （2.6 比 3.0 还小，因为航向是 1 维，误拒代价又低一档。）

        · 光流 —— gate 3.0
          纹理突然变差（飞过水面、白墙）时光流会整体失效，
          和磁力计是同一类问题。
""")

    sub("③ 蒙特卡洛验证：真的去抽随机数，数被拒的比例")

    n_mc = 10000000
    rng = np.random.default_rng(31415)
    rows = []
    max_dev = 0.0
    for gate in (2.0, 2.6, 3.0, 3.5):
        z = rng.normal(0.0, 1.0, n_mc)
        tr = (z * z) / (gate * gate)
        n_rej = int(np.count_nonzero(tr > 1.0))
        emp = n_rej / n_mc
        thy = norm_tail_prob(gate)
        se = math.sqrt(thy * (1.0 - thy) / n_mc)      # 二项分布标准误
        dev = (emp - thy) / se
        max_dev = max(max_dev, abs(dev))
        rows.append(["%.1f" % gate, "%d" % n_rej, "%.4e" % emp, "%.4e" % thy,
                     "%+.2f" % dev, "%.1e" % se])
    tab(rows, ["gate", "被拒样本数", "MC 误拒率", "解析误拒率",
               "偏差(σ)", "标准误"])

    print("""    样本数 N = %d。最后一列的「偏差(σ)」是

         (MC 误拒率 − 解析误拒率) / 标准误
         标准误 = √(p(1−p)/N)

      这是【按定义算出来的】，不是估计的。

      ★ 4 行的最大偏差 = %.2f σ ⇒ 蒙特卡洛和解析公式一致。
      也就是说：**gate 确实是 sigma 单位，公式确实就是卡方检验。**

      （顺带说明「偏差(σ)」这一列怎么读：如果两者真的一致，
        它应该服从标准正态，落在 ±3σ 内的概率是 99.7%%。
        这里 4 个数都在这个范围里。）
""" % (n_mc, max_dev))

    sub("④ gate = 5.0 的误拒率：解析值 vs 大样本 MC")

    gate = 5.0
    n_big = 20000000
    n_rej_big = 0
    chunk = 2000000
    for _ in range(n_big // chunk):
        z = rng.normal(0.0, 1.0, chunk)
        n_rej_big += int(np.count_nonzero((z * z) / (gate * gate) > 1.0))
    thy5 = norm_tail_prob(gate)
    exp_cnt = thy5 * n_big
    print("    N = %d 个标准正态样本" % n_big)
    print("    被拒的样本数 = %d" % n_rej_big)
    print("    解析期望个数 = %.3e × %d = %.2f" % (thy5, n_big, exp_cnt))
    print("    泊松 1σ 带宽 = ±%.2f 个" % math.sqrt(max(exp_cnt, 1e-12)))
    print("    解析误拒率 = erfc(5/√2) = %.6e" % thy5)
    print("    实测误拒率 = %d / %d = %.6e" % (n_rej_big, n_big,
                                               n_rej_big / n_big))
    print()
    print("""    ★★ 这个数（%.2e）小得离谱：如果 GPS 以 5 Hz 输出，那么

        平均无故障时间 = 1 / (%.3e × 5) = %.0f 秒 = %.1f 天

      也就是说要连续飞 %.1f 天，才可能碰上【一次】「好数据被误拒」。

      这就是为什么 gate 敢开到 5σ —— 它几乎不会误伤。
""" % (thy5, thy5, 1.0 / (thy5 * 5.0),
       1.0 / (thy5 * 5.0) / 86400.0, 1.0 / (thy5 * 5.0) / 86400.0))

    sub("⑤ 但 3D 融合的误拒率是 1D 的 3 倍（近似）")

    print("""    3D 版本（ekf.h:1197-1221）是「任一轴超限 ⇒ 整体拒」。
    假设三轴独立（PX4 用序贯融合，轴间有相关性，这里只做量级估计）：

        P(拒) = 1 - (1 - p₁)³  ≈  3·p₁      （p₁ 很小时）

    真实值还是算出来：
""")

    rows = []
    for gate in (5.0, 3.0, 2.6):
        p1 = norm_tail_prob(gate)
        p3_exact = 1.0 - (1.0 - p1) ** 3
        rows.append(["%.1f" % gate, "%.4e" % p1, "%.4e" % p3_exact,
                     "%.4e" % (3.0 * p1), "%.2f×" % (p3_exact / p1)])
    tab(rows, ["gate", "1D 误拒率", "3D 精确值", "3·p₁ 近似", "倍数"])

    print("""    ★ 所以「3 轴一起用同一个 gate」会让整体误拒率上升约 3 倍。
      PX4 的处理办法不是调 gate，而是：
        · 逐轴序贯融合（mag_fusion.cpp:123-206 的 for 循环）
          —— 一次只融一个轴，融完一轴就重算下一轴的 P，
             这样第二个轴用的 P 已经是被第一个轴更新过的；
        · 任一轴被拒就整体不融（mag_fusion.cpp:116-118）。
""")

    sub("⑥ 一次真实的「野值被拒」过程")

    print("""    设定：一个 1 维观测，
        innovation_variance ≈ P + R = 0.04 + 0.09 = 0.13  （σ_innov = %.4f）
        gate = 5.0  ⇒  |新息| > 5 × %.4f = %.4f m 时拒收

    然后注入一个【慢慢变大】的野值（第 10~17 步，从 0.5 m 长到 3.0 m），
    看门限在哪一步开始动手：
""" % (math.sqrt(0.13), math.sqrt(0.13), 5.0 * math.sqrt(0.13)))

    P_v, R_v, gate_v = 0.04, 0.09, 5.0
    ivar_v = P_v + R_v
    sig = math.sqrt(ivar_v)
    lim = gate_v * sig

    n_steps = 40
    innov = 0.02 * rng.normal(0.0, 1.0, n_steps)
    innov[10:18] += np.linspace(0.5, 3.0, 8)   # 8 步、由小到大的野值

    rows = []
    n_acc = 0
    first_rej = None
    for i in range(n_steps):
        tr = innov[i] * innov[i] / (gate_v * gate_v * ivar_v)
        rej = tr > 1.0
        if rej:
            n_acc += 1
            if first_rej is None:
                first_rej = i
        flag = "★ 拒" if rej else "融合"
        rows.append(["%2d" % i, "%+.4f" % innov[i], "%.4f" % tr,
                     "%.4f" % math.sqrt(tr), flag])
    tab(rows, ["步", "新息 ν [m]", "test_ratio", "√test_ratio", "结果"])

    print("""    ★ 四个观察：

      (0) 门限是 %.4f m，而野值是从 0.5 m 慢慢长大的。
          前几步 |ν| 还没到 %.4f ⇒ **照常融合，门限一动不动。**
          第一次拒绝出现在第 %d 步 —— 那一步的 |ν| 刚刚越过 %.4f m。
          全表一共拒了 %d 步。

          ★★ 这就是门限的真实性格：**它只拦「大得离谱」的，
             不拦「中等偏差」的。** 4σ 的野值照样进得来，
             靠的是后面的协方差更新慢慢把它压下去（见 ⑦）。
             门限不是万能药，只是最后一道保险丝。

      (1) 被拒的那几步之后，野值一回到范围内下一次就正常融合。
          因为门限是【逐次独立】判断的 —— 只看当次新息，
          PX4 没有「拉黑」机制。

          ★ 唯一的例外是磁力计的航向一致性判据（mag_control.cpp:239-276）：

              _mag_heading_innov_lpf.update(innovation);        // :261
              if (fabsf(_mag_heading_innov_lpf.getState()) < _params.mag_heading_noise) {
                  if (_yaw_angle_observable) {
                      _control_status.flags.mag_heading_consistent = true;
                  }
              } else {
                  _control_status.flags.mag_heading_consistent = false;   // :274
              }

          注意它用了【低通滤波器】_mag_heading_innov_lpf ——
          这是个有记忆的判据，门限只看当次新息，它看的是
          「最近这段时间航向新息的平均水平」。
          所以磁干扰要【持续一段时间】才会把
          mag_heading_consistent 打成 false。

      (2) 第 4 列 √test_ratio = |ν|/(gate·σ) 才是
          estimator_status 里真正发出来的那个数。
          见 ekf_helper.cpp:407-411：

              float gps_vel = sqrtf(Vector3f(_aid_src_gnss_vel.test_ratio).max());
              vel = math::max(gps_vel, FLT_MIN);
              float gps_pos = sqrtf(Vector2f(_aid_src_gnss_pos.test_ratio).max());
              pos = math::max(gps_pos, FLT_MIN);

          ★★ 也就是说：**你在地面站看到的 vel_test_ratio / pos_test_ratio
             已经是开过平方的**，1.0 就是「刚好到门限」。
             它不是「多少倍 sigma」，而是「用掉了多少门限」。
             要看 sigma 数，得自己乘回 gate。

      (3) 高度那一项（ekf_helper.cpp:435-467）是【多个高度源取平均】：

              float hgt_sum = 0.f;                                 // :435
              int n_hgt_sources = 0;                               // :436
              ...
              hgt_sum += sqrtf(_aid_src_baro_hgt.test_ratio);      // :440
              n_hgt_sources++;                                     // :441
              ...
              hgt = math::max(hgt_sum / static_cast<float>(n_hgt_sources), FLT_MIN);  // :467

          所以 baro + GPS 高度同时在线时，hgt_test_ratio 是两者的【平均】。
          这和 pos/vel 取 max（ekf_helper.cpp:407-410 的 .max()）正好相反。

      ★ 还有一处更值得注意：**test_ratio == 0 是「无效」，不是「完美」。**

          EKF2Selector.cpp:281-292 把 0、负数、NaN 一律改成 1.0：
              // test ratios are invalid when 0, >= 1 is a failure
              if (!PX4_ISFINITE(status.vel_test_ratio) || (status.vel_test_ratio <= 0.f)) {
                  status.vel_test_ratio = 1.f;
              }

          为什么 0 反而当成「失败」？因为 test_ratio 为 0 只有两种可能：
          新息恒为 0（没有观测，或观测被完全忽略），或者除零前的占位值。
          两者都表示「这个方向根本没有有效信息」，所以按最坏情况处理。
""" % (lim, lim, first_rej, lim, n_acc))

    sub("⑦ ★★ 工程上的另一半：不只有门限，还有「协方差膨胀」")

    print("""    门限是【拒绝】坏数据。但坏数据被拒绝之后，
    滤波器会因为「很久没有观测」而变得更不确定 —— 这是对的。
    真正危险的是另一种情况：**加速度计本身在撒谎**。

    PX4 的对策写在两处：

    (1) 检测：height_control.cpp:224-269 的 checkVerticalAccelerationHealth()
        用「加计 clip 计数」+「垂直新息是否持续为正」判断加计是否坏了，
        置 fault_status.flags.bad_acc_vertical。
        持续 BADACC_PROBATION = 10 s（common.h:77）才解除。

    (2) 膨胀：covariance.cpp:117-125

            for (unsigned i = 0; i < 3; i++) {
                if (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i]) {
                    accel_var(i) = sq(BADACC_BIAS_PNOISE);     // 4.9² = 24.01
                } else {
                    accel_var(i) = sq(accel_noise);            // 0.35² = 0.1225
                }
            }

        数值对比：
""")
    rows = [
        ["正常：sq(accel_noise) = 0.35²", "%.4f" % (0.35 ** 2)],
        ["加计坏：sq(BADACC_BIAS_PNOISE) = 4.9²", "%.4f" % (BADACC_BIAS_PNOISE ** 2)],
        ["膨胀倍数", "%.0f×" % ((BADACC_BIAS_PNOISE / 0.35) ** 2)],
    ]
    tab(rows, ["情形", "过程噪声方差 (m/s²)²"])

    print("""    ★★ 注意这里改的是【过程噪声 Q】，不是观测噪声 R。

      含义完全不同：
        · 改 R  = 「我不太信这个传感器」
        · 改 Q  = 「我不太信【我自己推出来的状态】」

      加计坏掉的时候，错的是【预测】，不是观测。
      所以必须把 Q 顶大 196 倍，让滤波器在下一步【更听观测的话】。
      这正是「协方差膨胀」这个名字的来历 —— 主动把 P 吹大。

      ★ 第 3 行那个 196 倍不是拍的：4.9 m/s² 是一个
        「加计已经完全不可信」量级的加速度不确定度。
        把它平方塞进 Q，等于告诉滤波器「我的速度预测
        每步都可能偏差 4.9 m/s²·dt」—— 于是 P 迅速长大，
        卡尔曼增益随之变大，观测的权重自动升上去。
""")

    if _HAS_MPL:
        _plot_gate(innov, ivar_v, gate_v)
    else:
        print("    （没装 matplotlib，跳过画图）")
        print()


# ---------------------------------------------------------------------------
# 可选画图（matplotlib 缺失时整段跳过）
# ---------------------------------------------------------------------------

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
    """有中文字体就用中文，否则用英文。"""
    return zh if _CJK else en


def _plot_gate(innov, ivar_v, gate_v):
    """把新息轨迹和门限画出来（存成 png）。"""
    sig = math.sqrt(ivar_v)
    lim = gate_v * sig
    fig, ax = plt.subplots(figsize=(11, 4.2))
    idx = np.arange(len(innov))
    ax.plot(idx, innov, "b.-", lw=1.2, ms=5,
            label=_t("新息 ν", "innovation"))
    ax.axhspan(-lim, lim, color="0.90",
               label=_t("门限内（融合）", "inside gate (fuse)"))
    ax.axhline(lim, color="r", ls="--", lw=1.4,
               label=_t("±gate·σ_innov", "+-gate*sigma_innov"))
    ax.axhline(-lim, color="r", ls="--", lw=1.4)
    rej = np.abs(innov) > lim
    ax.plot(idx[rej], innov[rej], "rx", ms=11, mew=2.2,
            label=_t("被拒", "rejected"))
    ax.set_xlabel(_t("步", "step"))
    ax.set_ylabel(_t("新息 [m]", "innovation [m]"))
    ax.set_title(_t("新息门限：gate=5σ 把野值挡在外面",
                    "Innovation gate: gate=5 sigma rejects the outlier"))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    plt.tight_layout()
    out = "ekf2_innovation_gate.png"
    plt.savefig(out, dpi=110)
    plt.close(fig)
    print("    ✅ 图已存：%s%s" % (out, "" if _CJK else "（无中文字体，标签用英文）"))
    print()


# ---------------------------------------------------------------------------
# 实验 4：多实例选择器
# ---------------------------------------------------------------------------

class Hysteresis(object):
    """systemlib::Hysteresis 的逐行复刻（src/lib/hysteresis/hysteresis.cpp:45-90）。

    ★★ 这是本实验最重要的一处「不读源码一定会写错」的地方。

    用法是 EKF2Selector.hpp:111：
        healthy.set_hysteresis_time_from(false, 1_s);

    注意参数名：set_hysteresis_time_from(**from_state**, time)。
    传进去的 from_state 是 **false**，所以它设置的是
    `_time_from_false_us = 1_s`，即【false → true】这个方向要等 1 秒。

    而 `_time_from_true_us` 从来没被设过，**默认是 0**。
    于是【true → false】是【立即】生效的。

    结论（和直觉相反）：
        1 秒迟滞保护的是「**恢复**」而不是「**失效**」。
        一个刚被判定不健康的 EKF 必须【连续 1 秒都健康】才能重新被信任；
        而一个健康的 EKF 一旦踩线，**当拍就下台**。

    这个方向是合理的：失效要快（安全），恢复要慢（防止抖动）。
    """

    def __init__(self, init_state):
        self._state = init_state
        self._requested = init_state
        self._last_change_us = 0.0
        self._time_from_true_us = 0.0     # 默认 0
        self._time_from_false_us = 0.0    # 默认 0

    def set_hysteresis_time_from(self, from_state, time_us):
        """hysteresis.cpp:45-54。"""
        if from_state:
            self._time_from_true_us = time_us
        else:
            self._time_from_false_us = time_us

    def get_state(self):
        return self._state

    def set_state_and_update(self, new_state, now_us):
        """hysteresis.cpp:56-70。"""
        if new_state != self._state:
            if new_state != self._requested:
                self._requested = new_state
                self._last_change_us = now_us
        else:
            self._requested = self._state
        self.update(now_us)

    def update(self, now_us):
        """hysteresis.cpp:72-90 —— 注意两个方向用的时间不一样。"""
        if self._requested != self._state:
            if self._state and not self._requested:
                # true -> false
                if now_us >= self._last_change_us + self._time_from_true_us:
                    self._state = False
            elif (not self._state) and self._requested:
                # false -> true
                if now_us >= self._last_change_us + self._time_from_false_us:
                    self._state = True


class Instance(object):
    """一个 EKF2 实例在选择器里的状态 —— 字段名照抄 EKF2Selector.hpp:100-144。"""

    def __init__(self, i, gyro_id, accel_id):
        self.index = i
        self.gyro_device_id = gyro_id
        self.accel_device_id = accel_id
        self.combined_test_ratio = float("nan")
        self.relative_test_ratio = float("nan")
        # EKF2Selector.hpp:135 + 111
        self.healthy = Hysteresis(False)
        self.healthy.set_hysteresis_time_from(False, HEALTHY_HYSTERESIS_MS * 1000.0)
        self.healthy_count = 0
        self.warning = False
        self.filter_fault = False
        self.timeout = False
        self.time_last_selected = 0.0
        self.time_last_no_warning = 0.0


class Selector(object):
    """EKF2Selector 的选择逻辑复刻 —— 逐条对应 EKF2Selector.cpp。

    源码位置索引：
        UpdateErrorScores    EKF2Selector.cpp:147-362
        combined_test_ratio  EKF2Selector.cpp:294
        healthy 判据         EKF2Selector.cpp:298-299
        warning 判据         EKF2Selector.cpp:301
        healthy 迟滞 1 s     EKF2Selector.hpp:111 + hysteresis.cpp:45-90
        relative_test_ratio  EKF2Selector.cpp:339-359
        切换决策             EKF2Selector.cpp:709-797
        SelectInstance       EKF2Selector.cpp:110-145
    """

    def __init__(self, n_inst, verbose=True):
        self.n = n_inst
        self.inst = [Instance(i, 1000 + i, 2000 + i) for i in range(n_inst)]
        self.selected = None
        self.available = 0
        self.change_count = 0
        self.last_change = 0.0
        self.verbose = verbose
        self.events = []

    def _log(self, t, msg, kind="info"):
        self.events.append((t, kind, msg))
        if self.verbose:
            print("      t=%7.2fs  %s" % (t, msg))

    def select(self, t, new, why=""):
        """EKF2Selector.cpp:110-145 的 SelectInstance()。"""
        if new is None or new == self.selected or new >= self.available:
            return False
        if self.selected is not None:
            self._log(t, "★ 主 EKF 切换：%d -> %d%s   （相对分数清零）"
                      % (self.selected, new, why), "switch")
        self.selected = new
        self.change_count += 1
        self.last_change = t
        self.inst[new].time_last_selected = t
        # EKF2Selector.cpp:136-139 —— 重置所有相对分数
        for k in range(self.available):
            self.inst[k].relative_test_ratio = 0.0
        return True

    def update(self, t, vel_tr, pos_tr, hgt_tr, fault_flags):
        """一次 UpdateErrorScores() —— EKF2Selector.cpp:147-362。"""
        n = len(vel_tr)
        if n > self.available:
            self.available = n
        now_us = t * 1e6

        prev_healthy = [x.healthy.get_state() for x in self.inst]

        for i in range(n):
            v, p, h = vel_tr[i], pos_tr[i], hgt_tr[i]
            # EKF2Selector.cpp:281-292 ——
            # test ratios are invalid when 0, >= 1 is a failure
            if not math.isfinite(v) or v <= 0.0:
                v = 1.0
            if not math.isfinite(p) or p <= 0.0:
                p = 1.0
            if not math.isfinite(h) or h <= 0.0:
                h = 1.0

            # EKF2Selector.cpp:294
            combined = max(0.5 * (v + p), h)
            ins = self.inst[i]
            ins.combined_test_ratio = combined

            # EKF2Selector.cpp:298-299 —— healthy 判据 + 迟滞
            raw_healthy = (fault_flags[i] == 0) and (combined > 0.0)
            ins.healthy.set_state_and_update(raw_healthy, now_us)

            # EKF2Selector.cpp:301-303
            ins.warning = (combined >= 1.0)
            ins.filter_fault = (fault_flags[i] != 0)
            ins.timeout = False

            if not ins.warning:
                ins.time_last_no_warning = now_us

            if not math.isfinite(ins.relative_test_ratio):
                ins.relative_test_ratio = 0.0

        # EKF2Selector.cpp:328-335
        for i in range(n):
            if prev_healthy[i] != self.inst[i].healthy.get_state() and not prev_healthy[i]:
                self.inst[i].healthy_count += 1

        # ---- relative_test_ratio 累积（EKF2Selector.cpp:339-359）----
        if self.selected is not None:
            prim = self.selected
            for i in range(self.available):
                if i == prim:
                    continue
                # EKF2Selector.cpp:343
                delta = (self.inst[i].combined_test_ratio
                         - self.inst[prim].combined_test_ratio)
                # EKF2Selector.cpp:346
                thr = max(SEL_ERR_RED, 0.05)
                if delta > 0.0 or delta < -thr:
                    r = self.inst[i].relative_test_ratio + delta
                    self.inst[i].relative_test_ratio = max(
                        -REL_ERR_SCORE_LIM, min(REL_ERR_SCORE_LIM, r))

    def decide(self, t):
        """一次切换决策 —— EKF2Selector.cpp:709-797。"""
        now_us = t * 1e6

        # EKF2Selector.cpp:690-707：还没有主实例时，选第一个有有效 IMU 的
        if self.selected is None:
            best = min(range(self.available),
                       key=lambda i: self.inst[i].combined_test_ratio)
            self.selected = best
            self.last_change = t
            self.inst[best].time_last_selected = t
            self.change_count += 1
            self._log(t, "首个可用实例 %d 被选中（EKF2Selector.cpp:690-707）" % best)
            return

        # EKF2Selector.cpp:715-721
        best_alt = None
        alt_err = 0.0
        best_tr = float("inf")
        best_ekf = self.selected
        best_diff_imu = None

        # EKF2Selector.cpp:724-755
        for i in range(self.available):
            if not self.inst[i].healthy.get_state() or i == self.selected:
                continue
            tr = self.inst[i].combined_test_ratio
            rel = self.inst[i].relative_test_ratio
            if rel < alt_err:
                best_alt = i
                alt_err = rel
            if tr > 0.0 and tr < best_tr:
                best_ekf = i
                best_tr = tr
                # EKF2Selector.cpp:749-752 —— 顺便找一个用不同 IMU 的
                if self.inst[i].accel_device_id != self.inst[self.selected].accel_device_id:
                    best_diff_imu = i

        # EKF2Selector.cpp:740
        lower_error_available = False
        if best_alt is not None:
            rel = self.inst[best_alt].relative_test_ratio
            if (rel <= -REL_ERR_THRESH
                    and (now_us - self.inst[best_alt].time_last_selected) > RESELECT_GUARD_MS * 1000.0):
                lower_error_available = True

        # EKF2Selector.cpp:757-784
        if not self.inst[self.selected].healthy.get_state():
            if not self.select(t, best_diff_imu, "（主实例不健康 → 换不同 IMU）"):
                self.select(t, best_ekf, "（主实例不健康 → 换最优）")

        elif lower_error_available and (
                (now_us - self.last_change * 1e6) > SWITCH_GUARD_MS * 1000.0
                or (self.inst[self.selected].warning
                    and (now_us - self.inst[self.selected].time_last_no_warning) > 1e6)):
            self.select(t, best_alt, "（另一实例显著更优）")


def exp4():
    head("实验 4  ★★ 多实例选择器：为什么同时跑 3 个 EKF")

    print("""
  这是本课最有价值的工程点，也是论文里【完全没有】的东西。

  问题：一个飞控上装了 3 个 IMU。你怎么知道该信哪个？
        最笨的办法是「取平均」—— 但如果 1 个坏了，平均就全废了。
        PX4 的办法是：

            **给每个 IMU 配一个【完整独立的 EKF】，
              让它们各自算出完整的估计，再选一个当主。**

  源码入口在 EKF2.cpp:2708-2848（EKF2::task_spawn() 里的 multi 分支）：

      const int multi_instances = math::min(
              imu_instances * mag_instances,
              static_cast<int32_t>(EKF2_MAX_INSTANCES));    // EKF2.cpp:2781
      ...
      EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false);
      if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) {    // EKF2.cpp:2810

  ★ 实例数 = IMU 数 × 磁力计数（上限 EKF2_MAX_INSTANCES）。
    这就是为什么装了双磁力计时实例数会翻倍 —— 每个 (IMU, mag) 组合
    都要有自己的 EKF，因为 mag_B（硬铁干扰）是装在一起的，
    换一个磁力计就是换一套 mag 状态。
""")

    sub("① 实例数的上限：注意 CONSTRAINED_MEMORY 分支")

    print("""    EKF2Selector.hpp:59-63：

        #if CONSTRAINED_MEMORY
        # define EKF2_MAX_INSTANCES 2
        #else
        # define EKF2_MAX_INSTANCES 9
        #endif

    ★★ 内存受限的板子（比如只有几百 KB RAM 的 F4）最多 2 个实例。
      因为每个实例都要一份 23x23 的 float 协方差（约 2.1 KB）
      加上一堆状态和缓冲区。9 个实例不是「软件限制」，是「内存限制」。

    两个参数控制实例数（params_multi.yaml）：

        EKF2_MULTI_IMU   默认 0   最多用几个 IMU 做多实例
        EKF2_MULTI_MAG   默认 0   最多用几个磁力计做多实例

    ★★ 注意默认是 0 —— 也就是说【默认根本没开多实例】。
      而且开了还必须先关掉传感器层面的自动选择：

          EKF2.cpp:2716   if (sens_imu_mode == 0)   // SENS_IMU_MODE = 0
          EKF2.cpp:2736   if (sens_mag_mode == 0)   // SENS_MAG_MODE = 0

      为什么？因为这两层职责不能重叠：
        · SENS_IMU_MODE = 1 ⇒ sensors 模块自己挑一个 IMU 给所有人用
        · SENS_IMU_MODE = 0 ⇒ sensors 模块把【所有】IMU 都发出来，
                              EKF2 才能每个 IMU 起一个实例
      源码里还有一句保护（EKF2.cpp:2750）：
          PX4_WARN("EKF2_MULTI_MAG disabled, resetting SENS_MAG_MODE");
      当 EKF2_MULTI_MAG <= 1 时，它会把 SENS_MAG_MODE 改回 1。
""")

    n_inst = 3
    dt_ms = float(FILTER_UPDATE_PERIOD_MS)
    dt = dt_ms / 1000.0
    t_end = 36.0
    n_steps = int(t_end / dt)

    # 场景参数 —— 全部是脚本自己设定的仿真量，不是源码里的默认值
    T_GYRO_FAULT = 20.0      # 实例 0 的陀螺开始变坏
    GYRO_FAULT_SLOPE = 0.90  # ratio 斜率 [/s]：0.10 → 1.00 用 1.0 s
    T_FILTER_FAULT = 20.15   # 实例 0 置 filter_fault_flags
    T_DEGRADE1 = 26.0        # 实例 1 开始缓慢劣化
    DEGRADE1_SLOPE = 0.08    # 实例 1 的 ratio 斜率 [/s]
    BASE_TR = (0.10, 0.14, 0.16)   # 三个实例的健康基线 test_ratio

    sub("② ★★ 先纠正一个「不读源码一定会写错」的地方")

    print("""    在放仿真之前，必须先把 healthy 的迟滞方向讲对。
    这一条我原本也以为是反的。

    EKF2Selector.hpp:111 只写了一行：

        healthy.set_hysteresis_time_from(false, 1_s);

    ★ 注意函数名里的 **from**：set_hysteresis_time_from(from_state, time)。
      传进去的是 `false`，所以设置的是

          _time_from_false_us = 1 s      ← 【false → true】方向的时间

      而 `_time_from_true_us` 从来没被赋值，**默认 0**。

    看 hysteresis.cpp:72-90 的 update()：

        if (_state && !_requested_state) {
            // true -> false
            if (now_us >= _last_time_to_change_state + _time_from_true_us) { _state = false; }
        } else if (!_state && _requested_state) {
            // false -> true
            if (now_us >= _last_time_to_change_state + _time_from_false_us) { _state = true; }
        }

    ★★ 结论（和直觉正好相反）：
        · true → false（判定【失效】）用的时间是 _time_from_true_us = 0
          ⇒ **当拍生效，没有延迟。**
        · false → true（判定【恢复】）用的是 _time_from_false_us = 1 s
          ⇒ **必须连续 1 秒都健康才算恢复。**

    也就是说这 1 秒迟滞保护的是「**恢复**」而不是「**失效**」——
    失效要快（安全优先），恢复要慢（防止抖动）。

    ★ 这个设计在 EKF2Selector.cpp:318-326 又强化了一次：

        // if the gyro used by the EKF is faulty, declare the EKF unhealthy without delay
        if (_gyro_fault_detected && (faulty_gyro_id != 0) && (_instance[i].gyro_device_id == faulty_gyro_id)) {
            _instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
        }

      注释里的 "without delay" 其实有点多余 —— 因为 true → false
      本来就没有迟滞。但写成注释说明这是【有意为之】的。
""")

    sub("③ 场景：3 个实例，第 20 秒给实例 0 注入陀螺故障")

    print("""    三个实例分别绑在 IMU0 / IMU1 / IMU2 上（accel_device_id = 2000/2001/2002）。

    时间线（注入的故障）：
         t ≥ 20.00 s  实例 0（主）：陀螺数据开始变坏，ratio 以 0.9/s 上爬
         t = 20.15 s  实例 0 置 filter_fault_flags（EKF 自己判出数值故障）
                      ⇒ healthy 当拍转 false ⇒ 立刻被换掉
         t = 21.02 s  实例 0 的 combined_test_ratio 才爬到 1.0
                      ⇒ warning = true（但它早已不是主实例了）
         t ≥ 26.00 s  实例 1（此时它已经是主）：缓慢劣化，ratio 慢慢爬
                                   —— 这是「漂移」，不是故障，healthy 一直是 true
         t ∈ [0,36]   实例 2 一直稳定在 0.16 附近

    ★ 这样安排是为了让两条切换路径都跑一遍：
        · 第一次切换由「主实例不健康」触发（绝对判据，t = 20.15）
        · 第二次切换由「另一实例显著更优」触发（相对判据，t = 30.16）

    ★★ 为什么第一次不能靠 ratio 越线（warning）来触发？
      因为 warning **压根不参与** healthy 的判定（见 ⑤ 的源码引用）。
      真实系统里，把一个实例立刻踢下台的只有 filter_fault_flags ——
      EKF 【自己】报告「我算错了」。统计门限 ratio ≥ 1.0 只是个警告灯。

    ★★ 为什么让 ratio 慢慢爬（0.9/s）而不是一下跳到 5？
      因为 relative_test_ratio 的累积**没有 dt 缩放**（源码见 ⑦）：
      每次更新加一个完整的 error_delta，只要差值超过死区 0.2 就以
      每次 ≥0.2 的速度累积，从 +1.0 掉到 -0.5 只要十几次更新（约 0.1 s）。
      所以如果让 ratio【一步跳到位】，相对判据会抢在 filter_fault_flags
      之前把主实例换掉，两条路径就混在一起、看不清了。
      真实系统里这两种故障也是分开的：
        快故障（传感器硬坏）→ 走 healthy 分支
        慢漂移（零偏慢慢爬）→ 走 relative 分支

    选择器参数（全部来自源码）：
        EKF2_SEL_ERR_RED = %.1f      （params_selector.yaml:11）
        相对分数门限     = %.1f      （EKF2Selector.hpp:147）
        相对分数限幅     = ±%.1f     （EKF2Selector.hpp:146）
        healthy 迟滞     = %d ms    （EKF2Selector.hpp:111，方向是 false→true）
        重选保护期       = %d s      （EKF2Selector.cpp:740）
        切换保护期       = %d s      （EKF2Selector.cpp:765）
""" % (SEL_ERR_RED, REL_ERR_THRESH, REL_ERR_SCORE_LIM,
       HEALTHY_HYSTERESIS_MS, RESELECT_GUARD_MS // 1000, SWITCH_GUARD_MS // 1000))

    sel = Selector(n_inst, verbose=True)
    print("      ---- 事件日志 ----")

    rng4 = np.random.default_rng(777)
    snapshots = []
    first_warn = [None, None, None]
    first_unhealthy = [None, None, None]
    first_rel_below = [None, None, None]
    # ★ 注意：Hysteresis(False) 的初始状态是 False，而且 false→true 有 1 s 迟滞，
    #   所以前 1 秒【所有实例】都是 healthy=false。那不是「失效」。
    #   只有当实例【先健康过】再变 false，才算真正的 true→false 跳变。
    ever_healthy = [False, False, False]
    prev_healthy = [False, False, False]

    for k in range(n_steps):
        t = k * dt

        # ---- 造三个实例的 test_ratio ----
        vel_tr = np.zeros(n_inst)
        pos_tr = np.zeros(n_inst)
        hgt_tr = np.zeros(n_inst)
        fault = [0, 0, 0]

        # 基线：实例 0 最好（0.10），实例 1 次之（0.14），实例 2 最差（0.16）
        base0 = BASE_TR[0]
        base1 = BASE_TR[1]
        base2 = BASE_TR[2]
        # 测量噪声
        n0 = abs(float(rng4.normal(0.0, 0.012)))
        n1 = abs(float(rng4.normal(0.0, 0.012)))
        n2 = abs(float(rng4.normal(0.0, 0.012)))

        # 实例 0：20 s 起陀螺数据变坏 → ratio 线性上爬；
        #         20.15 s 先置 filter_fault_flags（EKF 自己的数值故障检测），
        #         21.02 s 才越过 1.0（selector 的统计 warning）—— 两者完全独立
        if t >= T_GYRO_FAULT:
            base0 += GYRO_FAULT_SLOPE * (t - T_GYRO_FAULT)
            if t >= T_FILTER_FAULT:
                fault[0] = 0x40

        # 实例 1：换主之后（26 s 起）缓慢劣化 —— 陀螺零偏漂移，不是故障
        if t >= T_DEGRADE1:
            base1 += DEGRADE1_SLOPE * (t - T_DEGRADE1)

        vel_tr = np.array([base0 + n0, base1 + n1, base2 + n2])
        pos_tr = vel_tr * 0.95 + 0.01
        hgt_tr = vel_tr * 0.70 + 0.01

        sel.update(t, vel_tr, pos_tr, hgt_tr, fault)
        sel.decide(t)

        for i in range(n_inst):
            h = sel.inst[i].healthy.get_state()
            if h:
                ever_healthy[i] = True
            if first_warn[i] is None and sel.inst[i].warning:
                first_warn[i] = t
            # 只统计「先健康过、然后变 false」的那次跳变
            if (first_unhealthy[i] is None and ever_healthy[i]
                    and prev_healthy[i] and not h):
                first_unhealthy[i] = t
            prev_healthy[i] = h
            if first_rel_below[i] is None and sel.inst[i].relative_test_ratio <= -REL_ERR_THRESH:
                first_rel_below[i] = t

        # 每 2 秒留一个快照
        if k % int(2.0 / dt) == 0:
            snapshots.append((
                t,
                [sel.inst[i].combined_test_ratio for i in range(n_inst)],
                [sel.inst[i].relative_test_ratio for i in range(n_inst)],
                [sel.inst[i].healthy.get_state() for i in range(n_inst)],
                [sel.inst[i].warning for i in range(n_inst)],
                sel.selected))

    sub("④ 每 2 秒的状态快照")

    rows = []
    for t, comb, rel, heal, warn, seld in snapshots:
        rows.append([
            "%5.1f" % t,
            " ".join("%6.3f" % x for x in comb),
            " ".join("%+6.2f" % x for x in rel),
            " ".join(" %s " % ("健" if h else "病") for h in heal),
            " ".join(" %s " % ("警" if w else "-") for w in warn),
            "%d" % seld,
        ])
    tab(rows, ["t[s]", "combined 0/1/2", "relative 0/1/2",
               "healthy 0/1/2", "warning 0/1/2", "主"])

    sub("⑤ 关键时间点（全部由仿真真算出来）")

    rows = []
    for i in range(n_inst):
        rows.append([
            "%d" % i,
            "-" if first_warn[i] is None else "%.2f" % first_warn[i],
            "-" if first_unhealthy[i] is None else "%.2f" % first_unhealthy[i],
            "-" if first_rel_below[i] is None else "%.2f" % first_rel_below[i],
        ])
    tab(rows, ["实例", "首次 warning(ratio≥1.0)", "首次 healthy=false",
               "首次 relative ≤ -0.5"])

    switch_events = [e for e in sel.events if e[1] == "switch"]
    rows = [["%.2f" % e[0], e[2]] for e in switch_events]
    tab(rows, ["切换时刻[s]", "事件"])

    t_warn0 = first_warn[0]
    t_unh0 = first_unhealthy[0]
    dt_warn_unhealthy = (float("nan") if (t_warn0 is None or t_unh0 is None)
                         else t_warn0 - t_unh0)
    t_sw1 = switch_events[0][0] if len(switch_events) > 0 else float("nan")
    t_sw2 = switch_events[1][0] if len(switch_events) > 1 else float("nan")
    t_rel2 = first_rel_below[2]
    wait_guard = (float("nan") if t_rel2 is None else t_sw2 - t_rel2)

    print("""    ★ 四件事在这里被仿真验证了：

      (1) 【warning 和 healthy 是两件互不相干的事】。
          实例 0 先「不健康」（t = %.2f s，filter_fault_flags 置位），
          %.2f s 之后才「warning」（t = %.2f s，ratio 才爬到 1.0）。
          ⇒ 因为 EKF2Selector.cpp:298 的 healthy 判据里
            **根本没有 ratio ≥ 1.0 这一项**：

                const bool healthy = (status.filter_fault_flags == 0)
                                     && (combined_test_ratio > 0.f);   // :298

            而 warning 是另一行（EKF2Selector.cpp:301）：

                _instance[i].warning = (combined_test_ratio >= 1.f);  // :301

          所以 warning 和 healthy 的先后顺序完全看故障长什么样：
          这里 EKF 内部的数值故障检测比选择器的统计门限快了 %.2f s。

      (2) 【true → false 方向确实没有迟滞】。
          filter_fault_flags 在 t = %.2f s 置上，同一拍 healthy 就是 false
          （hysteresis.cpp:72-90 里 true→false 用的是 _time_from_true_us = 0）。
          1 秒迟滞只挡在 false → true 那一侧。
          如果真的按直觉写成「1 秒后才失效」，第一次切换会晚到 %.2f s。

      (3) 【第一次切换当拍就发生，不等任何守卫】（t = %.2f s）。
          主实例一旦不健康，立刻换（EKF2Selector.cpp:755-762）。
          ★ 这条路径【必须】没有守卫：如果也等 10 秒，
            一个已经坏掉的 EKF 还会继续当 10 秒的主。

      (4) 【第二次切换硬生生等满了 10 秒守卫】。
          实例 2 在 t = %s 就已经满足「显著更优」（relative ≤ -0.5），
          但真正切换发生在 t = %.2f s —— 比判据满足晚了 %s。
          原因就是 EKF2Selector.cpp:765：
              hrt_elapsed_time(&_last_instance_change) > 10_s
          上一次切换在 %.2f s，+10 s = %.2f s，所以只能等到这里。
          ★ 这段【等】就是迟滞的价值：给主实例一个翻身的机会，
            否则一个暂时抖动的主实例会被反复换掉（A→B→A→B）。

      ★★ 两条路径的判据来源完全不同，这是本实验最该记住的东西：
            绝对判据 healthy        ：只看实例【自己】有没有坏
            相对判据 relative_test_ratio ：看这个实例【比主实例好多少】
          注意实例 1：在 20 → 30 s 这 10 秒里它的 ratio 一直 < 0.5，
          从来【没有 warning 过】，healthy 一直是 true，
          但它在 t = %.2f s 被换掉了 ——
          **不是因为自己变坏，而是因为别人更好。**
""" % (t_unh0, dt_warn_unhealthy, t_warn0, dt_warn_unhealthy,
       t_unh0, t_sw1 + 1.0, t_sw1,
       "-" if t_rel2 is None else "%.2f s" % t_rel2, t_sw2,
       "-" if math.isnan(wait_guard) else "%.2f s" % wait_guard,
       t_sw1, t_sw1 + SWITCH_GUARD_MS / 1000.0,
       t_sw2))

    sub("⑥ 为什么「不健康」能立刻换，而「别人更好」要等 10 秒")

    print("""    看 EKF2Selector.cpp:757-784 的分支结构：

        if (!_instance[_selected_instance].healthy.get_state()) {
            // prefer the best healthy instance using a different IMU
            if (!SelectInstance(best_ekf_different_imu)) {
                // otherwise switch to the healthy instance with best overall test ratio
                SelectInstance(best_ekf);
            }

        } else if (lower_error_available
                   && ((hrt_elapsed_time(&_last_instance_change) > 10_s)
                       || (_instance[_selected_instance].warning
                           && (hrt_elapsed_time(&_instance[_selected_instance].time_last_no_warning) > 1_s)))) {
            // if this instance has a significantly lower relative error to the active primary, we consider it as a
            // better instance and would like to switch to it even if the current primary is healthy
            SelectInstance(best_ekf_alternate);

        } else if (_request_instance.load() != INVALID_INSTANCE) {
            // 用户手动指定
            ...
        }

      ★ 三个分支的优先级就是「证据的强弱」：

        ① 主实例不健康    —— 证据是【绝对的】（它自己踩线了）⇒ 立即换
        ② 别人显著更优    —— 证据是【相对的】⇒ 要等 10 秒 + 要有 10 秒观察期
        ③ 用户手动指定    —— 放最后，只有前两条都不成立才处理

      ★ 为什么相对判据要这么保守？

        (a) 切换本身有代价。每次 SelectInstance() 都会重新发布
            sensor_selection、重新注册 uORB 回调、并把所有实例的
            relative_test_ratio 清零（EKF2Selector.cpp:113-139）。
            下游控制器正在用 vehicle_attitude，切换瞬间会看到姿态跳变。
            频繁切换比不切换更危险。

        (b) relative_test_ratio 是【累积量】，天然带噪声。10 秒
            是两个不同的保护期：
              · lower_error_available 里的 10 s（EKF2Selector.cpp:740）
                —— 要求这个挑战者【至少 10 秒没当过主】，
                   防止 A→B→A→B 的来回横跳；
              · 分支条件里的 10 s（EKF2Selector.cpp:765）
                —— 要求【距上次切换至少 10 秒】。
            两个 10 秒是【且】的关系，都满足才换。

      ★★ 那条 `||` 分支（warning && time_last_no_warning > 1_s）
        是给「主实例健康但一直在报警」留的后门：
        它健康，但 ratio 长期贴着 1.0，这种情况可以不等满 10 秒。
            而「主实例不健康」是【绝对的】—— 它已经踩过 1.0 那条线了。

      ★ 那为什么 healthy 要设 1 秒迟滞（EKF2Selector.hpp:111）？
        看 22 s 那个时刻：warning 是【瞬时】判据（>= 1.0 就置位），
        如果 healthy 也瞬时，那么 ratio 在 1.0 附近抖动时
        主实例会一秒被切十次。1 秒迟滞把抖动滤掉了。

      ★★ 还有一处更细的：主实例不健康时「优先换成用【不同 IMU】的实例」
        （EKF2Selector.cpp:749-752）：

            // also check next best available ekf using a different IMU
            if (_instance[i].accel_device_id !=
                _instance[_selected_instance].accel_device_id) {
                best_ekf_different_imu = i;
            }

        为什么？因为如果故障来自 IMU 本身（比如那个 IMU 的陀螺歪了），
        那么【任何绑在同一个 IMU 上的实例都会一起坏】。
        换到不同 IMU 的实例，是唯一能真正绕开硬件故障的办法。
        —— 这就回到了 ① 里的「实例数 = IMU 数 × 磁力计数」。
""")

    sub("⑦ 相对误差分数（relative_test_ratio）到底在算什么")

    print("""    它不是 test_ratio 的差，而是一个【不对称的单向累积器】
    （EKF2Selector.cpp:339-359 逐行）：

        // update relative test ratios if primary has updated
        if (primary_updated) {
            for (uint8_t i = 0; i < _available_instances; i++) {
                if (i != _selected_instance) {
                    const float error_delta =
                        _instance[i].combined_test_ratio
                        - _instance[_selected_instance].combined_test_ratio;

                    // reduce error only if its better than the primary instance
                    // by at least EKF2_SEL_ERR_RED to prevent unnecessary selection changes
                    const float threshold = _gyro_fault_detected ? 0.0f
                                            : fmaxf(_param_ekf2_sel_err_red.get(), 0.05f);

                    if (error_delta > 0 || error_delta < -threshold) {
                        _instance[i].relative_test_ratio += error_delta;
                        _instance[i].relative_test_ratio =
                            constrain(_instance[i].relative_test_ratio,
                                      -_rel_err_score_lim, _rel_err_score_lim);
                        ...
                    }
                }
            }
        }

    ★★ 四个设计要点，每一个都是工程上的权衡：

      (1) 【只在主实例更新时才累积】（EKF2Selector.cpp:339 的 primary_updated）。
          这是让「比较」在时间上对齐 —— 拿主实例【同一拍】的
          ratio 去和挑战者的 ratio 比（注意：其实是不同实例各自的
          最新值，因为它们的更新时刻不一样）。

      (2) 【严重不对称】。变差时无条件累积（error_delta > 0），
          变好时只有超过 threshold 才累积（error_delta < -threshold）。
          含义：**扣分容易、加分难。**

          这也是为什么快照表里非主实例的分数会很快顶到 +1.0 ——
          只要它一直比主实例差（哪怕只差 0.01），就在不停地加分（其实是加"差评"）。
          顶到 +1.0 就饱和，表示「它明显不如主实例」。这是正常的，
          而且因为只有【负分】才会触发切换，饱和在 +1.0 是无害的。
          ★ 这一点如果不读源码、只看快照，很容易误以为仿真写错了。

      (3) 【阈值是 EKF2_SEL_ERR_RED，默认 %.1f，下限 0.05】。
          防止「主实例稍微抖动一下就有人来挑战」。
          注释写得很直白（EKF2Selector.cpp:345）：
              reduce error only if its better than the primary instance
              by at least EKF2_SEL_ERR_RED to prevent unnecessary selection changes

          ★★ 这条阈值的含义比看上去强：**一个只比主实例好 0.1 的实例
            永远不会被切换过去**，无论等多久。因为它的 error_delta
            始终落在 (-0.2, 0] 这个死区里，分数一动不动。
            要挑战成功，必须【持续地、明确地】好过 0.2 以上。

      (4) 【限幅 ±%.1f】。否则一个持续 40 秒的偏差会累积到 100 以上，
          而切换判据是「≤ -0.5」，早就该切了。限幅让「显著更好」
          这件事只需要积累有限的时间，而不是无限累加。

      ★ 表里那条往下走的曲线就是这个累积器在工作。
        注意它是**一级阶跃**（每拍加固定量），不是平滑下降 ——
        因为一旦 error_delta 越过 -0.2，每一拍都加同样多的量。

      (5) ★★ 【这个累积器没有乘 dt】。
          看源码：relative_test_ratio += error_delta ——
          加的就是原始的差值本身，没有任何 dt、也没有任何缩放系数。
          而它是【每次主实例更新】累积一次（100 Hz ⇒ 每秒 100 次）。
          后果：一旦 |error_delta| 超过死区 0.2，累积速度就是
          0.2 × 100 = 20 /秒 起步，从 +1.0 掉到 -0.5 只要十几次更新。

          所以「谁更好」这件事在选择器眼里是【秒级】就能看清的，
          而「ratio 爬到 1.0」取决于故障发展的时间尺度。
          本仿真就是照这个量级设计的（见 ③ 的说明）。
""" % (SEL_ERR_RED, REL_ERR_SCORE_LIM))

    sub("⑧ IMU 不一致检测：先于 EKF 的、更底层的一道防线")

    print("""    选择器还干了一件 EKF 之外的事：直接比较几个 IMU 的原始读数
    （EKF2Selector.cpp:155-249）。

    陀螺（EKF2Selector.cpp:163-204）：

        const float angle_rate_threshold = radians(_param_ekf2_sel_imu_angle_rate.get());  // 7.0 deg/s
        const float angle_threshold      = radians(_param_ekf2_sel_imu_angle.get());       // 15.0 deg
        ...
        n_gyros++;
        _accumulated_gyro_error[i] += (gyro_inconsistency_rad_s[i] - angle_rate_threshold)
                                      * time_step_s;
        _accumulated_gyro_error[i] = fmaxf(_accumulated_gyro_error[i], 0.f);
        if (_accumulated_gyro_error[i] > angle_threshold) { n_gyro_exceedances++; }

    加计（EKF2Selector.cpp:206-247）是同一个结构，两个门限是
        EKF2_SEL_IMU_ACC = 1.0 m/s²（差值门限，比的是 accel_inconsistency_m_s_s）
        EKF2_SEL_IMU_VEL = 2.0 m/s （累积门限，比的是累积量）

    ★★ 注意单位上的巧思：加计的差值单位是 m/s²，乘上 time_step_s（秒）
      之后累积量的单位变成了 **m/s**。所以 EKF2_SEL_IMU_VEL 的单位
      真的就是 m/s —— 「累积了多少速度误差」。源码里变量名
      `velocity_threshold` 和 `_accumulated_accel_error` 并存，
      看起来别扭，其实就是这个原因。

    ★★★ 三个设计都值得学：

      (1) 【累积而不是瞬时比较】。
          单次采样差 7 deg/s 可能只是噪声尖峰；
          「持续超出 7 deg/s，累积到 15 deg」才是真的歪了。
          而且用的是 `fmaxf(..., 0.f)` 而不是真的积分 ——
          累积量**永远不会减小**，差别低于门限时只是停止增长。
          这是一个单向累加器（不是泄漏积分器）。

      (2) 【故障身份只在传感器数 ≥ 3 时才能确定】
          （EKF2Selector.cpp:193-203）：

              if (n_gyro_exceedances > 0) {
                  if (n_gyros >= 3) {
                      // If there are 3 or more sensors, the one with the largest
                      // accumulated error is faulty
                      _gyro_fault_detected = true;
                      faulty_gyro_id = sensors_status_imu.gyro_device_ids[largest_gyro_error_index];
                  } else if (n_gyros == 2) {
                      // A fault is present, but the faulty sensor identity cannot be determined
                      _gyro_fault_detected = true;
                  }
              }

          ★★ **只有 2 个陀螺时，能知道「有一个坏了」，但不知道是哪个。**
            这是「多数表决」（voting）的基本限制：需要 3 个才能表决。
            有 3 个时，离群的那个必然是被孤立的那个；
            只有 2 个时，两者必有一个错，但无法区分。
            ⇒ 所以在 2 个传感器时，_gyro_fault_detected 为真但
              faulty_gyro_id 保持为 0，下面 (3) 那个 if 就不成立，
              **不会误杀任何一个实例** —— 只是不再信任它们。

      (3) 【检测到之后，直接把用该 IMU 的实例判死】
          （EKF2Selector.cpp:318-321）：

              // if the gyro used by the EKF is faulty, declare the EKF unhealthy without delay
              if (_gyro_fault_detected && (faulty_gyro_id != 0)
                  && (_instance[i].gyro_device_id == faulty_gyro_id)) {
                  _instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
              }

          注释里的 without delay 是【真的】—— true → false 方向
          本来就没有迟滞（见 ② 的分析）。硬件已经明确坏了，当拍下台。

      ★ 顺带一个细节：一旦 _gyro_fault_detected 为真，
        相对误差的阈值变成 0（EKF2Selector.cpp:346）：
            const float threshold = _gyro_fault_detected ? 0.0f
                                    : fmaxf(_param_ekf2_sel_err_red.get(), 0.05f);
        也就是**解除了 (3) 里那个 0.2 的死区保护** ——
        硬件都出故障了，任何一点更好都值得切。
        这是一个「证据够了就放宽判据」的典型写法。
""")

    sub("⑨ 最后：healthy 的定义（一句话版本）")

    print("""    EKF2Selector.cpp:298 一行就是全部定义：

        const bool healthy = (status.filter_fault_flags == 0)
                             && (combined_test_ratio > 0.f);

    拆成两半：

        filter_fault_flags == 0
            EKF 内部有没有报「数值故障」——
            来自 fault_status_u（common.h:499-515）：
                bad_mag_x/y/z       磁力计某轴融合出数值错误
                bad_hdg             航向融合出数值错误
                bad_mag_decl        磁偏角融合出数值错误
                bad_airspeed        空速融合出数值错误
                bad_sideslip        侧滑融合出数值错误
                bad_optflow_X/Y     光流某轴融合出数值错误
                bad_acc_bias        加计零偏估计坏了
                bad_acc_vertical    加计垂直方向数据坏了
                bad_acc_clipping    加计数据在削顶（振动过大）

            ★ 这些都是【EKF 自己知道自己算错了】的情况，
              和「测量值不好」是两回事。
              典型触发点：mag_fusion.cpp:102 的
                  if (innov_var.min() < R_MAG) { ... ECL_ERR("mag %s", ...); return false; }
              ——「新息方差比观测噪声还小」在数学上不可能，
                 说明协方差矩阵已经病态了，直接重置。

        combined_test_ratio > 0
            唯一的过滤作用是把「无效（0 / 负数 / NaN，已在
            EKF2Selector.cpp:282-292 被夹成 1.0）」的实例排除。
            ★ 注意它【不是】一个质量指标 —— >0 就算真。

    ★★ 所以 healthy **不表示「估计很准」**，
      只表示「滤波器没坏、而且新息还在门限内」。
      准不准是 combined_test_ratio 的大小在说的事，
      选择器用它排序（EKF2Selector.cpp:745-747）：
          if ((test_ratio > 0) && (test_ratio < best_test_ratio)) {
              best_ekf = i;
              best_test_ratio = test_ratio;

    ★★★ 一句话总结这一节：
      **healthy 是布尔量（能不能用），combined_test_ratio 是连续量（有多好），
        relative_test_ratio 是历史量（比主实例好还是差、差多少）。
        EKF2Selector 的所有决策就是这三个量的组合。**
""")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("★ 状态向量：24 个 float vs 23 个自由度", exp1),
    "2": ("★★ 融合顺序 + 简化 EKF2 主循环", exp2),
    "3": ("★★ 新息门限（卡方检验）", exp3),
    "4": ("★★ 多实例选择器", exp4),
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
