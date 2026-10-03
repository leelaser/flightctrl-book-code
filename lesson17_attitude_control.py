# -*- coding: utf-8 -*-
"""
第 17 周 · 02 姿态控制律设计 —— 配套实验
==========================================

运行方式（在 code 目录下）：

    python lesson17_attitude_control.py        # 跑全部 4 个实验
    python lesson17_attitude_control.py 1      # 只跑实验 1
    python lesson17_attitude_control.py 2      # 只跑实验 2
    python lesson17_attitude_control.py 3      # 只跑实验 3
    python lesson17_attitude_control.py 4      # 只跑实验 4

只依赖 numpy（matplotlib 可选，没有就跳过画图）。

配套讲义：05-控制理论/02-姿态控制律设计.md

约定（和全书一致）：
    R   = R_{body -> ned}，v^n = R · v^b
    q   = 同一个旋转的四元数，Hamilton 约定，v^n = q ⊗ v^b ⊗ q^{-1}
    ω   = 机体角速度（机体系分量），J ω̇ = M − ω × (Jω)
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
except Exception:                                          # noqa: BLE001
    HAVE_PLT = False

HERE = os.path.dirname(os.path.abspath(__file__))
W = 78


# ===========================================================================
# 输出小工具（含 CJK 宽度对齐）
# ===========================================================================

def _dw(s):
    """字符串的显示宽度：CJK 字符算 2 列。"""
    w = 0
    for ch in s:
        w += 2 if ("ᄀ" <= ch <= "ᅟ"
                   or "⺀" <= ch <= "꓏"
                   or "가" <= ch <= "힣"
                   or "豈" <= ch <= "﫿"
                   or "︰" <= ch <= "﹯"
                   or "＀" <= ch <= "｠"
                   or "￠" <= ch <= "￦") else 1
    return w


def _pad(s, n, align="l"):
    d = n - _dw(s)
    if d <= 0:
        return s
    if align == "r":
        return " " * d + s
    if align == "c":
        left = d // 2
        return " " * left + s + " " * (d - left)
    return s + " " * d


def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)
    print()


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * len(title))
    print()


def tab(rows, headers, aligns=None, gap=2):
    """按显示宽度对齐的表格。"""
    n = len(headers)
    widths = [_dw(h) for h in headers]
    for r in rows:
        for i in range(n):
            cell = str(r[i]) if i < len(r) else ""
            widths[i] = max(widths[i], _dw(cell))
    if aligns is None:
        aligns = ["l"] + ["r"] * (n - 1)
    line = (" " * gap).join(_pad(headers[i], widths[i], "l")
                            for i in range(n))
    print("  " + line.rstrip())
    print("  " + "-" * (sum(widths) + gap * (n - 1)))
    for r in rows:
        cells = [str(r[i]) if i < len(r) else "" for i in range(n)]
        line = (" " * gap).join(_pad(cells[i], widths[i], aligns[i])
                                for i in range(n))
        print("  " + line.rstrip())


def fnum(x, fmt="%.3f", big=1e5):
    """安全格式化：非有限值或过大时返回 —。"""
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return "—"
    if not math.isfinite(xf) or abs(xf) > big:
        return "—"
    return fmt % xf


# ===========================================================================
# 旋转：四元数 / 旋转矩阵 / 轴角
#
# ★ 全部按 q = q_{body->ned} 实现，与 PX4 的 Quatf 完全同约定
#   （见 PX4-Autopilot/src/lib/matrix/matrix/Quaternion.hpp 文件头注释）：
#       v_n = q_nb * [0;v_b] * q_nb^(-1)
# ===========================================================================

def q_mul(a, b):
    """Hamilton 积 a ⊗ b：先转 b 再转 a。a,b = (w,x,y,z)。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q):
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-15 else np.array([1.0, 0.0, 0.0, 0.0])


def q_canonical(q):
    """把四元数化为「标量部非负」的规范形式。

    ★ q 和 −q 表示【同一个旋转】，但 2·vec(q) 的正负号完全相反。
      PX4 的 qe.canonical() 就是干这个的（Quaternion.hpp:442-465）。
    """
    return -q if q[0] < 0.0 else q


def q_from_axis_angle(axis, angle):
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    a = a / n
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), a[0] * s, a[1] * s, a[2] * s])


def q_to_R(q):
    """q_{b->n} -> R_{b->n}（= q.dcm()）。"""
    w, x, y, z = q_norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def R_to_q(R):
    """R_{b->n} -> q_{b->n}（Shepperd 分支法，数值稳定）。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s,
                      (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    q = q_norm(q)
    return q if q[0] >= 0 else -q


def R_log(R):
    """R -> 旋转向量（轴角）θ·n̂，θ ∈ [0, π]。

    ★ 这就是 so(3) 的对数映射。θ 是【测地线角度】——
      两个姿态之间真正的「角度距离」。
    """
    c = (R[0, 0] + R[1, 1] + R[2, 2] - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    theta = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2],
                  R[0, 2] - R[2, 0],
                  R[1, 0] - R[0, 1]])
    if theta < 1e-6:
        # θ→0：sinθ ≈ θ，直接除 2θ
        return 0.5 * v
    if abs(math.pi - theta) < 1e-5:
        # θ→π：sinθ→0，改用 (R+I)/2 = n̂n̂ᵀ 的主对角元
        A = 0.5 * (R + np.eye(3))
        d = np.clip(np.diag(A), 0.0, None)
        n = np.sqrt(d)
        k = int(np.argmax(n))
        if n[k] > 1e-12:
            n = A[:, k] / n[k]
        n = n / max(float(np.linalg.norm(n)), 1e-15)
        return theta * n
    return theta * v / (2.0 * math.sin(theta))


def R_exp(v):
    """旋转向量 -> R（so(3) 的指数映射）。"""
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    n = v / th
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def q_to_euler(q):
    """q_{b->n} -> (roll, pitch, yaw)，ZYX 顺序，单位 rad。

    ★ 注意 pitch → ±90° 时 yaw 是奇异的（分母 cosθ → 0）。
    """
    R = q_to_R(q)
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) > 1.0 - 1e-9:
        # 万向节死锁：约定 yaw = 0
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def deg(v):
    return np.degrees(np.asarray(v, dtype=float))


# ===========================================================================
# 被控对象：刚体转动
#
#     J ω̇ = M − ω × (Jω) + M_d
#
#   ★ 叉乘项 ω × (Jω) 就是讲义第 17.3 节的主角：
#     它是【陀螺进动项】，不是力矩，但在转动坐标里长得像力矩。
# ===========================================================================

class RigidBody:
    def __init__(self, Jdiag=(0.008, 0.008, 0.014)):
        self.J = np.diag(np.asarray(Jdiag, dtype=float))
        self.Jinv = np.linalg.inv(self.J)

    def accel(self, w, M, Md=None, use_cross=True):
        gyro = np.cross(w, self.J @ w) if use_cross else np.zeros(3)
        d = np.zeros(3) if Md is None else np.asarray(Md, dtype=float)
        return self.Jinv @ (M - gyro + d)


def quat_kinematics(q, w):
    """q̇ = 0.5 · q ⊗ (0, ω^b)。

    ★ 这里 ω 必须是【机体系】分量 —— 因为 q 是 body→ned。
    """
    return 0.5 * q_mul(q, np.array([0.0, w[0], w[1], w[2]]))


def norm_torque_scale(mass=1.0, arm=0.25, twr=2.0):
    """「归一化力矩 1.0」对应多少 N·m —— 用第 3 章的混控矩阵算。

    ★ PX4 的 rate_control.cpp 输出的是【归一化力矩】（−1…1），
      真正乘上力臂/推力变成 N·m 的是控制分配器。
      本脚本要拿它把 PX4 的 MC_ROLLRATE_* 参数换算成物理单位，
      所以这里把第 16 周实验 1 ② 的那段计算原样搬过来
      （复现 AP_MotorsMatrix.cpp 的 add_motor 系数约定）。

    ★★ 这个数【不是拍脑袋来的】，它是本脚本运行的输出，
      并在实验 2 ④ 里当作 τ_max 使用 —— 两处必须一致。
    """
    g = 9.80665
    th = mass * g / 4.0
    t_max = twr * th
    frames = [(45, +1), (-135, +1), (-45, -1), (135, -1)]
    mix = []
    for ang, sig in frames:
        mix.append([math.cos(math.radians(ang + 90.0)),
                    math.cos(math.radians(ang)), float(sig), 1.0])
    mix = np.array(mix).T
    fac = mix @ np.array([1.0, 0.0, 0.0, 1.0])
    thrust = th * np.clip(fac, 0.0, t_max / th)
    tau = 0.0
    for i, (ang, _sig) in enumerate(frames):
        tau += -math.sin(math.radians(ang)) * arm * thrust[i]
    return abs(tau)


def rigid_sim(ctrl, T=3.0, dt=1.0 / 500, q0=None, w0=None, Md=None,
              body=None, use_cross=True, tau_act=0.0, rec=None):
    """闭环仿真：ctrl(q, w, t, dt) -> M（机体力矩，N·m）。

    tau_act > 0 时，实际作用到机体上的力矩 ua 以一阶滞后跟随 M：

        u̇a = (M − ua) / tau_act

    ★ 这是「真实内环不是理想一阶」的全部来源：
      电机升速、桨叶惯量、D 项滤波器、计算延迟。
      实验 16.4 已经证明过：没有它，带宽分离就讲不通。
    """
    body = body if body is not None else RigidBody()
    q = np.array([1.0, 0.0, 0.0, 0.0]) if q0 is None else q_norm(np.asarray(q0, float))
    w = np.zeros(3) if w0 is None else np.asarray(w0, dtype=float)
    Mdv = np.zeros(3) if Md is None else np.asarray(Md, dtype=float)

    n = int(round(T / dt))
    keys = ["t", "qw", "qx", "qy", "qz", "wx", "wy", "wz",
            "Mx", "My", "Mz", "Ux", "Uy", "Uz"]
    if rec is None:
        rec = list(keys)
    log = {k: np.zeros(n) for k in keys}
    extra = {k: np.zeros(n) for k in rec if k not in keys}
    log.update(extra)

    # ZOH 下的一阶滞后精确步进系数
    a_lag = 0.0 if tau_act <= 0.0 else (1.0 - math.exp(-dt / tau_act))
    ua = np.zeros(3)

    M_hold = np.zeros(3)
    for i in range(n):
        t = i * dt
        M = np.asarray(ctrl(q, w, t, dt), dtype=float)
        M_hold = M
        if tau_act <= 0.0:
            ua = M
        else:
            ua = ua + a_lag * (M - ua)

        def f(qq, ww):
            return (quat_kinematics(qq, ww),
                    body.accel(ww, ua, Mdv, use_cross))

        k1q, k1w = f(q, w)
        k2q, k2w = f(q_norm(q + 0.5 * dt * k1q), w + 0.5 * dt * k1w)
        k3q, k3w = f(q_norm(q + 0.5 * dt * k2q), w + 0.5 * dt * k2w)
        k4q, k4w = f(q_norm(q + dt * k3q), w + dt * k3w)
        q = q_norm(q + dt / 6.0 * (k1q + 2 * k2q + 2 * k3q + k4q))
        w = w + dt / 6.0 * (k1w + 2 * k2w + 2 * k3w + k4w)

        log["t"][i] = t
        log["qw"][i], log["qx"][i], log["qy"][i], log["qz"][i] = q
        log["wx"][i], log["wy"][i], log["wz"][i] = w
        log["Mx"][i], log["My"][i], log["Mz"][i] = M
        for k in extra:
            extra[k][i] = ctrl.last.get(k, 0.0) if hasattr(ctrl, "last") else 0.0
    return log


# ===========================================================================
# 控制器：外环（姿态 → 角速率指令）+ 内环（角速率 → 力矩）
# ===========================================================================

class AttController:
    """姿态 + 角速率串级控制器。

    ★ 两种姿态误差写法，都在这里实现，好做对照：

      att_mode='px4'      qe = q^{-1} ⊗ q_d ；e = 2·canonical(qe).vec
                          ω_sp = +K_att · e
                          （PX4 的写法，见 AttitudeControl.cpp:84-91）

      att_mode='rotvec'   R_e = R_d^T R ；e = log(R_e)
                          ω_sp = −K_att · e
                          （教科书/几何控制的写法）

      ★★ 两者数值上【完全等价】，因为 q_d^{-1}⊗q = (q^{-1}⊗q_d)^{-1}
         而单位四元数取逆 = 虚部取反。实验 1 ④ 会验证这一点。

      att_mode='euler'    e = wrap(euler_d − euler)
                          ω_sp = +K_att · e
                          ★ 这是【错】的：欧拉角速率不是机体角速率。
                            实验 1 ② 会量化它在 pitch→90° 时怎么炸。
    """

    def __init__(self, K_att=7.0, K_rate=0.15, Kd=0.0, Ki=0.0,
                 i_limit=None, att_mode="px4", rate_mode="P",
                 cross_ff=False, w_max=None, M_max=None, q_des=None,
                 body=None, d_lpf_hz=None):
        self.K_att = K_att
        self.K_rate = K_rate
        self.Kd = Kd
        self.Ki = Ki
        self.i_limit = i_limit
        self.att_mode = att_mode
        self.rate_mode = rate_mode
        self.cross_ff = cross_ff
        self.w_max = w_max
        self.M_max = M_max
        self.q_des = (np.array([1.0, 0.0, 0.0, 0.0]) if q_des is None
                      else q_norm(np.asarray(q_des, float)))
        self.body = body
        self.d_lpf_hz = d_lpf_hz
        self.last = {}
        self.reset()

    def reset(self):
        self._int = np.zeros(3)
        self._w_prev = None
        self._wd_f = None

    # ---- 外环：姿态误差 ------------------------------------------------
    def att_error(self, q):
        qd = self.q_des
        if self.att_mode == "euler":
            e = q_to_euler(qd) - q_to_euler(q)
            e = (e + math.pi) % (2 * math.pi) - math.pi
            return e
        if self.att_mode == "px4":
            qe = q_mul(q_conj(q), qd)
            return 2.0 * q_canonical(qe)[1:4]
        # rotvec：R_e = R_d^T R = R_{当前机体系 -> 期望机体系}
        R_e = q_to_R(qd).T @ q_to_R(q)
        return R_log(R_e)

    def att_sign(self):
        """rotvec 模式要带负号，px4/euler 模式不带。"""
        return -1.0 if self.att_mode == "rotvec" else 1.0

    # ---- 主循环 -------------------------------------------------------
    def __call__(self, q, w, t, dt):
        e_att = self.att_error(q)
        w_sp = self.att_sign() * self.K_att * e_att
        if self.w_max is not None:
            n = float(np.linalg.norm(w_sp))
            if n > self.w_max:
                w_sp = w_sp * (self.w_max / n)

        err = w_sp - w
        M = self.K_rate * err

        if self.rate_mode in ("PD", "PID") and self.Kd > 0.0:
            raw = (w - self._w_prev) / dt if self._w_prev is not None else np.zeros(3)
            if self.d_lpf_hz is not None:
                a = (2 * math.pi * self.d_lpf_hz * dt
                     / (1.0 + 2 * math.pi * self.d_lpf_hz * dt))
                self._wd_f = raw if self._wd_f is None else self._wd_f + a * (raw - self._wd_f)
                raw = self._wd_f
            # ★ 减 Kd·ω̇：这是 PX4 的 −_gain_d·angular_accel
            M = M - self.Kd * raw
        self._w_prev = w.copy()

        if self.rate_mode == "PID" and self.Ki > 0.0:
            trial = self._int + self.Ki * err * dt
            if self.i_limit is not None:
                trial = np.clip(trial, -self.i_limit, self.i_limit)
            self._int = trial
            M = M + self._int

        if self.cross_ff and self.body is not None:
            # ★ 陀螺进动前馈：+ ω × (Jω)
            M = M + np.cross(w, self.body.J @ w)

        if self.M_max is not None:
            M = np.clip(M, -self.M_max, self.M_max)

        self.last = {"e_att": float(np.linalg.norm(e_att)),
                     "w_sp": w_sp.copy(), "I": self._int.copy()}
        return M


# ===========================================================================
# 姿态误差的度量工具
# ===========================================================================

def geodesic_angle(q1, q2):
    """两个姿态之间的测地线夹角 θ ∈ [0, π]。"""
    d = abs(float(np.dot(q_norm(q1), q_norm(q2))))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def err_px4(q, qd):
    """PX4 写法：qe = q^{-1} ⊗ q_d，e = 2·canonical(qe).vec。"""
    return 2.0 * q_canonical(q_mul(q_conj(q), qd))[1:4]


def err_rotvec(q, qd):
    """教科书写法：R_e = R_d^T R，e = log(R_e)。"""
    return R_log(q_to_R(qd).T @ q_to_R(q))


def err_euler(q, qd):
    e = q_to_euler(qd) - q_to_euler(q)
    return (e + math.pi) % (2 * math.pi) - math.pi


def probe_sign(q, qd, e, dt=1e-4):
    """把 e 当作「机体角速率指令」推一步，看夹角是变小还是变大。

    ★ 返回 dθ/dt 的数值符号：负 = 在收敛（符号对了），
      正 = 在发散（符号反了）。
    """
    th0 = geodesic_angle(q, qd)
    q1 = q_norm(q + dt * quat_kinematics(q, e))
    th1 = geodesic_angle(q1, qd)
    return (th1 - th0) / dt, th0


def test_attitudes():
    """一组有代表性的「当前姿态」和「期望姿态」。"""
    x = np.array([1.0, 0.0, 0.0])
    y = np.array([0.0, 1.0, 0.0])
    z = np.array([0.0, 0.0, 1.0])
    d = np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0)
    cases = [
        ("水平（无误差）", q_from_axis_angle(x, 0.0), None),
        ("横滚 10°", q_from_axis_angle(x, math.radians(10)), None),
        ("俯仰 45°", q_from_axis_angle(y, math.radians(45)), None),
        ("绕 (1,1,0) 转 60°", q_from_axis_angle(d, math.radians(60)), None),
        ("俯仰 89°（近死锁）", q_from_axis_angle(y, math.radians(89)), None),
        ("横滚 90° 后俯仰 45°",
         q_mul(q_from_axis_angle(x, math.radians(90)),
               q_from_axis_angle(y, math.radians(45))), None),
        ("偏航 170°", q_from_axis_angle(z, math.radians(170)), None),
        ("横滚 30°→俯仰 40°",
         q_mul(q_from_axis_angle(x, math.radians(30)),
               q_from_axis_angle(y, math.radians(40))),
         q_from_axis_angle(x, math.radians(20))),
    ]
    return cases


# ===========================================================================
# 实验 1  ★★ 姿态误差的三种写法
# ===========================================================================

def exp1():
    head("实验 1  ★★ 姿态误差：旋转矩阵 / 四元数 / 轴角，各是什么")

    print("""
    姿态控制律要回答的第一个问题是：**「现在离目标差多少」怎么算？**
    这一步错了，后面的增益怎么调都是错的。

    有三种常见写法：

      (a) 旋转矩阵：R_e = R_d^T R        → 再取对数映射，得旋转向量
      (b) 四元数：  q_e = q_d^{-1} ⊗ q    → 再取 2·vec，得「轴角」
      (c) 欧拉角：  e = wrap(θ_euler_d − θ_euler)

    ★ 本实验的结论（先给答案）：

      ★★ (a) 和 (b) 是【同一个轴角】的两种算法：方向严格相反
         （差一个负号），模长分别是 θ 和 2·sin(θ/2)。

      ★★★ 真正被送进控制律的，必须是【轴角形式】（机体系里的一个向量），
          不能是四元数的四个分量，更不能是欧拉角之差。

      ★ 而且 (a)(b) 是【几何量】（机体系里的一个向量），
        (c) 是【坐标量】（依赖于欧拉角的定义和顺序，在 pitch=90° 处奇异）。
        把 (c) 当机体角速率用，飞机会在近死锁姿态直接失控。
""")

    # ---------------------------------------------------------------- ①
    sub("① 三种写法的数值对照（期望姿态：回到水平）")

    qd_level = np.array([1.0, 0.0, 0.0, 0.0])
    rows = []
    for name, q, qd in test_attitudes():
        qd = qd_level if qd is None else qd
        th = geodesic_angle(q, qd)
        e_rv = err_rotvec(q, qd)
        e_q = err_px4(q, qd)
        e_ev = err_euler(q, qd)
        # e_px4 与 e_rotvec 的夹角：0° = 同向，180° = 反向
        na, nb = np.linalg.norm(e_q), np.linalg.norm(e_rv)
        cosang = (float(np.dot(e_q, e_rv)) / (na * nb)) if na * nb > 1e-18 else -1.0
        ang = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        rows.append([name,
                     "%.2f" % math.degrees(th),
                     "%.2f" % math.degrees(nb),
                     "%.2f" % math.degrees(na),
                     "%.2f" % math.degrees(2 * math.sin(th / 2)),
                     "%.1f" % ang,
                     "%.2f" % math.degrees(np.linalg.norm(e_ev))])
    print("    第 5 列 = 2·sin(θ/2)（e_px4 的模长解析值）")
    print("    第 6 列 = e_px4 与 e_rotvec 的夹角（0° 同向 / 180° 反向）")
    print()
    tab(rows, ["当前姿态", "真实夹角 θ (°)", "‖e_rotvec‖ (°)",
               "‖e_px4‖ (°)", "2·sin(θ/2) (°)", "两者夹角 (°)",
               "‖欧拉角差‖ (°)"])

    print("""    ★★ 逐列读：

      · 第 3 列 = 第 2 列：**旋转向量的模长【严格等于】真实夹角 θ**。
        这不是巧合 —— log(R_e) 的定义就是「转多少角度才能对齐」。
        ⇒ 用它做误差，增益 K_att 的物理含义干净得不得了：
          「1 rad 的姿态偏差 ⇒ K_att rad/s 的角速率指令」。

      · 第 4 列 = 第 5 列：**‖2·vec(q_e)‖ = 2·sin(θ/2)**，
        和解析式逐位吻合。
        ★★ 注意 θ 大时它【小于】θ：θ = 170° 时只有 114°。
        所以 PX4 的写法在【大角度】时等效增益是自动收窄的
        （有点像自带一个软饱和），θ → 180° 时压到最狠（2/π ≈ 0.64 倍）。
        ★ 在小角度（悬停附近）两者都 ≈ θ，所以调参时区别不出来。

      · 第 6 列恒为 180°：**(a) 和 (b) 方向严格相反**。
        这就是那个「教科书写法里的负号」的全部来源。

      · 第 7 列：欧拉角之差。它的值一会儿大一会儿小，
        ★★ 而且**它根本不是机体系里的向量** ——
          它没有「方向」这个物理含义，只有「绕三个坐标轴各转多少」。
          这就是下一小节要量化的东西。
""")

    # ---------------------------------------------------------------- ②
    sub("② ★★ 为什么不能用欧拉角差：pitch → 90° 时的雅可比爆炸")

    print("""    机体角速率 ω = (p, q, r) 和欧拉角速率的关系是

        ⎡φ̇⎤   ⎡1  sinφ·tanθ  cosφ·tanθ⎤ ⎡p⎤
        ⎢θ̇⎥ = ⎢0    cosφ      −sinφ   ⎥ ⎢q⎥
        ⎣ψ̇⎦   ⎣0  sinφ/cosθ  cosφ/cosθ⎦ ⎣r⎦

    ★★ 注意 tanθ 和 1/cosθ 两个因子：θ → ±90° 时它们都 → ∞。

    一个把欧拉差【直接当机体角速率】用的控制器，等价于假设
    上面那个矩阵是单位阵 —— 在 θ = 0 附近勉强成立，
    在 θ = 89° 附近则完全不对。
""")

    rows = []
    for th_deg in (0.0, 30.0, 60.0, 80.0, 85.0, 89.0, 89.9):
        th = math.radians(th_deg)
        rows.append(["%.1f" % th_deg, "%.3f" % math.tan(th),
                     "%.2f" % (1.0 / math.cos(th))])
    tab(rows, ["pitch θ (°)", "tanθ", "1/cosθ"])

    print("""    ★ 在 θ = 89.9° 处，1/cosθ ≈ %.0f。

    ⇒ ★★ 这意味着：一个「偏航差 1°」的欧拉误差，如果按
      「欧拉差 → 机体角速率」的错映射去执行，实际要求的机体角速率
      会被放大到 1/cosθ 倍 —— 根号式的放大，最终把控制器推进
      振荡甚至失控。**而在 θ = 90° 处完全无解（奇异的）。**
""" % (1.0 / math.cos(math.radians(89.9))))

    print("""    ▸ 数字验证：把飞机放在【俯仰 85°】，然后命令它【偏航转 90°】。

      ★ 这是一个非常小的机动手势，但它是欧拉角写法最致命的场景：
        在 θ = 85° 处，偏航通道的「欧拉角速率 ÷ 机体角速率」
        = 1/cos(85°) ≈ 11.5（θ = 89° 时是 57）。
""")

    body = RigidBody()
    K_att, K_rate = 7.0, 0.15
    y_ax = np.array([0.0, 1.0, 0.0])
    z_ax = np.array([0.0, 0.0, 1.0])
    q0_hp = q_from_axis_angle(y_ax, math.radians(85.0))
    qd_hp = q_mul(q0_hp, q_from_axis_angle(z_ax, math.radians(90.0)))
    qd_pitch = qd_hp                      # 供画图复用
    th_true = math.degrees(geodesic_angle(q0_hp, qd_hp))
    print("    ★ 真实姿态偏差 θ = %.1f° —— 注意它【既不等于俯仰的 85°，" % th_true)
    print("      也不等于偏航的 90°】，而是两者合成后的测地线夹角。")
    print("      这正是「姿态误差不是几个角度的加减法」的最直接证据。")
    print()

    rows = []
    for mode, label in (("px4", "★ 旋转向量（轴角）"), ("euler", "欧拉角差")):
        c = AttController(K_att=K_att, K_rate=K_rate, att_mode=mode,
                          q_des=qd_hp, body=body)
        lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0_hp, body=body)
        qs = np.stack([lg["qw"], lg["qx"], lg["qy"], lg["qz"]], axis=1)
        errs = [math.degrees(geodesic_angle(qs[i], qd_hp))
                for i in range(len(lg["t"]))]
        peak_w = float(np.max(np.abs(np.stack(
            [lg["wx"], lg["wy"], lg["wz"]], axis=1))))
        rows.append([label,
                     "%.3f" % errs[-1],
                     "%.1f" % max(errs),
                     "%.1f" % peak_w,
                     fnum(float(np.max(np.abs(lg["Mz"]))), "%.3f")])
    tab(rows, ["误差写法", "终态误差 (°)", "峰值误差 (°)",
               "峰值角速率 (rad/s)", "峰值 |Mz| (N·m)"])

    print("""    ★★ 对照非常清楚：

      · 旋转向量：误差单调收敛到 0.000°。
      · 欧拉角差：★ 峰值角速率只比前者大一点点、力矩也只大一点点，
        **但它【收敛不了】—— 终态卡在 55° 左右，差了半个直角。**

    ⇒ ★★ 这才是最阴险的地方：它不是「抖一下」或者「慢一点」，
      而是**稳稳地停在一个完全错误、而且根本不该出现的大误差上**。
      因为偏航通道的等效增益被 1/cos(85°) ≈ 11.5 放大，
      控制器在「快到了」的时候用力过猛、冲过头，
      在俯仰/偏航两个通道之间来回倒能量，最后停在一个
      既不收敛也不发散的平衡点上。

    ⇒ ★ 而 θ = 89° 时这个放大是 57 倍，θ = 90° 时**完全无解**。
      所以任何用欧拉角做误差的实现，都必须把飞机的俯仰角
      限制在远小于 90° 的范围里 —— 这对一个要做特技动作的
      飞控是完全不可接受的。
""")

    print("""    ⇒ ★★ ArduPilot 和 PX4 的解法一模一样，都落在「轴角」上：

      · PX4  AttitudeControl.cpp:84-88

            // quaternion attitude control law, qe is rotation from q to qd
            const Quatf qe = q.inversed() * qd;
            // using sin(alpha/2) scaled rotation axis as attitude error
            // (see quaternion definition by axis angle)
            // also taking care of the antipodal unit quaternion ambiguity
            const Vector3f eq = 2.f * qe.canonical().imag();

      · AP   AC_AttitudeControl_TS.cpp:122-125

            error_quat = attitude_body.inverse() * _attitude_target;
            error_quat.to_axis_angle(att_error);
            _ang_vel_body = update_ang_vel_target_from_att_error(att_error);

        ★ 而 update_ang_vel_target_from_att_error
          （AC_AttitudeControl.cpp:1012）的形参名直接把答案写出来了：

              const Vector3f &attitude_error_rot_vec_rad
                             ~~~~~~~~~~~~~~~ 旋转向量，单位弧度

      ★★ 两家的误差定义是同一个（q^{-1} ⊗ q_d），
        取的都是【轴角/旋转向量】，然后逐轴乘增益。
        这就是「姿态误差」这件事在工程上的唯一正确答案。
""")

    # ---------------------------------------------------------------- ③
    sub("③ ★★ 为什么必须 canonical()：四元数的「双覆盖」陷阱")

    print("""    四元数有一个矩阵和欧拉角都没有的性质：**双覆盖**。

        q 和 −q 表示【同一个旋转】

    好处是没有奇异点；坏处是任何一次四元数乘法都可能吐出一个
    「标量部为负」的结果，而 2·vec(q) 在 q → −q 时会【整体反号】。

    ★ 这在真实飞控里不是理论问题：姿态估计器（第 4 章的 EKF/互补滤波）
      输出的四元数符号是【没有约定】的，两次更新之间可能翻号。
""")

    y = np.array([0.0, 1.0, 0.0])
    q = q_from_axis_angle(y, math.radians(30.0))
    qd = q_from_axis_angle(y, math.radians(50.0))
    q_bad = -q                                   # 同一个姿态，符号翻转

    for label, qq in (("正常符号 q", q), ("符号翻转 −q（同一姿态！）", q_bad)):
        qe = q_mul(q_conj(qq), qd)
        raw = 2.0 * qe[1:4]
        can = 2.0 * q_canonical(qe)[1:4]
        d_raw, th0 = probe_sign(qq, qd, raw)
        d_can, _ = probe_sign(qq, qd, can)
        print("    %-26s q_e 标量部 = %+.4f" % (label, qe[0]))
        print("      2·vec(q_e) = (%+.4f, %+.4f, %+.4f) ⇒ dθ/dt = %+8.3f  %s"
              % (raw[0], raw[1], raw[2], d_raw,
                 "★ 收敛" if d_raw < 0 else "★★ 发散（符号反了）"))
        print("      canonical 后 = (%+.4f, %+.4f, %+.4f) ⇒ dθ/dt = %+8.3f  %s"
              % (can[0], can[1], can[2], d_can,
                 "★ 收敛" if d_can < 0 else "发散"))
        print("      （初始夹角 θ₀ = %.2f°）" % math.degrees(th0))
        print()

    print("""    ★★ 结论：只有 canonical（标量部取正）之后，2·vec(q_e) 才是
      【从当前姿态指向期望姿态】的那个旋转向量。不做这一步，
      控制器会在估计器翻号的那一刻把飞机往反方向推 ——
      ★ 这是「偶发、不可复现、一秒钟就翻回来」的飞控事故的经典成因。

    ★ 两家飞控都做了这一步：
      · PX4  AttitudeControl.cpp:88   qe.canonical().imag()
      · AP   AC_AttitudeControl_TS.cpp:122-123
               error_quat = attitude_body.inverse() * _attitude_target;
               error_quat.to_axis_angle(att_error);
             ★ ArduPilot 的 Quaternion::to_axis_angle 内部同样
               按「轴向 + 角度取正」规范化，效果等价。
""")

    # ---------------------------------------------------------------- ④
    sub("④ ★ PX4 的写法和教科书写法：差一个负号 + 一个标量因子")

    print("""    两家的关系是纯代数的。记当前姿态 q、期望姿态 q_d：

        教科书：e_rv = log(R_d^T R)         ω_sp = −K_att · e_rv
        PX4   ：e_q  = 2·vec(q^{-1} ⊗ q_d)  ω_sp = +K_att · e_q

    ★ 第一步：方向。(q^{-1} ⊗ q_d)^{-1} = q_d^{-1} ⊗ q，而单位四元数
      取逆就是把虚部反号 ⇒ 两个误差向量【严格反向】。
      R_d^T R = (R^{-1} R_d)^{-1} 是同一件事的矩阵版。

    ★ 第二步：模长。设真实夹角为 θ，则

        ‖e_rv‖ = θ                     （旋转向量的定义）
        ‖e_q ‖ = 2·sin(θ/2)            （四元数标量部 = cos(θ/2)）

    ⇒ ★★ 所以完整的关系是

        e_q = − (2·sin(θ/2) / θ) · e_rv

      括号里的因子从 θ→0 时的 1 单调降到 θ=π 时的 2/π ≈ 0.637。
      **两者只在 θ→0 的极限下严格相等。**
""")

    rng = np.random.RandomState(20260916)
    worst = 0.0
    worst_dir = 0.0
    worst_th = 0.0
    for _ in range(4000):
        q1 = q_norm(rng.randn(4))
        q2 = q_norm(rng.randn(4))
        a = err_px4(q1, q2)
        b = err_rotvec(q1, q2)
        th = geodesic_angle(q1, q2)
        if th < 1e-9:
            continue
        k = 2.0 * math.sin(th / 2.0) / th
        worst = max(worst, float(np.max(np.abs(a + k * b))))
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na * nb > 1e-18:
            ca = float(np.dot(a, b)) / (na * nb)
            worst_dir = max(worst_dir,
                            math.degrees(math.acos(max(-1.0, min(1.0, ca)))))
        worst_th = max(worst_th, math.degrees(th))

    print("    随机 4000 组姿态对，验证 e_q + (2·sin(θ/2)/θ)·e_rv = 0：")
    print()
    print("      max ‖e_q + k·e_rv‖∞ = %.3e" % worst)
    print("      max ∠(e_q, e_rv)     = %.4f°" % worst_dir)
    print("      （覆盖到的最大夹角 = %.1f°）" % worst_th)
    print()
    print("""    ★ 第一个残差只有 1e-8 量级（相对误差 ~3e-9），第二个恒为 180°，
      ⇒ 上式是严格的恒等式。
      （θ → π 附近 acos 的条件数退化，所以残差没有低到 1e-15。）
""")

    print("""    ⇒ ★★ 所以「教科书公式里那个莫名其妙的负号」不是笔误，
      而是「误差取的是 q_d^{-1}⊗q 还是 q^{-1}⊗q_d」这个选择的必然结果。
      PX4 把负号吸收进了误差定义里。

      ★ 至于那个 2·sin(θ/2) 的因子，PX4 的代码注释（:86）知道得很清楚：

          // using sin(alpha/2) scaled rotation axis as attitude error

      ★★ 它带来了一个免费的好处：**大角度时等效增益自动收窄**，
        从 1 倍平滑降到 0.637 倍。所以 PX4 的姿态环在大角度
        机动时天生比纯线性律温和一点，不需要额外做增益调度。
        ★ 代价是 K_att 的物理含义不再是干净的「rad/s per rad」，
          而是「rad/s per 2·sin(rad/2)」。
""")

    print("""    ⇒ ★ 一条实践结论：

      **姿态误差用旋转向量（轴角），内环用角速率，这是唯一
      在全局（除 θ = π 一个点外）都成立、而且物理含义干净的组合。**
      欧拉角只在「小角度、单轴、人手调参」的场合勉强能用。
""")

    # ---- 图 --------------------------------------------------------
    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 3, figsize=(16, 4.2))

            # (a) θ vs 2sin(θ/2) vs ‖欧拉差‖
            ths = np.linspace(0.0, math.pi, 300)
            axes[0].plot(np.degrees(ths), np.degrees(ths),
                         label="theta  (rotation vector)")
            axes[0].plot(np.degrees(ths),
                         np.degrees(2 * np.sin(ths / 2.0)),
                         label="2*sin(theta/2)  (quaternion)")
            axes[0].plot([0, 180], [0, 180], "k:", lw=0.8)
            axes[0].set_xlabel("true angle theta (deg)")
            axes[0].set_ylabel("error vector norm (deg)")
            axes[0].set_title("two error magnitudes")
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(fontsize=8)

            # (b) 1/cos(theta)
            tt = np.linspace(0, 89.9, 400)
            axes[1].plot(tt, 1.0 / np.cos(np.radians(tt)))
            axes[1].set_yscale("log")
            axes[1].set_xlabel("pitch (deg)")
            axes[1].set_ylabel("1/cos(pitch)")
            axes[1].set_title("euler-rate singularity")
            axes[1].grid(True, alpha=0.3, which="both")

            # (c) 两种写法的响应
            for mode, lbl in (("px4", "rotation vector"),
                              ("euler", "euler difference")):
                c = AttController(K_att=K_att, K_rate=K_rate, att_mode=mode,
                                  q_des=qd_hp, body=body)
                lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0_hp, body=body)
                qs = np.stack([lg["qw"], lg["qx"], lg["qy"], lg["qz"]], axis=1)
                axes[2].plot(lg["t"], [math.degrees(geodesic_angle(qs[i], qd_hp))
                                       for i in range(len(lg["t"]))], label=lbl)
            axes[2].set_xlabel("time (s)")
            axes[2].set_ylabel("attitude error (deg)")
            axes[2].set_title("90 deg yaw at 85 deg pitch")
            axes[2].grid(True, alpha=0.3)
            axes[2].legend(fontsize=8)

            fig.tight_layout()
            fn = os.path.join(HERE, "pid_lesson17_exp1.png")
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:                              # noqa: BLE001
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 实验 2  ★ 单轴姿态控制：把飞机从倾斜拉回水平
# ===========================================================================

def _log_to_quat(lg):
    return np.stack([lg["qw"], lg["qx"], lg["qy"], lg["qz"]], axis=1)


def _err_series(lg, qd):
    return np.array([geodesic_angle(q, qd) for q in _log_to_quat(lg)])


def _roll_series(lg):
    """横滚角（度）。单轴机动时它是最干净的「被控量」。"""
    return np.array([math.degrees(q_to_euler(q)[0]) for q in _log_to_quat(lg)])


def _step_metrics(lg, qd, tail=0.05):
    """二阶阶跃响应指标。

    ★★ 注意这里不能用「测地线夹角」当被控量 —— 夹角恒 ≥ 0，
      冲过头之后它会【重新变大】，但永远不会变成负的，
      所以量不出超调。
      ⇒ 单轴机动必须用【带符号】的横滚角做阶跃响应。
    """
    t = lg["t"]
    x = _roll_series(lg)                       # 带符号的横滚角（度）
    r = 0.0                                    # 目标是水平
    n = len(t)
    e = x - r
    e0 = float(e[0])
    if abs(e0) < 1e-9:
        return {"overshoot": 0.0, "settle": 0.0, "rise": 0.0,
                "ss": 0.0, "peak_err": 0.0}
    s = 1.0 if e0 > 0 else -1.0
    # 超调：越过目标之后，反向偏离的最大值 / 初始偏差
    overshoot = max(0.0, float(np.max(-s * e)) / abs(e0) * 100.0)
    band = 0.02 * abs(e0)
    outside = np.where(np.abs(e) > band)[0]
    settle = float(t[outside[-1]]) if len(outside) else 0.0
    i10 = np.where(s * e <= 0.9 * abs(e0))[0]
    i90 = np.where(s * e <= 0.1 * abs(e0))[0]
    rise = (float(t[i90[0]] - t[i10[0]])
            if len(i10) and len(i90) else float("nan"))
    ss = float(np.mean(e[int((1 - tail) * n):]))
    return {"overshoot": overshoot, "settle": settle, "rise": rise,
            "ss": ss, "peak_err": float(np.max(np.abs(e)))}


def exp2():
    head("实验 2  ★ 单轴姿态控制：从 30° 倾斜回到水平")

    body = RigidBody()
    q0 = q_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(30.0))
    qd = np.array([1.0, 0.0, 0.0, 0.0])
    tau_max = norm_torque_scale()      # ③ 和 ④ 共用，两处必须是同一个数

    print("""
    现在有了正确的误差定义，可以设计控制律了。完整的链条是：

        ┌──────────────────────────────────────────────┐
        │  外环：姿态误差 → 角速率指令                   │
        │        ω_sp = K_att · e_att                  │
        ├──────────────────────────────────────────────┤
        │  内环：角速率误差 → 力矩                       │
        │        M = K_rate · (ω_sp − ω) − K_d · ω̇ ... │
        ├──────────────────────────────────────────────┤
        │  被控对象：J ω̇ = M − ω × (Jω)                 │
        └──────────────────────────────────────────────┘

    ★ 本实验只激励【一个轴】（横滚），所以 ω × (Jω) ≡ 0 ——
      这是把问题降成一维、好看清楚每一环作用的标准手法。
      三轴耦合留到实验 3。

    ▸ 场景：初始横滚 30°，目标水平，J = (%.4f, %.4f, %.4f) kg·m²
""" % (body.J[0, 0], body.J[1, 1], body.J[2, 2]))

    # ---------------------------------------------------------------- ①
    sub("① 外环 P + 内环 P：一个二阶系统的诞生")

    print("""    ★★ 先做解析推导（这一步不能跳）。

    无扰时 J·ω̇ = M。设内环用纯 P：

        M = K_rate · (ω_sp − ω)                                (17.1)

    代入转动方程：

        J·ω̇ = K_rate · (ω_sp − ω)
        ⇒ ω̇ = (K_rate/J)·ω_sp − (K_rate/J)·ω             (17.2)

    令

        λ ≜ K_rate / J                                        (17.3)

    则 (17.2) 是 **一阶线性微分方程**：

        ω̇ + λ·ω = λ·ω_sp                                      (17.4)

    ⇒ 内环闭环传函

        ω(s)        λ
        ──── = ─────────                                       (17.5)
        ω_sp(s)   s + λ

    ★ 极点 −λ 在左半平面（K_rate > 0, J > 0），所以内环【无条件稳定】。
      λ 的单位是 1/s，它就是内环闭环带宽。

    现在外套一个纯 P 的外环：

        ω_sp = K_att · e_att                                  (17.6)

    单轴小角度下 e_att ≈ 角度误差：

        e_att ≈ θ_d − θ，  且 θ̇ = ω                      (17.7)

    把 (17.7) 代入 (17.6) 再代入 (17.4)：

        θ̈ + λ·θ̇ = λ·K_att·(θ_d − θ)
        ⇒ θ̈ + λ·θ̇ + λ·K_att·θ = λ·K_att·θ_d          (17.8)

    ⇒ ★★ **整环仍然是二阶系统！** 对照标准形式
      θ̈ + 2ζω_n θ̇ + ω_n² θ = ω_n² θ_d：

        ω_n = sqrt(λ · K_att)                                 (17.9)
        2ζω_n = λ  ⇒  ζ = λ / (2·ω_n) = 0.5·sqrt(λ / K_att)  (17.10)

    ★★ 这两行是第 17 周最重要的两个公式：

      · ω_n 由【内环极点 × 外环增益】的几何平均决定
      · ζ   只由【内环比外环快多少】决定 —— 这就是实验 16.4
        说的「带宽分离」，在这里第一次有了闭式表达

    ★ 反解出整定公式（实验 4 会用）：

        K_rate = 2·ζ·ω_n·J                                    (17.11)
        K_att  = ω_n / (2·ζ)                                  (17.12)
""")

    K_rate0 = 2.0 * 0.7 * 8.0 * body.J[0, 0]
    K_att0 = 8.0 / (2.0 * 0.7)
    print("    ▸ 取 ω_n = 8 rad/s、ζ = 0.7、J = %.4f：" % body.J[0, 0])
    print("       K_rate = 2·ζ·ω_n·J = %.4f N·m/(rad/s)" % K_rate0)
    print("       K_att  = ω_n/(2·ζ)  = %.4f (rad/s)/rad" % K_att0)
    print("       λ = K_rate/J = %.2f rad/s，λ/ω_n = %.2f（带宽分离比）"
          % (K_rate0 / body.J[0, 0], (K_rate0 / body.J[0, 0]) / 8.0))
    print()

    rows = []
    slow_poles = []
    for ratio in (2.0, 4.0, 8.0, 16.0):
        lam = ratio * 8.0
        Kr = lam * body.J[0, 0]
        c = AttController(K_att=8.0 / (2 * 0.7), K_rate=Kr,
                          att_mode="px4", q_des=qd, body=body)
        lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0, body=body)
        m = _step_metrics(lg, qd)
        ze = 0.5 * math.sqrt(lam / (8.0 / (2 * 0.7)))
        # ★ ω_n 不是固定的：由 (17.9) ω_n = √(λ·K_att)，λ 涨它也涨
        wn = math.sqrt(lam * (8.0 / (2 * 0.7)))
        rows.append(["%.2f" % lam, "%.2f" % wn, "%.2f" % ze,
                     "%.1f%%" % m["overshoot"],
                     "%.0f%%" % overshoot_theory(ze),
                     "%.0f" % (m["settle"] * 1e3),
                     "%.4f" % m["ss"]])
        # 闭环特征方程 s² + λs + λ·K_att 的两个根里「最慢」的那个：
        #   λ > 4K_att ⇒ 两个实根，取模小的；λ < 4K_att ⇒ 复根，衰减率 = ζω_n
        disc = lam * lam - 4.0 * lam * (8.0 / (2 * 0.7))
        slow_poles.append((lam - math.sqrt(disc)) / 2.0 if disc > 0.0
                          else ze * wn)
    print("    ★ 固定外环增益 K_att = %.4f 不动，只改内环极点 "
          "λ = K_rate/J" % K_att0)
    print()
    tab(rows, ["λ (rad/s)", "ω_n 实际 (rad/s)", "ζ_att 解析", "超调(实测)",
               "超调(二阶解析)", "2% 调节 (ms)", "稳态误差 (°)"])
    ze_first = 0.5 * math.sqrt(16.0 / K_att0)
    ze_last = 0.5 * math.sqrt(128.0 / K_att0)
    settle_first = float(min(r[5] for r in rows))
    poles_txt = " / ".join("%.2f" % p for p in slow_poles)
    poles_ts = " / ".join("%.2f" % (4.0 / p) for p in slow_poles)
    settle_txt = " / ".join(r[5] for r in rows)

    print("""    ★★ 读表（这张表就是在验证 (17.9)(17.10)）：

      · ★★ 先看第 1、2 列：K_att 固定时，**ω_n = √(λ·K_att) 会跟着
        λ 一起涨**（9.56 → 27.04 rad/s）—— 这正是 (17.9)。
        同时 ζ_att = λ/(2·ω_n) = 0.5·√(λ/K_att) 从 %.2f 涨到 %.2f：
        **内环越快，外环阻尼越好** —— 这就是 (17.10)。
        ⇒ ⚠️ 所以「固定 ω_n、只改内环」这句话本身是矛盾的：
          K_att 不动时 ω_n 一定会动。要真的固定 ω_n，
          必须让 K_att 和 K_rate 按 (17.11)(17.12) 一起动（见下面 ①b）。

      · 实测超调和二阶解析【逐行一致】（差不超过 1 个百分点）。
        ⇒ ★★ 二阶模型在这个系统上是可用的设计工具，不是装饰品。

      · 但调节时间在 λ 最小时最短（%.0f ms），λ 再加大反而变慢
        （%s ms）。★ 用一个「慢极点」就能解释：
        闭环特征方程 s² + λs + λ·K_att 的两个根里，慢的那个是

            s_slow = (−λ + √(λ² − 4λ·K_att)) / 2
                   ⟶ −K_att   （当 λ ≫ 4·K_att）

        本表四行的 |s_slow| = %s rad/s，取 4/|s_slow| 得
        %s s —— 和实测调节时间 %s ms 对得上。

      ⇒ ★★★ **内环再快，也只能把慢极点推到「外环增益 K_att」为止。**
        想更快，唯一的办法是加大 K_att；而加大 K_att 又会
        破坏阻尼（ζ = 0.5·√(λ/K_att) 变小），所以必须同时加快内环。
        ⇒ ★★ 这就是串级姿态控制的核心结论：
        **外环的阻尼不是靠「外环加 D」得来的，
          而是靠「内环比外环快」得来的。**
        ★★ 这一条直接决定了整定顺序：**先内环、后外环。**
""" % (ze_first, ze_last, settle_first, settle_txt,
       poles_txt, poles_ts, settle_txt))

    sub("①b ★ 扫 ζ_att：二阶解析式和实测到底差多少")

    wn_fix = 8.0
    rows = []
    for ze in (0.3, 0.5, 0.7, 1.0, 1.5, 2.0):
        Kr = 2.0 * ze * wn_fix * body.J[0, 0]
        Ka = wn_fix / (2.0 * ze)
        c = AttController(K_att=Ka, K_rate=Kr, att_mode="px4",
                          q_des=qd, body=body)
        lg = rigid_sim(c, T=5.0, dt=1.0 / 1000, q0=q0, body=body)
        m = _step_metrics(lg, qd)
        lam = Kr / body.J[0, 0]
        rows.append(["%.2f" % ze, "%.2f" % lam, "%.2f" % (lam / wn_fix),
                     "%.4f" % Kr, "%.4f" % Ka,
                     "%.1f%%" % m["overshoot"],
                     "%.1f%%" % overshoot_theory(ze),
                     "%.0f" % (m["settle"] * 1e3),
                     "%.0f" % (m["rise"] * 1e3),
                     "%.4f" % m["ss"]])
    print("    ★ 固定 ω_n = %.0f rad/s、J = %.4f，按 (17.11)(17.12) "
          "由 ζ 反推两个增益"
          % (wn_fix, body.J[0, 0]))
    print()
    tab(rows, ["ζ_att 目标", "λ (rad/s)", "λ/ω_n", "K_rate", "K_att",
               "超调(实测)", "超调(二阶解析)", "2% 调节 (ms)", "上升 (ms)",
               "稳态误差 (°)"])

    print("""    ★★ 读表：

      · ★★ 六个 ζ 值上，**实测超调和解析式逐位吻合**
        （37.3/37.2、16.3/16.3、4.6/4.6、0/0、0/0、0/0）。
        ⇒ 二阶模型在这个系统上是【定量的】设计工具，不是装饰。
        ★ 这是「姿态误差用旋转向量 + 内环是角速率」这套结构的红利：
          在单轴小角度下它就是线性的二阶系统。

      · ζ ≥ 1.5：超调为 0（过阻尼），但【上升时间从 266 ms 涨到 1031 ms】——
        这就是 K_att = ω_n/(2ζ) 在 ζ 变大时变小、外环变软的直接后果。
        ★ 注意 ω_n 一直没变：**「过阻尼」和「慢」是两件事，
          在 ω_n 固定时它们是同一件事的两面。**

      · 稳态误差列全是 0.0000 —— 因为这一小节还是没有扰动的理想情况。
        有扰动时会怎样？见下面 ②。

      ⇒ ★★ 工程甜点区就是 **ζ_att ≈ 0.7 ~ 1.0**：
        超调 5% 以内、上升时间最短、而且二阶解析式还准
        （可以放心用它反推增益）。
        ★ 两家飞控的默认参数落在哪里？见实验 4 ——
          那里会把 PX4/AP 的默认增益按同一套公式反推成 (ω_n, ζ)，
          再和这张表对照。
""")

    # ---------------------------------------------------------------- ②
    sub("② ★ 内环只有 P 时，存在稳态姿态误差")

    print("""    ★★ 上面所有推导都假设「无扰」。现在加一个常值扰动力矩 M_d。

    稳态时 ω̇ = 0、ω = 0，由 (17.1)：

        0 = K_rate·(ω_sp − 0) + M_d
        ⇒ ω_sp = −M_d / K_rate                                (17.13)

    外环稳态：ω_sp = K_att·e_att ⇒

        e_att(∞) = −M_d / (K_rate · K_att)                    (17.14)

    ★★ 换成物理话：**飞机稳稳地停在一个错误的倾角上**，
      而且它不振荡、不漂移，看起来「稳得很」。
""")

    Md = 0.02                      # N·m，相当于 2 g 左右的偏心
    print("    ▸ 扰动力矩 M_d = %.3f N·m，ω_n = %.0f rad/s，ζ = %.2f"
          % (Md, wn_fix, 0.7))
    print()

    Kr = 2.0 * 0.7 * wn_fix * body.J[0, 0]
    Ka = wn_fix / (2.0 * 0.7)
    pred_deg = math.degrees(Md / (Kr * Ka))
    # ★ 积分增益照抄 PX4 的 I/P 比（MC_ROLLRATE_I=0.2 / P=0.15）
    Ki = (0.2 / 0.15) * Kr
    print("    ▸ 积分增益照抄 PX4：I/P = 0.2/0.15 ⇒ Ki = %.5f"
          "（积分时间 K_rate/Ki = %.2f s）" % (Ki, Kr / Ki))
    print("    ▸ 仿真时长 8 s ≈ %.0f 个积分时间常数"
          % (8.0 / (Kr / Ki)))
    print()
    rows = []
    got = []
    for mode, ki, ilim, label in (
            ("P", 0.0, None, "内环只用 P"),
            ("PID", Ki, None, "内环加 I（无积分限幅）"),
            ("PID", Ki, 4.0 * Md, "内环加 I（限幅 4·M_d）")):
        c = AttController(K_att=Ka, K_rate=Kr, att_mode="px4", q_des=qd,
                          body=body, rate_mode=mode, Ki=ki, i_limit=ilim)
        lg = rigid_sim(c, T=8.0, dt=1.0 / 1000, q0=q0, body=body, Md=(Md, 0, 0))
        m = _step_metrics(lg, qd, tail=0.05)
        got.append((m["ss"], float(c._int[0])))
        rows.append([label, "%.4f" % m["ss"],
                     "%.4f" % pred_deg if ki == 0 else "0",
                     "%.5f" % float(c._int[0]),
                     "%.5f" % (-Md)])
    tab(rows, ["配置", "稳态误差 (°)", "解析 M_d/(K_rate·K_att) (°)",
               "内环 I 终值 (N·m)", "−M_d (N·m)"])

    print("""    ★★ 读表：

      · 第 1 行：稳态误差 %.4f°，和解析式 %.4f° 对得上
        （相对差 %.2f%%）。
        ★★ 这个数字看上去不大，但注意它是**一个常值偏差**：
        飞机真的就歪着悬停。M_d = %.2f N·m 只相当于
        几克的重心偏移。

      · 第 2、3 行：加上积分之后误差被压到 %.4f°，
        而且 **I 的终值 %.5f N·m 恰好等于前一行那个解析误差
        所对应的 −M_d = %.3f N·m**
        —— 积分器把自己变成了一个「自动配平舵」，
        它记住的正是扰动力矩本身。
        ★ 限幅对终值没影响（4·M_d 够用），它影响的是抗饱和的过程。

      ⇒ ★★ 这就是为什么**速率环必须有 I**：
        不是为了消跟踪误差（速率指令本来就是被外环不断改写的），
        而是为了让【外环的稳态】成立 ——
        外环要的是「ω = ω_sp」，只有 I 能保证这一点在有扰动时也成立。

      ⇒ ★ 而姿态环（外环）通常【没有 I】：
        PX4 的 MC_ROLL_P / AP 的 ATC_ANG_RLL_P 都是纯 P。
        因为内环的 I 已经把 ω_sp 兑现了，外环再积分就是重复劳动，
        只会引入一个额外的慢极点、拖慢整个环。
""" % (got[0][0], pred_deg,
       abs(got[0][0] - pred_deg) / pred_deg * 100.0,
       Md, got[1][0], got[1][1], -Md))

    # ---------------------------------------------------------------- ③
    sub("③ ★★ 内环加 D：它抑制的不是角度，而是【未建模的快动态】")

    print("""    ★ 把内环从 P 改成 PD（这就是 PX4 的完整形式）：

        M = K_rate·(ω_sp − ω) − K_d·ω̇                        (17.15)

    注意 PX4 里减的是 angular_accel —— 见 rate_control.cpp:78：

        torque = _gain_p.emult(rate_error) + _rate_int
                 − _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

    ★★ 理论上的作用：把 (17.15) 代进 J·ω̇ = M，

        J·ω̇ = K_rate·(ω_sp − ω) − K_d·ω̇
        ⇒ (J + K_d)·ω̇ = K_rate·(ω_sp − ω)
        ⇒ ω̇ = (K_rate/(J + K_d))·(ω_sp − ω)                 (17.16)

    ⇒ ★★ 内环的极点从

        λ = K_rate / J      变成      λ = K_rate / (J + K_d)

      **D 项等价于把转动惯量「变大」了。**

      ★ 这和第 16 周实验 1 ② 的结论是同一件事：
        「D 项扮演阻尼」= 「D 项吃掉了有效惯量」。
        而且这里立刻能看出第 16 周那道 kd < J 的墙是怎么来的：
        K_d 越大，等效惯量越大，内环就越慢；
        离散化之后 K_d > J 会让内环的极点跨过单位圆。
""")

    KDS = (0.0, 0.002, 0.005, 0.008, 0.012)
    print("    ▸ 表 A：**理想内环**（τ_a = 0），固定 K_rate = %.4f，只扫 K_d"
          % Kr)
    print("      （J = %.4f，K_d = J 是第 16 周那道墙）" % body.J[0, 0])
    print()
    rows_a = []
    got_a = []
    for Kd in KDS:
        c = AttController(K_att=Ka, K_rate=Kr, att_mode="px4", q_des=qd,
                          body=body, rate_mode="PD", Kd=Kd,
                          d_lpf_hz=30.0)
        lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0, body=body)
        m = _step_metrics(lg, qd)
        lam_eff = Kr / (body.J[0, 0] + Kd)
        got_a.append((m["overshoot"], m["settle"], lam_eff))
        rows_a.append(["%.4f" % Kd, "%.2f" % (Kd / body.J[0, 0]),
                       "%.2f" % lam_eff,
                       "%.1f%%" % m["overshoot"],
                       "%.0f" % (m["settle"] * 1e3),
                       "%.2f" % float(np.max(np.abs(lg["wx"]))),
                       "%.4f" % m["ss"]])
    tab(rows_a, ["K_d", "K_d/J", "λ_eff = K_rate/(J+K_d)", "超调(实测)",
                 "2% 调节 (ms)", "峰值角速率 (rad/s)", "稳态误差 (°)"])

    wn_here = math.sqrt(Kr * Ka / body.J[0, 0])
    print("""    ★★ 读表 A（这个结论有点反直觉，请仔细看）：

      · K_d 从 0 加到 J：λ_eff 从 %.2f 掉到 %.2f（内环变慢），
        而 ★ **超调从 %.1f%% 涨到 %.1f%%、调节时间从 %.0f ms 拖到 %.0f ms**。
        —— 加「阻尼」反而更容易冲过头、而且更慢。
      · 最后一行 K_d = 1.5·J：λ_eff = %.2f rad/s，
        已经**低于外环的 ω_n = %.2f rad/s**，带宽分离彻底塌掉。
        ★★ 这就是第 16 周那道 kd < J 的墙在【姿态控制】里的样子。

      ⇒ ★★ 为什么？请回想上面的模型：
        在这个仿真里内环**没有任何未建模的快动态**，
        K_d 唯一能做的事就是 (17.16) —— 把惯量从 J 变成 J + K_d。
        惯量变大 ⇒ λ 变小 ⇒ ζ_att = 0.5·√(λ/K_att) 变小 ⇒ 超调变大。
        ★ 结论：**在理想模型里，D 项只会帮倒忙。**

      ⇒ ★★ 那 PX4 为什么还要 D？因为真实内环【不是理想的】。
        D 项真正抑制的，是那些被我们扔出模型的东西：
        电机/桨叶的升速滞后、D 项自身的滤波器、计算与通信延迟。
        ★ 下面这张表把它们加回来，D 项立刻换了张脸。
""" % (got_a[0][2], got_a[-1][2],
       got_a[0][0], got_a[-1][0],
       got_a[0][1] * 1e3, got_a[-1][1] * 1e3,
       got_a[-1][2], wn_here))

    # ---- 表 B：带执行器滞后、且增益推高 ------------------------------
    tau_a_demo = 0.02                     # 20 ms，对应 1/(2π·0.02) ≈ 8.0 Hz
    wn_hi = 40.0
    Kr_hi = 2.0 * 0.7 * wn_hi * body.J[0, 0]
    Ka_hi = wn_hi / (2.0 * 0.7)
    print("    ▸ 表 B：**内环带一阶执行器滞后** τ_a = %.0f ms"
          % (tau_a_demo * 1e3))
    print("      （折点 1/(2π·τ_a) = %.1f Hz），并且增益推到 ω_n = %.0f rad/s"
          % (1.0 / (2 * math.pi * tau_a_demo), wn_hi))
    print("      ⇒ K_rate = %.4f，K_att = %.4f，τ_a 的相位滞后不再可忽略"
          % (Kr_hi, Ka_hi))
    print()
    rows_b = []
    got_b = []
    KDS_B = (0.0, 0.002, 0.005, 0.008, 0.012, 0.018)
    for Kd in KDS_B:
        c = AttController(K_att=Ka_hi, K_rate=Kr_hi, att_mode="px4",
                          q_des=qd, body=body, rate_mode="PD", Kd=Kd,
                          d_lpf_hz=30.0)
        lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0, body=body,
                       tau_act=tau_a_demo)
        m = _step_metrics(lg, qd)
        got_b.append((m["overshoot"], m["settle"]))
        rows_b.append(["%.4f" % Kd, "%.2f" % (Kd / body.J[0, 0]),
                       "%.2f" % (Kr_hi / (body.J[0, 0] + Kd)),
                       "%.1f%%" % m["overshoot"],
                       "%.0f" % (m["settle"] * 1e3),
                       "%.2f" % float(np.max(np.abs(lg["wx"])))])
    tab(rows_b, ["K_d", "K_d/J", "λ_eff (rad/s)", "超调(实测)",
                 "2% 调节 (ms)", "峰值角速率 (rad/s)"])
    i_best = int(np.argmin([g[0] for g in got_b]))
    print("""    ★★ 读表 B（同一套指标，完全不同的形状）：

      · K_d = 0（只有 P）：超调 %.1f%%、调节 %.0f ms ——
        ★ τ_a 的相位滞后把增益推高之后的裕度吃光了。
      · K_d 加到 %.4f：超调掉到 **%.1f%%**、调节 **%.0f ms** ——
        ★★ 这就是 D 项「扮演阻尼」的真实面目：
        它抑制的是 **τ_a 这个未建模的一阶滞后**。
      · 再加到 %.4f：又反弹回 %.1f%% / %.0f ms ——
        因为 (17.16) 的副作用开始占上风，λ_eff 掉到 %.2f rad/s。

      ⇒ ★★★ 所以 D 项有一个**最优值**，而且这个最优值由
        「未建模快动态有多快」决定 —— 也就是由 τ_a 决定。
        τ_a 越小（电调/桨越猛），最优 K_d 就越小。

      ⇒ ★★ 这一下就把 PX4 的默认值解释清楚了。
        ★ MC_ROLLRATE_D 的单位是【归一化力矩 / (rad/s²)】，
          要换成 N·m·s² 必须乘上「归一化力矩 1.0 = 多少 N·m」。
          用本脚本现算的 %.6f N·m（mc_rate_control_params.c:88）：

              MC_ROLLRATE_D = 0.003 → %.6f N·m·s²

        ★★ 而 J = %.4f kg·m²，于是

              K_d / J = %.3f   （第 16 周那道墙在 K_d/J = 1）

        —— **PX4 的默认 D 只有墙的 1/3 左右**。
        ★★ 这不是巧合：D 项的可调范围本来就只剩「0 到 J」这么一小段，
          所以它**只能是小增益**。第 16 周实验 3 ④ 定量算过这一点。

      ⇒ ★★ 这也是第 16 周那句「不要用微分」的第二种含义：
        在串级结构里，阻尼的主要来源是**内环的 P 本身**
        （它给的是 λ = K_rate/J），D 项只做微调。
""" % (got_b[0][0], got_b[0][1] * 1e3,
       KDS_B[i_best], got_b[i_best][0], got_b[i_best][1] * 1e3,
       KDS_B[-1], got_b[-1][0], got_b[-1][1] * 1e3,
       Kr_hi / (body.J[0, 0] + KDS_B[-1]),
       tau_max, 0.003 * tau_max, body.J[0, 0], 0.003 * tau_max / body.J[0, 0]))

    # ---------------------------------------------------------------- ④
    sub("④ ★ 峰值力矩：控制律给出的 M 真的能实现吗")

    print("""    以上所有分析都假设「力矩想要多少就有多少」。这当然是假的：
    力矩来自四个电机的推力差，物理上界由力臂、推力和电机响应决定。

    ★ 用第 3 章的混控矩阵（QUAD_X，1 kg 机、0.25 m 力臂、推重比 2:1）
      现算一遍：`norm_torque_scale()` 返回 %.6f N·m
      （和第 16 周实验 1 ② 是同一段计算，同一个数）。
""" % tau_max)
    print("    ▸ 假设力矩上限 τ_max = %.4f N·m（逐轴硬限幅）" % tau_max)
    print()
    rows = []
    pk_list = []
    wn_list = (6.0, 8.0, 12.0, 16.0, 24.0)
    for wn_v in wn_list:
        Kr_v = 2.0 * 0.7 * wn_v * body.J[0, 0]
        Ka_v = wn_v / (2.0 * 0.7)
        c = AttController(K_att=Ka_v, K_rate=Kr_v, att_mode="px4",
                          q_des=qd, body=body, M_max=tau_max)
        lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0, body=body)
        m = _step_metrics(lg, qd)
        pkM = float(np.max(np.abs(lg["Mx"])))
        pk_list.append(pkM)
        sat = pkM >= tau_max - 1e-9
        rows.append(["%.0f" % wn_v, "%.1f" % (wn_v / (2 * math.pi)),
                     "%.4f" % Kr_v, "%.4f" % Ka_v,
                     "%.4f" % pkM, "★ 是" if sat else "否",
                     "%.0f" % (m["settle"] * 1e3)])
    tab(rows, ["ω_n (rad/s)", "闭环带宽 (Hz)", "K_rate", "K_att",
               "峰值 |Mx| (N·m)", "触发限幅", "2% 调节 (ms)"])

    sat_idx = [i for i, p in enumerate(pk_list) if p >= tau_max - 1e-9]
    i_first_sat = sat_idx[0] if sat_idx else len(wn_list) - 1
    free_txt = "、".join("%.0f" % wn_list[i] for i in range(i_first_sat))
    sat_txt = "、".join("%.0f" % wn_list[i] for i in sat_idx)
    free_pk = " / ".join("%.4f" % pk_list[i] for i in range(i_first_sat))
    print("""    ★★ 读表：

      · ω_n = %s rad/s：峰值力矩 %s N·m，
        **都还在 τ_max = %.4f N·m 以下**，线性设计成立，
        实测的调节时间也就对得上二阶解析式（17.9）(17.10)。
      · ω_n = %s rad/s：★ 峰值力矩顶到 τ_max = %.4f N·m（两行数值
        一模一样，就是被限幅削平的痕迹）。
        **「30° 阶跃」变成了一个力矩受限（bang-bang）问题**，
        二阶解析式不再成立 —— 调节时间由力矩上界决定。

      ⇒ ★★ 这是姿态整定里最容易被忽略的一环：
        **你能调多快，先取决于力矩够不够，然后才取决于增益。**
        所以正确的整定顺序是：

          ① 先确认力矩/推力有裕度（τ_max 够不够）
          ② 再按 ζ、ω_n 反推 K（实验 4）
          ③ 最后用飞行日志验证有没有长时间顶到饱和

      ⇒ ★ PX4/ArduPilot 都把「饱和」当成一等公民来处理：
        第 16 周的实验 2 讲的抗饱和（rate_control.cpp:88-118，
        pid.cpp:147-158）就是为这一刻准备的。
""" % (free_txt, free_pk, tau_max, sat_txt, tau_max))

    # ---- 图 --------------------------------------------------------
    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 4, figsize=(20, 4.2))

            for ze in (0.3, 0.5, 0.7, 1.0, 2.0):
                Kr_v = 2.0 * ze * wn_fix * body.J[0, 0]
                Ka_v = wn_fix / (2.0 * ze)
                c = AttController(K_att=Ka_v, K_rate=Kr_v, att_mode="px4",
                                  q_des=qd, body=body)
                lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0, body=body)
                axes[0].plot(lg["t"], np.degrees(_err_series(lg, qd)),
                             label="zeta = %.1f" % ze)
            axes[0].set_xlabel("time (s)")
            axes[0].set_ylabel("attitude error (deg)")
            axes[0].set_title("roll step 30 deg (wn = 8 rad/s)")
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(fontsize=8)

            for lbl, kd_v in (("Kd = 0", 0.0), ("Kd = J", body.J[0, 0])):
                c = AttController(K_att=Ka, K_rate=Kr, att_mode="px4",
                                  q_des=qd, body=body, rate_mode="PD",
                                  Kd=kd_v, d_lpf_hz=30.0)
                lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0, body=body)
                axes[1].plot(lg["t"], np.degrees(_err_series(lg, qd)),
                             label=lbl)
            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("attitude error (deg)")
            axes[1].set_title("ideal inner loop (tau_a=0): Kd only hurts")
            axes[1].grid(True, alpha=0.3)
            axes[1].legend(fontsize=8)

            for lbl, kd_v in (("Kd = 0", 0.0), ("Kd = 0.005", 0.005),
                              ("Kd = 0.018", 0.018)):
                c = AttController(K_att=Ka_hi, K_rate=Kr_hi, att_mode="px4",
                                  q_des=qd, body=body, rate_mode="PD",
                                  Kd=kd_v, d_lpf_hz=30.0)
                lg = rigid_sim(c, T=4.0, dt=1.0 / 1000, q0=q0, body=body,
                               tau_act=tau_a_demo)
                axes[2].plot(lg["t"], np.degrees(_err_series(lg, qd)),
                             label=lbl)
            axes[2].set_xlabel("time (s)")
            axes[2].set_ylabel("attitude error (deg)")
            axes[2].set_title("with tau_a = 20 ms: Kd has an optimum")
            axes[2].grid(True, alpha=0.3)
            axes[2].legend(fontsize=8)

            for wn_v in (6.0, 12.0, 24.0):
                Kr_v = 2.0 * 0.7 * wn_v * body.J[0, 0]
                Ka_v = wn_v / (2.0 * 0.7)
                c = AttController(K_att=Ka_v, K_rate=Kr_v, att_mode="px4",
                                  q_des=qd, body=body, M_max=tau_max)
                lg = rigid_sim(c, T=3.0, dt=1.0 / 1000, q0=q0, body=body)
                axes[3].plot(lg["t"], np.abs(lg["Mx"]),
                             label="wn = %.0f" % wn_v)
            axes[3].axhline(tau_max, color="k", ls=":",
                            label="tau_max = %.2f Nm" % tau_max)
            axes[3].set_xlabel("time (s)")
            axes[3].set_ylabel("|Mx| (Nm)")
            axes[3].set_title("torque saturation vs bandwidth")
            axes[3].grid(True, alpha=0.3)
            axes[3].legend(fontsize=8)

            fig.tight_layout()
            fn = os.path.join(HERE, "pid_lesson17_exp2.png")
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:                              # noqa: BLE001
            print("    [画图失败: %s]" % e)
    print()


def overshoot_theory(zeta):
    """二阶系统阶跃响应的超调百分比。"""
    if zeta <= 0.0 or zeta >= 1.0:
        return 0.0 if zeta >= 1.0 else 100.0
    return 100.0 * math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta * zeta))


# ===========================================================================
# 整定工具：(17.9)(17.10) 的正运算与逆运算
# ===========================================================================

def design_gains(zeta, wn, J):
    """(17.11)(17.12) 正运算：由目标 ζ、ω_n 和惯量 J 反推两个增益。

        K_rate = 2·ζ·ω_n·J
        K_att  = ω_n / (2·ζ)
    """
    return 2.0 * zeta * wn * J, wn / (2.0 * zeta)


def identify_design(K_rate, K_att, J):
    """(17.9)(17.10) 逆运算：由已有增益反推等效 λ、ω_n、ζ。

        λ  = K_rate / J
        ω_n = sqrt(λ · K_att)
        ζ  = λ / (2·ω_n)
    """
    lam = K_rate / J
    wn = math.sqrt(lam * K_att)
    return lam, wn, lam / (2.0 * wn)


# ===========================================================================
# 实验 3  ★★ 三轴耦合与陀螺进动项 ω × (Jω)
# ===========================================================================

def exp3():
    head("实验 3  ★★ 三个轴不是独立的：ω × (Jω) 与陀螺进动")

    body = RigidBody()
    tau_max = norm_torque_scale()
    Jx, Jy, Jz = body.J[0, 0], body.J[1, 1], body.J[2, 2]

    # ---------------------------------------------------------------- ①
    sub("① 叉乘项从哪来：不是力矩，是【转动坐标系的账】")

    print("""    ★★ 前面两个实验都只用了一个轴，所以可以把 J 当标量。
    现在把三轴一起写出来，必须回到最原始的定义（推导不跳步）：

      ▸ 第 1 步：角动量。机体系里的角动量是 J·ω，
        转到惯性系要左乘 R（本教程约定 R = R_{body→ned}）：

            H^n = R · J · ω^b

      ▸ 第 2 步：在惯性系里，外力矩 = 角动量的变化率：

            d/dt (R · J · ω^b) = M^n

      ▸ 第 3 步：用乘积法则展开左边。关键是旋转矩阵的导数

            Ṙ = R · [ω^b ×]        （[·×] 是叉乘反对称矩阵）

        所以

            Ṙ·J·ω + R·J·ω̇ = R·([ω^b ×]·J·ω) + R·J·ω̇
                          = R·( ω^b × (J·ω) + J·ω̇ )

      ▸ 第 4 步：两边左乘 Rᵀ（= R⁻¹），把 M^n 转回机体系：

            J·ω̇ + ω × (Jω) = M                                 (17.17)

    ★★★ 请把 (17.17) 和牛顿第二定律对比着看：

        J·ω̇ = M − ω × (Jω)                                     (17.18)
              └───┘  └────────┘
              你给的    多出来的
              力矩      这一项

      ★ 它**不是**一个真实的力矩，没有反作用对象；
        它是「把 dH/dt 写在转动坐标系里」多出来的修正项，
        物理上叫【陀螺进动项】，也叫欧拉方程里的交叉耦合项。

    ▸ 把 J = diag(Jx, Jy, Jz) 代进叉乘，右边可以展开成纯标量式：

        ω × (Jω) = [ (Jy − Jz)·q·r,
                     (Jz − Jx)·r·p,
                     (Jx − Jy)·p·q ]                           (17.19)

      ★★ 注意每一项都是【两个不同轴的角速率相乘】——
        只要有两个轴同时转，三个轴就耦合在一起了。
        上一节的单轴实验里 p = q = 0，所以 (17.19) ≡ 0，
        这就是为什么当时可以当它不存在。
""")

    # ---------------------------------------------------------------- ②
    sub("② ★ 先定量：这个「多出来的项」到底有多大")

    print("""    本脚本的 J = (%.4f, %.4f, %.4f) kg·m²，其中 Jx = Jy。
    代进 (17.19)：

        ω × (Jω) = [ (Jy − Jz)·q·r,
                     (Jz − Jx)·r·p,
                     (Jx − Jy)·p·q ]
                 = [ %.4f·q·r,  %.4f·r·p,  0 ]                (17.20)

    ★ 第三项恒等于 0（因为 Jx = Jy）—— 说明
      **这个真实的四旋翼模型里，耦合只发生在「偏航 与 横滚/俯仰」之间，
        横滚和俯仰之间没有耦合。** 这是个很有用的结构性事实。

    ▸ 取横滚角速率 = PX4 的上限（MC_ROLLRATE_MAX = 220 °/s），
      偏航角速率 r 从 1 rad/s 扫到能把力矩吃满：**（Jz − Jx）= %.4f**
""" % (Jx, Jy, Jz, Jy - Jz, Jz - Jx, Jz - Jx))
    p_max = math.radians(220.0)
    print("      （p = %.4f rad/s = 220 °/s，来源 mc_att_control_params.c:116"
          % p_max)
    print("        的 MC_ROLLRATE_MAX = 220.0；俯仰上限同值见 :135）")
    print()
    r_yaw_max = math.radians(200.0)
    coef = Jz - Jx
    rows = []
    r_show = (1.0, r_yaw_max, 2 * r_yaw_max, 20.0,
              tau_max / (coef * p_max))
    for r in r_show:
        mc = coef * r * p_max
        rows.append(["%.2f" % r, "%.0f" % math.degrees(r),
                     "%.4f" % mc, "%.2f%%" % (mc / tau_max * 100.0)])
    tab(rows, ["r (rad/s)", "r (°/s)", "|ω×(Jω)| (N·m)",
               "占 τ_max = %.3f 的比例" % tau_max])
    print("""      ★ 第二行的 r = %.2f rad/s 就是 PX4 的偏航角速率上限
        （MC_YAWRATE_MAX = 200 °/s，mc_att_control_params.c:147）。
        ⇒ **在最极端的「横滚满舵 + 偏航满舵」组合下，
          耦合项也只占 τ_max 的 %.1f%%。**

      ★★ 要让耦合项自己就把力矩吃满，需要 r = %.1f rad/s
        = %.0f °/s —— 是 PX4 偏航上限的 %.0f 倍。
        ★ 多旋翼做不到（桨叶转速差不够），
          但**固定翼/火箭/自旋弹**可以，所以那条赛道上
          这一项是必须建模的。
""" % (r_yaw_max,
       coef * r_yaw_max * p_max / tau_max * 100.0,
       tau_max / (coef * p_max), math.degrees(tau_max / (coef * p_max)),
       (tau_max / (coef * p_max)) / r_yaw_max))

    # ---------------------------------------------------------------- ③
    sub("③ ★★ 仿真对照：有没有前馈，差别是多少")

    print("""    ★ 控制律的对策很直接：**把耦合项前馈掉**。
      在 (17.17) 里，只要让内环输出

        M = K_rate·(ω_sp − ω) − K_d·ω̇ + ω × (Jω)                 (17.21)
                                    └──────────┘
                                    控制器主动补上这一项

      代回 (17.18)：

        J·ω̇ = K_rate·(ω_sp − ω) − ω × (Jω) + ω × (Jω)
        ⇒ J·ω̇ = K_rate·(ω_sp − ω)                              (17.22)

      ⇒ ★★★ **三个轴被彻底解耦了**，每个轴都退回到实验 2 的单轴形式。
        这就是所谓「反馈线性化」最简单的样子。

    ▸ 下面是一个真实的耦合场景：

        初始：横滚 30° 倾斜 + 偏航角速率 r = %.2f rad/s（PX4 上限）
        目标：回到水平（偏航速率保持）

      ★★ 三行对照：
        (a) plant 带叉乘、控制器**不**补 —— 真实飞控的做法
        (b) plant 带叉乘、控制器**补上** —— 教科书做法
        (c) plant 不 带叉乘      —— 理想对照（物理上不存在）
""" % r_yaw_max)
    # ★ 注意 design_gains 返回 (K_rate, K_att)
    Kr, Ka = design_gains(0.7, 8.0, Jx)
    q0 = q_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(30.0))
    qd = np.array([1.0, 0.0, 0.0, 0.0])
    w0 = np.array([0.0, 0.0, r_yaw_max])
    cases = [
        ("plant 有叉乘, 控制器不补", False, True),
        ("plant 有叉乘, 控制器补上", True, True),
        ("plant 无叉乘 (理想对照)", False, False),
    ]
    rows = []
    rec = []
    for lab, ff, uc in cases:
        c = AttController(K_att=Ka, K_rate=Kr, att_mode="px4", q_des=qd,
                          body=body, cross_ff=ff)
        lg = rigid_sim(c, T=3.0, dt=1.0 / 1000, q0=q0, w0=w0, body=body,
                       use_cross=uc)
        wy = lg["wy"]
        pit = np.array([abs(math.degrees(q_to_euler(q)[1]))
                        for q in _log_to_quat(lg)])
        rec.append((lab, abs(wy).max(), math.sqrt((wy ** 2).mean()),
                    float(pit.max()), float(np.abs(lg["My"]).max()),
                    float(np.abs(wy).max())))
        rows.append([lab,
                     "%.5f" % abs(wy).max(),
                     "%.5f" % math.sqrt((wy ** 2).mean()),
                     "%.4f" % pit.max(),
                     "%.5f" % np.abs(lg["My"]).max()])
    tab(rows, ["配置", "max|ω_y| (rad/s)", "rms|ω_y| (rad/s)",
               "max 俯仰角 (°)", "峰值 |M_y| (N·m)"])
    print("""    ★★ 读表：

      · 第 1 行 vs 第 3 行：耦合项把横向角速率**峰值**
        从 %.5f 抬到 %.5f rad/s（+%.1f%%），
        横向**姿态**误差峰值抬高 %.4f°。
        ★ 姿态误差被抬高得比角速率少，是因为速率环的 P
          和姿态环的 P 在双重积分里又把它压回去了一道。
      · 第 2 行 vs 第 3 行：★ **加了前馈之后，
        指标几乎逐位回到理想对照**（%.5f vs %.5f rad/s）。
        ⇒ (17.22) 说的「彻底解耦」在数值上是真的。
      · 但注意峰值 |M_y| 那一列：前馈让它是 %.5f，
        反而比不补的 %.5f **更大**。
        ★★ 这不是错 —— 前馈的意义就是「提前用力，别等误差长出来」。
        **代价是瞬时力矩占用变大，收益是横向误差变小。**

      ⇒ ★★ 所以「要不要补这一项」是一个【收益 / 代价】的工程判断，
        不是「对 / 错」的判断。下面看看两家真实飞控怎么选。
""" % (rec[2][2], rec[0][2],
       (rec[0][2] / rec[2][2] - 1.0) * 100.0,
       rec[0][3] - rec[2][3],
       rec[1][2], rec[2][2],
       rec[1][4], rec[0][4]))

    # ---------------------------------------------------------------- ④
    sub("④ ★★ 真实飞控都不补这一项 —— 为什么")

    print("""    ★ 先说事实（这是 grep 出来的，不是印象）：

      ▸ PX4 的内环，src/lib/rate_control/rate_control.cpp:78：

            const Vector3f torque = _gain_p.emult(rate_error) + _rate_int
                                    − _gain_d.emult(angular_accel)
                                    + _gain_ff.emult(rate_sp);

        整个函数（rate_control.cpp:71-86）里**没有 ω、没有 J、没有叉乘**。

      ▸ ArduPilot 的内环，libraries/AC_AttitudeControl/
        AC_AttitudeControl_Multi.cpp:442-467 的 rate_controller_run()：

            _motors.set_roll(get_rate_roll_pid().update_all(
                _ang_vel_body.x, gyro_latest.x, _dt,
                _motors.limit.roll, _pd_scale.x) + _actuator_sysid.x);

        它把速率 PID 的输出**直接送给电机混控**，
        中间也没有任何 ω × (Jω) 的补偿。

    ★★★ 两家都不做，**不是因为忘了**，是因为下面四条理由
       （前三条是本实验可以量化的，第四条是工程现实）：

      ① **收益太小。** 上面那张表：最极端的横滚满舵 + 偏航满舵
         下，耦合项只占 τ_max 的 %.1f%%。而正常飞行中
         横滚和偏航同时满舵的组合非常罕见。

      ② **它需要 J。** (17.21) 里的叉乘要用真实的惯量矩阵。
         J 只能靠系统辨识（悬吊/自由落体/频响扫频）测出来，
         而多旋翼的 J 会随载荷、电池位置变化。
         ★ 拿一个不准的 J 去前馈，可能比不补更糟。

      ③ **速率环自己会吸收它。** 耦合项在 (17.18) 里相当于
         一个**已知结构的扰动力矩**。实验 2 ② 已经证明：
         速率环的 I 项会把常值扰动力矩完全吃掉，
         P 项会把剩余的偏差压到 M_d/(K_rate·K_att)。
         ★ 耦合项虽然不是常值，但它的变化比速率环的带宽慢得多，
           所以 I 项能跟上。

      ④ **它和「角速率上限」互斥。** 由 ① 可知，只有当角速率
         远超 MC_ROLLRATE_MAX / MC_YAWRATE_MAX 时它才重要，
         而那正是姿态环**限幅**（AttitudeControl.cpp:105-107）
         不让飞机进入的区域。

      ⇒ ★★ 所以正确的说法不是「课本错了」，而是：
        **(17.21) 是把模型做全的写法，(17.22) 的收益在这个
          被控对象 + 这个工作点上小到不值得引入 J 的辨识误差。**
        ★ 但如果你做的是自旋弹、倾转旋翼、或者大角速率特技飞行，
          这一项的收益会大到不能忽略 —— 判断依据就是本实验 ② 的算法：
          **先算它占 τ_max 的百分比，再决定要不要补。**
""" % (coef * r_yaw_max * p_max / tau_max * 100.0))

    # ---- 图 --------------------------------------------------------
    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.2))

            for lab, ff, uc, st in (("no feedforward", False, True, "-"),
                                    ("with feedforward", True, True, "--"),
                                    ("ideal (no coupling)", False, False, ":")):
                c = AttController(K_att=Ka, K_rate=Kr, att_mode="px4",
                                  q_des=qd, body=body, cross_ff=ff)
                lg = rigid_sim(c, T=1.5, dt=1.0 / 1000, q0=q0, w0=w0,
                               body=body, use_cross=uc)
                axes[0].plot(lg["t"], lg["wy"], st, label=lab)
                axes[1].plot(lg["t"], np.degrees(_err_series(lg, qd)),
                             st, label=lab)
                axes[2].plot(lg["t"], lg["My"], st, label=lab)
            for ax, ti, yl in ((axes[0], "cross-axis rate wy", "wy (rad/s)"),
                               (axes[1], "attitude error", "error (deg)"),
                               (axes[2], "cross-axis torque My", "My (Nm)")):
                ax.set_xlabel("time (s)")
                ax.set_ylabel(yl)
                ax.set_title(ti)
                ax.grid(True, alpha=0.3)
                ax.legend(fontsize=8)

            fig.tight_layout()
            fn = os.path.join(HERE, "pid_lesson17_exp3.png")
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:                              # noqa: BLE001
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 实验 4  ★★ 整定：从 ζ、ω_n 反推增益，和真实飞控的默认值对照
# ===========================================================================

# ★ 以下默认值全部来自源码，每一行都标了 file:line
PX4_DEFAULTS = {
    "MC_ROLL_P":      (6.5,   "src/modules/mc_att_control/mc_att_control_params.c:53"),
    "MC_ROLLRATE_P":  (0.15,  "src/modules/mc_rate_control/mc_rate_control_params.c:51"),
    "MC_ROLLRATE_I":  (0.2,   "src/modules/mc_rate_control/mc_rate_control_params.c:63"),
    "MC_ROLLRATE_D":  (0.003, "src/modules/mc_rate_control/mc_rate_control_params.c:88"),
    "MC_ROLLRATE_FF": (0.0,   "src/modules/mc_rate_control/mc_rate_control_params.c:99"),
}
AP_DEFAULTS = {
    "ATC_ANG_RLL_P": (4.5,    "libraries/AC_AttitudeControl/AC_AttitudeControl.h:15"),
    "ATC_RAT_RLL_P": (0.135,  "libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h:11"),
    "ATC_RAT_RLL_I": (0.135,  "libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h:14"),
    "ATC_RAT_RLL_D": (0.0036, "libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h:17"),
}


def exp4():
    head("实验 4  ★★ 整定：从 ζ、ω_n 反推增益，与两家飞控的默认值对照")

    body = RigidBody()
    tau_max = norm_torque_scale()
    J = body.J[0, 0]

    # ---------------------------------------------------------------- ①
    sub("① 整定公式与它的正反两个用法")

    print("""    ★★ 实验 2 推出的两条公式，是整个第 17 周最有用的东西：

        ω_n = sqrt(λ · K_att)          λ = K_rate / J          (17.9)
        ζ   = λ / (2·ω_n)                                     (17.10)

    把它解成「已知目标、求增益」的形式（实验 2 已给）：

        K_rate = 2·ζ·ω_n·J                                    (17.11)
        K_att  = ω_n / (2·ζ)                                  (17.12)

    ★ 正用法（设计）：定 ζ、ω_n → 算 K_rate、K_att。
      ★ 注意要输入 J，所以**先得有惯量**。

    ★★ 逆用法（读别人的参数）：拿到 K_rate、K_att → 反推

        λ = K_rate / J,  ω_n = sqrt(λ·K_att),  ζ = λ/(2·ω_n)

      ⇒ **这个方法能把任何一个飞控的默认参数翻译成「多快、多抖」**，
        是本实验 ②③ 要用的核心工具。

    ★★★ J 不知道怎么办？好消息是**它不敏感**。推导一下：
      设真实惯量 J，你按假定值 Ĵ 算出 K_rate = 2ζ₀ω₀Ĵ，于是

        λ  = K_rate / J       = 2ζ₀ω₀·(Ĵ/J)
        ω_n = sqrt(λ·K_att)   = ω₀·sqrt(Ĵ/J)                  (17.23)
        ζ   = λ/(2ω_n)        = ζ₀·sqrt(Ĵ/J)                  (17.24)

      ⇒ ★★ **ω_n 和 ζ 都只按 sqrt(Ĵ/J) 变化** ——
        J 估错一倍，ω_n 和 ζ 也只差 41%。这解释了为什么工程上
        「差不多估个 J 就行」是成立的。
        ★ 顺带解释了另一个现象：**为什么阻尼比和带宽总是绑在一起动？**
          因为在这个结构里 ζ = 0.5·λ/ω_n，而 λ 和 ω_n 都是
          「同一个 K_rate」的函数 —— 想单独调一个，
          只能改 K_att（也就是 ω_n/(2ζ) 里的 ζ）。
""")

    rows = []
    for ratio in (0.4, 0.6, 0.8, 1.0, 1.25, 2.0, 3.0):
        s = math.sqrt(ratio)
        rows.append(["%.2f" % ratio,
                     "%.3f" % s,
                     "%.3f" % s,
                     "%.2f" % (10.0 * s), "%.2f" % (0.7 * s),
                     "%.1f%%" % overshoot_theory(min(0.7 * s, 1.0))
                     if 0.7 * s < 1.0 else "0.0%"])
    print("    ▸ 目标 ω₀ = 10 rad/s、ζ₀ = 0.7，按【假定惯量 Ĵ】整定，")
    print("      但真实惯量是 J（表格按 Ĵ/J 排序）：")
    print()
    tab(rows, ["Ĵ/J", "ω_n/ω₀ = √(Ĵ/J)", "ζ/ζ₀ = √(Ĵ/J)",
               "实际 ω_n (rad/s)", "实际 ζ", "实际超调"])

    print("""    ★★ 读表：

      · 假定惯量偏小（Ĵ/J < 1）⇒ 实际 ω_n 和 ζ 都偏小 ⇒ 变慢、变抖。
      · 假定惯量偏大（Ĵ/J > 1）⇒ 实际 ω_n 和 ζ 都偏大 ⇒ 变快、变保守。
      · ★★ 但变化率只有 sqrt —— **J 差 2.5 倍，ζ 只从 0.7 变成 %.2f**。
        ⇒ 惯量估计的精度要求是「量级对」，不是「小数点后两位」。
        ★ 这也解释了为什么 PX4 有 `MC_ROLLRATE_K` 这种「整体缩放」
          参数（mc_rate_control_params.c:119）——
          它的作用就是在上机之后按 sqrt 关系微调 ω_n 和 ζ。
""" % (0.7 * math.sqrt(2.5)))

    # ---------------------------------------------------------------- ②
    sub("② 把 PX4 的默认参数反推成 (ω_n, ζ)")

    px4_p = PX4_DEFAULTS["MC_ROLL_P"][0]
    px4_rp = PX4_DEFAULTS["MC_ROLLRATE_P"][0]
    px4_ri = PX4_DEFAULTS["MC_ROLLRATE_I"][0]
    px4_rd = PX4_DEFAULTS["MC_ROLLRATE_D"][0]
    px4_ff = PX4_DEFAULTS["MC_ROLLRATE_FF"][0]
    print("""    ▸ 先把四个默认值连出处抄一遍（都不是印象）：

        参数              值       出处
        ────────────────────────────────────────────────────────────────""")
    for k in ("MC_ROLL_P", "MC_ROLLRATE_P", "MC_ROLLRATE_I",
              "MC_ROLLRATE_D", "MC_ROLLRATE_FF"):
        v, src = PX4_DEFAULTS[k]
        print("        %-16s %-8s %s" % (k, "%.3f" % v, src))
    print("""
    ★★ 关键一步：**单位换算。**
      根据 mc_rate_control_params.c:43 的说明，
      MC_ROLLRATE_P 是「角速率误差 1 rad/s 对应的【控制输出】」，
      而 PX4 的控制输出是**归一化力矩**（rate_control.cpp:78 之后
      由控制分配器乘上力臂/推力），所以

        K_rate [N·m/(rad/s)] = MC_ROLLRATE_P × (归一化力矩 1.0 = 多少 N·m)

      ★ 而 MC_ROLL_P（mc_att_control_params.c:45：角误差 1 rad
        对应多少 rad/s 的期望角速率）**本身就是物理单位**，不用换算。

    ▸ 本脚本现算的归一化力矩尺度 τ_scale = %.6f N·m
      （就是实验 2 ④ 用的那个 τ_max，同一段计算）
""" % tau_max)

    Kr_px4 = px4_rp * tau_max
    Ka_px4 = px4_p
    lam_p, wn_p, ze_p = identify_design(Kr_px4, Ka_px4, J)
    rows = [
        ["MC_ROLLRATE_P × τ_scale", "%.4f × %.6f" % (px4_rp, tau_max),
         "%.5f N·m/(rad/s)" % Kr_px4, "内环 P"],
        ["MC_ROLL_P（已是物理单位）", "%.3f" % px4_p,
         "%.4f (rad/s)/rad" % Ka_px4, "外环 P"],
    ]
    tab(rows, ["换算", "原始 × 尺度", "物理值", "作用"], aligns="llrl")
    print("    ▸ 代进 (17.9)(17.10) 的逆运算：")
    print()
    print("        λ  = K_rate / J = %.5f / %.4f = %.4f rad/s"
          % (Kr_px4, J, lam_p))
    print("        ω_n = sqrt(λ·K_att) = sqrt(%.4f × %.4f) = %.4f rad/s"
          % (lam_p, Ka_px4, wn_p))
    print("        ζ  = λ/(2·ω_n) = %.4f / %.4f = %.4f"
          % (lam_p, 2.0 * wn_p, ze_p))
    print()
    print("    ★ 还要看两个比值：")
    print("        λ/ω_n（带宽分离比） = %.3f" % (lam_p / wn_p))
    print("        K_i/K_p（积分/比例） = %.3f / %.3f = %.4f"
          % (px4_ri, px4_rp, px4_ri / px4_rp))
    ti_p = px4_rp / px4_ri
    print("        ⇒ 积分时间 T_i = K_p/K_i = %.4f s" % ti_p)
    kd_p = px4_rd * tau_max
    print("        K_d = MC_ROLLRATE_D × τ_scale = %.5f N·m·s²"
          % kd_p)
    print("        K_d / J = %.4f  （第 16 周那道墙在 1.0）"
          % (kd_p / J))

    print("""    ★★ 反推结论（这几个数就是 PX4 默认值的「翻译」）：

      · 闭环带宽 ω_n ≈ **%.2f rad/s = %.2f Hz**，
        阻尼比 ζ ≈ **%.2f**（略欠阻尼，二阶超调约 %.1f%%）。
      · 带宽分离比 λ/ω_n = **%.2f**。
        ★ 注意由 (17.10) 有 ζ = 0.5·(λ/ω_n)，所以实验 2 ①
          里推荐的「λ/ω_n = 2」其实等价于 **ζ = 1.0（完全过阻尼）**。
          PX4 选了 %.2f，等于**愿意拿 %.1f%% 的超调去换更短的调节时间** ——
          这正好落在实验 2 ①b 说的「ζ ∈ 0.7 ~ 1.0 甜点区」里。
          ⇒ ★★ 整定不是找一个「对」的值，是选一个【工作点】。
      · 积分时间 T_i ≈ **%.2f s**，K_d/J ≈ **%.2f**（离那道墙还有 %.0f 倍余量）。

      ⇒ ★★ 记住这三个数：**「ω_n ≈ %.0f rad/s、ζ ≈ %.1f、T_i ≈ %.1f s」**
        —— 这就是一架 1 kg 级四旋翼「出厂手感」的样子。
""" % (wn_p, wn_p / (2 * math.pi), ze_p, overshoot_theory(ze_p),
       lam_p / wn_p, lam_p / wn_p, overshoot_theory(ze_p),
       ti_p, kd_p / J, 1.0 / (kd_p / J),
       wn_p, ze_p, ti_p))

    # ---------------------------------------------------------------- ③
    sub("③ 把 ArduPilot 的默认参数反推成 (ω_n, ζ)")

    ap_p = AP_DEFAULTS["ATC_ANG_RLL_P"][0]
    ap_rp = AP_DEFAULTS["ATC_RAT_RLL_P"][0]
    ap_ri = AP_DEFAULTS["ATC_RAT_RLL_I"][0]
    ap_rd = AP_DEFAULTS["ATC_RAT_RLL_D"][0]
    print("""    ▸ 同样先抄一遍出处：

        参数              值       出处
        ────────────────────────────────────────────────────────────────""")
    for k in ("ATC_ANG_RLL_P", "ATC_RAT_RLL_P", "ATC_RAT_RLL_I",
              "ATC_RAT_RLL_D"):
        v, src = AP_DEFAULTS[k]
        print("        %-16s %-8s %s" % (k, "%.4f" % v, src))
    print("""
    ★ 结构性说明（这几点直接看源码就能确认）：

      · ANG_RLL_P 属于 `AC_P` 类，在 AC_AttitudeControl.cpp:82-104
        注册成参数组 `ANG_RLL_`（AC_AttitudeControl.cpp:88）。
        ★ 它的作用见 AC_AttitudeControl.cpp:83-84 的注释：
          「把期望滚转角与实际滚转角的误差转换成期望滚转角速率」——
          和 PX4 的 MC_ROLL_P 是**同一个东西**。

      · RAT_RLL_P/I/D 属于 `AC_PID` 类，注册成参数组 `RAT_RLL_`
        （AC_AttitudeControl_Multi.cpp:103）。

      · 速率环的输出走哪里？AC_AttitudeControl_Multi.cpp:442-467 的
        rate_controller_run() 里，直接
        `_motors.set_roll(pid.update_all(...))` ——
        ★ 和 PX4 一样是**归一化输出**，所以 P 到 N·m 的换算
          用同一个 τ_scale。

    ▸ 换算与反推：
""")
    Kr_ap = ap_rp * tau_max
    Ka_ap = ap_p
    lam_a, wn_a, ze_a = identify_design(Kr_ap, Ka_ap, J)
    print("        K_rate = ATC_RAT_RLL_P × τ_scale = %.4f × %.6f = %.5f"
          % (ap_rp, tau_max, Kr_ap))
    print("        K_att  = ATC_ANG_RLL_P = %.4f" % Ka_ap)
    print("        λ  = %.5f / %.4f = %.4f rad/s" % (Kr_ap, J, lam_a))
    print("        ω_n = sqrt(%.4f × %.4f) = %.4f rad/s = %.2f Hz"
          % (lam_a, Ka_ap, wn_a, wn_a / (2 * math.pi)))
    print("        ζ  = %.4f / %.4f = %.4f" % (lam_a, 2 * wn_a, ze_a))
    ti_a = ap_rp / ap_ri
    kd_a = ap_rd * tau_max
    print("        λ/ω_n = %.3f，T_i = K_p/K_i = %.4f s" % (lam_a / wn_a, ti_a))
    print("        K_d = ATC_RAT_RLL_D × τ_scale = %.5f N·m·s²，"
          "K_d/J = %.4f" % (kd_a, kd_a / J))

    # ---------------------------------------------------------------- ④
    sub("④ ★★ 两家飞控放在同一张表里，再和实测对照")

    rows = [
        ["PX4 mc_att_control",
         "%.2f / %.3f" % (px4_p, px4_rp),
         "%.4f" % wn_p, "%.2f" % (wn_p / (2 * math.pi)),
         "%.3f" % ze_p, "%.3f" % (lam_p / wn_p),
         "%.3f" % ti_p, "%.4f" % (kd_p / J)],
        ["ArduPilot AC_AttitudeControl",
         "%.2f / %.4f" % (ap_p, ap_rp),
         "%.4f" % wn_a, "%.2f" % (wn_a / (2 * math.pi)),
         "%.3f" % ze_a, "%.3f" % (lam_a / wn_a),
         "%.3f" % ti_a, "%.4f" % (kd_a / J)],
    ]
    tab(rows, ["飞控（横滚轴）", "外环 P / 内环 P", "ω_n (rad/s)", "带宽 (Hz)",
               "ζ", "λ/ω_n", "T_i (s)", "K_d/J"])

    # 实测：把反推出来的增益灌进本脚本的仿真
    q0 = q_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(30.0))
    qd = np.array([1.0, 0.0, 0.0, 0.0])
    print("    ▸ 实测：把上面反推出来的增益灌进本脚本的仿真（无扰动、无滞后）")
    print()
    rows = []
    for lab, Kr_v, Ka_v, ze_v in (("PX4 默认", Kr_px4, Ka_px4, ze_p),
                                  ("AP 默认", Kr_ap, Ka_ap, ze_a),
                                  ("本课设计点 ζ=0.7, 8 rad/s",
                                   *design_gains(0.7, 8.0, J), 0.7)):
        c = AttController(K_att=Ka_v, K_rate=Kr_v, att_mode="px4",
                          q_des=qd, body=body)
        lg = rigid_sim(c, T=5.0, dt=1.0 / 1000, q0=q0, body=body)
        m = _step_metrics(lg, qd)
        rows.append([lab, "%.4f" % Kr_v, "%.4f" % Ka_v,
                     "%.1f%%" % m["overshoot"],
                     "%.1f%%" % overshoot_theory(ze_v),
                     "%.0f" % (m["settle"] * 1e3),
                     "%.0f" % (m["rise"] * 1e3),
                     "%.4f" % m["ss"]])
    tab(rows, ["增益来源", "K_rate", "K_att", "超调(实测)",
               "超调(二阶解析)", "2% 调节 (ms)", "上升 (ms)", "稳态误差 (°)"])
    print("""    ★★ 读表：

      · ★★ **两家飞控的默认增益，实测响应和二阶解析式都对得上**
        （误差在 1 个百分点以内）。
        ⇒ 这证明第 17 周那套「串级 = 二阶系统」的推导
          **不是纸上谈兵，它能预测真实参数的响应。**

      · 两家的设计点惊人地接近：ω_n ≈ %.1f ~ %.1f rad/s、
        ζ ≈ %.2f ~ %.2f、λ/ω_n ≈ %.1f ~ %.1f、T_i ≈ %.1f ~ %.1f s。
        ★★ 这不是抄的 —— 它们的代码库完全独立。
          这是**同一个物理问题的最优解长得很像**的证据。

      · 两家都把 K_d/J 放在 %.3f ~ %.3f，
        离第 16 周那道 1.0 的墙有 %.0f 倍余量。
        ★★ 再一次印证实验 2 ③：D 项在串级结构里只是微调。

    ⇒ ★★★ 完整的整定流程（把本课所有结论串起来）：

        ① 量（或估）惯量 J —— 精度只要「量级对」（本实验 ①）
        ② 选定 ω_n：由【力矩裕度】决定上限（实验 2 ④），
           1 kg 级四旋翼的经验区间是 8 ~ 12 rad/s
        ③ 选定 ζ：工程甜点 0.7 ~ 1.0（实验 2 ①b）
        ④ 用 (17.11)(17.12) 算出 K_rate、K_att
        ⑤ 内环加 I 消稳态误差，T_i 取 0.5 ~ 1 s（实验 2 ②）
        ⑥ D 项从 0 开始小步加，服从 K_d < J（实验 2 ③）
        ⑦ 上机：先看阶跃响应，再看飞行日志有没有长时间饱和
        ⑧ 如果整体偏软/偏硬，用 MC_ROLLRATE_K 这类总增益按
           sqrt 关系微调（本实验 ①）
""" % (wn_a, wn_p, ze_p, ze_a, lam_p / wn_p, lam_a / wn_a,
       ti_p, ti_a, kd_p / J, kd_a / J, 1.0 / (kd_p / J)))

    # ---- 图 --------------------------------------------------------
    if HAVE_PLT:
        try:
            fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
            for lab, Kr_v, Ka_v, st in (
                    ("PX4 default", Kr_px4, Ka_px4, "-"),
                    ("ArduPilot default", Kr_ap, Ka_ap, "--"),
                    ("this lesson (0.7, 8)", *design_gains(0.7, 8.0, J), ":")):
                c = AttController(K_att=Ka_v, K_rate=Kr_v, att_mode="px4",
                                  q_des=qd, body=body)
                lg = rigid_sim(c, T=3.0, dt=1.0 / 1000, q0=q0, body=body)
                axes[0].plot(lg["t"], np.degrees(_err_series(lg, qd)),
                             st, label=lab)
                axes[1].plot(lg["t"], lg["wx"], st, label=lab)
            axes[0].set_xlabel("time (s)")
            axes[0].set_ylabel("attitude error (deg)")
            axes[0].set_title("step response: vendor defaults vs design")
            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("wx (rad/s)")
            axes[1].set_title("inner loop rate")
            for ax in axes:
                ax.grid(True, alpha=0.3)
                ax.legend(fontsize=8)
            fig.tight_layout()
            fn = os.path.join(HERE, "pid_lesson17_exp4.png")
            fig.savefig(fn, dpi=110, bbox_inches="tight")
            print("    [图已保存: %s]" % fn)
        except Exception as e:                              # noqa: BLE001
            print("    [画图失败: %s]" % e)
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 姿态误差：旋转矩阵 / 四元数 / 轴角，各是什么", exp1),
    "2": ("★ 单轴姿态控制：从 30° 倾斜回到水平", exp2),
    "3": ("★★ 三轴耦合：ω × (Jω) 与陀螺进动", exp3),
    "4": ("★★ 整定：从 ζ、ω_n 反推增益，与两家飞控默认值对照", exp4),
}


def main(argv):
    print()
    print("#" * 78)
    print("#  第 17 周 · 02 姿态控制律设计")
    print("#  配套实验")
    print("#" * 78)
    print()
    print("  用法：  python lesson17_attitude_control.py          # 跑全部实验")
    print("          python lesson17_attitude_control.py 1 2 3 4  # 跑指定实验")
    print()
    print("  实验清单：")
    for k in sorted(EXPERIMENTS):
        print("    %s. %s" % (k, EXPERIMENTS[k][0]))
    print()

    want = [a for a in argv if a in EXPERIMENTS]
    if not want:
        want = sorted(EXPERIMENTS)
    for k in want:
        EXPERIMENTS[k][1]()
    print("#" * 78)
    print("#  全部实验结束")
    print("#" * 78)
    print()


if __name__ == "__main__":
    main(sys.argv[1:])



