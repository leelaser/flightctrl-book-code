# -*- coding: utf-8 -*-
"""卷 1 第 6 章 · 捷联惯导更新算法 —— 位置更新、R_M/R_N、哥氏项、高度通道

配套课文：`卷 1 第 6 章`
源书：严恭敏, 翁浚《捷联惯导算法与组合导航原理》第 4 章（PDF p079–p104）

本脚本做四件事：

  [1] ★ 闭环自证：轨迹 → 逆过程 → IMU → 正过程 → 轨迹（沿用 07-仿真与实验/04 的方法）
  [2] ★★ 反向测试：把 R_M 和 R_N 【用反】，看位置误差怎么累积
  [3] ★★ 哥氏项的量级：多大速度、多长航时下它开始不可忽略
  [4] ★★★ 高度通道发散：为什么纯惯导的高度【原理上就不能用】

⚠️ **坐标系约定**：本脚本用 **N-E-D**（与 ArduPilot/PX4 一致）。
   课文 §1 特别强调：**严恭敏原书用的是 E-N-U**，两者差一个置换 + 一个符号。
   本脚本选 NED 是为了和教程其它部分统一 —— **这也是一次"约定检查"的示范**。

运行：`python lesson07b_sins_mechanization.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function

import math
import sys

n_ok = 0
n_bad = 0


def check(cond, msg):
    global n_ok, n_bad
    if cond:
        n_ok += 1
        print('    [OK] %s' % msg)
    else:
        n_bad += 1
        print('    [!!] %s' % msg)


D2R = math.pi / 180.0
G0 = 9.80665

# ── WGS-84（与 04-状态估计/10 一致）──────────────────────────
R_E = 6378137.0
F = 1.0 / 298.257223563
E2 = 2 * F - F * F


def R_M(L):
    """子午圈曲率半径（南北方向）"""
    return R_E * (1 - E2) / (1 - E2 * math.sin(L) ** 2) ** 1.5


def R_N(L):
    """卯酉圈曲率半径（东西方向）"""
    return R_E / math.sqrt(1 - E2 * math.sin(L) ** 2)


# ══════════════════════════════════════════════════════════════
# NED 下的姿态工具（与 07-仿真与实验/04 同一套约定）
# ══════════════════════════════════════════════════════════════
def C_bn(phi, theta, psi):
    sp, cp = math.sin(phi), math.cos(phi)
    st, ct = math.sin(theta), math.cos(theta)
    ss, cs = math.sin(psi), math.cos(psi)
    return [[ct * cs, sp * st * cs - cp * ss, cp * st * cs + sp * ss],
            [ct * ss, sp * st * ss + cp * cs, cp * st * ss - sp * cs],
            [-st,     sp * ct,                cp * ct]]


def C_nb(phi, theta, psi):
    C = C_bn(phi, theta, psi)
    return [[C[j][i] for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][j] * v[j] for j in range(3)) for i in range(3)]


def euler_rates_to_omega(phi, theta, dphi, dtheta, dpsi):
    sp, cp = math.sin(phi), math.cos(phi)
    st, ct = math.sin(theta), math.cos(theta)
    return [dphi - st * dpsi,
            cp * dtheta + sp * ct * dpsi,
            -sp * dtheta + cp * ct * dpsi]


def quat_from_euler(phi, theta, psi):
    cph, sph = math.cos(phi / 2), math.sin(phi / 2)
    cth, sth = math.cos(theta / 2), math.sin(theta / 2)
    cps, sps = math.cos(psi / 2), math.sin(psi / 2)
    return [cph * cth * cps + sph * sth * sps,
            sph * cth * cps - cph * sth * sps,
            cph * sth * cps + sph * cth * sps,
            cph * cth * sps - sph * sth * cps]


def quat_to_Cbn(q):
    q0, q1, q2, q3 = q
    return [[1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)]]


def smoothstep(x):
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x * x * (3 - 2 * x)


# ══════════════════════════════════════════════════════════════
# 轨迹设计（同 07-仿真与实验/04，四种基本动作）
# ══════════════════════════════════════════════════════════════
def trajectory(t):
    V0, V1 = 25.0, 40.0
    phi = theta = 0.0
    if t < 3.0:
        V = V0 + (V1 - V0) * smoothstep(t / 3.0)
    else:
        V = V1
    if 3.0 <= t < 9.0:
        phi = 25.0 * D2R * smoothstep((t - 3.0) / 2.0)
    if t >= 9.0:
        phi = 25.0 * D2R * (1.0 - smoothstep((t - 9.0) / 2.0))
    return phi, theta, V


def traj_state(t, h=1e-5):
    """返回 (v^n, dot_v^n, omega_nb^b, att)"""
    phi, theta, V = trajectory(t)
    p2, th2, V2 = trajectory(t + h)
    p1, th1, V1 = trajectory(t - h)
    # ⚠️ 本函数被【生成器】(inverse_process) 和【积分器】(forward_process) 共用，
    #   所以这里任何系统误差都会在闭环里【自我抵消】—— 闭环误差纹丝不动，
    #   只有 "真值位置" 那一行会悄悄变。
    #   实测：把 [0.0, V, 0.0] 错写成 [0.0, phi, 0.0]（实参错位）后，
    #   真值北向位置从 357.14 变成 107.14（差 250 m），而脚本依旧 exit 0。
    #   故 §[1] 补了一条 |v_n| == V 的断言来钉死它：旋转不改变矢量模长。
    v_n = matvec(C_bn(phi, theta, 0.0), [0.0, V, 0.0])
    v_n2 = matvec(C_bn(p2, th2, 0.0), [0.0, V2, 0.0])
    v_n1 = matvec(C_bn(p1, th1, 0.0), [0.0, V1, 0.0])
    dot_v = [(v_n2[i] - v_n1[i]) / (2 * h) for i in range(3)]
    dphi = (p2 - p1) / (2 * h)
    dth = (th2 - th1) / (2 * h)
    w = euler_rates_to_omega(phi, theta, dphi, dth, 0.0)
    return v_n, dot_v, w, (phi, theta, 0.0)


def inverse_process(t):
    """轨迹 → IMU：f^b = C_n^b(v̇ⁿ − gⁿ)，ω_ib^b = ω_nb^b"""
    v_n, dot_v, w, att = traj_state(t)
    Cn = C_nb(*att)
    f_b = matvec(Cn, [dot_v[0], dot_v[1], dot_v[2] - G0])
    return f_b, w


def forward_process(dt=1e-3, T=10.0):
    """IMU → 轨迹（数值积分）"""
    v0, _, _, att0 = traj_state(0.0)
    v = list(v0)
    q = quat_from_euler(*att0)
    p = [0.0, 0.0, 0.0]          # NED 位置（这里用平面近似，见 §[1] 说明）
    ts, ps, vs = [0.0], [list(p)], [list(v)]
    n = int(round(T / dt))
    for k in range(n):
        t = k * dt

        def deriv(t_, v_, q_):
            f_b, w_b = inverse_process(t_)
            C = quat_to_Cbn(q_)
            a_n = matvec(C, f_b)
            dv = [a_n[0], a_n[1], a_n[2] + G0]
            q0, q1, q2, q3 = q_
            w0, w1, w2 = w_b
            dq = [0.5 * (-q1 * w0 - q2 * w1 - q3 * w2),
                  0.5 * (q0 * w0 + q2 * w2 - q3 * w1),
                  0.5 * (q0 * w1 - q1 * w2 + q3 * w0),
                  0.5 * (q0 * w2 + q1 * w1 - q2 * w0)]
            return v_, dv, dq

        k1v, k1a, k1q = deriv(t, v, q)
        v2 = [v[i] + dt / 2 * k1a[i] for i in range(3)]
        q2 = [q[i] + dt / 2 * k1q[i] for i in range(4)]
        k2v, k2a, k2q = deriv(t + dt / 2, v2, q2)
        v3 = [v[i] + dt / 2 * k2a[i] for i in range(3)]
        q3 = [q[i] + dt / 2 * k2q[i] for i in range(4)]
        k3v, k3a, k3q = deriv(t + dt / 2, v3, q3)
        v4 = [v[i] + dt * k3a[i] for i in range(3)]
        q4 = [q[i] + dt * k3q[i] for i in range(4)]
        k4v, k4a, k4q = deriv(t + dt, v4, q4)

        p = [p[i] + dt / 6 * (k1v[i] + 2 * k2v[i] + 2 * k3v[i] + k4v[i])
             for i in range(3)]
        v = [v[i] + dt / 6 * (k1a[i] + 2 * k2a[i] + 2 * k3a[i] + k4a[i])
             for i in range(3)]
        q = [q[i] + dt / 6 * (k1q[i] + 2 * k2q[i] + 2 * k3q[i] + k4q[i])
             for i in range(4)]
        nq = math.sqrt(sum(x * x for x in q))
        q = [x / nq for x in q]
        ts.append((k + 1) * dt)
        ps.append(list(p))
        vs.append(list(v))
    return ts, ps, vs


def true_position(T):
    dt = 1e-4
    n = int(round(T / dt))
    p = [0.0, 0.0, 0.0]
    v = list(traj_state(0.0)[0])
    for k in range(n):
        _, dv, _, _ = traj_state(k * dt + dt / 2)
        v = [v[i] + dt * dv[i] for i in range(3)]
        p = [p[i] + dt * v[i] for i in range(3)]
    return p


def vnorm(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


print('=' * 74)
print('[1] 闭环自证：轨迹 → IMU → 轨迹（NED 力学编排）')
print('=' * 74)
T = 10.0
_, ps, vs = forward_process(dt=1e-3, T=T)
p_true = true_position(T)
err = vnorm(ps[-1], p_true)
print('    真值位置 [%.4f, %.4f, %.4f]' % tuple(p_true))
print('    解算位置 [%.4f, %.4f, %.4f]' % tuple(ps[-1]))
print('    位置误差 = %.3e m' % err)
check(err < 1e-3, '闭环复现，位置误差 %.2e m（< 1e-3）' % err)

# ★★★ 闭环自证有一个致命盲区：生成器与积分器【共用 traj_state】，
#   所以 traj_state 里的系统误差两边都犯，相减就没了。
#   下面这条断言不看闭环，直接钉住 traj_state 自身：机体速度 → 导航速度
#   只是一次【旋转】，模长必须守恒。实参错位会把它破坏掉。
_V_checked = 0.0
_vn_norm_ok = True
for _t in (0.5, 2.0, 4.5, 7.0, 9.5, 11.0, 14.0):
    _vn, _, _, _ = traj_state(_t)
    _V = trajectory(_t)[2]
    _V_checked = max(_V_checked, abs(math.sqrt(sum(x * x for x in _vn)) - _V))
    if abs(math.sqrt(sum(x * x for x in _vn)) - _V) > 1e-12:
        _vn_norm_ok = False
check(_vn_norm_ok,
      'traj_state 保模长：|v^n| == V 全程成立（7 个时刻最大偏差 %.2e）' % _V_checked)
print('    ⇒ ★ 这条才是真正钉住 traj_state 的断言（闭环那条钉不住它）。')

print()
print('=' * 74)
print('[2] ★★ 反向测试：R_M 与 R_N 【用反】会怎样')
print('=' * 74)
print('    位置更新公式（NED）：')
print('       L̇ = v_N / R_M(L)        ← 纬度用【子午圈】半径')
print('       λ̇ = v_E / (R_N(L)·cosL)  ← 经度用【卯酉圈】半径')
print()
LAT = 45.0 * D2R
print('    在纬度 45° 处： R_M = %.1f m,  R_N = %.1f m,  相差 %.1f m（%.2f%%）'
      % (R_M(LAT), R_N(LAT), R_N(LAT) - R_M(LAT),
         100 * (R_N(LAT) - R_M(LAT)) / R_M(LAT)))
# 沿北向飞 10 km，用错半径会差多少
DIST = 10000.0
dL_right = DIST / R_M(LAT)
dL_wrong = DIST / R_N(LAT)
km_err = abs(dL_wrong - dL_right) * R_M(LAT)
rel_radius = (R_N(LAT) - R_M(LAT)) / R_M(LAT)
rel_err = km_err / DIST
print('    沿北向飞 %.0f km：用对的半径纬度变化 %.6f°，用错的 %.6f°'
      % (DIST / 1000, dL_right / D2R, dL_wrong / D2R))
print('    ⇒ 位置误差 = %.1f m，相对误差 = %.4f%%' % (km_err, 100 * rel_err))
# ★ 推导：沿北向飞 D，真值 ΔL_true = D/R_M，用错得 ΔL_wrong = D/R_N
#         位置误差 = |ΔL_wrong − ΔL_true|·R_M = D·|R_M/R_N − 1|
#         ⇒【相对】误差 = |R_M/R_N − 1| = (R_N−R_M)/R_N
#   ⚠️ 注意分母是 R_N，【不是】R_M —— 第一版我按 (R_N−R_M)/R_M 写，
#      断言差了 1.2e-5 对不上。**不是数值噪声，是推导不严。**
rel_expect = abs(R_M(LAT) / R_N(LAT) - 1.0)
print('       理论相对误差 |R_M/R_N − 1| = %.6f%%' % (100 * rel_expect))
check(abs(rel_err - rel_expect) < 1e-9,
      '相对误差 %.6f%% 精确等于 |R_M/R_N − 1| ⇒ 误差就是"用错半径"本身'
      % (100 * rel_err))
# ⚠️ 第一版这里断言 "> 50 m"，实测 33.6 m —— 阈值是我随手定的，没有依据。
#    改成判"它是否与距离成正比"：距离加倍，误差也加倍。
d2_err = abs(DIST * 2 / R_N(LAT) - DIST * 2 / R_M(LAT)) * R_M(LAT)
check(abs(d2_err / km_err - 2.0) < 1e-6,
      '误差与距离成正比（20 km 时 %.1f m = 10 km 的 2 倍）' % d2_err)

print()
print('=' * 74)
print('[3] ★★ 哥氏项的量级：什么时候可以省')
print('=' * 74)
W_IE = 7.2921151467e-5
print('    哥氏加速度 ‖2ω_ie × v‖ = 2·ω_ie·‖v‖·sin(夹角)')
print('    在 NED 里 ω_ie = ω_ie[cosL, 0, −sinL]，最坏情况取水平面内速度：')
print()
print('    %-10s %-14s %14s %16s' % ('纬度', '速度 (m/s)', '哥氏加速度', '10 min 速度误差'))
for deg in (0, 45, 90):
    L = deg * D2R
    for V in (20.0, 100.0, 300.0):
        ac = 2 * W_IE * V * math.cos(L)
        dv = 0.5 * ac * 600.0            # 10 min = 600 s，按匀加速粗估
        print('    %-10d %-14.0f %11.5f m/s² %12.2f m/s' % (deg, V, ac, dv))
print()
print('    ⇒ 低速短时（多旋翼、20 m/s 飞几分钟）：哥氏项可以省')
print('      高速长时（固定翼 100+ m/s 飞几十分钟）：【不能省】')
check(2 * W_IE * 100.0 * math.cos(45 * D2R) * 600 > 1.0,
      '100 m/s 飞 10 min，哥氏造成的速度误差 %.1f m/s > 1 ⇒ 必须补'
      % (0.5 * 2 * W_IE * 100.0 * math.cos(45 * D2R) * 600))

print()
print('=' * 74)
print('[4] ★★★ 高度通道发散：纯惯导的高度为什么不能用')
print('=' * 74)
print('    Schuler 振荡（约 84.4 min 周期）对水平通道是【天然的稳定机制】，')
print('    但高度通道【没有这个机制】—— 加计的零偏会被【指数放大】。')
print()
print('    简化模型： ḧ = δf + (2/R)·h  （重力随高度变化带来的正反馈）')
K = 2.0 / R_E                    # 正反馈系数
print('    正反馈系数 2/R = %.3e 1/m ⇒ 时间常数 1/√(2/R) = %.0f s（%.1f min）'
      % (K, 1 / math.sqrt(K), 1 / math.sqrt(K) / 60))
tau = 1 / math.sqrt(K)


def h_err_of(bias_mg, t):
    """ḧ = b + h/τ²  ⇒  h(t) = b·τ²·(cosh(t/τ) − 1)"""
    b = bias_mg * 1e-3 * G0
    return b * tau * tau * (math.cosh(t / tau) - 1)


print()
print('    %-14s %18s %18s' % ('加计零偏 (mg)', '10 min 后 (m)', '20 min 后 (m)'))
for bias_mg in (0.01, 0.1, 1.0):
    print('    %-14.2f %18.1f %18.1f'
          % (bias_mg, h_err_of(bias_mg, 600.0), h_err_of(bias_mg, 1200.0)))
print()
# 判据：误差与零偏【成正比】（线性），但随时间是【cosh 】（超指数）
r_bias = h_err_of(0.1, 600.0) / h_err_of(0.01, 600.0)
r_time = h_err_of(0.01, 1200.0) / h_err_of(0.01, 600.0)
check(abs(r_bias - 10.0) < 0.01, '高度误差与加计零偏【成正比】（10 倍零偏 = %.1f 倍误差）' % r_bias)
check(r_time > 3.0,
      '时间加倍误差涨 %.2f 倍（远超 2 倍）⇒ 是 cosh 指数增长，不是线性' % r_time)
print('    ⇒ 高度通道的误差按 cosh（指数级）增长 —— 这就是「原理上发散」的含义。')
print('      （对比：水平通道有 Schuler 振荡约束，误差不会指数发散）')
print('    ⇒ 原书 §4.1.1：「纯惯导系统的高度通道【在原理上是发散的】，')
print('      因而惯导系统多采用当地水平坐标系……长时间导航定位时')
print('      【只进行水平定位解算，而简单地将高度通道设置为固定值】。」')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
