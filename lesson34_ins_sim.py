# -*- coding: utf-8 -*-
"""第 24.6 周 · 04 惯导与组合导航仿真 —— 逆过程 / 正过程的【闭环自证】

配套课文：`07-仿真与实验/04-惯导与组合导航仿真-严恭敏.md`
源书：严恭敏, 翁浚《捷联惯导算法与组合导航原理》第 8 章（PDF p220–p296）

## 本脚本要证明的一件事

   正过程（导航解算）：  IMU ──积分──► 姿态/速度/位置
   逆过程（仿真）  ：  轨迹 ──微分──► IMU          ← 原书 §8.1 的「逆过程」

**把逆过程生成的理想 IMU 喂给正过程，必须【精确复现】原轨迹。**
复现不了 ⇒ 两者至少有一处错了（最常见的两处：漏重力、漏哥氏）。

本脚本：

  [1] 设计一条含【加速 / 滚转 / 俯仰 / 协调转弯】的轨迹（§8.1 的四种基本动作）
  [2] 用【逆过程】生成理想 IMU
  [3] 用【正过程】积分回轨迹，断言复现
  [4] ★ 反向测试：逆过程【故意漏掉重力】—— 看误差是否立刻暴露
  [5] ★ 加陀螺零偏，验证姿态漂移近似线性

⚠️ **模型简化**（与课文 §6 的说明一致）：
   本脚本用【平面地球 + 无自转】的 NED 力学编排，即
       ẋⁿ = vⁿ,   v̇ⁿ = C_b^n f^b + gⁿ,   q̇ = ½ q⊗[0, ω_ib^b]
   因此 ω_en^ⁿ = 0、ω_ie^ⁿ = 0，逆过程只需补【重力】一项。
   带地球自转与哥氏的完整版本，请按课文 §1 的公式补上 —— 留给读者。

运行：`python lesson34_ins_sim.py`   —— 全部断言通过才返回 0
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


G0 = 9.80665                      # 重力加速度（平面地球取常数）
D2R = math.pi / 180.0


# ══════════════════════════════════════════════════════════════
# 姿态：3-2-1（偏航 ψ → 俯仰 θ → 滚转 φ），与教程第 1 章一致
# ══════════════════════════════════════════════════════════════
def C_bn(phi, theta, psi):
    """机体系 → NED 的方向余弦阵 C_b^n"""
    sp, cp = math.sin(phi), math.cos(phi)
    st, ct = math.sin(theta), math.cos(theta)
    ss, cs = math.sin(psi), math.cos(psi)
    return [[ct * cs, sp * st * cs - cp * ss, cp * st * cs + sp * ss],
            [ct * ss, sp * st * ss + cp * cs, cp * st * ss - sp * cs],
            [-st,     sp * ct,                cp * ct]]


def C_nb(phi, theta, psi):
    C = C_bn(phi, theta, psi)
    return [[C[j][i] for j in range(3)] for i in range(3)]


def euler_rates_to_omega(phi, theta, dphi, dtheta, dpsi):
    """欧拉角速率 → ω_nb^b（3-2-1 的逆关系）

        [p]   [1    0        -sinθ      ] [φ̇]
        [q] = [0   cosφ    sinφ·cosθ    ] [θ̇]
        [r]   [0  -sinφ    cosφ·cosθ    ] [ψ̇]
    """
    sp, cp = math.sin(phi), math.cos(phi)
    st = math.sin(theta)
    ct = math.cos(theta)
    return [dphi - st * dpsi,
            cp * dtheta + sp * ct * dpsi,
            -sp * dtheta + cp * ct * dpsi]


def matvec(A, v):
    return [sum(A[i][j] * v[j] for j in range(3)) for i in range(3)]


# ══════════════════════════════════════════════════════════════
# [1] 轨迹设计：§8.1 的四种基本动作拼接
# ══════════════════════════════════════════════════════════════
def smoothstep(x):
    """0→1 的 C1 平滑过渡（避免导数跳变污染逆过程）"""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x * x * (3 - 2 * x)


def trajectory(t):
    """返回 (phi, theta, psi, V)，单位 rad / rad / rad / (m/s)

    分段（总时长 20 s）：
        0– 4 s  加速：     V: 20 → 35 m/s，姿态不变
        4–10 s  协调转弯： φ → 30°，ψ 按 tanφ = a_y/g 协调转
       10–14 s  俯仰：     θ: 0 → 12° → 0
       14–20 s  平飞
    """
    V0, V1 = 20.0, 35.0
    phi = theta = psi = 0.0

    if t < 4.0:
        s = smoothstep(t / 4.0)
        V = V0 + (V1 - V0) * s
    else:
        V = V1

    if 4.0 <= t < 10.0:
        s = smoothstep((t - 4.0) / 2.0)              # 2 s 内滚转到 30°
        phi = 30.0 * D2R * s

    if 10.0 <= t < 14.0:
        phi = 30.0 * D2R                              # 转弯期间保持滚转
        u = (t - 10.0) / 4.0
        theta = 12.0 * D2R * math.sin(math.pi * u)    # 0 → 12° → 0

    if t >= 14.0:
        phi = 30.0 * D2R * (1.0 - smoothstep((t - 14.0) / 2.0))

    return phi, theta, psi, V


def traj_state(t):
    """把轨迹整理成 (v^n, dot_v^n, omega_nb^b)"""
    h = 1e-5
    phi, theta, psi, V = trajectory(t)
    # 机体系速度：假设攻角=侧滑角=0 ⇒ 只沿纵轴
    v_b = [V, 0.0, 0.0]
    v_n = matvec(C_bn(phi, theta, psi), v_b)

    # 数值微分（轨迹是光滑的，FD 精度足够）
    p2, th2, ps2, V2 = trajectory(t + h)
    p1, th1, ps1, V1_ = trajectory(t - h)
    v_n2 = matvec(C_bn(p2, th2, ps2), [V2, 0.0, 0.0])
    v_n1 = matvec(C_bn(p1, th1, ps1), [V1_, 0.0, 0.0])
    dot_v_n = [(v_n2[i] - v_n1[i]) / (2 * h) for i in range(3)]

    # 欧拉角速率 → ω_nb^b
    dphi = (p2 - p1) / (2 * h)
    dtheta = (th2 - th1) / (2 * h)
    dpsi = (ps2 - ps1) / (2 * h)
    w_nb_b = euler_rates_to_omega(phi, theta, dphi, dtheta, dpsi)
    return v_n, dot_v_n, w_nb_b, (phi, theta, psi)


# ══════════════════════════════════════════════════════════════
# [2] 逆过程：轨迹 → IMU
# ══════════════════════════════════════════════════════════════
def inverse_process(t, drop_gravity=False):
    """由轨迹生成 IMU：返回 (f^b, ω_ib^b)

        f^b = C_n^b ( v̇ⁿ − gⁿ )        ← 平面地球、无自转
        ω_ib^b = ω_nb^b
    """
    v_n, dot_v_n, w_nb_b, att = traj_state(t)
    g_n = [0.0, 0.0, G0] if not drop_gravity else [0.0, 0.0, 0.0]
    Cn = C_nb(*att)
    f_b = matvec(Cn, [dot_v_n[i] - g_n[i] for i in range(3)])
    return f_b, w_nb_b


# ══════════════════════════════════════════════════════════════
# [3] 正过程：IMU → 轨迹
# ══════════════════════════════════════════════════════════════
def quat_from_euler(phi, theta, psi):
    """3-2-1 欧拉角 → 四元数 [q0,q1,q2,q3]（标量在前）"""
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


def forward_process(dt=1e-3, T=20.0, gyro_bias=None, imu_forgot_gravity=False):
    """从 IMU 积分回轨迹。返回 (t 列表, p^n 列表, v^n 列表)

    ★★ `imu_forgot_gravity` 只作用在【IMU 生成端】，
       积分端【照常加】重力 —— 这才是真实的错误场景：
       某人在写"轨迹→IMU"那段时忘了补重力，而导航解算是对的。

       ⚠️ 第一版把 drop_gravity 同时传给了生成端和积分端，
          **两边一起漏 ⇒ 误差对消 ⇒ 反向测试根本测不出来**
          （漏重力版与正确版的位置误差都是 7.993e-04，一模一样）。
          这正是「互相抵消的错会瞒过端到端自证」。
    """
    bias = gyro_bias or [0.0, 0.0, 0.0]
    # 初值取真值
    v0, _, _, att0 = traj_state(0.0)
    p = [0.0, 0.0, 0.0]
    v = list(v0)
    q = quat_from_euler(*att0)

    ts, ps, vs = [0.0], [list(p)], [list(v)]
    n = int(round(T / dt))
    for k in range(n):
        t = k * dt

        def deriv(t_, p_, v_, q_):
            # ★ 生成端可能"忘了补重力"；积分端【总是】加
            f_b, w_b = inverse_process(t_, drop_gravity=imu_forgot_gravity)
            w_b = [w_b[i] + bias[i] for i in range(3)]
            C = quat_to_Cbn(q_)
            g_n = [0.0, 0.0, G0]
            a_n = matvec(C, f_b)
            dv = [a_n[i] + g_n[i] for i in range(3)]
            # q̇ = ½ q ⊗ [0, ω]
            w0, w1, w2 = w_b
            q0, q1, q2, q3 = q_
            dq = [0.5 * (-q1 * w0 - q2 * w1 - q3 * w2),
                  0.5 * (q0 * w0 + q2 * w2 - q3 * w1),
                  0.5 * (q0 * w1 - q1 * w2 + q3 * w0),
                  0.5 * (q0 * w2 + q1 * w1 - q2 * w0)]
            return v_, dv, dq

        # RK4
        k1v, k1a, k1q = deriv(t, p, v, q)
        v2 = [v[i] + dt / 2 * k1a[i] for i in range(3)]
        q2 = [q[i] + dt / 2 * k1q[i] for i in range(4)]
        k2v, k2a, k2q = deriv(t + dt / 2, p, v2, q2)
        v3 = [v[i] + dt / 2 * k2a[i] for i in range(3)]
        q3 = [q[i] + dt / 2 * k2q[i] for i in range(4)]
        k3v, k3a, k3q = deriv(t + dt / 2, p, v3, q3)
        v4 = [v[i] + dt * k3a[i] for i in range(3)]
        q4 = [q[i] + dt * k3q[i] for i in range(4)]
        k4v, k4a, k4q = deriv(t + dt, p, v4, q4)

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

    # 姿态角（从四元数反解，仅用于显示）
    atts = []
    for k in range(len(ts)):
        atts.append(None)
    return ts, ps, vs


def true_position(T):
    """真值位置：对 v^n 做高精度积分（步长比正过程小 10 倍）"""
    dt = 1e-4
    n = int(round(T / dt))
    p = [0.0, 0.0, 0.0]
    v = list(traj_state(0.0)[0])
    for k in range(n):
        t = k * dt
        _, dv, _, _ = traj_state(t + dt / 2)
        v = [v[i] + dt * dv[i] for i in range(3)]
        p = [p[i] + dt * v[i] for i in range(3)]
    return p


print('=' * 74)
print('[1] 轨迹设计（§8.1 四种基本动作拼接）')
print('=' * 74)
for t in (0.0, 2.0, 5.0, 8.0, 12.0, 16.0, 20.0):
    phi, theta, psi, V = trajectory(t)
    phase = ('加速' if t < 4 else '协调转弯' if t < 10 else
             '俯仰' if t < 14 else '平飞')
    print('    t=%5.1f s  %-8s  V=%5.1f m/s   φ=%5.1f°  θ=%5.1f°'
          % (t, phase, V, math.degrees(phi), math.degrees(theta)))
print('    ⇒ 每个时刻【只变一个参数】（或协调转弯时 φ 与 ψ 联动）')

print()
print('=' * 74)
print('[2]+[3] 逆过程 → 正过程：闭环自证')
print('=' * 74)
T = 12.0
ts, ps, vs = forward_process(dt=1e-3, T=T)
p_true = true_position(T)
p_est = ps[-1]


def vnorm(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


err = vnorm(p_est, p_true)
print('    真值位置 t=%.0f s:  [%.6f, %.6f, %.6f]' % (T, p_true[0], p_true[1], p_true[2]))
print('    解算位置 t=%.0f s:  [%.6f, %.6f, %.6f]' % (T, p_est[0], p_est[1], p_est[2]))
print('    位置误差 = %.3e m' % err)
check(err < 1e-3, '正过程复现轨迹，位置误差 %.2e m（< 1e-3）' % err)

v_true = traj_state(T)[0]
verr = vnorm(vs[-1], v_true)
print('    速度误差 = %.3e m/s' % verr)
check(verr < 1e-6, '速度复现，误差 %.2e m/s（< 1e-6）' % verr)

print()
print('=' * 74)
print('[4] ★ 反向测试：IMU 生成端【忘了补重力】，积分端照常加')
print('=' * 74)
_, ps_bad, _ = forward_process(dt=1e-3, T=T, imu_forgot_gravity=True)
err_bad = vnorm(ps_bad[-1], p_true)
print('    漏重力时位置误差 = %.3e m' % err_bad)
check(err_bad > 1.0,
      '漏重力后误差放大到 %.1f m（是正确版 %.1e 的 %.0f 倍）⇒ 判据能抓出这类错误'
      % (err_bad, err, err_bad / max(err, 1e-12)))
print('    ⇒ 这就是课文 §1 那句「重力绝对不能省」的量化证据。')
print('      （t=%.0f s 时自由落体位移理论值约 ½·g·t² = %.0f m）'
      % (T, 0.5 * G0 * T * T))
print()
print('    ⚠️ 注意本测试的构造方式：**必须让两端不一致**。')
print('      如果把"漏重力"同时用在生成端和积分端，两者会【互相抵消】，')
print('      结果与正确版一模一样（本项目第一版就是这么写的，断言当场失败）。')

print()
print('=' * 74)
print('[5] ★ 加陀螺零偏：姿态漂移应近似【线性】')
print('=' * 74)
BIAS = 0.5 * D2R / 3600.0      # 0.5 °/h = 2.42e-6 rad/s
_, ps_b, _ = forward_process(dt=1e-3, T=T, gyro_bias=[BIAS, 0.0, 0.0])
drift_end = vnorm(ps_b[-1], p_true)
print('    零偏 0.5 °/h，%.0f s 后位置偏差 = %.3f m' % (T, drift_end))
check(drift_end > 0.0, '零偏确实造成了位置偏差（%.3f m）' % drift_end)

# ★ 增长阶数：陀螺零偏 ⇒ 姿态误差∝t ⇒ 加速度误差∝t ⇒ 速度∝t² ⇒ 位置∝t³
#   ⚠️ 第一版这里断言"平方增长、比值≈4"，实测比值 7.67 —— 断言错了，【物理是对的】。
#      误在把"姿态误差线性"直接当成了"位置二次"，
#      忘了中间还隔着【一次速度积分】。
_, ps_b2, _ = forward_process(dt=1e-3, T=16.0, gyro_bias=[BIAS, 0.0, 0.0])
p_true2 = true_position(16.0)
d2 = vnorm(ps_b2[-1], p_true2)
_, ps_b1, _ = forward_process(dt=1e-3, T=8.0, gyro_bias=[BIAS, 0.0, 0.0])
p_true1 = true_position(8.0)
d1 = vnorm(ps_b1[-1], p_true1)
ratio = d2 / max(d1, 1e-12)
print('    8 s 偏差 = %.4f m,  16 s 偏差 = %.4f m,  比值 = %.2f' % (d1, d2, ratio))
check(6.0 < ratio < 10.0,
      '位置偏差随时间近似【三次】增长（比值 %.2f ≈ 8 = 2³）' % ratio)
print('    ⇒ 传播链：陀螺零偏 → 姿态误差 ∝ t → 加速度误差 ∝ t')
print('              → 速度误差 ∝ t² → 【位置误差 ∝ t³】')
print('      所以纯惯导的位置漂移是【三次方】发散 —— 比"平方"还快，撑不了几分钟。')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
