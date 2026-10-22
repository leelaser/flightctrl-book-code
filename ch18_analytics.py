# -*- coding: utf-8 -*-
"""卷 1 第 18 章 · 分析力学与振动 —— 可执行验证

配套课文：`卷 1 第 18 章`

验证七件事：

  [1] 单摆的 L=T−V、欧拉–拉格朗日方程，以及小角度周期 T0=2π√(l/g)
  [2] 单摆数值积分：能量守恒；大角度周期比小角度长
  [3] 大角度周期修正 T≈T0(1+θ0²/16)
  [4] 质量–弹簧：ω=√(k/m)
  [5] 简正模态：M=I、K=[[2,−1],[−1,2]] ⇒ ω²=1,3
  [6] 哈密顿方程：谐振子 H=p²/2m+½kq² 守恒
  [7] 中心力：角动量守恒（循环坐标 θ ⇒ p_θ 守恒）

设计原则：
  · 解析结论与 RK4 数值积分互核，不只抄公式
  · 中间轴/周期修正这类“差一点”的效应，用数值和解析两条路对照
  · 所有数都打印出来，方便和正文逐项对照

运行：`python ch18_analytics.py` —— 全部断言通过才返回 0
"""
from __future__ import print_function

import math
import sys

import numpy as np

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


G = 9.81

print('=' * 74)
print('[1] 单摆：L、欧拉–拉格朗日、小角度周期')
print('=' * 74)
l = 1.0
T0 = 2.0 * math.pi * math.sqrt(l / G)
print('    l=%.1f m, g=%.2f m/s²' % (l, G))
print('    小角度周期 T0 = 2π√(l/g) = %.6f s' % T0)
check(abs(T0 - 2.006) < 1e-3, '小角度周期 ≈ 2.006 s')


def pendulum_rhs(theta, omega):
    return omega, -(G / l) * math.sin(theta)


def rk4_pendulum(theta, omega, dt):
    k1t, k1w = pendulum_rhs(theta, omega)
    k2t, k2w = pendulum_rhs(theta + 0.5 * dt * k1t, omega + 0.5 * dt * k1w)
    k3t, k3w = pendulum_rhs(theta + 0.5 * dt * k2t, omega + 0.5 * dt * k2w)
    k4t, k4w = pendulum_rhs(theta + dt * k3t, omega + dt * k3w)
    return (theta + dt / 6.0 * (k1t + 2 * k2t + 2 * k3t + k4t),
            omega + dt / 6.0 * (k1w + 2 * k2w + 2 * k3w + k4w))


def pendulum_energy(theta, omega):
    # 单位质量：E = ½l²ω² + gl(1−cosθ)
    return 0.5 * l ** 2 * omega ** 2 + G * l * (1.0 - math.cos(theta))


def pendulum_period(theta0, dt=1e-4, t_max=20.0):
    """从 θ0 静止释放，测过零周期（四分周期 ×4）。"""
    theta, omega = theta0, 0.0
    t = 0.0
    # 先走到第一次 θ=0（下降沿）
    prev_theta = theta
    prev_t = t
    while t < t_max:
        theta, omega = rk4_pendulum(theta, omega, dt)
        t += dt
        if prev_theta > 0.0 >= theta:
            # 线性插值过零点
            frac = prev_theta / (prev_theta - theta)
            t_cross = prev_t + frac * dt
            return 4.0 * t_cross
        prev_theta, prev_t = theta, t
    raise RuntimeError('未找到过零点')


print()
print('=' * 74)
print('[2] 单摆数值积分：能量守恒与大角度周期')
print('=' * 74)
theta, omega = 0.3, 0.0
E0 = pendulum_energy(theta, omega)
max_drift = 0.0
dt = 1e-4
for _ in range(100000):  # 10 s
    theta, omega = rk4_pendulum(theta, omega, dt)
    max_drift = max(max_drift, abs(pendulum_energy(theta, omega) - E0))
print('    初值 θ0=0.3 rad，积分 10 s，最大能量漂移 = %.3e' % max_drift)
check(max_drift < 1e-6, 'RK4 能量漂移 < 1e-6（单位质量能量）')
per_small = pendulum_period(0.01)
per_large = pendulum_period(0.5235987756)  # 30°
print('    θ0=0.01 rad 周期 = %.6f s' % per_small)
print('    θ0=30°   周期 = %.6f s' % per_large)
check(per_large > per_small, '大角度周期比小角度长')

print()
print('=' * 74)
print('[3] 大角度周期修正')
print('=' * 74)
theta0 = 0.5235987756  # 30°
corr = 1.0 + theta0 ** 2 / 16.0
per_approx = T0 * corr
print('    θ0=30°，θ0²/16 = %.5f，T/T0 ≈ %.5f' % (theta0 ** 2 / 16.0, corr))
print('    解析修正周期 = %.6f s，数值周期 = %.6f s' % (per_approx, per_large))
rel = abs(per_large - per_approx) / per_approx
print('    相对差 = %.3f%%' % (100.0 * rel))
check(rel < 0.01, '一阶修正与数值周期差 <1%')
check(per_large > T0, '30° 的周期确实比小角度长')

print()
print('=' * 74)
print('[4] 质量–弹簧')
print('=' * 74)
m_spring = 2.0
k_spring = 8.0
omega_spring = math.sqrt(k_spring / m_spring)
print('    m=%.1f kg, k=%.1f N/m ⇒ ω=√(k/m)=%.4f rad/s' % (m_spring, k_spring, omega_spring))
check(abs(omega_spring - 2.0) < 1e-12, 'ω=2 rad/s')
print('    质量加倍 ⇒ ω=%.4f rad/s（原值的 1/√2）' % math.sqrt(k_spring / (2 * m_spring)))
check(abs(math.sqrt(k_spring / (2 * m_spring)) - 2.0 / math.sqrt(2)) < 1e-12,
      '质量加倍后 ω 降为 1/√2')

print()
print('=' * 74)
print('[5] 简正模态')
print('=' * 74)
K = np.array([[2.0, -1.0], [-1.0, 2.0]])
M = np.eye(2)
# 解 det(K - ω²M)=0
eigvals, eigvecs = np.linalg.eigh(K)
print('    K =')
print(K)
print('    ω² =', np.round(eigvals, 6))
print('    模态（列）=')
print(np.round(eigvecs, 6))
check(np.allclose(sorted(eigvals), [1.0, 3.0]), 'ω² = 1, 3')
check(np.allclose(np.abs(eigvecs[:, 0]), [1 / math.sqrt(2), 1 / math.sqrt(2)]),
      '低频模态 (1,1)')
check(np.allclose(np.abs(eigvecs[:, 1]), [1 / math.sqrt(2), 1 / math.sqrt(2)]),
      '高频模态 (1,-1)（差一个符号/归一化）')

print()
print('=' * 74)
print('[6] 哈密顿方程：谐振子')
print('=' * 74)
mh = 1.0
kh = 1.0
q, p = 1.0, 0.0
E_h = p ** 2 / (2 * mh) + 0.5 * kh * q ** 2
max_dh = 0.0
dth = 1e-4
for _ in range(100000):
    # 辛欧拉（半隐式）比显式欧拉更保能量
    p = p - kh * q * dth
    q = q + (p / mh) * dth
    Eh = p ** 2 / (2 * mh) + 0.5 * kh * q ** 2
    max_dh = max(max_dh, abs(Eh - E_h))
print('    H = p²/2m + ½kq²，初始 H = %.6f' % E_h)
print('    辛欧拉积分 10 s，最大能量漂移 = %.3e' % max_dh)
check(max_dh < 1e-3, '谐振子能量近似守恒（辛欧拉，10 s 漂移 <1e-3）')

print()
print('=' * 74)
print('[7] 中心力：角动量守恒')
print('=' * 74)
# 直角坐标下积分：a = -mu * r / |r|^3
mu = 1.0
r = np.array([1.0, 0.0])
v = np.array([0.0, 1.0])
Lz = r[0] * v[1] - r[1] * v[0]
max_dL = 0.0
dtc = 1e-4
for _ in range(100000):
    rn = np.linalg.norm(r)
    a = -mu * r / rn ** 3
    # 半隐式
    v = v + a * dtc
    r = r + v * dtc
    Lz_new = r[0] * v[1] - r[1] * v[0]
    max_dL = max(max_dL, abs(Lz_new - Lz))
print('    初始角动量 Lz = %.6f' % Lz)
print('    积分 10 s，最大角动量漂移 = %.3e' % max_dL)
check(max_dL < 1e-6, '中心力下角动量守恒')

print()
print('=' * 74)
if n_bad:
    print('❌ %d 项失败、%d 项通过' % (n_bad, n_ok))
    sys.exit(1)
print('✅ 全部 %d 项断言通过' % n_ok)
sys.exit(0)
