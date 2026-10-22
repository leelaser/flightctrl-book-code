# -*- coding: utf-8 -*-
"""卷 1 第 17 章 · 刚体动力学与惯量张量 —— 可执行验证

配套课文：`卷 1 第 17 章`

验证八件事：

  [1] 6 个自由度 = 3（平动）+ 2（球面）+ 1（绕连线转）
  [2] 旋转坐标系的速度/向心加速度：v = ω×ρ，a = ω×(ω×ρ)
  [3] 四点多质量模型的惯量：Ixx=Iyy=2mR²、Izz=4mR²
  [4] 平行轴定理：电池偏移后的惯量增量 m d²
  [5] 立方体（角点）主惯量：1/6、11/12、11/12（ma²），迹 = 2
  [6] 无力矩转动的中间轴定理：I=1,2,3 时绕轴 2 自旋不稳定
  [7] 陀螺力矩：ω=(5,0,2)、I=diag(0.01,0.01,0.02) ⇒ (0,−0.1,0) N·m
  [8] 惯量变化对二阶响应的影响：I=0.01/0.02/0.04 的 ωn、ζ

设计原则：
  · 断言两边来源不同：解析公式 / 数值积分 / 线性化系数
  · 中间轴定理用 RK4 直接积分欧拉方程，不只抄线性化结论
  · 所有数都打印出来，方便和正文逐项对照

运行：`python ch17_rigid.py` —— 全部断言通过才返回 0
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


print('=' * 74)
print('[1] 刚体的自由度')
print('=' * 74)
dof = 3 + 2 + 1
print('    第 1 点 %d 个 + 第 2 点 %d 个 + 第 3 点 %d 个 = %d' % (3, 2, 1, dof))
check(dof == 6, '刚体有 6 个独立坐标（与质点数 N 无关）')

print()
print('=' * 74)
print('[2] 旋转坐标系：速度与向心加速度')
print('=' * 74)
rho = np.array([1.0, 0.0, 0.0])
omega = np.array([0.0, 0.0, 2.0])
v = np.cross(omega, rho)
a = np.cross(omega, np.cross(omega, rho))
print('    rho =', rho)
print('    omega =', omega)
print('    v = omega x rho =', v)
print('    a = omega x (omega x rho) =', a)
check(np.allclose(v, [0.0, 2.0, 0.0]), 'v = (0, 2, 0) m/s')
check(np.allclose(a, [-4.0, 0.0, 0.0]), '向心加速度 = (-4, 0, 0) m/s²，大小 ω²ρ = 4')

print()
print('=' * 74)
print('[3] 四点多质量模型的惯量')
print('=' * 74)
m = 0.25
R = 0.2
Ixx = Iyy = 2.0 * m * R ** 2
Izz = 4.0 * m * R ** 2
print('    m = %.2f kg, R = %.2f m' % (m, R))
print('    Ixx = Iyy = 2mR² = %.4f kg·m²' % Ixx)
print('    Izz = 4mR²     = %.4f kg·m²' % Izz)
check(abs(Ixx - 0.02) < 1e-12 and abs(Izz - 0.04) < 1e-12,
      'Ixx=Iyy=0.02、Izz=0.04，且 Izz/Ixx≈2')

print()
print('=' * 74)
print('[4] 平行轴定理：电池偏移')
print('=' * 74)
I0 = 0.02
mb = 0.5
d = 0.1
I1 = I0 + mb * d ** 2
print('    基础 Ixx = %.3f, 电池 m=%.1f kg, 偏移 d=%.1f m' % (I0, mb, d))
print('    增量 m d² = %.4f kg·m²' % (mb * d ** 2))
print('    新 Ixx = %.4f kg·m²（增加 %.0f%%）' % (I1, 100 * (I1 / I0 - 1)))
check(abs(I1 - 0.025) < 1e-12, '新 Ixx = 0.025 kg·m²')

print()
print('=' * 74)
print('[5] 立方体（原点在角点）的主惯量')
print('=' * 74)
a_side = 1.0
mass = 1.0
I_cube = mass * a_side ** 2 * np.array([
    [2.0 / 3.0, -1.0 / 4.0, -1.0 / 4.0],
    [-1.0 / 4.0, 2.0 / 3.0, -1.0 / 4.0],
    [-1.0 / 4.0, -1.0 / 4.0, 2.0 / 3.0],
])
eig = np.linalg.eigvalsh(I_cube)
print('    惯量矩阵 = ma² *')
print(I_cube / (mass * a_side ** 2))
print('    本征值 =', np.round(eig, 8))
print('    迹 = %.6f，本征值之和 = %.6f' % (np.trace(I_cube), eig.sum()))
check(np.allclose(sorted(eig), [1.0 / 6.0, 11.0 / 12.0, 11.0 / 12.0]),
      '主惯量 = 1/6、11/12、11/12（ma²）')
check(abs(np.trace(I_cube) - 2.0) < 1e-12, '迹 = 2')

print()
print('=' * 74)
print('[6] 无力矩转动与中间轴定理')
print('=' * 74)
I = np.array([1.0, 2.0, 3.0])
print('    I = (1, 2, 3) kg·m²；绕每个轴自旋，初始横向扰动 1e-3，RK4 积分 1 s')
print('    线性化系数：轴1 = %.4f，轴2 = %.4f，轴3 = %.4f'
      % ((I[2] - I[0]) * (I[0] - I[1]) / (I[1] * I[2]),
         (I[0] - I[1]) * (I[1] - I[2]) / (I[0] * I[2]),
         (I[1] - I[2]) * (I[2] - I[0]) / (I[0] * I[1])))


def euler_rhs(w, inertia):
    """dω/dt = I^{-1} (τ − ω×(Iω))，无力矩 τ=0。"""
    return -np.cross(w, inertia * w) / inertia


def rk4_step(w, inertia, dt):
    k1 = euler_rhs(w, inertia)
    k2 = euler_rhs(w + 0.5 * dt * k1, inertia)
    k3 = euler_rhs(w + 0.5 * dt * k2, inertia)
    k4 = euler_rhs(w + dt * k3, inertia)
    return w + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def transverse_growth(axis, inertia, t_end=1.0, dt=1e-4):
    w = np.zeros(3)
    w[axis] = 10.0
    for j in range(3):
        if j != axis:
            w[j] = 1e-3
    w0 = w.copy()
    steps = int(round(t_end / dt))
    for _ in range(steps):
        w = rk4_step(w, inertia, dt)
    trans = [abs(w[j]) for j in range(3) if j != axis]
    return max(trans) / max(abs(w0[j]) for j in range(3) if j != axis)


growth = [transverse_growth(k, I) for k in range(3)]
for k, g in enumerate(growth, 1):
    print('    绕轴 %d 的横向增长倍数（1 s）：%.3g' % (k, g))
check(growth[1] > 10.0, '绕中间轴（轴 2）的横向扰动显著增长，不稳定')
check(growth[0] < 2.0 and growth[2] < 2.0, '绕轴 1/3 的横向扰动有界，稳定')

print()
print('=' * 74)
print('[7] 陀螺力矩')
print('=' * 74)
Iq = np.array([0.01, 0.01, 0.02])
wq = np.array([5.0, 0.0, 2.0])
Tg = np.cross(wq, Iq * wq)
print('    I = diag(0.01, 0.01, 0.02), ω = (5, 0, 2)')
print('    Iω =', Iq * wq)
print('    ω×Iω =', Tg)
check(np.allclose(Tg, [0.0, -0.1, 0.0]), '陀螺力矩 = (0, −0.1, 0) N·m，作用在俯仰轴')

print()
print('=' * 74)
print('[8] 惯量变化对二阶响应的影响')
print('=' * 74)
Kp = 0.5
Kr = 0.2
print('    Kp=%.1f, Kr=%.1f' % (Kp, Kr))
for inertia in (0.01, 0.02, 0.04):
    wn = math.sqrt(Kp / inertia)
    zeta = Kr / (2.0 * math.sqrt(Kp * inertia))
    settle = 4.0 / (zeta * wn)
    print('    I=%.2f: ωn=%.3f rad/s, ζ=%.3f, 约稳定时间=%.2f s'
          % (inertia, wn, zeta, settle))
    if abs(inertia - 0.02) < 1e-12:
        check(abs(wn - 5.0) < 1e-9 and abs(zeta - 1.0) < 1e-9,
              'I=0.02 时 ωn=5、ζ=1')
    if abs(inertia - 0.04) < 1e-12:
        check(abs(wn - math.sqrt(12.5)) < 1e-9 and abs(zeta - 0.70710678) < 1e-6,
              'I=0.04 时 ωn≈3.54、ζ≈0.707')

print()
print('=' * 74)
if n_bad:
    print('❌ %d 项失败、%d 项通过' % (n_bad, n_ok))
    sys.exit(1)
print('✅ 全部 %d 项断言通过' % n_ok)
sys.exit(0)
