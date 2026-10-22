# -*- coding: utf-8 -*-
"""卷 3 第 10 章 · 航天器姿态确定与控制案例 —— 可执行验证

配套课文：`卷 3 第 10 章`

验证八件事：

  [1] 指向误差预算：RSS = √(a²+s²+k²+c²)，最坏 = 线性相加
  [2] 400 km 气动力矩的量级；200 km/400 km 的比值
  [3] 磁力矩 M×B；角度 90°/30° 的差别
  [4] 太阳光压力矩的量级
  [5] 干扰预算 → 每轨角动量 → 轮子容量/卸载周期
  [6] 二阶姿态环：ωn=√(Kp/I)、ζ=Kr/(2√(KpI))
  [7] TRIAD：两组正交基、T=M Nᵀ、正交性/行列式
  [8] 故障时间：轮子饱和、星敏感器失效后的陀螺漂移

设计原则：
  · 解析公式与数值结果互核；TRIAD 检查 T·N=M，不只看单个元素
  · 所有数都打印出来，方便和正文逐项对照
  · 本章是理论/案例章，脚本只复算**关键数值例子**，不是所有定义/常数的来源

运行：`python ch10_adcs.py` —— 全部断言通过才返回 0
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
print('[1] 指向误差预算')
print('=' * 74)
a, s, k, c = 0.05, 0.01, 0.03, 0.02
rss = math.sqrt(a ** 2 + s ** 2 + k ** 2 + c ** 2)
worst = a + s + k + c
print('    a=%.3f, s=%.3f, k=%.3f, c=%.3f 度' % (a, s, k, c))
print('    RSS = %.5f 度' % rss)
print('    最坏 = %.3f 度' % worst)
check(abs(rss - 0.06245) < 1e-4, 'RSS ≈ 0.062°')
check(abs(worst - 0.11) < 1e-12, '最坏 = 0.11°')
check(max(a, s, k, c) == a, '最大项是 a=0.05°，应优先改进')

print()
print('=' * 74)
print('[2] 气动力矩（400 km）')
print('=' * 74)
V = 7670.0
rho400 = 4e-12
rho200 = 3e-10
S = 5.0
Cd = 2.0
r = 0.1
F = 0.5 * rho400 * V ** 2 * S * Cd
T = F * r
print('    V=%.0f m/s, ρ400=%.0e kg/m³, S=%.0f m², Cd=%.0f, r=%.1f m'
      % (V, rho400, S, Cd, r))
print('    F = %.4e N' % F)
print('    T = %.4e N·m' % T)
check(abs(F - 1.176e-3) / 1.176e-3 < 0.01, 'F ≈ 1.18e-3 N')
check(abs(T - 1.176e-4) / 1.176e-4 < 0.01, 'T ≈ 1.18e-4 N·m')
print('    200 km / 400 km 密度比 = %.0f' % (rho200 / rho400))
check(abs((rho200 / rho400) - 75.0) < 1e-9, '气动力矩在 200 km 约为 400 km 的 75 倍')

print()
print('=' * 74)
print('[3] 磁力矩')
print('=' * 74)
M = 0.1
B = 3e-5
T90 = M * B
T30 = M * B * math.sin(math.radians(30.0))
print('    M=%.1f A·m², B=%.0e T' % (M, B))
print('    90°: T = %.2e N·m' % T90)
print('    30°: T = %.2e N·m' % T30)
check(abs(T90 - 3e-6) < 1e-12, '90° 磁力矩 = 3e-6 N·m')
check(abs(T30 - 1.5e-6) < 1e-12, '30° 磁力矩 = 1.5e-6 N·m')

print()
print('=' * 74)
print('[4] 太阳光压力矩')
print('=' * 74)
Ps = 1361.0 / 3e8
K = 0.5
Fs = (1 + K) * Ps * S
Ts = Fs * r
print('    Ps = I/c = %.3e N/m²' % Ps)
print('    Fs = (1+K)PsS = %.3e N' % Fs)
print('    Ts = %.3e N·m' % Ts)
check(abs(Ts - 3.4e-6) / 3.4e-6 < 0.02, '太阳光压力矩 ≈ 3.4e-6 N·m')

print()
print('=' * 74)
print('[5] 干扰预算 → 轮子容量')
print('=' * 74)
T_grav = 6.7e-5
T_mag = 3e-6
T_sun = 3.4e-6
T_total = T + T_grav + T_mag + T_sun
orbit = 6000.0
H_orbit = T_total * orbit
wheel_cap = 1.3
n_orbit = wheel_cap / H_orbit
print('    气动 %.2e + 重力梯度 %.2e + 磁 %.2e + 太阳 %.2e'
      % (T, T_grav, T_mag, T_sun))
print('    总干扰 = %.3e N·m' % T_total)
print('    每轨角动量 = %.3f N·m·s' % H_orbit)
print('    轮子容量 %.1f N·m·s ⇒ 约 %.2f 轨饱和' % (wheel_cap, n_orbit))
check(abs(T_total - 1.9e-4) / 1.9e-4 < 0.05, '总干扰 ≈ 1.9e-4 N·m')
check(abs(H_orbit - 1.14) < 0.05, '每轨角动量 ≈ 1.14 N·m·s')
check(1.0 < n_orbit < 1.3, '轮子约 1.1 轨饱和')

print()
print('=' * 74)
print('[6] 二阶姿态环')
print('=' * 74)
I = 20.0
Kp = 20.0
Kr = 40.0
wn = math.sqrt(Kp / I)
zeta = Kr / (2.0 * math.sqrt(Kp * I))
print('    I=%.0f, Kp=%.0f, Kr=%.0f' % (I, Kp, Kr))
print('    ωn=%.4f rad/s, ζ=%.4f' % (wn, zeta))
check(abs(wn - 1.0) < 1e-12 and abs(zeta - 1.0) < 1e-12,
      'ωn=1 rad/s、ζ=1（临界阻尼）')
zeta2 = 20.0 / (2.0 * math.sqrt(Kp * I))
print('    Kr=20 时 ζ=%.2f（约 16%% 超调）' % zeta2)
check(abs(zeta2 - 0.5) < 1e-12, 'Kr=20 时 ζ=0.5')


def triad(u, v):
    """返回以 i=u/|u|、j=u×v/|u×v|、k=i×j 为列的矩阵。"""
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    i = u / np.linalg.norm(u)
    j = np.cross(u, v)
    j = j / np.linalg.norm(j)
    kvec = np.cross(i, j)
    return np.column_stack([i, j, kvec])


print()
print('=' * 74)
print('[7] TRIAD')
print('=' * 74)
uB = np.array([1.0, 0.0, 0.0])
vB = np.array([0.0, 1.0, 0.0])
uR = np.array([0.0, 1.0, 0.0])
vR = np.array([1.0, 0.0, 0.0])
M = triad(uB, vB)
N = triad(uR, vR)
T = M.dot(N.T)
print('    M =')
print(M)
print('    N =')
print(N)
print('    T = M Nᵀ =')
print(T)
print('    det(N) = %.6f, det(T) = %.6f' % (np.linalg.det(N), np.linalg.det(T)))
check(abs(np.linalg.det(N) - 1.0) < 1e-12, 'N 是右手正交基（det=+1）')
check(np.allclose(T.dot(N), M), 'T·N = M（TRIAD 的定义）')
check(np.allclose(T.dot(uR), uB), 'T 把参考系的 u 映到本体系的 u')
check(np.allclose(T.dot(T.T), np.eye(3)), 'T 正交')
check(abs(np.linalg.det(T) - 1.0) < 1e-12, 'T 是转动（det=+1）')

print()
print('=' * 74)
print('[8] 故障时间')
print('=' * 74)
remaining = 0.13
dist = 1.9e-4
t_sat = remaining / dist
print('    轮子剩 0.13 N·m·s，干扰 1.9e-4 N·m ⇒ 到饱和 %.0f s = %.1f min'
      % (t_sat, t_sat / 60.0))
check(abs(t_sat - 684.2) < 1.0, '轮子约 684 s（11.4 min）饱和')
bias1 = 0.01
bias2 = 0.05
t1 = 0.1 / bias1
t2 = 0.1 / bias2
print('    陀螺零偏 0.01°/h ⇒ 漂 0.1° 需 %.0f h' % t1)
print('    陀螺零偏 0.05°/h ⇒ 漂 0.1° 需 %.0f h' % t2)
check(abs(t1 - 10.0) < 1e-12, '0.01°/h 时 10 h 漂 0.1°')
check(abs(t2 - 2.0) < 1e-12, '0.05°/h 时 2 h 漂 0.1°')

print()
print('=' * 74)
if n_bad:
    print('❌ %d 项失败、%d 项通过' % (n_bad, n_ok))
    sys.exit(1)
print('✅ 全部 %d 项断言通过' % n_ok)
sys.exit(0)
