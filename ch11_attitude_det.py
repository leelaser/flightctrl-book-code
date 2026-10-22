# -*- coding: utf-8 -*-
"""卷 3 第 11 章 · 姿态确定的估计方法 —— 可执行验证

配套课文：`卷 3 第 11 章`

验证六件事：

  [1] TRIAD：两组正交基给出精确旋转 T=M Nᵀ
  [2] 退化放大：两矢量夹角 10° 比 90° 的 TRIAD 误差大（Monte Carlo）
  [3] Wahba/SVD：多矢量最小二乘的代价不高于 TRIAD
  [4] 可观性：单矢量时绕观测矢量的转角不可观；两非平行矢量可观
  [5] 单轴 Kalman：陀螺传播 + 角度观测，零偏估计收敛、协方差下降
  [6] 工程数值：10° 夹角、0.1° 噪声；理论估计 0.58°，
      两矢量都加噪的 Monte Carlo 约 0.67°

设计原则：
  · 解析结论与 Monte Carlo/数值滤波互核
  · TRIAD 与 Wahba 用同一组观测比较代价，不各写一套数据
  · 可观性用“构造零空间旋转”直接验证，不只看条件数

运行：`python ch11_attitude_det.py` —— 全部断言通过才返回 0
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


def rot_axis(axis, angle):
    """Rodrigues：绕单位轴转 angle 的旋转矩阵（主动/参考→本体约定）。"""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    Kx = np.array([[0.0, -axis[2], axis[1]],
                   [axis[2], 0.0, -axis[0]],
                   [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * Kx + (1.0 - math.cos(angle)) * Kx.dot(Kx)


def triad(u, v):
    i = u / np.linalg.norm(u)
    j = np.cross(u, v)
    j = j / np.linalg.norm(j)
    k = np.cross(i, j)
    return np.column_stack([i, j, k])


def rot_error_deg(A, B):
    """两个旋转矩阵之间的转角（度）。"""
    c = (np.trace(A.T.dot(B)) - 1.0) / 2.0
    c = min(1.0, max(-1.0, c))
    return math.degrees(math.acos(c))


print('=' * 74)
print('[1] TRIAD：精确旋转')
print('=' * 74)
rng = np.random.default_rng(7)
R_true = rot_axis([0.3, -0.2, 0.9], math.radians(40.0))
r1 = np.array([1.0, 0.0, 0.0])
r2 = np.array([0.0, 1.0, 0.0])
b1 = R_true.dot(r1)
b2 = R_true.dot(r2)
M = triad(b1, b2)
N = triad(r1, r2)
T = M.dot(N.T)
print('    R_true =')
print(np.round(R_true, 6))
print('    T =')
print(np.round(T, 6))
print('    误差 = %.3e 度' % rot_error_deg(T, R_true))
check(np.allclose(T, R_true, atol=1e-12), 'T 精确等于 R_true')
check(abs(np.linalg.det(T) - 1.0) < 1e-12, 'det T = +1')

print()
print('=' * 74)
print('[2] 退化放大：10° vs 90°')
print('=' * 74)


def mc_triad(angle_deg, sigma_deg=0.1, trials=500):
    ang = math.radians(angle_deg)
    v = np.array([math.cos(ang), math.sin(ang), 0.0])
    sig = math.radians(sigma_deg)
    errs = []
    for _ in range(trials):
        n1 = rng.normal(0.0, sig, 3)
        n2 = rng.normal(0.0, sig, 3)
        bb1 = R_true.dot(r1) + n1
        bb2 = R_true.dot(v) + n2
        bb1 = bb1 / np.linalg.norm(bb1)
        bb2 = bb2 / np.linalg.norm(bb2)
        MM = triad(bb1, bb2)
        NN = triad(r1, v)
        TT = MM.dot(NN.T)
        errs.append(rot_error_deg(TT, R_true))
    return float(np.mean(errs)), float(np.std(errs))


e90, s90 = mc_triad(90.0)
e10, s10 = mc_triad(10.0)
print('    90° 夹角：平均误差 %.4f°（σ=%.4f°）' % (e90, s90))
print('    10° 夹角：平均误差 %.4f°（σ=%.4f°）' % (e10, s10))
print('    理论放大 1/sin10° = %.2f' % (1.0 / math.sin(math.radians(10.0))))
check(e10 > e90, '10° 夹角的 TRIAD 误差更大')
check(e10 > 0.2 and e90 < 0.2, '量级符合“小夹角放大”的直觉')

print()
print('=' * 74)
print('[3] Wahba/SVD')
print('=' * 74)
refs = [np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0])]
sig = math.radians(0.2)
cost_w, cost_t = [], []
for _ in range(300):
    body = []
    for r in refs:
        bb = R_true.dot(r) + rng.normal(0.0, sig, 3)
        body.append(bb / np.linalg.norm(bb))
    # Wahba: B = Σ b rᵀ
    B = sum(np.outer(b, r) for b, r in zip(body, refs))
    U, S, Vt = np.linalg.svd(B)
    D = np.diag([1.0, 1.0, np.linalg.det(U.dot(Vt))])
    R_w = U.dot(D).dot(Vt)
    J_w = sum(np.linalg.norm(b - R_w.dot(r)) ** 2
              for b, r in zip(body, refs))
    # TRIAD: 用前两个
    MM = triad(body[0], body[1])
    NN = triad(refs[0], refs[1])
    R_t = MM.dot(NN.T)
    J_t = sum(np.linalg.norm(b - R_t.dot(r)) ** 2
              for b, r in zip(body, refs))
    cost_w.append(J_w)
    cost_t.append(J_t)
print('    Wahba 平均代价 = %.6f' % np.mean(cost_w))
print('    TRIAD 平均代价 = %.6f' % np.mean(cost_t))
check(np.mean(cost_w) <= np.mean(cost_t) + 1e-9,
      'Wahba 的平均代价不高于 TRIAD')

print()
print('=' * 74)
print('[4] 可观性')
print('=' * 74)
# 单矢量：绕 b 的任意旋转仍把 r 映到 b
b = R_true.dot(r1)
for phi in (0.1, 1.0, 2.0):
    R_phi = rot_axis(b, phi).dot(R_true)
    residual = np.linalg.norm(b - R_phi.dot(r1))
    print('    单矢量 φ=%.1f rad：残差 = %.3e' % (phi, residual))
    check(residual < 1e-12, '单矢量：绕观测轴 φ=%.1f 的旋转不改变观测' % phi)
# 两非平行矢量：绕 b1 转 phi 会破坏第二个矢量
b2v = R_true.dot(r2)
for phi in (0.5,):
    R_phi = rot_axis(b, phi).dot(R_true)
    residual = np.linalg.norm(b2v - R_phi.dot(r2))
    print('    两矢量 φ=%.1f rad：第二矢量残差 = %.3e' % (phi, residual))
    check(residual > 1e-3, '两非平行矢量：绕 b1 的旋转会破坏第二个观测')

print()
print('=' * 74)
print('[5] 单轴 Kalman：陀螺 + 角度观测')
print('=' * 74)
dt = 0.01
T_total = 200.0
steps = int(T_total / dt)
theta_true = 0.0
rate_true = 0.1
bias_true = 0.01
# 状态 [theta, bias]
x = np.array([0.0, 0.0])
P = np.diag([0.1 ** 2, 0.02 ** 2])
Q = np.diag([1e-8, 1e-10])
R_meas = (math.radians(0.5)) ** 2
H = np.array([[1.0, 0.0]])
F = np.array([[1.0, -dt], [0.0, 1.0]])
bias_hist = []
theta_err_hist = []
for k in range(steps):
    # 真值
    theta_true += rate_true * dt
    # 陀螺测量（含零偏+噪声）
    omega_m = rate_true + bias_true + rng.normal(0.0, 1e-4)
    # 预测
    x = F.dot(x) + np.array([omega_m * dt, 0.0])
    P = F.dot(P).dot(F.T) + Q
    # 每 0.1 s 一次角度观测
    if k % 10 == 0:
        z = theta_true + rng.normal(0.0, math.radians(0.5))
        y = z - H.dot(x)[0]
        S = H.dot(P).dot(H.T) + R_meas
        K = P.dot(H.T) / S
        x = x + (K * y).ravel()
        P = (np.eye(2) - K.dot(H)).dot(P)
        bias_hist.append(x[1])
        theta_err_hist.append(abs(x[0] - theta_true))
print('    真实零偏 = %.5f rad/s' % bias_true)
print('    估计零偏（末值） = %.5f rad/s' % x[1])
print('    角度误差（末值） = %.5f rad = %.4f°'
      % (theta_err_hist[-1], math.degrees(theta_err_hist[-1])))
print('    零偏估计误差（末值） = %.5f rad/s' % abs(x[1] - bias_true))
check(abs(x[1] - bias_true) < 0.005, '零偏估计误差 < 0.005 rad/s')
check(theta_err_hist[-1] < math.radians(1.0), '末段角度误差 < 1°')
check(P[0, 0] < 0.1 ** 2, '角度协方差下降')
check(P[1, 1] < 0.02 ** 2, '零偏协方差下降')

print()
print('=' * 74)
print('[6] 10° 夹角、0.1° 噪声的 TRIAD 误差')
print('=' * 74)
print('    Monte Carlo 平均 = %.4f°；理论 0.1/sin10° = %.4f°'
      % (e10, 0.1 / math.sin(math.radians(10.0))))
check(abs(e10 - 0.1 / math.sin(math.radians(10.0))) < 0.3,
      '数值与 0.1/sinθ 同量级')

print()
print('=' * 74)
if n_bad:
    print('❌ %d 项失败、%d 项通过' % (n_bad, n_ok))
    sys.exit(1)
print('✅ 全部 %d 项断言通过' % n_ok)
sys.exit(0)
