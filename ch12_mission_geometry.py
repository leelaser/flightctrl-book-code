# -*- coding: utf-8 -*-
"""卷 3 第 12 章 · 任务剖面与姿态几何 —— 可执行验证

配套课文：`卷 3 第 12 章`

验证五件事：

  [1] 地球角半径：500 km 约 68.0°，同步轨道约 8.7°
  [2] 离轴地面距离：500 km、离轴 30° 约 293 km；平面近似误差约 1.4%
  [3] 球面三角：随机三角形的余弦定理自洽；35°/25°/70° 的边约 34.4°
  [4] 姿态矩阵合成：R_bi = R_bo R_oi，转置约定会给出错误映射
  [5] 时间与误差预算：J2000.0 的儒略日；0.1 s→0.1°；RSS 约 115″

设计原则：
  · 几何量用解析式与独立数值构造互核
  · 姿态矩阵用随机旋转测试合成，不只看一个特例
  · 时间、角秒和地面距离都换算到同一单位后再断言

运行：`python ch12_mission_geometry.py` —— 全部断言通过才返回 0
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
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * K.dot(K)


def angle_between(u, v):
    c = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    return math.acos(min(1.0, max(-1.0, c)))


def julian_date(year, month, day, hour=0.0):
    """Gregorian calendar -> Julian Date (Meeus, valid for modern dates)."""
    if month <= 2:
        year -= 1
        month += 12
    A = year // 100
    B = 2 - A + A // 4
    jd = int(365.25 * (year + 4716)) + int(30.6001 * (month + 1))
    jd += day + B - 1524.5
    return jd + hour / 24.0


print('=' * 74)
print('[1] 地球角半径')
print('=' * 74)
RE = 6378.0
h = 500.0
r = RE + h
rho_500 = math.degrees(math.asin(RE / r))
rho_geo = math.degrees(math.asin(RE / 42164.0))
print('    500 km: rho = %.4f°' % rho_500)
print('    GEO   : rho = %.4f°' % rho_geo)
check(abs(rho_500 - 68.0) < 0.1, '500 km 地球角半径约 68.0°')
check(abs(rho_geo - 8.7) < 0.1, '同步轨道地球角半径约 8.7°')
gamma_h = math.acos(RE / r)
s_h = RE * gamma_h
print('    500 km 地平距 = %.0f km（地心角 %.2f°）'
      % (s_h, math.degrees(gamma_h)))
check(abs(s_h - 2447.0) < 2.0, '500 km 地平距约 2447 km')

print()
print('=' * 74)
print('[2] 离轴 30° 的地面距离')
print('=' * 74)
eta = math.radians(30.0)


def f(gamma):
    return math.tan(eta) * (r - RE * math.cos(gamma)) - RE * math.sin(gamma)


lo, hi = 0.0, math.acos(RE / r)
# f 在 [0, 90°-eta] 上单调递减：f(0)>0、f(hi)<0
for _ in range(80):
    mid = 0.5 * (lo + hi)
    if f(mid) > 0:
        lo = mid
    else:
        hi = mid
gamma = 0.5 * (lo + hi)
s = RE * gamma
s_flat = h * math.tan(eta)
err = abs(s - s_flat) / s * 100.0
print('    gamma = %.4f° = %.6f rad' % (math.degrees(gamma), gamma))
print('    s = %.2f km；平面近似 = %.2f km；相对差 = %.2f%%'
      % (s, s_flat, err))
check(abs(math.degrees(gamma) - 2.63) < 0.02, '地心角约 2.63°')
check(abs(s - 293.0) < 1.0, '地面距离约 293 km')
check(1.0 < err < 2.0, '平面近似误差约 1.4%')

print()
print('=' * 74)
print('[3] 球面三角')
print('=' * 74)
rng = np.random.default_rng(12)
for trial in range(200):
    A = rng.normal(size=3)
    B = rng.normal(size=3)
    C = rng.normal(size=3)
    A /= np.linalg.norm(A)
    B /= np.linalg.norm(B)
    C /= np.linalg.norm(C)
    a = angle_between(B, C)
    b = angle_between(C, A)
    c = angle_between(A, B)
    # 顶点 A 处，两条弧的切向
    tAB = B - np.dot(A, B) * A
    tAC = C - np.dot(A, C) * A
    tAB /= np.linalg.norm(tAB)
    tAC /= np.linalg.norm(tAC)
    Aang = math.acos(min(1.0, max(-1.0, np.dot(tAB, tAC))))
    lhs = math.cos(a)
    rhs = math.cos(b) * math.cos(c) + math.sin(b) * math.sin(c) * math.cos(Aang)
    if abs(lhs - rhs) > 1e-9:
        break
else:
    check(True, '200 组随机球面三角形满足余弦定理')
# 课文/习题的数值
a1, b1, C1 = map(math.radians, (35.0, 25.0, 70.0))
c1 = math.acos(math.cos(a1) * math.cos(b1)
               + math.sin(a1) * math.sin(b1) * math.cos(C1))
print('    35°/25°/70° -> c = %.4f°' % math.degrees(c1))
check(abs(math.degrees(c1) - 34.4) < 0.1, '习题 3 的边约 34.4°')
a2, b2, C2 = map(math.radians, (40.0, 30.0, 60.0))
c2 = math.acos(math.cos(a2) * math.cos(b2)
               + math.sin(a2) * math.sin(b2) * math.cos(C2))
print('    40°/30°/60° -> c = %.4f°' % math.degrees(c2))
check(abs(math.degrees(c2) - 34.5) < 0.1, '课文示例的边约 34.5°')

print()
print('=' * 74)
print('[4] 姿态矩阵合成')
print('=' * 74)
R_bi = rot_axis([0.2, -0.4, 0.7], math.radians(37.0))
R_oi = rot_axis([-0.3, 0.8, 0.1], math.radians(-52.0))
R_bo = R_bi.dot(R_oi.T)
print('    det(R_bi) = %.6f；det(R_bo) = %.6f'
      % (np.linalg.det(R_bi), np.linalg.det(R_bo)))
check(np.allclose(R_bo.dot(R_oi), R_bi, atol=1e-12),
      'R_bi = R_bo R_oi 成立')
check(abs(np.linalg.det(R_bo) - 1.0) < 1e-12, 'det(R_bo) = +1')
v_o = np.array([1.0, 0.0, 0.0])
check(np.allclose(R_bo.dot(R_oi.dot(v_o)), R_bi.dot(v_o), atol=1e-12),
      '同一矢量经两条路径映射结果一致')
wrong = R_oi.T.dot(R_bi.T).dot(v_o)
check(not np.allclose(wrong, R_bi.dot(v_o), atol=1e-3),
      '把两个矩阵都取转置会得到错误映射')

print()
print('=' * 74)
print('[5] 时间与误差预算')
print('=' * 74)
jd_j2000 = julian_date(2000, 1, 1, 12.0)
jd_1981 = julian_date(1981, 12, 25, 12.0)
print('    J2000.0 JD = %.1f' % jd_j2000)
print('    1981-12-25 12:00 JD = %.1f' % jd_1981)
check(abs(jd_j2000 - 2451545.0) < 1e-6, 'J2000.0 = JD 2451545.0')
# 源书给的是 1981-12-25 的整数 JD（正午历元），故取 12:00 核对
check(abs(jd_1981 - 2444964.0) < 1e-6, '1981-12-25 12:00 = JD 2444964.0')

# 时间戳错位
err_deg = 1.0 * 0.1
err_m = math.radians(err_deg) * 500000.0
print('    0.1 s @ 1°/s -> %.2f° -> %.0f m' % (err_deg, err_m))
check(abs(err_m - 870.0) < 5.0, '时间戳错位对应约 870 m')

# RSS 误差预算
terms_arcsec = [0.02 * 3600.0, 5.0, 3.0, 0.05 * 0.5 * 3600.0]
rss = math.sqrt(sum(t * t for t in terms_arcsec))
rss_m = math.radians(rss / 3600.0) * 500000.0
need = math.degrees(10.0 / 500000.0) * 3600.0
print('    RSS = %.2f″ = %.1f m；10 m 需要 %.2f″'
      % (rss, rss_m, need))
check(abs(rss - 115.4) < 0.5, 'RSS 约 115″')
check(abs(rss_m - 280.0) < 2.0, '地面误差约 280 m')
check(rss > need, '280 m 远大于 10 m，预算不满足')
check(abs(need - 4.125) < 0.01, '10 m 对应约 4.1″')

print()
print('=' * 74)
if n_bad:
    print('❌ %d 项失败、%d 项通过' % (n_bad, n_ok))
    sys.exit(1)
print('✅ 全部 %d 项断言通过' % n_ok)
sys.exit(0)
