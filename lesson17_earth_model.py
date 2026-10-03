# -*- coding: utf-8 -*-
"""第 15.7 周 · 10 导航坐标系与地球模型 —— 可执行验证

配套课文：`04-状态估计/10-导航坐标系与地球模型-严恭敏.md`
源书：严恭敏, 翁浚《捷联惯导算法与组合导航原理》第 3 章（PDF p47–p78）

本脚本验证课文的三条结论：

  [1] 两个曲率半径 R_M / R_N —— 在极点处必须【相等】，且等于 R_e²/R_p
  [2] 纬度反解必须【迭代】—— 初值取地心纬度，3~4 次收敛到毫米级
  [3] ★ 地心纬度 vs 地理纬度的差 —— 最大 ≈0.19°，对应地面 ≈21 km
      （【反向测试】：这两个量在赤道和极点【恰好相等】，所以判据必须在中间纬度取）

运行：`python lesson17_earth_model.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function

import math
import sys

# ── WGS-84 椭球常数 ──────────────────────────────────────────
R_E = 6378137.0                     # 长半轴 (m)
F = 1.0 / 298.257223563             # 扁率
E2 = 2 * F - F * F                  # 第一偏心率平方
R_P = R_E * (1 - F)                 # 短半轴 (m)

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


def R_M(L):
    """子午圈曲率半径（南北方向）"""
    return R_E * (1 - E2) / (1 - E2 * math.sin(L) ** 2) ** 1.5


def R_N(L):
    """卯酉圈曲率半径（东西方向）"""
    return R_E / math.sqrt(1 - E2 * math.sin(L) ** 2)


print('=' * 74)
print('[1] 两个曲率半径：R_M（南北）与 R_N（东西）')
print('=' * 74)
print('    WGS-84:  R_e=%.1f m   f=1/%.6f   e²=%.10f' % (R_E, 1 / F, E2))
print()
print('    %-10s %16s %16s %14s' % ('纬度 L', 'R_M (m)', 'R_N (m)', 'R_M/R_N'))
for deg in (0, 15, 30, 45, 60, 75, 90):
    L = math.radians(deg)
    print('    %-10d %16.3f %16.3f %14.6f' % (deg, R_M(L), R_N(L), R_M(L) / R_N(L)))

# 赤道：R_M = R_e(1−e²)，R_N = R_e
check(abs(R_M(0.0) - R_E * (1 - E2)) < 1e-6, '赤道处 R_M = R_e(1−e²) = %.3f' % R_M(0.0))
check(abs(R_N(0.0) - R_E) < 1e-6, '赤道处 R_N = R_e = %.1f' % R_N(0.0))

# ★ 极点：两者必须相等，且 = R_e²/R_p
pole_expect = R_E * R_E / R_P
check(abs(R_M(math.pi / 2) - R_N(math.pi / 2)) < 1e-6,
      '极点处 R_M = R_N（两方向曲率相同）')
check(abs(R_M(math.pi / 2) - pole_expect) < 1e-6,
      '极点处 R_M = R_e²/R_p = %.3f' % pole_expect)
check(abs(R_N(math.pi / 2) - pole_expect) < 1e-6,
      '极点处 R_N = R_e²/R_p = %.3f' % pole_expect)
print()
print('    ⇒ 极点处两方向曲率【相同】（因为椭球在两极是旋转对称的）')

print()
print('=' * 74)
print('[2] 大地坐标反解：纬度必须迭代')
print('=' * 74)


def geodetic_to_ecef(lat, lon, h):
    """正解 (L, λ, h) → (x, y, z)，原书式 3.1-35"""
    rn = R_N(lat)
    x = (rn + h) * math.cos(lat) * math.cos(lon)
    y = (rn + h) * math.cos(lat) * math.sin(lon)
    z = (rn * (1 - E2) + h) * math.sin(lat)
    return x, y, z


def ecef_to_geodetic(x, y, z, max_iter=20, tol=1e-12):
    """反解 (x, y, z) → (L, λ, h)。经度显式，纬度迭代。返回 (lat, lon, h, 迭代次数)"""
    lon = math.atan2(y, x)                     # 显式
    p = math.hypot(x, y)
    lat = math.atan2(z, p)                     # ★ 初值：地心纬度
    n_iter = 0
    for k in range(max_iter):
        rn = R_N(lat)
        lat_new = math.atan2(z + rn * E2 * math.sin(lat), p)
        n_iter = k + 1
        if abs(lat_new - lat) < tol:
            lat = lat_new
            break
        lat = lat_new
    rn = R_N(lat)
    h = p / math.cos(lat) - rn
    return lat, lon, h, n_iter


# 取一个不在赤道也不在极点的点（★ 判据必须在中间纬度取，见 [3] 的说明）
L0, LON0, H0 = math.radians(39.9), math.radians(116.4), 50.0    # 北京附近
x, y, z = geodetic_to_ecef(L0, LON0, H0)
lat, lon, h, it = ecef_to_geodetic(x, y, z)
print('    正解: L=%.6f° λ=%.6f° h=%.1f m' % (math.degrees(L0), math.degrees(LON0), H0))
print('    反解: L=%.6f° λ=%.6f° h=%.1f m   用了 %d 次迭代'
      % (math.degrees(lat), math.degrees(lon), h, it))
err_m = abs(h - H0)
check(abs(math.degrees(lat) - math.degrees(L0)) < 1e-9, '纬度反解误差 < 1e-9 度')
check(err_m < 1e-6, '椭球高反解误差 %.3e m（< 1 微米）' % err_m)
check(it <= 5, '迭代 %d 次收敛（≤5 次）' % it)

print()
print('    再看如果【不迭代】、直接用地心纬度当地理纬度，会差多少：')
p = math.hypot(x, y)
lat_geocentric = math.atan2(z, p)
d_deg = math.degrees(lat - lat_geocentric)
print('      地心纬度 = %.6f°   地理纬度 = %.6f°   差 = %.6f°'
      % (math.degrees(lat_geocentric), math.degrees(lat), d_deg))
check(abs(d_deg) > 0.1, '北京的差值 %.4f° 不可忽略' % d_deg)

print()
print('=' * 74)
print('[3] ★ 地心纬度 vs 地理纬度：差的最大值在哪、有多大')
print('=' * 74)


def geocentric_of_geodetic(L):
    """给定【地理纬度】L，求同一点的【地心纬度】

    由原书式 3.1-11：z = x(1−e²)tanL，而地心纬度 tanφ = z/x
    ⇒ tanφ = (1−e²)·tan L
    """
    return math.atan((1 - E2) * math.tan(L))


print('    %-10s %14s %14s %14s %14s' % ('地理纬度 L', '地心纬度 φ', '差 (°)', '地面距离 km', ''))
worst = (0.0, 0.0)
for deg in (0, 15, 30, 45, 60, 75, 90):
    L = math.radians(deg)
    phi = geocentric_of_geodetic(L)
    d = math.degrees(L - phi)
    km = abs(math.radians(d)) * R_E / 1000.0
    print('    %-10d %14.6f %14.6f %14.2f' % (deg, math.degrees(phi), d, km))
    if abs(d) > abs(worst[0]):
        worst = (d, km)

print()
print('    ★★ 反向测试：这两个量在【赤道】和【极点】恰好相等 ——')
print('       所以判据【不能】在这两处取，必须在中间纬度取。')
L_eq = math.radians(0.0)
L_pole = math.radians(90.0)
d_eq = math.degrees(L_eq - geocentric_of_geodetic(L_eq))
d_pole = math.degrees(L_pole - geocentric_of_geodetic(L_pole))
print('       赤道处差 = %.6f°    极点处差 = %.6f°' % (d_eq, d_pole))
check(abs(d_eq) < 1e-12, '赤道处两纬度相等（差 %.1e°）' % abs(d_eq))
check(abs(d_pole) < 1e-12, '极点处两纬度相等（差 %.1e°）' % abs(d_pole))
check(abs(worst[0]) > 0.15, '最大差 %.4f° > 0.15°（在纬度 45° 附近）' % abs(worst[0]))
check(18.0 < worst[1] < 25.0, '最大地面偏差 %.1f km（约 21 km）' % worst[1])

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
