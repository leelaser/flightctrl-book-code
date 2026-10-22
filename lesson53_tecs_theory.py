# -*- coding: utf-8 -*-
"""卷 3 第 9 章 · TECS 总能量控制 —— 可执行验证

配套课文：`卷 3 第 9 章`
源码：`ardupilot-Plane-4.7.2/libraries/AP_TECS/`、
      `PX4-Autopilot-1.15.4/src/lib/tecs/`

验证十件事：

  [1] ★★★ 能量坐标的精确恒等式：d(STE)/dt = V(Tcosα−D)/m（重力项抵消）
  [2] ★★ 解耦矩阵是**下三角**的，不是对角的；它可逆，但油门仍影响 SEB
  [3] ★ 模型基增益 K_STE2Thr = 1/(τ·K_thr2STE)
  [4] ★ 权重双 cap 到 1 的真实作用（限制单权重最大值）
  [5] ★ AP 单斜率 vs PX4 双斜率油门映射：何时等价、何时分叉
  [6] ★ 转弯补偿：AP 的 1/cos²φ−1 vs PX4 的 1/cosφ−1
  [7] ★ 失速保护：AP 的二元滞环 vs PX4 的连续 ramp
  [8] ★ 拉平：高度/下沉率的运动学一致性，以及混合项的残差
  [9] 源码行锚定（有源码树时；缺树则跳过，不崩）
  [10] 参数默认值（有源码树时从源码读取并断言；缺树则跳过）

设计原则（本仓库的教训，见 memory）：
  · 断言两边来源必须不同：解析推导 / 数值仿真 / 源码常量
  · 反向对照写进脚本
  · 阈值来自恒等式或解析预言，不拍脑袋

运行：`python lesson53_tecs_theory.py` —— 全部断言通过才返回 0
"""
from __future__ import print_function

import io
import math
import os
import re
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


G = 9.80665

print('    版本基准：ArduPilot Plane 4.7.2；PX4 v1.15.4')

print('=' * 74)
print('[1] ★★★ 能量坐标的精确恒等式')
print('=' * 74)
print('    沿航迹方程： m·dV/dt = T·cosα − D − m·g·sinγ')
print('    垂直方程：   dh/dt   = V·sinγ')
print()
print('    比势能 SPE = g·h，比动能 SKE = ½V²')
print('    比总能量 STE = SPE + SKE = g·h + ½V²')
print('    比能量分配 SEB = SPE − SKE = g·h − ½V²')
print()


def dEdt_raw(T, D, m, V, gamma):
    """按【原始两方程】算 dE/dt —— 不代入化简结果，保留 γ 项。"""
    dV = (T - D - m * G * math.sin(gamma)) / m
    dh = V * math.sin(gamma)
    return G * dh + V * dV


def dBdt_raw(T, D, m, V, gamma):
    """按【原始两方程】算 dB/dt —— 保留 γ 项。"""
    dV = (T - D - m * G * math.sin(gamma)) / m
    dh = V * math.sin(gamma)
    return G * dh - V * dV


m_ = 2.0
T_, D_, V_ = 8.0, 3.0, 18.0
dE_closed = V_ * (T_ - D_) / m_
print('    数值验证（T=%.1f N, D=%.1f N, m=%.1f kg, V=%.1f m/s）：' % (T_, D_, m_, V_))
print('    d(STE)/dt 的化简值 V(T−D)/m = %.10f' % dE_closed)
print()
print('    %-10s %20s %20s %14s' % ('γ (度)', '原始式', '化简式', '相对差'))
worst_e = 0.0
for gdeg in (-30, -15, 0, 10, 25, 45):
    gam = math.radians(gdeg)
    a = dEdt_raw(T_, D_, m_, V_, gam)
    rel = abs(a - dE_closed) / abs(dE_closed)
    worst_e = max(worst_e, rel)
    print('    %-10g %20.10f %20.10f %13.2e' % (gdeg, a, dE_closed, rel))
check(worst_e < 1e-12,
      '★ d(STE)/dt 在 γ 从 −30° 到 +45° 全程【恒等于】V(T−D)/m'
      '（最大相对差 %.1e）' % worst_e)
check(abs(dEdt_raw(T_, D_, m_, V_, 0.0) -
          dEdt_raw(T_, D_, m_, V_, math.radians(45))) < 1e-9,
      '★ 反向对照：γ=0 与 γ=45° 的 d(STE)/dt 相同 ⇒ 重力项【精确抵消】，不是近似')

print()
print('    SEB 的导数：d(SEB)/dt = 2gV·sinγ − V(T−D)/m')
print('    %-10s %20s %20s %14s' % ('γ (度)', '原始式', '化简式', '相对差'))
worst_b = 0.0
for gdeg in (-30, -15, 0, 10, 25, 45):
    gam = math.radians(gdeg)
    a = dBdt_raw(T_, D_, m_, V_, gam)
    b = 2 * G * V_ * math.sin(gam) - V_ * (T_ - D_) / m_
    rel = abs(a - b) / max(abs(b), 1e-12)
    worst_b = max(worst_b, rel)
    print('    %-10g %20.10f %20.10f %13.2e' % (gdeg, a, b, rel))
check(worst_b < 1e-12,
      '★ d(SEB)/dt 与 2gV·sinγ − V(T−D)/m 恒等（最大相对差 %.1e）' % worst_b)
# 打印一个正文可能引用的例子
gam10 = math.radians(10.0)
print()
print('    γ=10° 时：d(STE)/dt = %.6f，d(SEB)/dt = %.6f'
      % (dEdt_raw(T_, D_, m_, V_, gam10), dBdt_raw(T_, D_, m_, V_, gam10)))

print()
print('=' * 74)
print('[2] ★★ 解耦矩阵：下三角、可逆，但油门仍影响 SEB')
print('=' * 74)
print('    [d(STE)/dt]   [ V/m      0 ] [Tcosα−D]')
print('    [d(SEB)/dt] = [−V/m    2gV ] [ sinγ   ]')
print()
A00 = V_ / m_
A10 = -V_ / m_
A11 = 2 * G * V_
det = A00 * A11 - 0.0 * A10
print('    V/m = %.6f，−V/m = %.6f，2gV = %.6f' % (A00, A10, A11))
print('    det(A) = %.6f = 2gV²/m' % det)
check(det > 0, '★ det(A) > 0 ⇒ 矩阵可逆（V>0,m>0,g>0）')

# 逆式：u=Tcosα−D, s=sinγ
Edot, Bdot = 1.2, -0.7
u = (m_ / V_) * Edot
s = (Edot + Bdot) / (2 * G * V_)
Edot_f = A00 * u + 0.0 * s
Bdot_f = A10 * u + A11 * s
print()
print('    给定 (dSTE/dt, dSEB/dt) = (%.1f, %.1f)：' % (Edot, Bdot))
print('      Tcosα−D = (m/V)dSTE/dt = %.10f' % u)
print('      sinγ    = (dSEB/dt+dSTE/dt)/(2gV) = %.12f' % s)
print('      前向残差：dSTE %.2e，dSEB %.2e'
      % (abs(Edot_f - Edot), abs(Bdot_f - Bdot)))
check(abs(Edot_f - Edot) < 1e-12 and abs(Bdot_f - Bdot) < 1e-12,
      '★ 逆式代回原矩阵，两个分量都精确复原（残差 < 1e-12）')

# 三角性：纯俯仰不改变 STE；纯油门同时改变两者
E_pitch = A00 * 0.0 + 0.0 * 0.1
B_pitch = A10 * 0.0 + A11 * 0.1
E_thr = A00 * 1.0 + 0.0 * 0.0
B_thr = A10 * 1.0 + A11 * 0.0
print()
print('    纯俯仰（sinγ=0.1，T−D=0）：dSTE=%.2f，dSEB=%.6f' % (E_pitch, B_pitch))
print('    纯油门（T−D=1，sinγ=0）：dSTE=%.6f，dSEB=%.6f' % (E_thr, B_thr))
check(abs(E_pitch) < 1e-12,
      '★ 纯俯仰对 d(STE)/dt 的贡献为 0 ⇒ 重力项抵消')
check(abs(B_pitch) > 1.0,
      '★ 纯俯仰对 d(SEB)/dt 的贡献非零（%.3f）⇒ 俯仰管分配' % B_pitch)
check(abs(B_thr) > 1e-9,
      '★ 纯油门对 d(SEB)/dt 的贡献也非零（%.3f）⇒ 矩阵【不是对角】的' % B_thr)
coupling = abs(A10) / A11
print('    耦合比 |−V/m| / (2gV) = 1/(2gm) = %.6f' % coupling)
check(coupling < 1.0,
      '★ 耦合比 %.4f < 1 ⇒ 油门对 SEB 的影响远小于俯仰；'
      '所以“先总能量、后分配”的顺序设计有效' % coupling)

print()
print('=' * 74)
print('[3] ★ 模型基增益：K_STE2Thr = 1/(τ·K_thr2STE)')
print('=' * 74)
STEdot_max, STEdot_min = 4.0, -3.0
THRmax, THRmin = 1.0, 0.1
K_thr2STE = (STEdot_max - STEdot_min) / (THRmax - THRmin)
print('    K_thr2STE = (%.1f − (%.1f)) / (%.1f − %.1f) = %.6f'
      % (STEdot_max, STEdot_min, THRmax, THRmin, K_thr2STE))
print('    量纲：(m²/s³)/油门 ⇒ 每单位油门能产生多少能量率')
print()
print('    %-10s %20s %20s' % ('τ (s)', 'K_STE2Thr', '相对 τ=5'))
K5 = 1.0 / (5.0 * K_thr2STE)
for tau in (2.5, 5.0, 10.0):
    K = 1.0 / (tau * K_thr2STE)
    print('    %-10g %20.8f %20.4f' % (tau, K, K / K5))
    check(abs(K * tau * K_thr2STE - 1.0) < 1e-12,
          'τ=%-4g：K_STE2Thr × τ × K_thr2STE ≡ 1' % tau)
check(abs(1.0 / (2.5 * K_thr2STE) - 2.0 * K5) < 1e-12,
      '★ τ 减半 ⇒ 增益精确翻倍；τ 翻倍 ⇒ 增益精确减半')
# 一个能量误差对应的油门修正
err = 1.0
print()
print('    例：STE 误差 = %.1f m²/s²，τ=5 s ⇒ 油门比例修正 = %.8f'
      % (err, err * K5))
print('    例：STEdot 误差 = %.1f m²/s³，damp=0.5 ⇒ 油门阻尼修正 = %.8f'
      % (err, err * 0.5 * K5))
check(abs(err * K5 * 5.0 * K_thr2STE - err) < 1e-12,
      '★ 能量误差经 1/(τK) 再乘 K，按时间常数 τ 衰减 —— 这就是“调 τ 而不是调 P”')

print()
print('=' * 74)
print('[4] ★ 权重双 cap 到 1：限制的是【单个权重的最大值】')
print('=' * 74)
print('    源码：w_P = 2 − w_K；w_P = min(w_P,1)；w_K = min(w_K,1)')
SPE_d, SKE_d = 100.0, 50.0
print()
print('    %-8s %-16s %-12s %-16s %-12s'
      % ('w_K', 'cap后(SPE,SKE)', 'SEB(cap)', '不cap(SPE,SKE)', 'SEB(不cap)'))
rows = []
for w in (0.0, 0.5, 1.0, 1.5, 2.0):
    sp_c, sk_c = min(2.0 - w, 1.0), min(w, 1.0)
    sp_r, sk_r = 2.0 - w, w
    seb_c = SPE_d * sp_c - SKE_d * sk_c
    seb_r = SPE_d * sp_r - SKE_d * sk_r
    rows.append((w, sp_c, sk_c, seb_c, sp_r, sk_r, seb_r))
    print('    %-8g (%4.2f, %4.2f)     %12.2f (%4.2f, %4.2f)     %12.2f'
          % (w, sp_c, sk_c, seb_c, sp_r, sk_r, seb_r))
max_c = max(max(r[1], r[2]) for r in rows)
max_r = max(max(r[4], r[5]) for r in rows)
print()
print('    cap 后最大单权重 = %.2f；不 cap 最大单权重 = %.2f' % (max_c, max_r))
check(abs(max_c - 1.0) < 1e-12,
      '★ cap 后【单个权重最大值】恒为 %.2f ⇒ 等效增益不超过 w=1 时标定值' % max_c)
check(abs(max_r - 2.0) < 1e-12,
      '★ 反向对照：不 cap 时端点权重涨到 %.2f ⇒ 等效增益翻倍' % max_r)
# w_K=2 时有效权重是 (0,1)，不是 (2,2)
w2 = rows[-1]
check(abs(w2[1] - 0.0) < 1e-12 and abs(w2[2] - 1.0) < 1e-12,
      '★ w_K=2 时有效权重是 (w_P,w_K)=(%.1f,%.1f)，不是 (2,2)'
      % (w2[1], w2[2]))
check(abs(w2[6] - 2.0 * w2[3]) < 1e-12,
      '★ 不 cap 的 SEB 需求 %.1f 是 cap 后 %.1f 的两倍 ⇒ cap 直接压掉一倍增益'
      % (w2[6], w2[3]))

print()
print('=' * 74)
print('[5] ★ 油门映射：AP 单斜率 vs PX4 双斜率')
print('=' * 74)
THRmin, THRmax = 0.1, 1.0
trim = 0.55
Emax, Emin = 5.0, -2.0
K = (Emax - Emin) / (THRmax - THRmin)
print('    THRmin=%.2f, THRmax=%.2f, trim=%.2f, Emax=%.2f, Emin=%.2f'
      % (THRmin, THRmax, trim, Emax, Emin))
print('    K_thr2STE=%.6f' % K)


def ap_thr(e, trim_):
    """AP 的 feedforward：以 trim 为零点、单一斜率；最后饱和。"""
    return min(max(trim_ + e / K, THRmin), THRmax)


def px4_thr(e, trim_):
    """PX4 的 feedforward：trim 上下两条斜率，端点锚在限幅上。"""
    above = (THRmax - trim_) / Emax
    below = (trim_ - THRmin) / Emin
    if e >= 0:
        v = trim_ + e * above
    else:
        v = trim_ - e * below
    return min(max(v, THRmin), THRmax)


print()
print('    %-12s %16s %16s' % ('STE 率需求', 'AP 油门', 'PX4 油门'))
for e in (Emin, 0.0, Emax):
    print('    %-12.2f %16.6f %16.6f' % (e, ap_thr(e, trim), px4_thr(e, trim)))
check(abs(ap_thr(Emax, trim) - 1.0) < 1e-12 and
      abs(px4_thr(Emax, trim) - 1.0) < 1e-12,
      '★ 上端：两者都到满油门（AP 被限幅截住）')
check(abs(px4_thr(Emin, trim) - THRmin) < 1e-12,
      '★ PX4 下端精确到 %.2f（端点锚定）' % THRmin)
check(ap_thr(Emin, trim) > THRmin + 0.1,
      '★ AP 下端只到 %.6f，没到 %.2f ⇒ 单斜率在不对称 trim 下【到不了端点】'
      % (ap_thr(Emin, trim), THRmin))
# 等价条件：trim 取让单斜率同时穿过两个端点的值
trim_eq = THRmax - Emax / K
print()
print('    等价条件：trim = THRmax − Emax/K = %.6f' % trim_eq)
print('    %-12s %16s %16s' % ('STE 率需求', 'AP 油门', 'PX4 油门'))
for e in (Emin, 0.0, Emax):
    print('    %-12.2f %16.6f %16.6f'
          % (e, ap_thr(e, trim_eq), px4_thr(e, trim_eq)))
check(abs(ap_thr(Emin, trim_eq) - THRmin) < 1e-12 and
      abs(ap_thr(Emax, trim_eq) - THRmax) < 1e-12,
      '★ trim 取 %.6f 时，AP 单斜率同时命中两端 ⇒ 此时与 PX4 映射等价' % trim_eq)
check(abs(trim - trim_eq) > 0.1,
      '★ 本机 trim=%.2f 与等价值 %.4f 相差 %.4f ⇒ 两栈的 feedforward 会分叉'
      % (trim, trim_eq, abs(trim - trim_eq)))

print()
print('=' * 74)
print('[6] ★ 转弯补偿：AP 的 1/cos²φ−1 vs PX4 的 1/cosφ−1')
print('=' * 74)
print('    诱导阻力 ∝ C_L²；平飞盘旋 C_L = C_L,level/cosφ')
print('    ⇒ 额外诱导阻力 ∝ (1/cos²φ − 1) = tan²φ')
print('    PX4 假设“诱导阻力随载荷因子线性增长”，用 (n−1) = (1/cosφ − 1)')
print()
print('    %-8s %14s %14s %14s %10s'
      % ('φ (度)', 'AP: tan²φ', 'PX4: 1/cosφ−1', 'AP/PX4', 'cosφ'))
worst_ratio = 0.0
for phideg in (15, 30, 45, 60):
    phi = math.radians(phideg)
    ap = math.tan(phi) ** 2
    px = 1.0 / math.cos(phi) - 1.0
    ratio = ap / px
    worst_ratio = max(worst_ratio, ratio)
    print('    %-8g %14.6f %14.6f %14.4f %10.6f'
          % (phideg, ap, px, ratio, math.cos(phi)))
check(abs(math.tan(math.radians(30)) ** 2 - (1 / math.cos(math.radians(30)) ** 2 - 1)) < 1e-12,
      '★ tan²φ ≡ 1/cos²φ − 1（同一个式子的两种写法）')
check(worst_ratio > 1.0,
      '★ 在 15°–60° 内 AP 的补偿因子【始终大于】PX4 的线性近似'
      '（最大比值 %.4f）' % worst_ratio)
check(abs((1 / math.cos(math.radians(45)) ** 2 - 1) - 1.0) < 1e-12,
      '★ 45° 时 AP 因子精确为 1.0；PX4 为 %.6f ⇒ 线性近似只给到约四成'
      % (1 / math.cos(math.radians(45)) - 1))

print()
print('=' * 74)
print('[7] ★ 失速保护：AP 的二元滞环 vs PX4 的连续 ramp')
print('=' * 74)
Vmin = 12.0
ap_entry = 0.9 * Vmin
ap_exit = 1.15 * Vmin
ap_hold = 3.0
print('    AP：V < %.1f·Vmin = %.1f 且油门 ≥ 95%% 时进入；'
      % (0.9, ap_entry))
print('         V ≥ %.2f·Vmin = %.1f 且持续 %.0f s 才退出'
      % (1.15, ap_exit, ap_hold))
check(abs(ap_entry - 10.8) < 1e-12, 'AP 进入阈值 = %.1f m/s' % ap_entry)
check(abs(ap_exit - 13.8) < 1e-12, 'AP 退出阈值 = %.1f m/s' % ap_exit)


def ap_underspeed_profile():
    """下降再上升的 V 剖面；返回 (V, 状态, 计时) 采样。"""
    v = 14.0
    state = False
    since_below = 0.0
    dt = 0.1
    out = []
    for step in range(120):
        if step < 50:
            v -= 0.1                 # 14 → 9
        else:
            v += 0.2                 # 9 → 15（上升更快，便于看滞环）
        if v < ap_entry:
            since_below = 0.0
        else:
            since_below += dt
        if state:
            if v >= ap_exit and since_below >= ap_hold:
                state = False
        else:
            if v < ap_entry and 1.0 >= 0.95:   # 油门已满
                state = True
        out.append((v, state, since_below))
    return out


prof = ap_underspeed_profile()
# 打印几个关键时刻
print()
print('    %-10s %-10s %-12s' % ('V (m/s)', 'underspeed', '计时 (s)'))
for v, st, t in prof:
    if abs(v - 10.4) < 0.05 or abs(v - 10.8) < 0.05 or abs(v - 13.8) < 0.05:
        print('    %-10.2f %-10s %-12.2f' % (v, st, t))
# 断言：低空速段已进入；达到退出阈值后保持 3 s 才退出
low = [(v, st) for v, st, t in prof if v < ap_entry]
high = [(v, st, t) for v, st, t in prof if v >= ap_exit]
check(all(st for v, st in low),
      '★ AP 在 V < %.1f 的低速段全部保持 underspeed' % ap_entry)
check(any(not st for v, st, t in high if t >= ap_hold),
      '★ AP 达到 %.1f 后还要等计时 ≥ %.0f s 才退出（滞环）'
      % (ap_exit, ap_hold))
check(any(st for v, st, t in high if t < ap_hold),
      '★ 刚越过 %.1f、计时不足 %.0f s 时仍保持 underspeed'
      % (ap_exit, ap_hold))

# PX4 连续 ramp
tas_min = 10.0
eas_trim = 15.0
delta = 0.15 * eas_trim
v_full = max(tas_min - 2 * delta, 0.0)
v_start = max(tas_min - delta, v_full)
print()
print('    PX4：Δ = 0.15·EAS_trim = %.2f m/s' % delta)
print('         V_full = %.2f，V_start = %.2f' % (v_full, v_start))


def px4_ramp(v):
    return 1.0 - min(max((v - v_full) / max(v_start - v_full, 1e-9), 0.0), 1.0)


print('    %-10s %-12s' % ('V (m/s)', 'r_under'))
for v in (5.0, 5.5, 6.0, 7.0, 7.75, 8.0, 10.0):
    print('    %-10.2f %-12.6f' % (v, px4_ramp(v)))
check(abs(px4_ramp(v_full) - 1.0) < 1e-12,
      '★ PX4 在 V_full=%.2f 处 r=1（完全 underspeed）' % v_full)
check(abs(px4_ramp(v_start)) < 1e-12,
      '★ PX4 在 V_start=%.2f 处 r=0（开始退出）' % v_start)
check(px4_ramp(6.0) > px4_ramp(7.0) > px4_ramp(7.5),
      '★ PX4 的 r 随速度单调下降 ⇒ 连续 ramp，没有二元跳变')

print()
print('=' * 74)
print('[8] ★ 拉平：高度/下沉率的运动学一致性')
print('=' * 74)
H0, H_HOLD = 15.0, 2.0
HDOT_ENTRY, LAND_SINK = -2.5, 0.25
DT = 0.01
h = H0
h_ideal = H0
h_adj = H0 + 1.5          # 预拉平跟踪偏置（AP 的 _flare_hgt_dem_adj 高于理想剖面）
worst_ideal = 0.0
extra_max = 0.0
p_first = None
p_last = 0.0
for step in range(20000):
    p = min(max((H0 - h) / (H0 - H_HOLD), 0.0), 1.0)
    if p_first is None:
        p_first = p
    hdot_ref = HDOT_ENTRY * (1.0 - p) - LAND_SINK * p
    h_ideal_new = h_ideal + DT * hdot_ref
    h_adj_new = h_adj + DT * hdot_ref
    h_d = (1.0 - p) * h_adj + p * h_ideal
    h_next = h + DT * hdot_ref
    p_next = min(max((H0 - h_next) / (H0 - H_HOLD), 0.0), 1.0)
    h_d_new = (1.0 - p_next) * h_adj_new + p_next * h_ideal_new
    num_ideal = (h_ideal_new - h_ideal) / DT
    num_blend = (h_d_new - h_d) / DT
    p_dot = (p_next - p) / DT
    worst_ideal = max(worst_ideal, abs(num_ideal - hdot_ref))
    extra_max = max(extra_max, abs(num_blend - hdot_ref))
    h = h_next
    h_ideal = h_ideal_new
    h_adj = h_adj_new
    p_last = p_next
    if h <= H_HOLD:
        break
print('    H0=%.1f m，hold=%.1f m；hdot_entry=%.2f m/s，land_sink=%.2f m/s'
      % (H0, H_HOLD, HDOT_ENTRY, LAND_SINK))
print('    p 起点=%.4f，p 终点=%.4f；h_ideal 与 h_d 用同一 hdot_ref 积分'
      % (p_first, p_last))
print('    |dh_ideal/dt − hdot_ref| 最大值 = %.2e' % worst_ideal)
print('    |dh_d/dt − hdot_ref| 最大值      = %.4f' % extra_max)
check(worst_ideal < 1e-12,
      '★ h_ideal 的数值导数【精确等于】hdot_ref（最大偏差 %.1e）'
      '⇒ 理想高度剖面与下沉率剖面运动学自洽' % worst_ideal)
check(extra_max > 1e-3,
      '★ 但混合高度 h_d 的导数【不等于】hdot_ref（最大偏差 %.4f）'
      '⇒ p 变化时多出 p_dot·(h_ideal−h_adj) 一项' % extra_max)
check(abs(HDOT_ENTRY - LAND_SINK) > 2.0,
      '★ 反向对照：进入拉平的下沉率 %.1f 与目标 %.2f 相差 %.1f m/s；'
      '若不用 p 淡入，就会阶跃' % (HDOT_ENTRY, LAND_SINK,
                                  abs(HDOT_ENTRY - LAND_SINK)))

print()
print('=' * 74)
print('[9] 源码行锚定（可选）')
print('=' * 74)
try:
    import _srcdirs
    AP = _srcdirs.src('APM_472_PLANE')
    PX4 = _srcdirs.src('PX4')
except Exception as exc:
    AP = PX4 = ''
    print('    跳过：无法导入 _srcdirs（%s）' % exc)

ANCHORS = [
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 722, 'SPE_err_max'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 738, '_STE_error'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 756, 'K_thr2STE'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 757, 'K_STE2Thr'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 764, '_rollComp'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 1015, 'SPE_weighting'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 1018, 'MIN(SPE_weighting'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 1027, 'SEBdot_dem'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 1056, 'gainInv'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 1099, '_pitch_dem_unc'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 647, '_detect_underspeed'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 610, 'flaring'),
    ('AP', 'libraries/AP_TECS/AP_TECS.cpp', 1245, '_update_STE_rate_lim'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 307, 'min_sink_rate'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 344, '_calcSpecificEnergyRates'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 401, 'spe_weighting'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 444, 'seb_rate.setpoint'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 460, 'climb_angle_to_SEB_rate'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 495, 'SEB_rate_correction'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 548, 'load_factor - 1.f'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 560, 'STE_rate_to_throttle'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 603, 'throttle_above_trim_per_ste_rate'),
    ('PX4', 'src/lib/tecs/TECS.cpp', 634, '_ratio_undersped'),
    ('PX4', 'src/modules/fw_pos_control/FixedwingPositionControl.cpp',
     2657, 'do not run TECS'),
    ('PX4', 'src/modules/fw_pos_control/FixedwingPositionControl.cpp',
     2674, 'HOTFIX'),
    ('PX4', 'src/modules/fw_pos_control/FixedwingPositionControl.cpp',
     3158, 'load_factor_from_bank_angle'),
]


def read_line(root, rel, n):
    p = os.path.join(root, rel.replace('/', os.sep))
    if not os.path.isfile(p):
        return None
    lines = io.open(p, encoding='utf-8', errors='replace').read().split('\n')
    if n < 1 or n > len(lines):
        return None
    return lines[n - 1]


if AP and PX4 and os.path.isdir(AP) and os.path.isdir(PX4):
    for tag, rel, n, token in ANCHORS:
        root = AP if tag == 'AP' else PX4
        line = read_line(root, rel, n)
        if line is None:
            check(False, '%s:%d 读不到' % (rel, n))
        else:
            check(token in line,
                  '%s:%d 含 `%s`' % (rel, n, token))
else:
    print('    跳过：源码树不在本机（AP=%r，PX4=%r）' % (AP, PX4))

print()
print('=' * 74)
print('[10] 参数默认值（可选）')
print('=' * 74)
AP_DEFAULTS = {
    'CLMB_MAX': 5.0,
    'SINK_MIN': 2.0,
    'TIME_CONST': 5.0,
    'THR_DAMP': 0.5,
    'INTEG_GAIN': 0.3,
    'VERT_ACC': 7.0,
    'RLL2THR': 10.0,
    'SPDWEIGHT': 1.0,
    'PTCH_DAMP': 0.3,
    'SINK_MAX': 5.0,
}
PX4_DEFAULTS = {
    'FW_T_ALT_TC': 5.0,
    'FW_T_TAS_TC': 5.0,
    'FW_T_STE_R_TC': 0.4,
    'FW_T_PTCH_DAMP': 0.1,
    'FW_T_THR_DAMPING': 0.05,
    'FW_T_THR_INTEG': 0.02,
    'FW_T_I_GAIN_PIT': 0.1,
    'FW_T_RLL2THR': 15.0,
    'FW_T_SPDWEIGHT': 1.0,
    'FW_T_SINK_MAX': 5.0,
}


def parse_ap_defaults(path):
    out = {}
    if not os.path.isfile(path):
        return out
    text = io.open(path, encoding='utf-8', errors='replace').read()
    for m in re.finditer(
            r'AP_GROUPINFO\("([A-Z0-9_]+)"\s*,\s*\d+\s*,\s*AP_TECS\s*,\s*'
            r'[^,]+,\s*([-+]?\d+(?:\.\d+)?)f?\s*\)', text):
        out[m.group(1)] = float(m.group(2))
    return out


def parse_px4_defaults(path):
    out = {}
    if not os.path.isfile(path):
        return out
    text = io.open(path, encoding='utf-8', errors='replace').read()
    for m in re.finditer(
            r'PARAM_DEFINE_FLOAT\(\s*([A-Z0-9_]+)\s*,\s*'
            r'([-+]?\d+(?:\.\d+)?)f?\s*\)', text):
        out[m.group(1)] = float(m.group(2))
    return out


ap_par = parse_ap_defaults(os.path.join(
    AP, 'libraries', 'AP_TECS', 'AP_TECS.cpp')) if AP else {}
px4_par = parse_px4_defaults(os.path.join(
    PX4, 'src', 'modules', 'fw_pos_control', 'fw_path_navigation_params.c')) if PX4 else {}

if ap_par:
    print('    AP 参数（从 AP_GROUPINFO 读取）：')
    for k in sorted(AP_DEFAULTS):
        got = ap_par.get(k)
        print('      TECS_%-12s = %s' % (k, '缺失' if got is None else ('%g' % got)))
        check(got is not None and abs(got - AP_DEFAULTS[k]) < 1e-9,
              'TECS_%s 默认值 = %g' % (k, AP_DEFAULTS[k]))
else:
    print('    跳过：AP 源码树不在本机或无 AP_GROUPINFO')

if px4_par:
    print('    PX4 参数（从 PARAM_DEFINE_FLOAT 读取）：')
    for k in sorted(PX4_DEFAULTS):
        got = px4_par.get(k)
        print('      %-20s = %s' % (k, '缺失' if got is None else ('%g' % got)))
        check(got is not None and abs(got - PX4_DEFAULTS[k]) < 1e-9,
              '%s 默认值 = %g' % (k, PX4_DEFAULTS[k]))
else:
    print('    跳过：PX4 源码树不在本机或无 PARAM_DEFINE_FLOAT')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
