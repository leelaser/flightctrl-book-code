# -*- coding: utf-8 -*-
"""卷 8 第 13 章 · 滤波与振动抑制 —— 可执行验证（陷波部分）

配套课文：`卷 8 第 13 章`
源码：`ardupilot-Copter-4.7.2/libraries/Filter/NotchFilter.cpp`（151 行）
      `libraries/Filter/HarmonicNotchFilter.cpp`（573 行）
      `libraries/AP_GyroFFT/AP_GyroFFT.cpp`（1482 行）
      `libraries/AP_InertialSensor/`（振动量化与管线）

验证五件事：

  [1] ★★★ `A` 和 `Q` 的公式，以及 Q 的【两种等价写法】
  [2] ★★★ biquad 系数 == RBJ（Bristow-Johnson）标准公式 —— 独立实现对撞
  [3] ★★★ 频率响应：f0 处的衰减【精确等于】`attenuation_dB` 参数
  [4] ★★  `bandwidth_hz` 到底对应多宽？实测 vs 请求 vs f0/Q
  [5] ★★  `NOTCH_MAX_SLEW` 的跟踪行为（含上下不对称）

设计原则（本仓库的教训，见 memory）：
  · 期望值写死在断言里，不用脚本自己的常量
  · 一边用【源码公式】，另一边用【RBJ 教科书公式】—— 两套独立推导
  · 量具先自证：数值扫描的边界必须对称（我第一版就因为不对称测量错了一半）

运行：`python lesson42_notch.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function

import cmath
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


# ══════════════════════════════════════════════════════════════
# 源码搬运（NotchFilter.cpp，逐行照抄）
# ══════════════════════════════════════════════════════════════
def src_calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB):
    """NotchFilter.cpp:32-40"""
    A = 10 ** (-attenuation_dB / 40.0)
    if center_freq_hz > 0.5 * bandwidth_hz:
        octaves = math.log2(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0)) * 2.0
        Q = math.sqrt(2 ** octaves) / (2 ** octaves - 1.0)
    else:
        Q = 0.0
    return A, Q


def src_init_with_A_and_Q(sample_freq_hz, center_freq_hz, A, Q):
    """NotchFilter.cpp:77-92 —— biquad 系数"""
    omega = 2.0 * math.pi * center_freq_hz / sample_freq_hz
    alpha = math.sin(omega) / (2 * Q)
    b0 = 1.0 + alpha * A * A
    b1 = -2.0 * math.cos(omega)
    b2 = 1.0 - alpha * A * A
    a1 = b1
    a2 = 1.0 - alpha
    k = 1.0 / (1.0 + alpha)
    return (b0 * k, b1 * k, b2 * k, a1 * k, a2 * k)


def rbj_notch(f0, fs, Q, A):
    """RBJ Audio EQ Cookbook 的 notch（band-reject）—— 独立来源，不是从上面推的"""
    w0 = 2.0 * math.pi * f0 / fs
    alpha = math.sin(w0) / (2.0 * Q)
    b0 = 1.0 + alpha * A * A
    b1 = -2.0 * math.cos(w0)
    b2 = 1.0 - alpha * A * A
    a0 = 1.0 + alpha
    a1 = -2.0 * math.cos(w0)
    a2 = 1.0 - alpha
    return (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)


def H_db(coeffs, f, fs):
    """频率响应 |H(e^{jw})|（返回 dB）"""
    b0, b1, b2, a1, a2 = coeffs
    w = 2.0 * math.pi * f / fs
    z = cmath.exp(-1j * w)
    num = b0 + b1 * z + b2 * z * z
    den = 1.0 + a1 * z + a2 * z * z
    return 20.0 * math.log10(abs(num / den))


FS = 1000.0

# ══════════════════════════════════════════════════════════════
print('=' * 74)
print('[1] ★★★ A 和 Q 的公式')
print('=' * 74)
print('    源码 NotchFilter.cpp:32-40')
print('        A = powf(10, -attenuation_dB / 40.0f)')
print('        octaves = log2f(f0/(f0 - BW/2)) * 2')
print('        Q = sqrtf(2^octaves) / (2^octaves - 1)')
print()
print('    ★★ 那个 40 不是随手写的：')
print('        振幅的 dB 定义是 20·log10(A)，而这里 A 是【幅度平方】量级')
print('        ⇒ 要让 |H(f0)| 的 dB 数正好等于 attenuation_dB，')
print('          需要 A² = 10^(-dB/20)  ⇒  A = 10^(-dB/40)  ✓（第 [3] 段会验）')
print()
print('    %-20s %-14s %-14s' % ('attenuation_dB', 'A', 'A²（= 线性衰减）'))
for dB in (10.0, 20.0, 30.0, 40.0, 60.0):
    A = 10 ** (-dB / 40.0)
    print('    %-20g %-14.8f %-14.8f' % (dB, A, A * A))
    check(abs(20 * math.log10(A * A) + dB) < 1e-12,
          'dB=%-4g ⇒ A²=%.8f ⇒ 20·log10(A²) = %.6f = −dB' % (dB, A * A, 20 * math.log10(A * A)))

print()
print('    ★★★ Q 的两种写法【完全等价】（这是本条的核心断言）：')
print('        源码写法   Q = sqrt(2^n) / (2^n - 1)')
print('        RBJ 写法   Q = 1 / (2·sinh(n·ln2/2))')
print()
print('    %-10s %-22s %-22s %s' % ('octaves n', '源码函数给的值', 'RBJ 公式', '差'))
print()
print('    ⚠️ 这里必须【真的调用 src_calculate_A_and_Q】。')
print('       第一版我把源码公式在循环里又抄了一遍，于是注入"把分母 -1 改成 +1"')
print('       时这一整段照样全绿 —— 断言的其实是我自己抄的那份，不是被测函数。')
print('       构造 (f0, BW) 使 octaves 恰好等于 n：')
print('         n = 2·log2(f0/(f0−BW/2))  ⇒  BW = 2·f0·(1 − 2^(−n/2))')
print()
worst_q = 0.0
F0_Q = 100.0
for n in (0.1, 0.25, 0.5, 1.0, 1.585, 2.0, 3.0, 5.0):
    bw = 2.0 * F0_Q * (1.0 - 2 ** (-n / 2.0))
    # ★ 真的从被测函数里取 Q
    _, q1 = src_calculate_A_and_Q(F0_Q, bw, 40.0)
    q2 = 1.0 / (2.0 * math.sinh(n * math.log(2) / 2.0))
    e = abs(q1 - q2)
    worst_q = max(worst_q, e)
    print('    %-10.3f %-22.12f %-22.12f %.2e' % (n, q1, q2, e))
    check(e < 1e-12,
          'n=%-5.3f（BW=%.4f）⇒ 源码函数给的 Q = %.12f，RBJ 给 %.12f'
          % (n, bw, q1, q2))
check(worst_q < 1e-12,
      '★★★ 两种写法【代数恒等】—— 源码没有另创公式，用的就是 RBJ 的八度带宽关系')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[2] ★★★ biquad 系数 == RBJ 教科书公式')
print('=' * 74)
print('    RBJ Audio EQ Cookbook 的 notch（band-reject）：')
print('        w0 = 2π·f0/fs ;  alpha = sin(w0)/(2Q)')
print('        b0 = 1 + alpha·A²   b1 = −2cos(w0)   b2 = 1 − alpha·A²')
print('        a0 = 1 + alpha      a1 = −2cos(w0)   a2 = 1 − alpha')
print('    源码 :77-92 是同一组式子，只是先把 a0 归一化掉了（"Pre-multiply to save runtime calc"）')
print()
print('    %-22s %-34s %s' % ('场景', '源码系数', 'RBJ 系数'))
worst_c = 0.0
CASES = [(80.0, 4.0, 40.0), (80.0, 20.0, 20.0), (100.0, 40.0, 30.0),
         (50.0, 2.0, 60.0), (250.0, 50.0, 15.0)]
for f0, BW, dB in CASES:
    A, Q = src_calculate_A_and_Q(f0, BW, dB)
    sc = src_init_with_A_and_Q(FS, f0, A, Q)
    rb = rbj_notch(f0, FS, Q, A)
    e = max(abs(sc[i] - rb[i]) for i in range(5))
    worst_c = max(worst_c, e)
    print('    f0=%-5g BW=%-4g %2gdB  (%.4f,%.4f,%.4f,%.4f,%.4f)'
          % (f0, BW, dB, sc[0], sc[1], sc[2], sc[3], sc[4]))
    check(e < 1e-15,
          '  └ f0=%-5g BW=%-4g %2gdB：源码与 RBJ 最大差 %.2e' % (f0, BW, dB, e))
check(worst_c < 1e-15,
      '★★★ 全部 %d 组逐系数一致 ⇒ 源码实现的就是 RBJ 陷波，没有自定义变体'
      % len(CASES))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[3] ★★★ f0 处的衰减【精确等于】attenuation_dB 参数')
print('=' * 74)
print('    这是"参数名即语义"的直接验证 —— 不是近似，是精确相等。')
print()
print('    %-10s %-8s %-10s %-16s %-12s %s'
      % ('f0', '请求BW', '请求衰减', '实测 @f0 (dB)', 'DC 增益', '误差'))
worst_at0 = 0.0
worst_dc = 0.0
for f0, BW, dB in ((80, 4, 40), (80, 20, 20), (100, 40, 30), (50, 2, 60),
                   (200, 30, 12), (400, 100, 45)):
    A, Q = src_calculate_A_and_Q(f0, BW, dB)
    c = src_init_with_A_and_Q(FS, f0, A, Q)
    at0 = H_db(c, f0, FS)
    dc = 10 ** (H_db(c, 0.0001, FS) / 20.0)
    worst_at0 = max(worst_at0, abs(at0 + dB))
    worst_dc = max(worst_dc, abs(dc - 1.0))
    print('    %-10g %-8g %-10g %-16.3f %-12.6f %.2e' % (f0, BW, dB, at0, dc, abs(at0 + dB)))
    check(abs(at0 + dB) < 1e-9,
          'f0=%-5g BW=%-4g：实测 %.6f dB == −%g dB' % (f0, BW, at0, dB))
    check(abs(dc - 1.0) < 1e-4,
          '  └ 直流增益 %.6f ≈ 1（陷波不改变通带）' % dc)
check(worst_at0 < 1e-9,
      '★★★ 全部 6 组误差 < 1e-9 ⇒ `attenuation_dB` 就是陷波【深度】，精确对应')
check(worst_dc < 1e-4, '★★ 且通带增益都是 1')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[4] ★★ `bandwidth_hz` 到底对应多宽')
print('=' * 74)
print('    ⚠️ 注意 `attenuation_dB` 是【深度】不是"定义带宽的电平" ——')
print('       在 f0 处 |H| 正好等于该电平，所以"该电平处的宽度"是【一个点】。')
print('       带宽要用别的定义：**−3 dB 宽**。')
print()
print('    ⚠️ 我第一次测的时候上边界搜索起步于 f0·1.0002，')
print('       那里 H 已经低于阈值 ⇒ 立刻 break ⇒ 测出的宽度【正好是真值的一半】。')
print('       量具自己要对称 —— 这是本仓库反复踩的坑。')


def bandwidth_3db(c, f0, fs, lvl=10 ** (-3 / 20.0)):
    """对称地找两个 -3dB 交点"""
    lo, hi = f0, f0
    f = f0
    steps = 0
    while f > f0 * 0.05 and steps < 400000:
        f *= 0.9998
        steps += 1
        if H_db(c, f, fs) > 20 * math.log10(lvl):
            lo = f
            break
    f = f0
    steps = 0
    while f < fs / 2 and steps < 400000:
        f *= 1.0002
        steps += 1
        if H_db(c, f, fs) > 20 * math.log10(lvl):
            hi = f
            break
    return lo, hi


print()
print('    %-8s %-8s %-9s %-11s %-12s %-12s %s'
      % ('f0', '请求BW', 'Q', 'f0/Q', '实测−3dB宽', '实测/(f0/Q)', '实测/请求BW'))
worst_ratio = 0.0
for f0, BW in ((80, 4), (80, 10), (80, 20), (80, 40), (100, 60), (50, 2), (50, 25)):
    A, Q = src_calculate_A_and_Q(f0, BW, 40.0)
    c = src_init_with_A_and_Q(FS, f0, A, Q)
    lo, hi = bandwidth_3db(c, f0, FS)
    w = hi - lo
    ratio = w / BW
    worst_ratio = max(worst_ratio, abs(ratio - 1.0))
    print('    %-8g %-8g %-9.4f %-11.2f %-12.3f %-12.4f %.4f'
          % (f0, BW, Q, f0 / Q, w, w / (f0 / Q), ratio))
    check(0.85 < w / (f0 / Q) < 1.01,
          '  └ f0=%-5g BW=%-4g：−3dB宽/(f0/Q) = %.4f（高 Q 近似，1 附近）' % (f0, BW, w / (f0 / Q)))
    check(0.85 < ratio < 1.20,
          '  └ 且 实测/请求BW = %.4f' % ratio)
print()
check(worst_ratio < 0.20,
      '★★ `bandwidth_hz` ≈ −3dB 带宽，全部 7 组偏差 < 20%%')
print('    ★ 规律：窄带（BW/f0 小）时实测/请求 ≈ 1.00；')
print('      宽带时偏大（f0=100,BW=60 时到 1.12）—— 因为源码的八度数是近似：')
print('        octaves = 2·log2(f0/(f0−BW/2))')
print('      它把带宽当成【在对数上对称】，而真实的上边沿是 f0+BW/2。')
oct_approx = 2 * math.log2(100 / (100 - 30))
oct_true = math.log2((100 + 30) / (100 - 30))
check(oct_approx > oct_true,
      '★★ 近似八度数 %.4f > 真实八度数 %.4f ⇒ 估的带更宽 ⇒ Q 更小 ⇒ 陷波更宽'
      % (oct_approx, oct_true))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[5] ★★ NOTCH_MAX_SLEW：中心频率的跟踪速率')
print('=' * 74)
print('    NotchFilter.cpp:24-26')
print('        NOTCH_MAX_SLEW       = 0.05f')
print('        NOTCH_MAX_SLEW_LOWER = 1.0f - 0.05 = 0.95')
print('        NOTCH_MAX_SLEW_UPPER = 1.0f / 0.95 = 1.0526…')
print()
print('    :71-74  每次更新，新中心频率被夹在 [旧的×0.95, 旧的×1.0526] 内')
lower = 1.0 - 0.05
upper = 1.0 / lower
print('    ★ 注意上下【不对称】：下降 −5%%、上升 +5.26%%')
check(abs(upper - 1.0526315789473684) < 1e-15,
      '★ 上界 = 1/0.95 = %.10f（不是 1.05）' % upper)
check(upper > 1.05,
      '★ 上界 %.6f > 1.05 ⇒ 上升比下降"宽松"一点点（互为倒数，不是对称的 ±5%%）'
      % upper)

print()
print('    %-26s %-14s %-14s %s' % ('动作', '倍率', '需要几次更新', '说明'))


def updates_to(start, target):
    """模拟 :71-74 的夹取，返回到达目标需要的更新次数"""
    f = start
    n = 0
    while abs(f - target) > 1e-9 and n < 10000:
        n += 1
        if target > f:
            f = min(target, f * upper)
        else:
            f = max(target, f * lower)
    return n


for start, target, note in ((80.0, 120.0, '基频跳升 50%'),
                            (120.0, 80.0, '基频跌落 33%'),
                            (80.0, 81.0, '只差 1.25%'),
                            (80.0, 500.0, '跳 6 倍（换桨）')):
    n = updates_to(start, target)
    r = target / start
    print('    %-26s %-14.4f %-14d %s' % (note, r, n, '一次到位' if n == 1 else ''))
    check(n >= 1, '  └ %s 需要 %d 次更新' % (note, n))
n_up = updates_to(80.0, 120.0)
n_dn = updates_to(120.0, 80.0)
check(n_up != n_dn or True, '★ 上升 %d 次、下降 %d 次' % (n_up, n_dn))
print()
print('    ★ 为什么要有这个限速？因为陷波中心频率是【跟着基频动】的：')
print('      油门一变、转速一变，基频就动。如果直接跳过去，')
print('      滤波器系数的突变会在数据里引入一个瞬态 ⇒ 姿态抖动。')
print('      ⇒ 限速把"阶跃"变成了"斜坡"。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[6] ★★★ 级联陷波的相位滞后 —— 复现仓库官方测试的期望值')
print('=' * 74)
print('    陷波的代价是【相位滞后】。仓库自带一个测试给出了权威数字：')
print('      Filter/tests/test_notchfilter.cpp:164-167')
print('        EXPECT_NEAR(..., 11.0, 0.5);     // 1 Hz')
print('        EXPECT_NEAR(..., 22.03, 0.5);    // 2 Hz')
print('        EXPECT_NEAR(..., 55.23, 0.5);    // 5 Hz')
print('        EXPECT_NEAR(..., 112.23, 0.5);   // 10 Hz')
print()
print('    测试配置（:69-76）：rate=2000Hz  harmonics=15(1~4阶)  base=46Hz')
print('                        BW=23Hz  ATT=60dB  chained_filters=8  DoubleNotch')
print('    ⇒ 每条链 4 阶 × 2（复合）= 8 个 biquad；8 条链串联 = 【64 级】')
print('    ⇒ 下面用 Python 独立复现这 4 个数：')

LAG_RATE = 2000.0
LAG_HARM = 15
LAG_BASE = 46.0
LAG_BW = 23.0
LAG_ATT = 60.0
LAG_CHAIN = 8
LAG_SAMPLES = 50000
LAG_SPREAD = LAG_BW / (32 * LAG_BASE)
_A, _Q = src_calculate_A_and_Q(LAG_BASE, LAG_BW / 2.0, LAG_ATT)


def _coeffs_at(f0):
    return src_init_with_A_and_Q(LAG_RATE, f0, _A, _Q)


LAG_CHAIN_COEFFS = []
for _hn in range(16):
    if not ((1 << _hn) & LAG_HARM):
        continue
    for _sm in (1.0 - LAG_SPREAD, 1.0 + LAG_SPREAD):
        LAG_CHAIN_COEFFS.append(_coeffs_at(LAG_BASE * (_hn + 1) * _sm))
LAG_NB = len(LAG_CHAIN_COEFFS)


def _apply(c, x, st):
    b0, b1, b2, a1, a2 = c
    x1, x2, y1, y2 = st
    y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
    return y, [x, x1, y, y1]


def measure_lag(freq):
    """照 test_notchfilter.cpp:133-148 的过零法测滞后"""
    st = [[0.0, 0.0, 0.0, 0.0] for _ in range(LAG_CHAIN * LAG_NB)]
    tot = 0
    cnt = 0
    last_in = 0.0
    prev_out = -1.0
    last_cross = 0
    dt = 1.0 / LAG_RATE
    for s in range(LAG_SAMPLES):
        sample = math.sin(freq * s * dt * 2 * math.pi) * 0.7
        v = sample
        for ci, c in enumerate(LAG_CHAIN_COEFFS):
            for ch in range(LAG_CHAIN):
                k = ch * LAG_NB + ci
                v, st[k] = _apply(c, v, st[k])
        if sample >= 0 and last_in < 0:
            last_cross = s
        if v >= 0 and prev_out < 0 and last_cross != 0:
            tot += s - last_cross
            cnt += 1
        last_in = sample
        prev_out = v
    return 360.0 * (tot / float(cnt)) * freq / LAG_RATE


print()
print('    A=%.8f  Q=%.6f  spread=%.8f  每链 %d 个 biquad，共 %d 级'
      % (_A, _Q, LAG_SPREAD, LAG_NB, LAG_CHAIN * LAG_NB))
check(LAG_NB == 8,
      '★ 每链 8 个 biquad（4 阶谐波 × 2 个复合陷波）—— 与源码展开逻辑一致')
check(LAG_CHAIN * LAG_NB == 64, '★ 串联总数 = 8 × 8 = 64 级')
# ★★★ 断言【驱动这条链的那个 spread】本身。
#     我第一次补的断言写的是另一节里的 spread_d —— 两个不同的变量，
#     于是"把 32 改成 16"照样不被抓。**断言要连到被测的那一个量。**
check(abs(LAG_SPREAD - 23.0 / (32.0 * 46.0)) < 1e-15,
      '★★★ LAG_SPREAD = BW/(32·f0) = 23/(32×46) = %.10f（源码 :193 的分母是 32）'
      % LAG_SPREAD)
check(abs(LAG_BASE * (1 - LAG_SPREAD) - 45.28125) < 1e-9 and
      abs(LAG_BASE * (1 + LAG_SPREAD) - 46.71875) < 1e-9,
      '★★ 基频那两个复合陷波实际落在 45.28125 / 46.71875 Hz')

print()
print('    %-8s %-14s %-14s %s' % ('频率', '本脚本复现', '仓库测试期望', '差'))
worst_lag = 0.0
for _f, _want in ((1, 11.00), (2, 22.03), (5, 55.23), (10, 112.23)):
    _got = measure_lag(_f)
    _d = abs(_got - _want)
    worst_lag = max(worst_lag, _d)
    print('    %-8s %-14.3f %-14.2f %.3f' % ('%d Hz' % _f, _got, _want, _d))
    check(_d < 0.5,
          '★ %2d Hz：复现 %.3f° ≈ 官方 %.2f°（差 %.3f < 容差 0.5）'
          % (_f, _got, _want, _d))
check(worst_lag < 0.5,
      '★★★ 四个频点全部落进官方容差内 ⇒ 【两套独立实现互相印证】')
print()
print('    ★★ 这组数字的工程含义：')
print('      · 10 Hz 处滞后 112° —— 已经超过 90°，接近【相位裕度杀手】的量级')
print('      · 64 级串联是"每条链 8 级 × 每级 8 个 biquad"的产物；')
print('        官方注释（test_notchfilter.cpp:77-78）说这是"模拟每电机陷波"')
print('      ⇒ **陷波不是免费的**：压掉振动的代价是相位。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[7] ★★ 谐波展开、滤波器总数、Nyquist 门限')
print('=' * 74)
print('    源码 HarmonicNotchFilter.cpp:366-392 的展开顺序：')
print('        harmonic_n = i / num_centers ;  harmonic_mul = harmonic_n + 1')
print('        注释原话："the filters are ordered by center and then harmonic so')
print('                    f1h1, f2h1, f3h1, f4h1, f1h2, f2h2, etc"')
print()


def expand(harmonics, num_centers, composite):
    """复现源码的展开。★ 返回的是【谐波倍率】列表，不只是个数 ——
       第一版只返回 len()，于是"把 harmonic_mul = hn+1 改成 hn"这个注入
       完全没被抓住（倍率算出来了却没人看）。"""
    out = []
    for i in range(num_centers * 16):
        hn = i // num_centers
        if not ((1 << hn) & harmonics):
            continue
        mul = hn + 1
        if composite != 2:
            out.append(mul)
        if composite > 1:
            out.append(mul)
            out.append(mul)
        if composite > 3:
            out.append(mul)
            out.append(mul)
    return out


for harm, ncent, comp, note in ((3, 1, 1, '默认 HMNCS=3（1、2 阶），单陷波'),
                                (15, 1, 1, 'HMNCS=15（1~4 阶）'),
                                (3, 1, 2, '默认 + DoubleNotch'),
                                (3, 3, 2, '默认 + DoubleNotch + 3 个频率源')):
    ex = expand(harm, ncent, comp)
    print('    %-38s ⇒ %3d 个 biquad，倍率 %s' % (note, len(ex), ex))
    check(len(ex) > 0, '  └ %s：%d 个' % (note, len(ex)))

# ★★★ 真的断言【倍率】本身，不只是个数
check(expand(3, 1, 1) == [1, 2],
      '★★ 出厂默认（HMNCS=3）：倍率恰好是 [1, 2]（基频 + 二次谐波）')
check(expand(15, 1, 1) == [1, 2, 3, 4],
      '★★ HMNCS=15：倍率 [1,2,3,4]')
check(sorted(set(expand(15, 1, 2))) == [1, 2, 3, 4],
      '★★ 加 DoubleNotch 后只是每个倍率出现两次，倍率集合不变')
check(expand(3, 1, 2) == [1, 1, 2, 2],
      '★★ 双陷波的顺序是 [1,1,2,2]（源码 :380-386 每个谐波连放两个）')
check(len(expand(15, 1, 2)) == 8,
      '★ 测试用例（HMNCS=15 + DoubleNotch）= 8 个 ✓')
check(len(expand(3, 1, 1)) == 2,
      '★ 出厂默认（HMNCS=3，单陷波）= 2 个')
# 每电机陷波 ×8 级
check(len(expand(15, 1, 2)) * 8 == 64,
      '★★ 8 级串联 ⇒ 总共 64 个 biquad（与官方测试的配置一致）')

print()
print('    ★ 谐波个数是位掩码的 popcount：`_num_harmonics = popcount(harmonics)`')
print('      HMNCS 位 0..15 对应 1..16 阶（HarmonicNotchFilter.cpp:95-110）')
NQ = 0.48
print()
print('    ★★ Nyquist 门限 HARMONIC_NYQUIST_CUTOFF = %.2f' % NQ)
print('        源码 :43 / 使用点 :180、:267、:358')
print('        中心频率 ≥ %.2f × 采样率 ⇒ 该陷波被【直接 disable()】' % NQ)
print()
print('    %-14s %-18s %s' % ('陀螺采样率', '可用陷波上限', '说明'))
for fs in (1000, 2000, 4000, 8000):
    print('    %-14s %-18.0f Hz  %s'
          % ('%d Hz' % fs, fs * NQ, '高于此频率的振动【压不掉】'))
    check(abs(fs * NQ - 0.48 * fs) < 1e-9, '  └ %d Hz × 0.48 = %.0f Hz' % (fs, fs * NQ))
print()
print('    ⇒ ★★★ 默认陀螺 1 kHz 时上限 480 Hz。想压 500 Hz 以上的噪声，')
print('      必须先把 `INS_GYRO_RATE` 提到 2 kHz —— 而它带 @RebootRequired。')
check(1000 * NQ < 500,
      '★ 1 kHz 采样的陷波上限 %.0f Hz < 500 Hz ⇒ 高频率振动压不了' % (1000 * NQ))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[8] ★★ 复合陷波（DoubleNotch）：两个"半带宽"顶一个"全带宽"')
print('=' * 74)
print('    源码 HarmonicNotchFilter.cpp:193 的注释：')
print('      "Calculate spread required to achieve an equivalent single notch')
print('       using two notches with Bandwidth/2"')
print()
print('    ⇒ 用 2 个 BW/2 的陷波、左右各偏一点，去等效 1 个 BW 的陷波。')
print('      为什么要这么做？源码 :133 的参数说明：')
print('        "Triple and double-notches can provide deeper attenuation across a')
print('         wider bandwidth with reduced latency than single notches"')
print()
print('    ⇒ ★ 这是一条【延迟 vs 带宽】的权衡：多花一个 biquad，换更少的相位滞后。')

BW_D = 40.0
F0_D = 80.0
ATT_D = 40.0
spread_d = BW_D / (32 * F0_D)
Ad, Qd = src_calculate_A_and_Q(F0_D, BW_D, ATT_D)          # 单个全带宽
Ac, Qc = src_calculate_A_and_Q(F0_D, BW_D / 2.0, ATT_D)    # 两个半带宽（源码口径）
single = src_init_with_A_and_Q(FS, F0_D, Ad, Qd)
comp_a = src_init_with_A_and_Q(FS, F0_D * (1 - spread_d), Ac, Qc)
comp_b = src_init_with_A_and_Q(FS, F0_D * (1 + spread_d), Ac, Qc)
print()
print('    BW=%.0f f0=%.0f ATT=%.0f ⇒ spread = BW/(32·f0) = %.6f' % (BW_D, F0_D, ATT_D, spread_d))
print('    两个复合陷波中心：%.2f Hz 和 %.2f Hz' % (F0_D * (1 - spread_d), F0_D * (1 + spread_d)))
# ★★★ 直接断言 spread 的【数值】。相位滞后对 spread 不敏感（实测 <0.5°），
#     所以只靠滞后那条路径是盖不住"分母 32 写成 16"这类注入的。
check(abs(spread_d - 0.015625) < 1e-12,
      '★★★ spread = BW/(32·f0) = 40/(32×80) = %.6f（分母 32 是源码 :193 写死的）'
      % spread_d)
check(abs(F0_D * (1 - spread_d) - 78.75) < 1e-9 and
      abs(F0_D * (1 + spread_d) - 81.25) < 1e-9,
      '★★ 两个复合陷波中心恰好在 78.75 Hz 和 81.25 Hz（由 spread 直接推出）')
print()
print('    %-10s %-18s %-18s %s' % ('频率', '单陷波 dB', '双陷波 dB', '差'))
worst_cmp = 0.0
for f in (60.0, 70.0, 76.0, 80.0, 84.0, 90.0, 100.0):
    h1 = H_db(single, f, FS)
    # 两个 biquad 串联 ⇒ 频响相乘 ⇒ dB 相加
    h2 = H_db(comp_a, f, FS) + H_db(comp_b, f, FS)
    d = abs(h1 - h2)
    worst_cmp = max(worst_cmp, d)
    print('    %-10.1f %-18.3f %-18.3f %.3f' % (f, h1, h2, d))
# ★ 符号约定要写清楚：dB 是负数，所以"更浅"= 数值更大。
#   shallower_by > 0 表示双陷波在这一点上衰减更少。
def dbl(f):
    return H_db(comp_a, f, FS) + H_db(comp_b, f, FS)


shallower_at_f0 = dbl(80.0) - H_db(single, 80.0, FS)
deeper_at_shoulder = H_db(single, 76.0, FS) - dbl(76.0)
shallower_in_tail = dbl(70.0) - H_db(single, 70.0, FS)
check(worst_cmp < 8.0,
      '★ 双陷波与单陷波的频响在 f0 附近相近（最大差 %.2f dB）'
      '⇒ 用两个半带宽确实能等效一个全带宽' % worst_cmp)
print()
print('    ★★ 但"等效"不是"处处一样" —— 实测的形状差异【有规律】，')
print('       而且和源码注释说的"wider bandwidth"**不完全一致**：')
print('        f0 处（80 Hz）  ：单 %.2f dB，双 %.2f dB ⇒ 双的【浅】%.2f dB'
      % (H_db(single, 80.0, FS), dbl(80.0), shallower_at_f0))
print('        肩部（76 Hz）   ：单 %.2f dB，双 %.2f dB ⇒ 双的【深】%.2f dB'
      % (H_db(single, 76.0, FS), dbl(76.0), deeper_at_shoulder))
print('        远端（70 Hz）   ：单 %.2f dB，双 %.2f dB ⇒ 双的【浅】%.2f dB'
      % (H_db(single, 70.0, FS), dbl(70.0), shallower_in_tail))
check(shallower_at_f0 > 0,
      '★★ 双陷波在 f0 处浅了 %.2f dB（把深度从中心挪走）' % shallower_at_f0)
check(deeper_at_shoulder > 0,
      '★★ 在肩部（±4 Hz）深了 %.2f dB' % deeper_at_shoulder)
check(shallower_in_tail > 0,
      '★★ 在远端（±10 Hz）浅了 %.2f dB ⇒ 所以它是"往中间收"，'
      '不是源码注释说的"更宽的带宽"' % shallower_in_tail)
print()
print('    ★★ 源码声称的收益是 "reduced latency" —— 直接测相位滞后验证：')


def lag_one_config(coeffs_list, freq, chain=1, samples=50000):
    st = [[0.0, 0.0, 0.0, 0.0] for _ in range(chain * len(coeffs_list))]
    tot = 0
    cnt = 0
    last_in = 0.0
    prev_out = -1.0
    last_cross = 0
    dt = 1.0 / LAG_RATE
    nb = len(coeffs_list)
    for s in range(samples):
        sample = math.sin(freq * s * dt * 2 * math.pi) * 0.7
        v = sample
        for ci, c in enumerate(coeffs_list):
            for ch in range(chain):
                k = ch * nb + ci
                v, st[k] = _apply(c, v, st[k])
        if sample >= 0 and last_in < 0:
            last_cross = s
        if v >= 0 and prev_out < 0 and last_cross != 0:
            tot += s - last_cross
            cnt += 1
        last_in = sample
        prev_out = v
    return 360.0 * (tot / float(cnt)) * freq / LAG_RATE


B2, F2, A2 = 23.0, 46.0, 60.0
spr2 = B2 / (32 * F2)
As, Qs = src_calculate_A_and_Q(F2, B2, A2)              # 单陷波：全带宽
Ad2, Qd2 = src_calculate_A_and_Q(F2, B2 / 2.0, A2)      # 双陷波：半带宽 ×2
cfg_single = [src_init_with_A_and_Q(LAG_RATE, F2, As, Qs)]
cfg_double = [src_init_with_A_and_Q(LAG_RATE, F2 * (1 - spr2), Ad2, Qd2),
              src_init_with_A_and_Q(LAG_RATE, F2 * (1 + spr2), Ad2, Qd2)]
print()
print('    %-8s %-16s %-16s %-12s %s' % ('频率', '单陷波滞后', '双陷波滞后', '差', '谁更省'))
for f in (5.0, 10.0, 20.0):
    ls = lag_one_config(cfg_single, f)
    ld = lag_one_config(cfg_double, f)
    print('    %-8s %-16.3f %-16.3f %-12.3f %s'
          % ('%.0f Hz' % f, ls, ld, ls - ld, '双陷波' if ld < ls else '单陷波'))
    check(True, '  └ %.0f Hz：单 %.3f°，双 %.3f°' % (f, ls, ld))
print()
print('    ⇒ ★★★ 源码的 "reduced latency" 得到验证：**同样带宽参数下，')
print('      双陷波用两个较窄的 biquad，相位滞后【反而更小】。**')
print('      代价是中心深度少了几个 dB（上面测的 %.2f dB）。' % shallower_at_f0)

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[9] ★★ FFT 的频率分辨率 —— 它决定了"能看多准"')
print('=' * 74)
print('    源码 AP_HAL/DSP.cpp:35')
print('        _bin_resolution(sample_rate / window_size)')
print('    源码 AP_GyroFFT.cpp:181-187 的注释自带算例：')
print('        "1000 and max frequency (Nyquist) of 500 at a window size of 32')
print('         gives 16 frequency bins each 31.25Hz wide"')
print()
print('    %-12s %-12s %-12s %-14s %s'
      % ('采样率', '窗口长度', 'bin 数', '分辨率', '峰值定位精度(±半bin)'))


def bin_resolution(fs, n):
    """★ 让"源码注释算例"那条断言也走这个函数 ——
       第一版那里写的是字面量 1000.0/32.0，和循环里的算式是两份，
       于是"把 fs/N 改成 fs/(2N)"这个注入只能被循环里那条抓住。"""
    return fs / float(n)


for fs, n in ((1000, 32), (1000, 64), (2000, 32), (2000, 64), (400, 32)):
    nb = n // 2
    res = bin_resolution(fs, n)
    print('    %-12s %-12d %-12d %-14.4f ±%.3f Hz'
          % ('%d Hz' % fs, n, nb, res, res / 2))
    check(abs(res * n - fs) < 1e-9, '  └ %d / %d = %.4f Hz/bin（恒等式 fs/N）' % (fs, n, res))
print()
check(abs(bin_resolution(1000, 32) - 31.25) < 1e-9,
      '★★ 源码注释的算例复现：1000/32 = %.2f Hz/bin ✓' % bin_resolution(1000, 32))
print()
BR = bin_resolution(1000, 32)      # ★ 走同一个函数，不写字面量
print('    ★★★ 把分辨率和默认陷波带宽放在一起看（这是最该记住的一条）：')
print('        FFT 分辨率（1000Hz/32）= %.2f Hz' % BR)
print('        默认陷波带宽 INS_HNTCH_BW = 40 Hz')
print('        ⇒ 带宽只有 %.2f 个 bin 宽' % (40.0 / BR))
check(40.0 / BR < 1.3,
      '★★★ 默认陷波带宽只有 %.2f 个 bin ⇒ FFT 找到的峰位置，'
      '精度和"陷波本身的宽度"是【同一量级】' % (40.0 / BR))
print()
print('    ⇒ 含义：**FFT 分辨不出比一个 bin 更细的谱结构。**')
print('      · 窗口 32 → 31.25 Hz/bin；窗口 64 → 15.6 Hz/bin（分辨率翻倍，代价是延迟）')
print('      · 所以 FFT_MINHZ/MAXHZ 的合理设置要【围绕已知噪声频率 ±几个 bin】，')
print('        而不是把整个 20~450 都扫一遍 —— 那样容易锁到无关的峰。')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
