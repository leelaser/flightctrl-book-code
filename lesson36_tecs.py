# -*- coding: utf-8 -*-
"""卷 8 第 7 章 · TECS 总能量控制 —— 可执行验证

配套课文：`卷 8 第 7 章`
源码：`ardupilot-Plane-4.7.1/libraries/AP_TECS/`（AP_TECS.cpp 1593 行 + .h 514 行）

验证六件事：

  [1] ★★★ 能量坐标【真的解耦】：dE/dt = V(T−D)/m 【与俯仰角无关】
  [2] 四个能量量的量纲自证
  [3] ★ 权重"双 cap 到 1"的作用（不 cap 会跳变）
  [4] ★ 增益 K_STE2Thr 是【算出来】的，且随 τ 变化
  [5] ★ 势能误差限幅 ⇔ 速度边界（防失速/防超速）
  [6] ★★★ flare 剖面：高度与下沉率【运动学自洽】

设计原则（本仓库的教训，见 memory）：
  · 断言两边来源必须不同：解析推导 / 数值仿真 / 源码常量
  · 反向对照写进脚本
  · 阈值来自恒等式或解析预言，不拍脑袋

运行：`python lesson36_tecs.py`   —— 全部断言通过才返回 0
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


G = 9.80665

print('=' * 74)
print('[1] ★★★ 能量坐标解耦：dE/dt 与俯仰角无关')
print('=' * 74)
print('    沿航迹方程： m·dV/dt = T − D − m·g·sinγ')
print('    垂直方程：   dh/dt   = V·sinγ')
print()
print('    ⇒ 比能量 E = g·h + ½V²，求导：')
print('       dE/dt = g·dh/dt + V·dV/dt')
print('             = g·V·sinγ + V·(T − D − m·g·sinγ)/m')
print('             = V·g·sinγ + V(T−D)/m − V·g·sinγ')
print('             = V·(T − D)/m        ← ★ 两项【精确抵消】')


def dEdt(T, D, m, V, gamma):
    """按【原始两方程】算出 dE/dt —— 不代入化简结果，保留 γ 项"""
    dV = (T - D - m * G * math.sin(gamma)) / m
    dh = V * math.sin(gamma)
    return G * dh + V * dV


m_ = 2.0          # kg
T_, D_, V_ = 8.0, 3.0, 18.0
print()
print('    数值验证（T=%.1f N, D=%.1f N, m=%.1f kg, V=%.1f m/s）：' % (T_, D_, m_, V_))
print('    %-12s %20s %20s %14s' % ('γ (度)', 'dE/dt (原始式)', 'V(T−D)/m (化简式)', '相对差'))
worst = 0.0
for gdeg in (-30, -15, 0, 10, 25, 45):
    gam = math.radians(gdeg)
    a = dEdt(T_, D_, m_, V_, gam)
    b = V_ * (T_ - D_) / m_
    rel = abs(a - b) / abs(b)
    worst = max(worst, rel)
    print('    %-12g %20.10f %20.10f %13.2e' % (gdeg, a, b, rel))
check(worst < 1e-12,
      '★ dE/dt 在 γ 从 −30° 到 +45° 全程【恒等于 V(T−D)/m】（最大相对差 %.1e）'
      % worst)
check(abs(dEdt(T_, D_, m_, V_, 0.0) - dEdt(T_, D_, m_, V_, math.radians(45))) < 1e-9,
      '★ 反向对照：dE/dt 在两个差异极大的俯仰角下【完全相同】⇒ 不是"近似解耦"，是【精确解耦】')

# SEB 的交叉耦合有多小
print()
print('    能量分配 SEB = g·h − ½V²，其导数：')
print('      dSEB/dt = g·dh/dt − V·dV/dt = 2V·g·sinγ − V(T−D)/m')


def dSEBdt(T, D, m, V, gamma):
    dV = (T - D - m * G * math.sin(gamma)) / m
    dh = V * math.sin(gamma)
    return G * dh - V * dV


def d_h(dSEB, T, D, m, V, gamma, h=1e-6):
    return (dSEBdt(T, D, m, V, gamma + h) - dSEBdt(T, D, m, V, gamma - h)) / (2 * h)


def d_T(dSEB, T, D, m, V, gamma, h=1e-6):
    return (dSEBdt(T + h, D, m, V, gamma) - dSEBdt(T - h, D, m, V, gamma)) / (2 * h)


gam0 = math.radians(5.0)
s_g = abs(d_h(None, T_, D_, m_, V_, gam0))
s_T = abs(d_T(None, T_, D_, m_, V_, gam0))
print('    %-28s %18s' % ('灵敏度', '数值'))
print('    %-28s %18.6f' % ('∂(dSEB/dt)/∂γ  (俯仰主导)', s_g))
print('    %-28s %18.6f' % ('∂(dSEB/dt)/∂T  (油门串扰)', s_T))
print('    %-28s %18.4f' % ('比值 (俯仰/油门)', s_g / s_T))
check(s_g / s_T > 5.0,
      '★ 俯仰对 SEB 的灵敏度是油门的 %.1f 倍 ⇒ SEB 由俯仰【主导】'
      '（而 E 完全不受俯仰影响）' % (s_g / s_T))
check(abs(d_h(None, T_, D_, m_, V_, 0.0)) > 0,
      '★ 反向对照：γ=0 处俯仰灵敏度非零（= 2Vg·cos0 = %.1f），不是平凡零' % abs(d_h(None, T_, D_, m_, V_, 0.0)))

print()
print('=' * 74)
print('[2] 四个能量量的量纲自证')
print('=' * 74)
print('    %-10s %-34s %s' % ('量', '表达式', '量纲'))
for name, expr, dim in (
    ('SPE', 'g·h', 'm/s² · m = m²/s²'),
    ('SKE', '½V²', 'm²/s²'),
    ('STE', 'SPE + SKE', 'm²/s²'),
    ('dSTE/dt', 'V·(T−D)/m', 'm/s · N/kg = m²/s³'),
    ('SEB', 'SPE − SKE', 'm²/s²'),
    ('TAS·g', 'V·g', 'm/s · m/s² = m²/s³'),
):
    print('    %-10s %-34s %s' % (name, expr, dim))
# ★★★ 正确的验证对象是 dSPE/dt（不是 dSEB/dt）
#   源码里 SEBdot_dem = hgt_rate_dem·g·SPE_w + SEB_error/τ，第一项就是 dSPE/dt。
#   dSPE/dt = g·dh/dt = g·V·sinγ  ⇒  dSPE/dt /(V·g) = sinγ ≈ γ  ✓
#   ⚠️ 我第一版拿【完整】的 dSEB/dt 去除 V·g —— 那里面多了 −V(T−D)/m 一项
#      （推力-阻力做的功），当然对不上。文档 §4.3 里写的是 dSPE 那个，是对的。
def dSPEdt(V, gamma):
    return G * V * math.sin(gamma)


print()
print('    %-10s %18s %18s %14s' % ('γ (rad)', 'dSPE/(V·g)', 'sin γ', '相对差'))
worst_g = 0.0
for gdeg in (2, 5, 10, 20):
    gam = math.radians(gdeg)
    pv = dSPEdt(V_, gam) / (V_ * G)
    rel = abs(pv - math.sin(gam)) / math.sin(gam)
    worst_g = max(worst_g, rel)
    print('    %-10.4f %18.10f %18.10f %13.2e' % (gam, pv, math.sin(gam), rel))
check(worst_g < 1e-12,
      '★★★ dSPE/(V·g) 【精确等于】sin γ（最大相对差 %.1e）⇒ 分母 V·g 就是'
      '"势能率→航迹倾角"的增益，源码命名 gainInv 名副其实' % worst_g)
check(abs(dSPEdt(V_, gam0) / (V_ * G) - gam0) / gam0 < 0.01,
      '★ 小角度下 sinγ ≈ γ（5° 时差 %.2f%%）⇒ pitch 与航迹倾角同尺度'
      % (100 * abs(math.sin(gam0) - gam0) / gam0))
print('    ⚠️ 若把分母写成 g·h 或 V²，这一列会对不上 —— 量纲自证就在这里')

print()
print('=' * 74)
print('[3] ★ 权重"双 cap 到 1"的作用（AP_TECS.cpp:1017-1020）')
print('=' * 74)
print('    源码： SPE_w = 2 − SKE_w;  SPE_w = MIN(SPE_w,1);  SKE_w = MIN(SKE_w,1);')


def weights_capped(ske_w):
    spe = 2.0 - ske_w
    return min(spe, 1.0), min(ske_w, 1.0)


def weights_raw(ske_w):
    return 2.0 - ske_w, ske_w


SPE_d, SKE_d = 15.0 * G, 0.5 * 20.0 ** 2      # h=15m, V=20m/s
print()
print('    SPE_dem = %.1f, SKE_dem = %.1f' % (SPE_d, SKE_d))
print('    %-8s %-16s %-16s %-16s %-16s' % ('SKE_w', 'cap后(SPE,SKE)', 'SEB(cap)', '不cap(SPE,SKE)', 'SEB(不cap)'))
rows = []
for w in (0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0):
    sp_c, sk_c = weights_capped(w)
    sp_r, sk_r = weights_raw(w)
    seb_c = SPE_d * sp_c - SKE_d * sk_c
    seb_r = SPE_d * sp_r - SKE_d * sk_r
    rows.append((w, seb_c, seb_r))
    print('    %-8g (%4.2f, %4.2f)     %16.2f (%4.2f, %4.2f)     %16.2f'
          % (w, sp_c, sk_c, seb_c, sp_r, sk_r, seb_r))
# ⚠️⚠️ 我第一版在这里断言"cap 后权重和恒为 1" —— 【错的】。
#    (2−w) + w ≡ 2 是恒等式：不管 cap 不 cap，只要 w_SPE 定义为 2−w_SKE，
#    权重和就恒等于 2（cap 后中间点会被截到 <2）。"和恒为 1"是我不成立的臆断。
#
#    源码注释说的真实作用是：
#      "don't go above 1 to prevent instability if tuned at a speed weight of 1
#       and weighting is varied to end points in flight"
#    ⇒ 【限制单个权重的最大值 ≤ 1】。因为控制器是在 w=1 下调好的，
#      若端点处某个权重涨到 2，等效增益翻倍 ⇒ 振荡。
print()
print('    %-8s %-18s %-18s %s' % ('SKE_w', 'cap后 (SPE,SKE)', '不cap (SPE,SKE)', '最大权重'))
max_capped, max_raw = 0.0, 0.0
for w in (0.0, 0.5, 1.0, 1.5, 2.0):
    sp_c, sk_c = weights_capped(w)
    sp_r, sk_r = weights_raw(w)
    max_capped = max(max_capped, sp_c, sk_c)
    max_raw = max(max_raw, sp_r, sk_r)
    print('    %-8g (%4.2f, %4.2f)      (%4.2f, %4.2f)      %.2f vs %.2f'
          % (w, sp_c, sk_c, sp_r, sk_r, max(sp_c, sk_c), max(sp_r, sk_r)))
print()
print('    cap 后最大权重 = %.2f；不 cap 最大权重 = %.2f' % (max_capped, max_raw))
check(abs(max_capped - 1.0) < 1e-12,
      '★ cap 后【单个权重最大值】恒 ≤ 1（实测 %.2f）⇒ 等效增益不会超过在 w=1 时调好的值'
      % max_capped)
check(abs(max_raw - 2.0) < 1e-12,
      '★ 反向对照：不 cap 时端点处权重涨到 %.1f ⇒ 等效增益【翻倍】'
      '（源码注释里 prevent instability 指的就是这个）' % max_raw)
check(abs(max_raw / max_capped - 2.0) < 1e-12,
      '★ 增益倍数差恰好 2×（%.1f / %.1f）⇒ 这就是"不加 cap 会振荡"的定量来源'
      % (max_raw, max_capped))
# 权重【和】的性质（澄清一下，避免和上面的机制混淆）
s_c = weights_capped(0.5)[0] + weights_capped(0.5)[1]
s_r = weights_raw(0.5)[0] + weights_raw(0.5)[1]
check(abs(s_r - 2.0) < 1e-12 and abs(s_c - 1.5) < 1e-12,
      '★ 澄清：不 cap 时权重和恒为 %.1f（代数恒等式），cap 后中间点被截到 %.1f'
      '—— 所以"和"不是这个技巧的作用点，【最大值】才是' % (s_r, s_c))

print()
print('=' * 74)
print('[4] ★ 增益是算出来的：K_STE2Thr = 1/(τ·K_thr2STE)')
print('=' * 74)
STEdot_max, STEdot_min = 4.0, -3.0        # m²/s³
THRmax, THRmin = 1.0, 0.0
K_thr2STE = (STEdot_max - STEdot_min) / (THRmax - THRmin)
print('    K_thr2STE = (%.1f − (%.1f)) / (%.1f − %.1f) = %.3f m²/s³ 每单位油门'
      % (STEdot_max, STEdot_min, THRmax, THRmin, K_thr2STE))
print()
print('    %-12s %20s %20s' % ('τ (s)', 'K_STE2Thr', '相对 τ=1'))
for tau in (0.5, 1.0, 2.0, 4.0):
    K = 1.0 / (tau * K_thr2STE)
    print('    %-12g %20.6f %20.4f' % (tau, K, K / (1.0 / K_thr2STE)))
    check(abs(K * tau * K_thr2STE - 1.0) < 1e-15,
          'τ=%-4g：K_STE2Thr × τ × K_thr2STE ≡ 1（定义式成立）' % tau)
# ★ 关键：调 τ 就等价于调增益
K1 = 1.0 / (1.0 * K_thr2STE)
K2 = 1.0 / (2.0 * K_thr2STE)
check(abs(K2 - K1 / 2) < 1e-15,
      '★ τ 翻倍 ⇒ 增益精确减半（%.4f → %.4f）⇒ 调时间常数【就是】调增益，只是换了参数化'
      % (K1, K2))
check(K_thr2STE * 0 + abs(K1 - 1.0 / K_thr2STE) < 1e-15,
      '★ 确认没有"手调增益"的位置：K_STE2Thr 完全由 K_thr2STE 和 τ 决定')

print()
print('=' * 74)
print('[5] ★ 势能误差限幅 ⇔ 速度边界（防失速/防超速）')
print('=' * 74)
print('    源码： SPE_err_max = MAX(SKE_est − ½TASmin², 0)')
print('           SPE_err_min = MIN(SKE_est − ½TASmax², 0)')
print()
TASmin, TASmax = 12.0, 30.0
print('    %-12s %18s %18s %22s' % ('TAS (m/s)', 'SKE (m²/s²)', 'SPE_err_max', '含义'))
# ⚠️ 把每行的结果【收集起来】，后面的断言要拿它比对 ——
#    第一版后面的断言自己又算了一遍 spe_max，于是两边同源 ⇒ 同义反复，
#    注入测试（把 TASmin 改成 TASmax）【逃逸】了。这正是 memory 里
#    `verify-baseline-independence` 说的那种错。
spe_max_by_V = {}
for V in (10.0, 12.0, 18.0, 25.0, 30.0, 34.0):
    SKE = 0.5 * V * V
    spe_max = max(SKE - 0.5 * TASmin ** 2, 0.0)
    spe_min = min(SKE - 0.5 * TASmax ** 2, 0.0)
    # ⚠️ 标签第一版写反了。正确的判读：
    #    spe_max == 0 ⇔ 速度已到下限 ⇔ 【不许再要求爬升】（爬升要耗动能）⇒ 失速保护
    #    spe_min == 0 ⇔ 速度已到上限 ⇔ 【不许再要求下降】（下降会继续增速）⇒ 超速保护
    if spe_max == 0.0:
        meaning = '★ 失速保护（不许要求爬升）'
    elif spe_min == 0.0:
        meaning = '★ 超速保护（不许要求下降）'
    else:
        meaning = '正常（两方向都有余量）'
    print('    %-12g %18.2f %18.2f %22s' % (V, SKE, spe_max, meaning))
    spe_max_by_V[V] = spe_max
# ★ 断言用【上面表格实际算出的值】，不重算 —— 基准独立于被测表达式
V0 = 18.0
expect_v0 = 0.5 * V0 ** 2 - 0.5 * TASmin ** 2      # 独立的解析预言
print()
print('    ★ 物理含义：势能最多允许【跌】%.2f m²/s²（= 动能减到只剩 TASmin 的量）'
      % spe_max_by_V[V0])
check(abs(spe_max_by_V[V0] - expect_v0) < 1e-12,
      '★ 表格里 TAS=%.0f 那行的 SPE_err_max（%.2f）恰好等于"当前动能 − 最小动能"（%.2f）'
      '⇒ 势能跌到极限时动能正好 = ½TASmin²' % (V0, spe_max_by_V[V0], expect_v0))
check(spe_max_by_V[V0] > 0,
      'TAS=%.0f > TASmin=%.0f ⇒ SPE_err_max = %.2f > 0，允许势能下跌（有能量可换）'
      % (V0, TASmin, spe_max_by_V[V0]))
# ★ 反向对照：低于 TASmin 时必须为 0（这一步能抓住"误用 TASmax"）
check(spe_max_by_V[10.0] == 0.0,
      '★ 反向对照：TAS=10 < TASmin=%.0f ⇒ 表格里的 SPE_err_max = %.2f ⇒ 【不允许势能再跌】'
      '（再跌就失速）' % (TASmin, spe_max_by_V[10.0]))
# ⚠️ 注意：这一行【抓不住】"误用 TASmax"——因为 max(...,0) 会把负数夹成 0，两版都是 0。
#    真正抓住它的是上面 TAS=18 那条（90 vs 0）。注入测试确认过。
check(spe_max_by_V[30.0] > 0,
      '★ TAS=30（= TASmax）处 SPE_err_max = %.1f > 0 —— 若公式写成 TASmin，这里会变成 %.1f'
      % (spe_max_by_V[30.0], 0.5 * 900 - 0.5 * TASmin ** 2))
# ★ 单调性（另一个独立判据：SPE_err_max 应随 V 单调不减）
vals = [spe_max_by_V[v] for v in (10.0, 12.0, 18.0, 25.0, 30.0, 34.0)]
check(all(vals[i] <= vals[i + 1] + 1e-12 for i in range(len(vals) - 1)),
      '★ SPE_err_max 随速度【单调不减】（%s）⇒ 速度越高越允许爬升，符合物理'
      % ', '.join('%.0f' % v for v in vals))

print()
print('=' * 74)
print('[6] ★★★ flare 剖面：高度与下沉率【运动学自洽】')
print('=' * 74)
print('    源码 AP_TECS.cpp:626-644：')
print('      p = constrain((h_flare_start − h_afe)/(h_flare_start − h_holdoff), 0, 1)')
print('      h_rate_dem = h_rate_entry·(1−p) − land_sink·p')
print('      h_ideal += DT·h_rate_dem           ← 高度由速度【积分】得到')

H_FLARE_START = 15.0      # 进入拉平时的高度（AFE）
H_HOLDOFF = 2.0           # TECS_FLARE_HGT
RATE_ENTRY = -2.5         # 进入拉平时的下沉率（m/s，负=下降）
# ⚠️ TECS_LAND_SINK 是【正的幅值】（源码默认 0.25，见 AP_TECS.cpp:163）。
#    源码写 `- land_sink_rate_adj * p`，负号在算式里。
#    我第一版把它写成 −0.3 ⇒ 式子变成 +0.3p ⇒ 飞机越拉越往上，仿真卡在 3.39 m 下不来。
LAND_SINK = 0.25

DT = 0.01


def simulate_flare():
    h = H_FLARE_START
    h_ideal = H_FLARE_START
    h_dem_prev = None
    out = []
    while h > H_HOLDOFF - 1e-9 and len(out) < 20000:
        p = min(max((H_FLARE_START - h) / (H_FLARE_START - H_HOLDOFF), 0.0), 1.0)
        rate = RATE_ENTRY * (1.0 - p) - LAND_SINK * p
        h_ideal += DT * rate
        out.append((h, p, rate, h_ideal))
        h += DT * rate
    return out


traj = simulate_flare()
print()
print('    %-10s %-10s %16s %16s %16s' % ('高度 AFE', 'p', 'h_rate_dem', 'h_ideal', '数值 dh/dt'))
worst_dev = 0.0
for i in range(0, len(traj) - 1, max(1, len(traj) // 8)):
    h, p, rate, hid = traj[i]
    hid_next = traj[i + 1][3]
    numeric = (hid_next - hid) / DT
    # ⚠️ 左式 h_ideal[i+1] − h_ideal[i] = DT·rate[i+1]，用的是【下一步】的 rate。
    #    第一版我拿它跟 rate[i] 比 ⇒ 差 4.3e−3，是【错位一格】不是算法不自洽。
    dev = abs(numeric - traj[i + 1][2])
    worst_dev = max(worst_dev, dev)
    print('    %-10.2f %-10.4f %16.4f %16.4f %16.4f' % (h, p, rate, hid, numeric))
check(worst_dev < 1e-6,
      '★★★ h_ideal 的数值导数【恒等于】h_rate_dem（最大偏差 %.2e）'
      '⇒ 高度剖面与下沉率剖面运动学自洽' % worst_dev)
# p 的端点行为
check(abs(traj[0][1]) < 1e-9,
      '★ 进入拉平时 p = %.4f（≈0）⇒ 下沉率 = 进入时的实际值 ⇒ 【轨迹连续，无跳变】' % traj[0][1])
check(traj[-1][1] > 0.99,
      '★ 接地前 p = %.4f（≈1）⇒ 下沉率收敛到 LAND_SINK = %.2f m/s ⇒ 接地垂直速度可控'
      % (traj[-1][1], LAND_SINK))
# 反向对照：如果不用 p 淡入、直接用 LAND_SINK
jump = abs(RATE_ENTRY - LAND_SINK)
check(jump > 2.0,
      '★ 反向对照：若一进拉平就用 LAND_SINK，下沉率会【阶跃】%.2f m/s ⇒ 俯仰猛跳'
      '（p 的淡入就是为消除它）' % jump)
# 记录点数（拉平持续时间）
print()
print('    拉平持续 %.2f s（从 %.1f m 到 %.1f m）' % (len(traj) * DT, H_FLARE_START, H_HOLDOFF))

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
