# -*- coding: utf-8 -*-
"""卷 1 第 6 章 · 捷联惯导更新算法 —— 补充：静基座误差分析（严恭敏 §4.3）

配套课文：`卷 1 第 6 章` §5.4–§5.6
源书：严恭敏, 翁浚《捷联惯导算法与组合导航原理》第 4 章 §4.3（PDF p100–p104）

验证四件事：

  [1] 舒勒频率 ωs = √(g/R) 与那张常数表（含 √(gR) = 第一宇宙速度）
  [2] 特征方程 (4.3-19) 的因式分解
  [3] 罗经效应：(4.3-20) 传递函数 ←→ (4.3-21) 时间解 ←→ (4.3-22) 短时近似
  [4] 误差增长阶数：加计零偏 t²、陀螺漂移 t³（数值测指数，不靠公式）

设计原则（本仓库的教训，见 memory）：
  · 每条断言的基准独立于被测对象（解析解 / 数值积分 / 物理常数各算各的）
  · ★ 不自己"记"一个状态空间模型再去验证它 —— 罗经效应直接验证【书上给的】传递函数与时间解
  · 阈值要么来自恒等式、要么来自量纲、要么来自物理常数
  · 反向对照写进脚本：错的写法必须被测出来

运行：`python lesson07c_static_error.py`   —— 全部断言通过才返回 0
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


# 物理常数（WGS-84 / 天文常数）
G0 = 9.80665          # m/s^2   标准重力
R_E = 6371000.0       # m       平均地球半径（严恭敏书中用 R）
OMEGA_IE = 7.292115e-5  # rad/s 地球自转速率
SIDEREAL_DAY = 86164.0905  # s   恒星日

print('=' * 74)
print('[1] §5.4 舒勒频率与常数表')
print('=' * 74)

omega_s = math.sqrt(G0 / R_E)
T_s = 2 * math.pi / omega_s
v_orbit_theory = math.sqrt(G0 * R_E)
T_ie = 2 * math.pi / OMEGA_IE
ratio = OMEGA_IE / omega_s

print('    ωs = √(g/R) = %.6e rad/s' % omega_s)
print('    Ts = 2π/ωs  = %.1f s = %.2f min = %.3f h' % (T_s, T_s / 60.0, T_s / 3600.0))
print('    √(gR)       = %.2f m/s' % v_orbit_theory)
print('    R·ωie       = %.2f m/s' % (R_E * OMEGA_IE))
print('    ωie/ωs      = %.5f' % ratio)
print('    ωie         = %.6e rad/s  → T = %.4f h（恒星日）' % (OMEGA_IE, T_ie / 3600.0))
print()
print('    书中 (PDF p098 表注) 给的近似值：√(gR)≈7900 m/s, R·ωie≈470 m/s, ωie/ωs≈0.06')
print()

check(abs(T_s - 5063.0) < 10.0,
      '舒勒周期 Ts = %.1f s ≈ 84.4 min（书中说"周期由 √(g/R) 决定"，量级核对）' % T_s)
# ★ 第一宇宙速度：v = √(GM/r)；用 g = GM/r² 即得 v = √(gR)。这是恒等式，不是巧合。
check(abs(v_orbit_theory - 7906.0) < 20.0,
      '√(gR) = %.1f m/s ≈ 第一宇宙速度 7.9 km/s（= 书中说的"第一宇宙速度"）' % v_orbit_theory)
check(abs(R_E * OMEGA_IE - 464.6) < 1.0,
      'R·ωie = %.1f m/s ≈ 书中给的 470 m/s（赤道自转线速度）' % (R_E * OMEGA_IE))
check(abs(ratio - 0.0588) < 0.001,
      'ωie/ωs = %.4f ≈ 书中给的 0.06 ⇒ 舒勒频率比地球自转快 %.0f 倍' % (ratio, 1.0 / ratio))
check(abs(T_ie / 3600.0 - 23.934) < 0.01,
      '地球自转周期 = %.3f h = **恒星日**（书中写"24h"是取整）' % (T_ie / 3600.0))

# ── 舒勒频率从【特征方程】来，不是从公式背来的 ──────────────────
# 水平单通道 (4.3-13/14)：δv̇N = −g·φE ,  φ̇E = −δvN/R
# 消元得 δv̈N = −(g/R)·δvN  ⇒ 特征方程 s² + g/R = 0
print('    水平单通道 (4.3-13/14) 的状态阵：[[0, −g], [−1/R, 0]]')
A2 = [[0.0, -G0], [-1.0 / R_E, 0.0]]
disc = -(A2[0][0] * A2[1][1] - A2[0][1] * A2[1][0])
print('    特征方程 det(sI−A) = s² + g/R = 0，根 = ±j·%.6e' % math.sqrt(disc))
check(abs(math.sqrt(disc) - omega_s) < 1e-12,
      '★ 从状态阵【算出的】特征频率 = √(g/R)（差 %.2e）—— 不是照抄公式' % abs(math.sqrt(disc) - omega_s))

print()
print('=' * 74)
print('[2] §5.4 特征方程 (4.3-19) 的因式分解')
print('=' * 74)
# 书：(4.3-19) Δ = s⁴ + (ωie² + g/R)s² + ωie²·g/R = (s²+ωie²)(s²+ωs²)
c2 = OMEGA_IE ** 2 + G0 / R_E
c0 = OMEGA_IE ** 2 * (G0 / R_E)
# 独立展开右边的乘积
lhs2, lhs0 = OMEGA_IE ** 2 + omega_s ** 2, OMEGA_IE ** 2 * omega_s ** 2
print('    书上系数：         s² 项 = %.6e   s⁰ 项 = %.6e' % (c2, c0))
print('    独立展开 (s²+ωie²)(s²+ωs²)：s² 项 = %.6e   s⁰ 项 = %.6e' % (lhs2, lhs0))
check(abs(c2 - lhs2) < 1e-20 and abs(c0 - lhs0) < 1e-24,
      '★ 因式分解逐项吻合 ⇒ 北向-方位通道确实【同时】含 24h 与 84.4min 两个无阻尼频率')
# ★ 反向对照：把 g/R 换成 2g/R（即"舒勒调谐调错一倍"），因式分解就不再成立
c2_wrong = OMEGA_IE ** 2 + 2 * G0 / R_E
check(abs(c2_wrong - lhs2) > 1e-6,
      '★ 反向对照：若把 ωs² 换成 2g/R，s² 项变成 %.6e ≠ %.6e ⇒ 上面那条有区分力'
      % (c2_wrong, lhs2))

print()
print('=' * 74)
print('[3] §5.5 罗经效应：(4.3-20) ↔ (4.3-21) ↔ (4.3-22)')
print('=' * 74)
LAT = 30.0 * math.pi / 180.0
PHI_U0 = 1.0 * math.pi / 180.0          # 1° 方位误差
K_GAIN = -G0 * OMEGA_IE * math.sin(LAT) * PHI_U0


def dv_exact(t):
    """(4.3-21) 精确时间解"""
    return K_GAIN / (omega_s ** 2 - OMEGA_IE ** 2) * (math.cos(OMEGA_IE * t) - math.cos(omega_s * t))


def dv_short(t):
    """(4.3-22) 短时近似。
    ⚠️ 是【负】的：cos 差 ≈ (ωs²−ωie²)t²/2 > 0，而 K_GAIN < 0。
       我第一版漏了负号，导致与精确解差 200%（符号相反、幅值相同）。"""
    return -0.5 * G0 * OMEGA_IE * math.sin(LAT) * PHI_U0 * t ** 2


def laplace_dv(s):
    """(4.3-20) 的右端（解析）。

    ⚠️⚠️ 分子上必须有【一个 s】—— 书上印的式子有，但 OCR 把它吃掉了。
       判定依据（两条互相印证）：
         ① 1/((s²+a²)(s²+b²)) 的逆变换是 **sin** 差；而书上 (4.3-21) 是 **cos** 差。
            cos 差对应的是 s/((s²+a²)(s²+b²))。
         ② 若取 sin 形式，短时阶数是 t³；而书上说 (4.3-22) 给 t² 且与表 4.3-2 一致。
       数值验证见下面的「数值拉氏变换」一节：加 s 后吻合到 0.0x%，不加差 33 万倍。
    """
    return K_GAIN * s / ((s ** 2 + OMEGA_IE ** 2) * (s ** 2 + omega_s ** 2))


def laplace_numeric(s, N=600000):
    """δv 的拉氏变换，用【数值积分】算 —— 独立于 (4.3-20) 的解析式。

    ⚠️ 积分上限必须随 s 走：被积函数的衰减尺度是 1/s。
       第一版固定 T=4000 s，对 s=1e-4（尺度 10⁴ s）严重截断，误差 42%。
       现在取 T = 10/s，覆盖 10 个衰减时间常数。"""
    T = 10.0 / s
    dt = T / N
    acc = 0.0
    for i in range(N):
        t = (i + 0.5) * dt
        acc += dv_exact(t) * math.exp(-s * t) * dt
    return acc


print('    输入：L=30°, φ_U0=1°, 等效加速度 a = g·ωie·sinL·φ_U0 = %.6e m/s²'
      % (G0 * OMEGA_IE * math.sin(LAT) * PHI_U0))
print()

# 短时近似的相对误差有【解析预言】：把两个 cos 展到四阶
#   cos差 = (ωs²−ωie²)t²/2 · [1 − (ωs²+ωie²)t²/12 + O(t⁴)]
# ⇒ 相对误差 ≈ (ωs²+ωie²)t²/12
# ⚠️ 第一版我拿"< 0.1%"当阈值 —— 那是拍脑袋的，t=200 时真实误差就是 0.51%。
#    现在改成【与解析预言的相对偏差】，这才是真正有区分力的判据。
print('    %-8s %16s %16s %14s %14s' % ('t (s)', '相对误差(实测)', '解析预言', '比值', ''))
worst_ratio_dev = 0.0
for t in (5.0, 20.0, 50.0, 100.0, 200.0):
    e, s_ = dv_exact(t), dv_short(t)
    rel = abs(e - s_) / abs(e)
    pred = (omega_s ** 2 + OMEGA_IE ** 2) * t ** 2 / 12.0
    dev = abs(rel - pred) / pred
    worst_ratio_dev = max(worst_ratio_dev, dev)
    print('    %-8g %16.6e %16.6e %13.4f %s' % (t, rel, pred, rel / pred, ''))
check(worst_ratio_dev < 0.05,
      '★ 短时近似的误差与解析预言 (ωs²+ωie²)t²/12 吻合（最大相对偏差 %.2f%%）'
      % (100 * worst_ratio_dev))
check(abs(dv_exact(200.0) - dv_short(200.0)) / abs(dv_exact(200.0)) < 0.01,
      't=200 s（ωs·t=%.2f）时两者仍差 < 1%%' % (omega_s * 200.0))

# ★★ 关键：精确解确实是 (4.3-20) 的逆变换 —— 用【数值积分】独立验证
print()
print('    ★ 验证 (4.3-21) 是 (4.3-20) 的逆拉氏变换（数值积分 vs 解析式）：')
worst = 0.0
for s in (1e-4, 3e-4, 1e-3):
    ana, num = laplace_dv(s), laplace_numeric(s)
    rel = abs(ana - num) / abs(ana)
    worst = max(worst, rel)
    print('      s=%.0e : 解析 %.8e   数值积分 %.8e   相对差 %.2e' % (s, ana, num, rel))
check(worst < 5e-3,
      '★ 数值拉氏变换与解析式 (4.3-20) 吻合（最差相对差 %.2e）⇒ 时间解 (4.3-21) 正确' % worst)

# ★ 「方位误差等效于一个常值北向加速度」= δvN 在 t=0 的二阶导等于 a。
#   ⚠️ 不要用有限差分硬算二阶导：
#     · 单边二阶差分 (f(2h)−2f(h)+f(0))/h² 的误差是 O(h)，我第一版就是这么写的，差 0.12%；
#     · 减小 h 又会撞上浮点相消（δv ~ 1e−14，h 太小就全被吃掉）。
#   改用【Richardson 外推】：δv 是偶函数 ⇒ δv(t)/t² = a/2 + c·t² + O(t⁴)，
#   于是 (4r(t/2) − r(t))/3 把 t² 项消掉，直接给出 a/2。
def a_from_limit(t):
    r1 = dv_exact(t) / t ** 2
    r2 = dv_exact(t / 2) / (t / 2) ** 2
    return 2.0 * (4.0 * r2 - r1) / 3.0


a_eq = -G0 * OMEGA_IE * math.sin(LAT) * PHI_U0      # ← 负号！
print()
print('    等效加速度 a = −g·ωie·sinL·φ_U0（解析）        ：%.8e' % a_eq)
print('    δvN(t)/t² 外推到 t→0 再乘 2（Richardson）      ：%.8e' % a_from_limit(50.0))
print('    同一外推换个起点 t=20                          ：%.8e' % a_from_limit(20.0))
check(abs(a_from_limit(50.0) - a_eq) / abs(a_eq) < 1e-6,
      '★ 「方位误差等效于常值北向加速度」在【数值极限】下成立（差 %.2e）'
      % (abs(a_from_limit(50.0) - a_eq) / abs(a_eq)))
# Richardson 外推的残差是 O(t⁴)；换起点（50→20）只应差到这个量级。
# ⚠️ 第一版我要求 < 1e−9，实际 1.0e−8 —— 又是拍脑袋的阈值。
check(abs(a_from_limit(20.0) - a_from_limit(50.0)) / abs(a_eq) < 1e-7,
      '换起点（t=50→t=20）外推结果不变到 %.2e ⇒ 与 O(t⁴) 残差量级一致，外推收敛'
      % (abs(a_from_limit(20.0) - a_from_limit(50.0)) / abs(a_eq)))

# ★★ 课文 §5.5 那张量级表的数字必须【由脚本算出来】，不能凭手推写进正文
print()
print('    ★ 课文 §5.5 量级表（由本脚本算出，正文照抄这些数）：')
print('    %-14s %16s %14s %14s %14s' % ('方位误差', 'a (m/s²)', '10s 后 δv', '100s 后 δv', '1000s 后 δv'))
mag = {}
for deg in (1.0, 0.1):
    phi0 = math.radians(deg)
    a = -G0 * OMEGA_IE * math.sin(LAT) * phi0
    row = [0.5 * a * t * t for t in (10.0, 100.0, 1000.0)]
    mag[deg] = (a, row)
    print('    %-14s %16.4e %14.4e %14.4e %14.4e'
          % ('%g°' % deg, a, row[0], row[1], row[2]))
check(abs(abs(mag[1.0][1][0]) - 3.1203e-4) < 1e-7,
      '校验：1° 方位误差 10 s 产生 %.4e m/s（≈0.31 mm/s）；'
      '⚠️ 我第一版凭手推写成 0.6 mm/s —— 漏了那个 1/2' % abs(mag[1.0][1][0]))

print()
print('=' * 74)
print('[4] §5.6 误差增长阶数：加计零偏 t²、陀螺漂移 t³')
print('=' * 74)
print('    模型（水平单通道 4.3-13/14，这是【书上明确给出】的两行）：')
print('      δv̇N = −g·φE + ∇N')
print('      φ̇E  = −δvN/R − εE')
print()


def simulate(nabla=0.0, eps=0.0, T=200.0, dt=1e-3):
    """直接数值积分上面两行，返回 (t, δvN, δpN) 的采样"""
    v = 0.0
    phi = 0.0
    p = 0.0
    t = 0.0
    n = int(round(T / dt))
    out = {}
    for i in range(n + 1):
        t = i * dt
        if i % int(round(50.0 / dt)) == 0 and i > 0:
            out[t] = (v, p)
        v += (-G0 * phi + nabla) * dt
        phi += (-v / R_E - eps) * dt
        p += v * dt
    return out


d_nab = simulate(nabla=1e-3)
d_eps = simulate(eps=1e-5)
print('    %-10s %20s %20s' % ('t (s)', '加计零偏: 位置 (m)', '陀螺漂移: 位置 (m)'))
for t in sorted(d_nab):
    print('    %-10g %20.6e %20.6e' % (t, d_nab[t][1], d_eps[t][1]))

ta, tb = 100.0, 200.0
ratio_nab = d_nab[tb][1] / d_nab[ta][1]
ratio_eps = d_eps[tb][1] / d_eps[ta][1]
exp_nab = math.log(ratio_nab) / math.log(tb / ta)
exp_eps = math.log(ratio_eps) / math.log(tb / ta)
print()
print('    时间翻倍（100→200 s）时位置误差的比值：')
print('      加计零偏 %.4f  ⇒ 实测阶数 t^%.3f' % (ratio_nab, exp_nab))
print('      陀螺漂移 %.4f  ⇒ 实测阶数 t^%.3f' % (ratio_eps, exp_eps))
check(abs(exp_nab - 2.0) < 0.02,
      '★ 加计零偏：位置误差按 t^%.3f 增长（理论 t²，比值应 = 4，实测 %.4f）' % (exp_nab, ratio_nab))
check(abs(exp_eps - 3.0) < 0.02,
      '★ 陀螺漂移：位置误差按 t^%.3f 增长（理论 t³，比值应 = 8，实测 %.4f）' % (exp_eps, ratio_eps))
# ★ 反向对照：两者必须【不同阶】—— 否则"陀螺漂移更危险"这句话没有依据
check(exp_eps > exp_nab + 0.5,
      '★ 反向对照：陀螺漂移的阶数(%.2f) 确实【高于】加计零偏(%.2f) ⇒ 「陀螺漂移更危险」有据' % (exp_eps, exp_nab))

print()
print('    ★ 与 `07-仿真与实验/04` 的 lesson34 相互印证：')
print('      lesson34 独立测出「陀螺零偏导致的漂移比值 ≈ 2³」——')
print('      这里用【另一套代码、另一套方程】测出同一个 t³，是交叉验证而非重复。')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
