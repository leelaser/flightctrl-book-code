# -*- coding: utf-8 -*-
"""卷 1 第 6 章 · 捷联惯导姿态解算 —— 圆锥误差补偿的**可执行验证**

配套课文：`卷 1 第 6 章`
源书：严恭敏, 翁浚《捷联惯导算法与组合导航原理》(第 5 刷), §2.6

本脚本做三件事：

  [1] 用**解析的**圆锥运动，量化「单子样朴素积分」到底漂多少度/小时
  [2] 验证教材表 2.6-2/-3 的补偿系数满足恒等式  Σ c_i·k_i = N³/12
  [3] 用该恒等式**独立反解** N=8 的 k₁，并与表中值比对

★ 关于 [3]：教材（第 5 刷）表 2.6-2 里 N=8 的 k₁ 一栏，OCR 出来是 4.083，
  但那一行其余 6 个数以及 N=7/N=9 都自洽，唯独 k₁ 破坏单调性
  （k₁ 序列每步约 +0.693：…4.111 → ? → 5.495…）。
  恒等式给出 4.801，与「4.804」一致、与「4.083」差 0.7。
  ⇒ 记为 **4.804**（OCR 把 4.804 读成了 4.083），本脚本把这次判定固化下来。

运行：`python lesson06_coning.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function

import math
import sys

# ─────────────────────────────────────────────────────────────
# 圆锥运动的解析描述（教材式 2.6-2 / 2.6-5 / 2.6-17）
#
#   四元数   Q(t) = [cos(φ/2), sin(φ/2)cos Ωt, sin(φ/2)sin Ωt, 0]
#   角速度   ω(t) = [−Ω sinφ sinΩt, Ω sinφ cosΩt, −2Ω sin²(φ/2)]
#                       ↑ 这一项是常值，它不是"真转动"，是圆锥误差的来源
#   等效旋转矢量  φ(T) 的 z 分量 = −2 sin²(φ/2) sin(ΩT)
#
# 注意：真值 φ(T) 的 z 分量**不为零**，而朴素积分 Δθ 的 z 分量是 −2sin²(φ/2)·ΩT。
#       两者的差才是要补偿的圆锥误差。
# ─────────────────────────────────────────────────────────────


def sub_increments(N, phi, theta, t_start=0.0):
    """一个更新周期内 N 个子样的角增量 Δθ_j（解析积分，无近似）

    Δθ_j = [ −2 sinφ sin(Δ/2) sin θ_j ,  2 sinφ sin(Δ/2) cos θ_j ,  −2 sin²(φ/2) Δ ]
    其中 Δ = θ/N 是每个子样的转角，θ_j 取该子样区间的**中点**。
    """
    d = theta / N
    out = []
    for j in range(1, N + 1):
        a = t_start + (j - 1) * d          # 区间起点（以 Ωt 计）
        b = t_start + j * d                # 区间终点
        # ∫ −Ω sinφ sin(Ωt) dt = sinφ (cos b − cos a)
        x = math.sin(phi) * (math.cos(b) - math.cos(a))
        # ∫  Ω sinφ cos(Ωt) dt = sinφ (sin b − sin a)
        y = math.sin(phi) * (math.sin(b) - math.sin(a))
        # ∫ −2Ω sin²(φ/2) dt
        z = -2.0 * math.sin(phi / 2.0) ** 2 * d
        out.append((x, y, z))
    return out


def cross_z(a, b):
    """两个角增量叉乘的 z 分量（圆锥误差只出现在锥轴方向）"""
    return a[0] * b[1] - a[1] * b[0]


def coning_drift(N, k, phi, theta):
    """返回**一个更新周期内**残余的 z 轴转角误差（弧度）

      真值   φ_z(T) = −2 sin²(φ/2) sin θ
      朴素   Δθ_z   = −2 sin²(φ/2) θ
      误差   δ_z    = φ_z − Δθ_z = 2 sin²(φ/2)(θ − sin θ)
      补偿   β_z    = Σ_i k_i · (Δθ_{N−i} × Δθ_N)_z
      残差   R      = δ_z − β_z

    ★★ k 的配对方向（本脚本自证时发现的关键坑）：
       k_i 配的是「从**最新**子样往回数第 i 个间隔」，
       即 k_1 配 Δθ_{N−1}×Δθ_N，k_{N−1} 配 Δθ_1×Δθ_N。
       反了的话 N=2 照样通过（只有一项，两种配对等价），
       但 N=3 会差 1.4 倍、N=4 差 11/7 倍 —— **只在 N≥3 才暴露**。
    """
    d = sub_increments(N, phi, theta)
    beta_z = sum(k[i - 1] * cross_z(d[N - 1 - i], d[N - 1])
                 for i in range(1, N))
    delta_z = 2.0 * math.sin(phi / 2.0) ** 2 * (theta - math.sin(theta))
    return delta_z - beta_z


# ─────────────────────────────────────────────────────────────
# 教材表 2.6-3（分数精确解，N=2..6）与表 2.6-2（小数解，N=7,9,10）
#   列序：k_1 对应「最近的一对」，k_{N−1} 对应「最远的一对」
# ─────────────────────────────────────────────────────────────
DECIMAL_N8 = [4.804, 2.495, 2.676, 1.951, 1.510, 0.999, 0.500]

TABLE = {
    2: [2.0 / 3.0],
    3: [27.0 / 20.0, 9.0 / 20.0],
    4: [214.0 / 105.0, 92.0 / 105.0, 54.0 / 105.0],
    5: [1375.0 / 504.0, 650.0 / 504.0, 525.0 / 504.0, 250.0 / 504.0],
    6: [15797.0 / 4620.0, 7834.0 / 4620.0, 7296.0 / 4620.0,
        4558.0 / 4620.0, 2315.0 / 4620.0],
    7: [4.111, 2.097, 2.124, 1.471, 1.004, 0.500],
    8: DECIMAL_N8,
    9: [5.495, 2.891, 3.231, 2.426, 2.018, 1.497, 1.000, 0.500],
    10: [6.178, 3.285, 3.790, 2.898, 2.529, 1.993, 1.501, 1.000, 0.500],
}

PHI = 1.0 * math.pi / 180.0      # 半锥角 φ = 1°
OMEGA = 10.0                     # 圆锥运动角频率 Ω = 10 rad/s
T = 0.01                         # 更新周期 T = 0.01 s（100 Hz）

n_ok = 0
n_bad = 0


def check(cond, msg):
    global n_ok, n_bad
    if cond:
        n_ok += 1
        print('    ✅ %s' % msg)
    else:
        n_bad += 1
        print('    ❌ %s' % msg)


print('=' * 72)
print('[1] 单子样朴素积分到底漂多少？（φ=1°, Ω=10 rad/s, T=0.01 s → 100 Hz）')
print('=' * 72)
theta = OMEGA * T
for N in (1, 2, 4):
    k = TABLE.get(N, [])
    R = abs(coning_drift(N, k, PHI, theta))
    deg_per_h = math.degrees(R / T) * 3600.0
    label = {1: '单子样（不补偿）', 2: '二子样', 4: '四子样'}[N]
    print('    N=%d  %-16s  残余 %9.3e rad/周期  =  %10.3f °/h'
          % (N, label, R, deg_per_h))
print()
print('    ⇒ 教材说「单子样算法不能达到惯性级要求」——上面这一行就是它的量级')

print()
print('=' * 72)
print('[2] 系数表的自检恒等式：Σ c_i·k_i = N³/12   （c_i = 距最新子样的间隔数）')
print('=' * 72)
print('    %-4s %16s %16s %10s' % ('N', 'Σ c_i·k_i（表值）', 'N³/12（理论）', '相对差'))
for N in sorted(TABLE):
    k = TABLE[N]
    s = sum((i + 1) * k[i] for i in range(len(k)))
    theory = N ** 3 / 12.0
    rel = abs(s - theory) / theory
    flag = 'OK' if rel < 5e-4 else 'CHECK'
    print('    %-4d %16.6f %16.6f %9.2e  %s' % (N, s, theory, rel, flag))
    check(rel < 5e-4, 'N=%d 满足恒等式（相对差 %.2e）' % (N, rel))

print()
print('=' * 72)
print('[3] 用恒等式独立反解 N=8 的 k₁')
print('=' * 72)
rest = DECIMAL_N8[1:]
s_rest = sum((i + 2) * rest[i] for i in range(len(rest)))
k1_solved = 8 ** 3 / 12.0 - s_rest
print('    k₂..k₇ 贡献        = %.6f' % s_rest)
print('    N³/12              = %.6f' % (8 ** 3 / 12.0))
print('    ⇒ 反解 k₁          = %.6f' % k1_solved)
print('    教材表 2.6-2 区间  = 4.804（OCR 误读为 4.083）')
check(abs(k1_solved - 4.804) < 0.01,
      '反解值与 4.804 一致（差 %.4f）' % abs(k1_solved - 4.804))
check(abs(k1_solved - 4.083) > 0.5,
      '反解值与 4.083 相差 %.4f（>0.5，确认 OCR 误读）' % abs(k1_solved - 4.083))

print()
print('=' * 72)
print('[4] 反向测试：把 k₁ 故意写错，检查残差是否真的变大')
print('=' * 72)
r_good = abs(coning_drift(8, DECIMAL_N8, PHI, theta))
r_bad = abs(coning_drift(8, [4.083] + DECIMAL_N8[1:], PHI, theta))
print('    k₁=4.804 残差 = %.4e' % r_good)
print('    k₁=4.083 残差 = %.4e' % r_bad)
check(r_bad > 100 * r_good, '错值残差是正确值的 %.0f 倍（>100 倍）' % (r_bad / r_good))

print()
print('=' * 72)
if n_bad == 0:
    print('  ✅ 全部 %d 项断言通过' % n_ok)
    print('=' * 72)
    sys.exit(0)
print('  ❌ %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 72)
sys.exit(1)
