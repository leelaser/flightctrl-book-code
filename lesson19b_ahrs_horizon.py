# -*- coding: utf-8 -*-
"""卷 2 第 12 章 · 初始对准与组合导航 —— 补充：低成本 AHRS 与地平仪（严恭敏 §7.5）

配套课文：`卷 2 第 12 章` §8
源书：严恭敏, 翁浚《捷联惯导算法与组合导航原理》第 7 章 §7.5（PDF p211–p219）

验证五件事：

  [1] §8.1 为什么 MEMS 必须砍掉哥氏项（信噪比估算）
  [2] §8.2 ★★★ 磁力计【沿磁场方向】的失准角不可观测
  [3] §8.3 加计水平对准的构造法（含 |C₃ₓ| 判据的退化情形）
  [4] §8.4 ★★★ 水平加速机动 = 人造磁北（用两套独立物理场景解出同一方位）
  [5] §8.6 |f|−g 判据太宽松：β=0.5 ⇒ 漏判 3.2 m/s²

设计原则（本仓库的教训，见 memory）：
  · 断言两边来源必须不同：几何事实 / 解析式 / 两套独立仿真
  · 反向对照写进脚本：错的判据、退化的构造法必须被测出来
  · 阈值来自解析预言或恒等式，不拍脑袋

运行：`python lesson19b_ahrs_horizon.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function

import math
import random
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


D2R = math.pi / 180.0
G0 = 9.80665
OMEGA_IE = 7.292115e-5


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return [x / n for x in a]


def matT(A):
    return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]


def matmul3(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][j] * v[j] for j in range(len(v))) for i in range(len(A))]


def C_enu(phi, theta, psi):
    """机体系 -> 当地水平系（严恭敏用 E-N-U：x 东、y 北、z 天）"""
    sp, cp = math.sin(phi), math.cos(phi)
    st, ct = math.sin(theta), math.cos(theta)
    ss, cs = math.sin(psi), math.cos(psi)
    return [[ct * cs, sp * st * cs - cp * ss, cp * st * cs + sp * ss],
            [ct * ss, sp * st * ss + cp * cs, cp * st * ss - sp * cs],
            [-st,     sp * ct,                cp * ct]]


print('=' * 74)
print('[1] §8.1 为什么 MEMS 必须砍掉哥氏项（信噪比）')
print('=' * 74)
gyro_bias_dps = 0.1                       # 书中给的 MEMS 陀螺零偏量级 (°)/s
wie_dps = OMEGA_IE / D2R * 3600.0 / 3600.0   # (°)/s = 0.004178
print('    地球自转        ωie = %.6f (°)/s = 15.041 (°)/h' % wie_dps)
print('    MEMS 陀螺零偏   bias = %.3f (°)/s = %.0f (°)/h' % (gyro_bias_dps, gyro_bias_dps * 3600))
snr = wie_dps / gyro_bias_dps
print('    信噪比 ωie/bias = 1 / %.1f' % (1.0 / snr))
check(abs(1.0 / snr - 24.0) < 2.0,
      '★ 自转信号被零偏淹没约 %.0f 倍 ⇒ 不是「精度差一点」，是【根本测不到】' % (1.0 / snr))
check(gyro_bias_dps > 20 * wie_dps,
      '⇒ 所以哥氏项可以整项丢弃（书中 (7.5-3)），代价是【方位不能自主对准】')

print()
print('=' * 74)
print('[2] §8.2 ★★★ 磁力计沿磁场方向的失准角【不可观测】')
print('=' * 74)
Hm = unit([0.6, 0.75, 0.3])                # 归一化地磁矢量（m 系）
PHI = [0.02, -0.015, 0.03]                 # 某个失准角


def Z_of_phi(phi):
    """(7.5-13)  Z_m = H^m × φ  （忽略安装角项）"""
    return cross(Hm, phi)


z0 = Z_of_phi(PHI)
print('    Z(φ) = H×φ，取 φ = %s' % ['%+.4f' % x for x in PHI])
print('    Z = %s' % ['%+.6f' % x for x in z0])
print()
print('    ★ 沿 H 方向平移 φ（φ → φ + αH），看 Z 变不变：')
worst = 0.0
for alpha in (-1.0, -0.1, 0.1, 0.5, 2.0):
    phi2 = [PHI[i] + alpha * Hm[i] for i in range(3)]
    z2 = Z_of_phi(phi2)
    d = max(abs(z2[i] - z0[i]) for i in range(3))
    worst = max(worst, d)
    print('      α=%+.2f : Z = %s   与 α=0 的差 = %.2e'
          % (alpha, ['%+.6f' % x for x in z2], d))
check(worst < 1e-15,
      '★ 把 φ 沿 H 平移任意距离，量测 Z 【完全不变】（最大差 %.2e）⇒ 该分量不可观测' % worst)

# ★ 反向对照：垂直于 H 的方向平移 φ，Z 必须【变】
Hperp = unit(cross(Hm, [1.0, 0.0, 0.0]))
phi_t = [PHI[i] + 0.5 * Hperp[i] for i in range(3)]
z_t = Z_of_phi(phi_t)
d_perp = max(abs(z_t[i] - z0[i]) for i in range(3))
check(d_perp > 0.1,
      '★ 反向对照：沿【垂直】H 的方向平移 φ，Z 明显改变（%.4f）⇒ 那两个分量可观测' % d_perp)

# 秩：φ ↦ H×φ 的零空间维数 = 1
# 用三个正交基矢验证：只有沿 H 的那个落进零空间
e_along = Z_of_phi(Hm)
check(norm(e_along) < 1e-15,
      '★ H ↦ H×H = 0 ⇒ 零空间恰由 H 张成（维数 1）⇒ 可观测量只有 2 个自由度')
print('    ⇒ 这就是课文 §8.2 结论①：单靠磁力计定不了完整姿态，第三个自由度要靠重力')

print()
print('=' * 74)
print('[3] §8.3 加计水平对准的构造法')
print('=' * 74)


def build_level(fb):
    """书中 (1)(2)(3)：由比力构造当地水平姿态阵 C_b^h"""
    C3 = unit(fb)
    if abs(C3[0]) > 0.5:
        C1 = [C3[1], -C3[0], 0.0]
    else:
        C1 = [0.0, C3[2], -C3[1]]
    n1 = norm(C1)
    if n1 < 1e-12:
        return None                      # 退化
    C1 = [x / n1 for x in C1]
    C2 = cross(C3, C1)
    return [C1, C2, C3]


def build_level_naive(fb):
    """【错的】构造法：不看 |C₃ₓ|，一律用 [C₃ᵧ, −C₃ₓ, 0]"""
    C3 = unit(fb)
    C1 = [C3[1], -C3[0], 0.0]
    n1 = norm(C1)
    if n1 < 1e-12:
        return None
    C1 = [x / n1 for x in C1]
    C2 = cross(C3, C1)
    return [C1, C2, C3]


rnd = random.Random(2026)
n_case = 0
worst_orth = 0.0
min_branch_norm = 1e9
for trial in range(400):
    fb = unit([rnd.gauss(0, 1) for _ in range(3)])
    C3 = unit(fb)
    if abs(C3[0]) > 0.5:
        pre = [C3[1], -C3[0], 0.0]         # 书中分支 1
    else:
        pre = [0.0, C3[2], -C3[1]]         # 书中分支 2
    min_branch_norm = min(min_branch_norm, norm(pre))   # 归一化【之前】的模长
    M = build_level(fb)
    if M is None:
        continue
    n_case += 1
    for i in range(3):
        for j in range(3):
            worst_orth = max(worst_orth, abs(dot(M[i], M[j]) - (1.0 if i == j else 0.0)))
    worst_orth = max(worst_orth, max(abs(M[2][i] - C3[i]) for i in range(3)))

print('    随机 %d 个重力方向：' % n_case)
check(worst_orth < 1e-12,
      '★ 构造出的姿态阵行向量正交规范、且第 3 行 = 归一化比力（最大偏差 %.2e）' % worst_orth)
# ★ 那 0.5 判据到底在保证什么：无论重力指向哪里，被选中的分支都【不会退化】。
#   分支1 模长 = √(C₃ₓ²+C₃ᵧ²) ≥ |C₃ₓ| > 0.5；分支2 模长 = √(1−C₃ₓ²) ≥ √0.75 ≈ 0.866
check(min_branch_norm > 0.5 - 1e-12,
      '★ 判据保证：被选中的分支模长始终 ≥ %.4f（> 0.5，永不为零向量）' % min_branch_norm)
print('      分支1 模长 ≥ |C₃ₓ| > 0.5；分支2 模长 = √(1−C₃ₓ²) ≥ √0.75 = %.4f'
      % math.sqrt(0.75))

# ★ 反向对照：不加判据、一律用分支 1，则在【重力沿机体系 z 轴】时退化
#   （⚠️ 我第一版在 C₃=[1,0,0] 处测，那里一点也不退化 —— 测错了方向）
C3z = [0.0, 0.0, 1.0]
check(build_level(C3z) is not None,
      '重力沿 +z（C₃=[0,0,1]，即飞机平放）时，带判据的构造法照常给出正交阵')
check(build_level_naive(C3z) is None,
      '★ 反向对照：同一方向上朴素构造法【直接退化】（[C₃ᵧ,−C₃ₓ,0] = 零向量）⇒ 判据确有用处')
check(1.0 / math.sqrt(3.0) > 0.5,
      '阈值取 0.5 的依据：单位向量最大分量必 ≥ 1/√3 = %.4f > 0.5 ⇒ 判据总能选到非退化分支'
      % (1.0 / math.sqrt(3.0)))

print()
print('=' * 74)
print('[4] §8.4 ★★★ 水平加速机动 = 人造磁北（两套独立场景解同一方位）')
print('=' * 74)
# ⚠️ 注意约定：本脚本的 C_enu 用 x=东、y=北、z=天，
#    所以 ψ 是【自东量起】的转角，不等于「方位角」。
#    ⇒ 测试不能拿 ψ 当期望值（我第一版就是这么错的，结果差恰好 90°）。
#    正确做法：从【水平系 C_b^h】出发，用一个【已知的方位修正角 α】构造真值，看能否解回 α。
PITCH, ROLL = 4.0 * D2R, -6.0 * D2R
C_true = C_enu(ROLL, PITCH, 0.0)     # 仅用来造重力方向（ψ 取 0 无所谓）
fb = matvec(matT(C_true), [0.0, 0.0, -G0])          # 重力(向下)在机体系
Cbh = build_level([-fb[i] for i in range(3)])       # C_b^h：第 3 行 = 归一化 f^b（指向"天"）

ALPHA = 35.0 * D2R          # 待求的方位修正角（水平系 → 导航系）


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def solve_azimuth(v_h, v_n):
    """(7.5-25)/(7.5-30) 的共同形式：由同一个矢量在 h 系与 n 系的投影解方位角。
       只用 X、Y 分量。⚠️ 两个向量必须【都】先在各自系里表示好。"""
    D = v_h[0] ** 2 + v_h[1] ** 2
    num_s = v_h[0] * v_n[1] - v_h[1] * v_n[0]
    num_c = v_h[0] * v_n[0] + v_h[1] * v_n[1]
    return math.atan2(num_s / D, num_c / D)


# —— 场景 A：由【地磁】定方位 ——
DECL, DIP = 3.0 * D2R, 55.0 * D2R           # 磁偏角、磁倾角
H_n = [math.cos(DIP) * math.cos(DECL), math.cos(DIP) * math.sin(DECL), -math.sin(DIP)]
H_b = matvec(matT(matmul3(Rz(ALPHA), Cbh)), H_n)     # 真值姿态 C_b^n = Rz(α)·C_b^h
H_h = matvec(Cbh, H_b)                               # 磁强计读数投到水平系
psi_mag = solve_azimuth(H_h, H_n)

# —— 场景 B：由【水平加速度机动】定方位（完全不同的物理量）——
a_hn = [1.8, 1.2, 0.0]                      # 导航系里的水平加速度（东 1.8、北 1.2）
f_n = [a_hn[0], a_hn[1], G0]                # 比力 = 加速度 − g（含重力）
f_b = matvec(matT(matmul3(Rz(ALPHA), Cbh)), f_n)
f_h = matvec(Cbh, f_b)
psi_acc = solve_azimuth(f_h, f_n)

print('    待求的方位修正角 α           = %+.4f°' % (ALPHA / D2R))
print('    ① 由【地磁】解出             = %+.4f°  （磁偏角 %.1f°、磁倾角 %.1f°）'
      % (psi_mag / D2R, DECL / D2R, DIP / D2R))
print('    ② 由【水平加速度矢量】解出   = %+.4f°' % (psi_acc / D2R))
check(abs(psi_mag - ALPHA) < 1e-9,
      '★ 场景①（地磁）：解回同一个 α（差 %.2e°）' % (abs(psi_mag - ALPHA) / D2R))
check(abs(psi_acc - ALPHA) < 1e-9,
      '★★★ 场景②（水平加速度，完全不同的物理量）：解出【同一个】α（差 %.2e°）'
      % (abs(psi_acc - ALPHA) / D2R))
check(abs(psi_mag - psi_acc) < 1e-12,
      '★ 两条独立路径给出的方位角【逐位一致】（差 %.2e°）' % (abs(psi_mag - psi_acc) / D2R))
print('    ⇒ ★ 这两套仿真【没有共用任何物理量】—— 一个用地磁、一个用加速度，')
print('      解的却是同一个方位角。这就是课文 §8.4「人造磁北」那句话的数值证据。')

# ★ 反向对照：纯竖直加速度 ⇒ 水平分量为零 ⇒ 方位无从谈起
f_n_vert = [0.0, 0.0, G0 + 3.0]
f_h_vert = matvec(Cbh, matvec(matT(matmul3(Rz(ALPHA), Cbh)), f_n_vert))
# 扣掉重力（水平系里重力是 [0,0,-g]，比力的竖直分量 = g + a_U）
horiz = math.sqrt(f_h_vert[0] ** 2 + f_h_vert[1] ** 2)
check(horiz < 1e-12,
      '★ 反向对照：纯【竖直】机动在水平系内的分量 = %.1e ⇒ 对方位【零信息】' % horiz)

print()
print('=' * 74)
print('[5] §8.6 |f|−g 判据太宽松：书中那个 β=0.5 的反例')
print('=' * 74)
BETA = 0.5
# 书中：水平加速度 a_H 使 |f| = sqrt(g² + a_H²)；判据 | |f| − g | < β 会被满足到多大 a_H？
a_max = math.sqrt((G0 + BETA) ** 2 - G0 ** 2)
print('    判据 | |f| − g | < β，取 β = %.1f m/s²' % BETA)
print('    水平加速度 a_H 使 |f| = √(g²+a_H²)；令 √((g+β)²−g²) = %.3f m/s²' % a_max)
print('    ⇒ 即：水平加速度高达 %.1f m/s² 时，模值判据仍然"通过"' % a_max)
check(abs(a_max - 3.17) < 0.05,
      '★ 复现书中反例：β=0.5 m/s² 的模值判据会漏过 %.2f m/s² 的水平加速度（书中写 3.2）' % a_max)
check(a_max / BETA > 6.0,
      '★ 漏判量是设定阈值 β 的 %.1f 倍 ⇒ "过于宽松"不是形容词，是可量化的' % (a_max / BETA))
# 解析解释：模值对水平加速度是【二阶】敏感。
#   √(g²+a²) − g = a²/(√(g²+a²)+g)   ← 这是【恒等式】（分子有理化），可精确验证
#   ⇒ Δ|f|/a = a/(√(g²+a²)+g) → a/(2g) 当 a≪g
ratio = (math.sqrt(G0 ** 2 + a_max ** 2) - G0) / a_max
identity = a_max / (math.sqrt(G0 ** 2 + a_max ** 2) + G0)
first_order = a_max / (2 * G0)
print('    原因：Δ|f| / a_H = %.6f' % ratio)
print('      恒等式 a/(√(g²+a²)+g) = %.6f  （差 %.2e）' % (identity, abs(ratio - identity)))
print('      一阶近似 a/(2g)        = %.6f  （差 %.2f%%，即 O(a²/g²) 修正）'
      % (first_order, 100 * abs(ratio - first_order) / ratio))
check(abs(ratio - identity) < 1e-15,
      '⇒ 模值对水平加速度是【二阶】敏感：Δ|f|/a 恰等于 a/(√(g²+a²)+g)（恒等式，差 %.1e）'
      % abs(ratio - identity))
# ⚠️ 第一版我把这条写成"与一阶近似相符到 1e-4" —— 那要求太紧：a/g≈0.32，
#    O(a²/g²) 修正本来就有约 2.5%，不是误差。
check(abs(ratio - first_order) / ratio < 0.05,
      '一阶近似 a/(2g) 与精确值差 %.2f%%（O(a²/g²) 量级，符合预期）'
      % (100 * abs(ratio - first_order) / ratio))

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
