# -*- coding: utf-8 -*-
"""卷 2 第 12 章 · 初始对准与组合导航 —— 双矢量定姿、正交化、杆臂误差

配套课文：`卷 2 第 12 章`
源书：严恭敏, 翁浚《捷联惯导算法与组合导航原理》第 7 章（PDF p190–p219）

本脚本做三件事：

  [1] ★★★ 双矢量定姿：由【重力 + 地磁】构造初始姿态阵
      ★ 反向测试：**不做正交化**的版本，CᵀC 偏离单位阵多少？
  [2] ★★★ 杆臂误差：δv = −ω × δl，算出量级表
      ★ 反向测试：**只在静止时测**，误差恒为零 ⇒ 杆臂标定必须在机动中做
  [3] ★★ 静基座粗对准：从含噪加计+磁力计读数反解初始姿态，看误差多大

坐标系：**N-E-D**（与教程其它部分一致）。⚠️ 严恭敏原书用 E-N-U，见课文 §1。

运行：`python lesson19_alignment.py`   —— 全部断言通过才返回 0
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


# ── 三维向量工具 ─────────────────────────────────────────────
def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return [x / n for x in a]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def matT(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][j] * v[j] for j in range(3)) for i in range(3)]


def C_bn(phi, theta, psi):
    """机体系 → NED"""
    sp, cp = math.sin(phi), math.cos(phi)
    st, ct = math.sin(theta), math.cos(theta)
    ss, cs = math.sin(psi), math.cos(psi)
    return [[ct * cs, sp * st * cs - cp * ss, cp * st * cs + sp * ss],
            [ct * ss, sp * st * ss + cp * cs, cp * st * ss - sp * cs],
            [-st,     sp * ct,                cp * ct]]


def mat_diff_norm(A, B):
    return max(abs(A[i][j] - B[i][j]) for i in range(3) for j in range(3))


# ══════════════════════════════════════════════════════════════
# [1] 双矢量定姿
# ══════════════════════════════════════════════════════════════
def triad_orth(v1, v2):
    """由两个非共线矢量构造三个【正交规范化】矢量（原书式 7.1-7）"""
    a = unit(v1)
    b = unit(cross(v1, v2))
    c = cross(a, b)
    return [a, b, c]        # 三行，构成一个正交阵


def C_from_triad(v1b, v2b, v1n, v2n):
    """双矢量定姿（正交化版）：C_b^n = Tnᵀ · Tb

    推导：triad_orth 返回的 Tb，它的三【行】是三方基在【机体系】下的坐标，
    也就是"机体系 → 三方基"的映射。同理 Tn 是"导航系 → 三方基"。
    故  C_b^n = (导航→三方)⁻¹ · (机身→三方) = Tnᵀ · Tb。

    ⚠️ 写成 matmul(matT(Tb), Tn) 求的是 C_n^b，即【差一个转置】。
    这个错法特别隐蔽：正交阵转置后【仍然满足 CᵀC = I】，
    下面第 [1] 条正交性断言对两个方向都会通过（实测：翻转后照样 [OK]）。
    能分辨方向的只有"零噪声下与真值姿态阵比对"，见紧随其后的 head_ok/head_t。
    """
    Tb = triad_orth(v1b, v2b)
    Tn = triad_orth(v1n, v2n)
    return matmul(matT(Tn), Tb)


def C_naive(v1b, v2b, v1n, v2n):
    """【不做正交化】的朴素做法：直接把三个矢量摞成矩阵"""
    def rows(a, b):
        return [a, b, cross(a, b)]          # ← 没归一化！
    Mb = rows(v1b, v2b)
    Mn = rows(v1n, v2n)
    return matmul(matT(Mn), Mb)


print('=' * 74)
print('[1] ★★★ 双矢量定姿：重力 + 地磁')
print('=' * 74)

TRUE = (3.0 * D2R, 5.0 * D2R, 30.0 * D2R)      # φ, θ, ψ
C_true = C_bn(*TRUE)
g_n = [0.0, 0.0, 1.0]                           # 重力方向（NED，指向下）
m_n = unit([0.5, 0.0, 0.866])                   # 地磁（设磁倾角 60°，指北偏下）

g_b = matvec(matT(C_true), g_n)
m_b = matvec(matT(C_true), m_n)

# 加测量噪声
rnd = random.Random(11)
def noise_vec(v, sigma):
    return [v[i] + rnd.gauss(0.0, sigma) for i in range(3)]

g_b_n = noise_vec(g_b, 0.002)                   # 0.2% 重力测量噪声
m_b_n = noise_vec(m_b, 0.01)                    # 1% 磁测噪声

C_orth = C_from_triad(g_b_n, m_b_n, g_n, m_n)
C_nv = C_naive(g_b_n, m_b_n, g_n, m_n)

print('    真值姿态 φ=%.1f° θ=%.1f° ψ=%.1f°' % tuple(x / D2R for x in TRUE))
print()
print('    正交化版：  ‖CᵀC − I‖∞ = %.3e' % mat_diff_norm(matmul(matT(C_orth), C_orth),
                                                            [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
print('    朴素版  ：  ‖CᵀC − I‖∞ = %.3e' % mat_diff_norm(matmul(matT(C_nv), C_nv),
                                                            [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))

dev_orth = mat_diff_norm(matmul(matT(C_orth), C_orth), [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
dev_nv = mat_diff_norm(matmul(matT(C_nv), C_nv), [[1, 0, 0], [0, 1, 0], [0, 0, 1]])

check(dev_orth < 1e-12,
      '正交化版严格满足 CᵀC = I（偏差 %.2e < 1e-12）' % dev_orth)
# ⚠️ 不报"多少倍" —— 分母是机器精度，比值没有意义（第一版打出了 1e16 这种数字）
check(dev_nv > 1e-3,
      '朴素版【不满足】正交性（偏差 %.2e，而正交化版只有 %.2e）' % (dev_nv, dev_orth))

# ★★★ 上面两条【抓不住转置错误】：正交阵转置后 CᵀC 依然是 I。
#   必须补一条【零噪声 + 对真值绝对比对】，才能把方向钉死。
#   （实测：把 C_from_triad 翻转成 matT(Tb)·Tn 后，上面两条依旧 [OK]。）
C_clean = C_from_triad(g_b, m_b, g_n, m_n)          # 用【无噪声】读数
dev_clean = mat_diff_norm(C_clean, C_true)
dev_T = mat_diff_norm([[C_clean[j][i] for j in range(3)] for i in range(3)], C_true)
check(dev_clean < 1e-12,
      '零噪声下 C 与真值姿态阵一致（偏差 %.2e）⇒ 方向正确，不是转置' % dev_clean)
check(dev_T > 1e-2,
      '而【转置】版与真值差 %.2e ⇒ 证明这条检查确实分得清方向' % dev_T)
print()
print('    ⇒ 原书 §7.1.1：「测量矢量既包含模值误差又包含方向误差，')
print('      使得按式 (7.1-6) 求解的姿态阵【并不能严格满足单位正交化要求】。」')

print()
print('=' * 74)
print('[2] ★★★ 杆臂误差：δv = −ω × δl')
print('=' * 74)
print('    物理场景：IMU 在重心附近，GNSS 天线在机背 —— 两者差几十厘米')
print()
print('    %-12s %-14s %16s %16s' % ('杆臂 (m)', '角速率 (rad/s)', '速度误差 (m/s)', '相当于'))
for l in (0.1, 0.3, 0.5):
    for w in (0.5, 1.0, 3.0):
        dv = l * w                       # |ω × l| = w·l（垂直时）
        tag = 'GPS 噪声量级' if dv < 0.6 else ('明显超噪声' if dv < 1.5 else '★ 主导误差')
        print('    %-12.1f %-14.1f %16.3f %16s' % (l, w, dv, tag))

# 反向测试：静止时测不出来
print()
print('    ★ 反向测试：如果【只在静止时】做杆臂标定会怎样？')
w_static = 0.0
dv_static = 0.5 * w_static
print('      静止时 ω = %.1f ⇒ 杆臂速度误差 = %.3f m/s（【恒为零】）' % (w_static, dv_static))
check(dv_static == 0.0, '静止时杆臂误差恒为 0 ⇒ 静止标定【测不出杆臂】')
dv_move = 0.5 * 1.0
check(dv_move > 0.1, '机动时（ω=1 rad/s）误差 %.2f m/s ⇒ 标定必须在【机动中】做' % dv_move)
print('      ⇒ 这解释了课文 §6 那句：「两者都【只在机动时暴露】」')

print()
print('=' * 74)
print('[3] ★★ 静基座粗对准：从含噪读数反解初始姿态')
print('=' * 74)
print('    真值 φ=%.1f° θ=%.1f° ψ=%.1f°' % tuple(x / D2R for x in TRUE))
print()
print('    %-10s %12s %12s %12s %14s' % ('噪声水平', 'Δφ(°)', 'Δθ(°)', 'Δψ(°)', '姿态阵偏差'))
for nlevel in (0.0, 0.005, 0.02, 0.05):
    r = random.Random(3)
    errs = []
    for trial in range(20):
        gb = [g_b[i] + r.gauss(0, nlevel) for i in range(3)]
        mb = [m_b[i] + r.gauss(0, nlevel) for i in range(3)]
        C = C_from_triad(gb, mb, g_n, m_n)
        # 从姿态阵反解欧拉角（ZYX）
        th = math.asin(-C[2][0])
        ph = math.atan2(C[2][1], C[2][2])
        ps = math.atan2(C[1][0], C[0][0])
        errs.append((abs(ph - TRUE[0]), abs(th - TRUE[1]), abs(ps - TRUE[2]),
                     mat_diff_norm(C, C_true)))
    n = len(errs)
    a = [sum(e[i] for e in errs) / n for i in range(4)]
    print('    %-10.3f %12.3f %12.3f %12.3f %14.2e'
          % (nlevel, a[0] / D2R, a[1] / D2R, a[2] / D2R, a[3]))

# 判据：噪声越大，姿态误差越大（单调）
r = random.Random(3)
res = []
for nlevel in (0.0, 0.005, 0.02, 0.05):
    e_tot = 0.0
    for trial in range(40):
        gb = [g_b[i] + r.gauss(0, nlevel) for i in range(3)]
        mb = [m_b[i] + r.gauss(0, nlevel) for i in range(3)]
        C = C_from_triad(gb, mb, g_n, m_n)
        e_tot += mat_diff_norm(C, C_true)
    res.append(e_tot / 40)
print()
check(all(res[i] < res[i + 1] for i in range(len(res) - 1)),
      '姿态误差噪声水平单调递增（%s）⇒ 定姿精度直接由传感器噪声决定'
      % ', '.join('%.1e' % x for x in res))
print('    ⇒ ★ 这就是课文 §7.3 说的：「磁力计没校准 ⇒ 初始航向就是错的」。')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
