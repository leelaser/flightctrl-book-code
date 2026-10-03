# -*- coding: utf-8 -*-
"""第 29 周 · 05 避障三层 —— 可执行验证

配套课文：`10-ArduPilot算法全解/05-避障三层.md`
源码：`ardupilot-Copter-4.7.1/libraries/AC_Avoidance/AC_Avoid.cpp`（1584 行）
      `ardupilot-Copter-4.7.1/libraries/AP_Math/control.cpp`（sqrt_controller / inv_sqrt_controller）

验证六件事（全部是【两个独立来源】对撞，不是自证）：

  [1] ★★★ `sqrt_controller` ↔ `inv_sqrt_controller` 互逆（源码里两个独立函数）
  [2] ★★★ `get_max_speed` ↔ `get_stopping_distance` 互逆（AC_Avoid 的"刹车"核心）
  [3] ★★  dt 钳位的临界点：解析预言 dt > 1/p 才咬合 —— 两个方向都验
  [4] ★★  `limit_velocity_NE` 的投影语义：只削法向、不动切向
  [5] ★★  象限合并的真实性质（`find_max_quadrant_velocity`）—— 我猜错了两次
  [6] ★★  SLIDE vs STOP 的几何差异

设计原则（本仓库的教训，见 memory）：
  · 断言两边来源必须不同 —— 这里两边都是【源码里现成的函数】，不是我编的公式
  · 反向对照写进脚本
  · 阈值来自解析推导，不拍脑袋

运行：`python lesson39_avoid.py`   —— 全部断言通过才返回 0
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


# ══════════════════════════════════════════════════════════════
# 源码搬运（逐行照抄，行号标注在注释里）
# ══════════════════════════════════════════════════════════════
def is_zero(x):
    return abs(x) <= 1e-9


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def sqrt_controller(error, p, second_ord_lim, dt):
    """AP_Math/control.cpp:544 —— 分段 sqrt + 线性控制器"""
    if second_ord_lim <= 0:                       # is_negative || is_zero
        rate = error * p
    elif is_zero(p):
        if error > 0:
            rate = math.sqrt(2.0 * second_ord_lim * error)
        elif error < 0:
            rate = -math.sqrt(2.0 * second_ord_lim * (-error))
        else:
            rate = 0.0
    else:
        linear_dist = second_ord_lim / (p * p)    # ★ 线性区/开方区的分界【距离】
        if error > linear_dist:
            rate = math.sqrt(2.0 * second_ord_lim * (error - linear_dist / 2.0))
        elif error < -linear_dist:
            rate = -math.sqrt(2.0 * second_ord_lim * (-error - linear_dist / 2.0))
        else:
            rate = error * p
    if dt > 0:
        return clamp(rate, -abs(error) / dt, abs(error) / dt)   # ★ dt 钳位
    return rate


def inv_sqrt_controller(output, p, D_max):
    """AP_Math/control.cpp:600 —— sqrt_controller 的逆"""
    if D_max > 0 and is_zero(p):
        return (output * output) / (2.0 * D_max)
    if D_max <= 0 and not is_zero(p):
        return output / p
    if D_max <= 0 and is_zero(p):
        return 0.0
    linear_velocity = D_max / p                   # ★ 分界【速度】
    if abs(output) < linear_velocity:
        return output / p
    linear_dist = D_max / (p * p)
    stopping_dist = (linear_dist * 0.5) + (output * output) / (2.0 * D_max)
    return stopping_dist if output > 0 else -stopping_dist


def get_max_speed(kP, accel, distance, dt):
    """AC_Avoidance/AC_Avoid.cpp:763"""
    if is_zero(kP):
        return math.sqrt(2.0 * distance * accel)
    return sqrt_controller(distance, kP, accel, dt)


def get_stopping_distance(kP, accel, speed):
    """AC_Avoidance/AC_Avoid.cpp:1492 —— 注释说 "Implementation copied from AC_PosControl" """
    if accel <= 0.0 or is_zero(speed):
        return 0.0
    if kP <= 0.0:
        return 0.5 * speed * speed / accel        # v² = 2as 的逆
    if speed < accel / kP:
        return speed / kP
    return accel / (2.0 * kP * kP) + (speed * speed) / (2.0 * accel)


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def limit_velocity_NE(kP, accel, vel, limit_direction, limit_distance, dt):
    """AC_Avoidance/AC_Avoid.cpp:585 —— 只削"朝着障碍"的那个分量"""
    max_speed = get_max_speed(kP, accel, limit_distance, dt)
    speed = vdot(vel, limit_direction)            # 投影到限幅方向
    if speed > max_speed:
        # 把超出部分【沿限幅方向】减掉 —— 切向分量一动不动
        return (vel[0] + limit_direction[0] * (max_speed - speed),
                vel[1] + limit_direction[1] * (max_speed - speed))
    return vel


# ══════════════════════════════════════════════════════════════
print('=' * 74)
print('[1] ★★★ sqrt_controller ↔ inv_sqrt_controller（两个源码函数互逆）')
print('=' * 74)
print('    AP_Math/control.cpp:544 vs :600')
print('    注意：inv_sqrt_controller 是【源码里就有的】，不是我反推的 —— 这是独立基准')

# dt 取小值 ⇒ 钳位不咬合（见 [3] 的推导）
DT_SAFE = 0.01
CASES = [(0.5, 1.0, 3.0), (3.0, 1.0, 3.0), (3.0001, 1.0, 3.0), (30.0, 1.0, 3.0),
         (0.1, 2.0, 5.0), (5.0, 2.0, 5.0), (100.0, 2.0, 5.0),
         (2.0, 0.5, 1.5), (20.0, 0.5, 1.5),
         (3.0, 1.0, 0.0)]
print()
print('    %-12s %-8s %-8s %-14s %-14s %s'
      % ('error', 'p', 'A', '正向 rate', '逆向还原', '往返误差'))
worst = 0.0
for e, p, A, in CASES:
    rate = sqrt_controller(e, p, A, DT_SAFE)
    back = inv_sqrt_controller(rate, p, A)
    err = abs(back - e)
    worst = max(worst, err)
    print('    %-12g %-8g %-8g %-14.6f %-14.6f %.3e' % (e, p, A, rate, back, err))
    check(err < 1e-9,
          'error=%-6g p=%-4g A=%-4g ⇒ 还原误差 %.2e < 1e-9' % (e, p, A, err))
check(worst < 1e-9,
      '★★★ 全部 %d 组往返误差最大 %.2e ⇒ 两函数互为逆' % (len(CASES), worst))

# ★★★ 【密集网格】——上面那几组是手挑的，会漏掉"分支切换带"。
#     实测：把 inv_sqrt_controller 的 linear_velocity 从 D_max/p 改成 D_max/p²，
#     手挑的 3 组【全部照样绿】—— 因为没有一组的输出落进那个带宽里。
#     ⇒ 密扫才盖得住分支边界。
print()
dense_worst = 0.0
for p, A in ((1.0, 3.0), (2.0, 5.0), (0.5, 1.5), (4.0, 2.0)):
    L = A / (p * p)
    V = A / p
    # 扫过【分界距离 L 附近的 ±60%】—— 这里正是两个分支交接、最容易写错的地方
    for i in range(1, 1201):
        e = L * 0.4 + i * (L * 1.2 / 1200.0)
        rate = sqrt_controller(e, p, A, DT_SAFE)
        back = inv_sqrt_controller(rate, p, A)
        dense_worst = max(dense_worst, abs(back - e))
    # 再扫一遍【输出速度】在分界速度 V 附近的区间（正向函数走线性支时它会落到这里）
    for i in range(1, 1201):
        out = V * 0.01 + i * (V * 1.98 / 1200.0)
        back = inv_sqrt_controller(out, p, A)
        fwd = sqrt_controller(back, p, A, DT_SAFE)
        dense_worst = max(dense_worst, abs(fwd - out))
check(dense_worst < 1e-9,
      '★★★ 密集扫描（分界 L 附近 ±60%% + 分界速度 V 附近 0.01~1.99 倍，'
      '共 9600 点）最大往返误差 %.2e < 1e-9 ⇒ 分支边界两侧都对得上'
      % dense_worst)

# 分段边界的两侧一致性：正向在 linear_dist 切换，逆向在 linear_velocity 切换，
# 这两个阈值必须【互相印证】
print()
for p, A in ((1.0, 3.0), (2.0, 5.0), (0.5, 1.5)):
    L = A / (p * p)                 # sqrt_controller 的分界【距离】
    V = A / p                       # inv_sqrt_controller 的分界【速度】
    v_at_L = sqrt_controller(L, p, A, DT_SAFE)
    check(abs(v_at_L - V) < 1e-12,
          'p=%-4g A=%-4g：正向分界距离 L=%.4f 处的速度 %.6f == 逆向分界速度 V=%.6f'
          % (p, A, L, v_at_L, V))
    check(abs(L * p - V) < 1e-12,
          '  └ 且 L·p = %.6f = V —— 两个分界是【同一个点】的两种坐标'
          % (L * p))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[2] ★★★ get_max_speed ↔ get_stopping_distance（AC_Avoid 的刹车核心）')
print('=' * 74)
print('    AC_Avoid.cpp:763 vs :1492 —— 一个问"给定距离能多快"，一个问"给定速度要多远"')
print('    kP = 0 时退化成纯运动学：v = sqrt(2·a·s) ⟺ s = v²/(2a)')

print()
print('    %-10s %-8s %-8s %-14s %-16s %s'
      % ('距离(m)', 'kP', 'accel', '算出的 v_max', '反算停止距离', '往返误差'))
worst2 = 0.0
GRID = []
for d in (0.5, 1.0, 2.0, 5.0, 10.0):
    for kP, A in ((0.0, 3.0), (1.0, 3.0), (2.0, 3.0), (1.0, 1.0), (0.5, 5.0)):
        GRID.append((d, kP, A))
for d, kP, A in GRID:
    v = get_max_speed(kP, A, d, DT_SAFE)
    s = get_stopping_distance(kP, A, v)
    err = abs(s - d)
    worst2 = max(worst2, err)
    print('    %-10g %-8g %-8g %-14.6f %-16.6f %.3e' % (d, kP, A, v, s, err))
check(worst2 < 1e-9,
      '★★★ %d 组全部往返一致，最大误差 %.2e ⇒ "能刹住"的预言和"要多少距离"的'
      '计算是自洽的（否则避障会承诺一个刹不住的速度）' % (len(GRID), worst2))

# kP=0 的纯运动学分支单独验（这才是课本里的 v²=2as）
print()
for d, A in ((2.0, 3.0), (5.0, 1.0), (10.0, 9.0)):
    v = get_max_speed(0.0, A, d, DT_SAFE)
    check(abs(v * v - 2.0 * A * d) < 1e-9,
          'kP=0：v² = %.6f，2·a·s = %.6f（差值 %.2e）—— 就是 v²=2as'
          % (v * v, 2.0 * A * d, abs(v * v - 2.0 * A * d)))

# ── 反向对照：把 accel 用错方向会怎样 ──
print()
v_ok = get_max_speed(1.0, 3.0, 5.0, DT_SAFE)
v_bad = get_max_speed(1.0, 30.0, 5.0, DT_SAFE)
check(v_bad > v_ok,
      '★ 反向对照：把 accel 写成 10 倍 ⇒ v_max 从 %.3f 涨到 %.3f。'
      '刹车能力越强，允许越快 —— 方向是对的' % (v_ok, v_bad))
check(abs(get_stopping_distance(1.0, 30.0, v_bad) - 5.0) < 1e-9,
      '★ 而且同样的距离仍然往返一致 ⇒ 两函数不是"碰巧在一个 accel 上对"')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[3] ★★ dt 钳位的临界点：为什么"源码里 p 和 dt 会互相约束"')
print('=' * 74)
print('    sqrt_controller 结尾有一句：')
print('        constrain_float(rate, -|error|/dt, |error|/dt)')
print('    ⇒ 只有当 |rate| > |error|/dt，也就是 dt > |error|/rate 时才咬合。')
print()
print('    推导：开方区 rate = sqrt(2A(e - L/2))，L = A/p²。')
print('          在 e 刚越过 L 时 e/rate 取【最小值】：')
print('          e=L ⇒ rate = sqrt(2A·L/2) = sqrt(A·L) = A/p')
print('          ⇒ (e/rate)_min = L/(A/p) = (A/p²)·(p/A) = 1/p')
print('          e→∞ 时 e/rate→∞。线性区 rate=e·p ⇒ e/rate ≡ 1/p。')
print('    ⇒ ★ 解析预言：dt ≤ 1/p 时钳位【永不咬合】；dt > 1/p 时【存在】e 使其咬合。')
print()
def unclamped(e, p, A):
    """把 dt 取到【极小】让钳位界 |e|/dt 宽到不可能咬合。
       ⚠️ 反过来（取很大的 dt）会把界压成 ~0，得到的是"全被钳位"的值 ——
          我第一版就是这么写的，于是"未咬合"的基准本身就是错的。"""
    return sqrt_controller(e, p, A, 1e-12)


for p, A in ((1.0, 3.0), (2.0, 3.0), (0.5, 5.0)):
    L = A / (p * p)
    thr = 1.0 / p
    print('    p=%.2f  A=%.1f  ⇒ 分界距离 L=A/p²=%.4f，理论临界 dt = 1/p = %.4f s'
          % (p, A, L, thr))

    # ① dt 恰好等于 1/p ⇒ 应当【处处】不咬合
    #    因为 |rate| ≤ |e|/dt  ⟺  dt ≤ e/rate，而 e/rate 的最小值恒为 1/p
    #    （线性区 e/rate ≡ 1/p；开方区 e=L 处也取到 1/p）
    bad = []
    for i in range(1, 2001):
        e = i * 0.05
        if abs(unclamped(e, p, A) - sqrt_controller(e, p, A, thr)) > 1e-9:
            bad.append(e)
    check(not bad,
          '★ dt = 1/p = %.5f（临界值）⇒ 2000 个误差点上【全部】未咬合'
          '（e/rate 的最小值恰好等于 1/p，取等号时不咬）' % thr)

    # ② dt = k/p (k>1) ⇒ 咬合区是【从 0 开始】的一段 [0, e_hi]
    #    解析解：解 e/rate = dt 得 e_hi = L·k·(k + sqrt(k²-1))
    #    ★ 注意咬合从【最小】的 e 就开始 —— 线性区 e/rate ≡ 1/p < dt 恒成立
    k = 1.5
    dt_hi = k / p
    e_hi_pred = L * k * (k + math.sqrt(k * k - 1.0))
    n_pt = 4000
    step = 2.0 * e_hi_pred / n_pt
    clipped = []
    for i in range(1, n_pt + 1):
        e = i * step
        if abs(unclamped(e, p, A) - sqrt_controller(e, p, A, dt_hi)) > 1e-9:
            clipped.append(e)
    check(len(clipped) > 0, '★ dt = %.5f（= 1.5/p）⇒ 确实有误差点被咬合' % dt_hi)
    if clipped:
        e_lo, e_hi = clipped[0], clipped[-1]
        check(e_lo <= step * 1.5,
              '★★ 咬合从 e = %.4f 就开始（≈ 网格下限）⇒ 是【从最小误差起】'
              '整段咬合，不是在 L=%.3f 附近才开始' % (e_lo, L))
        check(abs(e_hi - e_hi_pred) <= step * 1.5,
              '★★★ 咬合【上界】实测 %.4f，解析预言 L·k·(k+√(k²−1)) = %.4f'
              '（差 %.4f ≤ 网格步长 %.4f）⇒ 临界点有闭式解，不是拟合出来的'
              % (e_hi, e_hi_pred, abs(e_hi - e_hi_pred), step))
        check(e_hi < 2.0 * e_hi_pred,
              '  └ 且上界有限：再远就不咬了（%.4f 之外恢复原值）' % e_hi)

# 咬合后往返必然失效 —— 这是"往返精确"那条断言的边界条件，必须写出来
e0, p0, A0 = 3.0, 1.0, 3.0
r_clip = sqrt_controller(e0, p0, A0, 2.0)        # dt=2 > 1/p=1 ⇒ 咬合
back_clip = inv_sqrt_controller(r_clip, p0, A0)
print()
print('    实例：e=%.1f, p=%.1f, A=%.1f' % (e0, p0, A0))
print('      dt=0.01（未咬合）⇒ rate=%.6f，还原 %.6f' % (sqrt_controller(e0, p0, A0, 0.01),
                                                        inv_sqrt_controller(sqrt_controller(e0, p0, A0, 0.01), p0, A0)))
print('      dt=2.00（咬合）  ⇒ rate=%.6f，还原 %.6f' % (r_clip, back_clip))
check(abs(back_clip - e0) > 0.1,
      '★★★ 反例：dt 超过临界后往返【失效】（还原成 %.4f，原值 %.1f）'
      '⇒ [1] 的往返精确是【有前提的】，前提就是 dt ≤ 1/p' % (back_clip, e0))
print('      ⇒ 实践中 Copter 的 dt = 0.004 s（250Hz），而 p 是 PSC_NE_POS_P（量级 1）')
print('        ⇒ dt·p ≈ 0.004 ≪ 1 ⇒ 钳位永远不咬合。但它【不是】无条件成立的。')

print()
print('=' * 74)
print('[4] ★★ limit_velocity_NE 的投影语义（AC_Avoid.cpp:585）')
print('=' * 74)
print('    源码只有四行：把速度投影到限幅方向，超出就【沿该方向】把超出量减掉。')
print('    ⇒ 两个可验的推论：① 限幅后法向分量【恰好】等于 v_max')
print('                       ② 切向分量【一字未动】')
print()
kP, A, d = 1.0, 3.0, 4.0
vmax = get_max_speed(kP, A, d, 0.01)
print('    kP=%.1f accel=%.1f 距离=%.1fm ⇒ v_max = %.6f m/s' % (kP, A, d, vmax))
print()
    # ⚠️ 旧版这里是"算出 clipped → if clipped: 断言 == vmax else: 断言 == 原值"，
    #    ⇒ 【两条必有一条成立】，判据取反了也照样绿（注入实测：整体没红在该红的地方）。
    #    改成：每个用例【先独立写上"该不该削"】，再断言实际行为与它一致。
print('    %-20s %-20s %-12s %-12s %s'
      % ('原速度', '限幅后', '法向分量', '切向分量', '该不该削'))
TESTS = [
    ((1.0, 0.0), (1.0, 0.0), '正对障碍，速度小于 v_max', False),
    ((10.0, 0.0), (1.0, 0.0), '正对障碍，速度远超 v_max', True),
    ((10.0, 5.0), (1.0, 0.0), '斜着撞，法向超了', True),
    ((-10.0, 3.0), (-1.0, 0.0), '反方向（限幅方向是 -x）', True),
    ((2.0, 8.0), (1.0, 0.0), '主要是切向速度', False),
]
for vel, dirv, desc, expect_clip in TESTS:
    n_before = vdot(vel, dirv)
    # 独立算出来的期望：法向分量是否超过 v_max
    check((n_before > vmax) == expect_clip,
          '[前置] %s：法向 %.3f，v_max %.3f ⇒ 按定义应当%s'
          % (desc, n_before, vmax, '削' if expect_clip else '不削'))
    out = limit_velocity_NE(kP, A, vel, dirv, d, 0.01)
    n_after, t_after = vdot(out, dirv), vdot(out, (dirv[1], -dirv[0]))
    t_before = vdot(vel, (dirv[1], -dirv[0]))
    clipped = abs(n_before - n_after) > 1e-12
    print('    %-20s %-20s %-12.5f %-12.5f %s'
          % ('(%.1f,%.1f)' % vel, '(%.4f,%.4f)' % out, n_after, t_after,
             ('削了' if clipped else '没动') + '（期望:' + ('削' if expect_clip else '不削') + '）'))
    check(clipped == expect_clip,
          '  ★ %s：实际%s，与独立算出的期望【一致】'
          % (desc, '削了' if clipped else '没动'))
    check(abs(t_after - t_before) < 1e-9,
          '  %s：切向分量 %.6f → %.6f，【一字未动】' % (desc, t_before, t_after))
    if expect_clip:
        check(abs(n_after - vmax) < 1e-9,
              '  %s：削完的法向分量 %.9f 【恰好】等于 v_max %.9f'
              % (desc, n_after, vmax))
    else:
        check(out == vel, '  %s：法向没超 ⇒ 原样放行（连浮点都没动）' % desc)

# 速度模长只会变小
print()
mono = all(math.hypot(*limit_velocity_NE(kP, A, v, dirv, d, 0.01))
           <= math.hypot(*v) + 1e-12 for v, dirv, _, _ in TESTS)
print('    %-46s %s' % ('限幅只会让速度模长变小（不会凭空加速）', '✓' if mono else '✗'))
check(mono, '★ 反向对照：所有测试用例上，限幅后的速度模长 ≤ 限幅前')

# ── 反向对照：把"只削法向"改成"整个速度清零"会怎样 ──
v = (10.0, 5.0)
out_proj = limit_velocity_NE(kP, A, v, (1.0, 0.0), d, 0.01)
tang = abs(out_proj[1])
check(tang > 1.0,
      '★★ 反向对照：如果实现成"遇到障碍就急停"，切向分量会是 0；'
      '实际实现留下了 %.4f m/s 的切向速度 ⇒ 这就是 BEHAVIOR_SLIDE 的"贴着走"' % tang)

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[5] ★★ 象限合并到底做了什么（AC_Avoid.cpp:718）')
print('=' * 74)
print('    源码注释说它是为了"取到每个象限里的最大速度"。')
print('    ⚠️ 我最初以为它是【防抵消】的（A 要往 +x 退、B 要往 -x 退 ⇒ 别加出一个 0）。')
print('       实测发现**不是** —— 对角线相反的两个请求照样抵消（下面第 3 组）。')
print('       它真正干的是【组内取极值、跨组矢量相加】。')
print()


def find_max_quadrant_velocity(requests):
    """AC_Avoid.cpp:718 原样重写（返回四个象限向量）"""
    q = [[0.0, 0.0] for _ in range(4)]
    for vx, vy in requests:
        if is_zero(vx) or is_zero(vy):
            continue                       # 源码要求两个分量都非零
        if vx > 0 and vy > 0:
            q[0] = [max(q[0][0], vx), max(q[0][1], vy)]
        elif vx < 0 and vy > 0:
            q[1] = [min(q[1][0], vx), max(q[1][1], vy)]
        elif vx < 0 and vy < 0:
            q[2] = [min(q[2][0], vx), min(q[2][1], vy)]
        elif vx > 0 and vy < 0:
            q[3] = [max(q[3][0], vx), min(q[3][1], vy)]
    return q


def quad_sum(requests):
    q = find_max_quadrant_velocity(requests)
    return [sum(v[0] for v in q), sum(v[1] for v in q)]


def naive_sum(requests):
    return [sum(v[0] for v in requests), sum(v[1] for v in requests)]


print('    %-40s %-16s %-16s %s' % ('后退请求集合', '直接相加', '象限合并', '是否相同'))
SCEN = [
    ([(1.0, 1.0), (-1.0, 1.0)], '左右对夹（q1 与 q2）'),
    ([(1.0, 1.0), (1.0, -1.0)], '上下对夹（q1 与 q4）'),
    ([(1.0, 1.0), (-1.0, -1.0)], '对角对夹（q1 与 q3）'),
    ([(1.0, 1.0), (2.0, 3.0)], '同向，都在 q1'),
    ([(1.0, 2.0)], '只有一个请求'),
]
for reqs, desc in SCEN:
    ns, qs = naive_sum(reqs), quad_sum(reqs)
    same = abs(qs[0] - ns[0]) + abs(qs[1] - ns[1]) < 1e-12
    print('    %-40s %-16s %-16s %s'
          % (desc, '(%.1f,%.1f)' % tuple(ns), '(%.1f,%.1f)' % tuple(qs),
             '相同' if same else '★ 象限法更小'))

# ── 真实性质 ①：每个象限里只放一个请求时，结果 == 直接相加 ──
print()
one_each = [(1.0, 1.0), (-1.0, 2.0), (-3.0, -1.0), (2.0, -2.0)]
check(quad_sum(one_each) == naive_sum(one_each),
      '★ 性质①：四个请求恰好各占一个象限 ⇒ 象限法和直接相加【完全相同】'
      ' ⇒ 跨象限这一步就是普通的矢量相加')

# ── 真实性质 ②：同一象限里多个请求 ⇒ 逐轴取极值，【不叠加】 ──
print()
same_q = [(1.0, 1.0), (2.0, 3.0)]
print('    %-34s 直接相加 %-14s 象限法 %-14s'
      % ('两个请求都在第一象限', '(%.1f,%.1f)' % tuple(naive_sum(same_q)),
         '(%.1f,%.1f)' % tuple(quad_sum(same_q))))
check(quad_sum(same_q) == [2.0, 3.0],
      '★★ 性质②：同象限的两个请求 ⇒ 象限法给 (2.0,3.0) = 逐轴最大值；'
      '直接相加会给 (3.0,4.0) ⇒ 【组内取极值】确实挡住了同向叠加')

# ── 真实性质 ③（诚实记录）：对角线相反的请求【照样抵消】 ──
print()
diag = [(1.0, 1.0), (-1.0, -1.0)]
q_diag = quad_sum(diag)
print('    %-34s 直接相加 %-14s 象限法 %-14s'
      % ('对角反向（q1 与 q3）', '(%.1f,%.1f)' % tuple(naive_sum(diag)),
         '(%.1f,%.1f)' % tuple(q_diag)))
check(q_diag == [0.0, 0.0] and naive_sum(diag) == [0.0, 0.0],
      '★★★ 性质③（★ 与直觉相反）：对角相反的两个请求【两种方法都抵消成 (0,0)】'
      '⇒ 象限法**不是**防抵消机制。我最初的假设是错的，实测推翻了它')
print('      ⇒ 那它到底防什么？防的是【同一象限里多个请求叠加成越来越大的速度】。')
print('        跨象限的相反请求仍然会抵消 —— 这是源码的真实行为。')

# ── 真实性质 ④：同一轴的正向请求可以【跨象限累加】超过单个最大值 ──
print()
cross = [(1.0, 1.0), (1.0, -1.0)]          # q1 与 q4，都想要 +x
qs_cross = quad_sum(cross)
print('    %-34s 直接相加 %-14s 象限法 %-14s'
      % ('q1 与 q4 都想要 +x', '(%.1f,%.1f)' % tuple(naive_sum(cross)),
         '(%.1f,%.1f)' % tuple(qs_cross)))
check(qs_cross[0] > max(r[0] for r in cross) + 1e-9,
      '★★★ 性质④：x 分量累加成 %.1f，【超过】任何单个请求的 x=1.0'
      ' ⇒ 象限法出来的后退速度可以比任何单次请求都大' % qs_cross[0])
print('      ⇒ 所以下游那句 limit_length(AVOID_BACKUP_SPD) 是【承重】的，')
print('        不是装饰 —— 它兜住了这个放大。')

print()
print('=' * 74)
print('[6] ★★ SLIDE vs STOP：同一面墙，两种几何')
print('=' * 74)
print('    源码 AC_Avoid.cpp:1440（SLIDE）vs :1458（STOP）')
print('    SLIDE：限幅方向 = 指向【墙上最近点】')
print('    STOP ：限幅方向 = 指向【停止点与墙的交点】，且交点在 margin 内就直接清零')
print()


def slide(vel, pos, wall_x, kP, A, margin, dt):
    """SLIDE：墙是 x = wall_x 的竖直线，车在 pos（x < wall_x）"""
    limit_dir = (1.0, 0.0)                      # 指向墙的最近点
    limit_dist = max(wall_x - pos[0] - margin, 0.0)
    return limit_velocity_NE(kP, A, vel, limit_dir, limit_dist, dt)


def stop(vel, pos, wall_x, kP, A, margin, dt):
    """STOP：沿速度方向投影出"停止点 + margin"，再看这条线段【和墙交在哪】。
       ★ 限幅距离是【到交点的距离】，不是"到停止点的距离"——
         我第一版就写错了这一处，于是 STOP 永远不停。"""
    speed = math.hypot(*vel)
    if speed < 1e-12:
        return vel
    scale = (2.0 + margin + get_stopping_distance(kP, A, speed)) / speed
    stop_pt = (pos[0] + vel[0] * scale, pos[1] + vel[1] * scale)
    if vel[0] <= 0 or stop_pt[0] < wall_x:
        return vel                              # 停止点够不到墙 ⇒ 不管
    # 线段 pos→stop_pt 与竖直线 x=wall_x 的交点
    t = (wall_x - pos[0]) / (stop_pt[0] - pos[0])
    limit_dist = t * math.hypot(stop_pt[0] - pos[0], stop_pt[1] - pos[1])
    if limit_dist <= margin:
        return (0.0, 0.0)                       # 交点在 margin 内 ⇒ 直接停
    limit_dir = (1.0, 0.0)
    return limit_velocity_NE(kP, A, vel, limit_dir, max(limit_dist - margin, 0.0), dt)


kP, A, margin, dt = 1.0, 3.0, 2.0, 0.01
WALL = 10.0
print('    场景：墙在 x=%.0f m，车沿 x 轴以 v 前进，margin=%.1f m，kP=%.1f accel=%.1f'
      % (WALL, margin, kP, A))
print()
print('    %-10s %-10s %-18s %-18s %s'
      % ('距墙(m)', '车速', 'SLIDE 限幅后', 'STOP 限幅后', '说明'))
for pos_x, vx in ((1.0, 5.0), (5.0, 5.0), (9.0, 5.0), (9.9, 5.0), (9.0, 1.0)):
    pos = (pos_x, 0.0)
    vs = slide((vx, 2.0), pos, WALL, kP, A, margin, dt)
    vt = stop((vx, 2.0), pos, WALL, kP, A, margin, dt)
    note = 'STOP 直接停了' if vt == (0.0, 0.0) else ''
    print('    %-10.2f %-10.1f %-18s %-18s %s'
          % (WALL - pos_x, vx, '(%.3f,%.3f)' % vs, '(%.3f,%.3f)' % vt, note))

# 关键差异：SLIDE 永远保留切向速度，STOP 在 margin 内清零
v_slide = slide((5.0, 2.0), (9.9, 0.0), WALL, kP, A, margin, dt)
v_stop = stop((5.0, 2.0), (9.9, 0.0), WALL, kP, A, margin, dt)
check(abs(v_slide[1] - 2.0) < 1e-9,
      '★ SLIDE 在贴着墙时仍保留切向速度 %.3f ⇒ 会顺着墙滑走' % v_slide[1])
check(v_stop == (0.0, 0.0),
      '★ STOP 在 margin 内把速度【整个清零】⇒ 停在原地（连切向也没了）')
check(v_slide != v_stop,
      '★★ 两种行为在同一场景下给出不同的速度 ⇒ 参数 AVOID_BEHAVE 是真的在换算法')

# 远处时两者都不该干预
v_far_s = slide((5.0, 2.0), (1.0, 0.0), WALL, kP, A, margin, dt)
v_far_t = stop((5.0, 2.0), (1.0, 0.0), WALL, kP, A, margin, dt)
print()
check(v_far_t == (5.0, 2.0),
      '★ 反向对照：离墙远（%.0f m）时 STOP 【原样放行】(%.1f,%.1f)'
      % (WALL - 1.0, v_far_t[0], v_far_t[1]))
print('      SLIDE 在远处给出 (%.3f,%.3f) —— 它按"剩余距离"限速，'
      '所以只要距离有限就在限（哪怕限得很松）' % v_far_s)
check(v_far_s[0] > 0,
      '★ SLIDE 仍然允许前进（限到 %.3f，不是 0）' % v_far_s[0])

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
