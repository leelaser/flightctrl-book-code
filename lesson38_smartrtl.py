# -*- coding: utf-8 -*-
"""第 28 周 · 04 返航与备降 —— 可执行验证

配套课文：`10-ArduPilot算法全解/04-返航与备降.md`
源码：`ardupilot-Plane-4.7.1/libraries/AP_SmartRTL/`（AP_SmartRTL.cpp 914 + .h 246）
      `ardupilot-Plane-4.7.1/libraries/AP_Rally/`（AP_Rally.cpp 209 + .h 110）

验证四件事：

  [1] 抽稀：add_point() 的 SRTL_ACCURACY 门槛
  [2] ★★★ Douglas-Peucker：递归版 vs 时间片迭代版【必须逐点等价】
  [3] ★★★ 剪枝不变式：PRUNING_DELTA < ACCURACY 且必须先抽稀
  [4] Rally：最近点选择与 RALLY_LIMIT_KM 的"防串场"回退

设计原则（本仓库的教训，见 memory）：
  · 断言两边来源必须不同：递归实现 / 迭代实现 / 恒等式
  · 反向对照写进脚本（含【故意写错】的版本）
  · 阈值来自源码常量或恒等式，不拍脑袋

运行：`python lesson38_smartrtl.py`   —— 全部断言通过才返回 0
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


# ══════════════════════════════════════════════════════════════
# 几何工具（二维；源码是三维，结论不受影响）
# ══════════════════════════════════════════════════════════════
def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_seg_dist(p, a, b):
    """点到线段的距离（AP_Math 的 distance_to_segment 的二维版）"""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-18:
        return dist(p, a)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def seg_seg_dist(a1, a2, b1, b2, samples=2000):
    """两线段最近距离 —— 数值采样。
       源码的 segment_segment_dist()（:833）是解析解；
       这里【故意不照抄源码实现】，用作独立基准。"""
    best = 1e18
    for i in range(samples + 1):
        t = i / float(samples)
        p = (a1[0] + t * (a2[0] - a1[0]), a1[1] + t * (a2[1] - a1[1]))
        d = point_seg_dist(p, b1, b2)
        if d < best:
            best = d
    return best


# ══════════════════════════════════════════════════════════════
print('=' * 74)
print('[1] 抽稀：add_point() 的 SRTL_ACCURACY 门槛（AP_SmartRTL.cpp:327）')
print('=' * 74)

# 源码 AP_SmartRTL.h:
#   SMARTRTL_ACCURACY_DEFAULT        2.0f
#   SMARTRTL_SIMPLIFY_EPSILON        (_accuracy * 0.5f)
#   SMARTRTL_PRUNING_DELTA           (_accuracy * 0.99f)   ← 注意不是独立常量
SRTL_ACCURACY_DEFAULT = 2.0
print('    SMARTRTL_ACCURACY_DEFAULT = %.1f m（AP_SmartRTL.h:9）' % SRTL_ACCURACY_DEFAULT)


def add_points(traj, accuracy):
    """源码 :327-354 原样重写（去掉信号量与容量检查）"""
    path = []
    for p in traj:
        if path:
            last = path[-1]
            if (last[0] - p[0]) ** 2 + (last[1] - p[1]) ** 2 < accuracy ** 2:
                continue                      # 离上一点太近 ⇒ 不存
        path.append(p)
    return path


# 一条直线轨迹，每 0.5 m 采一个点，走 100 m
LINE = [(i * 0.5, 0.0) for i in range(201)]
print()
print('    直线轨迹：%d 个原始采样点（间隔 0.5 m，共 100 m）' % len(LINE))
print()
print('    %-16s %-14s %s' % ('SRTL_ACCURACY', '存下的点数', '解析预期'))
for acc in (0.5, 1.0, 2.0, 5.0, 10.0):
    path = add_points(LINE, acc)
    # 恒等式：点数 = 总长/accuracy + 1（直线均匀采样的精确值）
    expect = int(100.0 / acc) + 1
    print('    %-16g %-14d %d' % (acc, len(path), expect))
    check(len(path) == expect,
          'accuracy=%-5g ⇒ 存下 %d 点，恒等式 总长/accuracy+1 = %d（精确相等）'
          % (acc, len(path), expect))
check(len(add_points(LINE, 2.0)) < len(add_points(LINE, 0.5)),
      '★ 反向对照：accuracy 越大存得越少（2.0 得 %d 点 < 0.5 得 %d 点）'
      % (len(add_points(LINE, 2.0)), len(add_points(LINE, 0.5))))

# ── 抽稀后的最小间距：这是后面剪枝不变式的前提 ──
DECIMATED_LINE = add_points(LINE, SRTL_ACCURACY_DEFAULT)
gaps = [dist(DECIMATED_LINE[k], DECIMATED_LINE[k + 1]) for k in range(len(DECIMATED_LINE) - 1)]
print()
print('    抽稀后相邻点间距：最小 %.4f m，最大 %.4f m（共 %d 个间隔）'
      % (min(gaps), max(gaps), len(gaps)))
check(min(gaps) >= SRTL_ACCURACY_DEFAULT - 1e-9,
      '★ 抽稀【保证】相邻点间距 ≥ ACCURACY = %.1f m（实测最小 %.4f m）'
      % (SRTL_ACCURACY_DEFAULT, min(gaps)))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[2] ★★★ Douglas-Peucker：递归版 vs 时间片迭代版（AP_SmartRTL.cpp:500）')
print('=' * 74)
EPS = SRTL_ACCURACY_DEFAULT * 0.5          # SMARTRTL_SIMPLIFY_EPSILON


def dp_recursive(pts, epsilon):
    """教科书版：递归。返回 (保留的索引集合, 展开的节点数)"""
    keep = set([0, len(pts) - 1])
    nodes = [0]

    def rec(s, e):
        nodes[0] += 1
        if e - s < 2:
            return
        dmax, idx = 0.0, s
        for i in range(s + 1, e):
            d = point_seg_dist(pts[i], pts[s], pts[e])
            if d > dmax:
                dmax, idx = d, i
        if dmax > epsilon:
            keep.add(idx)
            rec(s, idx)
            rec(idx, e)
    rec(0, len(pts) - 1)
    return keep, nodes[0]


def dp_timesliced(pts, epsilon, budget_segs):
    """源码版：显式栈 + 时间片。

       ★★ 关键：源码的时间片检查在 while 顶部（AP_SmartRTL.cpp:521），
       不在内层 for 里 —— 一个栈段的【完整扫描是不可分割的工作单元】。
       一次扫描最多 SMARTRTL_POINTS_MAX(500) 次距离计算，是有界的。

       budget_segs 模拟"每个时间片最多展开几个栈段"。
       返回 (保留索引集合, 用了几个时间片)"""
    keep = set([0, len(pts) - 1])
    if len(pts) < 3:
        return keep, 0
    stack = [(0, len(pts) - 1)]
    slices = 0
    while stack:                       # ← 时间片：栈【是成员变量】，跨调用存活
        slices += 1
        work = budget_segs
        while stack and work > 0:
            s, e = stack.pop()
            work -= 1
            dmax, idx = 0.0, s
            for i in range(s + 1, e):
                d = point_seg_dist(pts[i], pts[s], pts[e])
                if d > dmax:
                    dmax, idx = d, i
            if dmax > epsilon:
                keep.add(idx)                  # ★ 分割点保留 —— 漏了这行结果就只剩首尾
                stack.append((s, idx))
                stack.append((idx, e))
    return keep, slices


def dp_naive_broken(pts, epsilon, budget_units, max_slices=20000):
    """❌ 故意写错的版本：把时间片检查放进内层 for，且【不保存扫描进度】。
       对照用 —— 验证它确实会活锁。"""
    keep = set([0, len(pts) - 1])
    stack = [(0, len(pts) - 1)]
    slices = 0
    while stack:
        slices += 1
        if slices > max_slices:
            return keep, slices, True          # 活锁被 max_slices 截断
        work = budget_units
        while stack and work > 0:
            s, e = stack.pop()
            dmax, idx = 0.0, s
            killed = False
            for i in range(s + 1, e):
                work -= 1
                if work < 0:
                    stack.append((s, e))        # ★ 原样压回，进度丢失
                    killed = True
                    break
                d = point_seg_dist(pts[i], pts[s], pts[e])
                if d > dmax:
                    dmax, idx = d, i
            if killed:
                break
            if dmax > epsilon:
                stack.append((s, idx))
                stack.append((idx, e))
    return keep, slices, False


def max_deviation(pts, keep):
    """★ 独立基准：DP 的正确性保证 ——
       每个【被删掉】的点，到【最终折线】的距离必须 ≤ epsilon。
       这个量不复算"保留了哪些点"，只检验结果必须满足的性质。"""
    ks = sorted(keep)
    worst = 0.0
    for a, b in zip(ks, ks[1:]):
        for i in range(a + 1, b):
            d = point_seg_dist(pts[i], pts[a], pts[b])
            if d > worst:
                worst = d
    return worst


def make_structured_path():
    """有结构的路径：两段直线 + 一个直角折点 + 一个缓弯"""
    pts = []
    for i in range(60):
        pts.append((i * 0.5, 0.0))
    for i in range(1, 40):
        pts.append((30.0, i * 0.5))
    for i in range(1, 60):
        pts.append((30.0 + i * 0.5, 20.0))
    for i in range(1, 50):
        ang = math.radians(i * 1.8)
        pts.append((60.0 + 20 * math.sin(ang), 20.0 + 20 * (1 - math.cos(ang))))
    return pts


def make_noisy_path(n=400, noise=2.0):
    """带噪声的长路径：让 DP 递归树足够"茂密"，用来看时间片的作用"""
    rng = random.Random(7)
    pts = []
    for i in range(n):
        pts.append((i * 1.0 + rng.uniform(-noise, noise),
                    rng.uniform(-noise, noise)))
    return pts


PATH = make_structured_path()
print()
print('    路径 A：%d 个点（两段直线 + 一个直角折点 + 一个缓弯）' % len(PATH))
print('    epsilon = %.1f m（= ACCURACY × 0.5）' % EPS)

k_rec, nodes_rec = dp_recursive(PATH, EPS)
print()
print('    递归版   保留 %d 点，展开 %d 个节点' % (len(k_rec), nodes_rec))
check(len(k_rec) < len(PATH) / 2,
      '★ 简化确实压缩了路径：%d → %d 点（压掉 %.0f%%）'
      % (len(PATH), len(k_rec), 100 * (1 - len(k_rec) / float(len(PATH)))))
check(0 in k_rec and (len(PATH) - 1) in k_rec,
      '★ 首尾点永远保留（简化只删中间点，不改变起终点）')
# ★★ 压缩率是【单边】的：压得越狠越"通过"，压成 2 个点也通过。
#    必须再加一条正确性保证，否则"压坏了"和"压得好"同样绿。
dev_rec = max_deviation(PATH, k_rec)
check(dev_rec <= EPS + 1e-9,
      '★★ DP 正确性保证：被删掉的点离最终折线最远 %.4f m ≤ epsilon %.1f m '
      '（这条挡住"压过头"——压成 2 个点时它会到 %.1f m）'
      % (dev_rec, EPS, max_deviation(PATH, set([0, len(PATH) - 1]))))

print()
print('    %-14s %-12s %-12s %s' % ('每片栈段数', '保留点数', '用的片数', '与递归版一致'))
for budget in (10 ** 9, 50, 10, 3, 1):
    k_it, slices = dp_timesliced(PATH, EPS, budget)
    same = (k_it == k_rec)
    label = '无限' if budget > 10 ** 8 else str(budget)
    print('    %-14s %-12d %-12d %s' % (label, len(k_it), slices, '✓' if same else '✗'))
    check(same, '★ 每片 %-8s 个栈段 ⇒ 迭代版与递归版保留【同一组点】（%d 点）'
          % (label, len(k_it)))
    # 每个预算下，结果都要满足 DP 的正确性保证（不只是"和递归版一样"）
    check(max_deviation(PATH, k_it) <= EPS + 1e-9,
          '  └ 该预算下 DP 保证仍成立：最大偏差 %.4f m ≤ %.1f m'
          % (max_deviation(PATH, k_it), EPS))
check(dp_timesliced(PATH, EPS, 10 ** 6)[1] == 1,
      '★ 预算无限时只用 1 个时间片（一次跑完，不分片）')

# ── 用"茂密"的路径展示时间片确实在切分工作 ──
NOISY = make_noisy_path()
kn_rec, kn_nodes = dp_recursive(NOISY, EPS)
_, sn = dp_timesliced(NOISY, EPS, 1)
print()
print('    路径 B（噪声路径）：%d 个点 ⇒ 递归树展开 %d 个节点' % (len(NOISY), kn_nodes))
print('      每片只做 1 个栈段时，用了 %d 个时间片' % sn)
check(sn == kn_nodes,
      '★★★ 恒等式：每片 1 个栈段 ⇒ 时间片数(%d) == 递归展开节点数(%d)。'
      '两个数来自完全不同的实现，精确相等 ⇒ 迭代版确实【一次不多一次不少】地'
      '把同一棵树摊开' % (sn, kn_nodes))
check(kn_nodes > 100,
      '★ 路径 B 的递归树有 %d 个节点 ⇒ 单次跑完确实超出 200 µs 的量级' % kn_nodes)

# ── ★ 反向对照：故意写错的版本会活锁 ──
print()
pops_needed = dp_timesliced(NOISY, EPS, 1)[1]
_, bad_slices, livelocked = dp_naive_broken(NOISY, EPS, 100)
check(livelocked,
      '★★★ 反向对照：把时间片塞进内层 for 且不保存扫描进度 ⇒ 【活锁】'
      '（%d 点、预算 100，跑了 %d 片还没完）' % (len(NOISY), bad_slices))
print('      ❌ 原因：每片都从 start+1 重新扫，预算 100 永远扫不到第 101 个点，')
print('         于是"重来 → 卡住 → 再重来"无限循环。')
print('      ✅ 源码的解法：工作单元 = 一个栈段的完整扫描（≤500 点，有界），')
print('         时间片检查放在 while 顶部 ⇒ 天然可续。')
print('      ✅ 同一个"错误"实现，预算给足够大就不活锁 —— 所以问题出在')
print('         【粒度 + 进度】，而非算法本身：')
_, ok_slices, ok_live = dp_naive_broken(NOISY, EPS, 10 ** 9)
check(not ok_live,
      '★ 预算无限时"错误"实现也能跑完（%d 片）' % ok_slices)
check(pops_needed < bad_slices / 10,
      '★★ 正确版跑同一条路径只需 %d 片，错误版 %d 片还差得远'
      '（差 %.0f 倍且还在跑）⇒ 对照成立'
      % (pops_needed, bad_slices, bad_slices / float(pops_needed)))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[3] ★★★ 剪枝不变式：PRUNING_DELTA < ACCURACY，且必须先抽稀')
print('=' * 74)
print('    源码 AP_SmartRTL.h:21 —— 注意它是【从 _accuracy 推出来的】，不是独立常量：')
print('        #define SMARTRTL_PRUNING_DELTA (_accuracy * 0.99f)')
print('        // "must be smaller than _ACCURACY parameter"')
PRUNING_DELTA = SRTL_ACCURACY_DEFAULT * 0.99
print()
print('    ACCURACY      = %.2f m' % SRTL_ACCURACY_DEFAULT)
print('    PRUNING_DELTA = %.2f m  (= %.2f × 0.99)' % (PRUNING_DELTA, SRTL_ACCURACY_DEFAULT))
print('    抽稀后最小间距 = %.2f m' % min(gaps))
check(PRUNING_DELTA < SRTL_ACCURACY_DEFAULT,
      '★ 不变式成立：PRUNING_DELTA(%.2f) < ACCURACY(%.2f)'
      % (PRUNING_DELTA, SRTL_ACCURACY_DEFAULT))
check(PRUNING_DELTA < min(gaps),
      '★ 且严格小于抽稀后的实际最小间距(%.2f) ⇒ 相邻线段【绝不可能】被误判为环'
      % min(gaps))


def detect_loops(path, delta):
    """源码 :574-616 的判定逻辑（去掉时间片/状态机，只验判定）"""
    found = []
    i = len(path) - 1
    while i >= 4:
        j = 1
        while j <= i - 2:
            d = seg_seg_dist(path[i], path[i - 1], path[j - 1], path[j])
            if d < delta:
                found.append((j, i - 1, d))
                j = i                            # ★ 源码：跳到外循环下一个，防重复命中
            j += 1
        i -= 1
    return found


# ── 不变式的一半：delta < accuracy ⇒ 直线不误判 ──
n_ok_line = len(detect_loops(DECIMATED_LINE, PRUNING_DELTA))
print()
print('    %-46s %s' % ('场景', '判为环的线段对数'))
print('    %-46s %d' % ('① 抽稀后的直线, delta = 0.99×ACC（源码配置）', n_ok_line))
check(n_ok_line == 0,
      '★★ 抽稀 + delta<ACC ⇒ 直线路径【0 个】假环 ⇒ 源码配置是对的')

# ── 不变式的另一半（反向对照）：只破坏一个条件就会误判 ──
n_bad1 = len(detect_loops(DECIMATED_LINE, SRTL_ACCURACY_DEFAULT * 1.01))
n_bad2 = len(detect_loops(LINE, PRUNING_DELTA))
print('    %-46s %d' % ('② 抽稀后, 但 delta = 1.01×ACC（破坏 delta<ACC）', n_bad1))
print('    %-46s %d' % ('③ 没抽稀(0.5m 间距), delta = 0.99×ACC', n_bad2))
check(n_bad1 > 0,
      '★★★ 反向对照：把 delta 调到略大于 ACC ⇒ 立刻冒出 %d 个【假环】'
      '⇒ "delta < ACC" 是【承重的】，不是随手取的' % n_bad1)
check(n_bad2 > 0,
      '★★★ 反向对照：跳过抽稀（点间距 0.5 m < delta 1.98 m）⇒ 冒出 %d 个假环'
      '⇒ 必须先抽稀再剪枝，两步的【顺序也不能换】' % n_bad2)

# ── 真正的环 ──
print()
# "飞出去又原路飞回来" —— 这正是 SmartRTL 剪枝要处理的形态：
# 去程在 y = 0，回程在 y = 0.6（航线偏移 0.6 m，远小于 delta 1.98 m）
LOOPY_RAW = []
for i in range(21):
    LOOPY_RAW.append((2.0 * i, 0.0))            # 去程 x: 0 → 40
for i in range(1, 21):
    LOOPY_RAW.append((40.0 - 2.0 * i, 0.6))     # 回程 x: 38 → 0
LOOPY = add_points(LOOPY_RAW, SRTL_ACCURACY_DEFAULT)
check(LOOPY == LOOPY_RAW,
      '★ 该路径点距恰为 2.0 m(= ACCURACY) ⇒ 抽稀是恒等变换（%d 点原样保留）'
      % len(LOOPY_RAW))
loops = detect_loops(LOOPY, PRUNING_DELTA)
print('    "飞出去又原路飞回来"的路径：%d 点（去程 y=0，回程 y=0.6）' % len(LOOPY))
print('    检测到 %d 个环，最近的一个距离 %.3f m（阈值 %.2f m）'
      % (len(loops), min(l[2] for l in loops) if loops else -1, PRUNING_DELTA))
check(len(loops) > 0, '★★ 真环被检测到（%d 个）⇒ 判据方向正确，不是恒假' % len(loops))
check(len(loops) > 0 and min(l[2] for l in loops) < PRUNING_DELTA,
      '★ 命中的环确实满足 < delta（几何判据与代码判据一致）')
check(n_ok_line == 0 and len(loops) > 0,
      '★★★ 同一套判据：直线 0 个 / 绕圈 >0 ⇒ 既不误报也不漏报')

# ── 几何判据本身 ──
print()
print('    %-40s %-14s %s' % ('场景', '最近距离(m)', '判定(delta=%.2f)' % PRUNING_DELTA))
SCEN = [
    ([(0, 0), (10, 0)], [(5, -1), (5, 1)], '十字交叉（真正相交）', 0.0),
    ([(0, 0), (10, 0)], [(5, 5), (5, 8)], '靠近但相距 5 m', 5.0),
    ([(0, 0), (10, 0)], [(0, 20), (10, 20)], '两条平行线，相距 20 m', 20.0),
    ([(0, 0), (10, 0)], [(10.5, 0), (20, 0)], '首尾相接，缝 0.5 m', 0.5),
]
for segA, segB, desc, want_d in SCEN:
    d = seg_seg_dist(segA[0], segA[1], segB[0], segB[1])
    is_loop = d < PRUNING_DELTA
    print('    %-40s %-14.4f %s' % (desc, d, '★ 判为环' if is_loop else '不是环'))
    check(abs(d - want_d) < 1e-6,
          '几何独立核验：%s ⇒ 最近距离 = %.4f（期望 %.1f，±1e-6）' % (desc, d, want_d))
# 三个"该/不该"的判定
check(seg_seg_dist((0, 0), (10, 0), (5, -1), (5, 1)) < PRUNING_DELTA,
      '★ 交叉线段判为环（距离 0）')
check(not (seg_seg_dist((0, 0), (10, 0), (5, 5), (5, 8)) < PRUNING_DELTA),
      '★ 反向对照：相距 5 m > delta ⇒ 不判为环')
check(not (seg_seg_dist((0, 0), (10, 0), (0, 20), (10, 20)) < PRUNING_DELTA),
      '★ 平行线（20 m）不判为环')
check(seg_seg_dist((0, 0), (10, 0), (10.5, 0), (20, 0)) < PRUNING_DELTA,
      '★ 首尾相接（缝 0.5 m < delta 1.98 m）判为环 —— 正确，飞机确实回来了')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[4] Rally：最近点选择与 RALLY_LIMIT_KM 的"防串场"（AP_Rally.cpp:152）')
print('=' * 74)

RALLY_LIMIT_KM = 1.0


def find_nearest(current, rally, limit_km):
    """源码 :152-176 原样重写。
       注意：limit_km 比的是【当前位置到最近备降点】的距离，
       不是【家到备降点】的距离。"""
    min_dis, ret = -1.0, None
    for name, loc in rally:
        d = dist(current, loc)
        if d < min_dis or min_dis < 0:
            min_dis, ret = d, name
    if limit_km > 0 and min_dis > limit_km * 1000.0:
        return None, min_dis               # ★ 太远 ⇒ 返回 nullptr，调用方用 home
    return ret, min_dis


print()
print('    场景 1：本场附近有 3 个备降点，其中 旧场C 是上一场遗留的')
RALLY = [('本场A', (800.0, 0.0)),
         ('本场B', (0.0, 900.0)),
         ('旧场C', (40000.0, 0.0))]
print('    %-22s %-14s %-12s %s' % ('当前位置', '选中的备降点', '距离(m)', '是否触发保护'))
CASES = [
    ((0.0, 0.0),       '本场A', '用 A（800 m，最近）'),
    ((790.0, 10.0),    '本场A', '用 A（14 m，就在旁边）'),
    ((40000.0, 100.0), '旧场C', '用 C（100 m）—— 人就在那儿，该用'),
]
for cur, want, note in CASES:
    got, d = find_nearest(cur, RALLY, RALLY_LIMIT_KM)
    print('    %-22s %-14s %-12.1f %s' % ('%.0f,%.0f' % cur, got, d, note))
    check(got == want, '%s ⇒ 选中 %s，距离 %.1f m' % (note, got, d))

print()
print('    场景 2 ★ 保护真正起作用的场景：')
print('      飞机在 (500, 0)，家(0,0) 就在 500 m 外；')
print('      但飞控里【只剩一个上一场遗留的备降点 旧场C 在 40 km 外】。')
RALLY_STALE = [('旧场C', (40000.0, 0.0))]
CUR = (500.0, 0.0)
d_home = dist(CUR, (0.0, 0.0))
got_off, d_off = find_nearest(CUR, RALLY_STALE, 0.0)              # 关保护
got_on, d_on = find_nearest(CUR, RALLY_STALE, RALLY_LIMIT_KM)     # 开保护
print()
print('      离家 %.0f m；离 旧场C %.0f m' % (d_home, d_off))
print('      %-28s ⇒ %-10s 距离 %.0f m' % ('LIMIT_KM = 0（保护关闭）', str(got_off), d_off))
print('      %-28s ⇒ %-10s 距离 %.0f m（调用方回退到 home）'
      % ('LIMIT_KM = %.1f（保护开启）' % RALLY_LIMIT_KM, str(got_on), d_on))
check(got_off == '旧场C',
      '★★ 关掉保护 ⇒ 选中 %.0f m 外的 旧场C（比回家多绕 %.0f km）—— 这就是 bug 的形状'
      % (d_off, (d_off - d_home) / 1000.0))
check(got_on is None,
      '★★★ 开启保护 ⇒ 返回 None ⇒ 调用方用 home（%.0f m）⇒ 少飞 %.0f km'
      % (d_home, (d_off - d_home) / 1000.0))
check(d_off > RALLY_LIMIT_KM * 1000.0 > d_home,
      '★★ 判据比的是【到我的备降点距离】(%.0f m > %.1f km) —— '
      'home(%.0f m) 根本不参与这个比较 ⇒ 保护的是"备降点",不是"家"'
      % (d_off, RALLY_LIMIT_KM, d_home))
print()
print('    ⇒ 参数说明原文："This prevents a leftover rally point from a different')
print('      airfield being used accidentally." —— 说的就是这个场景。')

# 边界：正好卡在 RALLY_LIMIT_KM 上 —— 用【恰好等于】才能区分 > 和 >=
got_in, _ = find_nearest((999.0, 0.0), [('X', (0.0, 0.0))], 1.0)
got_eq, _ = find_nearest((1000.0, 0.0), [('X', (0.0, 0.0))], 1.0)
got_out, _ = find_nearest((1001.0, 0.0), [('X', (0.0, 0.0))], 1.0)
print()
print('      999 m  ⇒ %-8s   （限内，不保护）' % str(got_in))
print('     1000 m  ⇒ %-8s   （恰好等于：源码用 > ，所以【不】保护）' % str(got_eq))
print('     1001 m  ⇒ %-8s   （限外，保护）' % str(got_out))
check(got_in == 'X', '★ 限内 999 m 不触发保护')
check(got_eq == 'X',
      '★★ 恰好 1000 m 不触发 ⇒ 判据是 > 而不是 ≥（用 1001 测不出这个区别）')
check(got_out is None, '★ 限外 1001 m 触发保护')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
