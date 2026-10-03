# -*- coding: utf-8 -*-
"""ch03_sched.py —— 第 3 章的可调度性分析（配套脚本）

本章给了三个"纸上算一算"的工具：
    ① 利用率界（RMS）：U ≤ n(2^{1/n} − 1)，n→∞ 时趋近 ln 2 ≈ 0.693
    ② 响应时间分析（RTA）：R ← C + Σ ⌈R/T_j⌉·C_j 的不动点迭代
    ③ EDF 的充要条件：U ≤ 1

★ 这三条都是**解析结论**。本脚本用第二个、完全不同的办法给它们做独立基线：
    写一个**离散事件仿真器**，按优先级真的把任务跑一遍，
    测出每个任务的**实测最坏响应时间**，再和解析值逐项对照。

  为什么这次能要求"逐项相等"而不是"同量级"：经典结论说
  最坏情况发生在**临界瞬时**（所有任务同时释放）——
  仿真的超周期里就包含这一刻，所以两者应当**精确相等**。
  对不上就说明有一方错了，没有中间地带。

跑法：python code/ch03_sched.py
"""

import math
from fractions import Fraction


# ---------------------------------------------------------------------------
# 任务模型
# ---------------------------------------------------------------------------
class Task:
    def __init__(self, name, T, C, D=None):
        self.name = name
        self.T = T                      # 周期（µs）
        self.C = C                      # 最坏执行时间（µs）
        self.D = D if D is not None else T   # 相对截止期（默认 = 周期）

    def __repr__(self):
        return '%s(T=%d, C=%d)' % (self.name, self.T, self.C)


def utilization(tasks):
    return sum(Fraction(t.C, t.T) for t in tasks)


# ---------------------------------------------------------------------------
# ① 利用率界
# ---------------------------------------------------------------------------
def rms_bound(n):
    """n 个任务的 RMS 充分利用率界：n(2^{1/n} − 1)。"""
    return n * (2.0 ** (1.0 / n) - 1.0)


# ---------------------------------------------------------------------------
# ② 响应时间分析
# ---------------------------------------------------------------------------
def rta(tasks, i, max_iter=10000):
    """第 i 个任务（tasks 已按**优先级从高到低**排好）的最坏响应时间。

    迭代式：R ← C_i + Σ_{j 优先级高于 i} ⌈R/T_j⌉·C_j
    从 R = C_i 起迭代；若超过 D_i 或超过迭代上限即判定不可调度。
    返回 (R, 迭代次数)；不可调度返回 (None, 迭代次数)。
    """
    R = tasks[i].C
    for k in range(1, max_iter + 1):
        nxt = tasks[i].C
        for j in range(i):
            nxt += math.ceil(R / tasks[j].T) * tasks[j].C
        if nxt == R:
            return R, k
        if nxt > tasks[i].D:
            return None, k
        R = nxt
    return None, max_iter


# ---------------------------------------------------------------------------
# ③ 独立基线：离散事件仿真
# ---------------------------------------------------------------------------
def simulate(tasks, horizon):
    """按固定优先级抢先式真的跑一遍，返回每个任务的最坏响应时间。

    仿真按**微秒**推进：每个时刻选"已释放且未完成"中优先级最高的那个任务执行 1 µs。
    优先级 = 列表下标（0 最高）。
    只在**临界瞬时**（t=0 全部同时释放）之后跑一个超周期即可 ——
    这正是经典结论说的最坏情况所在，也顺便验证了那条结论。
    """
    n = len(tasks)
    remaining = [0] * n
    release = [0] * n
    start = [None] * n
    worst = [0] * n
    next_rel = [t.T for t in tasks]

    for t in range(horizon):
        # 释放
        for i in range(n):
            if t == release[i]:
                remaining[i] = tasks[i].C
                start[i] = None
                next_rel[i] = t + tasks[i].T
            if t == next_rel[i] and t != release[i]:
                remaining[i] += 0          # 占位；下面统一处理
        # ↑ 简化为：每个任务在 t 是其周期整数倍时释放
        for i in range(n):
            if t % tasks[i].T == 0:
                remaining[i] = tasks[i].C
                start[i] = None
        # 选最高优先级的就绪任务
        pick = -1
        for i in range(n):
            if remaining[i] > 0:
                pick = i
                break
        if pick >= 0:
            if start[pick] is None:
                start[pick] = t
            remaining[pick] -= 1
            if remaining[pick] == 0:
                r = t + 1 - start[pick] + (start[pick] - (t - (t % tasks[pick].T)))
                # 响应时间 = 完成时刻 − 释放时刻 = (t+1) − 该次释放时刻
                rel = (t // tasks[pick].T) * tasks[pick].T
                r = (t + 1) - rel
                worst[pick] = max(worst[pick], r)
    return worst


def hyperperiod(tasks):
    h = 1
    for t in tasks:
        h = h * t.T // math.gcd(h, t.T)
    return h


# ---------------------------------------------------------------------------
def main():
    print('=' * 72)
    print('任务集 A：四个任务的飞控（单位 µs）')
    print('=' * 72)
    A = [Task('姿态环', 1000, 250),
         Task('位置环', 4000, 400),
         Task('日志',  10000, 600),
         Task('遥测',  50000, 900)]
    for t in A:
        print('  %-6s T=%6d  C=%5d  D=%6d' % (t.name, t.T, t.C, t.D))

    U = utilization(A)
    print()
    print('利用率 U = %s = %.4f' % (U, float(U)))
    b = rms_bound(len(A))
    print('RMS 充分界 n(2^(1/n)−1) = %.4f  ⇒ U ≤ 界 ⇒ %s'
          % (b, '**保证可调度**' if float(U) <= b else '界不够用，要查 RTA'))

    print()
    print('— 响应时间分析（解析）—')
    rta_res = []
    for i, t in enumerate(A):
        R, k = rta(A, i)
        rta_res.append(R)
        print('  %-6s R = %s（迭代 %d 次）%s'
              % (t.name, ('%6d µs' % R) if R else '  发散',
                 k, '' if (R and R <= t.D) else '  ✗ 超截止期'))

    print()
    print('— 独立基线：离散事件仿真 —')
    H = hyperperiod(A)
    sim = simulate(A, H)
    print('  超周期 = %d µs' % H)
    for i, t in enumerate(A):
        print('  %-6s 实测最坏响应 = %6d µs' % (t.name, sim[i]))

    print()
    for i, t in enumerate(A):
        assert rta_res[i] == sim[i], (
            '%s：解析 %s ≠ 仿真 %s —— 两边必有一错' % (t.name, rta_res[i], sim[i]))
    print('  ★ 逐项精确相等（解析 == 仿真）✓ —— 两个独立方法互证')

    print()
    print('=' * 72)
    print('任务集 B：把日志任务的执行时间抬到 5000 µs')
    print('=' * 72)
    B = [Task('姿态环', 1000, 250),
         Task('位置环', 4000, 400),
         Task('日志',  10000, 5000),
         Task('遥测',  50000, 900)]
    UB = utilization(B)
    print('  U = %.4f，RMS 界 = %.4f ⇒ 界说"%s"'
          % (float(UB), b, '不能保证' if float(UB) > b else '没问题'))
    okB = True
    for i, t in enumerate(B):
        R, k = rta(B, i)
        okB &= (R is not None and R <= t.D)
        print('  %-6s R = %s' % (t.name, ('%6d µs' % R) if R else '发散'))
    simB = simulate(B, hyperperiod(B))
    print('  仿真实测最坏：%s' % ', '.join('%s=%d' % (t.name, r) for t, r in zip(B, simB)))
    for i, tt in enumerate(B):
        if rta(B, i)[0] is not None:
            assert rta(B, i)[0] == simB[i], '%s 解析≠仿真' % tt.name
    # ★★ 这一组是**有意的反例**：利用率超过了充分界（界"不能保证"），
    #    但 RTA 逐项都过得去 —— 说明 **界只是充分条件，不是必要条件**。
    #    把它当成"判据"会白白放弃一大批本来能跑的配置。
    assert float(UB) > b, 'B 组的利用率必须超过充分界，否则这个对照没意义'
    assert okB, 'B 组按 RTA 应当是可调度的 —— 这正是本节要讲的"界过于保守"'
    print('  ★ 界说"不能保证"，但 RTA 逐项通过 ⇒ **界是充分不必要条件** ✓')

    print()
    print('=' * 72)
    print('任务集 C：**由搜索找到**的"U < 1 但固定优先级跑不了"的例子')
    print('=' * 72)
    found = None
    for T1 in range(2, 9):
        for C1 in range(1, T1):
            for T2 in range(T1 + 1, 13):
                for C2 in range(1, T2):
                    for T3 in range(T2 + 1, 15):
                        for C3 in range(1, T3):
                            ts = [Task('a', T1, C1), Task('b', T2, C2),
                                  Task('c', T3, C3)]
                            if float(utilization(ts)) >= 1.0:
                                continue
                            if any(rta(ts, i)[0] is None for i in range(3)):
                                found = ts
                                break
                        if found: break
                    if found: break
                if found: break
            if found: break
        if found: break
    assert found, '没搜到反例 —— 那样这一节的说法就没有实证'
    UC = utilization(found)
    print('  %s' % '，'.join('%s: T=%d C=%d' % (t.name, t.T, t.C) for t in found))
    print('  U = %s = %.4f < 1（连 EDF 的理论界都满足），但 RTA：'
          % (UC, float(UC)))
    for i, t in enumerate(found):
        R, _ = rta(found, i)
        print('    %s: R = %s' % (t.name, ('%d' % R) if R else '**发散**'))
    print('  ★ 同一个任务集，EDF 能跑而 RMS 不能 —— 这就是"固定优先级不是万能的"')

    print()
    print('=' * 72)
    print('利用率界的形状（第 3.3 节那张表）')
    print('=' * 72)
    print('  %-6s %s' % ('n', 'n(2^(1/n)−1)'))
    for n in (1, 2, 3, 4, 5, 10, 100, 1000):
        print('  %-6d %.4f' % (n, rms_bound(n)))
    print('  n→∞ 的极限 = ln 2 = %.4f' % math.log(2))
    assert abs(rms_bound(100000) - math.log(2)) < 1e-4
    print('  ★ 大 n 极限收敛到 ln 2 ✓')

    print()
    print('=' * 72)
    print('全部断言通过。')
    print('=' * 72)


if __name__ == '__main__':
    main()
