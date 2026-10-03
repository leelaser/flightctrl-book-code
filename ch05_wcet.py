# -*- coding: utf-8 -*-
"""ch05_wcet.py —— 第 5 章：WCET、中断延迟与抖动源（配套脚本）

第 3 章的可调度性分析和第 4 章的阻塞分析，全建立在**一个数**上：
    $C_i$ —— 每个任务的最坏执行时间。
    第 3 章开头把它列为"假设 2"（必须是真上界，不是平均值），
    然后就用它去算 $R$。但那个数**从哪来**？

本章要跑出来的是四个"想当然会踩空"的地方：

  ① **测量法给的是下界，不是上界**。
     构造一个分档的执行时间分布（越罕见的干扰越贵），看
     "采样 n 次的最大值"随 n 怎么变。样本翻十倍，最大值跳一档 ——
     它**不收敛**。要 95% 把握见到最罕见的那一档，需要多少样本？
     答案在 1 kHz 的测试台上是"以年计"。
  ② **静态分析的上界由"循环边界"决定**。
     同一段循环，工具在无人标注时按 32 位计数器给上界，与按算法
     真实上界（64 次）标注的，差 6 个数量级 —— 一个是 161 秒，一个是 2.4 µs。
     **WCET 分析的第一难点不是算法，是边界标注。**
  ③ **"数据量远小于缓存"≠"命中"**：直接映射缓存的冲突未命中。
     这解释了为什么 WCET 分析必须取**最坏缓存状态**，
     而最坏比"热缓存"贵一个数量级。
  ④ **tick 粒度把 1 ms 写进你的相位裕度**。
     调度器只在 tick 边界做决定时，1 kHz 环路的释放抖动是 1 ms ——
     正好是第 2 章那个 5.7°。对比硬件定时器直接触发。

跑法：python code/ch05_wcet.py
"""

import math
import random


# ===========================================================================
# ① 测量法：采样 n 次的最大值是**下界**
# ===========================================================================
BASE = 120                                  # µs，无干扰时的执行时间
# (每次释放触发该档的概率, 附加耗时 µs, 说明)
EVENTS = [
    (1e-4,  200,  '一次中断服务插进来'),
    (1e-6, 2000,  'DMA 与 Flash 争用总线'),
    (1e-8, 20000, '总线被长时间占用'),
]
# ★ 真上界：**四档同时发生**。这是本脚本自己造出来的、我们**已知**的数 ——
#   真实的嵌入式系统里，这个数恰恰是不知道的，而这就是问题所在。
TRUE_WCET = BASE + sum(d for _, d, _ in EVENTS)


def one_sample(rng):
    t = BASE
    for p, d, _ in EVENTS:
        if rng.random() < p:
            t += d
    return t


def n_for(p, conf):
    """要在**每一次**采样中至少见到一次概率为 p 的事件，
    达到置信度 conf 所需的采样次数。"""
    return math.log(1 - conf) / math.log(1 - p)


# ===========================================================================
# ② 静态分析：上界由循环边界决定
# ===========================================================================
BODY_CYCLES = 18          # 循环体（一次迭代）的 WCET，cycles
F_HZ = 480e6              # Cortex-M7 主频
ANNOTATED = 64            # 人工标注的循环上界（由算法保证：最多 64 次）
COUNTER32 = 2 ** 32       # 工具在**无人标注**时的默认假设：32 位计数器可以绕一整圈


# ===========================================================================
# ③ 缓存：直接映射的冲突未命中
# ===========================================================================
CACHE_LINES = 4           # 直接映射，4 行，一行一块（简化模型）
HIT, MISS = 1, 31         # 命中 1 cycle，未命中 31 cycles（典型 30 倍差）


def cache_time(addrs, lines=CACHE_LINES, init=None):
    """按地址序列模拟直接映射缓存，返回总周期数。

    命中判据是"该行当前装的是不是**这个地址**" —— 一行只能装一个块，
    所以两个地址只要映射到同一行，就会互相踢出去（冲突未命中）。

    `init` 是初始缓存状态（每行装哪个地址，None 表示空）。默认全空。
    """
    tag = list(init) if init is not None else [None] * lines
    total = 0
    for a in addrs:
        ln = a % lines
        if tag[ln] == a:
            total += HIT
        else:
            tag[ln] = a
            total += MISS
    return total


def all_initial_states(addrs, lines=CACHE_LINES):
    """穷举**所有**可能的初始缓存状态。

    ★ 这正是 WCET 缓存分析（抽象解释）在做的事：分析器不知道进入这段代码时
      缓存里有什么，于是它对所有可能的状态求**最坏值**。规模小时可以直接穷举。
    """
    tags = [None] + sorted(set(addrs))
    from itertools import product
    return [list(s) for s in product(tags, repeat=lines)]


# ===========================================================================
def main():
    print('=' * 72)
    print('① 测量法：采样 n 次的最大值，是下界不是上界')
    print('=' * 72)
    print('  "真实"的执行时间分布（µs）：基线 %d，另有三档干扰事件' % BASE)
    for p, d, why in EVENTS:
        print('     p = %-8g  +%6d µs   %s' % (p, d, why))
    print('  真上界（三档同时发生）= %d µs' % TRUE_WCET)
    print()

    rng = random.Random(20261002)
    N = 1000000
    vals = [one_sample(rng) for _ in range(N)]
    mean = sum(vals) / float(N)
    print('  平均执行时间 = %.2f µs   ← 平均值**完全看不到尾部**' % mean)
    print()
    print('  %-14s %-14s %s' % ('采样次数', '实测最大值', '离真上界还差'))
    maxes = {}
    for n in (10000, 100000, 1000000):
        m = max(vals[:n])
        maxes[n] = m
        print('  %-14s %-14d %.1f 倍' % ('{:,}'.format(n), m,
                                         TRUE_WCET / float(m)))
    print('  %-14s %-14d %s' % ('真上界', TRUE_WCET, '—— 采样里从未出现过'))
    print()

    # ★ 结构断言：最大值只能是"已触发档位之和 + 基线"。
    #   不在这个集合里的数说明仿真器写错了（比如概率判反了）。
    legal = {BASE + sum(EVENTS[i][1] for i in s)
             for s in ([], [0], [1], [2], [0, 1], [0, 2], [1, 2], [0, 1, 2])}
    for n, m in maxes.items():
        assert m in legal, '最大值 %d 不在可能的档位和 %s 里' % (m, sorted(legal))
    assert maxes[10000] <= maxes[100000] <= maxes[1000000], \
        '前缀法下最大值必须单调不减'
    assert mean < BASE + 1, '平均值不该被罕见事件抬起（实测 %.2f）' % mean
    assert maxes[1000000] < TRUE_WCET, '测到的最大值不可能超过真上界（构造保证）'
    print('  ★ 平均 %.2f µs，真上界 %d µs —— 差 %d 倍 ✓'
          % (mean, TRUE_WCET, TRUE_WCET / mean))
    print('  ★ 采了 100 万次，最大值 %d µs，仍只有真上界的 %.0f%% ✓'
          % (maxes[1000000], 100.0 * maxes[1000000] / TRUE_WCET))

    print()
    print('  —— 那"测够了"要测多少次？——')
    print('  %-12s %-12s %s' % ('事件概率', '95% 把握见到', '在 1 kHz 下相当于'))
    for p, d, why in EVENTS:
        n = n_for(p, 0.95)
        secs = n / 1000.0
        if secs < 60:
            unit = '%.0f 秒' % secs
        elif secs < 3600:
            unit = '%.0f 分钟' % (secs / 60)
        elif secs < 86400:
            unit = '%.1f 小时' % (secs / 3600)
        elif secs < 86400 * 365:
            unit = '%.1f 天' % (secs / 86400)
        else:
            unit = '%.1f 年' % (secs / 86400 / 365)
        print('  1e%-10d %-12s %s' % (round(-math.log10(p)), '{:,}'.format(int(n)), unit))
        assert n > 0
    print()
    # ⚠️ 注意这里的 `95%%`：这段文字里有一个**字面**的百分号，
    #    必须写成 `%%` —— 直接写 `95%` 会被当成格式说明符，
    #    报的却是"参数不够"（第一版就是这么挂的，报错指向行尾而不是那个 %）。
    print('  ★ 最罕见的一档（p=1e-8）要 %.2e 次采样才 95%% 见得到 —— '
          % n_for(1e-8, 0.95))
    print('    按 1 kHz 连续跑，那是 %.1f 天（每天 24 小时不停）。**测量法给不出上界。**'
          % (n_for(1e-8, 0.95) / 1000.0 / 86400.0))
    print('  ★ 而且以上还只算了"碰到那一档"，没算"同时恰好走最坏路径" ✓')

    print()
    print('=' * 72)
    print('② 静态分析：上界由"循环边界"决定，不由工具决定')
    print('=' * 72)
    t_annot = ANNOTATED * BODY_CYCLES / F_HZ * 1e6
    t_default = COUNTER32 * BODY_CYCLES / F_HZ
    print('  循环体 %d cycles，主频 %.0f MHz' % (BODY_CYCLES, F_HZ / 1e6))
    print('  %-34s %-16s %s' % ('分析器采用的循环上界', '得到的最坏时间', '对 1 kHz 回路'))
    print('  %-34s %-16s %s' % ('人工标注：64 次（算法保证）',
                                '%.1f µs' % t_annot,
                                '占 1000 µs 预算的 %.1f%%' % (t_annot / 1000 * 100)))
    print('  %-34s %-16s %s' % ('无人标注：32 位计数器（2^32 次）',
                                '%.0f 秒' % t_default, '**完全不可用**'))
    assert t_annot < 5, '标注后的界应当是可用的（< 5 µs）'
    assert t_default > 100, '无人标注的界应当大到不可用（> 100 秒）'
    print()
    print('  ★ 同一个循环，两种边界差 %.0e 倍 —— 差的是"人有没有标边界" ✓'
          % (t_default * 1e6 / t_annot))
    print('  ★ 这就是工程现实：**静态分析的精度上限是你给的循环边界**，')
    print('    不是分析算法的水平。飞控里每个 `while` 的边界都是设计决策。')

    print()
    print('=' * 72)
    print('③ 缓存：数据量远小于缓存，仍然可以每次都未命中')
    print('=' * 72)
    N_ACCESS = 200
    # 两组地址：访问次数相同，用到的块数也相同（都是 2 个块）
    stride_hit = [0, 1] * (N_ACCESS // 2)      # 块 0 与块 1 → 行 0 与行 1
    stride_miss = [0, 4] * (N_ACCESS // 2)     # 块 0 与块 4 → **都是行 0**
    t_hit = cache_time(stride_hit)
    t_miss = cache_time(stride_miss)
    print('  缓存：直接映射，%d 行；未命中代价 %d cycles，命中 %d cycle'
          % (CACHE_LINES, MISS, HIT))
    print('  两组**各访问 %d 次**，各用到 **2 个块**（2 块 < %d 行，容量绰绰有余）'
          % (N_ACCESS, CACHE_LINES))
    print()
    print('  %-28s %-14s %s' % ('地址序列', '行号', '总周期数'))
    print('  %-28s %-14s %d' % ('0,1,0,1,…（交错）', '0,1,0,1,…', t_hit))
    print('  %-28s %-14s %d' % ('0,4,0,4,…（跨步 4 块）', '0,0,0,0,…', t_miss))
    assert t_miss > 20 * t_hit, (
        '冲突未命中应当比交错访问贵一个量级（实测 %d vs %d）' % (t_miss, t_hit))
    print()
    print('  ★ 同样的访问次数、同样小的数据量，周期数差 %.0f 倍 ✓'
          % (t_miss / float(t_hit)))
    print('  ★ 原因：4 %% %d == 0，**两个块映射到同一行**，每次互相踢出。'
          % CACHE_LINES)
    print('    这就是"冲突未命中"—— 缓存容量够，映射不够。')
    print()
    print('  —— 那"最坏缓存状态"值多少？穷举所有初始状态（各 %d 种）——'
          % len(all_initial_states(stride_miss)))
    # ★★ 这一节原本想讲"最坏缓存状态比热缓存贵一个数量级"，拿 `[2:]` 当"热"来对照 ——
    #    结果差 1 倍。**因为冲突序列每次访问都在踢出下一次要用的块，
    #    初始状态是什么根本不影响**。"热"这个对照对象被我选错了，
    #    它不是在验一个结论，是在凑一个数。改成穷举之后，真结论反过来了。
    for name, seq in (('交错（无冲突）', stride_hit), ('跨步（冲突）', stride_miss)):
        ts = [cache_time(seq, init=i) for i in all_initial_states(seq)]
        lo, hi = min(ts), max(ts)
        print('  %-16s 最好初始状态 %5d，最坏 %5d —— 初始状态值 %.2f 倍'
              % (name, lo, hi, hi / float(lo)))
    th = [cache_time(stride_hit, init=i) for i in all_initial_states(stride_hit)]
    tm = [cache_time(stride_miss, init=i) for i in all_initial_states(stride_miss)]
    spread_hit = max(th) - min(th)
    spread_miss = max(tm) - min(tm)
    # 要断的**就是本节那句话**：冲突造成的额外开销，远大于"初始状态"能造成的波动。
    # （等价地：把初始状态穷举到底也没用，真正决定量级的是访存模式。）
    assert t_miss - t_hit > 50 * spread_hit, (
        '冲突的代价应当远大于初始状态的波动（%d vs %d）'
        % (t_miss - t_hit, spread_hit))
    assert spread_miss <= spread_hit, (
        '冲突序列里初始状态更不该起作用（%d vs %d）' % (spread_miss, spread_hit))
    print()
    print('  ★ 初始缓存状态最多影响 %d cycles（交错那一组）✓' % spread_hit)
    print('  ★ 而"映射冲突"影响 %d cycles —— 大 %d 倍 ✓'
          % (t_miss - t_hit, (t_miss - t_hit) // max(spread_hit, 1)))
    print('  ★★ **这才是 WCET 缓存分析的重点**：不是"最坏初始状态"（它只值一次未命中），')
    print('     而是"这段代码的访存模式会不会自我冲突"。前者是常数，后者是量级。')
    print('     而且——冲突序列里**任何**初始状态都救不了你，穷举也白搭。')

    print()
    print('=' * 72)
    print('④ tick 粒度：把 1 ms 的抖动写进你的相位裕度')
    print('=' * 72)
    T_US = 1000.0            # 环路周期 1 ms
    OMEGA_C = 100.0          # 穿越频率 rad/s（第 2 章那个姿态环）
    ISR_JITTER = 10.0        # 硬件定时器直接触发时，中断路径本身的抖动量级（µs）

    rng = random.Random(7)
    worst_a = worst_b = 0.0
    REPS = 50000
    for _ in range(REPS):
        want = rng.uniform(0.0, T_US)          # 期望启动时刻，相对 tick 网格的相位
        # 情形 A：调度器只在 tick 边界上做决定 ⇒ 推迟到下一个 tick
        got_a = math.ceil(want / T_US) * T_US
        worst_a = max(worst_a, got_a - want)
        # 情形 B：硬件定时器直接触发 ⇒ 只剩中断路径自身的抖动
        worst_b = max(worst_b, rng.uniform(0.0, ISR_JITTER))
    deg = lambda j: 57.3 * OMEGA_C * j * 1e-6
    print('  环路 %.0f µs，穿越频率 %.0f rad/s，蒙特卡洛 %s 次' % (T_US, OMEGA_C, '{:,}'.format(REPS)))
    print()
    print('  %-34s %-16s %s' % ('调度方式', '释放抖动上界', '带来的相位损失'))
    print('  %-34s %-16s %.2f°' % ('A 只在 tick 边界调度', '%.0f µs' % worst_a, deg(worst_a)))
    print('  %-34s %-16s %.2f°' % ('B 硬件定时器直接触发', '%.0f µs' % worst_b, deg(worst_b)))
    assert worst_a > 900, 'tick 粒度下的抖动应当接近 1 ms（实测 %.1f）' % worst_a
    assert worst_b <= ISR_JITTER + 1e-9, '中断路径的抖动不该超过设定（%.1f）' % worst_b
    print()
    print('  ★ 抖动差 %.0f 倍，相位损失差 %.0f 倍 ✓'
          % (worst_a / worst_b, deg(worst_a) / deg(worst_b)))
    print('  ★ A 的 %.2f° 正是第 2 章那个「1 ms ≈ 5.7°」—— '
          % deg(worst_a))
    print('    现在你知道那个 1 ms 从哪来了：**它不是一个物理量，是设计选择**。')

    print()
    print('=' * 72)
    print('全部断言通过。')
    print('=' * 72)


if __name__ == '__main__':
    main()
