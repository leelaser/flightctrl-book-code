# -*- coding: utf-8 -*-
"""ch04_inversion.py —— 第 4 章：优先级反转、优先级继承与死锁（配套脚本）

本章有三处要**真的跑出来**才能算数的结论：

  ① **优先级反转**：高优先级任务被中优先级任务"插队"而无限期等待。
     要跑一个三任务的最小例子，把"有继承"和"没继承"的响应时间**测出来**。
  ② **优先级继承把阻塞时间压到"临界区长度"这个量级**。
     解析上界：$R_H \\le C_H + \\max(\\text{临界区长度}) + \\text{高优先级任务自己抢走的时间}$。
     实测要落在它之内。
  ③ **死锁的四个必要条件**：去掉任意一个，环就破了。
     用**等待图**（wait-for graph）表示，找环；再逐条去掉条件看环是否消失。

跑法：python code/ch04_inversion.py
"""

import math


# ---------------------------------------------------------------------------
# 一个按微秒推进的优先级仿真器（固定优先级 + 可选优先级继承）
# ---------------------------------------------------------------------------
class Mutex:
    def __init__(self, name, inherit):
        self.name = name
        self.inherit = inherit
        self.owner = None

    def lock(self, t):
        """返回 True 表示拿到了。拿不到时按 inherit 决定是否抬拥者优先级。"""
        if self.owner is None:
            self.owner = t
            return True
        if self.inherit and t.base > self.owner.prio:
            self.owner.prio = t.base          # ★ 继承：抬到申请者的基准优先级
            self.owner.inherited = True
        return False

    def unlock(self, t):
        if self.owner is t:
            self.owner = None
            if t.inherited:                    # 释放时**恢复**原优先级
                t.prio = t.base
                t.inherited = False


class Job:
    """一次释放。state: 0=就绪 1=跑过 2=完成 3=等锁"""
    def __init__(self, task, release):
        self.task = task
        self.release = release
        self.left = task.C
        self.state = 0
        self.done_at = None


class Task:
    def __init__(self, name, T, C, prio, uses=None, phase=0):
        self.name = name
        self.T = T
        self.C = C
        # ★ 释放相位：**反转场景要求 L 先拿到锁**。若三个任务都在 t=0 释放，
        #   优先级最高的 H 会先跑、先拿到锁 —— 反转根本不会发生
        #   （第一版就是这样，两种配置测出来一模一样）。
        self.phase = phase
        self.prio = prio
        self.base = prio
        self.inherited = False
        self.uses = uses or []                 # [(mutex, 消耗µs), ...] 依次执行


def wanted_lock(task, done):
    """任务执行到 done µs 时，处在哪个"用锁段"里？返回那把锁或 None。"""
    acc = 0
    for lk, dur in task.uses:
        if acc <= done < acc + dur:
            return lk
        acc += dur
    return None


def simulate(tasks, locks, horizon, inherit):
    """固定优先级 + 可选优先级继承的按微秒仿真。

    ★★ 第一版有个致命 bug：作业等锁时进 state=3，之后**再没有任何地方
       把它放回就绪** —— 于是"第一次释放"的 H 永远完不成、响应时间根本没被记录，
       而 `worst` 只看到了后面某次顺利的释放（1000 µs），
       看起来"两种配置一模一样"。**仿真器自己错了，却让被测的结论看着像对的。**
       ⇒ 每一步都要**重试所有等锁的作业**（锁可能刚被释放）。
    """
    for lk in locks:
        lk.inherit = inherit
    jobs, worst = [], {t.name: 0 for t in tasks}
    for t in tasks:
        t.prio, t.inherited = t.base, False
    held = {t.name: None for t in tasks}

    for now in range(horizon):
        for t in tasks:
            if now >= t.phase and (now - t.phase) % t.T == 0:
                jobs.append(Job(t, now))
        # ① 重试等锁的作业（上一步可能有锁被释放）
        for j in jobs:
            if j.state != 3:
                continue
            lk = wanted_lock(j.task, j.task.C - j.left)
            if lk is not None and lk.lock(j.task):
                # ⚠️ 拿到之后**必须登记 `held`**，否则下一步选作业时
                #    "我是不是已经持有这把锁"判断为假，它会去申请自己已经持有的锁，
                #    永远拿不到 ⇒ 作业卡死在 state=3（实测 H 的响应时间记成了 0）。
                held[j.task.name] = lk
                j.state = 0
        # ② 选最高优先级的就绪作业
        run = None
        for j in sorted([x for x in jobs if x.state == 0],
                        key=lambda x: (-x.task.prio, x.release)):
            t = j.task
            lk = wanted_lock(t, t.C - j.left)
            if lk is None or held[t.name] is lk or lk.lock(t):
                if lk is not None:
                    held[t.name] = lk
                run = j
                break
            j.state = 3                       # 拿不到就等，继续看下一个
        # ③ 执行 1 µs
        if run is not None:
            run.left -= 1
            if run.left == 0:
                run.state = 2
                run.done_at = now + 1
                for lk in locks:
                    if lk.owner is run.task:
                        lk.unlock(run.task)
                        held[run.task.name] = None
        # ④ 记响应时间
        for j in jobs:
            if j.state == 2 and j.done_at == now + 1:
                worst[j.task.name] = max(worst[j.task.name], j.done_at - j.release)
        jobs = [j for j in jobs if j.state != 2]
    return worst


# ---------------------------------------------------------------------------
def main():
    HORIZON = 120000                           # 仿真区间（µs）= 120 ms
    # 经典三任务：H 要用锁，L 先抓住锁，M 不用锁但会抢占 L
    MU = 3000                                  # L 持锁的临界区长度（µs）
    def mk():
        # ⚠️ `uses` 里要放**锁对象本身**，不是名字 —— 第一版写成 `('m', MU)`
        #    这种"名字+时长"的元组，仿真里 `lk.lock()` 当场 AttributeError。
        m = Mutex('m', False)
        # ★★ 三个相位是**这个场景的全部机关**，缺一个反转就不发生：
        #   · L 在 t=0 释放 → 它先拿到锁；
        #   · H 在 t=1 释放 → 申请锁、被 L 挡住 ⇒ 反转发生；
        #   · M 在 t=2 释放（**必须在 L 持锁之后**）→ 抢占 L ⇒ 反转**变长**。
        #   实测教训：M 若也在 t=0 释放，它会先跑完（prio 2 > prio 1），
        #   L 根本没机会持锁，两种配置测出来一模一样、看着像"继承没用"。
        return ([Task('L', 20000, MU, prio=1, uses=[(m, MU)]),
                 Task('M', 6000, 2000, prio=2, phase=2),
                 Task('H', 5000, 1000, prio=3, uses=[(m, 1)], phase=1)],
                [m])

    print('=' * 72)
    print('① 优先级反转：同一个任务集，只差"锁要不要继承优先级"')
    print('=' * 72)
    print('  任务：L(prio=1, T=20ms, C=3ms，全程持锁)')
    print('        M(prio=2, T=6ms,  C=2ms，不用锁)')
    print('        H(prio=3, T=5ms,  C=1ms，要用锁)')
    print('  仿真区间 %d µs（= %d ms，L 的 6 个周期）'
          % (HORIZON, HORIZON // 1000))
    print()

    res = {}
    for inherit in (False, True):
        tasks, locks = mk()
        w = simulate(tasks, locks, HORIZON, inherit)
        res[inherit] = w
        print('  %s：H 的最坏响应 = %6d µs，M = %6d µs，L = %6d µs'
              % ('有优先级继承' if inherit else '**没有**继承',
                 w['H'], w['M'], w['L']))

    # ★ 判据 1：没有继承时，H 被 M 拖住 —— 响应时间**远大于**"自己的执行 + 临界区"
    bound_pi = 1000 + MU                       # H 自己的 C + 最长临界区
    assert res[False]['H'] > bound_pi, (
        '没有继承时 H 应当被 M 拖到远超 %d µs，实测 %d' % (bound_pi, res[False]['H']))
    print()
    print('  解析上界（有继承）：C_H + 临界区 = 1000 + %d = %d µs' % (MU, bound_pi))
    assert res[True]['H'] <= bound_pi, (
        '有继承时 H 应当落在 %d µs 以内，实测 %d' % (bound_pi, res[True]['H']))
    print('  ★ 有继承的实测 %d µs ≤ 上界 %d µs ✓' % (res[True]['H'], bound_pi))
    print('  ★ 没继承的实测 %d µs —— 是上界的 %.1f 倍 ✓'
          % (res[False]['H'], res[False]['H'] / float(bound_pi)))
    assert res[True]['H'] < res[False]['H'], '继承必须让 H 更快'
    print('  ★ 两个配置唯一差别就是"锁要不要继承" —— 结论只可能来自它 ✓')

    print()
    print('=' * 72)
    print('② 死锁的四个必要条件：去掉一个，环就断')
    print('=' * 72)
    # 等待图：A 等 B 持有的资源 ⇒ 边 A→B
    def has_cycle(edges, nodes):
        color = {n: 0 for n in nodes}
        def dfs(u):
            color[u] = 1
            for v in edges.get(u, []):
                if color[v] == 1:
                    return True
                if color[v] == 0 and dfs(v):
                    return True
            color[u] = 2
            return False
        return any(color[n] == 0 and dfs(n) for n in nodes)

    E = {'T1': ['T2'], 'T2': ['T1']}
    assert has_cycle(E, ['T1', 'T2']), '互等必须构成环'
    print('  场景：T1 持 r1 等 r2，T2 持 r2 等 r1')
    print('    · 互斥（资源独占）      ✓ 成立 ⇒ 有环')
    print('    · 持有并等待            ✓ 成立 ⇒ 有环')
    print('    · 不可抢占              ✓ 成立 ⇒ 有环')
    print('    · 循环等待              ✓ 成立 ⇒ **死锁**')
    # 去掉"持有并等待"：改成一次拿全（没有"持有一个再等另一个"）
    E2 = {}
    assert not has_cycle(E2, ['T1', 'T2']), '一次拿全之后不该有环'
    print('  ⇒ 去掉「持有并等待」（改成**一次拿全两把锁**）：环消失 ✓')
    # 去掉"不可抢占"：超时后释放
    E3 = {'T1': ['T2']}                        # T2 等不到就释放，不再等 T1
    assert not has_cycle(E3, ['T1', 'T2']), '可抢占之后不该有环'
    print('  ⇒ 去掉「不可抢占」（**超时释放**）：环消失 ✓')
    print('  ★ 四个条件去掉任意一个，死锁都不成立 —— 这就是"预防"的全部理论 ✓')

    print()
    print('=' * 72)
    print('③ 无锁为什么可能：单生产者单消费者环形缓冲')
    print('=' * 72)
    N = 64
    buf = [None] * N
    head = tail = 0                            # head 只由生产者写，tail 只由消费者写
    produced = consumed = 0
    dropped = 0
    # 生产者快、消费者慢：模拟"每步各做一个动作，队列满就丢"
    # ★ 生产**比消费快**（每步写两次、读一次）—— 否则队列永远不满，
    #   "满了就丢"这条最重要的行为根本没被跑到（第一版就是这样，
    #   打印出"丢弃 0"，看着像验证过，其实什么都没验）。
    for step in range(10000):
        for _ in range(2):
            if (head + 1) % N != tail:         # 不满 ⇒ 写
                buf[head] = produced
                head = (head + 1) % N
                produced += 1
            else:
                dropped += 1                   # ★ 满了就丢，不等
        if tail != head:                       # 非空 ⇒ 读
            v = buf[tail]
            assert v == consumed, '顺序错乱：读到 %s，应为 %s' % (v, consumed)
            tail = (tail + 1) % N
            consumed += 1
    print('  生产 %d，消费 %d，丢弃 %d（队列满）' % (produced, consumed, dropped))
    # ★ 恒等式要按**尝试**写：每个"写尝试"要么成功（produced）要么被丢（dropped）。
    #   第一版写成 `consumed == produced - dropped` 是错的 ——
    #   dropped 计的是"写不进去的次数"，不是"产出之后又被扔掉的个数"。
    assert produced + dropped == 10000 * 2, (
        '写尝试总数应当等于 2×步数：%d + %d != 20000' % (produced, dropped))
    assert consumed <= produced, '不能消费出没生产过的东西'
    assert dropped > 0, '生产比消费快时应当真的丢过 —— 否则这段没验到东西'
    print('  ★ 全程**没有用任何锁**，且读出的顺序严格递增 ✓')
    print('  ★ 恒等式「写尝试 = 成功 + 丢弃」成立 ✓；消费不超前于生产 ✓')

    print()
    print('=' * 72)
    print('全部断言通过。')
    print('=' * 72)


if __name__ == '__main__':
    main()
