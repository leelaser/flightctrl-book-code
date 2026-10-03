# -*- coding: utf-8 -*-
"""ch07_bounds.py —— 卷 7 第 9 章：把实测抖动灌进 RTA，看余量还剩多少

这一章要回答本卷最后一个问题：**Linux 上能不能做可调度性分析？**

卷 4 第 3 章那套 RTA 是这么算的：

    R_i = C_i + B_i + Σ ⌈R_i / T_j⌉ · C_j        —— 一个**确定的数**

它成立的前提是"每个任务在它该释放的时刻释放、该跑完的时候跑完"。
Linux 上这条前提**不成立**：第 8 章实测到，同一个配置两次跑，
唤醒抖动的最大值能差 27.5 倍。

本脚本把那份**真实测到的抖动分布**（20000 条原始样本）
直接灌进 RTA 的余量里，量一件事：

    **那个"确定的余量"，会被抖动吃掉多少？**

数据来源：`code/data/ch07_jitter_raw_fifo_load.txt`
    —— 由 `code/ch07_linux_jitter.c` 在 SCHED_FIFO + 12 个干扰线程下
       跑 20000 次、周期 1 ms 采集（环境变量 CH07_DUMP 导出）。

跑法：python code/ch07_bounds.py
"""

import io
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, 'data', 'ch07_jitter_raw_fifo_load.txt')


# ---------------------------------------------------------------------------
# ① 卷 4 第 3 章的任务集（单位 µs），按优先级从高到低
# ---------------------------------------------------------------------------
class Task(object):
    def __init__(self, name, T, C, D=None):
        self.name, self.T, self.C = name, T, C
        self.D = D if D is not None else T


TASKS = [Task('姿态环', 1000, 250),
         Task('位置环', 4000, 400),
         Task('日志',  10000, 600),
         Task('遥测',  50000, 900)]


def rta(tasks, i, max_iter=10000):
    """不动点迭代：R = C_i + Σ ⌈R/T_j⌉·C_j（同卷 4 第 3 章）"""
    R = tasks[i].C
    for k in range(1, max_iter + 1):
        nxt = tasks[i].C
        for j in range(i):
            nxt += math.ceil(R / tasks[j].T) * tasks[j].C
        if nxt == R:
            return R
        if nxt > tasks[i].D:
            return None
        R = nxt
    return None


def load_samples(path):
    vals = []
    with io.open(path, encoding='utf-8') as fh:
        for ln in fh:
            ln = ln.strip()
            if ln:
                vals.append(int(ln) / 1000.0)      # ns → µs
    return vals


def pct(sorted_vals, p):
    i = int(p * (len(sorted_vals) - 1))
    return sorted_vals[max(0, min(i, len(sorted_vals) - 1))]


# ---------------------------------------------------------------------------
def main():
    print('=' * 74)
    print('① 先按卷 4 的 RTA 算一遍：每个任务"确定"的响应时间与余量')
    print('=' * 74)
    print('  %-8s %7s %7s %9s %10s' % ('任务', 'T', 'C', 'R(确定)', '余量 D−R'))
    margins = []
    for i, t in enumerate(TASKS):
        R = rta(TASKS, i)
        m = t.D - R if R else 0
        margins.append(m)
        print('  %-8s %7d %7d %9s %10s'
              % (t.name, t.T, t.C, R if R else '发散', m))
    print()
    print('  ★ 这是卷 4 那套分析的输出：**一个数**。')
    print('    它的前提是"任务在该释放的时刻释放"。')

    # -----------------------------------------------------------------------
    print()
    print('=' * 74)
    print('② 把第 8 章实测的抖动灌进来')
    print('=' * 74)
    if not os.path.exists(RAW):
        print('  找不到 %s —— 先跑 code/ch07_jitter.sh 采集。' % RAW)
        return 1
    s = sorted(load_samples(RAW))
    n = len(s)
    total = sum(s)
    print('  样本 %d 条（SCHED_FIFO + 12 干扰线程，周期 1000 µs）' % n)
    print('  最小 %.1f   中位 %.1f   平均 %.1f  µs'
          % (s[0], pct(s, 0.5), total / n))
    print('  p99  %.1f   p99.9 %.1f   最大 %.1f  µs'
          % (pct(s, 0.99), pct(s, 0.999), s[-1]))
    over = sum(1 for v in s if v > TASKS[0].T)
    print('  超过一个周期（%d µs）的样本：%d 条（%.3f%%）'
          % (TASKS[0].T, over, 100.0 * over / n))

    # -----------------------------------------------------------------------
    print()
    print('=' * 74)
    print('③ ★★ 抖动吃掉的余量：姿态环')
    print('=' * 74)
    m0 = margins[0]
    R0 = rta(TASKS, 0)
    print('  姿态环：R = %d µs，D = %d µs，余量 = %d µs' % (R0, TASKS[0].D, m0))
    print()
    print('  %-14s %-12s %-12s %s' % ('抖动分位', '抖动值', '实际响应', '余量还剩'))
    for label, p in (('中位', 0.50), ('p99', 0.99), ('p99.9', 0.999), ('最大', 1.0)):
        j = pct(s, p) if p < 1.0 else s[-1]
        R_eff = R0 + j
        left = TASKS[0].D - R_eff
        flag = '' if left > 0 else '  ← **超期**'
        print('  %-14s %-12s %-12.1f %s%s'
              % (label, '%.1f µs' % j, R_eff,
                 ('%.1f µs' % left) if left > 0 else '已经为负', flag))

    # 有多少比例的释放会让姿态环超期
    bad = sum(1 for v in s if R0 + v > TASKS[0].D)
    print()
    print('  **会让姿态环超期的释放**：%d / %d = **%.3f%%**'
          % (bad, n, 100.0 * bad / n))
    if bad:
        need = 1.0 / (bad / float(n))
        print('  ⇒ 平均每 %.0f 次释放就有一拍晚于截止期' % need)

    # -----------------------------------------------------------------------
    print()
    print('=' * 74)
    print('④ 对照：如果抖动是个"设计量"（RTOS 的典型情形）')
    print('=' * 74)
    print('  %-22s %-14s %-14s %s' % ('假设的抖动量级', '实际响应', '余量', '超期次数'))
    for j in (1.0, 10.0, 50.0, 100.0):
        R_eff = R0 + j
        left = TASKS[0].D - R_eff
        k = sum(1 for v in s if v > j)      # 有多少实测样本超过这个量级
        print('  %-22s %-14s %-14s %s'
              % ('抖动 ≤ %.0f µs' % j, '%.0f µs' % R_eff,
                 '%.0f µs' % left, '0（设计保证）'))

    print()
    print('  ★★ 上表最后一行是关键：在 RTOS 上，抖动是**由设计决定的**')
    print('     （硬件定时器直接触发 ⇒ µs 量级、可写进设计文档）。')
    print('     而在 Linux 上，它是**一份实测分布** —— 尾部多长取决于负载与运气。')

    # -----------------------------------------------------------------------
    print()
    print('=' * 74)
    print('⑤ 结论：这个分析做不下去了')
    print('=' * 74)
    ok = True

    # ① 实测抖动确实会把最高优先级任务的余量吃光
    p1 = bad > 0
    ok &= p1
    print('  %s 实测抖动真的造成了超期（%d / %d）'
          % ('✓' if p1 else '✗', bad, n))

    # ② 最大值远超一个周期
    p2 = s[-1] > TASKS[0].T
    ok &= p2
    print('  %s 最大抖动 %.1f µs > 一个周期 %d µs'
          % ('✓' if p2 else '✗', s[-1], TASKS[0].T))

    # ③ 但中位数很小 —— 说明"平均很安全、尾巴要命"
    p3 = pct(s, 0.5) < 0.05 * TASKS[0].T
    ok &= p3
    print('  %s 中位数只有 %.1f µs（远小于周期）—— 「看着很安全」正是它的危险之处'
          % ('✓' if p3 else '✗', pct(s, 0.5)))

    print()
    print('  ⇒ 卷 4 的 RTA 给出的是"R = %d µs，余量 %d µs"，**确定**；' % (R0, m0))
    print('    而实际系统的响应时间是"R + 抖动"，其中抖动是**分布**。')
    print('    **一个确定的分析框架，灌不进一个无限尾的随机量。**')
    print('    这才是"Linux 上做不了硬实时分析"的准确表述 ——')
    print('    不是"Linux 太慢"，是**它给不出那个数**。')

    print()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
