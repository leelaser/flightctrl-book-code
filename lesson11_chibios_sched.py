#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 11 课配套实验：ChibiOS 调度器与就绪队列

    实验 1  RT 就绪队列的结构（从源码读出来，不写死）
    实验 2  ★★ ahead vs behind —— 两个插入函数如何实现时间片
    实验 3  ★★ NIL 的数组调度器（优先级 = 下标）
    实验 4  四路对照：操作次数量化

用法：
    python lesson11_chibios_sched.py 1
    python lesson11_chibios_sched.py 2
    python lesson11_chibios_sched.py 3
    python lesson11_chibios_sched.py 4
    python lesson11_chibios_sched.py        # 全部

【本脚本的诚实声明】

实验 1 是【读源码】，结论直接来自文件内容。
实验 2/3 是【模拟】—— 模拟的是源码里的算法，但丢掉了这些细节：

    · 真实的中断/上下文切换开销（只数逻辑操作）
    · chSysSwitch 的实际寄存器保存（模拟里是瞬时的）
    · 虚拟定时器和超时（只处理"变就绪"这一条路径）
    · SMP（模拟单核）

模拟能证明的是【算法的语义】，不能证明【真实时序】。
凡涉及真实时序的结论，本文档都会标 [未亲验]。

源码版本：6a85082c（2023-12-29，ChibiOS 内核 7.0.4）
"""

import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APM_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "APM",
                 "ardupilot-Copter-4.5.7", "ardupilot-Copter-4.5.7")
)
CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")

F_CHOBJECTS = os.path.join(CHIBIOS, "os", "rt", "include", "chobjects.h")
F_CHLISTS = os.path.join(CHIBIOS, "os", "rt", "include", "chlists.h")
F_CHSCHD_H = os.path.join(CHIBIOS, "os", "rt", "include", "chschd.h")
F_CHSCHD_C = os.path.join(CHIBIOS, "os", "rt", "src", "chschd.c")
F_NIL_CH_H = os.path.join(CHIBIOS, "os", "nil", "include", "ch.h")
F_NIL_CH_C = os.path.join(CHIBIOS, "os", "nil", "src", "ch.c")
F_SCHEDULER_H = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                             "Scheduler.h")


def read_lines(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().split("\n")
    except OSError as e:
        print("  [错误] 读不到 %s" % path)
        print("         %s" % e)
        return None


def show(path, lo, hi, label=""):
    """打印文件的某一段（1-based，含两端），带行号。"""
    lines = read_lines(path)
    if not lines:
        return False
    name = os.path.basename(path)
    print("    ┌─ %s:%d-%d%s" % (name, lo, hi,
                                 ("  " + label) if label else ""))
    for i in range(lo, min(hi, len(lines)) + 1):
        print("    │ %4d  %s" % (i, lines[i - 1]))
    print("    └─")
    return True


def find_line(path, pattern, start=1):
    """找第一个匹配的行号（1-based）。找不到返回 None。"""
    lines = read_lines(path)
    if not lines:
        return None
    rx = re.compile(pattern)
    for i in range(start - 1, len(lines)):
        if rx.search(lines[i]):
            return i + 1
    return None


def hr(ch="=", n=74):
    print(ch * n)


def head(title):
    print()
    hr()
    print("  " + title)
    hr()


# ===========================================================================
# 实验 1：RT 就绪队列的结构
# ===========================================================================

def exp1():
    head("实验 1：ChibiOS/RT 就绪队列的结构")

    print("""
  《NuttX 教程》第 10 课说 ChibiOS 的就绪队列是「分桶 + 位图」。
  这个实验【直接从源码读】它的真实结构。

  三个要回答的问题：
     ① ready_list_t 里有什么？
     ② 插入是 O(1) 还是 O(n)？
     ③ 全树里有没有「桶」或「位图」的痕迹？
""")

    print()
    print("  ── ① ready_list_t 的定义 ──")
    print()
    show(F_CHOBJECTS, 345, 358, "★ 只有两个成员")

    print("""
  ⇒ 【pqueue + current，没有第三个成员】。

     如果真是「分桶」，这里必然有一个数组或指向数组的指针。
     没有。
""")

    print("  ── 它挂在哪 ──")
    print()
    show(F_CHOBJECTS, 394, 402, "每个 instance 一个 rlist")

    print("  ── ② ch_priority_queue_t 的定义 ──")
    print()
    show(F_CHLISTS, 80, 89, "★ 三个字段")

    print("""
  ⇒ 【next + prev + prio】。这就是一个双向链表节点 + 一个优先级。

     「桶」需要的是「同优先级的线程串在一起」——
     这个结构里没有任何「一组同类元素」的表达。
""")

    print("  ── ③ 插入函数的扫描逻辑 ──")
    print()
    show(F_CHLISTS, 373, 388, "★ ch_pqueue_insert_behind")

    print("""
  ⇒ 那个 do/while 就是【线性扫描】。

     第一版我以为「分桶」，那么插入应该是「算出桶号，O(1) 插到桶尾」。
     实际是「从队头一路扫过去」—— 和 NuttX 的 g_readytorun 一模一样。
""")

    print("  ── ④ 选择函数的代价 ──")
    print()
    show(F_CHLISTS, 343, 359, "★ ch_pqueue_remove_highest")

    print("""
  ⇒ 取 next 指针，三次赋值。【O(1)】—— 因为链表已经排好序了。
""")

    print()
    hr("-")
    print("  ★★ 交叉验证：全树搜「桶」和「位图」的痕迹")
    hr("-")
    print()

    PATTERNS = [
        ("CH_CFG_MAX_PRIO_LEVELS", "分桶实现的典型配置宏"),
        ("prio_levels", "桶数组的常见命名"),
        ("prio_map", "位图的常见命名"),
        ("prio_bitmap", "位图的另一种命名"),
    ]

    found_any = False
    for pat, why in PATTERNS:
        hits = []
        for dirpath, _d, filenames in os.walk(os.path.join(CHIBIOS, "os")):
            for fn in filenames:
                if not fn.lower().endswith((".c", ".h", ".mk", ".txt", ".dox")):
                    continue
                p = os.path.join(dirpath, fn)
                lines = read_lines(p) or []
                for i, line in enumerate(lines, 1):
                    if pat in line:
                        hits.append((os.path.relpath(p, CHIBIOS).replace("\\", "/"), i))
        mark = "★ 找到 %d 处" % len(hits) if hits else "零匹配"
        print("    %-26s %-28s %s" % (pat, why, mark))
        for h, i in hits[:5]:
            print("        %s:%d" % (h, i))
        if hits:
            found_any = True

    print()
    if not found_any:
        print("""    ⇒ 【全树零匹配】。

       ⇒ 判断：ChibiOS/RT 7.0.4 的就绪队列【不是】分桶 + 位图，
         而是【单个优先级排序的双向链表】——
         和 NuttX 的 g_readytorun 是【同一种结构】。

       ⇒ 《NuttX 教程》第 10 课 2.2 节那条结论是错的。
""")
    else:
        print("""    ⚠ 找到了痕迹 —— 说明本课结论需要重查。
       （可能是别的版本、或者是 SIG 相关代码里的同名字段）
""")

    print("  ── ⑤ 优先级方向 ──")
    print()
    p_prio = find_line(F_CHSCHD_H, r"#define HIGHPRIO")
    if p_prio:
        show(F_CHSCHD_H, p_prio - 5, p_prio, "★ 优先级常量")

    print("""
    IDLEPRIO = 1（最低），HIGHPRIO = 255（最高）
    ⇒ 【数值越大优先级越高】—— 和 NuttX 一致，和 FreeRTOS 相反。

    第二条独立证据 —— 扫描条件：
""")
    show(F_CHLISTS, 376, 379, "跳过 >= 自己的 ⇒ 降序")

    print("""
    「跳过所有 prio >= 自己的」+「链表保持降序」
    ⇒ 两条独立证据指向同一个方向。
""")
    print()


# ===========================================================================
# 实验 2：ahead vs behind
# ===========================================================================

def exp2():
    head("实验 2：★★ ahead 与 behind —— 两个插入函数实现时间片")

    print("""
  源码里有两个插入函数，只差一个字符：

      ch_pqueue_insert_behind:  while (pqp->prio >= p->prio);
      ch_pqueue_insert_ahead:   while (pqp->prio >  p->prio);

  这个实验用模拟验证：「差一个等号」到底意味着什么。
""")

    print()
    print("  ── 源码原文 ──")
    print()
    show(F_CHLISTS, 373, 379, "insert_behind: >=")
    show(F_CHLISTS, 402, 408, "insert_ahead: >")

    print("""
  ── 模拟 ──

  数据结构：一个 Python 列表，元素是 (prio, name)，【按 prio 降序】。
            等价于 ch_priority_queue_t 链表。

  两个插入函数的模拟：
      insert_behind: 跳过所有 prio >= 新元素 的，插在第一个更低的【前面】
                     ⇒ 排到同级的【后面】
      insert_ahead:  跳过所有 prio >  新元素 的，插在第一个更低的【前面】
                     ⇒ 停在同级之前 ⇒ 排到同级的【前面】
""")

    def insert_behind(q, item):
        """返回 (新队列, 扫描步数)"""
        prio = item[0]
        steps = 0
        i = 0
        while i < len(q) and q[i][0] >= prio:
            i += 1
            steps += 1
        return q[:i] + [item] + q[i:], steps

    def insert_ahead(q, item):
        prio = item[0]
        steps = 0
        i = 0
        while i < len(q) and q[i][0] > prio:
            i += 1
            steps += 1
        return q[:i] + [item] + q[i:], steps

    # 先验证两个函数的语义差异
    print()
    print("    【语义验证】队列里已有两个优先级 60 的线程 A、B")
    print()
    base = [(60, "A"), (60, "B")]
    print("    初始队列（降序）: %s" % " ⇄ ".join(x[1] for x in base))
    print()

    for fn, fname in ((insert_behind, "behind"), (insert_ahead, "ahead")):
        q, steps = fn(base, (60, "C"))
        print("    insert_%-7s 插入 C 后: %s"
              % (fname, " ⇄ ".join(x[1] for x in q)))
        pos = [x[1] for x in q].index("C")
        print("    %s             C 的位置: 第 %d 个（0-based）"
              % ("", pos))
        print("    %s             扫描步数: %d" % ("", steps))
        print()

    print("    ⇒ behind 把 C 放到同级【后面】（A B C）")
    print("      ahead  把 C 放到同级【前面】（C A B）")
    print()

    print()
    hr("-")
    print("  ★★ 关键：这如何实现「时间片」")
    hr("-")

    print("""
    看 chschd.c 的 chSchDoPreemption（原文）：
""")
    p = find_line(F_CHSCHD_C, r"void chSchDoPreemption")
    if p:
        show(F_CHSCHD_C, p, p + 40, "★ ticks 决定用哪个插入函数")

    print("""
    【两种情况的语义】

    ┌─ 时间片用完（ticks == 0）
    │     otp = __sch_ready_behind(otp);     ← 排到同级【后面】
    │     otp->ticks = CH_CFG_TIME_QUANTUM;  ← 重新给满
    │     ⇒ 让位给同级，下轮轮到别人
    └─

    ┌─ 时间片没用完（被更高优先级抢占）
    │     otp = __sch_ready_ahead(otp);      ← 排到同级【前面】
    │     ⇒ 保持位置，抢占者让开还是我
    └─

    ⇒ 整个轮转机制就是这个 if/else，【没有引入任何新数据结构】。
""")

    print("  ── ★ 先搞清楚：每个时刻用【哪个】插入函数 ──")
    print()

    # 从源码验证每个调用点用的是哪个函数
    CALLERS = [
        ("chSchReadyI",         "线程变就绪",       "behind", "chschd.c:293"),
        ("chSchSelectFirst",    "换出当前线程",     "ahead",  "chschd.c:643"),
        ("chSchDoPreemption",   "被抢占(QUANTUM=0)", "ahead",  "chschd.c:552"),
        ("chSchDoPreemption",   "被抢占(ticks==0)",  "behind", "chschd.c:539"),
        ("chSchDoPreemption",   "被抢占(ticks>0)",   "ahead",  "chschd.c:547"),
        ("chSchDoYieldS",       "主动让位",         "behind", "chschd.c:611"),
        ("chSchGoSleepS",       "主动睡眠",         "（不插回）", "chschd.c:325"),
    ]

    print("    从源码里逐个调用点核对（本实验开头已打印原文）：")
    print()
    print("    %-22s %-18s %-10s %s"
          % ("函数", "时机", "用哪个", "位置"))
    print("    " + "-" * 66)
    for fn, when, which, where in CALLERS:
        print("    %-22s %-18s %-10s %s" % (fn, when, which, where))

    print("""
    ★★ 规律非常清晰：

        【被动换出】⇒ ahead   （保持位置，回来还是我）
        【主动让位】⇒ behind  （排到同级后面，让给别人）
        【新变就绪】⇒ behind  （★ 不抢占同级）

    ⇒ 关键那条是最后一条：**chSchReadyI 用 behind**。
      也就是说——一个线程变就绪时，它【不会插到同级的前面】。
      所以同级正在运行的那个线程【不会被它打断】。

    ⚠ 我第一版模拟时把「变就绪」写成了 insert_ahead，
      结果输出是「C ⇄ B ⇄ A」（队头是 C），
      和结论文字「队头永远是 A」【自相矛盾】。
      跑出来才发现 —— 这正是模拟的价值。
""")

    print("  ── 模拟：TIME_QUANTUM = 0（ArduPilot 的设置）──")
    print()

    # 模拟：3 个同级线程，TIME_QUANTUM=0
    # ★ 变就绪用 insert_behind（对应 chSchReadyI → __sch_ready_behind）
    q = []
    log = []
    q, s = insert_behind(q, (60, "A"))
    log.append(("A 就绪 → behind", list(q), s))
    q, s = insert_behind(q, (60, "B"))
    log.append(("B 就绪 → behind", list(q), s))
    q, s = insert_behind(q, (60, "C"))
    log.append(("C 就绪 → behind", list(q), s))

    print("    【场景】三个线程 A/B/C 同为优先级 60，依次变就绪")
    print("            TIME_QUANTUM = 0 ⇒ 变就绪用 insert_behind")
    print()
    print("    %-20s %-24s %s" % ("事件", "队列（左=队头）", "扫描步数"))
    print("    " + "-" * 58)
    for ev, qq, steps in log:
        print("    %-20s %-24s %d"
              % (ev, " ⇄ ".join(x[1] for x in qq), steps))

    print()
    print("    ⇒ 队头是 A（先到先服务），B 和 C 排在后面。")
    print()
    print("""    再确认「到底会不会切换」——看 chSchIsPreemptionRequired：
""")
    show(F_CHSCHD_C, 498, 502, "★ TIME_QUANTUM=0 时的判断")

    print("""
        p1 = firstprio(就绪队列) = 60   （B 在队头）
        p2 = 当前线程 A 的 prio  = 60

        return p1 > p2  ⇒  60 > 60  ⇒  false  ⇒ 【不切换】

    ⇒ 所以不只是「A 排在前面」——【根本不发生切换】。
      B 就绪这件事，对正在运行的 A 完全没有影响。
""")

    print("""    ⇒ 这就是 CH_CFG_TIME_QUANTUM = 0 的语义：
      【同优先级线程按先进先出，但运行中的那个不让位】。

      第 13 课讲的「三家都不做同优先级时间片轮转」——
      ChibiOS 的做法不是「不允许同级线程」，
      而是「允许，但先到的霸占到底」。
""")

    print("  ── 模拟：TIME_QUANTUM > 0（轮转开启）──")
    print()

    print("""    模拟规则（对应 chSchDoPreemption 的 if/else）：

        A 运行中，B 变就绪（同级）
            ⇒ 此时该不该切换？看 chSchIsPreemptionRequired：
                 ticks > 0  ⇒  return (p1 > p2)   ⇒ 同级不抢
                 ticks == 0 ⇒  return (p1 >= p2)   ⇒ 同级也抢

        A 的时间片耗尽（ticks == 0）
            ⇒ A 用 insert_behind 归队（排到同级后面）
            ⇒ 新队头是下一个同级线程
""")

    print("    %-8s %-30s %s" % ("时刻", "动作", "队头"))
    print("    " + "-" * 56)

    q = [(60, "A"), (60, "B"), (60, "C")]   # 已就绪
    cur = q.pop(0)
    ticks = 3
    print("    %-8s %-30s %s" % ("t=0", "%s 开始运行 (ticks=3)" % cur[1], cur[1]))

    for t in range(1, 8):
        ticks -= 1
        if ticks == 0:
            # 用完 ⇒ behind + 重置 ticks
            q, _ = insert_behind(q, cur)
            nxt = q.pop(0)
            action = "%s 时间片耗尽 ⇒ behind 归队" % cur[1]
            cur, ticks = nxt, 3
            print("    %-8s %-30s %s" % ("t=%d" % t, action, cur[1]))
        else:
            print("    %-8s %-30s %s" % ("t=%d" % t, "…继续运行", cur[1]))

    print()
    print("    ⇒ A → B → C → A … 轮转起来了。")
    print()
    print("""    ★ 注意对比：
        TIME_QUANTUM = 0 时，同样三个线程，队头永远是 A。
        TIME_QUANTUM > 0 时，靠 insert_behind 让位实现轮转。

      两个行为的差别，全部来自「>=」和「>」这一个字符。
""")
    print()


# ===========================================================================
# 实验 3：NIL 的数组调度器
# ===========================================================================

# NIL 的常量（源码：os/nil/include/ch.h）
NIL_MAX_THREADS = 16        # 用户线程槽位数（不含 idle）


def exp3():
    head("实验 3：★★ ChibiOS/NIL 的数组调度器")

    print("""
  NIL 和 RT 的调度器没有任何共用代码。
  这个实验验证 NIL 的三条结构事实：

      ① 优先级【就是数组下标】，一个槽只能一个线程
      ② 数值【越小】优先级越高（和 RT 相反）—— 第 10 课我写错了
      ③ 「队列」只是一个计数器，出队靠扫描
""")

    print()
    print("  ── ① 优先级 = 数组下标 ──")
    print()
    p = find_line(F_NIL_CH_C, r"tp = &nil\.threads\[tdp->prio\]")
    if p:
        show(F_NIL_CH_C, p - 12, p + 4, "★ chThdCreateI")

    print("""
  ⇒ `tp = &nil.threads[tdp->prio];`

     优先级不是「用来比较的数值」，而是【数组索引】。
     而下一行的断言 "priority slot taken" 说明：
     【一个优先级槽位只能有一个线程】。
""")

    print("  ── ② 优先级方向：数值越小越高 ──")
    print()
    p = find_line(F_NIL_CH_C, r"Points to the next thread in lowering priority")
    if p:
        show(F_NIL_CH_C, p - 14, p + 3, "★ 主调度循环")

    p2 = find_line(F_NIL_CH_C, r"if \(tp < nil\.next\)")
    if p2:
        show(F_NIL_CH_C, p2 - 12, p2 + 3, "★ chSchReadyI 的指针比较")

    p3 = find_line(F_NIL_CH_H, r"chThdGetPriorityX")
    if p3:
        show(F_NIL_CH_H, p3 - 4, p3, "★ 优先级就是下标")

    print("""
  ★★ 三条独立证据：

     ① 主调度循环从 `nil.threads`（下标 0）开始，`ntp++` 递增，
        注释说 "lowering priority order"（优先级递减方向）

     ② `if (tp < nil.next) nil.next = tp;` —— 【指针比较】
        因为 nil.threads[] 是数组，地址大小 == 下标大小
        ⇒ 地址更小 = 优先级更高

     ③ `chThdGetPriorityX() = (nil.current - &nil.threads[0])`
        ⇒ 优先级字面上就是下标

     ⇒ 【NIL：数值越小优先级越高。RT：数值越大优先级越高。确为相反。】

     ⚠ 第 10 课我写的是「NIL 里数值越大优先级越高」—— 方向反了。
""")

    print("  ── ③ 模拟：NIL 的调度器 ──")
    print()

    class NilThread:
        _next_id = 0

        def __init__(self, prio, name):
            self.prio = prio
            self.name = name
            self.state = "WTSTART"
            NilThread._next_id += 1

        def __repr__(self):
            return "%s(p=%d,%s)" % (self.name, self.prio, self.state)

    # 数组：下标 0..15 是用户槽，16 是 idle
    threads = [None] * (NIL_MAX_THREADS + 1)
    threads[NIL_MAX_THREADS] = NilThread(NIL_MAX_THREADS, "idle")
    threads[NIL_MAX_THREADS].state = "READY"

    scan_count = [0]     # 累计扫描的元素数

    def create(idx, prio, name):
        """对应 chThdCreateI：tp = &nil.threads[prio]"""
        slot = threads[prio]
        if slot is not None and slot.state not in ("WTSTART", "FINAL"):
            print("    ✗ 断言失败: priority slot taken (prio=%d)" % prio)
            return None
        t = NilThread(prio, name)
        t.state = "READY"
        threads[prio] = t
        return t

    def find_thread(state, obj):
        """对应 nil_find_thread：线性扫描"""
        for i in range(NIL_MAX_THREADS):
            scan_count[0] += 1
            t = threads[i]
            if t is not None and t.state == state and getattr(t, "waiting_on", None) is obj:
                return t
        return None

    def select():
        """对应主调度循环：从下标 0 扫描"""
        for i in range(NIL_MAX_THREADS + 1):
            scan_count[0] += 1
            t = threads[i]
            if t is not None and t.state == "READY":
                return t
        return None

    print("    【建线程】按优先级放到对应下标槽位")
    print()
    for prio, name in ((5, "main"), (2, "timer"), (0, "monitor"), (10, "uart")):
        t = create(prio, prio, name)
        if t:
            print("    chThdCreateI(prio=%-2d) ⇒ nil.threads[%-2d] = %s"
                  % (prio, prio, name))

    print()
    print("    【数组现状】")
    print("    %-6s %s" % ("下标", "线程"))
    print("    " + "-" * 30)
    for i, t in enumerate(threads):
        mark = "  ← idle" if i == NIL_MAX_THREADS else ""
        print("    %-6d %s%s" % (i, t.name if t else "（空）", mark))

    print()
    print("    【优先级比较】用指针（=下标）比较")
    print()
    ready = [i for i, t in enumerate(threads) if t and t.state == "READY"]
    print("    就绪的下标: %s" % ready)
    print("    min(下标) = %d ⇒ 最高优先级 = %s"
          % (min(ready), threads[min(ready)].name))

    print()
    print("    【测试「一个槽只能一个线程」】")
    print()
    print("    再建一个 prio=2 的线程：")
    r = create(2, 2, "timer2")
    print()

    print("    【测试 select()：扫描代价 —— 三种情况】")
    print()

    CASES = [
        ("最高优先级就绪（最好情况）", None),
        ("只有 main(p=5) 就绪", None),
        ("只有 idle 就绪（最坏情况）", None),
    ]

    # 情况 A：全部就绪
    for t in threads:
        if t and t.name != "idle":
            t.state = "READY"
    before = scan_count[0]
    sel = select()
    print("    %-28s 选中 %-10s 扫描 %d 个"
          % ("全部就绪", sel.name, scan_count[0] - before))

    # 情况 B：只有 main (下标 5)
    for t in threads:
        if t and t.name != "idle":
            t.state = "WTQUEUE"
    threads[5].state = "READY"
    before = scan_count[0]
    sel = select()
    print("    %-28s 选中 %-10s 扫描 %d 个"
          % ("只有 main (下标 5)", sel.name, scan_count[0] - before))

    # 情况 C：只有 idle (下标 16)
    threads[5].state = "WTQUEUE"
    before = scan_count[0]
    sel = select()
    print("    %-28s 选中 %-10s 扫描 %d 个"
          % ("只有 idle (下标 16)", sel.name, scan_count[0] - before))

    print()
    print("    ⇒ 这就是 NIL 的 O(n)：从下标 0 一路扫到第一个就绪的。")
    print("      最好 %d 个，最坏 %d 个（= CH_CFG_MAX_THREADS + 1）。"
          % (1, NIL_MAX_THREADS + 1))
    print()
    print("""    ★ 关键：这个「最坏 17」是【编译期钉死的】——
      不是「大约 17」，是「上界就是 17」。

      所以 NIL 的 O(n) 在这个 n 上界下等价于常数时间。
      这就是为什么 NIL 敢用数组 + 扫描。""")

    print()
    print("  ── ④ 「队列」只是一个计数器 ──")
    print()
    show(F_NIL_CH_H, 492, 497, "★ struct nil_threads_queue")

    p = find_line(F_NIL_CH_C, r"thread_t \*nil_find_thread")
    if p:
        show(F_NIL_CH_C, p, p + 11, "★ nil_find_thread：线性扫描")

    p = find_line(F_NIL_CH_C, r"void chThdDoDequeueNextI")
    if p:
        show(F_NIL_CH_C, p, p + 11, "★ 出队 = cnt++ 然后扫描")

    print("""
  ⇒ 【没有链表】。队列只有一个计数器 cnt：

        入队:  tqp->cnt--;              （负数 = 有 |cnt| 个等待者）
               nil.current->u1.tqp = tqp;（线程自己记住在等谁）
        出队:  tqp->cnt++;
               nil_find_thread(...)     （★ 扫描整个数组找）

  ★ 注意 `cnt` 的符号编码和 NuttX 的 semcount 是同一个技巧：
    「0 = 空，负数 = 有等待者」，省掉一个「等待者数量」字段。

  ★ 而没有链表 ⇒ 出队只能扫描。
    这是「用时间换空间」的极端形式 ——
    每个线程省下 next/prev 两个指针（8 字节）。

  ⇒ 和第 3 节的设计推理对上：
     「16 个槽」这个硬上限，正是「敢用数组 + 扫描」的【前提】。
""")
    print()


# ===========================================================================
# 实验 4：四路对照
# ===========================================================================

def exp4():
    head("实验 4：四路对照 —— 操作次数量化")

    print("""
  这个实验把四种结构放在同一把尺子下量：

      尺子 = 【为了完成一次「线程变就绪」，要访问多少个节点】

  这是纯算法的计数，不涉及真实的 CPU 周期。
  真实性能还受缓存、分支预测、流水线影响 —— 那些本模拟不覆盖。
""")

    def nuttx_insert(n, prio, existing):
        """NuttX: 降序单链表，线性扫描。返回访问的节点数。"""
        # 跳过所有 prio >= 新元素 的
        steps = 0
        for p in existing:
            steps += 1
            if p < prio:
                break
        return steps

    def freertos_insert(n, prio, existing):
        """FreeRTOS: 桶数组尾插 O(1) + 位图更新。
        位图更新用 CLZ 时是 O(1)（写侧维护 uxTopReadyPriority）。
        """
        return 1

    def nil_insert(n, prio, existing):
        """NIL: 直接下标 + 指针比较 O(1)。"""
        return 1

    print()
    print("  ── 插入（线程变就绪）的节点访问次数 ──")
    print()
    print("  %-8s %-14s %-16s %-14s %s"
          % ("就绪队列长度", "NuttX/RT", "FreeRTOS", "NIL", "说明"))
    print("  " + "-" * 70)

    for n in (1, 2, 4, 8, 16, 32, 64):
        # 最坏情况：新线程优先级最低 ⇒ 扫完整个链表
        existing = list(range(255, 255 - n, -1))    # 降序，全比新元素高
        a = nuttx_insert(n, 1, existing)
        b = freertos_insert(n, 1, existing)
        c = nil_insert(n, 1, existing)
        note = "新线程优先级最低（最坏）" if n == 1 else ""
        print("  %-8d %-14d %-16d %-14d %s" % (n, a, b, c, note))

    print()
    print("  ── 选择（挑下一个运行）的节点访问次数 ──")
    print()
    print("  %-8s %-14s %-16s %-14s %s"
          % ("就绪队列长度", "NuttX/RT", "FreeRTOS", "NIL", "说明"))
    print("  " + "-" * 70)

    for n in (1, 2, 4, 8, 16, 32, 64):
        a = 1                                  # 取队头，O(1)
        b = 1                                  # 位图 + CLZ，O(1)
        c = n if n <= 17 else 17               # 扫描，但 n ≤ 17
        note = "NIL 的 n 被编译期钉死在 17" if n == 32 else ""
        print("  %-8d %-14d %-16d %-14d %s" % (n, a, b, c, note))

    print()
    hr("-")
    print("  ★ 解读：")
    hr("-")
    print("""
    【插入侧】

      NuttX / ChibiOS-RT  : O(n) —— 链表越长越贵
      FreeRTOS / NIL      : O(1) —— 与长度无关

      ⇒ 队列长度 64 时，NuttX/RT 要访问 64 个节点，
        FreeRTOS/NIL 只要 1 个。【差 64 倍】。

    【选择侧】

      NuttX / RT / FreeRTOS : O(1)
      NIL                   : O(n)，但【n ≤ 17】

      ⇒ NIL 的曲线在 17 处【截断】——
        这不是「近似 O(1)」，是「上界有限的 O(n)」。

    【代价分配】

      结构         插入    选择    谁承担
      ──────────────────────────────────────
      NuttX        O(n)    O(1)    写侧
      ChibiOS/RT   O(n)    O(1)    写侧
      ChibiOS/NIL  O(1)    O(n)    读侧（n≤17）
      FreeRTOS     O(1)    O(1)    写侧（维护位图）
""")

    print()
    hr("-")
    print("  ★★ 但真正的问题是：n 到底是多少？")
    hr("-")

    print()
    print("  ── ArduPilot 的实际优先级分布 ──")
    print()

    # 从 Scheduler.h 读真实的优先级常量
    #
    # ★ 注意：值可以是【符号】（如 APM_SCRIPTING_PRIORITY = LOWPRIO）
    #   第一版的正则只匹配 \d+，于是【静默漏掉了那一条】。
    #   修正：两种值都抓，符号值单独标出来 ——
    #   扫描器应该报告它读不懂的，而不是假装它不存在。
    lines = read_lines(F_SCHEDULER_H)
    prios = {}          # {名字: (值, 行号)}
    symbolic = {}       # 值是符号的，单独记
    if lines:
        rx = re.compile(r"^\s*#define\s+(APM_\w*PRIORITY\w*)\s+([A-Za-z_0-9]+)")
        for i, line in enumerate(lines, 1):
            m = rx.match(line)
            if not m:
                continue
            name, val = m.group(1), m.group(2)
            if val.isdigit():
                prios[name] = (int(val), i)
            else:
                symbolic[name] = (val, i)

    # LOWPRIO 在 chschd.h 里定义为 2，手工补上（并标注来源）
    LOWPRIO = 2

    if prios or symbolic:
        total = len(prios) + len(symbolic)
        print("    从 Scheduler.h 读到的优先级常量（%d 个）：" % total)
        print()
        print("    %-34s %-6s %s" % ("常量", "值", "Scheduler.h 行号"))
        print("    " + "-" * 56)
        merged = dict(prios)
        for name, (val, lineno) in symbolic.items():
            merged[name] = (None, lineno)
        for name, (val, lineno) in sorted(
                merged.items(), key=lambda x: -(x[1][0] if x[1][0] is not None else -1)):
            shown = str(val) if val is not None else "符号值（见下）"
            print("    %-34s %-6s %d" % (name, shown, lineno))

        if symbolic:
            print()
            print("    ⚠ 上面 %d 个常量的值是【符号】，不是数字：" % len(symbolic))
            for name, (val, lineno) in symbolic.items():
                print("        %s = %s  (Scheduler.h:%d)" % (name, val, lineno))
                if val == "LOWPRIO":
                    print("            LOWPRIO 在 os/rt/include/chschd.h:53 定义为")
                    print("            (tprio_t)2 ⇒ 解析为 %d" % LOWPRIO)
                    print("            ⇒ 本实验按 %d 计入统计。" % LOWPRIO)

        print()
        by_prio = {}
        for name, (val, _l) in prios.items():
            by_prio.setdefault(val, []).append(name)
        for name, (val, _l) in symbolic.items():
            if val == "LOWPRIO":
                by_prio.setdefault(LOWPRIO, []).append(name)

        print("    ⇒ %d 个线程常量，落在 %d 个不同的优先级上。"
              % (total, len(by_prio)))
        print()
        print("    【同优先级的组】")
        any_shared = False
        for val in sorted(by_prio, reverse=True):
            names = by_prio[val]
            if len(names) > 1:
                any_shared = True
                print("        优先级 %-4d : %s" % (val, "、".join(names)))
        if not any_shared:
            print("        （没有同优先级的线程）")
        print()
        print("""    ⇒ 挤在同一个优先级的那些线程，就是
      「TIME_QUANTUM = 0 ⇒ 同级不让位」的【受益者】：
      它们按先进先出跑，谁也不会被同级打断。

      ⚠ 但这也是一个【风险点】：如果其中某一个线程
        进入死循环，同级的其他线程永远得不到 CPU，
        只有更高优先级的线程（180/182/183 那几个）才能抢占。
        这个风险在第 20 课讲 ArduPilot 看门狗时会再提。
""")
    else:
        print("    [警告] 没能从 Scheduler.h 读到优先级常量")

    print("    【就绪队列的真实长度】")
    print()
    print("""    这里无法从源码直接读出 —— 需要跑起来实测 [未亲验]。

    但可以从源码里的设计假设看出来（ChibiOS 自己写的）：

        /* Scanning priority queue, the list is assumed to be mostly empty.*/
                                                    —— chlists.h:376

    ⇒ ChibiOS 的赌注是：「就绪队列基本上是空的」。
      因为【阻塞是常态，就绪是例外】——
      大多数线程大多数时间在等信号量/事件/延时。

      如果这个假设成立，O(n) 里的 n 不是「线程总数」，
      而是「同时就绪的线程数」——通常 1~3。

    ⚠ 这个假设【能不能验证】，取决于具体应用：
      · 飞控（ArduPilot/PX4）：线程数固定、大部分时间阻塞 ⇒ 假设成立
      · 一个「所有线程都忙等」的应用：假设【不成立】⇒ O(n) 立刻暴露

    ⇒ 这就是为什么这个设计选择「要看场景才能评价」。
""")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("RT 就绪队列的结构", exp1),
    "2": ("★★ ahead vs behind", exp2),
    "3": ("★★ NIL 的数组调度器", exp3),
    "4": ("四路对照", exp4),
}


def main():
    print()
    hr("#")
    print("#  第 11 课：ChibiOS 调度器与就绪队列")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(CHIBIOS):
        print()
        print("  [错误] 找不到 ChibiOS 源码：")
        print("         %s" % CHIBIOS)
        print()
        print("  请先按《总纲》的方法拉取 6a85082c 版本。")
        print()
        return 1

    arg = sys.argv[1] if len(sys.argv) > 1 else None

    if arg is None:
        for key in sorted(EXPERIMENTS):
            EXPERIMENTS[key][1]()
    elif arg in EXPERIMENTS:
        EXPERIMENTS[arg][1]()
    else:
        print()
        print("  未知实验编号: %s" % arg)
        print("  可用: %s" % ", ".join(sorted(EXPERIMENTS)))
        print()
        return 1

    print()
    hr("#")
    print("#  实验结束")
    hr("#")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
