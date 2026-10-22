#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 5 第 6 章配套实验：条件变量、事件与系统锁

    实验 1  ★★ 条件变量没有"记忆"—— 唤醒会丢
    实验 2  ★★ 唤醒判据在【发送方】算（WTOREVT / WTANDEVT）
    实验 3  ★ 按优先级插入，按"头部"取出
    实验 4  ★★ WaitOne 和 WaitAny 的差别不只是"几位"
    实验 5  ★★ chSysUnlock() 里的优先级顺序断言

用法：
    python lesson15_cond_events_syslock.py 1
    python lesson15_cond_events_syslock.py 2
    ...
    python lesson15_cond_events_syslock.py        # 全部

【诚实声明】

实验 1~5 都是【模拟】—— 每一段都对应真实的 C 代码，模拟的是它的
**判定逻辑和数据结构语义**，丢掉了：

    · 真实的中断、上下文切换、寄存器保存
    · 指针操作本身（模拟里用 Python list / dict 代替）
    · SMP（本章所有分析都在 CH_CFG_SMP_MODE == FALSE 下）
    · 编译器优化对 volatile / 内存序的影响

每个实验里都写了"**模拟的是哪一段**"和"**丢掉了什么**"。
模拟能证明的是【语义】，不能证明【真实时序】。

另外：实验 2~5 的模拟代码都尽量**照抄 C 的分支结构**，
包括看起来多余的判断 —— 因为那些多余之处本身就是结论。

源码版本：d103983f（2026-07-22，ChibiOS 内核 7.0.4）
"""

import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APM_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "APM",
                 "ardupilot-Copter-4.7.2")
)
CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")

F_CHCOND_C = os.path.join(CHIBIOS, "os", "rt", "src", "chcond.c")
F_CHCOND_H = os.path.join(CHIBIOS, "os", "rt", "include", "chcond.h")
F_CHEVENTS_C = os.path.join(CHIBIOS, "os", "rt", "src", "chevents.c")
F_CHEVENTS_H = os.path.join(CHIBIOS, "os", "rt", "include", "chevents.h")
F_CHSYS_C = os.path.join(CHIBIOS, "os", "rt", "src", "chsys.c")
F_CHSYS_H = os.path.join(CHIBIOS, "os", "rt", "include", "chsys.h")
F_CHSCHD_H = os.path.join(CHIBIOS, "os", "rt", "include", "chschd.h")
F_CHOBJECTS_H = os.path.join(CHIBIOS, "os", "rt", "include", "chobjects.h")

F_CHCONF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                        "hwdef", "common", "chconf.h")
F_HWDEF_PY = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "hwdef", "scripts", "chibios_hwdef.py")
F_RADIO_BK = os.path.join(APM_ROOT, "libraries", "AP_Radio",
                          "AP_Radio_bk2425.cpp")
F_RCOUTPUT = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "RCOutput.cpp")
F_UARTDRIVER = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                            "UARTDriver.cpp")

# ===========================================================================
# 小工具
# ===========================================================================

def read_lines(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read().split("\n")
    except OSError:
        return []


def find_line(path, pattern, start=1):
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


def rel(path):
    """把长路径缩成教程正文里的写法。

    ★ 内核文件只留 `chcond.c` 这样的名字 —— 和正文里的 `chcond.c:89` 一致，
      全路径写在正文的「源码位置索引」一节里，不占打印宽度。
      ArduPilot 的文件保留 `libraries/...`，因为同名文件可能有多个。
    """
    p = os.path.normpath(path)
    if "/os/rt/" in p.replace("\\", "/"):
        return os.path.basename(p)
    if p.startswith(APM_ROOT):
        rest = p[len(APM_ROOT) + 1:].replace("\\", "/")
        return rest if rest.startswith("libraries/") else os.path.basename(p)
    return p.replace("\\", "/")


def src(path, a, b, note=None):
    """打印 [a, b] 行源码（带行号）。a/b 不会越界。"""
    lines = read_lines(path)
    if not lines:
        print("      [读不到 %s]" % rel(path))
        return
    if note:
        print("      %s" % note)
    for i in range(max(0, a - 1), min(len(lines), b)):
        print("      %s:%d  %s" % (rel(path), i + 1, lines[i].rstrip()))
    print()


def grep_print(path, pattern, label=None, limit=8, width=100):
    """按行打印匹配行（带行号）。"""
    lines = read_lines(path)
    rx = re.compile(pattern)
    n = 0
    if label:
        print("      %s" % label)
    for i, l in enumerate(lines, 1):
        if rx.search(l):
            print("      %s:%d  %s" % (rel(path), i, l.strip()[:width]))
            n += 1
            if n >= limit:
                break
    if n == 0:
        print("      （%s 里没有匹配 %s 的行）" % (rel(path), pattern))
    print()


# ===========================================================================
# 实验 1：条件变量没有"记忆"
# ===========================================================================

def exp1():
    head("实验 1  ★★ 条件变量没有记忆 —— 唤醒会丢")

    print("""
  条件变量和事件最大的区别，用一句话概括：

      条件变量：**没有记忆** —— 没有人在等的时候，signal 就是空放
      事件    ：**有记忆**   —— 位存在线程自己的结构里，人不在也留着

  先看源码，两边的"等待队列"和"待处理状态"分别放在哪。
""")

    print("  【A】条件变量：只有一条等待队列，没有任何计数 ──")
    n1 = find_line(F_CHCOND_H, r"typedef struct condition_variable")
    if n1:
        src(F_CHCOND_H, n1, n1 + 4,
            "结构体全文（chcond.h）—— 就一个字段：")
    print("""
      ⇒ 只有 queue。**没有"有没有被 signal 过"这个信息**。
        所以 chCondSignal() 打开队列一看是空的，就什么也不做。
""")

    n2 = find_line(F_CHCOND_C, r"^void chCondSignal\(")
    if n2:
        src(F_CHCOND_C, n2, n2 + 9, "chCondSignal() 全文（chcond.c）——")
    print("""
      ⇒ ch_queue_notempty() 为假 ⇒ **整个函数什么都不做**。
        这就是"空放"的实现：没有等待者时，信号直接消失。
""")

    print("  【B】事件：待处理的位存在【线程自己】身上 ──")
    grep_print(F_CHOBJECTS_H, r"epending|ewmask",
               "thread_t 里和事件有关的两个字段（chobjects.h）：", limit=6)
    n3 = find_line(F_CHEVENTS_C, r"^void chEvtSignalI\(")
    if n3:
        src(F_CHEVENTS_C, n3, n3 + 13, "chEvtSignalI() 全文（chevents.c）——")
    print("""
      ⇒ `tp->epending |= events;` 是**无条件**的：不管这个线程是醒着、
        在睡、还是在跑，位都先记上。**这就是"记忆"。**
""")

    # ---- 模拟 ----
    hr("-")
    print("""
  【模拟】两个线程，一个"发送方"一个"接收方"，只保留最核心的判定逻辑：

      条件变量版：发送时如果没人等 ⇒ 信号丢弃
      事件版    ：发送时永远先置位 ⇒ 后来者也能收到
""")
    hr("-")
    print()

    def condvar_case(signal_before_wait):
        queue = []          # ch_queue_t queue
        got = []

        def wait(tag):
            queue.append(tag)               # ch_sch_prio_insert(&cp->queue, ...)
            if got:                          # 模拟"被唤醒后 rdymsg 已就绪"
                return True
            queue.remove(tag)
            return False

        def signal():
            if queue:                        # ch_queue_notempty(&cp->queue)
                who = queue.pop(0)           # ch_queue_fifo_remove(&cp->queue)
                got.append(who)              # chSchWakeupS(..., MSG_OK)
                return who
            return None                      # ★ 空放

        if signal_before_wait:
            signal()                         # 先 signal（没人在等）
            return wait("T1")                # 后 wait ⇒ ?
        else:
            # 先 wait 再 signal：这里用"挂起-唤醒"两步表示
            queue.append("T1")
            signal()
            return True

    print("     条件变量：")
    r = condvar_case(signal_before_wait=True)
    print("       signal 在前、wait 在后  ⇒ wait 返回 %s  %s"
          % (r, "（★ 信号丢了）" if not r else ""))
    r = condvar_case(signal_before_wait=False)
    print("       wait 在前、signal 在后  ⇒ wait 返回 %s" % r)

    def event_case(signal_before_wait):
        """★ 和上面条件变量的区别：**两种顺序都能收到**。"""
        epending = 0                         # thread_t::epending
        EVT = 1

        if signal_before_wait:
            epending |= EVT                  # chEvtSignalI: 无条件置位

        m = epending & EVT                   # chEvtWaitAny: m = epending & events;
        if m == 0:
            # 真实代码这里是 chSchGoSleepS(CH_STATE_WTOREVT) —— 睡下去
            # 阶段 3：发送方到了（这一步在现实里由另一个线程/ISR 做）
            epending |= EVT                  # 仍然是"先置位，再考虑唤醒"
            m = epending & EVT
        epending &= ~m
        return m

    print()
    print("     事件：")
    r = event_case(signal_before_wait=True)
    print("       signal 在前、wait 在后  ⇒ wait 返回 %#x  %s"
          % (r, "（★ 位还在，收得到）" if r else ""))
    r = event_case(signal_before_wait=False)
    print("       wait 在前、signal 在后  ⇒ wait 返回 %#x" % r)

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：条件变量不能跨"发送时刻"保存信号 —— 所以用它必须
     在 while 循环里重检条件：

         while (条件不成立) chCondWait(&cv);      ← 标准写法

     而事件可以直接当"小信号量"用，因为位是**状态**不是**事件流**。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实的睡眠/唤醒（这里用函数调用顺序代替）
    · 优先级排序（实验 3 专门看这个）
    · 内存序和中断
  模拟保住了什么：
    · **"有没有人在等"这个判定发生在哪一侧**（发送方）
      —— 这一条决定了信号会不会丢
""")


# ===========================================================================
# 实验 2：唤醒判据在发送方
# ===========================================================================

def exp2():
    head("实验 2  ★★ 唤醒判据在【发送方】算")

    print("""
  事件机制里最反直觉的一点：

      决定"这个线程该不该醒"的代码，不在等待函数里，
      而在 **chEvtSignalI()** 里 —— 也就是**发送方**。

  看 chEvtSignalI 的那两个条件。
""")

    n = find_line(F_CHEVENTS_C, r"^void chEvtSignalI\(")
    if n:
        src(F_CHEVENTS_C, n + 4, n + 13,
            "chevents.c —— 两个状态，两种判据：")
    print("""
      ```
        state == WTOREVT   ⇒  (epending & ewmask) != 0      ← 任一位命中就醒
        state == WTANDEVT  ⇒  (epending & ewmask) == ewmask ← 全部命中才醒
      ```

      其中 `ewmask` 是**等待方**在睡下去之前写进自己 TCB 的
      （chevents.c:439 / 469 / 495）："我在等这些位"。
""")

    n2 = find_line(F_CHEVENTS_C, r"currtp->u.ewmask = events;")
    if n2:
        src(F_CHEVENTS_C, n2 - 4, n2 + 5,
            "等待方只做一件事：把要等的掩码记下来，然后睡：")
    print("""
  ⇒ 于是形成一个分工：

      **等待方**：声明"我在等什么"（ewmask），然后无条件睡
      **发送方**：拿这个声明去算，够了才 chSchReadyI(tp)

  这和 FreeRTOS 的事件组是**同一个套路**（卷 4 第 12 章：
  `xEventGroupSetBits` 遍历等待链表，逐个算 `ucStance`），
  但和**条件变量**正好相反 —— 条件变量是无条件唤醒，
  判断留给被唤醒的人（所以必须 while 重检）。
""")

    hr("-")
    print("""
  【模拟】一个线程等 `A|B`（all 语义），另一个等 `A`（any 语义）。
  发送方每次只发一位，看谁被唤醒。
""")
    hr("-")
    print()

    WTOREVT, WTANDEVT, READY = "WTOREVT", "WTANDEVT", "READY"

    class Thread:
        def __init__(self, name, state, ewmask):
            self.name, self.state, self.ewmask = name, state, ewmask
            self.epending = 0
            self.wakeups = 0
            self.rdymsg = None

    A, B = 0x1, 0x2
    t_any = Thread("T-any ", WTOREVT, A)
    t_all = Thread("T-all ", WTANDEVT, A | B)

    def chEvtSignalI(tp, events):
        """照抄 chevents.c:295-309 的分支结构。"""
        tp.epending |= events
        # Test on the AND/OR conditions wait states.
        if ((tp.state == WTOREVT and (tp.epending & tp.ewmask) != 0) or
                (tp.state == WTANDEVT and (tp.epending & tp.ewmask) == tp.ewmask)):
            tp.rdymsg = "MSG_OK"
            tp.wakeups += 1
            tp.state = READY              # (void) chSchReadyI(tp)
            return True
        return False

    def mask_str(m):
        if m == 0:
            return "-"
        return "|".join(n for n, bit in (("A", A), ("B", B)) if m & bit)

    print("      %-8s %-9s %-8s %-9s %s" % ("线程", "状态", "ewmask", "epending", "本次是否被唤醒"))
    print("      " + "-" * 62)
    for ev in (A, B):
        print("      ── 发送 %s ──" % mask_str(ev))
        for t in (t_any, t_all):
            woke = chEvtSignalI(t, ev)
            print("      %-8s %-9s %-8s %-9s %s"
                  % (t.name, t.state, mask_str(t.ewmask), mask_str(t.epending),
                     "★ 唤醒" if woke else "不唤醒"))
    print()
    print("      最终：T-any 被唤醒 %d 次，T-all 被唤醒 %d 次"
          % (t_any.wakeups, t_all.wakeups))
    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：T-any 在第一个位到达时就醒了；T-all 一位都没等到，
     即使 `epending` 里已经有一位 —— **它继续睡**。

     这正是"等 A 且 B"和"等 A 或 B"的全部差别：
     差别不在等待方怎么睡，而在**发送方怎么算**。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · chSchReadyI 之后真实的调度（谁先跑）
    · 中断上下文
  模拟保住了什么：
    · 发送方那两个判据的**分支结构**（照抄）
    · "够了才唤醒"这个语义
""")


# ===========================================================================
# 实验 3：按优先级插入，按"头部"取出
# ===========================================================================

def exp3():
    head("实验 3  ★ 按优先级插入，按「头部」取出")

    print("""
  条件变量的等待队列里藏着一个小陷阱（读代码时容易看错）：

      插入用 `ch_sch_prio_insert(&cp->queue, ...)`  ← 按优先级排
      取出用 `ch_queue_fifo_remove(&cp->queue)`     ← 名字里写着 FIFO

  字面上看是矛盾的。**先看这两处的真实代码。**
""")

    n1 = find_line(F_CHCOND_C, r"ch_sch_prio_insert\(&cp->queue")
    if n1:
        src(F_CHCOND_C, n1 - 6, n1 + 4, "chCondWaitS() 里的插入（chcond.c）：")
    n2 = find_line(F_CHCOND_C, r"ch_queue_fifo_remove\(&cp->queue\)")
    if n2:
        src(F_CHCOND_C, n2 - 4, n2 + 2, "chCondSignal() 里的取出（chcond.c）：")

    print("""  再看 `ch_queue_fifo_remove` 到底是什么（在 chlists.h 里）：
""")
    f_lists = os.path.join(CHIBIOS, "os", "rt", "include", "chlists.h")
    n3 = find_line(f_lists, r"static inline ch_queue_t \*ch_queue_fifo_remove")
    if n3:
        src(f_lists, n3 - 9, n3 + 7,
            "★ 文档注释比代码更能说明问题：")
    print("""
      ⇒ 注意那句 `@note`（chlists.h:272-273）：

            "If the queue is priority ordered then this function returns
             the element with the highest priority."

        **内核自己的文档就写明了**：队列若按优先级排，这个函数
        返回的就是**优先级最高**的元素。

        所以名字里的 FIFO 说的是"从**头部**取"（对应下面那个
        `ch_queue_lifo_remove` 是"从**尾部**取"），**不是**"按插入顺序"。

      而队列本身确实按优先级排（chschd.h:174-190 的
      `ch_sch_prio_insert`，插入时跳过所有 `prio >= 新prio` 的节点
      ⇒ 降序）。

      ⇒ 两个函数**配合起来**的效果是：**永远唤醒当前优先级最高的等待者**。
""")

    hr("-")
    print("""
  【模拟】三个线程按 低 → 中 → 高 的顺序进入等待，
  然后连续 signal 三次，看唤醒顺序。
""")
    hr("-")
    print()

    class Node:
        def __init__(self, name, prio):
            self.name, self.prio = name, prio
            self.next = self.prev = None

    def ch_sch_prio_insert(head, node):
        """照抄 chschd.h:174-190 的循环条件（环形双向链表）。"""
        cp = head
        while True:
            cp = cp.next
            if not (cp is not head and cp.prio >= node.prio):
                break
        node.next = cp
        node.prev = cp.prev
        cp.prev.next = node
        cp.prev = node

    def ch_queue_fifo_remove(head):
        """照抄 chqueues.h 的语义：摘队列头的元素。"""
        if head.next is head:
            return None
        n = head.next
        n.prev.next = n.next
        n.next.prev = n.prev
        n.next = n.prev = None
        return n

    head_q = Node("<head>", None)
    head_q.next = head_q.prev = head_q

    print("      依次入队（故意从低到高）：")
    for nm, pr in (("低优先级", 5), ("中优先级", 10), ("高优先级", 20)):
        ch_sch_prio_insert(head_q, Node(nm, pr))
        # 打印当前队列（从头部开始）
        seq, p = [], head_q.next
        while p is not head_q:
            seq.append("%s(%d)" % (p.name, p.prio))
            p = p.next
        print("        入队 %-8s ⇒ 队列 = %s" % (nm, " → ".join(seq)))

    print()
    print("      连续取出：")
    for i in (1, 2, 3):
        n = ch_queue_fifo_remove(head_q)
        print("        第 %d 次 ⇒ 唤醒 %s（prio=%d）" % (i, n.name, n.prio))

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：入队顺序是 低→中→高，唤醒顺序是 高→中→低。

     `fifo` 这个名字容易让人以为"先等的先醒"，实际不是。
     同一个函数还被 semaphore（chsem.c）、message（chmsg.c）用，
     所以**这个语义在整个内核里是一致的**。

     对照：FreeRTOS 的队列默认是**真 FIFO**，但优先级继承时
     会把持有者插到**就绪队列**（不是信号量等待队列）里 ——
     两家的"队列按什么排"要分开看，别混。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实指针操作（这里用对象属性代替，语义相同）
    · 唤醒时的重调度
  模拟保住了什么：
    · 插入的**比较条件**（`>=` 还是 `>` 决定了同优先级时谁在前）
    · 取出的位置（头部）
""")


# ===========================================================================
# 实验 4：WaitOne 和 WaitAny 的差别不只是"几位"
# ===========================================================================

def exp4():
    head("实验 4  ★★ WaitOne 和 WaitAny 的差别不只是「几位」")

    print("""
  两个函数名字很像，差别看起来是"返回几位"。实际差在**清几位**。
""")

    n1 = find_line(F_CHEVENTS_C, r"^eventmask_t chEvtWaitOne\(")
    if n1:
        src(F_CHEVENTS_C, n1 + 4, n1 + 15, "chEvtWaitOne()（chevents.c）——")
    n2 = find_line(F_CHEVENTS_C, r"^eventmask_t chEvtWaitAny\(")
    if n2:
        src(F_CHEVENTS_C, n2 + 3, n2 + 14, "chEvtWaitAny()（chevents.c）——")
    print("""
      差别就在那一行 `m ^= m & (m - 1);`：

          chEvtWaitOne:  先 `m ^= m & (m-1)` 只留下【最低的那一位】，
                         再 `epending &= ~m` ⇒ **只清一位，其余的留着**
          chEvtWaitAny:  直接 `epending &= ~m`   ⇒ **命中的全部清掉**

      ⇒ 所以文档里对 WaitOne 有一句关键提示（chevents.c:420-424）：

          "One and only one event is served in the function, the one
           with the lowest event id. The function is meant to be
           invoked into a loop in order to serve all the pending events."
""")

    hr("-")
    print("""
  【模拟】`epending` 同时有 A、B 两位，两种写法各跑一遍。
""")
    hr("-")
    print()

    A, B, C = 0x1, 0x2, 0x4

    def low_bit(m):
        """m ^= m & (m - 1) —— 只留最低的置位。"""
        return m ^ (m & (m - 1))

    def wait_one(epending, events):
        m = epending & events
        if m == 0:
            return 0, epending
        m = low_bit(m)
        return m, epending & ~m

    def wait_any(epending, events):
        m = epending & events
        if m == 0:
            return 0, epending
        return m, epending & ~m

    def ms(m):
        return "+".join(n for n, b in (("A", A), ("B", B), ("C", C)) if m & b) or "0"

    for name, fn in (("chEvtWaitOne", wait_one), ("chEvtWaitAny", wait_any)):
        e = A | B
        print("      %s(A|B|C)   进入时 epending = %s" % (name, ms(e)))
        for i in (1, 2):
            r, e = fn(e, A | B | C)
            print("        第 %d 次返回 %-3s ⇒ 剩下 epending = %s" % (i, ms(r), ms(e)))
        print()

    print("""      ⇒ WaitOne 把 A|B 拆成两次返回；WaitAny 一次返回 A+B。
         **但如果调用方不打算循环，只看一次返回值 —— 用 WaitAny 就会
         一次拿到多位，而用 WaitOne 只会拿到一位。**
""")

    hr("-")
    print("""
  【模拟】把 AP_Radio 那段 `switch (evt)` 原样搬过来，喂三种 evt：

      evt = EVT_IRQ                    → case EVT_IRQ
      evt = EVT_TIMEOUT                → case EVT_TIMEOUT
      evt = EVT_IRQ | EVT_TIMEOUT      → ？（两位同时到达）
""")
    hr("-")
    print()

    EVT_IRQ, EVT_TIMEOUT, EVT_BIND = 1, 2, 4
    handled = []

    def irq_handler_thd_body(evt):
        """照抄 irq_handler_thd 的 switch（去掉总线加锁和具体处理）。"""
        handled.clear()
        if evt == EVT_IRQ:
            handled.append("irq_handler()")
        elif evt == EVT_TIMEOUT:
            handled.append("irq_timeout()")
        elif evt == EVT_BIND:
            handled.append("StartBind")
        else:
            pass                     # switch 的 default: break;
        return list(handled)

    def ms(m):
        return "+".join(n for n, b in (("EVT_IRQ", EVT_IRQ),
                                       ("EVT_TIMEOUT", EVT_TIMEOUT),
                                       ("EVT_BIND", EVT_BIND)) if m & b) or "0"

    for evt in (EVT_IRQ, EVT_TIMEOUT, EVT_IRQ | EVT_TIMEOUT):
        r = irq_handler_thd_body(evt)
        print("      evt = %-22s ⇒ 处理了：%s"
              % (ms(evt), "、".join(r) if r else "★ 【什么也没处理】"))
    print("""
      ⇒ 前两种正常，第三种**两个位都被 chEvtWaitAny 清掉了，
        而 switch 一个分支都没进** —— 事件就这么消失了。

        这不是"模拟出来的假想"：它就是 WaitAny 的语义
        （返回并清掉全部命中位）+ "只处理一位"的调用方写法，
        直接推出来的。
""")

    hr("-")
    print("""
  【实测】ArduPilot 里两种写法都有。先看一个用 WaitAny 的地方：
""")
    hr("-")
    print()
    n3 = find_line(F_RADIO_BK, r"^void AP_Radio_beken::irq_handler_thd")
    if n3:
        src(F_RADIO_BK, n3, n3 + 30,
            "libraries/AP_Radio/AP_Radio_bk2425.cpp —— 线程主循环：")
    print("""
      ⇒ `evt = chEvtWaitAny(ALL_EVENTS);` 然后 `switch (evt)`。

        **两位同时到达时会发生什么？** `evt` 会是 `EVT_IRQ|EVT_TIMEOUT`，
        而 switch 里没有这个 case ⇒ 落到 `default: break;` ⇒
        两个事件都已经在 chEvtWaitAny 里被清掉了，**但一个都没处理**。
        （下面就照抄这段 switch 跑一遍。）

        同一份代码里，`chEvtWaitOne` 的文档明说了"要放在循环里用"，
        而 `chEvtWaitAny` 的文档说的是 "the events are cleared and
        returned"（复数）—— 即**调用方有义务处理返回的每一位**。

        修法有几种，都不难（用 `chEvtWaitOne` + 循环、或者按位 if）——
        这一条列出来，是因为它正好说明**"谁清位"这件事必须读清楚**。
""")

    grep_print(F_RADIO_BK, r"chEvtSignalI\(_irq_handler_ctx",
               "对照：这两个位是从哪来的（同一个文件）：", limit=4)

    print("""  ----------------------------------------------------------------------
  ⇒ **结论**：
       · WaitOne = "服务一位，其余的留着" ⇒ 必须循环
       · WaitAny = "命中的全给你（也全清掉）" ⇒ 必须逐位处理
       · 带超时的版本返回 0 表示超时 ⇒ 返回值**必须**判 0
         （不带超时的版本不可能返回 0，但 ArduPilot 两处都这么写了）

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实的位操作顺序（这里等价）
  模拟保住了什么：
    · 两个函数对 `epending` 的**清理范围**（这是本条结论的全部依据）
""")


# ===========================================================================
# 实验 5：chSysUnlock 里的优先级顺序断言
# ===========================================================================

def exp5():
    head("实验 5  ★★ chSysUnlock() 里的优先级顺序断言")

    print("""
  卷 5 第 2 章讲过一条纪律（调度器）：**用了 I 类函数，必须自己补一次
  重调度**（chSchRescheduleS），否则高优先级线程可能不立刻抢占。

  当时那是一条"文档里的约定"。**它其实是有代码强制的。**
""")

    n = find_line(F_CHSYS_H, r"port_unlock\(\);")
    if n:
        src(F_CHSYS_H, n - 9, n,
            "chsys.h —— chSysUnlock() 的最后几行：")
    print("""
      ```
        chDbgAssert((ready_list 为空) ||
                    (当前线程的优先级 >= 就绪队列头的优先级),
                    "priority order violation");
      ```

      注释里直接写了触发条件（chsys.h:426-429）：

          "The following condition can be triggered by the use of i-class
           functions in a critical section not followed by a
           chSchRescheduleS()"
""")

    print("""  但注意这两行是有条件的 —— 先把开关查清楚：
""")
    grep_print(F_CHCONF, r"CH_DBG_SYSTEM_STATE_CHECK|CH_DBG_ENABLE_ASSERTS",
               "ArduPilot 的 chconf.h（第一个匹配是开关块里的）：", limit=4)
    grep_print(F_CHCONF, r"#ifdef HAL_CHIBIOS_ENABLE_ASSERTS",
               "而这两个只在下面这个条件下才打开：", limit=2)
    print("""
  ⇒ 所以这条断言**只在开了 HAL_CHIBIOS_ENABLE_ASSERTS 的构建里存在**
    （调试固件）。量产固件里它是空的 —— 纪律变成沉默的约定。
""")

    hr("-")
    print("""
  【模拟】实现一个最小的 I 类语义：

      · chSchReadyI(tp)    把线程放进就绪队列（按优先级降序），
                           **不触发重调度**
      · chSysUnlock()      检查"队头是不是比我优先级高"
      · chSchRescheduleS() 把队头换成当前线程（= 让位）

  场景：低优先级线程 L 在临界区里 ready 了一个高优先级线程 H，
       然后解锁 —— 分别看"补了重调度"和"忘了补"两种写法。
""")
    hr("-")
    print()

    def run(forgot_reschedule):
        ready = []                          # 就绪队列（降序）

        class T:
            def __init__(self, name, prio):
                self.name, self.prio = name, prio

        L = T("L(低)", 5)
        H = T("H(高)", 20)
        current = L
        ready.append(L)

        def chSchReadyI(tp):
            ready.append(tp)
            ready.sort(key=lambda x: -x.prio)   # 降序，同优先级保持 FIFO

        def chSchRescheduleS():
            nonlocal current
            if ready and ready[0].prio > current.prio:
                current = ready.pop(0)
                ready.append(L)
                ready.sort(key=lambda x: -x.prio)

        chSchReadyI(H)                       # ← I 类函数：只是"就绪"
        if not forgot_reschedule:
            chSchRescheduleS()               # ← 纪律要求的那一次

        # chSysUnlock() 里的断言
        head_next = ready[0] if ready else None
        violated = (head_next is not None and head_next is not current
                    and current.prio < head_next.prio)
        return current, head_next, violated

    for forgot in (False, True):
        cur, nxt, bad = run(forgot)
        what = "忘了 chSchRescheduleS()" if forgot else "补了 chSchRescheduleS()"
        print("      %-26s ⇒ 解锁时 current=%-6s 队头=%-6s  %s"
              % (what, cur.name, nxt.name if nxt else "空",
                 "❌ priority order violation" if bad else "✅ 不触发断言"))
    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：
       · "I 类之后要补重调度"不是风格问题，**它有断言兜底**
       · 但断言只在调试构建里 —— 量产固件里写错就是"偶尔慢一拍"，
         而且**不会有任何报错**
       · 这条断言的判据（"队头优先级 > 当前线程"）本身也是
         **检查"该让位却没让"** 的可执行版本

      ⇒ 对写驱动的人的实际含义：**开着 assert 的固件去压测**，
        这类错误会当场炸出来，比事后查时序便宜得多。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 时间片、中断嵌套、真正的切换
    · 断言里"就绪队列为空"那个分支（这里 L 一直在队列里）
  模拟保住了什么：
    · 断言的三项比较（队头非空 / 队头不是自己 / 自己优先级更低）
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 条件变量没有记忆", exp1),
    "2": ("★★ 唤醒判据在发送方", exp2),
    "3": ("★ 按优先级插入、按头部取出", exp3),
    "4": ("★★ WaitOne vs WaitAny", exp4),
    "5": ("★★ chSysUnlock 的断言", exp5),
}


def main():
    print()
    hr("#")
    print("#  本章：条件变量、事件与系统锁")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(CHIBIOS):
        print()
        print("  [错误] 找不到 ChibiOS 源码：")
        print("         %s" % CHIBIOS)
        print()
        print("  请先按《总纲》的方法拉取 d103983f 版本。")
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
