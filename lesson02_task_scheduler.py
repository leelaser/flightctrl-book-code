#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 2 课配套实验：任务与调度器

设计原则（延续三条纪律）：
  1. 每个论断都能落到 文件:行号
  2. [未亲验] 是诚实的标记，不是免责声明
  3. 模拟的地方明说丢掉了什么

运行：
    python lesson02_task_scheduler.py         # 全部
    python lesson02_task_scheduler.py 1       # 只跑实验 1

源码来源：D:/DEVLib/PX4APM/FreeRTOS/   （V11.1.0）
"""

import io
import _srcdirs
import os
import re
import sys

FREERTOS_ROOT = _srcdirs.src("FREERTOS")


def sec(title, ch="=", width=78):
    print()
    print(ch * width)
    print(title)
    print(ch * width)


# ============================================================================
# 实验 1：两种 taskSELECT 实现的对照 ⭐
# ============================================================================
#
# 模拟 FreeRTOS 就绪队列的两种实现，对比"选择最高优先级任务"的成本。
#
# 优化版（tasks.c:216-235，portmacro.h:163-168）：
#     记录  bitmap |= (1 << prio)
#     清除  bitmap &= ~(1 << prio)          ← 每次离开就绪队列都要做
#     选择  31 - clz(bitmap)                ← O(1)，一次得到答案
#
# 通用版（tasks.c:161-193）：
#     记录  if (prio > top) top = prio      ← 只增不减
#     清除  （空操作，tasks.c:202-206）
#     选择  while (list[top] 空) --top      ← 下探
#           top = 修正后的值                 ← 回写
#
# 丢掉的东西（重要）：
#   · 真实 CPU 周期。`if` / `|=` / `clz` 的周期数不同，
#     而且内存访问的 cache 行为也不同。本实验统计的是【操作数】，
#     不是时间。要精确对比得在 ARM 目标上实测。
#   · 链表操作本身的开销（两种实现都一样，所以不影响对比）
#   · 中断打断的影响


class ReadyQueuesBitmap:
    """优化版：位图 + CLZ。对应 configUSE_PORT_OPTIMISED_TASK_SELECTION == 1"""

    def __init__(self, n_prio):
        self.n_prio = n_prio
        self.lists = [[] for _ in range(n_prio)]   # 每个优先级一个 FIFO 桶
        self.bitmap = 0
        # 统计
        self.n_set = 0
        self.n_clear = 0
        self.n_clz = 0
        self.n_scan = 0
        self.max_scan = 0
        self.n_select = 0

    def add(self, name, prio):
        """taskRECORD_READY_PRIORITY：bitmap |= (1 << prio)"""
        self.lists[prio].append(name)
        self.bitmap |= (1 << prio)
        self.n_set += 1

    def remove(self, name, prio):
        """任务离开就绪队列 → taskRESET_READY_PRIORITY
        只有链表【空了】才清位（tasks.c:234-241）
        """
        self.lists[prio].remove(name)
        if not self.lists[prio]:              # listCURRENT_LIST_LENGTH(...) == 0
            self.bitmap &= ~(1 << prio)
            self.n_clear += 1

    def select(self):
        """taskSELECT_HIGHEST_PRIORITY_TASK：31 - clz(bitmap)

        返回 (任务, 优先级, 本次选择的开销)。
        本实现的开销**恒为 1**（一条 CLZ），与优先级档数、任务数无关。
        """
        if self.bitmap == 0:
            return None, None, 0
        # 模拟 __builtin_clz（ARM 上是一条 CLZ 指令）
        top = self.bitmap.bit_length() - 1     # == 31 - clz(bitmap) 对正整数
        self.n_clz += 1
        self.n_select += 1
        cost = 1                                # ← 恒定
        self.max_scan = max(self.max_scan, cost)
        # listGET_OWNER_OF_NEXT_ENTRY 会推进轮转游标，本实验用 pop(0)+append 模拟
        task = self.lists[top].pop(0)
        self.lists[top].append(task)
        return task, top, cost

    def stats(self):
        return {
            "维护": self.n_clear,          # 清位次数（写入端的成本）
            "选择次数": self.n_select,
            "下探总数": 0,
            "最坏单次下探": self.max_scan,
        }


class ReadyQueuesCache:
    """通用版：最大值缓存 + 循环下探。
    对应 configUSE_PORT_OPTIMISED_TASK_SELECTION == 0"""

    def __init__(self, n_prio):
        self.n_prio = n_prio
        self.lists = [[] for _ in range(n_prio)]
        self.top = 0          # uxTopReadyPriority：一个可能过期的最大值
        self.n_set = 0
        self.n_clear = 0
        self.n_clz = 0
        self.n_scan = 0
        self.max_scan = 0     # ★ 最坏单次下探次数 —— WCET 相关的量
        self.n_writeback = 0
        self.n_select = 0

    def add(self, name, prio):
        """taskRECORD_READY_PRIORITY（通用版）：if (prio > top) top = prio"""
        self.lists[prio].append(name)
        if prio > self.top:
            self.top = prio
        self.n_set += 1

    def remove(self, name, prio):
        """通用版把 taskRESET_READY_PRIORITY 定义成【空操作】（tasks.c:205）
        ⇒ 什么都不做，top 可能保持一个过期的（太高的）值
        """
        self.lists[prio].remove(name)
        # 真的是 0 —— 这是本实验要展示的关键差异

    def select(self):
        """taskSELECT_HIGHEST_PRIORITY_TASK（通用版）：
        while (list[top] 空) { assert(top); --top; }
        ...
        top = 修正后的值        ← 回写

        返回 (任务, 优先级, 本次选择的开销 = 下探次数)
        """
        if not any(self.lists):
            return None, None, 0
        top = self.top
        cost = 0
        self.n_select += 1
        while not self.lists[top]:
            cost += 1
            self.n_scan += 1
            assert top > 0, "所有桶都空（已在上层判过）"
            top -= 1
        self.max_scan = max(self.max_scan, cost)
        # 回写（tasks.c:192）
        if top != self.top:
            self.top = top
            self.n_writeback += 1
        task = self.lists[top].pop(0)
        self.lists[top].append(task)
        return task, top, cost

    def stats(self):
        return {
            "维护": 0,                     # 空操作（tasks.c:205）
            "选择次数": self.n_select,
            "下探总数": self.n_scan,
            "最坏单次下探": self.max_scan,
        }


def _run_scenario(cls, n_tasks, n_prio, n_ops, seed=12345):
    """跑一个确定性的场景：n_tasks 个任务，随机进出就绪队列 + 选择。

    用固定种子的 xorshift32 而不是 LCG，保证结果可复现
    （本教程的一条纪律：所有数字都要能重跑出来）。

    【第一版踩的坑】原来用 LCG rnd = (rnd*1103515245 + 12345) & 0x7FFFFFFF：
      乘数和增量都是奇数 ⇒ 最低位每步翻转 ⇒ 每两步回到原值。
      而我每轮恰好取两次随机数（一次选任务、一次决定要不要 select），
      于是 next_rand(2) 永远返回同一个值 —— 实验跑出「0 次 select」。
      这是经典的 LCG 低位相关性，用 xorshift + 取高位避开。
    """
    rnd = seed & 0xFFFFFFFF or 0x1234567

    def next_rand(n):
        """xorshift32，取高位再取模"""
        nonlocal rnd
        rnd ^= (rnd << 13) & 0xFFFFFFFF
        rnd ^= (rnd >> 17)
        rnd ^= (rnd << 5) & 0xFFFFFFFF
        rnd &= 0xFFFFFFFF
        return (rnd >> 8) % n          # 取高位，避开低位相关性

    q = cls(n_prio)
    names = ["t%d" % i for i in range(n_tasks)]
    prios = [i % n_prio for i in range(n_tasks)]
    in_ready = [False] * n_tasks
    n_select = 0

    for _ in range(n_ops):
        k = next_rand(n_tasks)
        if in_ready[k]:
            q.remove(names[k], prios[k])
            in_ready[k] = False
        else:
            q.add(names[k], prios[k])
            in_ready[k] = True
        # 交替做"选择"
        if next_rand(2) == 0 and any(in_ready):
            q.select()
    return q.stats()


def exp1_two_implementations():
    sec("实验 1：两种 taskSELECT 实现的对照 ⭐")

    print("""
  FreeRTOS 为"找出最高优先级的就绪任务"提供了两套实现，
  由 configUSE_PORT_OPTIMISED_TASK_SELECTION 选择。

  第 1 课讲过它的三层默认值：CM7 上如果用户不设，实际是 **1**（优化版）。

  本实验把两套实现都写成可运行代码，跑同样的任务序列。

  ⚠ 统计的是【操作数】，不是【时间】。见文件头"丢掉的东西"。
""")

    scenarios = [
        ("小规模（16 任务 / 8 档）", 16, 8, 3000),
        ("典型（40 任务 / 32 档）", 40, 32, 3000),
        ("大规模（128 任务 / 32 档）", 128, 32, 3000),
    ]

    print("  %-26s %-16s %-16s" % ("指标", "优化版(位图)", "通用版(缓存)"))
    print("  " + "-" * 62)
    for title, n_tasks, n_prio, n_ops in scenarios:
        s_bm = _run_scenario(ReadyQueuesBitmap, n_tasks, n_prio, n_ops)
        s_ca = _run_scenario(ReadyQueuesCache, n_tasks, n_prio, n_ops)
        print("  [%s]" % title)
        print("    %-24s %-16d %-16d"
              % ("写入端维护次数", s_bm["维护"], s_ca["维护"]))
        print("    %-24s %-16d %-16d"
              % ("选择次数", s_bm["选择次数"], s_ca["选择次数"]))
        print("    %-24s %-16d %-16d"
              % ("下探总数", s_bm["下探总数"], s_ca["下探总数"]))
        print("    %-24s %-16d %-16d   ← ★ WCET 相关"
              % ("**最坏单次下探**", s_bm["最坏单次下探"], s_ca["最坏单次下探"]))
        print()

    print("""  ================== 怎么读这张表 ==================

  【注意】本实验**故意不给出"总计/倍数"这样的单一指标。**
      第一版我给了，结果算出"通用版更便宜"—— 因为我把
      CLZ、清位、下探都当成"1 次操作"来计数，而它们的实际代价
      差很多（CLZ 是 1 条指令，下探是一次循环体含内存访问）。
      **加权不当的"总计"比没有总计更危险** —— 它会给出错误结论。

  ⇒ 改成看两个互相独立的量：

  【量 1】写入端维护次数
      优化版 > 0：每次某个优先级的桶变空，就要清一次位
      通用版 = 0：**完全不做维护**（tasks.c:205 定义成空操作）
      ⇒ 这是优化版为 O(1) 付出的代价

  【量 2】最坏单次下探次数  ← ★ 这个才是关键
      优化版：**恒为 1**，与优先级档数、任务数、就绪状态都无关
      通用版：**随配置变化**，最坏可达 configMAX_PRIORITIES

  实时系统关心的是【最坏执行时间】(WCET)，不是平均值。
      因为错过一次截止时间就是失效，平均再快也没用。

  ================== 为什么通用版的最坏情况会那么差 ==================

      通用版的 top 只增不减（taskRECORD 只做 `if (prio > top) top = prio`），
      而且离开就绪队列时【什么都不做】。

      极端情况：一个优先级 31 的任务曾经就绪过（top 变成 31），
      然后它阻塞了。此时：
          · 其他任务都在优先级 0
          · 每次 select 都要从 31 一路下探到 0  →  32 次循环

      而优化版的位图里，优先级 31 的位**已经被清掉了**，
      CLZ 直接返回 0 对应的位号 —— 一次搞定。

  ⇒ **这就是"为什么值得为位图付出 640 字节固定内存"的真正答案。**

  ================== 为什么 FreeRTOS 还要保留通用版？ ==================

      位图需要 configMAX_PRIORITIES <= 32（一个 uint32_t 装得下），
      而 CLZ 只有部分架构有。

      FreeRTOS 的第一目标是"能跑在所有 MCU 上" ——
      包括 AVR、MSP430 这些 8/16 位机（它们既没有 CLZ，
      也放不下 32 位位图）。

      ⇒ **通用版不是"备用实现"，而是"可移植性的门票"。**
        它的存在是这个设计能覆盖所有平台的前提。

      portmacro.h:158-160 的 #error 里还有一句：
          "It is very rare that a system requires more than 10 to 15
           difference priorities as tasks that share a priority will time slice."
      ⇒ FreeRTOS 假设你不会配超过 32 档，所以一个字就够。
""")


# ============================================================================
# 实验 2：listGET_OWNER_OF_NEXT_ENTRY 的轮转机制
# ============================================================================

def exp2_round_robin():
    sec("实验 2：同优先级轮转的实现 —— 状态存在链表里")

    print("""
  include/list.h:286-297 的 listGET_OWNER_OF_NEXT_ENTRY：

      #define listGET_OWNER_OF_NEXT_ENTRY( pxTCB, pxList )                    \\
      do {                                                                    \\
          List_t * const pxConstList = ( pxList );                            \\
          ( pxConstList )->pxIndex = ( pxConstList )->pxIndex->pxNext;         \\
          if( ( void * ) ( pxConstList )->pxIndex ==                           \\
              ( void * ) &( ( pxConstList )->xListEnd ) )                     \\
          {                                                                   \\
              ( pxConstList )->pxIndex = ( pxConstList )->xListEnd.pxNext;     \\
          }                                                                   \\
          ( pxTCB ) = ( pxConstList )->pxIndex->pvOwner;                       \\
      } while( 0 )

  **三行，实现了一个完整的轮转调度。**
""")

    # 用 Python 列表 + 一个游标模拟这个概念
    class CircularList:
        """模拟 FreeRTOS 的 List_t：
        双向循环链表 + 哨兵 xListEnd + 一个游标 pxIndex。
        本实验只用单向 + 游标，因为轮转只依赖 pxNext。
        """

        def __init__(self, name):
            self.name = name
            self.items = []          # 真实元素（不含哨兵）
            self.index = 0           # pxIndex（指向"上一次选中的位置"）
            self.sentinel_hit = 0    # 撞到哨兵的次数（用于演示绕回）

        def add(self, owner):
            """vListInsertEnd —— 队尾插入（同优先级 FIFO）"""
            self.items.append(owner)

        def remove(self, owner):
            self.items.remove(owner)
            if self.index >= len(self.items):
                self.index = 0

        def get_owner_of_next_entry(self):
            """listGET_OWNER_OF_NEXT_ENTRY：推进游标并返回所有者。

            注意：调用前链表必须非空，否则会取到哨兵（垃圾）。
            这就是 tasks.c:228 那个 configASSERT 存在的理由。
            """
            if not self.items:
                self.sentinel_hit += 1
                return "<哨兵 xListEnd 的 pvOwner：垃圾值！>"
            self.index = (self.index + 1) % len(self.items)
            return self.items[self.index]

    print("  " + "-" * 74)
    print("  场景：同一个优先级（比如 5）有 3 个任务 A/B/C")
    print("  " + "-" * 74)
    print()
    lst = CircularList("pxReadyTasksLists[5]")
    for n in ("A", "B", "C"):
        lst.add(n)

    print("  链表里的顺序（vListInsertEnd 队尾插入）：%s" % lst.items)
    print("  初始游标 pxIndex = %d（指向哨兵）" % lst.index)
    print()
    print("  连续调用 7 次 listGET_OWNER_OF_NEXT_ENTRY：")
    print()
    for i in range(7):
        before = lst.index
        owner = lst.get_owner_of_next_entry()
        print("      第 %d 次：pxIndex %d → %d   返回 %s"
              % (i + 1, before, lst.index, owner))

    print("""
  ⇒ 输出严格轮转 A → B → C → A → B → C → A ...

  **轮转状态（游标）存在链表里，不在任务里，也不在调度器里。**
""")

    print("  " + "-" * 74)
    print("  对照：三种「记住轮到谁了」的实现方式")
    print("  " + "-" * 74)
    print("""
      (a) 存在【链表】里 —— FreeRTOS 的做法
          代价：0 个额外字段
          好处：每个优先级桶各自独立轮转，天然隔离

      (b) 存在【任务】里（比如每个任务一个"上次运行时刻"）
          代价：每任务 4 字节
          好处：可以做更复杂的公平策略（比如按累计运行时间）

      (c) 存在【调度器】里（一个"同优先级组内索引"）
          代价：需要记"当前在哪个优先级的第几个"
          问题：换优先级时索引会失效

      FreeRTOS 选 (a)，是因为就绪队列本来就是"每优先级一条链表"
      —— 游标挂在链表上是最自然的，不需要任何额外结构。
""")

    print("  " + "-" * 74)
    print("  ⚠ 一个安静的副作用：读取也会推进轮转")
    print("  " + "-" * 74)
    print("""
      listGET_OWNER_OF_NEXT_ENTRY 是【推进式】的 —— 每次调用都动游标。

      而它被用在两个地方：
          1. taskSELECT_HIGHEST_PRIORITY_TASK()  ← 调度器的选择（推进是想要的）
          2. 一些遍历/查询 API                    ← 这里推进是副作用

      ⇒ **一段诊断代码遍历了就绪队列，会改变任务的轮转顺序。**

      这不是 bug（FreeRTOS 的语义就是"取下一个"），
      但它是一个真实的、值得知道的行为 ——
      调轮转相关的 bug 时，想想有没有别的地方也在读这个队列。
""")

    print("  " + "-" * 74)
    print("  ⚠ 空链表上的行为（这个才是真陷阱）")
    print("  " + "-" * 74)
    print()
    lst2 = CircularList("空链表")
    r = lst2.get_owner_of_next_entry()
    print("  在空链表上调用 → 返回：%s" % r)
    print()
    print("""  ⇒ C 代码里这一步是 `pxTCB = pxIndex->pvOwner`，
    而空链表时 pxIndex 指向哨兵 xListEnd，
    它的 pvOwner 是**未初始化的垃圾值**。

    把垃圾当 TCB 用 → 读写野指针 → 崩溃或静默数据损坏。

    **所以 tasks.c:228 那个 configASSERT 不是防御性编程，是必需品：**

        configASSERT( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ uxTopPriority ] ) ) > 0 );

    它守的是一个**注释里写了、但编译器不会检查**的不变式。

    ⇒ 这正好说明：**不变式靠断言守，不靠注释。**
      如果这段代码是你写的，那个 ASSERT 不能省。
""")


# ============================================================================
# 实验 3：PendSV 汇编逐行注释器
# ============================================================================

def exp3_pendsv():
    sec("实验 3：PendSV 上下文切换汇编 —— 逐行读")

    print("""
  port.c:493-555 的 xPortPendSVHandler，是 FreeRTOS 上下文切换的全部实现。
  本实验把它逐行拆开，并**直接从本机源码读取**做交叉验证。
""")

    p = os.path.join(FREERTOS_ROOT, "portable", "GCC", "ARM_CM7", "r0p1", "port.c")
    src_ok = os.path.isfile(p)
    if src_ok:
        txt = io.open(p, encoding="utf-8", errors="ignore").read()
        print("  ✓ 已找到本机源码：port.c")
        # 交叉验证几个关键指令确实存在
        checks = [
            ("mrs r0, psp", "取当前任务的栈指针"),
            ("ldr r3, pxCurrentTCBConst", "取 pxCurrentTCB 的地址"),
            ("vstmdbeq r0!, {s16-s31}", "FPU 上下文保存"),
            ("stmdb r0!, {r4-r11, r14}", "保存核心寄存器"),
            ("str r0, [r2]", "把栈顶写回 TCB 第一个成员"),
            ("bl vTaskSwitchContext", "★ 调用 C 函数选择新任务"),
            ("msr basepri, r0", "★ 用 BASEPRI 而非 cpsid i"),
            ("bx r14", "异常返回"),
        ]
        print()
        print("  %-34s %-8s %s" % ("关键指令", "在源码里", "作用"))
        print("  " + "-" * 84)
        for ins, desc in checks:
            found = "✓" if ins in txt else "✗"
            print("  %-34s %-8s %s" % (ins, found, desc))
        missing = [i for i, _ in checks if i not in txt]
        if missing:
            print()
            print("  ⚠ 以下指令未找到，可能版本不匹配：%s" % ", ".join(missing))
    else:
        print("  ✗ 未找到源码：%s" % p)

    print()
    print("  " + "-" * 74)
    print("  关键：`pxCurrentTCB` 被读了【两次】，中间夹着 bl")
    print("  " + "-" * 74)
    print("""
      ldr r2, [r3]            ← 读 pxCurrentTCB（旧任务）
      ...保存旧现场...
      str r0, [r2]            ← 存回旧 TCB
      bl  vTaskSwitchContext  ← ★★★ 只改 pxCurrentTCB 这一个指针
      ldr r1, [r3]            ← 再读 pxCurrentTCB（可能已经是新任务）
      ldr r0, [r1]            ← 从新 TCB 取栈顶
      ...恢复新现场...
      bx r14                  ← 硬件自动恢复 r0~r3/r12/lr/pc/xPSR

      ⇒ **上下文切换的全部秘密就是：换掉 pxCurrentTCB，
        然后从"新的那个"里恢复现场。**

  ================== 硬件帮你做了什么 ==================

      第 4 步只保存 r4-r11 和 r14。为什么不是全部？

      ARMv7-M 异常进入时【硬件自动压栈】：
          xPSR, PC, LR, R12, R3, R2, R1, R0        （8 个字）

      软件需要自己保存：
          R4 ~ R11     （被调用者保存寄存器）
          R14          （EXC_RETURN，记录"怎么返回"）
          S16 ~ S31    （如果用了 FPU —— 硬件只存 S0~S15）

      分工判据是 ABI 的 caller-saved / callee-saved 约定。
      **利用已有约定把压栈数量降到最少。**

  ================== 第 ⑧ 步之前的勘误规避 ==================

      "   cpsid i          \\n"   /* ARM Cortex-M7 r0p1 Errata 837070 workaround. */
      "   msr basepri, r0  \\n"
      "   dsb              \\n"
      "   isb              \\n"
      "   cpsie i          \\n"

      ⇒ **芯片勘误的规避代码出现在【每次上下文切换】都执行的路径上。**
        选芯片时，"勘误表有多少条影响性能路径"是一个真实的选型因素。
""")


# ============================================================================
# 实验 4：TCB 大小计算器
# ============================================================================

# TCB 的各字段（按 tasks.c:358-430 的顺序，只列条件编译相关的）
TCB_FIELDS = [
    # (字段, 字节数, 条件, 说明)
    ("pxTopOfStack", 4, None, "必须是第一个成员（汇编依赖）"),
    ("xMPUSettings", 0, "portUSING_MPU_WRAPPERS", "MPU 相关（本实验按 0 算）"),
    ("xStateListItem", 20, None, "ListItem_t：2 指针 + 1 owner + 1 值"),
    ("xEventListItem", 20, None, "同上"),
    ("uxPriority", 4, None, "当前优先级（PI 会临时改它）"),
    ("pxStack", 4, None, "栈起始地址"),
    ("pcTaskName", 16, None, "configMAX_TASK_NAME_LEN 默认 16"),
    ("pxEndOfStack", 4, "configRECORD_STACK_HIGH_ADDRESS", "栈顶地址"),
    ("uxCriticalNesting", 4, "portCRITICAL_NESTING_IN_TCB", "临界区嵌套计数"),
    ("uxTCBNumber/uxTaskNumber", 8, "configUSE_TRACE_FACILITY", "trace 用"),
    ("uxBasePriority", 4, "configUSE_MUTEXES", "★ PI 的归位点"),
    ("uxMutexesHeld", 4, "configUSE_MUTEXES", "持有互斥量计数"),
    ("pxTaskTag", 4, "configUSE_APPLICATION_TASK_TAG", "用户标签"),
    ("ulRunTimeCounter", 4, "configGENERATE_RUN_TIME_STATS", "运行时间统计"),
    ("ulNotifiedValue[1]", 4, "configUSE_TASK_NOTIFICATIONS", "任务通知值"),
    ("ucNotifyState[1]", 1, "configUSE_TASK_NOTIFICATIONS", "任务通知状态"),
    ("ucStaticallyAllocated", 1, "tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE", "静态/动态分配标记"),
    ("ucDelayAborted", 1, "INCLUDE_xTaskAbortDelay", "延时中断标记"),
]


def exp4_tcb_size():
    sec("实验 4：TCB 大小计算器 —— 配置选项怎么影响每个任务的开销")

    print("""
  TCB 的大小**不是固定的** —— 它随配置选项变化（大量 #if）。

  这在内存吃紧的 MCU 上很重要：40 个任务 × 每个 TCB 省 20 字节 = 800 字节。

  ⚠ 本实验的字节数是**按 32 位平台、字段自然对齐**估算的，
    不是编译器实测值。真实值受对齐规则、编译器、
    以及我没列的字段影响。量级感是对的，精确值要 `sizeof()` 实测。
""")

    presets = [
        ("最小配置", {
            "configUSE_MUTEXES": 0, "configUSE_TRACE_FACILITY": 0,
            "configUSE_TASK_NOTIFICATIONS": 0,
            "configRECORD_STACK_HIGH_ADDRESS": 0,
            "portCRITICAL_NESTING_IN_TCB": 0,
            "configUSE_APPLICATION_TASK_TAG": 0,
            "configGENERATE_RUN_TIME_STATS": 0,
            "tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE": 0,
            "INCLUDE_xTaskAbortDelay": 0,
        }),
        ("典型配置（有互斥量 + 任务通知）", {
            "configUSE_MUTEXES": 1, "configUSE_TRACE_FACILITY": 0,
            "configUSE_TASK_NOTIFICATIONS": 1,
            "configRECORD_STACK_HIGH_ADDRESS": 1,
            "portCRITICAL_NESTING_IN_TCB": 0,
            "configUSE_APPLICATION_TASK_TAG": 0,
            "configGENERATE_RUN_TIME_STATS": 0,
            "tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE": 1,
            "INCLUDE_xTaskAbortDelay": 0,
        }),
        ("全开（含 trace + 运行时间统计）", {
            "configUSE_MUTEXES": 1, "configUSE_TRACE_FACILITY": 1,
            "configUSE_TASK_NOTIFICATIONS": 1,
            "configRECORD_STACK_HIGH_ADDRESS": 1,
            "portCRITICAL_NESTING_IN_TCB": 1,
            "configUSE_APPLICATION_TASK_TAG": 1,
            "configGENERATE_RUN_TIME_STATS": 1,
            "tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE": 1,
            "INCLUDE_xTaskAbortDelay": 1,
        }),
    ]

    results = []
    for name, cfg in presets:
        total = 0
        included = []
        for field, size, cond, desc in TCB_FIELDS:
            if cond is None or cfg.get(cond, 0):
                total += size
                included.append((field, size, desc))
        results.append((name, total, included))

    print("  %-32s %-10s %s" % ("配置", "TCB 估算大小", "相对最小配置"))
    print("  " + "-" * 68)
    base = results[0][1]
    for name, total, _ in results:
        delta = total - base
        print("  %-32s %-10s %s"
              % (name, "%d 字节" % total,
                 "—" if delta == 0 else "+%d 字节/任务" % delta))

    print()
    print("  " + "=" * 74)
    print("  典型配置的字段明细")
    print("  " + "=" * 74)
    print()
    name, total, included = results[1]
    print("  %s（%d 字节）" % (name, total))
    print()
    print("  %-36s %-8s %s" % ("字段", "字节", "说明"))
    print("  " + "-" * 84)
    for field, size, desc in included:
        print("  %-36s %-8d %s" % (field, size, desc))

    print("""
  ================== 三个可以带走的点 ==================

  【1】Task 通知的存储是【按数组】分配的
      configTASK_NOTIFICATION_ARRAY_ENTRIES 默认 1（4 字节 + 1 字节状态）。
      如果你把它设成 8，每个 TCB 就多 8×5 = 40 字节。
      ⇒ 40 个任务 = 1600 字节。这是一个容易忽略的配置项。

  【2】configUSE_MUTEXES 影响的不只是"有没有互斥量功能"
      它还决定 uxBasePriority 和 uxMutexesHeld 存不存在（各 4 字节）。
      不用互斥量的话，每个任务省 8 字节。

  【3】栈才是大头，不是 TCB
      典型配置的 TCB ≈ 100 字节，而任务栈通常是 512~4096 字节。
      ⇒ **优化内存时先看栈，再看 TCB。**
        但反过来：「把 config 选项关掉」是零风险的内存优化，
        而「调小栈」是有风险的动作 —— 所以两个都值得做，
        顺序是先关选项，再压栈。
""")


# ============================================================================
# 主入口
# ============================================================================

EXPERIMENTS = {
    "1": ("两种 taskSELECT 实现的对照", exp1_two_implementations),
    "2": ("轮转机制与空链表陷阱", exp2_round_robin),
    "3": ("PendSV 汇编逐行读", exp3_pendsv),
    "4": ("TCB 大小计算器", exp4_tcb_size),
}


def main():
    print("""
================================================================================
 第 2 课：任务与调度器 —— pxReadyTasksLists[] 与 CLZ
 配套实验代码

 实验列表：
   1  两种 taskSELECT 实现的对照 ★
   2  轮转机制与空链表陷阱
   3  PendSV 汇编逐行读
   4  TCB 大小计算器
================================================================================
""")

    args = sys.argv[1:]
    if args:
        for a in args:
            if a in EXPERIMENTS:
                EXPERIMENTS[a][1]()
            else:
                print("未知实验编号: %s（可选：%s）"
                      % (a, ", ".join(sorted(EXPERIMENTS))))
    else:
        for k in sorted(EXPERIMENTS):
            EXPERIMENTS[k][1]()

    print("""
================================================================================
 本课三条最重要的结论
================================================================================

1. **就绪队列的两套实现，差别在"谁维护正确性"。**
   优化版（CM7 默认）：位图精确，写入端维护，读取端一条 CLZ。
   通用版：缓存一个可能过期的最大值，读取端循环下探并回写。
   优化版选对了 —— 因为"选择"发生在每次上下文切换上，
   而"维护"只在任务状态变化时发生。
   但通用版不是"备用实现"，而是**可移植性的门票**
   （AVR/MSP430 既没有 CLZ 也放不下 32 位位图）。

2. **同优先级轮转的状态存在链表里。**
   listGET_OWNER_OF_NEXT_ENTRY 用一个 pxIndex 游标实现轮转，
   零额外字段。代价是必须在非空链表上调用 ——
   这就是 tasks.c:228 那个 configASSERT 存在的理由。
   **不变式靠断言守，不靠注释。**

3. **"选择"和"切换"分离，而分离方式决定了 API 的形态。**
   vTaskSwitchContext() 只改 pxCurrentTCB 一个指针；
   PendSV 汇编保存旧现场、调用它、从新 TCB 恢复。
   因为用 BASEPRI 做临界区（不是 cpsid i），高优先级中断可以
   打断临界区 ⇒ 内核 API 必须分两族 ⇒ 必须有
   pxHigherPriorityTaskWoken ⇒ 用户必须显式 portYIELD_FROM_ISR。

   ⇒ 这深化了《NuttX 教程》第 10 课 4.1 节的结论：
     "切换推迟"是第二层原因，**"临界区手段"才是第一层原因**。
================================================================================
""")


if __name__ == "__main__":
    main()
