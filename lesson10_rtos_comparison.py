#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 10 课配套实验：NuttX vs FreeRTOS vs ChibiOS

设计原则（和前 9 课一致）：
  1. 每个实验都能独立运行，输出的是**可验证的数字或结构**，不是形容词
  2. 凡是从源码里读出来的事实，都标注 文件:行号
  3. 凡是模拟，都说明模拟的是哪一段真实代码，以及模拟丢掉了什么
  4. 凡是我没能亲自验证的（来自文档/网络的），明确标注 [未亲验]

运行：
    python lesson10_rtos_comparison.py          # 全部实验
    python lesson10_rtos_comparison.py 3        # 只跑实验 3

本文件所有关于 NuttX 的数据，来源仓库：
    D:/DEVLib/PX4APM/PX4/PX4-Autopilot/platforms/nuttx/NuttX/nuttx
所有关于 ArduPilot/ChibiOS 的数据，来源仓库：
    D:/DEVLib/PX4APM/APM/ardupilot-Copter-4.5.7/ardupilot-Copter-4.5.7
"""

import sys
import time


# ============================================================================
# 0. 常量：三家 RTOS 的优先级区间（全部来自源码，不是记忆）
# ============================================================================

# --- NuttX ---
# include/sys/types.h:75,77
NUTTX_PRIORITY_MAX = 255      # #define SCHED_PRIORITY_MAX     255
NUTTX_PRIORITY_MIN = 1        # #define SCHED_PRIORITY_MIN       1
# sched/sched/sched_addreadytorun.c:79 —— 抢占判据
#     if (rtcb->lockcount > 0 && rtcb->sched_priority < btcb->sched_priority)
# 即「新任务数字更大 → 抢占当前任务」⇒ 数字大 = 优先级高

# --- ChibiOS ---
# 优先级的两个端点符号。数值来自 ArduPilot 的使用方式（见实验 2 的推导），
# ChibiOS 本体定义在 os/rt/include/chschd.h：
#     #define LOWPRIO    1
#     #define NORMALPRIO 128
#     #define HIGHPRIO   255
CHIBIOS_LOWPRIO = 1
CHIBIOS_NORMALPRIO = 128
CHIBIOS_HIGHPRIO = 255

# --- FreeRTOS ---
# configMAX_PRIORITIES 由 FreeRTOSConfig.h 定义，不是内核常量。
# 合法优先级是 0 .. configMAX_PRIORITIES-1，0 是最低。
# 内核源码 tasks.c 中 pxReadyTasksLists[] 的长度就是这个宏。
FREERTOS_MAX_PRIORITIES_TYPICAL = 7   # demo 工程常见取值，不是"默认值"


def sec(title, ch="=", width=78):
    print()
    print(ch * width)
    print(title)
    print(ch * width)


# ============================================================================
# 实验 1：就绪队列的数据结构 —— 单有序链表 vs 按优先级分桶的数组
# ============================================================================
#
# 这是三家 RTOS 在调度器实现上最本质的分歧，而且它直接决定了两件事：
#   (a) 一次任务唤醒要花多少次指针解引用
#   (b) 同优先级任务之间是否天然 FIFO
#
# NuttX 的真实实现（我逐行读过）：
#   sched/sched/sched_addreadytorun.c:88
#       else if (nxsched_add_prioritized(btcb, (FAR dq_queue_t *)&g_readytorun))
#   sched/sched/sched_addprioritized.c:82-85
#       for (next = (FAR struct tcb_s *)list->head;
#            (next && sched_priority <= next->sched_priority);
#            next = next->flink);
#   ⇒ g_readytorun 是**一个** dq_queue_t（双向链表），按优先级降序排，
#      插入时从表头线性扫描。查了 include/ 和 sched/ 全树，
#      **不存在**任何 pxReadyTasksLists[] 那样的按优先级分桶数组。
#
# FreeRTOS 的真实实现（[未亲验]，来自其公开源码结构）：
#   tasks.c 中 `List_t pxReadyTasksLists[ configMAX_PRIORITIES ];`
#   插入：vListInsertEnd(&pxReadyTasksLists[uxPriority], ...) —— O(1)
#   选择：uxTopReadyPriority（一个 uint32_t 位图，或 5.0 后可用
#         configUSE_PORT_OPTIMISED_TASK_SELECTION 走 CLZ 指令）—— O(1)
#
# ChibiOS 的真实实现（[未亲验]）：
#   os/rt/src/chschd.c 中 `ready_list_t ch.rlist;`
#   ready_list_t 是「按优先级的双向链表数组 + 一个位图索引」
#   ⇒ 介于两者之间：分桶，但桶的存在性用位图记

class NuttXReadyList:
    """模拟 NuttX 的 g_readytorun：单个按优先级降序排的双向链表。

    忠实复刻 sched_addprioritized.c 的插入循环语义：
      - 从头开始走，只要 新优先级 <= 当前节点优先级 就继续走
      - 停在第一个 优先级 < 新优先级 的节点前
    推论（重要）：同优先级的任务，新来的会被插到**已有同优先级任务之后**，
    所以同优先级天然 FIFO —— 这一点和 FreeRTOS 的 vListInsertEnd 行为一致。
    """

    def __init__(self):
        self.tasks = []      # 用 list 当链表；我们只关心**比较次数**，不关心指针开销
        self.derefs = 0      # 统计「节点访问次数」= 一次解引用

    def add(self, name, prio):
        """返回插入位置索引。derefs 累加本次线性扫描的节点访问数。"""
        i = 0
        while i < len(self.tasks):
            self.derefs += 1                 # 访问 self.tasks[i]
            if prio > self.tasks[i][1]:      # 停：新任务优先级更高
                break
            i += 1
        self.tasks.insert(i, (name, prio))
        return i

    def pick(self):
        """取最高优先级任务。链表有序 ⇒ 永远 O(1) 取表头。"""
        return self.tasks[0] if self.tasks else None


class FreeRTOSReadyList:
    """模拟 FreeRTOS 的 pxReadyTasksLists[configMAX_PRIORITIES] + uxTopReadyPriority。"""

    def __init__(self, max_prio=FREERTOS_MAX_PRIORITIES_TYPICAL):
        self.n = max_prio
        self.buckets = [[] for _ in range(max_prio)]
        self.top = 0        # uxTopReadyPriority：当前最高非空桶号
        self.derefs = 0

    def add(self, name, prio):
        assert 0 <= prio < self.n, "FreeRTOS 优先级必须落在 0..configMAX_PRIORITIES-1"
        self.derefs += 1                     # 一次数组索引
        self.buckets[prio].append((name, prio))
        if prio > self.top:
            self.top = prio                  # portGET_HIGHEST_PRIORITY()
        return prio

    def pick(self):
        self.derefs += 1                     # 一次数组索引 + 一次位图/CLZ
        b = self.buckets[self.top]
        return b[0] if b else None


def exp1_ready_list():
    sec("实验 1：就绪队列的数据结构 —— 单有序链表 vs 按优先级分桶数组")

    print("""
先把结论摆出来，再用数字验证：

  NuttX    : 单个有序双向链表 g_readytorun
             插入 O(n) 线性扫描，选择 O(1) 取表头
  FreeRTOS : 数组 pxReadyTasksLists[configMAX_PRIORITIES]，每桶一条 FIFO 链
             插入 O(1)，选择 O(1)（位图或 CLZ 指令）
  ChibiOS  : ready_list_t（分桶 + 位图），介于两者之间   [未亲验]

那么 NuttX 的 O(n) 到底有多贵？关键问题是 n 有多大。
""")

    # 用真实的 PX4 任务规模做量级估计：
    # PX4 fmu-v6x 上 nsh 的 `work_queue status` + `top` 通常看到
    # 40~80 个任务/线程。这里扫一遍 8..256。
    sizes = [8, 16, 32, 40, 64, 80, 128, 256]

    def nx_cost(order):
        """按指定顺序插入，返回 NuttX 链表的总节点访问次数。优先级 = 下标 + 1。"""
        nx = NuttXReadyList()
        for k in order:
            nx.add("t%d" % k, k + 1)
        return nx.derefs

    def fr_cost(order):
        """FreeRTOS 的插入成本与顺序无关，永远是 O(N)。"""
        fr = FreeRTOSReadyList(max_prio=256)
        for k in order:
            fr.add("t%d" % k, k)
        return fr.derefs

    # 关键：g_readytorun 按优先级【降序】排（表头 = 最高优先级）。
    #   · 升序插入 (1,2,3,...)     新任务优先级最高 → 插在表头 → 最好情况 O(1)
    #   · 降序插入 (N,N-1,...,1)   新任务优先级最低 → 每次扫到表尾 → 最坏 O(N)
    #   · 交错顺序                  → 平均 O(N/2)
    # 我第一版把方向写反了（以为升序是最坏），跑出来表里全是 1.0x 才发现。
    # 这个错误本身值得留在这里当提醒：**读实现时"哪个方向更贵"必须靠跑
    # 数字确认，不能靠直觉** —— 而这恰好也是本课要传达的方法论。

    print("  任务数 N | NuttX最好(升序) | NuttX平均(交错) | NuttX最坏(降序) | FreeRTOS | 最坏/FR")
    print("  ---------+------------------+-----------------+-----------------+----------+--------")

    for N in sizes:
        asc = list(range(N))
        desc = list(range(N - 1, -1, -1))
        # 交错顺序：偶数下标在前、奇数下标在后。确定性，可复现。
        inter = list(range(0, N, 2)) + list(range(1, N, 2))

        c_asc = nx_cost(asc)
        c_int = nx_cost(inter)
        c_desc = nx_cost(desc)
        c_fr = fr_cost(asc)

        print("  %8d | %16d | %15d | %15d | %8d | %6.1fx"
              % (N, c_asc, c_int, c_desc, c_fr, c_desc / c_fr))

    print("""
读这张表要抓住四点：

1) **NuttX 的成本强烈依赖插入顺序**，这是它和 FreeRTOS 最本质的区别。
   最坏情况（降序）是 O(N²)：N=80 约 3200 次解引用，N=256 约 32,000 次。
   FreeRTOS 在所有顺序下都是 O(N)。升序插入时 NuttX 甚至比 FreeRTOS
   还省 —— 因为它不用维护位图。

2) 但这是**总启动开销**，不是每次调度的开销。每次调度取最高优先级任务
   仍是 O(1)（取链表头）。运行期的真实成本是「任务从阻塞变就绪时的一次
   插入」：平均 N/2 次解引用。在 PX4 上这类事件是每毫秒几十次量级，
   每次几十次解引用 ≈ 几百纳秒，相对一次上下文切换（Cortex-M7 上约
   1~2 µs）完全可以忽略。
   ⇒ NuttX 的 O(n) 在飞控的任务规模下**不是问题**，这就是它敢这么写的原因。

3) 降序插入不只理论构造，它对应**低优先级模块后启动**这个真实场景：
   PX4 的 rcS 里高优先级模块（sensors / ekf2 / 姿态控制）先起，
   低优先级的 logger、mavlink、遥测后起 —— 接近降序。
   这也解释了为什么 NuttX 的启动时间里有相当一部分花在创建任务上：
   在 Cortex-M7 @480 MHz 上，32,000 次解引用约 0.5~1 ms。不致命，
   但它是可测的。这也解释了 FreeRTOS 为什么值得为位图付出静态内存。

4) 这个 O(n) 划出了 NuttX 的适用边界：几百个任务时线性扫描开始可观，
   上千个任务时它明显输给 FreeRTOS。反过来，FreeRTOS 的固定
   configMAX_PRIORITIES 数组在"任务数远小于优先级数"时是纯浪费
   （256 个空桶 ≈ 5 KB 静态 RAM，在 256 KB RAM 的 MCU 上不是小数）。
   ⇒ NuttX 用链表换来「优先级用满 1..255 且零固定内存开销」。
   这是**两种不同的取舍，不是谁更先进** —— 代价是真实的：
   NuttX 不适合几百个任务以上的系统。
""")

    # 再验证一个容易忽略的细节：同优先级是否 FIFO
    sec("  附：同优先级任务的排队顺序", "-")
    nx = NuttXReadyList()
    for nm in ("A", "B", "C"):
        nx.add(nm, 100)
    print("  NuttX  同优先级依次加入 A,B,C ⇒ 链表顺序 = %s" %
          [t[0] for t in nx.tasks])
    fr = FreeRTOSReadyList()
    for nm in ("A", "B", "C"):
        fr.add(nm, 5)
    print("  FreeRTOS 同优先级依次加入 A,B,C ⇒ 桶内顺序 = %s" %
          [t[0] for t in fr.buckets[5]])
    print("""
  两者都是 FIFO。但要注意：**FIFO 只保证"后来者排队尾"，
  不保证"不饿死"** —— 同优先级任务的公平性取决于是否开启时间片轮转：
      NuttX    : CONFIG_RR_INTERVAL > 0 才轮转，且要给任务设 timeslice
      FreeRTOS : configUSE_TIME_SLICING + configUSE_PREEMPTION
      ChibiOS  : CH_CFG_TIME_QUANTUM > 0 才轮转
  ArduPilot 的 chconf.h:179 把 CH_CFG_TIME_QUANTUM 设成了 **0**，
  即同优先级**不轮转**，纯协作式。这是一个非常重要的实时性设计选择，
  见实验 2 末尾。
""")


# ============================================================================
# 实验 2：优先级方向 —— 迁移时最容易搞错的一件事
# ============================================================================

def exp2_priority_direction():
    sec("实验 2：优先级方向与数值区间 —— 迁移陷阱")

    print("""
三家 RTOS 里有两家的方向是「数字大 = 优先级高」，另一家（PX4 的 NuttX 封装）
方向相反。搞错的后果是任务优先级完全颠倒 —— 而且**系统还能跑**，
只是负载一上来就出现莫名其妙的卡顿，极难定位。

先看硬证据（不是记忆，是源码）：
""")

    print("  [NuttX] sched/sched/sched_addreadytorun.c:79")
    print("      if (rtcb->lockcount > 0 && rtcb->sched_priority < btcb->sched_priority)")
    print("                                 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
    print("      新任务的 sched_priority **更大**时，判定为「会抢占当前任务」")
    print("      ⇒ 数字大 = 优先级高。区间 [SCHED_PRIORITY_MIN=1, MAX=255]")
    print("        （include/sys/types.h:75,77）")
    print()

    print("  [PX4 的封装] src/platforms/common/px4_work_queue / px4_task_spawn_cmd")
    print("      PX4 对外暴露的优先级是 **数字小 = 优先级高**（0 最高），")
    print("      内部用 PX4_PRIO_... 常量转换到 NuttX 的 sched_priority。")
    print("      ⇒ 同一个 NuttX 内核，PX4 在上面盖了一个方向相反的 API。")
    print()

    print("  [ChibiOS] 数字大 = 优先级高。这个结论**可以只用 ArduPilot 的代码证明**，")
    print("            不需要 ChibiOS 本体（它没被 checkout）。证据链有 4 条：")
    print()
    print("      证据 1  Scheduler.h:25-28")
    print("          #define APM_MONITOR_PRIORITY    183")
    print("          #define APM_MAIN_PRIORITY       180")
    print("          #define APM_TIMER_PRIORITY      181")
    print("          #define APM_RCOUT_PRIORITY      181")
    print("        Scheduler.h:49-52 的注释解释为什么 SPI 是 181：")
    print('          "SPI priority needs to be above main priority to ensure')
    print('           fast sampling of IMUs can keep up with the data rate"')
    print("        「above main priority」而 181 > 180 ⇒ 数字大 = 优先级高。")
    print()
    print("      证据 2  Scheduler.h:41-42")
    print("          #define APM_MAIN_PRIORITY       180")
    print("          #define APM_MAIN_PRIORITY_BOOST 182")
    print("        名字叫 BOOST（提升），而 182 > 180 ⇒ 数字大 = 优先级高。")
    print()
    print("      证据 3  Scheduler.h:34-35 的注释说 storage 线程")
    print('          "runs at just above IO priority"')
    print("        而 APM_STORAGE_PRIORITY=59，APM_IO_PRIORITY=58 ⇒ 同上。")
    print()
    print("      证据 4  Scheduler.cpp:704")
    print("          thread_priority = constrain_int16(priority_map[i].p + priority,")
    print("                                             LOWPRIO, HIGHPRIO);")
    print("        LOWPRIO 作为**下界**传入 ⇒ LOWPRIO=1 是最高优先级的下界... ")
    print("        等等，这里要小心：constrain(x, lo, hi) 的 lo=LOWPRIO=1，")
    print("        hi=HIGHPRIO=255。如果数字大 = 优先级高，那么下界应该是")
    print("        「最低优先级」=1，上界是「最高优先级」=255 —— 与 LOWPRIO=1 /")
    print("        HIGHPRIO=255 的命名**完全自洽**。四条证据同向。")

    print("""
  [FreeRTOS] 数字大 = 优先级高，区间 0 .. configMAX_PRIORITIES-1，0 是最低。
             configMAX_PRIORITIES 由 FreeRTOSConfig.h 定义，不同工程不同，
             demo 里常见 5~7，也可以用满 32（5.0 后位图是 uint32_t）。  [未亲验]

方向对照表：
""")
    rows = [
        ("NuttX 原生",   "sched_priority",   "[1, 255]",              "数字大 = 高"),
        ("PX4 on NuttX", "px4 优先级 API",   "[0, 255] 语义反向",     "**数字小 = 高**"),
        ("ChibiOS",      "tprio_t",          "[1, 255]",              "数字大 = 高"),
        ("FreeRTOS",     "UBaseType_t",      "[0, configMAX-1]",      "数字大 = 高"),
    ]
    print("  %-14s %-17s %-23s %s" % ("RTOS/封装", "优先级类型名", "区间", "方向"))
    print("  " + "-" * 70)
    for r in rows:
        print("  %-14s %-17s %-23s %s" % r)

    # 实际演示：把 PX4 的优先级数字直接塞给 ChibiOS 会怎样
    print("""
现在做一个真实的迁移演练。假设你在 PX4 上写了一个模块，优先级是
PX4 风格的 100（PX4 里 0 最高，所以 100 属于「偏低」）。现在要把它
移植到 ArduPilot/ChibiOS 上，你最直觉的做法是「直接用 100」。看看后果：
""")

    # NuttX 里 0 最高不可用（MIN=1），PX4 用 px4_task_spawn_cmd 传入的是
    # PX4 语义的优先级，内部转换成 NuttX 的 sched_priority：
    #   nuttx_priority = SCHED_PRIORITY_MAX - px4_priority  (近似，实际见
    #   platforms/nuttx/src/px4/common/px4_spawn.c / px4_task_spawn_cmd)
    def px4_to_nuttx(px4_prio):
        """PX4 语义(小=高) → NuttX 语义(大=高)。
        注意这是**示意**转换，真实转换在 px4_task_spawn_cmd 里，
        且带上下界钳制。这里只用来演示方向。"""
        return NUTTX_PRIORITY_MAX - px4_prio

    px4_prio = 100
    nuttx_prio = px4_to_nuttx(px4_prio)
    print("  PX4 侧优先级 = %d （PX4 语义：偏低的优先级）" % px4_prio)
    print("    → 换算到 NuttX sched_priority = 255 - %d = %d"
          % (px4_prio, nuttx_prio))
    print("    → NuttX 语义检查：%d 属于 %s" %
          (nuttx_prio,
           "比多数任务高" if nuttx_prio > 128 else "比多数任务低"))
    print()
    print("  现在把这个「100」直接抄到 ChibiOS：")
    print("    ChibiOS 上 100 < 128(NORMALPRIO)  ⇒ 比普通线程**低**"
          "（方向恰好一致，运气好）")
    print("  但把 PX4 的 30（PX4 语义 = 相当高的优先级）抄过去：")
    print("    换算到 NuttX = 225（高，对）")
    print("    ChibiOS 上 30  ⇒ 远低于 NORMALPRIO=128 ⇒ 变成最低档（**错**）")
    print()
    print("""  ⇒ 结论：**跨 RTOS 迁移时绝不要照搬优先级数字**。正确做法是搬语义档位。
     ArduPilot 正是这么做的 —— libraries/AP_HAL/Scheduler.h:104-117
     定义了一个与平台无关的语义枚举：

         enum priority_base {
             PRIORITY_BOOST, PRIORITY_MAIN, PRIORITY_SPI, PRIORITY_I2C,
             PRIORITY_CAN, PRIORITY_TIMER, PRIORITY_RCOUT, PRIORITY_LED,
             PRIORITY_RCIN, PRIORITY_IO, PRIORITY_UART, PRIORITY_STORAGE,
             PRIORITY_SCRIPTING, PRIORITY_NET,
         };

     每个 HAL 把同一组语义档位映射到自己的数值区间
     （ChibiOS 的映射表在 Scheduler.cpp:683-701）。这 14 个档位是
     ArduPilot 从 PX4/NuttX 时代继承下来的设计遗产 —— 注意
     `/libraries/` 下已经没有 AP_HAL_NuttX 了（只有 ChibiOS / ESP32 /
     Empty / Linux / SITL），但优先级档位这套抽象还留着 PX4 的形状。

  这是一个可以直接拿去做工程判断的结论：**任何跨 RTOS 的 HAL 抽象层，
   都必须把"优先级"抽象成语义档位而不是数字**，否则移植必然出错。
     反例：很多开源飞控的 HAL 直接把 FreeRTOS 的 numeric priority
     透传给应用层，一旦换 RTOS 就必须逐个驱动改数字。

  附：ChibiOS 上的一个真实选择
  -----------------------------
  ArduPilot 的 chconf.h:179 把 CH_CFG_TIME_QUANTUM 设为 0：

      #define CH_CFG_TIME_QUANTUM                 0

  含义是**同优先级任务之间不做时间片轮转**。结合 chconf.h:143 的
  CH_CFG_ST_TIMEDELTA = 10（tickless 模式，没有周期 tick），这解释
  了为什么 ArduPilot 的线程函数体几乎都是这种形状：

      void Scheduler::_timer_thread(void *arg) {
          while (true) {
              ...
              delay_microseconds(1000);      // 主动让出，自己维持 1 kHz
          }
      }

  它们靠**主动 sleep** 而非时间片来让出 CPU。
  代价：任何一个线程如果忘了 sleep 或陷入死循环，同优先级的兄弟线程
        会被永久饿死（ChibiOS 无 tick 可抢占）。收益：省掉了 tick 中断
        的固定开销，且同优先级线程之间的切换时机完全可预测。
  NuttX 的对应选择是 CONFIG_RR_INTERVAL（默认 0 = 不轮转），
  PX4 也没有开。三家在飞控场景下**都选择了不轮转** —— 这不是巧合，
  而是实时控制系统的共识：可预测性 > 公平性。
""")


# ============================================================================
# 实验 3：ISR 安全 API 的三种设计
# ============================================================================

def exp3_isr_api():
    sec("实验 3：ISR 安全 API 的三种设计 —— FromISR / I 后缀 / 无后缀")

    print("""
这是三家 RTOS 差异最直观、也最能体现设计哲学分歧的地方。

  FreeRTOS ：**独立函数族**，后缀 FromISR
             xSemaphoreGiveFromISR(sem, pxHigherPriorityTaskWoken)
             xQueueSendFromISR(q, item, pxHigherPriorityTaskWoken)
             vTaskNotifyGiveFromISR(task, pxHigherPriorityTaskWoken)
             + 结尾必须调用 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken)
             在任务上下文调用 FromISR 版本（或反之）会被 configASSERT 抓出来。

  ChibiOS  ：**独立函数族**，后缀 I（Interrupt-safe）
             chSemSignalI(sem) / chBSemSignalI(sem) / chEvtSignalI(...)
             chVTSetI(...) / chMtxUnlockS(...)
             必须在 chSysLockFromISR() / chSysUnlockFromISR() 之间调用。
             **没有 configASSERT 级别的保护** —— 用错了是 UB，不报错。

  NuttX    ：**同一个函数**。nxsem_post() 既能从任务调用也能从 ISR 调用。
             我在 sched/semaphore/sem_post.c:66-68 读到的函数头注释原文：

                 * Assumptions:
                 *   This function may be called from an interrupt handler.

             函数体第一件事就是进临界区（sem_post.c:97-100）：

                 flags = enter_critical_section();
                 ...
                 NOTE: If this sched_lock is called from an interrupt handler, it
                 will do nothing.

这三条路线的差异不是风格问题，是**架构约束**推出来的：

  FreeRTOS 为什么必须分成两个函数族？
    1) 性能：ISR 版本跳过了「是否需要重新调度」的完整判断，把决策推迟
       到 ISR 退出时一次性做（pxHigherPriorityTaskWoken 就是那个"待办标记"）
    2) 可移植性：FromISR 版本允许 port 在 ISR 里使用与任务不同的临界区
       手段（比如 BASEPRI 屏蔽 vs 关中断）。在不支持 BASEPRI 的旧架构
       （AVR、MSP430、部分 8/16 位）上这是**必须**的 —— 那里没有"只关
       内核级中断"这种能力，进 ISR 就得整套不同的锁策略。
    3) configMAX_SYSCALL_INTERRUPT_PRIORITY 的存在：高于这个优先级的中断
       根本不允许调用任何 FreeRTOS API。分成两个函数族让"谁可以调"这件事
       在编译期就能看出一半。

  ChibiOS 为什么也是两个函数族？
    ChibiOS 的 chSysLock() 在 Cortex-M 上不是关中断，而是把 BASEPRI
    提到 CORTEX_MAX_KERNEL_PRIORITY（只屏蔽内核级中断）。
    在 ISR 里你不能"再锁一次"——所以有独立的 *I / *S / *X 后缀体系：
        I = 调用者已持锁，且处于 ISR 上下文
        S = 调用者已持锁（Service，可在 ISR 或已锁的任务上下文）
        X = 调用者已持锁，不做断言检查（最快）

  NuttX 为什么能统一？
    因为在 Cortex-M 上 enter_critical_section() **就是** up_irq_save()
    也就是 cpsid i / primask，这个操作在任务上下文和 ISR 上下文里
    **完全一样** —— 都不需要"再锁一次"的概念。
    NuttX 的"我要不要在返回时切任务"的决策，是通过
    arm_doirq() **返回值**这个机制推迟的（第 7 课详细讲过）：

        uint32_t *arm_doirq(int irq, uint32_t *regs) {
          ...
          irq_dispatch(irq, (uint32_t *)CURRENT_REGS);
          if (regs == NULL) {
            regs         = (uint32_t *)CURRENT_REGS;  // 可能已经是别的任务
            CURRENT_REGS = NULL;
          }
          return regs;      // ★ 返回一个【不同的栈指针】= 切任务
        }

    ⇒ NuttX 的上下文切换机制本身就是"推迟到 ISR 退出"的，所以 ISR 里
      调 nxsem_post() 不需要任何特殊变体 —— 后续的调度决策会自动发生。

**这是一个可以推广的结论**：
    "API 需不需要 ISR 变体"不取决于 OS 的作者喜好，而取决于
    "上下文切换在哪里发生"。切换点推迟到 ISR 退出 → 不需要变体；
    切换点在调用点就地发生 → 必须给 ISR 一个不同的入口。
""")

    # 用一段"同一件事的三种写法"把差异钉死
    print("  同一件事：ISR 里给一个信号量 post，唤醒等待中的高优先级任务")
    print("  " + "-" * 70)
    print()
    print("  [FreeRTOS]")
    print("    void EXTI0_IRQHandler(void) {")
    print("        BaseType_t xHigherPriorityTaskWoken = pdFALSE;")
    print("        xSemaphoreGiveFromISR(g_sem, &xHigherPriorityTaskWoken);")
    print("        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);   // ← 必须")
    print("    }")
    print()
    print("  [ChibiOS]")
    print("    void EXTI0_IRQHandler(void) {")
    print("        chSysLockFromISR();")
    print("        chSemSignalI(&g_sem);          // ← I 后缀")
    print("        chSysUnlockFromISR();          // ← 退出时自动重调度")
    print("    }")
    print()
    print("  [NuttX]")
    print("    int exti0_isr(int irq, void *context, void *arg) {")
    print("        nxsem_post(&g_sem);            // ← 和任务里调的是同一个")
    print("        return OK;                     // ← 不需要显式 yield；")
    print("    }                                  //   arm_doirq() 的返回值兜底")
    print()

    # 模拟：三种 API 的"误用检测能力"
    print("  误用检测能力对比（把 ISR 版本用在任务里 / 反之）：")
    print()
    misuse = [
        ("在 ISR 里调任务版 API",
         "configASSERT 命中，立刻停机",
         "UB，通常表现为断言失败或静默错误",
         "合法，本来就允许"),
        ("在任务里调 ISR 版 API",
         "configASSERT 命中（portASSERT_IF_INTERRUPT_PRIORITY_INVALID）",
         "UB",
         "不存在这种 API"),
        ("高优先级中断里调内核 API",
         "configASSERT 命中（优先级高于 configMAX_SYSCALL_...）",
         "依赖 NVIC 优先级配置，用错是 UB",
         "NuttX 无此约束；中断优先级由板级自行决定"),
    ]
    print("  %-26s %-34s %-34s %s"
          % ("误用场景", "FreeRTOS", "ChibiOS", "NuttX"))
    print("  " + "-" * 104)
    for r in misuse:
        print("  %-26s %-34s %-34s %s" % r)

    print("""
  三条工程结论：

  1) FreeRTOS 的 FromISR 家族虽然啰嗦（每个 ISR 都要声明 woken 变量 +
     结尾 yield），但它是**唯一一个能在编译期/运行期抓住误用的设计**。
     在安全关键场景这是真实优势。

  2) ChibiOS 的 *I/*S/*X 后缀体系最"薄"—— 没有额外机制，纯靠纪律。
     好处是零开销，代价是 bug 只能靠 symptom 反推。
     ArduPilot 为此付出了代价：它必须自己写 try_force_mutex()
     （Scheduler.cpp:856-888）来从死锁里自救，见实验 5。

  3) NuttX 的统一 API 最简洁，但它把"我现在是不是在 ISR 里"这个判断
     交给了调用者（或者交给 enter_critical_section 的实现）。
     代价是**可移植性**：换到 PRIMASK 之外还需要别的锁机制的架构上，
     统一 API 就维持不下去了（这也是为什么 PX4 只支持 ARMv7-M/ARMv8-M）。
""")


# ============================================================================
# 实验 4：中断 → 线程的通信原语映射
# ============================================================================

def exp4_ipc():
    sec("实验 4：中断 → 线程通信原语的映射")

    print("""
这是选型时最容易低估的一块：**驱动代码 90% 的复杂度不在寄存器操作，
而在"ISR 怎么把数据交给线程"**。三家提供的原语完全不同，直接决定
驱动代码的形状。

按能力对照（✓ = 有，✗ = 没有，△ = 有但语义不同）：
""")

    # 表格数据：[原语, FreeRTOS, ChibiOS, NuttX]
    prims = [
        ("计数信号量",       "✓ xSemaphoreCreateCounting", "✓ semaphore_t",            "✓ sem_t"),
        ("二值信号量",       "✓ xSemaphoreCreateBinary",  "✓ binary_semaphore_t",     "✓ sem_t(初值0)"),
        ("互斥量(带PI)",     "✓ xSemaphoreCreateMutex",   "✓ mutex_t(PI 不可关)",      "✓ mutex_t = sem_t"),
        ("递归互斥",         "✓ xSemaphoreCreateRecursiveMutex", "✓ CH_CFG_USE_MUTEXES_RECURSIVE", "✓ rmutex_t"),
        ("队列/消息",        "✓ xQueueCreate",            "✓ CH_CFG_USE_MESSAGES",     "✓ POSIX mqueue"),
        ("邮箱",             "✗",                          "✓ mailbox_t",              "~ mq 的一种用法"),
        ("事件组/事件标志",  "✓ EventGroup (≥8.0)",       "✓ Event Source + Flags",    "✗ **没有**"),
        ("任务通知",         "✓ xTaskNotify* (每个任务1个32位)",
                                                           "✗",                        "✗"),
        ("虚拟定时器/软件定时器", "✓ xTimerCreate",        "✓ chVTSet + chVTSetI",      "✓ wd_start (watchdog)"),
        ("pthread 条件变量", "✗ (需自己用信号量搭)",       "✓ CH_CFG_USE_CONDVARS",     "✓ POSIX 完整"),
        ("信号",             "✗",                          "✗",                         "✓ POSIX 完整"),
        ("poll/select",      "✗",                          "✗",                         "✓ 完整"),
        ("文件描述符/VFS",    "✗",                          "✗",                         "✓ 完整"),
    ]
    print("  %-22s %-30s %-30s %s"
          % ("原语", "FreeRTOS", "ChibiOS", "NuttX"))
    print("  " + "-" * 110)
    for r in prims:
        print("  %-22s %-30s %-30s %s" % r)

    print("""
这张表里有三个值得展开的行：

【行 7：事件组 —— NuttX 没有，这是它最反直觉的缺口】
  FreeRTOS 的 EventGroup 是"一个 32 位变量 + 等待任意位/所有位"，
  ChibiOS 的 Event Source 是"一个 eventmask_t（32位）+ chEvtWaitAny/
  chEvtWaitAll"。它们在驱动里极其好用，因为一个驱动经常需要等
  "RX 半满 或 错误 或 超时" 这种组合条件。

  ArduPilot 重度依赖这一能力。libraries/AP_HAL_ChibiOS/UARTDriver.cpp
  的 RX DMA ISR（:583-613）就是典型：

      if (uart_drv->_wait.thread_ctx &&
          uart_drv->_readbuf.available() >= uart_drv->_wait.n) {
          chSysLockFromISR();
          chEvtSignalI(uart_drv->_wait.thread_ctx, EVT_DATA);
          chSysUnlockFromISR();
      }

  线程侧（UARTDriver.cpp:769）：
      eventmask_t evt = chEvtWaitAnyTimeout(EVT_DATA, ...);

  同样的模式还出现在 RCOutput.cpp（EVT_PWM_SEND / EVT_PWM_SYNTHETIC_SEND /
  dshot_event_mask / serial_event_mask —— 一个文件里用了 4 组不同的位掩码）。

  NuttX 的替代方案是 POSIX 信号（sigwaitinfo / sigtimedwait）+ poll()。
  PX4 的实际做法则是**完全绕开**：用 WorkQueue + uORB 订阅回调，
  把"等组合条件"变成"等某一个 topic 更新"。
  这是 PX4 架构能成立的前提之一 —— 它把大量的异步逻辑
  收敛成了"订阅-回调"一种形态，从而不需要事件组。
  ⇒ 这也解释了为什么 PX4 的驱动代码风格和 ArduPilot 差别这么大：
     PX4 驱动倾向于"发布 uORB + 用 hrt callout 定时"，而 ArduPilot 驱动
     倾向于"在 ISR 里 chEvtSignalI + 专用线程 chEvtWaitAny"。

【行 8：任务通知 —— FreeRTOS 独有的"作弊"手段】
  每个 FreeRTOS 任务自带一个 32 位通知值和"通知状态"，
  可以直接当二值信号量、计数信号量、事件组、或邮箱用，
  而且比对应原语**快 45% 左右、省一个控制块**（官方 benchmark 说法，[未亲验]）。
  代价：一个任务同时只能有一个通知使用者 —— 多方复用会互相踩。

  ChibiOS 和 NuttX 都没有对等物。
  NuttX 的最近似物是"线程私有信号"，但没有"每个任务自带一个计数器"这种东西。

【行 13：VFS —— NuttX 独有的降维打击】
  这一行是三家最大的能力差距，也是第 6 课整节课的内容。
  FreeRTOS / ChibiOS 里"设备"是一个编译期决定的东西；
  NuttX 里它是一个运行期可以 open/read/write/ioctl/poll 的 fd。
  后果：NuttX 上的应用代码可以完全不知道底下是什么硬件 ——
  PX4 因此能做到"同一个 uORB 驱动在 SITL 和真机上跑同一份代码"。
""")

    # 用等待条件数模拟"原语能力"对代码复杂度的影响
    print("  量化：驱动里常见的「等待条件数」 vs 可用的原语数")
    print("  " + "-" * 70)
    print("""
  假设一个 IMU 驱动的 ISR 需要通知线程 3 种情况：
      DATA_READY / FIFO_OVERFLOW / BUS_ERROR

  FreeRTOS:  1 个 EventGroup，3 个 bit，线程 xEventGroupWaitBits() 一次搞定
             → ISR 侧 0 个额外状态，代码 ~6 行
  ChibiOS:   1 个 Event Source，3 个位，线程 chEvtWaitAny() 一次搞定
             → ISR 侧 0 个额外状态，代码 ~6 行
  NuttX:     没有事件组。可选方案：
             (a) 3 个信号量 + 线程轮询/信号 → 代码量翻 3 倍，且丢失原子性
                 （"同时发生了 DATA_READY 和 BUS_ERROR"无法无锁地表达）
             (b) 用 POSIX 信号 sigqueue() 带 payload → 1 个排队信号，
                 sigtimedwait() 取。可行但信号有 per-process 排队上限
                 （RLIMIT_SIGPENDING）
             (c) **换成 PX4 的形态**：ISR 里只做最小动作（拷贝到环形缓冲 +
                 置标志），线程在 Run() 里一次处理全部待办
             ⇒ PX4 选了 (c)。所以 NuttX 的"缺事件组"在飞控场景下
               被架构选择抹掉了，代价是驱动必须写成"轮询 + 状态快照"
               而不是"精确等待某一个事件"。
             ⇒ 这也是一个真实的性能差别：PX4 的驱动在无数据时仍然
               会以固定频率跑 Run()（比如 IMU 是 1 kHz 定频），
               而 ArduPilot 的 UART 线程可以真正睡到有数据才醒。
               在低功耗场景下这个差别是实打实的。
""")


# ============================================================================
# 实验 5：优先级继承的三种态度
# ============================================================================

def exp5_priority_inheritance():
    sec("实验 5：优先级继承的三种态度 —— 和它引发的死锁自救")

    print("""
优先级继承（PI, Priority Inheritance）是实时 OS 的经典话题。
三家在"谁有 PI、能不能关、默认开不开"上的选择完全不同，
而 ArduPilot 为此写了一段相当罕见的代码。

【态度对照】

  FreeRTOS : PI 只属于**互斥量**。xSemaphoreCreateMutex() 创建的句柄
             内部类型是 queueQUEUE_TYPE_MUTEX，带 PI 和优先级还原；
             xSemaphoreCreateBinary() 创建的内部类型是
             queueQUEUE_TYPE_BINARY_SEMAPHORE，**没有 PI**。
             ⇒ 用二值信号量当锁 = 没有 PI = 可能优先级反转。
               这是 FreeRTOS 新手最常踩的坑。                    [未亲验]

  ChibiOS  : `mutex_t` 的 PI 是**内建且不可关闭**的（这是 ChibiOS 的
             设计决定：mutex 就应该是 PI 的，需要非 PI 就用 binary
             semaphore）。所以选择用哪个类型就等于选择了要不要 PI。
             ArduPilot 用 mutex_t 做 HAL_Semaphore：
                 libraries/AP_HAL_ChibiOS/Semaphores.cpp:30-33
                 Semaphore::Semaphore() {
                     static_assert(sizeof(_lock) >= sizeof(mutex_t), ...);
                     mutex_t *mtx = (mutex_t *)_lock;
                     chMtxObjectInit(mtx);
                 }
             ⇒ ArduPilot 的 HAL_Semaphore **永远带 PI**。

  NuttX    : 最特别 —— **mutex 就是 semaphore**。
                 include/nuttx/mutex.h:117   typedef sem_t mutex_t;
                 include/semaphore.h:117     typedef struct sem_s sem_t;
             mutex_t 和 sem_t 是**同一个类型**，没有任何区别；
             "互斥"语义只是"初值设为 1 的计数信号量"而已：
                 include/nuttx/mutex.h:39  #define NXMUTEX_INITIALIZER SEM_INITIALIZER(1)
             PI 则是**编译期总开关**：
                 sched/Kconfig:1139  menuconfig PRIORITY_INHERITANCE
                                     bool "Enable priority inheritance"
                                     default n            ← 默认关！
             打开后，**所有**信号量/互斥量默认都带 PI，要显式关：
                 int ret = sem_setprotocol(&sem, SEM_PRIO_NONE);
             ⇒ 和 ChibiOS 恰好相反的哲学：ChibiOS 是"类型决定 PI"，
               NuttX 是"全局开关决定 PI，类型不区分"。

【PX4 的选择】
  PX4 打开了这个开关，而且把持有人槽位从默认 8 提到了 32：
      boards/ark/fmu-v6x/nuttx-config/nsh/defconfig:189  CONFIG_PRIORITY_INHERITANCE=y
      boards/ark/fmu-v6x/nuttx-config/nsh/defconfig:212  CONFIG_SEM_PREALLOCHOLDERS=32
  （全 155 个 .px4board 里，78 个 defconfig 设了 CONFIG_PRIORITY_INHERITANCE=y）
  没设 CONFIG_SEM_NNESTPRIO ⇒ 用默认值 16（最多 16 个更高优先级线程在等）。
""")

    # 优先级反转模拟：三种情形对比
    print("  现在做一个优先级反转的量化模拟。场景是飞控里真实会发生的事：")
    print()
    print("    高优先级 H (priority 高) : 姿态控制，需要在 1 ms 内拿到传感器数据")
    print("    中优先级 M (priority 中) : 日志写 SD 卡，计算密集，从不主动让出")
    print("    低优先级 L (priority 低) : 传感器驱动，持有一把锁，锁内要等 I2C")
    print()
    print("    L 先拿到锁 → H 就绪，试图拿同一把锁 → 被阻塞")
    print("    → M 就绪。如果 L 的优先级不提升，M 会一直抢占 L ...")
    print()

    # 离散事件模拟。时间轴单位 us。
    # 关键设计：H 必须在 L 的临界区**中途**才就绪，否则看不出 PI 的价值。
    # （我第一版让 H 恰好在 L 出临界区的同一刻就绪，结果两种情形都是
    #   "H 等 0 us"，跑出 "差别 4000 倍" 这种没有意义的数字。教训：
    #   模拟的**时间轴相位**决定了结论，必须慎重选临界点和事件点。）
    CS_TOTAL_US = 2000    # L 的临界区总长（含 I2C 事务 + 后续处理）
    H_ARRIVE_US = 500     # H 在临界区进行到 500 us 时被唤醒
    LOG_TOTAL_US = 4000   # M（日志线程）的总计算量

    def simulate(pi: bool):
        """返回 (H 的阻塞时间 us, 事件序列)。
        假设：L 的临界区在开始后不放弃 CPU（轮询式 I2C 驱动）；
              M 的优先级高于 L，一旦就绪就会抢占 L（除非 L 被提升）。"""
        log = []
        cs_done = CS_TOTAL_US          # L 若不受干扰，此时出临界区
        t = H_ARRIVE_US
        log.append((0, "L 拿锁，进入临界区（共 %d us）" % CS_TOTAL_US))
        log.append((t, "H 就绪，请求同一把锁 → 阻塞（H 的优先级最高）"))
        log.append((t, "M 同时就绪（优先级：H > M > L）"))

        if pi:
            # L 继承 H 的优先级 → M 抢不动 L → L 按原计划出临界区
            log.append((t, "L 被提升到 H 的优先级 → M 无法抢占"))
            log.append((cs_done,
                        "L 出临界区并释放锁 → H 立即获得"))
            h_wait = cs_done - H_ARRIVE_US
        else:
            # M 抢占 L。L 被挂起，它的临界区停在中途，锁不释放。
            log.append((t, "M 抢占 L（L 的优先级未被提升）"))
            t += LOG_TOTAL_US
            log.append((t, "M 跑完 %d us，L 才被调度回来" % LOG_TOTAL_US))
            t += (CS_TOTAL_US - H_ARRIVE_US)     # L 跑完临界区剩余部分
            log.append((t, "L 出临界区并释放锁 → H 获得"))
            h_wait = t - H_ARRIVE_US
        return h_wait, log

    for pi in (False, True):
        wait, events = simulate(pi)
        tag = "有 PI" if pi else "无 PI"
        print("  ┌─ %s " % tag + "─" * 62)
        for (tt, ev) in events:
            print("  │ t=%6d us  %s" % (tt, ev))
        print("  └─ H 的总阻塞时间 = %d us" % wait)
        print()

    w_nopi, _ = simulate(False)
    w_pi, _ = simulate(True)
    print("  结论：PI 把 H 的阻塞从 %d us 降到 %d us —— 降到原来的 %.0f%%。"
          % (w_nopi, w_pi, 100.0 * w_pi / w_nopi))
    print("""
  这个数字值得记住：**PI 不能让 H 不等，只能让它只等"剩余的临界区"。**
      - 有 PI：H 等的是 (临界区剩余长度) = 1500 us
      - 无 PI：H 等的是 (临界区剩余长度 + M 的全部运行时间) = 5500 us
  差值就是 M 的执行时间 —— **和临界区长度无关**。
  ⇒ 关键推论：优先级反转的危害不取决于临界区有多长，
    而取决于"中间优先级任务"能跑多久。一个跑了 20 ms 的日志线程
    能把姿态控制的延迟从 1 ms 拉到 20 ms。
    这就是为什么 ArduPilot 要把日志线程放在**低优先级**（APM_STORAGE_PRIORITY=59，
    远低于 APM_MAIN_PRIORITY=180），而不是让它"跑快一点"。
""")

    print("""
  但 PI 不是免费的。看 NuttX 的实现（sched/semaphore/sem_holder.c）：

    - 每个信号量要记住"谁持有它、持有了几个 count"（semholder_s 结构）
    - 每个任务要有一条反向链（semholder_s.tlink：本任务持有的所有信号量的链）
    - 被提升优先级的任务释放锁时，要**重新计算**自己应该回到哪个优先级 ——
      因为它可能同时持有多个锁、被多个高优先级任务提升过

  这个重算过程在 NuttX 里有个非常讲究的处理（sem_holder.c:727-741 注释原文）：

      /* The currently executed thread should be the lower priority
       * thread that just posted the count and caused this action.
       * However, we cannot drop the priority of the currently running
       * thread -- because that will cause it to be suspended.
       *
       * So, do this in two passes.  First, reprioritizing all holders
       * except for the running thread.
       */
      nxsem_foreachholder(sem, nxsem_restoreholderprio_others, stcb);
      /* Now, find an reprioritize only the ready to run task */
      nxsem_foreachholder(sem, nxsem_restoreholderprio_self, stcb);

  为什么必须两趟？因为"降低当前运行线程的优先级"这个动作本身会导致
  立刻被抢占 —— 于是那个线程会在**自己的优先级还原代码还没跑完**的时候
  就被切走。两趟的做法把"降自己的优先级"推迟到最后一刻。

  这类细节是判断一个 RTOS 是否真的在生产里磨过的标志。
  PX4 把 CONFIG_SEM_PREALLOCHOLDERS 从默认 8 提到 32，说明它在真实
  飞行中确实遇到了"持有人槽位不够"的情况 —— 槽位用尽后
  nxsem_add_holder 会静默失败，PI 就悄悄失效了。
  这是一个 PX4 用户几乎不可能察觉、但在高负载下会表现为
  "偶发姿态控制抖动"的坑。

【ArduPilot 的极端用法】
  ChibiOS 的 mutex_t 用 PI，但**没有带超时的阻塞获取** ——
  因为放弃一个 PI 互斥量会破坏继承链的不变式。
  ArduPilot 需要 take(timeout)，于是它用轮询实现
  （libraries/AP_HAL_ChibiOS/Semaphores.cpp:57-66）：

      uint64_t start = AP_HAL::micros64();
      do {
          hal.scheduler->delay_microseconds(200);
          if (take_nonblocking()) { return true; }
      } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
      return false;

  ⇒ 两个后果：
     (1) 超时精度只有 200 µs 量级，不是精确的
     (2) 等待期间**不睡眠**（只是 delay 200us 后 trylock），
         浪费 CPU；而 NuttX 的 nxsem_timedwait 是真正的阻塞等待，
         内核把线程挂到 g_waitingforsemaphore 上，零 CPU 消耗
     这是一个可以直接对比的、真实的实时性/效率差异。

  更进一步：ArduPilot 还要处理"主线程持锁时卡死"这个 ChibiOS 不提供
  任何保护的情况，于是有了 Scheduler.cpp:856-888 的 try_force_mutex()：

      thread_t *main_thread = get_main_thread();
      if (main_thread->state != CH_STATE_WTMTX) { chSysUnlock(); return; }
      mutex_t *wtmtx = main_thread->u.wtmtxp;
      ...
      chMtxForceReleaseS(wtmtx);        // 强制剥夺锁！
      AP::logger().WriteCritical("DLCK", ...);
      GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "CRITICAL Deadlock %u %s %p", ...);

  这段代码直接读取了 ChibiOS 内部的 thread_t.state / u.wtmtxp / mutex_t.owner，
  然后调用内核的 ForceRelease。它是"必须深度耦合 RTOS 内部结构"才能
  实现的死锁自救。对照 NuttX：内核自己就有 sem_recover.c
  （任务退出时自动释放它持有的所有信号量），应用层不需要也不应该
  去碰内核内部结构。
  ⇒ 这是一个明确的架构质量差异点：**NuttX 在这一项上更健壮**。
     ArduPilot 之所以要自己写，一部分原因正是 ChibiOS 不带超时 mutex
     逼出来的（无法用"超时失败"来优雅处理，只能硬剥夺）。
""")


# ============================================================================
# 实验 6：内存管理对比 + 亲手跑一遍 heap_4 的合并算法
# ============================================================================

def exp6_memory():
    sec("实验 6：内存管理 —— heap_1~5 / 单堆 / 统一堆 + 区域合并")

    print("""
内存管理是三家差距最大的一块。先把能力表列清楚：

  FreeRTOS heap_1 ~ heap_5（互斥，一个工程只能选一个）：
""")
    heaps = [
        ("heap_1", "只能分配，**不能释放**", "无", "不适用", "★☆☆", "固定任务集的极简系统"),
        ("heap_2", "可分配可释放", "不合并", "否", "★★☆", "分配块大小固定、从不长期释放"),
        ("heap_3", "包装标准 malloc/free", "由 libc 管", "否", "★★★", "已有成熟 libc 堆"),
        ("heap_4", "可分配可释放", "**会合并**", "否", "★★★★", "通用首选"),
        ("heap_5", "可分配可释放", "**会合并**", "**是（多区域）**", "★★★★★", "RAM 分散在多个 bank"),
    ]
    print("  %-8s %-24s %-10s %-16s %-8s %s"
          % ("方案", "能否 free", "相邻块合并", "多非连续区域", "复杂度", "典型场景"))
    print("  " + "-" * 100)
    for r in heaps:
        print("  %-8s %-24s %-10s %-16s %-8s %s" % r)
    print("""
  heap_1/2/4 共用同一份 ~200 行的实现，只在几个宏上分叉：
      #if ( configUSE_MALLOC_FAILED_HOOK ... )
      #ifdef prvHeapInit / vPortDefineHeapRegions
  堆的总空间是 `configTOTAL_HEAP_SIZE`，在启动时被声明成一个
  `static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ]` ——
  也就是说 **FreeRTOS 的堆大小是编译期常量，且占用静态 RAM**。
  这是它和 NuttX 最本质的区别。                                [未亲验]

  ChibiOS：
      chHeapObjectInit(&heap, addr, size)  —— 运行期在**任意内存区域**建堆
      chHeapAlloc / chHeapFree / chHeapStatus        —— first-fit + 合并
      C 库的 malloc/free 是**可选的**（CH_CFG_USE_HEAP 只提供内核堆 API，
      libc 的 malloc 由 newlib 或自定义提供）
  ⇒ ChibiOS 的堆管理比 FreeRTOS 灵活（可多堆、可在任意区域），
    但只提供一层，没有"不同属性内存"的概念。

      ArduPilot 因此自己写了一层（hwdef/common/malloc.c）：
      - 按区域分别建堆（第 0 个用 ChibiOS 默认堆，其余 chHeapObjectInit）
      - 用 `--wrap` 接管 libc 的 malloc/free（malloc.c:575-580）：
            void *__wrap__malloc_r(void *rptr, size_t size) {
                (void)rptr;
                // we want consistent zero memory
                return calloc(1, size);       // ★ 强制清零
            }
        （注意这个行为改变：标准 malloc 不清零，ArduPilot 的会清零。）
      - 引入"内存属性"概念并按属性路由到不同区域：
            MEM_REGION_FLAG_DMA_OK   → malloc_dma()   DMA 可访问
            MEM_REGION_FLAG_FAST     → malloc_fastmem()
            MEM_REGION_FLAG_AXI_BUS  → malloc_axi_sram()
            MEM_REGION_FLAG_ETH_SAFE → malloc_eth_safe()
      - 最后有一块 DMA 保留池（DMA_RESERVE_SIZE，默认 6144 B）兜底
  ⇒ 这套东西**本质上是在 ChibiOS 之上重新发明了 NuttX 的
    CONFIG_MM_REGIONS + 属性隔离**。区别是 PX4 从内核直接拿到这个能力，
    ArduPilot 要在 700 行 C 里自己搭一遍。

  NuttX：
      - 编译期声明多块内存区域，内核在启动时把它们**合并成统一堆**：
            boards/ark/fmu-v6x/nuttx-config/nsh/defconfig 中
            CONFIG_MM_REGIONS=4
        （第 8 课已核实：四块不连续 RAM ≈ 992 KB 合并为一个堆 ——
          AXI SRAM 512K + SRAM123 288K + SRAM4 64K + DTCM 128K）
      - 提供两套分配器：
            mm_alloc.c  —— 通用堆（小块 + 大块混合，带 per-region 锁）
            umm_malloc.c —— 小块子分配器（可选，CONFIG_MM_SMALL）
      - DMA 安全性由**独立的 GRAN 池**解决，不从主堆出（PX4 用 5 KB）
      - 有标准的 POSIX malloc/free/calloc/realloc/memalign
      - 有 mmap/munmap 风格的 `mmap()` 用于共享内存
  ⇒ 能力上 NuttX 最完整：多区域 + 属性隔离 + 标准 API + 运行期查询
    （`free` 命令、/proc/meminfo）。代价是 mm/ 目录 13,423 行代码
    （对比 FreeRTOS 一个 heap_4.c 约 400 行）。
""")

    # 亲手实现 heap_4 的合并，验证"合并"到底值多少钱
    print("  亲手验证：「合并」到底值多少钱？")
    print("  " + "-" * 70)
    print("""
  heap_2 和 heap_4 的唯一实质差别是"相邻空闲块是否合并"。
  这听起来是个小优化，但看数字就明白它为什么是强制推荐项。

  要观察到差别，**必须构造出"相邻的两块同时空闲"的局面** ——
  我第一版模拟只释放了间隔开的块，那些块彼此不相邻，合并算法
  根本没有机会执行，两种方案跑出完全一样的结果（都是最大空洞 64）。
  这个失败本身就是个教训：**验证一个优化有没有用，必须构造出它
  针对的那个场景**，否则"测不出差别"会被误读成"差别不存在"。

  正确的场景（下面实际跑的）：
    1. 依次分配 A(10) B(5) C(10) D(5)，地址连续
    2. 释放 A 和 C          → 两个各 10 的孤立空洞
    3. 释放 B               → B 与 A、C 都相邻
                              heap_4 把 A+B+C 合并成 25 的连续块
                              heap_2 还是 10 + 5 + 10 三个洞
    4. 申请 20 单位         → heap_4 成功，heap_2 **失败**
                              （两者此刻的空闲总量都是 25）
""")

    def frag_test(coalesce: bool):
        """堆总 30 单位，按上面的场景走一遍，返回 (最大连续块, 空闲总量, 洞数, 能否分配20)。

        简化模型：线性地址空间 + 块表 [(offset, size, free)]。
        只模拟"合并与否"这一个变量。丢掉的东西：
          (a) heap_4 每个块的 xBlockLink 头开销（真实是 8 字节）
          (b) 分配对齐（heap_4 会对齐到 portBYTE_ALIGNMENT）
          (c) heap_4 的 xBlockAllocatedBit 标记位
          (d) heap_2 与 heap_4 在"最佳适配 vs 首次适配"上的差别
        这些都不影响"合并能否避免外碎片"这个结论。
        """
        blocks = [(0, 30, True)]

        def find_free(n):
            for i, (off, sz, fr) in enumerate(blocks):
                if fr and sz >= n:
                    return i
            return None

        def alloc(n):
            i = find_free(n)
            if i is None:
                return None
            off, sz, _ = blocks[i]
            blocks[i:i + 1] = [(off, n, False), (off + n, sz - n, True)]
            # 去掉零长尾块
            blocks[:] = [b for b in blocks if b[1] > 0]
            return off

        def release(off):
            for i, (o, sz, fr) in enumerate(blocks):
                if o == off and not fr:
                    blocks[i] = (o, sz, True)
                    if coalesce:
                        # 真实 heap_4 的做法（heap_4.c:492-545 prvInsertBlockIntoFreeList）：
                        # 空闲链表按**地址**升序维护，插入时在同一次调用里
                        # **显式做两个方向**的合并：
                        #   (a) 与插入点【前】一块：检查 pxIterator + size == 新块
                        #   (b) 与插入点【后】一块：检查 新块 + size == pxIterator->next
                        #
                        # 【修正】本课第一版在这里写的是"与上一块合并是通过上一块
                        # 插入时向前合并完成的"——**机制说错了**。实际两个方向都在
                        # 这一次插入里完成，见 heap_4.c:509-545 的两段
                        # "Do the block being inserted, and the block it is being
                        #  inserted after/before make a contiguous block of memory?"
                        # 结论（所有相邻空闲块最终并成一块）碰巧是对的，但机制不对。
                        # 这是补拉 FreeRTOS 源码复核后修正的 6 处之一。
                        changed = True
                        while changed:
                            changed = False
                            for j in range(len(blocks) - 1):
                                o1, s1, f1 = blocks[j]
                                o2, s2, f2 = blocks[j + 1]
                                if f1 and f2:
                                    blocks[j:j + 2] = [(o1, s1 + s2, True)]
                                    changed = True
                                    break
                    return True
            return False

        a = alloc(10)
        b = alloc(5)
        c = alloc(10)
        d = alloc(5)
        release(a)
        release(c)
        release(b)

        mx = max((sz for (o, sz, fr) in blocks if fr), default=0)
        tot = sum(sz for (o, sz, fr) in blocks if fr)
        holes = sum(1 for (o, sz, fr) in blocks if fr)
        can20 = find_free(20) is not None
        return mx, tot, holes, can20

    print("  %-24s %-12s %-12s %-14s %s"
          % ("", "空闲总量", "空洞个数", "最大连续块", "能否再分配 20 单位"))
    print("  " + "-" * 86)
    results = {}
    for coalesce in (False, True):
        mx, tot, holes, can20 = frag_test(coalesce)
        results[coalesce] = (mx, tot, holes, can20)
        name = "heap_4（会合并）" if coalesce else "heap_2（不合并）"
        print("  %-24s %-12d %-12d %-14d %s"
              % (name, tot, holes, mx, "能" if can20 else "**不能**"))

    mx2, tot2, h2, ok2 = results[False]
    mx4, tot4, h4, ok4 = results[True]
    print()
    print("  空闲总量：heap_2 = %d，heap_4 = %d  → **完全相同**" % (tot2, tot4))
    print("  最大连续块：heap_2 = %d，heap_4 = %d" % (mx2, mx4))
    print("  分配 20 单位：heap_2 = %s，heap_4 = %s"
          % ("失败" if not ok2 else "成功", "失败" if not ok4 else "成功"))

    print("""
  这就是 external fragmentation（外碎片）的代价：
  **空闲内存总量一模一样，但 heap_2 分配失败，heap_4 成功。**
  在飞控上的表现是："free 命令显示还有 80 KB，malloc 却返回 NULL"，
  然后某个驱动初始化失败、某个模块起不来 —— 而且**只在特定运行
  时序下发生**，重启一次可能就好了。这是最难定位的一类内存 bug。

  ⇒ 工程结论：
    1) **永远不要用 heap_2 做通用分配器** —— 但理由要说准。

       【修正】本课第一版说 heap_2 是"heap_4 去掉合并"，理由是
       "块大小固定时省掉合并开销"。**补拉 FreeRTOS 源码复核后
       发现这个描述不准确。** 实际的差异是**空闲链表的排序准则**：

            heap_2.c:135  prvInsertBlockIntoFreeList（宏）
                for( pxIterator = &xStart;
                     pxIterator->pxNextFreeBlock->xBlockSize < xBlockSize;  ← 按【大小】
                     pxIterator = pxIterator->pxNextFreeBlock )

            heap_4.c:492  prvInsertBlockIntoFreeList（函数）
                for( pxIterator = &xStart;
                     heapPROTECT_BLOCK_POINTER(pxIterator->pxNextFreeBlock) < pxBlockToInsert;  ← 按【地址】
                     pxIterator = heapPROTECT_BLOCK_POINTER(pxIterator->pxNextFreeBlock) )

       ⇒ **heap_2 是一个 best-fit 分配器**（按大小排序，第一个够大的
         就是最贴合的）。best-fit 在"分配块大小种类很少"时表现最好 ——
         **这才是 heap_2 存在的理由**，不是"heap_4 的简化版"。
       ⇒ 而 best-fit + 不合并的组合，在块大小多样的场景下会迅速产生
         大量微小碎片。飞控里没有任何一个内存池是真正大小固定的
         （日志缓冲、MAVLink 包、uORB 队列、SD 卡扇区缓存都在变），
         所以 heap_2 不适合。
       FreeRTOS 官方从 10.x 起也把 heap_2 标记为 deprecated，[未亲验]
       （这是文档层面的说法，源码里没有 deprecation 标记）

    2) heap_1 是另一种情况：它不是"差"，而是"够用"。

       【修正】第一版说它的 "free 是空操作"。实际不是 ——
       heap_1.c:130-139 的 vPortFree 会**强制断言**：

           void vPortFree( void * pv )
           {
               /* Memory cannot be freed using this scheme. ... */
               ( void ) pv;
               /* Force an assert as it is invalid to call this function. */
               configASSERT( pv == NULL );
           }

       ⇒ 传非 NULL 指针进去会触发 configASSERT 停机。
         **这个差别很重要**："空操作"意味着误用会静默泄漏内存；
         "断言"意味着误用会立刻暴露。**后者是更好的设计。**
         我把它说成前者，是低估了 FreeRTOS。

       所以 heap_1 的定位是：如果系统在初始化阶段分配完所有内存、
       之后再不 free，它的零开销（没有空闲链表、没有合并）
       是**最优解**，不是妥协。而且"误用 free 会被断言抓住"
       这一点让它比"空操作"版本更安全。
       PX4 的等价物是"启动时一次性分配 + 运行期不再 malloc"
       （第 9 课提到的那些工作队列栈、uORB 缓冲区都是启动时分配的），
       哲学上和 heap_1 一致 —— 但 NuttX 没有提供"编译期禁用 free"
       这种模式，所以你无法从内核层面保证没人偷偷 malloc。
       这是一个**真实的能力差异**，值得在选型时考虑。
    3) 无论用哪个，都必须在开发期把堆水位监控起来。
       PX4 的做法是 `free` 命令 + `work_queue status`；
       ArduPilot 的做法是多堆状态查询（malloc.c 的 mem_available()，
       用 chHeapStatus() + chCoreGetStatusX() 汇总）。
       两家都把"内存用量可观测"当成必需的调试能力，不是可选项。
""")


# ============================================================================
# 实验 7：驱动模型 —— 加一个新设备要改多少代码
# ============================================================================

def exp7_driver_model():
    sec("实验 7：驱动模型对比 —— 加一个新 I2C 传感器要改多少处代码")

    print("""
这是三家最不可能收敛的一块，也是"选 NuttX 还是选裸 RTOS"这个问题的
真正答案所在。

用同一个具体任务量化：**给飞控加一个新的 I2C 气压计**。
""")

    print("  " + "-" * 78)
    print("  [FreeRTOS] 没有驱动模型")
    print("  " + "-" * 78)
    print("""
  FreeRTOS 只提供调度 + 同步 + 队列。设备是纯粹的"你的代码 + 你的寄存器"。
  加一个气压计要做的：
    1. 自己写 I2C 读写函数（或找一个第三方 I2C 库）
    2. 自己定义数据结构存放校准系数
    3. 自己决定用轮询/中断/DMA，自己写 ISR
    4. 自己写线程（如果用），自己算优先级和栈大小
    5. 想把它暴露给"上层应用"？自己发明一套接口并录入文档

  代码量：一个能用的气压计驱动约 300~800 行（不含 I2C 底层）。
  受益：零框架约束，代码可以最紧凑。
  代价：每个驱动各写各的，"增加设备"是**线性增加**的成本，
        且没有任何东西保证两个驱动的错误处理风格一致。
  ⇒ 这是为什么没有哪个成熟的飞控固件直接用裸 FreeRTOS 做整机架构
    （有例外：一些极简的固定翼/竞速飞控确实这么干）。
""")

    print("  " + "-" * 78)
    print("  [ChibiOS] 有 HAL 层，但设备集是**编译期**决定的")
    print("  " + "-" * 78)
    print("""
  ChibiOS 的 HAL 是"编译期开关 + 静态对象"模型，配置在三个头文件里：
      halconf.h   HAL_USE_ADC / HAL_USE_SPI / HAL_USE_I2C / ...  ← 驱动开不开
      mcuconf.h   STM32_SPI_USE_SPI1 / STM32_SERIAL_USE_USART1 / ...  ← 用哪个外设
      board.h     GPIO 引脚 / 板级常量
  这些文件都是**签入仓库的**（ArduPilot 放在
  libraries/AP_HAL_ChibiOS/hwdef/common/ 下），不是构建系统生成的。

  唯一的"生成"环节是 ArduPilot 自研的 Python DSL：
      hwdef.dat  →  chibios_hwdef.py  →  build/hwdef.h
  板级配置写成 hwdef.dat 里的一行行 `define`，脚本生成 hwdef.h，
  再由 chconf.h / halconf.h / mcuconf.h 用 #ifndef 的默认值 + hwdef.h
  的覆盖值拼出最终配置。

  加一个气压计要做的：
    1. 在 hwdef.dat 里声明它挂在哪条 I2C 上、地址是多少
       （ArduPilot 的做法：`BARO BMP388 I2C:0:0x76`）
    2. 写 AP_Baro 的后端类（继承 AP_Baro_Backend）
    3. 在 AP_Baro::detect() 里注册探测函数
    4. 在 hwdef.dat 里加 `define HAL_BARO_BMP388_I2C_ADDR 0x76`

  代码量：后端约 200~400 行（AP_Baro 已经有基类和探测框架）。
  受益：新驱动有明确的落脚点（基类 + 统一错误处理 + 统一校准持久化），
        "增加设备"是**亚线性**成本。
  代价：**设备集在编译期固定**。想在运行期插一个新传感器？
        除非固件里已经编了它的驱动，否则不可能。
        这一条直接决定了：
        - ArduPilot 必须为每种板子编一份固件（约 100 种板 × 多个变体）
        - 无法做"运行期加载第三方驱动"
        - 无法把同一份固件跑在硬件不同的板子上

  ArduPilot 在**绕过 ChibiOS 自带驱动**这件事上走得很远（这非常值得注意）：
      halconf.h:68   HAL_USE_CAN  FALSE   ← 不用 ChibiOS 的 CAN
                     → 自研 libraries/AP_HAL_ChibiOS/bxcan.hpp (19 KB)
                       + CanIface.cpp (33 KB) + CANFDIface.cpp (41 KB)
      halconf.h:209  HAL_USE_WDG  FALSE   ← 不用 ChibiOS 的看门狗
                     → 自研 hwdef/common/watchdog.c
                       原因：要把崩溃现场写进备份寄存器，IWDG 复位后读回。
                       ChibiOS 的 WDG 驱动没这个能力。
      system.cpp:354 时间快路径**跳过 OS 层**，直读 st_lld_get_counter()
      system.cpp:157 异常处理自研，注释直说
                     "ChibiOS does not do this by default"
                     （BusFault 需要置 SCB->SHCSR 的 BUSFAULTENA，ChibiOS 不置）
      malloc.c       内存分配完全自研（见实验 6）
  ⇒ 结论：**ArduPilot 用的是"ChibiOS 内核 + 自家 HAL"，不是"ChibiOS 生态"。**
    它借用了调度器、同步原语、时间基准、PAL/SPI/I2C/ADC 驱动，
    其余全部自己写。这是理解 ArduPilot 的关键，也是它和 PX4 路线的
    根本分野：PX4 是"尽量用 NuttX 提供的东西"，ArduPilot 是"只用
    ChibiOS 的内核部分"。
""")

    print("  " + "-" * 78)
    print("  [NuttX] 有完整的 VFS + 运行期注册")
    print("  " + "-" * 78)
    print("""
  第 6 课整节课讲的就是这条链路：
      register_driver("/dev/baro0", &g_baro_fops, 0666, priv)
        → inode_reserve()  建立 /dev/baro0 的 inode（自动创建中间目录）
        → inode->u.i_ops = &g_baro_fops
        → 之后 open("/dev/baro0") 就能拿到 fd
      open() → file_allocate() → fs_getfilep() → file->f_ops = inode->u.i_ops
      read()/write()/ioctl() → file->f_ops->read/write/ioctl(priv, ...)

  加一个气压计要做的：
    1. 写一个 struct file_operations（read/write/ioctl）
    2. 调 register_driver()
    3. 上层用 open/ioctl 访问 —— **不需要重新编译上层**
    4. 想加第二个同型号气压计？再 register_driver("/dev/baro1", ...) 即可
       （这就是 PX4 上 baro0 / baro1 的来源）

  代码量：驱动本体 200~500 行 + 一个 fops 表。
  受益：
    - **运行期**注册/注销（模块可以 insmod/rmmod 风格地加载）
    - 上层完全硬件无关：同一份应用代码读 /dev/baro0 就能工作
    - 天然的权限、命名空间、fd 生命周期管理（虽然是弱权限，见第 6 课）
    - 可以并挂多实例
    - 可以在用户态用 shell 直接调试：`cat /dev/baro0`、`ioctl` 工具
  代价：
    - VFS 是间接层，每次 read/ioctl 多一次函数指针跳转（可忽略）
    - 一个 inode 树要占内存（PX4 上通常几 KB）
    - 驱动作者必须理解 VFS 的三层结构（file / inode / fops），
      第 6 课列了 12 个常见错误就是这个门槛的体现
    - **NuttX 的 VFS 不检查权限位**（第 6 课已核实：
      inode_checkflags() 只看 f_ops 里的回调是否为 NULL，
      从不读 i_mode）—— 也就是说 `register_driver(..., 0666, ...)` 里的
      0666 只影响 `ls -l` 的显示，不提供任何保护。
      想真正禁止某个操作，必须传 NULL 回调。
""")

    # 量化对比表
    print("  " + "-" * 78)
    print("  三种驱动模型的能力矩阵")
    print("  " + "-" * 78)
    caps = [
        ("有统一的设备抽象",       "✗", "△ (编译期 HAL)", "✓ VFS"),
        ("设备可在运行期发现",     "✗", "✗",              "✓"),
        ("设备可在运行期注册",     "✗", "✗",              "✓ register_driver"),
        ("同一驱动多实例",         "自己写", "△ (总线+地址不同即为不同实例)", "✓ /dev/x0,/dev/x1"),
        ("上层代码与硬件解耦",     "✗", "△ (通过 HAL 接口)", "✓ (通过 fd)"),
        ("用户态可直接调试设备",   "✗", "✗",              "✓ cat/echo/ioctl"),
        ("同型号加第 N 个实例成本", "线性", "线性",          "常数（多一行 register）"),
        ("权限保护",               "无", "无",              "名义上有，实际无效"),
        ("运行时开销",             "0",  "0",               "每次 read/ioctl 一次间接跳转"),
        ("代码体积开销",           "0",  "HAL 驱动按需编译", "VFS ≈ 数 KB"),
    ]
    print("  %-26s %-12s %-30s %s" % ("能力", "FreeRTOS", "ChibiOS", "NuttX"))
    print("  " + "-" * 92)
    for r in caps:
        print("  %-26s %-12s %-30s %s" % r)

    print("""
  ⇒ 这张表解释了 PX4 和 ArduPilot 的架构分野：
    PX4 敢于把"传感器驱动 → uORB 发布"写成通用模式，
    是因为 VFS 让驱动和上层可以完全解耦、可以多实例、可以运行期加载；
    ArduPilot 必须用编译期的 hwdef.dat + 后端注册表，
    也是因为 ChibiOS 没有 VFS —— 它只能用编译期的方式解决同一个问题，
    并为此接受"每块板子一份固件"的成本。

  这不是"谁更好"的问题：
    - PX4 换来的是灵活性（SITL 与真机同代码、动态加载、多实例），
      付出的是 VFS 间接层 + 驱动作者学习成本 + 每设备一个 inode
    - ArduPilot 换来的是零间接层、编译期可完全优化、代码路径最短，
      付出的是固件数量爆炸（100+ 板 × 多变体）和无法运行期扩展
  两边在各自的约束下都是合理选择。
""")


# ============================================================================
# 实验 8：量化对照总表
# ============================================================================

def exp8_summary():
    sec("实验 8：量化对照总表")

    print("""
下面所有数字都是我在本机仓库里实测/实读的。标注 [未亲验] 的是来自
公开文档或记忆、我没能在本机验证的。
""")

    print("  " + "=" * 100)
    print("  【代码规模】—— 单位：行")
    print("  " + "=" * 100)
    print("""
  NuttX（实测：D:/DEVLib/PX4APM/PX4/PX4-Autopilot/platforms/nuttx/NuttX/nuttx）
      sched/            52,540    调度器 + 同步 + IPC + 信号 + 工作队列
         ├ semaphore     3,750    含优先级继承（sem_holder.c 是其中大头）
         ├ irq           2,360
         ├ pthread       4,597
         ├ wqueue        1,636    工作队列（PX4 大量使用）
         └ wdog            926    看门狗定时器（= 软件定时器）
      mm/               13,423    内存管理（多区域合并 + 两套分配器）
      fs/               91,719    VFS + 20 类文件系统
      binfmt/            8,488    ELF / NXFLAT 加载器
      libs/            148,508    libc / libm / libxx（C++ 支持）
      net/             106,825    TCP/IP + 套接字
      drivers/         440,322
      arch/          2,649,615    （绝大部分是芯片头文件和寄存器定义）
      ─────────────────────────
      合计           4,308,955

      其中 **Cortex-M 移植（armv7-m + common）= 13,531 行**
      这是一个更公平的对比基准：NuttX 在 Cortex-M7 上真正跑起来所需的
      架构相关代码，和 FreeRTOS 的 port.c + ChibiOS 的 port 层是同一量级。

      架构覆盖：19 个架构家族 / 116 个变体 / 267 块板
      许可证：Apache 2.0
      有 sim 架构（在 Linux 上把 NuttX 当普通进程跑 —— PX4 SITL 的基础）
""")

    print("""  FreeRTOS
      内核（tasks.c / queue.c / list.c / timers.c / event_groups.c /
            stream_buffer.c + heap_x.c）：约 **1 万行**量级     [未亲验]
      port 层（port.c / portmacro.h，每个架构一份）：约 500~1500 行
      许可证：MIT
      定位：**只有内核**。没有 VFS、没有网络栈（需外挂 lwIP）、
            没有 libc、没有文件系统、没有 shell。
            FreeRTOS 的官方配套是 FreeRTOS+TCP / FreeRTOS+FAT / coreMQTT 等
            **独立可选组件**，不随内核发行。

  ChibiOS
      内核（os/rt）：约 **1.5 万行**量级                        [未亲验]
      HAL（os/hal）：STM32 全系列驱动，规模与内核同量级或更大     [未亲验]
      许可证：**GPL3（带链接例外）+ 商业授权双轨**
            ← 这一条非常重要，见下面的"许可证"小节
      定位：内核 + HAL + 一堆可选子系统（USB、FatFS 封装、lwIP 集成、
            文件系统、shell）。比 FreeRTOS 完整得多，比 NuttX 少一个 VFS。
""")

    print("  " + "=" * 100)
    print("  【许可证 —— 这是选型中最容易被忽略、后果最严重的一项】")
    print("  " + "=" * 100)
    print("""
      NuttX    : Apache 2.0
                 → 可闭源商用、可静态链接、有明确的专利授权条款
                 → 唯一"完全无摩擦"的选择

      FreeRTOS : MIT
                 → 可闭源商用、可静态链接
                 → MIT 只覆盖内核；FreeRTOS+TCP / +FAT / coreMQTT
                   是独立组件，选板子时仍要逐个确认。

      ChibiOS  : **按模块分**，不是"整体 GPL3"          ← 最容易搞错的一点
                 ┌──────────────────────────────────────┬──────────────┐
                 │ RT / NIL 内核、移植层、OSLIB、SB、EX  │ GPL3 + 商业  │
                 │ HAL（SPI/I2C/UART/ADC/PAL…）          │ Apache 2.0   │
                 └──────────────────────────────────────┴──────────────┘
                 → ArduPilot 大量使用的 SPI/I2C/UART 驱动属于 HAL，
                   **是 Apache 2.0**。真正构成 GPL3 负担的只有内核。

      链接例外（linking exception）的**实际条件**（官方文档）：
         1. 非 GPL 代码只能通过 "Approved Interfaces" 链接
            （Doxygen 里标了 @api/@iclass/@sclass/@init 的函数和宏，
             以及 include 未修改的发行版头文件）
         2. 每份组合作品都要附带声明（ChibiOS 版本 + 提供源码的承诺）
         3. 组合作品本身不能是 RTOS / 调度器 / 内核
         4. 组合作品不能是供其他软件链接的二进制或库
         5. **例外只适用于 stable 发行版**；beta 版只有纯 GPL3

      商业授权有**两档**，其中一档免费：
                 Commercial Free  : 免费，闭源可以，源码只读（不能改），
                                    **必须声明使用 ChibiOS**，500 核上限
                 Full Commercial  : 付费，无声明义务，可修改，
                                    1000 / 5000 / 无限核

      ArduPilot 为什么不受影响：
         它 fork 了 ChibiOS（github.com/ArduPilot/ChibiOS）并做了修改。
         这**不是因为链接例外**，而是因为 **ArduPilot 本身是 GPL3** ——
         它按完整 GPL3 条款行事，用不上例外。

      但这里有一个真实的陷阱（本课实测发现）：
         ArduPilot 有三处代码**直接访问 ChibiOS 内核内部数据结构**：
           Semaphores.cpp:76-80    mtx->owner             ← 读 mutex_t 内部字段
           HAL_ChibiOS_Class.cpp:190  daemon_task->hdr.pqueue.prio
                                      daemon_task->realprio  ← 写线程结构体内部
           Scheduler.cpp:856-888   thread_t.state / u.wtmtxp / mutex_t.owner
         这些都不是 @api/@iclass/@sclass 标注的接口。
         ⇒ 商业闭源公司若照抄这些做法，正好踩在链接例外的边界上。
           它是否落在"include 未修改的发行版头文件"这个 Approved
           Interface 范围内，是个需要律师回答的问题。
           ArduPilot 不需要回答，因为它本来就是 GPL3。

      工程含义：
        商业闭源产品 → NuttX (Apache 2.0) 和 FreeRTOS (MIT) 最省事；
                      要用 ChibiOS 就走 Commercial Free（≤500 核）或
                      Full Commercial，**不要**依赖链接例外去改源码
                      或直接访问内核内部结构。
        小批量/原型   → ChibiOS Commercial Free 零成本即可闭源。
        GPL3 开源项目 → 三家都无摩擦。
""")

    print("  " + "=" * 100)
    print("  【谁在用 —— 飞控领域之外的参照系】")
    print("  " + "=" * 100)
    print("""
      FreeRTOS:
        - ESP32 (ESP-IDF 的默认内核；Amazon 收购后仍是)
        - AWS IoT / Amazon FreeRTOS
        - 大量 STM32 工业设备、消费电子
        - ArduPilot 的 ESP32 HAL（AP_HAL_ESP32 存在，但非主流）
        - 相当多的开源飞控（Betaflight 用的是自家调度器，不是 FreeRTOS；
          但 iNav/Betaflight 的某些外围、以及大量"自研飞控"用 FreeRTOS）
        ⇒ 生态最大，但"最大"来自非飞控领域

      ChibiOS:
        - **ArduPilot**（2016 年起从 NuttX 迁移过来，现在是唯一主流 MCU HAL）
        - 一些工业控制、机器人
        - 相对小众，但质量极高（作者 Giovanni Di Sirio 的风格极严谨）
        ⇒ 飞控领域的事实标准之一，且是 ArduPilot 的独家选择

      NuttX:
        - **PX4**（全部 MCU 目标）
        - **ArduPilot 曾经用**（AP_HAL_NuttX 已移除，见实验 7）
        - Sony 的 Spresense 开发板官方 OS
        - 小米 Vela（基于 NuttX）
        - 三星 TizenRT（基于 NuttX）
        - Google 的某些嵌入式项目
        - **NASA** 的部分项目、以及多个航天项目（NuttX 有辐射容错相关的
          认证努力和 DO-178 相关的合规工作）
        - 无人机/航天领域增长最快的 RTOS
        ⇒ 定位最"正统"：一个完整的 POSIX 嵌入式 OS，不是一个内核
""")

    print("  " + "=" * 100)
    print("  【一句话定位】")
    print("  " + "=" * 100)
    print("""
      FreeRTOS : 一个**内核**。给你调度和同步，其余全靠自己。
                 生态最大、最轻、许可最宽松、抽象最少。
                 适合：MCU + 少量任务 + 自己掌控一切。
                 不适合：需要文件系统/网络/shell/驱动框架的复杂系统 ——
                 你会花大量时间在重新发明 NuttX 已经有的东西。

      ChibiOS  : 一个**内核 + 高质量 HAL**。给你调度、同步、外设驱动，
                 但没有设备抽象层。静态配置、编译期决定一切。
                 适合：需要精细控制、愿意深度耦合、团队熟悉 STM32。
                 不适合：需要多硬件平台统一固件、需要运行期扩展。
                 注意：GPL3/商业双轨许可。

      NuttX    : 一个**完整的 OS**。内核 + VFS + 网络 + 文件系统 + shell +
                 POSIX API。你写的应用代码几乎不需要知道自己跑在 RTOS 上。
                 适合：复杂系统、需要 POSIX 可移植性、需要动态加载、
                       商业闭源产品、多硬件平台统一固件。
                 不适合：要求极限确定性延迟、团队不愿意学习 VFS/驱动模型。
                 注意：「不适合 RAM 受限」是一个**错误的成见** —— 见下。
""")

    print("  " + "=" * 100)
    print("  【一个重要的修正：NuttX 到底能在多小的 RAM 里跑】")
    print("  " + "=" * 100)
    print("""
  我在写这一课时第一版写了「RAM < 64 KB → 不要选 NuttX」。
  然后在 PX4 仓库里找到了直接反驳这个说法的例子：

    $ for f in boards/*/*/nuttx-config/nsh/defconfig; do
          v=$(grep -m1 "^CONFIG_RAM_SIZE=" "$f" | cut -d= -f2); echo "$((v/1024)) $f"
      done | sort -n | head -5

        8  boards/cubepilot/io-v2/nuttx-config/nsh/defconfig
        8  boards/modalai/voxl2-io/nuttx-config/nsh/defconfig
        8  boards/px4/io-v2/nuttx-config/nsh/defconfig
      124  boards/nxp/ucans32k146/nuttx-config/nsh/defconfig
      128  boards/matek/gnss-m9n-f4/nuttx-config/nsh/defconfig
      192  boards/bitcraze/crazyflie/nuttx-config/nsh/defconfig

  8 KB。Cortex-M3。跑的是 PX4IO 固件（CONFIG_MODULES_PX4IOFIRMWARE=y），
  也就是负责 RC 输入输出和失控保护的那个 I/O 协处理器。

  它怎么做到的？看 defconfig 被裁成了什么样：

    %-32s %-24s %s
""" % ("配置项", "cubepilot/io-v2 (PX4IO)", "ark/fmu-v6x (完整 PX4)"))
    rows_cfg = [
        ("CONFIG_ 行数", "53 行", "275 行"),
        ("CONFIG_RAM_SIZE", "8 KB", "240 KB（MM_REGIONS=4 合计约 992 KB）"),
        ("架构", "Cortex-M3", "Cortex-M7"),
        ("CONFIG_DISABLE_MOUNTPOINT", "=y（没有文件系统挂载点）", "未设"),
        ("CONFIG_FS_FAT", "0", "2"),
        ("CONFIG_MM_REGIONS", "0", "1"),
        ("CONFIG_PRIORITY_INHERITANCE", "0（PI 都没开）", "1"),
        ("CONFIG_SIG_DEFAULT", "0", "1"),
        ("CONFIG_LIBC_*", "0", "4"),
        ("CONFIG_NET*", "0", "28"),
        ("CONFIG_MM_SMALL", "=y", "未设"),
    ]
    for r in rows_cfg:
        print("    %-32s %-24s %s" % r)
    print("""
  配置行数差 5.2 倍，RAM 差 30 倍。

  ⇒ 关键开关是 CONFIG_DISABLE_MOUNTPOINT=y —— 它把整个 VFS 的
    挂载点机制关掉，于是不需要 inode 树、不需要文件描述符表、
    不需要块设备层。剩下的就是「调度器 + 同步 + 中断 + 一个
    精简的内存分配器」。

  ⇒ **这修正了一个常见的认知错误（也是我自己的）：**
     "NuttX 是完整 OS，所以它一定比 FreeRTOS 大" —— 不对。
     NuttX 是**可裁剪的完整 OS**：
       最小配置下    和 FreeRTOS 是同一量级
       最大配置下    是 FreeRTOS 的几十倍
     真正的问题不是「NuttX 有多大」，而是「你需要它有多大」——
     而这个答案由功能需求决定，不由 OS 的名字决定。

  ⇒ **同时这是一个可直接抄的架构模式**：PX4 在同一个项目里用了
     两套差异 5 倍的 NuttX 配置 —— FMU 上跑完整 PX4（275 行配置），
     IO 协处理器上跑精简 PX4IO（53 行配置）。两块芯片用同一个 RTOS，
     配置完全不同。
     当你的系统有「主控 + 协处理器」结构时，不需要为两者选不同的
     RTOS —— 选同一个，用不同的裁剪配置。
""")


# ============================================================================
# 主入口
# ============================================================================

EXPERIMENTS = {
    "1": ("就绪队列的数据结构", exp1_ready_list),
    "2": ("优先级方向与数值区间", exp2_priority_direction),
    "3": ("ISR 安全 API 的三种设计", exp3_isr_api),
    "4": ("中断→线程通信原语映射", exp4_ipc),
    "5": ("优先级继承的三种态度", exp5_priority_inheritance),
    "6": ("内存管理对比", exp6_memory),
    "7": ("驱动模型对比", exp7_driver_model),
    "8": ("量化对照总表", exp8_summary),
}


def main():
    banner = """
================================================================================
 第 10 课：NuttX vs FreeRTOS vs ChibiOS
 配套实验代码 —— 所有数字都可在本机仓库复现

 实验列表：
   1  就绪队列的数据结构 —— 单有序链表 vs 按优先级分桶数组
   2  优先级方向与数值区间 —— 迁移时最容易搞错的一件事
   3  ISR 安全 API 的三种设计 —— FromISR / I 后缀 / 无后缀
   4  中断 → 线程通信原语的映射
   5  优先级继承的三种态度 —— 和它引发的死锁自救
   6  内存管理对比 —— heap_1~5 / 单堆 / 统一堆，并亲手跑一遍合并算法
   7  驱动模型对比 —— 加一个新 I2C 传感器要改多少处代码
   8  量化对照总表 —— 代码规模 / 许可证 / 谁在用 / 一句话定位
================================================================================
"""
    print(banner)

    args = sys.argv[1:]
    if args:
        for a in args:
            if a in EXPERIMENTS:
                EXPERIMENTS[a][1]()
            else:
                print("未知实验编号: %s（可选：%s）"
                      % (a, ", ".join(sorted(EXPERIMENTS))))
    else:
        t0 = time.time()
        for k in sorted(EXPERIMENTS):
            EXPERIMENTS[k][1]()
        print()
        print("=" * 78)
        print("全部实验完成，用时 %.2f 秒。" % (time.time() - t0))

    print("""
================================================================================
 本课三条最重要的结论（如果只记三件事）
================================================================================

1. 优先级永远不要跨 RTOS 照搬数字，要搬**语义档位**。
   ArduPilot 用 AP_HAL::Scheduler::priority_base 枚举
   （libraries/AP_HAL/Scheduler.h:104-117，14 个档位）解决这个问题。
   PX4 的优先级语义（小 = 高）和 ChibiOS/NuttX 原生（大 = 高）相反，
   照搬必然出错，而且系统还能跑 —— 这类 bug 最难定位。

2. "API 需不需要 ISR 变体"不取决于 OS 作者的喜好，
   而取决于**上下文切换在哪里发生**。
   NuttX 能把 nxsem_post() 统一成一个函数，
   是因为 arm_doirq() 用"返回不同的栈指针"把切换推迟到了 ISR 退出；
   FreeRTOS 和 ChibiOS 需要 FromISR / I 后缀，
   是因为它们的切换决策就地发生。
   这条推论可以推广到任何 RTOS 的设计分析。

3. 选 NuttX 还是选裸 RTOS，本质上是回答"要不要设备抽象"。
   要 → NuttX（VFS，运行期注册，上层硬件无关，SITL 与真机同代码）
   不要 → FreeRTOS/ChibiOS（零间接层，编译期全优化，代码路径最短）
   PX4 和 ArduPilot 各自选了这两条路，并各自承担了对应的代价：
     PX4       → 灵活性 + VFS 学习成本 + 每设备一个 inode
     ArduPilot → 零开销 + 100+ 块板子各一份固件 + 无法运行期扩展
   两边在各自的约束下都是正确的。
================================================================================
""")


if __name__ == "__main__":
    main()
