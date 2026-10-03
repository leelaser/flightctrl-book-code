#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 8 课 · 低功耗与 Tickless idle —— 配套实验

    实验 1 ★★  tickless 省了什么：数一数 tick 中断
    实验 2 ★★  prvGetExpectedIdleTime：四个「不能睡」的判断
    实验 3 ★★★ vTaskStepTick：补 tick 的那个「退一格」
    实验 4 ★★  常量计算器 + ChibiOS 对照 + ArduPilot 的实际配置

用法：
    python lesson08_tickless.py 1
    python lesson08_tickless.py 2
    ...
    python lesson08_tickless.py          # 全部

只需 numpy（画图需要 matplotlib）。

=======================================================================
参考源码（FreeRTOS-Kernel V11.1.0，D:/DEVLib/PX4APM/FreeRTOS/）
=======================================================================

include/FreeRTOS.h:2698-2700  configEXPECTED_IDLE_TIME_BEFORE_SLEEP 必须 ≥ 2
include/FreeRTOS.h:2702-2704  ★ configUSE_TICKLESS_IDLE 默认 0（关）
include/FreeRTOS.h:2706-2716  三个钩子：PRE_SUPPRESS / PRE_SLEEP / POST_SLEEP

tasks.c:665-675               ★ 注释解释了为什么判据用 `!= 0` 而不是 `== 1`
tasks.c:3891-3954             ★★ prvGetExpectedIdleTime 的四个分支
tasks.c:4499-4543             ★★★ vTaskStepTick（补 tick，含「退一格」）
tasks.c:5884-5934             ★★ eTaskConfirmSleepModeStatus
tasks.c:7915-7920             下一个唤醒时刻的重算（tick 处理里）
tasks.c:5373,5457             vTaskPlaceOnEventList 里对 xNextTaskUnblockTime 的维护

portable/GCC/ARM_CM7/r0p1/port.c:88-99     ★ 三个常量的定义（94 是什么）
portable/GCC/ARM_CM7/r0p1/port.c:158-178   tickless 用的三个静态变量
portable/GCC/ARM_CM7/r0p1/port.c:581-700   ★★★ vPortSuppressTicksAndSleep 前半
portable/GCC/ARM_CM7/r0p1/port.c:677-780   ★★★ 醒来后的 tick 补偿
portable/GCC/ARM_CM7/r0p1/port.c:805-812   ★ 三个常量的计算

ChibiOS（ArduPilot fork）
modules/ChibiOS/os/rt/include/chtime.h 附近   CH_CFG_ST_TIMEDELTA 的语义
modules/ChibiOS/os/license/chcustomer.h:70    CH_LICENSE = CH_LICENSE_GPL
modules/ChibiOS/os/license/chlicense.h:586-612 ★ GPL→FULL / GPL+Exception→BASIC
modules/ChibiOS/os/license/chrestrictions.h   ★★ BASIC 档把 ST_TIMEDELTA 强制成 0
libraries/AP_HAL_ChibiOS/hwdef/common/chconf.h:135-144  ★ 默认 10
libraries/AP_HAL_ChibiOS/hwdef/iomcu-dshot/hwdef.inc:37 ★ 唯一一个设成 2 的
"""

import sys

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_PLT = True
except ImportError:
    HAVE_PLT = False


def hr(ch="=", n=78):
    print(ch * n)


def head(title):
    print()
    hr()
    print("  " + title)
    hr()


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (len(title) + 2))


def tab(rows, headers, indent="  "):
    widths = [len(h) for h in headers]
    for r in rows:
        for i, c in enumerate(r):
            widths[i] = max(widths[i], len(str(c)))
    print(indent + " | ".join(h.ljust(widths[i]) for i, h in enumerate(headers)))
    print(indent + "-+-".join("-" * w for w in widths))
    for r in rows:
        print(indent + " | ".join(str(c).ljust(widths[i]) for i, c in enumerate(r)))


# ===========================================================================
# 公共：三个常量（照抄 port.c:809-811）
# ===========================================================================
#
#     ulTimerCountsForOneTick      = configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ
#     xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick
#     ulStoppedTimerCompensation   = portMISSED_COUNTS_FACTOR
#                                    / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ)
#
# portMAX_24_BIT_NUMBER   = 0xffffffUL     （SysTick 是 24 位计数器）
# portMISSED_COUNTS_FACTOR = 94UL          （"a fiddle factor"，见实验 4）

PORT_MAX_24_BIT = 0xFFFFFF
PORT_MISSED_COUNTS_FACTOR = 94


def tickless_constants(cpu_hz, systick_hz, tick_hz):
    """返回 port.c:809-811 那三行算出来的三个常量。"""
    counts_per_tick = systick_hz // tick_hz
    max_suppressed = PORT_MAX_24_BIT // counts_per_tick
    stopped_comp = PORT_MISSED_COUNTS_FACTOR // (cpu_hz // systick_hz)
    return counts_per_tick, max_suppressed, stopped_comp


# ===========================================================================
# 实验 1：tickless 省了什么
# ===========================================================================

def exp1():
    head("实验 1  tickless 省了什么：数一数 tick 中断")

    print("""
  先说清楚问题在哪。

  RTOS 的 tick 中断是【周期性】的：不管有没有活干，每 1/tick_hz 秒
  必然中断一次。而绝大多数嵌入式系统在大部分时间里是空闲的。

  ★ 空闲时那次中断做了什么？（任务把第 2 课的五条链表串起来了）

      SysTick ISR
        → xTaskIncrementTick()
            → xTickCount++
            → 检查延时链表，看有没有任务到期
            → 没有到期任务？什么都不做，返回
        → 检查是否需要切换
            → 不需要？还是运行 idle 任务
        → 返回，继续 WFI

  ⇒ **醒来一趟，发现没事，又睡回去。**
    这件事每秒发生 tick_hz 次，而它只在有任务到期时才有意义。
""")

    sub("① 量化：空闲时每秒被白叫醒多少次")
    print("  假设板子只是闲着（没有任务到期），统计一个「10 秒空转」窗口：\n")
    rows = []
    for tick_hz, name in [(100, "常见的低功耗配置"),
                          (1000, "★ 飞控常用"),
                          (10000, "高响应配置")]:
        n = tick_hz * 10
        rows.append([name, "%d Hz" % tick_hz, "%d 次" % n,
                     "%d 次" % 0, "%d 次" % n])
    tab(rows, ["配置", "tick 频率", "10 秒内 tick 中断", "其中有用", "白醒"])

    print("""
  ★ 注意「其中有用」那一列是 0 —— 因为这一栏的前提就是「没有任务到期」。
    真实系统里会有一些到期任务，但比例通常很低。
""")

    sub("② tickless 的做法：把中断延后到「下一个有用的时刻」")
    print("""  核心思路只有一句话：

      既然知道【下一个任务什么时候到期】，
      那就把 tick 中断推迟到那一刻，中间不要叫醒我。

  ★ 那个「下一个到期时刻」从哪来？—— `xNextTaskUnblockTime`。

    tasks.c:675-678（声明处的注释）：
        /* Set xNextTaskUnblockTime to the time at which the next Blocked
         * state task will exit the Blocked state. */

  ⇒ 它是**延时链表队头的到期时间**，在内核里一直是维护着的
    （第 2 课讲过延时链表按到期时间排序，所以队头就是最近的）。

  所以 tickless **没有引入新数据结构** ——
  它只是把已有的 `xNextTaskUnblockTime` 用起来了。
""")

    sub("③ 省了多少：一个典型场景")
    print("""  场景：飞控主循环 400 Hz，但传感器是 100 Hz 采样、
        每 10 ms 有一条任务到期。tick 频率 1000 Hz。

        有 tickless：每 10 ms 醒一次  →  10 秒内 1000 次
        无 tickless：每 1 ms 醒一次   →  10 秒内 10000 次

  下面把这个账算清楚：
""")
    tick_hz = 1000
    wake_period_ms = 10
    window_s = 10
    with_tl = window_s * 1000 // wake_period_ms
    without_tl = window_s * tick_hz
    rows = [
        ["无 tickless", "%d 次" % without_tl, "1.000 ms", "—"],
        ["有 tickless", "%d 次" % with_tl, "%d ms" % wake_period_ms,
         "%.1f×" % (without_tl / with_tl)],
    ]
    tab(rows, ["", "10 秒内唤醒", "平均间隔", "减少"])
    print("""
  ★ **减少 10 倍**（等于 tick_hz / 实际唤醒频率）。

  ⇒ 这个倍数就是 tickless 的价值：**它正比于「tick 频率 ÷ 真实活动频率」。**
    所以 tick 频率设得越高（比如为了响应精度），tickless 的收益越大。
""")

    sub("④ 但代价是什么？")
    print("""  ★ 三件事必须说清楚，否则「开了 tickless」会变成新的坑：

    ┌────────────────────────────────────────────────────────────┐
    │ 1. 时间精度        睡过去的那段时间靠【从硬件读】补偿，    │
    │                    会有微小漂移（源码里承认了，见实验 3）   │
    │                                                            │
    │ 2. 最少睡多久      睡一觉的开销（改 SysTick 重载值、        │
    │                    进出临界区）可能比省下的还大            │
    │                    ⇒ 所以有 EXPECTED_IDLE_TIME_BEFORE_SLEEP │
    │                                                            │
    │ 3. 唤醒来由        只有【中断】能把你叫醒 ——               │
    │                    如果外设没配好，可能一睡不醒            │
    └────────────────────────────────────────────────────────────┘
""")
    print("""  ★ 第 2 条在源码里是硬约束（include/FreeRTOS.h:2698-2700）：

      #if configEXPECTED_IDLE_TIME_BEFORE_SLEEP < 2
          #error configEXPECTED_IDLE_TIME_BEFORE_SLEEP must not be less than 2
      #endif

  ⇒ **至少预期空闲 2 个 tick 才允许睡。**
    源码里还有一句注释解释为什么（port.c 里）：

      /* The expected idle time is always at least two ticks. */

  ★ 为什么是 2 而不是 1？因为睡一觉的逻辑本身要花掉一部分第一个 tick，
    低于 2 就纯亏。
""")


# ===========================================================================
# 实验 2：prvGetExpectedIdleTime 的四个分支
# ===========================================================================

def exp2():
    head("实验 2  prvGetExpectedIdleTime：四个「不能睡」的判断")

    print("""
  空闲任务每轮会问一句：**「我能睡多久？」**

  答案由 `prvGetExpectedIdleTime()`（tasks.c:3891-3954）给出。
  返回 0 就是「别睡」，返回正整数就是「能睡这么多 tick」。

  ★ 它有**四个返回 0 的分支**，每个都在挡一种情况。
""")

    sub("① 逐个分支")
    print("""  tasks.c:3919-3950（结构照抄，注释为原文精简）：

      if( pxCurrentTCB->uxPriority > tskIDLE_PRIORITY )
      {
          xReturn = 0;                       /* 分支 A */
      }
      else if( listCURRENT_LIST_LENGTH( &pxReadyTasksLists[tskIDLE_PRIORITY] ) > 1U )
      {
          xReturn = 0;                       /* 分支 B */
      }
      else if( uxHigherPriorityReadyTasks != pdFALSE )
      {
          xReturn = 0;                       /* 分支 C */
      }
      else
      {
          xReturn = xNextTaskUnblockTime;    /* 分支 D：真的能睡 */
          xReturn -= xTickCount;
      }
""")

    rows = [
        ["A", "当前任务优先级 > idle",
         "← 不是空闲任务在跑，凭什么睡", "0"],
        ["B", "idle 优先级的就绪任务 > 1 个",
         "有多个空闲级任务 ⇒ 时间片轮转必须按 tick 走", "0"],
        ["C", "有更高优先级任务处于就绪态",
         "← 只在 configUSE_PREEMPTION == 0 时可能（协作式）", "0"],
        ["D", "以上都不是",
         "xNextTaskUnblockTime − xTickCount", "> 0"],
    ]
    tab(rows, ["分支", "条件", "为什么", "返回"])

    sub("② ★ 分支 B 值得单独说：它是「时间片」和「tickless」的冲突点")
    print("""  源码注释（tasks.c:3926-3928）写得很明确：

      /* There are other idle priority tasks in the ready state.  If
       * time slicing is used then the very next tick interrupt must be
       * processed. */

  ⇒ **时间片轮转是按 tick 切分的**（第 2 课讲过，靠 `pxIndex` 游标，
    而游标的推进发生在 tick 中断里）。

  ★★ 所以：**只要开了时间片轮转，且有 ≥2 个同优先级任务，
     就永远不能睡** —— 因为 tick 一停，轮转就停了。

  这是一个很实在的取舍：

      configUSE_TIME_SLICING = 1  ⇒  多任务公平，但 tickless 失效
      configUSE_TIME_SLICING = 0  ⇒  tickless 可用，但同优先级任务不轮转

  ★ 那 ArduPilot 选哪个？—— 查过了，**答案是"没选"**：
    FreeRTOS 的 `configUSE_TIME_SLICING` **默认是 1**（`FreeRTOS.h:2734-2736`），
    而 ArduPilot 的 ESP32 全树 grep `time_slicing` **零命中**
    （仓库里根本没有 `FreeRTOSConfig.h` —— 它是构建时的外部组件）。
    ⇒ 按本课的纪律：**零命中 ⇒ 走默认值 ⇒ 时间片是开着的。**

    ⚠️ 这里原来写的是"飞控开发者**主动关掉**了它" —— **没有依据**，
      是"看到默认是 1 就以为飞控必然要改它"的想当然。
      （真正"主动关掉时间片"的是 **ChibiOS 侧**：
        ArduPilot 的 `chconf.h:178-179` 把 `CH_CFG_TIME_QUANTUM` 设成了 0。
        那是明确可查的一行，不是推断。）

  ⇒ 也就是说：**飞控这边本来就给 tickless 留了门。**
    虽然实际上 ESP32 平台并没有开 tickless（见实验 4）。
""")

    sub("③ 把四个分支跑一遍")
    print("  （`xNextTaskUnblockTime = 1500`，`xTickCount = 1000`）\n")
    X_NEXT = 1500
    X_TICK = 1000

    def expected_idle_time(cur_prio, n_idle_ready, higher_ready, preemption):
        """★ 模拟 tasks.c:3894-3951 的四个分支。

        关键在分支 C 的判据 `uxHigherPriorityReadyTasks`
        （tasks.c:3897-3924）—— 它在两种调度模式下**算法不一样**：

            协作式（configUSE_PREEMPTION == 0） ⇒ **恒为 TRUE**
            抢占式                                ⇒ (uxTopReadyPriority > tskIDLE_PRIORITY)

        ⇒ 于是"抢占式 + 有更高优先级任务就绪"这个组合**不可达**：
          抢占式下真有更高优先级任务就绪，它早就被调度了，
          **idle 任务根本没机会运行**（源码注释原话：
          "can only be reached if configUSE_PREEMPTION is 0"）。
        """
        if cur_prio > 0:
            return 0, "A：当前不是 idle 任务"
        if n_idle_ready > 1:
            return 0, "B：多个 idle 级就绪任务 ⇒ 时间片要推进"
        if higher_ready:
            if not preemption:
                return 0, "C：有更高优先级就绪（协作式）"
            # ★ 不可达 —— 用 None 表示"这个状态进不来"，不是"能睡"
            return None, "★ 不可达（抢占式下它早就被抢占了）"
        return X_NEXT - X_TICK, "D：可以睡"

    cases = [
        ("正常空闲", 0, 1, 0, 1),
        ("有任务在跑", 3, 1, 1, 1),
        ("两个 idle 级任务（开了时间片）", 0, 2, 0, 1),
        ("协作式 + 高优先级就绪", 0, 1, 1, 0),
        ("★ 同上的抢占式（不可达）", 0, 1, 1, 1),
    ]
    rows = []
    for name, p, ni, hr_, pre in cases:
        t, why = expected_idle_time(p, ni, hr_, pre)
        rows.append([name, str(p), str(ni), "是" if hr_ else "否",
                     "开" if pre else "关",
                     "—" if t is None else str(t), why])
    tab(rows, ["场景", "当前优先级", "idle 就绪数", "高优先就绪", "抢占",
               "能睡(tick)", "命中分支"])

    print("""
  ★ 看最后一行：**「抢占式 + 有更高优先级任务就绪」这个组合【不可达】。**

    ⚠️ 这里原来打的是「可以睡（500 tick）」—— **那是错的**。
    错在把 `uxHigherPriorityReadyTasks` 当成了"当前状态"，
    而它在源码里是**按调度模式分别算出来的**：

        协作式（configUSE_PREEMPTION == 0） ⇒ **恒为 TRUE**
        抢占式                              ⇒ (uxTopReadyPriority > tskIDLE_PRIORITY)

    ⇒ 抢占式下真有更高优先级任务就绪时，它**早就被调度了** ——
      **idle 任务根本没机会运行**，这个状态进不来。
      （源码注释原话：分支 C "can only be reached if configUSE_PREEMPTION is 0"）

  ⇒ 所以分支 C 是**为协作式准备的**：
    协作式调度器下，就绪的高优先级任务**不会**自动抢占 ——
    它只在当前任务主动让出时才跑。所以 idle 任务一睡，
    那个高优先级任务就没机会跑了。

  ★★ 这是「同一个判据在不同调度策略下含义不同」的一个好例子 ——
     **而且比"含义不同"更强：它连算法都不一样。**
""")


# ===========================================================================
# 实验 3：vTaskStepTick 的补偿
# ===========================================================================

def exp3():
    head("实验 3  vTaskStepTick：补 tick 的那个「退一格」")

    print("""
  睡醒之后有一件必须做的事：**把睡着期间「欠下」的 tick 补上。**

  否则 `xTickCount` 会落后于真实时间，所有延时都会变长。

  ★ 但补偿不是简单地"加上去"。看源码（tasks.c:4501-4543）：

      void vTaskStepTick( TickType_t xTicksToJump )
      {
          TickType_t xUpdatedTickCount;

          /* Correct the tick count value after a period during which the tick
           * was suppressed.  Note this does *not* call the tick hook function for
           * each stepped tick. */
          xUpdatedTickCount = xTickCount + xTicksToJump;
          configASSERT( xUpdatedTickCount <= xNextTaskUnblockTime );

          if( xUpdatedTickCount == xNextTaskUnblockTime )
          {
              /* Arrange for xTickCount to reach xNextTaskUnblockTime in
               * xTaskIncrementTick() when the scheduler resumes.  This ensures
               * that any delayed tasks are resumed at the correct time. */
              ...
              xPendedTicks++;            /* ★ 补一个 pending tick */
              xTicksToJump--;            /* ★ 少跳一格 */
          }

          xTickCount += xTicksToJump;
      }

  ★★ 三个关键点，逐个说。
""")

    sub("① 为什么有 `configASSERT( xUpdatedTickCount <= xNextTaskUnblockTime )`")
    print("""  因为 `vTaskStepTick` **只改 xTickCount，不处理延时链表**。

  源码注释自己承认了这一半（"Note this does *not* call the tick hook function
  for each stepped tick"），但更关键的是它**也不调用 xTaskIncrementTick()** ——
  也就是说，**没有任务会因为这次跳变而被移到就绪链表**。

  ⇒ 所以必须保证：`xTickCount` 不会**越过**任何一个任务的到期时刻。
     否则那个任务就被"跳过了"，永远等不到唤醒。

  ★ 这正是那个 assert 在守的边界。
    而 assert 成立的前提是：port 层跳过的 tick 数**不超过**
    `prvGetExpectedIdleTime()` 给出的值（因为后者就是 `xNextTaskUnblockTime - xTickCount`）。
""")

    sub("② ★★★ 为什么「正好落在唤醒点」要退一格")
    print("""  这是整个 tickless 里最微妙的一处。

  假设：

      xTickCount           = 1000
      xNextTaskUnblockTime = 1500
      睡了 500 个 tick

  那 `xUpdatedTickCount = 1000 + 500 = 1500 == xNextTaskUnblockTime`。

  ★ 如果**直接跳**：

      xTickCount = 1500
      ⇒ 但延时链表里的那个任务**还在链表上**（因为没调 xTaskIncrementTick）
      ⇒ 它的到期时刻就是 1500 = 当前 tick
      ⇒ 下一次 tick 中断（真正的那条路径）会处理它吗？

    **不会！** 因为 `xTaskIncrementTick()` 的逻辑是
    「`xTickCount` 到达到期时刻时把它摘下来」——
    而现在 `xTickCount` **已经是** 1500 了，
    下一次进来是 1501，**已经错过了等于 1500 那一拍**。

  ★ 所以源码选择**退一格 + 挂一个 pending tick**：

      xPendedTicks++       ← 记住"还欠一个 tick"
      xTicksToJump--       ← 少跳一格（跳 499 而不是 500）
      xTickCount += 499    → xTickCount = 1499

    然后恢复调度时，pending tick 会被 `xTaskIncrementTick()` 处理，
    走**正常的路径**：xTickCount 从 1499 → 1500，**正好命中**到期时刻，
    把任务摘下来放进就绪链表。

  ⇒ **退的这一格，是为了让临界的那一拍走正规路径。**
""")

    sub("③ 数值验证：两种做法谁会漏掉任务")
    print("  模拟一个在 tick 1500 到期的任务，睡 500 tick：\n")

    def simulate(back_off, x_tick=1000, x_next=1500, jump=500):
        """返回 (最终 xTickCount, 到期任务有没有被摘下来)"""
        if back_off and x_tick + jump == x_next:
            pended = 1
            jump -= 1
        else:
            pended = 0
        x_tick += jump
        # 恢复调度后处理 pending tick（走正规路径）
        woken = False
        for _ in range(pended):
            x_tick += 1
            if x_tick == x_next:        # xTaskIncrementTick 的判据
                woken = True
        return x_tick, woken, pended

    rows = []
    for back_off, name in [(False, "不退格（直接跳 500）"),
                           (True, "★ 退格（跳 499 + 1 个 pending）")]:
        xt, woken, pend = simulate(back_off)
        rows.append([name, "1000", "500" if not back_off else "499",
                     str(pend), str(xt),
                     "✅ 被唤醒" if woken else "❌ 漏掉了！"])
    tab(rows, ["做法", "起始", "跳", "pending", "最终 tick", "tick 1500 到期的任务"])

    print("""
  ⇒ **不退格的做法会永久漏掉那个任务。**

  ★ 而且注意：这个 bug 的表现是「偶发某个延时变长」——
    只在**跳变正好落在唤醒点**时发生。
    如果实际空闲时间总是比预期短一点（被别的中断打断），
    就永远触发不到 —— 于是在实验室里测不出来。
""")

    sub("④ ★ 那「预期空闲时间」本身准不准？")
    print("""  源码（port.c:604-611）承认了不准：

      /* Stop the SysTick momentarily.  The time the SysTick is stopped for
       * is accounted for as best it can be, but using the tickless mode will
       * inevitably result in some tiny drift of the time maintained by the
       * kernel with respect to calendar time. */

  ⇒ **"as best it can be" + "inevitably result in some tiny drift"** ——
    内核作者自己写了两遍"会有漂移"。

  ★ 漂移从哪来？三处（都在 port.c 里）：

      1. 停表/重启表那几条指令本身花的时间
         ⇒ 用 `ulStoppedTimerCompensation`（= 94 个 SysTick 计数）去补

      2. `wfi` 醒来还需要几个周期才真正执行到读表
         ⇒ 同上

      3. `configPOST_SLEEP_PROCESSING` 钩子里做的事（比如切时钟源）
         ⇒ 补不了

  ★ 所以 tickless 是一个**精度换功耗**的交易 ——
    需要严格时间基准的场合（比如飞控的定时）不该指望它。
""")

    sub("⑤ ulStoppedTimerCompensation 里的 94 是怎么来的")
    print("""  port.c:97-99：

      /* A fiddle factor to estimate the number of SysTick counts that would have
       * occurred while the SysTick counter is stopped during tickless idle
       * calculations. */
      #define portMISSED_COUNTS_FACTOR              ( 94UL )

  ★ **"A fiddle factor"** —— 直译「一个瞎凑的系数」。

  ⇒ 连内核作者都不假装它是推导出来的。它是一个**实测出来的经验值**：
    "从停表到重启表，大概丢掉 94 个 SysTick 计数"。

  ★★ 这条值得单独记，因为它是本教程里第一个**明确承认自己是凑的**常数。

    对照前面几课看到的常数：
        heap_4 的 `heapSTRUCT_SIZE`  —— 算出来的（对齐规则）
        event_groups 的 `0xff000000` —— 设计出来的（控制位占最高字节）
        `portMISSED_COUNTS_FACTOR`   —— **凑出来的**

  ⇒ 工程里这三类常数都存在。**问题不在于"凑"，而在于凑的时候有没有说清楚。**
    这里说清楚了（连 "fiddle" 这个词都用上了）。
""")


# ===========================================================================
# 实验 4：常量计算器 + ChibiOS 对照 + ArduPilot 的实际配置
# ===========================================================================

def exp4():
    head("实验 4  常量计算器 + ChibiOS 对照 + ArduPilot 的实际配置")

    sub("① 三个常量算出来是多少")
    print("""  照抄 port.c:809-811：

      ulTimerCountsForOneTick      = configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ
      xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick
      ulStoppedTimerCompensation   = portMISSED_COUNTS_FACTOR
                                     / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ)
""")
    print("  下面算几个典型的 MCU 配置（均为假设值，只看数量级）：\n")
    rows = []
    for cpu_mhz, tick_hz, name in [(480, 1000, "STM32H7 480 MHz"),
                                   (216, 1000, "STM32F7 216 MHz"),
                                   (168, 1000, "STM32F4 168 MHz"),
                                   (480, 10000, "STM32H7 + 10 kHz tick")]:
        cpu_hz = cpu_mhz * 1_000_000
        systick_hz = cpu_hz          # 默认 configSYSTICK_CLOCK_HZ = configCPU_CLOCK_HZ
        c, m, s = tickless_constants(cpu_hz, systick_hz, tick_hz)
        rows.append([name, "%d" % c, "%d" % m,
                     "%.1f ms" % (1000.0 * m / tick_hz), "%d" % s])
    tab(rows, ["配置", "每 tick 计数", "最多抑制 tick", "≈ 最长睡", "停表补偿"])

    print("""
  ★ 注意最后两行的「最长睡」是**一样的**（34 ms 左右）——
    因为上限来自 SysTick 的 24 位计数范围除以【SysTick 时钟】，
    和 tick 频率无关：

        最长秒数 = 0xFFFFFF / configSYSTICK_CLOCK_HZ
                 = 16777215 / 480,000,000  ≈ 0.0349 s

    ⇒ **提高 tick 频率只会让"能睡几个 tick"变多，不会让你睡更久。**

  ★ 看第三列：**最多抑制的 tick 数受 SysTick 的 24 位限制。**

      SysTick 是 24 位递减计数器  ⇒  最多 16,777,215 个计数
      除以「每个 tick 多少计数」 ⇒  最多能睡多少个 tick

  ⇒ 在 480 MHz / 1 kHz tick 下是 34 个 tick = **34 ms**。
    也就是说：**想睡更久，tickless 这条内置路径做不到** ——
    需要走 `configUSE_TICKLESS_IDLE == 2`（用户自定义实现，
    比如换成 RTC 或 32.768 kHz 低速时钟）。

  ★★ 这是「内置实现的能力边界」：
     内置版只做**浅睡**（stop SysTick，CPU 进 WFI），
     不做**深睡**（关主时钟、切 RTC）。

  ★ 第四列也注意：**`ulStoppedTimerCompensation` 和 CPU 频率成反比。**
    因为 `(configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ)` 默认是 1，
    所以它恒等于 94 —— 也就是说**默认配置下这个补偿根本没随频率调整**。
    只有在显式定义了 `configSYSTICK_CLOCK_HZ` ≠ CPU 频率时才有意义。
""")

    sub("② ★ 三种模式：0 / 1 / 2")
    print("""  include/FreeRTOS.h:2702-2704 只给了一个默认值：

      #ifndef configUSE_TICKLESS_IDLE
          #define configUSE_TICKLESS_IDLE    0
      #endif

  而 tasks.c:665-671 的注释解释了为什么判断用 `!= 0` 而不是 `== 1`：

      /* This conditional compilation should use inequality to 0, not equality to 1.
       * This is to ensure portSUPPRESS_TICKS_AND_SLEEP() can be called when user
       * defined low power mode implementations require configUSE_TICKLESS_IDLE to be
       * set to a value other than 1. */

  三种模式：
""")
    rows = [
        ["0", "关（默认）", "tick 中断照常跑", "—"],
        ["1", "内置实现", "用 port 自带的 vPortSuppressTicksAndSleep", "★ 需要 port 支持"],
        ["2", "自定义", "应用层提供 portSUPPRESS_TICKS_AND_SLEEP()", "可做深睡（换 RTC）"],
    ]
    tab(rows, ["值", "含义", "行为", "备注"])
    print("""
  ★ 注意 `!= 0` 这个写法 —— 它是**前瞻性设计**：
    以后加模式 3、4 时不用改那些 `#if`。

  ⇒ 和本线第 10 课那条「`#if` 的判据方式本身就是设计信息」是同一个观察。
""")

    sub("③ ChibiOS 的对应机制：CH_CFG_ST_TIMEDELTA")
    print("""  三个 OS 里，ChibiOS 的 tickless 是**配置项的一个值**，而不是布尔开关。

  modules/ChibiOS/.../chconf.h（ArduPilot 的版本）:135-144：

      /**
       * @brief   Time delta constant for the tick-less mode.
       * @note    If this value is zero then the system uses the classic
       *          periodic tick. This value represents the minimum number
       *          of ticks that is safe to specify in a timeout directive.
       *          The value one is not valid, timeouts are rounded up to
       *          this value.
       */
      #ifndef CH_CFG_ST_TIMEDELTA
      #define CH_CFG_ST_TIMEDELTA                 10
      #endif

  ⇒ **0 = 关（经典周期 tick）；> 0 = 开，值的含义是「最小的安全超时量」。**

  ★ 为什么不能是 1？注释写了：`The value one is not valid`。
    因为 tickless 下每个超时都要被**向上取整**到这个值 ——
    如果是 1，等于每次超时都要求"至少 1 tick"，那 tickless 就白开了。

  ★★ 这和 FreeRTOS 的 `configEXPECTED_IDLE_TIME_BEFORE_SLEEP >= 2`
     是**同一件事的两种表达**：

        FreeRTOS：睡之前问「预期空闲 ≥ 2 tick 吗？」
        ChibiOS ：把所有超时都设成 ≥ 10 tick，等价于"睡就至少睡 10 个"
""")

    sub("④ ★ 但 tickless 在 ChibiOS 里是【受许可证门控】的")
    print("""  这是第 10 课已经挖过的一条，这里只把它接到本课的主题上。

  modules/ChibiOS/os/license/chrestrictions.h：

      /* Restrictions in basic mode.*/
      #if CH_LICENSE_FEATURES == CH_FEATURES_BASIC
      #undef CH_CFG_ST_TIMEDELTA
      #define CH_CFG_ST_TIMEDELTA                 0           /* ← tickless 禁用 */
      #undef CH_CFG_USE_TM
      #undef CH_CFG_USE_MUTEXES                                /* ★★ 互斥量禁用 */
      ...

  ★ 而档位由许可证决定（chlicense.h:586-612）：

      #if (CH_LICENSE == CH_LICENSE_GPL)             → CH_FEATURES_FULL
      #elif CH_LICENSE == CH_LICENSE_GPL_EXCEPTION   → CH_FEATURES_BASIC
      ...
      #define CH_LICENSE                          CH_LICENSE_GPL    ← ★ 默认

  ★★ 也就是说：

      **纯 GPL  → FULL → tickless 可用，但你的代码也得 GPL
        GPL+例外 → BASIC → tickless 被 #undef 掉，但代码可以闭源**

  ⇒ **「要不要 tickless」这件事，在某些许可证档位下不是技术选择，
    是法律选择。**

  ★ ArduPilot 用 `CH_LICENSE_GPL` ⇒ FULL ⇒ **tickless 可用**。
""")

    sub("⑤ ArduPilot 各平台的实际配置")
    print("""  实测（本机源码）：

  **ChibiOS 平台（主力，第 7~13 课）**

      默认（common/chconf.h:143）        CH_CFG_ST_TIMEDELTA = 10   ← 开

      显式改成 0（关）的，全部是外设板：
          HitecMosaic/hwdef{,-bl}.dat
          ZubaxGNSS/hwdef{,-bl}.dat
          f103-periph/hwdef{,-bl}.inc
          f303-MatekGPS/hwdef{,-bl}.dat
          f303-periph/hwdef{,-bl}.inc
          iomcu/hwdef.inc
          ⇒ 共 11 个文件

      显式改成 2 的，只有一个：
          iomcu-dshot/hwdef.inc:37           CH_CFG_ST_TIMEDELTA = 2

  **FreeRTOS 平台（ESP32）**

      grep `configUSE_TICKLESS_IDLE` 全库 → **零命中**
      ⇒ 走 FreeRTOS.h 的默认值 0 ⇒ **关**

  ★★ 读出来的结论有几层：

    ① **主力飞控板开着 tickless** —— 这和我最初的直觉相反
       （我原以为实时系统会关掉它）

    ② **"外设板关掉、主控板开着"** 这个分布值得想一下：
       外设板（GPS、CAN 节点、IO 协处理器）通常靠 **UART/CAN 中断**驱动，
       它们要么一直有数据、要么需要极低延迟响应，
       tickless 的"睡一觉再补偿"反而添麻烦

    ③ **iomcu-dshot 用了 2** —— 比默认的 10 更激进。
       那是给 DShot 电调做 IO 协处理的角色，对时序更敏感，
       所以把"最小安全超时"压到 2

    ④ **ESP32 平台没配 tickless** ——
       即使它跑的是 FreeRTOS（有这个功能），也没启用

  ★ 所以这一课的实用结论是：
    **"飞控该不该开 tickless"没有统一答案 ——
      主控开、外设关、IO 协处理用中间值。**
    判据不是"省不省电"，而是**"这个角色靠什么唤醒、对延迟多敏感"**。
""")

    sub("⑥ ★ 一句话总结本课")
    print("""  ┌──────────────────────────────────────────────────────────────┐
  │ tickless 的本质：                                             │
  │                                                              │
  │   把「周期性 tick 中断」换成「按需定时中断」，                │
  │   而"需"就是内核本来就维护着的 xNextTaskUnblockTime。         │
  │                                                              │
  │   它不引入新数据结构，也不改变调度语义 ——                     │
  │   只是**把没有意义的唤醒去掉**。                              │
  └──────────────────────────────────────────────────────────────┘

  ★ 代价集中在两处：

      补偿精度   —— 靠一个叫做 "a fiddle factor" 的经验常数 94
      能力边界   —— 内置版受 SysTick 24 位限制，只能睡几十毫秒

  ⇒ 想突破第二点，就要走模式 2（自定义实现）。
    这也是 ChibiOS 那种「一个配置项 + 值」的设计更灵活的地方：
    `CH_CFG_ST_TIMEDELTA` 换成别的时钟源，架构不用动。
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("tickless 省了什么：数一数 tick 中断", exp1),
    "2": ("prvGetExpectedIdleTime：四个「不能睡」的判断", exp2),
    "3": ("vTaskStepTick：补 tick 的那个「退一格」", exp3),
    "4": ("常量计算器 + ChibiOS 对照 + ArduPilot 的实际配置", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        args = sorted(EXPERIMENTS.keys())
    print()
    hr("#")
    print("#  第 8 课 · 低功耗与 Tickless idle —— 配套实验")
    hr("#")
    for a in args:
        if a not in EXPERIMENTS:
            print("  未知实验：%s（可选 %s）" % (a, "/".join(sorted(EXPERIMENTS))))
            return 1
        EXPERIMENTS[a][1]()
    print()
    hr("=")
    print("  完成。")
    hr("=")
    return 0


if __name__ == "__main__":
    sys.exit(main())
