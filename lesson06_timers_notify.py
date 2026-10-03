#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 6 课配套实验：软件定时器与任务通知

设计原则（延续三条纪律）：
  1. 每个论断都能落到 文件:行号
  2. [未亲验] 是诚实的标记，不是免责声明
  3. 模拟的地方明说丢掉了什么

运行：
    python lesson06_timers_notify.py         # 全部
    python lesson06_timers_notify.py 2       # 只跑实验 2

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
# 实验 1：软件定时器的异步命令模型
# ============================================================================
#
# timers.c:160-164 的注释：
#
#     * The timer service task (daemon).  Timer functionality is controlled by this
#     * task.  Other tasks communicate with the timer service task using the
#     * xTimerQueue queue.
#
# 所以 xTimerStart() 不是"启动一个定时器"，而是"往队列里发一条启动命令"。
# 这个区别带来一个容易踩的坑：**命令是异步的**。


class TimerQueueSim:
    """模拟 xTimerQueue + 守护任务。

    丢掉的东西：
      · 真实的链表操作（定时器链表、延时链表）
      · 真实的 tick 推进（这里用"推进 N 个 tick"的动作代替）
      · 阻塞与唤醒（这里用"队列满则失败"简化）
      · 回调的实际执行（这里只记录"回调被调用"）
    """

    def __init__(self, queue_length):
        self.qlen = queue_length
        self.queue = []                 # 命令队列
        self.timers = {}                # name -> {period, active, autoreload, remaining}
        self.callbacks = []             # 回调执行记录
        self.log = []

    def post_command(self, cmd, name, value=None):
        """对应 xTimerGenericCommandFromTask → xQueueSendToBack(xTimerQueue, ...)"""
        if len(self.queue) >= self.qlen:
            self.log.append("命令 %s(%s) **失败**：队列满（长度 %d）"
                            % (cmd, name, self.qlen))
            return False
        self.queue.append((cmd, name, value))
        self.log.append("命令 %s(%s) 已入队（队列深度 %d/%d）"
                        % (cmd, name, len(self.queue), self.qlen))
        return True

    def daemon_run(self):
        """对应 prvProcessReceivedCommands()：守护任务处理队列里的命令"""
        if not self.queue:
            self.log.append("守护任务：队列空，继续等")
            return
        while self.queue:
            cmd, name, value = self.queue.pop(0)
            if cmd == "start":
                t = self.timers.setdefault(
                    name, {"period": value, "active": False, "autoreload": True,
                           "remaining": 0})
                t["period"] = value
                t["active"] = True
                t["remaining"] = value
                self.log.append("守护任务：启动 %s（周期 %d tick）" % (name, value))
            elif cmd == "stop":
                t = self.timers.get(name)
                if t:
                    t["active"] = False
                    self.log.append("守护任务：停止 %s" % name)
            elif cmd == "reset":
                t = self.timers.get(name)
                if t:
                    t["remaining"] = t["period"]
                    self.log.append("守护任务：重置 %s" % name)

    def tick(self, n=1):
        """推进 tick，检查到期"""
        for _ in range(n):
            for name, t in self.timers.items():
                if not t["active"]:
                    continue
                t["remaining"] -= 1
                if t["remaining"] <= 0:
                    self.callbacks.append(name)
                    self.log.append("★ %s 到期 → 调用回调（在守护任务上下文里）"
                                    % name)
                    if t["autoreload"]:
                        t["remaining"] = t["period"]
                    else:
                        t["active"] = False


def exp1_timer_async():
    sec("实验 1：软件定时器的异步命令模型")

    print("""
  timers.c:160-164 的注释（原文）：

      * The timer service task (daemon).  Timer functionality is controlled by this
      * task.  Other tasks communicate with the timer service task using the
      * xTimerQueue queue.

  **⇒ xTimerStart() 不是"启动一个定时器"，而是"往队列里发一条启动命令"。**

  这个区别带来一个容易踩的坑：**命令是异步的。**
""")

    print("  " + "=" * 74)
    print("  场景 A：正常路径")
    print("  " + "=" * 74)
    print()
    s = TimerQueueSim(queue_length=5)
    print("  应用任务调用 xTimerStart(t1, 10)：")
    s.post_command("start", "t1", 10)
    for line in s.log:
        print("      %s" % line)
    print()
    print("  此刻定时器状态：%s（**还没被启动！**）"
          % ("空" if not s.timers else list(s.timers.keys())))
    print()
    s.log = []
    print("  守护任务被调度到，处理队列：")
    s.daemon_run()
    for line in s.log:
        print("      %s" % line)
    print()
    print("  ⇒ 现在 t1 才真的启动了。")

    print()
    print("  " + "=" * 74)
    print("  场景 B：异步带来的坑 —— Start 之后立刻 Stop")
    print("  " + "=" * 74)
    print()
    s2 = TimerQueueSim(queue_length=5)
    print("  应用任务连续调用 xTimerStart(t, 10) 和 xTimerStop(t)：")
    s2.post_command("start", "t", 10)
    s2.post_command("stop", "t")
    for line in s2.log:
        print("      %s" % line)
    print()
    print("  两条命令都在队列里排队（**守护任务还没跑**）")
    print()
    s2.log = []
    print("  守护任务处理：")
    s2.daemon_run()
    for line in s2.log:
        print("      %s" % line)
    print()
    print("  ⇒ 净效果是「停了」，但**中间经历了「启动→停止」**。")
    print("     如果这是你想避免的（比如启动会触发一些副作用），")
    print("     就不能依赖「两条命令互相抵消」。")

    print()
    print("  " + "=" * 74)
    print("  场景 C：队列满")
    print("  " + "=" * 74)
    print()
    s3 = TimerQueueSim(queue_length=2)
    print("  队列长度配置为 2，连续发 4 条命令：")
    for i in range(4):
        s3.post_command("start", "t%d" % i, 10)
    for line in s3.log:
        print("      %s" % line)
    print()
    print("""  ⇒ 第 3、4 条**失败**（xTimerStart 返回 pdFAIL）。

      **这是 configTIMER_QUEUE_LENGTH 配置太小的直接后果**，
      而且失败是"静默的"—— 除非你检查 xTimerStart 的返回值。

      ⚠ xTimerStart 有两个参数：
            xTimerStart(xTimer, xTicksToWait)
                                 ↑ 队列满时愿意等多久
        传 0 表示"不等，立刻返回失败"。传 portMAX_DELAY 会一直等
        （但**不能在 ISR 里用非 0 的值** —— 见第 4 课）。

  ================== 三个可以带走的点 ==================

  【1】定时器回调**永远在守护任务里跑**，不在调用者的上下文里

      由此推出四条约束：

          ❌ 回调里不能阻塞（会停掉【所有】定时器）
          ⚠ 所有回调共享一个栈（configTIMER_TASK_STACK_DEPTH）
          ⚠ 回调优先级 = 守护任务优先级（configTIMER_TASK_PRIORITY）
          ✅ 但回调里可以调大部分 FreeRTOS API（因为是任务上下文）

  【2】守护任务的栈是所有回调的【共享预算】

      如果你有 3 个回调，每个深度 200 字节，
      那 configTIMER_TASK_STACK_DEPTH 至少要能装下
      **最深的那一条调用链**（不是三个之和 —— 因为回调不嵌套）。

  【3】回调里正确的事：只做最小动作

          void my_timer_callback( TimerHandle_t xTimer )
          {
              xTaskNotifyGive(worker_task);   ✅ 只置标志 / 发通知
          }

      ⇒ 这和第 4 课 3.4 节（ISR 只做最小动作）、
        第 5 课 6.4 节（ISR 不分配内存）是同一个哲学：
        **共享的执行上下文，应该尽可能少做事。**
""")


# ============================================================================
# 实验 2：任务通知 vs 信号量 —— 事件链表的差别 ⭐
# ============================================================================
#
# tasks.c:7907-7913（xTaskGenericNotify 里）：
#
#     /* If the task is in the blocked state specifically to wait for a
#      * notification then unblock it now. */
#     if( ucOriginalNotifyState == taskWAITING_NOTIFICATION )
#     {
#         listREMOVE_ITEM( &( pxTCB->xStateListItem ) );
#         prvAddTaskToReadyList( pxTCB );
#
#         /* The task should not have been on an event list. */
#         configASSERT( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) == NULL );
#
# **最后那句断言是理解任务通知的钥匙。**


class TCB:
    """模拟 TCB 里和阻塞相关的字段（tasks.c:358-430）

    只保留本实验需要的：
        xEventListItem  —— 在不在事件链表里
        xStateListItem  —— 在不在延时链表里
        ulNotifiedValue —— 通知值
        ucNotifyState   —— 通知三态
    """

    def __init__(self, name, prio):
        self.name = name
        self.prio = prio
        self.in_event_list = None      # None 表示不在任何事件链表里
        self.in_delayed_list = False
        self.notified_value = 0
        self.notify_state = 0          # 0=NOT_WAITING 1=WAITING 2=RECEIVED

    def __repr__(self):
        return self.name


# 通知三态（tasks.c:111-114）
NOT_WAITING_NOTIFICATION = 0
WAITING_NOTIFICATION = 1
NOTIFICATION_RECEIVED = 2

STATE_NAME = {
    NOT_WAITING_NOTIFICATION: "NOT_WAITING (0)",
    WAITING_NOTIFICATION: "WAITING (1)",
    NOTIFICATION_RECEIVED: "RECEIVED (2)",
}


def block_on_queue(task, queue_name):
    """阻塞在队列上 —— 挂【两条】链表（第 3 课 3.1 节）"""
    task.in_event_list = queue_name
    task.in_delayed_list = True


def block_on_notification(task):
    """阻塞在通知上 —— 只挂【一条】链表"""
    task.in_event_list = None          # ★ 关键：不进事件链表
    task.in_delayed_list = True
    task.notify_state = WAITING_NOTIFICATION


def exp2_event_list_difference():
    sec("实验 2：任务通知 vs 信号量 —— 事件链表的差别 ⭐")

    print("""
  tasks.c:7907-7913（xTaskGenericNotify 里，原文）：

      /* If the task is in the blocked state specifically to wait for a
       * notification then unblock it now. */
      if( ucOriginalNotifyState == taskWAITING_NOTIFICATION )
      {
          listREMOVE_ITEM( &( pxTCB->xStateListItem ) );
          prvAddTaskToReadyList( pxTCB );

          /* The task should not have been on an event list. */
          configASSERT( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) == NULL );
          ...
      }

  **那句断言说：阻塞在通知上的任务，不应该在任何事件链表里。**

  对比第 3 课 3.2 节讲的"TCB 需要两个 ListItem_t" —— 那是为
  【队列/信号量】准备的。本实验把两者并排看。
""")

    print("  " + "=" * 74)
    print("  情形 1：任务阻塞在【队列】上")
    print("  " + "=" * 74)
    print()
    w = TCB("worker", 10)
    block_on_queue(w, "xTasksWaitingToReceive")
    print("      worker 调用 xQueueReceive(q, &item, 100)")
    print()
    print("      %-24s %s" % ("xEventListItem", w.in_event_list))
    print("      %-24s %s" % ("xStateListItem",
                              "在延时链表里" if w.in_delayed_list else "不在"))
    print()
    print("      ⇒ **两个 ListItem 都被占用**")
    print("        · xEventListItem → 队列的事件链表（按优先级排序）")
    print("        · xStateListItem → 延时链表（按到期时间排序）")
    print()
    print("      唤醒时：发送者要给【队列】发数据，内核执行")
    print("              listGET_OWNER_OF_HEAD_ENTRY(事件链表) 找出该唤醒谁")
    print("              （第 3 课 3.4 节）")

    print()
    print("  " + "=" * 74)
    print("  情形 2：任务阻塞在【任务通知】上")
    print("  " + "=" * 74)
    print()
    w2 = TCB("worker2", 10)
    block_on_notification(w2)
    print("      worker2 调用 ulTaskNotifyTake(pdTRUE, 100)")
    print()
    print("      %-24s %s" % ("xEventListItem",
                              "**空闲**（断言保证）" if w2.in_event_list is None
                              else w2.in_event_list))
    print("      %-24s %s" % ("xStateListItem",
                              "在延时链表里" if w2.in_delayed_list else "不在"))
    print("      %-24s %s" % ("ucNotifyState", STATE_NAME[w2.notify_state]))
    print()
    print("      ⇒ **只占用一个 ListItem**")
    print("        · xEventListItem → 空闲")
    print("        · xStateListItem → 延时链表")
    print("        · 通知值/状态直接在 TCB 里")
    print()
    print("      唤醒时：发送者调用 xTaskNotifyGive(worker2)")
    print("              —— **直接指名接收者**，不需要找")

    print()
    print("  " + "=" * 74)
    print("  ⇒ 这就是「任务通知快」的真正原因")
    print("  " + "=" * 74)
    print("""
      ┌──────────────────────────────────────────────────────────────┐
      │  队列 / 信号量：给的是【对象】，内核负责找等待者             │
      │                                                              │
      │      xSemaphoreGive(sem);                                    │
      │                      ↑                                       │
      │            内核要：                                           │
      │              ① 阻塞时 vListInsert 到事件链表（按优先级排序） │
      │              ② 唤醒时 listGET_OWNER_OF_HEAD_ENTRY 取链表头    │
      │      ⇒ 成本 = 一次有序插入 + 一次链表头操作                   │
      ├──────────────────────────────────────────────────────────────┤
      │  任务通知：给的是【任务】，内核不需要找                       │
      │                                                              │
      │      xTaskNotifyGive(worker_task);                           │
      │                      ↑                                       │
      │            内核要：                                           │
      │              ① 改 pxTCB->ulNotifiedValue  （一次赋值）        │
      │              ② 改 pxTCB->ucNotifyState    （一次赋值）        │
      │              ③ 从延时链表摘除 + 加入就绪队列                  │
      │      ⇒ 成本 = 两次赋值 + 常规的解除阻塞                       │
      └──────────────────────────────────────────────────────────────┘

      **省掉的是"插入/查找排序链表"这条完整路径，不是几条指令。**

      这也解释了为什么任务通知有【独占性限制】（2.4 节）——
      因为"发送者直接指名接收者"这个设计，
      天然不支持"多个等待者竞争同一个对象"。

      ⚠ 关于"快 45%"这个数字：那是 FreeRTOS 官方 benchmark 的结果，
        **源码里没有，需要在目标上实测**。本课标 [未亲验]。
        但"为什么快"这个机制是上面这条 —— 源码可验证。
""")

    # 演示三态机
    print("  " + "=" * 74)
    print("  附：通知的三态机（tasks.c:111-114）")
    print("  " + "=" * 74)
    print()
    print("      #define taskNOT_WAITING_NOTIFICATION    ( ( uint8_t ) 0 )")
    print("      #define taskWAITING_NOTIFICATION        ( ( uint8_t ) 1 )")
    print("      #define taskNOTIFICATION_RECEIVED       ( ( uint8_t ) 2 )")
    print()
    print("  三态之间的转移：")
    print()
    print("      ┌────────────────────────┐")
    print("      │ NOT_WAITING (0)        │ ← TCB 初始化时的值")
    print("      │ 没在等，也没有待处理通知 │")
    print("      └───┬────────────────┬───┘")
    print("          │ 任务调用        │ 收到通知")
    print("          │ NotifyTake/Wait │")
    print("          ↓                 ↓")
    print("      ┌────────────────┐  ┌────────────────────┐")
    print("      │ WAITING (1)    │  │ RECEIVED (2)       │")
    print("      │ 阻塞中         │→ │ 值已写入，还没取走 │")
    print("      └────────────────┘  └────────┬───────────┘")
    print("               ↑ 收到通知            │ 任务取走值")
    print("               └─────────────────────┘")
    print()
    print("""  **第三态（RECEIVED）是解决"通知比等待早到"的关键** ——
  相当于信号量的"计数"。

  但它**只有一个槽位**（`configTASK_NOTIFICATION_ARRAY_ENTRIES` 默认 1），
  所以"再来一条通知"只能覆盖或失败：

      tasks.c:7873-7885（eSetValueWithoutOverwrite）
          if( ucOriginalNotifyState != taskNOTIFICATION_RECEIVED )
          {
              pxTCB->ulNotifiedValue[ uxIndexToNotify ] = ulValue;
          }
          else
          {
              /* The value could not be written to the task. */
              xReturn = pdFAIL;
          }

  ⇒ **这就是独占性限制的源码依据。**
""")


# ============================================================================
# 实验 3：通知的五种动作 —— 一个机制扮演五种角色
# ============================================================================

def exp3_notify_actions():
    sec("实验 3：通知的五种动作 —— 一个机制扮演五种角色")

    print("""
  xTaskGenericNotify 的"动作"参数有五个取值，让"任务通知"能扮演
  五种不同的同步原语。本实验做对照。
""")

    actions = [
        ("eIncrement",
         "ulNotifiedValue += 1",
         "**计数信号量**（配 ulTaskNotifyTake(pdTRUE, ...)）",
         "give 一次加一，take 时清零"),
        ("eSetBits",
         "ulNotifiedValue |= ulValue",
         "**事件组**（32 位位掩码）",
         "可以同时置多个位，接收方检查想要的位"),
        ("eSetValueWithOverwrite",
         "ulNotifiedValue = ulValue",
         "**长度 1 的队列**（覆盖式）",
         "总是写入，旧值丢失"),
        ("eSetValueWithoutOverwrite",
         "if state != RECEIVED: ulNotifiedValue = ulValue",
         "**长度 1 的队列**（不覆盖）",
         "旧值未取走时返回 pdFAIL"),
        ("eNoAction",
         "（不改值）",
         "**二值信号量**（只唤醒）",
         "常用于「只关心有没有事件」，不关心值"),
    ]

    print("  " + "=" * 84)
    print("  五种动作对照")
    print("  " + "=" * 84)
    print()
    for i, (name, effect, equiv, note) in enumerate(actions, 1):
        print("  【%d】%s" % (i, name))
        print("       效果:   %s" % effect)
        print("       等价于: %s" % equiv)
        print("       说明:   %s" % note)
        print()

    # 演示 eSetBits 和 eIncrement 的区别
    print("  " + "=" * 84)
    print("  演示：eIncrement vs eSetBits")
    print("  " + "=" * 84)
    print()
    print("  假设 worker 任务同时关心两件事：'串口有数据' 和 '定时器到期'")
    print()

    print("  " + "-" * 74)
    print("  用 eIncrement（计数语义）")
    print("  " + "-" * 74)
    print()
    val = 0
    print("      初始 ulNotifiedValue = %d" % val)
    val += 1
    print("      串口 ISR: xTaskNotifyGive(worker, eIncrement)")
    print("          ⇒ ulNotifiedValue = %d" % val)
    val += 1
    print("      定时器回调: xTaskNotifyGive(worker, eIncrement)")
    print("          ⇒ ulNotifiedValue = %d" % val)
    print()
    print("      任务取走：ulTaskNotifyTake(pdTRUE, ...) → 返回 %d，然后清零" % val)
    print()
    print("      ⚠ **任务无法知道是【谁】通知的** —— 只知道「有 2 个事件」")
    print("        而且两个事件被合并成了一个计数")

    print()
    print("  " + "-" * 74)
    print("  用 eSetBits（位掩码语义）")
    print("  " + "-" * 74)
    print()
    val = 0
    UART_BIT, TIMER_BIT = 0x01, 0x02
    print("      初始 ulNotifiedValue = 0x%02X" % val)
    val |= UART_BIT
    print("      串口 ISR: xTaskNotify(worker, 0x%02X, eSetBits)" % UART_BIT)
    print("          ⇒ ulNotifiedValue = 0x%02X（bit0 = 串口）" % val)
    val |= TIMER_BIT
    print("      定时器回调: xTaskNotify(worker, 0x%02X, eSetBits)" % TIMER_BIT)
    print("          ⇒ ulNotifiedValue = 0x%02X（bit0 + bit1）" % val)
    print()
    print("      任务读：xTaskNotifyWait(0, 0xFFFFFFFF, &val, ...)")
    print("          ⇒ 拿到 0x%02X，可以判断 **哪几个** 事件发生了" % val)
    print()
    print("      ✅ 两个事件被区分开了")

    print("""
  ================== 这正是「事件组」的能力 ==================

      第 10 课 3.1 节讲过：NuttX 没有事件组，FreeRTOS 有。
      而 FreeRTOS 的"事件组"在【单接收者】场景下可以用任务通知模拟
      —— 就是 eSetBits。

      但两者有区别：

          |  | 事件组 (xEventGroup) | 任务通知 eSetBits |
          |---|---|---|
          | 等待者数量 | 任意多个 | **1 个** |
          | 等待"任意位" | xEventGroupWaitBits 支持 | 需要自己循环检查 |
          | 等待"所有位" | 支持 | 需要自己循环检查 |
          | 开销 | 较高（独立对象 + 事件链表） | 极低 |

      ⇒ **独占性限制在这里再次出现。**

  ================== 一个可以带走的判断 ==================

      **任务通知的"一个机制五种角色"看起来很美，但别忘了：
      它只有【一个槽位】、只有【一个等待者】。**

      什么时候该用任务通知：

          ✅ ISR → 一个特定任务（点对点）
          ✅ 只有一个等待者的信号量场景
          ✅ 需要携带一个 32 位值

      什么时候不该用：

          ❌ 多个任务等同一个事件
          ❌ 需要"计数 > 1"的语义（虽然有 eIncrement）
          ❌ 需要"等待者按优先级排队"
          ✅ **代码可读性优先时** —— 信号量比通知好读得多

      ⇒ 而最后一条往往被忽略：**性能优势通常不值得牺牲可读性。**
        在 100 MHz 以上的 MCU 上，两者的绝对差值是几百纳秒。
""")


# ============================================================================
# 实验 4：ucStatus 的三标志位（第 5 次位打包）
# ============================================================================

def exp4_status_bits():
    sec("实验 4：ucStatus 的三标志位 —— 第 5 次位打包")

    print("""
  timers.c:78-80（原文）：

      #define tmrSTATUS_IS_ACTIVE                  ( 0x01U )
      #define tmrSTATUS_IS_STATICALLY_ALLOCATED    ( 0x02U )
      #define tmrSTATUS_IS_AUTORELOAD              ( 0x04U )

  **一个 uint8_t 里塞了三个标志。**

  而结构体里的注释（原文）：

      uint8_t ucStatus;   /**< Holds bits to say if the timer was statically
                               allocated or not, and if it is active or not. */
""")

    IS_ACTIVE = 0x01
    IS_STATIC = 0x02
    IS_AUTORELOAD = 0x04

    def show(status):
        print("      0x%02X = %s" % (status, format(status, "08b")))
        print("             %-34s %s"
              % ("激活 (IS_ACTIVE)",
                 "✓" if status & IS_ACTIVE else "✗"))
        print("             %-34s %s"
              % ("静态分配 (IS_STATICALLY_ALLOCATED)",
                 "✓" if status & IS_STATIC else "✗"))
        print("             %-34s %s"
              % ("自动重载 (IS_AUTORELOAD)",
                 "✓" if status & IS_AUTORELOAD else "✗"))

    print("  " + "=" * 74)
    print("  四种组合")
    print("  " + "=" * 74)
    print()
    for label, st in [
        ("刚刚创建（什么都没设）", 0x00),
        ("静态分配的自动重载定时器，已启动",
         IS_STATIC | IS_AUTORELOAD | IS_ACTIVE),
        ("动态分配的单次定时器，已启动",
         IS_ACTIVE),
        ("动态分配的自动重载定时器，已停止（暂停）",
         IS_AUTORELOAD),
    ]:
        print("  【%s】" % label)
        show(st)
        print()

    print("""  ================== 和前四次位打包的区别 ==================

      前四次（第 1、3、5 课）：

          ① NuttX:        int16_t semcount 的【符号位】
          ② FreeRTOS:     pcHead 的 NULL（表示"是互斥量"）
          ③ FreeRTOS:     Queue_t 里的 union（复用 8 字节）
          ④ FreeRTOS:     xBlockSize 的【最高位】

      ⇒ 四次都是**借用别的用途的字段**

      这一次（⑤）：

          timers.c:       ucStatus 这个【专门的标志字节】

      ⇒ **不是借用，而是"用一个字节装三个标志"**

  ================== 收益与代价 ==================

      【收益】省 3 个字节（如果三个标志各占一个 uint8_t）

          Timer_t 在 32 位平台上大约 32 字节
          省 3 字节 ≈ 10%

          假设 20 个定时器：省 60 字节。**比前四次的收益小。**

      【代价】和前四次一样：读代码要查那三个宏

          if( ( xTimer->ucStatus & tmrSTATUS_IS_ACTIVE ) != 0 )    ← 难读
          if( xTimer->xIsActive )                                  ← 好读

      ⇒ **所以这一次的取舍更勉强** —— 收益更小，代价一样。

        但注意：FreeRTOS 用 uint8_t 而不是三个独立的 uint8_t，
        可能不只是为了省 3 字节，而是**让结构体布局更紧凑**
        （三个 uint8_t 会被对齐填充成 3 字节，效果一样；
         但一个 uint8_t 明确表达了"这些标志是一组"）。

      ⇒ 这更像是一个**表达意图**的选择，而不是纯粹的省内存。
        这是它和前四次的另一个区别。
""")


# ============================================================================
# 主入口
# ============================================================================

EXPERIMENTS = {
    "1": ("软件定时器的异步命令模型", exp1_timer_async),
    "2": ("通知 vs 信号量的事件链表差别", exp2_event_list_difference),
    "3": ("通知的五种动作", exp3_notify_actions),
    "4": ("ucStatus 的三标志位", exp4_status_bits),
}


def main():
    print("""
================================================================================
 第 6 课：软件定时器与任务通知
 配套实验代码（FreeRTOS 部分的最后一课）

 实验列表：
   1  软件定时器的异步命令模型
   2  通知 vs 信号量的事件链表差别 ★
   3  通知的五种动作 —— 一个机制扮演五种角色
   4  ucStatus 的三标志位 —— 第 5 次位打包
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

1. **软件定时器的架构是「守护任务 + 队列」，回调在守护任务里跑。**
   timers.c:160-164 的注释一句话说清了：
   "Other tasks communicate with the timer service task using the
    xTimerQueue queue."
   ⇒ 由此推出最重要的约束：**回调里不能阻塞**（否则所有定时器停摆），
     而且所有回调共享一个栈（configTIMER_TASK_STACK_DEPTH）。

2. **任务通知快的真正原因，是它不走事件链表。**
   xTaskGenericNotify 里那句 configASSERT 揭示了一切：
       /* The task should not have been on an event list. */
       configASSERT( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) == NULL );
   对比第 3 课：队列/信号量需要两个 ListItem_t，通知只需要一个。
   因为通知是【点对点】的 —— 发送者直接指名接收者。
   ⇒ 省掉的是"插入/查找排序链表"这条完整路径。

3. **一个字节能装三个标志（ucStatus），但这是第 5 次位打包中
   收益最小的一次。**
   前四次都是"借用别的用途的字段"，这次是"用一个字节装三个标志"。
   ⇒ 收益更小（省 3 字节），代价一样（读代码要查宏）。
     所以这次更像是一个【表达意图】的选择。

 FreeRTOS 部分（第 1~6 课）到此结束。六课的总结见文档第 4.2 节：
   · 它极度重视省内存       → 代价是读代码门槛高
   · 它把"不应该发生"变成立刻停机 → 代价是依赖 configASSERT
   · 它的复杂度是为"覆盖所有平台"付的 → 代价是在特定平台上不是最优
================================================================================
""")


if __name__ == "__main__":
    main()
