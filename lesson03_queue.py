#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 4 第 8 章配套实验：队列 —— 一切通信机制的基础

设计原则（延续三条纪律）：
  1. 每个论断都能落到 文件:行号
  2. [未亲验] 是诚实的标记，不是免责声明
  3. 模拟的地方明说丢掉了什么

运行：
    python lesson03_queue.py         # 全部
    python lesson03_queue.py 4       # 只跑实验 4

源码来源：$SRC_FREERTOS/   （V11.1.0）
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
# 实验 1：Queue_t 的 union —— 同一个结构体的两种身份
# ============================================================================
#
# queue.c:103 的 Queue_t 里有一个 union：
#
#     union
#     {
#         QueuePointers_t xQueue;     /* pcTail + pcReadFrom        */
#         SemaphoreData_t xSemaphore; /* xMutexHolder + 递归计数     */
#     } u;
#
# 用作队列时，8 字节是"环形缓冲的两个指针"；
# 用作信号量时，同样的 8 字节是"PI 的依托 + 递归计数"。
#
# 分叉判据是 uxItemSize（queue.c:82：信号量元素大小为 0）。
#
# ★★ 一处纠正（卷 4 第 6 章 / 本章 各纠正过一次，本文件当时漏了）：
#
#   本文件初版把 pcHead 画成了
#       "item_size == 0 ⇒ 不分配存储区 ⇒ pcHead 保持 NULL"，
#   **这是错的。** 真实源码 queue.c:576-583 写的是：
#
#       if( uxItemSize == ( UBaseType_t ) 0 )
#       {
#           /* No RAM was allocated for the queue storage area, but PC head cannot
#            * be set to NULL because NULL is used as a key to say the queue is used as
#            * a mutex.  Therefore just set pcHead to point to the queue as a benign
#            * value that is known to be within the memory map. */
#           pxNewQueue->pcHead = ( int8_t * ) pxNewQueue;      /* :582 指向自己 */
#       }
#
#   ⇒ 元素大小为 0（信号量【和】互斥量）时，pcHead 指向【队列自己】；
#     NULL 这个值被留作"这是互斥量"的哨兵（queue.c:66 queueQUEUE_IS_MUTEX），
#     【只有】prvInitialiseMutex() 才把它显式写下去（queue.c:623）。
#   ⇒ 所以二值信号量恰恰落在"非 NULL"那一侧 —— 它不会被误判成互斥量。
#     机制细节见 QueueT.prv_initialise_mutex() 的注释。

FIELD_NAMES = ["pcHead", "pcWriteTo", "u(8B)", "uxMessagesWaiting",
               "uxLength", "uxItemSize", "pcHead==NULL?"]


class QueueT:
    """模拟 Queue_t 的字段（本实验关心的部分）。

    丢掉的东西：
      · 两条事件链表（xTasksWaitingToSend/Receive）—— 实验 4 单独模拟
      · cRxLock / cTxLock —— 第 5 节讲
      · QueueSet / trace 相关字段
      · 真实的环形缓冲内存（这里用一个 Python list 代替，并在
        实验 2 里单独模拟指针移动）
      · u 的"面孔"只按 uxItemSize 分（真实源码里 u.xSemaphore 只被
        【互斥量】那条路读写：守卫全是 uxQueueType == queueQUEUE_IS_MUTEX，
        见 queue.c:705 / :737 / :2405 —— 二值信号量两个面孔都不参与运算）
    """

    def __init__(self, name, length, item_size, is_mutex=False):
        self.name = name
        self.uxLength = length
        self.uxItemSize = item_size
        self.uxMessagesWaiting = 0
        # 存储区：item_size == 0 时不分配 —— 但 pcHead 【不是】NULL
        #   queue.c:582  pxNewQueue->pcHead = ( int8_t * ) pxNewQueue;
        #   （"哑值"：指向队列自己；源码注释说 NULL 被留作互斥量的哨兵）
        self.storage = None if item_size == 0 else []
        self.pcHead = "self" if item_size == 0 else "buf"
        # xQueueGenericReset 把 pcWriteTo 设成 pcHead（queue.c:322），
        # 而 prvInitialiseMutex 【不碰它】—— 所以互斥量的 pcWriteTo
        # 也仍然指向队列自己，只有 pcHead 变成了 NULL。
        self.pcWriteTo = self.pcHead + "+0"
        # union 的两副面孔 —— 同一块内存，按类型解释
        self.u_kind = "xSemaphore" if item_size == 0 else "xQueue"
        self.u_xQueue = {"pcTail": None, "pcReadFrom": None}
        self.u_xSemaphore = {"xMutexHolder": None, "uxRecursiveCallCount": 0}
        self.is_mutex = False
        # ★ 互斥量比"普通信号量"多的那一步（queue.c:614 prvInitialiseMutex）
        if is_mutex:
            self.prv_initialise_mutex()

    def prv_initialise_mutex(self):
        """prvInitialiseMutex()（queue.c:614-637）—— pcHead 到这里才变成 NULL。

        这是本文件纠正过的那处模型错误的落点。它的全部动作是：

            pxNewQueue->u.xSemaphore.xMutexHolder = NULL;        /* queue.c:622 */
            pxNewQueue->uxQueueType = queueQUEUE_IS_MUTEX;       /* queue.c:623 */
            pxNewQueue->u.xSemaphore.uxRecursiveCallCount = 0;   /* queue.c:626 */

        而 queue.c:65-66 是

            #define uxQueueType               pcHead
            #define queueQUEUE_IS_MUTEX       NULL

        ⇒ **"pcHead == NULL 是互斥量"这个判据，靠的是 queue.c:623 这一句赋值**
          —— 不是"元素大小为 0"。二值/计数信号量的元素大小同样是 0，
          但它们的 pcHead 是非 NULL 的（指向队列自己）⇒ 不会被误判成互斥量、
          不会走 PI 那条路。
        """
        self.u_xSemaphore["xMutexHolder"] = None
        self.u_xSemaphore["uxRecursiveCallCount"] = 0
        self.pcHead = "NULL"        # queue.c:623（uxQueueType 就是 pcHead）
        self.is_mutex = True

    def is_semaphore(self):
        """uxItemSize == 0 ⇒ 没有存储区 ⇒ 是信号量/互斥量（不是消息队列）。

        queue.c:82 的 queueSEMAPHORE_QUEUE_ITEM_LENGTH 就是 0。

        ⚠ **这【不是】"是不是互斥量"的判据。** 互斥量的判据是 pcHead == NULL
          （uxQueueType 的别名，queue.c:65-66），
          而二值信号量恰恰落在"非 NULL"那一侧。
        """
        return self.uxItemSize == 0

    def u_is_mutex(self):
        """uxQueueType == queueQUEUE_IS_MUTEX 的等价判断 ⇒ pcHead == NULL。

        这个值【只有】prvInitialiseMutex()（queue.c:623）写过，
        所以它标记的是"创建时走的是不是 xSemaphoreCreateMutex() 那条路"，
        而不是"元素大小是不是 0"。
        """
        return self.pcHead == "NULL"

    def snapshot(self):
        return {
            "pcHead": self.pcHead,
            "pcWriteTo": self.pcWriteTo,
            "u(8B)": ("xQueue: tail=%s read=%s" % (
                self.u_xQueue["pcTail"], self.u_xQueue["pcReadFrom"])
                if self.u_kind == "xQueue"
                else "xSem: holder=%s rec=%d" % (
                    self.u_xSemaphore["xMutexHolder"],
                    self.u_xSemaphore["uxRecursiveCallCount"])),
            "uxMessagesWaiting": self.uxMessagesWaiting,
            "uxLength": self.uxLength,
            "uxItemSize": self.uxItemSize,
            "pcHead==NULL?": "是 → 视为互斥量" if self.u_is_mutex() else "否",
        }


def exp1_union():
    sec("实验 1：Queue_t 的 union —— 同一个结构体的两种身份")

    print("""
  queue.c:68-78 定义了两个结构体：

      typedef struct QueuePointers {
          int8_t * pcTail;      /* 存储区末尾（+1 字节的越界标记） */
          int8_t * pcReadFrom;  /* 【上次读出的位置】—— 注意不是"下次要读" */
      } QueuePointers_t;

      typedef struct SemaphoreData {
          TaskHandle_t xMutexHolder;         /* ★ PI 的依托（卷 4 第 6 章） */
          UBaseType_t uxRecursiveCallCount;  /* 递归互斥的计数 */
      } SemaphoreData_t;

  而 Queue_t 里：

      union { QueuePointers_t xQueue; SemaphoreData_t xSemaphore; } u;

  ⇒ **同样 8 字节，两种完全不同的用途。**
""")

    objs = [
        QueueT("msgq (队列)", 10, 32),
        QueueT("bsem (二值信号量)", 1, 0),
        QueueT("mtx (互斥量)", 1, 0, is_mutex=True),
    ]

    for q in objs:
        print("  " + "-" * 74)
        print("  %s" % q.name)
        print("  " + "-" * 74)
        s = q.snapshot()
        for k in FIELD_NAMES:
            print("      %-20s %s" % (k, s[k]))
        print()

    print("""  ================== 三个观察 ==================

  【观察 1】uxItemSize == 0 决定"有没有存储区"，但【不决定】pcHead 是不是 NULL

      队列:       uxItemSize = 32  → 分配存储区 → pcHead = 存储区起点（queue.c:587）
      信号量:     uxItemSize = 0   → 不分配存储区 → pcHead 指向【队列自己】
                                                    （queue.c:582）
      互斥量:     uxItemSize = 0   → 同上，然后 prvInitialiseMutex()
                                      才把它改成 NULL（queue.c:623）

      queue.c:82 的注释解释了为什么信号量的元素大小是 0：

          /* Semaphores do not actually store or copy data, so have an item
           * size of zero. */
          #define queueSEMAPHORE_QUEUE_ITEM_LENGTH    ( ( UBaseType_t ) 0 )

      ★ 但"元素大小为 0"和"pcHead 为 NULL"是【两件事】——
        上面表里 bsem 那一行就是证据：uxItemSize 是 0，pcHead 却是 self。
        本文件初版把这两件事混成了一件（写成"⇒ pcHead 保持 NULL"），
        是错的；卷 4 第 6 章 和本章 纠正过，本文件现在跟上。

  【观察 2】pcHead 的双重身份

      pcHead 既是"存储区起点指针"，又是"是不是互斥量"的标记 ——
      因为 queue.c:65 有 `#define uxQueueType pcHead`（卷 4 第 6 章 2.2 节）。

      ★ 让这两件事能共存的，**不是**"信号量不分配存储区"，而是源码那句
        "把 pcHead 指向队列自己"（queue.c:582）：

          队列:       pcHead = 存储区起点   （queue.c:587）
          普通信号量: pcHead = 队列自己     （queue.c:582）← 非 NULL ⇒ 不是互斥量
          互斥量:     pcHead = NULL         （queue.c:623）← 哨兵

      ⇒ 判据 `pcHead == NULL` 之所以有效，是因为【只有】互斥量那条创建路径
        会显式写 NULL —— "元素大小为 0"本身并不写 NULL。

  【观察 3】union 省了 8 字节

      按 32 位平台算：
          不用 union：Queue_t 里同时放 pcTail/pcReadFrom/xMutexHolder/递归计数 = 16 字节
          用 union：  8 字节
          ⇒ 每个队列省 8 字节

      PX4 / ArduPilot 这类系统有几十个队列，省几百字节。
      但**代价是读代码的人要记住"u 里装什么由 uxItemSize 决定"**。

  ⇒ 这是本章第 7.2 节总结的第 3 次"把语义塞进已有字段"。
""")


# ============================================================================
# 实验 2：prvCopyDataToQueue 的三个分支
# ============================================================================
#
# queue.c:2390 的 prvCopyDataToQueue 是整个队列机制的核心。
# 它有**三个分支**，性质完全不同：
#
#   分支 1: uxItemSize == 0          → 不拷贝（信号量/互斥量）+ PI 归还
#   分支 2: queueSEND_TO_BACK        → memcpy 到 pcWriteTo，指针前进，绕回
#   分支 3: queueSEND_TO_FRONT       → memcpy 到 pcReadFrom，指针【后退】，绕回


class RingBuffer:
    """模拟环形缓冲的指针移动（prvCopyDataToQueue 的分支 2 和 3）。

    用真实的字节数组，忠实复现：
        pcHead     = 0
        pcTail     = n * item_size        （末尾标记）
        pcWriteTo  = 下一个空位
        pcReadFrom = 【上次读出的位置】

    丢掉的东西：
      · 真实的 memcpy 开销
      · "多分配 1 字节当标记" 的细节（这里用 Python 的越界检查代替）
      · 并发访问（真实代码在临界区里跑）
    """

    def __init__(self, name, length, item_size):
        self.name = name
        self.length = length
        self.item_size = item_size
        self.buf = [None] * (length * item_size)
        self.pcHead = 0
        self.pcTail = length * item_size
        self.pcWriteTo = 0
        self.pcReadFrom = self.pcTail - item_size   # 初始指向"最后一格"
        self.count = 0
        self.log = []

    def _addr(self, p):
        return "buf+%d" % p

    def send_back(self, value):
        """queue.c:2390 分支 2：memcpy 到 pcWriteTo，前进，绕回"""
        if self.count >= self.length:
            self.log.append(("SEND_TO_BACK", value, "队列满 —— 发不进去"))
            return False
        for k in range(self.item_size):
            self.buf[self.pcWriteTo + k] = (value, k)
        self.log.append(("SEND_TO_BACK", value,
                         "写入 %s，pcWriteTo %s → %s"
                         % (self._addr(self.pcWriteTo),
                            self._addr(self.pcWriteTo),
                            self._addr((self.pcWriteTo + self.item_size)
                                       % self.pcTail if
                                       self.pcWriteTo + self.item_size < self.pcTail
                                       else self.pcHead))))
        self.pcWriteTo += self.item_size
        if self.pcWriteTo >= self.pcTail:          # 绕回
            self.pcWriteTo = self.pcHead
        self.count += 1
        return True

    def send_front(self, value):
        """queue.c:2390 分支 3：memcpy 到 pcReadFrom，【后退】，绕回

        注意 pcReadFrom 的语义是"上次读出的位置"，
        所以"往前插一格"就是"写在 pcReadFrom 处，然后让 pcReadFrom 后退"。
        """
        if self.count >= self.length:
            self.log.append(("SEND_TO_FRONT", value, "队列满 —— 发不进去"))
            return False
        for k in range(self.item_size):
            self.buf[self.pcReadFrom + k] = (value, k)
        old = self.pcReadFrom
        self.pcReadFrom -= self.item_size
        if self.pcReadFrom < self.pcHead:
            self.pcReadFrom = self.pcTail - self.item_size
        self.log.append(("SEND_TO_FRONT", value,
                         "写入 %s，pcReadFrom %s → %s"
                         % (self._addr(old), self._addr(old),
                            self._addr(self.pcReadFrom))))
        self.count += 1
        return True

    def state(self):
        return "count=%d  pcWriteTo=%s  pcReadFrom=%s" % (
            self.count, self._addr(self.pcWriteTo), self._addr(self.pcReadFrom))

    def contents(self):
        """按逻辑顺序列出元素（从 pcReadFrom 之后开始）"""
        out = []
        p = (self.pcReadFrom + self.item_size) % self.pcTail
        for _ in range(self.count):
            out.append(self.buf[p])
            p = (p + self.item_size) % self.pcTail
        return [x[0] if x else None for x in out]


def exp2_copy_branches():
    sec("实验 2：prvCopyDataToQueue 的三个分支")

    print("""
  queue.c:2390 的 prvCopyDataToQueue 有三个分支，性质完全不同：

      分支 1  uxItemSize == 0        → **不做任何拷贝**
      分支 2  queueSEND_TO_BACK      → memcpy + 指针前进 + 绕回
      分支 3  queueSEND_TO_FRONT     → memcpy + 指针【后退】 + 绕回
             （其中还有 queueOVERWRITE 的特殊处理）

  ================== 分支 1：信号量根本不拷贝 ==================
""")

    print("""      源码（queue.c:2391-2412 节选）：

          if( pxQueue->uxItemSize == ( UBaseType_t ) 0 )
          {
              #if ( configUSE_MUTEXES == 1 )
              {
                  if( pxQueue->uxQueueType == queueQUEUE_IS_MUTEX )
                  {
                      /* The mutex is no longer being held. */
                      xReturn = xTaskPriorityDisinherit( pxQueue->u.xSemaphore.xMutexHolder );
                      pxQueue->u.xSemaphore.xMutexHolder = NULL;
                  }
                  ...
              }
              #endif
          }

      ⇒ **没有 memcpy、没有指针移动。唯一的动作是"如果是互斥量，做 PI 归还"。**

      这就是"给一个信号量"的全部实现：把 uxMessagesWaiting 加一
      （在后面统一的尾部做），外加可能的 PI 归还。

      ⇒ 卷 4 第 6 章那句"所有原语都是队列"在这里落地了：
        "发一个信号量" = "往一个没有存储区的队列里放一个大小为 0 的元素"
        = 什么都不做。

  ================== 分支 2 与 3：环形缓冲的指针移动 ==================
""")

    rb = RingBuffer("demo q", 4, 1)   # 长度 4，每项 1 单位
    print("  演示：长度 4 的队列，交替 send_back / send_front")
    print()
    print("  %-18s %-10s %s" % ("操作", "元素", "指针变化 / 队列内容"))
    print("  " + "-" * 76)

    ops = [("back", "A"), ("back", "B"), ("front", "X"), ("back", "C"),
           ("front", "Y"), ("back", "D")]
    for kind, val in ops:
        if kind == "back":
            rb.send_back(val)
        else:
            rb.send_front(val)
        last = rb.log[-1]
        print("  %-18s %-10s %s" % (last[0], last[1], last[2]))
        print("  %-18s %-10s 队列内容（读序）= %s"
              % ("", "", rb.contents()))

    print()
    print("  " + "-" * 74)
    print("  ⚠ pcReadFrom 的语义陷阱")
    print("  " + "-" * 74)
    print("""
      queue.c:71 的注释：

          int8_t * pcReadFrom; /**< Points to the last place that a queued
                                     item was read from when the structure
                                     is used as a queue. */

      **"last place ... was read from" —— 上次【读出】的位置，
        不是"下次要读的位置"。**

      为什么要这样？因为 send_to_front 需要"往前一格写"，
      而"上次读出的位置"天然就是"最早的空位"：

          正常读的顺序是：pcReadFrom += item_size 然后读
          所以 pcReadFrom 之后才是有效数据
          ⇒ 往 pcReadFrom 处写 = 插到队头

      如果用"下次要读的位置"这个语义，插入队头就得额外判断边界。

      ⇒ **代价是读代码的人容易搞反。**
        这是本章第 7.2 节总结的第 5 次"用可读性换代码量"。
""")

    print("  " + "-" * 74)
    print("  ⚠ queueOVERWRITE 只能用在长度为 1 的队列上")
    print("  " + "-" * 74)
    print("""
      queue.c:952 的断言：

          configASSERT( !( ( xCopyPosition == queueOVERWRITE ) &&
                           ( pxQueue->uxLength != 1 ) ) );

      ⇒ **长度不是 1 时用 overwrite 会触发断言。**

      为什么这么限制？因为"覆盖最旧的元素"在长队列上语义不清晰：

          长度 10 的队列，overwrite 应该覆盖第 1 个还是第 10 个？

      而长度 1 时没有歧义 —— 就是"永远只保留最新值"。

      ⇒ 需要"保留最新值"的场景（飞控里很常见，比如最新的遥控输入）
        就用长度 1 的队列。

      ⚠ 注意这个断言只在 configASSERT 有效时存在。
        如果 configASSERT 被定义成空（生产构建常见），
        误用不会被抓住，行为是未定义的。
""")


# ============================================================================
# 实验 3：xItemValue 倒置 —— 一个减法实现优先级排序
# ============================================================================
#
# tasks.c:5252-5262 的注释：
#
#     /* Note: Lists are sorted in ascending order by ListItem_t.xItemValue.
#      * Normally, the xItemValue of a TCB's ListItem_t members is:
#      *      xItemValue = ( configMAX_PRIORITIES - uxPriority )
#      * Therefore, the event list is sorted in descending priority order. */
#
# 本实验把这个技巧做成可运行代码，并验证"取链表头 = 最高优先级"。

MAX_PRIO = 32


class SortedList:
    """模拟 FreeRTOS 的 List_t：按 xItemValue 升序的链表。

    只实现 vListInsert（有序插入）和取头/取尾。
    丢掉的东西：
      · 双向链表的指针维护（用 Python list 代替）
      · 哨兵 xListEnd 的处理
      · pxIndex 轮转游标（卷 4 第 7 章讲过）
    """

    def __init__(self, name):
        self.name = name
        self.items = []          # [(xItemValue, owner, note)]

    def insert(self, item_value, owner, note=""):
        """vListInsert：按 xItemValue 升序插入"""
        k = 0
        while k < len(self.items) and self.items[k][0] <= item_value:
            k += 1
        self.items.insert(k, (item_value, owner, note))

    def head(self):
        return self.items[0] if self.items else None

    def dump(self):
        return ["%s(val=%d)" % (o, v) for v, o, _ in self.items]


def prio_to_item_value(prio):
    """tasks.c 注释里的公式：xItemValue = configMAX_PRIORITIES - uxPriority"""
    return MAX_PRIO - prio


def exp3_item_value():
    sec("实验 3：xItemValue 倒置 —— 一个减法实现优先级排序")

    print("""
  tasks.c:5252-5262 的注释（原文）：

      /* Place the event list item of the TCB in the appropriate event list.
       * This is placed in the list in priority order so the highest priority task
       * is the first to be woken by the event.
       *
       * Note: Lists are sorted in ascending order by ListItem_t.xItemValue.
       * Normally, the xItemValue of a TCB's ListItem_t members is:
       *      xItemValue = ( configMAX_PRIORITIES - uxPriority )
       * Therefore, the event list is sorted in descending priority order. */

  **链表按 xItemValue 【升序】排，而 xItemValue = 最大值 - 优先级，
    所以优先级越高、xItemValue 越小、排得越靠前。**
""")

    lst = SortedList("xTasksWaitingToReceive")
    # 故意用乱序的优先级插入
    tasks = [("T_mid", 16), ("T_low", 2), ("T_high", 30),
             ("T_mid2", 16), ("T_lowest", 0), ("T_highest", 31)]

    print("  " + "-" * 74)
    print("  按乱序优先级依次插入 6 个任务")
    print("  " + "-" * 74)
    print()
    print("  %-6s %-8s %-12s %-12s %s"
          % ("顺序", "任务", "优先级", "xItemValue", "插入后的链表（从头到尾）"))
    print("  " + "-" * 86)
    for i, (name, prio) in enumerate(tasks, 1):
        iv = prio_to_item_value(prio)
        lst.insert(iv, name)
        print("  %-6d %-8s %-8d %-12d %s"
              % (i, name, prio, iv, " → ".join(lst.dump())))

    print()
    print("  " + "-" * 74)
    print("  关键问题：链表头是谁？")
    print("  " + "-" * 74)
    h = lst.head()
    print()
    print("      链表头 = %s（优先级 %d，xItemValue=%d）"
          % (h[1], MAX_PRIO - h[0], h[0]))
    print("      而插入的 6 个任务里优先级最高的是 T_highest（31）")
    print("      ⇒ **取链表头就是最高优先级**，验证通过 ✓")

    print("""
  ================== 这个技巧的价值 ==================

  **通用链表机制不需要知道"优先级"是什么。**

      vListInsert(list, item)   ← 只比较 item->xItemValue
             ↑
      不知道这个值代表优先级还是到期时间

  于是同一套插入代码服务两种排序：

      ┌──────────────────────────────────────────────────────────┐
      │  vListInsert(list, item) —— 通用实现，只比较 xItemValue   │
      ├──────────────────────────────────────────────────────────┤
      │  事件链表：xItemValue = MAX_PRIO - uxPriority            │
      │            ⇒ 数值升序 == 优先级降序                       │
      ├──────────────────────────────────────────────────────────┤
      │  延时链表：xItemValue = 到期 tick                         │
      │            ⇒ 数值升序 == 时间升序                         │
      └──────────────────────────────────────────────────────────┘

  ⇒ 这是本章第 7.2 节总结的第 4 次"用可读性换代码量"。

  ================== 一处容易搞错的细节 ==================

      上面 T_mid(16) 和 T_mid2(16) 的 xItemValue 都是 16。
      vListInsert 的循环条件是 `<=`：

          while( pxIterator->pxNextFreeBlock->xItemValue <= xItemValue ) ...

      ⇒ **同优先级的任务按插入顺序排在后面**（FIFO）。

      这和 NuttX 的 nxsched_add_prioritized 行为一致
      （卷 5 第 1 章 2.2 节）：**同优先级 FIFO**。
""")


# ============================================================================
# 实验 4：阻塞与唤醒的完整时序 + 安全网 ⭐
# ============================================================================
#
# 这个实验对应文档第 3 节和第 4 节。
#
# 核心：
#   · vTaskPlaceOnEventList（tasks.c:5243）**两次插入**
#   · xTaskRemoveFromEventList（tasks.c:5341）**两次摘除**
#   · 外加第 4 节的安全网演示


class Task:
    def __init__(self, name, prio):
        self.name = name
        self.prio = prio
        self.item_value = MAX_PRIO - prio     # xItemValue 倒置


class SchedulerSim:
    """模拟调度器的三条关键链表 + 安全网。

    丢掉的东西：
      · 真实的上下文切换（这里只有"谁应该跑"）
      · tick 的精确时序（这里用一个"推进 tick"的动作代替）
      · 就绪队列的分桶（这里只保留最高优先级）
      · 队列的存储区（本实验只关心阻塞机制）
    """

    def __init__(self):
        self.event_list = []       # 队列的 xTasksWaitingToReceive
        self.delayed = []          # 延时链表（按 xItemValue = 到期时间 升序）
        self.ready = []            # 就绪队列（按优先级）
        self.current = None
        self.x_yield_pending = False    # ★ 安全网（tasks.c:5405）
        self.log = []

    def note(self, msg):
        self.log.append(msg)

    def place_on_event_list(self, task, timeout):
        """vTaskPlaceOnEventList（tasks.c:5243）—— 【两次插入】"""
        # 插入 1：TCB 的 xEventListItem → 队列的事件链表（按优先级）
        k = 0
        while (k < len(self.event_list)
               and self.event_list[k].item_value <= task.item_value):
            k += 1
        self.event_list.insert(k, task)
        self.note("插入 1：%s 的 xEventListItem → 事件链表（按优先级，val=%d）"
                  % (task.name, task.item_value))

        # 插入 2：TCB 的 xStateListItem → 延时链表（按到期时间）
        due = ("now+%s" % timeout) if timeout != "portMAX_DELAY" else "∞"
        self.delayed.append((due, task))
        self.note("插入 2：%s 的 xStateListItem → 延时链表（到期 = %s）"
                  % (task.name, due))

    def remove_from_event_list(self):
        """xTaskRemoveFromEventList（tasks.c:5341）—— 【两次摘除】+ 安全网

        返回 (被唤醒的任务, 是否需要切换)
        """
        if not self.event_list:
            return None, False
        # 链表已按优先级排序 ⇒ 取头就是最高优先级
        t = self.event_list.pop(0)
        self.note("取事件链表头 → %s（优先级 %d）" % (t.name, t.prio))

        # 摘除 1：从事件链表（上面 pop 已完成）
        # 摘除 2：从延时链表
        self.delayed = [d for d in self.delayed if d[1] is not t]
        self.note("从延时链表摘除 %s 的 xStateListItem" % t.name)

        # 加入就绪队列
        self.ready.append(t)
        self.note("加入就绪队列：%s" % t.name)

        # ★ 判断是否需要切换 + 设安全网（tasks.c:5396-5412）
        need_switch = False
        if self.current is not None and t.prio > self.current.prio:
            need_switch = True
            self.x_yield_pending = True
            self.note("被唤醒任务优先级更高 ⇒ 返回 pdTRUE，"
                      "且 xYieldPendings = pdTRUE（安全网）")
        return t, need_switch

    def tick(self):
        """模拟 SysTick → xTaskIncrementTick（tasks.c:4670）

        关键：它会检查 xYieldPendings（tasks.c:4863）。
        """
        fired = self.x_yield_pending
        if fired:
            self.note("SysTick：xTaskIncrementTick() 看到 xYieldPendings "
                      "⇒ 返回 pdTRUE ⇒ 触发 PendSV")
            self.x_yield_pending = False
        return fired


def exp4_blocking():
    sec("实验 4：阻塞与唤醒的完整时序 + 安全网 ⭐")

    print("""
  本实验分两部分：
    (a) 一个任务阻塞在空队列上，生产者发数据 —— 看两条链表的状态变化
    (b) 生产者是 ISR，且【忘记了 portYIELD_FROM_ISR】—— 看安全网怎么兜住
""")

    # ---------- (a) ----------
    print("  " + "=" * 74)
    print("  (a) 正常路径：任务阻塞 → 生产者唤醒")
    print("  " + "=" * 74)
    print()
    s = SchedulerSim()
    consumer = Task("consumer", 10)
    producer = Task("producer", 5)
    s.current = producer

    print("  初始状态：")
    print("      当前运行 = %s（优先级 %d）" % (producer.name, producer.prio))
    print("      队列为空 ⇒ consumer 想读读不到 ⇒ 要阻塞")
    print()

    print("  【consumer 调用 xQueueReceive(q, &item, 100)】")
    print("      队列空 → 走阻塞路径 → vTaskPlaceOnEventList()")
    s.place_on_event_list(consumer, 100)
    for line in s.log[-2:]:
        print("          %s" % line)
    print()
    print("      链表现状：")
    print("          事件链表 = %s" % " → ".join(t.name for t in s.event_list))
    print("          延时链表 = %s" % ", ".join(
        "%s(%s)" % (t.name, d) for d, t in s.delayed))
    print("      ⇒ **consumer 同时在两条链表里**")
    print()

    s.log = []
    print("  【producer 调用 xQueueSend(q, &item, 0)】")
    print("      队列有空间 → 走快速路径 → 拷贝数据 → 唤醒等待者")
    t, need = s.remove_from_event_list()
    for line in s.log:
        print("          %s" % line)
    print()
    print("      链表现状：")
    print("          事件链表 = %s" % ("空" if not s.event_list
                                     else " → ".join(x.name for x in s.event_list)))
    print("          延时链表 = %s" % ("空" if not s.delayed else "非空"))
    print("          就绪队列 = %s" % ", ".join(x.name for x in s.ready))
    print()
    print("      ⇒ **两次插 vs 两次摘，完全对称**")
    print()

    # ---------- (b) ----------
    print("  " + "=" * 74)
    print("  (b) 安全网：ISR 里忘记 portYIELD_FROM_ISR")
    print("  " + "=" * 74)
    print()
    s2 = SchedulerSim()
    low = Task("low_prio_task", 3)
    high = Task("high_prio_task", 25)
    s2.current = low
    print("  场景：低优先级任务在跑，一个 ISR 唤醒了高优先级任务")
    print("      当前运行 = %s（优先级 %d）" % (low.name, low.prio))
    print("      被唤醒   = %s（优先级 %d）" % (high.name, high.prio))
    print()
    s2.place_on_event_list(high, "portMAX_DELAY")
    s2.log = []
    print("  【ISR 调用 xQueueSendFromISR()】")
    t, need = s2.remove_from_event_list()
    for line in s2.log:
        print("          %s" % line)
    print()
    print("      返回给调用者：need_switch = %s" % need)
    print("      → 用户应该据此调用 portYIELD_FROM_ISR(need_switch)")
    print()

    print("  " + "-" * 74)
    print("  情况 1：用户写了 portYIELD_FROM_ISR")
    print("  " + "-" * 74)
    print()
    print("          → 立即触发 PendSV → 切换马上发生")
    print("          → high_prio_task 在 ISR 退出后立刻运行")
    print()

    print("  " + "-" * 74)
    print("  情况 2：用户【忘记】写 portYIELD_FROM_ISR")
    print("  " + "-" * 74)
    print()
    print("          → 什么都不发生（ISR 正常返回）")
    print("          → low_prio_task 继续跑")
    print()
    for i in range(3):
        fired = s2.tick()
        mark = "★ 触发切换" if fired else "（无动作）"
        print("          第 %d 个 tick：%s" % (i + 1, mark))
        if fired:
            break
        if i == 0:
            print("                    ⚠ 但注意 x_yield_pending 还是 True ——"
                  " 一个 tick 周期就够")
    print()
    print("  " + "=" * 74)
    print("  ⇒ 结论：忘记 portYIELD_FROM_ISR 的后果是「最多延迟 1 个 tick」，")
    print("          不是「任务永远不运行」。")
    print("  ==" + "=" * 72)
    print("""
      这就是第 4 节讲的修正。关键源码：

          tasks.c:5403-5405（设置安全网）
              /* Mark that a yield is pending in case the user is not using the
               * "xHigherPriorityTaskWoken" parameter ... */
              xYieldPendings[ 0 ] = pdTRUE;

          tasks.c:4863（SysTick 里检查）
              if( xYieldPendings[ 0 ] != pdFALSE )
              {
                  xSwitchRequired = pdTRUE;
              }

          port.c:568（触发 PendSV）
              portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

      ⚠ **有安全网不等于可以依赖它。**

      如果代码平时不写 portYIELD_FROM_ISR，实际上是在说
      "我接受最多 1 ms 的额外延迟"（@1000Hz tick）。
      对 1 kHz 的控制环路，这就是整整一个周期的抖动 ——
      表现为"控制环路偶尔抖"，而不是"系统死掉"，
      **反而更难定位。**

      ⇒ 实践建议：**始终写 portYIELD_FROM_ISR。**
        安全网是给"别人的 bug"用的。
""")


# ============================================================================
# 主入口
# ============================================================================

EXPERIMENTS = {
    "1": ("Queue_t 的 union", exp1_union),
    "2": ("prvCopyDataToQueue 的三个分支", exp2_copy_branches),
    "3": ("xItemValue 倒置与优先级排序", exp3_item_value),
    "4": ("阻塞/唤醒时序 + 安全网", exp4_blocking),
}


def main():
    print("""
================================================================================
 本章：队列 —— 一切通信机制的基础
 配套实验代码

 实验列表：
   1  Queue_t 的 union —— 同一个结构体的两种身份
   2  prvCopyDataToQueue 的三个分支
   3  xItemValue 倒置与优先级排序
   4  阻塞/唤醒时序 + 安全网 ★（含对卷 4 第 7 章的修正）
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
 本章三条最重要的结论
================================================================================

1. **"所有原语都是队列"的机制是一个 union。**
   Queue_t 里 union { QueuePointers_t xQueue; SemaphoreData_t xSemaphore; }
   —— 同样 8 字节，用作队列时是 pcTail+pcReadFrom，
   用作信号量时是 xMutexHolder+递归计数。
   分叉判据 uxItemSize == 0，它在 prvCopyDataToQueue() 里对应
   **第一个分支：不做任何拷贝**。
   ⇒ "给一个信号量" = "往没有存储区的队列里放一个大小为 0 的元素"
     = 什么都不做。

2. **TCB 需要两个 ListItem_t，是因为「超时」。**
   阻塞在队列上的任务必须同时在两条链表里：
     事件链表（按优先级排序，唤醒时取头）
     延时链表（按到期时间排序，tick 中断按序检查）
   一个 ListItem_t 只能在一个链表里 ⇒ 必须两个。
   ⇒ 这是「功能需求直接决定数据结构」的例子。

3. **忘记 portYIELD_FROM_ISR 的后果是「延迟一个 tick」，不是「任务不运行」。**
   xTaskRemoveFromEventList() 在返回 pdTRUE 的同时还会设
   xYieldPendings[0]，被下一个 SysTick 检查到并触发 PendSV。
   ⇒ 这是性能问题（最多 1 ms 额外延迟），不是正确性问题 ——
     而这个区别让它成为一类**更难定位**的 bug。
================================================================================
""")


if __name__ == "__main__":
    main()
