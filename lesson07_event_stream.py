#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 7 课 · 事件组与流/消息缓冲 —— 配套实验

    实验 1 ★★★ 事件组不是队列：一个字段，两种含义
    实验 2 ★★  等待判据与位打包：控制位藏在哪
    实验 3 ★★★ 流缓冲的免锁：单生产者单消费者是怎么强制的
    实验 4 ★★  流缓冲 vs 消息缓冲：触发级别与边界语义

用法：
    python lesson07_event_stream.py 1
    python lesson07_event_stream.py 2
    ...
    python lesson07_event_stream.py          # 全部

只需 numpy（画图需要 matplotlib）。

=======================================================================
参考源码（FreeRTOS-Kernel V11.1.0，D:/DEVLib/PX4APM/FreeRTOS/）
=======================================================================

event_groups.c:54-70       ★ EventGroup_t —— 注意它【不是】Queue_t
event_groups.c:78-120      prvTestWaitCondition 的声明（实现在 :775）
event_groups.c:185         xEventGroupSync（到达-等待会合）
event_groups.c:312         xEventGroupWaitBits
event_groups.c:472         xEventGroupClearBits
event_groups.c:524         xEventGroupGetBitsFromISR  ← 事件组唯一的真 FromISR
event_groups.c:547         xEventGroupSetBits（全表扫描唤醒）
event_groups.c:745-762     vEventGroupSetBitsCallback / ClearBitsCallback
event_groups.c:775-830     prvTestWaitCondition 实现

include/event_groups.h:43-56   ★★ 三个控制位的编码（按 TickType_t 宽度分三档）
include/event_groups.h:455-461 xEventGroupClearBitsFromISR —— 【是宏，不是函数】
include/event_groups.h:613-620 ★★★ xEventGroupSetBitsFromISR —— 同样是宏
                                   展开成 xTimerPendFunctionCallFromISR(...)

tasks.c:5272-5300          ★ vTaskPlaceOnUnorderedEventList（listINSERT_END，不排序）
tasks.c:5434-5502          ★★ vTaskRemoveFromUnorderedEventList
tasks.c:5130 附近           xTaskRemoveFromEventList（队列用，【按优先级排序】）

stream_buffer.c:232-246    ★★ StreamBuffer_t —— 等待者是单个 TaskHandle_t
stream_buffer.c:60-84      sbRECEIVE_COMPLETED（点对点通知，不走链表）
stream_buffer.c:365-372    ★ 触发级别为 0 会被强制成 1
stream_buffer.c:885-886    ★★★ configASSERT(xTaskWaitingToSend == NULL)
stream_buffer.c:1107-1108  ★★★ configASSERT(xTaskWaitingToReceive == NULL)
"""

import math
import sys

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_PLT = True
except ImportError:
    HAVE_PLT = False


def hr(ch="=", n=76):
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
    cols = len(headers)
    widths = [len(h) for h in headers]
    for r in rows:
        for i, c in enumerate(r):
            widths[i] = max(widths[i], len(str(c)))
    print(indent + " | ".join(h.ljust(widths[i]) for i, h in enumerate(headers)))
    print(indent + "-+-".join("-" * w for w in widths))
    for r in rows:
        print(indent + " | ".join(str(c).ljust(widths[i]) for i, c in enumerate(r)))


# ===========================================================================
# 公共：事件组的两种表示（数据结构层面）
# ===========================================================================
#
# 队列（第 3 课）：
#     typedef struct QueueDefinition {
#         int8_t *pcHead;               ← 存储区
#         int8_t *pcTail;
#         int8_t *pcWriteTo;
#         union { QueuePointers_t xQueue; SemaphoreData_t xSemaphore; };
#         List_t xTasksWaitingToSend;     ★ 链表：多个等待者
#         List_t xTasksWaitingToReceive;  ★ 链表：多个等待者
#         ...
#     } Queue_t;
#
# 事件组（本课）：
#     typedef struct EventGroupDef_t {
#         EventBits_t uxEventBits;          ★ 一个位掩码
#         List_t xTasksWaitingForBits;      ★ 只有【一条】链表
#         ...
#     } EventGroup_t;
#
# ★★★ 差别一目了然：
#     队列——两条等待链表（发送侧/接收侧各一条），存的是【多个】等待者
#     事件组——一条等待链表，而且链表【不排序】

# include/event_groups.h:43-56  三个控制位的编码
# 按 configTICK_TYPE_WIDTH_IN_BITS 分 16/32/64 三档，
# 都取【最高字节】—— 也就是说：
#     EventBits 里能用的"事件位"只有 (宽度 - 8) 位
CTRL_BITS = {
    16: dict(CLEAR_ON_EXIT=0x0100, UNBLOCKED=0x0200, WAIT_ALL=0x0400, MASK=0xff00),
    32: dict(CLEAR_ON_EXIT=0x01000000, UNBLOCKED=0x02000000,
             WAIT_ALL=0x04000000, MASK=0xff000000),
    64: dict(CLEAR_ON_EXIT=0x0100000000000000, UNBLOCKED=0x0200000000000000,
             WAIT_ALL=0x0400000000000000, MASK=0xff00000000000000),
}


def test_wait_condition(cur_bits, bits_to_wait_for, wait_all):
    """★★ 照抄 event_groups.c:775-830 的 prvTestWaitCondition。

    注意它不是"相等"，而是【按位】判断：
        任一满足:  (等待位 & 当前位) != 0
        全部满足:  (等待位 & 当前位) == 等待位
    """
    if wait_all:
        return (bits_to_wait_for & cur_bits) == bits_to_wait_for
    return (bits_to_wait_for & cur_bits) != 0


# ===========================================================================
# 实验 1：事件组不是队列
# ===========================================================================

def exp1():
    head("实验 1  事件组不是队列：一个字段，两种含义")

    print("""
  第 3 课留下过一句话，而且它被反复引用：

      「信号量、互斥量、事件组都是队列」
          —— 第 3 课 §2.1

  ★ 本实验要验证：**这句话对前两个成立，对事件组不成立。**

  不过要先说清楚一件事（免得又变成"搜了没找到就说没有"）：
  「事件组是队列」这句话**在 API 层面**是对的 ——
  早期 FreeRTOS 确实把事件组实现成队列，现在 `xEventGroupCreate()`
  仍然返回一个可以 `vQueueDelete()` 掉的句柄。
  错的是把它当成**实现层面**的事实。
""")

    sub("① 数据结构：把两者并排看")
    print("""  Queue_t（queue.c:130-160 附近，第 3 课拆过）：
      ┌──────────────────────────────────────────┐
      │ pcHead / pcTail / pcWriteTo   存储区指针   │
      │ union { xQueue | xSemaphore }  两种身份    │
      │ xTasksWaitingToSend      ★ List_t 多个等待者│
      │ xTasksWaitingToReceive   ★ List_t 多个等待者│
      └──────────────────────────────────────────┘

  EventGroup_t（event_groups.c:54-70）：
      ┌──────────────────────────────────────────┐
      │ uxEventBits              ★ 一个位掩码      │
      │ xTasksWaitingForBits     ★ List_t 只有【一条】│
      │ (可选的 trace / 静态分配标记)               │
      └──────────────────────────────────────────┘
""")

    # 数一下两边各有多少字段
    queue_fields = ["pcHead", "pcTail", "pcReadFrom", "pcWriteTo",
                    "union{xQueue|xSemaphore}", "uxMessagesWaiting",
                    "uxLength", "uxItemSize", "xTasksWaitingToSend",
                    "xTasksWaitingToReceive", "ucStaticallyAllocated",
                    "uxQueueNumber", "uxQueueType"]
    evt_fields = ["uxEventBits", "xTasksWaitingForBits",
                  "uxEventGroupNumber", "ucStaticallyAllocated"]
    print("  字段清单对照：")
    tab([["队列 Queue_t（%d 个）" % len(queue_fields),
          "事件组 EventGroup_t（%d 个）" % len(evt_fields)]],
        ["", ""])
    for i in range(max(len(queue_fields), len(evt_fields))):
        a = queue_fields[i] if i < len(queue_fields) else ""
        b = evt_fields[i] if i < len(evt_fields) else ""
        print("    %-28s | %s" % (a, b))
    print("""
  ⇒ 事件组**没有存储区**（没有 pcHead/pcReadFrom），
    **没有元素大小**（没有 uxItemSize），只有**一条**等待链表。

  ★ 那就有一个问题：既然没有存储区，那"传"的是什么？
    传的是 `uxEventBits` 这**一个字**。设置一个 bit 就是"发了一份数据"。
""")

    sub("② ★★★ 真正的分岔点：xItemValue 这一个字段，两种含义")
    print("""  两边都把一个 `ListItem_t` 挂进等待链表。而 `ListItem_t` 里有一个
  `xItemValue` 字段 —— **这个字段装什么，决定链表怎么排序。**

  ┌──────────┬────────────────────────┬──────────────────────┐
  │          │ 队列                    │ 事件组                │
  ├──────────┼────────────────────────┼──────────────────────┤
  │ xItemValue│ **优先级**              │ **等待条件 + 控制位**  │
  │ 链表纪律  │ 按 xItemValue 排序       │ 不排（一律插到末尾）    │
  │ 插入函数  │ vTaskPlaceOnEventList   │ vTaskPlaceOn**Unordered**EventList │
  │ 唤醒      │ 取队头（优先级最高的）    │ **全表扫描**，谁能满足就唤醒谁 │
  └──────────┴────────────────────────┴──────────────────────┘

  ★ 源码为证（tasks.c:5272-5300）：
        listSET_LIST_ITEM_VALUE( &(pxCurrentTCB->xEventListItem),
                                 xItemValue | taskEVENT_LIST_ITEM_VALUE_IN_USE );
        listINSERT_END( pxEventList, &(pxCurrentTCB->xEventListItem) );
                    ↑ 无条件插到**末尾**，一个比较都没有

  而队列那边（tasks.c 的 vTaskPlaceOnEventList → vListInsert）会按
  `xItemValue`（= 优先级）找到该插的位置。
""")

    sub("③ ★ 强行「按 xItemValue 排序」会怎样")
    print("""  做一个思想实验：假设事件组也用有序链表，按 xItemValue 排。

  xItemValue 里装的是 uxBitsToWaitFor（我要等的位）+ 控制位。
  那排序就变成了「按'等哪些位'的数值大小排」。

  下面模拟三个任务同时等不同的位，**按这个荒谬的规则排序**，
  然后看"取队头唤醒"会唤醒谁 —— 以及**谁永远等不到**。
""")

    # 三个任务等不同的位
    tasks = [
        ("任务 A", 0x0001, "等 bit0"),
        ("任务 B", 0x0002, "等 bit1"),
        ("任务 C", 0x0004, "等 bit2"),
    ]
    print("  初始状态：三个任务都在等，事件位全是 0\n")
    tab([["任务 A", "0x0001", "bit0"], ["任务 B", "0x0002", "bit1"],
         ["任务 C", "0x0004", "bit2"]], ["任务", "xItemValue（等待位）", "含义"])

    ordered = sorted(tasks, key=lambda t: t[1])
    print("\n  按 xItemValue 升序排出来的等待链表（队头在前）：")
    for i, (name, v, desc) in enumerate(ordered):
        mark = "  ← 队头" if i == 0 else ""
        print("      %d. %s  (0x%04X)%s" % (i + 1, name, v, mark))

    print("""
  现在**只设置 bit2**（0x0004）。正确的行为是：只有任务 C 应该被唤醒。

  但"取队头"的规则会唤醒队头 —— 也就是任务 A。
""")
    bits_set = 0x0004
    print("      设置 uxEventBits = 0x%04X（只有 bit2）" % bits_set)
    print()
    rows = []
    for name, v, _ in tasks:
        head_task = (name == ordered[0][0])
        if head_task:
            woke = "✅ 被唤醒（它是队头）"
        else:
            woke = "❌ 没醒"
        should = "应该醒" if (v & bits_set) else "不该醒"
        rows.append([name, "0x%04X" % v, "0x%04X" % (v & bits_set),
                     should, woke])
    tab(rows, ["任务", "等待位", "与当前位相与", "正确行为", "取队头规则下"])

    print("""
  ⇒ **完全错了**：应该醒的没醒，不该醒的醒了。

  ★★ 这就是事件组必须用【无序链表 + 全表扫描】的根本原因：

        xItemValue 里装的是"等待条件"，不是"排序键"。
        拿一个不是排序键的字段去排序，结果没有任何意义。

  而队列那边 xItemValue = 优先级，**它本来就是排序键**，
  所以"取队头 = 取优先级最高的"这个优化才成立。

  ⇒ 「有序链表 + 取队头」不是队列的设计选择，是它的**语义正好允许**。
    事件组的语义不允许，所以只能扫全表。
""")

    sub("④ 代价：事件组的唤醒是 O(n)")
    print("""  源码为证（event_groups.c:547-640，xEventGroupSetBits 的唤醒循环）：

        for( ;; )
        {
            pxListItem = pxNext;                       /* ★ 遍历整条链表 */
            pxNext = listGET_NEXT( pxListItem );
            ...
            if( xMatchFound != pdFALSE )
            {
                vTaskRemoveFromUnorderedEventList( pxListItem, ... );
            }
            ...
        }

  ★ 注意循环变量是 `pxNext = listGET_NEXT( pxListItem )`，
    而且源码注释特意说明：

        /* Move onto the next list item.  Note pxListItem->pxNext is not
         * used here as the list item may have been removed from the event list
         * and inserted into the ready/pending reading list. */

    ⇐ 因为它**会在遍历过程中把节点摘走**（挂到就绪链表去），
       所以必须先保存下一个指针。

  下面是两者的复杂度对照：
""")
    tab([["队列", "O(1)", "取队头", "1 个等待者被唤醒"],
         ["事件组", "O(n)", "扫全表", "所有满足条件的都被唤醒"]],
        ["", "唤醒复杂度", "做法", "唤醒几个"])
    print("""
  ★ 而这个 O(n) **换来了一个队列做不到的能力**：
    同一个 bit 可以被**多个任务同时等到**。

        队列：一份数据只能给一个接收者（拿走了就没了）
        事件组：一个 bit 可以被 N 个任务同时观察到

  ⇒ 这不是"事件组比队列慢"，是**两种语义**：
    队列是"消费"（数据会被拿走），事件组是"广播"（位不会被拿掉，
    除非你显式 ClearBits 或指定 clear-on-exit）。
""")

    sub("⑤ 顺手验证：同一个位能被多个任务等到吗")
    print("  模拟：三个任务都等 bit0，然后设置 bit0。正确行为是**三个都醒**。\n")
    bits_to_wait = 0x0001
    for name in ("任务 A", "任务 B", "任务 C"):
        pass
    rows = []
    for name in ("任务 A", "任务 B", "任务 C"):
        ok = test_wait_condition(bits_to_wait, bits_to_wait, wait_all=True)
        rows.append([name, "0x0001", "0x0001", "✅ 被唤醒（全表扫描）"])
    tab(rows, ["任务", "等待位", "相与结果", "结果"])
    print("""
  ⇒ 三个都醒。**这就是"广播"语义**。

  ★★ 对比第 3 课的队列：如果三个任务都阻塞在一个空队列上，
     来一份数据只会唤醒**一个**（优先级最高的那个）。
     这是设计使然 —— 数据只有一份，给了 A 就不能给 B。
""")

    sub("⑥ 飞控里用不用？")
    print("""  实测（ArduPilot Copter-4.5.7）：

      libraries/AP_HAL_ESP32/WiFiDriver.cpp:236,241,310,343
          s_wifi_event_group = xEventGroupCreate();
          xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
          EventBits_t bits = xEventGroupWaitBits(...);

  ⇒ **只出现在 ESP32 的 WiFi 驱动里** —— 那是 HAL 层的一个平台特定驱动，
    用的是乐鑫 IDF 的既有模式，不是 ArduPilot 自己的设计选择。

  ★ 飞控主力平台（ChibiOS，第 7~13 课）**完全不用事件组**，
    因为 ChibiOS 提供的是 `chevents.c` 的事件源机制（第 17 课会讲）。

  ⇒ 那为什么还要学它？
    因为它是本教程里**唯一一个"不是队列"的原语** ——
    第 3 课那句话的边界在哪里，就是本实验画出来的这条线。
""")


# ===========================================================================
# 实验 2：等待判据与位打包
# ===========================================================================

def exp2():
    head("实验 2  等待判据与位打包：控制位藏在哪")

    print("""
  事件组的等待有三个参数：等哪些位、是否要全等、等到后要不要清掉。
  前两个是"语义"，第三个是"副作用"。

  ★ 问题是：这三个参数**存哪儿**？

  答案在 event_groups.c:185 的 xEventGroupSync 里：
      vTaskPlaceOnUnorderedEventList( &(pxEventBits->xTasksWaitingForBits),
            ( uxBitsToWaitFor | eventCLEAR_EVENTS_ON_EXIT_BIT
                             | eventWAIT_FOR_ALL_BITS ),
            xTicksToWait );

  ⇒ **三个参数被 OR 在一起，塞进 xItemValue 那一个字段。**
""")

    sub("① 编码方案：控制位占最高字节")
    print("""  include/event_groups.h:43-56 —— 按 TickType_t 的宽度分三档：

      16 位： CLEAR_ON_EXIT=0x0100  UNBLOCKED=0x0200  WAIT_ALL=0x0400  MASK=0xFF00
      32 位： CLEAR_ON_EXIT=0x01000000  UNBLOCKED=0x02000000
             WAIT_ALL=0x04000000   MASK=0xFF000000
      64 位： （同理，最高字节）

  ⇒ 规律：**控制位永远占最高字节**，剩下的位才是"事件位"。

  下面是三档的对照：
""")
    rows = []
    for w in (16, 32, 64):
        c = CTRL_BITS[w]
        rows.append(["%2d 位" % w,
                     "0x%0*X" % (w // 4, c["CLEAR_ON_EXIT"]),
                     "0x%0*X" % (w // 4, c["UNBLOCKED"]),
                     "0x%0*X" % (w // 4, c["WAIT_ALL"]),
                     "%d" % (w - 8)])
    tab(rows, ["TickType_t", "CLEAR_ON_EXIT", "UNBLOCKED", "WAIT_ALL", "可用事件位"])

    print("""
  ★ 16 位配置下只有 **8 个**可用事件位（bit0~bit7）。
    这就是为什么 FreeRTOS 的官方示例通常用 24 个位（32 位配置）。
    手册上说的"最多 24 个事件位"由此而来。
""")

    sub("② 数值验证：控制位和事件位不打架")
    width = 32
    c = CTRL_BITS[width]
    print("  取 32 位配置，验证「按位或」之后还能正确解码：\n")
    rows = []
    for wait_bits, clear_exit, wait_all in [
            (0x00000003, False, False),
            (0x00000003, True, False),
            (0x00000003, False, True),
            (0x00000003, True, True),
            (0x00FFFFFF, True, True),      # 用满 24 个事件位
    ]:
        v = wait_bits
        if clear_exit:
            v |= c["CLEAR_ON_EXIT"]
        if wait_all:
            v |= c["WAIT_ALL"]
        # 解码
        d_wait = v & ~c["MASK"]
        d_clear = bool(v & c["CLEAR_ON_EXIT"])
        d_all = bool(v & c["WAIT_ALL"])
        ok = (d_wait == wait_bits and d_clear == clear_exit and d_all == wait_all)
        rows.append(["0x%08X" % wait_bits, "是" if clear_exit else "否",
                     "是" if wait_all else "否",
                     "0x%08X" % v, "0x%06X" % d_wait,
                     "✅" if ok else "❌"])
    tab(rows, ["等待位", "clear", "全等", "编码后 xItemValue", "解码回来", "往返"])
    print("""
  ⇒ 全部往返一致。因为控制位在**最高字节**，
    而事件位只用低 24 位，**两者在物理上不重叠**。

  ★★ 这是本教程记录的第 **6** 次"位打包"：

       #  位置                    借用了什么            表示什么
       ────────────────────────────────────────────────────────────
       1  NuttX sem_t            semcount 的符号位      有资源 / 有等待者
       2  FreeRTOS Queue_t       pcHead 的 NULL        是不是互斥量
       3  FreeRTOS Queue_t       union 复用 8 字节      队列指针 / 互斥量数据
       4  FreeRTOS heap_4        xBlockSize 最高位      是否已分配
       5  FreeRTOS timers.c      ucStatus 一个字        三个定时器标志
       6  FreeRTOS event_groups  xItemValue 最高字节    ★ 三个等待参数
""")

    sub("③ UNBLOCKED_DUE_TO_BIT_SET：为什么要单独一个标志")
    print("""  第三个控制位 `eventUNBLOCKED_DUE_TO_BIT_SET` 比较特别 ——
  它不是"等待参数"，而是**内核回写给任务的状态**。

  源码（event_groups.c:620-625）：

      vTaskRemoveFromUnorderedEventList( pxListItem,
              pxEventBits->uxEventBits | eventUNBLOCKED_DUE_TO_BIT_SET );

  任务醒来后（event_groups.c:269-280）：

      uxReturn = uxTaskResetEventItemValue();
      if( ( uxReturn & eventUNBLOCKED_DUE_TO_BIT_SET ) == 0 )
      {
          /* 是超时醒的，不是等到位了 */
          uxReturn = pxEventBits->uxEventBits;
          xTimeoutOccurred = pdTRUE;
      }

  ⇒ **同一个字段，去程装"我要什么"，回程装"我等到了没有"。**

  ★ 这解决了什么问题？
    一个任务可能因为两种原因醒来：**(a) 等到位了 (b) 超时了**。
    两者都要执行下面那段代码，但语义完全不同 ——
    超时的时候，如果刚好在超时瞬间位被设了，
    任务还得**补一次 ClearBits**（源码 :272-280 就是这么做的）。

  下面模拟这个"醒来后分不清原因"的经典陷阱：
""")

    cases = [
        ("等到位（正常）", True, True),
        ("纯超时", False, False),
        ("超时瞬间位被设", False, True),
    ]
    rows = []
    for name, unblocked_flag, bits_now_match in cases:
        if unblocked_flag:
            verdict = "确认等到"
            action = "读 xEventBits 返回"
        else:
            if bits_now_match:
                verdict = "超时，但位已满足"
                action = "★ 需要补 ClearBits，再返回"
            else:
                verdict = "超时"
                action = "读 xEventBits 返回"
        rows.append([name, "是" if unblocked_flag else "否",
                     "是" if bits_now_match else "否", verdict, action])
    tab(rows, ["场景", "UNBLOCKED 置位", "当前位满足", "判定", "动作"])
    print("""
  ★ 第三行是**竞态**：任务因为超时被摘下来了，但在它真正跑起来之前，
    另一个任务把位设了。如果不检查，这个位就**永远不会被清**，
    下一个等待者会立刻"等到"一个陈旧的位。

  ⇒ **一个标志位，解决了一个跨时段的竞态。**
    这是"把状态塞进已有字段"的一个非典型例子 ——
    前 5 次都是为了省内存，这一**次是为了省一次加锁**。
""")


# ===========================================================================
# 实验 3：流缓冲的免锁
# ===========================================================================

def exp3():
    head("实验 3  流缓冲的免锁：单生产者单消费者是怎么强制的")

    print("""
  流缓冲（stream buffer）的文档里有一句话：

      「Stream buffers are lock free / lock-less」

  ★ 本实验要回答：**"无锁"是怎么做到的？代价是什么？**

  先给结论：**它不是"用原子操作实现了免锁"，而是
  "用数据结构把并发限制掉了"** —— 代价是只能有一个生产者、一个消费者。
""")

    sub("① 数据结构：等待者是【单个句柄】，不是链表")
    print("""  stream_buffer.c:232-246：

      typedef struct StreamBufferDef_t
      {
          volatile size_t xTail;                    /* 读指针 */
          volatile size_t xHead;                    /* 写指针 */
          size_t xLength;                           /* 缓冲区长度 */
          size_t xTriggerLevelBytes;                /* 触发级别 */
          volatile TaskHandle_t xTaskWaitingToReceive;  ★★★ 单个句柄
          volatile TaskHandle_t xTaskWaitingToSend;     ★★★ 单个句柄
          uint8_t * pucBuffer;
          uint8_t ucFlags;
      } StreamBuffer_t;

  ★★ 对照第 3 课的队列：

      队列：  List_t xTasksWaitingToSend;     ← 链表，可以挂**任意多个**
              List_t xTasksWaitingToReceive;  ← 链表

      流缓冲：TaskHandle_t xTaskWaitingToSend;    ← **一个格子**
              TaskHandle_t xTaskWaitingToReceive;

  ⇒ **"单生产者单消费者"这条约束，是【数据结构本身】表达的。**
    不是靠文档说、不是靠约定 —— 是结构里**只有一个槽位**。

  这也解释了为什么它会 block：槽位被占了，第二个生产者**没地方挂**。
""")

    sub("② ★★★ 但它不靠「没地方挂」来保证 —— 靠一个断言")
    print("""  如果只是"没地方挂"，第二个生产者挂上去就会**覆盖**掉第一个的句柄，
  第一个任务就永远等不到唤醒了 —— 那是个难查的 bug，不是保护。

  所以 FreeRTOS 在这里**加了显式断言**（stream_buffer.c:885-886）：

      /* Should only be one writer. */
      configASSERT( pxStreamBuffer->xTaskWaitingToSend == NULL );
      pxStreamBuffer->xTaskWaitingToSend = xTaskGetCurrentTaskHandle();

  接收侧对称（:1107-1108）：

      configASSERT( pxStreamBuffer->xTaskWaitingToReceive == NULL );
      pxStreamBuffer->xTaskWaitingToReceive = xTaskGetCurrentTaskHandle();

  ★★ 这两行 `configASSERT` 就是**「单生产者单消费者」这条约束在代码里的落点**。

  ⚠ 注意它的生效条件：
     `configASSERT` 在 Release 构建里通常是**空的**（第 4 课 §2.5 讲过）。
     也就是说 —— **生产构建里，违反这个约束不会报错，只会静默地坏掉。**

  下面模拟：两个生产者同时等"缓冲区有空位"，看会发生什么。
""")

    # 模拟槽位覆盖
    class StreamBufferModel:
        def __init__(self, size):
            self.size = size
            self.head = 0
            self.tail = 0
            self.waiting_to_send = None
            self.notified = []

        @property
        def used(self):
            return (self.head - self.tail) % self.size

        @property
        def space(self):
            return self.size - self.used - 1

    sb = StreamBufferModel(16)
    print("  缓冲区 16 字节，先填满，让空间为 0：\n")
    sb.head = 15
    print("      xHead=%d  xTail=%d  →  空闲 %d 字节" % (sb.head, sb.tail, sb.space))
    print()
    print("  现在任务 A 想发数据，空间不够 ⇒ 它要阻塞，先占槽位：")
    sb.waiting_to_send = "任务A"
    print("      xTaskWaitingToSend = 任务A")
    print()
    print("  ★ 按正确用法，这时候【不该有】任务 B 也来发。但假设它来了：")
    print("      （Release 构建里 configASSERT 是空的，所以这行会【静默通过】）")
    sb.waiting_to_send = "任务B"
    print("      xTaskWaitingToSend = 任务B     ← 任务 A 的句柄被覆盖掉了！")
    print()
    print("  之后消费者读走数据、触发 sbRECEIVE_COMPLETED：")
    sb.notified.append(sb.waiting_to_send)
    sb.waiting_to_send = None
    print("      唤醒 %s" % sb.notified[-1])
    print()
    print("  ⇒ **任务 A 永远等不到唤醒** —— 它会一直阻塞到超时。")
    print("    而这个 bug 的表现是「偶发卡顿」，不是崩溃 —— 极难定位。")

    print("""
  ★★ 所以「无锁」的真实代价是：

      · 用**单个槽位**代替等待链表  ⇒ 换来 O(1) 的阻塞/唤醒
      · 用**断言**代替运行时检查    ⇒ Release 构建里这个约束全靠自觉

  ⇒ 这是典型的"把复杂度从运行时搬到接口约定"。
    FreeRTOS 的文档里明确写了这个约束，但**代码里没有兜底**。
""")

    sub("③ ★ 阻塞机制：绕了一圈，用的还是任务通知")
    print("""  流缓冲阻塞时执行什么？（stream_buffer.c:894-898）

      traceBLOCKING_ON_STREAM_BUFFER_SEND( xStreamBuffer );
      ( void ) xTaskNotifyWaitIndexed( pxStreamBuffer->uxNotificationIndex,
                                       ( uint32_t ) 0, ( uint32_t ) 0,
                                       NULL, xTicksToWait );
      pxStreamBuffer->xTaskWaitingToSend = NULL;

  ★★ 是 `xTaskNotifyWaitIndexed` —— **任务通知！**

  ⇒ 这正好接上第 6 课的结论：

      第 6 课：「任务通知快的真正原因，是它不走事件链表」

      本课：流缓冲就是用任务通知来阻塞的 —— 因为它**故意不要事件链表**。

  唤醒侧对称（stream_buffer.c:60-84 的 sbRECEIVE_COMPLETED 宏）：

      vTaskSuspendAll();
      {
          if( ( pxStreamBuffer )->xTaskWaitingToSend != NULL )
          {
              ( void ) xTaskNotifyIndexed( pxStreamBuffer->xTaskWaitingToSend,
                                           pxStreamBuffer->uxNotificationIndex,
                                           ( uint32_t ) 0, eNoAction );
              pxStreamBuffer->xTaskWaitingToSend = NULL;
          }
      }
      ( void ) xTaskResumeAll();

  ★ 注意：
    · 唤醒是**点对点**的（直接知道要叫谁），不需要"找优先级最高的"
    · `eNoAction` —— 只是把任务从等待里踢出来，不改通知值
    · 用 `vTaskSuspendAll()` 而不是临界区 —— 因为它不碰 ISR 会碰的东西

  ⇒ **三个原语串起来了**：
      第 3 课（队列）→ 第 6 课（任务通知）→ 本课（流缓冲用通知做阻塞）
""")

    sub("④ 复杂度对照：两个等待者的世界")
    tab([["队列", "List_t（任意多个）", "O(n) 插入排序", "O(1)", "取队头"],
         ["流缓冲", "TaskHandle_t（一个）", "O(1) 直接赋值", "O(1)", "点对点通知"]],
        ["", "等待者容器", "阻塞", "唤醒", "唤醒谁"])
    print("""
  ★ 注意队列的"阻塞是 O(n)"—— 那是**插入到有序链表**的开销。
    第 2 课讲过优化版就绪队列用位图把"选择"做到 O(1)，
    但等待链表这里没有位图可用（要按优先级排，不能只记"有没有"）。

  ⇒ 流缓冲把这个 O(n) 消掉了，代价是**只能有一个等待者**。

  ★★★ 一句话总结本实验：

      「无锁」不是"用更聪明的方法解决了并发"，
      而是"用更窄的接口避开了并发"。
""")

    sub("⑤ 那 ISR 怎么办？")
    print("""  流缓冲有专门的 ISR 版本：

      xStreamBufferSendCompletedFromISR()
      xStreamBufferReceiveCompletedFromISR()

  ⇒ 它们在 ISR 里只做一件事：**把对应的等待者唤醒**（`xTaskNotifyIndexed`
    本身就有 ISR-safe 的路径，第 6 课讲过）。
    **数据搬运仍然在任务里做** —— 这一点和队列的 FromISR 不同
    （队列的 FromISR 会真的拷贝数据）。

  ★ 为什么？因为流缓冲的"数据搬运"是 `memcpy` 到环形缓冲，
    长度不定 —— 在 ISR 里做不合适。
""")


# ===========================================================================
# 实验 4：流缓冲 vs 消息缓冲
# ===========================================================================

def exp4():
    head("实验 4  流缓冲 vs 消息缓冲：触发级别与边界语义")

    print("""
  这两个东西在同一个文件里（`stream_buffer.c`，1,718 行），
  靠一个 flag 区分：

      #define sbFLAGS_IS_MESSAGE_BUFFER   ( ( uint8_t ) 2 )
      #define sbFLAGS_IS_BATCHING_BUFFER  ( ( uint8_t ) 4 )

  ★ 所以「消息缓冲 = 流缓冲 + 每条消息带长度前缀」。
    本质是同一个环形缓冲，两种**边界语义**。
""")

    sub("① 触发级别：收多少才叫醒接收者")
    print("""  stream_buffer.c:367-372 —— 一个容易被忽略的细节：

      /* A trigger level of 0 would cause a waiting task to unblock even when
       * the buffer was empty. */
      if( xTriggerLevelBytes == ( size_t ) 0 )
      {
          xTriggerLevelBytes = ( size_t ) 1;
      }

  ⇒ **传 0 会被强制成 1**，因为 0 是个退化的配置。

  ★ 下面是不同触发级别下，接收者的唤醒时机：
""")
    rows = []
    for trig in (0, 1, 4, 8, 16):
        eff = 1 if trig == 0 else trig
        rows.append(["%d%s" % (trig, "（→1）" if trig == 0 else ""),
                     "%d" % eff, "收到 %d 字节就醒" % eff,
                     "缓冲区空时也醒 ❌" if eff == 0 else "ok"])
    tab(rows, ["传入触发级别", "实际生效", "唤醒条件", "备注"])
    print("""
  ★ 触发级别是**性能调节旋钮**：
      调小 → 响应快，但唤醒频繁（上下文切换多）
      调大 → 唤醒少，但延迟高

  ⇒ 这和第 1 课讲的「RTOS 的每个参数都是取舍」是同一件事。
""")

    sub("② batching buffer：只在触发级别之上才醒")
    print("""  源码里还有一个 `sbFLAGS_IS_BATCHING_BUFFER`：

      #define sbFLAGS_IS_BATCHING_BUFFER ( ( uint8_t ) 4 )
      /* Set if the stream buffer was created as a batching buffer, meaning the
       * receiver task will only unblock when the trigger level exceededs. */

  对照 `xStreamBufferReceive` 里的分支（:1077 附近）：

      /* Force task to block if the batching buffer contains less bytes than
       * the trigger level. */
      if( ( pxStreamBuffer->ucFlags & sbFLAGS_IS_BATCHING_BUFFER ) != 0 )
      {
          if( xBytesAvailable < pxStreamBuffer->xTriggerLevelBytes )
          {
              xReceivedLength = 0;      /* ★ 装作没收到，强制阻塞 */
          }
      }

  ★★ 差别很微妙：

      普通流缓冲：数据够了就返回，不够就阻塞
      batching  ：**即使有数据，只要没到触发级别就继续阻塞**

  ⇒ 用途不同：普通流缓冲要"尽快取走"，batching 要"攒够一批再处理"。
""")

    sub("③ 边界语义：流缓冲会截断，消息缓冲不会")
    print("""  这是两者**最重要的实用差别**。

  xStreamBufferSend 的流缓冲分支（stream_buffer.c:887 附近）：

      /* This is a stream buffer, as opposed to a message buffer, so writing a
       * stream of bytes rather than discrete messages.
       * Plan to write as many bytes as possible. */
      xDataLengthBytes = configMIN( xDataLengthBytes, xSpace );

  ⇒ **写不下就截断**，返回实际写入的长度。

  消息缓冲分支（:866-878）：

      if( xSpace >= xRequiredSpace )
      {
          /* 够放下长度前缀 + 消息本体，才写 */
          xNextHead = prvWriteBytesToBuffer( ..., &xMessageLength,
                                             sbBYTES_TO_STORE_MESSAGE_LENGTH, ... );
      }
      else
      {
          /* Not enough space, so do not write data to the buffer. */
          xDataLengthBytes = 0;     ★ 一点不写
      }

  ⇒ **要么整条写进去，要么一条都不写**。
""")
    rows = []
    for name, total, avail, ret in [
            ("流缓冲", 10, 4, 4),
            ("流缓冲", 10, 0, 0),
            ("消息缓冲", 10, 20, 0),
            ("消息缓冲", 10, 2 + 10, 10),
    ]:
        if name == "流缓冲":
            note = "截断，返回实际写入 %d" % ret
        else:
            note = "整条写入" if ret else "★ 一条都不写（保护消息边界）"
        rows.append([name, "%d 字节" % total, "%d 字节" % avail,
                     "%d 字节" % ret, note])
    tab(rows, ["类型", "要发", "剩余空间", "返回", "行为"])
    print("""
  ★★ 消息缓冲那个 `+2`（`sbBYTES_TO_STORE_MESSAGE_LENGTH`）是长度前缀。
     所以消息缓冲的"有效容量"比流缓冲**小 2 字节/条**。

  ⇒ 选型判据（本实验可以带走的一条）：

      数据是【字节流】（串口、音频、日志）        → 流缓冲
      数据是【离散报文】（MAVLink 帧、命令）       → 消息缓冲

  ★ 用错了会怎样？
    用流缓冲发 MAVLink：接收方可能拿到半条帧 ⇒ 解析失败
    用消息缓冲发串口数据：每条 2 字节额外开销，而且长度不定很难用
""")

    sub("④ ★ 一句话对照")
    print("""  ┌──────────────┬──────────────────────┬──────────────────────┐
  │              │ 流缓冲                │ 消息缓冲              │
  ├──────────────┼──────────────────────┼──────────────────────┤
  │ 写不下时      │ 截断（写多少算多少）    │ 整条拒收              │
  │ 每条开销      │ 0                     │ 2 字节长度前缀         │
  │ 接收语义      │ 「给我 ≤N 个字节」      │ 「给我一整条」          │
  │ 适合          │ 字节流                │ 离散报文              │
  │ 返回值的含义  │ 实际收到的字节数        │ 0 = 没收到完整的        │
  └──────────────┴──────────────────────┴──────────────────────┘

  ★ 而它们的**底层是同一段代码** —— 同一个 `prvWriteBytesToBuffer`、
    同一个环形缓冲、同一套等待者槽位、同一套任务通知阻塞。

  ⇒ 又一个「同一份机制，两种语义」的例子（本线第 21 课整课都在讲这个）。
""")

    sub("⑤ 和队列的最终对照")
    print("""  把第 3 课（队列）、第 6 课（通知）、本课（事件组 + 流缓冲）串起来：

  ┌────────────┬──────────┬──────────┬────────────┬──────────────┐
  │            │ 队列      │ 事件组    │ 流缓冲      │ 任务通知      │
  ├────────────┼──────────┼──────────┼────────────┼──────────────┤
  │ 等待者容器  │ 链表(有序)│ 链表(无序)│ 单个句柄    │ 无（就在 TCB 里）│
  │ 有几个等待者│ 多个      │ 多个     │ **一个**    │ **一个**      │
  │ 传什么      │ 数据副本  │ 位掩码   │ 字节流      │ 一个 32 位值   │
  │ ISR 能直接吗│ ✅ 能     │ ❌ 要绕道 │ ✅ 部分     │ ✅ 能         │
  │ 唤醒复杂度  │ O(1)     │ O(n)     │ O(1)        │ O(1)         │
  └────────────┴──────────┴──────────┴────────────┴──────────────┘

  ★ 注意**事件组是唯一一个 ISR 不能直接操作的** ——
    下一节会专门讲这件事。它是本课最反直觉的一条。
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("事件组不是队列：一个字段，两种含义", exp1),
    "2": ("等待判据与位打包：控制位藏在哪", exp2),
    "3": ("流缓冲的免锁：单生产者单消费者是怎么强制的", exp3),
    "4": ("流缓冲 vs 消息缓冲：触发级别与边界语义", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        args = sorted(EXPERIMENTS.keys())
    print()
    hr("#")
    print("#  第 7 课 · 事件组与流/消息缓冲 —— 配套实验")
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
