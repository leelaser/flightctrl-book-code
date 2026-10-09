#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 4 第 6 章配套实验：FreeRTOS 是什么 —— 最小内核哲学

设计原则（延续《NuttX 教程》的三条纪律）：
  1. 每个论断都能落到 文件:行号
  2. [未亲验] 是诚实的标记，不是免责声明
  3. 模拟的地方明说丢掉了什么

运行：
    python lesson01_freertos_intro.py         # 全部
    python lesson01_freertos_intro.py 2       # 只跑实验 2

源码来源（本机可查）：
    $SRC_FREERTOS/     FreeRTOS V11.1.0
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
# 实验 1：Queue_t 一统天下 —— 五种原语的创建路径
# ============================================================================
#
# 本实验把 FreeRTOS 的五种同步原语的创建过程做成一张表，
# 展示它们在**哪一步分叉**。
#
# 所有字段名和取值都来自源码：
#   queue.c:65-66    #define uxQueueType pcHead
#                    #define queueQUEUE_IS_MUTEX NULL
#   queue.c:82       #define queueSEMAPHORE_QUEUE_ITEM_LENGTH ( UBaseType_t ) 0
#   semphr.h:41      #define semSEMAPHORE_QUEUE_ITEM_LENGTH ( ( uint8_t ) 0U )
#   queue.c:614-627  prvInitialiseMutex()
#   queue.c:644-656  xQueueCreateMutex()
#   queue.c:862-884  xQueueCreateCountingSemaphoreStatic()
#   queue.h:149      xQueueCreate 宏
#   semphr.h:167     xSemaphoreCreateBinary 宏

# Queue_t 里与本实验相关的字段（其余字段所有类型都一样，不列）
QUEUE_FIELDS = ["pcHead", "uxMessagesWaiting", "xMutexHolder", "uxRecursiveCallCount"]


class QueueT:
    """模拟一个 Queue_t。

    只保留本实验关心的 4 个字段。真实结构体见 include/queue.c 的
    struct QueueDefinition（约 20 个字段）。
    丢掉的东西：(a) 阻塞任务链表 xTasksWaitingToSend/Receive
               (b) 通知相关字段 ucNotifyState/ulNotifiedValue
               (c) 存储区相关 pcTail/pcReadFrom/pcWriteTo
               (d) DTrace/trace 钩子字段
    这些对"创建路径在哪分叉"这个结论没有影响。
    """

    def __init__(self, name, length, item_size, qtype):
        self.name = name
        self.length = length
        self.item_size = item_size
        self.qtype = qtype
        # xQueueGenericCreate 的公共初始化
        # 关键：item_size 为 0 时不分配存储区 ⇒ pcHead 保持 NULL
        self.pcHead = None if item_size == 0 else ("buf@%s" % name)
        self.uxMessagesWaiting = 0
        self.xMutexHolder = None
        self.uxRecursiveCallCount = 0
        self.extra_steps = []      # 记录"创建之后补了哪些初始化"

    def is_mutex(self):
        """queue.c:66  #define queueQUEUE_IS_MUTEX NULL
        而 uxQueueType 是 pcHead 的别名 ⇒ 判断就是 pcHead is None。
        注意：这只在 item_size == 0 时成立，即所有信号量类原语。"""
        return self.pcHead is None

    def snapshot(self):
        return {
            "pcHead": "NULL" if self.pcHead is None else "非 NULL",
            "uxMessagesWaiting": self.uxMessagesWaiting,
            "xMutexHolder": "NULL" if self.xMutexHolder is None else "非 NULL",
            "uxRecursiveCallCount": self.uxRecursiveCallCount,
        }


def create_queue(name, length, item_size):
    """xQueueCreate(len, size) → xQueueGenericCreate(len, size, BASE)"""
    return QueueT(name, length, item_size, "BASE")


def create_binary_sem(name):
    """semphr.h:167
    xSemaphoreCreateBinary() → xQueueGenericCreate(1, 0, BINARY_SEMAPHORE)
    注意：**没有**创建后的额外初始化。
    """
    return QueueT(name, 1, 0, "BINARY_SEMAPHORE")


def create_counting_sem(name, max_count, initial_count):
    """queue.c:873,877
    xQueueGenericCreateStatic(n, 0, NULL, buf, COUNTING_SEMAPHORE)
    然后 **手动**设置 uxMessagesWaiting（而不是通过队列 send）
    """
    q = QueueT(name, max_count, 0, "COUNTING_SEMAPHORE")
    q.uxMessagesWaiting = initial_count
    q.extra_steps.append("uxMessagesWaiting = %d" % initial_count)
    return q


def create_mutex(name, recursive=False):
    """queue.c:644-656
    xQueueCreateMutex():
        uxMutexLength = 1, uxMutexSize = 0
        xNewQueue = xQueueGenericCreate(uxMutexLength, uxMutexSize, ucQueueType);
        prvInitialiseMutex( ( Queue_t * ) xNewQueue );     ← ★ 唯一的分叉点
    """
    q = QueueT(name, 1, 0, "RECURSIVE_MUTEX" if recursive else "MUTEX")
    # prvInitialiseMutex 的三件事（queue.c:614-627）
    q.xMutexHolder = None                    # ← PI 的依托，**只有这里设置**
    q.uxRecursiveCallCount = 0               # ← 递归计数
    # 第三件事是 uxQueueType = queueQUEUE_IS_MUTEX
    # 但 uxQueueType 是 pcHead 的别名，而 pcHead 本来就是 NULL，
    # 所以严格说这一步是"再确认一次"。这也是这个技巧能成立的原因。
    q.extra_steps.append("xMutexHolder = NULL        (PI 的依托)")
    q.extra_steps.append("uxQueueType = NULL         (= pcHead，本来就是 NULL)")
    q.extra_steps.append("uxRecursiveCallCount = 0")
    return q


def exp1_queue_unified():
    sec("实验 1：Queue_t 一统天下 —— 五种原语的创建路径")

    print("""
  FreeRTOS 有一句流传很广的描述："信号量、互斥量、事件组都是队列"。
  方向是对的，但太含糊。本实验把它精确到"在哪一步分叉"。
""")

    objs = [
        create_queue("msgq", 10, 32),
        create_binary_sem("bsem"),
        create_counting_sem("csem", 5, 3),
        create_mutex("mtx"),
        create_mutex("rmtx", recursive=True),
    ]

    print("  " + "-" * 94)
    print("  创建后的状态（字段名来自 include/queue.c 的 struct QueueDefinition）")
    print("  " + "-" * 94)
    print()
    print("  %-18s %-20s %-10s %-8s %-12s %-10s"
          % ("名字", "类型", "pcHead", "count", "xMutexHolder", "递归计数"))
    print("  " + "-" * 94)
    for q in objs:
        s = q.snapshot()
        print("  %-18s %-20s %-10s %-8s %-12s %-10s"
              % (q.name, q.qtype, s["pcHead"], s["uxMessagesWaiting"],
                 s["xMutexHolder"], s["uxRecursiveCallCount"]))

    print()
    print("  " + "-" * 94)
    print("  哪些对象有「创建后的额外初始化」？")
    print("  " + "-" * 94)
    print()
    for q in objs:
        if q.extra_steps:
            print("  ● %s (%s)：" % (q.name, q.qtype))
            for s in q.extra_steps:
                print("        %s" % s)
        else:
            print("  ○ %s (%s)：**没有任何额外初始化**" % (q.name, q.qtype))

    print("""
  ================== 三个关键事实 ==================

  【事实 1】除了消息队列，所有原语的元素大小都是 0
            ⇒ **信号量不使用队列的存储区**
            ⇒ 所以 pcHead 为 NULL
            ⇒ 所以 `uxQueueType == queueQUEUE_IS_MUTEX`（NULL）能用 pcHead 表示

  【事实 2】只有互斥量需要「创建后再补几笔」
            这一行就是全部分叉：
                queue.c:652   prvInitialiseMutex( ( Queue_t * ) xNewQueue );

  【事实 3】优先级继承就在那几笔里，别处没有
            xMutexHolder 只在 prvInitialiseMutex() 里被设置（queue.c:620），
            而 PI 的两个动作都检查 uxQueueType 且依赖 xMutexHolder：
                queue.c:1784-1788   xTaskPriorityInherit( pxQueue->u.xSemaphore.xMutexHolder )
                queue.c:2405-2408   xTaskPriorityDisinherit( pxQueue->u.xSemaphore.xMutexHolder )
""")

    # 演示二值信号量当锁的后果
    print("  " + "-" * 94)
    print("  演示：用二值信号量当锁，PI 会发生什么？")
    print("  " + "-" * 94)
    print()
    bsem = create_binary_sem("my_lock (误用)")
    mtx = create_mutex("my_lock (正确)")
    print("  %-24s %-30s %s" % ("", "pcHead", "PI 的 if 判断会通过吗"))
    print("  " + "-" * 78)
    for q in (bsem, mtx):
        # 模拟 queue.c:1784 的 if 条件
        passes = q.is_mutex()
        print("  %-24s %-30s %s"
              % (q.name,
                 "NULL" if q.pcHead is None else "非 NULL",
                 "✓ 会 → 走 PI 逻辑" if passes
                 else "✗ 不会 → **静默跳过 PI**"))
    print("""
  ⇒ 两个对象的句柄类型完全相同（SemaphoreHandle_t），
    **编译器不会给任何提示**。这就是"用二值信号量当锁"这个坑的
    全部机制 —— 它不报错，只是在需要 PI 的时候悄悄不做。

  卷 5 第 1 章把这条结论标为 [未亲验]，现在有完整源码证据链了。
""")


# ============================================================================
# 实验 2：三层默认值解析器 ⭐
# ============================================================================
#
# 这是本章最有价值的一个实验。
#
# FreeRTOS 的配置项默认值有【三层】，优先级由 include 顺序决定：
#   第一层  FreeRTOSConfig.h       （用户）     FreeRTOS.h:64  最先
#   第二层  portmacro.h            （移植层）   FreeRTOS.h:107 → portable.h:53
#   第三层  FreeRTOS.h 末尾兜底    （内核）
#
# 本实验实现一个 mini 的"#ifndef/#define 序列求值器"，
# 喂进真实的宏定义，展示最终值是怎么被决定的。


class PreprocResolver:
    """模拟 C 预处理里 #ifndef / #define 的求值。

    只支持本实验需要的子集：
        #ifndef NAME
        #define NAME VALUE
        #endif
        /* 注释 */

    丢掉的东西：
      · #if / #elif / #else 等条件分支
      · 宏展开（只记录字面值）
      · #undef
    对"三层默认值"这个结论没有影响 —— 因为 FreeRTOS 的三层
    用的就是纯粹的 #ifndef/#define。
    """

    def __init__(self):
        self.defined = {}        # name -> (value, layer_name)
        self.trace = []          # (layer, name, action, detail)

    def feed(self, text, layer):
        """按顺序处理一段 #ifndef/#define 序列。"""
        lines = text.split("\n")
        i = 0
        while i < len(lines):
            line = lines[i].strip()

            m = re.match(r'#\s*ifndef\s+(\w+)', line)
            if m:
                name = m.group(1)
                # 找配对的 #define（跳过注释行）
                j = i + 1
                value = None
                while j < len(lines):
                    s = lines[j].strip()
                    if s.startswith("#endif"):
                        break
                    md = re.match(r'#\s*define\s+(\w+)\s+(.*)$', s)
                    if md and md.group(1) == name:
                        value = md.group(2).strip()
                        break
                    j += 1

                if name in self.defined:
                    prev_val, prev_layer = self.defined[name]
                    self.trace.append(
                        (layer, name, "跳过",
                         "已由 [%s] 定义为 %s" % (prev_layer, prev_val)))
                else:
                    if value is None:
                        self.trace.append((layer, name, "跳过", "块内无 #define"))
                    else:
                        self.defined[name] = (value, layer)
                        self.trace.append(
                            (layer, name, "定义", "-> %s  ← 生效" % value))
                i = j + 1
                continue
            i += 1

    def value(self, name):
        return self.defined.get(name)


def exp2_three_layers():
    sec("实验 2：三层默认值解析器 —— 一个真实的读代码陷阱")

    print("""
  拿 configUSE_PORT_OPTIMISED_TASK_SELECTION 这个宏做例子。
  它控制"用 CLZ 指令一条指令找最高优先级"这个优化。

  本实验喂进**真实的**三段源码文本（行号是实测的），
  看最终值是怎么被决定的。
""")

    # --- 真实的源码文本 ---
    LAYER1_USER = """
/* FreeRTOSConfig.h —— 用户配置。这是【第一层】，FreeRTOS.h:64 最先 include。
   下面留空表示"用户没定义"。 */
"""

    LAYER2_PORT = """
/* portable/GCC/ARM_CM7/r0p1/portmacro.h:140-143
   这是【第二层】，通过 FreeRTOS.h:107 → portable.h:53 引入。

   注意它在 FreeRTOS.h:2767 的兜底之前！
   （include 顺序：FreeRTOS.h:64 用户 → :107 portable.h → :2767 兜底） */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION    1
#endif
"""

    LAYER3_KERNEL = """
/* include/FreeRTOS.h:2767-2769 —— 【第三层】内核兜底 */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION    0
#endif
"""

    # --- 场景 1 ---
    print("  " + "=" * 74)
    print("  场景 1：用户什么都不定义（Cortex-M7 上的常见情况）")
    print("  " + "=" * 74)
    r = PreprocResolver()
    r.feed(LAYER1_USER, "第一层 用户配置")
    r.feed(LAYER2_PORT, "第二层 port(CM7)")
    r.feed(LAYER3_KERNEL, "第三层 内核兜底")
    _dump_trace(r)
    v = r.value("configUSE_PORT_OPTIMISED_TASK_SELECTION")
    print()
    print("  ⇒ 最终值 = %s" % v[0])
    print("     结论：**CLZ 优化是开着的！**")
    print()
    print("     但如果只看 FreeRTOS.h:2767 那个 #define ... 0，")
    print("     你会得出完全相反的结论。这就是这个陷阱。")

    # --- 场景 2 ---
    print()
    print("  " + "=" * 74)
    print("  场景 2：用户在 FreeRTOSConfig.h 里显式关掉")
    print("  " + "=" * 74)
    r = PreprocResolver()
    user = """
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION    0
#endif
"""
    r.feed(user, "第一层 用户配置")
    r.feed(LAYER2_PORT, "第二层 port(CM7)")
    r.feed(LAYER3_KERNEL, "第三层 内核兜底")
    _dump_trace(r)
    v = r.value("configUSE_PORT_OPTIMISED_TASK_SELECTION")
    print()
    print("  ⇒ 最终值 = %s（用户主动关掉了，后两层被跳过）" % v[0])

    # --- 场景 3 ---
    print()
    print("  " + "=" * 74)
    print("  场景 3：某个 port 没有第二层（比如一个简单的旧移植层）")
    print("  " + "=" * 74)
    r = PreprocResolver()
    r.feed(LAYER1_USER, "第一层 用户配置")
    r.feed("/* 这个 port 的 portmacro.h 里没有这一项 */\n", "第二层 port(无)")
    r.feed(LAYER3_KERNEL, "第三层 内核兜底")
    _dump_trace(r)
    v = r.value("configUSE_PORT_OPTIMISED_TASK_SELECTION")
    print()
    print("  ⇒ 最终值 = %s（兜底生效）" % v[0])
    print("     注意：兜底值是 **0**（最保守），而 CM7 的 port 值是 **1**。")
    print("     这正说明三层机制的意义 —— port 知道自己的芯片能不能用 CLZ，")
    print("     内核不知道，只能用最保守的值兜底。")

    # --- 一般化 ---
    print()
    print("  " + "=" * 74)
    print("  这个陷阱的一般形式")
    print("  " + "=" * 74)
    print("""
      **grep 一个配置项的「默认值」时，如果只找到一个
        `#ifndef ... #define ... X`，不要立刻下结论。**

      可能有另一个地方（在更早的 include 里）也定义了它。

      对 FreeRTOS 来说，要按这个顺序找：

          ┌──────────────────────────────────────────────────────┐
          │ 1. 用户的 FreeRTOSConfig.h          （最高优先级）    │
          │ 2. portable/<port>/portmacro.h      （移植层默认）    │
          │ 3. include/FreeRTOS.h 末尾          （内核兜底）      │
          └──────────────────────────────────────────────────────┘

      对 NuttX 来说不存在这个问题 —— Kconfig 引擎会算出最终值，
      menuconfig 里直接显示。这是声明式配置系统的一个真实优势。

  ================== 一个反例：哪些项没有第二层 ==================

      不是所有配置项都有三层。比如 configMAX_PRIORITIES：

          include/FreeRTOS.h:165-167
              #ifndef configMAX_PRIORITIES
                  #error Missing definition:  configMAX_PRIORITIES must be
                         defined in FreeRTOSConfig.h.  ...
              #endif

      **没有默认值，不定义就编译错误。**
      卷 5 第 1 章第一版写的是「demo 里常见 5~7」，把它描述成一个
      有默认值的选项 —— 实际它是一个必须显式做出的决定。

      ⇒ 检查方法：一个配置项是「有默认值」还是「必须定义」，
        看它在 FreeRTOS.h 里是 `#define` 还是 `#error`。
""")


def _dump_trace(r):
    """打印解析轨迹。

    不做列对齐 —— 中文字符是双宽的，用 %-Ns 对齐会错位。
    改成"每层一行、层名后用冒号"的形式，可读性更好。
    """
    print()
    for layer, name, action, detail in r.trace:
        mark = "  ·" if action == "跳过" else "  →"
        print("    %s %s：%s %s" % (mark, layer, action, detail))
    print()


# ============================================================================
# 实验 3：内核规模实测（直接读本机源码）
# ============================================================================

KERNEL_C = ["tasks.c", "queue.c", "stream_buffer.c", "timers.c",
            "event_groups.c", "croutine.c", "list.c"]


def exp3_measure():
    sec("实验 3：内核规模实测（直接读本机 FreeRTOS 源码）")

    if not os.path.isdir(FREERTOS_ROOT):
        print("  未找到 FreeRTOS 源码：%s" % FREERTOS_ROOT)
        print("  请先克隆：")
        print("      git clone --depth 1 --branch V11.1.0 \\")
        print("          https://github.com/FreeRTOS/FreeRTOS-Kernel.git")
        return

    print("  源码根：%s" % FREERTOS_ROOT)
    print()

    ver = "?"
    th = os.path.join(FREERTOS_ROOT, "include", "task.h")
    if os.path.isfile(th):
        for line in io.open(th, encoding="utf-8", errors="ignore"):
            m = re.search(r'tskKERNEL_VERSION_NUMBER\s+"([^"]+)"', line)
            if m:
                ver = m.group(1)
                break
    print("  版本：%s" % ver)
    print()
    print("  " + "-" * 66)
    print("  内核 .c 文件")
    print("  " + "-" * 66)

    total = 0
    rows = []
    for f in KERNEL_C:
        p = os.path.join(FREERTOS_ROOT, f)
        if os.path.isfile(p):
            n = sum(1 for _ in io.open(p, encoding="utf-8", errors="ignore"))
            rows.append((f, n))
            total += n
    for f, n in sorted(rows, key=lambda x: -x[1]):
        print("      %-20s %6d" % (f, n))
    print("      " + "-" * 28)
    print("      %-20s %6d" % ("合计", total))

    # 头文件
    hdr_total = 0
    hdir = os.path.join(FREERTOS_ROOT, "include")
    if os.path.isdir(hdir):
        for f in os.listdir(hdir):
            if f.endswith(".h"):
                hdr_total += sum(1 for _ in io.open(
                    os.path.join(hdir, f), encoding="utf-8", errors="ignore"))
    print()
    print("  内核头文件（include/*.h）：%d 行" % hdr_total)

    # port 层
    print()
    print("  " + "-" * 66)
    print("  Cortex-M7 port（portable/GCC/ARM_CM7/r0p1/）")
    print("  " + "-" * 66)
    pdir = os.path.join(FREERTOS_ROOT, "portable", "GCC", "ARM_CM7", "r0p1")
    ptot = 0
    if os.path.isdir(pdir):
        for f in sorted(os.listdir(pdir)):
            if f.endswith((".c", ".h", ".S")):
                n = sum(1 for _ in io.open(
                    os.path.join(pdir, f), encoding="utf-8", errors="ignore"))
                print("      %-20s %6d" % (f, n))
                ptot += n
    print("      " + "-" * 28)
    print("      %-20s %6d" % ("合计", ptot))

    # 对照 NuttX
    print()
    print("  " + "-" * 66)
    print("  对照《NuttX 教程》卷 6 第 10 章的实测数字")
    print("  " + "-" * 66)
    print("""
      NuttX 的对应部分：
          sched/          52,540 行   ← 调度+同步+IPC+信号+工作队列
          mm/             13,423 行
          fs/             91,719 行
          Cortex-M 移植层 13,531 行
          （全部）     4,308,955 行

      对比：
          FreeRTOS 内核 .c           %6d 行
          FreeRTOS 内核头文件         %6d 行
          FreeRTOS CM7 port           %6d 行
          ────────────────────────────────────
          内核 + port 合计            %6d 行

          NuttX 的 sched/            52,540 行
          ⇒ 比值：%.1fx
""" % (total, hdr_total, ptot, total + ptot, 52540.0 / (total + ptot)))

    print("""      ⚠ 但这个对比**不能单独看**。
        本章 0.3 节讲过：内核行数 ≠ 你的成本。
        NuttX 的 52,540 行里包含了 FreeRTOS 没有的一切
        （VFS 之外的调度增强、工作队列、信号、pthread 兼容层...），
        而 FreeRTOS 省下的那些，要由你来补。
""")


# ============================================================================
# 实验 4：起步成本估算
# ============================================================================

# 一个飞控固件需要的"基础设施"（对照 NuttX 提供的东西）
INFRA = [
    # (能力, NuttX 有吗, FreeRTOS 有吗, 自建量级（行）, 说明)
    ("调度/同步/时间",      True,  True,  0,
     "两家都有"),
    ("内存分配",           True,  True,  300,
     "FreeRTOS 有 heap_1~5，但多区域/属性路由要自己加"),
    ("设备抽象（VFS）",     True,  False, 800,
     "NuttX 有 VFS + register_driver()；FreeRTOS 没有对等物"),
    ("SPI/I2C/UART 驱动",  True,  False, 2500,
     "NuttX 的 drivers/ 有；FreeRTOS 连 HAL 都没有"),
    ("文件系统",           True,  False, 0,
     "可选外挂 FreeRTOS+FAT，不自建"),
    ("TCP/IP",            True,  False, 0,
     "可选外挂 lwIP / FreeRTOS+TCP，不自建"),
    ("shell / 命令行调试",  True,  False, 600,
     "★ 没有对等物，最痛的一项 —— 只能自己写个简易命令解析"),
    ("C 库适配",           True,  False, 200,
     "newlib + 自己写 _sbrk/_write/_read 等桩函数"),
    ("日志系统",           True,  False, 300,
     "自己写环形缓冲 + 落盘"),
]


def exp4_startup_cost():
    sec("实验 4：起步成本估算 —— 选 FreeRTOS 你要自己写多少")

    print("""
  "FreeRTOS 只有 16,653 行"这句话的另一面是：
  它把一堆东西留给了你自己写。

  下面这张表把"一个飞控固件需要的基础设施"列出来，
  对照 NuttX 和 FreeRTOS 各自提供什么，估算自建量级。

  ⚠ 行数是**量级估计**，不是精确测量。它的作用是给出量级感，
    不是给出准确值。参照来源：卷 5 第 1 章 6.4 节实测的
    ArduPilot 在 ChibiOS 之上自建的约 1700 行（ChibiOS 已经
    提供了 HAL，FreeRTOS 连 HAL 都没有）。
""")

    print("  %-22s %-8s %-10s %-12s %s"
          % ("能力", "NuttX", "FreeRTOS", "自建量级", "说明"))
    print("  " + "-" * 100)
    total = 0
    for name, has_nuttx, has_frtos, lines, note in INFRA:
        if has_frtos:
            mark = "—"
        else:
            mark = "%d 行" % lines
            total += lines
        print("  %-22s %-8s %-10s %-12s %s"
              % (name, "✓" if has_nuttx else "—", "✓" if has_frtos else "✗",
                 mark, note))

    print("  " + "-" * 100)
    print("  %-22s %-8s %-10s %-12s" % ("合计自建", "", "", "%d 行" % total))

    print("""
  ================== 怎么读这个数字 ==================

  约 %d 行 —— 这是把 FreeRTOS 补到"一个飞控固件能用"的量级。

  对照：
      FreeRTOS 内核本身                     16,653 行
      你要补的                              ~%d 行
      ────────────────────────────────────────────────
      你实际拥有的"系统"总量                 ~%d 行

  ⇒ 所以"FreeRTOS 比 NuttX 小 60 倍"这个说法，
    **只在"你什么都不补"的前提下成立** —— 而那个前提
    对应的系统跑不了飞控。

  真实的对比应该是：
      用 NuttX    ：写业务逻辑（几千行）
      用 FreeRTOS ：写基础设施（%d 行）+ 业务逻辑（几千行）

  这也是卷 5 第 1 章那句结论的量化版：

      「选 RTOS 的本质是回答一个问题：
        我愿意为多少基础设施负责？」
""" % (total, total, total + 16653, total))

    print("""  ================== 三个例外情况 ==================

  但下面三种情况**选 FreeRTOS 是对的**，不要被上面的数字吓到：

  1. 系统真的很简单
     几个任务 + 几个外设 + 没有文件系统/网络/shell。
     比如一个电调、一个舵机控制器、一个简单的传感器节点。
     → 基础设施需求接近 0，FreeRTOS 的极简就是纯收益。

  2. 你已经有自己的基础设施
     很多公司有自己的驱动库、日志库、参数系统，
     迁移到 FreeRTOS 只是换个调度器。
     → 但要注意：这些库的**可移植性**通常是靠抽象层实现的，
        而抽象层本身就要维护（卷 5 第 1 章 9.5 节讲的耦合问题）。

  3. 你要的是"可验证性"而不是"功能"
     FreeRTOS 有 MISRA 合规版本、有 Safety 认证包
     （FreeRTOS-MPU / SafeRTOS）。在需要认证的场景下，
     **"代码少"本身就是优势** —— 因为要审计的代码少。
     → 这是 NuttX 的 430 万行做不到的事。

  ⇒ 所以完整的判断是：

      **FreeRTOS 适合「基础设施需求小」或「已有基础设施」或
        「需要认证」的场景。
        它不适合「从零搭一个复杂飞控固件」的场景。**
""")


# ============================================================================
# 主入口
# ============================================================================

EXPERIMENTS = {
    "1": ("Queue_t 一统天下", exp1_queue_unified),
    "2": ("三层默认值解析器", exp2_three_layers),
    "3": ("内核规模实测", exp3_measure),
    "4": ("起步成本估算", exp4_startup_cost),
}


def main():
    print("""
================================================================================
 本章：FreeRTOS 是什么 —— 最小内核哲学
 配套实验代码

 实验列表：
   1  Queue_t 一统天下 —— 五种原语在哪一步分叉
   2  三层默认值解析器 —— 一个真实的读代码陷阱 ★
   3  内核规模实测 —— 直接读本机 FreeRTOS 源码
   4  起步成本估算 —— 选 FreeRTOS 你要自己写多少
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

1. **"最小内核"有两层含义，第二层更重要。**
   功能最小：只有 7 个 .c 文件，不提供任何 OS 服务。
   抽象最小：裸 API，不假装是 POSIX —— 这也意味着代码
   **不可移植到别的 RTOS**。

2. **"所有原语都是队列"的精确含义是：**
   它们都是 xQueueGenericCreate() 的返回值，差别只是
   (长度, 元素大小, 类型) 三元组 + 可选的"创建后补笔"。
   **优先级继承只在 prvInitialiseMutex() 里设置**（queue.c:614-627），
   这就是"二值信号量当锁 = 没有 PI"的源码级原因。

3. **三层默认值机制是一个真实的陷阱。**
   `用户配置 > port 默认 > 内核兜底`，优先级由 include 顺序决定。
   CM7 上 configUSE_PORT_OPTIMISED_TASK_SELECTION 实际是 **1**
   （开着），只看 FreeRTOS.h 末尾的 #define ... 0 会得出相反结论。

 配套：本章还用实验 4 量化了"选 FreeRTOS 要自己写多少"（约 %d 行），
 并给出了三个"选它是对的"的例外情况。
================================================================================
""" % sum(x[3] for x in INFRA if not x[2]))


if __name__ == "__main__":
    main()
