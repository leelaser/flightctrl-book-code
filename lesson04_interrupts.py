#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 4 课配套实验：中断、临界区与 FromISR

设计原则（延续三条纪律）：
  1. 每个论断都能落到 文件:行号
  2. [未亲验] 是诚实的标记，不是免责声明
  3. 模拟的地方明说丢掉了什么

运行：
    python lesson04_interrupts.py         # 全部
    python lesson04_interrupts.py 2       # 只跑实验 2

源码来源：D:/DEVLib/PX4APM/FreeRTOS/   （V11.1.0）
"""

import os
import _srcdirs
import sys

FREERTOS_ROOT = _srcdirs.src("FREERTOS")
PORT_DIR = os.path.join(FREERTOS_ROOT, "portable", "GCC", "ARM_CM7", "r0p1")


def sec(title, ch="=", width=78):
    print()
    print(ch * width)
    print(title)
    print(ch * width)


def read_source(relpath):
    p = os.path.join(FREERTOS_ROOT, relpath)
    if not os.path.isfile(p):
        return None
    import io
    return io.open(p, encoding="utf-8", errors="ignore").read()


# ============================================================================
# 实验 1：临界区的嵌套与 BASEPRI 的状态机
# ============================================================================
#
# 模拟 uxCriticalNesting + BASEPRI 的配合（port.c:462-489）。
#
# 关键点：**只有 nesting 回到 0 才真正恢复中断。**
#   如果内层退出时就恢复 BASEPRI，外层临界区就形同虚设。


class CriticalSection:
    """模拟 vPortEnterCritical / vPortExitCritical。

    BASEPRI 语义：
        0            = 不屏蔽任何中断
        MAX_SYSCALL  = 屏蔽优先级 >= MAX_SYSCALL 的中断（阈值）
    """

    def __init__(self, max_syscall_prio):
        self.max_syscall = max_syscall_prio
        self.basepri = 0              # 当前阈值（0 = 不屏蔽）
        self.nesting = 0xaaaaaaaa     # ★ 毒值（port.c:155）
        self.initialized = False
        self.log = []
        # 统计"实际写 BASEPRI 的次数" —— 用来展示内层不写
        self.n_basepri_writes = 0

    def scheduler_started(self):
        """port.c:431 —— 调度器启动时才把毒值换成 0"""
        self.nesting = 0
        self.initialized = True
        self.log.append("调度器启动：uxCriticalNesting = 0（毒值 0xaaaaaaaa 被替换）")

    def enter(self, note=""):
        """vPortEnterCritical（port.c:462）"""
        if not self.initialized:
            self.log.append(
                "⚠ enter()：uxCriticalNesting 还是毒值 0x%x —— 说明在调度器启动前用了临界区"
                % self.nesting)
        self.basepri = self.max_syscall
        self.n_basepri_writes += 1
        self.nesting += 1
        self.log.append("enter%s：BASEPRI=0x%02x，nesting=%d"
                        % (("(%s)" % note) if note else "",
                           self.basepri, self.nesting))
        # port.c:473 —— 只在 nesting == 1 时做 VECTACTIVE 断言
        if self.nesting == 1:
            self.log.append("        ↳ nesting==1 ⇒ 做 VECTACTIVE 断言（检查是否在 ISR 里）")

    def exit(self, note=""):
        """vPortExitCritical（port.c:482-489）"""
        assert self.nesting != 0, "configASSERT(uxCriticalNesting) 会在这里失败"
        self.nesting -= 1
        if self.nesting == 0:
            self.basepri = 0
            self.n_basepri_writes += 1
            self.log.append("exit%s：nesting=0 ⇒ BASEPRI=0（真正恢复中断）"
                            % (("(%s)" % note) if note else ""))
        else:
            self.log.append("exit%s：nesting=%d ⇒ **不动 BASEPRI**（外层还在临界区里）"
                            % (("(%s)" % note) if note else "", self.nesting))


def exp1_nesting():
    sec("实验 1：临界区的嵌套 —— 为什么需要 uxCriticalNesting")

    print("""
  port.c:462-489 的两个函数，核心是 uxCriticalNesting 这个计数器。

  **它存在的唯一理由是：支持"临界区里再进临界区"。**

  如果没有这个计数器，内层退出时会直接恢复 BASEPRI，
  外层临界区就形同虚设。本实验演示这一点。
""")

    cs = CriticalSection(0x50)   # 假设 configMAX_SYSCALL_INTERRUPT_PRIORITY = 0x50
    cs.scheduler_started()
    cs.log = []

    print("  " + "-" * 74)
    print("  场景：嵌套两层临界区")
    print("  " + "-" * 74)
    print()
    cs.enter("外层")
    cs.enter("内层")
    cs.exit("内层")
    cs.exit("外层")
    for line in cs.log:
        print("      %s" % line)
    print()
    print("  ⇒ **实际写 BASEPRI 只发生了 2 次**（enter 外层 + exit 外层）")
    print("     内层的 enter/exit 只动 nesting，不动 BASEPRI")
    print("     实测写入次数 = %d" % cs.n_basepri_writes)

    print()
    print("  " + "-" * 74)
    print("  反例：如果没有 nesting 计数器会怎样")
    print("  " + "-" * 74)
    print("""
      假设 vPortExitCritical 直接恢复 BASEPRI：

          外层 enter  → BASEPRI = 0x50   ← 临界区开始
          内层 enter  → BASEPRI = 0x50   （没变）
          内层 exit   → BASEPRI = 0      ← ★ 提前恢复了！
          外层 ...    → **中断可以打进来了，但外层以为自己在临界区里**

      ⇒ 这会破坏外层的原子性，而且**没有任何报错** ——
        表现为偶发的数据损坏。

  ================== 那个 VECTACTIVE 断言 ==================
""")

    print("""      port.c:471-475（原文）：

          /* This is not the interrupt safe version of the enter critical function so
           * assert() if it is being called from an interrupt context.  Only API
           * functions that end in "FromISR" can be used in an interrupt.  Only assert if
           * the critical nesting count is 1 to protect against recursive calls if the
           * assert function also uses a critical section. */
          if( uxCriticalNesting == 1 )
          {
              configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
          }

      **读一个寄存器就能判断"我现在在不在 ISR 里"**：

          portNVIC_INT_CTRL_REG = SCB->ICSR (0xE000ED04)
          portVECTACTIVE_MASK   = 0xFF
          ⇒ ICSR 低 9 位是 VECTACTIVE = 【当前正在执行的异常号】
          ⇒ VECTACTIVE == 0 表示"在 thread mode"（不在任何异常里）

      ⇒ 在 ISR 里调非 FromISR 的 API ⇒ VECTACTIVE != 0 ⇒ 断言失败。

      **这是 Cortex-M 特有的能力**（NVIC 硬件记录当前异常号）。
      其他架构上要自己维护一个"中断嵌套计数"。

  ================== 那句"只在 nesting==1 时断言" ==================

           * Only assert if the critical nesting count is 1 to protect against
           * recursive calls if the assert function also uses a critical section.

      推理链：

          如果 configASSERT 的实现本身用了临界区（比如要加锁才能打印）
              → 断言失败时再进一次临界区 → nesting 变成 2
              → 又触发断言 → 又是 nesting == 2 → ... 无限递归

          所以只在 nesting == 1 时断言
              → 断言内部再进临界区时 nesting 已是 2，不再断言
              → 递归被打断

      ⇒ **写断言时要想"断言失败的处理路径会不会再次触发同一个断言"。**
        会的话，加条件把它限制在"第一次"。
""")

    # 演示毒值
    print("  " + "-" * 74)
    print("  毒值：uxCriticalNesting = 0xaaaaaaaa")
    print("  " + "-" * 74)
    print()
    cs2 = CriticalSection(0x50)
    print("      port.c:155  static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;")
    print("      port.c:431  uxCriticalNesting = 0;   ← 调度器启动时才归零")
    print()
    print("      如果在这之前误用了临界区：")
    cs2.enter("启动前误用")
    for line in cs2.log:
        print("          %s" % line)
    print()
    print("      ⇒ nesting 从 0xaaaaaaaa 变成 0xaaaaaaab")
    print("         **永远不会回到 0** ⇒ exit 时永远不会恢复 BASEPRI")
    print("         ⇒ 中断再也开不回来（如果 configASSERT 关了，就是静默死机）")
    print()
    print("      ⇒ 这就是「毒值」的作用：**让「未初始化就使用」变成可检测的异常**，")
    print("        而不是从 0 开始的静默错误。")


# ============================================================================
# 实验 2：configMAX_SYSCALL_INTERRUPT_PRIORITY 的决策
# ============================================================================
#
# 这个实验是个**交互式的配置检查器**：
# 给定一组中断（优先级 + 是否需要调 API），
# 判断配置是否合法，并给出建议的阈值。

def exp2_priority_threshold():
    sec("实验 2：configMAX_SYSCALL_INTERRUPT_PRIORITY 的配置检查")

    print("""
  这个宏是一条【分界线】，不是"某个中断的优先级"：

      NVIC 优先级（数值小 = 逻辑优先级高）
      ┌─────────────────────────────────────────────────┐
      │  0  ┐                                           │
      │     │  ⚠ 高于阈值 —— 不允许调用任何 FreeRTOS API │
      │ ..  │      （包括 FromISR 版本！）               │
      ├─────┤  ← configMAX_SYSCALL_INTERRUPT_PRIORITY   │
      │ ..  ┐                                           │
      │     │  ✅ 低于/等于阈值 —— 可以调用 FromISR API  │
      │ 15  ┘                                           │
      └─────────────────────────────────────────────────┘
""")

    # 一个典型的飞控中断配置
    interrupts = [
        ("DShot 位定时",        2,  False, "需要极低延迟，不用 API"),
        ("电机换相",            1,  False, "同上"),
        ("IMU 数据就绪",        6,  True,  "要唤醒采样线程"),
        ("串口 RX (MAVLink)",   7,  True,  "要唤醒接收线程"),
        ("CAN RX",              8,  True,  "要唤醒处理线程"),
        ("SD 卡 DMA",          9,  True,  "要唤醒日志线程"),
        ("SysTick / PendSV",   15,  False, "内核自己用，最低优先级"),
    ]

    print("  " + "-" * 78)
    print("  场景：一块飞控的中断配置")
    print("  " + "-" * 78)
    print()
    print("  %-22s %-10s %-12s %s" % ("中断", "NVIC 优先级", "要调 API?", "说明"))
    print("  " + "-" * 78)
    for name, prio, uses_api, note in interrupts:
        print("  %-22s %-10d %-12s %s"
              % (name, prio, "是" if uses_api else "否", note))

    need_api = [p for _, p, u, _ in interrupts if u]
    no_api = [p for _, p, u, _ in interrupts if not u]
    print()
    print("      需要调 API 的中断优先级：%s" % need_api)
    print("      不需要的中断优先级：    %s" % no_api)

    # 合法的阈值 = 严格小于 min(need_api)，且大于 0
    min_api = min(need_api)
    max_no_api = max(no_api) if no_api else -1
    print()
    print("  " + "-" * 78)
    print("  求合法的阈值范围")
    print("  " + "-" * 78)
    print()
    print("      **先明确语义**（这是最容易搞反的地方）：")
    print()
    print("          BASEPRI = T  的效果是：屏蔽【优先级数值 >= T】的中断")
    print("          而 NVIC 里【数值小 = 逻辑优先级高】")
    print("          ⇒ BASEPRI = T 屏蔽的是【逻辑优先级不高于 T 的】中断")
    print("          ⇒ 数值 < T 的高优先级中断【照样能跑】")
    print()
    print("      **所以要调 API 的中断，必须满足 数值 >= 阈值**")
    print("      （这样它才会被临界区屏蔽住，不会打断内核数据结构）")
    print()
    print("      本场景里需要调 API 的中断，数值最小的是 %d" % min_api)
    print("      ⇒ 阈值必须 <= %d" % min_api)
    print("      ⇒ 阈值最大可以取到 %d" % min_api)
    print()
    print("  " + "-" * 78)
    print("  ⚠ 方向不要搞反：阈值的两个效果是【互相拉扯】的")
    print("  " + "-" * 78)
    print()
    print("      阈值【越小】：")
    print("          · 越多中断满足 数值 >= 阈值  ⇒ **越多中断能用 API**")
    print("          · 越少中断满足 数值 <  阈值  ⇒ 越少中断能免临界区延迟")
    print()
    print("      阈值【越大】：")
    print("          · 越少中断能用 API")
    print("          · 越多中断能免临界区延迟")

    print()
    print("  " + "=" * 78)
    print("  三个候选阈值对比")
    print("  " + "=" * 78)
    # 注意：不用多列表格 —— 中文字符是双宽的，用 %-Ns 对齐会错位。
    # 改成"每个阈值一块"的形式，可读性更好。
    for thr in (min_api, min_api + 2, min_api + 5):
        can_api = [n for n, p, u, _ in interrupts if u and p >= thr]
        free = [n for n, p, u, _ in interrupts if p < thr]
        bad = [n for n, p, u, _ in interrupts if u and p < thr]
        print()
        print("  ┌─ 阈值 = %d " % thr + "─" * 55)
        print("  │")
        print("  │  能用 FromISR API 的中断（优先级 >= %d）：" % thr)
        print("  │      %s" % ("、".join(can_api) if can_api else "（无）"))
        print("  │")
        print("  │  能免于临界区延迟的中断（优先级 < %d）：" % thr)
        print("  │      %s" % ("、".join(free) if free else "（无）"))
        if bad:
            print("  │")
            print("  │  ⚠ **非法配置**：以下中断要调 API 但优先级高于阈值：")
            print("  │        %s" % "、".join(bad))
        print("  └" + "─" * 62)

    print("""
  ================== 两个经典配置错误 ==================

  错误 1：中断优先级留在默认值 0

      port.c:864-869 的注释（原文）：

          * Interrupts that use the FreeRTOS API must not be left at their
          * default priority of zero as that is the highest possible priority,
          * which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
          * and therefore also guaranteed to be invalid.

      **NVIC 中断优先级的复位默认值就是 0（最高优先级）。**
      忘了设优先级的中断，如果有调 API，必然非法。

      ⚠ 严重性取决于 configASSERT：
          有效 → 断言失败，立刻停机，容易发现
          为空 → **静默的数据损坏**（高优先级 ISR 打断临界区）

  错误 2：优先级分组不是"全抢占位"

      port.c:884-897 的注释：

          * Priority grouping: The interrupt controller (NVIC) allows the bits
          * that define each interrupt's priority to be split between bits that
          * define the interrupt's pre-emption priority bits and bits that define
          * the interrupt's sub-priority. For simplicity all bits must be defined
          * to be pre-emption priority bits. ...

      **很多厂商 HAL 库会设成非零分组** ⇒ 断言失败。

      注释还提醒：改成分组 0 可能让厂商库行为异常 ——
      两边假设冲突，需要显式决策。

  ================== 建议 ==================
""")

    print("""      从上面的分析可以看出，本场景的**最大合法阈值**是 **%d**：

          数值 <  %d  →  %s（免于临界区延迟，但**不能用 API**）
          数值 >= %d  →  %s（能用 FromISR API）

      **阈值取 %d 是"不牺牲任何 API 用户"的极限**（再大就有 API 用户非法）。

      ⇒ 一般规则：
            阈值 = min(所有需要调 API 的中断的数值)

          这是"让尽可能多的中断免于临界区延迟"的取法。

      ⚠ 但**不是所有场景都该这么取**。考虑下面两种极端：

          取一个很小的阈值（比如 1）：
              ⇒ 几乎所有中断都能用 API（灵活）
              ⇒ 但几乎所有中断都会被临界区延迟（实时性差）

          取一个很大的阈值（接近 15）：
              ⇒ 几乎所有中断都免受临界区延迟（实时性好）
              ⇒ 但只有极少数中断能用 API（内核功能受限）

      ⇒ **这个取舍的判据是**：
            问："哪些中断真的需要调用 FreeRTOS API？"
                只有那些中断需要在阈值以下。
            其余中断都应该放在阈值以上 —— 享受"免延迟"。

        换句话说：**越少用 API 越好**。
        能用"轮询 + 共享内存"解决的中断，就不要用 API。
        （这一点和第 10 课 4.3 节讲的 PX4 驱动形态是一致的 ——
          PX4 的 ISR 只做最小动作 + 置标志，不调任何阻塞 API。）
""" % (min_api, min_api,
       "、".join(n for n, p, u, _ in interrupts if p < min_api) or "（无）",
       min_api,
       "、".join(n for n, p, u, _ in interrupts if u and p >= min_api),
       min_api))


# ============================================================================
# 实验 3：FromISR 家族全清单（从本机源码实测）
# ============================================================================

def exp3_fromisr_list():
    sec("实验 3：FromISR 家族全清单（从本机源码实测）")

    import io
    import re
    hdir = os.path.join(FREERTOS_ROOT, "include")
    if not os.path.isdir(hdir):
        print("  ✗ 未找到源码：%s" % hdir)
        return

    names = set()
    for f in os.listdir(hdir):
        if not f.endswith(".h"):
            continue
        txt = io.open(os.path.join(hdir, f), encoding="utf-8", errors="ignore").read()
        for m in re.finditer(r"\b([xv][A-Za-z]+FromISR)\b", txt):
            names.add(m.group(1))

    names = sorted(names)
    print("  源码根：%s" % hdir)
    print("  实测 FromISR 函数总数：**%d 个**" % len(names))
    print()

    # 按模块分组
    groups = {}
    for n in names:
        m = re.match(r"[xv]([A-Z][a-z]+)", n)
        key = m.group(1) if m else "其它"
        groups.setdefault(key, []).append(n)

    print("  " + "-" * 74)
    print("  按模块分组")
    print("  " + "-" * 74)
    print()
    print("  %-16s %-8s %s" % ("模块", "数量", "函数"))
    print("  " + "-" * 74)
    for key in sorted(groups, key=lambda k: -len(groups[k])):
        lst = groups[key]
        shown = ", ".join(lst[:3])
        if len(lst) > 3:
            shown += ", ..."
        print("  %-16s %-8d %s" % (key, len(lst), shown))

    print()
    print("  " + "-" * 74)
    print("  为什么这么多？")
    print("  " + "-" * 74)
    print("""
      推理链：

          内核 API 需要临界区保护
              ↓
          临界区用 BASEPRI（只屏蔽阈值以下）
              ↓
          阈值以上的中断能打断临界区
              ↓
          那些中断不能调用"会进临界区"的 API
              ↓
          ⇒ 必须提供"不进临界区"的版本 = FromISR 版本

      ⇒ **FromISR 版本的本质不是"功能不同"，而是
        "进入内核数据结构的方式不同"。**

  ================== 一个重要的提醒 ==================

      **清单 ≠ 可用清单。** 上面的 %d 个函数里有一部分是
      【条件编译】的 —— 配置关掉时根本不存在。

      本课实测的一个例子：

          tasks.c:7005   #if ( configNUMBER_OF_CORES > 1 )
              UBaseType_t vTaskEnterCriticalFromISR( void ) { ... }
                          #endif

      ⇒ **SMP 专用。单核 Cortex-M 上它不存在**，
        照着清单去用会链接失败。

      ⇒ 第 2 课的教训在这里再次成立：
        **看到名字要问"它在什么条件下才存在"。**
""" % len(names))


# ============================================================================
# 实验 4：断言 + 死循环 —— 一个可以抄的手法
# ============================================================================

def exp4_assert_loop():
    sec("实验 4：断言 + 死循环 —— 一个可以抄的手法")

    print("""
  FreeRTOS 里有好几处"永远为假的断言"，本实验把它们列出来，
  并说明这个手法的价值。
""")

    cases = []

    # 从源码实测这些断言
    port_c = read_source(os.path.join("portable", "GCC", "ARM_CM7", "r0p1", "port.c"))
    tasks_c = read_source("tasks.c")
    queue_c = read_source("queue.c")

    checks = [
        ("prvTaskExitError", "portable/GCC/ARM_CM7/r0p1/port.c",
         port_c, r"configASSERT\(\s*uxCriticalNesting == ~0UL\s*\)",
         "任务函数返回了（应该调 vTaskDelete）"),
        ("vPortEndScheduler", "portable/GCC/ARM_CM7/r0p1/port.c",
         port_c, r"configASSERT\(\s*uxCriticalNesting == 1000UL\s*\)",
         "调用了未实现的 vPortEndScheduler"),
        ("vPortEnterCritical", "portable/GCC/ARM_CM7/r0p1/port.c",
         port_c,
         # 源码是 "configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );"
         # 内部括号两侧有空格 —— 正则要容忍空白
         r"configASSERT\(\s*\(\s*portNVIC_INT_CTRL_REG\s*&\s*portVECTACTIVE_MASK\s*\)\s*==\s*0",
         "在 ISR 里调了非 FromISR 的 API"),
        ("vPortExitCritical", "portable/GCC/ARM_CM7/r0p1/port.c",
         port_c, r"configASSERT\(\s*uxCriticalNesting\s*\)",
         "退出临界区次数多于进入"),
        ("xQueueGenericSend", "queue.c",
         queue_c, r"configASSERT\(\s*!\(\s*\(\s*xCopyPosition == queueOVERWRITE",
         "在长度 != 1 的队列上用 overwrite"),
        ("vTaskPlaceOnEventList", "tasks.c",
         tasks_c, r"configASSERT\(\s*pxEventList\s*\)",
         "传了空的事件链表指针"),
    ]

    print("  " + "-" * 86)
    print("  实测的断言清单")
    print("  " + "-" * 86)
    print()
    print("  %-24s %-14s %-8s %s" % ("函数", "文件", "在源码里", "防的是什么"))
    print("  " + "-" * 86)

    import re
    for name, path, txt, pattern, desc in checks:
        if txt is None:
            found = "?"
        else:
            found = "✓" if re.search(pattern, txt) else "✗"
            if found == "✗":
                found = "（未匹配）"
        print("  %-24s %-14s %-8s %s" % (name, os.path.basename(path), found, desc))

    print()
    print("  " + "=" * 78)
    print("  手法：断言 + 死循环")
    print("  " + "=" * 78)
    print("""
  port.c:230-245 的 prvTaskExitError 是这个手法的完整形态：

      /* A function that implements a task must not exit or attempt to return to
       * its caller as there is nothing to return to.  If a task wants to exit it
       * should instead call vTaskDelete( NULL ).
       *
       * Artificially force an assert() to be triggered if configASSERT() is
       * defined, then stop here so application writers can catch the error. */
      configASSERT( uxCriticalNesting == ~0UL );       ← ① 永远为假
      portDISABLE_INTERRUPTS();                       ← ② 关中断
      while( ulDummy == 0 )                           ← ③ 死循环
      {
          /* ... */
      }

  **三层防护，各司其职：**

      ① 断言     —— configASSERT 有效时：立刻停机（最好）
      ② 关中断   —— 防止死循环期间被中断打扰
      ③ 死循环   —— configASSERT 为空时：**把执行流钉住**

  ================== 为什么这比"直接 return"好 ==================

  | 做法                  | configASSERT 有效 | configASSERT 为空      |
  |-----------------------|-------------------|-----------------------|
  | `return;`             | 静默继续 ⚠        | 静默继续 ⚠            |
  | 只有 configASSERT     | 立刻停机 ✅       | **什么都不做** ⚠      |
  | **断言 + 死循环**     | 立刻停机 ✅       | **卡住，调试器可见** ✅ |

  → **第三种在生产环境出错时给出最好的诊断信息。**

  特别是对嵌入式：固件跑飞之后往往要很久才表现出症状，
  而"PC 停在一个明确的地址"能让人立刻定位。

  ================== 适用场景 ==================

      ✅ 适合：
          · "不应该发生"的路径（任务函数返回、调度器结束）
          · 参数校验（但要注意开销 —— 热路径上别放）
          · 状态机的不可能状态

      ❌ 不适合：
          · 可能发生的错误（内存分配失败、通信超时）
            —— 这些要返回错误码，不能停机
          · 中断处理程序里（死循环会锁死系统）

  ⇒ **判据：这个情况如果发生了，程序还有意义继续跑吗？**
     没有 → 断言 + 死循环。
     有   → 返回错误码。
""")


# ============================================================================
# 主入口
# ============================================================================

EXPERIMENTS = {
    "1": ("临界区的嵌套与毒值", exp1_nesting),
    "2": ("configMAX_SYSCALL_INTERRUPT_PRIORITY 配置检查", exp2_priority_threshold),
    "3": ("FromISR 家族全清单", exp3_fromisr_list),
    "4": ("断言 + 死循环的手法", exp4_assert_loop),
}


def main():
    print("""
================================================================================
 第 4 课：中断、临界区与 FromISR
 配套实验代码

 实验列表：
   1  临界区的嵌套与毒值 —— 为什么需要 uxCriticalNesting
   2  configMAX_SYSCALL_INTERRUPT_PRIORITY 配置检查器
   3  FromISR 家族全清单（从本机源码实测）
   4  断言 + 死循环 —— 一个可以抄的手法
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

1. **临界区的完整实现，把 CM7 勘误规避放在了最热的路径上。**
   portENTER_CRITICAL → vPortRaiseBASEPRI 的汇编里，每一次都有
   cpsid i / cpsie i 包住 msr basepri（勘误 837070）。
   对一个每秒进出临界区几万次的系统，就是几万次不成对的中断关开对。

2. **configMAX_SYSCALL_INTERRUPT_PRIORITY 是一条分界线，
   而它有两个经典配置错误。**
     · 中断优先级留在默认值 0（= 最高）⇒ 必然非法
     · 优先级分组不是全抢占位 ⇒ 断言失败
   两个错误在 configASSERT 关掉后都是**静默的数据损坏**。

3. **FromISR 家族的 52 个函数，根源是 BASEPRI。**
   因为高优先级中断能打断临界区，所以必须提供"不进临界区"的版本。
   ⇒ FromISR 版本的本质不是"功能不同"，
     而是"进入内核数据结构的方式不同"。

 本课最值得抄的手法：**断言 + 死循环**
     configASSERT( 永远为假的条件 );
     for (;;) { }
   configASSERT 被关掉后，死循环仍能把执行流钉在出错点，
   让调试器能发现 —— 而不是让程序带着错误状态跑飞到别处才崩。
================================================================================
""")


if __name__ == "__main__":
    main()
