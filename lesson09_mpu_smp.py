#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 9 课配套实验：MPU 与 SMP

    实验 1  ★★ 先数：两个飞控用不用这两个特性
    实验 2  ★★ 单核变量 → 按核索引（SMP 的改写方式）
    实验 3  ★  MPU region：8 个区、权限、越界会怎样
    实验 4  ★★ 两种 SMP 调度模型（共享队列 vs 每核实例）
    实验 5  ★★ 成本清单：为什么两个飞控都不用

用法：
    python lesson09_mpu_smp.py 1
    ...
    python lesson09_mpu_smp.py        # 全部

【诚实声明】

实验 1/2/5 是【读源码】—— 结论来自文件内容和可执行的统计。
实验 3/4 是【模拟】—— 模拟的是机制语义，丢掉了：

    · 真实的 MPU 硬件（region 的对齐要求、权限位的编码）
    · 真实的缓存一致性协议（MESI 之类）
    · 自旋锁的实际争用开销、核间中断延迟
    · 编译器与内存序的影响

实验 3/4 证明的是【模型的差别】，不是【性能数据】。

★ 本课的写法有一个背景：第 18 课的预告里我写错了三个论断，
  所以这一课的方法改成**"先数，再写"**（实验 1 就是那个"数"）。

源码版本：FreeRTOS V11.1.0 / ArduPilot 4.5.7 / PX4 1.15.x
"""

import io
import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))
FREERTOS = os.path.join(BASE, "FreeRTOS")
APM_ROOT = os.path.join(BASE, "APM", "ardupilot-Copter-4.5.7",
                        "ardupilot-Copter-4.5.7")
PX4_ROOT = os.path.join(BASE, "PX4", "PX4-Autopilot")
CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")
NUTTX = os.path.join(PX4_ROOT, "platforms", "nuttx", "NuttX", "nuttx")

F_TASKS_C = os.path.join(FREERTOS, "tasks.c")
F_MPU_WRAPPERS = os.path.join(FREERTOS, "include", "mpu_wrappers.h")
F_FREERTOS_H = os.path.join(FREERTOS, "include", "FreeRTOS.h")
F_CHSCHD_C = os.path.join(CHIBIOS, "os", "rt", "src", "chschd.c")
F_CHSYS_C = os.path.join(CHIBIOS, "os", "rt", "src", "chsys.c")
F_AP_FRCONF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                           "hwdef", "common", "FreeRTOSConfig.h")
F_CHCONF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                        "hwdef", "common", "chconf.h")
F_NUTTX_KCONFIG = os.path.join(NUTTX, "arch", "arm", "Kconfig")
F_PX4_BOARDS = os.path.join(PX4_ROOT, "boards")

# ===========================================================================
# 小工具
# ===========================================================================

def read(path):
    try:
        with io.open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def find_line(path, pattern, start=1):
    lines = read(path).split("\n")
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
    p = os.path.normpath(path)
    for root, tag in ((FREERTOS, ""), (APM_ROOT, ""), (PX4_ROOT, "")):
        if p.startswith(root):
            rest = p[len(root) + 1:].replace(os.sep, "/")
            return rest
    return p.replace(os.sep, "/")


def src(path, a, b, note=None):
    lines = read(path).split("\n")
    if note:
        print("      %s" % note)
    for i in range(max(0, a - 1), min(len(lines), b)):
        print("      %s:%d  %s" % (rel(path), i + 1, lines[i].rstrip()[:104]))
    print()


def grep_count(root, pattern, exts=(".c", ".h", ".cpp", ".hpp", ".dat"),
               skip=()):
    """在 root 下数命中 pattern 的文件数。返回 (命中文件数, 命中次数)。

    ★ 必须带 `re.M` —— 否则 `^CONFIG_ARM_MPU=y` 这种带行首锚点的模式
      只会匹配**文件开头**，数字全是 0（作者第一版就是这样，
      于是"PX4 的 MPU 配置"被数成了 0 个 —— 而实际有 3 个）。
    """
    rx = re.compile(pattern, re.M)
    nf = nh = 0
    for dirpath, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d not in ('.git', 'build', '__pycache__')
                   and not any(s in dirpath for s in skip)]
        if any(s in dirpath for s in skip):
            continue
        for f in files:
            if not f.endswith(exts):
                continue
            s = read(os.path.join(dirpath, f))
            h = len(rx.findall(s))
            if h:
                nf += 1
                nh += h
    return nf, nh


# ===========================================================================
# 实验 1：先数
# ===========================================================================

def exp1():
    head("实验 1  ★★ 先数：两个飞控用不用这两个特性")

    print("""
  第 18 课的预告里我写错了三个论断，共同点都是**看到名字就下结论**。
  所以这一课第一步是**数**：

      "MPU / SMP 这两个特性，ArduPilot 和 PX4 到底用不用？"
""")

    hr("-")
    print("  【1】MPU（内存保护单元）")
    hr("-")
    print()
    print("      %-38s %-10s %s" % ("项目 / 关键词", "命中文件", "命中次数"))
    print("      " + "-" * 62)
    CHECKS = [
        ("ArduPilot", os.path.join(APM_ROOT, "libraries"),
         r"configENABLE_MPU|xTaskCreateRestricted|vTaskAllocateMPURegions|MPU_xTaskCreate"),
        # ★ 只查 ChibiOS 内核有没有 MPU 支持（不查 CMSIS 头里的 MPU_CTRL ——
        #   那是芯片寄存器定义，和"AP 用不用 MPU"无关）
        ("ArduPilot/ChibiOS 内核", os.path.join(CHIBIOS, "os", "rt"),
         r"CH_CFG_USE_MPU|chMpuConfigure"),
        ("PX4 板级配置", F_PX4_BOARDS, r"^CONFIG_ARM_MPU=y"),
        ("PX4 板级配置（只 EA_RST）", F_PX4_BOARDS, r"^CONFIG_ARM_MPU_EARLY_RESET=y"),
    ]
    for name, root, pat in CHECKS:
        nf, nh = grep_count(root, pat, exts=(".c", ".h", ".cpp", ".hpp",
                                             ".dat", "defconfig", ""))
        print("      %-38s %-10d %d" % (name, nf, nh))

    print("""
      ★★ **注意第三行和第四行的区别：**

          CONFIG_ARM_MPU=y               ← 真的开 MPU
          CONFIG_ARM_MPU_EARLY_RESET=y   ← ★ **只是在启动时把 MPU 关掉**

        后者**看起来像"用 MPU"**，实际是"防止 bootloader 留下的 MPU
        配置把 NuttX 挡住"（见 §2.3）。**名字骗人，必须读 Kconfig 的说明。**
""")

    hr("-")
    print("  【2】SMP（多核）")
    hr("-")
    print()
    for name, path, pat in (
            ("ArduPilot/FreeRTOSConfig.h", F_AP_FRCONF, r"configNUMBER_OF_CORES"),
            ("ArduPilot/chconf.h（ChibiOS）", F_CHCONF, r"CH_CFG_SMP_MODE")):
        n = find_line(path, pat)
        print("      %-38s %s" % (name,
              ("见 %s:%d" % (os.path.basename(path), n)) if n else "★ 没有这个宏"))
    nf, _ = grep_count(F_PX4_BOARDS, r"^CONFIG_SMP=y",
                       exts=("defconfig", ""))
    print("      %-38s %d 个板级配置" % ("PX4 boards 里的 CONFIG_SMP=y", nf))
    print()
    for path, pat, label in ((F_AP_FRCONF, r"configUSE_SMP|CONFIG_NUMBER_OF_CORES",
                              "AP 的 FreeRTOSConfig.h"),
                             (F_CHCONF, r"CH_CFG_SMP_MODE\s+(TRUE|FALSE)",
                              "AP 的 chconf.h")):
        lines = read(path).split("\n")
        for i, l in enumerate(lines, 1):
            if re.search(pat, l):
                print("      %-38s %s:%d  %s" % (label, os.path.basename(path), i,
                                                 l.strip()[:48]))

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论（这就是"先数"的结果）**：

      | 特性 | ArduPilot | PX4 |
      |---|---|---|
      | **MPU** | ❌ **0 处** | ⚠️ **122 个板级配置里只有 3 个** |
      | **SMP** | ❌ 不设核数（单核） | ❌ **0 个板子**开 CONFIG_SMP |

     ★ 两个成熟飞控**默认都不用这两个特性** —— 这本身就是这一课
       最值得讲的结论（§5 分析为什么）。

     ★ 而 PX4 那 3 个 MPU 配置**不是常规固件**：
       `fmu-v5/nuttx-config/protected`（一个专门的保护模式配置）
       和 `fmu-v6xrt` 的两个（RT 变体）—— 见 §2.3。
""")


# ===========================================================================
# 实验 2：单核变量 → 按核索引
# ===========================================================================

def exp2():
    head("实验 2  ★★ 单核变量 → 按核索引（SMP 的改写方式）")

    print("""
  从单核改成多核，FreeRTOS 做的事可以概括成一句话：

      **把"全局的单例"改成"按核索引的数组"。**

  看 `tasks.c` 里的实际改写：
""")

    lines = read(F_TASKS_C).split("\n")
    pairs = []
    for i, l in enumerate(lines, 1):
        m = re.search(r"pxCurrentTCBs\[\s*(?:portGET_CORE_ID\(\)|xCoreID|uxCore)\s*\]", l)
        if m:
            pairs.append((i, l.strip()))
    print("      `pxCurrentTCBs[...]` 出现 %d 次，前几处：" % len(pairs))
    for i, l in pairs[:6]:
        print("      tasks.c:%d  %s" % (i, l[:88]))
    print()

    n = find_line(F_TASKS_C, r"#define portGET_CRITICAL_NESTING_COUNT")
    if n:
        src(F_TASKS_C, n - 8, n + 6,
            "★ 临界区嵌套计数：单核是一个变量，多核变成【每核一个】：")

    n2 = find_line(F_TASKS_C, r"xYieldPendings\[ configNUMBER_OF_CORES \]")
    if n2:
        src(F_TASKS_C, n2 - 1, n2 + 1, "★ 跨核抢占标志，也是一个数组：")

    print("""  **被"数组化"的东西（实测清单）：**

      | 单核的写法 | SMP 的写法 | 为什么 |
      |---|---|---|
      | `pxCurrentTCB` | `pxCurrentTCBs[ core ]` | 每个核在跑不同的任务 |
      | `uxCriticalNesting` | `pxCurrentTCBs[core]->uxCriticalNesting` | 每个核的临界区深度不同 |
      | （没有） | `xYieldPendings[ core ]` | ★ 一个核要通知另一个核"该让位了" |

  ★★ **这就是多核带来的第一类改动**：**没有一个"全局的当前"了**。

  ⚠️ 但第二类改动更麻烦 —— **共享的数据结构**：
     `pxReadyTasksLists[]` 在 SMP 下**仍然是全局一份**（见实验 4），
     所以每次访问都要**自旋锁**保护。
""")

    n3 = find_line(F_TASKS_C, r"#define taskSELECT_HIGHEST_PRIORITY_TASK\( xCoreID \)")
    if n3:
        src(F_TASKS_C, n3 - 2, n3 + 2,
            "★ 连「选下一个任务」都从宏变成了带核号的函数：")


# ===========================================================================
# 实验 3：MPU region
# ===========================================================================

def exp3():
    head("实验 3  ★ MPU region：8 个区、权限、越界会怎样")

    print("""
  MPU 做的事：把内存切成若干**区（region）**，每个区配一组合权限，
  越权访问触发**硬件异常**（不是返回错误码）。

  ARMv7-M 的 MPU 最多 16 个区（region），至少 8 个；
  PX4 那个 protected 配置选的是 **8 个**（`CONFIG_ARM_MPU_NREGIONS=8`）。
""")

    print("""
  【模拟】一个 protected build 的典型分区表 —— 8 个区：

      · 内核代码 / 内核数据  → 特权态可读写，用户态**不可访问**
      · 用户代码 / 用户数据  → 用户态可读（代码）/ 可读写（数据）
      · 外设寄存器           → **只有特权态**能访问
""")
    hr("-")
    print()

    # (名字, 起始, 大小KB, 用户态权限)
    REGIONS = [
        ("内核 .text",   0x08000000, 256, "无"),
        ("内核 .data",   0x20000000, 64,  "无"),
        ("用户 .text",   0x08040000, 128, "只读"),
        ("用户 .data",   0x20010000, 96,  "读写"),
        ("用户栈",       0x20020000, 32,  "读写"),
        ("外设",         0x40000000, 128, "无"),
        ("共享内存",     0x20030000, 16,  "读写"),
        ("（未分配）",     None,      0,   "—"),
    ]

    print("      %-14s %-14s %-8s %s" % ("区", "起始地址", "大小", "用户态权限"))
    print("      " + "-" * 58)
    for name, addr, kb, perm in REGIONS:
        a = ("0x%08X" % addr) if addr else "—"
        print("      %-14s %-14s %-8s %s"
              % (name, a, ("%d KB" % kb) if kb else "—", perm))

    print("""
  然后模拟三种访问，看 MPU 放不放行：
""")
    KERNEL, USER = "特权态", "用户态"

    def check(region, mode, is_write):
        perm = dict((r[0], r[3]) for r in REGIONS)[region]
        if perm == "—":
            return False
        if mode == KERNEL:
            return True                        # 特权态不受限
        if perm == "无":
            return False
        if perm == "只读":
            return not is_write
        return True

    print("      %-12s %-8s %-8s %-10s %s"
          % ("访问哪个区", "谁", "读/写", "MPU", "后果"))
    print("      " + "-" * 72)
    cases = [
        ("内核 .text", KERNEL, False, "正常运行"),
        ("内核 .text", USER, False, "★ 用户态读内核代码"),
        ("外设", USER, True, "★ 用户态写寄存器"),
        ("用户 .data", USER, True, "正常读写自己的数据"),
        ("用户 .text", USER, True, "★ 用户态改自己的代码"),
        ("共享内存", USER, True, "约定的通道"),
    ]
    for region, mode, wr, desc in cases:
        ok = check(region, mode, wr)
        print("      %-12s %-8s %-8s %-10s %s"
              % (region, mode, "写" if wr else "读",
                 "✅ 放行" if ok else "❌ 触发异常", desc))

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：MPU 的语义是"**越权 = 硬件异常**"，不是"返回错误码"。

     ★ 这带来两个后果：

       ① **好处**：内存错误**立刻**暴露，而且**定位准确**
          （异常现场 + 访问地址 + 是哪个区，都在寄存器里）
       ② ★★ **代价**：**每个任务都要有自己的权限配置** ——
          也就是说，你**必须先把内存按"谁访问"分好区**

     ★★ **这才是飞控不用它的真正原因**（不是"性能开销"）：
        MPU 的前提是"**代码之间互不信任**"，而飞控的代码是
        **一个可信代码库**（全部自己写、一起编译）。
        分区带来的收益（防越权）在飞控上几乎为零，
        代价（分区 + 跨区调用变 syscall）却是实打实的。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · region 的对齐要求（ARMv7-M 要求 region 大小是 2 的幂且地址对齐）
    · 优先级（region 编号越大优先级越高，可覆盖）
    · 权限位编码（AP/XN/PXN 等）
  模拟保住了什么：
    · "越权 ⇒ 异常"这个语义
    · "权限是按【区】配的，不是按【代码】配的"这个结构事实
""")


# ===========================================================================
# 实验 4：两种 SMP 调度模型
# ===========================================================================

def exp4():
    head("实验 4  ★★ 两种 SMP 调度模型（共享队列 vs 每核实例）")

    print("""
  实验 2 看到 FreeRTOS 把"当前任务"数组化了。但**调度器本身**呢？
  这里两家 RTOS 走了**两条不同的路**。
""")

    n = find_line(F_TASKS_C, r"PRIVILEGED_DATA static List_t pxReadyTasksLists")
    if n:
        src(F_TASKS_C, n, n, "FreeRTOS：就绪队列是【全局一份】：")
    n2 = find_line(F_CHSYS_C, r"CH_SYS_CORE0_MEMORY os_instance_t ch0")
    if n2:
        src(F_CHSYS_C, n2 - 3, n2 + 1, "ChibiOS：【每核一个内核实例】：")

    print("""  ⇒ ★★ 两种模型：

      | | FreeRTOS SMP | ChibiOS SMP |
      |---|---|---|
      | 就绪队列 | **全局一份**（`pxReadyTasksLists[]`） | ★ **每核一份**（各在自己的 `os_instance_t` 里） |
      | 谁来保护 | ★ **自旋锁**（每次访问都要） | 各核只管自己的，**不冲突** |
      | 跨核唤醒 | `xYieldPendings[core]` + 核间中断 | `chSysNotifyInstance()` |
      | 负载均衡 | 内核里做（可以从别人队列里"偷"） | 应用自己安排 |
""")

    n3 = find_line(F_CHSCHD_C, r"chSysNotifyInstance\(ntp->owner\)")
    if n3:
        src(F_CHSCHD_C, n3 - 5, n3 + 6,
            "★ ChibiOS 的跨核唤醒（第 15 课 §1.7 见过这段）：")

    hr("-")
    print("""
  【模拟】两个核、三个任务，跑两种模型，看**调度时要不要抢锁**。

      共享模型：每次"选下一个任务"都要拿自旋锁（别的核可能正在改队列）
      每核模型：各核只看自己的队列，**不需要锁**
""")
    hr("-")
    print()

    HOLD = 3      # 一次"选下一个任务"要持锁 3 个时间单位

    def simulate(cores, offset):
        """★ 关键：**锁是【持有一段时间】的**。

        作者第一版在同一个函数里"拿了立刻放"，于是两个核永远
        不会撞上 —— 模拟恒返回 0 次争用，等于没模拟。
        真实的争用发生在"A 还持有锁的时候 B 也来要"。
        """
        # 每个核发起一次 select 的时刻（两个核错开 offset 个时间单位）
        events = [(core * offset, core) for core in range(cores)]
        busy_until = []          # 已经被占用的时间窗
        contended = 0
        for t, core in sorted(events):
            if any(t < end for end in busy_until):
                contended += 1   # ★ 撞上别人持锁 ⇒ 自旋等
            busy_until.append(t + HOLD)
        return cores, contended

    print("      【共享队列 + 自旋锁】—— 两个核发起调度的时刻错开多少，"
          "就会撞上多少次")
    print()
    print("      %-14s %-12s %-12s %s"
          % ("核数", "错开", "撞上次数", "说明"))
    print("      " + "-" * 62)
    for cores, offset in ((2, 5), (2, 2), (2, 1), (4, 1)):
        n, c = simulate(cores, offset)
        note = ""
        if cores == 2 and offset >= HOLD:
            note = "错开了（不发同一时刻）⇒ 不撞"
        elif cores == 2 and offset == 1:
            note = "★ 相邻时刻发起 ⇒ 必撞"
        elif cores == 4:
            note = "★ 核越多，窗口越容易重叠"
        print("      %-14d %-12d %-12d %s" % (cores, offset, c, note))

    print("""
      【每核一份】—— 不需要锁，所以怎么错开都不撞：

      %-14s %-12s %-12s %s
      --------------------------------------------------------------
      %-14d %-12d %-12d %s
      %-14d %-12d %-12d %s"""
          % ("核数", "错开", "撞上次数", "说明",
             2, 1, 0, "各核只看自己那份 ⇒ 零争用",
             4, 1, 0, "同上"))

    print("""
      ⇒ ★★ **"共享 ⇒ 要锁，独立 ⇒ 不要锁"** ——
        而锁的代价不是"一次原子操作"（那很便宜），而是
        **"别的核正在用"时我**必须等**。

      ★ 注意第 3 行（2 核错开 1）：**只要两个核的调度时刻接近，就必撞**。
        而"调度"在 1 kHz 的控制循环里**每秒发生上千次** ——
        这个窗口重叠的概率不低。

  ----------------------------------------------------------------------
  ⇒ **结论**：两种模型**没有免费的那一边**：

      | | 共享队列 | 每核一份 |
      |---|---|---|
      | 每次调度 | 要拿锁（★ 核越多越容易撞） | 零开销 |
      | 负载均衡 | ✅ 内核能做（可以从别的核"偷"任务） | ❌ 应用自己安排 |
      | 一个任务能在哪个核跑 | ★ **默认任意，但受【亲和性掩码】约束** | ★ **绑死在它被创建的那份队列上** |

     ⚠️ 这一格原来写的是"**任意**" —— **不准确**。
        FreeRTOS 的 SMP 有**亲和性（core affinity）**机制：
          · `uxCoreAffinityMask`（`tasks.c:366-367`）—— 每个 TCB 一个位掩码
          · `xTaskCreateAffinitySet()` / `vTaskCoreAffinitySet()`（`task.h:391/1412`）
          · `tskNO_AFFINITY`（`task.h:203`）= 不限制，这才是"任意"
          · 靠 `configUSE_CORE_AFFINITY` 打开（**默认 0**，`FreeRTOS.h:501-503`）
        ⇒ 准确说法是"**默认任意；开了亲和性就能绑定**"。

     ★★ **飞控为什么倾向"每核一份"**：因为**确定性**。
        共享队列下"我的任务什么时候被调度"取决于**别的核在干什么**；
        每核一份下，只要把任务绑到核上，**行为就退化成单核**。

     ★ 而 FreeRTOS 的 SMP 模型（共享队列）是为了**通用吞吐**设计的 ——
       这是"通用 RTOS"和"飞控 RTOS"的目标差异，不是好坏。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实自旋锁的原子指令与缓存行争用
    · 核间中断延迟、内存序
    · 负载均衡的实现（FreeRTOS 的 SMP 里有一套）
  模拟保住了什么：
    · "**共享 ⇒ 要锁，独立 ⇒ 不要锁**"这个结构事实
    · 两种模型各自失去什么（均衡 vs 确定性）
""")


# ===========================================================================
# 实验 5：成本清单
# ===========================================================================

def exp5():
    head("实验 5  ★★ 成本清单：为什么两个飞控都不用")

    print("""
  实验 1 数出来"两个飞控都不用"，这一节把**为什么**列成清单 ——
  而且每条都从源码里找依据，不靠印象。
""")

    hr("-")
    print("  【MPU 的成本】")
    hr("-")
    print()
    COST_MPU = [
        ("必须先分区",
         "MPU 的权限是按【区】配的，所以要把所有内存按'谁访问'归类",
         "§3 的分区表：8 个区才勉强够"),
        ("跨区调用变成系统调用",
         "用户态代码要访问内核对象（比如发一个队列）必须走 syscall",
         "FreeRTOS 的 mpu_wrappers.h + mpu_prototypes.h 就是干这个的"),
        ("每个任务一组配置",
         "xTaskCreateRestricted 要求每个任务带一张 region 表",
         "任务数 × region 数 = 配置工作量"),
        ("收益对飞控≈0",
         "MPU 防的是'不可信代码'，而飞控是单一可信代码库",
         "——"),
    ]
    print("      %-20s %-42s %s" % ("成本项", "说明", "依据"))
    print("      " + "-" * 84)
    for a, b, c in COST_MPU:
        print("      %-20s %-42s %s" % (a, b[:42], c))

    print("""
       ★ 前三条是**代码里能看到的**（`mpu_wrappers.h` 就是为此存在的），
         第四条是判断 —— 而它决定了前三条值不值。
""")

    hr("-")
    print("  【SMP 的成本】")
    hr("-")
    print()
    COST_SMP = [
        ("临界区变了",
         "单核关中断就够；多核还要抢自旋锁 —— 而自旋锁在中断里不能随便用",
         "tasks.c 里 uxCriticalNesting 变成 per-core"),
        ("数据结构要锁",
         "就绪队列、延时链表这些全局结构，每次访问都要保护",
         "pxReadyTasksLists 在 SMP 下仍是全局一份"),
        ("确定性下降",
         "任务的调度时机取决于别的核在干什么",
         "实验 4 的模拟"),
        ("飞控的实时性来自单核",
         "1 kHz 控制循环要的是'可预测'，不是'吞吐'",
         "——"),
    ]
    print("      %-20s %-42s %s" % ("成本项", "说明", "依据"))
    print("      " + "-" * 84)
    for a, b, c in COST_SMP:
        print("      %-20s %-42s %s" % (a, b[:42], c))

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论：三个层次的理由，从具体到根本**

      ① **具体**：这两个特性都要"额外的基础设施"
         （MPU 要 syscall 层，SMP 要自旋锁层）—— 都在代码里看得见

      ② **结构**：飞控是**单一可信代码库** + **单核确定性**，
         这两个前提恰好让 MPU 和 SMP 的主要收益都落空

      ③ ★★ **根本**：**"通用 RTOS 的特性"和"飞控的需求"不是一回事。**
         FreeRTOS 要覆盖从洗衣机到基站的所有场景，所以它提供一整套
         可选特性；而**飞控只需要其中一个很小的子集**。

     ★ 这也回到本线第一课的那个观察：
       **只见过一种实现的人，分不清"这是设计必然"还是"这是这个实现的偶然"。**
       现在你能分清了 —— 因为你在三条线上看到了同一件事的三种做法。

  ----------------------------------------------------------------------
  本实验的每条"依据"都可以自己去代码里核实（列在附：源码位置索引）。
  第四条（"收益≈0"）是**判断**，不是数据 —— 按本线的纪律，标出来。
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 先数：两个飞控用不用", exp1),
    "2": ("★★ 单核变量 → 按核索引", exp2),
    "3": ("★ MPU region 与权限", exp3),
    "4": ("★★ 两种 SMP 调度模型", exp4),
    "5": ("★★ 成本清单", exp5),
}


def main():
    print()
    hr("#")
    print("#  第 9 课：MPU 与 SMP")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(FREERTOS) or not os.path.isdir(APM_ROOT):
        print()
        print("  [错误] 找不到源码：")
        print("         %s" % FREERTOS)
        print("         %s" % APM_ROOT)
        print()
        print("  请先按《总纲》的方法准备源码。")
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
    print("  #  实验结束")
    hr("#")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
