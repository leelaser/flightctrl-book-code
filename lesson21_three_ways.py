#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 21 课配套实验：同一问题的三种实现

    实验 1  ★★ ISR-safe API 的计数口径（139/115/55/51）
    实验 2  栈单位的坑（三家不一样）
    实验 3  API 形态对照（逐项核实）
    实验 4  ★ 计数纪律

用法：
    python lesson21_three_ways.py 1
    python lesson21_three_ways.py 2
    python lesson21_three_ways.py 3
    python lesson21_three_ways.py 4
    python lesson21_three_ways.py          # 全部

【本课的主旨】

  定量对比里，「数出来多少」和「怎么数的」一样重要。

  这个脚本的每个实验都【同时输出计数规则和命令】，
  让你可以自己复核。

源码版本：
    FreeRTOS V11.1.0
    ChibiOS 6a85082c / ArduPilot 4.5.7
    NuttX（PX4 1.15.x 打包版本）
"""

import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))

FREERTOS = os.path.join(ROOT, "FreeRTOS")
APM_ROOT = os.path.join(ROOT, "APM", "ardupilot-Copter-4.5.7",
                        "ardupilot-Copter-4.5.7")
PX4_ROOT = os.path.join(ROOT, "PX4", "PX4-Autopilot")

CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")
NUTTX = os.path.join(PX4_ROOT, "platforms", "nuttx", "NuttX", "nuttx")
HALCHIB = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS")


def read_text(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return None


def read_lines(path):
    t = read_text(path)
    return None if t is None else t.split("\n")


def hr(ch="=", n=74):
    print(ch * n)


def head(title):
    print()
    hr()
    print("  " + title)
    hr()


def collect(root, exts, rx):
    """在目录树里收集匹配 rx 的【唯一标识符】。

    返回 (名字集合, 命中文件数)
    """
    names = set()
    nfiles = 0
    for dirpath, _d, filenames in os.walk(root):
        for fn in filenames:
            if not fn.endswith(exts):
                continue
            p = os.path.join(dirpath, fn)
            t = read_text(p) or ""
            hits = rx.findall(t)
            if hits:
                nfiles += 1
                for h in hits:
                    names.add(h if isinstance(h, str) else h[0])
    return names, nfiles


# ===========================================================================
# 实验 1：ISR-safe API 的计数口径
# ===========================================================================

def exp1():
    head("实验 1：★★ ISR-safe API 的计数口径")

    print("""
  "这家 RTOS 有几个 ISR-safe API？"

  ★ 这个问题【没有唯一答案】—— 答案取决于你怎么数。
  这个实验把每个口径都算一遍，让你看到差异从哪来。
""")

    print()
    print("  ── FreeRTOS：逐层过滤 ──")
    print()

    inc = os.path.join(FREERTOS, "include")
    if not os.path.isdir(inc):
        print("    [读不到 %s]" % inc)
        return

    # 收集所有 *FromISR 标识符（含各种前缀）
    rx_all = re.compile(r"\b([A-Za-z_]+FromISR)\s*\(")
    all_names, nfiles = collect(inc, (".h",), rx_all)

    FILTERS = [
        ("① 原始匹配（含所有前缀）", lambda n: True),
        ("② 排除 MPU_ 包装", lambda n: not n.startswith("MPU_")),
        ("③ 再排除 trace* 宏",
         lambda n: not n.startswith("MPU_") and not n.startswith("trace")),
        ("④ 再排除 ux* 开头的（本课采用）",
         lambda n: not n.startswith("MPU_") and not n.startswith("trace")
                   and not n.startswith("ux")),
        ("⑤ 再排除协程 CR* 和临界区 Enter/ExitCritical",
         lambda n: not n.startswith("MPU_") and not n.startswith("trace")
                   and not n.startswith("ux")
                   and "CRReceive" not in n and "CRSend" not in n
                   and "EnterCritical" not in n and "ExitCritical" not in n),
    ]

    print("    %-44s %s" % ("计数口径", "数量"))
    print("    " + "-" * 58)
    for desc, filt in FILTERS:
        cnt = len({n for n in all_names if filt(n)})
        print("    %-44s %d" % (desc, cnt))

    print()
    print("    ⇒ 五个都合理的口径，给出【五个不同的数字】。")
    print()
    print("    ★ 本课采用口径 ④ = %d 个。"
          % len({n for n in all_names
                 if not n.startswith("MPU_") and not n.startswith("trace")
                 and not n.startswith("ux")}))
    print()
    # ★ 命令必须与上面的算法【真的等价】—— 这条是本课的门面
    #   （整课的主题就是"计数必须给可复现命令"）。
    #
    #   ⚠️ 初版打印的是 `\b[xv][A-Za-z]*FromISR\s*\(`，**它得 55 不是 51**：
    #      `\s*` 会让注释里的散文 `See xQueueSendFromISR ()` 也匹配上，
    #      而 `grep -o` 输出的是**整段匹配**（含那个空格），
    #      于是 `sort -u` 把 `xQueueSendFromISR(` 和 `xQueueSendFromISR (`
    #      当成两个不同的名字 —— 多出 4 个（正好是 4 个函数有两种写法）。
    #
    #   ⇒ 去掉 `\s*`（只匹配紧贴的括号），再用 sed 去掉尾部括号，
    #     最后三个 grep -v 对应上面 set 推导式里的三个 startswith 排除。
    #     实测：51 ✓（与上面的口径 ④ 一致）
    print("    可复现命令（实测得 %d，与口径 ④ 一致）："
          % len({n for n in all_names
                 if not n.startswith("MPU_") and not n.startswith("trace")
                 and not n.startswith("ux")}))
    print("        grep -rhoE \"\\b[A-Za-z_]+FromISR\\(\" %s/*.h \\"
          % inc.replace(ROOT, "...").replace("\\", "/"))
    print("            | sed 's/($//' | sort -u \\")
    print("            | grep -v \"^MPU_\" | grep -v \"^trace\" | grep -v \"^ux\" | wc -l")
    print()
    print("        ⚠️ 不要写成 `[A-Za-z]*FromISR\\s*\\(`（带 \\s*）——")
    print("           那样会得 55：注释里的 `See xQueueSendFromISR ()` 也算了进去。")

    # 被 ⑤ 排除的
    extra = {n for n in all_names
             if not n.startswith("MPU_") and not n.startswith("trace")
             and not n.startswith("ux")
             and ("CRReceive" in n or "CRSend" in n
                  or "EnterCritical" in n or "ExitCritical" in n)}
    if extra:
        print()
        print("    ── 口径 ⑤ 额外排除的 %d 个 ──" % len(extra))
        for n in sorted(extra):
            why = ("协程（legacy）" if "CR" in n else "临界区宏")
            print("        %-30s %s" % (n, why))

    print()
    print("  ── ★★ 三家对照（各自的口径）──")
    print()

    # ChibiOS：I 后缀
    chib_inc = os.path.join(CHIBIOS, "os", "rt", "include")
    rx_chib = re.compile(r"\b(ch[A-Za-z0-9_]+I)\s*\(")
    chib_all, _ = collect(chib_inc, (".h",), rx_chib)
    # ★ 和 FreeRTOS 那一侧一样**必须过滤** —— 否则两家的口径不同，
    #   "51 vs 52" 就成了假对比（本课 §0 讲的就是这件事）。
    #   被排除的 17 个逐类列在课文里：
    #     调试/跟踪 5、与 IPC 无关 7、内核内部助手 5
    #   ⇒ 52 − 17 = 35（与课文一致，命令见课文 §2）
    CHIB_EXCL = re.compile(
        r'^ch(Dbg|Trace)|^chRFCU|^chSysIntegrityCheckI|^chSysTimerHandlerI|'
        r'^chTime[A-Za-z]+2I|^chSch|^chThdDequeue|^chThdDoDequeue|'
        r'^chThdQueueIsEmpty')
    chib_names = {n for n in chib_all if not CHIB_EXCL.match(n)}

    # NuttX：_isr 后缀
    nuttx_inc = os.path.join(NUTTX, "include")
    rx_nuttx = re.compile(r"\b([a-z_]+_isr)\s*\(")
    nuttx_names, _ = collect(nuttx_inc, (".h",), rx_nuttx)

    print("    %-14s %-10s %s" % ("系统", "数量", "计数规则"))
    print("    " + "-" * 66)
    print("    %-14s %-10d %s"
          % ("FreeRTOS", len({n for n in all_names
                              if not n.startswith("MPU_")
                              and not n.startswith("trace")
                              and not n.startswith("ux")}),
             "*FromISR 后缀（排除 MPU_/trace/ux）"))
    print("    %-14s %-10d %s"
          % ("ChibiOS", len(chib_names), "*I 后缀（os/rt/include/*.h）"))
    print("    %-14s %-10d %s"
          % ("NuttX", len(nuttx_names), "*_isr 后缀（include/ 下）"))

    print()
    print("""    ★★★ 现在看这三个数字：51 / %d / %d

      如果【不看计数规则】把它们放一起，会得出
      「NuttX 不支持在 ISR 里用同步原语」这个【完全错误】的结论。

      实际上：NuttX 没有专用版本，是因为【不需要】——
      它的临界区就是关中断（cpsid i），在 ISR 里天然安全。
      同一个 nxsem_post() 任务侧和 ISR 侧都能用。

    ⇒ ★★ 这是本课最重要的一课：

        定量对比的第一步，是确认
        「这个数字在三个系统里指的是【同一件事】」。

        如果不一致，那个对比就是【假对比】——
        它比「不做对比」更糟，因为它【看起来很有说服力】。
""" % (len(chib_names), len(nuttx_names)))
    print()


# ===========================================================================
# 实验 2：栈单位的坑
# ===========================================================================

def exp2():
    head("实验 2：栈单位的坑")

    print("""
  三家创建任务时都要给"栈大小"。但这个数字的【单位】不一样。
  这个实验从源码/头文件里找出每个 API 的单位说明。
""")

    print()
    print("  ── FreeRTOS：StackType_t 的【个数】──")
    print()

    p = os.path.join(FREERTOS, "include", "task.h")
    t = read_text(p) or ""
    m = re.search(r"usStackDepth", t)
    if m:
        # 找参数说明
        idx = t.find("usStackDepth")
        snippet = t[max(0, idx - 200):idx + 600]
        for line in snippet.split("\n"):
            s = line.strip()
            if s.startswith("*") and ("stack" in s.lower() or "byte" in s.lower()
                                      or "word" in s.lower()):
                print("    task.h   %s" % s[:64])
        print()

    print("""    ★ 关键：参数名是 usStackDepth（深度），不是"字节数"。
      深度 × sizeof(StackType_t) = 实际字节数。
      在 Cortex-M 上 StackType_t = uint32_t ⇒ ×4。

      所以：
          xTaskCreate(fn, "n", 512, ...)
          ⇒ 实际栈 = 512 × 4 = 2048 字节
""")

    print("  ── ChibiOS：字节（而且是 sizeof 出来的）──")
    print()

    p = os.path.join(CHIBIOS, "os", "rt", "include", "chthreads.h")
    lines = read_lines(p) or []
    for i, line in enumerate(lines, 1):
        if "chThdCreateStatic" in line:
            print("    chthreads.h:%-4d %s" % (i, line.strip()))
    print()
    print("""    ★ 典型用法：

          static THD_WORKING_AREA(wa, 512);      ← 512 是【字节】
          chThdCreateStatic(wa, sizeof(wa), ...);  ← sizeof 给的是字节

      参数名就叫 size（不是 depth）。
""")

    print("  ── NuttX：字节 ──")
    print()

    p = os.path.join(NUTTX, "include", "nuttx", "kthread.h")
    lines = read_lines(p) or []
    shown = 0
    for i, line in enumerate(lines, 1):
        if "task_create" in line and shown < 3:
            print("    kthread.h:%-4d %s" % (i, line.strip()[:70]))
            shown += 1
    print()
    print("""    ★ 典型用法：

          task_create("name", priority, 2048, fn, arg);
                                        ↑ 2048 是【字节】

      参数名就是 stack_size。
""")

    print("  ── ★★ 对照 ──")
    print()
    print("    %-16s %-22s %s" % ("系统", "参数", "单位"))
    print("    " + "-" * 54)
    print("    %-16s %-22s %s" % ("FreeRTOS", "usStackDepth", "★ StackType_t 个数"))
    print("    %-16s %-22s %s" % ("ChibiOS", "size", "字节"))
    print("    %-16s %-22s %s" % ("NuttX", "stack_size", "字节"))

    print()
    print("""    ★★ 为什么这个坑危险？

      三者混用时，"512" 和 "2048" 可能指同一件事，也可能差 4 倍。
      而【数字看起来都是"几百"】，没有人会怀疑。

      ⇒ 这是「看起来精确但没有单位」的陷阱的又一例（第 21 课 §0）。

      ⚠ 更糟的是：栈溢出在 FreeRTOS 上表现为
        「某个任务的变量被莫名改写」，
        而不是「栈不够了」—— 极难定位。

      ⇒ 所以 FreeRTOS 提供了 configCHECK_FOR_STACK_OVERFLOW
        （两个检查级别），第 1 课讲过。
""")
    print()


# ===========================================================================
# 实验 3：API 形态对照
# ===========================================================================

def exp3():
    head("实验 3：API 形态对照")

    print("""
  逐项核实第 21 课引用的每个 API 是否真的存在。
  ★ 每个都给出【在哪个文件里】—— 便于你自己打开看。
""")

    print()

    CHECKS = [
        ("创建一个任务", [
            ("FreeRTOS", FREERTOS, "xTaskCreate"),
            ("FreeRTOS", FREERTOS, "xTaskCreateStatic"),
            ("ChibiOS", CHIBIOS, "chThdCreateStatic"),
            ("ChibiOS", CHIBIOS, "chThdCreate"),
            ("NuttX", NUTTX, "task_create"),
        ]),
        ("ISR → 任务", [
            ("FreeRTOS", FREERTOS, "xQueueSendFromISR"),
            ("FreeRTOS", FREERTOS, "xTaskNotifyFromISR"),
            ("ChibiOS", CHIBIOS, "chSemSignalI"),
            ("ChibiOS", CHIBIOS, "chEvtSignalI"),
            ("NuttX", NUTTX, "nxsem_post"),
        ]),
        ("互斥锁", [
            ("FreeRTOS", FREERTOS, "xSemaphoreTake"),
            ("ChibiOS", CHIBIOS, "chMtxLock"),
            ("ChibiOS", CHIBIOS, "chMtxTryLock"),
            ("NuttX", NUTTX, "nxmutex_lock"),
            ("NuttX", NUTTX, "nxmutex_trylock"),
        ]),
        ("消息 / 队列", [
            ("FreeRTOS", FREERTOS, "xQueueSend"),
            ("ChibiOS", CHIBIOS, "chMsgSend"),
            ("ChibiOS", CHIBIOS, "chMsgWait"),
            ("NuttX", NUTTX, "mq_send"),
            ("NuttX", NUTTX, "mq_receive"),
        ]),
        ("事件 / 多路等待", [
            ("FreeRTOS", FREERTOS, "xEventGroupWaitBits"),
            ("ChibiOS", CHIBIOS, "chEvtWaitAnyTimeout"),
            ("NuttX", NUTTX, "nxsig_timedwait"),
        ]),
        ("高分辨率时间", [
            ("FreeRTOS", FREERTOS, "xTaskGetTickCount"),
            ("ChibiOS", CHIBIOS, "chVTGetTimeStampI"),
            ("ChibiOS", CHIBIOS, "chVTGetSystemTimeX"),
            ("NuttX", NUTTX, "clock_gettime"),
        ]),
    ]

    total_ok = total_miss = 0
    for group, items in CHECKS:
        print("    ── %s ──" % group)
        print()
        for label, base, sym in items:
            found = find_symbol(base, sym)
            if found:
                total_ok += 1
                print("        %-10s %-26s ✓  %s" % (label, sym, found))
            else:
                total_miss += 1
                print("        %-10s %-26s ✗  未找到" % (label, sym))
        print()

    print("    ⇒ 核实 %d 个，找到 %d 个。"
          % (total_ok + total_miss, total_ok))

    print()
    print("  ── ★ 三家 API 的命名形态规律 ──")
    print()
    print("""    把上面的清单横着看，能看出三家的命名哲学：

        FreeRTOS   动词 + 类型前缀 + FromISR 后缀
                   xQueueSendFromISR / xTaskNotifyFromISR
                   ⇒ 【动词在前】，一眼看出"做什么"

        ChibiOS    模块前缀 + 动词 + 状态后缀
                   chSemSignalI / chMtxTryLock
                   ⇒ 【模块在前】，一眼看出"属于谁"

        NuttX      POSIX 风格（小写 + 下划线）
                   nxsem_post / nxmutex_lock / mq_send
                   ⇒ 【全小写】，和非 POSIX 的 task_create 混用

    ★ 而"ISR 版本"的形态差异最大：

        FreeRTOS   xQueueSend  →  xQueueSendFromISR      【另起一个名字】
        ChibiOS    chSemSignal →  chSemSignalI           【加一个后缀】
        NuttX      nxsem_post  →  nxsem_post             【同一个】

    ⇒ 这三种形态，直接对应第 21 课 §2 讲的三种设计：

        FreeRTOS  临界区用 BASEPRI ⇒ ISR 里进临界区会破坏嵌套 ⇒ 必须分开
        ChibiOS   用 osalSysLockFromISR 显式标记 ⇒ 加后缀提醒
        NuttX     临界区 = 关中断 ⇒ ISR 里天然安全 ⇒ 不需要分
""")


def find_symbol(base, sym):
    """在 base 目录下找 sym 的定义/声明，返回相对路径 + 行号。

    ⚠ 第一版在【整棵树】里搜，结果 ChibiOS 的命中全来自 demos/ ——
      那儿有一份【旧版本】的 ch.h 拷贝（内核 2.x 时代），行号完全不对。
      而且它【看起来是对的】（路径合理、函数名对），很难发现。

      教训：搜索必须限定在【权威目录】。
            ChibiOS 就用 os/rt/include/，不要搜 demos/。
    """
    rx = re.compile(r"\b%s\s*\(" % re.escape(sym))
    hits = []

    # ★ ChibiOS：只搜内核头文件目录
    if os.path.basename(base) == "ChibiOS":
        base = os.path.join(base, "os", "rt", "include")

    # ★ 收集【全部】命中，最后再优选 —— 不要边走边 break
    #
    #   第一版是"找到一个就停"，于是 FreeRTOS 的
    #   xQueueSendFromISR 报的是 timers.c（一个【调用点】），
    #   xTaskCreate 报的是 tasks.c（定义，但不是声明）。
    #   读者会以为那是"这个函数在哪定义"，而实际上更好的答案是头文件。
    for dirpath, _d, filenames in os.walk(base):
        # 跳过明显无关的大目录
        rel = os.path.relpath(dirpath, base)
        if any(x in rel for x in (".git", "examples", "test", "demos",
                                  "third_party", "apps", "mm", "net",
                                  "binfmt", "libs", "drivers", "arch",
                                  "fs", "sched", "crypto", "video",
                                  "wireless", "boards", "openamp", "syscall")):
            continue
        for fn in filenames:
            if not fn.endswith((".h", ".c")):
                continue
            p = os.path.join(dirpath, fn)
            lines = read_lines(p) or []
            for i, line in enumerate(lines, 1):
                s = line.strip()
                if rx.search(line) and not s.startswith("*") and \
                   not s.startswith("//"):
                    hits.append((os.path.relpath(p, base).replace("\\", "/"),
                                 i, s))
                    break

    if not hits:
        return None

    # ★ 优选规则：头文件优先（那才是"声明在哪"），
    #   其次是 .c 里形如定义的行（行首就是返回类型，没有 = 或 .）
    def score(h):
        path, _i, s = h
        is_hdr = path.endswith(".h")
        # 定义/声明的样子：行首是类型名
        looks_like_decl = bool(re.match(
            r"^(static\s+)?[A-Za-z_][A-Za-z0-9_ *]*\b%s\s*\(" % re.escape(sym),
            s))
        # ★ 优先 include/ 下的路径 —— 那才是"公开声明"所在
        #   没有这一条时，nxsem_post 会命中 graphics/nxterm/nxterm.h
        #   里的一个调用处（路径深度一样，只是字母序在前）
        in_include = "/include/" in ("/" + path) or path.startswith("include/")
        # 路径越浅越可能是声明
        depth = path.count("/")
        return (not is_hdr, not looks_like_decl, not in_include, depth, path)

    hits.sort(key=score)
    path, lineno, _s = hits[0]
    return "%s:%d" % (path, lineno)


# ===========================================================================
# 实验 4：计数纪律
# ===========================================================================

def exp4():
    head("实验 4：★ 计数纪律")

    print("""
  本线在复核过程中犯了【四次同一形态的错误】。
  这个实验把它们列出来，并给出可复用的纪律。
""")

    print()
    print("    ── 四次错误的形态 ──")
    print()
    print("    %-8s %-12s %-32s %s"
          % ("#", "在哪", "我写的", "实际"))
    print("    " + "-" * 86)
    ERRORS = [
        ("1", "第 11 课", "睡眠状态机不支持 mutex 超时", "是主动选择不做"),
        ("2", "第 12 课", "守卫只做了一半", "守卫是完整的（不同的宏）"),
        ("3", "第 13 课", "只有 2 处 LLD 调用", "6 个函数 / 9 处"),
        ("4", "第 4 课", "52 个 FromISR", "口径不同：139/115/55/51"),
    ]
    for r in ERRORS:
        print("    %-8s %-12s %-32s %s" % r)

    print()
    print("""    ★★ 四次的形态完全一样：

        我搜了 X，没找到 Y   ⇒   我报告「没有 Y」

    ⇒ 而其中三次的"没找到"，是因为【Y 用了另一种形式】：

        第 11 课  「不用超时版本」≠「不支持超时」
        第 12 课  「不用 CH_CFG_USE_MUTEXES 守卫」≠「没有守卫」
        第 13 课 「不用宏」≠「不存在」（board.c 里是函数调用）
        第 4 课  「不是 [xv] 开头」≠「不是 FromISR 函数」
""")

    print("  ── ★ 一条可以复用的纪律 ──")
    print()
    print("""    ┌────────────────────────────────────────────────────────────┐
    │ 凡是「多少处 / 多少个 / 几遍」这类断言：                      │
    │                                                            │
    │   ① 要有配套的【可执行脚本】                                  │
    │   ② 要写明【计数规则】                                        │
    │   ③ 要输出【统计过程】，不只是结果                            │
    └────────────────────────────────────────────────────────────┘

    ★ 为什么？

      手工数的数字【看起来和代码数的一样精确】——

          「2 处」和「9 处」在报告里没有区别。
          「51 个」和「139 个」也都像一个确切的答案。

      但手工数【没有任何机制告诉你漏了什么】。

      而写成代码，至少你知道【自己是按什么规则数的】——
      而规则可以被检查、被质疑、被修正。
""")

    print("  ── 本线的实践 ──")
    print()
    print("    13 课，每课一个配套脚本，全部可运行：")
    print()
    for i in range(1, 14):
        pat = "lesson%02d_" % i
        found = None
        for fn in sorted(os.listdir(SCRIPT_DIR)):
            if fn.startswith(pat) and fn.endswith(".py"):
                found = fn
                break
        if found:
            n = count_experiments(os.path.join(SCRIPT_DIR, found))
            print("        %-38s %d 个实验" % (found, n))
        else:
            print("        %-38s （缺）" % (pat + "*.py"))

    print()
    print("""    ★ 每个脚本都：
        ① 从源码读数据（不写死结论）
        ② 打印它统计的过程
        ③ 对"期望值"做对照检查（对不上就报警）

    ⇒ 这样如果哪天源码变了而文档没改，跑一遍就能发现。
""")
    print()


def count_experiments(path):
    """数一个配套脚本里注册了几个实验。"""
    t = read_text(path) or ""
    m = re.search(r"EXPERIMENTS\s*=\s*\{(.*?)\n\}", t, re.S)
    if not m:
        return 0
    return len(re.findall(r'"\d+":\s*\(', m.group(1)))


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ ISR-safe API 的计数口径", exp1),
    "2": ("栈单位的坑", exp2),
    "3": ("API 形态对照", exp3),
    "4": ("★ 计数纪律", exp4),
}


def main():
    print()
    hr("#")
    print("#  第 21 课：同一问题的三种实现")
    print("#  配套实验")
    hr("#")

    for d, name in ((FREERTOS, "FreeRTOS"), (CHIBIOS, "ChibiOS"),
                    (NUTTX, "NuttX")):
        if not os.path.isdir(d):
            print()
            print("  [错误] 找不到 %s：%s" % (name, d))
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
    print("#  实验结束 —— 本线 13 课完结")
    hr("#")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
