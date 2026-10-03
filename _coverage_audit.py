# -*- coding: utf-8 -*-
"""覆盖边界审计 —— 这条教程【实际讲了什么、没讲什么】

为什么需要它
=======================================================================
本线的定位一度写成「补上那两个 RTOS 自己的【完整教程】」，
但实际覆盖是【有边界】的。边界如果只写在脑子里，会两件事同时发生：
    · 读者按「完整教程」的期待来读，然后发现缺东西
    · 作者自己也说不清到底缺哪些

所以把边界变成可执行的审计。

★ 一个重要的设计决定：行数由脚本算，覆盖状态【人工判定并写明依据】
=======================================================================
作者第一版想自动判「讲没讲」—— 用关键词命中的文档数当判据。
结果两个方向都错：

    timers.c     判「没讲」  实际第 6 课整课就是软件定时器
                             （判据要求命中 ≥2 篇文档，而它只在一课里深讲）
    event_groups.c 判「讲过」 实际第 21 课只有 46 行 API 对照
                             （判据只要 2 篇 × 5 行就放过）

⇒ 这和第 21 课 §9.4 那条纪律是同一个病：
  「凡是『有 / 没有』这类断言，判据写不清楚就等于没判」。

所以现在：**行数实时计算（确定的东西），覆盖状态是硬编码的结论 +
一条可核对的依据**。要改结论，就得改依据 —— 不能靠调阈值蒙过去。

★ 2026-09 第二次自检：课号重排【又抓出两条依据站不住】
=======================================================================
本线做了一次全局课号重排（补课 14/15 → 第 7/8 课，ChibiOS 各课顺移），
重排时逐条复核依据，发现两条**不是编号过时，而是当初就判错了**：

    依据原文                      实际
    ────────────────────────────  ──────────────────────────────────
    chsys.c  FULL「第 7/8 课」     grep chSysLock/chSysUnlock 只在
                                  第 12 课【引用的 ArduPilot 代码】里
                                  出现（讲的是 chMtxForceReleaseS），
                                  锁语义一个字没讲；chSysInit 零提及
                                  ⇒ 降为 MENTION
    chvt.c   FULL「第 8/9 课」     确实讲过，但在第 12 课 §5.3
                                  （栈上 virtual_timer_t）
                                  ⇒ 保留 FULL，依据改准

⇒ 这说明「依据」本身要能被复核，否则重排一次就暴露一次。
  凡是写「第 N 课」而不写「第 N 课 §哪一节」的，都可能是印象。

用法：
    python _coverage_audit.py            # 打印审计表
"""

import io
import _srcdirs
import os
import re
import sys

# ★★ 本脚本量的是**素材线**（`01-FreeRTOS/ 02-ChibiOS/ 03-合流/` 那套目录）的篇幅，
#    它按「自己在素材线的 code/ 里」推出根目录。搬进书稿的 `code/` 之后
#    `dirname(dirname())` 成了 `飞控丛书/`，`os.listdir` 里**一个 `NN-` 目录都没有**
#    ⇒ `lessons` 为空 ⇒ 后面 `frn[0]` 直接 IndexError。
#    ⇒ 改成**按已知位置找素材线**，并给出能看懂的失败信息。
def _find_root():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.dirname(here),                                    # 还在素材线里
        os.path.join(os.path.dirname(here), 'FreeRTOS-ChibiOS教程'),
        r'D:\DEVLib\PX4APM\FreeRTOS-ChibiOS教程',
    ]
    for c in cands:
        if os.path.isdir(os.path.join(c, '01-FreeRTOS')):
            return c
    raise SystemExit(
        '找不到素材线 `FreeRTOS-ChibiOS教程`（本脚本量的是那条线的篇幅）。\n'
        '  试过这些位置：\n    ' + '\n    '.join(cands) + '\n'
        '  ⇒ 这是**素材侧**脚本：请把它放在 `FreeRTOS-ChibiOS教程/` 旁边再跑。')


ROOT = _find_root()
os.chdir(ROOT)

FREERTOS = _srcdirs.src("FREERTOS")
CHIBIOS = _srcdirs.src("APM", "modules", "ChibiOS")

# ===========================================================================
# 覆盖状态用三个档：
#     FULL  专门讲过（有一课或一整节）
#     MENTION 只在别处顺带出现（列出名字 / 对照表里一行）
#     NONE  没讲
# 依据必须写清楚【在哪】，不能只写「讲过」。
# ===========================================================================
FULL, MENTION, NONE = "FULL", "MENTION", "NONE"

FREERTOS_KERNEL = [
    # (文件, 名称, 状态, 依据)
    ("tasks.c",        "任务与调度",     FULL,    "第 2 课整课"),
    ("queue.c",        "队列/信号量/互斥量", FULL,    "第 3 课整课"),
    ("list.c",         "链表",          FULL,    "第 2 课 §1 五条链表"),
    ("timers.c",       "软件定时器",     FULL,    "第 6 课 §1 整节"),
    ("event_groups.c", "事件组",         FULL,    "★ 第 7 课整课（2026-09 补齐）"),
    ("stream_buffer.c", "流/消息缓冲",    FULL,    "★ 第 7 课整课（2026-09 补齐）"),
    ("croutine.c",     "协程（legacy）",  MENTION, "第 1 课文件清单（官方已不推荐，影响小）"),
]

FREERTOS_FEATURES = [
    # (名称, 状态, 依据)
    ("MPU 内存保护",   FULL,    "★ 第 9 课 §2（mpu_wrappers/v1 vs v2/越权=异常）+ §1 的数：AP 0 处、PX4 122 个配置里 3 个"),
    ("Tickless 低功耗", FULL,    "★ 第 8 课整课（2026-09 补齐）"),
    ("SMP 多核",      FULL,    "★ 第 9 课 §3~§4（全局单例→按核索引、共享队列+自旋锁、xYieldPendings、三家 SMP 对照）"),
]

CHIBIOS_RT = [
    # (文件, 名称, 状态, 依据)
    ("chschd.c",      "调度器",      FULL, "第 11 课整课 + 第 12 课 §5.4（两套插入函数）"),
    ("chthreads.c",   "线程",        FULL, "API 散布第 10/12/20/21 课（CH_STATE_* 对照、chThdSetPriority 17 处），无独立一节"),
    ("chsem.c",       "信号量",      FULL, "第 12 课 §5.1 清单 + §5.2「semaphore 也有 PI」"),
    ("chmtx.c",       "互斥量 + PI", FULL, "第 12 课 §1~§4 整课（chMtx* 48 处）"),
    ("chsys.c",       "系统/中断",   FULL, "★ 第 15 课 §1 整节（锁的三个层次、解锁断言、chSysInit、完整性检查、tick 处理、环绕安全比较）；2026-09 曾经被误判为已覆盖"),
    ("chvt.c",        "虚拟定时器",   FULL, "第 12 课 §5.3（栈上 virtual_timer_t）+ 第 11 课 §2"),
    ("chmsg.c",       "消息",        FULL, "第 21 课 §5 专门一节（chMsgSend 12 处 / chMsgWait 6 处）"),
    ("chcond.c",      "条件变量",    FULL, "★ 第 15 课 §2 整节（结构体只 1 个字段、五步等待、fifo_remove 不是 FIFO、MSG_RESET、超时丢锁）"),
    ("chevents.c",    "事件",        FULL, "★ 第 15 课 §3 整节（位在线程里、判据在发送方、WaitOne/WaitAny 清几位、配置组合决定 API）+ 第 21 课 §6 对照"),
    ("chdynamic.c",   "动态线程",    NONE, "第 10 课门控表（CH_CFG_USE_DYNAMIC 编译错误），未讲 API"),
    ("chregistry.c",  "线程注册表",   NONE, "第 10 课文件清单 + 第 21 课 1 处 chRegSetThreadName"),
    ("chtm.c",        "时间测量 TM", NONE, "第 10 课文件清单 + NIL 对照表；chTMStartMeasurement 零提及"),
    ("chstats.c",     "统计",        NONE, "第 10 课门控对照表（NIL 侧 #error \"statistics not yet supported\"）"),
    ("chtrace.c",     "trace",      NONE, "只在第 10 课文件清单"),
    ("chdebug.c",     "调试",        NONE, "未提"),
    ("chinstances.c", "实例",        NONE, "未提"),
    ("chrfcu.c",      "RFU",        NONE, "未提"),
]

# ChibiOS 其余子树（行数由脚本算，备注人工写）
CHIBIOS_SUBTREES = [
    ("os/rt",      "内核",              "第 10~12 课讲了 os/rt/src 里的 7 个 .c（另有第 21 课讲 chmsg）"),
    ("os/nil",     "NIL 内核",          "第 10/11 课（对比性质，不逐行）"),
    ("os/hal",     "HAL 与驱动",         "★ 第 13 课只讲【架构】，见下拆分"),
    ("os/oslib",   "内核级工具库",        "★ 第 16 课整课（内存四层：core/heap/pool；其余模块见 §5 的\"谁在用\"实测清单）"),
    ("os/various", "可选组件",           "FatFS/LittleFS/lwIP/USB/wolfSSL/httpd/shell —— 只在绕过清单里"),
    ("os/ex",      "示例",              "未提（不是教材内容）"),
    # ★ 这一行的备注 2026-09 改过一次。原文是
    #   「未提（CMSIS 头文件，不是教材内容）」—— **不准确**：
    #   这里除了 380 万行 CMSIS 头，还有 **47 个 .S / 21,132 行汇编**，
    #   其中就有 `chcoreasm.S`（Cortex-M 的上下文切换汇编）
    #   和 `crt0`/`vectors`（启动代码）—— 那些**和教程直接相关**
    #   （第 2/4 课的 PendSV、第 10 课的 port 层都在讲同一件事的 FreeRTOS 版）。
    #   ⇒ 是「没讲」，不是「不是教材内容」。这个区别对读者是重要的。
    ("os/common",  "MCU 启动/寄存器头",   "⚠️ **没讲**：含 47 个 .S / 21,132 行"
                                          "（`chcoreasm.S` 上下文切换汇编、"
                                          "`crt0`/`vectors` 启动代码）；"
                                          "其余约 380 万行是 CMSIS 头（确实不是教材内容）"),
]

# os/hal 的拆分 —— 这是「只讲架构没讲驱动」这句话的量化依据
HAL_SPLIT = [
    ("os/hal/src",       "HAL 框架实现",     "第 13 课讲架构（四层模型），第 17 课讲三条线的实现（队列/中断/VMT）"),
    ("os/hal/include",   "HAL 接口定义",     "同上"),
    ("os/hal/ports",     "★ 各 MCU 具体驱动", "STM32/GD32/... 本教程没讲"),
    ("os/hal/boards",    "板级定义",         "ArduPilot 用自己的 hwdef，不走这里"),
    ("os/hal/templates", "模板",            "—"),
    ("os/hal/lib",       "外设辅助库",        "没讲"),
    ("os/hal/osal",      "OSAL 适配层",      "第 10 课 §1.3 提过「三种变体」"),
]


def one(path):
    """数行数 —— 本教程的**唯一口径**，所有数字都用它。

    ★ 不要写 `len(f.read().split('\\n'))`：
      文件以换行结尾时这会**多算一行**（split 出的最后一段是空串）。
      本教程作者第一版就这么写的，于是审计说 `tasks.c` 8697 行，
      而第 1 课正文写的是 8696 —— 同一份教程里出现两个矛盾的数字。

    ⚠️ **但它和 `wc -l` 并不完全等价**（这个 docstring 原来写的是
      "必须和 `wc -l` 一致" —— **那句话不准确**，是本轮自检抓出来的）：
      两个口径只在一种情况下分叉 —— **文件末尾没有换行符**时，
      `wc -l` 把最后那行**不算**，而这个函数**算**（因为那行确实是内容）。

      实测：ChibiOS 的 `os/hal` 下恰好有 3 个文件是这种情况
      （`hal_eicu.h` / `hal_eicu_lld.h` / `hal_eicu.c`，同一批生成的），
      于是 `os/hal` 的总数在 **485,558（本函数）** 与
      **485,555（`wc -l`）** 之间摆动。

      ⇒ **本教程统一用本函数的口径**（末尾无换行的那行也是代码，读者关心的是内容量）。
        `os/hal` 因此是 **485,558** —— 课文里所有写 485,555 的地方
        在 2026-09 已统一改过来。
    """
    try:
        s = io.open(path, encoding='utf-8', errors='replace').read()
    except OSError:
        return 0
    return s.count('\n') + (0 if s.endswith('\n') or not s else 1)


# ★ "长得像源文件、但没被 count_lines 计入"的后缀。
#   列出来是为了让 count_lines 报警，而不是为了自动计入 ——
#   "要不要算 .S（汇编）/ .inc（被 include 的片段）"是**需要人判断**的：
#   算 .inc 会和 include 它的 .c 重复计数。
#   ⇒ 所以策略是：**机器负责发现，人负责决定**。
SRC_LIKE_EXT = ('.cpp', '.hpp', '.cc', '.cxx', '.m', '.mm',
                '.asm', '.cs', '.rs', '.zig')
# （`.S`/`.s` 不在这个列表里 —— 它们**已经**被计入了，见 count_lines。）


def count_lines(root, warn=True):
    """递归数 .c/.h/.S/.s 行数。行数口径同 `one()`（**不是** `wc -l`，
    两者对"末尾无换行"的文件差一行，见 `one()` 的 docstring）。

    ★ 也不要用 `find ... | xargs wc -l | tail -1`：
      文件多了 xargs 会分批调用 wc，每批各印一个 total，
      tail -1 只拿到最后一批 —— 作者就因此把 os/hal 数成了 158,410。

    ★★ 而这个函数**自己错过两次**（2026-09）：它只认 .c/.h，
       于是 `os/various/cpp_wrappers`（全是 .cpp/.hpp）被数成 0 行，
       3,178 行凭空消失 —— 而且**在两处独立发生**。

       ⇒ 现在的防法是：**让函数自己报告"我遇到了没计入的源文件后缀"**。
         只报警、不改行为（因为"要不要算 .inc"这类判断需要人做）。
         这样同一个 bug 不会以第三种形式出现第三次。
    """
    tot = n = 0
    unknown = set()
    for dirpath, dirnames, filenames in os.walk(root):
        for f in filenames:
            # ★ `.S`/`.s` 也要算 —— 它们是**独立编译单元**（会被编成 .o），
            #   和 `.inc`（被 include 的片段，算了会和宿主 .c 重复计数）不同。
            #   2026-09 发现：`os/common` 里有 47 个 .S / 21,132 行没被算，
            #   而其中就有 **ARMv7-M 的 `chcoreasm.S`（上下文切换汇编）**
            #   和 `crt0`/`vectors`（启动代码）—— 这**不是**"不是教材内容"。
            #   （报警机制抓到它的：见本函数末尾。）
            if f.endswith(('.c', '.h', '.S', '.s')):
                tot += one(os.path.join(dirpath, f))
                n += 1
            elif f.endswith(SRC_LIKE_EXT):
                unknown.add(f.rsplit('.', 1)[-1])
    if unknown and warn:
        print("  ⚠️  count_lines(%s)：发现未计入的源文件后缀 %s ——"
              % (os.path.relpath(root, CHIBIOS) if root.startswith(CHIBIOS)
                 else root, sorted(unknown)))
        print("      这个目录下还有**没被算进行数**的代码。"
              "（这正是 cpp_wrappers 那个 bug 的形态，见本函数注释。）")
    return tot, n


def mark(st):
    return {FULL: "✅ 专门讲", MENTION: "⚠️ 仅提及", NONE: "❌ 没讲"}[st]


def main():
    print("=" * 80)
    print("覆盖边界审计")
    print("=" * 80)

    # ---------------------------------------------------------- FreeRTOS
    print("\n【FreeRTOS 内核】%s\n" % FREERTOS)
    tot = full = 0
    print("  %-17s %-16s %8s  %-10s %s" % ("文件", "模块", "行数", "状态", "依据"))
    print("  " + "-" * 92)
    for fn, name, st, why in FREERTOS_KERNEL:
        n = one(os.path.join(FREERTOS, fn))
        tot += n
        if st == FULL:
            full += n
        print("  %-17s %-16s %8d  %-10s %s" % (fn, name, n, mark(st), why))
    print("  " + "-" * 92)
    print("  合计 %d 行，其中【专门讲过】%d 行 = %.0f%%" % (tot, full, 100.0 * full / tot))

    print("\n  无独立源文件的特性：")
    for name, st, why in FREERTOS_FEATURES:
        print("  %-17s %-16s %8s  %-10s %s" % ("—", name, "—", mark(st), why))

    # ---------------------------------------------------------- ChibiOS
    print("\n\n【ChibiOS 内核】%s/os/rt/src\n" % CHIBIOS)
    rt = os.path.join(CHIBIOS, "os/rt/src")
    tot = full = 0
    print("  %-17s %-16s %8s  %-10s %s" % ("文件", "模块", "行数", "状态", "依据"))
    print("  " + "-" * 92)
    for fn, name, st, why in CHIBIOS_RT:
        n = one(os.path.join(rt, fn))
        tot += n
        if st == FULL:
            full += n
        print("  %-17s %-16s %8d  %-10s %s" % (fn, name, n, mark(st), why))
    print("  " + "-" * 92)
    print("  合计 %d 行，其中【专门讲过】%d 行 = %.0f%%" % (tot, full, 100.0 * full / tot))

    print("\n\n【ChibiOS 其余子树】\n")
    print("  %-14s %-18s %9s  %s" % ("子树", "内容", "行数", "本教程"))
    print("  " + "-" * 92)
    for sub, note, why in CHIBIOS_SUBTREES:
        # os/various 下面单独补了 C++ 兜底（见【os/various】那一节）⇒ 静音
        n, nf = count_lines(os.path.join(CHIBIOS, sub),
                            warn=(sub != 'os/various'))
        print("  %-14s %-18s %9d  %s" % (sub, note, n, why))

    print("\n\n【★ os/hal 的拆分】—— 「只讲架构、没讲驱动」的量化依据\n")
    print("  %-20s %-18s %9s  %s" % ("子树", "内容", "行数", "本教程"))
    print("  " + "-" * 92)
    for sub, note, why in HAL_SPLIT:
        n, nf = count_lines(os.path.join(CHIBIOS, sub))
        print("  %-20s %-18s %9d  %s" % (sub, note, n, why))
    hal_tot, _ = count_lines(os.path.join(CHIBIOS, "os/hal"))
    frame, _ = count_lines(os.path.join(CHIBIOS, "os/hal/src"))
    fi, _ = count_lines(os.path.join(CHIBIOS, "os/hal/include"))
    ports, _ = count_lines(os.path.join(CHIBIOS, "os/hal/ports"))
    listed = sum(count_lines(os.path.join(CHIBIOS, s))[0]
                 for s, _, _ in HAL_SPLIT)
    print("\n  ⇒ os/hal 共 %d 行，其中：" % hal_tot)
    print("       框架本体（src+include）%d 行 = %.0f%%   ← 第 13 课讲的是这部分"
          % (frame + fi, 100.0 * (frame + fi) / hal_tot))
    print("       各 MCU 具体驱动（ports）%d 行 = %.0f%%"
          % (ports, 100.0 * ports / hal_tot))
    # ★ 第 17 课起，ports 里【有了一小部分】被讲到 —— 要说清楚是多少，
    #   不然「没讲」这个词就成了不准确的。
    # ★ 第 17 课讲了三支（数据搬运类），第 18 课又讲了三支（时间/模拟量类）。
    #   两组分开算 —— 总纲的路线图里按课分别引用了这两个数。
    #   注意 FDCANv1 虽然被 AP 关掉了，但教程里讲了它（§4），所以算"讲到"。
    covered_ports = 0
    covered_names = []
    group_sums = []
    for group in (("USARTv3", "SPIv3", "I2Cv3"), ("ADCv4", "TIMv1", "FDCANv1")):
        gsum = 0
        for lld in group:
            n, _ = count_lines(os.path.join(CHIBIOS, "os/hal/ports/STM32/LLD", lld))
            covered_ports += n
            gsum += n
            covered_names.append("%s(%d)" % (lld, n))
        group_sums.append(gsum)
    print("         └ 其中第 17、18 课讲了六支 LLD：")
    print("              第 17 课：%s = %d 行"
          % (" + ".join(covered_names[:3]), group_sums[0]))
    print("              第 18 课：%s = %d 行"
          % (" + ".join(covered_names[3:]), group_sums[1]))
    print("              合计 %d 行（占 ports 的 %.1f%%）"
          % (covered_ports, 100.0 * covered_ports / ports))
    print("            ⇒ 都是 H7 用的版本；F4 用的是 v1 系列（第 17 课 §1.4）")
    print("            ⇒ 注意 FDCANv1 虽然被 AP 关掉了（halconf.h:68），"
          "但教程里讲了它")
    # ★ 上表各项之和必须等于总数，否则说明漏列了子树 —— 别让读者自己发现对不上
    assert listed == hal_tot, \
        "HAL_SPLIT 各项之和 %d ≠ os/hal 总数 %d，有子树没列出来" % (listed, hal_tot)
    print("       （上表 7 项之和 %d = 总数，无遗漏 ✅）" % listed)

    # ---------------------------------------------------------- MPU / SMP
    #
    # ★ 第 9 课的核心问题是「两个飞控用不用 MPU/SMP」。
    #   那一课的方法就是"先数，再写"—— 所以这几个数字也必须由脚本算，
    #   不能手写在文档里（作者手写了一次，自检立刻报了「没出处」）。
    print("\n\n【MPU / SMP：两个飞控用不用】（第 9 课）\n")
    # ★ apm_root 在下面 os/various 那节也要用 —— 定义放在这里（靠前）。
    #   作者第一版把它写在后面，结果这一节 UnboundLocalError
    #   （同一个函数里"用到时还没定义"，Python 直接报未绑定）。
    apm_root = os.path.dirname(os.path.dirname(CHIBIOS))
    # ★ 是【三级】`..` 才到 PX4APM：
    #   APM/ardupilot-Copter-4.5.7/ardupilot-Copter-4.5.7 → 上三级才是根。
    #   写两级的话目录不存在，count_files 会返回 None（静默变成"—"）。
    px4_boards = os.path.normpath(os.path.join(apm_root, "..", "..", "..",
                                               "PX4", "PX4-Autopilot", "boards"))

    def count_files(root, pattern, names=None):
        """数命中 pattern 的文件数（带 re.M —— 行首锚点必须多行模式）。"""
        if not os.path.isdir(root):
            return None
        rx = re.compile(pattern, re.M)
        n = 0
        for dirpath, dirs, files in os.walk(root):
            dirs[:] = [d for d in dirs if d not in ('.git', 'build', '__pycache__')]
            for f in files:
                if names and not any(f.endswith(x) for x in names):
                    continue
                try:
                    if rx.search(io.open(os.path.join(dirpath, f),
                                         encoding='utf-8', errors='replace').read()):
                        n += 1
                except OSError:
                    pass
        return n

    ap_lib = os.path.join(apm_root, "libraries")
    n_ap_mpu = count_files(ap_lib,
                           r"configENABLE_MPU|xTaskCreateRestricted|MPU_xTaskCreate")
    n_px4_mpu = count_files(px4_boards, r"^CONFIG_ARM_MPU=y")
    n_px4_rst = count_files(px4_boards, r"^CONFIG_ARM_MPU_EARLY_RESET=y")
    n_px4_all = count_files(px4_boards, r"^CONFIG_", names=("defconfig",))
    n_px4_smp = count_files(px4_boards, r"^CONFIG_SMP=y")
    print("  %-46s %s" % ("ArduPilot 里 MPU 关键词的命中文件数",
                          n_ap_mpu if n_ap_mpu is not None else "—"))
    print("  %-46s %s / %s" % ("PX4 开 CONFIG_ARM_MPU=y 的 defconfig",
                               n_px4_mpu, n_px4_all))
    print("  %-46s %s" % ("PX4 开 CONFIG_ARM_MPU_EARLY_RESET=y 的（★ 是关掉 MPU）",
                          n_px4_rst))
    print("  %-46s %s" % ("PX4 开 CONFIG_SMP=y 的 defconfig", n_px4_smp))
    print("  ⇒ ★ 两个飞控【默认都不用】这两个特性 —— 第 9 课讲的就是为什么")

    # ---------------------------------------------------------- os/various
    #
    # ★ 第 19 课的核心问题是「AP 到底编了 os/various 的哪几个」。
    #   答案在 board makefile 里，所以这个数字**必须让脚本去解析**，
    #   不能手写在文档里（作者第一版就是手写的，自检立刻报了「没出处」）。
    print("\n\n【os/various：AP 编了哪几个】（第 19 课）\n")
    board_mk = os.path.join(apm_root, "libraries", "AP_HAL_ChibiOS",
                            "hwdef", "common", "chibios_board.mk")
    mk = io.open(board_mk, encoding='utf-8', errors='replace').read() \
        if os.path.exists(board_mk) else ""
    used_mods = set(re.findall(r'os/various/(\w+)/', mk))
    various = os.path.join(CHIBIOS, "os", "various")
    used_lines = unused_lines = 0
    print("  %-24s %8s  %s" % ("模块", "行数", "AP"))
    print("  " + "-" * 56)
    # ★ 顶层散文件也要算。脚本原来只数【子目录】（下面那个 isdir 判断），
    #   把 syscalls.c / ramdisk.c / evtimer.c / evtimer.h / ramdisk.h
    #   这一层整个漏掉了 —— 第 19 课收尾核实时发现少了 714 行。
    #
    # ★ 顶层散文件的归属**要逐个看**，不能笼统归类。实测：
    #
    #   ✅ syscalls.c —— `cpp_wrappers/chcpp.mk:2` 明写
    #         CHCPPSRC = $(CHIBIOS)/os/various/syscalls.c
    #   ✅ evtimer.c  —— 不在 board.mk 里，但被 AP 自己的包装二次 include：
    #         `AP_Networking/lwip_hal/arch/evtimer.c:9` 的
    #         `#include <../evtimer.c>`（外面套了 `#if CH_CFG_USE_EVENTS`）
    #         ⇒ **间接被编**，算"编"
    #   ❌ evtimer.h / ramdisk.c / ramdisk.h —— board.mk 零命中，也没有消费者
    #
    #   ⚠️ 作者第一版把 evtimer.c 也归成了"不编"（只查了 board.mk），
    #      于是"编了"少了 87 行。**"不在这个文件里"不等于"没被编"** ——
    #      这条和第 19 课 §4.2「AP 不用 lwIP」那个错误是同一个形态。
    TOP_USED = ('syscalls.c', 'evtimer.c')
    top_lines = top_unused = 0
    for f in sorted(os.listdir(various)):
        p = os.path.join(various, f)
        if os.path.isfile(p) and f.endswith(('.c', '.h')):
            n = one(p)
            if f in TOP_USED:
                top_lines += n
                print("  %-24s %8d  ✅ 编（chcpp.mk）" % (f, n))
            else:
                top_unused += n
                print("  %-24s %8d  ❌ 不在 board.mk 里" % (f, n))
    for d in sorted(os.listdir(various)):
        p = os.path.join(various, d)
        if not os.path.isdir(p):
            continue
        n, _ = count_lines(p, warn=False)   # 下面有 C++ 兜底
        # ★ count_lines 只认 .c/.h —— 而 cpp_wrappers 里**只有** .cpp/.hpp，
        #   于是它被数成 0 行。这不是"没内容"，是"用 C++ 写的"：
        #   ch.hpp 3,133 + ch.cpp 45 = 3,178 行。
        #   ⚠️ 一个 0 出现在表格里，很容易被读成"这个模块是空的" ——
        #   第 19 课初版就是这么把它漏掉的。
        if n == 0:
            for dp, _, fns in os.walk(p):
                for fn in fns:
                    if fn.endswith(('.cpp', '.hpp', '.cc')):
                        n += one(os.path.join(dp, fn))
        used = d in used_mods
        if used:
            used_lines += n
        else:
            unused_lines += n
        print("  %-24s %8d  %s" % (d, n, "✅ 编" if used else "❌ 不编"))
    print("  " + "-" * 56)
    used_lines += top_lines
    unused_lines += top_unused
    print("  ⇒ AP 编了 %d 个子目录 + %d 个顶层文件 = %d 行；"
          "关掉 %d 行（判据：%s）"
          % (len(used_mods), len(TOP_USED), used_lines, unused_lines,
             os.path.basename(board_mk)))
    print("     （合计 %d 行 —— 这个数就是 §1 里那个「os/various 总量」）"
          % (used_lines + unused_lines))

    # ---------------------------------------------------------- 篇幅对照
    print("\n\n【篇幅对照】\n")
    lessons = []
    for d in sorted(os.listdir('.')):
        # ★ 判据写成 d[:2].isdigit() 而不是 d[0].isdigit() and d[1:3]=='-' ——
        #   后者对 '01-FreeRTOS' 取 d[1:3] 得到 '1-'，永远不匹配。
        if os.path.isdir(d) and d[:2].isdigit() and d[2:3] == '-':
            for f in sorted(os.listdir(d)):
                if f.endswith('.md'):
                    lessons.append((d, f, one(os.path.join(d, f))))
    total = sum(x[2] for x in lessons)
    # ★ 课号区间【从目录里的文件名算】，不要硬编码：
    #   作者第一版写的是「第 1~6 课 / 第 7~11 课 / 第 12~13 课」，
    #   2026-09 全局重排课号后这三行全错，而数字（行数）还是对的 ——
    #   最难发现的就是这种「数字对、标签错」。
    def span(prefix):
        nums = sorted(int(f[:2]) for d, f, _ in lessons if d.startswith(prefix))
        return nums, sum(n for d, _, n in lessons if d.startswith(prefix))
    frn, fr = span('01-')
    chn, ch = span('02-')
    hln, hl = span('03-')
    print("  本线：%d 课 / %d 行" % (len(lessons), total))
    print("        FreeRTOS 部分（第 %d~%d 课）  %6d 行" % (frn[0], frn[-1], fr))
    print("        ChibiOS  部分（第 %d~%d 课） %6d 行" % (chn[0], chn[-1], ch))
    print("        合流     部分（第 %d~%d 课）  %6d 行" % (hln[0], hln[-1], hl))
    # ★ 总纲的对照表用的是【每个 RTOS 分到多少】= 总行数 ÷ 2，
    #   而不是按目录求和 —— 合流那两课是两个 RTOS 共用的，不能只算给一边。
    print("        ⇒ 每个 RTOS 分到 %d 行（= %d ÷ 2，合流部分两边共有）"
          % (total // 2, total))
    # ---------------------------------------------------------- 派生数字
    #
    # ★ 总纲的「覆盖边界」一节里会出现一些【组合求和】的数字
    #   （比如「事件组 + 流缓冲 = 2,604 行」）。这些也要让脚本算出来，
    #   否则文档里的数字就成了手算的，没人能核对。
    NUTTX_DOC = _srcdirs.src("DOC_NUTTX")
    print("\n\n【派生数字】—— 总纲里那些组合/对照数字的出处\n")
    frmap = {fn: one(os.path.join(FREERTOS, fn)) for fn, _, _, _ in FREERTOS_KERNEL}
    rtmap = {fn: one(os.path.join(rt, fn)) for fn, _, _, _ in CHIBIOS_RT}
    # ★ hal 在下面自检时还要用，所以在这里就算好，别等到自检再定义 ——
    #   作者第一版把定义写在自检块里，结果 200 行之前的 print 引用它直接 UnboundLocalError。
    hal = {sub: (count_lines(os.path.join(CHIBIOS, sub), warn=False)[0]
                 if sub == 'os/various'            # 它下面单独补了 C++ 兜底
                 else count_lines(os.path.join(CHIBIOS, sub))[0])
           for sub, _, _ in CHIBIOS_SUBTREES + HAL_SPLIT}
    # ★ 但 count_lines 只认 .c/.h —— 而 `os/various/cpp_wrappers` 里
    #   **全是** .cpp/.hpp（ch.hpp 3,133 + ch.cpp 45 = 3,178 行），
    #   于是这一项被少算了 3,178 行（5,771 vs 8,949）。
    #
    # ★★ 这是**同一个 bug 的第二个实例**：下面【os/various】那一节
    #   也犯了同样的错（那里已经单独注释过）。两处独立地栽在
    #   "只认后缀的统计函数遇到混了别的后缀的目录"上。
    #   ⇒ 写统计函数时，"这个目录里会不会有别的后缀"要和"后缀列表"一起想。
    for sub in ('os/various',):
        for dp, _, fns in os.walk(os.path.join(CHIBIOS, sub)):
            for fn in fns:
                if fn.endswith(('.cpp', '.hpp', '.cc')):
                    hal[sub] += one(os.path.join(dp, fn))
    # ★ 档位要【现算】，不要写死「讲的 7 个」—— chsys.c 降级后就变成 6 个了。
    n_full = sum(1 for _, _, st, _ in CHIBIOS_RT if st == FULL)
    n_none = sum(1 for _, _, st, _ in CHIBIOS_RT if st != FULL)
    none_lines = sum(rtmap.values()) - sum(rtmap[f] for f, _, st, _ in CHIBIOS_RT
                                           if st == FULL)
    print("  FreeRTOS 未专门讲 = %d（只有协程 %d 行；MPU/SMP 无独立 .c）"
          % (frmap['croutine.c'], frmap['croutine.c']))
    print("  ChibiOS  未专门讲 = %d 行（%d 个 .c 减去专门讲过的 %d 个）"
          % (none_lines, len(CHIBIOS_RT), n_full))
    print("  ⇒ 第 15 课补完之后，剩余未覆盖的源码量：")
    print("      os/rt/src 剩下的 %d 个 .c = %d 行（全是周边设施，不再单开课）"
          % (n_none, none_lines))
    print("         %s" % "、".join(
        "%s(%d)" % (f.replace('.c', ''), rtmap[f])
        for f, _, st, _ in CHIBIOS_RT if st != FULL))
    print("      os/oslib  内存池/邮箱/管道/作业队列 = %d 行" % hal.get('os/oslib', 0))
    print("      os/hal/ports 各 MCU 具体驱动       = %d 行" % hal.get('os/hal/ports', 0))
    print("      os/various FatFS/lwIP/USB       = %d 行" % hal.get('os/various', 0))
    # 这两行是【已完成】的历史数字，总纲里用删除线保留着
    print("  （已完成）事件组 + 流缓冲       = %d 行 —— 第 7 课"
          % (frmap['event_groups.c'] + frmap['stream_buffer.c']))
    print("  （已完成）条件变量 + 事件 + 消息 = %d 行 —— 消息部分见第 21 课 §5"
          % (rtmap['chcond.c'] + rtmap['chevents.c'] + rtmap['chmsg.c']))
    nuts = []
    if os.path.isdir(NUTTX_DOC):
        for d in sorted(os.listdir(NUTTX_DOC)):
            sub = os.path.join(NUTTX_DOC, d)
            if os.path.isdir(sub):
                for f in sorted(os.listdir(sub)):
                    # ★ 只数【课程本体】—— 排除总纲 / 资料索引 / README / 参考手册，
                    #   否则会数成 21 课 24,936 行，和「11 课」这个说法对不上。
                    if f.endswith('.md') and not re.match(r'^(00-|README|SUMMARY)', f):
                        nuts.append((f, one(os.path.join(sub, f))))
        tot_n = sum(n for _, n in nuts)
        sync = [n for f, n in nuts if '同步机制' in f]
        print("  对照《NuttX 教程》：%d 课 / %d 行（只覆盖 NuttX 一个 RTOS）"
              % (len(nuts), tot_n))
        if sync:
            print("      其中单独一课「同步机制」= %d 行" % sync[0])
        print("      ⇒ 本线每个 RTOS 分到 %d 行 ≈ NuttX 的 1/%.1f"
              % (total // 2, tot_n / (total / 2.0)))
        # ★ 总纲里那句「NuttX 单独一课『同步机制』相当于本线整个 FreeRTOS 部分的 44%」
        #   也要让脚本算出来 —— 作者第一版这里写的是「比一半还多」，
        #   而 3,526 < 7,954÷2。**手算的结论没人核对，脚本算的才有人核对。**
        if sync:
            half = fr / 2.0
            print("      本线 FreeRTOS 部分 %d 行，一半 = %.1f 行；"
                  "NuttX 那一课 %d 行 = 它的 %.0f%%"
                  % (fr, half, sync[0], 100.0 * sync[0] / fr))

    # ================================================================
    # 自检：总纲「覆盖边界」一节里的数字，必须能在【本次输出】里找到
    # ================================================================
    #
    # ★ 为什么需要这一步：
    #   作者同步这批数字时，用了一次【全局替换】把旧值换成新值 ——
    #   结果连「课程列表」里第 1 课的行数（885）也被顺手改成了 884，
    #   而那个 885 跟本次审计毫无关系。
    #   这正是本教程反复记录的错误形态：**改动的影响面没有追踪**。
    #
    #   ⇒ 所以让脚本自己把「文档里的数字」和「算出来的数字」对一遍。
    #     数字对不上就报错，而不是等读者发现。
    print("\n\n【自检】总纲「覆盖边界」一节的数字 vs 本次输出\n")
    import contextlib
    buf = io.StringIO()
    # 重新收集本次已经打印过的所有内容不好取，改为直接核对关键量
    doc = io.open('00-总纲.md', encoding='utf-8').read()
    if '## ★ 覆盖边界' in doc:
        sec = doc.split('## ★ 覆盖边界')[1].split('## ★ 已复核')[0]
        # ★ known 必须把脚本【打印过的所有数字】都收进来，包括聚合值。
        #   作者第一版只收了逐文件的值，结果自检把「合计 16,653」
        #   这类小计也报成了「找不到出处」—— 又是一种误报。
        # frmap / rtmap / hal 在上面已经算过了，这里复用
        known = set()
        known |= {str(v) for v in frmap.values()}
        known |= {str(v) for v in rtmap.values()}
        known |= {str(v) for v in hal.values()}
        # 聚合值
        known |= {str(sum(frmap.values())),                                  # FreeRTOS 合计
                  str(sum(frmap[f] for f, _, st, _ in FREERTOS_KERNEL if st == FULL)),
                  str(sum(rtmap.values())),                                  # ChibiOS 合计
                  str(sum(rtmap[f] for f, _, st, _ in CHIBIOS_RT if st == FULL)),
                  str(sum(rtmap.values()) - sum(rtmap[f] for f, _, st, _ in CHIBIOS_RT if st == FULL)),
                  str(frmap['event_groups.c'] + frmap['stream_buffer.c']),
                  str(rtmap['chcond.c'] + rtmap['chevents.c'] + rtmap['chmsg.c']),
                  str(rtmap['chcond.c'] + rtmap['chevents.c']),   # 剩余：条件变量+事件
                  str(rtmap['chcond.c'] + rtmap['chevents.c'] + rtmap['chsys.c']),
                  str(total // 2),                                # 每个 RTOS 分到的行数
                  str(fr), str(fr // 2),                          # FreeRTOS 部分 / 其一半
                  str(hal.get('os/hal/src', 0) + hal.get('os/hal/include', 0)),  # HAL 框架本体
                  str(covered_ports),                    # 六支 LLD 合计
                  str(group_sums[0]), str(group_sums[1]),  # 第 17 / 18 课各三支
                  str(total), str(fr), str(ch), str(hl),
                  str(used_lines), str(unused_lines),   # os/various（第 19 课）
                  str(n_px4_mpu), str(n_px4_rst), str(n_px4_all),  # MPU/SMP（第 9 课）
                  }
        # ★ 历史值：总纲里【故意保留】的旧数字 —— 删除线里的（`~~5,771 行~~`）、
        #   以及"原来是 X，现在是 Y"这类叙述里的。
        #   **它们不是错，是记录**；但通用扫描会把它们当成"无出处的数字"报出来。
        #
        #   ⚠️ 这又暴露了同一个盲区（本文件顶上记过三次了）：
        #     **扫描器分不清"当前断言"和"历史引用"** ——
        #     和"引文 vs 断言"、"文档命令 vs 复现命令"是同一类问题。
        #   目前的修法是显式列出，不是让扫描器变聪明。
        known |= {'5771', '8524', '10388', '20777',      # 补齐前的聚合值
                  '4262', '1594', '1351', '891',         # 补齐前的派生值
                  # 各轮补齐新增的行数（随轮次变，逐个登记）
                  '10550', '14417', '14447', '14507', '14514', '14524',
                  # ★ 2026-09 复核：**补齐完成当时**的合计行数。
                  #   总纲「覆盖边界」一节里那句"全线从 20,777 涨到 35,291"
                  #   记的是**那次补齐事件**，属于历史叙述，不是当前断言 ——
                  #   和 '20777'（补齐前）配对出现。
                  #   现值（36,045）走 str(total) 那条自动进 known，
                  #   两者都在文档里，谁也别顶替谁。
                  #   ⚠️ 这正是本文件自检设计里那个老问题（"引文 vs 断言"）的
                  #      又一个实例：**同一个数字，角色不同，判据就该不同**。
                  '35291'}
        if nuts:
            known.add(str(tot_n))
            known |= {str(n) for _, n in nuts}
        # 百分比和日期不是数据
        sec_no_pct = re.sub(r'\d{3,}%', '', sec.replace(',', ''))
        nums = re.findall(r'\b\d{3,}\b', sec_no_pct)
        nums = [n for n in nums if not (n.startswith('20') and len(n) == 4)]
        bad = [n for n in sorted(set(nums), key=int) if n not in known]
        if bad:
            print("  ❌ 这些数字在本次输出里找不到出处：%s" % bad)
            print("     （很可能是手改文档时漏改/改错，或改到了别的段落）")
            return 1
        print("  ✅ 一节里的 %d 个数字全部有出处" % len(set(nums)))
        #
        # ★ 比值单独核对 —— 上面那个通用正则 `\b\d{3,}\b` 只抓【三位以上整数】，
        #   所以「≈ NuttX 的 1/2.4」这种【一位小数】它一个字符都抓不到。
        #   作者实测：脚本一直算的是 1/2.3（24,045 ÷ 10,377 = 2.32），
        #   而总纲两处写的是 1/2.4 —— **飘了很久，通用自检一次都没报**。
        #   这不是自检写错了，是它的【扫描面】没覆盖到这种形态。
        #   ⇒ 给比值单开一条检查，别指望通用正则。
        if nuts:
            ratio = tot_n / (total / 2.0)
            # ★ 扫【全文】而不是只扫「覆盖边界」一节 ——
            #   那个比值在开头「定位」引用块里也出现了一次，
            #   只扫一节会漏掉它（作者第一版就是这样，报了「核对 1 处」）。
            #
            # ★ 但扫全文立刻带出第二个问题：**引文也被当成断言扫了**。
            #   本节的说明文字里为了举例，写了「总纲原来写的是 1/2.4」——
            #   那是【引用一个历史错值】，不是断言，却照样被报成错。
            #   这和 ① 是同一个病：**判据没区分"这一处的角色"**。
            #   ⇒ 扫描前先剥掉【行内反引号】：`x` 在本文档里是**引文**。
            #
            #   ⚠ 但**不能连代码块一起剥** —— 作者第一版就是那么写的，
            #     结果把开头「定位」块里那处【真断言】也剥没了
            #     （那处为了排版用 ``` 包着），检查器反而少查了一处。
            #     ⇒ 本文档里两种标记的角色是**相反**的：
            #         `行内反引号`   = 引文，剥掉
            #         ```代码块```    = 示例/清单，里面是**真数字**，留着
            plain = re.sub(r'`[^`\n]*`', '', doc)
            got = re.findall(r'1/(\d+\.\d+)', plain)
            off = [g for g in got if abs(float(g) - ratio) >= 0.05]
            if off:
                print("  ❌ 总纲写的是「1/%s」，脚本算出的是「1/%.1f」"
                      "（%d ÷ %d = %.2f）"
                      % (off[0], ratio, tot_n, total // 2, ratio))
                return 1
            if got:
                print("  ✅ 篇幅比值 1/%.1f 与总纲一致（全文 %d 处）"
                      % (ratio, len(got)))
    else:
        print("  ⚠️ 总纲里没有「覆盖边界」一节，跳过")
    return 0


if __name__ == '__main__':
    sys.exit(main())
