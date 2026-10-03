#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 13 课配套实验：ChibiOS 的 HAL 与板级支持

    实验 1  三层配置的依赖关系（从源码读 include）
    实验 2  驱动模型的四层（量化行数）
    实验 3  ★ 板级层：跑生成器（或读已生成的产物）
    实验 4  用了什么、绕过了什么（核实 + 量化）

用法：
    python lesson13_chibios_hal.py 1
    python lesson13_chibios_hal.py 2
    python lesson13_chibios_hal.py 3
    python lesson13_chibios_hal.py 4
    python lesson13_chibios_hal.py          # 全部

【实验 3 说明】

实验 3 需要一份生成器的输出。两种来源：
    (a) 本机已有 /tmp/bl2（或 %TEMP%/bl2）—— 直接用
    (b) 没有 —— 脚本会打印生成命令，你自己跑一遍

生成命令（约 30~60 秒）：
    python libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py \
        --bootloader --outdir <目录> \
        libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef-bl.dat

★ 本实验的方法论要点：**能跑生成器就跑，别看它的代码。**

源码版本：6a85082c（2023-12-29，ChibiOS 内核 7.0.4）
"""

import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APM_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "APM",
                 "ardupilot-Copter-4.5.7", "ardupilot-Copter-4.5.7")
)
CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")
HWDEF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS", "hwdef")
COMMON = os.path.join(HWDEF, "common")
HALCHIB = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS")

F_CHCONF = os.path.join(COMMON, "chconf.h")
F_HALCONF = os.path.join(COMMON, "halconf.h")
F_MCUCONF = os.path.join(COMMON, "mcuconf.h")
F_BOARD_H = os.path.join(COMMON, "board.h")
F_BOARD_C = os.path.join(COMMON, "board.c")
F_HWDEF_PY = os.path.join(HWDEF, "scripts", "chibios_hwdef.py")

# 生成器输出的候选位置
GEN_CANDIDATES = [
    os.path.join(os.environ.get("TEMP", "/tmp"), "bl2"),
    os.path.join(os.environ.get("TEMP", "/tmp"), "bltest"),
    "/tmp/bl2",
    "/tmp/bltest",
]


def read_lines(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().split("\n")
    except OSError:
        return None


def read_text(path):
    lines = read_lines(path)
    return None if lines is None else "\n".join(lines)


def count_lines(path):
    t = read_text(path)
    return 0 if t is None else t.count("\n")


def show(path, lo, hi, label=""):
    lines = read_lines(path)
    if not lines:
        print("    [读不到 %s]" % os.path.basename(path))
        return False
    print("    ┌─ %s:%d-%d%s"
          % (os.path.basename(path), lo, hi, ("  " + label) if label else ""))
    for i in range(lo, min(hi, len(lines)) + 1):
        print("    │ %4d  %s" % (i, lines[i - 1]))
    print("    └─")
    return True


def find_line(path, pattern, start=1):
    lines = read_lines(path)
    if not lines:
        return None
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


def find_gen_dir():
    for d in GEN_CANDIDATES:
        if d and os.path.isfile(os.path.join(d, "hwdef.h")):
            return d
    return None


# ===========================================================================
# 实验 1：三层配置
# ===========================================================================

def exp1():
    head("实验 1：三层配置的依赖关系")

    print("""
  ChibiOS 的配置分三层：chconf（内核）/ halconf（HAL）/ mcuconf（MCU 外设）。
  这个实验从 #include 行读出它们的依赖方向。

  ★ 关键问题：第三层（mcuconf）是手写的还是生成的？
""")

    print()
    print("  ── ① 各文件的开头 ──")
    print()

    for path, name in ((F_CHCONF, "chconf.h（内核层）"),
                       (F_HALCONF, "halconf.h（HAL 层）"),
                       (F_MCUCONF, "mcuconf.h（MCU 层）")):
        lines = read_lines(path)
        if not lines:
            continue
        print("  ┌─ %s  (%d 行)" % (name, len(lines) - 1))
        for i, line in enumerate(lines[:50], 1):
            s = line.strip()
            if s.startswith("#include") or s.startswith("#define _CHIBIOS") \
               or "Override values" in s or "generated config" in s:
                print("  │  %-4d %s" % (i, line.rstrip()))
        print("  └─")
        print()

    print("  ── ② 依赖图 ──")
    print()
    print("""        hwdef.h  （★ 生成）
           ↑  ↑
           │  └──────────────────┐
           │                     │
        chconf.h            mcuconf.h
        （内核）             （MCU 外设）
                                ↑
                                │
                           halconf.h
                           （HAL 子系统）

    ⇒ 【都指向 hwdef.h】⇒ hwdef.dat 能覆盖三层里的任意一项。
""")

    print("  ── ③ mcuconf.h 是「生成」的证据 ──")
    print()
    p = find_line(F_MCUCONF, r"include .hwdef\.h.")
    if p:
        show(F_MCUCONF, p - 6, p + 1, "★ 注释自己说了")

    print("""
    「this provides the default mcuconf.h for each board.
      Override values in hwdef.dat」

    ⇒ 这一句就是【共享模板 + 每板数据】模式的自我说明。
""")

    print("  ── ④ 按 MCU 家族选变体 ──")
    print()
    p = find_line(F_MCUCONF, r"defined\(STM32F1\)")
    if p:
        show(F_MCUCONF, p - 2, p + 18, "★ mcuconf.h 里的家族分派")

    variants = sorted(f for f in os.listdir(COMMON)
                      if f.endswith("_mcuconf.h"))
    print()
    print("    实测 %s/ 下的 mcuconf 家族变体（%d 个）：" % (
        os.path.basename(COMMON), len(variants)))
    for v in variants:
        print("        %-28s %5d 行" % (v, count_lines(os.path.join(COMMON, v))))

    print("""
    ⇒ 共享的 mcuconf.h 只有 76 行 —— 它只做【分派】，
      真正的默认值在各家族文件里（每个几百行）。
""")
    print()


# ===========================================================================
# 实验 2：驱动模型的四层
# ===========================================================================

def exp2():
    head("实验 2：驱动模型的四层")

    print("""
  一个 ChibiOS 驱动分四层。这个实验量化每层的行数，
  并统计【高层实现依赖几个 LLD 函数】。

  ⇒ 回答一个问题：可移植性的成本是多少？
""")

    # (名字, API 头, 高层实现, LLD 接口, LLD 实现)
    DRIVERS = [
        ("I2C",
         "os/hal/include/hal_i2c.h",
         "os/hal/src/hal_i2c.c",
         "os/hal/ports/STM32/LLD/I2Cv2/hal_i2c_lld.h",
         "os/hal/ports/STM32/LLD/I2Cv2/hal_i2c_lld.c"),
        ("SPI",
         "os/hal/include/hal_spi.h",
         "os/hal/src/hal_spi.c",
         "os/hal/ports/STM32/LLD/SPIv3/hal_spi_lld.h",
         "os/hal/ports/STM32/LLD/SPIv3/hal_spi_lld.c"),
        ("ADC",
         "os/hal/include/hal_adc.h",
         "os/hal/src/hal_adc.c",
         "os/hal/ports/STM32/LLD/ADCv4/hal_adc_lld.h",
         "os/hal/ports/STM32/LLD/ADCv4/hal_adc_lld.c"),
    ]

    print()
    print("  %-6s %-12s %-12s %-12s %-12s %s"
          % ("驱动", "API 头", "高层实现", "LLD 接口", "LLD 实现", "总计"))
    print("  " + "-" * 70)

    for name, api, high, lldh, lldc in DRIVERS:
        vals = []
        for rel in (api, high, lldh, lldc):
            vals.append(count_lines(os.path.join(CHIBIOS, rel)))
        print("  %-6s %-12d %-12d %-12d %-12d %d"
              % (name, vals[0], vals[1], vals[2], vals[3], sum(vals)))

    print()
    print("  ── ★ 高层实现依赖几个 LLD 函数？──")
    print()

    for name, api, high, lldh, lldc in DRIVERS:
        hp = os.path.join(CHIBIOS, high)
        t = read_text(hp)
        if t is None:
            continue
        # 找高层文件里调用的 *_lld_* 函数名
        calls = sorted(set(re.findall(r"\b([a-z][a-z0-9_]*_lld_[a-z0-9_]+)\s*\(", t)))
        print("  %s 的高层实现（%s）:" % (name, os.path.basename(high)))
        print("      调用了 %d 个不同的 LLD 函数：" % len(calls))
        for c in calls:
            print("          %s" % c)
        print()

    print("""  ⇒ 结论：**接口很窄**。

      几百行的高层实现，只依赖个位数的 LLD 函数。
      ⇒ 换芯片时，只需要重写这几个函数的实现（LLD 层），
        高层代码一行都不用改。
""")

    # ── ★★ 但 SPI 不一样：高层也分岔 ──────────────────────
    print("  ── ★★ 但「四层」是个简化模型 —— SPI 的【高层也分岔】──")
    print()

    spi_c = os.path.join(CHIBIOS, "os", "hal", "src", "hal_spi.c")
    print("    os/hal/src/hal_spi.c 只有 %d 行。" % count_lines(spi_c))
    print("    它不是实现，是个【分派器】：")
    print()
    show(spi_c, 25, 36, "★ 整个文件的核心")

    inc_dir = os.path.join(CHIBIOS, "os", "hal", "src")
    print()
    print("    两套高层实现（实测）：")
    print()
    print("    %-22s %10s  %s" % ("文件", "字节", "调用的 LLD 函数"))
    print("    " + "-" * 68)
    for inc in ("hal_spi_v1.inc", "hal_spi_v2.inc"):
        fp = os.path.join(inc_dir, inc)
        if not os.path.isfile(fp):
            continue
        t = read_text(fp) or ""
        calls = sorted(set(re.findall(r"\b(spi_lld_[a-z0-9_]+)\s*\(", t)))
        print("    %-22s %10d  %d 个" % (inc, os.path.getsize(fp), len(calls)))
        for c in calls:
            print("        %s" % c)
        print()

    print("""    ⇒ V2 的 LLD 多了 exchange / send / receive / ignore / stop_transfer
      —— 因为它有真正的 DMA 传输支持。

    ⇒ 两套高层调用【不同的 LLD 接口】。

    ★★ 所以准确的模型是：

        ┌─ 统一的部分 ──────────────────┐
        │  API 头（hal_spi.h）           │  ← 对用户统一
        └────────────────────────────────┘
        ┌─ 分岔的部分 ──────────────────┐
        │   高层 v1  ←──→  LLD v1        │
        │   高层 v2  ←──→  LLD v2        │
        │   ↑ 按 HAL_LLD_SELECT_SPI_V2 选 │
        └────────────────────────────────┘

    ★ 这修正了"可移植性 = 高层统一 + 底层替换"的直觉：

      当硬件的【能力】不同时（不只是寄存器地址不同），
      高层也必须分岔 —— 否则要么 v1 实现不了，要么 v2 的能力用不上。

    ⇒ 这才是 os/hal 48 万行的真正原因：
      不只是「N 份 LLD」，而是【某些驱动有 N 份 LLD × M 份高层】。

    ⚠ 而这个分岔是有代价的：两套高层实现要【分别维护】
      （修一个 bug 要改两个文件），而它们对用户是同一个 API
      ⇒ bug 可能只在其中一个里存在。

    ⇒ 这是「可移植性」的隐藏成本，通常不被计入。
""")
    print()

    print("  ── ★ LLD 按【外设版本】分目录 ──")
    print()
    lld_dir = os.path.join(CHIBIOS, "os", "hal", "ports", "STM32", "LLD")
    if os.path.isdir(lld_dir):
        entries = sorted(os.listdir(lld_dir))
        print("    实测 %s" % "os/hal/ports/STM32/LLD/")
        print("    共 %d 个目录：" % len(entries))
        print()
        # 按前缀分组
        groups = {}
        for e in entries:
            m = re.match(r"^([A-Z][A-Za-z0-9]*?)(v\d+.*)?$", e)
            if m:
                groups.setdefault(m.group(1), []).append(e)
        for base in sorted(groups):
            vs = sorted(groups[base])
            if len(vs) > 1:
                print("        %-12s ★ %d 个版本：%s"
                      % (base, len(vs), "、".join(vs)))
            else:
                print("        %-12s %s" % (base, vs[0]))
    print()
    print("""    ⇒ 同名外设有多个版本，因为 ST 的同一外设在不同芯片上寄存器不同：

        STM32F1/F4 的 I2C = I2Cv1
        STM32F0/F3/F7/L4  = I2Cv2
        STM32H7/L4+       = I2Cv3

    ⇒ 一个目录 = 一类寄存器的实现。
      这是"硬件差异的显式编码"。

    ★ 对比 NuttX：它按【芯片系列】分目录
      （arch/arm/src/stm32h7/），同一系列的芯片共享目录。

      两种切法：
        ChibiOS  复用率高（一份 I2Cv2 服务 F0/F3/F7/L4）
                 但要维护"芯片→外设版本"的映射表
        NuttX    更直白，但跨系列的相同外设要各写一遍
""")
    print()


# ===========================================================================
# 实验 3：板级层
# ===========================================================================

def exp3():
    head("实验 3：★ 板级层 —— 一个 board.c 服务 300 块板子")

    print("""
  ★★ 本实验的核心方法：**能跑生成器就跑，别看它的代码。**

  问题时：共享的 board.c（353 行）怎么知道每块板子的引脚？
""")

    print()
    print("  ── ① 共享的 board.c 引用了什么 ──")
    print()
    p = find_line(F_BOARD_C, r"static const gpio_config_t gpio_default_config")
    if p:
        show(F_BOARD_C, p, p + 14, "★ VAL_GPIO*_MODER 从哪来")

    print("""
    ⇒ VAL_GPIOA_MODER 这些宏【不是 board.c 定义的】。
      它们来自 hwdef.h —— 【生成的文件】。
""")

    print("  ── ② 生成器里生成它们的地方 ──")
    print()
    p = find_line(F_HWDEF_PY, r'VAL_GPIO%s_%-7s\s+0x0')
    if p:
        show(F_HWDEF_PY, p - 3, p + 8, "★ 生成 VAL_GPIO*_MODER")

    print("  ── ③ 生成器产出哪些文件 ──")
    print()
    p = find_line(F_HWDEF_PY, r'write_hwdef_header')
    if p:
        show(F_HWDEF_PY, p - 5, p + 20, "★ 生成主函数的尾部")

    gen = find_gen_dir()
    print()
    if gen:
        print("    ★ 本机找到一份现成的产物：%s" % gen)
        print()
        print("    %-20s %8s %12s" % ("文件", "行数", "字节"))
        print("    " + "-" * 44)
        for fn in sorted(os.listdir(gen)):
            fp = os.path.join(gen, fn)
            if os.path.isfile(fp):
                print("    %-20s %8d %12d"
                      % (fn, count_lines(fp), os.path.getsize(fp)))
        print()
        show_gen_dir(gen)
    else:
        print("    ⚠ 本机没有现成的产物。要生成，跑：")
        print()
        print("        cd %s" % APM_ROOT)
        print("        python libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py \\")
        print("            --bootloader --outdir /tmp/bl2 \\")
        print("            libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef-bl.dat")
        print()
        print("    生成后再跑本实验。")
    print()

    # ── 不管有没有产物，都能读的部分 ──
    print("  ── ④ 上游 vs ArduPilot 的组织方式 ──")
    print()

    boards = [d for d in os.listdir(HWDEF)
              if os.path.isdir(os.path.join(HWDEF, d))]
    bl_variants = [b for b in boards
                   if os.path.isfile(os.path.join(HWDEF, b, "hwdef-bl.dat"))]

    print("    hwdef/ 下有 %d 块板子，其中 %d 块有 bootloader 变体。"
          % (len(boards), len(bl_variants)))
    print()
    print("    %-40s %s" % ("共享文件（一份）", "行数"))
    print("    " + "-" * 56)
    for f in ("board.h", "board.c", "chconf.h", "halconf.h", "mcuconf.h"):
        fp = os.path.join(COMMON, f)
        print("    %-40s %d" % ("hwdef/common/" + f, count_lines(fp)))
    print()
    print("""    ⇒ 300 块板子共享这一套，差异全在各自的 hwdef.dat 里。

    ┌──────────────────────────────────────────────────────────┐
    │  上游 ChibiOS：每板一份手写的 board.c/h + 三个 conf      │
    │      300 块 ⇒ 300 × 5 = 1500 个文件（大部分内容重复）     │
    ├──────────────────────────────────────────────────────────┤
    │  ArduPilot：一套共享 + 每板一份 hwdef.dat                │
    │      300 块 ⇒ 5 个共享文件 + 300 个 hwdef.dat            │
    └──────────────────────────────────────────────────────────┘
""")
    print()


def show_gen_dir(gen):
    """打印已生成的关键内容"""
    hwdef_h = os.path.join(gen, "hwdef.h")
    lds = os.path.join(gen, "ldscript.ld")
    hwdat = os.path.join(gen, "hw.dat")

    if os.path.isfile(lds):
        print("    ── 生成的 ldscript.ld（全文）──")
        print()
        for i, line in enumerate(read_lines(lds) or [], 1):
            print("        %2d  %s" % (i, line))
        print()

    if os.path.isfile(hwdat):
        print("    ── 生成的 hw.dat（展平后的输入，调试用）──")
        print()
        lines = read_lines(hwdat) or []
        for i, line in enumerate(lines[:24], 1):
            print("        %2d  %s" % (i, line))
        if len(lines) > 24:
            print("        ... （共 %d 行）" % len(lines))
        print()

    if os.path.isfile(hwdef_h):
        print("    ── 生成的 hwdef.h 里的关键项 ──")
        print()
        KEYS = [
            ("CH_CFG_USE_MUTEXES", "内核：互斥量"),
            ("CH_CFG_USE_DYNAMIC", "内核：动态线程"),
            ("CH_CFG_USE_HEAP", "内核：堆"),
            ("CH_CFG_USE_SEMAPHORES", "内核：信号量"),
            ("CH_CFG_USE_EVENTS", "内核：事件"),
            ("CH_CFG_USE_REGISTRY", "内核：注册表"),
            ("CH_CFG_OPTIMIZE_SPEED", "内核：内联优化"),
            ("HAL_USE_SPI", "HAL：SPI"),
            ("HAL_USE_I2C", "HAL：I2C"),
            ("HAL_USE_PWM", "HAL：PWM"),
            ("HAL_USE_ADC", "HAL：ADC"),
            ("HAL_USE_SERIAL_USB", "HAL：USB 串口"),
            ("HAL_NO_TIMER_THREAD", "ArduPilot：不要定时器线程"),
            ("HAL_NO_RCOUT_THREAD", "ArduPilot：不要 RCOUT 线程"),
            ("HAL_NO_RCIN_THREAD", "ArduPilot：不要 RCIN 线程"),
        ]
        lines = read_lines(hwdef_h) or []
        for key, desc in KEYS:
            found = None
            for i, line in enumerate(lines, 1):
                if re.match(r"^\s*#\s*define\s+%s\b" % key, line):
                    found = (i, line.strip())
                    break
            if found:
                print("        %-5d %-26s %s"
                      % (found[0], desc, found[1][:40]))
            else:
                print("        %-5s %-26s （未定义）" % ("—", desc))

        print()
        print("""    ⇒ 这个文件把【三层配置 + 板级引脚】统一在了一起：
          · ChibiOS 内核配置
          · ChibiOS HAL 子系统开关
          · ArduPilot 自己的 HAL_NO_* 开关
          · VAL_GPIO*_MODER（引脚模式）
          · HAL_GPIO_PINS / HAL_SPI_DEVICE_LIST 等表

      ⇒ 一个生成器、一个头文件，三个层次的配置都在这。
""")


# ===========================================================================
# 实验 4：用了什么、绕过了什么
# ===========================================================================

def exp4():
    head("实验 4：ArduPilot 用了什么、绕过了什么")

    print("""
  《NuttX 教程》第 10 课列了 10 项"绕过"。这个实验逐条核实，
  并给出一个量化指标：【侵入 LLD 层几处】。
""")

    print()
    print("  ── ① 开了哪些 HAL 子系统 ──")
    print()

    lines = read_lines(F_HALCONF) or []
    on, off = [], []
    for i, line in enumerate(lines, 1):
        m = re.match(r"^\s*#\s*define\s+(HAL_USE_\w+)\s+(TRUE|FALSE)", line)
        if m:
            (on if m.group(2) == "TRUE" else off).append((m.group(1), i))

    print("    总共 %d 个 HAL_USE_* 开关" % (len(on) + len(off)))
    print()
    print("    ┌─ 开着（%d 个）──" % len(on))
    for name, i in on:
        print("    │  halconf.h:%-4d %s" % (i, name))
    print("    └─")
    print()
    print("    ┌─ 关着（%d 个）──" % len(off))
    for j, (name, i) in enumerate(off):
        if j % 2 == 0:
            print("    │  halconf.h:%-4d %-32s%s"
                  % (i, name,
                     ("  halconf.h:%-4d %s" % (off[j + 1][1], off[j + 1][0]))
                     if j + 1 < len(off) else ""))
    print("    └─")
    print()
    print("""    ⚠ 注意 HAL_USE_UART FALSE 但 HAL_USE_SERIAL TRUE。

      因为 SERIAL 是更高层的抽象（含 UART + USART + 软件串口），
      而 UART 是它的一个子驱动。这是 ChibiOS 的层次设计。
""")

    print("  ── ② 自研替代品的规模 ──")
    print()

    SUBS = [
        ("CAN", "bxcan.hpp", "自研 CAN 寄存器操作（halconf:68 CAN FALSE）"),
        ("CAN", "CanIface.cpp", "bxCAN 驱动层"),
        ("CAN", "CANFDIface.cpp", "CAN-FD 驱动层"),
        ("GPIO", "GPIO.cpp", "自研 GPIO 管理层（含中断分发）"),
        ("", "system.cpp", "自研 fault handler"),
        ("", "malloc.c", "接管 libc 的 malloc/free"),
    ]
    print("    %-34s %9s  %s" % ("文件", "字节", "说明"))
    print("    " + "-" * 70)
    for _tag, fn, desc in SUBS:
        fp = os.path.join(HALCHIB, fn)
        if os.path.isfile(fp):
            print("    %-34s %9d  %s" % (fn, os.path.getsize(fp), desc))
        else:
            print("    %-34s %9s  %s" % (fn, "—", desc))

    print()
    print("  ── ③ ★ 量化：侵入 LLD 层几处 ──")
    print()
    print("    搜 `*_lld_*()` 形式的调用（LLD 层是公开 API 下面那一层）：")
    print()

    lld_calls = {}
    for dirpath, _d, filenames in os.walk(HALCHIB):
        for fn in filenames:
            if not fn.lower().endswith((".cpp", ".h", ".c")):
                continue
            fp = os.path.join(dirpath, fn)
            for i, line in enumerate(read_lines(fp) or [], 1):
                if line.strip().startswith("//") or line.strip().startswith("*"):
                    continue
                for m in re.finditer(r"\b([a-z][a-z0-9_]*_lld_[a-z0-9_]+)\s*\(", line):
                    lld_calls.setdefault(m.group(1), []).append(
                        (os.path.basename(fp), i))

    print("    找到 %d 个不同的 LLD 函数被调用：" % len(lld_calls))
    print()
    for name in sorted(lld_calls):
        locs = lld_calls[name]
        print("        %-28s %d 处" % (name, len(locs)))
        for fn, i in locs[:3]:
            print("            %s:%d" % (fn, i))
    print()

    total = sum(len(v) for v in lld_calls.values())
    print("    ⇒ 总共 %d 处。" % total)
    print()
    if total <= 4:
        print("""    ★★ 这是个【很低】的数字 —— 全代码库只有这么几处。

      说明 ArduPilot 对 HAL 的调用基本都走公开 API，
      只有【真正需要性能或公开 API 不够】的地方才下沉一层。

      两处分别是：
        GPIO.cpp      查"这个引脚注册过回调吗"
        system.cpp    ★ 读微秒计数器（时间快路径）
""")

    print("  ── ④ 侵入深度的金字塔 ──")
    print()
    print("""    把本节和前面几课的证据合起来：

        ┌─ 用公开 API ──────────────── 大量（PAL / SPI / I2C / ADC / SDC）
        │
        ├─ 自己写一层 ──────────────── 5 项（WorkQueue / CAN / 看门狗 / 异常 / 内存）
        │
        ├─ 直接调 LLD ──────────────── ★ 上面数出来的几处
        │
        └─ 直接改内核 ──────────────── ★ 1 处（chMtxForceReleaseS，第 12 课）

    ⇒ 健康的分布是"金字塔"：越深的侵入越少。

    ⚠ 反过来：如果"改内核"那一层有很多处，
      这个项目升级依赖库的成本会非常高 ——
      因为每次上游改动都可能冲突。

      ArduPilot 的 1 处意味着【它和 ChibiOS 上游的冲突面很窄】。
      这也是它能一直跟着 ChibiOS 升级的原因。
""")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("三层配置的依赖关系", exp1),
    "2": ("驱动模型的四层", exp2),
    "3": ("★ 板级层（跑生成器）", exp3),
    "4": ("用了什么、绕过了什么", exp4),
}


def main():
    print()
    hr("#")
    print("#  第 13 课：ChibiOS 的 HAL 与板级支持")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(CHIBIOS):
        print()
        print("  [错误] 找不到 ChibiOS 源码：")
        print("         %s" % CHIBIOS)
        print()
        print("  请先按《总纲》的方法拉取 6a85082c 版本。")
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
    print("#  实验结束")
    hr("#")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
