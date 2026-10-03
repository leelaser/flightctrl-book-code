#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 18 课配套实验：ADC / PWM / GPT / CAN

    实验 1  ★★ 一个 TIM 外设，四个逻辑驱动
    实验 2  ★★ ADC 的组配置：框架字段 / 裸寄存器 / 芯片分支
    实验 3  ★ PWM：占空比就是 CCR
    实验 4  ★★ DMA + cache：没 invalidate 会读到什么
    实验 5  ★★ AP 和 HAL 的五种关系（实测表）

用法：
    python lesson18_adc_pwm_gpt_can.py 1
    ...
    python lesson18_adc_pwm_gpt_can.py        # 全部

【诚实声明】

实验 1/2/5 是【读源码】—— 结论来自文件内容。
实验 3 是【模拟】—— 模拟 TIM 的 ARR/CCR/预分频关系，
丢掉了：真实的时钟树（APB 定时器时钟可以是 2 倍频）、死区时间、
互补输出、中心对齐模式。
实验 4 是【模拟】—— 模拟 D-Cache 与 DMA 的交互，
丢掉了：真实的 cache line 大小（H7 = 32 字节）、
写回策略、多核一致性。它证明的是【语义】，不是【时序】。

源码版本：ArduPilot 4.5.7 pin 的 ChibiOS 6a85082c（内核 7.0.4）
"""

import io
import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APM_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "APM",
                 "ardupilot-Copter-4.5.7", "ardupilot-Copter-4.5.7")
)
CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")
HAL = os.path.join(CHIBIOS, "os", "hal")
LLD = os.path.join(HAL, "ports", "STM32", "LLD")

F_TIM_PWM_C = os.path.join(LLD, "TIMv1", "hal_pwm_lld.c")
F_TIM_GPT_C = os.path.join(LLD, "TIMv1", "hal_gpt_lld.c")
F_TIM_ICU_C = os.path.join(LLD, "TIMv1", "hal_icu_lld.c")
F_TIM_EICU_C = os.path.join(LLD, "TIMv1", "hal_eicu_lld.c")
F_HAL_ADC_H = os.path.join(HAL, "include", "hal_adc.h")
F_ADC_LLD_H = os.path.join(LLD, "ADCv4", "hal_adc_lld.h")
F_PLATFORM_H7 = os.path.join(HAL, "ports", "STM32", "STM32H7xx", "platform.mk")

F_ANALOGIN = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "AnalogIn.cpp")
F_ANALOGIN_H = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                            "AnalogIn.h")
F_RCOUTPUT = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "RCOutput.cpp")
F_SOFTSIG = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                         "SoftSigReaderInt.cpp")
F_HALCONF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                         "hwdef", "common", "halconf.h")
F_HWDEF_PY = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "hwdef", "scripts", "chibios_hwdef.py")
F_STM32UTIL = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                           "hwdef", "common", "stm32_util.c")

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
    if "/os/hal/" in p.replace("\\", "/"):
        return os.path.basename(p)
    if p.startswith(APM_ROOT):
        rest = p[len(APM_ROOT) + 1:].replace(os.sep, "/")
        return rest if rest.startswith("libraries/") else os.path.basename(p)
    return p.replace(os.sep, "/")


def src(path, a, b, note=None):
    lines = read(path).split("\n")
    if note:
        print("      %s" % note)
    for i in range(max(0, a - 1), min(len(lines), b)):
        print("      %s:%d  %s" % (rel(path), i + 1, lines[i].rstrip()[:104]))
    print()


# ===========================================================================
# 实验 1：一个 TIM 外设，四个逻辑驱动
# ===========================================================================

def exp1():
    head("实验 1  ★★ 一个 TIM 外设，四个逻辑驱动")

    print("""
  第 17 课看到的 LLD 是"一个外设 = 一个目录"（USARTv3 / SPIv3 / I2Cv3）。
  **但 TIMv1 不是** —— 8,623 行里装了四个驱动。
""")

    rows = []
    for name, f in (("PWM", F_TIM_PWM_C), ("GPT", F_TIM_GPT_C),
                    ("ICU", F_TIM_ICU_C), ("EICU", F_TIM_EICU_C)):
        d = os.path.dirname(f)
        base = os.path.basename(f).replace("_lld.c", "")
        h = os.path.join(d, os.path.basename(f).replace(".c", ".h"))
        rows.append((name, read(f).count("\n"), read(h).count("\n")))
    print("      %-8s %-10s %-10s %s" % ("驱动", ".c 行数", ".h 行数", "干什么"))
    print("      " + "-" * 66)
    what = {"PWM": "输出方波（舵机 / 电调 / DShot）",
            "GPT": "通用定时器（周期性 / 单次中断）",
            "ICU": "输入捕获（量脉冲宽度）",
            "EICU": "增强输入捕获（一次抓两个边沿）"}
    for name, a, b in rows:
        print("      %-8s %-10d %-10d %s" % (name, a, b, what[name]))
    print("      " + "-" * 66)
    print("      %-8s %-10d %-10d  （另有 stm32_tim.h 等公共部分）"
          % ("合计", sum(r[1] for r in rows), sum(r[2] for r in rows)))

    print("""
  ⇒ ★★ **四个驱动共用同一份 TIM 寄存器定义**，但各有自己的
     框架层文件（`hal_pwm.c` / `hal_gpt.c` / `hal_icu.c` / `hal_eicu.c`）。

     这是"四层模型"的另一面：**LLD 的划分粒度是【外设】，不是【用途】**。
     一个 TIM 外设可以当 PWM 用、当定时器用、当输入捕获用 ——
     ChibiOS 为每种用法写一个驱动，但它们都落在 `TIMv1/` 里。
""")

    n = find_line(F_PLATFORM_H7, r"TIMv1/driver")
    if n:
        src(F_PLATFORM_H7, n - 2, n + 1, "H7 的 platform.mk 里只 include 一次：")
    print("""  ⇒ ★ 注意：**只 include 一个 TIMv1**。四个驱动是
     `TIMv1/driver.mk` 里的开关决定的（`HAL_USE_PWM` / `HAL_USE_GPT` …），
     不是靠 include 几个目录。

  ⇒ 所以 §5 那张表里"AP 用不用 GPT"这个问题，
    答案在 **`halconf.h` 的一个开关**上，而不是"有没有这个驱动"。
""")

    hr("-")
    print("  【实测】四个驱动的对外接口（框架调它们的那几个函数）：")
    hr("-")
    print()
    for name, f in (("PWM", F_TIM_PWM_C), ("GPT", F_TIM_GPT_C),
                    ("ICU", F_TIM_ICU_C), ("EICU", F_TIM_EICU_C)):
        lines = read(f).split("\n")
        fns = []
        for l in lines:
            m = re.match(r"^(?:void|msg_t|uint32_t|bool)\s+(\w+_lld_\w+)\(", l)
            if m:
                fns.append(m.group(1))
        print("      %-6s %s" % (name, "、".join(fns[:5]) if fns else "—"))
    print("""
      ⇒ 每个驱动都提供同一套动词的 LLD 实现：
        `start` / `stop` / `change_*` / `enable` / `disable`
        —— **框架层用同名函数调它们**（第 17 课 §1.3 那张虚函数表）。
""")


# ===========================================================================
# 实验 2：ADC 的组配置
# ===========================================================================

def exp2():
    head("实验 2  ★★ ADC 的组配置：框架字段 / 裸寄存器 / 芯片分支")

    print("""
  ADC 是这一课最特别的一个 —— 它的**配置结构几乎全是裸寄存器**。

  先看框架层要求了什么（`hal_adc.h`）：
""")
    n = find_line(F_HAL_ADC_H, r"struct hal_adc_configuration_group \{")
    if n:
        src(F_HAL_ADC_H, n, n + 7, "★ 又是那个模式：框架字段 + 一个 LLD 宏：")

    n2 = find_line(F_ADC_LLD_H, r"#define adc_lld_configuration_group_fields")
    if n2:
        src(F_ADC_LLD_H, n2, n2 + 22, "LLD 补的是什么（ADCv4）：")

    print("""  ★★ **十几个字段，全是 `uint32_t` 的寄存器初值**
     （`cfgr` / `cfgr2` / `ccr` / `pcsel` / `ltr1` / `htr1` …）。

  ⇒ 对比第 17 课的串口：那边的 `SerialConfig` 至少有
    `speed` 这种**语义化**字段；**ADC 这边几乎没有**。

  **为什么？** 因为 ADC 的配置空间太大：单次/连续/扫描/不连续/
  过采样/模拟看门狗/多 ADC 同步…… ChibiOS 的选择是
  **"sequencing 和 DMA 我管，寄存器你自己写"**。
""")

    hr("-")
    print("""
  【实测】ArduPilot 在 `AnalogIn.cpp` 里怎么填这个结构 ——
        它**同时用了两层**，还带芯片分支。
""")
    hr("-")
    n3 = find_line(F_ANALOGIN, r"adcgrpcfg\[index\]\.circular = true")
    if n3:
        src(F_ANALOGIN, n3 - 2, n3 + 18, "AnalogIn.cpp ——")

    print("""  ⇒ 拆成三层看：

      | 填的字段 | 属于哪一层 | 换 MCU 要改吗 |
      |---|---|---|
      | `circular` / `num_channels` / `end_cb` | ★ **框架层** | 不用 |
      | `cfgr = ADC_CFGR_CONT \\| ADC_CFGR_RES_16BITS` | ★ **LLD 的裸寄存器** | **要**（见下面的 `#elif`） |
      | `sqr1` / `cr2` | 老芯片的寄存器名 | 要 |

  ★★ 而那个 `#if defined(ADC_CFGR_RES_16BITS) … #elif … #else`
    是**应用层为不同 MCU 写的分支** —— 因为 H7 的 ADCv4 和老芯片的
    ADCv1/2 用的**寄存器名字都不一样**。

  ⇒ **这就是"分层"的代价转移**：LLD 把差异隔离在驱动里，
    但配置结构是应用填的 —— **配置的差异隔离不掉**，
    只能让应用自己写 `#if`。
""")

    hr("-")
    print("  【实测】ADC 的缓冲与回调：")
    hr("-")
    n4 = find_line(F_ANALOGIN_H, r"define ADC_DMA_BUF_DEPTH")
    if n4:
        src(F_ANALOGIN_H, n4, n4, "缓冲区深度（AnalogIn.h）：")
    n5 = find_line(F_ANALOGIN, r"malloc_type.*MEM_DMA_SAFE")
    if n5:
        src(F_ANALOGIN, n5, n5 + 3, "缓冲区从哪来（★ 接上第 16/17 课）：")
    n6 = find_line(F_ANALOGIN, r"sample_sum\[index\]\[j\] \+= \*buffer")
    if n6:
        src(F_ANALOGIN, n6 - 2, n6 + 2, "回调里怎么用（8 层求和 ⇒ 平均）：")


# ===========================================================================
# 实验 3：PWM 的占空比就是 CCR
# ===========================================================================

def exp3():
    head("实验 3  ★ PWM：占空比就是 CCR")

    print("""
  PWM 的 LLD 里，改占空比是**一行赋值**：
""")
    n = find_line(F_TIM_PWM_C, r"pwmp->tim->CCR\[channel\] = width;")
    if n:
        src(F_TIM_PWM_C, n - 6, n + 6, "hal_pwm_lld.c ——")

    print("""  ⇒ **占空比就是比较寄存器的值**，没有任何"百分比换算"。

  所以 PWM 的 API 里那两个参数是这样的（`hal_pwm.h`）：

      · `pwmcnt_t period`  → 写进 ARR（自动重装），决定**周期**
      · `pwmcnt_t width`   → 写进 CCR，决定**高电平多久**

  ★ **分辨率 = ARR**。STM32 的 TIM 是 16 位的（ARR ≤ 65535），
    所以"频率 × 分辨率"有个上限 —— 这就是为什么**高频率的
    DShot 要用别的办法**（第 15 课 §5.4 那个串口位操作就是干这个的）。
""")

    hr("-")
    print("""
  【模拟】给定定时器时钟和 ARR，算 PWM 的频率与分辨率，
  并看几组真实配置（舵机 / 电调 / 高分辨率）落在哪。
""")
    hr("-")
    print()

    def calc(clk_hz, arr, ccr):
        period_cnt = arr + 1                 # ARR 是 0 基
        freq = clk_hz / period_cnt
        duty = ccr / period_cnt
        return freq, duty

    print("      %-14s %-12s %-8s %-12s %-9s %s"
          % ("定时器时钟", "ARR", "CCR", "PWM 频率", "占空比", "用途"))
    print("      " + "-" * 78)
    cases = [
        (1_000_000,   19999,  1500, "50 Hz 舵机（1~2 ms 脉宽）"),
        (1_000_000,   19999,  2000, "  同上，满油门"),
        (24_000_000,   2499,  1250, "电调 9.6 kHz"),
        (24_000_000,    999,   500, "电调 24 kHz"),
        (240_000_000, 65535, 32767, "★ 顶到 16 位分辨率"),
    ]
    for clk, arr, ccr, use in cases:
        f, d = calc(clk, arr, ccr)
        print("      %-14s %-12s %-8s %-12s %-9s %s"
              % ("%d Hz" % clk, arr, ccr, "%.1f Hz" % f, "%.1f%%" % (d * 100), use))

    print("""
      ★ 注意第一行：50 Hz 舵机用 **1 MHz** 的定时器 + ARR=19999
        ⇒ 数一个数正好 1 µs ⇒ **CCR 的值就是脉宽的微秒数**
        （1500 µs = 1.5 ms 中位）。这是一种刻意选的刻度：
        **让寄存器值直接等于物理量**，读写都不用换算。

      ★ 最后一行：240 MHz 的时钟下要做到 50 Hz，ARR 要 480 万 ——
        **超过 16 位了**，所以必须用预分频（PSC）。
        这就是"频率 × 分辨率 ≤ 时钟"这条约束的实际后果。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 预分频器 PSC（这里假设 PSC=1，实际可以 1~65536）
    · 时钟树（APB 定时器时钟可以是总线时钟的 2 倍）
    · 死区时间、互补输出、中心对齐模式
  模拟保住了什么：
    · **占空比 = CCR / (ARR+1)** 这个关系
    · "频率 × 分辨率 ≤ 输入时钟"这条硬约束
""")


# ===========================================================================
# 实验 4：DMA + cache
# ===========================================================================

def exp4():
    head("实验 4  ★★ DMA + cache：没 invalidate 会读到什么")

    print("""
  H7 的 ADC 用 **DMA 循环模式**把结果连续写进内存。而 H7 有 **D-Cache**。
  这两件事凑在一起会有一个经典陷阱。

  先看 ArduPilot 在回调里做的第一件事：
""")
    n = find_line(F_ANALOGIN, r"stm32_cacheBufferInvalidate\(buffer")
    if n:
        src(F_ANALOGIN, n - 5, n + 2, "AnalogIn.cpp 的 ADC 回调开头 ——")

    n2 = find_line(F_STM32UTIL, r"^void stm32_cacheBufferInvalidate")
    if n2:
        src(F_STM32UTIL, n2, n2 + 4, "它是谁（★ AP 自己包的一层）：")

    print("""  ★★ **为什么必须 invalidate？** 因为：

      · **CPU 读写内存要经过 D-Cache**（H7 上默认开启）
      · **DMA 读写内存不经过 Cache**（它直接访问 SRAM）

  ⇒ DMA 把新数据写进了 SRAM，而 **Cache 里还是旧副本**。
    CPU 去读的时候会命中 Cache，**读到的是旧值**。
""")

    hr("-")
    print("""
  【模拟】一块 4 字的内存 + 一个 4 字的 cache，跑两种流程：

       A. 不 invalidate（直接读）
       B. invalidate 之后再读（AP 的做法）

  场景：CPU 先"预热"读到一份数据，随后 DMA 更新了内存，
        CPU 再读一次。
""")
    hr("-")
    print()

    class MemCache:
        def __init__(self, n):
            self.mem = [0] * n
            self.cache = [None] * n       # None = 未缓存（invalid）
            self.hits = 0
            self.misses = 0
        def cpu_read(self, i):
            if self.cache[i] is None:
                self.misses += 1
                self.cache[i] = self.mem[i]
            else:
                self.hits += 1
            return self.cache[i]
        def dma_write(self, i, v):
            """DMA 直接写 SRAM —— 【不经过 cache】。"""
            self.mem[i] = v
        def invalidate(self):
            """丢弃 cache 里的副本（下次读会重新取）。"""
            self.cache = [None] * len(self.mem)

    def run(do_invalidate):
        mc = MemCache(4)
        for i in range(4):
            mc.mem[i] = 100 + i          # DMA 已经填好第一批
        first = [mc.cpu_read(i) for i in range(4)]   # CPU 读到（进 cache）
        for i in range(4):
            mc.dma_write(i, 200 + i)                    # DMA 更新
        if do_invalidate:
            mc.invalidate()
        second = [mc.cpu_read(i) for i in range(4)]
        return first, second, mc

    for do_inv in (False, True):
        first, second, mc = run(do_inv)
        label = "B. invalidate 之后读（AP 的做法）" if do_inv else "A. 不 invalidate"
        print("      %s" % label)
        print("         第一次读（DMA 填的旧数据）：%s" % first)
        print("         DMA 更新成：                %s" % [200 + i for i in range(4)])
        print("         第二次读：                  %s  %s"
              % (second, "" if second == [200 + i for i in range(4)]
                 else "★ 读到的是【旧值】"))
        print("         cache 命中 %d 次 / 未命中 %d 次" % (mc.hits, mc.misses))
        print()

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：不 invalidate 时，CPU **读到的永远是第一批数据** ——
     而 DMA 明明已经把新数据写进去了。**硬件不会报任何错。**

     ★ 这类问题的可怕之处：
       · 它是**间歇性**的（取决于 cache 有没有命中、什么时候被换出）
       · 它在 F4 上**根本不会出现**（F4 没有 D-Cache）
       ⇒ **"在 F4 上测好的驱动，搬到 H7 上就偶发数据错误"**

     ★ 所以 ArduPilot 在三个地方都加了 invalidate：
       `AnalogIn.cpp`（ADC）、`bouncebuffer.cpp`（SPI/SD 的弹跳缓冲）、
       `flash.c`（内部 flash 读）—— 全是 **DMA/外设碰过的内存**。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实 cache line（H7 = 32 字节）—— 实际是按行失效，不是按字
    · 写回（write-back）与写直达（write-through）的区别
    · 多核一致性、cache 的替换策略
  模拟保住了什么：
    · **"DMA 写不经过 cache" + "CPU 读会命中 cache"** 这两个事实
    · invalidate 的语义（丢弃副本，下次重新取）
""")


# ===========================================================================
# 实验 5：AP 和 HAL 的五种关系
# ===========================================================================

def exp5():
    head("实验 5  ★★ AP 和 HAL 的五种关系（实测表）")

    print("""
  第 17 课的问题"ArduPilot 用了 HAL 的哪半层"，
  在**每个外设上的答案都不一样**。这一节把它数出来。

  第一步：看 `halconf.h` 里那些开关（这是"要不要编这个驱动"）。
""")

    SWITCHES = [
        ("ADC", "HAL_USE_ADC", r"HAL_USE_ADC", "hal_adc.c"),
        ("PWM", "HAL_USE_PWM", r"HAL_USE_PWM", "hal_pwm.c"),
        ("GPT", "HAL_USE_GPT", r"HAL_USE_GPT", "hal_gpt.c"),
        ("ICU", "HAL_USE_ICU", r"HAL_USE_ICU", "hal_icu.c"),
        ("CAN", "HAL_USE_CAN", r"HAL_USE_CAN", "hal_can.c"),
    ]
    lines = read(F_HALCONF).split("\n")
    vals = {}
    for name, key, pat, _f in SWITCHES:
        v = "?"
        for i, l in enumerate(lines):
            if re.search(r"#define\s+%s\s+(TRUE|FALSE)" % pat, l):
                v = re.search(r"(TRUE|FALSE)", l).group(1)
                break
        vals[key] = v
    print("      %-6s %-22s %-8s %s" % ("外设", "halconf.h 开关", "值", "在哪行"))
    print("      " + "-" * 56)
    for name, key, pat, _f in SWITCHES:
        n, _ = None, None
        for i, l in enumerate(lines, 1):
            if re.search(r"#define\s+%s\s+(TRUE|FALSE)" % pat, l):
                n = i
                break
        print("      %-6s %-22s %-8s :%s" % (name, key, vals[key], n))

    print("""
      ★ **ICU 是 FALSE，但 AP 确实用了输入捕获** ——
        用的是 **EICU**（增强型），而它的开关
        **不是 `halconf.h` 里的常量，是生成器按板子写进去的**
        （`chibios_hwdef.py:2098`）：
""")
    n = find_line(F_HWDEF_PY, r"HAL_USE_EICU TRUE")
    if n:
        src(F_HWDEF_PY, n - 8, n + 3,
            "hwdef.dat 里写 RCININT ⇒ 生成 EICU 配置：")

    print("""  ⇒ ★★ **所以"AP 用不用 HAL 的某个驱动"这个问题，
    答案在三层里，要看三层**：

        ① `halconf.h` 的常量开关（编不编）
        ② `hwdef.dat` + 生成器（板子级的选择 —— EICU 在这层）
        ③ 代码里实际调了几个 API（用了多少）

  第二步：把③数出来。
""")

    hr("-")
    # 每个外设：AP 侧的调用点计数
    API = {
        "ADC": (r"\badc(Start|Convert|Stop)", "adc(Start|Convert|Stop)"),
        "PWM": (r"\bpwm(Start|Stop|EnableChannel|ChangePeriod)", "pwm*"),
        "GPT": (r"\bgpt(Start|Stop|PolledDelay)", "gpt*"),
        "ICU": (r"\bicu(Start|EnableChannel)", r"icu*"),
        "EICU": (r"\beicu(Start|EnableChannel)", r"eicu*"),
        "CAN": (r"\bcan_lld_|chThdCreateFromHeap.*can", "HAL CAN *"),
    }
    files = []
    for root, dirs, fns in os.walk(os.path.join(APM_ROOT, "libraries")):
        dirs[:] = [d for d in dirs if d not in ('.git', 'hwdef')]
        for f in fns:
            if f.endswith(('.cpp', '.h')):
                files.append(os.path.join(root, f))

    # ★ 判定不能只看数量 —— 同一个数字在不同外设上含义不同。
    #   ADC 只有 2 处，但那是因为**配置一次、DMA 循环跑到底**，
    #   运行期根本不需要再调 API。作者第一版按"数量 < 3 就是少量"判，
    #   把 ADC 判成了"⚠ 少量"，那是**把"调用少"当成了"用得少"**。
    NOTE = {
        "ADC":  "★ 用（**配置一次，循环 DMA 跑到底**，运行期不碰 API）",
        "PWM":  "★ 完整使用（每通道、每次改频率都要调）",
        "GPT":  "❌ 关掉（HAL_USE_GPT FALSE）",
        "ICU":  "⚠ 看板子（普通 RCIN 路径用）",
        "EICU": "★ 用（RCININT 路径，脉冲捕获）",
        "CAN":  "❌ **整个换成 UAVCAN 借来的驱动**",
    }
    print("      %-6s %-26s %s" % ("外设", "AP 代码里的调用点", "结论"))
    print("      " + "-" * 84)
    for name, (pat, _label) in API.items():
        rx = re.compile(pat)
        cnt = 0
        where = set()
        for p in files:
            s = read(p)
            hits = len(rx.findall(s))
            if hits:
                cnt += hits
                where.add(os.path.basename(p))
        print("      %-6s %-26s %s"
              % (name, "%d 处 / %d 个文件" % (cnt, len(where)), NOTE[name]))
        if where and len(where) <= 4:
            print("      %-6s   %s" % ("", "、".join(sorted(where))))

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论：五条线的五种关系**

      | 外设 | HAL 给什么 | AP 的关系 |
      |---|---|---|
      | **ADC** | 框架（sequencing+DMA）+ 裸寄存器配置 | ★ **两层都用** —— 框架字段 + `cfgr` 直接写 |
      | **PWM** | 完整驱动（`hal_pwm.c` 330 行） | ✅ **完整使用** |
      | **EICU** | 完整驱动（在 `TIMv1/` 里） | ✅ 用（RC 输入脉冲捕获），**开关由板子决定** |
      | **GPT** | 完整驱动（`hal_gpt.c` 284 行） | ❌ **关掉**（`HAL_USE_GPT FALSE`） |
      | **CAN** | 完整驱动（框架 + 两支 LLD，**2,983 行**） | ❌ **整个换成别人的**（§5.3） |

  ★★ **一句话：**"用不用 HAL"这个问题问错了。
    正确的问题是"**这四条链路里，哪一段的差异是我要自己管的**"——
    而答案在每个外设上都不同。

  ----------------------------------------------------------------------
  统计口径：扫 `libraries/` 下所有 .cpp/.h（排除 hwdef 生成器），
  按正则数调用点。**同一条 API 在头文件里的声明也会被数进去**，
  所以数字只用于"用/不用"的粗判，不是精确的调用次数。
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 一个 TIM，四个驱动", exp1),
    "2": ("★★ ADC 的组配置两层", exp2),
    "3": ("★ PWM：占空比就是 CCR", exp3),
    "4": ("★★ DMA + cache", exp4),
    "5": ("★★ AP 和 HAL 的五种关系", exp5),
}


def main():
    print()
    hr("#")
    print("#  第 18 课：ADC / PWM / GPT / CAN")
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
    print("  #  实验结束")
    hr("#")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
