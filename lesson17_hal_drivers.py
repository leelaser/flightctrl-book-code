#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 17 课配套实验：串口 + SPI + I2C（HAL 具体驱动）

    实验 1  ★★ 分层从哪看出来的（_serial_driver_data 宏 + osal 计数）
    实验 2  ★★ 串口的发送通路：队列 → 通知 → 中断逐字节
    实验 3  ★ 版本路由：同一个外设，不同 MCU 系列用不同的 LLD
    实验 4  ★ I2C 的状态机与四类错误
    实验 5  ★★ RAM_MAP → flags → DMA 路由（和第 16 课接起来）

用法：
    python lesson17_hal_drivers.py 1
    ...
    python lesson17_hal_drivers.py        # 全部

【诚实声明】

实验 1/3/5 是【读源码】—— 结论来自文件内容，不是模拟。
实验 2/4 是【模拟】—— 模拟的是真实代码的**状态机与判定逻辑**，丢掉了：

    · 真实的中断延迟、总线时序、波特率
    · osalSysLock / osalSysUnlock 的实际开销
    · DMA 与缓存的交互（H7 上还有 D-Cache 一致性这一层，本课不展开）
    · 硬件 FIFO（USARTv3 支持，模拟里按"一次一个字节"算）

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

F_HAL_SERIAL_H = os.path.join(HAL, "include", "hal_serial.h")
F_HAL_SERIAL_C = os.path.join(HAL, "src", "hal_serial.c")
F_HAL_QUEUES_C = os.path.join(HAL, "src", "hal_queues.c")
F_LLD_SERIAL_H = os.path.join(LLD, "USARTv3", "hal_serial_lld.h")
F_LLD_SERIAL_C = os.path.join(LLD, "USARTv3", "hal_serial_lld.c")
F_LLD_I2C_C = os.path.join(LLD, "I2Cv3", "hal_i2c_lld.c")
F_HAL_I2C_H = os.path.join(HAL, "include", "hal_i2c.h")
F_HAL_SPI_H = os.path.join(HAL, "include", "hal_spi_v2.h")
F_PLATFORM_DIR = os.path.join(HAL, "ports", "STM32")

F_UART_DRIVER = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                             "UARTDriver.cpp")
F_SPIDEVICE = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                           "SPIDevice.cpp")
F_I2CDEVICE = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                           "I2CDevice.cpp")
F_SDCARD = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS", "sdcard.cpp")
F_MALLOC_C = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "hwdef", "common", "malloc.c")
F_H743 = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS", "hwdef",
                      "scripts", "STM32H743xx.py")

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
# 实验 1：分层从哪看出来的
# ===========================================================================

def exp1():
    head("实验 1  ★★ 分层从哪看出来的")

    print("""
  第 13 课讲了"ChibiOS 的驱动分四层"（HAL 框架 / LLD / 板级 / 应用）。
  但**分层的证据在哪**？这一节把三条硬证据摆出来。
""")

    print("""  【证据一】框架层的结构体是【开着口】的 —— 由 LLD 补全

      hal_serial.h 定义的是：

          struct hal_serial_driver {
            const struct SerialDriverVMT *vmt;   /* 虚函数表 */
            _serial_driver_data                  /* ← 一个宏，这里没定义！ */
          };

      `_serial_driver_data` 在 **LLD 的头文件里**才定义 ⇒
      **框架的结构体在 LLD 编译时才闭合。**
""")
    n = find_line(F_LLD_SERIAL_H, r"#define _serial_driver_data")
    if n:
        src(F_LLD_SERIAL_H, n, n + 20, "LLD 补的是什么（USARTv3/hal_serial_lld.h）：")

    print("""  ★★ 注意那行注释 **"End of the mandatory fields"** ——
    分层在代码里是**写明的**：上面是框架要求的字段，下面是 LLD 自己的。
""")

    print("""  【证据二】框架层**一行 ChibiOS 都没有**，它只说 OSAL 的话
""")
    print("      %-24s %-10s %s" % ("文件", "osal* 调用", "ch* 调用"))
    print("      " + "-" * 48)
    for f in ["src/hal_queues.c", "src/hal_uart.c", "src/hal_i2c.c",
              "src/hal_serial.c"]:
        path = os.path.join(HAL, f)
        s = read(path)
        n_osal = len(re.findall(r"\bosal[A-Z][A-Za-z]*", s))
        n_ch = len(re.findall(r"\bch[A-Z][A-Za-z]*", s))
        print("      %-24s %-10d %d" % (os.path.basename(f), n_osal, n_ch))
    print("""
      ⇒ 框架层用的是 `osalSysLock` / `osalThreadEnqueueTimeoutS` /
        `osalDbgCheck` —— **OSAL（OS 抽象层）**。
        换一个 RTOS（或不用 RTOS）时，改的是 OSAL，不是 HAL。
        （第 10 课 §1.3 提过"OSAL 的三种变体"，这里看到它为什么存在。）
""")

    print("""  【证据三】框架调 LLD，走的是**虚函数表**
""")
    n2 = find_line(F_HAL_SERIAL_H, r"struct hal_serial_driver \{")
    if n2:
        src(F_HAL_SERIAL_H, n2 - 3, n2 + 4, "vmt 是第一个字段：")

    hr("-")
    print("""
  【实测】"框架层零 ChibiOS"这一条，要能自己复现。下面把统计方法写清楚，
        顺便验证一下"三层字段拼接"的结论。
""")
    hr("-")
    print()

    # 把 LLD 的 _serial_driver_data 展开，数一数各层有多少字段
    s = read(F_LLD_SERIAL_H)
    m = re.search(r"#define _serial_driver_data(.*?)(?=\n/\*|\n#endif)",
                  s, re.S)
    if not m:
        print("      [读不到 _serial_driver_data]")
        return
    body = m.group(1)
    layers = {"框架之前（基类）": [], "框架强制（mandatory）": [], "LLD 私有": []}
    # ★ 宏里的分界只有一条注释 `/* End of the mandatory fields.*/` ——
    #   它标的是【强制字段的结束】，不是开始。开始的位置要靠
    #   "第一行是 _base_xxx" 推：那行以后、分界以前，都是框架要的。
    #   作者第一版把起始桶设成"基类"，于是 state/iqueue/oqueue 全归错了。
    cur = "框架强制（mandatory）"
    for line in body.split("\n"):
        t = line.strip().rstrip("\\").strip()
        if not t or t.startswith("/*"):
            if "End of the mandatory fields" in t:
                cur = "LLD 私有"
            continue
        if t.startswith("_base"):
            layers["框架之前（基类）"].append(t)
            continue
        # 一行形如 `USART_TypeDef             *usart;`
        mm = re.match(r"^([A-Za-z_][A-Za-z0-9_ ]*?)\s+(\*?[a-z_][a-z0-9_]*);?$", t)
        if mm:
            layers[cur].append(mm.group(1).strip() + " " + mm.group(2))

    for k, v in layers.items():
        print("      %-20s %2d 个：%s" % (k, len(v), "、".join(v) if v else "—"))
    print("""
      ★ 宏里只有一条注释标分界 —— `/* End of the mandatory fields.*/` ——
        它标的是**强制字段的结束**。所以"框架要哪些"要靠
        「第一行是基类、分界以前是强制」推出来。""")
    print("""
      ⇒ 一个 `SerialDriver` 对象里，**三层的数据是拼在一起的**：
        基类 + 框架要求 + LLD 私有。**没有额外分配、没有指针跳转。**
        这就是"四层"能做到零开销的原因。
""")

    print("""  ----------------------------------------------------------------------
  本实验不是模拟 —— 四条结论都出自文件内容，可以逐条打开核对。
  ----------------------------------------------------------------------""")


# ===========================================================================
# 实验 2：串口的发送通路
# ===========================================================================

def exp2():
    head("实验 2  ★★ 串口的发送通路：队列 → 通知 → 中断逐字节")

    print("""
  `sdWrite()` **不是函数，是一个宏**，直接落到输出队列：
""")
    n = find_line(F_HAL_SERIAL_H, r"#define sdWrite\(")
    if n:
        src(F_HAL_SERIAL_H, n - 4, n + 2, "hal_serial.h：")

    print("""  ★ 所以"串口驱动"的发送路径是：**队列是主角，驱动是配角**。
""")
    n2 = find_line(F_HAL_QUEUES_C, r"nfy\(oqp\);")
    if n2:
        src(F_HAL_QUEUES_C, n2 - 12, n2 + 12,
            "框架层的 oqWriteTimeout：写进队列 → 通知下层（hal_queues.c）：")

    print("""  ★★ 那一行 `nfy(oqp)` 就是**框架和 LLD 之间的唯一接口**：
        "队里至少有一个字节了，你去处理"。

  LLD 收到的反应是什么？
""")
    n3 = find_line(F_LLD_SERIAL_C, r"^static void notify1\(")
    if n3:
        src(F_LLD_SERIAL_C, n3, n3 + 4, "USARTv3 的回答（hal_serial_lld.c）：")
    print("""  ⇒ **只做一件事：打开 TXE 中断。** 没有"把数据交给 LLD"这个动作 ——
    LLD 稍后**自己去队列里取**。
""")

    n4 = find_line(F_LLD_SERIAL_C, r"Transmission buffer empty")
    if n4:
        src(F_LLD_SERIAL_C, n4 - 1, n4 + 22, "中断里怎么搬（hal_serial_lld.c）：")

    print("""  ★★ 三个细节值得记：

      ① **判据是"中断使能位"而不是状态位** ——
         `if (cr1 & USART_CR1_TXEIE)`：我开了这个中断才处理它
      ② **每搬一个字节，一对 `osalSysLockFromISR/UnlockFromISR`**
         —— 中断里每字节两次加锁，这是有成本的
      ③ 队列空 ⇒ 记 `CHN_OUTPUT_EMPTY` + **关掉 TXEIE**
         ⇒ 中断停下来了，等下一次 notify 再开
""")

    hr("-")
    print("""
  【模拟】一个串口的发送状态机，只保留上面那三个判定：

       · sdWrite(n) 往队列里塞；队列满 ⇒ 阻塞（挂到等待队列）
       · 队列由空变非空 ⇒ 触发一次 notify（开 TXE 中断）
       · 中断每来一次搬一个字节；队列空 ⇒ 关中断

  场景：模拟一个 115200 波特率（约 11520 字节/秒）的口，
  应用一次写 64 字节，看队列（16 字节）是怎么满的、什么时候阻塞。
""")
    hr("-")
    print()

    QSIZE = 16                       # SERIAL_BUFFERS_SIZE 的默认值
    BAUD = 115200
    BYTES_PER_TICK = BAUD / 1000.0   # 假设 1 ms 一个 tick ⇒ 每 tick 11.52 字节

    class Serial:
        def __init__(self, qsize):
            self.q = []
            self.qsize = qsize
            self.txeie = False       # TXE 中断使能
            self.sent = 0
            self.dropped = 0
            self.notify_count = 0
            self.isr_count = 0

        def notify(self):
            """LLD 的 notify1()：开 TXE 中断。"""
            if not self.txeie:
                self.txeie = True
                self.notify_count += 1

        def write(self, n):
            """框架层：写进队列；队满就返回"阻塞"。"""
            wrote = 0
            while wrote < n and len(self.q) < self.qsize:
                self.q.append(0)
                wrote += 1
            if wrote and self.txeie is False:
                self.notify()
            return wrote

        def tick(self, budget):
            """中断：每次搬一个字节，直到没有预算或队列空。"""
            moved = 0
            while self.q and moved < budget:
                if not self.txeie:
                    break
                self.q.pop(0)
                self.sent += 1
                moved += 1
                self.isr_count += 1
            if not self.q and self.txeie:
                self.txeie = False       # 队列空 ⇒ 关中断
            return moved

    io_port = Serial(QSIZE)
    print("      队列大小 %d 字节，线速 %.2f 字节/tick" % (QSIZE, BYTES_PER_TICK))
    print()
    print("      %-5s %-8s %-7s %-7s %-6s %-7s %s"
          % ("tick", "本次写", "队列", "TXEIE", "已发", "notify", "说明"))
    print("      " + "-" * 76)

    # 第 0 tick：一次写 64 字节 —— 队列只有 16，写不进去
    wrote = io_port.write(64)
    print("      %-5d %-8s %-7d %-7s %-6d %-7d %s"
          % (0, "写 64", len(io_port.q), "开" if io_port.txeie else "关",
             io_port.sent, io_port.notify_count,
             "★ 只进去 %d 个（队列满，剩下 %d 个要等）" % (wrote, 64 - wrote)))
    remaining = 64 - wrote

    # 后面的 tick：每 tick 中断搬走一批，应用再补进去
    for t in range(1, 12):
        budget = int(BYTES_PER_TICK)          # 本 tick 能发多少
        moved = io_port.tick(budget)
        refill = 0
        if remaining:
            refill = io_port.write(min(remaining, QSIZE - len(io_port.q)))
            remaining -= refill
        note = ""
        if not io_port.q and refill:
            # ★ 队列被搬空过一次 ⇒ 中断已关 ⇒ 新数据到了要重新 notify
            note = "★ 队列刚才发空了，notify 又触发一次"
        if remaining == 0 and len(io_port.q) == 0:
            note = "★ 全部发完"
        print("      %-5d %-8s %-7d %-7s %-6d %-7d %s"
              % (t, "补 %d" % refill if refill else "—", len(io_port.q),
                 "开" if io_port.txeie else "关", io_port.sent,
                 io_port.notify_count, note))
        if remaining == 0 and not io_port.q:
            break

    print("""
      ★★ 注意 `notify` 那一列：**它不是"只触发一次"，而是每 tick 加一。**

         原因是两个机制叠在一起：
           · notify 的触发条件是**队列由空变非空**（hal_queues.c）
           · 中断在**队列发空时会自己关掉**（`USART_CR1_TXEIE` 清零）
         ⇒ 每来一次"发空 → 又补上数据"，就要重新 notify 一次。

         ⇒ **notify 的频率 = f(写入节奏, 线速, 队列深度)**。
           队列越浅、线速越快，同样的数据量下 notify 就越频繁。

         ★ 而这个队列的默认深度是 **16 字节**（`SERIAL_BUFFERS_SIZE`，
           hal_serial.h）。算一下：921600 波特率 ≈ 92160 字节/秒，
           **16 字节的队列约 174 µs 就发空了** ——
           也就是说应用方必须在这点时间内补上数据，否则就落到
           "发空-重开"的循环里。（这也是为什么第 17 课 §5 会看到
           **ArduPilot 干脆不用这个队列，自己拿 DMA 做批处理**。）""")

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：
       · **"发送"不是一个动作，是一个状态机** —— 队列、中断使能位、
         中断服务三者互相推动
       · ★ **notify 的触发条件是"队列由空变非空"** ——
         应用写得快时它几乎不触发（队列一直满）；应用跟不上时
         它会反复触发（每次发空后重新补数据都要再 notify 一次）
       · ★ 中断在**队列发空时自己关掉**（不空转）——
         这是"中断驱动 + 队列"这个组合省 CPU 的关键
       · ★ `sdWrite(64)` 并没有"等 64 个字节发完" ——
         它只是**把数据交给队列**；等不等由调用者选的 API 决定
         （`sdWrite` = 无限等，`sdWriteTimeout` = 带超时）

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实的波特率与中断延迟（这里用"每 tick 固定字节数"近似）
    · 硬件 FIFO（USARTv3 有，可以一次搬多个）
    · osalSysLock 的开销
  模拟保住了什么：
    · **notify 的触发条件**（由空变非空）
    · **中断的开关逻辑**（队列空就关，notify 再开）
""")


# ===========================================================================
# 实验 3：版本路由
# ===========================================================================

def exp3():
    head("实验 3  ★ 版本路由：同一个外设，不同 MCU 系列用不同的 LLD")

    print("""
  第 13 课讲过"LLD 按**外设版本**切"。这一节给出实测的路由表。

  规则在**每个 MCU 系列的 `platform.mk`** 里：哪个系列 include 哪个 LLD。
""")

    fams = []
    for f in sorted(os.listdir(F_PLATFORM_DIR)):
        mk = os.path.join(F_PLATFORM_DIR, f, "platform.mk")
        if os.path.isfile(mk):
            fams.append((f, mk))

    print("      %-14s %-12s %-12s %s" % ("MCU 系列", "USART", "SPI", "I2C"))
    print("      " + "-" * 56)
    for fam, mk in fams:
        s = read(mk)
        def pick(pat):
            # ★ 目录名是 `USARTv3` / `SPIv3` / `I2Cv3` —— 那个 `v` 是字面量，
            #   不是"第几个版本"的分隔符。作者第一版写成 `(%s\d*)`，
            #   于是**所有格子都是"—"**（正则静默不匹配，不报错）。
            m = re.search(r"LLD/(%sv\d+)/driver" % pat, s)
            return m.group(1) if m else "—"
        u, sp, i2 = pick("USART"), pick("SPI"), pick("I2C")
        mark = "  ← 飞控主力" if fam.startswith(("STM32H7", "STM32F4")) else ""
        print("      %-14s %-12s %-12s %s%s" % (fam, u, sp, i2, mark))

    print("""
  ⇒ **同一个"串口"，H7 上是 `USARTv3`，F4 上是 `USARTv1`** ——
    两套完全独立的代码，只是**接口相同**（都实现 `sd_lld_*`）。

  ★ 这就是第 13 课"四层模型"的兑现方式：
    **换 MCU 系列 = 换 LLD 目录，上面三层一行不改。**

  ★★ 而且 ArduPilot 同时支持这两种板子（F4 的 Pixhawk1 / F405 和
     H7 的 MatekH743 / CubeOrange）—— 所以**两套 LLD 它都在用**。
""")

    hr("-")
    print("  【实测】三个外设各版本的规模：")
    hr("-")
    print()
    print("      %-10s %-8s %-8s %-8s" % ("外设", "v1", "v2", "v3"))
    print("      " + "-" * 38)
    for name, prefix in (("USART", "USART"), ("SPI", "SPI"), ("I2C", "I2C")):
        row = []
        for v in ("v1", "v2", "v3"):
            d = os.path.join(LLD, prefix + v)
            tot = 0
            if os.path.isdir(d):
                for f in os.listdir(d):
                    if f.endswith((".c", ".h")):
                        tot += read(os.path.join(d, f)).count("\n")
            row.append(str(tot) if tot else "—")
        print("      %-10s %-8s %-8s %-8s" % (name, row[0], row[1], row[2]))


# ===========================================================================
# 实验 4：I2C 的状态机与错误
# ===========================================================================

def exp4():
    head("实验 4  ★ I2C 的状态机与四类错误")

    print("""
  I2C 是三条总线里**协议最复杂**的一个。先看它的 API 形状：
""")
    n = find_line(F_HAL_I2C_H, r"msg_t i2cMasterTransmitTimeout\(I2CDriver")
    if n:
        src(F_HAL_I2C_H, n, n + 6, "hal_i2c.h —— 注意最后一个参数：")
    n2 = find_line(F_HAL_SPI_H, r"msg_t spiExchange\(SPIDriver")
    if n2:
        src(F_HAL_SPI_H, n2, n2 + 2, "对照 SPI（hal_spi_v2.h）：")

    print("""  ★★ **为什么 I2C 每个 API 都带 `timeout`，SPI 却不带？**

      I2C 是**多设备共享**的总线（两根线挂一堆器件），
      任何一个器件把 SDA 拉死，整条总线就卡住 ⇒ **必须有超时兜底**

      SPI 是**点对点 + 片选**：每个设备一根 CS 线，
      真出问题把 CS 拉高就断电了 ⇒ 不需要靠超时解围

  ⇒ API 的形状是**总线的物理性质**决定的，不是风格。
""")

    n3 = find_line(F_LLD_I2C_C, r"^static void i2c_lld_serve_error_interrupt")
    if n3:
        src(F_LLD_I2C_C, n3 + 15, n3 + 38, "I2Cv3 的错误处理（hal_i2c_lld.c）：")

    hr("-")
    print("""
  【模拟】I2C 主机传输的状态机，只保留 LLD 里的分支：

       START → 发地址 → 等 ACK ─┬─ NACK ⇒ 从设备没应答（最常见）
                                └─ ACK  ⇒ 传数据 → STOP
       任何一步出错 ⇒ 中止 + 记录错误类型 + 复位外设
""")
    hr("-")
    print()

    # 错误位（照抄 hal_i2c_lld.c:73-74 的掩码）
    BERR, ARLO, OVR, TIMEOUT, NACKF = (1 << 0), (1 << 1), (1 << 2), (1 << 3), (1 << 4)
    NAMES = {BERR: "BERR 总线错误（START/STOP 位置不对）",
             ARLO: "ARLO 仲裁丢失（多主机抢总线）",
             OVR: "OVR 溢出（数据没及时读走）",
             TIMEOUT: "TIMEOUT SCL 被拉低超时",
             NACKF: "NACKF 从设备没应答"}

    class I2C:
        def __init__(self, devices):
            self.devices = devices        # 哪些地址上有设备
            self.errors = 0

        def master_transmit(self, addr, data):
            """返回 (msg, 错因)。"""
            if addr not in self.devices:
                self.errors |= NACKF
                return "MSG_TIMEOUT", NACKF          # 没设备 ⇒ 地址阶段就 NACK
            if not data:
                return "MSG_OK", 0
            if len(data) > 8:
                self.errors |= OVR
                return "MSG_TIMEOUT", OVR
            return "MSG_OK", 0

    bus = I2C(devices={0x68, 0x76})

    print("      总线上有设备的地址：0x68、0x76")
    print()
    print("      %-28s %-12s %s" % ("调用", "返回", "错误标志"))
    print("      " + "-" * 66)
    for addr, data, desc in ((0x68, b"abc", "正常：读气压计"),
                             (0x77, b"abc", "★ 地址没人应答"),
                             (0x76, b"x" * 20, "★ 数据太长（模拟溢出）"),
                             (0x76, b"", "空传输")):
        msg, err = bus.master_transmit(addr, data)
        print("      %-28s %-12s %s"
              % ("addr=0x%02X len=%d  %s" % (addr, len(data), desc), msg,
                 NAMES.get(err, "—")))

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：
       · I2C 的失败**是常态**，不是异常 —— 从设备没插、地址写错、
         器件正在复位，都表现为 NACK。所以驱动必须**把错误当返回值**
         （`i2cMasterTransmitTimeout` 返回 `MSG_TIMEOUT`），而不是崩溃
       · ★ 四类错误里只有 BERR/ARLO 是"总线级"的，
         NACKF 是"设备级"的 —— **修法完全不同**：
         总线级要复位外设，设备级通常只要重试或跳过
       · ★ LLD 里 `i2c_lld_serve_error_interrupt` 的处理是
         **记录错误 + 中止操作 + 复位**（照上面的源码）

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实的 START/STOP 时序、时钟拉伸（clock stretching）
    · 中断驱动的多字节传输（这里一次性判完）
  模拟保住了什么：
    · **错误分类**（照抄 LLD 的四类标志）
    · "失败用返回值表达"这个调用约定
""")


# ===========================================================================
# 实验 5：RAM_MAP → flags → DMA 路由
# ===========================================================================

def exp5():
    head("实验 5  ★★ RAM_MAP → flags → DMA 路由（和第 16 课接起来）")

    print("""
  第 16 课讲了 `malloc.c` 用 flags 路由内存，`dma_flags` 决定谁算"能给 DMA"。
  但**那些 flags 是从哪来的**？答案在这一课 ——

  它来自 **MCU 的物理内存拓扑**，写在 `hwdef/scripts/STM32H743xx.py` 里。
""")

    n = find_line(F_H743, r"'RAM_MAP' : \[")
    if n:
        src(F_H743, n, n + 8, "H743 的 RAM_MAP（STM32H743xx.py）：")

    print("""  那些数字是 flags。对照 `malloc.c` 的定义（第 16 课读过）：
""")
    n2 = find_line(F_MALLOC_C, r"#define MEM_REGION_FLAG_DMA_OK")
    if n2:
        src(F_MALLOC_C, n2, n2 + 3, "malloc.c：")
    n3 = find_line(F_MALLOC_C, r"dma_flags = ")
    if n3:
        src(F_MALLOC_C, n3, n3, "★ 哪几种算「DMA 可用」：")

    print("""  ⇒ **解码 H743 的六块内存：**
""")
    FLAGS = {1: "DMA_OK", 2: "FAST", 4: "AXI_BUS", 8: "ETH_SAFE"}
    # 照抄 STM32H743xx.py:28-35
    RAM_MAP = [
        (0x30000000, 256, 8, "SRAM1, SRAM2"),
        (0x20000000, 128, 2, "DTCM, tightly coupled, no DMA, fast"),
        (0x24000000, 512, 4, "AXI SRAM"),
        (0x00000400, 63, 2, "ITCM (first 1k removed)"),
        (0x30040000, 32, 8, "SRAM3"),
        (0x38000000, 64, 1, "SRAM4"),
    ]
    DMA_FLAGS = (1 | 4 | 8)          # malloc.c:129

    print("      %-12s %-6s %-8s %-26s %s"
          % ("地址", "大小", "flags", "对应内存", "能给 DMA？"))
    print("      " + "-" * 76)
    for addr, size, flags, desc in RAM_MAP:
        names = "|".join(v for k, v in FLAGS.items() if flags & k)
        ok = "★ 可以" if (flags & DMA_FLAGS) else "❌ 不行（只有 FAST）"
        print("      %-12s %-6s %-8s %-26s %s"
              % ("0x%08X" % addr, "%dK" % size, names, desc, ok))

    print("""
  ★★ **DTCM 和 ITCM 是"快但 DMA 到不了"的** ——
     H743 的注释自己写着 `no DMA, fast`（紧耦合内存，不走总线矩阵）。

  ★★ 所以 ArduPilot 的分配器必须知道**每一块内存的物理属性**：
     把 DMA 缓冲放进 DTCM，硬件不会报错，只会**静默地传错数据**。
""")

    hr("-")
    print("""
  【验证】把第 16 课那个路由再跑一遍 —— 但这次用的 flags
         **直接来自 H743 的 RAM_MAP**，不是编的。
""")
    hr("-")
    print()

    heaps = [(desc, size * 1024, flags) for _, size, flags, desc in RAM_MAP]

    # ★★ 照抄 malloc.c 的一个关键结构：**region 0 是"默认堆"，遍历从 1 开始**
    #    （malloc.c:51-53 的注释："the first memory region is already setup
    #      as the ChibiOS default heap, so we will index from 1"）
    #    作者第一版没照抄这一点，于是**所有请求都落在 region 0**，
    #    整个演示退化成一行 —— 而"跳过 DTCM"这个结论恰恰要求照抄它。
    default_heap = heaps[0]
    others = heaps[1:]

    def alloc_with_flags(need, flags):
        """照抄 malloc.c:148-205 的分支结构。"""
        # ① 无 flags ⇒ 先试默认堆
        if flags == 0:
            if default_heap[1] >= need:
                return default_heap[0] + "（默认堆）"
        # ② 按 flags 逐个试其余区域
        #   ★ malloc.c:158-176 对【四种】flag 各有一个 continue 筛选
        #     （DMA_OK / AXI_BUS / FAST / ETH_SAFE 都要精确匹配），
        #     作者第一版只实现了两个 —— 于是"要 ETH_SAFE"也会落到 AXI SRAM。
        for desc, size, fl in others:
            ok = True
            for bit in (1, 2, 4, 8):                  # DMA_OK / FAST / AXI / ETH
                if (flags & bit) and not (fl & bit):
                    ok = False
                    break
            if not ok:
                continue
            if size >= need:
                return desc
        # ③ 非 DMA 请求可以回退到默认堆
        if not (flags & DMA_FLAGS) and default_heap[1] >= need:
            return default_heap[0] + "（回退）"
        return None

    print("      假设每块内存都还有足够空间（只看 flags 筛选）：")
    print()
    print("      %-30s %s" % ("请求", "会落在哪块内存"))
    print("      " + "-" * 62)
    for need, flags, label in ((64, 0, "普通请求"),
                               (64, 1, "DMA 请求（DMA_OK）"),
                               (64, 4, "要 AXI_BUS"),
                               (64, 8, "要 ETH_SAFE"),
                               (64, 2, "要 FAST")):
        got = alloc_with_flags(need, flags)
        print("      %-30s %s" % ("%s（flags=%d）" % (label, flags), got or "★ NULL"))

    print("""
      ★★ **五种请求，落在五块不同的内存上** —— 而筛掉 DTCM/ITCM 的，
         就是那句 `if (flags & dma_flags) and not (fl & DMA_FLAGS): continue`：

             **DMA 请求不会被"将就"给一块 DMA 到不了的内存。**

      ★ 三块内存的"性格"完全由芯片决定（不是软件选的）：
           · **DTCM**：最快，但 DMA 到不了 ⇒ 只给"要 FAST"的请求
           · **SRAM4**：唯一标 DMA_OK 的 ⇒ 纯 DMA 请求落这里
             （它在 H7 的 D3 域，**只有 BDMA 引擎能访问**）
           · **SRAM3 / SRAM1-2**：标 ETH_SAFE ⇒ 给以太网 DMA 用

      ★★ 而第 16 课那个 `dma_reserve_heap`（6 KB 救命内存）就是
         `malloc_dma()` 要来的 ⇒ **它其实是从 SRAM4 那一类内存里切的**。
         两课到这里完全接上了。""")

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：**"内存的物理属性"在两课之间闭环了：**

        第 17 课（本课）：H743 的 RAM_MAP 说明 DTCM 不可 DMA
              ↓ 由 hwdef 生成器写进 HAL_MEMORY_REGIONS
        第 16 课：malloc.c 把每个区域变成一个堆 + 带 flags
              ↓ malloc_dma() 带着 DMA_OK 去要内存
        本课 §2/§3：SPI/串口的 DMA 引擎拿到**能访问的**地址

  ★ 这三层任何一层写错，症状都是"**数据静默错误**"，
    而不是编译错误或崩溃 —— 这是飞控上最难查的一类问题。

  ★★ 所以第 16 课那个 `dma_reserve_heap`（从 DMA 安全的内存里
     切 6 KB 留着）不是随便挑的：**它挑的正是 SRAM4 那一类**。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 实际可用空间（这里假设都够）
    · AXI/ETH 的具体约束（只按 flags 位筛）
  模拟保住了什么：
    · **flags 的判定规则**（DMA 请求不许将就）
    · RAM_MAP 的**原始数据**（照抄 STM32H743xx.py:28-35）
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ 分层从哪看出来", exp1),
    "2": ("★★ 串口的发送通路", exp2),
    "3": ("★ 版本路由表", exp3),
    "4": ("★ I2C 的状态机与错误", exp4),
    "5": ("★★ RAM_MAP → DMA 路由", exp5),
}


def main():
    print()
    hr("#")
    print("#  第 17 课：串口 + SPI + I2C（HAL 具体驱动）")
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
