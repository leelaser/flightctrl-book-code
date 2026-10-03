#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 20 课配套实验：ArduPilot 如何用 ChibiOS

    实验 1  启动流程（三层展开）
    实验 2  ★★★ 优先级编排（本课核心）
    实验 3  模块生命周期（对照 PX4）
    实验 4  ChibiOS 特性使用清单（对照前 11 课）
    实验 5  那 10 项绕过的核实

用法：
    python lesson20_ardupilot_chibios.py 1
    python lesson20_ardupilot_chibios.py 2
    ...
    python lesson20_ardupilot_chibios.py      # 全部

源码版本：ArduPilot 4.5.7 + ChibiOS 6a85082c
"""

import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APM_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "APM",
                 "ardupilot-Copter-4.5.7", "ardupilot-Copter-4.5.7")
)
PX4_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "PX4", "PX4-Autopilot")
)

HALCHIB = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS")
COMMON = os.path.join(HALCHIB, "hwdef", "common")

F_HALCLASS = os.path.join(HALCHIB, "HAL_ChibiOS_Class.cpp")
F_SCHED_C = os.path.join(HALCHIB, "Scheduler.cpp")
F_SCHED_H = os.path.join(HALCHIB, "Scheduler.h")
F_CHCONF = os.path.join(COMMON, "chconf.h")
F_MAIN_H = os.path.join(APM_ROOT, "libraries", "AP_HAL", "AP_HAL_Main.h")
F_VEHICLE_H = os.path.join(APM_ROOT, "libraries", "AP_Vehicle", "AP_Vehicle.h")
F_COPTER_C = os.path.join(APM_ROOT, "ArduCopter", "Copter.cpp")
F_INS_C = os.path.join(APM_ROOT, "libraries", "AP_InertialSensor",
                       "AP_InertialSensor.cpp")
F_AHRS_C = os.path.join(APM_ROOT, "libraries", "AP_AHRS", "AP_AHRS.cpp")


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


def find_all(path, pattern):
    """返回 [(1-based 行号, 行内容), ...]。

    ⚠ 这里踩过一个坑：第一版写成
          return [(i, lines[i-1]) for i in range(len(lines)) if ...]
       索引 i 是 0-based，而 lines[i-1] 取的是【上一行】——
       行号全差 1，而且取到的内容也是错的。
       症状是"值"那一列全是 '?'（因为取到了上一行的注释）。

    ⇒ 1-based 就用 enumerate(lines, 1)，不要自己算下标。
    """
    lines = read_lines(path)
    if not lines:
        return []
    rx = re.compile(pattern)
    return [(i, line) for i, line in enumerate(lines, 1) if rx.search(line)]


def hr(ch="=", n=74):
    print(ch * n)


def head(title):
    print()
    hr()
    print("  " + title)
    hr()


# ===========================================================================
# 实验 1：启动流程
# ===========================================================================

def exp1():
    head("实验 1：启动流程（三层展开）")

    print("""
  ArduPilot 的启动链有三层：
      vehicle 的 main  →  AP_HAL_MAIN 宏  →  HAL_ChibiOS::run  →  main_loop
  这个实验把三层都展开。
""")

    print()
    print("  ── 第一层：vehicle 的 main ──")
    print()
    p = find_line(F_COPTER_C, r"AP_HAL_MAIN_CALLBACKS")
    if p:
        show(F_COPTER_C, max(1, p - 6), p, "★ ArduCopter 的入口")

    print()
    print("  ── 第二层：宏展开 ──")
    print()
    p = find_line(F_MAIN_H, r"define AP_HAL_MAIN_CALLBACKS")
    if p:
        show(F_MAIN_H, p, p + 8, "★ AP_HAL_MAIN_CALLBACKS")

    print("""
    ⇒ 展开后就是一个普通的 int main()：
          int main(int argc, char* const argv[]) {
              hal.run(argc, argv, CALLBACKS);
              return 0;
          }

      ★ 而 hal.run 里调 main_loop()，【不返回】。
""")

    print("  ── 第三层：HAL_ChibiOS::run ──")
    print()
    p = find_line(F_HALCLASS, r"void HAL_ChibiOS::run")
    if p:
        show(F_HALCLASS, p, p + 35, "★ run()")

    print()
    print("  ── ★★ 关键注释：main() 变成线程 ──")
    print()
    lines = read_lines(F_HALCLASS) or []
    for i, line in enumerate(lines, 1):
        if "the main() function becomes a thread" in line:
            show(F_HALCLASS, i - 4, i + 2, "★ ChibiOS 的入口模型")
            break

    print("""
    ⇒ ChibiOS 里【main() 本身就是主线程】——
      不需要"启动调度器"这一步。

      对比：
        FreeRTOS   main() 里创建任务 ⇒ 调 vTaskStartScheduler() ⇒ 调度器接管
        NuttX      main() 是 init 线程 ⇒ board_app_initialize() ⇒ 用户代码
        ChibiOS    main() 【就是】主线程（chThdGetSelfX() 拿到自己）

    ★ 这解释了这一行（HAL_ChibiOS_Class.cpp:209）：

          daemon_task = chThdGetSelfX();

      "主线程句柄"就是"我自己"。
""")
    print()


# ===========================================================================
# 实验 2：优先级编排（本课核心）
# ===========================================================================

def exp2():
    head("实验 2：★★★ 优先级编排")

    print("""
  这是本课的核心。

  ArduPilot 不是"用了调度器"，而是把 ChibiOS 的优先级机制
  当成了【时序编程语言】。这个实验把整套机制拆出来。
""")

    print()
    print("  ── ① 完整的优先级表 ──")
    print()

    lines = read_lines(F_SCHED_H) or []
    prios = []
    for i, line in enumerate(lines, 1):
        m = re.match(r"^#define\s+(APM_\w*PRIORITY\w*)\s+(\d+|LOWPRIO)", line)
        if m:
            prios.append((i, m.group(1), m.group(2)))

    # LOWPRIO = 2（chschd.h:53）
    def pval(v):
        return 2 if v == "LOWPRIO" else int(v)

    print("    %-38s %-8s %s" % ("宏", "值", "Scheduler.h 行号"))
    print("    " + "-" * 60)
    for i, name, val in sorted(prios, key=lambda x: -pval(x[2])):
        mark = "  ★ 提权用" if "BOOST" in name else ""
        print("    %-38s %-8s %d%s" % (name, val, i, mark))

    print()
    print("    ⇒ %d 个常量，%d 个不同的优先级。"
          % (len(prios), len(set(pval(v) for _i, _n, v in prios))))

    print("""
    ★★ 这个表不是"随便分的"，它编码了一套【时序策略】：

        183  监控线程          —— 最高，救命的
        182  ★ 主循环【提权】   —— 等采样点时用
        181  TIMER/RCOUT/SPI   —— 时序敏感，不能被拖延
        180  主循环【正常】
        ...
         60  LED/UART/NET     —— 可以慢
         58  IO               —— SD 卡读写
         10  STARTUP
          2  脚本             —— 最低
""")

    print("  ── ② 提权机制 ──")
    print()
    p = find_line(F_SCHED_C, r"static void set_high_priority")
    if p:
        show(F_SCHED_C, p - 4, p + 6, "★ set_high_priority")

    p = find_line(F_SCHED_C, r"void Scheduler::delay_microseconds_boost")
    if p:
        show(F_SCHED_C, p - 6, p + 9, "★★ delay_microseconds_boost")

    print("""
    ★★ 关键在于【提权发生在睡之前】：

        没有提权：
            主循环 (180) 睡觉 → 时间到 → 唤醒
            但 TIMER 是 181 ⇒ TIMER 先跑 ⇒ 主循环要等 ⇒ 【开始时间抖动】

        提权之后：
            主循环提到 182 → 睡觉 → 时间到 → 唤醒
            182 > 181 ⇒ 【立刻抢占 TIMER】⇒ 主循环准时开始

      ⇒ 源码注释原话：
          significantly improves the regularity of timing of the main loop
""")

    print("  ── ③ 降回机制 ──")
    print()
    p = find_line(F_SCHED_C, r"void Scheduler::boost_end")
    if p:
        show(F_SCHED_C, p - 4, p + 9, "★ boost_end")

    print("  ── ④ ★★ 完整的时序闭环：谁提权、谁降回、谁检查 ──")
    print()

    CALLPOINTS = [
        (F_INS_C, r"delay_microseconds_boost\(", "★ 提权", "INS 驱动等采样点"),
        (F_AHRS_C, r"boost_end\(\);", "★ 降回", "AHRS 更新完"),
        (F_HALCLASS, r"check_called_boost", "★ 检查", "主循环每圈"),
    ]
    for path, pat, tag, desc in CALLPOINTS:
        hits = find_all(path, pat)
        print("    %s %-16s %s" % (tag, os.path.basename(path), desc))
        for i, line in hits:
            print("        %-28s %-5d %s"
                  % (os.path.basename(path), i, line.strip()[:46]))
        print()

    print("""    串起来：

        ① INS 驱动等下一个采样点
               AP_InertialSensor.cpp:1943
                   hal.scheduler->delay_microseconds_boost(wait_usec);
               ⇒ 提权到 182 + chThdSleep

        ② 采样点到了 ⇒ 立刻抢占 ⇒ 读 IMU

        ③ AHRS 更新完成
               AP_AHRS.cpp:387
                   hal.scheduler->boost_end();
               ⇒ 降回 180

        ④ 主循环这一圈结束
               HAL_ChibiOS_Class.cpp:296-311
                   if (!schedulerInstance.check_called_boost()) {
                       hal.scheduler->delay_microseconds(50);
                   }
               ⇒ 提权过 ⇒ 不用让位
               ⇒ 没提权过 ⇒ 主动让出 50 µs 给低优先级线程
""")

    print("  ── ⑤ 在源代码里找那个 50 µs ──")
    print()
    p = find_line(F_HALCLASS, r"check_called_boost")
    if p:
        show(F_HALCLASS, max(1, p - 8), p + 8, "★★ 主循环的让位")

    print()
    print("  ── ⑥ ★★ 这套编排依赖 ChibiOS 的什么 ──")
    print()
    print("    %-42s %s" % ("依赖的 ChibiOS 特性", "在哪一课讲的"))
    print("    " + "-" * 66)
    DEPS = [
        ("数值越大优先级越高", "第 11 课 §1.5（两条源码证据）"),
        ("严格优先级抢占（不等时间片）", "第 11 课 §2.2（p1 > p2）"),
        ("★ CH_CFG_TIME_QUANTUM = 0（同级不让位）", "第 11 课 §2.2"),
        ("chThdSetPriority 只改当前线程", "第 12 课 §4.4"),
        ("★ hal_chibios_set_priority 抄了它", "第 12 课 §4.4"),
        ("chThdSleep 用栈上的虚拟定时器", "第 12 课 §5.3"),
        ("realprio vs hdr.pqueue.prio", "第 12 课 §1.3"),
    ]
    for a, b in DEPS:
        print("    %-42s %s" % (a, b))

    print()
    print("""    ★★ 特别注意最后两条：

      ① CH_CFG_TIME_QUANTUM = 0
         如果它 > 0，181 上的 4 个线程（TIMER/RCOUT/SPI/UART_UNBUFFERED）
         会互相轮转 ⇒ 时序不确定 ⇒ 整套编排失效。

      ② realprio
         反复提权/降权不会让基础优先级漂移
         （第 12 课 chThdSetPriority 里那个 if 条件就是为此）

    ⇒ 所以"ArduPilot 如何用 ChibiOS"的准确回答不是"用了调度器"，而是：

        ★★ 它把 ChibiOS 的优先级机制当成了一套【时序编程语言】。

           优先级      = 时间约束的【紧急程度】
           提权/降权   = 动态调整紧急程度
           TIME_QUANTUM = 0  ⇒  让"紧急程度"是【绝对的】

    ⚠ 这个用法在通用 RTOS 教材里很少讲 ——
      因为通用场景不需要这么精细的时序。飞控需要。
""")
    print()


# ===========================================================================
# 实验 3：模块生命周期
# ===========================================================================

def exp3():
    head("实验 3：模块生命周期（对照 PX4）")

    print("""
  ArduPilot 的模块是【编译期静态】的全局对象。
  这个实验看它的生命周期钩子，并和 PX4 对照。
""")

    print()
    print("  ── ① AP_Vehicle 的三个钩子 ──")
    print()
    p = find_line(F_VEHICLE_H, r"class AP_Vehicle")
    if p:
        show(F_VEHICLE_H, p, p + 30, "★ class AP_Vehicle")

    print()
    print("  ── ② 三个钩子的关系 ──")
    print()
    print("    %-22s %-30s %s" % ("钩子", "谁实现", "什么时候调"))
    print("    " + "-" * 68)
    print("    %-22s %-30s %s"
          % ("setup()", "AP_Vehicle::setup()", "main_loop 里（低优先级）"))
    print("    %-22s %-30s %s"
          % ("init_ardupilot()", "各 vehicle（纯虚函数）", "setup() 内部调"))
    print("    %-22s %-30s %s"
          % ("loop()", "AP_Vehicle::loop()", "主循环里（高优先级）"))
    print()
    print("    ⇒ 全是【编译期绑定】的虚函数调用。")

    print()
    print("  ── ③ 对照 PX4 ──")
    print()
    print("    %-24s %-28s %s"
          % ("", "ArduPilot + ChibiOS", "PX4 + NuttX"))
    print("    " + "-" * 76)
    ROWS = [
        ("模块是什么", "一个全局对象", "一个任务"),
        ("怎么启动", "编译期直接调 &copter", "运行期 px4_task_spawn_cmd()"),
        ("模块间通信", "直接函数调用 / 共享对象", "uORB 发布订阅"),
        ("能动态加载吗", "❌", "✅（/bin/xxx）"),
        ("加一个新模块要", "改代码 + 重新编译", "写一个新命令"),
        ("依赖的系统能力", "无", "★ VFS（/dev/xxx、/bin/xxx）"),
    ]
    for r in ROWS:
        print("    %-24s %-28s %s" % r)

    print("""
    ★★ 这个差别不是"设计品味"，是【RTOS 能力】决定的：

        NuttX 有 VFS
            ⇒ 每个模块可以是 /bin 下的一个文件
            ⇒ 运行期起一个任务
            ⇒ 模块之间通过 /dev、uORB 解耦

        ChibiOS 没有 VFS
            ⇒ 模块只能是编译进固件的一个对象
            ⇒ 编译期决定谁存在
            ⇒ 模块之间直接调用

    ⇒ 这正是《NuttX 教程》第 10 课 §6.5 反事实推演的核心：
      "如果 PX4 当年选了 ChibiOS，它的 uORB + WorkQueue 模式就不成立"。

    ★ 两种模型的代价：

        %-16s %-32s %s
        %s
        %-16s %-32s %s
        %-16s %-32s %s
""" % ("编译期静态", "❌ 改一个模块要重编整个固件", "灵活性",
       "-" * 76,
       "编译期静态", "✅ 体积/时序/内存全都编译期可知", "可预测性",
       "运行期动态", "⚠ 运行期才知道有什么在跑", ""))

    print("""      ⇒ 飞控领域【可预测性】很重要，所以 ArduPilot 的选择是合理的。
""")
    print()


# ===========================================================================
# 实验 4：ChibiOS 特性使用清单
# ===========================================================================

def exp4():
    head("实验 4：ChibiOS 特性使用清单")

    print("""
  把前 11 课讲过的 ChibiOS 特性，逐条核对 ArduPilot 用没用。
  这个实验【从 chconf.h 读实际值】，不是照抄文档。
""")

    print()
    print("  ── 内核特性（从 chconf.h 读实际值）──")
    print()

    FEATURES = [
        ("CH_CFG_ST_FREQUENCY", "系统 tick 频率", "1 MHz"),
        ("CH_CFG_ST_TIMEDELTA", "tickless 模式", "10"),
        ("CH_CFG_TIME_QUANTUM", "★ 时间片轮转", "0 = 关"),
        ("CH_CFG_USE_TM", "时间测量（TM）", "TRUE"),
        ("CH_CFG_USE_MUTEXES", "互斥量 + PI", "TRUE"),
        ("CH_CFG_USE_MUTEXES_RECURSIVE", "递归互斥量", "TRUE"),
        ("CH_CFG_USE_CONDVARS", "条件变量", "FALSE"),
        ("CH_CFG_USE_MESSAGES", "消息", "FALSE"),
        ("CH_CFG_USE_SEMAPHORES", "信号量", "TRUE"),
        ("CH_CFG_USE_EVENTS", "事件", "TRUE"),
        ("CH_CFG_USE_DYNAMIC", "动态线程", "TRUE"),
        ("CH_CFG_USE_REGISTRY", "线程注册表", "TRUE"),
        ("CH_CFG_OPTIMIZE_SPEED", "内联优化", "TRUE"),
        ("CH_DBG_STATISTICS", "线程统计", "条件化（见第 10 课）"),
    ]

    print("    %-34s %-22s %-10s %s"
          % ("宏", "含义", "chconf.h", "值"))
    print("    " + "-" * 78)

    on = off = 0
    lines = read_lines(F_CHCONF)
    for macro, desc, _note in FEATURES:
        hits = find_all(F_CHCONF, r"^\s*#define\s+%s\s" % macro)
        if not hits:
            print("    %-34s %-22s %-10s %s" % (macro, desc, "—", "（未找到）"))
            continue

        if len(hits) == 1:
            i, line = hits[0]
            parts = line.split(None, 2)
            val = parts[2].strip() if len(parts) > 2 else "?"
            print("    %-34s %-22s %-10d %s" % (macro, desc, i, val))
            if val in ("TRUE", "FALSE"):
                if val == "TRUE":
                    on += 1
                else:
                    off += 1
        else:
            # ★★ 条件化定义 —— 第 10 课踩过的坑
            print("    %-34s %-22s %-10s ★ 条件化（%d 处）"
                  % (macro, desc, str(hits[0][0]) + "...", len(hits)))
            for i, line in hits:
                print("    %-34s   %-5d %s"
                      % ("", i, line.strip()[:52]))

    print()
    print("    ⇒ 实测：TRUE %d 项 / FALSE %d 项" % (on, off))
    print()
    print("""    ⚠ 注意 CH_DBG_STATISTICS 显示为「条件化」——
      它有多个 #define，走哪条取决于 HAL_ENABLE_THREAD_STATISTICS。
      而那个宏【默认 0，300 块板卡里 0 块开启】
      ⇒ 实际恒为 FALSE。

      （这就是第 10 课 3.4 节那个坑：
       「grep 给出的是这一行长什么样，不是这一行算不算数」。）""")

    print()
    print("  ── HAL 子系统（从 halconf.h 读）──")
    print()

    f_halconf = os.path.join(COMMON, "halconf.h")
    lines = read_lines(f_halconf) or []
    hon, hoff = [], []
    for i, line in enumerate(lines, 1):
        m = re.match(r"^\s*#\s*define\s+(HAL_USE_\w+)\s+(TRUE|FALSE)", line)
        if m:
            (hon if m.group(2) == "TRUE" else hoff).append((m.group(1), i))

    print("    开着（%d 个）：" % len(hon))
    for name, i in hon:
        print("        halconf.h:%-5d %s" % (i, name))
    print()
    print("    关着（%d 个）：" % len(hoff))
    for j in range(0, len(hoff), 2):
        a = hoff[j]
        b = hoff[j + 1] if j + 1 < len(hoff) else None
        line = "        %-30s" % a[0]
        if b:
            line += "    %s" % b[0]
        print(line)

    print()
    print("""    ★ 关掉的里有两类：

        ① 主动关的（ArduPilot 不需要）
              CH_CFG_USE_CONDVARS / CH_CFG_USE_MESSAGES
              HAL_USE_GPT / HAL_USE_I2S / HAL_USE_RTC ...

        ② 用自研替代的
              HAL_USE_CAN  → bxcan.hpp + CanIface.cpp
              HAL_USE_WDG  → watchdog.c
""")
    print()


# ===========================================================================
# 实验 5：10 项绕过
# ===========================================================================

BYPASSES = [
    ("CAN 驱动", "ChibiOS 的 CAN 不支持 CAN-FD",
     [("bxcan.hpp", "HALCHIB"), ("CanIface.cpp", "HALCHIB"),
      ("CANFDIface.cpp", "HALCHIB")],
     "HAL_USE_CAN"),
    ("看门狗", "要把崩溃现场写进备份寄存器",
     [("watchdog.c", "COMMON")], "HAL_USE_WDG"),
    ("时间快路径", "每秒调用几百万次",
     [("system.cpp", "HALCHIB")], None),
    ("内存分配", "多堆路由 + canary + 强制清零",
     [("malloc.c", "COMMON")], None),
    ("异常处理", "ChibiOS 默认不使能 BusFault",
     [("system.cpp", "HALCHIB")], None),
    ("工作队列模型", "ChibiOS 没有 WorkQueue 概念",
     [("Scheduler.cpp", "HALCHIB")], None),
    ("HAL 的内核无关性", "ArduPilot 不打算内核无关",
     [("RCOutput.h", "HALCHIB")], None),
    ("mutex 内部结构", "死锁自救要读内部字段",
     [("Scheduler.cpp", "HALCHIB")], None),
    ("递归 mutex", "上游默认 FALSE",
     [], "CH_CFG_USE_MUTEXES_RECURSIVE"),
    ("内核功能裁剪", "bootloader 空间紧张",
     [], None),
]

DIRS = {"HALCHIB": HALCHIB, "COMMON": COMMON}


def exp5():
    head("实验 5：那 10 项绕过的核实")

    print("""
  第 13 课核实了这 10 项【存在】。这个实验核实它们的【证据】——
  每个文件是否真的存在、多大。
""")

    print()
    print("    %-20s %-30s %s" % ("绕过的东西", "为什么", "证据"))
    print("    " + "-" * 78)

    for name, why, files, cfg in BYPASSES:
        first = True
        if files:
            for fn, d in files:
                fp = os.path.join(DIRS[d], fn)
                exists = os.path.isfile(fp)
                size = os.path.getsize(fp) if exists else 0
                prefix = name if first else ""
                why_s = why if first else ""
                mark = ("%s (%d 字节)" % (fn, size)) if exists else ("%s (缺失)" % fn)
                print("    %-20s %-30s %s" % (prefix, why_s, mark))
                first = False
        else:
            # 只有配置项
            p = find_line(F_CHCONF, r"^\s*#define\s+%s\s" % cfg) if cfg else None
            if p:
                lines = read_lines(F_CHCONF)
                print("    %-20s %-30s chconf.h:%d  %s"
                      % (name, why, p, lines[p - 1].strip()))
            else:
                print("    %-20s %-30s %s" % (name, why, "（配置项见第 13 课）"))

        if first:
            continue
        print()

    print("  ── ★ 第 8 项（死锁自救）值得展开 ──")
    print()
    p = find_line(F_SCHED_C, r"void Scheduler::try_force_mutex")
    if p:
        show(F_SCHED_C, p - 4, p + 26, "★★ try_force_mutex")

    print("""
    ★★ 这 4 步里，每一步都踩在一个"ChibiOS 没提供"的东西上：

        步骤                        需要什么                    ChibiOS 有吗
        ─────────────────────────────────────────────────────────────────
        ① 看主线程是否在等锁        查一个线程的【状态】        ⚠ 有 API 但不够细
              main_thread->state != CH_STATE_WTMTX
        ② 找出它在等哪把锁          查一个线程【在等什么锁】     ❌ 没有
              main_thread->u.wtmtxp
        ③ 记下持有者名字            查一把锁【持有者是谁】       ❌ 没有
              wtmtx->owner->name
        ④ 强行释放                  改内核状态（强释）           ❌ 完全没有
              chMtxForceReleaseS(wtmtx)

    ⇒ 不是"一个 API 缺失"，是【四个】。

    ★ 而 ①②③ 用【读内部字段】解决了，④ 必须【改内核】——
      这就是 fork ChibiOS 的根本原因。

      第 12 课 §4.3 说的"三种绕过强度递增"在这里完整了：

          读内部字段（①②③）  →  容易，但依赖内核私有布局
          加内核函数（④）      →  难，但可以向上游反馈
""")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("启动流程", exp1),
    "2": ("★★★ 优先级编排", exp2),
    "3": ("模块生命周期", exp3),
    "4": ("ChibiOS 特性使用清单", exp4),
    "5": ("10 项绕过的核实", exp5),
}


def main():
    print()
    hr("#")
    print("#  第 20 课：ArduPilot 如何用 ChibiOS")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(APM_ROOT):
        print()
        print("  [错误] 找不到 ArduPilot：")
        print("         %s" % APM_ROOT)
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
