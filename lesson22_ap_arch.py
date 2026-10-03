#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 22 周 · 02 ArduPilot 软件架构 —— 配套实验

    实验 1  任务表调度器：AP_Scheduler::run() 到底怎么跑
    实验 2  ★★ 实时性是怎么保证的：max_time_micros、滑落、未达成、自动降速
    实验 3  ★★ AP_Param vs PX4 的参数系统
    实验 4  ★ DataFlash 日志：一个自描述格式的读写全过程

用法：
    python lesson22_ap_arch.py 1
    python lesson22_ap_arch.py 2
    python lesson22_ap_arch.py 3
    python lesson22_ap_arch.py 4
    python lesson22_ap_arch.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  ★ 四个实验都是**照着源码的语义重写一遍**，不是"打个比方"。
    任务表里的每一项（名字、频率、max_time_micros、优先级）
    **全部抄自 ArduCopter/Copter.cpp:112-...** 的真实表格。

  ★★ 本课的核心问题是：

    「PX4 用『模块 + 消息总线』，ArduPilot 用『一张任务表』。
      那这张表凭什么能保证实时性？某个任务超时了会怎么样？」

  答案在 libraries/AP_Scheduler/AP_Scheduler.cpp:182-308（run）
  和 :339-421（loop）这两段里 —— 本脚本把它们逐行翻译成 Python。

参考（路径相对 D:/DEVLib/PX4APM/APM/ardupilot-Copter-4.5.7/ardupilot-Copter-4.5.7/）：
    libraries/AP_Scheduler/AP_Scheduler.h        Task 结构 / 快速任务枚举
    libraries/AP_Scheduler/AP_Scheduler.cpp      调度主循环
    ArduCopter/Copter.cpp                         ★ 真实的 Copter 任务表
    libraries/AP_Vehicle/AP_Vehicle.cpp           公共任务表（和机型表交错）
    libraries/AP_Param/AP_Param.h                 GroupInfo / Info / ConversionInfo
    libraries/AP_Param/AP_Param.cpp               参数存取与异步保存
    libraries/AP_Logger/LogStructure.h            DataFlash 的 FMT 定义
    libraries/AP_InertialSensor/AP_InertialSensor.cpp   振动指标
    libraries/AP_InertialSensor/LogStructure.h    VIBE 消息

★ 任务表是怎么"保证实时性"的、和《FreeRTOS+ChibiOS 教程》第 20 课的关系：

  《FreeRTOS+ChibiOS 教程》第 20 课讲的是【内核侧】——
  ChibiOS 怎么给这些线程排优先级、HAL 怎么把中断和数据搬进来。
  本课讲的是【应用层】—— 这张任务表是谁写的、每一项什么意思、
  超时以后系统怎么反应。两者互补，不重复。
"""

import sys

import numpy as np

# ---------------------------------------------------------------------------
# 输出辅助（和本教程其它脚本一致）
# ---------------------------------------------------------------------------

W = 74


def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)
    print()


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (W - 4))
    print()


def tab(rows, headers, width=None):
    cols = len(headers)
    if width is None:
        width = [max(len(str(r[i])) for r in [headers] + rows) + 2
                 for i in range(cols)]
    print("  " + "".join("%-*s" % (width[i], headers[i]) for i in range(cols)))
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join("%-*s" % (width[i], str(r[i])) for i in range(cols)))
    print()


# ---------------------------------------------------------------------------
# 真实的 Copter 任务表（ArduCopter/Copter.cpp:112 起，逐行抄录，仅取主干）
# ---------------------------------------------------------------------------
#
# Task 结构（libraries/AP_Scheduler/AP_Scheduler.h:85-91）：
#     struct Task {
#         task_fn_t function;
#         const char *name;
#         float rate_hz;
#         uint16_t max_time_micros;
#         uint8_t priority;        // 0..255，【数字越小优先级越高】
#     };
#
# ★ 注意这里的 rate_hz 在 Copter.cpp 的 SCHED_TASK 宏里写的是
#   "interval_ticks" 的【字面量】，但 SCHED_TASK_CLASS 收到的其实是
#   rate_hz 字段 —— 表里写的数字就是 Hz。
#
# ★ 优先级不是连续整数，而是 3, 6, 7, 9, 12, 15, ... 一路 +3 ——
#   因为要留出空位给将来插入新任务（改一个数字比重新编号便宜得多）。

# (name, rate_hz, max_time_micros, priority, 每次执行实际耗时_us)
COPTER_TASKS = [
    # ---- 快速任务（FAST_TASK_*，优先级 0，不走频率检查）----
    ("ins.update",            0,     0,   0,   45),   # FAST_TASK_CLASS(AP_InertialSensor, ...)
    ("run_rate_controller",   0,     0,   0,  120),   # FAST_TASK
    ("motors_output",         0,     0,   0,   35),   # FAST_TASK
    ("read_AHRS",             0,     0,   0,  210),   # FAST_TASK
    ("read_inertia",          0,     0,   0,   90),   # FAST_TASK
    ("update_flight_mode",    0,     0,   0,  260),   # FAST_TASK
    # ---- 普通任务（SCHED_TASK(name, rate_hz, max_time_micros, priority)）----
    ("rc_loop",             250,   130,   3,   60),
    ("throttle_loop",        50,    75,   6,   20),
    ("fence_check",          25,   100,   7,   15),
    ("gps.update",           50,   200,   9,   80),
    ("update_batt_compass",  10,   120,  15,   40),
    ("arm_motors_check",     10,    50,  21,   10),
    ("auto_disarm_check",    10,    50,  27,   10),
    ("read_rangefinder",     20,   100,  33,   25),
    ("update_altitude",      10,   100,  42,   30),
    ("run_nav_updates",      50,   100,  45,   55),
    ("update_throttle_hover", 100,   90,  48,   35),
    ("three_hz_loop",         3,    75,  57,   20),
    ("one_hz_loop",           1,   100,  81,   45),
    ("ekf_check",            10,    75,  84,   25),
    ("check_vibration",      10,    50,  87,   15),
    ("lost_vehicle_check",   10,    50,  99,   10),
    ("gcs.update_receive",  400,   180, 102,   70),
    ("gcs.update_send",     400,   550, 105,  240),
    ("ten_hz_logging_loop",  10,   350, 114,  130),
    ("logger.periodic",     400,   300, 120,  110),
]

FAST_LOOP_HZ = 400          # ArduCopter 的主循环频率
                            # AP_Scheduler.cpp:41-45：COPTER/HELI/SUB 是 400，其余 50
MAX_FAST_TASK_PRIORITIES = 3    # AP_Scheduler.h:101


class Task:
    """对应 AP_Scheduler.h:85 的 struct Task。"""

    def __init__(self, name, rate_hz, max_time_micros, priority, actual_us):
        self.name = name
        self.rate_hz = rate_hz
        self.max_time_micros = max_time_micros
        self.priority = priority
        self.actual_us = actual_us          # 本脚本加的：模拟"实际耗时"
        self.last_run = 0                   # 对应 _last_run[i]
        self.runs = 0
        self.overruns = 0
        self.slips = 0

    @property
    def is_fast(self):
        """AP_Scheduler.cpp:222 —— priority <= MAX_FAST_TASK_PRIORITIES 就是快速任务。"""
        return self.priority <= MAX_FAST_TASK_PRIORITIES


class Scheduler:
    """对应 AP_Scheduler.cpp 的 AP_Scheduler。

    只保留和"实时性"有关的部分：
        _tick_counter / _last_run / _task_time_allowed / extra_loop_us
    """

    def __init__(self, tasks, loop_hz=FAST_LOOP_HZ, debug_level=0):
        self.tasks = tasks
        self.loop_hz = loop_hz
        self.loop_period_us = int(1000000 / loop_hz)
        self._tick_counter = 0
        self._task_time_allowed = 0
        self.extra_loop_us = 0
        self.task_not_achieved = 0
        self.task_all_achieved = 0
        self.debug_level = debug_level
        self.perf = {}                      # 每任务的执行时间统计
        self.log = []                       # (tick, 事件)

    # -- tick()  AP_Scheduler.cpp:160-164 -----------------------------------
    def tick(self):
        self._tick_counter += 1

    # -- run()   AP_Scheduler.cpp:182-308 -----------------------------------
    def run(self, time_available):
        """★ 这是本课最核心的一段。

        逐行对照 AP_Scheduler.cpp:190-298：
            for (i = 0; i < _num_tasks; i++) {
                ... 选优先级最高的那个任务 ...
                if (task.priority > MAX_FAST_TASK_PRIORITIES) {
                    dt = _tick_counter - _last_run[i];
                    interval_ticks = _loop_rate_hz / task.rate_hz;
                    if (dt < interval_ticks) continue;         // 还没到点
                    _task_time_allowed = task.max_time_micros;
                    if (dt >= interval_ticks*2) task_slipped(i);
                    if (dt >= interval_ticks*max_task_slowdown) task_not_achieved++;
                    if (_task_time_allowed > time_available) continue;  // 时间不够
                } else {
                    _task_time_allowed = get_loop_period_us();  // 快速任务：整个周期都是它的
                }
                task.function();
                _last_run[i] = _tick_counter;
                time_taken = now - _task_time_started;
                if (time_taken > _task_time_allowed) { overrun; debug(3,...); }
                perf_info.update_task_info(i, time_taken, overrun);
                if (time_taken >= time_available) time_available = 0;
                else time_available -= time_taken;
            }
        """
        # 表是按优先级排好序的（Copter.cpp:91 的注释：All entries in this
        # table must be ordered by priority.）
        ordered = sorted(self.tasks, key=lambda t: t.priority)

        for t in ordered:
            if t.is_fast:
                # ── 快速任务：不受频率限制，每轮都跑 ──
                # AP_Scheduler.cpp:251-253
                self._task_time_allowed = self.loop_period_us
            else:
                dt = self._tick_counter - t.last_run
                if t.rate_hz <= 0:
                    interval_ticks = 1
                else:
                    interval_ticks = int(self.loop_hz / t.rate_hz)
                if interval_ticks < 1:
                    interval_ticks = 1
                if dt < interval_ticks:
                    continue                    # 还没到该跑的时刻
                self._task_time_allowed = t.max_time_micros

                # ★ 滑落：晚了 2 倍以上（AP_Scheduler.cpp:236-238）
                if dt >= interval_ticks * 2:
                    t.slips += 1
                    self.log.append((self._tick_counter,
                                     "slipped: %s (dt=%d > 2*%d)"
                                     % (t.name, dt, interval_ticks)))

                # ★★ 未达成：晚了 4 倍以上（AP_Scheduler.cpp:240-244）
                if dt >= interval_ticks * 4:        # max_task_slowdown = 4
                    self.task_not_achieved += 1

                # ★ 时间不够，这一轮就跳过它（不是等它，而是【不做】）
                if self._task_time_allowed > time_available:
                    continue

            # ── 真正执行 ──
            # ★ 注意这里用的是 task.actual_us，对应真实代码里的
            #   now = AP_HAL::micros(); time_taken = now - _task_time_started;
            time_taken = t.actual_us
            t.runs += 1
            t.last_run = self._tick_counter
            self.perf.setdefault(t.name, []).append(time_taken)

            overrun = False
            if time_taken > self._task_time_allowed:
                overrun = True
                t.overruns += 1
                if self.debug_level >= 3:
                    self.log.append((self._tick_counter,
                                     "OVERRUN %s (%d/%d us)"
                                     % (t.name, time_taken,
                                        self._task_time_allowed)))

            # ★★ 预算耗尽：把 time_available 置 0，
            #    这样后面只剩快速任务还能跑（AP_Scheduler.cpp:284-297）
            if time_taken >= time_available:
                time_available = 0
            else:
                time_available -= time_taken

        return time_available

    # -- loop()  AP_Scheduler.cpp:339-421（只保留和实时性相关的部分）--------
    def loop(self, ins_wait_us=0, extra_us_override=None):
        """跑一轮主循环。返回 (time_available, 是否发生 overrun)。"""
        loop_us = self.loop_period_us
        self.tick()

        if extra_us_override is not None:
            self.extra_loop_us = extra_us_override

        now_rel = ins_wait_us                 # 相对 sample_time 已经过了多久
        time_available = 0
        if now_rel < loop_us:
            time_available = loop_us - now_rel
        time_available += self.extra_loop_us

        self.run(time_available)

        # ★★ 自动降速（AP_Scheduler.cpp:396-410）
        #    如果有任务"未达成"，就给每轮【多加 100 us 预算】，
        #    上限 5000 us —— 这等于把主循环频率【降下来】换取任务跑完。
        if self.task_not_achieved > 0:
            self.extra_loop_us = min(self.extra_loop_us + 100, 5000)
            self.task_not_achieved = 0
            self.task_all_achieved = 0
            self.log.append((self._tick_counter,
                             "加预算 -> extra_loop_us = %d" % self.extra_loop_us))
        elif self.extra_loop_us > 0:
            self.task_all_achieved += 1
            if self.task_all_achieved > 50:
                # 连续 50 轮都达标，慢慢把多给的预算收回来
                self.task_all_achieved = 0
                self.extra_loop_us = max(0, self.extra_loop_us - 50)
                self.log.append((self._tick_counter,
                                 "减预算 -> extra_loop_us = %d"
                                 % self.extra_loop_us))

        return time_available


# ---------------------------------------------------------------------------
# 实验 1：任务表调度器
# ---------------------------------------------------------------------------

def exp1():
    head("实验 1  任务表调度器 —— AP_Scheduler::run() 到底怎么跑")

    print("""
  对照：libraries/AP_Scheduler/AP_Scheduler.cpp:182-308（run）
        libraries/AP_Scheduler/AP_Scheduler.h:85-91（Task 结构）

  ★★ 先把 PX4 和 ArduPilot 的差别摆出来：

       PX4：每个模块【自己决定】什么时候跑
            （自己起线程死循环，或者挂到工作队列上定时唤醒）

       AP ：所有任务【都写在一张表里】，由一个调度器统一决定
            （表在 ArduCopter/Copter.cpp:112 起）

    ⇐ 这两种做法的差别，本课 §22.2 会讲透。先看这张表长什么样。
""")

    sub("① 真实的 Copter 任务表（节选，抄自 Copter.cpp）")

    rows = []
    for (name, rate, mx, pri, act) in COPTER_TASKS:
        if pri <= MAX_FAST_TASK_PRIORITIES:
            rows.append([pri, name, "FAST", "-", "-", "%d" % act])
        else:
            rows.append([pri, name, "%.0f" % rate, "%d" % mx,
                         "%.0f" % (1000000.0 / rate), "%d" % act])
    tab(rows, ["优先级", "任务名", "rate_hz", "max_time_us", "间隔_us", "实测_us"])

    print("""    ★ 读表的四个要点：

      ① 优先级数字【越小越优先】（Copter.cpp:101 的注释原文：
         priority (0 through 255, lower number meaning higher priority)）

      ② 优先级 0..3 是【快速任务】（FAST_TASK），rate_hz 写 0 ——
         它们不看频率，【每一轮主循环都跑】。
         看 ③ 就明白为什么：rate_controller、motors_output、
         read_AHRS 这些东西必须紧跟 IMU 的节奏，不能被降频。

      ③ 表里 priority 从 3 直接跳到 6、7、9、12、15……
         【不是连续的】—— 这是刻意留白，方便以后插入新任务。

      ④ ★★ 注意 rc_loop 的优先级是 3 —— 而快速任务的判据是
         task.priority > MAX_FAST_TASK_PRIORITIES（= 3）才【不算】快速，
         ⇒ **优先级正好等于 3 的 rc_loop，实际上被当成快速任务**，
           它每轮主循环都跑（400 Hz），rate_hz 写的 250 不起作用。
         ★ 这是读源码才能发现的东西：枚举里 MAX_FAST_TASK_PRIORITIES = 3
           的意思是"0/1/2 是快速任务档位"，但比较写的是 `>` 而不是 `>=`，
           于是 3 也落进了快速任务。具体见 AP_Scheduler.h:97-102 与
           AP_Scheduler.cpp:222，两处对照着读就明白了。

      ⑤ ★ 表的顺序【就是优先级的顺序】。Copter.cpp:91 的注释原文：
         All entries in this table must be ordered by priority.
         （本脚本的 Scheduler.run() 里仍然会 sort 一次，是为了
           防止表写错；真机上是靠代码审查 + 这条注释来保证的）

      ⑥ ★★ 最后两列是这张表的灵魂：
         max_time_us 是【作者承诺"我这个任务最多花这么久"】，
         interval_us 是【它应该多久跑一次】。
         这两个数就是"实时性保证"的全部依据 —— 实验 2 详讲。
""")

    sub("② 跑 20 轮主循环，看每个任务实际跑了几次")

    tasks = [Task(*t) for t in COPTER_TASKS]
    s = Scheduler(tasks, loop_hz=FAST_LOOP_HZ)

    n_loops = 20
    for _ in range(n_loops):
        s.loop(ins_wait_us=100)     # 假装等 INS 样本花了 100 us

    t_total_ms = n_loops * 1000.0 / FAST_LOOP_HZ
    rows = []
    for t in sorted(tasks, key=lambda x: x.priority):
        expect = (n_loops if t.is_fast
                  else n_loops * t.rate_hz / FAST_LOOP_HZ)
        rows.append([t.priority, t.name,
                     "%d" % t.runs,
                     "%.1f" % expect,
                     "是" if abs(t.runs - expect) < 1.0 else "否（见下）"])
    print("    模拟了 %.1f ms（%d 轮 × %d Hz 主循环）" % (t_total_ms, n_loops,
                                                        FAST_LOOP_HZ))
    print()
    tab(rows, ["优先级", "任务名", "实跑次数", "理论次数", "符合"])

    print("""    ★ 不吻合的那些【正是实时性问题的现场】——
      它们在 20 轮里被执行的时间预算挤掉了（见 ③）。

    ★★ 换个角度看同一件事：把"执行时间"加起来，
      看看主循环的预算到底被谁吃掉了。
""")

    used = {}
    for t in tasks:
        used[t.name] = sum(s.perf.get(t.name, [0]))
    total_used = sum(used.values())
    budget = n_loops * 1000.0 / FAST_LOOP_HZ * 1000.0      # us
    top = sorted(used.items(), key=lambda kv: -kv[1])[:8]
    rows = []
    for name, us in top:
        rows.append([name, "%d" % us, "%.1f%%" % (100.0 * us / budget)])
    tab(rows, ["任务", "累计耗时_us", "占主循环预算"])

    print("""    主循环总预算 = %d 轮 × %d us = %.0f us
    任务实际用掉 = %.0f us（%.1f%%）
    ⇒ ★ 剩下的就是"空闲时间"。
      ★★ 这个百分比就是 ArduPilot 里 load_average() 想告诉你的东西
         （AP_Scheduler.cpp:325-337，它把"主循环实际频率比设定值低 5%%"
           直接判成 load = 1.0）。""" % (n_loops, s.loop_period_us, budget,
                                          total_used,
                                          100.0 * total_used / budget))

    if s.log:
        sub("③ 调度器自己记录的事件")
        for (tick, msg) in s.log[:20]:
            print("      tick %-4d  %s" % (tick, msg))
        if len(s.log) > 20:
            print("      ...（共 %d 条）" % len(s.log))
    else:
        sub("③ 调度器自己记录的事件")
        print("      无（这一轮所有任务都按时跑完了）")


# ---------------------------------------------------------------------------
# 实验 2：实时性是怎么保证的
# ---------------------------------------------------------------------------

def exp2():
    head("实验 2  ★★ 实时性是怎么保证的 —— max_time_micros 的作用")

    print("""
  对照：libraries/AP_Scheduler/AP_Scheduler.cpp:222-308（预算与超时）
        libraries/AP_Scheduler/AP_Scheduler.cpp:396-410（自动降速）
        libraries/AP_Scheduler/AP_Scheduler.h:250-252（max_task_slowdown = 4）

  ★★ 先回答一个最容易被忽略的问题：

      「ArduPilot 没有抢占。一个任务跑的时候，别的任务【就是不能跑】。
        那它凭什么能保证实时性？」

    答案：**它不保证"每个任务都能按时跑"，它保证"系统知道
    自己没做到，并且会做出反应"。**

    具体有三层机制，本实验逐层验证。
""")

    sub("① 第一层：max_time_micros —— 一个「承诺」，不是一个「限制」")

    print("""    ★ 这里有个【非常容易误解】的地方：

      max_time_micros 并【不会】打断任务。C++ 里没有"时间到了
      就把这个函数掐掉"这种机制（那要靠抢占式内核或异常）。

      它的真实作用是【两个】：

        ① 当预算用：_task_time_allowed = task.max_time_micros;
           如果 _task_time_allowed > time_available（这轮剩下的时间），
           这个任务【这一轮就干脆不跑了】。（AP_Scheduler.cpp:246-250）

        ② 当标尺用：跑完之后比较 time_taken > _task_time_allowed，
           超过就标记 overrun 并打印（AP_Scheduler.cpp:272-280）：
               debug(3, "Scheduler overrun task[%u-%s] (%u/%u)\\n", ...)

    ★★ 所以 max_time_micros 真正的价值是【暴露问题】：
      它是开发者写下的"我认为这个任务不会超过这么久"这个断言。
      一旦实际超了，日志里就会出现 OVERRUN ——
      这是**设计意图和实际行为不一致**的明确信号。
""")

    sub("② 演示：给一个任务「加负担」，看调度器怎么反应")

    # 让 gcs.update_send（max 550us）变慢到 900us，越过了它自己的承诺
    tasks = [Task(*t) for t in COPTER_TASKS]
    for t in tasks:
        if t.name == "gcs.update_send":
            t.actual_us = 900       # 从 240 变成 900
    s = Scheduler(tasks, loop_hz=FAST_LOOP_HZ, debug_level=3)

    rows = []
    for n in range(1, 31):
        s.loop(ins_wait_us=100)
        if n in (1, 5, 10, 15, 20, 25, 30):
            gcs = [t for t in tasks if t.name == "gcs.update_send"][0]
            rows.append([n, "%d" % s.extra_loop_us,
                         "%.0f" % (1000000.0 / (s.loop_period_us
                                                + s.extra_loop_us)),
                         "%d" % gcs.overruns])
    tab(rows, ["轮次", "extra_loop_us", "等效主循环频率", "累计 overrun 次数"])

    print("""    ★★ 看这张表：
      · gcs.update_send 的 overrun 次数在涨 —— 问题被【记录下来了】
      · extra_loop_us 却还是 0 —— 因为 "未达成" 的判据是【别的】

    ★ 这里必须把三个概念分清楚（很多人混为一谈）：
""")

    rows = [
        ["slipped\n(滑落)", "dt >= interval_ticks * 2",
         "perf_info.task_slipped(i)\n(AP_Scheduler.cpp:236-238)",
         "只是统计，提醒你\n『它晚了一倍』"],
        ["task_not_achieved\n(未达成)", "dt >= interval_ticks * max_task_slowdown\n= interval_ticks * 4",
         "task_not_achieved++\n(AP_Scheduler.cpp:240-244)",
         "★ 触发【自动降速】\n(见 ③)"],
        ["overrun\n(超时)", "time_taken > _task_time_allowed",
         "debug(3, ...) + perf_info\n(AP_Scheduler.cpp:272-282)",
         "★ 只打日志\n说明「承诺」没兑现"],
    ]
    tab(rows, ["名字", "触发条件", "代码动作", "后果"])

    print("""    ★★★ 注意这三条的【判据完全不同】：

        · slipped / task_not_achieved 看的是【时间间隔】（多久没跑）
        · overrun 看的是【执行时长】（跑了多久）

      ⇒ 一个任务可以"跑得很频繁但每次都超时"（overrun 多，不降速），
        也可以"很久没跑但跑一次很快"（slipped 多，超 4 倍就降速）。
        **它们是两个正交的维度，不能互相替代。**
""")

    sub("③ 第三层：自动降速 —— 唯一「会动」的那一层")

    print("""    ★★ 现在造一个真正的"未达成"：

      主循环 400 Hz（周期 2500 us），让 one_hz_loop 之外的
      一个 50 Hz 任务占用大量时间，把它拖到 4 倍周期以外。
""")

    tasks = [Task(*t) for t in COPTER_TASKS]
    # 让 rate_controller 这个快速任务变得很重：
    # 它是快速任务，每轮都跑，会直接吃掉整个主循环预算
    for t in tasks:
        if t.name == "run_rate_controller":
            t.actual_us = 900        # 从 120 加到 900
        if t.name == "read_inertia":
            t.actual_us = 500

    s = Scheduler(tasks, loop_hz=FAST_LOOP_HZ, debug_level=3)

    rows = []
    for n in range(1, 61):
        s.loop(ins_wait_us=100)
        if n % 5 == 0 or n <= 3:
            eff = 1000000.0 / (s.loop_period_us + s.extra_loop_us)
            rows.append([n, "%d" % s.extra_loop_us, "%.1f" % eff])
    tab(rows, ["轮次", "extra_loop_us", "等效主循环频率 Hz"])

    print("""    ★★★ 这就是 ArduPilot 实时性保证的【核心机制】，请务必看懂：

      主循环的频率是【可变的】！
      它有一个设定的目标频率（400 Hz），但如果发现有任务
      晚到 4 倍周期还没跑，它就【给每轮多批 100 us 预算】，
      相当于主动把主循环频率降下来。

          extra_loop_us 上限 5000 us
          ⇒ 最慢会降到 1000000/(2500+5000) ≈ 133 Hz

      当连续 50 轮都达标了，再每轮收回 50 us，慢慢恢复。

    ★★ 这个设计的哲学，一句话：

        **』宁可整体跑慢一点，也不要某个任务被饿死。』**

      提高主循环频率听起来很美，但如果高频下任务老是跑不完，
      那"400 Hz"就是个【假数字】—— 实际的数据更新率根本达不到。
      ArduPilot 的选择是：**承认现实，把频率降到能跑完为止。**
""")

    sub("④ 那…为什么不在任务表里直接写一个更低的频率？")

    print("""    ★ 好问题。答案是：**设定频率是"设计值"，降速是"运行时的适应"。**

      设定值 400 Hz 是【在地面调参时定的】——
      设计师按"正常负载"选的。但飞行中负载会变：
        · 加了新传感器（比如避障雷达）
        · 日志文件变大，写 SD 卡变慢
        · 某些航段 EKF 要算更多东西

      这些情况在【编译期】是不知道的。所以需要运行时的自适应。

    ★ 更重要的是：**降速这件事本身会出现在日志里。**
      地面分析时看到 extra_loop_us 长期很大，
      就知道"这架飞机的 CPU 不够用了"，该优化了。
      （这就是本教程第 24 课要讲的"从日志看问题"的一部分。）
""")


# ---------------------------------------------------------------------------
# 实验 3：AP_Param vs PX4 的参数系统
# ---------------------------------------------------------------------------

def exp3():
    head("实验 3  ★★ AP_Param vs PX4 的参数系统")

    print("""
  对照：libraries/AP_Param/AP_Param.h:197-228（三个结构体）
        libraries/AP_Param/AP_Param.cpp:1250-1278（save 的队列与丢弃）
        Tools/autotest/param_metadata/param_parse.py:46（@Param 正则）

  ★★ 两个飞控的参数系统，长得【完全不一样】：

      PX4    ：参数是【独立的一份数据】，
               通过 topic 广播"参数变了"（parameter_update），
               任何模块都可以订阅

      AP     ：参数就是【类里的成员变量】，
               靠一张 var_info 表把"名字"和"成员变量的内存偏移"
               对应起来 —— 名字只在存取时用，飞行时用的是变量本身
""")

    sub("① 核心结构：GroupInfo / Info —— 一张「名字 ↔ 内存偏移」的表")

    print("""    源码原文（libraries/AP_Param/AP_Param.h:197-222，节选）：

        struct GroupInfo {
            const char *name;
            ptrdiff_t offset;      // ★★ 关键：在对象内的【字节偏移】
            union {
                const struct GroupInfo *group_info;
                const float def_value;
                ...
            };
            uint16_t flags;
            uint8_t idx;           // 组内编号
            uint8_t type;          // AP_PARAM_*
        };
        struct Info {
            const char *name;
            const void *ptr;       // ★★ 关键：指向变量的【绝对地址】
            union { ... };
            uint16_t flags;
            uint16_t key;          // k_param_*
            uint8_t type;
        };

    ★ 注意 GroupInfo 用的是 **offset（偏移）** 而不是指针。
      为什么？因为参数表是【静态常量】，写在 flash 里；
      而对象是在启动时构造的 —— 编译期不知道对象地址，
      只知道"这个参数在对象里偏移多少字节"。

      ⇒ 一个参数在内存里的地址 =
            对象基址 + GroupInfo.offset + 可能的嵌套偏移

    ★ 那嵌套怎么办（一个 AP_XXX 对象里又有多个子对象）？
      靠 AP_SUBGROUPINFO / AP_NESTEDGROUPINFO 这些宏
      （AP_Param.h:146-159）一层层往下走，累积 offset。
""")

    sub("② 用 Python 把这张表「重演」一遍：var_info 树查找")

    # 模拟 AP_Param 的树形结构
    class Param:
        def __init__(self, name, value, ptype="float", idx=0):
            self.name, self.value, self.ptype, self.idx = name, value, ptype, idx

    class Group:
        """对应一个 var_info[] 表 + 一个对象基址。"""

        def __init__(self, prefix, params, subgroups=None):
            self.prefix = prefix
            self.params = params                  # {idx: Param}
            self.subgroups = subgroups or {}      # {idx: (name, Group)}

    # 复刻 Copter 的参数树（名字都是真实的参数名）
    copter = Group("", {}, {
        0: ("Scheduler", Group("SCHED_", {
            1: Param("LOOP_RATE", 400, "int16"),          # AP_Scheduler.cpp:65
        })),
        1: ("INS", Group("INS_", {
            0: Param("GYRO_FILTER", 20, "int16"),
            1: Param("ACCEL_FILTER", 20, "int16"),
        })),
        2: ("ATC", Group("ATC_", {
            0: Param("RAT_RLL_P", 0.135, "float"),
            1: Param("RAT_RLL_I", 0.135, "float"),
            2: Param("RAT_RLL_D", 0.0036, "float"),
        }, {
            3: ("ANG", Group("ATC_ANG_", {
                0: Param("RLL_P", 4.5, "float"),
            })),
        })),
        3: ("BATT", Group("BATT_", {
            0: Param("CAPACITY", 3300, "int16"),
        })),
    })

    def flatten(prefix, group, out, depth=0):
        for idx, p in group.params.items():
            out.append((depth, prefix + p.name, p.ptype, p.value,
                        "GroupInfo(offset=...)"))
        for idx, (name, sub) in group.subgroups.items():
            flatten(prefix + name + "_", sub, out, depth + 1)

    out = []
    flatten("", copter, out)
    rows = [[("  " * d) + nm, ty, "%s" % v, how]
            for (d, nm, ty, v, how) in out]
    tab(rows, ["参数全名（用户看到的）", "类型", "默认值", "表项形式"])

    print("""    ★★ 读这张表要抓住两点：

      ① **参数名是拼出来的**：
             _ + ATC + _ + ANG + _ + RLL_P  =  ATC_ANG_RLL_P
         每一层子对象贡献一段前缀。
         ⇒ 这就是为什么 ArduPilot 的参数名都是
           「模块_功能_轴_量」这种格式（BATT_CAPACITY、
           ATC_RAT_RLL_P、SCHED_LOOP_RATE）。

      ② **查找是"顺着树往下走"的**，不是哈希表。
         用户敲 ATC_ANG_RLL_P 时，AP_Param 逐个匹配前缀、
         逐层深入，最后拿到一个 (对象指针, 累计偏移)。
         ⇒ 慢，但只在【地面站读写参数】时发生，
           飞行时用的是已经解析好的变量地址。

    ★ 对比 PX4：
        PX4 的参数是**平铺的**（param name → 一份独立存储），
        模块通过 `ParamFloat` 句柄访问。
        名字就是名字，没有树。
""")

    sub("③ 文档生成：@Param 注释 → HTML/XML/RST")

    print("""    ★ ArduPilot 的参数文档【不是手写的】，是从源码里抽出来的。

      源码里的写法（libraries/AP_Scheduler/AP_Scheduler.cpp:52-67）：

          // @Param: LOOP_RATE
          // @DisplayName: Scheduler loop rate
          // @Description: Scheduler loop rate in Hz. ...
          // @Range: 50 2000
          // @Increment: 1
          // @User: Advanced
          AP_GROUPINFO("LOOP_RATE",  1, AP_Scheduler, _loop_rate_hz, SCHEDULER_DEFAULT_LOOP_RATE),

      ★ 抽注释的正则（Tools/autotest/param_metadata/param_parse.py:46）：

          prog_param = re.compile(
              r"@Param(?:{([^}]+)})?: (\\w+).*((?:\\n[ \\t]*// @(\\w+)"
              r"(?:{([^}]+)})?: ?(.*))+)(?:\\n[ \\t\\r]*\\n|\\n[ \\t]+[A-Z]|\\n\\-\\-\\]\\])",
              re.MULTILINE)

      ⇒ 然后 Tools/autotest/param_metadata/ 下的一堆 emit 脚本
        生成 HTML / XML / JSON / RST / Markdown 各种格式。

    ★★ 这个做法的好处：

        · 参数和它的说明【在同一个文件里】—— 改代码时顺手改说明
        · 文档【不可能过时】—— 它是从代码生成的
        · 还能做【一致性检查】：@Range 写得不对会报错

      ★ 代价：注释的格式【必须严格】。
        少一个 `//` 或者 @Range 写错，生成就漏掉这个参数，
        而且不会有编译错误。所以 ArduPilot 有专门的 CI 检查这个。
""")

    sub("④ ★★ 异步保存：save() 为什么可能「什么都没发生」")

    print("""    ★ PX4 写参数是【同步】的（写完才返回，慢但确定）。
      ArduPilot 是【异步】的 —— 因为这个区别：

        参数存在 EEPROM / FRAM 里，写一次要【毫秒级】。
        而调参时地面站会【连续改几十个参数】。
        如果每次都同步写，主循环会被卡住几十毫秒 ⇒ 飞机抖一下。

    ★★ ArduPilot 的做法（AP_Param.cpp:1250-1278）：

        void AP_Param::save(bool force_save)
        {
            struct param_save p, p2;
            p.param = this;
            p.force_save = force_save;

            // ① 去重：同一个参数已经在队头等着了，就不重复入队
            if (save_queue.peek(p2) && p2.param == this
                && p2.force_save == force_save) {
                return;
            }

            // ② 入队；队满时的处理是【关键】
            while (!save_queue.push(p)) {
                if (hal.util->get_soft_armed()
                    && hal.scheduler->in_main_thread()) {
                    // ★★★ 解锁状态下、在主线程里 ⇒ 直接【丢掉这次保存】
                    return;
                }
                // 否则（没解锁，或者不在主线程）就等着
                hal.scheduler->delay_microseconds(500);
            }
        }

    ★★★ 注意那个 if 的判断条件，它是【两个条件的与】：

        soft_armed  &&  in_main_thread

        · 解锁了但不在主线程（比如在 IO 线程）→ 可以等
        · 没解锁（地面上）→ 可以等，改参数本来就在地面
        · 【解锁了 且 在主线程】→ 绝不能等！丢掉这次保存。

    ★ 队列深度是 30（AP_Param.cpp:118）：

        ObjectBuffer_TS<AP_Param::param_save> AP_Param::save_queue{30};

    ★ 实际的写盘发生在【IO 线程】（AP_Param.cpp:1283-1293）：

        void AP_Param::save_io_handler(void)
        {
            struct param_save p;
            while (save_queue.pop(p)) {
                p.param->save_sync(p.force_save, true);
            }
            ...
        }

    ⇒ ★★ 一句话总结这个设计：

        **』飞行安全 > 参数一定被保存。』**

      解锁状态下，宁可丢掉一次参数保存，也不能让主线程等 500 微秒
      的循环 —— 因为主线程就是控制回路。
      而丢掉的那次保存，地面站会收到参数值回读不一致，
      用户在地面上重试一次就行了。
""")

    sub("⑤ 三种「参数变了」的通知方式对比")

    rows = [
        ["PX4", "发布 parameter_update\ntopic", "所有模块都能订阅\n（广播）",
         "松耦合，但多一次\ntopic 拷贝"],
        ["AP_Param", "在 save() 里直接\n调用回调/通知 GCS", "点对点",
         "快，但新增消费者要\n改 AP_Param"],
        ["AP_Param\n(改参数时)", "参数变量【就地】改掉\n（它本来就是成员变量）",
         "不需要通知 ——\n下次读就是新值",
         "★ 最快，但无法\n「订阅变更事件」"],
    ]
    tab(rows, ["系统", "机制", "广播还是点对点", "代价"])

    print("""    ★★ 最后这条最值得玩味：

      PX4 里"参数变了"是一个【事件】，要广播出去；
      ArduPilot 里参数就是【一个全局变量】，
      改了它下一次读自然就是新值，**根本不需要通知**。

      ⇒ 这正是两套架构哲学差异的一个缩影：

          PX4    ：一切皆消息，解耦优先
          AP     ：一切皆全局状态，简单优先

      ★ 没有绝对的好坏 —— 看你更怕"耦合"还是更怕"意外"。
""")


# ---------------------------------------------------------------------------
# 实验 4：DataFlash 日志
# ---------------------------------------------------------------------------

def exp4():
    head("实验 4  ★ DataFlash 日志 —— 一个自描述格式的读写全过程")

    print("""
  对照：libraries/AP_Logger/LogStructure.h:115-122（包头与魔数）
        libraries/AP_Logger/LogStructure.h:176-183（log_Format）
        libraries/AP_Logger/LogStructure.h:1216-1217（FMT 自己的定义）
        libraries/AP_Logger/LogStructure.h:1394（LOG_FORMAT_MSG = 128）

  ★★ 日志格式的核心难题：

      飞机写日志的时候，格式可能随时在变
      （固件升级、参数不同、不同机型）。
      **读日志的时候怎么知道某个字节是什么意思？**

    ArduPilot 的答案：**把格式也写进日志里。**
      日志的第一批消息全是"格式定义"（FMT），
      之后的数据消息只要带上"我是第几号格式"，读者就能自己解析。

    ⇒ 这叫【自描述格式】（self-describing）。本实验把它完整走一遍。
""")

    sub("① 包结构：3 字节包头 + 数据")

    print("""    源码原文（libraries/AP_Logger/LogStructure.h:115-122）：

        #define LOG_PACKET_HEADER        uint8_t head1, head2, msgid;
        #define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id
        #define LOG_PACKET_HEADER_LEN 3   // bytes required for LOG_PACKET_HEADER

        #define HEAD_BYTE1  0xA3    // Decimal 163
        #define HEAD_BYTE2  0x95    // Decimal 149

    ⇒ 每个消息都是：

        ┌────────┬────────┬────────┬──────────────────────┐
        │ 0xA3   │ 0x95   │ msgid  │  数据（长度由 FMT 定） │
        └────────┴────────┴────────┴──────────────────────┘
            1 字节   1 字节   1 字节

    ★ 为什么要两个固定的魔数（0xA3 0x95）？
      因为从 SD 卡里读出来的可能是【乱码】——
      文件损坏、写到一半断电、或者干脆读错了文件。
      有了魔数就能【扫描式地重新同步】：
      一个字节一个字节找 0xA3 0x95，找到了就当它是一个新包的开头。

    ★ 为什么 msgid 只有 1 字节（最多 255 种消息）？
      因为要省空间。400 Hz 的日志每秒写几万个包，
      每多一个字节都是 SD 卡的负担。
      255 种消息对飞控来说够用了（实测 Copter 大约 100 多种）。
""")

    sub("② FMT 消息自己的定义（第 128 号）")

    print("""    源码原文（libraries/AP_Logger/LogStructure.h:1216-1217）：

        { LOG_FORMAT_MSG, sizeof(log_Format), \\
          "FMT", "BBnNZ", "Type,Length,Name,Format,Columns", "-b---", "-----" },

    以及（libraries/AP_Logger/LogStructure.h:1394）：

        LOG_FORMAT_MSG = 128, // this must remain #128

    而 log_Format 结构是（libraries/AP_Logger/LogStructure.h:176-183）：

        struct PACKED log_Format {
            LOG_PACKET_HEADER;      // 0xA3 0x95 128
            uint8_t type;           // ★ 这条 FMT 描述的是哪一号消息
            uint8_t length;         // ★ 那条消息【总共】多少字节
            char name[4];           // ★ 4 字符的名字（不含结尾 0！）
            char format[16];        // ★ 字段类型串，如 "QBfffI"
            char labels[64];        // ★ 逗号分隔的字段名
        };

    ★★ 注意这里有一个"自举"（bootstrap）的妙处：

        FMT 消息【自己也用 FMT 描述自己】，而且编号固定为 128。
        ⇒ 读者只要会解析 128 号，就能学会解析所有其它号。

    ★★ format 串的字符含义（ArduPilot 约定）：
        B = uint8     b = int8
        H = uint16    h = int16
        I = uint32    i = int32
        Q = uint64    q = int64
        f = float     d = double
        n = char[4]   N = char[16]   Z = char[64]
        c = int16 × 100（定点小数）  C = uint16 × 100
        e = int32 × 100               E = uint32 × 100
        L = int32 经纬度 × 1e7        M = uint8 模式
""")

    sub("③ 用 Python 造一条完整的日志：FMT + VIBE + ATT + PSCN")

    HEAD1, HEAD2, LOG_FORMAT_MSG = 0xA3, 0x95, 128

    # 类型串 -> (numpy dtype, 字节数)
    TYPE_MAP = {
        "B": ("u1", 1), "b": ("i1", 1),
        "H": ("<u2", 2), "h": ("<i2", 2),
        "I": ("<u4", 4), "i": ("<i4", 4),
        "Q": ("<u8", 8), "q": ("<i8", 8),
        "f": ("<f4", 4), "d": ("<f8", 8),
        "n": ("S4", 4), "N": ("S16", 16), "Z": ("S64", 64),
        # 定点小数：ArduPilot 为了省空间，常把小数乘 100 后存成 int
        "c": ("<i2", 2), "C": ("<u2", 2),     # int16/uint16 x 100
        "e": ("<i4", 4), "E": ("<u4", 4),     # int32/uint32 x 100
        "L": ("<i4", 4), "M": ("u1", 1),      # 经纬度 x 1e7 / 模式
    }

    def fmt_msg(msgid, name, fmt, labels):
        """构造一条 FMT 消息（对应 log_Format）。"""
        payload = bytearray()
        payload.append(msgid)                      # type
        size = 3 + sum(TYPE_MAP[c][1] for c in fmt)
        payload.append(size)                       # length（含 3 字节包头）
        payload += name.encode("ascii").ljust(4, b"\x00")[:4]
        payload += fmt.encode("ascii").ljust(16, b"\x00")[:16]
        payload += labels.encode("ascii").ljust(64, b"\x00")[:64]
        return bytes([HEAD1, HEAD2, LOG_FORMAT_MSG]) + bytes(payload)

    # ★ 定点小数的缩放因子。ArduPilot 为了省空间，把小数乘 100 存成 int16。
    #   c/C/e/E 都是 x100，L（经纬度）是 x1e7。
    SCALE = {"c": 100.0, "C": 100.0, "e": 100.0, "E": 100.0, "L": 1.0e7}

    def data_msg(msgid, fmt, values):
        """构造一条数据消息。

        ★ 关键：定点小数类型必须【先乘缩放因子再取整】，否则
          1) 精度全丢（12.34 存成 12）；
          2) 负数塞进无符号字段会直接报 OverflowError。
        """
        buf = bytearray([HEAD1, HEAD2, msgid])
        for c, v in zip(fmt, values):
            dt, _ = TYPE_MAP[c]
            if dt.startswith("S"):
                buf += str(v).encode("ascii").ljust(int(dt[1:]), b"\x00")
            elif c in SCALE:
                buf += np.array([int(round(v * SCALE[c]))], dtype=dt).tobytes()
            else:
                buf += np.array([v], dtype=dt).tobytes()
        return bytes(buf)

    def parse_fmt(pkt):
        """解析一条 FMT 消息。"""
        assert pkt[0] == HEAD1 and pkt[1] == HEAD2 and pkt[2] == LOG_FORMAT_MSG
        body = pkt[3:]
        msgid, length = body[0], body[1]
        name = body[2:6].split(b"\x00")[0].decode("ascii")
        fmt = body[6:22].split(b"\x00")[0].decode("ascii")
        labels = body[22:86].split(b"\x00")[0].decode("ascii")
        return msgid, length, name, fmt, labels

    # ---- 建表：真实消息（VIBE 来自 AP_InertialSensor/LogStructure.h:122）----
    FMT_TABLE = [
        # (msgid, name, format, labels)   ← 三条的 name/format/labels 全部抄自源码
        #
        # 注意：msgid 是本脚本随便给的编号（真机上是 LOG_xxx_MSG 枚举的值），
        # 但 name / format / labels 一个字都不能改 —— 改了日志就解析不出来。
        (129, "VIBE", "QBfffI",
         "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip"),      # AP_InertialSensor/LogStructure.h:122
        (130, "ATT",  "QccccCCCCB",
         "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw,AEKF"),
                                                    # AP_AHRS/LogStructure.h:202
        (131, "PSCN", "Qffffffff",
         "TimeUS,TPN,PN,DVN,TVN,VN,DAN,TAN,AN"),     # AP_Logger/LogStructure.h:1325
    ]

    log = bytearray()
    # ① 先写 FMT 自己的定义（编号 128）
    log += fmt_msg(128, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")
    # ② 再写所有消息的格式定义
    for (mid, name, fmt, labels) in FMT_TABLE:
        log += fmt_msg(mid, name, fmt, labels)

    fmt_bytes = len(log)
    # ③ 最后写数据
    t0 = 1_000_000
    for k in range(3):
        log += data_msg(129, "QBfffI",
                        [t0 + k * 100000, 1, 3.2 + k * 0.4, 2.8, 4.1, 0])
    # ATT 的字段顺序（照抄 labels）：
    #   TimeUS, DesRoll, Roll, DesPitch, Pitch, DesYaw, Yaw, ErrRP, ErrYaw, AEKF
    # 注意 DesRoll/Roll/DesPitch/Pitch 是 c（有符号 x100，因为可以是负的），
    # 而 DesYaw/Yaw/ErrRP/ErrYaw 是 C（无符号 x100，偏航角恒 >= 0）。
    for k in range(3):
        des_roll, roll = 12.0, 10.5 + k * 0.8
        des_pitch, pitch = -3.0, -2.4 + k * 0.2
        log += data_msg(130, "QccccCCCCB",
                        [t0 + k * 100000,
                         des_roll, roll,              # DesRoll, Roll
                         des_pitch, pitch,            # DesPitch, Pitch
                         91.0, 90.2,                  # DesYaw, Yaw
                         abs(des_roll - roll),        # ErrRP：姿态跟踪误差
                         0.8,                         # ErrYaw
                         1])                          # AEKF：用的哪个 EKF
    for k in range(2):
        log += data_msg(131, "Qffffffff",
                        [t0 + k * 100000, 5.0, 5.1, 0.3, 0.2, 0.1, 0.05, 0.02, 0.01])

    print("    造出来的日志共 %d 字节：其中前 %d 字节是格式定义，"
          "后面 %d 字节是数据。" % (len(log), fmt_bytes, len(log) - fmt_bytes))
    print()
    print("    前 %d 字节的十六进制（可以看到包头 A3 95 80）：" % 12)
    print("      " + " ".join("%02X" % b for b in log[:12]))
    print()
    print("    ★ 80 是十六进制 —— 128 号消息（FMT）自己的定义。")

    sub("④ 当「读者」：只凭日志本身解析出所有数据")

    # ---- 扫码：先建立 FMT 表，再解析数据 ----
    table = {}
    pos = 0
    rows = []
    while pos < len(log):
        if log[pos] != HEAD1 or log[pos + 1] != HEAD2:
            pos += 1                       # 不同步就往后挪一字节（重新同步）
            continue
        msgid = log[pos + 2]
        if msgid == LOG_FORMAT_MSG:
            (mid, length, name, fmt, labels) = parse_fmt(log[pos:pos + 89])
            table[mid] = (length, name, fmt, labels)
            pos += 89                      # 3 + 2 + 4 + 16 + 64 = 89
            continue
        if msgid not in table:
            print("      ! 遇到未知的 msgid=%d，跳过" % msgid)
            pos += 1
            continue
        (length, name, fmt, labels) = table[msgid]
        body = log[pos + 3:pos + length]
        off = 0
        vals = []
        for c, lab in zip(fmt, labels.split(",")):
            dt, sz = TYPE_MAP[c]
            chunk = body[off:off + sz]
            off += sz
            if dt.startswith("S"):
                vals.append(chunk.split(b"\x00")[0].decode("ascii"))
            elif c in ("c", "C", "e", "E"):
                vals.append("%.2f" % (np.frombuffer(chunk, dt)[0] / 100.0))
            elif c == "L":
                vals.append("%.6f" % (np.frombuffer(chunk, "<i4")[0] / 1.0e7))
            else:
                vals.append("%s" % np.frombuffer(chunk, dt)[0])
        rows.append([name] + vals)
        pos += length

    # 只打印 VIBE 和 ATT，并把 TimeUS 换算成 ms
    sub("⑤ 解析结果（完全靠日志自己带的格式信息）")

    vibe_rows = [[r[0], "%.1f" % (int(r[1]) / 1000.0), r[2], r[3], r[4], r[5]]
                 for r in rows if r[0] == "VIBE"]
    tab(vibe_rows, ["消息", "Time(ms)", "IMU", "VibeX", "VibeY", "VibeZ"])

    att_rows = [[r[0], "%.1f" % (int(r[1]) / 1000.0),
                 r[2], r[3],      # DesRoll, Roll
                 r[4], r[5],      # DesPitch, Pitch
                 r[8]]            # ErrRP
                for r in rows if r[0] == "ATT"]
    tab(att_rows, ["消息", "Time(ms)", "DesRoll", "Roll", "DesPitch", "Pitch",
                   "ErrRP"])

    print("""    ★★ 请注意：解析器【事先不知道】VIBE 有几个字段、每个什么类型。
      它是在扫到 FMT 消息时才【现场学会】的。

      这就是自描述格式的威力：

        · 固件升了级、加了新消息，老工具照样能读（不认识的就跳过）
        · 同一次飞行里不同阶段可以有不同的消息集
        · 甚至可以把【参数值】也当成一种 FMT 消息写进去

    ★★ 对比 PX4 的 ULog（第 24 课详讲）：
      ULog 也是自描述的，但它用的是 **消息定义 + 订阅列表**，
      而且有更强的类型系统。两者的目标一样，
      实现上 ULog 更适合"按 topic 回放"，DataFlash 更适合"顺序扫描"。
""")

    sub("⑥ ★ 从日志里能看出什么：振动与姿态跟踪")

    # 造一段"有问题"的数据来看
    print("""    现在把 VIBE 里的 VibeX 拿出来，接上"真实感"：
      正常的 VibeX 应该在 5 以下（m/s² 的均方根），
      超过 30 就是【严重振动】，是炸机的头号原因。

      上面造的 3 条 VIBE 是 3.2 / 3.6 / 4.0 —— 正常。
      下面看看"振动偏高"是什么样子：

    VibeX  vs  VibeZ：
        3.2  2.8      ← 正常
       12.5  8.1      ← 偏高，图像会果冻
       35.0  22.0     ← ★★ 危险：EKF 会被污染，姿态开始漂
       68.0  41.0     ← ★★ 必炸：加速度计已经饱和/限幅

    ★★ 振动的计算方式（两个飞控不一样，第 24 课会把两个都写出来）：

      ArduPilot（libraries/AP_InertialSensor/AP_InertialSensor.cpp:2119-2143）：

          // ① 把加速度计原始数据低通到 5 Hz（"低频基线"）
          Vector3f accel_filt = _accel_vibe_floor_filter[instance].apply(accel, dt);

          // ② 跟基线做差，平方，再低通到 2 Hz
          Vector3f accel_diff = (accel - accel_filt);
          accel_diff.x *= accel_diff.x;
          ...
          _accel_vibe_filter[instance].apply(accel_diff, dt);

          // ③ 最后开方（AP_InertialSensor.cpp:2163-2173）
          vibe.x = safe_sqrt(vibe.x);

      两个截止频率在 AP_InertialSensor.h:8-9：
          AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ  5.0f
          AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ        2.0f

      ⇒ ★ 物理含义：把 5 Hz 以上的【高频成分】提取出来，
        算它的能量（平方 + 低通），最后开方还原成"加速度"的量纲。
        **这正是"振动"的定义：相对低频基线的快速抖动。**

      PX4 的做法更简单（src/modules/sensors/vehicle_imu/VehicleIMU.cpp:740-746）：

          _status.accel_vibration_metric = 0.99f * _status.accel_vibration_metric
                                         + 0.01f * Vector3f(acceleration - _acceleration_prev).norm();

      ⇒ 直接用【相邻两拍加速度之差】的模长做一阶低通。
        没有开方，所以量纲不是加速度，它是一个【相对指标】。

      ★★ 两者的阈值【不能互相换算】！调参时必须查手册，
        不能凭"PX4 是 0.5，所以 ArduPilot 也是 0.5"。
""")

    sub("⑦ 姿态跟踪误差：从日志看控制器好不好")

    # 用 numpy 造一段有"跟踪误差"的姿态数据
    np.random.seed(7)
    dt = 0.1
    tt = np.arange(0, 5.0, dt)
    des = 15.0 * np.sin(2 * np.pi * 0.5 * tt)          # 期望横滚角（度）
    # 实际：一阶跟随 + 一点延迟 + 噪声
    act = np.zeros_like(des)
    a = 0.35
    for k in range(1, len(tt)):
        act[k] = act[k - 1] + a * (des[k - 1] - act[k - 1]) + \
            np.random.randn() * 0.25
    err = act - des

    rows = [
        ["跟踪误差 RMS", "%.2f deg" % np.sqrt(np.mean(err ** 2))],
        ["跟踪误差 峰值", "%.2f deg" % np.max(np.abs(err))],
        ["相位延迟（近似）", "%.0f ms" % (1000.0 / (2 * np.pi * 0.5)
                                          * np.arctan(2 * np.pi * 0.5 * dt / a))],
        ["期望角速度峰值", "%.1f deg/s" % (np.max(np.abs(np.diff(des) / dt)))],
    ]
    tab(rows, ["指标", "值"])

    print("""    ★★ 从日志看控制器的三个标准动作：

      ① 看【跟踪误差 RMS】：期望姿态和实际姿态差多少
         · RMS < 2° 且峰值 < 5°  →  正常
         · 峰值 > 10°           →  PID 增益不够 / 有延迟 / 振动干扰

      ② 看【相位延迟】：实际比期望【晚】多少
         · 延迟主要来自：滤波器（低通）、电机响应、采样周期
         · ★★ 延迟和振动是死对头：滤得越狠延迟越大，
           延迟越大越容易振荡。调参就是在两者之间找平衡。

      ③ 看【电机输出饱和】：
         · 如果 RCOU（电机输出）在某个轴【长期贴顶】，
           说明飞行器已经【没有余量】了，再大一点的扰动就控制不住
         · 这是"炸机前最后一次机会"的信号

    ★ 这三件事都属于"从日志发现问题"，第 24 课会给出完整的检查清单。
""")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    args = sys.argv[1:]
    which = args[0] if args else "all"

    if which in ("1", "all"):
        exp1()
    if which in ("2", "all"):
        exp2()
    if which in ("3", "all"):
        exp3()
    if which in ("4", "all"):
        exp4()

    if which == "all":
        print()
        print("=" * W)
        print("  全部实验结束。")
        print("=" * W)
        print("""
  ★ 一句话回顾本课：

      PX4      ：模块 + 消息总线  → 赢在【解耦】
      ArduPilot：一张任务表       → 赢在【确定性】

      任务表里的四个数字
          （函数、频率、最大耗时、优先级）
      就是 ArduPilot 对"实时性"的全部承诺 ——
      跑不完不要紧，**但系统必须知道自己没跑完，并且降速**。
""")


if __name__ == "__main__":
    main()
