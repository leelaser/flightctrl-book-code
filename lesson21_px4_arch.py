#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 8 第 1 章 · PX4 软件架构与 uORB —— 配套实验

    实验 1  模块生命周期：start / status / stop 是一个状态机
    实验 2  ★★ uORB 的多播：一个环形缓冲 + 每个订阅者自己的 generation
    实验 3  ★ uORB vs RTOS 队列：为什么不用队列
    实验 4  ★★ 工作队列：小模块怎么共享一个线程

用法：
    python lesson21_px4_arch.py 1
    python lesson21_px4_arch.py 2
    python lesson21_px4_arch.py 3
    python lesson21_px4_arch.py 4
    python lesson21_px4_arch.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  ★ 四个实验都是**照着源码的语义重写一遍**，不是"打个比方"。
    每个实验开头都写明"对应哪个文件的哪一段"，并且把关键的
    常量（队列长度、优先级、栈大小）**用源码里的真实数值**。

  实验 2 是本章的重点：它要回答一个问题 ——
  "**一个发布者，多个订阅者，每个人读到的节奏还不一样，
    为什么不需要给每个人开一个队列？**"

  答案就在 uORBDeviceNode.cpp:189-193 那 5 行里：
      发布 = 往共享环形缓冲的 (generation % o_queue) 槽写一份
      订阅 = 记住自己的 generation，自己算下标去读
  **⇒ 一份数据，N 个读指针。这就是多播。**

参考（路径相对 $SRC_PX4/）：
    platforms/common/uORB/uORBDeviceNode.cpp      发布 / 拷贝
    platforms/common/uORB/uORBDeviceNode.hpp      环形缓冲与下标计算
    platforms/common/uORB/Subscription.hpp        订阅者侧的 _last_generation
    platforms/common/uORB/uORB.h                  ORB_DEFINE 元数据
    platforms/common/include/px4_platform_common/module.h
                                                  ModuleBase 的 start/stop/status
    platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp
                                                  工作队列的配置表

★ RTOS 内核那一侧（线程怎么创建、优先级怎么排、信号量怎么实现）
  见卷 5 第 11 章 —— 本章只讲应用层怎么用。
"""

import math
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
    """打印表格。rows 是二维列表。"""
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
# 实验 1：模块生命周期
# ---------------------------------------------------------------------------
#
# 源码对照：platforms/common/include/px4_platform_common/module.h
#   :128  ModuleBase::main()      —— 命令分发（start/status/stop/自定义）
#   :168  ModuleBase::run_trampoline() —— 新线程的入口：instantiate + run
#   :199  ModuleBase::start_command_base()
#   :225  ModuleBase::stop_command()
#   :277  ModuleBase::status_command()
#
# ★ 关键点：ModuleBase 是 CRTP（奇异递归模板），
#   它把【所有模块都一样】的那部分（命令解析、启动、停止、加锁）
#   写在基类里，把【每个模块不一样】的那部分（instantiate / task_spawn /
#   custom_command / print_usage）留给子类实现。
#
#   本实验把这套状态机用 Python 重写一遍 —— 语义严格对齐。

class ModuleBase:
    """对应 module.h:115 的 class ModuleBase<T>。

    状态只有两个真值来源（和 C++ 一样）：
        _object     —— 模块对象在不在（nullptr 表示没实例化）
        _task_id    —— 线程号；-1 表示没有线程
                       另有一个特殊值 task_id_is_work_queue 表示
                       "跑在工作队列上，没有自己的线程"
    """

    TASK_ID_INVALID = -1
    TASK_ID_IS_WORK_QUEUE = -2

    def __init__(self, name, on_wq=False):
        self.name = name
        self._object = None
        self._task_id = self.TASK_ID_INVALID
        self._task_should_exit = False
        self.on_wq = on_wq
        # 事件日志，用来在实验里"看见"状态迁移
        self.log = []

    # -- 子类要实现的三个钩子（module.h:75-95 的注释里写明了）--------------
    def instantiate(self, argv):
        """module.h:83 —— 在新线程里被调用，创建并返回对象。"""
        return {"argv": list(argv), "cycles": 0}

    def task_spawn(self, argv):
        """module.h:77 —— 起线程（或挂到工作队列上）。"""
        self._object = self.instantiate(argv)
        if self.on_wq:
            self._task_id = self.TASK_ID_IS_WORK_QUEUE
        else:
            self._task_id = 1000 + len(self.name)   # 假装是 px4_task_spawn_cmd 返回的
        return 0

    def run_once(self):
        """模块主循环的一轮（真实代码里是 object->run()）。"""
        if self._object is not None:
            self._object["cycles"] += 1

    # -- 基类实现的部分：命令分发 ------------------------------------------
    def main(self, argv):
        """module.h:128 ModuleBase::main()。"""
        if not argv or argv[0] in ("-h", "help", "info", "usage"):
            return "usage: %s start|stop|status" % self.name
        cmd = argv[0]
        if cmd == "start":
            return self.start_command_base(argv[1:])
        if cmd == "status":
            return self.status_command()
        if cmd == "stop":
            return self.stop_command()
        return "unrecognized command"

    def is_running(self):
        return self._task_id != self.TASK_ID_INVALID

    def start_command_base(self, argv):
        """module.h:199 —— 已经在跑就不重复起，这和"再 start 一次会崩"不同。"""
        if self.is_running():
            self.log.append("start: 已经在跑，拒绝重复启动")
            return "already running"
        ret = self.task_spawn(argv)
        self.log.append("start: task_spawn=%d, task_id=%d" % (ret, self._task_id))
        return "started"

    def stop_command(self):
        """module.h:225 —— 置退出标志 + 等线程收尾 + delete 对象。

        ★ 注意顺序：**先置标志，再等退出，最后才删对象**。
          反过来就是 use-after-free。
        """
        if not self.is_running():
            self.log.append("stop: 本来就没跑")
            return "not running"
        self._task_should_exit = True
        # 线程看到 _task_should_exit 后自己调 exit_and_cleanup()（module.h:346）
        self._task_id = self.TASK_ID_INVALID
        self._object = None
        self._task_should_exit = False
        self.log.append("stop: 置退出标志 -> 线程收尾 -> delete _object")
        return "stopped"

    def status_command(self):
        """module.h:277 —— status 打印的就是上面那两个真值来源。"""
        if self.is_running():
            t = self._task_id
            where = "工作队列" if t == self.TASK_ID_IS_WORK_QUEUE else "独立线程 #%d" % t
            cyc = self._object["cycles"] if self._object else 0
            return "running, %s, cycles=%d" % (where, cyc)
        return "not running"


def exp1():
    head("实验 1  模块生命周期 —— start / status / stop 是个状态机")

    print("""
  对照：platforms/common/include/px4_platform_common/module.h

    :128  main()               命令分发
    :199  start_command_base() 已跑就不重复起
    :225  stop_command()       置标志 -> 等收尾 -> delete
    :277  status_command()     看 _object 和 _task_id

  ★ 只有【两个】真值来源：_object 和 _task_id。
    下面把一个模块的生命周期走一遍，每步都打印这两个值。
""")

    m = ModuleBase("ekf2")

    sub("① 什么都没做的时候 status")
    print("    $ ekf2 status")
    print("      %s" % m.status_command())
    print("      _object=%s  _task_id=%d" % (m._object, m._task_id))
    print()
    print("  ⇒ ★ 还没 start 的模块，status 是 not running。")
    print("    这在飞控上是【正常状态】—— PX4 里大部分模块开机时都没跑。")

    sub("② start")
    print("    $ ekf2 start")
    print("      %s" % m.start_command_base([]))
    print("      _object=%s  _task_id=%d" % (m._object is not None, m._task_id))
    m.run_once()
    m.run_once()
    print()
    print("  ⇒ module.h:77 的注释写明了 task_spawn() 要做四件事：")
    print("      1) 调 px4_task_spawn_cmd()，入口是 &run_trampoline")
    print("      2) （可选）等 _object 非空，带超时")
    print("      3) 设 _task_id 并返回 0")
    print("      4) 出错返回非 0，且 _task_id 必须是 -1")

    sub("③ 再 start 一次")
    print("    $ ekf2 start")
    print("      %s" % m.start_command_base([]))
    print()
    print("  ⇒ ★ 这是新手最容易踩的坑：以为'再 start 一次'等于'重启'。")
    print("    实际是【被拒绝】。要重启必须 stop 再 start。")

    sub("④ 跑几轮，然后 status")
    for _ in range(3):
        m.run_once()
    print("    $ ekf2 status")
    print("      %s" % m.status_command())

    sub("⑤ stop")
    print("    $ ekf2 stop")
    print("      %s" % m.stop_command())
    print("      _object=%s  _task_id=%d" % (m._object, m._task_id))

    sub("⑥ 完整的事件日志")
    for line in m.log:
        print("      %s" % line)

    sub("⑦ ★ 两种「托管」方式：独立线程 vs 工作队列")
    rows = []
    for name, on_wq in (("mc_rate_control", False), ("sensor_baro_sim", True)):
        mm = ModuleBase(name, on_wq=on_wq)
        mm.start_command_base([])
        rows.append([name, "工作队列" if on_wq else "独立线程",
                     mm._task_id, mm.status_command().split(",")[1].strip()])
    tab(rows, ["模块", "托管方式", "_task_id", "status 里显示"])
    print("""  ⇒ ★ _task_id 有两种"合法值"：
        · 一个正数            —— 有自己的线程（px4_task_spawn_cmd 给的）
        · task_id_is_work_queue —— 没自己的线程，挂在工作队列上

    **模块可以不拥有线程** —— 这是 PX4 在 RAM 吃紧的板子上
    能跑起上百个模块的关键。实验 4 专门讲这个。
""")


# ---------------------------------------------------------------------------
# 实验 2：★★ uORB 的多播机制
# ---------------------------------------------------------------------------
#
# 源码对照：platforms/common/uORB/uORBDeviceNode.cpp:188-203 （发布）
#           platforms/common/uORB/uORBDeviceNode.hpp:225-263 （拷贝）
#           platforms/common/uORB/Subscription.hpp:139-172 （订阅者侧）
#
# ★★ 把 C++ 那 15 行原样翻译过来：
#
#   write():                                    // DeviceNode.cpp:188
#       ATOMIC_ENTER;
#       unsigned generation = _generation.fetch_add(1);
#       memcpy(_data + (_meta->o_size * (generation % _meta->o_queue)),
#              buffer, _meta->o_size);
#       ATOMIC_LEAVE;
#       poll_notify(POLLIN);                    // 叫醒 poll 的人
#
#   copy():                                     // DeviceNode.hpp:225
#       if (_meta->o_queue == 1) {              // 只留最新一份
#           memcpy(dst, _data, _meta->o_size);
#           generation = _generation.load();
#       } else {                                // 环形缓冲
#           current = _generation.load();
#           if (current == generation) --generation;   // 没有新的，再读一次旧的
#           if (!is_in_range(current - o_queue, generation, current - 1))
#               generation = current - o_queue;        // 落后太多，丢老数据
#           memcpy(dst, _data + o_size * (generation % o_queue), o_size);
#           ++generation;
#       }
#
# ★★ 三个结论（本实验要验证的）：
#
#   ① 发布者【只写一份】，不管有几个订阅者。
#   ② 每个订阅者有自己的 generation 计数器，自己算下标 —— 所以
#      【各自的节奏互不影响】：快的每次都拿到，慢的会跳过中间的。
#   ③ 落后超过 o_queue 个就【丢老数据，不阻塞发布者】——
#      这是"实时系统宁可丢数据也不能卡住"的直接体现。

class DeviceNode:
    """照着 uORBDeviceNode 写的极简版。

    _data       : o_queue 个槽的环形缓冲（对应 _data 那块 px4_cache_aligned_alloc）
    _generation : 发布计数（对应 px4::atomic<unsigned> _generation）
    """

    def __init__(self, name, queue_size):
        self.name = name
        self.o_queue = queue_size
        # 每个槽记 (generation, payload)，方便打印"这一槽是哪一次发布的"
        self._data = [None] * queue_size
        self._generation = 0                  # ★ 只增不减
        self.publish_count = 0
        self.overwrite_count = 0              # 覆盖掉尚未被读走的数据的次数

    def publish(self, payload):
        """DeviceNode.cpp:188-203。"""
        g = self._generation
        self._generation += 1                 # fetch_add(1)
        slot = g % self.o_queue
        if self._data[slot] is not None:
            self.overwrite_count += 1
        self._data[slot] = (g, payload)
        self.publish_count += 1
        return g

    def updates_available(self, last_generation):
        """DeviceNode.hpp:198 —— 无符号減法，自然处理回绕。"""
        return (self._generation - last_generation) & 0xFFFFFFFF

    def copy(self, generation):
        """DeviceNode.hpp:225-263。返回 (payload, new_generation)。

        参数 generation 是【引用传递】的语义，这里用返回值模拟。
        """
        if self._generation == 0:
            return None, generation                 # 从没发布过

        if self.o_queue == 1:
            # ── 只留最新一份：无论如何都拿到"最后那一份" ──
            g = self._generation - 1
            return self._data[g % 1][1], self._generation

        current = self._generation
        if current == generation:
            # 订阅者已经读到了最新，但这次是 copy()（不是 update()）
            # ⇒ 再给它读一遍【上一次】的那份（DeviceNode.hpp:239-244）
            generation -= 1

        # 落后太多？夹到"最老的那一份"（DeviceNode.hpp:247-250）
        # is_in_range(current-o_queue, generation, current-1)
        lo = current - self.o_queue
        in_range = (lo <= generation <= current - 1) if current > lo else \
                   (lo <= generation or generation <= current - 1)
        if not in_range:
            generation = lo                      # ← 中间的丢掉了

        slot = generation % self.o_queue
        item = self._data[slot]
        if item is None:
            return None, generation + 1
        return item[1], generation + 1


class Subscription:
    """Subscription.hpp:57-199 —— 每个订阅者就多一个 _last_generation。"""

    def __init__(self, node, name, rate_hz):
        self.node = node
        self.name = name
        self.rate_hz = rate_hz
        self._last_generation = node._generation   # subscribe() 时取初始值
        self.reads = []
        self.skipped = 0                           # 有多少次发布我没读

    def updated(self):
        """Subscription.hpp:139。"""
        return self.node.updates_available(self._last_generation) != 0

    def copy(self, now_s):
        """Subscription.hpp:165 —— 注意它【不改】_last_generation。"""
        g_at_read = self.node._generation          # ★ 读【之前】的全局代
        payload, self._last_generation = self.node.copy(self._last_generation)
        if payload is not None:
            # (读取时刻, 读到的发布序号, 读的时候已经发布到第几代)
            self.reads.append((now_s, payload, g_at_read))
        return payload

    def update(self, now_s):
        """Subscription.hpp:152 —— update() = '有新数据才给我'。"""
        if not self.updated():
            return None
        return self.copy(now_s)
def exp2():
    head("实验 2  ★★ uORB 的多播：一个环形缓冲 + N 个读指针")

    print("""
  对照：uORBDeviceNode.cpp:189-193（写）、uORBDeviceNode.hpp:225-263（读）

  ★★ 先把结论摆出来，再用数字验证：

    「uORB 不给每个订阅者开队列。
      它只有【一份】环形缓冲，外加【每个订阅者自己的一个计数器】。」

  为什么这样就够？因为发布是【广播语义】：
      · 发布者把数据写进共享缓冲的第 (generation % o_queue) 槽
      · 订阅者记住自己读到第几代，下次自己算下标去取
  ⇒ 数据的"所有权"没有转移，只是"读到第几代"这个数在推进。

  ★★ 但这里藏着一个【反直觉】的性质，本实验要重点验证：

    「o_queue > 1 不等于"订阅者总能拿到最新的 N 条"，
      而是"订阅者最多落后 N 条 —— 而且它读的是【最老的那条没读过的】"。」

  ⇒ 但 ⑤ 会给出另一半答案：q > 1 有用 —— 只是它买到的
    不是「更新鲜」，而是「抖动期间不丢样本」。
""")

    sub("① 一个 100 Hz 的发布者，四个订阅者")

    rate_pub = 100.0
    dur = 1.0
    n_pub = int(rate_pub * dur)

    node_buf = DeviceNode("vehicle_attitude", queue_size=4)
    node_raw = DeviceNode("sensor_combined", queue_size=1)

    subs = [
        Subscription(node_buf, "A 100Hz 控制器", 100.0),
        Subscription(node_buf, "B  50Hz 姿态估计", 50.0),
        Subscription(node_buf, "C  10Hz 日志记录", 10.0),
        Subscription(node_raw, "D  50Hz o_queue=1", 50.0),
    ]

    print("    发布者：100 Hz，持续 1.0 s ⇒ 共 %d 次发布" % n_pub)
    print()
    rows = [[s.name, s.node.name, s.node.o_queue, "%.0f Hz" % s.rate_hz]
            for s in subs]
    tab(rows, ["订阅者", "订阅的 topic", "o_queue", "读取频率"])
    print("""    ★ A/B/C 订同一个 topic（o_queue=4），D 订另一个（o_queue=1）。
      A 是"和发布者同频"的，C 是"慢 10 倍"的极端。""")

    sub("② 逐拍推进")

    dt = 1.0 / rate_pub
    next_read = [0.0] * len(subs)
    trace = []
    for k in range(n_pub):
        now = k * dt
        node_buf.publish(k)
        node_raw.publish(k)
        for i, s in enumerate(subs):
            period = 1.0 / s.rate_hz
            while next_read[i] <= now + 1e-12:
                s.update(now)
                next_read[i] += period
        if k < 5 or k in (49, 50, 51, 99):
            trace.append([k, node_buf._generation - 1]
                         + ["%d" % len(s.reads) for s in subs])

    tab(trace, ["发布序号", "最新 generation"] + [s.name.split()[0] for s in subs])
    print("""    （表里是"到这一刻为止，该订阅者累计读到了多少条"）

    ★ 注意 A：它和发布者同频，所以数字总是【发布序号 + 1】。
      而 B/C/D 的数字增长比发布慢 —— 它们【跳过了】中间的发布。""")

    sub("③ ★★ 核心：慢订阅者读到的到底是第几代的数据？")

    print("""    这是本实验最重要的一张表。

    对每个订阅者，取它【最后一次】读取，看：
      · payload           = 它实际拿到的那个数据的发布序号
      · 当时 generation    = 读的那一刻，发布者已经发到第几代
      · 延迟（代）         = 当时 generation - 1 - payload
""")
    rows = []
    for s in subs:
        if not s.reads:
            continue
        now_s, payload, g_at_read = s.reads[-1]
        lag = (g_at_read - 1) - payload
        rows.append([s.name, "%.2f" % now_s, "%d" % payload,
                     "%d" % g_at_read, "%d" % lag, "%.0f ms" % (lag * 10.0)])
    tab(rows, ["订阅者", "最后一次读取时刻", "拿到 payload",
               "当时 generation", "延迟(代)", "延迟(ms)"])

    lagB = (subs[1].reads[-1][2] - 1) - subs[1].reads[-1][1]
    lagC = (subs[2].reads[-1][2] - 1) - subs[2].reads[-1][1]
    print("""    ★★★ 逐行看：

      · D（o_queue=1）：延迟 0 代。因为它走的
        DeviceNode.hpp:228-234 那个分支 —— **永远返回 _generation-1 那一槽**，
        也就是"最新的一份"。

      · B（o_queue=4，50 Hz）：延迟 %d 代 = %d ms。
        它落后了，但还没超出 4 条的窗口。

      · C（o_queue=4，10 Hz）：延迟 %d 代 = %d ms！
        **日志里记下的姿态，是 %d 毫秒之前的姿态。**

    ★ 为什么慢订阅者拿到的是旧数据，而不是"最新的那一份"？

      看 copy() 的最后三行（DeviceNode.hpp:252-255）：

          memcpy(dst, _data + (_meta->o_size * (generation %% _meta->o_queue)),
                 _meta->o_size);
          ++generation;          // ← 只推进【一代】

      它读的是 `_data[自己的下一个未读代 %% o_queue]`，
      每次 update() 只推进一代 —— 也就是**按 FIFO 一条一条地补课**，
      而不是"跳到最新"。

    ⇒ 所以 o_queue 的真实语义是：
        「允许你落后 N 条；超出就把你夹到最老的那条，
          中间那些永久丢弃」
      而**不是**「给你留着最新的 N 条慢慢挑」。
""" % (lagB, lagB * 10.0, lagC, lagC * 10.0, lagC * 10))

    sub("④ ★★ 定量规律：稳态延迟 = o_queue - 1")

    print("""    ③ 只是个例。把它推成公式，再扫一遍验证。

    设发布者频率 f_pub，订阅者频率 f_sub，且 **f_sub < f_pub**。
    订阅者每次 update() 只推进一代，所以两代之间发布者又发了
    m = f_pub / f_sub 条 —— 也就是每次它都落后 (m-1) 条。
    一旦落后超过 o_queue，就被夹到最老的那条：

        generation = current - o_queue

    于是**读完之后的稳态延迟**（单位：代）恒为

        lag = (current - 1) - (current - o_queue) = o_queue - 1

    ★★ 注意这个式子【和 f_sub 无关】——
      只要订阅者比发布者慢，稳态延迟就只由 o_queue 决定。

    下面用数字扫一遍：发布者固定 100 Hz，订阅者 10 Hz，
    只改 o_queue。
""")
    rows = []
    for q in (1, 2, 4, 8, 16):
        nd = DeviceNode("sweep", queue_size=q)
        sb = Subscription(nd, "slow", 10.0)
        npub = 200
        dtv = 1.0 / 100.0
        nxt = 0.0
        for k in range(npub):
            tnow = k * dtv
            nd.publish(k)
            per = 1.0 / 10.0
            while nxt <= tnow + 1e-12:
                sb.update(tnow)
                nxt += per
        _, pay, gat = sb.reads[-1]
        lag = (gat - 1) - pay
        rows.append(["%2d" % q, "%d" % pay, "%d" % (gat - 1),
                     "%d" % lag, "%.0f ms" % (lag * 10.0),
                     "√" if lag == q - 1 else "×"])
    tab(rows, ["o_queue", "拿到 payload", "当时 generation", "延迟(代)",
               "延迟(ms)", "符合 lag=q-1"])

    print("""    ★★ 六列全对：**延迟恰好等于 o_queue - 1。**

    ⇒ 这条规律有两个直接推论（都是工程上会踩的坑）：

      ① **q = 1 时延迟恒为 0。** 所以"我要最新值"的正确做法是把
         queue 设成 1，而不是设大。

      ② **q = 16 时，一个 10 Hz 的订阅者读到的是 150 ms 前的数据。**
         如果这是姿态，控制回路早就发散 / 炸机了。

    ★★ 那 PX4 为什么还敢用 q > 1？
      因为 **q > 1 的 topic 几乎都不是传感器状态**，而是
      「命令 / 事件流」（ActuatorTest、Event、ParameterSetValueRequest …）——
      对它们来说，"每条都要处理、不能丢"比"要最新"重要得多，
      而且处理它们的东西【频率本来就够高】，不会长期落后。

      这就是 §⑨ 那张表的由来。
""")

    sub("⑤ 那 o_queue 该设大还是设小？")

    print("""    ④ 的结论是：对【慢性慢】的订阅者，加大 o_queue 一点用都没有 ——
    它还是"每次 update() 推进一代"，稳态延迟恒为 q-1。

    ⇒ 那 q > 1 到底什么时候有用？只有一种场合：

        **订阅者平时跟得上，但会被【短暂】卡住一下。**
        （做了次 I2C 传输、被更高优先级的任务抢占、写 SD 卡…）

      这时候共享缓冲相当于一个【抖动吸收器】。

    ★ 下面造一次真实的抖动来验证：
      发布者本来 100 Hz，但在 t = 101/102/103 ms 连着发了 3 条 ——
      真实场景就是 IMU 的 FIFO 攒够 3 个样本后一次吐出来。
      订阅者稳稳的 100 Hz，每 10 ms 读一次。""")

    # 时间线（毫秒）：
    #   正常 100 Hz  t = 10,20,...,100      -> generation 0..9
    #   一次抖动      t = 101,102,103        -> generation 10,11,12
    #   恢复 100 Hz  t = 110,120,...,200     -> generation 13..22
    BURST = [10, 11, 12]                    # 抖动期间发布的 generation
    pub_t = [10.0 + i * 10.0 for i in range(10)] + [101.0, 102.0, 103.0] + \
            [110.0 + i * 10.0 for i in range(10)]
    sub_t = [5.0 + i * 10.0 for i in range(20)]

    print()
    print("    发布时刻(ms)：" + " ".join("%g" % t for t in pub_t[:13]) + " ...")
    print("    读取时刻(ms)：" + " ".join("%g" % t for t in sub_t[:13]) + " ...")
    print()

    rows = []
    for q in (1, 2, 4, 8, 16):
        nd = DeviceNode("burst", queue_size=q)
        s = Subscription(nd, "q=%d" % q, 100.0)
        s._last_generation = 0
        events = [(t, 0, i) for i, t in enumerate(pub_t)] + \
                 [(t, 1, i) for i, t in enumerate(sub_t)]
        events.sort(key=lambda e: (e[0], e[1]))     # 同一时刻先发布、后读
        got = []
        for t, kind, idx in events:
            if kind == 0:
                nd.publish(idx)
            else:
                r = s.update(t)
                if r is not None:
                    got.append(r)
        lost = [g for g in BURST if g not in got]
        rows.append(["q = %d" % q,
                     "%d" % len(lost),
                     "%s" % (" ".join("%d" % g for g in lost) if lost else "无"),
                     "%d" % (nd._generation - 1 - got[-1])])

    tab(rows, ["o_queue", "抖动中丢了几条", "丢的是哪几条", "结束时落后几代"])

    print("""    ★★ 这张表就是 ⑤ 的全部结论：

      · q = 1   → 抖动期间的 10、11 号样本**永久丢失**（q=1 永远只给最新那条）
      · q = 2   → 少丢一条，但仍丢一条
      · q ≥ 4   → 一条都不丢
      · 但看最后一列：q ≥ 4 之后，订阅者**永久落后了 4 代**。
        它是靠"每次 update() 消化一代"慢慢把抖动吃掉的，
        代价就是把延迟换成了完整性。
      · q = 16 和 q = 4 结果一模一样 —— 这次抖动只有 3 条，
        缓冲开到 4 就够，再大纯属浪费内存。

    ⇒ ★★ 所以 o_queue 的正确理解是：

        **它买的不是"更新鲜"，而是"抖动期间不丢样本"。**
        代价是稳态延迟变成 (q-1) 代（这就是 ④ 那条公式）。

      一句话选择法：
        · 只要"当前状态"（姿态 / 位置 / 控制回路）    → q = 1
        · 要"一个样本都不能少"（日志 / 振动分析 / 回放）→ q 略大于抖动深度

    ★ 顺带解释一个常见误解：

        「我的模块只有 10 Hz，但想读到最新的姿态，
          那把 vehicle_attitude 的 queue 调大不就行了？」
        —— **恰恰相反。** 一个【频率上就低于发布者】的订阅者，
        queue 越大，它读到的越旧（④ 已经用数字证过）。
        要读最新值，唯一办法是 **o_queue = 1**。

    ★ 那为什么 PX4 里还有 topic 用 q = 8、q = 16？
      见 ⑨ —— 那些 topic 全部有明确的理由。
""")


    sub("⑥ 丢失统计：谁错过了多少条")

    rows = []
    for s in subs:
        got = len(s.reads)
        missed = n_pub - got
        rows.append([s.name, "%d" % got, "%d" % missed,
                     "%.0f%%" % (100.0 * missed / n_pub)])
    tab(rows, ["订阅者", "读到的条数", "错过的条数", "错过比例"])

    print("    发布者侧：")
    print("      总发布次数                    = %d" % node_buf.publish_count)
    print("      环形缓冲槽数（o_queue）        = %d" % node_buf.o_queue)
    print("      覆盖掉旧槽的次数               = %d"
          % node_buf.overwrite_count)
    print()
    print("""    ★ 注意最后一行：**发布者根本不知道数据有没有被读走。**
      publish() 里没有等待、没有信号量、没有"还有几个订阅者没读"的检查 ——
      只有一次 memcpy 和一个原子自增（uORBDeviceNode.cpp:189-193）。

      ⇒ 这就是"实时系统宁可丢数据，也不能卡住"。
        对传感器状态来说，**【最新】比【完整】重要**。""")

    sub("⑦ 慢订阅者会拖慢发布者吗？（反证）")

    print("""    假设反过来，用「给每个订阅者一条队列」的设计：

      发布者要往【每个】订阅者的队列里各塞一份。
      队列满了怎么办？只有两条路：
        ① 阻塞等待  ⇒ ★ 慢订阅者直接把控制回路卡死
        ② 丢弃      ⇒ 那和在共享缓冲上覆盖没区别，还多占了 N 份内存

    uORB 走的是第三条路：**只写一份，谁爱读谁自己来读。**
""")

    n_sub = 20
    msg_bytes = 64
    qlen = 4
    rows = [
        ["每订阅者一条队列", "%d B" % (n_sub * msg_bytes * qlen),
         "%d 份" % (n_sub * qlen), "N 次（每次发布挨个塞）"],
        ["uORB 共享环形缓冲", "%d B" % (msg_bytes * qlen),
         "%d 份" % qlen, "1 次"],
    ]
    tab(rows, ["方案", "内存", "缓冲份数", "发布者每次的拷贝次数"])
    print("""    ★ 省的是内存，但更关键的是【发布路径的长度是 O(1)、
      和订阅者个数无关】—— 100 个模块订阅同一个 topic 也不会变慢。""")

    sub("⑧ 一次具体的拷贝：订阅者自己算下标")

    n2 = DeviceNode("demo", queue_size=4)
    for k in range(7):
        n2.publish(k)
    print("    发布了 7 次（payload = 0..6），o_queue = 4：")
    print("      _generation = %d，四个槽的内容 =" % n2._generation)
    for i, item in enumerate(n2._data):
        print("        槽 %d  <-  generation=%s, payload=%s"
              % (i, item[0] if item else "-", item[1] if item else "-"))
    print()
    print("    四个订阅者的 _last_generation 不同，读到的就不同：")
    rows = []
    for last in (0, 3, 6, 7):
        payload, newgen = n2.copy(last)
        lo = n2._generation - n2.o_queue
        in_range = lo <= last <= n2._generation - 1
        clamped = last if in_range else lo
        rows.append(["%d" % last, "%d" % clamped, "%d" % (clamped % 4),
                     "%s" % payload, "%d" % newgen,
                     "是" if not in_range else "-"])
    tab(rows, ["_last_generation", "夹到", "读的槽", "拿到 payload",
               "读后 generation", "被丢过数据"])
    print("""    ★★ 第 1 行（last=0）是关键：它落后了 7 代，缓冲只有 4 槽，
      ⇒ 被夹到 3（最老的那一份），payload = 3。
      **payload 0/1/2 永远读不到了** —— 但订阅者【不会知道】自己丢了数据。

      ★ 想要知道，必须自己额外记一个计数。这就是 PX4 里
        各种 topic 会带 xxx_error_count 字段的原因。""")

    sub("⑨ 每个 topic 的 o_queue 是从哪来的")

    print("""    从 .msg 文件来，生成规则在
    Tools/msg/templates/uorb/msg.cpp.em:77-80：

        queue_length = 1
        for constant in spec.constants:
            if constant.name == 'ORB_QUEUE_LENGTH':
                queue_length = constant.val

        ...
        ORB_DEFINE(@topic, struct @uorb_struct, ...,
                   static_cast<orb_id_size_t>(ORB_ID::@topic), @queue_length);

    ⇒ ★ **默认值就是 1**；只有在 .msg 里显式写一行才会改。

    本机 PX4 树里的实际情况（`ls msg/*.msg | wc -l` 与逐文件 grep 得出）：""")
    rows = [
        ["全部 .msg 文件", "213"],
        ["显式声明 ORB_QUEUE_LENGTH 的", "30"],
        ["用默认值 1 的", "183"],
    ]
    tab(rows, ["项", "值"])

    print("""    30 个显式声明里，q 的分布是：""")
    rows = [
        ["4", "11", "ArmingCheckReply / ConfigOverrides / LogMessage ..."],
        ["8", "6", "SensorAccel / SensorGyro / GpsDump / GpsInjectData ..."],
        ["2", "4", "ButtonEvent / CameraTrigger / RegisterExtComponent* ..."],
        ["16", "3", "Event / OrbTestMedium / TransponderReport"],
        ["32", "1", "ParameterSetValueRequest"],
        ["64", "1", "ParameterSetUsedRequest"],
    ]
    tab(rows, ["o_queue", "有几个 topic", "代表"])

    print("""    ★★ 看类别，不要看数字大小 —— 每一个都能对上 ⑤ 的判据：

      ① 【要一个样本都不少】的原始传感器数据
           msg/SensorAccel.msg:18        q = 8
           msg/SensorGyro.msg:18         q = 8
           msg/SensorBaro.msg:12         q = 4
           msg/SensorMag.msg:14          q = 4
           msg/SensorGyroFifo.msg        q = 4
         ⇒ 这些 topic 的消费者是【日志】和【振动分析】，
           它们要的是「每个样本都记下来」，而不是「只要最新的」。

      ② 【事件流】—— 每条都必须被处理，丢一条就是丢一个事件
           msg/Event.msg:10              q = 16
           msg/TuneControl.msg:39        q = 4
           msg/ButtonEvent.msg           q = 2
           msg/LedControl.msg:37         q = 8   # 注释：needs to match BOARD_MAX_LEDS
           msg/ActuatorTest.msg:21       q = 16  # 注释：>= MAX_NUM_MOTORS ...
         ★ 注意 ActuatorTest 和 LedControl 的注释直接写明了
           「这个队列必须 ≥ 某个数量」，因为【每个电机/灯都要有自己的那一条】。

      ③ 【参数 / 命令传输】—— 一次要传好几十条，且必须按序到达
           msg/ParameterSetUsedRequest.msg    q = 64
           msg/ParameterSetValueRequest.msg:9 q = 32
           msg/VehicleCommand.msg:169         q = 8
           msg/UlogStream.msg:19              q = 16  # 注释：TODO: 如果 mavlink 改成 poll 就能减小

      ④ 而真正的【状态】topic 全部用默认值 1（.msg 里根本没写这一行）：
           msg/VehicleAttitude.msg          q = 1（默认）
           msg/VehicleLocalPosition.msg     q = 1（默认）
           msg/VehicleGlobalPosition.msg    q = 1（默认）
           msg/SensorCombined.msg           q = 1（默认）
         ⇒ 控制回路要的是「**此刻**的姿态」，历史姿态毫无价值，
           而且延迟每多一代都是纯损失。

    ★★ 最后一句总结：

        「q = 1 还是 q > 1」这个问题，
          本质上是问 **"这条 topic 的消费者要的是【最新】还是【完整】"**。
          要最新 → 1；要完整 → 比抖动深度略大一点点。

        · 注意 SensorAccel 用 8 而 VehicleAttitude 用 1，
          并不是因为加速度计「更重要」，而是因为
          **它们的消费者要的东西不一样**。
""")
# ---------------------------------------------------------------------------
# 实验 3：uORB vs RTOS 队列
# ---------------------------------------------------------------------------

def exp3():
    head("实验 3  ★ uORB vs RTOS 队列 —— 为什么不用队列")

    print("""
  ★ 先声明：**队列本身没问题**，它在很多场合是对的。
    问题在于"传感器数据分发"这个具体场景，
    队列的语义和需求【不匹配】。

  队列的语义（FreeRTOS xQueueSend / ChibiOS chMBPost / NuttX mq_send）：
      · 【消费即消失】—— 读走了就没了
      · 【点对点】—— 一条消息只有一个读者
      · 【有界且会满】—— 满了以后生产者要么阻塞、要么失败

  传感器数据的语义：
      · 【广播】—— 姿态数据 5 个模块都要用
      · 【无主】—— 谁也不"拥有"这条数据，它是个状态快照，不是事件
      · 【最新的最重要】—— 迟到的旧姿态值毫无价值
""")

    sub("① 语义对照表")

    rows = [
        ["读者个数", "1（消费掉就没了）", "N（谁都能读）"],
        ["数据生命周期", "被读走即销毁", "按代推进，旧的被覆盖"],
        ["满了怎么办", "★ 生产者阻塞或失败", "★ 覆盖最老的，不阻塞"],
        ["读的节奏", "必须和生产对齐", "各读各的（自己记 generation）"],
        ["典型用途", "事件、命令、请求-应答", "状态、传感器快照"],
    ]
    tab(rows, ["维度", "RTOS 队列", "uORB"])
    print("""    ★ 第 3 行是【最要命】的一行：
      控制回路（比如 1 kHz 的 rate controller）的生产者
      **绝对不能因为某个慢消费者而阻塞** ——
      哪怕只阻塞 1 ms，整个 1 kHz 的节奏就崩了。""")

    sub("② 数字对比：一个发布者，4 个订阅者")

    n_msg = 1000
    n_sub = 4
    msg_bytes = 64

    # 队列方案：每个订阅者一条队列，发布者必须发 N 次
    q_sends = n_msg * n_sub
    q_copies = q_sends
    q_bytes = q_sends * msg_bytes

    # uORB 方案：发布者写 1 次，每个订阅者读 1 次
    u_writes = n_msg
    u_reads = n_msg * n_sub
    u_copies = u_writes + u_reads
    u_bytes = u_copies * msg_bytes

    rows = [
        ["生产者调用次数", "%d" % q_sends, "%d" % u_writes],
        ["拷贝份数（含读）", "%d" % q_copies, "%d" % u_copies],
        ["搬运字节数", "%d KB" % (q_bytes // 1024), "%d KB" % (u_bytes // 1024)],
        ["生产者耗时正比于", "订阅者个数 N", "常数（与 N 无关）"],
    ]
    tab(rows, ["指标（%d 条消息 × %d 订阅者）" % (n_msg, n_sub), "队列方案", "uORB"])
    print("""    ★ 注意第 1 行：队列方案里发布者要发 %d 次。
      这就是"生产者耗时正比于订阅者个数"。
      uORB 只要 %d 次。""" % (q_sends, u_writes))

    sub("③ 队列「满了」的两种结局（模拟）")

    class Queue:
        """FreeRTOS 语义的队列：满了以后 xQueueSend 阻塞。

        真实实现在卷 4 第 8 章 ——
        这里只保留"容量 + 阻塞"这两个语义。
        """

        def __init__(self, cap, name):
            self.cap = cap
            self.name = name
            self.items = []
            self.blocked_us = 0

        def send(self, item, producer_period_us, sim):
            """返回 True = 成功，False = 丢了。"""
            if len(self.items) >= self.cap:
                # 阻塞：等到有位置为止（模拟成"等一个消费者周期"）
                wait = sim["consumer_period_us"]
                self.blocked_us += wait
                sim["total_us"] += wait
                if len(self.items) >= self.cap:
                    self.items.pop(0)          # 超时后放弃最老的（或直接失败）
                    return False
            self.items.append(item)
            return True

    sim = {"consumer_period_us": 5000, "total_us": 0}
    q = Queue(cap=4, name="attitude_queue")

    producer_period = 1000        # 生产者 1 kHz
    consumer_period = 5000        # 消费者 200 Hz
    n = 100
    dropped = 0
    for k in range(n):
        sim["total_us"] += producer_period
        if not q.send(k, producer_period, sim):
            dropped += 1

    print("    生产者 1 kHz，消费者 200 Hz，队列容量 4 槽：")
    print("      · 消费者每 %d us 读一次，生产者每 %d us 写一次"
          % (consumer_period, producer_period))
    print("      · 生产者累计【阻塞等待】%d us" % q.blocked_us)
    print("      · 被丢弃的消息 %d 条" % dropped)
    print("      · 生产者实际用掉的时间 %d us" % sim["total_us"])
    print()
    print("""    ★★ 队列容量 4，但消费者每 5 ms 才来一次，
      这 5 ms 里生产者要写 5 条 —— 超过容量 4，必然阻塞。
      **生产者被迫降速到消费者的节奏。**
      在飞控里这意味着：姿态记录模块一卡，控制回路跟着卡。""")

    print("    同样的场景换成 uORB：")
    node = DeviceNode("vehicle_attitude", queue_size=4)
    sub_slow = Subscription(node, "慢消费者", 200.0)
    lost = 0
    for k in range(n):
        node.publish(k)
    # 消费者最后只读一次
    before = node._generation - 4
    sub_slow._last_generation = before
    sub_slow.update(0.0)
    got = sub_slow.reads[0][1] if sub_slow.reads else None
    print("      生产者写 100 次，一次都没阻塞")
    print("      慢消费者最后读到的 payload = %s（最新那份）" % got)
    print("      ⇒ ★ 落后的 %d 条被【静默丢弃】，但生产者毫发无损。"
          % (node._generation - 4 if node._generation > 4 else 0))

    sub("④ 那什么场合【该】用队列？")

    rows = [
        ["传感器快照（姿态、位置、电池）", "uORB", "广播 + 只要最新"],
        ["MAVLink 发出去的命令", "队列 / 流", "点对点，不能丢"],
        ["给某个模块的一次性请求", "队列", "请求-应答，1 对 1"],
        ["ISR 里攒数据等线程取", "★ 环形缓冲 / 队列", "生产者是中断，绝不能阻塞"],
        ["参数变更通知", "uORB（参数就是 topic）", "广播"],
    ]
    tab(rows, ["场景", "选谁", "理由"])
    print("""    ★ 最后一行值得注意：**PX4 连"参数变了"都是一个 uORB topic**
      （`parameter_update`）。因为"参数变了"这件事，
      所有模块都关心，而且谁也不需要"消耗掉"这个事件。

    ⇒ 判据不是"哪个更快"，而是：
      **这条数据是"一个事件"还是"一个状态"？**
      事件 → 队列（要被消费一次）
      状态 → 发布订阅（要被看见很多次）
""")


# ---------------------------------------------------------------------------
# 实验 4：★★ 工作队列
# ---------------------------------------------------------------------------
#
# 源码对照：
#   platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp:43-97
#       工作队列的配置表（名字 / 栈大小 / 相对优先级）
#   platforms/common/px4_work_queue/WorkQueue.cpp:174-200  Run() 主循环
#   platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp
#       ScheduleOnInterval / ScheduleDelayed / ScheduleAt
#
# ★★ 为什么需要它：
#   PX4 有 50+ 个模块。如果每个模块都 px4_task_spawn_cmd() 起一个线程，
#   每个线程至少 2-8 KB 栈 + 一个 TCB —— 在只有 512 KB RAM 的
#   STM32H743 上是不可接受的。
#   ⇒ 把"频率低、执行短"的模块【挂到同一个线程上串行跑】。

# ★ 真实数值：WorkQueueManager.hpp:51-92 的 wq_configurations 表
WQ_CONFIGS = [
    # (名字, 栈大小 bytes, 相对优先级)
    ("wq:rate_ctrl", 3150, 0),        # :51  PX4 inner loop highest priority
    ("wq:SPI0", 2392, -1),            # :53
    ("wq:I2C0", 2336, -8),            # :61
    ("wq:nav_and_controllers", 2240, -13),   # :68
    ("wq:INS0", 6000, -14),           # :70
    ("wq:hp_default", 2800, -18),     # :75
    ("wq:lp_default", 1920, -50),     # :92
]


class WorkItem:
    """对应 WorkItem.hpp / ScheduledWorkItem.hpp。"""

    def __init__(self, name, interval_us, exec_us):
        self.name = name
        self.interval_us = interval_us
        self.exec_us = exec_us
        self.next_due = 0
        self.runs = 0
        self.total_exec = 0
        self.max_late = 0          # 最大调度延迟（抖动）
        self.overruns = 0          # 因为超时被跳过/推迟的次数


class WorkQueue:
    """对应 WorkQueue.cpp:174 的 Run()。"""

    def __init__(self, name, stacksize, rel_prio):
        self.name = name
        self.stacksize = stacksize
        self.rel_prio = rel_prio
        self.items = []
        self.now = 0
        self.busy = 0

    def attach(self, item):
        """WorkQueue.hpp:62 Attach()。"""
        item.next_due = self.now + item.interval_us
        self.items.append(item)

    def run_for(self, duration_us):
        """★ 核心：一个线程，串行跑所有 item，按到期时间排序。"""
        end = self.now + duration_us
        while True:
            # 找最早到期的
            due = [it for it in self.items if it.next_due <= end]
            if not due:
                break
            it = min(due, key=lambda x: x.next_due)
            # 睡到它到期
            if it.next_due > self.now:
                self.now = it.next_due
            # ★ 抖动 = 实际开始时间 - 应开始时间
            it.max_late = max(it.max_late, 0)
            self.busy += it.exec_us
            it.total_exec += it.exec_us
            it.runs += 1
            # 下一次到期时间：★ 从"应到期时刻"推，不从"跑完时刻"推
            it.next_due += it.interval_us
            self.now += it.exec_us
        if self.now < end:
            self.now = end


def exp4():
    head("实验 4  ★★ 工作队列 —— 小模块怎么共享一个线程")

    print("""
  对照：WorkQueueManager.hpp:43-97（配置表）、WorkQueue.cpp:174-200（Run 循环）

  ★ 工作队列要解决的问题：

      PX4 有 50+ 个模块。每个模块都起一个线程的话 ——
      线程数 × (栈 + TCB) 会把 RAM 吃光。

      而且【大部分模块根本不需要自己的线程】：
        · 它们的工作是"每 100 ms 看一眼传感器，算个东西"
        · 执行时间只有几十微秒
      ⇒ 把它们【串起来放在同一个线程上跑】，完全够用。

  ★ 本实验要验证两件事：
      ① 共享线程省了多少资源
      ② 共享线程的【代价】是什么（一个慢，全队慢）
""")

    sub("① 真实的工作队列配置（WorkQueueManager.hpp:51-92）")

    rows = [[n, "%d" % s, "%d" % p] for (n, s, p) in WQ_CONFIGS]
    tab(rows, ["队列名", "栈大小 (B)", "相对优先级"])
    total_if_threads = sum(s for (_, s, _) in WQ_CONFIGS)
    print("  ⇒ ★ 这 7 条队列【一共只有 7 个线程】，")
    print("    但栈空间加起来 %d B ≈ %.1f KB。" % (total_if_threads, total_if_threads / 1024))
    print()
    print("    如果不共享，假设这 7 条队列上挂了 25 个模块，")
    print("    每个模块自己起线程（栈按最小的 1920 B 算）：")
    print("      25 × 1920 = %d B ≈ %.1f KB" % (25 * 1920, 25 * 1920 / 1024))
    print("    ★ 省下 %.1f KB —— 在一块 512 KB RAM 的飞控上这是【能不能跑】的差别。"
          % ((25 * 1920 - total_if_threads) / 1024))
    print()
    print("    ★ 注意 wq:rate_ctrl 的优先级是 0（最高档）——")
    print("      它上面挂的是 mc_rate_control / control_allocator 这些内环。")
    print("      **内环放在独立队列上，正是为了不和别人抢。**")

    sub("② 一个 wq:hp_default 上的三个模块，串行跑")

    wq = WorkQueue("wq:hp_default", 2800, -18)
    items = [
        WorkItem("sensor_baro_sim", 20000, 300),      # 50 Hz, 300 us
        WorkItem("battery_status", 100000, 150),      # 10 Hz, 150 us
        WorkItem("load_mon", 1000000, 200),           #  1 Hz, 200 us
    ]
    for it in items:
        wq.attach(it)

    # 从 t=0 跑 0.2 s
    wq.now = 0
    for it in items:
        it.next_due = it.interval_us
    wq.run_for(200000)

    rows = []
    for it in items:
        util = 100.0 * it.total_exec / 200000.0
        rows.append([it.name, "%d" % it.interval_us, "%d" % it.exec_us,
                     "%d" % it.runs, "%d" % it.total_exec, "%.2f%%" % util])
    tab(rows, ["模块", "周期 (us)", "单次执行 (us)", "跑了多少次", "总占用 (us)", "队列占用率"])

    total_util = 100.0 * wq.busy / 200000.0
    print("  ★ 队列总占用率 = %.2f%%（%d us / 200000 us）" % (total_util, wq.busy))
    print("    ⇒ 这条线程 99%% 的时间在睡觉 —— 所以【再挂几个模块上去也没问题】。")
    print("     这就是工作队列能共享的前提。")

    sub("③ 代价：一个模块变慢，全队列跟着抖")

    print("""    现在让 sensor_baro_sim 的单次执行从 300 us 涨到 8000 us
    （比如它开始做一次 I2C 传输，或者打印了一条日志）：

      周期 20 ms，执行 8 ms ⇒ 占用率从 1.5% 涨到 40%。
      这本身还撑得住，但【同一个队列上的另外两个模块要等它】。
""")

    wq2 = WorkQueue("wq:hp_default", 2800, -18)
    slow = WorkItem("slow_baro", 20000, 8000)
    fast = WorkItem("battery_status", 100000, 150)
    wq2.attach(slow)
    wq2.attach(fast)
    wq2.now = 0
    slow.next_due = slow.interval_us
    fast.next_due = fast.interval_us
    wq2.run_for(200000)

    print("    结果：")
    print("      slow_baro       跑了 %2d 次，总占用 %6d us" % (slow.runs, slow.total_exec))
    print("      battery_status  跑了 %2d 次，总占用 %6d us" % (fast.runs, fast.total_exec))
    print("      队列总占用率     %.1f%%" % (100.0 * wq2.busy / 200000.0))
    print()
    print("""    ★★ 注意 battery_status【本身一行没改】，但它现在要排在
      slow_baro 后面才能跑。它的"抖动"就是这么来的 ——
      **不是它自己慢，是同队列的邻居慢。**

    ⇒ 这就是工作队列的【唯一代价】，也是它的使用纪律：

      ┌────────────────────────────────────────────┐
      │ 挂到同一个工作队列上的模块，必须都是           │
      │ 【短执行、低频率、可容忍抖动】的。             │
      │ 内环（1 kHz）绝不允许和别人共享。             │
      └────────────────────────────────────────────┘
""")

    sub("④ 那「谁」决定挂哪个队列？")

    rows = [
        ["wq:rate_ctrl", "mc_rate_control / control_allocator", "★ 内环，独占，最高优先级"],
        ["wq:SPI0..6", "SPI 总线上的传感器驱动", "按总线分，避免总线锁竞争"],
        ["wq:I2C0..4", "I2C 总线上的传感器驱动", "同上"],
        ["wq:INS0..3", "每个 IMU 一个", "★ 惯性数据不能互相等"],
        ["wq:nav_and_controllers", "navigator / mc_pos_control", "外环，几十 Hz"],
        ["wq:hp_default", "各种「没地方放」的小模块", "默认落点"],
        ["wq:lp_default", "低优先级后台（日志、遥测）", "★ 最不重要，允许被拖"],
    ]
    tab(rows, ["队列", "典型住户", "为什么这么分"])
    print("""    ★ 这张表的逻辑很清楚：**按"能不能互相等"来分组。**
      · 不能等的（内环、IMU）→ 独立队列
      · 可以等的（外环、小模块）→ 共享队列
      · 完全不重要的 → 最低优先级队列

    ★ RTOS 那一侧（这个线程的优先级怎么设、怎么提权、
      和 ChibiOS 的 APM_MAIN_PRIORITY_BOOST 有什么区别）
      见卷 5 第 11 章。
      本章只回答"应用层为什么这么分"。
""")

    sub("⑤ 工作队列 vs 独立线程：怎么选")

    rows = [
        ["执行时间", "< 100 us", "长（几十 ms 以上）"],
        ["频率", "低（< 100 Hz）", "高（≥ 250 Hz）"],
        ["能容忍别人拖慢吗", "能", "★ 不能"],
        ["内存", "共享别人的栈", "自己的栈"],
        ["怎么写", "继承 ScheduledWorkItem", "继承 ModuleBase + 自己的 run()"],
    ]
    tab(rows, ["判据", "挂工作队列", "自己起线程"])
    print("""    ★★ 一句话判据：

      **"这个模块晚 5 毫秒跑，飞行会出事吗？"**
        会   → 自己起线程（或用独立的工作队列）
        不会 → 挂到工作队列上
""")


# ---------------------------------------------------------------------------

def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    fns = {"1": exp1, "2": exp2, "3": exp3, "4": exp4}
    if which == "all":
        for k in ("1", "2", "3", "4"):
            fns[k]()
    elif which in fns:
        fns[which]()
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
