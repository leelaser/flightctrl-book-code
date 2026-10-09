"""
============================================================================
 卷 6 第 3 章 · 任务与调度
 配套实验代码
============================================================================

 运行:  python lesson04_task_scheduling.py

 三个实验:
   1. 调度锁的意义       —— 不加锁会怎样?
   2. PX4 优先级流水线    —— 复现真实的执行顺序
   3. 栈大小计算器       —— 32 位 vs 64 位, 以及溢出检测

 素材来源 (PX4 1.15.4):
   platforms/nuttx/src/px4/common/tasks.cpp        px4_task_spawn_cmd
   platforms/common/include/px4_platform_common/tasks.h   优先级定义
   platforms/common/include/px4_platform_common/posix.h   PX4_STACK_ADJUSTED
============================================================================
"""


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


# ============================================================================
#  实验 1: 调度锁的意义
# ============================================================================
def exp1_sched_lock():
    sep("实验 1: 调度锁的意义")

    print("""
  场景: px4_task_spawn_cmd() 要做两件事
         ① task_create()          —— 创建任务, 它立刻变成"可运行"
         ② sched_setscheduler()   —— 设置调度策略

  问题: 如果这两步之间被抢占, 新任务会【带着默认策略】跑起来。

  真实代码 (platforms/nuttx/src/px4/common/tasks.cpp):

      int pid = task_create(name, priority, stack_size, entry, argv);
      if (pid > 0) {
          struct sched_param param = { .sched_priority = priority };
          sched_setscheduler(pid, scheduler, &param);     // <- 必须执行到!
      }
""")

    # ---- 模拟 ----
    class Task:
        _next_pid = 1000
        def __init__(self, name, priority, policy="SCHED_OTHER"):
            self.pid = Task._next_pid
            Task._next_pid += 1
            self.name = name
            self.priority = priority
            self.policy = policy
            self.started = False      # 是否已经开始执行
            self.log = []

        def run_first_time(self):
            """任务第一次被调度执行"""
            self.started = True

    class Scheduler:
        def __init__(self):
            self.ready = []           # 就绪队列
            self.locked = 0           # 调度锁计数
            self.log = []
            self.current = None

        def lock(self):
            self.locked += 1
            self.log.append(f"    [调度器] sched_lock()   count={self.locked}")

        def unlock(self):
            self.locked -= 1
            self.log.append(f"    [调度器] sched_unlock() count={self.locked}")

        def can_switch(self):
            return self.locked == 0

        def add_task(self, task):
            self.ready.append(task)
            self.log.append(f"    [调度器] task_create('{task.name}') -> pid={task.pid}"
                            f"  (策略默认={task.policy})")

        def maybe_switch(self, reason):
            """模拟"调度点": 如果没锁, 就会切换到最高优先级任务"""
            if not self.can_switch():
                self.log.append(f"    [调度器] ({reason}) 想切换, 但调度锁生效 -> 推迟")
                return None
            if not self.ready:
                return None
            winner = max(self.ready, key=lambda t: t.priority)
            return winner

    # ================================================================
    #  不加锁 (错误示范)
    # ================================================================
    print("  " + "-" * 72)
    print("  ❌ 不加锁")
    print("  " + "-" * 72)

    sched = Scheduler()
    print("\n  执行序列:")
    print("    [主任务] task_create('att_ctrl', prio=236)")

    newtask = Task("att_ctrl", 236)
    sched.add_task(newtask)          # ① 创建: 任务进入就绪队列
    for line in sched.log:
        print(line)
    sched.log.clear()

    # 模拟: 这里发生了一次调度 (比如时钟中断返回)
    print("    [中断] 时钟节拍 -> 触发调度")
    winner = sched.maybe_switch("时钟中断")
    if winner:
        print(f"    [调度器] 切换到 '{winner.name}' (优先级 {winner.priority})")
        winner.run_first_time()
        print(f"    ⚠️ '{winner.name}' 开始运行! 但它的策略还是 {winner.policy}")
        print(f"       (sched_setscheduler() 还没执行!)")

    print(f"\n    [主任务] 继续执行 sched_setscheduler(pid={newtask.pid}, SCHED_FIFO)")
    print(f"    ⚠️ 太晚了 —— 任务已经带着 {newtask.policy} 跑起来了")

    print("""
    【后果分析】

      这个例子中, 后果是"任务用了错误的调度策略"。
      更严重的场景:

        · 如果新任务在创建后、配置前就【访问了未初始化的资源】
        · 如果新任务【立刻开始控制电机】而参数还没设好
        · 如果多个新任务之间的【创建顺序】影响结果

      ==> 这类 bug 的特点是【偶发】—— 取决于调度时机, 极难复现。
""")

    # ================================================================
    #  加锁 (正确做法)
    # ================================================================
    print("  " + "-" * 72)
    print("  ✅ 加锁 (PX4 的做法)")
    print("  " + "-" * 72)

    sched2 = Scheduler()
    Task._next_pid = 2000
    print("\n  执行序列:")

    sched2.lock()
    for line in sched2.log: print(line)
    sched2.log.clear()

    print("    [主任务] task_create('att_ctrl', prio=236)")
    newtask2 = Task("att_ctrl", 236)
    sched2.add_task(newtask2)
    for line in sched2.log: print(line)
    sched2.log.clear()

    print("    [中断] 时钟节拍 -> 想触发调度")
    winner = sched2.maybe_switch("时钟中断")
    for line in sched2.log: print(line)
    sched2.log.clear()
    if winner is None:
        print("    ✓ 切换被推迟 —— 主任务继续执行")

    print(f"    [主任务] sched_setscheduler(pid={newtask2.pid}, SCHED_FIFO)")
    newtask2.policy = "SCHED_FIFO"      # 现在才设置, 但任务还没跑过

    sched2.unlock()
    for line in sched2.log: print(line)
    sched2.log.clear()

    print("    [调度器] 现在才切换到新任务")
    print(f"    ✓ '{newtask2.name}' 以 {newtask2.policy} 策略启动")

    print("""
    【核心认识】

      sched_lock() 不是"互斥锁" —— 它是【禁止任务切换】。
      中断仍然会发生(中断处理照常运行), 只是中断返回时不会切任务。

        保护的是: "创建 + 配置" 这段代码的【原子性】

      PX4 用它保证: 新任务第一次被调度时, 调度策略【一定】已经正确设置。
""")

    return sched2


# ============================================================================
#  实验 2: PX4 优先级流水线
# ============================================================================
def exp2_priority_pipeline():
    sep("实验 2: PX4 优先级流水线")

    # 真实的常量 (SCHED_PRIORITY_MAX = 255, PX4_WQ_HP_BASE = 240)
    MAX = 255
    HP_BASE = MAX - 15          # 240

    PRIORITIES = [
        ("FAST_DRIVER",       MAX - 0,         "快速驱动"),
        ("HPWORK",            249,             "高位工作队列"),
        ("ACTUATOR_OUTPUTS",  HP_BASE - 3,     "执行器输出"),
        ("ATTITUDE_CONTROL",  HP_BASE - 4,     "姿态控制"),
        ("ESTIMATOR",         HP_BASE - 5,     "状态估计"),
        ("LOG_WATCHDOG",      HP_BASE - 6,     "日志看门狗"),
        ("POSITION_CONTROL",  HP_BASE - 7,     "位置控制"),
        ("LOG_CAPTURE",       HP_BASE - 10,    "日志采集"),
        ("SLOW_DRIVER",       HP_BASE - 35,    "慢速驱动"),
        ("LPWORK",            50,              "低位工作队列"),
        ("IDLE",              0,               "空闲"),
    ]

    print("  === PX4 优先级体系 (真实数值) ===")
    print(f"  SCHED_PRIORITY_MAX = {MAX},  PX4_WQ_HP_BASE = {HP_BASE}\n")
    print(f"  {'常量':<22} {'值':>5}  {'相对HP':>7}  {'说明'}")
    print("  " + "-" * 62)
    for name, val, desc in PRIORITIES:
        rel = f"{val - HP_BASE:+d}" if val > 60 else "—"
        print(f"  {name:<22} {val:>5}  {rel:>7}  {desc}")

    print("""
  === 控制流水线的执行顺序 ===

  假设某一时刻, 传感器数据更新, 以下任务同时变为就绪。
  谁先跑?  —— 按优先级。

     时刻: 传感器中断返回
          │
          ▼
   ┌─────────────────────────────────────────────────────┐
   │ [237] ACTUATOR_OUTPUTS   ← 先把上一轮力矩发出去      │
   ├─────────────────────────────────────────────────────┤
   │ [236] ATTITUDE_CONTROL   ← 用新传感器数据算姿态指令   │
   ├─────────────────────────────────────────────────────┤
   │ [235] ESTIMATOR          ← 更新状态估计              │
   ├─────────────────────────────────────────────────────┤
   │ [233] POSITION_CONTROL   ← 最后跑 (它本来就慢)       │
   └─────────────────────────────────────────────────────┘
          │
          ▼
     等待下一次传感器更新
""")

    # ---- 模拟调度顺序 ----
    print("  " + "-" * 72)
    print("  模拟: 4 个任务同时就绪, 观察执行顺序")
    print("  " + "-" * 72)

    ready = []
    print(f"\n  就绪队列 (同时变为可运行):")
    for name in ["POSITION_CONTROL", "ESTIMATOR", "ATTITUDE_CONTROL",
                 "ACTUATOR_OUTPUTS"]:
        val = next(v for n, v, _ in PRIORITIES if n == name)
        ready.append((name, val))
        print(f"    {name:<20} 优先级 {val}")

    print(f"\n  调度顺序 (SCHED_FIFO, 高优先级先跑):")
    ready.sort(key=lambda x: -x[1])
    for i, (name, val) in enumerate(ready, 1):
        print(f"    {i}. [{val}] {name}")

    print("""
  【为什么是这个顺序? —— 看 PX4 源码里的注释】

    ACTUATOR_OUTPUTS (237):
      "Actuator outputs should run as soon as the rate controller publishes
       the actuator controls topic"
      -> 速率控制器一发布力矩, 执行器就要尽快输出。晚了电机就抖。

    ATTITUDE_CONTROL (236):
      "Attitude controllers typically are in a blocking wait on driver data
       they should be the first to run on an update, using the current sensor
       data and the *previous* attitude reference from the position controller"
      -> 用【当前】传感器 + 【上一次】位置指令。

    POSITION_CONTROL (233):
      "Position controllers typically are in a blocking wait on estimator data
       so when new sensor data is available they will run last."
      -> 位置环本来就慢 (50Hz), 排在最后没问题。

  ★ 核心设计原则: 优先级按【谁在等谁】排, 不是按【数据流顺序】排。
    姿态控制虽然"逻辑上"在估计器之后, 但优先级更高 ——
    因为它等的是【传感器】, 而位置控制等的是【估计器】。
""")


# ============================================================================
#  实验 3: 栈大小计算器
# ============================================================================
def exp3_stack_size():
    sep("实验 3: 栈大小计算器与溢出检测")

    # PX4_STACK_ADJUSTED 的真实定义
    # platforms/common/include/px4_platform_common/posix.h
    PLATFORMS = {
        "ARM 32-bit (真实飞控)": {"ptr_size": 4, "overhead": 0},
        "x86_64 (SITL 仿真)":    {"ptr_size": 8, "overhead": 1024 * 24},
    }

    print("""
  PX4 的栈大小宏 (platforms/common/include/px4_platform_common/posix.h):

      // The stack size is intended for 32-bit architectures; therefore
      // we often run out of stack space when pointers are larger than 4 bytes.
      // Double the stack size on posix when we're on a 64-bit architecture.
      // Most full-scale OS use 1-4K of memory from the stack themselves.
      #define PX4_STACK_ADJUSTED(_s) (_s * (__SIZEOF_POINTER__ >> 2) + PX4_STACK_OVERHEAD)

  PX4_STACK_OVERHEAD:
      NuttX (嵌入式):  0
      POSIX (仿真):    1024 * 24 = 24576 字节
""")

    print("  === 实测各任务的栈大小 ===\n")

    TASKS = [
        ("mavlink_shell", 1800),
        ("attitude_control", 1800),
        ("commander", 3250),
        ("logger", 3700),
    ]

    hdr = f"  {'任务':<20} {'源码写法':>12}"
    for pname in PLATFORMS:
        hdr += f" {pname:>22}"
    print(hdr)
    print("  " + "-" * 80)

    for name, size in TASKS:
        line = f"  {name:<20} {size:>12}"
        for pname, cfg in PLATFORMS.items():
            adj = size * (cfg["ptr_size"] >> 2) + cfg["overhead"]
            line += f" {adj:>22}"
        print(line)

    print("""
  【解读】

    ARM 32 位:  _s * (4>>2) + 0      = _s * 1 + 0      -> 原值
    x86_64:     _s * (8>>2) + 24576  = _s * 2 + 24576  -> 翻倍 + 24KB

    为什么 64 位要翻倍?
      栈上的指针和 long 变成 8 字节:
        int     a;   -> 32位:4   64位:4   (不变)
        long    b;   -> 32位:4   64位:8   <- 翻倍
        void   *p;   -> 32位:4   64位:8   <- 翻倍
      同样的代码, 64 位下用更多栈。

    为什么再加 24KB?
      Linux 的 pthread 有额外的内核栈和 guard page 开销。
""")

    # ---- 栈染色演示 ----
    print("  " + "-" * 72)
    print("  === CONFIG_STACK_COLORATION: 怎么知道栈用了多少 ===")
    print("  " + "-" * 72)

    print("""
  原理: 创建任务时, 用固定魔数(如 0xDEADBEEF)填充【整个栈区】。
        运行一段时间后, 检查哪些区域【没被覆盖】—— 那就是没用到的栈。

  可视化 (一个 1800 字节的栈):

    ┌─────────────────────────────────────────┐ ← 栈底 (高地址)
    │ DEADBEEF DEADBEEF DEADBEEF DEADBEEF ... │  未被使用
    │ DEADBEEF DEADBEEF DEADBEEF DEADBEEF ... │
    ├─────────────────────────────────────────┤ ← 使用边界 (高水位线)
    │ 实际数据 实际数据 实际数据 实际数据 ... │  已使用
    │ 实际数据 实际数据 实际数据 实际数据 ... │
    └─────────────────────────────────────────┘ ← 栈顶 (低地址, SP)
""")

    # 模拟栈染色
    import random
    random.seed(42)

    STACK_SIZE = 1800          # 字节
    WORD = 4                   # 32 位字
    NWORDS = STACK_SIZE // WORD
    MAGIC = 0xDEADBEEF

    stack = [MAGIC] * NWORDS   # 染色

    # 模拟: 任务运行, 使用了一部分栈
    peak_usage_words = int(NWORDS * 0.42)     # 假设用了 42%
    for i in range(peak_usage_words):
        stack[i] = random.randint(0, 0xFFFFFFFF)   # 写入真实数据

    # 检测: 从栈顶(索引0)往上数, 找第一个还是 MAGIC 的位置
    used_words = 0
    for w in stack:
        if w == MAGIC:
            break
        used_words += 1

    used_bytes = used_words * WORD
    free_bytes = STACK_SIZE - used_bytes
    usage_pct = used_bytes / STACK_SIZE * 100

    print(f"  === 实测一个 {STACK_SIZE} 字节的栈 ===\n")
    print(f"    栈总大小     : {STACK_SIZE:>6} 字节")
    print(f"    已使用(高水位): {used_bytes:>6} 字节  ({usage_pct:.1f}%)")
    print(f"    剩余         : {free_bytes:>6} 字节  ({100-usage_pct:.1f}%)")

    # 安全评估
    print(f"\n  === 安全评估 ===\n")
    MARGIN_WARN = 0.25         # 剩余 < 25% 警告
    MARGIN_CRIT = 0.10         # 剩余 < 10% 危险

    free_ratio = free_bytes / STACK_SIZE
    if free_ratio < MARGIN_CRIT:
        verdict = "🔴 危险 —— 随时可能溢出!"
    elif free_ratio < MARGIN_WARN:
        verdict = "🟡 警告 —— 余量不足, 建议增大栈"
    else:
        verdict = "🟢 正常"

    print(f"    剩余比例 = {free_ratio*100:.1f}%")
    print(f"    判定     = {verdict}")
    print(f"\n    经验阈值: 剩余 < 25% 警告, < 10% 危险")

    print("""
  【栈溢出的真实危险】

    栈溢出【不会立刻崩溃】(某些配置下), 而是:

      ┌──────────────┐  ← 栈底
      │  任务 A 的栈  │
      ├──────────────┤
      │  任务 B 的栈  │  ← A 溢出到这里, 污染了 B!
      ├──────────────┤
      │    堆        │  ← 或写到堆里, 破坏 malloc 元数据
      └──────────────┘

    症状: 随机崩溃、数据错乱、难以复现。

    ==> 这就是 CONFIG_STACK_COLORATION=y 的价值:
        让你能在崩溃【之前】发现栈快满了。

    在 NSH 里查看:
        nsh> ps        # 看任务列表
        nsh> free      # 看内存
        nsh> tasks     # PX4 的命令
""")

    return used_bytes


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  卷 6 第 3 章 · 任务与调度")
    print("=" * 76)

    exp1_sched_lock()
    exp2_priority_pipeline()
    exp3_stack_size()

    sep("本章小结")
    print("""
  1. sched_lock() 保证 "创建 + 配置" 的原子性
     防止新任务带着默认调度策略启动

  2. clearenv() 释放环境变量内存 —— 嵌入式特有优化

  3. task_create() 是 NuttX 原生 API (只接受优先级)
     sched_setscheduler() 是 POSIX 补充 (可指定策略)
     PX4 两者都用

  4. PX4 用 SCHED_FIFO (不是 RR) —— 确定性更强

  5. 优先级设计原则: 按 "谁在等谁" 排, 不是按数据流顺序
     ACTUATOR_OUTPUTS(237) > ATTITUDE_CONTROL(236) > ESTIMATOR(235)
     > POSITION_CONTROL(233)

  6. PX4_STACK_ADJUSTED 在 64 位下【翻倍】+ 24KB
     因为指针和 long 变成 8 字节

  7. NuttX 只继承前 3 个 fd —— I/O 重定向要特殊处理

  8. 栈溢出会静默破坏其他任务
     用 CONFIG_STACK_COLORATION 监控

  下一课: 卷 6 第 4 章
""")
