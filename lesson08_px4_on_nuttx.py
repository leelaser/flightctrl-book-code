"""
============================================================================
 卷 6 第 7 章 · PX4 如何用 NuttX
 配套实验代码
============================================================================

 运行:  python lesson08_px4_on_nuttx.py

 五个实验:
   1. 启动时序           —— 从复位到可起飞, 标注每一步是 NuttX 还是 PX4
   2. 内存布局           —— 四块不连续的 RAM 组成一个堆
   3. 模块生命周期        —— start / stop / status 的状态机
   4. 特性对照           —— PX4 用了 NuttX 的什么、绕过了什么
   5. 全课程知识地图      —— 8 课之间的关系

 对照的真实源码 (PX4 1.15.4):
   sched/init/nx_start.c               NuttX 内核启动
   sched/init/nx_bringup.c             board_late_initialize + init 线程
   apps/nshlib/nsh_init.c              nsh_initialize → boardctl(BOARDIOC_INIT)
   boards/px4/fmu-v6x/src/init.c       PX4 的 board_app_initialize
   boards/px4/fmu-v6x/nuttx-config/scripts/script.ld   链接脚本
   arch/arm/src/stm32h7/stm32_allocateheap.c           四块堆
============================================================================
"""

import math


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


def w(s):
    """终端显示宽度（中文算 2 列）"""
    return sum(2 if ord(c) > 0x1100 and (
        0x4E00 <= ord(c) <= 0x9FFF or 0x3000 <= ord(c) <= 0x303F or
        0xFF00 <= ord(c) <= 0xFF60 or 0x2500 <= ord(c) <= 0x257F or
        0x2190 <= ord(c) <= 0x21FF or 0x2600 <= ord(c) <= 0x27BF) else 1
        for c in s)


def pad(s, width, align="left"):
    d = max(0, width - w(s))
    return s + " " * d if align == "left" else " " * d + s


# ============================================================================
#  实验 1: 启动时序
# ============================================================================
def exp1_boot_sequence():
    sep("实验 1: 启动时序 —— 从复位到可起飞")

    print("""
  PX4 的启动是一条【NuttX 和 PX4 交替】的长链。下面逐步走一遍。

  图例:  [N] = NuttX 的代码    [P] = PX4 的代码
""")

    STEPS = [
        # (阶段, 执行者, 动作, 文件:行)
        ("硬件", "—", "复位 → 从 0x08000000 取初始 SP 和 PC", "向量表"),
        ("Bootloader", "—", "PX4 bootloader 检查固件、跳转到 app", "bootloader_main.c"),
        ("硬件", "—", "跳到 0x08020000 (固件起始)", "script.ld:113"),

        ("NuttX 启动", "N", "__start → 清 .bss / 复制 .data", "stm32_start.c"),
        ("", "N", "stm32_clockconfig() 配置时钟树", "stm32_start.c"),
        ("", "N", "stm32_fpuconfig() 使能 FPU", "stm32_start.c"),
        ("", "N", "stm32_lowsetup() 初始化串口 (为了能打印)", "stm32_start.c"),
        ("", "N", "nx_start()  ← 内核正式开始", "nx_start.c:332"),

        ("内核子系统", "N", "nxsem_initialize()      信号量", "nx_start.c:452"),
        ("", "N", "kmm_initialize()        堆管理", "nx_start.c:471"),
        ("", "N", "iob_initialize()        I/O 缓冲", "nx_start.c:500"),
        ("", "N", "创建 IDLE 任务 (每 CPU)", "nx_start.c:516"),
        ("", "N", "sched_initialize()      调度器", "nx_start.c:568"),
        ("", "N", "fs_initialize()         文件系统 + inode 树", "nx_start.c:585"),
        ("", "N", "irq_initialize()        g_irqvector[] 填默认值", "nx_start.c:589"),
        ("", "N", "nxsig_initialize()      信号系统", "nx_start.c:616"),
        ("", "N", "mq_initialize()         消息队列", "nx_start.c:626"),
        ("", "N", "up_initialize()         芯片相关 (时钟中断等)", "nx_start.c:658"),
        ("", "N", "drivers_initialize()    /dev/null /dev/zero /dev/console", "nx_start.c:660"),
        ("", "N", "board_early_initialize()", "nx_start.c:667"),

        ("创建 init", "N", "nx_bringup()", "nx_bringup.c:446"),
        ("", "N", "nxtask_create(\"init\", ..., nsh_main, argv)",
         "CONFIG_INIT_ENTRYPOINT=\"nsh_main\""),

        ("NSH 启动", "N", "nsh_main()  ← init 任务的入口", "nsh_main.c"),
        ("", "N", "nsh_initialize()", "nsh_init.c:60"),
        ("", "N", "nsh_romfsetc()   挂载 ROMFS 到 /etc", "nsh_init.c:78"),
        ("", "N", "boardctl(BOARDIOC_INIT)", "nsh_init.c:81"),

        ("PX4 板级初始化", "P", "board_app_initialize()", "boards/px4/fmu-v6x/src/init.c:209"),
        ("", "P", "px4_platform_init()   ★ PX4 的一切从这里开始", "init.c:222"),
        ("", "P", "px4_platform_configure()  按 manifest 配置硬件", "init.c"),
        ("", "P", "board_determine_hw_info()  识别硬件版本", "init.c"),
        ("", "P", "stm32_spiinitialize()", "init.c"),
        ("", "P", "board_dma_alloc_init()  5KB DMA 池", "init.c"),
        ("", "P", "hrt_call_every(serial_dma_call, 1000, 1000, ...)",
         "init.c  ← 1ms 串口兜底"),
        ("", "P", "drv_led_start()  →  /dev/led0 注册", "init.c"),
        ("", "P", "stm32_sdio_initialize()  SD 卡", "init.c"),

        ("执行脚本", "N", "nsh_consolemain()  进入 NSH", "nsh_consolemain.c"),
        ("", "N", "执行 /etc/init.d/rcS", "ROMFS/px4fmu_common/init.d/rcS"),
        ("", "P", "rcS 里: px4_init / 各种 start 命令", "rcS"),
        ("", "P", "rc.board_sensors  启动传感器驱动", "rc.board_sensors"),
        ("", "P", "rc.autostart  启动姿态/位置控制器", "rc.autostart"),
        ("就绪", "P", "commander 加载参数, 等待解锁", "src/modules/commander/"),
    ]

    print(f"  {pad('阶段',18)} {pad('执行者',8)} {pad('动作',52)} 位置")
    print("  " + "─" * 118)
    last = None
    for stage, who, act, loc in STEPS:
        s = stage if stage else ""
        if s and s != last:
            print("  " + "─" * 118)
            last = s
        mark = {"N": "[N]", "P": "[P]", "—": "   "}[who]
        print(f"  {pad(s,18)} {pad(mark,8)} {pad(act,52)} {loc}")

    print("""
  ────────────────────────────────────────────────────────────────────
  【三个关键交接点】

     ① NuttX → PX4 的入口是 board_app_initialize()
        触发路径: nsh_initialize() → boardctl(BOARDIOC_INIT)
        而不是 board_late_initialize()!

        PX4 用的是 CONFIG_NSH_ARCHINIT=y, 不是 CONFIG_BOARD_LATE_INITIALIZE
        ─────────────────────────────────────────────────────────────
        CONFIG_BOARD_LATE_INITIALIZE:
            nx_bringup() → board_late_initialize()   ← 内核启动阶段
        CONFIG_NSH_ARCHINIT:
            nsh_initialize() → boardctl(BOARDIOC_INIT) → board_app_initialize()
                                                     ← NSH 启动阶段
        ─────────────────────────────────────────────────────────────

        为什么选后者? 因为 NSH 已经起来了, 可以打日志、可以用文件系统。

     ② init 任务跑的就是 nsh_main
        CONFIG_INIT_ENTRYPOINT="nsh_main"
        → PX4 的"init 进程"就是这个 shell
        → 没有独立的 init 程序, 启动脚本就是普通的 shell 脚本

     ③ ROMFS 先挂载, 再执行板级初始化
        nsh_initialize() 里的顺序:
            nsh_romfsetc()            ← 先挂载 /etc
            boardctl(BOARDIOC_INIT)   ← 再做板级初始化
        → 所以 board_app_initialize 里可以读 /etc 下的文件
""")


# ============================================================================
#  实验 2: 内存布局
# ============================================================================
def exp2_memory():
    sep("实验 2: 内存布局 —— 四块不连续的 RAM 组成一个堆")

    print("""
  STM32H753II 的 RAM 不是一块连续区域 —— 它被切成好几块,
  分布在不同电源域、不同总线上。

  PX4 的做法: 【把它们全部合成一个堆】。

  📁 boards/px4/fmu-v6x/nuttx-config/nsh/defconfig:136

      CONFIG_MM_REGIONS=4
""")

    # ---- Flash ----
    print("  " + "-" * 72)
    print("  Flash 布局")
    print("  " + "-" * 72)
    print("""
     📁 boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:113

         FLASH (rx) : ORIGIN = 0x08020000, LENGTH = 1920K

     ⚠️ 注意起始地址是 0x08020000 而【不是】0x08000000!

        0x08000000  ┌──────────────────────────┐
                    │  PX4 Bootloader  128 KB   │  ← 独立编译, 独立烧录
        0x08020000  ├──────────────────────────┤
                    │  固件 .text / .rodata     │
                    │  + .data 的初始值         │  1920 KB
                    │  + .init_array (C++ 构造) │
                    │  + .ARM.exidx             │
        0x0A000000  └──────────────────────────┘
                       (2048 KB 总 Flash)

     为什么留 128KB 给 bootloader?
       · 固件升级时的安全网: 升级失败可以回滚
       · bootloader 可以校验固件完整性
       · 可以通过串口/USB 强制进入刷机模式
""")

    # ---- RAM ----
    print("  " + "-" * 72)
    print("  RAM 分布")
    print("  " + "-" * 72)

    REGIONS = [
        ("DTCM1", 0x20000000, 64, "Cortex-M7 紧耦合", "─┐"),
        ("DTCM2", 0x20010000, 64, "Cortex-M7 紧耦合", "─┴ 区域 4"),
        ("AXI SRAM", 0x24000000, 512, "D1 域, 所有主设备可访问", "★ 区域 1 主堆"),
        ("SRAM1", 0x30000000, 128, "D2 域 AHB", "─┐"),
        ("SRAM2", 0x30020000, 128, "D2 域 AHB", "├ 区域 2"),
        ("SRAM3", 0x30040000, 32, "D2 域 AHB", "─┘"),
        ("SRAM4", 0x38000000, 64, "D3 域", "区域 3"),
        ("BKPRAM", 0x38800000, 4, "备份域 (掉电不丢)", "不用作堆"),
    ]

    print(f"\n  {pad('区域',12)} {pad('起始地址',12)} {pad('大小',8)} {pad('总线/域',26)} 角色")
    print("  " + "─" * 96)
    for name, base, kb, bus, role in REGIONS:
        mark = " ★" if "★" in role else ""
        print(f"  {pad(name,12)} 0x{base:08X}   {pad(f'{kb} KB',8)} "
              f"{pad(bus,26)} {role.replace(' ★','')}{mark}")

    total = sum(kb for _, _, kb, _, r in REGIONS if "不用" not in r)
    print(f"\n  合计: {total} KB  ← 全部合成【一个】堆 (CONFIG_MM_REGIONS=4)")

    print("""
  ────────────────────────────────────────────────────────────────────
  【四块堆是怎么加的】

     📁 arch/arm/src/stm32h7/stm32_allocateheap.c

     区域 1 —— 由 up_allocate_heap() 设置 (FLAT 构建):

         #define HEAP_BASE  ((uintptr_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE)
         const uintptr_t g_idle_topstack = HEAP_BASE;

         *heap_start = (void *)g_idle_topstack;
         *heap_size  = SRAM_END - g_idle_topstack;

     区域 2/3/4 —— 由 arm_addregion() 追加:

         void arm_addregion(void)
         {
           unsigned mm_regions = 1;              /* 已有 1 个 */

           if (mm_regions < CONFIG_MM_REGIONS)
             {
               addregion(SRAM123_START, SRAM123_END - SRAM123_START, "SRAM1,2,3");
               mm_regions++;                      /* → 2 */
             }

         #ifdef HAVE_SRAM4
           if (mm_regions < CONFIG_MM_REGIONS)
             {
               addregion(SRAM4_HEAP_START, SRAM4_END - SRAM4_HEAP_START, "SRAM4");
               mm_regions++;                      /* → 3 */
             }
         #endif

         #ifdef HAVE_DTCM
           if (mm_regions < CONFIG_MM_REGIONS)
             {
               addregion(DTCM_START, DTCM_END - DTCM_START, "DTCM");
               mm_regions++;                      /* → 4, 满了 */
             }
         #endif
         }

     ★ CONFIG_MM_REGIONS 就是"最多加几块"的计数器。
       设成 4 就是"全都要"。

  ────────────────────────────────────────────────────────────────────
  【AXI SRAM 的详细划分】(主堆所在)

     0x24000000  ┌─────────────────────────┐
                 │  .data   已初始化变量     │
                 │  .bss    未初始化变量     │  ← 两者之和 = 固件的静态内存
     _ebss    ──►├─────────────────────────┤
                 │  IDLE 任务栈             │  CONFIG_IDLETHREAD_STACKSIZE = 750 B
     g_idle_topstack ──►├─────────────────┤
                 │                         │
                 │  ★ 堆 (malloc 从这里开始) │
                 │                         │
     0x24080000  └─────────────────────────┘

     💡 IDLE 栈只有 750 字节 —— 它够用是因为 IDLE 任务只做一件事:
        "当没有任何任务可运行时, 执行 WFI 指令等中断"。

     ⚠️ 堆的【起点是固定的】(接着 IDLE 栈), 但【总量】取决于
        .data/.bss 用了多少。所以:
            堆大小 = 512KB(AXI) - 静态内存 - 750B
                   + 288KB(SRAM123) + 64KB(SRAM4) + 128KB(DTCM)
""")

    # ---- 堆的可见性 ----
    print("  " + "-" * 72)
    print("  运行时可查")
    print("  " + "-" * 72)
    print("""
     nsh> free
                   total       used       free    largest
     Mem:        1000000     123456     876544     500000
     ...

     nsh> dmesg | grep "Kb of"
     512Kb of AXI SRAM at 0x2404a2b0        ← up_allocate_heap
     288Kb of SRAM1,2,3 at 0x30000000       ← addregion
     64Kb of SRAM4 at 0x38000000            ← addregion
     128Kb of DTCM at 0x20000000            ← addregion

     💡 这些打印来自 addregion() 里的 minfo():

            minfo("%" PRIu32 "Kb of %s at %p\\n", size / 1024, desc, (void *)start);

        【启动日志是理解内存布局最直接的入口】。
""")

    # ---- DMA 池 ----
    print("  " + "-" * 72)
    print("  DMA 专用池 —— 另一个独立的分配器")
    print("  " + "-" * 72)
    print("""
     📁 platforms/nuttx/src/px4/common/board_dma_alloc.c:73

         static uint8_t g_dma_heap[BOARD_DMA_ALLOC_POOL_SIZE] __attribute__((aligned(64)));

     📁 boards/px4/fmu-v6x/src/board_config.h:421

         #define BOARD_DMA_ALLOC_POOL_SIZE 5120      ← 5 KB

     ⚠️ 它【不在】上面那个大堆里 —— 是一个独立的静态数组, 用的是
        NuttX 的 GRAN 分配器 (固定粒度分配器), 不是通用 malloc。

     注释解释了为什么:

         /*
          * The DMA heap size constrains the total number of things that can be
          * ready to do DMA at a time.
          *
          * For example, FAT DMA depends on one sector-sized buffer per filesystem plus
          * one sector-sized buffer per file.
          *
          * We use a fundamental alignment / granule size of 64B; this is sufficient
          * to guarantee alignment for the largest STM32 DMA burst (16 beats x 32bits).
          */

     【算一下】5120 字节 / 512 字节每扇区 = 大约 10 个缓冲区

         → 这就是"最多同时有约 10 个文件在做 SD 卡 DMA"的上限

     💡 为什么单独搞一个池, 不用通用堆?
        · 通用堆的块可能不满足 DMA 的对齐要求
        · 通用堆可能碎片化, 导致大块 DMA 缓冲分配失败
        · GRAN 是固定粒度分配器 —— 分配/释放都是 O(1), 不会碎片
        · 上限明确, 可以监控 (dma_heap_peak_use)
""")


# ============================================================================
#  实验 3: 模块生命周期
# ============================================================================
def exp3_module_lifecycle():
    sep("实验 3: 模块生命周期 —— start / stop / status")

    print("""
  PX4 的"模块"统一用一套接口: 一个入口函数 + 三个子命令。

      nsh> <module> start     启动
      nsh> <module> stop      停止
      nsh> <module> status    查看状态
      nsh> <module> <自定义>   模块自己的命令
""")

    # ---- 状态机 ----
    print("  " + "-" * 72)
    print("  状态机")
    print("  " + "-" * 72)
    print("""
                      start
                        │
                        ▼
                ┌───────────────┐
                │  task_spawn() │  创建任务 / 提交到工作队列
                └───────┬───────┘
                        │
                        ▼
                ┌───────────────┐
                │  init()       │  注册设备 / 订阅 topic / 申请资源
                └───────┬───────┘
                        │
                 成功 ──┴── 失败
                  │         │
                  ▼         ▼
          ┌───────────┐  ┌──────────────┐
          │ _object   │  │ 返回错误     │
          │ 已存储    │  │ 清理并退出   │
          │ task_id>0 │  └──────────────┘
          └─────┬─────┘
                │
                ▼
          ┌───────────┐  ←──────────────┐
          │  运行中    │                 │
          │  run()    │                 │
          │  循环     │                 │
          └─────┬─────┘                 │
                │                       │
                │  status  → 打印状态    │
                │                       │
                │  stop                 │
                ▼                       │
          ┌───────────┐                 │
          │ _object   │                 │
          │ 置空      │                 │
          │ task 退出 │                 │
          └─────┬─────┘                 │
                │                       │
          再次 start                    │
                └───────────────────────┘
""")

    # ---- 三个必须实现的函数 ----
    print("  " + "-" * 72)
    print("  模块必须实现的三个静态函数")
    print("  " + "-" * 72)
    print("""
     📁 platforms/common/include/px4_platform_common/module.h

         template<class T>
         class ModuleBase
         {
         public:
             static int main(int argc, char *argv[])
             {
                 if (argc <= 1 || strcmp(argv[1], "help") == 0 || ...) {
                     return T::print_usage();
                 }

                 if (strcmp(argv[1], "start") == 0) {
                     return start_command_base(argc - 1, argv + 1);
                 }

                 if (strcmp(argv[1], "status") == 0) {
                     return status_command();
                 }

                 if (strcmp(argv[1], "stop") == 0) {
                     return stop_command();
                 }

                 lock_module();
                 int ret = T::custom_command(argc - 1, argv + 1);
                 unlock_module();
                 return ret;
             }

         protected:
             // ★ 子类必须实现这三个
             static int task_spawn(int argc, char *argv[]);
             static int custom_command(int argc, char *argv[]);
             static int print_usage(const char *reason = nullptr);

             static px4::atomic<T *> _object;
             static int _task_id;
             static constexpr int task_id_is_work_queue{-2};
         };

     ★ 注意 `start` / `stop` / `status` / `help` 是【基类统一处理】的,
       子类只需要写三个 static 函数。

     💡 这是一个 CRTP (奇异递归模板模式):
        ModuleBase<T> 里的 T::xxx 调用的是子类的实现。
        编译期分派, 零虚函数开销。
""")

    # ---- 状态演示 ----
    print("  " + "-" * 72)
    print("  演示: 一个模块的完整生命周期")
    print("  " + "-" * 72)

    class Module:
        def __init__(self, name, on_wq=False):
            self.name = name
            self.on_wq = on_wq
            self.state = "NOT_LOADED"
            self._object = None
            self._task_id = -1
            self.log = []

        def emit(self, what, detail):
            self.log.append((self.state, what, detail))

        def start(self, argv):
            self.emit("task_spawn", f"'{self.name}' {argv}")
            if self.on_wq:
                self._task_id = -2       # task_id_is_work_queue
                self.emit("task_spawn", f"提交到工作队列 (task_id = -2)")
            else:
                # px4_task_spawn_cmd(...)
                self._task_id = 1234
                self.emit("task_spawn", f"px4_task_spawn_cmd(...) → pid={self._task_id}")

            # wait_until_running()
            self.emit("wait", "wait_until_running()  轮询 _object, 每 2ms 一次")
            self._object = "instance"
            self.emit("init", "instance->init() 注册设备 / 订阅 topic")
            self.state = "RUNNING"
            self.emit("RUNNING", "run() 主循环开始")
            return 0

        def status(self):
            if self.state != "RUNNING":
                self.emit("status", "not running")
            else:
                self.emit("status", f"running, task_id={self._task_id}, "
                                    f"object={self._object}")
            return 0

        def stop(self):
            if self.state != "RUNNING":
                self.emit("stop", "not running")
                return -1
            self.emit("stop", "request_stop() → 通知主循环退出")
            self.emit("stop", "主循环检测到 should_exit() → 返回")
            self.emit("stop", "instance 析构, 释放资源")
            self._object = None
            self._task_id = -1
            self.state = "STOPPED"
            return 0

        def restart(self):
            if self.state == "RUNNING":
                self.emit("start", f"已经在运行 (task_id={self._task_id})")
                return 0
            return self.start([])

    for name, on_wq in [("sensors", False), ("wq:rate_ctrl 上的模块", True)]:
        print(f"\n  ═══ {name} ═══")
        m = Module(name, on_wq)
        print(f"\n  nsh> {name.split()[0]} start")
        m.start([])
        for st, what, detail in m.log:
            print(f"      [{pad(st,12)}] {pad(what,14)} {detail}")

        m.log.clear()
        print(f"\n  nsh> {name.split()[0]} start    (重复启动)")
        m.restart()
        for st, what, detail in m.log:
            print(f"      [{pad(st,12)}] {pad(what,14)} {detail}")

        m.log.clear()
        print(f"\n  nsh> {name.split()[0]} status")
        m.status()
        for st, what, detail in m.log:
            print(f"      [{pad(st,12)}] {pad(what,14)} {detail}")

        m.log.clear()
        print(f"\n  nsh> {name.split()[0]} stop")
        m.stop()
        for st, what, detail in m.log:
            print(f"      [{pad(st,12)}] {pad(what,14)} {detail}")

    print("""
  ────────────────────────────────────────────────────────────────────
  【两种 task_id 的语义】

     task_id > 0    独立的 NuttX 任务 (px4_task_spawn_cmd 创建)
                    → 停止靠请求主循环退出 + 任务自己 delete

     task_id == -2  task_id_is_work_queue
                    → 没有自己的任务! 跑在共享的工作队列线程上
                    → 停止只是不再往队列里排新工作

     💡 第二种更省资源 —— 一个工作队列线程可以跑几十个模块。
        代价是它们【共享栈】和【共享优先级】, 互相影响。

  【为什么要 wait_until_running(6000)】

     px4_task_spawn_cmd 只是创建了任务, 任务【还没开始跑】。
     start_command_base 必须等到 instance->init() 完成才能返回 ——
     否则脚本里的下一条命令可能用到还没就绪的资源。

     ⚠️ 最多等 6 秒 —— 超时就报错。
        这是一个"启动失败要吵, 不要静默继续"的设计。
""")


# ============================================================================
#  实验 4: 特性对照
# ============================================================================
def exp4_use_or_bypass():
    sep("实验 4: PX4 用了 NuttX 的什么、绕过了什么")

    USED = [
        ("任务与调度", "task_create / sched_setscheduler", "卷 6 第 3 章",
         "px4_task_spawn_cmd 的底座"),
        ("同步机制", "sem_t / pthread_mutex_t", "卷 6 第 4 章",
         "uORB 的节点锁、CDev 的 _lock"),
        ("设备驱动框架", "VFS + file_operations + inode", "卷 6 第 5 章",
         "CDev 跳板机制的全部基础"),
        ("中断框架", "irq_attach / g_irqvector / EXTI", "卷 6 第 6 章",
         "HRT、GPIO 中断、pwm_input"),
        ("文件系统", "ROMFS / FAT / PROCFS / BINFS", "卷 6 第 5 章",
         "启动脚本、SD 卡日志、/proc"),
        ("内建命令", "g_builtins[] + nsh", "卷 6 第 5 章",
         "所有模块的 nsh 命令入口"),
        ("消息队列", "mqueue", "卷 6 第 4 章",
         "少量使用（uORB 是主力）"),
        ("网络栈", "NuttX TCP/IP", "—",
         "MAVLink over UDP/TCP"),
        ("USB", "CDC-ACM / RNDIS", "—",
         "mavlink 的 USB 通道"),
        ("信号", "nxsig / SIGALRM", "卷 6 第 6 章",
         "定时和异常通知"),
    ]

    BYPASSED = [
        ("定时器", "HRT (1 MHz 硬件定时器)",
         "NuttX tick 是 1 ms, 精度差 1000 倍",
         "卷 6 第 6 章"),
        ("工作队列", "PX4 WorkQueue",
         "NuttX work_queue 的 delay 单位是 tick, 且只有 2 个固定队列",
         "卷 6 第 6 章"),
        ("IPC", "uORB",
         "mqueue 需要拷贝、性能不够; uORB 是共享内存 + 无锁读",
         "第 5/6 课"),
        ("参数系统", "PX4 param",
         "需要掉电保存、版本迁移、MAVLink 传输 —— NuttX 没有对应设施",
         "—"),
        ("任务创建", "px4_task_spawn_cmd (包装)",
         "要统一加 clearenv / sched_lock / 调度策略设置",
         "卷 6 第 3 章"),
        ("设备框架", "CDev (C++ 包装)",
         "想要虚函数而不是函数指针表",
         "卷 6 第 5 章"),
        ("构建模式", "FLAT (不用 protected/kernel)",
         "省去系统调用开销、可以直接共享内存",
         "本章"),
        ("内存管理", "统一大堆 + 专用 DMA 池",
         "需要跨 4 块不连续 RAM",
         "本章"),
    ]

    print(f"\n  ┌─────────────────────────────────────────────────────────────────┐")
    print(f"  │  ① PX4 直接【使用】的 NuttX 设施                                │")
    print(f"  └─────────────────────────────────────────────────────────────────┘\n")
    print(f"  {pad('领域',18)} {pad('NuttX 设施',34)} {pad('课程',8)} 用途")
    print("  " + "─" * 104)
    for area, fac, lesson, use in USED:
        print(f"  {pad(area,18)} {pad(fac,34)} {pad(lesson,8)} {use}")

    print(f"\n  ┌─────────────────────────────────────────────────────────────────┐")
    print(f"  │  ② PX4 【绕过】NuttX 自己造的轮子                                │")
    print(f"  └─────────────────────────────────────────────────────────────────┘\n")
    print(f"  {pad('领域',12)} {pad('PX4 的方案',24)} {pad('为什么',48)} 课程")
    print("  " + "─" * 104)
    for area, sol, why, lesson in BYPASSED:
        print(f"  {pad(area,12)} {pad(sol,24)} {pad(why,48)} {lesson}")

    print("""
  ────────────────────────────────────────────────────────────────────
  【规律: PX4 在什么情况下会绕过 NuttX】

     观察上表, "绕过"集中在三类:

       ① NuttX 的【精度不够】
          · NuttX tick = 1 ms, PX4 需要 µs   → HRT
          · work_queue delay 单位是 tick      → PX4 WorkQueue

       ② NuttX 的【性能不够】
          · mqueue 要拷贝                    → uORB (共享内存)
          · 通用堆可能碎片、对齐不保证        → DMA 专用池

       ③ NuttX 的【抽象不够】
          · 函数指针表 → 想要虚函数          → CDev
          · 没有参数持久化设施               → PX4 param

     ★ 而 PX4 【不重造】的都是"够用且稳定"的基础设施:
        调度器、信号量、VFS、中断框架、文件系统、网络栈。

  【一个反直觉的观察】

     卷 6 第 4 章我们发现: PX4 的 pthread_mutex 没有优先级继承。
     而 px4_sem_t 有。

     → PX4 是【有意地】在两类场景用不同原语:
        · 实时关键路径 (uORB 节点、CDev、SPI 总线) → px4_sem_t (有 PI)
        · 非实时路径 (mavlink、EKF2 模块管理)      → pthread_mutex

     ★ 这不是随便选的, 而是"知道自己在用什么"。

  【另一条规律: 能用 NuttX 就用, 只在关键时刻自己造】

     数一下: 上表【使用】10 项, 【绕过】8 项。
     而且绕过的 8 项里有 3 项 (任务创建/设备框架/内存管理)
     其实是【包装】而不是【替换】—— 底层还是 NuttX 的。
""")


# ============================================================================
#  实验 5: 全课程知识地图
# ============================================================================
def exp5_course_map():
    sep("实验 5: 全课程知识地图")

    print("""
  八节课讲完, 这是它们之间的关系。
""")

    print("""
                          ┌────────────────────────────────────┐
                          │  NuttX 是一个兼容 POSIX 的 RTOS     │
                          │  卷 6 第 1 章: 定位与架构                 │
                          └──────────────┬─────────────────────┘
                                         │
                     ┌───────────────────┼───────────────────┐
                     │                   │                   │
                     ▼                   ▼                   ▼
          ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
          │  卷 4 第 1 章          │ │  卷 6 第 3 章          │ │  卷 6 第 2 章          │
          │  为什么需要 RTOS   │ │  任务与调度        │ │  写一个应用       │
          │                  │ │                  │ │                  │
          │  · 裸机 vs RTOS   │ │  · task_create   │ │  · 模块结构       │
          │  · 实时性指标      │ │  · 优先级体系      │ │  · builtin 注册   │
          │  · 抖动 vs 偏差    │ │  · SCHED_FIFO    │ │  · C/C++ 混合     │
          └──────────────────┘ └────────┬─────────┘ └──────────────────┘
                                        │
                     ┌──────────────────┼──────────────────┐
                     │                  │                  │
                     ▼                  ▼                  ▼
          ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
          │  卷 6 第 4 章          │ │  卷 6 第 5 章          │ │  卷 6 第 6 章          │
          │  同步机制          │ │  设备驱动+文件系统  │ │  中断与下半部      │
          │                  │ │                  │ │                  │
          │  · 信号量内部     │ │  · VFS 三层结构   │ │  · irq_attach    │
          │  · 优先级继承 ★    │ │  · register_driver│ │  · 上下半部       │
          │  · 互斥锁=信号量   │ │  · CDev 跳板      │ │  · HRT ★          │
          └────────┬─────────┘ └────────┬─────────┘ └────────┬─────────┘
                   │                    │                    │
                   └────────────────────┼────────────────────┘
                                        │
                                        ▼
                          ┌────────────────────────────────────┐
                          │  本章 (本章)                      │
                          │  PX4 如何用 NuttX                   │
                          │                                    │
                          │  · 启动流程 (NuttX ↔ PX4 交接)      │
                          │  · 内存布局 (4 块堆)                │
                          │  · 模块生命周期                     │
                          │  · 用了什么 / 绕过了什么 ★           │
                          └────────────────────────────────────┘
""")

    print("  " + "-" * 72)
    print("  横贯全课的四条主线")
    print("  " + "-" * 72)
    print("""
   ┌────────────────────────────────────────────────────────────────┐
   │ 主线 ①  【实时性是被设计出来的, 不是测出来的】                    │
   │                                                                │
   │   卷 4 第 1 章: 延迟不可能为 0, 只能量化                              │
   │   卷 6 第 3 章: 优先级按"谁在等谁"排, 不是按数据流                     │
   │   卷 6 第 4 章: 优先级继承把"无界反转"变成"有界阻塞"                   │
   │   卷 6 第 5 章: 临界区里不调用外部代码 (close 先解锁再回调)             │
   │   卷 6 第 6 章: 时间戳必须在上半部取; HRT_INTERVAL_MAX 的双重约束      │
   │   本章: PX4 用线程优先级代替中断优先级                          │
   └────────────────────────────────────────────────────────────────┘

   ┌────────────────────────────────────────────────────────────────┐
   │ 主线 ②  【每层抽象都有代价, 知道代价才能正确选择】                 │
   │                                                                │
   │   卷 6 第 3 章: 优先级反转 —— 一个任务间接受制于另一个                  │
   │   卷 6 第 4 章: 信号量可重入性差 (不如互斥锁) → 但飞控用信号量拿 PI     │
   │   卷 6 第 5 章: VFS 统一接口的代价 = read 被用成了 write               │
   │   卷 6 第 6 章: 上下半部的代价 = 时间戳要小心处理                       │
   │   本章: uORB 绕过 mqueue 的代价 = 自己实现同步                 │
   └────────────────────────────────────────────────────────────────┘

   ┌────────────────────────────────────────────────────────────────┐
   │ 主线 ③  【源码是最诚实的老师 —— 包括它的不完美】                   │
   │                                                                │
   │   卷 6 第 4 章: nxsem_release_holder() 结尾的 TODO                    │
   │           "How do we choose the holder to decrement it's counts?"│
   │   卷 6 第 4 章: nxsem_restore_baseprio() 里被注释掉的 DEBUGASSERT     │
   │           "DSA: sometimes crashes when Telnet calls i2c"        │
   │   卷 6 第 5 章: 玩具 LED 驱动的 12 个问题                              │
   │   卷 6 第 6 章: cpuload.cpp 里一段从未被编译过的代码 (语法都不对)       │
   │   第 5/7 课: 文档承诺但未实现的功能 (EDEADLK 检测)                │
   └────────────────────────────────────────────────────────────────┘

   ┌────────────────────────────────────────────────────────────────┐
   │ 主线 ④  【PX4 和 ArduPilot 是两种哲学, 没有绝对的好坏】            │
   │                                                                │
   │              PX4                    ArduPilot                  │
   │   ──────────────────────────────────────────────────────       │
   │   架构    多任务 + 模块化            单主循环 + 少量线程          │
   │   设备    VFS + /dev + /obj          类方法直接调用              │
   │   同步    锁 + 优先级继承             尽量不用锁                 │
   │   IPC     uORB (共享内存)            类实例 + 缓冲区             │
   │   IMU     中断打时间戳 + 下半部       定时轮询                   │
   │   定时    HRT (µs)                   ChibiOS tick               │
   │                                                                │
   │   ✅ 理解差异比判断优劣重要                                      │
   └────────────────────────────────────────────────────────────────┘
""")

    print("  " + "-" * 72)
    print("  继续深入的方向")
    print("  " + "-" * 72)
    print("""
   ┌────────────────────────────────────────────────────────────────┐
   │ 还没讲的 NuttX 主题                                              │
   │                                                                │
   │   · 信号系统详解 (nxsig / 信号处理函数 / 与 ISR 的交互)            │
   │   · 消息队列 (mqueue) 与 POSIX 定时器                            │
   │   · 内存管理 (mm/ 目录: 堆、GRAN、页分配器)                       │
   │   · SMP 支持 (CONFIG_SMP, spinlock, CPU 亲和性)                 │
   │   · 电源管理 (PM 框架, DVFS)                                    │
   │   · 看门狗与故障恢复                                            │
   │   · 移植 NuttX 到新板子 (卷 6 第 9 章计划)                           │
   └────────────────────────────────────────────────────────────────┘

   ┌────────────────────────────────────────────────────────────────┐
   │ PX4 侧值得深入的                                                │
   │                                                                │
   │   · uORB 完整实现 (环形缓冲、订阅回调、多实例)                    │
   │   · 参数系统 (存储、迁移、MAVLink 同步)                          │
   │   · EKF2 的多实例与故障切换                                     │
   │   · 混控器 (Control Allocation) 与 VTOL 过渡                    │
   │   · logger 的写盘策略与掉电保护                                  │
   │   · MAVLink 的流控与多通道管理                                   │
   └────────────────────────────────────────────────────────────────┘
""")


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  卷 6 第 7 章 · PX4 如何用 NuttX")
    print("=" * 76)

    exp1_boot_sequence()
    exp2_memory()
    exp3_module_lifecycle()
    exp4_use_or_bypass()
    exp5_course_map()

    sep("本章小结")
    print("""
  1. 启动链是 NuttX 和 PX4 【交替】的长链
     NuttX: nx_start → 各子系统 → nx_bringup → nxtask_create(nsh_main)
     PX4:   从此处接手 (board_app_initialize → px4_platform_init)

  2. ★ 交接点是 board_app_initialize(), 不是 board_late_initialize()
     PX4 用 CONFIG_NSH_ARCHINIT=y → nsh_initialize() → boardctl(BOARDIOC_INIT)
     init 任务跑的就是 nsh_main, 启动脚本就是普通 shell 脚本

  3. ROMFS 先挂载, 再板级初始化 (nsh_init.c 里的顺序)

  4. 内存: 4 块不连续的 RAM 合成一个堆 (CONFIG_MM_REGIONS=4)
     AXI SRAM (主) + SRAM123 + SRAM4 + DTCM
     Flash 起始 0x08020000 —— 前 128K 留给 bootloader

  5. DMA 用独立的 5KB GRAN 池 (不是通用堆) —— 保证对齐、不碎片

  6. 模块用 ModuleBase<T> CRTP 统一生命周期
     start/stop/status 由基类处理, 子类只写三个 static 函数

  7. ★ PX4 绕过 NuttX 的规律:
     精度不够 (tick vs µs) → HRT
     性能不够 (mqueue 拷贝) → uORB
     抽象不够 (函数指针表)  → CDev
     其余全部复用

  8. PX4 用线程优先级代替中断优先级
     把优先级调度从硬件层下移到线程层

  9. 贯穿全课的四条主线:
     ① 实时性是设计出来的
     ② 每层抽象都有代价
     ③ 源码是最诚实的老师 (包括它的不完美)
     ④ PX4 和 ArduPilot 是两种哲学

  下一课: 卷 6 第 8 章
""")
