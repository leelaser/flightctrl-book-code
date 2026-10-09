"""
============================================================================
 卷 6 第 2 章 · 写一个 NuttX 应用
 配套实验代码
============================================================================

 运行:  python lesson03_nuttx_app.py

 三个实验:
   1. 复现 led 驱动的完整调用链   —— 从 nsh 敲命令到 GPIO 翻转
   2. 全局变量 vs 对象私有数据     —— 为什么 PX4 要传 this
   3. 命令 vs 任务                —— 演示阻塞 NSH 的后果

 素材来源:
   $SRC_NUTTX_SRC/stm32h7_fmu-master/src/drivers/led/led_main.c
   $SRC_NUTTX_SRC/stm32h7_fmu-master/src/modules/hello_nuttx/
============================================================================
"""


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


# ============================================================================
#  模拟 NuttX 的 VFS (虚拟文件系统)
# ============================================================================
class Inode:
    """NuttX 的 inode —— VFS 的核心数据结构"""
    def __init__(self, path):
        self.path = path
        self.fops = None          # 函数表 (struct file_operations)
        self.i_private = None     # ⭐ 私有数据指针! PX4 用它存 this
        self.mode = 0


class File:
    """打开的文件描述符"""
    def __init__(self, inode):
        self.f_inode = inode
        self.offset = 0


class VFS:
    """虚拟文件系统 —— 把 /dev/xxx 映射到驱动的 fops"""
    def __init__(self):
        self.nodes = {}

    def register_driver(self, path, fops, mode, priv):
        """对应 NuttX 的 register_driver()

        ⚠️ 四个参数的含义:
             path : 设备节点路径, 如 "/dev/led1"
             fops : 函数表 (open/close/read/write/...)
             mode : 权限, 如 0444
             priv : 【私有数据指针】—— 这是理解 PX4 的关键!
        """
        node = Inode(path)
        node.fops = fops
        node.i_private = priv      # ⭐ 原样保存
        node.mode = mode
        self.nodes[path] = node
        return 0

    def open(self, path, flags=0):
        if path not in self.nodes:
            raise OSError(-2, f"ENOENT: {path}")   # -ENOENT
        node = self.nodes[path]
        f = File(node)
        # 调用驱动的 open()
        if node.fops.get("open"):
            node.fops["open"](f)
        return f

    def read(self, f, buflen):
        """VFS 分发 —— 注意: 不直接调驱动函数, 而是走 fops 表"""
        fops = f.f_inode.fops
        if "read" not in fops:
            raise OSError(-38, "ENOSYS")
        return fops["read"](f, buflen)

    def write(self, f, data):
        fops = f.f_inode.fops
        if "write" not in fops:
            raise OSError(-38, "ENOSYS")
        return fops["write"](f, data)


# ============================================================================
#  实验 1: 复现 led 驱动的完整调用链
# ============================================================================
def exp1_led_call_chain():
    sep("实验 1: 复现 led 驱动的完整调用链")

    print("""
  目标: 看清楚 "nsh> led on" 这一条命令, 内部走了多少步。

  真实代码: Nuttx/stm32h7_fmu-master/src/drivers/led/led_main.c
""")

    # ---- 模拟硬件 ----
    class GPIO:
        def __init__(self):
            self.pins = {}

        def configgpio(self, cfg):
            self.pins[cfg] = 0
            print(f"      [硬件] stm32_configgpio({cfg})  -> 配置完成")

        def gpiowrite(self, cfg, val):
            self.pins[cfg] = val
            state = "亮" if val else "灭"
            print(f"      [硬件] stm32_gpiowrite({cfg}, {val})  -> LED {state}")

        def gpioread(self, cfg):
            return self.pins.get(cfg, 0)

    gpio = GPIO()
    GPIO_LED_BLUE = "PA1"

    # ---- 驱动实现 (完全对照 led_main.c) ----
    def led_open(filep):
        print("      [驱动] led_open() 被调用")
        gpio.configgpio(GPIO_LED_BLUE)
        return 0

    def led_close(filep):
        print("      [驱动] led_close() 被调用")
        return 0

    def led_read(filep, buflen):
        # ⚠️ 注意: 原版 led_main.c 里 read 和 write 代码完全一样
        #    这里保留原样, 后面实验会指出这个设计问题
        buf = filep._buf
        on = 1 if buf[0] > 0 else 0
        print(f"      [驱动] led_read()  buf[0]={buf[0]} (原版用 read 控制 LED)")
        gpio.gpiowrite(GPIO_LED_BLUE, on)
        return 0

    def led_write(filep, data):
        on = 1 if data[0] > 0 else 0
        print(f"      [驱动] led_write() data[0]={data[0]}")
        gpio.gpiowrite(GPIO_LED_BLUE, on)
        return len(data)

    # ---- 函数表 (对照 led_ops) ----
    led_ops = {
        "open":  led_open,
        "close": led_close,
        "read":  led_read,
        "write": led_write,
        "seek":  None,
        "ioctl": None,
        "poll":  None,
    }

    print("  【函数表 led_ops】")
    for k, v in led_ops.items():
        name = v.__name__ if v else "0 (未实现)"
        print(f"      {k:6s} : {name}")

    # ---- 应用入口 (对照 led_main) ----
    def led_register(vfs):
        print("      [注册] led_register() -> register_driver('/dev/led1', ...)")
        gpio.configgpio(GPIO_LED_BLUE)
        vfs.register_driver("/dev/led1", led_ops, 0o444, None)  # ← priv = NULL

    vfs = VFS()

    # 模拟 NSH 执行 "led on"
    print("\n  === 模拟: nsh> led on ===")
    print("  [步骤 1] NSH 查找符号 led_main 并调用")
    print("  [步骤 2] led_main(argc=2, argv=['led','on'])")
    print("     ↓ 调用 led_register()")
    led_register(vfs)

    print("     ↓ open('/dev/led1', O_RDONLY)")
    f = vfs.open("/dev/led1")
    print(f"     ✓ 返回 fd (f_ops={list(f.f_inode.fops.keys())[:4]}, "
          f"i_private={f.f_inode.i_private})")

    print("     ↓ read(fd, &on, 1)   on = 1")
    f._buf = [1]          # 模拟传入的 buffer
    vfs.read(f, 1)

    print("\n  === 调用链总结 ===")
    print("""
      nsh> led on
           │
           ▼
      NSH 查找符号 "led_main"
           │
           ▼
      led_main(argc, argv)
           │
           ├─ led_register()
           │      └─ register_driver("/dev/led1", &led_ops, 0444, NULL)
           │             └─ VFS 建 inode, 存下 fops 和 priv(NULL)
           │
           ├─ open("/dev/led1")
           │      └─ VFS 查 inode -> fops.open = led_open()
           │             └─ stm32_configgpio(PA1)
           │
           └─ read(fd, &on, 1)
                  └─ VFS -> fops.read = led_read()
                         └─ stm32_gpiowrite(PA1, 1)   ← LED 亮!
""")

    # ---- 指出原版的设计问题 ----
    print("  " + "-" * 72)
    print("  ⚠️ 原版 led_main.c 的一个设计问题")
    print("  " + "-" * 72)
    print("""
  看 led_read() 和 led_write() 的实现:

      static ssize_t led_read(file_t* filep, char* buffer, size_t len) {
          uint8_t on = *buffer > 0 ? 1 : 0;
          stm32_gpiowrite(GPIO_LED_BLUE, on);      // ← read 也在写硬件!
          return 0;
      }

      static ssize_t led_write(file_t* filep, const char* buffer, size_t len) {
          uint8_t on = *buffer > 0 ? 1 : 0;
          stm32_gpiowrite(GPIO_LED_BLUE, on);      // ← 和 read 一模一样!
          return 0;
      }

  问题: 【read 不应该改变硬件状态】—— 这违反 POSIX 语义。

  后果:
    · cat /dev/led1        -> 会意外点亮 LED
    · echo 1 > /dev/led1   -> 正常
    · 语义混乱, 维护困难

  正确写法:

      static ssize_t led_write(...) {
          stm32_gpiowrite(GPIO_LED_BLUE, buffer[0] > 0);
          return len;                                // 返回写入字节数
      }

      static ssize_t led_read(...) {
          buffer[0] = stm32_gpioread(GPIO_LED_BLUE); // 读回实际状态
          return 1;
      }
""")

    return vfs


# ============================================================================
#  实验 2: 全局变量 vs 对象私有数据
# ============================================================================
def exp2_priv_data():
    sep("实验 2: 全局变量 vs 对象私有数据 (priv 参数的作用)")

    print("""
  场景: 要支持两个 LED (PA1 蓝灯, PA2 红灯)
        两种驱动风格都能实现, 但优雅程度差很多。
""")

    # ---- 硬件 ----
    class GPIO:
        def __init__(self):
            self.pins = {}
            self.log = []
        def configgpio(self, cfg):
            self.pins[cfg] = 0
        def gpiowrite(self, cfg, val):
            self.pins[cfg] = val
            self.log.append(f"{cfg}={val}")

    # ================================================================
    #  风格 A: 全局变量 (对照 led_main.c)
    # ================================================================
    print("  " + "-" * 72)
    print("  风格 A: 全局变量 + priv = NULL  （led_main.c 的做法）")
    print("  " + "-" * 72)

    gpio_a = GPIO()

    # ⭐ 用一个【可变容器】模拟 C 语言里的文件级全局变量。
    #
    # 为什么不用普通变量 + global?
    #   本项目在这里反复踩了作用域的坑 (Python 的嵌套函数不能重新绑定
    #   外层函数的变量, 只能读)。容器则可以直接改内容, 效果等同于
    #   C 的全局变量 —— 这正是我们要演示的"全局状态"。
    #
    #     C 语言:  static char *_g_current_led;   (文件级, 所有函数共享)
    #     Python:  _g = {"led": None}             (同一份可变对象)
    _g = {"led": None}

    def a_open(filep):
        # 怎么知道该配置哪个 GPIO? 只能从【路径字符串】猜 —— 这就是问题所在
        path = filep.f_inode.path
        if path.endswith("led1"):
            _g["led"] = "PA1"
        elif path.endswith("led2"):
            _g["led"] = "PA2"
        gpio_a.configgpio(_g["led"])
        return 0

    def a_write(filep, data):
        # ⚠️ 这里依赖 _g["led"] 是"最近一次 open 的那个"
        gpio_a.gpiowrite(_g["led"], 1 if data[0] else 0)
        return len(data)

    a_fops = {"open": a_open, "write": a_write}

    vfs_a = VFS()
    vfs_a.register_driver("/dev/led1", a_fops, 0o666, None)   # priv = NULL
    vfs_a.register_driver("/dev/led2", a_fops, 0o666, None)   # 同一份 fops!

    print("\n  测试: 打开两个 LED, 然后【回头写第一个】")
    print("  ⚠️ 必须走 VFS 调用 (vfs_a.write), 不能直接调 a_write ——")
    print("     因为 vfs.write 才会设置 filep 的 buffer")

    gpio_a.log.clear()

    # 步骤 1: 打开 led1
    f1 = vfs_a.open("/dev/led1")
    print(f"\n    [1] open('/dev/led1') -> 全局状态 = {_g['led']!r}")

    # 步骤 2: 打开 led2 —— 全局变量被覆盖!
    f2 = vfs_a.open("/dev/led2")
    print(f"    [2] open('/dev/led2') -> 全局状态 = {_g['led']!r}"
          f"   <-- 被覆盖了!")

    # 步骤 3: 写 f1 —— 但全局变量已经是 PA2 了!
    print(f"    [3] write(f1, 1)  —— 期望操作 PA1")
    vfs_a.write(f1, [1])
    actual = gpio_a.log[-1]
    pin = actual.split('=')[0]
    print(f"        GPIO 记录: {actual}")
    if pin == "PA1":
        print(f"        ✓ 正确")
    else:
        print(f"        ✗ 错误! 实际操作了 {pin}")

    # 步骤 4: 再写 f2
    print(f"    [4] write(f2, 1)  —— 期望操作 PA2")
    vfs_a.write(f2, [1])
    print(f"        GPIO 记录: {gpio_a.log[-1]}   (这个碰巧对了)")

    print(f"""
    【为什么会错? 追溯调用链】

      open('/dev/led1')  ->  a_open()  ->  全局状态 = "PA1"
      open('/dev/led2')  ->  a_open()  ->  全局状态 = "PA2"    <- 覆盖了!
      write(f1, 1)       ->  a_write() ->  gpiowrite(全局状态)  <- 此时是 PA2
                                          ^^^^^^^^^^^^^^^^^
                                          a_write() 不知道 f1 对应哪个 LED!
""")

    print("""
    【问题 1】状态存全局变量 -> 多实例互相覆盖
      两个设备共用同一份 fops (函数指针一样),
      但状态 (哪个 GPIO) 存在【全局变量】里。
      全局变量只有一份, 后 open 的会覆盖先 open 的。

    【问题 2】只能靠【路径字符串】反推硬件
      驱动里写 if (path.endswith("led1")) ... 才能知道操作哪个 GPIO。
      这是极其脆弱的做法:
        · 改名 /dev/blue_led 就失效
        · 加第三个 LED 要改驱动代码
        · 硬件配置硬编码在驱动里, 无法复用

    【解决思路】每个设备要有【自己的状态】和一个"私有数据"指针 ——
                把 GPIO 配置和设备绑定在一起, 而不是靠路径猜。
""")

    # ================================================================
    #  风格 B: 对象 + priv = this  （PX4 的做法）
    # ================================================================
    print("  " + "-" * 72)
    print("  风格 B: 对象私有数据 + priv = this  （PX4 的做法）")
    print("  " + "-" * 72)

    gpio_b = GPIO()

    class LedCDev:
        """对应 PX4 的 cdev::CDev 子类"""
        def __init__(self, devname, gpiocfg):
            self._devname = devname
            self._gpiocfg = gpiocfg     # ⭐ 每个实例有自己的 GPIO
            self._on = False

        # ---- 虚函数实现 ----
        def open(self, filep):
            gpio_b.configgpio(self._gpiocfg)
            return 0

        def write(self, filep, data):
            self._on = bool(data[0])
            gpio_b.gpiowrite(self._gpiocfg, 1 if self._on else 0)
            return len(data)

        def read(self, filep, buflen):
            filep._buf = [1 if self._on else 0]
            return 1

    # ---- 桥接层: 完全复现 PX4 的 cdev_write() ----
    def cdev_write(filep, data):
        """中转函数 —— 对应 PX4 的 cdev_write()

        ⚠️ 关键: 从 inode 的 i_private 取回对象指针
        """
        cdev = filep.f_inode.i_private          # ⭐ 取出 this
        if cdev is None:
            return -19                          # -ENODEV
        return cdev.write(filep, data)          # 调虚函数

    def cdev_open(filep):
        cdev = filep.f_inode.i_private
        if cdev is None:
            return -19
        return cdev.open(filep)

    def cdev_read(filep, buflen):
        cdev = filep.f_inode.i_private
        if cdev is None:
            return -19
        return cdev.read(filep, buflen)

    # 一份 fops, 所有 LED 共用
    b_fops = {"open": cdev_open, "write": cdev_write, "read": cdev_read}

    vfs_b = VFS()
    led1 = LedCDev("/dev/led1", "PA1")
    led2 = LedCDev("/dev/led2", "PA2")

    # 注册时传各自的 this
    vfs_b.register_driver("/dev/led1", b_fops, 0o666, led1)   # ⭐ priv = led1
    vfs_b.register_driver("/dev/led2", b_fops, 0o666, led2)   # ⭐ priv = led2

    print("\n  注册信息:")
    for path, node in vfs_b.nodes.items():
        print(f"    {path}: i_private = {node.i_private._devname} "
              f"(GPIO={node.i_private._gpiocfg})")

    print("\n  测试: 同时打开两个 LED, 交替写")
    f1 = vfs_b.open("/dev/led1")
    f2 = vfs_b.open("/dev/led2")

    # 写 f1
    vfs_b.write(f1, [1])
    print(f"    write(f1, 1) -> GPIO 记录: {gpio_b.log[-1]}   ✓ 正确操作 PA1")

    # 写 f2 —— 关键测试: 会不会影响 f1?
    vfs_b.write(f2, [1])
    print(f"    write(f2, 1) -> GPIO 记录: {gpio_b.log[-1]}   ✓ 正确操作 PA2")

    # 再写 f1 —— 状态应该还在
    vfs_b.write(f1, [0])
    print(f"    write(f1, 0) -> GPIO 记录: {gpio_b.log[-1]}   ✓ 仍是 PA1!")

    print("\n  ✅ 两个实例互不干扰 —— 因为状态存在各自的对象里。")

    print("""
  【对比总结】

    ┌──────────────┬─────────────────┬──────────────────────┐
    │              │ 风格 A (全局)     │ 风格 B (对象)         │
    ├──────────────┼─────────────────┼──────────────────────┤
    │ 状态存哪      │ 全局变量          │ 对象成员              │
    │ priv 参数     │ NULL            │ (void*)this          │
    │ 多实例        │ ❌ 互相覆盖       │ ✅ 各自独立           │
    │ fops          │ 每个设备一份?     │ 所有设备共用一份       │
    │ 代码量        │ 少               │ 稍多 (要写类)         │
    │ 可维护性      │ 差               │ 好                   │
    └──────────────┴─────────────────┴──────────────────────┘

    PX4 选风格 B —— 因为一个飞控板上可能有:
      /dev/tfmini0  /dev/tfmini1     (两个同型号测距仪)
      /dev/imu0     /dev/imu1        (冗余 IMU)
      /dev/mag0     /dev/mag1        (冗余磁罗盘)

    没有对象私有数据, 这些都无法优雅实现。
""")

    return gpio_a, gpio_b


# ============================================================================
#  实验 3: 命令 vs 任务 —— 阻塞 NSH 的后果
# ============================================================================
def exp3_command_vs_task():
    sep("实验 3: 命令 vs 任务 —— 为什么控制环必须用任务")

    print("""
  场景: 写一个控制环, 每 1ms 跑一次。两种做法:

    做法 A: 在 xxx_main 里写 while(1)     —— 阻塞 NSH
    做法 B: 用 px4_task_spawn_cmd 创建任务  —— 正确
""")

    import time

    # ---- 模拟 NSH ----
    class NSH:
        def __init__(self):
            self.commands = {}
            self.blocked = False
            self.output = []

        def register(self, name, func):
            self.commands[name] = func

        def run(self, cmdline):
            """模拟 NSH 执行命令"""
            parts = cmdline.split()
            name = parts[0]
            if name not in self.commands:
                self.output.append(f"nsh: {name}: command not found")
                return

            self.output.append(f"nsh> {cmdline}")
            t0 = time.time()
            self.commands[name](parts)     # ← 同步调用!
            dt = (time.time() - t0) * 1000

            if dt > 100:
                self.output.append(f"      (命令耗时 {dt:.0f} ms —— NSH 被阻塞了!)")
            elif dt > 5:
                self.output.append(f"      (命令耗时 {dt:.1f} ms)")

    # ================================================================
    #  做法 A: 在 xxx_main 里死循环 (错误示范)
    # ================================================================
    print("  " + "-" * 72)
    print("  做法 A: 在 control_bad_main() 里写 while(1)  ❌")
    print("  " + "-" * 72)
    print("""
      int control_bad_main(int argc, char** argv) {
          while (1) {                       // ← 永不返回
              do_control();                 // 假设耗时 0.2ms
              usleep(800);                  // 睡 0.8ms
          }
          return 0;                         // 永远到不了
      }
""")

    nsh_a = NSH()
    RUN_MS = 300        # 模拟跑 300ms

    def control_bad_main(args):
        """错误示范: 在命令里死循环"""
        t_end = time.time() + RUN_MS / 1000.0
        iterations = 0
        while time.time() < t_end:
            iterations += 1
            time.sleep(0.001)       # 模拟 1ms 周期
        nsh_a.output.append(f"      control_bad 跑了 {iterations} 次, 然后才返回")

    def other_cmd(args):
        nsh_a.output.append("      other 命令执行了")

    nsh_a.register("control_bad", control_bad_main)
    nsh_a.register("other", other_cmd)

    print(f"  执行: nsh> control_bad   (会跑 {RUN_MS}ms)")
    t0 = time.time()
    nsh_a.run("control_bad")
    elapsed_a = (time.time() - t0) * 1000
    for line in nsh_a.output:
        print(f"    {line}")

    print(f"""
  ❌ 后果: nsh> 提示符在 {elapsed_a:.0f}ms 内【完全没有出现】。
           这期间用户无法输入任何其它命令。
           如果这是飞控, 你连"停止"命令都敲不进去。
""")

    # ================================================================
    #  做法 B: 创建独立任务 (正确做法)
    # ================================================================
    print("  " + "-" * 72)
    print("  做法 B: 用 px4_task_spawn_cmd() 创建任务  ✅")
    print("  " + "-" * 72)
    print("""
      int control_good_main(int argc, char** argv) {
          int pid = px4_task_spawn_cmd("control",       // 任务名
                                       SCHED_DEFAULT,   // 调度策略
                                       200,             // 优先级
                                       1024,            // 栈大小
                                       control_task,    // 入口函数
                                       nullptr);        // 参数
          return (pid > 0) ? 0 : -1;
      }

      void control_task(void* arg) {
          while (1) {                   // ← 在【自己的任务】里循环
              do_control();
              usleep(800);
          }
      }
""")

    import threading

    nsh_b = NSH()
    task_state = {"running": False, "iterations": 0}

    def control_task():
        """独立任务 —— 在后台线程里跑"""
        task_state["running"] = True
        t_end = time.time() + RUN_MS / 1000.0
        while time.time() < t_end:
            task_state["iterations"] += 1
            time.sleep(0.001)
        task_state["running"] = False

    def px4_task_spawn_cmd(name, scheduler, priority, stack, entry, arg):
        """模拟 PX4 的任务创建 —— 关键: 【立即返回】"""
        t = threading.Thread(target=entry, daemon=True)
        t.start()
        return 1001          # 返回 pid

    def control_good_main(args):
        pid = px4_task_spawn_cmd("control", "SCHED_DEFAULT", 200, 1024,
                                 control_task, None)
        nsh_b.output.append(f"      px4_task_spawn_cmd 返回 pid={pid} (立即返回!)")

    nsh_b.register("control_good", control_good_main)
    nsh_b.register("other", other_cmd)

    print(f"  执行: nsh> control_good")
    t0 = time.time()
    nsh_b.run("control_good")
    elapsed_b = (time.time() - t0) * 1000
    for line in nsh_b.output:
        print(f"    {line}")

    print(f"\n  继续执行: nsh> other")
    nsh_b.run("other")
    print(f"    {nsh_b.output[-1]}")

    print(f"""
  ✅ 后果: nsh> 提示符在 {elapsed_b:.1f}ms 内就回来了。
           用户【可以继续敲命令】。

  同时, 控制任务在后台以 1ms 周期运行:
    (等待任务跑完...)
""")

    time.sleep(RUN_MS / 1000.0 + 0.05)
    print(f"    控制任务共执行 {task_state['iterations']} 次, "
          f"仍在运行={task_state['running']}")

    print(f"""
  【对比总结】

    ┌────────────────┬──────────────────┬────────────────────┐
    │                │ 做法 A (死循环)   │ 做法 B (独立任务)   │
    ├────────────────┼──────────────────┼────────────────────┤
    │ NSH 阻塞        │ ❌ {elapsed_a:>5.0f} ms      │ ✅ {elapsed_b:>5.1f} ms           │
    │ 能否敲别的命令   │ ❌ 不能           │ ✅ 能               │
    │ 优先级          │ 继承 NSH          │ ✅ 自己指定         │
    │ 栈              │ NSH 的栈          │ ✅ 独立栈           │
    │ 适合            │ 一次性操作        │ 周期性控制环        │
    └────────────────┴──────────────────┴────────────────────┘

  ★ 这就是为什么飞控的每个控制模块都要 px4_task_spawn_cmd。
    传感器采样、姿态控制、位置控制…… 都是【独立任务】。

  下一课会逐行分析 px4_task_spawn_cmd 的实现。
""")


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  卷 6 第 2 章 · 写一个 NuttX 应用")
    print("=" * 76)

    exp1_led_call_chain()
    exp2_priv_data()
    exp3_command_vs_task()

    sep("本章小结")
    print("""
  1. NuttX 应用入口: <appname>_main(argc, argv) —— 命令式, 跑完返回
  2. 驱动 = 函数表(file_operations) + register_driver()
  3. register_driver 的第 4 个参数 priv 是【私有数据指针】
     · NuttX 官方例子: NULL  (用全局变量存状态)
     · PX4: (void*)this      (用对象成员存状态)
  4. 对象封装带来的关键能力: 【多实例】
     两个 tfmini、两个 IMU、两个磁罗盘 —— 都需要它
  5. 周期性任务必须用 px4_task_spawn_cmd
     在 xxx_main 里死循环会【阻塞 NSH】

  素材: $SRC_NUTTX_SRC/stm32h7_fmu-master/

  下一课: 逐行分析 px4_task_spawn_cmd()
""")
