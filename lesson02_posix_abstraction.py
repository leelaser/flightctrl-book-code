"""
============================================================================
 卷 6 第 1 章 · NuttX 是什么：架构与定位
 配套实验代码
============================================================================

 运行:  python lesson02_posix_abstraction.py

 三个实验:
   1. 无抽象 vs POSIX 抽象      —— 对比可扩展性
   2. C↔C++ 桥接模式演示         —— 复现 PX4 的 cdev_platform 机制
   3. 三种构建模式的对比         —— 系统调用开销的量化
============================================================================


 核心概念速览

   NuttX = POSIX 兼容的 RTOS
          ↓
   驱动层用 open/read/ioctl/poll, 与硬件解耦
          ↓
   同一份驱动能跑 NuttX 和 Linux (SITL 仿真的基础)
          ↓
   实现靠 C↔C++ 桥接: 静态中转函数 + i_private + 虚函数
"""


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


# ============================================================================
#  实验 1: 无抽象 vs POSIX 抽象
# ============================================================================
def exp1_abstraction_value():
    sep("实验 1: 无抽象 vs POSIX 抽象")

    print("""
  场景: 飞控要支持 3 种测距仪 (TFMINI / LeddarOne / SF45)
        应用层要读距离数据

  【方案 A】无抽象: 应用直接调驱动函数
  【方案 B】POSIX 抽象: 应用统一用 open/read/ioctl
""")

    # ---------------------------------------------------------------
    # 方案 A: 无抽象
    # ---------------------------------------------------------------
    print("  " + "-" * 72)
    print("  方案 A: 应用直接调驱动函数 (裸机风格)")
    print("  " + "-" * 72)

    # 模拟三个厂商各不相同的驱动 API
    class TFMINI_driver:
        """TFMINI: 读 9 字节, 用 mm 为单位, 小端"""
        @staticmethod
        def tfmini_read_raw(buf):
            buf[:9] = bytes([0x59, 0x59, 0x56, 0x01, 0x00, 0x00, 0x01, 0x02, 0x03])
            return 9

    class LeddarOne_driver:
        """LeddarOne: 读 12 字节, 用 cm 为单位, 大端"""
        @staticmethod
        def leddar_get_distance():
            return 3456      # cm  <- 与 SF45 区分开

    class SF45_driver:
        """SF45: 需要先查询再读, 返回浮点"""
        @staticmethod
        def sf45_query():
            return 7.89      # m   <- 与 Leddar 区分开

    def app_without_abstraction(sensor_type):
        """应用代码 —— 必须知道每个驱动的具体接口!"""
        if sensor_type == "tfmini":
            buf = bytearray(9)
            TFMINI_driver.tfmini_read_raw(buf)
            dist_m = (buf[2] + buf[3] * 256) / 100.0     # 解析协议
        elif sensor_type == "leddar":
            dist_m = LeddarOne_driver.leddar_get_distance() / 100.0
        elif sensor_type == "sf45":
            dist_m = SF45_driver.sf45_query()
        else:
            dist_m = 0.0
        return dist_m

    print("  应用代码长这样:\n")
    print("""      if sensor_type == "tfmini":
          buf = bytearray(9)
          TFMINI_driver.tfmini_read_raw(buf)
          dist_m = (buf[2] + buf[3]*256) / 100.0     # 解析协议!
      elif sensor_type == "leddar":
          dist_m = LeddarOne_driver.leddar_get_distance() / 100.0
      elif sensor_type == "sf45":
          dist_m = SF45_driver.sf45_query()
""")
    print("  ⚠️ 问题:")
    print("     · 加一种传感器 -> 必须改应用代码")
    print("     · 应用里有【协议解析】(谁该负责?)")
    print("     · 无法在 PC 上仿真 (依赖具体驱动)")
    print()

    for st in ["tfmini", "leddar", "sf45"]:
        print(f"      app_without_abstraction('{st}') = "
              f"{app_without_abstraction(st):.4f} m")

    # ---------------------------------------------------------------
    # 方案 B: POSIX 抽象
    # ---------------------------------------------------------------
    print("\n  " + "-" * 72)
    print("  方案 B: POSIX 抽象 (open/read/ioctl)")
    print("  " + "-" * 72)

    # 模拟 NuttX VFS
    class VFS:
        """虚拟文件系统 —— 把 /dev/xxx 映射到驱动对象"""
        def __init__(self):
            self.devices = {}          # devname -> driver object

        def register_driver(self, name, driver):
            self.devices[name] = driver

        def open(self, name):
            if name not in self.devices:
                raise OSError(f"ENOENT: {name}")
            return FileDescriptor(name, self.devices[name])

        def read(self, fd, buflen):
            return fd.driver.read(buflen)

        def ioctl(self, fd, cmd, arg):
            return fd.driver.ioctl(cmd, arg)

    class FileDescriptor:
        def __init__(self, name, driver):
            self.name = name
            self.driver = driver

    # --- 驱动基类 (对应 PX4 的 CDev) ---
    class CDev:
        """所有驱动继承它 —— 对应 src/lib/cdev/CDev.hpp"""
        def read(self, buflen):
            raise NotImplementedError
        def ioctl(self, cmd, arg):
            raise NotImplementedError

    # --- 三个驱动都实现同样的接口 ---
    class TFMiniDev(CDev):
        def read(self, buflen):
            return 3.42          # m, 已在驱动内解析协议
        def ioctl(self, cmd, arg):
            return 0

    class LeddarDev(CDev):
        def read(self, buflen):
            return 34.56      # m
        def ioctl(self, cmd, arg):
            return 0

    class SF45Dev(CDev):
        def read(self, buflen):
            return 7.89       # m
        def ioctl(self, cmd, arg):
            return 0

    vfs = VFS()
    vfs.register_driver("/dev/tfmini0", TFMiniDev())
    vfs.register_driver("/dev/leddar0", LeddarDev())
    vfs.register_driver("/dev/sf45_0", SF45Dev())

    def app_with_posix(vfs, devname):
        """应用代码 —— 完全不需要知道是什么传感器!"""
        fd = vfs.open(devname)
        dist_m = vfs.read(fd, 9)
        return dist_m

    print("  应用代码长这样:\n")
    print("""      fd = open(devname)          # 打开设备节点
      dist_m = read(fd, 9)        # 读数据 —— 协议差异被驱动吸收了
      close(fd)
""")
    print("  ✅ 好处:")
    print("     · 加传感器 -> 只写新驱动, 应用代码【一行不改】")
    print("     · 协议解析在驱动里 (职责清晰)")
    print("     · 同一份应用代码能在 Linux 上跑 (SITL!)")
    print()

    for dev in ["/dev/tfmini0", "/dev/leddar0", "/dev/sf45_0"]:
        print(f"      app_with_posix(vfs, '{dev}') = "
              f"{app_with_posix(vfs, dev):.4f} m")

    print(f"""
  [结论] 两种方案的应用代码:

     方案A: {app_without_abstraction('tfmini'):.2f} / {app_without_abstraction('leddar'):.2f} / {app_without_abstraction('sf45'):.2f}  <- 三次不同分支
     方案B: {app_with_posix(vfs,'/dev/tfmini0'):.2f} / {app_with_posix(vfs,'/dev/leddar0'):.2f} / {app_with_posix(vfs,'/dev/sf45_0'):.2f}  <- 同一行代码

     方案 B 就是 POSIX 抽象的价值 —— PX4 支持 40+ 种测距仪靠的就是它。
""")


# ============================================================================
#  实验 2: C↔C++ 桥接模式
# ============================================================================
def exp2_c_cpp_bridge():
    sep("实验 2: C↔C++ 桥接模式 (复现 PX4 cdev_platform)")

    print("""
  问题: NuttX 是 C, 它的 file_operations 是【函数指针表】
        PX4 是 C++, 希望驱动写成【虚函数】

        C 的函数指针【没法】指向 C++ 成员函数 (成员函数有隐含的 this)

  解法: 三步桥接 (对应 src/lib/cdev/nuttx/cdev_platform.cpp)
""")

    print("  " + "-" * 72)
    print("  【第一步】定义静态中转函数 (纯 C 可调)")
    print("  " + "-" * 72)
    print("""
      static ssize_t cdev_read(file_t *filp, char *buffer, size_t buflen)
      {
          // ⭐ 从 inode 的私有数据取出 C++ 对象指针
          cdev::CDev *cdev = (cdev::CDev *)(filp->f_inode->i_private);
          // 调虚函数 —— 多态生效
          return cdev->read(filp, buffer, buflen);
      }
""")

    print("  " + "-" * 72)
    print("  【第二步】函数表指向中转函数")
    print("  " + "-" * 72)
    print("""
      const struct file_operations cdev::CDev::fops = {
          read  : cdev_read,
          write : cdev_write,
          ioctl : cdev_ioctl,
          ...
      };
""")

    print("  " + "-" * 72)
    print("  【第三步】注册时把 this 存进 inode")
    print("  " + "-" * 72)
    print("""
      register_driver(name, &fops, 0666, (void *)this);
      //                                  ↑ NuttX 把它存进 inode->i_private
""")

    # --- 用 Python 模拟整个机制 ---
    print("\n  === Python 模拟: 完整调用链 ===")

    # 模拟 NuttX 的 inode
    class Inode:
        def __init__(self, name):
            self.name = name
            self.i_private = None       # 存放驱动对象指针
            self.fops = None            # 函数表

    # 模拟 NuttX 的 file 结构
    class File:
        def __init__(self, inode):
            self.f_inode = inode

    # ---- C 侧: 函数表 + 中转函数 ----
    # (Python 里没有指针, 用 "表里的函数" + "从 inode 取对象" 来模拟同样的机制)
    def cdev_read(file, buflen):
        """中转函数 —— 对应 C 的 cdev_read()"""
        # 关键: 从 inode 找回对象
        cdev = file.f_inode.i_private
        # 调"虚函数"
        return cdev.read(buflen)

    def cdev_ioctl(file, cmd, arg):
        cdev = file.f_inode.i_private
        return cdev.ioctl(cmd, arg)

    # 函数表 (对应 struct file_operations)
    fops = {
        "read":  cdev_read,
        "ioctl": cdev_ioctl,
    }

    # ---- C++ 侧: 驱动类 ----
    class CDev:
        """对应 PX4 的 cdev::CDev"""
        def __init__(self, devname):
            self._devname = devname

        def read(self, buflen):
            raise NotImplementedError

        def ioctl(self, cmd, arg):
            raise NotImplementedError

        def register(self, vfs):
            """对应 CDev::register_class_devname()"""
            node = vfs.mknod(self._devname)
            node.fops = fops                    # 函数表
            node.i_private = self               # ⭐ 把 this 存进去
            return 0

    class TFMiniDev(CDev):
        """TFMINI 驱动"""
        def __init__(self):
            super().__init__("/dev/tfmini0")
            self._count = 0

        def read(self, buflen):
            self._count += 1
            return 2.56 + self._count * 0.01

        def ioctl(self, cmd, arg):
            if cmd == "SENSORIOCSPOLLRATE":
                return 0
            return -1

    class LeddarDev(CDev):
        def __init__(self):
            super().__init__("/dev/leddar0")
        def read(self, buflen):
            return 34.56
        def ioctl(self, cmd, arg):
            return 0

    # ---- VFS ----
    class VFS:
        def __init__(self):
            self.nodes = {}
        def mknod(self, name):
            node = Inode(name)
            self.nodes[name] = node
            return node
        def open(self, name):
            if name not in self.nodes:
                raise OSError(f"ENOENT: {name}")
            return File(self.nodes[name])

    # ---- 组装 ----
    vfs = VFS()
    TFMiniDev().register(vfs)
    LeddarDev().register(vfs)

    print(f"\n  注册的设备: {list(vfs.nodes.keys())}")
    for name, node in vfs.nodes.items():
        print(f"    {name}: i_private={type(node.i_private).__name__}  "
              f"fops={list(node.fops.keys())}")

    print("\n  调用链演示 (注意虚函数分发):")
    for name in ["/dev/tfmini0", "/dev/tfmini0", "/dev/leddar0"]:
        f = vfs.open(name)
        # 走 fops 表 -> 中转函数 -> 虚函数
        val = f.f_inode.fops["read"](f, 9)
        impl = type(f.f_inode.i_private).__name__
        print(f"    open('{name}') -> fops['read']() -> {impl}.read() = {val:.4f}")

    print("""
  [关键观察] 同一个 fops['read'] (中转函数),
             因为 i_private 指向不同对象,
             最终调用了不同驱动的 read()。

             这就是【多态】在 C 接口上的实现。
""")

    print("  📁 真实代码对照:")
    print("     src/lib/cdev/CDev.hpp                 —— CDev 基类")
    print("     src/lib/cdev/nuttx/cdev_platform.cpp  —— cdev_read 等中转函数")
    print("     src/lib/cdev/CDev.cpp                 —— register_class_devname()")


# ============================================================================
#  实验 3: 三种构建模式的开销对比
# ============================================================================
def exp3_build_modes():
    sep("实验 3: 三种构建模式 (Flat / Protected / Kernel)")

    print("""
  NuttX 支持三种构建模式, PX4 用第一种:

    Flat Build      : 单一地址空间, 无隔离, 函数直接调用
    Protected Build : MPU 隔离内核/用户, 需系统调用 (SVC)
    Kernel Build    : MMU 隔离, 完整系统调用 (类 Linux)
""")

    # 用真实的 Cortex-M 数据估算
    print("  " + "-" * 72)
    print("  系统调用开销 (Cortex-M7 @ 480MHz, 实测典型值)")
    print("  " + "-" * 72)

    # Cortex-M7 @ 480MHz
    f_cpu = 480e6
    cycles_per_us = f_cpu / 1e6

    print(f"  CPU: Cortex-M7 @ {f_cpu/1e6:.0f} MHz  "
          f"({cycles_per_us:.0f} cycles/μs)")
    print()

    # 典型开销
    modes = [
        ("Flat",      0,   "直接函数调用"),
        ("Protected", 80,  "SVC 异常 + 参数检查 + 栈切换"),
        ("Kernel",    150, "完整陷阱 + MMU 切换 + TLB"),
    ]

    print(f"  {'模式':<12} {'系统调用开销':>14} {'单次耗时':>12} {'1kHz下CPU占比':>15}")
    print("  " + "-" * 58)
    for name, cycles, note in modes:
        us = cycles / cycles_per_us if cycles else 0.0
        # 假设 1kHz 控制环每周期调用 5 次系统调用
        overhead_per_period_us = us * 5
        pct = overhead_per_period_us / 1000 * 100
        cyc_str = f"{cycles} cycles" if cycles else "0 (直接调用)"
        us_str = f"{us:.3f} μs" if us else "0 μs"
        print(f"  {name:<12} {cyc_str:>14} {us_str:>12} {pct:>13.4f}%")

    print(f"""
  [计算假设] 1kHz 控制环, 每周期 5 次系统调用 (读 IMU + 读磁力计
             + 2 次 ioctl + 1 次 poll)

  [结论 —— 注意这里有个反直觉的点]
    Flat      : 0 开销 (直接函数调用)
    Protected : 每周期 0.835 μs = 0.0835% CPU 占用
    Kernel    : 每周期 1.56 μs  = 0.156% CPU 占用

    ⚠️ 单看【吞吐量】, Protected 模式的开销其实【很小】——
       0.0835% 的 CPU 占用, 一年也就累积约 7.3 小时。
       如果只看这个数字, 用 Protected 换来内存隔离似乎是划算的。

    ✅ 真正的理由是【不确定性】, 不是吞吐量:

       · Flat:      调用一个函数, 耗时 = 固定的几个时钟周期
       · Protected: 触发 SVC 异常 -> 硬件保存上下文 -> 切栈
                    -> 执行内核代码 -> 恢复 -> 返回
                    其中【硬件保存上下文】的耗时受【中断状态】影响

       如果此时来了一个高优先级中断, SVC 的返回会被推迟。
       于是系统调用的耗时不再是常数, 而是有【抖动】的。

       对 1kHz 控制环来说, 抖动比平均开销可怕得多
       (回想卷 4 第 1 章实验 3: 系统性偏差才致命)。

    ✅ 第二个理由是【复杂度】:
       Protected 模式要求代码分成"内核部分"和"用户部分",
       驱动要指定自己属于哪一边, 参数要跨空间传递...
       对飞控这种"所有代码都是自己写的"场景, 收益有限。

    ✅ 第三个理由: "隔离性"对自家代码意义有限 ——
       你不可能因为"驱动可能崩"就不让驱动碰硬件。

    所以 PX4 选 Flat Build。

    📌 一句话: Flat 的优势不是"快", 而是"确定"。
""")

    print("  📁 验证: boards/px4/fmu-v6x/nuttx-config/nsh/defconfig")
    print("     没有 CONFIG_BUILD_PROTECTED / CONFIG_BUILD_KERNEL")
    print("     ==> 确认是 Flat Build")


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  卷 6 第 1 章 · NuttX 是什么：架构与定位")
    print("=" * 76)

    exp1_abstraction_value()
    exp2_c_cpp_bridge()
    exp3_build_modes()

    sep("本章小结")
    print("""
  1. NuttX = POSIX 兼容的 RTOS, "长得像 Linux"
  2. POSIX 让驱动与硬件解耦 -> 同一份驱动能跑 NuttX 和 Linux
     ==> 这是 SITL 仿真能存在的基础
  3. NuttX 有三种构建模式; PX4 用 Flat (最快、无隔离)
  4. PX4 用 C<->C++ 桥接把 NuttX 的 file_operations 接到 C++ 虚函数
     三要素: 静态中转函数 + i_private 存对象 + 虚函数分发
  5. PX4 选 NuttX (POSIX + 文件系统 + Shell)
     ArduPilot 选 ChibiOS (轻 + HAL 完善)

  下一课: 卷 6 第 3 章
         逐行分析 px4_task_spawn_cmd()
""")
