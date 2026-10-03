"""
============================================================================
 NuttX 教程 · 第 6 课 · 设备驱动与文件系统
 配套实验代码
============================================================================

 运行:  python lesson06_device_driver.py

 五个实验:
   1. fd 表与路径解析        —— open("/dev/led1") 到底查了什么
   2. register_driver        —— 设备节点是怎么挂进 inode 树的
   3. 完整调用链              —— 从 open() 到 led_open() 的每一步
   4. 玩具驱动 vs 标准驱动    —— led_main.c 的 9 个问题逐条演示
   5. ioctl 命令号编码        —— _IOC / _IOC_TYPE / _IOC_NR

 对照的真实源码:
   实物参考  D:/DEVLib/PX4APM/Nuttx/stm32h7_fmu-master/src/drivers/led/led_main.c
   内核源码  platforms/nuttx/NuttX/nuttx/
      include/nuttx/fs/fs.h              struct file_operations / file / inode
      fs/vfs/fs_open.c                   nx_open
      fs/vfs/fs_read.c                   nx_read
      fs/vfs/fs_registerdriver.c         register_driver
      fs/inode/fs_inodereserve.c         inode_reserve
      drivers/leds/userled_upper.c       【标准】LED 驱动
      include/nuttx/fs/ioctl.h           _IOC 系列宏
============================================================================
"""


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


def w(s):
    """字符串在终端里的显示宽度（中文等全角字符算 2 列）"""
    return sum(2 if ord(c) > 0x1100 and (
        0x4E00 <= ord(c) <= 0x9FFF or      # CJK 统一表意
        0x3000 <= ord(c) <= 0x303F or      # CJK 标点
        0xFF00 <= ord(c) <= 0xFF60 or      # 全角
        0x2500 <= ord(c) <= 0x257F or      # 制表符
        0x2190 <= ord(c) <= 0x21FF or      # 箭头
        0x2600 <= ord(c) <= 0x27BF) else 1 for c in s)


def pad(s, width, align="left"):
    """按显示宽度对齐"""
    d = max(0, width - w(s))
    return s + " " * d if align == "left" else " " * d + s


# ============================================================================
#  实验 1: fd 表与路径解析
# ============================================================================
def exp1_fd_and_path():
    sep("实验 1: fd 表与路径解析 —— open 到底做了什么")

    print("""
  NuttX 的 open("/dev/led1", O_RDONLY) 内部要做三件事:

     ① 路径解析:  "/dev/led1"  →  struct inode*
     ② 分配 fd:   fd = 3 (前面 0/1/2 是 stdin/stdout/stderr)
     ③ 建 struct file: 把 inode 和 fd 关联起来

     最终用户拿到的只是一个【整数 fd】,
     所有后续操作都靠它反查回 struct file。
""")

    # ---- 模拟 inode 树（简化版：NuttX 用哈希表，见实验 2 说明）----
    INODES = {
        "/":          {"type": "dir",  "ops": None, "priv": None},
        "/dev":       {"type": "dir",  "ops": None, "priv": None},
        "/dev/led1":  {"type": "char", "ops": "led_ops",
                       "priv": None},        # ← register_driver 的 i_private 参数
        "/obj":       {"type": "dir",  "ops": None, "priv": None},
        "/obj/sensor_gyro": {"type": "char", "ops": "orb_ops", "priv": "gyro"},
    }

    # ---- 模拟 fd 表（每个线程一份，见下方说明）----
    class File:
        """对应 struct file"""
        _next = 0
        def __init__(self, path, oflags, inode):
            File._next += 1
            self.f_oflags = oflags     # 打开模式
            self.f_pos = 0             # 文件偏移
            self.f_inode = inode       # 指向 inode
            self.f_priv = None         # 驱动可以挂私有数据！
            self.path = path
            self.n = File._next

    class FDTable:
        def __init__(self):
            self.slots = [None, None, None]   # 0,1,2 被占用
            self.slots[0] = "stdin"
            self.slots[1] = "stdout"
            self.slots[2] = "stderr"

        def allocate(self, f):
            for i, s in enumerate(self.slots):
                if s is None:
                    self.slots[i] = f
                    return i
            self.slots.append(f)
            return len(self.slots) - 1

        def get(self, fd):
            if 0 <= fd < len(self.slots):
                return self.slots[fd]
            return None

        def release(self, fd):
            if 0 <= fd < len(self.slots):
                self.slots[fd] = None

    def do_open(fdt, path, oflags):
        print(f"\n  open(\"{path}\", {oflags})")
        print(f"    ├─ ① inode_find(\"{path}\")")
        if path not in INODES:
            print(f"    │     ❌ 没找到 → 返回 -ENOENT")
            return -1
        inode = INODES[path]
        print(f"    │     ✓ 找到 inode, type={inode['type']}, "
              f"i_ops={inode['ops']}")
        if inode["type"] != "char":
            print(f"    │     ❌ 不是字符设备 → -ENXIO")
            return -1

        # inode_checkflags() —— 只看函数表有没有实现, 不看 mode 位!
        # 📁 fs/vfs/fs_open.c:226-247
        print(f"    ├─ ② inode_checkflags(inode, {oflags})")
        want_write = oflags == "O_WRONLY"
        has_write = inode.get("has_write", True)
        if want_write and not has_write:
            print(f"    │     ❌ i_ops->write == NULL → -EACCES")
            print(f"    │        (注意: 不是因为有 0444 权限位!)")
            return -1
        print(f"    │     ✓ 通过  (i_ops->read/write 都已实现)")

        print(f"    ├─ ③ files_allocate() 分配 fd")
        f = File(path, oflags, inode)
        fd = fdt.allocate(f)
        print(f"    │     ✓ fd = {fd}  (struct file #{f.n})")

        print(f"    └─ ④ 调用驱动的 open 回调")
        print(f"          inode->u.i_ops->open(filep)  →  led_open()")
        return fd

    fdt = FDTable()

    print("  " + "-" * 72)
    print("  ① 正常打开")
    print("  " + "-" * 72)
    fd = do_open(fdt, "/dev/led1", "O_RDONLY")

    print("\n  " + "-" * 72)
    print("  ② 以写模式打开同一个设备")
    print("  " + "-" * 72)
    do_open(fdt, "/dev/led1", "O_WRONLY")

    print("\n  " + "-" * 72)
    print("  ③ 打开一个【只实现了 read、没实现 write】的设备")
    print("  " + "-" * 72)
    INODES["/dev/readonly"] = {"type": "char", "ops": "ro_ops",
                               "priv": None, "has_write": False}
    do_open(fdt, "/dev/readonly", "O_WRONLY")

    print("\n  " + "-" * 72)
    print("  ④ 打开不存在的设备")
    print("  " + "-" * 72)
    do_open(fdt, "/dev/nonexistent", "O_RDONLY")

    print(f"""
  【fd 表的状态】

     {"fd":<4} {"内容":<28} 说明
     {"─"*4} {"─"*28} {"─"*24}""")
    for i, s in enumerate(fdt.slots):
        if isinstance(s, File):
            desc = f"file#{s.n} -> {s.path}"
        elif s:
            desc = s
        else:
            desc = "(空闲)"
        print(f"     {i:<4} {desc:<28}")
    print(f"     ...")

    print("""
  【关键认识 ①: fd 只是数组下标】

     用户空间的 fd 就是一个【整数】。
     内核用它在【当前线程的 fd 表】里查出 struct file*。
     再从 file->f_inode 找到 inode，
     最后从 inode->u.i_ops 找到驱动的函数表。

          fd ──► struct file ──► struct inode ──► file_operations
          ↑         ↑                ↑                  ↑
        用户持有  打开状态        设备身份          驱动实现
                 (f_pos/f_oflags)  (i_private)     (open/read/...)

  负责 "fd → struct file*" 查找的函数叫 fs_getfilep()
  （📁 fs/inode/fs_files.c:342）

      int fs_getfilep(int fd, FAR struct file **filep);

  ★ 注意: 老教程里的 files_get() / files_release() / files_dup()
    在这个版本的 NuttX 里【已经删除了】。别照着老博客写代码。

  ★ fd 表本身也变了: 不再是定长扁平数组, 而是【按行动态扩容的二维数组】

      struct filelist {
        sem_t             fl_sem;     /* 保护这张表 */
        uint8_t           fl_rows;    /* 当前有多少行 */
        FAR struct file **fl_files;   /* 指向二维数组 */
      };

      查表公式:
        fl_files[fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                 [fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]

      CONFIG_NFILE_DESCRIPTORS_PER_BLOCK 默认 8。
      老教程里的 CONFIG_NFILE_DESCRIPTORS 已经不存在了。

  ★ "槽位是否空闲"的判据是 f_inode == NULL（不是 f_oflags）。

  【⚠️ 关键发现: NuttX 的 VFS 【不检查权限位】】

     看 file_vopen() 里那个"权限检查"的实际代码
     （📁 fs/vfs/fs_open.c:226-247）:

         int inode_checkflags(FAR struct inode *inode, int oflags)
         {
           if (INODE_IS_PSEUDODIR(inode))
             {
               return OK;
             }

           if (inode->u.i_ops == NULL)
             {
               return -ENXIO;
             }

           if (((oflags & O_RDOK) != 0 && !inode->u.i_ops->read) ||
               ((oflags & O_WROK) != 0 && !inode->u.i_ops->write))
             {
               return -EACCES;
             }
           else
             {
               return OK;
             }
         }

     它只看【函数表里有没有实现 read/write】——
     【完全没有看 inode->i_mode】!

     验证: 全树搜索 i_mode 的使用点

         fs/inode/fs_inodereserve.c:82    node->i_mode = mode;      ← 只在注册时【写】
         fs/vfs/fs_chstat.c:452           inode->i_mode = buf->st_mode;  ← chstat 时写
         fs/cromfs/...                    文件系统自己读(用于 stat)
         fs/vfs/fs_open.c                 (空 —— 没有读!)

     ==> register_driver(path, fops, mode, priv) 的 mode 参数
         【是装饰性的】, 只影响 ls -l 的显示, 【不阻止任何操作】。

     ==> 玩具 LED 以 0444 注册, 但它的 led_write 是【非 NULL】的,
         所以 open("/dev/led1", O_WRONLY) 【会成功】!
         作者用 read 控制 LED 【不是被迫的】—— 只是随手写的。

  【为什么这很重要】

     对飞控来说 NuttX 的 /dev 本来就是"内核内部接口",
     没有安全边界的概念, 所以这个设计【可以接受】。

     但如果你从 Linux 过来, 会以为 0444 真的只读 —— 那就错了。
     ⚠️ 想真正限制操作, 唯一的办法是【不实现那个回调】(传 NULL)。

     对照标准驱动 (📁 drivers/leds/userled_upper.c:106-118):

         static const struct file_operations userled_fops =
         {
           userled_open,   /* open */
           userled_close,  /* close */
           NULL,           /* read  ← 【不实现 read, 才是真的只读】 */
           userled_write,  /* write */
           NULL,           /* seek */
           userled_ioctl,  /* ioctl */
           NULL            /* poll */
         #ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
           , NULL          /* unlink */
         #endif
         };

     标准驱动传 NULL 而不是写个"什么也不做"的函数 —— 这才是正确做法。


  【关键认识 ②: 三层结构各有各的"可变性"】

     ┌──────────────┬────────────────────────────────────────────┐
     │ struct file  │ 【每次 open 新建】                          │
     │              │ f_pos (读到哪里了)、f_oflags (怎么打开的)   │
     │              │ f_priv ★ 驱动可以在这里挂【本次打开】的数据 │
     ├──────────────┼────────────────────────────────────────────┤
     │ struct inode │ 【register_driver 时创建, 全局唯一】        │
     │              │ i_ops (函数表)、i_private (设备实例数据)    │
     │              │ 同一个设备被打开 10 次 → 只有 1 个 inode    │
     ├──────────────┼────────────────────────────────────────────┤
     │ file_ops     │ 【驱动作者写的静态表】                      │
     │              │ 通常在 .rodata 里, 所有打开共享             │
     └──────────────┴────────────────────────────────────────────┘

     💡 玩具 LED 驱动【完全没有用 f_priv 和 i_private】——
        这就是它无法支持多实例的根因。见实验 4。
""")


# ============================================================================
#  实验 2: register_driver
# ============================================================================
def exp2_register():
    sep("实验 2: register_driver —— 设备节点是怎么挂进去的")

    print("""
  调用形式 (led_main.c:66):

      (void)register_driver("/dev/led1", &led_ops, 0444, NULL);
                                ↑          ↑         ↑     ↑
                             设备路径   函数表    权限   私有数据
                                                          (i_private)

  ★ 注意最后那个 NULL —— 它是 i_private。
    玩具驱动传 NULL, 意味着【驱动无法访问任何设备状态】。
    这就是 led_open/read/write 里从来不用 filep 的原因。
""")

    # 模拟 inode 表（NuttX 真实实现：哈希表 + 链表防冲突）
    class Node:
        def __init__(self, path, ops, mode, priv):
            self.path = path
            self.ops = ops
            self.mode = mode
            self.priv = priv
            self.opens = 0

    class FsTable:
        def __init__(self):
            self.nodes = {"/": Node("/", None, 0o755, None),
                          "/dev": Node("/dev", None, 0o755, None)}
            self.log = []

        def register(self, path, ops, mode, priv):
            self.log.append(f"  register_driver(\"{path}\", {ops}, {mode:04o}, "
                            f"{'NULL' if priv is None else priv})")

            # ① 路径是否已被占用（inode_search 探测）
            if path in self.nodes:
                self.log.append(f"    ├─ inode_search() 发现路径已存在")
                self.log.append(f"    │    (mode={self.nodes[path].mode:04o})")
                self.log.append(f"    └─ ❌ 返回 -EEXIST   ← 注意! 不是覆盖")
                return -17
            self.log.append(f"    ├─ inode_search() 路径空闲 ✓")

            # ② inode_reserve() 逐级建节点 —— 中间目录【自动创建】
            parts = [p for p in path.split("/") if p]
            cur = ""
            created = []
            self.log.append(f"    ├─ inode_reserve() 逐级建节点:")
            for i, part in enumerate(parts):
                cur += "/" + part
                is_leaf = (i == len(parts) - 1)
                if cur in self.nodes:
                    self.log.append(f"    │    \"{cur}\" 已存在, 跳过")
                    continue
                m = mode if is_leaf else 0o777
                self.nodes[cur] = Node(cur, ops if is_leaf else None, m,
                                       priv if is_leaf else None)
                kind = "叶子" if is_leaf else "中间目录"
                self.log.append(f"    │    + 创建 \"{cur}\"  "
                                f"mode={m:04o}  ({kind})")
                created.append(cur)

            # ③ 填内容
            leaf = self.nodes[path]
            self.log.append(f"    ├─ INODE_SET_DRIVER(node)   打类型标签")
            self.log.append(f"    ├─ node->u.i_ops   = {leaf.ops}")
            self.log.append(f"    └─ node->i_private = "
                            f"{'NULL' if leaf.priv is None else leaf.priv}")
            if len(created) > 1:
                self.log.append(f"       ↑ 自动创建了 {len(created)-1} 个中间目录!")
            return 0

    fs = FsTable()

    def show(label, path, ops, mode, priv):
        print("  " + "-" * 72)
        print(f"  {label}")
        print("  " + "-" * 72)
        fs.log.clear()
        ret = fs.register(path, ops, mode, priv)
        for l in fs.log:
            print(l)
        errname = {0: "OK", -2: "-ENOENT", -17: "-EEXIST"}.get(ret, str(ret))
        print(f"    返回值 = {ret}   ({errname})")

    show("① 注册 /dev/led1（第一次）", "/dev/led1", "&led_ops", 0o444, None)
    show("② 再注册一次同样的路径（led_main 每次运行都会做！）",
         "/dev/led1", "&led_ops", 0o444, None)
    show("③ 注册到一个【完全不存在的】目录",
         "/myapp/sub/led2", "&led_ops", 0o444, None)

    print(f"""
  【★ 反直觉的一点：中间目录会被【自动创建】】

     NuttX 的 inode_reserve() 不是"目录必须存在才能注册"，
     而是【逐级自动创建】——对照 fs/inode/fs_inodereserve.c:205-248:

         for (; ; )
           {{
             FAR const char *nextname = inode_nextname(name);
             if (*nextname != '\\0')
               {{
                 /* Insert an operationless node */
                 node = inode_alloc(name, 0777);      ← 中间目录, 0777, 无 ops
                 ...
                 parent = node;
                 continue;
               }}
             else
               {{
                 node = inode_alloc(name, mode);      ← 叶子节点, 用户给的 mode
                 ...
               }}
           }}

     所以 register_driver("/dev/led") 之所以能工作，
     是因为 "/dev" 会被【顺手创建出来】—— 而不是因为它预先存在。

     ⚠️ 这也意味着一个拼错的路径不会报错，而是【悄悄地创建一棵无用的目录树】:

         register_driver("/dev/leed1", ...)     ← 打错了
         → 创建 /dev/leed1, 注册成功, 返回 0
         → 之后 open("/dev/led1") 找不到设备, 但 register 时没人报警

     防御: 注册后立刻自己 open 一次验证；或统一用宏定义路径。
""")

    print(f"""
  【★ 发现的问题：重复注册返回 -EEXIST】

     看 led_main.c:64-67:

         void led_register() {{
           stm32_configgpio(GPIO_LED_BLUE);
           (void)register_driver("/dev/led1", &led_ops, 0444, NULL);
         }}

     而 led_main() 每次都调用 led_register():

         int led_main(int argc, char** argv) {{
           led_register();                    ← 每次运行都注册!
           int fd = open("/dev/led1", O_RDONLY);
           ...

     第二次运行 `led` 命令时:
       · register_driver 返回 -EEXIST
       · (void) 把它丢掉了 —— 【错误被静默忽略】
       · open 打开的是第一次注册的 inode
       · 恰好 led_ops 是同一个静态变量, 所以"看起来没问题"

     这是【靠巧合工作的代码】。如果 led_ops 是动态分配的, 就会出现
     "inode 指向已释放内存"的悬垂指针。

  【★ 发现的问题：注册时机错了】

     register_driver 应该在哪里调用?

       ❌ 现在: 在 led_main() 里 —— 意味着【只有运行 led 命令后】设备才存在
       ✅ 应该: 在 board 的 stm32_bringup() 里 —— 开机就注册

     现在这个设计导致: 任何想在开机时就 open("/dev/led1") 的代码都会失败,
     除非它先执行 led 命令。启动脚本 rcS 里的 `led off` 之所以能工作,
     正是因为 led_main 自己会注册。

     正确做法（对照 stm32_bringup.c 里 userled 的写法）:

         #if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
           ret = userled_lower_initialize("/dev/userleds");
           if (ret < 0) {{
             syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\\n", ret);
           }}
         #endif

     注意对比: 标准做法【检查返回值并记录日志】。
""")


# ============================================================================
#  实验 3: 完整调用链
# ============================================================================
def exp3_call_chain():
    sep("实验 3: 完整调用链 —— 从 nsh> led off 到 GPIO 电平变化")

    print("""
  这一节把整条链路走一遍。每一层都有真实的源码文件对应。

  ══════════════════════════════════════════════════════════════════════
   阶段 0: 开机 —— 固件里已经烧进了什么
  ══════════════════════════════════════════════════════════════════════

    源码文件  ROMFS/fmu_common/init.d/rcS
              │
              │  genromfs  (CMake 里的 add_custom_command)
              ▼
           romfs.img
              │
              │  xxd -i  →  C 数组
              ▼
    boards/fmu-h743vi/include/nsh_romfsimg.h
              │
              │  #include 进固件
              ▼
    const unsigned char romfs_img[] = { 0x2d,0x72,0x6f,0x6d,0x31, ... }
              │
              │  CONFIG_NSH_ROMFSETC=y
              ▼
    开机时 NSH 把 romfs_img 挂到 /etc, 然后执行 /etc/init.d/rcS

  ══════════════════════════════════════════════════════════════════════
   阶段 1: rcS 里的 `led off` 被解析
  ══════════════════════════════════════════════════════════════════════
""")

    steps = [
        ("rcS 的一行文本", "led off", None),
        ("NSH 词法分析", "cmd=\"led\"  argv=[\"led\",\"off\"]",
         "apps/nshlib/nsh_parse.c"),
        ("查内建命令表", "在 g_builtins[] 里找名字 \"led\"",
         "apps/builtin/builtin_list.c"),
        ("找到条目", "{ \"led\", 100, 2048, led_main }",
         "g_builtins[] 的一个元素"),
        ("sched_lock()", "防止子任务在 waitpid 之前就开跑",
         "apps/nshlib/nsh_builtin.c:82"),
        ("调用入口", "led_main(2, [\"led\",\"off\"])", "led_main.c:69"),
    ]

    for i, (what, detail, src) in enumerate(steps):
        mark = "├─" if i < len(steps) - 1 else "└─"
        print(f"    {mark} {what}")
        print(f"    │    {detail}")
        if src:
            print(f"    │    📁 {src}")
        print(f"    │")

    print("""
  ★ 那么 g_builtins[] 里的 { "led", 100, 2048, led_main } 是谁生成的?

     nuttx/CMakeLists.txt:36-49

        set(builtin_apps_string)
        get_property(module_libraries GLOBAL PROPERTY FMU_MODULE_LIBRARIES)
        foreach(module ${module_libraries})
          get_target_property(MAIN ${module} MAIN)
          get_target_property(STACK_MAIN ${module} STACK_MAIN)
          get_target_property(PRIORITY ${module} PRIORITY)
          if(MAIN)
            set(builtin_apps_string
                "${builtin_apps_string}{ \\"${MAIN}\\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main},\\n")
            set(builtin_apps_decl_string
                "${builtin_apps_decl_string}int ${MAIN}_main(int argc, char *argv[]);\\n")
          endif()
        endforeach()

     然后 configure_file 把 @builtin_apps_string@ 填进 fmu.bdat.in → fmu.bdat
     → 拷到 nuttx/apps/builtin/registry/fmu.bdat
     → NuttX 的 apps/builtin/Makefile 把所有 .bdat 拼成 builtin_list.h
     → builtin_list.c 里 #include 它, 塞进 g_builtins[]

     ⚠️ 注意生成的是 ${MAIN}_main  ——  所以 cmake 里写 MAIN led,
        对应 C 函数必须叫 led_main。

     ⚠️ 但 fmu_add_module.cmake:102 还有一个【没用的】定义:

           target_compile_definitions(${MODULE} PRIVATE FMU_MAIN=${MAIN}_app_main)

        全工程搜不到任何地方用 FMU_MAIN —— 它期待入口叫 led_app_main,
        而真实机制要的是 led_main。两份互相矛盾的约定, 其中一份是死代码。
        （这是从 PX4 抄 cmake 时留下的残迹）

  ══════════════════════════════════════════════════════════════════════
   阶段 2: led_main 内部
  ══════════════════════════════════════════════════════════════════════
""")

    print("""    led_main(2, ["led","off"])
      │
      ├─ led_register()                        [led_main.c:70 → 64]
      │    ├─ stm32_configgpio(GPIO_LED_BLUE)  配置 PA1 为输出
      │    └─ register_driver("/dev/led1", &led_ops, 0444, NULL)
      │         └─ inode_reserve("/dev/led1", &inode)
      │              ├─ 父目录 "/dev" 存在?          [fs/inode/fs_inodereserve.c]
      │              ├─ 路径已被占用? → -EEXIST
      │              └─ inode->u.i_ops     = &led_ops
      │                 inode->i_private   = NULL        ← 玩具驱动传了 NULL
      │
      ├─ open("/dev/led1", O_RDONLY)              [fs/vfs/fs_open.c]
      │    ├─ nx_open() → inode_find()            路径 → inode
      │    ├─ files_allocate()                    分配 fd = 3
      │    ├─ file->f_inode = inode
      │    └─ inode->u.i_ops->open(filep)
      │         └─ led_open()                    [led_main.c:39]
      │              └─ stm32_configgpio(GPIO_LED_BLUE)   ← 【又配置了一遍】
      │
      ├─ read(fd=3, &on, 1)                       [fs/vfs/fs_read.c]
      │    ├─ nx_read() → file_read()
      │    │    ├─ 检查 O_RDONLY 允许读 ✓
      │    │    └─ inode->u.i_ops->read(filep, buffer, 1)
      │    │         └─ led_read()               [led_main.c:50]
      │    │              ├─ uint8_t on = *buffer > 0 ? 1 : 0;
      │    │              │    buffer 是【调用者的栈变量 &on】
      │    │              │    buffer_len=1 但【没有检查】
      │    │              └─ stm32_gpiowrite(GPIO_LED_BLUE, 0)   ← 关灯
      │    │                  return 0            ← 【返回 0 = EOF!】
      │    └─ 返回 0 给调用者
      │
      └─ led_main 返回 0（没有 close(fd)!）

  ══════════════════════════════════════════════════════════════════════
   阶段 3: 硬件层面
  ══════════════════════════════════════════════════════════════════════

    stm32_gpiowrite(GPIO_LED_BLUE, 0)
      └─ 写 STM32H7 的 GPIOA->BSRR 寄存器
           └─ PA1 输出低电平
                └─ LED 灭（因为 GPIO_LED_BLUE 里含 GPIO_OUTPUT_CLEAR）

  【★ 这条链路上最反直觉的地方】

     read() 居然能【写硬件】。

     POSIX 语义: read  = 从设备取数据到用户缓冲区
                 write = 从用户缓冲区送数据到设备

     而 led_read() 是这么写的:

         static ssize_t led_read(file_t* filep, char* buffer, size_t buffer_len) {
           uint8_t on = *buffer > 0 ? 1 : 0;      ← 读【用户】的缓冲区
           stm32_gpiowrite(GPIO_LED_BLUE, on);    ← 写【硬件】
           return 0;
         }

     它把 read 用成了 write！

     而 led_write() 是一模一样的代码 —— 因为设备注册成 0444（只读）,
     write 路径【永远无法到达】。所以 led_main 只好用 read 来控制 LED。

     ⚠️ 这是"注册权限和驱动实现不匹配"造成的连锁妥协。
        正确做法: 注册成 0666, write 用来设置 LED 状态,
                  read 用来读回状态（如果有 ll_getall 之类的接口）。
""")


# ============================================================================
#  实验 4: 玩具驱动 vs 标准驱动
# ============================================================================
def exp4_toy_vs_standard():
    sep("实验 4: 玩具驱动 vs 标准驱动 —— 9 个问题逐条演示")

    ROWS = [
        ("① f_priv / 每次打开的状态", "❌ 完全没用",
         "✅ 分配 userled_open_s 挂到 filep->f_priv"),
        ("② i_private / 设备实例数据", "❌ register 传 NULL",
         "✅ 指向 userled_upperhalf_s"),
        ("③ 缓冲区长度检查", "❌ 无",
         "✅ if (len < sizeof(userled_set_t)) return -EINVAL;"),
        ("④ 缓冲区 NULL 检查", "❌ 无", "✅ DEBUGASSERT(buffer != NULL)"),
        ("⑤ 返回值", "❌ return 0（=EOF！）",
         "✅ return (ssize_t)sizeof(userled_set_t);"),
        ("⑥ 设备级互斥", "❌ 无", "✅ sem_t lu_exclsem + takesem/givesem"),
        ("⑦ 错误路径清理", "❌ 无 goto 清理",
         "✅ goto errout_with_sem 统一释放信号量"),
        ("⑧ 多实例支持", "❌ 硬编码 GPIO_LED_BLUE",
         "✅ 通过 inode->i_private 拿设备数据"),
        ("⑨ 函数表初始化", "❌ 位置式写 7 个（struct 有 8 个字段）",
         "✅ 每字段带注释 + unlink 处条件编译"),
    ]

    print(f"\n  {'方面':<30} {'玩具 led_main.c':<36} 标准 userled_upper.c")
    print("  " + "─" * 104)
    for a, toy, std in ROWS:
        print(f"  {a:<30} {toy:<36} {std}")

    print("""
  ──────────────────────────────────────────────────────────────────────
  下面逐条演示后果。
  ──────────────────────────────────────────────────────────────────────
""")

    # ---- 问题 ⑤: return 0 = EOF ----
    print("  " + "-" * 72)
    print("  问题 ⑤ 演示: read() 返回 0 意味着什么")
    print("  " + "-" * 72)

    def led_read_toy(buffer, buffer_len):
        """玩具驱动"""
        on = 1 if buffer[0] > 0 else 0
        return 0            # ← 永远是 0

    def led_read_correct(buffer, buffer_len):
        """正确实现: 返回读到的字节数"""
        if buffer_len < 1:
            return -22      # -EINVAL
        return 1

    print("""
     调用者的常见写法:

         char buf[64];
         ssize_t n;
         while ((n = read(fd, buf, sizeof(buf))) > 0) {
             process(buf, n);
         }

     用玩具驱动跑一遍:
""")
    buf = [1]
    n = led_read_toy(buf, 1)
    print(f"       第一次 read → {n}")
    print(f"       while ({n} > 0) → 条件【假】→ 循环体一次都不执行")
    print(f"       ❌ 调用者以为【文件结束了】")

    print(f"""
     用正确实现跑一遍:
        n = led_read_correct([...], 1) → {led_read_correct([1],1)}
        while ({led_read_correct([1],1)} > 0) → 条件真 → 处理数据 ✓

     ⚠️ 在 POSIX 里, read() 返回 0 的唯一含义是【文件结束 (EOF)】。
        内核和上层库（stdio 的 fread、mavlink 的读取循环）都依赖这个约定。
        返回 0 就是告诉整个系统"这个设备没数据了"。

        玩具驱动之所以"能用", 只是因为 led_main 自己【忽略了返回值】。
        任何用标准循环读取这个设备的人都会踩坑。
""")

    # ---- 问题 ③: 缓冲区长度 ----
    print("  " + "-" * 72)
    print("  问题 ③ 演示: 没有检查 buffer_len")
    print("  " + "-" * 72)
    print("""
     玩具驱动的第一行:

         uint8_t on = *buffer > 0 ? 1 : 0;

     它【无条件解引用 buffer 的第一个字节】。

     危险调用:

         read(fd, buf, 0);        ← 告诉驱动"缓冲区长度是 0"

     在 POSIX 语义下, len=0 时驱动应该【什么都不读】直接返回 0。
     而玩具驱动会去读 buffer[0] —— 如果调用者传的是
     read(fd, NULL, 0) 或者一个长度为 0 的 malloc 结果, 就是空指针解引用。

     正确实现（对照 userled_write）:

         if (len < sizeof(userled_set_t)) {
             lederr("ERROR: buffer too small: %lu\\n", (unsigned long)len);
             return -EINVAL;
         }
         DEBUGASSERT(buffer != NULL);
         ledset = *(userled_set_t *)buffer;
""")

    # ---- 问题 ⑧: 多实例 ----
    print("  " + "-" * 72)
    print("  问题 ⑧ 演示: 想加第二个 LED 会怎样")
    print("  " + "-" * 72)
    print("""
     玩具驱动的所有回调都【硬编码 GPIO_LED_BLUE】:

         static int led_open(file_t* filep) {
           stm32_configgpio(GPIO_LED_BLUE);       ← 写死的
           return 0;
         }

     现在你想加一个 /dev/led2 控制 PA2:

         register_driver("/dev/led2", &led_ops, 0444, NULL);
                                               ↑
                                      i_private 还是 NULL!

     结果: open("/dev/led2") 会去配置 PA1, 而不是 PA2。
     因为驱动【没有渠道】知道用户打开的是哪一个设备。

     标准驱动的做法（对照 userled_upper.c:135-178）:

         static int userled_open(FAR struct file *filep) {
           FAR struct inode *inode;
           FAR struct userled_upperhalf_s *priv;
           FAR struct userled_open_s *opriv;

           DEBUGASSERT(filep && filep->f_inode);
           inode = filep->f_inode;                        ← 拿到 inode
           DEBUGASSERT(inode->i_private);
           priv = (FAR struct userled_upperhalf_s *)inode->i_private;  ← 设备实例!

           ret = userled_takesem(&priv->lu_exclsem);       ← 加锁
           ...
           opriv = kmm_zalloc(sizeof(struct userled_open_s));  ← 每次打开的状态
           ...
           opriv->bo_flink = priv->lu_open;                ← 挂进设备的打开链表
           priv->lu_open = opriv;

           filep->f_priv = (FAR void *)opriv;              ← 挂到 file 上!
           ret = OK;
         }

     ★ 关键点: 数据流是
          filep->f_inode->i_private  →  设备实例（哪个 LED 控制器）
          filep->f_priv             →  本次打开的状态（谁在用）

     有了这两层, 一个驱动就能支持【任意多个实例 + 任意多个打开者】。

  ──────────────────────────────────────────────────────────────────────
  问题 ⑨ 演示: 函数表的 7 个初始化器
  ──────────────────────────────────────────────────────────────────────
""")

    FIELDS_8 = ["open", "close", "read", "write", "seek", "ioctl", "poll",
                "unlink *"]
    toy = ["led_open", "led_close", "led_read", "led_write", "0", "0", "0"]
    std = ["userled_open", "userled_close", "NULL", "userled_write",
           "NULL", "userled_ioctl", "NULL", "NULL"]

    print(f"  struct file_operations 的真实定义（include/nuttx/fs/fs.h:204）:\n")
    print(f"    {'#':<4} {'字段':<12} {'玩具驱动':<16} {'标准驱动':<16}")
    print("    " + "─" * 52)
    for i, fld in enumerate(FIELDS_8):
        t = toy[i] if i < len(toy) else "(未写 → 隐式 0)"
        s = std[i] if i < len(std) else "—"
        print(f"    {i:<4} {fld:<12} {t:<16} {s:<16}")

    print("""
    玩具驱动只写了 7 个 → 第 8 个字段 unlink 被 C 语言的
    "省略即零初始化"规则隐式设成 0 (NULL)。

    ★ 所以它【能跑】—— 但靠的是巧合:

       1. struct 字段顺序恰好没变过
       2. unlink = NULL 恰好是我们想要的值

    如果将来 NuttX 在中间插入一个字段（比如把 poll 拆成 poll + notify）,
    所有位置式初始化的驱动都会【静默错位】—— 编译不报错, 运行时才崩。

    ✅ 防御写法一: 用指定初始化器 (C99)

         static const struct file_operations led_ops = {
             .open  = led_open,
             .close = led_close,
             .read  = led_read,
             .write = led_write,
             .ioctl = led_ioctl,
             /* 其余字段自动为 NULL */
         };

    ✅ 防御写法二: 像 NuttX 自己那样, 每个字段都写 + 注释 + 条件编译

         static const struct file_operations userled_fops = {
           userled_open,   /* open */
           userled_close,  /* close */
           NULL,           /* read */
           userled_write,  /* write */
           NULL,           /* seek */
           userled_ioctl,  /* ioctl */
           NULL            /* poll */
         #ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
           , NULL          /* unlink */
         #endif
         };
""")


# ============================================================================
#  实验 5: ioctl 命令号编码
# ============================================================================
def exp5_ioctl_encoding():
    sep("实验 5: ioctl 命令号编码 —— 比 Linux 简单得多")

    print("""
  NuttX 的 ioctl 编码（include/nuttx/fs/ioctl.h:103-107）:

      #define _IOC_MASK       (0x00ff)
      #define _IOC_TYPE(cmd)  ((cmd) & ~_IOC_MASK)     /* 高字节 */
      #define _IOC_NR(cmd)    ((cmd) & _IOC_MASK)      /* 低字节 */
      #define _IOC(type,nr)   ((type)|(nr))

  ⚠️ 注意这和 Linux 完全不同!

     Linux:  32 位里打包了 方向(2bit) + 大小(14bit) + 类型(8bit) + 序号(8bit)
     NuttX:  只有 类型(高字节) + 序号(低字节) —— 简单到可以手算

  ┌──────────────────────────────────────────────────────────────┐
  │   15        8  7              0                              │
  │   ┌──────────┬────────────────┐                              │
  │   │  TYPE    │      NR        │     16 位视图                 │
  │   └──────────┴────────────────┘                              │
  │   （NuttX 的 cmd 是 int, 实际只用低 16 位）                    │
  └──────────────────────────────────────────────────────────────┘
""")

    def _IOC(ty, nr):
        return ty | nr
    def _IOC_TYPE(cmd):
        return cmd & ~0x00ff
    def _IOC_NR(cmd):
        return cmd & 0x00ff

    BASES = [
        (0x0100, "_TIOCBASE", "Terminal I/O"),
        (0x0200, "_WDIOCBASE", "Watchdog"),
        (0x0300, "_FIOCBASE", "File system"),
        (0x0400, "_DIOCBASE", "字符驱动"),
        (0x0500, "_BIOCBASE", "Block driver"),
        (0x0600, "_MTDIOCBASE", "MTD"),
        (0x0700, "_SIOCBASE", "Socket"),
        (0x1d00, "_ULEDBASE", "User LED  ★"),
        (0x1e00, "_ZCBASE", "Zero Cross"),
        (0x2200, "_SPIBASE", "SPI"),
        (0x2300, "_GPIOBASE", "GPIO"),
        (0xff00, "_BOARDBASE", "boardctl"),
    ]

    print("  === NuttX 注册的 ioctl 类型（节选）===\n")
    print(f"  {'BASE':<8} {'宏名':<14} {'用途':<16} 取值范围")
    print("  " + "─" * 62)
    for base, name, use in BASES:
        lo, hi = _IOC(base, 0x01), _IOC(base, 0xff)
        print(f"  0x{base:04x}   {name:<14} {use:<16} 0x{lo:04x} ~ 0x{hi:04x}")

    print(f"""
  ★ 每个类型有 255 个命令号可用（0x01 ~ 0xff）。
    0x00 被留空 —— 这样 0 就成了一个"明显的非法值"。

  === 真实命令举例 ===

  File system 类（_FIOCBASE = 0x0300）:
""")
    FIOC = [("FIOC_MMAP", 0x0001, "取得文件的内存映射地址"),
            ("FIOC_REFORMAT", 0x0002, "重新格式化"),
            ("FIOC_FILEPATH", 0x0004, "取得 fd 对应的完整路径"),
            ("FIONREAD", 0x0007, "还有多少字节可读"),
            ("FIONWRITE", 0x0008, "还有多少字节待写"),
            ("FIONBIO", 0x000b, "设置非阻塞模式"),
            ("FIOCLEX", 0x000c, "close-on-exec 标志")]
    for name, nr, desc in FIOC:
        cmd = _IOC(0x0300, nr)
        print(f"    {name:<16} = _FIOC(0x{nr:04x}) = 0x{cmd:04x}   {desc}")

    print(f"""
  User LED 类（_ULEDBASE = 0x1d00）:
    就是 led 驱动【应该】用的命令空间
""")
    ULED = [("ULEDIOC_SETALL", 0x0001),
            ("ULEDIOC_SUPPORTED", 0x0002)]
    for name, nr in ULED:
        print(f"    {name:<20} = 0x{_IOC(0x1d00, nr):04x}")

    # ---- 编解码演示 ----
    print(f"""
  ──────────────────────────────────────────────────────────────────────
  编码 / 解码 演示
  ──────────────────────────────────────────────────────────────────────
""")
    for base, name, use in [(0x0300, "_FIOCBASE", "File system"),
                            (0x1d00, "_ULEDBASE", "User LED"),
                            (0xff00, "_BOARDBASE", "boardctl")]:
        for nr in (0x01, 0x0b, 0xff):
            cmd = _IOC(base, nr)
            ty = _IOC_TYPE(cmd)
            n = _IOC_NR(cmd)
            valid = "✓" if ty == base else "✗"
            print(f"    _IOC(0x{base:04x}, 0x{nr:02x}) = 0x{cmd:04x}   "
                  f"→  _IOC_TYPE=0x{ty:04x} {valid} {name:<12} "
                  f"_IOC_NR=0x{n:02x}")
        print()

    print("""  === 驱动里的标准写法 ===

      static int mydrv_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
      {
        FAR struct mydrv_s *priv = filep->f_inode->i_private;
        int ret = OK;

        /* ① 先判断这个 cmd 是不是给我的 */

        if (_IOC_TYPE(cmd) != MYDRV_BASE)      /* MYDRV_BASE = 自定义的 0x???? */
          {
            return -ENOTTY;                     /* 不是我的命令 */
          }

        /* ② 再按命令号分派 */

        switch (_IOC_NR(cmd))
          {
            case MYDRV_SET_RATE:
              priv->rate = (uint32_t)arg;
              break;

            case MYDRV_GET_STATE:
              *(FAR struct mydrv_state_s *)arg = priv->state;   /* arg 是指针! */
              break;

            default:
              ret = -ENOTTY;
              break;
          }

        return ret;
      }

  ★ 三个容易忘的点:

     ① 返回 -ENOTTY（不是 -EINVAL）表示"这个命令我不认识"
     ② arg 通常是【指针】，要转换成正确的类型再解引用
     ③ 一定要有 default 分支返回 -ENOTTY —— 否则未知命令静默成功

  ★ 自定义 BASE 的选取:

     NuttX 已经用掉了 0x0100 ~ 0x3300 的大部分。
     你要写自己的驱动时有两个选择:

       a) 复用已有的语义匹配的 BASE（比如传感器用 _SNIOCBASE = 0x0a00）
       b) 在【自己的头文件】里定义一个新的, 比如

            #define _MYDRVBASE   (0x3400)
            #define _MYDRVIOCVALID(c)  (_IOC_TYPE(c) == _MYDRVBASE)
            #define _MYDRVIOC(nr)      _IOC(_MYDRVBASE, nr)

          然后向 NuttX 上游提 PR 把它加进 include/nuttx/fs/ioctl.h

     ⚠️ 不要随便挑一个没用过的数字就用 —— 会和未来的 NuttX 版本冲突。
        上游的 0x3300 (_MMCSDIOBASE) 之后还有空间, 但要以官方分配为准。
""")


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  NuttX 教程 · 第 6 课 · 设备驱动与文件系统")
    print("=" * 76)

    exp1_fd_and_path()
    exp2_register()
    exp3_call_chain()
    exp4_toy_vs_standard()
    exp5_ioctl_encoding()

    sep("本课小结")
    print("""
  1. NuttX 设备模型: 一切皆文件
     open/read/write/ioctl/close → 一套统一的 VFS 接口

  2. 三层数据结构各司其职:
     fd ──► struct file ──► struct inode ──► file_operations
             每次open新建    全局唯一(注册时)   驱动的静态函数表
             f_pos/f_priv    i_ops/i_private

  3. register_driver(path, fops, mode, priv)
     · priv 就是 inode->i_private —— 驱动拿设备实例的唯一渠道
     · 路径重复 → -EEXIST（不覆盖）
     · 应该在 board bringup 里调用, 不是在使用时

  4. struct file_operations 有 8 个字段（unlink 条件编译）
     位置式初始化很危险 → 用 .open = / .read = 这种指定初始化器

  5. read/write 的返回值约定:
     · read 返回 0 = 【EOF】, 不是"成功"
     · write 返回实际处理的字节数
     · 违反了上层循环逻辑就会静默出错

  6. 标准驱动必备:
     每-open 状态 (f_priv) + 设备实例 (i_private) + 设备锁 (sem_t)
     + 长度检查 + 错误路径清理 (goto)

  7. ioctl 编码: 高字节 = 类型, 低字节 = 命令号
     _IOC_TYPE(cmd) 判断归属, _IOC_NR(cmd) 分派
     未知命令返回 -ENOTTY

  8. 内建命令链: ROMFS rcS → NSH 查 g_builtins[] → <MAIN>_main()
     g_builtins[] 由 CMake 从 fmu_add_module 的 MAIN 属性生成

  下一课: 07-中断处理与下半部.md
""")
