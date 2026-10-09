"""
============================================================================
 卷 6 第 8 章 · 动手写一个 PX4 模块
 配套实验代码
============================================================================

 运行:  python lesson09_custom_module.py

 五个实验:
   1. 模块启动全流程      —— 从 nsh 命令到 Run() 执行
   2. uORB 发布/订阅      —— 环形缓冲 + 多订阅者游标
   3. 参数系统            —— 内存缓存 + flash 持久化 + 变更通知
   4. 构建系统解析        —— .px4board → config_module_list
   5. 模块骨架生成器      —— 生成可复制的文件

 对照的真实源码 (PX4 1.15.4):
   src/templates/template_module/        独立任务型模板
   src/examples/work_item/               工作队列型范例 ★推荐
   cmake/kconfig.cmake:170-205           .px4board → 模块列表
   cmake/px4_add_module.cmake            模块构建宏
   CMakeLists.txt:434                    add_subdirectory(src/${module})
   msg/                                  uORB topic 定义
============================================================================
"""


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


def w(s):
    return sum(2 if ord(c) > 0x1100 and (
        0x4E00 <= ord(c) <= 0x9FFF or 0x3000 <= ord(c) <= 0x303F or
        0xFF00 <= ord(c) <= 0xFF60 or 0x2500 <= ord(c) <= 0x257F or
        0x2190 <= ord(c) <= 0x21FF or 0x2600 <= ord(c) <= 0x27BF) else 1
        for c in s)


def pad(s, width, align="left"):
    d = max(0, width - w(s))
    return s + " " * d if align == "left" else " " * d + s


# ============================================================================
#  实验 1: 模块启动全流程
# ============================================================================
def exp1_module_startup():
    sep("实验 1: 模块启动全流程 —— 从 nsh 命令到 Run() 执行")

    print("""
  用户视角只有一行:

      nsh> my_module start

  背后发生了什么? 下面逐步展开。
""")

    STEPS = [
        ("NSH 解析", "nsh_parse() 拆出 argv = [\"my_module\", \"start\"]"),
        ("查内建表", "在 g_builtins[] 里找到 { \"my_module\", 100, 2048, my_module_main }"),
        ("创建任务", "exec_builtin() → task_spawn() 起一个临时任务跑 my_module_main"),
        ("", "（注意: 这个任务跑完就退出, 【不】是模块自己的任务）"),
        ("main()", "my_module_main() → ModuleBase<MyModule>::main(argc, argv)"),
        ("", "argv[1] == \"start\" → start_command_base(argc-1, argv+1)"),
        ("加锁", "lock_module()  ← px4_modules_mutex（全局唯一一把）"),
        ("查重", "is_running()?  _task_id != -1  →  已经在跑就报错返回"),
        ("task_spawn", "MyModule::task_spawn()  ← 派生类实现"),
        ("", "new MyModule()"),
        ("", "_object.store(instance)"),
        ("", "_task_id = task_id_is_work_queue   (−2)"),
        ("", "instance->init()  ← 注册订阅回调 / 排定周期"),
        ("解锁", "unlock_module()"),
        ("返回", "nsh 提示符回来, 但模块已经在后台跑了"),
        ("", ""),
        ("【异步部分】", ""),
        ("HRT 中断", "hrt_call_every 到点 → schedule_trampoline()"),
        ("", "ScheduleNow()  →  WorkQueue::Add(item)"),
        ("", "关中断入队 + sem_post 唤醒 wq 线程"),
        ("wq 线程", "WorkQueue::Run() 取出 item"),
        ("", "work->Run()  →  MyModule::Run()   ★ 你的代码在这里跑"),
    ]

    for what, detail in STEPS:
        if not what and not detail:
            print()
            continue
        if what.startswith("【"):
            print(f"\n  {what}")
            continue
        print(f"  {pad(what,16)} {detail}")

    print("""
  ────────────────────────────────────────────────────────────────────
  【三个容易混淆的点】

   ① `my_module_main` 是【整个模块的入口】, 不是任务入口
      它跑在一个临时任务里, 负责解析命令后就退出。

   ② 模块自己的执行体是 `Run()`（工作队列型）或 `run()`（独立任务型）
      两者的区别:
         工作队列型: 跑在共享的 wq:xxx 线程上, 没有自己的任务
         独立任务型: 有自己的任务, 阻塞在自己的 run() 里

   ③ `start` 是异步的
      nsh 提示符回来后模块可能还没跑第一轮 Run()。
      需要等的话用 wait_until_running()。

  ────────────────────────────────────────────────────────────────────
  【两种形态的 task_spawn 对比】

    ✅ 工作队列型（推荐, src/examples/work_item 的写法）:

        int WorkItemExample::task_spawn(int argc, char *argv[])
        {
            WorkItemExample *instance = new WorkItemExample();

            if (instance) {
                _object.store(instance);
                _task_id = task_id_is_work_queue;      ← −2

                if (instance->init()) {
                    return PX4_OK;
                }
            } else {
                PX4_ERR("alloc failed");
            }

            delete instance;
            _object.store(nullptr);
            _task_id = -1;

            return PX4_ERROR;
        }

     简单: 不需要 instantiate()、不需要 wait_until_running()。

    ⚠️ 独立任务型（src/templates/template_module 的写法）:

        int TemplateModule::task_spawn(int argc, char *argv[])
        {
            _task_id = px4_task_spawn_cmd("module",
                          SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1024,
                          (px4_main_t)&run_trampoline,      ← 模板基类的跳板
                          (char *const *)argv);

            if (_task_id < 0) {
                _task_id = -1;
                return -errno;
            }

            return 0;
        }

        TemplateModule *TemplateModule::instantiate(int argc, char *argv[])
        {
            // 运行在【新线程】里, 由 run_trampoline() 调用
            // 解析参数 → new → 返回
        }

     复杂: 必须有 instantiate(), 参数解析在那边。
""")

    print("  " + "-" * 72)
    print("  ⚠️ 工作队列型必须自己处理退出")
    print("  " + "-" * 72)
    print("""
     📁 src/examples/work_item/WorkItemExample.cpp

         void WorkItemExample::Run()
         {
             if (should_exit()) {
                 ScheduleClear();
                 exit_and_cleanup();        ← ★ 必须自己调!
                 return;
             }

             perf_begin(_loop_perf);
             ...
         }

     为什么?
       独立任务型: run_trampoline() 在 run() 返回后【自动】调 exit_and_cleanup()
       工作队列型: 没有 run_trampoline —— 所以要自己调

     ⚠️ 忘了调会怎样:
        _task_id 永远是 −2, `my_module stop` 会等满 5 秒然后报超时。
""")


# ============================================================================
#  实验 2: uORB 发布/订阅
# ============================================================================
def exp2_uorb():
    sep("实验 2: uORB 发布/订阅 —— 环形缓冲 + 多订阅者游标")

    print("""
  uORB 的核心设计: 【一份数据, N 个独立读者】。

  和消息队列（mqueue）的关键区别:

     mqueue:  消息被读了就【删掉】—— 一个消息只能给一个读者
     uORB:    数据留在环形缓冲里 —— 每个订阅者有自己的游标
""")

    print("  " + "-" * 72)
    print("  结构")
    print("  " + "-" * 72)
    print("""
     📁 platforms/common/uORB/uORBDeviceNode.hpp:284

         px4::atomic<unsigned>  _generation{0};   ← 全局代际计数

     📁 platforms/common/uORB/uORBDeviceNode.cpp:188-208（发布）

         ATOMIC_ENTER;                                    ← 关中断
         unsigned generation = _generation.fetch_add(1);   ← 原子增
         memcpy(_data + (_meta->o_size * (generation % _meta->o_queue)),
                buffer, _meta->o_size);
         ...
         ATOMIC_LEAVE;

     所以:

         _data 是一个环形缓冲, 长度 = o_size × o_queue
         _generation 是"总共发布了多少次"
         写的位置 = generation % o_queue
""")

    # ---- 模拟 ----
    class Topic:
        def __init__(self, name, queue_len, size):
            self.name = name
            self.queue = queue_len
            self.size = size
            self.data = [None] * queue_len
            self.generation = 0
            self.subs = []
            self.log = []

        def advertise(self, name):
            self.log.append(f"      advertise({self.name}) → 创建 DeviceNode")

        def subscribe(self, name):
            # 新订阅者从"当前最新"开始读
            s = {"name": name, "last_gen": self.generation}
            self.subs.append(s)
            self.log.append(f"      {name} 订阅, 起始游标 = {self.generation}")
            return s

        def publish(self):
            self.generation += 1
            slot = self.generation % self.queue
            self.data[slot] = self.generation
            self.log.append(
                f"      publish #{self.generation}  "
                f"→ 写入槽位 [{slot}]  "
                f"(generation={self.generation}, {self.generation}%{self.queue}={slot})")

        def updated(self, s):
            return self.generation != s["last_gen"]

        def copy(self, s):
            if not self.updated(s):
                return None
            s["last_gen"] = self.generation
            latest_slot = self.generation % self.queue
            return self.data[latest_slot]

    print("  " + "-" * 72)
    print("  演示: 一个发布者 + 两个订阅者")
    print("  " + "-" * 72)

    t = Topic("sensor_accel", queue_len=8, size=48)
    t.advertise("sensor_accel")
    for line in t.log: print(line)
    t.log.clear()

    A = t.subscribe("订阅者 A")
    B = t.subscribe("订阅者 B")
    for line in t.log: print(line)
    t.log.clear()

    print("\n      发布者连续发布 5 次:")
    for _ in range(5):
        t.publish()
    for line in t.log: print(line)
    t.log.clear()

    print(f"\n      A.updated() → {t.updated(A)}   "
          f"(A.last_gen={A['last_gen']}, generation={t.generation})")
    print(f"      B.updated() → {t.updated(B)}")
    print(f"\n      A.copy() → {t.copy(A)}      ← A 拿到 #5")
    print(f"      A.copy() → {t.copy(A)}      ← 再 copy 还是 #5（不消耗）")
    print(f"      B.copy() → {t.copy(B)}      ← B 也拿到 #5，互不影响")

    print("\n      发布者再发布 2 次:")
    t.log.clear()
    t.publish()
    t.publish()
    for line in t.log: print(line)

    print(f"\n      A.copy() → {t.copy(A)}      ← A 拿到 #7（跳过了 #6）")
    print(f"      B.copy() → {t.copy(B)}")

    print(f"""
  ────────────────────────────────────────────────────────────────────
  【关键认识】

   ① copy() 不消耗数据
      同一个数据可以被多个订阅者各读一次。
      这是 uORB 相对 mqueue 最本质的优势。

   ② 订阅者会【跳过中间值】
      A 上次读到 #5, 现在 generation=7, copy 拿到的是 #7。
      中间发布的 #6 被跳过了 —— 因为 uORB 是"保留最新值"语义。

      ⚠️ 这对高频 topic 很重要:
         如果订阅者处理慢, 它会丢数据。
         需要不丢的话要么加大 o_queue, 要么提高订阅者的执行频率。

   ③ updated() 和 copy() 的分工

         if (_sub.updated()) {{          ← 先问"有新数据吗"（不改变状态）
             sensor_accel_s data;
             _sub.copy(&data);          ← 再取（会更新 last_gen）
         }}

      ⚠️ 只调 copy() 不调 updated() 也可以（copy 返回 bool），
         但 copy 会【无条件尝试】读取，略微浪费。

   ④ 写的位置是有讲究的

         publish #5 → 槽位 5
         publish #6 → 槽位 6
         ...
         publish #8 → 槽位 0   ← 回绕, 覆盖 #0
         publish #9 → 槽位 1

      缓冲长度 8 → 最多保留【最近 8 条】。
      订阅者如果落后超过 8 条, 数据就被覆盖了（旧的丢失）。

  ────────────────────────────────────────────────────────────────────
  【o_queue 的选择】

     📁 各 msg 文件里的 ORB_QUEUE_LENGTH:

         SensorGyro.msg            8     高频传感器
         SensorAccel.msg           8
         SensorBaro.msg            4
         Event.msg                16     事件不能丢
         ParameterSetUsedRequest  64     参数同步要缓冲

     太小的后果: 订阅者慢了就丢数据 → 估计器输入有洞
     太大的后果: 内存浪费 + "陈旧数据"风险
""")

    # ---- 字段排序陷阱 ----
    print("  " + "-" * 72)
    print("  ⚠️ 陷阱: 生成的结构体【字段顺序会变】")
    print("  " + "-" * 72)
    print("""
     📁 Tools/msg/templates/uorb/msg.h.em:97-105

         def print_parsed_fields():
             # sort fields (using a stable sort)
             sorted_fields = sorted(spec.parsed_fields(),
                                    key=sizeof_field_type, reverse=True)
             struct_size, padding_end_size = add_padding_bytes(sorted_fields, search_path)
             for field in sorted_fields:
                 if (not field.is_header):
                     print_field_def(field)

     ★ 【按字段大小降序重排】，末尾自动补 _padding 让总大小是 8 的倍数。

     为什么这么做: 减少结构体内部的填充字节（padding），省内存。
""")

    SIZES = {"uint64": 8, "int64": 8, "float64": 8,
             "uint32": 4, "int32": 4, "float32": 4,
             "uint16": 2, "int16": 2,
             "uint8": 1, "int8": 1, "bool": 1, "char": 1}

    def sizeof(t):
        if t.endswith("]"):
            base, n = t[:-1].split("[")
            return SIZES[base] * int(n)
        return SIZES.get(t, 8)

    decl = [
        ("uint64", "timestamp"),
        ("uint64", "timestamp_sample"),
        ("float32[4]", "q"),
        ("float32[4]", "delta_q_reset"),
        ("uint8", "quat_reset_counter"),
    ]
    gen = sorted(decl, key=lambda f: sizeof(f[0]), reverse=True)

    print(f"     以 msg/VehicleAttitude.msg 为例:\n")
    print(f"     {'#':<3} {'.msg 里写的顺序':<26} {'生成的结构体顺序':<26} {'字节':>5}")
    print("     " + "─" * 68)
    for i in range(len(decl)):
        d = f"{decl[i][0]} {decl[i][1]}"
        g = f"{gen[i][0]} {gen[i][1]}"
        mark = "" if decl[i][1] == gen[i][1] else "  ←"
        print(f"     {i:<3} {d:<26} {g:<26} {sizeof(gen[i][0]):>5}{mark}")

    print(f"""
     ⚠️ 【q】从第 3 位跑到了第 1 位, timestamp 从第 1 位掉到第 3 位。

  ┌────────────────────────────────────────────────────────────────┐
  │  ★ 后果: 【绝对不要】假设 .msg 顺序 == 内存顺序                   │
  │                                                                │
  │     ❌ 错误做法:                                                │
  │         memcpy(&q, buf + 16, 16);        // 手工算偏移          │
  │         offsetof(vehicle_attitude_s, q)  // 假设偏移            │
  │         用 {{1.0f, 0.0f, 0.0f, 0.0f, ...}} 位置式初始化         │
  │                                                                │
  │     ✅ 正确做法:                                                │
  │         vehicle_attitude_s att{{}};                             │
  │         att.timestamp = hrt_absolute_time();   // 用字段名      │
  │         att.q[0] = 1.0f;                       // 用字段名      │
  │                                                                │
  │     编译器按【生成的】结构体定义解析字段名, 所以用名字永远对。   │
  └────────────────────────────────────────────────────────────────┘

     好处（为什么生成器要重排）:
       不重排的话, 编译器还要自己插填充字节 —— 结构体更大。
       重排 + 手工 _padding 能让结构体最紧凑。
""")

    # 队列长度对比
    print("  " + "-" * 72)
    print("  量化: 队列长度能容忍多少延迟")
    print("  " + "-" * 72)
    RATES = [("IMU (1 kHz)", 1000), ("气压计 (100 Hz)", 100), ("GPS (10 Hz)", 10)]
    print(f"\n    {'topic':<20} {'发布率':>10} {'队列':>6} {'能容忍的订阅者延迟':>20}")
    print("    " + "─" * 62)
    for name, hz in RATES:
        for q in [4, 8, 16]:
            ms = q / hz * 1000
            print(f"    {name:<20} {hz:>7} Hz {q:>6} {ms:>17.1f} ms")
        print()


# ============================================================================
#  实验 3: 参数系统
# ============================================================================
def exp3_parameters():
    sep("实验 3: 参数系统 —— 内存缓存 + flash 持久化 + 变更通知")

    print("""
  PX4 的参数系统要同时满足四个要求:

    ① 读得【快】—— 控制回路里要频繁读参数
    ② 掉电不丢   —— 必须存在 flash 里
    ③ 能【通知】 —— 参数变了相关模块要知道
    ④ 能【迁移】 —— 固件升级后参数要兼容

  它用三层设计满足这些要求。
""")

    print("  " + "-" * 72)
    print("  三层结构")
    print("  " + "-" * 72)
    print("""
     ┌─────────────────────────────────────────────────────────────┐
     │  第 1 层: 编译期枚举                                          │
     │                                                             │
     │    module.yaml  ──生成──►  *_params.c                       │
     │                          ──扫描──►  parameters.xml          │
     │                                     ──生成──►  px4_parameters.hpp
     │                                                             │
     │    enum class px4::params {                                 │
     │        SYS_AUTOSTART,                                       │
     │        MYMOD_GAIN,          ← 你的参数                       │
     │        ...                                                  │
     │    }                                                        │
     │                                                             │
     │    ★ 参数名在【编译期】就变成了枚举值 → 零字符串查找开销        │
     ├─────────────────────────────────────────────────────────────┤
     │  第 2 层: 内存缓存                                            │
     │                                                             │
     │    每个 Param<T> 对象里有一份【值副本】:                      │
     │                                                             │
     │        template<> class Param<float, p> {                   │
     │            float _val;                                      │
     │            Param() { param_set_used(handle()); update(); }  │
     │            float get() const { return _val; }  ← 极快       │
     │            void  update();                     ← 从存储刷新 │
     │        };                                                   │
     │                                                             │
     │    ★ get() 只是读一个成员变量 —— 无锁、无 IO                 │
     ├─────────────────────────────────────────────────────────────┤
     │  第 3 层: Flash 存储                                          │
     │                                                             │
     │    脏参数 → 后台异步写入 flash                                │
     │    用【裸 flash 接口】不做文件系统（卷 6 第 7 章）                  │
     │    双备份 + CRC + 原子切换                                    │
     └─────────────────────────────────────────────────────────────┘
""")

    # ---- 模拟 ----
    class ParamStore:
        """模拟 PX4 的参数存储"""

        def __init__(self):
            self.flash = {}          # "flash" 里的持久值
            self.ram = {}            # 内存缓存
            self.used = set()        # 被"用过"的参数（autosave 只存这些）
            self.dirty = set()
            self.subs = []           # 订阅变更的模块
            self.log = []

        def define(self, name, default, ptype="int32"):
            self.flash[name] = default      # "上电时从 flash 读"
            self.ram[name] = default
            self.log.append(f"    [定义] {pad(name,20)} 默认 {default}")

        def find(self, name):
            """模拟 param_find —— 二分查找（已有编译期枚举, 这里是运行时兜底）"""
            if name in self.ram:
                self.used.add(name)
                return name
            return None

        def get(self, h):
            """极快: 只读内存副本"""
            return self.ram.get(h)

        def set(self, h, val, notify=True):
            old = self.ram.get(h)
            self.ram[h] = val
            self.dirty.add(h)
            self.log.append(f"    [设置] {pad(h,20)} {old} → {val}"
                            + (f"   (通知 {len(self.subs)} 个订阅者)" if notify else "  (不通知)"))
            if notify:
                for s in self.subs:
                    s(h, val)

        def save(self):
            """模拟参数落盘"""
            if not self.dirty:
                self.log.append("    [保存] 没有脏参数")
                return
            for h in self.dirty:
                self.flash[h] = self.ram[h]
            self.log.append(f"    [保存] 写入 flash: {sorted(self.dirty)}")
            self.dirty.clear()

        def reboot(self):
            """模拟重启: 内存清空, 从 flash 重新加载"""
            self.ram = dict(self.flash)
            self.dirty.clear()
            self.log.append("    [重启] 内存缓存从 flash 重建")

    store = ParamStore()

    print("  " + "-" * 72)
    print("  场景: 定义一个参数, 改它, 重启看是否还在")
    print("  " + "-" * 72)

    # 注册变更通知
    notified = []
    store.subs.append(lambda h, v: notified.append((h, v)))

    store.define("MYMOD_GAIN", 1.0, "float")
    store.define("MYMOD_RATE", 50, "int32")
    for line in store.log: print(line)
    store.log.clear()

    print(f"\n    nsh> param show MYMOD_GAIN")
    print(f"      MYMOD_GAIN = {store.get('MYMOD_GAIN')}")

    print(f"\n    nsh> param set MYMOD_GAIN 2.5")
    store.set("MYMOD_GAIN", 2.5)
    for line in store.log: print(line)
    store.log.clear()
    print(f"      → 订阅者收到: {notified}")
    notified.clear()

    print(f"""
    ★ 注意 set 之后【内存立刻生效】(get 得到 2.5),
      但 flash 还没写 —— 这时掉电会丢。
""")

    print(f"    nsh> param save")
    store.save()
    for line in store.log: print(line)
    store.log.clear()

    print(f"\n    nsh> param set MYMOD_RATE 100      (故意不 save)")
    store.set("MYMOD_RATE", 100)
    for line in store.log: print(line)
    store.log.clear()

    print(f"\n    --- 模拟断电重启 ---")
    store.reboot()
    for line in store.log: print(line)
    store.log.clear()

    print(f"""
    重启后:
      MYMOD_GAIN = {store.get('MYMOD_GAIN')}    ← 保存过, 还在 ✓
      MYMOD_RATE = {store.get('MYMOD_RATE')}     ← 没保存, 回到默认 ✗

  ────────────────────────────────────────────────────────────────────
  【这就是"改了参数重启就没了"的原因】

     参数只在【以下时机】真正写入 flash:
       · nsh> param save
       · 关机时（自动保存）
       · 某些模块显式调用 param_set 后触发 autosave

     ⚠️ 调试时改参数记得 `param save`, 否则重启就白改了。

  ────────────────────────────────────────────────────────────────────
  【参数名的 16 字符限制】

     为什么是 16 字符? 因为 param_t 和存储格式的历史设计。

         ✅ SYS_AUTOSTART      (13)
         ✅ COM_FLTMODE1       (12)
         ✅ MYMOD_GAIN         (10)
         ❌ MY_REALLY_LONG_PARAMETER_NAME   → 会被截断/报错

     命名规范:
       · 3~6 字符的模块前缀 (SYS_ / COM_ / MC_ / EKF2_ / 你的模块名)
       · 全大写 + 下划线
       · 语义清晰但不啰嗦

  ────────────────────────────────────────────────────────────────────
  【参数变更通知机制】

     订阅 parameter_update topic:

         uORB::SubscriptionInterval _parameter_update_sub{{ORB_ID(parameter_update), 1_s}};

         void MyModule::Run()
         {{
             if (_parameter_update_sub.updated()) {{
                 parameter_update_s update;
                 _parameter_update_sub.copy(&update);
                 updateParams();        ← 刷新所有 DEFINE_PARAMETERS
             }}
         }}

     ★ 为什么用 SubscriptionInterval(1_s) 而不是普通订阅?
       因为参数变更可能【很频繁】（比如地面站批量下发），
       限流到 1 Hz 避免频繁刷新。

       ⚠️ 注意: updateParams() 刷新的是本模块【自己的】参数副本。
          Param<T>::get() 读的是缓存, 不刷新就还是旧值。
""")


# ============================================================================
#  实验 4: 构建系统
# ============================================================================
def exp4_build_system():
    sep("实验 4: 构建系统 —— .px4board 到 add_subdirectory")

    print("""
  PX4 的模块是【选择性编译】的 —— 不是所有模块都进固件。
  控制这条的是一条完整链路。
""")

    print("  " + "-" * 72)
    print("  完整链路")
    print("  " + "-" * 72)
    print("""
     ┌───────────────────────────────────────────────────────────────┐
     │ ① src/modules/<name>/Kconfig                                  │
     │                                                               │
     │      menuconfig MODULES_<NAME>                                │
     │          bool "<name>"                                        │
     │          default n               ← 默认【不】编译              │
     │                                                               │
     │    ★ src/modules/Kconfig:1 只有一行:                          │
     │          rsource "*/Kconfig"                                  │
     │      → 自动递归包含所有子目录的 Kconfig, 不用手工注册          │
     ├───────────────────────────────────────────────────────────────┤
     │ ② boards/px4/fmu-v6x/default.px4board                        │
     │                                                               │
     │      CONFIG_MODULES_COMMANDER=y                               │
     │      CONFIG_MODULES_NAVIGATOR=y                               │
     │      CONFIG_MODULES_MY_MODULE=y     ← 你加的这一行            │
     ├───────────────────────────────────────────────────────────────┤
     │ ③ cmake/kconfig.cmake:179-205                                 │
     │                                                               │
     │      解析每一行 CONFIG_MODULES_xxx=y:                         │
     │        · 去掉前缀 CONFIG_MODULES_                             │
     │        · 转小写 → my_module                                   │
     │        · 下划线变斜杠 → my/module                             │
     │        · 依次尝试几种目录布局                                  │
     │        · 找不到 → FATAL_ERROR                                 │
     │                                                               │
     │      → config_module_list += modules/my_module                │
     ├───────────────────────────────────────────────────────────────┤
     │ ④ CMakeLists.txt:434                                          │
     │                                                               │
     │      foreach(module ${config_module_list})                    │
     │          add_subdirectory(src/${module})                      │
     │      endforeach()                                             │
     ├───────────────────────────────────────────────────────────────┤
     │ ⑤ src/modules/<name>/CMakeLists.txt                          │
     │                                                               │
     │      px4_add_module(                                          │
     │          MODULE modules__my_module                            │
     │          MAIN my_module          ← nsh 命令名                 │
     │          SRCS MyModule.cpp MyModule.hpp                       │
     │      )                                                        │
     ├───────────────────────────────────────────────────────────────┤
     │ ⑥ platforms/nuttx/NuttX/CMakeLists.txt:87-102                 │
     │                                                               │
     │      生成 builtin 表项:                                        │
     │          { "my_module", 100, 2048, my_module_main },          │
     │      → nsh 里就能敲 `my_module` 了                            │
     └───────────────────────────────────────────────────────────────┘
""")

    # ---- 模拟 kconfig.cmake 的解析 ----
    print("  " + "-" * 72)
    print("  模拟: kconfig.cmake 怎么把 .px4board 的一行变成目录")
    print("  " + "-" * 72)

    def exists(path):
        """模拟文件系统检查（只包含真实存在的模块）"""
        return path in {
            "src/modules/commander", "src/modules/navigator",
            "src/modules/sensors", "src/modules/logger",
            "src/modules/my_module",
            "src/modules/ekf2",
        }

    def parse_board_line(line):
        """模拟 cmake/kconfig.cmake:179-205"""
        name = line.split("=")[0]

        if not name.startswith("CONFIG_MODULES"):
            return None, "不是 MODULES 项"

        module = name.replace("CONFIG_MODULES_", "").lower()
        module_path = module.replace("_", "/")

        # 依次尝试几种布局（真实代码里有 5 种）
        for cand in [module, module_path]:
            if exists(f"src/modules/{cand}"):
                return f"modules/{cand}", None

        return None, f"FATAL_ERROR: Couldn't find path for {module}"

    LINES = [
        "CONFIG_MODULES_COMMANDER=y",
        "CONFIG_MODULES_NAVIGATOR=y",
        "CONFIG_MODULES_SENSORS=y",
        "CONFIG_MODULES_MY_MODULE=y",
        "CONFIG_DRIVERS_GPS=y",
        "CONFIG_MODULES_TYPO_MODULE=y",
    ]

    print(f"\n    {'default.px4board 的一行':<38} {'解析结果':<28} 状态")
    print("    " + "─" * 88)
    modlist = []
    for line in LINES:
        res, err = parse_board_line(line)
        if err is None:
            modlist.append(res)
            print(f"    {pad(line,38)} {pad(res,28)} ✅")
        elif err.startswith("不是"):
            print(f"    {pad(line,38)} {pad('(跳过)',28)} —")
        else:
            print(f"    {pad(line,38)} {pad('—',28)} ❌ {err}")

    print(f"\n    config_module_list = {modlist}")
    print(f"""
    ★ 注意最后一行: 【目录不存在会 FATAL_ERROR】

         message(FATAL_ERROR "Couldn't find path for ${{module}}")

      这是一个好设计 —— 拼错模块名不会静默忽略, 而是直接编译失败。

    ★ 命名转换是【有歧义的】, 所以真实代码试了 5 种布局:

         CONFIG_MODULES_MY_MODULE=y
           → my_module      → src/modules/my_module/         ← 试一试
           → my/module      → src/modules/my/module/          ← 试一试

       还有 XXX_YYY_ZZZ 之类更复杂的情形。
""")

    print("  " + "-" * 72)
    print("  ★ 加一个模块的完整步骤（5 步）")
    print("  " + "-" * 72)
    print("""
     ① 建目录 + Kconfig

          src/modules/my_module/Kconfig:

              menuconfig MODULES_MY_MODULE
                  bool "my_module"
                  default n
                  ---help---
                      Enable support for my_module

     ② 写实现

          src/modules/my_module/MyModule.hpp
          src/modules/my_module/MyModule.cpp

     ③ 写构建脚本

          src/modules/my_module/CMakeLists.txt:

              px4_add_module(
                  MODULE modules__my_module
                  MAIN my_module
                  STACK_MAIN 2048
                  SRCS
                      MyModule.cpp
                      MyModule.hpp
                  DEPENDS
                      px4_work_queue
                  )

     ④ 打开开关

          boards/px4/fmu-v6x/default.px4board:

              CONFIG_MODULES_MY_MODULE=y

     ⑤ （可选）加参数

          src/modules/my_module/module.yaml

          ⚠️ 别忘了在 ③ 的 px4_add_module 里加:

                  MODULE_CONFIG
                      module.yaml

     然后:
          make px4_fmu-v6x_default
          # 烧录后
          nsh> my_module start
          nsh> my_module status
          nsh> my_module stop
""")


# ============================================================================
#  实验 5: 模块骨架生成器
# ============================================================================
def exp5_skeleton():
    sep("实验 5: 模块骨架生成器")

    print("""
  下面生成一个可以直接复制到 `src/modules/my_module/` 的完整骨架。

  这个骨架是【工作队列型】（推荐的现代写法），包含:
     · 一个 uORB 订阅（订阅 sensor_accel）
     · 一个 uORB 发布（发布自定义 topic）
     · 两个参数（一个 int、一个 float）
     · perf 计数器
     · 完整的 start/stop/status

  ⚠️ 输出里的 { } 是 C++ 语法, 直接复制即可。
""")

    files = {}

    files["Kconfig"] = '''menuconfig MODULES_MY_MODULE
	bool "my_module"
	default n
	---help---
		Enable support for my_module
'''

    files["CMakeLists.txt"] = '''px4_add_module(
	MODULE modules__my_module
	MAIN my_module
	STACK_MAIN 2048
	SRCS
		MyModule.cpp
		MyModule.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
	)
'''

    files["module.yaml"] = '''module_name: MyModule
parameters:
    - group: My Module
      definitions:
        MYMOD_ENABLE:
            description:
                short: Enable my module
                long: |
                    When enabled the module will run.
            type: boolean
            default: 0
            reboot_required: true

        MYMOD_RATE:
            description:
                short: Loop rate
                long: |
                    The rate at which the module runs, in Hz.
            type: int32
            default: 50
            min: 1
            max: 1000
            unit: Hz

        MYMOD_GAIN:
            description:
                short: Output gain
            type: float
            default: 1.0
            min: 0.0
            max: 10.0
            decimal: 3
'''

    files["MyModule.hpp"] = '''#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/my_module_output.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>

using namespace time_literals;

class MyModule : public ModuleBase<MyModule>, public ModuleParams,
                 public px4::ScheduledWorkItem
{
public:
	MyModule();
	~MyModule() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	uORB::Publication<my_module_output_s> _output_pub{ORB_ID(my_module_output)};

	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MYMOD_RATE>) _param_mymod_rate,
		(ParamFloat<px4::params::MYMOD_GAIN>) _param_mymod_gain
	)

	uint64_t _last_accel_timestamp{0};
	uint32_t _counter{0};
};
'''

    files["MyModule.cpp"] = '''#include "MyModule.hpp"

MyModule::MyModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

MyModule::~MyModule()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MyModule::init()
{
	// SubscriptionCallbackWorkItem::registerCallback() 让 Run() 在
	// sensor_accel 每次发布时被调度 —— 这是最省 CPU 的做法。
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// 备选: 固定周期执行
	// ScheduleOnInterval(20_ms);

	return true;
}

void MyModule::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// ---- 参数更新 ----
	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
	}

	// ---- 读传感器 ----
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// 用 ISR 抓的时间戳（卷 6 第 6 章）
			_last_accel_timestamp = accel.timestamp_sample;

			const float gain = _param_mymod_gain.get();

			// ---- 发布 ----
			my_module_output_s out{};
			out.timestamp = hrt_absolute_time();
			out.timestamp_sample = accel.timestamp_sample;
			out.value = accel.x * gain;
			out.counter = _counter++;

			_output_pub.publish(out);
		}
	}

	perf_end(_loop_perf);
}

int MyModule::task_spawn(int argc, char *argv[])
{
	MyModule *instance = new MyModule();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MyModule::print_status()
{
	PX4_INFO("running on work queue");
	PX4_INFO("gain: %.3f", (double)_param_mymod_gain.get());
	PX4_INFO("accel timestamp: %llu", (unsigned long long)_last_accel_timestamp);
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MyModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MyModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example module that subscribes to sensor_accel and publishes my_module_output.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("my_module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int my_module_main(int argc, char *argv[])
{
	return MyModule::main(argc, argv);
}
'''

    files["msg/my_module_output.msg"] = '''uint64 timestamp                # 发布时刻 (hrt_absolute_time)
uint64 timestamp_sample         # 采样时刻 (来自传感器)

float32 value                   # 处理后的输出值
uint32 counter                  # 递增计数器（便于观察丢帧）
'''

    # ---- 输出 ----
    for name, content in files.items():
        print("  " + "═" * 72)
        print(f"  📄 {name}")
        print("  " + "═" * 72)
        for line in content.rstrip().split("\n"):
            print(f"  │ {line}")
        print()

    print("""
  ────────────────────────────────────────────────────────────────────
  【文件放哪里】

     src/modules/my_module/Kconfig                   ①
     src/modules/my_module/CMakeLists.txt            ③
     src/modules/my_module/module.yaml               ⑤
     src/modules/my_module/MyModule.hpp              ②
     src/modules/my_module/MyModule.cpp              ②
     msg/my_module_output.msg                        ★ 新 topic

     boards/px4/fmu-v6x/default.px4board             ④ 加一行:
         CONFIG_MODULES_MY_MODULE=y

  ────────────────────────────────────────────────────────────────────
  【这个骨架里每部分对应前面哪一课】

     · ModuleBase<MyModule>                 卷 6 第 7 章（模块生命周期）
     · px4::ScheduledWorkItem               卷 6 第 6 章（WorkQueue / HRT）
     · uORB::Publication / Subscription     卷 6 第 5 章（uORB 就是 CDev）
     · SubscriptionCallbackWorkItem         卷 6 第 6 章（中断 → 下半部）
     · perf_counter                         卷 6 第 6 章（性能观测）
     · DEFINE_PARAMETERS                    本章（参数系统）
     · px4_add_module + Kconfig             卷 6 第 7 章（构建系统）
     · 裸 flash 参数存储                     卷 6 第 7 章（绕过文件系统）

  ────────────────────────────────────────────────────────────────────
  【编译与测试】

     $ make px4_fmu-v6x_default          # 真机固件
     $ make px4_sitl                    # 仿真

     # 烧录后进 nsh:
     nsh> my_module start
     nsh> my_module status
     nsh> param show MYMOD*
     nsh> param set MYMOD_GAIN 2.0
     nsh> param save
     nsh> listener my_module_output     # 看发布的 topic
     nsh> my_module stop

  ────────────────────────────────────────────────────────────────────
  【⚠️ 三个必须自己处理的细节】

   ① 新 topic 要在 msg/CMakeLists.txt 里注册

      PX4 的 msg 列表【不是】自动扫描的 —— 要手工加到 CMakeLists.txt 的
      msg_files 列表里。忘了加的话编译时会报 "my_module_output.h not found"。

   ② Run() 必须处理 should_exit()

      工作队列型模块没有 run_trampoline 帮你调 exit_and_cleanup()。
      忘了的话 `my_module stop` 会等满 5 秒超时。

   ③ 参数改了要 updateParams()

      Param<T>::get() 读的是【内存缓存】。不调 updateParams()
      就一直读旧值 —— 这是"我改了参数怎么没生效"的最常见原因。
""")


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  卷 6 第 8 章 · 动手写一个 PX4 模块")
    print("=" * 76)

    exp1_module_startup()
    exp2_uorb()
    exp3_parameters()
    exp4_build_system()
    exp5_skeleton()

    sep("本章小结")
    print("""
  1. 模块入口 my_module_main → ModuleBase<T>::main → 分发到 start/stop/status
     它在【临时任务】里跑, 解析完命令就退出

  2. 两种形态:
     工作队列型（推荐）: _task_id = -2, Run() 跑在共享 wq 线程上
     独立任务型:         _task_id > 0, 有 instantiate() + run_trampoline

  3. ★ 工作队列型必须自己在 Run() 里处理 should_exit():
         if (should_exit()) { ScheduleClear(); exit_and_cleanup(); return; }

  4. uORB 的核心: 一份数据 + N 个独立读者游标
     copy() 不消耗; 订阅者会跳过中间值
     o_queue 决定能容忍多少订阅者延迟

  5. 参数三层: 编译期枚举 → 内存缓存 → flash
     get() 极快（读缓存）; 改了要 updateParams() 才生效
     ⚠️ param set 后不 save, 重启就丢

  6. 参数名有 16 字符限制

  7. 加模块 5 步:
     ① Kconfig  ② 实现  ③ CMakeLists  ④ default.px4board 加一行  ⑤ module.yaml

  8. 构建链路: .px4board → kconfig.cmake 解析 → config_module_list
     → add_subdirectory → px4_add_module → builtin 表 → nsh 命令

  9. ⚠️ 新 topic 要手工加到 msg/CMakeLists.txt（不自动扫描）

  下一课: 10-NuttX vs FreeRTOS vs ChibiOS.md
""")
