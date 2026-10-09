"""
============================================================================
 卷 6 第 6 章 · 中断处理与下半部
 配套实验代码
============================================================================

 运行:  python lesson07_interrupts.py

 五个实验:
   1. 中断延迟与抖动      —— 复现 HRT 的延迟直方图
   2. 上半部/下半部       —— 为什么必须分, 怎么分
   3. 16 位计数器 → 64 位时间 —— hrt_absolute_time 的回绕检测
   4. deadline 为什么要截断 —— 50ms < 65.536ms 的硬约束
   5. 临界区 = 中断延迟    —— 关中断的代价量化

 对照的真实源码 (PX4 1.15.4):
   platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c   HRT 全部实现
   src/drivers/drv_hrt.h                                HRT API 契约
   boards/px4/fmu-v6x/src/board_config.h:315            HRT 用 TIM8 CH3
============================================================================
"""

import random
import math


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


def bar(v, vmax, width=42, ch="█"):
    n = int(v / vmax * width) if vmax else 0
    return ch * n


# ============================================================================
#  实验 1: 中断延迟与抖动
# ============================================================================
def exp1_latency():
    sep("实验 1: 中断延迟与抖动 —— 复现 HRT 的延迟直方图")

    print("""
  问题: 你让定时器在 T 时刻中断, 它【实际】什么时候来?

     想要的时刻  ─┬─────────────────────────────► 时间
     (写进 CCR)   │
                  │◄──── 中断延迟 ────►│
                                      │
     实际触发                          └─ 中断服务程序开始执行

  延迟的来源:
     · 硬件: 中断响应本身要几个周期
     · 临界区: 别人正在【关中断】, 你必须等
     · 中断嵌套: 更高优先级的中断正在跑
     · 指令: 长指令 (如除法、浮点) 不能被打断

  ┌────────────────────────────────────────────────────────────────┐
  │ ★ 关键认识: 延迟不可能为 0, 只能【统计】和【约束】              │
  │   所以工程上要做的不是"消除抖动", 而是"量化抖动"                │
  └────────────────────────────────────────────────────────────────┘
""")

    print("  " + "-" * 72)
    print("  HRT 的做法: 9 个计数器搞定 (不存原始数据)")
    print("  " + "-" * 72)
    print("""
     📁 platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c:939-953

         static void hrt_latency_update(void)
         {
           uint16_t latency = latency_actual - latency_baseline;
           unsigned index;

           /* bounded buckets */
           for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
             if (latency <= latency_buckets[index]) {
               latency_counters[index]++;
               return;
             }
           }

           /* catch-all at the end */
           latency_counters[index]++;
         }

     配套的定义:

         /* hrt.c:259-261 */
         const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;      /* = 8 */
         const uint16_t latency_buckets[8] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
         __EXPORT uint32_t latency_counters[9];      /* 8 个桶 + 1 个兜底 */

     其中:
         latency_baseline = 我们【想要】触发的时刻 (写进 CCR 的值)
         latency_actual   = 中断【实际】触发时的计数器值
         latency          = actual - baseline            (单位 µs)
""")

    # ---- 模拟 ----
    random.seed(7)
    BUCKETS = [1, 2, 5, 10, 20, 50, 100, 1000]
    NB = len(BUCKETS)
    counters = [0] * (NB + 1)

    N = 100000
    for _ in range(N):
        # 双峰分布: 大部分很快, 偶尔撞上临界区
        if random.random() < 0.97:
            lat = random.expovariate(1 / 1.2) + 0.3      # 平时 ~1.5µs
        else:
            lat = random.expovariate(1 / 15.0) + 2.0     # 3% 撞上临界区
        lat = max(0, int(lat))
        for i, b in enumerate(BUCKETS):
            if lat <= b:
                counters[i] += 1
                break
        else:
            counters[NB] += 1

    print(f"  === 模拟 {N} 次定时器中断, 统计延迟分布 ===\n")
    print(f"  {'延迟范围(µs)':<16} {'次数':>9}  {'占比':>7}  {'':<42}")
    print("  " + "─" * 78)

    maxc = max(counters)
    labels = []
    for i, b in enumerate(BUCKETS):
        lo = 0 if i == 0 else BUCKETS[i - 1]
        labels.append(f"{lo} ~ {b}")
    labels.append(f"> {BUCKETS[-1]}")

    for i, lab in enumerate(labels):
        c = counters[i]
        pct = c / N * 100
        if c == 0 and i < NB:
            continue
        marker = ""
        if i >= NB:
            marker = "  ← 兜底桶, 说明有严重阻塞"
        elif BUCKETS[i] >= 50:
            marker = "  ← 危险区!"
        elif BUCKETS[i] >= 10:
            marker = "  ← 偏高"
        print(f"  {lab:<16} {c:>9}  {pct:>6.2f}%  {bar(c, maxc, 30)}{marker}")

    def percentile(p):
        cum = 0
        for i in range(NB + 1):
            cum += counters[i]
            if cum >= N * p:
                return labels[i]
        return labels[-1]

    p50, p99, p999 = percentile(0.50), percentile(0.99), percentile(0.999)

    print(f"""
  【怎么看这张表】

     P50  在 {p50:<10} → 平时的延迟, 由硬件决定, 改不了
     P99  在 {p99:<10} → 【设计约束用这个】
     P99.9 在 {p999:<9} → 极端情况, 也要看
     尾部有 {counters[NB]} 次超过 1000µs  → 要查是什么阻塞了

  ┌────────────────────────────────────────────────────────────────┐
  │ 实时性设计用【最坏情况】, 不是平均值                              │
  │                                                                │
  │   "平均延迟 1.5µs" 对 1kHz 控制回路毫无意义                       │
  │   "最坏延迟 800µs" 才是决定你能不能跑 1kHz 的数字                 │
  └────────────────────────────────────────────────────────────────┘

  【为什么用直方图而不是记录每次延迟】

     · 1kHz × 1 小时 = 360 万条记录 × 4 字节 = 14 MB    ← 飞控装不下
     · 直方图: 9 个 uint32 = 36 字节                    ← 永远够
     · 代价: 丢失了精确值, 只剩区间

     这个取舍在嵌入式中极其常见 —— 【精度换内存】。

  【一个漂亮的技巧】

     latency_actual - latency_baseline 是【16 位无符号减法】。
     即使 actual < baseline (计数器回绕了), 结果依然正确 ——
     因为无符号整数的模运算自动处理了回绕。

         0x0010 - 0xFFF0 = 0x0020  (模 65536)

     【不需要任何 if 判断】。这是嵌入式计时的常用手法。
""")


# ============================================================================
#  实验 2: 上半部 / 下半部
# ============================================================================
def exp2_top_bottom():
    sep("实验 2: 上半部 / 下半部 —— 为什么必须分")

    print("""
  场景: 一个 IMU 通过 SPI 连接, 每次数据就绪(DRDY)会产生一个中断。

  数据流程:
     ① 读 6~9 个寄存器 (SPI 传输, 每个寄存器几十 µs)
     ② 时间戳 + 单位换算
     ③ 写进 uORB topic
     ④ 唤醒等待的订阅者
""")

    print("  " + "-" * 72)
    print("  ❌ 方案 A: 全部在 ISR 里做完")
    print("  " + "-" * 72)

    SPI_PER_REG = 8.0     # 每个寄存器 8µs (SPI @ 10MHz, 含开销)
    NREG = 9
    CONVERT = 5.0
    PUBLISH = 15.0
    IRQ_FREQ = 1000       # 1kHz

    total_isr = NREG * SPI_PER_REG + CONVERT + PUBLISH
    print(f"""
     中断处理程序:
        读 {NREG} 个寄存器    {NREG} × {SPI_PER_REG}µs = {NREG*SPI_PER_REG:>6.1f} µs
        单位换算                    {CONVERT:>6.1f} µs
        发布 uORB                   {PUBLISH:>6.1f} µs
        ────────────────────────────────────────
        总计                        {total_isr:>6.1f} µs

     中断频率: {IRQ_FREQ} Hz  → 周期 {1000/IRQ_FREQ:.0f} µs

  ┌────────────────────────────────────────────────────────────────┐
  │  ⚠️ {total_isr:.0f}µs 的中断处理程序, 每秒触发 {IRQ_FREQ} 次               │
  │                                                                │
  │     CPU 占用 = {total_isr}µs × {IRQ_FREQ}/s = {total_isr*IRQ_FREQ/10000:.1f}%                            │
  │                                                                │
  │     更严重的是: 这 {total_isr:.0f}µs 里【所有低优先级中断都被堵住】      │
  │     系统的中断延迟直接变成 {total_isr:.0f}µs                        │
  └────────────────────────────────────────────────────────────────┘
""")

    print("  " + "-" * 72)
    print("  ✅ 方案 B: 上半部 + 下半部")
    print("  " + "-" * 72)

    ISR_WORK = 1.5        # 中断里只记录时间戳 + 发信号
    print(f"""
     上半部 (ISR):  记录时间戳 + post 信号量      {ISR_WORK:>6.1f} µs
     下半部 (线程): 读 SPI + 换算 + 发布          {total_isr - ISR_WORK:>6.1f} µs

     对比:
       中断延迟     {total_isr:>6.1f} µs  →  {ISR_WORK:>6.1f} µs   改善 {total_isr/ISR_WORK:.0f}x
       谁能被打断   ❌ 谁都不能     →  ✅ 更高优先级的中断可以
""")

    # ---- 时序图对比 ----
    print("  " + "-" * 72)
    print("  时序对比 (一个周期内)")
    print("  " + "-" * 72)

    def render(slots, marks, width=64):
        line = ["·"] * width
        for i, m in marks.items():
            if 0 <= i < width:
                line[i] = m
        return "".join(line)

    print(f"\n    方案 A: 中断占满 {total_isr:.0f}µs")
    print(f"    ├{'─'*64}┤")
    occupied = int(total_isr / 1000 * 64)
    print(f"    │{'█'*occupied}{'·'*(64-occupied)}│")
    print(f"    ↑ 中断开始{' '*max(0,occupied-8)}↑ 中断结束 → 这段时间【系统僵死】")

    print(f"\n    方案 B: 中断只占 {ISR_WORK:.0f}µs, 剩下在线程里做")
    occ2 = max(1, int(ISR_WORK / 1000 * 64))
    rest = int((total_isr - ISR_WORK) / 1000 * 64)
    print(f"    │{'█'*occ2}{'▒'*rest}{'·'*(64-occ2-rest)}│")
    print(f"    ↑ISR{' '*(max(0,occ2-3))}↑ 下半部(线程) → 这期间【其他中断可以插进来】")

    print(f"""
  【上半部该做什么 —— 三条判据】

     ✅ 一定要在中断里做的:
        · 读硬件状态寄存器 (不读就会被下一次中断覆盖)
        · 清中断标志 (不清就永远不再触发)
        · 记录【时间戳】(这是最关键的 —— 时间戳必须反映"硬件产生的时刻")
        · 把数据从硬件搬到内存 (DMA 也行)

     ❌ 不该在中断里做的:
        · 任何可能阻塞的操作 (拿信号量、malloc)
        · 任何耗时的计算 (三角函数、矩阵运算、滤波)
        · 任何 I/O (SPI/I2C 传输可能等几十 µs)
        · 打印日志

     ⚠️ 最关键的一条: 【时间戳必须在上半部取】

        如果你在下半部才调 hrt_absolute_time(),
        那个时间戳反映的是"线程被调度到的时刻",
        而不是"传感器数据就绪的时刻"。

        两个时刻可能差几百 µs, 而且【抖动很大】——
        这直接毁掉 EKF 的观测质量。
""")

    print("  " + "-" * 72)
    print("  PX4 的实际模式")
    print("  " + "-" * 72)
    print("""
     传感器 DRDY 中断
          │
          ├─ 上半部 (ISR, ~1µs):
          │    · hrt_absolute_time() 记录精确时间戳
          │    · 把时间戳存进 px4::atomic<hrt_abstime>
          │    · px4_sem_post() 唤醒工作队列线程
          │      (或者 WorkItem::ScheduleNow())
          │
          ▼
     wq:SPI0 线程 (优先级 254)
          │
          └─ 下半部 (线程上下文, ~50µs):
               · SPI 传输读 9 个寄存器   ← 可以阻塞!
               · 单位换算
               · 发布 uORB topic

     ★ 时间戳在【上半部】取, 数据在【下半部】读 —— 这是关键的分工。
       上半部只花 ~1µs, 却拿到了最精确的时间。
""")

    # ---- 量化对比 ----
    print("  " + "-" * 72)
    print("  量化对比")
    print("  " + "-" * 72)
    rows = [
        ("中断处理时长", f"{total_isr:.1f} µs", f"{ISR_WORK:.1f} µs", f"{total_isr/ISR_WORK:.0f}x"),
        ("中断延迟增加值", f"{total_isr:.1f} µs", f"{ISR_WORK:.1f} µs", f"{total_isr/ISR_WORK:.0f}x"),
        ("单核 CPU 占用", f"{total_isr*IRQ_FREQ/10000:.1f}%",
         f"{total_isr*IRQ_FREQ/10000:.1f}%", "一样"),
        ("能否被高优先级中断抢占", "❌", "✅", "—"),
        ("时间戳精度", "精确", "精确", "—"),
    ]
    print(f"\n    {'指标':<24} {'方案 A (全在ISR)':<18} {'方案 B (上下半部)':<18} {'改善'}")
    print("    " + "─" * 76)
    for a, b, c, d in rows:
        print(f"    {a:<24} {b:<18} {c:<18} {d}")

    print(f"""
  【注意第三行】

     CPU 占用【没变】—— 总工作量是一样的。
     变的只是【这 {total_isr:.0f}µs 是否阻塞整个系统】。

     ✓ 分上下半部的收益不是"少干活", 而是"干活时不堵路"。
""")


# ============================================================================
#  实验 3: 16 位计数器 → 64 位时间
# ============================================================================
def exp3_time_extension():
    sep("实验 3: 16 位计数器 → 64 位时间 —— hrt_absolute_time 的精髓")

    print("""
  HRT 的硬件配置 (📁 hrt.c:410-441):

      /* configure the timer to free-run at 1MHz */
      rPSC = (HRT_TIMER_CLOCK / 1000000) - 1;

      /* run the full span of the counter */
      rARR = 0xffff;

      /* set an initial capture a little ways off */
      rCCR_HRT = 1000;

  ┌──────────────────────────────────────────────────────────────┐
  │  硬件只有:  16 位自由运行计数器 (1 MHz) + 1 个比较寄存器       │
  │                                                              │
  │    · 计数范围 0 ~ 65535                                      │
  │    · 1 count = 1 µs                                          │
  │    · 溢出周期 = 65536 µs = 65.536 ms                         │
  │    · 【每 65.536ms 回绕一次】                                 │
  └──────────────────────────────────────────────────────────────┘

  软件需要: 一个【永不回绕、永不倒退】的 64 位微秒时间戳。

  📁 src/drivers/drv_hrt.h:55-62

      /**
       * Absolute time, in microsecond units.
       *
       * Absolute time is measured from some arbitrary epoch shortly after
       * system startup.  It should never wrap or go backwards.
       */
      typedef uint64_t	hrt_abstime;

  解法: 软件维护一个"溢出累积量"。
""")

    print("  " + "-" * 72)
    print("  hrt_absolute_time() 的实现")
    print("  " + "-" * 72)
    print("""
     📁 hrt.c:655-696

         hrt_abstime hrt_absolute_time(void)
         {
           hrt_abstime abstime;
           uint32_t count;
           irqstate_t flags;

           static volatile hrt_abstime base_time;     /* 累积的溢出量 */
           static volatile uint32_t last_count;       /* 上次读到计数值 */

           /* prevent re-entry */
           flags = px4_enter_critical_section();

           /* get the current counter value */
           count = rCNT;

           /* Determine whether the counter has wrapped since the
            * last time we're called. */
           if (count < last_count) {
             base_time += HRT_COUNTER_PERIOD;         /* += 65536 */
           }

           /* save the count for next time */
           last_count = count;

           /* compute the current time */
           abstime = HRT_COUNTER_SCALE(base_time + count);

           px4_leave_critical_section(flags);
           return abstime;
         }
""")

    # ---- 模拟 ----
    print("  " + "-" * 72)
    print("  模拟: 跟着计数器走一遍")
    print("  " + "-" * 72)

    class HRT:
        def __init__(self):
            self.cnt = 0
            self.base = 0
            self.last = 0
            self.wraps = 0

        def advance(self, us):
            """推进硬件计数器 us 微秒"""
            self.cnt = (self.cnt + us) % 65536

        def absolute_time(self):
            flags_ = "🔒"     # 关中断
            if self.cnt < self.last:
                self.base += 65536
                self.wraps += 1
                wrapped = True
            else:
                wrapped = False
            self.last = self.cnt
            return self.base + self.cnt, wrapped

    h = HRT()
    print(f"    {'计数器的值':>12} {'回绕?':<8} {'base_time':>12} {'返回值':>12}  说明")
    print("    " + "─" * 74)

    # (推进量, 说明) —— 回绕发生在【计数加过头】的那一次
    steps = [(0,     "刚启动"),
             (1000,  "过 1ms"),
             (30000, "再过 30ms"),
             (34000, "再过 34ms → 逼近 65535"),
             (1000,  "再加 1ms → 【跨过 0, 触发回绕检测】"),
             (50000, "过 50ms"),
             (15000, "再过 15ms → 又逼近顶部"),
             (1000,  "再加 1ms → 【再次回绕】")]
    for step, note in steps:
        h.advance(step)
        t, wrapped = h.absolute_time()
        w = "★ 回绕!" if wrapped else ""
        print(f"    {h.cnt:>12} {w:<8} {h.base:>12} {t:>12}  {note}")

    print(f"""
  【看到的规律】

     · 返回值 = base_time + count        (单调递增!)
     · 每次回绕 base_time 加 65536
     · 返回的 64 位值【永不回绕】

  ⚠️ 但这一切依赖一个前提 —— 看源码注释:
""")

    print("""     📁 hrt.c:668-672

         /*
          * Determine whether the counter has wrapped since the
          * last time we're called.
          *
          * This simple test is sufficient due to the guarantee that
          * we are always called at least once per counter period.
          */
""")

    print("""  ★★ 【"we are always called at least once per counter period"】★★

     这个保证【不是自动成立】的 —— 必须由设计来保证。

     如果超过 65.536 ms 才有一次调用:
         · 计数器可能回绕了 1 次 (甚至多次)
         · 而 count < last_count 这个判断【只能检测到"回绕过奇数次"】
         · 结果: base_time 加少了, 时间戳【永久偏移 65.536ms】

     这是一个"看起来对, 但在特定条件下静默出错"的经典陷阱。

  【那么 HRT 怎么保证这个前提? —— 见实验 4】
""")


# ============================================================================
#  实验 4: deadline 为什么要截断
# ============================================================================
def exp4_deadline_truncation():
    sep("实验 4: deadline 为什么要截断 —— 50ms < 65.536ms 的硬约束")

    print("""
  📁 hrt.c:900-935  hrt_call_reschedule()

      static void hrt_call_reschedule()
      {
        hrt_abstime now = hrt_absolute_time();
        struct hrt_call *next = (struct hrt_call *)sq_peek(&callout_queue);
        hrt_abstime deadline = now + HRT_INTERVAL_MAX;      /* ← 默认值! */

        /*
         * Determine what the next deadline will be.
         *
         * Note that we ensure that this will be within the counter
         * period, so that when we truncate all but the low 16 bits
         * the next time the compare matches it will be the deadline
         * we want.
         *
         * It is important for accurate timekeeping that the compare
         * interrupt fires sufficiently often that the base_time update in
         * hrt_absolute_time runs at least once per timer period.
         */
        if (next != NULL) {
          if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
            deadline = now + HRT_INTERVAL_MIN;      /* 已过期, 尽快 */
          } else if (next->deadline < deadline) {
            deadline = next->deadline;              /* 队列里的下一个 */
          }
        }

        /* set the new compare value and remember it for latency tracking */
        rCCR_HRT = latency_baseline = deadline & 0xffff;    /* ★ 截断! */
      }
""")

    print(f"""
  【三个常量的关系】

      HRT_INTERVAL_MIN    = 50        (50 µs)
      HRT_INTERVAL_MAX    = 50000     (50 ms)
      HRT_COUNTER_PERIOD  = 65536     (65.536 ms)

      ┌──────────────────────────────────────────────────────────┐
      │  HRT_INTERVAL_MIN  <  HRT_INTERVAL_MAX  <  PERIOD        │
      │        50          <       50000        <   65536    ✓   │
      └──────────────────────────────────────────────────────────┘

  【为什么必须 MAX < PERIOD】

     因为最后那行:

         rCCR_HRT = deadline & 0xffff;

     deadline 是 64 位, 但 CCR 寄存器只有 16 位 —— 【必须截断】。

     截断后比较是否还正确?
        · hardware counter 也在 0~65535 循环
        · 于是 "deadline & 0xffff" 和 "counter == deadline" 是等价的
          —— 只要 deadline 距离 now 不超过一个周期
""")

    # ---- 演示 ----
    def truncated_match(now, deadline):
        """模拟: 硬件比较发生在 (deadline & 0xffff) 处, 判据是 counter == ccr"""
        ccr = deadline & 0xffff
        # 从现在走到 deadline, 看计数器什么时候等于 ccr
        elapsed = 0
        cnt = now & 0xffff
        while elapsed < 200000:
            if cnt == ccr:
                return elapsed
            cnt = (cnt + 1) % 65536
            elapsed += 1
        return None

    print("  " + "-" * 72)
    print("  验证: 截断后还能精确匹配吗")
    print("  " + "-" * 72)
    print(f"\n    {'now':>8} {'deadline':>10} {'间隔(µs)':>10} {'CCR=deadline&0xffff':>20} {'实际触发时刻':>14} {'正确?'}")
    print("    " + "─" * 80)

    cases = [
        (1000, 1000 + 50000),
        (1000, 1000 + 65535),
        (65530, 65530 + 50000),          # 跨回绕
        (1000, 1000 + 65536),            # ← 正好一个周期, 危险!
        (1000, 1000 + 100000),           # ← 远超一个周期, 必然错
    ]
    for now, dl in cases:
        ccr = dl & 0xffff
        got = truncated_match(now & 0xffff, dl)
        want = dl - now
        ok = (got == want)
        mark = "✅" if ok else "❌ 错!"
        extra = ""
        if not ok:
            if want >= 65536:
                extra = "  ← 间隔超过一个周期!"
            else:
                extra = "  ← 无法区分"
        print(f"    {now:>8} {dl:>10} {want:>10} {ccr:>20} "
              f"{str(got):>14} {mark}{extra}")

    print(f"""
  【结论】

     间隔 < 65536 µs  →  截断后能精确匹配 ✅
     间隔 ≥ 65536 µs  →  【无法正确匹配】 ❌

     这就是为什么 HRT_INTERVAL_MAX 必须 < 65536。

  【这个 50ms 还担着第二个责任】

     回想实验 3: hrt_absolute_time() 的正确性要求
     "至少每 65.536ms 调用一次"。

     谁在保证它?  就是这个默认的 50ms deadline!

         队列为空时 → deadline = now + 50000 → 50ms 后必定中断
                       ↓
                  hrt_tim_isr() → hrt_call_invoke()
                       ↓
                  hrt_absolute_time() 被调用 ✓
                       ↓
                  50ms < 65.536ms → 回绕检测永远不会漏

  ┌────────────────────────────────────────────────────────────────┐
  │ ★★ 一个常量同时满足两个约束:                                     │
  │                                                                │
  │     ① 硬件: deadline 必须能截断到 16 位  →  < 65536 µs          │
  │     ② 软件: 必须有"心跳"来更新时间基准   →  越小越安全           │
  │                                                                │
  │    选 50000 是两者的折中: 够小(满足②), 又不会太频繁(省 CPU)。    │
  └────────────────────────────────────────────────────────────────┘

  ⚠️ 所以 HRT_INTERVAL_MAX 【不能随便改大】!
     改成 70000 会同时破坏截断和回绕检测 —— 而且是【静默出错】,
     时间戳会每 65.536ms 偏一点, 没人会立刻发现。
""")


# ============================================================================
#  实验 5: 临界区 = 中断延迟
# ============================================================================
def exp5_critical_section():
    sep("实验 5: 临界区 = 中断延迟")

    print("""
  【核心等式】

      中断延迟 ≥ 当前正在执行的【最长关中断区段】

  这是一条无法绕过的硬约束。

  NuttX 里关中断的两把锁 (卷 6 第 4 章讲过):

     enter_critical_section()   ── 关中断, 【无法被任何东西打断】
     sched_lock()               ── 只禁任务切换, 【中断照跑】
""")

    # ---- 代价计算 ----
    print("  " + "-" * 72)
    print("  量化: 关中断多久, 中断延迟就增加多少")
    print("  " + "-" * 72)

    BASE_LATENCY = 1.5     # 硬件固有延迟 µs

    rows = [
        ("读一个寄存器 (关中断)",         0.5),
        ("查表 + 更新指针",               1.0),
        ("memcpy 64 字节",                2.0),
        ("memcpy 512 字节",              16.0),
        ("浮点开方 (无 FPU)",            40.0),
        ("浮点开方 (有 FPU)",             0.3),
        ("一次 SPI 传输 @10MHz, 16字节",  14.0),
        ("printf 一行",                 200.0),
        ("flash 擦除一个扇区",       100000.0),
    ]

    print(f"\n    {'临界区里的操作':<32} {'耗时(µs)':>10} {'总中断延迟(µs)':>16} 评价")
    print("    " + "─" * 84)
    for name, us in rows:
        total = BASE_LATENCY + us
        if us < 3:
            v = "✅ 可以接受"
        elif us < 20:
            v = "⚠️ 偏长, 要审视"
        elif us < 1000:
            v = "❌ 太长了"
        else:
            v = "💀 绝对不行"
        print(f"    {name:<32} {us:>10.1f} {total:>16.1f} {v}")

    print(f"""
  【读表要点】

     · 1kHz 控制回路的周期是 1000 µs
     · 高优先级中断 (比如 IMU DRDY) 的延迟预算通常 < 10 µs
     · 所以临界区里的操作【必须是个位数 µs 级】

     💀 flash 擦除 100ms 关中断 = 系统瘫痪 100ms
        (所以 flash 驱动必须分段操作, 每段之间放中断)

  【Cortex-M 上的四类"长操作"陷阱】

     ① 除法 / 浮点
        Cortex-M4F 有硬件 FPU, 但 M3/M0 没有 —— 靠软件模拟, 几百周期
        ✅ 有 FPU 时单精度开方 ~14 周期, 可以接受
        ❌ 双精度除法在 M4F 上也是软件模拟!

     ② 库函数
        memcpy / memset 看着简单, 但编译器可能展开成很长的代码
        大块内存操作必须移出临界区

     ③ 循环等硬件
        "写寄存器 → 等标志位" 这种代码是【定时炸弹】
        如果硬件不响应, 循环永远不退出, 中断永远不开

     ④ 日志打印
        printf 可能触发串口输出、内存分配、字符串格式化
        卷 6 第 3 章看到 PX4 的 px4_task_spawn_cmd 里就有
        "ERROR: ... printf 在关中断里" 的教训

  【怎么自查】

     ① 静态检查: 找所有 enter_critical_section / irqsave
        看看中间有多少条指令 (反汇编数一下)

     ② 运行时测量: NuttX 提供了 CONFIG_SCHED_CRITMONITOR

        📁 include/nuttx/sched.h 的 TCB 里有:

            /* Pre-emption monitor support ****************************/
            #ifdef CONFIG_SCHED_CRITMONITOR
              uint32_t premp_start;   /* Time when preemption disabled */
              uint32_t premp_max;     /* Max time preemption disabled  */
              uint32_t crit_start;    /* Time critical section entered */
              uint32_t crit_max;      /* Max time in critical section  */
              uint32_t run_start;     /* Time when thread begin run    */
              uint32_t run_max;       /* Max time thread run           */
            #endif

        ★ 注意它记的是 __MAX__ 不是平均 ——
          因为实时性只关心最坏情况 (见实验 1)。

        ⚠️ 但是: 【PX4 没有开这个选项】

              $ grep -rl "CONFIG_SCHED_CRITMONITOR=y" boards/ | wc -l
              0

            PX4 也没有任何代码去读 crit_max / premp_max。

            ==> 这是一个【可以加强的地方】:
                飞控最怕的就是"某个驱动的临界区偶尔变长",
                而这正是 crit_max 能直接抓出来的问题。

                打开成本: 每次进出临界区多几条指令
                收益: 能定位到【哪个线程】制造了中断延迟

     ③ 看 HRT 的延迟直方图 (实验 1)
        如果 >50µs 的桶里数量明显增加, 说明有人关中断太久了

        ⚠️ 但要注意: HRT 的延迟桶只统计【定时器中断】,
           而定时器中断本身也可能被别人的临界区挡住 ——
           所以它是"间接指标", 不是"直接定位工具"。

  【PX4 的实证】

     卷 6 第 4 章我们数过: PX4 全树只有 31 处 px4_enter_critical_section。
     而且几乎全部在:

       · uORB 的 ATOMIC_ENTER (包住一次 memcpy + 回调)
       · hrt.c 的时基读取 (几条指令)
       · SPI 的 LOCK_PREEMPTION 模式
       · pwm_input 的定时器配置

     没有一处是"大块计算"—— 这不是巧合, 是设计纪律。

  ┌────────────────────────────────────────────────────────────────┐
  │  ★ 附带发现: 一段【从未被编译过】的代码                          │
  └────────────────────────────────────────────────────────────────┘

     PX4 提供调度器 instrumentation 的接收端:

        📁 platforms/nuttx/src/px4/common/cpuload.cpp:221-240

            #ifdef CONFIG_SEGGER_SYSVIEW

            #ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
            void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
            {{
              sysview_sched_note_irqhandler(irq, handler, enter);
            }}
            #endif

            #ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
            void sched_note_syscall_enter(int nr);      /* <- 多写了个分号! */
            {{
              sysview_sched_note_syscall_enter(nr);     /* <- 裸块, 非法 */
            }}

            void sched_note_syscall_enter(int nr);      /* ← 重名! */
            {{
              sysview_sched_note_syscall_enter(nr);     /* ← 应为 _exit */
            }}
            #endif

            #endif

     三个问题:
       ① 函数签名后面【多了个分号】→ 变成声明
       ② 后面的花括号就成了【文件作用域的裸块】→ 非法
       ③ 第二个应该是 sched_note_syscall_exit, 却写成了 enter, 重名

     实测编译 (MSVC):

        $ cl /c deadcode.c
        error C2449: 在文件范围找到左花括号 (是否缺少函数头?)
        error C2059: 语法错误: 右花括号

     ★ 为什么没人发现?

       看板级配置:

          $ grep -rn "SCHED_INSTRUMENTATION_SYSCALL" boards/
          (空)

       【没有任何板子启用过这个选项】→ 这段代码从未被编译。

     这是一个绝佳的教材级案例:
       · 预处理器守卫让坏代码"合法地"留在源码里
       · 只有启用那个选项的人才会撞上
       · 而启用的人会先遇到编译错误, 不会遇到运行时 bug

     ==> 读内核/大型项目源码时, 要意识到:
         【不是所有能看到的分支都是活着的】。
""")


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  卷 6 第 6 章 · 中断处理与下半部")
    print("=" * 76)

    exp1_latency()
    exp2_top_bottom()
    exp3_time_extension()
    exp4_deadline_truncation()
    exp5_critical_section()

    sep("本章小结")
    print("""
  1. 中断延迟不可能为 0, 只能量化
     HRT 用 9 个计数器做直方图 —— 精度换内存

  2. 实时性设计看【最坏情况】不是平均值
     P99 / max 才有意义

  3. 上半部 / 下半部分工:
     上半部: 读硬件 + 清标志 + 【记录时间戳】
     下半部: 所有耗时/可能阻塞的工作

    ★ 时间戳必须在上半部取 —— 这是最关键的一条

  4. HRT 用"软件累积溢出量"把 16 位计数器扩展成 64 位时间
     前提: 每 65.536ms 至少调用一次 hrt_absolute_time

  5. HRT_INTERVAL_MAX = 50000 同时满足两个约束:
     · < 65536  → deadline 能截断到 16 位
     · 足够小   → 提供"心跳"保证回绕检测不漏
     ⚠️ 改大它会【静默】破坏时间基准

  6. 中断延迟 ≥ 最长关中断区段
     Cortex-M 上的四类陷阱: 浮点/除法、库函数、循环等硬件、printf

  7. NuttX 用 CONFIG_SCHED_CRITMONITOR 记录 crit_max (最坏值)
     PX4 只有 31 处关中断, 且都是几条指令 —— 设计纪律

  下一课: 卷 6 第 7 章
""")
