"""
============================================================================
 NuttX 教程 · 第 5 课 · 同步机制
 配套实验代码
============================================================================

 运行:  python lesson05_synchronization.py

 五个实验:
   1. 竞态可视化         —— 撕裂读是怎么发生的
   2. 保护手段的代价      —— 关中断 / 调度锁 / 信号量
   3. 优先级反转重现      —— 火星探路者号的场景
   4. 优先级继承完整算法  —— boost + 三分支 restore + pend_reprios 栈
   5. holder 池耗尽       —— CONFIG_SEM_PREALLOCHOLDERS 的后果

 算法严格对照源码:
   sched/semaphore/sem_holder.c   nxsem_boostholderprio
                                  nxsem_restoreholderprio
                                  nxsem_allocholder / nxsem_release_holder
   sched/semaphore/sem_wait.c     nxsem_wait
   sched/semaphore/sem_post.c     nxsem_post
============================================================================
"""

import math


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


# ============================================================================
#  实验 1: 竞态可视化
# ============================================================================
def exp1_race():
    sep("实验 1: 竞态可视化 —— 撕裂读")

    print("""
  场景: IMU 驱动任务写 3 轴加速度, 姿态任务读

      /* IMU 驱动 */
      g_accel[0] = x;  g_accel[1] = y;  g_accel[2] = z;

      /* 姿态控制 */
      a[0] = g_accel[0]; a[1] = g_accel[1]; a[2] = g_accel[2];
""")

    def run(interleave_at, label):
        """interleave_at: 在第几次写之后被抢占"""
        g = [0.0, 0.0, 0.0]
        new = [1.0, 2.0, 3.0]
        timeline = []

        # IMU 写第 1..interleave_at 个
        for i in range(interleave_at):
            g[i] = new[i]
            timeline.append(f"IMU 写 g[{i}]={new[i]}  -> g={g}")

        # 抢占: 姿态任务读全部
        a = list(g)
        timeline.append(f"⚡ 抢占! 姿态读 a={a}   <-- 【不一致的向量】")

        # IMU 继续写完
        for i in range(interleave_at, 3):
            g[i] = new[i]
            timeline.append(f"IMU 写 g[{i}]={new[i]}  -> g={g}")

        print(f"\n  --- {label} ---")
        for t in timeline:
            print(f"    {t}")

        norm = sum(x * x for x in a) ** 0.5
        print(f"    姿态拿到的向量模长 = {norm:.3f}  (期望 {sum(x*x for x in new)**0.5:.3f})")
        return a

    run(1, "在写第 1 个分量后被抢占")
    run(2, "在写第 2 个分量后被抢占")

    print("""
  【结论】

    姿态任务拿到的不是"旧数据" —— 是【物理上不可能存在】的向量:
       (1.0, 0.0, 0.0)  x轴新值 + y/z 轴旧值

    对姿态估计来说, 这比"数据旧 1ms"严重得多:
      · 方向完全错误
      · 模长也错误 (应该是 3.742, 拿到 1.0)
      · 如果这个值进了 EKF, 会污染协方差

  【三个必要条件】破坏任意一个, 竞态就消失:
    ① 多个执行流访问同一份数据
    ② 至少有一个在【写】
    ③ 访问之间【没有同步】
""")


# ============================================================================
#  实验 2: 保护手段的代价
# ============================================================================
def exp2_cost_ladder():
    sep("实验 2: 保护手段的代价阶梯")

    print("""
  用【最弱但够用】的手段 —— 这是实时系统的核心原则。
  下面用模拟数据展示三种手段的时间代价。
""")

    # 数值取自典型 Cortex-M7 @ 480MHz 的量级
    METHODS = [
        ("enter_critical_section", 0.08, "关中断",
         "中断延迟 +80ns, 连中断都停"),
        ("sched_lock",             0.10, "调度锁",
         "只禁任务切换, 中断照跑"),
        ("sem_wait (无竞争)",       0.20, "信号量·快路径",
         "只是减一下计数, 无上下文切换"),
        ("sem_wait (有竞争)",      12.00, "信号量·慢路径",
         "阻塞 + 上下文切换"),
        ("pthread_mutex (无竞争)",   0.25, "互斥锁·快路径",
         "= 信号量 + 元数据开销"),
    ]

    print(f"  {'手段':<26} {'耗时(µs)':>10}  {'作用范围':<18} 说明")
    print("  " + "-" * 90)
    for name, us, scope, desc in METHODS:
        # 用对数刻度画条, 保证小值也可见

        width = max(1, int(math.log10(us / 0.08 + 1) / math.log10(12 / 0.08 + 1) * 40))
        bar = "█" * width
        print(f"  {name:<26} {us:>10.2f}  {scope:<18} {desc}")
        print(f"  {'':<26} {bar}")

    print("""
    （柱长用对数刻度 —— 否则 0.08µs 和 12µs 差 150 倍, 线性刻度没法看）""")

    print("""
  【关键洞察】

    · 快路径 vs 慢路径 差 60 倍
      → 设计良好的系统, 99%+ 的加锁走快路径（无竞争）
      → 所以"用锁"本身不慢, "锁争用"才慢

    · 关中断的代价不在它自己耗时, 而在【它阻塞了所有人】
      → 关中断 1µs = 整个系统的中断延迟 +1µs
      → 所以临界区必须极短（几条指令）

    · 选择顺序（从上到下, 越上面越强也越贵）:
        关中断 > 调度锁 > 互斥锁 > 无锁

  【决策依据】
      中断也会访问?      → 只能关中断
      不能阻塞?          → 调度锁（但中断仍可能破坏你！）
      保护资源?          → pthread_mutex
      传递事件?          → sem_t (初值 0, 设 SEM_PRIO_NONE)
""")


# ============================================================================
#  实验 3: 优先级反转
# ============================================================================
def exp3_priority_inversion():
    sep("实验 3: 优先级反转 —— 火星探路者号的重现")

    DRV_CRIT = 3.0      # 驱动临界区长度
    LOG_WORK = 20.0     # 日志任务的计算量

    print(f"""
  三个任务共享一把锁 L（比如 SPI 总线）:

     姿态控制 (优先级 236)  —— 1kHz, 必须准时
     日志写卡 (优先级 230)  —— {LOG_WORK:.0f}ms 的重计算, 不碰 L
     慢速驱动 (优先级 205)  —— 持有 L, 临界区只有 {DRV_CRIT:.0f}ms

  ⚠️ 关键设定: 驱动的临界区【只有 {DRV_CRIT:.0f}ms】

     如果没有优先级反转, 姿态任务最多等 {DRV_CRIT:.0f}ms。
     实际会等多久? 往下看。
""")

    def simulate(pi):
        """
        一个真正按优先级调度的离散事件仿真。

        模型:
          · 每个任务有 (优先级, 到达时刻, 需要的工作量)
          · 需要锁的任务在"开始时"尝试拿锁
          · 调度器总是选【有效优先级】最高的可运行任务
          · 有效优先级 = 基础优先级, 或者(PI开启时)被等待者提升后的值
          · 任务跑完 = 释放锁
        """
        tasks = [
            {"n": "drv", "prio": 205, "arr": 0.0, "total": DRV_CRIT, "lock": True},
            {"n": "log", "prio": 230, "arr": 1.0, "total": LOG_WORK, "lock": False},
            {"n": "att", "prio": 236, "arr": 2.0, "total": 1.0,      "lock": True},
        ]
        for T in tasks:
            T.update(rem=T["total"], arrived=False, done=False,
                     blocked=False, has_lock=False)

        st = {"holder": None}
        t = 0.0
        ev = []
        res = {"att_block": None, "att_get": None}
        EPS = 1e-9

        def eff(T):
            """有效优先级 —— PI 的实现点"""
            if pi and st["holder"] == T["n"]:
                waiters = [x["prio"] for x in tasks if x["blocked"]]
                if waiters:
                    return max(waiters)          # 被提升到最高等待者
            return T["prio"]

        def logev(time, T, msg):
            ev.append((time, eff(T), T["n"], msg))

        for _ in range(200):                      # 防死循环
            # ---- 1. 处理到达 ----
            for T in tasks:
                if not T["arrived"] and T["arr"] <= t + EPS:
                    T["arrived"] = True
                    logev(t, T, "到达")

            # ---- 2. 想拿锁的任务按有效优先级从高到低尝试 ----
            wanters = sorted(
                [x for x in tasks
                 if x["arrived"] and not x["done"] and not x["blocked"]
                 and x["lock"] and not x["has_lock"]],
                key=lambda x: -eff(x))
            for T in wanters:
                if st["holder"] is None:
                    st["holder"] = T["n"]
                    T["has_lock"] = True
                    logev(t, T, "拿到 L")
                else:
                    T["blocked"] = True
                    logev(t, T, "要 L -> 【阻塞】")
                    if T["n"] == "att" and res["att_block"] is None:
                        res["att_block"] = t

                    # PI: 立即提升持有者（对照 sem_wait.c:140 nxsem_boost_priority）
                    if pi:
                        h = next(x for x in tasks if x["n"] == st["holder"])
                        if T["prio"] > h["prio"]:
                            ev.append((t, T["prio"], h["n"],
                                       f"★ 优先级继承: {h['prio']} -> {T['prio']}  "
                                       f"(因为 {T['n']}({T['prio']}) 在等)"))

            # ---- 3. 选中优先级最高的可运行任务 ----
            runnable = [x for x in tasks
                        if x["arrived"] and not x["done"] and not x["blocked"]]
            if not runnable:
                break
            cur = max(runnable, key=eff)

            # ---- 4. 它跑到【下一个到达时刻】或【自己完成】为止 ----
            next_arr = min([x["arr"] for x in tasks if not x["arrived"]],
                           default=float("inf"))
            t_next = min(next_arr, t + cur["rem"])
            dt = t_next - t
            cur["rem"] -= dt
            t = t_next

            # ---- 5. 完成了? ----
            if cur["rem"] <= EPS:
                cur["done"] = True
                if cur["has_lock"]:
                    # 先记日志（此时还能看到被提升后的有效优先级）
                    logev(t, cur, "临界区跑完, 释放 L")
                    st["holder"] = None
                    cur["has_lock"] = False

                    # 唤醒优先级最高的等待者, 并把锁交给它
                    blk = sorted([x for x in tasks if x["blocked"]],
                                 key=lambda x: -x["prio"])
                    if blk:
                        w = blk[0]
                        w["blocked"] = False
                        if w["lock"]:
                            w["has_lock"] = True
                            st["holder"] = w["n"]
                        logev(t, w, "被唤醒, 接手 L")
                        if w["n"] == "att" and res["att_get"] is None:
                            res["att_get"] = t
                else:
                    logev(t, cur, "完成")

            if all(x["done"] for x in tasks):
                break

        return ev, res

    for label, pi in (("❌ 没有优先级继承", False),
                      ("✅ 有优先级继承", True)):
        print("  " + "-" * 72)
        print(f"  {label}")
        print("  " + "-" * 72)
        ev, res = simulate(pi)
        for time, p, who, msg in ev:
            print(f"    t={time:>5.1f}  [{p}] {who:<4} {msg}")
        print()
        if pi:
            r2 = res
        else:
            r1 = res

    w1 = r1["att_get"] - r1["att_block"]
    w2 = r2["att_get"] - r2["att_block"]

    print(f"""  ────────────────────────────────────────────────────────────────
  【结果对比】

     驱动临界区实际长度   : {DRV_CRIT:.1f} ms
     ------------------------------------
     没有 PI 时姿态等待   : {w1:>5.1f} ms   ← 是临界区的 {w1/DRV_CRIT:.1f} 倍!
     有   PI 时姿态等待   : {w2:>5.1f} ms   ← 就是驱动【剩余的】临界区
     改善倍数             : {w1/w2:.1f}x

  【为什么会这样 —— 盯住优先级数字】

     没有 PI:
        姿态(236) 等 L, L 被 驱动(205) 持有
        驱动(205) 想继续跑, 但 日志(230) 优先级更高 → 【被抢占】
        → 姿态(236) 实际是在等 日志(230)          ← 【优先级反转】

     有 PI:
        姿态(236) 等 L
        → 驱动被临时提升到 236
        → 日志(230) 抢不过驱动了
        → 驱动跑完 {DRV_CRIT:.0f}ms 临界区、放锁
        → 姿态立刻拿到 L

  【★ 这个实验最该记住的一点】

     临界区只有 {DRV_CRIT:.0f}ms, 等待却长达 {w1:.0f}ms —— 放大了 {w1/DRV_CRIT:.1f} 倍!

     "我的临界区很短, 不会有问题" 是【错误】的推理。
     短临界区 + 优先级反转 = 长阻塞, 而且【上界不可预测】:
     中间那个日志任务可以跑任意久 (20ms / 200ms / 2s ...)。

  【PI 的本质】

     PI 【不能消除阻塞】, 只能给阻塞时间一个【上界】:

         最坏阻塞 ≤ 最长临界区 × 被阻塞的次数

     从"不可预测"变成"可计算" —— 这才是 PI 的真正价值。
     它是【可调度性分析】的前提。
     没有这个上界, 你无法证明 "1kHz 的姿态控制一定能准时"。
""")


# ============================================================================
#  实验 4: 优先级继承的完整算法
# ============================================================================
class TCB:
    """线程控制块 —— 只保留 PI 相关字段"""
    def __init__(self, name, priority):
        self.name = name
        self.base_priority = priority      # "Normal" priority —— PI 不改它
        self.sched_priority = priority     # "Current" priority —— PI 改它
        self.pend_reprios = []             # 待恢复优先级栈 (uint8_t[N])
        self.holds = []                    # 持有的 holder 列表

    def __repr__(self):
        boost = "" if self.sched_priority == self.base_priority else " ⬆BOOSTED"
        return f"{self.name}(base={self.base_priority}, cur={self.sched_priority}{boost})"


class Holder:
    """semholder_s"""
    _next_id = 0
    def __init__(self, sem, htcb):
        Holder._next_id += 1
        self.id = Holder._next_id
        self.sem = sem
        self.htcb = htcb
        self.counts = 0
        self.next = None


class Semaphore:
    """sem_t"""
    def __init__(self, name, value, prealloc=32):
        self.name = name
        self.semcount = value
        self.hhead = None
        self.pi_disabled = False
        self.pool_size = prealloc
        self.pool_free = prealloc
        self.trace = []

    def holders(self):
        out, h = [], self.hhead
        while h:
            out.append(h)
            h = h.next
        return out


NNESTPRIO = 16        # CONFIG_SEM_NNESTPRIO 默认值


def allocholder(sem, htcb):
    """nxsem_allocholder"""
    if sem.pool_free <= 0:
        raise RuntimeError(
            f"ERROR: Insufficient pre-allocated holders\n"
            f"  → 真实系统在这里 DEBUGPANIC()")
    sem.pool_free -= 1

    h = Holder(sem, htcb)
    h.next = sem.hhead          # 挂到信号量链
    sem.hhead = h
    htcb.holds.append(h)        # 挂到线程链 (tlink)
    sem.trace.append(f"    alloc holder#{h.id} for {htcb.name}  (池剩 {sem.pool_free})")
    return h


def freeholder(sem, h):
    """nxsem_freeholder"""
    sem.hhead = h.next if sem.hhead is not h else h.next
    # 从链表里摘除
    cur = sem.hhead
    prev = None
    while cur:
        if cur is h:
            if prev: prev.next = cur.next
            else: sem.hhead = cur.next
            break
        prev, cur = cur, cur.next

    if h in h.htcb.holds:
        h.htcb.holds.remove(h)
    sem.pool_free += 1
    sem.trace.append(f"    free holder#{h.id} ({h.htcb.name})  (池剩 {sem.pool_free})")


# ---------------------------------------------------------------------------
#  提权
# ---------------------------------------------------------------------------
def boost_one(H, R, sem):
    """nxsem_boostholderprio —— 对照 sem_holder.c:322-416"""
    htcb = H.htcb

    if R.sched_priority > htcb.base_priority:              # :334 门槛
        if R.sched_priority > htcb.sched_priority:         # :341 比当前还高
            if htcb.sched_priority > htcb.base_priority:   # :350 已被提过
                if len(htcb.pend_reprios) < NNESTPRIO:
                    htcb.pend_reprios.append(htcb.sched_priority)   # :356 保存现场
                    sem.trace.append(
                        f"    [boost] {htcb.name}.pend_reprios <- "
                        f"{htcb.pend_reprios[-1]}  {htcb.pend_reprios}")
            # :373 只改 sched_priority!
            old = htcb.sched_priority
            htcb.sched_priority = R.sched_priority
            sem.trace.append(
                f"    [boost] {htcb.name}.sched_priority: {old} -> "
                f"{htcb.sched_priority}   (等待者 {R.name}={R.sched_priority})")
        else:
            # :386 优先级介于 base 和 cur 之间 —— 只记账
            if len(htcb.pend_reprios) < NNESTPRIO:
                htcb.pend_reprios.append(R.sched_priority)
                sem.trace.append(
                    f"    [boost] {htcb.name}.pend_reprios <- {R.sched_priority} "
                    f"(不提升, 介于 {htcb.base_priority}~{htcb.sched_priority})")


def nxsem_boost_priority(sem, R):
    """nxsem_boost_priority —— 注意是【所有】holder"""
    sem.trace.append(f"  [sem_wait({sem.name})] {R.name} 要阻塞, 先提升持有者")
    for h in sem.holders():
        boost_one(h, R, sem)


# ---------------------------------------------------------------------------
#  恢复
# ---------------------------------------------------------------------------
def reprioritize(tcb, prio, sem, why):
    """nxsched_reprioritize —— 【会丢弃 PI 历史】"""
    tcb.sched_priority = prio
    tcb.base_priority = prio          # 同时改 base!
    tcb.pend_reprios.clear()          # 清空历史!
    sem.trace.append(f"    [restore] reprioritize({tcb.name}, {prio}) — {why}")


def set_priority(tcb, prio, sem, why):
    """nxsched_set_priority —— 【保留 base_priority】"""
    tcb.sched_priority = prio
    sem.trace.append(f"    [restore] set_priority({tcb.name}, {prio}) — {why}"
                     f"  (base={tcb.base_priority} 保留)")


def pop_max(lst):
    """取最大 + 与末元素交换删除 —— 对照 :538-555"""
    j = 0
    for i in range(1, len(lst)):
        if lst[i] > lst[j]:
            j = i
    rpriority = lst[j]
    last = len(lst) - 1
    if last > 0:
        lst[j] = lst[last]
    lst.pop()
    return rpriority


def restore_one(H, stcb, sem):
    """nxsem_restoreholderprio —— 对照 sem_holder.c:472-606"""
    htcb = H.htcb

    # :487 先释放 holder, 再改优先级
    if H.counts <= 0:
        freeholder(sem, H)

    # :496 唯一判据: 当前 != base 说明被提权过
    if htcb.sched_priority == htcb.base_priority:
        return

    if len(htcb.pend_reprios) < 1:                      # ── 情况 A (:501) ──
        reprioritize(htcb, htcb.base_priority, sem,
                     "只被提过一次, 回到 base")

    elif htcb.sched_priority <= stcb.sched_priority:    # ── 情况 B (:528) ──
        rp = pop_max(htcb.pend_reprios)
        set_priority(htcb, rp, sem,
                     f"弹最大待恢复值 {rp}, 剩余 {htcb.pend_reprios}")

    else:                                               # ── 情况 C (:563) ──
        # 比 stcb 还高 —— 只需销掉 stcb 那条借据
        for i, v in enumerate(htcb.pend_reprios):
            if v == stcb.sched_priority:
                last = len(htcb.pend_reprios) - 1
                if last > 0:
                    htcb.pend_reprios[i] = htcb.pend_reprios[last]
                htcb.pend_reprios.pop()
                sem.trace.append(
                    f"    [restore] {htcb.name} 销掉借据 {v}, "
                    f"当前 {htcb.sched_priority} 不变")
                break
        else:
            sem.trace.append(
                f"    [restore] {htcb.name} 找不到匹配借据 "
                f"{stcb.sched_priority} (可能已丢失)")


def nxsem_restore_baseprio(sem, stcb, this_task, in_irq=False):
    """nxsem_restore_baseprio —— 对照 :1038-1087"""
    if stcb is None:
        # 无人等待 —— 只回收 counts<=0 的 holder
        for h in sem.holders():
            if h.counts <= 0:
                freeholder(sem, h)
        return

    if in_irq:
        sem.trace.append("  [IRQ] 一遍遍历所有 holder（中断不是 PI 参与者）")
        for h in list(sem.holders()):
            restore_one(h, stcb, sem)
    else:
        sem.trace.append("  [TASK] 第一遍: 除自己外的所有 holder")
        for h in list(sem.holders()):
            if h.htcb is not this_task:
                restore_one(h, stcb, sem)
        sem.trace.append("  [TASK] 第二遍: 只处理自己（然后 break）")
        for h in list(sem.holders()):
            if h.htcb is this_task:
                restore_one(h, stcb, sem)
                break


# ---------------------------------------------------------------------------
def exp4_pi_algorithm():
    sep("实验 4: 优先级继承完整算法（对照源码逐行仿真）")

    print("""
  场景设计（刻意触发【情况 B】和【情况 A】）:

     线程 T   base = 100   同时持有信号量 L 和 M

     任务 C   prio = 240   等 L    → T 提升到 240
     任务 A   prio = 200   等 M    → T 不变(已经 240 了)
                                    但 pend_reprios 记下 200

   然后:
     C 拿到 L (T 释放) → 应该降到 200, 不是 100!  【情况 B】
     A 拿到 M (T 释放) → 这时才真的回到 100        【情况 A】
""")

    Holder._next_id = 0
    T = TCB("T  ", 100)
    A = TCB("A  ", 200)
    C = TCB("C  ", 240)

    L = Semaphore("L", 1)
    M = Semaphore("M", 1)

    # T 持有 L 和 M
    hL = allocholder(L, T); hL.counts = 1
    hM = allocholder(M, T); hM.counts = 1
    print(f"  初始: {T}")
    print(f"        T 持有 L 和 M")

    # ---- C 等 L ----
    print("\n  " + "-" * 72)
    print("  ① 任务 C (240) 等 L")
    print("  " + "-" * 72)
    L.trace.clear()
    nxsem_boost_priority(L, C)
    for line in L.trace: print(line)
    print(f"  → {T}")

    # ---- A 等 M ----
    print("\n  " + "-" * 72)
    print("  ② 任务 A (200) 等 M")
    print("  " + "-" * 72)
    M.trace.clear()
    nxsem_boost_priority(M, A)
    for line in M.trace: print(line)
    print(f"  → {T}")
    print(f"     pend_reprios = {T.pend_reprios}   ← 记着 A 的 200, 但不提升")

    # ---- C 拿到 L → 情况 B ----
    print("\n  " + "-" * 72)
    print("  ③ C 拿到 L, T 释放 L → restore")
    print("  " + "-" * 72)
    cond_b = T.sched_priority <= C.sched_priority
    print(f"     进入时: T.cur={T.sched_priority}, C.cur={C.sched_priority}")
    print(f"     判定:   T.cur({T.sched_priority}) <= C.cur({C.sched_priority})"
          f"  →  {'【情况 B】' if cond_b else '【情况 C】'}")
    L.trace.clear()
    hL.counts = 0
    nxsem_restore_baseprio(L, stcb=C, this_task=C)
    for line in L.trace: print(line)
    print(f"  → {T}")
    print(f"     ★ 没有跌回 base=100, 而是降到 {T.sched_priority}"
          f" —— 因为 A({A.sched_priority}) 还在等 M!")

    # ---- A 拿到 M → 情况 A ----
    print("\n  " + "-" * 72)
    print("  ④ A 拿到 M, T 释放 M → restore")
    print("  " + "-" * 72)
    case_a = not T.pend_reprios
    print(f"     进入时: T.cur={T.sched_priority}, T.pend={T.pend_reprios}")
    print(f"     判定:   pend 为空  →  {'【情况 A】' if case_a else '【情况 B】'}")
    M.trace.clear()
    hM.counts = 0
    nxsem_restore_baseprio(M, stcb=A, this_task=A)
    for line in M.trace: print(line)
    print(f"  → {T}")
    print(f"     ★ 这时才真的回到 base —— 所有提升原因都消失了")

    # ---- 演示情况 C ----
    print("\n\n  " + "-" * 72)
    print("  ⑤ 单独演示【情况 C】: 持有者比等待者还高")
    print("  " + "-" * 72)

    Holder._next_id = 0
    T2 = TCB("T2 ", 100)
    D = TCB("D  ", 200)
    E = TCB("E  ", 240)
    N = Semaphore("N", 1)

    hN = allocholder(N, T2); hN.counts = 1
    nxsem_boost_priority(N, D)          # T2 -> 200
    nxsem_boost_priority(N, E)          # T2 -> 240, pend=[200]
    print(f"     初始: {T2},  pend={T2.pend_reprios}")

    print(f"\n     现在 D(200) 拿到锁:")
    print(f"     T2.cur={T2.sched_priority} > D.cur={D.sched_priority}"
          f" → 【情况 C】")
    N.trace.clear()
    hN.counts = 0
    nxsem_restore_baseprio(N, stcb=D, this_task=D)
    for line in N.trace: print(line)
    print(f"  → {T2}")
    print(f"     ★ 销掉借据 200, 但仍保持 240 —— 因为 E 还在等!")

    print("""
  【三分支总结】

     ┌────────┬──────────────────────────────────┬──────────────────────┐
     │ 分支   │ 条件                              │ 动作                  │
     ├────────┼──────────────────────────────────┼──────────────────────┤
     │   A    │ npend_reprio < 1                 │ reprioritize(base)   │
     │        │ 只被提过一次                      │ 【改 base + 清历史】  │
     ├────────┼──────────────────────────────────┼──────────────────────┤
     │   B    │ cur <= stcb.cur                  │ 弹 pend 最大值        │
     │        │ 当前不高于刚拿锁的任务             │ set_priority 【保留base】│
     ├────────┼──────────────────────────────────┼──────────────────────┤
     │   C    │ cur >  stcb.cur                  │ 只销借据, 不改优先级   │
     │        │ 当前高于刚拿锁的任务               │                      │
     └────────┴──────────────────────────────────┴──────────────────────┘

     ★ 一句话记住情况 C:
       只有"【最后一个】让我提升的原因消失"时, 才真的降级。

     ★ 最容易写错的地方:
       · 分支 A 用 reprioritize (改 base), 分支 B 用 set_priority (保留 base)
       · 分支 B 弹最大值后还要把末元素搬到空位 (O(1) 删除)
       · 情况 C 是【查找匹配】而不是取最大
""")


# ============================================================================
#  实验 5: holder 池耗尽
# ============================================================================
def exp5_holder_exhaustion():
    sep("实验 5: holder 池耗尽 —— CONFIG_SEM_PREALLOCHOLDERS")

    print("""
  PX4 的配置 (boards/px4/fmu-v6x/nuttx-config/nsh/defconfig):

      CONFIG_PRIORITY_INHERITANCE=y
      CONFIG_SEM_PREALLOCHOLDERS=32

  一个【全局共享】的静态池, 32 个 holder, 所有信号量共用。
""")

    print("  " + "-" * 72)
    print("  模拟: 一个 signaling 信号量 + 多个等待者 → 幽灵 holder")
    print("  " + "-" * 72)

    print("""
  场景（NuttX 内核里真实存在的缺口）:

      sem_init(&sig, 0, 0);          /* 初值 0 —— signaling 用法 */
      /* 忘了设 SEM_PRIO_NONE! */

      任务 A:  sem_wait(&sig);       → 阻塞, 建立 holder (counts=1)
      中断:    sem_post(&sig);       → A 被唤醒, A 成为 holder
                                     【但 A 永远不会 post!】

  看 nxsem_release_holder() 遇到这种情况会怎样 ——
  它遍历所有 holder, 发现 post 者不是 holder, 于是:

      total = 1  →  candidate->counts--        (猜一个, 可能猜错)
      total >= 2 →  /* TODO: ... */            (什么都不做!)
""")

    Holder._next_id = 0
    sig = Semaphore("sig", 0, prealloc=32)
    POOL = 32

    print(f"  {'轮次':<6} {'动作':<34} {'池剩余':>8}  状态")
    print("  " + "-" * 72)

    # 每轮: 一个新任务 wait, 一个中断 post → 造一个幽灵 holder
    waiters = []
    for rnd in range(1, 9):
        if sig.pool_free <= 0:
            break
        w = TCB(f"W{rnd}", 100 + rnd)
        h = allocholder(sig, w)
        h.counts = 1
        waiters.append((w, h))

        status = f"幽灵 holder#{h.id} 建立"
        if len(waiters) > 2:
            status += "  ← post 者非 holder, counts 永不归零"
        print(f"  {rnd:<6} {'任务 wait + 中断 post':<34} {sig.pool_free:>8}  {status}")

    print(f"\n  ... 继续 {POOL} 轮后 ...")

    # 直接把池耗尽
    ghosts = []
    for rnd in range(9, POOL + 1):
        if sig.pool_free <= 0:
            break
        w = TCB(f"W{rnd}", 100 + (rnd % 100))
        h = allocholder(sig, w)
        h.counts = 1
        ghosts.append(h)

    print(f"  池剩余 = {sig.pool_free}")

    # 现在再要一个
    print(f"\n  " + "-" * 72)
    print("  现在任何【正常】的锁操作:")
    print("  " + "-" * 72)
    try:
        victim = TCB("受害者", 236)
        allocholder(sig, victim)
    except RuntimeError as e:
        print(f"\n  ❌ {e}\n")

    print(f"""  【后果清单】

     ① 幽灵 holder 的 counts 永远是 1, 永远不会被 nxsem_freecount0holder
        回收 —— 因为判据是 counts <= 0

     ② 幽灵占住池子 → 正常的锁操作拿不到 holder → DEBUGPANIC()
        【系统崩溃】

     ③ 更隐蔽的: 幽灵 holder 让 PI 【作用于错误的线程】
        之后任何人 wait 这个信号量, nxsem_boost_priority() 都会遍历到
        幽灵, 把那个【早就不相关】的线程提升到高优先级
        → 真正的高优先级任务反而被它抢占

     ④ pend_reprios[] 反复压栈 → 超过 CONFIG_SEM_NNESTPRIO (16)
        → Release 构建【静默丢失恢复记录】

  【修复】

      sem_init(&sig, 0, 0);
      sem_set_protocol(&sig, SEM_PRIO_NONE);   /* ← 必须加这一行! */
      /* 或者: sem_init(&sig, 0, 0) 之后立刻设 */

     实现上: sem->flags |= PRIOINHERIT_FLAGS_DISABLE;
             nxsem_destroyholder(sem);          /* 清掉已有的 */

     门卫在 nxsem_add_holder_tcb():
        if (htcb->flink != NULL && (sem->flags & PRIOINHERIT_FLAGS_DISABLE) == 0)

     ⚠️ 必须在【初始化时】设置 —— 门卫只在 add 时刻检查

  【判断标准】

     这个信号量有"谁持有"的概念吗?

       sem_init(sem, 0, 1)  → 有   (互斥)  → 保持 PI
       sem_init(sem, 0, 0)  → 没有 (通知)  → 设 SEM_PRIO_NONE

  【监控】

     nxsem_nfreeholders() 返回池里剩余数量。
     建议做成健康检查项 —— 接近 0 就告警。
""")


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  NuttX 教程 · 第 5 课 · 同步机制")
    print("=" * 76)

    exp1_race()
    exp2_cost_ladder()
    exp3_priority_inversion()
    exp4_pi_algorithm()
    exp5_holder_exhaustion()

    sep("本课小结")
    print("""
  1. 竞态 = 多流访问 + 至少一写 + 无同步
     破坏任意一个条件, 竞态就消失

  2. 保护手段的代价阶梯: 关中断 > 调度锁 > 互斥锁
     用【最弱但够用】的那一种

  3. NuttX 信号量 = 一个 16 位有符号计数
     >0 = 可用计数, <0 = 等待者数量

  4. 一个线程只能有一个 waitsem 和一个 waitdog
     → 同时只能有一个阻塞等待 + 一个定时等待

  5. 优先级继承: 把无界反转变成有界阻塞
     boost: 等待者 > 持有者当前优先级 → 提升
     restore 三分支: 只有"最后一个提升原因消失"才降级

  6. PI 用【全局共享】的静态 holder 池 (PX4 = 32)
     PREALLOCHOLDERS=0 时是 sem_t 内联 2 槽位, 不是 malloc

  7. PI 的三个已知局限:
     没有跨信号量重算 / pend_reprios 不检查等待者存活 / 溢出静默丢失

  8. nxsem_release_holder() 结尾是未解决的 TODO
     signaling 用法 + 多 holder → 泄漏 → DEBUGPANIC

  9. 互斥锁 = sem(1) + {pid,flags,type,nlocks}
     条件变量 = sem(0) + clockid
     PX4 主力是 pthread_mutex (235 次 vs 53 次)

 10. NuttX 不实现死锁检测 —— 文档里的 EDEADLK 是空承诺

 11. PX4 用锁 + PI; ArduPilot 尽量不用锁
     理解差异比判断优劣重要

  下一课: 06-设备驱动与文件系统.md
""")
