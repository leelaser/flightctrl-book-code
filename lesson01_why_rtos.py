"""
============================================================================
 NuttX 教程 · 第 1 课 · 为什么飞控需要 RTOS
 配套实验代码
============================================================================

 运行:  python lesson01_why_rtos.py
 依赖:  numpy

 本文件用 Python 模拟演示 RTOS 的核心概念 —— 不依赖真实硬件,
 目的是先建立"直觉", 第 3 课再上真实 NuttX 环境。

 三个实验:
   1. 超级循环 vs 优先级抢占  —— 看任务延迟
   2. RMS 可调度性分析        —— 用定理判断任务集能否满足
   3. 抖动对积分的影响        —— 量化 Δt 不确定性的代价
============================================================================
"""
import numpy as np


def sep(t):
    print("\n" + "=" * 76)
    print(f"  {t}")
    print("=" * 76)


# ============================================================================
#  1. 超级循环 vs 优先级抢占
# ============================================================================
class Task:
    def __init__(self, name, period_us, exec_us, priority):
        self.name = name
        self.period = period_us
        self.exec_time = exec_us
        self.priority = priority      # 数字越大优先级越高
        self.reset()

    def reset(self):
        self.next_run = 0
        self.runs = 0
        self.max_delay = 0
        self.remaining = self.exec_time   # 本轮剩余执行时间
        self.started = False              # 本轮是否已开始


def make_taskset():
    """PX4 风格的典型任务集"""
    return [
        Task("陀螺采样", period_us=1000,   exec_us=50,   priority=250),
        Task("姿态控制", period_us=1000,   exec_us=20,   priority=240),
        Task("位置控制", period_us=20000,  exec_us=200,  priority=100),
        Task("日志",     period_us=100000, exec_us=5000, priority=50),
    ]


# 模拟的时间步长 (μs)。越小越精确, 但越慢。
# 10μs 相当于 100kHz 的调度器分辨率, 远高于真实 RTOS。
SIM_TICK = 10


def simulate_superloop(tasks, duration_us):
    """超级循环: 严格顺序执行, 无法抢占

    ⚠️ 关键: 一旦某个任务开始执行, 就必须【跑完】才能轮到下一个。
       这正是问题的根源 —— 长任务会阻塞所有后续任务。
    """
    for tk in tasks:
        tk.reset()

    t = 0
    while t < duration_us:
        ran_any = False
        for tk in tasks:
            if tk.next_run <= t:
                delay = t - tk.next_run
                if delay > tk.max_delay:
                    tk.max_delay = delay
                # 原子地执行完整个任务 —— 不可打断
                if t + tk.exec_time > duration_us:
                    return tasks
                t += tk.exec_time
                tk.next_run += tk.period
                tk.runs += 1
                ran_any = True

        if not ran_any:
            # 所有任务都未到期 -> 推进时间 (但仍以 SIM_TICK 粒度)
            t += SIM_TICK
    return tasks


def simulate_rtos(tasks, duration_us):
    """RTOS: 高优先级任务可【在任意时刻】抢占低优先级

    ⚠️ 与超级循环的根本区别:
       在超级循环里, 任务一旦开始就必须跑完;
       在 RTOS 里, 每 SIM_TICK 检查一次, 高优先级任务随时可以插入。

       这就是"抢占式调度"的本质。
    """
    for tk in tasks:
        tk.reset()

    t = 0
    while t < duration_us:
        # 找出所有已到期的任务
        ready = [tk for tk in tasks if tk.next_run <= t]

        if ready:
            # 选优先级最高的执行一个时间片
            tk = max(ready, key=lambda x: x.priority)

            # 延迟 = 从"到期"到"第一次真正开始执行"
            if tk.exec_time > 0 and not tk.started:
                delay = t - tk.next_run
                if delay > tk.max_delay:
                    tk.max_delay = delay
                tk.started = True

            t += SIM_TICK
            tk.remaining -= SIM_TICK

            if tk.remaining <= 0:
                tk.next_run += tk.period
                tk.remaining = tk.exec_time
                tk.runs += 1
                tk.started = False
        else:
            t += SIM_TICK

    return tasks


def exp1_scheduler_comparison():
    sep("实验 1: 超级循环 vs 优先级抢占")

    tasks = make_taskset()

    print("  任务集 (参考 PX4 的实际配置):\n")
    print(f"  {'任务':<10} {'周期(μs)':>10} {'执行(μs)':>10} {'优先级':>8}")
    print("  " + "-" * 44)
    for tk in tasks:
        print(f"  {tk.name:<10} {tk.period:>10} {tk.exec_time:>10} {tk.priority:>8}")

    DURATION = 500_000       # 500 ms
    print(f"\n  模拟时长: {DURATION/1000:.0f} ms")

    r1 = simulate_superloop(make_taskset(), DURATION)
    r2 = simulate_rtos(make_taskset(), DURATION)

    print(f"\n  {'任务':<10} {'执行次数':>10} {'最大延迟(μs)':>14}")
    print("  " + "-" * 38)
    print("  【超级循环】")
    for tk in r1:
        print(f"    {tk.name:<8} {tk.runs:>10} {tk.max_delay:>14}")
    print("  【RTOS 抢占】")
    for tk in r2:
        print(f"    {tk.name:<8} {tk.runs:>10} {tk.max_delay:>14}")

    # 关键对比: 姿态控制的最大延迟
    name = "姿态控制"
    d1 = next(tk.max_delay for tk in r1 if tk.name == name)
    d2 = next(tk.max_delay for tk in r2 if tk.name == name)

    print(f"\n  === 关键指标: {name} 的最大延迟 ===")
    print(f"    超级循环: {d1} μs")
    print(f"    RTOS    : {d2} μs")
    if d1 > 0:
        print(f"    RTOS 改善 {d1/max(d2,1):.1f} 倍")

    assert d2 <= d1, "RTOS 的延迟不应大于超级循环"
    print("\n  [结论] 高优先级任务被低优先级任务阻塞的程度,")
    print("         就是 RTOS 与超级循环的本质区别。")

    return r1, r2


# ============================================================================
#  2. RMS 可调度性分析
# ============================================================================
def rms_bound(n):
    """Liu & Layland (1973) 的 RMS 利用率上界"""
    return n * (2.0 ** (1.0 / n) - 1.0)


def exp2_rms_analysis():
    sep("实验 2: RMS 可调度性分析")

    print("  Liu & Layland 定理: 对 n 个周期任务, 若优先级按周期倒数分配,")
    print("  则利用率满足下式时【必然可调度】:\n")
    print("      Σ(Ci/Ti) <= n * (2^(1/n) - 1)\n")

    print(f"  {'n':>3} {'上界':>10}")
    print("  " + "-" * 16)
    for n in range(1, 11):
        print(f"  {n:>3} {rms_bound(n):>10.4f}")
    print(f"  {'∞':>3} {np.log(2):>10.4f}   ← 趋于 ln(2)")

    # 分析一个真实的任务集
    print("\n  --- PX4 风格任务集 ---\n")

    taskset = [
        ("陀螺采样", 1000,   50),
        ("姿态控制", 1000,   20),
        ("位置控制", 20000, 200),
        ("导航",     50000, 500),
        ("日志",    100000, 800),
    ]

    print(f"  {'任务':<10} {'Ci(μs)':>9} {'Ti(μs)':>9} {'Ci/Ti':>10}")
    print("  " + "-" * 42)

    total = 0.0
    for name, T, C in taskset:
        u = C / T
        total += u
        print(f"  {name:<10} {C:>9} {T:>9} {u:>10.4f}")

    n = len(taskset)
    bound = rms_bound(n)
    print("  " + "-" * 42)
    print(f"  {'合计':<10} {'':>9} {'':>9} {total:>10.4f}")

    print(f"\n  任务数 n = {n}")
    print(f"  RMS 上界 = {bound:.4f}")
    print(f"  实际利用率 = {total:.4f}")

    if total <= bound:
        print(f"\n  ✓ 利用率 {total:.4f} <= 上界 {bound:.4f}")
        print(f"    ==> 由 RMS 定理, 该任务集【必然可调度】")
        margin = (bound - total) / bound * 100
        print(f"    余量: {margin:.1f}%")
    else:
        print(f"\n  ⚠ 利用率 {total:.4f} > 上界 {bound:.4f}")
        print(f"    ==> RMS 定理【不能保证】可调度")
        print(f"    注意: 这只是充分条件! 仍可能实际可调度,")
        print(f"          需要用【响应时间分析(RTA)】做精确判断。")

    print(f"\n  [工程解读] 飞控的 CPU 占用率通常 20~40%,")
    print(f"             看起来【浪费】, 但这正是实时性的余量。")
    print(f"             必须为最坏情况 (传感器突发/日志写入/异常) 留空间。")

    return total, bound


# ============================================================================
#  3. 抖动对积分的影响
# ============================================================================
def exp3_jitter_effect():
    sep("实验 3: Δt 抖动对姿态积分的影响")

    def integrate(omega, dt_nominal, n_steps, jitter_ratio, bias_ratio, seed):
        """带抖动和偏差的积分"""
        rng = np.random.default_rng(seed)
        angle = 0.0
        for _ in range(n_steps):
            # 抖动: 随机 ±jitter_ratio
            j = rng.uniform(-jitter_ratio, jitter_ratio)
            # 偏差: 系统性偏移
            dt = dt_nominal * (1.0 + bias_ratio + j)
            angle += omega * dt
        return angle

    omega = 1.0
    dt_nom = 0.001
    N = 10000
    truth = omega * N * dt_nom       # 真实角度

    print(f"  设定: ω={omega} rad/s, 标称 dt={dt_nom*1e6:.0f}μs, {N} 步")
    print(f"  真实总角度 = {truth:.4f} rad\n")

    # --- A: 只有随机抖动 ---
    print("  A) 只有随机抖动 (无系统性偏差)")
    print(f"  {'抖动幅度':>10} {'积分结果':>12} {'相对误差':>12}")
    print("  " + "-" * 38)
    for jr in [0.0, 0.01, 0.05, 0.10, 0.50, 1.00]:
        a = integrate(omega, dt_nom, N, jr, 0.0, 42)
        err = abs(a - truth) / truth * 100
        print(f"  {jr*100:>9.0f}% {a:>12.6f} {err:>11.4f}%")

    print("\n  [关键结论] 随机抖动【不直接】造成积分误差!")
    print("             因为正负抖动长期抵消, 均值仍接近真值。")
    print("             这是好消息: 飞控对随机抖动不敏感。")

    # --- B: 系统性偏差 ---
    print("\n  B) 只有系统性偏差 (平均 dt 偏离标称值)")
    print(f"  {'偏差':>10} {'积分结果':>12} {'相对误差':>12}")
    print("  " + "-" * 38)
    for br in [-0.05, -0.01, 0.0, 0.01, 0.05, 0.10]:
        a = integrate(omega, dt_nom, N, 0.0, br, 42)
        err = abs(a - truth) / truth * 100
        print(f"  {br*100:>9.0f}% {a:>12.6f} {err:>11.4f}%")

    print("\n  [关键结论] 系统性偏差【1:1 传递】到积分结果!")
    print("             dt 平均偏大 5% -> 角度就偏大 5%, 而且【不抵消】。")
    print("             这才是致命的。")

    # --- C: 组合 ---
    print("\n  C) 抖动 + 偏差 组合")
    a = integrate(omega, dt_nom, N, 0.10, 0.05, 42)
    err = abs(a - truth) / truth * 100
    print(f"  抖动 ±10% + 偏差 +5% -> 误差 {err:.4f}%")
    print(f"  ==> 误差主要由【偏差】贡献, 抖动只是叠加了一点噪声")

    print(f"""
  [工程含义]
    1. RTOS 要保证的是【时间上界】(deterministic), 而不是"平均够快"
    2. 但真正的精度杀手是【系统性偏差】——
       比如调度周期总是比设定值长 (因为内核 tick 粒度、任务切换开销)
    3. 所以 PX4 用【硬件定时器测量实际间隔】(HRT 时间戳),
       而不是假设 dt 就是设定值

    📁 对应实现: platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c
       hrt_tim_isr() 里第一件事就是 latency_actual = rCNT
       —— 记录中断【实际】发生的时刻, 用于统计延迟。
""")


# ============================================================================
if __name__ == "__main__":
    print("=" * 76)
    print("  NuttX 教程 · 第 1 课 · 为什么飞控需要 RTOS")
    print("=" * 76)

    exp1_scheduler_comparison()
    exp2_rms_analysis()
    exp3_jitter_effect()

    sep("本课小结")
    print("""
  1. 超级循环三大问题: 时序不确定、慢任务阻塞、无法抢占
  2. RTOS 提供: 优先级抢占、确定性调度、同步原语
  3. RMS 定理: 利用率 <= n(2^(1/n)-1), n 大时趋于 ln2 ≈ 0.693
  4. 内核 tick ≠ 控制周期:
       NuttX 内核 tick = 1ms (CONFIG_USEC_PER_TICK=1000)
       PX4 控制精度靠 1MHz 硬件定时器 (HRT)
  5. 飞控 CPU 占用低是【特性】——留余量给最坏情况
  6. 随机抖动不致命, 【系统性偏差】才致命
     -> 所以必须【测量】实际间隔, 而不是假设

  下一课: 02-NuttX是什么-架构与定位.md
""")
