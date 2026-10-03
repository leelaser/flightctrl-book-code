#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 6 章（软件架构）配图生成脚本 —— 8 张图，全部由数据/源码常量生成。

为什么单独写一个画图脚本
------------------------
第 6 章讲的是「架构」：模块怎么连、线程怎么排、一张表怎么撑起实时性。
这些内容**天生是图和时序**，纯文字要读者自己在脑子里搭图。
所以这里把课文里已经写死的**源码常量**（`WorkQueueManager.hpp` 的
`wq_config_t` 表、`Copter.cpp` 的任务表、`default.px4board` 的串口行）
直接搬进画图脚本 —— 图里的每一个数字都能在课文里找到出处，
改一个常量重跑脚本，图就跟着变。

用法：
    python figures_ch6.py            # 生成全部 8 张
    python figures_ch6.py l1         # 只生成 01 课（uORB）的 3 张
    python figures_ch6.py l2         # 只生成 02 课（ArduPilot）的 3 张
    python figures_ch6.py l3         # 只生成 03 课（BSP）的 2 张

输出（全部落在本目录下）：
    01 课   fig_uorb_dataflow.png        发布/订阅数据流图
            fig_uorb_threads.png         工作队列的栈大小与相对优先级
            fig_uorb_pubsub_timing.png   publish→subscribe 时序 + 稳态延迟 q-1
    02 课   fig_ap_tasktable.png         rate_hz × max_time_micros 预算甘特图
            fig_ap_mainloop.png          主循环一轮的调用顺序与预算扣减
            fig_arch_compare.png         PX4 vs ArduPilot 同一条控制路径
    03 课   fig_bsp_layers.png           BSP 各层关系与两条构建路径
            fig_bsp_serial_map.png       角色 → /dev/ttyS* → UART 三层映射

只依赖 numpy + matplotlib。缺 matplotlib 时打印提示并退出 0（不报错）。
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DPI = 140

# ---------------------------------------------------------------------------
# 画图环境（CJK 字体自动探测 —— 与 04-状态估计/code/lesson10_kalman.py 同款做法）
# ---------------------------------------------------------------------------

_HAS_MPL = False
_CJK = False
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyBboxPatch, Rectangle
    import matplotlib.font_manager as _fm

    for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                  "Source Han Sans SC", "WenQuanYi Micro Hei",
                  "PingFang SC", "Heiti SC"):
        try:
            _fm.findfont(_name, fallback_to_default=False)
        except Exception:
            continue
        matplotlib.rcParams["font.sans-serif"] = [_name]
        matplotlib.rcParams["axes.unicode_minus"] = False
        _CJK = True
        break

    _HAS_MPL = True
except ImportError:
    pass


def _t(zh, en):
    """中文标签 / 英文回退。找不到 CJK 字体时图里全是英文。"""
    return zh if _CJK else en


def head(title):
    print()
    print("=" * 74)
    print("  " + title)
    print("=" * 74)


def saved(path):
    print("    ✅ 已存 %s  (%d KB)" % (os.path.basename(path),
                                    os.path.getsize(path) // 1024))


# ---------------------------------------------------------------------------
# 画图小工具：一堆「框 + 箭头」，所有坐标都在 0~100 的虚拟画布上
# ---------------------------------------------------------------------------

C_BG = "#f7f8fa"


def new_canvas(w=14.0, h=8.0):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.set_facecolor(C_BG)
    ax.axis("off")
    return fig, ax


def box(ax, x, y, w, h, text, fc="#ffffff", ec="#333333", tc="#111111",
        fs=9.0, lw=1.3, weight="normal", r=1.2, ls="solid", va="center"):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0,rounding_size=%g" % r,
                                facecolor=fc, edgecolor=ec, linewidth=lw,
                                linestyle=ls, zorder=2))
    ax.text(x + w / 2.0, y + h / 2.0, text, ha="center", va=va,
            fontsize=fs, color=tc, weight=weight, zorder=3, linespacing=1.45)


def band(ax, x, y, w, h, fc="#eef2f7", ec="#c8d2de", lw=1.0):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=fc, edgecolor=ec,
                           linewidth=lw, zorder=1))


def arrow(ax, p0, p1, color="#2f6fb5", lw=1.5, style="-|>", rad=0.0,
          ls="solid", z=4, ms=8):
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle=style, color=color, lw=lw,
                                linestyle=ls, mutation_scale=ms,
                                shrinkA=0, shrinkB=0,
                                connectionstyle="arc3,rad=%g" % rad),
                zorder=z)


def tag(ax, x, y, text, fs=8.0, color="#444444", ha="center", va="center",
        weight="normal", bg=None, rot=0):
    kw = {}
    if bg:
        kw["bbox"] = dict(boxstyle="round,pad=0.25", facecolor=bg,
                          edgecolor="none")
    ax.text(x, y, text, fontsize=fs, color=color, ha=ha, va=va,
            weight=weight, zorder=6, rotation=rot, linespacing=1.4, **kw)


def title(ax, text, sub=None):
    ax.text(50, 97.5, text, ha="center", va="top", fontsize=12.5,
            weight="bold", color="#1a1a1a", zorder=6)
    if sub:
        ax.text(50, 92.5, sub, ha="center", va="top", fontsize=9.0,
                color="#555555", zorder=6)


# ===========================================================================
# 01 课 · 图 1：uORB 发布 / 订阅数据流图
# ===========================================================================

def fig_uorb_dataflow():
    fig, ax = new_canvas(15.5, 9.4)
    title(ax, _t("图 6-1-1  uORB 数据流：谁发布、谁订阅、经过哪些 topic",
                 "Fig 6-1-1  uORB data flow"),
          _t("★ 看箭头【没有一条对准某个具体订阅者】—— 发布者只往 topic 上贴，"
             "不知道谁在看（§21.2.1）", "publish/subscribe fan-out"))

    # ---- 三栏底色 ----
    band(ax, 2.0, 21.0, 26.0, 61.0)     # 生产者
    band(ax, 37.0, 21.0, 26.0, 61.0)    # uORB
    band(ax, 72.0, 21.0, 26.0, 61.0)    # 消费者
    tag(ax, 15.0, 84.3, _t("生产者（线程 / 队列项）", "producers"), fs=10.0,
        weight="bold", color="#1f4e79")
    tag(ax, 50.0, 84.3, _t("uORB topic（共享环形缓冲）", "uORB topics"), fs=10.0,
        weight="bold", color="#7d4a00")
    tag(ax, 85.0, 84.3, _t("消费者（线程 / 队列项）", "consumers"), fs=10.0,
        weight="bold", color="#1f4e79")

    # 6 行，从 y=74 往下每行 9.8
    ROWS = [74.0 - i * 9.8 for i in range(6)]
    H = 8.0

    # ---- 生产者 ----
    prod = [
        _t("sensors 模块\nwq:SPI0…SPI6（各占一个）", "sensors\nwq:SPI0..SPI6"),
        _t("VehicleIMU / VehicleAngularVelocity\n（在 sensors 里做积分与滤波）",
           "VehicleIMU"),
        _t("ekf2\nwq:INS0", "ekf2  wq:INS0"),
        _t("navigator\nwq:nav_and_controllers", "navigator"),
        _t("mc_rate_control / mc_att_control\nwq:rate_ctrl（优先级 0）",
           "rate controllers"),
        _t("commander / mavlink / logger", "commander etc."),
    ]
    for y, txt in zip(ROWS, prod):
        box(ax, 3.0, y, 24.0, H, txt, fc="#dbe9f7", ec="#4a7fb5", fs=7.8)

    # ---- uORB topic 栏 ----
    topics = [
        _t("sensor_gyro / sensor_accel\nq=8", "sensor_gyro/accel  q=8"),
        _t("sensor_combined\nvehicle_angular_velocity", "sensor_combined"),
        _t("vehicle_attitude\nvehicle_local_position", "vehicle_attitude"),
        _t("position_setpoint / vehicle_command\nq=1（状态快照）",
           "setpoints  q=1"),
        _t("actuator_motors\nq=1", "actuator_motors  q=1"),
        _t("vehicle_status / vehicle_command_ack", "vehicle_status"),
    ]
    for y, txt in zip(ROWS, topics):
        box(ax, 38.0, y, 24.0, H, txt, fc="#fff3d6", ec="#c99a2e", fs=7.8)

    # ---- 消费者 ----
    cons = [
        _t("sensors 前端 / GyroFFT\n（订阅原始样本）", "GyroFFT"),
        _t("ekf2\n250 Hz", "ekf2  250 Hz"),
        _t("mc_rate_control / mc_att_control\n内环 → 外环", "controllers"),
        _t("navigator\n100 Hz", "navigator  100 Hz"),
        _t("pwm_out / dshot\n（→ 电机）", "pwm_out"),
        _t("logger（50 Hz）/ mavlink\n★ 慢订阅者", "logger / mavlink"),
    ]
    for y, txt in zip(ROWS, cons):
        box(ax, 73.0, y, 24.0, H, txt, fc="#e3f0e3", ec="#4a9a5b", fs=7.8)

    # ---- 箭头：生产者 → topic（右向） ----
    for i, y in enumerate(ROWS):
        arrow(ax, (27.0, y + H / 2), (38.0, ROWS[i] + H / 2))

    # ---- 箭头：topic → 消费者（右向，交叉是【有意】的） ----
    pairs = [(0, 0), (1, 1), (1, 2), (2, 2), (2, 3), (3, 3), (4, 4), (5, 5),
             (0, 5)]
    for a, b in pairs:
        arrow(ax, (62.0, ROWS[a] + H / 2), (73.0, ROWS[b] + H / 2),
              color="#4a9a5b", rad=0.0)

    # ---- 三条机制说明（挪到底部，横排三栏） ----
    for x, txt in ((2.0, _t("发布：memcpy 进 (gen % q) 槽\n写完就走，不等人",
                            "publish: write & go")),
                   (34.5, _t("★ 一份共享缓冲 + N 个读指针\n（没有 per-subscriber 队列）",
                             "one buffer, N read pointers")),
                   (67.0, _t("订阅：自己记 _last_generation\n自己算下标去读",
                             "subscriber keeps its own gen"))):
        box(ax, x, 10.6, 31.0, 8.4, txt, fc="#eef2f7", ec="#8899aa",
            fs=7.9, tc="#334455")

    box(ax, 2.0, 1.6, 96.0, 7.6,
        _t("★ 看图说话：这里【没有任何一条箭头是「专线」】。\n"
           "加一个 logger 订阅 vehicle_attitude —— 不用改 ekf2 一行代码，"
           "不用通知发布者，也不用新建队列；发布者的写操作次数与订阅者个数【无关】。",
           "Fig: no dedicated links — adding a subscriber changes nothing upstream"),
        fc="#fdf3d0", ec="#c99a2e", fs=8.4, tc="#5a3d00")

    fig.tight_layout()
    p = os.path.join(HERE, "fig_uorb_dataflow.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 01 课 · 图 2：工作队列的栈大小与相对优先级
# ===========================================================================

# 全部抄自 platforms/common/include/px4_platform_common/px4_work_queue/
# WorkQueueManager.hpp:51-92（逐字，只取了 fmu-v6x 上真实会用到的那几条）
WQ = [
    ("wq:rate_ctrl",           3150,   0, "内环角速率控制（最高优先级）"),
    ("wq:SPI0",                2392,  -1, "SPI 总线 0（会阻塞，独占）"),
    ("wq:SPI1",                2392,  -2, "SPI 总线 1"),
    ("wq:I2C0",                2336,  -8, "I2C 总线 0（会阻塞，独占）"),
    ("wq:nav_and_controllers", 2240, -13, "导航 + 控制器（数据流串行）"),
    ("wq:INS0",                6000, -14, "传感器前端（矩阵运算多）"),
    ("wq:INS1",                6000, -15, "传感器前端（第二组 IMU）"),
    ("wq:hp_default",          2800, -18, "小模块共享队列"),
    ("wq:uavcan",              3624, -19, "UAVCAN 协议栈"),
    ("wq:ttyS0",               1728, -21, "串口 0（栈最小）"),
    ("wq:ttyS5",               1728, -26, "串口 5（本例是 RC）"),
    ("wq:lp_default",          1920, -50, "低优先级杂活"),
]


def fig_uorb_threads():
    fig, axes = plt.subplots(1, 2, figsize=(15.0, 7.6),
                             gridspec_kw=dict(width_ratios=[2.05, 1.0]))
    fig.patch.set_facecolor("white")

    names = [w[0] for w in WQ]
    stacks = [w[1] for w in WQ]
    prios = [w[2] for w in WQ]
    y = np.arange(len(WQ))

    ax = axes[0]
    ax.set_facecolor(C_BG)
    colors = ["#e05252" if p == 0 else ("#f0a53a" if p > -20 else "#5b8fd6")
              for p in prios]
    ax.barh(y, stacks, color=colors, edgecolor="#333333", linewidth=0.7,
            height=0.62, zorder=2)
    ax.set_yticks(y)
    ax.set_yticklabels(names, fontsize=8.6)
    ax.invert_yaxis()
    ax.set_xlabel(_t("线程栈大小（字节）—— stacksize 字段", "stack size (bytes)"),
                  fontsize=9.5)
    ax.set_xlim(0, 7600)
    ax.grid(axis="x", ls=":", color="#bbbbbb", zorder=0)
    for i, (s, p) in enumerate(zip(stacks, prios)):
        ax.text(s + 90, i, "%d B   rel_prio=%d" % (s, p), va="center",
                fontsize=8.0, color="#333333")
    ax.axvline(1728, color="#2b7a3d", ls="--", lw=1.2, zorder=1)
    ax.axvline(6000, color="#b02b2b", ls="--", lw=1.2, zorder=1)
    ax.text(1760, len(WQ) - 0.35,
            _t("最小 1728（ttyS*）", "min 1728 (ttyS*)"), fontsize=8.2,
            color="#2b7a3d")
    ax.text(6060, len(WQ) - 0.35,
            _t("最大 6000（INS0/1）\n★ 相差 3.5 倍", "max 6000\n3.5x"),
            fontsize=8.2, color="#b02b2b", va="top")
    ax.set_title(_t("图 6-1-2a  每条工作队列的栈大小 —— 红=优先级 0（最高）",
                    "Fig 6-1-2a  stack size per work queue"),
                 fontsize=11.0, weight="bold", pad=10)

    # 右图：相对优先级轴
    ax2 = axes[1]
    ax2.set_facecolor(C_BG)
    ax2.barh(y, [-p for p in prios], color=colors, edgecolor="#333333",
             linewidth=0.7, height=0.62, zorder=2)
    ax2.set_yticks(y)
    ax2.set_yticklabels(["" for _ in WQ])
    ax2.invert_yaxis()
    ax2.set_xlabel(_t("相对优先级（取负，0 = 最高）", "relative priority"),
                   fontsize=9.5)
    ax2.grid(axis="x", ls=":", color="#bbbbbb", zorder=0)
    ax2.set_title(_t("图 6-1-2b  优先级是按【数据流】排的",
                     "Fig 6-1-2b  priority follows the data flow"),
                  fontsize=11.0, weight="bold", pad=10)
    ax2.annotate(_t("内环（角速率）", "inner loop"),
                 xy=(4, 0), xytext=(22, 1.9), fontsize=8.6, color="#b02b2b",
                 arrowprops=dict(arrowstyle="-|>", color="#b02b2b", lw=1.2))
    ax2.annotate(_t("传感器前端", "sensor front-end"),
                 xy=(17, 6), xytext=(24, 4.4), fontsize=8.6, color="#7a4a00",
                 arrowprops=dict(arrowstyle="-|>", color="#7a4a00", lw=1.2))
    ax2.annotate(_t("串口/杂活（最低）\n★ 但栈也最小", "serial / misc"),
                 xy=(50, 11), xytext=(14, 8.6), fontsize=8.6, color="#1f4e79",
                 arrowprops=dict(arrowstyle="-|>", color="#1f4e79", lw=1.2))

    fig.suptitle(_t("★ 看图说话：栈大小和优先级【不是】一回事 —— "
                    "优先级最高的是 rate_ctrl（3150 B），栈最大的是 INS0（6000 B）；"
                    "而 ttyS* 队列的优先级最低，它们各占一个队列只为「互不拖累」",
                    "stack size != priority"),
                 fontsize=9.6, color="#5a3d00", y=0.035)
    fig.tight_layout(rect=[0, 0.055, 1, 1])
    p = os.path.join(HERE, "fig_uorb_threads.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 01 课 · 图 3：publish → subscribe 时序 + 稳态延迟 = q - 1
# ===========================================================================

def sim_subscriber(q, f_pub, f_sub, dur=1.0, jitter=None):
    """照抄 §21.2.4 的 copy() 逻辑，返回订阅者每次 update() 读到的代。

    对齐 platforms/common/uORB/uORBDeviceNode.hpp:225-263：
        current = _generation.load()          # 已发布的条数（下一代的编号）
        if current == generation:  --generation
        if not is_in_range(current - q, generation, current - 1):
            generation = current - q          # 落后太多 ⇒ 夹到最老的那一份
        read = generation                      # 读的是「第 generation 代」
        generation += 1                        # ★ 读完之后前进一代

    jitter: [(t, 额外条数)]，用来模拟「发布者突然连发几条」（§21.2.6 第 ⑤ 节）。
    """
    pub_t, sub_t = [], []
    t, gen_pub = 0.0, 0
    dt_pub = 1.0 / f_pub
    while t < dur + dt_pub:
        pub_t.append((round(t, 9), 1))
        t += dt_pub
    if jitter:
        for (jt, extra) in jitter:
            pub_t.append((jt, extra))
        pub_t.sort(key=lambda z: z[0])

    gen_now = 0        # _generation：已发布条数
    pi = 0
    t = 0.0
    dt_sub = 1.0 / f_sub
    generation = 0     # 订阅者的 _last_generation
    while t < dur + dt_sub:
        while pi < len(pub_t) and pub_t[pi][0] <= t + 1e-12:
            gen_now += pub_t[pi][1]
            pi += 1
        cur = gen_now
        if cur == generation:
            generation -= 1
        if not (cur - q <= generation <= cur - 1):
            generation = cur - q
        sub_t.append((round(t, 9), generation, cur, max(cur - 1 - generation, 0)))
        generation += 1
        t += dt_sub
    return pub_t, sub_t


def fig_uorb_pubsub_timing():
    q = 4
    f_pub, f_sub = 100.0, 10.0
    pub_t, sub_t = sim_subscriber(q, f_pub, f_sub, dur=0.6)

    # ---- 自检：稳态落后必须等于 q-1 ----
    steady = [r[3] for r in sub_t if r[2] >= q + 1]
    lag_ss = steady[-1] if steady else -1
    print("    · 自检 q=%d, f_pub=%.0fHz, f_sub=%.0fHz ⇒ 稳态落后 %d 代 "
          "(期望 q-1=%d) %s" % (q, f_pub, f_sub, lag_ss, q - 1,
                                "OK" if lag_ss == q - 1 else "✗"))
    assert lag_ss == q - 1, "与 §21.2.5 的推导不符"

    # ---- 表：q 与延迟（对应 §21.2.5 的实测表）----
    qs = [1, 2, 4, 8, 16]
    lags, ms = [], []
    for qq in qs:
        _, s = sim_subscriber(qq, 100.0, 10.0, dur=1.2)
        st = [r[3] for r in s if r[2] >= qq + 1]
        lags.append(st[-1])
        ms.append(st[-1] * 10.0)      # 发布周期 10 ms

    # 订阅者 A（100 Hz，跟得上）与 C（10 Hz，跟不上）的真实读取序列
    _, sub_a = sim_subscriber(q, f_pub, f_pub, dur=0.14)
    _, sub_c = sim_subscriber(q, f_pub, f_sub, dur=0.25)

    fig, axes = plt.subplots(1, 2, figsize=(15.4, 7.9),
                             gridspec_kw=dict(width_ratios=[2.4, 1.0]))
    fig.patch.set_facecolor("white")

    # ================= 左：竖排时序图（4 条生命线） =================
    ax = axes[0]
    ax.set_facecolor(C_BG)
    x_pub, x_buf, x_a, x_c = 10.0, 38.0, 64.0, 89.0
    NROW = 13
    ytop, dy = 90.0, 6.2
    ybot = 14.5

    for x, lab, col in ((x_pub, _t("发布者\n100 Hz", "publisher"), "#b02b2b"),
                        (x_buf, _t("共享环形缓冲\nq = 4", "ring buffer"), "#8a5a00"),
                        (x_a, _t("订阅者 A\n100 Hz", "sub A"), "#2b7a3d"),
                        (x_c, _t("订阅者 C\n10 Hz", "sub C"), "#1f4e79")):
        ax.plot([x, x], [ybot, 93.0], color=col, lw=1.6, zorder=2)
        tag(ax, x, 96.0, lab, fs=9.0, color=col, weight="bold")

    def rowy(g):
        return ytop - g * dy

    # 每一代：发布者 → 缓冲
    for g in range(NROW):
        y = rowy(g)
        ax.plot([x_pub, x_c], [y, y], color="#e4e4e4", lw=0.6, zorder=1)
        arrow(ax, (x_pub, y), (x_buf, y), color="#cc7777", lw=0.9, ms=5)
        tag(ax, (x_pub + x_buf) / 2.0, y + 1.1,
            _t("gen %d → 槽 %d", "gen %d -> slot %d") % (g, g % q),
            fs=6.4, color="#8a5a00")

    # 订阅者 A：每次 update 前进一代 ⇒ 第 g 行读的就是 gen g
    for (t, gen, cur, lag) in sub_a:
        g = int(round(cur - 1))
        if g >= NROW:
            continue
        y = rowy(g)
        arrow(ax, (x_buf, y), (x_a, y), color="#4a9a5b", lw=0.9, ms=5)
        tag(ax, (x_buf + x_a) / 2.0, y + 1.1, _t("读 gen %d", "read gen %d") % gen,
            fs=6.4, color="#2b7a3d")

    # 订阅者 C：两次读都【落后 q-1 代】
    c_hits = []
    for (t, gen, cur, lag) in sub_c:
        g = int(round(cur - 1))
        if g >= NROW:
            continue
        c_hits.append((g, gen, lag))
        y = rowy(g) - 2.9
        ax.plot([x_buf, x_buf], [rowy(g), y], color="#9db4d4", lw=0.8,
                ls=":", zorder=3)
        ax.plot([x_c, x_c], [y, rowy(g)], color="#9db4d4", lw=0.8, ls=":",
                zorder=3)
        arrow(ax, (x_buf, y), (x_c, y), color="#3f6fb5", lw=1.0, ms=5)
        tag(ax, (x_buf + x_c) / 2.0, y + 0.9,
            _t("t=%d ms 读 gen %d", "t=%d ms read gen %d") % (round(t * 1000), gen),
            fs=6.4, color="#1f4e79", bg=C_BG)

    # C 的落后：从「读的那一刻」向上连到「那一代被发布的时刻」
    if len(c_hits) >= 2:
        g_read, gen_read, lag_now = c_hits[1]
        y_hi, y_lo = rowy(gen_read), rowy(g_read)
        xd = x_c + 2.4
        ax.plot([xd, xd], [y_hi, y_lo], color="#b02b2b", lw=1.4, ls="--",
                zorder=5)
        ax.annotate("", xy=(xd, y_hi), xytext=(xd, y_lo),
                    arrowprops=dict(arrowstyle="<->", color="#b02b2b", lw=1.4),
                    zorder=6)
        tag(ax, 76.0, (y_hi + y_lo) / 2.0,
            _t("★ 落后 %d 代 = q-1 = %d ms" % (lag_now, lag_now * 10),
               "★ lag = q-1"), fs=7.8, color="#b02b2b", weight="bold",
            bg="#fff4d0")

    tag(ax, 68.0, rowy(3) + 3.0,
        _t("★ A 每代都读得到\n（落后 0 代）", "★ A: lag 0"),
        fs=7.6, color="#2b7a3d", weight="bold", ha="left", bg="#ffffff")

    box(ax, 2.0, 1.4, 96.0, 11.6,
        _t("★ 看图说话：订阅者不是「总抓最新」，而是「自己记 _last_generation，每次 update 前进一代」（§21.2.4 ②）。\n"
           "100 Hz 的 A 恰好每代都读得到（落后 0）；10 Hz 的 C 每读一次，发布者已经又跑了 10 代 ——\n"
           "它的读指针被夹在【q-1 代】之前，而且【与它自己多快无关】：这就是 q 的代价，稳态延迟。",
           "★ the reader advances one generation per update; slow readers clamp at q-1"),
        fc="#fdf3d0", ec="#c99a2e", fs=8.0, tc="#5a3d00")
    ax.set_title(_t("图 6-1-3a  publish → subscribe 竖排时序（q = 4，100 Hz 发布）",
                    "Fig 6-1-3a  publish->subscribe sequence"),
                 fontsize=11.0, weight="bold", pad=8)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    # ================= 右：q 与延迟 =================
    ax2 = axes[1]
    ax2.set_facecolor(C_BG)
    xs = np.arange(len(qs))
    bars = ax2.bar(xs, ms, color=["#5b8fd6", "#5b8fd6", "#f0a53a", "#e05252",
                                  "#8a2b2b"],
                   edgecolor="#333333", linewidth=0.8, width=0.6, zorder=2)
    ax2.set_xticks(xs)
    ax2.set_xticklabels(["q=%d" % v for v in qs], fontsize=9.5)
    ax2.set_ylabel(_t("稳态延迟（ms）—— 发布 100 Hz，订阅 10 Hz", "lag (ms)"),
                   fontsize=9.4)
    ax2.grid(axis="y", ls=":", color="#bbbbbb", zorder=0)
    for b, l, m in zip(bars, lags, ms):
        ax2.text(b.get_x() + b.get_width() / 2, m + 4.0,
                 "%d 代\n%.0f ms" % (l, m), ha="center", fontsize=8.4,
                 color="#222222")
    ax2.set_ylim(0, max(ms) * 1.32)
    ax2.set_title(_t("图 6-1-3b  延迟 = q - 1（与订阅者频率无关）",
                     "Fig 6-1-3b  lag = q-1"), fontsize=11.0, weight="bold",
                  pad=8)
    ax2.annotate(_t("★ q=16 时，一个 10 Hz 的订阅者\n读的是 150 ms 前的数据\n"
                    "—— 姿态若这样，控制回路早就发散了",
                    "q=16 -> 150 ms stale"),
                 xy=(4, ms[-1]), xytext=(0.55, max(ms) * 0.72), fontsize=8.4,
                 color="#8a2b2b",
                 arrowprops=dict(arrowstyle="-|>", color="#8a2b2b", lw=1.3))

    fig.suptitle(_t("★ 看图说话：q 调大的收益是「抖动时不丢样本」，"
                    "代价是「稳态永久落后 q-1 代」—— 两条曲线在 §21.2.6 都量化过",
                    "trade-off of o_queue"),
                 fontsize=9.6, color="#5a3d00", y=0.03)
    fig.tight_layout(rect=[0, 0.05, 1, 1])
    p = os.path.join(HERE, "fig_uorb_pubsub_timing.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 02 课 · 图 1：任务表预算甘特图（rate_hz × max_time_micros）
# ===========================================================================

# 逐字抄自 ArduCopter/Copter.cpp:150-220（课文 §22.2.1 的表）
TASKS = [
    ("rc_loop",                   250, 130,   3),
    ("throttle_loop",              50,  75,   6),
    ("fence_check",                25, 100,   7),
    ("AP_GPS::update",             50, 200,   9),
    ("run_nav_updates",            50, 100,  45),
    ("one_hz_loop",                 1, 100,  81),
    ("ekf_check",                  10,  75,  84),
    ("GCS::update_receive",       400, 180, 102),
    ("GCS::update_send",          400, 550, 105),
    ("ten_hz_logging_loop",        10, 350, 114),
    ("AP_Logger::periodic_tasks", 400, 300, 120),
]
LOOP_RATE = 400.0
LOOP_US = 1e6 / LOOP_RATE          # 2500 us，AP_Scheduler.cpp:41-42 的默认值


def fig_ap_tasktable():
    fig, axes = plt.subplots(1, 2, figsize=(15.4, 7.4),
                             gridspec_kw=dict(width_ratios=[1.35, 1.0]))
    fig.patch.set_facecolor("white")

    names = [t[0] for t in TASKS]
    buds = [t[2] for t in TASKS]
    rates = [t[1] for t in TASKS]
    prios = [t[3] for t in TASKS]
    y = np.arange(len(TASKS))
    fast = [p <= 3 for p in prios]

    # ---------- 左：单次预算 ----------
    ax = axes[0]
    ax.set_facecolor(C_BG)
    cols = ["#e05252" if f else ("#f0a53a" if b >= 300 else "#5b8fd6")
            for f, b in zip(fast, buds)]
    ax.barh(y, buds, color=cols, edgecolor="#333333", linewidth=0.8,
            height=0.62, zorder=2)
    ax.set_yticks(y)
    ax.set_yticklabels(["%s\n(%g Hz, prio %d)" % (n, r, p)
                        for n, r, p in zip(names, rates, prios)], fontsize=8.2)
    ax.invert_yaxis()
    ax.set_xlabel(_t("max_time_micros —— 声明式预算（微秒）", "max_time_micros"),
                  fontsize=9.5)
    ax.set_xlim(0, 760)
    ax.grid(axis="x", ls=":", color="#bbbbbb", zorder=0)
    ax.axvline(LOOP_US, color="#2b7a3d", ls="--", lw=1.4, zorder=1)
    ax.text(LOOP_US + 8, 0.15, _t("一个循环周期 2500 us", "loop 2500 us"),
            fontsize=8.4, color="#2b7a3d", weight="bold")
    for i, (b, f) in enumerate(zip(buds, fast)):
        ax.text(b + 8, i, "%d us%s" % (b, _t("  快任务", " fast") if f else ""),
                va="center", fontsize=8.0,
                color="#b02b2b" if f else "#333333")
    tot = sum(buds)
    ax.set_title(_t("图 6-2-1a  表里每项的声明预算\n"
                    "★ 全部 11 项预算之和 = %d us ≈ 一个循环周期 2500 us"
                    % tot, "Fig 6-2-1a  declared budgets"),
                 fontsize=10.6, weight="bold", pad=10)

    # ---------- 右：折算到「每一轮循环」的占用 ----------
    ax2 = axes[1]
    ax2.set_facecolor(C_BG)
    per_loop = [b * r / LOOP_RATE for b, r in zip(buds, rates)]
    order = np.argsort(per_loop)
    yy = np.arange(len(TASKS))
    ax2.barh(yy, [per_loop[i] for i in order],
             color=[cols[i] for i in order], edgecolor="#333333",
             linewidth=0.8, height=0.62, zorder=2)
    ax2.set_yticks(yy)
    ax2.set_yticklabels([names[i] for i in order], fontsize=8.6)
    ax2.invert_yaxis()
    ax2.set_xlabel(_t("每轮循环的期望占用 = max_time_micros × rate_hz / 400"
                      "（微秒）", "us per loop"), fontsize=9.4)
    ax2.grid(axis="x", ls=":", color="#bbbbbb", zorder=0)
    for k, i in enumerate(order):
        ax2.text(per_loop[i] + 6, k, "%.0f us" % per_loop[i], va="center",
                 fontsize=8.0, color="#333333")
    tot2 = sum(per_loop)
    ax2.axvline(LOOP_US, color="#2b7a3d", ls="--", lw=1.4, zorder=1)
    ax2.set_xlim(0, 780)
    ax2.set_title(_t("图 6-2-1b  折算到每一轮的占用上界\n"
                     "★ 合计 %.0f us / %d us = %.0f%% —— 余量只有 %.0f%%"
                     % (tot2, LOOP_US, tot2 / LOOP_US * 100,
                        (1 - tot2 / LOOP_US) * 100),
                     "Fig 6-2-1b  us per loop"),
                  fontsize=10.6, weight="bold", pad=10)
    ax2.annotate(_t("★ GCS::update_send 一项就吃掉每轮 550 us\n"
                    "（全表最大，是第二名 AP_Logger::periodic_tasks\n"
                    " 的 1.8 倍）→ 它必然被频繁跳过（§22.2.1）",
                    "GCS::update_send dominates"),
                 xy=(545, list(order).index(8)), xytext=(150, 6.5),
                 fontsize=8.4, color="#8a2b2b",
                 arrowprops=dict(arrowstyle="-|>", color="#8a2b2b", lw=1.3))

    fig.suptitle(_t("★ 看图说话：这不是「实测耗时」，是【程序员在表里写下的承诺】。"
                    "预算之和刚好塞满一个循环周期 —— 所以「能用一个循环」靠的是"
                    "「每个任务都被切得足够小」，不是调度器聪明（§22.5.3）",
                    "budgets fill exactly one loop"),
                 fontsize=9.6, color="#5a3d00", y=0.035)
    fig.tight_layout(rect=[0, 0.055, 1, 1])
    p = os.path.join(HERE, "fig_ap_tasktable.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 02 课 · 图 2：主循环一轮的调用顺序与预算扣减
# ===========================================================================

def fig_ap_mainloop():
    fig, axes = plt.subplots(2, 1, figsize=(15.2, 10.4),
                             gridspec_kw=dict(height_ratios=[0.86, 1.4]))
    fig.patch.set_facecolor("white")

    # ================= 上：调用链 =================
    ax = axes[0]
    ax.set_facecolor(C_BG)
    chain = [
        ("main_loop()", "HAL_ChibiOS_Class.cpp:207", "#fde3e3"),
        (_t("g_callbacks->loop()", "g_callbacks->loop()"),
         "HAL_ChibiOS_Class.cpp:307", "#fde3e3"),
        ("AP_Vehicle::loop()", "AP_Vehicle.cpp:455", "#dbe9f7"),
        ("AP_Scheduler::loop()", "AP_Scheduler.cpp:339", "#fff3d6"),
        ("AP_Scheduler::run()", "AP_Scheduler.cpp:182", "#fff3d6"),
        (_t("task.function()  ← 你的代码在这里", "task.function()"),
         "AP_Scheduler.cpp:265", "#e3f0e3"),
    ]
    w, gap = 14.0, 2.2
    x0 = (100.0 - (len(chain) * w + (len(chain) - 1) * gap)) / 2.0
    for i, (nm, loc, fc) in enumerate(chain):
        x = x0 + i * (w + gap)
        box(ax, x, 50.0, w, 22.0, "%s\n%s" % (nm, loc), fc=fc,
            ec="#555555", fs=8.2)
        if i:
            arrow(ax, (x - gap, 61.0), (x, 61.0), color="#555555", lw=1.4)
    tag(ax, 50, 80.5, _t("★ 从 main_loop() 到你的任务函数，只有 4 层 —— "
                         "ArduPilot 的层级【少而直】；PX4 要多穿 2 个线程边界",
                         "only 4 layers"),
        fs=9.6, color="#1a1a1a", weight="bold")

    box(ax, 8, 24, 84, 20,
        _t("对比 PX4：px4 主程序 → 模块 start → 模块自己的线程/队列项 → Run()\n"
           "★ 差别不是「谁更高级」，而是「数据要穿过几个线程边界」"
           "（穿一次 = 一次拷贝 + 一次调度延迟）\n\n"
           "非抢占：没有任何东西能打断一个正在跑的任务。\n"
           "max_time_micros 在执行期的作用【只有一条】—— 准入控制：\n"
           "    预算 > 剩余时间  →  【不启动】本轮跳过（不是跑一半被打断）",
           "PX4 crosses thread boundaries; budget gates start, never interrupts"),
        fc="#fdf3d0", ec="#c99a2e", fs=8.6, tc="#5a3d00")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    # ================= 下：一轮循环的调用顺序（慢任务实际 = 声明 × 1.6，示意） =================
    ax2 = axes[1]
    ax2.set_facecolor(C_BG)

    OVER = 1.6          # ★ 示意假设：实际耗时 = 声明预算 × 1.6
    FRAME = 260.0       # ★ 示意：①②③ 三件调度器固定开销
    avail = LOOP_US - FRAME
    cur = FRAME
    bars = [dict(name=_t("①②③ 调度器帧开销", "frame overhead"), x=0.0,
                 w=FRAME, bud=FRAME, skip=False, fast=False, prio=-1)]
    for (nm, rate, bud, prio) in sorted(TASKS, key=lambda z: z[3]):
        fast = prio <= 3
        act = bud * OVER
        # 【B4】准入控制：判的是【声明预算】与【剩余时间】
        skip = (not fast) and (bud > avail + 1e-9)
        bars.append(dict(name=nm, x=cur, w=act, bud=bud, act=act, skip=skip,
                         fast=fast, prio=prio, rate=rate))
        if not skip:
            cur += act
            avail = 0.0 if act >= avail else avail - act
    total = cur

    n = len(bars)
    for i, b in enumerate(bars):
        y = n - i
        col = ("#c9c9c9" if i == 0 else
               ("#f6c9c9" if b["fast"] else "#cfe0f5"))
        if b["skip"]:
            col = "#f0f0f0"
        ax2.barh([y], [b["w"]], left=b["x"], height=0.62, color=col,
                 edgecolor="#777777", linewidth=0.8, zorder=2)
        if i == 0:
            ax2.text(b["x"] + b["w"] + 22, y, _t("固定开销（示意）", "frame"),
                     va="center", fontsize=7.8, color="#555555")
        elif b["fast"]:
            ax2.text(b["x"] + b["w"] + 22, y,
                     _t("★ 快任务（prio %d）：预算 = 2500 us" % b["prio"],
                        "fast"), va="center", fontsize=7.8, color="#b02b2b",
                     weight="bold")
        else:
            ax2.text(b["x"] + b["w"] + 22, y,
                     _t("声明 %d us（prio %d）" % (b["bud"], b["prio"]),
                        "%d us" % b["bud"]), va="center", fontsize=7.8,
                     color="#555555")
        if b["skip"]:
            ax2.text(b["x"] + b["w"] / 2.0, y, "×", va="center",
                     ha="center", fontsize=10.0, color="#b02b2b",
                     weight="bold", zorder=6)

    # 每轮循环的边界
    for xv, col, lab, ha in (
            (LOOP_US, "#2b7a3d", _t("本轮 2500 us 到点", "loop 2500 us"),
             "right"),
            (total, "#b02b2b",
             _t("★ 实际跑到 %.0f us —— 超了 %.0f us" % (total, total - LOOP_US),
                "actual %.0f us" % total), "left")):
        ax2.axvline(xv, color=col, ls="--", lw=1.5, zorder=3)
        ax2.text(xv + (-40 if ha == "right" else 40), n + 0.7, lab,
                 fontsize=8.6, color=col, weight="bold", va="center", ha=ha)

    # 被【B4】挡在门外的那两项（一条注解统管两行，避免压在条形上）
    ax2.annotate(_t("★ 排在最后的两个日志任务被【B4】挡在门外：\n"
                    "声明预算 350/300 us > 剩余时间 → 【不启动】（这一轮没有日志）",
                    "logger tasks skipped by B4"),
                 xy=(2950, 2.2), xytext=(2020, 8.6), fontsize=8.2,
                 color="#b02b2b", ha="left",
                 arrowprops=dict(arrowstyle="-|>", color="#b02b2b", lw=1.2))

    # 三个关键点的剩余预算
    for (k, txt, tx, ty) in (
            (1, _t("★ 剩 2032 us", "left 2032"), 1000, 9.5),
            (8, _t("★ 剩 704 us", "left 704"), 2450, 4.3),
            (9, _t("★ 550 ≤ 704 → 启动；实际 880 us → time_available 归零",
                   "budget -> 0"), 1180, 0.45)):
        b = bars[k]
        ax2.annotate(txt, xy=(b["x"] + b["w"], n - k), xytext=(tx, ty),
                     fontsize=8.0, color="#1f4e79",
                     arrowprops=dict(arrowstyle="-|>", color="#1f4e79",
                                     lw=1.0))

    ax2.set_yticks([n - i for i in range(n)])
    ax2.set_yticklabels([b["name"] for b in bars], fontsize=8.2)
    ax2.set_ylim(0.2, n + 1.4)
    ax2.set_xlim(-60, 4500)
    ax2.set_xlabel(_t("循环内的微秒数（本轮 = 2500 us @ 400 Hz）", "us"),
                   fontsize=9.4)
    ax2.grid(axis="x", ls=":", color="#c8c8c8", zorder=0)
    ax2.set_title(_t("图 6-2-2b  一轮循环内的真实调用顺序与预算扣减"
                     "（★ 示意：假设慢任务实际耗时 = 声明预算 × 1.6）\n"
                     "★ 快任务（prio ≤ 3）排在最前，预算 = 一整个循环周期；"
                     "慢任务按 priority 顺序扣，扣到不够就【不启动】",
                     "Fig 6-2-2b  one loop: order + budget"),
                  fontsize=10.6, weight="bold", pad=10)

    fig.suptitle(_t("★ 看图说话：一轮里两件事同时发生 —— ①排在最后的两个日志任务"
                    "被【B4】跳过（这一轮没有日志）；"
                    "②用时 %.0f us 超过 2500 us，overrun 记账 + 自动降速闭环"
                    "（§22.2.4）接管" % total,
                    "two things happen in one loop"),
                 fontsize=9.4, color="#5a3d00", y=0.038)
    fig.tight_layout(rect=[0, 0.055, 1, 1])
    p = os.path.join(HERE, "fig_ap_mainloop.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 02 课 · 图 3：PX4 vs ArduPilot 同一条控制路径
# ===========================================================================

def fig_arch_compare():
    fig, ax = new_canvas(15.4, 8.8)
    title(ax, _t("图 6-2-3  同一条「陀螺仪采样 → 电机输出」路径，两套架构各走了什么",
                 "Fig 6-2-3  same path, two architectures"),
          _t("★ 图里每个彩色小块都是【一次数据交接】：PX4 有 5 次（其中 3 次跨线程），"
             "ArduPilot 有 0 次（全在一个线程里的普通函数调用）",
             "each colored block is one data hand-over"))

    # ---------- 上：PX4 ----------
    box(ax, 2, 63, 96, 25, "", fc="#f4f8fd", ec="#4a7fb5", lw=1.4)
    tag(ax, 4.5, 85.5, _t("PX4：模块 + 消息总线", "PX4: modules + uORB bus"),
        fs=10.0, color="#1f4e79", weight="bold", ha="left")
    px4 = [
        (_t("sensors 线程\nwq:SPI0", "sensors"), "#e05252"),
        (_t("uORB\n拷贝", "uORB\ncopy"), "#f0a53a"),
        (_t("ekf2\nwq:INS0", "ekf2"), "#e05252"),
        (_t("uORB\n拷贝", "uORB\ncopy"), "#f0a53a"),
        (_t("rate_ctrl\n队列项", "rate_ctrl"), "#e05252"),
        (_t("uORB\n拷贝", "uORB\ncopy"), "#f0a53a"),
        (_t("pwm_out\n驱动", "pwm_out"), "#5b8fd6"),
        (_t("电机", "motor"), "#8a8a8a"),
    ]
    w = 10.4
    for i, (nm, col) in enumerate(px4):
        x = 4.0 + i * 11.9
        box(ax, x, 68.5, w, 11.0, nm, fc=col if col != "#8a8a8a" else "#e8e8e8",
            ec="#333333", fs=7.8)
        if i:
            arrow(ax, (x - 1.5, 74.0), (x, 74.0), color="#555555", lw=1.3)
    for i in (1, 3, 5):
        tag(ax, 4.0 + i * 11.9 + w / 2, 66.0,
            _t("跨线程", "cross\nthread"), fs=7.4, color="#b02b2b",
            weight="bold")
    tag(ax, 4.5, 62.5,
        _t("抢占式调度：高优先级【随时】能抢到 CPU（响应快），"
           "但任务可能在任意指令处被打断 → 共享数据要加锁，"
           "「这个函数要跑多久」无法单独分析",
           "preemptive: fast response, lock needed, hard to analyze"),
        fs=8.4, color="#8a2b2b", ha="left")

    # ---------- 下：ArduPilot ----------
    box(ax, 2, 22, 96, 26, "", fc="#f4fbf5", ec="#4a9a5b", lw=1.4)
    tag(ax, 4.5, 46.5,
        _t("ArduPilot：一个主循环线程 + 一张任务表", "ArduPilot: one loop + table"),
        fs=10.0, color="#1f5e33", weight="bold", ha="left")
    ap = [
        (_t("wait_for_sample()\n等 IMU 新样本", "wait_for_sample"), "#e05252"),
        (_t("ins.update()\n读陀螺仪", "ins.update()"), "#5b8fd6"),
        (_t("run_rate_controller()\n角速率环", "rate ctrl"), "#f0a53a"),
        (_t("motors_output()\n写电机", "motors_output()"), "#5b8fd6"),
    ]
    w2 = 20.0
    for i, (nm, col) in enumerate(ap):
        x = 5.0 + i * 23.0
        box(ax, x, 27.0, w2, 12.0, nm, fc=col, ec="#333333", fs=8.0)
        if i:
            arrow(ax, (x - 1.6, 33.0), (x, 33.0), color="#555555", lw=1.3)
    tag(ax, 50, 24.4,
        _t("★ 同一个线程、同一次循环、普通函数调用 —— 没有拷贝、没有队列、"
           "没有调度延迟（相位天然对齐）",
           "same thread, plain calls, no copy, no queue"),
        fs=8.6, color="#1f5e33", weight="bold")

    # ---------- 表格 ----------
    rows = [
        (_t("调度单位", "unit"), _t("线程 / 工作队列项", "thread / wq item"),
         _t("任务表里的一行", "a row in the table")),
        (_t("调度器", "scheduler"), _t("RTOS 抢占 + 队列 FIFO", "preemptive"),
         _t("一个非抢占的 for 循环", "non-preemptive for loop")),
        (_t("优先级语义", "priority"), _t("CPU 优先级（可抢占）", "CPU prio"),
         _t("顺序键（只决定先调用谁）", "ordering key")),
        (_t("慢任务被拖延", "delay"), _t("跑一半被打断", "interrupted"),
         _t("不启动（启动了就跑完）", "not started")),
        (_t("超时行为", "overrun"), _t("抢占 + hrt 驱动", "preempt"),
         _t("只记账 + 自动降速闭环", "accounting only")),
        (_t("时序可预测性", "predictability"), _t("弱（取决于负载）", "weak"),
         _t("强（顺序固定、无抢占）", "strong")),
    ]
    y0 = 18.0
    ax.text(20, y0, _t("维度", "dim"), fontsize=8.6, weight="bold",
            ha="center")
    ax.text(50, y0, "PX4", fontsize=8.6, weight="bold", ha="center")
    ax.text(80, y0, "ArduPilot", fontsize=8.6, weight="bold", ha="center")
    for i, (a, b, c) in enumerate(rows):
        yy = y0 - 2.6 - i * 2.35
        if i % 2 == 0:
            band(ax, 3, yy - 1.05, 94, 2.1, fc="#ffffff", ec="#ffffff")
        ax.text(20, yy, a, fontsize=8.2, ha="center")
        ax.text(50, yy, b, fontsize=8.2, ha="center", color="#8a2b2b")
        ax.text(80, yy, c, fontsize=8.2, ha="center", color="#1f5e33")

    tag(ax, 50, 1.6,
        _t("★ 看图说话：第 4、5 两行连起来读 —— PX4 的「被打断」和 "
           "ArduPilot 的「不启动」是两种完全不同的失败方式：\n"
           "前者让你无法给单个函数定时，后者让你必须把每个函数切得足够小。",
           "interrupted vs not-started"), fs=8.8, color="#5a3d00")

    fig.tight_layout()
    p = os.path.join(HERE, "fig_arch_compare.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 03 课 · 图 1：BSP 各层关系与两条构建路径
# ===========================================================================

def fig_bsp_layers():
    fig, ax = new_canvas(15.4, 9.6)
    title(ax, _t("图 6-3-1  BSP 的位置：一份固件凭什么跑几十块板子",
                 "Fig 6-3-1  where the BSP sits"),
          _t("★ 注意两条平行箭头：同一份 default.px4board【同时】喂给"
             "「代码里的 #ifdef」和「构建系统编哪些目录」两条路（§22.5.2.2）",
             "one config file, two build paths"))

    # ---- 第 1 层：上层 ----
    box(ax, 3, 78.0, 94, 11.0,
        _t("上层（与板子无关，一行都不用改）\n"
           "src/modules/  ·  src/drivers/  ·  src/lib/",
           "upper layers: modules / drivers / lib"),
        fc="#e8f1e9", ec="#4a9a5b", fs=9.2)
    # ---- 第 2 层：三个抽象 ----
    box(ax, 3, 68.0, 94, 7.2,
        _t("它只认三个抽象：一条 SPI 总线 + 一个片选 + 一个数据就绪中断",
           "only 3 abstractions: bus / CS / DRDY"),
        fc="#f4fbf5", ec="#8fbf9a", fs=8.6)
    # ---- 第 3 层：板级代码 / 启动脚本 ----
    box(ax, 3, 53.5, 45, 12.0,
        _t("板级 C/C++ 代码  boards/px4/<型号>/src/\n\n"
           "board_config.h   引脚宏 + 能力开关 + HW 版本\n"
           "spi.cpp          七个 HW 版本各自的 SPI 表\n"
           "i2c.cpp / mtd.cpp / hw_config.h\n"
           "→ 编译成普通静态库 drivers_board",
           "board C/C++ code"),
        fc="#fff3d6", ec="#c99a2e", fs=8.2)
    box(ax, 52, 53.5, 45, 12.0,
        _t("启动脚本  boards/px4/<型号>/init/\n\n"
           "rc.board_sensors   挂哪些传感器\n"
           "rc.board_defaults  参数默认值\n"
           "rc.board_mavlink   哪个口跑 MAVLink\n"
           "→ 烧进 ROMFS，开机按顺序执行",
           "startup scripts"),
        fc="#fde3e3", ec="#b02b2b", fs=8.2)
    # ---- 第 4 层：两个配置源 ----
    box(ax, 3, 40.0, 45, 10.5,
        _t("配置源 A：default.px4board（Kconfig，98 行）\n"
           "CONFIG_BOARD_* / CONFIG_DRIVERS_* / CONFIG_MODULES_*",
           "config source A: default.px4board"),
        fc="#dbe9f7", ec="#4a7fb5", fs=8.2)
    box(ax, 52, 40.0, 45, 10.5,
        _t("配置源 B：nuttx-config/nsh/defconfig\n"
           "STM32H743 有几路 UART/SPI、Flash 怎么切",
           "config source B: nuttx defconfig"),
        fc="#e6e0f2", ec="#7a5fb5", fs=8.2)
    # ---- 第 5 层：两条路 ----
    box(ax, 3, 26.0, 45, 10.5,
        _t("路①  →  px4_boardconfig.h\n生成 C 头文件，给代码内部 #ifdef 用\n"
           "kconfig.cmake:68",
           "path 1: -> px4_boardconfig.h"),
        fc="#dff0e2", ec="#4a9a5b", fs=8.2)
    box(ax, 52, 26.0, 45, 10.5,
        _t("路②  →  config_module_list → add_subdirectory\n"
           "CONFIG_MODULES_MAVLINK=y → src/modules/mavlink\n"
           "kconfig.cmake:102/124/177 → CMakeLists.txt:433",
           "path 2: -> module list"),
        fc="#dff0e2", ec="#4a9a5b", fs=8.2)
    # ---- 第 6 层：运行期产物 ----
    box(ax, 3, 13.0, 94, 9.5,
        _t("运行期产物（不会出现在源码树里）\n"
           "px4_boardconfig.h · /etc/init.d/rc.serial（Tools/serial/generate_config.py 生成）\n"
           "SER_TEL1_BAUD 等参数定义 · TEL1 成为 MAVLink 候选口",
           "runtime artifacts"),
        fc="#eef2f7", ec="#8899aa", fs=8.2)
    # ---- 第 7 层：三层差异 ----
    box(ax, 3, 1.5, 94, 9.5,
        _t("★ 三层差异，三种处理方式（改错一层，报错晚一截）\n"
           "芯片级（几路 UART/SPI/Flash）→ RTOS，改 nuttx defconfig　"
           "★ 改错：NuttX 编不过 / 整条总线全哑\n"
           "引脚级（片选 PI4、中断 PI6）→ BSP，改 board_config.h + spi.cpp　"
           "★ 改错：编译通过，运行时「设备不存在」\n"
           "产品级（CRSF 还是 SBUS、要不要光流）→ BSP+构建，改 default.px4board + init/rc.board_*　"
           "★ 改错：设备在，但功能不对",
           "three levels of difference"),
        fc="#fdf3d0", ec="#c99a2e", fs=8.0, tc="#5a3d00")

    # ---- 箭头：全部竖直，互不交叉 ----
    arrow(ax, (22, 65.2), (22, 68.0), color="#4a9a5b", lw=1.5)
    arrow(ax, (50, 75.2), (50, 78.0), color="#4a9a5b", lw=1.5)
    arrow(ax, (12, 50.5), (12, 53.5), color="#4a7fb5", lw=1.5)   # A → 板级代码
    arrow(ax, (30, 40.0), (30, 36.5), color="#4a9a5b", lw=1.5)   # A → 路①
    arrow(ax, (74, 40.0), (74, 36.5), color="#7a5fb5", lw=1.5)   # B → 路②
    arrow(ax, (25, 26.0), (25, 22.5), color="#8899aa", lw=1.5)   # 路① → 产物
    arrow(ax, (74, 26.0), (74, 22.5), color="#8899aa", lw=1.5)   # 路② → 产物
    arrow(ax, (50, 13.0), (50, 11.0), color="#8899aa", lw=1.5)   # 产物 → 三层差异

    fig.tight_layout()
    p = os.path.join(HERE, "fig_bsp_layers.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 03 课 · 图 2：角色 → /dev/ttyS* → UART 三层映射
# ===========================================================================

# 角色 ←→ 设备节点：逐字抄自 boards/px4/fmu-v6x-qgbf/default.px4board:3-7
SERIAL_ROLE = [
    (_t("GPS1", "GPS1"), "/dev/ttyS0"),
    (_t("GPS2", "GPS2"), "/dev/ttyS7"),
    (_t("TEL1", "TEL1"), "/dev/ttyS6"),
    (_t("TEL2", "TEL2"), "/dev/ttyS4"),
    (_t("RC（读者新增）", "RC (added)"), "/dev/ttyS5"),
]

# UART 外设 ←→ 引脚：取自同一块 PCB（Pixhawk6X / fmu-v6x）的
# libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X/hwdef.dat:47-78
UART_PINS = {
    "USART1": ("PB6", "PB7"), "USART2": ("PD5", "PA3"),
    "USART3": ("PD8", "PD9"), "UART4": ("PH13", "PH14"),
    "UART5": ("PC12", "PD2"), "USART6": ("PC6", "PC7"),
    "UART7": ("PE8", "PF6"), "UART8": ("PE1", "PE0"),
}

# 由源码确证的映射（唯一一条能在 PX4 侧直接读出来的）
#   boards/px4/fmu-v6x/src/board_config.h:61-70
#   PX4IO_SERIAL_DEVICE "/dev/ttyS5" + PX4IO_SERIAL_TX_GPIO GPIO_USART6_TX
VERIFIED = {"/dev/ttyS5": "USART6"}


def fig_bsp_serial_map():
    fig, ax = new_canvas(15.4, 8.6)
    title(ax, _t("图 6-3-2  一个串口的四层身份：角色 → 设备节点 → UART 外设 → 引脚",
                 "Fig 6-3-2  four identities of one serial port"),
          _t("★ 看第二栏：TEL1=ttyS6、TEL2=ttyS4、GPS2=ttyS7 —— "
             "编号【不是】按顺序发的，所以「这个口是哪个 ttyS」只能查配置，不能猜",
             "the ttyS numbers are not in order"))

    # 四栏
    band(ax, 2, 26.5, 22, 57.5)
    band(ax, 27, 26.5, 20, 57.5)
    band(ax, 50, 26.5, 20, 57.5)
    band(ax, 73, 26.5, 25, 57.5)
    tag(ax, 13, 86.0, _t("① 功能角色\n（你要什么）", "role"), fs=9.4,
        weight="bold", color="#1f4e79")
    tag(ax, 37, 86.0, _t("② 设备节点\n（代码 open() 谁）", "/dev/ttyS*"), fs=9.4,
        weight="bold", color="#7d4a00")
    tag(ax, 60, 86.0, _t("③ UART 外设\n（芯片内）", "UART peripheral"), fs=9.4,
        weight="bold", color="#1f4e79")
    tag(ax, 85.5, 86.0, _t("④ PCB 引脚\n（原理图上的）", "pins"), fs=9.4,
        weight="bold", color="#1f5e33")

    ytop = 76.0
    for i, (role, dev) in enumerate(SERIAL_ROLE):
        y = ytop - i * 12.0
        box(ax, 3.2, y, 19.6, 8.6, role, fc="#dbe9f7", ec="#4a7fb5", fs=9.2)
        box(ax, 28.2, y, 17.6, 8.6, dev, fc="#fff3d6", ec="#c99a2e", fs=9.2)
        arrow(ax, (22.8, y + 4.3), (28.2, y + 4.3), color="#555555", lw=1.4)

        uart = VERIFIED.get(dev)
        if uart:
            box(ax, 51.0, y, 18.0, 8.6, uart, fc="#dff0e2", ec="#4a9a5b",
                fs=9.2, weight="bold")
            tx, rx = UART_PINS[uart]
            box(ax, 74.0, y, 23.0, 8.6,
                _t("TX = %s   RX = %s", "TX=%s RX=%s") % (tx, rx),
                fc="#f0f7f1", ec="#8fbf9a", fs=8.8)
            arrow(ax, (69.0, y + 4.3), (74.0, y + 4.3), color="#4a9a5b", lw=1.5)
            tag(ax, 60.0, y - 1.6, _t("★ 源码可确证", "verified in source"),
                fs=7.4, color="#2b7a3d", weight="bold")
        else:
            box(ax, 51.0, y, 18.0, 8.6, _t("？（不是猜出来的）", "?"),
                fc="#f2f2f2", ec="#aaaaaa", fs=8.6, tc="#777777", ls="dashed")
            box(ax, 74.0, y, 23.0, 8.6,
                _t("由 NuttX 注册顺序决定\n（defconfig 里 8 路 UART 都开着）",
                   "set by NuttX registration order"),
                fc="#f7f7f7", ec="#bbbbbb", fs=7.8, tc="#666666", ls="dashed")

    # 说明框 ①
    box(ax, 2, 14.0, 95, 11.0, "", fc="#fdf3d0", ec="#c99a2e")
    ax.text(49.5, 19.5,
            _t("★ 看图说话：①→② 这一段【没有任何语义规律】—— 角色名里的 "
               "1/2 和 ttyS 的 0/7/6/4/5 毫无关系。\n"
               "   所以移植新板时的第 5 步（§22.5.7.1「串口角色」）必须逐行对着原理图写，"
               "写错了编译通过、运行时那个口就是死的。",
               "no semantic rule between role name and ttyS number"),
            fontsize=8.4, color="#5a3d00", ha="center", va="center",
            linespacing=1.6)

    # 说明框 ②
    box(ax, 2, 2.0, 95, 10.0, "", fc="#eef2f7", ec="#8899aa")
    ax.text(49.5, 7.0,
            _t("★ ③④ 两栏：PX4 侧不存在一张「引脚 → ttyS」的文本表 —— "
               "引脚在原理图里、ttyS 号由 NuttX 注册顺序决定；\n"
               "   而同款硬件的 ArduPilot 用一行 hwdef.dat 把这两件事写在一起"
               "（这就是 §22.5.8 说的「分离 vs 集中」）。",
               "PX4 has no pin<->ttyS table; AP has one in hwdef.dat"),
            fontsize=8.4, color="#1f4e79", ha="center", va="center",
            linespacing=1.6, weight="bold")

    fig.tight_layout()
    p = os.path.join(HERE, "fig_bsp_serial_map.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

GROUPS = {
    "l1": [fig_uorb_dataflow, fig_uorb_threads, fig_uorb_pubsub_timing],
    "l2": [fig_ap_tasktable, fig_ap_mainloop, fig_arch_compare],
    "l3": [fig_bsp_layers, fig_bsp_serial_map],
}


def main():
    if not _HAS_MPL:
        print("没装 matplotlib，跳过画图（脚本仍然正常退出）。")
        return 0

    print("第 6 章配图 · CJK 字体：%s" % ("已找到" if _CJK else
                                          "未找到，图里用英文（并已注明）"))

    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    want = args[0].lower() if args else "all"
    todo = []
    if want in ("all", ""):
        for k in ("l1", "l2", "l3"):
            todo += GROUPS[k]
    elif want in GROUPS:
        todo = GROUPS[want]
    else:
        print("用法：python figures_ch6.py [all|l1|l2|l3]")
        return 2

    for fn in todo:
        head(fn.__doc__ or fn.__name__)
        fn()
    print()
    print("  共 %d 张图，全部落在 %s" % (len(todo), HERE))
    return 0


if __name__ == "__main__":
    sys.exit(main())
