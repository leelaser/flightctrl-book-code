#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 7 章（仿真与实验）配图生成脚本 —— 6 张图，全部由课文里的数据生成。

为什么这一章最需要图
--------------------
第 7 章讲的是**数据怎么流动**（SITL 的闭环链路）和**时间**（延迟、日志时间轴）。
这两件事在纯文字里都要读者自己在脑子里搭模型：

  · 01 课的数据链路，课文里的 ASCII 图到「ekf2 订阅」就停了 ——
    **回路闭合的那一半（控制量怎么回到 Gazebo）没画**，读者以为数据是单向的。
  · FLU→FRD 那个 180° 旋转，三种写法（qx/qy/qz）症状完全不同，
    文字描述要看三遍才分得清，一张表就能一眼看完。
  · 延迟那 10 行相位裕度表，是「仿真里调好、真机上振荡」的定量解释。
  · 日志的两种格式（ULog / DataFlash）在「长度含不含头」上**恰好相反**，
    这是读二进制日志最常踩的坑。
  · 排查决策树的「顺序不能换」和「两条独立因果链」，
    是第 7 章最像侦探小说的部分，靠时间轴才看得清。

用法：
    python figures_ch7.py            # 生成全部 6 张
    python figures_ch7.py l1         # 只生成 01 课（SITL）的 3 张
    python figures_ch7.py l2         # 只生成 02 课（日志）的 3 张

输出（全部落在本目录下）：
    01 课   fig_sitl_datalink.png      闭环数据链路（含缺掉的那一半回路）
            fig_flu_frd.png            FLU→FRD 的 180° 旋转 + 三种写法对照
            fig_sitl_latency.png       真机额外延迟的来源 + 相位裕度扫描
    02 课   fig_log_formats.png        ULog vs DataFlash 字节布局对照
            fig_log_decision_tree.png  日志排查决策树（第 0~6 步）
            fig_log_evidence_chain.png 40 秒飞行的两条独立因果链

只依赖 numpy + matplotlib。缺 matplotlib 时打印提示并退出 0（不报错）。
"""

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
# 01 课 · 图 1：SITL 闭环数据链路（把课文 ASCII 图缺掉的那半圈补上）
# ===========================================================================

# 链路节点：(框里的字, 源码位置, 颜色)
LINK_ROW1 = [
    (_t("Gazebo 世界\nworld: default.sdf\nmodel: x500 → link: base_link\nsensor: imu_sensor",
        "Gazebo world"), "default.sdf", "#dbe9f7"),
    (_t("gz-transport\nworld/$WORLD/model/$MODEL/\nlink/base_link/sensor/\nimu_sensor/imu",
        "gz topic"), "gz topic 名字", "#e6e0f2"),
    (_t("gz_bridge::imuCallback()\n★ FLU → FRD 旋转\n四元数 (0,1,0,0)",
        "gz_bridge::imuCallback"), "GZBridge.cpp:450 / 464-470", "#fff3d6"),
    (_t("sensor_accel / sensor_gyro\n发布到 uORB（250 Hz）",
        "publish sensor_accel/gyro"), "GZBridge.cpp:487 / 510", "#fde3e3"),
]
LINK_ROW2 = [
    (_t("GZMixingInterfaceESC\n::updateOutputs()", "mixing interface"),
     "→ gz topic", "#fde3e3"),
    (_t("actuator_motors\n（uORB，q=1）", "actuator_motors"), "q = 1", "#fff3d6"),
    (_t("mc_rate_control / mc_att_control\n内环 → 外环", "rate / att control"),
     "wq:rate_ctrl", "#fff3d6"),
    (_t("VehicleIMU → ekf2\n订阅 uORB topic", "VehicleIMU -> ekf2"),
     "IMU_INTEG_RATE 250 Hz", "#e3f0e3"),
]


def fig_sitl_datalink():
    fig, ax = new_canvas(15.6, 9.2)
    title(ax, _t("图 7-1-1  SITL 的闭环数据链路：Gazebo 刚体 → uORB → 控制量 → 回到 Gazebo",
                 "Fig 7-1-1  the SITL closed loop"),
          _t("★ 课文里的链路图到「ekf2 订阅」就停了 —— 这张图把【回路闭合】的"
             "后半圈补上：控制量必须回到 Gazebo 的物理引擎，仿真才会动",
             "the second half of the loop"))

    # ---- 上半圈：Gazebo → PX4 ----
    band(ax, 2, 62, 96, 24, fc="#eaf2fb", ec="#9dbbdc")
    tag(ax, 3.5, 83.5, _t("上半圈：Gazebo 造数据 → PX4 收数据（传感器方向）",
                          "Gazebo -> PX4"), fs=9.4, color="#1f4e79",
        weight="bold", ha="left")
    w, gap = 22.0, 2.6
    for i, (txt, loc, fc) in enumerate(LINK_ROW1):
        x = 3.0 + i * (w + gap)
        box(ax, x, 65.0, w, 15.0, txt, fc=fc, ec="#4a7fb5", fs=7.8)
        tag(ax, x + w / 2, 63.0, loc, fs=7.0, color="#666666")
        if i:
            arrow(ax, (x - gap, 72.5), (x, 72.5), color="#2f6fb5", lw=1.6)

    # ---- 下半圈：PX4 → Gazebo ----
    band(ax, 2, 33, 96, 24, fc="#eef7ef", ec="#9dc4a6")
    tag(ax, 3.5, 54.5, _t("下半圈：PX4 算控制量 → Gazebo 被推动（执行方向）"
                          " —— 少了这半圈，仿真就是「只读不动的录像」",
                          "PX4 -> Gazebo"), fs=9.4, color="#1f5e33",
        weight="bold", ha="left")
    for i, (txt, loc, fc) in enumerate(LINK_ROW2):
        x = 3.0 + i * (w + gap)            # ekf2 在右（正对上面的 uORB 发布）
        box(ax, x, 36.0, w, 15.0, txt, fc=fc, ec="#4a9a5b", fs=7.8)
        tag(ax, x + w / 2, 34.0, loc, fs=7.0, color="#666666")
        if i == 3:
            arrow(ax, (x + w / 2, 65.0), (x + w / 2, 51.0),
                  color="#2f6fb5", lw=1.6)     # 上→下：跨进程边界
        if i:
            arrow(ax, (x, 43.5), (x - gap, 43.5), color="#4a9a5b", lw=1.6)

    # ---- 闭环回边 ----
    box(ax, 3.0, 22.0, 45.0, 9.0,
        _t("Gazebo 物理引擎：按电机转速积分刚体状态\n"
           "→ 下一个 IMU 采样点（250 Hz）",
           "Gazebo physics integrates the body"),
        fc="#e6e0f2", ec="#7a5fb5", fs=8.2)
    arrow(ax, (14.0, 36.0), (14.0, 31.0), color="#7a5fb5", lw=1.6)
    ax.plot([14.0, 14.0, 0.9, 0.9], [22.0, 18.0, 18.0, 78.0],
            color="#7a5fb5", lw=1.8, zorder=4)
    arrow(ax, (0.9, 78.0), (2.8, 78.0), color="#7a5fb5", lw=1.8)
    tag(ax, 16.5, 19.6,
        _t("★ 这条回边就是「闭环」：仿真里飞机能悬停，是因为这圈每秒转 250 次",
           "this edge closes the loop"), fs=8.4, color="#5a2b7a",
        weight="bold", ha="left")

    box(ax, 52.0, 12.0, 46.0, 19.0,
        _t("★ 看图说话：整条链路【没有任何一环是「可选的」】。\n\n"
           "断在 ① imuCallback 不转 FLU→FRD → 姿态估计把「上」当成「下」\n"
           "断在 ② uORB 没发布 → ekf2 收不到新样本，解锁都过不了\n"
           "断在 ③ 回边（电机转速没回 Gazebo）→ 飞机在 Gazebo 里【一动不动】，\n"
           "        而 PX4 侧日志看起来「一切正常」—— 这是最难查的一种 SITL 故障",
           "every link is mandatory"),
        fc="#fdf3d0", ec="#c99a2e", fs=8.2, tc="#5a3d00")

    tag(ax, 50, 7.0,
        _t("★ 与课文 ASCII 图的差别：课文停在「VehicleIMU / ekf2 订阅」；"
           "本图把 actuator_motors → GZMixingInterfaceESC → gz /x500/command/motor_speed "
           "→ 物理引擎 → 回到 IMU 采样这一段补全。",
           "this figure completes the loop"), fs=8.4, color="#1f4e79",
        weight="bold")
    tag(ax, 50, 3.0,
        _t("★ 启动脚本把这些接起来：ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator:117 "
           "拉起 gz_bridge，:5 设 IMU_INTEG_RATE=250。",
           "px4-rc.simulator:117 / :5"), fs=8.0, color="#555555")

    fig.tight_layout()
    p = os.path.join(HERE, "fig_sitl_datalink.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 01 课 · 图 2：FLU → FRD 的 180° 旋转（含三种写法的症状对照）
# ===========================================================================

def _triad(ax, cx, cy, s, names, cols, title_txt, subtitle):
    """画一个二维投影的三轴坐标系。names/cols 各 3 个，顺序 (x, y, z)。"""
    dirs = [(0.80, 0.60), (-0.80, 0.60), (0.0, 1.0)]   # 前-右上 / 左-左上 / 上
    for (dx, dy), nm, col in zip(dirs, names, cols):
        arrow(ax, (cx, cy), (cx + dx * s, cy + dy * s), color=col, lw=2.2,
              ms=12)
        tag(ax, cx + dx * s * 1.22, cy + dy * s * 1.22, nm, fs=10.5,
            color=col, weight="bold")
    ax.add_patch(plt.Circle((cx, cy), 1.1, color="#333333", zorder=5))
    tag(ax, cx, cy - 6.5, title_txt, fs=9.6, weight="bold", color="#1a1a1a")
    tag(ax, cx, cy - 10.5, subtitle, fs=8.2, color="#666666")


def fig_flu_frd():
    fig, ax = new_canvas(15.6, 9.0)
    title(ax, _t("图 7-1-2  FLU → FRD：那个 180 度旋转到底把什么变了号",
                 "Fig 7-1-2  the FLU -> FRD 180 deg rotation"),
          _t("★ 看中间那列：X 不变，Y 和 Z 同时变号 —— 这等价于「绕 X 轴转 180°」，"
             "四元数就是 (0,1,0,0)（GZBridge.cpp:464-470）",
             "X unchanged, Y and Z negated"))

    # ---- 左：两个坐标系 ----
    band(ax, 2, 40, 30, 48, fc="#eaf2fb", ec="#9dbbdc")
    tag(ax, 17, 85.0, _t("Gazebo / ROS 的机体系", "Gazebo body frame"),
        fs=9.4, weight="bold", color="#1f4e79")
    tag(ax, 17, 81.0, _t("FLU：x 前、y 左、z 上", "FLU"), fs=8.6,
        color="#1f4e79")
    _triad(ax, 17.0, 62.0, 13.0, ["x", "y", "z"],
           ["#2b7a3d", "#b02b2b", "#2f6fb5"],
           _t("FLU", "FLU"), _t("右手系：x × y = z", "right-handed"))

    band(ax, 34, 40, 30, 48, fc="#f4fbf5", ec="#9dc4a6")
    tag(ax, 49, 85.0, _t("PX4 的机体系", "PX4 body frame"), fs=9.4,
        weight="bold", color="#1f5e33")
    tag(ax, 49, 81.0, _t("FRD：x 前、y 右、z 下", "FRD"), fs=8.6,
        color="#1f5e33")
    _triad(ax, 49.0, 62.0, 13.0, ["x", "-y", "-z"],
           ["#2b7a3d", "#b02b2b", "#2f6fb5"],
           _t("FRD", "FRD"),
           _t("同一物理方向：在 FRD 里读作 -y / -z", "same physics, negated"))

    # ---- 中：映射表 ----
    band(ax, 66, 40, 32, 48, fc="#fffbf0", ec="#c99a2e")
    tag(ax, 82, 85.0, _t("同一个向量，两边读出来", "the same vector, two frames"),
        fs=9.2, weight="bold", color="#7d4a00")
    rows = [("x", "x", _t("不变", "same")),
            ("y", "-y", _t("变号", "negated")),
            ("z", "-z", _t("变号", "negated"))]
    tag(ax, 73.0, 79.5, _t("FLU", "FLU"), fs=9.0, weight="bold", color="#1f4e79")
    tag(ax, 91.0, 79.5, _t("FRD", "FRD"), fs=9.0, weight="bold",
        color="#1f5e33")
    for i, (a, b, c) in enumerate(rows):
        y = 73.5 - i * 6.0
        box(ax, 68.0, y, 10.0, 4.6, a, fc="#dbe9f7", ec="#4a7fb5", fs=9.0)
        arrow(ax, (78.6, y + 2.3), (83.4, y + 2.3), color="#888888", lw=1.3)
        box(ax, 84.0, y, 12.0, 4.6, b, fc="#dff0e2", ec="#4a9a5b", fs=9.0,
            weight="bold")
    box(ax, 68.0, 52.0, 28.0, 6.4,
        _t("等价于：绕 X 轴转 180°\n四元数 (w,x,y,z) = (0, 1, 0, 0)",
           "(w,x,y,z) = (0,1,0,0)"), fc="#fdf3d0", ec="#c99a2e", fs=8.2,
        tc="#5a3d00")

    # ---- 下：三种写法对照（练习 2(b) 的表）----
    box(ax, 2, 20.0, 96.0, 16.0, "", fc="#ffffff", ec="#aaaaaa")
    tag(ax, 50, 34.2, _t("★ 三种写法都能编译通过 —— 症状完全不同（练习 2(b)）",
                         "three spellings, three symptoms"), fs=9.2,
        weight="bold", color="#b02b2b")
    cols = [(_t("写法", "spelling"), 12.0), (_t("实际旋转", "rotation"), 20.0),
            (_t("映射", "mapping"), 30.0), (_t("症状", "symptom"), 56.0)]
    for lab, x in cols:
        tag(ax, x, 31.3, lab, fs=8.6, weight="bold", color="#333333")
    rows2 = [
        ("Quaterniond(0,1,0,0)", _t("绕 X 180°", "about X"),
         "(x, -y, -z)", _t("★ 正确：姿态估计与真机一致", "correct"), "#2b7a3d"),
        ("Quaterniond(0,0,0,1)", _t("绕 Z 180°", "about Z"),
         "(-x, -y, +z)",
         _t("前进方向反了：地面测试看不出来，起飞就翻；偏航速率环正反馈",
            "forward/backward reversed"), "#b02b2b"),
        ("Quaterniond(0,0,1,0)", _t("绕 Y 180°", "about Y"),
         "(-x, +y, -z)",
         _t("z 反号：悬停读出 +9.81 → 飞控认为倒扣 → 拒绝解锁",
            "z negated -> refuses to arm"), "#b02b2b"),
    ]
    for i, (sp, rot, mp, sym, col) in enumerate(rows2):
        y = 28.2 - i * 3.2
        tag(ax, 12.0, y, sp, fs=8.0, color="#333333")
        tag(ax, 20.0, y, rot, fs=8.0, color="#555555")
        tag(ax, 30.0, y, mp, fs=8.4, color="#1f4e79", weight="bold")
        tag(ax, 56.0, y, sym, fs=8.0, color=col, weight="bold")

    box(ax, 2, 8.0, 96.0, 11.0,
        _t("★ 看图说话：`1` 落在第几个参数上，就绕第几根轴转 180°"
           "（gz::math::Quaterniond 的参数顺序是 (w,x,y,z)，"
           "证据在 GZBridge.cpp:539-543 与 :756）。\n"
           "   记忆法：(0,0,0,1) 里的 1 在【最后】→ 对应最后一根轴 z。"
           "编译器和类型系统对这三种写法【完全不设防】。",
           "the position of the 1 decides the axis"),
        fc="#fdf3d0", ec="#c99a2e", fs=8.4, tc="#5a3d00")
    tag(ax, 50, 4.0,
        _t("★ 为什么必须转：Gazebo 给的是 FLU（z 上），PX4 控制器认的是 FRD（z 下）；"
           "不转的话，加速度计的 z 符号是反的。",
           "Gazebo is FLU, PX4 is FRD"), fs=8.4, color="#1f4e79",
        weight="bold")

    fig.tight_layout()
    p = os.path.join(HERE, "fig_flu_frd.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 01 课 · 图 3：真机额外延迟的来源与相位裕度扫描
# ===========================================================================

# 逐字抄自课文 §23.4.3 的扫描表（$T_{extra}$ 是【额外】延迟，不含 T0 = 8 ms）
PM_TABLE = [
    (0.0, 57.0, 2.97), (2.0, 51.8, 2.43), (4.0, 46.6, 2.08),
    (6.0, 41.4, 1.82), (8.0, 36.2, 1.63), (10.0, 31.0, 1.48),
    (15.0, 18.0, 1.22), (20.0, 5.0, 1.05), (25.0, -8.0, 0.93),
    (30.0, -21.0, 0.85),
]
T0_MS = 8.0            # 仿真里就有的采样 + 计算延迟
DELAY_SRC = [          # 真机上 T_extra 的三个来源（课文 §23.4.3 表）
    (_t("传感器采样 + 低通滤波（群延迟）", "sensor sampling + LPF"), 2.0, 6.0,
     "#5b8fd6"),
    (_t("数据传输（uORB / 总线）", "data transport (uORB/bus)"), 1.0, 3.0,
     "#f0a53a"),
    (_t("电机 + 桨的响应", "motor + propeller response"), 3.0, 12.0,
     "#e05252"),
]


def fig_sitl_latency():
    fig, axes = plt.subplots(1, 2, figsize=(15.4, 7.6),
                             gridspec_kw=dict(width_ratios=[1.0, 1.15]))
    fig.patch.set_facecolor("white")

    # ================= 左：延迟的来源 =================
    ax = axes[0]
    ax.set_facecolor(C_BG)
    y = np.arange(len(DELAY_SRC) + 1)
    for i, (nm, lo, hi, col) in enumerate(DELAY_SRC):
        ax.barh([i], [hi - lo], left=lo, height=0.55, color=col,
                edgecolor="#333333", linewidth=0.9, zorder=3)
        ax.text(hi + 0.35, i, "%s ~ %s ms" % (_g(lo), _g(hi)), va="center",
                fontsize=9.0, color="#333333")
    lo_sum = sum(s[1] for s in DELAY_SRC)
    hi_sum = sum(s[2] for s in DELAY_SRC)
    ax.barh([len(DELAY_SRC)], [hi_sum - lo_sum], left=lo_sum, height=0.55,
            color="#8a2b2b", edgecolor="#333333", linewidth=0.9, zorder=3)
    ax.text(hi_sum + 0.35, len(DELAY_SRC),
            _t("合计 %s ~ %s ms", "%s ~ %s ms") % (_g(lo_sum), _g(hi_sum)),
            va="center", fontsize=9.0, color="#8a2b2b", weight="bold")
    ax.set_yticks(y)
    ax.set_yticklabels([s[0] for s in DELAY_SRC] +
                       [_t("★ 真机的 T_extra", "real T_extra")], fontsize=8.8)
    ax.invert_yaxis()
    ax.set_xlim(0, 24)
    ax.set_xlabel(_t("毫秒（400 Hz 循环的一个周期 = 2.5 ms）", "ms"), fontsize=9.4)
    ax.grid(axis="x", ls=":", color="#bbbbbb", zorder=0)
    ax.set_ylim(len(DELAY_SRC) + 0.95, -0.75)
    ax.axvline(2.5, color="#2b7a3d", ls="--", lw=1.3)
    ax.text(2.75, len(DELAY_SRC) + 0.5,
            _t("1 个循环周期 = 2.5 ms　★ 6~20 ms 相当于滞后 2~8 个周期",
               "1 loop = 2.5 ms"), fontsize=8.4, color="#2b7a3d")
    ax.set_title(_t("图 7-1-3a  真机上 T_extra 的三个来源（课文 §23.4.3 表）",
                    "Fig 7-1-3a  where T_extra comes from"),
                 fontsize=10.8, weight="bold", pad=10)

    # ================= 右：相位裕度扫描 =================
    ax2 = axes[1]
    ax2.set_facecolor(C_BG)
    xs = [p[0] for p in PM_TABLE]
    pms = [p[1] for p in PM_TABLE]
    ax2.plot(xs, pms, "-o", color="#8a2b2b", lw=1.8, ms=5.0, zorder=4,
             label=_t("相位裕度（度）", "phase margin"))
    ax2.axhline(45.0, color="#2b7a3d", ls="--", lw=1.3, zorder=2)
    ax2.text(0.4, 46.5, _t("45°：工程上的「够用」线", "45 deg"), fontsize=8.4,
             color="#2b7a3d")
    ax2.axhline(0.0, color="#333333", lw=1.0, zorder=2)
    ax2.axvspan(6.0, 20.0, color="#f0a53a", alpha=0.18, zorder=1)
    ax2.text(12.6, 52.0,
             _t("★ 真机实际落在这一段\n（T_extra = 6 ~ 20 ms）", "real machines"),
             fontsize=8.6, color="#8a5a00", weight="bold", ha="center")
    ax2.annotate(_t("★ 仿真：T_extra = 0\n→ PM = 57.0°（很稳）",
                    "sim: PM = 57.0"),
                 xy=(0.0, 57.0), xytext=(3.0, 33.0), fontsize=8.6,
                 color="#1f4e79", weight="bold",
                 arrowprops=dict(arrowstyle="-|>", color="#1f4e79", lw=1.2))
    ax2.annotate(_t("真机 10 ms → PM 31.0°\n「开始难飞」",
                    "10 ms -> 31.0 deg"),
                 xy=(10.0, 31.0), xytext=(0.4, -7.0), fontsize=8.4,
                 color="#8a2b2b",
                 arrowprops=dict(arrowstyle="-|>", color="#8a2b2b", lw=1.2))
    ax2.annotate(_t("20 ms → PM 5.0°，图里写得很直白：必炸\n"
                    "（表里是 T_extra，总延迟 = 8 ms + T_extra = 28 ms）",
                    "20 ms -> 5.0 deg"),
                 xy=(20.0, 5.0), xytext=(6.0, -18.0), fontsize=8.4,
                 color="#8a2b2b",
                 arrowprops=dict(arrowstyle="-|>", color="#8a2b2b", lw=1.2))
    ax2.set_xlabel(_t("表里的 T_extra（ms）—— 【额外】延迟，口径见课文",
                      "T_extra (ms)"), fontsize=9.4)
    ax2.set_ylabel(_t("相位裕度（度）", "phase margin (deg)"), fontsize=9.4)
    ax2.set_xlim(-1.0, 32.0)
    ax2.set_ylim(-28.0, 62.0)
    ax2.grid(ls=":", color="#bbbbbb", zorder=0)
    ax2.set_title(_t("图 7-1-3b  扫 T_extra：PM 几乎线性下降，斜率 2.60°/ms\n"
                     "★ 同一个 $\\omega_c T$ 公式：穿越频率越高，延迟越致命",
                     "Fig 7-1-3b  PM vs T_extra"),
                  fontsize=10.8, weight="bold", pad=10)

    fig.suptitle(_t("★ 看图说话：仿真把 T_extra 当 0，所以 PM 有 57°；真机有 6~20 ms，"
                    "PM 掉到 41°~5° —— 这就是「仿真里调好、真机上振荡」的定量解释。\n"
                    "   注意口径：表里的 T_extra 是【额外】延迟，总延迟 = 8 ms + T_extra；"
                    "课文练习 3(b) 问的「总延迟 15 ms」不是表里的 15 ms（那是总延迟 23 ms）。",
                    "sim vs real phase margin"),
                 fontsize=9.2, color="#5a3d00", y=0.035)
    fig.tight_layout(rect=[0, 0.075, 1, 1])
    p = os.path.join(HERE, "fig_sitl_latency.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


def _g(v):
    """3.0 -> '3'，3.5 -> '3.5'（图里少几个小数点）"""
    return ("%g" % v)


# ===========================================================================
# 02 课 · 图 1：ULog vs DataFlash 字节布局对照
# ===========================================================================

def _bytes(ax, x, y, w, h, text, fc, ec, fs=8.2, tc="#111111"):
    box(ax, x, y, w, h, text, fc=fc, ec=ec, fs=fs, tc=tc)


def fig_log_formats():
    fig, ax = new_canvas(15.6, 9.2)
    title(ax, _t("图 7-2-1  ULog（PX4，小端）vs DataFlash（ArduPilot，大端）逐字节对照",
                 "Fig 7-2-1  ULog vs DataFlash"),
          _t("★ 看两张图里【同一个问题】的两个相反答案：ULog 的 msg_size "
             "【不含】3 字节消息头，DataFlash 的 FMT.length 【含】3 字节记录头",
             "length field: excluding vs including the header"))

    # ================= 左：ULog =================
    band(ax, 2, 8, 47, 80, fc="#eef4fb", ec="#9dbbdc")
    tag(ax, 4.0, 84.5, _t("ULog（PX4）· 小端 · 有文件头", "ULog (PX4)"),
        fs=10.4, weight="bold", color="#1f4e79", ha="left")

    _bytes(ax, 4, 76, 20, 6.6,
           _t("55 4C 6F 67 01 12 35 01\nASCII ULog + 4 个版本字节",
              "magic"), "#dbe9f7", "#4a7fb5", fs=7.6)
    _bytes(ax, 24.5, 76, 22, 6.6,
           _t("uint64 起始时间戳\n（0x000F4240 = 1 s）", "uint64 timestamp"),
           "#dbe9f7", "#4a7fb5", fs=7.6)
    tag(ax, 25.5, 73.0, _t("★ 固定 16 字节文件头", "16-byte header"), fs=8.2,
        color="#1f4e79", weight="bold")

    _bytes(ax, 4, 64, 42.5, 7.0,
           _t("FLAG_BITS：3 字节 = 0x0028(40) + 'B'\n"
              "★ MUST be written right after the ulog header（logger.cpp:2029）",
              "FLAG_BITS must follow the header"), "#fde3e3", "#b02b2b",
           fs=8.0)

    _bytes(ax, 4, 51, 15, 8.0,
           _t("3 字节头\nuint16 msg_size\nuint8 msg_type",
              "3-byte msg header"), "#fff3d6", "#c99a2e", fs=7.8)
    _bytes(ax, 20.5, 51, 26, 8.0,
           _t("payload = msg_size 字节\n★ msg_size【不含】左边这 3 字节\n"
              "（messages.h:99 原文 excluding the header size）",
              "payload"), "#fff9e8", "#c99a2e", fs=7.8)
    arrow(ax, (19.6, 55.0), (20.4, 55.0), color="#c99a2e", lw=1.6)

    tag(ax, 25.5, 47.5,
        _t("解析：size, mtype = read(3)；body = read(size)",
           "read(3) then read(size)"), fs=8.2, color="#7d4a00", weight="bold")
    _bytes(ax, 4, 34, 42.5, 11.0,
           _t("13 种消息类型（messages.h）\n"
              "F 格式 / D 数据 / I 信息 / M 多实例 / P 参数 / Q 默认参数\n"
              "A 记录消息 / R 删除消息 / S 同步 / O 掉数据 / L 文本 / C 带 tag 文本 / B 标志",
              "13 message types"), "#dff0e2", "#4a9a5b", fs=7.6)

    # ================= 右：DataFlash =================
    band(ax, 51, 8, 47, 80, fc="#f2f7f2", ec="#9dc4a6")
    tag(ax, 53.0, 84.5, _t("DataFlash（ArduPilot）· 大端 · 无文件头",
                           "DataFlash (ArduPilot)"), fs=10.4, weight="bold",
        color="#1f5e33", ha="left")

    _bytes(ax, 53, 76, 43, 6.6,
           _t("★ 没有文件头：第 1 条记录就是 FMT（msgid = 128）\n"
              "（LogStructure.h:1394  this must remain #128）",
              "no header: first record is FMT"), "#fde3e3", "#b02b2b", fs=8.0)

    _bytes(ax, 53, 63, 13, 8.0,
           _t("记录头\n0xA3 0x95\n+ uint8 msgid",
              "3-byte record header"), "#fff3d6", "#c99a2e", fs=7.8)
    _bytes(ax, 66.5, 63, 29.5, 8.0,
           _t("payload：type length name[4] format[16] labels[64]\n"
              "FMT 用 BBnNZ 描述自己（自举：解析器先查 FMT 表）",
              "payload of FMT"), "#fff9e8", "#c99a2e", fs=7.8)
    arrow(ax, (62.6, 67.0), (63.4, 67.0), color="#c99a2e", lw=1.6)

    _bytes(ax, 53, 50, 43, 8.5,
           _t("★ FMT.length = 89 【含】左边那 3 字节 = 3 + 1 + 1 + 4 + 16 + 64\n"
              "解析：body_len = length - 3（LogFile.cpp:35 原样写回日志）",
              "FMT.length = 89 includes the header"), "#fde3e3", "#b02b2b",
           fs=8.0)

    _bytes(ax, 53, 35, 43, 10.0,
           _t("一般记录：A3 95 | msgid | payload\n"
              "格式字符：B=uint8  n=char[4]  N=char[16]  Z=char[64]\n"
              "c/C/e/E = 定点小数（int16/int32，带精度）",
              "record layout + format chars"), "#dff0e2", "#4a9a5b", fs=7.8)

    # ================= 对照条 =================
    box(ax, 2, 21.0, 96, 9.0,
        _t("★ 同一件事，两个相反答案：\n"
           "   ULog：size【不含】3 字节头 → body = read(size)　　"
           "DataFlash：length【含】3 字节头 → body = read(length - 3)",
           "the length field means opposite things"),
        fc="#fdf3d0", ec="#c99a2e", fs=9.0, tc="#5a3d00")

    box(ax, 2, 8.0, 96, 11.5,
        _t("★ 看图说话：两套格式的差别【不在字段多少，而在元信息的放法】——\n"
           "   · ULog 把「怎么读这个文件」放在最前面（文件头 + FLAG_BITS），"
           "所以能【流式】解析（log_writer_mavlink 边飞边往地面站发）；\n"
           "   · DataFlash 把「怎么读」写进每一条 FMT 记录里，"
           "所以能【随便加新消息】而不破坏老解析器。\n"
           "   读任何二进制日志的第一件事，就是把「长度含不含头」问清楚 —— "
           "这是实战里最常踩的坑。",
           "metadata first vs self-describing"),
        fc="#eef2f7", ec="#8899aa", fs=8.2, tc="#334455")

    tag(ax, 50, 4.0,
        _t("★ 小端 vs 大端也是【相反的】：ULog 全部小端，DataFlash 全部大端 —— "
           "同一个 uint16 在两套日志里字节序不同。",
           "little-endian vs big-endian"), fs=8.4, color="#b02b2b",
        weight="bold")

    fig.tight_layout()
    p = os.path.join(HERE, "fig_log_formats.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 02 课 · 图 2：日志排查决策树（第 0~6 步，顺序不能换）
# ===========================================================================

TREE = [
    (_t("第 0 步\n日志可信吗", "step 0"),
     _t("① 有 DROPOUT 消息？（PX4；AP 看 MSG 文本 + 序号）\n"
        "② 时间戳有跳变 / dt 不均？",
        "dropouts / time jumps"),
     _t("先修记录链路：换高速卡、降日志频率；\n修好之前【不要】看后面的数据",
        "fix the recording path first"), "#eef2f7", "#8899aa"),
    (_t("第 1 步\n削顶", "step 1"),
     _t("PX4：vehicle_imu_status.accel_clipping > 0\n"
        "AP：VIBE.Clip 持续增长",
        "clipping counters"),
     _t("★ 最硬的证据。治机械：桨平衡 / 桨叶损伤 /\n"
        "电机轴承 / 机架软；此时【禁止调 PID】", "fix mechanics, not PID"),
     "#fde3e3", "#b02b2b"),
    (_t("第 2 步\n振动量级", "step 2"),
     _t("PX4：accel_vibration_metric 比同机型历史值涨 2 倍以上？（注意采样率！）\n"
        "AP：VIBE > 30 m/s²（这个数可以直接比）",
        "vibration trend"),
     _t("同上，治机械；可临时改\nINS_ACCEL_FILTER / INS_GYRO_FILTER",
        "filter as a stopgap"), "#fff3d6", "#c99a2e"),
    (_t("第 3 步\n姿态跟踪", "step 3"),
     _t("误差 RMS 比正常段涨了 3 倍以上？\n"
        "  · 且振动同期变大 → 振动串进姿态环 → 回第 1/2 步\n"
        "  · 振动正常 → 增益 / 延迟 / 重心 → 查 RATE 环与时间戳间隔",
        "attitude tracking error"),
     _t("分别指向「机械」和「控制」\n两条完全不同的路", "mechanics or control"),
     "#dbe9f7", "#4a7fb5"),
    (_t("第 4 步\n电机输出", "step 4"),
     _t("① 触限比例 > 0 → 动力余量不足\n"
        "② 四路极差 > 0.05 → 重心偏 / 电机效率不一\n"
        "③ 对角一高一低 → 配平重心",
        "motor outputs"),
     _t("换桨 / 减重 / 放宽 MPC_THR_MAX；\n配平重心后再看要不要动 trim",
        "power margin / CG"), "#dbe9f7", "#4a7fb5"),
    (_t("第 5 步\nEKF", "step 5"),
     _t("检验比持续 > 1（不是偶尔）：\n"
        "  · 只有 GPS 相关 → 多路径 / 干扰　· 磁相关 → 电流干扰 / 装反 / 附近有铁\n"
        "  · 气压相关 → 气流扰动、桨洗　· filter_fault_flags 非 0 / reset 计数增加",
        "EKF innovation ratio"),
     _t("治【观测源】，而不是调 EKF 参数", "fix the observation source"),
     "#e6e0f2", "#7a5fb5"),
    (_t("第 6 步\nFailSafe", "step 6"),
     _t("① failsafe_flags 里【哪个】条件被置位？  ← ★ 先看这个\n"
        "② vehicle_status.nav_state 切到了哪个模式？",
        "failsafe flags"),
     _t("回到第 1~5 步去找：\n这个条件【为什么】会被置位", "trace back to step 1-5"),
     "#e3f0e3", "#4a9a5b"),
]


def fig_log_decision_tree():
    fig, ax = new_canvas(15.6, 9.6)
    title(ax, _t("图 7-2-2  日志排查决策树：七步，而且【顺序不能换】",
                 "Fig 7-2-2  log triage decision tree"),
          _t("★ 不能换的理由只有一条：振动会同时污染姿态误差、电机输出和 EKF 新息 —— "
             "先看 EKF 很容易误判成「传感器坏了」（§24.5.4）",
             "order matters: vibration pollutes everything downstream"))

    n = len(TREE)
    h = 10.4
    ytop = 77.6
    for i, (lab, crit, act, fc, ec) in enumerate(TREE):
        y = ytop - i * 11.0
        box(ax, 2.5, y, 12.5, h, lab, fc=fc, ec=ec, fs=8.4, weight="bold")
        box(ax, 16.5, y, 50.0, h, crit, fc="#ffffff", ec=ec,
            fs=8.0 if _CJK else 7.6, tc="#222222")
        box(ax, 68.0, y, 29.5, h, act, fc=fc, ec=ec, fs=8.0, tc="#333333")
        if i:
            # 行与行之间只画【一支向下】的箭头：第 0 → 6 步是自上而下的顺序
            arrow(ax, (8.75, y + h + 1.05), (8.75, y + h + 0.02),
                  color="#888888", lw=1.6, ms=7)

    # ★ 列标题的位置试过两次才对：
    #   放在 84.6 → 落进第 0 步的框里，压住「修好之前【不要】看后面的数据」；
    #   放在 90.5 → 撞上副标题那一行（副标题 va="top" 从 92.5 往下占 ~1.3 格）。
    #   最终：把第一行整体下移到 88.0 顶，标题放 89.4，两侧各留 ~1.1 格。
    tag(ax, 41.5, 89.4,
        _t("判据（看什么数）", "criteria"), fs=9.2, weight="bold",
        color="#1f4e79")
    tag(ax, 82.7, 89.4,
        _t("结论 / 动作", "action"), fs=9.2, weight="bold", color="#1f5e33")

    box(ax, 2.5, 2.5, 95.0, 8.5,
        _t("★ 看图说话：七步的排列不是「重要性」顺序，而是【污染关系】顺序 —— "
           "上面的每一步都会污染下面所有步的证据。\n"
           "   所以第 1 步（削顶）的证据最硬：它是独立的计数器，"
           "而第 2 步的振动指标会被削顶【反向】污染（削得越狠，指标反而越小）。",
           "order = pollution, not importance"),
        fc="#fdf3d0", ec="#c99a2e", fs=8.4, tc="#5a3d00")
    tag(ax, 50.0, 0.6,
        _t("★ 第 3 步的分叉是关键：同一个「姿态误差变大」，"
           "同期振动大 → 回第 1/2 步治机械；振动正常 → 查增益/延迟/重心。",
           "step 3 forks"), fs=8.2, color="#1f4e79", weight="bold")

    fig.tight_layout()
    p = os.path.join(HERE, "fig_log_decision_tree.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ===========================================================================
# 02 课 · 图 3：40 秒飞行的两条独立因果链
# ===========================================================================

# 逐字抄自课文「练习 5」的表格：t0, t1, 备注, 振动, 削顶, 姿态误差, 触限, GPS 检验比
PHASES = [
    (0, 12, _t("起飞 + 悬停", "takeoff + hover"), "0.35 → 0.55", "0",
     0.98, "0%", "0.2~0.3"),
    (12, 15, _t("正常", "normal"), "0.55", "0", 1.1, "0%", "0.2~0.3"),
    (15, 19, _t("开始抖", "shake starts"), "2.6", "0", 5.5, "0%", "0.2~0.3"),
    (19, 25, _t("继续抖", "shaking"), "2.2", "11, 6", 5.5, "0%", "0.2~0.3"),
    (25, 28, _t("猛推俯仰", "hard pitch input"), "0.9", "0", 3.1, "50%",
     "0.2~0.3"),
    (28, 34, _t("正常", "normal"), "0.9", "0", 0.97, "0%", "0.2~0.3"),
    (34, 40, _t("failsafe = 1", "failsafe = 1"), "0.9", "0", 0.99, "0%",
     "0.2~1.8"),
]

LANES = [
    _t("accel_vibration_metric\n（振动指标）", "vibration"),
    _t("accel_clipping\n（削顶计数）", "clipping"),
    _t("姿态误差 RMS (deg)", "attitude err"),
    _t("电机触限比例", "motor saturation"),
    _t("gps_hpos 检验比", "gps innovation ratio"),
    _t("事件", "event"),
]


def fig_log_evidence_chain():
    fig, ax = plt.subplots(figsize=(15.6, 8.0))
    fig.patch.set_facecolor("white")
    ax.set_facecolor(C_BG)

    faint = "#f2f2f2"
    for (t0, t1, note, vib, clip, att, sat, gps) in PHASES:
        w = t1 - t0
        # 振动
        v = 0.0 if vib.startswith("0.35") else float(vib)
        cv = "#e05252" if v >= 2.0 else ("#f0a53a" if v >= 1.0 else "#cfe0c8")
        cells = [
            (vib, cv),
            (clip, "#e05252" if clip != "0" else "#cfe0c8"),
            ("%.2f" % att, "#e05252" if att >= 5.0 else
             ("#f0a53a" if att >= 1.5 else "#cfe0c8")),
            (sat, "#e05252" if sat != "0%" else "#cfe0c8"),
            (gps, "#e05252" if "1.8" in gps else "#cfe0c8"),
            (note, "#e8eef5"),
        ]
        for i, (txt, fc) in enumerate(cells):
            y = 5 - i
            tc = "#111111"
            if txt in ("11, 6",) or (i == 4 and "1.8" in txt):
                tc = "#ffffff"
            ax.add_patch(Rectangle((t0 + 0.08, y - 0.42), w - 0.16, 0.84,
                                   facecolor=fc, edgecolor="#ffffff",
                                   linewidth=1.2, zorder=3))
            ax.text((t0 + t1) / 2.0, y, txt, ha="center", va="center",
                    fontsize=8.6 if i < 5 else 8.2, color=tc, zorder=5,
                    weight="bold" if i < 5 else "normal")

    # 两条因果链的底色
    for (a, b, col, lab, sub, ha) in (
            (15, 28, "#b02b2b",
             _t("链 A（15 ~ 28 s）机械振动链",
                "chain A: mechanical vibration"),
             _t("桨叶损伤 → 振动 2.6 → 加速度计削顶 11,6\n"
                "→ 姿态环被激励 → 姿态误差 5.5°",
                "blade damage -> vibration -> clipping -> attitude error"),
             "left"),
            (34, 40, "#1f4e79",
             _t("链 B（34 ~ 40 s）GPS 观测链", "chain B: GPS observation"),
             _t("GPS 多路径 → 位置新息变大 → 检验比 > 1(1.8)\n"
                "→ EKF 拒绝 GPS → 触发 failsafe",
                "multipath -> innovation -> EKF rejects -> failsafe"),
             "right")):
        ax.add_patch(Rectangle((a, -3.3), b - a, 8.85,
                               facecolor=col, alpha=0.055, edgecolor="none",
                               zorder=1))
        ax.plot([a, a], [-3.3, 5.55], color=col, lw=1.4, ls="--", zorder=2)
        ax.plot([b, b], [-3.3, 5.55], color=col, lw=1.4, ls="--", zorder=2)
        x0 = a + 0.3 if ha == "left" else b - 0.3
        ax.text(x0, 10.0, lab, fontsize=9.2, color=col, weight="bold",
                ha=ha, va="center")
        ax.text(x0, 9.0, sub, fontsize=7.6, color=col, ha=ha,
                va="center", linespacing=1.5)

    ax.set_yticks(list(range(6))[::-1])
    ax.set_yticklabels(LANES, fontsize=8.8)
    ax.set_xlim(0, 40)
    ax.set_ylim(-3.4, 10.7)
    ax.set_xticks(range(0, 41, 5))
    ax.set_xlabel(_t("时间（秒）—— 一次 40 秒的飞行（课文练习 5 的表）", "t (s)"),
                  fontsize=9.6)
    ax.grid(axis="x", ls=":", color="#c0c0c0", zorder=0)
    ax.set_title(_t("图 7-2-3  一次 40 秒飞行里的两条【互相独立】的因果链",
                    "Fig 7-2-3  two independent causal chains in one 40 s log"),
                 fontsize=11.6, weight="bold", pad=26)

    ax.annotate(_t("★ 关键证据：链 B 期间振动已经回到 0.9、削顶 = 0\n"
                   "→ 链 B【不是】链 A 的延续，是另一个独立问题",
                   "vibration is back to normal during chain B"),
                xy=(36.6, 5.35), xytext=(15.2, 6.9), fontsize=8.4,
                color="#1f4e79", weight="bold", ha="left", va="center",
                arrowprops=dict(arrowstyle="-|>", color="#1f4e79", lw=1.4))
    ax.annotate(_t("★ 19~25 s：削顶计数 > 0 而振动指标【下降】（2.6 → 2.2）\n"
                   "—— 削顶砍掉的正是波峰，指标反而变小 → 指标不能用来判断削顶",
                   "clipping makes the metric go DOWN"),
                xy=(21.0, 4.0), xytext=(0.4, -1.15), fontsize=8.4,
                color="#b02b2b", weight="bold", ha="left", va="center",
                arrowprops=dict(arrowstyle="-|>", color="#b02b2b", lw=1.4))
    ax.annotate(_t("25~28 s 的 50% 触限是第三种问题（动力余量 / 重心），"
                   "不是振动链的后果 —— 它出现在振动已回落的时刻",
                   "50% saturation is a third problem"),
                xy=(26.6, 2.45), xytext=(0.4, -2.75), fontsize=8.4,
                color="#8a5a00", weight="bold", ha="left", va="center",
                arrowprops=dict(arrowstyle="-|>", color="#8a5a00", lw=1.2))

    fig.tight_layout()
    p = os.path.join(HERE, "fig_log_evidence_chain.png")
    fig.savefig(p, dpi=DPI, facecolor="white")
    plt.close(fig)
    saved(p)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

GROUPS = {
    "l1": [fig_sitl_datalink, fig_flu_frd, fig_sitl_latency],
    "l2": [fig_log_formats, fig_log_decision_tree, fig_log_evidence_chain],
}


def main():
    if not _HAS_MPL:
        print("没装 matplotlib，跳过画图（脚本仍然正常退出）。")
        return 0

    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    keys = args if args else list(GROUPS)
    for k in keys:
        if k not in GROUPS:
            print("未知分组 %s（可选：%s）" % (k, " / ".join(GROUPS)))
            return 0
    print("CJK 字体：%s" % ("已找到（中文标签）" if _CJK else
                            "★ 没找到，图里用英文回退"))
    for k in keys:
        for fn in GROUPS[k]:
            head(fn.__name__)
            fn()

    print()
    print("  共 %d 张图，全部落在 %s" % (sum(len(GROUPS[k]) for k in keys),
                                         HERE))
    return 0


if __name__ == "__main__":
    sys.exit(main())
