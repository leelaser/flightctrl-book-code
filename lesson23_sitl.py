#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""卷 10 第 1 章 —— SITL 仿真环境搭建（配套脚本）

    python lesson23_sitl.py        # 跑全部 4 个实验
    python lesson23_sitl.py 1      # 只跑实验 1
    python lesson23_sitl.py 2      # 只跑实验 2
    python lesson23_sitl.py 3      # 只跑实验 3
    python lesson23_sitl.py 4      # 只跑实验 4

★ 本脚本只用 numpy（可选 matplotlib），在【任何机器】上都能跑通。
  实验 1 会检测本机有没有 SITL 环境；检测不到就打印
  「环境不满足，下面是原理性演示」，然后继续做数值演示 —— 不会报错。

四个实验：
    1  环境体检：本机能不能真跑 SITL（优雅降级）
    2  ★★ PX4 + Gazebo 链路：传感器数据是怎么被凭空造出来的
    3  ★  ArduPilot SITL 链路：同一份物理真值，走另一条路
    4  ★  仿真边界：什么能测、什么测不了（含延迟的定量演示）

课程分工说明：
    本脚本讲的是【应用层】：数据从仿真器流到飞控应用层代码的那条路径。
    Gazebo 进程怎么调度、ChibiOS/POSIX 线程怎么创建，属于 RTOS/系统层，
    详见卷 5 第 11 章，本脚本不重复。
"""

import math
import _srcdirs
import os
import shutil
import sys

import numpy as np


# ============================================================================
#  输出小工具（与第 21、22 课脚本一致的排版）
# ============================================================================

W = 74


def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (W - 4))


def tab(rows, headers, width=None):
    """打印一张左对齐的表。width 是【显示宽度】（CJK 按 2 列算）。"""
    rows = [[str(c) for c in r] for r in rows]
    ncol = len(headers)
    if width is None:
        width = [0] * ncol
        for i in range(ncol):
            width[i] = _wide(str(headers[i]))
            for r in rows:
                if i < len(r):
                    width[i] = max(width[i], _wide(r[i]))
        width = [min(w, 34) for w in width]
    print()
    print("  " + "  ".join(str(headers[i]) + " " * (width[i] - _wide(str(headers[i])))
                           for i in range(ncol)))
    print("  " + "-" * (sum(width) + 2 * (ncol - 1)))
    for r in rows:
        cells = []
        for i in range(ncol):
            s = r[i] if i < len(r) else ""
            cells.append(s + " " * max(0, width[i] - _wide(s)))
        print("  " + "  ".join(cells))


def _dw(ch):
    """字符的显示宽度：CJK 占 2 列，其他占 1 列。"""
    o = ord(ch)
    if 0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF or \
       0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF or \
       0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60 or \
       0xFFE0 <= o <= 0xFFE6:
        return 2
    return 1


def _wide(s):
    return sum(_dw(c) for c in s)


# 行首禁则：这些标点不能出现在行首（否则会「孤字成行」，很难看）
NO_LINE_START = "：。，、；！？）」』】》…—～%"


def wrap(text, indent=4, width=W - 4):
    """按【显示宽度】折行打印（CJK 按 2 列算，对齐才好看）。"""
    pad = " " * indent
    line = pad
    w = 0
    for ch in text:
        if ch == "\n":
            print(line)
            line = pad
            w = 0
            continue
        cw = _dw(ch)
        # 标点不落行首：允许它把当前行挤出去一点点
        if w + cw > width and ch not in NO_LINE_START:
            print(line)
            line = pad
            w = 0
        line += ch
        w += cw
    if line.strip():
        print(line)


def bullet(items, indent=8):
    """打印一组要点（每条前面加 ·，缩进比 wrap 多一级）。"""
    for it in items:
        wrap("· " + it, indent=indent)


# ============================================================================
#  环境检测（实验 1 用）
# ============================================================================

# 本机可能存在的源码树候选路径（从常见位置找）
PX4_CANDIDATES = [
    _srcdirs.src("PX4"),
    _srcdirs.src("PX4_115"),
    r"~/PX4-Autopilot",
    r"~/src/PX4-Autopilot",
]

AP_CANDIDATES = [
    _srcdirs.src("APM"),
    r"~/ardupilot",
    r"~/src/ardupilot",
]


def expand(p):
    return os.path.abspath(os.path.expanduser(p))


def looks_like_px4(path):
    """PX4 源码树的指纹：Makefile + ROMFS/px4fmu_common/init.d-posix/rcS"""
    if not os.path.isdir(path):
        return False
    must = [
        "Makefile",
        "ROMFS/px4fmu_common/init.d-posix/rcS",
        "boards/px4/sitl/default.px4board",
    ]
    return all(os.path.exists(os.path.join(path, m)) for m in must)


def looks_like_ardupilot(path):
    """ArduPilot 源码树的指纹：ArduCopter + Tools/autotest/sim_vehicle.py"""
    if not os.path.isdir(path):
        return False
    must = [
        "ArduCopter",
        "libraries/AP_Scheduler/AP_Scheduler.cpp",
        "Tools/autotest/sim_vehicle.py",
    ]
    return all(os.path.exists(os.path.join(path, m)) for m in must)


def find_first(cands, test):
    for c in cands:
        p = expand(c)
        if test(p):
            return p
    return None


def have_cmd(name):
    """命令是否存在（不执行，只查 PATH）。"""
    return shutil.which(name) is not None


def have_module(name):
    """Python 模块是否存在（只查找，不导入，避免副作用）。"""
    try:
        import importlib.util
        return importlib.util.find_spec(name) is not None
    except Exception:
        return False


def run_cmd(args, timeout=10):
    """跑一个外部命令，返回 (returncode, 文本)。跑不起来返回 (None, "")。

    ★ 只用于【探测】，不用于启动任何服务 —— 探测失败必须返回空，不能抛异常，
      否则「环境不满足时优雅降级」这条设计就破了。
    """
    try:
        import subprocess
        p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout)
    except Exception:
        return None, ""
    # ★ wsl.exe 在 Windows 上输出的是 UTF-16LE。常见的错法是「把 NUL 删掉
    #   再按 UTF-8 解」—— 对纯 ASCII 没问题，但一遇到中文就露馅：
    #   「版本」（UTF-16LE 是 48 72 2C 67）会被当成 "Hr,g" 印出来。
    #   正确做法是先按 UTF-16LE 解，解不出来再退回单字节编码。
    raw = p.stdout
    encs = []
    if b"\x00" in raw:
        encs.append("utf-16-le")
    encs += ["utf-8", "gbk", "latin-1"]
    for enc in encs:
        try:
            s = raw.decode(enc)
        except Exception:
            continue
        return p.returncode, s.replace("\x00", "")
    return p.returncode, ""


def wsl_distros():
    """返回本机已安装的 WSL 发行版名列表。

    返回值有三种含义，必须区分开：
        None —— 不是 Windows，或压根没有 wsl.exe（不适用）
        []   —— WSL 装了，但【一个发行版都没有】（本机就是这种）
        [..] —— 真装了发行版
    """
    if sys.platform != "win32":
        return None
    if shutil.which("wsl") is None and shutil.which("wsl.exe") is None:
        return None
    rc, out = run_cmd(["wsl.exe", "-l", "-q"])
    if rc is None:
        return []
    names = []
    for line in out.splitlines():
        s = line.strip()
        if not s:
            continue
        # 剔除「适用于 Linux 的 Windows 子系统没有已安装的分发版」这类提示。
        # ★ 这句话是【中文】的，用 "Windows" 匹配不到，必须按中文关键词过滤。
        if ("Windows" in s or "wsl" in s.lower() or ":" in s
                or "没有" in s or "分发" in s or "distribution" in s.lower()):
            continue
        names.append(s)
    return names


def wsl_version():
    """WSL 内核版本字符串（没装返回 ""）。"""
    if sys.platform != "win32":
        return ""
    rc, out = run_cmd(["wsl.exe", "--version"])
    if rc != 0 or not out:
        return ""
    for line in out.splitlines():
        if "WSL" in line or "版本" in line:
            return line.strip()
    return out.splitlines()[0].strip() if out.splitlines() else ""


# 真机日志（SITL 跑不了时的【离线替代】）的候选位置
ULOG_ROOTS = [
    _srcdirs.src("ULOG"),
    r"~/PX4调参/ulog",
    r"~/Documents/Flight_Logs",
]


def scan_ulogs():
    """返回 (根目录或 None, .ulg 文件数)。"""
    for r in ULOG_ROOTS:
        p = os.path.abspath(os.path.expanduser(r))
        if not os.path.isdir(p):
            continue
        n = 0
        for dirpath, _d, files in os.walk(p):
            n += sum(1 for f in files if f.lower().endswith(".ulg"))
        return p, n
    return None, 0


# ============================================================================
#  实验 1：环境体检
# ============================================================================

def exp1():
    head("实验 1  环境体检：本机到底能不能真跑 SITL？")

    wrap("★ 这一节不是「检测不到就算了」，而是一张【可执行的检查表】："
         "每一项都真去查，并把【证据】（实际路径 / 命令输出）印出来。"
         "换一台机器跑，这张表会跟着变 —— 结论以你的机器为准。")

    sub("① 操作系统与虚拟化层（最容易被跳过的一步）")

    rows = []
    if sys.platform == "win32":
        rows.append(["原生 Linux", "否",
                     "当前是 Windows —— PX4 / ArduPilot 的 SITL "
                     "官方只支持 Linux / macOS"])
    else:
        rows.append(["原生 Linux",
                     "是" if sys.platform.startswith("linux") else "否",
                     "sys.platform = %s" % sys.platform])

    distros = wsl_distros()
    if distros is None:
        rows.append(["WSL 发行版", "不适用",
                     "不是 Windows，或本机压根没有 wsl.exe"])
    elif not distros:
        rows.append(["WSL 发行版", "0 个",
                     "WSL 本体在，但【一个发行版都没装】→ 仍然没有 Linux"])
    else:
        rows.append(["WSL 发行版", "%d 个" % len(distros), "、".join(distros)])

    wv = wsl_version()
    rows.append(["WSL 内核", "有" if wv else "没有",
                 wv or "wsl.exe --version 没有输出"])

    tab(rows, ["检查项", "结果", "证据"], width=[16, 8, 44])

    print()
    wrap("★★ 这一节是本章【最容易被误解】的地方，要分清三层：")
    bullet([
        "Windows 能不能原生跑 SITL：不能。SITL 编译出来的是一个 POSIX "
        "可执行文件（px4_sitl_default/bin/px4），它按 Linux 的进程 / 串口 / "
        "socket 模型写，Windows 上没有对应的构建目标和运行时；",
        "「装了 WSL」和「WSL 里有发行版」是【两件事】。本机 WSL 内核是有的，"
        "但 `wsl -l -q` 输出为空 —— 发行版一个都没有，没有 Linux 用户态；",
        "所以判断条件是【发行版数量 > 0】，而不是 wsl.exe 存在。"
        "只看 wsl.exe 在不在，会把「有 WSL」误判成「能跑 SITL」。",
    ])

    sub("② 源码树")

    px4 = find_first(PX4_CANDIDATES, looks_like_px4)
    ap = find_first(AP_CANDIDATES, looks_like_ardupilot)

    rows = []
    rows.append(["PX4 源码树",
                 "找到" if px4 else "没找到",
                 px4 if px4 else "（候选 %d 个位置都没找到）" % len(PX4_CANDIDATES)])
    rows.append(["ArduPilot 源码树",
                 "找到" if ap else "没找到",
                 ap if ap else "（候选 %d 个位置都没找到）" % len(AP_CANDIDATES)])
    tab(rows, ["项目", "结果", "路径"], width=[18, 8, 44])

    print()
    wrap("★ 源码树是【前提】—— SITL 不是一个独立的软件，"
         "它就是把你手上的固件源码换一个目标平台编译出来的产物。"
         "没有源码，就没有 SITL。")

    sub("③ PX4 侧：编译产物与依赖")

    rows = []

    px4_bin = None
    if px4:
        p = os.path.join(px4, "build/px4_sitl_default/bin/px4")
        if os.path.exists(p):
            px4_bin = p
    if px4_bin:
        rows.append(["PX4 SITL 已编译", "是", px4_bin])
    else:
        rows.append(["PX4 SITL 已编译", "否",
                     os.path.join(px4, "build/px4_sitl_default/bin/px4")
                     if px4 else "（连源码树都没有）"])
        rows.append(["  上面的目录存在吗", "否",
                     "要跑过 make px4_sitl gz_x500 才会生成"])

    gz_new = have_cmd("gz")
    gz_old = have_cmd("gazebo") or have_cmd("gzserver")
    gz_any = gz_new or gz_old
    rows.append(["Gazebo (gz sim)",
                 "有" if gz_new else "没有",
                 shutil.which("gz") or "新版本 Gazebo (Garden / Harmonic)"])
    rows.append(["Gazebo (classic)",
                 "有" if gz_old else "没有",
                 shutil.which("gazebo") or shutil.which("gzserver")
                 or "老版本 gazebo / gzserver"])

    tab(rows, ["检查项", "结果", "证据 / 实际路径"], width=[22, 6, 42])

    sub("④ ArduPilot 侧：编译产物与依赖")

    rows = []

    ap_bin = None
    if ap:
        for rel in ["build/sitl/bin/arducopter", "build/sitl/bin/arduplane"]:
            p = os.path.join(ap, rel)
            if os.path.exists(p):
                ap_bin = p
                break
    rows.append(["ArduPilot SITL 已编译",
                 "是" if ap_bin else "否",
                 ap_bin if ap_bin else "要 ./waf configure --board sitl "
                                       "&& ./waf copter 才会生成"])

    for mod, why in [("pexpect", "sim_vehicle.py 用它启动子进程"),
                     ("mavproxy", "默认会拉起 MAVProxy 控制台"),
                     ("pymavlink", "MAVLink 通信 / DataFlash 日志"),
                     ("pyulog", "PX4 ULog 解析（卷 10 第 2 章、离线替代都用它）")]:
        rows.append(["Python: " + mod,
                     "有" if have_module(mod) else "没有", why])

    tab(rows, ["检查项", "结果", "说明"], width=[22, 6, 42])

    sub("⑤ 离线替代：真机日志（SITL 跑不了时走这条路）")

    ulog_root, ulog_n = scan_ulogs()
    rows = []
    if ulog_root:
        rows.append(["真机 .ulg 日志", "%d 份" % ulog_n, ulog_root])
    else:
        rows.append(["真机 .ulg 日志", "0 份",
                     "候选：%s" % "；".join(ULOG_ROOTS[:2])])
    tab(rows, ["检查项", "结果", "证据"], width=[16, 8, 44])

    print()
    wrap("★ 这一行只要非 0，本章就【不会空手而归】：仿真告诉你算法对不对，"
         "真机日志告诉你这套飞机实际长什么样 —— 后者反而是仿真给不了的。"
         "配套脚本 code/lesson23b_real_log.py 就是用这批日志做实验的，"
         "振动 RMS / FFT 主峰 / GPS 精度 / EKF 检验比全都从里面算。")

    sub("⑥ 结论：三种状态，别混为一谈")

    can_native = (sys.platform.startswith("linux") or sys.platform == "darwin")
    can_px4 = bool(px4 and px4_bin and gz_any)
    can_ap = bool(ap and ap_bin and have_module("pexpect"))
    can_wsl = bool(distros)
    can_offline = ulog_n > 0

    rows = [
        ["原生 Linux / macOS 直接跑", "能" if can_native else "不能",
         "当前平台 = %s" % sys.platform],
        ["WSL 里跑（发行版已就绪）", "能" if can_wsl else "不能",
         "发行版 %d 个" % (len(distros) if distros else 0)],
        ["PX4 SITL 现在就能跑", "能" if can_px4 else "不能",
         "源码 + 编译产物 + Gazebo 三者齐全才算数"],
        ["ArduPilot SITL 现在就能跑", "能" if can_ap else "不能",
         "源码 + 编译产物 + pexpect 三者齐全才算数"],
        ["离线替代（真机日志）", "能" if can_offline else "不能",
         "%d 份 .ulg" % ulog_n],
    ]
    tab(rows, ["路径", "结果", "判据"], width=[26, 6, 36])

    print()
    if can_px4 or can_ap:
        wrap("★★ 本机 SITL 环境已就绪，实验 2/3/4 的链路可以直接对着真机跑。")
    elif can_wsl:
        wrap("★★ 本机有 WSL 发行版，但 SITL 还没编译 —— 先把源码编出来再回来。")
    else:
        print("  " + "*" * (W - 4))
        print("  ★★ 本机跑不了 SITL：没有 Linux，也没有可用的 WSL 发行版。")
        print("  " + "*" * (W - 4))
        print()
        wrap("★ 这是【实测结论】，不是猜的。上面每一行都能自己复查：")
        wrap("  想看 WSL 到底有没有发行版，敲一次：  wsl -l -q")
        wrap("  没有输出（或提示「没有已安装的分发版」）"
             "= 一个发行版都没有 = 没有 Linux 用户态可用。")
        print()
        wrap("★ 所以本章的做法是【两条腿走路】：")
        bullet([
            "实验 2/3/4 继续做【数值演示】—— 链路上每一步都是确定的数学，"
            "用 numpy 就能讲透，不需要仿真器；",
            "另开一条线：用 code/lesson23b_real_log.py 做【真机日志实验】—— "
            "卷 10 第 2 章要用的振动 / GPS / EKF 指标，全部从真机数据里算出来；",
            "想真跑 SITL 的读者，照 卷 10 第 1 章 的 §23.6 在 WSL 里装。"
            "本机没装，所以那几步【没有在本机验证过】，文档里逐条标了"
            "【已验证】/【未验证】。",
        ])
    print()
    wrap("★ 为什么本教程不把 SITL 设成必修：")
    wrap("  装 PX4 SITL 需要 Linux（或 WSL2）+ 几个 GB 的依赖；"
         "装 ArduPilot SITL 需要 pexpect + MAVProxy + 编译工具链。")
    wrap("  但【理解数据怎么流动】这件事，用 numpy 就能讲透 ——")
    wrap("  因为链路上每一步都是确定的数学，不是玄学。")

    sub("⑦ SITL 到底是什么：一张图")

    print("""
  ┌──────────────────── 你的电脑（一台机器，几个进程）────────────────────┐
  │                                                                      │
  │  ┌──────────────┐        ┌──────────────────────────────────────┐    │
  │  │  仿真器       │        │  飞控固件（真的固件！只是换了目标平台）│    │
  │  │              │        │                                      │    │
  │  │ Gazebo /     │        │  PX4:  px4_sitl_default/bin/px4      │    │
  │  │ jMAVSim /    │◀──────▶│  APM:  build/sitl/bin/arducopter     │    │
  │  │ JSBSim /     │  TCP/  │                                      │    │
  │  │ SIH /        │  UDP   │  ★ 姿态估计、控制律、导航、日志 ——    │    │
  │  │ SIM_*        │        │    全是【同一份源码】编译的          │    │
  │  └──────────────┘        └──────────────────────────────────────┘    │
  │         ▲                              ▲                             │
  │         │                              │                             │
  │         └──── 物理引擎算出来的真值       └──── 电机的 PWM 指令        │
  │                                                                      │
  └──────────────────────────────────────────────────────────────────────┘

  ★★ 关键认识：
     1. 右边跑的是【真固件】，不是模拟版固件。
        同一份 ekf2 / AC_AttitudeControl 代码，在 SITL 里和真机上逐字节相同。
     2. 左边和右边之间传的不是传感器读数，而是【物理真值 + 电机指令】。
        传感器读数是右边自己造出来的（这就是实验 2 要讲的）。
     3. 所以 SITL 测的是：算法逻辑、状态机、参数、时序。
        测不了的是：真实传感器的误差特性、真实振动、真实电源（实验 4）。
""")

    sub("⑧ 三条命令（环境就绪后自己跑）")

    print("""
  PX4 + Gazebo（在 PX4 源码根目录）：
      make px4_sitl gz_x500
      # 起来后在 pxh> 提示符里： commander takeoff

  PX4 内置仿真器 SIH（【不需要】装 Gazebo，最轻量）：
      make px4_sitl sihsim_quadx
      # ★ 这是最快的上手方式，机型定义在 airframes/10040_sihsim_quadx

  ArduPilot（在 ArduPilot 源码根目录）：
      ./waf configure --board sitl && ./waf copter     # 第一次要编译
      cd Tools/autotest
      python3 sim_vehicle.py -v ArduCopter -f quad --console --map
      # -v ArduCopter 的默认 frame 就是 quad，可以省略 -f
""")

    wrap("★ 上面三条命令里的名字（gz_x500 / sihsim_quadx / ArduCopter）"
         "都不是编的 —— 它们在源码里都有对应的文件，"
         "实验 2/3 会把它们的确切位置列出来。")


# ============================================================================
#  实验 2：PX4 + Gazebo —— 传感器数据是怎么被造出来的
# ============================================================================

def exp2():
    head("实验 2  ★★ PX4 + Gazebo：传感器数据是怎么被凭空造出来的")

    sub("① 先看链路：从 Gazebo 的一个刚体到飞控的 sensor_gyro")

    print("""
  [Gazebo 世界]                                    PX4 进程
  ┌────────────────────────┐                  ┌──────────────────────────┐
  │ world: default.sdf     │                  │                          │
  │  └ model: x500         │                  │  gz_bridge 模块          │
  │      └ link: base_link │                  │   ├─ imuCallback()       │
  │          └ sensor:     │  gz-transport    │   │   ↓ FLU→FRD 旋转     │
  │              imu_sensor│ ───────────────▶ │   ├─ sensor_accel 发布   │
  │                        │   (gz topic)     │   └─ sensor_gyro  发布   │
  │  物理引擎每几毫秒算一次 │                  │          ↓ uORB          │
  └────────────────────────┘                  │   VehicleIMU / ekf2 订阅 │
                                              └──────────────────────────┘
""")

    print("  ★ 源码位置（在本机 PX4-Autopilot 里逐行核对过）：")
    print()
    tab([
        ["gz_bridge 订阅 IMU", "src/modules/simulation/gz_bridge/GZBridge.cpp:182-189"],
        ["  gz topic 名字", "/world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu"],
        ["★ 造数据的函数", "GZBridge.cpp:450  imuCallback()"],
        ["★ FLU→FRD 旋转", "GZBridge.cpp:465-470"],
        ["★ 发布 accel", "GZBridge.cpp:487"],
        ["★ 发布 gyro", "GZBridge.cpp:510"],
        ["发布器类型", "GZBridge.hpp:130-131  uORB::PublicationMulti<sensor_accel_s>"],
        ["baro 同一条路", "GZBridge.cpp:399  barometerCallback()"],
        ["启动脚本", "ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator:117"],
        ["机型定义", "ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500"],
    ], ["环节", "位置"], width=[18, 56])

    sub("② 核心代码：一个回调函数，40 行，一点都不神秘")

    print("""
  // src/modules/simulation/gz_bridge/GZBridge.cpp:450
  void GZBridge::imuCallback(const gz::msgs::IMU &imu)
  {
      if (hrt_absolute_time() == 0) return;
      pthread_mutex_lock(&_node_mutex);

      const uint64_t time_us = (imu.header().stamp().sec() * 1000000)
                             + (imu.header().stamp().nsec() / 1000);

      // ★ FLU -> FRD：绕 X 轴转 180 度
      static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);   // :465

      gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
          imu.linear_acceleration().x(),                                   // :468
          imu.linear_acceleration().y(),
          imu.linear_acceleration().z()));

      // publish accel
      sensor_accel_s sensor_accel{};
      sensor_accel.timestamp_sample = hrt_absolute_time();      // :478
      sensor_accel.timestamp        = hrt_absolute_time();
      sensor_accel.device_id = 1310988;   // :481  DRV_IMU_DEVTYPE_SIM, BUS:1, ADDR:1
      sensor_accel.x = accel_b.X();
      sensor_accel.y = accel_b.Y();
      sensor_accel.z = accel_b.Z();
      sensor_accel.temperature = NAN;                           // :485
      sensor_accel.samples = 1;
      _sensor_accel_pub.publish(sensor_accel);                  // :487
      ...  // gyro 完全对称，见 :490-510
  }
""")

    print()
    wrap("★ 源码里还有一个 #if 值得注意（GZBridge.cpp:474-480、497-503）：")
    print()
    print("      #if defined(ENABLE_LOCKSTEP_SCHEDULER)")
    print("          sensor_accel.timestamp_sample = time_us;    // 仿真器时间")
    print("      #else")
    print("          sensor_accel.timestamp_sample = hrt_absolute_time();")
    print("      #endif")
    print()
    wrap("★★ 这是 SITL 里最重要的一个开关：")
    wrap("  开锁步（lockstep）时，Gazebo 和 PX4 用【同一个时钟】，"
         "仿真器算一步、PX4 跑一步，严格交替；")
    wrap("  关锁步时，两边各跑各的，靠墙上时间对齐。")
    wrap("  ★ 想复现「同一个种子跑出同一个结果」，必须开锁步 ——")
    wrap("    这正好呼应了实验 4 讲的「可复现性是 SITL 最大的价值」。")

    print()
    wrap("★★ 请注意这里【没有】做的事 —— 这才是关键：")

    print()
    for s in [
        "① 没有加噪声     —— Gazebo 的 IMU 插件可以加噪声，但 gz_bridge 不做",
        "② 没有加偏置     —— 真机上标定不掉的零偏，仿真里是 0",
        "③ 没有标定误差   —— 尺度因子 = 1、轴间不对齐 = 0",
        "④ 没有温度漂移   —— temperature 字段直接填 NAN",
        "⑤ 没有量化       —— 直接是 float，不是 ADC 读出来的 int16",
        "⑥ timestamp 是【软件时间】，不是传感器硬件采样时刻",
        "",
        "★ 所以 SITL 里的 ekf2 面对的是一个【完美传感器】。",
        "  它照样能跑、照样收敛 —— 但它收敛的难度和真机完全不是一回事。",
        "  这就是实验 4 要展开的仿真边界。",
    ]:
        print("    " + s)

    sub("③ 数值演示：把同一份物理真值走一遍这条链路")

    print()
    wrap("下面用 numpy 复现这条链路。真值取自一个悬停中的轻微横滚摆动 ——"
         "这是最常见的场景，也是加速度计最容易被误解的地方。")

    # --- 物理真值：悬停 + 横滚小幅摆动 ---
    fs_truth = 1000.0                 # 物理引擎内部步长 1 kHz
    fs_pub = 250.0                    # IMU 发布率 250 Hz（PX4 的 IMU_INTEG_RATE 默认值）
    T = 1.0
    n_truth = int(fs_truth * T)
    n_pub = int(fs_pub * T)

    t_truth = np.arange(n_truth) / fs_truth
    f_osc = 2.0                       # 摆动频率 2 Hz
    amp = math.radians(5.0)           # 摆幅 5 度
    g0 = 9.80665

    roll_truth = amp * np.sin(2 * math.pi * f_osc * t_truth)
    p_truth = amp * 2 * math.pi * f_osc * np.cos(2 * math.pi * f_osc * t_truth)

    # 悬停时推力抵消重力，比力（specific force）在机体 FLU 系里是：
    #     f_body = R_bw^T * (0, 0, +g0)      （ENU 系里向上 = +z）
    # 绕 x 轴滚转 phi 时： f_body = (0, g0*sin(phi), g0*cos(phi))
    f_flu_x = np.zeros_like(roll_truth)
    f_flu_y = g0 * np.sin(roll_truth)
    f_flu_z = g0 * np.cos(roll_truth)

    # --- 降采样到发布率 250 Hz（模拟 IMU 插件）---
    idx = (np.arange(n_pub) * (fs_truth / fs_pub)).astype(int)
    t_pub = t_truth[idx]
    ax_flu, ay_flu, az_flu = f_flu_x[idx], f_flu_y[idx], f_flu_z[idx]

    # 陀螺仪真值（FLU 系）。除了横滚角速率 p，再故意给一点俯仰率和偏航率，
    # 这样三根轴的符号变化都能在表里看见。
    gyro_flu_x = p_truth[idx]                 # 横滚角速率 p
    gyro_flu_y = np.full(n_pub, -0.10)        # 俯仰角速率 q
    gyro_flu_z = np.full(n_pub, 0.20)         # 偏航角速率 r

    # --- ★ FLU -> FRD：q = (w=0, x=1, y=0, z=0)，即绕 X 轴 180 度 ---
    # 用四元数旋转公式验算一遍（不跳步）：
    #   v' = v + 2*w*(q_vec x v) + 2*(q_vec x (q_vec x v))
    #   取 q = (w=0, q_vec=(1,0,0))，对 v = (vx, vy, vz)：
    #       (1,0,0) x v          = (0, -vz, vy)
    #       (1,0,0) x (0,-vz,vy) = (0, -vy, -vz)
    #       v' = v + 0 + 2*(0, -vy, -vz)
    #          = (vx, vy - 2*vy, vz - 2*vz)
    #          = (vx, -vy, -vz)                       ✓
    def flu_to_frd(v):
        w, qx, qy, qz = 0.0, 1.0, 0.0, 0.0
        qv = np.array([qx, qy, qz])
        cross1 = np.cross(qv, v)
        cross2 = np.cross(qv, cross1)
        return v + 2.0 * w * cross1 + 2.0 * cross2

    ax_frd = np.empty(n_pub)
    ay_frd = np.empty(n_pub)
    az_frd = np.empty(n_pub)
    for k in range(n_pub):
        r = flu_to_frd(np.array([ax_flu[k], ay_flu[k], az_flu[k]]))
        ax_frd[k], ay_frd[k], az_frd[k] = r

    gx_frd = np.empty(n_pub)
    gy_frd = np.empty(n_pub)
    gz_frd = np.empty(n_pub)
    for k in range(n_pub):
        r = flu_to_frd(np.array([gyro_flu_x[k], gyro_flu_y[k], gyro_flu_z[k]]))
        gx_frd[k], gy_frd[k], gz_frd[k] = r

    # ★ 数值自检：证明 a_FRD_y 就是 -g0*sin(roll)，而不是「差不多」
    err_y = float(np.max(np.abs(ay_frd + g0 * np.sin(roll_truth[idx]))))
    err_z = float(np.max(np.abs(az_frd + g0 * np.cos(roll_truth[idx]))))

    print()
    print("  表 A：加速度计（每 40 ms 打一行，共 1 秒）")
    step = int(fs_pub / 25)          # 每 40 ms
    rows = []
    for k in range(0, n_pub, step):
        rows.append([
            "%6.1f" % (t_pub[k] * 1000.0),
            "%7.2f" % math.degrees(roll_truth[idx][k]),
            "%8.3f" % ax_flu[k],
            "%8.3f" % ay_flu[k],
            "%8.3f" % az_flu[k],
            "%9.3f" % ax_frd[k],
            "%9.3f" % ay_frd[k],
            "%9.3f" % az_frd[k],
        ])
    tab(rows, ["t(ms)", "roll(deg)", "a_FLU_x", "a_FLU_y", "a_FLU_z",
               "a_FRD_x", "a_FRD_y", "a_FRD_z"],
        width=[7, 10, 9, 9, 9, 9, 9, 9])

    print()
    print("  表 B：陀螺仪（同样的取样点）")
    rows = []
    for k in range(0, n_pub, step):
        rows.append([
            "%6.1f" % (t_pub[k] * 1000.0),
            "%9.3f" % gyro_flu_x[k],
            "%9.3f" % gyro_flu_y[k],
            "%9.3f" % gyro_flu_z[k],
            "%9.3f" % gx_frd[k],
            "%9.3f" % gy_frd[k],
            "%9.3f" % gz_frd[k],
        ])
    tab(rows, ["t(ms)", "w_FLU_x", "w_FLU_y", "w_FLU_z",
               "w_FRD_x", "w_FRD_y", "w_FRD_z"],
        width=[7, 9, 9, 9, 9, 9, 9])

    print()
    print("  说明：a_* 单位 m/s^2，w_* 单位 rad/s。")
    print()
    print("  ★ 数值自检（max |误差|，用来证明不是「差不多」）：")
    print("      |a_FRD_y - (-g0*sin(roll))| = %.3e" % err_y)
    print("      |a_FRD_z - (-g0*cos(roll))| = %.3e" % err_z)
    if max(err_y, err_z) == 0.0:
        print("    误差是【精确的 0】—— 说明这不是「近似相等」，而是恒等式。")
    else:
        print("    误差只来自浮点舍入 —— 也就是说这个等式是【恒等式】。")

    sub("④ 这两张表要读出三件事")

    print()
    for s in [
        "① 【静止悬停时，Z 轴是 -9.81，不是 +9.81】",
        "   FLU 系里 z 向上，所以静止时是 +9.81（表 A 的 a_FLU_z）。",
        "   转成 FRD（z 向下）后变成 -9.81（表 A 的 a_FRD_z）。",
        "   ★ 在 PX4 日志里看到 sensor_accel 的 z 是负的，那是对的，不是符号错了。",
        "",
        "② 【横滚角被投影到 Y 轴上了】",
        "   上面那个自检说明：a_FRD_y 恒等于 -g0*sin(roll)，一点都不差。",
        "   roll = 4.99 度时，-g0*sin(4.99 度) = %.3f，表 A 里 t=120 ms 那行就是它。"
        % (-g0 * math.sin(math.radians(4.99))),
        "   ★★ 这就是加速度计能测姿态的全部秘密：",
        "       它测的不是姿态，是【重力在机体轴上的投影】。",
        "       而投影这件事，在匀加速运动时会被干扰 ——",
        "       飞机加速时 a_FRD_y 里混进了运动加速度，",
        "       此时单靠加速度计无法区分倾斜和加速。",
        "       → 这就是为什么必须有陀螺仪，也是 EKF 存在的理由。",
        "",
        "③ 【绕 X 轴转 180 度，X 分量不变，Y 和 Z 反号】",
        "   表 B 里：w_FRD_x == w_FLU_x（横滚角速率 p 没变），",
        "   而 w_FLU_y = -0.10 → w_FRD_y = +0.10，",
        "       w_FLU_z = +0.20 → w_FRD_z = -0.20。",
        "   ★ 这不是巧合：旋转轴上的分量当然不动，垂直轴上的分量才反号。",
        "     这也解释了为什么很多人第一眼觉得「转 180 度应该全都变号」是错的。",
        "",
        "   ★★ 如果这一步漏了或者转错了呢？",
        "      偏航角速率 r 的符号错了，偏航角速率环就变成【正反馈】——",
        "      飞机往右偏，控制律还往右打舵，一秒内转圈失控。",
        "      加速度计的 Y 轴符号错了，姿态估计会把「右倾」当成「左倾」，",
        "      起飞瞬间就往错误方向修正，直接翻掉。",
        "      → 这是 SITL 里最经典的一类 bug，也正是 gz_bridge 里",
        "        这一行要单独写注释的原因（GZBridge.cpp:464 写着 FLU -> FRD）。",
    ]:
        print("    " + s)

    sub("⑤ 发布出去之后：这条数据在 uORB 里怎么走")

    print()
    wrap("发布动作本身是 uORB 的活儿（卷 8 第 1 章讲透了），这里只补充"
         "本场景特有的两个数字：")

    print()
    tab([
        ["sensor_accel", "8", "250 Hz", "VehicleIMU、ekf2、logger 等"],
        ["sensor_gyro", "8", "250 Hz", "VehicleIMU、ekf2、logger 等"],
        ["sensor_baro", "4", "约 50 Hz", "ekf2、baro 相关模块"],
        ["sensor_mag", "4", "约 100 Hz", "ekf2、commander"],
    ], ["uORB topic", "ORB_QUEUE_LENGTH", "仿真发布率", "典型订阅者"],
        width=[16, 18, 12, 26])

    print()
    wrap("★ queue 长度 8 的含义（卷 8 第 1 章 的结论）：")
    wrap("  慢订阅者的稳态延迟 = o_queue - 1 = 7 代。")
    wrap("  250 Hz 发布 + 100 Hz 订阅，订阅者读到的是 7/250 = 28 ms 之前的数据。")
    wrap("  ★ 在真机上这是可以接受的（IMU 数据本来就带几毫秒延迟）；")
    wrap("    但在 SITL 里做延迟敏感的算法验证时，必须把这 28 ms 算进去 ——")
    wrap("    否则你会得到一个仿真里没问题、真机上振荡的控制器。")

    print()
    wrap("★★ 一句话总结实验 2：")
    print()
    print("      Gazebo 只负责算【刚体的比力和角速度】；")
    print("      gz_bridge 只负责【转坐标系 + 填结构体 + 发布】；")
    print("      真正的传感器是一个【空壳】—— 没有噪声、没有偏置、没有标定误差。")
    print()
    wrap("★ 所以仿真里 EKF 收敛得漂亮这件事，说明不了任何关于真机的问题。")


# ============================================================================
#  实验 3：ArduPilot SITL —— 同一份真值，走另一条路
# ============================================================================

def exp3():
    head("实验 3  ★ ArduPilot SITL：同一份物理真值，走【另一条】路")

    sub("① 怎么跑：一条命令，但下面藏着 5 件事")

    print("""
  cd ArduPilot/Tools/autotest
  python3 sim_vehicle.py -v ArduCopter -f quad --console --map

  ★ 这一条命令实际做了【五件事】（见 Tools/autotest/sim_vehicle.py）：
      1. 编译（除非加 -N/--no-rebuild）：waf 目标 bin/arducopter
      2. 生成参数文件：把 default_params/copter.parm 摊平成 SITL 的初始参数
      3. 启动仿真器进程（默认是 ArduPilot 自带的 SIM_*，不是 Gazebo）
      4. 启动 build/sitl/bin/arducopter，连到仿真器的 TCP/UDP 端口
      5. 拉起 MAVProxy（--console --map），让你在地面站里操作
""")

    print("  ★ 源码里的对应位置（本机 ardupilot-Copter-4.5.7 核对过）：")
    print()
    tab([
        ["命令行参数 -v / -f", "Tools/autotest/sim_vehicle.py:1030, :1035"],
        ["机型表（vehicle_choices）", "Tools/autotest/sim_vehicle.py:1024"],
        ["★ 机型 / 默认 frame 表", "Tools/autotest/pysim/vehicleinfo.py:10"],
        ["  ArduCopter 默认 frame", "vehicleinfo.py:11   default_frame = quad"],
        ["  quad 的 waf 目标", "vehicleinfo.py:18-20   waf_target = bin/arducopter"],
        ["  quad 的默认参数文件", "vehicleinfo.py:21   default_params/copter.parm"],
        ["★ 依赖 pexpect", "Tools/autotest/pysim/util.py:19   import pexpect"],
        ["物理模型入口", "libraries/SITL/SIM_Aircraft.cpp:609   Aircraft::update_model()"],
        ["★ 真值结构体", "libraries/SITL/SITL.h:50   struct sitl_fdm"],
        ["  机体系加速度真值", "libraries/SITL/SITL.h:59   xAccel, yAccel, zAccel"],
        ["★ 造加速度计的地方", "AP_InertialSensor_SITL.cpp:73   generate_accel()"],
        ["★ 造陀螺仪的地方", "AP_InertialSensor_SITL.cpp:207   generate_gyro()"],
        ["  accel 注册", "AP_InertialSensor_SITL.cpp:385   _imu.register_accel(...)"],
        ["  gyro 注册", "AP_InertialSensor_SITL.cpp:383   _imu.register_gyro(...)"],
        ["★ SIM_* 参数表", "libraries/SITL/SITL.cpp:863 起   AP_GROUPINFO(\"ACC1_BIAS\", ...)"],
    ], ["环节", "位置"], width=[24, 50])

    sub("② ★★ 关键差别：ArduPilot 没有 uORB，它往哪里发布？")

    print("""
  PX4 的链路（实验 2）：
      Gazebo ──gz topic──▶ gz_bridge ──uORB publish──▶ VehicleIMU ──▶ ekf2
                                        ↑
                              【跨线程、跨模块的边界】

  ArduPilot 的链路（本实验）：
      SIM_*（物理模型）──▶ sitl->state（struct sitl_fdm）
                              │
                              │  ★ 就是一块普通内存，同一个进程
                              ▼
      AP_InertialSensor_SITL::generate_accel()   ← 定时器回调，不是线程
                              │
                              │  加 trim / scale / bias / noise / vibration
                              ▼
      _imu._accel[instance].push_sample()        ← 写进 IMU 后端的采样缓冲
                              │
                              ▼
      AP_Scheduler 主循环里的 AP::ins().wait_for_sample()  ← 阻塞等新样本
                              │
                              ▼
      AP_AHRS / 控制律 / 导航 —— 全是同线程的函数调用

  ★★ 所以 ArduPilot 的传感器是【一个 C++ 对象的内部缓冲】，
     不是一条消息总线上的消息。
     数据传递靠【共享内存 + 直接调用】，不靠拷贝和队列。
     —— 这正是第 6 章 02 §22.1 说的耦合换确定性。

  ★ 但请注意：ArduPilot 也有跨线程的地方 ——
     定时器回调（generate_accel）跑在定时器线程，
     主循环跑在主线程，两者之间靠【采样缓冲的索引】同步，不是靠锁。
     这部分属于 RTOS 层，详见卷 5 第 11 章。
""")

    sub("③ 数值演示：把 sitl_fdm 真值走一遍 generate_accel()")

    print()
    wrap("下面把 AP_InertialSensor_SITL::generate_accel() 的每一步列出来 ——"
         "每一步都标了源码行号，顺序和源码完全一致"
         "（libraries/AP_InertialSensor/AP_InertialSensor_SITL.cpp:73-202）：")

    print()
    print("""
  :73   void AP_InertialSensor_SITL::generate_accel()
  :76       uint8_t nsamples = enable_fast_sampling(accel_instance) ? 4 : 1;
  :78       for (uint8_t j = 0; j < nsamples; j++) {
  :80-82         accel = Vector3f(sitl->state.xAccel, .yAccel, .zAccel);  ← 物理真值
  :84-89         if (!accel_trim.is_zero()) { ... accel = trim_rotation.transposed()*accel; }
  :91-103        if (!is_zero(accel_scale.x)) accel.x /= accel_scale.x;   ← 注意是【除】
  :105-107       accel += accel_bias;                                     ← 直接加
  :111           float accel_noise = 0.01f;        ← 【每轮循环都重置】
  :112           float noise_variation = 0.05f;
  :117           accel += Vector3f{rand_float(), rand_float(), rand_float()} * accel_noise;
  :119           bool motors_on = sitl->throttle > sitl->ins_noise_throttle_min;
  :123-126       if (motors_on) accel_noise = sitl->accel_noise[accel_instance];
  :128-136       if (!vibe_freq.is_zero() && motors_on) { 加正弦振动 }     ← 用新 accel_noise
  :138-158       if (!is_zero(vibe_motor) && motors_on) { 按 RPM 加谐波振动 }
  :160-176       if (!pos_offset.is_zero()) { 加杆臂 + 向心加速度修正 }
  :178-180       if (fabsf(accel_fail) > 1.0e-6f) accel = accel_fail;     ← 硬故障
  :194           accel_accum += accel;
  :195       }
  :197       accel_accum /= nsamples;                                    ← 多点平均
  :198       _rotate_and_correct_accel(accel_instance, accel_accum);      ← 机体系→板载系
  :199       _notify_new_accel_raw_sample(accel_instance, accel_accum, AP_HAL::micros64());
  :201       _publish_temperature(accel_instance, get_temperature());
""")

    print()
    wrap("★ 陀螺仪那边是对称的（generate_gyro，:208-300 起），但有三处【不一样】，"
         "而且都是很容易踩的坑：")

    print()
    print("""
  ① 噪声是【均匀分布】，不是高斯：
       :224-226   p += gyro_noise * rand_float();
                 rand_float() 的定义在 libraries/AP_Math/AP_Math.cpp:344-347：
                     return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
                 也就是【均匀分布在 -1.0 到 +1.0 之间】。
       ★ 真 MEMS 陀螺的噪声近似高斯，SITL 用的是均匀分布 ——
         这是一个真实的建模差异。均匀分布的标准差 = 幅值/sqrt(3)。

  ② 尺度的量纲不一样：
       加速度计（:96-102）：accel.x /= accel_scale.x;       ← 直接【除】1.002 这种数
       陀螺仪  （:283-285）：gyro.x *= (1 + scale.x*0.01f); ← 1 表示 1%，是【百分比】
       ★ 两个相邻参数，一个量纲是「倍数」、一个是「百分比」，而且加速度计那个
         默认 0 表示【不缩放】（is_zero 判断直接跳过除法），陀螺仪那个默认 0 表示
         不缩放（1 + 0 = 1）。含义完全不同，看名字绝对猜不出来。

  ③ 陀螺的噪声随油门放大：
       :233   gyro_noise = ToRad(sitl->gyro_noise[gyro_instance]) * sitl->throttle;
       ★ 单位是【度/秒】（ToRad 转换），还乘了油门 ——
         因为真机上电机转速越高，振动耦合到陀螺上就越大。
""")

    # --- 用同一个物理真值（沿用实验 2 的横滚摆动），但按 AP 的流程走 ---
    fs = 1000.0
    T = 1.0
    N = int(fs * T)
    t = np.arange(N) / fs
    f_osc = 2.0
    amp = math.radians(5.0)
    g0 = 9.80665

    roll = amp * np.sin(2 * math.pi * f_osc * t)

    # AP 的 sitl_fdm 里 xAccel/yAccel/zAccel 已经是【机体系 FRD】、单位 m/s^2
    # （libraries/SITL/SITL.h:59  double xAccel, yAccel, zAccel; // m/s/s in body frame）
    # 悬停 + 横滚 phi：比力 = (0, -g0*sin(phi), -g0*cos(phi))
    # ★ 和实验 2 的 a_FRD_y / a_FRD_z 完全一致 —— 两个飞控用的是同一套物理。
    ax_true = np.zeros(N)
    ay_true = -g0 * np.sin(roll)
    az_true = -g0 * np.cos(roll)

    # 真实参数（默认值和量纲都来自 libraries/SITL/SITL.cpp）
    accel_bias = np.array([0.05, -0.12, 0.08])   # SIM_ACC1_BIAS :868，默认 0
    accel_scale = np.array([1.002, 0.998, 1.001])  # SIM_ACC1_SCAL :957，默认 0 = 不缩放
    acc1_rnd = 0.35                              # SIM_ACC1_RND :890，默认 0（m/s^2）
    accel_noise_floor = 0.01                     # 源码 :111 float accel_noise = 0.01f;
    throttle = 0.5                               # 悬停油门
    ins_thr_min = 0.1                            # SIM_INS_THR_MIN :317 默认 0.1f

    rng = np.random.default_rng(20240501)

    # ★ 复刻 rand_float()：均匀分布在 [-1, +1)
    #   （AP_Math.cpp:347  (random()%2000000 - 1.0e6)/1.0e6）
    def rand_float(n):
        return rng.uniform(-1.0, 1.0, n)

    def uniform_rms(amplitude):
        """均匀分布 [-a, +a] 的标准差 = a / sqrt(3)"""
        return amplitude / math.sqrt(3.0)

    def rms(x):
        return float(np.sqrt(np.mean(np.square(x))))

    # 源码 :91-103  —— 注意是【除】，而且 accel_scale 为 0 时【整步跳过】
    ax_s = ax_true / accel_scale[0]
    ay_s = ay_true / accel_scale[1]
    az_s = az_true / accel_scale[2]

    # 源码 :105-107  加零偏
    ax_b = ax_s + accel_bias[0]
    ay_b = ay_s + accel_bias[1]
    az_b = az_s + accel_bias[2]

    # 源码 :111-117  ★ 这里用的是【0.01 的下限】，不是 SIM_ACC1_RND！
    nx = rand_float(N) * accel_noise_floor
    ny = rand_float(N) * accel_noise_floor
    nz = rand_float(N) * accel_noise_floor
    ax_n = ax_b + nx
    ay_n = ay_b + ny
    az_n = az_b + nz

    # 源码 :119-126  电机开了之后，accel_noise 才被换成 SIM_ACC1_RND
    motors_on = throttle > ins_thr_min
    accel_noise = acc1_rnd if motors_on else accel_noise_floor

    # 源码 :128-136  振动（VIB_FREQ 非零且电机开着才会走这一步）
    vibe_freq = np.array([0.0, 0.0, 0.0])        # SIM_VIB_FREQ 默认 0
    if motors_on and np.any(vibe_freq != 0.0):
        accel_time = np.arange(N) / fs
        ax_n = ax_n + np.sin(accel_time * 2 * math.pi * vibe_freq[0]) * accel_noise
        ay_n = ay_n + np.sin(accel_time * 2 * math.pi * vibe_freq[1]) * accel_noise
        az_n = az_n + np.sin(accel_time * 2 * math.pi * vibe_freq[2]) * accel_noise

    # 源码 :178-180  硬故障（SIM_ACCEL1_FAIL 默认 0 = 不故障）
    accel_fail = 0.0
    if abs(accel_fail) > 1.0e-6:
        ax_n[:] = ay_n[:] = az_n[:] = accel_fail

    print()
    print("  三个轴，逐步统计（单位 m/s^2）。")
    print("  ★ 前四行是【累积】的：②包含②之前所有步骤，③包含②，④包含③。")
    print()
    print("  X 轴（横滚轴，理想情况恒为 0）：")
    tab([
        ["① 真值 (sitl->state)", "%.4f" % float(np.mean(ax_true)),
         "%.4f" % rms(ax_true)],
        ["② 除以 SIM_ACC1_SCAL(1.002)", "%.4f" % float(np.mean(ax_s)),
         "%.4f" % rms(ax_s)],
        ["③ 加 SIM_ACC1_BIAS(+0.05)", "%.4f" % float(np.mean(ax_b)),
         "%.4f" % rms(ax_b)],
        ["④ 加噪声(下限 %g)" % accel_noise_floor, "%.4f" % float(np.mean(ax_n)),
         "%.4f" % rms(ax_n)],
        ["   纯噪声部分 nx", "%.4f" % float(np.mean(nx)), "%.4f" % rms(nx)],
        ["   理论 %g/sqrt(3)" % accel_noise_floor, "-",
         "%.4f" % uniform_rms(accel_noise_floor)],
    ], ["步骤", "均值", "RMS"], width=[28, 12, 12])

    print()
    print("  Y 轴（承载重力投影 -g0*sin(roll)，roll 摆幅 5 度，"
          "理论峰值 %.4f）：" % (-g0 * math.sin(amp)))
    tab([
        ["① 真值 (sitl->state)", "%.4f" % float(np.mean(ay_true)),
         "%.4f" % rms(ay_true)],
        ["② 除以 SIM_ACC1_SCAL(0.998)", "%.4f" % float(np.mean(ay_s)),
         "%.4f" % rms(ay_s)],
        ["③ 加 SIM_ACC1_BIAS(-0.12)", "%.4f" % float(np.mean(ay_b)),
         "%.4f" % rms(ay_b)],
        ["④ 加噪声(下限 %g)" % accel_noise_floor, "%.4f" % float(np.mean(ay_n)),
         "%.4f" % rms(ay_n)],
        ["   纯噪声部分 ny", "%.4f" % float(np.mean(ny)), "%.4f" % rms(ny)],
        ["   理论 %g/sqrt(3)" % accel_noise_floor, "-",
         "%.4f" % uniform_rms(accel_noise_floor)],
    ], ["步骤", "均值", "RMS"], width=[28, 12, 12])

    print()
    print("  Z 轴（悬停时承载 -g0 = %.4f）：" % (-g0))
    tab([
        ["① 真值 (sitl->state)", "%.4f" % float(np.mean(az_true)),
         "%.4f" % rms(az_true)],
        ["② 除以 SIM_ACC1_SCAL(1.001)", "%.4f" % float(np.mean(az_s)),
         "%.4f" % rms(az_s)],
        ["③ 加 SIM_ACC1_BIAS(+0.08)", "%.4f" % float(np.mean(az_b)),
         "%.4f" % rms(az_b)],
        ["④ 加噪声(下限 %g)" % accel_noise_floor, "%.4f" % float(np.mean(az_n)),
         "%.4f" % rms(az_n)],
        ["   纯噪声部分 nz", "%.4f" % float(np.mean(nz)), "%.4f" % rms(nz)],
        ["   理论 %g/sqrt(3)" % accel_noise_floor, "-",
         "%.4f" % uniform_rms(accel_noise_floor)],
    ], ["步骤", "均值", "RMS"], width=[28, 12, 12])

    # ★ 标定实验：采样点数 N 越多，零偏估计越准 —— 定量验证 1/sqrt(N)
    def est_std(n, trials=3000):
        """重复 trials 次「采 n 个点求均值」的零偏估计，返回估计值的标准差"""
        e = np.empty(trials)
        for i in range(trials):
            e[i] = float(np.mean(rand_float(n) * accel_noise_floor))
        return float(np.std(e))

    print()
    print("  ★ 标定实验：静止时把加速度计采样 N 次取平均，用均值反解零偏。")
    print("    噪声带来的估计误差应该按 1/sqrt(N) 缩小 —— 验证一下：")
    print()
    rows = []
    for n in [1, 10, 100, 1000, 10000]:
        emp = est_std(n)
        theo = uniform_rms(accel_noise_floor) / math.sqrt(n)
        rows.append(["%6d" % n,
                     "%.5f" % emp,
                     "%.5f" % theo,
                     "%.2f" % (emp / theo if theo > 0 else 0.0),
                     "%.4f" % math.degrees(math.atan(emp / g0))])
    tab(rows, ["采样点数 N", "实测标准差",
               "理论 %g/sqrt(3N)" % accel_noise_floor,
               "实测/理论", "等效倾角(度)"],
        width=[11, 12, 20, 11, 14])

    print()
    wrap("★★ 这张表的含义：")
    print()
    for s in [
        "· 「实测/理论」一列都在 1.0 附近 —— 说明 1/sqrt(N) 这个规律是对的；",
        "· N = 1 时，单次采样的噪声就有 %.4f m/s^2，等效倾角 %.3f 度；"
        % (uniform_rms(accel_noise_floor),
           math.degrees(math.atan(uniform_rms(accel_noise_floor) / g0))),
        "· N = 10000 时，只剩 %.5f m/s^2，等效倾角 %.5f 度。"
        % (uniform_rms(accel_noise_floor) / 100.0,
           math.degrees(math.atan(uniform_rms(accel_noise_floor) / 100.0 / g0))),
        "",
        "★ 这就是为什么加速度计标定要「静置几秒、采上千个点」——",
        "  不是为了等传感器稳定，而是为了用 1/sqrt(N) 把噪声平均掉。",
        "★★ 也是为什么标定必须在【真正静止且水平】时做：",
        "  如果飞机在动，均值里混进的是运动加速度，那不是零偏，",
        "  平均再多也消不掉 —— 它会变成一个【假的零偏】被写进参数里。",
    ]:
        print("    " + s)

    sub("④ 从这张表读出的结论")

    print()
    for s in [
        "① 【零偏是最致命的误差，因为它不会被平均掉】",
        "   X 轴加了 +0.05 的 bias，④ 行的均值就是 0.0502 ——",
        "   加噪声只是让它在 0.05 附近抖，压不到 0。",
        "   而纯噪声那一行的均值是 %.4f（接近 0），RMS 是 %.4f。"
        % (float(np.mean(nx)), rms(nx)),
        "   ★ 均值 = 零偏，RMS = 零偏 + 噪声。这就是「标定看均值」的原因。",
        "   ★★ 0.05 m/s^2 对应的等效倾角是：",
        "       atan(0.05 / 9.81) = %.3f 度" % math.degrees(math.atan(0.05 / 9.81)),
        "     这个倾角误差会【永久】地让飞机往一个方向漂。",
        "   ★★ 所以飞控开机第一件事是【加速度计标定】：",
        "       标定的本质不是校准传感器，而是【测出零偏并减掉】。",
        "",
        "② 【SITL 的噪声是均匀分布，不是高斯】",
        "   上面「纯噪声部分」的 RMS 和「理论 %g/sqrt(3) = %.4f」完全对上。"
        % (accel_noise_floor, uniform_rms(accel_noise_floor)),
        "   ★ 真 MEMS 器件的噪声近似高斯（数据手册给的是噪声密度，",
        "     单位 mg/sqrt(Hz)），SITL 用的是均匀分布 rand_float()。",
        "   ★★ 后果：仿真里的噪声【没有长尾】——",
        "      真机上偶尔会出现的那个「5 sigma 野值」在仿真里永远不会出现。",
        "      而 EKF 的卡方检验（innovation gate）恰恰是靠长尾才有意义的。",
        "      这就是「仿真里 EKF 从不拒绝量测，真机上却偶尔拒」的原因之一。",
        "",
        "③ 【尺度因子误差只在有信号时才显现】",
        "   X 轴真值是 0，除以 1.002 还是 0 —— scale 误差在 X 轴上【毫无效果】。",
        "   Z 轴有 -9.81 的大信号，SIM_ACC1_SCAL = 1.001 就让 Z 从",
        "   %.4f 变成 %.4f，差了 %.4f m/s^2。"
        % (float(np.mean(az_true)), float(np.mean(az_s)),
           float(np.mean(az_s) - np.mean(az_true))),
        "   ★ 这解释了为什么加速度计标定要【六个面都做】：",
        "     只有一个面朝上，你只能标出那一个轴的 scale；",
        "     六个面各朝上一次，才能把三个轴的 scale 和三个零偏都解出来。",
        "",
        "④ 【源码里最容易看错的一步：噪声用的是哪一档】",
        "   源码 :111 每轮循环都先把 accel_noise 重置成 0.01f，:117 用它加噪声，",
        "   之后 :123-126 才换成 SIM_ACC1_RND。",
        "   ★★ 也就是说：如果 nsamples == 1（默认情况），",
        "      基础噪声永远用 0.01 这个下限，",
        "      SIM_ACC1_RND 只影响【振动项】（:131-136）。",
        "   ★ 这种「顺序决定语义」的地方，光看参数文档是看不出来的 ——",
        "     必须读代码。这也正是本教程反复强调「不许凭印象」的原因。",
    ]:
        print("    " + s)

    sub("⑤ ArduPilot SITL 比 PX4 SITL 强在哪：故障注入")

    print()
    wrap("看 libraries/SITL/SITL.cpp 的参数表就知道 —— ArduPilot 把"
         "传感器故障做成了一等公民：")

    print()
    tab([
        ["SIM_ACC1_BIAS / 2 / 3", "每个加速度计的零偏 (m/s^2)", "SITL.cpp:868,874,881"],
        ["SIM_ACC1_SCAL / 2 / 3", "每个加速度计的尺度因子（除数）", "SITL.cpp:957,963,970"],
        ["SIM_ACC1_RND / 2 / 3", "加速度计噪声强度 (m/s^2)", "SITL.cpp:890,892,895"],
        ["SIM_GYR1_RND / 2 / 3", "陀螺仪噪声强度 (deg/s)", "SITL.cpp:883,885,888"],
        ["SIM_GYR1_SCALE / 2 / 3", "陀螺仪尺度因子（百分比）", "SITL.cpp:902,908,915"],
        ["SIM_ACCEL1_FAIL / 2 / 3", "★ 直接让某个加速度计失效", "SITL.cpp:922,929,937"],
        ["SIM_GYR_FAIL_MSK", "★ 陀螺仪失效掩码", "SITL.cpp:944"],
        ["SIM_ACC_FAIL_MSK", "★ 加速度计失效掩码", "SITL.cpp:950"],
        ["SIM_VIB_FREQ", "★ 定频正弦振动 (Hz)", "SITL.cpp:233"],
        ["SIM_VIB_MOT_MAX", "★ 按 RPM 缩放的振动幅值", "SITL.cpp:315"],
        ["SIM_VIB_MOT_HMNC", "★ 振动谐波次数（默认 1）", "SITL.cpp:309"],
        ["SIM_VIB_MOT_MULT", "★ 振动缩放（默认 1.0）", "SITL.cpp:319"],
        ["SIM_INS_THR_MIN", "振动生效的最小油门（默认 0.1）", "SITL.cpp:317"],
        ["SIM_ACC_TRIM", "IMU 安装角误差 (rad)", "SITL.cpp:972"],
        ["SIM_IMU_POS", "IMU 安装位置偏移 (m)", "SITL.cpp:168"],
    ], ["参数", "作用", "源码位置"], width=[24, 30, 22])

    print()
    wrap("★★ 这几个参数的价值极高，因为它们能直接验证【故障检测与隔离】"
         "（FDI）能力：")
    print()
    for s in [
        "· 把 SIM_ACC2_BIAS 设成一个明显错误的零偏 →",
        "  看飞控能不能检测到两个加速度计不一致，并隔离其中一个；",
        "· 把 SIM_GYR_FAIL_MSK 打开 → 模拟陀螺仪彻底不响应 →",
        "  看会不会触发 EKF 的 failsafe（卷 10 第 2 章）；",
        "· 把 SIM_VIB_MOT_MAX 设成 5 左右 + SIM_INS_THR_MIN 调低 →",
        "  模拟真实振动水平。这个 5 不是瞎猜的：",
        "  源码 :121-122 的注释写着「真机 180mm 机架在油门 0.2-0.8 时",
        "  加速度计噪声在 0.8 到 4 m/s/s 之间」，也就是这个量级。",
        "· 把 SIM_ACC_TRIM 设成 0.02 rad（约 1.1 度）→",
        "  模拟 IMU 装歪了，看姿态估计会被带偏多少。",
        "",
        "★ PX4 也能做类似的事（改 SDF 模型加噪声插件，或用 sensor_*_sim 模块），",
        "  但没有这么细的按实例参数化。",
        "  这是 ArduPilot 在用仿真验证可靠性这件事上的明显优势。",
        "  ★ 原因还是架构：ArduPilot 的 SITL 传感器就在库内部，",
        "    库的参数表（AP_GROUPINFO）顺手就能加一行；",
        "    PX4 的传感器在另一个模块里，要动 gz_bridge 或 SDF 文件。",
    ]:
        print("    " + s)


# ============================================================================
#  实验 4：仿真边界 —— 什么能测、什么测不了
# ============================================================================

def exp4():
    head("实验 4  ★ 仿真边界：什么能测、什么测不了")

    sub("① 能测 / 不能测：一张清单")

    tab([
        ["算法逻辑（EKF 推导、控制律、状态机）", "能", "同一份源码，逐字节相同"],
        ["参数取值与默认值", "能（且无风险）", "改错了最多重开一次仿真"],
        ["任务调度与时序（第 6 章 02 的内容）", "能", "@SYS/tasks.txt 直接看"],
        ["MAVLink 协议与地面站交互", "能", "协议层是纯软件"],
        ["航线 / 任务 / 返航逻辑", "能", "用命令注入模拟各种情况"],
        ["飞行模式切换与解锁检查", "能", "可以构造任意前置条件"],
        ["Failsafe 的触发条件", "能（条件可控）", "卷 10 第 2 章详讲"],
        ["", "", ""],
        ["★ 传感器误差特性", "不能", "没有温漂、没有轴间不对齐、没有非线性"],
        ["★ 真实振动环境", "不能", "仿真加的是正弦，真机是宽频且随转速变"],
        ["★ 电机 / 螺旋桨动力学", "不能", "仿真用一阶模型，真机有死区、迟滞、形变"],
        ["★ 电源系统", "不能", "电压跌落、电流冲击、电池内阻"],
        ["★ 磁干扰", "近似不能", "电机电流产生的磁场很难建模"],
        ["★ 气动 / 地面效应", "不能", "近地悬停的吸地效应、风场湍流"],
        ["★ GPS 真实失效模式", "不能", "多径、遮挡、电离层、欺骗"],
        ["★ 结构共振", "不能", "机架/桨的共振频率会与滤波器互相影响"],
        ["", "", ""],
        ["★★ 真实负载下的时序抖动", "不能", "仿真里的 CPU 负载是假的"],
    ], ["项目", "SITL 能不能测", "原因"], width=[30, 14, 32])

    sub("② 为什么：仿真里没有的三类东西")

    print()
    wrap("上面清单里最有价值的不是能不能，而是【为什么不能】。"
         "归一下类，只有三类：")

    print()
    print("  第一类：【未建模的物理】")
    print("      仿真器只算它知道的物理。Gazebo 会算刚体动力学、会算接触力，")
    print("      但它【不知道】你的机架是哪块碳纤维板、桨叶在高速下会形变多少。")
    print("      ★ 典型症状：仿真里一切正常，真机上电机一转到某个转速就开始抖 ——")
    print("        因为那个转速激起了机架共振，而仿真里没有共振。")
    print()
    print("  第二类：【被简化成模型的物理】")
    print("      仿真里用一阶模型近似的东西，真机上是高阶的。")
    print("      比如电机：仿真里 thrust = k * pwm，真机上有：")
    print("        · 死区（pwm 低于某值完全不转）")
    print("        · 迟滞（升速和降速的曲线不一样）")
    print("        · 响应延迟（电流建立需要时间）")
    print("        · 桨叶在不同攻角下效率不同")
    print("      ★ 典型症状：仿真里参数调到完美，真机上超调或振荡。")
    print()
    print("  第三类：【真实世界的不确定性】")
    print("      仿真里同一个场景跑两次，结果完全一样（种子固定）。")
    print("      真机上同一个场景跑两次，结果从来不一样。")
    print("      ★ 所以【仿真里跑通一次不算数】——")
    print("        要跑很多次、每次换不同的随机种子和初始条件。")

    sub("③ ★★ 定量演示：延迟是怎么毁掉一个完美控制器的")

    print()
    wrap("这是仿真里调好了、真机上振荡最常见的单一原因，"
         "而且它是【纯数学】的，可以在 numpy 里算得清清楚楚。")

    print()
    print("  被控对象（角速率环的简化模型，两个惯性环节 + 一个固有延迟）：")
    print("      G(s) = K / ((tau1*s + 1)*(tau2*s + 1)) * exp(-s*T0)")
    print("      K    = 1.0      （归一化：单位舵量产生单位角速率）")
    print("      tau1 = 0.050 s  （机身 + 电机的时间常数）")
    print("      tau2 = 0.015 s  （传感器 PT1 滤波的时间常数）")
    print("      T0   = 0.008 s  （【仿真里就有】的那部分延迟：采样 + 计算）")
    print()
    print("  控制器：")
    print("      C(s) = Kp + Ki/s，   Kp = 3.0，  Ki = 4.0")
    print()
    print("  ★ 仿真的问题在于：它通常把一个【额外的】纯延迟环节")
    print("      P(s) = exp(-s*T_extra)")
    print("    当成 T_extra = 0。而真机上，T_extra 来自：")
    print("      · 传感器采样与低通滤波   （群延迟 2 到 6 ms）")
    print("      · 数据传输（uORB / 总线） （1 到 3 ms）")
    print("      · 电机 + 桨的响应         （3 到 12 ms）")
    print("    合计 6 到 20 ms —— 在 400 Hz（2.5 ms）的循环里，")
    print("    这相当于【2 到 8 个循环周期】的滞后。")

    print()
    print("  下面扫描【额外】延迟 T_extra，计算闭环的相位裕度（Phase Margin）。")
    print("  相位裕度的含义：在穿越频率处，还能再多滞后多少度才失稳。")
    print("  ★ 经验判据：PM < 30 度就开始难飞；PM < 15 度基本必炸。")

    Kp = 3.0
    Ki = 4.0
    K = 1.0
    tau1 = 0.050
    tau2 = 0.015
    T0 = 0.008

    w = np.logspace(-1, 3.4, 6000)
    s = 1j * w

    def loop_gain(t_extra):
        G = K / ((1.0 + s * tau1) * (1.0 + s * tau2)) * np.exp(-s * T0)
        C = Kp + Ki / s
        delay = np.exp(-s * t_extra)
        return C * G * delay

    def margins(t_extra):
        """返回 (穿越频率, 相位裕度[度], 增益裕度)"""
        L = loop_gain(t_extra)
        mag = np.abs(L)
        phase_deg = np.degrees(np.unwrap(np.angle(L)))
        idx = np.where((mag[:-1] >= 1.0) & (mag[1:] < 1.0))[0]
        if len(idx) == 0:
            return None, None, None
        i = idx[0]
        wc = float(np.interp(1.0, [mag[i + 1], mag[i]], [w[i + 1], w[i]]))
        ph = float(np.interp(math.log(wc), np.log(w), phase_deg))
        pm = 180.0 + ph
        idx2 = np.where((phase_deg[:-1] > -180.0) & (phase_deg[1:] <= -180.0))[0]
        if len(idx2) == 0:
            gm = float("inf")
        else:
            j = idx2[0]
            gm = 1.0 / float(np.interp(-180.0,
                                       [phase_deg[j], phase_deg[j + 1]],
                                       [mag[j], mag[j + 1]]))
        return wc, pm, gm

    tds = [0.000, 0.002, 0.004, 0.006, 0.008, 0.010,
           0.015, 0.020, 0.025, 0.030]

    def judge_of(pm):
        if pm is None:
            return "不开环"
        if pm >= 60.0:
            return "良好"
        if pm >= 45.0:
            return "可以接受"
        if pm >= 30.0:
            return "开始难飞"
        if pm >= 15.0:
            return "★ 危险"
        return "★★ 必炸"

    rows = []
    first_pm = None
    for td in tds:
        wc, pm, gm = margins(td)
        if pm is None:
            rows.append(["%5.1f" % (td * 1000), "-", "-", "-", "不开环"])
            continue
        if first_pm is None:
            first_pm = pm
        rows.append(["%5.1f" % (td * 1000),
                     "%.1f" % float(wc),
                     "%.1f" % pm,
                     ("%.2f" % gm) if gm != float("inf") else "inf",
                     judge_of(pm)])

    tab(rows, ["T_extra(ms)", "穿越频率(rad/s)", "相位裕度(度)",
               "增益裕度", "判据"],
        width=[12, 15, 12, 10, 10])

    # 从表里【算】出下降速率，而不是凭印象写
    wc0, pm0, _ = margins(0.0)
    wc1, pm1, _ = margins(0.010)
    slope = (pm0 - pm1) / 10.0
    pm_at_20 = margins(0.020)[1]

    print()
    print("  ★ 穿越频率 = %.1f rad/s = %.2f Hz；T_extra=0 时相位裕度 = %.1f 度。"
          % (wc0, wc0 / (2 * math.pi), pm0))
    print("  ★ 延迟每增加 1 ms，相位裕度下降 %.2f 度。" % slope)
    print("  ★ 到 T_extra = 20 ms 时，相位裕度只剩 %.1f 度。" % pm_at_20)

    print()
    wrap("★★ 读这张表：")
    print()
    for sline in [
        "① 相位裕度的下降几乎是【线性】的 —— 因为 exp(-s*T) 在穿越频率处",
        "   贡献的相位就是 wc*T（弧度），延迟每多 1 ms 就多滞后 wc*0.001 弧度。",
        "   本控制器算出来是 %.2f 度/ms，和表里完全对得上。" % slope,
        "",
        "② 仿真里 T_extra = 0，PM = %.1f 度（判据：%s）；"
        % (pm0, judge_of(pm0)),
        "   真机上 T_extra = 10 ms，PM = %.1f 度（判据：%s）；"
        % (pm1, judge_of(pm1)),
        "   到 20 ms 就只剩 %.1f 度，判据已经是【%s】。"
        % (pm_at_20, judge_of(pm_at_20)),
        "   ★ 而 10 到 20 ms 在真机上一点都不夸张：一个 50 Hz 的低通滤波",
        "     的群延迟就有 3 ms，电机响应再算 5 到 10 ms，轻松就超了。",
        "",
        "③ 【穿越频率越高，延迟越致命】。",
        "   这是同一个公式 wc*T 的直接推论：想飞得更敏捷（提高 wc），",
        "   就必须先把延迟压下去，否则相位裕度是【双重】损失的。",
        "   ★ 所以第 8 章调参的顺序是：先降延迟（换滤波器、提循环率），",
        "     再提增益 —— 顺序反了会一直振荡，而且怎么调都调不好。",
    ]:
        print("    " + sline)

    sub("④ 所以 SITL 到底该怎么用")

    print()
    for s in [
        "★★ 正确的用法：把 SITL 当【回归测试】，不当【性能验证】。",
        "",
        "  能做：",
        "    · 每次改代码后跑一遍自动测试，确认没有把原来能飞的功能改坏",
        "      （ArduPilot 有 Tools/autotest/arducopter.py，几千个测试用例，",
        "        就是干这个的 —— 见里面 test_ 开头的函数）",
        "    · 验证新写的状态机 / 参数逻辑的所有分支",
        "    · 在【危险场景】里练手：GPS 丢失、电池告警、失控返航",
        "      —— 这些在真机上试一次可能就摔了，在 SITL 里试一百次都不要钱",
        "    · 复现别人报告的问题（日志 + 参数 + 同一版本固件）",
        "",
        "  不能做：",
        "    · 用 SITL 调 PID 然后直接上真机（延迟和振动都没建模）",
        "    · 用 SITL 判断振动大不大（仿真里没有真实振动）",
        "    · 用 SITL 验证滤波器截止频率（输入信号本身就是干净的）",
        "    · 用 SITL 判断续航多久（电池模型是假的）",
    ]:
        print("    " + s)

    sub("⑤ ★★ 一句话收尾")

    print()
    print("      SITL 最大的价值不是省一架飞机，")
    print("      而是【让每一次实验都可复现】——")
    print("      同一个种子、同一份参数、同一个版本，结果永远一样。")
    print()
    print("      而它的最大陷阱也正是这一点：")
    print("      真实世界【不接受复现】，它每次都给你一个新的随机数。")
    print()
    wrap("★ 下一课（日志分析与 FailSafe）会讲怎么用【真机的日志】"
         "去弥补 SITL 测不了的那部分 —— 因为振动、延迟、传感器误差，"
         "最后都会【留在日志里】。")
    print()
    wrap("★ 现在就想去看看真机日志长什么样：")
    print("       python code/lesson23b_real_log.py")
    wrap("  它会读本机的 53 份 .ulg 真机日志，把振动 RMS、FFT 主峰、"
         "GPS 精度、EKF 检验比这些指标【真算一遍】并画图。"
         "跑不了 SITL 的时候，这就是离真机最近的那条路。")

    print()
    print("  " + "-" * (W - 4))
    print("  四课收束：")
    print("      卷 8 第 1 章  PX4 怎么组织代码       -> 模块 + uORB")
    print("      卷 8 第 2 章  ArduPilot 怎么组织代码 -> 任务表 + 库")
    print("      本章  怎么验证（本章）       -> SITL（能测逻辑，测不了物理）")
    print("      卷 10 第 2 章  怎么发现真机的问题     -> 日志 + FailSafe")
    print("  " + "-" * (W - 4))


# ============================================================================
#  入口
# ============================================================================

def main():
    args = sys.argv[1:]
    if not args:
        which = ["1", "2", "3", "4"]
    else:
        which = args

    table = {"1": exp1, "2": exp2, "3": exp3, "4": exp4}
    for a in which:
        if a not in table:
            print("用法：python lesson23_sitl.py [1|2|3|4]（不带参数 = 全部）")
            return 1
        table[a]()

    print()
    print("=" * W)
    print("  完成。")
    print("=" * W)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
