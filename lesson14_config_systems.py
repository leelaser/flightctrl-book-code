#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 5 第 5 章配套实验：配置系统与构建

    实验 1  NuttX 的 Kconfig（规模 + 依赖语法）
    实验 2  ArduPilot 的 hwdef.dat（生成器的校验能力）
    实验 3  PX4 的两层配置
    实验 4  ★★ 校验能力对照 + 实测「依赖默认值」的风险

用法：
    python lesson14_config_systems.py 1
    python lesson14_config_systems.py 2
    python lesson14_config_systems.py 3
    python lesson14_config_systems.py 4
    python lesson14_config_systems.py        # 全部

【本脚本的诚实声明】

实验 1/2/3 是【读文件】。
实验 4 的「依赖默认值」部分是【静态分析】——
它统计「defconfig 里没写、但 Kconfig 给了非 n 默认值」的符号数量，
这【证明不了】这些符号将来会变，只能证明【它们的值不在这块板子的控制之下】。

源码版本：
    ChibiOS 6a85082c / ArduPilot 4.5.7 / PX4 1.15.x + 其打包的 NuttX
"""

import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))   # 工作区根

APM_ROOT = os.path.join(ROOT, "APM", "ardupilot-Copter-4.5.7",
                        "ardupilot-Copter-4.5.7")
PX4_ROOT = os.path.join(ROOT, "PX4", "PX4-Autopilot")

HWDEF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS", "hwdef")
HWDEF_PY = os.path.join(HWDEF, "scripts", "chibios_hwdef.py")

NUTTX = os.path.join(PX4_ROOT, "platforms", "nuttx", "NuttX", "nuttx")
PX4_BOARD_DIR = os.path.join(PX4_ROOT, "boards", "px4", "fmu-v6x")
F_PX4BOARD = os.path.join(PX4_BOARD_DIR, "default.px4board")
F_DEFCONFIG = os.path.join(PX4_BOARD_DIR, "nuttx-config", "nsh", "defconfig")
F_BOARD_KCONFIG = os.path.join(PX4_BOARD_DIR, "nuttx-config", "Kconfig")
F_SCHED_KCONFIG = os.path.join(NUTTX, "sched", "Kconfig")


def read_lines(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().split("\n")
    except OSError:
        return None


def count_lines(path):
    """★ 只数换行符，不 +1。

    踩过的坑：read_lines() 返回 split(chr(10))，长度是【换行符数 + 1】。
    用它当行数会【每个文件多算 1 行】——
    571 个 Kconfig 文件正好多算 571 行
    （131,001 变成 131,572），而差值恰好等于文件数，很难看出来。

    ⇒ 要行数就用 count(chr(10))，和 wc -l 对齐。
    """
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().count("\n")
    except OSError:
        return 0


def read_text(path):
    lines = read_lines(path)
    return None if lines is None else "\n".join(lines)


def show(path, lo, hi, label=""):
    lines = read_lines(path)
    if not lines:
        print("    [读不到 %s]" % os.path.basename(path))
        return False
    print("    ┌─ %s:%d-%d%s"
          % (os.path.basename(path), lo, hi, ("  " + label) if label else ""))
    for i in range(lo, min(hi, len(lines)) + 1):
        print("    │ %4d  %s" % (i, lines[i - 1]))
    print("    └─")
    return True


def find_line(path, pattern, start=1):
    lines = read_lines(path)
    if not lines:
        return None
    rx = re.compile(pattern)
    for i in range(start - 1, len(lines)):
        if rx.search(lines[i]):
            return i + 1
    return None


def hr(ch="=", n=74):
    print(ch * n)


def head(title):
    print()
    hr()
    print("  " + title)
    hr()


def walk_files(root, name="Kconfig"):
    out = []
    for dirpath, _d, filenames in os.walk(root):
        for fn in filenames:
            if fn == name:
                out.append(os.path.join(dirpath, fn))
    return out


# ===========================================================================
# 实验 1：NuttX 的 Kconfig
# ===========================================================================

def exp1():
    head("实验 1：NuttX 的 Kconfig")

    print("""
  NuttX 用 Kconfig 做配置系统。这个实验量它的规模，
  并从源码里找出 Kconfig 的三个关键语法。

  ★ 要回答的问题：NuttX 的配置系统有多大？为什么这么大？
""")

    print()
    print("  ── ① 规模 ──")
    print()

    if not os.path.isdir(NUTTX):
        print("    [读不到 %s]" % NUTTX)
        print("    （PX4 的 NuttX 树不在预期位置）")
        return

    files = walk_files(NUTTX, "Kconfig")
    total = sum(count_lines(f) for f in files)
    print("    路径: %s" % NUTTX)
    print()
    print("    Kconfig 文件数:  %d" % len(files))
    print("    Kconfig 总行数:  %s" % "{:,}".format(total))
    print()

    # 对比：整个 NuttX 树的 .c 文件
    all_c = 0
    all_c_files = 0
    for dirpath, _d, filenames in os.walk(NUTTX):
        for fn in filenames:
            if fn.endswith(".c"):
                all_c_files += 1
                all_c += count_lines(os.path.join(dirpath, fn))
    print("    对比：整个 NuttX 树的 .c")
    print("        %d 个文件，%s 行" % (all_c_files, "{:,}".format(all_c)))
    print()
    print("    ⇒ 配置系统占全部 C 代码的 %.1f%%。"
          % (100.0 * total / max(1, all_c)))
    print()
    print("""    ★ 也就是说：NuttX 用了【相当于全部 C 代码 4~5% 的量】
      去描述「有哪些可配项、它们什么关系」。

      这不是浪费 —— 配置系统的规模正比于【配置空间】（多少项可配），
      代码的规模正比于【功能量】。

      NuttX 支持几十种架构、上百块板子 ⇒ 配置空间巨大，
      所以 4~5% 是合理的开销。

      ★ 而如果只有几块板子、几十个配置项，
        专门维护一套 Kconfig 就不划算了 ——
        这正是 ArduPilot 的选择（它用生成器代替）。
""")

    print("  ── ② Kconfig 的三个关键语法 ──")
    print()

    print("    【语法一】depends on —— 「我需要 X」")
    print()
    p = find_line(F_BOARD_KCONFIG, r"depends on BOARD_HAS_PROBES")
    if p:
        show(F_BOARD_KCONFIG, max(1, p - 8), p + 2,
             "★ PX4 板级 Kconfig 里最短的例子")

    print("""
    ⇒ BOARD_USE_PROBES 只在 BOARD_HAS_PROBES 为 y 时【可见】。
      没开的话，menuconfig 里根本看不到这个选项。

    【语法二】select —— 「我要 X」
""")

    p = find_line(F_SCHED_KCONFIG, r"config SCHED_SPORADIC")
    if p:
        show(F_SCHED_KCONFIG, p, p + 5, "★ select 自动带上依赖项")

    print("""
    ⇒ 选 SCHED_SPORADIC 就自动开了 SCHED_SUSPENDSCHEDULER
      和 SCHED_RESUMESCHEDULER。

    ★★ depends on 和 select 的区别：

        depends on X    「我需要 X」—— X 没开，我不可选
                        （用户【看得见】这个约束）

        select X        「我要 X」—— 我开了就强制开 X
                        （用户【看不见】这个约束，它被自动满足了）

    ⚠ select 是 Kconfig 里最有争议的关键字 ——
      它【绕过】了依赖检查。Linux 内核文档专门警告：
      不要 select 一个 depends on 别的东西的符号。

    【语法三】类型 —— bool / int / string
""")

    p = find_line(F_SCHED_KCONFIG, r"^config RR_INTERVAL")
    if p:
        show(F_SCHED_KCONFIG, p, p + 7, "★ RR_INTERVAL 是 int，不是 bool")

    print("""
    ★★ 注意：它是 int，不是 bool！

      「开不开时间片轮转」在 NuttX 里不是开关，是【一个数值】。
      0 = 关，正数 = 开（开多少毫秒的片）。

      ⇒ 后果：menuconfig 里看不到复选框，只有一个填数字的输入框。
        用户得读 ---help--- 才知道 0 是关。

      ⇒ 而紧接着的 SCHED_SPORADIC 用的是 bool。

    ★ 这说明了配置系统的一个【边界】：
      Kconfig 能表达「这是个整数，默认 0」，
      但表达不了「0 表示禁用」这个【语义】——
      语义只能写在 help 里，而 help 不会被校验。
""")
    print()


# ===========================================================================
# 实验 2：ArduPilot 的 hwdef.dat
# ===========================================================================

def exp2():
    head("实验 2：ArduPilot 的 hwdef.dat")

    print("""
  ArduPilot 没有配置系统那一层 —— 它用【一个 Python 生成器】。
  这个实验看它长什么样、有多少块板子、以及它的校验能力。
""")

    print()
    print("  ── ① 板子数量 ──")
    print()

    if not os.path.isdir(HWDEF):
        print("    [读不到 %s]" % HWDEF)
        return

    boards = sorted(d for d in os.listdir(HWDEF)
                    if os.path.isfile(os.path.join(HWDEF, d, "hwdef.dat")))
    bl_boards = sorted(d for d in os.listdir(HWDEF)
                       if os.path.isfile(os.path.join(HWDEF, d, "hwdef-bl.dat")))
    print("    hwdef/<board>/hwdef.dat    : %d 块" % len(boards))
    print("    hwdef/<board>/hwdef-bl.dat : %d 块（bootloader 变体）"
          % len(bl_boards))
    print()

    print("  ── ② 一份 hwdef.dat 长什么样 ──")
    print()

    sample = os.path.join(HWDEF, "MatekH743", "hwdef.dat")
    if os.path.isfile(sample):
        lines = read_lines(sample)
        print("    %s（%d 行）" % (sample.replace(ROOT, "..."), len(lines) - 1))
        print()
        for i, line in enumerate(lines[:22], 1):
            if line.strip():
                print("        %3d  %s" % (i, line))

    print("""
    ⇒ 这是一个【裸文本 DSL】：每行一个事实。
        引脚：  PD10  PINIO1 OUTPUT LOW
        参数：  FLASH_SIZE_KB 2048
        宏：    define HAL_LED_ON 0
        包含：  include ../common/xxx.dat

    ★ 注意：没有类型声明、没有依赖声明、没有默认值继承。
      和 Kconfig 相比，它少了整整一套"配置系统"。
""")

    print("  ── ③ 那配错了怎么办？—— 生成器里的校验 ──")
    print()
    print("    实测 %s 里所有 raise 语句：" % os.path.basename(HWDEF_PY))
    print()

    lines = read_lines(HWDEF_PY) or []
    checks = []
    for i, line in enumerate(lines, 1):
        s = line.strip()
        if s.startswith("raise ") or s.startswith("self.error("):
            # 取错误消息
            m = re.search(r'"([^"]{8,90})"', s)
            msg = m.group(1) if m else s[:60]
            checks.append((i, msg))

    print("    %-6s %s" % ("行号", "校验内容"))
    print("    " + "-" * 68)
    for i, msg in checks[:20]:
        print("    %-6d %s" % (i, msg[:60]))
    print()
    print("    共 %d 处校验。" % len(checks))
    print()
    print("""    ★ 这些校验是【手写的 if 判断】—— 一条一条加上去的。

      对比 Kconfig：
        Kconfig      声明式（depends on / select）
        hwdef 生成器  命令式（if + raise）

      各有强项：
        Kconfig 强在【关系】—— A 依赖 B、选了 C 就带 D
        生成器强在【任意逻辑】—— 比如「include 不能成环」
                                  （Kconfig 表达不了这个概念）

      ⚠ 但「任意逻辑」是【手工】加上去的：
        没想到的情况就不会被检查。
""")
    print()


# ===========================================================================
# 实验 3：PX4 的两层配置
# ===========================================================================

def exp3():
    head("实验 3：PX4 的两层配置")

    print("""
  PX4 有两套配置，分开管理。这个实验看清它们的分工。
""")

    print()
    print("  ── ① 第一层：PX4 自己的配置 ──")
    print()

    lines = read_lines(F_PX4BOARD)
    if lines:
        print("    %s（%d 行）"
              % (F_PX4BOARD.replace(ROOT, "..."), len(lines)))
        print()
        for i, line in enumerate(lines[:16], 1):
            if line.strip():
                print("        %3d  %s" % (i, line))
        print("        ...")

        # 分类统计
        cats = {}
        for line in lines:
            m = re.match(r"^CONFIG_(\w+?)_", line)
            if m:
                cats[m.group(1)] = cats.get(m.group(1), 0) + 1
        print()
        print("    按前缀分类：")
        for c in sorted(cats, key=lambda x: -cats[x])[:8]:
            print("        CONFIG_%-22s %3d 项" % (c + "_*", cats[c]))

    print()
    print("""    ⇒ 这一层管的是「用哪些驱动、串口怎么映射、板载什么传感器」。
      它是【PX4 开发者】关心的东西。
""")

    print("  ── ② 第二层：NuttX 侧的配置 ──")
    print()

    lines = read_lines(F_DEFCONFIG)
    if lines:
        print("    %s（%d 行）"
              % (F_DEFCONFIG.replace(ROOT, "..."), len(lines)))
        print()
        # 那三句注释最有信息量
        for i, line in enumerate(lines[:12], 1):
            print("        %3d  %s" % (i, line))

        print()
        print("    ★★ 那三句话说明了整个机制：")
        print()
        print("        ① This file is autogenerated: PLEASE DO NOT EDIT IT.")
        print("           ⇒ 它不是手写的，是 make savedefconfig 从 .config 生成的")
        print()
        print("        ② You can use make menuconfig to make any modifications")
        print("           to the installed .config file.")
        print("           ⇒ 改配置的正确方式是改 .config，不是改 defconfig")
        print()
        print("        ③ You can then do make savedefconfig to generate a new")
        print("           defconfig file that includes your modifications.")
        print("           ⇒ .config → defconfig 的方向是【缩减】")

        # 统计 "is not set" 的行
        notset = sum(1 for l in lines if l.startswith("# CONFIG_") and
                     l.rstrip().endswith("is not set"))
        set_to = sum(1 for l in lines if re.match(r"^CONFIG_\w+=", l))
        print()
        print("    实测这份 defconfig：")
        print("        CONFIG_X=y 或 =value 的行:  %d" % set_to)
        print("        # CONFIG_X is not set   的行:  %d" % notset)
        print()
        print("""    ⇒ 两种行都是【显式设置】：
          · CONFIG_X=y           显式打开
          · # CONFIG_X is not set 显式关闭（bool 默认 y 时要关掉必须写）

      ★ 而【没出现的项】= 用 Kconfig 当前的默认值。
        这就是「差量配置」。
""")

    print("  ── ③ 板级额外选项挂在哪 ──")
    print()
    if os.path.isfile(F_BOARD_KCONFIG):
        lines = read_lines(F_BOARD_KCONFIG)
        print("    %s（%d 行）"
              % (F_BOARD_KCONFIG.replace(ROOT, "..."), len(lines)))
        print()
        print("    ⇒ 只有 17 行 —— 它是挂到 NuttX 那 571 个 Kconfig 上的")
        print("      板级扩展。NuttX 的顶层 Kconfig 会 source 它。")

    print()
    print("""    ★ 为什么分两层？因为这是两个专业领域：

        PX4 层   → 用哪些驱动、串口映射、板载传感器
                   【PX4 开发者】懂
        NuttX 层 → 内核配置、外设驱动、文件系统
                   【NuttX 懂的人】懂

      ⇒ 两套配置语言，两套人维护。
""")
    print()


# ===========================================================================
# 实验 4：校验能力 + 实测「依赖默认值」的风险
# ===========================================================================

def parse_kconfig_defaults(path):
    """从一个 Kconfig 文件里抽出「有非 n 默认值」的符号。

    只处理最常见的形态：
        config NAME
            bool "..."  / int "..." / string "..."
            default y   / default 0 / default "..."
    """
    lines = read_lines(path)
    if not lines:
        return {}

    out = {}
    cur = None
    for i, line in enumerate(lines, 1):
        s = line.strip()
        if s.startswith("#"):
            continue
        m = re.match(r"^config\s+(\w+)", s)
        if m:
            cur = m.group(1)
            continue
        m = re.match(r"^default\s+(.*?)\s*$", s)
        if m and cur:
            val = m.group(1)
            # 只记第一个 default
            out.setdefault(cur, (val, i))
    return out


def exp4():
    head("实验 4：★★ 校验能力对照 + 实测「依赖默认值」的风险")

    print("""
  这一节是本章的重点。

  第一部分：把三家的校验能力并排列出来。
  第二部分：★★ 实测「差量配置」的一个后果 ——
            【板子的行为有一部分不在它自己的控制之下】。
""")

    print()
    hr("-")
    print("  ① 三家的校验能力对照")
    hr("-")
    print()

    print("    %-14s %-26s %-28s %s"
          % ("阶段", "NuttX（Kconfig）", "ArduPilot（生成器）", "PX4"))
    print("    " + "-" * 92)
    ROWS = [
        ("写配置时", "✅ menuconfig 只显示可见项", "❌ 无提示", "✅ 同 NuttX"),
        ("生成时", "✅ 类型 + 依赖检查", "✅ 任意 Python 校验", "✅ 两者都有"),
        ("编译时", "✅ #error", "✅ 同", "✅ 同"),
        ("运行前", "❌", "❌", "❌"),
        ("运行中", "❌", "❌", "❌"),
    ]
    for r in ROWS:
        print("    %-14s %-26s %-28s %s" % r)

    print()
    print("""    ★ 三个观察：

      ① menuconfig 的价值是「把不可选项藏起来」。
         没有它，用户会看到一堆互相冲突的选项
         （ArduPilot 的 hwdef.dat 就是这样）。

      ② ArduPilot 的「任意 Python 校验」在某些方面【更强】——
         比如 include loop（循环 include），Kconfig 根本表达不了。

      ③ 但三家的边界都一样：配置系统管不了「语义正确性」。
""")

    print()
    hr("-")
    print("  ② ★★ 实测：板子有多少配置「不在自己手里」")
    hr("-")
    print()

    print("""    原理：defconfig 只写「和默认值不同的项」。
          那么【没写的项】的值，取决于 Kconfig 当前的默认值。

    做法：拿 NuttX 的 sched/Kconfig，找出所有「默认值不是 n」的符号，
          再看 PX4 的 fmu-v6x defconfig 里提了几个。
""")

    if not os.path.isfile(F_SCHED_KCONFIG) or not os.path.isfile(F_DEFCONFIG):
        print("    [读不到所需的 Kconfig / defconfig，跳过]")
        print()
        return

    defaults = parse_kconfig_defaults(F_SCHED_KCONFIG)
    # 只关心「默认不是 n / 空」的
    nonzero = {k: v for k, v in defaults.items()
               if v[0].lower() not in ("n", "0", '""', "''")}

    defcfg = read_text(F_DEFCONFIG) or ""
    mentioned = set()
    for line in defcfg.split("\n"):
        m = re.match(r"^CONFIG_(\w+)=", line)
        if m:
            mentioned.add(m.group(1))
        m = re.match(r"^#\s*CONFIG_(\w+)\s+is not set", line)
        if m:
            mentioned.add(m.group(1))

    in_sched = {k: v for k, v in nonzero.items()
                if k not in mentioned}

    print()
    print("    sched/Kconfig 里有非 n 默认值的符号: %d 个"
          % len(nonzero))
    print("    其中 fmu-v6x 的 defconfig 【没有提到】的: %d 个"
          % len(in_sched))
    print()
    print("    ⇒ 这 %d 个符号的值，完全由 Kconfig 的默认值决定。"
          % len(in_sched))
    print()

    print("    抽几个看看（前 15 个）：")
    print()
    print("    %-28s %-12s %s" % ("符号", "Kconfig 默认", "Kconfig 行号"))
    print("    " + "-" * 56)
    for k in sorted(in_sched)[:15]:
        val, lineno = in_sched[k]
        print("    %-28s %-12s %d" % (k, val[:12], lineno))

    print()
    print("""    ⚠⚠ 【分寸】先说清楚：这【不是】说"有 64 个就危险"。

      大部分默认值是【刻意设计成默认】的：
        · DEFAULT_TASK_STACKSIZE  默认 2048 —— 板子不关心就不写
        · INIT_FILEPATH           默认 "/bin/init" —— 所有板子都一样
        · DISABLE_POSIX_TIMERS    默认 n —— 默认开，符合大多数板子

      这些用默认值是【正确的做法】——
      否则每个 defconfig 都要抄一遍几百行。

    ★★ 真正要注意的是【默认值会变的那些】。

      那些符号的值【不在板子的控制之下】：

        defconfig 里没写  ⇒  值 = Kconfig 当前的 default
        上游改 default    ⇒  板子行为变了，而板子【什么都没改】

    ⚠ 而且【没有任何提示】—— 没有编译错误、没有警告。

    ⇒ 这不是 bug，是「差量配置」这种设计的【必然】。
      NuttX / ArduPilot / PX4 / Linux 内核 —— 全都这样。

    ★ 所以「差量配置」有一个隐含的【契约】：

        板子的作者必须知道：
        「哪些默认值是我依赖的，哪些我只是懒得写」

        依赖的那些，按道理应该【显式写进 defconfig】——
        即使它和默认值一样。

      ⚠ 但没有任何机制强制这件事，
        也没有任何机制提醒作者"这个默认值变了"。

    ★ 一般化的结论：

        「默认值」不是「文档里的建议」——
        它是【配置的实际来源之一】。

        所以改一个 Kconfig 的 default，
        等于同时改了一批板子的配置 —— 而它们都不知道。
""")

    print()
    hr("-")
    print("  ③ ★ 配置系统能保证什么")
    hr("-")
    print()
    print("    %-30s %s" % ("能保证", "不能保证"))
    print("    " + "-" * 62)
    PAIRS = [
        ("语法正确", "★ 语义正确"),
        ("类型正确（bool/int/string）", "★ 值适合这块硬件"),
        ("依赖满足（如果声明了）", "★ 这个组合能工作"),
        ("有个值（从默认值来）", "★ 这个值是你要的"),
    ]
    for a, b in PAIRS:
        print("    %-30s %s" % (a, b))
    print()
    print("""    ⇒ 校验能力的上限，在于【能表达多少语义】。

      而 Kconfig / hwdef.dat 都不是为表达语义设计的 ——
      它们是「配置分发」工具，不是「配置验证」工具。

    ⚠ 所以「我们用了 Kconfig，所以配置是安全的」这个推论是错的。
      它只说明配置能被正确地【分发和展开】。

    ★ 实证：《NuttX 教程》卷 6 第 9 章发现的
      17/26 块 PX4 板卡的 RCC RGE 配置不匹配 ——
      语法合法（Kconfig 检查通过）、编译通过（#error 没触发），
      但【硬件上是错的】。
""")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("NuttX 的 Kconfig", exp1),
    "2": ("ArduPilot 的 hwdef.dat", exp2),
    "3": ("PX4 的两层配置", exp3),
    "4": ("★★ 校验能力 + 默认值风险", exp4),
}


def main():
    print()
    hr("#")
    print("#  本章：配置系统与构建")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(APM_ROOT) or not os.path.isdir(PX4_ROOT):
        print()
        print("  [错误] 找不到源码：")
        print("         ArduPilot: %s" % APM_ROOT)
        print("         PX4:       %s" % PX4_ROOT)
        print()
        return 1

    arg = sys.argv[1] if len(sys.argv) > 1 else None

    if arg is None:
        for key in sorted(EXPERIMENTS):
            EXPERIMENTS[key][1]()
    elif arg in EXPERIMENTS:
        EXPERIMENTS[arg][1]()
    else:
        print()
        print("  未知实验编号: %s" % arg)
        print("  可用: %s" % ", ".join(sorted(EXPERIMENTS)))
        print()
        return 1

    print()
    hr("#")
    print("#  实验结束")
    hr("#")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
