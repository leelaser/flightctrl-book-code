#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 22.5 周 · 06-软件架构 6.3 —— 板级支持包（BSP）与硬件抽象

    实验 1  ★    default.px4board 逐行分类：四个命名空间各管什么
    实验 2  ★★   读者那块板 vs 官方 fmu-v6x：移植到底改了多少行
    实验 3  ★★   board_config.h 引脚表 + spi.cpp 总线表（引脚是怎么落到外设的）
    实验 4  ★★   一行 CONFIG_BOARD_SERIAL_TEL1 在软件里变成了什么
    实验 5  ★★★  内存映射：从链接脚本算 bootloader / app / 参数区的大小

用法：
    python lesson29_bsp.py 1
    python lesson29_bsp.py 2
    python lesson29_bsp.py 3
    python lesson29_bsp.py 4
    python lesson29_bsp.py 5
    python lesson29_bsp.py            # 全部

【本脚本的原则】

  ★ 只用 Python 标准库 —— 不需要 numpy、不需要 PX4 工具链、不需要编译。
  它干的事只有一件：**把散在四五个文件里的板级配置读出来、
  拼成一张能对照的表**，让你看清「一个固件统治几十块板子」是怎么做到的。

  所以它不是仿真，是**索引器**：
  每一个数字都直接来自你本机的那几个文件，行号一并打印出来，可回查。

【两块板子】

  CUSTOM  你画的板子：硬件/.../fmu-v6x-qgbf-20251116/fmu-v6x-qgbf/
  OFFICIAL 上游原版：  PX4/PX4-Autopilot-1.15.4/.../boards/px4/fmu-v6x/

  实验 2 会把两份 default.px4board 做**分类差集**，然后回答一个问题：
  「把 PX4 移植到一块新板子上，最少要改几行？」
  答案会让你意外 —— 但它是从文件里数出来的，不是我说的。
"""

import os
import re
import sys

W = 74


# ===========================================================================
# 0. 找路径
# ===========================================================================
#
#   ★ 路径全部【推导】出来，不写死盘符 —— 换台机器也能跑。
#     推导链：本文件在 <教程根>/06-软件架构/code/ 下
#             → 教程根 = 上两级
#             → 项目根 = 教程根上一级 = D:/DEVLib/PX4APM
#
HERE = os.path.dirname(os.path.abspath(__file__))
TUTORIAL = os.path.dirname(os.path.dirname(HERE))
PROJECT = os.path.dirname(TUTORIAL)

PX4_ROOT = os.path.join(PROJECT, "PX4", "PX4-Autopilot-1.15.4",
                        "PX4-Autopilot-1.15.4")

HW_ROOT = os.path.join(PROJECT, "硬件")

# 读者那块板子的板名 —— 出现在目录名里，用它来定位
BOARD_NAME = "fmu-v6x-qgbf"

OFFICIAL_BOARD = os.path.join(PX4_ROOT, "boards", "px4", "fmu-v6x")


def find_custom_board():
    """在 硬件/ 下找读者那块板子的 BSP 目录。

    ★ 这里必须【搜索】而不是拼字符串 —— 目录名里带中文、带冒号、
      带空格（「版本V1.4 - 从零开始手搓无人机-B站：千古八方的玩具」），
      而且日期后缀（20251116）以后还会变。写死路径 = 半年后失效。
    """
    if not os.path.isdir(HW_ROOT):
        return None
    for dirpath, dirnames, _ in os.walk(HW_ROOT):
        # 板级目录实际在第 4 层：
        #   硬件/版本V1.4 - …/PX4-FMU-v6x/fmu-v6x-qgbf-20251116/fmu-v6x-qgbf/
        # 所以放宽到 6 层，同时跳过 ELRS 那种无关分支。
        depth = dirpath[len(HW_ROOT):].count(os.sep)
        if depth > 6:
            dirnames[:] = []
            continue
        dirnames[:] = [d for d in dirnames if "ELRS" not in d]
        if os.path.basename(dirpath) == BOARD_NAME:
            return dirpath
    return None


CUSTOM_BOARD = find_custom_board()


def hr(title=""):
    print()
    print("=" * W)
    if title:
        print("  " + title)
        print("=" * W)


def sub(title):
    print()
    print("-" * W)
    print("  " + title)
    print("-" * W)


def _w(s, n):
    """按【显示宽度】截断/补齐 —— 中日韩字符算 2 格。

    ★ 不这么做的话，含中文的表格会整列错位：
      len('传感器') == 3，但它在终端里占 6 格。
      这是本教程所有表格脚本的共同坑。
    """
    def width(t):
        return sum(2 if ord(c) > 0x2000 else 1 for c in t)

    out, acc = "", 0
    for ch in s:
        cw = 2 if ord(ch) > 0x2000 else 1
        if acc + cw > n:
            break
        out += ch
        acc += cw
    return out + " " * (n - acc)


def need_board():
    if CUSTOM_BOARD is None:
        print()
        print("  ⚠ 没找到读者的板级支持包目录。")
        print("    应该在：%s" % HW_ROOT)
        print("    下面某个子目录里名为 %s 的文件夹。" % BOARD_NAME)
        return False
    return True


def read_lines(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read().split("\n")
    except OSError:
        return []


def grep(path, pattern):
    """返回 [(行号, 行内容), ...]，行号从 1 开始 —— 和编辑器一致。"""
    rx = re.compile(pattern)
    return [(i + 1, ln) for i, ln in enumerate(read_lines(path))
            if rx.search(ln)]


# ===========================================================================
# 1. default.px4board 解析器
# ===========================================================================
#
#   .px4board 是 kconfig 的 defconfig 格式：
#       CONFIG_XXX=y            布尔打开
#       CONFIG_XXX=n            布尔关闭
#       CONFIG_XXX="字符串"      字符串值
#       # CONFIG_XXX is not set 布尔关闭的另一种写法
#
#   ★ 它【不是】Makefile，不是 CMake，也不是头文件 ——
#     它是 Kconfig 的输入，最终被 genconfig 生成 px4_boardconfig.h（C 头文件）。
#     这一条是本课最关键的心智模型：
#         .px4board  →  boardconfig  →  px4_boardconfig.h  →  C/C++ 里的 #if
#       三跳。少了任何一跳，你都会以为「改了配置但代码没反应」。
#
#   命名空间（按前缀）—— 决定这一行进哪个列表：
#       CONFIG_BOARD_*      板级标量/字符串（串口、以太网、保护模式…）
#       CONFIG_DRIVERS_*    驱动模块（会变成 add_subdirectory(src/drivers/...)）
#       CONFIG_MODULES_*    功能模块（会变成 add_subdirectory(src/modules/...)）
#       CONFIG_SYSTEMCMDS_* 命令行工具（nsh 里能敲的那些）
#       CONFIG_COMMON_*     功能开关（不是模块，是编译期能力）
#       CONFIG_USER_*       把模块丢到用户态跑（内存保护）
#       其它                ROMFSROOT / MAVLINK_DIALECT / ARCH_CHIP_* …
#
NS_TABLE = [
    ("CONFIG_BOARD_",      "BOARD_",      "板级标量/字符串", "串口映射、以太网、内存保护…"),
    ("CONFIG_DRIVERS_",    "DRIVERS_",    "驱动",           "传感器/总线驱动模块"),
    ("CONFIG_MODULES_",    "MODULES_",    "功能模块",       "ekf2 / commander / logger…"),
    ("CONFIG_SYSTEMCMDS_", "SYSTEMCMDS_", "命令行工具",     "nsh 里能敲的命令"),
    ("CONFIG_COMMON_",     "COMMON_",     "编译期能力",     "GPS/磁罗盘/光流等能力开关"),
    ("CONFIG_USER_",       "USER_",       "用户态模块",     "内存保护（CONFIG_BOARD_PROTECTED）"),
]


def classify(name):
    for prefix, _short, label, _desc in NS_TABLE:
        if name.startswith(prefix):
            return label
    return "其它"


def parse_px4board(path):
    """→ [(行号, 完整 key, 值, 是否打开), ...]

    值：
      'y' / 'n'  布尔
      '"..."'    字符串（引号保留，方便看清它确实是个字符串）
      None       `# CONFIG_X is not set` 这种
    """
    out = []
    for ln, text in enumerate(read_lines(path), 1):
        t = text.strip()
        if not t or t.startswith("#") and " is not set" not in t:
            continue
        m = re.match(r"^#\s*(CONFIG_[A-Za-z0-9_]+)\s+is not set\s*$", t)
        if m:
            out.append((ln, m.group(1), "n", False))
            continue
        m = re.match(r"^(CONFIG_[A-Za-z0-9_]+)=(.*)$", t)
        if m:
            key, val = m.group(1), m.group(2).strip()
            out.append((ln, key, val, val != "n"))
    return out


def show_tree(label, path, key):
    """按命名空间分类打印一份 .px4board。"""
    sub(label)
    if not os.path.isfile(path):
        print("  ⚠ 找不到：%s" % path)
        return None
    items = parse_px4board(path)
    print("  %s" % path)
    print("  共 %d 项配置（含 =n 的关闭项）" % len(items))

    buckets = {}
    for ln, name, val, on in items:
        buckets.setdefault(classify(name), []).append((ln, name, val, on))

    print()
    print("  %s %s %s %s" % (_w("命名空间", 14), _w("条数", 6),
                             _w("打开", 6), _w("对应软件里的什么", 40)))
    print("  " + "-" * 70)
    for prefix, _s, label, desc in NS_TABLE:
        got = buckets.get(label, [])
        if not got:
            continue
        n_on = sum(1 for _l, _n, _v, on in got if on)
        print("  %s %s %s %s" % (_w(label, 14), _w(str(len(got)), 6),
                                 _w(str(n_on), 6), _w(desc, 40)))
    other = buckets.get("其它", [])
    if other:
        print("  %s %s %s %s" % (_w("其它", 14), _w(str(len(other)), 6),
                                 _w(str(sum(1 for _l, _n, _v, o in other if o)), 6),
                                 _w("ROMFS/编译器/芯片系列等", 40)))
    return items


def exp1():
    hr("实验 1 ★ default.px4board 逐行分类：四个命名空间各管什么")

    print("""
  先记住这条链（本课最重要的一张图）：

      default.px4board            kconfig 的【输入】—— 人写的
            ↓  Tools/kconfig/merge_config.py
      boardconfig                 kconfig 的【合并结果】—— 和 {label}.px4board 合并
            ↓  genconfig --header-path px4_boardconfig.h
      px4_boardconfig.h           C 头文件 —— 代码里 #if defined(CONFIG_XXX) 用的就是它
            ↓  同时 cmake/kconfig.cmake 把每一行解析成 CMake 变量
      config_module_list          模块列表 → add_subdirectory(src/<模块>)

  ★ 所以「改了 .px4board 但代码没反应」这个经典问题，病因一定在这四跳的某一跳。
""")

    if not need_board():
        return
    show_tree("读者那块板子", os.path.join(CUSTOM_BOARD, "default.px4board"), None)
    show_tree("上游原版 fmu-v6x", os.path.join(OFFICIAL_BOARD, "default.px4board"), None)

    print()
    print("  ★ 观察：读者那块板子有 %d 项，官方有 %d 项。"
          % (len(parse_px4board(os.path.join(CUSTOM_BOARD, "default.px4board"))),
             len(parse_px4board(os.path.join(OFFICIAL_BOARD, "default.px4board")))))
    print("    数量接近是【故意的】—— 移植的起点是复制，不是从零写。")


# ===========================================================================
# 2. 两块板子的分类差集
# ===========================================================================
#
#   ★★ 这是本课最有信息量的一张表。
#
#   把差集按「为什么不一样」分成四类：
#       [串口]  改角色分配 —— 引脚布线不同，必须改
#       [驱动]  换了一颗芯片（或者不用某颗）—— 和原理图一一对应
#       [模块]  裁剪功能 —— 飞机不需要相机/以太网，删掉省 flash
#       [其它]  工具链、方言、芯片系列 —— 通常不动
#
#   分类之后你会发现：**必须改的只有第一类**。
#   第二、三类是「我选了不同的器件」，不是「移植必须做的」。
#
def why_diff(key):
    if key.startswith("CONFIG_BOARD_SERIAL"):
        return "串口角色"
    if key.startswith("CONFIG_DRIVERS_"):
        return "驱动增减"
    if key.startswith(("CONFIG_MODULES_", "CONFIG_COMMON_")):
        return "模块裁剪"
    if key.startswith("CONFIG_SYSTEMCMDS_"):
        return "工具增减"
    if key.startswith("CONFIG_BOARD_"):
        return "板级能力"
    return "其它"


def exp2():
    hr("实验 2 ★★ 读者那块板 vs 官方 fmu-v6x：移植到底改了多少行")

    if not need_board():
        return
    a_path = os.path.join(OFFICIAL_BOARD, "default.px4board")
    b_path = os.path.join(CUSTOM_BOARD, "default.px4board")

    a = {n: (ln, v) for ln, n, v, _o in parse_px4board(a_path)}
    b = {n: (ln, v) for ln, n, v, _o in parse_px4board(b_path)}

    only_a = sorted(set(a) - set(b))
    only_b = sorted(set(b) - set(a))
    changed = sorted(k for k in set(a) & set(b) if a[k][1] != b[k][1])

    print("  官方 default.px4board：%d 项" % len(a))
    print("  读者 default.px4board：%d 项" % len(b))
    print()
    print("  只在官方有（读者删掉/没开）：%d 项" % len(only_a))
    print("  只在读者有（读者新增）：      %d 项" % len(only_b))
    print("  两边都有但值不同：            %d 项" % len(changed))

    if only_a:
        sub("官方有、读者没有 —— 按「为什么」分类")
        groups = {}
        for k in only_a:
            groups.setdefault(why_diff(k), []).append(k)
        for g in sorted(groups, key=lambda g: -len(groups[g])):
            print()
            print("  【%s】%d 项" % (g, len(groups[g])))
            for k in groups[g]:
                print("      %s=%s   (官方 :%d)" % (k, a[k][1], a[k][0]))

    if only_b:
        sub("读者有、官方没有")
        for k in only_b:
            print("  %s=%s   (读者 :%d)" % (k, b[k][1], b[k][0]))

    if changed:
        sub("两边都有但值不同")
        for k in changed:
            print("  %s" % k)
            print("      官方 :%d  = %s" % (a[k][0], a[k][1]))
            print("      读者 :%d  = %s" % (b[k][0], b[k][1]))

    sub("★ 结论：把差异分三类 —— 只有第一类是「移植」")
    ser_a = [k for k in only_a if k.startswith("CONFIG_BOARD_SERIAL")]
    ser_b = [k for k in only_b if k.startswith("CONFIG_BOARD_SERIAL")]
    print("""
  【第一类 · 必改】串口角色 —— 由 PCB 走线唯一决定，改不了
      读者删掉：%s
      读者新增：%s
    ★ 为什么它「必改」：/dev/ttySx 到【物理连接器】的对应关系是画板子时
      就焊死的。写错了不会编译报错，只会「GPS 插上去没反应」——
      这是移植期最难查的一类 bug。

  【第二类 · 建议删】板子上根本没有那颗芯片
      PX4IO、Ethernet、相机捕获、4 颗用不上的 IMU、加热丝……
    ★ 不删也能编译过，但：① 白占 flash（本课 §6.3.6 会算这笔账）；
      ② init 脚本会去 start 一个不存在的设备，控制台一直喷错误，
      把真正的错误淹掉。

  【第三类 · 选配】产品功能 —— 和移植无关，纯粹是「这台机器要干什么」
      CRSF/SBUS 接收机、光流、PWM 输入、MFT_CFG……
    这些在官方板上加也一样要改，属于机型定制，不是移植工作量。

  ⇒ 移植的「必需工作量」只有：串口角色 + 引脚表 + 存储 + board_id。
    本课 §6.3.7 会把这条清单一条条列出来。
""" % ("、".join(ser_a) or "（无）", "、".join(ser_b) or "（无）"))


# ===========================================================================
# 3. board_config.h 引脚表 + spi.cpp 总线表
# ===========================================================================
#
#   board_config.h 里全是宏，但它们是【有结构】的宏：
#
#     #define GPIO_nLED_RED   /* PE3 */  (GPIO_OUTPUT|...|GPIO_PORTE|GPIO_PIN3)
#     ^ 宏名              ^注释里的   ^真正的值 —— 端口和引脚号在这里
#                         物理引脚
#
#   ★ 注意那个「注释里的物理引脚」：它是【给人看的】，
#     代码只认 GPIO_PORTE|GPIO_PIN3。两者不一致时，以代码为准 ——
#     而它们不一致是【很常见】的（改板子时忘了改注释）。
#     这正是本课要教的第一条纪律：**注释和代码要能被程序对拍**。
#
#   所以这个实验干的事就是：把两边都抽出来，**并排打印**。
#     如果不一致，一眼就能看见。
#
GPIO_RX = re.compile(
    r"^\s*#?\s*(?:#\s*)?define\s+([A-Za-z_][A-Za-z0-9_]*)\s+"
    r"/\*\s*([A-Za-z0-9_]+)\s*\*/\s*"
    r"\(?([^)\n]*GPIO_PORT([A-Z])\s*\|\s*GPIO_PIN(\d+)[^)\n]*)\)?"
)

PORT_BIT = {"A": 0, "B": 1, "C": 2, "D": 3, "E": 4,
            "F": 5, "G": 6, "H": 7, "I": 8, "J": 9, "K": 10}


def parse_gpio(path):
    """→ [(行号, 宏名, 注释引脚, 端口字母, 引脚号, 值里的功能关键词)]"""
    out = []
    for ln, text in enumerate(read_lines(path), 1):
        m = GPIO_RX.match(text)
        if not m:
            continue
        name, note, val, port, pin = m.groups()
        # 从值里挑出「这个脚被配成什么」—— 只挑对我们有用的
        feats = []
        for key in ("GPIO_OUTPUT", "GPIO_INPUT", "GPIO_ALT", "GPIO_ANALOG",
                    "GPIO_OPENDRAIN", "GPIO_PULLUP", "GPIO_PULLDOWN",
                    "GPIO_EXTI", "GPIO_FLOAT"):
            if key in val:
                feats.append(key.replace("GPIO_", ""))
        out.append((ln, name, note, port, int(pin), "|".join(feats)))
    return out


def exp3():
    hr("实验 3 ★★ 引脚是怎么落到外设的：board_config.h + spi.cpp")

    if not need_board():
        return
    cfg = os.path.join(CUSTOM_BOARD, "src", "board_config.h")
    gpios = parse_gpio(cfg)

    sub("读者板子 board_config.h 里的 GPIO 宏（%d 个）" % len(gpios))
    print("  %s %s %s %s %s" % (_w("行", 5), _w("宏名", 30),
                                _w("注释引脚", 9), _w("代码里的", 10),
                                _w("配置", 26)))
    print("  " + "-" * 70)
    mismatch = 0
    for ln, name, note, port, pin, feats in gpios:
        code_pin = "P%s%d" % (port, pin)
        flag = ""
        if note != code_pin:
            flag = "  ← ★注释与代码不一致"
            mismatch += 1
        print("  %s %s %s %s %s%s"
              % (_w(str(ln), 5), _w(name, 30), _w(note, 9),
                 _w(code_pin, 10), _w(feats, 26), flag))

    print()
    if mismatch:
        print("  ★ 抓到 %d 处「注释里的引脚」和「代码里的 PORT|PIN」不一致。"
              % mismatch)
        print("    两种可能：① 改板子时只改了代码；② 只改了注释。")
        print("    无论哪种，都说明【有一个是错的】，而且只有机器能可靠地查出来。")
    else:
        print("  ★ 本文件的注释与代码完全一致 —— 这是难能可贵的。")

    # ---- SPI 总线表 ----
    spi = os.path.join(CUSTOM_BOARD, "src", "spi.cpp")
    off_spi = os.path.join(OFFICIAL_BOARD, "src", "spi.cpp")

    def spi_table(path, ver_filter=None):
        rows, cur_ver, cur_bus = [], None, None
        for ln, text in enumerate(read_lines(path), 1):
            m = re.search(r"initSPIFmumID\(\s*([A-Za-z0-9_]+)\s*,", text)
            if m:
                cur_ver = m.group(1)
                continue
            m = re.search(r"initSPIBus(?:External)?\(\s*SPI::Bus::(SPI\d)", text)
            if m:
                cur_bus = m.group(1)
                continue
            m = re.search(
                r"initSPIDevice\(\s*([A-Za-z0-9_()]+)\s*,\s*"
                r"SPI::CS\{GPIO::Port([A-Z]),\s*GPIO::Pin(\d+)\}"
                r"(?:,\s*SPI::DRDY\{GPIO::Port([A-Z]),\s*GPIO::Pin(\d+)\})?", text)
            if m:
                dev, cp, cn, dp, dn = m.groups()
                rows.append((cur_ver, cur_bus, dev,
                             "P%s%s" % (cp, cn),
                             ("P%s%s" % (dp, dn)) if dp else "—",
                             ln))
        return rows

    for label, path in (("读者板子", spi), ("官方 fmu-v6x", off_spi)):
        rows = spi_table(path)
        sub("%s spi.cpp —— HW 版本 → 总线 → 器件（共 %d 条）" % (label, len(rows)))
        print("  %s %s %s %s %s %s"
              % (_w("HW版本", 8), _w("总线", 6), _w("器件", 22),
                 _w("CS", 6), _w("DRDY", 6), _w("行", 5)))
        print("  " + "-" * 70)
        last = None
        for ver, bus, dev, cs, drdy, ln in rows:
            v = ver if ver != last else ""
            last = ver
            print("  %s %s %s %s %s %s"
                  % (_w(v, 8), _w(bus, 6), _w(dev, 22),
                     _w(cs, 6), _w(drdy, 6), _w(str(ln), 5)))

    sub("★ 两个数字要对上")
    n_ver = len(grep(os.path.join(CUSTOM_BOARD, "src", "board_config.h"),
                     r"^#define BOARD_NUM_SPI_CFG_HW_VERSIONS"))
    ver_line = grep(os.path.join(CUSTOM_BOARD, "src", "board_config.h"),
                    r"^#define BOARD_NUM_SPI_CFG_HW_VERSIONS")
    n_blocks = len(set(r[0] for r in spi_table(spi)))
    if ver_line:
        ln, txt = ver_line[0]
        declared = int(re.search(r"(\d+)\s*$", txt.strip()).group(1))
        print("  fmu-v6x-qgbf/src/board_config.h:%d" % ln)
        print("      %s" % txt.strip())
        print("      ⇒ 声明了 %d 个 HW 版本" % declared)
        print("  spi.cpp 里实际有 %d 个 initSPIFmumID(...) 块" % n_blocks)
        if declared == n_blocks:
            print("      ✅ 一致")
        else:
            print("      ⚠ 不一致！数组大小是 %d，初始化了 %d 个 —— "
                  "要么编译报错，要么有版本是空的。" % (declared, n_blocks))

    print("""
  ★ 这张表的读法（三列就是三层抽象）：

      HW 版本        ← board_get_hw_version() 运行时读到的值
      总线 / CS      ← 片选引脚：告诉 SPI 控制器「跟谁说话」
      器件 / DRDY    ← 数据就绪中断：告诉驱动「数据好了，来取」

  驱动拿到的是「总线 + 片选 + 中断」三件套，它**完全不知道**自己
  焊在哪块板子上 —— 这就是硬件抽象的收益。
""")


# ===========================================================================
# 4. 一行 CONFIG_BOARD_SERIAL_TEL1 变成了什么
# ===========================================================================
#
#   ★★ 这是「每一项配置对应软件里的什么」最漂亮的一个例子：
#      一行配置，被 cmake 抓出来后**同时喂给两个生成器**，产出三样东西：
#
#        ①  /etc/init.d/rc.serial        启动脚本：把参数值翻译成 -d /dev/ttyS6
#        ②  SER_TEL1_BAUD 参数定义      地面站里能改的波特率
#        ③  MAV_x_CONFIG 的候选值        「串口配置」下拉框里的一项
#
#      对应源码（都可回查）：
#        cmake/kconfig.cmake:308-309      TEL1:/dev/ttyS6  拼成这个形式
#        ROMFS/CMakeLists.txt:130-132     生成 rc.serial
#        src/lib/parameters/CMakeLists.txt:93-95  生成 SER_* 参数
#        Tools/serial/serial_params.c.jinja:42    PARAM_DEFINE_INT32(SER_...,默认值)
#        Tools/serial/rc.serial_port.jinja:7-10   按 MAV_x_CONFIG 查表选设备
#
#   所以「把 GPS 从 ttyS0 挪到 ttyS7」不是一个动作，是**一串**动作 ——
#   理解了这个，你就理解了 BSP 的本质：
#       **配置不是开关，是数据源。**
#
SERIAL_INDEX = {          # 来自 Tools/serial/generate_config.py
    "TEL1": (101, 57600), "TEL2": (102, 921600), "TEL3": (103, 57600),
    "TEL4": (104, 57600), "GPS1": (201, 0), "GPS2": (202, 0),
    "GPS3": (203, 0), "RC": (300, 0), "WIFI": (301, 0),
}


def exp4():
    hr("实验 4 ★★ 一行 CONFIG_BOARD_SERIAL_TEL1 在软件里变成了什么")

    if not need_board():
        return

    def ports(path):
        out = []
        for ln, text in enumerate(read_lines(path), 1):
            m = re.match(r'^CONFIG_BOARD_SERIAL_([A-Z0-9_]+)="(.*)"\s*$',
                         text.strip())
            if m:
                out.append((ln, m.group(1), m.group(2)))
        return out

    cust = ports(os.path.join(CUSTOM_BOARD, "default.px4board"))
    off = ports(os.path.join(OFFICIAL_BOARD, "default.px4board"))

    sub("① 原始配置：功能角色 → Linux 设备名")
    print("  %s %s %s %s" % (_w("来源", 10), _w("行", 5),
                             _w("角色", 8), _w("设备", 16)))
    print("  " + "-" * 70)
    for ln, tag, dev in cust:
        print("  %s %s %s %s" % (_w("读者", 10), _w(str(ln), 5),
                                 _w(tag, 8), _w(dev, 16)))
    for ln, tag, dev in off:
        print("  %s %s %s %s" % (_w("官方", 10), _w(str(ln), 5),
                                 _w(tag, 8), _w(dev, 16)))

    sub("② cmake 抓出来拼成的形式（cmake/kconfig.cmake:288-330）")
    print("  board_serial_ports =")
    for _ln, tag, dev in cust:
        print("      %s:%s" % (tag, dev))

    sub("③-a 第一个产物：SER_<角色>_BAUD 参数")
    print("  Tools/serial/serial_params.c.jinja:42 会把每一项展开成：")
    print()
    for _ln, tag, _dev in cust:
        idx, baud = SERIAL_INDEX.get(tag, (0, 0))
        print("      PARAM_DEFINE_INT32(SER_%s_BAUD, %d);   ← 默认 %d"
              % (tag, baud, baud))
    print("""
  ★ 注意 GPS 的默认波特率是 0 —— 不是「0 bps」，是**自动**。
    GPS 驱动会自己扫波特率（这正是「0 Auto」那一行的含义）。
    而 TEL2 默认 921600，是因为它通常接机载计算机。

  ★ 另一个后果：**参数名是生成的**。
    读者板子没有 TEL3，所以固件里**根本没有 SER_TEL3_BAUD 这个参数** ——
    你在 QGC 里搜不到它，不是地面站的问题，是它压根没被编译进去。
""")

    sub("③-b 第二个产物：/etc/init.d/rc.serial 启动脚本")
    print("  ROMFS/px4fmu_common/init.d/rcS:437 会 . 这个文件；")
    print("  它先用 MAV_x_CONFIG 的值（就是那个 101）查到设备名：")
    print()
    for _ln, tag, dev in cust:
        idx, _b = SERIAL_INDEX.get(tag, (0, 0))
        print('      if param compare "$PRT" %d; then set SERIAL_DEV %s ...'
              % (idx, dev))
    print("""
  然后 mavlink 模块的 yaml（src/modules/mavlink/module.yaml:43）说
  MAV_1_CONFIG 的默认候选是 TEL1，于是拼出：

      mavlink start -d ${SERIAL_DEV} -b p:SER_TEL1_BAUD -m p:MAV_1_MODE ...

  ⇒ 「TEL1 是 /dev/ttyS6」这句话，最终变成了**一个 shell 变量**。
""")

    sub("④ 两块板子的串口角色差异 —— 这才是移植必须改的部分")
    ct = {t for _l, t, _d in cust}
    ot = {t for _l, t, _d in off}
    print("  读者独有：%s" % (sorted(ct - ot) or "（无）"))
    print("  官方独有：%s" % (sorted(ot - ct) or "（无）"))
    print()
    if "RC" in ct and "RC" not in ot:
        print("""  ★ 官方 fmu-v6x 没有 CONFIG_BOARD_SERIAL_RC，读者那块【有】。
    为什么？看 board_config.h 就明白了 —— 官方的 RC 串口是**写死**的：

        boards/px4/fmu-v6x/src/board_config.h:323   #define RC_SERIAL_PORT  "/dev/ttyS5"

    而读者那块板子上，这个宏在 fmu-v6x-qgbf/src/board_config.h:326，
    并且下面两行被注释掉了（**这就是「接收机怎么接」的全部秘密**）：

        :328  //#define RC_SERIAL_SINGLEWIRE     ← ELRS CRSF 是全双工，不能用半双工
        :331  //#define RC_SERIAL_SWAP_RXTX      ← 信号没走反，不用交换 TX/RX

  ⇒ 同一份固件、同一颗芯片、同一个串口，
    **三行宏决定了它能不能认你的接收机。** 这就是板级抽象的力量。""")


# ===========================================================================
# 5. 内存映射：从链接脚本算出来
# ===========================================================================
#
#   ★★★ 这一节把「BSP 三件套」里的第三件（链接脚本）和
#        「固件怎么烧」连起来 —— 它们其实是同一个数字的两种说法。
#
#   推导（不跳步）：
#
#     STM32H753 的片上 FLASH 从 0x0800_0000 开始，共 2048 KB。
#     链接脚本把 FLASH 区起点定在 0x0802_0000
#     （= fmu-v6x-qgbf/nuttx-config/scripts/script.ld:113），
#     那么被让出去给 bootloader 的部分是
#
#         0x0802_0000 − 0x0800_0000 = 0x2_0000 = 131072 B = 128 KiB
#
#     而 fmu-v6x-qgbf/src/hw_config.h:64 里 bootloader 自己也这么写：
#
#         #define APP_LOAD_ADDRESS  0x08020000      ← 它只会跳到这里
#
#     ⇒ **两个文件里的同一个地址，一个是给链接器看的，一个是给 bootloader 看的。**
#       它们不一致的后果：能烧进去，但一上电就跳飞。
#       这是「改板子时最容易漏的一处一致性」。
#
#     剩下的给 app：
#
#         2048 KiB − 128 KiB = 1920 KiB = 0x1E0_0000 B = 1 966 080 B
#
#     而 firmware.prototype:10 里写着：
#
#         "image_maxsize": 1966080
#
#     ⇒ 完全相等。三个文件、三个数字、一个事实。
#
MEM_PAT = re.compile(
    r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*\((\w+)\)\s*:\s*"
    r"ORIGIN\s*=\s*(0x[0-9A-Fa-f]+)\s*,\s*LENGTH\s*=\s*(\d+)\s*([KMG]?)"
)


def parse_ld(path):
    """只解析 MEMORY { ... } 那一段（script.ld 里还有 SECTIONS，别混进来）。"""
    rows, in_mem = [], False
    for ln, text in enumerate(read_lines(path), 1):
        if re.match(r"^\s*MEMORY\s*$", text):
            in_mem = True
            continue
        if not in_mem:
            continue
        # MEMORY 块以单独一行的 `{` 开始、单独一行的 `}` 结束
        if re.match(r"^\s*\}\s*;?\s*$", text):
            break
        m = MEM_PAT.match(text)
        if m:
            name, attr, origin, size, unit = m.groups()
            mult = {"K": 1024, "M": 1024 * 1024, "G": 1024 ** 3, "": 1}[unit]
            rows.append((ln, name, attr, int(origin, 16), int(size) * mult))
    return rows


def exp5():
    hr("实验 5 ★★★ 内存映射：从链接脚本算 bootloader / app / 参数区")

    if not need_board():
        return
    ld = os.path.join(CUSTOM_BOARD, "nuttx-config", "scripts", "script.ld")
    rows = parse_ld(ld)

    sub("script.ld 的 MEMORY 段（%d 个区）" % len(rows))
    print("  %s %s %s %s %s %s" % (_w("行", 5), _w("区名", 12), _w("权限", 6),
                                    _w("起始地址", 12), _w("大小", 10),
                                    _w("字节", 12)))
    print("  " + "-" * 72)
    for ln, name, attr, origin, size in rows:
        if size >= 1024 * 1024:
            human = "%.0f MiB" % (size / 1024 / 1024)
        else:
            human = "%d KiB" % (size // 1024)
        print("  %s %s %s %s %s %s"
              % (_w(str(ln), 5), _w(name, 12), _w(attr, 6),
                 _w("0x%08X" % origin, 12), _w(human, 10),
                 _w("%d" % size, 12)))

    flash = [r for r in rows if r[1] == "FLASH"]
    if not flash:
        print("\n  ⚠ 没解析到 FLASH 区，后面的推导跳过。")
        return
    _ln, _n, _a, flash_origin, flash_size = flash[0]

    hw = os.path.join(CUSTOM_BOARD, "src", "hw_config.h")
    app_line = grep(hw, r"#define\s+APP_LOAD_ADDRESS")
    app_addr = int(re.search(r"(0x[0-9A-Fa-f]+)", app_line[0][1]).group(1), 16) \
        if app_line else None

    # STM32H7 的片内 FLASH 固定从 0x08000000 开始映射，链接脚本只用了后面一段。
    CHIP_FLASH_BASE = 0x08000000
    bl_bytes = flash_origin - CHIP_FLASH_BASE

    sub("推导：这 2048 KiB 的 FLASH 被怎么切")
    print("  ① 芯片 FLASH 起点（STM32H7 的固定映射，见 script.ld 的注释）")
    print("         0x%08X" % CHIP_FLASH_BASE)
    print("  ② 链接脚本 FLASH 区起点          script.ld:%d" % _ln)
    print("         0x%08X" % flash_origin)
    print("         ⇒ 让给 bootloader 的部分 = 0x%08X − 0x%08X = 0x%X"
          % (flash_origin, CHIP_FLASH_BASE, bl_bytes))
    print("                                   = %d B = %d KiB"
          % (bl_bytes, bl_bytes // 1024))
    if app_addr is not None:
        print("  ③ bootloader 自己的跳转目标      hw_config.h:%d" % app_line[0][0])
        print("         %s" % app_line[0][1].strip())
        print("         0x%08X" % app_addr)
        if app_addr == flash_origin:
            print("         ✅ 和 ② 相等 —— 链接器和 bootloader 说的是同一件事")
        else:
            print("         ⚠ 和 ② 不等！链接器和 bootloader 对不上，"
                  "烧进去也跳飞。")
    print("  ④ 留给 app 的 FLASH                script.ld:%d" % _ln)
    print("         LENGTH = %d B = %d KiB" % (flash_size, flash_size // 1024))
    total = bl_bytes + flash_size
    print("  ⑤ 两段加起来")
    print("         %d + %d = %d B = %d KiB = %.2f MiB"
          % (bl_bytes, flash_size, total, total // 1024, total / 1024 / 1024))
    if total == 2 * 1024 * 1024:
        print("         ✅ = 2048 KiB —— 正好是 STM32H753 的全部片内 FLASH，")
        print("            一个字节都没剩下（所以参数区不可能放在片内 FLASH）")
    else:
        print("         ⚠ 不等于 2048 KiB（差 %d B），说明还有别的段没算进来。"
              % (2 * 1024 * 1024 - total))

    sub("交叉验证：第三个文件里的同一个数字")
    proto = os.path.join(CUSTOM_BOARD, "firmware.prototype")
    plines = read_lines(proto)
    maxsize = None
    for i, t in enumerate(plines, 1):
        m = re.search(r'"image_maxsize":\s*(\d+)', t)
        if m:
            maxsize = int(m.group(1))
            print("  firmware.prototype:%d  %s" % (i, t.strip()))
    if maxsize is not None:
        print()
        print("      image_maxsize = %d B" % maxsize)
        print("      script.ld FLASH LENGTH = %d B" % flash_size)
        if maxsize == flash_size:
            print("      ✅ 相等 —— 固件包的最大尺寸就是链接脚本给的 app 区大小")
        else:
            print("      ⚠ 不等：差值 %+d B。差在哪需要查。" % (maxsize - flash_size))

    sub("★ 第三个数字：board_id —— 决定「这个固件能不能烧进这块板」")
    for i, t in enumerate(plines, 1):
        if "board_id" in t:
            print("  firmware.prototype:%d  %s" % (i, t.strip()))
    bl = grep(hw, r"#define\s+BOARD_TYPE")
    if bl:
        print("  hw_config.h:%d         %s" % (bl[0][0], bl[0][1].strip()))
    off_proto = os.path.join(OFFICIAL_BOARD, "firmware.prototype")
    for i, t in enumerate(read_lines(off_proto), 1):
        if "board_id" in t:
            print("  (官方) firmware.prototype:%d  %s" % (i, t.strip()))
    print("""
  ⇒ bootloader 通过 USB 报告 BOARD_TYPE，px_uploader.py 拿它和固件包里的
    board_id 比对（Tools/px_uploader.py:616）。不等就拒绝烧录：

        Firmware not suitable for this board (board_type=%u board_id=%u)

  ★★ 注意：读者那块板的 board_id 和官方 fmu-v6x **完全相同**（都是 53）。
     这意味着：
       · 官方 PX4 固件可以刷进读者那块板 —— 上传器拦不住它；
       · 反过来也一样。
     这是设计上的选择（保持兼容），也是风险：
     **board_id 只保证「硬件大致同源」，不保证「引脚完全一致」。**
     真出问题时，第一件事就是核对 board_config.h 的引脚表。

  ★ 想强制烧入不匹配的固件：make <board>_default force-upload
    （platforms/nuttx/cmake/upload.cmake:89 定义了 force-upload 目标，
     对应 px_uploader.py 的 --force。）
""")


# ===========================================================================
# main
# ===========================================================================
EXPS = {"1": exp1, "2": exp2, "3": exp3, "4": exp4, "5": exp5}


def main():
    hr("第 22.5 周 · 板级支持包（BSP）与硬件抽象")
    print()
    print("  教程根   : %s" % TUTORIAL)
    print("  项目根   : %s" % PROJECT)
    print("  官方 PX4 : %s" % (PX4_ROOT if os.path.isdir(PX4_ROOT) else "⚠ 不存在"))
    print("  读者板子 : %s"
          % (CUSTOM_BOARD if CUSTOM_BOARD else "⚠ 未找到"))
    print()
    print("  ★ 本脚本只读文件、不编译、不联网。只用 Python 标准库。")

    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if args and args[0] in EXPS:
        EXPS[args[0]]()
    else:
        for k in sorted(EXPS):
            EXPS[k]()

    print()
    print("=" * W)
    print("  完成。所有数字都可以按上面打印的行号回原文核对。")
    print("=" * W)


if __name__ == "__main__":
    main()
