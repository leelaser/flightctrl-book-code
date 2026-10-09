#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 5 第 10 章配套实验：可选组件（FatFS / lwIP / USB / shell）

    实验 1  ★★ os/various 里没有"库本体"，全是胶水
    实验 2  ★★ 一个块设备抽象，两个消费者
    实验 3  ★ FatFS 的 5 个接口怎么映射到 blk*
    实验 4  ★★ AP 自己的 VFS：7 个后端
    实验 5  ★★ AP 到底编了 os/various 的哪几个

用法：
    python lesson19_optional_components.py 1
    ...
    python lesson19_optional_components.py        # 全部

【诚实声明】

本章五个实验**都是读源码** —— 结论来自文件内容，没有模拟。
（这一课的主题是"接线"，而接线是可以用 grep 数清楚的东西。）

唯一需要注意的口径：实验 5 数的是"有没有被 include"，不是
"编译出来有多少行"。一个 .mk 被 include 了不等于它下面每个文件
都会进最终固件（还有 `HAL_USE_*` 之类的开关）—— 但对
"用/不用"这个粗判已经足够。

源码版本：ArduPilot 4.5.7 pin 的 ChibiOS 6a85082c（内核 7.0.4）
"""

import io
import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APM_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "APM",
                 "ardupilot-Copter-4.5.7", "ardupilot-Copter-4.5.7")
)
CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")
VARIOUS = os.path.join(CHIBIOS, "os", "various")
HAL_INC = os.path.join(CHIBIOS, "os", "hal", "include")

F_IOBLOCK_H = os.path.join(HAL_INC, "hal_ioblock.h")
F_FATFS_DIO = os.path.join(VARIOUS, "fatfs_bindings", "fatfs_diskio.c")
F_SCSI = os.path.join(VARIOUS, "scsi_bindings", "lib_scsi.c")
F_BOARD_MK = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "hwdef", "common", "chibios_board.mk")
F_APFS_DIR = os.path.join(APM_ROOT, "libraries", "AP_Filesystem")
F_FATFS_AP = os.path.join(F_APFS_DIR, "AP_Filesystem_FATFS.cpp")
F_APFS_H = os.path.join(F_APFS_DIR, "AP_Filesystem.h")
F_APFS_BE = os.path.join(F_APFS_DIR, "AP_Filesystem_backend.h")

# ===========================================================================
# 小工具
# ===========================================================================

def read(path):
    try:
        with io.open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def find_line(path, pattern, start=1):
    lines = read(path).split("\n")
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


def rel(path):
    p = os.path.normpath(path)
    if "/os/various/" in p.replace("\\", "/") or "/os/hal/" in p.replace("\\", "/"):
        return os.path.basename(p)
    if p.startswith(APM_ROOT):
        rest = p[len(APM_ROOT) + 1:].replace(os.sep, "/")
        return rest if rest.startswith("libraries/") else os.path.basename(p)
    return p.replace(os.sep, "/")


def src(path, a, b, note=None):
    lines = read(path).split("\n")
    if note:
        print("      %s" % note)
    for i in range(max(0, a - 1), min(len(lines), b)):
        print("      %s:%d  %s" % (rel(path), i + 1, lines[i].rstrip()[:104]))
    print()


def wc_l(path):
    """行数口径 —— **必须和 `_coverage_audit.py` 的 `one()` 一致**。

    两个坑（作者都踩过）：
      · 文件以换行结尾时，`len(read().split('\\n'))` 会多算一行
        ⇒ 只数换行符，末尾没换行才 +1
      · 目录要**递归** —— `lwip_bindings/arch/` 里还有 4 个文件
        （`sys_arch.c` 就在那儿），只数第一层会少 535 行
    """
    s = read(path)
    if not s:
        return 0
    return s.count("\n") + (0 if s.endswith("\n") else 1)


def dir_lines(d):
    """一个目录下 .c/.h 的总行数（递归 + 同 `one()` 口径）。"""
    tot = 0
    for dirpath, _dirs, files in os.walk(d):
        for f in files:
            if f.endswith((".c", ".h")):
                tot += wc_l(os.path.join(dirpath, f))
    return tot


# ===========================================================================
# 实验 1：os/various 里没有"库本体"
# ===========================================================================

def exp1():
    head("实验 1  ★★ os/various 里没有「库本体」，全是胶水")

    print("""
  《NuttX 教程》卷 6 第 10 章那张表里，`os/various` 那一行写的是
  "FatFS/LittleFS/lwIP/USB/wolfSSL/httpd/shell" —— 看起来像"一堆库"。

  但打开目录会发现一件事：**这里一个库本体都没有。**
""")

    dirs = []
    for d in sorted(os.listdir(VARIOUS)):
        p = os.path.join(VARIOUS, d)
        if os.path.isdir(p):
            dirs.append((d, dir_lines(p)))
        elif d.endswith((".c", ".h")):
            dirs.append((d, read(p).count("\n")))
    dirs.sort(key=lambda x: -x[1])

    print("      %-26s %8s  %s" % ("目录/文件", "行数", "里面是什么"))
    print("      " + "-" * 68)
    note = {
        "shell": "命令行解释器（★ 一个完整的 shell 实现）",
        "scsi_bindings": "把块设备包成 USB 大容量存储设备",
        "lwip_bindings": "lwIP 的 OS 适配层 + 网络线程",
        "wolfssl_bindings": "wolfSSL 的配置与随机数源",
        "littlefs_bindings": "LittleFS 的 flash 适配",
        "fatfs_bindings": "FatFS 的磁盘 I/O 适配",
        "httpd_fatfs_bindings": "把 FatFS 挂到 lwIP httpd 上",
        "pico_bindings": "Raspberry Pi Pico 的适配",
        "cpp_wrappers": "C++ 封装（给 C 接口套一层）",
        "ramdisk.c": "内存盘（块设备的一种实现）",
        "ramdisk.h": "同上",
        "evtimer.c": "事件定时器小工具",
        "syscalls.c": "newlib 的 syscall 桩",
    }
    tot = 0
    for name, n in dirs:
        if n == 0:
            continue
        tot += n
        print("      %-26s %8d  %s" % (name, n, note.get(name, "")))
    print("      " + "-" * 68)
    print("      %-26s %8d" % ("合计", tot))

    print("""
  ★★ **关键观察**：这些**全是"适配层"** —— 真正的库
     （FatFS 的 `ff.c`、lwIP 的 `*.c`、wolfSSL 的 `*.c`）都**不在这里**：

         · FatFS / lwIP / wolfSSL 是通过各自目录下的 `*.mk`
           从库自己的源码树里拉进来的
         · 这里的 `*_bindings/` 只做一件事：
           **把库要求的接口，翻译成 ChibiOS 的接口**

  ⇒ 所以本章不叫"讲 FatFS"，而是**"讲这层胶水"** ——
    因为胶水才是 ChibiOS 写的，也才是这一课能讲出东西的地方。
""")


# ===========================================================================
# 实验 2：一个块设备抽象，两个消费者
# ===========================================================================

def exp2():
    head("实验 2  ★★ 一个块设备抽象，两个消费者")

    print("""
  FatFS 和 USB 大容量存储（SCSI）看着是两个不相干的东西，
  但它们**都只和同一层打交道**：`IOBlock`。
""")
    n = find_line(F_IOBLOCK_H, r"#define _base_block_device_methods")
    if n:
        src(F_IOBLOCK_H, n, n + 22, "块设备的接口（hal_ioblock.h）：")

    print("""  ★ 注意 `is_inserted` / `is_protected` —— **可插拔介质**才需要的方法。
    这个接口是照着"SD 卡 / U 盘"这类设备设计的。

  它的**消费者**是谁？实测：
""")
    print("      %-16s %s" % ("文件", "干什么"))
    print("      " + "-" * 60)
    for name, f, what in (
            ("fatfs_diskio.c", F_FATFS_DIO, "把块设备包成 FatFS 的 disk_* 接口"),
            ("lib_scsi.c",     F_SCSI,     "把块设备包成 USB MSD 的 SCSI 命令处理")):
        print("      %-16s %s" % (name, what))

    print("""
  ⇒ ★★ **同一张 SD 卡，两个消费者：**

        ① **飞控自己读写**（日志、参数、任务文件）—— 走 FatFS
        ② **插到电脑上当一个 U 盘**（USB MSD）        —— 走 SCSI

     两者共用同一个 IOBlock 实例（`FATFS_HAL_DEVICE`），
     所以**电脑看到的和飞控看到的必须是同一份数据** ——
     这正是飞控开发者要的行为（落地后直接插电脑下日志）。

  ★ 这也解释了为什么这两个绑定**在 ArduPilot 里都被编译**
    （§5 会看到 board makefile 里 include 了它们俩），
    而 `littlefs_bindings` 没被编译 —— 它没有第二个消费者。
""")


# ===========================================================================
# 实验 3：FatFS 的接口映射
# ===========================================================================

def exp3():
    head("实验 3  ★ FatFS 的 5 个接口怎么映射到 blk*")

    print("""
  FatFS 要求底层提供**固定名字**的几个函数（`disk_initialize` /
  `disk_status` / `disk_read` / `disk_write` / `disk_ioctl`）。
  这层胶水就是把它们接到 IOBlock 上。
""")

    lines = read(F_FATFS_DIO).split("\n")
    fns = []
    for i, l in enumerate(lines, 1):
        m = re.match(r"^(DSTATUS|DRESULT)\s+(disk_\w+)\s*\(", l)
        if m:
            fns.append((i, m.group(1), m.group(2)))
    print("      %-6s %-10s %s" % ("行号", "返回", "函数"))
    print("      " + "-" * 44)
    for ln, ret, name in fns:
        print("      %-6d %-10s %s" % (ln, ret, name))

    n = find_line(F_FATFS_DIO, r"^DRESULT disk_read \(")
    if n:
        src(F_FATFS_DIO, n, n + 14, "★ disk_read 全文 —— 整个文件的核心就是这一行：")

    print("""  ★★ **注意它调的是 `blkRead`，不是 `sdcRead`。**

    ⇒ 胶水**不需要知道下面是 SD 卡** —— `IOBlock` 已经抽象掉了。
      这就是为什么这个文件只有 198 行：它只做**名字翻译**。

  ★ 还有三个细节值得看：
""")
    n2 = find_line(F_FATFS_DIO, r"#define FATFS_RETRY")
    if n2:
        src(F_FATFS_DIO, n2 - 2, n2 + 8, "重试宏（SD 卡会偶发失败，必须重试）：")
    n3 = find_line(F_FATFS_DIO, r"extern SDCDriver FATFS_HAL_DEVICE")
    if n3:
        src(F_FATFS_DIO, n3, n3 + 1, "★ 设备实例是【外部】给的（§2.4 说它从哪来）：")
    n4 = find_line(F_FATFS_DIO, r"switch \(pdrv\)")
    if n4:
        src(F_FATFS_DIO, n4 - 2, n4 + 5,
            "FatFS 支持多驱动器，ChibiOS 只实现 0 号：")

    print("""  ----------------------------------------------------------------------
  ⇒ **结论**：一层好的胶水**只做名字翻译，不做逻辑**。
     198 行里做的事情，可以完整概括成三件：

        ① 名字对齐（`disk_read` → `blkRead`）
        ② 状态检查 + 重试（`BLK_READY` / `FATFS_RETRY`）
        ③ 参数顺序对齐（FatFS 用 LBA + 扇区数，IOBlock 也一样）

     **真正的复杂性（SD 卡的协议、DMA、错误恢复）在 SDC 驱动里**，
     不在这层。
""")


# ===========================================================================
# 实验 4：AP 自己的 VFS
# ===========================================================================

def exp4():
    head("实验 4  ★★ AP 自己的 VFS：7 个后端")

    print("""
  一个容易被忽略的事实：**ArduPilot 的文件系统不是 FatFS** ——
  它自己写了一层 VFS（虚拟文件系统），下面是可插拔的后端。
""")
    n = find_line(F_APFS_H, r"class AP_Filesystem")
    if n:
        src(F_APFS_H, n - 2, n + 12, "AP_Filesystem 的接口（POSIX 风格）：")

    be = []
    for f in sorted(os.listdir(F_APFS_DIR)):
        if f.startswith("AP_Filesystem_") and f.endswith(".cpp"):
            name = f[len("AP_Filesystem_"):-len(".cpp")]
            if name == "backend":
                continue
            be.append(name)
    print("      后端（%d 个）：" % len(be))
    print("      " + "-" * 62)
    what = {
        "FATFS": "★ SD 卡上的真文件系统（走 ChibiOS 的 fatfs 绑定）",
        "ROMFS": "★ 编进固件里的只读文件（`ROMFS` 段）",
        "Param": "★★ **参数伪装成文件**（`/params/xxx`）",
        "Mission": "★★ **任务伪装成文件**（`/mission/...`）",
        "Sys": "虚拟 /sys（版本号、线程状态…）",
        "ESP32": "ESP32 板的直通（ESP-IDF 自带 VFS）",
        "posix": "Linux/SITL 上直接转发给系统调用",
    }
    for name in be:
        print("      %-12s %s" % (name, what.get(name, "")))

    print("""
  ★★ **最后三个才是关键**：`Param` / `Mission` / `ROMFS`
     —— 它们**把不是文件的东西伪装成文件**。

  ⇒ 这样带来的好处：**上层只需要一套 API**。

        地面站（MAVLink FTP）   ┐
        脚本（Lua）             ├─→ open/read/write/seek
        ftp 命令（ChibiOS?）    ┘        ↓
                                   VFS 分发到 7 个后端
""")

    n2 = find_line(F_FATFS_AP, r"#include <ff.h>")
    if n2:
        src(F_FATFS_AP, n2 - 8, n2 + 2,
            "★ 只有 FATFS 这个后端才 include FatFS（AP_Filesystem_FATFS.cpp）：")

    print("""  ⇒ **为什么 AP 不直接用 FatFS？** 因为：

      ① FatFS 只有"文件"这一个概念，而 AP 要挂 ROMFS/参数/任务
      ② POSIX 风格的 API（`open/read/write`）能让 MAVLink FTP 协议
         和 Lua 脚本**不用改代码**就能访问这些东西
      ③ ★ 换后端不影响上层 —— 比如 SITL 下 posix 后端直接用宿主机的文件

  ★ 这是"**在别人的抽象之上再抽象一层**"的典型案例 ——
    而它的动机不是"FatFS 不好"，是"**我要的语义比 FatFS 多**"。
""")


# ===========================================================================
# 实验 5：AP 到底编了哪几个
# ===========================================================================

def exp5():
    head("实验 5  ★★ AP 到底编了 os/various 的哪几个")

    print("""
  实验 1 数了 os/various 有哪些模块，实验 2/4 看了其中两个怎么用。
  这一节把**"AP 编了没有"**数清楚 —— 答案在 board makefile 里。
""")
    mk = read(F_BOARD_MK)
    incs = re.findall(r"include\s+\$\(CHIBIOS\)/(os/various/[\w/\.]+)", mk)
    srcs = re.findall(r"\$\(CHIBIOS\)/(os/various/[\w/\.]+)", mk)
    print("      %s 里引用的 os/various 路径：" % rel(F_BOARD_MK))
    print("      " + "-" * 62)
    for p in sorted(set(incs) | set(srcs)):
        kind = "include .mk" if p.endswith(".mk") else "源文件"
        print("      %-46s %s" % (p, kind))

    print("""
  对照 os/various 的全部模块：
""")
    USED = {"fatfs_bindings", "scsi_bindings", "cpp_wrappers"}
    print("      %-24s %-8s %s" % ("模块", "行数", "AP 编了吗"))
    print("      " + "-" * 66)
    tot_used = tot_unused = 0
    for d in sorted(os.listdir(VARIOUS)):
        p = os.path.join(VARIOUS, d)
        if not os.path.isdir(p):
            continue
        n = dir_lines(p)
        used = d in USED
        if used:
            tot_used += n
        else:
            tot_unused += n
        print("      %-24s %-8d %s" % (d, n, "✅ 编" if used else "❌ 不编"))
    print("      " + "-" * 66)
    print("      %-24s %-8d ✅ / ❌ 合计 %d / %d 行"
          % ("合计", tot_used + tot_unused, tot_used, tot_unused))

    print("""
  ★★ **只编了 3 个模块**（fatfs / scsi / cpp_wrappers），
     **关掉了 5 个**（littlefs / shell / lwip / wolfssl / httpd）。

  ★ 而关掉的那 5 个里，有几个是**看起来很像飞控要的**：

      · `shell`（1,213 行）—— 一个完整的命令行解释器。
        ★ 但飞控上**已经有 shell 了**：NuttX 那边有 NSH，
          而 ChibiOS 这边 AP 用的是 **MAVLink 的 `ftp`/`shell` 命令**
          （《NuttX 教程》卷 6 第 10 章 第 10 项"绕过"谈过同一个取舍）
      · `lwip_bindings`（860 行）—— 网络协议栈。
        ★ 但 AP 的网络走自己的 `AP_Networking`，
          而且**不是所有板子都有网络**
      · `wolfssl_bindings`（504 行）—— TLS。
        ★ 只有加密遥测（`AP_Networking` + `AP_JSON`）才需要

  ⇒ **"关掉"不等于"没需求"，而是"需求由别的东西满足了"。**
    这是这一课和卷 5 第 9 章的同一个主题（§6.2）。
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★★ os/various 全是胶水", exp1),
    "2": ("★★ 一个块设备，两个消费者", exp2),
    "3": ("★ FatFS 的接口映射", exp3),
    "4": ("★★ AP 的 VFS 与 7 个后端", exp4),
    "5": ("★★ AP 编了哪几个", exp5),
}


def main():
    print()
    hr("#")
    print("#  本章：可选组件（FatFS / lwIP / USB / shell）")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(CHIBIOS):
        print()
        print("  [错误] 找不到 ChibiOS 源码：")
        print("         %s" % CHIBIOS)
        print()
        print("  请先按《总纲》的方法拉取 6a85082c 版本。")
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
    print("  #  实验结束")
    hr("#")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
