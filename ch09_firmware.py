# -*- coding: utf-8 -*-
"""ch09_firmware.py —— 卷 9 第 15 章：一份板子，几份固件

## 这一章要讲的事

"给这块板子编固件"听起来像**一件事**。实际上是**好几件**：

    同一份 BSP 目录下，躺着 **7 份** `.px4board` 配置。
    最大的一份 98 行配置，最小的一份 **1 行**。

它们不是"版本"，是**用途完全不同的固件**：
    引导程序（只需要 3 项）、主固件、诊断固件、机型变体……

## 本章最值得看的一份：诊断固件

`performance-test` 那一份列了 **20 条系统命令**，
而**飞控模块一条都没有** —— 这是一个专门用来看板子、不看姿态的固件。

## 本脚本做什么

**直接解析这些配置文件**：
① 七份配置各自多少项开关（看"能裁到多小"）
② 把 performance-test 那份按类别摊开（它到底装了什么）
③ `extras/` 里躺着的产物与它们的大小
④ ★ 一块板子**同时存在两个引导程序**，差多少字节

跑法：python code/ch09_firmware.py
"""

import io
import _srcdirs
import os
import re
import unicodedata


def pad(s, n):
    """按显示宽度补齐（汉字占两列）。"""
    return s + ' ' * max(0, n - sum(
        2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s))


BSP = _srcdirs.src("BSP")

# 开关按前缀分类（顺序即匹配优先级）
CATS = [
    ('SYSTEMCMDS_', '系统命令'),
    ('MODULES_',    '飞控模块'),
    ('DRIVERS_',    '驱动'),
    ('BOARD_',      '板级设置'),
    ('COMMON_',     '通用功能'),
]


def rd(p):
    return io.open(p, encoding='utf-8', errors='replace').read()


def cfg_lines(text):
    """取**全部配置行**（含字符串/数值型）。

    ⚠️ 不能只匹配 `=y`：`bootloader.px4board` 里三项**全是字符串**
       （工具链、架构、ROMFS 根），只匹配布尔值会把它数成 **0 行** ——
       于是"最小的一份"这个结论就整个错了。
    """
    return [m.group(1) for m in
            re.finditer(r'^(CONFIG_[\w]+)=', text, re.M)]


def switches(text, only_true=True):
    """取布尔型开关（`=y`，或 `=n` 当 only_true=False）。"""
    out = []
    for m in re.finditer(r'^(CONFIG_[\w]+)=(y|n)\s*$', text, re.M):
        if only_true and m.group(2) != 'y':
            continue
        out.append(m.group(1))
    return out


def main():
    boards = sorted(f for f in os.listdir(BSP) if f.endswith('.px4board'))

    print('=' * 74)
    print('① 同一份 BSP 目录下的全部配置')
    print('=' * 74)
    print()
    print('  %s %s %s' % (pad('配置文件', 30), pad('配置行', 10), '规模'))
    rows = []
    for f in boards:
        rows.append((f, len(cfg_lines(rd(os.path.join(BSP, f))))))
    for f, n in sorted(rows, key=lambda x: -x[1]):
        bar = '█' * max(1, int(round(n / 3.0)))
        print('  %s %s %s' % (pad(f, 30), pad('%d' % n, 10), bar))
    print()
    print('  ★★ 一共 **%d 份配置**，从 %d 行到 %d 行 —— **相差 %.0f 倍**。'
          % (len(rows), min(n for _f, n in rows), max(n for _f, n in rows),
             max(n for _f, n in rows) / float(max(1, min(n for _f, n in rows)))))
    print()
    print('  ★ 最小的那份是 `flash-analysis.px4board`：**只改一个链接脚本**。')
    print('    它就是"同一份固件、换一种摆放方式"—— 用来做**闪存占用分析**。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('② 诊断固件：`performance-test` 里装了什么')
    print('=' * 74)
    print()
    pt = switches(rd(os.path.join(BSP, 'performance-test.px4board')),
                  only_true=True)
    print('  这份配置共 **%d 项**开关，按类别摊开：' % len(pt))
    print()
    print('  %s %s' % (pad('类别', 16), '条数'))
    for pref, tag in CATS:
        n = len([x for x in pt if x.startswith('CONFIG_' + pref)])
        print('  %s %s' % (pad(tag, 16), '%d' % n))
    other = [x for x in pt
             if not any(x.startswith('CONFIG_' + p) for p, _t in CATS)]
    print('  %s %s' % (pad('其余', 16), '%d' % len(other)))
    print()
    names = [x.split('CONFIG_SYSTEMCMDS_')[-1]
             for x in pt if x.startswith('CONFIG_SYSTEMCMDS_')]
    print('  那 %d 条系统命令是：' % len(names))
    for i in range(0, len(names), 6):
        print('    %s' % '、'.join(names[i:i + 6]))
    print()
    mods = [x for x in pt if x.startswith('CONFIG_MODULES_')]
    print('  ★★★ 而**飞控模块：%d 条**。' % len(mods))
    print()
    print('  ⇒ 这不是"精简版固件"，是**用途完全不同的一份**：')
    print('    它不飞，它只负责**把板子的状态摊开给你看** ——')
    print('    看内存、看任务、看中断、看总线、看闪存、看 SD 卡。')
    print()
    print('  ★★ 为什么值得单独做一份：**把诊断和飞控放在同一个固件里，')
    print('     你会分不清"这个现象是板子的，还是飞控的"。**')
    print('     分成两份之后，诊断固件里**没有飞行代码可以背锅**。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('③ 产物：`extras/` 里躺着什么')
    print('=' * 74)
    ex = os.path.join(BSP, 'extras')
    for f in sorted(os.listdir(ex)):
        p = os.path.join(ex, f)
        print('  %s %s' % (pad(f, 44), '%d 字节' % os.path.getsize(p)))
    print()
    print('  ★ 三个产物，但它们**不是同一类东西**：')
    print('    · 两个是**引导程序**（一个带板名、一个不带）')
    print('    · 一个是 **IO 板**的固件（那块板子上的另一颗 MCU）')
    print()
    print('  ⚠️ 注意最后那个：**飞控板上的软件不止跑在一颗芯片上。**')
    print('     IO 板有自己的固件、自己的产物、自己的烧写方式 ——')
    print('     而它和主固件**是一起发布的**。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('④ ★ 一块板子，两个引导程序')
    print('=' * 74)
    print()
    boot = [f for f in os.listdir(ex) if 'bootloader' in f]
    sizes = sorted(((os.path.getsize(os.path.join(ex, f)), f) for f in boot))
    for n, f in sizes:
        print('  %s %s' % (pad(f, 44), '%d 字节' % n))
    if len(sizes) == 2:
        d = sizes[1][0] - sizes[0][0]
        print()
        print('  ⇒ 两者相差 **%d 字节**（%.1f%%）。'
              % (d, 100.0 * d / sizes[0][0]))
        print()
        print('  ★★ 带板名的那一个是**自己编的**（V1.4 新增），')
        print('     不带板名的那个是**官方原版** —— 它还留在这里。')
        print()
        print('  ⚠️ 两个都留着，本身不是错（可以拿来对照），')
        print('     但它意味着：**"用哪个引导程序"必须有一个人说得清。**')
        print('     否则下次烧写时，选错的那个**也能烧进去、也能跑** ——')
        print('     因为它本来就是为同一颗芯片编的。')
        print()
        print('  ⇒ 判据：**同名同类但不同来源的产物放在一起时，')
        print('     必须有文档说明"该用哪一个、另一个为什么在"。**')
        print('     （这与第 7 章"报告指哪就查哪"是同一类纪律：')
        print('      产物也得说清自己是谁。）')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('⑤ 同一块板，被描述了几次')
    print('=' * 74)
    print()
    print('  · 主固件配置      ：`default.px4board`（%d 行）'
          % len(cfg_lines(rd(os.path.join(BSP, 'default.px4board')))))
    nl = os.path.join(BSP, 'nuttx-config')
    sub = []
    for r, _d, fs in os.walk(nl):
        for f in fs:
            if 'defconfig' in f:
                sub.append(os.path.relpath(os.path.join(r, f), BSP)
                           .replace(os.sep, '/'))
    print('  · 引导程序配置    ：`bootloader.px4board`（%d 行）'
          % len(cfg_lines(rd(os.path.join(BSP, 'bootloader.px4board')))))
    print('  · 内核配置（主）  ：%s' % '、'.join('`%s`' % s for s in sub))
    print()
    print('  ★★ 也就是说：**同一块板子，被描述了好几次** ——')
    print('     应用层一份（`default.px4board`）、引导层一份、')
    print('     内核层还各有一份（主固件 / 引导程序分开）。')
    print()
    print('  ⇒ 回到第 11 章那句话：**"三层"其实不止三层。**')
    print('     每多一层描述，就多一处**可以不一致的地方**；')
    print('     而它们**都不报错** —— 因为每一层单独看都是合法的。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('自检 —— 解析器必须先自证读对了')
    print('=' * 74)
    assert len(boards) >= 5, '① 配置文件太少 —— 路径不对'
    print('  ✓ 解出 %d 份 .px4board 配置' % len(boards))
    assert all(n > 0 for _f, n in rows), '① 有配置文件一行都没解到'
    print('  ✓ 每份配置都解到了非零行（最小的一份 %d 行）'
          % min(n for _f, n in rows))
    assert len(pt) > 10, '② 诊断固件解析到的项太少'
    n_cmd = len([x for x in pt if x.startswith('CONFIG_SYSTEMCMDS_')])
    assert n_cmd >= 10, '② 系统命令解到的条数太少'
    print('  ✓ 诊断固件：%d 项开关、其中系统命令 %d 条' % (len(pt), n_cmd))
    # 关键：诊断固件里必须**没有飞控模块**，否则 ② 那节的结论不成立
    assert len(mods) == 0, '② 诊断固件里居然有飞控模块 —— 结论反了'
    print('  ✓ 诊断固件里飞控模块为 **0**（结论成立）')
    # ④ 必须真的有**两个** bootloader，否则那节是空的
    assert len(sizes) == 2, '④ 引导程序不是两个（实数 %d）' % len(sizes)
    print('  ✓ 检出两个引导程序产物，相差 %d 字节' % (sizes[1][0] - sizes[0][0]))
    # ⑤ 内核配置必须找到（否则"被描述几次"是空话）
    assert len(sub) >= 2, '⑤ 内核配置少于两份'
    print('  ✓ 内核配置文件 %d 份' % len(sub))
    print()
    print('  ✅ 自检通过')
    print()
    print('=' * 74)
    print('口径：本脚本**直接解析 BSP 目录里的配置文件与产物**。')
    print('      根目录：D:/DEVLib/PX4APM/硬件/版本V1.4 …/fmu-v6x-qgbf/')
    print('      ⚠️ 配置与产物随版本变化 —— 换版本后输出会变（这是刻意的）。')
    print('=' * 74)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
