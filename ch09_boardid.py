# -*- coding: utf-8 -*-
"""ch09_boardid.py —— 卷 9 第 10 章：板子的身份

## 这一章要讲的事

固件上电，怎么知道**自己在哪块板上、上面焊了哪些传感器**？

PX4 的答案是一张**版本号表**。而这张表在**三个地方各有一份副本**：

    ① `board_config.h` 里的 `#define V6X_*`        —— 有哪些版本号
    ② `board_config.h` 里的 `BOARD_NUM_..._VERSIONS` —— 有几个版本号
    ③ `src/spi.cpp` 里的 `initSPIFmumID(...)` 块     —— 每个版本号上挂什么传感器

**这三份必须一致。** 而它们一致与否**编译器不管** ——
② 只是个数组长度，多一个少一个都能编过。

## 本脚本做什么

**直接解析这两份源码**，把三份副本的版本号集合取出来对一遍：
集合一致 ⇒ 表是完整的；不一致 ⇒ 会静默出问题。

⚠️ 本脚本读的是**真实源码**（路径见文末"口径"），
   所以它既是复算，也是**交叉检查**：
   一旦有人给 board_config.h 加了版本号却忘了改 spi.cpp，
   这里会红。

跑法：python code/ch09_boardid.py
"""

import io
import os
import re
import unicodedata


def pad(s, n):
    """按显示宽度补齐（汉字占两列）。"""
    return s + ' ' * max(0, n - sum(
        2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s))


# ---- 被检查的源码（读者的自研 BSP，已在 _check_citations.py 的 ROOTS 里）----
BSPS = [
    "D:/DEVLib/PX4APM/硬件/版本V1.4 - 从零开始手搓无人机-B站：千古八方的玩具/"
    "PX4-FMU-v6x/fmu-v6x-qgbf-20251116/fmu-v6x-qgbf",
]
CFG_REL = "src/board_config.h"
SPI_REL = "src/spi.cpp"

# board_config.h：版本号定义 + 版本号个数
DEF_RE = re.compile(
    r'^#define\s+(V6X_\d+)\s+HW_FMUM_ID\(\s*(0x[0-9a-fA-F]+)\s*\)\s*(?://\s*(.*))?$',
    re.M)
NUM_RE = re.compile(r'^#define\s+BOARD_NUM_SPI_CFG_HW_VERSIONS\s+(\d+)', re.M)
# spi.cpp：块名 + 块内器件
BLOCK_RE = re.compile(r'initSPIFmumID\(\s*(V6X_\d+)')
DEV_RE = re.compile(r'initSPIDevice\(\s*(DRV_\w+)')
EXT_RE = re.compile(r'initSPIConfigExternal\(')


def read(p):
    return io.open(p, encoding='utf-8', errors='replace').read()


def load(bsp):
    """返回 (版本号定义表, 声明的版本号个数, spi.cpp 的各块)。"""
    cfg = read(os.path.join(bsp, CFG_REL))
    spi = read(os.path.join(bsp, SPI_REL))

    defs = [(m.group(1), m.group(2), (m.group(3) or '').strip())
            for m in DEF_RE.finditer(cfg)]
    n_decl = int(NUM_RE.search(cfg).group(1))

    # 按 initSPIFmumID( 切块；每块取到下一个块之前
    marks = [(m.start(), m.group(1)) for m in BLOCK_RE.finditer(spi)]
    blocks = []
    for i, (pos, name) in enumerate(marks):
        end = marks[i + 1][0] if i + 1 < len(marks) else len(spi)
        body = spi[pos:end]
        devs = DEV_RE.findall(body)
        nexts = EXT_RE.findall(body)
        blocks.append((name, devs, len(nexts)))
    return defs, n_decl, blocks


def main():
    print('=' * 74)
    print('① 一张版本号表，三份副本')
    print('=' * 74)
    defs, n_decl, blocks = load(BSPS[0])
    print()
    print('  ① board_config.h 里的 #define V6X_*      ：**%d 个**' % len(defs))
    print('  ② board_config.h 里的 BOARD_NUM_...      ：**%d**' % n_decl)
    print('  ③ spi.cpp 里的 initSPIFmumID(...) 块      ：**%d 个**'
          % len(blocks))
    print()
    print('  ★★ 这三份**必须一致** —— 但 ② 只是个数组长度，')
    print('     **多一个少一个都能编译通过**。编译器不会替你核对这张表。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('② 每个版本号挂什么传感器（从 spi.cpp 实解出来的）')
    print('=' * 74)
    print()
    print('  %s %s %s %s' % (pad('版本号', 12), pad('SPI 器件', 12),
                            pad('外部配置', 12), '器件'))
    for name, devs, n_ext in blocks:
        short = ', '.join(d.replace('DRV_', '') for d in devs)
        print('  %s %s %s %s'
              % (pad(name, 12), pad('%d' % len(devs), 12),
                 pad('%d' % n_ext, 12), short))
    print()
    print('  ★ 注意这张表**不是文档写的，是从源码里解出来的** ——')
    print('    改的是哪一份、漏的是哪一份，一眼就能对上。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('③ 作者选的那一个版本号，声明了什么')
    print('=' * 74)
    chosen = [(i, d) for i, d in enumerate(defs) if d[1].lower() == '0x10']
    print()
    if chosen:
        i, (_name, val, comment) = chosen[0]
        print('  选中：**%s** = `%s`' % (defs[i][0], val))
        print('  它在 board_config.h 里的注释是：')
        print('    「%s」' % comment)
        blk = [b for b in blocks if b[0] == defs[i][0]]
        if blk:
            print()
            print('  它在 spi.cpp 里声明的器件：')
            for d in blk[0][1]:
                print('    · %s' % d)
        print()
        print('  ★★ 注意注释里那三个字：**from EEPROM**。')
        print('     这个版本号的**语义**是"传感器配置从 EEPROM 里读"，')
        print('     而作者是把它**写死在代码里**用的 ——')
        print('     ⇒ **借用了别人的身份**。')
        print()
        print('  ⚠️ 为什么值得单独记一笔：写死之后，')
        print('     板子上真实的那颗 EEPROM 里是什么、有没有，就**不影响启动了**；')
        print('     而一旦将来改成读 EEPROM，行为会**突然变化**。')
        print('     ⇒ 写死不是"少写几行"，是**改了一条语义**。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('④ 交叉检查 —— 三份副本的版本号集合必须一致')
    print('=' * 74)
    s_def = set(n for n, _v, _c in defs)
    s_blk = set(n for n, _d, _e in blocks)
    print()
    print('  %s %s' % (pad('项', 30), '结果'))
    print('  %s %s' % (pad('定义里有、spi.cpp 里没有的', 30),
                       '、'.join(sorted(s_def - s_blk)) or '（无）'))
    print('  %s %s' % (pad('spi.cpp 里有、定义里没有的', 30),
                       '、'.join(sorted(s_blk - s_def)) or '（无）'))
    dup = [n for n in s_blk if [b[0] for b in blocks].count(n) > 1]
    print('  %s %s' % (pad('spi.cpp 里重复出现的', 30),
                       '、'.join(sorted(set(dup))) or '（无）'))
    print()
    print('  ★★ 这三行就是本章的判据：**表有两份，就得有人对表。**')
    print('     对不上不会编译报错，只会**某块板子认不出自己的传感器**。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('自检 —— 解析器必须先自证读对了')
    print('=' * 74)
    # ① 解析器必须真的读到东西（否则"全对"是空的）
    assert len(defs) > 0, '① 一个版本号都没解析到 —— 正则不对'
    assert len(blocks) > 0, '① 一个 spi.cpp 块都没解析到 —— 正则不对'
    assert n_decl > 0, '① 版本号个数没解析到'
    print('  ✓ 解出 %d 个版本号定义、%d 个 spi 块、版本数声明 %d'
          % (len(defs), len(blocks), n_decl))
    # ② 三份必须相等 —— 这就是本章要验的东西
    assert len(defs) == n_decl == len(blocks), \
        '② 三份副本数量不一致：定义 %d / 声明 %d / 块 %d' % (
            len(defs), n_decl, len(blocks))
    print('  ✓ 三份副本数量一致（都是 %d）' % len(defs))
    # ③ 集合必须一致
    assert s_def == s_blk, '③ 版本号集合不一致'
    print('  ✓ 版本号集合一致：%s' % '、'.join(sorted(s_def)))
    # ④ 不能有重复（重复 = 后者静默覆盖前者）
    assert not dup, '④ spi.cpp 里有重复的版本号块'
    print('  ✓ 无重复块')
    # ⑤ 反向对照：故意弄坏一个名字，确认检查**会红**
    fake = s_def | {'V6X_99'}
    assert fake != s_blk, '⑤ 反向对照失效 —— 检查看不出差异'
    print('  ✓ 反向对照：把集合改成含 V6X_99 后与 %d 个块**不一致**（检查会红）'
          % len(blocks))
    # ⑥ 选中的那个版本号必须真的在表里（否则 ③ 那段是空的）
    assert chosen, '⑥ 没找到 0x10 那个版本号 —— ③ 那节会是空的'
    print('  ✓ 选中版本号 %s = %s 确实在表里' % (defs[chosen[0][0]][0], '0x10'))
    print()
    print('  ✅ 自检通过')
    print()
    print('=' * 74)
    print('口径：本脚本**直接解析真实源码**（不是抄正文里的数）：')
    print('      %s' % CFG_REL)
    print('      %s' % SPI_REL)
    print('      根目录：D:/DEVLib/PX4APM/硬件/版本V1.4 …/fmu-v6x-qgbf/')
    print('      ⇒ 源码改了，本脚本的结论会跟着变（这是刻意的）。')
    print('=' * 74)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
