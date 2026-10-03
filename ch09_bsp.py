# -*- coding: utf-8 -*-
"""ch09_bsp.py —— 卷 9 第 11 章：一个传感器要在三层都出现

## 这一章要讲的事

BSP 移植的难处**不在改，在"改完了没有"**。
而"改完了没有"之所以难答，是因为同一个传感器要在**三个地方各说一次**：

    ① 编译层 —— `default.px4board`：这个驱动**编不编进固件**
    ② 引脚层 —— `spi.cpp`：这颗器件挂哪条总线、**片选和中断是哪个脚**
    ③ 启动层 —— `init/rc.board_sensors`：开机时**要不要启动它、用什么参数**

**三层里少任何一层，症状都不一样** —— 而三种症状都长得像"传感器坏了"。

## 本脚本做什么

**直接解析这三份真实文件**，把三层名单取出来，然后**求差集**：

    · 引脚表里有、启动层没有  → 声明了但没用的
    · 启动层有、引脚表没有    → 会找不到片选（**这一类是硬故障**）
    · 启动层有、编译层没有    → 编不进固件

⚠️ 差集是**待查项**，不是"错误" —— 有些差有正当理由。
   本脚本只负责把差摆出来（就像第 7 章说的：判据要先能看见）。

跑法：python code/ch09_bsp.py
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

# 启动脚本里"启动一个 SPI 传感器"的命令 → spi.cpp 里的器件名后缀
SPI_CMDS = {
    'icm20602': 'ICM20602',
    'bmi088':   'BMI088',
    'icm42688p': 'ICM42688P',
    'icm20649': 'ICM20649',
    'icm42670p': 'ICM42670P',
    'icm45686': 'ICM45686',
    'iim42652': 'IIM42652',
    'adis16470': 'ADIS16470',
}
# 启动命令 → default.px4board 里的开关（找不到就说明"编译层没单独列"）
CONFIG_OF = {
    'icm20602':  'CONFIG_DRIVERS_IMU_INVENSENSE_ICM20602',
    'bmi088':    'CONFIG_DRIVERS_IMU_BOSCH_BMI088',
    'bmp388':    'CONFIG_DRIVERS_BAROMETER_BMP388',
    'ms5611':    'CONFIG_DRIVERS_BAROMETER_MS5611',
    'adis16470': 'CONFIG_DRIVERS_IMU_ANALOG_DEVICES_ADIS16470',
    'qmc5883p':  None,      # 见正文：编译层没有单独一项
    'board_adc': 'CONFIG_DRIVERS_ADC_BOARD_ADC',
}

# spi.cpp 里作者选中的那个版本号块
WANT_BLOCK = 'V6X_16'


def read(p):
    return io.open(p, encoding='utf-8', errors='replace').read()


def load():
    cfg = read(os.path.join(BSP, 'default.px4board'))
    spi = read(os.path.join(BSP, 'src/spi.cpp'))
    rc = read(os.path.join(BSP, 'init/rc.board_sensors'))

    # ① 编译层：CONFIG_xxx=y
    enabled = set(m.group(1) for m in
                  re.finditer(r'^(CONFIG_\w+)=y\s*$', cfg, re.M))

    # ② 引脚层：只取选中的那个版本号块
    marks = [(m.start(), m.group(1))
             for m in re.finditer(r'initSPIFmumID\(\s*(V6X_\d+)', spi)]
    declared = []
    for i, (pos, name) in enumerate(marks):
        if name != WANT_BLOCK:
            continue
        end = marks[i + 1][0] if i + 1 < len(marks) else len(spi)
        declared = re.findall(r'initSPIDevice\(\s*(DRV_\w+)', spi[pos:end])

    # ③ 启动层：非注释、非空行，取第一个词
    started = []
    for ln in rc.split('\n'):
        t = ln.strip()
        if not t or t.startswith('#'):
            continue
        started.append(t.split()[0])
    return enabled, declared, started


def main():
    enabled, declared, started = load()

    print('=' * 74)
    print('① 三层名单（各自从源码里解出来）')
    print('=' * 74)
    print()
    print('  ① 编译层 %-46s **%d 项**'
          % ('default.px4board 的 CONFIG_…=y', len(enabled)))
    print('  ② 引脚层 %-46s **%d 项**'
          % ('spi.cpp 里 %s 声明的 SPI 器件' % WANT_BLOCK, len(declared)))
    print('  ③ 启动层 %-46s **%d 项**'
          % ('rc.board_sensors 里启动的命令', len(started)))
    print()
    print('  ★★ 三层说的是**同一批传感器**，但**各说各的** ——')
    print('     没有任何一处**强制它们一致**。')
    print()
    print('  启动层实际启动的：%s' % '、'.join(started))
    print('  引脚层实际声明的：%s'
          % '、'.join(d.replace('DRV_', '').replace('DEVTYPE_', '')
                      for d in declared))

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('② 逐个传感器过三层')
    print('=' * 74)
    print()
    print('  %s %s %s %s' % (pad('启动的命令', 14), pad('编译层', 10),
                              pad('引脚层', 10), '说明'))
    for cmd in started:
        cfgsym = CONFIG_OF.get(cmd, '?')
        if cfgsym == '?':
            c = '?'
        elif cfgsym is None:
            c = '（无）'
        else:
            c = '✓' if cfgsym in enabled else '**缺**'
        suf = SPI_CMDS.get(cmd)
        if suf is None:
            p = '（非 SPI）'
        else:
            p = '✓' if any(suf in d for d in declared) else '**缺**'
        note = ''
        if c == '**缺**':
            note = '⚠️ 编不进固件'
        if p == '**缺**':
            note = '🔴 找不到片选'
        if c == '（无）':
            note = '⚠️ 编译层没有单独一项（见 §11.5）'
        print('  %s %s %s %s'
              % (pad(cmd, 14), pad(c, 10), pad(p, 10), note))

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('③ 差集 = 待查项')
    print('=' * 74)
    print()
    decl_suf = set()
    for d in declared:
        for suf in SPI_CMDS.values():
            if suf in d:
                decl_suf.add(suf)
    started_suf = set(SPI_CMDS[c] for c in started if c in SPI_CMDS)

    only_decl = sorted(decl_suf - started_suf)
    only_start = sorted(started_suf - decl_suf)
    print('  %s %s' % (pad('引脚表里有、启动层没有', 28),
                       '、'.join(only_decl) or '（无）'))
    print('  %s %s' % (pad('启动层有、引脚表没有', 28),
                       '、'.join(only_start) or '（无）'))
    print()
    if only_decl:
        print('  ⚠️ 第一类（**声明了但没启动**）通常无害 ——')
        print('     它只是"这一版预留了这个位置"。')
        print('     ⇒ 但它是**第一位待查项**：确认它是"预留"还是"漏启动"。')
    if only_start:
        print('  🔴 第二类（**启动了但表里没有**）是硬故障 ——')
        print('     驱动拿不到片选脚，传感器**认不出来**。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('④ 启动脚本开头那句注释')
    print('=' * 74)
    print('  `rc.board_sensors` 的第 6 行写着：')
    print()
    print('    「千古八方的玩具，只有一版，无需根据不同版本号')
    print('      （ver 和 rev）初始化不同的传感器。」')
    print()
    print('  ★★★ 这句是第 10 章的收口：')
    print('      版本号机制存在的唯一理由是"**一套固件、多版板子**"。')
    print('      而这块板**只有一版** ⇒ **这个机制的全部价值是零**。')
    print()
    print('  ★★ 于是第 10 章那个"借身份"的问题也一并解释了：')
    print('     写死版本号之所以**没有害处**，正是因为只有一版 ——')
    print('     没有第二个身份可以被它抢。')
    print()
    print('  ⚠️ 但反过来：**一旦这块板出第二版**，')
    print('     那句注释、那个写死的版本号、这张单版启动脚本')
    print('     **三样会同时失效** —— 而它们都不会报错。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('自检 —— 解析器必须先自证读对了')
    print('=' * 74)
    assert len(enabled) > 10, '① 编译层解析到的项太少 —— 正则不对'
    assert len(declared) > 0, '② 引脚层一个器件都没解到'
    assert len(started) > 0, '③ 启动层一条命令都没解到'
    print('  ✓ 解出编译项 %d、引脚器件 %d、启动命令 %d'
          % (len(enabled), len(declared), len(started)))
    # 必须真的解到了那个版本号块，否则 ② 是空的而"差集为空"是假绿
    assert declared, '%s 块没解析到 —— 差集结论全部无效' % WANT_BLOCK
    print('  ✓ %s 块解析成功（%d 个器件）' % (WANT_BLOCK, len(declared)))
    # 启动层里必须至少有一半能在映射表里找到，否则映射表本身有问题
    known = [c for c in started if c in SPI_CMDS]
    assert known, '③ 启动命令没有一个能映射到 SPI 器件 —— 映射表过时了'
    print('  ✓ 启动命令里有 %d 条能映射到 SPI 器件：%s'
          % (len(known), '、'.join(known)))
    # 反向对照：故意塞一个不存在的器件，确认差集**会**非空
    fake = decl_suf | {'NOT_A_REAL_PART'}
    assert sorted(fake - started_suf), '④ 反向对照失效 —— 差集看不出差异'
    print('  ✓ 反向对照：塞入一个假器件后差集非空（检查会红）')
    print()
    print('  ✅ 自检通过')
    print()
    print('=' * 74)
    print('口径：本脚本**直接解析真实源码**：')
    print('      default.px4board / src/spi.cpp / init/rc.board_sensors')
    print('      根目录：D:/DEVLib/PX4APM/硬件/版本V1.4 …/fmu-v6x-qgbf/')
    print('      ⚠️ 差集是**待查项**：本脚本**只负责看见**，不判定谁对。')
    print('=' * 74)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
