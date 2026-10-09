# -*- coding: utf-8 -*-
"""ch09_chain.py —— 卷 9 第 12 章：从引脚到 probe()，一颗传感器要过五道门

## 这一章要讲的事

"固件认得这颗传感器"这句话，背后是**一条链**：

    ① 一个**编号**（DEVTYPE）：驱动和板子约定"这颗器件叫 0x66"
    ② 一个**片选**（CS）：板子告诉固件"它挂在哪个脚上"
    ③ 一个**中断**（DRDY）：数据好了用哪个脚通知
    ④ 一次**读 ID**：驱动去问芯片"你是谁"
    ⑤ 一次**比对**：读回来的和期望的一样，才放行

**五道门，任何一道不过，传感器都不会上线。**

## 本脚本做什么 —— 锚点核对

这条链跨了**两个工程、六个文件**（读者的 BSP + PX4 源码树）。
脚本把每一跳的 `文件:行号` **连同那一行的内容**钉住：

    去文件里**把那一行读出来**，核对它是否还包含该有的内容。

⇒ 行号漂了、代码改了，这里会**红**。
   （这正是本丛书在别处栽过的那个坑：引用行号会漂，
     而"引用看起来还在"和"引用还对"是两件事。）

跑法：python code/ch09_chain.py
"""

import io
import _srcdirs
import os
import unicodedata


def pad(s, n):
    """按显示宽度补齐（汉字占两列）。"""
    return s + ' ' * max(0, n - sum(
        2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s))


BSP = _srcdirs.src("BSP")
PX4 = _srcdirs.src("PX4")

# ---- 链条锚点：(门, 说明, 文件, 行号, 该行必须包含的子串) ----
CHAIN = [
    ('①', '驱动与板子约定的"编号"',
     os.path.join(PX4, 'src/drivers/drv_sensor.h'), 143,
     'DRV_GYR_DEVTYPE_BMI088'),
    ('②', '板子告诉固件：它挂在哪',
     os.path.join(BSP, 'src/spi.cpp'), 189,
     'DRV_GYR_DEVTYPE_BMI088'),
    ('②', '   └ 片选脚',
     os.path.join(BSP, 'src/spi.cpp'), 189,
     'SPI::CS'),
    ('②', '   └ 中断脚',
     os.path.join(BSP, 'src/spi.cpp'), 189,
     'SPI::DRDY'),
    ('③', '启动命令把它翻成那个编号',
     os.path.join(PX4, 'src/drivers/imu/bosch/bmi088/bmi088_main.cpp'), 68,
     'DRV_GYR_DEVTYPE_BMI088'),
    ('④', '驱动按编号分派到具体实现',
     os.path.join(PX4, 'src/drivers/imu/bosch/bmi088/BMI088.cpp'), 46,
     'devid_driver_index == DRV_GYR_DEVTYPE_BMI088'),
    ('⑤', 'probe()：问芯片"你是谁"',
     os.path.join(PX4, 'src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp'), 82,
     'probe()'),
    ('⑤', '   └ 读哪个寄存器',
     os.path.join(PX4, 'src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp'), 84,
     'GYRO_CHIP_ID'),
    ('⑤', '   └ 跟什么比',
     os.path.join(PX4, 'src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp'), 86,
     'chipid != ID'),
    ('⑤', '   └ 期望值是多少（**黑芯片改的就是这一行**）',
     os.path.join(PX4, 'src/drivers/imu/bosch/bmi088/'
                       'Bosch_BMI088_Gyroscope_Registers.hpp'), 52,
     'constexpr uint8_t ID = 0x0F'),
]


def main():
    print('=' * 74)
    print('① 链条：五道门，跨两个工程、六个文件')
    print('=' * 74)
    print()
    bad = []
    for i, (gate, what, path, line, want) in enumerate(CHAIN, 1):
        name = os.path.basename(path)
        if not os.path.isfile(path):
            bad.append((i, '文件不存在：%s' % path))
            continue
        lines = io.open(path, encoding='utf-8', errors='replace').read().split('\n')
        if line > len(lines):
            bad.append((i, '行号 %d 越界（文件只有 %d 行）' % (line, len(lines))))
            continue
        got = lines[line - 1].strip()
        mark = '✓'
        if want not in lines[line - 1]:
            mark = '✗'
            bad.append((i, '%s:%d 里没有 `%s`' % (name, line, want)))
        print('  %s %s %s' % (gate, mark, what))
        print('       %s:%d' % (name, line))
        print('       %s' % (got[:66] + ('…' if len(got) > 66 else '')))
        print()
    if bad:
        print('  ❌ 链条断了：')
        for i, why in bad:
            print('     第 %d 跳：%s' % (i, why))
        print()
        print('  ★ 这正是本章要的：**引用行号会漂**。')
        print('    只要有人动过这些文件，这里就必须重新核 ——')
        print('    而不是"当时核过，所以现在还对"。')
        return 1

    # ------------------------------------------------------------------
    print('=' * 74)
    print('② 这条链上，每一道门失败会怎样')
    print('=' * 74)
    print()
    rows = [
        ('① 编号',   '驱动和板子的编号对不上', '启动时报"命令不支持"'),
        ('② 片选',   '片选脚写错 / 没声明',    '读不到数据（总线没选中它）'),
        ('③ 中断',   '中断脚写错 / 没声明',    '有数据但不触发（靠轮询兜着）'),
        ('④ 分派',   '编号不认识',            '驱动直接不认这个器件'),
        ('⑤ 比对',   '**读回来的 ID ≠ 期望值**', '🔴 驱动**拒绝上线**（见第 13 章）'),
    ]
    print('  %s %s %s' % (pad('门', 12), pad('怎么坏', 26), '症状'))
    for a, b, c in rows:
        print('  %s %s %s' % (pad(a, 12), pad(b, 26), c))
    print()
    print('  ★★ 注意最后一行：**它和前四行完全不同**。')
    print('     前四行是"配置写错了"，最后一行是"**配置没错，是芯片不认**"。')
    print('     ⇒ 而第五道门恰恰是最容易被误解成"传感器坏了"的那一道。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('③ 第五道门长什么样')
    print('=' * 74)
    p = os.path.join(PX4, 'src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp')
    src = io.open(p, encoding='utf-8', errors='replace').read().split('\n')
    print()
    for n in range(82, 93):
        print('    %4d  %s' % (n, src[n - 1]))
    print()
    print('  ★★ 就看这 11 行：读一个寄存器、和一个常量比、不等就返回错误。')
    print('     **整个"固件认得这颗传感器吗"就压缩在这几行里。**')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('自检 —— 锚点核对器必须先自证在工作')
    print('=' * 74)
    # ① 锚点数量必须够（太少说明配置被删空了）
    assert len(CHAIN) >= 8, '① 锚点太少'
    print('  ✓ 共 %d 个锚点' % len(CHAIN))
    # ② 每个锚点都要真的读到内容（空行不算）
    n_ok = 0
    for _g, _w, path, line, _want in CHAIN:
        ls = io.open(path, encoding='utf-8', errors='replace').read().split('\n')
        if line <= len(ls) and ls[line - 1].strip():
            n_ok += 1
    assert n_ok == len(CHAIN), '② 有锚点落在了空行上（行号已经漂了）'
    print('  ✓ %d 个锚点全部落在非空行上' % n_ok)
    # ③ **正向已过**；再做一个**反向对照**：故意改一个期望值，确认会红
    _g, _w, path, line, _want = CHAIN[-1]
    ls = io.open(path, encoding='utf-8', errors='replace').read().split('\n')
    fake_want = 'constexpr uint8_t ID = 0xEE'
    assert fake_want not in ls[line - 1], '③ 反向对照失效 —— 假期望值居然对上了'
    print('  ✓ 反向对照：把期望值改成 `%s` 后**对不上**（核对会红）' % fake_want)
    # ④ 两个工程都必须在（少一个就说明路径变了）
    for root, tag in ((BSP, '读者的 BSP'), (PX4, 'PX4 源码树')):
        assert os.path.isdir(root), '④ %s 找不到：%s' % (tag, root)
    print('  ✓ 两个工程都在：读者的 BSP、PX4 源码树')
    print()
    print('  ✅ 自检通过 —— 这条链的每一跳都还钉着')
    print()
    print('=' * 74)
    print('口径：本脚本**读两个真实工程的源码**，逐行核对锚点。')
    print('      BSP ：' + BSP)
    print('      PX4 ：' + PX4)
    print('      ⚠️ 链上任何一环被改动，本脚本会**红** —— 这是刻意的。')
    print('=' * 74)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
