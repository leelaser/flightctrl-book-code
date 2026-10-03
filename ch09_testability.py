# -*- coding: utf-8 -*-
"""ch09_testability.py —— 卷 9 第 16 章：可测试性是设计输入

## 这一章要讲的事

"测试"看起来是**产线的事**。但有一类测试**产线补不了** ——
因为它要求板子上**本来就有**某些东西。

而"板子上有没有这些东西"，**在板级支持包里看得见**。

## 本脚本做什么

**直接解析 `board_config.h`**，数出这块板子的两种能力：

    ① **能关** —— 有几个电源域可以被单独使能/关断
    ② **能看** —— 有几路电压被接到 ADC 上监测
    ③ **能判断** —— 有几个"电源有效"的输入信号

这三种能力的**数量**，不是软件写出来的，是**画板子的人定下的**。

## 本章的判据

**"能关"和"能看"是可测试性的两个要素，
而它们都是引脚分配阶段就决定的事 —— 事后加不上去。**

跑法：python code/ch09_testability.py
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

# ① 电源域使能脚：名字里带 _EN 的**输出**脚（低有效的是 _nEN）
EN_RE = re.compile(
    r'^#define\s+(GPIO_\w*(?:_nEN|_EN))\s+/\*\s*(\w+)\s*\*/\s*'
    r'\(GPIO_OUTPUT', re.M)
# ② 被 ADC 监测的电压
ADC_RE = re.compile(r'^#define\s+(ADC_\w*_CHANNEL)\s+/\*\s*(\w+)\s*\*/', re.M)
# ③ 电源有效信号：名字里带 POWER_IN 的输入脚
VALID_RE = re.compile(
    r'^#define\s+(GPIO_nPOWER_IN_\w+)\s+/\*\s*(\w+)\s*\*/', re.M)


def rd(p):
    return io.open(p, encoding='utf-8', errors='replace').read()


def main():
    cfg = rd(os.path.join(BSP, 'src/board_config.h'))

    print('=' * 74)
    print('① 能关：几个电源域可以被单独控制')
    print('=' * 74)
    print()
    ens = EN_RE.findall(cfg)
    print('  %s %s %s' % (pad('使能脚', 34), pad('引脚', 8), '极性'))
    for name, pin in ens:
        pol = '低有效' if name.endswith('_nEN') else '高有效'
        print('  %s %s %s' % (pad(name, 34), pad(pin, 8), pol))
    print()
    n_low = len([n for n, _p in ens if n.endswith('_nEN')])
    print('  ⇒ **%d 个可单独控制的电源域**（其中 %d 个是低有效的使能脚）。'
          % (len(ens), n_low))
    print()
    print('  ★★ 注意极性那一列**不是一个细节**：')
    print('     "低有效"意味着**上电默认是开着的**（靠上拉），')
    print('     固件要主动拉低才能关掉它 ——')
    print('     ⇒ **断电这件事在硬件上是"默认做不到"的。**')
    print('       这正是"可测试性"最难的地方：**故障时你想要的是"断得开"。**')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('② 能看：几路电压被接到 ADC 上')
    print('=' * 74)
    print()
    adcs = ADC_RE.findall(cfg)
    print('  %s %s' % (pad('监测项', 44), '引脚'))
    for name, pin in adcs:
        print('  %s %s' % (pad(name, 44), pin))
    print()
    print('  ⇒ **%d 路**被引到 ADC 上，分成三类：' % len(adcs))
    print()
    # ⚠️ 过滤"版本/修订"那两路**不能只写 'HW_V'** ——
    #   修订那一路叫 `HW_REV_`，不含 `HW_V`，会漏掉一个。
    n_sen = len([x for x, _p in adcs if '3V3_SENSORS' in x])
    n_ver = len([x for x, _p in adcs if x.startswith('ADC_HW_')])
    print('    · **%d 路**传感器供电电压（用来判断哪一路塌了）' % n_sen)
    print('    · **%d 路**系统电压（主 5 V 与两路基准）'
          % (len(adcs) - n_sen - n_ver))
    print('    · **%d 路**版本/修订电阻的分压（板子"报身份"用的，见第 10 章）'
          % n_ver)
    assert n_sen + n_ver < len(adcs), '② 分类把全部通道都吃掉了 —— 过滤条件写错了'
    print()
    print('  ★★ 它们的意义不是"固件能用"，而是**"你能看见"**：')
    print('     板子上的电有没有、够不够、哪一路塌了 ——')
    print('     这些在**没有示波器**的时候，只能靠这几路 ADC。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('③ 能判断：几个"电源有效"输入')
    print('=' * 74)
    print()
    val = VALID_RE.findall(cfg)
    print('  %s %s' % (pad('有效信号', 34), '引脚'))
    for name, pin in val:
        print('  %s %s' % (pad(name, 34), pin))
    br = re.search(r'^#define\s+BOARD_NUMBER_BRICKS\s+(\d+)', cfg, re.M)
    print()
    print('  ⇒ **%d 个"电源有效"输入**。' % len(val))
    if br:
        print('     而配置里声明的**电源路数**是 **%s**。' % br.group(1))
        print()
        print('  ★★ 这两个数**不一定相等**，而这正是设计的取舍：')
        print('     两个"电源有效"信号 + 一个 USB 有效信号，')
        print('     覆盖了"电从哪里来"的三种情形。')
        print()
        print('  ★★★ 判据：**"能关" + "能看" + "能判断"** ——')
        print('       这三样凑齐了，一块板子才**可测**。')
        print('       而这三样**都是引脚分配阶段定的**，'
              '软件只能使用、不能创造。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('④ 于是"可测试性"是什么')
    print('=' * 74)
    print()
    print('  %s %s' % (pad('要素', 16), '在这一块板上的体现'))
    rows = [
        ('能关',   '%d 个电源域使能脚' % len(ens)),
        ('能看',   '%d 路电压接 ADC' % len(adcs)),
        ('能判断', '%d 个电源有效输入' % len(val)),
    ]
    for a, b in rows:
        print('  %s %s' % (pad(a, 16), b))
    print()
    print('  ★★★ 这三样**没有一样是"测试部门"能加上去的** ——')
    print('       它们全都是**画板子的时候**留下的引脚。')
    print('       ⇒ 所以"可测试性"是一条**设计输入**，')
    print('         而不是一道**生产工序**。')
    print()
    print('  ⚠️ 反过来看："这块板子好不好测"这个问题，')
    print('     **可以在原理图阶段就被回答** —— ')
    print('     而在那之后，答案就不会再变了。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('自检 —— 解析器必须先自证读对了')
    print('=' * 74)
    assert ens, '① 一个使能脚都没解到 —— 正则不对'
    assert adcs, '② 一路 ADC 都没解到'
    assert val, '③ 一个电源有效信号都没解到'
    print('  ✓ 解出使能脚 %d、ADC 监测 %d、有效信号 %d'
          % (len(ens), len(adcs), len(val)))
    # 三类都必须是"非空但不多"的规模 —— 数字太大会说明正则抓过头了
    assert 2 <= len(ens) <= 20, '① 使能脚数量不合常理（%d）' % len(ens)
    assert 2 <= len(adcs) <= 20, '② ADC 数量不合常理（%d）' % len(adcs)
    assert 1 <= len(val) <= 10, '③ 有效信号数量不合常理（%d）' % len(val)
    print('  ✓ 三类数量都在合理范围（没有抓过头）')
    # 极性统计必须真的分出了两种，否则"低有效"那段结论不成立
    assert 0 < n_low < len(ens), '① 极性没有分成两类 —— 结论不成立'
    print('  ✓ 使能脚极性分成两类（低有效 %d / 高有效 %d）'
          % (n_low, len(ens) - n_low))
    # 三大类的引脚不能重复（同一个脚不该既当使能又当监测）
    s_en = set(p for _n, p in ens)
    s_ad = set(p for _n, p in adcs)
    inter = s_en & s_ad
    assert not inter, '引脚重复：%s' % '、'.join(sorted(inter))
    print('  ✓ 使能脚与 ADC 脚无重叠（%d 个脚各自独立）'
          % (len(s_en | s_ad)))
    print()
    print('  ✅ 自检通过')
    print()
    print('=' * 74)
    print('口径：本脚本**直接解析读者的 BSP 源码** `src/board_config.h`。')
    print('      这三类引脚是**板子设计时定的** —— 换一版板子，数就变。')
    print('=' * 74)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
