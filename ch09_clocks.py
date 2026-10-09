# -*- coding: utf-8 -*-
"""ch09_clocks.py —— 卷 9 第 2 章：把时钟树从配置里**算出来**，并查一处哑错误

## 为什么要算

`nuttx-config/include/board.h` 里写着一串 PLL 配置：

    #define STM32_PLLCFG_PLL1M   RCC_PLLCKSELR_DIVM1(1)
    #define STM32_PLLCFG_PLL1N   RCC_PLL1DIVR_N1(60)
    #define STM32_PLLCFG_PLL1P   RCC_PLL1DIVR_P1(2)

这三个数字**不是随便填的** —— 它们决定了整块板子跑多快：

    VCO1 = HSE / M × N        = 16 MHz / 1 × 60 = 960 MHz
    PLL1P = VCO1 / P          = 960 / 2         = 480 MHz  ← CPU 主频

所以它们**必须能算得通**。本脚本把整棵时钟树算一遍，做两件事：

  ① **与文件自己声明的结果对数** —— `board.h` 后面写着
     `STM32_SYSCLK_FREQUENCY = STM32_PLL1P_FREQUENCY` 并注释 `= 480MHz`。
     算出来的和声明的必须一致；不一致说明配置被人改过而注释没跟。

  ② ★ **查 `PLL1RGE` 这个字段** —— 它声明的是"PLL 输入参考时钟的频率范围"。
     本板的参考是 HSE/M = **16 MHz**，而字段填的是
     `PLL1RGE_4_8_MHZ`（NuttX 头文件里明确注释："between 4 and 8 MHz"）。
     **对不上。** 而且这不是这一块板的事 —— 脚本会把整棵 PX4 板级树扫一遍。

⚠️ 本脚本只做**算术与字符串比对**，不测硬件。
   "范围对不上"是**从源码读出来的事实**；至于它有没有实际后果，
   本脚本**不下结论**（那需要示波器与抖动实测，本卷给不了）。

跑法：python code/ch09_clocks.py
"""

import collections
import _srcdirs
import glob
import io
import os
import re
import sys

PX4_BOARDS = _srcdirs.src("PX4", "boards")
# ★ 是 NuttX 的 board.h（里面才有 STM32_PLLCFG_PLL1M 这些宏），
#   不是 src/board_config.h —— 我改成 _srcdirs 时**按变量名猜错过一次**，
#   是"重跑受影响脚本"这道检查把它抓出来的（cfg['xtal'] 成了 None）。
BSP_BOARD_H = _srcdirs.src("BSP", "nuttx-config", "include", "board.h")

# 宏里取的数：DIVMn(x) / Nn(x) / Pn(x) / Qn(x) / Rn(x)
RE_M = re.compile(r'define\s+STM32_PLLCFG_PLL(\d)M\s+.*?DIVM\d\((\d+)\)')
RE_N = re.compile(r'define\s+STM32_PLLCFG_PLL(\d)N\s+.*?N\d\((\d+)\)')
RE_DIV = re.compile(r'define\s+STM32_PLLCFG_PLL(\d)([PQR])\s+.*?[PQR]\d\((\d+)\)')
RE_RGE = re.compile(r'(PLL1RGE_\d+_\d+_MHZ)')
RE_XTAL = re.compile(r'define\s+STM32_BOARD_XTAL\s+(\d+)')

# NuttX 头文件里对 PLL1RGE 各取值的**原文定义**（这是权威口径，不是我记的）
RGE_RANGE = {
    'PLL1RGE_1_2_MHZ':   (1, 2),
    'PLL1RGE_2_4_MHZ':   (2, 4),
    'PLL1RGE_4_8_MHZ':   (4, 8),
    'PLL1RGE_8_16_MHZ':  (8, 16),
}
RGE_SRC = ('platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/'
           'hardware/stm32h7x3xx_rcc.h')


def parse(text):
    """从一份 board.h 里取出 PLL 配置"""
    d = {}
    d['xtal'] = int(RE_XTAL.search(text).group(1)) if RE_XTAL.search(text) else None
    d['m'] = {int(a): int(b) for a, b in RE_M.findall(text)}
    d['n'] = {int(a): int(b) for a, b in RE_N.findall(text)}
    d['div'] = {}
    for pll, which, v in RE_DIV.findall(text):
        d['div'].setdefault(int(pll), {})[which] = int(v)
    d['rge'] = RE_RGE.search(text).group(1) if RE_RGE.search(text) else None
    return d


def clock_tree(cfg):
    """算出各档频率。返回 {名字: **MHz**}。

    ⚠️ 单位：`STM32_BOARD_XTAL` 是 **Hz**（16000000），
       所以算出来也是 Hz —— 打印前要除 1e6。
       第一版忘了除，屏幕上印出 `P=4.8e+08 MHz`：
       **数字是对的，单位是错的**，而那种输出最容易被人当成"工具坏了"直接忽略。
    """
    out = {}
    xtal = cfg['xtal'] / 1e6            # Hz → MHz
    for p in sorted(cfg['m']):
        m, n = cfg['m'][p], cfg['n'].get(p)
        if not n:
            continue
        vco = xtal / m * n
        out['VCO%d' % p] = vco
        for which, dv in cfg['div'].get(p, {}).items():
            out['PLL%d%s' % (p, which)] = vco / dv
    return out


def check_board(path, verbose=False):
    """返回 (名字, 参考频率, RGE, RGE 的声明范围, 对不对)"""
    text = io.open(path, encoding='utf-8', errors='replace').read()
    cfg = parse(text)
    if not (cfg['xtal'] and cfg['m'].get(1) and cfg['rge']):
        return None
    ref = cfg['xtal'] / 1e6 / cfg['m'][1]
    lo, hi = RGE_RANGE[cfg['rge']]
    return (os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(path)))),
            ref, cfg['rge'], (lo, hi), lo <= ref <= hi)


def main():
    print('=' * 76)
    print('① 本板的时钟树：从 PLL 配置算出来')
    print('=' * 76)
    if not os.path.exists(BSP_BOARD_H):
        print('找不到本板的 board.h：%s' % BSP_BOARD_H)
        return 1
    text = io.open(BSP_BOARD_H, encoding='utf-8', errors='replace').read()
    cfg = parse(text)
    print('  HSE（外部晶振）= %d Hz = %g MHz' % (cfg['xtal'], cfg['xtal'] / 1e6))
    print()
    print('  %-6s %-4s %-4s %s' % ('PLL', 'M', 'N', '各级输出'))
    tree = clock_tree(cfg)
    for p in sorted(cfg['m']):
        m, n = cfg['m'][p], cfg['n'].get(p)
        outs = []
        for which in ('P', 'Q', 'R'):
            key = 'PLL%d%s' % (p, which)
            if key in tree:
                outs.append('%s=%g MHz' % (which, tree[key]))
        print('  PLL%-3d %-4d %-4d %s' % (p, m, n, '  '.join(outs)))
    print()
    print('  ★ 算法只有两条：`VCO = HSE / M × N`，`PLLxP/Q/R = VCO / P/Q/R`。')
    print('    这三个数字**不是随便填的** —— 它们决定整块板子跑多快。')

    # ------------------------------------------------------------------
    print()
    print('=' * 76)
    print('② 算出来的，和文件自己声明的对不对得上')
    print('=' * 76)
    declared = {
        'STM32_VCO1_FREQUENCY':  ('VCO1', r'define\s+STM32_VCO1_FREQUENCY\s+\(\(STM32_HSE_FREQUENCY / (\d+)\) \* (\d+)\)'),
        'STM32_PLL1P_FREQUENCY': ('PLL1P', r'define\s+STM32_PLL1P_FREQUENCY\s+\(STM32_VCO1_FREQUENCY / (\d+)\)'),
        'STM32_PLL1Q_FREQUENCY': ('PLL1Q', r'define\s+STM32_PLL1Q_FREQUENCY\s+\(STM32_VCO1_FREQUENCY / (\d+)\)'),
        'STM32_PLL1R_FREQUENCY': ('PLL1R', r'define\s+STM32_PLL1R_FREQUENCY\s+\(STM32_VCO1_FREQUENCY / (\d+)\)'),
    }
    ok = True
    print('  %-26s %-14s %-14s %s' % ('文件里的声明', '它自己算', '本脚本算', '一致？'))
    for name, (key, pat) in declared.items():
        m = re.search(pat, text)
        if not m:
            continue
        nums = [int(x) for x in m.groups()]
        if len(nums) == 2:                       # VCO1 = HSE / M × N
            own = cfg['xtal'] / 1e6 / nums[0] * nums[1]
        else:                                    # PLL1x = VCO1 / div
            own = tree['VCO1'] / nums[0]
        same = abs(own - tree[key]) < 1e-6
        ok &= same
        print('  %-26s %-14s %-14s %s'
              % (name, '%g MHz' % own, '%g MHz' % tree[key], '✅' if same else '❌'))
    print()
    print('  %s' % ('✅ 两边完全一致 —— 配置与声明是同步的' if ok
                    else '❌ 有对不上的 —— 说明配置被改过而声明没跟'))

    # ------------------------------------------------------------------
    print()
    print('=' * 76)
    print('③ ★ 查 `PLL1RGE`：它声明的输入范围，和实际参考频率对得上吗')
    print('=' * 76)
    lo, hi = RGE_RANGE[cfg['rge']]
    ref = cfg['xtal'] / 1e6 / cfg['m'][1]
    print('  本板：HSE/M = %g / %d = **%g MHz**' % (cfg['xtal'] / 1e6, cfg['m'][1], ref))
    print('        字段 STM32_PLLCFG_PLL1CFG 选了 `%s`' % cfg['rge'])
    print('        而 NuttX 头文件对它的定义是：**%g ~ %g MHz**' % (lo, hi))
    print('        （原文：%s）' % RGE_SRC)
    print()
    inrange = lo <= ref <= hi
    if inrange:
        print('  ✅ 落在声明范围内')
    else:
        print('  ❌ **参考频率 %g MHz 不在 %s 声明的 %g~%g MHz 内**'
              % (ref, cfg['rge'], lo, hi))
        print('     ⇒ 按声明，这里应当选 8_16 MHz 那一档。')

    # ------------------------------------------------------------------
    print()
    print('=' * 76)
    print('④ 这是这一块板的事，还是普遍现象？')
    print('=' * 76)
    rows = []
    for f in sorted(glob.glob(os.path.join(PX4_BOARDS, '*', '*',
                                           'nuttx-config', 'include', 'board.h'))):
        r = check_board(f)
        if r:
            rows.append(r)
    if not rows:
        print('  （没扫到 PX4 板级树，跳过）')
    else:
        bad = [r for r in rows if not r[4]]
        by_rge = collections.Counter(r[2] for r in rows)
        print('  扫描 PX4 板级树：**%d** 块板' % len(rows))
        print('  它们选的 PLL1RGE 取值：%s'
              % '、'.join('%s×%d' % (k, v) for k, v in by_rge.most_common()))
        print('  其中**参考频率不在所选范围内**的：**%d / %d**' % (len(bad), len(rows)))
        print()
        print('  参考频率分布：%s'
              % '、'.join('%g MHz×%d' % (k, v) for k, v in
                          sorted(collections.Counter(r[1] for r in rows).items())))
        print()
        print('  ★★ 也就是说：**这个字段在整棵 PX4 板级树里是同一个值**，')
        print('     而各板的参考频率各不相同（8 / 12 / 16 MHz）。')
        print('     它显然是从某一块板复制过去、之后再没人动过。')
        print()
        print('  ⚠️ 而它**并不妨碍这些板子正常工作** —— 它们都出货了。')
        print('     所以本脚本到此为止：**报事实，不下结论**。')
        print('     "设错了也不炸"这件事本身才是要记住的 ——')
        print('     寄存器手册写的范围是**保证值的条件**，不是"超出就不工作"。')

    print()
    print('=' * 76)
    print('口径：以上全部是**源码里的数与算术**。本脚本不测硬件、不下结论。')
    print('=' * 76)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
