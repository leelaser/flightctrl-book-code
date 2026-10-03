# -*- coding: utf-8 -*-
"""ch09_pinmap.py —— 卷 9：从 BSP 源码里把引脚分配读出来，并查三类错

## 为什么是"读"而不是"测"

硬件卷给不了实测（没有板子就量不了波形）。但 **BSP 源码是给得了的** ——
它描述的是"固件怎么认识这块板"，不是"板子怎么做出来"（**不是可制造文件**）。

于是本卷的可复现性走这条路：**每一条关于引脚的结论，都能在源码里查到**。
本脚本就是那个"查"的动作，而且它顺手做三件机器擅长、人容易看漏的事：

  ① **注释与代码对不对得上**
     `#define GPIO_nLED_RED  /* PE3 */  (…|GPIO_PORTE|GPIO_PIN3)`
     注释说 PE3，代码说 PORTE/PIN3 —— 两边应当一致。
     不一致多半是**改了代码没改注释**（或反过来），是典型的"哑错误"。

  ② **同一个引脚有没有被两个功能占**
     一个物理引脚在同一份配置里被两处定义引用，通常意味着冲突。

  ③ **串口映射与板级配置对不对得上**
     `default.px4board` 里的 `CONFIG_BOARD_SERIAL_*` 与
     `nuttx-config/include/board.h` 里的 UART 配置应当能互相印证。

⚠️ 口径：本脚本只读**源码**，不碰 EDA 工程、不输出任何可制造信息。
   它读的是 C 代码里的宏，不是原理图。

跑法：python code/ch09_pinmap.py [--bsp <BSP 根路径>]
"""

import collections
import _srcdirs
import io
import os
import re
import sys

DEFAULT_BSP = _srcdirs.src("BSP")

# `#define GPIO_xxx /* PXn */ (…|GPIO_PORTX|GPIO_PINn…)`
RE_DEFINE = re.compile(
    r'^\s*#\s*define\s+(?P<name>GPIO_\w+)\s*'
    r'(?:/\*\s*(?P<pin_comment>[A-Z]+\d+)\s*\*/)?\s*'
    r'(?P<body>\(.*?\))\s*(?P<trail>//.*)?$')
RE_PORT = re.compile(r'GPIO_PORT([A-Z])')
RE_PIN = re.compile(r'GPIO_PIN(\d+)')
RE_ANY_DEFINE = re.compile(r'^\s*#\s*define\s+GPIO_\w+')
PORT_BITS = 16          # STM32 每个 GPIO 端口 16 个引脚


def parse_board_config(path):
    """解析 board_config.h。

    返回 (rows, n_total)：
      rows    —— [(名字, 注释里的引脚, 代码里的引脚, 行号)]
      n_total —— 文件里**以 `#define GPIO_` 开头的行数**（分母）

    ★★ 分母必须报出来。实测这个文件有 **53** 条 `#define GPIO_`，
      而带括号、能解出引脚的只有 34 条 —— 另外 18 条是**别名**：

          #  define GPIO_nLED_RED    GPIO_TRACED0        ← 把一个名字指向另一个名字

      它们没有 `GPIO_PORTx|GPIO_PINn`，本来就解不出物理引脚。
      **不报分母的话，"解析出 34 条"看着像"全解出来了"。**

    ★★ 注释里的引脚写作 `PE3`（P + 端口 E + 引脚 3），而代码里是
      `GPIO_PORTE|GPIO_PIN3` —— 拼出来是 `E3`。
      两边**口径差一个前缀 `P`**，第一版直接比字符串，于是
      **27 条全部报"不一致"**，而实际一条都没错。这是纯粹的假阳性。
    """
    rows, n_total = [], 0
    for i, line in enumerate(io.open(path, encoding='utf-8',
                                     errors='replace').read().split('\n'), 1):
        if RE_ANY_DEFINE.match(line):
            n_total += 1
        m = RE_DEFINE.match(line)
        if not m:
            continue
        body = m.group('body')
        pm, pn = RE_PORT.search(body), RE_PIN.search(body)
        code_pin = ('%s%s' % (pm.group(1), pn.group(1))) if (pm and pn) else None
        c = m.group('pin_comment')
        if c and c.startswith('P') and len(c) > 1 and c[1].isalpha():
            c = c[1:]                      # `PE3` → `E3`，与代码侧同一口径
        rows.append((m.group('name'), c, code_pin, i))
    return rows, n_total


def parse_px4board_serial(path):
    """`default.px4board` 里的 CONFIG_BOARD_SERIAL_* → 设备名"""
    out = {}
    if not os.path.exists(path):
        return out
    for line in io.open(path, encoding='utf-8', errors='replace'):
        m = re.match(r'CONFIG_BOARD_SERIAL_(\w+)="([^"]+)"', line.strip())
        if m:
            out[m.group(1)] = m.group(2)
    return out


def main():
    bsp = DEFAULT_BSP
    if '--bsp' in sys.argv:
        bsp = sys.argv[sys.argv.index('--bsp') + 1]
    bc = os.path.join(bsp, 'src', 'board_config.h')
    if not os.path.exists(bc):
        print('找不到 %s' % bc)
        print('（用 --bsp <BSP 根路径> 指一个）')
        return 1

    print('=' * 74)
    print('从 BSP 源码读引脚分配')
    print('=' * 74)
    print('源码：%s' % bc)

    rows, n_total = parse_board_config(bc)
    with_comment = [r for r in rows if r[1]]
    with_code = [r for r in rows if r[2]]
    print()
    print('  文件里有 **%d** 条 `#define GPIO_*`' % n_total)
    print('    能解出引脚（带括号、含 GPIO_PORTx|GPIO_PINn）：%d 条' % len(with_code))
    print('    其中带 `/* PXn */` 注释的：%d 条' % len(with_comment))
    print('    其余 %d 条是**别名**（`#define A B` 形式，指向另一个名字），'
          '本来就没有物理引脚可比' % (n_total - len(rows)))

    # ★ 分母要拿**文件里实际有多少条**来卡，不是拍一个数。
    #   覆盖率太低说明解析规则与文件格式脱节了 —— 那时"报绿"是假的。
    if len(rows) < 0.5 * n_total:
        print()
        print('❌ 解析到 %d/%d，不到一半 —— 多半是解析规则与文件格式对不上了，'
              '别把"没解析到"当成"没问题"' % (len(rows), n_total))
        return 1

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('① 注释与代码对不对得上')
    print('=' * 74)
    bad = [r for r in rows if r[1] and r[2] and r[1] != r[2]]
    if bad:
        print('  ❌ 有 %d 条**注释与代码不一致**：' % len(bad))
        for name, c, code, ln in bad:
            print('     %s:%d  %s  注释说 %s，代码是 %s'
                  % (os.path.basename(bc), ln, name, c, code))
        print('     ⇒ 多半是"改了代码没改注释"。**这种错不报错**，'
              '只是后来的人按注释去查线会查错。')
    else:
        print('  ✅ %d 条**注释与代码完全一致** —— 说明这份 BSP 的注释是跟着代码走的'
              % len(with_comment))
        print('     （反过来，如果哪天有人改了代码没改注释，这一栏就会红。'
              '这类错不报错、也不影响编译，只坑后来查线的人。）')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('② 同一个引脚被几个 define 用到')
    print('=' * 74)
    by_pin = collections.defaultdict(list)
    for name, _c, code, ln in rows:
        if code:
            by_pin[code].append((name, ln))
    multi = {p: v for p, v in by_pin.items() if len(v) > 1}
    print('  用到的引脚共 %d 个，其中被**多个 define** 引用的 %d 个：'
          % (len(by_pin), len(multi)))
    for p in sorted(multi, key=lambda x: (x[0], int(x[1:]))):
        names = '、'.join(n for n, _ in multi[p])
        print('     %-5s ← %s' % (p, names))
    print()
    print('  ★ 这一栏**不一定是错**，要分开看：')
    print('    · 同一个引脚先当输入再当输出（如 `GPIO_nARMED_INIT` / `GPIO_nARMED`）')
    print('      ⇒ 正常，那是**同一根线的两种用途**，靠 IO 的模式切换')
    print('    · 名字毫不相干的两个外设指向同一脚 ⇒ **要人工看一眼**，')
    print('      多半是复制模板时忘了改')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('③ 串口映射：`default.px4board` 说了什么')
    print('=' * 74)
    pb = os.path.join(bsp, 'default.px4board')
    serial = parse_px4board_serial(pb)
    if serial:
        print('  %-10s %s' % ('用途', '设备节点'))
        for k in sorted(serial):
            print('  %-10s %s' % (k, serial[k]))
        # 自洽性：同一个设备节点不该被两个用途占用
        rev = collections.defaultdict(list)
        for k, v in serial.items():
            rev[v].append(k)
        clash = {v: ks for v, ks in rev.items() if len(ks) > 1}
        print()
        if clash:
            print('  ❌ 有设备节点被两个用途占用：')
            for v, ks in clash.items():
                print('     %s ← %s' % (v, '、'.join(ks)))
        else:
            print('  ✅ %d 个用途各占一个设备节点，没有冲突' % len(serial))
        print()
        print('  ★ 这张表是**板级配置**，不是驱动代码 —— 换个口只要改这里。')
        print('    第 9 章那版 V1.4 就把 Telem2 从 mavlink 口改成了')
        print('    "接光流与激光测距"的普通口，改的正是这一层（见 §9）。')
    else:
        print('  （没读到 CONFIG_BOARD_SERIAL_*）')

    print()
    print('=' * 74)
    print('口径：以上全部来自 **BSP 源码**（C 宏与板级配置）。')
    print('      本脚本不读 EDA 工程、不输出任何可制造信息。')
    print('=' * 74)
    return 0


if __name__ == '__main__':
    sys.exit(main())
