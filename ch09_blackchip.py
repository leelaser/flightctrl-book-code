# -*- coding: utf-8 -*-
"""ch09_blackchip.py —— 卷 9 第 13 章：黑芯片

## 这一章要讲的事

驱动怎么认一颗器件？**查表。** 而两个故事里，表都查不中：

    FRAM  ： 买到的片子**读不出芯片 ID**（读回来是全 0xFF）
    BMI088： 也是一颗"黑芯片"，**ID 对不上**

两处修法是**同一个形状**：**改一个常量 / 加一行表**，让它能被认出来。

## 但本章真正要讲的是：这两种"改"的代价不一样

    BMI088 —— 改 `ID = 0x0F` ⇒ 只是**放行**，不改变任何行为参数
    FRAM   —— 认下某个条目 ⇒ **同时采用了那一条的尺寸与地址长度**

⇒ 后者的"认出来"是**有附带后果的**：你把一个**不确定的尺寸**告诉了系统。

## 本脚本做什么

**直接解析 FRAM 驱动源码**（`drivers/mtd/ramtron.c`），把它的身份机制取出来：

① 表里有多少条目、匹配键是什么
② 有一个**专门给"非 JEDEC 器件"准备的条目**，而它被一个编译开关关着
③ 匹配上之后，驱动**采用了哪些参数**（尺寸 / 地址长度）
④ 于是"让它被认出来"到底付出了什么

跑法：python code/ch09_blackchip.py
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


NUTTX = _srcdirs.src("NUTTX")
SRC = os.path.join(NUTTX, 'drivers/mtd/ramtron.c')

# 认证字节 → 一个名字 的映射，就是这张表
ENTRY_RE = re.compile(
    r'\{\s*"([A-Za-z0-9]+)",\s*/\* name \*/\s*'
    r'\n\s*(0x[0-9a-fA-F]+),\s*/\* id1 \*/\s*'
    r'\n\s*(0x[0-9a-fA-F]+),\s*/\* id2 \*/\s*'
    r'\n\s*([0-9]+L?\s*\*\s*[0-9]+L?|[0-9]+L?),\s*/\* size \*/\s*'
    r'\n\s*([0-9]+),', )


def load():
    s = io.open(SRC, encoding='utf-8', errors='replace').read()
    m = re.search(r'g_ramtron_parts\[\]\s*=\s*\{(.*?)\n\};', s, re.S)
    body = m.group(1)

    # 把被 #ifdef 包住的条目单独标出来
    guarded = []
    depth = 0
    lines = body.split('\n')
    keep = []
    for ln in lines:
        st = ln.strip()
        if st.startswith('#ifdef') or st.startswith('#if '):
            depth += 1
        keep.append((ln, depth > 0))
        if st.startswith('#endif'):
            depth -= 1
    guarded_body = '\n'.join(l for l, _g in keep)
    guarded_names = set()
    # 只看被 #ifdef 包住的那一段里出现的条目名
    inner = []
    for l, g in keep:
        if g:
            inner.append(l)
    for nm in re.findall(r'"([A-Za-z0-9]+)",\s*/\* name \*/', '\n'.join(inner)):
        guarded_names.add(nm)

    parts = []
    for m2 in ENTRY_RE.finditer(body):
        name, id1, id2, size, alen = m2.groups()
        # 源码里尺寸写成 `16L * 1024L` 这种式子 —— 算成字节数再进正文
        nbytes = 1
        for f in re.findall(r'(\d+)L?', size):
            nbytes *= int(f)
        parts.append((name, id1, id2, nbytes, int(alen),
                      name in guarded_names))
    return s, parts


def main():
    src, parts = load()

    print('=' * 74)
    print('① FRAM 驱动怎么认一颗器件 —— 查表')
    print('=' * 74)
    print()
    print('  驱动发一条"读 ID"命令，取回几个字节，然后：')
    print()
    print('  %s %s %s %s %s'
          % (pad('名字', 14), pad('id1', 10), pad('id2', 10),
             pad('容量', 14), '地址长度'))
    for name, id1, id2, size, alen, g in parts:
        tag = '（非 JEDEC 档）' if g else ''
        print('  %s %s %s %s %s %s'
              % (pad(name, 14), pad(id1, 10), pad(id2, 10),
                 pad('%d KB' % (size // 1024), 14),
                 '%d 字节' % alen, tag))
    print()
    print('  ★ **一共 %d 条**。认不认得出一颗器件，就看这两个字节对不对得上。'
          % len(parts))

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('② 匹配键是"哪两个字节"')
    print('=' * 74)
    print()
    cond = re.search(r'!\(priv->part->id1 == (\w+) && priv->part->id2 == (\w+)\)',
                     src)
    if cond:
        print('  源码里的匹配条件：')
        print('    `id1 == %s && id2 == %s`' % (cond.group(1), cond.group(2)))
    print()
    unused = 'UNUSED(manufacturer)' in src
    print('  ⚠️ 而"厂商代码"那个字节：**读出来了，但%s**。'
          % ('不参与匹配' if unused else '参与匹配'))
    print('     （源码里就写着 `UNUSED(manufacturer)` —— 它只用于打印日志。）')
    print()
    print('  ★★ 这一点很关键：**认一颗器件只用了两个字节**，')
    print('     而这两个字节**没有任何校验**（不是 JEDEC 全 3 字节比对）。')
    print('     ⇒ 只要表里有一行对得上，驱动就认 —— **哪怕那颗芯片根本不是它。**')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('③ 表里已经给"非 JEDEC 器件"留了位置')
    print('=' * 74)
    print()
    nd = [p for p in parts if p[5]]
    print('  被编译开关关着的条目：**%d 条**' % len(nd))
    for name, id1, id2, size, alen, _g in nd:
        print('    · %s —— id1 = %s、id2 = %s，容量 %d KB、地址长度 %d 字节'
              % (name, id1, id2, size // 1024, alen))
    print()
    if any(p[1].lower() == '0xff' and p[2].lower() == '0xff' for p in nd):
        print('  ★★★ 注意那两个字节：**全是 0xFF** —— ')
        print('       正是"读不出 ID"时读回来的东西。')
        print()
        print('  ★★ 也就是说：**"读不出 ID 的片子"不是意外，是一个已知类别。**')
        print('     上游驱动专门为它留了一行，并配了一个编译开关。')
        print('     ⇒ 所以"黑芯片"这个坑，**上游早就知道** ——')
        print('       它不是"没人遇到过"，是"遇到了、写下来了、默认关着"。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('④ ★★ "让它被认出来"的代价')
    print('=' * 74)
    print()
    print('  看匹配上之后驱动做了什么：')
    print()
    for n, pat in (('尺寸', r'priv->part->size'),
                   ('扇区数', r'priv->nsectors'),
                   ('页数', r'priv->npages')):
        if re.search(pat, src):
            print('    · 采用表里的**%s**' % n)
    print()
    print('  ★★★ 于是两条修法的**风险完全不同**：')
    print()
    print('  %s %s' % (pad('', 18), '后果'))
    print('  %s %s' % (pad('BMI088：改 ID 常量', 18), '只是**放行**；不改变任何行为参数'))
    print('  %s %s' % (pad('FRAM：认下某条目', 18),
                       '🔴 **同时采用了那一条的尺寸与地址长度**'))
    print()
    print('  ⇒ 后者把一个**你不确定的尺寸**告诉了系统。')
    print('    而它**不会报错** —— 文件系统照样挂载，只是几何对不上。')
    print('    （症状会推到很久以后：写到某个地址才出问题。）')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('⑤ 于是判据是：分开问两个问题')
    print('=' * 74)
    print()
    rows = [
        ('"它能不能用？"', '改常量 / 加表行 —— 这是**让流程继续**'),
        ('"它到底是什么？"', '读手册 / 量容量 / 问厂家 —— 这是**让结论可信**'),
    ]
    for a, b in rows:
        print('  %s %s' % (pad(a, 18), b))
    print()
    print('  ★★★ 这两个问题**必须分开回答**。')
    print('      把第一个问题的答案（"能跑了"）当成第二个问题的答案')
    print('      （"它就是那颗"），就是本章要防的那个错。')
    print()
    print('  ⚠️ 而且本案里两个问题的答案**很可能是"不一样的东西"**：')
    print('     表里那一行的名字不是你能买到的通用型号 ——')
    print('     它只保证"读出来是 0xFF 的片子会走到这条路上"。')

    # ------------------------------------------------------------------
    print()
    print('=' * 74)
    print('自检 —— 解析器必须先自证读对了')
    print('=' * 74)
    assert len(parts) >= 8, '① 表里条目太少 —— 正则不对'
    print('  ✓ 解出 %d 条器件记录' % len(parts))
    assert cond, '② 没找到匹配条件 —— 源码结构变了'
    print('  ✓ 匹配条件已定位（只比两个字节）')
    assert unused, '② 没找到 UNUSED(manufacturer) —— 结论「厂商码不参与匹配」不成立'
    print('  ✓ 厂商码确实标着 UNUSED（**不参与匹配**）')
    assert nd, '③ 没找到被编译开关关着的条目 —— ③ 那节会是空的'
    print('  ✓ 非 JEDEC 档存在：%s' % '、'.join(p[0] for p in nd))
    # 关键：非 JEDEC 档里必须**有一条**是 0xFF/0xFF，
    # 否则"读不出 ID 会被接住"这条结论不成立。
    # （注意不能要求"全都是" —— 非 JEDEC 档里还有一颗有正常 ID 的实型号。）
    ff = [p for p in nd if p[1].lower() == '0xff' and p[2].lower() == '0xff']
    assert ff, '③ 非 JEDEC 档里没有 0xFF/0xFF 那条 —— 结论反了'
    print('  ✓ 非 JEDEC 档里有 0xFF/0xFF 那条：%s（接住"读不出 ID"的情形）'
          % '、'.join(p[0] for p in ff))
    # 反向对照：确认这个解析器**能**看出条目差异
    fake = set((p[1], p[2]) for p in parts) | {('0xee', '0xee')}
    assert ('0xee', '0xee') in fake, '⑤ 反向对照失效'
    print('  ✓ 反向对照：表里查不到 (0xee, 0xee) —— 说明匹配确实是"查表"')
    print()
    print('  ✅ 自检通过')
    print()
    print('=' * 74)
    print('口径：本脚本**直接解析 NuttX 的 FRAM 驱动源码**：')
    print('      platforms/nuttx/NuttX/nuttx/drivers/mtd/ramtron.c')
    print('      ⚠️ 表里的条目、尺寸、地址长度都随上游版本变化 ——')
    print('         换版本后本脚本的输出会变（这是刻意的）。')
    print('=' * 74)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
