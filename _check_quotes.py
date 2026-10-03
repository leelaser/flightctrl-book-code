#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Python 源文件的引号 / 字符串陷阱检查器。

这个工具是《NuttX 教程》和《FreeRTOS+ChibiOS 教程》里反复踩坑之后写出来的。
它检查**三类**问题，按可靠性排序：

  【一】语法错误（权威）—— 直接调 compile()。
        这一类**零误报**，因为解释器说什么就是什么。
        但它只能报第一个错误。

  【二】未闭合的单行字符串 —— 状态机扫描。
        对应 Python 报的 "unterminated string literal"。
        这类问题在**写完文件之前**就能全部找出来（不用等编译）。

  【三】字符串过早闭合 —— 状态机扫描。
        形如  print("A"B"C") ：引号在语法上是配平的，
        但 Python 会报 "invalid syntax. Is this intended to be
        part of the string?"。
        compile() 只能报第一处；本检查能列出全部。

不检查（有意为之）：
  · 三引号内部的 ASCII 引号 —— 那只是文本，完全合法
  · C/C++ 原始字符串 R"delim(...)delim" 嵌在 Python 字符串里
  · 反斜杠续行、隐式字符串拼接的合法性

用法：
    python _check_quotes.py file.py [more.py ...]
退出码：0 = 全部干净，1 = 有问题
"""
import io
import sys

DQ = '"'
SQ = "'"
DQ3 = DQ * 3
SQ3 = SQ * 3
BACKSLASH = chr(92)

# 合法的"字符串之后"的字符：这些说明字符串确实结束了，
# 后面跟的是正常的 Python 语法
LEGAL_AFTER_STRING = set("),]:}" + DQ + SQ + BACKSLASH + "#")

# 合法的"字符串之后"的【关键字】。
# 关键：条件表达式 "A" if cond else "B" 里的 if / else 是关键字，
# 完全合法 —— 第一版检查器没考虑这一点，误报了 11 处。
LEGAL_AFTER_STRING_KEYWORDS = {
    "if", "else", "and", "or", "not", "in", "is", "for", "as",
    "elif", "while",
}


def _is_ident_char(c):
    """是否是标识符字符或 CJK 字符"""
    if c.isalnum() or c == '_':
        return True
    # CJK 统一表意文字、全角标点、中文标点
    o = ord(c)
    return (0x3000 <= o <= 0x303f or
            0x4e00 <= o <= 0x9fff or
            0xff00 <= o <= 0xffef)


def _word_at(line, k):
    """从位置 k 开始取出一个连续的标识符/CJK 词"""
    n = len(line)
    start = k
    while k < n and _is_ident_char(line[k]):
        k += 1
    return line[start:k]


def scan(path):
    """返回 (unterminated, early_close)，都是 [(行号, 行内容, 说明), ...]"""
    lines = io.open(path, encoding='utf-8').read().split('\n')
    state = 'normal'          # normal | in_dq | in_sq | in_dq3 | in_sq3
    unterminated = []
    early_close = []

    for i, line in enumerate(lines, 1):
        j, n = 0, len(line)
        while j < n:
            if state == 'normal':
                if line.startswith(DQ3, j):
                    state, j = 'in_dq3', j + 3
                    continue
                if line.startswith(SQ3, j):
                    state, j = 'in_sq3', j + 3
                    continue
                c = line[j]
                if c == '#':
                    break                      # 注释，跳到行尾
                if c == DQ:
                    state, j = 'in_dq', j + 1
                    continue
                if c == SQ:
                    state, j = 'in_sq', j + 1
                    continue
                j += 1

            elif state in ('in_dq', 'in_sq'):
                c = line[j]
                if c == BACKSLASH:
                    j += 2                     # 转义，跳过两个字符
                    continue
                if (state == 'in_dq' and c == DQ) or (state == 'in_sq' and c == SQ):
                    state = 'normal'
                    # ★ 检查三：字符串闭合之后，紧跟的是不是裸标识符/CJK
                    k = j + 1
                    while k < n and line[k] == ' ':
                        k += 1
                    if k < n:
                        nxt = line[k]
                        if (_is_ident_char(nxt)
                                and nxt not in LEGAL_AFTER_STRING):
                            word = _word_at(line, k)
                            # 关键字（if/else/and/...）是合法的续接
                            if word not in LEGAL_AFTER_STRING_KEYWORDS:
                                early_close.append(
                                    (i, line,
                                     "字符串在第 %d 列闭合后紧跟 %r —— "
                                     "若这不是有意的隐式拼接，就是引号用错了"
                                     % (j + 1, word)))
                j += 1

            else:                              # in_dq3 / in_sq3
                closer = DQ3 if state == 'in_dq3' else SQ3
                if line.startswith(closer, j):
                    state = 'normal'
                    j += 3
                    continue
                j += 1

        if state in ('in_dq', 'in_sq'):
            unterminated.append((i, line, "行尾时字符串仍未闭合"))
            state = 'normal'                   # 假定行尾是漏掉的闭合，继续扫

    return unterminated, early_close


def compile_check(path):
    """返回 None（通过）或 (行号, 消息)"""
    try:
        src = io.open(path, encoding='utf-8').read()
    except Exception as e:                     # noqa: BLE001
        return (0, "读取失败：%s" % e)
    try:
        compile(src, path, 'exec')
        return None
    except SyntaxError as e:
        return (e.lineno or 0, e.msg)
    except ValueError as e:                    # 比如源码里有 NUL 字节
        return (0, "compile 失败：%s" % e)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    total = 0
    for path in sys.argv[1:]:
        unterminated, early_close = scan(path)
        comp = compile_check(path)
        n_issues = len(unterminated) + len(early_close) + (1 if comp else 0)
        total += n_issues

        status = "OK" if n_issues == 0 else "%d 处问题" % n_issues
        print("=== %s: %s ===" % (path, status))

        if comp:
            line, msg = comp
            print("  【语法错误（权威）】第 %s 行：%s" % (line, msg))
        if unterminated:
            print("  【未闭合的单行字符串】")
            for n, l, why in unterminated:
                print("    %5d| %s" % (n, l.strip()[:96]))
        if early_close:
            print("  【字符串过早闭合】")
            for n, l, why in early_close:
                print("    %5d| %s" % (n, l.strip()[:96]))
                print("          %s" % why)
        print()

    print("合计 %d 处需要修正" % total)
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main())
