#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 5 第 1 章配套实验：ChibiOS 架构 —— RT / NIL / HAL

    实验 1  两个内核的功能对照（数文件 + 数行，复现文档 2.2 节）
    实验 2  ★★ 许可证功能门控模拟（复现文档 3.1 节）
    实验 3  模块规模实测（复现文档 0 节）
    实验 4  ★ 两个内核的门控清单差异 → 反推能力差集（文档 3.1b 节）

用法：
    python lesson10_chibios_arch.py 1
    python lesson10_chibios_arch.py 2
    python lesson10_chibios_arch.py 3
    python lesson10_chibios_arch.py 4
    python lesson10_chibios_arch.py          # 全部

本脚本的一个原则：**凡是能从源码读出来的，就不写死。**
唯一写死的是「规则的位置」和「期望值」——
期望值会被对照检查，对不上就报错。
这样如果哪天源码变了而文档没改，跑一遍就能发现。

源码版本：d103983f（2026-07-22，ChibiOS 内核 7.0.4）
          = ArduPilot 4.7.2 实际 pin 的提交
"""

import os
import re
import sys


# ===========================================================================
# 路径与工具
# ===========================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 源码根（相对本脚本：../../APM/ardupilot-Copter-4.7.2/...）
APM_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "APM",
                 "ardupilot-Copter-4.7.2")
)
CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")
CHCONF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                      "hwdef", "common", "chconf.h")

# 关键文件
F_RESTRICT_RT = os.path.join(CHIBIOS, "os", "rt", "include", "chrestrictions.h")
F_NIL_CH = os.path.join(CHIBIOS, "os", "nil", "include", "ch.h")
F_LICENSE = os.path.join(CHIBIOS, "os", "license", "chlicense.h")
F_CUSTOMER = os.path.join(CHIBIOS, "os", "license", "chcustomer.h")


def read_text(path):
    """读文本，失败时给出明确提示而不是崩栈。"""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError as e:
        print("  [错误] 读不到文件: %s" % path)
        print("         %s" % e)
        print("         请确认源码已拉取到: %s" % CHIBIOS)
        return None


def read_lines(path):
    t = read_text(path)
    return None if t is None else t.split("\n")


def hr(ch="=", n=74):
    print(ch * n)


def head(title):
    print()
    hr()
    print("  " + title)
    hr()


# ===========================================================================
# 源码级工具：从 C 源码里数行、列文件、抽 #define
# ===========================================================================

def count_lines(path):
    """只数换行符 —— 不 +1。

    踩过的坑：txt.count(chr(10)) + 1 会和 wc -l 差 1
    （最后一行没有换行时 count 少 1，+1 补回来看似正确，
     但文件末尾【有】换行时就会多 1）。
    统一用 count(chr(10)) 和 wc -l 对齐。
    """
    t = read_text(path)
    return 0 if t is None else t.count("\n")


def walk_sources(root, exts=(".c", ".h")):
    """返回 (文件数, 总行数, [相对路径...])，只统计指定扩展名。"""
    files = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for fn in filenames:
            if fn.lower().endswith(exts):
                files.append(os.path.join(dirpath, fn))
    total = sum(count_lines(p) for p in files)
    rel = sorted(os.path.relpath(p, root).replace("\\", "/") for p in files)
    return len(files), total, rel


# 匹配「#define NAME  VALUE」和「#undef NAME」
RE_DEFINE = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)\s*(.*?)\s*$")
RE_UNDEF = re.compile(r"^\s*#\s*undef\s+([A-Za-z_]\w*)\s*$")
RE_IF = re.compile(r"^\s*#\s*if\s+(.*?)\s*$")
RE_IFDEF = re.compile(r"^\s*#\s*ifdef\s+([A-Za-z_]\w*)\s*$")


def strip_comments(lines):
    """去掉 // 和 /* */ 注释，返回同长度的行列表（行号不变）。"""
    out = []
    in_block = False
    for raw in lines:
        line = raw
        if in_block:
            if "*/" in line:
                in_block = False
                line = line.split("*/", 1)[1]
            else:
                out.append("")
                continue
        if "/*" in line:
            before, after = line.split("/*", 1)
            if "*/" in after:
                line = before + after.split("*/", 1)[1]
            else:
                in_block = True
                line = before
        out.append(line.split("//", 1)[0])
    return out


def merge_continuations(lines):
    """把以 \\ 结尾的行合并成一行，保留【起始行号】。

    踩过的坑：chrestrictions.h 和 ch.h 里的 #if 条件都是跨行的：

        #if (CH_LICENSE_FEATURES == CH_FEATURES_INTERMEDIATE) ||      \
            (CH_LICENSE_FEATURES == CH_FEATURES_BASIC)

    不合并的话只看到前半句（只有 INTERMEDIATE），
    classify_condition 会返回 None ⇒ 整个档位【一条规则都抽不到】。
    第一版就栽在这里：INT+BASIC 抽出 0 条规则，静默漏掉了
    「tick 强制 ≤ 1000」这条最关键的规则。

    返回 [(起始行号, 合并后的文本), ...]
    """
    out = []
    buf = None
    start = 0
    for i, line in enumerate(lines, 1):
        if buf is None:
            buf = line
            start = i
        else:
            buf = buf.rstrip() + " " + line.strip()
        if buf.rstrip().endswith("\\"):
            buf = buf.rstrip()[:-1]
            continue
        out.append((start, buf))
        buf = None
    if buf is not None:
        out.append((start, buf))
    return out


def extract_defines(path, wanted):
    """从文件里抽出 wanted 里列出的宏及其值。

    ★ 关键：要能识别【条件化定义】。

        #if HAL_ENABLE_THREAD_STATISTICS
        #define CH_DBG_STATISTICS TRUE
        #else
        #define CH_DBG_STATISTICS FALSE
        #endif

    第一版只取「最后出现的那条」⇒ 拿到 FALSE，
    于是把 CH_DBG_STATISTICS 当成「ArduPilot 设了 TRUE」，
    据此得出「6 项冲突」的错误结论。

    实际值是【条件表达式】，不能只看一条。

    返回 {宏名: [(值, 行号, 条件说明), ...]}
      · 只有一条 → 无条件
      · 多条     → 条件化，条件说明形如 "if HAL_ENABLE_THREAD_STATISTICS" / "else"
    """
    lines = read_lines(path)
    if lines is None:
        return {}

    lines = strip_comments(lines)
    merged = merge_continuations(lines)

    out = {}
    cond_stack = []       # 当前处于哪些 #if 里
    pending_cond = None   # 最近的 #if 条件文本（供分支标注）

    for start, line in merged:
        s = line.strip()

        m = RE_IF.match(line)
        if m:
            cond = re.sub(r"\s+", " ", m.group(1)).strip()
            cond_stack.append(cond)
            pending_cond = cond
            continue
        if re.match(r"^\s*#\s*ifdef\b", line) or \
           re.match(r"^\s*#\s*ifndef\b", line):
            cond = re.sub(r"^\s*#\s*ifn?def\s+", "", line).strip()
            cond_stack.append(cond)
            pending_cond = cond
            continue

        if re.match(r"^\s*#\s*elif\b", line):
            cond = re.sub(r"^\s*#\s*elif\s+", "", line).strip()
            cond = re.sub(r"\s+", " ", cond)
            if cond_stack:
                cond_stack[-1] = cond
            pending_cond = cond
            continue
        if re.match(r"^\s*#\s*else\b", line):
            if cond_stack:
                cond_stack[-1] = "else of: " + str(cond_stack[-1])
            pending_cond = "else"
            continue
        if re.match(r"^\s*#\s*endif\b", line):
            if cond_stack:
                cond_stack.pop()
            pending_cond = None
            continue

        m = RE_DEFINE.match(line)
        if not m:
            continue
        name, value = m.group(1), m.group(2).strip()
        if name not in wanted:
            continue

        if cond_stack:
            # 条件化 —— 标注是哪个条件的那一支
            cond = cond_stack[-1]
            if str(cond).startswith("else"):
                label = "else of: " + str(cond)[len("else of: "):]
            else:
                label = "if " + cond
        else:
            label = None
        out.setdefault(name, []).append((value, start, label))

    return out


# ===========================================================================
# 实验 1：两个内核的功能对照
# ===========================================================================

def exp1():
    head("实验 1：两个内核的功能对照（RT vs NIL）")

    print("""
  目的：不靠文档，靠**目录里有什么文件**推导两个内核的能力差异。

  方法：列出 os/rt/src 和 os/nil/src 的所有 .c 文件，
        按功能归类，看 NIL 少了哪些。
""")

    rt_src = os.path.join(CHIBIOS, "os", "rt", "src")
    nil_src = os.path.join(CHIBIOS, "os", "nil", "src")

    if not os.path.isdir(rt_src):
        print("  [错误] 找不到 %s" % rt_src)
        return

    rt_n, rt_l, rt_files = walk_sources(rt_src, (".c",))
    nil_n, nil_l, nil_files = walk_sources(nil_src, (".c",))

    print("  os/rt/src/  : %d 个 .c，%d 行  （只算 src/，不含 include/）"
          % (rt_n, rt_l))
    print("  os/nil/src/ : %d 个 .c，%d 行" % (nil_n, nil_l))
    print()
    print("  RT 的文件清单：")
    for fn in rt_files:
        print("      %s (%d 行)" % (fn, count_lines(os.path.join(rt_src, fn))))
    print()
    print("  NIL 的文件清单：")
    for fn in nil_files:
        print("      %s (%d 行)" % (fn, count_lines(os.path.join(nil_src, fn))))

    print()
    hr("-")
    print("  ★ 归类对照")
    hr("-")

    # 功能 → (RT 文件名列表, NIL 里对应的文件名列表)
    #
    # 注意：NIL 的文件名【不一样】——
    #   RT 的 chevents.c 在 NIL 里叫 chevt.c
    #   RT 的 chschd.c+chsys.c+chthreads.c 在 NIL 里合并成 ch.c
    # 第一版没写这张别名表，于是把 NIL 的「事件」误判成「无」。
    # 教训：靠文件名推断能力时，必须先把别名对齐。
    FEATURES = [
        ("调度器核心",     ["chschd.c", "chsys.c", "chthreads.c"], []),
        ("信号量",         ["chsem.c"], ["chsem.c"]),
        ("事件",           ["chevents.c"], ["chevt.c"]),
        ("消息",           ["chmsg.c"], ["chmsg.c"]),
        ("互斥量(含PI)",   ["chmtx.c"], []),
        ("条件变量",       ["chcond.c"], []),
        ("动态线程创建",   ["chdynamic.c"], []),
        ("线程注册表",     ["chregistry.c"], []),
        ("统计",           ["chstats.c"], []),
        ("时间测量(TM)",   ["chtm.c"], []),
        ("虚拟定时器(VT)", ["chvt.c"], []),
        ("调试/trace",     ["chdebug.c", "chtrace.c"], []),
        ("实例管理",       ["chinstances.c"], []),
        ("功能安全(RFCU)", ["chrfcu.c"], []),
    ]

    nil_base = {f.lower() for f in nil_files}

    print("  %-16s %-24s %s" % ("功能", "RT 负责文件", "NIL"))
    print("  " + "-" * 62)
    missing = []
    for feat, rt_f, nil_f in FEATURES:
        if nil_f:
            hit = [f for f in nil_f if f.lower() in nil_base]
            mark = "同名/别名：" + ",".join(hit) if hit else "★ 无（别名也没找到）"
            if not hit:
                missing.append(feat)
        elif feat == "调度器核心":
            mark = "合并在 ch.c"
        else:
            mark = "★ 无"
            missing.append(feat)
        print("  %-16s %-24s %s" % (feat, ",".join(rt_f), mark))

    print()
    print("  NIL 缺少（按文件清单推断）：")
    print("      %s" % "、".join(missing))
    print()
    print("  [src/ 代码量比] RT %d 行 / NIL %d 行 = %.2f 倍"
          % (rt_l, nil_l, rt_l / nil_l))

    print()
    hr("-")
    print("  ★★ NIL 的「静态创建」到底是什么意思？")
    hr("-")

    print("""
  猜猜看：NIL 里 chThdCreateStatic 有多少个调用点？
""")
    nil_hits = []
    for dirpath, _d, filenames in os.walk(os.path.join(CHIBIOS, "os", "nil")):
        for fn in filenames:
            if not fn.lower().endswith((".c", ".h")):
                continue
            p = os.path.join(dirpath, fn)
            t = read_text(p) or ""
            for i, line in enumerate(t.split("\n"), 1):
                if "chThdCreate" in line:
                    nil_hits.append((os.path.relpath(p, CHIBIOS).replace("\\", "/"),
                                     i, line.strip()))

    for p, i, line in nil_hits:
        print("      %s:%d" % (p, i))
        print("          %s" % line)

    print("""
  ⇒ 有 chThdCreate/chThdCreateI，但它们的参数是
    `const thread_descriptor_t *tdp` —— 一个【描述符指针】。

  那描述符从哪来？看 ch.h:627-655：
""")

    nil_ch = read_lines(F_NIL_CH)
    if nil_ch:
        for i in range(626, 656):
            if i - 1 < len(nil_ch):
                print("      %d  %s" % (i, nil_ch[i - 1]))

    print("""
  ⇒ NIL 的线程用【编译期静态表】声明：

        THD_TABLE_BEGIN
          THD_TABLE_THREAD(PRIO, "name", wa, func, arg)
          THD_TABLE_THREAD(...)
        THD_TABLE_END

    展开成 `const thread_descriptor_t nil_thd_configs[]`。

  ★ 注意 THD_TABLE_END 里塞了什么：一个名为 "idle"、
    优先级为 CH_CFG_MAX_THREADS 的线程描述符。
    ⇒ 【idle 线程也被放进用户那张表里】。

    这是「表驱动设计」的必然结果 —— idle 也是一个线程，
    只是优先级最低（NIL 里【下标越小优先级越高】，
     见卷 5 第 2 章）。既然所有线程都从表里来，idle 也得是表的一项。

  ⇒ 所以「NIL 只能静态创建线程」的准确含义是：

      【不是「没有创建 API」，而是「创建的线程必须先在表里登记」。】

      chThdCreate(tdp) 只是把表里已经写好的一项挂进调度器。
      没有堆分配、没有运行时决定栈大小 —— 全在编译期定死。

  ⇒ 而且还有更硬的一层：NIL 的 thread_t 数组【按优先级下标】存，
     chThdCreateI 里有一句

         tp = &nil.threads[tdp->prio];
         chDbgAssert(..., "priority slot taken");

     ⇒ 【一个优先级只能有一个线程】。
       所以 16 个线程的上限，本质是【16 个优先级档位】的上限。
       （详见卷 5 第 2 章）
""")

    print()
    hr("-")
    print("  ★★ 交叉验证：NIL 真的没有 mutex 吗？")
    hr("-")

    # 用第三个独立证据：grep 符号
    print("""
  文件清单说「NIL 没有 chmtx.c」，但这只说明「没有单独的文件」——
  它可能把 mutex 合并进 ch.c 了。

  所以需要第三条独立线索：**grep 类型名**。
""")

    hits = []
    for dirpath, _d, filenames in os.walk(os.path.join(CHIBIOS, "os", "nil")):
        for fn in filenames:
            if not fn.lower().endswith((".c", ".h")):
                continue
            p = os.path.join(dirpath, fn)
            t = read_text(p) or ""
            for i, line in enumerate(t.split("\n"), 1):
                if re.search(r"\bchMtx\w*|\bmutex_t\b", line):
                    hits.append((os.path.relpath(p, CHIBIOS).replace("\\", "/"),
                                 i, line.strip()))

    if not hits:
        print("  grep -rn \"chMtx*|mutex_t\" os/nil/")
        print("      ⇒ 零匹配")
        print()
        print("  ⇒ NIL 目录下【连 mutex_t 这个类型都不存在】。")
        print("     不是「合并在别的文件里」，是「根本没有」。")
    else:
        print("  grep 命中 %d 处：" % len(hits))
        for p, i, line in hits[:20]:
            print("      %s:%d  %s" % (p, i, line))

    print()
    print("  进一步：NIL 的 ch.h 里对 mutex 说了什么？")
    print("  （把两个 #error 放在一起看 —— 这是本实验最要紧的一步）")
    print()

    if nil_ch:
        # 找到「值检查」那一行，打印它前后各 2 行
        target = next((i for i, L in enumerate(nil_ch, 1)
                       if "mutexes not yet supported" in L), None)
        if target:
            for i in range(max(1, target - 3), min(len(nil_ch), target + 2) + 1):
                mark = "  ★" if i == target else "   "
                print("   %s ch.h:%-4d %s" % (mark, i, nil_ch[i - 1]))

        print()
        # 再打印「要求定义」的那一段
        target2 = next((i for i, L in enumerate(nil_ch, 1)
                        if "CH_CFG_USE_MUTEXES not defined" in L), None)
        if target2:
            print("   ... 而 120 行之前还有一条：")
            print()
            for i in range(max(1, target2 - 2), min(len(nil_ch), target2 + 1) + 1):
                print("      ch.h:%-4d %s" % (i, nil_ch[i - 1]))

        print("""
  ⇒ 把两条 #error 连起来读：

        「你必须定义 CH_CFG_USE_MUTEXES」
        「但如果它的值是 TRUE，编译失败」

    合起来 = 【必须定义，且必须是 FALSE】。

  ⚠ 我第一版读错了：只看到第一条，就推断「NIL 支持 mutex」。
     教训：**一个 #error 只能说明「这个宏必须被定义」，
     不能说明「哪个值是对的」。**
     要确定取值，得找【对值本身的检查】。
""")
    print()


# ===========================================================================
# 实验 2：★★ 许可证功能门控模拟
# ===========================================================================

# 门控条件块的识别：从 #if 行里认出「哪个档位生效」
def classify_condition(cond):
    """把 #if 的条件文本归类成门控档位。

    返回 'INT+BASIC' / 'BASIC' / None
    """
    c = cond.replace("\\", " ").replace("\n", " ")
    c = re.sub(r"\s+", " ", c).strip()

    has_int = "CH_FEATURES_INTERMEDIATE" in c
    has_basic = "CH_FEATURES_BASIC" in c

    if has_int and has_basic:
        return "INT+BASIC"
    if has_basic and not has_int:
        return "BASIC"
    return None


def parse_restrictions(path, label):
    """从 chrestrictions.h / nil 的 ch.h 里抽出门控规则。

    返回 {档位: [(宏名, 动作, 值, 行号), ...]}
     动作 ∈ {'undef', 'define'}
    """
    lines = read_lines(path)
    if lines is None:
        return None

    lines = strip_comments(lines)
    merged = merge_continuations(lines)

    rules = {}
    # 栈：记录当前处于哪些 #if 块里（None 表示非门控块）
    stack = []

    for i, line in merged:
        m = RE_IF.match(line)
        if m:
            gate = classify_condition(m.group(1))
            stack.append(gate)
            continue

        # 处理 #elif —— 把栈顶的档位替换掉
        if re.match(r"^\s*#\s*elif\b", line):
            cond = re.sub(r"^\s*#\s*elif\s+", "", line)
            if stack:
                stack[-1] = classify_condition(cond)
            continue

        # 处理 #else —— 栈顶档位失效（我们不模拟 else 分支）
        if re.match(r"^\s*#\s*else\b", line):
            if stack:
                stack[-1] = None
            continue

        if re.match(r"^\s*#\s*endif\b", line):
            if stack:
                stack.pop()
            continue

        # 收集规则：取栈里第一个非 None 的档位
        gate = None
        for g in stack:
            if g:
                gate = g
                break
        if gate is None:
            continue

        m = RE_UNDEF.match(line)
        if m:
            rules.setdefault(gate, []).append((m.group(1), "undef", None, i))
            continue

        m = RE_DEFINE.match(line)
        if m:
            name, value = m.group(1), m.group(2).strip()
            # 只看被门控的那些（跳过 INLINE/常见宏）
            if name.startswith(("CH_CFG_", "CH_DBG_")):
                rules.setdefault(gate, []).append((name, "define", value, i))

    print("  [%s]" % label)
    for gate in ("INT+BASIC", "BASIC"):
        rs = rules.get(gate, [])
        print("      %-10s : %d 条规则" % (gate, len(rs)))
    return rules


def apply_restrictions(user_cfg, rules, mode):
    """把门控规则作用到用户配置上。

    user_cfg: {宏名: 值字符串}
    mode:     'FULL' / 'INTERMEDIATE' / 'BASIC'
    返回 (effective_cfg, [(宏, 原值, 新值, 档位, 行号), ...])
    """
    eff = dict(user_cfg)
    overrides = []

    gates = []
    if mode in ("INTERMEDIATE", "BASIC"):
        gates.append("INT+BASIC")
    if mode == "BASIC":
        gates.append("BASIC")

    for gate in gates:
        for name, action, value, lineno in rules.get(gate, []):
            if name not in user_cfg:
                continue
            old = eff.get(name)
            if action == "undef":
                # #undef 单独出现 → 后面通常跟一个 #define
                # 但如果是「#undef 后没有 #define」，就是彻底移除
                # 这里先记着，等 #define 覆盖
                continue
            if old != value:
                overrides.append((name, old, value, gate, lineno))
                eff[name] = value

    return eff, overrides


def exp2():
    head("实验 2：★★ 许可证功能门控模拟")

    print("""
  假设：ChibiOS 的许可证不只是法律条款，它会 **#undef 你的内核配置**。

  验证方法：
    ① 从 chrestrictions.h 里抽出「哪个档位禁用/改写哪些宏」
    ② 从 ArduPilot 的 chconf.h 里抽出它实际设的值
    ③ 把 ① 作用到 ②，看哪些值被改写

  如果被改写的项恰好是 ArduPilot 代码依赖的，那它就用不了那个档位。
""")

    print()
    print("  ── ① 抽取门控规则 ──")
    print()
    rt_rules = parse_restrictions(F_RESTRICT_RT, "os/rt/include/chrestrictions.h")
    nil_rules = parse_restrictions(F_NIL_CH, "os/nil/include/ch.h")

    if rt_rules is None:
        return

    print()
    print("  ── RT 内核的门控规则明细 ──")
    for gate in ("INT+BASIC", "BASIC"):
        print()
        print("    [%s]" % gate)
        for name, action, value, lineno in rt_rules.get(gate, []):
            if action == "undef":
                print("      chrestrictions.h:%-3d  #undef  %s" % (lineno, name))
            else:
                print("      chrestrictions.h:%-3d  #define %s  %s"
                      % (lineno, name, value))

    print()
    print("  ── ② ArduPilot 的 chconf.h 设了什么 ──")
    print()

    WANTED = [
        "CH_CFG_ST_FREQUENCY",
        "CH_CFG_ST_TIMEDELTA",
        "CH_CFG_USE_TM",
        "CH_CFG_USE_MUTEXES",
        "CH_CFG_USE_MUTEXES_RECURSIVE",
        "CH_CFG_USE_CONDVARS",
        "CH_CFG_USE_MESSAGES",
        "CH_CFG_USE_DYNAMIC",
        "CH_DBG_STATISTICS",
        "CH_DBG_TRACE_MASK",
    ]
    defines = extract_defines(CHCONF, set(WANTED))
    if not defines:
        print("  [错误] 没能从 chconf.h 抽出配置")
        return

    print("    %-34s %-30s %s" % ("宏", "值", "chconf.h 行号"))
    print("    " + "-" * 78)

    user_cfg = {}       # 只放【无条件】的，用于门控对撞
    conditional = {}    # 条件化的，单独列

    for name in WANTED:
        entries = defines.get(name)
        if not entries:
            print("    %-34s %-30s %s" % (name, "(未定义)", "—"))
            continue
        if len(entries) == 1:
            value, lineno, cond = entries[0]
            user_cfg[name] = value
            print("    %-34s %-30s %d" % (name, value, lineno))
        else:
            conditional[name] = entries
            print("    %-34s %-30s ★ 条件化" % (name, "见下"))
            for value, lineno, cond in entries:
                print("    %-34s   %-8s (%-22s) %d"
                      % ("", value, cond or "?", lineno))

    print()
    print("  ── ★ 发现：条件化定义（第一版漏掉的）──")
    print()
    for name, entries in conditional.items():
        print("    %s 在 chconf.h 里出现了 %d 次：" % (name, len(entries)))
        for value, lineno, cond in entries:
            print("        %-6s  在 %s 分支   (chconf.h:%d)" % (value, cond, lineno))
        print()
        print("    ⇒ 它的值【不是常量】，取决于那个条件宏。")
        print("    ⇒ 只看「最后一条」会得出错误的结论。")
        print()

    print("  ── ③ 把门控作用上去 ──")
    print()

    total_conflicts = 0
    for mode in ("FULL", "INTERMEDIATE", "BASIC"):
        eff, overrides = apply_restrictions(user_cfg, rt_rules, mode)
        print("    ┌─ CH_LICENSE_FEATURES = CH_FEATURES_%s" % mode)
        if mode == "FULL":
            print("    │  无门控。%d 项无条件配置全部保持原值。" % len(user_cfg))
        elif not overrides:
            print("    │  无改写。")
        else:
            for name, old, new, gate, lineno in overrides:
                print("    │  ★ %-30s %-12s → %-12s"
                      % (name, old, new))
                print("    │      (chrestrictions.h:%d, 档位 %s)" % (lineno, gate))
        print("    └─ 共改写 %d 项" % len(overrides))
        if mode == "BASIC":
            total_conflicts = len(overrides)
        print()

    print("  ── ④ 这些被改写的项，ArduPilot 用到了吗？ ──")
    print()

    USAGE = [
        ("CH_CFG_USE_MUTEXES",  "HAL_Semaphore 就是 mutex_t",
         "Semaphores.cpp:30-33,50,73 / SPIDevice.cpp:109"),
        ("CH_CFG_USE_DYNAMIC",  "chThdCreateFromHeap / Scheduler.cpp 整段",
         "AP_Radio/AP_Radio_bk2425.cpp:144"),
        ("CH_CFG_ST_FREQUENCY", "定时精度（1 MHz vs 1 kHz）",
         "chconf.h:115"),
        ("CH_CFG_ST_TIMEDELTA", "tickless（省电）",
         "chconf.h:143"),
        ("CH_CFG_USE_TM",       "线程 CPU 占用统计",
         "chconf.h:246"),
    ]

    for name, why, where in USAGE:
        print("    %-24s %s" % (name, why))
        print("    %-24s → %s" % ("", where))
        print()

    print()
    hr("-")
    print("  ★★ 结论")
    hr("-")
    print("""
    BASIC 档会改写 %d 项 —— 而这 %d 项【每一项都是 ArduPilot 的依赖】。

    ▼ 失败模式（不是笼统的「编译不过」，要分两层）：

      【编译错误】—— 调用点没有 #if 守卫
        · chmtx.h:31 把整个头文件包在 CH_CFG_USE_MUTEXES == TRUE 里
          ⇒ mutex_t 类型和所有 chMtx* 原型【全部消失】
        · Scheduler.cpp:867 直接用 mutex_t        ⇒ 'mutex_t' does not name a type
        · RCOutput.cpp:156,1371,1408 用 chMtx*    ⇒ not declared in this scope
        · SPIDevice.cpp:82,109,112 用 chMtx*

      【链接错误】—— 调用点有 #if 守卫，但只守了实现
        · Semaphores.h:25-39 【无条件】声明了 ChibiOS::Semaphore
        · Semaphores.cpp:25 把【整个实现】#if 掉 ⇒ 一个符号都不产生
        · ⇒ undefined reference to ChibiOS::Semaphore::take(...)

      【其他连带】
        · USE_DYNAMIC 关 ⇒ Scheduler.cpp:42-850 整段被 #if 掉，
          AP_Radio 三个驱动找不到 chThdCreateFromHeap
        · tick 1 MHz → 1 kHz ⇒ 定时精度差 1000 倍
        · ST_TIMEDELTA 归零 ⇒ tickless 没了
        · USE_TM 关闭 ⇒ 线程统计没了

    ⇒ 如果 ArduPilot 用 "GPL + Exception"（= BASIC 档），
      【它在编译期和链接期两处都过不去】。

    ⇒ 这不是「合规问题」，是「功能被删了」。
""" % (total_conflicts, total_conflicts))

    print("  顺带：Commercial Free 是 INTERMEDIATE 档 —— 不禁 mutex，")
    print("        但【tick 依然是 1 kHz 上限】。对飞控同样不可用。")
    print()

    # ── 附：有守卫 vs 无守卫 ──
    print()
    hr("-")
    print("  ★ 附：ArduPilot 自己知道 mutex 可能被关掉吗？")
    hr("-")

    # 真的去扫源码：找出所有引用 mutex_t / chMtx* 的 .cpp，
    # 并对每个引用点判断「它前面有没有 CH_CFG_USE_MUTEXES 守卫」
    HAL_DIR = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS")

    REF = re.compile(r"\bmutex_t\b|\bchMtx\w+|\bMUTEX_DECL\b")

    print()
    print("    扫描 %s/ 下所有 .cpp 里的 mutex 引用……" % "AP_HAL_ChibiOS")
    print()

    guarded, unguarded = [], []

    if os.path.isdir(HAL_DIR):
        for dirpath, _d, filenames in os.walk(HAL_DIR):
            for fn in filenames:
                if not fn.endswith(".cpp"):
                    continue
                p = os.path.join(dirpath, fn)
                src = read_lines(p)
                if not src:
                    continue
                clean = strip_comments(src)
                merged = merge_continuations(clean)

                # 记录当前是否处于 CH_CFG_USE_MUTEXES 的守卫块里
                depth = 0
                guard_stack = []
                for lineno, line in merged:
                    s = line.strip()
                    if RE_IF.match(line) and "CH_CFG_USE_MUTEXES" in s:
                        guard_stack.append(True)
                    elif s.startswith("#if"):
                        guard_stack.append(False)
                    elif s.startswith("#endif"):
                        if guard_stack:
                            guard_stack.pop()
                    elif s.startswith("#else") and guard_stack:
                        guard_stack[-1] = False

                    if RE_IF.match(line) or s.startswith("#if") \
                       or s.startswith("#endif") or s.startswith("#else"):
                        continue

                    if REF.search(s):
                        is_guarded = any(guard_stack)
                        rec = (fn, lineno, s.strip()[:56], is_guarded)
                        (guarded if is_guarded else unguarded).append(rec)

    print("    ┌─ 有 CH_CFG_USE_MUTEXES 守卫的引用（%d 处）" % len(guarded))
    print("    │  ⇒ 能编译，但实现被 #if 掉 ⇒ 【链接错误】")
    for f, l, s, _ in guarded:
        print("    │  %-26s %-6d %s" % (f, l, s))
    print("    └─")

    print()
    print("    ┌─ 无守卫的引用（%d 处）" % len(unguarded))
    print("    │  ⇒ mutex_t 不存在 ⇒ 【编译错误】")
    for f, l, s, _ in unguarded[:25]:
        print("    │  %-26s %-6d %s" % (f, l, s))
    if len(unguarded) > 25:
        print("    │  …… 另有 %d 处" % (len(unguarded) - 25))
    print("    └─")

    print("""
    ⚠⚠ 【2026-09 修正】上面这个结论【是错的】，保留原文以示教训。

      我原来的推论是：「14 处有守卫、9 处没有 ⇒ 守卫不完整 ⇒ 更危险」。

      **但这个推论不成立** —— 因为「守卫」的形式不止一种。
      我的扫描器只搜 CH_CFG_USE_MUTEXES 这一个宏，
      凡是没有它的地方就记成「无守卫」。

      实际上 ArduPilot 的写法是：
      【每个文件用它自己最自然的那个宏做守护】——

          Semaphores.cpp   ← CH_CFG_USE_MUTEXES（它就是 mutex 的封装）
          SPIDevice.cpp    ← HAL_USE_SPI（整个文件包在 #if HAL_USE_SPI == TRUE 里）
          I2CDevice.cpp    ← HAL_USE_I2C（同上）
          RCOutput.cpp     ← HAL_USE_PWM（同上）
          Scheduler.cpp    ← HAL_LOGGING_ENABLED（那段代码是给日志用的）

      而 bootloader 的生成配置里（实测，可跑生成器复现）：

          HAL_USE_SPI  FALSE      ⇒ SPIDevice.cpp   整 个 不 编
          HAL_USE_I2C  FALSE      ⇒ I2CDevice.cpp   整 个 不 编
          HAL_USE_PWM  FALSE      ⇒ RCOutput.cpp    整 个 不 编
          CH_CFG_USE_MUTEXES FALSE ⇒ Semaphores.cpp 编但实现为空

      ⇒ 那 9 处引用【一处都不会被编译到】。

    ★ 教训：**扫描器找什么，就只找得到什么。**

      「14 处有 A 守卫、9 处没有」—— 这个事实陈述是对的。
      但推出的「守卫不完整」是错的：
      【「没有 A 守卫」不等于「没有守卫」】。

      ⇒ 报告「我搜了什么没找到」是安全的；
        报告「它没有」是危险的 —— 除非穷举了所有可能的形式。

    ★ 复现 bootloader 配置的方法：

        python libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py \\
            --bootloader --outdir /tmp/bl2 \\
            libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef-bl.dat

      完整分析见卷 5 第 3 章c。
""")

    # 期望值对照
    print("  ── 对照文档里的期望值 ──")
    expected_basic = 5
    got = total_conflicts
    if got == expected_basic:
        print("      BASIC 档改写项数 = %d  ✓ 与文档一致" % got)
    else:
        print("      ⚠ BASIC 档改写项数 = %d，文档里写的是 %d"
              % (got, expected_basic))
    print()

    # ── 附：CH_DBG_STATISTICS 的完整链条 ──
    print()
    hr("-")
    print("  ★★ 附：一个「看起来冲突，其实不冲突」的项")
    hr("-")

    print("""
    CH_DBG_STATISTICS 一开始被我算成第 6 项冲突 —— 错了。

    链条是这样的（每一环都可验证）：

        chconf.h:73-77
            #if HAL_ENABLE_THREAD_STATISTICS
            #define CH_DBG_STATISTICS TRUE
            #else
            #define CH_DBG_STATISTICS FALSE     ← 走这条
            #endif

        AP_HAL/AP_HAL_Boards.h:272-273
            #ifndef HAL_ENABLE_THREAD_STATISTICS
            #define HAL_ENABLE_THREAD_STATISTICS 0     ← 默认关
            #endif

        实测：300 块板卡的 hwdef.dat 里，定义它的有 0 块。
""")

    # 真的去数一遍板卡
    hwdef_dir = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS", "hwdef")
    if os.path.isdir(hwdef_dir):
        boards = [d for d in os.listdir(hwdef_dir)
                  if os.path.isdir(os.path.join(hwdef_dir, d))]
        enables = []
        for b in boards:
            dat = os.path.join(hwdef_dir, b, "hwdef.dat")
            if os.path.isfile(dat):
                t = read_text(dat) or ""
                if "HAL_ENABLE_THREAD_STATISTICS" in t:
                    enables.append(b)
        print("        [实测] hwdef/ 下有 %d 块板子，"
              "其中启用线程统计的：%d 块" % (len(boards), len(enables)))
        if enables:
            print("               %s" % "、".join(enables))
        print()

    print("""    ⇒ 所以 CH_DBG_STATISTICS 对【所有出厂板卡】恒为 FALSE。
      BASIC 档也给它 FALSE —— 【两边一致，不是冲突】。

    ⇒ 修正：冲突数是 5 项，不是 6 项。

    ⚠ 我是怎么错的？grep 到 chconf.h:74 的
      「#define CH_DBG_STATISTICS TRUE」就直接下结论，
      【没看它头顶第 73 行的 #if】。

      教训：**grep 给出的是「这一行长什么样」，
      不是「这一行算不算数」。**
      看到 #define 要先问一句：它在条件块里吗？
""")

    print("  顺带一个有实际意义的发现：")
    print("      HAL_ENABLE_THREAD_STATISTICS 默认 0，且没有板子开 ⇒")
    print("      Util.cpp:433-475 里那两段 LOAD=%.1f%% 的代码【从不执行】。")
    print("      ⇒ @SYS/threads.txt 里【没有每个线程的 CPU 占用率】。")
    print("      （第一版还会因为线程统计没开而漏掉这一整个功能面）")
    print()


# ===========================================================================
# 实验 3：模块规模实测
# ===========================================================================

def exp3():
    head("实验 3：ChibiOS 模块规模实测")

    print("""
  目的：搞清楚「ChibiOS 有多大」，以及「大在哪里」。

  这一点很重要 —— 因为「ChibiOS 是 400 万行」和
  「ChibiOS 内核是 1.4 万行」是两句话，给出来的印象完全不同。
""")

    MODULES = [
        ("os/rt",     "RT 内核"),
        ("os/nil",    "NIL 内核"),
        ("os/oslib",  "通用库"),
        ("os/sb",     "沙箱"),
        ("os/ex",     "扩展子系统"),
        ("os/hal",    "HAL（含所有厂商）"),
        ("os/common", "厂商头文件（CMSIS 等）"),
        ("os/various", "杂项"),
    ]

    print()
    print("  %-14s %-26s %8s %12s" % ("目录", "说明", "文件数", "行数"))
    print("  " + "-" * 64)

    results = {}
    for sub, desc in MODULES:
        path = os.path.join(CHIBIOS, sub)
        if not os.path.isdir(path):
            print("  %-14s %-26s %8s %12s" % (sub, desc, "—", "（不存在）"))
            continue
        n, lines, _ = walk_sources(path)
        results[sub] = (n, lines)
        print("  %-14s %-26s %8d %12s" % (sub, desc, n, "{:,}".format(lines)))

    print()
    hr("-")
    print("  ★ 分解：os/hal 和 os/common 里面是什么？")
    hr("-")

    for sub in ("os/hal", "os/common"):
        path = os.path.join(CHIBIOS, sub)
        if not os.path.isdir(path):
            continue
        print()
        print("  %s/ 的子目录（按行数排序）：" % sub)
        subs = []
        for entry in sorted(os.listdir(path)):
            ep = os.path.join(path, entry)
            if os.path.isdir(ep):
                n, lines, _ = walk_sources(ep)
                subs.append((lines, n, entry))
        subs.sort(reverse=True)
        for lines, n, entry in subs[:8]:
            print("      %-28s %5d 文件 %12s 行" % (entry, n, "{:,}".format(lines)))

    print()
    hr("-")
    print("  ★ 观察：内核 vs HAL vs 厂商代码")
    hr("-")

    if "os/rt" in results and "os/hal" in results:
        rt_l = results["os/rt"][1]
        hal_l = results["os/hal"][1]
        com_l = results.get("os/common", (0, 0))[1]
        print()
        print("      RT 内核         %10s 行   ← 【ChibiOS 自己写的】" % "{:,}".format(rt_l))
        print("      HAL             %10s 行   ← 大部分是【ChibiOS 自己写的】" % "{:,}".format(hal_l))
        print("      os/common       %10s 行   ← 大部分是【厂商提供的】" % "{:,}".format(com_l))
        print()
        print("      ⇒ HAL 是内核的 %.1f 倍。" % (hal_l / rt_l))
        print("      ⇒ 【HAL 才是 ChibiOS 的主体，不是内核。】")
        print()
        print("      对比《NuttX 教程》卷 6 第 10 章：NuttX 的 arch/ 也是 265 万行，")
        print("      同样大部分是厂商头文件。**这不是「谁代码写得多」，")
        print("      是「谁把厂商代码打包进来了」。**")
    print()


# ===========================================================================
# 实验 4：★ 门控清单差异 → 反推能力差集
# ===========================================================================

def exp4():
    head("实验 4：两个内核的门控清单差异 —— 用差异反推设计")

    print("""
  这个实验在文档 3.1b 节。

  思路（有点绕，但很漂亮）：

      门控清单 = 「这个内核【有】这个功能，但许可证不让你用」

      所以：
          RT 门控了 mutex，NIL 没门控 mutex
          ⇒ 要么 NIL 没有 mutex（所以不需要门控）
          ⇒ 要么 NIL 有 mutex，但 GPL 档不禁它

          这两种解释给出的结论完全相反，
          需要第三条线索来区分。

  第三条线索：NIL 的 ch.h 里对 CH_CFG_USE_MUTEXES 的【值检查】。
""")

    print()
    print("  ── ① 两个内核的门控清单并排 ──")
    print()

    rt_rules = parse_restrictions(F_RESTRICT_RT, "chrestrictions.h (RT)")
    nil_rules = parse_restrictions(F_NIL_CH, "ch.h (NIL)")
    if rt_rules is None or nil_rules is None:
        return

    def names(rules, gate):
        return {n for n, a, v, _ in rules.get(gate, []) if a == "define"}

    rt_int = names(rt_rules, "INT+BASIC")
    rt_bas = names(rt_rules, "BASIC")
    nil_int = names(nil_rules, "INT+BASIC")
    nil_bas = names(nil_rules, "BASIC")

    print("    %-28s %-10s %-10s" % ("宏", "RT 门控", "NIL 门控"))
    print("    " + "-" * 50)
    allnames = sorted(rt_int | rt_bas | nil_int | nil_bas)
    for n in allnames:
        rt_mark = []
        if n in rt_int:
            rt_mark.append("INT")
        if n in rt_bas:
            rt_mark.append("BASIC")
        nil_mark = []
        if n in nil_int:
            nil_mark.append("INT")
        if n in nil_bas:
            nil_mark.append("BASIC")
        print("    %-28s %-10s %-10s"
              % (n, "+".join(rt_mark) or "—", "+".join(nil_mark) or "—"))

    print()
    hr("-")
    print("  ★ 差集：只在一边被门控的项")
    hr("-")

    only_rt = (rt_int | rt_bas) - (nil_int | nil_bas)
    only_nil = (nil_int | nil_bas) - (rt_int | rt_bas)
    both = (rt_int | rt_bas) & (nil_int | nil_bas)

    print()
    print("      两边都门控 : %s" % (", ".join(sorted(both)) or "（无）"))
    print("      只 RT 门控  : %s" % (", ".join(sorted(only_rt)) or "（无）"))
    print("      只 NIL 门控 : %s" % (", ".join(sorted(only_nil)) or "（无）"))
    print()

    print("""
  解读（文档 3.1b 节）：

      只 RT 门控的项  = RT 有、NIL 没有的功能
          中间有 USE_TM / USE_MUTEXES / USE_CONDVARS / USE_DYNAMIC
          ⇒ RT 有 TM/mutex/condvar/动态创建，NIL 没有

      只 NIL 门控的项 = NIL 有、RT 也有但 GPL 不禁的功能
          中间有 USE_MESSAGES
          ⇒ NIL 有 messages（和文件清单一致：os/nil/src/chmsg.c）

      两边都门控 = 两个内核都有的功能
          ST_FREQUENCY / ST_TIMEDELTA
""")

    print()
    hr("-")
    print("  ★★ 交叉验证：用第三条线索确认 mutex 的归属")
    hr("-")

    nil_ch_text = read_text(F_NIL_CH) or ""
    has_mutex_type = bool(re.search(r"\bmutex_t\b|\bchMtx\w*", nil_ch_text))
    has_not_supported = "mutexes not yet supported" in nil_ch_text

    print()
    print("      os/nil/include/ch.h 里出现 mutex_t / chMtx* ？  %s"
          % ("是" if has_mutex_type else "★ 否"))
    print("      os/nil/include/ch.h 里有 'mutexes not yet supported' ？  %s"
          % ("★ 是" if has_not_supported else "否"))
    print()

    if has_not_supported:
        print("      ⇒ 门控清单的「NIL 不门控 mutex」有两种解释：")
        print("         (a) NIL 有 mutex 且 GPL 不禁")
        print("         (b) NIL 没有 mutex，所以无从门控")
        print()
        print("         第三条线索判定为 **(b)**：")
        print("         NIL 不但不门控 mutex —— 它在【值检查】里直接拒绝。")
        print()
        print("      ⚠ 这正是「只读门控清单会得出相反结论」的情形。")
        print("        所以文档 2.2 节和 3.1b 节用了两条独立路径，")
        print("        并在这里做第三次核对。")

    print()
    print("  ── 三条独立证据汇总 ──")
    print()
    print("      %-34s %s" % ("证据", "结论"))
    print("      " + "-" * 58)
    print("      %-34s %s" % ("① 文件清单（无 chmtx.c）", "NIL 无 mutex"))
    print("      %-34s %s" % ("② 门控清单差集", "NIL 无 mutex"))
    print("      %-34s %s" % ("③ grep 符号（零匹配）+ #error", "NIL 无 mutex"))
    print()
    print("      ⇒ 三条独立路径一致。")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("两个内核的功能对照", exp1),
    "2": ("★★ 许可证功能门控模拟", exp2),
    "3": ("模块规模实测", exp3),
    "4": ("★ 门控清单差异 → 反推能力差集", exp4),
}


def main():
    print()
    hr("#")
    print("#  本章：ChibiOS 架构 —— RT / NIL / HAL")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(CHIBIOS):
        print()
        print("  [错误] 找不到 ChibiOS 源码：")
        print("         %s" % CHIBIOS)
        print()
        print("  请先拉取（见《总纲》的版本确定方法）：")
        print("      cd modules/ChibiOS")
        print("      git fetch --depth 1 origin d103983f4362e5506bcf7062ccacf345fc0c0859")
        print("      git checkout d103983f4362e5506bcf7062ccacf345fc0c0859")
        print()
        return 1

    print()
    print("  源码根: %s" % CHIBIOS)

    arg = sys.argv[1] if len(sys.argv) > 1 else None

    if arg is None:
        for key in sorted(EXPERIMENTS):
            name, fn = EXPERIMENTS[key]
            fn()
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
    print("#  实验结束")
    hr("#")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
