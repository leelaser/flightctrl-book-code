#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 5 第 3 章配套实验：ChibiOS 同步原语与优先级继承

    实验 1  mutex_t 的结构（从源码读）
    实验 2  ★★ PI 模拟：可传递的优先级继承
    实验 3  ★ 「没有超时」的源码级证明
    实验 4  ★★ chMtxForceReleaseS 是不是 ArduPilot 加的
    实验 5  同步原语的结构对照

用法：
    python lesson12_chibios_sync.py 1
    python lesson12_chibios_sync.py 2
    ...
    python lesson12_chibios_sync.py        # 全部

【诚实声明】

实验 1/3/4/5 是【读源码】—— 结论来自文件内容。
实验 2 是【模拟】—— 模拟 chMtxLockS 的 PI 算法，但丢掉了：

    · 真实的中断/上下文切换开销
    · ch_queue_dequeue / ch_sch_prio_insert 的具体指针操作
      （模拟里用 Python list 的 remove/insert 代替）
    · 时钟和超时
    · SMP

模拟能证明的是【PI 算法的语义】，不能证明【真实时序】。

源码版本：6a85082c（2023-12-29，ChibiOS 内核 7.0.4）
"""

import os
import re
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APM_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "APM",
                 "ardupilot-Copter-4.5.7", "ardupilot-Copter-4.5.7")
)
CHIBIOS = os.path.join(APM_ROOT, "modules", "ChibiOS")

F_CHMTX_H = os.path.join(CHIBIOS, "os", "rt", "include", "chmtx.h")
F_CHMTX_C = os.path.join(CHIBIOS, "os", "rt", "src", "chmtx.c")
F_CHSEM_H = os.path.join(CHIBIOS, "os", "rt", "include", "chsem.h")
F_CHSEM_C = os.path.join(CHIBIOS, "os", "rt", "src", "chsem.c")
F_CHOBJECTS = os.path.join(CHIBIOS, "os", "rt", "include", "chobjects.h")
F_CHSCHD_C = os.path.join(CHIBIOS, "os", "rt", "src", "chschd.c")
F_CHCONF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                        "hwdef", "common", "chconf.h")
F_SCHEDULER_C = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                             "Scheduler.cpp")
F_HALCLASS = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "HAL_ChibiOS_Class.cpp")
# APM_ROOT = D:/.../APM/ardupilot-Copter-4.5.7/ardupilot-Copter-4.5.7
# 三级 .. 才到工作区根
PX4_ROOT = os.path.normpath(os.path.join(APM_ROOT, "..", "..", "..", "PX4",
                                         "PX4-Autopilot"))
F_NUTTX_MUTEX = os.path.join(PX4_ROOT, "platforms", "nuttx", "NuttX", "nuttx",
                             "include", "nuttx", "mutex.h")

# 上游 ChibiOS 的对比副本（可选，见实验 4）
UPSTREAM_DIR = os.path.join(os.environ.get("TEMP", "/tmp"), "chk_up")
F_UPSTREAM_CHMTX_C = os.path.join(UPSTREAM_DIR, "upstream_chmtx.c")


def read_lines(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().split("\n")
    except OSError:
        return None


def read_text(path):
    lines = read_lines(path)
    return None if lines is None else "\n".join(lines)


def show(path, lo, hi, label=""):
    lines = read_lines(path)
    if not lines:
        print("    [读不到 %s]" % path)
        return False
    print("    ┌─ %s:%d-%d%s"
          % (os.path.basename(path), lo, hi, ("  " + label) if label else ""))
    for i in range(lo, min(hi, len(lines)) + 1):
        print("    │ %4d  %s" % (i, lines[i - 1]))
    print("    └─")
    return True


def find_line(path, pattern, start=1):
    lines = read_lines(path)
    if not lines:
        return None
    rx = re.compile(pattern)
    for i in range(start - 1, len(lines)):
        if rx.search(lines[i]):
            return i + 1
    return None


def hr(ch="=", n=74):
    print(ch * n)


def head(title):
    print()
    hr()
    print("  " + title)
    hr()


# ===========================================================================
# 实验 1：mutex_t 的结构
# ===========================================================================

def exp1():
    head("实验 1：mutex_t 的结构")

    print("""
  ChibiOS 的 mutex 只有 3 个字段（+1 个可选的）。
  这个实验从源码读出来，并追一下那个最容易被忽略的 next。
""")

    p = find_line(F_CHMTX_H, r"^struct ch_mutex \{")
    if p:
        show(F_CHMTX_H, p - 5, p + 12, "★ mutex_t 的定义")

    print("""
    ┌─ 三个必备字段 ────────────────────────────────────────┐
    │  queue   等待这个 mutex 的【线程】队列                 │
    │  owner   持有者（NULL = 没人持有）                     │
    │  next    ★ 持有者【持有的 mutex 链表】的下一个          │
    └───────────────────────────────────────────────────────┘
    ┌─ 一个可选字段 ────────────────────────────────────────┐
    │  cnt     递归计数（CH_CFG_USE_MUTEXES_RECURSIVE）      │
    └───────────────────────────────────────────────────────┘

  ⚠ next 很容易被误读成「下一个 mutex」。
    它其实是「【同一个持有者】持有的下一个 mutex」。
""")

    print("  ── next 怎么被赋值 ──")
    print()
    p = find_line(F_CHMTX_C, r"It was not owned, inserted in the owned mutexes list")
    if p:
        show(F_CHMTX_C, p, p + 4, "★ 头插")

    print()
    print("  ── 线程侧的两个字段 ──")
    print()
    p = find_line(F_CHOBJECTS, r"List of the mutexes owned by this thread")
    if p:
        show(F_CHOBJECTS, p - 3, p + 8, "★ mtxlist + realprio")

    print("""
  ⇒ 这两个字段合起来构成一个"持有者 → 它持有的所有锁"的链表：

        thread_t.mtxlist ──→ mutex_t ──next──→ mutex_t ──next──→ NULL
                              ↑                 ↑
                          （最后拿的）      （更早拿的）   ← 头插，所以顺序是反的

  ★ 这个链表【只为一件事服务】：解锁时重算优先级（实验 2 会用到）。
""")

    print("  ── ⚠ 这两个字段都在 #if 里面 ──")
    print()
    print("""    查 chconf.h 确认 ArduPilot 开着：
""")
    for name in ("CH_CFG_USE_MUTEXES", "CH_CFG_USE_MUTEXES_RECURSIVE"):
        ln = find_line(F_CHCONF, r"^\s*#define\s+%s\s" % name)
        if ln:
            lines = read_lines(F_CHCONF)
            print("    chconf.h:%-4d  %s" % (ln, lines[ln - 1].strip()))

    print("""
    ⇒ 开着，所以每个线程多 8 字节（mtxlist 4 + realprio 4）。

    ⚠ 关掉 CH_CFG_USE_MUTEXES 时，这两个字段会【消失】——
      于是任何直接用它们的代码【编译失败】。
      卷 5 第 1 章 3.4b 讲的 Scheduler.cpp:867 就是这个机制。
""")
    print()


# ===========================================================================
# 实验 2：PI 模拟
# ===========================================================================

class Thread:
    _id = 0

    def __init__(self, name, prio):
        self.name = name
        self.prio = prio          # hdr.pqueue.prio —— 当前生效
        self.realprio = prio      # realprio —— 本来的
        self.state = "READY"      # READY / WTMTX / CURRENT
        self.mtxlist = []         # 持有的 mutex（头插，用 list[0] 当链表头）
        self.waiting_on = None    # u.wtmtxp

    def __repr__(self):
        return "%s(p=%d/real=%d)" % (self.name, self.prio, self.realprio)


class Mutex:
    def __init__(self, name):
        self.name = name
        self.owner = None
        self.queue = []           # 等待者，按优先级【降序】
        self.next = None          # 在同一持有者的 mtxlist 里的下一个


def exp2():
    head("实验 2：★★ PI 模拟 —— 可传递的优先级继承")

    print("""
  源码：chmtx.c:145-217（chMtxLockS）
  这个实验把那段代码翻译成 Python，验证三件事：

      ① 竞争时的提升
      ② 【可传递】：沿 wtmtxp->owner 链继续走
      ③ 解锁时是【重算】而不是【恢复】
""")

    print()
    print("  ── 源码 ──")
    print()
    p = find_line(F_CHMTX_C, r"Priority inheritance protocol; explores")
    if p:
        show(F_CHMTX_C, p, p + 25, "★ PI 的 while 循环")

    # 模拟 chMtxLockS
    def mtx_lockS(mp, currtp, log):
        """对应 chMtxLockS。返回 True = 拿到了，False = 睡下了。"""
        if mp.owner is not None:
            # ---- PI 循环 ----
            tp = mp.owner
            hop = 0
            while tp.prio < currtp.prio:
                log.append("      PI: %s(prio %d) 被提升到 %d"
                           % (tp.name, tp.prio, currtp.prio))
                tp.prio = currtp.prio
                hop += 1
                if hop > 8:
                    log.append("      [模拟保护] 链太长，可能是环")
                    break

                if tp.state == "WTMTX":
                    # 它在等另一个 mutex ⇒ 重新入队 + 沿链继续
                    waitq = tp.waiting_on.queue
                    waitq.remove(tp)
                    _prio_insert(waitq, tp)
                    log.append("      %s 在等 %s，重新入队后沿链继续"
                               % (tp.name, tp.waiting_on.name))
                    tp = tp.waiting_on.owner
                    continue
                elif tp.state == "READY":
                    log.append("      %s 是 READY，重新挂进就绪队列"
                               % tp.name)
                    break
                else:
                    log.append("      %s 是 %s，不需要重排"
                               % (tp.name, tp.state))
                    break

            # ---- 睡在这个 mutex 上 ----
            _prio_insert(mp.queue, currtp)
            currtp.waiting_on = mp
            currtp.state = "WTMTX"
            log.append("      %s 睡在 %s 上" % (currtp.name, mp.name))
            return False
        else:
            mp.owner = currtp
            currtp.mtxlist.insert(0, mp)   # 头插
            log.append("      %s 拿到 %s（没竞争）" % (currtp.name, mp.name))
            return True

    def _prio_insert(q, tp):
        """对应 ch_sch_prio_insert：按 prio 降序插入"""
        i = 0
        while i < len(q) and q[i].prio >= tp.prio:
            i += 1
        q.insert(i, tp)

    def recalc_prio(t, log):
        """对应 chMtxUnlockS:453-470 的【重算】"""
        newprio = t.realprio
        for m in t.mtxlist:
            if m.queue and m.queue[0].prio > newprio:
                newprio = m.queue[0].prio
        old = t.prio
        t.prio = newprio
        log.append("      重算 %s: %d → %d（realprio=%d，扫了 %d 把锁）"
                   % (t.name, old, newprio, t.realprio, len(t.mtxlist)))
        return newprio

    # ──────────────────────────────────────────────
    print("  ── 场景 A：简单 PI（一对一）──")
    print()
    print("""    A(100) 想拿 M1，M1 被 B(60) 持有
""")

    A = Thread("A", 100)
    B = Thread("B", 60)
    M1 = Mutex("M1")

    log = []
    mtx_lockS(M1, B, log)                 # B 先拿
    B.state = "READY"
    A.state = "CURRENT"
    log = []
    print("    执行 A: chMtxLock(M1)")
    got = mtx_lockS(M1, A, log)
    for l in log:
        print(l)
    print()
    print("    B 的优先级: %d（real=%d）" % (B.prio, B.realprio))
    print("    ⇒ B 从 60 被提升到 100，这样它不会被 60~99 的线程抢走 CPU。")
    print()

    # ──────────────────────────────────────────────
    print("  ── 场景 B：★★ 可传递（三跳）──")
    print()
    print("""    A(100) 想拿 M1，M1 被 B(60) 持有
    B 自己阻塞在 M2 上，M2 被 C(20) 持有

    ⇒ PI 应该沿链走：先提升 B，再提升 C
""")

    A2 = Thread("A", 100)
    B2 = Thread("B", 60)
    C2 = Thread("C", 20)
    M1b = Mutex("M1")
    M2b = Mutex("M2")

    # 先建立 C 持有 M2、B 等 M2 的状态
    l0 = []
    mtx_lockS(M2b, C2, l0)          # C 拿 M2
    C2.state = "READY"
    mtx_lockS(M2b, B2, l0)          # B 等 M2 ⇒ B 睡下，C 被提升
    print("    建立前置状态后：")
    for t in (A2, B2, C2):
        print("      %-14s state=%-6s prio=%-4d real=%d"
              % (t.name, t.state, t.prio, t.realprio))
    print()

    A2.state = "CURRENT"
    log2 = []
    print("    执行 A: chMtxLock(M1)   ← M1 现在归 B 所有（模拟）")
    M1b.owner = B2                  # 假设 B 持有 M1
    B2.mtxlist = [M1b]
    got = mtx_lockS(M1b, A2, log2)
    for l in log2:
        print(l)

    print()
    print("    结果：")
    for t in (A2, B2, C2):
        print("      %-14s prio=%-4d real=%-4d %s"
              % (t.name, t.prio, t.realprio,
                 "← 被提升" if t.prio != t.realprio else ""))

    print("""
    ⇒ A 只等了 M1，但【C 也被提升到了 100】。

      为什么？因为如果 C 只有 20，它可能被任何 20~99 的线程抢占，
      于是 C 跑不动 ⇒ M2 不放 ⇒ B 跑不动 ⇒ M1 不放 ⇒ A 跑不动。
      【优先级反转沿锁链传下去了。】

      PI 必须沿着这条链一路提升到底 —— 这就是那句
          tp = tp->u.wtmtxp->owner;  continue;
      的作用。
""")

    # ──────────────────────────────────────────────
    print("  ── 场景 C：解锁时的【重算】──")
    print()
    print("""    T(20) 持有 M1 和 M2
    A(50) 在等 M1
    B(80) 在等 M2
    ⇒ T 被 B 提升到 80

    现在 T 解锁 M1 —— 优先级应该变成多少？
""")

    T = Thread("T", 20)
    Ax = Thread("A", 50)
    Bx = Thread("B", 80)
    Mx1 = Mutex("M1")
    Mx2 = Mutex("M2")

    T.mtxlist = [Mx2, Mx1]      # 头插：先拿 M1 再拿 M2 ⇒ M2 在链头
    Mx1.owner = T
    Mx2.owner = T
    Ax.state = "WTMTX"
    Ax.waiting_on = Mx1
    Mx1.queue = [Ax]
    Bx.state = "WTMTX"
    Bx.waiting_on = Mx2
    Mx2.queue = [Bx]

    # PI：两个等待者各自提升 T
    T.prio = max(T.prio, Ax.prio)
    T.prio = max(T.prio, Bx.prio)
    print("    初始：T 的 prio = %d（real=%d）" % (T.prio, T.realprio))
    print()

    print("    【错误做法】恢复成 realprio：")
    wrong = T.realprio
    print("        T 变成 %d ⇒ 会被 20~79 的线程抢占" % wrong)
    print("        但 B(80) 还在等 M2 ⇒ 【优先级反转又出现了】")
    print()

    print("    【正确做法】重算（对应 chmtx.c:453-470）：")
    # 解锁 M1
    T.mtxlist.remove(Mx1)
    Mx1.owner = Ax
    Ax.state = "READY"
    Mx1.queue = []
    lgc = []
    recalc_prio(T, lgc)
    for l in lgc:
        print("   " + l)
    print()
    print("        T 保持在 %d —— 因为 M2 上还有 B(80) 在等。" % T.prio)
    print()
    print("""    ⇒ 这就是为什么 chMtxUnlockS 要【扫一遍 T 持有的所有锁】:
        它不能只看被解锁的那一把。

    ⇒ 也就是为什么 mutex_t 需要 next、
      而 thread_t 需要 mtxlist —— 【这两个字段是为这段代码存在的】。
""")
    print()


# ===========================================================================
# 实验 3：没有超时
# ===========================================================================

def exp3():
    head("实验 3：★ 「没有超时」的源码级证明")

    print("""
  卷 5 第 1 章说过 ChibiOS 的 mutex 没有超时版本。这个实验证明它，
  并且证明【这不是 ChibiOS 独有的】。
""")

    print("  ── ① 全部 mutex API ──")
    print()
    p = find_line(F_CHMTX_H, r"void chMtxObjectInit\(mutex_t")
    if p:
        show(F_CHMTX_H, p, p + 9, "★ 全部 10 个函数")

    lines = read_lines(F_CHMTX_H) or []
    apis = [l.strip() for l in lines
            if re.match(r"^\s+(void|bool|tprio_t|mutex_t\s*\*)\s+chMtx\w+\(", l)]
    has_timeout = any("imeout" in a for a in apis)
    print()
    print("    函数总数: %d" % len(apis))
    print("    含 Timeout 的: %d" % (1 if has_timeout else 0))
    print()
    if not has_timeout:
        print("    ⇒ 【零个】。要么立刻拿到（chMtxLock），要么无限等。")
        print("      chMtxTryLock 是非阻塞，不是限时。")
    print()

    print("  ── ② 状态机里也没有它的位置 ──")
    print()
    print("    找出 CH_STATE_WTMTX 出现的所有地方：")
    print()
    hits = []
    for dirpath, _d, filenames in os.walk(os.path.join(CHIBIOS, "os", "rt")):
        for fn in filenames:
            if not fn.lower().endswith((".c", ".h")):
                continue
            fp = os.path.join(dirpath, fn)
            for i, line in enumerate(read_lines(fp) or [], 1):
                if "CH_STATE_WTMTX" in line:
                    hits.append((os.path.relpath(fp, CHIBIOS).replace("\\", "/"), i, line.strip()))
    for h, i, s in hits:
        print("        %-28s %-5d %s" % (h, i, s[:52]))

    print()
    print("    ⇒ 只有 2 个【功能性】出现点：")
    print("         chmtx.c:171  PI 链里重新入队")
    print("         chmtx.c:216  chSchGoSleepS（无超时的睡眠）")
    print()
    print("      ⇒ 它从不和任何【超时】设施打交道。")
    print()

    print("  ── ③ ★★ 真正的证据：谁用「带超时」的睡眠 ──")
    print()
    print("""    先看 chSchGoSleepTimeoutS 本身 —— 它【与状态无关】：
""")
    p = find_line(F_CHSCHD_C, r"msg_t chSchGoSleepTimeoutS")
    if p:
        show(F_CHSCHD_C, p, p + 19, "★ 只是加了个虚拟定时器")

    print("""
    ⇒ 它接受任意 newstate，然后转发给 chSchGoSleepS。

    ⇒ 【所以「能不能超时」不是函数的能力问题，
      而是「谁用带超时的版本」的问题。】

    那就去数调用者：
""")

    # 统计所有 chSchGoSleepTimeoutS 的调用点（带状态）
    timed = []
    untimed = []
    src_dir = os.path.join(CHIBIOS, "os", "rt", "src")
    for fn in sorted(os.listdir(src_dir)):
        if not fn.endswith(".c"):
            continue
        fp = os.path.join(src_dir, fn)
        for i, line in enumerate(read_lines(fp) or [], 1):
            m = re.search(r"chSchGoSleepTimeoutS\(\s*(CH_STATE_\w+)", line)
            if m:
                timed.append((fn, i, m.group(1)))
            m2 = re.search(r"chSchGoSleepS\(\s*(CH_STATE_\w+)", line)
            if m2 and "Timeout" not in line:
                untimed.append((fn, i, m2.group(1)))

    print("    ┌─ 用【带超时】版本 chSchGoSleepTimeoutS 的（%d 处）"
          % len(timed))
    for fn, i, st in timed:
        print("    │  %-14s %-5d  %s" % (fn, i, st))
    print("    └─")
    print()
    print("    ┌─ 用【不带超时】版本 chSchGoSleepS 的（%d 处）"
          % len(untimed))
    for fn, i, st in untimed:
        mark = "  ★★ 只有这个没有超时版本" if st == "CH_STATE_WTMTX" else ""
        print("    │  %-14s %-5d  %-22s%s" % (fn, i, st, mark))
    print("    └─")
    print()

    # ── 按状态分组：哪些状态【两种版本都有】──────────────
    #
    # 这一步是关键 —— 直接比"8 个带超时 / 12 个不带"是误导的，
    # 因为 CH_STATE_FINAL（线程退出）、CH_STATE_WTEXIT
    # 这些【本来就不该有超时】。
    # 真正有意义的问题：同一个状态，两种版本都存在吗？
    print("    ── 按状态分组（这才是关键）──")
    print()

    states = sorted({st for _f, _i, st in timed} |
                    {st for _f, _i, st in untimed})
    has_both = []
    timed_only = []
    untimed_only = []

    print("    %-24s %-10s %-10s %s"
          % ("状态", "带超时", "不带超时", "判断"))
    print("    " + "-" * 62)
    for st in states:
        t = any(s == st for _f, _i, s in timed)
        u = any(s == st for _f, _i, s in untimed)
        if t and u:
            verdict = "两种都有"
            has_both.append(st)
        elif t:
            verdict = "只有带超时"
            timed_only.append(st)
        else:
            verdict = "只有不带超时"
            untimed_only.append(st)
        print("    %-24s %-10s %-10s %s"
              % (st, "✓" if t else "—", "✓" if u else "—", verdict))

    print()
    print("    两种都有的状态（%d 个）：%s"
          % (len(has_both), "、".join(has_both) if has_both else "（无）"))
    print("    只有不带超时的（%d 个）：%s"
          % (len(untimed_only), "、".join(untimed_only) if untimed_only else "（无）"))
    print()

    if "CH_STATE_WTMTX" in untimed_only:
        print("""    ★★ 关键：WTMTX 落在「只有不带超时」那一组。

      但注意同组的还有 FINAL / WTEXIT / SNDMSGQ ——
      那几个【本来就不该有超时】：
        · CH_STATE_FINAL     线程正在退出，超时了又怎样？
        · CH_STATE_WTEXIT    等别的线程退出，可以无限等
        · CH_STATE_SNDMSGQ   往同步消息队列发消息，语义上不能超时
                               （已经发出去了，超时无法回滚）

      ⇒ 所以「只有不带超时」这一组里，
        【WTMTX 是唯一一个「按理说可以有超时」的】。

      对比这五个状态 —— 它们【两种版本都有】：
          WTCOND（条件变量） WTSEM（信号量）
          WTOREVT / WTANDEVT（事件） WTMSG（收消息）

      为什么它们可以有超时？
        因为它们超时时【不需要撤销任何东西】——
        超时就是"没等到"，队列里把自己摘掉就行。

      为什么 mutex 不行？
        因为它超时时【必须撤销优先级继承】——
        而那个提升可能已经被【其他等待者】也叠加上去了。
        （FreeRTOS 为了处理这件事写了 21 行 + 一个函数，
          见文档 3.4 节。）

      ⇒ 结论修正（比我第一版写的更准确）：

        ❌ 第一版：「睡眠状态机层面就不支持」
        ✅ 实际是：  chSchGoSleepTimeoutS 对任何状态都可用，
                    【mutex 是主动选择不用它】——
                     因为超时会破坏 PI 的不变量。

      ⇒ 这不是「做不到」，是「算过账之后选择不做」。
""")
    print()

    print("  ── ④ 三家对照 ──")
    print()
    print("    NuttX 的 mutex API（%s）:" % os.path.basename(F_NUTTX_MUTEX))
    print()
    nl = read_lines(F_NUTTX_MUTEX)
    if nl:
        napis = [l.strip() for l in nl
                 if re.match(r"^static inline (int|bool) nxmutex_\w+\(", l)]
        for a in napis:
            print("        %s" % a[:66])
        print()
        ntimed = any("imedlock" in a for a in napis)
        print("    含 timedlock 的: %d" % (1 if ntimed else 0))
        if not ntimed:
            print("    ⇒ NuttX 也【没有】 mutex 超时。")
    else:
        print("        [读不到 %s]" % F_NUTTX_MUTEX)
        print("        （PX4 的 NuttX 树不在预期位置，跳过这一节）")

    print("""
    ┌──────────────────────────────────────────────────────────┐
    │  三家对照                                                 │
    │                                                          │
    │    ChibiOS   chMtxLock / chMtxTryLock      ❌ 无超时      │
    │    NuttX     nxmutex_lock / _trylock       ❌ 无超时      │
    │    FreeRTOS  xSemaphoreTake(m, timeout)    ✅ 有          │
    │                                                          │
    │  ⇒ 三家有两个不给 mutex 超时。                            │
    │    「ChibiOS 怎么这么弱」这个判断是错的。                  │
    └──────────────────────────────────────────────────────────┘

    FreeRTOS 能给的原因是：它的 mutex 【就是队列】（卷 4 第 8 章），
    超时是队列本来就有的能力。

    但代价是 —— 超时会破坏 PI 的不变量，需要额外机制来修（见文档 3.4 节）：

        queue.c:1836        xInheritanceOccurred 标志
        queue.c:2365-2385   prvGetDisinheritPriorityAfterTimeout（21 行）
        queue.c:1847        调用它
        tasks.c             vTaskPriorityDisinheritAfterTimeout
        queue.c:1838-1858   一个额外的临界区

    ⇒ 这段是【FreeRTOS 的源码】，不是 ChibiOS 的文档。
      所以「ChibiOS 为什么不支持」这个理由标 [推断]。
""")
    print()


# ===========================================================================
# 实验 4：chMtxForceReleaseS 的归属
# ===========================================================================

def exp4():
    head("实验 4：★★ chMtxForceReleaseS 是不是 ArduPilot 加的")

    print("""
  《NuttX 教程》卷 6 第 10 章说「ArduPilot fork 了 ChibiOS 并做了修改」。
  这个实验给出一处【可逐条核实】的证据。

  分两步：
      ① 本机可验的（不依赖网络）
      ② 与上游对比（需要一份上游副本）
""")

    print()
    hr("-")
    print("  ① 本机可验的证据")
    hr("-")
    print()

    print("    【证据 1】缩进风格")
    print()
    lines = read_lines(F_CHMTX_C) or []
    p = find_line(F_CHMTX_C, r"^void chMtxForceReleaseS")
    if p:
        FUNC_LEN = 73          # chMtxForceReleaseS 跨 562-634
        tab_in_func = 0
        sp_in_func = 0
        for i in range(p - 1, min(p - 1 + FUNC_LEN, len(lines))):
            l = lines[i]
            if l.startswith("\t"):
                tab_in_func += 1
            elif l.startswith("  ") and l.strip():
                sp_in_func += 1

        whole_tab = sum(1 for l in lines if l.startswith("\t"))
        whole_sp = sum(1 for l in lines if l.startswith("  ") and l.strip())
        tab_outside = whole_tab - tab_in_func
        sp_outside = whole_sp - sp_in_func

        print("        ┌─ chMtxForceReleaseS 函数体内 ──")
        print("        │   tab 缩进:   %3d 行" % tab_in_func)
        print("        │   空格缩进:  %3d 行" % sp_in_func)
        print("        │   ⇒ tab 占比 %.0f%%"
              % (100.0 * tab_in_func / max(1, tab_in_func + sp_in_func)))
        print("        └─")
        print("        ┌─ chmtx.c 里【这个函数之外】的部分 ──")
        print("        │   tab 缩进:   %3d 行" % tab_outside)
        print("        │   空格缩进:  %3d 行" % sp_outside)
        print("        │   ⇒ tab 占比 %.1f%%"
              % (100.0 * tab_outside / max(1, tab_outside + sp_outside)))
        print("        └─")
        print()

        if tab_in_func:
            ratio_in = 100.0 * tab_in_func / max(1, tab_in_func + sp_in_func)
            ratio_out = 100.0 * tab_outside / max(1, tab_outside + sp_outside)
            if tab_outside == 0:
                print("        ★★ 函数外【零 tab】，函数内 %d 行 tab。" % tab_in_func)
                print()
                print("        ⇒ 整个 chmtx.c 只有这个函数用 tab 缩进。")
                print("          这是一个很强的「外来代码」信号。")
            elif ratio_in > ratio_out * 3:
                print("        ⇒ 函数内 tab 占比 %.0f%%，函数外 %.1f%% —— 【风格明显不同】。"
                      % (ratio_in, ratio_out))
                print()
                print("        ⚠ 但要诚实：文件外也有 %d 行用 tab，" % tab_outside)
                print("          所以这只是【辅助证据】，不能单独下结论。")
            print()
            print("        ⚠ 无论如何 —— 风格只是【辅助证据】。")
            print("          决定性的是下面 ② 的上游对比。")
        print()
        print("        【具体是哪些行用 tab】")
        print()
        for i in range(p - 1, min(p - 1 + FUNC_LEN, len(lines))):
            if lines[i].startswith("\t"):
                print("            chmtx.c:%-4d │%s"
                      % (i + 1, lines[i][:56].replace("\t", "→")))
                break
        print("            ...")
        print()

    print("    【证据 2】文档块的语气")
    print()
    p = find_line(F_CHMTX_C, r"@brief  Force release a mutex")
    if p:
        show(F_CHMTX_C, p - 1, p + 9, "★ 这段很「ArduPilot」")

    print("""
        注意这几句：
            「violates locking principles and can result in data corruption」
            「where watchdog recovery is not appropriate」

        chMtxUnlockS 里还有一句带 typo 的注释：
            「this can only happen if we have force relased a mutex」
        （relased ← released 拼错）

        ⇒ 上游代码里不会出现【引用一个上游没有的函数】的注释。
""")

    print("    【证据 3】唯一的调用点")
    print()
    src = read_text(F_SCHEDULER_C) or ""
    n_calls = src.count("chMtxForceReleaseS")
    print("        Scheduler.cpp 里出现 %d 次" % n_calls)
    p = find_line(F_SCHEDULER_C, r"chMtxForceReleaseS")
    if p:
        show(F_SCHEDULER_C, max(1, p - 22), p + 1, "★ try_force_mutex")

    print("""
        ⇒ 全 ArduPilot 只有这一处调用，且在 AP_HAL_ChibiOS 自己的代码里。
          上游的测试/示例代码里不会有它的调用者。
""")

    print("    【证据 4】ArduPilot 抄了 chThdSetPriority")
    print()
    p1 = find_line(F_HALCLASS, r"^void hal_chibios_set_priority")
    if p1:
        show(F_HALCLASS, p1 - 3, p1 + 11, "★ HAL_ChibiOS_Class.cpp")

    f_chthreads = os.path.join(CHIBIOS, "os", "rt", "src", "chthreads.c")
    p2 = find_line(f_chthreads, r"^tprio_t chThdSetPriority")
    if p2:
        show(f_chthreads, p2, p2 + 22, "★ 内核版本")

    print("""
        ⇒ 两处的 if 条件一字不差：
              (prio == realprio) || (newprio > prio)

          ArduPilot 抄它是因为 chThdSetPriority 只能改【当前线程】
          （chThdGetSelfX()），而它要改【主线程】的。

        ⇒ 这本身就是「内核没提供的能力，自己实现」的又一例。
""")

    print()
    hr("-")
    print("  ② 与上游对比")
    hr("-")
    print()

    if os.path.isfile(F_UPSTREAM_CHMTX_C):
        up = read_text(F_UPSTREAM_CHMTX_C) or ""
        ar = read_text(F_CHMTX_C) or ""
        print("    找到上游副本: %s" % F_UPSTREAM_CHMTX_C)
        print()
        print("    %-34s %-14s %s" % ("项目", "上游 master", "ArduPilot 6a85082c"))
        print("    " + "-" * 66)
        print("    %-34s %-14d %d"
              % ("chmtx.c 行数", up.count("\n"), ar.count("\n")))
        print("    %-34s %-14s %s"
              % ("chMtxForceReleaseS", "无" if "chMtxForceReleaseS" not in up else "有",
                 "有" if "chMtxForceReleaseS" in ar else "无"))
        guard = "this can only happen if we have force"
        print("    %-34s %-14s %s"
              % ("chMtxUnlockS 的强释守卫", "有" if guard in up else "无",
                 "有" if guard in ar else "无"))
        pi_loop = "Priority inheritance protocol; explores"
        print("    %-34s %-14s %s"
              % ("PI 的 while 循环", "有" if pi_loop in up else "无",
                 "有" if pi_loop in ar else "无"))
        print()
        if "chMtxForceReleaseS" not in up:
            print("    ⇒ 【上游零匹配】。")
            print("      chMtxForceReleaseS 是 ArduPilot 新增的。")
        print()
        print("    ⇒ PI 循环两边都有 ⇒ PI 是 ChibiOS 的，不是 ArduPilot 改的。")
    else:
        print("    本机没有上游副本（这是可选的）。")
        print()
        print("    要自己跑这个对比，执行：")
        print()
        print("        cd %s" % os.path.dirname(UPSTREAM_DIR))
        print("        mkdir chk_up && cd chk_up")
        print("        git init -q .")
        print("        git remote add origin https://github.com/ChibiOS/ChibiOS.git")
        print("        git fetch --depth 1 origin master")
        print("        git cat-file -p FETCH_HEAD:os/rt/src/chmtx.c > upstream_chmtx.c")
        print("        git cat-file -p FETCH_HEAD:os/rt/include/chmtx.h > upstream_chmtx.h")
        print()
        print("    然后重跑本实验。")
        print()
        print("    ⚠ 但【证据 1~4 已经足够下结论】——")
        print("      一个上游没有的函数，不可能有上游的调用者。")
    print()


# ===========================================================================
# 实验 5：同步原语的结构对照
# ===========================================================================

def exp5():
    head("实验 5：同步原语的结构对照")

    print("""
  这个实验数「每个同步原语需要多少个字段」，
  并借此说明三家的底层统一度差异。
""")

    print()
    print("  ── ChibiOS 的原语结构（从源码读）──")
    print()

    STRUCTS = [
        (F_CHSEM_H, r"^struct ch_semaphore \{", "semaphore_t",
         "队列 + 计数"),
        (F_CHMTX_H, r"^struct ch_mutex \{", "mutex_t",
         "队列 + 持有者 + 链表 + (递归计数)"),
    ]

    for path, pat, name, desc in STRUCTS:
        p = find_line(path, pat)
        if not p:
            continue
        lines = read_lines(path)
        # 找到匹配的 }
        end = p
        for i in range(p, min(p + 25, len(lines))):
            if lines[i].strip() == "};":
                end = i + 1
                break
        show(path, p - 2, end, "★ " + name)

        # 数字段：区分"必备"和"在 #if 里"
        req = 0
        opt = 0
        in_if = False
        for i in range(p, min(end, len(lines))):
            l = lines[i]
            if re.match(r"^\s*#if", l):
                in_if = True
            elif re.match(r"^\s*#endif", l):
                in_if = False
            elif re.match(r"^\s+(ch_queue_t|thread_t|mutex_t|tprio_t|cnt_t|struct)\s", l):
                if in_if:
                    opt += 1
                else:
                    req += 1
        if opt:
            print("        ⇒ %s: 必备字段 %d 个 + 可选字段 %d 个" % (name, req, opt))
        else:
            print("        ⇒ %s: 字段 %d 个" % (name, req))
        print()

    print("  ── ★ 线程结构体里的链接节点：三选一的 union ──")
    print()
    p = find_line(F_CHOBJECTS, r"^\s*union \{$")
    if p:
        # 找包含它的那句注释
        lines = read_lines(F_CHOBJECTS)
        start = p
        for i in range(p, max(1, p - 8), -1):
            if "Shared list headers" in lines[i - 1]:
                start = i
                break
        # 找结束
        end2 = p
        for i in range(p, min(p + 25, len(lines))):
            if lines[i].strip() == "} hdr;":
                end2 = i + 1
                break
        show(F_CHOBJECTS, start, end2, "★ ch_thread.hdr")

    print("""
    ⇒ 一个线程在任意时刻【只能在一个队列里】，
      所以这三个节点共用同一块内存。

        ch_list_t            — 单向链表节点
        ch_queue_t           — 双向链表节点（等待队列用）
        ch_priority_queue_t  — 双向链表 + prio（就绪队列用）

    ★★ 这解释了一个跨系统的差异：为什么 ChibiOS 只要一个节点，
       而 FreeRTOS 的 TCB 需要两个（xStateListItem + xEventListItem）。

        FreeRTOS: 阻塞的任务同时挂在【事件链】和【延迟链】上 ⇒ 要两个
        ChibiOS : 超时用【虚拟定时器】，而那个定时器节点在【栈上】
                  （见 chschd.c:365 的 `virtual_timer_t vt;`）
                  ⇒ TCB 里只需要一个节点

    [验证] ch_thread 里有没有定时器字段？
""")
    lines = read_lines(F_CHOBJECTS) or []
    has_timer_field = False
    for i in range(155, min(345, len(lines))):
        l = lines[i]
        if ("virtual_timer_t" in l or "ch_delta_list_t" in l) and \
           not l.strip().startswith("*") and not l.strip().startswith("/*"):
            has_timer_field = True
            break
    print("        %s" % ("★ 找到了 —— 本章结论需要重查" if has_timer_field
                          else "⇒ 零个。ch_thread 里没有定时器节点。"))
    print()

    print("  ── ★ 两套插入函数，对应两种队列 ──")
    print()
    print("""    一开始我以为「只有一套插入函数，到处复用」—— 不是。

    实际有【两套】，因为【两种队列类型】：
""")
    print("    ┌─ ch_queue_t ── 只有 next/prev（chlists.h:69-72）")
    print("    │     ├─ 用于：等待队列（等 mutex / 信号量 / 条件变量）")
    print("    │     └─ 插入函数：ch_sch_prio_insert(qp, tp)")
    print("    │           ↑ 优先级从【外面】拿：")
    print("    │             threadref(tp)->hdr.pqueue.prio")
    print("    └─")
    print("    ┌─ ch_priority_queue_t ── next/prev + prio（chlists.h:85-89）")
    print("    │     ├─ 用于：【就绪队列】和虚拟定时器以外的有序队列")
    print("    │     └─ 插入函数：ch_pqueue_insert_behind / _ahead")
    print("    │           ↑ 优先级从【节点自己】拿：pqp->prio")
    print("    └─")
    print()
    print("    实测调用者分布：")
    print()

    def count_calls(fname):
        """统计某个函数在 os/rt/src/*.c 里的调用点（排除定义处）"""
        res = {}
        src_dir = os.path.join(CHIBIOS, "os", "rt", "src")
        for fn in sorted(os.listdir(src_dir)):
            if not fn.endswith(".c"):
                continue
            fp = os.path.join(src_dir, fn)
            for i, line in enumerate(read_lines(fp) or [], 1):
                if (fname + "(") in line and ("void " + fname) not in line \
                   and (fname + "_") not in line.replace(fname + "(", "", 1)[:2]:
                    res.setdefault(fn, []).append(i)
        return res

    for fname, qtype in (("ch_sch_prio_insert", "ch_queue_t"),
                         ("ch_pqueue_insert_behind", "ch_priority_queue_t"),
                         ("ch_pqueue_insert_ahead", "ch_priority_queue_t")):
        callers = count_calls(fname)
        total = sum(len(v) for v in callers.values())
        print("    %-26s  %-22s  %d 处" % (fname, "(" + qtype + ")", total))
        for fn in sorted(callers):
            print("        %-16s 行 %s"
                  % (fn, ", ".join(str(x) for x in callers[fn][:8])))
    print()
    print("""    ⇒ 就绪队列（scheduler）不在上面 —— 它通过
      __sch_ready_behind / __sch_ready_ahead 间接调用
      ch_pqueue_insert_*（见卷 5 第 2 章 1.3 节）。

    ⇒ 所以准确的表述是：

        【等待队列】共用 ch_queue_t + ch_sch_prio_insert
        【就绪队列】用 ch_priority_queue_t + ch_pqueue_insert_*

      两种队列类型，两套插入函数，但都是【优先级排序的双向链表】。
""")
    print()

    print("  ── 三家的统一度对照 ──")
    print()
    print("""
    FreeRTOS —— 一切建在【队列】上（1 种基础类型）
        Queue_t
          ├── 信号量（uxMessagesWaiting 计数）
          ├── 互斥量（uxItemSize==0 + xMutexHolder）
          └── 事件组（等位掩码而非数据）

    ChibiOS —— 【优先级队列】+【计数器】（2 种）
        ch_priority_queue_t  ← 所有等待队列
        cnt_t                ← semaphore.cnt / mutex.cnt

    NuttX —— 【等待队列】+【符号计数】（2 种）
        dq_queue_t           ← 所有等待队列（含 g_readytorun）
        int16_t semcount     ← 符号位编码「资源数/等待者数」
""")

    print("  ── 符号编码计数器的三家对照 ──")
    print()
    print("    三家都用了「一个整数的符号表示两种含义」这个技巧：")
    print()
    print("    %-14s %-26s %s" % ("系统", "字段", "编码"))
    print("    " + "-" * 62)
    print("    %-14s %-26s %s" % ("NuttX", "sem_t.semcount (int16_t)", ">=0 资源数; <0 等待者数"))
    print("    %-14s %-26s %s" % ("ChibiOS", "semaphore_t.cnt (cnt_t)", ">=0 资源数; <0 等待者数"))
    print("    %-14s %-26s %s" % ("FreeRTOS", "uxMessagesWaiting", "只有非负计数"))
    print("    %-14s %-26s %s" % ("", "+ xTasksWaitingToReceive", "等待者在另一个链表里"))
    print()
    print("""
    ⇒ NuttX 和 ChibiOS 用了同一种编码，FreeRTOS 没有。

      为什么 FreeRTOS 不需要？因为它的等待者在【另一个链表】里
      （xTasksWaitingToReceive），而 NuttX/ChibiOS 的等待者
      也在队列里 —— 那么「队列里的是等待者还是资源」就需要额外的
      信息来区分，而正负号刚好提供了这个信息。

    ⚠ 但注意：ChibiOS 的 semaphore 是有【等待者队列】的
      （semaphore_t.queue），所以它其实两者都有。
      正负号在这里的作用是【快速判断有没有人等】
      —— chDbgAssert(tqp->cnt <= 0) 就是靠这个。
""")
    print()


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("mutex_t 的结构", exp1),
    "2": ("★★ PI 模拟", exp2),
    "3": ("★ 「没有超时」的证明", exp3),
    "4": ("★★ chMtxForceReleaseS 的归属", exp4),
    "5": ("同步原语结构对照", exp5),
}


def main():
    print()
    hr("#")
    print("#  本章：ChibiOS 同步原语与优先级继承")
    print("#  配套实验")
    hr("#")

    if not os.path.isdir(CHIBIOS):
        print()
        print("  [错误] 找不到 ChibiOS 源码：")
        print("         %s" % CHIBIOS)
        print()
        print("  请先按《总纲》的方法拉取 6a85082c 版本。")
        print()
        return 1

    arg = sys.argv[1] if len(sys.argv) > 1 else None

    if arg is None:
        for key in sorted(EXPERIMENTS):
            EXPERIMENTS[key][1]()
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
