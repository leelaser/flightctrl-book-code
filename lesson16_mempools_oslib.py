#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 5 第 7 章配套实验：内存池与 oslib

    实验 1  ★ core 内存：两端向中间分配的 arena
    实验 2  ★★ 堆的 first-fit + 合并（和 FreeRTOS heap_4 同一个算法）
    实验 3  ★★ 池 vs 堆：同一组分配序列，碎片差多少
    实验 4  ★★ heap_header 的 union：读与写的顺序纪律
    实验 5  ★★ ArduPilot 的多区路由（malloc.c 的回退链）

用法：
    python lesson16_mempools_oslib.py 1
    python lesson16_mempools_oslib.py 2
    ...
    python lesson16_mempools_oslib.py        # 全部

【诚实声明】

实验 1~5 都是【模拟】—— 每一段都对应真实的 C 代码，模拟的是它的
**数据结构语义和判定逻辑**，丢掉了：

    · 真实指针操作（模拟里用 Python 对象/list 代替）
    · 对齐的字节级计算（模拟里用页 = 8 字节的抽象单位）
    · mutex / 信号量的真实阻塞与优先级继承
    · 中断、SMP、编译器优化

**特别说明实验 2 和 3 的关系**：这两个是**同一组分配序列**在
两种分配器上的对照 —— 这正是卷 4 第 10 章（FreeRTOS `heap_1`~`heap_5`）
那个问题的另一种问法：**"合并"到底值多少钱**。

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
OSLIB = os.path.join(CHIBIOS, "os", "oslib")

F_MEMCORE_C = os.path.join(OSLIB, "src", "chmemcore.c")
F_MEMCORE_H = os.path.join(OSLIB, "include", "chmemcore.h")
F_MEMHEAPS_C = os.path.join(OSLIB, "src", "chmemheaps.c")
F_MEMHEAPS_H = os.path.join(OSLIB, "include", "chmemheaps.h")
F_MEMPOOLS_C = os.path.join(OSLIB, "src", "chmempools.c")
F_MEMPOOLS_H = os.path.join(OSLIB, "include", "chmempools.h")
F_OBJFIFOS_H = os.path.join(OSLIB, "include", "chobjfifos.h")
F_JOBS_H = os.path.join(OSLIB, "include", "chjobs.h")
F_MAILBOX_H = os.path.join(OSLIB, "include", "chmboxes.h")
F_PIPES_H = os.path.join(OSLIB, "include", "chpipes.h")
F_BSEM_H = os.path.join(OSLIB, "include", "chbsem.h")

F_MALLOC_C = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "hwdef", "common", "malloc.c")
F_CHCONF = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                        "hwdef", "common", "chconf.h")
F_HWDEF_PY = os.path.join(APM_ROOT, "libraries", "AP_HAL_ChibiOS",
                          "hwdef", "scripts", "chibios_hwdef.py")

# ===========================================================================
# 小工具
# ===========================================================================

def read_lines(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read().split("\n")
    except OSError:
        return []


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


def rel(path):
    """内核文件只留文件名；ArduPilot 的保留 libraries/ 前缀。"""
    p = os.path.normpath(path)
    if os.sep + "oslib" + os.sep in p or os.sep + "os" + os.sep + "rt" + os.sep in p:
        return os.path.basename(p)
    if p.startswith(APM_ROOT):
        rest = p[len(APM_ROOT) + 1:].replace(os.sep, "/")
        return rest if rest.startswith("libraries/") else os.path.basename(p)
    return p.replace(os.sep, "/")


def src(path, a, b, note=None):
    lines = read_lines(path)
    if not lines:
        print("      [读不到 %s]" % rel(path))
        return
    if note:
        print("      %s" % note)
    for i in range(max(0, a - 1), min(len(lines), b)):
        print("      %s:%d  %s" % (rel(path), i + 1, lines[i].rstrip()))
    print()


def grep_print(path, pattern, label=None, limit=8, width=100):
    lines = read_lines(path)
    rx = re.compile(pattern)
    n = 0
    if label:
        print("      %s" % label)
    for i, l in enumerate(lines, 1):
        if rx.search(l):
            print("      %s:%d  %s" % (rel(path), i, l.strip()[:width]))
            n += 1
            if n >= limit:
                break
    if n == 0:
        print("      （%s 里没有匹配 %s 的行）" % (rel(path), pattern))
    print()


def grep_count(path, pattern):
    """数一个模式在文件里命中多少【行】。

    ★ 存在的理由：正文里写死的数字会过期。
      作者在这里写过 "malloc.c 里 4 处调用 chHeapAllocAligned" ——
      而实际是 5 处（`:151` `:175` `:184` `:190` `:198`）。
      同一个数字在卷 5 第 6 章、本章正文、本章附录里
      **一共出现了三个版本**，最后是卷 4 第 14 章"先数再写"时对上的。
      ⇒ 能算的数字就别写死。
    """
    rx = re.compile(pattern)
    return sum(1 for l in read_lines(path) if rx.search(l))


# ===========================================================================
# 实验 1：core 内存 —— 两端向中间分配
# ===========================================================================

def exp1():
    head("实验 1  ★ core 内存：两端向中间分配的 arena")

    print("""
  ChibiOS 的内存最底层叫 **core memory**，整个分配器的状态**只有两个指针**。
""")
    grep_print(F_MEMCORE_H, r"basemem|topmem",
               "memcore_t 的字段（chmemcore.h）：", limit=4)

    print("""  两个分配方向、两个函数：
""")
    n1 = find_line(F_MEMCORE_C, r"^void \*chCoreAllocFromBaseI")
    if n1:
        src(F_MEMCORE_C, n1 + 6, n1 + 16, "chCoreAllocFromBaseI()（向上）：")
    n2 = find_line(F_MEMCORE_C, r"^void \*chCoreAllocFromTopI")
    if n2:
        src(F_MEMCORE_C, n2 + 6, n2 + 16, "chCoreAllocFromTopI()（向下）：")

    print("""
  ★★ 注意两边都有的这个判断：

        if ((next > topmem) || (next < basemem))  return NULL;

      第二个子句（`next < basemem`）挡的不是"不够"，而是
      **指针回绕**（`p + size` 溢出成小地址）—— 见 §1.2。

  ★ 还要注意 `chCoreGetStatusX()`：**"还剩多少"就是两个指针之间的距离**。

  ★★ 这个分配器【没有 free】—— 文档第一句就写了（chmemcore.c:27-29）：

        "a simplified allocator that only allows to allocate memory blocks
         without the possibility to free them"
""")

    hr("-")
    print("""
  【模拟】一个 32 页的 arena，core 分配器从两端吃：
      · "系统侧"用 chCoreAllocFromBase 向上申请
      · "应用侧"用 chCoreAllocFromTop  向下申请
  两边相向而行，看看什么时候撞上、以及"还剩多少"怎么算。
""")
    hr("-")
    print()

    PAGES = 32

    class MemCore:
        def __init__(self, pages):
            self.base = 0                 # 页号，向上增长
            self.top = pages              # 页号，向下增长
            self.total = pages

        def alloc_from_base(self, n):
            next_ = self.base + n
            # ★ 照抄 chmemcore.c:122 的两个子句
            if next_ > self.top or next_ < self.base:
                return None
            p = self.base
            self.base = next_
            return (p, n)

        def alloc_from_top(self, n):
            prev = self.top - n
            if prev < self.base or prev > self.top:
                return None
            self.top = prev
            return (prev, n)

        def free_pages(self):
            return self.top - self.base      # chCoreGetStatusX()

    mc = MemCore(PAGES)
    print("      初始：base=0  top=%d  剩余 %d 页" % (mc.top, mc.free_pages()))
    print()
    seq = [("base", 4), ("top", 2), ("base", 6), ("top", 3), ("base", 20)]
    for side, n in seq:
        r = mc.alloc_from_base(n) if side == "base" else mc.alloc_from_top(n)
        if r is None:
            print("      %-4s 申请 %2d 页 ⇒ ★ NULL（core 耗尽）"
                  "      剩余 %d 页" % (side, n, mc.free_pages()))
        else:
            print("      %-4s 申请 %2d 页 ⇒ 拿到页 %2d~%2d  base=%2d top=%2d 剩余 %2d 页"
                  % (side, n, r[0], r[0] + r[1] - 1, mc.base, mc.top, mc.free_pages()))

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：
       · 整个 arena 的状态就是 `base` / `top` 两个数 —— 没有任何元数据
       · "还剩多少"就是两者之差（`chCoreGetStatusX` 就一行减法）
       · 两个方向的使用者**互不干扰，直到撞上** —— 这是"给不同用途
         预留不同生长方向"的经典手法（和栈、堆相向生长是一个道理）
       · ★ **没有 free** —— 所以 core 只适合"启动阶段一次性分配"

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 对齐（真实代码里要 `MEM_ALIGN_NEXT`，会多消耗几个字节）
    · 页的单位（这里 1 页 = 抽象单位，真实是字节）
  模拟保住了什么：
    · 两个方向各自的**越界判据**（照抄，含回绕那个子句）
    · "剩余 = top - base"这个定义
""")


# ===========================================================================
# 实验 2：堆的 first-fit + 合并
# ===========================================================================

def exp2():
    head("实验 2  ★★ 堆的 first-fit + 合并")

    print("""
  ChibiOS 的堆（chmemheaps.c）用的算法是：

      **first-fit（首次适配）+ 释放时双向合并**

  卷 4 第 10 章讲过 FreeRTOS 的 `heap_4` —— **是同一个算法**。
  `heap_2` 也是 first-fit，但**不合并**。所以这个实验顺便
  把"合并到底值多少钱"量出来。
""")

    n = find_line(F_MEMHEAPS_C, r"^void \*chHeapAllocAligned")
    if n:
        src(F_MEMHEAPS_C, n + 24, n + 34, "first-fit 的循环头（chmemheaps.c）：")
    n2 = find_line(F_MEMHEAPS_C, r"Merge with the next block")
    if n2:
        src(F_MEMHEAPS_C, n2 - 4, n2 + 10, "★ 释放时的双向合并：")
    print("""
  ⇒ `chHeapFree` 里连着两个 if：**先与后一块合并，再与前一块合并**
    （chmemheaps.c:321-330）。这正是 `heap_4` 做的事。
""")

    hr("-")
    print("""
  【模拟】两个分配器跑同一组序列：

       A. first-fit + 合并   （= ChibiOS chmemheaps / FreeRTOS heap_4）
       B. first-fit 不合并   （= FreeRTOS heap_2）

  分配序列故意做成"交错释放"—— 那是外碎片最容易暴露的形态。
""")
    hr("-")
    print()

    PAGE = 8          # 抽象：1 页 = 8 字节

    class Block:
        def __init__(self, start, pages, free=True):
            self.start, self.pages, self.free = start, pages, free
        def __repr__(self):
            return "%s@%d(%dp)" % ("free" if self.free else "used",
                                   self.start, self.pages)

    class Heap:
        def __init__(self, pages, merge):
            self.merge = merge
            self.blocks = [Block(0, pages, True)]

        def alloc(self, pages, name=""):
            for b in self.blocks:
                if b.free and b.pages >= pages:
                    if b.pages == pages:
                        b.free = False
                        b.name = name
                    else:
                        nb = Block(b.start + pages, b.pages - pages, True)
                        b.pages, b.free, b.name = pages, False, name
                        self.blocks.insert(self.blocks.index(b) + 1, nb)
                    return b
            return None

        def free(self, blk):
            blk.free = True
            blk.name = ""
            if self.merge:
                self._merge()

        def _merge(self):
            """照抄 chHeapFree 的两步：与后合并、再与前合并。"""
            i = 0
            while i < len(self.blocks) - 1:
                a, b = self.blocks[i], self.blocks[i + 1]
                if a.free and b.free and a.start + a.pages == b.start:
                    a.pages += b.pages          # H_PAGES(hp) += H_PAGES(next) + 1
                    del self.blocks[i + 1]
                    i = max(0, i - 1)
                else:
                    i += 1

        def status(self):
            frees = [b.pages for b in self.blocks if b.free]
            return len(frees), sum(frees), max(frees) if frees else 0

        def dump(self):
            return " | ".join(
                ("[%d:%dp]" % (b.start, b.pages)) + ("" if b.free else "*")
                for b in self.blocks)

    def run(merge):
        """★ 堆只有 12 页（= 3 × 4），这样"合并与否"才有区别：
           合并版能把 [7:1] 和 [8:4] 拼成 [7:5]，不合并版最大只有 4。"""
        h = Heap(12, merge)
        a = h.alloc(4, "A")
        b = h.alloc(4, "B")
        c = h.alloc(4, "C")
        h.free(b)                       # ★ 中间那块先释放（制造夹心）
        d = h.alloc(3, "D")             # 从 B 那块里切 3 页，留下 1 页碎片
        h.free(a)
        h.free(c)
        # ★ 在申请 E【之前】取一次状态 —— 这一时刻两边的"总空闲"是一样的，
        #   差别只在"最大连续"，那才是能否分配成功的判据。
        before = h.status()
        e = h.alloc(5, "E")             # 需要 5 页：合并了就有，没合并就没有
        return h, e, before

    print("      序列：分配 A(4) B(4) C(4) → 释放 B → 分配 D(3) → 释放 A、C → 分配 E(5)")
    print("      （堆总共 12 页 —— 故意做小，好让「合并」这一步变得关键）")
    print()
    for merge, label in ((True, "A. first-fit + 合并（ChibiOS / heap_4）"),
                         (False, "B. first-fit 不合并（heap_2）")):
        h, e, before = run(merge)
        print("      %s" % label)
        print("         %s" % h.dump())
        print("         申请 E(5) 之前：碎片 %d 块 / 总空闲 %d 页 / 最大连续 %d 页"
              % before)
        print("         分配 E(5) ⇒ %s" % ("成功" if e else "★ 失败（NULL）"))
        print("         分配之后：    %s" % h.dump())
        print()

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：两边在申请 E(5) 之前的**总空闲都是 9 页** ——
     但一边的最大连续是 **5**（成功），另一边是 **4**（失败）。

     合并版能把"释放出来的相邻块"拼回大块，不合并版永远拼不回去。

     ★ 所以 `chHeapStatus()` 返回的是**三个**量而不是一个：

          碎片数 / 总空闲 / **最大连续空闲**

       前两个一样不代表能分配得一样 —— **第三个才是能否成功的判据**。
       （卷 4 第 10 章讲 `heap_2` 时，那个"陷阱"就藏在这里。）

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实的对齐开销（每块多一个 header，合并时还要 +1 页）
    · 分配失败后向 provider 要内存那条路（§2.3）
  模拟保住了什么：
    · first-fit 的**查找顺序**（从头扫，第一个够大的就用）
    · 释放时的**双向合并**（照抄 chmemheaps.c:321-330 的两步）
""")


# ===========================================================================
# 实验 3：池 vs 堆
# ===========================================================================

def exp3():
    head("实验 3  ★★ 池 vs 堆：同一组分配序列，碎片差多少")

    print("""
  内存池（chmempools.c）和堆解决的是**两个不同的问题**：

      堆  ：**任意大小**，代价是 O(n) 查找 + 外碎片
      池  ：**固定大小**，代价是浪费（每个对象都按最大者算）
""")

    n = find_line(F_MEMPOOLS_C, r"^void \*chPoolAllocI")
    if n:
        src(F_MEMPOOLS_C, n + 4, n + 17, "chPoolAllocI() —— 出池就是摘链表头：")
    n2 = find_line(F_MEMPOOLS_C, r"^void chPoolFreeI")
    if n2:
        src(F_MEMPOOLS_C, n2 + 4, n2 + 10, "★ 入池：把对象**自己**当链表节点用：")
    print("""
  ⇒ `php->next = mp->next;` —— `php` 就是刚释放的那个对象。
    **空闲对象的存储被复用了**，所以池的每个对象**零额外开销**
    （堆每块要一个 header，池不要）。

  ⇒ 代价也在这里：**取出来的对象里没有"大小"信息**，
    因为那个字段被 `next` 占了。所以池 API 里没有 `chPoolGetSize()`，
    而堆有 `chHeapGetSize()`（读 `used.size`）。
""")

    hr("-")
    print("""
  【模拟】同一组"请求大小不一"的序列跑两遍：

       A. 固定 32 字节的池
       B. 堆（实验 2 的合并版）

  看两件事：**能不能分配**、**浪费多少**。
""")
    hr("-")
    print()

    OBJ = 32                      # 池的对象大小
    POOL_OBJS = 8

    class Pool:
        def __init__(self, obj_size, n):
            self.obj_size = obj_size
            self.free = list(range(n))       # 空闲链表（用编号代替指针）
            self.used = []
        def alloc(self, size):
            if size > self.obj_size:
                return None                  # ★ 池不能给比对象大的
            if not self.free:
                return None
            i = self.free.pop(0)             # ★ O(1)
            self.used.append(i)
            return i
        def free(self, i):
            self.free.insert(0, i)           # ★ O(1)
        def waste(self):
            return len(self.used) * self.obj_size

    REQS = [8, 32, 16, 24, 32, 8, 16, 32]

    p = Pool(OBJ, POOL_OBJS)
    heap_pages = 0
    heap_cap = 24 * 8                     # 和实验 2 同一个 24 页 = 192 字节
    heap_used = 0

    print("      池：对象 %d 字节 × %d 个 = %d 字节" % (OBJ, POOL_OBJS, OBJ * POOL_OBJS))
    print("      堆：%d 字节" % heap_cap)
    print()
    print("       %-6s | %-28s | %s" % ("请求", "池", "堆（合并版）"))
    print("      " + "-" * 62)
    pool_ok = heap_ok = 0
    for size in REQS:
        pi = p.alloc(size)
        # 堆：按 8 字节对齐后记账
        need = (size + 7) // 8 * 8
        if heap_used + need <= heap_cap:
            heap_used += need
            hres, heap_ok = "成功（占 %d）" % need, heap_ok + 1
        else:
            hres = "★ 失败"
        if pi is not None:
            pool_ok += 1
            pres = "成功（第 %d 号对象）" % pi
        else:
            pres = "★ 失败"
        print("      %-6d | %-28s | %s" % (size, pres, hres))
    print()
    print("      池：成功 %d / %d，实际占用 %d 字节（每个都按 %d 算）"
          % (pool_ok, len(REQS), p.waste(), OBJ))
    print("      堆：成功 %d / %d，实际占用 %d 字节（按需）"
          % (heap_ok, len(REQS), heap_used))
    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：这就是"池 vs 堆"的全部取舍 ——

      |          | 池 | 堆 |
      |---|---|---|
      | 查找 | ★ **O(1)**（摘链表头） | O(n)（first-fit 扫） |
      | 每对象开销 | ★ **0**（复用对象自己的存储） | 一个 header |
      | 外碎片 | ★ **不存在** | 有（靠合并缓解） |
      | 大小 | **固定**（超了就给不了） | 任意 |
      | 能问大小吗 | ❌ 不能（字段被 next 占了） | ✅ `chHeapGetSize` |

  ★ 所以选型问题不是"哪个更好"，而是：
    **"我要分配的对象是不是同一种大小？"** 是 ⇒ 池；不是 ⇒ 堆。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实的 freelist 指针操作（这里用编号代表对象）
    · 对齐（池也要对齐，见 `chPoolObjectInitAligned` 的 align 参数）
  模拟保住了什么：
    · 池的 **O(1) 出/入 + 零元数据**（结构上就决定了）
    · 池**给不了超过对象大小的请求**这条硬约束
""")


# ===========================================================================
# 实验 4：union 的读与写顺序
# ===========================================================================

def exp4():
    head("实验 4  ★★ heap_header 的 union：读与写的顺序纪律")

    print("""
  堆里每个块都有一个 header。ChibiOS 把它做成**一个 union**：

      union heap_header {
        struct { heap_header_t *next;  size_t pages; } free;   /* 空闲时用 */
        struct { memory_heap_t *heap;  size_t     size; } used; /* 占用时用 */
      };
""")
    n = find_line(F_MEMHEAPS_H, r"^union heap_header")
    if n:
        src(F_MEMHEAPS_H, n, n + 12, "chmemheaps.h ——")

    print("""  ★★ **同一块 8 字节，空闲时是链表节点，占用时是记账信息。**

    这带来一个必须小心的地方 —— 看 `chHeapFree` 的开头（chmemheaps.c:300-307）：

        hp = (heap_header_t *)p - 1U;
        heapp = H_HEAP(hp);              /* ← ① 先读 used.heap */
        qp = &heapp->header;
        H_PAGES(hp) = MEM_ALIGN_NEXT(H_SIZE(hp), CH_HEAP_ALIGNMENT)
                      / CH_HEAP_ALIGNMENT;  /* ← ② 再把 union 改写成 free 视图 */

    `H_HEAP(hp)` 和 `H_SIZE(hp)` 是 `used` 视图，
    `H_PAGES(hp)` 和 `H_NEXT(hp)` 是 `free` 视图 —— **两组字段重叠**。
""")

    hr("-")
    print("""
  【模拟】用两个 dict 表示那个 union 的两种视图，跑两种顺序：

       正确：先读 heap/size，再改成 free 视图
       错误：先改成 free 视图，再读 heap/size
""")
    hr("-")
    print()

    def heap_free(order_correct):
        # union 的字节视图：偏移 0 和偏移 4
        hdr = {"off0": "H", "off4": 64}      # used 视图：heap 指针 / size=64
        log = []
        if order_correct:
            heap = hdr["off0"]               # ① 读 used.heap
            size = hdr["off4"]               #    读 used.size
            log.append("读 heap=%s, size=%d" % (heap, size))
            hdr["off4"] = size // 8          # ② 写 free.pages（覆盖 size 那格）
            hdr["off0"] = "next"             #    写 free.next（覆盖 heap 那格）
            log.append("改写为 free 视图：next=%s, pages=%d"
                       % (hdr["off0"], hdr["off4"]))
            return heap, log
        else:
            hdr["off4"] = hdr["off4"] // 8   # 先写 free.pages ⇒ 覆盖了 size
            hdr["off0"] = "next"             # 再写 free.next ⇒ 覆盖了 heap
            log.append("先改写为 free 视图：next=%s, pages=%d"
                       % (hdr["off0"], hdr["off4"]))
            heap = hdr["off0"]               # 现在读 heap ⇒ 读到的是 "next"！
            size = hdr["off4"]               # 现在读 size ⇒ 读到的是 pages！
            log.append("再读 heap=%s, size=%d" % (heap, size))
            return heap, log

    for ok in (True, False):
        heap, log = heap_free(ok)
        print("      %s" % ("正确顺序" if ok else "错误顺序（先改写）"))
        for l in log:
            print("         %s" % l)
        print("         ⇒ 归还到的堆 = %s  %s"
              % (heap, "" if heap == "H" else "★ 堆指针丢了！"))
        print()

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：这个 union 能成立，靠的是一条**读与写的顺序纪律**：

       ① `H_HEAP(hp)` 必须在 `H_PAGES(hp)` 写入【之前】读
       ② `H_SIZE(hp)` 必须在同一表达式里先求值（C 保证右手边先算）

     ★ 这不是"union 不安全"，而是 **union 的代价**：
       它省了一半的 RAM（4 个字段挤进 2 个字），
       换来的是"改写视图之后，另一套字段全没了"。

     ★★ 而且这两个视图**永远不会同时需要** —— 这正是能省下来的原因：
        空闲时不需要知道"属于哪个堆、多大"，占用时不需要 next。

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 真实的字节布局（这里只用两个格子表示两个字的偏移）
    · 编译器对内联函数 `H_PAGES` / `H_HEAP` 的展开
  模拟保住了什么：
    · **两套视图共用同一块存储**这个结构事实
    · "读晚了就读到已改写的数据"这个后果
""")


# ===========================================================================
# 实验 5：ArduPilot 的多区路由
# ===========================================================================

def exp5():
    head("实验 5  ★★ ArduPilot 的多区路由（malloc.c 的回退链）")

    # ★ 数字由脚本数出来，不写死 —— 见 grep_count() 的说明。
    n_alloc = grep_count(F_MALLOC_C, r"chHeapAllocAligned")
    print("""
  ★ 先纠正一个容易想当然的说法：**"ArduPilot 不用 ChibiOS 的堆"是错的。**

  实测：`malloc.c` 里 %d 处调用 `chHeapAllocAligned`，
  只是**在它外面包了一层路由**。
""" % n_alloc)
    grep_print(F_MALLOC_C, r"chHeapAllocAligned|chHeapObjectInit|memory_heap_t heaps",
               "malloc.c 里的关键行：", limit=8)

    print("""  **为什么能包这一层？** 因为 ChibiOS 的堆有两个 FreeRTOS 没有的能力：

      ① **可以有很多个堆**（`heaps[NUM_MEMORY_REGIONS]`，每个自己一块内存）
      ② **分配时可以要求对齐**（`chHeapAllocAligned(heapp, size, align)`）

    FreeRTOS 的 `heap_4` 两个都没有：一个全局数组、对齐是编译期固定的。
""")
    n = find_line(F_MALLOC_C, r"static const struct memory_region memory_regions")
    if n:
        src(F_MALLOC_C, n - 1, n + 2,
            "内存区域表来自 hwdef（生成器写进 HAL_MEMORY_REGIONS）：")
    grep_print(F_HWDEF_PY, r"HAL_MEMORY_REGIONS",
               "生成它的那一行：", limit=2)
    print("""  而切到内核层看，**堆描述符里就带着"上游提供者"**（chmemheaps.h）：

        struct memory_heap {
          memgetfunc2_t  provider;   /* ← 堆不够时可以向上游要 */
          heap_header_t  header;
          mutex_t        mtx;        /* ← 每个堆自己的锁 */
        };

  ⇒ 一个堆 = **一块内存 + 一个 provider + 一把锁**。
    所以"多堆"不需要内核做任何新事 —— 多开几个 `memory_heap_t` 就行。
""")

    hr("-")
    print("""
  【模拟】照抄 malloc_flags() 的**回退链**（malloc.c:148-198）：

       ① flags == 0 或（要 DMA 且默认堆就是 DMA 安全的）⇒ 先试默认堆
       ② 按 flags 逐个试匹配的区域堆
       ③ 不是 DMA 请求 ⇒ 退而求其次，**任意**区域堆都能用
       ④ 还不行 ⇒ 用 DMA 保留池
       ⑤ 都不行 ⇒ NULL

  场景：三个区域（一块 DMA 安全 / 一块快 / 一块普通），
  故意让前两个很早就满，看请求怎么被"逐级下沉"。
""")
    hr("-")
    print()

    DMA_OK, NORMAL, FAST = 1, 0, 2

    class HeapSim:
        def __init__(self, name, free, flags):
            self.name, self.free, self.flags = name, free, flags
        def alloc(self, size, align=8):
            if self.free >= size:
                self.free -= size
                return "%s+%d" % (self.name, size)
            return None

    def malloc_sim(heaps, default_heap, dma_reserve, size, flags):
        """照抄 malloc.c:130-205 的结构（去掉对齐细节）。"""
        if size == 0:
            return None
        if flags == 0 or (flags & DMA_OK and default_heap.flags & DMA_OK):
            p = default_heap.alloc(size)
            if p:
                return p, "① 默认堆"
        for h in heaps:
            if (flags & DMA_OK) and not (h.flags & DMA_OK):
                continue
            if (flags & FAST) and not (h.flags & FAST):
                continue
            p = h.alloc(size)
            if p:
                return p, "② 匹配 flags 的区域堆（%s）" % h.name
        if not (flags & DMA_OK):
            for h in heaps:
                p = h.alloc(size)
                if p:
                    return p, "③ 回退：任意区域堆（%s）" % h.name
            p = default_heap.alloc(size)
            if p:
                return p, "③ 回退：默认堆"
        if dma_reserve.free >= size:
            dma_reserve.free -= size
            return "dma_reserve+%d" % size, "④ DMA 保留池"
        return None, "⑤ NULL"

    # ★ 场景要设计成"五级各至少走一次"，否则演示是退化的。
    heaps = [HeapSim("region1", 24, DMA_OK | FAST),
             HeapSim("region2", 16, NORMAL)]
    default_heap = HeapSim("默认堆", 16, DMA_OK)
    dma_reserve = HeapSim("dma_reserve", 32, DMA_OK)

    print("      初始状态：")
    print("        默认堆      16 B    flags = DMA 安全")
    print("        region1     24 B    flags = DMA 安全 + FAST")
    print("        region2     16 B    flags = 普通")
    print("        dma_reserve 32 B    （平时不动的最后手段）")
    print()
    print("      %-22s %-7s %-30s %s"
          % ("请求", "结果", "走了哪一级", "剩余（默认/区1/区2/保留）"))
    print("      " + "-" * 84)
    for size, flags in ((8, 0), (16, DMA_OK), (16, FAST), (16, 0),
                        (32, 0), (16, 0), (24, 0)):
        name = "size=%-3d %s" % (size, {0: "(无 flags)", DMA_OK: "(要 DMA)",
                                        FAST: "(要 FAST)"}[flags])
        p, how = malloc_sim(heaps, default_heap, dma_reserve, size, flags)
        print("      %-22s %-7s %-30s %d/%d/%d/%d"
              % (name, "成功" if p else "★ NULL", how,
                 default_heap.free, heaps[0].free, heaps[1].free, dma_reserve.free))

    print("""
      ★ 注意第 4、6 行：**非 DMA 的请求也会退到 DMA 保留池**。
        看 malloc.c 的结构 —— 那一级写在 `if (!(flags & dma_flags))` 块
        【外面】（malloc.c:194-201），所以它对所有请求都是最后手段。
        名字里的"保留"说的是"平时不动"，不是"只给 DMA 用"。

      ★ 再看"剩余"那一列：默认堆和两个区域堆一直卡在 8/8/0 没动过 ——
        因为剩下的那点零头谁也满足不了。**这正是"外部碎片的死法"**：
        总空闲 16 字节，却一个 24 字节的请求都接不了。""")

    print("""
  ----------------------------------------------------------------------
  ⇒ **结论**：
       · 这一层路由做的事，内核一个都做不了 —— 它靠的是
         **多堆**（内核支持）+ **内存区域表**（hwdef 生成）+ **flags 语义**
       · ★ 顺序里最值得看的是第 ③ 级：**非 DMA 的请求可以"将就"**，
         DMA 的请求不行 —— 因为 DMA 要的是一块**物理上**满足条件的
         内存，把普通内存给它就是"能分配但会出事"，
         比分配失败糟得多
       · ★ 第 ④ 级（DMA 保留池）是"最后 6 KB 救命内存"：
         宁可留一块平时不用的，也不要系统在关键时刻彻底分配不出

  ----------------------------------------------------------------------
  模拟丢掉了什么：
    · 对齐（真实代码里 DMA 要 32 字节对齐，H7 上尤其严）
    · `chHeapAllocAligned` 内部的 first-fit 扫描（实验 2 已经模拟过）
  模拟保住了什么：
    · 回退链的**顺序和前置条件**（照抄 malloc.c:148-198 的分支结构）
""")


# ===========================================================================
# 入口
# ===========================================================================

EXPERIMENTS = {
    "1": ("★ core 内存的两端分配", exp1),
    "2": ("★★ 堆的 first-fit + 合并", exp2),
    "3": ("★★ 池 vs 堆", exp3),
    "4": ("★★ union 的读写顺序", exp4),
    "5": ("★★ ArduPilot 的多区路由", exp5),
}


def main():
    print()
    hr("#")
    print("#  本章：内存池与 oslib")
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
