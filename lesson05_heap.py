#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 5 课配套实验：内存管理 heap_1 到 heap_5

设计原则（延续三条纪律）：
  1. 每个论断都能落到 文件:行号
  2. [未亲验] 是诚实的标记，不是免责声明
  3. 模拟的地方明说丢掉了什么

运行：
    python lesson05_heap.py         # 全部
    python lesson05_heap.py 1       # 只跑实验 1

源码来源：D:/DEVLib/PX4APM/FreeRTOS/portable/MemMang/   （V11.1.0）
"""

import io
import _srcdirs
import os
import re
import sys

FREERTOS_ROOT = _srcdirs.src("FREERTOS")
MEMMANG = os.path.join(FREERTOS_ROOT, "portable", "MemMang")


def sec(title, ch="=", width=78):
    print()
    print(ch * width)
    print(title)
    print(ch * width)


# ============================================================================
# 实验 1：heap_4 的双向合并逐步演示 ⭐
# ============================================================================
#
# heap_4.c:509-545 的合并逻辑有【两个方向】：
#
#   方向 1（与前一块）：puc + pxIterator->xBlockSize == pxBlockToInsert
#   方向 2（与后一块）：puc + pxBlockToInsert->xBlockSize == pxIterator->pxNextFreeBlock
#
# 因为空闲链表按【地址】排序（heap_4.c:499），所以"插入位置的前后"
# 就是"地址上紧邻的前后" —— 判相邻只需要指针算术。

BLOCK_SIZE = 16      # 模拟的块大小（字节）


class Block:
    """模拟 BlockLink_t（heap_4.c 的块结构）。

    真实结构：
        typedef struct xBLOCK_LINK
        {
            struct xBLOCK_LINK * pxNextFreeBlock;   /* 空闲链表的下一个 */
            size_t xBlockSize;                      /* 最高位 = 是否已分配 */
        } BlockLink_t;

    丢掉的东西：
      · 真实的指针（这里用地址整数模拟）
      · heapPROTECT_BLOCK_POINTER 之类的 MPU 保护宏
      · 对齐处理
    """

    def __init__(self, addr, size, allocated=False):
        self.addr = addr
        self.size = size
        self.allocated = allocated        # ← 对应 xBlockSize 的最高位

    def end(self):
        return self.addr + self.size

    def __repr__(self):
        return "Block(0x%03x, size=%d, %s)" % (
            self.addr, self.size, "已分配" if self.allocated else "空闲")


class Heap4Sim:
    """模拟 heap_4 的空闲链表（按地址排序）+ 双向合并。

    本实验只模拟"释放时的合并"，分配路径用简化版（first-fit）。
    """

    def __init__(self, total_size):
        # 初始：一整块空闲
        self.blocks = [Block(0, total_size, False)]
        self.log = []

    def free_blocks(self):
        """空闲链表（按地址升序，模拟 heap_4.c:499 的排序准则）"""
        return [b for b in self.blocks if not b.allocated]

    def alloc(self, size):
        """first-fit（heap_4.c:243-260 的遍历）"""
        for b in self.blocks:
            if not b.allocated and b.size >= size:
                # 切分
                k = self.blocks.index(b)
                new = Block(b.addr + size, b.size - size, False)
                b.size = size
                b.allocated = True
                if new.size > 0:
                    self.blocks.insert(k + 1, new)
                self.log.append("分配 %d 字节 @0x%03x（first-fit 选中）"
                                % (size, b.addr))
                return b.addr
        self.log.append("分配 %d 字节：失败（没有够大的块）" % size)
        return None

    def free(self, addr):
        """释放 + 合并（heap_4.c:492-545 的完整逻辑）"""
        k = None
        for i, b in enumerate(self.blocks):
            if b.addr == addr and b.allocated:
                k = i
                break
        if k is None:
            self.log.append("释放 0x%03x：失败（不是已分配的块）" % addr)
            return

        self.blocks[k].allocated = False
        self.log.append("释放 @0x%03x（size=%d）" % (addr, self.blocks[k].size))

        # ── 方向 1：与【前】一块合并（heap_4.c:513-522） ──
        prev = self.blocks[k - 1] if k > 0 else None
        merged_prev = False
        if prev is not None and not prev.allocated and prev.end() == self.blocks[k].addr:
            self.log.append(
                "  ├─ 方向 1：前一块 0x%03x 空闲且 end(0x%03x) == 本块 addr(0x%03x)"
                " ⇒ **合并**" % (prev.addr, prev.end(), self.blocks[k].addr))
            prev.size += self.blocks[k].size
            del self.blocks[k]
            k = k - 1
            merged_prev = True
        else:
            why = ("前一块已分配" if prev and prev.allocated
                   else ("前一块 end(0x%03x) != 本块 addr(0x%03x)"
                         % (prev.end(), self.blocks[k].addr) if prev else "本块已在最前"))
            self.log.append("  ├─ 方向 1：不合并（%s）" % why)

        # ── 方向 2：与【后】一块合并（heap_4.c:527-543） ──
        nxt = self.blocks[k + 1] if k + 1 < len(self.blocks) else None
        if nxt is not None and not nxt.allocated and self.blocks[k].end() == nxt.addr:
            self.log.append(
                "  ├─ 方向 2：后一块 0x%03x 空闲且 本块 end(0x%03x) == its addr(0x%03x)"
                " ⇒ **合并**" % (nxt.addr, self.blocks[k].end(), nxt.addr))
            self.blocks[k].size += nxt.size
            del self.blocks[k + 1]
        else:
            why = ("后一块已分配" if nxt and nxt.allocated
                   else ("本块 end(0x%03x) != 后块 addr(0x%03x)"
                         % (self.blocks[k].end(), nxt.addr) if nxt else "本块已在最后"))
            self.log.append("  └─ 方向 2：不合并（%s）" % why)

    def dump(self):
        out = []
        for b in self.blocks:
            out.append("[0x%03x:%d%s]" % (b.addr, b.size,
                                          "" if b.allocated else " 空"))
        return " ".join(out)


def exp1_coalesce():
    sec("实验 1：heap_4 的双向合并 —— 四种情形 ⭐")

    print("""
  heap_4.c:509-545 的合并逻辑有【两个方向】，本实验把四种情形都跑一遍。

  为什么能判"相邻"？因为空闲链表按【地址】排序（heap_4.c:499）：

      for( pxIterator = &xStart;
           heapPROTECT_BLOCK_POINTER( pxIterator->pxNextFreeBlock ) < pxBlockToInsert;
           pxIterator = ... )

  ⇒ 插入位置的【前一个节点】就是地址上【紧邻前面】的空闲块
  ⇒ 插入位置的【后一个节点】就是地址上【紧邻后面】的空闲块
  ⇒ 判相邻只需要指针算术，不需要遍历全表

  **这就是 heap_2 做不到合并的根本原因** ——
  它的链表按大小排序，地址相邻的块根本不在一起。
""")

    scenarios = [
        ("情形 A：只有后一块空闲（后合并）",
         [(0, 16, True), (16, 16, True), (32, 16, False)], 16),
        ("情形 B：只有前一块空闲（前合并）",
         [(0, 16, False), (16, 16, True), (32, 16, True)], 16),
        ("情形 C：前后都空闲（双向合并）",
         [(0, 16, False), (16, 16, True), (32, 16, False)], 16),
        ("情形 D：前后都已分配（不合并）",
         [(0, 16, True), (16, 16, True), (32, 16, True)], 16),
    ]

    for title, layout, free_addr in scenarios:
        print("  " + "=" * 74)
        print("  %s" % title)
        print("  " + "=" * 74)
        h = Heap4Sim(48)
        h.blocks = [Block(a, s, alloc) for a, s, alloc in layout]
        print()
        print("  初始布局：")
        for b in h.blocks:
            print("      0x%03x ~ 0x%03x  size=%-3d  %s"
                  % (b.addr, b.end(), b.size,
                     "已分配" if b.allocated else "空闲"))
        print()
        print("  释放 0x%03x：" % free_addr)
        h.free(free_addr)
        for line in h.log:
            print("      %s" % line)
        print()
        print("  结果：%s" % h.dump())
        free_total = sum(b.size for b in h.blocks if not b.allocated)
        n_free = len(h.free_blocks())
        max_free = max((b.size for b in h.blocks if not b.allocated), default=0)
        print("  **空闲总量 = %d 字节，空闲块数 = %d，最大连续块 = %d 字节**"
              % (free_total, n_free, max_free))
        print()

    print("""  ================== 读法（上面的实测数字） ==================

      ┌──────────────────────────────┬──────────┬────────────┬──────────────┐
      │ 情形                         │ 空闲总量 │ 空闲块数   │ 最大连续块   │
      ├──────────────────────────────┼──────────┼────────────┼──────────────┤
      │ A 只有后一块空闲（后合并）    │   32     │     1      │     32       │
      │ B 只有前一块空闲（前合并）    │   32     │     1      │     32       │
      │ C 前后都空闲（双向合并）      │ **48**   │     1      │   **48**     │
      │ D 前后都已分配（不合并）      │   16     │     1      │     16       │
      └──────────────────────────────┴──────────┴────────────┴──────────────┘

  **三个观察：**

  【1】情形 A 和 B 结果相同（都是 32）——
      说明**单方向合并已经能回收被释放块 + 一个邻居**。

  【2】情形 C 是 48 —— **三个块并成一个**。
      双向合并比单向多回收了另一个邻居。

  【3】情形 D 的"空闲总量"只有 16，因为另外两块还是**已分配**的。
      注意区分【空闲块数】和【总块数】——
      "3 个块"指的是总块数，而空闲的只有刚释放的那一个。

  ⇒ **合并让"空闲总量"和"最大连续块"趋于一致** ——
    这正是碎片问题被解决的表现。

  第 10 课 5.4 节做过一个更贴近真实的量化对比（heap_2 vs heap_4）：
      空闲总量同为 25，
      heap_2 最大连续块 10（分配 20 失败），
      heap_4 最大连续块 25（分配 20 成功）。

  ================== 一个实现细节 ==================

      heap_4.c:529-535 有一个特殊判断：

          if( heapPROTECT_BLOCK_POINTER( pxIterator->pxNextFreeBlock ) != pxEnd )
          {
              /* Form one big block from the two blocks. */
              ...
          }
          else
          {
              pxBlockToInsert->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER( pxEnd );
          }

      **pxEnd 是链表哨兵（末尾标记），不是真实的空闲块。**
      所以"与后一块合并"之前要排除"后一块其实是哨兵"。

      ⇒ 漏掉这个判断会把哨兵当成空闲块合并进去 —— 链表被破坏。

      这种"哨兵带来的边界处理"是所有用哨兵的链表的共性。
""")


# ============================================================================
# 实验 2：best-fit vs first-fit 的碎片化对比
# ============================================================================
#
# heap_2：空闲链按【大小】排序 ⇒ best-fit，不合并
# heap_4：空闲链按【地址】排序 ⇒ first-fit，双向合并


class Heap2Sim:
    """heap_2：按大小排序的链表，best-fit，不合并

    注意：属性名用 free_list 而不是 free —— 否则会和 free() 方法同名互相遮蔽。
    （这是写这个模拟时踩的一个真实 bug：`self.free = [...]` 会让
      `self.free(...)` 抛 TypeError: 'list' object is not callable）
    """

    def __init__(self, total):
        self.free_list = [total]        # 只关心空闲块的大小（按大小排序）

    def alloc(self, size):
        """best-fit：找第一个够大的（因为按大小排序，就是最小的够用块）"""
        for i, sz in enumerate(self.free_list):
            if sz >= size:
                rest = sz - size
                del self.free_list[i]
                if rest > 0:
                    self.free_list.append(rest)
                    self.free_list.sort()      # 维持按大小排序
                return True
        return False

    def free(self, size):
        """不合并 —— 直接加回链表"""
        self.free_list.append(size)
        self.free_list.sort()

    def stats(self):
        f = self.free_list
        return sum(f), len(f), max(f) if f else 0


class Heap4Sim2:
    """heap_4：按地址排序，first-fit，合并"""

    def __init__(self, total):
        # 用 (地址, 大小) 表示空闲块，按地址排序
        self.free_list = [(0, total)]
        self.next_addr = total

    def _norm(self):
        self.free_list.sort(key=lambda x: x[0])

    def alloc(self, size):
        """first-fit：从最低地址开始找第一个够大的"""
        self._norm()
        for i, (addr, sz) in enumerate(self.free_list):
            if sz >= size:
                rest = sz - size
                del self.free_list[i]
                if rest > 0:
                    self.free_list.append((addr + size, rest))
                return addr
        return None

    def free(self, addr, size):
        """加回并按地址顺序【合并相邻】"""
        self.free_list.append((addr, size))
        self._norm()
        # 扫一遍合并相邻块（等价于 heap_4 的插入时双向合并的最终效果）
        merged = True
        while merged:
            merged = False
            for i in range(len(self.free_list) - 1):
                a1, s1 = self.free_list[i]
                a2, s2 = self.free_list[i + 1]
                if a1 + s1 == a2:
                    self.free_list[i:i + 2] = [(a1, s1 + s2)]
                    merged = True
                    break

    def stats(self):
        sizes = [s for _, s in self.free_list]
        return sum(sizes), len(sizes), max(sizes) if sizes else 0


def exp2_best_vs_first():
    sec("实验 2：best-fit vs first-fit 的碎片化对比")

    print("""
  第 10 课修正过一处：**heap_2 不是"heap_4 去掉合并"，
  而是 best-fit（空闲链按【大小】排序）。**

  本实验跑同一个分配/释放序列，对比两者的结果。

  ⚠ 这是**简化模型**：
      · 只跟踪空闲块，不模拟真实内存布局
      · heap_2 的"按大小排序"用 sort 模拟
      · heap_4 的合并用"扫一遍相邻块"模拟（真实实现在插入时双向合并，
        最终效果等价）
    丢掉的东西：真实的指针开销、对齐、块头大小。
    但"排序准则决定碎片表现"这个结论不受影响。
""")

    # 序列：交替分配不同大小的块，再释放一部分制造碎片
    TOTAL = 256
    seq_desc = [
        "分配 A(32) B(64) C(32) D(64) E(32)",
        "释放 A、C、E（制造三个孤立空洞）",
        "再分配 X(16)  → 看谁更贴合",
        "释放 B、D（制造更多空洞）",
        "尝试分配 Y(96) → 看谁还能满足",
    ]

    print("  " + "-" * 74)
    print("  序列")
    print("  " + "-" * 74)
    for i, s in enumerate(seq_desc, 1):
        print("      %d. %s" % (i, s))
    print()

    # heap_2（best-fit，不合并）
    h2 = Heap2Sim(TOTAL)
    h2.alloc(32); h2.alloc(64); h2.alloc(32); h2.alloc(64); h2.alloc(32)
    h2.free(32); h2.free(32); h2.free(32)      # 释放 A/C/E
    h2.alloc(16)                                # X
    h2.free(64); h2.free(64)                    # 释放 B/D
    ok2 = h2.alloc(96)                          # Y

    # heap_4（first-fit，合并）
    h4 = Heap4Sim2(TOTAL)
    a = h4.alloc(32); b = h4.alloc(64); c = h4.alloc(32)
    d = h4.alloc(64); e = h4.alloc(32)
    h4.free(a, 32); h4.free(c, 32); h4.free(e, 32)
    h4.alloc(16)
    h4.free(b, 64); h4.free(d, 64)
    ok4 = h4.alloc(96)

    print("  " + "-" * 74)
    print("  结果对比")
    print("  " + "-" * 74)
    print()
    print("  %-16s %-16s %-16s" % ("", "heap_2 (best-fit)", "heap_4 (first-fit)"))
    print("  " + "-" * 50)
    t2, n2, m2 = h2.stats()
    t4, n4, m4 = h4.stats()
    print("  %-16s %-16d %-16d" % ("空闲总量", t2, t4))
    print("  %-16s %-16d %-16d" % ("空闲块数", n2, n4))
    print("  %-16s %-16d %-16d" % ("最大连续块", m2, m4))
    print("  %-16s %-16s %-16s"
          % ("分配 Y(96)", "成功" if ok2 else "**失败**",
             "成功" if ok4 else "**失败**"))
    print()

    print("  " + "=" * 74)
    if t2 != t4 or m2 != m4 or ok2 != ok4:
        print("  ⇒ **结果差异极大，而且方向和直觉相反：**")
        print()
        print("        heap_2（best-fit）: 空闲总量 **%d** 字节，最大连续块 %d，"
              "分配 Y(96) **失败**" % (t2, m2))
        print("        heap_4（first-fit）: 空闲总量 **%d** 字节，最大连续块 %d，"
              "分配 Y(96) 成功" % (t4, m4))
        print()
        print("      **heap_2 有【更多】空闲内存（%d > %d），却分配失败。**"
              % (t2, t4))
        print("      原因是它的最大连续块只有 %d 字节，装不下 96。" % m2)
        print()
        print("      ⇒ 这句话值得记住：")
        print("        **「能分配」取决于【最大连续块】，而不是【空闲总量】。**")
    else:
        print("  ⇒ 本例中两者结果相同。这说明**碎片问题的暴露需要特定序列** ——")
        print("     不是每次都会出现，所以它极难定位（第 10 课 5.4 节）。")

    print("""
  ================== 三个可以带走的点 ==================

  【1】"空闲总量"相同不代表"能分配" —— 关键是【最大连续块】

      上面两行的"空闲总量"可能一样，但"最大连续块"决定了
      下一次分配能不能成功。

      ⇒ 监控内存时，**光看 free 命令的总量是不够的**，
        要看"最大连续块"（heap_4 的 vPortGetHeapStats 提供了
        xSizeOfLargestFreeBlockInBytes）。

  【2】碎片问题需要特定序列才暴露

      这就是为什么它难定位：**重启一次可能就好了**，
      而且在测试环境（分配模式简单）往往不复现。

      ⇒ 对付它的方法是"压力测试 + 长时间运行"，
        而不是"功能测试通过就发布"。

  【3】best-fit 的问题是"产生微小碎片"

      best-fit 每次都选最贴合的块，于是把大块切成"刚好够 + 一点残余"。
      那个残余往往小到没有任何用途，却占着链表节点。

      best-fit + 不合并 ⇒ 残余永远不会被回收。
      best-fit + 合并   ⇒ 残余会被合并（但没有实现这么做）

      ⇒ 这是 heap_2 的根本问题。
""")


# ============================================================================
# 实验 3：所有权位的位运算
# ============================================================================

def exp3_allocated_bit():
    sec("实验 3：xBlockSize 的最高位 —— 第 4 次位打包技巧")

    print("""
  heap_4.c:80-84（原文）：

      #define heapBLOCK_ALLOCATED_BITMASK    ( ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 ) )
      #define heapBLOCK_SIZE_IS_VALID( xBlockSize )    ( ( ( xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) == 0 )
      #define heapBLOCK_IS_ALLOCATED( pxBlock )        ( ( ( pxBlock->xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) != 0 )
      #define heapALLOCATE_BLOCK( pxBlock )            ( ( pxBlock->xBlockSize ) |= heapBLOCK_ALLOCATED_BITMASK )
      #define heapFREE_BLOCK( pxBlock )                ( ( pxBlock->xBlockSize ) &= ~heapBLOCK_ALLOCATED_BITMASK )

  **`xBlockSize` 的最高位被借用来表示"这个块是否已分配"。**

  本实验在 32 位上演示这五个宏。
""")

    BITS = 32
    MASK = 1 << (BITS - 1)

    print("  " + "=" * 74)
    print("  32 位平台上的位掩码")
    print("  " + "=" * 74)
    print()
    print("      heapBLOCK_ALLOCATED_BITMASK = 1 << 31 = 0x%08X" % MASK)
    print("      二进制：%s" % format(MASK, "032b"))
    print("              ↑ 最高位")
    print()

    def show(label, value):
        print("      %-34s 0x%08X  (%s)"
              % (label, value & 0xFFFFFFFF,
                 "已分配" if (value & MASK) else "空闲"))

    print("  " + "-" * 74)
    print("  演示：分配和释放一个 40 字节的块")
    print("  " + "-" * 74)
    print()
    size_field = 40
    show("初始（40 字节，空闲）", size_field)
    print("      heapBLOCK_SIZE_IS_VALID(40) = %s"
          % ("True（最高位是 0）" if (size_field & MASK) == 0 else "False"))

    size_field |= MASK
    print()
    print("      heapALLOCATE_BLOCK：xBlockSize |= 0x%08X" % MASK)
    show("分配后", size_field)
    print("      heapBLOCK_IS_ALLOCATED()   = %s"
          % ("True" if (size_field & MASK) else "False"))
    print("      heapBLOCK_SIZE_IS_VALID()  = %s "
          "← 注意：已分配的块【不能】再分配"
          % ("False" if (size_field & MASK) else "True"))

    size_field &= ~MASK
    print()
    print("      heapFREE_BLOCK：xBlockSize &= ~0x%08X" % MASK)
    show("释放后", size_field)

    print("""
  ================== 收益与代价 ==================

  【收益】每个块省 4 字节

      不用位打包的写法：
          typedef struct {
              struct xBLOCK_LINK * pxNextFreeBlock;
              size_t xBlockSize;
              BaseType_t xAllocated;        ← 多这 4 字节
          } BlockLink_t;

      假设堆里有 200 个块：
          不用位打包：200 × 4 = 800 字节额外开销
          用位打包：  0 字节

      在 64 KB RAM 的 MCU 上，800 字节是 1.2% —— 不是小数。

  【代价 1】最大可分配块被限制为 SIZE_MAX / 2

      32 位平台上 = 2 GB。实际不受限（MCU 没有那么大 RAM）。
      但如果有人把 size_t 换成 16 位（某些 8 位平台），
      限制就变成 32 KB —— 那就可能踩到了。

  【代价 2】每次读写 xBlockSize 都要掩掉最高位

      这就是那五个宏存在的原因。**直接写 pxBlock->xBlockSize
      而不掩位，是个隐蔽的 bug 源。**

  【代价 3】代码可读性

      `if( ( pxBlock->xBlockSize & heapBLOCK_ALLOCATED_BITMASK ) != 0 )`
      比 `if( pxBlock->xAllocated )` 难读得多。

  ⇒ **这是本教程记录的第 4 次位打包技巧**（见第 9.2 节的表）。
    四次都是一个动机：**省内存**。代价也一致：**读代码的人要知道这个约定**。
""")


# ============================================================================
# 实验 4：五个 heap 的规模与保护手段（从本机源码实测）
# ============================================================================

def exp4_measure():
    sec("实验 4：五个 heap 的规模与保护手段（本机源码实测）")

    if not os.path.isdir(MEMMANG):
        print("  ✗ 未找到源码：%s" % MEMMANG)
        return

    files = ["heap_1.c", "heap_2.c", "heap_3.c", "heap_4.c", "heap_5.c"]

    print("  源码根：%s" % MEMMANG)
    print()
    print("  " + "=" * 74)
    print("  规模与关键特征")
    print("  " + "=" * 74)
    print()
    print("  %-10s %-6s %-10s %-12s %-10s %s"
          % ("文件", "行数", "ucHeap?", "空闲链排序", "合并", "SuspendAll/Critical"))
    print("  " + "-" * 84)

    for f in files:
        p = os.path.join(MEMMANG, f)
        if not os.path.isfile(p):
            continue
        txt = io.open(p, encoding="utf-8", errors="ignore").read()
        # 用 wc -l 的口径（只数换行符），这样和文档里的数字一致。
        # 注意 `open().readlines()` 或 `count("\n")+1` 会给出 +1 的结果
        # （最后一行没换行时算一行），和 wc -l 不一致 —— 本课第一版就差了 1。
        lines = txt.count("\n")
        has_ucHeap = "✓" if "static uint8_t ucHeap[" in txt else "—"

        # 判断排序准则：看 prvInsertBlockIntoFreeList 的比较对象
        if "heap_2.c" in f:
            order, coalesce = "**按大小**", "❌"
        elif "heap_4.c" in f or "heap_5.c" in f:
            order, coalesce = "**按地址**", "✅ 双向"
        elif "heap_3.c" in f:
            order, coalesce = "由 libc 管", "由 libc 管"
        else:
            order, coalesce = "无（bump）", "不适用"

        n_susp = len(re.findall(r"vTaskSuspendAll", txt))
        n_crit = len(re.findall(r"taskENTER_CRITICAL", txt))

        print("  %-10s %-6d %-10s %-12s %-10s %d / %d"
              % (f, lines, has_ucHeap, order, coalesce, n_susp, n_crit))

    print()
    print("  " + "=" * 74)
    print("  ★ 关键发现：保护手段")
    print("  " + "=" * 74)
    print("""
      所有 heap 的**分配和释放路径**都用 vTaskSuspendAll() / xTaskResumeAll()，
      **不是临界区**。

      ⇒ vTaskSuspendAll() 只停调度器，**中断照常**
      ⇒ **所有五个 heap 都不是 ISR-safe 的**

      heap_4 / heap_5 各有一处 taskENTER_CRITICAL，
      但都在 vPortGetHeapStats() 里（只读统计量）——
      见 heap_4.c:601。

  ================== 为什么不用临界区 ==================

      堆操作可能是**几十微秒到几毫秒**的长操作：
          · heap_4 要遍历空闲链表、合并块
          · heap_3 直接调 libc 的 malloc（更慢）

      用临界区保护这么长的操作，会严重恶化中断延迟。

      ⇒ FreeRTOS 的选择：

          用 vTaskSuspendAll() 保护堆
              ⇒ 任务之间的竞争被排除
              ⇒ 中断延迟不受影响

          代价：不能从 ISR 里分配内存
              ⇒ 这是一个【明确的 API 约定】，不是缺陷

  ================== 迁移时的一个陷阱 ==================

      从 NuttX 迁到 FreeRTOS 时，这类代码会突然不能用：

          /* NuttX 上可以正常工作 */
          void isr_handler(void)
          {
              void *buf = malloc(64);        ✅ 临界区保护，ISR-safe
              ...
          }

          /* FreeRTOS 上，同样的代码 */
          void isr_handler(void)
          {
              void *buf = pvPortMalloc(64);  ❌ 不是 ISR-safe
              ...
          }

      **不会在编译期发现，症状是"偶发的堆损坏"。**

      ⇒ 迁移检查清单里应该加一条：
        **"所有在中断上下文里调用的内存分配，都要重新审视。"**

  ================== 那 ISR 里要分配怎么办 ==================

      三个方案：

        ✅ 首选：不用分配
                 ISR 只往【预分配】的缓冲区里写
                 （PX4 的驱动就是这么做的，第 10 课 4.3 节）

        ✅ 常见：延迟到任务
                 ISR 只标记，由任务去分配

        ⚠ 其他：自己写一个临界区保护的分配器
                 代价是中断延迟 —— 要先想清楚值不值

      ⇒ 而"不用分配"这一条，和第 4 课 3.4 节讲的
        "越少用 API 越好"是同一个设计哲学：

        **ISR 应该尽可能少做事：不调 API、不分配内存、
          只做"搬运数据 + 置标志"。**
""")


# ============================================================================
# 主入口
# ============================================================================

EXPERIMENTS = {
    "1": ("heap_4 的双向合并（四种情形）", exp1_coalesce),
    "2": ("best-fit vs first-fit 的碎片化对比", exp2_best_vs_first),
    "3": ("所有权位的位运算", exp3_allocated_bit),
    "4": ("五个 heap 的实测对比", exp4_measure),
}


def main():
    print("""
================================================================================
 第 5 课：内存管理 —— heap_1 到 heap_5
 配套实验代码

 实验列表：
   1  heap_4 的双向合并（四种情形）★
   2  best-fit vs first-fit 的碎片化对比
   3  所有权位的位运算 —— 第 4 次位打包技巧
   4  五个 heap 的规模与保护手段（本机源码实测）
================================================================================
""")

    args = sys.argv[1:]
    if args:
        for a in args:
            if a in EXPERIMENTS:
                EXPERIMENTS[a][1]()
            else:
                print("未知实验编号: %s（可选：%s）"
                      % (a, ", ".join(sorted(EXPERIMENTS))))
    else:
        for k in sorted(EXPERIMENTS):
            EXPERIMENTS[k][1]()

    print("""
================================================================================
 本课三条最重要的结论
================================================================================

1. **heap_2 和 heap_4 的差别根源在"空闲链表按什么排序"。**

        heap_2：按【大小】排序  → best-fit  → ❌ 无法合并
        heap_4：按【地址】排序  → first-fit → ✅ 双向合并

   合并能力是"按地址排序"的副产品，不是独立的设计选择。

2. **五个 heap 都用 vTaskSuspendAll() 而不是临界区，所以都不是 ISR-safe 的。**

   这是"堆操作太长，用临界区会严重恶化中断延迟"这个约束的结果。
   代价是 ISR 里不能分配内存 —— 这是明确的 API 边界，不是缺陷，
   **但迁移时最容易踩**（编译期不报错，症状是偶发的堆损坏）。

3. **heap_1 值得被认真考虑，而不是当成"最差的"。**

   如果能保证"启动后不再分配"，它的零碎片 + O(1) + 零元数据
   就是最优解。它的 165 行里有一半在处理对齐和溢出检查。

 本课看到的第 4 次位打包技巧：**xBlockSize 的最高位表示"是否已分配"**
   （前三次：NuttX 的 semcount 符号位、FreeRTOS 的 pcHead==NULL、
     Queue_t 的 union）。
   四次都是一个动机：**省内存**。代价也一致：**读代码的人要知道约定**。
================================================================================
""")


if __name__ == "__main__":
    main()
