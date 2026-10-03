# -*- coding: utf-8 -*-
"""ch07_dts.py —— 卷 7 第 5 章：设备树是怎么被"读"出来的（配套实验）

设备树（Device Tree）是 ARM 嵌入式 Linux 描述硬件的方式：
**内核源码里不再写"这块板子上有什么"，改由一份数据文件描述**。
内核启动时读它，按里面的 `compatible` 字符串去找对应驱动。

本脚本做一件在 x86 机器上也能做的事：**自己解析一份 DTS**，
把内核读它的过程演示出来。重点讲清三件事 ——

  ① **`compatible` 是驱动匹配的唯一钥匙**：
     `"vendor,device"` 两段式，内核拿它去驱动表里查。
  ② **`reg` 的含义不是固定的** —— 它由**父节点**的
     `#address-cells` / `#size-cells` 决定。同一串数字，
     换个父节点就解出完全不同的地址和长度。
     ★ 这是设备树里最容易搞错、又最不容易发现的一处。
  ③ **`<&label>` 是引用**，解析后变成一个整数（phandle），
     内核靠它在节点之间连线（"这个设备的中断接到哪个控制器"）。

⚠️ 这是一个**教学用的简化解析器**，只认本脚本用到的那部分语法。
   真编译要 `dtc`（device tree compiler）。它的价值在于**语义**，
   不在于替代 dtc。

跑法：python code/ch07_dts.py
"""

import re
import sys

# ---------------------------------------------------------------------------
# 一份虚构的飞控板设备树（结构照着真实板子写，内容是为了讲清语义）
# ---------------------------------------------------------------------------
DTS = """
/dts-v1/;

/ {
    compatible = "qgbf,fmu-v1\0";
    #address-cells = <1>;
    #size-cells = <1>;

    soc {
        compatible = "simple-bus";
        #address-cells = <1>;
        #size-cells = <1>;

        uart4: serial@40004000 {
            compatible = "st,stm32-uart";
            reg = <0x40004000 0x400>;
            interrupts = <38 0>;
            status = "okay";
        };

        imu: spi@40013000 {
            compatible = "st,stm32-spi";
            reg = <0x40013000 0x400>;
            #address-cells = <1>;
            #size-cells = <0>;

            icm42688: imu@0 {
                compatible = "invensense,icm42688";
                reg = <0>;                 /* 片选号，长度由父节点决定：0 个 cell */
                spi-max-frequency = <24000000>;
                interrupts = <45 1>;
            };
        };

        can1: can@40006400 {
            compatible = "bosch,bxcan-fd";
            reg = <0x40006400 0x400>;
            interrupts = <21 0>;
            status = "disabled";
        };

        led: gpio@40020000 {
            compatible = "st,stm32-gpio";
            reg = <0x40020000 0x400>;
            #address-cells = <1>;
            #size-cells = <0>;
            led0: led@0 {
                compatible = "gpio-leds";
                reg = <0>;
                label = "blue";
            };
        };
    };

    /* 从别处引用一个节点：靠 label 解析成 phandle */
    chosen {
        stdout-path = &uart4;
    };
};
"""


# ===========================================================================
# 一个够用的 DTS 解析器（教学用）
# ===========================================================================
TOK = re.compile(r'''
      (?P<comment>/\*.*?\*/)
    | (?P<version>/dts-v1/|/plugin/)      # 文件头，解析时直接跳过
    | (?P<label>[A-Za-z_][\w]*)\s*:
    | (?P<ref><&[A-Za-z_][\w]*>)          # 必须排在 cells 前面
    | (?P<bareref>&[A-Za-z_][\w]*)        # 不带尖括号的引用：`prop = &uart4;` 也合法
    | (?P<cells><[^>]*>)
    | (?P<string>"[^"]*")
    | (?P<bytes>\[[^\]]*\])
    | (?P<rootname>/)                     # 根节点就写作 `/`
    | (?P<name>[A-Za-z_#][\w,.#@+-]*)
    | (?P<punct>[{};=])
    | (?P<ws>\s+)
''', re.X | re.S)


class Node(object):
    def __init__(self, name, label=None, parent=None):
        self.name = name
        self.label = label
        self.parent = parent
        self.props = {}
        self.children = []
        self.phandle = None

    def path(self):
        if self.parent is None:
            return '/'
        p = self.parent.path()
        return (p.rstrip('/') + '/' + self.name)

    def find(self, name):
        for c in self.children:
            if c.name == name or c.name.split('@')[0] == name:
                return c
        return None


def tokenize(src):
    out, pos = [], 0
    while pos < len(src):
        m = TOK.match(src, pos)
        if not m:
            raise ValueError('看不懂的字符 %r 在第 %d 字节' % (src[pos], pos))
        pos = m.end()
        kind = m.lastgroup
        if kind in ('ws', 'comment', 'version'):
            continue
        if kind == 'rootname':
            out.append(('name', '/'))          # 根节点当普通节点名处理
            continue
        # ⚠️ 用 `m.group(kind)` 而不是 `m.group()`：`label` 那条模式把结尾的
        #    `:` 也算进了**整体匹配**，用 group() 取回来的标签会长成 `uart4:`，
        #    于是后面 `&uart4` 按名字查表时 KeyError（实测）。
        out.append((kind, m.group(kind)))
    return out


def parse_cells(text):
    """`<0x40004000 0x400>` → [0x40004000, 0x400]；`<&uart4>` → [('ref','uart4')]"""
    inner = text.strip()[1:-1].strip()
    vals = []
    for part in inner.split():
        if part.startswith('&'):
            vals.append(('ref', part[1:]))
        else:
            vals.append(int(part, 0))
    return vals


def parse_dts(src):
    toks = tokenize(src)
    i = 0
    root = Node('')
    stack = [root]
    pending_label = None

    while i < len(toks):
        kind, val = toks[i]
        if kind == 'label':
            pending_label = val
            i += 1
            continue
        if kind == 'name':
            # 节点定义？往后看是不是 `{`
            if i + 1 < len(toks) and toks[i + 1] == ('punct', '{'):
                # ★ 文件里的 `/ { … }` 就是**根节点本身**，不是它的孩子。
                #   第一版按普通节点建了孩子，于是路径全变成了 `//soc`、`//chosen`。
                #   （真 dtc 里根节点路径就是 `/`，下面挂 soc/chosen。）
                if val == '/' and len(stack) == 1 and not stack[-1].children:
                    stack[-1].name = '/'
                    stack.append(stack[-1])
                    pending_label = None
                    i += 2
                    continue
                node = Node(val, pending_label, stack[-1])
                stack[-1].children.append(node)
                stack.append(node)
                pending_label = None
                i += 2
                continue
            # 否则是属性名
            if i + 1 < len(toks) and toks[i + 1] == ('punct', '='):
                vk, vv = toks[i + 2]
                if vk == 'string':
                    value = vv.strip('"').replace('\\0', '\0').split('\0')
                    value = [x for x in value if x != '']
                    if len(value) == 1:
                        value = value[0]
                elif vk == 'cells':
                    value = parse_cells(vv)
                elif vk == 'bytes':
                    value = vv
                elif vk in ('ref', 'bareref'):
                    value = [('ref', vv.lstrip('<&').rstrip('>'))]
                else:
                    value = vv
                stack[-1].props[val] = value
                i += 3
                while i < len(toks) and toks[i] != ('punct', ';'):
                    i += 1
                i += 1
                continue
        if val == '{' if kind == 'punct' else False:
            i += 1
            continue
        if kind == 'punct' and val == '}':
            if len(stack) > 1:
                stack.pop()
            i += 1
            while i < len(toks) and toks[i] == ('punct', ';'):
                i += 1
            continue
        i += 1

    # 分配 phandle：按出现顺序编号（真实 dtc 也是这么给的，只是号不一样）
    n = [1]
    def assign(node):
        if node.label:
            node.phandle = n[0]
            n[0] += 1
        for c in node.children:
            assign(c)
    assign(root)
    return root


def walk(node, out):
    out.append(node)
    for c in node.children:
        walk(c, out)
    return out


# ===========================================================================
def main():
    print('=' * 72)
    print('① 解析这份 DTS')
    print('=' * 72)
    root = parse_dts(DTS)
    nodes = walk(root, [])
    print('  解析出 %d 个节点：' % (len(nodes) - 1))
    for nd in nodes:
        if nd.parent is None:
            continue
        tag = ('  [%s]' % nd.label) if nd.label else ''
        print('    %-34s%s' % (nd.path(), tag))

    # -----------------------------------------------------------------------
    print()
    print('=' * 72)
    print('② compatible：内核靠这个字符串找驱动')
    print('=' * 72)
    print('  %-30s %s' % ('节点', 'compatible'))
    for nd in nodes:
        if 'compatible' in nd.props:
            c = nd.props['compatible']
            print('  %-30s %s' % (nd.path(), c if isinstance(c, str) else c))

    print()
    print('  ★ compatible 的写法是 "厂商,型号"：')
    print('    · **从前到后**依次尝试匹配（越靠前越具体）')
    print('    · 内核里对应的驱动用 of_device_id 表登记同一串字符')
    print('    · 两边**逐字节相等**才算匹配 —— 拼错一个字符就是"设备找不到"')
    print('    这就是为什么改设备树能换驱动，而不用改内核源码。')

    # -----------------------------------------------------------------------
    print()
    print('=' * 72)
    print('③ ★★ reg 的解码：同一串数字，两种含义')
    print('=' * 72)
    print('  reg 的格式**由父节点**的 #address-cells / #size-cells 决定：')
    print('    前 #address-cells 个 cell 是地址，后 #size-cells 个是长度。')
    print()

    def decode_reg(nd):
        p = nd.parent
        ac = p.props.get('#address-cells', [2])
        sc = p.props.get('#size-cells', [1])
        ac = ac[0] if isinstance(ac, list) else ac
        sc = sc[0] if isinstance(sc, list) else sc
        cells = nd.props.get('reg')
        if not isinstance(cells, list):
            return None
        cells = [c for c in cells if isinstance(c, int)]
        addr = 0
        for k in range(ac):
            addr = (addr << 32) | cells[k]
        size = 0
        for k in range(sc):
            size = (size << 32) | cells[ac + k]
        return ac, sc, addr, size

    for nd in nodes:
        if 'reg' not in nd.props:
            continue
        r = decode_reg(nd)
        if not r:
            continue
        ac, sc, addr, size = r
        raw = ' '.join('0x%x' % c for c in nd.props['reg'] if isinstance(c, int))
        p = nd.parent
        print('  %s' % nd.path())
        print('      父节点 #address-cells=%d  #size-cells=%d' % (ac, sc))
        print('      reg = <%s>' % raw)
        if sc == 0:
            # ⚠️ 这里不是"长度 = 0"，而是**根本没有长度这一项** ——
            #    多出来的数字属于**下一个设备**。印成"长度 0x0"会让人
            #    以为"长度为 0"，那是另一回事。
            print('      ⇒ 地址 0x%X，**没有长度字段**（剩下的数字属于下一个设备）'
                  % addr)
        else:
            print('      ⇒ 地址 0x%X，长度 0x%X' % (addr, size))
        print()

    # ★★ 本节的杀手实验：把同一个 reg 放到不同的父节点下
    print('  —— 把同一串 `reg = <0 0x400>` 放进两种父节点 ——')
    for ac_sc, why in (((1, 1), '一个地址 cell + 一个长度 cell'),
                       ((1, 0), '一个地址 cell + **没有**长度 cell')):
        cells = [0, 0x400]
        ac, sc = ac_sc
        addr = cells[0]
        size = cells[1] if sc == 1 else None
        print('    #address-cells=%d #size-cells=%d （%s）'
              % (ac, sc, why))
        if size is None:
            print('         ⇒ 地址 = 0x0，**长度没有**（剩下的 0x400 是**下一个设备**的）')
        else:
            print('         ⇒ 地址 = 0x0，长度 = 0x400')

    print()
    print('  ★★ 这就是设备树里最容易搞错的一处：')
    print('     你看到的 `reg = <0 0x400>` 是**裸数字**，')
    print('     它的含义要**往上翻到父节点**才知道。')
    print('     写错了不会报"语法错误"，只会让地址解错 —— **症状是设备不工作**。')

    # -----------------------------------------------------------------------
    print()
    print('=' * 72)
    print('④ <&label> 是引用：解析成 phandle')
    print('=' * 72)
    print('  %-12s %-10s %s' % ('label', 'phandle', '指向的节点'))
    by_label = {nd.label: nd for nd in nodes if nd.label}
    for lb, nd in by_label.items():
        print('  %-12s %-10s %s' % (lb, '<%d>' % nd.phandle, nd.path()))

    ch = root.find('chosen')
    ref = ch.props.get('stdout-path') if ch else None
    if isinstance(ref, list) and ref and isinstance(ref[0], tuple):
        target_label = ref[0][1]
        target = by_label[target_label]
        print()
        print('  chosen/stdout-path = &%s' % target_label)
        print('    ⇒ 解析为 phandle <%d>' % target.phandle)
        print('    ⇒ 内核据此知道"console 用哪个串口" = %s' % target.path())

    print()
    print('  ★ 引用为什么必须存在：vmlinux 里没有"字符串形式的节点名"，')
    print('    节点之间靠**整数 phandle** 连线。所以 &label 打错 = 编译期就报错'
          '（dtc 会查），比运行期才发现好得多。')

    # -----------------------------------------------------------------------
    print()
    print('=' * 72)
    print('⑤ status："有硬件但这次不用"')
    print('=' * 72)
    for nd in nodes:
        st = nd.props.get('status')
        if st:
            print('  %-30s status = "%s"' % (nd.path(), st))
    print()
    print('  ★ `status = "disabled"` 是个很有用的状态：节点还在（引脚、时钟的')
    print('    占位还在），但内核**不给它绑驱动**。')
    print('    飞控上常见的用法：一颗 SoC 做几个型号的板子，')
    print('    差异部分靠设备树里的 enabled/disabled 区分 —— **同一个内核**。')

    print()
    print('=' * 72)
    print('小结：内核从设备树里读到的三样东西')
    print('=' * 72)
    print('  1. **有什么**   —— 节点 + compatible（→ 匹配哪个驱动）')
    print('  2. **在哪**     —— reg 按父节点的 cells 解码出地址与长度')
    print('  3. **怎么连**   —— interrupts 的控制器、phandle 引用、时钟/引脚')
    print()
    print('  ★ 三者缺一，设备就"存在但用不了"。排查顺序也照这个来：')
    print('    先看 compatible 匹不匹配（dmesg 里有没有 probe 成功），')
    print('    再看 reg 解出来的地址对不对，最后看中断/时钟连对没有。')


if __name__ == '__main__':
    main()
