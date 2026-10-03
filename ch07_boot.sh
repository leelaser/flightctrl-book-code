#!/usr/bin/env bash
# ch07_boot.sh —— 卷 7 第 3 章：启动链的时间都花在哪了
#
# 启动优化的第一步永远是**先量**：不要凭印象砍，要看时间花在哪。
# Linux 内核把启动过程中每一步的时间戳都留在 ring buffer 里（`dmesg`）。
#
# 这个脚本解析它，做两件事：
#   ① 把启动过程按**消息之间的空档**分解 —— 空档 = 内核在等某件事完成；
#   ② 报出最大的几段空档，以及它们前后是哪些子系统。
#
# 跑法：bash code/ch07_boot.sh [窗口秒数]
#   不给参数时**自动判**"启动窗口"到哪结束：从 0 起，到第一个
#   **超过 5 秒的空档**为止 —— 那之后内核已经在正常运行了，不再算启动。
#
#   ★ 为什么要自动判：第一版把默认窗口写死成 30 秒，结果**正文里的数
#     和检查器跑出来的数对不上** —— 因为正文写的是"启动窗口 3 秒"那次的输出，
#     而检查器不带参数跑的是 30 秒窗口，dmesg 里的运行期消息全被算进来了。
#     数字没错，是**口径被参数偷偷改了**。写死的默认值就是这种坑的来源。
#   ★ 显式给参数时以参数为准（便于固定口径做对比）。
#
# ★★ 口径声明：`dmesg` 的内容**完全取决于平台** ——
#    这里在 WSL2 上读到的里程碑，和一块树莓派、一块 Zynq 上的**完全不同**。
#    这个脚本要教的不是"启动要多久"（那个数换台机器就变），
#    而是**"读时间戳、找空档、定位子系统"这套方法**。
#    拿到你自己的板子上跑，得到的才是你的数。

set -u

# ---- 取数据源 ------------------------------------------------------------
# 默认读**你这台机器**的 dmesg；`--sample` 读随脚本带的一份**冻结样本**。
#
# ★★ 为什么要冻结样本：`dmesg` **不是确定的** —— 它会把运行期的消息
#    一直往 ring buffer 里追加（实测同一台机器间隔几分钟再跑，
#    行数从 618 变成 657）。于是"本书正文里印的这几个数"**每次跑都不一样**，
#    没法核对。
#    ⇒ 正文引的数一律取自 `--sample`（冻结样本，任何人跑都是同一份），
#      而**读者自己不带参数跑**，拿到的是自己机器的真实数据。
#      这也正是本卷的口径：**方法可复现，数字各自读**。
SAMPLE="$(dirname "$0")/data/ch07_dmesg_sample.txt"
USE_SAMPLE=0
if [ "${1:-}" = "--sample" ]; then
    USE_SAMPLE=1
    shift
fi
WINDOW="${1:-auto}"

if [ "$USE_SAMPLE" = 1 ]; then
    if [ ! -r "$SAMPLE" ]; then
        echo "找不到冻结样本：$SAMPLE"
        exit 1
    fi
    SRC="$SAMPLE"
    echo "（数据源：冻结样本 code/data/ch07_dmesg_sample.txt —— 为了正文可核对）"
elif ! command -v dmesg >/dev/null 2>&1; then
    echo "dmesg 不可用（有些发行版默认限制非 root 读取）。"
    echo "试试: sudo sysctl -w kernel.dmesg_restrict=0   或   sudo dmesg"
    echo "也可以先用冻结样本：bash code/ch07_boot.sh --sample"
    exit 1
else
    # ★★ 不能写成 `dmesg | python3 - <<'PYEOF'` —— 管道与 heredoc
    #    **抢同一个 stdin**：重定向后 heredoc 赢了，python 从 heredoc 读程序，
    #    dmesg 的输出**被丢掉**，于是程序读到空输入、报"没解析到任何行"。
    #    （第一版就是这么写的。症状是"格式不符"，看着像解析写错了。）
    #    ⇒ 数据走**临时文件**，stdin 只留给程序本身。
    SRC=$(mktemp)
    trap 'rm -f "$SRC"' EXIT
    dmesg 2>/dev/null > "$SRC"
fi

python3 - "$SRC" "$WINDOW" <<'PYEOF'
import re
import sys

path = sys.argv[1]
window_arg = sys.argv[2]
LINE = re.compile(r'^\[\s*(\d+\.\d+)\]\s*(.*)$')

rows = []
with open(path, encoding='utf-8', errors='replace') as fh:
    for ln in fh:
        m = LINE.match(ln.rstrip('\n'))
        if m:
            rows.append((float(m.group(1)), m.group(2).strip()))

if not rows:
    print('  没解析到任何带时间戳的行 —— dmesg 格式可能与预期不符。')
    sys.exit(1)

IDLE = 5.0          # 超过这个长度的空档 ⇒ 认为启动已经结束，后面是运行期

if window_arg == 'auto':
    t_end = rows[-1][0]
    for (t0, _m0), (t1, _m1) in zip(rows, rows[1:]):
        if t1 - t0 > IDLE:
            t_end = t0          # 停在长时间空闲**之前**
            break
    how = '自动判定（第一个 > %.0f s 的空档处）' % IDLE
else:
    t_end = min(rows[-1][0], float(window_arg))
    how = '按参数指定 %.1f s' % float(window_arg)

boot = [r for r in rows if r[0] <= t_end]
print('  ring buffer 共 %d 行，启动窗口取 %s' % (len(rows), how))
print('  窗口内 %d 行，首行 %.6f s，窗口末 %.3f s' % (len(boot), rows[0][0], t_end))

# ---- 空档分析：相邻两条消息之间"没有消息"的那段时间 ----
gaps = []
for (t0, m0), (t1, _m1) in zip(boot, boot[1:]):
    if t1 - t0 > 0:
        gaps.append((t1 - t0, t0, t1, m0))

gaps.sort(reverse=True)
span = boot[-1][0] - boot[0][0]
if span <= 0:
    print('\n  窗口内只有一条消息，算不出空档。')
    sys.exit(0)

# ★★ "所有空档之和"是个**同义反复**：相邻消息的空档按定义就加起来等于跨度，
#    所以"占 100%"永远成立、毫无信息量（第一版就是这么印的）。
#    有意义的量是**大空档占了多少** —— 启动优化要砍的正是那几段。
BIG = 0.05                      # 50 ms 以上才算"一段等待"
big = [g for g in gaps if g[0] >= BIG]
big_sum = sum(g[0] for g in big)
print()
print('  窗口跨度 %.3f s，共 %d 条消息' % (span, len(boot)))
print('  其中 **%.0f ms 以上的大空档** %d 段，合计 %.3f s —— 占跨度的 %.0f%%'
      % (BIG * 1000, len(big), big_sum, 100.0 * big_sum / span))
print('  （剩下的 %d 条消息之间都是细碎的小间隔，合计 %.3f s）'
      % (len(gaps) - len(big), span - big_sum))
print()
print('  最大的 5 段空档（内核在这些时间里没打印任何东西 —— 它在等）：')
print('  %-10s %-10s %-10s %s' % ('空档', '起', '止', '起点的最后一条消息'))
for d, t0, t1, msg in gaps[:5]:
    tail = msg if len(msg) <= 54 else msg[:51] + '...'
    print('  %8.3f s %9.3f s %9.3f s  %s' % (d, t0, t1, tail))

print()
print('  —— 怎么读这张表 ——')
print('  ★ 空档大的地方就是优化靶子。但要**先判断它在等什么**：')
print('    · 等硬件（外设复位、PHY 协商、存储枚举）→ 硬件/驱动层面的事')
print('    · 等超时（探测失败的设备要等满超时才继续）→ **最值得砍的一类**，')
print('      把不需要的驱动关掉，超时那几秒直接消失')
print('    · 等用户空间（init 起来了但服务在慢慢启动）→ 系统层面的事')
print('  ★ 另一种常见靶子是**分散的小延迟**：单看每段都不大，')
print('    加起来却占了一半 —— 那要靠减少驱动数量、简化探测来解决。')

print()
print('  —— 里程碑（按关键字匹配，平台不同匹配到的行也不同）——')
# ⚠️ 这些模式是**按平台写的**，换个平台要跟着改。
#    第一版这里匹配出过笑话：`用户空间` 那条写了个 `init`，
#    结果命中了命令行里的 `initrd=\initrd.img`（0.000 s）——
#    里程碑"全都发生在 0 秒"，看着像启动极快，其实是模式太松。
#    ⇒ 里程碑模式要**盯住那条唯一的、可辨识的消息**，别用泛词。
MILESTONES = [
    ('内核起始',   r'Linux version \d'),
    ('内存管理',   r'Memory: \d|On node \d+, zone'),
    ('中断控制器', r'GIC|IO-APIC|Max logical packages'),
    ('时钟源',     r'clocksource: Switched to'),
    ('存储',       r'SCSI subsystem initialized|mmc\d|nvme|virtio_blk'),
    ('网络',       r'NETDEV_CHANGE|netdev|eth\d.*link'),
    ('用户空间',   r'Freeing unused kernel|Run /sbin/init|systemd\[\d+\]'),
]
for name, pat in MILESTONES:
    hit = None
    for t, m in boot:
        if re.search(pat, m, re.I):
            hit = (t, m)
            break
    if hit:
        tail = hit[1] if len(hit[1]) <= 48 else hit[1][:45] + '...'
        print('  %-10s %9.3f s  %s' % (name, hit[0], tail))
PYEOF

echo
echo "（口径：以上全部数字来自**本机**的 dmesg，换平台就变。方法不变。）"
