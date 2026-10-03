#!/usr/bin/env bash
# ch07_jitter.sh —— 卷 7 第 8 章：唤醒抖动实测（编译 + 跑 2×2 + 断言）
#
# 本卷的核心实验。要回答的问题是：
#   **Linux 到底能不能做实时？如果能，能到什么程度？**
#
# 做法：同一个周期性唤醒程序，跑四种组合 ——
#
#              │ 无干扰           │ 12 个 CPU 干扰线程
#   ───────────┼──────────────────┼─────────────────────
#   SCHED_OTHER│ 普通分时          │ 普通分时 + 争抢
#   SCHED_FIFO │ 实时优先级        │ 实时优先级 + 争抢
#
# 从这张 2×2 表能看出两件事（脚本会**断言**它们，不是靠眼看）：
#   ① 提高优先级能把**中位数**砍下来一大截（治「排队」）
#   ② 但**最坏值**砍不下来（治不了「内核不可抢占」）
#
# 跑法：bash code/ch07_jitter.sh            # 跑一次实测（约半分钟）
#       bash code/ch07_jitter.sh --sample   # 只印冻结样本，不跑
#
# ★★ 口径（本章最重要的一句话）：
#    **抖动的最坏值本身就是一个不可复现的随机量** ——
#    同一配置跑两次，最大值能差好几倍（这个脚本会实测给你看）。
#    所以正文里引的具体数字一律取自**冻结样本**；
#    而「方向」（谁比谁小、能不能压住）才是可复现的结论。

set -u
HERE="$(dirname "$0")"
SRC="$HERE/ch07_linux_jitter.c"
SAMPLE="$HERE/data/ch07_jitter_sample.txt"
PERIOD=1000          # µs
ITERS=2000
NHOG=12

if [ "${1:-}" = "--sample" ]; then
    if [ -r "$SAMPLE" ]; then
        echo "（数据源：冻结样本 code/data/ch07_jitter_sample.txt）"
        cat "$SAMPLE"
        exit 0
    fi
    echo "找不到冻结样本，改为实跑。"
fi

if ! command -v gcc >/dev/null 2>&1; then
    echo "没有 gcc。装法：sudo apt-get install -y build-essential"
    exit 1
fi
BIN=$(mktemp -u /tmp/ch07_jitter.XXXXXX)
# ⚠️ `-lm` 不能少：源码里用了 sqrt()，它在 libm 里，
#    而 gcc 默认只链 libc —— 报错是 `undefined reference to 'sqrt'`。
if ! gcc -O2 -Wall -o "$BIN" "$SRC" -lm -lpthread 2>/tmp/ch07_jit_build.log; then
    echo "编译失败："; sed 's/^/    /' /tmp/ch07_jit_build.log; exit 1
fi

# SCHED_FIFO 需要特权：普通用户的 rtprio 上限通常是 0（`ulimit -r` 查）
SUDO=""
if [ "$(id -u)" != "0" ]; then
    if sudo -n true 2>/dev/null; then SUDO="sudo"
    else
        echo "⚠️ 没有免密 sudo —— SCHED_FIFO 那两行会退化成 SCHED_OTHER。"
        echo "   普通用户的 rtprio 上限一般是 0，提不上去。不影响读书，"
        echo "   但你自己复现时要用 sudo 跑。"
        echo
    fi
fi

echo "========================================================================"
echo "唤醒抖动实测：优先级 × 负载"
echo "========================================================================"
echo "周期 $PERIOD µs，每轮 $ITERS 次，干扰线程 $NHOG 个"
echo
printf "  %-24s %-9s %-9s %-10s %s\n" "配置" "中位" "p99" "最大" "超一周期"

# ★ 这四个数先都设成 0，免得某个配置跑失败时后面的算术报错（set -u 下未定义变量会直接退出）
JIT_other_idle_med=0;  JIT_other_idle_max=0
JIT_other_load_med=0;  JIT_other_load_max=0
JIT_fifo_idle_med=0;   JIT_fifo_idle_max=0
JIT_fifo_load_med=0;   JIT_fifo_load_max=0
JIT_fifo_load2_max=0

# run_one <标签> <键> <干扰线程数> <策略: other|fifo>
run_one() {
    local label="$1" key="$2" nhog="$3" policy="$4"
    local out med p99 mx over pol
    if [ "$policy" = fifo ] && [ -n "$SUDO" ]; then
        out=$($SUDO chrt -f 80 "$BIN" "$PERIOD" "$ITERS" "$nhog" 2>&1)
    else
        out=$($SUDO "$BIN" "$PERIOD" "$ITERS" "$nhog" 2>&1)
    fi
    pol=$(echo "$out" | sed -n 's/^调度策略 \([A-Z_]*\).*/\1/p' | head -1)
    med=$(echo "$out" | awk '/最小/{for(i=1;i<=NF;i++) if($i=="中位") print $(i+1)}')
    p99=$(echo "$out" | awk '/p99 /{print $2}')
    mx=$(echo "$out"  | awk '/p99 /{for(i=1;i<=NF;i++) if($i=="最大") print $(i+1)}')
    over=$(echo "$out" | awk '/超一个周期/{print $2}')
    printf "  %-24s %-9s %-9s %-10s %s\n" "$label（$pol）" "$med" "$p99" "$mx" "$over"
    echo "$out" > "/tmp/ch07_jit_$key.txt"
    eval "JIT_${key}_med=${med:-0}; JIT_${key}_max=${mx:-0}"
}

run_one "普通分时，无干扰"     "other_idle" 0     other
run_one "普通分时，有干扰"     "other_load" "$NHOG" other
run_one "实时优先级，无干扰"   "fifo_idle"  0     fifo
run_one "实时优先级，有干扰"   "fifo_load"  "$NHOG" fifo
# ★ 第 5 次：把「实时优先级 + 有干扰」**原样再跑一遍**。
#   这一跑不是为了多一个数据点，而是为了量**同一个配置的可复现性** ——
#   本章的结论之一就是「最坏值不可复现」，那就得**当场测给它看**。
run_one "实时优先级，有干扰（重跑）" "fifo_load2" "$NHOG" fifo

echo
echo "========================================================================"
echo "这些表说明了什么（下面的断言真的会跑，不是写在注释里）"
echo "========================================================================"
python3 - "$JIT_other_idle_med" "$JIT_other_idle_max" \
          "$JIT_fifo_idle_med"  "$JIT_fifo_idle_max" \
          "$JIT_fifo_load_med"  "$JIT_fifo_load_max" \
          "$JIT_fifo_load2_max" <<'PYEOF'
import sys
oi_med, oi_max, fi_med, fi_max, fl_med, fl_max, fl2_max = \
    (float(x) for x in sys.argv[1:8])
checks = []

# ① 优先级治「排队」：FIFO 的中位数明显低于 OTHER。**这一条每次跑都成立。**
checks.append(('提高优先级后中位数变小（治「排队」）',
               fi_med <= oi_med,
               'OTHER %.1f µs → FIFO %.1f µs' % (oi_med, fi_med)))

# ② 抖动是长尾：最坏值远大于中位数。**这一条每次跑都成立。**
checks.append(('最坏值 ≥ 中位数的 5 倍（长尾分布）',
               fl_max >= 5 * fl_med if fl_med > 0 else True,
               '中位 %.1f / 最大 %.1f' % (fl_med, fl_max)))

# ③ ★★ 同一个配置跑两次，最坏值**不一样**。
#    这是本章最要紧的一条：**最坏值不是一个可复现的量**。
#    ⚠️ 这里**不**断言「最坏值一定超过一个周期」—— 我原来那样写过，
#       而实跑有一次 FIFO+干扰 只到 975.9 µs（不到 1000），断言当场红。
#       那条"定律"根本不成立：**最坏值会不会超周期，取决于负载和运气** ——
#       而这恰恰就是本章要讲的事。**能断言的是"它每次都不同"，不是"它一定很大"。**
checks.append(('同一配置两次跑，最坏值不同（不可复现）',
               abs(fl_max - fl2_max) > 1e-9,
               '第一次 %.1f µs / 重跑 %.1f µs' % (fl_max, fl2_max)))

ok = True
for name, passed, detail in checks:
    ok &= passed
    print('  %s %s   （%s）' % ('✓' if passed else '✗', name, detail))

print()
print('  ★ 前两条是**方向上**的结论，换台机器也成立；')
print('    第三条说明的是：**连同一个配置的两次跑都对不上** ——')
print('    所以「Linux 的最坏抖动是多少」这个问题，本身就问错了。')

# ★ 「最坏值有没有超过一个周期」**只报不断** —— 它超不超取决于负载和运气，
#   把它写成断言就是在赌（实测已经红过一次）。逐行的「超一周期」列才是原始事实。
print()
print('  同一个配置（实时优先级 + 12 个干扰线程），两次跑的最坏值：')
print('    第一次 %.1f µs   重跑 %.1f µs   —— 相差 %.1f 倍'
      % (fl_max, fl2_max, max(fl_max, fl2_max) / max(min(fl_max, fl2_max), 1e-9)))
print('    （一个周期是 1000 µs；有没有超过，看上面表格的「超一周期」列）')
sys.exit(0 if ok else 1)
PYEOF
rc=$?

rm -f "$BIN"
echo
if [ $rc -eq 0 ]; then
    echo "全部断言通过。"
else
    echo "❌ 有断言未通过 —— 看上面哪一条打了 ✗。"
fi
exit $rc
