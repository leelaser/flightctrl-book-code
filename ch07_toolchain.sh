#!/usr/bin/env bash
# ch07_toolchain.sh —— 卷 7 第 2 章：交叉编译与工具链（配套实验）
#
# 本章要讲清"同一份源码，为什么换个平台就要换一套工具链"。
# 这件事光讲没用，得**真的编出两个架构的可执行文件**，然后当场验证：
#
#   ① `readelf -h` 里的 Machine 字段不同 —— 机器码不是同一套
#   ② 把 arm64 的产物拿回 x86 上跑 —— **Exec format error**（退出码 126）
#   ③ 静态链接把"运行时依赖"从目标平台上删掉 —— 代价是体积
#   ④ `-O0/-O2/-Os` 影响的是**代码段**，不是文件大小（两个量别混）
#
# 依赖：gcc、aarch64-linux-gnu-gcc（可选）、binutils
#   Ubuntu: sudo apt-get install -y gcc-aarch64-linux-gnu libc6-dev-arm64-cross
#
# 跑法：bash code/ch07_toolchain.sh
#
# ★★ 口径声明：本脚本打印的**体积是字节数，跨平台可比**；
#    但**能否执行**那一栏取决于你本机的架构 ——
#    在 x86 上跑，arm64 那份必然报 Exec format error；
#    在 arm64 上跑，反过来。这是本实验要展示的"不可互换"，不是环境缺陷。

set -u
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
cd "$W"

fail() { echo "  ✗ $1"; exit 1; }
ok()   { echo "  ✓ $1"; }

cat > demo.c <<'EOF'
#include <stdio.h>
#include <math.h>
/* 一个够"实在"的函数：能被打包成一条 FMA 指令 */
static double poly(double x) { return ((3.0 * x + 2.0) * x - 5.0) * x + 1.0; }
int main(void)
{
    double s = 0;
    for (int i = 0; i < 1000; i++) s += poly(i * 0.001);
    printf("%f\n", s);
    return 0;
}
EOF

echo "========================================================================"
echo "本机架构与工具链"
echo "========================================================================"
HOST_ARCH=$(uname -m)
echo "  本机架构（uname -m）: $HOST_ARCH"
echo "  本机 gcc           : $(gcc -dumpmachine)"
CROSS=""
for c in aarch64-linux-gnu-gcc arm-linux-gnueabihf-gcc; do
    if command -v "$c" >/dev/null 2>&1; then CROSS=$c; break; fi
done
if [ -n "$CROSS" ]; then
    echo "  交叉工具链         : $CROSS  ($($CROSS -dumpmachine))"
else
    echo "  交叉工具链         : 未安装 —— 只跑本机的部分"
    echo "    装法: sudo apt-get install -y gcc-aarch64-linux-gnu libc6-dev-arm64-cross"
fi

# ---------------------------------------------------------------------------
echo
echo "========================================================================"
echo "① 优化级别影响的是【代码段】，不是文件大小"
echo "========================================================================"
printf "  %-6s %-12s %-12s %s\n" "级别" "代码段(字节)" "文件(字节)" "说明"
for o in 0 2 s; do
    gcc -O$o -o "demo_O$o" demo.c -lm 2>/dev/null || fail "gcc -O$o 失败"
    txt=$(size -d "demo_O$o" | tail -1 | awk '{print $1}')
    fs=$(stat -c %s "demo_O$o")
    case $o in
        0) why="不优化，便于调试" ;;
        2) why="速度优先" ;;
        s) why="体积优先（嵌入式常用）" ;;
    esac
    printf "  -O%-4s %-12s %-12s %s\n" "$o" "$txt" "$fs" "$why"
done
# 代码段随优化级别单调不增；文件大小则因为 ELF 对齐而不一定跟着变
t0=$(size -d demo_O0 | tail -1 | awk '{print $1}')
t2=$(size -d demo_O2 | tail -1 | awk '{print $1}')
ts=$(size -d demo_Os | tail -1 | awk '{print $1}')
[ "$t2" -le "$t0" ] || fail "O2 的代码段不该比 O0 大（$t2 > $t0）"
[ "$ts" -le "$t2" ] || fail "Os 的代码段不该比 O2 大（$ts > $t2）"
ok "代码段满足 -O0 ≥ -O2 ≥ -Os（$t0 ≥ $t2 ≥ $ts）"
echo "  ★ 注意「文件字节」一列几乎不动 —— 那个量主要被 ELF 段对齐和动态"
echo "    链接的运行时开销决定，**它不衡量你的代码有多大**。"
echo "    在飞控上要看的通常是代码段（进 Flash 的那部分）。"

# ---------------------------------------------------------------------------
echo
echo "========================================================================"
echo "② 动态 vs 静态链接：拿体积换"运行时依赖""
echo "========================================================================"
gcc -O2 -static -o demo_O2_static demo.c -lm 2>/dev/null || fail "静态链接失败"
printf "  %-18s %-12s %s\n" "链接方式" "文件(字节)" "运行时需要什么"
printf "  %-18s %-12s %s\n" "动态" "$(stat -c %s demo_O2)" "目标机上的 libc.so / ld.so，且版本要对得上"
printf "  %-18s %-12s %s\n" "静态" "$(stat -c %s demo_O2_static)" "什么都不需要"
d=$(stat -c %s demo_O2); s=$(stat -c %s demo_O2_static)
[ "$s" -gt "$d" ] || fail "静态链接应当更大（实测 $s vs $d）"
ok "静态比动态大 $((s / d)) 倍（$d → $s 字节）"
echo "  ★ 飞控上常见的选择是**静态**：多花几百 KB 的 Flash，换掉"
echo "    「目标机 libc 版本不对就起不来」这一类现场故障。"
echo "  ★ 真正的依赖清单用 ldd 看（静态的那个会打印 not a dynamic executable）："
ldd demo_O2 | sed 's/^/      /'

# ---------------------------------------------------------------------------
if [ -z "$CROSS" ]; then
    echo
    echo "（未装交叉工具链，跳过 ③④ —— 装了才有 ③④ 这两节）"
    echo
    echo "全部可跑的断言通过。"
    exit 0
fi

X=${CROSS%%-*}       # aarch64 或 arm
echo
echo "========================================================================"
echo "③ 同一份源码，两个架构：机器码不是同一套"
echo "========================================================================"
$CROSS -O2 -o "demo_cross" demo.c -lm || fail "交叉编译失败"
printf "  %-14s %-22s %-12s %s\n" "文件" "ELF Machine 字段" "代码段" "文件大小"
for pair in "demo_O2:本机" "demo_cross:目标"; do
    f=${pair%%:*}; tag=${pair##*:}
    printf "  %-14s %-22s %-12s %s\n" "$tag($f)" \
        "$(readelf -h $f | grep Machine | sed 's/.*: *//')" \
        "$(size -d $f | tail -1 | awk '{print $1}')" \
        "$(stat -c %s $f)"
done
m1=$(readelf -h demo_O2 | grep Machine | sed 's/.*: *//')
m2=$(readelf -h demo_cross | grep Machine | sed 's/.*: *//')
[ "$m1" != "$m2" ] || fail "两个架构的 Machine 字段不该相同（都是 $m1）"
ok "Machine 字段不同：$m1  vs  $m2"

# ---------------------------------------------------------------------------
echo
echo "========================================================================"
echo "④ 把目标架构的程序拿回本机跑 —— 这是本实验的重点"
echo "========================================================================"
set +e
out=$(./demo_cross 2>&1); rc=$?
set -e
echo "  \$ ./demo_cross"
echo "      $out"
echo "      退出码 = $rc"
[ $rc -ne 0 ] || fail "跨架构的程序不该能在本机跑起来（退出码 $rc）"
case "$out" in
    *"Exec format error"*|*"cannot execute"*)
        ok "被内核拒绝：Exec format error（退出码 $rc）" ;;
    *)
        ok "被拒绝，报错为：$out（退出码 $rc）" ;;
esac
echo "  ★ 这就是交叉编译的全部理由：**机器码只认一种架构**。"
echo "    所以"在 PC 上编给板子跑"必然要多一整套工具链 —— 它做的事"
echo "    就是换掉 Machine 字段那一栏，以及后面所有的指令编码。"

# ---------------------------------------------------------------------------
echo
echo "========================================================================"
echo "⑤ 目标架构上的静态链接（真正会烧进板子的那种产物）"
echo "========================================================================"
$CROSS -O2 -static -o demo_cross_s demo.c -lm 2>/dev/null || fail "交叉静态链接失败"
printf "  %-22s %-12s %s\n" "产物" "文件(字节)" "上板后需要什么"
printf "  %-22s %-12s %s\n" "交叉动态" "$(stat -c %s demo_cross)" "板子上要有对应的 libc/ld.so"
printf "  %-22s %-12s %s\n" "交叉静态" "$(stat -c %s demo_cross_s)" "什么都不需要"
ok "交叉静态产物 $(stat -c %s demo_cross_s) 字节，自包含"

echo
echo "========================================================================"
echo "全部断言通过。"
echo "========================================================================"
