#!/usr/bin/env bash
# ch07_userspace.sh —— 卷 7 第 6 章：编译并跑用户态接口实验
#
# 本章的四道门（ioctl / sysfs / poll / mmap）都在 `ch07_userspace.c` 里，
# 这个脚本负责**编译它、跑它、并检查它真的跑了**。
#
# 跑法：bash code/ch07_userspace.sh
#
# ★ 为什么要包一层：`.c` 本身不是可执行文件，检查器"跑不了源码"。
#   包一层之后，正文里引的那几个数就都能在这段输出里找到 ——
#   这是本丛书对"每个数字都能被复算"的最低要求。

set -u

SRC="$(dirname "$0")/ch07_userspace.c"
BIN=$(mktemp -u /tmp/ch07_userspace.XXXXXX)

if [ ! -r "$SRC" ]; then
    echo "找不到 $SRC"
    exit 1
fi

if ! command -v gcc >/dev/null 2>&1; then
    echo "没有 gcc —— 这个实验需要一个 C 编译器。"
    echo "  Ubuntu/Debian: sudo apt-get install -y build-essential"
    exit 1
fi

if ! gcc -O2 -Wall -o "$BIN" "$SRC" 2>/tmp/ch07_us_build.log; then
    echo "编译失败："
    sed 's/^/    /' /tmp/ch07_us_build.log
    exit 1
fi

"$BIN"
rc=$?
rm -f "$BIN"
exit $rc
