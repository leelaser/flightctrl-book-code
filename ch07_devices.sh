#!/usr/bin/env bash
# ch07_devices.sh —— 卷 7 第 7 章：设备模型与主次设备号
#
# 用户态看到的是 `/dev/ttyS0` 这样一个**文件名**。
# 内核看到的是另一套东西：**主设备号（major）→ 哪个驱动，
# 次设备号（minor）→ 这个驱动的第几个实例**。
#
# 这个脚本把这条链**解出来**给你看：拿 /dev 里的节点，
# 回到 /proc/devices 里去查它属于哪一类驱动。
#
# 跑法：bash code/ch07_devices.sh
#
# ⚠️ 关于"编一个驱动":  真正加载一个字符设备驱动需要**内核头文件**
#    （/lib/modules/$(uname -r)/build）。很多环境里没有它 ——
#    本脚本最后会**如实报告**这一点，而不是假装能编。
#    能读、能解、能判断的部分，本章全部跑得出来。

set -u

echo "========================================================================"
echo "① /proc/devices：内核里注册了哪些设备驱动"
echo "========================================================================"
if [ ! -r /proc/devices ]; then
    echo "  读不到 /proc/devices"
    exit 1
fi

# 该文件分两段：Character devices / Block devices
n_char=$(awk '/^Character devices:/{f=1;next} /^Block devices:/{f=0} f&&/^ *[0-9]+ /{c++} END{print c+0}' /proc/devices)
n_blk=$(awk '/^Block devices:/{f=1;next} f&&/^ *[0-9]+ /{c++} END{print c+0}' /proc/devices)
echo "  字符设备：$n_char 类"
echo "  块设备  ：$n_blk 类"
echo
echo "  前几类（主设备号 → 驱动名）："
awk '/^Character devices:/{f=1;next} /^Block devices:/{f=0} f&&/^ *[0-9]+ /{print "    " $0; n++} n>=8{exit}' /proc/devices

echo
echo "  ★ 内核里的设备是**按「主设备号」分派**的："
echo "    打开 /dev/xxx 时，内核只看它的主设备号，拿这个号去这张表里"
echo "    找到对应的驱动，然后把请求交给它。**驱动名和 /dev 下的文件名无关**。"
echo "    这就是为什么同一个驱动可以对应好几个 /dev 节点。"

# ---------------------------------------------------------------------------
echo
echo "========================================================================"
echo "② 把 /dev 里的节点解回驱动：主次设备号"
echo "========================================================================"
echo
printf "  %-16s %6s %6s  %s\n" "设备节点" "主号" "次号" "对应哪一类驱动（查 /proc/devices）"

# 从 /proc/devices 建一张 主号→名字 的表
MAJORS=$(awk '/^Character devices:/{f=1;next} /^Block devices:/{t="C";f=0} f&&/^ *[0-9]+ /{printf "%s|CHAR|%s\n", $1, $2}
     /^Block devices:/{b=1;next} b&&/^ *[0-9]+ /{printf "%s|BLK|%s\n", $1, $2}' /proc/devices)

lookup() {
    echo "$MAJORS" | awk -F'|' -v m="$1" '$1==m {print $2 " " $3; exit}'
}

shown=0
for dev in /dev/null /dev/zero /dev/tty /dev/console /dev/random /dev/full; do
    [ -e "$dev" ] || continue
    read -r maj min < <(stat -c '%t %T' "$dev" 2>/dev/null | awk '{printf "%d %d\n", strtonum("0x"$1), strtonum("0x"$2)}' 2>/dev/null)
    [ -z "${maj:-}" ] && continue
    info=$(lookup "$maj")
    printf "  %-16s %6s %6s  %s\n" "$dev" "$maj" "$min" "${info:-（未在本机注册）}"
    shown=$((shown + 1))
done
[ "$shown" -eq 0 ] && echo "  （本机 /dev 下没找到可解析的字符设备节点）"

echo
echo "  ★★ 读法：**主设备号选驱动，次设备号选实例**。"
echo "     · 上面几行的主号都是 1 ⇒ 它们全归同一个驱动（mem 类）管"
echo "     · 次号不同（3 / 5 / 1 / 8 / 9）⇒ 同一个驱动里的不同「功能」"
echo "     这就是「一类驱动，多个设备」的实现方式。"

# ---------------------------------------------------------------------------
echo
echo "========================================================================"
echo "③ 设备模型：总线 / 设备 / 驱动 三个角色"
echo "========================================================================"
echo
echo "  /sys 把这三者的关系**做成了目录结构**："
echo
for p in /sys/bus /sys/class /sys/devices; do
    if [ -d "$p" ]; then
        n=$(ls -1 "$p" 2>/dev/null | wc -l)
        printf "    %-16s %4d 项\n" "$p" "$n"
    fi
done
echo
echo "  三者怎么分工："
echo "    · /sys/bus/<总线>/devices/   这条总线上**发现了哪些设备**"
echo "    · /sys/bus/<总线>/drivers/   有哪些驱动**注册**在这条总线上"
echo "    · 内核做的事：把两边**配对**（靠 compatible 或 总线 id）"
echo "      —— 配上了就调用驱动的 probe()，设备这才「活」过来"
echo
echo "  这一节和第 5 章是同一件事的两半："
echo "    第 5 章讲「设备树怎么告诉内核有这个硬件」，"
echo "    这一节讲「内核把这个硬件交给谁去管」。"
echo
if [ -d /sys/bus ]; then
    echo "  本机几条总线上的设备数："
    for b in platform i2c spi usb; do
        [ -d "/sys/bus/$b/devices" ] || continue
        n=$(ls -1 "/sys/bus/$b/devices" 2>/dev/null | wc -l)
        printf "    %-12s %4d 个设备\n" "$b" "$n"
    done
fi

# ---------------------------------------------------------------------------
echo
echo "========================================================================"
echo "④ 飞控上最常用的两个「用户态直通」"
echo "========================================================================"
echo
echo "  有些总线的设备**不用写内核驱动**也能用 —— 内核提供一个通用驱动，"
echo "  把总线操作直接暴露给用户态："
echo
echo "    · i2c-dev  →  /dev/i2c-N         用户态直接发 I2C 读写"
echo "    · spidev   →  /dev/spidevB.C     用户态直接发 SPI 传输"
echo
printf "  %-14s %s\n" "i2c-dev 设备" "$(ls /dev/i2c-* 2>/dev/null | tr '\n' ' ' | sed 's/^$/（无）/')"
printf "  %-14s %s\n" "spidev 设备" "$(ls /dev/spidev* 2>/dev/null | tr '\n' ' ' | sed 's/^$/（无）/')"
echo
echo "  ★★ 这对飞控很有用：**调试和标定时不必为每颗传感器写驱动**。"
echo "     拿个 Python 脚本配上 i2c-dev，就能读写寄存器、验证数据手册。"
echo "     等到量产再考虑换成内核驱动（换来更低的开销和更好的时序）。"
echo
echo "     代价要说清楚：用户态直通**每次传输都要一次系统调用**，"
echo "     而且**时序由调度器决定**（不像内核驱动那样能贴着中断走）。"
echo "     所以高频、要求确定性的采集要用内核驱动。"

# ---------------------------------------------------------------------------
echo
echo "========================================================================"
echo "⑤ 本机能不能真的编一个驱动？"
echo "========================================================================"
if [ -d "/lib/modules/$(uname -r)/build" ]; then
    echo "  /lib/modules/$(uname -r)/build 存在 ⇒ 可以编外部模块。"
    echo "    make -C /lib/modules/\$(uname -r)/build M=\$PWD modules"
else
    echo "  ❌ /lib/modules/$(uname -r)/build **不存在** ⇒ 没有内核头文件，**编不了模块**。"
    echo
    echo "  ⇒ 本章以「读内核暴露出来的结构」为主 —— 这些全部可复现。"
    echo "     要真编一次驱动，需要："
    echo "       · 与运行内核**完全同版本**的内核源码/头文件"
    echo "       · （交叉编译时）目标架构的工具链"
    echo "     编完之后用 insmod/rmmod 加载，dmesg 里看 probe 的结果。"
fi

echo
echo "========================================================================"
echo "读取完成（只读不写）。"
echo "========================================================================"
