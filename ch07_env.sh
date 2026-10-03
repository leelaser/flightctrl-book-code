#!/usr/bin/env bash
# ch07_env.sh —— 卷 7 第 1 章：读出你这台 Linux 的实时性底子
#
# 本卷的开篇要回答一个问题：**Linux 够不够实时？**
# 这个问题没有统一答案 —— 它取决于**你这台机器怎么配的**。
#
# 所以第一个实验不是跑程序，而是**读配置**：
#   内核的抢占模式、时钟节拍、定时器精度……全都写在 `/proc/config.gz` 里，
#   任何人任何机器上都能读到。数字各不相同，**判读方法一致**。
#
# 跑法：bash code/ch07_env.sh
#
# ★ 这个脚本**不评价你的机器好坏**，它只把"这份配置意味着什么"翻译出来。

set -u

echo "========================================================================"
echo "① 这是什么机器"
echo "========================================================================"
if [ -r /etc/os-release ]; then
    . /etc/os-release
    echo "  发行版     : ${PRETTY_NAME:-未知}"
fi
echo "  内核       : $(uname -r)"
echo "  架构       : $(uname -m)"
echo "  逻辑核数   : $(nproc)"
if [ -r /proc/uptime ]; then
    echo "  已运行     : $(awk '{printf "%.0f 秒", $1}' /proc/uptime)"
fi

echo
echo "========================================================================"
echo "② 实时性相关的内核配置（这是本章的重点）"
echo "========================================================================"
CFG=""
if [ -r /proc/config.gz ]; then
    CFG=$(zcat /proc/config.gz 2>/dev/null)
elif [ -r "/boot/config-$(uname -r)" ]; then
    CFG=$(cat "/boot/config-$(uname -r)")
fi

if [ -z "$CFG" ]; then
    echo "  读不到内核配置（既没有 /proc/config.gz 也没有 /boot/config-*)"
    echo "  ⇒ 这一节跳过。要看这份配置，得在编译内核时打开 CONFIG_IKCONFIG_PROC。"
else
    echo "  配置来源   : $([ -r /proc/config.gz ] && echo /proc/config.gz || echo /boot/config-$(uname -r))"
    echo

    # ---- 抢占模式：本卷最关键的一项 ----
    echo "  —— 抢占模式（决定「内核态能不能被抢占」）——"
    for k in CONFIG_PREEMPT_NONE CONFIG_PREEMPT_VOLUNTARY CONFIG_PREEMPT \
             CONFIG_PREEMPT_RT CONFIG_PREEMPT_DYNAMIC; do
        if echo "$CFG" | grep -q "^$k=y"; then
            case $k in
                CONFIG_PREEMPT_NONE)
                    echo "    ★ $k=y  —— 核内**完全不抢占**。"
                    echo "       一个系统调用只要不主动让出 CPU，就一直占着。"
                    echo "       ⇒ 这是最不适合实时的一档；下面那些延迟就是它造成的。" ;;
                CONFIG_PREEMPT_VOLUNTARY)
                    echo "    ★ $k=y  —— 只在显式让出点（might_sleep 之类）抢占。"
                    echo "       比 NONE 好，仍不满足硬实时。" ;;
                CONFIG_PREEMPT)
                    echo "    ★ $k=y  —— 核内可抢占（低延迟桌面版常用）。"
                    echo "       延迟从毫秒级降到百微秒级，但**仍不是硬实时**。" ;;
                CONFIG_PREEMPT_RT)
                    echo "    ★ $k=y  —— **PREEMPT_RT 实时内核**。"
                    echo "       理论上最坏延迟可到几十微秒 —— 但只有实测才算数。" ;;
                CONFIG_PREEMPT_DYNAMIC)
                    echo "    · $k=y  —— 抢占模式可在**启动时**切换（见下一条目）" ;;
            esac
        fi
    done
    if ! echo "$CFG" | grep -qE '^CONFIG_PREEMPT(_NONE|_VOLUNTARY|_RT)?=y'; then
        echo "    （没有匹配到任何一项 —— 可能是 PREEMPT_DYNAMIC，看 /sys/kernel/debug/sched/preempt）"
    fi

    # ---- 时钟节拍：决定"调度器多久看一次" ----
    echo
    echo "  —— 时钟节拍 HZ（决定调度粒度的时间基准）——"
    hz=$(echo "$CFG" | grep -E '^CONFIG_HZ=[0-9]+' | head -1 | cut -d= -f2)
    [ -n "$hz" ] && {
        echo "    ★ CONFIG_HZ=$hz —— 每秒跳 $hz 次，名义间隔 $((1000 / hz)) ms"
        if [ "$hz" -le 100 ]; then
            echo "       （100 是**服务器/通用**取向：省电、吞吐好，调度粒度粗）"
        elif [ "$hz" -ge 1000 ]; then
            echo "       （1000 是**低延迟**取向，桌面/实时常用）"
        fi
    }
    for k in CONFIG_HZ_100 CONFIG_HZ_250 CONFIG_HZ_300 CONFIG_HZ_1000 CONFIG_NO_HZ_IDLE CONFIG_NO_HZ_FULL CONFIG_HIGH_RES_TIMERS; do
        echo "$CFG" | grep -q "^$k=y" && echo "    · $k=y"
    done

    # ---- 其它与延迟有关的开关 ----
    echo
    echo "  —— 其它与延迟有关的开关 ——"
    for k in CONFIG_SMP CONFIG_RT_GROUP_SCHED CONFIG_IRQ_FORCED_THREADING \
             CONFIG_PREEMPT_NOTIFIERS CONFIG_PREEMPTIRQ_DELAY_TEST; do
        echo "$CFG" | grep -q "^$k=y" && echo "    · $k=y"
    done
fi

echo
echo "========================================================================"
echo "③ 时钟源：量抖动的那把「尺子」准不准"
echo "========================================================================"
if [ -r /sys/devices/system/clocksource/clocksource0/current_clocksource ]; then
    echo "  当前时钟源 : $(cat /sys/devices/system/clocksource/clocksource0/current_clocksource)"
    echo "  可选       : $(cat /sys/devices/system/clocksource/clocksource0/available_clocksource)"
    echo "  ★ 用 tsc/hpet 这类高分辨率时钟源，才有微秒级的「尺子」；"
    echo "    若退到 jiffies（HZ 只有 100 ⇒ 10 ms 一格），量抖动的分辨率就没了。"
else
    echo "  （读不到 clocksource 信息）"
fi

echo
echo "========================================================================"
echo "④ 手边有哪些工具"
echo "========================================================================"
printf "  %-14s %-8s %s\n" "工具" "有没有" "用途"
probe() {
    if command -v "$1" >/dev/null 2>&1; then s="有"; else s="—"; fi
    printf "  %-14s %-8s %s\n" "$1" "$s" "$2"
}
probe gcc        "编译（本机）"
probe aarch64-linux-gnu-gcc "交叉编译（arm64）"
probe python3    "写实验脚本"
probe gdb        "调试"
probe chrt       "改调度策略/优先级（本卷第 8 章要用）"
probe taskset    "绑核（隔离干扰）"
probe cyclictest "业界标准的延迟测量工具（rt-tests 包）"
probe dtc        "设备树编译（第 5 章要用）"
probe cmake      "构建系统"
echo
echo "  ★ 缺工具不影响读书，但**影响你自己复现** —— 上面哪一行是「—」，"
echo "    就说明本卷对应那一节的实验，你要先把它装上。"

echo
echo "========================================================================"
echo "⑤ 一句话总结你这台机器的实时性底子"
echo "========================================================================"
if [ -n "$CFG" ] && echo "$CFG" | grep -q '^CONFIG_PREEMPT_RT=y'; then
    echo "  → PREEMPT_RT 内核。够用来做实时实验，但**永远以实测为准**（第 8 章）。"
elif [ -n "$CFG" ] && echo "$CFG" | grep -q '^CONFIG_PREEMPT=y'; then
    echo "  → 低延迟内核（PREEMPT）。微秒到百微秒量级，达不到硬实时。"
elif [ -n "$CFG" ] && echo "$CFG" | grep -q '^CONFIG_PREEMPT_NONE=y'; then
    echo "  → PREEMPT_NONE：**通用内核，不做实时**。"
    echo "    这不是缺陷 —— 它本来就是为吞吐设计的。第 8 章会量给你看它差在哪。"
else
    echo "  → 判不出来（配置读不到或用了动态抢占）。直接跑第 8 章的抖动实验更实在。"
fi
echo
echo "========================================================================"
echo "读取完成（本脚本只读不写，不改你机器上的任何东西）。"
echo "========================================================================"
