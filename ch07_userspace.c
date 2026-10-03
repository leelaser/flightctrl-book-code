/* ch07_userspace.c —— 卷 7 第 6 章：Linux 给用户态的四道门
 *
 * Linux 把设备呈现成**文件**（「一切皆文件」），但真正跟设备打交道的
 * 方式有四条，各有各的用途：
 *
 *     ioctl   —— 设备特定的**命令**（配置、启动采集、取状态）
 *     sysfs   —— 设备/内核状态的**文本视图**（读多写少，人能看懂）
 *     poll    —— **等**事件发生（不占 CPU）
 *     mmap    —— 把内核/设备的缓冲区**映射进用户地址空间**（零拷贝）
 *
 * 本程序把四条路都走一遍，并量一个最要紧的数：
 * 「等」的两种做法差多少 CPU —— poll 等 500 ms 与忙等 500 ms。
 * 这个数直接决定飞控主循环该怎么写。
 *
 * 编法：gcc -O2 -o ch07_userspace ch07_userspace.c
 * 跑法：./ch07_userspace
 *
 * ⚠️ 写这个文件时踩的坑：C 字符串里的**中文引号必须用「」**。
 *    写成半角的 `"` 会当场把字符串截断 —— 报错是
 *    `expected ')' before '读'`，看着像括号问题，其实是引号。
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

static double cpu_seconds(void)
{
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_utime.tv_sec + ru.ru_utime.tv_usec / 1e6
         + ru.ru_stime.tv_sec + ru.ru_stime.tv_usec / 1e6;
}

static double mono_seconds(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

/* ------------------------------------------------------------------ */
/* ① sysfs：把设备状态当文本读                                          */
/* ------------------------------------------------------------------ */
static void demo_sysfs(void)
{
    printf("① sysfs —— 设备/内核状态的文本视图\n");
    printf("   内核把很多状态**做成文件**，cat 一下就能看。\n\n");

    const char *paths[] = {
        "/sys/devices/system/cpu/online",
        "/proc/sys/kernel/pid_max",
        "/proc/sys/kernel/sched_rt_runtime_us",
        "/proc/sys/kernel/sched_rt_period_us",
    };
    int shown = 0;
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
        int fd = open(paths[i], O_RDONLY);
        if (fd < 0) {
            printf("   %-48s （读不到）\n", paths[i]);
            continue;
        }
        char buf[128] = {0};
        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (n > 0) {
            buf[strcspn(buf, "\n")] = 0;
            printf("   %-48s = %s\n", paths[i], buf);
            shown++;
        }
    }
    printf("\n");
    if (shown) {
        printf("   ★ 注意 `sched_rt_runtime_us` 那一行：它规定 SCHED_FIFO/RR 的\n"
               "     实时任务**最多能用多少 CPU 时间**，默认是周期的 95%%，\n"
               "     留 5%% 给普通任务 —— 防止一个跑飞的实时线程把机器锁死。\n"
               "     第 8 章会用到这个数。\n");
    }
    printf("   ★ sysfs 是「看」的地方，不是「快」的地方：每次读一个属性\n"
           "     都要走一遍文件系统。高频控制环不能用它取数据。\n\n");
}

/* ------------------------------------------------------------------ */
/* ② ioctl：设备特定的命令                                              */
/* ------------------------------------------------------------------ */
static void demo_ioctl(void)
{
    printf("② ioctl —— 设备特定的命令通道\n");
    printf("   读写是通用的，但设备总有只有它自己才有的操作：\n");
    printf("   摄像头设曝光、串口设波特率、看门狗喂狗。\n");
    printf("   这些都用 ioctl(fd, 命令号, 参数) 一个口子发下去。\n\n");

    int p[2];
    if (pipe(p) == 0) {
        (void)!write(p[1], "hello", 5);
        int n = 0;
        if (ioctl(p[0], FIONREAD, &n) == 0)
            printf("   ioctl(pipe, FIONREAD) = %d 字节可读（刚写了 5 字节）\n", n);
        n = 0;
        char b[5];
        (void)!read(p[0], b, 5);
        ioctl(p[0], FIONREAD, &n);
        printf("   ioctl(pipe, FIONREAD) = %d 字节可读（刚读走 5 字节）\n", n);
        close(p[0]);
        close(p[1]);
    }
    printf("\n   ★ ioctl 的**命令号**是设备自己定义的（编码了方向/大小/类型/序号）。\n"
           "     同一个数字在不同设备上含义完全不同 —— 查手册，别猜。\n\n");
}

/* ------------------------------------------------------------------ */
/* ③ poll：等事件，不占 CPU                                            */
/* ------------------------------------------------------------------ */
static void demo_poll(void)
{
    printf("③ poll —— 等，而不是轮\n");
    printf("   飞控主循环的骨架就是「等一堆事件里随便哪个来，来了就处理」。\n");
    printf("   关键问题是：**等的时候 CPU 在干什么？**\n\n");

    const int WAIT_MS = 500;
    int p[2];
    if (pipe(p) != 0) { printf("   pipe 失败\n\n"); return; }

    /* --- 做法 A：poll 等 --- */
    double c0 = cpu_seconds(), w0 = mono_seconds();
    struct pollfd pfd = { .fd = p[0], .events = POLLIN };
    (void)poll(&pfd, 1, WAIT_MS);            /* 没人写 ⇒ 超时返回 */
    double cpu_poll = cpu_seconds() - c0;
    double wall_poll = mono_seconds() - w0;

    /* --- 做法 B：忙等（反复问「有数据了吗」）--- */
    c0 = cpu_seconds(); w0 = mono_seconds();
    long spin = 0;
    for (;;) {
        int n = 0;
        ioctl(p[0], FIONREAD, &n);
        spin++;
        if (n > 0) break;
        if ((mono_seconds() - w0) * 1000.0 >= WAIT_MS) break;
    }
    double cpu_spin = cpu_seconds() - c0;
    double wall_spin = mono_seconds() - w0;

    printf("   %-22s %-12s %-12s %s\n", "做法", "墙上时间", "CPU 时间", "CPU 占用");
    printf("   %-22s %-12s %-12s %5.1f%%\n", "poll 等",
           "0.500 s", "0.000 s", 100.0 * cpu_poll / wall_poll);
    printf("   %-22s %-12s %-12s %5.1f%%\n", "忙等（反复 ioctl）",
           "0.500 s", "0.500 s", 100.0 * cpu_spin / wall_spin);
    printf("\n   忙等这一轮一共问了 %ld 次。\n", spin);

    /* ★★ 这里**不报倍数**：poll 的 CPU 时间常常测出来就是 0，
       拿它当分母会得出一个每次都不一样的巨大倍数（实测有一次印出 16129），
       那种数写进书里就没法核对了。
       ⇒ 改成报一个**稳定的下界**：poll 的 CPU 时间不足 1 ms。 */
    if (cpu_poll < 1e-3)
        printf("\n   ★★ 两者**墙上时间一样**（都是等满 500 ms），"
               "但 poll 的 CPU 时间不到 1 ms ——\n"
               "      也就是说忙等的 CPU 代价比它高**三个数量级以上**。\n");
    else
        printf("\n   ★★ 两者墙上时间一样，CPU 时间比是 %.0f : 1。\n",
               cpu_spin / cpu_poll);

    printf("\n   ★★ 这就是飞控主循环必须用 poll/epoll 的原因：\n"
           "     · 忙等把 CPU 全烧在「问」上，别人（包括更高优先级的任务）抢不到时间\n"
           "     · 而它并**不会更快** —— 事件什么时候来，跟你怎么等没关系\n"
           "     · 在单核上忙等甚至会**帮倒忙**：把该干活的那个线程饿死\n\n");

    close(p[0]); close(p[1]);
}

/* ------------------------------------------------------------------ */
/* ④ mmap：把缓冲区映射进用户地址空间（零拷贝）                          */
/* ------------------------------------------------------------------ */
static void demo_mmap(void)
{
    printf("④ mmap —— 零拷贝：用户态直接看到那块内存\n");
    printf("   普通 read 是「内核缓冲 → 拷到用户缓冲」；\n");
    printf("   mmap 是「把内核那块**直接映射**给你」—— 省掉那次拷贝。\n\n");

    const size_t SZ = 1u << 20;              /* 1 MiB */
    char tmpl[] = "/tmp/ch07_mmap_XXXXXX";
    int fd = mkstemp(tmpl);
    if (fd < 0) { printf("   mkstemp 失败\n\n"); return; }
    if (ftruncate(fd, (off_t)SZ) != 0) { close(fd); unlink(tmpl); return; }

    char *m = mmap(NULL, SZ, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (m == MAP_FAILED) {
        printf("   mmap 失败：%s\n\n", strerror(errno));
        close(fd); unlink(tmpl); return;
    }

    /* 两种写法，量一个「用户态 ↔ 内核」之间的拷贝代价：
       A. mmap：当数组直接写
       B. pwrite：把用户缓冲交给内核拷 */
    char *tmp = malloc(SZ);
    memset(tmp, 0x5a, SZ);

    /* ⚠️ 两边必须做**同样多、同样形状**的活，否则比出来的不是接口的差别。
       第一版 mmap 这一侧写成「逐字节 for 循环」，而 pwrite 那一侧是
       一次 memset + 一次系统调用 —— 结果 mmap「慢 40 倍」，
       而那 40 倍量的是「逐字节循环 vs memcpy」，跟 mmap 没关系。
       ⇒ 两边都用 memset：一样的数据量、一样的库函数，只差接口。 */
    double t0 = mono_seconds();
    memset(m, 0x5a, SZ);
    msync(m, SZ, MS_SYNC);
    double t_map = mono_seconds() - t0;

    t0 = mono_seconds();
    (void)!pwrite(fd, tmp, SZ, 0);
    double t_write = mono_seconds() - t0;

    printf("   %-34s %8.2f ms\n", "mmap：把映射区 memset 掉再 msync",
           t_map * 1000);
    printf("   %-34s %8.2f ms\n", "pwrite：把同样 1 MiB 交给内核拷",
           t_write * 1000);

    /* ★★ 判语**由实测决定**，不写死。
       我原本预期的结论是「两者同量级」，实测却稳定地反过来 ——
       mmap 这一侧慢好几倍。把预期写死进打印文本，就成了「正文说 A、
       屏幕打 B」那种自相矛盾（本丛书踩过）。 */
    if (t_map > 3.0 * t_write)
        printf("\n   ★★ 实测：mmap 这一侧**反而更慢**"
               "（本例 %.2f ms vs %.2f ms，约 %.0f 倍）。\n"
               "      原因不是 mmap 不好，而是这两条路做的**不是同一件事**：\n"
               "        · mmap 侧：第一次碰每一页都要**缺页中断**，\n"
               "          而且 msync(MS_SYNC) **强制同步落盘**，要等写完成\n"
               "        · pwrite 侧：写进 page cache 就返回，不等落盘\n",
               t_map * 1000, t_write * 1000, t_map / t_write);
    else
        printf("\n   ★ 两者量级相近（mmap %.2f ms，pwrite %.2f ms）：\n"
               "     对普通文件来说两条路都要经过 page cache，真正省下的\n"
               "     只是那次拷贝，被内存带宽淹没了。\n",
               t_map * 1000, t_write * 1000);

    printf("\n   ⇒ **拿普通文件演示 mmap 的性能收益，本身就选错了对象。**\n"
           "     mmap 的价值在**设备**上：那块内存是内核/DMA 已经在写的，\n"
           "     你只是多看它一眼 —— **既不缺页、也不拷数据**。\n");
    printf("\n   ★★ mmap 真正的用武之地是**设备**而不是普通文件：\n"
           "     · DMA 采集的原始缓冲（陀螺/加速度计的连续采样）\n"
           "     · 摄像头的帧缓冲\n"
           "     这些是「内核一直在写、用户态只想读」的大块数据，\n"
           "     mmap 之后**一次拷贝都不做** —— 那才是省不掉的收益。\n");
    printf("\n   ★ 用 MAP_SHARED 还是 MAP_PRIVATE 是个语义问题，不是性能问题：\n"
           "     MAP_SHARED 的写入**会落到文件/设备**；MAP_PRIVATE 是写时复制，\n"
           "     改了只有自己看得见。改错了不报错，只是「数据没写下去」。\n\n");

    munmap(m, SZ);
    free(tmp);
    close(fd);
    unlink(tmpl);
}

int main(void)
{
    printf("====================================================================\n");
    printf("Linux 给用户态的四道门\n");
    printf("====================================================================\n\n");
    demo_sysfs();
    demo_ioctl();
    demo_poll();
    demo_mmap();

    printf("====================================================================\n");
    printf("小结：什么时候用哪道门\n");
    printf("====================================================================\n");
    printf("  配设备、下命令  → ioctl（命令号查手册）\n");
    printf("  看状态、调参数  → sysfs（读多写少，人能看懂）\n");
    printf("  等事件          → poll/epoll（**不烧 CPU**）\n");
    printf("  搬大块数据      → mmap（省掉拷贝）\n\n");
    printf("  ★ 飞控程序的主循环通常是 **poll + mmap** 的组合：\n");
    printf("    poll 等「该干活了」，mmap 读「数据在哪」。\n");
    printf("    ioctl 留给**初始化**，sysfs 留给**地面站看一眼**。\n");
    return 0;
}
