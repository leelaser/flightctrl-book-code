/* ch07_linux_jitter.c —— 卷 7：Linux 上的唤醒抖动（配套实验）
 *
 * 这个程序做一件事：让一个线程**周期性地按绝对时刻唤醒**，
 * 记录每次"实际醒来时刻 − 期望时刻"，把分布打出来。
 *
 * 它对应卷 4 第 2 章那个 $J = R_{\max} - R_{\min}$ 的抖动定义 ——
 * 只不过在 RTOS 上那是**任务释放抖动**，在 Linux 上成为**唤醒抖动**。
 *
 * ★★ 关于数字的口径（重要）：
 *    这个程序量的是**你这台机器、这个内核、这个负载下**的抖动。
 *    换台机器数字就变，WSL2 / 虚拟机 / 真板子各不一样。
 *    它**不代表"Linux 的抖动是多少"** —— 那种说法不成立。
 *    它要演示的是三件**形状上**成立的事：
 *      ① 抖动分布是**右偏长尾**的：中位数好看，最坏值难看；
 *      ② 提高优先级（SCHED_FIFO）能砍掉**调度器公平性**那部分，
 *         但砍不掉内核不可抢占（PREEMPT_NONE）那部分；
 *      ③ 有干扰时，最坏值涨得比平均值快得多。
 *
 * 编法：gcc -O2 -o ch07_linux_jitter ch07_linux_jitter.c -lpthread -lm
 * 跑法：./ch07_linux_jitter [周期µs] [次数] [干扰线程数]
 *       提高优先级要 sudo（普通用户的 rtprio 上限通常是 0）：
 *       sudo chrt -f 80 ./ch07_linux_jitter 1000 5000 12
 */

#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile int g_run = 1;

/* ------------------------------------------------------------------ */
/* 干扰线程：纯 CPU 忙等，用来模拟"系统里还有别的活要干"           */
/* ------------------------------------------------------------------ */
static void *hog(void *arg)
{
    (void)arg;

    /* ★★ 这一行是本实验最贵的一课，不能删。
     *
     *    第一版没有它，程序**直接挂死**（实测：跑满 120 秒超时被杀）。
     *    原因：**新建线程默认继承创建者的调度策略**。当主线程被
     *    `chrt -f 80` 提到 SCHED_FIFO 之后，它 pthread_create 出来的 12 个
     *    干扰线程**全都是 SCHED_FIFO:80** —— 而 SCHED_FIFO 在**同优先级下
     *    不轮转**（它跑完才让），12 个 CPU 上 12 个 FIFO 干扰线程一起跑，
     *    主线程再也拿不到 CPU，连"该退出了"这个判断都执行不到。
     *
     *    ⇒ 在 Linux 上写实时程序，**必须显式管好每一个线程的策略**：
     *      提优先级时尤其要问一句"我随后创建的那些线程，现在是什么策略？"
     *      这不是理论问题 —— 它让这个实验第一次跑就挂死了。 */
    struct sched_param sp = {0};
    if (sched_setscheduler(0, SCHED_OTHER, &sp) != 0)
        perror("sched_setscheduler(hog)");

    volatile double x = 1.0;
    while (g_run) {
        for (int i = 0; i < 100000; i++)
            x = x * 1.0000001 + 0.0000001;
    }
    return NULL;
}

static int cmp_long(const void *a, const void *b)
{
    long x = *(const long *)a, y = *(const long *)b;
    return (x > y) - (x < y);
}

static long pct(const long *v, int n, double p)
{
    int i = (int)(p * (n - 1));
    if (i < 0) i = 0;
    if (i >= n) i = n - 1;
    return v[i];
}

int main(int argc, char **argv)
{
    long period_us = (argc > 1) ? atol(argv[1]) : 1000;
    int iters      = (argc > 2) ? atoi(argv[2]) : 5000;
    int nhog       = (argc > 3) ? atoi(argv[3]) : 0;
    long period_ns = period_us * 1000L;

    /* 当前调度策略：这是本实验里最重要的一行输出 ——
       SCHED_OTHER 和 SCHED_FIFO 量到的是**两回事**。 */
    int pol = sched_getscheduler(0);
    struct sched_param sp;
    sched_getparam(0, &sp);
    const char *polname = pol == SCHED_FIFO  ? "SCHED_FIFO"
                        : pol == SCHED_RR    ? "SCHED_RR"
                        : pol == SCHED_OTHER ? "SCHED_OTHER" : "?";

    printf("周期 %ld µs，次数 %d，干扰线程 %d\n", period_us, iters, nhog);
    printf("调度策略 %s，优先级 %d，CPU 数 %ld\n",
           polname, sp.sched_priority, sysconf(_SC_NPROCESSORS_ONLN));

    pthread_t *tids = NULL;
    if (nhog > 0) {
        tids = calloc(nhog, sizeof(*tids));
        for (int i = 0; i < nhog; i++)
            pthread_create(&tids[i], NULL, hog, NULL);
    }

    long *jit = malloc(sizeof(long) * iters);
    if (!jit) return 1;

    struct timespec t0, next, now;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    next = t0;
    long overrun = 0;

    for (int i = 0; i < iters; i++) {
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }

        /* ★ 用 TIMER_ABSTIME：每次都瞄**绝对时刻**，误差不累积。
           写成相对睡眠（sleep(period)）会让误差一路累加，
           量到的是"累计漂移"而不是"单次抖动"—— 那是另一个量。 */
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == EINTR)
            ;

        clock_gettime(CLOCK_MONOTONIC, &now);
        long d = (now.tv_sec - next.tv_sec) * 1000000000L
               + (now.tv_nsec - next.tv_nsec);
        if (d < 0) d = 0;          /* 早于期望（不可能，但别让负数污染统计） */
        jit[i] = d;
        if (d > period_ns) overrun++;   /* 晚了一个整周期以上 */
    }

    g_run = 0;
    for (int i = 0; i < nhog; i++) pthread_join(tids[i], NULL);

    long *s = malloc(sizeof(long) * iters);
    memcpy(s, jit, sizeof(long) * iters);
    qsort(s, iters, sizeof(long), cmp_long);

    double sum = 0;
    for (int i = 0; i < iters; i++) sum += s[i];
    double mean = sum / iters;
    double var = 0;
    for (int i = 0; i < iters; i++) var += (s[i] - mean) * (s[i] - mean);
    double sd = sqrt(var / iters);

    printf("\n  抖动（实际 − 期望，单位 µs）\n");
    printf("  最小 %8.1f   中位 %8.1f   平均 %8.1f\n",
           s[0] / 1000.0, pct(s, iters, 0.50) / 1000.0, mean / 1000.0);
    printf("  p99  %8.1f   p99.9 %7.1f   最大 %8.1f   标准差 %7.1f\n",
           pct(s, iters, 0.99) / 1000.0, pct(s, iters, 0.999) / 1000.0,
           s[iters - 1] / 1000.0, sd / 1000.0);
    printf("  超一个周期的次数 %ld / %d\n", overrun, iters);

    /* ★ 这张直方图是本实验的重点：抖动不是正态的，是一条**长尾**。
       只报平均值会把这条尾完全藏掉 —— 而实时性只认那条尾。 */
    printf("\n  分布（每格 10 µs，只画到 p99.9）\n");
    long cap = pct(s, iters, 0.999);
    int nb = (int)(cap / 10000) + 2;
    if (nb > 40) nb = 40;
    int *h = calloc(nb + 1, sizeof(int));
    for (int i = 0; i < iters; i++) {
        int b = (int)(s[i] / 10000);
        if (b > nb) b = nb;
        h[b]++;
    }
    int hmax = 0;
    for (int b = 0; b <= nb; b++) if (h[b] > hmax) hmax = h[b];
    for (int b = 0; b <= nb; b++) {
        if (h[b] == 0) continue;
        printf("  %6d~%-6d µs |%-*s| %d\n", b * 10, b * 10 + 10,
               (int)(40.0 * h[b] / hmax), "",
               h[b]);
    }

    /* ★ 把**原始样本**倒出来给别的脚本用：卷 7 第 9 章要拿这份真实分布
       去做 RTA 的余量分析（"抖动吃掉多少余量"）。用环境变量控制，
       不设就什么都不做 —— 不占位置参数（那三个位置参数已有含义）。 */
    const char *dump = getenv("CH07_DUMP");
    if (dump) {
        FILE *fp = fopen(dump, "w");
        if (fp) {
            for (int i = 0; i < iters; i++)
                fprintf(fp, "%ld\n", jit[i]);
            fclose(fp);
            printf("\n  （原始样本 %d 条已写入 %s）\n", iters, dump);
        } else {
            printf("\n  （写不了 %s：%s）\n", dump, strerror(errno));
        }
    }

    free(tids); free(jit); free(s); free(h);
    return 0;
}

