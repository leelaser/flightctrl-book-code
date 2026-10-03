#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lesson25_iterate.py —— 第 25 周 03 课《控制律迭代方法论》配套演示

    实验 1  原则一「问」    ：不解锁，问飞控为什么
    实验 2  原则四「量具自证」：编译目标数 vs 是否真重编
    实验 3  原则五「采样率」 ：同一段飞行，不同采样率算出的 max 差多少
    实验 4  原则六「一次一变量」：升降副翼反向的三组枚举
    实验 5  原则七「闭环」   ：重复性 —— 没测噪声就别谈对比

只用标准库（实验 1 需要 SITL 在跑，其余可离线）。
运行：
    python lesson25_iterate.py            # 全部（能跑的跑，不能跑的说明原因）
    python lesson25_iterate.py 3          # 只跑实验 3
"""

import os
import sys
import csv
import glob
import math
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SITL_DIR = os.path.abspath(os.path.join(
    HERE, "..", "..", "03b-机型动力学", "code", "sitl_delta_wing"))
RESULTS = os.path.join(SITL_DIR, "iterate", "results")


def banner(t):
    print()
    print("═" * 74)
    print("  " + t)
    print("═" * 74)


def sub(t):
    print()
    print("── " + t)
    print("─" * 74)


# ═════════════════════════════════════════════════════════════════════
# 实验 1 —— 原则一：把「猜」换成「问」
# ═════════════════════════════════════════════════════════════════════

def exp1_ask():
    banner("实验 1 · 原则一：把「猜」换成「问」")
    print("""
   ★★ 场景：解锁失败。你的直觉可能是「加个参数跳过检查」——
      但那是【猜】。飞控其实会告诉你原因。

   ★★★ 做法：主动触发 pre-arm 检查，然后听 STATUSTEXT。
""")
    try:
        from pymavlink import mavutil
    except ImportError:
        print("  ⚠️ 未安装 pymavlink，本实验跳过")
        print("     安装：pip3 install pymavlink")
        return

    try:
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=8)
        m.wait_heartbeat(timeout=8)
    except Exception as e:
        print("  ⚠️ 连不上 SITL（%s）" % type(e).__name__)
        print("     先启动 SITL 再跑本实验。启动方法见 03b-06 课。")
        return

    print("  ✓ 已连接 SITL")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    sub("不问，先看飞控自己刷什么（按 severity 分类）")
    buckets = {}
    t0 = time.time()
    while time.time() - t0 < 6:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg:
            buckets.setdefault(msg.severity, []).append(msg.text)

    names = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
             4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}
    for sev in sorted(buckets):
        items = buckets[sev]
        print("   [%d] %-10s %2d 条   例: %s"
              % (sev, names.get(sev, "?"), len(items), items[0][:48]))

    n_noise = sum(len(v) for k, v in buckets.items() if k >= 5)
    n_real = sum(len(v) for k, v in buckets.items() if k < 5)
    print()
    print("   ▶ 信息类(≥5) %d 条，问题类(≤4) %d 条 —— 这就是为什么要过滤" % (n_noise, n_real))

    sub("★ 主动问：MAV_CMD_RUN_PREARM_CHECKS")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                            0, 0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    got = []
    while time.time() - t0 < 5:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg and msg.severity <= 4:
            got.append(msg.text)
    if got:
        for g in got[:6]:
            print("   [问题] %s" % g)
        print()
        print("   ▶ 这就是【确定的知识】：飞控明说了哪一项没过。")
        print("     拿着这句话去搜源码，比试五种参数快得多。")
    else:
        print("   （本次没有 pre-arm 问题 —— 说明当前配置可解锁）")

    print("""
   ★★★ 原则的一般形式：

       遇到「操作没生效」，先找有没有「查询状态」的接口，
       而不是试不同的操作。

       ┌────────────────┬──────────────────┬─────────────────────────────┐
       │ 你想知道       │ 别做             │ 去做                        │
       ├────────────────┼──────────────────┼─────────────────────────────┤
       │ 为什么不能解锁 │ 反复试解锁       │ RUN_PREARM_CHECKS + STATUS  │
       │ 参数生效没有   │ 假设生效了       │ PARAM_REQUEST_READ 读回来    │
       │ 舵机输出多少   │ 看代码推         │ 听 SERVO_OUTPUT_RAW         │
       │ 飞机在不在飞   │ 看高度猜         │ 听 VFR_HUD.groundspeed      │
       └────────────────┴──────────────────┴─────────────────────────────┘
""")


# ═════════════════════════════════════════════════════════════════════
# 实验 2 —— 原则四：量具必须先自证
# ═════════════════════════════════════════════════════════════════════

def exp2_instrument(ap=None):
    banner("实验 2 · 原则四：量具必须先自证（编译耗时）")
    print("""
   ★★ 场景：要测「改控制律后重编多久」。
      直觉做法：touch 一下文件，然后计时 ── 这是【错的】。
""")
    ap = ap or os.path.expanduser("~/ardupilot")
    if not os.path.isdir(os.path.join(ap, "ArduPlane")):
        print("  ⚠️ 找不到 ArduPilot 树（%s），本实验跳过" % ap)
        print("     用 --ardupilot 指定，或在 WSL 里跑。")
        return

    import subprocess

    def build():
        r = subprocess.run(["./waf", "plane"], cwd=ap,
                           capture_output=True, text=True, timeout=3600)
        n = sum(1 for ln in r.stdout.splitlines() if ln.startswith("["))
        return n, r.returncode

    target = os.path.join(ap, "ArduPlane", "Attitude.cpp")

    sub("A. 用 touch（改 mtime，不改内容）")
    subprocess.run(["touch", target])
    t0 = time.time(); n, rc = build(); dt = time.time() - t0
    print("     编译目标数 = %d   耗时 %.1f s" % (n, dt))
    print("     %s" % ("← 几乎没有编译动作" if n <= 2 else "← 有编译"))

    sub("B. 真改内容（加一行注释）")
    orig = open(target, encoding="utf-8").read()
    try:
        with open(target, "a", encoding="utf-8") as f:
            f.write("\n// __LESSON25_PROBE__\n")
        t0 = time.time(); n2, rc2 = build(); dt2 = time.time() - t0
        print("     编译目标数 = %d   耗时 %.1f s" % (n2, dt2))
    finally:
        with open(target, "w", encoding="utf-8") as f:
            f.write(orig)
        subprocess.run(["grep", "-c", "__LESSON25_PROBE__", target],
                       capture_output=True)
        print("     （探针已还原）")

    print("""
   ★★★ 结论：

        A(touch) 和 B(真改) 的目标数差别巨大 —— 说明
        【waf 用内容签名，不用 mtime】。touch 根本没触发重编。

        如果你只看 A 的结果，会得出「编译只要 %.1f 秒」的错误结论，
        而写进文档。实际改一个 .cpp 是 %.1f 秒。

   ★★ 自证方法：制造一个【必然触发重编】的改动，确认目标数 > 0。
""" % (dt, dt2))


# ═════════════════════════════════════════════════════════════════════
# 实验 3 —— 原则五：采样率要匹配现象的时间尺度（离线，用已有 CSV）
# ═════════════════════════════════════════════════════════════════════

def _load_csv(tag, maneuver="level"):
    p = os.path.join(RESULTS, "%s_%s.csv" % (maneuver, tag))
    if not os.path.isfile(p):
        return None
    with open(p, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _resample(series, dt):
    """把 (t, v) 序列按间隔 dt 抽稀，模拟低采样率。"""
    out, nxt = [], 0.0
    for t, v in series:
        if t >= nxt:
            out.append((t, v)); nxt = t + dt
    return out


def exp3_sampling():
    banner("实验 3 · 原则五：采样率要匹配现象的时间尺度")
    rows = _load_csv("baseline")
    if not rows:
        print("  ⚠️ 找不到存档 %s" % os.path.join(RESULTS, "level_baseline.csv"))
        print("     先在 03b-06 的 iterate/ 下跑一次 --maneuver level --tag baseline")
        return

    def f(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return None

    base = f(rows[0]["alt"])
    series = [(f(r["t"]), (f(r["alt"]) - base) if f(r["alt"]) is not None else None)
              for r in rows]
    series = [(t, a) for t, a in series if t is not None and a is not None]

    print("   同一段飞行记录 %d 帧（逐帧 ~50 Hz），模拟不同采样率：" % len(series))
    print()
    print("     采样间隔   max高度(m)   与逐帧的差   峰值时刻(s)")
    print("   " + "─" * 58)

    full_max = max(a for _, a in series)
    full_t = next(t for t, a in series if a == full_max)

    for dt in (5.0, 2.0, 1.0, 0.5, 0.1):
        picked = []
        nxt = 0.0
        for t, a in series:
            if t >= nxt:
                picked.append((t, a)); nxt = t + dt
        mx = max(a for _, a in picked)
        tmx = next(t for t, a in picked if a == mx)
        flag = "  ← 漏掉峰值" if abs(mx - full_max) > 0.5 else ""
        print("     %5.1f s   %8.2f   %+9.2f   %8.1f%s"
              % (dt, mx, mx - full_max, tmx, flag))

    print()
    print("   ▶ 逐帧真值：max = %.2f m @ t=%.1f s" % (full_max, full_t))

    # 用实测数字说话，不写死结论
    coarse = max(a for t, a in _resample(series, 5.0))
    err = coarse - full_max
    pct = 100.0 * abs(err) / full_max if full_max else 0.0
    print("""
   ★★★ 实测结论：5 秒采样得到 %.2f m，逐帧真值 %.2f m —— 【低估 %.2f m（%.1f%%）】

       注意这个误差【不算大】，因为本次峰值持续了 1~2 秒，5 秒采样侥幸采到了接近值。
       但它足以把「改前 24.46 / 改后 25.00」这样的对比结论【判反】——
       因为两次跑的采样相位不同，误差方向可能相反。

   ★★★ 这个错误在本次实践里【真实发生过】（而且更严重）：
       弹射的作用窗口只有 2 秒，我 5 秒采一次，每次都采在推力【撤掉之后】，
       于是得出了完全错误的结论「弹射根本没生效」，
       而真相是它一直在工作（+15.2 m/s）。

   ★★ 经验法则：采样间隔应使现象被采到 5~10 个点。
       · 弹射（2 s）      → 采 0.2~0.4 s
       · 姿态振荡（几 Hz）→ 采 ≥50 Hz
       · 电池耗尽（几分钟）→ 采 1 s 够了

   ★ 关键区分：
       · max/min 这类【极值型】指标对采样率极敏感
       · mean（均值）相对宽容 —— 但也不能随便降
""" % (coarse, full_max, abs(err), pct))


# ═════════════════════════════════════════════════════════════════════
# 实验 4 —— 原则六：一次只改一个变量
# ═════════════════════════════════════════════════════════════════════

def exp4_onevar():
    banner("实验 4 · 原则六：一次只改一个变量")
    print("""
   ★★ 场景：飞机弹射后速度到 33 m/s，但高度一直是 0.0 m —— 不抬头。

   可能原因有四个：舵面反向、舵面权限不够、地面模型限制、配平不对。

   ★★★ 做法：先挑【离散且可枚举】的那个变量，全试一遍。
""")
    print("     升降副翼反向只有 4 种组合，实测三种：")
    print()
    print("       REV1  REV2   最大高度     最大俯仰    最大地速   结论")
    print("     " + "─" * 66)
    data = [
        (1, 1, 0.0, 0.2, 33.7, "✗ 完全不对"),
        (0, 0, 25.5, 82.2, 22.2, "✓ 能起飞"),
        (1, 0, 12.4, 87.2, 33.7, "△ 半对（滚转轴错）"),
    ]
    for r1, r2, alt, pit, spd, concl in data:
        print("        %d     %d    %+7.1f m   %+7.1f°   %6.1f    %s"
              % (r1, r2, alt, pit, spd, concl))
    print("""
   ★★★ 从这张表能读出三件事：

       1. 0/0 是对的（唯一能起飞的）
       2. 1/0 是「半对」—— 说明两个轴是【独立】的，不是绑在一起的
       3. 1/1 完全不行 —— 说明反向确实影响起飞，不是别的原因

   ★★ 为什么必须一次一个变量：

       如果同时改了【反向】和【增益】，结果变好了 ——
       你没法知道是哪个起作用了，也没法知道另一个是不是其实改错了、
       只是被掩盖了。**那样得到的是「这次的配方」，不是「知识」。**

   ★ 离散变量（反向/模式/通道号）：枚举全部组合，成本低、信息完整。
     连续变量（增益/限幅）：扫掠，但必须先固定其他所有量。
""")


# ═════════════════════════════════════════════════════════════════════
# 实验 5 —— 原则七：先有闭环，再谈优化（重复性）
# ═════════════════════════════════════════════════════════════════════

def exp5_repeat():
    banner("实验 5 · 原则七：重复性 —— 没测噪声就别谈对比")
    fs = sorted(glob.glob(os.path.join(RESULTS, "level_*.json")))
    if not fs:
        print("  ⚠️ 找不到存档目录 %s" % RESULTS)
        return

    import json
    print("   现有存档：")
    print()
    print("     标签                  flew   max_alt    max_spd   frac_flying")
    print("   " + "─" * 66)
    for p in fs:
        try:
            d = json.load(open(p, encoding="utf-8"))
        except Exception:
            continue
        m = d.get("metrics", {})
        print("     %-20s  %-5s  %8s  %9s   %s"
              % (d.get("tag", "?"), m.get("flew"),
                 m.get("max_alt_m"), m.get("max_spd"),
                 m.get("frac_flying")))

    print("""
   ★★★ 这里只有一个配置的多次运行才能算噪声。
       如果上面的标签是不同配置（baseline / demo_pitchlim），
       那它们是【不同条件】，不能用来估噪声。

   ★★ 正确做法：同一配置连跑 3 次，看指标的离散程度。

       若 离散度 > 你要测的改动带来的差异
       ⟹ 这个闭环还【测不出】那个改动
          → 要么降低噪声（固定随机种子 / 跑更久）
          → 要么增大样本（多次取统计量）

   ★ 这一步极容易被跳过：闭环搭好了、能出数了，就直接开始对比改前改后。
     但【不知道噪声水平的对比是没有意义的】——
     你看到的差异可能只是随机波动。

   ★ 噪声可能的来源：
       · EKF 收敛程度每次略不同
       · SITL 的 add_noise() 有随机性
       · MAVLink 到达时刻有抖动
       · 科目起点的 t0 取在收到遥测时，本身有抖动
""")


# ═════════════════════════════════════════════════════════════════════

EXPS = {
    "1": ("原则一：把「猜」换成「问」", exp1_ask),
    "2": ("原则四：量具必须先自证", exp2_instrument),
    "3": ("原则五：采样率要匹配时间尺度", exp3_sampling),
    "4": ("原则六：一次只改一个变量", exp4_onevar),
    "5": ("原则七：先有闭环再谈优化", exp5_repeat),
}


def main():
    print("╔" + "═" * 72 + "╗")
    print("║" + "  lesson25 · 控制律迭代方法论 · 配套演示".ljust(62) + "║")
    print("╚" + "═" * 72 + "╝")
    print()
    for k in sorted(EXPS):
        print("    %s. %s" % (k, EXPS[k][0]))
    print()
    args = sys.argv[1:] or sorted(EXPS)
    for a in args:
        if a in EXPS:
            EXPS[a][1]()
        else:
            print("  !! 未知实验编号：%s" % a)
    print()
    print("═" * 74)
    print("  完成。")
    print("═" * 74)
    print()


if __name__ == "__main__":
    main()
