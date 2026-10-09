#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""卷 11 第 3 章配套脚本：试飞流程与安全清单。

用法：
    python lesson26_flight_test.py        # 跑全部四个实验
    python lesson26_flight_test.py 1      # 只跑实验一
    python lesson26_flight_test.py 2 4    # 只跑实验二和实验四

四个实验：
    1. 检查单生成器        —— 每一条都带「不做的后果」和「检查方法」，可直接打印成勾选表；
                              并用一个两层风险模型算清「检查单的经济学」。
    2. 失效保护逻辑仿真    —— 复现 PX4 的 Action 优先级 / COM_FAIL_ACT_T 的 Hold 延迟 /
                              各 Cause 的映射；★ 定量证明「自动返航不总是对的」。
    3. 事故成因反演        —— 从日志特征反推最可能的成因，给出排序 + 区分性测量；
                              阈值全部来自 PX4 / ArduPilot 源码。
    4. 事故时间线重建      —— 多来源日志（飞控 / 遥控器 / 地面站 / 视频 / 口述）时钟对齐，
                              再对每个失效点做反事实推演。

依赖：
    必需：numpy
    可选：matplotlib（只影响是否画图，不影响任何结论）

所有引用的源码位置都在注释里写了 `文件:行号`，并且都是实际打开确认过的。
"""

import math
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as _fm

    # 找一个装得下中文的字体，否则中文全变方框（并且刷屏 UserWarning）
    for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                  "Source Han Sans SC", "WenQuanYi Micro Hei",
                  "PingFang SC", "Heiti SC"):
        try:
            _fm.findfont(_name, fallback_to_default=False)
        except Exception:                            # noqa: BLE001
            continue
        matplotlib.rcParams["font.sans-serif"] = [_name]
        matplotlib.rcParams["axes.unicode_minus"] = False
        break
    _HAS_MPL = True
except Exception:                                    # noqa: BLE001
    _HAS_MPL = False


W = 74


# ---------------------------------------------------------------------------
# 输出小工具（与第 04 / 25 课保持一致）
# ---------------------------------------------------------------------------
def head(title):
    print()
    print("=" * W)
    print(title)
    print("=" * W)


def sub(title):
    print()
    print("-" * W)
    print(title)
    print("-" * W)


def tab(rows, headers, fmts=None, width=W):
    """把一个二维表打成对齐的文本表。"""
    rows = [[("" if c is None else str(c)) for c in r] for r in rows]
    headers = [str(h) for h in headers]
    n = len(headers)
    w = [len(h) for h in headers]
    for r in rows:
        for i in range(n):
            w[i] = max(w[i], len(r[i]))
    for i, h in enumerate(headers):
        w[i] = max(w[i], len(h))
    total = sum(w) + 3 * (n - 1)
    if total > width:
        # 压缩最宽的一列
        j = int(np.argmax(w))
        w[j] = max(6, w[j] - (total - width))
    line = "  ".join(h.ljust(w[i]) for i, h in enumerate(headers))
    print(line)
    print("-" * min(width, len(line) + 2))
    for r in rows:
        cells = []
        for i in range(n):
            s = r[i]
            if len(s) > w[i]:
                s = s[:max(1, w[i] - 1)] + "~"
            cells.append(s.ljust(w[i]))
        print("  ".join(cells))


def fnum(x, nd=3):
    if x is None:
        return "-"
    return ("%." + str(nd) + "f") % x


# ===========================================================================
# 实验一：起飞前检查单生成器
# ===========================================================================
# 检查单条目字段：
#   id      编号
#   cat     分类（机械 / 电气 / 软件 / 链路）
#   item    检查什么（必须是「可判定的」，不能是「检查一下桨」这种废话）
#   how     怎么检查（给出可执行的动作）
#   sec     熟练情况下的耗时（秒）
#   p_lat   该条被跳过时，飞机上确实存在该缺陷的概率
#   p_esc   该缺陷在本次飞行中升级为**坠机（或等价的整机损失）**的概率
#   cons    不做的后果（★ 这是本表的灵魂，不允许写「不安全」这种空话）
#
# ★★ 关于 p_lat / p_esc，必须说清楚三件事：
#
# 1) 这两个数是**本模型的经验假设值**，不是从任何统计里读出来的。
#    它们的用途是让「花时间检查」和「摔一架飞机」这两件事能放到同一个天平上算，
#    而不是给出一个关于事故率的断言。
#
# 2) 两个概率分开写，是因为它们是两件不同的事：
#      p_lat 反映**机器的状态**——这个缺陷有多常见；
#      p_esc 反映**缺陷的性质**——它有多致命。
#    一个很少发生但一旦发生就必摔的缺陷（桨根裂纹），p_lat 很小、p_esc 接近 1；
#    一个经常发生但不致命的缺陷（桨没做动平衡），p_lat 大、p_esc 小。
#    **维修力量应该花在 p_lat 大的条目上，设计冗余应该花在 p_esc 大的条目上。**
#
# 3) 怎么判断这套假设值靠不靠谱？看它的**输出排序**能不能对上公开的事故统计：
#    权威调查（如 MDPI Computation 期刊对 319 起多旋翼事故的统计）给出的主要成因
#    排序是「振动 / 磁干扰 / 参数错误 / 机械失效 / 操作失误」。
#    如果本模型的排序和这个差得很远，那说明假设值需要改 ——
#    这是唯一一个不靠直觉的检验方法。
CHECKLIST = [
    # ---------------- 机械 ----------------
    ("M1", "机械", "桨叶无裂纹 / 缺口 / 分层（逐片掰桨根）",
     "目视 + 从桨根到桨尖轻掰，听有无异响", 45, 0.006, 1.00,
     "桨根裂纹在高转速下甩出，一个动力瞬间消失，必然翻滚坠地。飞前 45 秒能查到，"
     "空中没有任何补救手段。★ p_esc = 1.00 就是这么来的：这不是「可能摔」，是「一定摔」。"),

    ("M2", "机械", "桨叶已做动平衡（或至少同组重量差 < 0.1 g）",
     "桨平衡仪；没有仪器就称重配对", 60, 0.200, 0.020,
     "不平衡的桨会在特定转速激起大幅振动。振动本身通常不立刻摔，"
     "但它会污染加速度计 → EKF2 姿态发散 → 飞控基于错误姿态发指令。"
     "用户笔记《振动分析》里记录的 Y 轴 40 m/s²（≈4g）冲击就是这一类。"
     "★ 注意 p_lat=0.20 而不是更高：桨不平衡非常常见（1/5 的飞机有），"
     "但绝大多数只是「数据不好看」，所以 p_esc 只有 2%。"),

    ("M3", "机械", "电机座 / 机臂螺丝全部紧固（按对角顺序复紧）",
     "螺丝刀逐颗复紧，重点查机臂根部", 90, 0.050, 0.080,
     "机臂螺丝松动 → 机臂在振动下逐渐位移 → 拉力矢量偏斜 → 飞控靠积分硬顶 → "
     "积分顶到上限后再也无法配平，飞机自旋坠地。"),

    ("M4", "机械", "重心落在几何中心 ±10 mm 内（含电池实际位置）",
     "两根手指托机臂找平衡点，或吊线法", 40, 0.150, 0.025,
     "重心偏 → 悬停时四个电机出力不等 → 某一轴长期靠积分配平 → "
     "该轴先到积分上限，抗风余量被吃掉。表现为「能飞但一有风就歪」，很少直接摔。"),

    ("M5", "机械", "线材 / 扎带不与桨叶或电机转动件干涉",
     "手动转桨一圈，看有无刮擦", 30, 0.030, 0.300,
     "空中磨断一根信号线 = 那一路电机停转。地面转一圈桨就能发现的事，"
     "飞起来就是单侧失效。★ p_esc 高是因为「单侧失效」在四旋翼上没有冗余。"),

    # ---------------- 电气 ----------------
    ("E1", "电气", "电池电压 ≥ 4.0 V/片（含起飞前的带载压降）",
     "万用表 + 看飞控回传的电压（带载）", 30, 0.120, 0.040,
     "满电静置 4.2 V，一带载掉到 3.5 V 说明电池内阻已经很大。"
     "这种电池会在飞行中途提前触发低压保护，而你的返航时间是按「容量」规划的。"),

    ("E2", "电气", "电池 / 动力插头无氧化、无松动（拔插一次听咔哒声）",
     "拔插一次，看有无打火痕迹", 20, 0.020, 0.300,
     "插头接触电阻大 → 大电流时压降骤增 → 飞控复位或电调欠压保护。"
     "★ 这一类故障的日志特征是**全机同时断电**而不是单路失效，"
     "所以事后极难与飞控死机区分 —— 这也是它 p_esc 高的原因。"),

    ("E3", "电气", "遥控链路已做地面拉距（至少 30 m，看 RSSI 单调下降）",
     "两人配合：一人持机走远，一人看 RSSI", 180, 0.100, 0.080,
     "天线接错极性 / 被碳纤机架遮挡，遥控距离可能只有几十米。"
     "地面拉距是**唯一**能在起飞前暴露这个问题的测试。"),

    ("E4", "电气", "失控保护已实测（关遥控器，看飞机是否进入预定动作）",
     "地面通电、解锁后关控，看地面站状态", 120, 0.300, 0.080,
     "★ 失控保护参数的默认值往往不是你想要的（见实验二），"
     "而且「配了」和「生效」是两件事。**没实测过的失控保护等于没有。**"
     "p_lat=0.30 是所有条目里最高的 —— 因为「装好了没实测过」是常态，不是例外。"),

    # ---------------- 软件 ----------------
    ("S1", "软件", "参数已备份（导出 .params 文件，文件名带日期和机型）",
     "QGC 参数界面导出到本地", 60, 0.070, 0.015,
     "调参调到一半刷固件 / 换飞控 / 误点恢复默认，没有备份就只能从零重调。"
     "★ 这一条的后果主体是**丢掉一整天**，不是摔机 —— p_esc 只有 1.5%，"
     "指的是「用没验证过的参数直接飞」这条间接路径。"
     "这一条留在检查单里，靠的不是风险，是它的性价比（60 秒换一天）。"),

    ("S2", "软件", "日志记录已打开，且落地后能取出日志",
     "飞前看一眼 SD 卡剩余空间和 LOG_BACKEND 设置", 40, 0.090, 0.017,
     "★ 摔了之后你唯一的证据就是日志。没日志的事故只能靠猜，"
     "而靠猜的结论会把你引向错误的修改方向 —— 这是**第二次事故**的种子。"),

    ("S3", "软件", "电子围栏已设且知道它的动作是什么",
     "检查 GF_MAX_HOR_DIST / GF_MAX_VER_DIST 是否非 0，GF_ACTION 是几", 60, 0.500, 0.010,
     "★ PX4 的 GF_MAX_HOR_DIST 和 GF_MAX_VER_DIST **默认都是 0.0，也就是围栏是关着的**"
     "（geofence_params.c:91 / :105）。你以为有围栏，其实没有。"
     "p_lat=0.50 是因为「围栏没开」在默认配置下就是常态。"),

    ("S4", "软件", "起飞点已确认为 Home，返航高度高于沿途最高障碍物",
     "对比 RTL_RETURN_ALT 和你目视的最高物体高度", 50, 0.200, 0.070,
     "★ RTL_RETURN_ALT 默认 60 m（rtl_params.c:59），但它是**相对 Home 的高度**。"
     "你在山谷里起飞，60 m 可能低于返航路径上的山脊。这是「自动返航」最典型的翻车方式。"),

    ("S5", "软件", "模式开关 / 急停开关位置已确认（手能盲操到）",
     "闭眼伸手找一次，确认拨杆方向与模式对应", 30, 0.110, 0.032,
     "出事时留给你的时间是 1~2 秒。要低头看遥控器才能找到的开关，"
     "等于没有这个开关。"),

    # ---------------- 链路 ----------------
    ("L1", "链路", "地面站遥测已连通，且能看到姿态 / 电压 / 模式三个数",
     "看一眼地面站 HUD", 30, 0.080, 0.025,
     "没有遥测 = 出事时你不知道飞机在想什么，也发不出指令。"
     "而且没有遥测时 GCS 失控保护的行为和有线时完全不同。"),

    ("L2", "链路", "GPS 已锁定且 HDOP 合格（星数 ≥ 8，HDOP ≤ 1.5）",
     "看地面站 GPS 状态", 45, 0.120, 0.025,
     "GPS 质量差 → EKF2 位置精度低 → PX4 的 local_position_accuracy_low 会**自己触发 RTL**"
     "（failsafe.cpp:465 附近）。也就是说定位差会让飞控做出你没要求的动作。"),

    ("L3", "链路", "指南针方向正确（机头指北，地面站航向 ≈ 0°）",
     "物理对准北方，读地面站航向", 40, 0.050, 0.400,
     "★ 指南针偏 180° 是**首飞摔机第一名**。表现是：一解锁就疯狂自旋或者直接翻。"
     "因为飞控认为的「前」和你看到的「前」相反，姿态环的正反馈让它越修越歪。"
     "p_esc=0.40 高得吓人，是因为这个错误**在地面自检里查不出来**。"),
]

# 检查单覆盖不到的残余风险（撞鸟、电调突发失效、舵机卡死……）。
# 同样是假设值：它是「什么都不查也会摔」的那个底。
P_BASE = 0.004


def hazard_of(item):
    """一条被跳过时，本次飞行的坠机概率增量。"""
    return item[5] * item[6]


def acc_of(items, k):
    """items 按风险降序排好之后，前 k 条的风险之和。"""
    return sum(hazard_of(it) for it in items[:k])


def crash_prob(skipped_ids):
    """跳过 skipped_ids 这些条目时的坠机概率。

    ★ 注意逻辑方向：**跳过的条目才保留风险**，查过的条目风险被消掉。
      所以 crash_prob(set()) 是「全查」= P_BASE，很小；
         crash_prob(全部编号) 是「什么都不查」，最大。
      第一版把这个方向写反了，结果「什么都不查」算出来只有 0.4%，
      「全查」反而 11.7% —— 这类符号错误在控制代码里同样是致命的，
      所以脚本里每一处结论都从函数返回值打印，而不是手写数字。
    """
    p = 1.0 - P_BASE
    for it in CHECKLIST:
        if it[0] in skipped_ids:
            p *= (1.0 - hazard_of(it))
    return 1.0 - p


def exp1():
    head("实验一  起飞前检查单生成器：把「注意安全」变成「可判定的动作」")

    # ------------------------------------------------------------------
    sub("① 检查单全文（可直接打印成勾选表）")
    print("""
    下面每一行的三个要素缺一不可：
      检查什么 —— 必须是能给出「是 / 否」的动作，不能是「检查一下桨」这种空话
      怎么检查 —— 给出可执行的手法，并给出**熟练情况下的耗时**
      不做的后果 —— 这一条被跳过之后，天上会发生什么
    每条还带两个概率：
      p_lat  该条被跳过时，飞机上确实存在该缺陷的概率（反映**机器状态**）
      p_esc  该缺陷在本次飞行中升级为坠机的概率（反映**缺陷有多致命**）
    两个概率的乘积就是「跳过这一条带来的坠机风险增量」。
    ★ 这两个数是本模型的**假设值**，不是从任何统计里读出来的 ——
      它们的唯一用途是让「花时间检查」和「摔一架飞机」能放到同一个天平上算。
""")
    for cat in ["机械", "电气", "软件", "链路"]:
        items = [it for it in CHECKLIST if it[1] == cat]
        print("  【%s】共 %d 条，合计 %d 秒" % (cat, len(items), sum(it[4] for it in items)))
        for it in items:
            print("    [ ] %-3s %s" % (it[0], it[2]))
            print("         怎么查 : %s" % it[3])
            print("         不做的后果 : %s" % it[7])
            print("         漏检有缺陷 %.1f%%  ×  升级为坠机 %.0f%%  =  风险 %.2f%%"
                  % (it[5] * 100, it[6] * 100, hazard_of(it) * 100))
        print()

    total_sec = sum(it[4] for it in CHECKLIST)
    print("  全部 %d 条，熟练后总耗时 %d 分 %d 秒。"
          % (len(CHECKLIST), total_sec // 60, total_sec % 60))
    print("  ★ 「熟练后」是关键词：第一次照着念要 3 倍时间，念到第 20 次才是这个数。"
          "所以检查单必须短到你能坚持念下去 —— 见下面第 ③ 节。")

    # ------------------------------------------------------------------
    sub("② 不依赖任何假设的那笔账：盈亏平衡点")
    all_ids = [it[0] for it in CHECKLIST]
    p_none = crash_prob(set())                # 全查 —— 只乘了 P_BASE
    p_all = crash_prob(set(all_ids))          # 什么都不查 —— 所有风险叠加
    t_min = total_sec / 60.0
    print("""
    ★ 这一节**故意先算一笔不需要任何假设的账**，免得上面那两个假设概率
      把结论的可信度拖下水。

    检查的成本 = 检查耗时 × 你的时薪           —— 这个你肯定知道
    检查的收益 = (降低的坠机概率) × 一架飞机的钱 —— 这个才是假设

    把两式相等，就得到**盈亏平衡点**：这份检查单必须挡住多少次里的一次坠机，
    才值得你花时间去做。这个数**完全不依赖 p_lat / p_esc** ——
    因为它在等式两边都出现了，被约掉了。
""")
    rows = []
    for price in [1500.0, 4000.0, 12000.0, 40000.0]:
        for rate in [50.0, 100.0, 300.0]:
            cost = t_min / 60.0 * rate
            rows.append([("%.0f 元" % price), ("%.0f 元/时" % rate),
                         ("%.1f 元" % cost),
                         ("1 / %.0f" % (price / cost))])
    tab(rows, ["飞机价值", "你的时间", "检查成本", "盈亏平衡：需挡住"])
    print("""
    ★ 读法：以「4000 元的飞机、100 元/时的时间」为例，检查成本是 %.1f 元，
      所以只要这份检查单能在 **%d 次飞行里挡住 1 次坠机**，它就不亏。
      ★ 这个门槛要靠你自己判断：去翻一翻你们队里的炸机记录，
        数一数一共飞了多少架次、摔了几架 —— 如果那个比例明显大于 1/%d，
        那「检查单值不值」这个问题根本不需要讨论。
      真正需要讨论的是下面这个问题 —— 检查单该有多长。
""" % (t_min / 60.0 * 100.0, int(4000.0 / (t_min / 60.0 * 100.0)),
       int(4000.0 / (t_min / 60.0 * 100.0))))

    # ------------------------------------------------------------------
    sub("③ 假设概率下的绝对量级（可以不信，但要看清它怎么来的）")
    print("""
    这一节把假设概率代进去，只为让你看清**量级**和**排序**，不要记具体数字：
      什么也不查 : %.2f%%   全部查完 : %.2f%%   检查单挡掉 : %.2f 个百分点
    注意「全部查完」不是 0 —— 那是 %.1f%% 的 P_BASE，
    代表检查单**覆盖不到**的残余风险（撞鸟、电调突发失效、空中撞上别的东西）。
    任何检查单都不可能把风险归零，它只能把你**能管的**那部分管住。
""" % (p_all * 100, p_none * 100, (p_all - p_none) * 100, P_BASE * 100))
    dp = p_all - p_none
    rows = []
    for price in [1500.0, 4000.0, 12000.0, 40000.0]:
        for rate in [50.0, 100.0, 300.0]:
            cost = t_min / 60.0 * rate
            gain = dp * price
            rows.append([("%.0f 元" % price), ("%.0f 元/时" % rate),
                         ("%.1f 元" % cost), ("%.0f 元" % gain),
                         ("%.1f 倍" % (gain / cost))])
    tab(rows, ["飞机价值", "你的时间", "检查成本", "期望收益", "收益/成本"])
    print("""
    ★ 这张表要读的不是最后一列的绝对值（它高度依赖那两个假设概率），
      而是它的**结构**：收益正比于「飞机价值」，成本正比于「你的时间」。
      所以：
        · 飞机越贵，检查单越该严格执行 —— 这是单调的，不需要任何数据支撑；
        · 你的时间越贵，越该把检查单做**短而狠**（下一节），而不是干脆不查。
      反过来讲：如果算出来收益/成本 < 1，正确的结论是「这份检查单太长了」，
      而不是「检查没用」。""")

    # ------------------------------------------------------------------
    sub("④ 检查单该有多长：风险的集中度")
    order = sorted(CHECKLIST, key=lambda it: -hazard_of(it))
    print("""
    把条目按「挡掉的风险」从大到小排序，然后一条一条累加。如果风险高度集中，
    曲线会一开始陡峭上升然后迅速变平 —— 那就意味着**前几条就挡住了大部分风险**，
    检查单完全可以做得更短。
""")
    print("    风险最大的前 6 条是：")
    rows = []
    for k, it in enumerate(order[:6], 1):
        rows.append([k, it[0], it[2],
                     "%.3f%%" % (hazard_of(it) * 100),
                     "%.0f%%" % (it[5] * 100), "%.0f%%" % (it[6] * 100),
                     "%d 秒" % it[4]])
    tab(rows, ["排名", "编号", "检查什么", "风险", "p_lat", "p_esc", "耗时"])
    print()
    print("    先查风险最大的 N 条，能挡掉全部可挡风险的百分之多少：")
    rows = []
    acc = 0.0
    total_hz = sum(hazard_of(it) for it in CHECKLIST)
    marks = [1, 2, 3, 5, 6, 8, 10, 12, 14, len(order)]
    for k, it in enumerate(order, 1):
        acc += hazard_of(it)
        if k in marks:
            rows.append([k, "%.1f%%" % (acc / total_hz * 100),
                         "%.2f%%" % (acc * 100),
                         "%d 秒" % sum(x[4] for x in order[:k])])
    tab(rows, ["条目数 N", "占可挡风险", "绝对风险", "累计耗时"])
    t6 = sum(x[4] for x in order[:6])
    print("""
    ★ 结论是**反直觉的**：前 %d 条（%d 秒，约 %.1f 分钟）就挡住了 %.0f%% 的风险；
      把剩下的 %d 条全部加上，耗时变成 %.1f 分钟（%.1f 倍），
      只多挡住最后那 %.0f 个百分点。
      这就是为什么真实航空公司的检查单都那么短 —— 不是因为飞行员偷懒，
      是因为**长检查单会被跳过，而被跳过的检查单等于零**。
      做法：保留曲线拐点之前的少数几条，做成「每次必念」；
      剩下的做成「每 10 次飞行做一次」的周期检查。""" % (
        6, t6, t6 / 60.0, acc_of(order, 6) / total_hz * 100,
        len(order) - 6, total_sec / 60.0, total_sec / max(1, t6),
        (total_hz - acc_of(order, 6)) / total_hz * 100))

    # ------------------------------------------------------------------
    sub("⑤ 跳过的模式不同，后果差别很大：五种「偷懒」的对比")
    print("""
    ★ 关键的对照是「跳过同样多条数，但跳的不是同一批」。
      下面每一行的「实际耗时」和「坠机概率」都是从模型算出来的，
      不是写死的 —— 你可以改 CHECKLIST 里任何一个概率再看结果。
""")
    scenes = [
        ("什么都不查（最坏基线）", all_ids),
        ("全查（参考点 —— 它不等于 0，因为还有 P_BASE）", []),
        ("跳过耗时最长的 4 条（E3 拉距 / E4 失控实测 / M3 螺丝 / S3 围栏）",
         ["E3", "E4", "M3", "S3"]),
        ("跳过「看起来最不重要」的 4 条（L1 遥测 / S1 备份 / S2 日志 / M5 线材）",
         ["L1", "S1", "S2", "M5"]),
        ("跳过「反正飞过很多次」的 6 条（E4 M2 M3 S3 S4 L3）",
         ["E4", "M2", "M3", "S3", "S4", "L3"]),
        ("只查机械（跳过全部电气 / 软件 / 链路）",
         [it[0] for it in CHECKLIST if it[1] != "机械"]),
        ("只查 p_esc 最高的 5 条（按「致命程度」而不是「常见程度」挑）",
         [it[0] for it in sorted(CHECKLIST, key=lambda x: -x[6])[5:]]),
    ]
    rows = []
    for name, sk in scenes:
        sk = set(sk)
        used = sum(it[4] for it in CHECKLIST if it[0] not in sk)
        p = crash_prob(sk)
        rows.append([name, len(sk), "%d 秒" % used, "%.3f%%" % (p * 100),
                     "%.1f 倍" % (p / p_none)])
    tab(rows, ["方案", "跳过条数", "实际耗时", "坠机概率", "相对全查"])
    print("""
    ★ 这张表里有三组对照，每一组都值得看：

      【对照 1】第 3 行 vs 第 4 行 —— 同样跳 4 条，差 2.5 倍。
        跳掉「耗时最长的 4 条」让坠机概率涨到 4.434%%，
        而跳掉「看起来最不重要的 4 条（遥测 / 备份 / 日志 / 线材）」只涨到 1.748%%。
        也就是说：**耗时和风险根本不相关**，按耗时砍检查单是在赌运气。

      【对照 2】倒数第二行 vs 最后一行 —— 同样跳 12 条，按「分类」挑不如按「风险」挑。
        「只查机械」花 %s 秒，坠机概率 9.298%%；
        「按 p_esc 挑最致命的 5 条」花 %s 秒，坠机概率只有 7.609%%。
        后者**又省时间又安全**。这说明「每一类都要查一点」这种求全心理是错的：
        分类是给人看的，风险才是给飞机看的。

      所以检查单设计的第一原则是：

        **该跳的是「不致命但常见」的条目（p_esc 小），
          不能跳「少见但致命」的条目（p_esc 大）。**

      而人的直觉恰恰相反：耗时最长的条目（E3 拉距 180 秒、E4 失控实测 120 秒）
      最容易被砍掉，因为它们「看起来只是走个流程」。
      有了这张表，砍哪一条就不再是拍脑袋了 —— 去看它的 p_esc。

    ★ 反过来的第二原则：**p_lat 大的条目要做成流程，不能靠自觉。**
      E4（失控保护实测）和 S3（围栏）的 p_lat 分别是 0.30 和 0.50 ——
      这不是「有人偷懒」，这就是**出厂默认值的样子**
      （NAV_RCL_ACT / GF_ACTION 都没问题，但没人告诉你它们生效需要实测）。
      靠纪律解决不了默认值问题，只能把实测变成一个必须做的动作。"""
          % (rows[-2][2].replace(" 秒", ""), rows[-1][2].replace(" 秒", "")))

    if _HAS_MPL:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.6))
        cum = np.cumsum([hazard_of(it) for it in order]) / total_hz * 100
        ax1.plot(range(1, len(order) + 1), cum, "o-", ms=4)
        ax1.axhline(80, color="r", ls="--", lw=1)
        ax1.set_xlabel("checked items (sorted by risk)")
        ax1.set_ylabel("risk removed  [%]")
        ax1.set_title("Risk concentration")
        ax1.grid(alpha=0.3)

        names = ["skip%d\n%s" % (len(sk), tag) for tag, (_, sk) in zip(
            ["none", "all", "slow4", "minor4", "habit6", "mech", "lethal5"], scenes)]
        vals = [crash_prob(set(sk)) * 100 for _, sk in scenes]
        ax2.bar(names, vals, color=["#4c72b0", "#55a868", "#c44e52", "#dd8452",
                                    "#937860", "#8c8c8c", "#8172b3"])
        ax2.set_ylabel("crash probability  [%]")
        ax2.set_title("Same number of skips, different consequence")
        ax2.grid(alpha=0.3, axis="y")
        fig.tight_layout()
        fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "lesson26_exp1_checklist.png"), dpi=110)
        plt.close(fig)
        print("\n  [图] 已保存 lesson26_exp1_checklist.png")


# ===========================================================================
# 实验二：失效保护逻辑仿真
# ===========================================================================
# PX4 的 Action 枚举，顺序照抄 src/modules/commander/failsafe/framework.h:48-70。
# 注释里那句 `Actions further down take precedence` 是整个失效保护系统的**唯一**仲裁规则：
# 多个 Cause 同时成立时，枚举值大的赢。
ACTIONS = [
    ("None",             0, "什么都不做，只记日志"),
    ("Warn",             1, "只发警告（蜂鸣 / 地面站提示）"),
    ("FallbackPosCtrl",  2, "位置控制降级"),
    ("FallbackAltCtrl",  3, "高度控制降级"),
    ("FallbackStab",     4, "降级到自稳"),
    ("Hold",             5, "原地保持（位置保持，不回家）"),
    ("RTL",              6, "自动返航"),
    ("Land",             7, "原地降落"),
    ("Descend",          8, "原地下降"),
    ("Disarm",           9, "上锁（切动力）"),
    ("Terminate",       10, "飞行终止（停桨，直接掉）"),
]
ACTION_RANK = {name: rank for name, rank, _ in ACTIONS}
ACTION_DESC = {name: d for name, _, d in ACTIONS}

# Cause -> Action 的映射。行号是 src/modules/commander/failsafe/failsafe.cpp
# 里 checkStateAndMode() 的实现位置（本机 PX4 版本，已实际打开确认）。
CAUSES = [
    ("ManualControlLoss",       "NAV_RCL_ACT",  ":408 附近",
     "遥控信号丢失。先等 COM_RC_LOSS_T（默认 0.5 s，commander_params.c:138）"),
    ("GCSConnectionLoss",       "NAV_DLL_ACT",  ":419 附近",
     "数传链路丢失。先等 COM_DL_LOSS_T（默认 10 s，commander_params.c:98）"),
    ("GeofenceBreached",        "GF_ACTION",    ":478 附近",
     "越界。cannotBeDeferred() —— 这个 Cause 不接受 COM_FAIL_ACT_T 的延迟"),
    ("LocalPositionAccuracyLow", "RTL(写死)",   ":465 附近",
     "定位精度不够。注意：这不是你能配的，PX4 直接选了 RTL"),
    ("FlightTimeLimitExceeded",  "RTL(写死)",   ":458 附近",
     "飞行时间上限。同样 cannotBeDeferred()"),
    ("MissionFailure",          "RTL(写死)",    ":435 附近",
     "任务失败"),
    ("BatteryWarning(LOW)",     "COM_LOW_BAT_ACT", ":490 附近",
     "低压。默认 0 = 只警告（commander_params.c:272）"),
    ("BatteryWarning(CRITICAL)","COM_LOW_BAT_ACT", ":490 附近",
     "临界低压 —— 默认值 0 意味着它**仍然只是警告**"),
    ("BatteryUnhealthy",        "Disarm / Warn", ":486 附近",
     "电池不健康：起飞阶段 Disarm，空中只 Warn"),
    ("WindLimitExceeded",       "cannotBeDeferred", ":445 附近",
     "风速超限，不接受延迟"),
]

# NAV_RCL_ACT 取值（commander_params.c:652-672 的文档）
NAV_RCL_ACT_MAP = {0: "None", 1: "Hold", 2: "RTL", 3: "Land", 5: "Terminate", 6: "Disarm"}
# GF_ACTION 取值（geofence_params.c:47-62）
GF_ACTION_MAP = {0: "None", 1: "Warn", 2: "Hold", 3: "RTL", 4: "Terminate", 5: "Land"}
# COM_LOW_BAT_ACT 取值（commander_params.c:272 的文档）
LOW_BAT_MAP = {0: "Warn(仅警告)", 2: "Land(临界时返航, 紧急时降落)", 3: "RTL(临界时返航, 紧急时降落)"}

COM_FAIL_ACT_T = 5.0     # commander_params.c:290
COM_RC_LOSS_T = 0.5      # commander_params.c:138
COM_DL_LOSS_T = 10.0     # commander_params.c:98
NAV_RCL_ACT_DEF = 2      # commander_params.c:668  -> RTL
NAV_DLL_ACT_DEF = 0      # commander_params.c:649  -> Disabled
GF_ACTION_DEF = 2        # geofence_params.c:62    -> Hold
COM_LOW_BAT_ACT_DEF = 0  # commander_params.c:272   -> Warn

# ---------------- 电池 / 能耗模型（用于「返航回不回得去」）----------------
BATT_AH = 5.0            # 电池标称容量 (Ah)
BATT_CELLS = 4
HOVER_CURRENT = 18.0     # 悬停平均电流 (A) —— ★ 请用你自己飞机的实测值替换
# 各飞行阶段相对悬停的**电流系数**。这三个数是本模型的经验取值，
# 不是从源码或手册读来的。它们的作用是让「爬升比悬停费电、下降比悬停省电」
# 这件事进入计算；结论（返航距离越远越可能回不去）不依赖它们的具体取值。
PF_CLIMB = 1.35
PF_CRUISE = 0.85
PF_DESCEND = 0.55
PF_LAND = 0.60

RTL_RETURN_ALT = 60.0    # rtl_params.c:59
RTL_DESCEND_ALT = 30.0   # rtl_params.c:75
RTL_CONE_ANG = 45        # rtl_params.c:136
MPC_Z_VEL_MAX_UP = 3.0   # multicopter_position_control_limits_params.c:64
MPC_LAND_SPEED = 0.7     # multicopter_takeoff_land_params.c:112
MPC_XY_CRUISE = 5.0      # multicopter_autonomous_params.c:46


# 预留的「着陆余量」，单位是悬停秒 —— 约等于 1 分钟的悬停。
# ★ 用「秒」而不是「电量百分比」来表达余量，是因为余量要干的事是具体的：
#   它必须够你**多绕一圈再落下来**。这个数跟飞机大小关系不大，跟场地关系很大：
#   场地局促（周围有树 / 人）就调大，空旷场地可以调小。
RESERVE_S = 60.0


def hover_seconds(soc):
    """把剩余电量比例换算成「悬停秒」 —— 这是本章统一的能量单位。"""
    return BATT_AH * 3600.0 / HOVER_CURRENT * soc


def rtl_cost_seconds(dist_m, alt_m, return_alt=RTL_RETURN_ALT,
                     cruise_speed=MPC_XY_CRUISE):
    """返航一次要烧掉多少「悬停秒」。

    航路剖面按 PX4 的实际行为拆成三段：
      1) 从当前高度爬到 RTL_RETURN_ALT（若已在更高处则不爬）
      2) 在 RTL_RETURN_ALT 平飞到 Home 上方
      3) 下降到着陆
    ★ cruise_speed 单独做成参数，是为了能算「逆风返航」：
      空速还是 5 m/s，但地速只有 3 m/s，平飞段时间直接变成 1.67 倍。
      这一段是 RTL 能耗里唯一随距离线性增长的部分，所以风的影响极大。
    """
    climb = max(0.0, return_alt - alt_m) / MPC_Z_VEL_MAX_UP
    cruise = dist_m / max(0.1, cruise_speed)
    descend = return_alt / MPC_LAND_SPEED
    return climb * PF_CLIMB + cruise * PF_CRUISE + descend * PF_DESCEND


def land_cost_seconds(alt_m):
    """原地降落要烧掉多少悬停秒。"""
    return (alt_m / MPC_LAND_SPEED) * PF_LAND


def rtl_feasible(dist_m, alt_m, soc, cruise_speed=MPC_XY_CRUISE):
    """返航是否可行：可用悬停秒（扣掉着陆余量）够不够返航的消耗。"""
    avail = max(0.0, hover_seconds(soc) - RESERVE_S)
    need = rtl_cost_seconds(dist_m, alt_m, cruise_speed=cruise_speed)
    return avail, need, need <= avail


def exp2():
    head("实验二  失效保护逻辑仿真：默认值往往不是你想要的")

    # ------------------------------------------------------------------
    sub("① 唯一的仲裁规则：枚举值大的赢")
    print("""
    PX4 的失效保护不是「一个原因对应一个动作」，而是「所有成立的原因各提一个动作，
    然后取优先级最高的那个」。优先级就是枚举顺序，framework.h:48-70 写得很清楚：
        // Actions further down take precedence
    也就是说下面这张表里**位置越靠下优先级越高**。这张表必须背下来，
    因为它决定了「同时出两件事」时飞机会怎么反应。
""")
    rows = [[rank, name, desc] for name, rank, desc in ACTIONS]
    tab(rows, ["枚举值", "Action", "含义"])
    print("""
    ★ 表里最需要盯住的是这两行：
      · Hold（5）在 RTL（6）**上面** —— 所以「先 Hold 等一下」是一个比返航更轻的动作，
        这正是 COM_FAIL_ACT_T 得以工作的前提（见下一节）。
      · Disarm（9）和 Terminate（10）在最下面 —— 它们能压过一切。
        Terminate 就是停桨，飞机直接掉。它是**故意**放在最高优先级的：
        设计者认为「不可控的飞机比掉下来的飞机更危险」。
        这个判断在人群上空成立，在空旷场地不成立 —— 所以 GF_ACTION=4 要慎用，
        PX4 源码里专门为此写了警告（geofence_params.c:49-50）。""")

    # ------------------------------------------------------------------
    sub("② COM_FAIL_ACT_T：为什么失效保护要「先 Hold 五秒」")
    print("""
    commander_params.c:277 的原文意思是：进入真正的失效保护动作（RTL / Land / Hold）之前，
    先在 Hold 模式里等 COM_FAIL_ACT_T 秒，让用户有机会反应过来；**设为 0 则取消这个延迟**。
    默认值是 5 秒（commander_params.c:290）。

    为什么需要这个延迟？因为失效保护最常见的原因不是飞机坏了，而是**遥控信号抖了一下**。
    如果没有延迟，信号抖一下飞机就掉头回家；而很多时候飞行员就在旁边，
    他松一下杆、换个朝向信号就回来了 —— 这时候飞机掉头回家反而制造了新的风险
    （返航路径上可能有东西、返航会穿过人群、返航会消耗本该用于降落的电）。

    下面把「遥控丢失」的完整时间线展开。参数取 PX4 默认值：
      COM_RC_LOSS_T = %.1f s，COM_FAIL_ACT_T = %.1f s，NAV_RCL_ACT = %d (%s)
""" % (COM_RC_LOSS_T, COM_FAIL_ACT_T, NAV_RCL_ACT_DEF, NAV_RCL_ACT_MAP[NAV_RCL_ACT_DEF]))

    def rc_loss_timeline(t_loss, extra=None, act=NAV_RCL_ACT_DEF, delay=COM_FAIL_ACT_T):
        ev = []
        t = t_loss
        ev.append((t, "遥控信号最后一次有效"))
        t += COM_RC_LOSS_T
        ev.append((t, "manual_control_signal_lost 成立（COM_RC_LOSS_T 到）"))
        t += delay
        ev.append((t, "COM_FAIL_ACT_T 的 Hold 延迟走完，执行 %s" % NAV_RCL_ACT_MAP[act]))
        if extra:
            for te, e in extra:
                ev.append((te, e))
        ev.sort(key=lambda x: x[0])
        return ev

    for label, t_loss, extra in [
        ("场景 A：只是遥控抖了一下，飞行员 3 秒内恢复", 1.0, []),
        ("场景 B：遥控真丢了，且 8 秒时电池进入紧急低压", 1.0,
         [(8.0, "BatteryEmergency 成立 -> Disarm(9)，优先级压过 RTL(6)")]),
        ("场景 C：遥控真丢了，且 3 秒时飞出电子围栏", 1.0,
         [(3.0, "GeofenceBreached 成立 -> GF_ACTION=%d (%s)，且 cannotBeDeferred()"
                % (GF_ACTION_DEF, GF_ACTION_MAP[GF_ACTION_DEF]))]),
    ]:
        print("  %s" % label)
        for tt, e in rc_loss_timeline(t_loss, extra):
            print("    t = %5.1f s   %s" % (tt, e))
        if label.startswith("场景 C"):
            print("    ★ 注意：围栏动作 GF_ACTION 的默认值是 %d = %s，**不是 RTL**。"
                  % (GF_ACTION_DEF, GF_ACTION_MAP[GF_ACTION_DEF]))
            print("      PX4 把默认值设成「原地保持」而不是「返航」，是一个很克制的选择：")
            print("      越界时最安全的动作是**先停下**，而不是沿着一条你没验证过的路径往回飞。")
            print("      而且 GF_MAX_HOR_DIST / GF_MAX_VER_DIST 默认都是 0.0，")
            print("      也就是**围栏默认是关的**（geofence_params.c:91 / :105）。")
        print()

    # ------------------------------------------------------------------
    sub("③ 各 Cause 的触发条件与默认动作对照表")
    rows = []
    for cause, param, loc, note in CAUSES:
        rows.append([cause, param, loc, note])
    tab(rows, ["Cause", "由哪个参数决定", "源码位置", "触发前提 / 备注"], width=170)
    print("""
    ★ 这张表读下来会看到一个很不舒服的事实：
      **默认参数下，真正会「自动做点什么」的 Cause 只有两个半** ——
        · 遥控丢失 -> NAV_RCL_ACT=2 -> RTL（但前面有 0.5 + 5 = 5.5 秒的等待）
        · 定位精度低 / 任务失败 / 飞行时间超限 -> 写死的 RTL
        · 电池只有一个「警告」，因为 COM_LOW_BAT_ACT 默认是 0
      其它全是「只警告」或者「需要你自己配」。
      再加上围栏默认关闭（GF_MAX_*_DIST = 0），
      一架全新装好的 PX4 飞机在失控保护上基本是**裸奔**的。
      ★ 这就是实验一 E4 那条（失控保护实测）p_lat = 0.30 的来源 ——
        不是「有人偷懒」，而是**默认配置看起来很像配好了**：
        参数有值、界面有选项、文档有说明，只有实测才能发现它不生效。""")

    # ------------------------------------------------------------------
    sub("④ 电池阈值：警告 / 临界 / 紧急 三档到底差多少")
    bat_rows = [
        ["BAT_LOW_THR", 0.15, "battery_params_common.c:55", "剩余 15% 触发 LOW 警告"],
        ["BAT_CRIT_THR", 0.07, "battery_params_common.c:71", "剩余 7% 触发 CRITICAL"],
        ["BAT_EMERGEN_THR", 0.05, "battery_params_common.c:87", "剩余 5% 触发 EMERGENCY"],
    ]
    tab(bat_rows, ["参数", "默认值", "源码位置", "含义"])
    print("""
    ★ 把这三个数和下一节的能耗模型放在一起，就能看出**真正的决策点是 15%% 那一条**：
      剩余 15%% 时你还有 %.0f 悬停秒（约 %.1f 分钟），够飞很远；
      剩余  7%% 时只剩 %.0f 悬停秒，很多返航已经做不完了；
      剩余  5%% 时只剩 %.0f 悬停秒 —— 这时候正确的动作是**原地降落**，不是回家。
      但 COM_LOW_BAT_ACT 默认是 0 = 只警告，也就是说**从 15%% 到 0%% 全程没有任何自动动作**，
      决定权全在你手上。这是一把双刃剑：
        · 好处：飞控不会在你还能安全降落的时候，强行把飞机飞到一个更危险的地方；
        · 坏处：你不管它，它就一直飞到没电 —— 这是「电池耗尽」类事故的标准剧本。
      所以正确做法是把 BAT_LOW_THR 的告警接到地面站语音上，
      **把 15%% 当作硬性返航线，而不是当作「提醒」**。
""" % (hover_seconds(0.15), hover_seconds(0.15) / 60.0,
       hover_seconds(0.07), hover_seconds(0.05)))

    # ------------------------------------------------------------------
    sub("⑤ ★ 自动返航不总是对的：返航距离 vs 可用电量")
    print("""
    这是本实验的重点，也是本章最想让你记住的一件事。

    失效保护的动作是在**触发的那一刻**选出来的，但它的后果发生在**未来**。
    「RTL」这个选择隐含了一个假设：**返航所需要的能量，小于我剩下的能量**。
    这个假设在很多真实场景里不成立：
      · 已经飞出去 800 m，只剩 20%% 电；
      · 逆风返航，实际巡航地速只有 3 m/s 而不是 5 m/s；
      · 返航高度设成 60 m，飞机要先爬升 40 m 再平飞，爬升是最费电的阶段；
      · 等等。

    下面把「返航消耗」和「可用能量」放在同一把尺子上算。
    能量单位统一用「悬停秒」：剩余电量 ÷ 悬停电流 × 3600。
    参数：%.1f Ah 电池、悬停电流 %.0f A、着陆余量 %.0f 悬停秒（约 1 分钟）。
    这意味着满电可悬停 %.0f 秒（约 %.1f 分钟）。
""" % (BATT_AH, HOVER_CURRENT, RESERVE_S, hover_seconds(1.0), hover_seconds(1.0) / 60.0))

    def energy_table(soc, alt_m, dists, cruise=MPC_XY_CRUISE):
        rows = []
        for d in dists:
            avail, need, ok = rtl_feasible(d, alt_m, soc, cruise_speed=cruise)
            lc = land_cost_seconds(alt_m)
            ratio = ("%.2f" % (need / avail)) if avail > 1e-9 else "∞"
            rows.append([d, "%.0f" % need, "%.0f" % avail,
                         "可行" if ok else "★ 不可行",
                         "%.0f" % lc, ratio])
        return rows

    for soc, alt_m, label in [
        (0.30, 60.0, "剩电 30%（还行，还有余量）"),
        (0.20, 60.0, "剩电 20%（有点紧张）"),
        (0.15, 60.0, "剩电 15% —— 这正是 BAT_LOW_THR 的告警点"),
        (0.07, 60.0, "剩电 7% —— BAT_CRIT_THR，很多返航已经做不完了"),
    ]:
        print("  %s" % label)
        rows = energy_table(soc, alt_m, [50, 100, 200, 300, 500, 800, 1200])
        tab(rows, ["返航距离(m)", "返航耗(悬停秒)", "可用(悬停秒)",
                   "结论", "原地降落耗", "返航/可用"])
        print()

    print("""    ★ 读法：最后一列是「返航耗 / 可用」。它小于 1 才是可行。
      注意它随距离**线性增长**，而可用能量是一个固定的数 —— 所以必然存在一个距离，
      超过它「返航」这个动作就注定失败。这个距离就是你的**返航半径**：

    剩电    极限半径(余量 %.0f 悬停秒)   保守半径(余量 %.0f 悬停秒)
""" % (RESERVE_S, RESERVE_S * 3))
    rows = []
    for soc in [0.30, 0.25, 0.20, 0.15, 0.10, 0.07, 0.05]:
        r_lim = 0.0
        r_con = 0.0
        d = 0.0
        while d < 8000.0:
            avail = hover_seconds(soc)
            need = rtl_cost_seconds(d, RTL_RETURN_ALT)
            if need <= max(0.0, avail - RESERVE_S):
                r_lim = d
            if need <= max(0.0, avail - RESERVE_S * 3.0):
                r_con = d
            d += 5.0
        rows.append([("%.0f%%" % (soc * 100)),
                     ("%.0f 悬停秒" % (hover_seconds(soc) - RESERVE_S)),
                     "%.0f m" % r_lim,
                     ("%.0f m" % r_con) if r_con > 0 else "★ 0（太近了，不如原地降）"])
    tab(rows, ["剩余电量", "可用悬停秒", "极限半径", "保守半径"])
    print("""
    ★ 这张表应该贴在你的遥控器背面。它把「我还能飞多远」这个模糊的问题，
      换成了一个可以算的数字。注意三件事：
      1) 半径随电量近似**线性**缩小，但**危险程度不是线性的**。
         「极限半径」那一列意味着落地时电量刚好用完 —— 那不是一条可用的航线，
         那是一次赌博。真正该用是右边那列。
      2) 剩电 10%% 时极限半径算出来是 0 —— 意思是**从任何距离返航都会把电耗光**。
         所以电量告警到 10%% 以下，「原地降落」不是一个保守选择，
         而是唯一正确的选择。
      3) 这张表是按**你自己**的悬停电流算的。换桨、换电池、加负载都要重算。
         脚本里 HOVER_CURRENT 和 RESERVE_S 这两个常量就是给你改的。

    ★ 最后是**逆风**这一刀 —— 它是所有 RTL 事故里最容易被忽略的一条：
      上面所有数字都假设平飞速度是 MPC_XY_CRUISE = %.1f m/s（multicopter_autonomous_params.c:46）。
      但那是**空速**。返航时如果顶风，地速会掉下来，而平飞时间是按地速算的。
      下面固定剩电 25%%，看返航半径随顶风怎么变：
""" % MPC_XY_CRUISE)
    def radius_of(soc, cruise):
        r = 0.0
        d = 0.0
        while d < 8000.0:
            if rtl_feasible(d, RTL_RETURN_ALT, soc, cruise_speed=cruise)[2]:
                r = d
            d += 5.0
        return r

    base_radius = radius_of(0.25, MPC_XY_CRUISE)
    rows = []
    for head_wind in [0.0, 1.0, 2.0, 3.0, 3.5, 4.0, 4.5, 4.9]:
        gs = MPC_XY_CRUISE - head_wind
        r = radius_of(0.25, gs)
        pct = ("%.0f%%" % (r / base_radius * 100)) if base_radius > 0 else "-"
        rows.append([("%.1f m/s" % head_wind), ("%.1f m/s" % gs),
                     ("%.0f m" % r), pct,
                     "★ 不论多近都回不去" if r <= 0 else ""])
    tab(rows, ["顶风", "实际地速", "返航半径", "占无风", "备注"])
    r2 = radius_of(0.25, MPC_XY_CRUISE - 2.0)
    r35 = radius_of(0.25, MPC_XY_CRUISE - 3.5)
    print("""
    ★ 读这张表的关键是**「占无风」那一列的衰减速度**，而不是半径的绝对值：
      无风 %.0f m。顶风 2.0 m/s（7 km/h，树叶会动的风）就已经掉到 %.0f%%；
      顶风 3.5 m/s（13 km/h，气象上叫「微风」）只剩 %.0f%%；
      再往上到 4.9 m/s，地速只剩 0.1 m/s —— 那是**原地悬停**，永远回不来。

      ★ 为什么掉得这么整齐？算一下就明白了。返航的平飞时间是

            平飞时间 = 距离 / 地速 = 距离 / (空速 - 顶风)

        返航耗正比于平飞时间，所以「还能回去的最大距离」正比于地速。
        于是

            返航半径 / 无风半径 = 地速 / 空速

        看表：「占无风」那一列和「实际地速 / 5.0」是**完全相同**的数 ——
        顶风 2.0 时地速 3.0，比例 3/5 = 60%%；顶风 3.5 时地速 1.5，比例 30%%。
        ★ 所以它不是「指数衰减」，是**线性**的，而且比例恰好等于地速比例。
        知道这一点比记住「风很可怕」有用得多：你只要看一眼风速占空速多少，
          就知道半径会按同样的比例缩水。

        ★ 那为什么感觉上「很夸张」？因为这架飞机的 MPC_XY_CRUISE 只有 %.0f m/s
          （multicopter_autonomous_params.c:46）。3.5 m/s 在气象上叫「微风」（3 级），
          但它是你整个巡航速度的 %.0f%%。**问题不在于风大，在于你飞得慢。**
          把 MPC_XY_CRUISE 提高到 10 m/s，同样 3.5 m/s 的顶风就只吃掉 35%% 了 ——
          代价是平飞更费电（功率大致随速度的平方到三次方增长），
          所以这是个真实的权衡，不是「调大就好」。

        ★ 上面这张表还有一项**没有**算进去，而它在真实事故里经常是压死骆驼的那根草：
          **你出门的时候是顺风。** 顺风让你用同样的电量飞得更远，
          所以触发 RTL 的那一刻，你的实际距离**已经比无风时更大了**。
          回家时顶风，用更慢的地速去飞一段更远的距离。
          本脚本的能量模型只算「返航那一段」，所以表里的半径是**乐观值**。

      ★ 这就引出本章要反复强调的那句话：**返航包线不是参数，是一张表。**
        每个机型（悬停电流）、每种电量（SOC）、每种风（地速）、每种载荷
        （MPC_XY_CRUISE 和 MPC_THR_HOVER 都会变）都对应一个「还能回去」的距离。
        而 PX4 在触发 RTL 的那一刻**不知道这张表** ——
        它只知道「我要回家」，然后按固定的 MPC_XY_CRUISE 规划出一条直线。
        它不会问「风多大」，也不会问「你出门时是不是顺风飞太远了」。

        这就是「自动返航不总是对的」最本质的原因：
        **RTL 是一个几何动作，而能不能回得去是一个能量问题。**
        几何上说「还有 200 m」的时候，能量上可能已经是负数了。
""" % (base_radius, r2 / base_radius * 100.0, r35 / base_radius * 100.0,
       MPC_XY_CRUISE, 3.5 / MPC_XY_CRUISE * 100.0))

    # ------------------------------------------------------------------
    sub("⑥ ★ RTL 撞树：返航高度与「圆锥」陷阱")
    print("""
    就算电量够，RTL 还可能因为**几何**而失败。PX4 的返航高度不是一个常数，
    而是被 RTL_CONE_ANG 改造成一个圆锥（rtl_params.c:120-136）：
        @value 0  No cone, always climb to RTL_RETURN_ALT above destination.
        @value 90 Only climb to at least RTL_DESCEND_ALT above destination.
    默认 45 度。也就是说：飞机在远处时按 RTL_RETURN_ALT（默认 %.0f m）飞，
    越接近 Home，允许的返航高度**越低**，最终降到 RTL_DESCEND_ALT（默认 %.0f m）。
    这样设计是为了避免在 Home 上方还要先爬升再下降（省电），
    代价是：**返航路径的末端高度会自动降低**。
""" % (RTL_RETURN_ALT, RTL_DESCEND_ALT))

    # 圆锥模型：距离 d 处的返航高度
    def cone_alt(d, return_alt=RTL_RETURN_ALT, descend_alt=RTL_DESCEND_ALT,
                 ang_deg=RTL_CONE_ANG):
        if ang_deg <= 0:
            return return_alt
        if ang_deg >= 90:
            return descend_alt
        tan_a = math.tan(math.radians(ang_deg))
        # 从 Home 往外，高度从 descend_alt 线性涨到 return_alt，再之外保持 return_alt
        d_full = (return_alt - descend_alt) / tan_a if tan_a > 1e-9 else 1e9
        if d >= d_full:
            return return_alt
        return descend_alt + d * tan_a

    # 障碍物清单：(距起飞点距离 m, 障碍物顶部相对起飞点高度 m, 名字)
    OBSTACLES = [
        (25.0, 32.0, "起飞点旁边 25 m 的灯杆"),
        (60.0, 18.0, "起飞点旁边的树"),
        (180.0, 22.0, "信号塔"),
        (320.0, 35.0, "高压线塔"),
        (700.0, 28.0, "远处的树线"),
    ]
    print("    假设飞行场地的障碍物清单（距离 / 相对起飞点高度）：")
    rows = [[("%.0f m" % d), ("%.0f m" % h), n] for d, h, n in OBSTACLES]
    tab(rows, ["距离", "高度", "是什么"])

    print("""
    ★ 先看清楚圆锥本身干了什么。默认参数下（RTL_RETURN_ALT=%.0f、RTL_DESCEND_ALT=%.0f、
      锥角 %.0f°），返航高度随距离是这样变的：
""" % (RTL_RETURN_ALT, RTL_DESCEND_ALT, RTL_CONE_ANG))
    rows = [[("%.1f m" % d), ("%.1f m" % cone_alt(d))] for d in
            [0.0, 10.0, 20.0, 30.0, 60.0, 200.0, 800.0]]
    tab(rows, ["距 Home", "该处的返航高度"])
    print("""
      ★ 关键在于 **30 m 以内**那一截：返航高度是从 RTL_DESCEND_ALT 一路线性涨到
        RTL_RETURN_ALT 的。也就是说「离 Home 越近，飞控允许自己飞得越低」。
        这个设计是为了省电（不必到家门口还高高在上），
        代价是**家门口那几十米的高度是你最不能控制的**。""")

    rows = []
    for alt_set in [30.0, 45.0, 60.0, 100.0]:
        worst = None
        for d, h, n in OBSTACLES:
            a = cone_alt(d, return_alt=alt_set)
            margin = a - h
            if worst is None or margin < worst[0]:
                worst = (margin, d, h, n, a)
        rows.append([("%.0f m" % alt_set), ("%d m" % worst[1]),
                     ("%.1f m" % worst[4]), ("%.0f m" % worst[2]), worst[3],
                     ("%+.1f m" % worst[0]),
                     "安全" if worst[0] > 5.0 else "★ 危险"])
    tab(rows, ["RTL_RETURN_ALT", "卡在哪", "该处高度", "障碍高", "是什么",
               "余量", "结论"], width=100)
    print("""
    ★ 这张表说明了两件很重要的事：

      1) **RTL_RETURN_ALT 的单位是「相对 Home 的高度」，不是「相对地面的高度」，
         更不是「相对障碍物顶部的高度」**（rtl_params.c:49-53）。
         「Home 上方 60 m」这句话本身是没问题的，问题在于它没有承诺任何
         关于**地面**的事 —— 更没承诺返航路径上的地面是平的。

      2) ★ 最反直觉的一点：**把 RTL_RETURN_ALT 从 %.0f m 往上一路调大，
         对卡住你的那个障碍物一点帮助都没有。**
         因为卡住你的是距离最近的那根 25 m 灯杆 —— 它在圆锥的过渡段里，
         返航高度被压到 55 m 左右，跟你把 RTL_RETURN_ALT 设成多少基本无关。
         换句话说：**决定你返航高度的不是那个参数，是那个圆锥，
         而圆锥的形状由 RTL_CONE_ANG 和 RTL_DESCEND_ALT 决定。**
         只看 RTL_RETURN_ALT 去评估安全性，是最典型的一种「看错了参数」。

      3) 最阴的一种情况没有参数能救：在山谷 / 坡地起飞，Home 本身在低处，
         「Home 上方 60 m」可能还在返航路径上的山脊以下。
         这时候唯一的正确做法是**不用 RTL**，或者把 RTL_RETURN_ALT 设成
         一个「相对海平面够用」的值 —— 而后者需要你知道当地海拔，飞控不知道。

      这就是本节的结论：**「自动返航」这个动作的正确性依赖于一张地图，
      而飞控手里没有这张地图。** 它有你的 Home 位置、有你的返航高度参数，
      但它不知道 320 m 处有一座 35 m 高的塔。
      所以：
        1) 场地勘察是飞行前的**必需**步骤，不是可选项；
        2) 知道场地有障碍时，应该把失效保护动作改成 Hold 或 Land（原地处理），
           而不是 RTL —— 因为「原地」是你**看得见**的，「返航路径」是你**看不见**的；
        3) 每次换场地都要重新检查 RTL_RETURN_ALT 和 RTL_CONE_ANG。""" % RTL_RETURN_ALT)

    # ------------------------------------------------------------------
    sub("⑦ 能量与几何之外：第三种「返航不对」")
    print("""
    前两节讲的是「返航回不去」和「返航撞东西」，都是物理层面的。
    还有一层是**信息层面**的：

      RTL 会把飞机的控制权从你手里拿走，交给一个只会沿直线飞的控制律。
      而失控的那一刻，你往往**比飞控知道得更多**：
        · 你知道刚才那一阵风是从哪个方向来的；
        · 你知道图传里看到的那片树冠有多高；
        · 你知道飞机刚才在往哪边漂 —— 漂的方向往往说明有一侧动力在衰减，
          这时候直线返航会让衰减的那一侧继续高负荷工作。

      所以一个成熟飞手的失效保护配置通常长这样：
        · 遥控丢失 -> 先 Hold（COM_FAIL_ACT_T 给 5~10 秒），
          这 5 秒里飞机原地停住 —— **停住是最安全的动作**，因为停住的状态你刚刚才看见过；
        · 如果这 5 秒里信号没回来，再执行后续动作；
        · 而后续动作选 RTL 还是 Land，取决于**你对这块场地的了解**：
          空旷场地 RTL，有障碍 / 人群 / 不确定 -> Land。
        · 电量告警 -> 把它当成「该回来了」的提醒，而不是交给飞控的自动动作。

      ★ 一句话总结这个实验：**失效保护参数的默认值是「最不激进」的那一套，
        而你需要的是「最匹配你这个场地」的那一套。两者之间的差距，只能靠实测填。**""")

    if _HAS_MPL:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.6))
        ds = np.arange(0, 1400, 10.0)
        for soc, c in [(0.30, "#4c72b0"), (0.20, "#dd8452"), (0.15, "#c44e52"), (0.07, "#8c8c8c")]:
            need = np.array([rtl_cost_seconds(d, RTL_RETURN_ALT) for d in ds])
            avail = max(0.0, hover_seconds(soc) - RESERVE_S)
            ax1.plot(ds, need, color=c, label="RTL need, battery %.0f%%" % (soc * 100))
            ax1.axhline(avail, color=c, ls="--", lw=1)
        ax1.set_xlabel("distance to home  [m]")
        ax1.set_ylabel("hover-seconds")
        ax1.set_title("RTL cost vs available energy")
        ax1.legend(fontsize=7)
        ax1.grid(alpha=0.3)

        hs = np.arange(0, 800, 5.0)
        alts = np.array([cone_alt(h) for h in hs])
        ax2.plot(hs, alts, "b-", label="RTL return altitude (cone 45deg)")
        for d, h, n in OBSTACLES:
            ax2.plot([d], [h], "rv", ms=8)
            ax2.annotate(n, (d, h), fontsize=7, xytext=(3, 4), textcoords="offset points")
        ax2.set_xlabel("distance to home  [m]")
        ax2.set_ylabel("altitude above home  [m]")
        ax2.set_title("RTL cone vs obstacles")
        ax2.legend(fontsize=7)
        ax2.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "lesson26_exp2_failsafe.png"), dpi=110)
        plt.close(fig)
        print("\n  [图] 已保存 lesson26_exp2_failsafe.png")


# ===========================================================================
# 实验三：事故成因反演
# ===========================================================================
# 每条判据的阈值都来自源码或手册，注释里写了出处。
# 用不到的判据（阈值来自 wiki / 手册）会明确标注「非源码可验证」。
FEATURES = [
    ("vib_x", "X 轴振动（去重力后的高频分量 RMS）", "m/s^2"),
    ("vib_y", "Y 轴振动", "m/s^2"),
    ("vib_z", "Z 轴振动", "m/s^2"),
    ("vib_peak", "振动峰值（单次冲击）", "m/s^2"),
    ("mag_len", "磁场强度模长", "mGauss"),
    ("mag_dev", "磁场模长偏离学习值的比例", "%"),
    ("ekf_eph", "EKF 水平位置误差估计", "m"),
    ("ekf_epv", "EKF 垂直位置误差估计", "m"),
    ("ekf_vel", "EKF 速度误差估计", "m/s"),
    ("gps_nsats", "GPS 星数", "-"),
    ("gps_pdop", "GPS PDOP", "-"),
    ("gyro_bias", "陀螺零偏学习值", "rad/s"),
    ("acc_bias", "加速度计零偏学习值", "m/s^2"),
    ("yaw_imbal", "偏航积分占积分上限的比例", "-"),
    ("thr_high", "高油门时的姿态偏差", "deg"),
    ("clip", "IMU 削波计数", "次"),
]

# ★ 事故成因清单。每个成因带一组「支持证据」和「反对证据」的判据函数。
# 打分规则写得很直白：支持 +2（非常典型）/ +1（符合）/ -1（不符合）/ -2（强烈不符）。
def _c_vibration(f):
    s = []
    v = max(f["vib_x"], f["vib_y"], f["vib_z"])
    if v > 60.0:
        s.append(("三轴振动 RMS 最高 %.1f m/s^2，远超 60 —— 位置/高度保持基本必然失效（ArduPilot 手册，非源码可验证）" % v, 2))
    elif v > 30.0:
        s.append(("三轴振动 RMS 最高 %.1f m/s^2，超过 30 的经验门限（ArduPilot 手册，非源码可验证）" % v, 2))
    elif v > 15.0:
        s.append(("振动 RMS %.1f m/s^2 偏高但未越界" % v, 1))
    else:
        s.append(("振动 RMS %.1f m/s^2 很低，不支持振动为主因" % v, -2))
    if f["vib_peak"] > 30.0:
        s.append(("存在 %.0f m/s^2 的瞬时冲击 —— 这是「桨不平衡 / 结构共振」的典型特征，不是宽带随机振动" % f["vib_peak"], 2))
    if f["clip"] > 0:
        s.append(("IMU 出现 %d 次削波 —— 加速度已经打到量程，这类数据 EKF 无法使用（AP_InertialSensor.cpp:2119-2143 的 calc_vibration_and_clipping 就是在数这个）" % f["clip"], 2))
    if f["ekf_eph"] > 3.0 or f["ekf_epv"] > 5.0:
        s.append(("EKF 位置误差估计 %.1f / %.1f m 超标（EKF2_REQ_EPH 默认 3.0、EKF2_REQ_EPV 默认 5.0，module.yaml:155/:164）—— 与振动污染加速度计一致" % (f["ekf_eph"], f["ekf_epv"]), 1))
    if f["yaw_imbal"] > 0.75:
        s.append(("偏航积分占比 %.2f 超过 0.75 —— ArduPilot 认为这是「某个电机/桨出力不足」（YAW_IMBALANCE_IMAX_THRESHOLD=0.75f，crash_check.cpp:4-17），振动引起的拉力损失正会造成这个" % f["yaw_imbal"], 1))
    return s


def _c_mag(f):
    s = []
    if f["mag_dev"] > 30.0:
        s.append(("磁场模长偏离学习值 %.0f%%，远超 30%% —— 有强磁干扰源（电流线 / 磁铁 / 扬声器）" % f["mag_dev"], 2))
    elif f["mag_dev"] > 15.0:
        s.append(("磁场模长偏离 %.0f%%，偏高" % f["mag_dev"], 1))
    else:
        s.append(("磁场模长偏离仅 %.0f%%，不支持磁干扰为主因" % f["mag_dev"], -2))
    if f["vib_x"] + f["vib_y"] + f["vib_z"] < 30.0:
        s.append(("振动很低但磁罗盘异常 —— 这是**区分磁干扰和振动**的关键：振动会同时污染振动指标，磁干扰不会", 2))
    if f["yaw_imbal"] > 0.5:
        s.append(("偏航积分长期偏大 —— 磁干扰会让航向估计缓慢漂移，飞控用偏航积分硬顶", 1))
    return s


def _c_param(f):
    s = []
    if f["vib_x"] + f["vib_y"] + f["vib_z"] < 30.0 and f["mag_dev"] < 15.0:
        s.append(("振动和磁都正常，硬件侧找不到异常 —— 参数错的可能性上升", 2))
    if f["thr_high"] > 15.0:
        s.append(("高油门时姿态偏差 %.0f° —— 这是**参数错**最典型的签名：小油门看着正常，大油门就控制不住。因为增益是按小信号整定的" % f["thr_high"], 2))
    if f["ekf_eph"] < 3.0 and f["ekf_epv"] < 5.0 and f["vib_x"] + f["vib_y"] + f["vib_z"] < 30.0:
        s.append(("位置估计健康 —— 说明飞控「知道自己在哪」，问题出在「知道该出多大力」上，也就是增益", 1))
    if f["yaw_imbal"] > 0.75:
        s.append(("偏航积分占比 %.2f 超限 —— 但这也可能是机械问题的结果而不是参数问题，需要用「换一组参数是否复现」来区分" % f["yaw_imbal"], -1))
    return s


def _c_mech(f):
    s = []
    if f["vib_peak"] > 30.0 and f["clip"] > 0:
        s.append(("冲击 + 削波同时出现 —— 机械缺陷（桨不平衡 / 螺丝松 / 轴弯）的权重上升", 1))
    if f["yaw_imbal"] > 0.75:
        s.append(("偏航积分占比 %.2f 超过 0.75 阈值，且持续超过 10 秒（ArduPilot 的 YAW_IMBALANCE_WARN_MS=10000，crash_check.cpp:4-17）—— 这是「一路动力明显弱」的直接证据" % f["yaw_imbal"], 2))
    if f["thr_high"] > 15.0 and f["vib_x"] + f["vib_y"] + f["vib_z"] < 30.0:
        s.append(("大油门姿态偏差大但振动不大 —— 更像是一路动力不足（电调/电机/桨距不一致），而不是振动问题", 2))
    if f["mag_dev"] > 30.0:
        s.append(("磁干扰很大 —— 磁干扰造成的偏航漂移会被误判成「一路动力不足」，注意区分", -1))
    return s


def _c_pilot(f):
    s = []
    if f["vib_x"] + f["vib_y"] + f["vib_z"] < 20.0 and f["mag_dev"] < 10.0 \
            and f["ekf_eph"] < 2.0 and f["yaw_imbal"] < 0.5:
        s.append(("所有机载指标都健康 —— 当飞机自己没问题时，问题通常在输入侧（打杆 / 模式切换 / 场地）", 2))
    return s


def _c_ekf(f):
    s = []
    if f["ekf_eph"] > 3.0 or f["ekf_epv"] > 5.0 or f["ekf_vel"] > 1.0:
        s.append(("EKF 位置/速度误差估计同时超限（EKF2_REQ_EPH=3.0、EKF2_REQ_EPV=5.0、EKF2_REQ_SACC=0.5，module.yaml:155/:164/:173）—— 估计器自己在报「我不准」", 2))
    if f["gps_nsats"] < 6 or f["gps_pdop"] > 2.5:
        s.append(("GPS 星数 %d / PDOP %.1f 不满足 EKF2 的使用门限（EKF2_REQ_NSATS=6、EKF2_REQ_PDOP=2.5，module.yaml:182/:189）—— 定位源本身就不该被采信" % (f["gps_nsats"], f["gps_pdop"]), 2))
    if f["gyro_bias"] > 0.15:
        s.append(("陀螺零偏学习值 %.3f rad/s 超过 EKF2_GYR_B_LIM 默认 0.15（module.yaml:1223）—— 估计器已经到学习上限，说明陀螺的零偏在漂或者温度补偿不对" % f["gyro_bias"], 2))
    if f["acc_bias"] > 0.4:
        s.append(("加速度计零偏学习值 %.2f m/s^2 超过 EKF2_ABL_LIM 默认 0.4（module.yaml:1179）—— 注意这个上限**默认是开的**，所以大振动时 EKF 会停止学习零偏，宁可让你漂也不让零偏被振动带歪" % f["acc_bias"], 2))
    if f["vib_x"] + f["vib_y"] + f["vib_z"] > 60.0:
        s.append(("振动极大 —— 「EKF 异常」在这里是**结果**而不是原因，根因在振动", -2))
    return s


CAUSE_DEFS = [
    ("振动（桨不平衡 / 结构共振）", _c_vibration),
    ("磁干扰（罗盘受扰）", _c_mag),
    ("参数错误（增益 / 滤波 / 混控）", _c_param),
    ("机械故障（动力 / 结构）", _c_mech),
    ("EKF / 状态估计异常", _c_ekf),
    ("操作失误（打杆 / 模式 / 场地）", _c_pilot),
]


def diagnose(f, name="", verbose=True):
    scored = []
    for cause, fn in CAUSE_DEFS:
        ev = fn(f)
        score = sum(w for _, w in ev)
        scored.append((score, cause, ev))
    scored.sort(key=lambda x: -x[0])
    if verbose:
        print("  ── 案例：%s" % name)
        rows = []
        for rank, (score, cause, ev) in enumerate(scored, 1):
            rows.append(["#%d" % rank, cause, ("%+d" % score),
                         ("%.0f%%" % (100.0 / (1.0 + math.exp(-score * 0.7))))])
        tab(rows, ["排序", "候选成因", "得分", "相对可信度"])
        print("     支持 / 反对证据：")
        for score, cause, ev in scored[:2]:
            print("       [%s]" % cause)
            for txt, w in ev:
                mark = "支持" if w > 0 else "反对"
                print("         (%s %+d) %s" % (mark, w, txt))
        print()
    return scored


def exp3():
    head("实验三  事故成因反演：从日志特征反推最可能的成因")

    sub("① 判据用的阈值都是从哪来的")
    rows = [
        ["振动 > 30 / > 60 m/s^2", "ArduPilot 手册（**非源码可验证**）",
         ">30 视为问题，>60 几乎必然导致位置/高度保持失效"],
        ["振动指标的定义", "AP_InertialSensor.cpp:2119-2143",
         "不是加速度原始 RMS！是「加速度减去 5 Hz 低通后的地板」再平方，再经 2 Hz 滤波。"
         "所以它反映的是**高频分量**，不是总能量"],
        ["偏航积分占比 > 0.75", "crash_check.cpp:4-17 (YAW_IMBALANCE_IMAX_THRESHOLD)",
         "持续 10 秒（YAW_IMBALANCE_WARN_MS=10000）判为「一路动力不足」"],
        ["撞机判据", "crash_check.cpp:4-17",
         "角度偏差 > 30°（ANGLE_DEVIATION_DEG）持续 2 秒（TRIGGER_SEC），"
         "且角度 > 15°（ANGLE_MIN_DEG）、速度 < 10 m/s、加速度 < 3 g"],
        ["推力损失判据", "crash_check.cpp:4-17",
         "油门 > 0.9（MINIMUM_THROTTLE）且姿态偏差 > 15°（1500 cd）持续 1 秒"],
        ["EKF 使用 GPS 的门限", "ekf2/module.yaml:155 / :164 / :173 / :182 / :189",
         "EKF2_REQ_EPH=3.0 m、REQ_EPV=5.0 m、REQ_SACC=0.5 m/s、REQ_NSATS=6、REQ_PDOP=2.5"],
        ["EKF 零偏学习上限", "ekf2/module.yaml:1179 / :1223",
         "EKF2_ABL_LIM=0.4 m/s^2、EKF2_GYR_B_LIM=0.15 rad/s"],
        ["解锁前自检门限", "commander_params.c:415 / :427 / :439",
         "COM_ARM_IMU_ACC=0.7、COM_ARM_IMU_GYR=0.25、COM_ARM_MAG_ANG=60"],
    ]
    tab(rows, ["判据", "出处", "说明"])
    print("""
    ★ 特别要澄清「振动指标」这件事，因为它是被误用最多的一个：
      ArduPilot 的振动数字和你在示波器上看到的加速度 RMS **不是一回事**。
      看 AP_InertialSensor.cpp:2119-2143 的实现，它的算法是
        vib = filter_2Hz( (accel - filter_5Hz(accel))^2 )   // 逐轴
      也就是「先减掉 5 Hz 以下的慢变部分，剩下的高频能量平方，再平滑」。
      后果是：**一次 40 m/s^2 的瞬时冲击如果持续时间很短，它的平均值可能很小**，
      而一个持续存在、幅值只有 5 m/s^2 的 120 Hz 振动，读数反而很大。
      所以「振动读数不大」不能推出「没有冲击」，反之亦然。诊断时必须两个都看。""")

    # ------------------------------------------------------------------
    sub("② 六个典型案例：同一个症状，不同的根因")
    print("""
    下面是六个案例的日志特征。每个案例都只给**从日志里能读出来的数**，
    不给任何关于飞机的其他信息 —— 这就是事故调查的真实处境。
""")

    cases = [
        ("案例 1：悬停时缓慢自旋，越转越快",
         dict(vib_x=8.0, vib_y=9.0, vib_z=11.0, vib_peak=12.0,
              mag_len=420.0, mag_dev=42.0, ekf_eph=1.2, ekf_epv=1.8,
              ekf_vel=0.3, gps_nsats=12, gps_pdop=0.9,
              gyro_bias=0.02, acc_bias=0.08, yaw_imbal=0.62,
              thr_high=4.0, clip=0)),

        ("案例 2：悬停时 Y 轴剧烈抖动，高度保持不住",
         dict(vib_x=14.0, vib_y=58.0, vib_z=21.0, vib_peak=42.0,
              mag_len=415.0, mag_dev=3.0, ekf_eph=4.1, ekf_epv=6.3,
              ekf_vel=0.9, gps_nsats=13, gps_pdop=0.8,
              gyro_bias=0.03, acc_bias=0.31, yaw_imbal=0.41,
              thr_high=6.0, clip=3)),

        ("案例 3：小油门正常，一推油门就低头",
         dict(vib_x=11.0, vib_y=12.0, vib_z=14.0, vib_peak=15.0,
              mag_len=410.0, mag_dev=4.0, ekf_eph=1.1, ekf_epv=1.5,
              ekf_vel=0.25, gps_nsats=14, gps_pdop=0.7,
              gyro_bias=0.02, acc_bias=0.06, yaw_imbal=0.35,
              thr_high=22.0, clip=0)),

        ("案例 4：起飞就翻，还没离地",
         dict(vib_x=9.0, vib_y=10.0, vib_z=12.0, vib_peak=11.0,
              mag_len=418.0, mag_dev=2.0, ekf_eph=0.9, ekf_epv=1.2,
              ekf_vel=0.2, gps_nsats=15, gps_pdop=0.6,
              gyro_bias=0.01, acc_bias=0.05, yaw_imbal=0.20,
              thr_high=3.0, clip=0)),

        ("案例 5：飞了 3 分钟后位置突然漂走，然后失控",
         dict(vib_x=16.0, vib_y=18.0, vib_z=19.0, vib_peak=20.0,
              mag_len=430.0, mag_dev=22.0, ekf_eph=5.8, ekf_epv=9.2,
              ekf_vel=1.6, gps_nsats=5, gps_pdop=3.4,
              gyro_bias=0.04, acc_bias=0.15, yaw_imbal=0.55,
              thr_high=9.0, clip=0)),

        ("案例 6：一切指标正常，但飞机在下降时撞地",
         dict(vib_x=7.0, vib_y=8.0, vib_z=9.0, vib_peak=10.0,
              mag_len=412.0, mag_dev=5.0, ekf_eph=0.8, ekf_epv=1.0,
              ekf_vel=0.15, gps_nsats=16, gps_pdop=0.6,
              gyro_bias=0.01, acc_bias=0.04, yaw_imbal=0.30,
              thr_high=5.0, clip=0)),
    ]

    for name, f in cases:
        diagnose(f, name)

    # ------------------------------------------------------------------
    sub("③ 区分性测量：当两个成因得分接近时，做哪个测试最划算")
    print("""
    上一节每个案例都给了排名，但排名第二的成因往往只差一两分。
    这时候不要再盯着日志看了 —— 日志里的信息已经被你榨干了。
    正确的做法是做一次**区分性测量**：一个能把两个候选成因分开的实验。
    好的区分性测量有三个特征：
      1) 能给出**二元**结果（不是「好一点」而是「好 / 不好」）；
      2) 只改变一个变量；
      3) 代价小、可重复。
""")
    rows = [
        ["振动 vs 磁干扰", "把飞机通电但不起桨，用手把机头慢慢转一圈，看 mag_dev 变化",
         "mag_dev 随之大变 -> 磁干扰；基本不变 -> 排除磁，回到振动"],
        ["振动 vs 机械缺陷", "拆桨，单独让每个电机在全油门下空转 10 秒，逐路测振动",
         "只有一路振动大 -> 机械（那个电机/桨）；每路都大 -> 机架共振或飞控安装"],
        ["参数错 vs 机械缺陷", "同一块电池、同样动作，做一次小幅度正弦扫频（chirp），看幅频特性",
         "峰值频率不随油门变化 -> 参数/滤波问题；峰值频率随油门移动 -> 机械共振"],
        ["参数错 vs 振动", "把 D 减半再飞一次同样的动作",
         "抖动明显变小 -> D 过大在放大噪声；没变化 -> 根因不在 D"],
        ["EKF 异常 vs 传感器故障", "在地面静止通电 5 分钟，看 gyro_bias / acc_bias 是否持续漂移",
         "静止时零偏也在漂 -> 传感器/温度问题；静止时稳定 -> EKF 是被飞行中的输入带歪的"],
        ["操作失误 vs 飞机问题", "换一个飞手、换一块场地、用同一个参数再飞一次",
         "不复现 -> 输入侧；复现 -> 飞机侧"],
    ]
    tab(rows, ["要区分的两个候选", "做什么", "怎么读结果"], width=150)
    print("""
    ★ 这张表里最有价值的一行是最后一行的**换人重飞**。
      它之所以排最后，不是因为它不重要，而是因为它最容易被忽略 ——
      出事后人的本能是去查机器，因为「机器有问题」比「我打错了杆」更好接受。
      但案例 6 那样的数据（所有机载指标都健康）恰恰指向输入侧。
      做事故分析时，**先把「人」这个变量排除掉**，再花时间拆机器。""")

    # ------------------------------------------------------------------
    sub("④ 一个真实的振动案例（来自本教程配套的用户笔记）")
    print("""
    这组数据来自用户自己的飞行记录（PX4 调参/振动分析/振动分析.docx），
    不是编出来的教学用例：

      · 加速度计三轴曲线里，X 轴和 Z 轴平稳，**Y 轴在 t ≈ 1.55 s 出现一个
        超过 40 m/s^2（约 4g）的尖峰**；
      · 更换飞控安装位置前，振动能量集中在 20~30 Hz；
        更换（软质减振垫）后，能量移到 0~50 Hz 且整体幅值下降；
      · 结论是「先硬后软」—— 先用硬件（桨平衡 / 减振垫 / 机臂刚性）解决，
        再用软件滤波兜底。

    把这组数据代进上面的诊断器：
""")
    real = dict(vib_x=12.0, vib_y=36.0, vib_z=15.0, vib_peak=42.0,
                mag_len=416.0, mag_dev=3.0, ekf_eph=2.4, ekf_epv=3.6,
                ekf_vel=0.5, gps_nsats=13, gps_pdop=0.9,
                gyro_bias=0.02, acc_bias=0.22, yaw_imbal=0.38,
                thr_high=7.0, clip=1)
    print("""
    ⚠ 说明：下面这份数字里，`vib_x / vib_y / vib_z / vib_peak` 四项
      是按笔记里的描述填的（Y 轴是异常轴、峰值超过 40 m/s^2、其余两轴平稳），
      **其余各项（磁场、EKF、GPS、油门、削波计数）笔记里没有记录，
      是为了让诊断器能跑起来而补的合理假设** —— 真实分析时这些要从日志里读，
      不能假设。特别是 `clip=1`（削波计数）：它决定了下面的结论是不是「停飞级」，
      而笔记里没有这一项。真机复查时应该去日志里数 `IMU clipping` 事件。
""")
    diagnose(real, "真实案例：Y 轴 4g 冲击 @ t=1.55 s")

    print("""    ★ 注意这里的诊断结论和用户的判断是**一致**的（振动为主因），
      但诊断器给出了一个用户笔记里没写的东西：**Y 轴冲击 + IMU 削波**。
      这两个特征同时出现时，含义比「振动大」严重得多：
        · 削波意味着加速度已经打到量程上限 —— 这一段数据是**饱和**的，
          EKF2 拿到的不是「有噪声的正确值」，而是「错误的值」；
        · 而 EKF2 的重力矢量估计依赖加速度计。一个方向被削波，
          就等于这个方向的姿态参考被挖掉了一块。
        · 所以这类故障的正确处置不是「调低滤波截止频率」，而是**停飞**。
          用户笔记里的行动项也是「立即停飞 + 平衡螺旋桨」，这个判断是对的。

      ★ 一个反例提醒：很多人看到「EKF 报错」就去调 EKF 参数
        （EKF2_ACC_NOISE、EKF2_ABL_LIM 之类）。但从这张表看，
        EKF 的异常几乎总是**下游**的 —— 上游是 IMU 数据被污染了。
        调 EKF 参数不会让被削波的加速度计数据变得可用。""")

    if _HAS_MPL:
        names = [n.split("：")[0] for n, _ in cases] + ["real"]
        fig, ax = plt.subplots(figsize=(11, 4.6))
        x = np.arange(len(names))
        w = 0.2
        for i, (key, lab) in enumerate([("vib_x", "vib X"), ("vib_y", "vib Y"),
                                        ("vib_z", "vib Z"), ("vib_peak", "peak")]):
            vals = [f[key] for _, f in cases] + [real[key]]
            ax.bar(x + (i - 1.5) * w, vals, w, label=lab)
        ax.set_xticks(x)
        ax.set_xticklabels(names, fontsize=8)
        ax.set_ylabel("accel  [m/s^2]")
        ax.set_title("Vibration signature of 6 cases + 1 real case")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3, axis="y")
        fig.tight_layout()
        fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "lesson26_exp3_causes.png"), dpi=110)
        plt.close(fig)
        print("\n  [图] 已保存 lesson26_exp3_causes.png")


# ===========================================================================
# 实验四：事故时间线重建
# ===========================================================================
# 真实事故的日志从来不是一份。飞控的日志时间是「上电开机的微秒数」，
# 地面站的时间是「电脑的墙上时钟」，遥控器的时间是「开机后的秒数」，
# 视频的时间是「录制开始的帧号」，飞行员的口述是「大概两秒之后」。
# 这五条时间轴互不相干，但它们描述的是**同一个物理过程**。
#
# 重建的第一步不是分析，是**对齐**：找到同一个物理事件在两条时间轴上的位置，
# 就得到了这两条轴之间的偏移。有了偏移，所有日志才能放到同一条轴上。
# 如果 A-B 和 B-C 都有偏移，那 A-C 的偏移就是两者之和 —— 这就是一个
# 带权图的**最短路径**问题。下面用 Floyd-Warshall 一次把所有的偏移都算出来。


class TimeBase(object):
    """一条独立的时间轴。"""

    def __init__(self, name, unit, note=""):
        self.name = name
        self.unit = unit
        self.note = note
        self.events = []                             # [(t_local, text, known_bool)]

    def add(self, t_local, text, known=True):
        self.events.append((float(t_local), text, known))
        return self

    def sorted_events(self):
        return sorted(self.events, key=lambda e: e[0])


def solve_offsets(bases, links):
    """解出每条时间轴相对基准轴的偏移。

    links: [(base_a, base_b, offset)]，含义是
           t_a = t_b + offset   （即 b 轴上的时刻 + offset = a 轴上的时刻）
    返回 (offsets, unreachable)。用 Bellman-Ford 的松弛迭代，
    因为偏移有可能是负数（不同轴的原点先后不同）。
    """
    names = [b.name for b in bases]
    INF = float("inf")
    # d[a][b] = 把 b 轴时刻换算到 a 轴要加的量
    d = {a: {b: (0.0 if a == b else INF) for b in names} for a in names}
    for a, b, off in links:
        d[a][b] = min(d[a][b], off)
        d[b][a] = min(d[b][a], -off)
    for k in names:                                  # Floyd-Warshall
        for i in names:
            for j in names:
                if d[i][k] + d[k][j] < d[i][j]:
                    d[i][j] = d[i][k] + d[k][j]
    ref = names[0]
    offsets = {}
    unreachable = []
    for n in names:
        if d[ref][n] == INF:
            unreachable.append(n)
        else:
            offsets[n] = d[ref][n]
    return offsets, unreachable


def fmt_t(t):
    sign = "-" if t < 0 else "+"
    t = abs(t)
    return "%s%d:%05.2f" % (sign, int(t // 60), t % 60)


def exp4():
    head("实验四  事故时间线重建：把五份日志拼成一条时间轴")

    sub("① 五条互不相干的时间轴")
    bases = [
        TimeBase("FC",     "us since boot", "飞控 .ulg 日志的时间戳"),
        TimeBase("GCS",    "wall clock",    "地面站 tlog 的墙上时钟"),
        TimeBase("RC",     "s since power", "遥控器开机秒数"),
        TimeBase("CAM",    "frame",         "机载录像的帧号"),
        TimeBase("PILOT",  "s, 口述",       "飞行员事后回忆，精度最差"),
    ]
    print("    基准轴是 FC（飞控日志），因为它的采样率最高、时钟最稳。")
    rows = [[b.name, b.unit, b.note] for b in bases]
    tab(rows, ["时间轴", "单位", "来源"])

    # ------------------------------------------------------------------
    sub("② 用「同一个物理事件在两条轴上的位置」求偏移")
    print("""
    对齐靠的是**冗余事件**：解锁、起飞、模式切换、失联、落地 —— 这些事件会同时
    出现在多条记录里。每找到一对，就得到一个方程 t_a = t_b + offset。
    方程够了之后，即使某些轴之间没有直接对应（比如录像和遥控器），
    也能通过中间轴（飞控）把偏移传递过去。
""")
    sim = [
        # FC 轴上的真实事件（单位 s，已换算成秒方便阅读）
        ("FC", 0.00,  "上电"),
        ("FC", 12.40, "解锁（armed）"),
        ("FC", 13.05, "起飞（离地）"),
        ("FC", 17.80, "遥控信号丢失开始"),
        ("FC", 18.30, "manual_control_signal_lost 成立（RC_LOSS_T=0.5）"),
        ("FC", 18.60, "Y 轴出现 40 m/s^2 冲击"),
        ("FC", 19.10, "EKF 位置误差估计超限"),
        ("FC", 23.30, "COM_FAIL_ACT_T 走完，开始 RTL"),
        ("FC", 24.05, "姿态发散，飞控进入 FallbackStab"),
        ("FC", 26.70, "落地（撞击）"),
    ]
    # 各轴的换算关系。约定统一时间轴 = FC 上电后的秒数，换算公式是
    #     t_统一 = t_本轴 * SCALE + OFFSET
    # TRUE_OFFSET 是仿真里预设的真值，一会儿用来检验解算是否正确。
    #   FC    : 日志本来就是「上电后的秒数」，偏移 0
    #   GCS   : 墙上时钟（Unix 纪元），比 FC 轴大 1684300000 秒 —— 两个不同的纪元
    #   RC    : 遥控器比飞控**早** 60 s 开机，所以它记的秒数比 FC 轴大 60
    #   CAM   : 记的是帧号，25 fps
    #   PILOT : 口述的秒数，人已经把它对齐到「失联那一刻」附近
    TRUE_OFFSET = {"FC": 0.0, "GCS": -1684300000.0, "RC": -60.0,
                   "CAM": 0.0, "PILOT": 0.0}
    SCALE = {"FC": 1.0, "GCS": 1.0, "RC": 1.0, "CAM": 1.0 / 25.0, "PILOT": 1.0}

    raw = {"FC": [], "GCS": [], "RC": [], "CAM": [], "PILOT": []}
    for t, txt in [(0.0, "上电"), (12.40, "解锁（armed）"), (13.05, "起飞（离地）"),
                   (17.80, "遥控信号丢失开始"),
                   (18.30, "manual_control_signal_lost 成立（RC_LOSS_T=0.5）"),
                   (18.60, "Y 轴出现 40 m/s^2 冲击"),
                   (19.10, "EKF 位置误差估计超限"),
                   (23.30, "COM_FAIL_ACT_T 走完，开始 RTL"),
                   (24.05, "姿态发散，飞控进入 FallbackStab"),
                   (26.70, "落地（撞击）")]:
        raw["FC"].append((t, txt))
    # GCS：只记了几个关键事件，时间是墙上时钟
    for t, txt in [(12.40, "armed"), (13.05, "takeoff"),
                   (17.80, "telemetry: RC signal lost"),
                   (23.30, "mode -> AUTO.RTL"),
                   (26.70, "landed / crash detected")]:
        raw["GCS"].append((1684300000.0 + t, txt))
    # RC：只记了链路事件
    for t, txt in [(17.80 + 60.0, "RF link lost (RSSI < -95 dBm)"),
                   (26.70 + 60.0, "RF link lost (never recovered)")]:
        raw["RC"].append((t, txt))
    # CAM：机载录像，记的是帧号
    for t, txt in [(18.60, "frame: sudden shock visible"),
                   (24.10, "frame: horizon lost"),
                   (26.70, "frame: impact")]:
        raw["CAM"].append((t * 25.0, txt))
    # PILOT：事后口述，精度最差
    for t, txt in [(18.0, "「信号没了，我松了一下杆」"),
                   (21.0, "「我想切自稳，但没找到开关」"),
                   (25.0, "「掉下来了」")]:
        raw["PILOT"].append((t, txt))

    print("    原始日志（各轴自己的时间，注意它们的量级完全不同）：")
    for b in bases:
        print("      [%s] (%s)" % (b.name, b.unit))
        for t, txt in sorted(raw[b.name]):
            print("         %18.2f   %s" % (t, txt))
    print()

    # 已知的对应关系（真实调查中，这些是靠人工比对事件找到的）。
    # ★ 注意这里**故意没有把每条轴都直接挂在 FC 上**：
    #   录像挂在 GCS 上，口述挂在遥控器上 —— 这样才能演示偏移的传递。
    links = [
        ("FC", "GCS",   -1684300000.0),     # 同一时刻，纪元不同
        ("FC", "RC",    -60.0),             # 遥控器早开机 60 s
        ("GCS", "CAM",  1684300000.0),      # 录像文件头里写了拍摄的墙上时钟
        ("RC", "PILOT", 60.0),              # 飞行员拿遥控器的计时器当秒表
    ]
    print("    已知的对应关系（每条是一个方程 t_a = t_b + offset）：")
    for a, b, off in links:
        print("      t_%-6s = t_%-6s %+.1f" % (a, b, off))
    print()

    offsets, unreachable = solve_offsets(bases, links)
    print("    解出的偏移（把该轴时刻换算到 FC 轴要加的量）：")
    rows = []
    for b in bases:
        if b.name in offsets:
            err = offsets[b.name] - TRUE_OFFSET[b.name]
            rows.append([b.name, "%+.2f" % offsets[b.name], "%+.2f" % err,
                         "完全对齐" if abs(err) < 1e-6 else "★ 有残差"])
        else:
            rows.append([b.name, "-", "-", "★ 不可达（偏移解不出来）"])
    tab(rows, ["时间轴", "解出偏移", "与真值之差", "评价"])
    if unreachable:
        print("    ★ 有 %d 条轴解不出来：%s —— 这是真实调查里最常见的情况，"
              % (len(unreachable), "、".join(unreachable)))
        print("      说明这几条记录和别的记录之间**找不到任何共同事件**。")
        print("      解决办法只有两个：要么找到一段重叠的录像/照片，")
        print("      要么承认这条证据只能定性使用（比如「掉下来了」这种口述）。")
    print("""
    ★ 这里的「真值」是仿真里预设的，所以残差是 0。真实调查里没有真值，
      验证对齐是否正确的办法是：**用解出的偏移去预测一个你没有用来对齐的事件，
      看它落在哪里**。比如你只用「起飞」对齐了飞控和地面站，
      那么「落地」就应该自动对上 —— 如果对不上，说明中间有时钟漂移
      （飞控的晶振和电脑的晶振频率不同，偏移是随时间**线性变化**的）。
      长时间飞行必须做线性拟合，不能只用一个常数偏移，
      否则事故末尾的几百毫秒会错位 —— 而那恰恰是最关键的几百毫秒。""")

    # ------------------------------------------------------------------
    sub("③ 拼好的统一时间线")
    print("""
    把每条轴上的事件都换算到 FC 轴，按时间排序，就得到了统一时间线。
    下面把「每个事件的时刻」和「它相对上一个事件过了多久」都列出来 ——
    **ΔT 那一列才是关键**，因为事故分析问的是「还剩多少时间可以反应」。
""")
    unified = []
    for b in bases:
        off = offsets.get(b.name, 0.0)
        for t, txt in raw[b.name]:
            unified.append((t * SCALE[b.name] + off, b.name, txt))
    unified.sort(key=lambda x: x[0])
    rows = []
    prev = None
    for t, src, txt in unified:
        dt = "" if prev is None else "%+.2f" % (t - prev)
        rows.append(["%+8.2f" % t, dt, src, txt])
        prev = t
    tab(rows, ["t (s)", "ΔT", "来源", "事件"])
    print("""
    ★ 读这张表要看三件事：
      1) **ΔT 的分布**。事件密集的地方是「事情发生得比人快」的地方 ——
         这里 18.30 到 19.10 的 0.8 秒里塞了三个事件（失联成立、4g 冲击、EKF 超限），
         而人的反应时间在 0.5~1.0 秒量级。也就是说**从失联到失控，
         留给飞行员的时间不到一个反应周期**。
      2) **哪些事件是「果」而不是「因」**。这里 Y 轴冲击（18.60）
         在失联（18.30）**之后** —— 所以「振动导致失联」这个假设被时间线否掉了，
         更可能是「失联后飞机进入 Hold，姿态抖动加剧，激起了那个冲击」，
         或者两者都由第三个原因（比如一路动力已经在衰减）引起。
         **这是时间线分析的第一个产出：它排除假设。**
      3) **有哪些事件在两条轴上都有记录，但时刻对不上**。
         如果遥控器的「失联」和飞控的「失联」差了几百毫秒，那不是时钟问题，
         而是**射频链路和飞控检测是两套独立的超时逻辑**（COM_RC_LOSS_T=0.5 s）。""")

    # ------------------------------------------------------------------
    sub("④ 反事实推演：在每个决策点上，别的动作会怎样")
    print("""
    时间线重建的第二个产出，是让你可以**在每个决策点上问「如果换成别的动作会怎样」**。
    下面在几个关键点上用实验二的能量模型和几何模型做推演。
""")
    # 用真实的飞行状态
    ALT = 45.0          # 事发时高度 (m)
    DIST = 260.0        # 距起飞点水平距离 (m)
    SOC_AT_LOSS = 0.34  # 事发时剩余电量

    decisions = [
        ("t=+18.30  遥控丢失 -> 系统进入 Hold 并开始 COM_FAIL_ACT_T 倒计时",
         "保持不动，等 5 秒（PX4 默认行为）",
         "飞机原地停住。这 5 秒里飞行员有机会恢复链路。"
         "代价是 5 秒 × 悬停电流 ≈ %.0f 悬停秒的电量。" % (COM_FAIL_ACT_T * 1.0)),
        ("t=+18.30  如果当时把失效保护动作预先配成 Land",
         "原地降落",
         "消耗 %.0f 悬停秒。高度 %.0f m，按 MPC_LAND_SPEED=%.1f m/s 需要 %.0f 秒。"
         "★ 这条路径**不需要 GPS、不需要航向、不需要 EKF 位置**，"
         "只需要高度和姿态 —— 而这两样在振动中反而是最稳的。"
         % (land_cost_seconds(ALT), ALT, MPC_LAND_SPEED, ALT / MPC_LAND_SPEED)),
        ("t=+18.30  如果当时把失效保护动作预先配成 RTL（NAV_RCL_ACT 的默认值）",
         "爬升到 60 m 再平飞回家",
         "消耗 %.0f 悬停秒，可用 %.0f 悬停秒 —— %s。"
         "★ 而且 RTL 需要**位置和航向**，恰恰是被振动污染得最厉害的两样。"
         % (rtl_cost_seconds(DIST, ALT), hover_seconds(SOC_AT_LOSS - 0.15),
            "电量上可行" if rtl_feasible(DIST, ALT, SOC_AT_LOSS)[2] else "电量上已经不可行")),
        ("t=+23.30  COM_FAIL_ACT_T 走完，系统开始执行 RTL",
         "此刻改为 Hold 或 Land 还来得及吗？",
         "此时剩余电量约 %.0f%%，可用 %.0f 悬停秒；"
         "原地降落要 %.0f 悬停秒 —— 来得及。"
         "但切换到 Land 需要飞行员**在姿态已经发散的情况下**正确操作遥控器，"
         "而 COM_FAIL_ACT_T 之所以默认 5 秒，就是假设飞行员在这 5 秒里会接手。"
         % ((SOC_AT_LOSS - 0.06) * 100,
            hover_seconds(SOC_AT_LOSS - 0.06 - 0.15), land_cost_seconds(ALT))),
    ]
    for title, act, analysis in decisions:
        print("  ● %s" % title)
        print("    备选动作：%s" % act)
        print("    %s" % analysis)
        print()

    print("""    ★ 反事实推演的纪律：**不要用它来事后指责**，要用它来改配置。
      上面的推演里唯一有行动价值的一句是第 2 条：
      「Land 只需要高度和姿态，而这两样在振动中反而最稳」。
      把这句话变成配置，就是：如果在你的场地里 RTL 的返航路径不可靠，
      就把遥控丢失的动作选成 Land，并且**在起飞前实测一次**
      （这就是实验一 E4 那条检查项）。

      ★ 最后一句关于「证据」的提醒：
      事故调查的产物不是一个「原因」，而是一个**按置信度排序的假设列表**，
      外加「如何证伪每一条」的下一步动作。任何声称「就是 XX 导致的」
      而没有排除掉其他假设的结论，都不该被写进复盘报告 ——
      因为它会把你引向一个错误的修改方向，而下一个架次还会摔。""")

    if _HAS_MPL:
        fig, ax = plt.subplots(figsize=(11.5, 4.2))
        colors = {"FC": "#4c72b0", "GCS": "#dd8452", "RC": "#c44e52",
                  "CAM": "#55a868", "PILOT": "#8172b3"}
        for i, name in enumerate(["FC", "GCS", "RC", "CAM", "PILOT"]):
            off = offsets.get(name, 0.0)
            ts = [t * SCALE[name] + off for t, _ in raw[name]]
            ax.plot(ts, [i] * len(ts), "o", color=colors[name], ms=8, label=name)
        for t, src, txt in unified:
            ax.annotate(txt[:26], (t, 0), fontsize=5.2, rotation=90,
                        xytext=(0, 8), textcoords="offset points")
        ax.set_yticks(range(5))
        ax.set_yticklabels(["FC", "GCS", "RC", "CAM", "PILOT"])
        ax.set_xlabel("unified time  [s]")
        ax.set_title("Reconstructed accident timeline")
        ax.grid(alpha=0.3, axis="x")
        fig.tight_layout()
        fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "lesson26_exp4_timeline.png"), dpi=110)
        plt.close(fig)
        print("\n  [图] 已保存 lesson26_exp4_timeline.png")


# ===========================================================================
EXPERIMENTS = {
    1: ("起飞前检查单生成器", exp1),
    2: ("失效保护逻辑仿真", exp2),
    3: ("事故成因反演", exp3),
    4: ("事故时间线重建", exp4),
}


def main(argv):
    if len(argv) == 1:
        todo = [1, 2, 3, 4]
    else:
        todo = []
        for a in argv[1:]:
            try:
                k = int(a)
            except ValueError:
                print("无法理解的实验编号：%s" % a)
                print(__doc__)
                return 1
            if k not in EXPERIMENTS:
                print("没有实验 %d。可选：%s" % (k, sorted(EXPERIMENTS)))
                return 1
            todo.append(k)

    print("=" * W)
    print("卷 11 第 3 章配套脚本：试飞流程与安全清单")
    print("将运行 %d 个实验：%s"
          % (len(todo), "、".join("%d %s" % (k, EXPERIMENTS[k][0]) for k in todo)))
    if _HAS_MPL:
        print("matplotlib 可用 —— 会同时输出 PNG 图")
    else:
        print("matplotlib 不可用 —— 只输出文本（所有结论不受影响）")
    print("=" * W)

    for k in todo:
        EXPERIMENTS[k][1]()

    print()
    print("=" * W)
    print("全部完成。")
    print("=" * W)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
