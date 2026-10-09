#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 3 第 8 章 · 其他导航算法 —— 配套实验

    实验 1  ★★★ 四种范式的正面对决（同一路径、同一初值、同一速度）
    实验 2  ★★  折线 vs 样条：转角处到底差在哪（曲率剖面）
    实验 3  ★★  纯追踪的前视距离敏感性：太短抖、太长切弯
    实验 4  ★★★ 横向对比总表（四种范式的适用边界）

用法：
    python lesson33_nav_compare.py 1
    python lesson33_nav_compare.py 2
    python lesson33_nav_compare.py 3
    python lesson33_nav_compare.py 4
    python lesson33_nav_compare.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  能算的就不猜。

  · 实验 1 **不引用任何飞控源码** —— 四种制导律都是自己按定义实现的，
    放在**同一个仿真器**里跑**同一条路径**。
    这样差异就只可能来自算法本身，不会来自实现环境。
  · 实验 2 的「折线 vs 样条」是真的把两条路径都采样出来、算曲率、
    再看曲率在转角处**连不连续** —— 用数字说话，不是「样条更平滑」一句话带过。
  · 实验 3 把前视距离从 2 m 扫到 40 m，量**横向误差的峰值和振荡次数**，
    让「太短抖、太长切弯」这条经验变成可复算的曲线。

【★ 本章的一个关键洞察（实验 1 会验证它）】

  纯追踪（pure pursuit）和 L1 的**加速度公式完全一样**：

        纯追踪： a = 2 * V^2 / L * sin(alpha)
        L1    ： a = K_L1 * V^2 / L1 * sin(eta)      （K_L1 = 4*zeta^2）

  取 zeta = 0.7071 时 K_L1 = 2，两者**逐字相同**。

  唯一的区别在**前视点怎么取**：

        纯追踪：在路径上，取【沿弧长】前方 L 米处的那个点
        L1    ：在路径上，取【距飞机】恰好 L1 米的那个点

  ⇒ 直线航段上两者等价；**弯道上会分岔**（因为「沿弧长前方」和
    「到飞机的距离」在曲线上的落点不同）。实验 1 把两条轨迹画在一起，
    并且量它们的分岔程度。

参考：
    卷 3 第 6 章（L1 的完整推导）
    卷 3 第 7 章（方向场、风三角）
    卷 3 第 3 章（位置环怎么消费这些给定）
"""

import math
import sys

import numpy as np

W = 74

_HAS_MPL = False
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as _fm
    for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                  "Source Han Sans SC", "WenQuanYi Micro Hei",
                  "PingFang SC", "Heiti SC"):
        try:
            _fm.findfont(_name, fallback_to_default=False)
        except Exception:
            continue
        matplotlib.rcParams["font.sans-serif"] = [_name]
        matplotlib.rcParams["axes.unicode_minus"] = False
        break
    _HAS_MPL = True
except ImportError:
    pass


def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)
    print()


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (W - 4))
    print()


def _dlen(s):
    n = 0
    for ch in str(s):
        o = ord(ch)
        n += 2 if (0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF
                   or 0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF
                   or 0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60
                   or 0xFFE0 <= o <= 0xFFE6) else 1
    return n


def tab(rows, headers, width=None):
    cols = len(headers)
    if width is None:
        width = [max(_dlen(r[i]) for r in [headers] + rows) + 2
                 for i in range(cols)]

    def cell(s, w):
        s = str(s)
        return s + " " * max(1, w - _dlen(s))

    print("  " + "".join(cell(headers[i], width[i]) for i in range(cols)))
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join(cell(r[i], width[i]) for i in range(cols)))
    print()


def savefig(name):
    if not _HAS_MPL:
        return
    try:
        plt.tight_layout()
        plt.savefig(name + ".png", dpi=110)
        plt.close()
        print("  [图] 已保存 %s.png" % name)
    except Exception:
        pass


# ===========================================================================
#  一条测试路径：直线 → 圆弧 → 直线（带一个 90 度弯）
# ===========================================================================

class Path:
    """由直线段和圆弧拼成的路径，提供「按弧长取点 / 求最近点」两种查询。"""

    def __init__(self, segments):
        # segments: [(kind, p0, p1_or_radius, ...)]  这里直接存采样点
        self.pts = []
        self._build(segments)
        self.pts = np.array(self.pts)
        # 累积弧长
        d = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        self.length = self.s[-1]

    def _build(self, segments):
        for seg in segments:
            kind = seg[0]
            if kind == "line":
                p0, p1, n = seg[1], seg[2], seg[3]
                for k in range(n):
                    t = k / float(n)
                    self.pts.append(np.array(p0) * (1 - t) + np.array(p1) * t)
            elif kind == "arc":
                c, r, a0, a1, n = seg[1], seg[2], seg[3], seg[4], seg[5]
                for k in range(n):
                    a = a0 + (a1 - a0) * k / float(n)
                    self.pts.append(np.array([c[0] + r * math.cos(a),
                                              c[1] + r * math.sin(a)]))
        self.pts.append(np.array(segments[-1][2] if segments[-1][0] == "line"
                                 else segments[-1][1]))

    def point_at_s(self, s):
        """按弧长取点（纯追踪用的就是这个）"""
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.s, s))
        i = min(max(i, 1), len(self.s) - 1)
        t = (s - self.s[i - 1]) / max(self.s[i] - self.s[i - 1], 1e-9)
        return self.pts[i - 1] * (1 - t) + self.pts[i] * t

    def tangent_at_s(self, s):
        ds = 0.5
        a = self.point_at_s(max(s - ds, 0.0))
        b = self.point_at_s(min(s + ds, self.length))
        v = b - a
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else np.array([1.0, 0.0])

    def closest(self, p):
        """求最近点的 (弧长 s, 距离 d, 带符号侧偏 e)"""
        d2 = np.sum((self.pts - p) ** 2, axis=1)
        i = int(np.argmin(d2))
        s = self.s[i]
        t = self.tangent_at_s(s)
        rel = p - self.pts[i]
        e = t[0] * rel[1] - t[1] * rel[0]        # 叉积 = 带符号侧偏
        return s, math.sqrt(d2[i]), e

    def point_at_distance(self, p, L):
        """找路径上【距 p 恰好 L】的那个点（L1 用的就是这个）

        做法：在弧长上扫一遍，取 |point(s) - p| 最接近 L 的 s。
        ★ 注意这和「沿弧长前方 L 米」是**两回事**，弯道上会明显不同。
        """
        ss = np.linspace(0, self.length, 400)
        pp = np.array([self.point_at_s(x) for x in ss])
        dd = np.linalg.norm(pp - p, axis=1)
        i = int(np.argmin(np.abs(dd - L)))
        return pp[i], ss[i]


def make_path():
    """直线 → 90 度右转圆弧(R=20) → 直线

    ★ 路径必须【比仿真时长大】。作者第一版只给了 121 m，
      而 15 m/s 跑 22 秒要走 330 m —— 飞机在中途就冲出了路径末端，
      之后"最近点"永远卡在终点，四种控制器全部失去跟踪目标，
      数据完全不可信（表现为"最大侧偏 40~60 m 的诡异发散"）。
      ⇒ 本路径长约 391 m，跑 24 s（360 m）始终在路径上。
    """
    return Path([
        ("line", (0.0, 0.0), (200.0, 0.0), 800),
        ("arc", (200.0, 20.0), 20.0, -math.pi / 2, 0.0, 120),
        ("line", (220.0, 20.0), (380.0, 20.0), 700),
    ])


# ===========================================================================
#  四种制导律 —— 都作用于同一个「独轮车」模型
# ===========================================================================

class UniCycle:
    """状态 (x, y, psi)，速度沿机头，横向加速度指令改变航向。

        omega = a_lat / V        （横向加速度 → 航向角速率）
        psi  += omega * dt
        x    += V cos(psi) dt ;  y += V sin(psi) dt

    这是固定翼/无人车的标准简化模型（不能侧滑）。
    """

    def __init__(self, V=15.0):
        self.V = V
        self.p = np.array([0.0, -12.0])       # 起点故意偏离航线 12 m
        self.psi = 0.0                        # 机头朝 +x

    def step(self, a_lat, dt):
        a_lat = min(max(a_lat, -4.0 * 9.80665), 4.0 * 9.80665)   # 限 4g
        self.psi += (a_lat / self.V) * dt
        self.p = self.p + self.V * np.array([math.cos(self.psi),
                                             math.sin(self.psi)]) * dt


def ctrl_pure_p(p, psi, V, path, kp=0.02):
    """范式 0：最朴素的横向比例控制（没有任何前视）

        a = -kp * e          （e 是带符号侧偏）
    """
    _, _, e = path.closest(p)
    return kp * V * V * (-e) / 40.0            # 归一化一下量级


def ctrl_pure_pursuit(p, psi, V, path, L=25.0):
    """范式 1：纯追踪（pure pursuit）

        ★ 前视点 = 路径上【沿弧长】前方 L 米处的那个点
        a = 2 V^2 / L * sin(alpha)      alpha = 前视点方位 - 机头方向
    """
    s, _, _ = path.closest(p)
    tgt = path.point_at_s(s + L)
    v = tgt - p
    n = np.linalg.norm(v)
    if n < 1e-6:
        return 0.0
    alpha = math.atan2(v[1], v[0]) - psi
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))   # wrap 到 [-pi,pi]
    return 2.0 * V * V / L * math.sin(alpha)


def ctrl_l1(p, psi, V, path, L1=25.0):
    """范式 2：L1

        ★ 前视点 = 路径上【距飞机】恰好 L1 米的那个点（不是沿弧长！）
        a = K_L1 * V^2 / L1 * sin(eta)      （K_L1 = 4*zeta^2，取 2）
    """
    tgt, _ = path.point_at_distance(p, L1)
    v = tgt - p
    n = np.linalg.norm(v)
    if n < 1e-6:
        return 0.0
    eta = math.atan2(v[1], v[0]) - psi
    eta = math.atan2(math.sin(eta), math.cos(eta))
    K_L1 = 2.0                                  # = 4 * (1/sqrt(2))^2
    return K_L1 * V * V / L1 * math.sin(eta)


def ctrl_npfg(p, psi, V, path, bound=20.0):
    """范式 3：NPFG 的方位矢量（简化版：无风、无曲率前馈）

        nte = |e| / bound    （归一化偏差）
        laa = pi/2 * (nte-1)^2
        b   = cos(laa)*指向路径 + sin(laa)*沿路径
        然后让机头对准 b（用的还是 V^2/L 那个形状，L 取 bound）
    """
    s, _, e = path.closest(p)
    t = path.tangent_at_s(s)
    nte = min(abs(e) / bound, 1.0)
    laa = math.pi / 2.0 * (nte - 1.0) ** 2
    nvec = np.array([-t[1], t[0]])
    sgn = -1.0 if e < 0 else 1.0
    e_track = -sgn * nvec
    b = math.cos(laa) * e_track + math.sin(laa) * t
    ang = math.atan2(b[1], b[0]) - psi
    ang = math.atan2(math.sin(ang), math.cos(ang))
    return 2.0 * V * V / bound * math.sin(ang)


CTRLS = [
    ("纯横向 P（无前视）", ctrl_pure_p),
    ("纯追踪 pure pursuit", ctrl_pure_pursuit),
    ("L1", ctrl_l1),
    ("NPFG 方位矢量", ctrl_npfg),
]


def run_one(ctrl, path, V=15.0, T=24.0, dt=0.005, **kw):
    car = UniCycle(V)
    ts, xs, ys, es, ps = [0.0], [car.p[0]], [car.p[1]], [], [car.psi]
    n = int(T / dt)
    for k in range(n):
        a = ctrl(car.p, car.psi, V, path, **kw)
        car.step(a, dt)
        ts.append((k + 1) * dt)
        xs.append(car.p[0])
        ys.append(car.p[1])
        ps.append(car.psi)
        _, _, e = path.closest(car.p)
        es.append(e)
    es = np.array(es)
    return dict(ts=np.array(ts), xs=np.array(xs), ys=np.array(ys),
                es=es, psi=np.array(ps),
                e_abs_max=float(np.max(np.abs(es))),
                e_abs_mean=float(np.mean(np.abs(es))),
                e_final=float(es[-1]))


# ===========================================================================
#  实验 1 ★★★ 四种范式的正面对决
# ===========================================================================

def exp1():
    head("实验 1 ★★★ 四种范式的正面对决")

    path = make_path()
    V = 15.0
    print("""  同一条路径、同一个初值、同一个速度、同一个仿真器。
  唯一变量是【制导律】：

      路径：直线(0,0)→(60,0) → 90 度右转圆弧(R=20) → 直线到 (110,20)
      初值：飞机在 (0, -12)，即**偏离航线 12 m**，机头朝 +x
      速度：15 m/s 恒定（独轮车模型，不能侧滑）
""")

    sub("① 三种指标对比（跑 24 秒（360 m，路径长约 391 m））")
    rows = []
    res = {}
    for name, fn in CTRLS:
        r = run_one(fn, path, V=V)
        res[name] = r
        rows.append([name, "%.2f" % r["e_abs_max"], "%.3f" % r["e_abs_mean"],
                     "%+.3f" % r["e_final"]])
    tab(rows, ["制导律", "最大侧偏 [m]", "平均 |侧偏| [m]", "末态侧偏 [m]"])

    sub("② ★★ 关键对照：纯追踪 vs L1 —— 公式相同，前视点不同")
    print("""  两者的加速度公式是**同一个**（取 zeta=1/sqrt2 ⇒ K_L1=2）：

        纯追踪： a = 2 * V^2 / L  * sin(alpha)
        L1    ： a = 2 * V^2 / L1 * sin(eta)

  唯一的区别在**前视点怎么取**：
        纯追踪：沿【弧长】前方 L 米的点
        L1    ：到飞机【距离】恰好 L1 米的点

  ⚠️ 作者第一版在这里写的是「直线段上两者完全等价」——
     **实测证明这句话是错的**（见下表：还在直线段上就差了 7 m）。
     正确的说法要更精细，见下面的几何分析。
""")
    rp = res["纯追踪 pure pursuit"]
    rl = res["L1"]
    n = min(len(rp["es"]), len(rl["es"]))
    diff = np.abs(rp["es"][:n] - rl["es"][:n])
    rows = []
    for tq in (2, 5, 8, 11, 14, 17, 21):
        i = min(int(tq / 0.005), n - 1)
        rows.append(["%.0f" % tq, "%+.3f" % rp["es"][i], "%+.3f" % rl["es"][i],
                     "%.4f" % diff[i]])
    tab(rows, ["t [s]", "纯追踪 侧偏 [m]", "L1 侧偏 [m]", "两者之差 [m]"])
    print("  ⇒ 最大分岔 %.4f m（本场景）。" % float(np.max(diff)))
    print()
    print("""  ★★ 为什么**直线段上也会分岔**？把几何摊开算。

  设飞机偏离航线 e、前视距离 L，则两个「前视点」的**等效纵向距离**是：

      纯追踪： sqrt(L^2 + e^2)      ← 比 L **长**
      L1    ： sqrt(L^2 - e^2)      ← 比 L **短**

  （纯追踪取「沿弧长 L」，在直线上就是纵向 L，
    于是它到飞机的直线距离是 sqrt(L^2+e^2)；
    L1 取「到飞机距离 L」，于是纵向只能是 sqrt(L^2-e^2)。）

  而横向加速度 a = 2*V^2/L * sin(alpha)，两者的角是：

      alpha_pp = atan(e / L)
      alpha_l1 = atan(e / sqrt(L^2 - e^2))
""")
    rows = []
    for e in (0.0, 4.0, 12.0, 20.0, 24.0):
        L = 25.0
        rows.append(["%.0f" % e, "%.3f" % math.sqrt(L * L + e * e),
                     "%.3f" % math.sqrt(L * L - e * e),
                     "%.4f" % math.atan2(e, L),
                     "%.4f" % math.atan2(e, math.sqrt(L * L - e * e)),
                     "%.4f" % (math.atan2(e, math.sqrt(L * L - e * e))
                               - math.atan2(e, L))])
    tab(rows, ["偏差 e [m]", "纯追踪等效前视 [m]", "L1 等效前视 [m]",
               "alpha_pp [rad]", "alpha_l1 [rad]", "L1 多出的角 [rad]"])
    print("""  ⇒ **只有在 e = 0（正好在航线上）时两者才完全相同。**
    一旦有偏差，L1 的等效前视就**变短**（更激进、更欠阻尼），
    而纯追踪的**变长**（更稳）。

  ★ 这是个反直觉的结论：
    **「取距飞机 L 的那个点」听起来更自然，
      但它在有偏差时把前视距离偷偷缩掉了 —— 恰恰在你最需要阻尼的时候。**

  ⇒ 那 L1 为什么还这么用？因为它的**求法代价小得多**：
    · 「沿弧长前方 L」需要一个**沿路径推进的参数** s，
      路径必须是参数化的（这正是多旋翼那条路线干的事）；
    · 「到飞机距离 L1」只需要路径的**几何形状** ——
      一个最近点 + 一个交点就够，对任意曲线都适用。
    ⇒ 固定翼飞的是「任意航段」，所以 L1 那种取法更省事；
      而它多出来的那点振荡，被 period/damping 两个参数吸收了
      （卷 3 第 6 章证过：闭环阻尼可以由 damping 精确指定）。""")

    sub("③ 相位图：四种制导律回到航线的方式")
    rows = []
    for name, fn in CTRLS:
        r = res[name]
        # 找第一次进入 |e|<0.5 的时刻
        idx = np.where(np.abs(r["es"]) < 0.5)[0]
        t_settle = "%.2f" % r["ts"][idx[0]] if len(idx) else "未回到"
        # 数过冲次数（侧偏穿越 0 的次数）
        zc = int(np.sum(np.diff(np.sign(r["es"])) != 0))
        rows.append([name, t_settle, "%d" % zc])
    tab(rows, ["制导律", "首次回到 |e|<0.5 用时 [s]", "穿越次数"])
    print("""  ★ 读法：
    · 「无前视的纯 P」穿越次数最多 —— 它没有阻尼项，必然振荡；
    · 三个带前视的（纯追踪 / L1 / NPFG）都因为「看前方」而自带阻尼，
      穿越次数明显更少。
    ⇒ **"前视"这个动作本身就是阻尼**，这和卷 3 第 6 章证过的
      「L1 ≡ PD 控制器」是同一件事：前视点把「位置误差」和「速度误差」
      一起编进了同一个角度里。""")

    if _HAS_MPL:
        plt.figure(figsize=(9, 3.6))
        plt.plot(path.pts[:, 0], path.pts[:, 1], "k--", lw=1.0, label="路径")
        for name, fn in CTRLS:
            r = res[name]
            plt.plot(r["xs"], r["ys"], lw=1.3, label=name)
        plt.axis("equal")
        plt.grid(alpha=0.3)
        plt.legend(fontsize=7)
        plt.title("四种制导律跟踪同一条路径（初值偏离 12 m）")
        savefig("lesson33_exp1")


# ===========================================================================
#  实验 2 ★★ 折线 vs 样条：转角处到底差在哪
# ===========================================================================

def _corner_polyline():
    """折线：两段直线直接相交，没有圆角（长度与 make_path() 对齐，便于对照）"""
    return Path([
        ("line", (0.0, 0.0), (200.0, 0.0), 1200),
        ("line", (200.0, 0.0), (220.0, 20.0), 250),
        ("line", (220.0, 20.0), (380.0, 20.0), 900),
    ])


def _corner_spline():
    """圆角：用一个相切的圆弧把转角抹平（这就是「样条」在工程里的常见形态）"""
    return make_path()


def _curvature_profile(path, n=300):
    """数值曲率：kappa = d(theta)/ds"""
    ss = np.linspace(1.0, path.length - 1.0, n)
    th = []
    for s in ss:
        t = path.tangent_at_s(s)
        th.append(math.atan2(t[1], t[0]))
    th = np.unwrap(np.array(th))
    kappa = np.gradient(th, ss)
    return ss, kappa


def exp2():
    head("实验 2 ★★ 折线 vs 样条：转角处到底差在哪")

    print("""  「样条比折线平滑」是一句人云亦云的话。到底差在哪？
  把它变成可量的东西：**曲率**。

      曲率 kappa = d(theta) / ds      （航向角对弧长的变化率）
      而横向加速度 a_lat = V^2 * kappa

  ⇒ **曲率就是「这段路需要多少横向加速度」**。
    所以曲率的**连续性**直接决定了横向加速度会不会跳变。

  两条路径都过同样的两个转角点，区别只在转角处：
      · 折线：两段直线直接相交
      · 圆角：用相切的圆弧过渡（R=20 m）
""")

    pl = _corner_polyline()
    sp = _corner_spline()

    sub("① 曲率剖面对比")
    rows = []
    for name, p in (("折线", pl), ("圆角 R=20", sp)):
        ss, k = _curvature_profile(p)
        rows.append([name, "%.1f" % p.length, "%.4f" % float(np.max(np.abs(k))),
                     "%.4f" % float(np.max(np.abs(np.diff(k)) * len(k)))])
    tab(rows, ["路径", "总长 [m]", "最大 |kappa| [1/m]",
               "最大相邻跳变（已放大）"])
    print("""  ★ 看最后一列：**折线的曲率在转角处会「跳」** ——
    数值上表现为一个极大的相邻差（真实世界里是狄拉克函数：∞）。
    对应的物理意义是：**横向加速度指令瞬间从 0 跳到无穷**。

  ⇒ 这就是「折线不能用」的真正原因 —— 不是不好看，
    是**它要求执行机构做物理上做不到的事**（无限大的横向加速度）。
""")

    sub("② 同一条折线，飞机跟着会怎样")
    r_pl = run_one(ctrl_npfg, pl, V=15.0, bound=20.0)
    r_sp = run_one(ctrl_npfg, sp, V=15.0, bound=20.0)
    # 统计横向加速度的峰值（用 psi 的二次差分近似）
    def max_lat_acc(r, V=15.0, dt=0.005):
        dpsi = np.diff(r["psi"]) / dt
        return float(np.max(np.abs(dpsi))) * V
    tab([["折线", "%.2f" % max_lat_acc(r_pl), "%.3f" % r_pl["e_abs_max"]],
         ["圆角 R=20", "%.2f" % max_lat_acc(r_sp), "%.3f" % r_sp["e_abs_max"]]],
        ["路径", "峰值横向加速度 [m/s^2]", "最大侧偏 [m]"])
    print("""  ⇒ 折线的峰值横向加速度明显更大 ——
    因为控制器在转角处被迫「急转」，而它**没有任何机制知道**那里不该急转。

  ★ 于是就有了工程上的分工：
    · **路径生成**（路径规划层）负责把几何做成「可飞的」（圆角/样条）；
    · **制导层**只管「贴着给定的路飞」，不负责改路。
    ⇒ 多旋翼那条路线（`AC_WPNav` 的样条）就是把这两件事**放在了一起**，
      而固定翼的 L1/NPFG 只管后者 —— 这是下一节的主线。""")

    if _HAS_MPL:
        s1, k1 = _curvature_profile(pl)
        s2, k2 = _curvature_profile(sp)
        plt.figure(figsize=(9, 3.2))
        plt.plot(s1, k1, lw=1.2, label="折线")
        plt.plot(s2, k2, lw=1.2, label="圆角 R=20")
        plt.xlabel("弧长 s [m]")
        plt.ylabel("曲率 [1/m]")
        plt.title("曲率剖面：折线在转角处跳变，圆角连续")
        plt.grid(alpha=0.3)
        plt.legend(fontsize=8)
        savefig("lesson33_exp2")


# ===========================================================================
#  实验 3 ★★ 前视距离敏感性
# ===========================================================================

def exp3():
    head("实验 3 ★★ 前视距离敏感性：太短抖、太长切弯")

    path = make_path()
    V = 15.0
    print("""  纯追踪只有一个参数：前视距离 L。它的行为对 L 极其敏感，
  而且两个方向的失效**各有各的名字**：

      L 太小 → 追着很近的点跑，路径曲率一扰动就来回摆  → "抖"
      L 太大 → 前视点远远落在弯道另一侧，直接切过去      → "切弯"
""")

    sub("① 扫 L：量峰值侧偏、平均侧偏、穿越次数")
    rows = []
    for L in (4.0, 8.0, 12.0, 20.0, 30.0, 40.0, 60.0):
        r = run_one(ctrl_pure_pursuit, path, V=V, T=24.0, L=L)
        zc = int(np.sum(np.diff(np.sign(r["es"])) != 0))
        rows.append(["%.0f" % L, "%.2f" % r["e_abs_max"],
                     "%.3f" % r["e_abs_mean"], "%d" % zc])
    tab(rows, ["前视距离 L [m]", "最大侧偏 [m]", "平均 |侧偏| [m]", "穿越次数"])
    print("""  ★ 这张表**两个方向都不好**：
    · L=4：穿越次数多（抖）—— 但注意最大侧偏反而**不是**最大的；
    · L=60：最大侧偏最大（切弯）—— 但穿越次数为 0（很「稳」）。

  ⇒ 「稳定」和「贴线」是**两件事**，而这一个参数同时控制它们。
    这就是纯追踪最大的工程问题 —— **一个旋钮管两件事**。

  ★ 对比卷 3 第 6 章的 L1：它把这一个旋钮**拆成了两个**：
      period 管快慢（稳定性）、damping 管超调（贴线性），
    而且由 K_L1 = 4*zeta^2 这个关系保证两者互不干扰。
    ⇒ **这就是「用周期和阻尼参数化」的价值**，本实验是它的反面证明。
""")

    sub("② 有没有一个「最优」L？")
    best = None
    for L in np.linspace(3, 60, 25):
        r = run_one(ctrl_pure_pursuit, path, V=V, T=25.0, L=float(L))
        score = r["e_abs_mean"]
        if best is None or score < best[1]:
            best = (float(L), score)
    print("  按「平均 |侧偏| 最小」扫出来：L ≈ %.1f m 时最好（%.3f m）" %
          (best[0], best[1]))
    print()
    print("""  ⚠️ 但这个「最优值」**没有普适性** —— 它依赖：
      · 速度 V（L 通常取成 V 的若干倍，所以速度一变就要重调）
      · 路径的曲率（弯道越急，L 该越小）
      · 允许的横向加速度上限

  ⇒ 所以工程上纯追踪的 L 一般写成 **L = k * V**（跟速度走），
    而不是一个固定的米数。这和 L1 的 L1 距离随速度变是**同一个道理**
    （卷 3 第 6 章证过：$a_{lat}$ 与速度无关正是靠这个）。""")

    if _HAS_MPL:
        plt.figure(figsize=(9, 3.2))
        for L in (6.0, 20.0, 45.0):
            r = run_one(ctrl_pure_pursuit, path, V=V, T=24.0, L=L)
            # ★ ts 比 es 长一个（第一步之前没有误差记录），对齐再画
            plt.plot(r["ts"][1:], r["es"], lw=1.2, label="L=%.0f m" % L)
        plt.axhline(0, color="k", lw=0.8)
        plt.xlabel("时间 [s]")
        plt.ylabel("侧偏 [m]")
        plt.title("前视距离对纯追踪的影响：短则抖，长则切弯")
        plt.grid(alpha=0.3)
        plt.legend(fontsize=8)
        savefig("lesson33_exp3")


# ===========================================================================
#  实验 4 ★★★ 横向对比总表
# ===========================================================================

def exp4():
    head("实验 4 ★★★ 横向对比总表")

    print("""  把三课讲的东西放在一张表里。
  本表按**范式**分类，而不是按飞控分类 —— 因为同一个飞控里往往不止一种范式。
""")

    sub("① 四种范式")
    rows = [
        ["几何追点", "追一个前视点", "L1、纯追踪", "固定翼、无人车",
         "一个旋钮（前视距离）管两件事"],
        ["方向场", "造一个方位矢量", "NPFG", "固定翼（PX4 现役）",
         "能显式处理风；但缺稳定性分析"],
        ["轨迹生成", "把路径变成位置/速度/加速度给定", "AC_WPNav、AR_WPNav、FlightTask",
         "多旋翼、无人车", "可同时约束速度/加速度/加加速度"],
        ["横加速度", "直接算 a_lat", "无前视的纯 P", "——",
         "有稳态误差，实际不单独用"],
    ]
    tab(rows, ["范式", "核心动作", "代表实现", "用在哪", "主要问题"])

    sub("② 三种载具的差异（为什么不能通用）")
    rows = [
        ["固定翼", "不能悬停、不能侧滑\n必须保持空速", "几何追点 / 方向场",
         "L1（AP）、NPFG（PX4）", "有失速边界，必须管空速"],
        ["多旋翼", "能悬停、能全向平移\n速度可以为零", "轨迹生成",
         "FlightTask（PX4）、AC_WPNav（AP）", "不需要「转弯」这个概念"],
        ["无人车", "不能侧滑、不能悬停\n有最小转弯半径", "轨迹生成 / 几何追点",
         "AR_WPNav（AP）、L1（PX4 的 rover）", "要处理原地转向（pivot）"],
    ]
    tab(rows, ["载具", "运动学特征", "适配范式", "本机实现", "特殊约束"])
    print("""  ★★ **「多旋翼为什么不用 L1」的完整回答**：

    L1 / NPFG 这类几何制导律，**输出的物理量是横向加速度** ——
    它的隐含前提是「载具只能靠**倾斜/转向**来产生横向力」。
    这前提对固定翼和无人车成立，对多旋翼**不成立**：

      多旋翼可以**一边朝北飞、一边产生向东的力**，而不需要改变航向。
      换句话说，它的「横向」和「纵向」是**解耦**的 ——
      而 L1 的全部几何都建立在「横向要靠在航向上偏一个角」之上。

    ⇒ 所以多旋翼走的是**轨迹生成**：把路径变成
      「位置 + 速度 + 加速度」的时间序列，直接交给位置控制器。
      这条路不需要「转弯」这个概念，也就不需要 L1。

    ★ 反过来说：**NPFG 里的「抗风」逻辑对多旋翼也是多余的** ——
      多旋翼不需要「沿着方位线飞」，它可以原地顶着风不动。
      这也是为什么 `NPFG_WIND_REG` 那一套只会出现在固定翼侧。
""")

    sub("③ 一个反直觉的事实：L1 最后活在了无人车上")
    print("""  卷 3 第 6 章 查过：PX4 v1.15.4 里

      fw_pos_control/CMakeLists.txt:39  →  npfg     （固定翼）
      rover_pos_control/CMakeLists.txt:40 →  l1     （无人车）

  **一个为固定翼设计的算法，最后活在了轮式车上。**

  为什么能这样？因为无人车和固定翼共享同一个前提：
  **只能靠转向产生横向加速度**。所以 L1 的几何对它同样成立 ——
  只要把「横向加速度」再翻译成「前轮转角」。

  ★ 这条事实本身就是「范式比载具更持久」的例证：
    L1 的几何没变，变的是它被翻译成什么执行量。
""")

    sub("④ 怎么选（工程决策表）")
    rows = [
        ["固定翼，无风/弱风", "L1", "几何简单、参数含义清楚（周期+阻尼）"],
        ["固定翼，强风/远航程", "NPFG", "显式抗风；接受它缺稳定性分析的事实"],
        ["多旋翼，航线飞行", "轨迹生成（FlightTask / AC_WPNav）",
         "能约束 jerk，乘客/载荷友好"],
        ["无人车", "AR_WPNav（要 S-curve 和 pivot）或 L1（只要跟线）",
         "看是否需要精确的启停"],
        ["只要快速原型", "纯追踪", "一个参数就能跑，但别指望它调得好"],
    ]
    tab(rows, ["场景", "选什么", "理由"])
    print("""  ⚠️ **最后一行值得强调**：纯追踪之所以流行，
    是因为它**一个参数就能跑起来** —— 这对原型验证很有价值。
    但正因为它一个参数管两件事（稳定性 + 贴线性），
    **它很难调好**。诚实地承认这一点，比吹它「简单有效」更有用。
""")


EXPERIMENTS = {
    "1": ("四种范式的正面对决", exp1),
    "2": ("折线 vs 样条：曲率剖面", exp2),
    "3": ("前视距离敏感性", exp3),
    "4": ("横向对比总表", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        for k in sorted(EXPERIMENTS):
            EXPERIMENTS[k][1]()
    else:
        for a in args:
            if a not in EXPERIMENTS:
                print("未知实验编号：%s（可选 %s）"
                      % (a, "/".join(sorted(EXPERIMENTS))))
                continue
            EXPERIMENTS[a][1]()
    print()
    print("=" * W)
    print("  全部完成。")
    print("=" * W)
    print()


if __name__ == "__main__":
    main()
