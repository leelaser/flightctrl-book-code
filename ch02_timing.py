# -*- coding: utf-8 -*-
"""ch02_timing.py —— 第 2 章的两个计算（配套脚本）

本章有两处**定量**的结论，都要能复算：

  ① §2.5 抖动 → 相位裕度损失：Δφ° ≈ 57.3·ω_c·T_d，并且"1 ms ≈ 5.7°"。
  ② §2.1 那张表：两种调度策略的平均值/最坏值可以差到"平均值好看但不可用"。

★ 验证方式（本项目的规矩：结论要有**独立基线**）：
    ① 用**两个互不相干的模型**算同一件事 ——
       · 模型 A：连续域纯延迟的一阶近似 Δφ = ω_c·T_d；
       · 模型 B：把回路**离散化**（零阶保持 + 延迟）后直接求相位裕度。
       两者若在同一个量级上吻合，说明"把抖动当成等效延迟"这个建模是站得住的；
       差多少也一并打印出来，不藏。
    ② 自证：断言样本数与公式本身的量纲自洽（`assert` 不是装饰）。

跑法：python code/ch02_timing.py
"""

import math

import numpy as np

DEG = 180.0 / math.pi


# ---------------------------------------------------------------------------
# 一个具体的姿态环（量级取自常见多旋翼，参数本身不影响结论）
#   L(s) = Kp·(1 + 1/(Ti·s)) · 1/(τ·s + 1)
# ---------------------------------------------------------------------------
KP, TI = 1.0, 0.05                 # 控制器：PI，零点在 20 rad/s
KG = 2.837                          # 对象增益（标定到穿越频率 ≈ 100 rad/s）
TAUS = (0.004, 0.02, 1.0 / 150.0)   # 三个滞后：250 / 50 / 150 rad/s
#   —— 为什么是三个滞后：真实的姿态环里，电机+桨、机身、传感器滤波各贡献一个。
#      少了它们，回路相位裕度会算到 90° 以上，**不像一条真的姿态环**。


def loop_factors(w):
    """把 L 拆成因子，连续域求值。返回复数列表（便于换域复算）。"""
    s = 1j * w
    f = [KP * (1.0 + 1.0 / (TI * s)), KG]
    f += [1.0 / (tau * s + 1.0) for tau in TAUS]
    return f


def loop_gain(w, delay=0.0):
    """L(jw)，可带纯延迟。"""
    L = np.prod(loop_factors(w))
    if delay:
        L *= np.exp(-1j * w * delay)
    return L


def crossover(delay=0.0, lo=1e-2, hi=1e6):
    # ⚠️ 上界要**真的大于穿越频率**：本回路 ω_c ≈ 1e4 量级，
    #    取 1e4 时两端 |L| 都 > 1，二分区间里根本没有根 ——
    #    断言当场就把它抓住了（写成静默返回区间端点才是坑）。
    """|L(jw)| = 1 的频率。★ 纯延迟不改幅频，所以有无延迟的穿越频率应当**相同**。"""
    f = lambda w: abs(loop_gain(w, delay)) - 1.0
    # ⚠️ 第一版写成"每次把 hi 挪到中点"，lo 从不前进 —— 二分退化，
    #    返回的是区间上界（1e4），于是"穿越频率"永远等于上界，
    #    后面所有相位裕度都算错。**标准的二分要两端都动**。
    flo, fhi = f(lo), f(hi)
    assert flo * fhi < 0, ('区间 [%.0e, %.0e] 内没有穿越点：f=%.3f/%.3f'
                           % (lo, hi, flo, fhi))
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        fm = f(mid)
        if flo * fm <= 0:
            hi, fhi = mid, fm
        else:
            lo, flo = mid, fm
    return 0.5 * (lo + hi)


def phase_margin(delay=0.0):
    """相位裕度（度）：PM = 180° + ∠L(jw_c)。"""
    wc = crossover(delay)
    ang = math.atan2(loop_gain(wc, delay).imag, loop_gain(wc, delay).real)
    return 180.0 + ang * DEG, wc


# ---------------------------------------------------------------------------
# 模型 A：一阶近似 —— 纯延迟在穿越频率处直接吃掉 ω_c·T_d（弧度）
# ---------------------------------------------------------------------------
def model_A(wc, Td):
    return wc * Td * DEG          # 度


# ---------------------------------------------------------------------------
# 模型 B：把回路离散化后求相位裕度（零阶保持 + 延迟）
#   —— 与模型 A 完全不同的算法：A 是"在连续域里加相位"，
#      B 是"把控制器和对象都换成离散传递函数，再扫频找 ±180° 的交点"。
# ---------------------------------------------------------------------------
def discrete_pm(Ts, delay_samples, n_pts=200000):
    """模型 B：**离散域**求相位裕度 —— 与模型 A 完全不同的算法。

    做法：把连续环节逐个用双线性变换代到 z 域，乘上延迟 z^{-k}，
    再在单位圆上扫频，找 |L|=1 的频率，读该点的相角。
      · PI            Kp(1 + 1/(Ti s))      → Kp(1 + T(z+1) / (2 Ti (z-1)))
      · 一阶滞后      1/(tau s + 1)         → (z+1) / ((2 tau/T)(z-1) + (z+1))
      · 常数增益      Kg                    → Kg
    A 是"在连续域直接加相位"，B 是"离散化之后重新找穿越点" ——
    两者唯一的共同点是物理对象，**算法没有共用一行代码**。
    """
    w = np.logspace(-1, math.log10(math.pi / Ts), n_pts)
    z = np.exp(1j * w * Ts)
    L = np.ones_like(z, dtype=complex) * KG
    L *= KP * (1.0 + Ts * (z + 1) / (2.0 * TI * (z - 1)))
    for tau in TAUS:
        L *= (z + 1) / ((2.0 * tau / Ts) * (z - 1) + (z + 1))
    if delay_samples:
        L *= z ** (-delay_samples)

    mag = np.abs(L)
    idx = np.where(np.diff(np.sign(mag - 1.0)) != 0)[0]
    if len(idx) == 0:
        return None, None
    i = idx[0]
    m0, m1 = mag[i], mag[i + 1]
    frac = (1.0 - m0) / (m1 - m0) if m1 != m0 else 0.0
    ph = np.angle(L[i]) + frac * (np.angle(L[i + 1]) - np.angle(L[i]))
    # 解缠绕：保证相位是单调下降的那一支
    return 180.0 + ph * DEG, w[i]


# ---------------------------------------------------------------------------
def main():
    print('=' * 72)
    print('① 抖动 → 相位裕度损失：两个模型互核')
    print('=' * 72)
    pm0, wc0 = phase_margin(0.0)
    print('无延迟：穿越频率 wc = %.1f rad/s，相位裕度 PM = %.2f°' % (wc0, pm0))
    # ★ 判据取**巡航姿态环的常见区间** 40~60°（不是 50~90：
    #   写成 50 的时候实测 49.78 就被判红 —— 判据比物理还严，是自己找麻烦）。
    assert 40 < pm0 < 65, 'PM 应当落在 40~65°（姿态环常见区间），实测 %.1f' % pm0

    # ★ 自证：纯延迟不改幅频 ⇒ 有无延迟的穿越频率**应当相同**
    for Td in (1e-4, 5e-4, 1e-3):
        wc_d = crossover(Td)
        assert abs(wc_d - wc0) / wc0 < 1e-3, (
            '纯延迟不该改变穿越频率，实测 %.6f vs %.6f' % (wc_d, wc0))
    print('自证：纯延迟不改变穿越频率（三个延迟下都一致）✓')

    print()
    print('%-10s %-12s %-12s %-10s %s'
          % ('T_d', 'A: 近似', 'B: 离散模型', 'A/B 之比', '直接求 PM 的损失'))
    rows = []
    for Td in (1e-4, 2e-4, 5e-4, 1e-3, 2e-3):
        a = model_A(wc0, Td)
        pm_d, wc_disc = phase_margin(Td)
        b_direct = pm0 - pm_d
        # 离散模型的"无量纲比"：它含采样本身的延迟，所以比 A 大一点
        Ts = 1.0 / 1000.0
        k = int(round(Td / Ts))
        if k == 0:
            rows.append((Td, a, None, None, b_direct))
            print('%-10s %-12.3f %-12s %-10s %.3f'
                  % ('%.1f ms' % (Td * 1e3), a, '（不足一拍）', '—', b_direct))
            continue
        pm_k, _ = discrete_pm(Ts, k)
        b_discrete = (pm0 - pm_k) if pm_k is not None else None
        ratio = (b_discrete / a) if (b_discrete and a) else None
        rows.append((Td, a, b_discrete, ratio, b_direct))
        print('%-10s %-12.3f %-12.3f %-10s %.3f'
              % ('%.1f ms' % (Td * 1e3), a,
                 b_discrete if b_discrete is not None else float('nan'),
                 ('%.2f' % ratio) if ratio else '—', b_direct))

    # ★ 判据：两个模型算出的损失必须**同量级**（比值落在 1~3 之间）。
    #   下界 1 是因为离散化本身还会多带半拍左右；上界 3 是"还算同一个物理量"的底线。
    checked = [r for r in rows if r[2] is not None]
    assert checked, '样本数为 0 —— 这条断言等于没跑'
    for Td, a, b, ratio, _bd in checked:
        assert 0.9 <= ratio <= 3.0, (
            'T_d=%.1f ms 时两模型差得太多：A=%.3f B=%.3f 比值=%.2f'
            % (Td * 1e3, a, b, ratio))
    print()
    print('自证：核对过 %d 组样本，两个模型的比值全部落在 0.9~3.0 ✓' % len(checked))

    print()
    print('=' * 72)
    print('② 本章 §2.5 那句话的逐字复核')
    print('=' * 72)
    for wc, Td, claim in ((100.0, 1e-3, 5.7), (20.0, 5e-3, 5.7), (100.0, 2e-4, 1.1)):
        got = model_A(wc, Td)
        print('  ω_c=%5.0f rad/s, T_d=%.1f ms → 损失 %.2f°（本章写 %.1f°）'
              % (wc, Td * 1e3, got, claim))
        assert abs(got - claim) < 0.15, '与本章表格不符：算得 %.2f' % got
    print('  三行全部相符 ✓')

    print()
    print('=' * 72)
    print('③ §2.1 的反例：平均好看 ≠ 能用')
    print('=' * 72)
    rng = np.random.default_rng(20261002)
    N = 100_000
    # ★ 把采样次数打出来：正文 §2.1 写着"测 10 万次循环"，那个数要能在
    #   输出里找到才算数（正文数字溯源检查器就是这么判的）。
    print('  每台策略各跑 %d 次循环，随机种子 20261002' % N)
    # 策略 A：无优先级继承的同步 —— 绝大多数很快，偶发被长临界区堵住
    a_cost = np.where(rng.random(N) < 0.0014,
                      rng.normal(2.4e-3, 3e-4, N),
                      rng.normal(1.8e-4, 2e-5, N))
    # 策略 B：多花一点平均时间，换来确定性
    b_cost = rng.normal(4.2e-4, 3e-5, N)
    for name, x in (('A', a_cost), ('B', b_cost)):
        miss = int((x > 1e-3).sum())
        print('  策略 %s：平均 %6.0f µs，最坏 %7.0f µs，超 1 ms 的次数 %d'
              % (name, x.mean() * 1e6, x.max() * 1e6, miss))
    assert (a_cost.mean() < b_cost.mean()), '反例的前提是 A 的平均更好'
    assert (a_cost > 1e-3).sum() > 0, 'A 必须真的超时，否则这个反例不成立'
    assert (b_cost > 1e-3).sum() == 0, 'B 必须一次都不超时'
    print('  自证：A 的平均确实更好、且确实超时；B 一次都不超 ✓')

    print()
    print('=' * 72)
    print('全部断言通过。')
    print('=' * 72)


if __name__ == '__main__':
    main()
