# -*- coding: utf-8 -*-
"""ch07_geodesic.py —— 卷 7 第 12 章：平面近似 vs 测地线，差多少

第 12 章那个跟随模式的 ROS 包，从 V2 到 V5 有一处关键改动：
**经纬度换算从"平面近似"换成了 GeographicLib 的测地线精确解。**

本脚本把这两套算法**并排算一遍**，回答一个问题：

    那个平面近似到底差多少？值不值得换成测地线？

对照的两套做法（都来自那个包的真实代码）：

  ① 平面近似（V2，`src_target_publisher_Version2.cpp`）：
       static constexpr double M_PER_DEG_LAT = 111320.0;
       lat += meters / M_PER_DEG_LAT;
       lon += meters / (M_PER_DEG_LAT * std::cos(lat));
     —— 一个常数 111320，经度再乘个 cos(纬度)

  ② 测地线（V5，用 GeographicLib 的 `Geodesic::Direct`）：
     在 WGS84 椭球上正解，结果精确到库的精度

本脚本用 WGS84 的**曲率半径公式**算出"真值"，再和 ① 比。
（不是调用 GeographicLib —— 只需要椭球几何，公式是标准的。）

跑法：python code/ch07_geodesic.py
"""

import math

# WGS84 椭球参数
A = 6378137.0                     # 长半轴（m）
F = 1.0 / 298.257223563           # 扁率
E2 = F * (2 - F)                  # 第一偏心率平方

M_PER_DEG_FLAT = 111320.0         # V2 代码里那个常数


def radii(phi_deg):
    """给定纬度，返回 (子午圈曲率半径 M, 卯酉圈曲率半径 N)，单位 m。

    这两个量就是"椭球上，一度有多长"的答案：
      · 沿**南北**走 1 弧度 ≈ M
      · 沿**东西**走 1 弧度 ≈ N·cos(φ)
    """
    phi = math.radians(phi_deg)
    s2 = math.sin(phi) ** 2
    w = 1 - E2 * s2
    M = A * (1 - E2) / (w ** 1.5)
    N = A / math.sqrt(w)
    return M, N


def true_deg_lat_m(phi_deg):
    """该纬度上，1° 纬度对应多少米（真值）"""
    M, _ = radii(phi_deg)
    return M * math.pi / 180.0


def true_deg_lon_m(phi_deg):
    """该纬度上，1° 经度对应多少米（真值）"""
    _, N = radii(phi_deg)
    return N * math.cos(math.radians(phi_deg)) * math.pi / 180.0


def flat_deg_lon_m(phi_deg):
    """V2 的算法：111320 × cos(φ)"""
    return M_PER_DEG_FLAT * math.cos(math.radians(phi_deg))


def main():
    print('=' * 76)
    print('① 两套算法各是什么')
    print('=' * 76)
    print('  平面近似（V2）：')
    print('     每度纬度 = %.0f m（常数）' % M_PER_DEG_FLAT)
    print('     每度经度 = %.0f × cos(纬度)' % M_PER_DEG_FLAT)
    print()
    print('  测地线（V5）：在 WGS84 椭球上解，每度的长度**随纬度变化**：')
    print('     每度纬度 = M(φ) · π/180        M 是子午圈曲率半径')
    print('     每度经度 = N(φ) · cos(φ) · π/180   N 是卯酉圈曲率半径')
    print()
    print('  椭球参数：a = %.0f m，1/f = %.3f' % (A, 1 / F))

    # -----------------------------------------------------------------------
    print()
    print('=' * 76)
    print('② 每度差多少（中纬度，也就是那个包默认的位置）')
    print('=' * 76)
    # 包里的默认中心（PX4 SITL 的默认点）
    LAT0 = 47.397684
    print('  取纬度 %.6f°（那个包里 center_lat 的默认值）' % LAT0)
    M, N = radii(LAT0)
    print('  M(φ) = %.1f m    N(φ) = %.1f m' % (M, N))
    print()
    print('  %-8s %-14s %-14s %s' % ('', '平面近似', '测地线（真）', '相对误差'))
    tl = true_deg_lat_m(LAT0)
    print('  %-8s %-14s %-14s %.3f%%'
          % ('1° 纬度', '%.1f m' % M_PER_DEG_FLAT, '%.1f m' % tl,
             100 * (M_PER_DEG_FLAT - tl) / tl))
    tn = true_deg_lon_m(LAT0)
    fn = flat_deg_lon_m(LAT0)
    print('  %-8s %-14s %-14s %.3f%%'
          % ('1° 经度', '%.1f m' % fn, '%.1f m' % tn,
             100 * (fn - tn) / tn))
    print()
    print('  ★ 在**中纬度、短距离**下，平面近似的误差是 **千分之几**：')
    print('    换成一公里的位移，误差在米级以下 —— 对跟随演示够用了。')
    print('    **这就是为什么 V2 能跑起来、而且看上去"没问题"。**')

    # -----------------------------------------------------------------------
    print()
    print('=' * 76)
    print('③ ★ 但"随纬度恶化"这个预期是**错的** —— 误差其实是**饱和**的')
    print('=' * 76)
    print('  %-10s %-14s %-14s %-12s %s'
          % ('纬度', '平面 1°经度', '真值 1°经度', '相对误差', '走 1 km 错多少'))
    errs = []
    for lat in (0.0, 30.0, 47.397684, 60.0, 75.0, 85.0, 89.0):
        f_ = flat_deg_lon_m(lat)
        t_ = true_deg_lon_m(lat)
        err = (f_ - t_) / t_
        errs.append(err)
        m_err = abs(err) * 1000.0
        print('  %-10s %-14s %-14s %-12s %.2f m'
              % ('%.4f°' % lat, '%.1f m' % f_, '%.1f m' % t_,
                 '%.3f%%' % (100 * err), m_err))
    print()
    print('  ★★ 我原本的预期是"误差随纬度急剧恶化、在极区发散"。')
    print('     **实测不是**：它从 0.000% 一路涨到 -0.335% 之后就**饱和**了 ——')
    print('     连极点附近也不再变大。')
    print()
    print('  为什么？因为这个常数 %.0f 本身约等于**赤道处的每度长度**' % M_PER_DEG_FLAT)
    print('  (%.1f m)，而真值 N(φ)·π/180 在赤道到极点之间**只变化约 0.35%%**' % true_deg_lon_m(0.0))
    print('  （椭球很接近球，扁率只有 1/298）。所以这个近似的**相对误差有界**。')

    # -----------------------------------------------------------------------
    print()
    print('=' * 76)
    print('④ 那真正的失效边界在哪：不是纬度，是**距离**')
    print('=' * 76)
    print('  上面的表是把"每度多长"这一个**比例**算错了多少 —— 它有界。')
    print('  但 V2 的代码里还有一处更容易被忽略的做法：')
    print()
    print('      lon += meters / (M_PER_DEG_LAT * cos(**center_lat**))')
    print()
    print('  ★ 注意那个 `center_lat` 是**写死的中心点纬度**，')
    print('    所有目标点都用它 —— 哪怕目标已经飞出去很远。')
    print('    而 cos(纬度) 是随纬度变的 ⇒ **目标离中心越远，这个 cos 越不对**。')
    print()
    print('  %-16s %-14s %-12s %-14s %s'
          % ('往北飞', '纬度变成', 'cos 变化', '经度换算误差', '（1 km 位移）'))
    for dn_km in (1.0, 10.0, 50.0, 100.0, 200.0):
        dphi = dn_km * 1000.0 / true_deg_lat_m(LAT0)
        lat2 = LAT0 + dphi
        c1 = math.cos(math.radians(LAT0))
        c2 = math.cos(math.radians(lat2))
        err = (c1 - c2) / c2
        print('  %-16s %-14s %-12s %-14s %.1f m'
              % ('%.0f km' % dn_km, '%.4f°' % lat2,
                 '%.3f%%' % (100 * (c2 - c1) / c1),
                 '%.3f%%' % (100 * err), abs(err) * 1000.0))
    print()
    print('  ★★ 这才是那个"已知失效边界"的真身：')
    print('     **每度多长**算错了千分之几（有界）；')
    print('     但**用哪个纬度去算 cos**这件事，误差随距离**线性增长**。')
    print('     飞到 100 km 外，1 km 的横向位移就错了约 17.5 m（见上表）。')

    # -----------------------------------------------------------------------
    print()
    print('=' * 76)
    print('⑤ 那 V5 换测地线，换来了什么')
    print('=' * 76)
    print('  %-30s %-20s %s' % ('', '平面近似（V2）', '测地线（V5）'))
    rows = [
        ('中纬度、短距离', '够用（千分之几）', '精确'),
        ('远距离（百 km 级）', '**误差随距离累积**', '无累积'),
        ('用了哪个纬度算 cos', '写死中心点', '每点各自算'),
        ('代码依赖', '无（几行算术）', '需要 libgeographic-dev'),
    ]
    for a, b, c in rows:
        print('  %-30s %-20s %s' % (a, b, c))
    print()
    print('  ★★ 换句话说：V5 换测地线**不是为了把中纬度那千分之几再压小**，')
    print('     而是为了**去掉"用中心点纬度代表全图"这个隐含假设**。')
    print()
    print('  ⇒ 这是工程上常见的一类升级：')
    print('    **当前场景没出错，但当前的近似有一个"已知的失效边界"。**')
    print('    判断值不值得换，就看那个边界会不会被碰到。')

    # -----------------------------------------------------------------------
    print()
    print('=' * 76)
    print('⑥ 和本章主题的关系')
    print('=' * 76)
    print('  第 10 章说"坐标系是跨系统集成里最容易出错的两件事之一"。')
    print('  这个包的经历是那句话的具体版本：')
    print()
    print('    · 它**没有报错**，V2 一直跑得好好的')
    print('    · 它的误差**在当前场景里小到看不出来**（千分之几）')
    print('    · 但它有一个**已知的失效边界**（远距离时"用中心点纬度"不再成立）')
    print()
    print('  ★★ 最危险的不是"错了"，而是"**在你看的那个场景里恰好是对的**"。')
    print('     一个误差随距离增长的近似，会在整个开发与演示期都表现正常 ——')
    print('     直到有人把它开到几十公里外。')
    print()
    print('  ⇒ 所以第 10 章那句"起飞前用静态数据验一遍坐标系"还可以再进一步：')
    print('    **验的时候要用"你打算飞的最远距离"来验**，')
    print('    而不是在起飞点附近转两圈就算数 ——')
    print('    因为这类误差的特性正是"**近处没有、远处才现形**"。')

    # 断言：证明上面那些说法确实成立（而不是看着像成立）
    # ① 中纬度的误差是千分之几
    assert abs((M_PER_DEG_FLAT - true_deg_lat_m(47.4)) / true_deg_lat_m(47.4)) < 0.005, \
        '中纬度的纬度换算误差应当是千分之几'
    # ② ★ 相对误差**有界**（这正是推翻我原先"发散"预期的那条）
    assert max(abs(e) for e in errs) < 0.01, \
        '平面近似的相对误差应当有界（实测最大 %.3f%%）' % (100 * max(abs(e) for e in errs))
    # ③ ★ 而"用固定中心纬度"带来的误差**随距离增长**（这才是真边界）
    def _north_err(km):
        lat2 = LAT0 + km * 1000.0 / true_deg_lat_m(LAT0)
        c1 = math.cos(math.radians(LAT0)); c2 = math.cos(math.radians(lat2))
        return abs((c1 - c2) / c2)
    assert _north_err(200.0) > 5 * _north_err(40.0), \
        '固定纬度带来的误差应当随距离增长（实测 200 km 时约 %.2f%%）' \
        % (100 * _north_err(200.0))

    print()
    print('  （上面三条说法都有断言兜底：误差有界、且随距离增长，见脚本末尾。）')
    print()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
