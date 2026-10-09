# -*- coding: utf-8 -*-
"""用涡格法(VLM)独立验算小展弦比三角翼的**中性点**（= 气动中心）。

## 为什么需要这个脚本

`lesson09k_delta_wing.py` 里：

    x_cg_mac = 0.40      # 硬编码假设
    x_ac_mac = 0.48      # 硬编码假设（脚本自注 [B] 估计值）
    SM  = x_ac_mac - x_cg_mac          # = 0.08
    Cm_a = cm_alpha(a, SM)             # C_m,α = -a·SM

于是 `c_m_a / c_lift_a ≡ -0.08` —— **除回去当然还是 0.08**，
那正是输进去的假设，**零信息量**。所以静稳定裕度**从来没有被验证过**。

本脚本用势流涡格法**从几何直接算**气动中心：
不需要 `c_m_a`，不需要 `c_y_a`，也不需要任何关于 AC 位置的假设。

## ⚠️ 两条必须知道的边界

1. **VLM 是势流，不含涡升力**。它给的是"附着流气动中心"。
2. **不含机身**。机身/整流件会把中性点**往前拉**（见西蒙斯《模型飞机空气动力学》
   §12.18），所以真实中性点只会比本脚本算出的**更靠前** —— 即本结果是**乐观侧**。

## 自证

先跑 `self_test()`：矩形翼的升力线斜率必须对上 Helmbold、气动中心必须落在 25% 弦附近。
不过关就直接 assert 失败 —— 见本项目的一条教训：
**量具必须先自证在工作**。

用法：
    python neutral_point.py
"""
import math
import numpy as np

from vlm import make_planform, solve, seg_velocity, horseshoe_velocity


# ═══════════════════════════════════════════════════════════════════
#  平面形状参数（与 lesson09k_delta_wing.py 的 Geom 保持一致）
# ═══════════════════════════════════════════════════════════════════
B       = 2.50      # 翼展 m
C_ROOT  = 1.80      # 根弦 m
C_TIP   = 0.25      # 尖弦 m
X_CG_MAC = 0.40     # 重心（MAC 比例）—— 这是被测的**假设**，不是测量值


def planform(b=B, c_root=C_ROOT, c_tip=C_TIP, sweep_deg=None):
    """后掠前缘 + 平直后缘。sweep_deg=None 时用真实几何（根尖弦差决定）。"""
    dx = (c_root - c_tip) if sweep_deg is None else (b / 2.0) * math.tan(math.radians(sweep_deg))
    le = lambda y: (abs(y) / (b / 2.0)) * dx
    te = lambda y: c_root
    return le, te


def mac_and_le(panels):
    """"平均气动弦" + "MAC 前缘位置"。

    ★★ MAC 前缘必须用**弦长加权均值**从平面形状直接算，不要套公式：

        x̄_MAC = (1/S) ∮ c(y)·x_le(y) dy

    本项目已经栽过一次"根弦比例 / MAC 比例混算"（见 lesson09k 的注释），
    这里从几何直接推，杜绝再次混淆。
    """
    strips = {}
    for p in panels:
        k = (p['y'], p['dy'])
        # ★ 前缘要取**该条里最小的 x0**，不能直接写 p['x0'] ——
        #   同一条的 8 个弦向面元共用这个字典键，后写的会覆盖前面的，
        #   取到的是**最后一个弦向面元**的前缘（矩形翼上差 7/8 弦长）。
        #   自证里 AC 因此算成 -0.636 MAC（应约 +0.24），当场抓出来。
        prev = strips.get(k)
        le = p['x0'] if prev is None else min(prev[1], p['x0'])
        strips[k] = (p['strip_c'], le)                        # (弦长, 前缘x)
    S = sum(dy * c for (_, dy), (c, _) in strips.items())
    mac = sum(dy * c ** 2 for (_, dy), (c, _) in strips.items()) / S
    mac_le = sum(dy * c * xl for (_, dy), (c, xl) in strips.items()) / S
    return mac, mac_le, S


def aero_center(le, te, n_span=48, n_chord=12, a1=1.0, a2=5.0):
    """返回气动中心的**绝对位置**（从根弦前缘量起）与其 MAC 比例。"""
    panels = make_planform(B, te, le, n_span, n_chord)
    mac, mac_le, S = mac_and_le(panels)
    CL1, Cm1, _, _ = solve(panels, a1)
    CL2, Cm2, _, _ = solve(panels, a2)
    dCm_dCL = (Cm2 - Cm1) / (CL2 - CL1)
    x_ac = -mac * dCm_dCL                 # 力矩写在 x=0（根弦前缘）
    return dict(x_ac=x_ac, x_ac_mac=(x_ac - mac_le) / mac,
                mac=mac, mac_le=mac_le, S=S,
                slope=(CL2 - CL1) / math.radians(a2 - a1))


def self_test():
    """在**已知答案**的矩形翼上自证。不过关就炸。"""
    print('=' * 66)
    print('自证：矩形翼的升力线斜率与气动中心')
    ok = True
    for b, c, tol in ((6.0, 1.0, 0.08), (2.0, 1.0, 0.08)):
        le = lambda y: 0.0
        te = lambda y: c
        panels = make_planform(b, te, le, 32, 8)
        mac, mac_le, S = mac_and_le(panels)
        CL1, Cm1, _, _ = solve(panels, 1.0)
        CL2, Cm2, _, _ = solve(panels, 5.0)
        slope = (CL2 - CL1) / math.radians(4)
        x_ac_mac = (-mac * (Cm2 - Cm1) / (CL2 - CL1) - mac_le) / mac
        AR = b / c
        helm = 2 * math.pi * AR / (2 + math.sqrt(AR ** 2 + 4))
        e1 = abs(slope - helm) / helm
        e2 = abs(x_ac_mac - 0.25)
        print('  AR=%-4.2f 斜率 %6.3f (Helmbold %6.3f, 差 %5.1f%%)  AC=%.4f MAC  S=%.4f'
              % (AR, slope, helm, 100 * e1, x_ac_mac, S))
        if e1 > tol:
            print('    ❌ 斜率超出 %d%% 容差' % (100 * tol))
            ok = False
        if e2 > 0.06:
            print('    ❌ AC 偏离 0.25 超过 0.06')
            ok = False
    assert ok, 'VLM 自证失败 —— 下面的数一个都不能用'
    print('  ✅ 自证通过')
    print('=' * 66)


def main():
    self_test()

    le, te = planform()
    r = aero_center(le, te)
    print()
    print('平面形状：b=%.2f  根弦=%.2f  尖弦=%.2f  λ=%.4f  Λ_LE=%.2f°'
          % (B, C_ROOT, C_TIP, C_TIP / C_ROOT,
             math.degrees(math.atan((C_ROOT - C_TIP) / (B / 2)))))
    print('MAC = %.4f m   MAC 前缘距根弦前缘 = %.4f m   S = %.4f m²'
          % (r['mac'], r['mac_le'], r['S']))
    print()
    print('VLM 算出的气动中心：')
    print('  x_ac = %.4f m（根弦前缘量起）  =  %.4f MAC' % (r['x_ac'], r['x_ac_mac']))
    print('  （升力线斜率 %.3f /rad，lesson09k 的 Helmbold 给 2.973 —— 两条独立路径）'
          % r['slope'])
    print()
    print('─' * 66)
    print('静稳定裕度  SM = x_ac − x_cg = %.4f − %.4f MAC' % (r['x_ac_mac'], X_CG_MAC))
    sm = r['x_ac_mac'] - X_CG_MAC
    print('             SM = %+.4f MAC   →  %s'
          % (sm, '静稳定 ✅' if sm > 0 else '★ 静不稳定 ❌'))
    print('─' * 66)
    print()
    print('中性点落在 %.2f%% MAC。重心必须放在它**之前**。' % (100 * r['x_ac_mac']))
    print('当前假设 x_cg_mac = %.2f → 重心比中性点靠后 %.2f%% MAC（= %.3f m）'
          % (X_CG_MAC, 100 * abs(sm), abs(sm) * r['mac']))
    print()
    print('若要让静稳定裕度取常见值，重心应放到：')
    for target in (0.05, 0.08, 0.15):
        print('  SM=%.0f%% MAC  →  x_cg_mac = %.3f  （从根弦前缘量 %.3f m）'
              % (100 * target, r['x_ac_mac'] - target, r['mac_le'] + (r['x_ac_mac'] - target) * r['mac']))
    print()
    print('⚠️ 两条边界：① VLM 不含涡升力；② 不含机身。')
    print('   机身会把中性点**往前拉**，所以上面这个中性点是**乐观侧**。')


if __name__ == '__main__':
    main()
