# -*- coding: utf-8 -*-
"""涡格法(VLM)求气动中心 —— 用来独立验算 x_ac_mac = 0.48 这个假设。

为什么自己写：
    lesson09k 里的 x_ac_mac = 0.48 是**硬编码的估计值**，
    而 c_m_a = -a·(x_ac-x_cg) 又是由它算出来的。
    所以任何"从 c_m_a 反推静稳定裕度"的做法都是循环论证（见文档 07 §2④）。

    VLM 是**势流解**：只要求出每个展向/弦向面元的环流，
    升力与俯仰力矩就完全由几何决定，**不需要任何关于 AC 位置的假设**。
    这是能拿到的、最干净的独立基准。

⚠️ VLM 是势流，**不含涡升力**。所以它给的是"附着流气动中心"，
   对三角翼是**偏前**的下界。这一点必须写进结论，不能当成真值。

自证（先跑 validate()）：
    矩形翼 AR=6 → 升力线斜率应接近 Helmbold 解，AC 应落在 25% 弦附近。
    过不了这一关，下面的数一个都不能信。
"""
import numpy as np

# ─────────────────────────────────────────────────────────────
#  Biot–Savart：直线涡段 A→B（强度 G）在 P 点诱导的速度
# ─────────────────────────────────────────────────────────────
def seg_velocity(P, A, B, G):
    r1 = P - A
    r2 = P - B
    cross = np.cross(r1, r2)
    n2 = cross @ cross
    if n2 < 1e-20:
        return np.zeros(3)
    n1 = np.linalg.norm(r1)
    n2n = np.linalg.norm(r2)
    # ★★ r0 必须**保持有量纲**（= r2 − r1，不归一化）。
    #    量纲检查：cross/n2 是 1/长度；r0·(r̂1 − r̂2) 必须带长度量纲才能配平。
    #    归一化后会丢掉一个长度因子 —— 长涡段被系统性低估
    #    （实测：far=50 的尾涡被低估 50 倍），升力线斜率因此掉到 1/2.7。
    #    这是本项目里第三次栽在"量纲没检查"上。
    r0 = r2 - r1
    # ★ 整体再加负号：本式与"直接积分 Biot–Savart"逐分量对比恰好差一个符号。
    #   数值积分是无歧义的基准（v = Γ/4π ∮ (dl × r̂)/r²），所以以它为准。
    return -(G / (4 * np.pi)) * (cross / n2) * (r0 @ (r1 / n1 - r2 / n2n))


def horseshoe_velocity(P, A, B, G):
    """马蹄涡：无限长尾涡从 B 拖到 +x 无穷远，再从 -x 无穷远回到 A。

    坐标系：x 沿弦向后（来流沿 +x），y 展向，z 向上。
    """
    far = 1.0e6
    v = seg_velocity(P, A, B, G)                                  # 附着涡 A→B
    # ★ 两条后拖涡都从附着涡的端点**向下游(+x)延伸到无穷远**。
    #   B 端：沿 +x 方向拖出；A 端：同一条线，但**行程方向相反**（从 +∞ 回到 A）。
    #   初版把 A 端写成了"从 −∞ 过来"，等于在机翼上游放了一条涡线，
    #   把升力抵掉了大半（自证里斜率只有 Helmbold 的 37%）。
    v += seg_velocity(P, B, B + np.array([far, 0, 0]), G)
    v += seg_velocity(P, A + np.array([far, 0, 0]), A, G)
    return v


# ─────────────────────────────────────────────────────────────
#  平面形状：给定 LE 与 TE 的 x(y)
# ─────────────────────────────────────────────────────────────
def make_planform(b, te_x, le_x, n_span, n_chord):
    """把半展 [-b/2, b/2] 切成 n_span 条、每条弦向切 n_chord 格。

    返回面元列表：(y_center, dy, x_le, x_te, x_quarter, x_threequarter)
    """
    panels = []
    ys = np.linspace(-b / 2, b / 2, n_span + 1)
    for i in range(n_span):
        y0, y1 = ys[i], ys[i + 1]
        yc = 0.5 * (y0 + y1)
        dy = y1 - y0
        # 用该条的**中心**处的弦长（梯形用中点即可，误差 O(dy²)）
        xle = le_x(yc)
        xte = te_x(yc)
        c = xte - xle
        if c <= 1e-9:
            continue
        for j in range(n_chord):
            f0 = j / n_chord
            f1 = (j + 1) / n_chord
            xa = xle + f0 * c
            xb = xle + f1 * c
            panels.append(dict(y=yc, dy=dy, strip_c=c,     # ★ 条的弦长（算 MAC 用）
                               x0=xa, x1=xb,
                               xq=0.75 * xa + 0.25 * xb,     # 附着涡在 1/4 弦
                               xc=0.25 * xa + 0.75 * xb))    # 配置点在 3/4 弦
    return panels


def solve(panels, alpha_deg, V=1.0):
    """给定迎角，解出每个面元的环流；返回 CL、Cm（绕 x=0，抬头为正）。"""
    a = np.radians(alpha_deg)
    # 来流方向（沿 +x），攻角 a 时机体法向速度分量
    # 边界条件：气流不能穿过面元 → V·n + 诱导法向 = 0
    # 面元法向近似取 z 向（薄翼、小迎角）
    n = len(panels)
    A = np.zeros((n, n))
    rhs = np.zeros(n)
    for i, pi in enumerate(panels):
        Pi = np.array([pi['xc'], pi['y'], 0.0])
        for j, pj in enumerate(panels):
            Aj = np.array([pj['xq'], pj['y'] - pj['dy'] / 2, 0.0])
            Bj = np.array([pj['xq'], pj['y'] + pj['dy'] / 2, 0.0])
            v = horseshoe_velocity(Pi, Aj, Bj, 1.0)      # 单位环流的影响系数
            A[i, j] = v[2]                                # 只要 z 向（法向）
        # ★ 符号：总法向速度 = 来流法向 + 诱导法向 = 0
        #   来流沿 +x 且 α>0（前缘抬起）→ 来流法向 = −V sinα
        #   故诱导法向必须 = +V sinα，即 rhs = +V sinα。
        #   ⚠️ 这里曾经和 seg_velocity 的符号错**互相抵消**过 ——
        #      两个错一起犯时端到端结果看着是对的（斜率、AC 都对），
        #      所以只靠端到端自证抓不出来。是"逐分量对数值积分"才逼出来的。
        rhs[i] = -V * np.sin(a)

    G = np.linalg.solve(A, rhs)

    S = sum(p['dy'] * (p['x1'] - p['x0']) for p in panels)
    # 升力 ∝ Σ Γ·Δy（Kutta–Joukowski）
    L = sum(G[i] * panels[i]['dy'] for i in range(n))
    CL = 2.0 * L / (V * S)
    # 俯仰力矩绕 x=0（抬头为正）：升力作用在 1/4 弦处
    M = -sum(G[i] * panels[i]['dy'] * panels[i]['xq'] for i in range(n))
    cbar = mac_of(panels)
    Cm = 2.0 * M / (V * S * cbar)
    return CL, Cm, cbar, S


def mac_of(panels):
    """平均气动弦 = (1/S)∮c²dy —— 必须按**条**累加，不能按弦向面元累加。

    ★ 初版就是在这里错的：面元的弦长是 c/n_chord，
      但弦向有 n_chord 个面元，Σ(c/n)² × n = c²/n，把 MAC 算成了 1/n。
      自证里 cbar=0.1667（应为 1.0）当场抓住了它。
    """
    strips = {}
    for p in panels:
        strips[(p['y'], p['dy'])] = p['strip_c']       # 每个展向条只记一次
    S = sum(dy * c for (_, dy), c in strips.items())
    m = sum(dy * c ** 2 for (_, dy), c in strips.items()) / S
    return m


def aero_center(b, te_x, le_x, n_span=24, n_chord=6, a1=1.0, a2=5.0):
    """用两个迎角求 dCm/dCL，再定 AC：x_ac = x0 - cbar·(dCm/dCL)，x0 = 0。"""
    panels = make_planform(b, te_x, le_x, n_span, n_chord)
    CL1, Cm1, cbar, S = solve(panels, a1)
    CL2, Cm2, cbar, S = solve(panels, a2)
    dCm_dCL = (Cm2 - Cm1) / (CL2 - CL1)
    x_ac = 0.0 - cbar * dCm_dCL
    return dict(x_ac=x_ac, x_ac_mac=x_ac / cbar, cbar=cbar, S=S,
                CL1=CL1, CL2=CL2, slope=(CL2 - CL1) / np.radians(a2 - a1),
                dCm_dCL=dCm_dCL)


# ─────────────────────────────────────────────────────────────
#  自证：在**已知答案**的矩形翼上校准
# ─────────────────────────────────────────────────────────────
def validate():
    print('=' * 62)
    print('自证 1：矩形翼 AR=6（b=6, c=1）')
    print('  期望：升力线斜率接近 Helmbold 解；AC 落在 25% 弦附近')
    r = aero_center(b=6.0, te_x=lambda y: 1.0, le_x=lambda y: 0.0)
    helmbold = 2 * np.pi * 6 / (2 + np.sqrt(36 + 4))
    print('  VLM 斜率 = %.3f /rad   Helmbold = %.3f /rad   偏差 %.1f%%'
          % (r['slope'], helmbold, 100 * (r['slope'] - helmbold) / helmbold))
    print('  cbar = %.4f（解析 1.0）   S = %.3f（解析 6.0）' % (r['cbar'], r['S']))
    print('  AC = %.4f mac   %s' % (r['x_ac_mac'],
          '✅ 接近 0.25' if abs(r['x_ac_mac'] - 0.25) < 0.05 else '❌ 偏离 0.25 太多'))

    print()
    print('自证 2：矩形翼 AR=2（低展弦比，检验 AR 依赖性）')
    r2 = aero_center(b=2.0, te_x=lambda y: 1.0, le_x=lambda y: 0.0)
    h2 = 2 * np.pi * 2 / (2 + np.sqrt(4 + 4))
    print('  VLM 斜率 = %.3f /rad   Helmbold = %.3f /rad' % (r2['slope'], h2))
    print('  AC = %.4f mac   %s' % (r2['x_ac_mac'],
          '✅ 接近 0.25' if abs(r2['x_ac_mac'] - 0.25) < 0.06 else '❌ 偏离太多'))
    print('=' * 62)
    return r


if __name__ == '__main__':
    validate()
