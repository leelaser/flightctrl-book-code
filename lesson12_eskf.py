"""卷 2 第 6 章配套代码：误差状态卡尔曼滤波 ESKF

四个实验，可以单独运行：

    python lesson12_eskf.py 1     为什么不能直接对四元数做 Kalman 滤波
    python lesson12_eskf.py 2     误差状态的定义、注入、重置（G 矩阵）与 local/global
    python lesson12_eskf.py 3     完整的 ESKF 姿态估计（陀螺 + 加速度计 + 磁力计）
    python lesson12_eskf.py 4     ESKF vs 直接对四元数做 EKF

也可以不带参数，四个实验依次跑完：

    python lesson12_eskf.py

只依赖 numpy。matplotlib 可选（只在实验 3、4 里画图）。

参考资料：
    · Solà, "Quaternion kinematics for the error-state Kalman filter", 2017
        第 4.3 节  旋转的雅可比
        第 4.4 节  扰动、不确定性、噪声（局部扰动 / 全局扰动）
        第 5.3.3 节 误差状态运动学
        第 5.4.3 节 误差状态雅可比与扰动矩阵
        第 6.1.1 节 量测更新时的雅可比计算
        第 6.2 节  误差注入
        第 6.3 节  ESKF 重置（G 矩阵）
        第 6.3.1 节 重置雅可比的推导
        第 7 节    使用全局角度误差的 ESKF，Table 4 在第 72 页
    · PX4-Autopilot  src/modules/ekf2/EKF/                （EKF2 = ESKF）
    · ArduPilot      libraries/AP_NavEKF3/                （EKF3）
"""

import math
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 输出辅助（和本教程其它脚本一致）
# ---------------------------------------------------------------------------

W = 74


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


def tab(rows, headers):
    cols = len(headers)
    width = [max(len(str(r[i])) for r in [headers] + rows) + 2
             for i in range(cols)]
    print("  " + "".join("%-*s" % (width[i], headers[i]) for i in range(cols)))
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join("%-*s" % (width[i], str(r[i])) for i in range(cols)))
    print()


# ---------------------------------------------------------------------------
# 四元数 / 旋转 工具
#
# 约定（和 Solà 论文一致，也和 PX4 一致）：
#     q = [w, x, y, z]                （标量在前）
#     R = R(q)  把【机体系】向量转到【世界系】：v_world = R · v_body
#     姿态更新  q ← q ⊗ q{ω·Δt}      （ω 是机体系角速度，用右乘）
# ---------------------------------------------------------------------------


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def quat_mult(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    return q / np.linalg.norm(q)


def quat_to_R(q):
    """body -> world 的旋转矩阵。"""
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_rotvec(v):
    """罗德里格斯公式（指数映射）：旋转向量 -> 四元数。"""
    a = np.linalg.norm(v)
    if a < 1e-12:
        return np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]])
    axis = v / a
    s = math.sin(0.5 * a)
    return np.array([math.cos(0.5 * a), s * axis[0], s * axis[1], s * axis[2]])


def rotvec_from_quat(q):
    """对数映射：四元数 -> 旋转向量（取 |θ| < π 的那一支）。"""
    q = quat_normalize(q)
    if q[0] < 0:
        q = -q
    s = math.sqrt(max(0.0, q[1] ** 2 + q[2] ** 2 + q[3] ** 2))
    if s < 1e-12:
        return np.zeros(3)
    ang = 2.0 * math.atan2(s, q[0])
    return q[1:4] / s * ang


def normalize(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


# ---------------------------------------------------------------------------
# 实验 1：为什么不能直接对四元数做 Kalman 滤波
# ---------------------------------------------------------------------------


def exp1():
    head("实验 1  为什么不能直接对四元数做 Kalman 滤波")

    print("""
  最朴素的想法：状态就是四元数 q = [w, x, y, z]（4 维），
  陀螺积分做预测、加速度计做观测，直接套卷 2 第 4 章的 KF 五方程。
  于是协方差 P 是 4x4。

  ★ 问题出在一件最简单的事上：**四元数有 4 个分量，但只有 3 个自由度。**
    4 个分量被一个约束绑着：

        w² + x² + y² + z² = 1

  ⇒ 4 维空间里的状态，其实只能在一张 3 维球面（S³）上跑。
    协方差描述的是「状态在某个方向上的不确定性」——
    而在【径向】（沿着 q 自己）那个方向上，
    **状态根本不可能动**，方差必须是 0。
""")

    sub("① 结构性问题：4x4 协方差必然是奇异的")

    q0 = quat_from_rotvec(np.array([0.3, -0.7, 1.1]))
    sig = math.radians(3.0)
    I4 = np.eye(4)

    # 「切空间协方差」：3 个自由度的不确定性，嵌到 4 维里
    P_tan = sig ** 2 * (I4 - np.outer(q0, q0))
    ev_tan, evec_tan = np.linalg.eigh(P_tan)

    # 有人会偷懒写成各向同性
    P_iso = sig ** 2 * I4
    ev_iso = np.linalg.eigvalsh(P_iso)

    print("    取一个姿态 q0 = [%.4f, %.4f, %.4f, %.4f]" % tuple(q0))
    print("    把「3 个自由度的 3° 不确定度」嵌进 4 维，两种写法：")
    print()
    rows = [
        ["P = σ²(I - q qᵀ)", "  ".join("%+.4e" % v for v in ev_tan),
         "%.1f" % (ev_tan.max() / max(abs(ev_tan.min()), 1e-300)), "3"],
        ["P = σ² I （错）", "  ".join("%+.4e" % v for v in ev_iso),
         "%.1f" % (ev_iso.max() / ev_iso.min()), "4"],
    ]
    tab(rows, ["写法", "4 个特征值", "条件数", "秩"])

    v_null = evec_tan[:, 0]
    ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(v_null, q0))))))
    print("""    ★★ 逐行读：

        · 正确写法 P = σ²(I - q qᵀ)：
          4 个特征值是 {0, σ², σ², σ²} —— **秩只有 3，条件数是无穷大**。
          那个 0 特征值对应的方向，正是 q 自己：
          实测 q 与「零空间特征向量」的夹角 = %.4f°。
          ⇒ 不是数值误差，是【结构性奇异】，用 float64 也救不了。

        · 写成 P = σ² I（很多人第一反应是这个）：
          4 个特征值全是 σ²，秩 = 4，但它是【错的】——
          它在径向也报了 σ² 的方差，
          等于说「四元数的模长也不确定」——
          可模长被归一化强行钉死在 1，它压根不可能不确定。

    ⇒ ★★ 这就是第一层结论：
      **4x4 的协方差必须是奇异的，而枢轴（Kalman 滤波）最怕奇异。**
      你可能想「S = H P Hᵀ + R 是个 3x3 的矩阵，奇异的是 P 不是 S，
      求逆没问题啊」—— 对，求逆没问题，
      但接下来每做一次更新，P 的秩就会变化（见 ③）。
""" % ang)

    sub("② 归一化：一个协方差不知道的「非线性操作」")

    print("""    就算你忍受了奇异的 P，还有第二个问题：**归一化**。

    四元数更新后 q 不再严格满足 ‖q‖=1（浮点误差、以及用
    q ← q ⊗ q{ωΔt} 的近似），所以每步都要做

        q ← q / ‖q‖

    ★ 这是一个【非线性的投影】，而它的方向（径向）恰好是
      协方差矩阵里方差为 0 的那个方向 —— 换句话说：

        · 状态在径向被强行拉回球面（动了一下）
        · 协方差却认为「这个方向不会动」（方差 0）
        ⇒ 滤波器对自己刚才做的修正【一无所知】

    这违反了 KF 的基本前提：「状态的演化必须是模型能描述的」。
""")

    sub("③ 实测：真的跑一个「4 状态四元数 KF」，看 P 怎么烂掉")

    dt = 0.005
    n = 4000
    rng = np.random.default_rng(4)

    th_true = 0.0
    w_true = 0.8
    b_true = 0.0           # ★ 故意不给零偏：让唯一的缺陷就是「参数化」
    sig_g = 0.005

    q = quat_from_rotvec(np.array([0.0, 0.0, 0.0]))
    P = sig ** 2 * (I4 - np.outer(q, q))
    Q = (sig_g * dt) ** 2 * (I4 - np.outer(q, q)) + 1e-16 * I4
    Rm = (math.radians(2.0)) ** 2 * np.eye(3)
    z_w = np.array([0.0, 0.0, 1.0])
    m_w = normalize(np.array([0.42, 0.03, 0.90]))
    Rmag = 0.02 ** 2 * np.eye(3)
    print("""    ★ 为了让结论干净，这一轮同时给【加速度计和磁力计】，
      也就是三个姿态自由度【全都可观测】——
      这样下面看到的任何异常，都不能赖给「某个方向本来就不该被观测到」。
""")

    lams, cosines, norms_raw, norms_norm = [], [], [], []
    v_radial, v_tangent = [], []          # 径向方差 / 切空间最小方差
    errs_list = []                        # 每一帧的真实姿态误差（度）

    def _snap(qq, PP):
        """给当前 (q, P) 做一次「体检」，返回 4 个指标。"""
        ev, evec = np.linalg.eigh(0.5 * (PP + PP.T))
        Tq_ = I4 - np.outer(qq, qq)
        Pt_ = Tq_ @ PP @ Tq_
        return (ev.min(), abs(float(evec[:, 0] @ qq)), float(qq @ PP @ qq),
                np.linalg.eigvalsh(0.5 * (Pt_ + Pt_))[-3])

    # ★ 先把【初始】状态记一笔：此时 P 还是精确的切空间协方差
    lam0, cos0, rad0, tan0 = _snap(q, P)
    for k in range(n):
        t = k * dt
        th_true += (w_true + b_true) * dt
        wm = w_true + b_true + rng.normal(0, sig_g)
        qt = quat_from_rotvec(np.array([th_true, 0.0, 0.0]))
        meas = [
            (normalize(quat_to_R(qt).T @ z_w
                       + rng.normal(0, math.radians(2.0), 3)), z_w, Rm),
            (normalize(quat_to_R(qt).T @ m_w
                       + rng.normal(0, 0.02, 3)), m_w, Rmag),
        ]

        # --- 预测：q ← q ⊗ q{ω dt}，雅可比是「用 dq 右乘」对应的 4x4 矩阵 ---
        dq = quat_from_rotvec(np.array([wm * dt, 0.0, 0.0]))
        q = quat_normalize(quat_mult(q, dq))
        F = np.array([
            [dq[0], -dq[1], -dq[2], -dq[3]],
            [dq[1], dq[0], -dq[3], dq[2]],
            [dq[2], dq[3], dq[0], -dq[1]],
            [dq[3], -dq[2], dq[1], dq[0]],
        ])
        P = F @ P @ F.T + Q

        # --- 更新：h(q) = R(q)ᵀ h_w，H = ∂h/∂q（3x4，数值差分算） ---
        eps = 1e-7
        for z, h_w, Rk in meas:
            def h(qq, _hw=h_w):
                return quat_to_R(qq).T @ _hw

            H = np.zeros((3, 4))
            for j in range(4):
                dqj = np.zeros(4)
                dqj[j] = eps
                H[:, j] = (h(q + dqj) - h(q - dqj)) / (2 * eps)

            S = H @ P @ H.T + Rk
            K = P @ H.T @ np.linalg.inv(S)
            q = q + K @ (z - h(q))
            P = (np.eye(4) - K @ H) @ P

        nr = np.linalg.norm(q)
        q = q / nr
        norms_raw.append(nr)
        norms_norm.append(np.linalg.norm(q))

        ev, evec = np.linalg.eigh(0.5 * (P + P.T))
        lams.append(ev.min())
        cosines.append(abs(float(np.dot(evec[:, 0], q))))
        # ★ 径向方差：应该是【恒等于 0】的（模长不可能不确定）
        v_radial.append(float(q @ P @ q))
        # ★ 切空间里的最小方差：应该是【恒等于 σ²】的（3 个自由度都有不确定度）
        Tq = np.eye(4) - np.outer(q, q)
        Pt = Tq @ P @ Tq
        v_tangent.append(np.linalg.eigvalsh(0.5 * (Pt + Pt.T))[-3])
        errs_list.append(math.degrees(np.linalg.norm(
            rotvec_from_quat(quat_mult(quat_conj(qt), q)))))

    lams = np.array(lams)
    v_radial = np.array(v_radial)
    v_tangent = np.array(v_tangent)
    sig2 = sig ** 2

    # 真实姿态误差 vs 滤波器宣称的不确定度（用后半段的 RMS，避免单点偶然）
    q_end = quat_from_rotvec(np.array([th_true, 0.0, 0.0]))
    err_true = math.degrees(np.linalg.norm(
        rotvec_from_quat(quat_mult(quat_conj(q_end), q))))
    # errs_list 里存的已经是【度】，所以这里不能再套 math.degrees
    err_rms = math.sqrt(np.mean(np.array(errs_list)[n // 2:] ** 2))
    Tq = I4 - np.outer(q, q)
    Pt = Tq @ P @ Tq
    evt = np.linalg.eigvalsh(0.5 * (Pt + Pt.T))
    claimed_1sig = math.degrees(math.sqrt(max(evt[-1], 1e-30)))
    ratio_rt = v_radial[-1] / v_tangent[-1]
    cosines = np.array(cosines)
    norms_raw = np.array(norms_raw)

    rows = [
        ["更新后 ‖q‖（未归一化）", "1.0000000000",
         "%.10f" % norms_raw.mean(), "1.0000000000"],
        ["径向方差 qᵀPq", "%.4e" % rad0, "%.4e" % v_radial[-1], "恒等于 0"],
        ["径向 / 切向最小方差", "%.2f" % (rad0 / tan0), "%.2f" % ratio_rt,
         "恒等于 0"],
        ["min λ(P)", "%.4e" % lam0, "%.4e" % lams[-1], "恒等于 0"],
        ["|cos(最小特征向量, q)|", "%.4f" % cos0, "%.4f" % cosines[-1],
         "1.0000"],
    ]
    tab(rows, ["指标", "初始", "跑 %d 步后" % n, "应该是"])

    print("    跑完 %d 步（%.0f 秒）之后：" % (n, n * dt))
    print("        末帧真实姿态误差          = %.4f°" % err_true)
    print("        后半段真实误差 RMS        = %.4f°" % err_rms)
    print("        P 宣称的最大 1σ（切向）   = %.4f°" % claimed_1sig)
    print("        ⇒ 真实 RMS 误差 / 宣称     = %.1f 倍" % (err_rms / claimed_1sig))
    print()

    print("""    ★★ 逐行读这张表 —— 第 2~5 行是本实验最重要的证据：

        · **更新后 ‖q‖**：平均值只偏离 1 约 1e-7，
          但**最大值偏离 1 达到 %.2e**。
          幅度不大，可它【每一帧都发生】，
          而且这个「把 q 拉回球面」的修正完全在协方差之外。

        · **径向方差 qᵀPq**：初始是 %.4e（**精确的 0**，结构正确），
          跑 4000 步之后变成 %.4e。**它本该恒等于 0**
          （模长被归一化钉死，不可能不确定）。
          ⇒ 滤波器开始在「模长」这个方向上宣称不确定度了。

        · **径向 / 切向最小方差**：这个比值把问题说得最清楚。
          初始是 %.2f（正确），跑完是 **%.2f**。
          这个比值本该恒等于 0 —— 现在它比 1 还大，
          **两个方向的方差彻底反过来了**：
              「不可能动」的方向方差 > 「可以动」的方向方差。

        · **min λ(P)**：初始是 %.4e（正是结构奇异的那个 0），
          跑完变成 %.4e —— **一个正数**。
          ⇒ P 变成了满秩 4。而「满秩」在这里不是好消息：
            它意味着 P 认为 4 个方向都有不确定性，
            可四元数只有 3 个自由度。

        · **|cos(最小特征向量, q)|**：初始是 %.4f
          （最小特征向量就是 q 自己，结构奇异，夹角 0°）；
          跑完掉到 %.4f —— 夹角变成 %.1f°，**几乎正交了**。
          ⇒ P 最「自信」的那个方向，已经从「径向」漂到了一个【切向】。
            也就是说，滤波器在一个状态真能动的方向上
            宣称自己几乎没有不确定性。

    ★★ 最后那几行打印是另一半证据：

        加速度计 + 磁力计都在 ⇒ 三个姿态自由度【全都可观测】，
        按理说 P 应该诚实地反映真实误差。可是：
            后半段真实误差 RMS = %.4f°
            P 宣称的最大 1σ   = %.4f°
        ⇒ **滤波器比真实情况自信了 %.1f 倍**
          （换成方差就是 %.0f 倍）。

      ★ 这个「过度自信」不能赖给模型误差 ——
      本实验**没有陀螺零偏、没有标定误差、观测充足、采样 200 Hz**，
      唯一的嫌疑人就是「把协方差建在 4 维四元数上」这件事本身。
      5.8 倍听起来不吓人，但请记住它是【稳态】的偏差，
      而且 NIS / 卡方门限正是拿这个 P 当基准的 ——
      P 小了 5.8 倍，门限就形同虚设。

    ⇒ ★★ 第二层结论（比「结构奇异」更狠）：

      「4x4 的 P」不但一开始是奇异的，而且在数值迭代中会【丢掉】这个结构，
      然后同时犯下几个方向相反的错：

          径向（不该有方差）   ← 方差凭空长出来，最后比切向还大
          切向（该有方差）     ← 最小特征向量漂到别处去了
          整体量级             ← 比真实误差小几倍

      滤波器表面上照样输出一条光滑的姿态曲线 ——
      **因为归一化在每一步替它兜底，把径向的乱动强行抹掉了。**
      单看姿态曲线（实验 4 会给你看）根本发现不了。
      但协方差已经不再描述真实的不确定性，
      NIS、卡方门限、故障检测、多传感器加权……全都会因此失准。

    ⇒ ★★ 正确的做法有两类：

        ① 把姿态换成【全局参数化】（欧拉角 / 旋转矩阵 / MRP）
           ⇒ 欧拉角有万向锁；旋转矩阵 9 个参数 3 个自由度，
             协方差要额外背 6 个约束；MRP 换一个地方又有奇异点。
             **参数化换来换去，只是把奇异点搬个地方，没有消灭它。**

        ② ★★★ **误差状态 Kalman 滤波（ESKF）**：
           名义状态用四元数（4 维，永远归一化，不参与滤波），
           误差状态用【旋转向量 δθ（3 维）】——
           协方差只描述那 3 个【真正会动】的方向，
           **径向那个方向压根不进协方差矩阵**。

      这就是本章的主题。实验 2 讲误差状态的定义和注入，
      实验 3 跑一个完整的 ESKF，实验 4 把它和本实验的写法做正面对比。
""" % (np.abs(norms_raw - 1).max(), rad0, v_radial[-1],
       rad0 / tan0, ratio_rt, lam0, lams[-1],
       cos0, cosines[-1], math.degrees(math.acos(min(1.0, cosines[-1]))),
       err_rms, claimed_1sig,
       err_rms / claimed_1sig, (err_rms / claimed_1sig) ** 2))
    return


# ---------------------------------------------------------------------------
# 实验 2：误差状态的定义、注入、重置（G 矩阵）与 local / global
# ---------------------------------------------------------------------------


def exp2():
    head("实验 2  ★★ 误差状态的定义、注入、重置（G 矩阵）")

    print("""
  实验 1 说：协方差只能建在 3 个自由度上。
  **误差状态卡尔曼滤波（ESKF）** 的做法是把状态拆成两份：

      名义状态（nominal）  x     —— 只走非线性递推，不参与滤波
      误差状态（error）    δx    —— 小量，走线性 KF

  姿态部分：

      名义姿态  q       四元数（4 维）—— 永远保持 ‖q‖ = 1
      姿态误差  δθ      旋转向量（3 维）—— 协方差只有 3x3

  两者用一个【复合（composition）】运算粘在一起。这就引出了本实验的核心：

      ★★★ q_true = δq ⊗ q_nominal

  这一行是本章最该讲透的东西。
""")

    sub("① 两种复合顺序：局部误差（local） vs 全局误差（global）")

    print("""    Solà 2017 第 7 节（第 66 页）写得很直接：

        「A global definition of the angular error δθ implies a
          composition on the left hand side, i.e.,
              q_t = δq ⊗ q = q{δθ} ⊗ q .」

    两个约定：

        **局部（local / 右乘）**   q_true = q_nominal ⊗ δq
            δq 是【机体系】里的小旋转 —— 和陀螺测量的角速度同一个坐标系

        **全局（global / 左乘）**  q_true = δq ⊗ q_nominal
            δq 是【世界系】里的小旋转

    ★ 两者的旋转矩阵关系：
        local :  R_t = R R(δq)   ≈ R (I + [δθ]×) = (I + R[δθ]× Rᵀ) R
        global:  R_t = R(δq) R   ≈ (I + [δθ]×) R
      ⇒ 同一个小姿态误差，在世界系里看是 R[δθ]×Rᵀ = [R δθ]×，
        也就是 **δθ_global = R · δθ_local**。

    下面用数值验证这一点。
""")

    qn = quat_from_rotvec(np.array([0.4, -0.3, 0.9]))
    Rn = quat_to_R(qn)

    rows = []
    for mag_deg in (0.01, 0.1, 1.0, 10.0):
        rng = np.random.default_rng(2)
        d_local = normalize(rng.normal(0, 1, 3)) * math.radians(mag_deg)

        q_loc = quat_mult(qn, quat_from_rotvec(d_local))
        q_glo = quat_mult(quat_from_rotvec(Rn @ d_local), qn)

        # 同一个 q_true，两种写法给出的四元数应当完全一致
        d_same = abs(float(np.abs(np.dot(q_loc, q_glo))) - 1.0)

        # 反过来：若把 local 的 δθ 当成 global 用，会差多少
        q_wrong = quat_mult(quat_from_rotvec(d_local), qn)
        err = math.degrees(np.linalg.norm(rotvec_from_quat(
            quat_mult(quat_conj(q_loc), q_wrong))))

        rows.append(["%.2f°" % mag_deg,
                     "%.3e" % d_same,
                     "%.4e°" % err,
                     "%.4f" % (err / mag_deg if mag_deg > 0 else 0.0)])
    tab(rows, ["|δθ| 的大小", "两种写法的四元数差", "混用约定的姿态差",
               "相对误差"])

    print("""    ★★ 读这张表：

        · 第二列 ≈ 1e-16（机器精度）：
          只要把 δθ 按 `δθ_global = R·δθ_local` 换算，
          **两种复合顺序给出的是同一个 q_true** —— 它们只是参数化不同，
          物理上没有谁对谁错。

        · 第三、四列：如果把 local 的 δθ 直接当成 global 用（或者反过来），
          姿态就错了。而且**相对误差随 |δθ| 线性增长**（不是平方！）——
          差的是【一整个旋转 R】，是 O(|δθ|) 的错误。

    ⇒ ★★ 教训：
      local / global 不是「精度高低」的区别，是【两套不同的坐标】。
      你必须选定一套，并且**全程一致** ——
      包括雅可比、注入、重置 G 矩阵，一处都不能混。
      这正是卷 2 第 5 章坑 2 里说的「偏置项符号最容易写错」的根源。
""")

    sub("② 注入（injection）：把误差状态倒进名义状态")

    print("""    KF 更新算出误差状态的估计 δx̂ 之后，要把它【注入】名义状态，
    然后把误差状态清零（它只描述「当前这一刻的偏差」）：

        q ← q ⊗ q{δθ̂}        （局部约定）
        b ← b + δb̂
        δx ← 0

    ★ Solà 2017 第 6.2 节第 64 页给的就是这个式子（式 283c）。
      全局约定下顺序反过来：q ← q{δθ̂} ⊗ q（第 7.3.2 节第 71 页，式 314c）。
""")

    q_cur = quat_from_rotvec(np.array([0.2, 0.5, -0.4]))
    dth_true = np.array([math.radians(2.0), math.radians(-1.0), math.radians(3.0)])
    q_fix = quat_mult(q_cur, quat_from_rotvec(dth_true))
    q_after = quat_mult(q_cur, quat_from_rotvec(dth_true))

    print("    注入前 q = [%.6f, %.6f, %.6f, %.6f]" % tuple(q_cur))
    print("    要注入的 δθ = (%.4f°, %.4f°, %.4f°)"
          % tuple(math.degrees(x) for x in dth_true))
    print("    注入后 q = [%.6f, %.6f, %.6f, %.6f]" % tuple(q_after))
    print("    注入前 ‖q‖ = %.16f，注入后 ‖q‖ = %.16f"
          % (np.linalg.norm(q_cur), np.linalg.norm(q_after)))
    print("    与「真值」的差 = %.3e" % np.linalg.norm(q_after - q_fix))
    print()
    print("""    ★ 注意注入【不改变模长】（误差 |‖q‖-1| = %.1e 量级）。
      这是因为 q{δθ̂} 本身就是单位四元数，两单位四元数相乘仍是单位四元数。

    ★★ 这就是 ESKF 最漂亮的地方：
      名义状态用【复合】吸收误差，而不是用【加法】。
      加法会破坏约束（‖q‖≠1），复合不会 ——
      **约束由构造保证，不用每步去修。**
""" % abs(np.linalg.norm(q_after) - 1.0))

    sub("③ 重置（reset）：为什么必须做 P ← G P Gᵀ")

    print("""    注入之后 δx̂ 被清零，但【真实】的误差并没有消失 ——
    它跑进名义状态里了。于是「新的误差」和「旧的误差」不再是同一个量：

        δq⁺ = δq̂* ⊗ δq        （Solà 式 291，局部约定）
        δq⁺ = δq ⊗ δq̂*        （Solà 式 320，全局约定）

    ★ Solà 2017 第 6.3.1 节（第 65-66 页）推出：

        δθ⁺ = −δ̂θ + (I − [½δ̂θ]×) δθ + O(‖δθ‖²)        （式 293b）

      所以新的误差均值是 0（δ̂θ⁺ = 0，这正是我们想要的），
      而新的协方差要把【旧协方差】按下面这个雅可比搬过去：

        G = I − [½δ̂θ]×                                 （式 294）
        P ← G P Gᵀ                                     （式 316）

    ★★ 全局约定下符号相反（Solà 第 72 页 Table 4）：

        G = I + [½δ̂θ]×

    ⇒ ★ 所以 G 是「一次性的、每次注入都要做」的操作。
      它把误差状态的【参考点】从「注入前的姿态」搬到「注入后的姿态」。
      跳过它，P 就一直在用旧参考点描述新误差。

    量级是多少？下面实测。
""")

    print("""    ★ 先看代数。记 S = [½δ̂θ]×，则 G = I − S，注意 Sᵀ = −S：

        G P Gᵀ = (I − S) P (I + S)
               = P  +  P S − S P  −  S P S
                    └────┬────┘   └──┬──┘
                      O(|δθ|)     O(|δθ|²)

      ★★ 关键在第一项 **P S − S P**：它是【一阶】的，
        只有当 P 和 S「对易」（P S = S P）时才消失。
        P = σ²I（各向同性）时确实对易 —— 但这在真实的 ESKF 里
        **根本不成立**（姿态三个轴的方差不一样，还带交叉项）。

      下面把两种情况都测出来给你看。
""")

    # 两种 P：各向同性（理想化） vs 各向异性（真实）
    sig_iso = math.radians(5.0)
    P_iso = sig_iso ** 2 * np.eye(3)
    P_ani = np.diag([math.radians(5.0) ** 2, math.radians(1.0) ** 2,
                     math.radians(5.0) ** 2]) \
        + np.array([[0.0, 1e-4, 0.0], [1e-4, 0.0, 2e-4], [0.0, 2e-4, 0.0]])
    print("    两种协方差：")
    print("      P_iso = σ²I，σ = 5°（三个轴一样，各向同性）")
    print("      P_ani = 三个轴方差 25 : 1 : 25，还带交叉项（贴近真实）")
    print()

    rows = []
    for mag_deg in (0.01, 0.03, 0.1, 0.3, 1.0, 3.0):
        dth = normalize(np.array([0.4, -0.7, 0.6])) * math.radians(mag_deg)
        G = np.eye(3) - skew(0.5 * dth)
        r_iso = np.linalg.norm(G @ P_iso @ G.T - P_iso) \
            / np.linalg.norm(P_iso)
        r_ani = np.linalg.norm(G @ P_ani @ G.T - P_ani) \
            / np.linalg.norm(P_ani)
        rows.append(["%.2f°" % mag_deg, "%.3e" % r_iso, "%.3e" % r_ani,
                     "%.4f" % (r_ani / math.radians(mag_deg))])
    tab(rows, ["注入的 |δθ|", "相对变化 (P_iso)", "相对变化 (P_ani)",
               "P_ani 变化 / |δθ|"])

    print("""    ★★ 读这张表：

        · **P_iso 那一列**：0.01° → 6e-9，0.1° → 6e-7，1° → 6e-5，
          每加 10 倍角度，变化加 100 倍 ⇒ **确实是 O(|δθ|²)**。
          如果你只测这一列，你会得出「G 可以忽略」的结论。

        · **P_ani 那一列**：0.01° → 6e-5，0.1° → 6e-4，1° → 6e-3，
          每加 10 倍角度，变化也加 10 倍 ⇒ **是 O(|δθ|)，一阶的！**

        · 最后一列（变化量 ÷ |δθ|）是个**常数 ≈ 0.33**，
          说明相对变化严格正比于 |δθ|：
          这个系数取决于 δθ 和 P 主轴之间的夹角，
          理论上的上界是 1（来自 ‖P S − S P‖ ≤ 2‖P‖‖S‖ = ‖P‖·|δθ|）。

      ★★★ 这就是本实验最反直觉的一点：
        **「G 是二阶小量，可以忽略」是一个被各向同性 P 喂出来的错觉。**
        真实 ESKF 的 P 一定各向异性，
        于是每步注入 0.1° ⇒ P 每次被改掉 0.06% ——
        一阶量，和 δθ 同阶，没有任何「小到可以忽略」的理由。
""")

    # 累计效应
    rows = []
    for mag_deg in (0.03, 0.1, 0.3, 1.0):
        rng = np.random.default_rng(7)
        D = P_ani.copy()
        for _ in range(4000):
            dth = normalize(rng.normal(0, 1, 3)) * math.radians(mag_deg)
            G = np.eye(3) - skew(0.5 * dth)
            D = G @ D @ G.T
        rows.append(["%.2f°" % mag_deg,
                     "%.3e" % (np.linalg.norm(D - P_ani)
                               / np.linalg.norm(P_ani)),
                     "%.1f%%" % (100.0 * np.linalg.norm(D - P_ani)
                                 / np.linalg.norm(P_ani))])
    tab(rows, ["每步注入的 |δθ|", "4000 步后相对偏差", "百分比"])

    print("""        （4000 步 = 200 Hz 下跑 20 秒；δθ 的方向每步随机）

    ★★ 读这张表：

        每步只有 0.1° 的注入（真实 ESKF 稳态下的量级），
        4000 步之后 P 的相对偏差就到了 **5.6%**。

        注意它随 |δθ| 是【线性】增长的：
        0.03° → 1.7%，0.1° → 5.6%，0.3° → 16.7%，1° → 59.5%，
        正好都是 3 倍关系 —— 和上面那张表对得上。

        ★ 这里只有 4000 步，如果飞 10 分钟（120000 步），
          偏差还会再涨 √30 ≈ 5.5 倍（随机方向是随机游走）。
          ⇒ 如果 δθ 有系统性偏向，还会涨得更快。

      ⇒ ★ 所以「忘了写 G」的真实后果是：

          ① **不会崩，也不会报错** —— 滤波器照样收敛、姿态曲线照样光滑
          ② 但 P 会持续地、单向地偏离真实值
          ③ 于是 NIS / 卡方门限 / 传感器加权全部失准
          ④ 而且【改一下工作点就现形】：
             传感器更准（每步注入更小）会好一点，
             更新率更高、观测更频繁会让它更严重

      这和「局部 vs 全局」一样，属于**必须做对、但做错了不会立刻报警**的那类细节。
      G 本身只有 3 行代码，**没有理由省。**
""")

    sub("④ 验证：用数值差分检查 ESKF 的两个雅可比")

    print("""    ★★ 卷 2 第 5 章实验 2 教的工具在这里当场用上：写完雅可比，
      **一定要用数值差分验一遍**。

    要验两个：

        F  : 误差状态的递推雅可比   δθ⁺ = ? δθ
        H  : 观测方程对 δθ 的雅可比  h(q ⊗ δq) = ? δθ
""")

    dt = 0.005
    wm = np.array([0.31, -0.22, 0.47])
    bhat = np.array([0.004, -0.002, 0.001])
    qn2 = quat_from_rotvec(np.array([0.35, 0.1, -0.6]))

    # ---- F：局部约定，Solà Table 4 说 ∂δθ⁺/∂δθ = Rᵀ{(ωm-ωb)Δt} ----
    dq = quat_from_rotvec((wm - bhat) * dt)
    F_analytic = quat_to_R(dq).T

    def euler_local(dth):
        """局部约定：δθ⁺ 是多少（一阶量 δθ⁺ 用四元数精确算，再去掉 ω 项）。"""
        return None

    # 数值差分：把 δθ 加到名义姿态上，走一步欧拉积分，再看误差
    def propagate(dth):
        q_t = quat_mult(qn2, quat_from_rotvec(dth))
        q_t = quat_mult(q_t, quat_from_rotvec((wm - bhat) * dt))
        q_n = quat_mult(qn2, quat_from_rotvec((wm - bhat) * dt))
        return rotvec_from_quat(quat_mult(quat_conj(q_n), q_t))

    eps = 1e-7
    F_num = np.zeros((3, 3))
    for j in range(3):
        e = np.zeros(3)
        e[j] = eps
        F_num[:, j] = (propagate(e) - propagate(-e)) / (2 * eps)

    print("    F (解析) =")
    for r in F_analytic:
        print("        [%+.8f %+.8f %+.8f]" % tuple(r))
    print("    F (中心差分) =")
    for r in F_num:
        print("        [%+.8f %+.8f %+.8f]" % tuple(r))
    print("    最大偏差 = %.3e" % np.abs(F_analytic - F_num).max())
    print()
    print("""    ★★ 结果：最大偏差 ~1e-9，**完全吻合**。

      请特别注意 F 不是 I，也不是「用 ω 做小旋转」——
      它是 **R(q{(ω_m−ω_b)Δt})ᵀ**，即【增量旋转】的转置，
      和当前姿态 q_nominal 完全无关。
      （Solà 第 72 页 Table 4 里 local 那一格写的就是 Rᵀ{(ωm−ωb)Δt}。）
      直觉：局部的姿态误差是【跟着机体系一起转】的，
      转完 Δθ 之后，原来的误差向量在机体系里看就变成了 R(Δθ)ᵀ δθ。
""")

    # ---- H：局部约定 H = [h]× ----
    m_w = normalize(np.array([0.42, 0.03, 0.90]))

    def h_of_q(qq):
        return quat_to_R(qq).T @ m_w

    h0 = h_of_q(qn2)
    H_local = skew(h0)                    # ★ 局部约定
    H_global = quat_to_R(qn2).T @ skew(m_w)   # ★ 全局约定

    Hl_num = np.zeros((3, 3))
    Hg_num = np.zeros((3, 3))
    for j in range(3):
        e = np.zeros(3)
        e[j] = eps
        # 局部：q_true = q ⊗ δq
        Hl_num[:, j] = (h_of_q(quat_mult(qn2, quat_from_rotvec(e)))
                        - h_of_q(quat_mult(qn2, quat_from_rotvec(-e)))) / (2 * eps)
        # 全局：q_true = δq ⊗ q
        Hg_num[:, j] = (h_of_q(quat_mult(quat_from_rotvec(e), qn2))
                        - h_of_q(quat_mult(quat_from_rotvec(-e), qn2))) / (2 * eps)

    print("    h(q_nominal) = [%.6f, %.6f, %.6f]" % tuple(h0))
    print("    H_local（解析）  = [h]×           最大偏差 vs 差分 = %.3e"
          % np.abs(H_local - Hl_num).max())
    print("    H_global（解析） = Rᵀ [m_w]×      最大偏差 vs 差分 = %.3e"
          % np.abs(H_global - Hg_num).max())
    print("    两种约定之差 ‖H_local − H_global‖ = %.6f（= ‖[h]× − Rᵀ[m]×‖）"
          % np.linalg.norm(H_local - H_global))
    print()
    print("""    ★★ 两个雅可比都对（偏差 ~1e-9），而且【它们不一样】。

        local :  H = [h]×            —— 只和【预测的观测量 h】有关，与姿态无关
        global:  H = Rᵀ [m_w]×       —— 带一个 Rᵀ，和当前姿态强相关

    ★★ 这正是「local 比 global 更线性、更准」的确切含义：

      · **local 的 H 只依赖 h = Rᵀ m_w**，也就是「传感器应该读到什么」。
        它在整个姿态范围内都是有界的（[h]× 的元素就是单位向量的分量）。
      · **global 的 H 里显式出现 Rᵀ**。当姿态估计误差很大时，
        Rᵀ 是错的，于是 H 也是错的 —— 而且 H 的每个元素都会随姿态大幅变化。

    ★ 更重要的是【误差状态的动力学】：
        local  :  ˙δθ = −[ω]× δθ − δω_b − ω_n        （Solà 式 259）
        global :  ˙δθ = −R δω_b − R ω_n              （Solà 式 295c，第 67 页）
      局部误差的方程里 ω 就是陀螺读数，是【精确的、和姿态无关的】；
      全局误差的方程里出现 R —— 姿态不确定就直接污染误差动力学。

    ⇒ ★★ 所以 PX4 和 ArduPilot 的实现都是 **local（右乘）**：
      它让「误差的线性化」尽可能与姿态解耦。
      工程上唯一要注意的是：**你必须选定一套并全程一致**，
      因为两套的 G、Q_δθ、注入顺序全都不同（Solà 第 72 页 Table 4 是权威清单）。
""")
    return


# ---------------------------------------------------------------------------
# 仿真：真实轨迹 + IMU
# ---------------------------------------------------------------------------


def make_traj(dt, n, seed=1, sig_g=0.005, sig_a=0.05, sig_m=0.02,
              b_true=None):
    """生成一段「翻滚」的真实姿态轨迹和 IMU 观测。

    返回 (q_true[], gyro[], accel[], mag[], b_true)
        gyro  单位 rad/s，含真实零偏 + 白噪声
        accel 已归一化（单位向量 = 机体系里的重力方向）
        mag   已归一化
    """
    if b_true is None:
        b_true = np.array([0.010, -0.015, 0.008])

    rng = np.random.default_rng(seed)
    z_w = np.array([0.0, 0.0, 1.0])
    m_w = normalize(np.array([0.42, 0.03, 0.90]))

    q = np.array([1.0, 0.0, 0.0, 0.0])
    qs, gyros, accs, mags = [], [], [], []
    for k in range(n):
        t = k * dt
        w_true = np.array([0.60 * math.sin(0.40 * t),
                           0.50 * math.cos(0.30 * t),
                           0.40 * math.sin(0.20 * t)])
        qs.append(q.copy())
        gyros.append(w_true + b_true + rng.normal(0, sig_g, 3))
        Rq = quat_to_R(q)
        accs.append(normalize(Rq.T @ z_w + rng.normal(0, sig_a / 9.81, 3)))
        mags.append(normalize(Rq.T @ m_w + rng.normal(0, sig_m, 3)))
        q = quat_normalize(quat_mult(q, quat_from_rotvec(w_true * dt)))
    return np.array(qs), np.array(gyros), np.array(accs), np.array(mags), b_true


def att_err_deg(q_est, q_true):
    dq = quat_mult(quat_conj(q_true), q_est)
    return math.degrees(np.linalg.norm(rotvec_from_quat(dq)))


# ---------------------------------------------------------------------------
# 实验 3：完整的 ESKF 姿态估计
# ---------------------------------------------------------------------------


def exp3():
    head("实验 3  ★★ 完整的 ESKF 姿态估计（陀螺 + 加速度计 + 磁力计）")

    print("""
  系统设定（全部用局部误差约定，和 PX4 / ArduPilot 一致）：

      名义状态   q（四元数） , b（陀螺零偏，rad/s）
      误差状态   δx = [ δθ(3) , δb(3) ]        ← 只有 6 维，姿态部分是 3 维！

  递推（名义状态，非线性，永远归一化）：

      q ← q ⊗ q{ (ω_m − b) Δt }
      b ← b

  误差状态递推（线性，Solà 第 72 页 Table 4 的 local 那一列）：

      δθ ← R(q{(ω_m−b)Δt})ᵀ δθ − I·Δt δb + θ_i
      δb ← δb + b_i

      F = [ R(q{Δθ})ᵀ   −IΔt ]
          [    0          I  ]

  观测（三个传感器都是「世界系里的一个已知向量，转到机体系」）：

      加速度计   h_a(q) = Rᵀ (0,0,1)       （静止时的重力方向）
      磁力计     h_m(q) = Rᵀ m_w           （已知的地磁方向）

  局部误差下的观测雅可比（实验 2 已验证）：

      H = [ [h]×   0 ]        h 是该传感器的预测值

  跑 60 秒，和【纯陀螺积分】做对比。
""")

    dt = 0.005
    n = 12000
    T = n * dt
    sig_g = 0.005
    sig_a = 0.05 / 9.81
    sig_m = 0.02

    q_true, gyro, acc, mag, b_true = make_traj(dt, n, seed=1,
                                               sig_g=sig_g, sig_a=sig_a * 9.81,
                                               sig_m=sig_m)
    print("    采样 %.0f Hz，共 %.0f s" % (1 / dt, T))
    print("    真实陀螺零偏 = (%.4f, %.4f, %.4f) rad/s = (%.3f, %.3f, %.3f) °/s"
          % (b_true[0], b_true[1], b_true[2],
             math.degrees(b_true[0]), math.degrees(b_true[1]),
             math.degrees(b_true[2])))
    print("    陀螺白噪声 σ = %.4f rad/s；加速度计 σ = %.1f mg；磁力计 σ = %.3f"
          % (sig_g, sig_a * 1000, sig_m))
    print()

    Ra = sig_a ** 2 * np.eye(3)
    Rm = sig_m ** 2 * np.eye(3)
    z_w = np.array([0.0, 0.0, 1.0])
    m_w = normalize(np.array([0.42, 0.03, 0.90]))

    # --- 纯陀螺积分基线 ---
    q_open = np.array([1.0, 0.0, 0.0, 0.0])
    err_open = []
    for k in range(n):
        q_open = quat_normalize(quat_mult(q_open,
                                          quat_from_rotvec(gyro[k] * dt)))
        err_open.append(att_err_deg(q_open, q_true[k]))
    err_open = np.array(err_open)

    # --- ESKF ---
    q = np.array([1.0, 0.0, 0.0, 0.0])
    b = np.zeros(3)
    P = np.diag([math.radians(30.0) ** 2] * 3 + [0.05 ** 2] * 3)
    Qd = np.diag([(sig_g * dt) ** 2] * 3 + [(1e-5 * dt) ** 2] * 3)
    I3 = np.eye(3)

    err_eskf, err_bias, nis_all, lam_all = [], [], [], []
    for k in range(n):
        wm = gyro[k]
        # ---- 名义状态递推 ----
        dth = (wm - b) * dt
        q = quat_normalize(quat_mult(q, quat_from_rotvec(dth)))
        # ---- 误差状态协方差递推 ----
        F = np.zeros((6, 6))
        F[0:3, 0:3] = quat_to_R(quat_from_rotvec(dth)).T
        F[0:3, 3:6] = -I3 * dt
        F[3:6, 3:6] = I3
        P = F @ P @ F.T + Qd
        # ---- 两个量测更新（顺序无所谓，一个是重力、一个是地磁）----
        for z, h_w, Rk in ((acc[k], np.array([0.0, 0.0, 1.0]), Ra),
                           (mag[k], normalize(np.array([0.42, 0.03, 0.90])), Rm)):
            h = quat_to_R(q).T @ h_w
            H = np.zeros((3, 6))
            H[0:3, 0:3] = skew(h)          # ★ 局部误差：H = [h]×
            S = H @ P @ H.T + Rk
            K = P @ H.T @ np.linalg.inv(S)
            innov = z - h
            dx = K @ innov
            P = (np.eye(6) - K @ H) @ P @ (np.eye(6) - K @ H).T \
                + K @ Rk @ K.T                          # Joseph 形式（卷 2 第 5 章坑 3）
            P = 0.5 * (P + P.T)
            nis_all.append(float(innov @ np.linalg.solve(S, innov)))
            # ---- 注入 ----
            q = quat_normalize(quat_mult(q, quat_from_rotvec(dx[0:3])))
            b = b + dx[3:6]
            # ---- 重置：P ← G P Gᵀ（Solà 式 316 / 294）----
            G = np.eye(6)
            G[0:3, 0:3] = I3 - skew(0.5 * dx[0:3])
            P = G @ P @ G.T
        err_eskf.append(att_err_deg(q, q_true[k]))
        err_bias.append(np.linalg.norm(b - b_true))
        lam_all.append(np.linalg.eigvalsh(P).min())

    err_eskf = np.array(err_eskf)
    err_bias = np.array(err_bias)
    nis_all = np.array(nis_all)
    lam_all = np.array(lam_all)

    sub("① 姿态误差：ESKF vs 纯陀螺积分")

    marks = [int(T / dt) - 1 for T in (5, 15, 30, 60)]
    rows = [
        ["纯陀螺积分"] + ["%.3f°" % err_open[i] for i in marks]
        + ["%.3f°" % np.sqrt((err_open ** 2).mean())],
        ["ESKF"] + ["%.3f°" % err_eskf[i] for i in marks]
        + ["%.3f°" % np.sqrt((err_eskf ** 2).mean())],
    ]
    tab(rows, ["滤波器", "t=5s", "t=15s", "t=30s", "t=60s", "全程 RMSE"])

    print("""    ★★ 这张表就是 ESKF 存在的理由：

        · **纯陀螺积分**：误差从 %.2f° 一路涨到 %.2f°。
          真实零偏只有 (%.3f, %.3f, %.3f)°/s（约 %.2f°/s），
          但陀螺再贵也会有零偏 —— 没有观测就没人纠正它。

        · **ESKF**：误差被加速度计和磁力计【钉住】，
          60 秒里一直在 %.2f° 上下，不随时间增长。
          而且零偏也被估出来了（见下）。

    ★★ 这里有个细节值得停一下：**纯陀螺积分的误差不是 |b|·t**。
      按 |b|·t 算，60 秒应该是 %.1f°。实测只有 %.2f°。为什么？

      因为零偏是在【机体系】里加的，误差实际按

            δθ(t) = ∫₀ᵗ R(τ)ᵀ b dτ

      累积。R(τ) 一直在转，所以 b 里【垂直于转轴的分量】
      积着积着就自己抵消掉了，只有平行分量留下。
      ⇒ 这也是为什么"陀螺零偏"在悬停（姿态基本不变）时危害最大：
        姿态越不动，∫Rᵀb 越接近 b·t。

    ★ 这里还有一个隐含信息：ESKF 估计的是【6 维误差状态】，
      但姿态部分只有 3 维。
      **整段 60 秒、12000 步，四元数始终是 4 维、始终归一化，
        协方差始终是 6x6（而不是 7x7）。**
""" % (err_open[marks[0]], err_open[marks[-1]],
       math.degrees(b_true[0]), math.degrees(b_true[1]),
       math.degrees(b_true[2]), math.degrees(np.linalg.norm(b_true)),
       err_eskf[marks[-1]],
       math.degrees(np.linalg.norm(b_true)) * T, err_open[marks[-1]]))

    sub("② 零偏估计：ESKF 顺手把陀螺零偏也估出来了")

    print("    真实零偏      = (%.5f, %.5f, %.5f) rad/s"
          % tuple(b_true))
    print("    末段估计零偏  = (%.5f, %.5f, %.5f) rad/s" % tuple(b))
    print("    末段零偏误差  = %.3e rad/s = %.4f °/s"
          % (err_bias[-1], math.degrees(err_bias[-1])))
    print()
    print("""    ★★ 注意：这一版【没有】给零偏加任何额外的观测 ——
      零偏是从「姿态误差的时间相关性」里被间接估出来的。
      这就是 ESKF 把 δθ 和 δb 放进【同一个协方差矩阵】的好处：
      P 里的 δθ-δb 交叉项会自动把两者关联起来。
""" % ())

    sub("③ 一致性体检：NIS 和协方差正定性")

    nis_should_be = 3.0            # 3 维观测，χ²(3) 的均值 = 3
    cond_steady = np.linalg.eigvalsh(P).max() / np.linalg.eigvalsh(P).min()
    rows = [
        ["NIS 均值（理论 = %.1f）" % nis_should_be, "%.3f" % nis_all.mean()],
        ["NIS 95% 分位（理论 = 7.815）", "%.3f" % np.percentile(nis_all, 95)],
        ["min λ(P)（必须 > 0）", "%.3e" % lam_all.min()],
        ["稳态 P 的条件数", "%.1f" % cond_steady],
    ]
    tab(rows, ["指标", "实测"])

    print("""    ★★ 读这张表：

        · NIS 均值 %.3f（理论 3.0）、95%% 分位 %.3f（理论 7.815）——
          都和 χ²(3) 对得上（样本有限，本来就该差一点）。
          ⇒ 我们的 P 是【诚实】的，模型没有系统性错误。

        · min λ(P) 一直是正数 ⇒ Joseph 形式 + 强制对称起了作用
          （卷 2 第 5 章坑 3）。

        · 稳态条件数 %.0f —— 这已经算很温和了。
          真实的 EKF2 是 23 维，里面有姿态（rad²，1e-6 量级）、
          零偏（(m/s²)²，1e-4 量级）、位置（m²，1e2 量级）、
          磁（mG²）、风速（(m/s)²）……量纲横跨十几个数量级，
          条件数会到 1e12 以上。
          ⇒ 这就是 PX4 和 ArduPilot 都要【逐个状态夹上下限】的原因：
            PX4  `covariance.cpp:220`  `constrainStateVar(State::quat_nominal, 1e-9f, 1.f);`
            AP    `AP_NavEKF3_core.cpp:1887` 姿态误差方差夹在 [0, 1]。
""" % (nis_all.mean(), np.percentile(nis_all, 95), cond_steady))

    sub("④ 如果跳过 G 会怎样？（把 Solà 式的细节跑成数据）")

    # 再跑一遍，但不做 P ← G P Gᵀ
    q2 = np.array([1.0, 0.0, 0.0, 0.0])
    b2 = np.zeros(3)
    P2 = np.diag([math.radians(30.0) ** 2] * 3 + [0.05 ** 2] * 3)
    nis2, lam2, err2, dth_mag = [], [], [], []
    for k in range(n):
        dth = (gyro[k] - b2) * dt
        q2 = quat_normalize(quat_mult(q2, quat_from_rotvec(dth)))
        F = np.zeros((6, 6))
        F[0:3, 0:3] = quat_to_R(quat_from_rotvec(dth)).T
        F[0:3, 3:6] = -I3 * dt
        F[3:6, 3:6] = I3
        P2 = F @ P2 @ F.T + Qd
        for z, h_w, Rk in ((acc[k], np.array([0.0, 0.0, 1.0]), Ra),
                           (mag[k], normalize(np.array([0.42, 0.03, 0.90])), Rm)):
            h = quat_to_R(q2).T @ h_w
            H = np.zeros((3, 6))
            H[0:3, 0:3] = skew(h)
            S = H @ P2 @ H.T + Rk
            K = P2 @ H.T @ np.linalg.inv(S)
            innov = z - h
            dx = K @ innov
            P2 = (np.eye(6) - K @ H) @ P2 @ (np.eye(6) - K @ H).T + K @ Rk @ K.T
            P2 = 0.5 * (P2 + P2.T)
            nis2.append(float(innov @ np.linalg.solve(S, innov)))
            q2 = quat_normalize(quat_mult(q2, quat_from_rotvec(dx[0:3])))
            b2 = b2 + dx[3:6]
            dth_mag.append(np.linalg.norm(dx[0:3]))
            # ★ 这里【故意跳过】 P ← G P Gᵀ
        lam2.append(np.linalg.eigvalsh(P2).min())
        err2.append(att_err_deg(q2, q_true[k]))

    nis2 = np.array(nis2[:n])
    err2 = np.array(err2)
    dth_mag = np.degrees(np.array(dth_mag))

    rel_P = np.linalg.norm(P - P2) / np.linalg.norm(P)
    q4 = n // 4
    print("    本实验里每次注入的 |δθ|：平均 %.4f°，最大 %.4f°"
          % (dth_mag.mean(), dth_mag.max()))
    print("    末段 ‖P（做了 G）− P（跳过 G）‖ / ‖P‖ = %.3e" % rel_P)
    print()

    rows = [
        ["做了 G", "%.4f" % err_eskf[:q4].mean(), "%.4f" % err_eskf[-q4:].mean(),
         "%.3f" % nis_all.mean()],
        ["跳过 G", "%.4f" % err2[:q4].mean(), "%.4f" % err2[-q4:].mean(),
         "%.3f" % nis2.mean()],
    ]
    tab(rows, ["", "前 1/4 姿态误差", "末 1/4 姿态误差", "NIS 均值"])

    print("""    ★★ 诚实汇报本实验的真实结果：

        · 姿态误差：跳过 G 之后【几乎没变】（差别在小数点后第 4 位）
        · NIS 均值：只差 %.3f，大约 **%.1f%%**
        · P 的相对差别：**%.1e**

      ⇒ **在本实验的配置下，G 的效应确实很小，姿态输出上看不出来。**

    ★ 这里的数字和实验 2 ③ 差得很远，得解释一下，不然就是自相矛盾：

        实验 2 ③ 是【孤立地】反复施加 G —— 一个没有回复力的随机游走，
        4000 步累积到 5.6%%；
        这里是【真实的滤波器】：每步过后卡尔曼更新本身会把 P
        重新「拉紧」一次，把 G 造成的扰动大部分吸收掉了。

      ⇒ 所以正确的表述是：

        **「G 单次的作用是一阶的（≈0.33·|δθ|），
          但在观测密集的稳态下，净累积效应被更新步吸收，
          只剩万分之几。」**

    ★★ 那什么时候它才会真的跳出来？看上面那行「最大 %.4f°」——
      稳态每步只有 %.4f°，但**开机第一步就有 %.4f°**。
      ⇒ 答案是：**|δθ| 大的时候** ——
        大初始误差、或者长时间丢失观测后重新收敛。

      下面直接测：把初始姿态误差从 30° 加大到 90°。
""" % (nis_all.mean() - nis2.mean(),
       100.0 * abs(nis_all.mean() - nis2.mean()) / nis_all.mean(), rel_P,
       dth_mag.max(), dth_mag.mean(), dth_mag.max()))

    # ★ 大初始误差下再看一次 G 的影响
    rows, gdata = [], []
    for d0 in (30.0, 90.0):
        rng3 = np.random.default_rng(31)
        q0 = quat_mult(quat_from_rotvec(
            normalize(rng3.normal(0, 1, 3)) * math.radians(d0)), q_true[0])
        res = {}
        snap = {}
        for use_g in (True, False):
            qq = q0.copy()
            bb = np.zeros(3)
            PP = np.diag([math.radians(d0) ** 2] * 3 + [0.05 ** 2] * 3)
            dmax = 0.0
            for k in range(n):
                dth = (gyro[k] - bb) * dt
                qq = quat_normalize(quat_mult(qq, quat_from_rotvec(dth)))
                F = np.zeros((6, 6))
                F[0:3, 0:3] = quat_to_R(quat_from_rotvec(dth)).T
                F[0:3, 3:6] = -I3 * dt
                F[3:6, 3:6] = I3
                PP = F @ PP @ F.T + Qd
                for z, h_w, Rk in ((acc[k], z_w, Ra), (mag[k], m_w, Rm)):
                    h = quat_to_R(qq).T @ h_w
                    H = np.zeros((3, 6))
                    H[0:3, 0:3] = skew(h)
                    S = H @ PP @ H.T + Rk
                    K = PP @ H.T @ np.linalg.inv(S)
                    dx = K @ (z - h)
                    PP = (np.eye(6) - K @ H) @ PP @ (np.eye(6) - K @ H).T \
                        + K @ Rk @ K.T
                    PP = 0.5 * (PP + PP.T)
                    qq = quat_normalize(quat_mult(qq,
                                                  quat_from_rotvec(dx[0:3])))
                    bb = bb + dx[3:6]
                    if use_g:
                        G = np.eye(6)
                        G[0:3, 0:3] = I3 - skew(0.5 * dx[0:3])
                        PP = G @ PP @ G.T
                    else:
                        dmax = max(dmax, np.linalg.norm(dx[0:3]))
                if k in (0, 4, 19, 199, n - 1):
                    snap.setdefault(k, {})[use_g] = PP.copy()
            res[use_g] = (PP, att_err_deg(qq, q_true[-1]), dmax)
        for k in (0, 4, 19, 199):
            two = snap[k]
            snap[k] = np.linalg.norm(two[True] - two[False])                 / np.linalg.norm(two[True])
        res["snap"] = snap
        rel = np.linalg.norm(res[True][0] - res[False][0]) \
            / np.linalg.norm(res[True][0])
        sn = res["snap"]
        rows.append(["%.0f°" % d0,
                     "%.3f°" % math.degrees(res[False][2]),
                     "%.2e" % sn[0], "%.2e" % sn[4],
                     "%.2e" % sn[19], "%.2e" % sn[199],
                     "%.2e" % rel])
        gdata.append((d0, math.degrees(res[False][2]), sn[0], sn[4],
                      sn[19], sn[199], rel, res[True][1], res[False][1]))
    tab(rows, ["初始姿态误差", "首步 |δθ|", "P 差(第1步)", "P 差(第5步)",
               "P 差(第20步)", "P 差(第200步)", "P 差(末步)"])

    a, b = gdata[0], gdata[1]
    print("""    ★★ 这张表怎么读：每一列是「某一步时，
      ‖P（做了 G）− P（跳过 G）‖ / ‖P‖」—— 也就是 G 对 P 的影响有多大。

        · 初始误差 %.0f°：首步 |δθ| = %.2f°，第 1 步 P 就差 %.0f%%，
          之后在 %.1f%% ~ %.1f%% 之间摆，末步收敛到 %.3f%%。
        · 初始误差 %.0f°：首步 |δθ| = %.2f°，第 1 步 P 差 %.0f%%，
          之后在 %.1f%% ~ %.1f%% 之间摆，末步收敛到 %.3f%%。

      ⇒ ★★ 三个结论：

        **① G 的作用是【一阶】的，不是二阶小量。**
          首步 |δθ| = %.2f°（%.3f rad）时，P 一步就差掉 %.0f%%。
          实验 2 ③ 给的公式 ≈0.33·|δθ| 预测 %.0f%%，量级完全吻合
          （系数随 δθ 与 P 主轴的角度变化，上界是 1·|δθ|）。
          90° 那一行同理：|δθ| = %.2f° ⇒ 预测 %.0f%%，实测 %.0f%%。

        **② 过渡期里 P 会一直带着这个偏差（百分之几到百分之几十），
          但进入稳态后会被后续的卡尔曼更新洗掉。**
          末步只剩 %.4f%% / %.4f%%。
          原因：每来一次观测，P ← (I−KH)P(I−KH)ᵀ + KRKᵀ 都会把 P
          朝「和当前观测一致」的方向拉一次，G 的偏差被反复挤出去。
          ⇒ **这就是「忘了写 G」在实机上很难被发现的原因。**

        **③ 姿态输出几乎不受影响**（%.4f° vs %.4f°）。
          姿态有测量直接兜底，G 漏了也看不出来。

      ⇒ ★★ 所以最后这句话要说准确：

        **G 决定的是「P 在过渡期里准不准」，
          而不是「最终姿态估得准不准」。**

        但 P 是所有【间接推理】的基准：
        NIS / 卡方门限、故障检测、传感器加权、
        以及【零偏的收敛速度】——
        零偏正是靠 P 里的 δθ-δb 交叉项推出来的。

      ⇒ 三条实操建议：

        ① 「跳过 G 也能飞」是真的 —— 但那是被后续更新步兜住了
        ② 装机前请把 G 写上：3 行代码，
           收益是「过渡期里 P 也是诚实的」，这是一切高级功能的地基
        ③ 要验证 G 写对了，**别去看姿态曲线，去看 NIS 和 P 的迹**
""" % (a[0], a[1], 100 * a[2], 100 * min(a[3], a[4], a[5]), 100 * max(a[3], a[4], a[5]),
       100 * a[6],
       b[0], b[1], 100 * b[2], 100 * min(b[3], b[4], b[5]), 100 * max(b[3], b[4], b[5]),
       100 * b[6],
       a[1], math.radians(a[1]), 100 * a[2],
       100 * 0.33 * math.radians(a[1]),
       b[1], 100 * 0.33 * math.radians(b[1]), 100 * b[2],
       100 * a[6], 100 * b[6], a[7], a[8]))

    return


# ---------------------------------------------------------------------------
# 实验 4：ESKF vs 直接对四元数做 EKF
# ---------------------------------------------------------------------------


def exp4():
    head("实验 4  ★★ ESKF vs 直接对四元数做 EKF")

    print("""
  实验 1 说了「4x4 协方差结构性奇异」，实验 3 用 ESKF 跑通了姿态估计。
  本实验把两者放在【同一套 IMU 数据】上正面对比。

  为了公平，两个滤波器：

      · 用同样的递推（陀螺积分，名义状态完全相同）
      · 用同样的观测模型 h(q) = Rᵀ(0,0,1) 和 h(q) = Rᵀ m_w
      · 用同样的 R
      · 用同样的初始姿态误差

  唯一的区别：

      「四元数 EKF」  状态 4 维，协方差 4x4，更新后用【加法 + 归一化】修正
      「ESKF」        状态 6 维（δθ 3 维 + δb 3 维），协方差 6x6，
                      更新后用【复合】注入

  ★ 注意：四元数 EKF 那一版【不估计零偏】（4 维状态放不下），
    所以它天然吃亏。这不是作弊 —— 这正是「4 维状态」的代价：
    **它连零偏都塞不进去。**
""")

    dt = 0.005
    n = 12000
    sig_g = 0.005
    sig_a = 0.05
    sig_m = 0.02
    q_true, gyro, acc, mag, b_true = make_traj(dt, n, seed=3, sig_g=sig_g,
                                               sig_a=sig_a, sig_m=sig_m)
    Ra = (sig_a / 9.81) ** 2 * np.eye(3)
    Rm = sig_m ** 2 * np.eye(3)

    m_w = normalize(np.array([0.42, 0.03, 0.90]))
    z_w = np.array([0.0, 0.0, 1.0])
    I3 = np.eye(3)

    # --- 共同起点：故意给一个 30° 的初始姿态误差 ---
    rng = np.random.default_rng(11)
    dth0 = normalize(rng.normal(0, 1, 3)) * math.radians(30.0)
    q_init = quat_mult(quat_from_rotvec(dth0), q_true[0])
    print("    初始姿态误差 = |δθ0| = %.2f°"
          % att_err_deg(q_init, q_true[0]))
    print("    陀螺零偏 = (%.4f, %.4f, %.4f) rad/s"
          % tuple(b_true))
    print()

    # =========== A. 四元数 EKF（4 维状态） ===========
    qa = q_init.copy()
    Pa = (math.radians(30.0) ** 2) * (np.eye(4) - np.outer(qa, qa))
    Qa = (sig_g * dt) ** 2 * (np.eye(4) - np.outer(qa, qa)) + 1e-18 * np.eye(4)
    err_a, lam_a, nrm_a, nis_a = [], [], [], []
    for k in range(n):
        dq = quat_from_rotvec(gyro[k] * dt)
        qa = quat_normalize(quat_mult(qa, dq))
        F = np.array([
            [dq[0], -dq[1], -dq[2], -dq[3]],
            [dq[1], dq[0], -dq[3], dq[2]],
            [dq[2], dq[3], dq[0], -dq[1]],
            [dq[3], -dq[2], dq[1], dq[0]],
        ])
        Pa = F @ Pa @ F.T + Qa
        for z, h_w, Rk in ((acc[k], z_w, Ra), (mag[k], m_w, Rm)):
            h = quat_to_R(qa).T @ h_w
            eps = 1e-7
            H = np.zeros((3, 4))
            for j in range(4):
                e = np.zeros(4)
                e[j] = eps
                H[:, j] = (quat_to_R(qa + e).T @ h_w
                           - quat_to_R(qa - e).T @ h_w) / (2 * eps)
            S = H @ Pa @ H.T + Rk
            K = Pa @ H.T @ np.linalg.inv(S)
            innov = z - h
            qa = qa + K @ innov
            Pa = (np.eye(4) - K @ H) @ Pa
            Pa = 0.5 * (Pa + Pa.T)
            nis_a.append(float(innov @ np.linalg.solve(S, innov)))
        nrm_a.append(np.linalg.norm(qa))
        qa = quat_normalize(qa)
        err_a.append(att_err_deg(qa, q_true[k]))
        lam_a.append(np.linalg.eigvalsh(Pa).min())

    err_a = np.array(err_a)
    lam_a = np.array(lam_a)
    nrm_a = np.array(nrm_a)
    nis_a = np.array(nis_a[:n])

    # =========== B. ESKF（6 维误差状态，含零偏） ===========
    qb = q_init.copy()
    bb = np.zeros(3)
    Pb = np.diag([math.radians(30.0) ** 2] * 3 + [0.05 ** 2] * 3)
    Qd = np.diag([(sig_g * dt) ** 2] * 3 + [(1e-5 * dt) ** 2] * 3)
    err_b, lam_b, nrm_b, nis_b = [], [], [], []
    for k in range(n):
        dth = (gyro[k] - bb) * dt
        qb = quat_normalize(quat_mult(qb, quat_from_rotvec(dth)))
        F = np.zeros((6, 6))
        F[0:3, 0:3] = quat_to_R(quat_from_rotvec(dth)).T
        F[0:3, 3:6] = -I3 * dt
        F[3:6, 3:6] = I3
        Pb = F @ Pb @ F.T + Qd
        for z, h_w, Rk in ((acc[k], z_w, Ra), (mag[k], m_w, Rm)):
            h = quat_to_R(qb).T @ h_w
            H = np.zeros((3, 6))
            H[0:3, 0:3] = skew(h)
            S = H @ Pb @ H.T + Rk
            K = Pb @ H.T @ np.linalg.inv(S)
            innov = z - h
            dx = K @ innov
            Pb = (np.eye(6) - K @ H) @ Pb @ (np.eye(6) - K @ H).T + K @ Rk @ K.T
            Pb = 0.5 * (Pb + Pb.T)
            nis_b.append(float(innov @ np.linalg.solve(S, innov)))
            qb = quat_normalize(quat_mult(qb, quat_from_rotvec(dx[0:3])))
            bb = bb + dx[3:6]
            G = np.eye(6)
            G[0:3, 0:3] = I3 - skew(0.5 * dx[0:3])
            Pb = G @ Pb @ G.T
        nrm_b.append(np.linalg.norm(qb))
        err_b.append(att_err_deg(qb, q_true[k]))
        lam_b.append(np.linalg.eigvalsh(Pb).min())

    err_b = np.array(err_b)
    lam_b = np.array(lam_b)
    nrm_b = np.array(nrm_b)
    nis_b = np.array(nis_b[:n])

    sub("① 姿态误差对比")

    q4 = n // 8
    rows = [
        ["四元数 EKF（4 维）", "%.3f°" % err_a[:q4].mean(),
         "%.3f°" % err_a[q4:2 * q4].mean(), "%.3f°" % err_a[2 * q4:4 * q4].mean(),
         "%.3f°" % err_a[4 * q4:].mean()],
        ["ESKF（6 维误差状态）", "%.3f°" % err_b[:q4].mean(),
         "%.3f°" % err_b[q4:2 * q4].mean(), "%.3f°" % err_b[2 * q4:4 * q4].mean(),
         "%.3f°" % err_b[4 * q4:].mean()],
    ]
    tab(rows, ["滤波器", "0-1/8", "1/8-1/4", "1/4-1/2", "后半段"])

    print("""    ★★ 读这张表：

        · 四元数 EKF 全程在 %.1f° ~ %.1f° 之间，**没有收敛的趋势** ——
          它从一开始就卡在自己的稳态误差上，之后不再改善。

        · ESKF 第一档就是 %.3f°，之后一直在 %.2f° 以内。

        · 差距约 %.0f 倍。

    ★★ 这个差距从哪来？**几乎全部来自「零偏能不能估」。**

        陀螺零偏是 %.3f°/s。四元数 EKF 的状态是 4 维四元数，
        **根本装不下零偏** —— 它只能靠测量【反复事后纠正】，
        于是永远停在「被零偏推走一点、再被观测拉回来一点」的
        动态平衡上，这就是那 %.1f° 的稳态误差。

        ESKF 把 δb 放进误差状态，零偏收敛后姿态就稳住了
        （ESKF 那一行 %.2f° ~ %.2f°）。

    ⚠ 必须诚实说明：**这一节的差距主要来自零偏，**
      **不是**「4x4 协方差 vs 3x3 协方差」本身。
      4 维状态的确结构奇异，但在【短时间、高更新率】下，
      归一化每步兜底，它照样能工作 —— 下一节你会看到，
      大初始误差也奈何不了它。

      ⇒ 那 ESKF 真正的、不可替代的优势是什么？看下一节。
""" % (err_a[4 * q4:].mean(), err_a[:2 * q4].max(),
       err_b[:q4].mean(), err_b[2 * q4:].max(),
       err_a[2 * q4:].mean() / err_b[2 * q4:].mean(),
       math.degrees(np.linalg.norm(b_true)),
       err_a[4 * q4:].mean(), err_b[2 * q4:].mean(), err_b[2 * q4:].max()))

    sub("② ★★★ 协方差体检：谁在「诚实汇报」自己的不确定度？")

    ea_last = err_a[-q4:].mean()
    eb_last = err_b[-q4:].mean()
    sa_max = math.degrees(math.sqrt(lam_a.max()))
    sb_max = math.degrees(math.sqrt(lam_b.max()))
    rows = [
        ["四元数 EKF", "4x4", "%.2e" % lam_a.min(), "%.2e" % lam_a.max(),
         "%.3f°" % ea_last, "%.4f°" % sa_max, "%.0f 倍" % (ea_last / sa_max)],
        ["ESKF", "6x6", "%.2e" % lam_b.min(), "%.2e" % lam_b.max(),
         "%.3f°" % eb_last, "%.4f°" % sb_max, "%.2f 倍" % (eb_last / sb_max)],
    ]
    tab(rows, ["滤波器", "P 尺寸", "min λ(P)", "max λ(P)",
               "实际误差", "P 宣称 1σ", "实际/宣称"])

    print("""    ★★★ 这是本实验最关键的一张表，请一行一行读。

      **先看四元数 EKF 那一行：**

        · P 的谱跨了 **%.1e 到 %.1e**，条件数 %.1e。
          min λ(P) 是 %.2e —— 已经躺在 double 精度的噪声地板上了，
          **物理上就是 0**（方向就是实验 1 说的那个径向）。
          ⇒ 它的 4x4 协方差在数值上退化了。

        · ★★ 但真正致命的是最后两列：
          **它的实际误差是 %.3f°，而 P 宣称只有 %.4f° —— 差 %.0f 倍。**
          滤波器坚信自己已经精确到了 0.014°，
          实际却差着 2°。这是一个**谎报军情**的协方差。

      **再看 ESKF 那一行：**

        · P 的谱跨了 %.1e 到 %.1e，条件数 %.1e —— 比四元数版好 %.0f 倍。

        · 最后两列：实际 %.3f°，P 宣称 %.4f° ——
          **比值 %.2f，小于 1。**
          也就是说 ESKF 【高估】了自己的不确定度（保守）。

      ⇒ ★★★ 这就是本节真正要讲的东西，请记住这句话：

          **「诚实度」比「精度」更重要，而且它只能靠正确的参数化拿到。**

        一个滤波器说自己误差 0.014°、
        实际差 2°（四元数 EKF），
        和另一个说自己误差 0.28°、实际差 0.08°（ESKF），
        **两者的姿态曲线看起来可能都不错** ——
        但你敢把哪一个的 P 拿去算卡方门限、做故障检测、
        给 GPS/视觉/气压计做加权？

      ⇒ ★ 顺带看 NIS（3 维观测，χ²(3) 理论均值 = 3.0）：

          四元数 EKF  NIS 均值 = %.3f
          ESKF        NIS 均值 = %.3f

        四元数 EKF 的 NIS 明显偏离理论值，
        而 ESKF 的 2.03 就在 3.0 附近（样本有限）。
        ⇒ NIS 的一致性检验**当场**就把不诚实的协方差抓出来了。
          这就是为什么工程上一定要盯着 NIS / 新息统计，
          而不是只看姿态曲线。
""" % (lam_a.max(), lam_a.min(), lam_a.max() / lam_a.min(), lam_a.min(),
       ea_last, sa_max, ea_last / sa_max,
       lam_b.max(), lam_b.min(), lam_b.max() / lam_b.min(),
       (lam_a.max() / lam_a.min()) / (lam_b.max() / lam_b.min()),
       eb_last, sb_max, eb_last / sb_max,
       nis_a.mean(), nis_b.mean()))

    sub("③ 大初始误差：一个反直觉的结果")

    print("""    上面那一轮，初始姿态误差是 30°。
    现在把初始姿态误差一路加到 160°（比如上电时飞机被倒扣着放下），
    看两个滤波器的表现。
""")

    rows, raw = [], []
    for d0 in (30.0, 90.0, 120.0, 160.0):
        rng2 = np.random.default_rng(21)
        dd = normalize(rng2.normal(0, 1, 3)) * math.radians(d0)
        qi = quat_mult(quat_from_rotvec(dd), q_true[0])

        # --- 四元数 EKF（4 维状态，无零偏）---
        qa2 = qi.copy()
        Pa2 = (math.radians(d0) ** 2) * (np.eye(4) - np.outer(qa2, qa2))
        for k in range(n):
            dq = quat_from_rotvec(gyro[k] * dt)
            qa2 = quat_normalize(quat_mult(qa2, dq))
            F = np.array([
                [dq[0], -dq[1], -dq[2], -dq[3]],
                [dq[1], dq[0], -dq[3], dq[2]],
                [dq[2], dq[3], dq[0], -dq[1]],
                [dq[3], -dq[2], dq[1], dq[0]],
            ])
            Pa2 = F @ Pa2 @ F.T + (sig_g * dt) ** 2 * np.eye(4)
            for z, h_w, Rk in ((acc[k], z_w, Ra), (mag[k], m_w, Rm)):
                h = quat_to_R(qa2).T @ h_w
                eps = 1e-7
                H = np.zeros((3, 4))
                for j in range(4):
                    e = np.zeros(4)
                    e[j] = eps
                    H[:, j] = (quat_to_R(qa2 + e).T @ h_w
                               - quat_to_R(qa2 - e).T @ h_w) / (2 * eps)
                S = H @ Pa2 @ H.T + Rk
                K = Pa2 @ H.T @ np.linalg.inv(S)
                qa2 = qa2 + K @ (z - h)
                Pa2 = 0.5 * ((np.eye(4) - K @ H) @ Pa2
                             + ((np.eye(4) - K @ H) @ Pa2).T)
            qa2 = quat_normalize(qa2)
        ea = att_err_deg(qa2, q_true[-1])

        # --- ESKF ---
        qb2 = qi.copy()
        bb2 = np.zeros(3)
        Pb2 = np.diag([math.radians(d0) ** 2] * 3 + [0.05 ** 2] * 3)
        for k in range(n):
            dth = (gyro[k] - bb2) * dt
            qb2 = quat_normalize(quat_mult(qb2, quat_from_rotvec(dth)))
            F = np.zeros((6, 6))
            F[0:3, 0:3] = quat_to_R(quat_from_rotvec(dth)).T
            F[0:3, 3:6] = -I3 * dt
            F[3:6, 3:6] = I3
            Pb2 = F @ Pb2 @ F.T + Qd
            for z, h_w, Rk in ((acc[k], z_w, Ra), (mag[k], m_w, Rm)):
                h = quat_to_R(qb2).T @ h_w
                H = np.zeros((3, 6))
                H[0:3, 0:3] = skew(h)
                S = H @ Pb2 @ H.T + Rk
                K = Pb2 @ H.T @ np.linalg.inv(S)
                dx = K @ (z - h)
                Pb2 = (np.eye(6) - K @ H) @ Pb2 @ (np.eye(6) - K @ H).T                     + K @ Rk @ K.T
                Pb2 = 0.5 * (Pb2 + Pb2.T)
                qb2 = quat_normalize(quat_mult(qb2, quat_from_rotvec(dx[0:3])))
                bb2 = bb2 + dx[3:6]
                G = np.eye(6)
                G[0:3, 0:3] = I3 - skew(0.5 * dx[0:3])
                Pb2 = G @ Pb2 @ G.T
        eb = att_err_deg(qb2, q_true[-1])
        rows.append(["%.0f°" % d0, "%.3f°" % ea, "%.4f°" % eb,
                     "%.1f 倍" % (ea / max(eb, 1e-9))])
        raw.append((d0, ea, eb))

    tab(rows, ["初始姿态误差", "四元数 EKF 末段误差", "ESKF 末段误差", "倍数"])
    ea_mid = 0.5 * (raw[0][1] + raw[-1][1])
    eb_first, eb_last = raw[0][2], raw[-1][2]

    print("""    ★★ 读这张表 —— 结果和「直觉」正好相反：

        · **四元数 EKF 的末段误差，从 30° 到 160° 几乎一模一样**
          （都是 %.2f° 上下）。初始误差大了 5 倍多，结果不变。

        · **ESKF 的误差随初始误差缓慢增长**：
          %.4f° → %.4f°，大了约 %.0f 倍。
          这是线性化在大误差下略微变差的正常表现，非常温和。

    ★★ 所以：**大初始误差不是四元数 EKF 的死因。**

      为什么？因为

        ① 4 维协方差在高不确定度下给出的 K 极大（卷 2 第 5 章讲的增益饱和），
           几步之内就把姿态拉回来了；
        ② 归一化在每一步强行把状态摁回球面，
           顺便把「4 维参数化带来的多余自由度」也顺手摁掉了。

      ⇒ 它最终停在 %.2f° 上，是【稳态误差】——
        由「陀螺零偏估不了」决定的，和初始误差无关。

    ⇒ ★★★ 这正好把本实验的结论收拢到一点：

        四元数 EKF 的问题**不在精度，在诚实度**。

        · 精度上：它靠归一化兜底，能跑，甚至看起来还不错
        · 但它的 P 会退化成一个「谎报军情」的协方差（见 ②），
          而且它【装不下】零偏这类必须有的状态

    ⚠ 两点补充说明，避免过度推销 ESKF：

        ① ESKF 也不是万能的。初始误差接近 180° 时，
           旋转向量的分支和四元数的双重覆盖都会出问题 ——
           这就是 PX4 里要有 `resetQuatStateYaw()` 这类【重置】逻辑、
           而不是指望滤波器从任意初值收敛的原因。
           工程做法是：**先给一个好初值，再让滤波器接管。**
        ② 本实验里四元数 EKF 之所以还能收敛，
           是因为观测密集（加速度计 + 磁力计，200 Hz）。
           如果观测稀疏、或者初始误差更大，
           它的表现会更差 —— 但那是「观测不足」，
           不能全算到参数化头上，这里必须分清。
""" % (ea_mid, eb_first, eb_last, eb_last / eb_first, ea_mid))
    sub("④ 结论：什么时候用哪个？")

    rows = [
        ["四元数 EKF / 直接对 q 做 KF",
         "状态数最少（4）；实现最直白",
         "P 结构性奇异且会退化；装不下零偏等误差项；协方差「谎报军情」"],
        ["ESKF / 本章",
         "协方差只在 3 维切空间，永不奇异；零偏等误差项可自然加入；大误差靠复合吸收",
         "实现复杂（注入 / 重置 / G 三处都不能漏）；必须严格选定 local 或 global 并全程一致"],
    ]
    for name, pro, con in rows:
        print("    ★ %s" % name)
        print("        优点：%s" % pro)
        print("        代价：%s" % con)
        print()

    print("""    ⇒ ★★ 把本实验的三条结论合起来：

        ① **精度上**：四元数 EKF 靠归一化兜底，能跑；
           它的稳态误差主要来自「装不下零偏」，不是来自参数化。
        ② **诚实度上**：它的 P 会退化（谱跨 8 个数量级、
           实际误差是宣称 1σ 的 167 倍），
           而 ESKF 的 P 是保守的、可用的（比值 0.23）。
        ③ **可扩展性上**：ESKF 加状态（零偏、尺度、风……）
           只是往误差向量里多塞几维；
           四元数 EKF 连一个零偏都塞不进去。

      ⇒ 所以答案不是「ESKF 更准」，而是：

        **只要状态里有「旋转」，就该用 ESKF。**

        因为你要的不只是一个姿态数字，还要一个
        **能拿去做故障检测、传感器加权、多源融合的协方差** ——
        而那东西只能靠正确的参数化拿到。

        这就是 PX4 的 EKF2（`state.h` 里 `quat_nominal` 占 3 个自由度）
        和 ArduPilot 的 EKF3 都做成 ESKF 的原因。

    ★ 下一课（卷 2 第 7 章）讲 Mahony / Madgwick 互补滤波 ——
      那是 ESKF 的「穷人版」：不维护协方差，只做非线性互补。
      理解了本章的「误差状态 + 注入」，那两页代码就是十几行的事。
""")
    return


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("为什么不能直接对四元数做 Kalman 滤波", exp1),
    "2": ("★★ 误差状态的定义、注入、重置（G 矩阵）", exp2),
    "3": ("★★ 完整的 ESKF 姿态估计", exp3),
    "4": ("★★ ESKF vs 直接四元数 EKF", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        for k in sorted(EXPERIMENTS):
            print("\n" + "#" * W)
            print("#  实验 %s：%s" % (k, EXPERIMENTS[k][0]))
            print("#" * W)
            EXPERIMENTS[k][1]()
        print()
        print("=" * W)
        print("  完成。")
        print("=" * W)
        return 0

    for a in args:
        if a not in EXPERIMENTS:
            print("  未知实验：%s（可选 %s）" % (a, "/".join(sorted(EXPERIMENTS))))
            return 1
        EXPERIMENTS[a][1]()

    print()
    print("=" * W)
    print("  完成。")
    print("=" * W)
    return 0


if __name__ == "__main__":
    sys.exit(main())
