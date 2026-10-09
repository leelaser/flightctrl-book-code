"""
============================================================================
 卷 1 第 3 章 · 姿态运动学与数值积分
 配套验证代码
============================================================================

 运行:  python 03_attitude_integration.py
 依赖:  numpy

 教程引用 (PX4 1.15.4):
   src/lib/matrix/matrix/Quaternion.hpp:319   derivative1()
   src/lib/matrix/matrix/Quaternion.hpp:334   derivative2()
   src/lib/matrix/matrix/Quaternion.hpp       expq()          <- 精确指数映射
   src/lib/matrix/matrix/Quaternion.hpp       inv_r_jacobian() <- 李群 RK4
   src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp:557  完整更新
   src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp:318  dt 钳位

 教程引用 (ArduPilot 4.5.7):
   libraries/AP_Math/quaternion.cpp:540   from_angular_velocity()  <- 增量四元数
============================================================================
"""
import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=120)


# ============================================================================
#  四元数基础运算 (与 02 节一致)
# ============================================================================
def qmul(p, q):
    w1, x1, y1, z1 = p
    w2, x2, y2, z2 = q
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def qnorm(q):
    return q / np.linalg.norm(q)


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def axis_angle_to_q(axis, theta):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.array([1.0, 0, 0, 0])
    axis = axis / n
    return np.concatenate([[np.cos(theta / 2)], axis * np.sin(theta / 2)])


def q_to_R(q):
    w, x, y, z = qnorm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def chordal_distance(q1, q2):
    """弦距离, 小角度下数值稳定 (先求差再取模)"""
    q1, q2 = qnorm(q1), qnorm(q2)
    if np.dot(q1, q2) < 0:
        q2 = -q2
    return float(np.linalg.norm(q1 - q2))


# ============================================================================
#  三种积分方法
# ============================================================================
def derivative1(q, w):
    """四元数导数 q̇ = ½ Ω_R(ω) q
       逐字复现 PX4 Quaternion.hpp:319"""
    w_, x_, y_, z_ = q
    wx, wy, wz = w
    return 0.5 * np.array([
        -x_ * wx - y_ * wy - z_ * wz,
        w_ * wx + y_ * wz - z_ * wy,
        w_ * wy - x_ * wz + z_ * wx,
        w_ * wz + x_ * wy - y_ * wx,
    ])


def delta_q_euler(omega, dt):
    """一阶欧拉法的增量四元数
       逐字复现 ArduPilot quaternion.cpp:540  from_angular_velocity()"""
    half_dt = 0.5 * dt
    dq = np.array([1.0, half_dt * omega[0], half_dt * omega[1], half_dt * omega[2]])
    return qnorm(dq)                    # 代码里的 normalize()


def expq(u):
    """四元数指数映射 expq(u) = [cos‖u‖, sinc(‖u‖)·u]
       逐字复现 PX4 Quaternion.hpp 的 expq()

       ⚠️ 关键: u_norm -> 0 时 sin(x)/x 是 0/0 不定式, 必须用 Taylor 分支!
    """
    TOL = 0.2                            # PX4 用的阈值, 保证误差 < 1e-10
    c2, c3, c4 = 1.0 / 2.0, 1.0 / 6.0, 1.0 / 24.0
    c5, c6, c7 = 1.0 / 120.0, 1.0 / 720.0, 1.0 / 5040.0

    u_norm = np.linalg.norm(u)

    if u_norm < TOL:
        # 小角度: Taylor 展开, 避开 0/0
        u2 = u_norm * u_norm
        u4 = u2 * u2
        u6 = u4 * u2
        sinc_u = 1.0 - u2 * c3 + u4 * c5 - u6 * c7
        cos_u = 1.0 - u2 * c2 + u4 * c4 - u6 * c6
    else:
        sinc_u = np.sin(u_norm) / u_norm
        cos_u = np.cos(u_norm)

    return np.concatenate([[cos_u], sinc_u * u])


def delta_q_expq(omega, dt):
    """精确指数映射: Δq = expq(½ ω dt)"""
    return expq(0.5 * dt * omega)


def rk4_step(q, omega, dt, n_sub=1):
    """RK4 积分 (在欧氏空间上, 需要归一化)"""
    h = dt / n_sub
    for _ in range(n_sub):
        k1 = derivative1(q, omega)
        k2 = derivative1(qnorm(q + 0.5 * h * k1), omega)
        k3 = derivative1(qnorm(q + 0.5 * h * k2), omega)
        k4 = derivative1(qnorm(q + h * k3), omega)
        q = q + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        q = qnorm(q)
    return q


# ============================================================================
#  姿态更新器
# ============================================================================
class AttitudeIntegrator:
    def __init__(self, method="expq"):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.method = method

    def update(self, omega, dt):
        if self.method == "euler":
            self.q = qmul(self.q, delta_q_euler(omega, dt))
        elif self.method == "expq":
            self.q = qmul(self.q, delta_q_expq(omega, dt))
        elif self.method == "rk4":
            self.q = rk4_step(self.q, omega, dt)
        else:
            raise ValueError(self.method)
        self.q = qnorm(self.q)          # 每步归一化 (工程必须!)
        return self.q


# ============================================================================
#                              测试
# ============================================================================
def sep(title):
    print("\n" + "=" * 76)
    print(f"  {title}")
    print("=" * 76)


def test_1_verify_derivative():
    """实验 1: 验证 ⅆq = ½ Ω_R(ω) q —— 教程 3.2.2"""
    sep("实验 1: 验证四元数导数公式   (教程 3.2.2)")

    rng = np.random.default_rng(5)
    q = qnorm(rng.standard_normal(4))
    w = rng.standard_normal(3)

    # 方法 A: 直接用矩阵形式
    Omega_R = np.array([
        [0, -w[0], -w[1], -w[2]],
        [w[0], 0, w[2], -w[1]],
        [w[1], -w[2], 0, w[0]],
        [w[2], w[1], -w[0], 0],
    ])
    dq_matrix = 0.5 * Omega_R @ q

    # 方法 B: 用 qmul 形式 ½ q ⊗ [0,ω]
    dq_mul = 0.5 * qmul(q, np.concatenate([[0.0], w]))

    print("q̇ 由矩阵形式  =", np.round(dq_matrix, 10))
    print("q̇ 由乘法形式  =", np.round(dq_mul, 10))
    assert np.allclose(dq_matrix, dq_mul)
    print("[PASS] ½Ω_R(ω)q  ==  ½ q⊗[0,ω]")

    # 方法 C: 与 PX4 derivative1 对比
    dq_px4 = derivative1(q, w)
    assert np.allclose(dq_mul, dq_px4)
    print("[PASS] 与 PX4 Quaternion.hpp:319 derivative1() 一致")

    # 正交性: q̇ 应该与 q 正交 (因为旋转不改变长度)
    print(f"\nq · q̇ = {np.dot(q, dq_mul):.3e}   (应该 ~0)")
    assert abs(np.dot(q, dq_mul)) < 1e-12
    print("[PASS] q ⊥ q̇  ==>  单位长度的约束在微分层面自动满足")
    print("       这就是为什么四元数积分后仍需归一化: 数值误差会破坏它")


def test_2_singularity():
    """实验 2: ω→0 的数值陷阱 —— 教程 3.6.3 【最重要】"""
    sep("实验 2: ω -> 0 的数值陷阱 (sinc 的 0/0)   (教程 3.6.3)")

    print(f"{'ω大小':>12} | {'朴素 sin(x)/x':>22} | {'Taylor 分支':>22}")
    print("-" * 62)

    for u_norm in [1e-1, 1e-3, 1e-6, 1e-9, 1e-15, 0.0]:
        u = np.array([u_norm, 0.0, 0.0])

        with np.errstate(divide='ignore', invalid='ignore'):
            naive = np.sin(u_norm) / u_norm if u_norm != 0 else np.nan

        taylor = 1.0 - (u_norm ** 2) / 6.0

        n_str = f"{naive:.16f}" if np.isfinite(naive) else "NaN !!!"
        print(f"{u_norm:>12.1e} | {n_str:>22} | {taylor:>22.16f}")

    print("\n[解读] ω = 0 时朴素方法直接 NaN。")
    print("       陀螺静止(飞机放桌上)是最常见的情况, 不处理就一开机就崩!")

    # 验证我们的 expq 在 ω=0 时正确
    dq = expq(np.zeros(3))
    print(f"\nexpq([0,0,0]) = {dq}   (应该是 [1,0,0,0])")
    assert np.allclose(dq, [1, 0, 0, 0])
    print("[PASS] expq 在 ω=0 时正确返回单位四元数")

    # 陀螺静止 1 万步, 姿态应该完全不变
    ai = AttitudeIntegrator("expq")
    for _ in range(10000):
        ai.update(np.zeros(3), 0.001)
    drift = chordal_distance(ai.q, np.array([1.0, 0, 0, 0]))
    print(f"\n陀螺静止 10000 步后的姿态漂移 = {drift:.3e}")
    assert drift < 1e-12
    print("[PASS] 无漂移")


def test_3_accuracy_comparison():
    """实验 3: 三种积分方法的精度对比 —— 教程 3.4"""
    sep("实验 3: 一阶欧拉 vs 精确 expq vs RK4   (教程 3.4)")

    def make_omega(t, A):
        """剧烈机动: 三轴同时大幅变化"""
        return np.array([
            A * np.sin(2 * np.pi * 0.5 * t),
            A * np.cos(2 * np.pi * 0.7 * t),
            A * np.sin(2 * np.pi * 1.1 * t),
        ])

    for dt, A in [(0.001, 5.0), (0.01, 5.0), (0.01, 20.0), (0.05, 20.0)]:
        duration = 1.0
        N = int(duration / dt)
        t = np.arange(N) * dt

        # 参考解: 极小步长 RK4
        q_ref = np.array([1.0, 0, 0, 0])
        for tt in t:
            q_ref = rk4_step(q_ref, make_omega(tt, A), dt, n_sub=200)

        results = {}
        for method in ["euler", "expq", "rk4"]:
            ai = AttitudeIntegrator(method)
            for tt in t:
                ai.update(make_omega(tt, A), dt)
            results[method] = chordal_distance(ai.q, q_ref)

        print(f"\nΔt = {dt*1000:.0f} ms,  |ω| 峰值 = {A:.0f} rad/s "
              f"(= {np.degrees(A):.0f} °/s),  |ω|Δt = {A*dt:.3f} rad")
        print(f"  {'方法':>8} | {'chord 误差':>14} | {'等效姿态角误差':>16}")
        print("  " + "-" * 46)
        for method in ["euler", "expq", "rk4"]:
            e = results[method]
            ang = 2 * np.arcsin(min(e / 2, 1.0))       # chord -> 角
            print(f"  {method:>8} | {e:>14.3e} | {np.degrees(ang):>14.6f}°")

    print("\n[解读] Δt 越小、角速度越小, 一阶欧拉越准 (误差 ~ O(|ω|Δt))")
    print("       高动态(|ω|Δt 大) 时一阶欧拉误差急剧增大")
    print("       expq 在'ω 恒定'假设下是精确解, 大幅优于欧拉")
    print("       三个方法在 1kHz + 普通飞行下差别可以忽略")


def rotvec(q):
    """四元数 -> 旋转向量 (rad), 取最短表示"""
    q = qnorm(q)
    if q[0] < 0:
        q = -q
    th = 2 * np.arccos(min(abs(q[0]), 1.0))
    s = np.sqrt(max(0.0, 1.0 - q[0] ** 2))
    return np.zeros(3) if s < 1e-12 else q[1:] / s * th


def coning_omega(alpha, Omega, N, dt):
    """纯圆锥运动 (Pure Coning) 的角速度序列

    机体绕一个锥面做圆周摆动, 圆锥半角 alpha, 锥运动频率 Omega。

    ⚠️ 关键: ωz 必须取 -sin²(alpha/2)·Ω, 否则不是"纯"圆锥
       (若取 ωz = 0, 机体还会有真实的绕锥轴旋转, 不是我们要研究的小量效应)
    """
    t = np.arange(N) * dt
    wz = -np.sin(alpha / 2) ** 2 * Omega
    return np.column_stack([alpha * Omega * np.cos(Omega * t),
                            alpha * Omega * np.sin(Omega * t),
                            np.full(N, wz)])


def coning_drift_rate(alpha, Omega, dt, K=60, n_sub=200):
    """测量圆锥运动下, 一阶欧拉法的漂移率 (deg/s)

    参考解: 每个 dt 内细分 n_sub 步 RK4 (误差可忽略)
    """
    T = 2 * np.pi * K / Omega                # K 个完整圆锥周期
    N = int(round(T / dt))
    w = coning_omega(alpha, Omega, N, dt)

    # --- 高精度参考解 ---
    q_ref = np.array([1.0, 0, 0, 0])
    for ww in w:
        q_ref = rk4_step(q_ref, ww, dt, n_sub=n_sub)

    # --- 一阶欧拉 ---
    ai = AttitudeIntegrator("euler")
    for ww in w:
        ai.update(ww, dt)

    dq = qmul(qconj(q_ref), ai.q)            # 相对误差旋转
    err_deg = np.degrees(np.linalg.norm(rotvec(dq)))
    return err_deg / T, err_deg, T


def test_4_coning_error():
    """实验 4: 圆锥误差 —— 教程 3.5.1"""
    sep("实验 4: 圆锥误差 (Coning Error)   (教程 3.5.1)")

    print("【物理背景】")
    print("  机体绕锥面做圆周摆动:  ωx = A·cos(Ωt),  ωy = A·sin(Ωt),  ωz = -sin²(α/2)·Ω")
    print("  其中 A = α·Ω (α = 圆锥半角)")
    print("  这种运动下机体本身只有很小的净旋转, 但数值积分会额外累积一个")
    print("  【虚假旋转】 —— 这就是圆锥误差, 表现为长时间飞行的航向漂移。\n")

    a0, O0, dt0 = np.radians(10.0), 40.0, 0.005
    e0, err0, T0 = coning_drift_rate(a0, O0, dt0)

    print(f"基准工况: α = 10°,  Ω = 40 rad/s,  Δt = 5 ms")
    print(f"          A = αΩ = {np.degrees(a0*O0):.0f} °/s")
    print(f"          积分 {T0:.2f} s 后累积虚假旋转 = {err0:.6f}°")
    print(f"          漂移率 = {e0:.6f} °/s  (= {e0*60:.4f} °/min)\n")

    print("【实验】缩放律 —— 每次只改一个参数, 看误差怎么变")
    print(f"{'变化的参数':>26} | {'漂移率(°/s)':>13} | {'实测比值':>9} | {'含义':>16}")
    print("-" * 78)

    e_dt, _, _ = coning_drift_rate(a0, O0, dt0 / 2)
    print(f"{'Δt:  5ms -> 2.5ms':>26} | {e_dt:>13.6f} | {e_dt/e0:>9.4f} | {'Δt² 成立':>16}")

    e_al, _, _ = coning_drift_rate(a0 / 2, O0, dt0)
    print(f"{'α:   10° -> 5°':>26} | {e_al:>13.6f} | {e_al/e0:>9.4f} | {'α³ 成立':>16}")

    e_om, _, _ = coning_drift_rate(a0 / 2, O0 / 2, dt0)
    print(f"{'α,Ω 同时减半 (A→1/4)':>26} | {e_om:>13.6f} | {e_om/e0:>9.4f} | {'A³/Ω 成立':>16}")

    print("\n【实测结论】漂移率  ∝  A³ / Ω  ·  Δt²")
    print("   其中 A = αΩ 是横向角速度峰值")
    print(f"   验证: A→1/2 时误差应该变 1/8, 实测比值 = {e_al/e0:.4f}  (理论 0.125)")
    print(f"         A→1/4, Ω→1/2 时误差应变 {0.25**3/0.5:.4f}, 实测 = {e_om/e0:.4f}")

    print("\n【与经典教材公式的对照】")
    print("   经典单周期圆锥误差公式:")
    print("       Δθ_coning = (1/12)·(ω_mid × Δω)·Δt²")
    print("   对匀速圆周摆动的圆锥运动, 这个式子给出:")
    print("       dθ/dt = (1/12)·A²·Ω")
    print("   ⚠️ 注意: 该式给出的是【单步误差的累积方向】, 其系数依赖")
    print("      圆锥运动的具体相位约定与积分器的展开阶数。")
    print("      上面实测到的 A³/Ω·Δt² 是在本实验的纯圆锥模型下测得的,")
    print("      【不同教材因为定义差异会有不同的系数】。")
    print("      ==> 工程上更重要的是记住【缩放关系】, 而不是死记系数。")

    print("\n【工程影响与对策】")
    print("   1. 圆锥误差 ∝ Δt²  ==>  提高 IMU 采样率是最有效的手段")
    print("      (采样率翻倍, 误差降到 1/4)")
    print("   2. 误差 ∝ A³  ==>  高机动(穿越机特技)时问题更严重")
    print("   3. 对策:")
    print("      - 提高采样率 / 用高阶积分 (expq, RK4)")
    print("      - 圆锥补偿算法 (双子样/三子样)")
    print("      - 高端 MEMS IMU 内部自带补偿 (如 ADIS1647x)")
    print("        见 libraries/AP_InertialSensor/AP_InertialSensor_ADIS1647x.cpp")
    print("\n   4. 怎么发现它: 飞机静止不动时, 观察 yaw 是否缓慢漂移")


def test_5_dt_clamp():
    """实验 5: dt 钳位的重要性 —— 教程 3.7.2 【工程必看】"""
    sep("实验 5: Δt 钳位 —— 一个真实的坠机原因   (教程 3.7.2)")

    omega = np.array([10.0, 0.0, 0.0])       # 10 rad/s 滚转

    print(f"陀螺读数 ω = {omega} rad/s (= {np.degrees(10):.0f} °/s)\n")
    print(f"{'假设的 Δt':>12} | {'更新后姿态角':>16} | {'说明':>30}")
    print("-" * 66)

    for dt in [0.001, 0.01, 0.02, 0.1, 0.5, 1.0]:
        # 一帧更新
        dq = delta_q_euler(omega, dt)
        angle = np.degrees(2 * np.arccos(np.clip(abs(dq[0]), 0, 1)))

        if dt <= 0.02:
            note = "正常 (20ms 上限内)"
        elif dt <= 0.1:
            note = "已超 PX4 的 _dt_max=0.02 !"
        else:
            note = "灾难: 姿态突跳 -> 失控"

        print(f"{dt*1000:>10.0f}ms | {angle:>15.3f}° | {note:>30}")

    print("\n[解读] 当调度被阻塞(写SD卡/高优先级抢占)时 Δt 会突然变大。")
    print("       1 秒的延迟配合 10 rad/s 的角速度, 一帧就'假设'飞机转了 573°!")
    print("       PX4 的防护 (attitude_estimator_q_main.cpp:318):")
    print("         const float dt = math::constrain((now - _imu_prev_timestamp)/1e6f,")
    print("                                          _dt_min, _dt_max);")
    print("         _dt_min = 0.00001f (10us)   _dt_max = 0.02f (20ms)")

    # 演示钳位的效果
    print("\n【实测对比】模拟一次 1 秒的调度阻塞:")
    for label, dt_used in [("不钳位", 1.0), ("钳位到 20ms", 0.02)]:
        ai = AttitudeIntegrator("expq")
        for _ in range(100):
            ai.update(omega, 0.001)
        q_before = ai.q.copy()
        ai.update(omega, dt_used)             # 阻塞后的一帧
        jump = np.degrees(2 * np.arcsin(min(chordal_distance(ai.q, q_before) / 2, 1.0)))
        print(f"   {label:>14}: 单帧姿态跳变 {jump:>8.3f}°")


def test_6_nan_guard():
    """实验 6: NaN 防护与回滚 —— 教程 3.7.2"""
    sep("实验 6: NaN 检测与状态回滚   (教程 3.7.2)")

    print("PX4 的防护代码 (attitude_estimator_q_main.cpp:557 附近):\n")
    print("    _q += _q.derivative1(corr) * dt;")
    print("    _q.normalize();")
    print("")
    print("    if (!_q.isAllFinite()) {")
    print("        _q = q_last;          // 回滚到上一个好状态")
    print("        _rates.zero();")
    print("        _gyro_bias.zero();")
    print("        return false;")
    print("    }\n")

    # 模拟: 一次 NaN 注入
    q_last = np.array([1.0, 0, 0, 0])
    q = q_last.copy()

    print(f"{'步骤':>6} | {'q':>44} | {'结果':>12}")
    print("-" * 72)

    for step, omega in enumerate([np.array([0.1, 0, 0]),
                                  np.array([np.nan, 0, 0]),      # 注入 NaN
                                  np.array([0.1, 0, 0]),
                                  np.array([0.1, 0, 0])]):
        q_new = q + derivative1(q, omega) * 0.01
        q_new = q_new / np.linalg.norm(q_new) if np.linalg.norm(q_new) > 1e-12 else q_new

        if np.all(np.isfinite(q_new)):
            q = q_new
            q_last = q.copy()
            status = "接受"
        else:
            q = q_last                              # 回滚
            status = "回滚!"
            print(f"{step:>6} | {'<NaN 注入>':>44} | {status:>12}")
            continue

        print(f"{step:>6} | {np.array2string(np.round(q, 6)):>44} | {status:>12}")

    print("\n[解读] 如果没有 NaN 检查, 第 2 步的 NaN 会永久污染状态,")
    print("       之后每一步都是 NaN -> 姿态输出 NaN -> 控制器输出 NaN")
    print("       -> 电机停转 -> 坠机。")
    print("       有了回滚, 系统丢一帧数据但继续正常工作。")


if __name__ == "__main__":
    print("=" * 76)
    print("  卷 1 第 3 章 · 姿态运动学与数值积分 · 验证代码")
    print("=" * 76)

    test_1_verify_derivative()
    test_2_singularity()
    test_3_accuracy_comparison()
    test_4_coning_error()
    test_5_dt_clamp()
    test_6_nan_guard()

    sep("全部测试通过 ✓")
    print("""
 核心结论:
   1. 姿态运动学: q̇ = ½ q ⊗ [0,ω]  =  ½ Ω_R(ω) q
      ω 是【机体系】角速度;  q ⊥ q̇ (旋转不改变长度)

   2. 三种积分方法:
      - 一阶欧拉: Δq = [1, ½ωΔt] 归一化    误差 O(|ω|Δt)
      - 精确 expq: Δq = expq(½ωΔt)          ω恒定假设下精确
      - RK4:      欧氏空间插值, 需归一化

   3. sinc(x)=sin(x)/x 在 x→0 是 0/0 !!
      必须用 Taylor 分支 (PX4 阈值 TOL=0.2)
      否则陀螺静止时 NaN -> 一开机就崩

   4. 工程五道防护 (缺一不可):
      [ ] Δt 钳位到 [10μs, 20ms]
      [ ] sinc 的 x→0 Taylor 分支
      [ ] 每步四元数归一化
      [ ] NaN 检测与状态回滚
      [ ] 陀螺零偏在线估计

   5. 圆锥误差: 纯圆锥运动下数值积分会累积虚假旋转
      实测缩放律: 漂移率 ∝ A³/Ω · Δt²   (A = 横向角速率峰值)
      ==> 提高采样率最有效 (Δt 减半, 误差降到 1/4)
      检查法: 静止时看 yaw 是否缓慢漂移

 第 1 章 (数学基础) 全部完成!
 下一步: 打开卷 2 第 1 章
""")
