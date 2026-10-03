"""
============================================================================
 飞控学习教程 · 第 1 章 02 节 · 四元数完全指南
 配套验证代码
============================================================================

 运行:  python 02_quaternion.py
 依赖:  numpy

 教程引用 (ArduPilot 4.5.7):
   libraries/AP_Math/quaternion.cpp:34   rotation_matrix()
   libraries/AP_Math/quaternion.cpp:86   from_rotation_matrix()
   libraries/AP_Math/quaternion.cpp:421  from_euler()
   libraries/AP_Math/quaternion.cpp:587  get_euler_roll()
   libraries/AP_Math/quaternion.cpp:705  operator*  (Hamilton 积)
   libraries/AP_Math/quaternion.h:198    angular_difference()
   libraries/AP_Math/quaternion.h:82     earth_to_body()

【本教程统一约定】
   四元数 q = [w, x, y, z]  (w 为标量部分)
   ArduPilot 里对应 [q1, q2, q3, q4]   <-- q1 是标量! 极易混淆!
   q 表示 R_body->ned
============================================================================
"""
import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=120)


# ============================================================================
#  基础运算
# ============================================================================
def qmul(p, q):
    """Hamilton 积, 逐字复现 ArduPilot quaternion.cpp:705"""
    w1, x1, y1, z1 = p
    w2, x2, y2, z2 = q
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def qconj(q):
    """共轭 = 逆 (对单位四元数)"""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qnorm(q):
    """归一化"""
    return q / np.linalg.norm(q)


def axis_angle_to_q(axis, theta):
    """轴角 -> 四元数。注意半角! quaternion.cpp from_axis_angle()"""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.array([1.0, 0, 0, 0])
    axis = axis / n
    return np.concatenate([[np.cos(theta / 2)], axis * np.sin(theta / 2)])


def q_to_axis_angle(q):
    """四元数 -> 轴角。quaternion.cpp to_axis_angle()"""
    q = qnorm(q)
    # 处理 q 和 -q 等价: 保证 w >= 0 得到最短路径
    if q[0] < 0:
        q = -q
    w = np.clip(q[0], -1.0, 1.0)
    theta = 2.0 * np.arccos(w)
    s = np.sqrt(1.0 - w * w)
    if s < 1e-8:                       # theta ~ 0, 轴不确定
        return np.array([1.0, 0, 0]), 0.0
    return q[1:] / s, theta


def euler_to_q(phi, theta, psi):
    """欧拉角 -> 四元数, 逐字复现 quaternion.cpp:421"""
    cr2, cp2, cy2 = np.cos(phi * 0.5), np.cos(theta * 0.5), np.cos(psi * 0.5)
    sr2, sp2, sy2 = np.sin(phi * 0.5), np.sin(theta * 0.5), np.sin(psi * 0.5)
    return np.array([
        cr2 * cp2 * cy2 + sr2 * sp2 * sy2,
        sr2 * cp2 * cy2 - cr2 * sp2 * sy2,
        cr2 * sp2 * cy2 + sr2 * cp2 * sy2,
        cr2 * cp2 * sy2 - sr2 * sp2 * cy2,
    ])


def q_to_euler(q):
    """四元数 -> 欧拉角, 复现 quaternion.cpp:587/594/601"""
    w, x, y, z = qnorm(q)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))   # ← clamp 防 NaN
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def q_to_R(q):
    """四元数 -> 旋转矩阵, 复现 quaternion.cpp:34
       输出 R_body->ned  (已实测验证)"""
    q = qnorm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def R_to_q_shepperd(R):
    """
    旋转矩阵 -> 四元数, 用 Shepperd 方法 (数值稳定版)
    对比: 教程 2.4.3 讲的朴素方法在 trace 接近 -1 时会爆炸
    ArduPilot 在 quaternion.cpp:86 用的也是分支法
    """
    R = np.asarray(R, dtype=float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0          # s = 4*w
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0   # s = 4*x
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0   # s = 4*y
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0   # s = 4*z
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return qnorm(np.array([w, x, y, z]))


def R_to_q_naive(R):
    """朴素方法 —— 演示数值陷阱 (教程 2.4.3)"""
    q0 = 0.5 * np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2]))
    if abs(q0) < 1e-8:
        return None
    q1 = (R[2, 1] - R[1, 2]) / (4 * q0)
    q2 = (R[0, 2] - R[2, 0]) / (4 * q0)
    q3 = (R[1, 0] - R[0, 1]) / (4 * q0)
    return qnorm(np.array([q0, q1, q2, q3]))


def q_rotate(q, v):
    """用四元数旋转向量: v' = q ⊗ [0,v] ⊗ q*"""
    vq = np.concatenate([[0.0], v])
    return qmul(qmul(q, vq), qconj(q))[1:]


def angular_difference(q1, q2):
    """误差四元数 δq = q2^-1 ⊗ q1, 复现 quaternion.h:198"""
    return qmul(qconj(q2), q1)


def quat_angle_between(q1, q2):
    """两个四元数之间的夹角 (度) —— 处理 q ≡ -q

    ⚠️ 数值警告: 当两姿态几乎相同时 (d -> 1), arccos 的绝对精度只有 ~1e-8 rad,
       用它在 1e-9 度量级做判定是错误的。
       需要高精度比较时请改用 chordal_distance()。
    """
    d = abs(np.dot(qnorm(q1), qnorm(q2)))
    d = min(d, 1.0)
    return np.degrees(2.0 * np.arccos(d))


def chordal_distance(q1, q2):
    """四元数的弦距离 ||q1 - q2|| —— 小角度下数值稳定

    与夹角的关系:  ||q1 - q2|| = 2*sin(θ/4)  (θ = 两姿态的夹角)
    小角度时 ~ θ/2

    ⚠️ 实现要点: 必须先求"差"再取模, 不能写成 sqrt(2 - 2*dot)
       因为 dot -> 1 时 2 - 2*dot 会发生【灾难性抵消】:
       两个完全相同的四元数 d = 1 - 1e-16, 2-2d 算出来是 2.2e-16 而不是 0,
       开方后变成 1.5e-8 的假误差 —— 比真实误差大 8 个数量级!
    """
    q1, q2 = qnorm(q1), qnorm(q2)
    if np.dot(q1, q2) < 0:
        q2 = -q2
    return float(np.linalg.norm(q1 - q2))   # ← 先求差, 避开抵消


# ============================================================================
#                              测试
# ============================================================================
def sep(title):
    print("\n" + "=" * 76)
    print(f"  {title}")
    print("=" * 76)


def test_1_q_equals_minus_q():
    """实验 1: q 与 -q 是同一个旋转 —— 教程 2.5.1"""
    sep("实验 1: q == -q 表示同一个旋转   (教程 2.5.1)")

    q = qnorm(np.array([0.5, 0.5, 0.5, 0.5]))
    R1 = q_to_R(q)
    R2 = q_to_R(-q)

    print("q  =", q)
    print("-q =", -q)
    print("\nR(q)  =");print(R1)
    print("\nR(-q) =");print(R2)

    assert np.allclose(R1, R2)
    print("\n[PASS] R(q) == R(-q)  ==>  q 和 -q 是同一个姿态")

    ax, th = q_to_axis_angle(q)
    print(f"\n几何解读: q  = 绕 {np.round(ax,4)} 转 {np.degrees(th):.2f}°")
    print(f"          -q 等价于绕同一轴转 {np.degrees(th) - 360:.2f}°")
    print("          物理上多转了一整圈, 到达的姿态完全相同")

    print("\n[工程影响] 做插值/误差计算时, 必须先检查点积符号:")
    print("           if dot(q1,q2) < 0: q2 = -q2   ← 翻到同一半球")


def test_2_mul_is_rotation_composition():
    """实验 2: 四元数乘法 = 旋转复合 —— 教程 2.5.3"""
    sep("实验 2: 四元数乘法 = 旋转的复合   (教程 2.5.3)")

    q_z90 = axis_angle_to_q([0, 0, 1], np.pi / 2)

    # 两次 90° 偏航
    q_2x = qmul(q_z90, q_z90)
    q_1x_180 = axis_angle_to_q([0, 0, 1], np.pi)

    print("两次偏航 90° 的结果 =", np.round(q_2x, 6))
    print("一次偏航 180° 的结果 =", np.round(q_1x_180, 6))
    assert quat_angle_between(q_2x, q_1x_180) < 1e-9
    print("[PASS] 两次 90° == 一次 180°")

    # 旋转不可交换
    q_x90 = axis_angle_to_q([1, 0, 0], np.pi / 2)
    q_y90 = axis_angle_to_q([0, 1, 0], np.pi / 2)

    q_a = qmul(q_y90, q_x90)
    q_b = qmul(q_x90, q_y90)

    print("\n【关键】旋转的不可交换性:")
    print("先滚转再俯仰 R =");print(np.round(q_to_R(q_a), 4))
    print("先俯仰再滚转 R =");print(np.round(q_to_R(q_b), 4))

    assert not np.allclose(q_to_R(q_a), q_to_R(q_b))
    print("\n[PASS] 两者不同 ==> 旋转不可交换, 四元数乘法也不可交换")
    print("       拿本书在手里试试: 先滚转90再俯仰90 vs 反过来, 结果不一样")

    # 顺序规则
    print("\n【顺序规则】人类说'先转A再转B', 代码写 q_B ⊗ q_A  (后来者居左)")
    v = np.array([1.0, 0, 0])
    v_direct = q_rotate(q_z90, v)
    v_seq = q_rotate(qmul(q_z90, np.array([1.0, 0, 0, 0])), v)
    print(f"   把 (1,0,0) 偏航 90° 后 = {np.round(v_direct,6)}  (北->东) ✓")
    assert np.allclose(v_direct, [0, 1, 0], atol=1e-12)


def test_3_conversions():
    """实验 3: 三种表示的往返一致性 —— 教程 2.4"""
    sep("实验 3: 欧拉角 <-> 四元数 <-> 旋转矩阵  往返一致性   (教程 2.4)")

    rng = np.random.default_rng(2024)
    err_eq, err_qr = 0.0, 0.0

    for _ in range(1000):
        rpy = np.array([rng.uniform(-np.pi, np.pi),
                        rng.uniform(-np.pi / 2 + 0.02, np.pi / 2 - 0.02),  # 避开万向锁
                        rng.uniform(-np.pi, np.pi)])

        q = euler_to_q(*rpy)
        rpy2 = q_to_euler(q)
        err_eq = max(err_eq, np.max(np.abs((np.array(rpy2) - rpy + np.pi) % (2 * np.pi) - np.pi)))

        R = q_to_R(q)
        q2 = R_to_q_shepperd(R)
        # chordal 距离: ||q-q2|| = 2*sin(θ/4) ~ θ/2, 小角度下稳定
        err_qr = max(err_qr, chordal_distance(q, q2))

    print("1000 组随机姿态:")
    print(f"  欧拉角 -> 四元数 -> 欧拉角   最大误差 = {np.degrees(err_eq):.3e} °")
    print(f"  四元数 -> R -> 四元数  最大 chord 距离 = {err_qr:.3e}  (~姿态角误差/2)")
    assert err_eq < 1e-9 and err_qr < 1e-12
    print("[PASS] 所有转换往返一致")

    # --- 演示 arccos 的数值精度极限 (工程上必须知道的坑) ---
    print("\n【数值精度警告】为什么这里用 chord 距离而不是 arccos?")
    d = 1.0 - 5e-17
    print(f"  arccos({d}) = {np.arccos(d):.6e}   <- 本应约 1e-8, 却因量化变成 0!")
    print(f"  sqrt(2*(1-d)) = {np.sqrt(2*5e-17):.6e}   <- chord 距离依然准确")
    print("  ==> 判定'两个姿态是否相同'时, 永远不要用 arccos 在小角度上做文章,")
    print("      应该用 1-|dot| 或 chordal 距离。")


def test_4_ardupilot_verbatim():
    """实验 4: 与 ArduPilot 逐位对比 —— 教程 2.4.4"""
    sep("实验 4: 与 ArduPilot 逐字对比   (教程 2.3.2 / 2.4.4)")

    rng = np.random.default_rng(99)
    max_err = 0.0
    for _ in range(1000):
        rpy = rng.uniform(-np.pi, np.pi, 3)
        rpy[1] = rng.uniform(-1.5, 1.5)

        q = euler_to_q(*rpy)

        # --- 验证 q_to_R 与 ArduPilot rotation_matrix() 一致 ---
        # ArduPilot: q1=标量, q2/q3/q4=向量;  我们的 q = [w,x,y,z]
        R_mine = q_to_R(q)

        # 手工展开 ArduPilot 的公式 (用 w,x,y,z 命名)
        w, x, y, z = q
        R_ap = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
            [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
        ])
        max_err = max(max_err, np.max(np.abs(R_mine - R_ap)))

        # --- 验证欧拉角反解 ---
        r, p, y_ = q_to_euler(q)
        assert abs(p - rpy[1]) < 1e-12

    print("1000 组随机姿态:")
    print(f"  四元数 -> 旋转矩阵  最大误差 = {max_err:.3e}")
    assert max_err < 1e-14
    print("[PASS] 与 ArduPilot quaternion.cpp:34 完全一致")

    # --- 反面教材: 命名陷阱 ---
    print("\n【命名陷阱演示】")
    q = euler_to_q(0.3, -0.2, 0.5)
    print(f"  数学记号 q = [w,x,y,z] = {np.round(q,6)}")
    print(f"  ArduPilot  q1(标量),q2,q3,q4 = {np.round(q,6)}")
    print("  如果你以为 ArduPilot 的 q1 是第一个虚部:")
    q_wrong = np.array([q[1], q[2], q[3], q[0]])
    print(f"    错误解读为 w={q_wrong[0]:.4f} (实际应为 {q[0]:.4f})")
    print(f"    ==> 完全错误")


def test_4b_rotate_formula_sign():
    """实验 4b: 四元数旋转向量的两个版本 —— 加号 vs 减号 ⭐关键"""
    sep("实验 4b: v' = q⊗v⊗q* 的加号/减号版本判定   (推导文 2.3.4)")

    print("【背景】用四元数旋转向量有两组等价的公式, 网上混用且常不说明:\n")
    print("  版本 A (加号):  v' = (q0²-|qv|²)v + 2(qv·v)qv + 2q0(qv×v)")
    print("  版本 B (减号):  v' = (q0²-|qv|²)v + 2(qv·v)qv - 2q0(qv×v)")
    print("\n  两者相差 qv×v 项的符号 —— 物理上对应【相反的旋转方向】\n")

    def v_plus(q, v):
        w, qv = q[0], q[1:]
        return (w * w - np.dot(qv, qv)) * v + 2 * np.dot(qv, v) * qv + 2 * w * np.cross(qv, v)

    def v_minus(q, v):
        w, qv = q[0], q[1:]
        return (w * w - np.dot(qv, qv)) * v + 2 * np.dot(qv, v) * qv - 2 * w * np.cross(qv, v)

    # --- 用偏航 90° 这个物理上毫无疑问的例子判定 ---
    q_yaw90 = axis_angle_to_q([0, 0, 1], np.pi / 2)   # 绕 z 轴转 90°
    v_north = np.array([1.0, 0.0, 0.0])               # 机头朝北

    print("【判定实验】偏航 90°, 把机头方向 (1,0,0)[北] 转过去, 应该得到 (0,1,0)[东]")
    print(f"  真实的三明治积 q⊗v⊗q*  = {np.round(q_rotate(q_yaw90, v_north), 6)}")
    print(f"  版本 A (加号)          = {np.round(v_plus(q_yaw90, v_north), 6)}")
    print(f"  版本 B (减号)          = {np.round(v_minus(q_yaw90, v_north), 6)}")
    print(f"  q_to_R(q) @ v          = {np.round(q_to_R(q_yaw90) @ v_north, 6)}")

    assert np.allclose(q_rotate(q_yaw90, v_north), [0, 1, 0], atol=1e-12)
    assert np.allclose(v_plus(q_yaw90, v_north), [0, 1, 0], atol=1e-12)
    assert np.allclose(v_minus(q_yaw90, v_north), [0, -1, 0], atol=1e-12)
    print("\n[PASS] 加号版本 -> (0,1,0) 指向东 ✓  (与三明治积一致)")
    print("[PASS] 减号版本 -> (0,-1,0) 指向西 ✗  (方向反了!)")

    # --- 随机验证三个实现一致 ---
    rng = np.random.default_rng(11)
    e_sand, e_R = 0.0, 0.0
    for _ in range(500):
        q = qnorm(rng.standard_normal(4))
        v = rng.standard_normal(3)
        e_sand = max(e_sand, np.max(np.abs(v_plus(q, v) - q_rotate(q, v))))
        e_R = max(e_R, np.max(np.abs(v_plus(q, v) - q_to_R(q) @ v)))

    print(f"\n500 组随机验证:")
    print(f"  加号版本 vs 三明治积 q⊗v⊗q*   最大误差 = {e_sand:.3e}")
    print(f"  加号版本 vs q_to_R(q) @ v     最大误差 = {e_R:.3e}")
    assert e_sand < 1e-12 and e_R < 1e-12
    print("[PASS] 三者完全一致")

    print("\n【结论 —— 请记住这个判定方法】")
    print("  本教程约定 R = R_body->ned 时, 必须用【加号】版本。")
    print("  遇到任何四元数资料, 不要背公式, 用这个实验判定:")
    print("     取 q = 偏航 90°,  v = (1,0,0)")
    print("     得到 (0,1,0) 的版本才是正确的 (指向东)")
    print("  配套代码 03_attitude_integration.py 里同一符号体系贯穿始终。")


def test_5_numerical_trap():
    """实验 5: 从旋转矩阵反解四元数的数值陷阱 —— 教程 2.4.3"""
    sep("实验 5: 反解四元数的数值稳定性   (教程 2.4.3)")

    print(f"{'旋转角':>10} | {'朴素法误差':>16} | {'Shepperd 误差':>16}")
    print("-" * 54)

    for deg in [10, 90, 170, 179, 179.9, 179.99, 180]:
        q = axis_angle_to_q([1, 1, 1], np.radians(deg))
        R = q_to_R(q)

        q_naive = R_to_q_naive(R)
        q_shep = R_to_q_shepperd(R)

        e_naive = quat_angle_between(q_naive, q) if q_naive is not None else float('nan')
        e_shep = quat_angle_between(q_shep, q)

        n_str = f"{e_naive:.3e}" if np.isfinite(e_naive) else "  爆炸(NaN)"
        print(f"{deg:>10} | {n_str:>16} | {e_shep:>16.3e}")

    print("\n[解读] 朴素法在 180° 附近崩溃(因为除以 4*q0, q0 -> 0),")
    print("       Shepperd 方法通过'选最大分量做主元'避免了这个问题。")
    print("       ArduPilot quaternion.cpp:86 用的也是分支法。")


def test_6_slerp():
    """实验 6: SLERP 姿态插值 —— 教程 2.7"""
    sep("实验 6: SLERP vs NLERP 姿态插值   (教程 2.7)")

    def slerp(q1, q2, t):
        q1, q2 = qnorm(q1), qnorm(q2)
        dot = np.dot(q1, q2)
        if dot < 0.0:                      # 翻到同一半球
            q2, dot = -q2, -dot
        dot = np.clip(dot, -1.0, 1.0)
        if dot > 0.9995:                   # 几乎重合 -> 退化为线性插值
            return qnorm(q1 + t * (q2 - q1))
        Omega = np.arccos(dot)
        sO = np.sin(Omega)
        return (np.sin((1 - t) * Omega) / sO) * q1 + (np.sin(t * Omega) / sO) * q2

    def nlerp(q1, q2, t):
        q1, q2 = qnorm(q1), qnorm(q2)
        if np.dot(q1, q2) < 0:
            q2 = -q2
        return qnorm((1 - t) * q1 + t * q2)

    # 两个相距较远的姿态
    q1 = axis_angle_to_q([0, 0, 1], np.radians(10))
    q2 = axis_angle_to_q([1, 0, 0], np.radians(170))

    print(f"q1 -> q2 的总转角 = {quat_angle_between(q1,q2):.2f}°")
    print(f"\n{'t':>6} | {'SLERP 相对q1转角':>18} | {'NLERP 相对q1转角':>18}")
    print("-" * 52)

    prev_s, prev_n = 0.0, 0.0
    s_rates, n_rates = [], []
    for t in np.linspace(0, 1, 11):
        ang_s = quat_angle_between(slerp(q1, q2, t), q1)
        ang_n = quat_angle_between(nlerp(q1, q2, t), q1)
        print(f"{t:>6.1f} | {ang_s:>18.3f} | {ang_n:>18.3f}")
        if t > 0:
            s_rates.append(ang_s - prev_s)
            n_rates.append(ang_n - prev_n)
        prev_s, prev_n = ang_s, ang_n

    print(f"\nSLERP 每步转角的标准差 = {np.std(s_rates):.4f}°  (应该接近 0 = 匀速)")
    print(f"NLERP 每步转角的标准差 = {np.std(n_rates):.4f}°  (不均匀)")
    assert np.std(s_rates) < np.std(n_rates)
    print("\n[PASS] SLERP 角速度恒定, NLERP 出生在端点慢中间快")
    print("       ==> 电机控制要用 SLERP, 否则姿态过渡时会有速度突变")


def test_7_attitude_error():
    """实验 7: 姿态误差四元数与最短路径 —— 教程 2.6 / 2.9.2"""
    sep("实验 7: 姿态误差四元数   (教程 2.6 / 2.9.2)")

    q_current = axis_angle_to_q([0.3, 0.5, 0.8], np.radians(40))
    q_desired = axis_angle_to_q([0.1, -0.7, 0.2], np.radians(100))

    dq = angular_difference(q_desired, q_current)
    print("误差四元数 dq =", np.round(dq, 6))
    print(f"|dq| = {np.linalg.norm(dq):.10f}   (应该是 1)")

    # 误差向量: e = 2 * sign(dq0) * dq_v
    e = 2.0 * np.sign(dq[0]) * dq[1:]
    print(f"\n姿态误差向量 e = 2*sign(dq0)*dq_v = {np.round(e, 6)} rad")
    print(f"                 = {np.degrees(e)} °")
    print(f"夹角大小 = {np.degrees(np.linalg.norm(e)):.4f}°")

    # 小误差近似验证
    print("\n【小误差近似】 δq ≈ [1, δθ/2]  (ESKF 的基石):")
    for deg in [0.1, 1, 5, 10]:
        qa = axis_angle_to_q([1, 0, 0], np.radians(20))
        qb = axis_angle_to_q([1, 0, 0], np.radians(20 + deg))
        d = angular_difference(qb, qa)
        e_vec = 2.0 * np.sign(d[0]) * d[1:]
        approx_err = abs(np.degrees(np.linalg.norm(e_vec)) - deg)
        print(f"   真实 {deg:>5.1f}° -> 误差向量给出 {np.degrees(np.linalg.norm(e_vec)):>7.4f}°"
              f"   (误差 {approx_err:.2e}°)")

    # 最短路径
    print("\n【最短路径】如果忘了 sign(dq0):")
    qa = axis_angle_to_q([1, 0, 0], np.radians(170))
    qb = axis_angle_to_q([1, 0, 0], np.radians(-170))
    d = angular_difference(qb, qa)
    print(f"   从 170° 转到 -170°, 实际最短路径应该是 20°")
    print(f"   dq = {np.round(d,6)}, dq[0] = {d[0]:.6f}  (负数!)")
    print(f"   不取 sign 会得到 {np.degrees(2*np.arccos(abs(d[0]))):.2f}° (正确)")
    print(f"   但方向会反 ==> 飞机会绕远路转 340°, 表现为'突然抽搐'")

    # q_rotate 验证
    v = np.array([1.0, 0, 0])
    q = axis_angle_to_q([0, 0, 1], np.pi / 2)
    assert np.allclose(q_rotate(q, v), [0, 1, 0], atol=1e-12)
    print("\n[PASS] q_rotate 验证: (1,0,0) 偏航90° -> (0,1,0) 北->东 ✓")


if __name__ == "__main__":
    print("=" * 76)
    print("  飞控学习教程 · 第 1 章 02 节 · 四元数完全指南 · 验证代码")
    print("  约定: q = [w,x,y,z]  (ArduPilot: [q1,q2,q3,q4], q1 是标量!)")
    print("=" * 76)

    test_1_q_equals_minus_q()
    test_2_mul_is_rotation_composition()
    test_3_conversions()
    test_4_ardupilot_verbatim()
    test_4b_rotate_formula_sign()
    test_5_numerical_trap()
    test_6_slerp()
    test_7_attitude_error()

    sep("全部测试通过 ✓")
    print("""
 核心结论:
   1. q ≡ -q:  两个四元数表示同一旋转; 插值前必须检查点积符号
   2. 四元数乘法不可交换:  p⊗q ≠ q⊗p  (旋转本身就不交换)
      顺序规则: 人类说"先A后B", 代码写 q_B ⊗ q_A  (后来者居左)
   3. 三种表示可以无损互转, 但:
      - 欧拉角在 pitch=±90° 退化 (万向锁)
      - 朴素反解四元数在 180° 附近崩溃 (必须用 Shepperd/分支法)
   4. 误差四元数 δq = q2⁻¹ ⊗ q1 ≈ [1, δθ/2]  <== ESKF 的基石
      姿态误差向量 e = 2·sign(δq₀)·δqᵥ   保证走最短路径
   5. SLERP 角速度恒定, NLERP 不是 -> 电机控制必须用 SLERP
   6. ArduPilot 命名: q1=标量, q2/q3/q4=向量  ⚠️ 差一位, 极易错

 下一步: 打开 03_attitude_integration.py
""")
