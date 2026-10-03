"""
============================================================================
 飞控学习教程 · 第 1 章 01 节 · 坐标系与旋转表示
 配套验证代码
============================================================================

 运行:  python 01_coordinate_frames.py
 依赖:  numpy

 本文件验证教程 1.4 ~ 1.6 节的所有公式。

【本教程统一约定】
     R = R_body->ned   表示   v_ned = R @ v_body
     R 的第 0/1/2 列 = 机体 X/Y/Z 轴在 NED 下的方向

 教程引用 (ArduPilot 4.5.7):
   libraries/AP_Math/quaternion.cpp:34   rotation_matrix()
   libraries/AP_Math/quaternion.cpp:421  from_euler()
   libraries/AP_Math/quaternion.cpp:587  get_euler_roll()
   libraries/AP_Math/quaternion.cpp:594  get_euler_pitch()
   libraries/AP_Math/quaternion.cpp:601  get_euler_yaw()
   libraries/AP_Math/matrix3.cpp:141     Matrix3::operator*(Vector3)
============================================================================
"""
import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=120)


# ============================================================================
#  【1】 NED / ENU 互转  —— 教程 1.2.3 节
# ============================================================================
def ned_to_enu(v_ned):
    """NED -> ENU: 交换 X/Y，Z 取反"""
    v_ned = np.asarray(v_ned)
    return np.array([v_ned[1], v_ned[0], -v_ned[2]])


def enu_to_ned(v_enu):
    """ENU -> NED: 同一个操作（该变换矩阵是对合的，即自身的逆）"""
    v_enu = np.asarray(v_enu)
    return np.array([v_enu[1], v_enu[0], -v_enu[2]])


# ============================================================================
#  【2】 欧拉角 -> 旋转矩阵 (ZYX)  —— 教程 1.4.2 节
# ============================================================================
def euler_to_R(phi, theta, psi):
    """
    ZYX 顺序的  R_body->ned    (先偏航 ψ -> 再俯仰 θ -> 最后滚转 φ)

    物理含义: R 的【列】= 机体三轴在 NED 下的方向
              v_ned = R @ v_body

    自检: 偏航 90° 时 R[:,0] 应等于 (0,1,0)  = 机头指向东
    """
    c_p, s_p = np.cos(phi), np.sin(phi)
    c_t, s_t = np.cos(theta), np.sin(theta)
    c_y, s_y = np.cos(psi), np.sin(psi)

    return np.array([
        [c_t * c_y, s_p * s_t * c_y - c_p * s_y, c_p * s_t * c_y + s_p * s_y],
        [c_t * s_y, s_p * s_t * s_y + c_p * c_y, c_p * s_t * s_y - s_p * c_y],
        [-s_t,      s_p * c_t,                   c_p * c_t],
    ])


def R_to_euler(R):
    """从 R_body->ned 反解欧拉角。教程 1.4.4"""
    theta = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    phi = np.arctan2(R[2, 1], R[2, 2])
    psi = np.arctan2(R[1, 0], R[0, 0])
    return phi, theta, psi


# ============================================================================
#  【3】 四元数 -> 旋转矩阵 (逐字复现 ArduPilot)  —— 教程 1.4.3 节
# ============================================================================
def quat_to_R_ardupilot(q1, q2, q3, q4):
    """
    逐字复现 ArduPilot quaternion.cpp:34 的 rotation_matrix()

    ⚠️ ArduPilot 命名约定:
         q1 = 标量(w)   q2/q3/q4 = 向量部分(x/y/z)
       对应数学记号:
         q0            q1/q2/q3

    实测验证: 输出为 R_body->ned (不是它的转置!)
    """
    q3q3, q3q4 = q3 * q3, q3 * q4
    q2q2, q2q3, q2q4 = q2 * q2, q2 * q3, q2 * q4
    q1q2, q1q3, q1q4 = q1 * q2, q1 * q3, q1 * q4
    q4q4 = q4 * q4

    return np.array([
        [1 - 2 * (q3q3 + q4q4), 2 * (q2q3 - q1q4),     2 * (q2q4 + q1q3)],
        [2 * (q2q3 + q1q4),     1 - 2 * (q2q2 + q4q4), 2 * (q3q4 - q1q2)],
        [2 * (q2q4 - q1q3),     2 * (q3q4 + q1q2),     1 - 2 * (q2q2 + q3q3)],
    ])


def euler_to_quat(phi, theta, psi):
    """欧拉角 -> 四元数, 逐字复现 ArduPilot quaternion.cpp:421
       返回 [w, x, y, z]  (即 ArduPilot 的 [q1,q2,q3,q4])"""
    cr2, cp2, cy2 = np.cos(phi * 0.5), np.cos(theta * 0.5), np.cos(psi * 0.5)
    sr2, sp2, sy2 = np.sin(phi * 0.5), np.sin(theta * 0.5), np.sin(psi * 0.5)

    q1 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2
    q2 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2
    q3 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2
    q4 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2
    return np.array([q1, q2, q3, q4])


# ============================================================================
#                              测试开始
# ============================================================================
def sep(title):
    print("\n" + "=" * 76)
    print(f"  {title}")
    print("=" * 76)


def test_0_convention():
    """【最重要】实测确定 ArduPilot 的旋转矩阵方向 —— 教程 1.4.3"""
    sep("实验 0: 实测 ArduPilot rotation_matrix() 的方向约定   (教程 1.4.3)")

    # 纯偏航 90°: q = [cos45, 0, 0, sin45]
    c = np.cos(np.pi / 4)
    q = [c, 0.0, 0.0, c]
    R = quat_to_R_ardupilot(*q)

    print("纯偏航 90° 的四元数 q = [cos45, 0, 0, sin45]")
    print("\nrotation_matrix() 输出 R =")
    print(R)

    col0 = R[:, 0]
    print(f"\nR 的第 0 列 = {col0}   <-- 这是机体系 X 轴(机头)在 NED 下的方向")
    print("机头原本朝北, 偏航 90 度后应该朝【东】= (0, 1, 0)")

    assert np.allclose(col0, [0, 1, 0], atol=1e-12), "约定判断错误!"
    print("\n[PASS] 第 0 列 = (0,1,0) = 指向东")
    print("       ==> ArduPilot 的 rotation_matrix() 输出 R_body->ned")
    print(f"\n验证: R[:,1] (机体Y轴) = {R[:,1]}   应为 (-1,0,0) = 指向西")
    assert np.allclose(R[:, 1], [-1, 0, 0], atol=1e-12)
    print("      R[:,2] (机体Z轴) =", R[:, 2], "  应为 (0,0,1) = 指向下 (水平时)")
    print("\n[结论] 本教程统一约定: R = R_body->ned,  v_ned = R @ v_body")


def test_1_orthogonality():
    """实验 1: 正交性验证 —— 教程 1.6 实验 1"""
    sep("实验 1: 旋转矩阵正交性   (教程 1.6)")

    R = euler_to_R(np.pi / 2, 0, 0)          # 右滚 90°
    print("R(右滚90°) =\n", R)
    print("(第 1 列 = 机体 Y 轴在 NED 下的方向: 右滚 90° 后指向下)")

    assert np.allclose(R @ R.T, np.eye(3)), "不是正交矩阵!"
    assert np.allclose(np.linalg.det(R), 1.0), "行列式不是 +1!"
    print("\n[PASS] R @ R.T == I       (正交性)")
    print("[PASS] det(R) == +1       (真旋转, 非镜像)")


def test_2_ardupilot_match():
    """实验 2: 复现 ArduPilot rotation_matrix —— 教程 1.6 实验 2"""
    sep("实验 2: 与 ArduPilot rotation_matrix() 对比   (教程 1.6)")

    rng = np.random.default_rng(42)
    max_err = 0.0
    for _ in range(300):
        rpy = rng.uniform(-np.pi, np.pi, 3)
        rpy[1] = rng.uniform(-np.pi / 2 + 0.01, np.pi / 2 - 0.01)  # 避开万向锁

        q = euler_to_quat(*rpy)                    # [w,x,y,z]
        # ⚠️ ArduPilot 的 q1=标量, 所以按 [w,x,y,z] 顺序传即可
        R_ap = quat_to_R_ardupilot(q[0], q[1], q[2], q[3])
        R_std = euler_to_R(*rpy)

        max_err = max(max_err, np.max(np.abs(R_ap - R_std)))

    print("300 组随机姿态: ArduPilot rotation_matrix()  vs  标准 ZYX 公式")
    print(f"最大逐元素误差 = {max_err:.3e}")
    assert max_err < 1e-12, "不匹配!"
    print("[PASS] 完全一致 ==> ArduPilot 用的就是标准 ZYX 公式")

    # ---- 反面教材: 参数顺序传错的后果 ----
    print("\n【反面教材】如果误以为 ArduPilot 的 q1 是第一个虚部(它其实是标量):")
    rpy = np.array([0.3, -0.2, 0.5])
    q = euler_to_quat(*rpy)
    R_wrong = quat_to_R_ardupilot(q[1], q[2], q[3], q[0])   # 故意传错顺序
    R_right = quat_to_R_ardupilot(q[0], q[1], q[2], q[3])
    print(f"           最大误差 = {np.max(np.abs(R_wrong - R_right)):.4f}"
          f"   <== 完全错误, 飞机必然失控")
    print("           这就是为什么必须记住 ArduPilot 的四元数命名约定。")


def test_3_gimbal_lock():
    """实验 3: 亲手体验万向锁 —— 教程 1.6 实验 3"""
    sep("实验 3: 万向锁 (Gimbal Lock)   (教程 1.4.5)")

    phi_true, psi_true = 30.0, 60.0
    print(f"固定真值: roll={phi_true}°, yaw={psi_true}°")
    print(f"\n{'pitch(°)':>10} | {'反解roll(°)':>13} {'反解pitch(°)':>14} {'反解yaw(°)':>12} {'roll+yaw':>10}")
    print("-" * 70)

    for theta_deg in [0, 45, 89, 89.999, 90]:
        R = euler_to_R(np.radians(phi_true), np.radians(theta_deg), np.radians(psi_true))
        phi2, theta2, psi2 = R_to_euler(R)
        p, t, y = np.degrees(phi2), np.degrees(theta2), np.degrees(psi2)
        print(f"{theta_deg:>10} | {p:>13.3f} {t:>14.3f} {y:>12.3f} {p + y:>10.3f}")

    print("\n[解读] pitch -> 90 度时, 反解的 roll/yaw 与真值(30/60)严重偏离,")
    print("       但 roll+yaw 之和保持 ~90 度不变  ==>  两个自由度退化成一个")
    print("       这就是万向锁。四元数没有这个问题。")


def test_4_ned_enu():
    """实验 4: NED <-> ENU —— 教程 1.6 实验 4"""
    sep("实验 4: NED <-> ENU 转换   (教程 1.2.3)")

    v_ned = np.array([10.0, 5.0, -3.0])
    print(f"NED (北,东,地) = {v_ned}")
    v_enu = ned_to_enu(v_ned)
    print(f"ENU (东,北,天) = {v_enu}")
    print("解读: 北10->东10, 东5->北5, 地-3(即上升3米)->天+3")

    assert np.allclose(enu_to_ned(ned_to_enu(v_ned)), v_ned)
    print("[PASS] 往返转换一致 (该矩阵是对合的)")


def test_5_roundtrip():
    """实验 5: 欧拉角 <-> 旋转矩阵 往返一致性"""
    sep("实验 5: 欧拉角 <-> R 往返一致性   (教程 1.9 练习 1-4)")

    print(f"{'pitch(°)':>10} | {'最大往返误差 (rad)':>22}")
    print("-" * 40)

    rng = np.random.default_rng(7)
    for theta_deg in [0, 30, 60, 85, 89, 89.9]:
        errs = []
        for _ in range(500):
            phi = rng.uniform(-np.pi, np.pi)
            psi = rng.uniform(-np.pi, np.pi)
            theta = np.radians(theta_deg)

            R = euler_to_R(phi, theta, psi)
            phi2, theta2, psi2 = R_to_euler(R)
            R2 = euler_to_R(phi2, theta2, psi2)
            errs.append(np.max(np.abs(R - R2)))

        print(f"{theta_deg:>10} | {max(errs):>22.3e}")

    print("\n[解读] 旋转矩阵层面的往返始终精确(误差~1e-16),")
    print("       但欧拉角本身在 pitch->90 时不稳定 —— 万向锁的本质。")


def test_6_small_angle():
    """实验 6: 小角度近似 —— 教程 1.5.5"""
    sep("实验 6: 小角度近似 R ~= I + [w]x   (教程 1.5.5)")

    print(f"{'角度(°)':>10} | {'||R_exact - R_approx||':>24}")
    print("-" * 42)

    for deg in [0.1, 1, 5, 10, 30]:
        th = np.radians(deg)
        axis = np.array([1, 1, 1]) / np.sqrt(3)
        w = axis * th

        q = np.concatenate([[np.cos(th / 2)], axis * np.sin(th / 2)])
        R_exact = quat_to_R_ardupilot(*q)

        wx = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        R_approx = np.eye(3) + wx

        print(f"{deg:>10} | {np.max(np.abs(R_exact - R_approx)):>24.3e}")

    print("\n[解读] 10 度以内误差 < 1%, 这是 EKF 线性化的基础 (第 4 章)")


def test_7_right_hand():
    """验证 NED 是右手系 —— 教程 1.2.1"""
    sep("补充: 验证 NED / FRD 是右手系   (教程 1.2.1)")

    N, E, D = np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])
    cross = np.cross(N, E)
    print(f"北 x 东 = {cross}   (应该等于 地 = {D})")
    assert np.allclose(cross, D)
    print("[PASS] NED 是右手系: 北 x 东 = 地")
    print("\n右手定则记忆: 拇指=北, 食指=东, 中指自然朝下 ==> Z 轴朝下")


if __name__ == "__main__":
    print("=" * 76)
    print("  飞控学习教程 · 第 1 章 01 节 · 坐标系与旋转表示 · 验证代码")
    print("  统一约定: R = R_body->ned,  v_ned = R @ v_body")
    print("=" * 76)

    test_0_convention()
    test_1_orthogonality()
    test_2_ardupilot_match()
    test_3_gimbal_lock()
    test_4_ned_enu()
    test_5_roundtrip()
    test_6_small_angle()
    test_7_right_hand()

    sep("全部测试通过 ✓")
    print("""
 核心结论:
   1. NED = 北东地, Z 朝下; FRD = 前右下, Z 朝下; 都是右手系
   2. 本教程约定 R = R_body->ned, 即 v_ned = R @ v_body
      R 的列 = 机体三轴在 NED 下的方向
   3. ZYX = 先偏航 -> 再俯仰 -> 最后滚转
   4. R 是正交矩阵: R^-1 = R^T,  det(R) = +1
   5. ArduPilot rotation_matrix() 输出 R_body->ned (已实测), 逐字复现误差 1e-16
   6. 万向锁: pitch = +-90 度时欧拉角退化 -> 必须用四元数
   7. NED <-> ENU: 交换 X/Y, Z 取反

 自检口诀: 偏航 90 度后, R 的第 0 列(机头方向)应该指向【东】

 下一步: 打开 02_quaternion.py
""")
