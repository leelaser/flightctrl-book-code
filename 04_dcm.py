"""
============================================================================
 飞控学习教程 · 第 3.6 周 · DCM 路线：旋转矩阵姿态解算
 配套验证代码
============================================================================

 运行:  python 04_dcm.py
 依赖:  numpy（画图需要 matplotlib，没装也能跑）

 本脚本的 8 个实验，全部在本课正文里被引用：

   exp_1_orthogonality_drift()       纯陀螺积分的正交性漂移（本课动机）
   exp_2_orthonormalization()        正交归一化三家族 + 速查表步骤 6 核实
   exp_3_poisson_sign()              泊松方程符号 + ★「退化测试点」「循环论证」两个陷阱
   exp_4_accel_sign_convention()     加速度计符号 + 叉积顺序（速查表核实第一案）
   exp_5_pitch_rate_earth()          地球系角速度 vs 欧拉角速率（读者原创推导核实）
   exp_6_complementary_filter_6dof() 完整 6DOF DCM 互补滤波
   exp_7_complementary_filter_9dof() 加磁力计：航向锁定对比
   exp_8_dcm_vs_quaternion()         DCM vs 四元数：精度 + 计算量

 教程引用（ArduPilot Copter 4.5.7，已逐个打开核对）:
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:148   matrix_update()
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:158   _dcm_matrix.rotate(...)        <- 右乘
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:220   reset(): pitch = atan2f(x, norm(y,z))
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:222   reset(): roll = atan2f(-y, -z)
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:274   renorm()
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:277   注释：不用泰勒展开，sqrt() 只要 44 微秒
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:296   const float renorm_val = 1.0f / a.length();
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:326   normalize()
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:332   error = a * b            (eq.18)
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:334   t0 = a - b*(0.5*error)   (eq.19)
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:335   t1 = b - a*(0.5*error)   (eq.19)
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:336   t2 = t0 % t1             (eq.20)
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:677   _ra_sum[i] += accel_ef * deltat
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:683   accel_ef = _dcm_matrix * (delta_velocity/dt)
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:775   Vector3f GA_e(0.0f, 0.0f, -1.0f);   <- 指向「天」
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:829   GA_b[i].normalize()
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:834   error[i] = GA_b[i] % GA_e
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:895   error[besti] = _dcm_matrix.mul_transpose(...)
   libraries/AP_AHRS/AP_AHRS_DCM.cpp:918   _omega_P = error[besti] * _P_gain * _kp
   libraries/AP_Math/matrix3.cpp:115       Matrix3<T>::rotate()
   libraries/AP_Math/matrix3.cpp:128       Matrix3<T>::normalize()

 教程引用（ArduCopter 2.9.1，读者原创推导的那个函数在这里）:
   libraries/AP_AHRS/AP_AHRS.cpp:70        get_pitch_rate_earth()
   libraries/AP_AHRS/AP_AHRS.cpp:77        get_roll_rate_earth()
   libraries/AP_AHRS/AP_AHRS.h:82-83       两个函数的声明
   libraries/APM_Control/AP_PitchController.cpp:44   调用点
   libraries/APM_Control/AP_RollController.cpp:38    调用点

 教程引用（MatrixPilot 4.0）:
   libDCM/rmat.c:250                       rupdate()
   libDCM/rmat.c:265                       theta = omega * ggain
   libDCM/rmat.c:284                       rup[1] = -theta[2]      <- I + [theta x]
   libDCM/rmat.c:292                       MatrixMultiply(... rmat, rup)   <- 右乘
   libDCM/rmat.c:297                       normalize()
   libDCM/rmat.c:310                       error = -VectorDotProduct(...)
   libDCM/rmat.c:323                       VectorPower + 泰勒展开
   libDCM/rmat.c:341                       roll_pitch_drift()：gplane x rmat[6]
   libDCM/rmat.c:780                       dcm_run_imu_step()
   libDCM/rmat.h:29-31                     6 constraints on the 9 elements
   libDCM/rmat.h:33-36                     1/2 藏在 2.14 定点标度里
============================================================================
"""
import time

import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=140)

# 全局约定：本课一律用 NED（与第 1 章 01/02/03 一致，也与 ArduPilot 一致）
#   C = R_{机体系 -> NED}，行 = {北, 东, 地} 在机体系下的坐标
#   天顶向量 K_B = -C[2,:]      （"天" = -"地"）
#   北向向量 I_B =  C[0,:]
#   西向向量 J_B = -C[1,:]      （西 = -东）


# ============================================================================
#  0. 打印小工具
# ============================================================================
def head(title):
    print()
    print("=" * 78)
    print("  " + title)
    print("=" * 78)


def sub(title):
    print()
    print("-" * 78)
    print("  " + title)
    print("-" * 78)


def table(rows, headers, widths=None):
    """极简中文表格打印。rows 里的元素会被 str() 化。"""
    rows = [[str(c) for c in r] for r in rows]
    ncol = len(headers)
    if widths is None:
        widths = [max(len(headers[i]), max((len(r[i]) for r in rows), default=0))
                  for i in range(ncol)]
    line = "  ".join(headers[i].ljust(widths[i]) for i in range(ncol))
    print("  " + line)
    print("  " + "-" * len(line))
    for r in rows:
        print("  " + "  ".join(r[i].ljust(widths[i]) for i in range(ncol)))


# ============================================================================
#  1. 基础数学工具
# ============================================================================
def skew(w):
    """反对称矩阵 [w x]，与 ArduPilot / PX4 的 skew 一致：
        [ 0   -wz   wy]
        [ wz   0   -wx]
        [-wy   wx   0 ]
    """
    w = np.asarray(w, dtype=float)
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def expm_so3(w):
    """SO(3) 指数映射 = 罗德里格斯公式（精确）。"""
    w = np.asarray(w, dtype=float)
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3) + skew(w)
    K = skew(w / th)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


def expm_so3_2nd(w):
    """二阶截断 I + [w x] + [w x]^2/2。MatrixPilot 更进一步做了
       theta' = theta*(1+|theta|^2/3) 的非线性修正（rmat.c:271-278）。"""
    K = skew(w)
    return np.eye(3) + K + 0.5 * (K @ K)


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_to_R(roll, pitch, yaw):
    """3-2-1 (Z-Y-X) 机体系 -> NED，与 ArduPilot Matrix3::from_euler 一致。"""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def R_to_euler(R):
    """从机体系->NED 的 R 反解 roll/pitch/yaw（3-2-1）。"""
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def orth_err(C):
    """矩阵偏离正交归一的程度：max|C^T C - I|。"""
    return float(np.abs(C.T @ C - np.eye(3)).max())


def det_err(C):
    return float(abs(np.linalg.det(C)) - 1.0)


def geo_deg(C_est, C_true):
    """两个姿态之间的测地距离（度）。

    ⚠️ 注意：这个指标在「矩阵不严格正交」时会**饱和**。
    若 C_est = (1+ε)·R（整体缩放），trace(C_est^T C_true) ≈ 3(1+ε)，
    c = (trace-1)/2 ≈ 1 + 1.5ε，被 clip 到 1 后 arccos(1) = 0 —— 读出来是 0 度，
    比真实误差还小。所以对比「未正交归一化」的方案时，必须用 mat_err()。"""
    dR = C_est.T @ C_true
    c = (np.trace(dR) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def mat_err(C_est, C_true):
    """不饱和的姿态误差：max|C_est·C_trueᵀ − I|（正交时与 geo_deg 单调对应）。

    C_est·C_trueᵀ = I 当且仅当 C_est = C_true。缩放、非正交都会如实反映出来。"""
    return float(np.abs(C_est @ C_true.T - np.eye(3)).max())


def _orthonormalize_ned(C):
    """速查表步骤 6 的完整流程（NED 版）：
       对称正交化(北,西) -> 单位化 -> 天顶 = 北 x 西
       对应 ArduPilot AP_AHRS_DCM::normalize()（AP_AHRS_DCM.cpp:326）：
           error = a * b;  t0 = a - b*(0.5*error);  t1 = b - a*(0.5*error);
           t2 = t0 % t1;
       注意 ArduPilot 里 a=北、b=东、c=地，这里用 北/西 只是为了让 I x J = K
       的右手关系更直观，两者等价。"""
    I = C[0, :].copy()          # 北
    J = -C[1, :].copy()         # 西 = -东
    Err = float(np.dot(I, J)) / 2.0
    In = I - Err * J            # ★ 用原始的 I、J 算两个新值
    Jn = J - Err * I
    I = In / np.linalg.norm(In)
    J = Jn / np.linalg.norm(Jn)
    return np.array([I, -J, -np.cross(I, J)])   # 行 = 北、东、地


# ============================================================================
#  实验 1：纯陀螺积分的正交性漂移实测
# ============================================================================
def exp_1_orthogonality_drift():
    head("实验 1 · 纯陀螺积分的正交性漂移 —— 本课的动机")

    print("""
  做法：给一个恒定的机体系角速度，用一阶形式 C <- C + C*[w x]*dt 递推，
        看 C 偏离正交归一有多快。唯一的变量是「每步做完要不要按行归一化」。
        一阶形式等价于 C <- C * (I + [w dt x])，而
            det(I + [w dt x]) = 1 + |w dt|^2  > 1
        —— 每步都让行列式涨一点点，这就是漂移的**解析来源**。
""")

    w = np.array([0.30, -0.20, 0.50])       # rad/s，机体系角速度
    dt = 0.001                              # 1 kHz
    th = w * dt

    print("  角速度 w = [%.2f, %.2f, %.2f] rad/s,  dt = %.4f s" % (w[0], w[1], w[2], dt))
    print("  每步 |w*dt| = %.6e,  理论 det(I+[w dt x]) - 1 = |w dt|^2 = %.6e"
          % (np.linalg.norm(th), np.linalg.norm(th) ** 2))

    rows = []
    C_no = np.eye(3)        # (a) 什么都不做
    C_nm = np.eye(3)        # (b) 每步按行归一化
    C_on = np.eye(3)        # (c) 每步完整正交归一化
    for k in range(1, 1_000_001):
        C_no = C_no + C_no @ skew(th)
        C_nm = C_nm + C_nm @ skew(th)
        C_nm = C_nm / np.linalg.norm(C_nm, axis=1, keepdims=True)
        C_on = _orthonormalize_ned(C_on + C_on @ skew(th))
        if k in (1_000, 10_000, 100_000, 1_000_000):
            rows.append(["%d" % k, "%.1f" % (k * dt),
                         "%.3e" % orth_err(C_no), "%.3e" % det_err(C_no),
                         "%.3e" % orth_err(C_nm), "%.3e" % det_err(C_nm),
                         "%.3e" % orth_err(C_on), "%.3e" % det_err(C_on)])
    table(rows,
          ["步数", "时间/s", "(a) max|CᵀC-I|", "(a) |det|-1",
           "(b) max|CᵀC-I|", "(b) |det|-1", "(c) max|CᵀC-I|", "(c) |det|-1"],
          [9, 8, 15, 13, 15, 13, 15, 13])

    print("""
  ★ 三列的读法（(a) 什么都不做 / (b) 每步按行归一化 / (c) 每步完整正交归一化）：
    · 漂移是**确定性**的，不是随机游走 —— 每步 det 涨 |w dt|^2，线性累积。
      (a) 跑 1e6 步（1000 秒）后 |det|-1 到 4.6e-01，矩阵已经不是旋转矩阵了。
    · (b) 按行归一化后 |det|-1 从 4.6e-01 降到 1.7e-02（小了一个多量级），
      但 max|CᵀC-I| 只从 4.1e-01 降到 1.4e-01。**归一化压模长，压不住夹角。**
    · (c) 只有把对称正交化、单位化、叉积重建**全部做完**，
      两个指标才同时回到机器精度。
    ⇒ 这就是 DCM 路线存在的全部理由，也是本课的主线：
      **9 个数、6 个约束，其中「夹角」那 3 个约束不会自动满足。**
      而四元数只有 4 个数、1 个约束，「归一化」一步就把两件事一起办了
      —— 这是四元数最大的工程优势，第 8 节会用数字再确认一次。
""")


# ============================================================================
#  实验 2：正交归一化三家族 + 速查表步骤 6 核实
# ============================================================================
def ortho_symmetric(I, J, n_iter=1, do_normalize=False, err_scale=0.5):
    """速查表步骤 6-1/6-2：对称正交化（双向各修一半）+ 可选单位化。

    ★ 必须用**原始**的 I、J 去算两个新值（ArduPilot 的 t0/t1、MatrixPilot 的
      rbuff 都是这么写的）；写成 I = I - Err*J; J = J - Err*I 就变成了顺序更新，
      结果和 c³/4 的理论值对不上。

    err_scale 默认 0.5（= ArduPilot 显式写的 0.5f，也是 MatrixPilot 藏在
    2.14 定点标度里的那个 1/2）。实验 2.4 会把它改成 1.0 做对照 —— 提醒
    「这个 1/2 是推导出来的，不能省」。
    """
    I = np.array(I, dtype=float).copy()
    J = np.array(J, dtype=float).copy()
    for _ in range(n_iter):
        Err = float(np.dot(I, J)) * err_scale
        In = I - Err * J
        Jn = J - Err * I
        I, J = In, Jn
        if do_normalize:
            I = I / np.linalg.norm(I)
            J = J / np.linalg.norm(J)
    return I, J


def ortho_cross_only(I, J):
    """只单位化 + 叉积造 K，跳过对称正交化。"""
    I = np.array(I, dtype=float)
    J = np.array(J, dtype=float)
    I = I / np.linalg.norm(I)
    J = J / np.linalg.norm(J)
    return I, J, np.cross(I, J)


def ortho_gram_schmidt(I, J):
    """Gram-Schmidt：不对称，第一个向量完全不动，误差全推给第二个。"""
    I = np.array(I, dtype=float)
    I = I / np.linalg.norm(I)
    J = np.array(J, dtype=float)
    J = J - np.dot(J, I) * I
    J = J / np.linalg.norm(J)
    return I, J, np.cross(I, J)


def exp_2_orthonormalization():
    head("实验 2 · 正交归一化三家族 —— 速查表步骤 6 到底对不对")

    sub("2.1 对称正交化：一步之后**并不严格正交**，残差正好是 c^3/4")
    print("""
  速查表步骤 6-1 写的是（速查表.md:150-154，三个版本一致）：
      Err = (I·J)/2 ,  I' = I - Err*J ,  J' = J - Err*I
  把它代进 I'·J' 硬算（设 |I|=|J|=1, I·J = c）：
      I'·J' = (I - Err*J)·(J - Err*I)
            = c - Err*|I|^2 - Err*|J|^2 + Err^2*c
            = c - c/2 - c/2 + (c/2)^2*c
            = c^3/4              <-- 不是 0！
  ⇒ 一次对称正交化只做到**近似**正交，残差是 c 的三阶小量。
""")
    rows = []
    for c in (1e-1, 1e-2, 1e-3, 1e-4):
        I0 = np.array([1.0, 0.0, 0.0])
        J0 = np.array([c, np.sqrt(1 - c * c), 0.0])
        I1, J1 = ortho_symmetric(I0, J0, 1, do_normalize=False)
        rows.append(["%.0e" % c, "%.6e" % float(np.dot(I1, J1)),
                     "%.6e" % (c ** 3 / 4.0),
                     "%.10f" % np.linalg.norm(I1),
                     "%.10f" % np.sqrt(1.0 - 3 * c * c / 4)])
    table(rows, ["初始 I·J", "一步后 I·J", "理论 c³/4", "一步后 |I'|", "理论 √(1-3c²/4)"],
          [12, 16, 14, 18, 20])

    print("""
  ★ 两个结论一起读：
    ① 残差是 c³/4 —— 三次方下降。实际跑起来 c 本来就在 1e-3 量级，
       一步之后残差进 1e-10，**够用了**。所以速查表「只做一次」不是偷懒，
       而是这个公式本身的精度已经超过了单精度浮点的分辨率。
    ② 但它**同时缩短了矢量**：|I'| = √(1 - 3c²/4)，c=0.1 时就缩了 0.38%。
       所以步骤 6-2 的**单位化绝对不能省** —— 省了矩阵行列式会慢慢小于 1，
       姿态会「缩水」。
    ⇒ 速查表的顺序（正交化 -> 单位化 -> 叉积重建）是**对的**。
      这一条是本课核实下来速查表最扎实的部分，三个版本都没写错。
""")

    sub("2.2 迭代几次够？—— 两次就到 double 的机器精度")
    rows = []
    I0 = np.array([1.0, 0.0, 0.0])
    J0 = np.array([1e-2, np.sqrt(1 - 1e-4), 0.0])
    for n in range(0, 6):
        if n == 0:
            I, J = I0, J0
        else:
            I, J = ortho_symmetric(I0, J0, n_iter=n, do_normalize=True)
        rows.append(["%d" % n, "%.6e" % float(np.dot(I, J))])
    table(rows, ["对称正交化迭代次数", "I·J 残差"], [20, 18])
    print("""
  ★ 迭代收益在第 2 次就归零（残差到 IEEE double 的 eps 量级）。
    ⇒ 工程实现**都只做一次**：单精度下再做也没用了（float32 的 eps 是 1.2e-7，
      第一次迭代后的 ~c³/4 早就低于它了）。
    ⇒ ArduPilot 连论文里的**泰勒展开**都省了，直接算 1/|a|：
       AP_AHRS_DCM.cpp:296   const float renorm_val = 1.0f / a.length();
       AP_AHRS_DCM.cpp:277 的源码注释写得很清楚：sqrt() 只要 44 微秒，
       不值得为省这点时间引入额外的误差累积。
       MatrixPilot 则**用了**泰勒展开（rmat.c:323-325，(3-|r|²)/2）。
       两家做了相反的选择 —— 说明这不是对错问题，是工程取舍。
""")

    sub("2.3 ★ 陷阱：叉积重建**不能**替代正交化")
    print("""
  常见误解：「反正最后要用 K = I x J 重建第三轴，前面正交化可以跳过」。
  错。看数据：
""")
    c = 0.1
    I0 = np.array([1.0, 0.0, 0.0])
    J0 = np.array([c, np.sqrt(1 - c * c), 0.0])

    I, J, K = ortho_cross_only(I0, J0)
    C_cross = np.array([I, J, K])
    I2, J2 = ortho_symmetric(I0, J0, 1, do_normalize=True)
    C_sym = np.array([I2, J2, np.cross(I2, J2)])

    table([["跳过对称正交化，直接 单位化+叉积",
            "%.10f" % abs(np.linalg.det(C_cross)), "%.4e" % orth_err(C_cross)],
           ["对称正交化 -> 单位化 -> 叉积",
            "%.10f" % abs(np.linalg.det(C_sym)), "%.4e" % orth_err(C_sym)]],
          ["流程", "|det(C)|", "max|CᵀC - I|"], [34, 14, 16])
    print("""
  ★ 原因：K = I x J 只保证 K ⊥ I、K ⊥ J，**它管不了 I 和 J 之间的夹角**。
    I·J = 0.1 原封不动地留在矩阵里，于是 CᵀC 的非对角元就是 0.1，det 变成 0.99。
  ⇒ 正确顺序**只能**是「先把 I、J 摆正，再用叉积造 K」。这正是速查表把叉积
    放在步骤 6-3（最后）的原因 —— 这个顺序是有道理的。
""")

    sub("2.4 ★★ 源码核实：那个 1/2 是**必须**的，砍掉就失效")
    print("""
  对称正交化里 Err 前面那个 1/2，看着像「作者随手写的经验系数」。
  它不是。去掉它，正交化就**基本失效**。
  两家真实源码在这里的写法**不一样**，正好拿来对照：

    ArduPilot，AP_AHRS_DCM.cpp:332-335 —— 显式写出 0.5：
        const float error = _dcm_matrix.a * _dcm_matrix.b;              // :332
        const Vector3f t0 = _dcm_matrix.a - (_dcm_matrix.b * (0.5f*error)); // :334
        const Vector3f t1 = _dcm_matrix.b - (_dcm_matrix.a * (0.5f*error)); // :335

    MatrixPilot，rmat.c:310-319 —— 不写 0.5，注释说「1/2 已包含在 2.14 标度里」：
        error = -VectorDotProduct(3, &rmat[0], &rmat[3]); // note, 1/2 is built into 2.14
        VectorScale(3, &rbuff[0], &rmat[3], error);
        VectorScale(3, &rbuff[3], &rmat[0], error);
        VectorAdd(3, &rbuff[0], &rbuff[0], &rmat[0]);
        VectorAdd(3, &rbuff[3], &rbuff[3], &rmat[3]);

  两家都把 **1/2 算进去了**（一家显式、一家藏在定点标度里）——
  因为不这样做，残差不会降到三阶。数据：
""")
    print("""  ★ 那个 1/2 藏在哪？rmat.h:33-36 的头注释自己交代了：
        //  All numbers are stored in 2.14 format.
        //  Vector and matrix libraries work in 1.15 format.
        //  This combination allows values of matrix elements between -2 and +2.
        //  Multiplication produces results scaled by 1/2.
    「2.14 存、1.15 算」的接口本身就把每次乘法缩了一半 ——
    所以 rmat.c:310 只写 `error = -VectorDotProduct(...)`，1/2 自动到位。
    （⚠️ 我无法逐位追出这个 1/2 的确切落点：Microchip 的 MatrixMultiply /
      VectorScale 不在 MatrixPilot_4_0.zip 里。但 rmat.h 的注释直接承认了它存在，
      而 3.6.6.1 的推导证明「净效果必须是 1/2」，两者吻合。
      **移植定点代码到浮点时，这类「藏在标度里的常数」必须逐个核对** ——
      这是把 MatrixPilot 的 DCM 抄成浮点最常见的 bug 来源。）
""")
    rows = []
    for label, half in (("Err = (I·J)/2   （ArduPilot / 本课）", 0.5),
                        ("Err = I·J       （去掉 1/2）", 1.0)):
        for cval in (0.1, 0.01):
            i0 = np.array([1.0, 0.0, 0.0])
            j0 = np.array([cval, np.sqrt(1 - cval * cval), 0.0])
            i1, j1 = ortho_symmetric(i0, j0, 1, do_normalize=False,
                                     err_scale=half)
            rows.append([label, "%.2e" % cval, "%.4e" % abs(float(np.dot(i1, j1))),
                         "c^3/4 = %.2e" % (cval ** 3 / 4.0)])
    table(rows, ["Err 的系数", "I·J = c", "一步后 |I'·J'|", "理论值 c³/4"],
          [34, 10, 18, 18])
    print("""
  ★ 读法：
    · 取 1/2：残差 ≈ c³/4 —— 从 c 一步掉到 c³，**降了三阶**。
    · 不取 1/2：残差 ≈ |−c + c³| ≈ c —— **一点没降**，正交化白做。
      因为 I' = I - c·J 里，c 太大了，修正过头，反而留下 -c 量级的残留。
  ⚠️ 这是本课要你记住的第二个「指标陷阱」的反面：
    **一个看起来像经验系数的常数，往往是推导出来的、且不能省。**
    推导见 3.6.6 节：把 I'、J' 代回去令 I'·J' 的二阶项为零，就能解出那个 1/2。
""")

    sub("2.5 三家族横向对比（同样输入，同样只做一步）")
    I0 = np.array([1.0, 0.05, 0.02])
    J0 = np.array([0.10, 0.99, -0.03])
    I0 = I0 / np.linalg.norm(I0)
    J0 = J0 / np.linalg.norm(J0)
    c0 = float(np.dot(I0, J0))

    I_a, J_a = ortho_symmetric(I0, J0, 1, do_normalize=True)
    C_a = np.array([I_a, J_a, np.cross(I_a, J_a)])
    I_b, J_b, K_b = ortho_cross_only(I0, J0)
    C_b = np.array([I_b, J_b, K_b])
    I_c, J_c, K_c = ortho_gram_schmidt(I0, J0)
    C_c = np.array([I_c, J_c, K_c])

    table([["初始（未处理）", "%.6f" % c0,
            "%.4e" % orth_err(np.array([I0, J0, np.cross(I0, J0)]))],
           ["对称正交化（一步）", "%.6e" % float(np.dot(I_a, J_a)), "%.4e" % orth_err(C_a)],
           ["叉积重建（跳过正交化）", "%.6e" % float(np.dot(I_b, J_b)), "%.4e" % orth_err(C_b)],
           ["Gram-Schmidt", "%.6e" % float(np.dot(I_c, J_c)), "%.4e" % orth_err(C_c)]],
          ["方法", "处理后 I·J", "max|CᵀC - I|"], [26, 14, 16])

    print("""
  ★ 三种方法的性格：
    · 对称正交化：I、J 各承担一半修正 —— **不偏心**，残差 c³/4。
      这是速查表 / ArduPilot / MatrixPilot 三家**共同**的选择。
    · 叉积重建：只管第三轴，前两轴的夹角原样保留 —— 单独用不够，但**必须做**。
    · Gram-Schmidt：I 一动不动，误差全推给 J —— **偏心**。
      好处是计算量最小；坏处是长期跑会把误差都堆在 J 轴上。
      ⚠️ 所以飞控里几乎不用 Gram-Schmidt，除非你明确知道哪一轴更可信。
""")


# ============================================================================
#  实验 3：泊松方程符号 + 两个验证陷阱
# ============================================================================
def exp_3_poisson_sign():
    head("实验 3 · 泊松方程 dC/dt 的符号与左右乘 —— 附两个「验证陷阱」")

    sub("3.1 正确的真值基准：绕体轴转，就右乘")
    print("""
  验证一个微分方程写法，**真值基准必须独立于被测公式**。
  这里唯一干净的基准是几何事实本身：
      C(t+dt) = C(t) · exp([w_b x] dt)          （w_b 在机体系）
  另一条等价路径（用于交叉验证基准自身）：
      C(t+dt) = exp([w_e x] dt) · C(t)          （w_e = C w_b，在地球系）
  这两条必须给出**同一个矩阵**，否则基准就写错了。
""")

    rng = np.random.default_rng(7)
    C0 = expm_so3(rng.normal(size=3) * 1.2)          # ★ 非平凡初始姿态
    wb = np.array([0.3, -0.2, 0.5])
    dt = 1e-4
    we = C0 @ wb

    exact_body = C0 @ expm_so3(wb * dt)
    exact_earth = expm_so3(we * dt) @ C0
    print("  基准自洽性检查：两种构造的精确解之差 = %.3e   （应为机器精度）"
          % float(np.abs(exact_body - exact_earth).max()))
    print("  C0 = 随机旋转, det = %.15f, max|C0ᵀC0-I| = %.2e"
          % (np.linalg.det(C0), orth_err(C0)))

    cands = [
        ("dC =  C[w_b x]",   C0 @ skew(wb)),
        ("dC = +[w_b x]C",   skew(wb) @ C0),
        ("dC = -[w_b x]C",  -skew(wb) @ C0),
        ("dC =  C[w_e x]",   C0 @ skew(we)),
        ("dC = +[w_e x]C",   skew(we) @ C0),
        ("dC = -[w_e x]C",  -skew(we) @ C0),
    ]
    rows = []
    pair = {}
    for name, dC in cands:
        err = float(np.abs((C0 + dC * dt) - exact_body).max())
        pair[name] = err
        mark = "  <- 正确" if err < 1e-8 else ""
        rows.append([name, "%.6e" % err, mark])
    table(rows, ["写法", "一步后与精确解的偏差", ""], [18, 20, 10])

    print("""
  ★ 正确的盒子（本课主线公式）：
        dC/dt = C·[w_b x]  =  +[w_e x]·C        两个都是**正号**
    两者恒等，因为 [C a x] = C [a x] C^T（由 C(a×b) = Ca × Cb 得到），
    代入 w_e = C w_b：
        [w_e x] C = C [w_b x] C^T C = C [w_b x]
    ⇒ 所以这两行的偏差**逐位相同**（上面表里都是 %.6e）。
      这个「逐位相同」就是恒等式存在的指纹 —— 一个公式和它的等价形式给出
      完全一样的浮点结果，说明它们本来就是同一个表达式。
  ★ 负号那一支确实存在，但它属于**转置矩阵**：
        d/dt(C^T) = -[w_b x]·C^T
    这正是 Starlino 教程里 DCM^B = (DCM^G)^T 那一支。
    ⚠️ 看到 dC/dt = -[w x] C，先问一句：「这个 C 是 机体系<-地球系 还是 地球系<-机体系？」
""" % pair["dC =  C[w_b x]"])

    sub("3.2 ★ 陷阱一：C0 = I 是退化测试点，区分不出任何东西")
    rows = []
    I3 = np.eye(3)
    for name, dC in [("C[w_b x]", I3 @ skew(wb)),
                     ("+[w_e x]C", skew(wb) @ I3),
                     ("-[w_e x]C", -skew(wb) @ I3)]:
        exact_I = expm_so3(wb * dt)
        rows.append([name, "%.6e" % float(np.abs((I3 + dC * dt) - exact_I).max())])
    table(rows, ["写法（C0 = I）", "一步偏差"], [18, 16])
    print("""
  ★ 前两行的偏差**逐位相同**：C0 = I 时 C[w_b x] 与 +[w_e x]C 是同一个矩阵
    （都等于 [w_b x]），**左右乘的区别被完全掩盖**。
    ⇒ 验证「左乘/右乘」「加号/减号」这类约定时，**绝对不能用单位阵当测试点**。
      那是最容易通过、也最没有信息量的点。
      本课的所有姿态积分实验，初始姿态一律取**随机非单位**旋转。
""")

    sub("3.3 ★ 陷阱二：循环论证 —— 拿自己的公式造真值，再用它证明自己")
    print("""
  这是一个比「符号写反」更高层的错，而且它**极具欺骗性**。

  演示：假设某人「觉得」正确写法是 dC = -[w_e x]C，于是他把基准写成
        C_fake(t+dt) = exp(-[w_e x] dt) · C0
  这个基准**本身带着同一个负号**，再拿去检验：
""")
    fake_exact = expm_so3(-we * dt) @ C0
    rows = []
    for name, dC in [("dC = -[w_e x]C", -skew(we) @ C0),
                     ("dC = +[w_e x]C", skew(we) @ C0),
                     ("dC =  C[w_b x]", C0 @ skew(wb))]:
        rows.append([name, "%.6e" % float(np.abs((C0 + dC * dt) - fake_exact).max())])
    table(rows, ["写法（基准 = exp(-[w_e x]dt)·C0）", "与「假基准」的偏差"], [32, 20])
    print("""
  ★ 看第一行：负号写法「完美通过」（1e-09 量级），而且**数字干净得不像有问题**。
    这不是验证，是**循环论证**：公式和基准用的是同一个错误约定，
    误差根本不会产生 —— 两个错误互相抵消了。

  ⇒ 纪律：**真值基准必须独立于被测公式。**
    验证 dC/dt 的写法，基准只能来自「绕体轴转就右乘」这个几何事实，
    或者来自源码实现，**绝不能来自另一个同样是我写的矩阵指数**。

  💡 自己动手做一遍：把 C0 换成随机旋转、两种基准各跑一次，
     看同一个公式在两种基准下分别被判成「对」还是「错」。
     亲手撞一次循环论证，比读十遍「要注意独立性」管用。

  ★ 附带结论：这就是为什么本课不敢用「我自己写的 DCM 积分」去验证
    「我自己写的 DCM 积分」。实验 1/2/3 的真值全部只来自两样东西：
      (a) 几何事实 C(t+dt) = C·exp([w x]dt)
      (b) ArduPilot / MatrixPilot 的源码原文
""")

    sub("3.4 ★ 行向量是「余矢量」：同一个机体旋转，行和列转的方向相反")
    print("""
  这是所有 DCM 符号事故的总根源，值得单独说清楚。

  设 C = 机体系->NED。取其中的一个**行** r = C[2,:]（地球的「地」方向在机体系
  下的坐标），同时取一个**列** c = C[:,2]（机体的 z 轴在 NED 下的坐标）。
  这两个是同一个矩阵里的两批数字，但物理含义完全不同。下面让机体绕机体系
  +x 轴转 0.1 rad，看它们各自怎么变：
""")
    th = 0.1
    C0 = np.eye(3)
    C1 = C0 @ expm_so3([th, 0, 0])          # 绕机体 +x 转
    rows = [
        ["行 r = C[2,:]（地球的「地」）",
         "(%.4f, %.4f, %.4f)" % tuple(C0[2, :]),
         "(%.4f, %.4f, %.4f)" % tuple(C1[2, :]),
         "Δy = %+.4f" % (C1[2, 1] - C0[2, 1])],
        ["列 c = C[:,2]（机体的 z 轴）",
         "(%.4f, %.4f, %.4f)" % tuple(C0[:, 2]),
         "(%.4f, %.4f, %.4f)" % tuple(C1[:, 2]),
         "Δy = %+.4f" % (C1[1, 2] - C0[1, 2])],
    ]
    table(rows, ["对象", "转动前", "转动后", "变化"], [26, 20, 20, 14])

    inc_row = C1[2, :] - C0[2, :]
    pred_row = np.cross(C0[2, :], np.array([th, 0, 0]))
    inc_col = C1[:, 2] - C0[:, 2]
    pred_col = np.cross(np.array([th, 0, 0]), C0[:, 2])
    print("    行的增量 = r × dθ ?  实测 (%.4f,%.4f,%.4f) vs 预测 (%.4f,%.4f,%.4f)"
          % (inc_row[0], inc_row[1], inc_row[2], pred_row[0], pred_row[1], pred_row[2]))
    print("    列的增量 = dθ × c ?  实测 (%.4f,%.4f,%.4f) vs 预测 (%.4f,%.4f,%.4f)"
          % (inc_col[0], inc_col[1], inc_col[2], pred_col[0], pred_col[1], pred_col[2]))
    print("""
  ★ 两条更新公式（本课后面反复用，必须记牢）：
        行：  r_new = r + r × dθ        （右乘 C <- C·(I+[dθ x]) 的效果）
        列：  c_new = c + dθ × c        （同一个旋转，列是同向的）
    因为 r × dθ = -(dθ × r)，所以**同一个 dθ，行和列的变化方向恰好相反**。
    这就是「为什么叉积顺序会写反」的全部原因：你在纸上推的时候用的是列
    （物理课上的矢量），而代码里更新的往往是行。

  ★ 由此直接推出加计校正的**叉积顺序**：
    设 r 是估计的地球方向（行），m 是对应的测量方向（单位矢量）。
    要让 r 朝 m 靠，需要 r × dθ ∝ m - r(r·m)。两边左叉 r：
        r × (r × dθ) = dθ(r·r) - r(r·dθ) = dθ - r(r·dθ)  ⇒  dθ ∝ -(r × m) = m × r
    ⇒ 加计/磁力计的等效角位移一律写成
        dθ = 测量方向 × 估计方向        （m × r，不是 r × m！）
    下一节用源码和数字同时验证这条。
""")


# ============================================================================
#  实验 4：加速度计符号 + 叉积顺序 —— 速查表核实第一案
# ============================================================================
def _simple_correction(accel_sign, cross_order, steps=2000, gain=0.01):
    """初值故意错 30 度，真值水平，纯加计校正（无陀螺）。

       accel_sign:  +1 -> K_B1A = +Normalize(A)   （比力约定）
                    -1 -> K_B1A = -Normalize(A)   （重力矢量约定，速查表写法）
       cross_order: 'm x r' -> dθ = K_B1A x K_B0 （本课结论）
                    'r x m' -> dθ = K_B0 x K_B1A （速查表写法）
    """
    C = euler_to_R(np.radians(30.0), 0.0, 0.0)     # 估计（错 30 度）
    A_body = np.array([0.0, 0.0, -1.0])            # 真值水平，读数指向天
    hist = []
    for k in range(steps):
        K_B0 = -C[2, :].copy()                     # 估计的天顶向量
        K_B1A = accel_sign * A_body / np.linalg.norm(A_body)
        if cross_order == "m x r":
            dtheta_a = np.cross(K_B1A, K_B0)
        else:
            dtheta_a = np.cross(K_B0, K_B1A)
        # 只取垂直于天顶的分量（加计测不到绕天顶的旋转）
        dtheta_a = dtheta_a - np.dot(dtheta_a, K_B0) * K_B0
        C = C @ expm_so3_2nd(gain * dtheta_a)
        # 正交归一化（速查表步骤 6）
        I = C[0, :].copy()
        J = -C[1, :].copy()
        Err = float(np.dot(I, J)) / 2.0
        In = I - Err * J
        Jn = J - Err * I
        I = In / np.linalg.norm(In)
        J = Jn / np.linalg.norm(Jn)
        C = np.array([I, -J, -np.cross(I, J)])
        if k % 100 == 0:
            hist.append(np.degrees(R_to_euler(C)[0]))
    return C, hist


def exp_4_accel_sign_convention():
    head("实验 4 · 加速度计符号 + 叉积顺序 —— 速查表核实第一案")

    sub("4.1 物理事实：MEMS 加速度计测的是**比力** f = a - g")
    print("""
  · 静止时 a = 0，所以 f = -g —— 比力指向**天**（与重力加速度方向相反）。
  · 实测佐证：ADXL345 / MPU6050 的 +Z 轴朝上时，Z 轴输出 +1g。
    也就是「哪根轴朝天，哪根轴读 +g」。
  · 所以若 A 是**原始加速度计读数**，则
        K_B（天顶向量，机体系）= +Normalize(A)          ← 没有负号
    只有当 A 被定义成「重力矢量读数」（指向地）时，才需要 K_B = -Normalize(A)。

  ⚠️ 速查表三个版本都写 K_B = -Normalize(A)，同时又都自称 K_B 是「天顶向量」。
     这两句话**不能同时成立**：
       · 若 A 是原始读数（指向天），则 -A 指向**地**，
         所以那个 K_B 是**地心向量（nadir）**，不是天顶向量；
       · 要让「天顶」这个称呼成立，A 必须先被定义成「指向地的重力矢量」，
         但速查表没有这个定义。
     这是**命名与内容不符**，不是算错 —— 数值上它和步骤 3 的叉积顺序
     互相绑定，整套照抄能跑（见 4.3、4.4）。
     但名字会误导人：谁按「天顶」这个名字把它搬到矩阵写法里，就会翻车。
""")

    sub("4.2 用 ArduPilot 源码判案（两条独立证据 + 一条旁证）")
    print("""
  证据一 —— AP_AHRS_DCM.cpp:775，漂移校正的参考矢量：
        Vector3f GA_e(0.0f, 0.0f, -1.0f);
      NED 里 z 轴指向**地**，所以 -1 表示这个矢量指向**天**。
      而 AP_AHRS_DCM.cpp:683 / :677 把加速度计读数**不取负**地转到地球系：
        accel_ef = _dcm_matrix * (delta_velocity / delta_velocity_dt);   (:683)
        _ra_sum[i] += accel_ef * deltat;                                 (:677)
      然后在 :829 归一化、:834 与参考矢量**直接叉积**：
        error[i] = GA_b[i] % GA_e;                                       (:834)
      两者同向（都指天），叉积才为 0。若读数取了负号，这里方向会整体反转。

  证据二 —— AP_AHRS_DCM.cpp:220-222，用加速度计做**姿态初始化**：
        pitch = atan2f(initAccVec.x, norm(initAccVec.y, initAccVec.z));   (:220)
        roll  = atan2f(-initAccVec.y, -initAccVec.z);                     (:222)
      水平静止时这两行必须输出 roll = pitch = 0。
      代入 roll = atan2(-0, -initAccVec.z)：**只有** initAccVec.z ≈ -9.81 才得 0。
      若读数真的指向地（+9.81），这里会算出 atan2(0, -9.81) = π = 180 度
      —— 飞机一起飞就倒扣。

  证据三 —— MatrixPilot 的 rmat.c，**另一条完全独立的技术路线**
  （Premerlani & Bizard 的 DCM IMU 论文系，与 ArduPilot 无渊源）：

  （a）坐标约定，rmat.c:66-67：
        // The columns of rmat are the axis vectors of the plane,
        // as measured in the earth reference frame.
      「rmat 的列 = 机体轴在地球系下的坐标」
      ⇒ 等价于「行 = 地球轴在机体系下的坐标」= 本课的 C。
      这一句把 MatrixPilot 和本课的约定**钉死在一起**了。

  （b）陀螺更新，rmat.c:284-289（rup 就是增量矩阵 I + [θ×]）：
        rup[1] = -theta[2];   rup[2] =  theta[1];
        rup[3] =  theta[2];   rup[5] = -theta[0];
        rup[6] = -theta[1];   rup[7] =  theta[0];
      按行展开正是反对称矩阵
            [ 0   -θz   θy ]
            [ θz   0   -θx ]
            [-θy   θx   0  ]
      即 +[θ×]，且对角线是 RMAX(=1.0)。
      配合 rmat.c:292 的 MatrixMultiply(..., rmat, rup)（右乘），
      得到的就是 **C ← C·(I + [θ×])，θ = ω·dt，全是正号**。
      ⇒ 与本课推导、与 ArduPilot 完全一致。

  （c）定点实现里还有一个二阶非线性修正，rmat.c:265-282：
        thetaSquare = θx² + θy² + θz²;
        nonlinearAdjust = RMAX + (thetaSquare >> 14) / 3;
      这是把 θ 乘上 (1 + |θ|²/3)，用来补偿大角速率下的二阶误差
      （对应第 3 周讲的圆锥误差/二阶项）。**一阶近似在高速旋转时会掉精度，
      真机代码里是显式补掉的** —— 这一点很多人不知道。

  ★ 证据一、二（ArduPilot）和证据三（MatrixPilot）是两条**独立血统**的实现，
    却在同一个符号上完全一致：**加速度计读数指向天，C ← C·(I + [θ×]) 全正号**。
    两条独立实现互相印证，比任何单一出处都可靠。
""")

    sub("4.3 数值判决：两次「只改一个变量」的对照实验")
    print("""
  真值 roll = 0，初值故意给 30 度，纯加计校正（无陀螺）。
  注意这里是**单变量对照**：第一次只改叉积顺序，第二次只改加计符号。
""")

    def _verdict(sgn, order):
        Cend, hist = _simple_correction(sgn, order)
        r_end = abs(hist[-1])
        ok = "✅ 收敛到 0" if r_end < 1.0 else "❌ 被推向 180°"
        return hist[0], hist[-1], ok

    print("  对照一：固定 K_B1A = +Normalize(A)，只改叉积顺序")
    rows = []
    for order, olabel in (("m x r", "K_B1A × K_B0  （测量 × 估计）"),
                          ("r x m", "K_B0 × K_B1A  （速查表步骤 3）")):
        r0, r1, ok = _verdict(+1.0, order)
        rows.append([olabel, "%.2f" % r0, "%.2f" % r1, ok])
    table(rows, ["dθ_a 写法", "起始 roll/度", "2000 步后 roll/度", "结果"],
          [30, 12, 16, 14])

    print("  对照二：固定叉积顺序为「测量 × 估计」，只改加计符号")
    rows = []
    for sgn, slabel in ((+1.0, "K_B1A = +Normalize(A)  （本课 / 源码）"),
                        (-1.0, "K_B1A = -Normalize(A)  （速查表步骤 2）")):
        r0, r1, ok = _verdict(sgn, "m x r")
        rows.append([slabel, "%.2f" % r0, "%.2f" % r1, ok])
    table(rows, ["加速度计约定", "起始 roll/度", "2000 步后 roll/度", "结果"],
          [30, 12, 16, 14])

    print("""
  ★ 每个对照里都**只有一个变量**，另一行必然发散。
    两个符号（加计约定、叉积顺序）写反任何一个，校正就变成**推离**而不是拉近，
    姿态会朝 180 度跑过去。现象很好认：
    **飞机静止放着，姿态角自己慢慢翻过去。**

  ⚠️⚠️ 但这个 bug 有一个**非常阴险**的性质，必须点明：
    把「加计符号」和「叉积顺序」**同时**取反，得到的角位移完全不变：
        (-m) × (-r) = m × r
    也就是说，(+Normalize(A), 测量×估计) 和 (-Normalize(A), 估计×测量)
    在数值上是**同一套算法**。
    ⇒ 所以「跑通了」根本不能证明你的符号约定和别人的一致！
      要判断哪一套真的对，**只能回到物理或源码**（4.1/4.2 两节）。
      这也是本课把源码核实放在数值实验**之前**的原因。

  ★ 那速查表到底是哪一套？只靠上面两个对照**判不出来** ——
    因为它用的是**矢量更新** K ← K + dθ × K，而不是矩阵更新 C ← C·(I + [dθ×])，
    这两套里 dθ 的符号约定**正好相反**。
    把「更新形式」当第三个变量加进来，才能给出判决。见 4.4。
""")

    sub("4.4 ★★ 判决：把速查表**字面**实现一遍（三变量全试）")
    print("""
  三个可以独立翻转的变量：
    ① 更新形式  矢量 K ← K + dθ × K      vs  矩阵 C ← C·(I + [dθ×])
       —— 注意这两者**不等价**：由 r ← r + r × dθ（行向量恒等式）可知
          K + dθ × K = K - K × dθ，也就是「反着转 dθ」。
          所以同一个 dθ 在两套里的物理含义差一个负号。
    ② 加计符号  K_B1A = ±Normalize(A)
    ③ 叉积顺序  dθ_a = K_B0 × K_B1A  vs  K_B1A × K_B0

  注：①的数值实现里，位姿状态始终是一个完整的旋转矩阵 C；
     「矢量更新」指的是**修正量的施加方式**（K = K + dθ×K 用在向量上，
      再重建成矩阵），「矩阵更新」指的是直接右乘 C·(I+[dθ×])。
""")
    _sign_matrix_table()

    print("""
  ★★ 三条结论，每一条都是这次跑出来的，不是推的：

    ① **加计符号和状态符号永远互相抵消**。
       第 A 行和第 E 行数字**一字不差**（1.2587），第 D 行和第 H 行也是
       （0.0127）。因为把 KB 和 KB1A 同时取负，(m,r) → (-m,-r)，
       叉积 (m × r) 不变、K + dθ × K 里的 K 也整体取负 —— 整条链路不变。
       ⇒ 这就是为什么速查表的负号「看起来没错」：
         **它和状态向量的符号是绑定的，光看这一行永远发现不了。**
       ⇒ 也说明「加计符号」根本不是独立变量，真正的独立变量只有
         更新形式 ① 和叉积顺序 ③。

    ② 真正的判决在 ①③ 的相对符号上。
       · D ≡ H（更新形式与叉积顺序**配套**）：稳态误差 **0.0127 度**。
       · A ≡ E（速查表字面：矢量更新 + 估计×测量）：稳态误差 **1.2587 度**，
         大了 **100 倍**。
       · B/F 和 C/G（只改一个）：直接发散到 180 度附近。
       为什么 A 行没有发散、只是精度差？因为**两个错误互相掩盖**：
       陀螺项和加计项的相对符号被整体翻了一次，
       陀螺在「往反方向推」，加计在「往回拉」，两者形成一个**对抗**的平衡，
       最后被加计拉住 —— 稳态能收敛，但动态性能损失殆尽。

    ③ 最干净的判决证据是**纯陀螺**（把加计那一路完全关掉）——
       没有「加计把它拉回来」的掩盖效应。下表由 _gyro_only_test() 现场算出：
""")
    g = _gyro_only_test()
    table([["矢量 K ← K + dθ×K（速查表步骤 1 字面）",
            "%.4f" % g["vec"][0], "%.4f" % g["vec"][1], "%.4f" % g["vec"][2]],
           ["矩阵 C ← C·(I+[dθ×])（正确）",
            "%.4f" % g["mat"][0], "%.4f" % g["mat"][1], "%.4f" % g["mat"][2]]],
          ["修正量施加方式", "1 秒后误差/度", "10 秒后误差/度", "1000 秒后/度"],
          [38, 15, 15, 15])
    print("""  角速率 |ω| = 0.1118 rad/s，1 秒真实转过 6.4 度。

  ★ 读法：矢量形式是在「往反方向转」，所以误差不是「不增长」，
    而是**以两倍速率增长** —— 真值转 +6.4 度、估计转 -6.4 度，
    差距 12.8 度 = 2 × 6.4。10 秒后 128 度，再往后就撞到 180 度上限来回摆
    （两个单位向量的夹角最大就是 180 度）。
    矩阵形式 1 秒、10 秒都是 0.0000 度，1000 秒后才 0.0007 度（浮点累积）。
    ⇒ 一个字面照抄速查表步骤 1 的人，**纯陀螺积分根本跑不起来**。
      而它在「陀螺 + 加计」一起跑时又能收敛 —— 这就是它藏了三个版本的原因。

    ④ 最后把角速率从小到大扫一遍，看这个错误**怎么随速率变化**：
""")
    _rate_sweep_table()
    print("""  ★ 误差与角速率**成正比**，比值稳定在 100 倍左右（5 rad/s 时超过 160 倍）。
    这就是「符号/约定写反」这类 bug 的**指纹**：
      · 不像噪声那样随机抖动
      · 不像陀螺零偏那样只随时间增长
      · 而是 **飞得越猛、错得越多**（悬停时看不出来，一做大机动就现形）
    ⚠️ 排查姿态问题时，「误差随机动幅度增大」几乎是符号约定的铁证，
       而不是滤波器参数没调好 —— 参数问题只会表现为滞后或抖动，
       不会随角速率线性放大。

  ⚠️⚠️ 所以本课对速查表的最终核实结论，必须分成「文档问题」和「数值问题」两层：

  【第一层：文档问题 —— 三版都有，且最致命】
     三份文档都**没写明 dθ 用的是哪一套符号约定**：
       · 是「矢量更新」K ← K + dθ × K（dθ 表示使 K 按右手转过去的角度）
       · 还是「矩阵右乘」C ← C·(I + [dθ×])（dθ 表示机体自身的转动）
     这两套里同一个符号 dθ 的物理含义**差一个负号**（4.4 开头已推）。
     不写明约定，读者就无法校验，必须靠「跑起来对不对」去猜 ——
     而 4.3 已经证明「跑起来对不对」分辨不出符号约定。

  【第二层：数值问题 —— 一条真错，一条「约定依赖」】
     ① 步骤 1  K_B1G = K_B0 + dθ_g × K_B0      -> ★ 真错
        它用的是矢量更新形式，却配了「陀螺角位移 = +ω dt」。
        在矢量更新约定下，正确的陀螺项应当是 -ω dt。
        后果：表行 A（= 速查表字面）误差 1.2587 度，是本课写法（0.0127 度）
        的 **100 倍**，且随角速率线性放大；纯陀螺使用时直接发散。
     ② 步骤 2  K_B1A = -Normalize(A)            -> 约定依赖（不是错，但误导）
        如果 A 是原始比力读数（指向天），那么 -A 指向**地**。
        所以这个 K_B 是**地心向量**，不是文档里写的「天顶向量」。
        数值上它和步骤 3 的叉积顺序**互相绑定**（表行 A ≡ E），
        只要整套照抄就不会错 —— 但**命名与内容矛盾**，
        一旦读者按「天顶」这个名字迁移到矩阵写法，立刻出错。
     ③ 步骤 3  dθ_a = K_B0 × (K_B1A - K_B0)     -> 约定依赖
        在上面的矢量约定下，这是**对**的；但换成矩阵右乘写法就正好反了
        （表行 C/G 发散）。ArduPilot 用矩阵写法，所以它的对应式是
        error = GA_b % GA_e，即「测量 × 估计」（AP_AHRS_DCM.cpp:834）。

  ★ 一句话总结：**速查表没有「算错」，它是「约定没说清 + 步骤 1 差了符号」**。
    而步骤 1 那个符号，在常规互补滤波里被加计拉住了、看不出来，
    只有做纯陀螺积分或大机动时才会现形 —— 这就是它躲过三个版本的原因。
    ⚠️ 这也是本课要你带走的方法论：
       看见任何公式，先问「它用的是哪套约定」，再问「对不对」。
       不写约定的公式，等于没写。
""")

    sub("4.5 三个速查表版本对照")
    print("""
  速查表（无后缀）.md:86, :121   ->  K_B = -Normalize(A)
  速查表1.md:106, :140           ->  K_B = -Normalize(A)      （一字不差）
  速查表2.docx 同名位置           ->  K_B = -Normalize(A)      （同样带负号）
  三版步骤 3 也都是：dθ_a = K_B0 × (K_B1A − K_B0)             （= K_B0 × K_B1A）

  ★ 三版都错，而且错在同一个地方 —— 说明它**光靠看笔记发现不了**：
    三份文档互相对照只会互相印证。必须拿物理（比力 = a - g）
    或源码（AP_AHRS_DCM.cpp:775 / :220-222）去对。
    这正是本课把「核实」放在公式推导之前的原因。
""")


# ============================================================================
#  实验 5：地球系角速度 vs 欧拉角速率 —— 读者原创推导核实
# ============================================================================
def _sign_convention_run(vec_form, neg_accel, order, n=4000, dt=0.002,
                         sa=0.02, sg=1.0):
    """只跟踪「天顶/地心」这一个向量，把三种约定组合跑一遍。

    vec_form:  True -> K ← K + dθ × K（速查表的矢量更新）
               False-> K ← K + K × dθ（等价于矩阵右乘 C ← C·(I+[dθ×])）
    neg_accel: True -> K_B1A = -Normalize(A)（速查表）   False -> +Normalize(A)（本课）
    order:     'r x m' -> dθ_a = K_B × (K_B1A-K_B)（速查表）
               'm x r' -> dθ_a = (K_B1A-K_B) × K_B（本课）
    返回 (轨迹, 稳态误差/度, 最大误差/度)。误差按「该向量本应代表的方向」算。
    """
    C = np.eye(3)                                  # 只用来生成真值
    A = C.T @ np.array([0.0, 0.0, -1.0])           # 比力读数（指天）
    KB = (-A if neg_accel else A)
    KB = KB / np.linalg.norm(KB)
    errs = []
    for k in range(n):
        t = k * dt
        w = np.array([0.10, 0.05 * np.sin(0.2 * t), 0.0])
        C = C @ expm_so3(w * dt)
        A = C.T @ np.array([0.0, 0.0, -1.0])

        dth_g = w * dt
        if vec_form:
            KBg = KB + np.cross(dth_g, KB)
        else:
            KBg = KB + np.cross(KB, dth_g)
        KB1A = (-A if neg_accel else A)
        KB1A = KB1A / np.linalg.norm(KB1A)
        d = KB1A - KBg
        dth_a = np.cross(KBg, d) if order == "r x m" else np.cross(d, KBg)
        dth = (sa * dth_a + sg * dth_g) / (sa + sg)
        KB = KB + (np.cross(dth, KB) if vec_form else np.cross(KB, dth))
        KB = KB / np.linalg.norm(KB)

        tgt = C[2, :] if neg_accel else -C[2, :]    # 「地」或「天」
        errs.append(np.degrees(np.arccos(np.clip(np.dot(KB, tgt), -1.0, 1.0))))
    e = np.array(errs)
    return e


def _rate_sweep_table():
    """把角速率从小到大扫一遍，看两套约定的误差怎么随速率变化。

    「误差与角速率成正比」是**符号写反**这类 bug 的指纹：
    它不像噪声那样随机、也不像零偏那样只随时间增长，
    而是**飞得越猛错得越多**。"""
    rows = []
    for amp in (0.02, 0.10, 0.50, 2.00, 5.00):
        a = []
        h = []
        for vec, neg, order, sink in ((True, True, "r x m", a),
                                      (False, False, "m x r", h)):
            C = np.eye(3)
            A = C.T @ np.array([0.0, 0.0, -1.0])
            KB = (-A if neg else A)
            KB = KB / np.linalg.norm(KB)
            errs = []
            for k in range(4000):
                t = k * 0.002
                w = np.array([amp, 0.05 * np.sin(0.2 * t), 0.0])
                C = C @ expm_so3(w * 0.002)
                A = C.T @ np.array([0.0, 0.0, -1.0])
                KBg = KB + (np.cross(w * 0.002, KB) if vec
                            else np.cross(KB, w * 0.002))
                M = (-A if neg else A)
                M = M / np.linalg.norm(M)
                d = M - KBg
                da = np.cross(KBg, d) if order == "r x m" else np.cross(d, KBg)
                dth = (0.02 * da + 1.0 * (w * 0.002)) / 1.02
                KB = KB + (np.cross(dth, KB) if vec else np.cross(KB, dth))
                KB = KB / np.linalg.norm(KB)
                tgt = C[2, :] if neg else -C[2, :]
                errs.append(np.degrees(np.arccos(
                    np.clip(np.dot(KB, tgt), -1.0, 1.0))))
            sink.append(np.mean(errs[-800:]))
        rows.append(["%.2f" % amp, "%.4f" % a[0], "%.4f" % h[0],
                     "%.0f x" % (a[0] / max(h[0], 1e-12))])
    table(rows, ["角速率幅值/rad·s⁻¹", "速查表字面写法 稳态误差/度",
                 "本课写法 稳态误差/度", "倍数"], [18, 22, 20, 10])


def _gyro_only_test(n=200000, dt=0.005):
    """纯陀螺：把加计那一路完全关掉，只留修正量的施加方式这一个变量。
    这一条最干净 —— 没有「加计把它拉回来」的掩盖效应。"""
    w = np.array([0.10, 0.05, 0.0])
    out = {}
    for mode in ("vec", "mat"):
        C = np.eye(3)
        K = C[2, :].copy()                 # 状态取「地」，与 -Normalize(A) 同向
        errs = []
        for _ in range(n):
            C = C @ expm_so3(w * dt)
            K = K + (np.cross(w * dt, K) if mode == "vec" else np.cross(K, w * dt))
            K = K / np.linalg.norm(K)
            errs.append(np.degrees(np.arccos(np.clip(np.dot(K, C[2, :]), -1.0, 1.0))))
        e = np.array(errs)
        out[mode] = (e[int(1 / dt) - 1], e[int(10 / dt) - 1], e[-1])
    return out


def _sign_matrix_table():
    """三变量 2×2×2 全试。表里数字由 _sign_convention_run 现场算出。"""
    cases = [
        ("A", True, True, "r x m", "★ 速查表字面《矢量 + (-A) + 估计×测量》"),
        ("B", True, True, "m x r", "只改叉积顺序"),
        ("C", False, True, "r x m", "只改更新形式"),
        ("D", False, True, "m x r", "改更新形式 + 叉积顺序"),
        ("E", True, False, "r x m", "只改加计符号"),
        ("F", True, False, "m x r", "改加计符号 + 叉积顺序"),
        ("G", False, False, "r x m", "改更新形式 + 加计符号"),
        ("H", False, False, "m x r", "★ 本课 / ArduPilot（矩阵右乘 + 测量×估计）"),
    ]
    rows = []
    cache = {}
    for tag, vec, neg, order, note in cases:
        e = _sign_convention_run(vec, neg, order)
        cache[tag] = e
        rows.append([tag,
                     "矢量 K+dθ×K" if vec else "矩阵 C·(I+[dθ×])",
                     "-Normalize(A)" if neg else "+Normalize(A)",
                     "估计×测量" if order == "r x m" else "测量×估计",
                     "%.4f" % e[-1],
                     "%.4f" % e.max(),
                     note])
    table(rows, ["#", "更新形式", "加计符号", "叉积顺序",
                 "8s 末误差/度", "最大/度", "说明"],
          [3, 16, 15, 10, 11, 9, 40])
    return cache


def exp_5_pitch_rate_earth():
    head("实验 5 · 地球系角速度 vs 欧拉角速率 —— 读者原创推导核实")

    sub("5.1 那个函数到底在哪个版本里？（材料核实）")
    print("""
  ★ 材料核实结果：get_pitch_rate_earth() / get_roll_rate_earth() **不在** 4.5.7 里。
    它们在 **ArduCopter 2.9.1**：
        libraries/AP_AHRS/AP_AHRS.cpp:70   get_pitch_rate_earth()
        libraries/AP_AHRS/AP_AHRS.cpp:77   get_roll_rate_earth()
        libraries/AP_AHRS/AP_AHRS.h:82-83  两个函数的声明
    调用方：
        libraries/APM_Control/AP_PitchController.cpp:44
        libraries/APM_Control/AP_RollController.cpp:38
    4.5.7 里只剩一个 get_yaw_rate_earth()（AP_AHRS.h:514-516），而且它换了含义：
        return get_gyro() * get_rotation_body_to_ned().c;   <- 地球系 z **分量**
    两个版本都叫「earth rate」，含义却不是同一个东西。⚠️ 跨版本读代码要当心。
""")

    sub("5.2 两个函数返回的其实是**欧拉角速率** phi_dot / theta_dot")
    print("""
  3-2-1 (Z-Y-X) 的角速率运动学（标准结果，p/q/r 是机体系角速度）：
        p = phi_dot - psi_dot*sin(theta)
        q = theta_dot*cos(phi) + psi_dot*cos(theta)*sin(phi)
        r = -theta_dot*sin(phi) + psi_dot*cos(theta)*cos(phi)
  反解：
        theta_dot = q*cos(phi) - r*sin(phi)                     <-- get_pitch_rate_earth
        phi_dot   = p + tan(theta)*(q*sin(phi) + r*cos(phi))    <-- get_roll_rate_earth
  验证：下面用两条独立路径各算一遍（源码公式 vs 解线性方程组），看是否一致。
""")
    rng = np.random.default_rng(3)
    rows = []
    max_e1 = max_e2 = 0.0
    for _ in range(5):
        r = float(rng.uniform(-1.2, 1.2))
        p = float(rng.uniform(-1.2, 1.2))
        q_b = rng.uniform(-2, 2, 3)          # 机体系角速度 p,q,r
        roll_dot_code = q_b[0] + np.tan(p) * (q_b[1] * np.sin(r) + q_b[2] * np.cos(r))
        pitch_dot_code = np.cos(r) * q_b[1] - np.sin(r) * q_b[2]
        # 独立路径：用欧拉率 -> 机体系角速度的完整关系反解
        M = np.array([[1.0, 0.0, -np.sin(p)],
                      [0.0, np.cos(r), np.cos(p) * np.sin(r)],
                      [0.0, -np.sin(r), np.cos(p) * np.cos(r)]])
        phi_d, th_d, psi_d = np.linalg.solve(M, q_b)
        max_e1 = max(max_e1, abs(roll_dot_code - phi_d))
        max_e2 = max(max_e2, abs(pitch_dot_code - th_d))
        rows.append(["%6.2f" % np.degrees(r), "%6.2f" % np.degrees(p),
                     "%9.5f" % roll_dot_code, "%9.5f" % phi_d,
                     "%9.5f" % pitch_dot_code, "%9.5f" % th_d])
    table(rows, ["roll/度", "pitch/度", "代码 roll_dot", "反解 phi_dot",
                 "代码 pitch_dot", "反解 theta_dot"], [10, 10, 15, 15, 16, 16])
    print("    5 组随机姿态，两条独立路径的最大差：roll_dot %.2e, pitch_dot %.2e"
          % (max_e1, max_e2))
    print("""
  ★ 结论：2.9.1 的这两个函数返回的就是 **phi_dot / theta_dot（欧拉角速率）**，
    而且公式是**精确**的，没有任何近似。
""")

    sub("5.3 那它是不是「地球系角速度」？—— 不是，两者差一个 cos(theta)")
    print("""
  地球系角速度 = C·w_body，它的分量和欧拉率**不是**一回事。
     · 地球系 z 分量：r_e = psi_dot - phi_dot*sin(theta)
     · 4.5.7 的 get_yaw_rate_earth() 返回的正是 r_e
     · 欧拉偏航率 psi_dot = r_e / cos(theta)
  ⇒ θ = 0 时两者相等；θ ≠ 0 时**不同**。同理 theta_dot ≠ q_e。
""")
    rows = []
    for p_deg in (0.0, 30.0, 60.0, 80.0):
        p = np.radians(p_deg)
        r, y = np.radians(20.0), np.radians(35.0)
        C = euler_to_R(r, p, y)
        q_b = np.array([0.4, 0.7, -0.3])
        w_e = C @ q_b
        theta_dot = np.cos(r) * q_b[1] - np.sin(r) * q_b[2]
        rows.append(["%.0f" % p_deg, "%9.6f" % w_e[1], "%9.6f" % theta_dot,
                     "%9.6f" % abs(w_e[1] - theta_dot),
                     "%.1f%%" % (100 * abs(w_e[1] - theta_dot) / max(abs(theta_dot), 1e-9))])
    table(rows, ["pitch/度", "地球系 y 分量 q_e", "theta_dot（源码）", "绝对差", "相对差"],
          [10, 20, 18, 12, 10])
    print("""
  ★ 所以「地球系俯仰角速度」这个说法**本身有歧义**，必须说清是哪一个：
       (a) w 在地球系下的 y 分量  q_e = (C·w) 的第 2 个元素
       (b) 俯仰欧拉角的导数      theta_dot
    2.9.1 的 get_pitch_rate_earth() 返回的是 (b)，**不是** (a)。
    （两者在 pitch = 0 时数值相同，这也是它容易被误标的原因。）

  ★ 为什么控制律要 (b) 而不是机体系的 q？
    因为 AP_PitchController 控制的是**俯仰角**这个量。
    速率反馈必须是**被控量的导数**，否则在有大滚转角时，反馈的幅值和相位都会错
    （q 里混进了 psi_dot*cos(theta)*sin(phi) 这一项）。
    用 theta_dot 做阻尼，等效于对俯仰角做 PD —— 这才是它的物理意义。
""")

    sub("5.4 核实读者推导里的两处问题（只指出，不改原文）")
    print("""
  读者文档把这条链子基本走通了，最终公式与源码一致，但中间有两步不成立：

  ❶ 「AP_AHRS__get_pitch_rate_earth 函数（带详细注释版）.md」的 roll 那一半：
     文档写「等式两边除以 cos(theta)，再由 cos(theta) ≈ 1 得 p/cos(theta) ≈ p」。
     这一步是**多余且有害**的：除以 cos(theta) 之后，左边已经不是 p 而是 phi_dot
     —— 源代码返回的本来就是 phi_dot，不需要任何近似。
     若按文档「近似」的说法去实现（真的去算 p 而不除以 cos），在 30 度俯仰下
     会引入 1/cos(30°) - 1 = 15.5% 的**系统性**误差
     （文档里写的 13.4% 是 1-cos(30°)，那是另一个量，不是这个误差）。
     而源码是精确的，根本没有这个误差。

  ❷ 文档说「代码 Z 轴约定与标准旋转矩阵有细微差异，所以把括号里的 - 改成 +」。
     这属于**用符号调整掩盖推导缺口**。
     实测（5.2 表）证明 + 号是**精确推导的结果**，不需要任何符号补救：
     把 q、r 的完整表达式代进 q*cos(phi) - r*sin(phi)，psi_dot 项自动抵消。
     原因：文档用的是组合矩阵直接投影，得到的是地球系分量 q_e；
     而函数要的是欧拉率 theta_dot。两者的桥不是「符号差异」，
     是**欧拉角速率运动学**（即 5.2 里那个线性方程组）。

  ★ 这条链子本身没问题：控制律需要「地球系（欧拉）角速率」这个**需求判断**是对的，
    只是第 4 章的互补滤波还没学，所以文档里的论证绕了远路。
    本课第 3.6.12 节会把这条链子重新走直。
""")


# ============================================================================
#  6DOF / 9DOF 互补滤波（供实验 6、7 共用）
# ============================================================================
def synth_trajectory(n, dt, seed=11, gyro_bias=(0.01, -0.015, 0.008),
                     gyro_noise=0.002, accel_noise=0.01, mag_noise=0.01,
                     declination=0.0, inclination=0.0):
    """合成一段「陀螺 + 加计 + 磁」数据。

    真值：用精确指数映射积分 C_true（机体系->NED）。
    传感器：
      gyro_body  = w_true + bias + noise
      accel_body = C_true^T · (0,0,-1) + noise    （比力：静止指向天）
      mag_body   = C_true^T · m_ned + noise       （m_ned = 北向绕「地」转 declination）

    m_ned 的构造（declination = 磁偏角，inclination = 磁倾角，北半球为**正**）：
      m_ned = ( cosI·cosD,  cosI·sinD,  sinI )
        · I = 0,  D = 0 → (1,0,0)，纯水平指北（理想磁力计，用于对比实验）
        · I ≠ 0        → 磁场有向下的分量，此时磁矢量**不在水平面内**，
                          如果不投影掉机体系上的天顶分量，就会把倾角当成俯仰误差
    """
    rng = np.random.default_rng(seed)
    C = np.eye(3)
    bias = np.array(gyro_bias, dtype=float)
    cI, sI = np.cos(inclination), np.sin(inclination)
    m_ned = np.array([cI * np.cos(declination), cI * np.sin(declination), sI])

    gyro_l, accel_l, mag_l, Ctrue_l = [], [], [], []
    for k in range(n):
        t = k * dt
        w_true = np.array([0.45 * np.sin(0.35 * t),
                           0.40 * np.cos(0.27 * t) - 0.10,
                           0.30 * np.sin(0.19 * t) + 0.35])
        C = C @ expm_so3(w_true * dt)
        C = C / np.linalg.norm(C, axis=1, keepdims=True)
        gyro_l.append(w_true + bias + rng.normal(0, gyro_noise, 3))
        accel_l.append(C.T @ np.array([0.0, 0.0, -1.0]) + rng.normal(0, accel_noise, 3))
        mag_l.append(C.T @ m_ned + rng.normal(0, mag_noise, 3))
        Ctrue_l.append(C.copy())
    return np.array(gyro_l), np.array(accel_l), np.array(mag_l), np.array(Ctrue_l)


def _cross_for(m, r, order):
    return np.cross(m, r) if order == "m x r" else np.cross(r, m)


def dcm_filter(gyro, accel, mag, dt, sa=0.02, sg=1.0, sm=0.0,
               cross_order="m x r", use_ortho=True, normalize_step=True,
               mag_projection=True):
    """速查表 7 步流程的完整实现（NED 约定，C = 机体系->NED）。

    与速查表的对应关系：
      天顶向量 K_B = -C[2] （C 的第三行是「地」，取负就是「天」）
      北向向量 I_B =  C[0]
      西向向量 J_B = -C[1]
    修正角位移一律用 dθ = 测量 × 估计（cross_order='m x r'）。
    """
    n = len(gyro)
    C = np.eye(3)
    logs = []
    for k in range(n):
        # --- 步骤 1：陀螺预测（角位移）---
        dtheta_g = gyro[k] * dt

        # --- 步骤 2：测量方向 ---
        K_B0 = -C[2, :].copy()                       # 估计的天顶
        I_B0 = C[0, :].copy()                        # 估计的北向
        K_B1A = accel[k] / np.linalg.norm(accel[k])  # ★ 比力约定：不取负

        # --- 步骤 3：加速度计等效角位移 ---
        dtheta_a = _cross_for(K_B1A, K_B0, cross_order)
        dtheta_a = dtheta_a - np.dot(dtheta_a, K_B0) * K_B0    # 去掉绕天顶的分量

        # --- 步骤 3b：磁力计等效角位移 ---
        if mag is not None and sm > 0.0:
            I_B1M = mag[k] / np.linalg.norm(mag[k])
            if mag_projection:
                I_B1M = I_B1M - np.dot(I_B1M, K_B1A) * K_B1A
                I_B1M = I_B1M / np.linalg.norm(I_B1M)
            dtheta_m = _cross_for(I_B1M, I_B0, cross_order)
            num = sa * dtheta_a + sg * dtheta_g + sm * dtheta_m
            den = sa + sg + sm
        else:
            num = sa * dtheta_a + sg * dtheta_g
            den = sa + sg

        # --- 步骤 5：更新（等价于 C <- C·(I+[dθ x])，右乘！）---
        C = C @ expm_so3_2nd(num / den)

        # --- 步骤 6：正交归一化 ---
        if use_ortho:
            I = C[0, :].copy()          # 北
            J = -C[1, :].copy()         # 西 = -东
            Err = float(np.dot(I, J)) / 2.0
            In = I - Err * J            # ★ 用原始的 I、J 算两个新值
            Jn = J - Err * I
            I, J = In, Jn
            if normalize_step:
                I = I / np.linalg.norm(I)
                J = J / np.linalg.norm(J)
            K = np.cross(I, J)          # 天顶由叉积重建
            if normalize_step:
                K = K / np.linalg.norm(K)
            C = np.array([I, -J, -K])   # 行 = 北、东、地
        elif normalize_step:
            C = C / np.linalg.norm(C, axis=1, keepdims=True)

        logs.append(C.copy())
    return np.array(logs)


def exp_6_complementary_filter_6dof():
    head("实验 6 · 完整 6DOF DCM 互补滤波")

    dt = 0.002
    n = 12000                      # 24 秒
    gyro, accel, _mag, Ctrue = synth_trajectory(n, dt)

    print("""
  流程（速查表步骤 1~7，NED 约定）：
    1. dtheta_g = w_g * dt                             陀螺预测
    2. K_B1A = +Normalize(A)                           ★ 比力约定，不取负
    3. dtheta_a = K_B1A x K_B0                         加计等效角位移（测量 × 估计）
    4. dtheta = (sa*dtheta_a + sg*dtheta_g)/(sa+sg)    加权融合
    5. C <- C * (I + [dtheta x])                       更新（右乘！）
    6. 对称正交化 -> 单位化 -> K = I x J                正交归一化
    7. 回到 1
""")

    sub("6.1 权重 sa 扫描：sa 越大越信加计")
    rows = []
    for sa in (0.0, 0.005, 0.02, 0.1, 1.0):
        Cest = dcm_filter(gyro, accel, None, dt, sa=sa, sg=1.0)
        errs = [geo_deg(Cest[k], Ctrue[k]) for k in range(0, n, 400)]
        rows.append(["%.3f" % sa, "%.3f" % errs[10], "%.3f" % np.mean(errs[10:]),
                     "%.3f" % np.max(errs[10:])])
    table(rows, ["sa", "第 10 次采样误差/度", "稳态平均误差/度", "稳态最大误差/度"],
          [10, 18, 18, 18])
    print("""
  ★ 读法：
    · sa = 0（纯陀螺）：误差**线性发散** —— 陀螺的常值零偏被积分成角度漂移。
    · sa 从小到大：稳态误差先降后升。太小压不住零偏，太大把加计的噪声
      和「非重力加速度」也一起信了。
    · 存在一个最优区间。「谁快信谁、谁慢信谁」的直觉，在这里被量化成一条曲线。

  ★ 加权融合的物理意义（一阶低通类比）：
    设 s = sa/(sa+sg)，则
        dtheta = (1-s)*dtheta_g + s*dtheta_a
    这就是一个**互补滤波器**：
        · 陀螺支路是「高通」—— 短期信它（它快、但会漂）
        · 加计支路是「低频」—— 长期信它（它慢、但有绝对参考）
    连续域的等效截止频率约 f_c ≈ s / (2*pi*dt)。本实验 dt = 2ms、s = 0.02，
    f_c ≈ 1.6 Hz —— 意思是：姿态变化慢于 1.6 Hz 的部分听加计的，
    快于 1.6 Hz 的部分听陀螺的。
    第 4 章会把这条式子写成标准的一阶互补滤波传递函数。
""")

    sub("6.2 ★ 多方对拍：正确实现 vs 叉积写反 vs 不做正交归一化")
    sa = 0.02
    C_std = dcm_filter(gyro, accel, None, dt, sa=sa, sg=1.0)
    C_bad = dcm_filter(gyro, accel, None, dt, sa=sa, sg=1.0, cross_order="r x m")
    C_noortho = dcm_filter(gyro, accel, None, dt, sa=sa, sg=1.0,
                           use_ortho=False, normalize_step=False)
    C_nonorm = dcm_filter(gyro, accel, None, dt, sa=sa, sg=1.0,
                          use_ortho=False, normalize_step=True)

    rows = []
    for label, Cx in [("正确实现（速查表 7 步 + 正确符号）", C_std),
                      ("★ 叉积顺序写反（= 速查表步骤 3 的字面写法）", C_bad),
                      ("省略步骤 6，只按行单位化", C_nonorm),
                      ("省略步骤 6 且完全不归一化", C_noortho)]:
        errs = [geo_deg(Cx[k], Ctrue[k]) for k in range(0, n, 400)]
        rows.append([label, "%.3f" % np.mean(errs[10:]), "%.3f" % np.max(errs[10:]),
                     "%.2e" % orth_err(Cx[-1]), "%.2e" % det_err(Cx[-1])])
    table(rows, ["实现", "稳态平均误差/度", "稳态最大误差/度", "max|CᵀC-I|", "|det|-1"],
          [38, 16, 16, 12, 12])
    print("""
  ★ 多方对拍告诉你四件事：
    ① 正确实现稳态误差在几度以内（下限由加计噪声和 sa/sg 配比决定），
       验证了整条 7 步流程和「dθ = 测量 × 估计」这个叉积顺序是对的。
       这也和 ArduPilot 的 error[i] = GA_b[i] % GA_e（AP_AHRS_DCM.cpp:834）
       + 右乘（AP_Math/matrix3.cpp:115）完全一致 —— 两条独立实现路径互相印证。
    ② 叉积顺序写反那一版：姿态被**推离**真值，稳态误差 179.9 度，
       是正确实现的约 60 倍（几乎完全反向）。
       和第 4.3/4.4 节的符号问题是同一类错误，现象也一样：
       静止时姿态自己慢慢翻过去。
    ③ ★★ 关于正交归一化，数值给出的结论比「想当然」更微妙，必须照实说：
       · 看**姿态误差**列：三种「省略步骤 6」的写法**数字一模一样**
         （2.887 / 3.768），完全分不出来。
         原因是 —— 互补滤波的加计校正项本身就在把矩阵往真值方向拉，
         主导误差是「陀螺零偏 0.01 rad/s × 24 s = 0.24 rad ≈ 14 度」，
         而约束误差只有 1e-8 量级，完全被淹没。
       · 看 **max|CᵀC-I|** 列才是正交归一化的主场：
         从 1.02e-08 直接掉到 2.22e-16，**8 个数量级**。
       ⇒ **本实验不是「不正交归一化就会飞歪」的证据**；
         它证明的是另一件事：**姿态精度**和**矩阵约束**是两个独立的指标，
         互补滤波能救前者，救不了后者。
         正交归一化要展示威力，需要「长时间 / 高角速率 / 单精度浮点」
         这三个条件之一 —— 见实验 1（1e6 步 → 偏差 0.41）
         和实验 8（一阶不归一化 → 约束误差 5.6e-02，8 个数量级之外）。

  ★ 顺带一个值得记住的定量事实：这 1e-8 是哪来的？
    本实现的更新用二阶指数 `I + [dθ×] + [dθ×]²/2`。直接算它的正交性缺陷：
        MᵀM = (I - K + K²/2)(I + K + K²/2) = I + K⁴/4      （K = [dθ×]）
    ⇒ **二阶指数的单步正交性缺陷是 O(|dθ|⁴)**，不是 O(|dθ|²)。
    |dθ| ≈ 1e-3 ⇒ 单步 ≈ 2.5e-13；12000 步累积 ≈ 1e-8 ✓ 和实测吻合。
    而实验 8.1 里那个**一阶**的 `I + [dθ×]`（注意 Kᵀ = -K）：
        (I+K)ᵀ(I+K) = (I-K)(I+K) = I - K²   ⇒ 单步缺陷 O(|dθ|²) ≈ 2.6e-6
    40000 步累积到 5.6e-02 ✓ 也吻合。（两式差一个符号，不重要；差的是
    |dθ|² ≈ 1e-6 倍这个**阶数**。）
    ★ 所以「从一阶升到二阶」不只是提高积分精度，还把**约束退化速度**
      从 O(|dθ|²) 降到了 O(|dθ|⁴) —— 一举两得，而且几乎不增加计算量。
      这就是 ArduPilot 在 `matrix3.cpp:115` 里仍然只用一阶 `+= C[g×]`
      （因为它每步都做 normalize()，不需要靠高阶指数维持约束），
      而 MatrixPilot 在 rmat.c:271-278 显式做二阶修正的原因。
      两种选择都自洽，取决于「你打不打算每步都归一化」。
       · 但它**必须做**：一旦矩阵不再严格正交，后续任何坐标变换
         （比如 ArduPilot 把加计转到地球系算 _accel_ef，AP_AHRS_DCM.cpp:677）
         都会把「尺度误差」当成真实物理量，误差会耦合放大；
         而且误差是**单调累积**的，飞 10 分钟、1 小时以后就不一样了。
         工程上做这一步的代价只是一次 sqrt，没有不做的理由。
""")

    sub("6.3 绘制姿态收敛曲线")
    _plot_6dof(gyro, accel, dt, Ctrue)


def _cjk_font():
    """自动探测可用中文字体，返回 (plt, 是否找到中文字体)。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.font_manager as _fm
    except ImportError:
        return None, False
    for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                  "Source Han Sans SC", "WenQuanYi Micro Hei",
                  "PingFang SC", "Heiti SC"):
        try:
            _fm.findfont(_name, fallback_to_default=False)
        except Exception:
            continue
        matplotlib.rcParams["font.sans-serif"] = [_name]
        matplotlib.rcParams["axes.unicode_minus"] = False
        return plt, True
    return plt, False


def _plot_6dof(gyro, accel, dt, Ctrue):
    plt, cjk = _cjk_font()
    if plt is None:
        print("  （没装 matplotlib，跳过画图）")
        return

    def T(zh, en):
        return zh if cjk else en

    n = len(gyro)
    cases = [("sa=0（纯陀螺）", 0.0), ("sa=0.005", 0.005),
             ("sa=0.02", 0.02), ("sa=0.1", 0.1)]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    for label, sa in cases:
        Cest = dcm_filter(gyro, accel, None, dt, sa=sa, sg=1.0)
        ts, er, rp = [], [], []
        for k in range(0, n, 40):
            ts.append(k * dt)
            er.append(geo_deg(Cest[k], Ctrue[k]))
            rp.append(np.degrees(R_to_euler(Cest[k])[0]))
        axes[0].plot(ts, er, lw=1.2, label=label)
        if sa == 0.02:
            rr = [np.degrees(R_to_euler(Ctrue[k])[0]) for k in range(0, n, 40)]
            axes[1].plot(ts, rr, "k--", lw=1.0, label=T("真值 roll", "true roll"))
            axes[1].plot(ts, rp, lw=1.2, label=T("估计 roll", "est roll"))
    axes[0].set_yscale("log")
    axes[0].set_xlabel(T("时间 / s", "time / s"))
    axes[0].set_ylabel(T("姿态误差 / 度（对数）", "attitude err / deg (log)"))
    axes[0].set_title(T("6DOF：权重 sa 对收敛的影响", "6DOF: effect of weight sa"))
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel(T("时间 / s", "time / s"))
    axes[1].set_ylabel(T("roll / 度", "roll / deg"))
    axes[1].set_title(T("6DOF：roll 跟踪（sa=0.02）", "6DOF: roll tracking (sa=0.02)"))
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    out = "04_dcm_6dof.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print("  ✅ 图已存：%s%s" % (out, "" if cjk else "（无中文字体，标签用英文）"))


def exp_7_complementary_filter_9dof():
    head("实验 7 · 9DOF：加磁力计锁航向")

    dt = 0.002
    n = 16000

    # 对比实验用「理想磁力计」：磁矢量严格水平指北（D = 0, I = 0）。
    # 这不是为了好看，而是为了**单变量**：先让磁力计这个传感器本身可信、
    # 无系统误差，才能干净地回答「加了磁力计到底有没有用」。
    # 磁偏角 / 磁倾角这两个真实世界的系统误差，放到 7.3 / 7.4 单独处理。
    gyro, accel, mag, Ctrue = synth_trajectory(n, dt)

    print("""
  6DOF 的根本缺陷：加速度计测的是「天顶」，它对**绕天顶轴**的旋转完全不敏感
  （K_B 不变），所以偏航角没有任何绝对参考，只能靠陀螺积分 -> 航向必然漂移。
  加磁力计给出「北」，才把最后一个自由度锁住。
""")

    def _euler_errs(Cx, yaw_offset_deg=0.0):
        roll_err, pitch_err, yaw_err = [], [], []
        for k in range(0, n, 400):
            r1, p1, y1 = R_to_euler(Cx[k])
            r2, p2, y2 = R_to_euler(Ctrue[k])
            yaw_err.append(abs(np.degrees(wrap_pi(y1 - yaw_offset_deg - y2))))
            roll_err.append(abs(np.degrees(wrap_pi(r1 - r2))))
            pitch_err.append(abs(np.degrees(wrap_pi(p1 - p2))))
        return roll_err, pitch_err, yaw_err

    sub("7.1 6DOF vs 9DOF：三个角的平均误差对比")
    C6 = dcm_filter(gyro, accel, None, dt, sa=0.02, sg=1.0)
    C9 = dcm_filter(gyro, accel, mag, dt, sa=0.02, sg=1.0, sm=0.05)

    rows = []
    for label, Cx in [("6DOF（陀螺+加计）", C6), ("9DOF（+磁力计）", C9)]:
        roll_err, pitch_err, yaw_err = _euler_errs(Cx)
        rows.append([label, "%.2f" % np.mean(roll_err[10:]),
                     "%.2f" % np.mean(pitch_err[10:]),
                     "%.2f" % np.mean(yaw_err[10:]),
                     "%.2f" % np.max(yaw_err[10:])])
    table(rows, ["配置", "roll 平均误差/度", "pitch 平均误差/度",
                 "yaw 平均误差/度", "yaw 最大误差/度"], [22, 16, 17, 16, 17])
    print("""
  ★ 读法：
    · roll/pitch 两行差不多（都在一度以内）—— 磁力计对横滚俯仰**没有副作用**，
      它只管它该管的那一个自由度。
    · 差别**只在 yaw**：6DOF 的 yaw 误差大一个量级，而且它是**随时间增长**的
      （陀螺零偏 0.01 rad/s × 32 s ≈ 0.32 rad ≈ 18 度，这就是「方位角漂移」）；
      9DOF 被磁力计拉住，误差回到噪声量级。
    ⇒ **互补滤波缺哪个自由度，就要补哪个自由度的绝对参考。**
      加计补 pitch/roll，磁补 yaw，GPS 速度补地速 —— 这是所有姿态/导航滤波器的
      统一骨架，第 4 章会把它写成一个通用形式。
""")

    sub("7.2 磁力计权重 sm 扫描")
    rows = []
    for sm in (0.0, 0.005, 0.02, 0.05, 0.2, 0.5):
        Cx = dcm_filter(gyro, accel, mag, dt, sa=0.02, sg=1.0, sm=sm)
        _, re_, ye = _euler_errs(Cx)
        rows.append(["%.3f" % sm, "%.2f" % np.mean(re_[10:]), "%.2f" % np.mean(ye[10:]),
                     "%.2f" % np.max(ye[10:])])
    table(rows, ["sm", "roll 平均误差/度", "yaw 平均误差/度", "yaw 最大误差/度"],
          [10, 16, 16, 16])
    print("""
  ★ 读法：
    · sm = 0（退化成 6DOF）：yaw 误差随陀螺零偏线性增长，这就是「方位角漂移」。
    · sm 增大：yaw 被磁力计拉回，误差掉到一度以内。
    · sm 再增大（0.5）：yaw 误差反而回升 —— 磁力计噪声开始主导。
      而且 sm 过大会把磁力计的**硬磁/软磁误差**也一起信进来，
      所以工程上 sm 一般取得比 sa 还小。
      速查表 v0:190 的工程注意事项「需抗干扰则增大 s_g，需姿态精度则增大 s_a/s_m」
      说的就是这个取舍。

  ⚠️ 注意 sm 的最优值和 sa 一样**不是普适常数**，它取决于：
     陀螺零偏有多大、磁力计噪声有多大、磁场环境有多脏。
     真实工程里要按传感器实际噪声密度算（第 4 章会给出公式）。
""")

    sub("7.3 ★ 陷阱一：磁力计必须先「去天顶投影」，不能直接归一化")
    print("""
  磁力计测的是**三维地磁矢量**。在真实地球上它并不水平，而是斜的
  （磁倾角 inclination，北京约 58°，即磁矢量指向北偏下）。
  直接 Normalize(M) 得到的是「磁北 + 一部分天顶」的合成方向，
  拿它当「北」用，就等于把一部分「天顶」信息混进航向观测里 ——
  磁力计会去纠正俯仰，和加速度计**打架**。
  正确做法：先在估计的姿态下把它投影到垂直于天顶的平面上，才是干净的北向。

  下面用真实量级的磁倾角（58°）重做一遍，两种做法摆一起：
""")
    gyro_i, accel_i, mag_i, Ctrue_i = synth_trajectory(
        n, dt, inclination=np.radians(58.0))

    def _euler_errs_i(Cx):
        re_, ye = [], []
        for k in range(0, n, 400):
            r1, p1, y1 = R_to_euler(Cx[k])
            r2, p2, y2 = R_to_euler(Ctrue_i[k])
            ye.append(abs(np.degrees(wrap_pi(y1 - y2))))
            re_.append(abs(np.degrees(wrap_pi(r1 - r2))))
        return re_, ye

    C9p = dcm_filter(gyro_i, accel_i, mag_i, dt, sa=0.02, sg=1.0, sm=0.05,
                     mag_projection=True)
    C9n = dcm_filter(gyro_i, accel_i, mag_i, dt, sa=0.02, sg=1.0, sm=0.05,
                     mag_projection=False)
    rows = []
    for label, Cx in [("先投影到水平面（正确）", C9p), ("直接归一化（错）", C9n)]:
        re_, ye = _euler_errs_i(Cx)
        rows.append([label, "%.2f" % np.mean(re_[10:]), "%.2f" % np.mean(ye[10:]),
                     "%.2f" % np.max(ye[10:])])
    table(rows, ["做法", "roll 平均误差/度", "yaw 平均误差/度", "yaw 最大误差/度"],
          [24, 16, 16, 16])
    print("""
  ★ 磁倾角 58° 时，磁矢量的「天顶」分量占 sin58° ≈ 85%，比水平分量还大。
    不投影就等于让磁力计用 85% 的力气去纠正俯仰 —— 它会和加计争夺俯仰角，
    表现为 roll/pitch 被拽偏、yaw 抖动。
    ArduPilot 的做法更巧妙（AP_AHRS_DCM.cpp:353-355）：
        Vector2f rb = _dcm_matrix.mulXY(mag);
    mulXY 只取 C 的**前两行**（地球系 x、y 轴）与 mag 点乘
    （AP_Math/matrix3.cpp:150-155），**天然就把天顶分量丢掉了** ——
    投影不是额外做的一步，而是「只取水平两个分量」的必然结果。
""")

    sub("7.4 ★ 陷阱二：磁北 ≠ 真北（磁偏角）")
    print("""
  磁力计锁住的是**磁北**。航向要的是**真北**，两者差一个磁偏角 D。
  不补偿的话，航向误差是一个**恒定值 D** —— 它不漂、不抖，就静静地错着，
  所以极其容易被漏掉（飞行表现为「一直朝某个方向偏一点」）。
  下面注入 D = 8° 的磁偏角，看补偿前后的差别：
""")
    gyro_d, accel_d, mag_d, Ctrue_d = synth_trajectory(
        n, dt, declination=np.radians(8.0))

    def _yaw_err_d(Cx, offset_deg):
        ye = []
        for k in range(0, n, 400):
            _, _, y1 = R_to_euler(Cx[k])
            _, _, y2 = R_to_euler(Ctrue_d[k])
            ye.append(abs(np.degrees(wrap_pi(y1 + np.radians(offset_deg) - y2))))
        return ye

    C9d = dcm_filter(gyro_d, accel_d, mag_d, dt, sa=0.02, sg=1.0, sm=0.05)
    print("  磁矢量 m_ned = (cos D, sin D, 0)，D = +8 度（磁北在真北以东 8 度）")
    rows = []
    for label, off in [("不补偿磁偏角（错）", 0.0),
                       ("补偿 D = +8°（正确）", 8.0)]:
        ye = _yaw_err_d(C9d, off)
        rows.append([label, "%.2f" % np.mean(ye[10:]), "%.2f" % np.max(ye[10:])])
    table(rows, ["做法", "yaw 平均误差/度", "yaw 最大误差/度"], [24, 16, 16])
    print("""
  ★ 不补偿那一行的 yaw 误差**恰好等于磁偏角本身**（8.00 度），
    且**不随时间变化** —— 这就是「恒定系统误差」的指纹，
    和陀螺零偏造成的「线性增长」是两种完全不同的病。
    分清这两者，是排查航向问题的第一步：
      · 误差随时间线性长  -> 陀螺零偏 / 标定问题
      · 误差恒定但很大    -> 磁偏角没补偿 / 安装角没对齐
      · 误差随机抖动      -> 磁干扰 / 振动
    实机上补偿磁偏角要么用 GPS 航迹（有速度时）自动估，
    要么从磁偏角模型按经纬度查表 —— 第 2 章会展开。
""")


# ============================================================================
#  实验 8：DCM vs 四元数
# ============================================================================
def qmul(p, q):
    w1, x1, y1, z1 = p
    w2, x2, y2, z2 = q
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def q_to_R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def delta_q_first_order(omega, dt):
    w = omega * dt
    return np.array([1.0, 0.5 * w[0], 0.5 * w[1], 0.5 * w[2]])


def delta_q_exp(omega, dt):
    """精确增量四元数，对应 ArduPilot Quaternion::from_angular_velocity()。"""
    w = omega * dt
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.array([1.0, 0.5 * w[0], 0.5 * w[1], 0.5 * w[2]])
    axis = w / th
    return np.concatenate([[np.cos(th / 2)], axis * np.sin(th / 2)])


def exp_8_dcm_vs_quaternion():
    head("实验 8 · DCM vs 四元数：精度 + 计算量")

    dt = 0.002
    n = 40000                              # 80 秒
    gyro, _a, _m, Ctrue = synth_trajectory(n, dt, seed=5,
                                           gyro_bias=(0, 0, 0),
                                           gyro_noise=0.0,
                                           accel_noise=0.0, mag_noise=0.0)

    sub("8.1 同样一段角速率序列，六种积分方法的末态误差")
    print("""
  ⚠️ 指标说明：这里的角速率序列是**精确离散**的（真值由精确指数映射生成），
     所以末态误差同时包含「积分方案误差」和「约束维护误差」两部分，
     必须分开看：max|C·C_trueᵀ−I| 看**姿态**，max|CᵀC−I| 看**约束**。
     不能用 geo_deg 单看 —— 它对「整体缩放」会饱和（见 geo_deg 的注释），
     会把最差的那一行显示成 0。
""")
    C = np.eye(3)
    for k in range(n):
        C = C + C @ skew(gyro[k] * dt)
    r_dcm_raw = (geo_deg(C, Ctrue[-1]), mat_err(C, Ctrue[-1]), orth_err(C), det_err(C))

    C = np.eye(3)
    for k in range(n):
        C = C @ expm_so3_2nd(gyro[k] * dt)
        C = C / np.linalg.norm(C, axis=1, keepdims=True)
    r_dcm_norm = (geo_deg(C, Ctrue[-1]), mat_err(C, Ctrue[-1]), orth_err(C), det_err(C))

    C = np.eye(3)
    for k in range(n):
        C = _orthonormalize_ned(C @ expm_so3_2nd(gyro[k] * dt))
    r_dcm_ortho = (geo_deg(C, Ctrue[-1]), mat_err(C, Ctrue[-1]), orth_err(C), det_err(C))

    C = np.eye(3)
    for k in range(n):
        C = C @ expm_so3(gyro[k] * dt)
    r_dcm_exp = (geo_deg(C, Ctrue[-1]), mat_err(C, Ctrue[-1]), orth_err(C), det_err(C))

    q = np.array([1.0, 0, 0, 0])
    for k in range(n):
        q = qmul(q, delta_q_first_order(gyro[k], dt))
        q = q / np.linalg.norm(q)
    Rq1 = q_to_R(q)
    r_q_norm = (geo_deg(Rq1, Ctrue[-1]), mat_err(Rq1, Ctrue[-1]),
                orth_err(Rq1), det_err(Rq1))

    q = np.array([1.0, 0, 0, 0])
    for k in range(n):
        q = qmul(q, delta_q_exp(gyro[k], dt))
        q = q / np.linalg.norm(q)
    Rq2 = q_to_R(q)
    r_q_exp = (geo_deg(Rq2, Ctrue[-1]), mat_err(Rq2, Ctrue[-1]),
               orth_err(Rq2), det_err(Rq2))

    rows = []
    for label, (e, m, o, d), nstate in [
            ("DCM 一阶，不归一化", r_dcm_raw, "9 / 否"),
            ("DCM 一阶 + 每步按行归一化", r_dcm_norm, "9 / 是"),
            ("DCM + 完整正交归一化（本课流程）", r_dcm_ortho, "9 / 是"),
            ("DCM 精确指数（罗德里格斯）", r_dcm_exp, "9 / 否"),
            ("四元数一阶 + 归一化", r_q_norm, "4 / 是"),
            ("四元数精确指数", r_q_exp, "4 / 是")]:
        rows.append([label, "%.4e" % e, "%.3e" % m, "%.3e" % o, "%.3e" % d, nstate])
    table(rows, ["方法", "测地误差/度", "max|C·Ctᵀ−I|", "max|CᵀC−I|", "|det|−1", "状态/归一"],
          [32, 12, 13, 13, 11, 10])
    print("""
  ★ 六个读数，逐个说清楚：
    ① DCM 一阶不归一化：姿态误差 2.704e-02、约束误差 5.596e-02、|det|−1 6.339e-02，
       三项**同量级**，是六行里最差的。这就是实验 1 的后果在 80 秒尺度上的样子。
       注意它的「测地误差」列反而显示 0.0000e+00 —— 那正是 geo_deg 的饱和 bug，
       矩阵被整体放大了 det ≈ 1.063，trace 超过 3 被 clip，于是读出 0 度。
       ⚠️ **这是本课要你记住的第一个「指标陷阱」**：用一个会在非正交矩阵上
          失真的指标去评价 DCM，会得出完全相反的结论（把最差的判成最好的）。
    ② DCM 一阶 + 按行归一化：|det|−1 被压到机器精度，测地误差也回到 3.2e-04。
       但 max|CᵀC−I| 只压到 2.0e-09 —— 比 ③ 差 **近 7 个数量级**。
       归一化只管模长、不管夹角，这句话在这里被量化了。
    ③ DCM + 完整正交归一化：约束误差 4.4e-16（机器精度），
       测地误差 3.2e-04 —— 和 ② **完全一样**（3.2387e-04 vs 3.2386e-04）。
       ★★ 这是本实验最重要的一条结论，也是最反直觉的一条：
          **正交归一化不改善姿态精度，它只维护矩阵约束。**
          姿态误差此时完全由「积分方案是一阶」决定，
          正交归一化把那 9 个数重新约束到 SO(3) 上，但约束不回退积分误差。
          它的价值在于：让后续所有坐标变换合法、让误差不随时间单调累积
          （本实验只有 80 秒，累积项还没成为主导 —— 见实验 1 的 1000 秒）。
    ④ DCM 精确指数：约束误差 2.376e-14（一次正交归一化都没做，纯靠精确指数
       自己维持），测地误差 7.636e-06 —— 比一阶好 42 倍。
       所以**真正的精度来源是积分阶数，不是正交归一化**。
       这正是第 3 周（03-姿态运动学与数值积分.md）讲二阶/指数积分的原因，
       也是本课的定位：正交归一化解决的是**约束**问题，
       而约束问题和精度问题是两件事，必须分开谈、分开治。
    ⑤ 四元数一阶 + 归一化：测地误差 1.6193e-04，比 DCM 一阶+正交归一化
       还好 2 倍，代价只有 4 个数、1 个约束、1 次 sqrt。
       归一化一步就把**全部**约束满足 —— 因为四元数只有 1 个约束，
       而 DCM 有 6 个。这就是四元数在工程上胜出的根本原因（不是「精度高」，
       是「约束便宜」）。
    ⑥ 四元数精确指数：测地误差 0.0e+00（和真值走同一个更新公式，
       真值那一路还多做了每步行归一化，所以 max 列的 9.354e-14 是
       真值归一化引入的，不是四元数的误差）。
""")

    sub("8.2 计算量实测（纯 Python，只比较**相对**量级）")
    reps = 3

    def timeit(fn):
        best = 1e9
        for _ in range(reps):
            t0 = time.perf_counter()
            fn()
            best = min(best, time.perf_counter() - t0)
        return best

    def run_dcm_euler():
        C = np.eye(3)
        for k in range(n):
            C = C + C @ skew(gyro[k] * dt)
        return C

    def run_dcm_ortho():
        C = np.eye(3)
        for k in range(n):
            C = _orthonormalize_ned(C @ expm_so3_2nd(gyro[k] * dt))
        return C

    def run_quat():
        q = np.array([1.0, 0, 0, 0])
        for k in range(n):
            q = qmul(q, delta_q_first_order(gyro[k], dt))
            q = q / np.linalg.norm(q)
        return q

    timings = []
    for label, fn in [("DCM：一阶积分（不归一化）", run_dcm_euler),
                      ("DCM：积分 + 完整正交归一化", run_dcm_ortho),
                      ("四元数：一阶 + 归一化", run_quat)]:
        timings.append((label, timeit(fn)))

    base = timings[-1][1]
    rows = [[label, "%.1f" % (t * 1000), "%.2f" % (t / base)] for label, t in timings]
    table(rows, ["方法", "%d 步总耗时/ms" % n, "相对四元数倍数"], [30, 18, 18])
    print("""
  ⚠️ 这是 **Python + numpy** 的计时，每次 `@` 都在建临时数组，
     绝对数值没有意义，只看**相对量级**。
     真实 MCU 上的对比见第 6 章（C++ 浮点/定点实现）。

  ★ 量级结论：
    · 纯姿态传播的乘加次数：DCM 一阶 ≈ 9 次乘加 + 9 次加法；
      四元数一阶 ≈ 16 次乘法。在 C 里是**同一量级**，DCM 甚至略少。
    · 但 DCM **必须**再加正交归一化（约 3 次 sqrt + 20 次乘加），
      四元数只需 1 次 sqrt 归一化。**差距主要在这里。**
    · 更关键的是**可读性**：四元数只有 4 个数、1 个约束，
      「归一化」是一个物理意义明确的动作；DCM 有 9 个数、6 个约束，
      正交归一化是一堆难以直观看懂的技巧。
      这也解释了为什么 PX4 干脆**完全没有** DCM 实现，
      而 ArduPilot 只在 AP_AHRS_DCM 这个可切换的后端里保留它。
""")


# ============================================================================
#  9. 选型速查
# ============================================================================
def summary():
    head("选型速查：四元数 vs DCM")
    table([
        ["状态量个数", "4（1 个约束）", "9（6 个约束）"],
        ["保持有效性的代价", "除以模长（1 次 sqrt）", "对称正交化 + 3 次归一化 + 叉积重建"],
        ["数值漂移", "慢，只漂模长", "快，行列式和正交性同时漂"],
        ["圆锥误差", "需专门补偿（本课 1-03 节）", "直接维护正交矩阵，抗一部分"],
        ["可读性", "抽象，需要习惯", "直观：每行就是一个轴的方向"],
        ["万向锁", "无", "无（但转欧拉角时有）"],
        ["典型实现", "PX4 EKF2 / attitude_estimator_q；ArduPilot EKF3",
         "ArduPilot AP_AHRS_DCM；MatrixPilot libDCM"],
        ["今天还用在哪", "主流，所有新代码",
         "ArduPilot 的备用姿态源（AHRS_EKF_TYPE=0）、MatrixPilot（至今仍是 DCM）"],
    ], ["维度", "四元数", "DCM"], [16, 30, 44])
    print("""
  ★ PX4 完全没有 DCM 实现 —— 只有 attitude_estimator_q（四元数）和 EKF2。
    「DCM 是另一条路线」这句话在 PX4 里不成立，只在 ArduPilot / MatrixPilot 里成立。
""")


# ============================================================================
#  主程序
# ============================================================================
def main():
    print("=" * 78)
    print("  飞控学习教程 · 第 3.6 周 · DCM 路线：旋转矩阵姿态解算")
    print("  配套验证代码 —— 8 个实验")
    print("=" * 78)
    t_start = time.perf_counter()

    exp_1_orthogonality_drift()
    exp_2_orthonormalization()
    exp_3_poisson_sign()
    exp_4_accel_sign_convention()
    exp_5_pitch_rate_earth()
    exp_6_complementary_filter_6dof()
    exp_7_complementary_filter_9dof()
    exp_8_dcm_vs_quaternion()
    summary()

    print()
    print("=" * 78)
    print("  全部实验完成，总耗时 %.1f s" % (time.perf_counter() - t_start))
    print("=" * 78)


if __name__ == "__main__":
    main()
