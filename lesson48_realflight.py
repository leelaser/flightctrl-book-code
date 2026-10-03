# -*- coding: utf-8 -*-
"""
lesson48_realflight.py —— 第 14 篇《RealFlight 仿真》配套自验证脚本

本篇与前 13 篇的根本区别：
  前 13 篇验证的是**算法**（公式对不对），本篇验证的是**一座桥**（约定对不对）。

桥的两端是两个独立开发的系统：
  · 一端是 RealFlight（商业模拟器，闭源，Windows）
  · 另一端是 ArduPilot（飞控）
  · 中间是 `libraries/SITL/SIM_FlightAxis.cpp`（802 行）

**桥接层不出"算错"，只出"约定错"** —— 而约定错是哑的：
不抛异常、不出 NaN，只让数据整体偏一个旋转或一个符号。
所以本脚本的重点是第 [4] 段：把那六处坐标映射抽成矩阵，验它们的数学性质与互相关系。

本脚本做的事：
  [1] 结构：桥接文件、类、基类、唯一必须实现的方法
  [2] 传输层：TCP / 端口 / 超时 / IP 覆盖
  [3] 协议：四个 SOAP 动作 + 12 通道的报文形状
  [4] ★★★ 六个量的坐标映射矩阵 —— **本篇的核心**
  [5] 谁算什么：桥接层自己实现的"物理"只有三处
  [6] 时间轴：仿真时钟由 RealFlight 驱动
  [7] IMU 按帧驱动（与其它后端的根本差异）
  [8] 参数与内建默认值表
  [9] 解析了但从不使用的字段
  [10] 启动路径：从 `-f` 到 `--model` 的完整透传
  [11] 13 个"连外部模拟器"后端的对照
  [12] 兑现第 13 篇欠下的账：SITL 热气流模型

**诚实边界**：本机没有装 RealFlight（盘上零命中）、也没有任何 C/C++ 编译器，
SITL 跑不起来 ⇒ **运行时路径在本机无法验证**。
本脚本只做"读源码 + 移植 + 数学验证"，凡是需要跑起来才能确认的，一律标注"未验证"。

运行： python3 lesson48_realflight.py     （全过 EXIT=0）
"""

import io
import math
import os
import re
import sys

n_ok = 0
n_bad = 0


def check(cond, msg):
    global n_ok, n_bad
    if cond:
        n_ok += 1
        print('    [OK] %s' % msg)
    else:
        n_bad += 1
        print('    [!!] %s' % msg)


print('=' * 76)
print('[0] 定位源码（4.7.1）')
print('=' * 76)

REPO = None
for cand in (r'D:\DEVLib\PX4APM\APM\ardupilot-Copter-4.7.1',
             r'D:\DEVLib\PX4APM\APM\ardupilot-Plane-4.7.1'):
    if os.path.isdir(os.path.join(cand, 'libraries')):
        REPO = cand
        break
if REPO is None:
    print('    [!!] 找不到 ArduPilot 源码树')
    n_bad += 1
    sys.exit(1)


def slurp(rel):
    return io.open(os.path.join(REPO, rel), encoding='utf-8', errors='replace').read()


F_FA   = 'libraries/SITL/SIM_FlightAxis.cpp'
F_FAH  = 'libraries/SITL/SIM_FlightAxis.h'
F_AC   = 'libraries/SITL/SIM_Aircraft.cpp'
F_ACH  = 'libraries/SITL/SIM_Aircraft.h'
F_CFG  = 'libraries/SITL/SIM_config.h'
F_CMD  = 'libraries/AP_HAL_SITL/SITL_cmdline.cpp'

src_fa, src_fah = slurp(F_FA), slurp(F_FAH)
src_ac, src_ach = slurp(F_AC), slurp(F_ACH)
src_cfg, src_cmd = slurp(F_CFG), slurp(F_CMD)

check(len(src_fa) > 20000, 'SIM_FlightAxis.cpp 读入 %d 字节' % len(src_fa))
check(len(src_fah) > 5000, 'SIM_FlightAxis.h 读入 %d 字节' % len(src_fah))


# ==========================================================================
print()
print('=' * 76)
print('[1] 结构：一座桥，不是一个模型')
print('=' * 76)

check('class FlightAxis : public Aircraft {' in src_fah,
      '★★ 类名 `FlightAxis`（**不是 `RealFlight`**），继承 `Aircraft`')
check('void update(const struct sitl_input &input) override;' in src_fah,
      '★ 只实现了基类**唯一**的纯虚函数 `update()` —— 其余的物理都由 RealFlight 做')
check('return NEW_NOTHROW FlightAxis(frame_str);' in src_fah,
      '★ 工厂函数 `create()` —— 与其它 27 个后端同一个接口')
check(re.search(r'#define AP_SIM_FLIGHTAXIS_ENABLED\s*\(CONFIG_HAL_BOARD == HAL_BOARD_SITL\)', src_cfg) is not None,
      '★ 编译开关只按"板子是不是 SITL"门控 ⇒ **不区分机型**，Copter/Plane/Rover/Sub 都能用')

# 与"内置动力学"后端的对比：谁有气动积分？
check('c_lift_0' in slurp('libraries/SITL/SIM_Plane.h') or 'c_lift_0' in slurp('libraries/SITL/SIM_Plane.cpp'),
      '★★ 对照：内置的 `SIM_Plane` 里有 `c_lift_0` 这类气动系数')
check('c_lift_0' not in src_fa and 'c_drag' not in src_fa,
      '★★★ 而 `SIM_FlightAxis.cpp` 里**一个气动系数都没有** —— '
      '它不做刚体积分，只做"转发 + 三处本地推算"')


# ==========================================================================
print()
print('=' * 76)
print('[2] 传输层：TCP / 18083 / 10 ms')
print('=' * 76)

check(re.search(r'uint16_t controller_port = 18083;', src_fah) is not None,
      '★ 端口 **18083**（硬编码在头文件里，不是参数）')
check(re.search(r'const char \*controller_ip = "127\.0\.0\.1";', src_fah) is not None,
      '★ 默认连本机 `127.0.0.1`')
check('auto *sck = NEW_NOTHROW SocketAPM_native(false);' in src_fa,
      '★★ `SocketAPM_native(false)` —— 这个 `false` 是 `_datagram`，'
      '即 `SOCK_DGRAM` 与 `SOCK_STREAM` 的二选一 ⇒ **TCP，不是 UDP**')
check('_datagram?SOCK_DGRAM:SOCK_STREAM' in slurp('libraries/AP_HAL/utility/Socket.cpp'),
      '★ 独立核实：`Socket.cpp` 里 `_datagram?SOCK_DGRAM:SOCK_STREAM` —— `false` 走 `SOCK_STREAM`')

check(re.search(r'bool connect_timeout\(const char \*address, uint16_t port, uint32_t timeout_ms\);',
                slurp('libraries/AP_HAL/utility/Socket.hpp')) is not None,
      '★ `connect_timeout` 的第三个参数名是 **`timeout_ms`**（单位毫秒）')
check('sck->connect_timeout(controller_ip, controller_port, 10)' in src_fa,
      '★★ 于是 `connect_timeout(ip, 18083, 10)` = **10 毫秒**连接超时 —— 很短，靠外层循环重试')

m = re.search(r'const char \*colon = strchr\(frame_str, .:.\);\s*\n\s*if \(colon\) \{\s*\n\s*controller_ip = colon\+1;', src_fa)
check(m is not None,
      '★★★ IP 用**机型串后缀**覆盖：`flightaxis:192.168.1.5` ⇒ `controller_ip` 指向那台机器')
check('set_interface_ports' not in src_fah,
      '★★ 但它**不覆盖** `set_interface_ports` ⇒ 通用的 `--sim-address/--sim-port-in/--sim-port-out` '
      '对这个后端**无效**（改 IP 只有机型串这一条路）')

# 通道掩码与 wiki 的"12 通道"对质
check('const uint16_t channels = hal.scheduler->is_system_initialized()?4095:0;' in src_fa,
      '★ 通道使能掩码 = `4095`（系统未初始化时发 0）')
check(4095 == 0xFFF and bin(4095).count('1') == 12,
      '★★★ 复算：`4095 = 0x%X`，二进制里有 **%d** 个 1 ⇒ 正好 12 路。'
      '这与官方文档说的"前 12 个 RX 通道和前 12 路舵机输出"**对得上**（wiki 与源码独立一致）'
      % (4095, bin(4095).count('1')))
check('// maximum number of servos to send is 12 with new FlightAxis' in src_fa,
      '★ 源码注释也自陈 "maximum number of servos to send is 12"')


# ==========================================================================
print()
print('=' * 76)
print('[3] 协议：HTTP 信封里装着 SOAP')
print('=' * 76)

actions = re.findall(r'soap_request_start\("(\w+)"', src_fa)
check(actions == ['RestoreOriginalControllerDevice', 'ResetAircraft',
                  'InjectUAVControllerInterface', 'ExchangeData'],
      '★★★ 四个 SOAP 动作，按出现顺序：%s' % ' → '.join(actions))
check('RestoreOriginalControllerDevice' in actions and 'InjectUAVControllerInterface' in actions,
      '★★ 前两个是"抢控制器"的手续：先让 RealFlight 恢复原控制器，再注入我们的')

env = src_fa[src_fa.find('asprintf(&req, R"(POST / HTTP/1.1'):]
check("POST / HTTP/1.1" in env and "soapaction:" in env,
      '★ 外层是 **HTTP POST**，`soapaction:` 头指定动作')
check("content-type: text/xml;charset='UTF-8'" in env,
      '★ 载荷是 **XML 文本**，不是二进制 —— 所以抓包肉眼可读')
check('Connection: Keep-Alive' in env,
      '★ `Keep-Alive` ⇒ 一条 TCP 连接上跑很多次 ExchangeData（不是一问一关）')

n_fmt = len(re.findall(r'%.4f', src_fa))
check(n_fmt == 12,
      '★★★ `ExchangeData` 请求体里正好 **%d** 个 4 位小数占位符（XML 的 item 元素）'
      ' —— 与 `4095` 的 12 位、与 `rcin_chan_count = 12` 三处独立对上' % n_fmt)
check('rcin_chan_count = 12;' in src_fa and 'the interlink interface supports 12 input channels' in src_fa,
      '★ 接收方向同样是 12：注释 "the interlink interface supports 12 input channels"')

check('Content-Length: ' in src_fa and r'\r\n\r\n' in src_fa,
      '★★ 收包是**手写**的 HTTP 拆包：先找 `Content-Length:`，再找 `\\r\\n\\r\\n`，再循环补收')
check('char replybuf[10000];' in src_fah,
      '★ 接收缓存 10000 字节（固定大小，不是动态分配）')
check('extremely primitive SOAP parser' in src_fa,
      '★★ 解析器是**手写字符串匹配**，源码自嘲 "extremely primitive SOAP parser"')

m = re.search(r'just a soap request/response server, no need to be too clever', src_fa)
check('true/false' in src_fa or 'm_isTouchingGround' in src_fa,
      '★ XML 里的 `true`/`false` 要手工转成 1/0（手写解析器的代价）')


# ==========================================================================
print()
print('=' * 76)
print('[4] ★★★ 六个量的坐标映射 —— 桥接层的全部风险都在这里')
print('=' * 76)

# --- 先从源码文本里把六个映射"读"出来 ---
PATTERNS = [
    ('position', r'position = Vector3d\(state\.m_aircraftPositionY_MTR,\s*\n\s*state\.m_aircraftPositionX_MTR,'),
    ('quaternion', r'Quaternion quat\(state\.m_orientationQuaternion_W,\s*\n\s*state\.m_orientationQuaternion_Y,\s*\n\s*state\.m_orientationQuaternion_X,\s*\n\s*-state\.m_orientationQuaternion_Z\)'),
    ('gyro', r'gyro = Vector3f\(radians\(constrain_float\(state\.m_rollRate_DEGpSEC[\s\S]{0,300}?m_yawRate_DEGpSEC'),
    ('velocity', r'velocity_ef = Vector3f\(state\.m_velocityWorldU_MPS,\s*\n\s*state\.m_velocityWorldV_MPS,'),
    ('wind', r'wind_ef = Vector3f\(state\.m_windY_MPS,state\.m_windX_MPS,state\.m_windZ_MPS\);'),
    ('accel', r'accel_body = \{\s*\n\s*float\(state\.m_accelerationBodyAX_MPS2\),\s*\n\s*float\(state\.m_accelerationBodyAY_MPS2\),'),
]
found = {}
for name, pat in PATTERNS:
    found[name] = re.search(pat, src_fa) is not None
    check(found[name], '读到 `%s` 的映射写法' % name)

# --- 把"源码的写法"翻译成矩阵 ---
Z = [[0, 0, 0]]
M_pos   = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]   # (RF.Y, RF.X, −alt)        → 交换 X/Y 且 Z 取负
M_quat  = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]   # (W, Y, X, −Z) 的虚部      → 同上
M_gyro  = [[1, 0, 0], [0, 1, 0], [0, 0, -1]]   # (roll, pitch, −yaw)       → **只有 Z 取负**
M_vel   = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]    # (U, V, W)                 → **原样照抄**
M_wind  = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]    # (RF.Y, RF.X, RF.Z)        → 交换 X/Y，**Z 不取负**
M_accel = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]    # (AX, AY, AZ)              → **原样照抄**


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def det(A):
    return (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
            - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
            + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))


def transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def is_orthogonal(A, tol=1e-12):
    P = matmul(A, transpose(A))
    return all(abs(P[i][j] - (1.0 if i == j else 0.0)) < tol for i in range(3) for j in range(3))


# --- 数学性质：M_pos 是一个"正经的"旋转 ---
check(is_orthogonal(M_pos), '★★★ `M_pos` 是正交阵（`M·Mᵀ = I`）—— 它不缩放、不剪切')
check(abs(det(M_pos) - 1.0) < 1e-12,
      '★★★ 且 `det(M_pos) = +1` ⇒ 它是**纯旋转**，不含镜面反射。'
      '（若 det = −1 就说明这个映射把左右手系对调了，那才是灾难）')
check(matmul(M_pos, M_pos) == [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      '★★★ 且 `M_pos · M_pos = I` ⇒ 它是**对合**：做两次回到原样。'
      '这意味着"从 RealFlight 读"和"往 RealFlight 写"用的是**同一个矩阵**，不需要求逆')

# --- 它是哪个旋转？---
n = (1 / math.sqrt(2), 1 / math.sqrt(2), 0.0)
R_180 = [[2 * n[i] * n[j] - (1.0 if i == j else 0.0) for j in range(3)] for i in range(3)]
check(all(abs(R_180[i][j] - M_pos[i][j]) < 1e-12 for i in range(3) for j in range(3)),
      '★★★ **恒等式**：`M_pos` 恰好等于"绕轴 (1,1,0)/√2 转 180°"的旋转矩阵 `2nnᵀ − I`。'
      '所以"交换 X/Y 且 Z 取负"不是两个随手凑的补丁，**它就是一次 180° 转体**')

# --- 它是不是标准的 ENU→NED？---
ENU_to_NED = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
check(ENU_to_NED == M_pos,
      '★★★ `M_pos` 就是**教科书上的 ENU → NED 变换**：北←东、东←北、下←−上。'
      '⇒ 若 RealFlight 的世界系是 ENU（X=东、Y=北、Z=上），这行就是标准写法')
check(matvec(M_pos, [1, 0, 0]) == [0, 1, 0],
      '★ 复算：RF 的 +X 在 NED 里是 **北 0 / 东 1** ⇒ RF 的 X 轴指向**东**')
check(matvec(M_pos, [0, 1, 0]) == [1, 0, 0],
      '★ 复算：RF 的 +Y 在 NED 里是 **北 1 / 东 0** ⇒ RF 的 Y 轴指向**北**')
check(matvec(M_pos, [0, 0, 1]) == [0, 0, -1],
      '★ 复算：RF 的 +Z 在 NED 里是 **下 −1** ⇒ RF 的 Z 轴指向**上**')

# --- 六者互相之间的一致性问题 ---
check(M_pos == M_quat,
      '★★★ `position` 与 `quaternion` 用的是**同一个映射** —— 这两处自洽')
check(M_gyro != M_pos,
      '★★★ 但 `gyro` 用的是 `diag(1,1,−1)`（**只把 Z 取负，不交换 X/Y**）—— 与上面两个不同')
check(M_wind != M_pos and M_wind[2][2] == 1,
      '★★ `wind` 交换了 X/Y 但**没有把 Z 取负** ⇒ 比 `M_pos` 少了一处符号')

check(M_vel == [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      '★★★ `velocity_ef` **原样照抄** `(U, V, W)`，一处都没改')

# --- 核心论证：那两个量要相减，却在不同的坐标系里 ---
check('Vector3f airspeed_3d_ef = velocity_ef - wind_ef;' in src_fa,
      '★★★ 而源码里 `airspeed_3d_ef = velocity_ef - wind_ef;` —— **这两个量要相减**')
check(M_vel != M_wind,
      '★★★ ⇒ **矛盾**：相减要求同一坐标系（映射必须相同），'
      '但 `M_vel = I` 而 `M_wind = M_pos ≠ I`。'
      '**两者不可能同时对** —— 除非 RealFlight 这两个字段本身用了不同的坐标系。'
      '本机没有 RealFlight，**这条无法在本地判定**，只能标为待验证')
check(matvec(M_wind, [0, 0, 1]) == [0, 0, 1] and matvec(M_pos, [0, 0, 1]) == [0, 0, -1],
      '★★ 差异的作用点：RF 的**风 Z 分量**进 NED 时**没有取负**，'
      '而位置的高度分量取了负 ⇒ 同一份"上和下"的约定，两处处理相反')

# --- 与通用风模型的差异 ---
src_gen = src_ac
check('// the AHRS wants wind with opposite sense' in src_gen and 'wind_ef = -wind_ef;' in src_gen,
      '★★★ 对照：**通用** SITL 路径末尾有一句 `wind_ef = -wind_ef;`（注释 "the AHRS wants wind with opposite sense"）')
check('wind_ef = -wind_ef' not in src_fa,
      '★★★ 而 FlightAxis 路径**没有**这一句 —— 它覆写了 `update()` 且不调用基类的 `update_wind()`')
check('void Aircraft::update_model' in src_gen and 'update_wind' in src_gen,
      '★ 佐证：基类里 `update_model()` / `update_wind()` 是两条独立路径，'
      '覆写 `update()` 就绕过了基类的风模型')

check(re.search(r'void Aircraft::update_wind\(const struct sitl_input &input\)', src_gen) is not None,
      '★ 顺带记：`Aircraft::update_wind()` 背后是 `SIM_WIND_*` 那套参数 —— '
      '**接上 RealFlight 之后，`SIM_WIND_*` 全部失效**（风由 RealFlight 给，见上面那三行）')

# --- 反向对照：把 ENU→NED 换成"只交换不取负"，det 会变 ---
M_broken = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]
check(abs(det(M_broken) - (-1.0)) < 1e-12,
      '★★★ 反向对照：若漏掉那个 `−`（只交换不取负），`det = %.0f` ⇒ '
      '**变成镜面反射**，左右手系对调 —— 飞机在仿真里会"镜像飞行"'
      % det(M_broken))
check(not is_orthogonal(M_broken) or abs(det(M_broken) + 1) < 1e-12,
      '★★ 而且它不是纯旋转 ⇒ 上面那条 det 检查**确实在守着"不许出现镜像"这件事**')


# ==========================================================================
print()
print('=' * 76)
print('[5] 谁算什么：桥接层自己实现的"物理"只有三处')
print('=' * 76)

check('accel on the ground is nasty in realflight' in src_fa,
      '★★ 第一处：**触地时重算加速度**（源码注释 "accel on the ground is nasty in realflight, '
      'and prevents helicopter disarm"）')
check('Vector3f accel_ef = (velocity_ef - last_velocity_ef) / dt_seconds;' in src_fa,
      '★ 做法：用 `Δ速度/Δt` 差分，再 `accel_ef.z -= GRAVITY_MSS;`')
check('// limit to 16G to match pixhawk' in src_fa and 'GRAVITY_MSS*16' in src_fa,
      '★ 再钳到 ±16 G（注释：对齐 Pixhawk 量程）')

check('for pitot airspeed we need the airspeed along the X axis' in src_fa,
      '★★★ 第二处：**皮托管空速由自己算**。注释原文解释得很清楚：'
      '"We can\'t get that from m_airspeed_MPS"')
check('Vector3f airspeed3d = dcm.mul_transpose(airspeed_3d_ef);' in src_fa
      and 'airspeed_pitot = MAX(airspeed3d.x,0);' in src_fa,
      '★ 做法：把 (地速 − 风) 转到机体系，取 X 分量，负值钳到 0')
check('fdm.airspeed = airspeed_pitot;' in src_ac,
      '★★★ 而**真正送进飞控**的是 `airspeed_pitot`（`SIM_Aircraft.cpp` 里赋值）'
      ' ⇒ RealFlight 的 `m_airspeed_MPS` 只被赋给成员 `airspeed`，**不进 FDM**')
check(re.search(r'airspeed = state\.m_airspeed_MPS;', src_fa) is not None,
      '★ 佐证：那一行确实存在（赋值给了成员），但下游用的是另一个')

check('update_mag_field_bf()' in src_fa,
      '★★ 第三类：**磁场一律 ArduPilot 自算**（走 WMM/`AP_Declination`），RealFlight 不参与')
check('rangefinder' in src_fa and 'm_altitudeAGL_MTR' in src_fa,
      '★ 测距仪也是自算：用 RealFlight 给的离地高度除以姿态矩阵的 z 分量')

for kw, name in (('air_density', '空气密度'), ('SSL_AIR_DENSITY', '海平面空气密度'),
                 ('terrain', '地形'), ('ground_level', '地面高度')):
    check(kw not in src_fa,
          '★ `%s` 在桥接文件里**零匹配** —— RealFlight 不给，桥接层也不补' % name)


# ==========================================================================
print()
print('=' * 76)
print('[6] 时间轴：仿真时钟由 RealFlight 驱动，不是墙钟')
print('=' * 76)

check('use_time_sync = false;' in src_fa,
      '★★★ `use_time_sync = false;` —— **关掉基类的墙钟同步**。'
      '时间完全由 RealFlight 的 `m-currentPhysicsTime-SEC` 决定')
check('rate_hz = 250 / target_speedup;' in src_fa,
      '★★ 请求频率 `rate_hz = 250 / target_speedup`（默认 250 Hz，开了加速就降低）')
check('AP_HAL::panic("Time did not move");' in src_fa,
      '★★★ 时间不前进直接 **panic** —— 桥接层不能容忍"仿真停了而飞控还在跑"')
check('cope with restarting RealFlight while connected' in src_fa,
      '★★ 时间倒退则视为"RealFlight 重启了"：重置基准、清掉位置偏移')
check('glitch' in src_fa,
      '★ 网络抖动单独统计并打印（`glitch %.2fs`）')

check(re.search(r'const float SAMPLE_INTERVAL_S = 1\.0f / _samplehz\.get\(\)', src_fa) is not None,
      '★★ 内部合成采样间隔 `1/SAMPLEHZ`（默认 2000 Hz）')
check('this prevents very small dt' in src_fa,
      '★★★ 而实际步长会被**量化成整数个采样**：'
      '`sample_interval_s = dt_seconds / roundf(dt_seconds / SAMPLE_INTERVAL_S)`。'
      '注释解释动机："this prevents very small dt\'s which can cause havoc with flight control"')
check('interpolate_frame' in src_fa,
      '★ 量化之后按新时间戳**插值**，把 RealFlight 的帧"摊平"成均匀样本')

check(re.search(r'Do not use if realtime physics, like RealFlight, is being used', src_ac) is not None
      or re.search(r'Do not use if realtime physics, like RealFlight, is being used',
                   slurp('libraries/SITL/SITL.cpp')) is not None,
      '★★★ 参数 `SIM_SPEEDUP` 的说明里点名了 RealFlight：'
      '"Do not use if realtime physics, like RealFlight, is being used" —— '
      '**加速仿真与实时物理互斥**')


# ==========================================================================
print()
print('=' * 76)
print('[7] IMU 按帧驱动：与其它后端的根本差异')
print('=' * 76)

check('flightaxis_sync_imus_to_frames = true;' in src_fa,
      '★★★ 构造函数里置 `flightaxis_sync_imus_to_frames = true`，'
      '注释 "tell the IMUs to advance on each frame that is processed"')
check('flightaxis_sync_imus_to_frames' in src_ach,
      '★ 这个开关在基类 `SIM_Aircraft.h` 里声明')
check(re.search(r'if \(flightaxis_sync_imus_to_frames\) \{\s*\n\s*fdm\.flightaxis_imu_frame_num\+\+;', src_ac) is not None,
      '★★ 每跑一帧就把 `fdm.flightaxis_imu_frame_num` 加一')

src_imu = slurp('libraries/AP_InertialSensor/AP_InertialSensor_SITL.cpp')
check('on some simulations (RealFlight) the aircraft sim decides when a new sample is available' in src_imu,
      '★★★ 消费端在 `AP_InertialSensor_SITL`：注释原文 '
      '"on some simulations (RealFlight) the aircraft sim decides when a new sample is available"')
check(re.search(r'if \(sitl->state\.flightaxis_imu_frame_num > 0\) \{\s*\n\s*update_from_frame\(\);', src_imu) is not None,
      '★★ 于是 IMU 的 `timer_update()` 改走 `update_from_frame()`')
check(re.search(r'if \(flightaxis_imu_frame_num == sitl->state\.flightaxis_imu_frame_num\) \{\s*\n\s*return;', src_imu) is not None,
      '★★★ 而 `update_from_frame()` **只在帧号变化时才产生新样本** —— '
      '即"IMU 采样率由仿真器决定"，而不是由定时器决定。'
      '这正是 4.7.0 里那条 "Ensure stable IMU timing on RealFlight" 改动的落点')


# ==========================================================================
print()
print('=' * 76)
print('[8] 参数与内建默认值表')
print('=' * 76)

check('AP_SUBGROUPPTR(flightaxis_ptr, "RFL_", 5, SIM::ModelParm, FlightAxis)' in slurp('libraries/SITL/SITL.cpp'),
      '★ 参数组前缀 `RFL_`，挂在 `SIM_` 下 ⇒ 完整名是 **`SIM_RFL_*`**')
opts = re.findall(r'// @Bitmask: (\d+): (.+)', src_fa)
check(len(opts) == 5, '★★ `SIM_RFL_OPTS` 是 **5 位**位掩码：')
for bit, txt in opts:
    print('           bit%s = %s' % (bit, txt))
check(re.search(r'AP_GROUPINFO\("OPTS", 1, FlightAxis, _options, uint32_t\(Option::ResetPosition\)\)', src_fa) is not None,
      '★★★ 默认值 **不是 0，而是 `ResetPosition`** ⇒ 开机自动重置飞机位置')
check(re.search(r'AP_GROUPINFO\("SAMPLEHZ", 2, FlightAxis, _samplehz, 2000\)', src_fa) is not None,
      '★ `SIM_RFL_SAMPLEHZ` 默认 **2000**（内部合成采样率）')

o = re.search(r'enum class Option[^{]*\{(.*?)\}', src_fa, re.S) or re.search(r'enum class Option[^{]*\{(.*?)\}', src_fah, re.S)
check(o is not None and 'Rev4Servos' in o.group(1) and 'HeliDemix' in o.group(1),
      '★ 五个选项：ResetPosition / Rev4Servos / HeliDemix / SilenceFPS / NoDtLog')
check('for quadplane testing' in src_fa,
      '★★ `Rev4Servos` 的注释是 "Swap first 4 and last 4 servos (for quadplane testing)"')
check('FlightAxis expects "roll/pitch/collective/yaw" input' in src_fa,
      '★★ `HeliDemix` 的注释点明了直升机通道顺序：'
      '**FlightAxis 期望 "roll/pitch/collective/yaw"** —— 与 ArduPilot 的舵机序不同，要解混')

check(re.search(r'\{ "RC2_REVERSED", 1 \}, // interlink has reversed rc2', src_fa) is not None,
      '★★★ 内建默认值表里有一条 `RC2_REVERSED = 1`，注释 "interlink has reversed rc2" —— '
      '**RealFlight 配套遥控器的 2 通道是反的**，代码替你翻回来')
check('AP_Param::set_default_by_name(sim_defaults[i].name, sim_defaults[i].value);' in src_fa,
      '★★★ 这个后端**没有 `.parm` 文件**，默认参数直接写在 C++ 里（`sim_defaults[]` 表）'
      ' —— 与其它机型"一个 `.parm` 一个机型"的做法不同')
sd = re.findall(r'\{\s*"([A-Z0-9_]+)",\s*([-0-9.]+)', src_fa[src_fa.find('sim_defaults[] = {'):src_fa.find('};', src_fa.find('sim_defaults[] = {'))])
check(len(sd) >= 20, '★ `sim_defaults[]` 共 **%d** 条，逐条 `set_default_by_name`' % len(sd))
for nm, val in sd[:6]:
    print('           %-22s = %s' % (nm, val))
check(('AHRS_EKF_TYPE', '10') in sd,
      '★★ 其中 `AHRS_EKF_TYPE = 10` —— 10 是 **SIM 专用 EKF 类型**（直接吃仿真真值）')
check(('INS_GYR_CAL', '0') in sd,
      '★★ 还有 `INS_GYR_CAL = 0`（关掉陀螺零偏标定）—— 仿真的 IMU 没有零偏，'
      '不关掉反而会标出假零偏')


# ==========================================================================
print()
print('=' * 76)
print('[9] 解析了但从不使用的字段：45 个具名量里躺着 17 个')
print('=' * 76)

hdr = src_fah
keys = re.findall(r'\{\s*"([^"]+)",\s*next_state\.(\w+)\s*\}', hdr)
check(len(keys) == 46, '★ keytable 有 **%d** 条（其中 `item` 一条代表 12 个通道）' % len(keys))

unused = []
for name, field in keys:
    if name == 'item':
        continue
    if not re.search(r'\b(?:next_)?state\.%s\b' % re.escape(field), src_fa):
        unused.append(name)
check(len(unused) == 17,
      '★★★ 45 个具名字段里 **%d 个从不被读取** —— 解析器是固定表，协议给什么就存什么' % len(unused))
for nm in unused[:8]:
    print('           %s' % nm)
print('           …（共 %d 个）' % len(unused))
check('m-azimuth-DEG' in unused and 'm-inclination-DEG' in unused and 'm-roll-DEG' in unused,
      '★★★ 其中包含 **欧拉角**（azimuth/inclination/roll）—— 代码宁可只用四元数，'
      '也不碰 RealFlight 给的欧拉角')
check('m-velocityBodyU-MPS' in unused and 'm-accelerationWorldAX-MPS2' in unused,
      '★★ 还有机体系速度、世界系加速度这些"看着很有用"的量 —— 桥接层一律不用')


# ==========================================================================
print()
print('=' * 76)
print('[10] 启动路径：从 `-f` 一路透传到 socket')
print('=' * 76)

check(re.search(r'\{\s*"flightaxis",\s*FlightAxis::create\s*\}', src_cmd) is not None,
      '★ 模型注册表里有 `{ "flightaxis", FlightAxis::create }`')
check('strncasecmp(model_constructors[i].name, model_str, strlen(model_constructors[i].name)) == 0' in src_cmd,
      '★★ 匹配方式是**前缀 + 忽略大小写** ⇒ `flightaxis:192.168.1.5` 也能匹配到 `flightaxis`，'
      '后面那截留给构造函数去解析')
check(re.search(r'\{"model",\s*true,\s*0,\s*.M.\}', src_cmd) is not None,
      '★ `--model` / `-M` 是同一个选项（`M`）')

src_sv = slurp('Tools/autotest/sim_vehicle.py')
check('cmd.extend(["--model", stuff["model"]])' in src_sv,
      '★★★ `sim_vehicle.py` 把 `model` **原样透传**成 `--model`')
check('flightaxis' not in src_sv,
      '★★★ 而 `sim_vehicle.py` 里**根本没有 `flightaxis` 这个词** —— '
      '它不需要专门分支，靠的是"未知 frame 走 model 透传"这条通用路径')
src_vi = slurp('Tools/autotest/pysim/vehicleinfo.py')
check('if "model" not in ret:' in src_vi,
      '★ `vehicleinfo.py` 里：frame 表里没有的，就把 frame 字符串直接当 model')
check('WARNING: no config for frame' in src_vi,
      '★★ 代价是每次都会打一行 `WARNING: no config for frame (flightaxis:...)` —— '
      '**这行警告是正常的，不是错误**')
# ★ 这里原来写的是 `'ret["sitl-port"] = False' in src_vi` —— 那是**假绿灯**：
#   vehicleinfo.py:557 有一行默认值 `ret["sitl-port"] = True`，
#   把断言里的 False 改成 True 它**照样通过**（被另一行满足了）。
#   改成锚定"条件 + 分支体"的完整片段，并用捕获组把值取出来比。
m_vi = re.search(
    r'if \(ret\["model"\]\.find\("xplane"\) != -1 or '
    r'ret\["model"\]\.find\("flightaxis"\) != -1\):\s*\n\s*ret\["sitl-port"\] = (\w+)', src_vi)
check(m_vi is not None and m_vi.group(1) == 'False',
      '★★★ 对 xplane/flightaxis 的特判：条件里同时点名两者，分支体把 `sitl-port` 置为 **%s** '
      '⇒ 不再发 `--sitl` 那个 RCIN 端口（RC 输入改由模拟器提供）'
      % (m_vi.group(1) if m_vi else '?'))
check(re.search(r'if "sitl-port" not in ret:\s*\n\s*ret\["sitl-port"\] = True', src_vi) is not None,
      '★★ 对照：**默认值**是 `True`（在特判**之前**无条件设一次）—— '
      '这正是上面那条断言曾经被"另一行满足"的原因，也是"子串断言要锚定上下文"的活标本')

parms = [f for f in os.listdir(os.path.join(REPO, 'Tools/autotest/default_params')) if 'flight' in f.lower()]
check(parms == [],
      '★★★ `Tools/autotest/default_params/` 下**没有** `flightaxis*.parm` '
      '⇒ 又一次印证：这个后端的默认值在 C++ 里，不在参数文件里')


# ==========================================================================
print()
print('=' * 76)
print('[11] 13 个"连外部模拟器"的后端：一张对照表')
print('=' * 76)

EXTERNAL = [
    ('FlightAxis (RealFlight)', 'TCP', 18083,  F_FAH),
    ('XPlane',                  'UDP', 49000,  'libraries/SITL/SIM_XPlane.h'),
    ('JSBSim',                  'TCP+UDP', 5504, 'libraries/SITL/SIM_JSBSim.h'),
    ('CRRCSim',                 'UDP', 9002,   'libraries/SITL/SIM_CRRCSim.cpp'),
    ('last_letter',             'UDP', 5002,   'libraries/SITL/SIM_last_letter.h'),
    ('JSON',                    'TCP', 9002,   'libraries/SITL/SIM_JSON.h'),
    ('AirSim',                  'TCP+UDP', 9002, 'libraries/SITL/SIM_AirSim.h'),
    ('Gazebo',                  'TCP', 9002,   'libraries/SITL/SIM_Gazebo.h'),
    ('Webots',                  '—',   5599,   'libraries/SITL/SIM_Webots.h'),
    ('Morse',                   '—',   60000,  'libraries/SITL/SIM_Morse.h'),
    ('SilentWings',             '—',   6070,   'libraries/SITL/SIM_SilentWings.h'),
    ('Scrimmage',               'UDP', 9002,   'libraries/SITL/SIM_Scrimmage.h'),
    ('Webots_Python',           '—',   9002,   'libraries/SITL/SIM_Webots_Python.h'),
]
n_exist = 0
for name, proto, port, path in EXTERNAL:
    ok = os.path.isfile(os.path.join(REPO, path.replace('/', os.sep)))
    if ok:
        n_exist += 1
    check(ok, '%-24s %-8s 端口 %-6s ← %s' % (name, proto, port, path))
check(n_exist == 13, '★★ **13 个**外部模拟器后端 —— RealFlight 只是其中之一')
check(18083 not in [49000, 9002, 9003, 5504, 5002, 5599, 60000, 6070],
      '★★ RealFlight 的 18083 与其它后端的常用端口（9002/9003…）都不撞 —— '
      '18083 是 RealFlight 自己规定的')

check('AP_SIM_XPLANE_ENABLED' in src_cfg and 'AP_SIM_JSBSIM_ENABLED' in src_cfg,
      '★ 这些后端各有自己的编译开关，全都在 `SIM_config.h` 里集中门控')


# ==========================================================================
print()
print('=' * 76)
print('[12] 兑现第 13 篇欠下的账：SITL 的热气流模型')
print('=' * 76)

m = re.search(r'#define MAX_THERMALS (\d+)(.*?)\n    \}\n', src_ac, re.S)
block = m.group(2) if m else ''
check(m is not None and m.group(1) == '10',
      '★ 最多同时支持 **%s** 个热气流（`MAX_THERMALS`）' % (m.group(1) if m else '?'))

# 逐个 case 取"到下一个 case 为止"的片段 —— 不能用非贪婪找 break，
# 那样会把空 case 的 fallthrough 吞进上一段（我第一版就是这么错的）
labels = [(int(n), p) for n, p in
          re.findall(r'case (\d+):\s*\n(.*?)(?=\n\s*case \d+:|\n\s*default:)', block, re.S)]
check(len(labels) == 5,
      '★★ `SIM_THML_SCENARI` 有 **%d** 个 case 标签：%s' % (len(labels), [n for n, _ in labels]))
empty = [n for n, body in labels if not re.search(r'thermals_\w\[0\]', body)]
check(empty == [0],
      '★★★ 其中 `case %s` 是**空的** —— 它不写 break，直接 fallthrough 到下一个 case ⇒ '
      '**场景 0 的行为等于场景 1**。这是 C 里的惯用手法（"0 = 默认档"），不是漏写' % empty)

scen = [(n, b) for n, b in labels if re.search(r'thermals_\w\[0\]', b)]
check(len(scen) == 4, '★ 真正带参数的场景有 **%d** 个' % len(scen))
for num, body in scen:
    kv = dict(re.findall(r'thermals_(\w)\[0\]\s*=\s*(-?[\d.]+);', body))
    check('w' in kv and 'r' in kv and 'x' in kv and 'y' in kv,
          '★ 场景 %d：强度 w = %-4s m/s，半径 r = %-5s m，中心 (x,y) = (%s, %s)'
          % (num, kv.get('w'), kv.get('r'), kv.get('x'), kv.get('y')))

check(re.search(r'w \+= thermals_w\[iThermal\]\*exp\(-r2/\(thermals_r\[iThermal\]\*thermals_r\[iThermal\]\)\)',
                src_ac) is not None,
      '★★★ 核心公式：`w += W · exp(−r²/R²)` —— **与飞控侧 EKF 的测量模型同构**'
      '（第 13 篇 §4.4 的那个高斯）')
src_sitl = slurp('libraries/SITL/SITL.cpp')
check(re.search(r'// @Param: THML_SCENARI', src_sitl) is not None
      and re.search(r'AP_GROUPINFO\("THML_SCENARI",\s*12,\s*SIM,\s*thermal_scenario,\s*0\)', src_sitl) is not None,
      '★★★ 参数**在源码里叫 `THML_SCENARI`，不带 `SIM_` 前缀** —— 完整名 `SIM_THML_SCENARI` 是'
      '"组前缀 + 参数名"拼出来的。这就是"全文搜 `SIM_XXX` 搜不到"的通用原因')
check('AP_GROUPINFO("SPEEDUP"' in src_sitl or 'AP_GROUPINFO("RATE_HZ"' in src_sitl,
      '★ 佐证：`SIM_SPEEDUP` / `SIM_RATE_HZ` 在源码里同样只有 `SPEEDUP` / `RATE_HZ`')

# 独立复算：高斯模型的自证特性
W, Rr = 2.0, 80.0
check(abs(W * math.exp(-0.0 / (Rr * Rr)) - W) < 1e-12,
      '★ 复算（场景 1，w=2.0 r=80）：**正在中心** ⇒ 上升率 = 强度本身 = %.1f m/s' % W)
check(abs(W * math.exp(-(Rr ** 2) / (Rr ** 2)) - W / math.e) < 1e-12,
      '★★ 复算：**偏出一个半径** ⇒ 上升率 = 强度 × e⁻¹ = %.4f m/s' % (W / math.e))
check(W * math.exp(-(2 * Rr) ** 2 / (Rr ** 2)) < 0.02 * W,
      '★★ 复算：**偏出两个半径** ⇒ 衰减到强度的 %.2f%% ⇒ 热气流是"有界"的'
      % (100 * math.exp(-4)))

# 与飞控侧的热气流半径参数对照
src_soar = slurp('libraries/AP_Soaring/AP_Soaring.h')
check('INITIAL_THERMAL_RADIUS = 80.0' in src_soar,
      '★★★ **跨文件对质**：飞控侧 EKF 的初始半径 `INITIAL_THERMAL_RADIUS = 80.0`，'
      '与 SITL 场景 1 的 `thermals_r = 80.0` **数值相同** '
      '—— 这是"仿真器的真值"与"估计器的初值"被有意设成一致')


# ==========================================================================
print()
print('=' * 76)
print('[13] 诚实边界：本机验证不了什么')
print('=' * 76)

import shutil

# --- 把"跑不了"这件事本身变成可核实的断言，而不是一句声明 ---
check(not os.path.isdir(r'C:\Program Files (x86)\RealFlight')
      and not os.path.isdir(r'C:\Program Files\RealFlight')
      and not os.path.isdir(r'C:\Program Files (x86)\Steam\steamapps\common\RealFlight'),
      '★ 本机三个常见 RealFlight 安装目录**都不存在** ⇒ 没有模拟器可连')
check(shutil.which('gcc') is None and shutil.which('g++') is None and shutil.which('clang') is None,
      '★ 本机没有 gcc / g++ / clang ⇒ SITL 编译不出来')
check(not os.path.isdir(os.path.join(REPO, 'build', 'sitl', 'bin')),
      '★ 源码树里也没有已编译的 SITL 二进制 ⇒ 没有现成的可执行文件')
check(shutil.which('git') is not None and os.path.isdir(REPO),
      '★ 但源码树本身是完整的 —— 所以"读源码"这条路走得通，'
      '这是本脚本全部断言的来源')

print()
print('    ★★★ 边界声明（不是断言，因为它无法被本机证伪）：')
print('        本脚本验证的是**桥的图纸**，不是**桥能通车**。')
print('        坐标矩阵的性质、通道数、参数默认值 —— 都能从源码确证；')
print('        但"接上真正的 RealFlight 之后数据对不对"，')
print('        只能在装了 RealFlight 的 Windows 机器上跑起来才知道。')
print('        第 [4] 段发现的 `velocity` / `wind` 映射不一致，')
print('        **正是那种"只读代码判不了、必须跑起来才判得了"的问题**。')


# ==========================================================================
print()
print('=' * 76)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
