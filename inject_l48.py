# -*- coding: utf-8 -*-
"""反向注入：确认 lesson48（RealFlight/FlightAxis 桥接）的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson48_realflight.py')
TMP = os.path.join(tempfile.gettempdir(), 'l48_mut.py')

INJECTIONS = [
    # ---- [1] 结构与定位 ----
    ('J1  把桥接文件说成带气动系数（谎称它是模型）',
     "check('c_lift_0' not in src_fa and 'c_drag' not in src_fa,",
     "check('c_lift_0' not in src_fa and 'c_drag' in src_fa,",
     ['一个气动系数都没有'], "'c_drag' in src_fa,"),

    # ---- [2] 传输层 ----
    ('J2  端口说成 18084',
     "check(re.search(r'uint16_t controller_port = 18083;', src_fah) is not None,",
     "check(re.search(r'uint16_t controller_port = 18084;', src_fah) is not None,",
     ['端口'], "controller_port = 18084;'"),

    ('J3  把 TCP 说成 UDP',
     "check('auto *sck = NEW_NOTHROW SocketAPM_native(false);' in src_fa,",
     "check('auto *sck = NEW_NOTHROW SocketAPM_native(true);' in src_fa,",
     ['TCP，不是 UDP'], "SocketAPM_native(true);' in src_fa,"),

    ('J4  超时说成秒（10 → 10000）',
     "check('sck->connect_timeout(controller_ip, controller_port, 10)' in src_fa,",
     "check('sck->connect_timeout(controller_ip, controller_port, 10000)' in src_fa,",
     ['10 毫秒'], "controller_port, 10000)' in src_fa,"),

    ('J5  通道掩码说成 11 位',
     "check(4095 == 0xFFF and bin(4095).count('1') == 12,",
     "check(4095 == 0xFFE and bin(4095).count('1') == 12,",
     ['正好 12 路'], "4095 == 0xFFE"),

    # ---- [3] 协议 ----
    ('J6  SOAP 动作少一个',
     "check(actions == ['RestoreOriginalControllerDevice', 'ResetAircraft',",
     "check(actions == ['RestoreOriginalControllerDevice',",
     ['四个 SOAP 动作'], "check(actions == ['RestoreOriginalControllerDevice',"),

    ('J7  请求体占位符说成 11 个',
     "check(n_fmt == 12,",
     "check(n_fmt == 11,",
     ['请求体里正好'], "n_fmt == 11,"),

    ('J8  HTTP 载荷说成二进制',
     "check(\"content-type: text/xml;charset='UTF-8'\" in env,",
     "check(\"content-type: application/octet-stream\" in env,",
     ['XML 文本'], "application/octet-stream\" in env,"),

    # ---- [4] ★ 坐标矩阵（核心） ----
    ('J9  position 矩阵漏掉 Z 的负号',
     "M_pos   = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]",
     "M_pos   = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]",
     ['det(M_pos) = +1'], "M_pos   = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]"),

    ('J10 声称 velocity 与 wind 用同一映射',
     "M_vel   = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]    # (U, V, W)                 → **原样照抄**",
     "M_vel   = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]    # (U, V, W)                 → **原样照抄**",
     ['原样照抄'], "M_vel   = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]"),

    ('J11 声称 gyro 与 quaternion 同映射',
     "M_gyro  = [[1, 0, 0], [0, 1, 0], [0, 0, -1]]   # (roll, pitch, −yaw)       → **只有 Z 取负**",
     "M_gyro  = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]   # (roll, pitch, −yaw)       → **只有 Z 取负**",
     ['但 `gyro` 用的是'], "M_gyro  = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]"),

    ('J12 声称 position 与 quaternion 映射不同',
     "M_quat  = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]   # (W, Y, X, −Z) 的虚部      → 同上",
     "M_quat  = [[1, 0, 0], [0, 1, 0], [0, 0, -1]]   # (W, Y, X, −Z) 的虚部      → 同上",
     ['`position` 与 `quaternion` 用的是'], "M_quat  = [[1, 0, 0], [0, 1, 0], [0, 0, -1]]"),

    ('J13 漏掉 Z 负号却还声称 det=+1（反向对照那条）',
     "check(abs(det(M_broken) - (-1.0)) < 1e-12,",
     "check(abs(det(M_broken) - (1.0)) < 1e-12,",
     ['反向对照：若漏掉那个'], "det(M_broken) - (1.0)"),

    ('J14 声称对合性不成立（把 M·M 说成 −I）',
     "check(matmul(M_pos, M_pos) == [[1, 0, 0], [0, 1, 0], [0, 0, 1]],",
     "check(matmul(M_pos, M_pos) == [[-1, 0, 0], [0, -1, 0], [0, 0, -1]],",
     ['对合'], "matmul(M_pos, M_pos) == [[-1, 0, 0]"),

    ('J15 把 ENU→NED 说成不是（矩阵改一位）',
     "ENU_to_NED = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]",
     "ENU_to_NED = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]",
     ['ENU → NED 变换'], "ENU_to_NED = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]"),

    ('J16 声称通用风模型不取反（把存在说成不存在）',
     "check('// the AHRS wants wind with opposite sense' in src_gen and 'wind_ef = -wind_ef;' in src_gen,",
     "check('// the AHRS wants wind with opposite sense' in src_gen and 'wind_ef = -wind_ef;' not in src_gen,",
     ['通用** SITL 路径末尾'], "'wind_ef = -wind_ef;' not in src_gen,"),

    # ---- [5] 谁算什么 ----
    ('J17 声称皮托管空速直接来自 RealFlight',
     "check('fdm.airspeed = airspeed_pitot;' in src_ac,",
     "check('fdm.airspeed = airspeed;' in src_ac,",
     ['真正送进飞控'], "fdm.airspeed = airspeed;' in src_ac,"),

    ('J18 声称空气密度由 RealFlight 提供',
     "    check(kw not in src_fa,",
     "    check(kw in src_fa,",
     ['零匹配'], "check(kw in src_fa,"),

    # ---- [6][7] 时间轴与 IMU ----
    ('J19 把 use_time_sync 说成 true',
     "check('use_time_sync = false;' in src_fa,",
     "check('use_time_sync = true;' in src_fa,",
     ['关掉基类的墙钟同步'], "use_time_sync = true;' in src_fa,"),

    ('J20 请求频率说成 500 Hz',
     "check('rate_hz = 250 / target_speedup;' in src_fa,",
     "check('rate_hz = 500 / target_speedup;' in src_fa,",
     ['请求频率'], "rate_hz = 500 /"),

    ('J21 声称 IMU 由定时器驱动（不是按帧）',
     "check('flightaxis_sync_imus_to_frames = true;' in src_fa,",
     "check('flightaxis_sync_imus_to_frames = false;' in src_fa,",
     ['flightaxis_sync_imus_to_frames = true'], "flightaxis_sync_imus_to_frames = false;'"),

    # ---- [8] 参数 ----
    ('J22 OPTS 默认值说成 0',
     "check(re.search(r'AP_GROUPINFO\\(\"OPTS\", 1, FlightAxis, _options, uint32_t\\(Option::ResetPosition\\)\\)', src_fa) is not None,",
     "check(re.search(r'AP_GROUPINFO\\(\"OPTS\", 1, FlightAxis, _options, 0\\)', src_fa) is not None,",
     ['默认值 **不是 0'], "_options, 0\\)', src_fa)"),

    ('J23 声称有 flightaxis.parm 文件',
     "check(parms == [],",
     "check(parms != [],",
     ['没有** `flightaxis*.parm`'], "parms != [],"),

    ('J24 AHRS_EKF_TYPE 说成 3',
     "check(('AHRS_EKF_TYPE', '10') in sd,",
     "check(('AHRS_EKF_TYPE', '3') in sd,",
     ['SIM 专用 EKF 类型'], "('AHRS_EKF_TYPE', '3')"),

    # ---- [9] 未读字段 ----
    ('J25 未读字段数说成 16',
     "check(len(unused) == 17,",
     "check(len(unused) == 16,",
     ['个从不被读取'], "len(unused) == 16,"),

    # ---- [10] 启动路径 ----
    ('J26 声称 sim_vehicle.py 里有 flightaxis 分支',
     "check('flightaxis' not in src_sv,",
     "check('flightaxis' in src_sv,",
     ['没有 `flightaxis` 这个词'], "'flightaxis' in src_sv,"),

    ('J27 声称 xplane/flightaxis 分支把 sitl-port 置 True',
     r"""r'ret\["model"\]\.find\("flightaxis"\) != -1\):\s*\n\s*ret\["sitl-port"\] = (\w+)', src_vi)""",
     r"""r'ret\["model"\]\.find\("flightaxis"\) != -1\):\s*\n\s*ret\["sitl-port"\] = (True)', src_vi)""",
     ['对 xplane/flightaxis 的特判'], "= (True)', src_vi)"),

    # ---- [11] 后端对照 ----
    ('J28 外部后端数说成 12',
     "check(n_exist == 13,",
     "check(n_exist == 12,",
     ['个**外部模拟器后端'], "n_exist == 12,"),

    # ---- [12] 热气流 ----
    ('J29 高斯 e^-1 衰减说成 e^-2',
     "check(abs(W * math.exp(-(Rr ** 2) / (Rr ** 2)) - W / math.e) < 1e-12,",
     "check(abs(W * math.exp(-(Rr ** 2) / (Rr ** 2)) - W / (math.e ** 2)) < 1e-12,",
     ['偏出一个半径'], "W / (math.e ** 2))"),

    ('J30 初始半径对质说成 40',
     "check('INITIAL_THERMAL_RADIUS = 80.0' in src_soar,",
     "check('INITIAL_THERMAL_RADIUS = 40.0' in src_soar,",
     ['跨文件对质'], "INITIAL_THERMAL_RADIUS = 40.0'"),

    ('J31 声称源码里的参数名自带 SIM_ 前缀',
     "r'AP_GROUPINFO\(\"THML_SCENARI\",\s*12,\s*SIM,\s*thermal_scenario,\s*0\)'",
     "r'AP_GROUPINFO\(\"SIM_THML_SCENARI\",\s*12,\s*SIM,\s*thermal_scenario,\s*0\)'",
     ['不带 `SIM_` 前缀'], 'AP_GROUPINFO\("SIM_THML_SCENARI"'),
]


def run(path):
    p = subprocess.run([sys.executable, '-u', path],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=900)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


base = io.open(SRC, encoding='utf-8').read()
rc, out = run(SRC)
print('基线（未注入）: EXIT=%s  %s' % (rc, 'GREEN' if rc == 0 else 'RED'))
if rc != 0:
    print(out[-3000:])
    sys.exit(1)
print('基线断言数 = %d' % (out.count('[OK]') - 1))
print()

fails = 0
for name, old, new, expect_red, must_appear in INJECTIONS:
    if base.count(old) != 1:
        print('[SKIP] %-46s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-46s 注入后 grep 不到' % name)
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-46s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:24] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:5]:
            print('         red> %s' % l[:106])
        if crashed:
            print('         err> %s' % out.strip().splitlines()[-1][:106])

print()
print('=' * 70)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
