# -*- coding: utf-8 -*-
"""反向注入：确认 lesson47 的断言【真的会红】。
每条注入 = 把脚本里某个事实/公式改错，然后要求指定的断言变红。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson47_special.py')
TMP = os.path.join(tempfile.gettempdir(), 'l47_mut.py')

INJECTIONS = [
    # ---- [2] 多边形 ----
    ('J1  射线法初值 true→false（奇偶基准反了）',
     "    outside = True\n    for i in range(n):",
     "    outside = False\n    for i in range(n):",
     ['独立基准：射线法'], "outside = False\n    for i in range(n):"),

    # ---- [5] 参数表 ----
    ('J2  FENCE_ 参数个数谎报 17',
     "check(len(params) == 16,",
     "check(len(params) == 17,",
     ['FENCE_ 共'], "== 17,"),

    ('J3  声称 Copter 也有 Terminate',
     "check('Terminate' in per_veh.get('Rover', ''),",
     "check('Terminate' in per_veh.get('Copter', ''),",
     ['只有 Rover 有 Terminate'], "per_veh.get('Copter', ''),"),

    # ---- [6] 注册位置 + 假绿灯 ----
    ('J4  把注册位置说成 ArduCopter',
     "m.group(3) == 'AP_Vehicle' and m.group(4) == 'AC_Fence',",
     "m.group(3) == 'ArduCopter' and m.group(4) == 'AC_Fence',",
     ['注册在 AP_Vehicle'], "m.group(3) == 'ArduCopter'"),

    ('J5  假绿灯证据：谎称 Copter 有 FENCE_',
     "check(('FENCE_' in _copter_txt) and (veh_hits['ArduCopter'] == []),",
     "check(('FENCE_' in _copter_txt) and (veh_hits['ArduCopter'] != []),",
     ['假绿灯证据'], "veh_hits['ArduCopter'] != []),"),

    # ---- [7] 围栏存储 ----
    ('J6  每载具点数说成各 8 点',
     "check(non_cop // 8 == 20 and cop // 8 == 6,",
     "check(non_cop // 8 == 8 and cop // 8 == 8,",
     ['每个点是'], "== 8 and cop // 8 == 8,"),

    ('J7  返航点：把 AABB 中点这条判反',
     "ret\\.x = \\(min_loc\\.x / 2\\) \\+ \\(max_loc\\.x / 2\\);', body) is not None,",
     "ret\\.x = \\(min_loc\\.x / 2\\) \\+ \\(max_loc\\.x / 2\\);', body) is None,",
     ['但代码算的是'], "body) is None,"),

    ('J8  溢出断言判反（说两者相等）',
     "check(i32(lon_e7 + lon_e7) // 2 != (lon_e7 // 2) + (lon_e7 // 2),",
     "check(i32(lon_e7 + lon_e7) // 2 == (lon_e7 // 2) + (lon_e7 // 2),",
     ['溢出后'], "== (lon_e7 // 2) + (lon_e7 // 2),"),

    # ---- [8] Terrain 磁盘格式 ----
    ('J9  恒等式：height 少算一行',
     "c_A = 8 + 4 + 4 + 2 + 2 + 2 + SZ_X * SZ_Y * 2 + 2 + 2 + 2 + 1",
     "c_A = 8 + 4 + 4 + 2 + 2 + 2 + (SZ_X - 1) * SZ_Y * 2 + 2 + 2 + 2 + 1",
     ['offsetof(grid_block, version_minor)'], "(SZ_X - 1) * SZ_Y * 2"),

    ('J10 恒等式：Python 打包丢掉 int8',
     "_struct.calcsize('<HHhb'))",
     "_struct.calcsize('<HHh'))",
     ['offsetof(grid_block, version_minor)'], "'<HHh'))"),

    ('J11 尾部填充说成 226',
     "check(2048 - c_A == 227,",
     "check(2048 - c_A == 226,",
     ['尾部填充'], "== 226,"),

    ('J12 AcceptOldData 算成 2（就是量具原来那个 bug）',
     "check(o is not None and o.get('AcceptOldData') == 4,",
     "check(o is not None and o.get('AcceptOldData') == 2,",
     ['复算：`AcceptOldData'], "== 2,"),

    # ---- [9] 网格与插值 ----
    ('J13 块重叠恒等式说成 5',
     "check(SZ_X - SP_X == 4 and SZ_Y - SP_Y == 4,",
     "check(SZ_X - SP_X == 5 and SZ_Y - SP_Y == 5,",
     ['块大小 − 块步进'], "== 5 and SZ_Y - SP_Y == 5,"),

    ('J14 插值把 x 的权重配到 h01/h11（串轴）',
     "    avg1 = (1.0 - fx) * h00 + fx * h10",
     "    avg1 = (1.0 - fx) * h00 + fx * h01",
     ['独立基准①（恒等式）'], "fx * h01"),

    ('J15 角点条件：frac=(1,1) 说成取 h10',
     "check(bilinear(10, 20, 30, 40, 0.0, 0.0) == 10 and bilinear(10, 20, 30, 40, 1.0, 1.0) == 40,",
     "check(bilinear(10, 20, 30, 40, 0.0, 0.0) == 10 and bilinear(10, 20, 30, 40, 1.0, 1.0) == 30,",
     ['角点条件'], "1.0, 1.0) == 30,"),

    ('J16 轴混：worst_x 拿 SP_Y 的上界去比',
     "check(worst_x == SP_X - 1 and worst_y == SP_Y - 1,",
     "check(worst_x == SP_Y - 1 and worst_y == SP_Y - 1,",
     ['且正好取到上界'], "worst_x == SP_Y - 1"),

    # ---- [10] Follow ----
    ('J17 Follow 说成用了卡尔曼滤波',
     "check('kalman' not in src_follow.lower() and 'ekf' not in src_follow.lower(),",
     "check('kalman' in src_follow.lower() or 'ekf' in src_follow.lower(),",
     ['没有 kalman'], "'kalman' in src_follow.lower()"),

    ('J18 ω×r 的解析期望漏乘 ω（手算错误重演）',
     "expect_wxr = (w[2] * (-r[1]), w[2] * r[0], 0.0)",
     "expect_wxr = (1.0 * (-r[1]), 1.0 * r[0], 0.0)",
     ['复算 ω×r'], "expect_wxr = (1.0 * (-r[1])"),

    # ---- [11] SpeedToFly ----
    ('J19 目标函数 J 掺 1% 误差',
     "    return stf_T(CL, Wxp) / stf_t1(CL, WZ, CD0, B)",
     "    return stf_T(CL, Wxp) / stf_t1(CL, WZ, CD0, B) * 1.01",
     ['独立基准（有限差分）'], "* 1.01"),

    ('J20 Jdd 末项 0.75→0.5',
     "            + 0.75 * Wxp / (_m.sqrt(CL) * t1))",
     "            + 0.50 * Wxp / (_m.sqrt(CL) * t1))",
     ['同上，Jdd 与数值二阶导'], "+ 0.50 * Wxp"),

    ('J21 反向对照失效：broken 版改回正确',
     "    t2 = 1.5 * _m.sqrt(CL) * WZ + 3.0 * B * CL          # ← 2.0 改成 3.0",
     "    t2 = 1.5 * _m.sqrt(CL) * WZ + 2.0 * B * CL          # ← 2.0 改成 3.0",
     ['反向对照：把 t2 的系数'], "+ 2.0 * B * CL          #"),

    # ---- [12] EKF ----
    ('J22 雅可比 ∂w/∂X1 的指数 3→2',
     "         2 * X[0] * (((X[2] - Px) ** 2 + (X[3] - Py) ** 2) / (X[1] ** 3)) * expon,",
     "         2 * X[0] * (((X[2] - Px) ** 2 + (X[3] - Py) ** 2) / (X[1] ** 2)) * expon,",
     ['解析雅可比 4 个分量与数值导数'], "/ (X[1] ** 2)) * expon,"),

    ('J23 一个半径处读数说成 e^-0.5',
     "check(abs(r_half - 2.0 * _m.exp(-1.0)) < 1e-12,",
     "check(abs(r_half - 2.0 * _m.exp(-0.5)) < 1e-12,",
     ['偏出**一个半径**'], "_m.exp(-0.5)"),

    # ---- [13] 极曲线 ----
    ('J24 最小下沉 CL 的根号里 3→2',
     "    return _m.sqrt(3.0 * CD0 / B)",
     "    return _m.sqrt(2.0 * CD0 / B)",
     ['数值扫描 20001 点得'], "_m.sqrt(2.0 * CD0 / B)"),

    ('J25 声称硬编码的 1.5 与极曲线无关 → 判反',
     "    check(abs(CL_star - 1.5) > 0.05,",
     "    check(abs(CL_star - 1.5) < 0.05,",
     ['而代码硬编码的是'], "abs(CL_star - 1.5) < 0.05,"),

    ('J26 50° 处的放大率判据收到 5%',
     "    check(abs(e) < 0.10,",
     "    check(abs(e) < 0.05,",
     ['φ=50°'], "abs(e) < 0.05,"),

    ('J27 盘旋转弯时间常数漏乘 2π',
     "    tau = 2 * _m.pi * V / (9.81 * _m.tan(phi))",
     "    tau = V / (9.81 * _m.tan(phi))",
     ['跨文件一致性'], "tau = V / (9.81"),

    ('J28 VAR.cl 说成被赋值过（计数 2→3）',
     "))) == 2,\n      '★★★ `_raw_climb_rate` 全库只出现",
     "))) == 3,\n      '★★★ `_raw_climb_rate` 全库只出现",
     ['全库只出现'], "))) == 3,"),

    ('J29 手动接管窗口按"秒"读（10000→1000）',
     "check(_ms == 10000.0 and re.search(",
     "check(_ms == 1000.0 and re.search(",
     ['飞手接管后的'], "_ms == 1000.0 and re.search("),

    ('J30 单位陷阱：把 millis 换成 micros',
     "r'\\(AP_HAL::millis\\(\\) - _manual_recovery_start_ms\\) < AC_FENCE_MANUAL_RECOVERY_TIME_MIN'",
     "r'\\(AP_HAL::micros\\(\\) - _manual_recovery_start_ms\\) < AC_FENCE_MANUAL_RECOVERY_TIME_MIN'",
     ['单位陷阱'], "AP_HAL::micros"),

    ('J31 ALT_MIN 强制排除那条判反',
     "m.group(1) == 'FENCE_' and m.group(3) == 'AP_Vehicle' and m.group(4) == 'AC_Fence',",
     "m.group(1) == 'FENCE_' and m.group(3) == 'AP_Vehicle' and m.group(4) != 'AC_Fence',",
     ['注册在 AP_Vehicle'], "m.group(4) != 'AC_Fence',"),
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
        print('[SKIP] %-44s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-44s 注入后 grep 不到' % name)
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-44s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:26] for h in hit)))
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
