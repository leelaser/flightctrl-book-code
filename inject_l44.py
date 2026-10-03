# -*- coding: utf-8 -*-
"""反向注入：确认 lesson44 的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson44_customctl.py')
TMP = os.path.join(tempfile.gettempdir(), 'l44_mut.py')

INJECTIONS = [
    ('J1  get_type 的 -1 映射去掉',
     "    return (ctype - 1) if ctype > 0 else 0",
     "    return ctype",
     ['运行时 get_type'], "return ctype"),

    ('J2  Empty 分支"其实能建 backend"',
     "    if ctype == ENUM.get('CONT_EMPTY'):\n        return False            # 整段被注释，落到 break",
     "    if ctype == ENUM.get('CONT_EMPTY'):\n        return True             # 整段被注释，落到 break",
     ['唯一能被 is_safe_to_run'], "return True             # 整段被注释"),

    ('J3  删掉 backend 为空那道门',
     "    if not backend_exists:\n        return False, 'Reboot to enable selected custom controller'",
     "    if False:\n        return False, 'Reboot to enable selected custom controller'",
     ['收到的是「Custom controller is ON」'], "if False:"),

    ('J4  角度环 0.9 容差改成永假',
     "check(all(abs(float(b) - 0.90) < 1e-9 for _, b in angle_args),",
     "check(all(abs(float(b) - 0.90) < -1 for _, b in angle_args),",
     ['角度环三个轴的缩放系数全是 0.90'], "< -1 for _, b"),

    ('J5  Hamilton 积 y 分量符号写反',
     "            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,",
     "            w1 * y2 + x1 * z2 + y1 * w2 + z1 * x2,",
     ['四元数自证'], "w1 * y2 + x1 * z2"),

    ('J6  qrot 漏掉那个 2 倍',
     "    ux, uy, uz = 2 * ux, 2 * uy, 2 * uz",
     "    ux, uy, uz = ux, uy, uz",
     ['旋转保长'], "= ux, uy, uz"),

    ('J7  推力轴漏掉 attitude_body.inverse()',
     "    cross = qrot(qinv(att_body), cross)          # 把旋转轴表达回机体系",
     "    cross = qrot(att_body, cross)                # 把旋转轴表达回机体系",
     ['定义性基准 ①'], "qrot(att_body, cross)"),

    ('J8  to_axis_angle 去掉 wrap_PI',
     "        ang = wrap_pi(2.0 * math.atan2(l, q[0]))",
     "        ang = 2.0 * math.atan2(l, q[0])",
     ['③'], "ang = 2.0 * math.atan2"),

    ('J9  sqrt_controller 的 linear_dist 写错',
     "        linear_dist = second_ord_lim / (p * p)",
     "        linear_dist = second_ord_lim / p",
     ['往返恒等式'], "second_ord_lim / p"),

    ('J10 动态上限的入参取错（用 P 而非 1/P）',
     "dyn_limit = inv_sqrt_controller(1.0 / YAW_RATE_P, YAW_GAIN, heading_accel_default)",
     "dyn_limit = inv_sqrt_controller(YAW_RATE_P, YAW_GAIN, heading_accel_default)",
     ['默认增益下，动态上限算出'], "dyn_limit = inv_sqrt_controller(YAW_RATE_P"),

    ('J11 45 度天花板被改成 450 度',
     "cap_45 = math.radians(45.0)",
     "cap_45 = math.radians(450.0)",
     ['AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD'], "math.radians(450.0)"),

    ('J12 主控制器增益换成后端自己那份',
     "YAW_RATE_P = MULTI_DEF['AC_ATC_MULTI_RATE_YAW_P']           # ATC_RAT_YAW_P 默认",
     "YAW_RATE_P = scaled('AC_ATC_MULTI_RATE_YAW_P')              # ATC_RAT_YAW_P 默认",
     ['两个增益不同源'], "scaled('AC_ATC_MULTI_RATE_YAW_P')"),

    ('J13 调度顺序判据写反',
     "check(i_rate < i_cust < i_out,",
     "check(i_rate < i_out < i_cust,",
     ['顺序：主 rate 控制器'], "i_rate < i_out < i_cust"),

    ('J14 RC 选项号写成 110',
     "check(AUX_NUM == 109, 'AUX_FUNC::CUSTOM_CONTROLLER = %d' % AUX_NUM)",
     "check(AUX_NUM == 110, 'AUX_FUNC::CUSTOM_CONTROLLER = %d' % AUX_NUM)",
     ['AUX_FUNC::CUSTOM_CONTROLLER'], "AUX_NUM == 110"),

    ('J15 README 漂移判定取反',
     'check(len(listed) != int(spoken) if spoken.isdigit() else True,',
     'check(len(listed) == int(spoken) if spoken.isdigit() else True,',
     ['说的 4 与列的 2 不一致'], "== int(spoken)"),

    ('J16 MAX_TYPES 一致性判据取反',
     "check(readme_max[1] != str(MAX_TYPES),",
     "check(readme_max[1] == str(MAX_TYPES),",
     ['示例的"改后"值'], "== str(MAX_TYPES)"),
]


def run(path):
    p = subprocess.run([sys.executable, '-u', path],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


base = io.open(SRC, encoding='utf-8').read()
rc, out = run(SRC)
print('基线（未注入）: EXIT=%s  %s' % (rc, 'GREEN' if rc == 0 else 'RED'))
if rc != 0:
    print(out[-3000:])
    sys.exit(1)
# 注意：脚本最后那行汇总自己就含 "[OK]"，直接 count 会多算 1。
n_ok_base = out.count('[OK]') - 1
print('基线断言数 = %d' % n_ok_base)
assert n_ok_base > 100, '断言数异常，先查脚本'
print()

fails = 0
for name, old, new, expect_red, must_appear in INJECTIONS:
    if base.count(old) != 1:
        print('[SKIP] %-38s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-38s 注入后 grep 不到' % name)
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-38s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:26] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:5]:
            print('         red> %s' % l[:110])
        if crashed:
            print('         err> %s' % out.strip().splitlines()[-1][:110])

print()
print('=' * 66)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
