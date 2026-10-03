# -*- coding: utf-8 -*-
"""反向注入：确认 lesson41 的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson41_wind.py')
TMP = os.path.join(tempfile.gettempdir(), 'l41_mut.py')

INJECTIONS = [
    ('J1 真风矢量改减法',
     '    wtv = (wav[0] + veh_vel_ne[0], wav[1] + veh_vel_ne[1])',
     '    wtv = (wav[0] - veh_vel_ne[0], wav[1] - veh_vel_ne[1])',
     ['真风速 = '], 'wav[0] - veh_vel_ne[0]'),

    ('J2 少加 180°（来向↔去向）',
     '    wind_dir_180 = app_dir_body + yaw + math.pi',
     '    wind_dir_180 = app_dir_body + yaw',
     ['真风速 = '], 'app_dir_body + yaw\n'),

    ('J3 输出不转回"来向"',
     '    return wrap_pi(math.atan2(wtv[1], wtv[0]) - math.pi), math.hypot(wtv[0], wtv[1])',
     '    return wrap_pi(math.atan2(wtv[1], wtv[0])), math.hypot(wtv[0], wtv[1])',
     ['方向 = '], 'return wrap_pi(math.atan2(wtv[1], wtv[0])),'),

    ('J4 欧拉角俯仰符号取反',
     '    R = from_euler_zyx(0.0, math.radians(-incl_deg), math.radians(decl_deg))',
     '    R = from_euler_zyx(0.0, math.radians(incl_deg), math.radians(decl_deg))',
     ['三分量都对得上'], 'math.radians(incl_deg), math.radians(decl_deg)'),

    ('J5 退化为纯北向（丢掉旋转）',
     '    R = from_euler_zyx(0.0, math.radians(-incl_deg), math.radians(decl_deg))\n    v = [intensity, 0.0, 0.0]\n    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]',
     '    return [intensity, 0.0, 0.0]',
     # ★ 只期望"三分量"红：恒等旋转【仍然保模长】，
     #   所以模长守恒那条不会红 —— 两条断言各守一个轴，缺一不可。
     ['三分量都对得上'], 'return [intensity, 0.0, 0.0]'),

    ('J6 磁矢量乘 2（破坏模长守恒）',
     '    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]',
     '    return [2.0 * sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]',
     ['模长守恒'], 'return [2.0 * sum(R[i][j]'),

    ('J7 解缠绕阈值放宽到 360（等于不折）',
     '        while v - prev > 180.0:',
     '        while v - prev > 360.0:',
     ['解缠绕后的最大二阶差分'], 'while v - prev > 360.0:'),

    ('J8 分带切片上下界写反',
     '    k0 = int(round((-lat_abs_max - MINLAT) / RES))\n    k1 = int(round((lat_abs_max - MINLAT) / RES))',
     '    k0 = int(round((lat_abs_max - MINLAT) / RES))\n    k1 = int(round((-lat_abs_max - MINLAT) / RES))',
     ['分带切片'], 'k1 = int(round((-lat_abs_max - MINLAT) / RES))'),

    ('J9 磁偏角越界标志被转发（第 6 段）',
     "check('valid_input_data' not in snippet,",
     "check('valid_input_data' in snippet,",
     ['片段里【没有】valid_input_data'], "check('valid_input_data' in snippet,"),
]


def run(path):
    p = subprocess.run([sys.executable, '-u', path],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=300)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


base = io.open(SRC, encoding='utf-8').read()
rc, out = run(SRC)
print('基线（未注入）: EXIT=%s  %s' % (rc, 'GREEN' if rc == 0 else 'RED'))
if rc != 0:
    print(out[-2000:])
    sys.exit(1)
print()

fails = 0
for name, old, new, expect_red, must_appear in INJECTIONS:
    if base.count(old) != 1:
        print('[SKIP] %-32s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-32s 注入后 grep 不到' % name)
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    # 异常退出也算"被抓到"（如 assert 触发），但要点名
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-32s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:28] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:4]:
            print('         red> %s' % l[:106])
        if crashed:
            print('         err> %s' % out.strip().splitlines()[-1][:106])

print()
print('=' * 62)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
