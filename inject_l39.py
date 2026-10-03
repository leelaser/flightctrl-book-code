# -*- coding: utf-8 -*-
"""反向注入：确认 lesson39 的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson39_avoid.py')
TMP = os.path.join(tempfile.gettempdir(), 'l39_mut.py')

INJECTIONS = [
    ('J1 停止距离漏掉线性项',
     '    return accel / (2.0 * kP * kP) + (speed * speed) / (2.0 * accel)',
     '    return (speed * speed) / (2.0 * accel)',
     ['往返一致'], '/ (2.0 * accel)'),

    ('J2 sqrt 分界用错幂次',
     '        linear_dist = second_ord_lim / (p * p)    # ★ 线性区/开方区的分界【距离】',
     '        linear_dist = second_ord_lim / p          # 注入：少了平方',
     ['还原误差', '互为逆', '密集扫描'], 'second_ord_lim / p          # 注入'),

    ('J3 逆函数分界用错幂次',
     '    linear_velocity = D_max / p                   # ★ 分界【速度】',
     '    linear_velocity = D_max / (p * p)             # 注入：多了平方',
     # ★ 手挑的 3 组盖不住这个注入（实测全绿）——密集扫描才抓得到
     ['密集扫描'], 'D_max / (p * p)             # 注入'),

    ('J4 纯运动学分支少个 2',
     '        return math.sqrt(2.0 * distance * accel)',
     '        return math.sqrt(distance * accel)',
     ['v² = ', '往返一致'], 'math.sqrt(distance * accel)'),

    ('J5 投影限幅的判据取反',
     '    if speed > max_speed:',
     '    if speed < max_speed:',
     ['与独立算出的期望'], 'if speed < max_speed:'),

    ('J6 dt 钳位写成乘法',
     '        return clamp(rate, -abs(error) / dt, abs(error) / dt)   # ★ dt 钳位',
     '        return clamp(rate, -abs(error) * dt, abs(error) * dt)   # 注入',
     ['全部】未咬合', '上界】实测'], 'abs(error) * dt, abs(error) * dt'),

    ('J7 象限内取 min 而非 max',
     '            q[0] = [max(q[0][0], vx), max(q[0][1], vy)]',
     '            q[0] = [min(q[0][0], vx), min(q[0][1], vy)]',
     ['逐轴最大值', '各占一个象限'], 'min(q[0][0], vx)'),

    ('J8 STOP 用停止点距离而非交点距离',
     '    limit_dist = t * math.hypot(stop_pt[0] - pos[0], stop_pt[1] - pos[1])',
     '    limit_dist = math.hypot(stop_pt[0] - pos[0], stop_pt[1] - pos[1])',
     ['整个清零'], 'limit_dist = math.hypot(stop_pt'),
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
    print(out[-2500:])
    sys.exit(1)
print()

fails = 0
for name, old, new, expect_red, must_appear in INJECTIONS:
    if base.count(old) != 1:
        print('[SKIP] %-34s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))

    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-34s 注入后 grep 不到 %r' % (name, must_appear))
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    ok = (rc != 0) and (len(hit) == len(expect_red))
    print('%-7s %-34s EXIT=%-3s 变红 %d/%d  %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             ' | '.join(h[:40] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:5]:
            print('         red> %s' % l[:110])

print()
print('=' * 62)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
