# -*- coding: utf-8 -*-
"""反向注入：确认 lesson40 的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson40_nogps.py')
TMP = os.path.join(tempfile.gettempdir(), 'l40_mut.py')

INJECTIONS = [
    ('J1 几何真值漏掉 y² 项',
     '    return -v_y * h / (h * h + y * y)',
     '    return -v_y * h / (h * h)',
     ['几何法本身可信'], 'return -v_y * h / (h * h)'),

    ('J2 光流模型多除一个 h',
     '    return (vy / height_m, -vx / height_m)',
     '    return (vy / (height_m * height_m), -vx / (height_m * height_m))',
     ['== 几何真值(正下方)'], 'height_m * height_m'),

    ('J3 小角近似误差公式写错',
     '    pred = y_rel ** 2                      # ★ 相对误差就是 (y/h)²',
     '    pred = y_rel ** 2 / (1.0 + y_rel ** 2)  # 注入：错的形式',
     ['解析预言 (y/h)²'], 'y_rel ** 2 / (1.0 + y_rel ** 2)'),

    ('J4 倾斜门限改成 0.5',
     'DCM33FlowMin = 0.71     # AP_NavEKF3.h:506',
     'DCM33FlowMin = 0.50     # 注入',
     ['acos(0.71)'], 'DCM33FlowMin = 0.50'),

    ('J5 光流饱和阈值改成 25',
     'EK3_FLOW_MAX = 2.5      # AP_NavEKF3.cpp:312',
     'EK3_FLOW_MAX = 25.0     # 注入',
     ['独立算出的期望值'], 'EK3_FLOW_MAX = 25.0'),

    ('J6 投票去掉 clamp',
     '        vote = max(min(vote + d, VOTE_MAX), -VOTE_MAX)',
     '        vote = vote + d                     # 注入：不夹',
     # ★ 注意：覆盖 clamp 的是"40 拍后停在 +20"那条，
     #   不是"第 20 拍打满"—— 加不夹都在第 20 拍打满。
     ['被 clamp 住'], 'vote = vote + d'),

    ('J7 Yaw 轴的 GPS 编号改成 3',
     "'GPS': 2, 'GPS_COMPASS_FALLBACK': 3",
     "'GPS': 3, 'GPS_COMPASS_FALLBACK': 3",
     ['轴内编号互不相同'], "'GPS': 3, 'GPS_COMPASS_FALLBACK': 3"),

    ('J8 SRC2 也配上 GPS（假装有冗余）',
     "2: {'POSXY': ('NONE', 0)",
     "2: {'POSXY': ('GPS', 3)",
     ['只有第 1 套'], "2: {'POSXY': ('GPS', 3)"),
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
        print('[SKIP] %-30s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-30s 注入后 grep 不到' % name)
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    ok = (rc != 0) and (len(hit) == len(expect_red))
    print('%-7s %-30s EXIT=%-3s 变红 %d/%d  %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             ' | '.join(h[:34] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:4]:
            print('         red> %s' % l[:108])

print()
print('=' * 60)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
