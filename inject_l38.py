# -*- coding: utf-8 -*-
"""反向注入：确认 lesson38 的断言【真的会红】。

每条注入：
  1) 在源码里做【精确字符串替换】，替换后 grep 确认注入确实落盘（memory: reverse-test-must-actually-inject）
  2) 跑改过的副本
  3) 要求 EXIT != 0，且【指定的那条断言】出现在 [!!] 行里
     （memory: 死掉的守卫函数只会让检查器更绿 —— 必须点名验证是哪条红了）
"""
import io
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson38_smartrtl.py')
TMP = os.path.join(tempfile.gettempdir(), 'l38_mut.py')

INJECTIONS = [
    # (名字, 原文, 改成, 期望变红的断言的子串, 落地后应能 grep 到的串)
    ('J1 delta 破坏不变式',
     'PRUNING_DELTA = SRTL_ACCURACY_DEFAULT * 0.99',
     'PRUNING_DELTA = SRTL_ACCURACY_DEFAULT * 1.01',
     ['直线路径【0 个】假环', '不变式成立'], '* 1.01'),

    ('J2 抽稀门槛反转',
     'if (last[0] - p[0]) ** 2 + (last[1] - p[1]) ** 2 < accuracy ** 2:',
     'if (last[0] - p[0]) ** 2 + (last[1] - p[1]) ** 2 > accuracy ** 2:',
     ['恒等式 总长/accuracy+1'], '> accuracy ** 2:'),

    ('J3 DP 判据取反',
     '            if dmax > epsilon:\n                keep.add(idx)',
     '            if dmax < epsilon:\n                keep.add(idx)',
     # 注意：12/16 缩进命中的是 dp_timesliced；递归版在 J7 单独注入
     ['同一组点', '该预算下 DP 保证仍成立'], 'if dmax < epsilon:'),

    ('J7 递归版 DP 判据取反（压过头）',
     '        if dmax > epsilon:\n            keep.add(idx)\n            rec(s, idx)',
     '        if dmax < epsilon:\n            keep.add(idx)\n            rec(s, idx)',
     # 只点名 DP 正确性保证：压缩率那条【不会】红（它对压过头也放行）——
     # 这正是要证明的弱点，所以不能把它写进期望
     ['DP 正确性保证'], 'if dmax < epsilon:\n            keep.add(idx)\n            rec(s, idx)'),

    ('J4 Rally 保护 >= 而非 >',
     'if limit_km > 0 and min_dis > limit_km * 1000.0:',
     'if limit_km > 0 and min_dis >= limit_km * 1000.0:',
     ['恰好 1000 m 不触发'], 'min_dis >= limit_km'),

    ('J5 环检测去掉 i-2 间隔',
     '        while j <= i - 2:',
     '        while j <= i:',
     ['直线路径【0 个】假环'], 'while j <= i:'),

    ('J6 时间片版丢掉分割点',
     '                keep.add(idx)                  # ★ 分割点保留 —— 漏了这行结果就只剩首尾\n',
     '',
     ['同一组点'], 'keep.add(idx)'),
]


def run(path):
    p = subprocess.run([sys.executable, '-u', path],
                       capture_output=True, text=True, encoding='utf-8', errors='replace',
                       timeout=600)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


base = io.open(SRC, encoding='utf-8').read()

# ── 先证明【未注入】时是绿的（否则下面的"变红"没有意义）──
rc, out = run(SRC)
print('基线（未注入）: EXIT=%s  %s' % (rc, 'GREEN' if rc == 0 else 'RED'))
if rc != 0:
    print(out[-3000:])
    sys.exit(1)
print()

fails = 0
for name, old, new, expect_red, must_appear in INJECTIONS:
    if base.count(old) != 1:
        print('[SKIP] %-24s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    mutated = base.replace(old, new, 1)
    io.open(TMP, 'w', encoding='utf-8').write(mutated)

    # ① 确认注入真的落盘
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-24s 注入后 grep 不到 %r —— 没注进去' % (name, must_appear))
        fails += 1
        continue
    if old in landed and new not in old:
        print('[SKIP] %-24s 原文仍在，替换没生效' % name)
        fails += 1
        continue

    # ② 跑
    rc, out = run(TMP)
    red_lines = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red_lines)]

    ok = (rc != 0) and (len(hit) == len(expect_red))
    print('%-6s %-24s EXIT=%-3s 变红 %d/%d 项  %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             ' | '.join(h[:46] for h in hit)))
    if not ok:
        fails += 1
        for l in red_lines[:6]:
            print('         red> %s' % l[:110])

print()
print('=' * 60)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
