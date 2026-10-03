# -*- coding: utf-8 -*-
"""反向注入：确认 lesson42 的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson42_notch.py')
TMP = os.path.join(tempfile.gettempdir(), 'l42_mut.py')

INJECTIONS = [
    ('J1 A 的分母 40 改成 20',
     '    A = 10 ** (-attenuation_dB / 40.0)',
     '    A = 10 ** (-attenuation_dB / 20.0)',
     ['实测', '== −'], 'attenuation_dB / 20.0'),

    ('J2 Q 公式分母改成 2^n+1',
     '        Q = math.sqrt(2 ** octaves) / (2 ** octaves - 1.0)',
     '        Q = math.sqrt(2 ** octaves) / (2 ** octaves + 1.0)',
     # ★ 第一版这条注入【没被抓住】—— 因为那段断言内联重抄了公式，
     #   没有调用 src_calculate_A_and_Q。改成真调用后就能抓住了。
     ['源码函数给的 Q', '代数恒等'], 'math.sqrt(2 ** octaves) / (2 ** octaves + 1.0)'),

    ('J3 alpha 少除一个 2',
     '    alpha = math.sin(omega) / (2 * Q)',
     '    alpha = math.sin(omega) / Q',
     ['源码与 RBJ 最大差'], 'math.sin(omega) / Q'),

    ('J4 b2 的符号取反',
     '    b2 = 1.0 - alpha * A * A\n    a1 = b1\n    a2 = 1.0 - alpha',
     '    b2 = 1.0 + alpha * A * A\n    a1 = b1\n    a2 = 1.0 - alpha',
     ['源码与 RBJ 最大差'], 'b2 = 1.0 + alpha * A * A'),

    ('J5 a2 的符号取反',
     '    a2 = 1.0 - alpha\n    k = 1.0 / (1.0 + alpha)',
     '    a2 = 1.0 + alpha\n    k = 1.0 / (1.0 + alpha)',
     ['源码与 RBJ 最大差'], 'a2 = 1.0 + alpha'),

    ('J6 归一化因子用错',
     '    k = 1.0 / (1.0 + alpha)',
     '    k = 1.0 / (1.0 - alpha)',
     ['源码与 RBJ 最大差'], 'k = 1.0 / (1.0 - alpha)'),

    ('J7 八度数不乘 2',
     '        octaves = math.log2(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0)) * 2.0',
     '        octaves = math.log2(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0))',
     ['实测/请求BW'], 'bandwidth_hz / 2.0))'),

    ('J8 SLEW 下限改成 0.5',
     'lower = 1.0 - 0.05',
     'lower = 1.0 - 0.5',
     ['上界 = 1/0.95'], 'lower = 1.0 - 0.5'),

    ('J9 近似八度数改成真实八度数',
     'oct_approx = 2 * math.log2(100 / (100 - 30))',
     'oct_approx = math.log2((100 + 30) / (100 - 30))',
     ['近似八度数'], 'oct_approx = math.log2((100 + 30)'),

    ('J10 谐波倍率少 1（阶数错位）',
     '        mul = hn + 1\n        if composite != 2:',
     '        mul = hn\n        if composite != 2:',
     ['倍率恰好是', '倍率 [1,2,3,4]'], 'mul = hn\n        if composite != 2:'),

    ('J11 复合陷波 spread 分母 32→16',
     'LAG_SPREAD = LAG_BW / (32 * LAG_BASE)',
     'LAG_SPREAD = LAG_BW / (16 * LAG_BASE)',
     ['LAG_SPREAD = BW/(32·f0)', '复合陷波实际落在'],
     'LAG_SPREAD = LAG_BW / (16 * LAG_BASE)'),

    ('J12 Nyquist 门限 0.48→0.90',
     'NQ = 0.48',
     'NQ = 0.90',
     ['1 kHz 采样的陷波上限'], 'NQ = 0.90'),

    ('J13 FFT 分辨率多除一个 2',
     '    return fs / float(n)',
     '    return fs / (2.0 * float(n))',
     ['恒等式 fs/N', '源码注释的算例复现'], 'return fs / (2.0 * float(n))'),

    ('J14 双陷波第二条不加 spread',
     '    for _sm in (1.0 - LAG_SPREAD, 1.0 + LAG_SPREAD):',
     '    for _sm in (1.0 - LAG_SPREAD, 1.0):',
     ['四个频点全部落进官方容差内'], 'for _sm in (1.0 - LAG_SPREAD, 1.0):'),
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
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-30s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:26] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:3]:
            print('         red> %s' % l[:104])

print()
print('=' * 60)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
