# -*- coding: utf-8 -*-
"""反向注入：确认 lesson45 的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson45_autotune.py')
TMP = os.path.join(tempfile.gettempdir(), 'l45_mut.py')

INJECTIONS = [
    ('J1  1-1/e 的基准算错',
     "one_minus_inv_e = 1.0 - math.exp(-1.0)",
     "one_minus_inv_e = 1.0 - math.exp(-1.2)",
     ['0.6321 与 1-1/e 的差'], "math.exp(-1.2)"),

    ('J2  被调对象的单调方向写反',
     "    return k * tune_d / (tune_d + d50)      # d50=0.15 时，D=0.05 恰好给出 0.075",
     "    return k * d50 / (tune_d + d50)         # d50=0.15 时，D=0.05 恰好给出 0.075",
     ['模型自证'], "k * d50 / (tune_d + d50)"),

    ('J3  D 的几何步长漏掉 ×2',
     "        tune_d += tune_d * RD_STEP * 2.0",
     "        tune_d += tune_d * RD_STEP",
     ['每一次上调的倍率'], "tune_d * RD_STEP\n"),

    ('J4  SUCCESS_COUNT 预期改成 5',
     "check(m is not None and int(m.group(1)) == 4,",
     "check(m is not None and int(m.group(1)) == 5,",
     ['AUTOTUNE_SUCCESS_COUNT'], "== 5,"),

    ('J5  FreqResp 增益算错（多乘 2）',
     "                c['curr_test_gain'] = sum_gain / gcnt",
     "                c['curr_test_gain'] = sum_gain / gcnt * 2.0",
     ['增益回读'], "sum_gain / gcnt * 2.0"),

    ('J6  5730 的预期值改错',
     "check(abs(C5730 - 5730.0) < 1e-9, '★ 源码里的常数 = %s' % C5730)",
     "check(abs(C5730 - 5700.0) < 1e-9, '★ 源码里的常数 = %s' % C5730)",
     ['源码里的常数'], "C5730 - 5700.0"),

    ('J7  大括号配平写反（函数体提取失效）',
     "        elif text[j] == '}':\n            depth -= 1",
     "        elif text[j] == '}':\n            depth += 1",
     ['空实现'], "depth += 1"),

    ('J8  枚举解析退回"要求尾逗号"的老写法',
     "    for item in body.split(','):",
     "    for item in re.findall(r'(\\\\w+)\\\\s*(?:=\\\\s*(\\\\d+))?\\\\s*,', body + ','):",
     ['枚举 Step'], "body + ','"),

    ('J9  Tune Check 不计入序列长度',
     "    if mask & 16:\n        n += 1",
     "    if mask & 16:\n        n += 0",
     ['越界'], "n += 0"),

    ('J10 gain_mul 的指数写错',
     "gain_mul = math.exp(math.log(2.0) / (RATE * max(1, DTIME)))",
     "gain_mul = math.exp(math.log(2.0) / (RATE * max(1, DTIME) * 2.0))",
     ['独立恒等式'], "max(1, DTIME) * 2.0)"),

    ('J11 get_pname 处理的 Stage 预期缩小',
     "check(handled == {'P', 'D', 'FLTT', 'FLTD'},",
     "check(handled == {'P', 'D'},",
     ['get_pname() 只处理'], "== {'P', 'D'},"),

    ('J12 QAUTOTUNE 的 SITL 条件判反',
     "check(m and 'HAL_BOARD_SITL' in m.group(1),",
     "check(m and 'HAL_BOARD_SITL' not in m.group(1),",
     ['只在 SITL 编译'], "not in m.group(1)"),

    ('J13 save 里 PI_RATIO_FINAL 计数改错',
     "check(SAVE.count('AUTOTUNE_PI_RATIO_FINAL') == 2,",
     "check(SAVE.count('AUTOTUNE_PI_RATIO_FINAL') == 3,",
     ['PI_RATIO_FINAL 出现'], "== 3,"),

    ('J14 4.5.7 的 tune_seq 去 .cpp 里找',
     "    m_old_seq = re.search(r'tune_seq\\[0\\]\\s*=\\s*(\\w+);', old_multih)",
     "    m_old_seq = re.search(r'tune_seq\\[0\\]\\s*=\\s*(\\w+);', old_multi)",
     ['4.5.7 的序列第 0 步'], "old_multi)"),

    ('J15 4.5.7 参数个数预期改成 5',
     "    check(n_old == 4 and n_par == 6,",
     "    check(n_old == 5 and n_par == 6,",
     ['签名从 4.5.7'], "n_old == 5"),

    ('J16 period 的身份判定放宽',
     "check(period_hits == {'get_loop_period_s'},",
     "check(period_hits == set(),",
     ['period 全文件只以'], "== set(),"),

    ('J17 Quicktune 不注入激励这条判反',
     "check('input_angle_step' not in src_qt and 'input_rate_step' not in src_qt,",
     "check('input_angle_step' in src_qt and 'input_rate_step' not in src_qt,",
     ['零命中'], "'input_angle_step' in src_qt"),
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
        print('[SKIP] %-40s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-40s 注入后 grep 不到' % name)
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-40s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:26] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:5]:
            print('         red> %s' % l[:108])
        if crashed:
            print('         err> %s' % out.strip().splitlines()[-1][:108])

print()
print('=' * 68)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
