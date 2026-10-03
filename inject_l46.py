# -*- coding: utf-8 -*-
"""反向注入：确认 lesson46 的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson46_emergency.py')
TMP = os.path.join(tempfile.gettempdir(), 'l46_mut.py')

INJECTIONS = [
    ('J1  释放延迟改成 400 ms',
     "check(on_time == 1000 + DELAY,",
     "check(on_time == 1000 + 400,",
     ['时序：release'], "1000 + 400,"),

    ('J2  _release_initiated 计数改成 2',
     "check(src_chute.count('_release_initiated') == 3,",
     "check(src_chute.count('_release_initiated') == 2,",
     ['_release_initiated 在 .cpp 里只出现'], "== 2,"),

    ('J3  声称 _released 会被复位（判反）',
     "'_released = false' not in src_chute,",
     "'_released = false' in src_chute,",
     ['_released 同样只置位、从不复位'], "in src_chute,"),

    ('J4  注释与代码不符那条判反',
     "check('// reset released flag and release_time' in upd,",
     "check('// reset released flag and release_time' not in upd,",
     ['reset released flag'], "not in upd,"),

    ('J5  下沉率持续门限预期改成 500 ms',
     "check(SINK_MS == 1000,",
     "check(SINK_MS == 500,",
     ['check_sink_rate 的判据是'], "SINK_MS == 500,"),

    ('J6  下沉率触发时刻算错（预期 1100）',
     "check(sink_trigger(cont) == 1200,",
     "check(sink_trigger(cont) == 1100,",
     ['连续超阈值'], "== 1100,"),

    ('J7  AFS 的 terminate_action 值改错',
     "check(TA == {'TERMINATE_ACTION_TERMINATE': 42, 'TERMINATE_ACTION_LAND': 43},",
     "check(TA == {'TERMINATE_ACTION_TERMINATE': 0, 'TERMINATE_ACTION_LAND': 1},",
     ['terminate_action 只有两项'], "'TERMINATE_ACTION_LAND': 1},"),

    ('J8  声称 should_crash_vehicle 只认 42',
     "check(scv.count('TERMINATE_ACTION_TERMINATE') == 1 and\n"
     "      scv.count('TERMINATE_ACTION_LAND') == 1,",
     "check(scv.count('TERMINATE_ACTION_TERMINATE') == 1 and\n"
     "      scv.count('TERMINATE_ACTION_LAND') == 0,",
     ['而紧跟的代码接受'], "count('TERMINATE_ACTION_LAND') == 0,"),

    ('J9  FLARE 相位"可达"（可达性判反）',
     "check(unreachable == ['FLARE_INIT', 'FLARE', 'TOUCH_DOWN_INIT', 'TOUCH_DOWN'],",
     "check(unreachable == [],",
     ['从来没有被赋值过'], "== [],"),

    ('J10 转速控制器的误差定义取反',
     "check(re.search(r'_head_speed_error\\s*=\\s*head_speed_norm\\s*-\\s*_target_head_speed;', hs)",
     "check(re.search(r'_head_speed_error\\s*=\\s*_target_head_speed\\s*-\\s*head_speed_norm;', hs)",
     ['误差定义是'], "_target_head_speed\\s*-\\s*head_speed_norm;"),

    ('J11 转速符号链的模拟取反',
     "    err = meas_norm - target_norm",
     "    err = target_norm - meas_norm",
     ['符号链：转速【偏低】时'], "err = target_norm - meas_norm"),

    ('J12 圆周运动的 ω = a/v 改成 ω = a·v',
     "    w_pred = a / v                     # ← 源码用的那个公式",
     "    w_pred = a * v                     # ← 源码用的那个公式",
     ['独立基准：源码那条'], "w_pred = a * v"),

    ('J13 着地判据三条件 AND 改成 OR',
     "return\\s+_landed_reason\\.min_speed\\s*&&\\s*_landed_reason\\.land_col\\s*&&\\s*'\n"
     "                r'_landed_reason\\.is_still;', cl) is not None,",
     "return\\s+_landed_reason\\.min_speed\\s*\\|\\|\\s*_landed_reason\\.land_col\\s*&&\\s*'\n"
     "                r'_landed_reason\\.is_still;', cl) is not None,",
     ['返回的是三个条件的'], "min_speed\\s*\\|\\|"),

    ('J14 Copter 的 AFS 默认值改成 1',
     "check(m is not None and m.group(1) == '0',\n"
     "      '★★ Copter 的 AP_COPTER_ADVANCED_FAILSAFE_ENABLED 默认 = %s'",
     "check(m is not None and m.group(1) == '1',\n"
     "      '★★ Copter 的 AP_COPTER_ADVANCED_FAILSAFE_ENABLED 默认 = %s'",
     ['AP_COPTER_ADVANCED_FAILSAFE_ENABLED 默认'], "m.group(1) == '1',"),

    ('J15 SERVOn_FUNCTION 编号改成 28',
     "check(m is not None and int(m.group(1)) == 27,",
     "check(m is not None and int(m.group(1)) == 28,",
     ['物理出口'], "int(m.group(1)) == 28,"),

    ('J16 COL_FILT_E 的"越界"判定取反',
     "check(not (lo_rng <= dflt <= hi_rng),",
     "check((lo_rng <= dflt <= hi_rng),",
     ['而它的默认值是'], "lo_rng <= dflt <= hi_rng"),

    ('J17 FWD_* 注释顺序与真实槽位"一致"',
     "check([x.replace('FWD_', '') for x in doc_order] != order,",
     "check([x.replace('FWD_', '') for x in doc_order] == order,",
     ['两个顺序不一样'], "for x in doc_order] == order,"),

    ('J18 do_parachute 死声明那条判反',
     "check(all('do_parachute' not in slurp(p) for p in\n"
     "          ('ArduCopter/mode_auto.cpp', 'ArduCopter/mode_guided.cpp')),",
     "check(any('do_parachute' in slurp(p) for p in\n"
     "          ('ArduCopter/mode_auto.cpp', 'ArduCopter/mode_guided.cpp')),",
     ['任务路径走的是'], "any('do_parachute' in slurp(p)"),
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
        print('[SKIP] %-42s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-42s 注入后 grep 不到' % name)
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-42s EXIT=%-3s 变红 %d/%d %s'
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
