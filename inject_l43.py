# -*- coding: utf-8 -*-
"""反向注入：确认 lesson43 的断言【真的会红】。"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

SRC = os.path.join(HERE, 'lesson43_lua.py')
TMP = os.path.join(tempfile.gettempdir(), 'l43_mut.py')

INJECTIONS = [
    ('J1 float32 精度边界算错（2^24→2^23）',
     'FLOAT24 = 2 ** 24',
     'FLOAT24 = 2 ** 23',
     ['2^24+1'], 'FLOAT24 = 2 ** 23'),

    ('J2 沙箱：把"被注释"判成"启用"',
     "            commented = t.startswith('//')",
     "            commented = False",
     ['`os` 库被注释掉'], "commented = False"),

    ('J3 rename 行的过滤条件写反',
     "rn = [l for l in bd if 'rename' in l and ' method ' not in l]",
     "rn = [l for l in bd if 'rename' in l and ' method ' in l]",
     ["`rename` "], "' method ' in l]"),

    ('J4 _ud 后缀检测失效',
     "ud = [k for k in cls if k.endswith('_ud')]",
     "ud = [k for k in cls if k.endswith('_ZZ')]",
     ['`_ud` 后缀'], "k.endswith('_ZZ')"),

    ('J5 UNKNOWN 免检口子被堵死',
     "    if ((len(A) == 1 and A[0] == 'UNKNOWN') or\n"
     "            (len(B) == 1 and B[0] == 'UNKNOWN')):\n        return True",
     "    if ((len(A) == 1 and A[0] == 'UNKNOWN') or\n"
     "            (len(B) == 1 and B[0] == 'UNKNOWN')):\n        return False",
     ['UNKNOWN'], 'return False'),

    ('J6 skip_check 统计改成数 method 行',
     "    c = sum(1 for l in sub if mark in l)",
     "    c = sum(1 for l in sub if mark in l and ' method ' in l)",
     # 第一版这里只断言 "> 0"，注入后计数掉了但仍是正数 ⇒ 全绿。
     # 收紧成比例带之后才抓得住。
     ['近三成'], 'and \' method \' in l'),

    ('J7 类名匹配改成"模糊匹配也算"',
     "same = [k for k in cls if k in cpp_cls]",
     "same = [k for k in cls if k.replace('_ud', '') in cpp_cls]",
     ['同名比对根本不成立'], "k.replace('_ud', '') in cpp_cls"),

    ('J8 全局/方法分类判据写反',
     "methods = [l for l in funcs if ':' in lua_name_of(l)]\n"
     "globals_ = [l for l in funcs if ':' not in lua_name_of(l)]",
     "methods = [l for l in funcs if ':' not in lua_name_of(l)]\n"
     "globals_ = [l for l in funcs if ':' in lua_name_of(l)]",
     ['方法占绝对多数'], "':' not in lua_name_of(l)]\n"),
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
        print('[SKIP] %-34s 原文出现 %d 次（应为 1）' % (name, base.count(old)))
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(base.replace(old, new, 1))
    landed = io.open(TMP, encoding='utf-8').read()
    if must_appear not in landed:
        print('[SKIP] %-34s 注入后 grep 不到' % name)
        fails += 1
        continue

    rc, out = run(TMP)
    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-34s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:24] for h in hit)))
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
