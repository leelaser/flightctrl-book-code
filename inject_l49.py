# -*- coding: utf-8 -*-
"""反向注入：确认 lesson49（算法地图）的断言【真的会红】。

★ 本注入器的特殊性：**大部分注入打在【章】上，不是打在脚本上。**
  第 49 个脚本的职责是"校验那张地图准不准"，
  所以要问的问题是："**地图写错了，它抓得到吗**" —— 把地图改错，看脚本红不红。

★ 2026-10-02 改造：打的对象从**素材那篇**换成**成书这一章**
  （`书稿/卷8-机载软件架构与算法/第05章-…md`）。
  两个必须记住的坑，都是踩过才写的：
    · 变异脚本**必须写在 `code/` 里** —— 新脚本靠 `dirname(__file__)` 推出书根，
      落到临时目录就找不到章了，于是**红的是"读不到文件"而不是我们要验的那条**
      （[[reverse-test-must-actually-inject]] 的下一层：红了，但要红在对的那条）。
    · 章被注入期间会**临时改写**，跑完必须还原；还原放在 `finally` 里。
"""
import io
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DOC = os.path.join(ROOT, '书稿', '卷8-机载软件架构与算法',
                   '第05章-算法地图-ArduPilot的全部算法库.md')
SRC = os.path.join(HERE, 'lesson49_map.py')
DOC_BAK = os.path.join(tempfile.gettempdir(), '_l49_doc_bak.md')
TMP_S = os.path.join(HERE, '_l49_mut.py')      # ★ 必须与 SRC 同目录，见文件头

# (说明, 目标, old, new, 期望变红的片段, 注入后必须 grep 到)
INJECTIONS = [
    # ---------- 打在地图上：行数与断言数 ----------
    ('D1  §5.2 把第 6 章的行数 489 改成 490', 'doc',
     '| 第 6 章 | 489 |', '| 第 6 章 | 490 |',
     ['14 章行数与表里写的一致'], '| 第 6 章 | 490 |'),

    ('D2  §5.1 把总行数 15399 改成 15400', 'doc',
     '| 正文 | **14 章 / 15399 行** |', '| 正文 | **14 章 / 15400 行** |',
     ['表格合计'], '15400 行'),

    ('D3  §5.2 把第 6 章的断言数 40 改成 41', 'doc',
     '`lesson35_mission.py` (40)', '`lesson35_mission.py` (41)',
     ['逐一**真跑**了这 14 个脚本'], '(41)'),

    ('D4  §5.2 把脚本名写错一个', 'doc',
     '`lesson36_tecs.py` (25)', '`lesson36_tecss.py` (25)',
     ['逐一**真跑**了这 14 个脚本'], 'lesson36_tecss.py'),

    # ---------- 打在地图上：库归属 ----------
    ('D5  §5.3.4 把 AC_CustomControl 的主场 第 15 章 改成 第 6 章', 'doc',
     '| `AC_CustomControl` | **自定义控制律的挂载点** | **第 15 章**（65） |',
     '| `AC_CustomControl` | **自定义控制律的挂载点** | **第 6 章**（65） |',
     ['有明确归属声明'], '**第 6 章**（65）'),

    ('D6  §5.3.2 把 AP_Mission 的主场 第 6 章 改成 第 10 章', 'doc',
     '| `AP_Mission` | 任务存储与执行 | **第 6 章**（44 处） |',
     '| `AP_Mission` | 任务存储与执行 | **第 10 章**（44 处） |',
     ['有明确归属声明'], '**第 10 章**（44 处）'),

    ('D7  §5.3.3 把 AP_NavEKF3 的归属 卷 2 第 9 章 改成 第 13 章', 'doc',
     '| `AP_NavEKF3` | 主用 EKF | **卷 2 第 9 章**（52） |',
     '| `AP_NavEKF3` | 主用 EKF | **第 13 章**（52） |',
     ['有明确归属声明'], '**第 13 章**（52）'),

    ('D8  §5.3.2 把 AP_L1_Control 的归属 卷 3 第 6 章 改成 第 6 章', 'doc',
     '| `AP_L1_Control` | L1 横向制导 | **卷 3 第 6 章**（17） |',
     '| `AP_L1_Control` | L1 横向制导 | **第 6 章**（17） |',
     ['有明确归属声明'], 'L1 横向制导 | **第 6 章**'),

    # ---------- 打在地图上：零命中表 ----------
    ('D9  §5.3.8 往零命中表里塞一个非零库 AP_GPS', 'doc',
     '| `AP_Airspeed` | 空速计驱动 |',
     '| `AP_GPS` | 定位 | 属定位层 |\n| `AP_Airspeed` | 空速计驱动 |',
     ['集合相等'], '| `AP_GPS` | 定位 |'),

    ('D10 §5.3.8 从零命中表里删掉一行 AP_RPM', 'doc',
     '| `AP_RPM` | 转速传感器 | 只在机型章侧面提过原理 |\n', '',
     ['集合相等'], '## 5.4'),

    ('D11 §5.3.3 把 AP_GyroFFT 的主场改成 第 6 章（不是最大）', 'doc',
     '| `AP_GyroFFT` | 陀螺频谱分析（自动找陷波频率） | **第 13 章**（19）= 卷 3 第 4 章（19） |',
     '| `AP_GyroFFT` | 陀螺频谱分析（自动找陷波频率） | **第 6 章**（19） |',
     ['有明确归属声明'], '自动找陷波频率） | **第 6 章**'),

    # ---------- 打在脚本上：判据本身 ----------
    ('S1  零命中清单加一个非零库', 'script',
     "ZERO = ['AP_Airspeed', 'AP_Camera',", "ZERO = ['AP_GPS', 'AP_Airspeed', 'AP_Camera',",
     ['除本章外**命中'], "'AP_GPS', 'AP_Airspeed'"),

    ('S2  量具自证的门槛抬到不可能', 'script',
     "check(lib_total_excl('AP_Mission') > 30,", "check(lib_total_excl('AP_Mission') > 3000,",
     ['先自证量具'], '> 3000,'),

    ('S3  全书章数门槛抬到不可能', 'script',
     "check(len(DOCS) > 125,", "check(len(DOCS) > 5000,",
     ['全书扫到'], '> 5000,'),

    ('S4  把 build.py 挑章模式的期望改坏', 'script',
     "CHAP_RE = re.compile(r'^第(\\d{2})章-(.+)\\.md$')",
     "CHAP_RE = re.compile(r'^第(\\d{3})章-(.+)\\.md$')",
     ['符合 build.py 的挑章模式'], r"第(\d{3})章"),

    ('S5  卷内归属的默认卷号改错（卷 8 → 卷 1）', 'script',
     "    vno = int(m.group(1)) if m.group(1) else 8      # 不带「卷」= 本卷",
     "    vno = int(m.group(1)) if m.group(1) else 1      # 不带「卷」= 本卷",
     ['有明确归属声明'], "else 1      #"),

    ('S6  脚本数从 14 改成 15', 'script',
     "check(len(uniq) == 14,", "check(len(uniq) == 15,",
     ['速查表里点到'], '== 15,'),

    ('S7  未读字段数从 17 改成 16', 'script',
     "check(len(unused) == 17,", "check(len(unused) == 16,",
     ['个具名字段从不被读'], 'len(unused) == 16,'),

    ('S8  _raw_climb_rate 计数从 2 改成 3', 'script',
     "check(len(re.findall(r'_raw_climb_rate', vario)) == 2,",
     "check(len(re.findall(r'_raw_climb_rate', vario)) == 3,",
     ['只 2 次'], '== 3,'),

    ('S9  自指污染那条判反（== 改成 !=）', 'script',
     "    check(m is not None and int(m.group(1)) == real,",
     "    check(m is not None and int(m.group(1)) != real,",
     ['地图确实在污染自己的统计'], '!= real,'),

    ('S10 排除本章改成排除别章（零命中立刻不成立）', 'script',
     "               if not (exclude_self and os.path.abspath(p) == os.path.abspath(DOC)))",
     "               if not (exclude_self and os.path.abspath(p) == os.path.abspath(DOCS[0])))",
     ['除本章外**命中'], 'os.path.abspath(DOCS[0])'),

    ('S11 覆盖回归那条判反（is not None → is None）', 'script',
     "check(CHAP_RE.match(bn) is not None,", "check(CHAP_RE.match(bn) is None,",
     ['符合 build.py 的挑章模式'], 'CHAP_RE.match(bn) is None,'),
]


def run_script(fast=True):
    cmd = [sys.executable, '-u', SRC] + (['--fast'] if fast else [])
    p = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=900)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


# 这几例打的正是"真跑 14 个脚本"那条断言 ⇒ 不能加 --fast
NEEDS_FULL = ('D3', 'D4')


def main():
    shutil.copyfile(DOC, DOC_BAK)
    base_script = io.open(SRC, encoding='utf-8').read()
    doc_orig = io.open(DOC, encoding='utf-8').read()
    fails = 0
    try:
        rc, out = run_script(fast=True)
        print('基线（未注入）: EXIT=%s  %s' % (rc, 'GREEN' if rc == 0 else 'RED'))
        if rc != 0:
            print(out[-3000:])
            return 1
        n0 = out.count('[OK]')
        print('基线断言数 ≈ %d' % n0)
        print()

        for name, target, old, new, expect_red, must_appear in INJECTIONS:
            src = doc_orig if target == 'doc' else base_script
            if src.count(old) != 1:
                print('[SKIP] %-52s 原文出现 %d 次' % (name, src.count(old)))
                fails += 1
                continue
            mutated = src.replace(old, new, 1)
            if must_appear not in mutated:
                print('[SKIP] %-52s 注入后 grep 不到' % name)
                fails += 1
                continue

            if target == 'doc':
                io.open(DOC, 'w', encoding='utf-8', newline='\n').write(mutated)
                sc, so = run_script(fast=name[:2] not in NEEDS_FULL)
                io.open(DOC, 'w', encoding='utf-8', newline='\n').write(doc_orig)
            else:
                io.open(TMP_S, 'w', encoding='utf-8', newline='\n').write(mutated)
                cmd = [sys.executable, '-u', TMP_S] + (['--fast'] if True else [])
                p = subprocess.run(cmd, capture_output=True, text=True,
                                   encoding='utf-8', errors='replace', timeout=900)
                sc, so = p.returncode, (p.stdout or '') + (p.stderr or '')

            red = [l.strip() for l in so.splitlines() if '[!!]' in l]
            hit = [e for e in expect_red if any(e in l for l in red)]
            crashed = sc != 0 and not red
            ok = (sc != 0) and (len(hit) == len(expect_red) or crashed)
            print('%-7s %-52s EXIT=%-3s 变红 %d/%d %s'
                  % ('[PASS]' if ok else '[FAIL]', name, sc, len(hit), len(expect_red),
                     '(异常退出)' if crashed else ' | '.join(h[:26] for h in hit)))
            if not ok:
                fails += 1
                for l in red[:4]:
                    print('         red> %s' % l[:106])
                if crashed:
                    print('         err> %s' % so.strip().splitlines()[-1][:106])
    finally:
        # ★ 无论中途怎么退出，章都必须还原
        io.open(DOC, 'w', encoding='utf-8', newline='\n').write(doc_orig)
        if os.path.isfile(TMP_S):
            os.remove(TMP_S)

    print()
    print('=' * 70)
    print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
