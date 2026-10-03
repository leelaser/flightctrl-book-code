# -*- coding: utf-8 -*-
"""反向注入：确认 lesson51（JSBSim）的断言【真的会红】。

★ 本注入器有两种目标：
    · `script` —— 改判据本身
    · `xml`    —— 改**模型文件**（Rascal.xml / Zenoah.xml / README），
                 问"模型被改坏了，脚本抓不抓得到"
"""
import io
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

REPO = r'D:\DEVLib\PX4APM\APM\ardupilot-Copter-4.7.1'
TUT = r'D:\DEVLib\PX4APM\飞控学习教程'
SRC = os.path.join(HERE, 'lesson51_jsbsim.py')
# ★ 变异脚本必须落在原脚本所在目录（`__file__` 相对路径），否则它在读源码树那步就崩
TMP = os.path.join(tempfile.gettempdir(), '_l51_mut.py')
XMLS = {
    'rascal': 'Tools/autotest/aircraft/Rascal/Rascal.xml',
    'zenoah': 'Tools/autotest/aircraft/Rascal/Engines/Zenoah_G-26A.xml',
    'readme': 'Tools/autotest/aircraft/Rascal/README.Rascal',
}
BAKS = os.path.join(TUT, '07-仿真与实验', 'code', '_l51_bak')

INJECTIONS = [
    # ---- 打在脚本上 ----
    ('S1  谎称结构体大小是 412',
     'script', "check(size == 408,", "check(size == 412,",
     ['结构体大小'], 'size == 412,'),
    ('S2  谎称字段数只有 30 个',
     'script', "check(len(fields) == 58,", "check(len(fields) == 30,",
     ['个字段声明'], 'check(len(fields) == 30,'),
    ('S3  声称结构体里有 4 个 double',
     'script', "== ['longitude', 'latitude', 'altitude'],", "== ['longitude', 'latitude', 'altitude', 'warp'],",
     ['正好 **3 个 double**'], "'altitude', 'warp'],"),
    ('S4  把"没链接 JSBSim"判反',
     'script', "check(lib_incs == [],", "check(len(lib_incs) > 0,",
     ['没有一条 include 指向 JSBSim 库'], 'len(lib_incs) > 0,'),
    ('S5  端口算术说成差 10',
     'script', "check(5505 - 5504 == 1,", "check(5505 - 5504 == 10,",
     ['总是差 1'], '== 10,'),
    ('S6  把锁步 iterate 那条判反',
     'script', 'check(\'"iterate 1\\\\n"\' in src_cpp,',
     'check(\'"iterate 2\\\\n"\' in src_cpp,',
     ['锁步'], "'\"iterate 2"),
    # ★ 这条串在脚本里出现两次（一次在正则里、一次在断言的文案里），
    #   所以必须连**正则的转义形式**一起写才能唯一定位。
    ('S7  握手字符串改错',
     'script',
     r"""re.search(r'if \(!expect\("JSBSim Execution beginning"\)\)'""",
     r"""re.search(r'if \(!expect\("JSBSim Starting Now"\)\)'""",
     ['句握手'], 'JSBSim Starting Now'),
    ('S8  外部二进制名改错',
     'script', 'check(re.search(r\'execlp\\("JSBSim",\', src_cpp) is not None,',
     'check(re.search(r\'execlp\\("FlightAxis",\', src_cpp) is not None,',
     ['执行一个**外部可执行文件**'], 'execlp\\("FlightAxis"'),
    ('S9  声称有 SIM_JSBSIM_* 参数',
     'script', "check(re.search(r'\\bvar_info\\b', src_cpp) is None",
     "check(re.search(r'\\bvar_info\\b', src_cpp) is not None",
     ['都没有 `var_info`'], "src_cpp) is not None"),
    ('S10 三通道说成两个 socket 是同一个',
     'script', "check('SocketAPM_native sock_control;' in src_h and 'SocketAPM_native sock_fgfdm;' in src_h,",
     "check('SocketAPM_native sock_ctl;' in src_h and 'SocketAPM_native sock_fgfdm;' in src_h,",
     ['两个 socket 成员'], "'SocketAPM_native sock_ctl;'"),
    ('S11 七个顶层段少写一个',
     'script', "== ['fileheader', 'metrics', 'mass_balance', 'ground_reactions',",
     "== ['fileheader', 'metrics', 'ground_reactions',",
     ['七个顶层段'], "'fileheader', 'metrics', 'ground_reactions',"),
    ('S12 声称模型单位是公制',
     'script', "== 'LBS'", "== 'KG'",
     ['单位是**英制**'], "== 'KG'"),
    ('S13 声称树里有 4 个模型',
     'script', "check(len(allxml) == 3,", "check(len(allxml) == 4,",
     ['只有 **3** 个真正的 JSBSim 模型'], 'len(allxml) == 4,'),
    ('S14 声称四旋翼有 4 个空轴',
     'script', "check(len(qempty) == 5 and qfull == ['DRAG'],",
     "check(len(qempty) == 4 and qfull == ['DRAG'],",
     ['个空轴'], 'len(qempty) == 4'),
    ('S15 声称本地装了 JSBSim',
     'script', "check(shutil.which('JSBSim') is None,",
     "check(shutil.which('JSBSim') is not None,",
     ['没有 JSBSim 可执行文件'], "which('JSBSim') is not None,"),

    # ---- 打在模型/数据上 ----
    ('X1  Rascal.xml：把一张气动表改成不单调',
     'xml:rascal',
     '-0.2600\t0.0560\n                              0.0000\t0.0280',
     '-0.2600\t0.0560\n                             -0.5000\t0.0280',
     ['严格单调递增'], '-0.5000\t0.0280'),

    # ★ 这两条原来想"改名/清空"，但**改名会破坏 XML 良构性**
    #   （开标签改了、闭标签没改 ⇒ 解析直接失败）——红的会是"解析错误"而不是判据本身。
    #   改成**插入一个元素**：良构，且正好触发"数量/名字列表"那两条断言。
    ('X2  Rascal.xml：多插一个气动轴',
     'xml:rascal',
     '        </axis>\n    </aerodynamics>',
     '        </axis>\n        <axis name="EXTRA">\n        </axis>\n    </aerodynamics>',
     ['六个气动轴'], '<axis name="EXTRA">'),

    ('X3  Rascal.xml：删掉 AERORP 参考点',
     'xml:rascal',
     '<location name="AERORP" unit="IN">',
     '<location name="AERORPX" unit="IN">',
     ['两个参考点都在'], 'AERORPX'),

    ('X4  Rascal.xml：把 emptywt 单位改成 KG',
     'xml:rascal',
     '<emptywt unit="LBS">',
     '<emptywt unit="KG">',
     ['单位是**英制**'], '<emptywt unit="KG">'),

    ('X5  Rascal.xml：多插一个起落架接触点',
     'xml:rascal',
     '        </contact>\n    </ground_reactions>',
     '        </contact>\n        <contact type="BOGEY" name="EXTRA_LG">\n        </contact>\n    </ground_reactions>',
     ['个接触点'], '<contact type="BOGEY" name="EXTRA_LG">'),

    ('X6  Zenoah_G-26A.xml：把文件名/注释里的油机字样删掉',
     'xml:zenoah',
     '<!-- Zenoah G-26A -->',
     '<!-- the engine -->',
     ['Zenoah G-26A 汽油机'], '<!-- the engine -->'),

    ('X7  README.Rascal：删掉那句免责声明',
     'xml:readme',
     'This information has not been updated for the Rascal, please ignore, we\nare still at pre-pre-pre-alpha with this model!',
     'This information applies to the Rascal.',
     ['免责声明'], 'This information applies to the Rascal.'),
]


def run(path, fast=False):
    p = subprocess.run([sys.executable, '-u', path],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


os.makedirs(BAKS, exist_ok=True)
for k, rel in XMLS.items():
    shutil.copyfile(os.path.join(REPO, rel.replace('/', os.sep)), os.path.join(BAKS, k))
base_script = io.open(SRC, encoding='utf-8').read()

rc, out = run(SRC)
print('基线（未注入）: EXIT=%s  %s' % (rc, 'GREEN' if rc == 0 else 'RED'))
if rc != 0:
    print(out[-3000:])
    sys.exit(1)
print('基线断言数 = %d' % (out.count('[OK]') - 1))
print()

fails = 0
for name, target, old, new, expect_red, must_appear in INJECTIONS:
    if target == 'script':
        txt, path = base_script, None
    else:
        key = target.split(':')[1]
        path = os.path.join(REPO, XMLS[key].replace('/', os.sep))
        txt = io.open(path, encoding='utf-8', errors='replace').read()

    if txt.count(old) != 1:
        print('[SKIP] %-46s 原文出现 %d 次' % (name, txt.count(old)))
        fails += 1
        continue
    mutated = txt.replace(old, new, 1)
    if must_appear not in mutated:
        print('[SKIP] %-46s 注入后 grep 不到' % name)
        fails += 1
        continue

    if path:
        io.open(path, 'w', encoding='utf-8').write(mutated)
        runpath = SRC
    else:
        io.open(TMP, 'w', encoding='utf-8').write(mutated)
        runpath = TMP

    rc, out = run(runpath)

    if path:                                     # 还原
        key = target.split(':')[1]
        shutil.copyfile(os.path.join(BAKS, key), path)

    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-46s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:26] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:4]:
            print('         red> %s' % l[:106])
        if crashed:
            print('         err> %s' % out.strip().splitlines()[-1][:106])

for k, rel in XMLS.items():                      # 双保险还原
    shutil.copyfile(os.path.join(BAKS, k), os.path.join(REPO, rel.replace('/', os.sep)))
if os.path.isfile(TMP):
    os.remove(TMP)
shutil.rmtree(BAKS, ignore_errors=True)

print()
print('=' * 70)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
