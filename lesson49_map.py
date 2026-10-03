# -*- coding: utf-8 -*-
"""lesson49_map.py —— 卷 8 第 5 章《算法地图》配套自验证脚本

**本脚本的特殊性：它校验的对象是那一章正文本身。**

其余脚本校验「源码对不对」；这一个校验「地图准不准」——
把 `第05章-算法地图-ArduPilot的全部算法库.md` 里那些可以机器复核的说法重算一遍：

  [1] 14 章存在，且行数与 §5.2 表里写的一致          ← 正文里的数字会不会飘
  [2] 14 个配套脚本存在、**真跑一遍**、断言数一致
  [3] ★★★「某库在哪一章讲」：逐个重算命中数，验证主场归属
  [4] ★★★「零命中清单」：验证那些库**真的**零命中（并证明自指污染的存在）
  [5] 三条贯穿线索的证据复核（注册点 / 阈值出处 / 死值）—— 这一节读 ArduPilot 源码
  [6] 本章必须被**导出流水线**看见（文件名模式一改，build.py 会静默漏掉这一章）

★ 与上一版的区别（2026-10-02 改造）：
  上一版校验的是**素材**那一篇（`飞控学习教程/10-ArduPilot算法全解/00-总览-算法地图.md`），
  统计口径也是素材树。成书之后，读者手里只有书 —— 于是：
    · 校验对象 = 成书这一章
    · 统计范围 = **全书各章**（`书稿/卷*/第*.md`），并**排除本章自己**（自指污染）
    · 归属写法 = 本书章号（`第 6 章` / `卷 3 第 6 章`），不再是 `A1`~`A14`
  旧版备份在 `code/_lesson49_map.py.素材侧备份`。

运行： python3 lesson49_map.py        （全过 EXIT=0）
        python3 lesson49_map.py --fast  （跳过"真跑 14 个脚本"那一步）
"""

import glob
import io
import os
import re
import sys

n_ok = 0
n_bad = 0


def check(cond, msg):
    global n_ok, n_bad
    if cond:
        n_ok += 1
        print('    [OK] %s' % msg)
    else:
        n_bad += 1
        print('    [!!] %s' % msg)


HERE = os.path.dirname(os.path.abspath(__file__))       # 本书的 code/
ROOT = os.path.dirname(HERE)                            # 书根
BOOKDIR = os.path.join(ROOT, '书稿')

VOL8 = '卷8-机载软件架构与算法'
DOC = os.path.join(BOOKDIR, VOL8, '第05章-算法地图-ArduPilot的全部算法库.md')
TXT = io.open(DOC, encoding='utf-8').read()

# 源码树：本卷用 4.7.1（**不是**卷首语那份 4.5.7，两棵树的引用会错位）
REPO = None
for cand in (r'D:\DEVLib\PX4APM\APM\ardupilot-Copter-4.7.1',
             r'D:\DEVLib\PX4APM\APM\ardupilot-Plane-4.7.1'):
    if os.path.isdir(os.path.join(cand, 'libraries')):
        REPO = cand
        break

CH = re.compile(r'^第(\d+)章.*\.md$')


def all_chapters():
    """全书各章（含本章自己）—— 用于统计与"排除本章"的对照。"""
    out = []
    for vol in sorted(os.listdir(BOOKDIR)):
        vd = os.path.join(BOOKDIR, vol)
        if not os.path.isdir(vd):
            continue
        for f in sorted(os.listdir(vd)):
            if CH.match(f):
                out.append(os.path.join(vd, f))
    return out


DOCS = all_chapters()
CHAP_OF = {}          # (卷号, 章号) → 路径


def _scan_index():
    for p in DOCS:
        vol = os.path.basename(os.path.dirname(p))
        m = re.match(r'^卷(\d+)-', vol)
        c = CH.match(os.path.basename(p))
        if m and c:
            CHAP_OF[(int(m.group(1)), int(c.group(1)))] = p


_scan_index()


def hits(lib, path):
    t = io.open(path, encoding='utf-8', errors='replace').read()
    return len(re.findall(r'\b%s\b' % re.escape(lib), t))


def lib_total_excl(lib, exclude_self=True):
    return sum(hits(lib, p) for p in DOCS
               if not (exclude_self and os.path.abspath(p) == os.path.abspath(DOC)))


print('=' * 76)
print('[0] 定位')
print('=' * 76)
check(os.path.isfile(DOC), '读到 %s' % os.path.basename(DOC))
check(REPO is not None, '找到 ArduPilot 源码树（4.7.1）')
check(len(DOCS) > 125, '全书扫到 %d 章' % len(DOCS))
check((8, 5) in CHAP_OF, '本章（卷 8 第 5 章）在索引里 —— 统计时能把它自己排除掉')


# ==========================================================================
print()
print('=' * 76)
print('[1] 14 章：存在性 + 行数 vs §5.2 表')
print('=' * 76)

# ▲ 表行两种写法：`| 第 6 章 | 488 | …` 与 `| 卷 10 第 8 章 | 1195 | …`
rows = re.findall(r'^\|\s*(?:卷\s*(\d+)\s*)?第\s*(\d+)\s*章\s*\|\s*(\d+)\s*\|', TXT, re.M)
check(len(rows) == 14, '§5.2 速查表抽到 **%d** 章' % len(rows))

bad_lines = []
for v, c, ln in rows:
    vno, cno = (int(v) if v else 8), int(c)
    p = CHAP_OF.get((vno, cno))
    if p is None:
        bad_lines.append('卷%d 第%d 章(找不到文件)' % (vno, cno))
        continue
    real = len(io.open(p, encoding='utf-8', errors='replace').read().splitlines())
    if real != int(ln):
        bad_lines.append('卷%d 第%d 章 表%s/实%s' % (vno, cno, ln, real))
check(not bad_lines,
      '14 章行数与表里写的一致（不一致的：%s）' % ('无' if not bad_lines else '、'.join(bad_lines)))

total = sum(int(ln) for _v, _c, ln in rows)
m_decl = re.search(r'正文\s*\|[^|]*?(\d+)\s*行', TXT)
decl = int(m_decl.group(1)) if m_decl else None
check(decl == total,
      '表格合计 %d 行 == §5.1 声明的 %s 行（**真读了那条声明**，不是两边写死同一个数）'
      % (total, decl))


# ==========================================================================
print()
print('=' * 76)
print('[2] 十四个配套脚本：存在性 + 断言数（真跑）')
print('=' * 76)

import subprocess

scripts = re.findall(r'`(lesson\d+_[a-z0-9_]+\.py)`\s*\((\d+)\)', TXT)
uniq = {}
for s, n in scripts:
    uniq[s] = int(n)
check(len(uniq) == 14, '速查表里点到 **%d** 个脚本' % len(uniq))

# ★ 表里那个括号数字是**断言数**，不是行数。要核它只能真跑 ——
#   这同时也回答另一件事：**地图说这 14 个脚本是绿的，现在还绿吗？**
FAST = '--fast' in sys.argv
bad_s, ran = [], 0
for s, want in sorted(uniq.items()):
    p = os.path.join(HERE, s)
    if not os.path.isfile(p):
        bad_s.append(s + '(缺文件)')
        continue
    if FAST:
        continue
    try:
        r = subprocess.run([sys.executable, '-u', p], capture_output=True, text=True,
                           encoding='utf-8', errors='replace', timeout=600)
    except subprocess.TimeoutExpired:
        bad_s.append(s + '(超时)')
        continue
    m = re.search(r'全部\s+(\d+)\s+项断言通过', r.stdout or '')
    if r.returncode != 0 or not m:
        bad_s.append('%s 现在**跑不过**（EXIT=%s）' % (s, r.returncode))
        continue
    ran += 1
    if int(m.group(1)) != want:
        bad_s.append('%s 表%d/实%d' % (s, want, int(m.group(1))))
if FAST:
    print('         [--fast] 跳过"真跑 14 个脚本"；14 个脚本文件本身仍逐一核过存在性')
else:
    check(not bad_s,
          '★★★ 逐一**真跑**了这 14 个脚本：实跑断言数与表里写的一致，且**全部通过**'
          '（%s）' % ('无不一致' if not bad_s else '、'.join(bad_s)))
    print('         实跑 %d 个脚本，累计断言 %d 条' % (ran, sum(uniq.values())))


# ==========================================================================
print()
print('=' * 76)
print('[3] ★★★ 「某库在哪一章讲」：逐个重算')
print('=' * 76)

claims = []
for line in TXT.split('\n'):
    m = re.match(r'^\|\s*`((?:AP|AC)_\w+)`\s*\|[^|]*\|\s*(.+?)\s*\|\s*$', line)
    if m:
        claims.append((m.group(1), m.group(2)))
check(len(claims) >= 40, '§5.3 各表共抽到 **%d** 条「库 → 归属」行' % len(claims))

# 归属单元格的两种写法：`**第 6 章**（44 处）` / `**卷 3 第 6 章**（17）`
HOME = re.compile(r'\*\*(?:卷\s*(\d+)\s*)?第\s*(\d+)\s*章\*\*\s*[（(](\d+)')

checked = no_claim = 0
wrong_home = []
for lib, cell in claims:
    m = HOME.search(cell)
    if not m:
        no_claim += 1
        continue
    checked += 1
    vno = int(m.group(1)) if m.group(1) else 8      # 不带「卷」= 本卷
    cno, want = int(m.group(2)), int(m.group(3))
    p = CHAP_OF.get((vno, cno))
    if p is None:
        wrong_home.append('%s 的归属「卷%d 第%d 章」—— 那一章不存在' % (lib, vno, cno))
        continue
    real = hits(lib, p)
    if real != want:
        wrong_home.append('%s 说在卷%d 第%d 章 %d 处，实测 %d 处' % (lib, vno, cno, want, real))
        continue
    # 且必须是**最密**的那一章（允许并列）
    others = [(hits(lib, q), q) for q in DOCS
              if os.path.abspath(q) != os.path.abspath(DOC)]
    others.sort(reverse=True)
    if others and others[0][0] > real:
        wrong_home.append('%s 的最大命中在 %s（%d 处 > %d）'
                          % (lib, os.path.basename(others[0][1]), others[0][0], real))

check(not wrong_home,
      '★★★ 有明确归属声明的 **%d** 条全部复核通过：'
      '（a）命中数与表里一致、（b）那一章确实是全书最密的' % checked)
if wrong_home:
    for w in wrong_home[:8]:
        print('           ✗ %s' % w)
check(no_claim >= 4,
      '★ 另有 %d 条**故意不声称唯一主场**（写成「零命中」「分散」「只被带过」）'
      '—— 不纳入复核，因为断言它们本身没有分辨力' % no_claim)


# ==========================================================================
print()
print('=' * 76)
print('[4] ★★★ 零命中清单：验证它们**真的**零命中')
print('=' * 76)

ZERO = ['AP_Airspeed', 'AP_Camera', 'AP_OSD', 'AP_RPM', 'AP_Tuning',
        'AC_Sprayer', 'AC_InputManager', 'AP_EFI', 'AP_ExternalControl']

# ★★★ 自指陷阱：**本章自己逐条列出了这些库名**，于是"零命中"立刻不成立。
#   ⇒ 判据必须**排除本章**；而且章里那段说明也把污染数写出来了（下面一并断言）。
check(lib_total_excl('AP_Mission') > 30,
      '★★ 先自证量具：`AP_Mission` 命中 %d 处（非零）⇒ 统计函数确实在读文件'
      % lib_total_excl('AP_Mission'))

for lib in ZERO:
    tot = lib_total_excl(lib)
    check(tot == 0, '`%s` **除本章外**命中 **%d** 处 ⇒ 「全书零命中」成立' % (lib, tot))

# 章里那段"自指证据"写的数，要跟实测对上（不然就是正文数字飘了）
for lib in ('AP_Airspeed', 'AP_Tuning', 'AP_ExternalControl', 'AC_InputManager'):
    m = re.search(r'%s\s+(\d+)\s*处' % re.escape(lib), TXT)
    real = lib_total_excl(lib, exclude_self=False)      # 含本章
    check(m is not None and int(m.group(1)) == real,
          '★★★ 章里写的「%s %s 处（含本章）」== 实测 %d 处 —— '
          '地图确实在污染自己的统计' % (lib, m.group(1) if m else '?', real))

# ★★ 覆盖缺口：ZERO 是**脚本里硬编码**的。文档表里删一行，脚本毫无反应 ⇒
#   把文档 §5.3.8 表里的库名抽出来，与 ZERO 做**集合相等**比较。
sec38 = TXT.split('### 5.3.8')[1].split('\n## ')[0] if '### 5.3.8' in TXT else ''
doc_zero = re.findall(r'^\|\s*\*{0,2}`((?:AP|AC)_\w+)`', sec38, re.M)
check(sorted(doc_zero) == sorted(ZERO),
      '★★★ 文档 §5.3.8 表里的零命中库（%d 个）与脚本里的清单**集合相等**' % len(doc_zero))
if sorted(doc_zero) != sorted(ZERO):
    check(False, '不一致：只在文档里 %s；只在脚本里 %s'
          % (sorted(set(doc_zero) - set(ZERO)), sorted(set(ZERO) - set(doc_zero))))

check(lib_total_excl('AC_Fence') > 0,
      '★★ 反向对照：`AC_Fence` 非零（%d 处）⇒ 清单不是"随便写什么都过"'
      % lib_total_excl('AC_Fence'))


# ==========================================================================
print()
print('=' * 76)
print('[5] 三条线索的证据复核（读 ArduPilot 源码）')
print('=' * 76)

if REPO is None:
    check(False, '没有源码树，跳过本节')
else:
    def slurp(rel):
        return io.open(os.path.join(REPO, rel.replace('/', os.sep)),
                       encoding='utf-8', errors='replace').read()

    # --- 线索① 注册点 ≠ 定义点 ---
    apv = slurp('libraries/AP_Vehicle/AP_Vehicle.cpp')
    check(re.search(r'AP_SUBGROUPINFO\(fence,\s*"FENCE_"', apv) is not None,
          '① `FENCE_` 的注册点在 `AP_Vehicle.cpp`（不在 `AC_Fence` 所在载具里）')
    sitl = slurp('libraries/SITL/SITL.cpp')
    check(re.search(r'AP_GROUPINFO\("THML_SCENARI"', sitl) is not None,
          '① `SIM_THML_SCENARI` 在源码里只有 `THML_SCENARI`（前缀是拼的）')
    check('SIM_THML_SCENARI' not in sitl,
          '① 反证：源码里**搜不到**带 `SIM_` 的全名')

    # --- 线索② 阈值有出处 ---
    check(abs(1 - 1 / 2.718281828459045 - 0.6321205588285577) < 1e-15,
          '② `1 − 1/e = 0.6321…` 复算成立（第 16 章那个判据的出处）')
    check(abs(1 / 2.718281828459045 - 0.3678794411714423) < 1e-15,
          '② `e⁻¹ = 0.3679…` 复算成立（第 18 章 / 卷 10 第 8 章那个高斯衰减）')

    # --- 线索③ 声明了但永不使用 ---
    soal = slurp('libraries/AP_Soaring/AP_Soaring.cpp')
    m = re.search(r'float SoaringController::McCready\(float alt\)\s*\{(.*?)\}', soal, re.S)
    check(m is not None and 'A method shell to be filled in later' in m.group(1)
          and 'alt' not in re.sub(r'float alt', '', m.group(1)),
          '③ `McCready(alt)` 是空壳：函数体里**用不到 `alt`**')
    vario = slurp('libraries/AP_Soaring/Variometer.cpp') + slurp('libraries/AP_Soaring/Variometer.h')
    check(len(re.findall(r'_raw_climb_rate', vario)) == 2,
          '③ `_raw_climb_rate` 全库只 2 次（声明 + 读日志）⇒ 从不赋值')
    fence = slurp('libraries/AC_Fence/AC_Fence.cpp')
    check('GIVE_UP_DISTANCE' not in fence or 'not used by library directly' in fence,
          '③ `AC_FENCE_GIVE_UP_DISTANCE` 在库里不被使用（注释自陈）')
    check('GIVE_UP_DISTANCE' in slurp('ArduCopter/fence.cpp'),
          '③ 而载具在用 ⇒ "库给口径、载具决定" 的分工成立')

    # --- 卷 10 第 8 章的未读字段 ---
    fah = slurp('libraries/SITL/SIM_FlightAxis.h')
    fap = slurp('libraries/SITL/SIM_FlightAxis.cpp')
    keys = re.findall(r'\{\s*"([^"]+)",\s*next_state\.(\w+)\s*\}', fah)
    unused = [k for k, f in keys if k != 'item'
              and not re.search(r'\b(?:next_)?state\.%s\b' % re.escape(f), fap)]
    check(len(unused) == 17,
          '③ RealFlight 协议有 **%d** 个具名字段从不被读（地图说 17）' % len(unused))


# ==========================================================================
print()
print('=' * 76)
print('[6] ★★★ 导出覆盖回归：本章必须被 build.py 装进书里')
print('=' * 76)

# 这一条防的是**静默漏章**：build.py 用 `^第(\d{2})章-(.+)\.md$` 挑章文件，
# 文件名一改（比如少个零、多个空格），它不会报错，只是**这一章不出现在书里**。
CHAP_RE = re.compile(r'^第(\d{2})章-(.+)\.md$')
bn = os.path.basename(DOC)
check(CHAP_RE.match(bn) is not None,
      '★★★ 文件名 `%s` 符合 build.py 的挑章模式（否则它会静默漏掉这一章）' % bn)
check(bn.startswith('第05章'),
      '★ 章号两位补零（`第05章` 而不是 `第5章`）—— 排序与编号都依赖它')

# 与体例检查器的扫描范围对齐：check_book / check_private_refs 都按 `卷*/第*.md` 找章
check(os.path.basename(os.path.dirname(DOC)).startswith('卷'),
      '★ 本章在 `卷*/` 目录下（检查器按这个形状找章）')
scan = [p for p in DOCS if re.match(r'^第\d+章.*\.md$', os.path.basename(p))]
check(os.path.abspath(DOC) in [os.path.abspath(x) for x in scan],
      '★★ 本章在"体例检查器会扫的章"清单里（%d 章）' % len(scan))


# ==========================================================================
print()
print('=' * 76)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过 —— 这张地图经得起复核' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
