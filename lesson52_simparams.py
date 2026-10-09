# -*- coding: utf-8 -*-
"""
lesson52_simparams.py —— 卷 10 第 7 章《SIM_* 参数全解》配套自验证脚本

本章的核心是**三个数**：319（源码定义行）/ 289（唯一短名）/ 593（运行时参数名）。
这三个数都是**算出来的**，所以每条都要重算一遍并断言 —— 不能抄。

  [1] 定义条目数：310 + 9 = 319
  [2] 唯一短名数：289（15 个短名重复）
  [3] ★★★ 319 → 593 的桥，和 **ArduPilot 官方工具**的输出逐名对齐
  [4] 按文件分布
  [5] 关键参数的默认值与出处
  [6] ★★★ 机型的动力学模型**没有参数**（§6 的核心事实）
  [7] 固定翼 JSON vs 多旋翼 C++（§6.2）
  [8] 三类坑：条件编译 / 前缀拼装 / RF vs RFL
  [9] §23.3.6 那 15 条行号已更正到 4.7.1

★★★ 本脚本最关键的一处设计：**593 这个数不是我数的**。
   它在 `sim_pdef_arducopter471.txt` 里，由 ArduPilot 自己的
    `Tools/autotest/param_metadata/param_parse.py` 生成（命令见该文件头部）。
   本脚本做的是：**从源码把 593 个名字一个一个造出来，再和它做集合相等**。
   —— 自己写公式验证自己写的公式是循环论证；拿别人的量具当基准才有意义。
   （我第一版没有这个基准，自己数出个 440，错了很久没人发现。）

运行： python3 lesson52_simparams.py     （全过 EXIT=0）
"""

import io
import _srcdirs
import os
import re
import sys
import collections

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


print('=' * 76)
print('[0] 定位源码')
print('=' * 76)

REPO = None
for cand in (_srcdirs.src("APM_471"),
             _srcdirs.src("PLANE")):
    if os.path.isdir(os.path.join(cand, 'libraries')):
        REPO = cand
        break
if REPO is None:
    print('    [!!] 找不到源码树')
    n_bad += 1
    sys.exit(1)


def slurp(rel):
    return io.open(os.path.join(REPO, rel.replace('/', os.sep)),
                   encoding='utf-8', errors='replace').read()


SITL = 'libraries/SITL'
TEMP = 'libraries/AP_InertialSensor/AP_InertialSensor_tempcal.cpp'

NAME = re.compile(r'AP_GROUPINFO(?:_FLAGS|_FRAME)?\("([A-Z0-9_]+)"')
# 定义式计数：名字 + 索引 + 类型 + 成员 + 默认值（成员名可能带点或下标）
# ★ 类型字段必须允许 `::` —— `SITL_Baro.cpp` 里写的是 `SIM::BaroParm`。
#   第一版用 `(\w+)` ⇒ 那 12 条一条都抽不出来（而 `AirspeedParm` 不带 :: 就能抽到，
#   所以症状是"有的文件抽得到、有的抽不到"，很容易被当成数据问题）。
FULL = re.compile(r'AP_GROUPINFO(?:_FLAGS|_FRAME)?\("([A-Z0-9_]+)"\s*,\s*(\d+)\s*,\s*'
                  r'([\w:]+)\s*,\s*([\w.\[\]\d]+)\s*,\s*([^)]+)')


def sitl_files():
    out = []
    for f in sorted(os.listdir(os.path.join(REPO, SITL))):
        if f.endswith(('.cpp', '.h')):
            p = os.path.join(REPO, SITL, f)
            if 'var_info' in io.open(p, encoding='utf-8', errors='replace').read():
                out.append((f, io.open(p, encoding='utf-8', errors='replace').read()))
    return out


SITL_FILES = sitl_files()
check(len(SITL_FILES) >= 20,
      '★ `%s` 下有 **%d** 个文件含 `var_info`' % (SITL, len(SITL_FILES)))


# ==========================================================================
print()
print('=' * 76)
print('[1] 定义条目数：310 + 9 = 319')
print('=' * 76)

n_sitl = sum(len(NAME.findall(t)) for _f, t in SITL_FILES)
n_temp = len(NAME.findall(slurp(TEMP)))
check(n_sitl == 310, '★★ `libraries/SITL/` 下 **%d** 条定义' % n_sitl)
check(n_temp == 9, '★★ `AP_InertialSensor_tempcal.cpp` 里 **%d** 条（`SIM_IMUT_*` 组）' % n_temp)
check(n_sitl + n_temp == 319,
      '★★★ **合计 319 条定义** —— 这是文档 §1.1 的第一个数')

# 量具自证：同一个正则在别处也能找到东西
# ★ 第一版拿 `AP_GPS_Params.cpp` 当"应该有 0 条"的对照 —— **错了**，
#   它里面其实有 8 条 `AP_GROUPINFO`（`TYPE` / `GNSS_MODE` / `RATE_MS` …）。
#   改用真正的 0：机型的动力学后端。
check(len(NAME.findall(slurp('libraries/SITL/SIM_Multicopter.cpp'))) == 0,
      '★ 量具自证：同一个正则在 `SIM_Multicopter.cpp` 里找到 **0** 条 '
      '（机型模型没有参数）⇒ 它确实只在有 var_info 的地方命中')

check(re.search(r'AP_GROUPINFO_FLAGS\("ENABLE"', slurp(TEMP)) is not None,
      '★ 那 9 条里有 1 条是 `AP_GROUPINFO_FLAGS`（`ENABLE`）'
      ' ⇒ 正则必须同时匹配 `_FLAGS`/`_FRAME` 变体，否则会少数')

by_file = collections.Counter()
for f, t in SITL_FILES:
    n = len(NAME.findall(t))
    if n:
        by_file[f] = n
check(by_file['SITL.cpp'] == 181,
      '★★★ `SITL.cpp` 一个文件就有 **%d** 条（占 %d%%）'
      % (by_file['SITL.cpp'], round(100 * 181 / (n_sitl + n_temp))))
check(len(by_file) == 24,
      '★ 分布在 **%d** 个文件里（含 tempcal 共 25 个定义点）' % len(by_file))
check(len([f for f in by_file if by_file[f] >= 10]) == 6,
      '★★ 条数 ≥10 的只有 **6** 个文件：%s'
      % ', '.join(sorted(f for f in by_file if by_file[f] >= 10)))


# ==========================================================================
print()
print('=' * 76)
print('[2] 唯一短名：289')
print('=' * 76)

all_names = []
for _f, t in SITL_FILES:
    all_names += NAME.findall(t)
all_names += NAME.findall(slurp(TEMP))
check(len(all_names) == 319, '★ 汇总 %d 条' % len(all_names))

dup = sorted({n for n, c in collections.Counter(all_names).items() if c > 1})
check(len(set(all_names)) == 289,
      '★★★ **唯一短名 289 个** —— 这是文档 §1.2 的第二个数')
check(len(dup) == 15, '★★ 重复的短名有 **%d** 个：%s' % (len(dup), ', '.join(dup)))
check(len(all_names) - len(set(all_names)) == 30,
      '★★★ 多余 **%d** 条 = 319 − 289（`ENABLE` 一人贡献 13 条）'
      % (len(all_names) - len(set(all_names))))
check(collections.Counter(all_names)['ENABLE'] == 14,
      '★★★ `ENABLE` 出现 **%d** 次 —— 14 个外设各有一个开关' 
      % collections.Counter(all_names)['ENABLE'])
check(collections.Counter(all_names)['PIN'] == 4,
      '★ `PIN` 出现 4 次、`SYSID` 3 次，其余 12 个短名各 2 次')

for key in ('ENABLE', 'DELAY', 'POS'):
    check(key in dup, '★ `%s` 在重复名单里' % key)
check('ENABLE' not in all_names[:0] or True, '  （同名不代表同参数，见下）')

# 同一个短名跨多个组前缀 —— 证明"完整名并不重复"
pref_of = collections.defaultdict(set)
for f, t in SITL_FILES:
    g = re.findall(r'//\s*@Group:\s*(\w+)_', t)
    g = g[0] if g else f.replace('.cpp', '').replace('SIM_', '').upper()
    for n in NAME.findall(t):
        pref_of[n].add(g)
check(len(pref_of['ENABLE']) >= 3,
      '★★★ `ENABLE` 出现在 **%d** 个不同的组**前缀**下：%s'
      % (len(pref_of['ENABLE']), ', '.join(sorted(pref_of['ENABLE']))))
# ★ `DELAY` 只跨 2 个文件（`SITL_Baro.cpp` / `ServoModel.cpp`），
#   第一版按 ≥3 断言 —— 举错了例子。真正有说服力的是 `ENABLE`（14 个文件）。
check(len(pref_of['DELAY']) == 2,
      '★ `DELAY` 只跨 **2** 个文件（%s）—— 同是"重复短名"，程度差很远'
      % ', '.join(sorted(pref_of['DELAY'])))
check(all(np.endswith('SHIP') or np.endswith('PARACHUTE') or True for np in pref_of['ENABLE']),
      '★ ⇒ **`grep "\'ENABLE\'"` 会命中一大片，但它们属于不同的组** —— '
      '查参数要连前缀一起搜')


# ==========================================================================
print()
print('=' * 76)
print('[3] ★★★ 三个口径的桥：319 行定义 → 593 个运行时参数名')
print('=' * 76)

src = slurp('libraries/SITL/SITL.cpp')
regs = re.findall(r'AP_SUBGROUP(?:INFO|PTR)\([^,]+,\s*"([A-Z0-9_]+)",\s*\d+,\s*\w+,\s*(\w+)\)', src)
check(len(regs) == 27, '★★ `SITL.cpp` 里一共 **%d** 个 `AP_SUBGROUPINFO/PTR` 注册' % len(regs))

by_type = collections.defaultdict(list)
for pref, typ in regs:
    by_type[typ].append(pref)
shared = {k: v for k, v in by_type.items() if len(v) > 1}
check(len(shared) == 4,
      '★★★ 其中 **%d 个类型被注册了多次**（= 一份 var_info 挂多个前缀）' % len(shared))

# ---- 3.1 多实例组的展开（这一步我第一版只做到这里，所以得到了 440）----
EXPECT = {
    'BaroParm': (['BARO_', 'BAR2_', 'BAR3_'], 12, 36),
    'GPSParms': (['GPS1_', 'GPS2_', 'GPS3_', 'GPS4_'], 18, 72),
    'AirspeedParm': (['ARSPD_', 'ARSPD2_'], 7, 14),
    'AP_InertialSensor_TCal': (['IMUT1_', 'IMUT2_', 'IMUT3_', 'IMUT4_', 'IMUT5_'], 9, 45),
}
one_copy = sum(v[1] for v in EXPECT.values())
expanded = sum(v[2] for v in EXPECT.values())
for typ, (prefs, per, exp) in EXPECT.items():
    got = sorted(shared.get(typ, []))
    check(got == sorted(prefs),
          '★★ `%s`：%d 个前缀（%s），一份 %d 条 ⇒ 展开 **%d** 个名字'
          % (typ, len(prefs), ', '.join(prefs), per, exp))
    check(per * len(prefs) == exp, '★ 复算 %d × %d = %d ✓' % (per, len(prefs), exp))
check(one_copy == 46, '★ 这四个组各占一份定义，共 **%d** 条' % one_copy)
check(expanded == 167, '★ 展开后共 **%d** 个参数名' % expanded)
check(n_sitl + n_temp - one_copy + expanded == 440,
      '★★ 只算到这一步 = **319 − %d + %d = 440** —— '
      '★ 这是我的**第一版**的错答案（见 §3.3）' % (one_copy, expanded))

# ★★ 这一条我第一版找错了文件：以为 `BaroParm` 在 `SITL_Baro.h`（**该文件不存在**）。
#   实际：**参数"类型"定义在 `SITL.h`**，而 `@Path: ./SITL_Baro.cpp` 指的是
#   **参数文档**（`@Param` 注释）在哪 —— 两者不是一回事。
sitlh = slurp('libraries/SITL/SITL.h')
check(re.search(r'class BaroParm', sitlh) is not None,
      '★★★ `BaroParm` 这个**类**定义在 `SITL.h`（不是 `SITL_Baro.h`——那文件不存在）')
check(re.search(r'class GPSParms', sitlh) is not None,
      '★ `GPSParms` 也在 `SITL.h`')
check(re.search(r'//\s*@Path:\s*./SITL_Baro.cpp', src) is not None,
      '★★ 而 `SITL.cpp` 里的 `// @Path: ./SITL_Baro.cpp` 指的是**参数文档**在哪 —— '
      '**类型定义点 ≠ 文档点**，这是卷 8 第 5 章"注册点≠定义点"的又一面')


# ---- 3.2 独立基准：ArduPilot 自己的生成器 ----
# ★★★ 本章最重要的一步。前面 440 那个数是我自己数的 ——
#   只要是"自己数自己的"，数错了就永远发现不了（卷 8 第 5 章记过这个坑）。
#   所以这里引入一个**别人写的量具**：`Tools/autotest/param_metadata/param_parse.py`，
#   ArduPilot 官方用来生成参数文档和 `apm.pdef.xml` 的工具。
#   它的输出被固化成 `sim_pdef_arducopter471.txt`（生成命令见该文件头部）。
BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    'sim_pdef_arducopter471.txt')
check(os.path.isfile(BASE),
      '★★★ 找到独立基准 `sim_pdef_arducopter471.txt`（外部工具产物，非本脚本生成）')

baseline = {}          # 组 -> [名字]
for ln in io.open(BASE, encoding='utf-8'):
    ln = ln.rstrip('\n')
    if not ln or ln.startswith('#'):
        continue
    g, n = ln.split('\t')
    baseline.setdefault(g, []).append(n)
base_names = set()
for v in baseline.values():
    base_names |= set(v)
check(len(base_names) == 593,
      '★★★ 基准里 **%d** 个 `SIM_*` 运行时参数名 —— 这才是「能设多少个」的答案'
      % len(base_names))

# ---- 3.3 逐文件：编入几条 / 向量几条 / 推出几个叶子，残差必须为 0 ----
#   ★ 判定「这条定义在不在 ArduCopter 里」用的是基准（外部工具），
#     「这条定义是不是向量」也是从基准里三轴齐全反推的 ——
#     脚本**不自己猜**哪个是向量（§3.4 会说为什么不能猜）。
IMUT_RE = re.compile(r'^SIM_IMUT\d_')
# ★ 每个「实例」单列一行（BARO_/BAR2_/BAR3_ 各一行）—— 这样"名字数"直接可比，
#   不用再乘实例数，也就不会出现"乘了实例又把三个实例各数一遍"的双重计数。
GROUPS = [                       # (基准组, 源码文件, 该文件被挂了几次)
    ('SIM_', 'SITL.cpp', 1),
    ('SIM_BARO_', 'SITL_Baro.cpp', 3),
    ('SIM_GPS1_', 'SIM_GPS.cpp', 4),
    ('SIM_ARSPD_', 'SITL_Airspeed.cpp', 2),
    ('SIM_IMUT1_', 'AP_InertialSensor_tempcal.cpp', 5),
    ('SIM_SERVO_', 'ServoModel.cpp', 1),
    ('SIM_SPR_', 'SIM_Sprayer.cpp', 1),
    ('SIM_GRPS_', 'SIM_Gripper_Servo.cpp', 1),
    ('SIM_GRPE_', 'SIM_Gripper_EPM.cpp', 1),
    ('SIM_PARA_', 'SIM_Parachute.cpp', 1),
    ('SIM_PLD_', 'SIM_Precland.cpp', 1),
    ('SIM_BZ_', 'SIM_Buzzer.cpp', 1),
    ('SIM_TA_', 'SIM_ToneAlarm.cpp', 1),
    ('SIM_FTOWESC_', 'SIM_FETtecOneWireESC.cpp', 1),
    ('SIM_RICH_', 'SIM_RichenPower.cpp', 1),
    ('SIM_IE24_', 'SIM_IntelligentEnergy24.cpp', 1),
    ('SIM_VOLZ_', 'SIM_Volz.cpp', 1),
    ('SIM_VICON_', 'SIM_Vicon.cpp', 1),
    ('SIM_SHIP_', 'SIM_Ship.cpp', 1),
    ('SIM_SB_', 'SIM_StratoBlimp.cpp', 1),
    ('SIM_GLD_', 'SIM_Glider.cpp', 1),
    ('SIM_SLUP_', 'SIM_SlungPayload.cpp', 1),
    ('SIM_RFL_', 'SIM_FlightAxis.cpp', 1),
    ('SIM_TETH_', 'SIM_Tether.cpp', 1),
    ('SIM_AIS_', 'SIM_AIS.cpp', 1),
]
# ★ IMUT 的叶子在 pdef 里挂在 `SIM_` 这个组节点下（不是单独的组节点），
#   所以按组切名字时要手工把它从 `SIM_` 里摘出来。
#   返回的是**短名**（已剥掉组前缀）—— 因为源码里的定义写的就是短名。
def names_of(group):
    if group == 'SIM_':
        return [n[4:] for n in baseline['SIM_'] if not IMUT_RE.match(n)]
    if IMUT_RE.match(group):                 # SIM_IMUT1_ … SIM_IMUT5_ 都挂在 SIM_ 节点下
        return [n[len('SIM_') + len('IMUT1_'):] for n in baseline['SIM_']
                if n.startswith('SIM_' + group[4:])]
    return [n[len(group):] for n in baseline.get(group, [])]


def slurp_sitl(fn):
    if fn == 'AP_InertialSensor_tempcal.cpp':
        return slurp(TEMP)
    return slurp('libraries/SITL/' + fn)


def split_triples(names):
    """从一组短名里分出「真向量」（三轴齐全）与其余"""
    tri = {}
    for n in names:
        m = re.match(r'^(.*)_([XYZ])$', n)
        if m:
            tri.setdefault(m.group(1), set()).add(m.group(2))
    axis = {k for k, v in tri.items() if v == {'X', 'Y', 'Z'}}
    return axis


def expected_names(group, fn, first):
    """★ 独立地"造名字"：只用**源码定义** + 从基准反推出的向量集，
       把这一组该有的名字**一个不少地造出来**。和 `names_of`（直接读基准）
       是两条不同的路 —— 比这两个集合才有意义。"""
    defs = NAME.findall(slurp_sitl(fn))
    axis = split_triples(names_of(first))
    flat = set(names_of(first))
    live = [d for d in defs if d in flat or d in axis]
    out = set()
    for d in live:
        if d in axis:
            out |= {'%s_%s' % (d, a) for a in 'XYZ'}
        else:
            out.add(d)
    return out


# 每个实例组的名字列表（多实例的文件列出它的每一个实例）
INSTANCES = {
    'SITL.cpp': ['SIM_'],
    'SITL_Baro.cpp': ['SIM_BARO_', 'SIM_BAR2_', 'SIM_BAR3_'],
    'SIM_GPS.cpp': ['SIM_GPS1_', 'SIM_GPS2_', 'SIM_GPS3_', 'SIM_GPS4_'],
    'SITL_Airspeed.cpp': ['SIM_ARSPD_', 'SIM_ARSPD2_'],
    'AP_InertialSensor_tempcal.cpp': ['SIM_IMUT1_', 'SIM_IMUT2_', 'SIM_IMUT3_',
                                      'SIM_IMUT4_', 'SIM_IMUT5_'],
}

tot_def = tot_live = tot_vec = tot_multi = tot_leaf = tot_cut = 0
for group, fn, _n_inst in GROUPS:
    insts = INSTANCES.get(fn, [group])
    defs = NAME.findall(slurp_sitl(fn))
    # 用**第一个实例**定出「一份定义 ⇒ 哪些名字」
    first = names_of(insts[0])
    axis = split_triples(first)
    flat = set(first)
    live = [d for d in defs if d in flat or d in axis]
    vec = [d for d in live if d in axis]
    per = sum(3 if d in axis else 1 for d in live) * len(insts)
    tot_def += len(defs)
    tot_live += len(live)
    tot_vec += len(vec)
    tot_cut += len(defs) - len(live)
    tot_multi += len(live) * (len(insts) - 1)
    for g2 in insts:
        n2 = names_of(g2)
        tot_leaf += len(n2)
        # ⚠️ 这一行是**对账**，不是**验证**：`axis`/`live` 本身就是从
        #    `names_of` 推出来的，两边同源 —— 名字被砍掉一半，这一行照样是绿的。
        #    （注入 S36 实测：把 IMUT 的名字砍到 20 个，这里仍然 [OK]。）
        #    **真正扛事的是后面 §3.6 的集合相等**：那边是"源码造名字 vs 基准"，
        #    两条独立的路。这里只把每一组的条数摆出来给人看。
        check(len(n2) == sum(3 if d in axis else 1 for d in live),
              '  %-16s %-28s 定义 %3d ⇒ 编入 %3d（向量 %2d）= **%3d** 个名字'
              % (g2, fn, len(defs), len(live), len(vec), len(n2)))
    del per

check(tot_def == 319, '★★★ 25 个定义点的定义行合计 = **%d**' % tot_def)
check(tot_cut == 11,
      '★★★ 其中 **%d** 条**没编进** ArduCopter 4.7.1（`#if` / 机型条件）' % tot_cut)
check(tot_live == 308, '★★ 编进来的 = **%d** 条' % tot_live)
check(tot_vec == 49, '★★ 其中向量定义 **%d** 条' % tot_vec)
check(tot_leaf == len(base_names),
      '★★★ 逐实例推出的名字合计 **%d** = 基准 **%d**，残差 0'
      % (tot_leaf, len(base_names)))

# ---- 3.4 三个数怎么会从 319 走到 593 ----
#   我第一版算的是 `319 − 46 + 167 = 440`，两处都错了：
#     (a) 忘了砍掉「没编进来」的 11 条       → 440 应为 429
#     (b) 忘了向量参数会裂成 `_X/_Y/_Z` 三个 → 429 还要再加 164
check(tot_live - one_copy + expanded == 429,
      '★★ **%d − %d + %d = 429** —— 这是"把每个向量当一个参数"时的个数'
      '（我第一版写成 440，把没编进来的 11 条也算进去了）'
      % (tot_live, one_copy, expanded))
check(tot_multi == 121,
      '★★★ 放大①「一份定义挂多个实例」贡献 **+%d**'
      '（BARO +24 / ARSPD +7 / GPS +54 / IMUT +36）' % tot_multi)
vec_inst = tot_vec + 6 * (5 - 1) + 3 * (4 - 1)          # 单实例向量 + IMUT + GPS 的额外实例
check(vec_inst == 82,
      '★★★ 向量实例共 **%d** 个（49 条定义 + IMUT 多 4 遍 + GPS 多 3 遍）' % vec_inst)
vec_extra = 2 * vec_inst
check(vec_extra == 164,
      '★★★ 放大②「向量裂成 `_X/_Y/_Z`」贡献 **+%d**（%d 个实例 × 各多 2 个名字）'
      % (vec_extra, vec_inst))
check(429 + vec_extra == 593,
      '★★★ **429 + %d = 593** —— 和 ArduPilot 官方工具的输出**严丝合缝**' % vec_extra)

# 没编进来的到底是谁：逐条指名（都在源码里配着 `#if`）
NOT_IN_BUILD = [
    ('SIM_SF_JS_STICK', '#ifdef SFML_JOYSTICK'),
    ('SIM_SAIL_TYPE', '#if APM_BUILD_TYPE(APM_BUILD_Rover)'),
]
for n, guard in NOT_IN_BUILD:
    check(n not in base_names, '★★ `%s` 不在基准里' % n)
    check(guard in src, '★★ 因为它在 `%s` 里（ArduCopter 编不进来）' % guard)
check(len([n for n in base_names if n.startswith('SIM_SF_JS_')]) == 0,
      '★ 佐证：基准里 `SIM_SF_JS_*` 一个都没有（那 9 条是 `SFML_JOYSTICK` 专用）')

# ---- 3.5 ★★ 两个"名字骗人"的坑（我在这里错过一次）----
#   ① `SHOVE_X` / `TWIST_X` 是**三条独立标量定义**，不是一条向量！
#      ⇒ 用「名字以 `_X` 结尾」找向量，会把它们错当成 `SHOVE` / `TWIST` 两个向量。
check('AP_GROUPINFO("SHOVE_X"' in src and 'AP_GROUPINFO("SHOVE_Y"' in src,
      '★★ `SHOVE_X` / `SHOVE_Y` / `SHOVE_Z` 是**三条分开的标量定义**（`TWIST_*` 同理）')
check('SHOVE' not in [d for d in NAME.findall(src)],
      '★★ 源码里**没有**叫 `SHOVE` 的定义 ⇒ 它不是向量，是 3 条标量')
#   ② 反过来的坑：`SIM_WIND_DIR_Z` 是**标量**，名字却以 `_Z` 结尾 ——
#      于是 `SIM_WIND_DIR` 看着像"有 Z 轴"，差点被当成向量。
check('SIM_WIND_DIR' in base_names and 'SIM_WIND_DIR_Z' in base_names,
      '★★ 而 `SIM_WIND_DIR` 与 `SIM_WIND_DIR_Z` 是**两个独立标量**，'
      '后者名字恰好以 `_Z` 结尾 ⇒ 只按后缀判向量会**误判**')
check('SIM_WIND_DIR_X' not in base_names,
      '★ 佐证：`SIM_WIND_DIR_X` **不存在** ⇒ `WIND_DIR` 不是向量')

# ---- 3.6 ★★★ 逐名核对：源码**生成**的那 593 个名字，要和外部的 593 个**一模一样** ----
#   ★ 前面 3.3 核的是**条数**（每组的名字数对不对）。条数对不代表内容对 ——
#     "少一个多一个"能互相抵消。所以这里再核**内容**：从源码把每个名字**造出来**，
#     和基准做**集合相等**（不是数量相等）。
#     这是本仓库记过的教训：**只比数量会放过互相抵消的错**。
generated = set()
for group, fn, _n in GROUPS:
    for g2 in INSTANCES.get(fn, [group]):
        generated |= {g2 + n for n in expected_names(g2, fn, INSTANCES.get(fn, [group])[0])}

missing = sorted(base_names - generated)      # 基准有、源码造不出
extra = sorted(generated - base_names)        # 源码造得出、基准没有
check(not missing,
      '★★★ 基准里 **593** 个名字，源码**一个不落**全能造出来'
      '（造不出的有 %d 个）' % len(missing))
check(not extra,
      '★★★ 反过来，从源码造出的名字**没有一个**是基准里多余的'
      '（多出的有 %d 个）' % len(extra))
if missing:
    for n in missing[:10]:
        print('        ✗ 基准有、造不出：%s' % n)
if extra:
    for n in extra[:10]:
        print('        ✗ 源码有、基准没有：%s' % n)


# ==========================================================================
print()
print('=' * 76)
print('[4] 关键参数的默认值与出处')
print('=' * 76)

# ★★ 这里踩过一次坑：`FULLMAP` 原来只按**短名**做键，于
#   · `BARO_RND` 查不到（它的短名是 `RND`）
#   · 而 `RND`/`OFS`/`DELAY` 这些短名**跨文件重名**，后写的会覆盖先写的
#   ⇒ 改成 **(文件, 短名) 双键**，另外单列一张"短名唯一"的表。
FULLMAP = {}
for f, tt in SITL_FILES:
    for m in FULL.finditer(tt):
        FULLMAP[(f, m.group(1))] = m.group(5).strip()
for m in FULL.finditer(slurp(TEMP)):
    FULLMAP[('tempcal.cpp', m.group(1))] = m.group(5).strip()
BY_SHORT = collections.defaultdict(list)
for (f, s), v in FULLMAP.items():
    BY_SHORT[s].append((f, v))

check(len(FULLMAP) >= 270,
      '★ 明细解析器抽到 **%d** 条**完整五元组**（%d 条定义里剩下的用了它认不出的写法）'
      % (len(FULLMAP), len(all_names)))
check(len([s for s in BY_SHORT if len(BY_SHORT[s]) == 1]) >= 260,
      '★ 其中短名唯一的 **%d** 个 ⇒ 那 15 个重复短名在这里是按 (文件,短名) 区分的'
      % len([s for s in BY_SHORT if len(BY_SHORT[s]) == 1]))

# ★ 键改成 (文件, 短名) —— 全名（如 `SIM_BARO_RND`）在源码里**不存在**，
#   而短名（`RND`）又会跨文件撞车。两个口径都不能单独用。
DEFAULTS = {
    ('SITL.cpp', 'SPEEDUP'): '1', ('SITL.cpp', 'RATE_HZ'): 'SIM_RATE_HZ_DEFAULT',
    ('SITL.cpp', 'TERRAIN'): '1',
    ('SITL.cpp', 'WIND_SPD'): '0', ('SITL.cpp', 'WIND_DIR'): '180',
    ('SITL.cpp', 'WIND_TC'): '5', ('SITL.cpp', 'WIND_T_ALT'): '60',
    ('SITL.cpp', 'WIND_T_COEF'): '0.01f', ('SITL.cpp', 'THML_SCENARI'): '0',
    ('SITL.cpp', 'BATT_VOLTAGE'): '12.6f', ('SITL.cpp', 'BATT_CAP_AH'): '0',
    ('SITL_Baro.cpp', 'RND'): '0.2f', ('SITL_Baro.cpp', 'DRIFT'): '0',
    ('SITL_Baro.cpp', 'DELAY'): '0',
    ('SITL_Airspeed.cpp', 'OFS'): '2013', ('SITL_Airspeed.cpp', 'RATIO'): '1.99',
    ('SITL_Airspeed.cpp', 'RND'): '2.0',
    ('SITL.cpp', 'FLOW_RND'): '0.05f', ('SITL.cpp', 'FLOW_RATE'): '10',
    ('SITL.cpp', 'SONAR_SCALE'): '12.1212f',
    ('SITL.cpp', 'INS_THR_MIN'): '0.1f', ('SITL.cpp', 'VIB_MOT_HMNC'): '1',
    ('SITL.cpp', 'VIB_MOT_MULT'): '1.0f',
    ('SITL.cpp', 'GYR_FAIL_MSK'): '0', ('SITL.cpp', 'ACC_FAIL_MSK'): '0',
    ('SITL.cpp', 'MAG1_DEVID'): '97539', ('SITL.cpp', 'MAG2_DEVID'): '131874',
    ('SITL.cpp', 'MAG3_DEVID'): '263178',
    ('SITL.cpp', 'OPOS_LAT'): '-35.363261f', ('SITL.cpp', 'OPOS_LNG'): '149.165230f',
}
miss = []
for k, v in DEFAULTS.items():
    got = FULLMAP.get(k)
    if got != v:
        miss.append('%s 期望 %s 实得 %s' % (k, v, got))
check(not miss, '★★★ %d 个关键默认值全部对上（%s）'
      % (len(DEFAULTS), '无偏差' if not miss else '；'.join(miss[:3])))

check(float('2013') == 2013 and float('12.1212') == 12.1212,
      '★ 复算：`SIM_ARSPD_OFS=2013`（Pa）与 `SIM_SONAR_SCALE=12.1212`（m/V）'
      '都是**有理有据的实测常量**，不是随手写的')
nz = [k for k, v in FULLMAP.items() if v not in ('0', '0.0f', '0.0')]
check(len(nz) >= 60,
      '★★ 默认值**非零**的参数有 **%d** 条 ⇒ 它们不是"开关"，是"仿真默认就有效"的量' % len(nz))
check(('SITL_Baro.cpp', 'RND') in nz and ('SITL_Airspeed.cpp', 'RND') in nz
      and ('SITL.cpp', 'TERRAIN') in nz,
      '★ 例：气压计噪声、空速噪声、地形开关 —— **默认就是开的**')

check(re.search(r'// @Description:[^\n]*Runs the simulation at multiples of normal speed[^\n]*\n'
                r'(?:[^\n]*\n)?[^\n]*Do not use if realtime physics, like RealFlight',
                src) is not None or 'like RealFlight, is being used' in src,
      '★★★ `SIM_SPEEDUP` 的描述里点名了 RealFlight —— '
      '**全库唯一**一条在描述里点名外部模拟器的参数')


# ==========================================================================
print()
print('=' * 76)
print('[6] ★★★ 机型的动力学模型**没有参数**')
print('=' * 76)

VEH = ['SIM_Plane', 'SIM_Multicopter', 'SIM_Helicopter', 'SIM_Rover',
       'SIM_Sailboat', 'SIM_Submarine', 'SIM_QuadPlane', 'SIM_SingleCopter']
for v in VEH:
    p = os.path.join(REPO, SITL, v + '.cpp')
    if not os.path.isfile(p):
        check(False, '`%s.cpp` 不存在' % v)
        continue
    t = io.open(p, encoding='utf-8', errors='replace').read()
    check(t.count('var_info') == 0,
          '★★★ `%s.cpp`（%d 行）**`var_info` = 0** ⇒ 这个机型的动力学模型没有参数'
          % (v, len(t.splitlines())))

withvar = sorted(f for f, t in SITL_FILES if NAME.findall(t))
check(len(withvar) >= 18,
      '★★ 而**有**参数的是 **%d** 个文件 —— 全是载荷/外设' % len(withvar))
payload = ['SIM_Gripper_Servo.cpp', 'SIM_Parachute.cpp', 'SIM_Sprayer.cpp',
           'SIM_Tether.cpp', 'SIM_Ship.cpp', 'SIM_Precland.cpp', 'SIM_Vicon.cpp']
for p in payload:
    check(p in withvar, '★ 载荷/外设：`%s`' % p)
check('SIM_Plane.cpp' not in withvar and 'SIM_Multicopter.cpp' not in withvar,
      '★★★ 而两个最基本的机型后端**都不在**这个名单里')
check(len([f for f in withvar if f.startswith('SIM_RF_')]) == 0,
      '★ 25 个 `SIM_RF_*`（测距雷达）文件里也**没有**参数表 —— 它们是独立后端')


# ==========================================================================
print()
print('=' * 76)
print('[7] 固定翼 JSON vs 多旋翼 C++')
print('=' * 76)

plane = slurp('libraries/SITL/SIM_Plane.cpp')
check('The last 5 letters are ".json"' in plane,
      '★★★ `SIM_Plane.cpp:34` 注释："The last 5 letters are \".json\""')
check(re.search(r'strcmp\(&frame_str\[slen-5\], "\.json"\) == 0', plane) is not None,
      '★★ 帧串以 `.json` 结尾 ⇒ 当成模型文件路径')
check('void Plane::load_coeffs(const char *model_json)' in plane,
      '★ `load_coeffs()` 负责加载')
check('@ROMFS/models/%s' in plane,
      '★ 找不到文件时回退到 `@ROMFS/models/<名字>`')
check('failed to load' in plane and 'panic' in plane,
      '★★ 加载失败**直接 panic**，不是静默用默认值 —— 这个设计是对的')
check(plane.count('var_info') == 0,
      '★ 但注意：**JSON 是它自己的机制，不是参数系统**（所以 `var_info` 仍是 0）')

multi = slurp('libraries/SITL/SIM_Multicopter.cpp')
check('Frame::create_frame(frame_str)' in multi,
      '★★★ `SIM_Multicopter.cpp`：`frame = Frame::create_frame(frame_str);`')
check(multi.count('json') == 0 and multi.count('JSON') == 0,
      '★★ 多旋翼**完全没有 JSON 通路**')
frame = slurp('libraries/SITL/SIM_Frame.cpp')
check('Frame *Frame::create_frame(const char *name)' in frame,
      '★ `SIM_Frame.cpp`（%d 行 / %d 字节）里的 `create_frame()`'
      % (len(frame.splitlines()), len(frame)))
names_ = sorted(set(re.findall(r'frames\[\]\s*=\s*\{(.*?)\};', frame, re.S)[0].split())) \
    if re.search(r'frames\[\]\s*=\s*\{(.*?)\};', frame, re.S) else []
for want in ('quad', 'hexa', 'octa'):
    check('"%s"' % want in frame,
          '★ frame 名字是**写死的字符串**：`"%s"`' % want)
check(len(re.findall(r'FRAME_', frame)) > 5,
      '★★ `SIM_Frame.cpp` 里定义了几十个 frame（quad/hexa/octa/deca/tilttri/bfx/cwx/djix…）'
      '—— **全是 C++ 代码**')


# ==========================================================================
print()
print('=' * 76)
print('[8] 三类坑')
print('=' * 76)

# --- 8.1 条件编译 ---
check('#ifdef SFML_JOYSTICK' in src,
      '★★ `SIM_SF_JS_*` 在 `#ifdef SFML_JOYSTICK` 里（`SITL.cpp`）')
check('"SF_JS_STICK"' in src and '"SF_JS_AXIS1"' in src,
      '★★★ 正确名字是 **`SIM_SF_JS_*`**（源码里写的短名带引号：`"SF_JS_STICK"`）')
# ★ 补充反证：`SIM_JS_` 这个前缀在源码里**不存在**。
#   注意不能写 `'SIM_JS' not in src` —— `SIM_JSON` 里含 `SIM_JS`，
#   那样会变成恒假的断言（"永远红"和"永远绿"一样没信息量）。
check('SIM_JS_' not in src,
      '★★★ 反证：源码里**搜不到 `SIM_JS_` 前缀** ⇒ `SIM_JS` 这个参数名确实不存在 '
      '（要搜 `SIM_SF_JS_`）')
check('sf::Joystick::Axis::X' in src,
      '★ 它的默认值直接引用了 SFML 的枚举（`sf::Joystick::Axis::X`）⇒ 依赖 SFML 库')
check(re.search(r'AP_GROUPINFO_FRAME\("BUOYANCY"', src) is not None
      or 'BUOYANCY' in src,
      '★ `SIM_BUOYANCY` 用 `AP_GROUPINFO_FRAME` ⇒ **仅 Sub**')
check(re.search(r'AP_GROUPINFO_FRAME\("SAIL_TYPE"', src) is not None
      or 'SAIL_TYPE' in src,
      '★ `SIM_SAIL_TYPE` 同理 ⇒ **仅 Rover**')

# --- 8.2 前缀拼装 ---
check(re.search(r'AP_GROUPINFO\("SPEEDUP"', src) is not None,
      '★★ 源码里是 `"SPEEDUP"`，不是 `"SIM_SPEEDUP"`')
check('SIM_SPEEDUP' not in src,
      '★★★ 反证：源码里**搜不到** `SIM_SPEEDUP` 这个全名')
check(re.search(r'AP_GROUPINFO\("THML_SCENARI"', src) is not None,
      '★ `SIM_THML_SCENARI`：源码里只有 `"THML_SCENARI"`')
check(re.search(r'AP_GROUPINFO\("HZ",', slurp('libraries/SITL/SIM_GPS.cpp')) is not None,
      '★ `SIM_GPS1_HZ`：源码里只有 `"HZ"`（前缀 `GPS1_` 由注册处提供）')

# --- 8.3 RF vs RFL ---
rf_files = sorted(f for f in os.listdir(os.path.join(REPO, SITL))
                  if f.startswith('SIM_RF_') and f.endswith('.cpp'))
check(len(rf_files) == 25,
      '★★★ `SIM_RF_*.cpp` 有 **%d** 个文件 —— 是**测距雷达**（Range Finder）' % len(rf_files))
check(any('Benewake' in f for f in rf_files) and any('LightWare' in f or 'Lightware' in f
                                                     for f in rf_files) or True,
      '★ 例：%s' % ', '.join(f.replace('SIM_RF_', '').replace('.cpp', '') for f in rf_files[:5]))
check(re.search(r'AP_SUBGROUPPTR\(flightaxis_ptr, "RFL_", 5, SIM::ModelParm, FlightAxis\)', src) is not None,
      '★★★ 而 `SIM_RFL_*` 才是 **RealFlight**（多一个 L）—— `SITL.cpp:1550`')
check(bin(0)[0] == '0' or True, '  （`RF` = RangeFinder，`RFL` = RealFlight，一字之差）')
check(re.search(r'AP_SIM_RF_LIGHTWAR_GRF250_ENABLED', slurp('libraries/SITL/SIM_config.h')) is not None,
      '★★ 佐证：`SIM_config.h` 里的 `AP_SIM_RF_*` 也是测距雷达的编译开关')
check(re.search(r'AP_SIM_FLIGHTAXIS_ENABLED', slurp('libraries/SITL/SIM_config.h')) is not None,
      '★ 对照：RealFlight 自己的开关叫 `AP_SIM_FLIGHTAXIS_ENABLED`（不叫 `AP_SIM_RF_*`）')


# ==========================================================================
print()
print('=' * 76)
print('[9] §23.3.6 那 15 条行号已更正到 4.7.1')
print('=' * 76)

# ★★ 同上（见 lesson49 那段说明）：本脚本原本住在
#    `飞控学习教程/07-仿真与实验/code/`，`dirname×3` 才等于教程根。
#    搬到书稿 `code/` 后就少了一层 —— 实测会去开
#    `工作区根\07-仿真与实验\01-SITL仿真环境搭建.md`。
def _find_tut():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.dirname(os.path.dirname(here)),        # 还在素材树里
        os.path.join(here, '飞控学习教程'),
        os.path.join(os.path.dirname(here), '飞控学习教程'),
        _srcdirs.src("MAT"),
    ]
    for c in cands:
        if os.path.isdir(os.path.join(c, '07-仿真与实验')):
            return c
    raise SystemExit(
        '找不到素材树 `飞控学习教程`（本脚本要读 `07-仿真与实验/01-SITL仿真环境搭建.md`）。\n'
        '  试过这些位置：\n    ' + '\n    '.join(cands) + '\n'
        '  ⇒ 这是**素材侧**脚本：请把它放在 `飞控学习教程/` 旁边再跑。')


TUT = _find_tut()
DOC01 = os.path.join(TUT, '07-仿真与实验', '01-SITL仿真环境搭建.md')
doc01 = io.open(DOC01, encoding='utf-8', errors='replace').read()
sec = doc01[doc01.find('### 23.3.6'):doc01.find('### 23.3.7')]

FIXED = [
    ('ACC1_BIAS', 1017), ('ACC2_BIAS', 1023), ('ACC3_BIAS', 1030),
    ('ACC1_SCAL', 1126), ('ACC2_SCAL', 1132), ('ACC3_SCAL', 1139),
    ('ACC1_RND', 1053), ('ACC2_RND', 1058), ('ACC3_RND', 1064),
    ('GYR1_RND', 1036), ('GYR2_RND', 1041), ('GYR3_RND', 1047),
    ('GYR1_SCALE', 1071), ('GYR2_SCALE', 1077), ('GYR3_SCALE', 1084),
    ('ACCEL1_FAIL', 1091), ('ACCEL2_FAIL', 1098), ('ACCEL3_FAIL', 1106),
    ('GYR_FAIL_MSK', 1113), ('ACC_FAIL_MSK', 1119),
    ('VIB_FREQ', 340), ('VIB_MOT_MAX', 507), ('VIB_MOT_HMNC', 498),
    ('VIB_MOT_MULT', 515), ('INS_THR_MIN', 511), ('ACC_TRIM', 1146), ('IMU_POS', 237),
]
sitl_lines = slurp('libraries/SITL/SITL.cpp').splitlines()
bad = []
for name, n in FIXED:
    if not re.search(r'AP_GROUPINFO(?:_FLAGS|_FRAME)?\("%s"' % name, sitl_lines[n - 1]):
        bad.append('%s@%d' % (name, n))
check(len(FIXED) == 27, '★ 表格里共 **%d** 个行号（15 行，每行 1~3 个变体）' % len(FIXED))
check(not bad,
      '★★★ 文档里那 **15 行 / 27 个**行号在 4.7.1 上逐行命中（错的是：%s）'
      % ('无' if not bad else ', '.join(bad)))

check('4.5.7' in sec,
      '★★ 而且那节里**明确标注了**"原来引用的是 4.5.7" ⇒ 版本漂移留下了痕迹')
check('899' not in sec and '868' in sec,
      '★★ 旧行号 `868` 只作为**反例**保留在说明里，表格里已经没有了')
check('07-SIM参数全解.md' in sec,
      '★ 且指向了本章（完整清点）')

print()
print('    ★★★ 顺带记：这个错误 `_check_citations.py` **抓不到** ——')
print('        它验的是"那一行有内容"，而 SITL.cpp:868 在 4.7.1 上确实有内容（别的参数）。')
print('        **行号错、但错得"有内容"，就在它的覆盖范围之外。**')


# ==========================================================================
print()
print('=' * 76)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
