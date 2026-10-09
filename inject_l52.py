# -*- coding: utf-8 -*-
"""反向注入：确认 lesson52（SIM_* 参数全解）的断言【真的会红】。

★ 本章的核心是**三个算出来的数**（319 / 289 / 440），
  所以注入的重点是：**把这三个数改错，看脚本抓不抓得到**。
"""
import io
import _srcdirs
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

TUT = _srcdirs.src("MAT")
SRC = os.path.join(HERE, 'lesson52_simparams.py')
# ★★ 这一个**必须**把变异副本写在 `code/` 里，不能放临时目录：
#    `lesson52_simparams.py` 的独立基准 `sim_pdef_arducopter471.txt`
#    是**按"自己旁边"找的**（`dirname(__file__)`）。副本一旦落到临时目录，
#    它找不到基准 ⇒ **变红的是「找不到基准」那条断言，不是我们注入的那一条**，
#    于是 39 条注入里 27 条报「变红 0/1」。
#    （`code/` 里那个数据文件在，且本脚本结尾会 `os.remove(TMP)`。）
TMP = os.path.join(HERE, '_l52_mut.py')

INJECTIONS = [
    # ---- 三个数 ----
    ('S1  定义数说成 318', "check(n_sitl + n_temp == 319,", "check(n_sitl + n_temp == 318,",
     ['合计 319 条定义'], '== 318,'),
    ('S2  唯一短名说成 288', "check(len(set(all_names)) == 289,", "check(len(set(all_names)) == 288,",
     ['唯一短名 289 个'], '== 288,'),
    ('S3  重复短名说成 14 个', "check(len(dup) == 15,", "check(len(dup) == 14,",
     ['重复的短名有'], 'len(dup) == 14,'),
    ('S4  多余条数说成 29', "check(len(all_names) - len(set(all_names)) == 30,",
     "check(len(all_names) - len(set(all_names)) == 29,",
     ['多余 **'], 'set(all_names)) == 29,'),
    ('S5  ENABLE 次数说成 13', "check(collections.Counter(all_names)['ENABLE'] == 14,",
     "check(collections.Counter(all_names)['ENABLE'] == 13,",
     ['`ENABLE` 出现'], "['ENABLE'] == 13,"),
    ('S6  展开数说成 441', "check(n_sitl + n_temp - one_copy + expanded == 440,",
     "check(n_sitl + n_temp - one_copy + expanded == 441,",
     ['319 −'], '== 441,'),
    ('S7  一份定义条数说成 45', "check(one_copy == 46,", "check(one_copy == 45,",
     ['共 **'], 'one_copy == 45,'),
    ('S8  复用组数说成 3', "check(len(shared) == 4,", "check(len(shared) == 3,",
     ['个类型被注册了多次'], 'len(shared) == 3,'),
    ('S9  注册数说成 26', "check(len(regs) == 27,", "check(len(regs) == 26,",
     ['个 `AP_SUBGROUPINFO'], 'len(regs) == 26,'),
    ('S10 SITL.cpp 条数说成 180', "check(by_file['SITL.cpp'] == 181,",
     "check(by_file['SITL.cpp'] == 180,", ['一个文件就有'], "== 180,"),
    ('S11 ≥10 的文件说成 5 个', "check(len([f for f in by_file if by_file[f] >= 10]) == 6,",
     "check(len([f for f in by_file if by_file[f] >= 10]) == 5,",
     ['条数 ≥10'], '>= 10]) == 5,'),
    ('S12 分布文件数说成 23', "check(len(by_file) == 24,", "check(len(by_file) == 23,",
     ['分布在 **'], 'len(by_file) == 23,'),

    # ---- 默认值 ----
    ('S13 WIND_DIR 默认说成 181', "('SITL.cpp', 'WIND_DIR'): '180',",
     "('SITL.cpp', 'WIND_DIR'): '181',", ['关键默认值'], "'WIND_DIR'): '181',"),
    ('S14 ARSPD 零点说成 2014', "('SITL_Airspeed.cpp', 'OFS'): '2013',",
     "('SITL_Airspeed.cpp', 'OFS'): '2014',", ['关键默认值'], "'OFS'): '2014',"),
    ('S15 气压计噪声说成 0.3', "('SITL_Baro.cpp', 'RND'): '0.2f',",
     "('SITL_Baro.cpp', 'RND'): '0.3f',", ['关键默认值'], "'RND'): '0.3f',"),
    ('S16 非零默认值门槛抬到不可能', "check(len(nz) >= 60,",
     "check(len(nz) >= 600,", ['默认值**非零**'], '>= 600,'),

    # ---- §6 机型模型 ----
    ('S17 假装多旋翼有参数', "    check(t.count('var_info') == 0,",
     "    check(t.count('var_info') == 1,", ['动力学模型没有参数'], "var_info') == 1,"),
    ('S18 假装多旋翼有 JSON', "check(multi.count('json') == 0 and multi.count('JSON') == 0,",
     "check(multi.count('json') == 0 and multi.count('JSON') > 0,",
     ['完全没有 JSON 通路'], "multi.count('JSON') > 0,"),
    ('S19 假装固定翼不读 JSON', "check('The last 5 letters are \".json\"' in plane,",
     "check('The last 5 letters are \".txt\"' in plane,",
     ['注释：'], '".txt"'),

    # ---- 三类坑 ----
    ('S20 假装源码里有 SIM_JS_ 前缀', "check('SIM_JS_' not in src,",
     "check('SIM_JS_' in src,", ['搜不到 `SIM_JS_` 前缀'], "'SIM_JS_' in src,"),
    ('S21 假装 RF 有 24 个文件', "check(len(rf_files) == 25,",
     "check(len(rf_files) == 24,", ['有 **'], 'len(rf_files) == 24,'),
    ('S22 把 RF 说成 RealFlight', "check(re.search(r'AP_SUBGROUPPTR\\(flightaxis_ptr, \"RFL_\", 5, SIM::ModelParm, FlightAxis\\)', src) is not None,",
     "check(re.search(r'AP_SUBGROUPPTR\\(flightaxis_ptr, \"RF_\", 5, SIM::ModelParm, FlightAxis\\)', src) is not None,",
     ['才是 **RealFlight**'], '"RF_", 5,'),
    ('S23 假装源码里有 SIM_SPEEDUP 全名', "check('SIM_SPEEDUP' not in src,",
     "check('SIM_SPEEDUP' in src,", ['搜不到'], "'SIM_SPEEDUP' in src,"),

    # ---- 行号表 ----
    ('S24 行号表某一条改错', "    ('VIB_FREQ', 340), ('VIB_MOT_MAX', 507), ('VIB_MOT_HMNC', 498),",
     "    ('VIB_FREQ', 339), ('VIB_MOT_MAX', 507), ('VIB_MOT_HMNC', 498),",
     ['在 4.7.1 上逐行命中'], "('VIB_FREQ', 339)"),

    # ---- 319 → 593 这条桥（本章的核心，2026-10 重写后新增）----
    ('S25 运行时名字数说成 592', "check(len(base_names) == 593,",
     "check(len(base_names) == 592,", ['运行时参数名'], "== 592,"),
    ('S26 编入条数说成 307', "check(tot_live == 308,",
     "check(tot_live == 307,", ['编进来的'], "tot_live == 307,"),
    ('S27 没编入的条数说成 10', "check(tot_cut == 11,",
     "check(tot_cut == 10,", ['没编进'], "tot_cut == 10,"),
    ('S28 向量定义数说成 48', "check(tot_vec == 49,",
     "check(tot_vec == 48,", ['向量定义'], "tot_vec == 48,"),
    ('S29 429 说成 430', "check(tot_live - one_copy + expanded == 429,",
     "check(tot_live - one_copy + expanded == 430,",
     ['把每个向量当一个参数'], "expanded == 430,"),
    ('S30 多实例贡献说成 120', "check(tot_multi == 121,",
     "check(tot_multi == 120,", ['一份定义挂多个实例'], "tot_multi == 120,"),
    ('S31 向量实例数说成 81', "check(vec_inst == 82,",
     "check(vec_inst == 81,", ['向量实例共'], "vec_inst == 81,"),
    ('S32 向量多出说成 162', "check(vec_extra == 164,",
     "check(vec_extra == 162,", ['向量裂成'], "vec_extra == 162,"),
    ('S33 最终恒等式说成 594', "check(429 + vec_extra == 593,",
     "check(429 + vec_extra == 594,", ['官方工具的输出'], "vec_extra == 594,"),
    # ★ 这两条要**改数据**而不是改判据：把判据改成恒真只是让它不会红，
    #   证明不了"它平时真的在判"。要让"造名字"的环节漏掉/多出名字，
    #   再看不看得到 —— 这才是集合相等那条断言的真实负载。
    ('S34 造名字时漏掉 Z 轴', "            out |= {'%s_%s' % (d, a) for a in 'XYZ'}",
     "            out |= {'%s_%s' % (d, a) for a in 'XY'}",
     ['一个不落'], "for a in 'XY'}"),
    ('S35 造名字时多造一个', "    out.add(d)\n    return out",
     "    out.add(d)\n    out.add('BOGUS_X')\n    return out",
     ['多余'], "BOGUS_X"),
    # ★ S36 的教训：我原以为"基准的名字数 vs 源码算出的名字数"是独立比对，
    #   实测**不是** —— `axis` 本身就来自 `names_of`，两边同源，砍掉一半也照样绿。
    #   真正扛事的是 §3.6 的集合相等。所以这一条改成去扎那一处。
    ('S36 IMUT 组名数砍到 20', "        return [n[len('SIM_') + len('IMUT1_'):] for n in baseline['SIM_']\n                if n.startswith('SIM_' + group[4:])]",
     "        return [n[len('SIM_') + len('IMUT1_'):] for n in baseline['SIM_']\n                if n.startswith('SIM_' + group[4:])][:20]",
     ['逐实例推出的名字合计'], "][:20]"),

    # ---- 两个"名字骗人"的坑 ----
    ('S37 假装源码里有 SHOVE 向量定义', "check('SHOVE' not in [d for d in NAME.findall(src)],",
     "check('SHOVE' in [d for d in NAME.findall(src)],", ['不是向量'], "'SHOVE' in [d for d in"),
    ('S38 假装 WIND_DIR 有 X 轴', "check('SIM_WIND_DIR_X' not in base_names,",
     "check('SIM_WIND_DIR_X' in base_names,", ['不是向量'], "'SIM_WIND_DIR_X' in base_names,"),
    ('S39 假装 SAIL_TYPE 编得进来', "    ('SIM_SAIL_TYPE', '#if APM_BUILD_TYPE(APM_BUILD_Rover)'),",
     "    ('SIM_SAIL_TYPE', '#if APM_BUILD_TYPE(APM_BUILD_Heli)'),",
     ['编不进来'], "APM_BUILD_Heli"),
]


def run(path):
    p = subprocess.run([sys.executable, '-u', path],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
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
        print('[SKIP] %-44s 原文出现 %d 次' % (name, base.count(old)))
        fails += 1
        continue
    mutated = base.replace(old, new, 1)
    if must_appear not in mutated:
        print('[SKIP] %-44s 注入后 grep 不到' % name)
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(mutated)
    rc, out = run(TMP)

    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-44s EXIT=%-3s 变红 %d/%d %s'
          % ('[PASS]' if ok else '[FAIL]', name, rc, len(hit), len(expect_red),
             '(异常退出)' if crashed else ' | '.join(h[:24] for h in hit)))
    if not ok:
        fails += 1
        for l in red[:4]:
            print('         red> %s' % l[:106])
        if crashed:
            print('         err> %s' % out.strip().splitlines()[-1][:106])

if os.path.isfile(TMP):
    os.remove(TMP)
print()
print('=' * 70)
print('注入测试：%d/%d 通过' % (len(INJECTIONS) - fails, len(INJECTIONS)))
sys.exit(1 if fails else 0)
