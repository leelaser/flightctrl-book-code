# -*- coding: utf-8 -*-
"""反向注入：确认 lesson50（HIL 已删除）的断言【真的会红】。

★ 这一课验的是**否定命题**（"某功能不存在了"）。
  否定命题的注入方向和肯定命题相反：
  不能"把功能改坏"，只能**把判据改反**（假装它还在）。
"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

TUT = r'D:\DEVLib\PX4APM\飞控学习教程'
SRC = os.path.join(HERE, 'lesson50_hil.py')
TMP = os.path.join(tempfile.gettempdir(), '_l50_mut.py')

INJECTIONS = [
    ('S1  假装 HIL 消息还有收发端',
     "check(hits_hil_msg == [],", "check(hits_hil_msg != [],",
     ['既不收也不发 HIL 消息'], 'hits_hil_msg != [],'),

    ('S2  量具自证的门槛抬到不可能',
     "check(files_scanned > 3000,", "check(files_scanned > 300000,",
     ['量具自证：扫了'], '> 300000,'),

    # ★ 这条串在脚本里出现两次（一次在主体、一次在"推理链"），
    #   所以要连消息一起写才能唯一定位。
    ('S3  假装 `_detect_instance` 里有 HIL 分支',
     "check('GPS_TYPE_HIL' not in handled,\n      '★★★ 而**里面没有",
     "check('GPS_TYPE_HIL' in handled,\n      '★★★ 而**里面没有",
     ['里面没有 `case GPS_TYPE_HIL`'], "check('GPS_TYPE_HIL' in handled,"),

    ('S4  把墓碑数说成 2 个',
     "'k_param_hil_mode_unused'],", "],",  # 期望只留两个墓碑名
     ['参数墓碑'], "'k_param_hil_err_limit_unused',"),

    ('S5  假装 `AP_Baro_HIL.cpp` 与 HIL 有关',
     "check(not re.search(r'\\bHIL\\b', bh),",
     "check(re.search(r'\\bHIL\\b', bh) is not None,",
     ['文件名骗人'], "re.search(r'\\bHIL\\b', bh) is not None,"),

    ('S6  把删除记录说成 4.6.0',
     "check(above and above[-1][1].startswith('4.7.0'),",
     "check(above and above[-1][1].startswith('4.6.0'),",
     ['它属于 **4.7.0**'], "startswith('4.6.0'),"),

    ('S7  假装 `SIM_OH_MASK` 不存在',
     "check(re.search(r'// @Param: OH_MASK', sitl) is not None,",
     "check(re.search(r'// @Param: OH_MASK', sitl) is None,",
     ['SIM-on-hardware'], "OH_MASK', sitl) is None,"),

    ('S8  假装传感器回放只有写没有读',
     "check(m is not None and 'Read data from file' in m.group(1) and 'Write data to a file' in m.group(1),",
     "check(m is not None and 'Read data from file' in m.group(1) and 'Write data to a file' not in m.group(1),",
     ['它们**双向**'], "'Write data to a file' not in m.group(1),"),

    ('S9  假装 PX4 没有 SYS_HITL',
     "check(re.search(r'PARAM_DEFINE_INT32\\(SYS_HITL,\\s*0\\)', sp) is not None,",
     "check(re.search(r'PARAM_DEFINE_INT32\\(SYS_HITL,\\s*0\\)', sp) is None,",
     ['PX4 有 `SYS_HITL` 参数'], "SYS_HITL,\\s*0\\)', sp) is None,"),

    ('S10 假装 PX4 不收 HIL_SENSOR',
     "    for msg in ('HIL_SENSOR', 'HIL_STATE_QUATERNION', 'HIL_OPTICAL_FLOW', 'HIL_GPS'):",
     "    for msg in ('HIL_SENSORX', 'HIL_STATE_QUATERNION', 'HIL_OPTICAL_FLOW', 'HIL_GPS'):",
     ['PX4 收 `HIL_SENSORX`'], "'HIL_SENSORX',"),

    ('S11 把 HIL_SENSOR 的 ID 说成 108',
     "check(re.search(r'<message id=\"107\" name=\"HIL_SENSOR\"', xml) is not None,",
     "check(re.search(r'<message id=\"108\" name=\"HIL_SENSOR\"', xml) is not None,",
     ['`HIL_SENSOR` = '], 'id="108" name="HIL_SENSOR"'),

    ('S12 把 time_usec 说成 uint32',
     "    check(ts and 'uint64_t' in ts[0][0],",
     "    check(ts and 'uint32_t' in ts[0][0],",
     ['是 uint64'], "'uint32_t' in ts[0][0],"),

    ('S13 假装 HIL_SENSOR 里没有 fields_updated',
     "    check('fields_updated' in names,",
     "    check('fields_updated' not in names,",
     ['是个**位掩码**'], "'fields_updated' not in names,"),

    ('S14 假装 HIL_STATE 没有被标废弃',
     "          and 'deprecated' in xml[xml.find('name=\"HIL_STATE\"'):xml.find('name=\"HIL_STATE\"') + 400],",
     "          and 'deprecated' not in xml[xml.find('name=\"HIL_STATE\"'):xml.find('name=\"HIL_STATE\"') + 400],",
     ['已被标为'], "'deprecated' not in xml["),

    ('S15 假装参数文档里没有 7:HIL',
     "check(re.search(r'//\\s*@Values:[^\\n]*7:HIL', slurp('libraries/AP_GPS/AP_GPS_Params.cpp')) is not None,",
     "check(re.search(r'//\\s*@Values:[^\\n]*7:HIL', slurp('libraries/AP_GPS/AP_GPS_Params.cpp')) is None,",
     ['参数文档里还列着'], "@Values:[^\\n]*7:HIL', slurp('libraries/AP_GPS/AP_GPS_Params.cpp')) is None,"),

    ('S16 假装墓碑名不带 _unused',
     "check(all(t.endswith('_unused') for t in tombs),",
     "check(all(not t.endswith('_unused') for t in tombs),",
     ['名字都带 `_unused` 后缀'], "not t.endswith('_unused')"),

    ('S17 把 SITL 反向对照的门槛抬到不可能',
     "check(sitl_hits > 5,", "check(sitl_hits > 5000,",
     ['反向对照：同一套搜索'], '> 5000,'),

    ('S18 假装 `HAL_INS_HIL_UNUSED` 是板级宏',
     "check('HAL_INS_NONE' in boards and 'HAL_BOARD_SITL' in boards,",
     "check('HAL_INS_NONE' in boards and 'HAL_BOARD_HILX' in boards,",
     ['是同一族宏，不是编译开关'], "'HAL_BOARD_HILX' in boards,"),
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
        print('[SKIP] %-46s 原文出现 %d 次' % (name, base.count(old)))
        fails += 1
        continue
    mutated = base.replace(old, new, 1)
    if must_appear not in mutated:
        print('[SKIP] %-46s 注入后 grep 不到' % name)
        fails += 1
        continue
    io.open(TMP, 'w', encoding='utf-8').write(mutated)
    rc, out = run(TMP)

    red = [l.strip() for l in out.splitlines() if '[!!]' in l]
    hit = [e for e in expect_red if any(e in l for l in red)]
    crashed = rc != 0 and not red
    ok = (rc != 0) and (len(hit) == len(expect_red) or crashed)
    print('%-7s %-46s EXIT=%-3s 变红 %d/%d %s'
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
