#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""卷 10 第 9 章的三处源码论断 —— 拿 PX4 源码自动核一遍，而不是靠人眼。

为什么要有它：这一章的骨架改编自一份**教学级**的第三方文档，作者在写作时发现
原文有三处论断与 PX4 1.15.4 的源码不符（见该章 §9.10）。那三处是**逐行人眼核**出来的 ——
而"人眼核过"这件事**无法复算**：源码一升级、行号一漂，读者无从判断书上写的还成不成立。

所以把这三处写成断言：**跑一遍就知道书里的说法在你这棵树上还成不成立。**

    python ch10_offboard_claims.py

需要一棵 PX4-Autopilot **1.15.4** 源码树（默认取 `_srcdirs` 的 `PX4_115`，
可用环境变量 `SRC_PX4_115` 覆盖）。**找不到树就报错退出** —— 不静默跳过
（本项目有过"工具以为跑了、其实整节没执行"的先例）。

断言分三组，与 §9.4.3、§9.4.2 一一对应。每条都打印 `文件:行号`，可回查。
"""
import io
import os
import re
import sys

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

FAILED = []


def check(cond, label, where=u''):
    ok = bool(cond)
    print(u'  %s %s%s' % (u'✅' if ok else u'❌', label, (u'   [%s]' % where) if where else u''))
    if not ok:
        FAILED.append(label)
    return ok


def find_first(path, pattern):
    """返回 (行号, 行内容)；找不到返回 (0, '')。"""
    if not os.path.isfile(path):
        return 0, u''
    rx = re.compile(pattern)
    for i, line in enumerate(io.open(path, encoding='utf-8', errors='replace'), 1):
        if rx.search(line):
            return i, line.rstrip()
    return 0, u''


def selftest():
    """量具自证：check() 该红的会红吗？find_first() 找得到/找不到分得清吗？"""
    bad = 0
    save = list(FAILED)
    check(True, u'自证·应通过')
    check(False, u'自证·应不通过')
    if len(FAILED) == len(save) + 1:
        print(u'  ✅ check() 的记账正确')
    else:
        print(u'  ❌ check() 记账不对（FAILED %d → %d）' % (len(save), len(FAILED)))
        bad += 1
    FAILED[:] = save
    import tempfile
    d = tempfile.mkdtemp()
    f = os.path.join(d, 'sample.txt')
    io.open(f, 'w', encoding='utf-8').write(u'第一行\nCOM_OF_LOSS_T = 1.0f\n')
    ln, s = find_first(f, r'COM_OF_LOSS_T')
    if ln == 2 and u'COM_OF_LOSS_T' in s:
        print(u'  ✅ find_first() 找得到（行号 %d）' % ln)
    else:
        print(u'  ❌ find_first() 没找到应有的行（%d）' % ln)
        bad += 1
    ln2, _ = find_first(f, r'这个模式不存在')
    if ln2 == 0:
        print(u'  ✅ find_first() 找不到时返回 0（不假装找到）')
    else:
        print(u'  ❌ find_first() 找不到却返回 %d' % ln2)
        bad += 1
    print(u'自证：%s' % (u'✅ 全过' if not bad else u'❌ %d 条不对' % bad))
    return bad


def main():
    if '--selftest' in sys.argv:
        return selftest()
    try:
        import _srcdirs
        root = _srcdirs.src('PX4_115')
    except Exception as e:
        print(u'❌ 取不到 PX4_115 源码根：%r' % (e,))
        print(u'   设环境变量 SRC_PX4_115 指向 PX4-Autopilot 1.15.4 的根目录再跑。')
        return 2

    print(u'PX4 源码根：%s' % root)
    if not os.path.isdir(root):
        print(u'❌ 目录不存在。**不跳过** —— 断言需要有树才能核。')
        return 2

    CMDR = os.path.join(root, 'src/modules/commander')
    OCHK = os.path.join(CMDR, 'HealthAndArmingChecks/checks/offboardCheck.cpp')
    RCV = os.path.join(root, 'src/modules/mavlink/mavlink_receiver.cpp')

    # ── 组 ①：超时阈值（§9.4.3 第 1 条）──────────────────────────────
    print(u'\n① 超时阈值 = COM_OF_LOSS_T（默认 1.0 s），**不是** 0.5 s')
    ln, s = find_first(os.path.join(CMDR, 'commander_params.c'),
                       r'PARAM_DEFINE_FLOAT\(COM_OF_LOSS_T\s*,\s*([0-9.]+)f?\)')
    m = re.search(r',\s*([0-9.]+)f?\)', s) if s else None
    check(bool(m) and float(m.group(1)) == 1.0,
          u'COM_OF_LOSS_T 默认值 == 1.0', u'commander_params.c:%d' % ln)
    if m:
        print(u'      实际读到：%s' % m.group(1))

    # ── 组 ②：判据的形状（§9.4.3 第 2~3 条）─────────────────────────
    print(u'\n② 判据 =「消息时间戳 + COM_OF_LOSS_T」&& 七通道任一，**与心跳无关**')
    body = io.open(OCHK, encoding='utf-8', errors='replace').read() if os.path.isfile(OCHK) else u''
    ln, _ = find_first(OCHK, r'_param_com_of_loss_t\.get\(\)')
    check(ln > 0, u'用到了 COM_OF_LOSS_T', u'offboardCheck.cpp:%d' % ln)
    ln, s = find_first(OCHK, r'hrt_absolute_time\(\)\s*<\s*offboard_control_mode\.timestamp')
    check(ln > 0, u'比的是 offboard_control_mode.timestamp（消息时间戳）',
          u'offboardCheck.cpp:%d' % ln)
    chans = [u'position', u'velocity', u'acceleration', u'attitude',
             u'body_rate', u'thrust_and_torque', u'direct_actuator']
    missing = [c for c in chans if (u'offboard_control_mode.%s' % c) not in body]
    check(not missing, u'七个控制通道都在判据里（缺：%s）' % (missing or u'无'),
          u'offboardCheck.cpp')
    # ★ 本组的关键：超时判据里不该有心跳
    check(u'heartbeat' not in body.lower(),
          u'★ 超时判据里**没有** heartbeat（教学文档说"必须先发心跳"）',
          u'offboardCheck.cpp')

    # ── 组 ③：心跳在别处按类型记（§9.4.3 的注）───────────────────────
    print(u'\n③ 心跳的作用在别处：按**类型**分别记时间')
    kinds = [u'_heartbeat_type_gcs', u'_heartbeat_type_onboard_controller',
             u'_heartbeat_type_gimbal']
    ln0, _ = find_first(RCV, r'_heartbeat_type_gcs')
    found = [k for k in kinds if k in io.open(RCV, encoding='utf-8', errors='replace').read()]
    check(len(found) >= 2, u'接收端按类型记心跳（找到 %d 类：%s）'
          % (len(found), u'、'.join(k.replace(u'_heartbeat_type_', u'') for k in found)),
          u'mavlink_receiver.cpp:%d' % ln0)

    # ── 组 ④：设定点的"频率"从哪来（§9.4.2 末）──────────────────────
    print(u'\n④ offboard_control_mode 由接收端在收到设定点时发布')
    src = io.open(RCV, encoding='utf-8', errors='replace').read() if os.path.isfile(RCV) else u''
    pubs = [i for i, l in enumerate(src.split('\n'), 1)
            if '_offboard_control_mode_pub.publish' in l]
    check(len(pubs) >= 3, u'至少 3 处发布（实测 %d 处）' % len(pubs),
          u'mavlink_receiver.cpp:%s' % u','.join(str(x) for x in pubs))
    ln, _ = find_first(RCV, r'POSITION_TARGET_TYPEMASK_[XYZ]_IGNORE')
    check(ln > 0, u'type_mask 逐位判（忽略的轴填 NAN 的那套）',
          u'mavlink_receiver.cpp:%d' % ln)

    print(u'\n' + u'=' * 60)
    if FAILED:
        print(u'❌ %d 条断言不过：' % len(FAILED))
        for f in FAILED:
            print(u'   · %s' % f)
        print(u'\n⇒ 若你用的是**别的 PX4 版本**，这几条本来就可能不成立 ——')
        print(u'   书上那三处是按 1.15.4 核的，换版本请先跑本脚本再判断。')
        return 1
    print(u'✅ 全部通过：第 9 章 §9.4 的三处论断在你这棵树上成立')
    return 0


if __name__ == '__main__':
    sys.exit(main())
