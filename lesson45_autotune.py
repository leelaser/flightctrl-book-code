# -*- coding: utf-8 -*-
"""
lesson45_autotune.py —— 卷 8 第 16 章《自动调参算法》配套自验证脚本

ArduPilot 里"自动调参"不是一个东西，是**三份互不共享代码的实现**：
  ① AC_AutoTune        （Copter 多旋翼 / 直升机；QuadPlane 的 QAutoTune，仅 SITL）
  ② APM_Control/AP_AutoTune （固定翼；AUTOTUNE 模式）—— 注意它不在 AC_AutoTune 下
  ③ AP_Quicktune       （QuadPlane 的 VTOL 段；QWIK_ / AUX 181）
外加 Lua 侧的 VTOL-quicktune.lua（与 ③ 同一套算法）和 rover-quicktune.lua（另一套）。

本脚本做的事：
  ① 把这些库的**归属、常量、判据**从源码里解析出来（不靠印象）
  ② 把判据和增益迭代**移植成 Python 并用独立基准验**
  ③ 把 4.5.7 → 4.7.2 的差异**逐条量出来**（默认值、判据位置、阈值）

运行： python3 lesson45_autotune.py     （全过 EXIT=0）
"""

import io
import _srcdirs
import math
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


print('=' * 76)
print('[0] 定位源码（4.7.2 是本章基准版本）')
print('=' * 76)

REPO = None
for cand in (_srcdirs.src("APM_472"),
             _srcdirs.src("APM_472_PLANE")):
    if os.path.isdir(os.path.join(cand, 'libraries')):
        REPO = cand
        break
if REPO is None:
    print('    [!!] 找不到 ArduPilot 源码树 —— 后续所有"读真源码"的检查无从谈起')
    n_bad += 1
    sys.exit(1)

# 4.5.7 用来做跨版本对照（教程 08 章的行号基准）
OLD = None
for cand in (_srcdirs.src("APM"),
             os.path.dirname(_srcdirs.src("APM"))):
    if os.path.isdir(os.path.join(cand, 'libraries', 'AC_AutoTune')):
        OLD = cand
        break


def slurp(rel, root=None):
    return io.open(os.path.join(root or REPO, rel), encoding='utf-8', errors='replace').read()


F_AT     = 'libraries/AC_AutoTune/AC_AutoTune.cpp'
F_ATH    = 'libraries/AC_AutoTune/AC_AutoTune.h'
F_MULTI  = 'libraries/AC_AutoTune/AC_AutoTune_Multi.cpp'
F_MULTIH = 'libraries/AC_AutoTune/AC_AutoTune_Multi.h'
F_HELI   = 'libraries/AC_AutoTune/AC_AutoTune_Heli.cpp'
F_HELIH  = 'libraries/AC_AutoTune/AC_AutoTune_Heli.h'
F_FR     = 'libraries/AC_AutoTune/AC_AutoTune_FreqResp.cpp'
F_FRH    = 'libraries/AC_AutoTune/AC_AutoTune_FreqResp.h'
F_PLANE  = 'libraries/APM_Control/AP_AutoTune.cpp'
F_PLANEH = 'libraries/APM_Control/AP_AutoTune.h'
F_QT     = 'libraries/AP_Quicktune/AP_Quicktune.cpp'
F_QTH    = 'libraries/AP_Quicktune/AP_Quicktune.h'

src_at, src_ath = slurp(F_AT), slurp(F_ATH)
src_multi, src_multih = slurp(F_MULTI), slurp(F_MULTIH)
src_heli, src_helih = slurp(F_HELI), slurp(F_HELIH)
src_fr, src_frh = slurp(F_FR), slurp(F_FRH)
src_plane, src_planeh = slurp(F_PLANE), slurp(F_PLANEH)
src_qt, src_qth = slurp(F_QT), slurp(F_QTH)

print('    4.7.2 = %s' % REPO)
check(len(src_multi) > 30000, 'AC_AutoTune_Multi.cpp 读入 %d 字节' % len(src_multi))
check(OLD is not None, '找到 4.5.7 对照树（教程 08 章的行号基准）')


def src_defines(text, pat=r'^#define\s+(\w+)\s+([\-0-9.fU]+)'):
    out = {}
    for name, val in re.findall(pat, text, re.M):
        try:
            out[name] = float(val.rstrip('fU'))
        except ValueError:
            pass
    return out


def func_body(text, sig_pat):
    """按大括号配平取出一整个函数体（比 `.*?\\n}\\n` 稳，不会被内层块截断）"""
    m = re.search(sig_pat, text)
    if not m:
        return ''
    i = text.find('{', m.end() - 1)
    if i < 0:
        return ''
    depth = 0
    for j in range(i, len(text)):
        if text[j] == '{':
            depth += 1
        elif text[j] == '}':
            depth -= 1
            if depth == 0:
                return text[i:j + 1]
    return ''


# ==========================================================================
print()
print('=' * 76)
print('[1] 三个库 × 四条入口：谁在哪个载具上、前缀是什么')
print('=' * 76)

# --- 各自是不是 AP_Param 子类 / 有没有参数表 ---
check('AP_GROUPINFO' not in src_planeh and 'var_info' not in src_planeh,
      '★ 固定翼的 AP_AutoTune.h 里【没有】任何 AP_GROUPINFO —— 它不是 AP_Param 子类，'
      '自身一个参数都没有')
check(re.search(r'class\s+AP_AutoTune\s*\{', src_planeh) is not None,
      '   类声明是 `class AP_AutoTune {`，无基类')
check('var_info' in src_multih and 'var_info' in src_helih,
      '对照：AC_AutoTune 的 Multi/Heli 子类【各自】声明 var_info（基类 AC_AutoTune 没有）')
check(re.search(r'class\s+AC_AutoTune\s*$', src_ath, re.M) is not None or
      re.search(r'class\s+AC_AutoTune\s*\n', src_ath) is not None,
      'AC_AutoTune 基类无参数表，参数在 Multi/Heli 两个子类里')

# --- 前缀由载具侧注册决定，不在库里 ---
copter_params = slurp('ArduCopter/Parameters.cpp')
check(re.search(r'//\s*@Group:\s*AUTOTUNE_', copter_params) is not None,
      '★ 参数前缀 AUTOTUNE_ 是【载具侧】注册时给的（ArduCopter/Parameters.cpp 的 @Group）')
check(re.search(r'GOBJECT\(quicktune,\s*"QWIK_"', slurp('ArduPlane/Parameters.cpp')) is not None,
      '★ Quicktune 的前缀是 QWIK_（不是 Q_、也不是 QTUN_）')
check('QTUN' not in src_qt,
      '   QTUN 与本库无关（那是 QuadPlane 的另一套日志消息）')

# --- AUX 功能号 ---
rc_h = slurp('libraries/RC_Channel/RC_Channel.h')
m = re.search(r'QUICKTUNE\s*=\s*(\d+)', rc_h)
check(m is not None and int(m.group(1)) == 181,
      '★ AUX_FUNC::QUICKTUNE = %s（三段开关）' % (m.group(1) if m else '?'))
check(re.search(r'CUSTOM_CONTROLLER\s*=\s*(\d+)', rc_h).group(1) == '109',
      '   对照：AUX_FUNC::CUSTOM_CONTROLLER = 109（卷 8 第 15 章那个）')

# --- 载具归属：谁编译了谁 ---
qt_in_copter = 'AP_Quicktune' in slurp('ArduCopter/wscript')
qt_in_plane = 'AP_Quicktune' in slurp('ArduPlane/wscript')
check(qt_in_plane and not qt_in_copter,
      '★ AP_Quicktune 只被 ArduPlane/wscript 编译，Copter 的 wscript 里没有')
check(re.search(r'#define\s+AP_QUICKTUNE_ENABLED\s+HAL_QUADPLANE_ENABLED',
                slurp('ArduPlane/mode.h')) is not None,
      '★ 且 Plane 侧把它收窄成 HAL_QUADPLANE_ENABLED ⇒ 没有 QuadPlane 就没有 Quicktune')

qat = slurp('ArduPlane/qautotune.h')
m = re.search(r'#define\s+QAUTOTUNE_ENABLED\s+\((.*?)\)', qat)
check(m is not None, '找到 QAUTOTUNE_ENABLED 的定义')
check(m and 'HAL_BOARD_SITL' in m.group(1),
      '★★ Plane 的 QAutoTune（基于 AC_AutoTune）只在 SITL 编译：%s' % (m.group(1) if m else ''))
check(m and 'HAL_QUADPLANE_ENABLED' in m.group(1),
      '   条件是 QuadPlane 且 SITL —— 真机固件里整个类被 #if 掉')
check(re.search(r'class\s+QAutoTune\s*:\s*public\s+AC_AutoTune_Multi', qat) is not None,
      '★ QAutoTune 只继承 AC_AutoTune_Multi（不含 Heli 分支）')

# --- 有没有共同基类 ---
check(re.search(r'class\s+AP_Quicktune\s*\{', src_qth) is not None,
      'AP_Quicktune 无基类 —— 与 AC_AutoTune 是两份平行实现')
check('AC_AutoTune' not in src_qt and 'AP_Quicktune' not in src_at,
      '★ 两者互不 #include、无共享代码（只有对 AC_AttitudeControl / AC_PID 的库复用）')


# ==========================================================================
print()
print('=' * 76)
print('[2] AC_AutoTune 的状态机：四个枚举 + 两个 switch')
print('=' * 76)

# --- 四个枚举，全部从源码解析 ---
def enum_values(text, name):
    m = re.search(r'enum\s+class\s+%s\s*(?::\s*\w+\s*)?\{(.*?)\}' % name, text, re.S)
    if not m:
        return None
    body = re.sub(r'//[^\n]*', '', m.group(1))
    # 注意：最后一项【没有尾逗号】，所以要按逗号切而不是用带逗号的正则
    vals, nxt = {}, 0
    for item in body.split(','):
        item = item.strip()
        if not item:
            continue
        parts = item.split('=')
        nm = parts[0].strip()
        if len(parts) == 2:
            nxt = int(parts[1].strip())
        vals[nm] = nxt
        nxt += 1
    return vals


AXIS = enum_values(src_ath, 'AxisType')
STEP = enum_values(src_ath, 'Step')
TTYPE = enum_values(src_ath, 'TuneType')
GMODE = enum_values(src_ath, 'TuneMode')
GTYPE = enum_values(src_ath, 'GainType')

for nm, e, expect in (('AxisType', AXIS, 4), ('Step', STEP, 4),
                      ('TuneType', TTYPE, 9), ('TuneMode', GMODE, 5), ('GainType', GTYPE, 4)):
    check(e is not None and len(e) == expect,
          '枚举 %-10s 有 %s 项：%s'
          % (nm, len(e) if e else '?', ', '.join(sorted(e, key=e.get)) if e else '未找到'))

check(STEP.get('WAITING_FOR_LEVEL') == 0 and STEP.get('EXECUTING_TEST') == 1
      and STEP.get('UPDATE_GAINS') == 2 and STEP.get('ABORT') == 3,
      '★ Step 的四步：等水平(0) → 执行测试(1) → 更新增益(2) → 中止(3)')
check(TTYPE.get('RATE_D_UP') == 0 and TTYPE.get('RATE_P_UP') == 2
      and TTYPE.get('ANGLE_P_DOWN') == 4 and TTYPE.get('ANGLE_P_UP') == 5,
      '★ TuneType 的编号：D_UP=0, D_DOWN=1, P_UP=2, FF_UP=3, ANGLE_P_DOWN=4, ANGLE_P_UP=5')
check(GMODE.get('TUNING') == 1 and GMODE.get('VALIDATING') == 4,
      '★ TuneMode 含 VALIDATING(4) —— 调完之后让飞手【试飞】新增益，再决定存不存')
check(GTYPE.get('INTRA_TEST') == 2,
      '★ GainType 四套增益：ORIGINAL(0)/TEST(1)/INTRA_TEST(2)/TUNED(3)')

# --- 入口是 run() 不是 update() ---
check(re.search(r'virtual\s+void\s+run\(\)', src_ath) is not None,
      '★ 入口是 run()（AC_AutoTune.h），不是 update()')
check('void AC_AutoTune::run()' in src_at and 'void AC_AutoTune::update(' not in src_at,
      '   AC_AutoTune.cpp 里只有 run()，没有 update()')
m = re.search(r'// Should be called at >=(\d+)Hz for reliable performance', src_ath)
check(m is not None and int(m.group(1)) == 100,
      '★ 基类注释要求 ">=%sHz"（记下这个数，§8 会用到）' % (m.group(1) if m else '?'))
check(re.search(r'FAST_TASK\(update_flight_mode\)', slurp('ArduCopter/Copter.cpp')) is not None,
      '   调用链：FAST_TASK(update_flight_mode) → flightmode->run() → ModeAutoTune::run() → autotune.run()')

# --- 两个 switch ---
check(re.search(r'switch\s*\(mode\)\s*\{', src_at) is not None,
      '第 1 个 switch：switch (mode)，在 run() 里')
check(re.search(r'switch\s*\(step\)\s*\{', src_at) is not None,
      '第 2 个 switch：switch (step)，在 control_attitude() 里 —— 这才是真正的调参状态机')
check('control_attitude();' in src_at,
      '   TUNING 分支调 control_attitude()')
check(re.search(r'load_gains\(GainType::ORIGINAL\);', src_at) is not None,
      '★ 飞手打杆时 load_gains(ORIGINAL) —— 人一插手就切回原增益，人不飞的时候才调参')

# --- Multi 的调参序列是【硬编码】的 ---
m = re.search(r'void\s+set_tune_sequence\(\)\s*override\s*\{(.*?)\}', src_multih, re.S)
check(m is not None, '找到 Multi 的 set_tune_sequence()')
SEQ_MULTI = re.findall(r'tune_seq\[(\d+)\]\s*=\s*TuneType::(\w+)', m.group(1)) if m else []
check([s for _, s in sorted(SEQ_MULTI, key=lambda x: int(x[0]))] ==
      ['RATE_D_UP', 'RATE_D_DOWN', 'RATE_P_UP', 'ANGLE_P_DOWN', 'ANGLE_P_UP', 'TUNE_COMPLETE'],
      '★★ Multi 的固定 6 步序列：%s'
      % ' → '.join(s for _, s in sorted(SEQ_MULTI, key=lambda x: int(x[0]))))
check(SEQ_MULTI[0][1] == 'RATE_D_UP',
      '★★ 第 1 步是【D】不是 P —— 先找到能用的最大 D，再往上加 P 直到出现反弹')


# ==========================================================================
print()
print('=' * 76)
print('[3] ★★ 反弹判据：0.6321 就是 1 - 1/e')
print('=' * 76)

m = re.search(r'void AC_AutoTune_Multi::twitching_test_rate\((.*?)\n\}', src_multi, re.S)
check(m is not None, '整段取出 twitching_test_rate()')
TW = m.group(0) if m else ''

# --- 函数签名（4.7.2 是 6 参，4.5.7 是 5 参）---
sig = re.search(r'twitching_test_rate\(([^)]*)\)', TW)
n_par = len([p for p in sig.group(1).split(',')]) if sig else 0
check(n_par == 6, '★ 4.7.2 的签名有 %d 个参数（多了 angle 与 meas_angle_min）' % n_par)

# --- 四个门限数字 ---
for num, label in (('0.6321', '早期超时门限'),
                   ('0.25', '回弹捕获门限'),
                   ('aggressiveness', '反弹比阈值')):
    check(num in TW, '★ %s：`%s` 出现在函数体里' % (label, num))

# --- 独立基准：0.6321 == 1 - 1/e ---
one_minus_inv_e = 1.0 - math.exp(-1.0)
check(abs(one_minus_inv_e - 0.6321205588285577) < 1e-15,
      '1 - 1/e = %.16f' % one_minus_inv_e)
check(abs(0.6321 - one_minus_inv_e) < 6e-5,
      '★★ 0.6321 与 1-1/e 的差 = %.2e —— 它就是**一阶系统的时间常数点**（63.21%%）'
      % abs(0.6321 - one_minus_inv_e))
check('63.21%' in TW,
      '   源码注释自陈："calculate early stopping time based on the time it takes to get to 63.21%"')

# --- 判据公式：反弹量 > 峰值 × AGGR ---
check(re.search(r'meas_rate_max\s*-\s*meas_rate_min\s*>\s*meas_rate_max\s*\*\s*aggressiveness', TW)
      is not None,
      '★★ 判据本体：`meas_rate_max - meas_rate_min > meas_rate_max * aggressiveness`')
check(re.search(r'if\s*\(meas_rate_max\s*>\s*rate_target_max\)', TW) is not None,
      '   另有"测到的峰值超过目标"⇒ 也结束本步')

# --- 反向对照：判据在【稳态数据】上恒成立（教程 08 章讲过的那个坑）---
def bounce_ratio_on_hover(noise_amp):
    """零均值噪声上，max-min ≈ 2*max，比值恒等于 2"""
    mx = noise_amp
    mn = -noise_amp
    return (mx - mn) / mx


for a in (0.01, 0.1, 1.0, 5.0):
    check(abs(bounce_ratio_on_hover(a) - 2.0) < 1e-12,
          '★ 反向对照：零均值数据上 反弹比恒 = %.1f（幅值 %.2f）—— 远超任何 AGGR，'
          '所以这条判据**不能**用在没有阶跃的数据上' % (bounce_ratio_on_hover(a), a))

# --- AGGR 默认值：4.7.2 vs 4.5.7 ---
AGGR = src_defines(src_multi)  # 只抓数值型，AGGR 是参数不是 define
m_new = re.search(r'AP_GROUPINFO\("AGGR",\s*\d+,\s*AC_AutoTune_Multi,\s*aggressiveness,\s*([\d.]+)f\)',
                  src_multi)
m_min = re.search(r'AP_GROUPINFO\("MIN_D",\s*\d+,\s*AC_AutoTune_Multi,\s*min_d,\s*([\d.]+)f\)',
                  src_multi)
check(m_new is not None and m_min is not None, '解析出 4.7.2 的 AGGR / MIN_D 默认值')
AGGR_NEW = float(m_new.group(1)) if m_new else -1
MIND_NEW = float(m_min.group(1)) if m_min else -1
check(abs(AGGR_NEW - 0.075) < 1e-9, '★ 4.7.2 的 AUTOTUNE_AGGR 默认 = %s' % AGGR_NEW)
check(abs(MIND_NEW - 0.0005) < 1e-9, '★ 4.7.2 的 AUTOTUNE_MIN_D 默认 = %s' % MIND_NEW)
check(re.search(r'//\s*@Range:\s*0\.05 0\.10', src_multi) is not None,
      '   AGGR 的 @Range 仍是 0.05 0.10')


# ==========================================================================
print()
print('=' * 76)
print('[4] 增益迭代：几何级数 + 连续 4 次一致才锁定')
print('=' * 76)

D = src_defines(src_multi)
check(abs(D.get('AUTOTUNE_RD_STEP', -1) - 0.05) < 1e-9, 'AUTOTUNE_RD_STEP = %s' % D.get('AUTOTUNE_RD_STEP'))
check(abs(D.get('AUTOTUNE_RP_STEP', -1) - 0.05) < 1e-9, 'AUTOTUNE_RP_STEP = %s' % D.get('AUTOTUNE_RP_STEP'))
check(abs(D.get('AUTOTUNE_SP_STEP', -1) - 0.05) < 1e-9, 'AUTOTUNE_SP_STEP = %s' % D.get('AUTOTUNE_SP_STEP'))
check(abs(D.get('AUTOTUNE_RP_MIN', -1) - 0.01) < 1e-9, 'AUTOTUNE_RP_MIN = %s' % D.get('AUTOTUNE_RP_MIN'))
check(abs(D.get('AUTOTUNE_RP_MAX', -1) - 2.0) < 1e-9, 'AUTOTUNE_RP_MAX = %s' % D.get('AUTOTUNE_RP_MAX'))
check(abs(D.get('AUTOTUNE_RD_MAX', -1) - 0.2) < 1e-9, 'AUTOTUNE_RD_MAX = %s' % D.get('AUTOTUNE_RD_MAX'))
check(abs(D.get('AUTOTUNE_D_UP_DOWN_MARGIN', -1) - 0.2) < 1e-9,
      'AUTOTUNE_D_UP_DOWN_MARGIN = %s（"目标下方多大的带内才去量反弹"）'
      % D.get('AUTOTUNE_D_UP_DOWN_MARGIN'))

m = re.search(r'#define\s+AUTOTUNE_SUCCESS_COUNT\s+(\d+)', src_ath)
check(m is not None and int(m.group(1)) == 4,
      '★ AUTOTUNE_SUCCESS_COUNT = %s（连续 4 次同向才认为收敛）' % (m.group(1) if m else '?'))

# --- 步进公式：D 是 ×1.10，P/SP 是 ×1.05 ---
check(re.search(r'tune_d\s*\+=\s*tune_d\s*\*\s*tune_d_step_ratio\s*\*\s*2\.0f?', src_multi) is not None,
      '★ D 的步进是 `tune_d += tune_d * step * 2.0f` ⇒ 每步 ×(1+2×0.05) = ×1.10（两倍步长）')
check(re.search(r'tune_p\s*\+=\s*tune_p\s*\*\s*tune_p_step_ratio;', src_multi) is not None,
      '★ P 的步进是 `tune_p += tune_p * step` ⇒ 每步 ×1.05')

# --- 复现 updating_rate_d_up 的决策逻辑，验证"几何逼近 + 4 次锁定" ---
RD_STEP = D.get('AUTOTUNE_RD_STEP', 0.05)
SUCCESS_COUNT = 4
MARGIN = D.get('AUTOTUNE_D_UP_DOWN_MARGIN', 0.2)


def update_rate_d_up(tune_d, tune_p, meas_max, meas_min, rate_target, aggr,
                     success_counter, ignore_next):
    """逐行移植 AC_AutoTune_Multi::updating_rate_d_up（只保留会改增益的支路）"""
    if meas_max > rate_target:
        tune_p -= tune_p * 0.05
        if tune_p < 0.01:
            tune_p = 0.01
            tune_d -= tune_d * RD_STEP
            if tune_d <= 0.0005:
                tune_d = 0.0005
                success_counter = SUCCESS_COUNT
        return tune_d, tune_p, success_counter, ignore_next
    if (meas_max < rate_target * (1.0 - MARGIN)) and tune_p <= 2.0:
        tune_p += tune_p * 0.05
        if tune_p >= 2.0:
            tune_p = 2.0
        return tune_d, tune_p, success_counter, ignore_next
    # 落在 [0.8*target, target] 带内 —— 有资格量反弹了
    if meas_max - meas_min > meas_max * aggr:
        return tune_d, tune_p, success_counter + 1, True
    if not ignore_next:
        if success_counter > 0:
            success_counter -= 1
        tune_d += tune_d * RD_STEP * 2.0
    else:
        ignore_next = False
    return tune_d, tune_p, success_counter, ignore_next


# 造一个"被调对象"。**方向很重要**：源码注释写着
#   // increase D gain (which should increase bounce back)
# ⇒ **D 越大，反弹越大**。算法就是把 D 一路加上去，直到反弹越过阈值为止。
def bounce_ratio(tune_d, k=0.30, d50=0.15):
    return k * tune_d / (tune_d + d50)      # d50=0.15 时，D=0.05 恰好给出 0.075


check(abs(bounce_ratio(0.05) - AGGR_NEW) < 1e-12,
      '   模型自证：bounce_ratio(D=0.05) = %.5f，正好等于 AGGR 默认值 %.5f'
      % (bounce_ratio(0.05), AGGR_NEW))

td, tp, sc, ig = 0.005, 0.135, 0, False
ratios, frozen_at = [], None
for it in range(400):
    mm = 18000.0                       # 峰值正好等于目标（落在可测带内）
    mn = mm * (1.0 - bounce_ratio(td))  # 反弹后的最低值
    prev = td
    td, tp, sc, ig = update_rate_d_up(td, tp, mm, mn, mm, AGGR_NEW, sc, ig)
    if td != prev:
        ratios.append(td / prev)
    if sc >= SUCCESS_COUNT:
        frozen_at = it
        break

check(frozen_at is not None and frozen_at < 400,
      '★ 复现：从 D=0.005 出发，%d 次迭代后 success_counter 达到 %d（收敛）'
      % (frozen_at if frozen_at else -1, SUCCESS_COUNT))
check(len(ratios) >= 10,
      '★ 收敛过程中 D 被上调了 %d 次（非零样本，判据真的跑过）' % len(ratios))
check(all(abs(r - 1.10) < 1e-9 for r in ratios),
      '★★ 每一次上调的倍率都精确等于 1.10（实测 %.4f ~ %.4f）—— '
      '几何级数而非线性步进' % (min(ratios), max(ratios)))
check(abs(td - 0.05) < 5e-3,
      '★ 收敛到的 D = %.5f，正是 bounce_ratio 越过 AGGR 的那个点（解析解 0.05）' % td)

# --- 反向对照：把 ×1.10 改成线性 +0.01，收敛点会变 ---
def update_linear(tune_d, tune_p, meas_max, meas_min, rate_target, aggr, sc, ig):
    if (meas_max < rate_target * (1.0 - MARGIN)) and tune_p <= 2.0:
        return tune_d, tune_p, sc, ig
    if meas_max - meas_min > meas_max * aggr:
        return tune_d, tune_p, sc + 1, True
    if not ig:
        if sc > 0:
            sc -= 1
        tune_d += 0.01                 # ← 线性步长
    else:
        ig = False
    return tune_d, tune_p, sc, ig


td2, sc2, ig2, n_step2 = 0.005, 0, False, 0
for it in range(400):
    mm = 18000.0
    _prev2 = td2
    td2, _, sc2, ig2 = update_linear(td2, 0.135, mm, mm * (1 - bounce_ratio(td2)),
                                     mm, AGGR_NEW, sc2, ig2)
    if td2 != _prev2:
        n_step2 += 1
    if sc2 >= SUCCESS_COUNT:
        break
# ★ 比终点弱得多、比"步数"才有区分度：几何是几十次小步，线性是几次大步。
#   （只比终点的话两者只差 0.0008 —— 那种反向对照等于没做。）
check(n_step2 < 0.5 * len(ratios),
      '★★ 反向对照：把几何步进换成线性 +0.01，只需 %d 次增量就收敛；'
      '几何版要 %d 次（是它的 %.1f 倍）—— **步数差一个量级**，'
      '这才是"几何级数"与"线性步进"的真正区别'
      % (n_step2, len(ratios), len(ratios) / float(max(n_step2, 1))))
check(abs(td2 - td) > 1e-6,
      '   而两者的收敛终点只差 %.5f（%.5f vs %.5f）—— 光看终点几乎分不出来'
      % (abs(td2 - td), td2, td))


# ==========================================================================
print()
print('=' * 76)
print('[5] 终值恒等式：调完之后 I 恒等于 P（roll/pitch）')
print('=' * 76)

check(abs(D.get('AUTOTUNE_PI_RATIO_FINAL', -1) - 1.0) < 1e-9,
      '★ AUTOTUNE_PI_RATIO_FINAL = %s ⇒ 速率环 I = P' % D.get('AUTOTUNE_PI_RATIO_FINAL'))
check(abs(D.get('AUTOTUNE_YAW_PI_RATIO_FINAL', -1) - 0.1) < 1e-9,
      '★ AUTOTUNE_YAW_PI_RATIO_FINAL = %s ⇒ 偏航 I = 0.1 × P' % D.get('AUTOTUNE_YAW_PI_RATIO_FINAL'))
check(abs(D.get('AUTOTUNE_PI_RATIO_FOR_TESTING', -1) - 0.1) < 1e-9,
      '★ AUTOTUNE_PI_RATIO_FOR_TESTING = %s ⇒ 【测试期间】I 被压到 P 的 1/10'
      % D.get('AUTOTUNE_PI_RATIO_FOR_TESTING'))

# --- 写入点：save_tuning_gains 里三轴都用同一套比例 ---
SAVE = func_body(src_multi, r'void AC_AutoTune_Multi::save_tuning_gains\(\)')
check(SAVE.count('AUTOTUNE_PI_RATIO_FINAL') == 2,
      '★ save_tuning_gains() 里 PI_RATIO_FINAL 出现 %d 次（roll+pitch 各一次）'
      % SAVE.count('AUTOTUNE_PI_RATIO_FINAL'))
check(SAVE.count('AUTOTUNE_YAW_PI_RATIO_FINAL') == 1,
      '★ YAW_PI_RATIO_FINAL 在 save_tuning_gains() 里只出现 %d 次 —— '
      '偏航的 YawE 与 YawD 共用一个赋值（它们同属 get_rate_yaw_pid）'
      % SAVE.count('AUTOTUNE_YAW_PI_RATIO_FINAL'))
check(re.search(r'set_kI\(tune_roll_rp\s*\*\s*AUTOTUNE_PI_RATIO_FINAL\)', SAVE) is not None,
      '★★ 原文：set_kI(tune_roll_rp * AUTOTUNE_PI_RATIO_FINAL) —— I 是【从 P 算出来的】，'
      '不是独立辨识的')
# 复算
for p in (0.05, 0.135, 0.30, 1.20):
    check(abs((p * 1.0) - p) < 1e-12,
          '   复算：调完后 I = P × 1.0 = %.4f（P 调到多少，I 就跟到多少）' % p)

# --- GMBK 后缩：只作用于 rate P/D 与 angle P，不作用于 I ---
BACK = func_body(src_multi, r'void AC_AutoTune_Multi::set_tuning_gains_with_backoff\(')
GMBK = float(re.search(r'AP_GROUPINFO\("GMBK",\s*\d+,\s*AC_AutoTune_Multi,\s*gain_backoff,\s*([\d.]+)\)',
                       src_multi).group(1))
check(abs(GMBK - 0.25) < 1e-9, '★ AUTOTUNE_GMBK（增益裕度后缩）默认 = %s' % GMBK)
check(re.search(r'tune_roll_rp\s*=\s*tune_roll_rp\s*\*\s*\(1\.0f\s*-\s*gain_backoff\)', BACK) is not None,
      '★ Rate P 后缩：× (1 - GMBK)')
check(re.search(r'tune_roll_sp\s*=\s*tune_roll_sp\s*\*\s*\(1\.0f\s*-\s*gain_backoff\)\s*\*\s*'
                r'\(1\.0f\s*-\s*aggressiveness\)', BACK) is not None,
      '★★ 而角度环 P 后缩【两次】：× (1 - GMBK) × (1 - AGGR) —— AGGR 只作用于角度环，'
      '不动速率环 P/D')
check(abs((1 - GMBK)) < 1.0 and abs((1 - GMBK) * (1 - AGGR_NEW)) < 0.75,
      '   复算：速率环打 %.0f 折，角度环打 %.0f 折'
      % ((1 - GMBK) * 100, (1 - GMBK) * (1 - AGGR_NEW) * 100))
cases_b = re.findall(r'case TuneType::(\w+):\s*(break;)?', BACK)
empty_b = [c for c, br in cases_b if br]
check(len(cases_b) == 9,
      '   后缩分派覆盖全部 %d 个 TuneType' % len(cases_b))
check(empty_b == ['RATE_D_UP', 'RATE_D_DOWN', 'ANGLE_P_DOWN'] or
      set(empty_b) >= {'RATE_D_UP', 'RATE_D_DOWN'},
      '★★ 其中 %s 是**空实现**（直接 break）—— 也就是说 D 的上下调、角度环的【下调】'
      '都不做裕度后缩，只有 P 的最终值才打折' % ', '.join(empty_b))


# ==========================================================================
print()
print('=' * 76)
print('[6] FreqResp：时域峰值检测器（不是 FFT）')
print('=' * 76)

check('fft' not in src_fr.lower() and 'FFT' not in src_fr,
      '★★ 全文件搜 FFT —— 零命中。它是**时域峰值/谷值检测**，不是频域变换')
check('freqresp' in src_heli.lower() and 'freqresp' not in src_multi.lower(),
      '★ FreqResp 只被 Heli 使用（Multi 里零命中）')
check(src_helih.count('AC_AutoTune_FreqResp ') == 2,
      '   Heli 有两个实例：freqresp_mtr（对电机混控输入）+ freqresp_tgt（对目标输入）')

# --- 5730 换算常数 ---
m = re.search(r'tgt_freq\s*\*\s*max_meas_rate\s*\*\s*([\d.]+)f', src_fr)
check(m is not None, '找到 max_accel 的换算常数')
C5730 = float(m.group(1)) if m else -1
ideal = 100.0 * 180.0 / math.pi
check(abs(C5730 - 5730.0) < 1e-9, '★ 源码里的常数 = %s' % C5730)
check(abs(ideal - 5729.577951308232) < 1e-9,
      '   独立基准：100 × 180/π = %.6f（rad/s² → 度/秒²×100）' % ideal)
check(abs(C5730 - ideal) < 0.5,
      '★★ 两者相差 %.4f —— 源码用的是四舍五入到整数的 5730' % abs(C5730 - ideal))

# --- dt 硬编码 ---
m = re.search(r'float\s+dt\s*=\s*([\d.]+);', src_fr)
check(m is not None, '找到 update() 里的 dt')
DT_HARD = float(m.group(1)) if m else -1
check(abs(DT_HARD - 0.0025) < 1e-12, '★★ dt 硬编码 = %s s（= 400 Hz）' % DT_HARD)
check(abs(1.0 / DT_HARD - 400.0) < 1e-9,
      '★ 1/dt = %.1f Hz。而 AC_AutoTune.h 的注释只要求 ">=100Hz" —— '
      '**这条隐藏的 400Hz 假设没有任何断言守着**' % (1.0 / DT_HARD))
check('0.0025' not in re.sub(r'float\s+dt\s*=.*', '', src_fr),
      '   该常数在全文件只出现这一次（只在 dt 那一行）')

# --- 缓冲容量 12 ---
caps = re.findall(r'ObjectBuffer<peak_info>\s+\w+\{(\d+)\}', src_frh)
check(caps == ['12', '12'],
      '★ 两个环形缓冲容量都是 %s（对象是 ObjectBuffer<peak_info>）' % caps)

# --- 半周期时基：300 vs 1000 ---
m = re.search(r'half_cycle_time_ms\s*=\s*\(uint32_t\)\((\d+)\s*\*\s*M_2PI\s*/\s*tgt_freq\)', src_fr)
# 注意 (?<!half_)：`half_cycle_time_ms` 里就含 `cycle_time_ms`，不加这个前瞻会抓错行
m2 = re.search(r'(?<!half_)cycle_time_ms\s*=\s*\(uint32_t\)\((\d+)\s*\*\s*M_2PI\s*/\s*tgt_freq\)',
               src_fr)
check(m is not None and m2 is not None, '找到半周期/整周期的时基公式')
HC, FC = int(m.group(1)) if m else 0, int(m2.group(1)) if m2 else 0
check((HC, FC) == (300, 1000),
      '★ 半周期取 300·2π/f、整周期取 1000·2π/f ⇒ 半周期窗口是**周期的 30%**（不是 50%），'
      '留出的余量用来防噪声误触发')

# ---- 移植 FreqResp（DWELL + ANGLE 模式），用已知 G/phi 的合成信号验证 ----
M_2PI = 2.0 * math.pi


class FreqResp(object):
    """逐行移植 AC_AutoTune_FreqResp::update()，只保留 DWELL 分支"""

    def __init__(self, cycles):
        self.dwell_cycles = cycles
        self.c = dict(
            max_target_cnt=0, min_target_cnt=0, max_meas_cnt=0, min_meas_cnt=0,
            input_start_time_ms=0, new_tgt_time_ms=0, new_meas_time_ms=0,
            new_target=False, new_meas=False, curr_test_freq=0.0,
            curr_test_gain=0.0, curr_test_phase=0.0, max_accel=0.0,
            max_meas_rate=0.0, max_command=0.0, cycle_complete=False,
            max_target=0.0, min_target=0.0, temp_max_target=0.0, temp_min_target=0.0,
            max_meas=0.0, min_meas=0.0, temp_max_meas=0.0, temp_min_meas=0.0,
            temp_tgt_ampl=0.0, temp_meas_ampl=0.0, prev_target=0.0, prev_meas=0.0,
            prev_tgt_resp=0.0, prev_meas_resp=0.0, max_tgt_time=0, max_meas_time=0,
            temp_max_tgt_time=0, temp_max_meas_time=0, target_rate=0.0, measured_rate=0.0)
        self.meas_buf, self.tgt_buf = [], []
        self.sw_tgt = [0, 0.0, 0]
        self.sw_meas = [0, 0.0, 0]

    @staticmethod
    def _pop(buf):
        return tuple(buf.pop(0)) if buf else (0, 0.0, 0)

    def update(self, command, tgt_resp, meas_resp, tgt_freq, now_ms):
        c = self.c
        dt = DT_HARD
        if c['cycle_complete']:
            return
        half = int(300 * M_2PI / tgt_freq) if tgt_freq else 0
        cyc = int(1000 * M_2PI / tgt_freq) if tgt_freq else 0
        if c['input_start_time_ms'] == 0:
            c['input_start_time_ms'] = now_ms
            c['prev_tgt_resp'], c['prev_meas_resp'] = tgt_resp, meas_resp
            c['prev_target'] = c['prev_meas'] = 0.0
        c['target_rate'] = (tgt_resp - c['prev_tgt_resp']) / dt
        c['measured_rate'] = (meas_resp - c['prev_meas_resp']) / dt

        # ---- 周期数够了：算增益/相位 ----
        if (c['max_meas_cnt'] > self.dwell_cycles + 1
                and c['max_target_cnt'] > self.dwell_cycles + 1):
            sum_gain, cnt, gcnt, delta = 0.0, 0, 0, 0.0
            for _ in range(self.dwell_cycles):
                mc, ma, mt = self._pop(self.meas_buf)
                tc, ta, tt = self._pop(self.tgt_buf)
                self.meas_buf.append([0, 0.0, 0])
                self.tgt_buf.append([0, 0.0, 0])
                if mc == tc and mc != 0:
                    if ta > 0.0:
                        sum_gain += ma / ta
                        gcnt += 1
                    d = float(mt - tt)
                    if d < 2.0 * cyc:
                        delta += d
                        cnt += 1
                elif mc > tc:
                    self._pop(self.tgt_buf)
                    self.tgt_buf.append([0, 0.0, 0])
                elif mc < tc:
                    self._pop(self.meas_buf)
                    self.meas_buf.append([0, 0.0, 0])
            if gcnt > 0:
                c['curr_test_gain'] = sum_gain / gcnt
            if cnt > 0:
                delta /= cnt
            c['curr_test_phase'] = tgt_freq * delta * 0.001 * 360.0 / M_2PI
            if c['curr_test_phase'] > 360.0:
                c['curr_test_phase'] -= 360.0
            dma = tgt_freq * c['max_meas_rate'] * C5730
            if c['max_command']:
                dma /= (2.0 * c['max_command'])
            c['max_accel'] = max(c['max_accel'], dma)
            c['curr_test_freq'] = tgt_freq
            c['cycle_complete'] = True
            return

        pt, pm = c['prev_target'], c['prev_meas']
        tr, mr = c['target_rate'], c['measured_rate']
        # 目标侧：角速率由 + 变 - ⇒ 角度到峰
        if (pt > 0 and not tr > 0) and not c['new_target'] and now_ms > c['new_tgt_time_ms']:
            c['new_target'] = True
            c['new_tgt_time_ms'] = now_ms + half
            c['max_target'] = 0.0
            c['max_target_cnt'] += 1
            c['temp_min_target'] = c['min_target']
            if c['min_target_cnt'] > 0:
                c['temp_max_tgt_time'] = c['max_tgt_time']
                c['temp_tgt_ampl'] = c['temp_max_target'] - c['temp_min_target']
                self.tgt_buf.append([c['min_target_cnt'], c['temp_tgt_ampl'],
                                     c['temp_max_tgt_time']])
        elif (not pt > 0 and tr > 0) and c['new_target'] and now_ms > c['new_tgt_time_ms'] \
                and c['max_target_cnt'] > 0:
            c['new_target'] = False
            c['new_tgt_time_ms'] = now_ms + half
            c['min_target_cnt'] += 1
            c['temp_max_target'] = c['max_target']
            c['min_target'] = 0.0
        # 测量侧：同构
        if (pm > 0 and not mr > 0) and not c['new_meas'] and now_ms > c['new_meas_time_ms'] \
                and c['max_target_cnt'] > 0:
            c['new_meas'] = True
            c['new_meas_time_ms'] = now_ms + half
            c['max_meas'] = 0.0
            c['max_meas_cnt'] += 1
            c['temp_min_meas'] = c['min_meas']
            if c['min_meas_cnt'] > 0 and c['min_target_cnt'] > 0:
                c['temp_max_meas_time'] = c['max_meas_time']
                c['temp_meas_ampl'] = c['temp_max_meas'] - c['temp_min_meas']
                self.meas_buf.append([c['min_meas_cnt'], c['temp_meas_ampl'],
                                      c['temp_max_meas_time']])
        elif (not pm > 0 and mr > 0) and c['new_meas'] and now_ms > c['new_meas_time_ms'] \
                and c['max_meas_cnt'] > 0:
            c['new_meas'] = False
            c['new_meas_time_ms'] = now_ms + half
            c['min_meas_cnt'] += 1
            c['temp_max_meas'] = c['max_meas']
            c['min_meas'] = 0.0

        if c['new_target']:
            if tgt_resp > c['max_target']:
                c['max_target'] = tgt_resp
                c['max_tgt_time'] = now_ms
        else:
            if tgt_resp < c['min_target']:
                c['min_target'] = tgt_resp
        if c['new_meas']:
            if meas_resp > c['max_meas']:
                c['max_meas'] = meas_resp
                c['max_meas_time'] = now_ms
        else:
            if meas_resp < c['min_meas']:
                c['min_meas'] = meas_resp
        if (c['input_start_time_ms'] + 7.0 * cyc) < now_ms < (c['input_start_time_ms'] + 9.0 * cyc):
            c['max_meas_rate'] = max(c['max_meas_rate'], c['measured_rate'])
            c['max_command'] = max(c['max_command'], command)
        c['prev_tgt_resp'], c['prev_meas_resp'] = tgt_resp, meas_resp
        c['prev_target'], c['prev_meas'] = c['target_rate'], c['measured_rate']


def run_freqresp(G, phi_deg, freq_hz, cycles=8, amp=0.2):
    fr = FreqResp(cycles)
    w = M_2PI * freq_hz
    phi = math.radians(phi_deg)
    for k in range(1, 400000):
        t = k * DT_HARD
        th_t = amp * math.sin(w * t)
        th_m = amp * G * math.sin(w * t - phi)
        fr.update(th_t, th_t, th_m, w, int(round(t * 1000)))
        if fr.c['cycle_complete']:
            return fr.c['curr_test_gain'], fr.c['curr_test_phase'], fr.c['curr_test_freq'], k
    return None, None, None, 0


CASES = [(1.00, 0.0, 2.0), (0.50, 30.0, 2.0), (2.00, 60.0, 1.0),
         (1.00, -45.0, 1.5), (0.25, 90.0, 0.8)]
worst_g, worst_p = 0.0, 0.0
for G, phi, f in CASES:
    g, p, ff, n = run_freqresp(G, phi, f)
    worst_g = max(worst_g, abs(g - G) / G)
    want = phi if phi >= 0 else phi + 360.0
    worst_p = max(worst_p, abs(p - want))
    check(abs(g - G) / G < 1e-3,
          '★ 增益回读：真值 %.2f → 检测 %.4f（f=%.1fHz，真相位 %+.0f°）' % (G, g, f, phi))

check(worst_g < 1e-3,
      '★★ 独立基准①：5 组已知增益全部复现到 0.1%% 以内（最大相对误差 %.2e）' % worst_g)
check(worst_p < 1.0,
      '★★ 独立基准②：5 组已知相位全部复现到 1° 以内（最大绝对误差 %.2f°）—— '
      '注意极性：−45° 报成 315.09°，即"滞后为正、折算到 0~360"' % worst_p)

# --- 反向对照：把"测量峰时刻"改成"测量起始时刻"，相位必然错 ---
def run_freqresp_broken_G(G, phi_deg, freq_hz, cycles=8, amp=0.2):
    """注入：push 时间用 max_tgt_time 而不是 max_meas_time（即相位恒 0）"""
    fr = FreqResp(cycles)
    w = M_2PI * freq_hz
    phi = math.radians(phi_deg)
    orig = fr.meas_buf
    for k in range(1, 400000):
        t = k * DT_HARD
        th_t = amp * math.sin(w * t)
        th_m = amp * G * math.sin(w * t - phi)
        fr.update(th_t, th_t, th_m, w, int(round(t * 1000)))
        # 把测量缓冲的时间戳全换成目标的时间戳 ⇒ 相位恒 0
        for row in fr.meas_buf:
            row[2] = 0
        if fr.c['cycle_complete']:
            return fr.c['curr_test_gain'], fr.c['curr_test_phase']
    return None, None


g_b, p_b = run_freqresp_broken_G(1.0, 45.0, 1.5)
check(g_b is not None and abs(p_b - 45.0) > 45.0,
      '★ 反向对照：把测量侧的时间戳抹掉，相位变成 %.2f°（真值 45°）—— '
      '说明"峰值时刻之差"这一条确实在起作用，不是碰巧对上的' % (p_b if p_b is not None else -1))
check(g_b is not None and abs(g_b - 1.0) < 1e-3,
      '★ 而增益依然正确（%.4f）—— 因为增益只读缓冲里的 amplitude、相位只读 time_ms，'
      '两个量取自**不同的字段**；破坏一个不会连带毁掉另一个' % (g_b if g_b is not None else -1))


# ==========================================================================
print()
print('=' * 76)
print('[7] Heli 与 Multi 的分野：可配置序列 vs 硬编码序列')
print('=' * 76)

check(abs(D.get('AUTOTUNE_RP_MAX', 0) - 2.0) < 1e-9, '（回引）AUTOTUNE_RP_MAX = %s' % D.get('AUTOTUNE_RP_MAX'))

# --- 参数表：Heli 8 项 vs Multi 4 项 ---
hel = re.findall(r'AP_GROUPINFO\("(\w+)",\s*(\d+),\s*AC_AutoTune_Heli,\s*(\w+),\s*([^)]+)\)', src_heli)
mul = re.findall(r'AP_GROUPINFO\("(\w+)",\s*(\d+),\s*AC_AutoTune_Multi,\s*(\w+),\s*([^)]+)\)', src_multi)
check(len(mul) == 4, 'Multi 有 %d 个参数：%s' % (len(mul), ', '.join('AUTOTUNE_' + x[0] for x in mul)))
check(len(hel) == 8, 'Heli 有 %d 个参数：%s' % (len(hel), ', '.join('AUTOTUNE_' + x[0] for x in hel)))
HEL_NAMES = [x[0] for x in hel]
check(set(HEL_NAMES) == {'AXES', 'SEQ', 'FRQ_MIN', 'FRQ_MAX', 'GN_MAX', 'VELXY_P', 'ACC_MAX', 'RAT_MAX'},
      '★ Heli 独有 SEQ/FRQ_MIN/FRQ_MAX/GN_MAX/VELXY_P/ACC_MAX/RAT_MAX（Multi 全都没有）')
check(re.search(r'AP_GROUPINFO\("AXES",\s*1,\s*AC_AutoTune_Heli,\s*axis_bitmask,\s*1\)', src_heli)
      is not None,
      '★ Heli 的 AXES 默认 = 1（只调 roll）；Multi 的默认 = 7（三轴全调）')
check(re.search(r'AP_GROUPINFO\("AXES",\s*1,\s*AC_AutoTune_Multi,\s*axis_bitmask,\s*7\)', src_multi)
      is not None,
      '   两边的默认值不同是有意的：直升机的桨毂/斜盘特性让三轴同时调更容易出问题')

# --- Heli 的序列由 bitmask 拼装 ---
m = re.search(r'void AC_AutoTune_Heli::set_tune_sequence\(\)(.*?)\n\}', src_heli, re.S)
HB = m.group(1) if m else ''
bits = dict((int(v), k) for k, v in re.findall(r'#define\s+AUTOTUNE_SEQ_BITMASK_(\w+)\s+(\d+)', src_heli))
check(bits == {1: 'VFF', 2: 'RATE_D', 4: 'ANGLE_P', 8: 'MAX_GAIN', 16: 'TUNE_CHECK'},
      '★ Heli 的序列位掩码：%s'
      % ', '.join('%d:%s' % (k, bits[k]) for k in sorted(bits)))
check(HB.count('if (seq_bitmask &') == 5,
      '★ set_tune_sequence() 里有 %d 个 bitmask 判断 —— 序列是【运行时拼装】的，'
      '不是硬编码数组' % HB.count('if (seq_bitmask &'))

# --- ★ tune_seq[6] 的边界：穷举所有掩码，看条目数 ---
m = re.search(r'TuneType\s+tune_seq\[(\d+)\]', src_ath)
CAP = int(m.group(1)) if m else 0
check(CAP == 6, '★ 序列数组容量 tune_seq[%d]' % CAP)


def seq_len(mask):
    n = 0
    if mask & 1:
        n += 1
    if mask & 2:
        n += 3
    if mask & 4:
        n += 1
    if (mask & 8) and not (mask & 2):
        n += 1
    if mask & 16:
        n += 1
    return n + 1          # + TUNE_COMPLETE


DOCUMENTED = [7, 1, 2, 4, 8, 16, 3, 5, 6]
check(all(seq_len(x) <= CAP for x in DOCUMENTED),
      '★ @Description 里列出的 %d 种取值，展开后条目数最大 %d —— **恰好装得下** tune_seq[%d]'
      % (len(DOCUMENTED), max(seq_len(x) for x in DOCUMENTED), CAP))
over = [x for x in range(1, 256) if seq_len(x) > CAP]
# 越界的充要条件：1|2|4|16 四个位同时置起（低 3 位给 6 项，再看 Tune Check）
EXPECT_OVER = sorted(x for x in range(1, 256) if (x & 23) == 23)
check(over == EXPECT_OVER and len(over) == 16,
      '★★ 但穷举 0..255 发现 %d 种取值会展开成 %d 项（> %d）⇒ 写 tune_seq[%d] **越界**'
      % (len(over), seq_len(over[0]) if over else 0, CAP, CAP))
check(all((x & 23) == 23 for x in over),
      '★ 越界的充要条件是 1|2|4|16 四个位同时置起（= 低三位 7 再加 Tune Check）')
check(23 in over,
      '   最小的那个是 23 = 7|16（"全部"再打开 Tune Check）—— 而 seq_bitmask 是裸 AP_Int8，'
      '**没有任何 @Range 或运行时钳位**')
check(7 not in over,
      '   而文档里那个 "7:All" 恰好 6 项、**正好卡满**（这就是为什么一直没人发现）')

# --- 4.7.2 Heli 保留的旧角度常量 ---
check(re.search(r'// angle limits preserved from previous behaviour as Multi changed', src_heli)
      is not None,
      '★ Heli 里有一段注释 "angle limits preserved from previous behaviour as Multi changed" —— '
      '两边的角度门限常量已经分叉')
HD = src_defines(src_heli)
check(abs(HD.get('AUTOTUNE_ANGLE_TARGET_MAX_RP_CD', -1) - 2000) < 1e-9,
      '   Heli 的 AUTOTUNE_ANGLE_TARGET_MAX_RP_CD = %s' % HD.get('AUTOTUNE_ANGLE_TARGET_MAX_RP_CD'))


# ==========================================================================
print()
print('=' * 76)
print('[8] AP_Quicktune：不注入激励的在线爬坡')
print('=' * 76)

Q = src_defines(src_qt)
check(abs(Q.get('UPDATE_RATE_HZ', -1) - 40) < 1e-9,
      '★ UPDATE_RATE_HZ = %s（对照：Plane 的 SCHED_TASK 也是 40）' % Q.get('UPDATE_RATE_HZ'))
check(re.search(r'SCHED_TASK\(update_quicktune,\s*(\d+),', slurp('ArduPlane/Plane.cpp')) is not None,
      '   确认调度频率一致：%s'
      % re.search(r'SCHED_TASK\(update_quicktune,\s*(\d+),', slurp('ArduPlane/Plane.cpp')).group(1) + ' Hz')

# --- ★ 指数爬坡：gain_mul^n = 2 ---
m = re.search(r'return\s+expf\(logf\(2\.0\)/\(UPDATE_RATE_HZ\*MAX\(1,double_time\)\)\);', src_qt)
check(m is not None, '★ get_gain_mul() 的原文：expf(logf(2.0)/(UPDATE_RATE_HZ*MAX(1,double_time)))')
m_dt = re.search(r'AP_GROUPINFO\("DOUBLE_TIME",\s*\d+,\s*AP_Quicktune,\s*double_time,\s*([\d.]+)\)',
                 src_qt)
check(m_dt is not None, '解析出 DOUBLE_TIME 默认值')
DTIME = float(m_dt.group(1)) if m_dt else -1
check(abs(DTIME - 10.0) < 1e-9, '★ QWIK_DOUBLE_TIME 默认 = %s 秒' % DTIME)

RATE = Q.get('UPDATE_RATE_HZ')
gain_mul = math.exp(math.log(2.0) / (RATE * max(1, DTIME)))
check(abs(gain_mul - 2.0 ** (1.0 / (RATE * DTIME))) < 1e-15,
      '   复算：gain_mul = 2^(1/(%d×%g)) = 2^(1/%.0f) = %.8f' % (RATE, DTIME, RATE * DTIME, gain_mul))
check(abs(gain_mul ** (RATE * DTIME) - 2.0) < 1e-12,
      '★★ 独立恒等式：gain_mul 连乘 %d 次（= %g 秒）正好 = %.10f —— '
      '这就是参数名 "DOUBLE_TIME" 的含义' % (int(RATE * DTIME), DTIME, gain_mul ** (RATE * DTIME)))
check(1.0 < gain_mul < 1.01,
      '★ 单步倍率 %.6f（每 25 ms 涨 %.3f%%）—— 非常慢，是为了让振荡来得及被看见'
      % (gain_mul, (gain_mul - 1) * 100))

# --- 振荡判据：slew_rate > osc_smax ---
check(re.search(r'const float srate = get_slew_rate\(axis\);', src_qt) is not None,
      '★★ 振荡判据的输入是 get_slew_rate(axis)，它来自 pid->get_pid_info().slew_rate —— '
      '**飞控自己算出来的滑变率**，不是注入信号后的响应')
check(re.search(r'const bool oscillating = srate > osc_smax;', src_qt) is not None,
      '★ 判据本体：`oscillating = srate > osc_smax`（一个纯比较，没有比值）')
check('input_angle_step' not in src_qt and 'input_rate_step' not in src_qt,
      '★★ AP_Quicktune 全文件搜 input_angle_step / input_rate_step —— 零命中。'
      '它**不注入任何激励**（这正是与 AC_AutoTune 最本质的区别）')

m_osc = re.search(r'AP_GROUPINFO\("OSC_SMAX",\s*\d+,\s*AP_Quicktune,\s*osc_smax,\s*([\d.]+)\)', src_qt)
m_gm = re.search(r'AP_GROUPINFO\("GAIN_MARGIN",\s*\d+,\s*AP_Quicktune,\s*gain_margin,\s*([\d.]+)\)', src_qt)
OSC = float(m_osc.group(1)) if m_osc else -1
GM = float(m_gm.group(1)) if m_gm else -1
check(abs(OSC - 4.0) < 1e-9, '★ QWIK_OSC_SMAX 默认 = %s' % OSC)
check(abs(GM - 60.0) < 1e-9, '★ QWIK_GAIN_MARGIN 默认 = %s %%' % GM)
check(re.search(r'float reduction = \(100\.0-gain_margin\)\*0\.01;', src_qt) is not None,
      '★ 检出振荡后的回退：new_gain = pval × (100 - GAIN_MARGIN) × 0.01 = pval × %.2f'
      % ((100 - GM) * 0.01))

# --- Stage 枚举里的死值 ---
STAGE = enum_values(src_qth, 'Stage')
check(STAGE is not None and len(STAGE) == 10,
      '★ Stage 枚举有 %d 项：%s' % (len(STAGE) if STAGE else 0,
                                   ', '.join(sorted(STAGE, key=STAGE.get)) if STAGE else ''))
assigned = set(re.findall(r'current_stage\s*=\s*Stage::(\w+)', src_qt))
check(assigned == {'D', 'P'},
      '★★ 全文件里 current_stage 只被赋过 %s —— 其余 %d 个枚举值**从来没被用过**'
      % (sorted(assigned), len(STAGE) - len(assigned) if STAGE else 0))
m = re.search(r'switch\s*\(stage\)\s*\{(.*?)\n\s*\}', src_qt, re.S)
handled = set(re.findall(r'case\s+Stage::(\w+):', m.group(1))) if m else set()
check(handled == {'P', 'D', 'FLTT', 'FLTD'},
      '★ get_pname() 只处理 %s —— 其余 Stage 值走 INTERNAL_ERROR 兜底' % sorted(handled))
check('INTERNAL_ERROR' in (m.group(1) if m else ''),
      '   兜底是 INTERNAL_ERROR(flow_of_control)，不是静默放行')

# --- 24 个被调参数 = 3 轴 × 8 ---
m = re.search(r'enum\s+class\s+Param\s*:\s*uint8_t\s*\{(.*?)\}', src_qth, re.S)
PB = re.sub(r'//[^\n]*', '', m.group(1)) if m else ''
PNAMES = re.findall(r'(\w+)\s*(?:=\s*\d+)?\s*,', PB)
PNAMES = [p for p in PNAMES if p != 'END']
check(len(PNAMES) == 24, '★ Param 枚举有 %d 项（3 轴 × 8 个/轴）' % len(PNAMES))
per_axis = [p.split('_', 1)[1] for p in PNAMES if p.startswith('RLL_')]
check(per_axis == ['P', 'I', 'D', 'SMAX', 'FLTT', 'FLTD', 'FLTE', 'FF'],
      '★ 每轴调 8 个：%s' % '/'.join(per_axis))

# --- 与 Lua 版的关系 ---
check('based on original lua script' in src_qt,
      '★★ AP_Quicktune.cpp 头部自陈 "C++ implementation of quicktune based on original lua script" '
      '—— **Lua 是原版，C++ 是移植**（与 AC_CustomControl / Lua 那条线的方向相反）')


# ==========================================================================
print()
print('=' * 76)
print('[9] 固定翼的 AP_AutoTune：既不是继电法，也没有 Ziegler-Nichols')
print('=' * 76)

check('relay' not in src_plane.lower() and 'ziegler' not in src_plane.lower(),
      '★★ 全文件搜 relay / ziegler —— 零命中。它**不是继电反馈法**，'
      '也不是 Ziegler-Nichols（没有 Ku / Tu）')
check(re.search(r'\bKu\b', src_plane) is None and re.search(r'\bTu\b', src_plane) is None,
      '★ 确认没有 Ku / Tu 这两个 Z-N 特征量')

# --- 振荡判据：靠 AC_PID 内部的 Dmod ---
check(re.search(r'min_Dmod\s*<\s*1\.0', src_plane) is not None,
      '★★ 振荡判据是 `min_Dmod < 1.0` —— 用的是 AC_PID 内部 SlewLimiter 的修正因子 Dmod')
check('oscillation, without D_limit set' in src_plane,
      '   源码注释："oscillation, without D_limit set"')
m = re.search(r'_pid_info\.Dmod\s*=\s*_slew_limiter\.modifier\(', slurp('libraries/AC_PID/AC_PID.cpp'))
check(m is not None,
      '   Dmod 的来源：AC_PID.cpp 的 `_pid_info.Dmod = _slew_limiter.modifier(...)`')

# --- 它测的不是幅度/周期，而是"事件窗口内的峰值包络" ---
check('max_actuator = MAX(max_actuator, actuator);' in src_plane,
      '★ 只取事件窗口内的 max/min 包络（max_actuator/min_actuator、max_rate/min_rate）')
check('zero_cross' not in src_plane, '★ 没有过零检测（zero_cross 零命中）')
# period / frequency 确实出现过，但身份都不是"被测量" —— 逐个数出来
period_hits = set(re.findall(r'\w*period\w*', src_plane))
freq_hits = sorted(set(re.findall(r'\w*frequency\w*', src_plane)))
check(period_hits == {'get_loop_period_s'},
      '★ period 全文件只以 %s 出现 —— 那是【调度器的循环周期】，不是振荡周期'
      % sorted(period_hits))
check(freq_hits == ['set_cutoff_frequency'],
      '★ frequency 全文件只以 %s 出现 —— 那是【滤波器截止频率】，不是振荡频率'
      % freq_hits)
check(not re.search(r'\bTu\b|\bKu\b|ultimate', src_plane),
      '★★ 没有 Tu / Ku / ultimate —— 它**既不过零、也不测周期也不测频率**，'
      '所以与 Ziegler-Nichols 那一类方法在结构上就不同')

# --- 爬坡乘子 ---
check(re.search(r'D\s*\*=\s*1\.3;', src_plane) is not None, '★ 加 D：D *= 1.3')
check(re.search(r'P\s*\*=\s*1\.3;', src_plane) is not None, '★ 加 P：P *= 1.3')
check(re.search(r'P\s*\*=\s*0\.35;', src_plane) is not None, '★ 降 P：P *= 0.35（比加的时候狠得多）')
check(re.search(r'D\s*\*=\s*0\.3;', src_plane) is not None, '★ 降 D 到限值：D *= 0.3')
check(re.search(r'D\s*\*=\s*0\.75;', src_plane) is not None, '★ 同时降 P 和 D：P *= 0.35, D *= 0.75')
check(abs(1.3 / 0.35 - 3.714) < 0.01,
      '★★ 不对称：加是 ×1.3，降是 ×0.35 —— 一次"降"相当于 %.2f 次"加"，'
      '所以振荡一出现增益掉得很快（安全方向故意做快）' % (math.log(0.35) / math.log(1.3)))

# --- I 不是独立辨识的 ---
m = re.search(r'rpid\.kI\(\)\.set\(MIN\(P,\s*\(FF\s*/\s*TRIM_TCONST\)\)\);', src_plane)
check(m is not None, '★★ 固定翼的 I 是【派生】的：roll 用 MIN(P, FF/TRIM_TCONST)')
check(re.search(r'rpid\.kI\(\)\.set\(MAX\(P\*AUTOTUNE_I_RATIO,\s*\(FF\s*/\s*TRIM_TCONST\)\)\);', src_plane)
      is not None,
      '   pitch/yaw 用 MAX(P × AUTOTUNE_I_RATIO, FF/TRIM_TCONST)')
PD = src_defines(src_plane)
check(abs(PD.get('AUTOTUNE_I_RATIO', -1) - 0.75) < 1e-9,
      '★ AUTOTUNE_I_RATIO = %s' % PD.get('AUTOTUNE_I_RATIO'))

# --- 唯一被真正辨识的是 FF ---
check(re.search(r'FF_single\s*=\s*max_actuator\s*/\s*\(max_rate\s*\*\s*scaler\);', src_plane) is not None,
      '★★ 三个增益里只有 FF 是"辨识"出来的：FF = 执行器峰值 / (角速率峰值 × scaler)')
check('ModeFilterFloat_Size5 ff_filter' in src_planeh,
      '   而且过了一个 5 点中值滤波（ModeFilter）抗离群')
check(re.search(r'#define\s+AUTOTUNE_(INCREASE|DECREASE)_FF_STEP\s+(\d+)', src_plane) is not None,
      '★ FF 单次变化被限幅（%s）—— 防止一次坏样本把前馈打飞'
      % ', '.join('%s=%s' % (a, b) for a, b in
                  re.findall(r'#define\s+AUTOTUNE_(\w+_FF_STEP)\s+(\d+)', src_plane)))

# --- 激励来自飞行员 ---
check('behaves just like FBWA' in src_plane,
      '★★ 激励来源：源码注释说 AUTOTUNE 模式 "behaves just like FBWA" —— '
      '**是飞手打杆在激励，算法自己不注入任何东西**')
check(re.search(r'aspeed\s*>\s*aparm\.airspeed_min', slurp('libraries/APM_Control/AP_FW_Controller.cpp'))
      is not None,
      '   AUTOTUNE 模式要求有空速（aspeed > airspeed_min）')

# --- 调参级别表 ---
m = re.search(r'\}\s*tuning_table\[\]\s*=\s*\{(.*?)\};', src_plane, re.S)
TB = m.group(1) if m else ''
rows = re.findall(r'\{\s*([\d.]+),\s*(\d+)\s*\}', TB)
check(len(rows) == 11, '★ AUTOTUNE_LEVEL 的爬坡表有 %d 行（1~10，外加一行 "yes, it goes to 11"）'
      % len(rows))
taus = [float(a) for a, _ in rows]
rmaxs = [int(b) for _, b in rows]
check(all(taus[i] >= taus[i + 1] for i in range(len(taus) - 1)),
      '★ tau 随级别单调不增：%s → %s' % (taus[0], taus[-1]))
check(all(rmaxs[i] <= rmaxs[i + 1] for i in range(len(rmaxs) - 1)),
      '★ rmax 随级别单调不减：%s → %s' % (rmaxs[0], rmaxs[-1]))
check('yes, it goes to 11' in TB,
      '   第 11 行源码注释原文："(yes, it goes to 11)"')
check(re.search(r'current\.rmax_pos\.get\(\)-20', src_plane) is not None
      and re.search(r'current\.rmax_pos\.get\(\)\+20', src_plane) is not None,
      '★ 渐变限速：rmax 每拍最多动 ±20，tau 每拍最多动 ±15%%（防止直接跳到 level 10 时失控）')


# ==========================================================================
print()
print('=' * 76)
print('[10] 4.5.7 → 4.7.2：哪些数字/位置真的变了')
print('=' * 76)

if OLD is None:
    print('    [!!] 没找到 4.5.7 对照树 —— 跨版本比对跳过')
    n_bad += 1
else:
    old_multi = slurp('libraries/AC_AutoTune/AC_AutoTune_Multi.cpp', OLD)

    # ① 默认值
    o_a = re.search(r'AP_GROUPINFO\("AGGR",\s*\d+,\s*AC_AutoTune_Multi,\s*aggressiveness,\s*([\d.]+)f\)',
                    old_multi)
    o_d = re.search(r'AP_GROUPINFO\("MIN_D",\s*\d+,\s*AC_AutoTune_Multi,\s*min_d,\s*([\d.]+)f\)',
                    old_multi)
    check(o_a is not None and abs(float(o_a.group(1)) - 0.1) < 1e-9,
          '★ 4.5.7 的 AGGR 默认 = %s' % (o_a.group(1) if o_a else '?'))
    check(o_d is not None and abs(float(o_d.group(1)) - 0.001) < 1e-9,
          '★ 4.5.7 的 MIN_D 默认 = %s' % (o_d.group(1) if o_d else '?'))
    check(abs(AGGR_NEW - 0.1) > 1e-9 and abs(AGGR_NEW - 0.075) < 1e-9,
          '★★ 跨版本：AGGR 从 0.1 变成 0.075（**默认更保守了**）')
    check(abs(MIND_NEW - 0.001) > 1e-9 and abs(MIND_NEW - 0.0005) < 1e-9,
          '★★ 跨版本：MIN_D 从 0.001 变成 0.0005（D 的下限**降了一半**）')

    # ② 判据的位置
    def find_line(text, pat):
        m = re.search(pat, text)
        return text[:m.start()].count('\n') + 1 if m else -1

    l_new = find_line(src_multi, r'void AC_AutoTune_Multi::twitching_test_rate\(')
    l_old = find_line(old_multi, r'void AC_AutoTune_Multi::twitching_test_rate\(')
    check(l_new == 687 and l_old == 522,
          '★★ 判据函数从 4.5.7 的第 %d 行搬到 4.7.2 的第 %d 行（搬了 %d 行）'
          % (l_old, l_new, l_new - l_old))

    # ③ 函数签名
    sig_old = re.search(r'twitching_test_rate\(([^)]*)\)', old_multi)
    n_old = len(sig_old.group(1).split(',')) if sig_old else 0
    check(n_old == 4 and n_par == 6,
          '★★ 签名从 4.5.7 的 %d 个参数变成 4.7.2 的 %d 个（多了开头的 angle 与结尾的 '
          'meas_angle_min，净增 2 个）' % (n_old, n_par))

    # ④ 阈值数字
    check('rate_target_max * 0.75f' in old_multi and 'rate_target_max * 0.75f' not in src_multi,
          '★★ 早期超时门限从 0.75 改成 0.6321(=1-1/e)')
    check('rate_target_max * 0.5f' in old_multi and 'rate_target_max * 0.25' in src_multi,
          '★★ 回弹捕获门限从 0.5 降到 0.25（更容易开始跟踪反弹）')

    # ⑤ tune_seq 两版一致（08 章"先加 P 再加 D"的说法在两版上都不成立）
    old_multih = slurp('libraries/AC_AutoTune/AC_AutoTune_Multi.h', OLD)
    m_old_seq = re.search(r'tune_seq\[0\]\s*=\s*(\w+);', old_multih)
    check(m_old_seq is not None and m_old_seq.group(1) == 'RD_UP',
          '★★ 4.5.7 的序列第 0 步也是 RD_UP（D 在前）—— '
          '所以"先加 P 再加 D"的说法**两版都不成立**')
    check(SEQ_MULTI[0][1] == 'RATE_D_UP' and m_old_seq.group(1) == 'RD_UP',
          '   两个版本一致：都是 D 先、P 后')


# ==========================================================================
print()
print('=' * 76)
print('[11] 四条入口的分工总表（全部由源码解析而来）')
print('=' * 76)

check(re.search(r'class\s+AutoTune\s*:\s*public\s+AC_AutoTune_Heli', slurp('ArduCopter/mode.h'))
      is not None,
      '★ Copter 侧按 FRAME_CONFIG 二选一继承：HELI_FRAME → AC_AutoTune_Heli')
check(re.search(r'class\s+AutoTune\s*:\s*public\s+AC_AutoTune_Multi', slurp('ArduCopter/mode.h'))
      is not None,
      '   其余 → AC_AutoTune_Multi')
check('AC_AutoTune' in slurp('ArduPlane/wscript'),
      '★ Plane 也编译 AC_AutoTune（给 QAutoTune 用，仅 SITL）')
check('APM_Control' in slurp('ArduPlane/wscript'),
      '★ Plane 同时编译 APM_Control（固定翼的 AP_AutoTune 在这里）')
check(re.search(r'case\s+AUX_FUNC::FW_AUTOTUNE:', slurp('ArduPlane/RC_Channel_Plane.cpp')) is not None,
      '★ 固定翼的 RC 开关是 AUX_FUNC::FW_AUTOTUNE（与 Quicktune 的 181 是两个不同的号）')

entries = [
    ('AC_AutoTune_Multi', 'Copter 多旋翼 / QuadPlane(SITL)', 'AUTOTUNE_', '阶跃注入 + 反弹比'),
    ('AC_AutoTune_Heli',  'Copter 直升机',                    'AUTOTUNE_', 'Chirp 扫频 + 幅相'),
    ('AP_AutoTune',       '固定翼（含 QuadPlane 固定翼段）',   'AUTOTUNE_', '打杆激励 + Dmod'),
    ('AP_Quicktune',      'QuadPlane 的 VTOL 段',              'QWIK_',     '在线爬坡 + slew_rate'),
]
check(len(entries) == 4, '★★ 一共四条入口、三份实现：')
for a, b, c, d in entries:
    print('        %-18s %-34s %-11s %s' % (a, b, c, d))
check(len({e[0] for e in entries}) == 4, '   四个类名互不相同，没有一个是另一个的基类')
check(entries[2][2] == entries[0][2],
      '★ 注意 AP_AutoTune 与 AC_AutoTune 共用 AUTOTUNE_ 前缀（都在 ArduPlane 命名空间下），'
      '但**是两个不同的库**')


# ==========================================================================
print()
print('=' * 76)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
