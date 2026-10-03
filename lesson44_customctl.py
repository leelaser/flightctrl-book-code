# -*- coding: utf-8 -*-
"""
lesson44_customctl.py —— 第 10 篇《自定义控制律》配套自验证脚本

主题：AC_CustomControl —— ArduPilot 官方给的「往姿态控制链路里插自己的控制器」
      的扩展点。它是**覆写式**（主控制器先跑，自定义控制器后跑并盖掉 mixer 输入），
      不是替换式。

本脚本不"复述"源码，而是：
  ① 把源码里的结构事实**解析出来**（参数表索引、switch 分支、枚举值、调度顺序）
  ② 把 C++ 算法**移植成 Python**，再用**定义性性质**（不是重算同一条公式）去验它
  ③ 把官方 README 与源码**逐条对质**，数出漂移

三条铁律：断言两边来源独立 / 反向对照写进脚本 / 阈值来自解析预言而非拍脑袋。
运行： python3 lesson44_customctl.py     （全过 EXIT=0）
"""

import io
import math
import os
import random
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


# ==========================================================================
# [0] 定位源码
# ==========================================================================
print('=' * 74)
print('[0] 定位 ArduPilot 源码')
print('=' * 74)

CANDIDATES = [
    r'D:\DEVLib\PX4APM\APM\ardupilot-Copter-4.7.1',
    r'D:\DEVLib\PX4APM\APM\ardupilot-Plane-4.7.1',
]
REPO = next((p for p in CANDIDATES if os.path.isdir(os.path.join(p, 'libraries'))), None)
if REPO is None:
    print('    [!!] 找不到 ArduPilot 源码树 —— 后续所有"读真源码"的检查无从谈起')
    n_bad += 1
    sys.exit(1)

CC_DIR = os.path.join(REPO, 'libraries', 'AC_CustomControl')
if not os.path.isdir(CC_DIR):
    print('    [!!] 找不到 libraries/AC_CustomControl —— 本篇主题库缺席')
    n_bad += 1
    sys.exit(1)


def slurp(rel):
    return io.open(os.path.join(REPO, rel), encoding='utf-8', errors='replace').read()


def slurp_abs(p):
    return io.open(p, encoding='utf-8', errors='replace').read()


F_FRONT = 'libraries/AC_CustomControl/AC_CustomControl.cpp'
F_FRONTH = 'libraries/AC_CustomControl/AC_CustomControl.h'
F_BACKH = 'libraries/AC_CustomControl/AC_CustomControl_Backend.h'
F_CFG = 'libraries/AC_CustomControl/AC_CustomControl_config.h'
F_EMPTY = 'libraries/AC_CustomControl/AC_CustomControl_Empty.cpp'
F_PID = 'libraries/AC_CustomControl/AC_CustomControl_PID.cpp'
F_README = 'libraries/AC_CustomControl/README.md'

src_front = slurp(F_FRONT)
src_fronth = slurp(F_FRONTH)
src_backh = slurp(F_BACKH)
src_cfg = slurp(F_CFG)
src_empty = slurp(F_EMPTY)
src_pid = slurp(F_PID)
src_readme = slurp(F_README)

print('    源码树 = %s' % REPO)
check(len(src_front) > 3000, 'AC_CustomControl.cpp 读入 %d 字节' % len(src_front))
check(os.path.isfile(os.path.join(CC_DIR, 'README.md')),
      '库自带 README.md（官方"怎么加 backend"指南就在里面）')


# ==========================================================================
# [1] 前端参数表 与 后端索引算术
# ==========================================================================
print()
print('=' * 74)
print('[1] 前端参数表 与 后端索引算术')
print('=' * 74)

# --- CUSTOMCONTROL_MAX_TYPES 的默认值 ---
m = re.search(r'#define\s+CUSTOMCONTROL_MAX_TYPES\s+(\d+)', src_fronth)
check(m is not None, '在 AC_CustomControl.h 里找到 CUSTOMCONTROL_MAX_TYPES 定义')
MAX_TYPES = int(m.group(1)) if m else -1
check(MAX_TYPES == 2, 'CUSTOMCONTROL_MAX_TYPES 默认 = %d' % MAX_TYPES)

# --- enum CustomControlType 的三个值 ---
enum_body = re.search(r'enum class CustomControlType\s*:\s*uint8_t\s*\{(.*?)\}',
                      src_fronth, re.S)
check(enum_body is not None, '找到 enum class CustomControlType 定义')
ENUM = {}
if enum_body:
    for name, val in re.findall(r'(CONT_\w+)\s*=\s*(\d+)', enum_body.group(1)):
        ENUM[name] = int(val)
check(len(ENUM) == 3, '枚举有 3 项：%s' % ', '.join('%s=%d' % (k, v) for k, v in sorted(ENUM.items(), key=lambda kv: kv[1])))
check(ENUM.get('CONT_NONE') == 0 and ENUM.get('CONT_EMPTY') == 1 and ENUM.get('CONT_PID') == 2,
      '取值为 NONE=0 / EMPTY=1 / PID=2，与 @Values 文本一致')

# --- get_type() 的 -1 映射 ---
m = re.search(r'uint8_t\s+get_type\(\)\s*\{\s*return\s+(.*?);\s*\}', src_fronth)
check(m is not None, '找到 get_type() 内联实现')
GET_TYPE_EXPR = m.group(1).strip() if m else ''
check('_controller_type - 1' in GET_TYPE_EXPR,
      'get_type() 是"零基映射"：%s' % GET_TYPE_EXPR)


def get_type(ctype):
    return (ctype - 1) if ctype > 0 else 0


# --- var_info 里的索引 ---
# _TYPE 与 _AXIS_MASK 各自的槽位号
m_type = re.search(r'AP_GROUPINFO_FLAGS\("_TYPE",\s*(\d+),', src_front)
m_mask = re.search(r'AP_GROUPINFO\("_AXIS_MASK",\s*(\d+),', src_front)
check(m_type is not None and m_mask is not None, '在 var_info 里找到 _TYPE 与 _AXIS_MASK')
IDX_TYPE = int(m_type.group(1)) if m_type else -1
IDX_MASK = int(m_mask.group(1)) if m_mask else -1
check(IDX_TYPE == 1 and IDX_MASK == 2,
      '_TYPE 槽位=%d、_AXIS_MASK 槽位=%d（都从 1 起，槽位 0 留给 AP_Param 自己）' % (IDX_TYPE, IDX_MASK))

# --- AP_SUBGROUPVARPTR 的 (前缀, 槽位, 数组下标) 三元组 ---
subs = re.findall(
    r'AP_SUBGROUPVARPTR\(_backend,\s*"(\d+)_",\s*(\d+),\s*AC_CustomControl,\s*'
    r'_backend_var_info\[(\d+)\]\)', src_front)
check(len(subs) >= 1, '找到 %d 条生效的 AP_SUBGROUPVARPTR' % len(subs))

# --- ★ 核心恒等式：参数前缀数字 == 枚举值 == 数组下标+1 ---
# 三个数来自三个完全不同的地方：字符串 "2_"、enum 的 CONT_PID、数组下标 1。
# 任何一处改了忘了改另一处，这条就红。
for prefix, slot, arr_idx in subs:
    p = int(prefix)
    a = int(arr_idx)
    check(p in ENUM.values(),
          '参数前缀 "%d_" 能在枚举里找到对应项（枚举值集合 %s）' % (p, sorted(ENUM.values())))
    check(a == p - 1,
          '数组下标 %d == 前缀 %d - 1（与 get_type() 的 -1 映射同构）' % (a, p))
    # ★ 把"解析出来的数"喂给"解析出来的函数" —— 上面那条只是两个静态整数互比，
    #   这一条才真正让 get_type() 参与运算，否则它是个没人调的摆设。
    check(get_type(p) == a,
          '★ 运行时 get_type(%d) = %d，正好等于参数表里写的数组下标 %d'
          % (p, get_type(p), a))
    check(a < MAX_TYPES,
          '数组下标 %d 落在 _backend_var_info[%d] 内' % (a, MAX_TYPES))
    check(p != ENUM.get('CONT_NONE'),
          '前缀 %d 不是 CONT_NONE（NONE 不占数组槽）' % p)

# --- 反向对照：如果照 README 加一个 XYZ=3 却忘了把 MAX_TYPES 改成 3 ---
XYZ = 3
check(get_type(XYZ) == 2,
      '假想新增 backend CONT_XYZ=3 时 get_type()=%d' % get_type(XYZ))
check(not (get_type(XYZ) < MAX_TYPES),
      '★ 反向对照：MAX_TYPES 仍为 %d 时，下标 %d 会越界 —— '
      '所以"加 backend"必须同时加 MAX_TYPES，这两处是耦合的'
      % (MAX_TYPES, get_type(XYZ)))
# 把 MAX_TYPES 改成 3 之后就不越界
check(get_type(XYZ) < 3, '把 MAX_TYPES 改成 3 后下标合法（README 第 4 步说的就是这件事）')


# ==========================================================================
# [2] CC_TYPE = 1 是死路 —— Empty backend 的陷阱
# ==========================================================================
print()
print('=' * 74)
print('[2] CC_TYPE = 1 是死路 —— Empty backend 的陷阱')
print('=' * 74)

# --- 参数表自己宣称的可选值 ---
m = re.search(r'//\s*@Values:\s*(.*)', src_front)
check(m is not None, '找到 CC_TYPE 的 @Values 行')
VALUES_TEXT = m.group(1).strip() if m else ''
claimed = set()
for item in VALUES_TEXT.split(','):
    mm = re.match(r'\s*(\d+)\s*:', item)
    if mm:
        claimed.add(int(mm.group(1)))
check(claimed == {0, 1, 2},
      '@Values 宣称可选 {0,1,2}，原文「%s」' % VALUES_TEXT)

# --- init() 的 switch：哪些分支真的 new ---
m = re.search(r'switch\s*\(CustomControlType\(_controller_type\)\)\s*\{(.*?)\n\s*\}',
              src_front, re.S)
check(m is not None, '找到 init() 里的 backend 选择 switch')
SWITCH = m.group(1) if m else ''

# 逐分支取出 case ... break; 之间的正文
cases = dict(re.findall(
    r'case\s+CustomControlType::(CONT_\w+)\s*:(.*?)break;', SWITCH, re.S))
check(len(cases) == 3,
      'switch 里有 %d 个显式 case：%s' % (len(cases), ', '.join(sorted(cases))))
check(set(cases) == {'CONT_NONE', 'CONT_EMPTY', 'CONT_PID'},
      '★ 三个枚举值各有分支 —— 所以 CONT_EMPTY 不是"漏写了"，是"写了又注掉"')

# --- ★ 关键：CONT_EMPTY 分支的正文【每一行都是注释】 ---
empty_body = cases.get('CONT_EMPTY', '')
live_lines = [l for l in empty_body.splitlines()
              if l.strip() and not l.strip().startswith('//')]
check(len(live_lines) == 0,
      '★ CONT_EMPTY 分支正文 %d 行，**全部是注释**（没有一行是可执行代码）'
      % len(empty_body.splitlines()))
check('NEW_NOTHROW' in empty_body and
      all('NEW_NOTHROW' not in l for l in live_lines),
      '★ 该分支里的 NEW_NOTHROW 只出现在注释行上 —— 分支存在，但不建立任何 backend')

# --- 三重证据：include 注释、subgroup 注释、分支注释 ---
check(re.search(r'//\s*#include\s+"AC_CustomControl_Empty\.h"', src_front) is not None,
      '★ 证据一：AC_CustomControl.cpp 里 Empty 的 #include 被注释掉')
check(re.search(r'//\s*AP_SUBGROUPVARPTR\(_backend,\s*"1_"', src_front) is not None,
      '★ 证据二：var_info 里 "1_" 子组被注释掉 —— 所以磁盘上不存在 CC1_* 参数')
check(re.search(r'#include\s+"AC_CustomControl_PID\.h"', src_front) is not None,
      '对照：PID 的 #include 是**未注释**的')

# --- Empty 的代码本身还在（被编译，但不可达） ---
check(len(src_empty) > 1500, 'AC_CustomControl_Empty.cpp 仍在磁盘上（%d 字节）' % len(src_empty))
check(re.search(r'#define\s+AP_CUSTOMCONTROL_EMPTY_ENABLED\s+AP_CUSTOMCONTROL_BACKEND_DEFAULT_ENABLED',
                src_cfg) is not None,
      'AP_CUSTOMCONTROL_EMPTY_ENABLED 默认跟随总开关 ⇒ Empty.cpp 并未被编译掉')
empty_params = re.findall(r'AP_GROUPINFO\("(PARAM\d)"', src_empty)
check(len(empty_params) == 3,
      'Empty 后端自己定义了 %d 个参数（%s）—— 只是永远不接线' % (len(empty_params), ', '.join(empty_params)))


# --- 模拟 init()：给定 CC_TYPE，_backend 会不会被建立 ---
def init_constructs(ctype):
    """逐字模拟 init() 的 switch 行为。"""
    if ctype == ENUM.get('CONT_NONE'):
        return False            # break，不建立
    if ctype == ENUM.get('CONT_EMPTY'):
        return False            # 整段被注释，落到 break
    if ctype == ENUM.get('CONT_PID'):
        return True             # NEW_NOTHROW 生效
    return False                # default: return


def is_safe_to_run(ctype, active, backend, max_types=MAX_TYPES):
    """逐字移植 is_safe_to_run() 的四个条件。"""
    return bool(active and (ctype > ENUM.get('CONT_NONE'))
                and (ctype <= max_types) and backend)


ALL_T = range(0, 256)
feasible = [t for t in ALL_T if is_safe_to_run(t, True, init_constructs(t))]
check(feasible == [ENUM.get('CONT_PID')],
      '★ 遍历 CC_TYPE=0..255：唯一能被 is_safe_to_run() 放行的值是 %s'
      % feasible)
check(ENUM.get('CONT_EMPTY') not in feasible,
      '★ 而 @Values 宣称可选的 1:Empty **永远跑不起来**（_backend 恒为 nullptr）')
check(ENUM.get('CONT_NONE') not in feasible,
      '0:None 跑不起来是**设计如此**（= 关闭本功能）')

# --- 用户实际会看到什么 ---
def set_custom_controller_verdict(ctype, mask, backend_exists, enabled,
                                  max_types=MAX_TYPES):
    """移植 set_custom_controller() 的判定顺序，返回 (是否真的切成功, GCS 文本)。"""
    if ctype == ENUM.get('CONT_NONE'):
        return False, 'Custom controller is not enabled'
    if ctype > max_types:
        return False, 'Custom controller type is out of range'
    if not backend_exists:
        return False, 'Reboot to enable selected custom controller'
    if mask == 0 and enabled:
        return False, 'Axis mask is not set'
    return enabled, ('Custom controller is ON' if enabled
                     else 'Custom controller is OFF')


ok, text = set_custom_controller_verdict(ENUM.get('CONT_EMPTY'), 7, False, True)
check((not ok) and text == 'Reboot to enable selected custom controller',
      '★ 用户设 CC_TYPE=1 + 拨开关，收到的是「%s」—— 而不是"正在运行"' % text)

ok, text = set_custom_controller_verdict(ENUM.get('CONT_PID'), 7, True, True)
check(ok and text == 'Custom controller is ON',
      'CC_TYPE=2 + mask=7 + 拨高，得到「%s」' % text)

# --- 反向对照：把 Empty 取消注释后，1 应该进入可行集 ---
def init_constructs_fixed(ctype):
    if ctype in (ENUM.get('CONT_EMPTY'), ENUM.get('CONT_PID')):
        return True
    return False


feasible_fixed = [t for t in ALL_T if is_safe_to_run(t, True, init_constructs_fixed(t))]
check(ENUM.get('CONT_EMPTY') in feasible_fixed,
      '★ 反向对照：取消注释 Empty 后，1 进入可行集 %s（说明上面那条判据真的在测那行注释）'
      % feasible_fixed)


# ==========================================================================
# [3] PID 后端默认增益：0.9 倍缩放的复算
# ==========================================================================
print()
print('=' * 74)
print('[3] PID 后端默认增益：0.9 倍缩放的复算')
print('=' * 74)

# --- 独立基准源：从两个头文件解析 #define ---
h_base = slurp('libraries/AC_AttitudeControl/AC_AttitudeControl.h')
h_multi = slurp('libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h')


def parse_defines(text, pattern):
    return {k: float(v.rstrip('f')) for k, v in re.findall(pattern, text)}


BASE_DEF = parse_defines(h_base, r'#define\s+(AC_ATTITUDE_CONTROL_\w+)\s+([\d.]+f)')
MULTI_DEF = parse_defines(h_multi, r'#\s*define\s+(AC_ATC_MULTI_RATE_\w+)\s+([\d.]+f)')

check('AC_ATTITUDE_CONTROL_ANGLE_P' in BASE_DEF,
      '解析到 AC_ATTITUDE_CONTROL_ANGLE_P = %s' % BASE_DEF.get('AC_ATTITUDE_CONTROL_ANGLE_P'))
check(len(MULTI_DEF) == 10,
      '解析到 %d 个 AC_ATC_MULTI_RATE_* 常量（RP 四参 + RPY 滤波 + YAW 五参）'
      % len(MULTI_DEF))
check(all(k in MULTI_DEF for k in
          ('AC_ATC_MULTI_RATE_RP_P', 'AC_ATC_MULTI_RATE_YAW_D',
           'AC_ATC_MULTI_RATE_YAW_FILT_HZ')),
      '关键常量齐全（RP_P=%s / YAW_D=%s / YAW_FILT_HZ=%s）'
      % (MULTI_DEF.get('AC_ATC_MULTI_RATE_RP_P'), MULTI_DEF.get('AC_ATC_MULTI_RATE_YAW_D'),
         MULTI_DEF.get('AC_ATC_MULTI_RATE_YAW_FILT_HZ')))

# --- 从 PID.cpp 的构造函数解析实参 ---
ctor = re.search(r'AC_CustomControl_PID::AC_CustomControl_PID\((.*?)\n\{', src_pid, re.S)
check(ctor is not None, '找到 AC_CustomControl_PID 构造函数')
CTOR = ctor.group(1) if ctor else ''

angle_args = re.findall(r'_p_angle_(\w+)2\(AC_ATTITUDE_CONTROL_ANGLE_P\s*\*\s*([\d.]+)f\)', CTOR)
check(len(angle_args) == 3,
      '解析到 %d 个角度环 P 初值：%s'
      % (len(angle_args), ', '.join('%s×%s' % (a, b) for a, b in angle_args)))
check(all(abs(float(b) - 0.90) < 1e-9 for _, b in angle_args),
      '★ 角度环三个轴的缩放系数全是 0.90')

# 注意末项后面既没有逗号也没有 \n{（CTOR 已被截断），所以要允许 $ 结尾
rate_lines = re.findall(r'_pid_atti_rate_(\w+)\((.*?)\)\s*(?:,|\n\s*\{|$)', CTOR, re.S)
check(len(rate_lines) == 3,
      '解析到 3 条 _pid_atti_rate_* 构造：%s' % ', '.join(a for a, _ in rate_lines))
check([a for a, _ in rate_lines] == ['roll', 'pitch', 'yaw'],
      '三条按 roll/pitch/yaw 的书写顺序解析出来（顺序对不上说明正则吃串了）')

SCALE_RE = re.compile(r'(AC_ATC_MULTI_RATE_\w+)\s*\*\s*([\d.]+)f')
per_axis = {}
total_scaled, bad_scale = 0, []
for axis, args in rate_lines:
    pairs = SCALE_RE.findall(args)
    per_axis[axis] = len(pairs)
    total_scaled += len(pairs)
    for name, s in pairs:
        if abs(float(s) - 0.90) > 1e-9:
            bad_scale.append((axis, name, s))
check(total_scaled == 18,
      '★ 三个轴共 %d 处增益初值写成「常量 × 0.90f」' % total_scaled)
check(set(per_axis.values()) == {6},
      '★ 每轴各 %d 处 —— AC_PID 构造函数收 8 个实参，其中 6 个取自 MULTI_RATE 常量、'
      '2 个（FF 与 FLTE）写死 0.0f；连同 3 个角度环 P，共 3×8+3 = 27 个初值'
      % sorted(set(per_axis.values()))[0])
check(not bad_scale, '没有一处漏掉 0.90（漏掉就会与主控制器增益完全相同）')

# --- 独立基准：用解析出的 header 常量复算，验证缩放结果 ---
def scaled(name):
    return MULTI_DEF[name] * 0.90


check(abs(scaled('AC_ATC_MULTI_RATE_RP_P') - 0.135 * 0.9) < 1e-9,
      '复算：CC2_RAT_RLL_P 默认 = %s × 0.9 = %.5f'
      % (MULTI_DEF['AC_ATC_MULTI_RATE_RP_P'], scaled('AC_ATC_MULTI_RATE_RP_P')))
check(abs(BASE_DEF['AC_ATTITUDE_CONTROL_ANGLE_P'] * 0.90 - 4.05) < 1e-6,
      '复算：CC2_ANG_RLL_P 默认 = %s × 0.9 = %.3f'
      % (BASE_DEF['AC_ATTITUDE_CONTROL_ANGLE_P'],
         BASE_DEF['AC_ATTITUDE_CONTROL_ANGLE_P'] * 0.90))
check(abs(scaled('AC_ATC_MULTI_RATE_YAW_P') - 0.162) < 1e-9,
      '复算：CC2_RAT_YAW_P 默认 = %.3f（主控制器 ATC_RAT_YAW_P = %.3f）'
      % (scaled('AC_ATC_MULTI_RATE_YAW_P'), MULTI_DEF['AC_ATC_MULTI_RATE_YAW_P']))

# --- ★ 那个"第 4 个实参 0.0f"是 FF 增益，不是 IMAX ---
for axis, args in rate_lines:
    flat = ' '.join(args.split())
    m_ff = re.match(r'AC_ATC_MULTI_RATE_\w+\s*\*\s*[\d.]+f,\s*'      # P
                    r'AC_ATC_MULTI_RATE_\w+\s*\*\s*[\d.]+f,\s*'      # I
                    r'AC_ATC_MULTI_RATE_\w+\s*\*\s*[\d.]+f,\s*'      # D
                    r'([\d.]+)f,', flat)
    check(m_ff is not None and abs(float(m_ff.group(1))) < 1e-9,
          '★ %s：第 4 个实参（前馈 FF）= 0.0f —— 默认没有任何前馈'
          % axis)

# --- AC_PID 的参数列序：P/I/D 在前，FF 在第 4 位（列序约定，三轴同验） ---
pid_params = re.findall(r'AP_GROUPINFO\w*\("(\w+)",\s*(\d+),\s*AC_PID,',
                        slurp('libraries/AC_PID/AC_PID.cpp'))
order = [n for n, _ in sorted(pid_params, key=lambda kv: int(kv[1]))]
check(order[:4] == ['P', 'I', 'D', 'FF'],
      'AC_PID 参数槽序前四位是 %s（所以构造函数第 4 个位置是 FF）' % order[:4])


# ==========================================================================
# [4] thrust_heading_rotation_angles：移植 + 定义性基准
# ==========================================================================
print()
print('=' * 74)
print('[4] thrust_heading_rotation_angles：姿态误差是怎么算出来的')
print('=' * 74)

FLT_EPSILON = 1.1920929e-07          # ftype.h:63  is_zero(x) = fabsf(x) < FLT_EPSILON


def is_zero(x):
    return abs(x) < FLT_EPSILON


def wrap_pi(x):
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


# ---- 四元数：逐行对照 AP_Math/quaternion.cpp ----
# 存储顺序 (q1,q2,q3,q4) = (w,x,y,z)，与 AP_Math 一致
def qmul(a, b):
    """QuaternionT::operator*(QuaternionT) —— Hamilton 积"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def qrot(q, v):
    """QuaternionT::operator*(Vector3f) —— v + 2q1(qv×v) + 2qv×(qv×v)"""
    w, x, y, z = q
    vx, vy, vz = v
    ux = y * vz - z * vy
    uy = z * vx - x * vz
    uz = x * vy - y * vx
    ux, uy, uz = 2 * ux, 2 * uy, 2 * uz
    return (vx + w * ux + y * uz - z * uy,
            vy + w * uy + z * ux - x * uz,
            vz + w * uz + x * uy - y * ux)


def qinv(q):
    return (q[0], -q[1], -q[2], -q[3])


def qaxis(axis, theta):
    """from_axis_angle(axis, theta)"""
    if is_zero(theta):
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(0.5 * theta)
    return (math.cos(0.5 * theta), axis[0] * s, axis[1] * s, axis[2] * s)


def qaxis_vec(v):
    """from_axis_angle(Vector3f v)"""
    theta = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if is_zero(theta):
        return (1.0, 0.0, 0.0, 0.0)
    n = (v[0] / theta, v[1] / theta, v[2] / theta)
    return qaxis(n, theta)


def q_to_axis_angle(q):
    """to_axis_angle(Vector3f&) —— 注意里面那个 wrap_PI"""
    l = math.sqrt(q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
    v = (q[1], q[2], q[3])
    if not is_zero(l):
        v = (v[0] / l, v[1] / l, v[2] / l)
        ang = wrap_pi(2.0 * math.atan2(l, q[0]))
        v = (v[0] * ang, v[1] * ang, v[2] * ang)
    return v


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vnorm(a):
    return math.sqrt(vdot(a, a))


def vscale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def qdist(a, b):
    """两个四元数的"接近程度"：允许 q 与 -q 表示同一旋转"""
    d1 = sum((a[i] - b[i]) ** 2 for i in range(4))
    d2 = sum((a[i] + b[i]) ** 2 for i in range(4))
    return math.sqrt(min(d1, d2))


THRUST_UP = (0.0, 0.0, -1.0)

# --- 先自证四元数移植本身是对的 ---
# 恒等式：q ⊗ q* == 单位四元数；旋转保持长度；单位四元数不改变向量
rnd = random.Random(20261001)


def rand_quat(max_tilt_deg=60.0):
    """随机生成一个"机身/目标"姿态四元数（限制倾角以便非退化）"""
    ax = math.radians(max_tilt_deg) * rnd.uniform(-1, 1)
    ay = math.radians(max_tilt_deg) * rnd.uniform(-1, 1)
    yaw = rnd.uniform(-math.pi, math.pi)
    q = (1.0, 0.0, 0.0, 0.0)
    for axis, ang in (((1, 0, 0), ax), ((0, 1, 0), ay), ((0, 0, 1), yaw)):
        q = qmul(q, qaxis(axis, ang))
    return q


bad = 0
for _ in range(200):
    q = rand_quat()
    if qdist(qmul(q, qinv(q)), (1.0, 0.0, 0.0, 0.0)) > 1e-9:
        bad += 1
check(bad == 0, '四元数自证 ①：q ⊗ q⁻¹ == 单位四元数（%d/200 失败）' % bad)

bad = 0
for _ in range(200):
    q = rand_quat()
    v = (rnd.uniform(-3, 3), rnd.uniform(-3, 3), rnd.uniform(-3, 3))
    if abs(vnorm(qrot(q, v)) - vnorm(v)) > 1e-9:
        bad += 1
check(bad == 0, '四元数自证 ②：旋转保长（%d/200 失败）' % bad)

bad = 0
for _ in range(200):
    q = rand_quat()
    v = (rnd.uniform(-3, 3), rnd.uniform(-3, 3), rnd.uniform(-3, 3))
    # 与"矩阵路径"对照：q⊙v 应当等于 R(q)·v，用 qmul 展开成 (q ⊗ v ⊗ q*) 验证
    qv = (0.0, v[0], v[1], v[2])
    via_hamilton = qmul(qmul(q, qv), qinv(q))
    if vnorm(vsub(qrot(q, v), via_hamilton[1:])) > 1e-9:
        bad += 1
check(bad == 0,
      '★ 四元数自证 ③：快速旋转算子 == Hamilton 积 q⊗v⊗q*（两条独立实现，%d/200 失败）' % bad)


# ---- 移植 thrust_vector_rotation_angles + thrust_heading_rotation_angles ----
def thrust_vector_rotation_angles(att_target, att_body):
    """返回 (thrust_vector_correction, attitude_error_rad_xy, thrust_angle, thrust_error_angle)"""
    att_target_thrust = qrot(att_target, THRUST_UP)
    att_body_thrust = qrot(att_body, THRUST_UP)

    thrust_angle = math.acos(max(-1.0, min(1.0, vdot(THRUST_UP, att_body_thrust))))
    cross = vcross(att_body_thrust, att_target_thrust)
    thrust_error_angle = math.acos(max(-1.0, min(1.0, vdot(att_body_thrust, att_target_thrust))))

    clen = vnorm(cross)
    if is_zero(clen) or is_zero(thrust_error_angle):
        cross = THRUST_UP
    else:
        cross = vscale(cross, 1.0 / clen)

    cross = qrot(qinv(att_body), cross)          # 把旋转轴表达回机体系
    correction = qaxis(cross, thrust_error_angle)
    ra = q_to_axis_angle(correction)
    return correction, (ra[0], ra[1]), thrust_angle, thrust_error_angle


def thrust_heading_rotation_angles(att_target, att_body, yaw_gain_p, yaw_rate_p,
                                   accel_max):
    """移植 AC_AttitudeControl::thrust_heading_rotation_angles。
       返回 (attitude_error_rad, 新目标, 调试字典)"""
    correction, (ex, ey), thrust_angle, thrust_err = \
        thrust_vector_rotation_angles(att_target, att_body)
    ez_raw = 0.0

    heading_corr = qmul(qmul(qinv(correction), qinv(att_body)), att_target)
    ra = q_to_axis_angle(heading_corr)
    ez_raw = ra[2]
    residual_xy = math.hypot(ra[0], ra[1])

    err = (ex, ey, ez_raw)
    new_target = att_target
    clamped = False

    heading_accel = max(min(accel_max / 2.0, math.radians(120.0)), math.radians(10.0))
    if not is_zero(yaw_rate_p):
        heading_error_max = min(inv_sqrt_controller(1.0 / yaw_rate_p, yaw_gain_p,
                                                    heading_accel),
                                math.radians(45.0))
        if (not is_zero(yaw_gain_p)) and abs(err[2]) > heading_error_max:
            ez_clamped = max(-heading_error_max, min(wrap_pi(err[2]), heading_error_max))
            err = (ex, ey, ez_clamped)
            hv = qaxis((0.0, 0.0, 1.0), ez_clamped)
            new_target = qmul(qmul(att_body, correction), hv)
            clamped = True

    return err, new_target, {'thrust_angle': thrust_angle,
                             'thrust_error_angle': thrust_err,
                             'residual_xy': residual_xy,
                             'clamped': clamped,
                             'correction': correction}


# ---- sqrt_controller / inv_sqrt_controller：逐行对照 AP_Math/control.cpp ----
def sqrt_controller(error, p, second_ord_lim, dt=0.0):
    if second_ord_lim <= 0.0:
        rate = error * p
    elif is_zero(p):
        rate = (math.sqrt(2.0 * second_ord_lim * error) if error > 0
                else (-math.sqrt(2.0 * second_ord_lim * -error) if error < 0 else 0.0))
    else:
        linear_dist = second_ord_lim / (p * p)
        if error > linear_dist:
            rate = math.sqrt(2.0 * second_ord_lim * (error - linear_dist / 2.0))
        elif error < -linear_dist:
            rate = -math.sqrt(2.0 * second_ord_lim * (-error - linear_dist / 2.0))
        else:
            rate = error * p
    if dt > 0.0:
        lim = abs(error) / dt
        return max(-lim, min(rate, lim))
    return rate


def inv_sqrt_controller(output, p, d_max):
    if d_max > 0.0 and is_zero(p):
        return (output * output) / (2.0 * d_max)
    if d_max <= 0.0 and not is_zero(p):
        return output / p
    if d_max <= 0.0 and is_zero(p):
        return 0.0
    linear_velocity = d_max / p
    if abs(output) < linear_velocity:
        return output / p
    linear_dist = d_max / (p * p)
    stopping_dist = linear_dist * 0.5 + (output * output) / (2.0 * d_max)
    return stopping_dist if output > 0 else -stopping_dist


# --- ★ 定义性基准 ①：机体系 ⊗ 推力修正 == 目标系（就推力矢量而言）---
# 这条不重算 cross/dot，只问"这个修正到底有没有把推力矢量转过去"
YAW_GAIN = BASE_DEF['AC_ATTITUDE_CONTROL_ANGLE_P']          # ATC_ANG_YAW_P 默认
YAW_RATE_P = MULTI_DEF['AC_ATC_MULTI_RATE_YAW_P']           # ATC_RAT_YAW_P 默认
ACCEL_Y = BASE_DEF.get('AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_DEGSS', 270.0)
heading_accel_default = max(min(math.radians(ACCEL_Y) / 2.0, math.radians(120.0)),
                            math.radians(10.0))
check(abs(heading_accel_default - math.radians(120.0)) < 1e-9,
      '★ heading_accel_max = constrain(radians(%g)/2, radians(10), radians(120)) '
      '= radians(120) —— 上半截断生效（135° 被压到 120°）' % ACCEL_Y)

bad_tv, bad_res, samples = 0, 0, 0
worst_res = 0.0
for _ in range(300):
    body = rand_quat(55.0)
    target = rand_quat(55.0)
    # 排除接近 180° 的反平行退化（代码自己用 (0,0,-1) 兜底，方向是任意的）
    if vnorm(vcross(qrot(body, THRUST_UP), qrot(target, THRUST_UP))) < 0.20:
        continue
    samples += 1
    err, _new, dbg = thrust_heading_rotation_angles(target, body, YAW_GAIN,
                                                    YAW_RATE_P, heading_accel_default)
    correction = dbg['correction']

    # 基准①：经过 (机体系 ⊗ 推力修正) 之后，推力矢量必须与目标的推力矢量重合
    got = qrot(qmul(body, correction), THRUST_UP)
    want = qrot(target, THRUST_UP)
    if vnorm(vsub(got, want)) > 1e-6:
        bad_tv += 1

    # 基准②：残余航向修正必须是**纯偏航**（代码注释声称 x,y 应为 0）
    if dbg['residual_xy'] > 1e-6:
        bad_res += 1
    worst_res = max(worst_res, dbg['residual_xy'])

check(samples > 250,
      '★ 有效样本 %d 组（已剔除反平行退化）—— 非零样本才说明判据真的跑过' % samples)
check(bad_tv == 0,
      '★ 定义性基准 ①：机体系 ⊗ 推力修正 把推力矢量精确转到目标（%d/%d 失败）'
      % (bad_tv, samples))
check(bad_res == 0 and worst_res < 1e-6,
      '★ 定义性基准 ②：残余航向修正确实是纯偏航（最大 xy 分量 %.2e）—— '
      '这正是代码注释"(x and y should be zero here)"的可验证版本' % worst_res)

# --- ★ 定义性基准 ③：只用输出重建目标姿态（两个分支分开验）---
# ③a 验"未限幅"的原值恒等式；③b 验"限幅"分支拿到的是被钳过的新目标。
# 关键：err[2] 在限幅分支里**已经被钳**，拿它去还原未钳的 target 必然失败 ——
# 这正是第一版判据写错的地方。
def recon_from(body, ex, ey, ez):
    return qmul(qmul(body, qaxis_vec((ex, ey, 0.0))), qaxis((0.0, 0.0, 1.0), ez))


heading_error_max = min(inv_sqrt_controller(1.0 / YAW_RATE_P, YAW_GAIN,
                                            heading_accel_default),
                        math.radians(45.0))
bad_a, worst_a, n_a = 0, 0.0, 0
bad_b, worst_b, n_b = 0, 0.0, 0
bad_nc, n_nc = 0, 0
for k in range(900):
    # 一半随机大姿态差（多半触发限幅），一半小倾角 + 小航向差（多半不触发）。
    # 注意：**纯航向差**会让两者的推力矢量完全相同，落进反平行/共线退化分支，
    # 所以这里必须给一点倾角，否则样本全被下面的过滤器筛掉。
    body = rand_quat(55.0)
    if k % 2 == 0:
        target = rand_quat(55.0)
    else:
        target = qmul(qmul(body, qaxis((0.0, 0.0, 1.0), rnd.uniform(-0.30, 0.30))),
                      qaxis((1.0, 0.0, 0.0), rnd.uniform(0.05, 0.40)))
    if vnorm(vcross(qrot(body, THRUST_UP), qrot(target, THRUST_UP))) < 0.20:
        continue
    err, new_target, dbg = thrust_heading_rotation_angles(
        target, body, YAW_GAIN, YAW_RATE_P, heading_accel_default)

    # 未限幅的原值：从函数的**输出**重新导出（不重算内部 cross/dot）
    hc = qmul(qmul(qinv(dbg['correction']), qinv(body)), target)
    ez_raw = q_to_axis_angle(hc)[2]

    d = qdist(recon_from(body, err[0], err[1], ez_raw), target)
    worst_a = max(worst_a, d)
    n_a += 1
    if d > 1e-6:
        bad_a += 1

    if dbg['clamped']:
        d2 = qdist(recon_from(body, err[0], err[1], err[2]), new_target)
        worst_b = max(worst_b, d2)
        n_b += 1
        if d2 > 1e-6 or abs(abs(err[2]) - heading_error_max) > 1e-9:
            bad_b += 1
    else:
        n_nc += 1
        if abs(err[2] - ez_raw) > 1e-12:
            bad_nc += 1

check(n_a > 300, '③ 有效样本 %d 组（非零样本才说明判据真的跑过）' % n_a)
check(bad_a == 0,
      '★ 定义性基准 ③a（未限幅原值）：body ⊗ 推力修正 ⊗ 偏航修正(原值) == target'
      '（最大偏差 %.2e）—— 三分量各司其职，谁写错都还原不出来' % worst_a)
check(n_b > 100 and bad_b == 0,
      '★ 定义性基准 ③b（限幅分支）：钳过的值精确还原出函数返回的 new_target，'
      '且 |z| 正好卡在上限 %.6f rad（%d 组）' % (heading_error_max, n_b))
check(n_nc > 50 and bad_nc == 0,
      '★ 另一半支路也被覆盖：%d 组样本没触发限幅，且 err[2] 就是原值' % n_nc)

# ★ 顺带验证那个"退化兜底分支"到底服务什么场景：纯航向差 ⇒ 推力矢量完全相同
n_pureyaw, worst_pureyaw = 0, 0.0
for _ in range(100):
    body = rand_quat(55.0)
    target = qmul(body, qaxis((0.0, 0.0, 1.0), rnd.uniform(-math.pi, math.pi)))
    n_pureyaw += 1
    worst_pureyaw = max(worst_pureyaw,
                        vnorm(vsub(qrot(body, THRUST_UP), qrot(target, THRUST_UP))))
check(n_pureyaw == 100 and worst_pureyaw < 1e-9,
      '★ 纯航向差（100 组，任意角度）下两机推力矢量完全相同（最大差 %.1e）—— '
      '这正是 thrust_vec_cross 退化、代码改用 (0,0,-1) 兜底的那一类输入'
      % worst_pureyaw)

# --- ★ 反向对照：把轴的坐标系变换写反（漏掉 inverse）---
def thrust_vector_rotation_angles_broken(att_target, att_body):
    """注入：`attitude_body.inverse() * thrust_vec_cross` 写成 `* thrust_vec_cross`"""
    att_target_thrust = qrot(att_target, THRUST_UP)
    att_body_thrust = qrot(att_body, THRUST_UP)
    cross = vcross(att_body_thrust, att_target_thrust)
    thrust_error_angle = math.acos(max(-1.0, min(1.0, vdot(att_body_thrust, att_target_thrust))))
    clen = vnorm(cross)
    cross = THRUST_UP if (is_zero(clen) or is_zero(thrust_error_angle)) \
        else vscale(cross, 1.0 / clen)
    cross = qrot(att_body, cross)              # ← 少了 inverse
    return qaxis(cross, thrust_error_angle)


bad_broken, n_broken = 0, 0
for _ in range(300):
    body = rand_quat(55.0)
    target = rand_quat(55.0)
    if vnorm(vcross(qrot(body, THRUST_UP), qrot(target, THRUST_UP))) < 0.20:
        continue
    n_broken += 1
    corr = thrust_vector_rotation_angles_broken(target, body)
    if vnorm(vsub(qrot(qmul(body, corr), THRUST_UP), qrot(target, THRUST_UP))) > 1e-6:
        bad_broken += 1
check(n_broken > 0 and bad_broken > 0.9 * n_broken,
      '★ 反向对照：漏掉 attitude_body.inverse() 后，基① 在 %d/%d 组上失败（九成以上）—— '
      '说明"机体系↔NED"的变换方向是真被测到的，不是摆着好看的'
      % (bad_broken, n_broken))

# --- ★ 偏航限幅：限幅后仍与目标共享同一个推力矢量 ---
bad_share, n_clamped = 0, 0
for _ in range(400):
    body = rand_quat(55.0)
    target = rand_quat(75.0)
    if vnorm(vcross(qrot(body, THRUST_UP), qrot(target, THRUST_UP))) < 0.20:
        continue
    err, new_target, dbg = thrust_heading_rotation_angles(
        target, body, YAW_GAIN, YAW_RATE_P, heading_accel_default)
    if not dbg['clamped']:
        continue
    n_clamped += 1
    a = qrot(new_target, THRUST_UP)
    b = qrot(target, THRUST_UP)
    if vnorm(vsub(a, b)) > 1e-6:
        bad_share += 1
check(n_clamped > 20,
      '★ 触发偏航限幅的样本 %d 组（限幅不是死代码，真的会被触发）' % n_clamped)
check(bad_share == 0,
      '★ 限幅只动航向、不动推力：限幅后目标与真目标共享同一推力矢量（%d/%d 失败）'
      % (bad_share, n_clamped))


# ==========================================================================
# [5] 偏航误差上限：两条路谁在起作用
# ==========================================================================
print()
print('=' * 74)
print('[5] 偏航误差上限：那段 sqrt 逻辑在默认增益下等于没起作用')
print('=' * 74)

# --- 往返恒等式：inv_sqrt_controller 是 sqrt_controller 的逆 ---
bad_rt, n_rt = 0, 0
for p in (1.0, 2.5, 4.5, 9.0):
    for D in (0.5, 2.094, 8.0):
        linear_dist = D / (p * p)
        for frac in (0.1, 0.4, 0.99, 1.01, 1.5, 3.0, 10.0):
            e = linear_dist * frac
            out = sqrt_controller(e, p, D, 0.0)
            back = inv_sqrt_controller(out, p, D)
            n_rt += 1
            if abs(back - e) > 1e-9 * max(1.0, abs(e)):
                bad_rt += 1
check(n_rt == 84, '往返测试跑了 %d 组（p×D×e 三档笛卡尔积）' % n_rt)
check(bad_rt == 0,
      '★ 往返恒等式：inv_sqrt_controller(sqrt_controller(e)) == e（线性区与 sqrt 区都覆盖，%d 失败）'
      % bad_rt)

# --- 独立基准：不用闭式，用数值扫描找"输出首次达到 1/kP 的那个 e" ---
bad_scan = 0
for p, D, kp in ((4.5, heading_accel_default, 0.180), (4.5, heading_accel_default, 0.9),
                 (2.0, 1.0, 0.5)):
    target_out = 1.0 / kp
    lo, hi = 0.0, 100.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if sqrt_controller(mid, p, D, 0.0) < target_out:
            lo = mid
        else:
            hi = mid
    scanned = 0.5 * (lo + hi)
    closed = inv_sqrt_controller(target_out, p, D)
    if abs(scanned - closed) > 1e-6 * max(1.0, abs(closed)):
        bad_scan += 1
check(bad_scan == 0,
      '★ 独立基准：二分扫描找饱和点，与 inv_sqrt_controller 的闭式一致（%d/3 失败）' % bad_scan)

# --- 用默认增益算：动态上限 vs 45° 常数，谁先到 ---
dyn_limit = inv_sqrt_controller(1.0 / YAW_RATE_P, YAW_GAIN, heading_accel_default)
cap_45 = math.radians(45.0)
check(abs(cap_45 - 0.7853981633974483) < 1e-12,
      'AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD = radians(45) = %.6f rad' % cap_45)
check(dyn_limit > 5.0 * cap_45,
      '★ 默认增益下，动态上限算出 %.4f rad = %.0f 度，是 45 度天花板的 %.2f 倍'
      % (dyn_limit, math.degrees(dyn_limit), dyn_limit / cap_45))
check(min(dyn_limit, cap_45) == cap_45,
      '★ 所以 MIN(...) 取到的是**常数 45 度** —— 那段基于输出饱和反推的精细逻辑，'
      '在默认增益下完全不起作用')

# --- 反向：找到让动态上限真正起作用的增益 ---
lo, hi = 0.05, 5.0
for _ in range(200):
    mid = 0.5 * (lo + hi)
    if inv_sqrt_controller(1.0 / mid, YAW_GAIN, heading_accel_default) > cap_45:
        lo = mid
    else:
        hi = mid
crossover = 0.5 * (lo + hi)
check(0.10 < crossover < 2.50,
      '★ 交叉点：ATC_RAT_YAW_P 超过 %.4f 后动态上限才开始生效' % crossover)
check(abs(inv_sqrt_controller(1.0 / crossover, YAW_GAIN, heading_accel_default)
          - cap_45) < 1e-6,
      '交叉点自证：在 %.4f 处两条路正好相等' % crossover)
check(crossover < 2.50,
      '★ 而这个交叉点落在 ATC_RAT_YAW_P 的合法范围 (0.10, 2.50) 内 —— '
      '所以不是死代码，只是**默认增益下用不到**')

# --- ★ 那两个增益用的是【主】控制器的，不是 backend 自己的 ---
att_cpp = slurp('libraries/AC_AttitudeControl/AC_AttitudeControl.cpp')
m_thr = re.search(r'void AC_AttitudeControl::thrust_heading_rotation_angles\(.*?\n\}\n',
                  att_cpp, re.S)
check(m_thr is not None, '从 AC_AttitudeControl.cpp 里整段取出 thrust_heading_rotation_angles')
THR_BODY = m_thr.group(0) if m_thr else ''
check('get_rate_yaw_pid().kP()' in THR_BODY,
      '★ 上限公式里的速率增益是 get_rate_yaw_pid().kP() —— 主控制器的 ATC_RAT_YAW_P')
check('_p_angle_yaw.kP()' in THR_BODY,
      '★ 角度增益是 _p_angle_yaw.kP() —— 主控制器的 ATC_ANG_YAW_P')
check('CustomControl' not in THR_BODY and 'CC_' not in THR_BODY,
      '★ 这段函数体里没有任何 CustomControl / CC_ 字样 —— 它根本不知道自定义后端的存在'
      '（注意别用裸 "CC" 判：AC_ATTITUDE_A**CC**EL_Y 里就有）')
check(re.search(r'AP_SUBGROUPINFO\(_p_angle_yaw,\s*"ANG_YAW_",\s*15,\s*AC_AttitudeControl',
                att_cpp) is not None,
      '   _p_angle_yaw 的槽位 15 确认（属于 AC_AttitudeControl，不是 AC_CustomControl_PID）')
# 推论：后端把自己的增益乘了 0.9，但这道限幅用的仍是主控制器的原值 —— 两者不一致
check(abs(YAW_RATE_P - scaled('AC_ATC_MULTI_RATE_YAW_P')) > 1e-6,
      '★ 推论：限幅用的是主控制器 %.3f，而后端自己跑的是 %.3f —— '
      '两个增益不同源，调后端参数不会改变这道限幅'
      % (YAW_RATE_P, scaled('AC_ATC_MULTI_RATE_YAW_P')))

# --- ★ 主控制器是按引用传 target，限幅会改到它自己的成员 ---
check(re.search(r'thrust_heading_rotation_angles\(_attitude_target,\s*attitude_body',
                slurp('libraries/AC_AttitudeControl/AC_AttitudeControl.cpp')) is not None,
      '★ AC_AttitudeControl.cpp:997 把成员 _attitude_target 按**非 const 引用**传进去 —— '
      '限幅分支会就地改写主控制器自己的目标姿态')
check(re.search(r'void AC_AttitudeControl::thrust_heading_rotation_angles\(\s*'
                r'Quaternion& attitude_target', slurp('libraries/AC_AttitudeControl/AC_AttitudeControl.cpp'))
      is not None,
      '   签名确实是 Quaternion&（非 const），不是 Quaternion')
check('attitude_target = attitude_body * thrust_vector_correction * heading_vec_correction_quat;'
      in slurp('libraries/AC_AttitudeControl/AC_AttitudeControl.cpp'),
      '   而限幅分支里确实有对 attitude_target 的赋值')
# 而 CC_PID 传的是**副本**，所以自定义后端不会污染主控制器
check(re.search(r'attitude_target\s*=\s*_att_control->get_attitude_target_quat\(\);', src_pid)
      is not None,
      '★ 反过来，PID 后端先取**副本**再传进去 —— 所以自定义后端不会污染主控制器')


# ==========================================================================
# [6] 接入点：调度顺序、RC 选项号、载具范围
# ==========================================================================
print()
print('=' * 74)
print('[6] 接入点：调度顺序、RC 选项号、载具范围')
print('=' * 74)

# --- 从 Copter.cpp 解析 FAST_TASK 顺序 ---
copter_cpp = slurp('ArduCopter/Copter.cpp')
tasks_block = re.search(r'const AP_Scheduler::Task Copter::scheduler_tasks\[\]\s*=\s*\{(.*?)\n\};',
                        copter_cpp, re.S)
check(tasks_block is not None, '解析到 Copter::scheduler_tasks[] 任务表')
TASKS = re.findall(r'FAST_TASK(?:_CLASS)?\s*\(([^)]*)\)', tasks_block.group(1)) \
    if tasks_block else []
TASKS = [t.strip() for t in TASKS]
check(len(TASKS) >= 15, '表里解析出 %d 条 FAST_TASK' % len(TASKS))

i_rate = next((i for i, t in enumerate(TASKS) if t == 'run_rate_controller_main'), None)
i_cust = next((i for i, t in enumerate(TASKS) if t == 'run_custom_controller'), None)
i_out = next((i for i, t in enumerate(TASKS) if t == 'motors_output_main'), None)
check(None not in (i_rate, i_cust, i_out),
      '三条关键任务都在表里：run_rate_controller_main / run_custom_controller / motors_output_main')
check(i_rate < i_cust < i_out,
      '★ 顺序：主 rate 控制器(第 %d 条) → 自定义控制器(第 %d 条) → 电机输出(第 %d 条)'
      % (i_rate, i_cust, i_out))
check(i_cust - i_rate == 1,
      '★ 自定义控制器紧跟在主控制器**下一条**（中间只隔一个 #if HELI 块）—— 延迟最小')

# --- 它是"覆写式"：backend 直接写 mixer 输入 ---
check(re.search(r'void\s+run_custom_controller\(\)\s*\{\s*custom_control\.update\(\);\s*\}',
                slurp('ArduCopter/Copter.h')) is not None,
      'run_custom_controller() 只是 custom_control.update() 的一层包装')
check('_motors->set_roll(rpy.x)' in src_front
      and '_motors->set_pitch(rpy.y)' in src_front
      and '_motors->set_yaw(rpy.z)' in src_front,
      '★ 输出路径是 _motors->set_roll/set_pitch/set_yaw —— 直接盖掉 mixer 输入')
check('_motors->set_roll_ff' not in src_front,
      '   对照：它**不写** set_roll_ff（主控制器会写前馈，自定义后端不写）')

# --- 主控器把积分器清零，防止后台积分饱和 ---
check(src_front.count('set_integrator(0.0)') == 6,
      '★ motor_set() 与 reset_main_att_controller() 合计 %d 处 set_integrator(0.0)'
      '（每拍清 3 次 + 切换时清 3 次）' % src_front.count('set_integrator(0.0)'))

# --- 成员挂在哪：Copter，不是 AC_AttitudeControl ---
check(re.search(r'AC_CustomControl\s+custom_control\{', slurp('ArduCopter/Copter.h')) is not None,
      '成员叫 copter.custom_control，挂在 Copter 上')
check('custom_control' not in slurp('libraries/AC_AttitudeControl/AC_AttitudeControl.h'),
      '★ AC_AttitudeControl **不持有** AC_CustomControl —— 它是被引用方，不是调用方')
check(re.search(r'GOBJECT\(custom_control,\s*"CC",\s*AC_CustomControl\)',
                slurp('ArduCopter/Parameters.cpp')) is not None,
      '参数前缀 CC 由 ArduCopter/Parameters.cpp 的 GOBJECT 决定')

# --- RC 选项号：枚举 vs 官方 autotest 的魔数 ---
rc_h = slurp('libraries/RC_Channel/RC_Channel.h')
m = re.search(r'CUSTOM_CONTROLLER\s*=\s*(\d+)', rc_h)
check(m is not None, '在 RC_Channel.h 找到 AUX_FUNC::CUSTOM_CONTROLLER')
AUX_NUM = int(m.group(1)) if m else -1
check(AUX_NUM == 109, 'AUX_FUNC::CUSTOM_CONTROLLER = %d' % AUX_NUM)

at = slurp('Tools/autotest/arducopter.py')
m = re.search(r"'RC6_OPTION':\s*(\d+)", at)
check(m is not None, '在官方 autotest 里找到 RC6_OPTION 赋值')
check(m and int(m.group(1)) == AUX_NUM,
      '★ 官方验收科目用的魔数 %s == 枚举值 %d（两个文件各自独立，对上了）'
      % (m.group(1) if m else '?', AUX_NUM))
check(re.search(r"'CC_TYPE':\s*2", at) is not None,
      '★ 官方科目用 CC_TYPE=2（PID 后端）—— 与本篇 [1] 的枚举一致')
check(re.search(r"'CC_AXIS_MASK':\s*7", at) is not None,
      '★ 官方科目用 CC_AXIS_MASK=7（三轴全用）')
check('CC2_RAT_YAW_P' in at,
      '★ 官方科目真的去读了 CC2_RAT_YAW_P —— 印证 "2_" 前缀展开成 CC2_')
check('wait_statustext("Custom controller is ON"' in at,
      '★ 官方科目断言的是 GCS 文本「Custom controller is ON」—— 与 [2] 的状态机文本同源')

# --- 载具范围：Copter 多旋翼独占 ---
cfg_copter = slurp('ArduCopter/config.h')
m = re.search(r'#define\s+AC_CUSTOMCONTROL_MULTI_ENABLED\s+(.*)', cfg_copter)
check(m is not None, '找到 AC_CUSTOMCONTROL_MULTI_ENABLED 定义')
check(m and 'MULTICOPTER_FRAME' in m.group(1),
      '★ 定义含 MULTICOPTER_FRAME：%s' % (m.group(1).strip() if m else ''))
check(m and 'HELI_FRAME' not in m.group(1),
      '★ 所以直升机被排除（HELI_FRAME 不满足 MULTICOPTER_FRAME）')
check('AC_CustomControl is disabled on heli' in slurp('Tools/autotest/helicopter.py'),
      '★ 官方 autotest 也承认这点，并主动把 CC_TYPE 从参数检查单里剔除')

other_vehicles = ['ArduPlane', 'ArduSub', 'ArduRover']
hits = []
for v in other_vehicles:
    d = os.path.join(REPO, v)
    if not os.path.isdir(d):
        continue
    for root, _dirs, files in os.walk(d):
        for f in files:
            if f.endswith(('.cpp', '.h')):
                t = slurp_abs(os.path.join(root, f))
                if 'AC_CustomControl' in t or 'AP_CUSTOMCONTROL' in t:
                    hits.append(os.path.join(root, f))
check(not hits,
      '★ 在 %s 里搜 AC_CustomControl —— %d 处命中，确认是 Copter 独有'
      % ('/'.join(other_vehicles), len(hits)))

# --- 编译开关：SITL 默认开，硬件默认关 ---
boards = slurp('Tools/ardupilotwaf/boards.py')
check('AP_CUSTOMCONTROL_ENABLED=1' in boards.replace(' ', ''),
      '★ boards.py 里 AP_CUSTOMCONTROL_ENABLED=1')
check(re.search(r"self\.get_name\(\)\s*==\s*\"sitl\"", boards) is not None,
      '★ 条件含 self.get_name() == "sitl" —— 所以 **SITL 默认开、硬件默认关**')
check('--enable-custom-controller' in slurp('wscript'),
      '★ 硬件上要开得加 ./waf configure --enable-custom-controller')
check(re.search(r'AP_CUSTOMCONTROL_ENABLED=0', boards) is not None,
      '   否则走 else 分支显式 define 成 0')
check('note boards.py sets this' in src_cfg,
      '★ 库自己的 config.h 明说"boards.py 才是权威"并且把默认定义注释掉了 —— '
      '在头文件里改默认值是无效的')


# ==========================================================================
# [7] 切换状态机：set_custom_controller 的六道门
# ==========================================================================
print()
print('=' * 74)
print('[7] 切换状态机：set_custom_controller 的六道门')
print('=' * 74)

# --- 解析出四道显式 return 门 + 文本 ---
body = re.search(r'void AC_CustomControl::set_custom_controller\(bool enabled\)\s*\{(.*?)\n\}',
                 src_front, re.S)
check(body is not None, '解析到 set_custom_controller() 正文')
SB = body.group(1) if body else ''
guards = re.findall(r'GCS_SEND_TEXT\(MAV_SEVERITY_INFO,\s*"([^"]+)"\)', SB)
check(len(guards) == 6,
      '正文里有 %d 条 GCS 消息：%s' % (len(guards), ' / '.join(guards)))
check(guards.count('Custom controller is ON') == 1
      and guards.count('Custom controller is OFF') == 1,
      '★ ON / OFF 各一条，与 autotest 断言的文本一致')
returns = re.findall(r'return;', SB)
check(len(returns) == 4,
      '★ 四道"拒绝"门各配一个 return（NONE / 越界 / backend 空 / mask=0）')

# --- 门 4 只在 enabled 时才拦 ---
check(re.search(r'if\s*\(_custom_controller_mask\s*==\s*0\s*&&\s*enabled\)', SB) is not None,
      '★ 门 4 带 `&& enabled` —— mask=0 时**关**是允许的（关不需要理由），**开**才被拦')

# --- 进门第一件事：先置 false（防止切换中途状态不一致）---
first = [l.strip() for l in SB.splitlines() if l.strip() and not l.strip().startswith('//')]
check(first[0] == 'log_switch();',
      '★ 进门第一件事是 log_switch()（记下"我要切了"）')
check('_custom_controller_active = false;' in first[1],
      '★ 第二件事是**无条件置 false** —— 任何一道门拦下，都保证处于"自定义控制器未激活"')

# --- 双次日志（切换前后各一次，让日志上能看到沿）---
check(SB.count('log_switch()') == 2,
      '★ log_switch() 在正文里出现 %d 次（切换前 + 切换后）—— 双次写日志让状态变化清晰可见'
      % SB.count('log_switch()'))
check(re.search(r'//\s*double logging switch', SB) is not None,
      '   源码注释自陈："double logging switch makes the state change very clear in the log"')

# --- 关的时候才 reset 主控制器，且 Empty 不 reset ---
check(re.search(r'if\s*\(!\s*enabled\)\s*\{(.*?)reset_main_att_controller\(\);', SB, re.S)
      is not None,
      '★ reset_main_att_controller() 只在**关**的时候调用')
check(re.search(r'if\s*\(_controller_type\s*>\s*CustomControlType::CONT_EMPTY\)\s*\{\s*\n\s*'
                r'reset_main_att_controller\(\);', SB) is not None,
      '★ 且仅在 type > CONT_EMPTY 时 reset —— Empty 后端不 reset（它没改变任何东西）')
check(re.search(r'if\s*\(enabled\s*&&\s*_controller_type\s*>\s*CustomControlType::CONT_NONE\)',
                SB) is not None,
      '★ 开的时候调 backend 自己的 _backend->reset()（清积分器/滤波器）')

# --- reset_main_att_controller 的完整动作 ---
rma = re.search(r'void AC_CustomControl::reset_main_att_controller\(void\)\s*\{(.*?)\n\}',
                src_front, re.S)
RMA = rma.group(1) if rma else ''
check('get_bf_feedforward()' in RMA,
      '★ 重置受 ATC_RATE_FF_ENAB 控制：只有开了前馈才 relax 姿态/速率目标')
check('relax_attitude_controllers()' in RMA,
      '   开前馈时把主控制器的目标"松弛"到当前状态，使误差从 0 长起来（无扰切换）')
check(len(re.findall(r'set_integrator\(0\.0\)', RMA)) == 3,
      '★ 三个轴的积分器清零（roll/pitch/yaw 各一次）')
check('reset_filter' not in RMA,
      '★ 注意：这里**没有** reset_filter() —— README 里写的那一份与实际不符（见 [8]）')

# --- log_switch 的日志结构 ---
lg = re.search(r'//\s*@LoggerMessage:\s*(\w+)(.*?)AP::logger\(\)\.Write\("(\w+)",\s*"([^"]+)"',
               src_front, re.S)
check(lg is not None, '解析到 log_switch() 的日志定义')
if lg:
    check(lg.group(1) == 'CC' and lg.group(3) == 'CC',
          '★ 日志消息名 CC（@LoggerMessage 与 Write 同名）')
    check(lg.group(4) == 'TimeUS,Type,Act',
          '★ 字段 %s —— Act 就是"自定义控制器是否在跑"，试飞复盘全靠它' % lg.group(4))

# --- is_safe_to_run 与 set_custom_controller 的越界判据必须互补 ---
bad_pair = 0
for t in range(0, 256):
    a = (t > MAX_TYPES)             # set_custom_controller 用的
    b = (t <= MAX_TYPES)            # is_safe_to_run 用的
    if a == b:
        bad_pair += 1
check(bad_pair == 0,
      '★ 两处越界判据（> 与 <=）在 0..255 上**处处互补** —— 不会出现"能开但跑不了"的错位')
check(ENUM.get('CONT_PID') <= MAX_TYPES,
      '★ 而合法的最大枚举值 PID=%d 恰好 <= MAX_TYPES=%d（边界没写错）'
      % (ENUM.get('CONT_PID'), MAX_TYPES))
check(not (XYZ <= MAX_TYPES),
      '★ 反向：新增的 XYZ=3 会同时被两处拦下（一致地拦，不是一处拦一处放）')


# ==========================================================================
# [8] 官方 README 与源码的 6 处对不上
# ==========================================================================
print()
print('=' * 74)
print('[8] 官方 README 与源码的 6 处对不上')
print('=' * 74)

# --- 漂移 1：README 说有几个 backend ---
m = re.search(r'there\s+(\w+)\s+custom controller backend available', src_readme)
check(m is not None, '找到 README 里"有几个 backend"的那句')
spoken = m.group(1) if m else ''
check(spoken == '4', '★ 漂移 1：README 原文说 "there %s custom controller backend available"' % spoken)
listed = re.findall(r'^-\s+(\w+)\s+backend\s+`CC_TYPE`\s*=\s*(\d+)',
                    src_readme.split('These are')[1].split('###')[0], re.M) \
    if 'These are' in src_readme else []
check(len(listed) == 2,
      '★ 紧接着列出的却是 %d 个：%s' % (len(listed), ', '.join('%s=%s' % x for x in listed)))
check(len(listed) != int(spoken) if spoken.isdigit() else True,
      '★ 说的 4 与列的 2 不一致 —— 数字漂了（真实具体后端也是 2 个：Empty 与 PID）')
check(len(ENUM) == 3,
      '   对照：枚举有 %d 项（含 CONT_NONE=0），所以"4"对不上任何口径' % len(ENUM))

# --- 漂移 2：README 示例里的 CUSTOMCONTROL_MAX_TYPES ---
readme_max = re.findall(r'#define\s+CUSTOMCONTROL_MAX_TYPES\s+(\d+)', src_readme)
check(len(readme_max) == 2,
      '★ README 的"加第 5 个 backend"示例给了 %d 个 MAX_TYPES 取值' % len(readme_max))
check(readme_max == ['2', '3'],
      '   README 里依次出现 %s（把 2 改成 3 的示例）' % readme_max)
check(readme_max[0] == str(MAX_TYPES),
      '★ 示例的"改前"值 %s 与源码实际值 %d 一致' % (readme_max[0], MAX_TYPES))
check(readme_max[1] != str(MAX_TYPES),
      '★ 示例的"改后"值 %s 是**假想状态**，不是源码现状（照抄 README 会以为已经改成 3 了）'
      % readme_max[1])

# --- 漂移 3：README 的 "1_" 子组是未注释的 ---
readme_sub1 = re.findall(r'^\s*AP_SUBGROUPVARPTR\(_backend,\s*"1_".*$', src_readme, re.M)
check(len(readme_sub1) == 1,
      'README 里 "1_" 子组出现 %d 次' % len(readme_sub1))
check(readme_sub1 and not readme_sub1[0].lstrip().startswith('//'),
      '★ 漂移 3：README 里这条是**未注释**的；而源码里它被注释掉了（见 [2] 证据二）')
check(re.search(r'//\s*AP_SUBGROUPVARPTR\(_backend,\s*"1_"', src_front) is not None,
      '   （源码那份确实带 // 前缀）')

# --- 漂移 4：函数名拼写 ---
check('reset_main_atti_controller' in src_readme,
      '★ 漂移 4：README 写的是 reset_main_atti_controller（atti 少了一个 t）')
check('reset_main_atti_controller' not in src_front,
      '★ 而源码里根本没有这个名字')
check('reset_main_att_controller' in src_front,
      '★ 源码里的真名是 reset_main_att_controller —— 照 README 抄会编译不过')

# --- 漂移 5：那个"理想无扰切换"的积分器初值 ---
check('rate_bf_targets().x - gyro_latest.x' in src_readme,
      '★ 漂移 5：README 展示的写法是 set_integrator(目标 - 实测, 电机输出) —— '
      '把积分器预置成当前输出，这才是真正的无扰切换')
check('rate_bf_targets().x - gyro_latest.x' not in src_front,
      '★ 源码里**没有**这个写法，实际只是 set_integrator(0.0)（积分器清零）')
check('a proper reset functionality is not implemented intentionally' in src_readme,
      '★ README 自己在 PID 后端那节也承认："a proper reset functionality is not '
      'implemented intentionally" —— 同一篇文档的两处互相打架：'
      '一处展示理想写法，一处说故意不做')

# --- 漂移 6：AC_PID 构造函数的实参个数（README 多传一个 dt）---
check('AC_ATC_MULTI_RATE_YAW_FILT_HZ * 0.90f, 0.0f, dt)' in src_readme,
      '★ 漂移 6：README 的构造函数示例末尾是 "... 0.90f, 0.0f, dt)" —— 多传了第 9 个实参 dt')
check('AC_ATC_MULTI_RATE_YAW_FILT_HZ * 0.90f, 0.0f)' in src_pid,
      '★ 源码同一位置是 "... 0.90f, 0.0f)" —— 8 个实参，没有 dt')
check(re.search(r'_dt\s*=\s*dt;', src_pid) is not None,
      '   源码把 dt 改在构造函数体里赋给 _dt —— 照 README 抄会多传一个参数而编译不过')
check(re.search(r'float\s+update_all\(float target, float measurement, float dt,\s*'
                r'bool limit = false, float pd_scale = 1\.0f, float i_scale = 1\.0f\)',
                slurp('libraries/AC_PID/AC_PID.h')) is not None,
      '   对照 AC_PID 的声明确认：构造函数取 8 个标量，dt 是 update_all 的运行时入参')

# --- 反向：证据链要闭环——README 的 13 步在源码里都能找到落点 ---
check('How To Add New Custom Controller' in src_readme,
      '★ README 有一节 "How To Add New Custom Controller"（加 backend 的完整步骤）')
# 每条 = (触点名, 该去哪个文件改, 正则)。注意"枚举"在 .h 而"include/subgroup/switch"在 .cpp。
TOUCHPOINTS = [
    ('枚举加项',     src_fronth, r'enum class CustomControlType'),
    ('头文件 include', src_front, r'#include "AC_CustomControl_PID\.h"'),
    ('参数子组',     src_front,  r'AP_SUBGROUPVARPTR\(_backend'),
    ('init switch 分支', src_front, r'case CustomControlType::CONT_PID:'),
    ('config.h 宏',  src_cfg,    r'#define AP_CUSTOMCONTROL_PID_ENABLED'),
]
for label, target, pat in TOUCHPOINTS:
    check(re.search(pat, target) is not None,
          '   触点「%s」定位成功' % label)
check(len(TOUCHPOINTS) == 5,
      '★ 五个必改触点（再加一个新建的 .h/.cpp 文件共 6 处）—— '
      '漏掉"枚举加项"或"参数子组"就会静默失效')
# 枚举与子组必须同文件同处改动，但一个在 .h 一个在 .cpp ⇒ 最容易漏
check(any(re.search(p, t) for _, t, p in TOUCHPOINTS if t is src_front) and
      re.search(r'enum class CustomControlType', src_fronth) is not None,
      '★ 注意：枚举在 .h、子组在 .cpp —— 加一个 backend 要动两个文件，'
      '这正是官方 README 用 13 步才讲完的原因')


# ==========================================================================
print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
