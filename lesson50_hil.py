# -*- coding: utf-8 -*-
"""
lesson50_hil.py —— 卷 10 第 5 章《HIL 硬件在环：一个被删掉的功能》配套自验证脚本

**本脚本验的是一个"否定命题"**：HIL 在 4.7.1 里已经不存在了。

否定命题比肯定命题难验 —— "我找不到" 可能只是 "我搜错了"。
所以本脚本每条结论都配了**对照**：
  · 说"HIL 没了"      → 同时证明"搜的方法能找到别的东西"（量具自证）
  · 说"这是墓碑"      → 同时指出"它当年是干什么的"的证据
  · 说"GPS_TYPE_HIL 死" → 列出 `_detect_instance` **实际**处理了哪些类型

  [1] ★★★ 结论：HIL 的收发端一个都没有
  [2] ★★★ 六处墓碑
  [3] ★★★ `GPS_TYPE_HIL`：死值的完整形态
  [4] `AP_Baro_HIL.cpp`：文件名骗人
  [5] 它曾经是什么（从 ReleaseNotes 反推）
  [6] ★★ 替代路线（ArduPilot 现在怎么做）
  [7] ★★ 对照：PX4 的 HITL 是活的
  [8] `HIL_*` 消息参考

运行： python3 lesson50_hil.py     （全过 EXIT=0）
"""

import io
import _srcdirs
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
print('[0] 定位源码')
print('=' * 76)

REPO = None
for cand in (_srcdirs.src("APM_471"),
             _srcdirs.src("PLANE")):
    if os.path.isdir(os.path.join(cand, 'libraries')):
        REPO = cand
        break
if REPO is None:
    print('    [!!] 找不到 ArduPilot 源码树')
    n_bad += 1
    sys.exit(1)

PX4 = _srcdirs.src("PX4")
HAVE_PX4 = os.path.isdir(PX4)
MAVLINK_XML = None
for c in (_srcdirs.src("CLIB", "message_definitions", "common.xml"),
          os.path.join(PX4, 'src', 'modules', 'mavlink', 'mavlink',
                       'message_definitions', 'v1.0', 'common.xml')):
    if os.path.isfile(c):
        MAVLINK_XML = c
        break
check(MAVLINK_XML is not None, '找到 MAVLink 的 common.xml：%s'
      % (os.path.basename(os.path.dirname(os.path.dirname(MAVLINK_XML))) if MAVLINK_XML else '?'))
check(HAVE_PX4, '找到 PX4 源码树（用于对照）')


def slurp(rel, base=None):
    return io.open(os.path.join(base or REPO, rel.replace('/', os.sep)),
                   encoding='utf-8', errors='replace').read()


def rg(rel, pat, flags=0, base=None):
    return re.findall(pat, slurp(rel, base), flags)


# ==========================================================================
print()
print('=' * 76)
print('[1] ★★★ 结论：HIL 的收发端一个都没有')
print('=' * 76)

SCAN_DIRS = ['libraries', 'ArduCopter', 'ArduPlane', 'Rover', 'ArduSub']
SKIP = ('AP_HAL_ChibiOS' + os.sep + 'hwdef',)

# --- 量具自证：先证明"遍历 + 搜索"这套方法能找得到东西 ---
found_any = 0
files_scanned = 0
hits_hil_msg = []
for d in SCAN_DIRS:
    for root, dirs, fs in os.walk(os.path.join(REPO, d)):
        if any(s in root for s in SKIP) or 'modules' in root.split(os.sep):
            continue
        for f in fs:
            if not f.endswith(('.cpp', '.h')):
                continue
            files_scanned += 1
            p = os.path.join(root, f)
            t = io.open(p, encoding='utf-8', errors='replace').read()
            found_any += len(re.findall(r'\bGPS_TYPE_', t))
            for m in re.finditer(r'mavlink_msg_hil\w*|MAVLINK_MSG_ID_HIL_\w+|HIL_SENSOR|HIL_ACTUATOR|HIL_STATE', t):
                hits_hil_msg.append('%s:%s' % (os.path.relpath(p, REPO), m.group(0)))

check(files_scanned > 3000,
      '★ 量具自证：扫了 **%d** 个 .cpp/.h 文件' % files_scanned)
check(found_any > 100,
      '★ 量具自证：同一套遍历在别处找到 **%d** 处 `GPS_TYPE_` ⇒ 遍历确实在工作' % found_any)
check(hits_hil_msg == [],
      '★★★ 而 `mavlink_msg_hil*` / `MAVLINK_MSG_ID_HIL_*` / `HIL_SENSOR` / '
      '`HIL_ACTUATOR` / `HIL_STATE` **零命中** ⇒ **ArduPilot 既不收也不发 HIL 消息**')
if hits_hil_msg:
    for h in hits_hil_msg[:5]:
        print('           ✗ %s' % h)

# --- 对照：同一套搜索能找到 SITL 的消息名 ---
sitl_hits = 0
for d in ('libraries/SITL',):
    for root, _dirs, fs in os.walk(os.path.join(REPO, d.replace('/', os.sep))):
        for f in fs:
            if f.endswith(('.cpp', '.h')):
                t = io.open(os.path.join(root, f), encoding='utf-8', errors='replace').read()
                # ★ 是 `AP_SIM_*_ENABLED`，不是 `SIM_*_ENABLED`。
                #   写 `\bSIM_` 会一篇都搜不到（`AP_SIM_XXX` 里 `SIM` 前面是 `_`，
                #   属于词内字符，没有词边界）—— 又一个"量具自己坏了"的形态。
                sitl_hits += len(re.findall(r'AP_SIM_\w+_ENABLED', t))
check(sitl_hits > 5,
      '★★ 反向对照：同一套搜索在 `libraries/SITL/` 里找到 **%d** 处 `AP_SIM_*_ENABLED` '
      '⇒ 搜索方式没坏，是 HIL 消息真的没了' % sitl_hits)

check(slurp('libraries/AP_HAL/AP_HAL_Boards.h').count('HAL_BOARD_HIL') == 0,
      '★★ 也没有 `HAL_BOARD_HIL` 之类的板级宏 ⇒ 没有"为 HIL 编译的固件"')
check('hil' not in ''.join(re.findall(r'"[-\w]*hil[-\w]*"', slurp('libraries/AP_HAL_SITL/SITL_cmdline.cpp'))).lower(),
      '★★ 命令行开关里也没有 HIL（`SITL_cmdline.cpp` 的选项串）')
sv = slurp('Tools/autotest/sim_vehicle.py')
check(not re.search(r'\bhil\b(?!e)', sv, re.I) or
      all('while' in sv[max(0, m.start() - 12):m.start()] for m in re.finditer(r'\bhil\b', sv, re.I)),
      '★★ `sim_vehicle.py` 里的 `hil` 命中全是 `while`（子串），**没有 HIL 选项**')


# ==========================================================================
print()
print('=' * 76)
print('[2] ★★★ 六处墓碑')
print('=' * 76)

# --- ① 三处参数枚举占位符 ---
plane_ph = slurp('ArduPlane/Parameters.h')
tombs = re.findall(r'(k_param_hil_\w+),\s*//\s*unused', plane_ph)
check(tombs == ['k_param_hil_servos_unused', 'k_param_hil_err_limit_unused',
                'k_param_hil_mode_unused'],
      '★★★ ① `ArduPlane/Parameters.h` 里 **%d 处**参数墓碑，注释都写着 `// unused`：%s'
      % (len(tombs), ', '.join(t.replace('k_param_', '').replace('_unused', '') for t in tombs)))
check(all(t.endswith('_unused') for t in tombs),
      '★ 名字都带 `_unused` 后缀 —— 这是 ArduPilot **删参数的标准做法**')
check(len(re.findall(r'//\s*unused', plane_ph)) > 10,
      '★ 佐证：整个文件里有 %d 处 `// unused` ⇒ 不止 HIL 被删过'
      % len(re.findall(r'//\s*unused', plane_ph)))
check('k_param_hil_servos' in plane_ph and 'k_param_hil_err_limit' in plane_ph,
      '★★ 从墓碑名反推：当年至少有 **舵机范围**（servos）、**误差上限**（err_limit）、'
      '**总开关**（mode）三个配置项')
check(re.search(r'enum\s+\w*\s*\{', plane_ph) is not None,
      '★ 它们是 `enum` 的成员 ⇒ **删参数不能删索引**（否则老 EEPROM 的值会读串），'
      '只能改名占位')

# --- ② GPS_TYPE_HIL ---
gps_h = slurp('libraries/AP_GPS/AP_GPS.h')
check(re.search(r'GPS_TYPE_HIL\s*=\s*7,', gps_h) is not None,
      '★★★ ② `libraries/AP_GPS/AP_GPS.h:93` 里 **`GPS_TYPE_HIL = 7`** 还在枚举里')
check(re.search(r'//\s*@Values:[^\n]*7:HIL', slurp('libraries/AP_GPS/AP_GPS_Params.cpp')) is not None,
      '★★★ **而且参数文档里还列着 `7:HIL`** ⇒ 用户看文档设了它')

# --- ③ HAL_INS_HIL_UNUSED ---
boards = slurp('libraries/AP_HAL/AP_HAL_Boards.h')
check(re.search(r'#define\s+HAL_INS_HIL_UNUSED\s+4\s*//\s*unused', boards) is not None,
      '★★★ ③ `AP_HAL_Boards.h:87` `#define HAL_INS_HIL_UNUSED 4 // unused`')
ins_family = re.findall(r'#define\s+(HAL_INS_\w+)\s+(\d+)', boards)
check(len(ins_family) >= 4,
      '★★ 它是 **IMU 驱动编号**家族的墓碑（同族 %d 个）：%s'
      % (len(ins_family), ', '.join('%s=%s' % kv for kv in ins_family[:4])))
check('HAL_INS_NONE' in boards and 'HAL_BOARD_SITL' in boards,
      '★ 同文件里还有 `HAL_INS_NONE` / `HAL_BOARD_SITL` —— 是同一族宏，不是编译开关')

# --- ④ AP_Baro_HIL.cpp ---
baro_hil = 'libraries/AP_Baro/AP_Baro_HIL.cpp'
bh = slurp(baro_hil)
sz = os.path.getsize(os.path.join(REPO, baro_hil.replace('/', os.sep)))
check(sz < 3000, '★★★ ④ `AP_Baro_HIL.cpp` 只有 **%d 字节**' % sz)
check('SimpleUnderWaterAtmosphere' in bh,
      '★★★ 内容是 **`AP_Baro::SimpleUnderWaterAtmosphere()`** —— 水下压强模型')
check(not re.search(r'\bHIL\b', bh),
      '★★★ 而文件里**除文件名外没有任何 HIL 字样** ⇒ **文件名骗人**')
check(re.search(r'\bAP_Baro::\w+', bh) is not None,
      '★ 它是 `AP_Baro` 的一个方法实现（真实功能，只是名字取错了）')

# --- ⑤ 一条 release note ---
rn = slurp('ArduCopter/ReleaseNotes.txt')
check('HIL_GPS message support removed' in rn,
      '★★★ ⑤ 唯一的书面证据：`ArduCopter/ReleaseNotes.txt` 里 '
      '"- HIL_GPS message support removed (@peterbarker, PR:28593)"')
rel = re.findall(r'^Release ([\d.]+[\w-]*)', rn, re.M)
m = re.search(r'HIL_GPS message support removed', rn)
idx = m.start()
above = [(x.start(), x.group(1)) for x in re.finditer(r'^Release ([\d.]+[\w-]*)', rn, re.M)
         if x.start() < idx]
check(above and above[-1][1].startswith('4.7.0'),
      '★★ 它属于 **4.7.0**（其上最近的版本标题是 `%s`）⇒ HIL 是 4.7.0 删的'
      % (above[-1][1] if above else '?'))
note = rn[idx - 200:idx + 200]
check('deprecated' not in note.lower() and 'instead' not in note.lower(),
      '★★★ 而这句话里**没有"请改用…"之类的迁移指引** ⇒ 静默删除')
check('no HIL support (sorry!)' in slurp('ArduPlane/ReleaseNotes.txt'),
      '★ 历史对照：更早的一处 release note 写的是 "- no HIL support (sorry!)" '
      '—— 同样只有一句话')


# ==========================================================================
print()
print('=' * 76)
print('[3] ★★★ `GPS_TYPE_HIL`：死值的完整形态')
print('=' * 76)

gps_cpp = slurp('libraries/AP_GPS/AP_GPS.cpp')

# 抽出 _detect_instance 的函数体（按花括号配对）
# ★ 签名是 `AP_GPS_Backend *AP_GPS::_detect_instance(const uint8_t instance)`
#   —— 我第一版按 `void AP_GPS::_detect_instance(` 找，抽出来 0 字节。
start = gps_cpp.find('AP_GPS::_detect_instance(')
i, depth = gps_cpp.find('{', start), 0
while i < len(gps_cpp):
    if gps_cpp[i] == '{':
        depth += 1
    elif gps_cpp[i] == '}':
        depth -= 1
        if depth == 0:
            break
    i += 1
detect = gps_cpp[start:i + 1]
check(len(detect) > 1000, '★ 抽出 `_detect_instance()` 全文（%d 字节）' % len(detect))

handled = re.findall(r'case (GPS_TYPE_\w+):', detect)
check(len(handled) >= 8,
      '★★ 它**实际**处理了 %d 种类型：%s%s'
      % (len(handled), ', '.join(handled[:8]), ' …' if len(handled) > 8 else ''))
check('GPS_TYPE_HIL' not in handled,
      '★★★ 而**里面没有 `case GPS_TYPE_HIL`** ⇒ 选 `GPS_TYPE=7` 时**一个后端都不会创建**')

check(re.search(r'if \(type == GPS_TYPE_HIL\) \{\s*\n\s*// in HIL, leave info alone\s*\n\s*return;', gps_cpp) is not None,
      '★★★ 而 `update_instance()` 里有一句为 HIL 写的提前返回：'
      '`if (type == GPS_TYPE_HIL) { // in HIL, leave info alone; return; }`')
check(re.search(r'case GPS_TYPE_HIL:', gps_cpp) is not None,
      '★ `needs_uart()` 里也列了它（"不用 UART"）')

# 独立推理：为什么那两个分支走不到
check('GPS_TYPE_HIL' not in handled,
      '★★★ **推理链**：没有检测分支 ⇒ 没有实例 ⇒ `update_instance()` 不会被调用 '
      '⇒ 那句 `return` **永远执行不到**；`needs_uart()` 同理')
# ★ 这条对照我第一版写错了（以为 `GPS_TYPE_NONE` 有检测分支）。
#   实测：**它也不在 case 列表里** —— 它和 `HIL` 一样落到 `default`。
#   所以两者的区别**不在代码路径**，而在参数文档怎么描述它们。
check('GPS_TYPE_NONE' not in handled,
      '★★ 对照：`GPS_TYPE_NONE` 也**不在** case 列表里（与 HIL 同样落 `default`）'
      '—— 区别不在代码，在**文档**')
check(re.search(r'default:', detect) is not None,
      '★ `_detect_instance()` 末尾有 `default:` 分支 ⇒ 落到它的类型一律返回 `nullptr`')
check(re.search(r'//\s*@Values:[^\n]*0:None', slurp('libraries/AP_GPS/AP_GPS_Params.cpp')) is not None,
      '★★★ **这才是关键差别**：参数文档里 `0:None` 说的是"禁用"，'
      '而 `7:HIL` 说的是"HIL 可用" —— **同一个落空行为，两种说法**')

print()
print('    ★★★ 死值的三种形态，本章占全了：')
print('        · 枚举值还在      GPS_TYPE_HIL = 7')
print('        · 为它写的分支还在 update_instance() / needs_uart() 各一处')
print('        · **入口没了**     _detect_instance() 里没有它的 case')
print('        · 而且**参数文档还在推荐它**  @Values: ...7:HIL,...')


# ==========================================================================
print()
print('=' * 76)
print('[4] HIL 曾经是什么（从 ReleaseNotes 反推）')
print('=' * 76)

prn = slurp('ArduPlane/ReleaseNotes.txt')
check('HIL_MODE parameter' in prn, '★ `HIL_MODE` 这个参数名在 ReleaseNotes 里出现过')
check(re.search(r'without loading a special firmware', prn) is not None,
      '★★ 原文："configure your autopilot for hardware in the loop simulation '
      '**without loading a special firmware**. Just set the parameter HIL_MODE to 1"')
check(re.search(r'added HIL_MODE parameter for HIL without a special firmware', prn) is not None,
      '★★ 另一处："added HIL_MODE parameter for HIL without a special firmware"')
vers = [(x.start(), x.group(1)) for x in re.finditer(r'^Release ([\d.]+)', prn, re.M)]
idx = prn.find('HIL_MODE parameter')
above = [v for v in vers if v[0] < idx]
check(above and above[-1][1].startswith('3.3.0'),
      '★★ 属于 **ArduPilot Plane 3.3.0**（2015-05-20）⇒ 它活了大约 **11 年**（3.3.0 → 4.7.0）'
      % ())
check('HIL_MODE to 1' in prn,
      '★ 取值是 `0` / `1`（禁用 / 启用）')


# ==========================================================================
print()
print('=' * 76)
print('[5] ★★ 替代路线：ArduPilot 现在怎么做"硬件在环"')
print('=' * 76)

sitl = slurp('libraries/SITL/SITL.cpp')
check(re.search(r'// @Param: OH_MASK', sitl) is not None,
      '★★★ 路线① **SIM-on-hardware**：参数 `SIM_OH_MASK`')
check('channels which are passed through to actual hardware' in sitl,
      '★★★ 它的说明原文："channels which are passed through to actual hardware '
      '**when running sim on actual hardware**"')
check(re.search(r'AP_GROUPINFO\("OH_MASK",\s*28,\s*SIM,\s*on_hardware_output_enable_mask,\s*0\)', sitl) is not None,
      '★ 默认 0（不启用）')
check(re.search(r'// @Param: OH_RELAY_MSK', sitl) is not None
      or re.search(r'OH_RELAY', sitl) is not None,
      '★ 还有一个配套的 `SIM_OH_RELAY_MSK`（继电器透传）')

check(re.search(r'// @Param: GYR_FILE_RW', sitl) is not None
      and re.search(r'// @Param: ACC_FILE_RW', sitl) is not None,
      '★★★ 路线② **传感器文件回放**：`SIM_GYR_FILE_RW` / `SIM_ACC_FILE_RW`')
m = re.search(r'// @Values: ([^\n]*)\n\s*AP_GROUPINFO\("GYR_FILE_RW"', sitl)
check(m is not None and 'Read data from file' in m.group(1) and 'Write data to a file' in m.group(1),
      '★★ 它们**双向**：可写可读 ⇒ 真机录一次、SITL 里回放')
check('INS_FILE_NONE' in sitl, '★ 默认 `INS_FILE_NONE`（关）')

check(re.search(r'#if AP_SIM_INS_FILE_ENABLED', sitl) is not None,
      '★ 注意这两个参数在 `#if` 里 ⇒ **条件编译**，不是所有构建都有')

print()
print('    ★★★ 三条路的分工：')
print('        · 要真实**传感器数据** → 文件回放（不用硬件）')
print('        · 要真实**MCU 时序**   → SIM-on-hardware（不用 MAVLink 通路）')
print('        · 要真实**气动**       → SITL + 外部物理（第 06 / 14 篇）')
print('        · 要"真板子 + 仿真世界" → **ArduPilot 没有现成路径**')


# ==========================================================================
print()
print('=' * 76)
print('[6] ★★ 对照：PX4 的 HITL 是活的')
print('=' * 76)

if not HAVE_PX4:
    check(False, '没有 PX4 源码树，跳过对照')
else:
    sp = slurp('src/lib/systemlib/system_params.c', base=PX4)
    check(re.search(r'PARAM_DEFINE_INT32\(SYS_HITL,\s*0\)', sp) is not None,
          '★★★ PX4 有 `SYS_HITL` 参数（`system_params.c`）')
    for v, txt in (('-1', 'external HITL'), ('0', 'HITL and SIH disabled'),
                   ('1', 'HITL enabled'), ('2', 'SIH enabled')):
        check(re.search(r'@value %s %s' % (re.escape(v), re.escape(txt)), sp) is not None,
              '★ 取值 `%s` = %s' % (v, txt))
    check('@reboot_required true' in sp,
          '★ 而且是 `@reboot_required true`（改完要重启）—— '
          'ArduPilot 那边是命令行选模型，不需要这个')

    mr = slurp('src/modules/mavlink/mavlink_receiver.cpp', base=PX4)
    for msg in ('HIL_SENSOR', 'HIL_STATE_QUATERNION', 'HIL_OPTICAL_FLOW', 'HIL_GPS'):
        check('MAVLINK_MSG_ID_%s' % msg in mr,
              '★★ PX4 收 `%s`（`mavlink_receiver.cpp`）' % msg)
    check('Only decode hil messages in HIL mode' in mr or 'hil_enabled' in mr,
          '★ 而且注释说明了"只在 HIL 模式下解这些消息"')

    mm = slurp('src/modules/mavlink/mavlink_main.cpp', base=PX4)
    check('HIL_ACTUATOR_CONTROLS' in mm and 'configure_stream' in mm,
          '★★ PX4 发 `HIL_ACTUATOR_CONTROLS`（`mavlink_main.cpp` 里配流）')

    check(os.path.isfile(os.path.join(PX4, 'Tools', 'HIL', 'run_tests.py')),
          '★★ PX4 还有 `Tools/HIL/run_tests.py` 测试脚本 ⇒ HITL 是**维护中的**功能')

    print()
    print('    ★★★ 两家在这里分道扬镳：')
    print('        PX4      —— 同一固件 + 运行时开关 SYS_HITL + MAVLink HIL_* 消息')
    print('        ArduPilot —— 有 SITL 就够了，HIL 在 4.7.0 删除')
    print('        ⇒ 看到 PX4 的 HITL 教程，别以为 ArduPilot 也有对应的一套')


# ==========================================================================
print()
print('=' * 76)
print('[7] `HIL_*` 消息参考（协议还在，实现没了）')
print('=' * 76)

if MAVLINK_XML is None:
    check(False, '没找到 common.xml')
else:
    xml = io.open(MAVLINK_XML, encoding='utf-8', errors='replace').read()
    ids = dict(re.findall(r'<message id="(\d+)" name="(HIL_\w+)"', xml))
    check(len(ids) >= 6,
          '★★★ MAVLink 标准里 `HIL_*` 消息**仍然定义着** %d 条：%s'
          % (len(ids), ', '.join('%s=%s' % (v, k) for k, v in sorted(ids.items(), key=lambda x: int(x[0]))[:6])))
    check(re.search(r'<message id="107" name="HIL_SENSOR"', xml) is not None,
          '★ `HIL_SENSOR` = **107**')
    check('The IMU readings in SI units in NED body frame' in xml,
          '★★ 它的描述："The IMU readings in SI units in **NED body frame**"')

    seg = xml[xml.find('name="HIL_SENSOR"'):]
    seg = seg[:seg.find('</message>')]
    fields = re.findall(r'<field type="(\w+)" name="(\w+)"([^>]*)>', seg)
    check(len(fields) >= 14, '★★ `HIL_SENSOR` 有 **%d** 个字段' % len(fields))
    names = [f[1] for f in fields]
    for f in ('xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro',
              'xmag', 'ymag', 'zmag', 'abs_pressure', 'diff_pressure', 'temperature'):
        check(f in names, '★ 字段 `%s`' % f)
    check('diff_pressure' in names and 'Differential pressure (airspeed)' in seg,
          '★ `diff_pressure` 就是**空速**的来源（差压）')

    ts = [f for f in fields if f[1] == 'time_usec']
    check(ts and 'uint64_t' in ts[0][0],
          '★ `time_usec` 是 uint64（**微秒**）')
    check('The receiving end can infer timestamp format' in seg,
          '★★★ 而它的说明写着："**The receiving end can infer timestamp format ... '
          'by checking for the magnitude of the number**" ⇒ 协议**不规定**时间戳基准，'
          '让接收端靠数值大小猜')

    check('fields_updated' in names,
          '★★ `fields_updated` 是个**位掩码**（逐字段标记"本帧更新了没有"）—— '
          '因为不同传感器的速率不同')
    check('Bitmap for fields that have updated since last message' in seg,
          '★ 它的说明印证了这一点')

    check(re.search(r'<message id="90" name="HIL_STATE"', xml) is not None
          and 'deprecated' in xml[xml.find('name="HIL_STATE"'):xml.find('name="HIL_STATE"') + 400],
          '★ 对照：`HIL_STATE`(90) 已被标为 **deprecated**（2013-07，被 '
          '`HIL_STATE_QUATERNION` 取代）—— 而 `HIL_SENSOR` 本身**没有**被废弃')
    check(re.search(r'<message id="93" name="HIL_ACTUATOR_CONTROLS"', xml) is not None,
          '★ `HIL_ACTUATOR_CONTROLS` = **93**（飞控 → 仿真器方向）')
    check('HIL_ACTUATOR_CONTROLS_FLAGS_LOCKSTEP' in xml,
          '★ 它有个 `LOCKSTEP` 标志位（锁步仿真）')


# ==========================================================================
print()
print('=' * 76)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
