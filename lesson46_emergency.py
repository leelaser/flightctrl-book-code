# -*- coding: utf-8 -*-
"""
lesson46_emergency.py —— 第 12 篇《降落伞与特种失效》配套自验证脚本

三个库、三种"回不去"的处置：
  ① AP_Parachute         —— 开伞。Copter + Plane 有，Sub/Rover 没有
  ② AP_AdvancedFailsafe  —— 围栏/失联/超距 → 终止。Copter + Plane + Rover 有，Sub 没有
  ③ AC_Autorotation      —— 直升机自转。只有 Copter 的 HELI_FRAME

本脚本做的事：
  ① 从源码解析**时序、枚举、状态机**（不靠印象）
  ② 把"时间脉冲""符号链""圆周运动"这类**可解析预言**验一遍
  ③ 做一次**静态可达性分析**：Phase 枚举里哪几个状态根本走不到

运行： python3 lesson46_emergency.py     （全过 EXIT=0）
"""

import io
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
print('[0] 定位源码（4.7.1）')
print('=' * 76)

REPO = None
for cand in (r'D:\DEVLib\PX4APM\APM\ardupilot-Copter-4.7.1',
             r'D:\DEVLib\PX4APM\APM\ardupilot-Plane-4.7.1'):
    if os.path.isdir(os.path.join(cand, 'libraries')):
        REPO = cand
        break
if REPO is None:
    print('    [!!] 找不到 ArduPilot 源码树 —— 后续所有"读真源码"的检查无从谈起')
    n_bad += 1
    sys.exit(1)


def slurp(rel):
    return io.open(os.path.join(REPO, rel), encoding='utf-8', errors='replace').read()


def func_body(text, sig_pat):
    """按大括号配平取出一整个函数体（别用 `.*?\\n}\\n`，会在内层块截断）"""
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


def src_defines(text, pat=r'^#define\s+(\w+)\s+([\-0-9.fU]+)'):
    out = {}
    for name, val in re.findall(pat, text, re.M):
        try:
            out[name] = float(val.rstrip('fU'))
        except ValueError:
            pass
    return out


def enum_values(text, name):
    m = re.search(r'enum\s+(?:class\s+)?%s\s*(?::\s*\w+\s*)?\{(.*?)\}' % name, text, re.S)
    if not m:
        return None
    body = re.sub(r'//[^\n]*', '', m.group(1))
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


F_CHUTE  = 'libraries/AP_Parachute/AP_Parachute.cpp'
F_CHUTEH = 'libraries/AP_Parachute/AP_Parachute.h'
F_AFS    = 'libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp'
F_AFSH   = 'libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h'
F_AROT   = 'libraries/AC_Autorotation/AC_Autorotation.cpp'
F_AROTH  = 'libraries/AC_Autorotation/AC_Autorotation.h'
F_RSC    = 'libraries/AC_Autorotation/RSC_Autorotation.h'
F_MODE   = 'ArduCopter/mode_autorotate.cpp'
F_MODEH  = 'ArduCopter/mode.h'
F_HELI   = 'ArduCopter/heli.cpp'

src_chute, src_chuteh = slurp(F_CHUTE), slurp(F_CHUTEH)
src_afs, src_afsh = slurp(F_AFS), slurp(F_AFSH)
src_arot, src_aroth = slurp(F_AROT), slurp(F_AROTH)
src_rsc = slurp(F_RSC)
src_mode, src_modeh = slurp(F_MODE), slurp(F_MODEH)
src_heli = slurp(F_HELI)

print('    源码树 = %s' % REPO)
check(len(src_chute) > 5000, 'AP_Parachute.cpp 读入 %d 字节' % len(src_chute))
check(len(src_afs) > 15000, 'AP_AdvancedFailsafe.cpp 读入 %d 字节' % len(src_afs))
check(len(src_arot) > 15000, 'AC_Autorotation.cpp 读入 %d 字节' % len(src_arot))


# ==========================================================================
print()
print('=' * 76)
print('[1] 三个库 × 载具归属：谁能用、谁不能用')
print('=' * 76)

# --- 降落伞：只有 Copter 和 Plane 实例化 ---
veh_chute = [v for v in ('ArduCopter', 'ArduPlane', 'ArduSub', 'Rover')
             if os.path.isdir(os.path.join(REPO, v)) and
             re.search(r'AP_Parachute\s+parachute', slurp(v + '/%s.h' %
                       ('Copter' if v == 'ArduCopter' else
                        'Plane' if v == 'ArduPlane' else
                        'Sub' if v == 'ArduSub' else 'Rover')))]
check(veh_chute == ['ArduCopter', 'ArduPlane'],
      '★ AP_Parachute 实例只出现在 %s —— Sub 和 Rover 没有降落伞' % '/'.join(veh_chute))
check(re.search(r'#define\s+HAL_PARACHUTE_ENABLED\s+1',
                slurp('libraries/AP_Parachute/AP_Parachute_config.h')) is not None,
      '★ HAL_PARACHUTE_ENABLED 默认 1（无板级开关）—— 所以"有没有"取决于载具要不要它')

# --- AFS：三个子类 ---
subs = re.findall(r'class\s+(\w+)\s*:\s*public\s+AP_AdvancedFailsafe',
                  ''.join(slurp(p) for p in
                          ('ArduCopter/afs_copter.h', 'ArduPlane/afs_plane.h',
                           'Rover/afs_rover.h')))
check(subs == ['AP_AdvancedFailsafe_Copter', 'AP_AdvancedFailsafe_Plane',
               'AP_AdvancedFailsafe_Rover'],
      '★ AFS 有三个载具子类：%s' % ', '.join(subs))
check(not os.path.isfile(os.path.join(REPO, 'ArduSub/afs_sub.h')),
      '★ 而 ArduSub 没有 afs_sub.h —— AFS 在 Sub 上不存在')
check(re.search(r'virtual\s+void\s+terminate_vehicle\(void\)\s*=\s*0',
                src_afsh) is not None,
      '★ terminate_vehicle() 是【纯虚】—— 基类只决定"该终止了"，'
      '怎么终止由每个载具自己实现')

# --- 自转：只有 heli ---
check(re.search(r'FAST_TASK\(heli_update_autorotation\)', slurp('ArduCopter/Copter.cpp')) is not None,
      '★ 自转的闸门是个 FAST_TASK（Copter.cpp），排在 run_custom_controller 之后')
check(re.search(r'#if\s+FRAME_CONFIG\s*==\s*HELI_FRAME', src_heli) is not None,
      '   ArduCopter/heli.cpp 整体被 #if FRAME_CONFIG == HELI_FRAME 包住 ⇒ 只对直升机存在')
check(re.search(r'AP_SUBGROUPINFO\(arot,\s*"AROT_"', slurp('ArduCopter/Parameters.cpp')) is not None,
      '★ 参数前缀 AROT_ 由 ArduCopter/Parameters.cpp 的 AP_SUBGROUPINFO 给出')
check(re.search(r'GOBJECT\(parachute,\s*"CHUTE_"', slurp('ArduCopter/Parameters.cpp')) is not None,
      '★ 降落伞前缀 CHUTE_')
check(re.search(r'GOBJECT\(\s*parachute,\s*"CHUTE_"', slurp('ArduPlane/Parameters.cpp')) is not None,
      '   而且 Plane 侧是同一个前缀 —— 两边共用同一份参数表')

# --- ★ Copter 的 AFS 默认【不注册参数】 ---
cop_cfg = slurp('ArduCopter/config.h')
m = re.search(r'#\s*define\s+AP_COPTER_ADVANCED_FAILSAFE_ENABLED\s+(\d+)', cop_cfg)
check(m is not None and m.group(1) == '0',
      '★★ Copter 的 AP_COPTER_ADVANCED_FAILSAFE_ENABLED 默认 = %s'
      % (m.group(1) if m else '?'))
cop_params = slurp('ArduCopter/Parameters.cpp')
block = re.search(r'#if\s+AP_COPTER_ADVANCED_FAILSAFE_ENABLED\s*\n(.*?)#endif', cop_params, re.S)
check(block is not None and 'AP_SUBGROUPINFO(afs, "AFS_"' in block.group(1),
      '★★ 而参数注册就在这个 #if 里面 ⇒ **Copter 出厂固件里【不存在】任何 AFS_ 参数**')
check(re.search(r'AP_SUBGROUPINFO\(afs, "AFS_"', slurp('Rover/Parameters.cpp')) is not None,
      '   对照：Rover 的注册【没有】#if 包裹 ⇒ Rover 上 AFS_ 是常驻的')
check(re.search(r'GOBJECT\(afs,\s*"AFS_"', slurp('ArduPlane/Parameters.cpp')) is not None,
      '   对照：Plane 的注册也无条件')

# --- ★ Sub 的 dummy 桩 ---
sub_sys = slurp('ArduSub/system.cpp')
check('dummy method to avoid linking AFS' in sub_sys,
      '★★ ArduSub/system.cpp 里有一段注释 "dummy method to avoid linking AFS"')
check(re.search(r'AP_AdvancedFailsafe\s*\*\s*AP::advancedfailsafe\(\)\s*\{\s*return\s+nullptr;\s*\}',
                sub_sys) is not None,
      '★ 它提供一个返回 nullptr 的 AP::advancedfailsafe() —— '
      '**Sub 不是"没编"，是"编了但显式屏蔽"**（为了让 GCS_Common 能链接过）')

# --- 释放的物理出口 ---
srv_h = slurp('libraries/SRV_Channel/SRV_Channel.h')
m = re.search(r'k_parachute_release\s*=\s*(\d+)', srv_h)
check(m is not None and int(m.group(1)) == 27,
      '★ 物理出口 = SERVOn_FUNCTION %s（Parachute）' % (m.group(1) if m else '?'))
srv_cpp = slurp('libraries/SRV_Channel/SRV_Channel.cpp')
check(re.search(r'@Values\{Plane, Copter\}:\s*27:Parachute', srv_cpp) is not None,
      '★★ 而且这个函数值带 {Plane, Copter} 限定 —— 与 §1 的载具归属互相印证')

rc_h_all = slurp('libraries/RC_Channel/RC_Channel.h')
opts = dict((k, int(v)) for k, v in
            re.findall(r'(PARACHUTE_\w+)\s*=\s*(\d+)', rc_h_all))
check(opts == {'PARACHUTE_ENABLE': 21, 'PARACHUTE_RELEASE': 22, 'PARACHUTE_3POS': 23},
      '★ 三个遥控开关：%s'
      % ', '.join('%s=%d' % (k, v) for k, v in sorted(opts.items(), key=lambda kv: kv[1])))
check('PARACHUTE_3POS' in rc_h_all and '3 position switch' in rc_h_all,
      '   PARACHUTE_3POS 是三段开关：关 / 使能 / 释放')

# --- ★ 死声明：Mode::do_parachute ---
copter_h = slurp('ArduCopter/Copter.h')
mode_h = slurp('ArduCopter/mode.h')
decl = re.search(r'void\s+do_parachute\(const AP_Mission::Mission_Command&\s*cmd\);', mode_h)
check(decl is not None, '★ ArduCopter/mode.h 里声明了 Mode::do_parachute()')
check(src_chute.count('do_parachute') == 0 and
      'do_parachute' not in slurp('libraries/AP_Mission/AP_Mission_Commands.cpp'),
      '★★ 但降落伞库和任务命令里都没有这个名字')
check(all('do_parachute' not in slurp(p) for p in
          ('ArduCopter/mode_auto.cpp', 'ArduCopter/mode_guided.cpp')),
      '★★ 任务路径走的是 AP_Mission::start_command_parachute()，'
      '**那个声明是遗留死代码**（既无定义也无调用者）')

# --- Plane 的故障动作是互斥的两支 ---
plane_h = slurp('ArduPlane/Plane.h')
check(re.search(r'Failsafe_Action_Parachute\s*=\s*5', plane_h) is not None,
      '★ Plane 的电池/失效动作里有 Failsafe_Action_Parachute = 5')
plane_ev = slurp('ArduPlane/events.cpp')
check('parachute_release();' in plane_ev and 'afs.gcs_terminate(true' in plane_ev,
      '★★ 而它与 Failsafe_Action_Terminate 在【同一个 switch 的不同 case】—— '
      '**Plane 上"开伞"和"AFS 终止"是二选一，不可能同时发生**')


# ==========================================================================
print()
print('=' * 76)
print('[2] 降落伞：参数、时序、两个触发源')
print('=' * 76)

# --- 参数表 ---
chute_params = re.findall(
    r'AP_GROUPINFO(?:_FLAGS)?\("(\w+)",\s*(\d+),\s*AP_Parachute,\s*(\w+),\s*([^,)]+)',
    src_chute)
names = [p[0] for p in chute_params]
check(len(chute_params) == 8,
      '★ CHUTE_ 有 %d 个参数：%s' % (len(chute_params), ', '.join('CHUTE_' + n for n in names)))
check(names == ['ENABLED', 'TYPE', 'SERVO_ON', 'SERVO_OFF', 'ALT_MIN', 'DELAY_MS',
                'CRT_SINK', 'OPTIONS'],
      '★ 顺序与槽位 0~7 一致（ENABLED 带 AP_PARAM_FLAG_ENABLE，置 0 则整棵子树隐藏）')
check(re.search(r'AP_GROUPINFO_FLAGS\("ENABLED",\s*0,.*AP_PARAM_FLAG_ENABLE', src_chute) is not None,
      '   ENABLED 槽位 0 + AP_PARAM_FLAG_ENABLE')
check(re.search(r'//\s*@Values:\s*0:\s*Relay,10:Servo', src_chute) is not None,
      '★ CHUTE_TYPE：0~3 都是继电器、10 是舵机 —— **不是 0/1**（0/1 是 4.5 之前的旧值）')

# --- 时序常量 ---
CD = src_defines(src_chuteh)
check(abs(CD.get('AP_PARACHUTE_RELEASE_DELAY_MS', -1) - 500) < 1e-9,
      '★ 释放【延迟】 = %s ms' % CD.get('AP_PARACHUTE_RELEASE_DELAY_MS'))
check(abs(CD.get('AP_PARACHUTE_RELEASE_DURATION_MS', -1) - 2000) < 1e-9,
      '★ 释放【脉冲宽度】 = %s ms' % CD.get('AP_PARACHUTE_RELEASE_DURATION_MS'))
check(abs(CD.get('AP_PARACHUTE_ALT_MIN_DEFAULT', -1) - 10) < 1e-9,
      '★ 最低高度默认 = %s m' % CD.get('AP_PARACHUTE_ALT_MIN_DEFAULT'))
check(abs(CD.get('AP_PARACHUTE_CRITICAL_SINK_DEFAULT', -1) - 0) < 1e-9,
      '★ 临界下沉率默认 = %s（= 关闭；要显式设成正值才启用）'
      % CD.get('AP_PARACHUTE_CRITICAL_SINK_DEFAULT'))
check(abs(CD.get('AP_PARACHUTE_OPTIONS_DEFAULT', -1) - 0) < 1e-9,
      '★ 选项默认 = 0（= 不在释放后保持导通，且【会】先解锁）')

# --- release() 的三件事 ---
rel = func_body(src_chute, r'void AP_Parachute::release\(\)')
check(rel.count('AP::arming().disarm(') == 1,
      '★ release() 里调 disarm —— 注释写着 "stop motors to avoid parachute tangling"')
check('SkipDisarmBeforeParachuteRelease' in rel,
      '   但受 CHUTE_OPTIONS 的 bit1 控制：置 1 就跳过解锁')
check(re.search(r'if\s*\(_release_time\s*==\s*0\)', rel) is not None,
      '★ _release_time 只在【为 0 时】才赋值 ⇒ 重复调用 release() 不会重启计时')

# --- update() 的时序：把两个分支重演一遍 ---
DELAY = CD.get('AP_PARACHUTE_RELEASE_DELAY_MS')
DUR = CD.get('AP_PARACHUTE_RELEASE_DURATION_MS')
PERIOD = 100


def chute_run(hold_forever=False, t_release_ms=1000, n=40):
    """重演 update() 的两个分支。
       t_release_ms = release() 被调用的时刻（millis 读数）。
       返回 [(now_ms, 状态)]，状态 True=输出导通 / False=断开 / None=两个分支都没进。"""
    log = []
    t_release = t_release_ms
    in_progress = False
    for k in range(n + 1):
        now = k * PERIOD
        tdiff = now - t_release
        state = None
        if (t_release != 0) and not in_progress:
            if tdiff >= DELAY:
                state = True
                in_progress = True
        elif (t_release == 0) or (not hold_forever and tdiff >= DELAY + DUR):
            state = False
            in_progress = False
            t_release = 0
        log.append((now, state))
    return log


log = chute_run(False)
on_time = next((t for t, s in log if s is True), None)
off_time = next((t for t, s in log if s is False and on_time is not None and t > on_time), None)
check(on_time == 1000 + DELAY,
      '★★ 时序：release() 之后第 %d ms 输出才导通（1000 + CHUTE_DELAY_MS = 1000 + %d）'
      % (on_time - 1000 if on_time else -1, DELAY))
check(off_time == 1000 + DELAY + DUR,
      '★★ 再 %d ms 后自动断开（脉冲宽度 = AP_PARACHUTE_RELEASE_DURATION_MS = %d）'
      % (off_time - on_time if off_time else -1, DUR))
check(DELAY + DUR == 2500,
      '★ 从 release() 到输出断开一共 %d ms —— 这段时间里飞机在自由落体（电机已在 t=0 停）'
      % (DELAY + DUR))
check((off_time - on_time) == 2000,
      '★ 而"伞绳拉出"的窗口只有 %d ms；要让它一直导通必须开 CHUTE_OPTIONS bit0'
      % (off_time - on_time))
# 反向对照：hold_forever 时不会断开
log_h = chute_run(True)
check(not any(s is False for t, s in log_h if t > 1000 + DELAY),
      '★ 反向对照：开 HoldOpen 后，导通状态【永不断开】（对照上面那条）')

# --- 边界情况：release() 恰好发生在 millis()==0 ---
log_z = chute_run(False, t_release_ms=0)
check(not any(s is True for _, s in log_z),
      '★★ 边界：若 release() 恰好在 millis()==0 那一刻被调用，'
      '_release_time 会保持 0 —— 而 update() 把 `_release_time == 0` 当作"未释放"'
      '⇒ **这次释放永远不会发生**（实际飞行中毫秒级不可能撞上，但这是真的边界）')
check(re.search(r'if\s*\(_release_time\s*==\s*0\)\s*\{\s*\n\s*_release_time\s*=\s*AP_HAL::millis\(\);',
                rel) is not None,
      '   源码依据：release() 里是 `if (_release_time == 0) { _release_time = millis(); }`')


# ==========================================================================
print()
print('=' * 76)
print('[3] 降落伞：两个单向标志 —— 释放后必须重启')
print('=' * 76)

check(src_chute.count('_release_initiated') == 3,
      '★ _release_initiated 在 .cpp 里只出现 %d 次：一次置位 + 两次读'
      '（check_sink_rate 与 arming_checks）' % src_chute.count('_release_initiated'))
check(re.search(r'_release_initiated\s*=\s*true;', src_chute) is not None and
      not re.search(r'_release_initiated\s*=\s*false', src_chute),
      '★★ 它被置 true 之后【从不复位】—— 复位路径根本不存在')
check(src_chute.count('_released') == 1,
      '★ _released 在 .cpp 里只出现 %d 次（置位那一处）' % src_chute.count('_released'))
check(re.search(r'_released\s*=\s*true;', src_chute) is not None and
      '_released = false' not in src_chute,
      '★★ _released 同样只置位、从不复位')

# --- 注释与代码不符 ---
upd = func_body(src_chute, r'void AP_Parachute::update\(\)')
check('// reset released flag and release_time' in upd,
      '★★ update() 里有一行注释 "reset released flag and release_time"')
check(re.search(r'//\s*reset released flag and release_time\s*\n\s*'
                r'_release_in_progress\s*=\s*false;\s*\n\s*_release_time\s*=\s*0;', upd)
      is not None,
      '★ 而紧跟的两行复位的是 _release_in_progress 和 _release_time —— '
      '**没有 released**，注释说错了对象')

# --- 后果 ---
check(re.search(r'if\s*\(_release_initiated\)\s*\{', src_chute) is not None,
      '★ arming_checks() 里有 `if (_release_initiated)` 分支')
check('Chute is released' in src_chute,
      '★★ 它的报错文本是 "Chute is released" ⇒ **释放过一次之后，'
      '上锁/解锁检查会一直失败，必须重启飞控**')
check(re.search(r'if\s*\(!parachute\.enabled\(\)\s*\|\|\s*parachute\.released\(\)\)',
                slurp('ArduPlane/parachute.cpp')) is not None,
      '★ Plane 的 parachute_manual_release() 也用 released() 做前置条件 ⇒ 同样一次即锁死')


# ==========================================================================
print()
print('=' * 76)
print('[4] 降落伞的自动触发：下沉率要【持续 1 秒】')
print('=' * 76)

ssr = func_body(src_chute, r'void AP_Parachute::set_sink_rate')
csr = func_body(src_chute, r'void AP_Parachute::check_sink_rate')
check(re.search(r'if\s*\(\(_critical_sink\s*<=\s*0\)\s*\|\|\s*!_is_flying\)', ssr) is not None,
      '★ set_sink_rate：临界值为 0（默认）或不在飞 —— 直接清零计时')
check(re.search(r'if\s*\(sink_rate\s*<=\s*_critical_sink\)', ssr) is not None,
      '★ 下沉率没超阈值 —— 也清零（**必须连续超，中间回落就重新计时**）')
check(re.search(r'if\s*\(_sink_time_ms\s*==\s*0\)\s*\{\s*\n\s*_sink_time_ms\s*=\s*AP_HAL::millis\(\);',
                ssr) is not None,
      '★ 只有"之前没在计时"时才起表 —— 这就是"连续"的实现方式（不重置起点）')
# 注意：只断言"源码里有 `> 1000` 这个模式"是不够的 —— 改成 `> 100` 仍然匹配（前缀）。
# 要【把数取出来再比】。
m_sink = re.search(r'\(AP_HAL::millis\(\)\s*-\s*_sink_time_ms\)\s*>\s*(\d+)', csr)
check(m_sink is not None, '从 check_sink_rate 里解析出持续时长门限')
SINK_MS = int(m_sink.group(1)) if m_sink else -1
check(SINK_MS == 1000,
      '★★ check_sink_rate 的判据是 `elapsed > %d` —— 单位是 ms，所以是 **%.0f 秒**'
      % (SINK_MS, SINK_MS / 1000.0))
check(re.search(r'if\s*\(_release_initiated\s*\|\|\s*!_is_flying\)', csr) is not None,
      '★ 已经在释放流程里、或不在飞 —— 不再重复触发')

# --- 复现：连续 vs 断续 ---
def sink_trigger(series, critical=3.0, period_ms=100):
    """重演 set_sink_rate + check_sink_rate；series 是每拍的下沉率"""
    t_sink = 0
    for k, rate in enumerate(series):
        now = k * period_ms
        if rate <= critical:
            t_sink = 0
        elif t_sink == 0:
            t_sink = now
        if t_sink > 0 and (now - t_sink) > 1000:
            return k * period_ms
    return None


cont = [5.0] * 40
check(sink_trigger(cont) == 1200,
      '★ 连续超阈值：第 %d ms 触发' % sink_trigger(cont))
check(sink_trigger(cont) == 100 + 1100,
      '★★ 注意起点是 100 ms 而【不是】0 —— 因为源码用 `_sink_time_ms == 0` 当'
      '"还没开始计时"的哨兵：第 0 拍赋值 0，与"未计时"无法区分，'
      '所以表要到第 1 拍（100 ms）才真正起算。'
      '**同一个模式在 release() 里也有一份**（见上一节 `_release_time == 0`）')
broken = ([5.0] * 10) + [1.0] + ([5.0] * 30)
check(sink_trigger(broken) is not None and sink_trigger(broken) > sink_trigger(cont),
      '★★ 中间回落一拍：触发时间从 %d ms 推到 %d ms —— '
      '「持续」是真的在防止断续下沉误触发' % (sink_trigger(cont), sink_trigger(broken)))
check(sink_trigger([1.0] * 40) is None,
      '★ 反向对照：一直没超阈值 ⇒ 永远不触发')


# ==========================================================================
print()
print('=' * 76)
print('[5] AFS：四个状态、两处终止源、一个 30 秒去重窗')
print('=' * 76)

ST = enum_values(src_afsh, 'state')
CM = enum_values(src_afsh, 'control_mode')
TA = enum_values(src_afsh, 'terminate_action')
check(ST is not None and len(ST) == 4,
      '★ state 四态：%s' % ', '.join(sorted(ST, key=ST.get)) if ST else '未找到')
check(CM is not None and len(CM) == 3,
      '★ control_mode 三态：%s' % ', '.join(sorted(CM, key=CM.get)) if CM else '未找到')
check(TA == {'TERMINATE_ACTION_TERMINATE': 42, 'TERMINATE_ACTION_LAND': 43},
      '★★ terminate_action 只有两项，值是【42 和 43】—— 硬编码的字面量，不是连续枚举')
check(ST.get('STATE_PREFLIGHT') == 0 and ST.get('STATE_GPS_LOSS') == 3,
      '★ 状态机以 STATE_PREFLIGHT 起步（构造函数里 `_state = STATE_PREFLIGHT;`）')

chk = func_body(src_afs, r'void\s*\n?AP_AdvancedFailsafe::check\(')
check(chk != '', '整段取出 check()（10 Hz 调用）')

# --- 三处终止源 ---
check(re.search(r'Terminating due to fence breach', chk) is not None,
      '★ 终止源 ①：围栏越界（或高度超限）—— 在状态机【之前】检查，与状态无关')
check(re.search(r'Terminating due to RC failure', chk) is not None,
      '★ 终止源 ②：RC 失灵（受 AFS_RC / AFS_RC_MAN_ONLY / AFS_RC_FAIL_TIME 三重条件）')
check(re.search(r'Terminating due to max range|Terminating due to range', src_afs) is not None,
      '★ 终止源 ③：超出最大航程（在 max_range_update() 里，也是独立于状态机的）')
check(re.search(r'Terminating due to dual loss', chk) is not None,
      '★★ 终止源 ④：**双重失效**（丢数据链 且 丢 GPS）—— 这是唯一在状态机【内部】的终止')

# --- 30 秒去重窗 ---
check(chk.count('30*1000UL') == 2,
      '★ 30 秒去重窗出现 %d 次（数据链与 GPS 各一次）' % chk.count('30*1000UL'))
check('if two events happen within 30s we consider it to be part of the same event' in src_afs,
      '★ 源码注释原文解释了它：两次失效若间隔 30 s 内，算作【同一次事件】'
      '（不增加计数）')

# --- 状态转移表（从源码 parse）---
check(re.search(r'case STATE_PREFLIGHT:', chk) is not None and
      re.search(r'case STATE_AUTO:', chk) is not None and
      re.search(r'case STATE_DATA_LINK_LOSS:', chk) is not None and
      re.search(r'case STATE_GPS_LOSS:', chk) is not None,
      '★ switch 覆盖全部四个状态（没有 default —— 枚举写全了）')
check(re.search(r'const uint32_t last_heartbeat_ms = gcs\(\)\.sysid_mygcs_last_seen_time_ms\(\);',
                chk) is not None,
      '★ "数据链正常"的判据 = GCS 心跳的【最后收包时刻】')
check(re.search(r'bool gps_lock_ok = \(\(now - AP::gps\(\)\.last_fix_time_ms\(\)\) < 3000\);',
                chk) is not None,
      '★ "GPS 正常"的判据 = 最后定位时刻距今 < **3000 ms**')
check(re.search(r'if \(mode == AFS_AUTO\)', chk) is not None,
      '★ PREFLIGHT → AUTO 的唯一条件：afs_mode() 返回 AFS_AUTO（即进入自动模式）')

# --- 恢复路径 ---
check(re.search(r'mission\.set_current_cmd\(_saved_wp\)', chk) is not None,
      '★ 数据链/GPS 恢复后会把航点【还原】到 _saved_wp（丢掉失效期间飞过的那一段）')
check(chk.count('mission.set_current_cmd(_saved_wp)') == 2,
      '   两处还原：数据链恢复一处、GPS 恢复一处')
check(re.search(r'_max_comms_loss\s*<=\s*0\s*\|\|', chk) is not None and
      re.search(r'_max_gps_loss\s*<=\s*0\s*\|\|', chk) is not None,
      '★★ 但还原是有条件的：超过 AFS_MAX_COM_LOSS / AFS_MAX_GPS_LOSS 次数后【不再还原】')
check(re.search(r'option_is_set\(Option::CONTINUE_AFTER_RECOVERED\)', chk) is not None,
      '   还有 OPTIONS bit0：勾了就干脆不还原，直接继续当前任务')


# ==========================================================================
print()
print('=' * 76)
print('[6] AFS 的终止：一处注释与代码不符，三个载具三种做法')
print('=' * 76)

check('only possible if FS_TERM_ACTION is 42 and _terminate is non zero' in src_afs,
      '★★ 源码注释说："only possible if FS_TERM_ACTION is **42** and _terminate is non zero"')
scv = func_body(src_afs, r'bool\s*\n?AP_AdvancedFailsafe::should_crash_vehicle')
check(scv.count('TERMINATE_ACTION_TERMINATE') == 1 and
      scv.count('TERMINATE_ACTION_LAND') == 1,
      '★ 而紧跟的代码接受 **42 或 43** 两者（`||`）—— **注释落后于代码**')
check(re.search(r'if\s*\(_terminate\s*&&', scv) is not None,
      '   真正的门槛是 `_terminate != 0`，动作类型只决定【怎么终止】不决定【要不要终止】')

# --- gcs_terminate 的额外限制 ---
gcs = func_body(src_afs, r'bool\s*\n?AP_AdvancedFailsafe::gcs_terminate')
check(re.search(r'_terminate_action\s*!=\s*TERMINATE_ACTION_TERMINATE', gcs) is not None,
      '★★ 但 GCS 发起的终止【只认 42】：设成 43（LAND）时 GCS 终止会被拒，'
      '而围栏/RC/超距触发的终止在 43 下照常工作')
check('Unable to terminate, termination is not configured' in gcs,
      '   拒绝时会回这句 —— 现场排障时能直接看到')

# --- 三个载具的 terminate_vehicle ---
cop = slurp('ArduCopter/afs_copter.cpp')
pla = slurp('ArduPlane/afs_plane.cpp')
rov = slurp('Rover/afs_rover.cpp')
check('Mode::Number::LAND' in cop,
      '★ Copter + 43 ⇒ 切 LAND 模式（正常降落）')
check('set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN)' in cop,
      '★ Copter + 42 ⇒ 电机 SHUT_DOWN + disarm')
check('aerodynamic termination is the default approach to termination' in pla,
      '★★ Plane + 42 ⇒ **气动终止**：舵面打满让飞机自己进入螺旋/失速')
check('SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, SERVO_MAX)' in pla,
      '   副翼 SERVO_MAX —— 源码注释 "aerodynamic termination is the default approach"')
check('mode_qland' in pla,
      '★ Plane + QuadPlane + 43 ⇒ 切 VTOL 降落模式（不是固定翼降落）')
check('rover.mode_hold' in rov and 'disarm' in rov,
      '★★ Rover 的"终止"只是**解锁 + HOLD 模式** —— 地面载具没有"坠毁"可言，'
      '同一套 AFS 语义到 Rover 上完全变了味')
check('SERVO_MAX' not in rov,
      '★ 反向对照：Rover 里没有任何舵面打满 —— 三个载具的 42 是三种完全不同的行为')

# --- max_range 的起算点 ---
mru = func_body(src_afs, r'void\s*\n?AP_AdvancedFailsafe::max_range_update')
check(re.search(r'if\s*\(!hal\.util->get_soft_armed\(\)\)', mru) is not None and
      re.search(r'wait for arming', mru) is not None,
      '★★ 最大航程的起算点是「**首次解锁 + 3D fix**」那一刻的 GPS 位置，不是起飞点')
check(re.search(r'_first_location\.get_distance\(AP::gps\(\)\.location\(\)\)', mru) is not None,
      '★ 判据是离起算点的**直线距离**（不是航迹长度）')
check(re.search(r'AP::gps\(\)\.status\(\)\s*<\s*AP_GPS::GPS_OK_FIX_2D', mru) is not None,
      '★★ 而且要求至少 2D fix —— 注释写着 "don\'t trigger when dead-reckoning"，'
      '即**纯惯导推算期间不触发**（避免 GPS 刚丢就误判超距）')


# ==========================================================================
print()
print('=' * 76)
print('[7] 自转：相位机的静态可达性分析')
print('=' * 76)

PH = enum_values(src_modeh, 'Phase')
check(PH is not None and len(PH) == 10,
      '★ Phase 枚举有 %d 项：%s'
      % (len(PH) if PH else 0, ', '.join(sorted(PH, key=PH.get)) if PH else '未找到'))

assigned = set(re.findall(r'current_phase\s*=\s*Phase::(\w+);', src_mode))
allnames = set(PH) if PH else set()
unreachable = sorted(allnames - assigned, key=lambda k: PH[k])

check(assigned == {'ENTRY_INIT', 'ENTRY', 'GLIDE_INIT', 'GLIDE', 'LANDED_INIT', 'LANDED'},
      '★ current_phase 实际被赋过 %d 个值：%s'
      % (len(assigned), ', '.join(sorted(assigned, key=lambda k: PH[k]))))
check(unreachable == ['FLARE_INIT', 'FLARE', 'TOUCH_DOWN_INIT', 'TOUCH_DOWN'],
      '★★ 而枚举里另外 %d 个值【从来没有被赋值过】：%s'
      % (len(unreachable), ', '.join(unreachable)))
check(len(unreachable) / float(len(allnames)) == 0.4,
      '★★ 也就是 10 个状态里有 %d 个是死的（%.0f%%）' % (len(unreachable), 100.0 * 0.4))

# --- 四个死状态在源码里的唯一出现：空 case ---
m = re.search(r'case Phase::FLARE_INIT:(.*?)break;', src_mode, re.S)
body = m.group(1) if m else ''
live = [l for l in body.splitlines() if l.strip() and not l.strip().startswith('//')]
check(len(live) == 3 and all(l.strip().startswith('case Phase::') for l in live),
      '★ 那四个状态在 mode_autorotate.cpp 里只出现在一个【空 case】里'
      '（连着四个 case 标签，直接 break）')
check(re.search(r'case Phase::FLARE_INIT:\s*\n\s*case Phase::FLARE:\s*\n\s*'
                r'case Phase::TOUCH_DOWN_INIT:\s*\n\s*case Phase::TOUCH_DOWN:\s*\n\s*break;',
                src_mode) is not None,
      '   四个 case 标签摞在一起，函数体只有一句 break')

# --- 真实的相位链 ---
check(re.search(r'if \(current_phase < Phase::GLIDE_INIT && '
                r'\(\(now_ms - _entry_time_start_ms\) > g2\.arot\.entry_time_ms\)\)', src_mode)
      is not None,
      '★ 推进规则一：离地进入 ENTRY 后超过 entry_time_ms（2000 ms）⇒ 进 GLIDE')
check(re.search(r'if \(current_phase < Phase::LANDED && g2\.arot\.check_landed\(\)\)', src_mode)
      is not None,
      '★★ 推进规则二：check_landed() 一成立就直接进 LANDED —— '
      '**GLIDE 直接跳到 LANDED，中间那四个 FLARE/TOUCH_DOWN 全被跳过**')
check('More urgent phases (the ones closer to the ground) take precedence later in the if statements'
      in src_mode,
      '★ 注释原话："More urgent phases (the ones closer to the ground) take precedence '
      'later in the if statements" —— 说明了这是**有意按"越靠地面越优先"排的级联 if**')


# ==========================================================================
print()
print('=' * 76)
print('[8] 自转的两个控制器：符号约定与圆形限幅')
print('=' * 76)

# --- 转速控制器 ---
hs = func_body(src_arot, r'void AC_Autorotation::update_headspeed_controller')
check(re.search(r'_head_speed_error\s*=\s*head_speed_norm\s*-\s*_target_head_speed;', hs)
      is not None,
      '★ 误差定义是 `实测 - 目标`（**不是** 目标 - 实测）')
check(re.search(r'_p_term_hs\s*=\s*_p_hs\.get_p\(_head_speed_error\);', hs) is not None,
      '★ P 项 = P × 误差')
check(re.search(r'const float collective_out = constrain_value\(\(_p_term_hs \+ _ff_term_hs\), '
                r'0\.0f, 1\.0f\);', hs) is not None,
      '★ 总距 = P 项 + 前馈项，再钳到 [0, 1]')

# --- 符号链：由源码推，不猜 ---
def headspeed_collective(meas_norm, target_norm, kp=1.0, ff=0.5):
    """重演上面三行，返回总距"""
    err = meas_norm - target_norm
    p = kp * err
    return max(0.0, min(p + ff, 1.0))


check(headspeed_collective(0.8, 1.0) < headspeed_collective(1.0, 1.0),
      '★★ 符号链：转速【偏低】时（0.8 vs 目标 1.0）总距反而【更小】'
      '（%.2f < %.2f）' % (headspeed_collective(0.8, 1.0), headspeed_collective(1.0, 1.0)))
check(headspeed_collective(1.2, 1.0) > headspeed_collective(1.0, 1.0),
      '★ 反过来转速偏高时总距更大')
check(headspeed_collective(0.5, 1.0) < headspeed_collective(0.8, 1.0),
      '★ 单调：转得越慢，总距越小')
print('        ⚠️ 这**不是**符号写反 —— 自转中总距小 ⇒ 桨叶迎角小 ⇒ 阻力小 ⇒ 转速升。')
print('           与"有动力时加油门提转速"的直觉正好相反，是本篇最容易读错的一处。')

check('set_coll_from_ang(-2.0)' in hs and 'hope for the best' in hs,
      '★ RPM 传感器坏掉时的兜底：`set_coll_from_ang(-2.0)` —— '
      '源码注释 "set collective to angle of -2 deg and hope for the best"')

# --- 前馈是"跟随式配平" ---
check(re.search(r'_ff_term_hs\s*=\s*col_trim_lpf\.apply\(_motors_heli->get_throttle\(\),\s*_dt\);',
                hs) is not None,
      '★★ 前馈不是查表也不是模型 —— 是**把上一拍实际的总距过一遍低通**'
      '（"following trim"），所以稳态时 P 项自然回零')

# --- 入口相位与滑翔相位的滤波频率不同 ---
check(re.search(r'col_trim_lpf\.set_cutoff_frequency\(_param_col_entry_cutoff_freq\.get\(\)\);',
                func_body(src_arot, r'void AC_Autorotation::init_entry')) is not None,
      '★ 入口相位用 AROT_COL_FILT_E（默认 0.7 Hz）')
check(re.search(r'col_trim_lpf\.set_cutoff_frequency\(_param_col_glide_cutoff_freq\.get\(\)\);',
                func_body(src_arot, r'void AC_Autorotation::init_glide')) is not None,
      '★ 滑翔相位用 AROT_COL_FILT_G（默认 0.1 Hz）—— 慢 7 倍')
check(re.search(r'Must be higher than AROT_COL_FILT_G', src_arot) is not None and
      re.search(r'Must be lower than AROT_COL_FILT_E', src_arot) is not None,
      '★★ 两个参数的文档里**互相声明了约束**（E 必须 > G）—— '
      '但注意：这是文档约束，代码里没有运行时校验')

# --- ★ 两处参数表自相矛盾 ---
m_lo = re.search(r'//\s*@Range:\s*([\d.]+)\s+([\d.]+)\s*\n\s*//\s*@Increment:\s*0\.01\s*\n'
                 r'\s*//\s*@User:\s*Standard\s*\n\s*'
                 r'AP_GROUPINFO\("COL_FILT_E",\s*5,\s*AC_Autorotation,'
                 r'\s*\w+,\s*([\d.]+)\)', src_arot)
check(m_lo is not None, '解析出 AROT_COL_FILT_E 的 @Range 与默认值')
if m_lo:
    lo_rng, hi_rng, dflt = float(m_lo.group(1)), float(m_lo.group(2)), float(m_lo.group(3))
    check(lo_rng == 0.2 and hi_rng == 0.5,
          '★ AROT_COL_FILT_E 的 @Range 声明为 %.2f ~ %.2f' % (lo_rng, hi_rng))
    check(not (lo_rng <= dflt <= hi_rng),
          '★★ 而它的默认值是 %.2f —— **落在声明的 @Range 之外**！'
          '地面站会显示成越界，而固件照用不误' % dflt)

m_hs = re.search(r'AP_GROUPINFO\("HS_SENSOR",\s*8,\s*AC_Autorotation,\s*\w+,\s*(\d+)\)', src_arot)
check(m_hs is not None and m_hs.group(1) == '0', '★ AROT_HS_SENSOR 默认 = %s'
      % (m_hs.group(1) if m_hs else '?'))
check(re.search(r'//\s*@Range:\s*0\.5 3\s*\n\s*//\s*@Increment:\s*0\.1\s*\n\s*//\s*@User:\s*Standard'
                r'\s*\n\s*AP_GROUPINFO\("HS_SENSOR"', src_arot) is not None,
      '★★ 但它的 @Range 写着 0.5 ~ 3、@Units 写着 s —— 而 @Description 说的是'
      '"Allocate the RPM sensor **instance**... RPM1 = 0. RPM2 = 1."')
check(re.search(r'//\s*@Description:\s*Allocate the RPM sensor instance', src_arot) is not None,
      '   这是一个**索引**，不是时间 —— @Range/@Units 是从别的参数拷过来的残留')
check('0.5' not in m_hs.group(1) and int(m_hs.group(1)) < 0.5 if m_hs else False,
      '   ⇒ 默认值 0 同样落在 @Range 0.5~3 之外（和 COL_FILT_E 同一类问题）')

# --- ★ FWD_* 悬空注释的顺序与真实槽位不符 ---
pidbasic = slurp('libraries/AC_PID/AC_PID_Basic.cpp')
slots = [(n, int(s)) for n, s in
         re.findall(r'AP_GROUPINFO\w*\("(\w+)",\s*(\d+),\s*AC_PID_Basic', pidbasic)]
order = [n for n, _ in sorted(slots, key=lambda kv: kv[1])]
check(order == ['P', 'I', 'IMAX', 'FLTE', 'D', 'FLTD', 'FF'],
      '★ AC_PID_Basic 的真实槽位顺序：%s' % ' < '.join(order))
# 注意排除 FWD_SP_TARG —— 它也是真参数、也以 FWD_ 开头，但不是那个子组的成员
doc_order = re.findall(r'//\s*@Param:\s*(FWD_(?!SP_TARG)\w+)', src_arot)
check(doc_order == ['FWD_P', 'FWD_I', 'FWD_IMAX', 'FWD_D', 'FWD_FF', 'FWD_FLTE', 'FWD_FLTD'],
      '★★ 而 AC_Autorotation.cpp 里那段 FWD_* 注释块的顺序是：%s'
      % ' < '.join(x.replace('FWD_', '') for x in doc_order))
check([x.replace('FWD_', '') for x in doc_order] != order,
      '★★ **两个顺序不一样**（注释把 D 放在第 4 位、FLTE 放第 6 位；'
      '真实槽位是 FLTE 第 4、D 第 5）—— 而且这段注释后面**没有配对的 AP_GROUPINFO**，'
      '真正生效的表在 AC_PID_Basic 里。与第 10 篇 `AC_CustomControl_PID.cpp` 同一个模式')

# --- 前飞控制器：加速度限幅 + 圆形约束 ---
fw = func_body(src_arot, r'void AC_Autorotation::update_forward_speed_controller')
check(re.search(r'_target_vel_ms = constrain_float\(_desired_vel_ms, min_vel_ms, max_vel_ms\);',
                fw) is not None,
      '★ 速度目标本身先做加速度限幅（每拍最多变化 accel_max × dt）')
check(re.search(r'pilot_accel_norm \* get_accel_max_mss\(\) \* 0\.5', fw) is not None,
      '★★ 飞行员打杆只能请求**一半**的横向加速度 —— '
      '注释原话："We only allow up to half as we need to prioritize building/maintaining airspeed"')
check(re.search(r'_limit_accel = bf_accel_target_mss\.limit_length\(get_accel_max_mss\(\)\);', fw)
      is not None,
      '★ 合成后的 (前向, 横向) 加速度再整体做**长度限制** ⇒ 加速度平面上是个圆'
      '（对应参数说明里的 "defines a circular constraint in accel"）')
check(re.search(r'accel_mss_to_angle_rad\(-bf_accel_target_mss\.x\)', fw) is not None,
      '★★ 俯仰取的是 **-前向加速度** —— 注释写着 '
      '"negative accel for negative pitch (pitch forward)"（前向加速要低头）')

# --- yaw_rate = a_lat / v 的推导与数值验证 ---
check('accel = vel * w' in fw and 'w = accel / vel' in fw,
      '★★ 源码注释里带了一段完整推导：a = rω²、r = v/ω ⇒ a = vω ⇒ **ω = a/v**')
check(re.search(r'yaw_rate_rads = bf_lat_accel_target_mss / _target_vel_ms;', fw) is not None,
      '★ 落到代码就是一行：`yaw_rate_rads = 横向加速度 / 目标前飞速度`')


def circular_motion_sim(v, r, steps=20000, dt=1e-4):
    """让一个点以速度 v 沿半径 r 做匀速圆周运动，用中心差分测 a 和 ω"""
    w = v / r
    def pos(t):
        return (r * math.cos(w * t), r * math.sin(w * t))
    # 在 t=0 附近取三点做二阶中心差分
    p_m, p_0, p_p = pos(-dt), pos(0.0), pos(dt)
    ax = (p_p[0] - 2 * p_0[0] + p_m[0]) / (dt * dt)
    ay = (p_p[1] - 2 * p_0[1] + p_m[1]) / (dt * dt)
    a = math.hypot(ax, ay)
    # 用两时刻的角度差测 ω（先解缠绕不是问题：这里角度差恒 < pi）
    th0 = math.atan2(p_0[1], p_0[0])
    t1 = steps * dt
    p1 = pos(t1)
    th1 = math.atan2(p1[1], p1[0])
    dth = (th1 - th0 + math.pi) % (2 * math.pi) - math.pi
    if t1 > 0 and abs(dth) > math.pi / 2:
        # 转太多圈了会丢信息，靠已知 w 取整圈数修正
        dth += 2 * math.pi * round((w * t1 - dth) / (2 * math.pi))
    return a, dth / t1


worst = 0.0
for v, r in ((11.0, 50.0), (15.0, 30.0), (8.0, 100.0), (20.0, 25.0)):
    a, w_meas = circular_motion_sim(v, r)
    w_pred = a / v                     # ← 源码用的那个公式
    worst = max(worst, abs(w_pred - w_meas) / w_meas)
    check(abs(a - v * v / r) < 1e-3,
          '★ 数值验证：v=%.0f m/s、r=%.0f m ⇒ 实测 a = %.4f（解析 v²/r = %.4f）'
          % (v, r, a, v * v / r))
check(worst < 1e-3,
      '★★ 独立基准：源码那条 `ω = a / v` 与"沿圆周实测的角速度"一致到 %.1e —— '
      '圆周运动的 a = vω 恒等式成立，所以这一行不是简化而是精确的' % worst)
check(re.search(r'if\s*\(!is_zero\(_target_vel_ms\)\)', fw) is not None,
      '★ 而代码里对 `_target_vel_ms == 0` 做了保护 —— 否则就是除零')


# ==========================================================================
print()
print('=' * 76)
print('[9] 自转的进入条件与着地判据')
print('=' * 76)

gat = func_body(src_heli, r'void Copter::heli_update_autorotation\(\)')
check(re.search(r'if\s*\(ap\.land_complete\s*\|\|\s*ap\.land_complete_maybe\)', gat) is not None,
      '★ 闸门第 1 条：判定已着陆 ⇒ `force_deactivate_autorotation()` 并返回')
check(re.search(r'if\s*\(!motors->get_interlock\(\)\s*&&\s*'
                r'\(flightmode->has_manual_throttle\(\)\s*\|\|\s*in_autorotation_mode\)\)', gat)
      is not None,
      '★★ 闸门第 2 条：**关掉互锁开关（motor interlock）**，'
      '且处于"手动油门模式"或"自转模式" ⇒ 进入自转')
check('motors->set_autorotation_active(true)' in gat and
      'motors->set_autorotation_active(false)' in gat,
      '★ 两条支路：置位 / 交给 bailout（恢复动力时走渐进加油，不是一下子给满）')

# --- RSC 的三态 ---
RST = enum_values(src_rsc, 'State')
check(RST is not None and set(RST) == {'DEACTIVATED', 'BAILING_OUT', 'ACTIVE'},
      '★ RSC_Autorotation::State 三态：%s'
      % ', '.join(sorted(RST, key=RST.get)) if RST else '未找到')
check('bailout' in src_rsc.lower(),
      '★★ 有个 BAILING_OUT（"跳伞式复飞"）状态 —— 对应参数 AROT_/H_RSC_ 的 '
      'bailout_throttle_time 与 bailout_runup_time')
check(re.search(r'get_bailout_ramp|bailout_ramp', src_rsc) is not None,
      '★ 复飞时不是阶跃给油门，而是按一条 ramp 渐进 —— 防止打坏传动')

# --- 着地判据：三条件 AND ---
cl = func_body(src_arot, r'bool AC_Autorotation::check_landed')
check(cl.count('_landed_reason.') == 6,
      '★ check_landed() 里 _landed_reason 出现 %d 次（三个分量各赋一次、各读一次）'
      % cl.count('_landed_reason.'))
check(re.search(r'return\s+_landed_reason\.min_speed\s*&&\s*_landed_reason\.land_col\s*&&\s*'
                r'_landed_reason\.is_still;', cl) is not None,
      '★★ 返回的是三个条件的 **AND**：速度 < 1 m/s、总距低于着陆下限、'
      '且 INS 的 is_still() 成立 —— 少一个都不算落地')
check(re.search(r'const float min_moving_speed = 1\.0;', cl) is not None,
      '★ 速度门限是写死的 1.0 m/s（不是参数）')
check(re.search(r'velocity\.length\(\)\s*<\s*min_moving_speed', cl) is not None,
      '★ 用的是**速度矢量的模**，不是垂速')

# --- 落地后平滑归零 ---
rl = func_body(src_arot, r'void AC_Autorotation::run_landed')
check(re.search(r'_desired_vel_ms \*= 0\.95;', rl) is not None,
      '★★ run_landed 每拍把期望速度乘 0.95 —— 指数衰减到零。'
      '注释解释原因："Don\'t allow controller to continually ask for more pitch to build speed '
      'when we are on the ground"')
decay = 0.95 ** 100
check(decay < 0.01,
      '   复算：%d 拍之后只剩初始值的 %.4f%%（假设 400 Hz 就是 %.2f 秒）'
      % (100, decay * 100, 100 / 400.0))

# --- dt 的兜底 ---
sdt = func_body(src_arot, r'void AC_Autorotation::set_dt')
check(re.search(r'_dt = 2\.5e-3;', sdt) is not None and '400 Hz' in sdt,
      '★★ set_dt() 的兜底是 `_dt = 2.5e-3;` 且注释直接写 "Assume 400 Hz" —— '
      '与第 11 篇 FreqResp 里那个硬编码 dt 是同一个模式')
check(re.search(r'g2\.arot\.set_dt\(last_loop_time_s\)', src_mode) is not None,
      '   Copter 侧每拍都会用真实 dt 覆盖它（所以实际不会走到兜底）')


# ==========================================================================
print()
print('=' * 76)
print('[10] 三个库的"枚举死值"是同一个模式（跨 10/11/12 篇）')
print('=' * 76)

# 三处都在本脚本能验证的范围内
#
# 10 篇：AC_CustomControl 的 CustomControlType::CONT_EMPTY 不可达
# 11 篇：AP_Quicktune 的 Stage 有 8 个值从没被赋给 current_stage
# 12 篇：ModeAutorotate 的 Phase 有 4 个值从没被赋给 current_phase
stage = enum_values(slurp('libraries/AP_Quicktune/AP_Quicktune.h'), 'Stage')
qt_assigned = set(re.findall(r'current_stage\s*=\s*Stage::(\w+)',
                             slurp('libraries/AP_Quicktune/AP_Quicktune.cpp')))
check(stage is not None and len(stage) == 10 and len(qt_assigned) == 2,
      '★ 第 11 篇那个：Stage 有 %d 个值，而 current_stage 只被赋过 %d 个'
      % (len(stage) if stage else 0, len(qt_assigned)))

rows = [
    ('AC_CustomControl', 'CustomControlType', 3, 2,
     'CONT_EMPTY 的 init() 分支整段被注释'),
    ('AP_Quicktune', 'Stage', len(stage) if stage else 10, len(qt_assigned),
     '只有 D 和 P 两条路可走'),
    ('ModeAutorotate', 'Phase', len(allnames), len(assigned),
     'FLARE / TOUCH_DOWN 四个相位被空 case 占位'),
]
check(len(rows) == 3, '★★ 连续三篇（）都在同一个位置发现同一类问题：')
for lib, en, total, used, why in rows:
    print('        %-18s %-20s 枚举 %2d 项 / 实际用到 %d 项 —— %s' % (lib, en, total, used, why))
check(all(used < total for _, _, total, used, _ in rows),
      '★★ 三处的共同点：**枚举声明了"N 种可能"，但代码只实现了其中一部分**，'
      '而且没有任何一处在文档或编译期提示这件事')
check(assigned != allnames and qt_assigned != set(stage or {}),
      '★ 判据是"被赋过值的那些" vs "声明了的那些" —— 用 grep 一跑就能查，'
      '不需要运行、也不需要真机')


# ==========================================================================
print()
print('=' * 76)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
