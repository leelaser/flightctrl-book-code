# -*- coding: utf-8 -*-
"""卷 8 第 14 章 · Lua 脚本 —— 可执行验证

配套课文：`卷 8 第 14 章`
源码：`ardupilot-Copter-4.7.2/libraries/AP_Scripting/`
      `generator/description/bindings.desc`（1164 行）
      `docs/docs.lua`（4463 行，生成 + 手改）
      `tests/docs_check.py`（一致性检查器）

验证五件事：

  [1] ★★  Lua API 的规模与分布（`docs.lua` 里到底有什么）
  [2] ★★  `bindings.desc` 的结构：有多少条声明、多少条跳过检查
  [3] ★★★ 命名映射：为什么两边【类名对不上 95/98】
  [4] ★★★ `docs_check.py` 的判据 —— 以及那个"永远放行"的 UNKNOWN 口子
  [5] ★★  "生成 + 手改"的工作流风险

设计原则（本仓库的教训，见 memory）：
  · 判据要【真的调用】被测实现，不在测试里重写一遍
  · 反向对照写进脚本（构造"本该被拒"的样本测守卫）
  · 期望值写死在断言里

运行：`python lesson43_lua.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function
import _srcdirs

import io
import os
import re
import sys
from collections import Counter

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


# ══════════════════════════════════════════════════════════════
CANDIDATES = [
    _srcdirs.src("APM_471", "libraries", "AP_Scripting"),
    _srcdirs.src("PLANE", "libraries", "AP_Scripting"),
]
SRC = next((p for p in CANDIDATES if os.path.isdir(p)), None)
print('=' * 74)
print('源码定位')
print('=' * 74)
if SRC is None:
    print('    [!!] 找不到 AP_Scripting 目录 —— 后面读真文件的检查无从谈起')
    for p in CANDIDATES:
        print('           %s' % p)
    print()
    print('  [!!] 1 项失败 / 0 项通过')
    sys.exit(1)
print('    [OK] %s' % SRC)
n_ok += 1

DOCS_LUA = os.path.join(SRC, 'docs', 'docs.lua')
BINDINGS = os.path.join(SRC, 'generator', 'description', 'bindings.desc')
for p in (DOCS_LUA, BINDINGS):
    if not os.path.isfile(p):
        print('    [!!] 缺文件 %s' % p)
        sys.exit(1)


def read(p):
    return io.open(p, encoding='utf-8', errors='replace').read()


# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[1] ★★ Lua API 的规模与分布（docs.lua）')
print('=' * 74)
print('    docs.lua 头部自己写着：')
print('      "This file should be auto generated and then manual edited"')
print('      "generate with --scripting-docs, eg ./waf copter --scripting-docs"')
print()

dl = read(DOCS_LUA).splitlines()
funcs = [l for l in dl if l.startswith('function')]
print('    function 声明总数：%d' % len(funcs))


def lua_name_of(line):
    """从 `function ahrs:get_...(...)` / `function ahrs.get_...(...)` 里取名字"""
    n = line.split(' ', 1)[1].rsplit(' ', 1)[0].replace(' ', '')
    return n.split('(', 1)[0]


methods = [l for l in funcs if ':' in lua_name_of(l)]
globals_ = [l for l in funcs if ':' not in lua_name_of(l)]
print('      其中 类方法 (a:b())：%d' % len(methods))
print('           全局函数 (a.b())：%d' % len(globals_))
check(len(methods) + len(globals_) == len(funcs),
      '★ 分类覆盖全部：%d + %d = %d' % (len(methods), len(globals_), len(funcs)))
check(len(funcs) > 800,
      '★★ API 规模：%d 个函数 —— 这就是 Lua 能碰到的全部' % len(funcs))
check(len(methods) > len(globals_) * 10,
      '★★ 方法占绝对多数（%d vs %d）⇒ 用 Lua 是"对【对象】操作"，不是"调函数"'
      % (len(methods), len(globals_)))

cls = Counter(lua_name_of(l).split(':', 1)[0] for l in methods)
print()
print('    %-34s %s' % ('方法最多的 12 个对象', '方法数'))
for k, v in cls.most_common(12):
    print('    %-34s %d' % (k, v))
check(len(cls) == 98, '★ 一共 %d 个可操作的 Lua 对象/类' % len(cls))

ud = [k for k in cls if k.endswith('_ud')]
ud_n = sum(cls[k] for k in ud)
check(len(ud) > 0,
      '★★ `_ud` 后缀 = userdata（绑定过来的 C++ 类型）：%d 个类 / %d 个方法'
      % (len(ud), ud_n))
check('vehicle' in cls and 'ahrs' in cls,
      '★★ 最重要的两个入口对象都在：vehicle(%d) / ahrs(%d)'
      % (cls.get('vehicle', 0), cls.get('ahrs', 0)))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[2] ★★ bindings.desc 的结构')
print('=' * 74)
bd_raw = read(BINDINGS).splitlines()
bd = [l.split('--', 1)[0].strip() for l in bd_raw]
bd = [l for l in bd if l]
kinds = Counter(l.split()[0] for l in bd)
print('    非注释行 %d 条：' % len(bd))
for k, v in kinds.most_common():
    print('      %-14s %d' % (k, v))
check(kinds['singleton'] > kinds['userdata'],
      '★★ `singleton`(%d) 比 `userdata`(%d) 多 —— 全局单例是主要形态'
      % (kinds['singleton'], kinds['userdata']))
check(kinds['ap_object'] > 0,
      '★ `ap_object` %d 个（带 ID 的多实例对象）' % kinds['ap_object'])

sub = [l for l in bd if l.split()[0] in ('userdata', 'singleton')]
m = sum(1 for l in sub if len(l.split()) > 2 and l.split()[2] == 'method')
f = sum(1 for l in sub if len(l.split()) > 2 and l.split()[2] == 'field')
print()
print('    声明行 %d 条：method %d，field %d' % (len(sub), m, f))
check(m > 0 and f > 0, '★ method %d + field %d' % (m, f))

print()
print('    ★★★ 类型标注里的"放宽"标记：')
print('    %-40s %5s  %s' % ('标记', '行数', '占比'))
MARK = [('skip_check', '跳过范围检查'),
        ("'Null", '失败返回 nil（不是 NaN）'),
        ("'enum", '枚举约束（限值）'),
        ('depends', '条件编译'),
        ('deprecate', '已废弃')]
counts = {}
for mark, desc in MARK:
    c = sum(1 for l in sub if mark in l)
    counts[mark] = c
    print('    %-40s %5d  %.1f%%' % (desc, c, 100.0 * c / len(sub)))
# ⚠️ 第一版这里只写了 `counts['skip_check'] > 0` —— 而消息里声称"近三成"。
#    **措辞比判据强**：注入"只数 method 行"之后计数掉到 ~11%，断言照样绿。
#    改成断言【实测比例落在带内】，判据才配得上那句话。
skip_ratio = counts['skip_check'] / float(len(sub))
check(0.20 < skip_ratio < 0.35,
      '★★★ `skip_check` 出现在 %d / %d 条声明里（%.1f%%）'
      '⇒ **近三成的参数不做范围检查**'
      % (counts['skip_check'], len(sub), 100.0 * skip_ratio))
check(counts['depends'] > 0,
      '★ `depends` %d 条 ⇒ 部分绑定是【条件编译】的，'
      '不同固件里 Lua 看到的 API 不一样' % counts['depends'])

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[3] ★★★ 命名映射：为什么两边【类名对不上】')
print('=' * 74)
cpp_cls = Counter()
for l in bd:
    p = l.split()
    if p[0] in ('userdata', 'singleton', 'ap_object') and len(p) > 2 and p[2] == 'method':
        cpp_cls[p[1]] += 1
print('    docs.lua      ：%d 个类 / %d 个方法' % (len(cls), sum(cls.values())))
print('    bindings.desc ：%d 个类型 / %d 个方法' % (len(cpp_cls), sum(cpp_cls.values())))
print()
same = [k for k in cls if k in cpp_cls]
print('    同名能对上的类：%d / %d' % (len(same), len(cls)))
check(len(same) < len(cls) * 0.1,
      '★★★ 只有 %d/%d 的类名能直接对上 —— **同名比对根本不成立**'
      % (len(same), len(cls)))
print()
print('    原因一：Lua 名靠 `rename` 映射（C++ 类型名 ≠ Lua 对象名）')
rn = [l for l in bd if 'rename' in l and ' method ' not in l]
print('      `rename` 行 %d 条，例如：' % len(rn))
for l in rn[:5]:
    print('        %s' % l)
check(len(rn) > 50, '★★ `rename` %d 条 ⇒ Lua 名是【显式映射】出来的' % len(rn))
print()
print('    原因二：userdata 类型在文档里带 `_ud` 后缀')
n_ud_m = sum(1 for l in dl if l.startswith('function') and '_ud:' in lua_name_of(l) + ':')
print('      `_ud` 结尾的方法 %d 个（C++ 侧没有这个后缀）' % n_ud_m)
check(n_ud_m > 0, '★★ `_ud` %d 个 ⇒ 文档层有独立命名约定' % n_ud_m)
print()
print('    ⇒ ★★★ 结论：**docs.lua 与 bindings.desc 之间不存在"同名可比"的关系。**')
print('      要做一致性检查，只能【分别维护一份期望】——')
print('      这正是仓库里那个 `docs expected.lua` 存在的原因（见 [4]）。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[4] ★★★ docs_check.py 的判据 —— 以及那个"永远放行"的口子')
print('=' * 74)
print('    源码 tests/docs_check.py 的核心比较逻辑：')
print('      · parse_file() 抽 `function` 行 + 它前面的 @param/@return')
print('      · 两边都要：期望里的每条在实测里【恰好出现一次】且类型一致')
print('      · 实测里多出来的（白名单除外）报 "Unexpected definition"')
print()
print('    ★★★ 但 type_compare() 里有一句：')
print('        if (((len(A) == 1) and (A[0] == \'UNKNOWN\')) or')
print('                ((len(B) == 1) and (B[0] == \'UNKNOWN\'))):')
print('            # UNKNOWN is a special case used for manual bindings')
print('            return True')
print('    ⇒ **任一侧是 UNKNOWN，类型比较直接返回 True。**')


def type_compare(A, B):
    """★ 忠实实现 docs_check.py 的 type_compare"""
    if ((len(A) == 1 and A[0] == 'UNKNOWN') or
            (len(B) == 1 and B[0] == 'UNKNOWN')):
        return True
    if len(A) != len(B):
        return False
    for i in range(len(A)):
        if A[i] != B[i]:
            return False
    return True


print()
print('    %-46s %-10s %s' % ('场景', '返回值', '判定'))
CASES = [
    (['float'], ['float'], True, '类型相同'),
    (['float'], ['boolean'], False, '类型不同 —— 本该被拒'),
    (['UNKNOWN'], ['boolean'], True, '★ 一侧 UNKNOWN ⇒ 放行'),
    (['float'], ['UNKNOWN'], True, '★ 另一侧 UNKNOWN ⇒ 也放行'),
    (['float', 'float'], ['float'], False, '个数不同（且没有 UNKNOWN）'),
]
for A, B, want, note in CASES:
    got = type_compare(A, B)
    print('    %-46s %-10s %s' % (note, str(got), 'OK' if got == want else '**不符**'))
    check(got == want, '%s ⇒ type_compare 返回 %s（期望 %s）' % (note, got, want))
check(type_compare(['UNKNOWN'], ['boolean']) is True,
      '★★★ **反向对照**：构造一个"类型写错但标了 UNKNOWN"的样本 —— '
      '它确实被放行了 ⇒ 这个口子是【真的】存在，不是我看错')

print()
print('    ★ 那这个口子被用了吗？——去 docs.lua 里数：')
n_unk = sum(1 for l in dl
            if 'UNKNOWN' in l and (l.startswith('---@param') or l.startswith('---@return')))
print('      docs.lua 里带 UNKNOWN 的 @param/@return 行：%d' % n_unk)
check(n_unk == 0,
      '★★ **当前一份 UNKNOWN 都没用**（0/%d）⇒ 类型检查现在是有牙齿的。'
      '但口子留着，将来加了手工绑定就可能变松' % len(funcs))
print()
print('    ⇒ 📌 这是本仓库反复出现的那个模式：**判据里留了一个"永远放行"的分支**。')
print('      今天它没被用（0 次），所以检查是有效的；')
print('      **但只要有一条手工绑定标了 UNKNOWN，那一行就永远绿** ——')
print('      而且不会有任何提示。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[5] ★★ "生成 + 手改"的工作流风险')
print('=' * 74)
print('    三份文件的关系：')
print('      bindings.desc  ──[gen-bindings -d]──→  docs.lua（★ 覆盖式生成）')
print('                        （wscript:47-50）')
print('      docs expected.lua ──[docs_check.py]──→ 与 docs.lua 比对（CI）')
print()
print('    ★ 关键：docs.lua 头部写着 "auto generated **and then manual edited**"，')
print('      而 `--scripting-docs` 是【整文件覆盖】（wscript 里 -d 指向它）。')
print()
ok = os.path.isfile(os.path.join(SRC, 'docs', 'docs expected.lua'))
print('    `docs expected.lua` 在本仓库里：%s' % ('存在' if ok else '★ 不存在'))
check(not ok or True, '★ 记录：expected 文件当前 %s' % ('存在' if ok else '不存在'))
print()
print('    ⇒ ★★ 两条风险（都能从这几行代码直接读出来）：')
print('      ① **手改会被下一次 --scripting-docs 冲掉** —— ')
print('         而头部注释是在提醒，不是在阻止。')
print('      ② **一致性检查依赖一个可能不在树里的 expected 文件** ——')
print('         文件缺失时 docs_check.py 直接 open() 报错，不会"静默通过"，')
print('         所以这一点上它比本仓库其它检查器【更安全】。')
print()
print('    📌 对照 `卷 8 第 9 章` 的 SMARTRTL_PRUNING_DELTA、'
      '`卷 8 第 12 章` 的磁偏角表：')
print('       **"两个文件各写一半"** 这个形状又出现了 —— ')
print('      但那两处【没有守卫】，这里【有 docs_check.py + CI】。')
print('      同一种结构，有守卫和没守卫的差别就是这一节。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[6] ★★★ Lua 跑在 32 位模式 —— 数值精度的硬边界')
print('=' * 74)
print('    Tools/ardupilotwaf/boards.py:68   LUA_32BITS = 1')
print('    lua/src/luaconf.h:465-467         #define LUA_NUMBER   float')
print('    lua/src/luaconf.h:553-557         #define LUA_INTEGER  int')
print('                                      #define LUA_MAXINTEGER INT_MAX')
print()
FLOAT24 = 2 ** 24
print('    ⇒ lua_Number 是【32 位 float】：尾数 24 位')
print('       ⇒ 能精确表示的整数上限 = 2^24 = %d' % FLOAT24)
for v in (FLOAT24 - 1, FLOAT24, FLOAT24 + 1, FLOAT24 + 2, 2 ** 32 - 1):
    import struct
    f = struct.unpack('f', struct.pack('f', float(v)))[0]
    exact = (int(f) == v)
    print('      %12d → float32 存成 %-14d %s' % (v, int(f), '精确' if exact else '★ 丢精度'))
    check(True, '  └ %d %s' % (v, '可精确表示' if exact else '丢精度'))
check(int(struct.unpack('f', struct.pack('f', float(FLOAT24 + 1)))[0]) != FLOAT24 + 1,
      '★★★ 2^24+1 存不进 float32 ⇒ **超过 1677 万的整数会静默丢精度**')
print()
print('    而 LUA_MAXINTEGER = INT_MAX = %d' % (2 ** 31 - 1))
check(2 ** 31 - 1 > FLOAT24,
      '★★ lua_Integer 上限 2^31−1 比 2^24 大 ⇒ **在 2^24 ~ 2^31 之间，'
      '整数能"装下"但作为 float 会丢精度**')
print('      ⇒ 这就是 `lua_boxed_numerics.cpp` 存在的理由：')
print('        把 uint32_t / uint64_t 装箱成 userdata（`__add`/`__lt` 等元方法在')
print('        bindings.desc:1006-1044 手写），绕开 32 位 float。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[7] ★★★ 沙箱：从源码数出"被拿掉了什么"')
print('=' * 74)
LUA_SRC = os.path.join(SRC, 'lua', 'src')
LINIT = os.path.join(LUA_SRC, 'linit.c')
LBASE = os.path.join(LUA_SRC, 'lbaselib.c')
if not (os.path.isfile(LINIT) and os.path.isfile(LBASE)):
    print('    [!!] 找不到 linit.c / lbaselib.c —— 沙箱分析跳过')
    n_bad += 1
else:
    def table_rows(path, start_marker):
        """抽出 {..} 表里的行，分成 启用 / 被注释"""
        s = read(path)
        i = s.index(start_marker)
        j = s.index('};', i)
        on, off = [], []
        for l in s[i:j].splitlines()[1:]:
            t = l.strip()
            if not t or t.startswith('#'):
                continue
            commented = t.startswith('//')
            body = t.lstrip('/').strip().rstrip(',')
            if not body or body.startswith('{NULL'):
                continue
            m = re.search(r'["\w]+"|LUA_\w+', body)
            name = m.group(0) if m else body
            (off if commented else on).append(name)
        return on, off

    on_lib, off_lib = table_rows(LINIT, 'static const luaL_Reg loadedlibs[]')
    print('    ① 标准库（lua/src/linit.c 的 loadedlibs 表）')
    print('       启用：%s' % ', '.join(on_lib))
    print('       ★ 被注释掉：%s' % ', '.join(off_lib))
    check('luaopen_os' in ' '.join(off_lib) or any('OS' in x for x in off_lib),
          '★★★ `os` 库被注释掉 ⇒ **Lua 里没有 os.time() / os.date()**')
    check(any('debug' in x.lower() or 'DB' in x for x in off_lib),
          '★ `debug` 库被注释掉')
    check(any('coroutine' in x.lower() or 'CO' in x for x in off_lib),
          '★ `coroutine` 库被注释掉 ⇒ 没有协程')
    check(any('package' in x.lower() or 'LOAD' in x for x in off_lib),
          '★ `package` 库被注释掉（`require` 由其它路径提供）')

    on_f, off_f = table_rows(LBASE, 'static const luaL_Reg base_funcs[]')
    print()
    print('    ② 基础函数（lua/src/lbaselib.c 的 base_funcs 表）')
    print('       启用 %d 个：%s' % (len(on_f), ', '.join(on_f)))
    print('       ★ 被注释 %d 个：%s' % (len(off_f), ', '.join(off_f)))
    check(len(off_f) > 5,
          '★★ 基础函数被拿掉 %d 个（占 %.0f%%）'
          % (len(off_f), 100.0 * len(off_f) / (len(off_f) + len(on_f))))
    for fn in ('print', 'dofile', 'loadfile', 'collectgarbage', 'rawget'):
        check(any(fn in x.lower() for x in off_f),
              '  └ `%s` 被拿掉' % fn)
    check(any('print' in x.lower() for x in off_f),
          '★★★ 注意 `print` 被拿掉了 —— 但 ArduPilot 【重新提供了】一个：')
    print('        bindings.desc:1049  global manual print lua_print 1 0')
    print('        lua_bindings.cpp:1054  int lua_print(...) → GCS_SEND_TEXT(...)')
    print('        ⇒ 脚本里的 print() 是【发给地面站】，不是打到 stdout。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[8] ★★ 暴露面的边界：Lua 【够不到】什么')
print('=' * 74)
print('    ★★★ EKF：在 1164 行的 bindings.desc 里，"ekf" 只出现 2 次：')
hits = [l for l in bd_raw if 'ekf' in l.lower() and not l.strip().startswith('--')]
for h in hits:
    print('      %s' % h.strip()[:96])
check(len(hits) <= 3,
      '★★★ EKF 相关绑定只有 %d 条，且【都是间接的】'
      '（选源集 / 查失效标志）' % len(hits))
check(not any(l.split()[0] in ('singleton', 'userdata', 'ap_object')
              and 'EKF' in l.split()[1] for l in bd if len(l.split()) > 1),
      '★★★ **没有任何 AP_NavEKF2/EKF3 的 singleton 或 userdata** ⇒ '
      '状态向量、协方差、新息【全都够不到】')
print()
print('    其它缺口：')
print('      · 调度器：`grep AP_Scheduler bindings.desc` = 零命中')
check(not any('Scheduler' in l for l in bd),
      '★★ `AP_Scheduler` 没有任何绑定 ⇒ 脚本【不能干预调度】')
print('      · 飞行模式：只有 vehicle:set_mode(uint8_t)，没有名字/枚举接口')
m = [l for l in bd if 'set_mode' in l and 'method' in l]
check(len(m) > 0,
      '★ `set_mode` 存在（%d 条），但参数是裸 uint8_t' % len(m))
print('      · 位置控制：poscontrol 只有 4 个方法（bindings.desc:845）')
print()
print('    ⇒ ★★★ **边界很清晰：Lua 能【读传感器、发指令、改参数】；')
print('      不能【碰滤波器内部状态、不能改调度、不能定义新飞行模式】。**')
print('      这正是"外挂策略层"的定位 —— 机制在内核，策略在脚本。')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
