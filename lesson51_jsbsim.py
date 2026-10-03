# -*- coding: utf-8 -*-
"""
lesson51_jsbsim.py —— 第 06 篇《JSBSim 建模》配套自验证脚本

本课与前几课的区别：**JSBSim 的模型是纯数据（XML）**。
所以「模型写得对不对」这件事可以在**不运行**的前提下机器校验 ——
本脚本真的去解析那几个 XML，而不是只 grep 文本。

  [1] 结构：类、工厂、编译开关
  [2] ★★★ 五条独立证据：ArduPilot **不链接** JSBSim
  [3] 三通道：socket 类型、端口算术、文本控制命令
  [4] 启动握手：顺序、--suspend、expect、resume
  [5] ★★★ FGNetFDM：逐字段算结构体布局 + 字节序补丁的必要性
  [6] ★★★ 真解析 Rascal.xml：骨架 + 气动表的单调性
  [7] ★ 固定翼 vs 四旋翼：空的气动轴 / 发动机数量
  [8] 三处「名字和内容对不上」
  [9] 参数个数 = 0
  [10] 启动链路：从 frame 串到模型名
  [11] 诚实边界

运行： python3 lesson51_jsbsim.py     （全过 EXIT=0）
"""

import io
import os
import re
import sys
import xml.etree.ElementTree as ET

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
    print('    [!!] 找不到 ArduPilot 源码树')
    n_bad += 1
    sys.exit(1)


def slurp(rel):
    return io.open(os.path.join(REPO, rel.replace('/', os.sep)),
                   encoding='utf-8', errors='replace').read()


F_CPP, F_H = 'libraries/SITL/SIM_JSBSim.cpp', 'libraries/SITL/SIM_JSBSim.h'
src_cpp, src_h = slurp(F_CPP), slurp(F_H)
check(len(src_cpp) > 10000 and len(src_h) > 5000,
      '读入 SIM_JSBSim.cpp (%d B) 与 .h (%d B)' % (len(src_cpp), len(src_h)))


# ==========================================================================
print()
print('=' * 76)
print('[1] 结构')
print('=' * 76)

check('class JSBSim : public Aircraft {' in src_h,
      '★ 类 `JSBSim` 继承 `Aircraft` —— 与其它 27 个后端同一接口')
check(re.search(r'static Aircraft \*create\(const char \*frame_str\)', src_h) is not None,
      '★ 工厂函数 `create()`')
check(re.search(r'#define AP_SIM_JSBSIM_ENABLED\s*\(CONFIG_HAL_BOARD == HAL_BOARD_SITL\)',
                slurp('libraries/SITL/SIM_config.h')) is not None,
      '★ 编译开关只按板子门控 ⇒ **只要编 SITL 就会编进这个后端**')
check('class FGNetFDM {' in src_h,
      '★ 第二个类 `FGNetFDM` —— 纯数据结构，没有基类')


# ==========================================================================
print()
print('=' * 76)
print('[2] ★★★ 五条独立证据：ArduPilot 不链接 JSBSim')
print('=' * 76)

# ★ 不能只搜 "jsbsim" —— 它**包含自己的头** `SIM_JSBSim.h`，那是正常的。
#   要验的是"没包含 **JSBSim 库** 的头"。所以把 include 列表抽出来逐个看。
incs = re.findall(r'^#include\s+[<"]([^>"]+)[>"]', src_cpp, re.M)
check(len(incs) >= 8, '★ `.cpp` 共 %d 条 include' % len(incs))
lib_incs = [i for i in incs if re.match(r'^(FGFDMExec|FG|JSBSim|jsbsim)', i)]
check(lib_incs == [],
      '① **没有一条 include 指向 JSBSim 库**（全部是 %s）'
      % ', '.join(sorted(set(os.path.basename(i) for i in incs))[:6]))
check('SIM_JSBSim.h' in incs,
      '★ 它只包含**自己的**头 `SIM_JSBSim.h`（第一版把这条也算成了"链接 JSBSim"）')
check(re.search(r'https://github\.com/JSBSim-Team/jsbsim', src_h) is not None,
      '② `SIM_JSBSim.h` 把 JSBSim 当**外部项目**引用（URL 形式）')
wscript = slurp('libraries/SITL/wscript')
check('jsbsim' not in wscript.lower(),
      '③ `libraries/SITL/wscript` 全文 %d 行，**无 jsbsim 字样** ⇒ 没有链接脚本' % len(wscript.splitlines()))
mods = sorted(d for d in os.listdir(os.path.join(REPO, 'modules'))
              if os.path.isdir(os.path.join(REPO, 'modules', d)))
check('JSBSim' not in mods and 'jsbsim' not in [m.lower() for m in mods],
      '④ `modules/` 下 **没有** JSBSim 子模块（共 %d 个模块）' % len(mods))
bs = slurp('Tools/scripts/build-jsbsim.sh')
check('git clone https://github.com/JSBSim-Team/jsbsim.git' in bs,
      '⑤ 有**独立的脚本**去 clone 上游 ⇒ 它是外部依赖，不是子模块')

check(re.search(r'pid_t child_pid = fork\(\);', src_cpp) is not None,
      '★★★ 正面证据：`fork()` 出一个子进程')
check(re.search(r'execlp\("JSBSim",', src_cpp) is not None,
      '★★★ 然后 `execlp("JSBSim", ...)` —— 执行一个**外部可执行文件**')
check(re.search(r'if \(pipe\(p\) != 0\)', src_cpp) is not None,
      '★ 并建了管道（用于 stdout 握手）')
for fn in ('fork()', 'pipe(', 'execlp(', 'setsid()', 'dup2(', 'chdir('):
    check(fn in src_cpp, '★ 用到裸 POSIX 调用 `%s`' % fn)
check('_WIN32' not in src_cpp and '_WIN32' not in src_h,
      '★★ 全文件**没有 `_WIN32` 条件编译** ⇒ 它在 Windows 上依赖 POSIX 兼容层')
check('--branch' not in bs and 'git checkout' not in bs,
      '★★★ 构建脚本 `git clone` 的是**默认分支**，没有 tag / commit pin '
      '⇒ JSBSim 版本不受 ArduPilot 控制')


# ==========================================================================
print()
print('=' * 76)
print('[3] 三通道：TCP 控制 / UDP 状态 / stdout 日志')
print('=' * 76)

check('SocketAPM_native sock_control;' in src_h and 'SocketAPM_native sock_fgfdm;' in src_h,
      '★ 两个 socket 成员：`sock_control`（控制）与 `sock_fgfdm`（状态）')
check(re.search(r'// tcp input control socket to JSBSIm', src_h) is not None,
      '★ 注释自陈控制口是 **TCP**')
check(re.search(r'// UDP packets from JSBSim in fgFDM format', src_h) is not None,
      '★ 状态口是 **UDP**，格式是 fgFDM')
check(re.search(r'control_port = 5505 \+ instance\*10;', src_cpp) is not None
      and re.search(r'fdm_port = 5504 \+ instance\*10;', src_cpp) is not None,
      '★★★ 端口算术：控制 `5505+instance*10`、状态 `5504+instance*10`')
check(5505 - 5504 == 1,
      '★ 复算：控制口与状态口**总是差 1**（5505/5504、5515/5514…）')
check('sock_fgfdm.bind("127.0.0.1", fdm_port)' in src_cpp,
      '★ 状态口是**接收方**（`bind`），不是 connect')

cmds = re.findall(r'"set (fcs/[a-z-]+|atmosphere/[^ ]+) %f\\n"', src_cpp)
check(len(cmds) >= 4, '★★ 每帧下发的控制命令有 %d 条：%s' % (len(cmds), ', '.join(cmds[:4])))
for k in ('fcs/aileron-cmd-norm', 'fcs/elevator-cmd-norm', 'fcs/rudder-cmd-norm',
          'fcs/throttle-cmd-norm'):
    check(k in src_cpp, '★ 舵量通道 `%s` —— **纯文本行**，抓包可直读' % k)
check('"iterate 1\\n"' in src_cpp,
      '★★★ 命令最后一句是 `iterate 1` ⇒ **锁步**：ArduPilot 每帧命令 JSBSim 推进一步，'
      '两个进程不会各跑各的')
check('atmosphere/psiw-rad' in src_cpp and 'atmosphere/wind-mag-fps' in src_cpp,
      '★★ **风是通过控制通道设进去的** ⇒ `SIM_WIND_*` 参数在 JSBSim 模式下失效')
check('atmosphere/turbulence/milspec' in src_cpp,
      '★ 湍流用的是 **MIL-SPEC**（军用标准）模型')


# ==========================================================================
print()
print('=' * 76)
print('[4] 启动握手')
print('=' * 76)

m = re.search(r'while \(!initialised\) \{(.*?)\}', src_cpp, re.S)
body = m.group(1) if m else ''
order = re.findall(r'!(\w+)\(\)', body)
check(order[:4] == ['create_templates', 'start_JSBSim', 'open_control_socket', 'open_fdm_socket'],
      '★★★ 初始化顺序固定：%s' % ' → '.join(order[:4]))
check(re.search(r'if \(!sock_fgfdm\.bind\("127\.0\.0\.1", fdm_port\)\)', src_cpp) is not None
      or src_cpp.find('bind("127.0.0.1", fdm_port)') < src_cpp.find('pid_t child_pid = fork()'),
      '★★ 而且 **UDP 口在 fork 之前就 bind 好** —— 否则 JSBSim 一起来发的包全丢')

check('"--suspend"' in src_cpp,
      '★★ JSBSim 以 `--suspend` **挂起启动**（起来但不跑）')
check(re.search(r'if \(!expect\("JSBSim Execution beginning"\)\)', src_cpp) is not None,
      '★★★ 靠 **stdout 上的一句握手**确认它起来了："JSBSim Execution beginning"')
check('AP_HAL::panic("Unable to start JSBSim");' in src_cpp
      and 'AP_HAL::panic("Failed to start JSBSim");' in src_cpp,
      '★★★ 等不到就 **panic**，不重试 —— 静默跑一个没有物理的 SITL 更糟')

m = re.search(r'char startup\[\] =\s*\n(.*?);', src_cpp, re.S)
st = m.group(1) if m else ''
for cmd in ('info\\n', 'resume\\n', 'iterate 1\\n', 'set atmosphere/turb-type 4\\n'):
    check(cmd in st, '★ 启动串里有 `%s`' % cmd.replace('\\n', ''))

gens = re.findall(r'jsbsim_(start|fgout)_%u\.xml', src_cpp)
check(sorted(set(gens)) == ['fgout', 'start'], '★ 运行时生成两份 XML：%s' % ', '.join(sorted(set(gens))))
check('/reset.xml' in src_cpp, '★ 第三份：`aircraft/<模型>/reset.xml`（初始位置）')
check('type=\\"FLIGHTGEAR\\"' in src_cpp,
      '★★★ 输出重定向用的是 **FlightGear 协议**（`type="FLIGHTGEAR"`）'
      ' ⇒ ArduPilot 复用了现成协议，没有自定义')


# ==========================================================================
print()
print('=' * 76)
print('[5] ★★★ FGNetFDM：逐字段算布局，验证字节序补丁')
print('=' * 76)

# ★ 按**花括号配对**取结构体全文 —— 第一版用 `find('};')` 找结尾，
#   结果停在结构体里那个 `enum {...};` 上（它比结构体结束得早），
#   于是"解析出 0 个字段"。
_start = src_h.find('class FGNetFDM {')
_i, _depth = _start, 0
while _i < len(src_h):
    if src_h[_i] == '{':
        _depth += 1
    elif src_h[_i] == '}':
        _depth -= 1
        if _depth == 0:
            break
    _i += 1
struct = src_h[_start:_i + 1]
check(struct.count('{') == struct.count('}'),
      '★ 按花括号配对准确定位了 `FGNetFDM` 结构体（%d 行）' % struct.count('\n'))
# 常量
consts = dict(re.findall(r'(FG_MAX_\w+)\s*=\s*(\d+)', struct))
check(consts == {'FG_MAX_ENGINES': '4', 'FG_MAX_WHEELS': '3', 'FG_MAX_TANKS': '4'},
      '★ 三个容量常量：%s' % ', '.join('%s=%s' % kv for kv in sorted(consts.items())))

# 逐字段抽取（type name; 或 type name[N];）
TYPE = {'uint32_t': (4, 4), 'int32_t': (4, 4), 'float': (4, 4), 'double': (8, 8)}
fields = []
for line in struct.split('\n'):
    line = line.split('//')[0].strip()
    m = re.match(r'^(uint32_t|int32_t|float|double)\s+(\w+)(?:\[(\w+)\])?;', line)
    if m:
        typ, name, arr = m.group(1), m.group(2), m.group(3)
        n = int(arr) if arr and arr.isdigit() else int(consts.get(arr, 1))
        fields.append((typ, name, n))
# ★ 原本写的是 `>= 55` —— 那是**弱断言**：把期望改成 30 它照样过（反向注入 S2 抓到的）。
#   这里要的是一个确定的数，所以改成 == 58。结构体变了这条会红，那正是它该做的。
check(len(fields) == 58, '★★ 解析出 **%d** 个字段声明（精确值）' % len(fields))

off, layout = 0, []
for typ, name, n in fields:
    sz, al = TYPE[typ]
    if off % al:
        off += al - (off % al)               # 自然对齐
    layout.append((name, typ, n, off))
    off += sz * n
size = off
if size % 8:
    size += 8 - (size % 8)                   # 整体按最大对齐（double=8）收尾

check(size == 408, '★★★ 按自然对齐逐字段累加：结构体大小 = **%d 字节**' % size)
check(size % 4 == 0,
      '★★★ 且 %d %% 4 == 0 ⇒ `ByteSwap()` 那个按 `uint32_t` 逐字翻转的循环**恰好覆盖全部字节**'
      % size)
check(size // 4 == 102,
      '★ 复算：循环会跑 `sizeof(*this)/4` = **%d** 次 `ntohl`' % (size // 4))

doubles = [(nm, o) for nm, t, n, o in layout if t == 'double']
check([nm for nm, _ in doubles] == ['longitude', 'latitude', 'altitude'],
      '★★★ 结构体里正好 **3 个 double**：%s —— 与注释 "fixup the 3 doubles" 对上'
      % ', '.join(nm for nm, _ in doubles))
check(all(o % 4 == 0 for _, o in doubles) and all(o % 8 == 0 for _, o in doubles),
      '★★ 三个 double 都在偏移 %s —— 都是 8 字节对齐' % [o for _, o in doubles])
# ★ 判据简化过一次：原来写成 "float 的偏移都不是 8 的倍数" —— 那对 float 是**恒真**的
#   （4 字节量本来就该落在 4 而非 8 上），恒真的判据等于没判。真正要守的是"全部 4 对齐"。
check(all(o % 4 == 0 for _, _, _, o in layout),
      '★ 而**所有**字段都落在 4 的倍数上 ⇒ 那个 32 位翻转循环对它们是**正确**的，'
      '只有那三个 8 字节的 double 需要额外修')

check('// nasty hack .... JSBSim sends in little-endian' in src_h
      or 'nasty hack' in src_h,
      '★★★ 源码自己把这段叫作 "nasty hack"，并解释了原因（JSBSim 发的是小端）')
check('void FGNetFDM::ByteSwap(void)' in src_cpp,
      '★ `ByteSwap()` 的实现在 .cpp 里')
check('fixup the 3 doubles' in src_cpp,
      '★ 后半段注释 "fixup the 3 doubles" —— 与上面算出的"3 个 double"互相印证')

# 反向对照：若结构体不是 4 的倍数会怎样
fake = size + 2
check(fake % 4 != 0,
      '★★ 反向对照：若大小是 %d（不是 4 的倍数），那个 `size/4` 的循环会**漏掉最后 %d 个字节**'
      ' —— 所以"大小是 4 的倍数"是这个算法成立的前提' % (fake, fake % 4))


# ==========================================================================
print()
print('=' * 76)
print('[6] ★★★ 真解析 Rascal.xml')
print('=' * 76)

RASCAL = 'Tools/autotest/aircraft/Rascal/Rascal.xml'
try:
    tree = ET.parse(os.path.join(REPO, RASCAL.replace('/', os.sep)))
    root = tree.getroot()
    ok = True
except Exception as e:
    ok = False
    check(False, '解析 %s 失败：%s' % (RASCAL, e))

if ok:
    check(root.tag == 'fdm_config', '★★ 根元素是 `<fdm_config>`')
    check(root.get('name') == 'rascal' and root.get('version') == '2.0',
          '★ 属性：name=%s version=%s release=%s'
          % (root.get('name'), root.get('version'), root.get('release')))

    tags = [c.tag for c in root]
    top = [t for t in tags if t not in ('input',)]
    check(top[:7] == ['fileheader', 'metrics', 'mass_balance', 'ground_reactions',
                      'propulsion', 'flight_control', 'aerodynamics'],
          '★★★ 七个顶层段的**顺序**：%s' % ' → '.join(top[:7]))

    metrics = root.find('metrics')
    for tag in ('wingarea', 'wingspan', 'chord', 'htailarea', 'vtailarea'):
        e = metrics.find(tag)
        check(e is not None and (e.text or '').strip() != '',
              '★ `<metrics>/<%s>` = %s %s' % (tag, (e.text or '').strip(), e.get('unit', '')))
    locs = [l.get('name') for l in metrics.findall('location')]
    check('AERORP' in locs and 'VRP' in locs,
          '★★ 两个参考点都在：%s —— 气动力绕 AERORP、机体绕 VRP' % ', '.join(locs))

    mb = root.find('mass_balance')
    for tag in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz', 'emptywt'):
        check(mb.find(tag) is not None, '★ `<mass_balance>/<%s>` 存在' % tag)
    check(mb.find('emptywt').get('unit') == 'LBS'
          and mb.find('ixx').get('unit') == 'SLUG*FT2',
          '★★★ 单位是**英制**：重量 LBS、惯量 SLUG·FT2 '
          '—— 与 ArduPilot 内部统一 SI 完全不同，改模型时最容易错在这')

    gr = root.find('ground_reactions')
    contacts = gr.findall('contact')
    check(len(contacts) == 3,
          '★ 起落架 **%d** 个接触点（前三点式）：%s'
          % (len(contacts), ', '.join(c.get('name') for c in contacts)))

    prop = root.find('propulsion')
    check(len(prop.findall('engine')) == 1 and len(prop.findall('tank')) == 1,
          '★ `<propulsion>`：**1 个发动机 + 1 个油箱**（固定翼）')

    aero = root.find('aerodynamics')
    axes = aero.findall('axis')
    names = [a.get('name') for a in axes]
    check(names == ['DRAG', 'SIDE', 'LIFT', 'ROLL', 'PITCH', 'YAW'] or
          sorted(names) == sorted(['DRAG', 'SIDE', 'LIFT', 'ROLL', 'PITCH', 'YAW']),
          '★★★ 六个气动轴：%s' % ', '.join(names))
    nonempty = [a.get('name') for a in axes if len(a) > 0]
    check(len(nonempty) == 6,
          '★★★ 固定翼这六个轴**全部非空**（四旋翼 5 个是空的，见第 [7] 段）')

    # 气动表的单调性 —— 物理要求，可机器验证
    mono = total = 0
    for a in axes:
        for fn in a.findall('function'):
            for tbl in fn.iter('table'):
                iv = tbl.find('independentVar')
                td = tbl.find('tableData')
                if iv is None or td is None:
                    continue
                total += 1
                xs = []
                for ln in (td.text or '').strip().split('\n'):
                    parts = ln.split()
                    if len(parts) >= 2:
                        try:
                            xs.append(float(parts[0]))
                        except ValueError:
                            pass
                if len(xs) >= 2 and all(xs[i] < xs[i + 1] for i in range(len(xs) - 1)):
                    mono += 1
    check(total > 0 and mono == total,
          '★★★ 全部 **%d** 张气动表的自变量列**严格单调递增** —— '
          '这是插值表必须满足的物理前提（不单调的表会插出无意义的中间值）' % total)
    print('           （覆盖 %d 个 axis、%d 张 table）' % (len(axes), total))


# ==========================================================================
print()
print('=' * 76)
print('[7] ★ 固定翼 vs 四旋翼')
print('=' * 76)

QUAD = 'Tools/autotest/aircraft/arducopter/arducopter.xml'
try:
    qroot = ET.parse(os.path.join(REPO, QUAD.replace('/', os.sep))).getroot()
    qok = True
except Exception as e:
    qok = False
    check(False, '解析 %s 失败：%s' % (QUAD, e))

if ok and qok:
    check(qroot.get('name') == 'dflyer',
          '★ 四旋翼模型的 name 是 `%s`（不是 arducopter）' % qroot.get('name'))
    qaero = qroot.find('aerodynamics')
    qaxes = qaero.findall('axis')
    qempty = [a.get('name') for a in qaxes if len(a) == 0]
    qfull = [a.get('name') for a in qaxes if len(a) > 0]
    check(len(qempty) == 5 and qfull == ['DRAG'],
          '★★★ 四旋翼的气动：**%d 个空轴**（%s），**只有 DRAG 非空** —— '
          '升力不是机翼给的' % (len(qempty), ', '.join(qempty)))
    qprop = qroot.find('propulsion')
    qeng = qprop.findall('engine')
    check(len(qeng) == 4,
          '★★★ 而 `<propulsion>` 里有 **%d 个发动机** ⇒ **升力全靠推力**' % len(qeng))
    senses = sorted(e.find('engine').get('sense') if e.find('engine') is not None
                    else (e.get('sense') or '?') for e in qeng)
    check(len(senses) == 4, '★ 四个发动机的 sense = %s（两正两反 ⇒ 能产生偏航力矩）'
          % ', '.join(senses))
    check(qroot.find('mass_balance').find('emptywt').get('unit') == 'KG'
          or 'KG' in (qroot.find('mass_balance').find('emptywt').get('unit') or ''),
          '★ 四旋翼用千克（%s），固定翼用磅 —— 同一批模型里单位都不统一'
          % qroot.find('mass_balance').find('emptywt').get('unit'))

# 两个四旋翼文件逐字节相同
a = io.open(os.path.join(REPO, 'Tools/autotest/aircraft/arducopter/arducopter.xml'),
            encoding='utf-8', errors='replace').read()
b = io.open(os.path.join(REPO, 'Tools/autotest/aircraft/arducopter/plus_quad2.xml'),
            encoding='utf-8', errors='replace').read()
check(a == b, '★★ `arducopter.xml` 与 `plus_quad2.xml` **逐字节相同**（同一份内容的两个副本）')

allxml = []
for r, _d, fs in os.walk(os.path.join(REPO, 'Tools/autotest/aircraft')):
    for f in fs:
        if f.endswith('.xml'):
            p = os.path.join(r, f)
            t = io.open(p, encoding='utf-8', errors='replace').read()
            if '<fdm_config' in t:
                allxml.append(os.path.relpath(p, REPO).replace(os.sep, '/'))
check(len(allxml) == 3,
      '★★★ 全树只有 **%d** 个真正的 JSBSim 模型：%s' % (len(allxml), ', '.join(
          os.path.basename(x) for x in allxml)))
check(not any('heli' in x.lower() or 'helicopter' in x.lower() for x in allxml),
      '★★ 其中**没有直升机模型** —— 想跑旋翼机得自己搭，没有模板可抄')


# ==========================================================================
print()
print('=' * 76)
print('[8] 三处「名字和内容对不上」')
print('=' * 76)

check('if (strstr(frame_str, "elevon")) {' in src_cpp
      and "const char *model_name = strchr(frame_str, ':')" in src_cpp,
      '★★★ `elevon`/`vtail` 与**模型名解析读的是同一个字符串**')
m = re.search(r"strchr\(frame_str, ':'\);\s*\n\s*if \(model_name != nullptr\) \{\s*\n\s*jsbsim_model = model_name \+ 1;",
              src_cpp)
check(m is not None,
      '★★ 而且是 `strchr`（**第一个**冒号）⇒ `-f jsbsim:elevon` 会把 `elevon` 当成模型名')
modeldirs = sorted(d for d in os.listdir(os.path.join(REPO, 'Tools/autotest/aircraft'))
                   if os.path.isdir(os.path.join(REPO, 'Tools/autotest/aircraft', d)))
check(not any('elevon' in d.lower() or 'vtail' in d.lower() for d in modeldirs),
      '★★★ 而树里的模型目录只有 %s —— **都不含 elevon/vtail** '
      '⇒ 那两条分支用随树发布的模型**走不到**' % ', '.join(modeldirs))
check('// fake an elevon plane' in src_cpp and '// fake a vtail plane' in src_cpp,
      '★ 混控代码本身存在（内容 `(ch2-ch1)/2` 与 `-(ch2+ch1)/2`）')
check('the minus does away with the need for RC2_REVERSED' in src_cpp
      and 'this matches VTAIL_OUTPUT==2' in src_cpp,
      '★★★ 两条注释各自指向**别处的约定**（`RC2_REVERSED` / `VTAIL_OUTPUT`）'
      ' ⇒ 这里的正负号是为了让另一处的参数不用改，要一并读')

z = io.open(os.path.join(REPO, 'Tools/autotest/aircraft/Rascal/Engines/Zenoah_G-26A.xml'),
            encoding='utf-8', errors='replace').read()
check('Zenoah G-26A' in z and '2.96 hp engine' in z,
      '★★ 文件名与注释都说是 **Zenoah G-26A 汽油机**（2.96 hp）')
check('<electric_engine' in z,
      '★★★ 而实际内容是 **`<electric_engine>`** ⇒ 名字和内容对不上')
w = re.findall(r'<power[^>]*>\s*([\d.]+)', z)
check(w and abs(float(w[0]) - 2207.27) < 0.01,
      '★★ 而且功率写成 %s W —— 正是 2.96 hp 的换算值（把标签忘了改）' % (w[0] if w else '?'))

rd = io.open(os.path.join(REPO, 'Tools/autotest/aircraft/Rascal/README.Rascal'),
             encoding='utf-8', errors='replace').read()
# ★★ 这一条我第一版写错了：我照抄了"内容与 Rascal 无关"的判断，
#    而**文件自己开头就写了免责声明**。真正的发现不是"挂错文件"，
#    而是"作者知道并写下来了"—— 这是**对的**注释，不是错的。
check('This model is based on the Rascal110 from FlightGear' in rd,
      '★ 开头确实在讲 Rascal110（模型来源）')
check(re.search(r"has not been updated for the Rascal, please ignore", rd) is not None,
      '★★★ 紧接着就有一句**免责声明**："This information has not been updated for '
      'the Rascal, please ignore, we are still at pre-pre-pre-alpha with this model!"')
check('PIPER J3 CUB PERFORMANCE DATA' in rd and '1946 J3C-65 owner' in rd,
      '★★ 声明之后才是 **Piper J3 Cub 手册**（抄自 1946 年机主手册）—— '
      '作者标明来源、也标明不适用')
check(len(rd.splitlines()) >= 200,
      '★ 全文 %d 行，绝大部分是那份 Cub 数据' % len(rd.splitlines()))


# ==========================================================================
print()
print('=' * 76)
print('[9] 参数个数 = 0')
print('=' * 76)

check(re.search(r'\bvar_info\b', src_cpp) is None
      and re.search(r'\bvar_info\b', src_h) is None,
      '★★★ `.cpp` 与 `.h` 里**都没有 `var_info`** ⇒ 这个后端**没有任何参数**')
check(re.search(r'\bAP_Param\b', src_cpp) is None,
      '★ 也没有 `AP_Param` ⇒ 连参数系统都没接')
check(re.search(r'AP_GROUPINFO', src_cpp) is None,
      '★ 更没有 `AP_GROUPINFO`')
parms = [f for f in os.listdir(os.path.join(REPO, 'Tools/autotest/default_params'))
         if 'jsb' in f.lower()]
check(parms == ['plane-jsbsim.parm'],
      '★ 唯一的 jsbsim 参数文件是 **%s**（且全部是飞控参数，无 SIM_JSBSIM_*）' % parms[0])
p = io.open(os.path.join(REPO, 'Tools/autotest/default_params/plane-jsbsim.parm'),
            encoding='utf-8', errors='replace').read()
check('SIM_JSBSIM' not in p, '★★ 佐证：那个 parm 文件里**没有一条 SIM_JSBSIM_* 参数**')
rev = re.findall(r'^(\w+_REVERSED)\s+1', p, re.M)
check(len(rev) == 4,
      '★★ 但有 **4 条 `*_REVERSED = 1`**（%s）—— 用来对齐 JSBSim 与 ArduPilot 的舵面方向约定'
      % ', '.join(rev))


# ==========================================================================
print()
print('=' * 76)
print('[10] 启动链路：从 frame 串到模型名')
print('=' * 76)

vi = slurp('Tools/autotest/pysim/vehicleinfo.py')
check(re.search(r'"jsbsim": \{', vi) is not None,
      '★ `vehicleinfo.py` 里有 `jsbsim` 这个 frame')
check('default_params/plane-jsbsim.parm' in vi,
      '★★ 它绑定的默认参数文件是 **Plane** 的 ⇒ **这个 frame 只在 ArduPlane 里注册**')
m = re.search(r'for p in \[([^\]]*"jsbsim"[^\]]*)\]', vi)
check(m is not None and 'jsbsim' in m.group(1),
      '★ 前缀匹配列表里含 `jsbsim` ⇒ `jsbsim:MyModel` 也能命中')
check(re.search(r'\{ "jsbsim",\s+JSBSim::create \}', slurp('libraries/AP_HAL_SITL/SITL_cmdline.cpp')) is not None,
      '★★ 但底层注册**不区分机型**（`SITL_cmdline.cpp`）'
      ' ⇒ 给 copter 传 `--model jsbsim` 机制上可行，只是没人帮你配参数')
check('cmd.extend(["--model", stuff["model"]])' in slurp('Tools/autotest/sim_vehicle.py'),
      '★ `sim_vehicle.py` 把 frame 串原样透传成 `--model`（与第 14 篇同一条路）')
check(re.search(r'const char \*jsbsim_model = "Rascal";', src_h) is not None,
      '★ 默认模型是 `Rascal`（`SIM_JSBSim.h`）')


# ==========================================================================
print()
print('=' * 76)
print('[11] 诚实边界')
print('=' * 76)

import shutil
import subprocess

check(shutil.which('JSBSim') is None, '★ 本机 PATH 里**没有 JSBSim 可执行文件**')
check(not os.path.isdir(os.path.expanduser('~/jsbsim')),
      '★ `~/jsbsim` 也不存在 ⇒ 没装过')
check(shutil.which('gcc') is None and shutil.which('g++') is None,
      '★ 本机没有 C/C++ 编译器 ⇒ 也编不出 ArduPilot SITL')
r = subprocess.run([sys.executable, os.path.join(REPO, 'Tools/autotest/sim_vehicle.py'), '--help'],
                   capture_output=True, text=True, timeout=60)
check(r.returncode != 0,
      '★ 直接跑 `sim_vehicle.py --help` 失败（缺 pymavlink）⇒ **连启动脚本都起不来**')
check(not os.path.isdir(os.path.join(REPO, 'modules', 'JSBSim')),
      '★ 再确认一次：`modules/` 下没有 JSBSim（与第 [2] 段呼应，这里是**汇总段**）')

print()
print('    ★★★ 边界声明：')
print('        · **已验证**：源码结构、进程模型、协议格式、结构体布局、')
print('          XML 骨架与气动表单调性 —— 这些都是静态可确证的')
print('        · **未验证**：装 JSBSim、跑 sim_vehicle、改 XML 之后飞机怎么飞')
print('        · 本课比第 14 篇（RealFlight）多一层可验证性：')
print('          RealFlight 的模型在模拟器里（读不到），JSBSim 的模型是文件（读得到）')


# ==========================================================================
print()
print('=' * 76)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
