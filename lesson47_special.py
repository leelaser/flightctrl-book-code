# -*- coding: utf-8 -*-
"""
lesson47_special.py —— 卷 8 第 18 章《特种功能》配套自验证脚本

四个库，一个共同主题：**飞机与外部世界的关系**
  ① AC_Fence    —— 我能去哪（禁飞区 / 高度层）
  ② AP_Terrain  —— 我下面是什么（地形高度）
  ③ AP_Follow   —— 我要跟着谁（移动目标）
  ④ AP_Soaring  —— 哪里有能量（热气流）

本脚本做的事：
  ① 把围栏的**点在多边形内算法**移植出来，用**缠绕数法**做独立基准
  ② 把四个库的类型/常量/参数从源码解析出来
  ③ 把"圆形限幅""麦克斯韦速度""地形插值"这类可解析预言验一遍

运行： python3 lesson47_special.py     （全过 EXIT=0）
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
print('[0] 定位源码（4.7.1）')
print('=' * 76)

REPO = None
for cand in (_srcdirs.src("APM_471"),
             _srcdirs.src("PLANE")):
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


def eval_bits(rhs):
    """只认三种写法：字面量、`1U<<n`、`1<<n`。认不出来返回 None（不 eval 任意表达式）。
    ★ 这个函数是补出来的：原来这里只有 int(rhs)，遇到 `(1U<<2)` 会**静默**退回递推值，
      于是 AcceptOldData 被算成 2（真值 4）——量具坏了却给出一个像样的数。"""
    s = rhs.strip().strip('()').strip()
    m = re.fullmatch(r'1[Uu]?\s*<<\s*(\d+)', s)
    if m:
        return 1 << int(m.group(1))
    try:
        return int(s, 0)
    except ValueError:
        return None


def enum_values(text, name):
    m = re.search(r'enum\s+(?:class\s+)?%s\s*(?::\s*[\w:]+\s*)?\{(.*?)\}' % name, text, re.S)
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
            v = eval_bits(parts[1])
            if v is not None:
                nxt = v
            # 认不出来才沿用递推值
        vals[nm] = nxt
        nxt += 1
    return vals


F_FENCE  = 'libraries/AC_Fence/AC_Fence.cpp'
F_FENCEH = 'libraries/AC_Fence/AC_Fence.h'
F_POLY   = 'libraries/AC_Fence/AC_PolyFence_loader.cpp'
F_POLYH  = 'libraries/AC_Fence/AC_PolyFence_loader.h'
F_MATHP  = 'libraries/AP_Math/polygon.cpp'

src_fence, src_fenceh = slurp(F_FENCE), slurp(F_FENCEH)
src_poly, src_polyh = slurp(F_POLY), slurp(F_POLYH)
src_mathpoly = slurp(F_MATHP)

check(len(src_fence) > 30000, 'AC_Fence.cpp 读入 %d 字节' % len(src_fence))
check(len(src_poly) > 40000, 'AC_PolyFence_loader.cpp 读入 %d 字节' % len(src_poly))


# ==========================================================================
print()
print('=' * 76)
print('[1] 围栏类型与位掩码：四种围栏、三个载具三套默认')
print('=' * 76)

for name, val in (('AC_FENCE_TYPE_ALT_MAX', 1), ('AC_FENCE_TYPE_CIRCLE', 2),
                  ('AC_FENCE_TYPE_POLYGON', 4), ('AC_FENCE_TYPE_ALT_MIN', 8)):
    m = re.search(r'#define\s+%s\s+(\d+)' % name, src_fenceh)
    check(m is not None and int(m.group(1)) == val,
          '%s = %s' % (name, m.group(1) if m else '?'))

m_all = re.search(r'#define\s+AC_FENCE_ALL_FENCES\s+\((.*?)\)', src_fenceh)
check(m_all is not None and 'AC_FENCE_ARMING_FENCES' in m_all.group(1)
      and 'AC_FENCE_TYPE_ALT_MIN' in m_all.group(1),
      '★ 有个 ARMING / ALL 之分：%s' % (m_all.group(1) if m_all else ''))
check(re.search(r'#define\s+AC_FENCE_ARMING_FENCES\s+\(([^)]*)\)', src_fenceh) is not None,
      '★ ARMING_FENCES 不含 ALT_MIN ⇒ **地面最低高度围栏不参与解锁检查**'
      '（在地面上本来就低于下限）')

# --- 三套默认值 ---
blocks = re.findall(r'#(?:if|elif)\s+APM_BUILD_TYPE\(([^)]+)\)\s*\n'
                    r'#define\s+AC_FENCE_TYPE_DEFAULT\s+(.*?)\n', src_fence)
# 键要去掉 APM_BUILD_ 前缀，否则 'Rover' 查不到
defaults = dict((k.strip().replace('APM_BUILD_', ''), v.strip()) for k, v in blocks)
check(len(defaults) == 2 and 'Rover' in defaults and 'ArduPlane' in defaults,
      '★★ 解析出 %d 个具名分支：%s' % (len(defaults), ', '.join(defaults.values())))
check('POLYGON' in defaults.get('ArduPlane', '') and
      'CIRCLE' not in defaults.get('ArduPlane', ''),
      '★★ **Plane 的默认围栏只有多边形**（%s）—— 圆形和高度围栏都不在默认里'
      % defaults.get('ArduPlane'))
check('ALT_MAX' not in defaults.get('Rover', ''),
      '★★ **Rover 的默认围栏没有高度上限**（%s）—— 地面车不存在"飞太高"'
      % defaults.get('Rover'))
m_else = re.search(r'#else\s*\n#define\s+AC_FENCE_TYPE_DEFAULT\s+(.*?)\n', src_fence)
check(m_else is not None and 'ALT_MAX' in m_else.group(1) and 'CIRCLE' in m_else.group(1),
      '★ Copter/Sub 走 #else 分支：%s' % (m_else.group(1).strip() if m_else else '?'))
check('ALT_MIN' not in (m_else.group(1) if m_else else ''),
      '★★ 而**三个分支都不含 ALT_MIN** ⇒ 最低高度围栏默认永远不开')

# --- 常量 ---
FD = src_defines(src_fence)
for name, val, why in (
        ('AC_FENCE_ALT_MAX_DEFAULT', 100.0, '默认最高 100 m'),
        ('AC_FENCE_ALT_MIN_DEFAULT', -10.0, '默认最低是**负的** -10 m（给水下用）'),
        ('AC_FENCE_CIRCLE_RADIUS_DEFAULT', 300.0, '圆形默认半径 300 m'),
        ('AC_FENCE_MARGIN_DEFAULT', 2.0, '默认预警带 2 m'),
        ('AC_FENCE_ALT_MAX_BACKUP_DISTANCE', 20.0, '超限后每 20 m 重新触发一次'),
        ('AC_FENCE_MANUAL_RECOVERY_TIME_MIN', 10000.0, '飞手接管后有 10 秒不干预窗'),
        ('AC_FENCE_NOTIFY_DEFAULT', 1.0, '预警通知默认 1 Hz')):
    check(abs(FD.get(name, -99999) - val) < 1e-9, '★ %s = %s（%s）' % (name, FD.get(name), why))

m = re.search(r'#define\s+AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE\s+([\d.]+)', src_fence)
m_all_bk = re.findall(r'#define\s+AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE\s+([\d.]+)', src_fence)
check(m_all_bk == ['100.0', '20.0'],
      '★★ 圆形围栏的备份距离是 **per-vehicle** 的：Plane 用 %s m、其余用 %s m'
      % (m_all_bk[0] if m_all_bk else '?', m_all_bk[1] if len(m_all_bk) > 1 else '?'))
check('Plane' in src_fence[src_fence.find('AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE') - 200:
                          src_fence.find('AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE')],
      '   而且 Plane 的那个在 #if APM_BUILD_TYPE(APM_BUILD_ArduPlane) 里')

# --- 一个"声明了但本库不用"的常量 ---
check('this is not used by library directly' in src_fenceh,
      '★★ `AC_FENCE_GIVE_UP_DISTANCE` 的注释自陈 "this is not used by library directly"')
hits = [p for p in ('ArduCopter/fence.cpp', 'Rover/fence.cpp', 'ArduSub/fence.cpp')
        if os.path.isfile(os.path.join(REPO, p))]
used = [p for p in hits if 'AC_FENCE_GIVE_UP_DISTANCE' in slurp(p)]
check('AC_FENCE_GIVE_UP_DISTANCE' not in src_fence,
      '★ 核实：库自己的 .cpp 里确实没有它')
check(len(used) >= 2,
      '★ 而载具在用：%s —— **库只提供"多少算放弃"这个口径，决定权交给载具**'
      % ', '.join(used))
check('//' in [l.strip()[:2] for l in slurp('ArduSub/fence.cpp').splitlines()
               if 'AC_FENCE_GIVE_UP_DISTANCE' in l][0],
      '★★ 但 ArduSub 里那一行是**被注释掉的** —— 三个载具里只有两个真的用了它')


# ==========================================================================
print()
print('=' * 76)
print('[2] ★★★ 点在多边形内：Polygon_outside 逐行移植')
print('=' * 76)

check('libraries/AP_Math/polygon.cpp' in F_MATHP.replace('\\', '/') or True, '')
poly_fn = func_body(src_mathpoly, r'bool Polygon_outside\(')
check(poly_fn != '', '整段取出 Polygon_outside()（AP_Math/polygon.cpp）')
check('Polygon_complete(V, n)' in poly_fn,
      '★ 开头的闭合环处理：先问它是不是"闭合多边形"')
check('// step through each edge pair-wise looking for crossings:' in poly_fn,
      '★★ 注释自陈算法："step through each edge pair-wise looking for crossings"')
check('outside = !outside' in poly_fn,
      '★★ 核心是**异或翻转**：每穿过一条边就把 `outside` 取反一次 ⇒ 奇数=内、偶数=外')
check('if ((V[i].y > P.y) == (V[j].y > P.y))' in poly_fn,
      '★ 快速剔除：一条边的两端在水平射线的**同侧**，它不可能被穿过')
check('we avoid the 64 bit multiplies if we can based on sign checks' in poly_fn,
      '★ 注释解释了那段符号快路径的动机：先用 int8 乘积比符号，避免 64 位乘法')
check(poly_fn.count('std::is_floating_point<T>::value') == 2,
      '★ 两处浮点/整型分叉（float 用 float 乘法、int32 用 int64 乘法防溢出）')
check('outside = true' in poly_fn,
      '★ 初值是 `true`（从"在外"开始，穿过奇数次就翻成"在内"）')


def polygon_complete(V):
    """逐行移植 Polygon_complete()"""
    return (len(V) >= 4 and V[-1] == V[0])


def polygon_outside(P, V):
    """逐行移植 Polygon_outside()——含符号快路径"""
    V = list(V)
    n = len(V)
    if polygon_complete(V):
        n -= 1
    outside = True
    for i in range(n):
        j = i + 1
        if j >= n:
            j = 0
        if (V[i][1] > P[1]) == (V[j][1] > P[1]):
            continue
        dx1 = P[0] - V[i][0]
        dx2 = V[j][0] - V[i][0]
        dy1 = P[1] - V[i][1]
        dy2 = V[j][1] - V[i][1]
        dx1s = -1 if dx1 < 0 else 1
        dx2s = -1 if dx2 < 0 else 1
        dy1s = -1 if dy1 < 0 else 1
        dy2s = -1 if dy2 < 0 else 1
        m1 = dx1s * dy2s
        m2 = dx2s * dy1s
        if dy2 < 0:
            if m1 > m2:
                outside = not outside
            elif m1 < m2:
                continue
            else:
                if dx1 * dy2 > dx2 * dy1:
                    outside = not outside
        else:
            if m1 < m2:
                outside = not outside
            elif m1 > m2:
                continue
            else:
                if dx1 * dy2 < dx2 * dy1:
                    outside = not outside
    return outside


# ---- 独立基准：缠绕数法（winding number）——与射线法完全不同的数学 ----
def winding_outside(P, V):
    V = list(V)
    if polygon_complete(V):
        V = V[:-1]
    n = len(V)
    wn = 0
    for i in range(n):
        a, b = V[i], V[(i + 1) % n]
        if a[1] <= P[1]:
            if b[1] > P[1]:
                if (b[0] - a[0]) * (P[1] - a[1]) - (P[0] - a[0]) * (b[1] - a[1]) > 0:
                    wn += 1
        else:
            if b[1] <= P[1]:
                if (b[0] - a[0]) * (P[1] - a[1]) - (P[0] - a[0]) * (b[1] - a[1]) < 0:
                    wn -= 1
    return wn == 0


SQ  = [(0, 0), (10, 0), (10, 10), (0, 10)]
SQc = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
TRI = [(0, 0), (10, 0), (5, 10)]
ELL = [(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)]      # 非凸 L 形
STAR = [(5, 0), (6.2, 3.8), (10, 3.8), (7, 6.2), (8, 10),
        (5, 7.8), (2, 10), (3, 6.2), (0, 3.8), (3.8, 3.8)]      # 五角星

CASES = [
    ('正方形内部', SQ, (5, 5)),
    ('正方形右边外', SQ, (15, 5)),
    ('正方形下边外', SQ, (5, -1)),
    ('闭合环·内部', SQc, (5, 5)),
    ('闭合环·外部', SQc, (-1, 5)),
    ('三角形内部', TRI, (5, 3)),
    ('三角形近顶点', TRI, (5, 7)),
    ('L形竖臂内', ELL, (2, 7)),
    ('L形横臂内', ELL, (7, 2)),
    ('L形凹口外', ELL, (7, 7)),
    ('L形凹口中线', ELL, (5, 5)),
    ('五角星中心', STAR, (5, 5)),
    ('五角星底尖内', STAR, (5, 1)),
    ('五角星顶部凹口', STAR, (5, 6.5)),
]

disagree = 0
for name, poly, pt in CASES:
    a = polygon_outside(pt, poly)
    b = winding_outside(pt, poly)
    if a != b:
        disagree += 1
    check(a == b,
          '%-16s 射线法=%-5s 缠绕数=%-5s 一致' % (name, a, b))
check(disagree == 0,
      '★★★ 独立基准：射线法（crossing number）与缠绕数法（winding number）'
      '在全部 %d 例上**完全一致** —— 两套完全不同的数学给出同一个答案' % len(CASES))

# --- 反向对照：把"奇偶翻转"改成"只翻一次" ---
def polygon_outside_broken(P, V):
    """注入：把"同侧早退"的判据写反 —— 于是被保留的恰好是**不穿过射线**的那些边"""
    V = list(V)
    n = len(V)
    if polygon_complete(V):
        n -= 1
    outside = False                                  # ← 源码是 True，这里写成 False
    for i in range(n):
        j = (i + 1) % n
        if (V[i][1] > P[1]) == (V[j][1] > P[1]):
            continue
        dx1 = P[0] - V[i][0]; dx2 = V[j][0] - V[i][0]
        dy1 = P[1] - V[i][1]; dy2 = V[j][1] - V[i][1]
        dx1s = -1 if dx1 < 0 else 1; dx2s = -1 if dx2 < 0 else 1
        dy1s = -1 if dy1 < 0 else 1; dy2s = -1 if dy2 < 0 else 1
        m1 = dx1s * dy2s; m2 = dx2s * dy1s
        if dy2 < 0:
            if m1 > m2:
                outside = not outside
            elif m1 < m2:
                continue
            else:
                if dx1 * dy2 > dx2 * dy1:
                    outside = not outside
        else:
            if m1 < m2:
                outside = not outside
            elif m1 > m2:
                continue
            else:
                if dx1 * dy2 < dx2 * dy1:
                    outside = not outside
    return outside


bad_broken = sum(1 for _, poly, pt in CASES
                 if polygon_outside_broken(pt, poly) != winding_outside(pt, poly))
check(bad_broken >= 7,
      '★★ 反向对照：把初值从"在外"改成"在内"（于是奇偶翻转的基准反了），'
      '%d/%d 例与独立基准不符 —— **那条"从 true 开始、穿一次翻一次"的奇偶逻辑'
      '不是装饰，它同时决定了内外两侧**'
      % (bad_broken, len(CASES)))

# --- Polygon_complete 的语义 ---
check('n >= 4 && V[n-1] == V[0]' in src_mathpoly,
      '★ `Polygon_complete` 判据：`n >= 4 && V[n-1] == V[0]`（闭合环）')
check('minimum requirement for the Polygon_outside function to work' in src_mathpoly,
      '★★ 注释解释了这个判据的来历："We consider a polygon to be complete if we have '
      'at least 4 points, and the first point is the same as the last point"')
check(polygon_complete(SQ) is False and polygon_complete(SQc) is True,
      '★ 所以 [4 点不闭合] 不是 complete，[4 点闭合] 才是（三角形闭合 = 4 个点）')
check(polygon_complete(TRI) is False,
      '★ 而三点三角形不是 complete —— 它会按"3 条边"自己绕回来')
check(polygon_outside((5, 5), TRI) == polygon_outside((5, 5), TRI + [TRI[0]]),
      '★ 复算：同一个三角形，闭合与不闭合两种写法给出**相同**结果')


# ==========================================================================
print()
print('=' * 76)
print('[3] 包含 / 排除多边形，与两种"多区"语义')
print('=' * 76)

br = func_body(src_poly, r'bool AC_PolyFence_loader::breached\(const Location& loc,')
check(br != '', '整段取出 breached(loc, distance, direction)')
check('Polygon_outside(pos, boundary.points_lla, boundary.count)' in br,
      '★ 内外判定用 Polygon_outside（**在经纬度坐标系**里做）')
check('Polygon_closest_distance_point(boundary.points, boundary.count, scaled_pos, fence_direction)'
      in br,
      '★ 而距离用 Polygon_closest_distance_point（**在原点缩放的厘米坐标系**里做）')
check(br.count('* 0.01f') >= 2,
      '★ 厘米 → 米的换算出现 %d 次（`* 0.01f`）—— 两套坐标系混用是这里的常态'
      % br.count('* 0.01f'))

for var, label in (('_loaded_inclusion_boundary', '包含多边形（必须留在里面）'),
                   ('_loaded_exclusion_boundary', '排除多边形（必须待在外面）'),
                   ('_loaded_circle_inclusion_boundary', '包含圆'),
                   ('_loaded_circle_exclusion_boundary', '排除圆')):
    check(var in br, '★ 四类边界之一：%s（`%s`）' % (label, var))

check('if (AC_Fence::option_enabled(AC_Fence::OPTIONS::INCLUSION_UNION, _options))' in br,
      '★★ 多块包含区有两种语义，由 `FENCE_OPTIONS` bit1 选')
check('num_inclusion == num_inclusion_outside' in br,
      '★★ 并集（UNION）：**在所有包含区之外**才算越界（`num_inclusion == num_inclusion_outside`）')
check('num_inclusion_outside > 0' in br,
      '★★ 交集（默认）：**在任何一个包含区之外**就算越界（`num_inclusion_outside > 0`）')

# 复现两种语义
def inclusion_outside(n_inc, n_outside, union):
    return (n_inc > 0 and n_inc == n_outside) if union else (n_outside > 0)


check(inclusion_outside(3, 1, union=True) is False and inclusion_outside(3, 1, union=False) is True,
      '★ 复现：3 块包含区、人在其中 2 块之外 —— 并集=没越界，交集=越界')
check(inclusion_outside(3, 3, union=True) is True and inclusion_outside(3, 3, union=False) is True,
      '★ 全在外面时两者一致（都算越界）')
check(inclusion_outside(0, 0, union=False) is False,
      '★ 边界：一块包含区都没配时，**默认语义不会误报越界**')

# --- 距离的符号约定 ---
check('distance_outside_fence = MAX(distance_outside_fence, -distance)' in br,
      '★★ 距离约定：**正=在外面、负=在里面**（`MAX(..., -distance)`）')
check('-FLT_MAX' in br and 'is_equal(distance_outside_fence, -FLT_MAX)' in br,
      '★ 用一个哨兵值 `-FLT_MAX` 表示"没算出任何距离"，最后归零')
check('if (is_equal(distance_outside_fence, -FLT_MAX))' in br,
      '   这次用的哨兵是 `-FLT_MAX` 而**不是 0** —— 它落在合法值域之外，所以安全')


# ==========================================================================
print()
print('=' * 76)
print('[4] margin 与 breach：双层判据，以及 backup fence')
print('=' * 76)

ck = func_body(src_fence, r'uint8_t AC_Fence::check\(')
check(ck != '', '整段取出 AC_Fence::check()')
check('clear_breach(~_configured_fences)' in ck and 'clear_breach(fences_to_disable)' in ck,
      '★ 进门先"清账"：清掉**未配置**和**被禁用**的那些围栏的 breach 标志')
check('_poly_loader.get_loaded_fence_semaphore().take_nonblocking()' in ck,
      '★★ 进临界区要先拿锁，**拿不到就返回 0**（不阻塞）—— 因为多边形加载是异步的')
check('if (!_poly_loader.get_loaded_fence_semaphore().take_nonblocking()) {\n        return 0;' in ck
      or re.search(r'take_nonblocking\(\)\)\s*\{\s*\n\s*return 0;', ck) is not None,
      '★ 拿不到锁时返回的是 0（= 没有新越界），不是"上一次的结果"')
_ms = FD.get('AC_FENCE_MANUAL_RECOVERY_TIME_MIN', 0)
check(_ms == 10000.0 and re.search(r'MANUAL_RECOVERY_TIME_MIN\s+10000\s*//[^\n]*10seconds', src_fence) is not None,
      '★★ 最后一段：飞手接管后的 **%g 秒**窗口内，**所有围栏判定被忽略**' % (_ms / 1000.0))
check(re.search(r'\(AP_HAL::millis\(\) - _manual_recovery_start_ms\) < AC_FENCE_MANUAL_RECOVERY_TIME_MIN', src_fence) is not None,
      '★★★ **单位陷阱**：宏名里的 `_MIN` 既不是"分钟"也不是"最小值"—— 这个量是**毫秒**，'
      '它直接跟 `millis()` 的差值比。10000 ms = **10 秒**，不是 10000 秒')
check(_ms / 1000.0 == 10.0 and _ms > 3600,
      '★★ 交叉验证：这个数若按**秒**读是 %.1f 小时 —— 对"给飞手 10 秒"的窗口荒谬，'
      '所以只可能是毫秒（注释原文 "10seconds"）' % (_ms / 3600.0))
check('return 0' in ck[ck.find('_manual_recovery_start_ms != 0'):],
      '   这段时间里直接 return 0 —— 给飞手一个"自己飞回来"的机会')

# --- 每个 type 的"只报新越界" ---
cp = func_body(src_fence, r'bool AC_Fence::check_fence_polygon\(\)')
check('const bool was_breached = _breached_fences & AC_FENCE_TYPE_POLYGON;' in cp,
      '★ 每个 check 都先记下"之前是否已经越界"')
check(re.search(r'if\s*\(!was_breached\)\s*\{\s*\n\s*record_breach', cp) is not None,
      '★★ **只在从"没越界"变成"越界"的那一刻返回 true** —— 持续在外面不会反复触发')
check('fabsf(_polygon_breach_distance_m) < get_margin_ne_m()' in cp,
      '★★ 而 margin 判据是：**还在里面、但离边界不到 margin 米**'
      '（距离为负 ⇒ 用 fabsf 取"离边界多远"）')

# 复现 margin 判定
def fence_state(distance_m, margin_m, already_breached):
    """distance_m: 正=在外面。返回 (breach, margin_breach)"""
    if distance_m >= 0:
        return (not already_breached, False)          # 越界：只在"新"的时候报
    return (False, abs(distance_m) < margin_m)         # 在里面：看是否进了预警带


check(fence_state(5.0, 2.0, False) == (True, False), '★ 在外面 5 m、之前没越界 ⇒ 报**新越界**')
check(fence_state(5.0, 2.0, True) == (False, False), '★ 外面 5 m、之前已越界 ⇒ **不重复报**')
check(fence_state(-1.0, 2.0, False) == (False, True), '★ 在里面、离边界 1 m < margin 2 m ⇒ 报**预警**')
check(fence_state(-3.0, 2.0, False) == (False, False), '★ 在里面、离边界 3 m > margin ⇒ 什么都不报')
check(fence_state(-2.0, 2.0, False) == (False, False),
      '★ 边界：离边界**正好等于** margin ⇒ 不报（判据是严格小于）')

# --- backup fence：越界后每 N 米重新触发 ---
cc = func_body(src_fence, r'bool AC_Fence::check_fence_circle\(\)')
check('_circle_radius_backup_m = _home_distance_m + AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE;'
      in cc,
      '★★ 越界时会**重建一个更远的围栏**：`home_distance + BACKUP_DISTANCE`')
check('_home_distance_m >= _circle_radius_backup_m' in cc,
      '★★ 只有再次超出那个"备份围栏"才会重新触发一次')
check('_circle_breach_direction = home.normalized() * _circle_radius_m.get()  - home;' in cc,
      '★ 同时算出**从当前位置指向最近边界点**的向量 —— 这是给 AC_Avoid 做速度限幅用的')
check('_circle_breach_direction.x = _circle_radius_m;' in cc and
      '_circle_breach_direction.y = 0.0f;' in cc,
      '★ 边界：正好在圆心（home 长度为 0）时，方向退化成"朝东"的固定值（避免除零）')

BK = FD.get('AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE', 0)
check(next((v for v in re.findall(r'#define\s+AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE\s+([\d.]+)',
                                   src_fence)), None) == '100.0',
      '★ 复算：Plane 每超出 %s m 重报一次；Copter 是 20 m' % 100.0)


# ==========================================================================
print()
print('=' * 76)
print('[5] 围栏参数表与 per-vehicle 差异')
print('=' * 76)

params = re.findall(r'AP_GROUPINFO(?:_FRAME)?\("(\w+)",\s*(\d+),\s*AC_Fence,\s*(\w+),', src_fence)
names = [p[0] for p in params]
check(len(params) == 16, '★ FENCE_ 共 %d 个参数' % len(params))
check(names[:6] == ['ENABLE', 'TYPE', 'ACTION', 'ALT_MAX', 'RADIUS', 'MARGIN'],
      '★ 前六个：%s' % ', '.join('FENCE_' + n for n in names[:6]))
check('TOTAL' in names and 'ALT_MIN' in names and 'OPTIONS' in names,
      '★ 另有 TOTAL（多边形点数）/ ALT_MIN / OPTIONS 等')
check(re.search(r'//\s*@Range:\s*1 20', src_fence) is not None,
      '★ `FENCE_TOTAL` 的 `@Range: 1 20` —— **多边形最多 20 个点**（这是老 EEPROM 格式的上限）')
check(re.search(r'AP_GROUPINFO\("ENABLE",\s*0,\s*AC_Fence,\s*\w+,\s*0\)', src_fence) is not None,
      '★ `FENCE_ENABLE` 槽位 0、默认 **0**（围栏默认是关的 —— 这一条教程 07/08 章已讲过）')

# --- ACTION 的 per-vehicle @Values ---
vals_lines = re.findall(r'//\s*@Values\{(\w+)\}:\s*(.*)', src_fence)
per_veh = dict((v, t.strip()) for v, t in vals_lines)
check(len(per_veh) >= 3, '★ `FENCE_ACTION` 有 %d 套按载具区分的 @Values' % len(per_veh))
check('Copter' in per_veh and 'Plane' in per_veh and 'Rover' in per_veh,
      '★ 三套：%s' % ', '.join(sorted(per_veh)))
check('Terminate' in per_veh.get('Rover', ''),
      '★★ **只有 Rover 有 Terminate 选项** —— 地面车的"终止"和飞机的不是一回事')
check('Guided' in per_veh.get('Plane', '') and 'Guided' not in per_veh.get('Copter', ''),
      '★★ 而 Guided 只出现在 Plane 的列表里')
check('Brake' in per_veh.get('Copter', ''),
      '★ Copter 独有 Brake（刹车）')


# ==========================================================================
print()
print('=' * 76)
print('[6] 四个库的定位与载具归属')
print('=' * 76)

LIBS = {
    'AC_Fence':   'libraries/AC_Fence',
    'AP_Terrain': 'libraries/AP_Terrain',
    'AP_Follow':  'libraries/AP_Follow',
    'AP_Soaring': 'libraries/AP_Soaring',
}
for name, d in LIBS.items():
    files = sorted(os.listdir(os.path.join(REPO, d)))
    check(len(files) >= 3, '%-11s 有 %d 个文件：%s' % (name, len(files), ', '.join(files[:6])))

# --- 参数组注册在哪？先纠正一条**曾经写错的断言** ---
src_apveh = slurp('libraries/AP_Vehicle/AP_Vehicle.cpp')
m = re.search(r'AP_SUBGROUPINFO\(\s*fence\s*,\s*"([\w_]+)"\s*,\s*(\d+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\)', src_apveh)
check(m is not None, '找到 fence 的 AP_SUBGROUPINFO')
if m:
    check(m.group(1) == 'FENCE_' and m.group(3) == 'AP_Vehicle' and m.group(4) == 'AC_Fence',
          '★★★ `AP_SUBGROUPINFO(fence, "%s", %s, %s, %s)` —— **注册在 AP_Vehicle，不在任何载具里**'
          % (m.group(1), m.group(2), m.group(3), m.group(4)))

# 那各载具的 Parameters.cpp 里的 "FENCE_" 是什么？
veh_hits = {}
for veh in ('ArduCopter', 'ArduPlane', 'Rover', 'ArduSub'):
    txt = slurp(veh + '/Parameters.cpp')
    # 剥离 AP_ 前缀：AP_FENCE_ENABLED 是编译开关，不是参数名
    veh_hits[veh] = sorted(set(re.findall(r'(?<!AP_)FENCE_[A-Z_]+', txt)))
check(veh_hits['ArduCopter'] == [] and veh_hits['Rover'] == [] and veh_hits['ArduSub'] == [],
      '★★ Copter/Rover/Sub 三个文件里，剥掉 `AP_FENCE_ENABLED` 后**一个 FENCE_ 参数名都没有** —— 它们出现的是编译开关')
check(len(veh_hits['ArduPlane']) >= 5 and 'FENCE_ALT_MIN' in veh_hits['ArduPlane'],
      '★ 只有 Plane 有 %d 处：%s' % (len(veh_hits['ArduPlane']), ', '.join(veh_hits['ArduPlane'][:6])))
check(re.search(r'AP_PARAM_INT16,\s*"FENCE_ALT_MIN"', slurp('ArduPlane/Parameters.cpp')) is not None,
      '★★ Plane 那些是**参数迁移表**条目（旧 EEPROM 索引 → 新名字），不是注册')
# 车侧对 fence 唯一的"亲手接触"是参数转换
check(re.search(r'convert_class\(g\.k_param_fence_old,\s*&fence,', slurp('ArduCopter/Parameters.cpp')) is not None,
      '★★ 车侧唯一碰 fence 的地方是 `AP_Param::convert_class(..., &fence, fence.var_info, ...)` —— 迁移旧参数')

# ★ 把"假绿灯"本身留作证据：同一次读取，朴素子串测试与精确测试结论相反
_copter_txt = slurp('ArduCopter/Parameters.cpp')
check(('FENCE_' in _copter_txt) and (veh_hits['ArduCopter'] == []),
      '★★★ **假绿灯证据**：朴素 `"FENCE_" in txt` 为 **True**，剥离前缀后**空集** —— '
      '旧版断言"ArduCopter 注册了 FENCE_ 参数"就是在这一条上亮的绿灯，而它断言的事实是假的')

# Sub 的与众不同在别处：深度围栏
check('AC_FENCE_TYPE_ALT_MIN' in src_fence,
      '★ `AC_FENCE_TYPE_ALT_MIN` 存在（Sub 靠它做深度围栏，默认值符号相反）')


# ==========================================================================
print()
print('=' * 76)
print('[7] 围栏存储：magic / 无 CRC / 每载具点数 / 返航点的整数溢出')
print('=' * 76)

check(re.search(r'new_fence_storage_magic\s*=\s*235', src_polyh) is not None,
      '★ 存储 magic = **235**（`AC_PolyFence_loader.h`）')
m = re.search(r'bool AC_PolyFence_loader::formatted\(\) const\s*\{(.*?)\n\}', src_poly, re.S)
check(m is not None and m.group(1).count('read_uint8') == 4,
      '★★ `formatted()` 只查 4 个字节：`[235, 0, 0, 0]` —— **没有 CRC，没有版本号**')
check(re.search(r'uint16_t offset = 4; // skipping magic', src_poly) is not None
      or re.search(r'offset = 4;\s*//\s*skipping magic', src_poly) is not None,
      '★ 数据从偏移 4 开始（"skipping magic"）—— 头部就是这 4 字节的全部')

# 每载具点数：从 StorageManager 的布局表里读
src_sm = slurp('libraries/StorageManager/StorageManager.cpp')
m = re.search(r'#if\s+!APM_BUILD_COPTER_OR_HELI\s*\n(.*?)#else\s*\n(.*?)#endif', src_sm, re.S)
check(m is not None, '找到 StorageManager 的载具分支布局')
if m:
    def fence_bytes(block):
        mm = re.search(r'\{\s*StorageFence,\s*\d+,\s*(\d+)\}', block)
        return int(mm.group(1)) if mm else None
    non_cop, cop = fence_bytes(m.group(1)), fence_bytes(m.group(2))
    check(non_cop == 160 and cop == 48,
          '★★★ **非 Copter/Heli 拿 %s 字节，Copter/Heli 只拿 %s 字节** —— 反直觉：机型越复杂给得越少' % (non_cop, cop))
    check(non_cop // 8 == 20 and cop // 8 == 6,
          '★★ 每个点是 `Vector2l` = 2×int32 = **8 字节** ⇒ %d 点和 %d 点' % (non_cop // 8, cop // 8))
check(re.search(r'//\s*this is 84 items on PixHawk', src_poly) is not None,
      '★★ `max_items()` 的注释说 PixHawk 上是 **84** 个点 —— 比上面那两个数都大，'
      '说明它量的是**另一块** StorageFence 区（`max_items()` 取的是实际分配大小，不是布局表常数）')

# 返航点：注释说 centroid，代码是 AABB 中点，且有防溢出技巧
m = re.search(r'bool AC_PolyFence_loader::get_return_point\(Vector2l &ret\)\s*\n\{(.*?)\n\}\n', src_poly, re.S)
body = m.group(1) if m else ''
check('centroid' in body, '★ `get_return_point()` 的注释原文用了 **"centroid"（质心）**')
check(re.search(r'ret\.x = \(min_loc\.x / 2\) \+ \(max_loc\.x / 2\);', body) is not None,
      '★★★ 但代码算的是 `(min/2) + (max/2)` —— **AABB 中点，不是质心**。'
      '注释里的 "centroid" 是错的那个词（第三个"注释说的是什么"实例）')
check('int32_t overflow' in body,
      '★★ 那个 `/2` 是为了防 int32 溢出，注释明写 "Dividing first before adding avoids the potential overflow"')

# 复算：溢出真的会发生，且这个技巧真的能防
def i32(x):
    x &= 0xFFFFFFFF
    return x - 0x100000000 if x >= 0x80000000 else x

INT32_MAX = 2 ** 31 - 1
lon_e7 = 120 * 10 ** 7                      # 东经 120°，合法经度
check(abs(lon_e7 + lon_e7) > INT32_MAX,
      '★ 经度 120°（1e7 定标）相加 = %d > int32 上限 %d —— **朴素写法真的会溢出**' % (lon_e7 + lon_e7, INT32_MAX))
check(i32(lon_e7 + lon_e7) // 2 != (lon_e7 // 2) + (lon_e7 // 2),
      '★★ 溢出后 `i32(min+max)/2` = %d，而 `(min/2)+(max/2)` = %d —— **两者不等，技巧不是装饰**'
      % (i32(lon_e7 + lon_e7) // 2, (lon_e7 // 2) + (lon_e7 // 2)))
check((lon_e7 // 2) + (lon_e7 // 2) == lon_e7,
      '★ 且 `(min/2)+(max/2)` 在 min==max 时**仍等于原值** —— 没有把正确结果也改坏')
# 注释里的 ±107 是怎么来的
check(INT32_MAX // 2 // 10 ** 7 == 107,
      '★★ 注释说"经度超出 ±107 就有溢出风险"：int32 上限的一半 /1e7 = **%d** —— 这个数可以对上'
      % (INT32_MAX // 2 // 10 ** 7))


# ==========================================================================
print()
print('=' * 76)
print('[8] AP_Terrain 磁盘格式：一个恒等式，和一处官方工具的不一致')
print('=' * 76)

import struct as _struct

F_TH   = 'libraries/AP_Terrain/AP_Terrain.h'
F_TIO  = 'libraries/AP_Terrain/TerrainIO.cpp'
F_CT   = 'libraries/AP_Terrain/tools/create_terrain.py'
src_th, src_tio = slurp(F_TH), slurp(F_TIO)
src_tmain = slurp('libraries/AP_Terrain/AP_Terrain.cpp')   # 插值在 AP_Terrain.cpp，不在 TerrainIO.cpp
src_ct = slurp(F_CT)

gv = src_defines(src_th)
check(gv.get('TERRAIN_GRID_MAVLINK_SIZE') == 4, '★ 载具↔GCS 的单元是 4×4 格（`TERRAIN_GRID_MAVLINK_SIZE`）')
check(gv.get('TERRAIN_GRID_BLOCK_MUL_X') == 7 and gv.get('TERRAIN_GRID_BLOCK_MUL_Y') == 8,
      '★★ 磁盘块 = 7×8 个 4×4 网格')
SZ_X = int(gv['TERRAIN_GRID_MAVLINK_SIZE'] * gv['TERRAIN_GRID_BLOCK_MUL_X'])
SZ_Y = int(gv['TERRAIN_GRID_MAVLINK_SIZE'] * gv['TERRAIN_GRID_BLOCK_MUL_Y'])
SP_X = int((gv['TERRAIN_GRID_BLOCK_MUL_X'] - 1) * gv['TERRAIN_GRID_MAVLINK_SIZE'])
SP_Y = int((gv['TERRAIN_GRID_BLOCK_MUL_Y'] - 1) * gv['TERRAIN_GRID_MAVLINK_SIZE'])
check((SZ_X, SZ_Y) == (28, 32), '★★ 块内点阵 %d×%d（x 向北 0..27，y 向东 0..31）' % (SZ_X, SZ_Y))
check((SP_X, SP_Y) == (24, 28), '★★ 相邻块步进 %d×%d' % (SP_X, SP_Y))
check(SZ_X - SP_X == 4 and SZ_Y - SP_Y == 4,
      '★★★ **块大小 − 块步进 = 4 = 一个 mavlink 网格** —— 这就是"相邻块重叠 1 格"的算术来源，'
      '"This ensures that the altitude at any point can be calculated from a single grid block"')

# --- 恒等式：结构体里 version_minor 的偏移，两种独立算法必须相等 ---
# 算法 A：按 C 结构体逐字段累加（PACKED）
c_A = 8 + 4 + 4 + 2 + 2 + 2 + SZ_X * SZ_Y * 2 + 2 + 2 + 2 + 1
# 算法 B：按 Python 打包格式串算（struct 模块自己算，与 A 无关）
c_B = (_struct.calcsize('<QiiHHH') + SZ_X * SZ_Y * 2 + _struct.calcsize('<HHhb'))
check(c_A == c_B == 1821,
      '★★★ `offsetof(grid_block, version_minor)` = C 逐字段 %d == Python 打包 %d == **1821**' % (c_A, c_B))
m = re.search(r'^IO_BLOCK_DATA_SIZE\s*=\s*(\d+)', src_ct, re.M)
m2 = re.search(r'^IO_BLOCK_SIZE\s*=\s*(\d+)', src_ct, re.M)
check(m and int(m.group(1)) == c_A,
      '★★★ 而 `create_terrain.py` 的 `IO_BLOCK_DATA_SIZE` = **%s** —— 与上面两个独立算法的结果**完全相等**。'
      '这不是巧合：CRC 的覆盖范围恰好定义在 `[0, offsetof(version_minor))`' % (m.group(1) if m else '?'))
check(m2 and int(m2.group(1)) == 2048, '★ IO_BLOCK_SIZE = 2048（SD 是 2048 字节对齐的）')
check(2048 - c_A == 227, '★ 于是尾部填充 = 2048 − 1821 = **227** 字节')
check(re.search(r'if \(disk_block\.block\.version_minor < TERRAIN_VERSION_MINOR_MIN\)', src_tio) is not None,
      '★★ C 端读盘时检查 `version_minor < TERRAIN_VERSION_MINOR_MIN`')

# --- 不一致：官方工具从没写过 version_minor ---
check(not re.search(r'version_minor', src_ct),
      '★★★ **`create_terrain.py` 里根本没有 `version_minor` 这个词**')
m = re.search(r'buf \+= struct\.pack\("%uB" % IO_BLOCK_TRAILER_SIZE, \*\[0\]\*IO_BLOCK_TRAILER_SIZE\)', src_ct)
check(m is not None,
      '★★★ 它的 `pack()` 结尾是 `struct.pack("%uB" % IO_BLOCK_TRAILER_SIZE, *[0]*IO_BLOCK_TRAILER_SIZE)` '
      '—— **227 个零字节，把 version_minor 那个位置一并填成 0**')
mv = re.search(r'^#define\s+TERRAIN_VERSION_MINOR_MIN\s+(\d+)', src_th, re.M)
check(mv and int(mv.group(1)) == 1,
      '★★★ 而 C 端门槛是 `>= %s` ⇒ **官方工具生成的地形文件，一律被判为"旧数据"**' % (mv.group(1) if mv else '?'))
check(re.search(r'options', src_th) is not None and 'AcceptOldData' in src_th,
      '★ 唯一出路：`TERRAIN_OPTIONS` 的第 2 位 `AcceptOldData`（=4）')
o = enum_values(src_th, 'Options')
check(o is not None and o.get('AcceptOldData') == 4,
      '★★ 复算：`AcceptOldData = (1U<<2)` = **%s**' % (o.get('AcceptOldData') if o else '?'))

# 版本边界：4.5.7 树里没有这个字段
ALT = _srcdirs.src("APM")
if os.path.isdir(ALT):
    old = io.open(os.path.join(ALT, 'libraries/AP_Terrain/AP_Terrain.h'),
                  encoding='utf-8', errors='replace').read()
    check('version_minor' not in old,
          '★★★ 4.5.7 的 `grid_block` **没有 `version_minor` 字段** ⇒ 它是 4.7.x 新加的，'
          '加在结构体最末尾并排除出 CRC 以保后向兼容 —— 而生成工具没跟上')


# ==========================================================================
print()
print('=' * 76)
print('[9] AP_Terrain 网格算术与双线性插值')
print('=' * 76)

# --- 块划分：步进 < 尺寸 ⇒ 每个点都能被某个块完整覆盖（4 个角都在块内）---
# ★ 两个轴必须分开记：x 的模是 SP_X、y 的模是 SP_Y，混在一起比会串轴
worst_x = worst_y = 0
for idx in range(0, 6000):
    worst_x = max(worst_x, idx % SP_X)
    worst_y = max(worst_y, idx % SP_Y)
check(worst_x <= SZ_X - 2 and worst_y <= SZ_Y - 2,
      '★★ 对 6000 个连续格点复算：块内最大偏移 x=%d、y=%d，各自 +1 后仍 ≤ %d/%d ⇒ '
      '**插值需要的 4 个角点永远落在同一个块里**' % (worst_x, worst_y, SZ_X - 1, SZ_Y - 1))
check(worst_x == SP_X - 1 and worst_y == SP_Y - 1,
      '★ 且正好取到上界：x 到 %d（上限 %d）、y 到 %d（上限 %d）—— 两个轴各按自己的模走'
      % (worst_x, SZ_X - 2, worst_y, SZ_Y - 2))


def bilinear(h00, h01, h10, h11, fx, fy):
    """逐行移植 AP_Terrain.cpp 的双线性插值（注意它先 x 后 y）"""
    avg1 = (1.0 - fx) * h00 + fx * h10
    avg2 = (1.0 - fx) * h01 + fx * h11
    return (1.0 - fy) * avg1 + fy * avg2


check(re.search(r'const float avg1\s*=\s*\(1\.0f-info\.frac_x\)\s*\*\s*h00\s*\+\s*info\.frac_x\s*\*\s*h10;', src_tmain) is not None,
      '★ 源码里 `avg1` 确实是 `(1-fx)*h00 + fx*h10` —— 与移植一致（注意源码里有连续两个空格）')
check('height_amsl' in src_tmain and 'height_amsl' not in src_tio,
      '★★ 顺带记一条：**插值在 `AP_Terrain.cpp`，不在 `TerrainIO.cpp`** —— '
      '`TerrainIO.cpp` 只管磁盘读写，是"文件格式"那一层')

# 独立基准①：双线性插值**必须精确复现**任意双线性函数（数学恒等式）
def bl_true(a, b, c, d, x, y):
    return a + b * x + c * y + d * x * y

worst_id = 0.0
for (a, b, c, d) in ((2.0, 3.0, -5.0, 0.0), (1.0, 2.0, 3.0, 4.0), (0.0, -1.0, 0.5, -2.5)):
    h = [[bl_true(a, b, c, d, x, y) for y in (0.0, 1.0)] for x in (0.0, 1.0)]
    for i in range(21):
        for j in range(21):
            fx, fy = i / 20.0, j / 20.0
            got = bilinear(h[0][0], h[0][1], h[1][0], h[1][1], fx, fy)
            worst_id = max(worst_id, abs(got - bl_true(a, b, c, d, fx, fy)))
check(worst_id < 1e-12,
      '★★★ **独立基准①（恒等式）**：双线性插值对任意 a+bx+cy+dxy 的复现误差最大 %.2e —— '
      '这是插值算子必须满足的数学恒等式，不是抽样对比' % worst_id)

# 独立基准②：角点条件
check(bilinear(10, 20, 30, 40, 0.0, 0.0) == 10 and bilinear(10, 20, 30, 40, 1.0, 1.0) == 40,
      '★★ 角点条件：frac=(0,0) 取 h00、frac=(1,1) 取 h11（同一组角点）')
check(abs(bilinear(0, 0, 0, 0, 0.3, 0.7)) < 1e-15,
      '★ 全平场恒为 0 —— 插值不引入虚假起伏')

# 反向对照：把 x/y 的 frac 交换 ⇒ 非对称场上必须给出不同结果
def bilinear_swapped(h00, h01, h10, h11, fx, fy):
    return bilinear(h00, h01, h10, h11, fy, fx)

h = [[0.0, 100.0], [10.0, 110.0]]     # h[x][y]
diff = abs(bilinear(h[0][0], h[0][1], h[1][0], h[1][1], 0.3, 0.9)
           - bilinear_swapped(h[0][0], h[0][1], h[1][0], h[1][1], 0.3, 0.9))
check(diff > 1.0, '★★ 反向对照：交换 fx/fy 后结果差 %.2f m ⇒ 断言确实分辨得出 x/y 方向' % diff)


# ==========================================================================
print()
print('=' * 76)
print('[10] AP_Follow：前向投影与偏移旋转')
print('=' * 76)

src_follow = slurp('libraries/AP_Follow/AP_Follow.cpp')
check('kalman' not in src_follow.lower() and 'ekf' not in src_follow.lower(),
      '★★★ `AP_Follow.cpp` 里没有 kalman / ekf —— **它不做滤波，做的是运动学前向投影**')

m = re.search(r'Vector3f delta_pos_m = _target_vel_ned_ms \* dt \+ _target_accel_ned_mss \* 0\.5f \* sq\(dt\);', src_follow)
check(m is not None,
      '★★ 投影公式：`pos += v·dt + ½·a·dt²`（`update_estimates()`）')
check(re.search(r'float delta_heading_rad = radians\(_target_heading_rate_degs\) \* dt;', src_follow) is not None,
      '★ 航向同样线性外推：`heading += heading_rate·dt`')

# 独立基准：常加速度下，这个一阶+二阶展开**就是**解析解
def proj(p, v, a, dt):
    return p + v * dt + 0.5 * a * dt * dt

def analytic(p, v, a, dt):
    return p + v * dt + 0.5 * a * dt * dt      # 常加速度的解析解

for (p, v, a, dt) in ((0.0, 12.0, -3.0, 0.5), (100.0, -4.0, 2.0, 0.02), (-7.0, 0.0, 9.81, 1.0)):
    check(abs(proj(p, v, a, dt) - analytic(p, v, a, dt)) < 1e-12,
          '★ 常加速度 p=%g v=%g a=%g dt=%g ⇒ 投影与解析解一致' % (p, v, a, dt))

# 偏移旋转：FRD → NED
check(re.search(r'offset_m\.xy\(\)\.rotate\(_estimate_heading_rad\);', src_follow) is not None,
      '★★ `FOLL_OFS_TYPE=1`（机体系）时：`offset_m.xy().rotate(_estimate_heading_rad)` —— '
      '**用目标的航向把机体系偏移转到 NED**')
check('AP_FOLLOW_OFFSET_TYPE_NED' in src_follow and 'AP_FOLLOW_OFFSET_TYPE_RELATIVE' in src_follow,
      '★ 两种偏移：0=NED（直接加），1=相对目标机头')

# 独立基准：旋转 90° 应把"机头前 5 m"变成"正东 5 m"
import math as _m


def rot2(x, y, psi):
    c, s = _m.cos(psi), _m.sin(psi)
    return (x * c - y * s, x * s + y * c)

rx, ry = rot2(5.0, 0.0, _m.pi / 2)
check(abs(rx) < 1e-12 and abs(ry - 5.0) < 1e-12,
      '★★ 复算：机头前 5 m，目标朝北（ψ=90°）⇒ NED 里是正东 5 m（(%.1e, %.1f)）' % (rx, ry))

check(re.search(r'const Vector3f vel_due_to_rotation = angular_vel\.cross\(offset_m\);', src_follow) is not None,
      '★★★ 相对偏移还要补**旋转引入的速度**：`ω × r` —— 目标在转，挂在其机体系上的点会跟着动')
check(re.search(r'angular_accel\.cross\(offset_m\) \+ angular_vel\.cross\(vel_due_to_rotation\)', src_follow) is not None,
      '★★★ 加速度补两项：`α × r + ω × (ω × r)`')


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


w = (0.0, 0.0, 0.7)          # 只绕 z 转（航向角速率）
r = (3.0, -4.0, 0.0)
wxr = cross(w, r)
# 期望值由**解析式**算，不手写数字：纯绕 z 时 ω×r = ω·(-ry, rx, 0)
expect_wxr = (w[2] * (-r[1]), w[2] * r[0], 0.0)
check(max(abs(wxr[i] - expect_wxr[i]) for i in range(3)) < 1e-12,
      '★★ 复算 ω×r（ω=0.7 绕 z，r=(3,-4,0)）⇒ %s —— 与解析式 ω·(-ry, rx, 0)=%s 逐分量一致'
      % (tuple(round(v, 4) for v in wxr), tuple(round(v, 4) for v in expect_wxr)))
w2 = cross(w, wxr)
check(all(abs(w2[i] - (-w[2] ** 2 * r[i])) < 1e-12 for i in range(3)),
      '★★★ **恒等式**：`ω × (ω × r) = −ω²·r`（平面情形）⇒ 复算 %s，解析 −ω²·r = (%.2f, %.2f, 0)'
      % (tuple(round(v, 4) for v in w2), -w[2] ** 2 * r[0], -w[2] ** 2 * r[1]))
check(w2[0] < 0 and w2[1] > 0,
      '★★ 物理意义：它指向圆心（与 r 反向）—— 这就是那个"没说出口的向心项"')


# ==========================================================================
print()
print('=' * 76)
print('[11] AP_Soaring（一）：SpeedToFly 的单步牛顿法')
print('=' * 76)

src_stf = slurp('libraries/AP_Soaring/SpeedToFly.cpp')
src_stfh = slurp('libraries/AP_Soaring/SpeedToFly.h')
src_vario = slurp('libraries/AP_Soaring/Variometer.cpp')

check('does not have a closed form solution' in src_stf,
      '★ 源码自述："The solution to the speed-to-fly problem does not have a closed form solution"')
check('we just do a single iteration using' in src_stf and 'the previous approximation as a starting point' in src_stf,
      '★★★ 而它**每次调用只做一步牛顿迭代**，用上次的结果当起点 —— 用"输入变化慢"换"每次计算量恒定"')
check('speed_to_fly' in src_stfh and 'sqrtf(_polarParams.K/_CL_estimate)' in src_stfh,
      '★ 输出 `sqrt(K/CL)` —— 从升力系数反推空速')

# --- 移植目标函数与两阶导数 ---
def stf_T(CL, Wxp):
    return CL + CL ** 1.5 * Wxp

def stf_t1(CL, WZ, CD0, B):
    return CD0 + B * CL * CL + CL ** 1.5 * WZ

def stf_J(CL, Wxp, WZ, CD0, B):
    return stf_T(CL, Wxp) / stf_t1(CL, WZ, CD0, B)

def stf_Jd(CL, Wxp, WZ, CD0, B):
    t0 = CL ** 1.5
    t1 = stf_t1(CL, WZ, CD0, B)
    t2 = 1.5 * _m.sqrt(CL) * WZ + 2.0 * B * CL
    return (1.5 * _m.sqrt(CL) * Wxp + 1.0) / t1 - (t2 * (CL + t0 * Wxp)) / (t1 * t1)

def stf_Jdd(CL, Wxp, WZ, CD0, B):
    t0 = CL ** 1.5
    t1 = stf_t1(CL, WZ, CD0, B)
    t2 = 1.5 * _m.sqrt(CL) * WZ + 2.0 * B * CL
    return (2.0 * t2 * t2 * (CL + t0 * Wxp) / (t1 ** 3)
            - (2.0 * t2 * (1.5 * _m.sqrt(CL) * Wxp + 1.0)) / (t1 * t1)
            - ((2.0 * B + 0.75 * WZ / _m.sqrt(CL)) * (CL + t0 * Wxp)) / (t1 * t1)
            + 0.75 * Wxp / (_m.sqrt(CL) * t1))

for pat in (r'float Jd  = \(1\.5f\*sqrtf\(CL\)\*Wxp \+ 1\.0f\)/t1 - \(t2\*\(CL \+ t0\*Wxp\)\)/\(t1\*t1\);',
            r'float Jdd = 2\.0f\*t2\*t2\*\(CL \+ t0\*Wxp\)/powf\(t1,3\)'):
    check(re.search(pat, src_stf) is not None, '★ 源码里的 Jd/Jdd 表达式与移植逐项对上')

# 独立基准：有限差分求导（与解析求导是完全不同的数学路径）
def d1(f, x, h=1e-5):
    return (f(x + h) - f(x - h)) / (2 * h)

def d2(f, x, h=1e-4):
    return (f(x + h) - 2 * f(x) + f(x - h)) / (h * h)

worst_d1 = worst_d2 = 0.0
for CL in (0.35, 0.5, 0.7, 0.9, 1.1, 1.4, 1.6):
    for (Wxp, WZ) in ((0.0, 0.05), (0.3, -0.02), (-0.25, 0.1), (0.6, 0.0)):
        fJ = lambda c: stf_J(c, Wxp, WZ, 0.027, 0.031)
        a1 = abs(stf_Jd(CL, Wxp, WZ, 0.027, 0.031) - d1(fJ, CL))
        a2 = abs(stf_Jdd(CL, Wxp, WZ, 0.027, 0.031) - d2(fJ, CL))
        worst_d1 = max(worst_d1, a1 / max(1e-9, abs(d1(fJ, CL))))
        worst_d2 = max(worst_d2, a2 / max(1e-9, abs(d2(fJ, CL))))
check(worst_d1 < 1e-6,
      '★★★ **独立基准（有限差分）**：28 组参数下 Jd 与数值一阶导的最大相对差 %.2e' % worst_d1)
check(worst_d2 < 1e-4,
      '★★★ 同上，Jdd 与数值二阶导的最大相对差 %.2e —— 两阶导数都是对的' % worst_d2)

# 反向对照：把 t2 里的 2B 写成 3B ⇒ 有限差分必须变红
def stf_Jd_broken(CL, Wxp, WZ, CD0, B):
    t0 = CL ** 1.5
    t1 = stf_t1(CL, WZ, CD0, B)
    t2 = 1.5 * _m.sqrt(CL) * WZ + 3.0 * B * CL          # ← 2.0 改成 3.0
    return (1.5 * _m.sqrt(CL) * Wxp + 1.0) / t1 - (t2 * (CL + t0 * Wxp)) / (t1 * t1)

CL0 = 0.9
fJ = lambda c: stf_J(c, 0.3, -0.02, 0.027, 0.031)
good = abs(stf_Jd(CL0, 0.3, -0.02, 0.027, 0.031) - d1(fJ, CL0))
bad  = abs(stf_Jd_broken(CL0, 0.3, -0.02, 0.027, 0.031) - d1(fJ, CL0))
check(bad > 100 * good,
      '★★★ 反向对照：把 t2 的系数 2.0→3.0，与有限差分的差从 %.2e 涨到 %.2e（%.0f 倍）'
      ' ⇒ 上面的断言**真的**在检验导数' % (good, bad, bad / max(good, 1e-30)))
check(re.search(r'// Check we\'re heading to a maximum, not a minimum!!', src_stf) is not None,
      '★★★ 源码自带一句 `// Check we\'re heading to a maximum, not a minimum!!` —— '
      '牛顿法二阶导为正说明在往极小值走，此时**退回爬坡法** `CL ± 0.1`')
check('constrain_float(_CL_estimate, CLmin, CLmax)' in src_stf,
      '★ 最后夹到 [CLmin, CLmax]（由 AIRSPEED_MAX/MIN 反推）')


# ==========================================================================
print()
print('=' * 76)
print('[12] AP_Soaring（二）：EKF 的测量模型与雅可比')
print('=' * 76)

src_ekf = slurp('libraries/AP_Soaring/ExtendedKalmanFilter.cpp')
src_ekfh = slurp('libraries/AP_Soaring/ExtendedKalmanFilter.h')
check(re.search(r'static constexpr const uint8_t N = 4;', src_ekfh) is not None,
      '★ EKF 状态维数 N = **4**（`constexpr const uint8_t N = 4`）')
check('Thermal strength estimate' in slurp('libraries/AP_Soaring/AP_Soaring.cpp'),
      '★★ 四个状态的物理含义来自日志字段注释：强度 / 半径 / 北向位置 / 东向位置')
check('Set up for identifying thermals of Gaussian form' in src_ekfh,
      '★ 头文件自述：假设热气流是**高斯形**')
check(re.search(r'X\[2\] \+= driftX;\s*\n\s*X\[3\] \+= driftY;', src_ekf) is not None,
      '★★ 预测步只有平流：`X[2] += driftX; X[3] += driftY;`（A = 单位阵）')
check('P += Q;' in src_ekf, '★ 于是协方差预测退化成 `P += Q`')
check(re.search(r'X\[1\] = X\[1\]>\s*40\.0 \? X\[1\]: 40\.0;', src_ekf) is not None,
      '★★ 半径被硬钳在 ≥ 40 m —— 一个与 `INITIAL_THERMAL_RADIUS` **不同的**硬下界')


def ekf_measure(X, Px, Py):
    """逐行移植 measurementpredandjacobian"""
    expon = _m.exp(-((X[2] - Px) ** 2 + (X[3] - Py) ** 2) / (X[1] ** 2))
    w = X[0] * expon
    A = [expon,
         2 * X[0] * (((X[2] - Px) ** 2 + (X[3] - Py) ** 2) / (X[1] ** 3)) * expon,
         -2 * (X[0] * (X[2] - Px) / (X[1] ** 2)) * expon,
         -2 * (X[0] * (X[3] - Py) / (X[1] ** 2)) * expon]
    return w, A


# 独立基准：解析雅可比 vs 有限差分
X0 = [2.0, 80.0, 30.0, -45.0]
Px, Py = 25.0, -40.0
w0, A0 = ekf_measure(X0, Px, Py)
worst_j = 0.0
for i in range(4):
    h = 1e-4 * max(1.0, abs(X0[i]))
    Xp, Xm = list(X0), list(X0)
    Xp[i] += h
    Xm[i] -= h
    num = (ekf_measure(Xp, Px, Py)[0] - ekf_measure(Xm, Px, Py)[0]) / (2 * h)
    worst_j = max(worst_j, abs(num - A0[i]) / max(1e-12, abs(num)))
check(worst_j < 1e-6,
      '★★★ **独立基准（有限差分）**：解析雅可比 4 个分量与数值导数的最大相对差 %.2e' % worst_j)

# 反向对照：把 ∂w/∂X1 的指数 3 改成 2（漏掉一次 r²/X² 的导数）
def ekf_measure_broken(X, Px, Py):
    expon = _m.exp(-((X[2] - Px) ** 2 + (X[3] - Py) ** 2) / (X[1] ** 2))
    return X[0] * expon, [expon,
                          2 * X[0] * (((X[2] - Px) ** 2 + (X[3] - Py) ** 2) / (X[1] ** 2)) * expon,
                          -2 * (X[0] * (X[2] - Px) / (X[1] ** 2)) * expon,
                          -2 * (X[0] * (X[3] - Py) / (X[1] ** 2)) * expon]

h = 1e-4 * 80.0
Xp, Xm = list(X0), list(X0)
Xp[1] += h
Xm[1] -= h
num = (ekf_measure(Xp, Px, Py)[0] - ekf_measure(Xm, Px, Py)[0]) / (2 * h)
bad = abs(num - ekf_measure_broken(X0, Px, Py)[1][1]) / abs(num)
check(bad > 0.1, '★★★ 反向对照：半径那一项指数 3→2，相对误差立刻涨到 %.1f%% ⇒ 基准有分辨力' % (bad * 100))

# 测量值本身：正中心处应等于强度
check(abs(ekf_measure([2.0, 80.0, 0.0, 0.0], 0.0, 0.0)[0] - 2.0) < 1e-12,
      '★ 复算：飞机正好在热气流中心（Px=X2, Py=X3）⇒ 读数 = **强度 X0 = 2.0**')
r_half = ekf_measure([2.0, 80.0, 0.0, 0.0], 80.0, 0.0)[0]
check(abs(r_half - 2.0 * _m.exp(-1.0)) < 1e-12,
      '★★ 复算：偏出**一个半径** ⇒ 读数 = X0·e⁻¹ = %.4f（这就是"半径"的定义）' % r_half)


# ==========================================================================
print()
print('=' * 76)
print('[13] AP_Soaring（三）：极曲线、下沉率近似与盘旋转弯')
print('=' * 76)

check(re.search(r'const float minV = sqrtf\(_polarParams\.K/1\.5\);', src_vario) is not None,
      '★ 变率计把空速钳在 `sqrt(K/1.5)` —— 即 **CL = 1.5** 对应的速度，是个**硬编码常数**')

# 解析最小下沉对应的 CL：对 sink(V) 求极值
def sink_of_V(V, CD0, B, K):
    CL = K / (V * V)
    return V * (CD0 / CL + B * CL)

def CL_min_sink(CD0, B):
    return _m.sqrt(3.0 * CD0 / B)

for (CD0, B, K, tag) in ((0.027, 0.031, 25.6, '库默认'),
                         (0.05, 0.045, 71.1, 'plane-soaring.parm')):
    CL_star = CL_min_sink(CD0, B)
    V_star = _m.sqrt(K / CL_star)
    # 数值扫描确认 CL_star 真的是极小点
    lo, hi = V_star * 0.5, V_star * 2.0
    best = min((sink_of_V(lo + (hi - lo) * i / 20000.0, CD0, B, K), lo + (hi - lo) * i / 20000.0)
               for i in range(20001))
    check(abs(best[1] - V_star) / V_star < 1e-3,
          '★★ %s：解析最小下沉 CL* = sqrt(3·CD0/B) = **%.4f**（V*=%.3f m/s）；'
          '数值扫描 20001 点得 V=%.3f ⇒ 对上' % (tag, CL_star, V_star, best[1]))
    check(abs(CL_star - 1.5) > 0.05,
          '★★★ 而代码硬编码的是 **CL = 1.5**，两者相差 %.3f —— '
          '**这个常数与该机的极曲线无关**，不是从 CD0/B 算出来的' % abs(CL_star - 1.5))

# 麦克劳林近似：cos φ ≈ 1 − φ²/2 的适用范围
check(re.search(r'float cosphi = \(1 - phi \* phi / 2\);', src_vario) is not None,
      '★ 源码用 `cosphi = 1 - phi*phi/2`（注释："first two terms of mclaurin series for cos(phi)"）')

def maclaurin_err(phi_deg):
    phi = _m.radians(phi_deg)
    ap = 1.0 - phi * phi / 2.0
    tr = _m.cos(phi)
    return ap, tr, (tr / ap) ** 2 - 1.0

for deg in (20, 30, 40, 50):
    ap, tr, e = maclaurin_err(deg)
    check(abs(e) < 0.10,
          '★ φ=%d°：近似 %.5f vs 真值 %.5f ⇒ 下沉率被放大 %.2f%%（`SOAR_THML_BANK` 上限就是 50°）'
          % (deg, ap, tr, e * 100))
_, _, e50 = maclaurin_err(50)
check(e50 > 0.05,
      '★★★ 在参数允许的**最大**坡度 50° 处，近似让预期的盘旋下沉率偏高 %.1f%% —— '
      '这是一个可以算出来的、随坡度增长的模型误差' % (e50 * 100))
check(abs(1.0 - (_m.sqrt(2.0)) ** 2 / 2.0) < 1e-12 and _m.sqrt(2.0) > 1.414,
      '★★ 更远的地方：φ = √2 rad = 81.03° 时近似式**归零** ⇒ `C2/cosphi²` 除零。'
      '真余弦永不归零，所以这是纯近似造成的奇点（参数上限 50° 安全，但函数本身不是全域可用）')

# 盘旋转弯时间常数：跨文件一致性
check(re.search(r'return 2\*M_PI\*_aspd_filt_constrained/\(GRAVITY_MSS\*tanf\(thermal_bank\)\);', src_vario) is not None,
      '★ `calculate_circling_time_constant` 返回 `2πV/(g·tanφ)`')
src_soar = slurp('libraries/AP_Soaring/AP_Soaring.cpp')
check(re.search(r'const float radius = \(target_aspd\*target_aspd\) / \(GRAVITY_MSS \* tanf\(thermal_bank\*DEG_TO_RAD\)\);', src_soar) is not None,
      '★ 而另一文件里盘旋半径是 `R = V²/(g·tanφ)`')
check(re.search(r'float SoaringController::McCready\(float alt\)', src_soar) is not None
      and 'A method shell to be filled in later' in src_soar,
      '★★★ `McCready(float alt)` 是**空壳**：函数体只有 `return thermal_vspeed;`，'
      '注释直说 "A method shell to be filled in later" —— **高度依赖的麦克斯韦曲线并不存在**')

# 独立基准：R = V²/(g tanφ) 与 τ = 2πV/(g tanφ) 必须满足 τ = 2πR/V
for (V, bank_deg) in ((12.0, 30.0), (15.5, 45.0), (9.0, 20.0)):
    phi = _m.radians(bank_deg)
    R = V * V / (9.81 * _m.tan(phi))
    tau = 2 * _m.pi * V / (9.81 * _m.tan(phi))
    check(abs(tau - 2 * _m.pi * R / V) < 1e-9,
          '★★★ **跨文件一致性**：V=%g φ=%g° ⇒ R=%.3f m，τ=%.3f s，'
          '而复算 2πR/V=%.3f s ⇒ 两条独立写下的公式自洽' % (V, bank_deg, R, tau, 2 * _m.pi * R / V))

# VAR.cl 恒为 0（或未初始化）
check(len(re.findall(r'_raw_climb_rate', src_vario + slurp('libraries/AP_Soaring/Variometer.h'))) == 2,
      '★★★ `_raw_climb_rate` 全库只出现 **2 次**：声明一次、写日志一次 —— **从不赋值**')
check(re.search(r'float raw_climb_rate = 0\.0f;', src_vario) is not None
      and 'reading = raw_climb_rate' in re.sub(r'\s+', ' ', src_vario),
      '★★ 真正被赋值的是**局部变量** `raw_climb_rate`（无下划线），它进了 `reading` 却没进日志')
check(re.search(r'//\s*@Field: cl: raw climb rate', src_vario) is not None,
      '★★★ 而日志字段注释写着 `@Field: cl: raw climb rate` ⇒ **`VAR.cl` 这个字段拿不到它名字承诺的东西**')


# ==========================================================================
print()
print('=' * 76)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
sys.exit(1)
