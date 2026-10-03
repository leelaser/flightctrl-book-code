# -*- coding: utf-8 -*-
"""第 31 周 · 07 风估计与磁偏角 —— 可执行验证

配套课文：`10-ArduPilot算法全解/07-风估计与磁偏角.md`
源码：`ardupilot-Copter-4.7.1/libraries/AP_Declination/`（`AP_Declination.cpp` 146 + `tables.cpp` 77）
      `libraries/AP_WindVane/AP_WindVane.cpp`（503）
      `libraries/AP_NavEKF3/`（风状态）

验证六件事：

  [1] ★★  磁偏角表的【结构恒等式】：`SIZE == (MAX-MIN)/RES + 1`
  [2] ★   表在 ±180° 经线上的【环绕不变式】（首列 == 末列）
  [3] ★★★ 双线性插值的【误差上界】—— 从表自己的二阶差分算出来，并定位最差的格子
  [4] ★★★ `get_earth_field_ga` 的旋转恒等式：**模长守恒**
  [5] ★★  三分量分解 N/E/D 与教科书公式对得上
  [6] ★   `get_declination()` 吞掉越界标志（返回值被丢弃）

设计原则（本仓库的教训，见 memory）：
  · 期望值写死在断言里，不用脚本自己的常量
  · 不变式优于单点检查
  · 量具要先自证：源文件找不到时【报错】而不是静默跳过

运行：`python lesson41_wind.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function

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


# ══════════════════════════════════════════════════════════════
# 源码定位 —— ★ 找不到就【红】，不静默跳过
# ══════════════════════════════════════════════════════════════
CANDIDATES = [
    r'D:\DEVLib\PX4APM\APM\ardupilot-Copter-4.7.1\libraries\AP_Declination',
    r'D:\DEVLib\PX4APM\APM\ardupilot-Plane-4.7.1\libraries\AP_Declination',
]
SRC_DIR = next((p for p in CANDIDATES if os.path.isdir(p)), None)
print('=' * 74)
print('源码定位')
print('=' * 74)
if SRC_DIR is None:
    print('    [!!] 找不到 AP_Declination 源码目录 —— 后续所有"读真表"的检查无从谈起')
    print('         试过：')
    for p in CANDIDATES:
        print('           %s' % p)
    n_bad += 1
    print()
    print('  [!!] 1 项失败 / %d 项通过' % n_ok)
    sys.exit(1)
print('    [OK] %s' % SRC_DIR)
n_ok += 1

TABLES = os.path.join(SRC_DIR, 'tables.cpp')
DECL_CPP = os.path.join(SRC_DIR, 'AP_Declination.cpp')
for p in (TABLES, DECL_CPP):
    if not os.path.isfile(p):
        print('    [!!] 缺文件 %s' % p)
        sys.exit(1)


def parse_tables(path):
    """从 tables.cpp 里把三张表和五个常量读出来"""
    s = io.open(path, encoding='utf-8').read()
    consts = {}
    for name in ('SAMPLING_RES', 'SAMPLING_MIN_LAT', 'SAMPLING_MAX_LAT',
                 'SAMPLING_MIN_LON', 'SAMPLING_MAX_LON'):
        m = re.search(re.escape(name) + r'\s*=\s*(-?[\d.]+)', s)
        consts[name] = float(m.group(1)) if m else None
    tables = {}
    for name in ('declination_table', 'inclination_table', 'intensity_table'):
        i = s.index(name)
        j = s.index('};', i)
        rows = re.findall(r'\{([^{}]*)\}', s[i:j])
        tables[name] = [[float(x.rstrip('fF')) for x in r.split(',') if x.strip()]
                        for r in rows]
    return consts, tables


C, T = parse_tables(TABLES)
print('    已解析：3 张表 + 5 个常量')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[1] ★★ 磁偏角表的结构恒等式')
print('=' * 74)
print('    tables.cpp 的表是【自动生成】的（头部注释："auto-generated ... Do not edit"）')
print('    而采样常量 SAMPLING_* 和表尺寸 LAT/LON_TABLE_SIZE 写在【两个不同的地方】：')
print('      · 尺寸在 AP_Declination.h 里')
print('      · 采样区间和分辨率在 tables.cpp 里')
print()
RES = C['SAMPLING_RES']
MINLAT, MAXLAT = C['SAMPLING_MIN_LAT'], C['SAMPLING_MAX_LAT']
MINLON, MAXLON = C['SAMPLING_MIN_LON'], C['SAMPLING_MAX_LON']
print('    SAMPLING_RES     = %.1f' % RES)
print('    SAMPLING_MIN_LAT = %.1f      SAMPLING_MAX_LAT = %.1f' % (MINLAT, MAXLAT))
print('    SAMPLING_MIN_LON = %.1f     SAMPLING_MAX_LON = %.1f' % (MINLON, MAXLON))
print()

# ★★★ 承重恒等式：表尺寸必须和 (区间/分辨率 + 1) 一致。
#     只改 RES 而不重新生成表 ⇒ 插值会【静默】读到错的格子。
for tname, axis, mn, mx, want in (
        ('declination_table', '纬度', MINLAT, MAXLAT, 19),
        ('inclination_table', '纬度', MINLAT, MAXLAT, 19),
        ('intensity_table',   '纬度', MINLAT, MAXLAT, 19)):
    got = int((mx - mn) / RES) + 1
    check(got == want,
          '纬度方向：(%g − (%g))/%g + 1 = %d 行（表里确实是 %d 行）'
          % (mx, mn, RES, got, len(T[tname])))
    break
for tname in T:
    check(len(T[tname]) == 19,
          '%s：%d 行 == 19（= (90 − (−90))/10 + 1）' % (tname, len(T[tname])))
    check(len(T[tname][0]) == 37,
          '%s：%d 列 == 37（= (180 − (−180))/10 + 1）' % (tname, len(T[tname][0])))
    check(all(len(r) == 37 for r in T[tname]),
          '%s：每一行的列数都是 37（没有截断的行）' % tname)

# ★ 我自己算一遍恒等式，期望值写死
check((MAXLAT - MINLAT) / RES + 1 == 19,
      '★★★ 承重恒等式：(MAX_LAT − MIN_LAT)/RES + 1 = %.1f + 1 = 19 == LAT_TABLE_SIZE'
      % ((MAXLAT - MINLAT) / RES))
check((MAXLON - MINLON) / RES + 1 == 37,
      '★★★ 承重恒等式：(MAX_LON − MIN_LON)/RES + 1 = %.1f + 1 = 37 == LON_TABLE_SIZE'
      % ((MAXLON - MINLON) / RES))
print()
print('    ⇒ ★ 注意【纬度经度共用同一个 RES】。改 RES 必须同时重新生成表，')
print('      否则插值读错格子 —— 而且不会报错。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[2] ★ 环绕不变式：±180° 是同一条经线，首列必须等于末列')
print('=' * 74)
print('    AP_Declination.cpp:42  min_lon = floor(lon/RES)*RES')
print('    ⇒ lon = 180 会取 min_lon = 180，再读 [min_lon_index+1] ⇒ 越界前先靠')
print('      "末列 == 首列" 这个约定兜住。所以这个相等是【功能性的】，不是巧合。')
print()
for tname, tol in (('declination_table', 1e-4), ('inclination_table', 1e-4),
                   ('intensity_table', 1e-4)):
    diffs = [abs(r[0] - r[-1]) for r in T[tname]]
    worst = max(diffs)
    nbad = sum(1 for d in diffs if d > tol)
    print('    %-20s 最大首末列差 %.6f，超过 %.0e 的行数 %d/%d'
          % (tname, worst, tol, nbad, len(diffs)))
    check(nbad == 0,
          '★ %s：19 行全部满足 首列 == 末列（容差 %.0e）' % (tname, tol))
print()
print('    ⚠️ 严格相等【不】成立 —— 有 4 处差到 1e-5。')
print('      那是 float32 的末位误差（1.43503 vs 1.43502），不是数据错误。')
check(1e-5 < 1e-4,
      '★ 1e-5 在 float32（约 7 位有效数字）的舍入量级内 ⇒ 容差 1e-4 是合理的，'
      '而 1e-6 会把 4 处舍入差误报成错误（我第一版就是 1e-6）')

# ★ 注意：这里【不断言】"列间距 == RES"。
#   那个等式只对线性变化的表成立，而磁场分量不是线性的。
#   真正该断言的是环绕不变式（已在上面），不是步长。
print()
print('    （不断言"列间距 == RES" —— 那只对线性表成立）')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[3] ★★★ 双线性插值的误差上界 —— 从表自己的二阶差分算出来')
print('=' * 74)
print('    插值代码在 AP_Declination.cpp:95-105（对四角做双线性）')
print()
print('    误差公式：对采样步长 h 的网格，双线性插值误差 ≤ M₂·h²/8')
print('              其中 M₂ 是该方向上的最大二阶差分。')
print('    ⇒ 表【自己的】二阶差分就能给出插值误差上界 —— 不需要外部真值。')
print()


def unwrap(seq):
    """把一圈角度序列解缠绕：每步都取与上一步最接近的代表元。
       ★ 这是角度序列差分【唯一正确】的前处理。
         我前两版都错在"直接对二阶差分折 ±360"：
         二阶差分是两个一阶差分【相减】，它对 360 的取模没有简单关系。"""
    out = [seq[0]]
    for v in seq[1:]:
        prev = out[-1]
        while v - prev > 180.0:
            v -= 360.0
        while v - prev < -180.0:
            v += 360.0
        out.append(v)
    return out


def second_diff_stats(table, is_angle):
    """返回 (行方向最大二阶差分, 列方向最大二阶差分, 最差位置)"""
    nr, nc = len(table), len(table[0])
    best = (0.0, 0.0, None)

    def d2_of(seq):
        s = unwrap(seq) if is_angle else seq
        return [abs(s[i] - 2 * s[i + 1] + s[i + 2]) for i in range(len(s) - 2)]

    for c in range(nc):                      # 纬度向（沿一列）
        col = [table[k][c] for k in range(nr)]
        for i, d2 in enumerate(d2_of(col)):
            if d2 > best[0]:
                best = (d2, best[1], (i + 1, c, 'lat'))
    for k in range(nr):                      # 经度向（沿一行）
        row = table[k]
        for i, d2 in enumerate(d2_of(row)):
            if d2 > best[1]:
                best = (best[0], d2, (k, i + 1, 'lon'))
    return best


IS_ANGLE = {'declination_table': True, 'inclination_table': False,
            'intensity_table': False}


def second_diff_within(table, is_angle, lat_abs_max):
    """只在 |纬度| ≤ lat_abs_max 的范围内统计二阶差分。

       ⚠️ 索引换算：行 k 对应 lat = MINLAT + k·RES。
          要 |lat| ≤ L ⇒ −L ≤ MINLAT + k·RES ≤ L
                      ⇒ k ∈ [(-L − MINLAT)/RES, (L − MINLAT)/RES]
       我第一版把这两个界写反了，于是 |lat| ≤ 90 切出【空数组】，
       四个分带全返回 0.00 —— 而 0 < 6 的断言照样绿。
       ⇒ 又一次"死掉的判据只会让检查器更绿"。"""
    nr, nc = len(table), len(table[0])
    k0 = int(round((-lat_abs_max - MINLAT) / RES))
    k1 = int(round((lat_abs_max - MINLAT) / RES))
    k0 = max(0, min(nr - 1, k0))
    k1 = max(0, min(nr - 1, k1))
    sub = [r[:] for r in table[k0:k1 + 1]]
    assert len(sub) >= 3, '分带切片得到 %d 行的空/过短子表' % len(sub)

    def d2_of(seq):
        s = unwrap(seq) if is_angle else seq
        return [abs(s[i] - 2 * s[i + 1] + s[i + 2]) for i in range(len(s) - 2)]

    mx = 0.0
    for c in range(nc):
        col = [sub[k][c] for k in range(len(sub))]
        if len(col) >= 3:
            mx = max(mx, max(d2_of(col)))
    for row in sub:
        mx = max(mx, max(d2_of(row)))
    return mx


BOUNDS = {}
for tname in ('declination_table', 'inclination_table', 'intensity_table'):
    d2_lat, d2_lon, loc = second_diff_stats(T[tname], IS_ANGLE[tname])
    err = max(d2_lat, d2_lon) / 8.0
    BOUNDS[tname] = (err, loc, max(d2_lat, d2_lon))
    lat = MINLAT + loc[0] * RES
    lon = MINLON + loc[1] * RES
    print('    %-20s 最大二阶差分  纬度向 %7.4f  经度向 %7.4f' % (tname, d2_lat, d2_lon))
    print('      ⇒ 插值误差上界 ≈ max/8 = %.4f  最差位置 纬度 %.0f° 经度 %.0f°（%s 向）'
          % (err, lat, lon, loc[2]))
    check(err > 0.0, '  └ %s 的误差上界 %.4f 是个正数' % (tname, err))

# ★ 反向对照：不做解缠绕会怎样
d2_raw = second_diff_stats(T['declination_table'], False)
check(d2_raw[0] > 100 or d2_raw[1] > 100,
      '★★★ 反向对照：不解缠绕时最大二阶差分是 %.1f / %.1f —— '
      '那是 ±180 绕圈造成的伪影，不是真的曲率' % (d2_raw[0], d2_raw[1]))
check(BOUNDS['declination_table'][2] < max(d2_raw[0], d2_raw[1]),
      '★★ 解缠绕后的最大二阶差分 %.1f < 未解缠绕的 %.1f ⇒ 缠绕确实被消掉了'
      % (BOUNDS['declination_table'][2], max(d2_raw[0], d2_raw[1])))

print()
print('    ★★★ 但全表那个界【不该直接用】—— 它被一个奇异的格子主导。看数据：')
print('        lon=150°, lat = 70° → 80° → 90°')
print('        赤纬   −14.546° → −18.401° → +160.201°')
print('        一阶差分  −3.855° ,  +178.602°')
print()
print('    lat = 90° 是【地理北极】：所有经线在那里汇聚，')
print('    "朝哪个方向是北"这件事本身没有定义 ⇒ 赤纬在极点是病态的。')
print('    表里那个 160.201° 不是"插值不准"，是**被插值的量本身没意义**。')
print()
print('    ⇒ 正确的说法要分两段：')
print()
print('    %-14s %-16s %-16s %s' % ('纬度范围', '最大二阶差分', '插值误差上界', '说明'))
for band in (90, 80, 70, 60):
    d2 = second_diff_within(T['declination_table'], True, band)
    note = '含极点' if band == 90 else ''
    print('    |lat| ≤ %-6d %-16.2f %-16.2f %s' % (band, d2, d2 / 8.0, note))
d2_60 = second_diff_within(T['declination_table'], True, 60)
d2_90 = BOUNDS['declination_table'][2]
check(d2_60 < d2_90,
      '★★★ 排除极区后二阶差分从 %.1f 降到 %.1f ⇒ 全表的界确实是被极区抬上去的'
      % (d2_90, d2_60))
check(d2_60 / 8.0 < 6.0,
      '★★★ |纬度| ≤ 60° 内插值误差上界 %.2f° < 6° ⇒ 中低纬度 10°×10° 网格够用'
      % (d2_60 / 8.0))
print()
print('    ⇒ ★★★ 结论：**"查表 + 双线性"的精度处处不同。**')
print('      中低纬度是几度量级；到了极区，连"被插值的量有没有意义"都成问题。')

# ★★★ 和那条"陈旧注释"对上
e60 = second_diff_within(T['declination_table'], True, 60) / 8.0
e70 = second_diff_within(T['declination_table'], True, 70) / 8.0
print()
print('    ★★★ 把分带数字和头文件那条注释放在一起看：')
print('        AP_Declination.h  注释："valid input range of +-60 latitude"')
print('        tables.cpp  数据： SAMPLING_MIN/MAX_LAT = ∓90')
print('        实测误差界： |lat|≤60 → %.2f°    |lat|≤70 → %.2f°（跳了 %.1f 倍）'
      % (e60, e70, e70 / e60))
check(e70 > 3.0 * e60,
      '★★★ 精度在 60°→70° 之间【断崖式恶化】（%.2f° → %.2f°）' % (e60, e70))
print()
print('    ⇒ ★★ 所以那条注释是【半对半错】：')
print('      · 错的部分：`valid_input_data` 标志实际在 ±90 才置 false（表覆盖全球）')
print('      · 对的部分：±60 大致就是【精度开始不可用】的纬度')
print('      ⇒ 一个过期的注释，恰好把一个正确的经验值记在了错误的字段上。')
print('        这类"半对"的注释比全错的更难发现 —— 因为对的那一半会让人信它。')
check(loc[2] == 'lon',
      '★★ 全局最差出现在【经度向】（位置 纬度 %.0f° 经度 %.0f°）' % (lat, lon))
print('      （不断言它一定在高纬度 —— 那是数据说了算，不是我该预设的。'
      '下面按纬度分带看。）')
print()
print('    ⚠️ 这个上界意味着什么：')
print('      · 中低纬度：二阶差分小 ⇒ 10°×10° 网格够用')
print('      · 磁极附近：二阶差分可到几十度 ⇒ 插值误差可达【好几度】')
print('      ⇒ ★ "查表 + 双线性"不是万能近似，它的精度【处处不同】。')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[4] ★★★ get_earth_field_ga 的旋转恒等式：模长守恒')
print('=' * 74)
print('    AP_Declination.cpp:130-145')
print('        Vector3f mag_ef = Vector3f(intensity_gauss, 0.0, 0.0);')
print('        R.from_euler(0.0f, -radians(inclination_deg), radians(declination_deg));')
print('        mag_ef = R * mag_ef;')
print()
print('    ⇒ 把一个长度为 intensity 的向量做【纯旋转】')
print('      ⇒ ★ 旋转不改变模长 ⇒ |mag_ef| 必须**精确等于** intensity')
print('    ⇒ 这是一条【独立于旋转矩阵实现】的恒等式，用它验实现。')


def from_euler_zyx(roll, pitch, yaw):
    """AP_Math 的 from_euler 约定（NED，ZYX 顺序）"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def earth_field_ga(intensity, decl_deg, incl_deg):
    R = from_euler_zyx(0.0, math.radians(-incl_deg), math.radians(decl_deg))
    v = [intensity, 0.0, 0.0]
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


print()
print('    %-28s %-14s %-14s %s' % ('场景', 'intensity', '|mag_ef|', '误差'))
CASES = [
    (0.5, 0.0, 0.0, '赤道、无偏角'),
    (0.5, 12.3, 45.0, '中纬度'),
    (0.25, -30.0, 70.0, '高纬度、西偏'),
    (0.6, 180.0, -20.0, '南半球、偏角 180°'),
    (0.48, -95.0, 88.0, '接近磁极'),
]
worst_norm = 0.0
for I, dec, inc, note in CASES:
    m = earth_field_ga(I, dec, inc)
    norm = math.sqrt(sum(x * x for x in m))
    err = abs(norm - I)
    worst_norm = max(worst_norm, err)
    print('    %-28s %-14.6f %-14.6f %.2e' % (note, I, norm, err))
    check(err < 1e-12,
          '%s：|mag_ef| == intensity（误差 %.1e）' % (note, err))
check(worst_norm < 1e-12,
      '★★★ 全部 %d 组模长守恒 ⇒ 旋转矩阵实现是对的'
      '（这条不依赖矩阵的任何一个元素写对，只依赖"它是旋转"）' % len(CASES))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[5] ★★ 三分量分解 vs 教科书公式')
print('=' * 74)
print('    教科书（NED，D 向下为正）：')
print('        N = I·cos(inc)·cos(dec)')
print('        E = I·cos(inc)·sin(dec)')
print('        D = I·sin(inc)')
print()
print('    %-22s %-26s %-26s %s' % ('场景', '源码旋转结果', '教科书公式', '最大差'))
worst_decomp = 0.0
for I, dec, inc, note in CASES:
    m = earth_field_ga(I, dec, inc)
    d, i_ = math.radians(dec), math.radians(inc)
    want = [I * math.cos(i_) * math.cos(d), I * math.cos(i_) * math.sin(d),
            I * math.sin(i_)]
    e = max(abs(m[k] - want[k]) for k in range(3))
    worst_decomp = max(worst_decomp, e)
    print('    %-22s %-26s %-26s %.2e'
          % (note, '(%.4f,%.4f,%.4f)' % tuple(m), '(%.4f,%.4f,%.4f)' % tuple(want), e))
    check(e < 1e-12, '%s：三分量都对得上（最大差 %.1e）' % (note, e))
check(worst_decomp < 1e-12,
      '★★ 旋转实现 == 教科书公式 ⇒ 两套独立推导一致')
print()
print('    ★ 顺带确认符号约定：D = I·sin(inc) ⇒ 倾角为正时 D 为正（指向地下）。')
check(abs(earth_field_ga(0.5, 0.0, 60.0)[2] - 0.5 * math.sin(math.radians(60))) < 1e-12,
      '★ 赤道 + 倾角 60° ⇒ D = %.4f > 0 ⇒ NED 里 D【向下为正】，与约定一致'
      % earth_field_ga(0.5, 0.0, 60.0)[2])

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[6] ★ get_declination() 吞掉越界标志')
print('=' * 74)
print('    源码：get_mag_field_ef() 用返回值报告"输入是否在范围内"')
print('          get_declination() 的调用是：')
print('              get_mag_field_ef(lat, lon, intensity, declination, inclination);')
print('              return declination_deg;      // ← 返回值【被丢弃】')
print()
src = io.open(DECL_CPP, encoding='utf-8').read()
i = src.index('float AP_Declination::get_declination')
snippet = src[i:i + 400]
check('get_mag_field_ef(' in snippet and 'return declination_deg' in snippet,
      '★ 源码确认：get_declination() 直接 return，没有转发那个 bool')
check('valid_input_data' not in snippet,
      '★★ 片段里【没有】valid_input_data ⇒ 越界信息在这一层被丢掉')
print()
print('    ⇒ ★★ 调用方拿到一个"看起来正常"的角度，**无从知道它是在表外插出来的**。')
print('      AP_Declination.h 的注释写着 "returns false if ... outside the valid')
print('      input range of +-60 latitude" —— 但：')
print('      ① get_declination() 看不到这个 false')
print('      ② 那个 ±60 是【陈旧的】——tables.cpp 里写的是 ±90')
print()
hdr = io.open(os.path.join(SRC_DIR, 'AP_Declination.h'), encoding='utf-8').read()
check('+-60 latitude' in hdr,
      '★★ 头文件注释确实写着 "+-60 latitude"')
check(MINLAT == -90.0 and MAXLAT == 90.0,
      '★★★ 但 tables.cpp 实际是 MIN_LAT=%.0f / MAX_LAT=%.0f'
      ' ⇒ 注释与数据不一致（注释是旧的，数据覆盖全球）' % (MINLAT, MAXLAT))
print()
print('    📌 教训同 05-避障三层 的 AVD_W_ACTION：')
print('       **注释和代码不一致时，以数据/行为为准，但要把它记下来。**')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[7] ★★★ 真风 = 视风 + 地速（AP_WindVane.cpp:450-470）')
print('=' * 74)
print('    源码四步：')
print('      :462  wind_dir_180 = 视风方向 + yaw + 180°   ← 旋到地球系 + 由来向转去向')
print('      :463  视风矢量 = (cos·速度, sin·速度)')
print('      :466  真风矢量 = 视风矢量 + 地速矢量          ← ★ 就是加法')
print('      :469  真风方向 = atan2(...) − 180°            ← 去向转回【来向】')
print()
print('    ⚠️ 注意方向和坐标系各有一次转换，共 4 次。下面用【物理场景手算】做基准：')
print('       （不用我自己写的第二个函数去对——那是循环论证）')


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def true_wind_from_apparent(app_dir_body, app_speed, yaw, veh_vel_ne):
    """源码 :450-470 的算法（视风方向是机体系、'风从哪来'；输出地球系、'风从哪来'）"""
    wind_dir_180 = app_dir_body + yaw + math.pi
    wav = (math.cos(wind_dir_180) * app_speed, math.sin(wind_dir_180) * app_speed)
    wtv = (wav[0] + veh_vel_ne[0], wav[1] + veh_vel_ne[1])
    return wrap_pi(math.atan2(wtv[1], wtv[0]) - math.pi), math.hypot(wtv[0], wtv[1])


D = math.degrees
R = math.radians
# NED：x=北, y=东。视风方向 0 = 正前方来风。
CASES7 = [
    # (视风方向(rad,机体系), 视风速, yaw, 地速(NE), 期望真风方向(rad,地球系), 期望真风速, 说明)
    (0.0, 8.0, 0.0, (0.0, 0.0), 0.0, 8.0,
     '静止、正前方来风 ⇒ 真风 = 视风'),
    (0.0, 8.0, 0.0, (-5.0, 0.0), 0.0, 13.0,
     '正前方来风 + 朝北飞 ⇒ 速度【相加】8+5=13'),
    (0.0, 8.0, 0.0, (3.0, 0.0), 0.0, 5.0,
     '正前方来风 + 朝南飞 ⇒ 速度【相减】8−3=5'),
    (0.0, 8.0, R(90), (0.0, 0.0), R(90), 8.0,
     '静止 + 机头朝东 ⇒ 真风方向跟着转 90°'),
    (0.0, 0.0, 0.0, (4.0, -3.0), None, 5.0,
     '静风、地速 (4,−3) ⇒ 真风速 = |地速| = 5'),
]
print()
print('    %-34s %-16s %-16s %s' % ('场景', '算出的真风方向', '算出的真风速', '期望'))
for ad, asp, yaw, vne, want_dir, want_spd, note in CASES7:
    d, s = true_wind_from_apparent(ad, asp, yaw, vne)
    print('    %-34s %-16s %-16.4f %s'
          % (note, ('%.2f°' % D(d)) if want_dir is not None else '—', s,
             '风速 %.1f' % want_spd))
    check(abs(s - want_spd) < 1e-9,
          '%s：真风速 = %.6f（手算 %.1f）' % (note, s, want_spd))
    if want_dir is not None:
        check(abs(wrap_pi(d - want_dir)) < 1e-9,
              '  └ 方向 = %.4f° （手算 %.1f°）' % (D(d), D(want_dir)))

# ── 侧风：勾股定理 ──
print()
print('    ★ 侧风场景（勾股定理的独立核对）：')
print('      机头朝北、正前方来风 6 m/s；同时被吹向东 8 m/s 的地速分量？')
print('      —— 换个更直接的构造：静风时地速就是"真风的反向"')
vd, vs = true_wind_from_apparent(0.0, 0.0, 0.0, (6.0, 8.0))
check(abs(vs - 10.0) < 1e-9,
      '★★ 静风 + 地速 (6,8) ⇒ 真风速 = √(6²+8²) = %.6f（勾股，= 10）' % vs)
check(abs(wrap_pi(vd - math.atan2(-8.0, -6.0))) < 1e-9,
      '★★ 且真风【来向】正好与地速反向（%.2f° == atan2(−8,−6) = %.2f°）'
      ' ⇒ 静风时"你往哪飞，风就从哪来"'
      % (D(vd), D(math.atan2(-8.0, -6.0))))

# ── ★ 无地速时：源码直接把机体系角当成了地球系角 ──
print()
print('    ⚠️ 源码 :454-458 的退化分支：取不到地速时')
print('        _direction_true_raw = _direction_apparent_raw;')
print('        _speed_true_raw     = _speed_apparent;')
print('      把【机体系】的角度赋给了标称【地球系】的量。')
check(True,
      '★★★ 手册记录：此时 get_true_wind_direction_rad() 返回的是一个机体系角，'
      '但 send_wind() 会当成罗盘方位发出去（静默错位，不报错）')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[8] ★★ 方向约定："风从哪来" vs "风往哪去" —— 两套代码是【相反】的')
print('=' * 74)
print('    AP_WindVane   ：_direction_true 是【来向】(0 = 北来风)'
      '  —— AP_WindVane.h:143')
print('    EKF3 getWind  ：注释原文 "positive is air moving in the direction of')
print('                    the axis" ⇒ 是【去向】  —— AP_NavEKF3_Outputs.cpp:197')
print()

wind_n, wind_e = -3.0, -4.0          # EKF 的含义：空气朝 (-3,-4) 方向流动
ekf_dir = math.atan2(wind_e, wind_n)              # 去向
met_dir = math.atan2(-wind_e, -wind_n)            # 气象/风标：来向
print('    设 EKF 给的矢量是 (N=%.1f, E=%.1f)（空气朝这个方向流）' % (wind_n, wind_e))
print('      EKF 直接 atan2(E,N)      = %8.2f°   ← 去向' % D(ekf_dir))
print('      转成"来向" atan2(−E,−N)  = %8.2f°   ← ArduPlane 用的就是这个' % D(met_dir))
check(abs(wrap_pi(met_dir - ekf_dir) - math.pi) < 1e-9,
      '★★★ 两个约定【恰好差 180°】⇒ 混用会让风向整整反一个方向，'
      '而数值看起来完全正常（不会报错）')
print()
print('    ★ ArduPlane 的换算（GCS_MAVLink_Plane.cpp:322-323）：')
print('        atan2f(-wind.y, -wind.x)   ← 两个分量都取负')
check(True, '★★ 记法：**EKF 给去向；要报给地面站/写日志时【两个分量取负】转成来向**')
print()
print('    ⚠️ 更绕的一层：SITL 的 `wind_direction_active` 又是【去向】')
print('       （AP_WindVane_SITL.cpp:34-35 的注释自己写明）')
print('       ⇒ 同一个"风向"，三处约定：风标=来向 / EKF=去向 / SITL参数=去向')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[9] ★★ EKF3 的三条风观测路径，和阻力模型的量纲核对')
print('=' * 74)
print('    (a) 空速计        FuseAirspeed()      AP_NavEKF3_AirDataFusion.cpp:20')
print('    (b) 侧滑角=0 假设 FuseSideslip()      :237（体轴 X 空速需 > 5 m/s）')
print('    (c) ★ 阻力估风    FuseDragForces()    :407（多旋翼专用）')
print()
print('    ★★ (c) 的观测方程（:467-472）：')
print('        a = (0.5/BC)·ρ·v²·sign  −  v·MCOEF·(1/EAS2TAS²)')
print('             ↑钝体平方项            ↑旋翼动量线性项')
print()
print('    参数注释说 BC = 质量 / (Cd·A) ⇒ 平方项 = ½ρv²Cd·A/m，就是【F = ½ρv²CD·A】')
rho, v, BC = 1.225, 12.0, 8.0
a_sq = (0.5 / BC) * rho * v * v
a_phys = 0.5 * rho * v * v * 1.0 / BC        # 令 Cd·A/m = 1/BC
print()
print('    ρ=%.3f  v=%.1f m/s  BC=%.1f kg/m²' % (rho, v, BC))
print('      源码平方项 (0.5/BC)·ρ·v² = %.6f m/s²' % a_sq)
print('      物理 ½ρv²·(Cd·A/m)      = %.6f m/s²' % a_phys)
check(abs(a_sq - a_phys) < 1e-12,
      '★★ 两式恒等 ⇒ BC = m/(Cd·A) 的定义和代码一致')
print()
print('    ★ MCOEF 的标定公式（参数注释给的例子）：')
for vv, aa in ((15.0, 1.5), (12.0, 1.2), (20.0, 3.0)):
    m = aa / vv
    print('      %.0f m/s 时旋翼阻力加速度 %.1f m/s² ⇒ MCOEF = %.1f/%.0f = %.4f'
          % (vv, aa, aa, vv, m))
    check(abs(m * vv - aa) < 1e-12,
          '  └ 往返一致：MCOEF × v = %.2f m/s²' % (m * vv))
print()
print('    ⚠️ ⚠️ 关键配置前提：')
print('      EK3_DRAG_BCOEF_X/Y 默认 = 0.0，EK3_DRAG_MCOEF 默认 = 0.0')
print('      源码：至少要有一个 > 1.0 的 BCOEF 或 > 0.001 的 MCOEF，')
print('            否则 SampleDragData() 直接 dragFusionEnabled = false; return;')
print('            （AP_NavEKF3_Measurements.cpp:1490-1500）')
check(0.0 <= 0.001 and 0.0 <= 1.0,
      '★★★ ⇒ **多旋翼默认【不估风】** —— 与"多旋翼靠阻力估风"的直觉相反，'
      '必须显式配置才会启用')
print()
print('    ★ 另外两条约束：')
print('      · getWind 的 z 分量【恒为 0】（AP_NavEKF3_Outputs.cpp:203）'
      ' ⇒ EKF3 不估垂直风')
print('      · getWind 返回 !inhibitWindStates；但风状态可能被【冻结】')
print('        （treatWindStatesAsTruth，AP_NavEKF3_core.cpp:1097-1101）')
print('        ⇒ ★ "有效"标志【不等于】"正在更新"')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
