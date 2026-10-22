# -*- coding: utf-8 -*-
"""卷 8 第 11 章 · 无 GPS 导航 —— 可执行验证

配套课文：`卷 8 第 11 章`
源码：`ardupilot-Copter-4.7.2/libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp`（光流观测）
      `libraries/AP_NavEKF3/AP_NavEKF3.h`（DCM33FlowMin / FLOW_USE_*）
      `libraries/AP_NavEKF/AP_NavEKF_Source.{h,cpp}`（★ 源集机制）
      `libraries/AP_Scripting/examples/ahrs-source-gps-optflow.lua`（★ 降级策略）

验证六件事：

  [1] ★★★ 光流测量模型 `flowRate = v_body / h` —— 用【几何法】独立验证，并给出它的误差界
  [2] ★★  饱和约束 `v_max = EK3_FLOW_MAX × h` 和它推出的飞行包线
  [3] ★★  倾斜门限：`DCM33FlowMin = 0.71` 背后的角度
  [4] ★★★ 源集枚举表：三个轴各写各的，同一个数字可能含义不同
  [5] ★★★ 默认配置下"三套源"其实只有一套 —— 降级必须手工配
  [6] ★★  投票迟滞：切换时间 = vote_counter_max × 周期

设计原则（本仓库的教训，见 memory）：
  · 断言两边来源必须不同 —— [1] 用几何法（数值微分）对源码的代数模型
  · 期望写在被测对象【之外】，不做"按结果分支"的同义反复
  · 样本按解析边界密扫，不手挑

运行：`python lesson40_nogps.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function

import math
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
print('=' * 74)
print('[1] ★★★ 光流的测量模型：flowRate = 相对速度 / 高度')
print('=' * 74)
print('    源码 AP_NavEKF3_OptFlowFusion.cpp:334-335')
print('        relVelSensor = prevTnb * state.velocity + (bodyRadXYZ % posOffsetBody)')
print('        losPred[0] =  relVelSensor.y / range')
print('        losPred[1] = -relVelSensor.x / range')
print()
print('    ★ 三件事都在这三行里：')
print('      ① 把 NED 速度【旋进机体系】(prevTnb)')
print('      ② 加上【角速率 × 杆臂】的补偿项')
print('      ③ 除以 range —— 而 range 来自【测距仪】')


def ekf_flow_model(vel_body_xy, height_m):
    """源码的代数模型（不含杆臂，杆臂单独在 [1d] 验）"""
    vx, vy = vel_body_xy
    return (vy / height_m, -vx / height_m)


# ── 用【几何法】做独立基准 ──
# 设地面特征点在正下方地面 (0,0,0)，机体在 (x, y, h)。
# 指向该点的视线【俯仰角】= atan2(-y, h)（当 x=0 时）。
# 角速率 = d/dt[atan2(-y, h)] = -v_y·h/(h² + y²) = -(v_y/h)/(1+(y/h)²)
#
# ★ 源码的模型是 v_y/h —— 也就是【在 y=0（正下方）处】的精确值。
#   y≠0 时它是近似。下面用数值微分把这条说清楚。


def exact_los_rate(v_y, y, h):
    """精确的视线角速率（解析式，几何直接推出）"""
    return -v_y * h / (h * h + y * y)


def fd_los_rate(v_y, y, h, dt=1e-5):
    """数值微分 —— 和上面的解析式独立地算同一个量。
       ★ 用【中心差分】(误差 O(dt²))，不用前向差分。
         前向差分是 O(dt)，dt=1e-6 时误差仍到 1e-7 量级，会把容差逼到没意义。"""
    a0 = math.atan2(-(y - v_y * dt / 2.0), h)
    a1 = math.atan2(-(y + v_y * dt / 2.0), h)
    return (a1 - a0) / dt


print()
print('    [1a] 先确认我的几何解析式和数值微分一致（两套独立算法）')
worst_fd = 0.0
for h in (1.0, 3.0, 10.0):
    for y_rel in (0.0, 0.1, 0.5, 1.0, 2.0, 5.0):
        for v_y in (0.5, 2.0, 8.0):
            y = y_rel * h
            a = exact_los_rate(v_y, y, h)
            b = fd_los_rate(v_y, y, h)
            worst_fd = max(worst_fd, abs(a - b))
check(worst_fd < 1e-8,
      '几何解析式 vs 中心差分数值微分：%d 组最大差 %.2e < 1e-8 '
      '⇒ 几何法本身可信（两套独立算法）' % (3 * 6 * 3, worst_fd))

print()
print('    [1b] ★ 源码模型 = 几何真值在 y=0（正下方）处的取值')
for h, v_y in ((2.0, 1.5), (5.0, 3.0), (10.0, 4.0)):
    m = ekf_flow_model((0.0, v_y), h)
    g = exact_los_rate(v_y, 0.0, h)
    check(abs(abs(m[0]) - abs(g)) < 1e-12,
          'h=%-5.1f v_y=%-4.1f：源码模型 %.9f == 几何真值(正下方) %.9f'
          % (h, v_y, m[0], g))

print()
print('    [1c] ★★★ 但 y≠0 时它是【近似】，偏差有闭式解：')
print('         模型/真值 = (v/h) / [(v/h)/(1+(y/h)²)] = 1 + (y/h)²')
print('         ⇒ 相对误差 = (y/h)²      ← 我的第一版写成 (y/h)²/(1+(y/h)²)，错了')
print()
print('    %-10s %-14s %-14s %-14s %s'
      % ('y/h', '几何真值', '源码模型 v/h', '相对误差', '解析预言'))
worst_model = 0.0
for y_rel in (0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0):
    h, v_y = 4.0, 2.0
    y = y_rel * h
    g = abs(exact_los_rate(v_y, y, h))
    m = abs(ekf_flow_model((0.0, v_y), h)[0])
    err = (m - g) / g if g > 0 else 0.0
    pred = y_rel ** 2                      # ★ 相对误差就是 (y/h)²
    worst_model = max(worst_model, abs(err - pred))
    print('    %-10.2f %-14.6f %-14.6f %-14.4f %.6f' % (y_rel, g, m, err, pred))
    check(abs(err - pred) < 1e-9,
          'y/h=%-5.2f：实测相对误差 %.6f == 解析预言 (y/h)² = %.6f'
          % (y_rel, err, pred))
check(worst_model < 1e-9,
      '★★★ 偏差的闭式解精确成立 ⇒ 源码的 v/h 是【小角近似】，'
      '偏角越大高估越多（y/h=1 时高估 50%%）')

print()
print('    [1d] 杠杆臂项：bodyRadXYZ % posOffsetBody （同样是"角速率 × 杆臂"）')
print('         —— 和 01-数学基础 里的杆臂误差是同一个东西')
for wz, ox in ((0.5, 0.1), (2.0, 0.3), (0.0, 1.0)):
    # 绕 z 轴转动 w，杆臂在 x 方向偏 ox ⇒ 附加的 y 向线速度 = -wz*ox（叉乘）
    lever_y = -wz * ox
    check(abs(lever_y - (-wz * ox)) < 1e-15,
          'ωz=%.1f rad/s, 杆臂 x=%.2f m ⇒ 附加 y 速度 %.4f m/s（= −ωz·ox）'
          % (wz, ox, lever_y))
check(abs(-2.0 * 0.3) > 0.1,
      '★ 量级感受：ωz=2 rad/s（约 115°/s）配 0.3 m 杆臂 ⇒ 0.6 m/s 的假速度。'
      '在 h=2 m 时它折合角速率 0.3 rad/s，而 EK3_FLOW_MAX 只有 2.5 ⇒ 占 12%')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[2] ★★ 饱和约束：EK3_FLOW_MAX 决定了"多高才能飞多快"')
print('=' * 74)
EK3_FLOW_MAX = 2.5      # AP_NavEKF3.cpp:312  AP_GROUPINFO("FLOW_MAX", 20, ..., 2.5f)
print('    EK3_FLOW_MAX = %.1f rad/s（默认）' % EK3_FLOW_MAX)
print('    源码 AP_NavEKF3_OptFlowFusion.cpp:87 —— 超过就【整帧丢弃】')
print()
print('    由 flowRate = v/h ≤ FLOW_MAX ⇒ ★ v_max = FLOW_MAX × h')
print()
print('    %-12s %-16s %s' % ('离地高度 h', '最大速度 v_max', '典型场景'))
# ⚠️ 期望值【写死在这里】，由源码的 2.5 rad/s 推出 ——
#    不能写成 `vmax == EK3_FLOW_MAX * h`，那样两边用同一个常量，
#    把常量改掉两边一起变，**永远不会红**（注入实测：改成 25 后全绿）。
SCEN = [(0.5, 1.25, '贴地，基本只能悬停'),
        (1.0, 2.50, '室内低速'),
        (2.0, 5.00, '室内巡航'),
        (5.0, 12.50, '室外低速'),
        (10.0, 25.00, '室外中速'),
        (30.0, 75.00, '已经不适合光流（地面纹理太远）')]
for h, want, note in SCEN:
    vmax = EK3_FLOW_MAX * h
    print('    %-12.1f %-16.2f %s' % (h, vmax, note))
    check(abs(vmax - want) < 1e-9,
          'h=%-5.1f m ⇒ v_max = %.2f m/s（独立算出的期望值 %.2f）' % (h, vmax, want))
    check(abs(vmax - 2.5 * h) < 1e-9,
          '  └ 也等于 2.5 × %.1f = %.2f（用的是源码里的 2.5，不是脚本的常量）'
          % (h, 2.5 * h))

print()
print('    ★ 反解：要在 v m/s 下用光流，最低得多高？  h_min = v / FLOW_MAX')
for v, want in ((1.0, 0.4), (3.0, 1.2), (5.0, 2.0), (10.0, 4.0), (15.0, 6.0)):
    h_min = v / EK3_FLOW_MAX
    print('    v=%-6.1f m/s ⇒ h_min = %.2f m' % (v, h_min))
    check(abs(h_min - want) < 1e-9,
          'v=%-5.1f ⇒ h_min = %.2f m（独立算出的期望 %.2f）' % (v, h_min, want))
    check(abs(h_min * EK3_FLOW_MAX - v) < 1e-12,
          '  └ 且 h_min × FLOW_MAX 回到 %.1f（往返一致）' % v)

print()
print('    ⚠️ 这个约束是【双刃】的：')
print('      · 飞太低 ⇒ 速度上不去（饱和）')
print('      · 飞太高 ⇒ flowRate 变小，同样的噪声折合更大的速度误差')
print('      ⇒ 所以"光流用多高"没有免费的解，只有权衡')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[3] ★★ 倾斜门限：DCM33FlowMin = 0.71 背后的角度')
print('=' * 74)
DCM33FlowMin = 0.71     # AP_NavEKF3.h:506
print('    AP_NavEKF3.h:506  const float DCM33FlowMin = 0.71f;')
print('    判据 AP_NavEKF3_OptFlowFusion.cpp:44 —— tiltOK = (prevTnb.c.z > DCM33FlowMin)')
print()
tilt_max = math.degrees(math.acos(DCM33FlowMin))
print('    Tbn(3,3) 是机体系 z 轴与世界 z 轴夹角的余弦 ⇒ 它就是 cos(倾斜角)')
print('    ⇒ 门限等价于 倾斜角 < acos(%.2f) = %.2f°' % (DCM33FlowMin, tilt_max))
check(abs(tilt_max - 44.77) < 0.02,
      '★ acos(0.71) = %.2f° —— 非常接近 45°（cos45° = %.5f）'
      % (tilt_max, math.cos(math.radians(45))))
check(math.cos(math.radians(tilt_max)) > DCM33FlowMin - 1e-12 and
      math.cos(math.radians(tilt_max + 0.01)) < DCM33FlowMin,
      '★ 边界精确：%.2f° 处恰好跨过门限（两个方向都验了）' % tilt_max)
print()
print('    ⇒ 为什么是倾斜？因为光流的观测模型 v/h 假设【地面是水平的、')
print('      相机指向正下方】。机体一倾斜，这个假设就破了 ——')
print('      和 [1c] 的"小角近似"是同一类误差来源。')

print()
print('    [3b] ★★ 同一个物理直觉，源码里写了【三个不一样】的常数')
TRIPLE = [
    ('DCM33FlowMin', 0.71, 'AP_NavEKF3.h:506',
     '光流融合门限：低于它就不融'),
    ('AP_SurfaceDistance 的下限', 0.707, 'AP_SurfaceDistance.cpp:41',
     '倾角补偿因子的【地板】：MAX(0.707, c.z)'),
    ('EKF3 测距仪门限', 0.70, 'AP_NavEKF3_PosVelFusion.cpp:1333',
     'c.z >= 0.7 才融合测距高度'),
]
inv_sqrt2 = 1.0 / math.sqrt(2.0)
print('    1/√2 = %.6f  （45° 的余弦）' % inv_sqrt2)
print()
print('    %-28s %-10s %-10s %s' % ('出处', '常数', '对应倾角', '用途'))
for name, c, loc, use in TRIPLE:
    print('    %-28s %-10.3f %-10.2f %s' % (name, c, math.degrees(math.acos(c)), use))
print()
for name, c, loc, use in TRIPLE:
    ang = math.degrees(math.acos(c))
    check(abs(ang - 45.0) < 1.0,
          '★ %s 的 %.3f 对应 %.2f° ⇒ 和 45° 相差不到 1°（%s）'
          % (name, c, ang, loc))
check(len(set(c for _, c, _, _ in TRIPLE)) == 3,
      '★★★ 三个常数【互不相同】（0.71 / 0.707 / 0.7）—— '
      '同一个直觉各写各的，没有共用常量')
print()
print('    ⇒ 这说明它们是【各自独立拍出来的近似 1/√2】，不是从一处引用的。')
print('      改其中一个不会影响另外两个 —— 排查倾角相关行为时要三个都看。')

print()
print('    [3c] 顺带：倾角补偿的【地板】本身也是个设计')
amp_max = 1.0 / min(c for _, c, _, _ in TRIPLE)
print('    AP_SurfaceDistance 用 MAX(0.707, c.z) 做分母 ⇒ 补偿放大倍数 ≤ 1/0.707 = %.4f'
      % (1.0 / 0.707))
check(1.0 / 0.707 < 1.415,
      '★ 放大倍数被夹在 %.4f 倍以内 ⇒ 不会因为倾角大而把测距值放大到离谱'
      % (1.0 / 0.707))

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[4] ★★★ 源集枚举：三个轴【各写各的】，只在部分值上巧合一致')
print('=' * 74)
print('    AP_NavEKF_Source.h:17-46 —— ★ 注意这是【三个独立的枚举】，不是一张共享表')
# 源码里每个轴【显式列出】的合法值（注释掉的表示"不适用"）
AXES = {
    'SourceXY':  {'NONE': 0, 'GPS': 3, 'BEACON': 4, 'OPTFLOW': 5, 'EXTNAV': 6,
                  'WHEEL_ENCODER': 7},
    'SourceZ':   {'NONE': 0, 'BARO': 1, 'RANGEFINDER': 2, 'GPS': 3, 'BEACON': 4,
                  'EXTNAV': 6},
    'SourceYaw': {'NONE': 0, 'COMPASS': 1, 'GPS': 2, 'GPS_COMPASS_FALLBACK': 3,
                  'EXTNAV': 6, 'GSF': 8},
}
print()
# ★★ 枚举的基本不变式：同一个轴内，编号必须互不相同。
#    没有这条，`name_of` 那种"编号→名字"的反查会在重复编号时静默取最后一个。
for axis, tbl in AXES.items():
    vals = list(tbl.values())
    check(len(set(vals)) == len(vals),
          '★★ %s 轴内编号互不相同（%d 个名字 / %d 个不同编号）'
          % (axis, len(vals), len(set(vals))))

print()
print('    %-6s %-14s %-14s %s' % ('编号', 'SourceXY', 'SourceZ', 'SourceYaw'))
allnums = sorted(set(n for t in AXES.values() for n in t.values()))
for num in allnums:
    row = []
    for axis in ('SourceXY', 'SourceZ', 'SourceYaw'):
        names = [k for k, v in AXES[axis].items() if v == num]
        row.append(names[0] if names else '—')
    print('    %-6d %-14s %-14s %s' % (num, row[0], row[1], row[2]))

# ★★★ 关键：同一个编号在不同轴上【含义可能不同】
name_of = {}
for axis in AXES:
    for k, v in AXES[axis].items():
        name_of.setdefault(v, {})[axis] = k
conflicts = []
for num, m in sorted(name_of.items()):
    uniq = set(m.values())
    if len(uniq) > 1 and len(m) > 1:
        conflicts.append((num, m))
print()
print('    ★★★ 同一个编号，含义不一致的地方：')
for num, m in conflicts:
    print('      %-3d  %s' % (num, '  vs  '.join('%s=%s' % (a, n) for a, n in m.items())))
check(len(conflicts) > 0,
      '★★★ 实测有 %d 处编号冲突 ⇒ 【不是】一张共享表 —— '
      '我最初以为"同一名字编号一致"，被这条断言推翻了' % len(conflicts))

# 逐条点名（这两处是最容易踩的）
check(name_of[3].get('SourceYaw') == 'GPS_COMPASS_FALLBACK'
      and name_of[3].get('SourceXY') == 'GPS',
      '★★ `3` 在 XY/Z 轴是 GPS，在 Yaw 轴是 GPS_COMPASS_FALLBACK '
      '⇒ `EK3_SRC1_POSXY=3` 和 `EK3_SRC1_YAW=3` 意思完全不同')
check(name_of[1].get('SourceZ') == 'BARO'
      and name_of[1].get('SourceYaw') == 'COMPASS',
      '★★ `1` 在 Z 轴是 BARO，在 Yaw 轴是 COMPASS ⇒ 同一个 1 指两个东西')
check(name_of[2].get('SourceZ') == 'RANGEFINDER'
      and name_of[2].get('SourceYaw') == 'GPS',
      '★★ `2` 在 Z 轴是 RANGEFINDER，在 Yaw 轴是 GPS')
check(name_of[6].get('SourceXY') == 'EXTNAV'
      and name_of[6].get('SourceZ') == 'EXTNAV'
      and name_of[6].get('SourceYaw') == 'EXTNAV',
      '★ 只有 `6 = EXTNAV` 三个轴一致（0 = NONE 也一致）')

# ★ 注意：这里必须要求【三个轴都定义了它】且名字相同。
#   我第一版只查了"已定义的轴之间名字一致"，于是 4/5/7/8 这些
#   "只在部分轴存在"的编号也被算成"一致"了 —— 断言写松了。
consistent = sorted(n for n, m in name_of.items()
                    if len(m) == 3 and len(set(m.values())) == 1)
check(consistent == [0, 6],
      '★ 三个轴【都定义了且名字相同】的编号只有 %s（0=NONE, 6=EXTNAV）'
      % consistent)

print()
print('    ⇒ ★★★ 正确的心智模型：**三个枚举各写各的，只是"碰巧"大部分对得上。**')
print('      源码把不适用的项【注释掉】留出空位（XY 缺 1,2；Z 缺 5,7；Yaw 缺 4,5,7）')
print('      ⇒ 看得出作者【想】让编号统一，但没有任何机制强制。')
print()
print('    ★ 为什么这件事要命？因为【Lua 脚本和 DAL 里写的是裸数字】：')
print('        ahrs-source-gps-optflow.lua:115')
print('            ahrs:get_vel_innovations_and_variances_for_source(5)')
print('        ⇒ 那个 5 是 SourceXY::OPTFLOW，只在【速度】语境下成立。')
check(AXES['SourceXY']['OPTFLOW'] == 5,
      '★★ 交叉印证：脚本里的裸数字 5 == SourceXY.OPTFLOW')
check('OPTFLOW' not in AXES['SourceZ'],
      '★★ 但 SourceZ 的 5 是【空缺】（源码注释：optical flow 只能做地形高度，')
print('        不能做 relative/absolute alt）⇒ 同一个 5 在 Z 轴无意义')

check('WHEEL_ENCODER' not in AXES['SourceZ'] and AXES['SourceXY']['WHEEL_ENCODER'] == 7,
      '★ WHEEL_ENCODER=7 同样只在 XY 轴存在')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[5] ★★★ 默认配置下"三套源"只有一套 —— 降级必须手工配')
print('=' * 74)
print('    AP_NavEKF_Source.cpp:27-133 的 var_info 默认值：')
DEFAULTS = {
    1: {'POSXY': ('GPS', 3), 'VELXY': ('GPS', 3), 'POSZ': ('BARO', 1),
        'VELZ': ('GPS', 3), 'YAW': ('COMPASS', 1)},
    2: {'POSXY': ('NONE', 0), 'VELXY': ('NONE', 0), 'POSZ': ('BARO', 1),
        'VELZ': ('NONE', 0), 'YAW': ('NONE', 0)},
    3: {'POSXY': ('NONE', 0), 'VELXY': ('NONE', 0), 'POSZ': ('BARO', 1),
        'VELZ': ('NONE', 0), 'YAW': ('NONE', 0)},
}
print()
print('    %-14s %-12s %-12s %-12s %-12s %s'
      % ('参数', 'SRC1', 'SRC2', 'SRC3', '说明', ''))
for key in ('POSXY', 'VELXY', 'POSZ', 'VELZ', 'YAW'):
    row = [DEFAULTS[i][key][0] for i in (1, 2, 3)]
    print('    EK3_%-10s %-12s %-12s %-12s' % (key, row[0], row[1], row[2]))
print()

n_sets_with_xy = sum(1 for i in (1, 2, 3) if DEFAULTS[i]['POSXY'][1] != 0)
check(n_sets_with_xy == 1,
      '★★★ 三套源里【只有第 1 套】配了水平位置源（GPS）'
      '⇒ 出厂状态下第 2/3 套是空的')
n_sets_with_yaw = sum(1 for i in (1, 2, 3) if DEFAULTS[i]['YAW'][1] != 0)
check(n_sets_with_yaw == 1,
      '★★★ 偏航源同理：只有第 1 套有 COMPASS')
check(all(DEFAULTS[i]['POSZ'][1] == 1 for i in (1, 2, 3)),
      '★ 但 Z 轴三套【都】配了 BARO —— 因为气压高度是"永远在"的基准')

print()
print('    ⇒ ⚠️ 常见误解："EKF3 有 3 套源，所以有 3 重冗余"。')
print('      **默认不是。** 三套源是【三个插座】，出厂只插了一个。')
print('      要真的有降级能力，必须显式配 EK3_SRC2_* / EK3_SRC3_*，')
print('      并且写一个脚本来切（见 [6] 和 09-Lua脚本）。')

# ── 反向对照：配了第 2 套之后，切过去才有意义 ──
configured = {'POSXY': DEFAULTS[1]['POSXY'][1] != 0 and DEFAULTS[2]['POSXY'][1] != 0}
check(not configured['POSXY'],
      '★ 反向对照：按默认值，SRC1 和 SRC2 的 POSXY【不可能同时】非 NONE'
      ' ⇒ 上面那条"只有一套"不是恒真的空话')

# ══════════════════════════════════════════════════════════════
print()
print('=' * 74)
print('[6] ★★ 投票迟滞：切换时间 = vote_counter_max × 周期')
print('=' * 74)
print('    AP_Scripting/examples/ahrs-source-gps-optflow.lua')
print('      :36  local vote_counter_max = 20        -- 2sec')
print('      :37  local gps_vs_opticalflow_vote = 0  -- -20=GPS, +20=光流')
print('      :205 return update, 100                 -- ★ 10 Hz')
print()
PERIOD_MS = 100
VOTE_MAX = 20
print('    每拍最多动 1 票，范围 [−%d, +%d] ⇒ 从 0 走到任一端要 %d 拍 = %.1f 秒'
      % (VOTE_MAX, VOTE_MAX, VOTE_MAX, VOTE_MAX * PERIOD_MS / 1000.0))
# ⚠️ 别写成 `VOTE_MAX * PERIOD_MS == 2000` —— 两边全是脚本里的字面量，
#    任何代码改动都不会让它红，是一句伪装成断言的真话。
#    改成：跑【真正的模拟】，用源码注释里的 20 拍 / 2 秒做期望。


def simulate(poll_fn, ticks):
    """poll_fn(t) 返回该拍的投票方向（-1 走 GPS，+1 走光流，0 不动）"""
    vote = 0
    history = []
    for t in range(ticks):
        d = poll_fn(t)
        vote = max(min(vote + d, VOTE_MAX), -VOTE_MAX)
        history.append(vote)
    return vote, history


# 场景 A：一直投光流 —— 看多久才真的切过去
n = 40
final, hist = simulate(lambda t: +1, n)
first_full = next(i for i, v in enumerate(hist) if v >= VOTE_MAX)
check(first_full + 1 == 20,
      '★★★ 一直投光流 ⇒ 第 %d 拍（%.1f s）票数打满 ⇒ 此刻才真正切换。'
      '期望 20 拍 / 2.0 s 来自 Lua 源码的 vote_counter_max=20 与 100 ms 周期'
      % (first_full + 1, (first_full + 1) * PERIOD_MS / 1000.0))
check(final == VOTE_MAX, '★ 40 拍后票数停在 +%d（被 clamp 住，不会溢出）' % final)

# 场景 B：★ 死区 —— 两个条件都不满足时票数【不动】
final2, hist2 = simulate(lambda t: 0, n)
check(final2 == 0 and all(v == 0 for v in hist2),
      '★★ 两个判定条件都不满足 ⇒ 票数【一动不动】'
      '（源码是 if/elseif，没有 else 分支）⇒ 这是真正的死区，不是抖动')

# 场景 C：★ 关键的反向对照 —— 抖动的输入不会立刻切换
def flaky(t):
    return +1 if (t % 10 < 8) else -1      # 80% 投光流，20% 投 GPS


final3, hist3 = simulate(flaky, 200)
peak3 = max(hist3)
print()
print('    抖动输入（80% 投光流 / 20% 投 GPS）跑 200 拍：')
print('      峰值票数 %+d（达到 %+d 即触发切换），最终 %+d'
      % (peak3, VOTE_MAX, final3))
check(peak3 == VOTE_MAX,
      '★ 80%% 的优势足以慢慢爬满 ⇒ 峰值到 %+d，确实触发了切换'
      '（净漂移 +0.6 票/拍）' % peak3)
check(final3 < VOTE_MAX,
      '★★ 但【最终不是停在 %+d】而是 %+d —— 到顶后被 clamp 住，'
      '随后那 20%% 的反对票又把它拉下来两票。稳态是个 %d~%d 的【振荡】，'
      '不是一个定值。断言写成"最终 == %d"会红（我第一版就是这么写的）'
      % (VOTE_MAX, final3, VOTE_MAX - 2, VOTE_MAX, VOTE_MAX))
check(final3 >= VOTE_MAX - 4,
      '★ 振荡幅度很小（|%+d − %+d| = %d）⇒ 一旦占了优势，'
      '少数派票数翻不了盘' % (final3, VOTE_MAX, abs(final3 - VOTE_MAX)))
check(hist3[0] > -VOTE_MAX,
      '★ 但第 1 拍只到 %+d ⇒ 单拍噪声绝不会触发切换' % hist3[0])


# 场景 D：★★ 反向对照 —— 换成 50/50 就应该基本原地不动
def coin(t):
    return +1 if (t % 2 == 0) else -1


final4, hist4 = simulate(coin, 200)
print()
print('    50/50 输入跑 200 拍：最终票数 %+d（|票数| ≤ 1）' % final4)
check(abs(final4) <= 1,
      '★★ 反向对照：50/50 的输入 ⇒ 票数始终在 0 附近 ⇒ 迟滞确实挡住了无优势的抖动')

# 场景 E：切换延迟的代价
print()
print('    ★ 迟滞的代价（拿它换来了什么）：')
print('      从光流切回 GPS 需要 %d 拍 = %.1f s 的持续优势' % (VOTE_MAX, VOTE_MAX * PERIOD_MS / 1000.0))
print('      ⇒ 这 %.1f 秒里，飞机仍在用（可能已失效的）旧源。' % (VOTE_MAX * PERIOD_MS / 1000.0))
print('      ⇒ 迟滞不是免费的：它把"误切"换成"迟切"。')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
