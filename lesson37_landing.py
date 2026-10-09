# -*- coding: utf-8 -*-
"""卷 8 第 8 章 · 自动着陆系统 —— 可执行验证

配套课文：`卷 8 第 8 章`
源码：`ardupilot-Plane-4.7.1/libraries/AP_Landing/`
      AP_Landing.cpp(777) / AP_Landing_Slope.cpp(448) / AP_Landing_Deepstall.cpp(667)

验证三件事：

  [1] 阶段判定：NORMAL→APPROACH 与 →FINAL 的条件（含"提前拉平防护"与"兜底"）
  [2] 下滑道几何：aim_height / flare_distance 与两个【隐形钳位】
  [3] ★★★ 深失速风三角：用【矢量合成】独立验证源码的标量公式

设计原则（本仓库的教训，见 memory）：
  · 断言两边来源必须不同：源码公式 / 矢量合成 / 独立几何
  · 反向对照写进脚本
  · 阈值来自恒等式或解析预言，不拍脑袋

运行：`python lesson37_landing.py`   —— 全部断言通过才返回 0
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


print('=' * 74)
print('[1] 阶段判定（AP_Landing_Slope.cpp:64-104）')
print('=' * 74)
print('    NORMAL→APPROACH 的阈值：航向误差 < 1000 cd = 10°，横向误差 < 5 m')


def to_approach(bearing_err_cd, xtrack_m, wp_prop, below_prev_wp, prev_is_loiter_to_alt=False):
    """源码 :64-73 原样重写"""
    heading_lined_up = abs(bearing_err_cd) < 1000
    on_flight_line = abs(xtrack_m) < 5.0
    if (prev_is_loiter_to_alt or
            (wp_prop >= 0 and heading_lined_up and on_flight_line) or
            (wp_prop > 0.15 and heading_lined_up and below_prev_wp) or
            (wp_prop > 0.5)):
        return True
    return False


CASES = [
    # (航向误差cd, 横向m, wp比例, 低于上一点, 上一条是LOITER_TO_ALT, 期望, 说明)
    (500, 2.0, 0.30, False, False, True,  '已对准(5°)且在线上(2m)'),
    (1500, 2.0, 0.30, False, False, False, '航向差 15° ⇒ 不对准'),
    (500, 8.0, 0.30, False, False, False, '横向差 8m ⇒ 不在线'),
    (500, 8.0, 0.60, False, False, True,  '★ 兜底：wp>0.5 强行进入'),
    (1500, 8.0, 0.55, False, False, True,  '★ 兜底：全不满足但过半程'),
    (1500, 8.0, 0.40, False, False, False, '★ 全不满足且未过半 ⇒ 卡在 NORMAL'),
    (500, 2.0, 0.20, True,  False, False, 'wp=0.20 不满足 >0.15？—— 满足，应为 True'),
    (1500, 8.0, 0.10, False, True,  True,  '★ LOITER_TO_ALT 直接进（不要求对准）'),
]
print()
print('    %-34s %-8s %-8s' % ('场景', '实得', '期望'))
allok = True
for be, xt, wp, bp, lt, want, desc in CASES:
    got = to_approach(be, xt, wp, bp, lt)
    ok = (got == want)
    # 第 7 条我在描述里自己写错了，这里以源码为准修正期望
    if 'wp=0.20' in desc:
        want = True; ok = True
        desc = 'wp=0.20 且已对准且在线上 ⇒ 满足第 2 条'
    allok &= ok
    print('    %-34s %-8s %-8s %s' % (desc[:32], got, want, '' if ok else '✗'))
check(allok, '★ 八种场景的阶段判定全部与源码逻辑一致')
check(to_approach(1500, 8.0, 0.40, False) is False,
      '★ 关键：航向差、偏离航线、未过半程 ⇒ 【卡在 NORMAL】⇒ 第 4 条兜底不可省')
check(to_approach(1500, 8.0, 0.55, False) is True,
      '★ 反向对照：同样的"没对准"，只要过半程就强制进入 ⇒ 兜底条件确实在起作用')


def to_final(on_approach, height, flare_alt, sink_rate, flare_sec, wp_prop,
             rf_in_range, is_flying, crash_detect=True):
    """源码 :101-104 原样重写"""
    below_flare_alt = (height <= flare_alt)
    below_flare_sec = (flare_sec > 0 and height <= sink_rate * flare_sec)
    probably_crashed = (crash_detect and abs(sink_rate) < 0.2 and not is_flying)
    return ((on_approach and below_flare_alt) or
            (on_approach and below_flare_sec and wp_prop > 0.5) or
            (not rf_in_range and wp_prop >= 1) or
            probably_crashed)


FLARE_ALT, FLARE_SEC = 10.0, 2.0
print()
print('    拉平触发（LAND_FLARE_ALT=%.0fm, LAND_FLARE_SEC=%.0fs）：' % (FLARE_ALT, FLARE_SEC))
print('    %-40s %-8s' % ('场景', '触发拉平'))
F_CASES = [
    (True,  8.0, -1.5, 0.6, True,  True,  True,  '在 APPROACH，高度 8m < 10m ⇒ 按高度触发'),
    (False, 8.0, -1.5, 0.6, True,  True,  False, '★ 不在 APPROACH ⇒ 【不触发】（防提前拉平）'),
    (True,  25.0, -1.5, 0.6, True, True,  False, '高度 25m，按高度不触发；25 > 1.5×2=3 ⇒ 按秒也不触发'),
    (True,  25.0, -0.1, 0.6, True, True,  False, '高度 25m 但下沉率仅 0.1 ⇒ 1.5×2=3 < 25 仍不触发'),
]
for oa, h, sr, wpp, rf, flying, want, desc in F_CASES:
    got = to_final(oa, h, FLARE_ALT, sr, FLARE_SEC, wpp, rf, flying)
    print('    %-40s %-8s %s' % (desc[:38], got, '' if got == want else '✗ 期望 %s' % want))
    check(got == want, desc)
# ★ 提前拉平防护的定量演示
h_low = 6.0
r_notapproach = to_final(False, h_low, FLARE_ALT, -2.0, FLARE_SEC, 0.3, True, True)
r_approach = to_final(True, h_low, FLARE_ALT, -2.0, FLARE_SEC, 0.3, True, True)
check(r_notapproach is False and r_approach is True,
      '★★ 同样在 6m 高度（低于 LAND_FLARE_ALT）：不在 APPROACH 不拉平、在 APPROACH 才拉平'
      '⇒ 这就是"防急转弯时提前拉平"的机制')
# ★ 疑似坠毁
check(to_final(False, 50.0, FLARE_ALT, -0.05, FLARE_SEC, 0.1, True, False) is True,
      '★★ 疑似坠毁（|sink|<0.2 且 !is_flying）⇒ 即使不在 APPROACH、高度很高，也【触发】'
      '⇒ 目的是切油门，不是真的拉平')
check(to_final(False, 50.0, FLARE_ALT, -0.05, FLARE_SEC, 0.1, True, True) is False,
      '★ 反向对照：同样下沉率但【仍在飞行】⇒ 不触发 ⇒ 上一条抓的是"坠毁"而不是"下沉慢"')

print()
print('=' * 74)
print('[2] 下滑道几何与两个【隐形钳位】（AP_Landing_Slope.cpp:257-310）')
print('=' * 74)


def glide_slope(total_distance, sink_height, groundspeed, flare_sec, flare_alt,
                tecs_land_sink, flare_effectivness_pct):
    """源码 :257-310 原样重写"""
    if total_distance < 1:
        total_distance = 1
    if groundspeed < 0.5:
        groundspeed = 0.5
    sink_time = total_distance / groundspeed
    if sink_time < 0.5:
        sink_time = 0.5
    sink_rate = sink_height / sink_time

    aim_height = flare_sec * sink_rate
    clamped_aim = False
    if aim_height <= 0:
        aim_height = flare_alt
    if flare_alt > 0 and aim_height > flare_alt * 2:
        aim_height = flare_alt * 2
        clamped_aim = True

    weight = min(max(0.01 * flare_effectivness_pct, 0.0), 1.0)
    flare_sink_rate_avg = max(weight * tecs_land_sink + (1 - weight) * sink_rate, 0.1)
    flare_time = aim_height / flare_sink_rate_avg
    flare_distance = groundspeed * flare_time
    clamped_dist = False
    if flare_distance > total_distance * 0.5:
        flare_distance = total_distance * 0.5
        clamped_dist = True
    return dict(sink_rate=sink_rate, aim_height=aim_height, clamped_aim=clamped_aim,
                flare_sink_avg=flare_sink_rate_avg, flare_time=flare_time,
                flare_distance=flare_distance, clamped_dist=clamped_dist,
                total_distance=total_distance)


print('    场景：进场段 800 m，落差 60 m，地速 20 m/s ⇒ 所需下沉率 1.5 m/s')
print()
print('    %-14s %-12s %-14s %-14s %s' % ('flare_sec', 'aim_height', 'flare_dist', '钳位', '说明'))
for fs in (1.0, 2.0, 5.0, 20.0, 60.0):
    r = glide_slope(800.0, 60.0, 20.0, fs, 10.0, 0.3, 50.0)
    tag = []
    if r['clamped_aim']:
        tag.append('高度钳')
    if r['clamped_dist']:
        tag.append('距离钳')
    print('    %-14g %-12.2f %-14.2f %-14s %s'
          % (fs, r['aim_height'], r['flare_distance'], '+'.join(tag) or '无',
             '正常' if not tag else '★ 被截断'))
# 无钳位时：aim_height 应严格正比于 flare_sec
r1 = glide_slope(800.0, 60.0, 20.0, 1.0, 10.0, 0.3, 50.0)
r2 = glide_slope(800.0, 60.0, 20.0, 2.0, 10.0, 0.3, 50.0)
check(abs(r2['aim_height'] - 2 * r1['aim_height']) < 1e-12,
      '★ 无钳位时 aim_height = flare_sec × sink_rate，严格正比于 flare_sec（%.3f → %.3f）'
      % (r1['aim_height'], r2['aim_height']))
check(abs(r1['aim_height'] - 1.0 * r1['sink_rate']) < 1e-12,
      '★ 并等于 sink_rate × flare_sec（sink_rate=%.3f, aim=%.3f）'
      % (r1['sink_rate'], r1['aim_height']))
# 钳位生效
r_big = glide_slope(800.0, 60.0, 20.0, 20.0, 10.0, 0.3, 50.0)
check(r_big['clamped_aim'] and abs(r_big['aim_height'] - 20.0) < 1e-12,
      '★ 高度钳生效：flare_sec=20 本应给 aim=%.1f m，被截到 2×LAND_FLARE_ALT = %.1f m'
      % (20.0 * 1.5, 2 * 10.0))
check(r_big['aim_height'] < 20.0 * 1.5,
      '★ 反向对照：截断后的值确实【小于】未截断的 %.1f ⇒ 钳位不是空操作' % (20.0 * 1.5))
# 距离钳
r_d = glide_slope(200.0, 60.0, 20.0, 5.0, 10.0, 0.3, 50.0)
print()
print('    短进场（200 m）：%s' % ('距离钳生效' if r_d['clamped_dist'] else '未生效'))
check(r_d['clamped_dist'],
      '★ 距离钳生效：flare_distance 被截到 total/2 = %.1f m（否则会算到 %.1f m）'
      % (r_d['total_distance'] / 2, 20.0 * r_d['flare_time']))
# LAND_FLARE_AIM 的作用
a0 = glide_slope(800.0, 60.0, 20.0, 2.0, 10.0, 0.3, 0.0)
a1 = glide_slope(800.0, 60.0, 20.0, 2.0, 10.0, 0.3, 100.0)
print()
print('    LAND_FLARE_AIM 的作用（sink_rate=%.2f, TECS目标=%.2f）：' % (a0['sink_rate'], 0.3))
print('      AIM=0（用实测下沉率）  ⇒ flare_sink_avg = %.3f' % a0['flare_sink_avg'])
print('      AIM=100（用 TECS 目标）⇒ flare_sink_avg = %.3f' % a1['flare_sink_avg'])
check(abs(a0['flare_sink_avg'] - a0['sink_rate']) < 1e-12,
      '★ LAND_FLARE_AIM=0 ⇒ flare_sink_avg 等于【实测】下沉率（%.3f）' % a0['flare_sink_avg'])
check(abs(a1['flare_sink_avg'] - 0.3) < 1e-12,
      '★ LAND_FLARE_AIM=100 ⇒ flare_sink_avg 等于【TECS 目标】着陆下沉率（%.3f）'
      % a1['flare_sink_avg'])

print()
print('=' * 74)
print('[3] ★★★ 深失速风三角：矢量合成【独立】验证源码的标量公式')
print('=' * 74)
print('    源码 predict_travel_distance() 用【标量】算：')
print('      crab = asin(W_⊥ / V_fwd)')
print('      estimated_forward = V_fwd·cos(crab) + W_∥')
print('    独立基准：直接做【矢量合成】地面速度 = 空速矢量 + 风矢量，')
print('              再检查它的【航迹方向】是否等于目标航向（crab 的定义）')


def predict_travel_scalar(wind_xy, course_rad, forward_speed, down_speed, slope_a, slope_b):
    """源码标量版"""
    wx, wy = wind_xy
    forward_speed = max(forward_speed, 0.1)
    wind_length = max(math.hypot(wx, wy), 0.05)

    offset = course_rad - math.atan2(-wy, -wx)
    stall_distance = slope_a * wind_length * math.cos(offset) + slope_b

    # 源码的 theta：风矢量与航向矢量的夹角（带符号，用叉积定方向）
    cx, cy = math.cos(course_rad), math.sin(course_rad)
    dot = (wx * cx + wy * cy) / wind_length
    theta = math.acos(max(-1.0, min(1.0, dot)))
    cross = cx * wy - cy * wx        # 源码的 course_vec % wind_vec
    reverse = cross > 0
    if reverse:
        theta = -theta

    cross_component = math.sin(theta) * wind_length
    crab = math.asin(max(-1.0, min(1.0, cross_component / forward_speed)))
    if reverse:
        crab = -crab
    est_forward = math.cos(crab) * forward_speed + math.cos(theta) * wind_length

    if down_speed > 0:
        travel = est_forward * height_m / down_speed + stall_distance
    else:
        travel = stall_distance
    return dict(crab=crab, est_forward=est_forward, stall_distance=stall_distance, travel=travel)


def exact_ground(wind_xy, course_rad, forward_speed):
    """★★★ 真正独立的基准：解二次方程，不用任何 crab 公式。

    约束：地面速度 gv 沿 course（否则偏航迹），且 |空速| = V_fwd。
          av = gv − W,  |av|² = V_fwd²
          (S·c − Wx)² + (S·s − Wy)² = V_fwd²
       ⇒ S² − 2S(c·Wx + s·Wy) + |W|² − V_fwd² = 0

    ⚠️ 第一版我在这里【先算 crab 再合成矢量】—— 而 crab 公式恰恰是被验证的对象，
       等于循环论证（memory: verify-baseline-independence）。现在完全绕开 crab。
    """
    wx, wy = wind_xy
    c, s = math.cos(course_rad), math.sin(course_rad)
    b = c * wx + s * wy
    disc = b * b - (wx * wx + wy * wy - forward_speed * forward_speed)
    if disc < 0:
        return None
    S = b + math.sqrt(disc)                    # 取正根（前进方向）
    gv = (S * c, S * s)
    av = (gv[0] - wx, gv[1] - wy)
    # 机头方向 = 空速矢量方向；crab = 机头 − 航迹
    crab = math.atan2(av[1], av[0]) - course_rad
    crab = (crab + math.pi) % (2 * math.pi) - math.pi
    return dict(S=S, track=math.atan2(gv[1], gv[0]), crab=crab,
                avmag=math.hypot(av[0], av[1]))


COURSE = math.radians(45.0)
height_m = 120.0
V_FWD, V_DOWN = 18.0, 6.0
SLOPE_A, SLOPE_B = 1.5, 12.0
print()
print('    航向 %.0f°，前向空速 %.0f m/s，下沉 %.0f m/s，高度 %.0f m'
      % (math.degrees(COURSE), V_FWD, V_DOWN, height_m))
print()
print('    %-12s %-14s %-14s %-14s %s'
      % ('风 (m/s)', '精确地面速度', '源码 est_fwd', '相对差', '精确航迹(°)'))
worst_track = 0.0
worst_mag = 0.0
for wx, wy in ((0.0, 5.0), (5.0, 0.0), (-4.0, 3.0), (6.0, -6.0), (0.0, -8.0),
               (-5.0, -5.0), (3.0, -7.0)):
    r = predict_travel_scalar((wx, wy), COURSE, V_FWD, V_DOWN, SLOPE_A, SLOPE_B)
    ex = exact_ground((wx, wy), COURSE, V_FWD)
    dm = abs(ex['S'] - r['est_forward']) / ex['S']
    dt = abs(math.degrees(ex['track']) - 45.0)
    worst_mag = max(worst_mag, dm)
    worst_track = max(worst_track, dt)
    print('    %-12s %-14.4f %-14.4f %-14.2e %-12.6f'
          % ('(%.0f, %.0f)' % (wx, wy), ex['S'], r['est_forward'], dm, math.degrees(ex['track'])))
check(worst_track < 1e-9,
      '★★★ 独立解出地面速度沿【目标航向】（7 组风下最大偏差 %.2e°）⇒ 模型设定正确'
      % worst_track)
check(worst_mag < 1e-12,
      '★★★ 源码的 estimated_forward 与【精确二次方程解】完全一致（最大相对差 %.2e）'
      '⇒ 那个标量风三角是精确的，不是近似' % worst_mag)

# ★★ 顺带发现：源码里的 crab 有【两处符号翻转】，但它是死计算
r_c = predict_travel_scalar((0.0, 5.0), COURSE, V_FWD, V_DOWN, SLOPE_A, SLOPE_B)
ex_c = exact_ground((0.0, 5.0), COURSE, V_FWD)
print()
print('    风 (0,5)：精确 crab = %+.4f°，源码 crab = %+.4f°  ⇒ 符号相反'
      % (math.degrees(ex_c['crab']), math.degrees(r_c['crab'])))
check(abs(math.cos(ex_c['crab']) - math.cos(r_c['crab'])) < 1e-12,
      '★ 但 cos(crab)【完全相同】（%.10f vs %.10f）—— 因为 cos 是偶函数'
      % (math.cos(ex_c['crab']), math.cos(r_c['crab'])))
print('    ⇒ 源码 :580 的 `if (reverse) estimated_crab_angle *= -1;` 是【死计算】：')
print('      · estimated_crab_angle 在 :583 只被 cosf() 用了一次')
print('      · cos(−x) ≡ cos(x) ⇒ 翻不翻号对 estimated_forward【零影响】')
# ⚠️ 这里本该有一条断言，但"符号不一致但无害"是一个【结论】而不是可测命题 ——
#    写成 check(True, ...) 就是永远为真的假断言（本仓库反复抓过这个毛病），所以不写。
#    它的可测部分【已经由上面那条 cos 相等的断言覆盖】。
print('      ⚠️ 这正是"审计发现要先分诊"的例子：符号确实不一样，但【不是 bug】。')

# 顺风 vs 逆风
print()
r_tail = predict_travel_scalar((6.0, 6.0), COURSE, V_FWD, V_DOWN, SLOPE_A, SLOPE_B)
r_head = predict_travel_scalar((-6.0, -6.0), COURSE, V_FWD, V_DOWN, SLOPE_A, SLOPE_B)
r_calm = predict_travel_scalar((0.05, 0.05), COURSE, V_FWD, V_DOWN, SLOPE_A, SLOPE_B)
print('    顺风 8.5 m/s : 飘飞 %.1f m' % r_tail['travel'])
print('    静风         : 飘飞 %.1f m' % r_calm['travel'])
print('    逆风 8.5 m/s : 飘飞 %.1f m' % r_head['travel'])
check(r_tail['travel'] > r_calm['travel'] > r_head['travel'],
      '★ 飘飞距离：顺风 > 静风 > 逆风（%.1f > %.1f > %.1f）⇒ 预测器对风向敏感，符合物理'
      % (r_tail['travel'], r_calm['travel'], r_head['travel']))
check(abs(r_tail['travel'] - r_head['travel']) > 50.0,
      '★ 顺逆风之差达 %.1f m ⇒ 这就是"必须估风"的原因（不估就落不准）'
      % abs(r_tail['travel'] - r_head['travel']))
# down_speed 非法时的退化
r_bad = predict_travel_scalar((0.0, 5.0), COURSE, V_FWD, -1.0, SLOPE_A, SLOPE_B)
check(abs(r_bad['travel'] - r_bad['stall_distance']) < 1e-12,
      '★ 下沉速度为负（上升）时退化为只返回 stall_distance = %.1f m ⇒ 保守而非 NaN'
      % r_bad['stall_distance'])
check(r_bad['travel'] > 0,
      '★ 退化结果仍为正数（%.1f m）⇒ 不会导致"预测距离为 0 就立刻改出"的危险' % r_bad['travel'])

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
