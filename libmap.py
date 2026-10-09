# -*- coding: utf-8 -*-
"""按出现频次统计每个算法库"在哪一课讲" —— 让数据决定地图，不靠印象。"""
import io
import _srcdirs
import os
import re
import collections

# ★★ 同 lesson49：本脚本量的是**素材树**，按已知位置找，不靠“我在哪”。
def _find_tut():
    here = os.path.dirname(os.path.abspath(__file__))
    for c in (os.path.join(here, '飞控学习教程'),
              os.path.join(os.path.dirname(here), '飞控学习教程'),
              _srcdirs.src("MAT")):
        if os.path.isdir(os.path.join(c, '01-数学基础')):
            return c
    raise SystemExit('找不到素材树 `飞控学习教程`。')


ROOT = _find_tut()

LIBS = """AC_AttitudeControl AC_AutoTune AC_Autorotation AC_Avoidance AC_CustomControl AC_Fence
AC_InputManager AC_PID AC_PrecLand AC_Sprayer AC_WPNav AP_ADSB AP_AHRS AP_AdvancedFailsafe
AP_Airspeed AP_Arming AP_Avoidance AP_Baro AP_BattMonitor AP_Beacon AP_Camera AP_Compass
AP_Declination AP_EFI AP_ExternalControl AP_Follow AP_GPS AP_GyroFFT AP_L1_Control AP_Landing
AP_LandingGear AP_Logger AP_Math AP_Mission AP_Motors AP_Mount AP_NavEKF2 AP_NavEKF3
AP_Navigation AP_OpticalFlow AP_OSD AP_Parachute AP_Param AP_Proximity AP_Quicktune AP_Rally
AP_RangeFinder AP_RPM AP_Scheduler AP_Scripting AP_SmartRTL AP_Soaring AP_TECS AP_Terrain
AP_Tuning AP_Vehicle AP_VisualOdom AP_WindVane APM_Control""".split()

files = []
for root, dirs, fs in os.walk(ROOT):
    parts = root.split(os.sep)
    if 'code' in parts or '.git' in parts:
        continue
    for f in fs:
        if f.endswith('.md'):
            files.append(os.path.join(root, f))

for lib in LIBS:
    hits = collections.Counter()
    for p in files:
        t = io.open(p, encoding='utf-8', errors='replace').read()
        n = len(re.findall(r'\b%s\b' % re.escape(lib), t))
        if n:
            rel = os.path.relpath(p, ROOT).replace(os.sep, '/')
            hits[rel] = n
    if not hits:
        print('%-22s **零命中**' % lib)
        continue
    top = hits.most_common(3)
    print('%-22s 共 %-4d 处；主：%s' % (
        lib, sum(hits.values()),
        '  '.join('%s(%d)' % (os.path.basename(p), n) for p, n in top)))
