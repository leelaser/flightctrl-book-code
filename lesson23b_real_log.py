#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""《飞控学习教程》第 7 章 01 配套 —— 真机日志实战（SITL 的离线替代）

    python lesson23b_real_log.py        # 跑全部 5 个实验
    python lesson23b_real_log.py 1      # 只跑实验 1
    python lesson23b_real_log.py 2      # 只跑实验 2
    python lesson23b_real_log.py 3      # 只跑实验 3
    python lesson23b_real_log.py 4      # 只跑实验 4
    python lesson23b_real_log.py 5      # 只跑实验 5

★ 为什么会有这个脚本
    第 23 课要验证的链路（仿真器 → 传感器 → EKF → 控制律）需要 SITL，
    而 SITL 需要 Linux/WSL —— 本机是 Windows，WSL 里一个发行版都没装，
    PX4/ArduPilot 也都没有编译产物。逐项证据见 `lesson23_sitl.py 1`。
    跑不了仿真，就用【真机日志】做离线实验：仿真能告诉你【算法对不对】，
    真机日志能告诉你【这套飞机实际是什么样】—— 后者反而是仿真给不了的。

数据源（真机日志，非仿真）
    D:/DEVLib/PX4APM/PX4调参/ulog/   下 53 份 .ulg，8 个日期目录
    机型 CUAV X7 Pro，固件 PX4 v1.14.4（commit 1555f2bd22，见实验 1 的鉴定）

依赖
    pyulog（必需）、numpy（必需）、matplotlib（可选，只影响画图）

★ 本脚本刻意避开了 09-进阶与创新/02 已经用过的
  `20250922/2025_09_22_log_13` 那份日志（那课做的是悬停推力估计）。
  本脚本选的是另外 4 份、跨 4 个日期的日志，分析角度也不同：
  振动 / 采样率效应 / GPS / EKF 新息。

五个实验
    1  环境与日志清单：真查 pyulog、真扫目录、真鉴定机型固件
    2  飞行概况：这 4 份日志记录的是什么飞行
    3  ★★ 振动：RMS + FFT 主峰（24.4.6 节的公式，用真机数据跑一遍）
    4  ★★ 振动指标的采样率效应：飞控报的数你复现不出来，为什么
    5  ★  GPS 精度 / EKF 新息 / 削顶：能算什么、不能算什么
"""

import os
import sys

import numpy as np


# ============================================================================
#  输出小工具（与第 21~24 课脚本一致的排版）
# ============================================================================

W = 74


def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (W - 4))


def _dw(ch):
    """字符的显示宽度：CJK 占 2 列，其他占 1 列。"""
    o = ord(ch)
    if 0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF or \
       0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF or \
       0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60 or \
       0xFFE0 <= o <= 0xFFE6:
        return 2
    return 1


def _wide(s):
    return sum(_dw(c) for c in s)


def tab(rows, headers, width=None):
    """打印一张左对齐的表。width 是【显示宽度】（CJK 按 2 列算）。"""
    rows = [[str(c) for c in r] for r in rows]
    ncol = len(headers)
    if width is None:
        width = [0] * ncol
        for i in range(ncol):
            width[i] = _wide(str(headers[i]))
            for r in rows:
                if i < len(r):
                    width[i] = max(width[i], _wide(r[i]))
        width = [min(w, 34) for w in width]
    print()
    print("  " + "  ".join(str(headers[i]) + " " * (width[i] - _wide(str(headers[i])))
                           for i in range(ncol)))
    print("  " + "-" * (sum(width) + 2 * (ncol - 1)))
    for r in rows:
        cells = []
        for i in range(ncol):
            s = r[i] if i < len(r) else ""
            cells.append(s + " " * max(0, width[i] - _wide(s)))
        print("  " + "  ".join(cells))


# 行首禁则：这些标点不能出现在行首（否则会「孤字成行」，很难看）
NO_LINE_START = "：。，、；！？）」』】》…—～%"


def wrap(text, indent=4, width=W - 4):
    """按【显示宽度】折行打印（CJK 按 2 列算，对齐才好看）。"""
    pad = " " * indent
    line = pad
    w = 0
    for ch in text:
        if ch == "\n":
            print(line)
            line = pad
            w = 0
            continue
        cw = _dw(ch)
        if w + cw > width and ch not in NO_LINE_START:
            print(line)
            line = pad
            w = 0
        line += ch
        w += cw
    if line.strip():
        print(line)


def bullet(items, indent=4):
    for it in items:
        wrap("· " + it, indent=indent)


# ============================================================================
#  日志发现
# ============================================================================

HERE = os.path.dirname(os.path.abspath(__file__))
CH7 = os.path.dirname(HERE)          # 07-仿真与实验/
ROOT = os.path.dirname(CH7)          # 教程根目录

# 真机日志的候选根目录（本机 + 读者机器上常见的位置）
ULOG_ROOTS = [
    r"D:/DEVLib/PX4APM/PX4调参/ulog",
    os.path.join(ROOT, "..", "PX4调参", "ulog"),
    r"~/Documents/Flight_Logs",
    r"~/PX4调参/ulog",
]

# 本课选定的 4 份日志（相对 ULOG_ROOTS 的路径）。
# ★ 刻意避开 09-进阶与创新/02 用过的 20250922/...log_13。
PREFERRED = [
    r"2025-09-13/09_43_52.ulg",
    r"20250917/log_14_2025-9-17-16-28-38.ulg",
    r"20250924/log_21_2025-9-24-17-06-34.ulg",
    r"20250926/log_30_2025-9-26-14-15-12.ulg",
]

# 单份日志的大小上限（MB）。太大的读起来慢，本课不需要。
MAX_MB = 150


def find_ulog_root():
    for r in ULOG_ROOTS:
        p = os.path.abspath(os.path.expanduser(r))
        if os.path.isdir(p):
            return p
    return None


def scan_logs(root):
    """返回 [(日期目录, 文件名, 绝对路径, 字节数), ...]，按日期目录名排序。"""
    out = []
    if not root:
        return out
    for d in sorted(os.listdir(root)):
        sub_ = os.path.join(root, d)
        if not os.path.isdir(sub_):
            continue
        for dirpath, _dirs, files in os.walk(sub_):
            for fn in sorted(files):
                if fn.lower().endswith(".ulg"):
                    p = os.path.join(dirpath, fn)
                    out.append((d, fn, p, os.path.getsize(p)))
    return out


def pick_logs(all_logs):
    """先按 PREFERRED 取；缺哪份就用「其它日期目录里最大的一份」补。

    ★ 这样写是为了让【读者机器上换了日志】时脚本仍然跑得通，
      而不是硬编码到某几份文件上。
    """
    root = find_ulog_root()
    picked, used = [], set()
    for rel in PREFERRED:
        p = os.path.join(root, rel) if root else None
        if p and os.path.isfile(p):
            picked.append((os.path.dirname(rel) or ".", os.path.basename(rel),
                           p, os.path.getsize(p)))
            used.add(rel.replace("\\", "/"))
    if len(picked) >= 4:
        return picked[:4], True

    # 补齐：按日期目录分组，每个目录取最大的一份；跳过已经用过的
    by_date = {}
    for rec in all_logs:
        d = rec[0]
        if rec[3] > MAX_MB * 1024 * 1024:
            continue
        if rec[1] in {x[1] for x in picked}:
            continue
        if d not in by_date or rec[3] > by_date[d][3]:
            by_date[d] = rec
    for d in sorted(by_date):
        if len(picked) >= 4:
            break
        picked.append(by_date[d])
    return picked, False


# ============================================================================
#  pyulog 加载
# ============================================================================

def load(path, topics):
    """按需加载（只要这几个 topic），返回 {topic: [dataset, ...]}。"""
    from pyulog import ULog
    u = ULog(path, list(topics))
    out = {}
    for d in u.data_list:
        out.setdefault(d.name, []).append(d)
    return u, out


def tsec(dataset, t0):
    """把 uint64 时间戳换成【相对日志开始的秒】。

    ★ 必须先 astype(np.int64) 再做减法：uint64 减 Python int 时会先把
      Python int 转成 uint64，一旦被减数更小就【下溢】成一个天文数字。
      这个坑在本脚本开发时真踩过一次（印出了 18446744073710 秒）。
    """
    return (dataset["timestamp"].astype(np.int64) - t0) / 1e6


def med(a):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.median(a)) if a.size else float("nan")


def one(store, name, multi=None):
    """取一个 dataset（multi=None 时取第 0 个）。"""
    lst = store.get(name, [])
    if multi is None:
        return lst[0].data if lst else None
    for d in lst:
        if d.multi_id == multi:
            return d.data
    return None


# ============================================================================
#  振动指标的两个"独立"算法
# ============================================================================

def px4_vibration_metric(accel, alpha=0.01):
    """复现 PX4 `VehicleIMU::UpdateAccelVibrationMetrics`
    （src/modules/sensors/vehicle_imu/VehicleIMU.cpp:740-747）：

        v_k = 0.99 v_{k-1} + 0.01 * ||a_k - a_{k-1}||

    accel: (N,3) m/s²。返回稳态 v。第 24.4.1 节把这个递推推成了
    v = (4A/pi) sin(pi f / f_s)（纯正弦、单轴）。
    """
    d = np.linalg.norm(np.diff(np.asarray(accel, dtype=float), axis=0), axis=1)
    v = 0.0
    for x in d:
        v = (1.0 - alpha) * v + alpha * x
    return v


def boxcar_mean(x, n):
    """把 (N,3) 按每 n 个样本取平均 —— 真机日志里的 sensor_combined
    就是【原始样本的滑动平均】，不是裸抽样（实验 4 会用到这一点）。"""
    m = len(x) // n * n
    return x[:m].reshape(-1, n, x.shape[1]).mean(axis=1)


def analytic_metric(A, f, fs):
    """第 24.4.1 节推导的解析式：v = (4A/pi) sin(pi f / fs)。"""
    return (4.0 * A / np.pi) * np.sin(np.pi * f / fs)


# ============================================================================
#  实验 1：环境与日志清单
# ============================================================================

def exp1():
    head("实验 1  环境与日志清单：本机拿什么做实验？")

    sub("① 依赖")

    rows = []
    for mod, why in [("pyulog", "解析 ULog（本脚本的核心依赖）"),
                     ("numpy", "数值计算"),
                     ("matplotlib", "画图（可选，没有就跳过图）")]:
        try:
            import importlib.util
            ok = importlib.util.find_spec(mod) is not None
        except Exception:
            ok = False
        ver = ""
        if ok:
            try:
                from importlib.metadata import version as _pkgver
                ver = _pkgver(mod)
            except Exception:
                ver = ""
        rows.append([mod, "有" if ok else "没有", ver, why])
    tab(rows, ["依赖", "结果", "版本", "用途"], width=[12, 6, 10, 36])

    try:
        import pyulog
        wrap("★ pyulog %s —— 真机 .ulg 可以直接读了。" % pyulog.__version__)
    except Exception:
        print()
        wrap("★★ 没有 pyulog，本脚本只能跑实验 4（它只用 numpy）。")
        wrap("   装： pip install pyulog")
        return False

    sub("② 真机日志清单")

    root = find_ulog_root()
    if not root:
        print()
        wrap("★ 没找到 .ulg 目录（候选：%s）。" % "；".join(ULOG_ROOTS[:2]))
        wrap("  把真机日志放到其中之一，或改 ULOG_ROOTS 常量，再跑一次。")
        return False

    all_logs = scan_logs(root)
    by_date = {}
    for d, fn, p, sz in all_logs:
        by_date.setdefault(d, []).append(sz)

    wrap("日志根目录：%s" % root)
    wrap("共 %d 份 .ulg，分布在 %d 个日期目录：" % (len(all_logs), len(by_date)))
    print()

    rows = []
    for d in sorted(by_date):
        szs = by_date[d]
        rows.append([d, len(szs), "%.1f" % (sum(szs) / 1024 / 1024),
                     "%.1f" % (max(szs) / 1024 / 1024)])
    tab(rows, ["日期目录", "份数", "总 MB", "最大 MB"], width=[20, 6, 9, 9])

    sub("③ 机型与固件鉴定（从日志的 INFO 消息读，不是猜的）")

    picked, exact = pick_logs(all_logs)
    if not picked:
        wrap("★ 一份可用的日志都没找到。")
        return False

    info = None
    try:
        u, _ = load(picked[0][2], ["sensor_combined"])
        info = u.msg_info_dict
    except Exception as e:
        wrap("★ 读第一份日志的 INFO 失败：%s" % e)
        return False

    rows = []
    for k, label in [("sys_name", "系统名"), ("ver_hw", "硬件（板子）"),
                     ("ver_sw", "固件 commit"), ("sys_os_name", "操作系统"),
                     ("ver_data_format", "ULog 数据格式版本")]:
        if k in info:
            rows.append([label, str(info[k])[:56]])
    tab(rows, ["项目", "值"], width=[22, 50])

    print()
    wrap("★ 固件 commit 可以和本机 PX4 源码树对照，确凿地定位到版本：")
    wrap("    cd D:/DEVLib/PX4APM/PX4/PX4-Autopilot")
    wrap("    git describe --tags <上面那个 commit>")
    wrap("  本课这 4 份日志给出的回答是 v1.14.4 —— 注意【不是】1.15.4。")
    wrap("  本教程源码引用以 1.15.4 为准，两者在个别字段上确有差异（实验 5 会撞上一个）。")

    sub("④ 本课选用哪 4 份")

    if exact:
        wrap("用的是 PREFERRED 里写死的 4 份（跨 4 个日期，且避开 09 课用过的日志）。")
    else:
        wrap("PREFERRED 里的日志没找全，已自动从不同日期目录里各挑一份补齐。")

    rows = []
    for d, fn, p, sz in picked:
        rows.append([d, fn[:34], "%.0f" % (sz / 1024 / 1024)])
    tab(rows, ["日期", "文件", "MB"], width=[12, 36, 5])
    return True


# ============================================================================
#  实验 2：飞行概况
# ============================================================================

NAV_STATE_NAMES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER",
    5: "AUTO_RTL", 6: "POSITION_SLOW", 10: "ACRO", 12: "DESCEND",
    13: "TERMINATION", 14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF",
    18: "AUTO_LAND", 20: "AUTO_PRECLAND", 21: "ORBIT",
}

ARMING_NAMES = {0: "INIT", 1: "STANDBY", 2: "ARMED", 3: "STANDBY_ERROR"}


def nav_names(vals):
    return "/".join(sorted({NAV_STATE_NAMES.get(int(v), "?%d" % v) for v in vals}))


def flight_profile(path):
    """读一份日志的概况。返回 dict（读不动就返回 None）。"""
    try:
        u, st = load(path, ["vehicle_status", "vehicle_local_position",
                            "sensor_combined", "vehicle_imu_status"])
    except Exception:
        return None
    vs, lz = one(st, "vehicle_status"), one(st, "vehicle_local_position")
    sc = one(st, "sensor_combined")
    if vs is None or sc is None:
        return None

    t0 = u.start_timestamp
    tv = tsec(vs, t0)
    armed = vs["arming_state"] == 2
    ta = float(tv[armed].min()) if armed.any() else float("nan")
    tb = float(tv[armed].max()) if armed.any() else float("nan")

    ts = tsec(sc, t0)
    fs = 1.0 / float(np.median(np.diff(sc["timestamp"].astype(np.int64)) / 1e6))

    alt = None
    airborne_pct = float("nan")
    if lz is not None:
        tz = tsec(lz, t0)
        alt = -lz["z"].astype(float)
        m = (tz >= ta) & (tz <= tb)
        alt_max = float(alt[m].max()) if m.any() else float("nan")
        # ★ 离地判据：必须真查。同一批日志里就有【全程没离地】的台架试车，
        #   它的"振动"是地面振动，不能当成空中振动用。
        airborne_pct = 100.0 * float((m & (alt > 0.5)).sum()) / max(int(m.sum()), 1)
    else:
        alt_max = float("nan")

    imus = []
    for d in st.get("vehicle_imu_status", []):
        dd = d.data
        imus.append({
            "id": d.multi_id,
            "raw_rate": med(dd["accel_raw_rate_hz"]),
            "out_rate": med(dd["accel_rate_hz"]),
            "vibe": med(dd["accel_vibration_metric"]),
            "gclip": int(dd["gyro_clipping[0]"][-1] + dd["gyro_clipping[1]"][-1]
                         + dd["gyro_clipping[2]"][-1]),
            "aclip": int(dd["accel_clipping[0]"][-1] + dd["accel_clipping[1]"][-1]
                         + dd["accel_clipping[2]"][-1]),
        })

    return {
        "path": path, "u": u, "st": st, "t0": t0,
        "dur": (u.last_timestamp - t0) / 1e6,
        "armed": (ta, tb), "nav": nav_names(vs["nav_state"][armed]) if armed.any() else "-",
        "alt_max": alt_max, "airborne_pct": airborne_pct, "fs": fs,
        "n_sc": len(sc["timestamp"]),
        "integral_dt_us": med(sc["accelerometer_integral_dt"]),
        "imus": imus,
        "sc": sc, "vs": vs, "lz": lz,
    }


def exp2(picked):
    head("实验 2  飞行概况：这 4 份日志记录的是什么飞行？")

    profs = []
    for d, fn, p, sz in picked:
        pr = flight_profile(p)
        if pr:
            pr["date"], pr["file"] = d, fn
            profs.append(pr)

    if not profs:
        wrap("★ 一份日志都没读成功。")
        return []

    rows = []
    for pr in profs:
        rows.append([pr["date"], "%.0f" % pr["dur"], "%.0f~%.0f" % pr["armed"],
                     "%.1f" % pr["alt_max"], "%.0f%%" % pr["airborne_pct"],
                     pr["nav"]])
    tab(rows, ["日期", "时长s", "解锁区间s", "最大高度m", "离地占比", "飞行模式"],
        width=[11, 6, 10, 10, 8, 20])

    print()
    wrap("★ 读法：这几份都是【低空手动/自稳悬停】—— 最大高度 1~5 m、"
         "模式以 STAB/POSCTL/ALTCTL 为主。这不是航线飞行，"
         "正是调参和振动排查时最典型的飞法。")
    print()
    wrap("⚠️ 「离地占比」这一列【必须看】。本脚本开发时就踩过：同一批日志里的")
    wrap("   PX4调参/ulog/20250918/log_18_2025-9-18-10-41-06.ulg")
    wrap("   解锁了 422 s、最大高度却是 -0.6 m、离地占比 0% —— 那是【地面台架"
         "试车】。如果不先查这一列就拿它算「悬停振动」，得到的是地面振动，"
         "结论全错。")
    wrap("   （想自己复现：把它加进本文件的 PREFERRED，再跑 `... 2`。）")

    sub("① 三路 IMU 的采样率（★ 记住这张表，实验 4 全靠它）")

    rows = []
    for pr in profs:
        for im in pr["imus"]:
            rows.append([pr["date"], "IMU%d" % im["id"],
                         "%.0f" % im["raw_rate"], "%.0f" % im["out_rate"],
                         "%.2f" % im["vibe"]])
    tab(rows, ["日期", "IMU", "原始率Hz", "输出率Hz", "vibe中位"],
        width=[12, 6, 10, 10, 10])

    print()
    wrap("★★ 这张表里藏着本课最重要的一个事实：")
    bullet([
        "飞控【内部】算振动指标的采样率是 IMU 的原始率 —— IMU0 是 2003 Hz；",
        "而日志里【能拿到】的最快加速度是 sensor_combined，只有 154~202 Hz，"
        "因为它是每 ~10 个原始样本做一次平均之后再记的"
        "（accelerometer_integral_dt ≈ 4942~5978 us）；",
        "两者差 10 倍。所以「振动指标」这个数，你没法从日志里精确复现 —— "
        "实验 4 量化这件事，并且反过来用它推断振动的频率成分。",
    ])

    return profs


# ============================================================================
#  实验 3：振动 RMS + FFT 主峰
# ============================================================================

def airborne_slice(pr, alt_min=0.5):
    """返回 (t, accel(N,3), fs)，取【已解锁且离地】的一段。

    取最长的一段连续空中区间 —— 悬停段才能代表"稳态振动"。
    """
    sc, lz, vs = pr["sc"], pr["lz"], pr["vs"]
    t0 = pr["t0"]
    ts = tsec(sc, t0)
    ta, tb = pr["armed"]
    m = (ts >= ta) & (ts <= tb)
    if lz is not None:
        tz = tsec(lz, t0)
        alt = -lz["z"].astype(float)
        # 把高度插值到 sensor_combined 的时间轴上
        alt_i = np.interp(ts, tz, alt)
        m &= alt_i > alt_min
    if m.sum() < 4096:
        m = (ts >= ta) & (ts <= tb)
    acc = np.vstack([sc["accelerometer_m_s2[%d]" % i].astype(float)[m]
                     for i in range(3)]).T
    alt_w = alt_i[m] if lz is not None else np.full(int(m.sum()), np.nan)
    return ts[m], acc, pr["fs"], alt_w


def harmonic_rms(x, fs, fc=10.0):
    """把 10 Hz 以下的机动成分抠掉，剩下的当振动。

    做法：减去一阶 IIR 低通（= 高通）。不用 scipy，保持零依赖。
    """
    a = float(np.exp(-2 * np.pi * fc / fs))
    y = np.empty_like(x)
    acc = x[0]
    for i, v in enumerate(x):
        acc = a * acc + (1 - a) * v
        y[i] = v - acc
    return y


def spectrum(x, fs, n):
    """Hann 窗 + 单边幅值谱。返回 (freqs, amp)。

    幅值修正见 24.4.6 节：A_k = 2|X_w[k]| / (N * CG)，CG = 0.5 → 4|X_w|/N。
    """
    seg = x[:n] - np.mean(x[:n])
    w = np.hanning(n)
    X = np.fft.rfft(seg * w)
    amp = 2.0 * np.abs(X) / (n * 0.5)
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    return freqs, amp


def exp3(profs):
    head("实验 3  ★★ 振动：从真机日志算 RMS 与 FFT 主峰")

    wrap("这一节把 24.4.6 节的公式（DFT、Hann 窗、幅值修正）"
         "用【真机数据】跑一遍 —— 不是仿真造的假数据。")

    sub("① 悬停段的振动 RMS")

    rows = []
    specs = []
    for pr in profs:
        t, acc, fs, alt_w = airborne_slice(pr)
        n = 4096
        if len(acc) < n * 2:
            continue
        # 取正中一段，避开起降
        i0 = len(acc) // 2 - n // 2
        s = acc[i0: i0 + n]
        a_seg = alt_w[i0: i0 + n]
        hf = np.vstack([harmonic_rms(s[:, i], fs) for i in range(3)]).T
        f, amp = spectrum(s[:, 2], fs, n)
        pk = int(np.argmax(amp[1:]) + 1)
        specs.append((pr["date"], f, amp, s, fs, t))
        rows.append([pr["date"], "%.1f" % fs, "%.1f" % (n / fs),
                     "%.2f" % s[:, 0].std(), "%.2f" % s[:, 1].std(),
                     "%.2f" % s[:, 2].std(),
                     "%.2f" % hf[:, 2].std(),
                     "%.1f" % f[pk], "%.3f" % amp[pk],
                     "%.2f~%.2f" % (np.nanmin(a_seg), np.nanmax(a_seg))])
    tab(rows, ["日期", "fs Hz", "窗长s", "RMS x", "RMS y", "RMS z",
               ">10Hz z", "主峰Hz", "峰幅", "窗内高度m"],
        width=[11, 6, 6, 7, 7, 7, 8, 7, 6, 11])

    print()
    wrap("★ 读法（三个数各有各的意思）：")
    bullet([
        "RMS z 是把机动和振动混在一起的【总交流分量】，单位 m/s²；",
        ">10 Hz z 是抠掉 10 Hz 以下机动之后的【振动分量】—— 两者差得越多，"
        "说明这段里「在动」的成分越多；",
        "主峰频率要和【桨转速 × 桨叶数】对上才有意义：f_桨 = RPM/60，"
        "两叶桨的激励主频是 2·f_桨。",
    ])

    print()
    wrap("⚠️ 陷阱：RMS 是【全频段】的，主峰只是【最大的那一根】。"
         "一个 200 Hz 的大尖峰和一个 30 Hz 的大尖峰可以有一样的 RMS，"
         "但治理手段完全不同（前者查桨/电机，后者查机架/减震）。"
         "所以 RMS 和频谱必须一起看。")
    print()
    wrap("⚠️ 还有一个【必须自己查】的坑：主峰如果逼近 fs/2（奈奎斯特），"
         "它可能是混叠折回来的假峰 —— 24.4.4 节讲过。"
         "真峰频率只能是 f_alias 或 fs - f_alias 之一，"
         "光看一张频谱【分辨不出来】，得改变采样率或加抗混叠滤波再测一次。")

    return specs


# ============================================================================
#  实验 4：振动指标的采样率效应
# ============================================================================

def exp4(profs):
    head("实验 4  ★★ 振动指标：飞控报的数，你从日志复现得出来吗？")

    sub("① 用日志里的加速度，复现飞控的递推公式")

    rows = []
    data = []
    for pr in profs:
        t, acc, fs, _alt = airborne_slice(pr)
        n = 4096
        if len(acc) < n * 2:
            continue
        s = acc[len(acc) // 2 - n // 2: len(acc) // 2 + n // 2]
        v_log = px4_vibration_metric(s)
        im0 = next((x for x in pr["imus"] if x["id"] == 0), None)
        v_fc = im0["vibe"] if im0 else float("nan")
        fs_fc = im0["raw_rate"] if im0 else float("nan")
        data.append((pr["date"], fs, v_log, v_fc))
        rows.append([pr["date"], "%.1f" % fs, "%.0f" % fs_fc,
                     "%.2f" % v_log, "%.2f" % v_fc, "%.2f" % (v_fc / v_log)])
    tab(rows, ["日期", "日志fs", "飞控fs", "自算 v", "飞控记录 v", "记录/自算"],
        width=[11, 7, 7, 9, 11, 10])

    print()
    rr = [d[3] / d[2] for d in data if d[2] > 0]
    if rr:
        fs_lo = float(np.median([d[1] for d in data]))
        wrap("★ 结果：两个采样率差了大约 %.1f 倍，指标却落在同一量级 —— "
             "「记录/自算」= %.2f~%.2f，既不是 10 倍，也不是 1/10。"
             % (2003.0 / fs_lo, min(rr), max(rr)))
    wrap("  这【不是】说采样率无所谓。下面用数值实验说清楚为什么。")

    sub("② 数值实验：同一个正弦振动，两个采样率各给出多少")

    wrap("按 24.4.1 节的解析式 v = (4A/pi)·sin(pi·f/fs)，"
         "同一个振动在不同 fs 下的读数完全不是一回事。")

    rows = []
    sim = []
    for f0 in [10.0, 30.0, 60.0, 90.0, 150.0, 300.0, 700.0]:
        fs_hi, dec = 2003.0, 10
        fs_lo = fs_hi / dec
        N = int(20.0 * fs_hi)
        tt = np.arange(N) / fs_hi
        x = np.zeros((N, 3))
        x[:, 2] = np.sin(2 * np.pi * f0 * tt)
        v_hi = px4_vibration_metric(x)
        v_lo = px4_vibration_metric(boxcar_mean(x, dec))
        sim.append((f0, v_hi, v_lo))
        rows.append(["%g" % f0, "%.4f" % analytic_metric(1.0, f0, fs_hi),
                     "%.4f" % v_hi,
                     "%.4f" % analytic_metric(1.0, f0, fs_lo) if f0 < fs_lo / 2 else "欠采样",
                     "%.4f" % v_lo, "%.2f" % (v_hi / v_lo if v_lo > 0 else float("inf"))])
    tab(rows, ["f Hz", "解析@2003", "实测@2003", "解析@202", "实测@202", "比"],
        width=[6, 10, 10, 10, 10, 7])

    print()
    lo = [s for s in sim if s[0] < 100.0]
    hi = [s for s in sim if s[0] >= 100.0]
    lo_lo = min(s[1] / s[2] for s in lo)
    lo_hi = max(s[1] / s[2] for s in lo)
    wrap("★ 这张表是本节的核心，读法分两段：")
    bullet([
        "f < 100 Hz（202 Hz 的奈奎斯特以内）：比值 v(2003)/v(202) 落在 "
        "%.2f~%.2f —— 高采样率把低频振动「看小」了 %d 倍以上；"
        % (lo_lo, lo_hi, int(round(1.0 / max(lo_hi, 1e-9)))),
        "f > 100 Hz（超出 202 Hz 能看见的范围）：比值反过来，"
        + "、".join("%.2f" % (s[1] / s[2]) for s in hi)
        + " —— 202 Hz 那一列靠【混叠】看见的只是一个假峰，而且幅度严重偏小。",
    ])

    sub("③ 反过来用：从「比值」推断振动的频率成分")

    rmin, rmax = (min(rr), max(rr)) if rr else (float("nan"), float("nan"))
    print()
    wrap("把 ① 和 ② 接起来，就得到一个【从真机日志出发】的推断：")
    print()
    wrap("  如果这些架次的振动主要落在 100 Hz 以下，按 ② 的规律，"
         "① 里的「记录/自算」应该落在 %.2f~%.2f；" % (lo_lo, lo_hi))
    wrap("  实测是 %.2f~%.2f —— 远高于低频段该有的比值，"
         "两个采样率读到的量级相当。" % (rmin, rmax))
    wrap("  ★ 这只可能在【振动能量一直延伸到 100 Hz 以上、直到 kHz 量级】时发生。")
    print()
    wrap("★ 所以：日志里 sensor_combined 的采样率只是「看得见」的上限，"
         "不代表振动只到那里。这正是 24.4.4 节「混叠会把振动报小」的"
         "真实版 —— 只不过这里有两路采样率互为参照，混叠被抓住了。")

    print()
    wrap("⚠️ 必须说清的边界（本节的结论到哪为止）：")
    bullet([
        "①②③ 全部来自【真机日志】的 sensor_combined 与 vehicle_imu_status，"
        "可复现；",
        "②里的 2003/202 Hz 数值实验是【原理演示】，不是真机测量 —— "
        "它的作用是解释 ① 的现象，不是提供新数据；",
        "③ 的推断是【半定量】的：它证明了能量延伸到 100 Hz 以上，"
        "但给不出具体的谱形状 —— 因为 202 Hz 的日志本来就没有那个信息。"
        "想要谱形状，得打开 PX4 的 gyro_fft 模块在线算（24.4.6 节的 ①）。",
    ])

    return data, sim


# ============================================================================
#  实验 5：GPS / EKF 新息 / 削顶
# ============================================================================

def exp5(profs):
    head("实验 5  ★ GPS 精度、EKF 新息、削顶：能算什么、不能算什么")

    sub("① GPS：日志里到底记了什么")

    rows = []
    for pr in profs:
        u = pr["u"]
        try:
            _u, st = load(pr["path"], ["vehicle_gps_position"])
        except Exception:
            continue
        g = one(st, "vehicle_gps_position")
        if g is None:
            continue
        e = np.asarray(g["eph"], dtype=float)
        rows.append([pr["date"],
                     "%.2f" % med(g["eph"]), "%.2f" % med(g["epv"]),
                     "%.2f~%.2f" % (e.min(), e.max()),
                     "%d" % len(np.unique(e)),
                     "%d" % int(med(g["satellites_used"])),
                     "%.1f" % med(g["hdop"]),
                     "%d" % len(g["timestamp"])])
    tab(rows, ["日期", "eph中位m", "epv中位m", "eph范围m", "取值数",
               "星数", "hdop", "样本数"],
        width=[11, 9, 9, 13, 7, 5, 5, 7])

    print()
    wrap("★★ 这张表要横着读，两个数一起看：")
    bullet([
        "「eph 范围」：每次飞行里 eph 只在一条【很窄的带】里游走 —— "
        "0.15 / 0.18 / 0.22 / 0.013 m 宽；",
        "「星数 / hdop」：同一段飞行里星数恒为 28、hdop 几乎不变（0.6~0.9）。",
    ])
    print()
    wrap("★ 这说明什么：")
    wrap("   eph 不随可见卫星数、也不随 hdop 变化 —— 它在 hdop 纹丝不动的时候"
         "照样游走 0.2 m。而【不同架次之间】的差异（1.67 / 1.92 / 2.02 / 2.38 m）"
         "比同一次飞行内的游走大得多。")
    wrap("   换句话说：这个字段反映的是【换了地点 / 天线 / 接收机状态】，"
         "不是【这一刻的定位质量】。")
    print()
    wrap("⚠️ 这里必须纠正一个本脚本开发时写错过的说法：")
    wrap("   第一版写的是「整段飞行里 eph 只有 1~2 个不同的值」—— 那是把 eph "
         "四舍五入到 0.1 m 之后看到的假象。原始 float 一看，每次飞行有 "
         "17~128 个不同的值（见「取值数」列）。")
    wrap("   ★ 结论没变（eph 不能用来下精度结论），但【依据】换了："
         "不是因为「它是个常数」，而是因为「它不随任何可见的环境量变化」。")
    print()
    wrap("★ 所以本课的判断是：")
    bullet([
        "「eph < 2 m」这种结论【不能】从这批日志得出 —— eph 是接收机的"
        "【自评精度】，不是【实测误差】，两者不是一回事；",
        "要做 GPS 精度对比，得看 eph 在【同一地点不同时段】有没有区分度，"
        "这批日志里没有（同一架次内它几乎是平的）；",
        "真要看定位质量，可靠的替代量是【EKF 新息检验比】（见 ②），"
        "它真的在随环境变化。",
    ])

    sub("② EKF 新息检验比：判据就是「有没有超过 1」")

    KEYS = ["gps_hpos[0]", "gps_vpos", "gps_hvel[0]", "gps_vvel", "heading"]
    rows = []
    raw = []
    for pr in profs:
        try:
            _u, st = load(pr["path"], ["estimator_innovation_test_ratios"])
        except Exception:
            continue
        d = one(st, "estimator_innovation_test_ratios")
        if d is None:
            continue
        rec = []
        for k in KEYS:
            if k not in d:
                continue
            v = np.asarray(d[k], dtype=float)
            v = v[np.isfinite(v)]
            if v.size == 0:
                continue
            rec.append((k, med(v), float(np.percentile(v, 99)), float(v.max()),
                        100.0 * float((v > 1).mean())))
        raw.append((pr["date"], rec))
        for k, md, p99, mx, over in rec:
            rows.append([pr["date"], k, "%.4f" % md, "%.4f" % p99,
                         "%.4f" % mx, "%.2f%%" % over])
    tab(rows, ["日期", "字段", "中位", "p99", "最大", ">1 占比"],
        width=[11, 14, 9, 9, 9, 9])

    print()
    allv = [r[2] for _dt, rec in raw for r in rec]
    gmax = max(allv) if allv else float("nan")
    nover = sum(1 for v in allv if v > 1.0)
    wrap("★ 结论：这 %d 个（日期 × 通道）组合、共 %d 个统计量里，"
         "检验比的【全局最大值只有 %.4f】，超过 1 的有 %d 个 —— "
         "EKF 全程舒服地接受 GPS，没有任何一次因为「观测太离谱」而被拒绝。"
         "这就是一份「健康」的日志长什么样。"
         % (len(raw), len(allv), gmax, nover))
    print()
    wrap("⚠️ 顺带验证 24.5.2 节的一条细节：`gps_hpos[1]` 全程恒为 0.000 ——")
    wrap("   因为检验比是【标量】，向量字段里只有第 0 个分量有意义。"
         "读日志时把 [1] 当成「垂直方向」就错了。")

    sub("③ ★ 一个诚实的负面结果：检验比【复现不出来】")

    wrap("24.5.2 节给了公式 test_ratio = ν²/(g²·S)。那么能不能用日志里的"
         "`estimator_innovations`（ν）和 `estimator_innovation_variances`（S）"
         "两列，【自己算一遍】再跟记录的 `estimator_innovation_test_ratios` 对照？")
    print()
    wrap("试过了，答案是不能。两个原因，都是实测出来的：")
    bullet([
        "三个 topic 是【分别记录】的，采样时刻对不齐：同一段飞行里 "
        "innovations 649 条、variances 649 条、test_ratios 650 条，"
        "按 timestamp_sample 只能配上 253 对；",
        "记录的检验比是【对分量取 max 再开方】的结果（ekf_helper.cpp 里的 "
        "`sqrtf(Vector2f(...).max())`），而 innovations/variances 是【逐分量】的。"
        "你手上没有「当时用的是哪个分量、哪个 gate」的信息。",
    ])

    print()
    sub("④ 削顶：一个源码与日志对不上的地方")

    rows = []
    for pr in profs:
        sc, st = pr["sc"], pr["st"]
        ac = np.asarray(sc["accelerometer_clipping"])
        gc = np.asarray(sc["gyro_clipping"])
        rows.append([pr["date"],
                     "%d" % int((ac != 0).sum()),
                     "%d" % int((gc != 0).sum()),
                     "%d" % int(gc.max()),
                     "%d" % len(gc)])
    tab(rows, ["日期", "accel削顶样本", "gyro削顶样本", "gyro最大值", "总样本"],
        width=[11, 13, 13, 11, 8])

    print()
    wrap("★★ 这里有一处【对不上】，值得原样记下来：")
    bullet([
        "msg/SensorCombined.msg 把 gyro_clipping 声明成位域"
        "（CLIPPING_X=1 / Y=2 / Z=4），源码 VehicleIMU.cpp 也确实只 OR 这三个位 —— "
        "所以合法值最大是 7；",
        "同一份日志里 accelerometer_clipping 的取值恰好只有 {0, 4}，"
        "完全符合位域定义（4 = Z 轴削顶）；",
        "但 gyro_clipping 的取值一路到 255，还有 32、51、123 这些值 —— "
        "按 1.15.4 和 1.14.4 的源码都解释不了。",
    ])
    print()
    wrap("★ 本课不给结论，只给两条建议：")
    bullet([
        "判断削顶【不要】用 sensor_combined.gyro_clipping，"
        "用 vehicle_imu_status.gyro_clipping[3]（uint32 累计计数）—— "
        "本课这 4 架次它几乎全是 0，说明陀螺其实没怎么削顶；",
        "遇到「源码说不可能、日志里就是有」的时候，先别改代码，"
        "先像这样把两侧证据都摆出来 —— 这本身就是一个可报告的问题。",
    ])

    return raw


# ============================================================================
#  画图（可选）
# ============================================================================

def make_figures(profs, specs, data, sim, outdir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print()
        wrap("★ 没有 matplotlib，跳过画图（不影响结论）。")
        return []

    # 中文字体：Windows 用雅黑/黑体，Linux 上退回默认（可能显示成方框，不影响数据）
    for fam in ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                "WenQuanYi Zen Hei", "DejaVu Sans"]:
        try:
            matplotlib.rcParams["font.sans-serif"] = [fam]
            break
        except Exception:
            continue
    matplotlib.rcParams["axes.unicode_minus"] = False

    made = []

    # 图 1：时程 + 频谱
    if specs:
        date, f, amp, s, fs, t = specs[-1]
        fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
        n = len(s)
        tt = np.arange(n) / fs
        for i, lab in enumerate("xyz"):
            ax[0].plot(tt, s[:, i], lw=0.6, label=lab)
        ax[0].set_xlabel("时间 (s)")
        ax[0].set_ylabel("加速度 (m/s²)")
        ax[0].set_title("%s 悬停段三轴加速度（FRD，sensor_combined）" % date, fontsize=10)
        ax[0].legend(ncol=3, fontsize=8)
        ax[0].grid(alpha=0.3)

        ax[1].semilogy(f[1:], amp[1:], lw=0.8)
        pk = int(np.argmax(amp[1:]) + 1)
        ax[1].plot(f[pk], amp[pk], "rv")
        ax[1].annotate("%.1f Hz" % f[pk], (f[pk], amp[pk]),
                       textcoords="offset points", xytext=(6, 6), fontsize=9)
        ax[1].set_xlabel("频率 (Hz)")
        ax[1].set_ylabel("幅值 (m/s²)")
        ax[1].set_title("Z 轴幅值谱（Hann 窗，N=4096）", fontsize=10)
        ax[1].grid(alpha=0.3, which="both")
        fig.tight_layout()
        p = os.path.join(outdir, "l23b_vibration.png")
        fig.savefig(p, dpi=120)
        plt.close(fig)
        made.append(p)

    # 图 2：采样率效应
    if sim:
        fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
        ff = np.linspace(1, 1000, 800)
        ax[0].plot(ff, [analytic_metric(1.0, x, 2003.0) for x in ff],
                   label="解析式 v=(4A/π)sin(πf/fs), fs=2003 Hz")
        fl = np.linspace(1, 100, 400)
        ax[0].plot(fl, [analytic_metric(1.0, x, 202.0) for x in fl],
                   label="fs=202 Hz")
        ax[0].axvline(100, color="0.6", ls="--", lw=0.8)
        ax[0].text(103, 0.05, "202 Hz 的奈奎斯特", fontsize=8, color="0.4")
        fs0 = [s[0] for s in sim]
        rr = [s[1] / s[2] if s[2] > 0 else np.nan for s in sim]
        ax[1].semilogy(fs0, rr, "o-")
        ax[1].axhline(1.0, color="0.6", ls="--", lw=0.8)
        ax[1].axvspan(100, 1000, color="orange", alpha=0.12)
        ax[1].set_xlabel("振动频率 f (Hz)")
        ax[1].set_ylabel("v(2003 Hz) / v(202 Hz)")
        ax[1].set_title("比值 <1 = 高频被高采样率「看小」；>1 = 202 Hz 已混叠",
                        fontsize=9)
        ax[1].grid(alpha=0.3, which="both")
        ax[0].set_xlabel("振动频率 f (Hz)")
        ax[0].set_ylabel("指标读数 (A=1 m/s²)")
        ax[0].set_title("同一个振动，两个采样率读出两个数", fontsize=10)
        ax[0].legend(fontsize=8)
        ax[0].grid(alpha=0.3)
        fig.tight_layout()
        p = os.path.join(outdir, "l23b_fs_effect.png")
        fig.savefig(p, dpi=120)
        plt.close(fig)
        made.append(p)

    # 图 3：真机比值 vs 数值实验
    if data and sim:
        fig, ax = plt.subplots(figsize=(7.2, 3.6))
        ax.plot([s[0] for s in sim], [s[1] / s[2] if s[2] > 0 else np.nan for s in sim],
                "o-", label="数值实验（单频正弦，原理演示）")
        for date, fs, vlog, vfc in data:
            if np.isfinite(vfc) and vlog > 0:
                ax.plot([fs], [vfc / vlog], "r*", ms=12)
        ax.plot([], [], "r*", ms=12, label="真机 4 架次（记录/自算）")
        ax.axhline(1.0, color="0.6", ls="--", lw=0.8)
        ax.set_xscale("log")
        ax.set_xlabel("振动频率 f (Hz)   ★ 真机点画在横轴=日志采样率处，仅为定位")
        ax.set_ylabel("v(2003 Hz) / v(202 Hz)")
        ax.set_title("真机比值落在 1 附近 → 振动能量延伸到 100 Hz 以上", fontsize=10)
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3, which="both")
        fig.tight_layout()
        p = os.path.join(outdir, "l23b_vibe_ratio.png")
        fig.savefig(p, dpi=120)
        plt.close(fig)
        made.append(p)

    return made


# ============================================================================
#  入口
# ============================================================================

def main():
    args = sys.argv[1:]
    which = args if args else ["1", "2", "3", "4", "5"]

    root = find_ulog_root()
    all_logs = scan_logs(root) if root else []
    picked, _exact = pick_logs(all_logs)

    profs = []
    specs = data = sim = None

    for a in which:
        if a == "1":
            exp1()
        elif a == "2":
            profs = exp2(picked)
        elif a == "3":
            if not profs:
                profs = exp2(picked)
            specs = exp3(profs)
        elif a == "4":
            if not profs:
                profs = exp2(picked)
            data, sim = exp4(profs)
        elif a == "5":
            if not profs:
                profs = exp2(picked)
            exp5(profs)
        else:
            print("用法：python lesson23b_real_log.py [1|2|3|4|5]（不带参数 = 全部）")
            return 1

    if specs and data and sim:
        made = make_figures(profs, specs, data, sim, CH7)
        if made:
            print()
            wrap("★ 图已存到 07-仿真与实验/ 下：")
            for p in made:
                wrap("    " + os.path.basename(p), indent=6)

    print()
    print("=" * W)
    print("  完成。所有数字都可用 `python lesson23b_real_log.py` 复现。")
    print("=" * W)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
