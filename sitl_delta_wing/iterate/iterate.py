#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
iterate.py —— 控制律迭代闭环（A：自动化 / B：标准科目与指标）

一条命令完成：增量编译 → 无头跑 SITL → 驱动标准科目 → 记录遥测 → 算指标 → 存档对比

用法（在 WSL 里，以 apuser 身份）：
    python3 iterate.py --ardupilot ~/ardupilot --maneuver roll_step --tag baseline
    python3 iterate.py --maneuver level --tag v2 --compare baseline

设计要点
--------
* **不做 .BIN 日志解析**：直接流式抓 MAVLink 存 CSV。SITL 在未进入飞行时
  根本不落盘 .BIN，而且解析 .BIN 要额外的库和字段映射 —— 遥测够用且更直接。
* **科目可重复**：每个科目用固定的时间轴和指令序列，跑两次结果应一致。
* **指标是给"对比"用的**：单看一条曲线说明不了好坏，所以要能跟历史存档比。

⚠️ 前置（都是实测踩出来的，见同目录 README）：
    1. WSL 必须镜像网络（否则 pip/git 全超时）
    2. SITL 用 --defaults 加载参数，其中必须有 ARMING_SKIPCHK -1
    3. 弹射靠 SERVO7_FUNCTION 非零 + MAV_CMD_DO_SET_SERVO(7, 2000)
    4. 模型 JSON 用相对路径
"""

import os
import sys
import csv
import json
import math
import time
import argparse
import subprocess
from datetime import datetime

# ═════════════════════════════════════════════════════════════════════
#  配置
# ═════════════════════════════════════════════════════════════════════

HOME = "-35.363261,149.165230,584,353"
HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")

# 与科目无关的 SITL 基础参数
BASE_DEFAULTS = """
SERVO1_FUNCTION 77
SERVO2_FUNCTION 78
SERVO3_FUNCTION 70
SERVO7_FUNCTION 28
SERVO1_REVERSED 0
SERVO2_REVERSED 0
ARMING_SKIPCHK -1
LOG_BITMASK 65535
"""

# ── 科目定义 ──────────────────────────────────────────────────────────
#   t_arm / t_launch : 相对科目起点的秒数
#   steps            : [(t, 模式名 or None, 泛用通道值 or None)]
MANEUVERS = {
    "level": {
        "desc": "弹射后转 FBWA 平飞 —— 检查能否建立稳定基线",
        "duration": 60,
        "steps": [(0, None, None)],
    },
    "manual": {
        # ★★★ 决定性的对照实验：
        #     不加任何增稳，纯手动。若照样翻滚 ⟹ 气动模型本身不稳；
        #     若能大致平飞 ⟹ 问题在控制律，模型没问题。
        #     这是 25 课「原则三：对照实验」的直接应用。
        "desc": "MANUAL 模式飞行 —— 隔离「模型不稳」与「控制律不好」",
        "duration": 50,
        "steps": [(15, "MANUAL", None)],
    },
    "roll_step": {
        "desc": "滚转阶跃：FBWA 下副翼摇杆 +40% 保持 8 秒",
        "duration": 55,
        "steps": [(15, "FBWA", None), (20, None, ("roll", 40, 8))],
    },
    "pitch_step": {
        "desc": "俯仰阶跃：FBWA 下俯仰摇杆 +25% 保持 6 秒",
        "duration": 55,
        "steps": [(15, "FBWA", None), (20, None, ("pitch", 25, 6))],
    },
}

# 判定"是否在飞"的门限（用于稳定性闸门）
FLY_ALT_M = 8.0        # 相对起飞点的高度
FLY_SPD_MS = 8.0       # 地速


# ═════════════════════════════════════════════════════════════════════
#  构建
# ═════════════════════════════════════════════════════════════════════

def do_build(ap, quiet=False):
    """增量编译。返回 (成功, 耗时秒, 编译目标数)。"""
    t0 = time.time()
    r = subprocess.run(["./waf", "plane"], cwd=ap,
                       capture_output=True, text=True, timeout=3600)
    dt = time.time() - t0
    n = sum(1 for ln in r.stdout.splitlines() if ln.startswith("["))
    ok = (r.returncode == 0)
    if not quiet:
        status = "✓" if ok else "✗"
        print("  %s 编译 %d 个目标，耗时 %.1f s" % (status, n, dt))
        if not ok:
            print(r.stdout[-1500:]); print(r.stderr[-1500:])
    return ok, dt, n


# ═════════════════════════════════════════════════════════════════════
#  SITL 操控
# ═════════════════════════════════════════════════════════════════════

class Sitl:
    def __init__(self, ap, model_json_rel):
        self.ap = ap
        self.model = model_json_rel
        self.proc = None
        self.m = None
        self.rows = []

    def start(self, defaults_path, wait_ekf=25, json_over=None):
        import shutil
        src = os.path.join(HERE, "..", "delta_wing.json")
        dst = os.path.join(self.ap, "delta_wing.json")
        if json_over:
            # 覆盖气动模型里的字段（CGOffset / c_m_a 等）
            model = json.load(open(src, encoding="utf-8"))
            for k, v in json_over.items():
                model[k] = v
            with open(dst, "w", encoding="utf-8") as f:
                json.dump(model, f, indent=4, ensure_ascii=False)
        else:
            shutil.copy(src, dst)
        log = open("/tmp/iterate-sitl.log", "w")
        self.proc = subprocess.Popen(
            ["./build/sitl/bin/arduplane",
             "--model", "plane-elevon-catapult:" + self.model,
             "--defaults", defaults_path,
             "--home", HOME, "-w"],
            cwd=self.ap, stdout=log, stderr=subprocess.STDOUT)
        time.sleep(3)
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=60)
        self.m.wait_heartbeat(timeout=60)
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)
        self._drain(wait_ekf)
        return True

    def stop(self):
        if self.proc:
            self.proc.kill()
        subprocess.run(["pkill", "-f", "arduplane"],
                       capture_output=True)
        time.sleep(1)

    def _drain(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            self.m.recv_match(blocking=True, timeout=0.5)

    def set_mode(self, name):
        from pymavlink import mavutil
        mid = self.m.mode_mapping()[name]
        self.m.mav.set_mode_send(
            self.m.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid)

    def arm(self):
        from pymavlink import mavutil
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)

    def launch(self):
        from pymavlink import mavutil
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 7, 2000, 0, 0, 0, 0, 0)

    def wait_launch(self, timeout=15):
        """
        等弹射【真正触发】的那一刻，返回 True。

        ⚠️ 这是降噪的关键：MANEUVERS 里的时间轴原来以"墙钟"为基准，
           而"发送弹射指令"到"舵机真的跳变"之间有不确定的延迟
           （MAVLink 往返 + 飞控主循环节拍）。
           以墙钟为 t=0，每次的科目起点都漂几帧 —— 累积起来就是可见的差异。

           改用【事件对齐】：以 SERVO_OUTPUT_RAW.servo7_raw > 1700 的
           上升沿为 t=0。这与 SITL 物理里 launch_triggered 的判据同源
           （SIM_Plane.cpp:403 `input.servos[6] > 1700`）。
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="SERVO_OUTPUT_RAW",
                                    blocking=True, timeout=0.5)
            if msg and msg.servo7_raw > 1700:
                return True
        return False

    def wait_ekf(self, flags_mask=0x01, timeout=60):
        """等 EKF 姿态就绪（不用固定等待时间，消掉收敛程度的漂移）。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="EKF_STATUS_REPORT",
                                    blocking=True, timeout=1)
            if msg and (msg.flags & flags_mask):
                return True
        return False

    # 巡航油门。⚠️ 必须持续发，否则 FBWA/MANUAL 下油门跟 RC 走 = 0，
    #    飞机会在弹射速度耗尽后减速失速 —— 这正是最初"翻滚"的真实原因。
    CRUISE_THR_PCT = 70

    def rc_override(self, roll=0, pitch=0, throttle=None):
        from pymavlink import mavutil
        if throttle is None:
            throttle = self.CRUISE_THR_PCT
        r = [0] * 8
        r[0] = 1500 + int(roll * 5)       # 副翼：±100% → 1000~2000
        r[1] = 1500 + int(pitch * 5)      # 升降
        r[2] = 1000 + int(throttle * 10)  # 油门：0~100% → 1000~2000
        self.m.mav.rc_channels_override_send(
            self.m.target_system, self.m.target_component, *r)

    def sample(self, t):
        """收一条遥测，返回行字典或 None。"""
        types = ["VFR_HUD", "ATTITUDE", "NAV_CONTROLLER_OUTPUT",
                 "SERVO_OUTPUT_RAW"]
        msg = self.m.recv_match(type=types, blocking=True, timeout=0.3)
        if msg is None:
            return None
        g = msg.get_type()
        self.cur = getattr(self, "cur", {})
        if g == "VFR_HUD":
            self.cur.update(alt=msg.alt, spd=msg.groundspeed,
                            thr=msg.throttle)
        elif g == "ATTITUDE":
            # ★★★ 角速度是【调参的指南针】——必须采！
            #     只采角度的话，"翻滚"和"快要翻滚"看起来差不多，
            #     指标没有梯度，就无从判断一次改动是变好还是变坏。
            #     翻滚时 rate ~ 几百 deg/s，平飞时 ~ 几十 deg/s。
            self.cur.update(roll=math.degrees(msg.roll),
                            pitch=math.degrees(msg.pitch),
                            yaw=math.degrees(msg.yaw),
                            p=math.degrees(msg.rollspeed),
                            q=math.degrees(msg.pitchspeed),
                            r=math.degrees(msg.yawspeed))
        elif g == "NAV_CONTROLLER_OUTPUT":
            self.cur.update(nav_roll=msg.nav_roll, nav_pitch=msg.nav_pitch,
                            aspd_error=msg.aspd_error,
                            alt_error=msg.alt_error)
        elif g == "SERVO_OUTPUT_RAW":
            self.cur.update(s1=msg.servo1_raw, s2=msg.servo2_raw,
                            s3=msg.servo3_raw)
        if all(k in self.cur for k in ("alt", "spd", "roll", "pitch", "thr")):
            row = dict(self.cur)
            row["t"] = t
            self.rows.append(row)
            return row
        return None


# ═════════════════════════════════════════════════════════════════════
#  跑一个科目
# ═════════════════════════════════════════════════════════════════════

def run_maneuver(ap, name, defaults_path, verbose=True, json_over=None):
    spec = MANEUVERS[name]
    if verbose:
        print("  科目 %s：%s（%d 秒）" % (name, spec["desc"], spec["duration"]))

    s = Sitl(ap, "delta_wing.json")
    try:
        # ── 降噪：用【事件】而不是固定等待时间 ──────────────────────
        # 固定 drain(25) 的问题：EKF 每次收敛程度不同，而且"25 秒"到
        # "EKF 真的就绪"之间的余量每次都不一样 → 科目起点漂移。
        s.start(defaults_path, wait_ekf=0, json_over=json_over)
        s._drain(5)
        if not s.wait_ekf():
            if verbose:
                print("     ⚠️ EKF 未在超时内就绪，继续（结果可能不可比）")
        from pymavlink import mavutil
        s.set_mode("TAKEOFF"); s._drain(2)
        s.arm(); s._drain(3)

        s.launch()
        # t=0 对齐到【弹射真的触发】那一刻，而不是"发出指令"的时刻
        if not s.wait_launch():
            if verbose:
                print("     ✗ 弹射未触发 —— 本次结果无效")
            return []
        t0 = time.time()

        # ── RC 状态：必须【持续发送】 ────────────────────────────────
        #  ⚠️⚠️ 两个坑，都是实测踩出来的：
        #   ① RC_CHANNELS_OVERRIDE 有超时（几秒），发一次就失效 →
        #      必须每个循环都发。
        #   ② 【最关键】TAKEOFF 模式自动控制油门；一旦切到 FBWA/MANUAL，
        #      油门就跟着 RC 输入走。如果从不发油门通道，它就是 1000（0%）
        #      → 飞机在弹射速度耗尽后减速、失速、翻滚。
        #      【这正是最初判定"气动模型有问题"的真实原因 —— 其实是这个 bug。】
        rc_state = {"roll": 0, "pitch": 0,
                    "throttle": Sitl.CRUISE_THR_PCT}
        step_reset = {}

        done = set()
        while time.time() - t0 < spec["duration"]:
            el = time.time() - t0
            for i, (at, mode, rc) in enumerate(spec["steps"]):
                if i in done or el < at:
                    continue
                if mode:
                    s.set_mode(mode)
                    if verbose:
                        print("     t=%.0fs 切 %s" % (el, mode))
                if rc:
                    which, mag, dur = rc
                    rc_state[which] = mag
                    step_reset[i] = el + dur
                    if verbose:
                        print("     t=%.0fs %s 阶跃 %+d%% 保持 %ds"
                              % (el, which, mag, dur))
                done.add(i)

            for i, t_reset in list(step_reset.items()):
                if el > t_reset:
                    rc_state[spec["steps"][i][2][0]] = 0
                    del step_reset[i]

            s.rc_override(**rc_state)
            s.sample(el)
    finally:
        s.stop()

    return s.rows


# ═════════════════════════════════════════════════════════════════════
#  指标
# ═════════════════════════════════════════════════════════════════════

def compute_metrics(rows):
    if not rows:
        return {"ok": False, "reason": "无遥测数据"}

    base_alt = rows[0]["alt"]
    for r in rows:
        r["alt_rel"] = r["alt"] - base_alt

    # ── ★★★ 核心指标：姿态角速度 RMS ──────────────────────────────
    #  这是【调参的指南针】，与"飞不飞得起来"无关，永远可算，且有明确梯度：
    #      翻滚          → 几百 ~ 上千 deg/s
    #      勉强飞但振荡   → 一百左右
    #      平稳飞行      → 几十
    #  没有它，一次改动"变好还是变坏"就无从判断 —— 这是最初卡住的真正原因。
    rates = [math.sqrt(r.get("p", 0) ** 2 + r.get("q", 0) ** 2
                       + r.get("r", 0) ** 2)
             for r in rows if "p" in r]
    rate_rms = _rms(rates) if rates else None

    # ① 稳定性闸门：有没有进入持续飞行
    flying = [r for r in rows
              if r["alt_rel"] > FLY_ALT_M and r["spd"] > FLY_SPD_MS]
    frac_flying = len(flying) / len(rows)

    m = {
        "ok": True,
        "samples": len(rows),
        "duration_s": round(rows[-1]["t"], 1),
        "max_alt_m": round(max(r["alt_rel"] for r in rows), 2),
        "max_spd": round(max(r["spd"] for r in rows), 2),
        "frac_flying": round(frac_flying, 3),
        "flew": frac_flying > 0.5,
        # ★ 调参指南针：永远可算、有梯度（翻滚几百~上千，平飞几十）
        "rate_rms": round(rate_rms, 1) if rate_rms is not None else None,
        "rate_max": round(max(rates), 1) if rates else None,
    }

    if not m["flew"]:
        m["reason"] = "未进入稳定飞行（高度>%.0fm 且地速>%.0fm/s 的样本占比 %.0f%%）" \
                      % (FLY_ALT_M, FLY_SPD_MS, 100 * frac_flying)
        return m

    # ② 姿态稳定性（飞行段）
    m["roll_std_deg"] = round(_std([r["roll"] for r in flying]), 2)
    m["pitch_std_deg"] = round(_std([r["pitch"] for r in flying]), 2)

    # ③ 舵量饱和占比
    sat = sum(1 for r in flying
              if abs(r.get("s1", 1500) - 1500) > 450
              or abs(r.get("s2", 1500) - 1500) > 450)
    m["servo_sat_frac"] = round(sat / len(flying), 3)

    # ④ 跟随误差（导航层目标 vs 实际）
    if all("nav_roll" in r for r in flying):
        err = [abs(r["roll"] - r["nav_roll"]) for r in flying]
        m["roll_track_rmse_deg"] = round(_rms(err), 2)
        m["roll_track_max_deg"] = round(max(err), 2)

    return m


def _std(xs):
    if len(xs) < 2:
        return 0.0
    mu = sum(xs) / len(xs)
    return math.sqrt(sum((x - mu) ** 2 for x in xs) / (len(xs) - 1))


def _rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0


# ═════════════════════════════════════════════════════════════════════
#  存档
# ═════════════════════════════════════════════════════════════════════

def aggregate(runs, per_run_metrics):
    """
    多次重复 → 统计量。

    ⚠️⚠️ 这不是锦上添花，是【必需】的：
       实测同一配置连跑 3 次，max_alt_m 跨度 5.05 m（21%）、
       max_spd 跨度 7.92 m/s（32%）。
       【低于这个跨度的差异，测的是噪声，不是改动。】

    所以对外报【中位数 + 跨度】，不报单次值。
    """
    keys = ["max_alt_m", "max_spd", "frac_flying",
            "rate_rms", "rate_max",
            "roll_std_deg", "pitch_std_deg", "servo_sat_frac"]
    agg = {"n_runs": len(runs),
           "flew_any": any(m.get("flew") for m in per_run_metrics)}
    for k in keys:
        vs = [m[k] for m in per_run_metrics
              if k in m and isinstance(m[k], (int, float))]
        if not vs:
            continue
        sv = sorted(vs)
        agg[k + "_median"] = sv[len(sv) // 2]
        agg[k + "_spread"] = round(max(vs) - min(vs), 3)
    # 噪声水平以【调参用的那个指标】为准（rate_rms），不是 max_alt_m
    base = agg.get("rate_rms_median")
    if base:
        agg["noise_pct"] = round(100.0 * agg["rate_rms_spread"] / base, 1)
        agg["noise_metric"] = "rate_rms"
    elif agg.get("max_alt_m_median"):
        agg["noise_pct"] = round(
            100.0 * agg["max_alt_m_spread"] / agg["max_alt_m_median"], 1)
        agg["noise_metric"] = "max_alt_m"
    return agg


def save(tag, maneuver, runs, metrics, build_info, per_run=None):
    """runs: [[rows], ...]（可多次运行）"""
    os.makedirs(RESULTS, exist_ok=True)
    stem = os.path.join(RESULTS, "%s_%s" % (maneuver, tag))

    with open(stem + ".csv", "w", newline="", encoding="utf-8") as f:
        allrows = []
        for i, rows in enumerate(runs):
            for r in rows:
                rr = dict(r); rr["run"] = i
                allrows.append(rr)
        if allrows:
            # ⚠️ 不能用 rows[0].keys()：遥测是逐步收齐的，
            #    第一行只有部分字段。必须取【所有行的 key 并集】。
            keys = sorted({k for r in allrows for k in r})
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader(); w.writerows(allrows)

    meta = {"tag": tag, "maneuver": maneuver,
            "time": datetime.now().isoformat(timespec="seconds"),
            "build": build_info, "metrics": metrics, "per_run": per_run}
    with open(stem + ".json", "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    return stem


def load(tag, maneuver):
    p = os.path.join(RESULTS, "%s_%s.json" % (maneuver, tag))
    if not os.path.isfile(p):
        return None
    return json.load(open(p, encoding="utf-8"))


def report(m, ref=None):
    print()
    print("  ── 指标 ──")
    if m.get("n_runs", 1) > 1:
        print("     （%d 次重复，报【中位数】与【跨度】）" % m["n_runs"])
        # ⚠️ rate_rms 必须排在第一位 —— 它是调参的指南针
        order = ["rate_rms", "rate_max", "frac_flying",
                 "max_alt_m", "max_spd",
                 "roll_std_deg", "pitch_std_deg", "servo_sat_frac"]
        for k in order:
            med, spr = m.get(k + "_median"), m.get(k + "_spread")
            if med is None:
                continue
            print("     %-20s 中位 %-10s 跨度 %s" % (k, med, spr))
        if "noise_pct" in m:
            print()
            print("     ⚠️ 噪声水平：%s 相对跨度 %.1f%%"
                  % (m.get("noise_metric", "?"), m["noise_pct"]))
            print("        【任何小于该跨度的差异都不能当作改动效果。】")
        return

    order = ["rate_rms", "rate_max", "samples", "duration_s", "flew",
             "frac_flying", "max_alt_m", "max_spd",
             "roll_std_deg", "pitch_std_deg",
             "servo_sat_frac", "roll_track_rmse_deg", "roll_track_max_deg"]
    for k in order:
        if k not in m:
            continue
        v = m[k]
        line = "     %-24s %s" % (k, v)
        if ref and k in ref.get("metrics", {}):
            rv = ref["metrics"][k]
            if isinstance(v, (int, float)) and isinstance(rv, (int, float)):
                d = v - rv
                line += "   (基线 %s, 差 %+.3f)" % (rv, d)
        print(line)
    if not m.get("flew"):
        print("     ⚠️ %s" % m.get("reason", ""))
    print()
    print("     ⚠️ 单次运行【不能用于对比】—— 实测同配置的跨度为 21%~32%。")
    print("        先跑 --repeat 3 测出噪声，再做对比。")


# ═════════════════════════════════════════════════════════════════════
#  主流程
# ═════════════════════════════════════════════════════════════════════

def main():
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--ardupilot", default=os.path.expanduser("~/ardupilot"))
    ap_.add_argument("--maneuver", default="level", choices=list(MANEUVERS))
    ap_.add_argument("--tag", default=None, help="存档名，默认自动生成")
    ap_.add_argument("--compare", default=None, help="与哪个 tag 对比")
    ap_.add_argument("--no-build", action="store_true")
    ap_.add_argument("--repeat", type=int, default=1,
                     help="重复次数。⚠️ 对比前必须先用 3 次测噪声，否则"
                          "分不清是改动效果还是随机波动")
    ap_.add_argument("--param", action="append", metavar="NAME=VALUE",
                     help="覆盖一个 ArduPilot 参数，可重复。"
                          "⚠️ 一次只改一个，否则结果好了也不知道为什么")
    ap_.add_argument("--json", action="append", metavar="KEY=VALUE",
                     help="覆盖气动模型 JSON 里的字段（如 CGOffset / c_m_a）。"
                          "CGOffset 是数组，写成 \"[x,y,z]\"")
    a = ap_.parse_args()

    json_over = None
    if a.json:
        json_over = {}
        for kv in a.json:
            if "=" not in kv:
                print("  ✗ --json 格式应为 KEY=VALUE：%s" % kv)
                return 1
            k, v = kv.split("=", 1)
            try:
                json_over[k] = json.loads(v)
            except ValueError:
                json_over[k] = float(v)
        print("\n  模型覆盖：")
        for k, v in json_over.items():
            print("     %-14s = %s" % (k, v))

    tag = a.tag or datetime.now().strftime("%m%d-%H%M")
    print("═" * 70)
    print("  控制律迭代闭环")
    print("  ArduPilot : %s" % a.ardupilot)
    print("  科目      : %s — %s" % (a.maneuver, MANEUVERS[a.maneuver]["desc"]))
    print("  标签      : %s" % tag)
    print("═" * 70)

    build_info = {}
    if not a.no_build:
        print("\n【1/3】编译")
        ok, dt, n = do_build(a.ardupilot)
        build_info = {"sec": round(dt, 1), "targets": n}
        if not ok:
            return 1
    else:
        print("\n【1/3】编译 — 已跳过")

    # defaults 文件（BASE_DEFAULTS + 命令行覆盖）
    defaults_path = os.path.join(a.ardupilot, "iterate_defaults.parm")
    txt = BASE_DEFAULTS
    if a.param:
        txt += "\n# ── 命令行覆盖 ──\n"
        for kv in a.param:
            if "=" not in kv:
                print("  ✗ --param 格式应为 NAME=VALUE：%s" % kv)
                return 1
            txt += kv.replace("=", " ") + "\n"
        print("\n  参数覆盖：")
        for kv in a.param:
            print("     %s" % kv.replace("=", " = "))
    with open(defaults_path, "w", encoding="utf-8") as f:
        f.write(txt)

    print("\n【2/3】跑科目 %s × %d 次" % (a.maneuver, a.repeat))
    all_runs, per_run = [], []
    for i in range(a.repeat):
        if a.repeat > 1:
            print("\n  ── 第 %d/%d 次 ──" % (i + 1, a.repeat))
        rows = run_maneuver(a.ardupilot, a.maneuver, defaults_path,
                            verbose=(a.repeat == 1), json_over=json_over)
        all_runs.append(rows)
        per_run.append(compute_metrics(rows))

    print("\n【3/3】指标")
    m = aggregate(all_runs, per_run) if a.repeat > 1 else per_run[0]
    ref = load(a.compare, a.maneuver) if a.compare else None
    if a.compare and ref is None:
        print("     ⚠️ 找不到基线 %s，只报本次结果" % a.compare)
    report(m, ref)

    stem = save(tag, a.maneuver, all_runs, m, build_info,
                per_run=[{k: v for k, v in pm.items()
                          if k in ("flew", "max_alt_m", "max_spd",
                                   "frac_flying")}
                         for pm in per_run])
    print()
    print("  存档: %s.csv / .json" % stem)
    print("═" * 70)
    return 0 if m.get("flew_any", m.get("flew")) or a.maneuver != "level" else 2


if __name__ == "__main__":
    sys.exit(main())
