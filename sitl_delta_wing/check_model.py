#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_model.py —— 校验 SITL 模型文件与 ArduPilot 源码是否对得上

在【没有 Linux / 跑不了 SITL】的机器上，这是唯一能做的静态校验。
它检查两类"写错了也不会报错、只会静默失效"的问题：

  1. delta_wing.json 的字段名 vs SIM_Plane.cpp 里注册的 COFF_FLOAT
     —— 字段名写错 → SITL 用默认值，你以为在测自己的模型，其实没有

  2. delta_wing.parm 的参数名 vs ArduPilot 源码
     —— 参数名写错 → 被静默忽略，你以为设了，其实没设

用法：
    python check_model.py --ardupilot /path/to/ardupilot

证据来源（三级，任一命中即认可）：
    A. 源码里的 `@Param: NAME` 元数据声明
    B. 源码（.cpp/.h）里出现过 "NAME" 字面量
    C. ArduPilot 官方参数文件里用到过 NAME（Tools/autotest/**/*.parm）
       —— 覆盖 PID 前缀型参数（RLL_RATE_* 等无 @Param 全名声明）
"""

import os
import re
import sys
import json
import glob
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))


def collect_sim_plane_fields(ap):
    """从 SIM_Plane.cpp 提取 COFF_FLOAT 注册的全部字段名。"""
    src = os.path.join(ap, 'libraries', 'SITL', 'SIM_Plane.cpp')
    if not os.path.isfile(src):
        return None, '找不到 %s' % src
    txt = open(src, encoding='utf-8', errors='replace').read()
    fields = set(re.findall(r'COFF_FLOAT\((\w+)\)', txt))
    return fields, None


def collect_param_evidence(ap):
    """收集参数名的三级证据。返回 (evidence_dict, stats)。"""
    ev = {}          # name -> set of 证据类型

    def add(name, kind):
        ev.setdefault(name, set()).add(kind)

    # ── C: 全仓库 .parm 文件（最直接的证据）──
    #   ⚠️ 初版只扫 Tools/autotest/**，漏掉 Tools/Frame_params/**，
    #      导致 NAVL1_DAMPING / TECS_TIME_CONST / TECS_THR_DAMP 被误报为
    #      "不存在" —— 那三个是【前缀式参数】(NAVL1_ / TECS_ 前缀 + 后缀)，
    #      源码里没有 @Param 全名声明，只能靠 .parm 使用记录来证明。
    #   ⚠️ 两种扩展名都要收：ArduPilot 同时用 .parm 和 .param
    #      （只收 .parm 会漏掉 NAVL1_DAMPING 等，误报为"参数不存在"）
    n_parm = 0
    for ext in ('*.parm', '*.param'):
        for p in glob.glob(os.path.join(ap, '**', ext), recursive=True):
            if '.git' in p:
                continue
            for line in open(p, encoding='utf-8', errors='replace'):
                s = line.strip()
                if not s or s.startswith('#'):
                    continue
                m = re.match(r'^([A-Z][A-Z0-9_]+)[\s,]', s)
                if m:
                    add(m.group(1), 'C')
                    n_parm += 1

    # ── A + B: 源码，外加前缀式参数的组合展开 ──
    n_files = 0
    pat_param = re.compile(r'@Param:\s*([A-Z][A-Z0-9_]+)')
    pat_str = re.compile(r'"([A-Z][A-Z0-9_]{2,})"')
    suffixes = set()      # AP_GROUPINFO("SUFFIX", ...)
    prefixes = set()      # "PREFIX_" 形式的表名前缀
    for root, _, files in os.walk(ap):
        if '.git' in root:
            continue
        for fn in files:
            if not fn.endswith(('.cpp', '.h')):
                continue
            n_files += 1
            try:
                txt = open(os.path.join(root, fn), encoding='utf-8',
                           errors='replace').read()
            except OSError:
                continue
            for m in pat_param.finditer(txt):
                add(m.group(1), 'A')
            for m in pat_str.finditer(txt):
                v = m.group(1)
                add(v, 'B')
                if v.endswith('_'):
                    prefixes.add(v)
            for m in re.finditer(r'AP_GROUPINFO\(\s*"([A-Z0-9_]+)"', txt):
                suffixes.add(m.group(1))

    # ⚠️ 这里【刻意不做】前缀×后缀的笛卡尔积展开。
    #    那样确实能自动覆盖 TECS_TIME_CONST 这类前缀式参数，
    #    但会造出成千上万个"看起来合法"的假名字，把假阴性换成假阳性 ——
    #    而假阳性更危险（拼错的参数名被放行、静默失效）。
    #    前缀式参数改由证据 C（全仓库 .parm 使用记录）覆盖，更可靠。

    return ev, dict(n_parm_lines=n_parm, n_source_files=n_files,
                    n_prefixes=len(prefixes), n_suffixes=len(suffixes))


def check_json(ap, verbose=True):
    fields, err = collect_sim_plane_fields(ap)
    if err:
        print('  ✗ %s' % err)
        return False
    path = os.path.join(HERE, 'delta_wing.json')
    if not os.path.isfile(path):
        print('  ✗ 找不到 delta_wing.json')
        return False
    mine = set(json.load(open(path, encoding='utf-8')))
    vector_fields = {'CGOffset'}

    missing = fields - mine                       # 源码要但我没给
    extra = mine - fields - vector_fields         # 我给了但源码不认

    print('  SIM_Plane.cpp 注册标量字段 : %d' % len(fields))
    print('  JSON 提供字段              : %d' % len(mine))
    if missing:
        print('  ✗ 源码需要但 JSON 缺失 (%d):' % len(missing))
        for m in sorted(missing):
            print('        %s' % m)
    if extra:
        print('  ✗ JSON 提供但源码不认 (%d):' % len(extra))
        for e in sorted(extra):
            print('        %s' % e)
    if not missing and not extra:
        print('  ✓ 字段名完全吻合（含 CGOffset 向量字段）')
    return not missing and not extra


def check_parm(ap, ev, verbose=True):
    path = os.path.join(HERE, 'delta_wing.parm')
    if not os.path.isfile(path):
        print('  ✗ 找不到 delta_wing.parm')
        return False
    names = []
    for line in open(path, encoding='utf-8'):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        m = re.match(r'^([A-Z][A-Z0-9_]+)\s', s)
        if m:
            names.append(m.group(1))

    ok, bad = 0, []
    for n in names:
        kinds = ev.get(n)
        if kinds:
            ok += 1
            if verbose:
                print('  ✓ %-22s 证据: %s' % (n, '/'.join(sorted(kinds))))
        else:
            bad.append(n)
    print()
    print('  参数总数 %d，有源码证据 %d' % (len(names), ok))
    if bad:
        print('  ✗ 找不到任何源码证据 (%d):' % len(bad))
        for b in bad:
            print('        %s   <- 很可能是拼错了，会被静默忽略' % b)
    else:
        print('  ✓ 全部参数名都能在 ArduPilot 源码中找到证据')
    return not bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ardupilot', required=True,
                    help='ArduPilot 源码树路径（ArduPlane 所在）')
    a = ap.parse_args()
    apath = os.path.abspath(a.ardupilot)

    print('═' * 70)
    print('  SITL 模型静态校验')
    print('  ArduPilot: %s' % apath)
    print('═' * 70)
    if not os.path.isdir(apath):
        print('✗ 路径不存在')
        return 1

    print()
    print('── 1. 气动模型 JSON 字段名 ──')
    ok_json = check_json(apath)

    print()
    print('── 2. 参数文件参数名 ──')
    ev, stats = collect_param_evidence(apath)
    print('  证据库：%d 个参数名（.parm 使用记录 %d 条，源码 %d 个文件）'
          % (len(ev), stats['n_parm_lines'], stats['n_source_files']))
    print('  证据分级：A=@Param 声明  B=源码字面量  C=.parm 使用记录')
    print()
    ok_parm = check_parm(apath, ev)

    print()
    print('═' * 70)
    if ok_json and ok_parm:
        print('  ✓ 两项校验全部通过')
        print('    ⚠️ 但这只是【静态】校验：字段名/参数名对得上，')
        print('       不代表气动系数正确、也不代表 SITL 真能飞起来。')
        print('       真正的验证必须在 WSL/Linux 下跑 run_sitl.sh。')
        return 0
    print('  ✗ 有校验失败项，先修再用')
    return 1


if __name__ == '__main__':
    sys.exit(main())
