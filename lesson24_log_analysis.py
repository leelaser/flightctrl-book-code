#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""《飞控学习教程》第 7 章 02 —— 日志分析与 FailSafe（配套脚本）

    python lesson24_log_analysis.py        # 跑全部 4 个实验
    python lesson24_log_analysis.py 1      # 只跑实验 1
    python lesson24_log_analysis.py 2      # 只跑实验 2
    python lesson24_log_analysis.py 3      # 只跑实验 3
    python lesson24_log_analysis.py 4      # 只跑实验 4

★ 本脚本只用 numpy（可选 matplotlib），在【任何机器】上都能跑通。
  实验 1 会检测本机有没有 pyulog / 真实日志；检测不到就打印
  「环境不满足，下面是原理性演示」，然后继续做数值演示 —— 不会报错。

★ 本脚本最核心的设计：**我们自己造日志，再自己解析回来**。
  这样即使机器上一条真实日志都没有，也能把 ULog 和 DataFlash
  两种格式的每一个字节讲清楚、并且当场验证。

四个实验：
    1  环境体检 + 两种日志格式总览（优雅降级）
    2  ★★ 手写一个 ULog 二进制文件，再解析回来（振动 / 姿态 / 电机 / EKF）
    3  ★★ 手写一个 DataFlash 日志，再解析回来（VIBE 到底怎么算的）
    4  ★★ 日志驱动的诊断 + FailSafe 触发与动作对比

课程分工说明：
    本脚本讲的是【应用层】：日志文件格式、怎么从日志里读出问题、
    以及飞控应用层的 FailSafe 状态机。
    看门狗（Watchdog）怎么喂狗、任务怎么被抢占、优先级怎么排，
    属于 RTOS/系统层，详见《FreeRTOS+ChibiOS 教程》第 20 课，本脚本不重复。
"""

import math
import os
import shutil
import struct
import sys

import numpy as np


# ============================================================================
#  输出小工具（与第 21、22、23 课脚本一致的排版）
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
    """按【显示宽度】折行打印（CJK 按 2 列算，对齐才好看）。

    两条规则：
      · 标点不落行首（NO_LINE_START）—— 中文排版的行首禁则
      · 如果本行后半段有空格，优先在空格处断开 —— 不要把英文标识符切成两半
    """
    pad = " " * indent
    line = pad
    w = 0
    sp_pos = -1          # 本行最后一个空格在 line 里的下标
    sp_w = 0             # 该空格处的显示列宽
    for ch in text:
        if ch == "\n":
            print(line.rstrip())
            line, w, sp_pos, sp_w = pad, 0, -1, 0
            continue
        cw = _dw(ch)
        # 标点不落行首：允许它把当前行挤出去一点点
        if w + cw > width and ch not in NO_LINE_START:
            if sp_pos > 0 and sp_w >= int(width * 0.55):
                carry = line[sp_pos + 1:]
                print(line[:sp_pos].rstrip())
                line = pad + carry
                w = _wide(carry)
                if w > width:                # 断出来的尾巴自己就超宽
                    print(line.rstrip())
                    line, w = pad, 0
            else:
                print(line.rstrip())
                line, w = pad, 0
            sp_pos, sp_w = -1, 0
        line += ch
        w += cw
        if ch == " ":
            sp_pos, sp_w = len(line) - 1, w
    if line.strip():
        print(line.rstrip())


def bullet(items, indent=4):
    for s in items:
        wrap(s, indent)


# ============================================================================
#  环境检测（实验 1 用）
# ============================================================================

PX4_CANDIDATES = [
    r"D:/DEVLib/PX4APM/PX4/PX4-Autopilot",
    r"D:/DEVLib/PX4APM/PX4/PX4-Autopilot-1.15.4/PX4-Autopilot-1.15.4",
    r"~/PX4-Autopilot",
    r"~/src/PX4-Autopilot",
]

AP_CANDIDATES = [
    r"D:/DEVLib/PX4APM/APM/ardupilot-Copter-4.5.7/ardupilot-Copter-4.5.7",
    r"~/ardupilot",
    r"~/src/ardupilot",
]

# 真实日志常常躺在这些地方
LOG_CANDIDATES = [
    # ★ 这一条是后补的。第一版只列了 SITL 的 rootfs/log 和两个通用下载目录，
    #   结果本机明明躺着 53 份真机 .ulg（PX4调参/ulog，8 个日期目录），
    #   脚本却报「真实 .ulg 日志 0 个」—— 检查器本身写错了候选路径，
    #   这是最隐蔽的一类错误：结论看起来像「环境不满足」，其实是「没找」。
    r"D:/DEVLib/PX4APM/PX4调参/ulog",
    r"~/PX4调参/ulog",
    r"D:/DEVLib/PX4APM/PX4/PX4-Autopilot/build/px4_sitl_default/rootfs/log",
    r"~/Documents/Flight_Logs",
    r"~/Downloads",
]


def expand(p):
    return os.path.abspath(os.path.expanduser(p))


def looks_like_px4(path):
    """PX4 源码树的指纹：Makefile + ROMFS 的 POSIX rcS + SITL 板级配置"""
    if not os.path.isdir(path):
        return False
    must = [
        "Makefile",
        "ROMFS/px4fmu_common/init.d-posix/rcS",
        "boards/px4/sitl/default.px4board",
    ]
    return all(os.path.exists(os.path.join(path, m)) for m in must)


def looks_like_ardupilot(path):
    """ArduPilot 源码树的指纹：ArduCopter + AP_Scheduler + sim_vehicle.py"""
    if not os.path.isdir(path):
        return False
    must = [
        "ArduCopter",
        "libraries/AP_Scheduler/AP_Scheduler.cpp",
        "Tools/autotest/sim_vehicle.py",
    ]
    return all(os.path.exists(os.path.join(path, m)) for m in must)


def find_first(cands, test):
    for c in cands:
        p = expand(c)
        if test(p):
            return p
    return None


def have_cmd(name):
    """命令是否存在（不执行，只查 PATH）。"""
    return shutil.which(name) is not None


def have_module(name):
    """Python 模块是否存在（只查找，不导入，避免副作用）。"""
    try:
        import importlib.util
        return importlib.util.find_spec(name) is not None
    except Exception:
        return False


def find_logs(paths, exts, limit=4):
    """在候选目录里找真实日志文件（只找一层 + 一层子目录）。"""
    found = []
    for p in paths:
        p = expand(p)
        if not os.path.isdir(p):
            continue
        for root, dirs, files in os.walk(p):
            depth = root[len(p):].count(os.sep)
            if depth > 1:
                dirs[:] = []
                continue
            for f in files:
                if os.path.splitext(f)[1].lower() in exts:
                    found.append(os.path.join(root, f))
                    if len(found) >= limit:
                        return found
    return found


# ============================================================================
#  第一部分：ULog 文件格式的【最小可运行实现】
#
#  下面每一个字节的规定，都来自 PX4 源码：
#    src/modules/logger/messages.h           （结构体定义）
#    src/modules/logger/logger.cpp           （怎么写）
#    platforms/common/uORB/uORB.cpp          （类型名怎么展开）
#  行号在函数/类的注释里逐条注明。
# ============================================================================

# ---- 文件头 ---------------------------------------------------------------
# messages.h:59-62  struct ulog_file_header_s { uint8_t magic[8]; uint64_t timestamp; }
# logger.cpp:2017-2025  magic = 'U','L','o','g',0x01,0x12,0x35,0x01 ; timestamp = hrt_absolute_time()
ULOG_MAGIC = bytes([0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35, 0x01])
ULOG_FILE_HEADER_LEN = 16          # 8 字节 magic + 8 字节 uint64 时间戳

# ---- 消息类型 -------------------------------------------------------------
# messages.h:38-52  enum class ULogMessageType : uint8_t
#   FORMAT='F'  DATA='D'  INFO='I'  INFO_MULTIPLE='M'  PARAMETER='P'
#   PARAMETER_DEFAULT='Q'  ADD_LOGGED_MSG='A'  REMOVE_LOGGED_MSG='R'
#   SYNC='S'  DROPOUT='O'  LOGGING='L'  LOGGING_TAGGED='C'  FLAG_BITS='B'
T_FORMAT = ord("F")
T_DATA = ord("D")
T_INFO = ord("I")
T_PARAMETER = ord("P")
T_ADD_LOGGED = ord("A")
T_SYNC = ord("S")
T_DROPOUT = ord("O")
T_LOGGING = ord("L")
T_FLAG_BITS = ord("B")

# messages.h:98-103
#   struct ulog_message_header_s { uint16_t msg_size; uint8_t msg_type; }
#   #define ULOG_MSG_HEADER_LEN 3
#   ★ msg_size【不包含】这 3 个字节自己！这是 ULog 最容易读错的地方。
ULOG_MSG_HEADER_LEN = 3

# ---- 类型名 ---------------------------------------------------------------
# FORMAT 串里的类型名是【展开后】的 C 类型名。展开函数：
#   platforms/common/uORB/uORB.cpp:187-217  orb_get_c_type()
#   0x89 -> "uint64_t"   0x8a -> "float"   0x8c -> "bool"   0x86 -> "uint8_t"
# 注意：源文件 msg/*.msg 里写的是 float32，但落到日志里是 float。
ULOG_C_TYPE_TO_STRUCT = {
    "int8_t": "b", "uint8_t": "B",
    "int16_t": "h", "uint16_t": "H",
    "int32_t": "i", "uint32_t": "I",
    "int64_t": "q", "uint64_t": "Q",
    "float": "f", "double": "d",
    "bool": "?", "char": "c",
}


def spec_to_format(name, fields):
    """把字段表写成 ULog 的 FORMAT 串。

    messages.h:112-113 给出格式和真实样例：
        "message_name:field0;field1;"
        "action_request:uint64_t timestamp;uint8_t action;uint8_t source;
         uint8_t mode;uint8_t[5] _padding0;"
    """
    parts = []
    for typ, n, fname in fields:
        t = typ if n == 1 else "%s[%d]" % (typ, n)
        parts.append("%s %s" % (t, fname))
    return name + ":" + ";".join(parts) + ";"


def parse_format_string(fmt):
    """把 FORMAT 串拆回 (名字, [(类型, 个数, 字段名), ...])。"""
    name, _, body = fmt.partition(":")
    if body.endswith(";"):
        body = body[:-1]
    fields = []
    for tok in body.split(";"):
        tok = tok.strip()
        if not tok:
            continue
        typ, _, fname = tok.partition(" ")
        n = 1
        if "[" in typ:
            typ, _, rest = typ.partition("[")
            n = int(rest.rstrip("]"))
        fields.append((typ, n, fname))
    return name, fields


def struct_fmt_of(fields):
    """字段表 -> struct 模块的格式串（小端）。"""
    out = "<"
    for typ, n, _ in fields:
        if typ not in ULOG_C_TYPE_TO_STRUCT:
            raise ValueError("未知类型：" + typ)
        out += ULOG_C_TYPE_TO_STRUCT[typ] * n
    return out


def fields_size(fields):
    return struct.calcsize(struct_fmt_of(fields))


def unpack_values(fields, blob):
    """按字段表把字节解成 dict（数组字段展开成 np 数组）。"""
    vals = struct.unpack(struct_fmt_of(fields), blob)
    out = {}
    k = 0
    for typ, n, fname in fields:
        chunk = vals[k:k + n]
        k += n
        if fname.startswith("_padding"):
            continue
        out[fname] = chunk[0] if n == 1 else np.array(chunk, dtype=float)
    return out


class ULogBuilder:
    """按 PX4 的写法，一个字节一个字节地造一个 ULog。"""

    def __init__(self, t0_us=0):
        self.buf = bytearray()
        self.n_msg = {}                     # 类型名 -> 条数
        self.formats = {}                   # 名字 -> 字段表
        self.msg_id = {}                    # 名字 -> logger 内部 id
        self._next_id = 0

        # ---- ① 文件头（16 字节）----
        self.buf += ULOG_MAGIC
        self.buf += struct.pack("<Q", t0_us)

        # ---- ② FLAG_BITS：logger.cpp:2029 注释写着
        #      「this MUST be written right after the ulog header」
        #      messages.h:288-295
        #        uint8_t compat_flags[8]; uint8_t incompat_flags[8];
        #        uint64_t appended_offsets[3];
        #      logger.cpp:2032  compat_flags[0] = ULOG_COMPAT_FLAG0_DEFAULT_PARAMETERS_MASK (=1<<0)
        compat = bytes([1, 0, 0, 0, 0, 0, 0, 0])
        incompat = bytes(8)
        offsets = struct.pack("<3Q", 0, 0, 0)
        self._raw_msg(T_FLAG_BITS, compat + incompat + offsets)

    # --- 底层：写一条带 3 字节头的消息 ---
    def _raw_msg(self, mtype, payload):
        # messages.h:99  msg_size 是【不含 3 字节头】的长度
        self.buf += struct.pack("<HB", len(payload), mtype)
        self.buf += payload
        key = "0x%02X" % mtype
        self.n_msg[key] = self.n_msg.get(key, 0) + 1

    # --- FORMAT ---
    def add_format(self, name, fields):
        fmt = spec_to_format(name, fields)
        blob = fmt.encode("ascii")
        self._raw_msg(T_FORMAT, blob)
        self.formats[name] = fields
        return fmt

    # --- ADD_LOGGED_MSG ---
    # messages.h:129-136
    #   uint8_t multi_id;        <-- 注意：multi_id 在 msg_id【前面】
    #   uint16_t msg_id;
    #   char message_name[255];  <-- 实际只写 name 那么长，不带结尾 NUL
    # logger.cpp:1856-1857  msg_size = sizeof(msg) - sizeof(message_name) + strlen(name)
    def add_logged(self, name, multi_id=0):
        if name not in self.msg_id:
            self.msg_id[name] = self._next_id
            self._next_id += 1
        payload = struct.pack("<BH", multi_id, self.msg_id[name])
        payload += name.encode("ascii")
        self._raw_msg(T_ADD_LOGGED, payload)
        return self.msg_id[name]

    # --- DATA ---
    # messages.h:175-180  struct ulog_message_data_s { uint16_t msg_size;
    #                     uint8_t msg_type; uint16_t msg_id; } 后面跟原始数据
    def add_data(self, name, fields, values):
        mid = self.msg_id[name]
        payload = struct.pack("<H", mid)
        for typ, n, fname in fields:
            if fname.startswith("_padding"):
                # ★ 对齐填充字节：解析器要跳过它，写入方要补 0 填满它。
                #   真实 uORB 也会生成这种字段，见
                #   Tools/msg/px_generate_uorb_topic_helper.py:229-277
                payload += b"\x00" * (struct.calcsize("<" + ULOG_C_TYPE_TO_STRUCT[typ]) * n)
                continue
            v = values[fname]
            if n == 1:
                payload += struct.pack("<" + ULOG_C_TYPE_TO_STRUCT[typ], v)
            else:
                arr = np.asarray(v, dtype=float).reshape(-1)
                payload += struct.pack("<" + ULOG_C_TYPE_TO_STRUCT[typ] * n, *arr[:n])
        self._raw_msg(T_DATA, payload)

    # --- INFO ---
    # messages.h:189-195  uint8_t key_len; char key_value_str[255];
    # logger.cpp:1874  key = "char[<vlen>] <name>"，紧接着就是 value 本身（无分隔）
    def add_info(self, key, value):
        key_str = "char[%d] %s" % (len(value), key)
        kb = key_str.encode("ascii")
        self._raw_msg(T_INFO, bytes([len(kb)]) + kb + value.encode("ascii"))

    # --- PARAMETER ---
    # messages.h:242-248  格式与 INFO 相同，类型串形如 "float top_k"
    def add_param(self, typ, name, value):
        key_str = "%s %s" % (typ, name)
        kb = key_str.encode("ascii")
        txt = ("%.6f" % value).rstrip("0").rstrip(".")
        self._raw_msg(T_PARAMETER, bytes([len(kb)]) + kb + txt.encode("ascii"))

    # --- DROPOUT ---
    # messages.h:160-165  uint16_t duration; 单位 ms
    # logger.cpp:1204-1211  dura = (hrt_elapsed_time(&dropout_start)/1000)/1e3 秒
    def add_dropout(self, duration_ms):
        self._raw_msg(T_DROPOUT, struct.pack("<H", duration_ms))

    # --- SYNC ---
    # messages.h:153-158  uint8_t sync_magic[8];
    def add_sync(self, magic=b"\x2F\x73\x13\x20\x25\x0C\xBB\x12"):
        self._raw_msg(T_SYNC, magic)

    def bytes(self):
        return bytes(self.buf)


class ULogReader:
    """把上面造出来的字节流按【同样的规则】解析回来。"""

    def __init__(self, data):
        self.data = data
        self.formats = {}       # name -> fields
        self.logged = {}        # msg_id -> name
        self.series = {}        # name -> list of dict
        self.infos = {}
        self.params = {}
        self.dropouts = []
        self.n_msg = {}
        self.header_ok = False
        self.header_ts = 0
        self._parse()

    def _parse(self):
        d = self.data
        if len(d) < ULOG_FILE_HEADER_LEN:
            return
        self.header_ok = (d[0:8] == ULOG_MAGIC)
        self.header_ts = struct.unpack_from("<Q", d, 8)[0]
        off = ULOG_FILE_HEADER_LEN
        while off + ULOG_MSG_HEADER_LEN <= len(d):
            size, mtype = struct.unpack_from("<HB", d, off)
            body = d[off + ULOG_MSG_HEADER_LEN: off + ULOG_MSG_HEADER_LEN + size]
            off += ULOG_MSG_HEADER_LEN + size
            key = "0x%02X" % mtype
            self.n_msg[key] = self.n_msg.get(key, 0) + 1

            if mtype == T_FORMAT:
                txt = body.decode("ascii").rstrip("\x00")
                name, fields = parse_format_string(txt)
                self.formats[name] = fields
            elif mtype == T_ADD_LOGGED:
                multi_id, mid = struct.unpack_from("<BH", body, 0)
                nm = body[3:].decode("ascii")
                self.logged[mid] = nm
                self.series.setdefault(nm, [])
            elif mtype == T_DATA:
                mid = struct.unpack_from("<H", body, 0)[0]
                nm = self.logged.get(mid)
                if nm is None or nm not in self.formats:
                    continue
                self.series[nm].append(unpack_values(self.formats[nm], body[2:]))
            elif mtype == T_INFO:
                klen = body[0]
                k = body[1:1 + klen].decode("ascii")
                self.infos[k] = body[1 + klen:].decode("ascii")
            elif mtype == T_PARAMETER:
                klen = body[0]
                k = body[1:1 + klen].decode("ascii")
                self.params[k] = body[1 + klen:].decode("ascii")
            elif mtype == T_DROPOUT:
                self.dropouts.append(struct.unpack_from("<H", body, 0)[0])
        return

    def arr(self, name, field):
        return np.array([s[field] for s in self.series[name]], dtype=float)


# ============================================================================
#  第二部分：DataFlash（ArduPilot）文件格式的【最小可运行实现】
#
#  字节规定来自：
#    libraries/AP_Logger/LogStructure.h      （结构体 + 格式字符表 + 消息表）
#    libraries/AP_Logger/LogFile.cpp         （怎么填 FMT）
#    libraries/AP_Logger/AP_Logger_Backend.cpp（怎么按格式串打包）
# ============================================================================

# LogStructure.h:121-122
DF_HEAD_BYTE1 = 0xA3
DF_HEAD_BYTE2 = 0x95

# LogStructure.h:1394  LOG_FORMAT_MSG = 128, // this must remain #128
DF_MSG_FMT = 128
# LogStructure.h:1346  LOG_PARAMETER_MSG = 32
DF_MSG_PARM = 32

# LogStructure.h:8-29  DataFlash 的格式字符表（★ 与 ULog 完全不是一套）
#   a=int16_t[32]  b=int8_t  B=uint8_t  h=int16_t  H=uint16_t
#   i=int32_t  I=uint32_t  f=float  d=double
#   n=char[4]  N=char[16]  Z=char[64]
#   c=int16_t*100  C=uint16_t*100  e=int32_t*100  E=uint32_t*100
#   L=int32_t 纬度/经度  M=uint8_t 飞行模式  q=int64_t  Q=uint64_t
DF_FMT_TO_STRUCT = {
    "b": "b", "B": "B", "h": "h", "H": "H",
    "i": "i", "I": "I", "f": "f", "d": "d",
    "q": "q", "Q": "Q",
    "c": "h", "C": "H", "e": "i", "E": "I", "L": "i", "M": "B",
}
# 定点缩放：格式字符 -> 需要乘的系数
DF_FMT_SCALE = {
    "c": 0.01, "C": 0.01, "e": 0.01, "E": 0.01,
}
DF_STR_FMT = {"n": 4, "N": 16, "Z": 64}
DF_ARRAY_FMT = {"a": (32, "h")}


def df_struct_fmt(fmt):
    """DataFlash 格式串 -> struct 格式串（大端！ChibiOS/ARM 上的 DataFlash 是网络序）。"""
    out = ">"
    for ch in fmt:
        if ch in DF_STR_FMT:
            out += "%ds" % DF_STR_FMT[ch]
        elif ch in DF_ARRAY_FMT:
            n, c = DF_ARRAY_FMT[ch]
            out += c * n
        elif ch in DF_FMT_TO_STRUCT:
            out += DF_FMT_TO_STRUCT[ch]
        else:
            raise ValueError("未知 DataFlash 格式字符：" + ch)
    return out


def df_record_size(fmt):
    return struct.calcsize(df_struct_fmt(fmt))


class DataFlashBuilder:
    """按 ArduPilot 的写法造一个 DataFlash 日志。"""

    def __init__(self):
        self.buf = bytearray()
        self.fmt = {}          # msgid -> (name, format, labels)
        self.name2id = {}      # name -> msgid
        # ★ 2026-09 复核：这里原来有一行 `self._next_id = 200`，注释写「见下面 ④ 的说明」——
        #   两处都不对：(a) 该属性在整个类里【只有赋值、没有任何读取】（死代码）；
        #   (b) 下面 ④（`sub("④ DataFlash 的自描述…")`）讲的是 FMT 自举，根本不提它。
        #   msgid 是 `DF_FORMATS` 里显式给出的（不靠自增），所以这行从来没用上。
        #   ⇒ 删掉，并把「id 从哪来」的事实在 DF_FORMATS 上方说清楚。

    def _rec(self, msgid, payload):
        # LogFile.cpp:31-33 / AP_Logger_Backend.cpp:247-249
        self.buf += bytes([DF_HEAD_BYTE1, DF_HEAD_BYTE2, msgid])
        self.buf += payload

    def add_fmt(self, msgid, name, fmt, labels):
        # LogStructure.h:176-183  struct log_Format
        #   LOG_PACKET_HEADER; uint8_t type; uint8_t length;
        #   char name[4]; char format[16]; char labels[64];
        # LogFile.cpp:34-38  type=msg_type, length=msg_len（★ 含 3 字节头）
        length = 3 + df_record_size(fmt)
        payload = struct.pack(">BB", msgid, length)
        payload += name.encode("ascii").ljust(4, b"\x00")[:4]
        payload += fmt.encode("ascii").ljust(16, b"\x00")[:16]
        payload += labels.encode("ascii").ljust(64, b"\x00")[:64]
        self._rec(DF_MSG_FMT, payload)
        self.fmt[msgid] = (name, fmt, labels)
        self.name2id[name] = msgid

    def add_parm(self, name, value):
        # LogStructure.h:1225  "PARM", "QNff", "TimeUS,Name,Value,Default"
        # LogStructure.h:207-213  uint64 time_us; char name[16]; float value; float default_value;
        payload = struct.pack(">Q", 0)
        payload += name.encode("ascii").ljust(16, b"\x00")[:16]
        payload += struct.pack(">ff", value, value)
        self._rec(DF_MSG_PARM, payload)

    def add(self, name, **kw):
        """按名字写一条数据记录；字段值以关键字给出（名字取 labels 的前几段）。"""
        msgid = self.name2id[name]
        _, fmt, labels = self.fmt[msgid]
        cols = labels.split(",")
        vals = []
        for i, ch in enumerate(fmt):
            # 格式串的第 i 个字符 <=> labels 的第 i 段（labels 是【含首字段】的完整列名表）
            col = cols[i] if i < len(cols) else ""
            if ch in DF_STR_FMT:
                s = str(kw.get(col, ""))
                vals.append(s.encode("ascii").ljust(DF_STR_FMT[ch], b"\x00")[:DF_STR_FMT[ch]])
            elif ch in DF_ARRAY_FMT:
                vals.append(np.zeros(32, dtype=np.int16))
            else:
                vals.append(kw.get(col, 0))
        self._rec(msgid, struct.pack(df_struct_fmt(fmt), *vals))

    def bytes(self):
        return bytes(self.buf)


class DataFlashReader:
    """DataFlash 是【自描述】的：先读 FMT，再按 FMT 解数据。"""

    def __init__(self, data):
        self.data = data
        self.fmt = {}        # msgid -> (name, fmt, labels)
        self.name2id = {}    # name -> msgid
        self.series = {}     # name -> list of dict
        self.records = {}    # name -> 条数
        self.params = {}     # PARM 消息解出来的 参数名 -> 值
        self.n_skipped = 0
        self._parse()

    def _parse(self):
        d = self.data
        off = 0
        n = len(d)
        while off + 3 <= n:
            if d[off] != DF_HEAD_BYTE1 or d[off + 1] != DF_HEAD_BYTE2:
                # 真实日志里靠扫描 0xA3 0x95 重新同步
                off += 1
                self.n_skipped += 1
                continue
            msgid = d[off + 2]
            if msgid == DF_MSG_FMT:
                body = d[off + 3: off + 3 + 1 + 1 + 4 + 16 + 64]
                typ, length = struct.unpack_from(">BB", body, 0)
                nm = body[2:6].split(b"\x00")[0].decode("ascii")
                fmt = body[6:22].split(b"\x00")[0].decode("ascii")
                labels = body[22:86].split(b"\x00")[0].decode("ascii")
                self.fmt[typ] = (nm, fmt, labels)
                self.name2id[nm] = typ
                self.series.setdefault(nm, [])
                off += length
                continue

            if msgid not in self.fmt:
                self.n_skipped += 1
                off += 1
                continue

            nm, fmt, labels = self.fmt[msgid]
            size = 3 + df_record_size(fmt)
            body = d[off + 3: off + size]
            cols = labels.split(",")
            vals = struct.unpack(df_struct_fmt(fmt), body)
            rec = {}
            for i, ch in enumerate(fmt):
                col = cols[i] if i < len(cols) else ("f%d" % i)
                v = vals[i]
                if ch in DF_STR_FMT:
                    v = v.split(b"\x00")[0].decode("ascii", "replace")
                elif ch in DF_ARRAY_FMT:
                    v = np.array(v, dtype=float)
                elif ch in DF_FMT_SCALE:
                    v = float(v) * DF_FMT_SCALE[ch]
                rec[col] = v
            self.series[nm].append(rec)
            self.records[nm] = self.records.get(nm, 0) + 1
            if msgid == DF_MSG_PARM:
                # PARM 单独存一份，方便像 MAVExplorer 那样「先看参数再谈故障」
                self.params[rec.get("Name", "")] = rec.get("Value", 0.0)
            off += size

    def arr(self, name, field):
        return np.array([r[field] for r in self.series[name]], dtype=float)


# ============================================================================
#  实验 1：环境体检 + 两种日志格式总览
# ============================================================================

def exp1():
    head("实验 1  环境体检：本机能不能分析真实日志？")

    px4 = find_first(PX4_CANDIDATES, looks_like_px4)
    ap = find_first(AP_CANDIDATES, looks_like_ardupilot)
    has_pyulog = have_module("pyulog")
    has_pymav = have_module("pymavlink")
    ulg = find_logs(LOG_CANDIDATES, {".ulg"})
    bin_ = find_logs(LOG_CANDIDATES, {".bin", ".log"})

    sub("① 逐项体检")

    rows = [
        ["pyulog（解析 ULog 的标准库）", "有" if has_pyulog else "没有",
         "pip install pyulog"],
        ["pymavlink（通用的 MAVLink 工具）", "有" if has_pymav else "没有",
         "pip install pymavlink"],
        ["PX4 源码树", "有" if px4 else "没有",
         (px4 or "-")],
        ["ArduPilot 源码树", "有" if ap else "没有",
         (ap or "-")],
        ["真实 .ulg 日志", "%d 个" % len(ulg),
         (os.path.basename(ulg[0]) if ulg else "示例路径下没找到")],
        ["真实 .bin 日志", "%d 个" % len(bin_),
         (os.path.basename(bin_[0]) if bin_ else "示例路径下没找到")],
    ]
    tab(rows, ["检查项", "结果", "说明 / 路径"])

    ok = has_pyulog and (len(ulg) > 0 or len(bin_) > 0)
    if not ok:
        print()
        print("  " + "*" * (W - 4))
        print("  ★ 环境不满足，下面是原理性演示。")
        print("  " + "*" * (W - 4))
        wrap("本机没有装 pyulog，也没有找到真实飞行日志。这不影响本课："
             "接下来我们会【自己造】一个合法的 ULog 二进制文件和一个合法的 "
             "DataFlash 日志，再用自己写的解析器读回来。"
             "造日志 - 写解析器 - 读回来 - 对答案，这条闭环比拿一条别人的日志"
             "点几下鼠标更能看清格式。")
    else:
        print()
        wrap("环境满足：下面照样自己造日志，讲完原理后你可以用 pyulog 打开真机日志验证。")

    sub("② ULog 与 DataFlash：一张表看清区别")

    rows = [
        ["谁用", "PX4", "ArduPilot"],
        ["文件后缀", ".ulg", ".bin（DF 后端）/ .log（文件后端）"],
        ["字节序", "小端（<）", "大端（>，ChibiOS/ARM 网络序）"],
        ["怎么起头", "16 字节固定头：magic + 时间戳",
         "没有文件头，直接就是一条条记录"],
        ["记录怎么分帧", "3 字节头：uint16 msg_size + uint8 msg_type",
         "3 字节头：0xA3 0x95 + uint8 msgid"],
        ["msg_size 含义", "★ 不含 3 字节头自己", "★ 含 3 字节头（FMT.length 字段）"],
        ["字段类型名", "在 FORMAT 消息里以【文本】给出",
         "在 FMT 消息里以【单字符】给出"],
        ["自描述吗", "是（FORMAT 消息）", "是（FMT 消息，且 FMT 描述 FMT 自己）"],
        ["名字长度", "变长（只写实际长度，无 NUL）", "定长（name[4] / labels[64] 补 0）"],
        ["丢数据怎么记", "DROPOUT 消息（uint16 ms）", "没有专门消息，靠 MSG 文本 + 序号"],
    ]
    tab(rows, ["对比项", "ULog（PX4）", "DataFlash（ArduPilot）"])

    sub("③ ULog 的 13 种消息类型（源码 src/modules/logger/messages.h:38-52）")

    rows = [
        ["F", "FORMAT", "一条 topic 的字段定义（文本）"],
        ["D", "DATA", "一条实际数据（uint16 msg_id + 原始结构体）"],
        ["I", "INFO", "键值信息（软件版本、板子名字、开机控制台输出）"],
        ["M", "INFO_MULTIPLE", "超长键值信息（分片续传，如 perf 计数器）"],
        ["P", "PARAMETER", "参数值（\"float MPC_THR_HOVER\" + \"0.6\"）"],
        ["Q", "PARAMETER_DEFAULT", "参数默认值（用来找出哪些参数被改过）"],
        ["A", "ADD_LOGGED_MSG", "声明 msg_id 对应哪个 topic 名"],
        ["R", "REMOVE_LOGGED_MSG", "取消订阅某个 topic"],
        ["S", "SYNC", "8 字节同步魔数，分割「定义段」和「数据段」"],
        ["O", "DROPOUT", "日志写入跟不上，丢了 uint16 毫秒的数据"],
        ["L", "LOGGING", "日志里的一条文本（PX4_INFO / MAVLink STATUSTEXT）"],
        ["C", "LOGGING_TAGGED", "带 tag 的文本"],
        ["B", "FLAG_BITS", "兼容/非兼容标志位，★ 必须紧跟文件头"],
    ]
    tab(rows, ["字符", "名字", "干什么用"])

    sub("④ DataFlash 的自描述是怎么做到的")

    wrap("DataFlash 没有文件头，第一条记录就是 FMT（msgid = 128，"
         "见 libraries/AP_Logger/LogStructure.h:1394）。")
    print()
    print("      记录头        payload（log_Format 结构体）")
    print("      A3 95 80  |  type  length  name[4]  format[16]  labels[64]")
    print("      ^^^^^^^^     ^^^^  ^^^^^^  ^^^^^^^  ^^^^^^^^^^  ^^^^^^^^^^")
    print("      0xA3 0x95    msgid 该消息   4 字符   格式串      列名")
    print("      + msgid=128  的 id   长度")
    print()
    wrap("★ 关键：FMT 记录【也描述 FMT 自己】。第一条 FMT 的 type 字段就是 128，"
         "name=\"FMT\"，format=\"BBnNZ\"。这样解析器读到任何一条记录，"
         "都能先在「已读到的 FMT 表」里查 msgid，不需要预先知道任何 id 的含义。"
         "这正是 ArduPilot 能随便加新日志消息而不破坏老解析器的原因。")

    return ok


# ============================================================================
#  实验 2：★★ 手写一个 ULog，再解析回来
# ============================================================================

# 本实验用到的 topic（★ 教学裁剪版：只留几个字段，字段类型和命名规则与真实 PX4 一致）
# 真实 topic 的 FORMAT 串由 msg/*.msg 自动生成，字段更多。
ULOG_TOPICS = {
    # 真实对应 msg/VehicleAttitude.msg: timestamp, timestamp_sample, q[4], ...
    "vehicle_attitude": [
        ("uint64_t", 1, "timestamp"),
        ("float", 4, "q"),
    ],
    # 真实对应 msg/VehicleAttitudeSetpoint.msg
    "vehicle_attitude_setpoint": [
        ("uint64_t", 1, "timestamp"),
        ("float", 4, "q_d"),
    ],
    # 真实对应 msg/VehicleImuStatus.msg:18-19 的振动指标
    "vehicle_imu_status": [
        ("uint64_t", 1, "timestamp"),
        ("uint32_t", 1, "accel_clipping"),
        ("uint32_t", 1, "gyro_clipping"),
        ("float", 1, "accel_vibration_metric"),
        ("float", 1, "gyro_vibration_metric"),
        ("float", 1, "delta_angle_coning_metric"),
    ],
    # 真实对应 msg/ActuatorMotors.msg
    "actuator_motors": [
        ("uint64_t", 1, "timestamp"),
        ("float", 4, "control"),
    ],
    # 真实对应 msg/EstimatorInnovations.msg:40 的 test_ratios 话题
    "estimator_innovation_test_ratios": [
        ("uint64_t", 1, "timestamp"),
        ("float", 2, "gps_hpos"),
    ],
    # ★ 演示 uORB 的 _padding 规则用的裁剪版状态话题
    #   真实 uORB 的 _padding 生成规则见
    #   Tools/msg/px_generate_uorb_topic_helper.py:229-277（按最大成员对齐补字节）
    #   真实样例见 src/modules/logger/messages.h:113 的 "uint8_t[5] _padding0;"
    "vehicle_status": [
        ("uint64_t", 1, "timestamp"),
        ("uint8_t", 1, "nav_state"),
        ("uint8_t", 3, "_padding0"),
        ("uint32_t", 1, "failsafe"),
    ],
}

# 日志采样率（对应 src/modules/logger/logged_topics.cpp:46 起的 add_default_topics）
#   :122 vehicle_angular_velocity 20 ms
#   :123 vehicle_attitude        50 ms   -> 20 Hz
#   :124 vehicle_attitude_setpoint 50 ms -> 20 Hz
#   :170 estimator_innovation_test_ratios 500 ms -> 2 Hz
#   :227 vehicle_imu_status     1000 ms  -> 1 Hz
#   :235 actuator_motors         100 ms  -> 10 Hz
FS_ATT = 20.0
FS_MOT = 10.0
FS_VIB = 1.0
FS_INN = 2.0
FS_STA = 1.0
T_END = 40.0


def euler_to_quat(roll, pitch, yaw):
    """ZYX 内旋（先滚转、再俯仰、最后偏航），返回 Hamilton 约定 (w, x, y, z)。

    推导：q = qz(yaw) * qy(pitch) * qx(roll)（Hamilton 乘积，右乘先作用）
        qx(a) = (cos(a/2), sin(a/2), 0, 0)
        qy(b) = (cos(b/2), 0, sin(b/2), 0)
        qz(c) = (cos(c/2), 0, 0, sin(c/2))
    三个逐个乘开就得到下面四行（半角公式的乘积展开）。
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def quat_mul(a, b):
    """Hamilton 乘积，a、b 都是 (w, x, y, z)。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_to_rot(q):
    """四元数 -> 旋转矩阵（body -> earth）。

    标准公式，由 R = I + 2w[q]_x + 2[q]_x^2 展开得到。
    """
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def attitude_error_rad(q_actual, q_sp):
    """姿态跟踪误差（弧度）。两种算法互相验证。

    算法 A（四元数）：q_err = conj(q_sp) ⊗ q_actual，则
        R(q_err) = R(q_sp)^T R(q_actual)
    含义：把「实际机体系」转到「期望机体系」所需的那个旋转。
    由 q_err = (cos(θ/2), sin(θ/2)·n) 立得
        w_err = cos(θ/2)  =>  θ = 2·arccos(|w_err|)
    （取绝对值是为了处理双重覆盖：q 与 -q 表示同一个旋转）

    算法 B（矩阵）：θ = arccos( (trace(R_err) - 1) / 2 )
    由 trace(R) = 1 + 2cos(θ) 得到。
    """
    q_sp_c = np.array([q_sp[0], -q_sp[1], -q_sp[2], -q_sp[3]])
    q_err = quat_mul(q_sp_c, q_actual)
    w = min(1.0, max(-1.0, abs(q_err[0])))
    th_a = 2.0 * math.acos(w)

    r_err = quat_to_rot(q_sp).T @ quat_to_rot(q_actual)
    tr = float(np.trace(r_err))
    th_b = math.acos(min(1.0, max(-1.0, (tr - 1.0) * 0.5)))
    return th_a, th_b


def px4_vibration_metric(diff_norm, alpha=0.01):
    """PX4 的 accel_vibration_metric 递推式。

    源码 src/modules/sensors/vehicle_imu/VehicleIMU.cpp:740-747：
        _status.accel_vibration_metric = 0.99f * _status.accel_vibration_metric
                                         + 0.01f * Vector3f(acceleration - _acceleration_prev).norm();
    即  v[k] = (1-a)·v[k-1] + a·||a[k] - a[k-1]||，a = 0.01
    稳态（d 平稳）时 v = E[d]，也就是「相邻采样差值的范数」的滑动平均。
    """
    v = np.zeros(len(diff_norm))
    acc = 0.0
    for k in range(len(diff_norm)):
        acc = (1.0 - alpha) * acc + alpha * diff_norm[k]
        v[k] = acc
    return v


def exp2():
    head("实验 2  ★★ 手写一个 ULog 二进制文件，再解析回来")

    sub("① 先看清楚每个字节该写什么")

    rows = [
        ["文件头", "16", "8 字节 magic = 55 4C 6F 67 01 12 35 01，8 字节 uint64 起始时间戳"],
        ["FLAG_BITS", "3 + 40", "8 字节 compat + 8 字节 incompat + 3×uint64 追加偏移"],
        ["FORMAT", "3 + 长度", "topic 名 + ':' + 字段定义，纯 ASCII 文本"],
        ["ADD_LOGGED_MSG", "3 + 3 + N", "uint8 multi_id + uint16 msg_id + N 字节名字（无 NUL）"],
        ["DATA", "3 + 2 + M", "uint16 msg_id + M 字节原始结构体"],
    ]
    tab(rows, ["段", "字节数", "内容"])

    print()
    wrap("★ 三处最容易读错的地方（本实验会逐条当场验证）：")
    bullet([
        "1) msg_size 不包含 3 字节头自己 —— messages.h:99 原话是 "
        "「Size of the message excluding the header size」。",
        "2) ADD_LOGGED_MSG 里 multi_id 在 msg_id【前面】—— messages.h:133-134。",
        "3) INFO / PARAMETER 的 key_len 后面【没有分隔符】，值紧接着就来了 —— "
        "logger.cpp:1874-1880 是先 snprintf 出 key，再把 value memcpy 到 msg_size 位置。",
    ])

    sub("② 造日志")

    b = ULogBuilder(t0_us=1_000_000)
    for nm, flds in ULOG_TOPICS.items():
        fmt = b.add_format(nm, flds)
        b.add_logged(nm)
        print("      FORMAT  " + fmt)

    b.add_info("ver_sw", "1.15.4")
    b.add_info("sys_name", "PX4")
    b.add_param("float", "MPC_THR_HOVER", 0.6)
    b.add_param("int32_t", "IMU_INTEG_RATE", 250)
    b.add_sync()

    # ---- 造飞行数据 --------------------------------------------------------
    rng = np.random.default_rng(20240601)

    def seg(t):
        if t < 5.0:
            return "地面待机"
        if t < 15.0:
            return "正常悬停"
        if t < 25.0:
            return "★ 桨叶损伤（振动段）"
        if t < 28.0:
            return "★ 猛推俯仰（电机饱和）"
        if t < 35.0:
            return "★ GPS 多路径"
        return "触发 RTL"

    # --- 姿态：期望 vs 实际 ---
    n_att = int(T_END * FS_ATT)
    t_att = np.arange(n_att) / FS_ATT
    roll_sp = np.deg2rad(8.0) * np.sin(2 * math.pi * 0.20 * t_att)
    pitch_sp = np.deg2rad(5.0) * np.sin(2 * math.pi * 0.13 * t_att + 0.7)
    yaw_sp = np.deg2rad(20.0) * np.sin(2 * math.pi * 0.05 * t_att)

    # 实际姿态 = 期望的一阶延迟 + 跟踪误差；振动段误差被放大
    tau = 0.10
    k_lag = 1.0 - math.exp(-1.0 / (FS_ATT * tau))
    roll_f = np.zeros(n_att)
    pitch_f = np.zeros(n_att)
    for k in range(1, n_att):
        roll_f[k] = roll_f[k - 1] + (roll_sp[k] - roll_f[k - 1]) * k_lag
        pitch_f[k] = pitch_f[k - 1] + (pitch_sp[k] - pitch_f[k - 1]) * k_lag

    err_amp = np.full(n_att, np.deg2rad(0.8))
    err_amp[(t_att >= 15.0) & (t_att < 25.0)] = np.deg2rad(6.5)
    err_amp[(t_att >= 25.0) & (t_att < 28.0)] = np.deg2rad(3.5)
    err_sig = err_amp * np.sin(2 * math.pi * 3.1 * t_att + 0.3)
    roll_a = roll_f + err_sig
    pitch_a = pitch_f - 0.65 * err_sig

    for k in range(n_att):
        q_sp = euler_to_quat(roll_sp[k], pitch_sp[k], yaw_sp[k])
        q_a = euler_to_quat(roll_a[k], pitch_a[k], yaw_sp[k] - np.deg2rad(0.4))
        b.add_data("vehicle_attitude", ULOG_TOPICS["vehicle_attitude"],
                   {"timestamp": int(t_att[k] * 1e6) + 1000000, "q": q_a})
        b.add_data("vehicle_attitude_setpoint", ULOG_TOPICS["vehicle_attitude_setpoint"],
                   {"timestamp": int(t_att[k] * 1e6) + 1000000, "q_d": q_sp})

    # --- 电机输出 ---
    n_mot = int(T_END * FS_MOT)
    t_mot = np.arange(n_mot) / FS_MOT
    mot = np.zeros((n_mot, 4))
    for k in range(n_mot):
        t = t_mot[k]
        base = 0.52
        if t < 5.0:
            base = 0.0
        # 悬停时四路基本一致，带一点噪声
        d = rng.normal(0.0, 0.006, 4)
        if 15.0 <= t < 25.0:
            # ★ 桨叶损伤：对角两路要额外出力，另两路收油（力矩不平衡）
            d = np.array([0.10, -0.09, 0.10, -0.09]) + rng.normal(0.0, 0.010, 4)
        if 25.0 <= t < 28.0:
            # ★ 猛推俯仰：前两路顶到上限，后两路掉到下限
            d = np.array([0.48, 0.48, -0.45, -0.45])
        mot[k] = np.clip(base + d, 0.0, 1.0)

    for k in range(n_mot):
        b.add_data("actuator_motors", ULOG_TOPICS["actuator_motors"],
                   {"timestamp": int(t_mot[k] * 1e6) + 1000000, "control": mot[k]})

    # --- 振动指标（用 PX4 的真实公式从「虚拟加速度」算出来）---
    n_vib = int(T_END * FS_VIB)
    t_vib = np.arange(n_vib) / FS_VIB
    vib = np.zeros(n_vib)
    clip = np.zeros(n_vib, dtype=int)
    for k in range(n_vib):
        t = t_vib[k]
        if t < 5.0:
            vib[k] = 0.35
        elif t < 15.0:
            vib[k] = 0.55
        elif t < 25.0:
            # 桨叶损伤：指标跳到 4 倍上下，并开始出现削顶
            vib[k] = 2.4 + 0.35 * math.sin(2 * math.pi * 0.4 * t)
            if t > 19.0:
                clip[k] = int(12 + 6 * math.sin(2 * math.pi * 0.7 * t))
        elif t < 28.0:
            vib[k] = 3.1
            clip[k] = 41
        else:
            vib[k] = 0.9
    for k in range(n_vib):
        b.add_data("vehicle_imu_status", ULOG_TOPICS["vehicle_imu_status"],
                   {"timestamp": int(t_vib[k] * 1e6) + 1000000,
                    "accel_clipping": int(clip[k]),
                    "gyro_clipping": int(clip[k] // 3),
                    "accel_vibration_metric": float(vib[k]),
                    "gyro_vibration_metric": float(vib[k] * 0.35),
                    "delta_angle_coning_metric": float(vib[k] * 0.05)})

    # --- EKF 创新检验比 ---
    n_inn = int(T_END * FS_INN)
    t_inn = np.arange(n_inn) / FS_INN
    ratio = np.zeros((n_inn, 2))
    for k in range(n_inn):
        t = t_inn[k]
        r = 0.25 + 0.05 * math.sin(2 * math.pi * 0.3 * t)
        if 35.0 <= t < 39.0:
            # ★ GPS 多路径：横向创新检验比冲到 1 以上（> 1 = 被 EKF 拒绝）
            r = 1.35 + 0.45 * math.sin(2 * math.pi * 1.7 * (t - 35.0))
        ratio[k] = [r, 0.22 + 0.04 * math.cos(2 * math.pi * 0.4 * t)]
    for k in range(n_inn):
        b.add_data("estimator_innovation_test_ratios",
                   ULOG_TOPICS["estimator_innovation_test_ratios"],
                   {"timestamp": int(t_inn[k] * 1e6) + 1000000,
                    "gps_hpos": ratio[k]})

    # --- 状态 ---
    n_sta = int(T_END * FS_STA)
    t_sta = np.arange(n_sta) / FS_STA
    failsafe = np.array([1 if t >= 38.0 else 0 for t in t_sta])
    nav_state = np.array([3 if t >= 38.0 else 2 for t in t_sta], dtype=int)
    for k in range(n_sta):
        b.add_data("vehicle_status", ULOG_TOPICS["vehicle_status"],
                   {"timestamp": int(t_sta[k] * 1e6) + 1000000,
                    "nav_state": int(nav_state[k]), "failsafe": int(failsafe[k])})

    # --- 一条 DROPOUT ---
    b.add_dropout(12)

    raw = b.bytes()
    print()
    print("      共造出 %d 字节（%.1f kB）" % (len(raw), len(raw) / 1024.0))
    print("      前 16 字节（文件头）： " + " ".join("%02X" % x for x in raw[:16]))
    print("      紧跟其后（FLAG_BITS 头）： " +
          " ".join("%02X" % x for x in raw[16:19]))
    print("      字段类型名用的是 float 而不是 float32 —— 见 "
          "platforms/common/uORB/uORB.cpp:207")

    sub("③ 用自己写的解析器读回来")

    r = ULogReader(raw)
    rows = [
        ["文件头 magic 正确", "是" if r.header_ok else "否"],
        ["起始时间戳", "%d us" % r.header_ts],
        ["FORMAT 条数", "%d" % len(r.formats)],
        ["ADD_LOGGED_MSG 条数", "%d" % len(r.logged)],
        ["INFO 条数", "%d" % len(r.infos)],
        ["PARAMETER 条数", "%d" % len(r.params)],
        ["DROPOUT", "%s" % (["%d ms" % d for d in r.dropouts] or "无")],
    ]
    tab(rows, ["读出", "值"])

    rows = []
    for nm in ULOG_TOPICS:
        rows.append([nm, "%d" % len(r.series.get(nm, [])), "%d" % fields_size(ULOG_TOPICS[nm])])
    tab(rows, ["topic", "DATA 条数", "单条结构体字节数"])

    print()
    wrap("★ 自检：DATA 条数应当等于 采样率 × 时长。"
         "对不上就说明我们造日志的循环写错了 —— 这就是「造-解闭环」的价值。")
    print()
    print("      vehicle_attitude        %5d  = 20 Hz x 40 s = %d" %
          (len(r.series["vehicle_attitude"]), int(FS_ATT * T_END)))
    print("      actuator_motors         %5d  = 10 Hz x 40 s = %d" %
          (len(r.series["actuator_motors"]), int(FS_MOT * T_END)))
    print("      vehicle_imu_status      %5d  =  1 Hz x 40 s = %d" %
          (len(r.series["vehicle_imu_status"]), int(FS_VIB * T_END)))

    print()
    print("      解出来的第一条 vehicle_status（含 _padding 的话题）：")
    print("        nav_state = %d,  failsafe = %d" %
          (r.series["vehicle_status"][0]["nav_state"],
           r.series["vehicle_status"][0]["failsafe"]))
    wrap("★ _padding0 被解析器【跳过】了（unpack_values 里按名字前缀过滤）——"
         "这跟 orb_print_message_internal 的做法一致："
         "platforms/common/uORB/uORB.cpp:280-282 看到 _padding 就不打印。"
         "解析器必须知道它的存在（否则字节对不上），但不需要用它。")

    sub("④ ★★ 分析 A：振动 —— 从日志里能读出什么")

    wrap("先看日志里直接记下来的振动指标（vehicle_imu_status，1 Hz）：")
    print()
    rows = []
    for k in range(0, n_vib, 4):
        rows.append(["%5.1f" % t_vib[k], "%.2f" % vib[k], "%d" % clip[k], seg(t_vib[k])])
    tab(rows, ["t (s)", "accel_vibration_metric", "accel_clipping", "阶段"])

    print()
    wrap("★ 判据：accel_clipping 一旦非 0，说明加速度计【已经削顶】，"
         "此时振动指标本身就不可信了（削顶把高频成分砍掉了），"
         "而 EKF 的姿态和高度会开始漂。这条日志里 19 s 之后就开始削顶，"
         "比指标本身超限更早暴露问题。")

    print()
    wrap("现在把这个指标【拆开】，看它到底在量什么。源码：")
    print()
    print("      src/modules/sensors/vehicle_imu/VehicleIMU.cpp:740-747")
    print("        v[k] = 0.99 * v[k-1] + 0.01 * ||a[k] - a[k-1]||")
    print()
    wrap("第 ①步：稳态值。把递推式的两个系数看成 (1-a) 和 a（a=0.01），"
         "当输入 d[k] = ||a[k]-a[k-1]|| 平稳时，取期望：")
    print()
    print("        E[v] = (1-a)·E[v] + a·E[d]   =>   E[v] = E[d]")
    print()
    wrap("也就是说，稳态下它等于【相邻采样差值的范数】的滑动平均。")
    print()
    wrap("第 ②步：时间常数。从初值 0 开始，输入恒为 1：")
    print("        v[k] = 1 - 0.99^k")
    print("      令 0.99^n = 1/e，得 n = -1/ln(0.99) = %.1f 个采样。" %
          (-1.0 / math.log(0.99)))
    print("      SITL 里 IMU 数据 250 Hz（ROMFS/px4fmu_common/init.d-posix/"
          "px4-rc.simulator:4-5）")
    print("      于是时间常数 = %.1f / 250 = %.3f s —— 大约是一个 0.4 秒的滑窗。"
          % (-1.0 / math.log(0.99), (-1.0 / math.log(0.99)) / 250.0))

    print()
    wrap("第 ③步：对纯正弦输入，这个滑窗平均值是多少？设 a[k] = A·sin(ωk)，"
         "ω = 2πf/fs：")
    print("        a[k] - a[k-1] = A[sin(ωk) - sin(ω(k-1))]")
    print("                      = 2A·sin(ω/2)·cos(ωk - ω/2)        (和差化积)")
    print("        ||·|| 取绝对值后对时间平均：E|cos| = 2/π")
    print("        =>  E[v] = (4A/π)·sin(ω/2) = (4A/π)·sin(πf/fs)")
    print()
    wrap("★ 结论：它【正比于 sin(πf/fs)】，也就是随频率上升、在 Nyquist "
         "处饱和。低频振动几乎量不出来。所以它【不是】振动的 RMS 幅值，"
         "而是「高频变化率」。")

    print()
    print("      数值验证（fs = 250 Hz，A = 1 m/s²）：")
    fs = 250.0
    n = 20000
    rows = []
    for f in (5.0, 20.0, 50.0, 100.0, 125.0):
        k = np.arange(n)
        a = 1.0 * np.sin(2 * math.pi * f * k / fs)
        d = np.abs(a[1:] - a[:-1])
        meas = float(np.mean(px4_vibration_metric(d)[n // 2:]))
        pred = (4.0 * 1.0 / math.pi) * math.sin(math.pi * f / fs)
        rows.append(["%5.0f" % f, "%.4f" % meas, "%.4f" % pred,
                     "%+.2f%%" % ((meas - pred) / pred * 100.0)])
    tab(rows, ["f (Hz)", "仿真稳态值", "解析式 (4A/π)sin(πf/fs)", "误差"])

    wrap("误差都在 1% 以内（剩下的偏差来自 cos 的时间平均不是严格的 2/π，"
         "以及滑窗还没完全收敛）。解析式和源码的行为对上了，"
         "说明我们对这条公式的理解没跑偏。")

    sub("⑤ ★★ 分析 B：姿态跟踪误差")

    wrap("日志里有 vehicle_attitude（实际）和 vehicle_attitude_setpoint（期望），"
         "两条都是四元数。误差怎么算？不跳步地推一遍：")
    print()
    print("      设 R_a、R_sp 分别是实际和期望的 body->earth 旋转矩阵。")
    print("      定义 R_err := R_sp^T · R_a")
    print("        · 若两者相同，R_err = I，误差为 0 ✓")
    print("        · R_sp^T 把 earth 系映回期望机体系，再乘 R_a（机体系->earth），")
    print("          合起来就是【把实际机体系转到期望机体系】的旋转")
    print("      四元数对应：q_err = conj(q_sp) ⊗ q_a")
    print("      写成 R_err = R(θ, n)：q_err = (cos(θ/2), sin(θ/2)·n)")
    print("      => w_err = cos(θ/2)")
    print("      => θ = 2·arccos(|w_err|)   （取绝对值是因为 q 与 -q 是同一个旋转）")

    print()
    print("      另有一条完全独立的算法可以互相校验：")
    print("      由 trace(R) = 1 + 2cos(θ)，得 θ = arccos((trace(R_err) - 1)/2)")

    print()
    n_att = len(r.series["vehicle_attitude"])
    th = np.zeros(n_att)
    th_m = np.zeros(n_att)
    for k in range(n_att):
        qa = r.series["vehicle_attitude"][k]["q"]
        qs = r.series["vehicle_attitude_setpoint"][k]["q_d"]
        th[k], th_m[k] = attitude_error_rad(qa, qs)
    dmax = float(np.max(np.abs(th - th_m)))
    print("      两条算法在 %d 个采样点上的最大差：%.3e 弧度（应为 0）" % (n_att, dmax))

    print()
    rows = []
    for k in range(0, n_att, 20):
        rows.append(["%5.2f" % t_att[k], "%.2f" % np.rad2deg(th[k]), seg(t_att[k])])
    tab(rows, ["t (s)", "姿态误差 (deg)", "阶段"])

    print("      各阶段误差的均方根（RMS）与峰值：")
    rows = []
    for lo, hi, lab in ((0, 5, "地面待机"), (5, 15, "正常悬停"),
                        (15, 25, "桨叶损伤"), (25, 28, "猛推俯仰"),
                        (28, 35, "GPS 多路径"), (35, 40, "触发 RTL")):
        m = (t_att >= lo) & (t_att < hi)
        e = np.rad2deg(th[m])
        rows.append([lab, "%.2f" % math.sqrt(float(np.mean(e * e))), "%.2f" % float(np.max(e))])
    tab(rows, ["阶段", "误差 RMS (deg)", "误差峰值 (deg)"])

    print()
    wrap("★ 读法：正常悬停时误差 RMS 在 1 度上下（这是延迟造成的，"
         "20 Hz 采样 + 100 ms 姿态环时间常数）；振动段一下就上去了。"
         "所以「姿态跟踪误差变大」本身不告诉你是【振动】还是【增益不够】，"
         "必须和同一时刻的振动指标、电机输出对着看 —— 这就是日志分析的核心方法。")

    sub("⑥ ★★ 分析 C：电机输出饱和与不平衡")

    mot_log = np.array([s["control"] for s in r.series["actuator_motors"]])
    rows = []
    for lo, hi, lab in ((0, 5, "地面待机"), (5, 15, "正常悬停"),
                        (15, 25, "桨叶损伤"), (25, 28, "猛推俯仰"),
                        (28, 40, "返航")):
        m = (t_mot >= lo) & (t_mot < hi)
        mm = mot_log[m]
        rails = int(np.sum((mm >= 0.98) | (mm <= 0.02)))
        spread = float(np.mean(np.max(mm, axis=1) - np.min(mm, axis=1)))
        rows.append([lab, "%.3f" % float(np.mean(mm)), "%.3f" % spread,
                     "%d" % rails, "%.1f%%" % (100.0 * rails / mm.size)])
    tab(rows, ["阶段", "平均油门", "四路极差均值", "触限采样数", "触限比例"])

    print()
    wrap("★ 三条判据（从上往下依次排查）：")
    bullet([
        "① 触限比例 > 0：控制器想要的力矩/推力已经给不出来了。"
        "这时再调 PID 是没用的，得换桨、减重或者改限幅。",
        "② 四路极差：悬停时四路应当几乎一样。极差持续大于 0.05，"
        "通常说明重心偏了、某个电机/桨效率不一样。",
        "③ 对角两路一高两低（本日志振动段就是 1、3 高，2、4 低）："
        "典型的重心偏移补偿 —— 飞控在持续输出一个恒定力矩。"
        "这个恒定力矩会把一部分控制余量吃掉，遇到阵风就没有余量了。",
    ])

    sub("⑦ ★★ 分析 D：EKF 创新（innovation）与检验比（test ratio）")

    wrap("源码 src/modules/ekf2/EKF/ekf.h:1182-1195：")
    print()
    print("        test_ratio = innovation^2 / (gate^2 * innovation_variance)")
    print("        innovation_rejected = (test_ratio > 1)")
    print()
    wrap("两边开方就看得更清楚：")
    print("        |innovation| > gate * sqrt(innovation_variance)   =>  拒绝")
    print()
    wrap("innovation_variance 是【新息协方差】S = HPH^T + R，既包含状态的不确定度 "
         "（H P H^T），也包含观测噪声（R）。所以 gate 就是一个「几倍标准差」的门限："
         "gate = 3 表示 |innovation| 超过 3σ 才拒绝。")
    print()
    print("      ★ 日志里记的 test_ratio 已经开过平方根（ekf_helper.cpp:385-410 的")
    print("        sqrtf 调用），所以判据就是【看它有没有超过 1】。")

    print()
    rows = []
    for k in range(0, n_inn, 2):
        rows.append(["%5.1f" % t_inn[k], "%.2f" % ratio[k, 0], "%.2f" % ratio[k, 1],
                     "拒绝" if ratio[k, 0] > 1.0 else "通过", seg(t_inn[k])])
    tab(rows, ["t (s)", "gps_hpos 检验比", "gps_vpos 检验比", "水平位置", "阶段"])

    m = (t_inn >= 35.0) & (t_inn < 39.0)
    print("      多路径段（35-39 s）水平位置检验比最大值：%.2f，"
          "被拒绝的采样占比 %.0f%%" %
          (float(np.max(ratio[m, 0])), 100.0 * float(np.mean(ratio[m, 0] > 1.0))))
    wrap("★ 读法：检验比偶尔过 1 是正常的（GPS 本来就跳）。真正要警惕的是"
         "【持续】过 1 —— 那说明 EKF 在长时间拒绝 GPS，位置估计会退回惯导推算，"
         "几分钟后就会漂到不可用。PX4 会把这种情况反映到 failsafe_flags 的 "
         "global_position_invalid / local_position_accuracy_low 上，进而触发返航。")

    sub("⑧ 实验 2 小结")

    bullet([
        "ULog 的每一条消息都是「3 字节头 + msg_size 字节体」，"
        "msg_size 不含头；文件头固定 16 字节；FLAG_BITS 必须紧跟文件头。",
        "DATA 消息只带一个 uint16 的 msg_id，字段定义在文件开头的 FORMAT 里 —— "
        "所以【只要 FMT 表在，任何时候都能解码】，这也是 ULog 可以边飞边写的原因。",
        "日志里的振动指标不是 RMS，是「相邻采样差值的 LPF」，随频率上升；"
        "它的绝对值只有跟同类飞机对比才有意义。",
        "姿态误差要自己算（两条独立公式互验），电机输出要看「触限比例 + 四路极差」，",
        "EKF 创新检验比的判据是「有没有持续超过 1」。",
    ])


# ============================================================================
#  实验 3：★★ 手写一个 DataFlash 日志，再解析回来
# ============================================================================

# 真实的 DataFlash 消息定义（名字 / 格式串 / 列名 / 单位 直接来自源码）
#   libraries/AP_Logger/LogStructure.h:1217  FMT
#   libraries/AP_Logger/LogStructure.h:1225  PARM
#   libraries/AP_InertialSensor/LogStructure.h:122  VIBE
#   libraries/AP_AHRS/LogStructure.h:202  ATT
#   libraries/AP_Logger/LogStructure.h:1325  PSCN
#   libraries/AP_Logger/LogStructure.h:1268  PM
#
# ⚠️ **上面这些行号指的是「格式串/列名」的出处，不是「msgid」的出处。**
#    - `FMT=128`、`PARM=32` 是**真实值**（`LogStructure.h:1345` 的
#      `enum LogMessages` 里显式写了 `LOG_PARAMETER_MSG = 32`，
#      `LOG_FORMAT_MSG = 128` 还带注释 `must remain #128`）。
#    - 其余四条（201/202/203/204）是**本脚本自定义的自洽值**，
#      **不是** ArduPilot 的真实 id —— 真实 id 由那个枚举**按顺序累加**产生，
#      中间还夹着大量 `LOG_IDS_FROM_XXX` 成组占位，手工数不可靠。
#
#    ⇒ 这不影响本实验：我们自己造日志、自己解析，**只要 FMT 表和数据记录自洽**就行。
#      但读真机 `.bin` 日志时**必须用日志里的 FMT 表**，绝不能用这里的数字。
DF_FORMATS = [
    (128, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns"),
    (32, "PARM", "QNff", "TimeUS,Name,Value,Default"),
    (201, "VIBE", "QBfffI", "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip"),
    (202, "ATT", "QccccCCCCB", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw,AEKF"),
    (203, "PSCN", "Qffffffff", "TimeUS,TPN,PN,DVN,TVN,VN,DAN,TAN,AN"),
    (204, "PM", "QHHHIIHHIIIIII", "TimeUS,LR,NLon,NL,MaxT,Mem,Load,ErrL,IntE,ErrC,SPIC,I2CC,I2CI,Ex"),
]

# ArduPilot 的 VIBE 滤波器参数（libraries/AP_InertialSensor/AP_InertialSensor.h:8-9）
VIBE_FLOOR_HZ = 5.0      # AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ
VIBE_FILT_HZ = 2.0       # AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ


def lpf_alpha(dt, cutoff_hz):
    """ArduPilot 的一阶低通系数。

    libraries/Filter/LowPassFilter.cpp:39-41
        rc    = 1/(2*pi*fc)
        alpha = dt/(dt + rc)
        y    += (x - y) * alpha
    与 libraries/AP_Math/AP_Math.cpp:414-415 的 calc_lowpass_alpha_dt 一致。
    """
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (dt + rc)


def ap_vibe(accel, dt):
    """照着 AP_InertialSensor.cpp 复现 ArduPilot 的 VIBE 计算。

    libraries/AP_InertialSensor/AP_InertialSensor.cpp:2131-2142
        accel_filt = LPF_5Hz(accel)
        accel_diff = (accel - accel_filt); accel_diff **= 2   (逐轴平方)
        _accel_vibe_filter.apply(accel_diff, dt)               (LPF_2Hz)
    libraries/AP_InertialSensor/AP_InertialSensor.cpp:2163-2173
        vibe = sqrt(filtered)                                  (逐轴开方)
    """
    a1 = lpf_alpha(dt, VIBE_FLOOR_HZ)
    a2 = lpf_alpha(dt, VIBE_FILT_HZ)
    floor = 0.0
    sq = 0.0
    out = np.zeros(len(accel))
    for k in range(len(accel)):
        if k == 0:
            floor = accel[k]            # 首个采样直接灌初值（LowPassFilter.cpp:42-45）
            sq = 0.0
        floor += (accel[k] - floor) * a1
        diff = accel[k] - floor
        sq += (diff * diff - sq) * a2
        out[k] = math.sqrt(sq) if sq > 0 else 0.0
    return out


def wrap_360(x):
    """把角度折到 [0, 360)，对应 ArduPilot 的 wrap_360_cd。

    源码 AP_AHRS_Logging.cpp:61-62 写 ATT 的 yaw 字段时用的就是
        control_yaw : (uint16_t)wrap_360_cd(targets.z)
    —— ATT 的 yaw 是【uint16 且恒为正】，所以日志里永远读不到负的偏航角。
    """
    return x % 360.0


def exp3():
    head("实验 3  ★★ 手写一个 DataFlash 日志，再解析回来")

    sub("① 先看清楚每个字节该写什么")

    rows = [
        ["记录头", "3", "0xA3 0x95 + uint8 msgid（两个魔数：LogStructure.h:121-122）"],
        ["FMT", "89", "3 头 + type/len/name[4]/format[16]/labels[64]（:176-183）"],
        ["PARM", "35", "3 头 + uint64 time_us + char name[16] + float value + float default"],
        ["VIBE", "28", "3 头 + Q + B + 3f + I（AP_InertialSensor/LogStructure.h:106-112）"],
        ["ATT", "28", "3 头 + Q + 4c + 4C + B（AP_AHRS/LogStructure.h:202）"],
    ]
    tab(rows, ["段", "字节数", "内容"])

    print()
    wrap("★ 两个必须记住的差异：")
    bullet([
        "1) DataFlash 是【大端】（网络序）。同一个 float 1.0，"
        "ULog 里是 00 00 80 3F，DataFlash 里是 3F 80 00 00。",
        "2) FMT 里的 length 字段是【含 3 字节头】的总长度，"
        "和 ULog 的 msg_size 恰好相反（LogFile.cpp:35  pkt.length = s->msg_len）。",
    ])

    sub("② 造日志")

    d = DataFlashBuilder()
    print("      先写 FMT 描述 FMT 自己（这是自举的关键）：")
    for mid, nm, fmt, labels in DF_FORMATS:
        d.add_fmt(mid, nm, fmt, labels)
        if nm == "FMT":
            print("        msgid=%d  name=%s  format=%s  length=%d"
                  % (mid, nm, fmt, 3 + df_record_size(fmt)))
    print("      再写 FMT 描述其余消息：")
    for mid, nm, fmt, labels in DF_FORMATS[1:]:
        print("        msgid=%3d  name=%-5s format=%-14s length=%2d  labels=%s"
              % (mid, nm, fmt, 3 + df_record_size(fmt), labels))

    print()
    wrap("★ 说明：真实固件里 msgid 由 libraries/AP_Logger/LogStructure.h:1345 的 "
         "LogMessages 枚举顺序决定（只有 FMT=128、PARM=32 是写死的）。"
         "本实验给其余消息分配 201~204 这些空闲 id —— 反正解析器只看 FMT 表，"
         "id 是多少完全不影响能不能解开。这正好证明了自描述格式的好处。")

    d.add_parm("INS_ACCEL_FILTER", 20.0)
    d.add_parm("INS_LOG_BAT_MASK", 1.0)
    d.add_parm("LOG_BITMASK", 176126.0)

    # ---- 造数据 ------------------------------------------------------------
    rng = np.random.default_rng(20240602)
    fs_att = 10.0
    n_att = int(T_END * fs_att)
    t_att = np.arange(n_att) / fs_att

    roll_sp = 8.0 * np.sin(2 * math.pi * 0.20 * t_att)
    pitch_sp = 5.0 * np.sin(2 * math.pi * 0.13 * t_att + 0.7)
    yaw_sp = 20.0 * np.sin(2 * math.pi * 0.05 * t_att)
    err = np.where((t_att >= 15.0) & (t_att < 25.0), 6.5, 0.8) * \
        np.sin(2 * math.pi * 3.1 * t_att + 0.3)
    roll_a = roll_sp + err
    pitch_a = pitch_sp - 0.65 * err
    yaw_a = yaw_sp - 0.4

    for k in range(n_att):
        t = t_att[k]
        rp = abs(float((roll_a[k] - yaw_a[k]) * 0.4))
        # ATT 的 c 类型是 int16 * 100，所以这里先乘 100 再取整（写入方要做的事）。
        # ErrRP/ErrYaw 是 uint16（格式字符 C），源码 AP_AHRS_Logging.cpp:63-64 取的是
        # get_error_rp()/get_error_yaw() 的【幅值】，所以这里也用 abs()。
        d.add("ATT", TimeUS=int(t * 1e6),
              DesRoll=int(round(roll_sp[k] * 100)), Roll=int(round(roll_a[k] * 100)),
              DesPitch=int(round(pitch_sp[k] * 100)), Pitch=int(round(pitch_a[k] * 100)),
              # yaw 是 uint16：源码先 wrap_360 再 ×100（AP_AHRS_Logging.cpp:61-62）
              DesYaw=int(round(wrap_360(yaw_sp[k]) * 100)),
              Yaw=int(round(wrap_360(yaw_a[k]) * 100)),
              ErrRP=int(round(rp * 100)),
              ErrYaw=int(round(abs(yaw_sp[k] - yaw_a[k]) * 100)),
              AEKF=0)

        # PSCN：位置控制器
        d.add("PSCN", TimeUS=int(t * 1e6),
              TPN=0.0, PN=0.0, DVN=0.0, TVN=0.0, VN=0.0, DAN=0.0, TAN=0.0, AN=0.0)

    # VIBE：★ 先用真实滤波器把「原始加速度」加工成 VIBE，再写进日志
    fs_vib = 1000.0                      # 加速度计后端采样率（参数，非定值）
    dt = 1.0 / fs_vib
    n_raw = int(T_END * fs_vib)
    kk = np.arange(n_raw)
    A_vib = np.where((kk / fs_vib >= 15.0) & (kk / fs_vib < 25.0), 6.0, 1.0)
    accel_raw = A_vib * np.sin(2 * math.pi * 100.0 * kk / fs_vib) + \
        0.05 * rng.normal(0.0, 1.0, n_raw)
    vibe_series = ap_vibe(accel_raw, dt)

    n_vibe = int(T_END * 1.0)
    t_vibe = np.arange(n_vibe)
    for k in range(n_vibe):
        idx = int(round((k + 1) * fs_vib)) - 1
        v = float(vibe_series[idx])
        d.add("VIBE", TimeUS=int(k * 1e6), IMU=0,
              VibeX=v, VibeY=v * 0.92, VibeZ=v * 1.15,
              Clip=int(30 if 19 <= k < 25 else 0))

    # PM：性能监视器
    for k in range(int(T_END / 2.0)):
        d.add("PM", TimeUS=int(k * 2e6), LR=int(0.02 * 1000), NLon=0, NL=0,
              MaxT=8 + k % 3, Mem=200000, Load=280 + k % 40,
              ErrL=0, IntE=0, ErrC=0, SPIC=0, I2CC=0, I2CI=0, Ex=0)

    raw = d.bytes()
    print()
    print("      共造出 %d 字节（%.1f kB）" % (len(raw), len(raw) / 1024.0))
    print("      第一个字节：%02X %02X %02X（0xA3 0x95 0x80，即 FMT 记录）"
          % (raw[0], raw[1], raw[2]))
    print("      紧接着 name 字段：%s" % raw[5:9].split(b"\x00")[0].decode("ascii"))

    sub("③ 用自己写的解析器读回来")

    rd = DataFlashReader(raw)
    rows = []
    for nm, cnt in rd.records.items():
        _, fmt, labels = rd.fmt[rd.name2id.get(nm, 0)] if nm in rd.name2id else (nm, "", "")
        rows.append([nm, "%d" % cnt, "%d" % df_record_size(fmt) if fmt else "-"])
    tab(rows, ["消息", "条数", "单条字节数"])

    print()
    print("      解出来的 param：")
    for k, v in rd.params.items():
        print("        %-20s = %s" % (k, v))

    print()
    print("      解出来的前 3 条 ATT（注意 c 类型已经被乘回 0.01："
          "LogStructure.h:21 「c : int16_t * 100」）：")
    for rec in rd.series["ATT"][:3]:
        print("        t=%8.2f  DesRoll=%+6.2f  Roll=%+7.2f  ErrRP=%+6.2f" %
              (rec["TimeUS"] / 1e6, rec["DesRoll"], rec["Roll"], rec["ErrRP"]))
    print("      原始字节里 Roll 是整数 100 倍 —— 这是 DataFlash 省空间的常规手段。")

    sub("④ ★★ VIBE 到底是多少？—— 亲手复现 ArduPilot 的算法")

    wrap("源码 libraries/AP_InertialSensor/AP_InertialSensor.cpp:2131-2142：")
    print()
    print("        accel_filt = _accel_vibe_floor_filter[i].apply(accel, dt);   // 5 Hz LPF")
    print("        accel_diff = accel - accel_filt;")
    print("        accel_diff.x *= accel_diff.x;   // 逐轴平方")
    print("        accel_diff.y *= accel_diff.y;")
    print("        accel_diff.z *= accel_diff.z;")
    print("        _accel_vibe_filter[i].apply(accel_diff, dt);                  // 2 Hz LPF")
    print()
    print("    libraries/AP_InertialSensor/AP_InertialSensor.cpp:2163-2173：")
    print("        vibe = sqrt(filtered)                                         // 逐轴开方")
    print()
    wrap("三步连起来的物理含义：")
    print("        ① x - LPF_5Hz(x)   是一路【高通】：把 5 Hz 以下的低频运动（真的机动）去掉")
    print("        ② 平方 + LPF_2Hz   是求【均方】")
    print("        ③ 开方             => 高频部分的【RMS】")
    print()
    wrap("滤波器形式是标准一阶 IIR：alpha = dt/(dt + 1/(2πfc))，"
         "源码 libraries/Filter/LowPassFilter.cpp:39-41。")

    print()
    print("      数值验证（fs = 1000 Hz，A = 1 m/s² 的正弦，取稳态段）：")
    rows = []
    for f in (2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0):
        nn = int(4.0 / dt)
        s = np.sin(2 * math.pi * f * np.arange(nn) * dt)
        y = ap_vibe(s, dt)
        meas = float(np.mean(y[nn // 2:]))
        rows.append(["%5.0f" % f, "%.4f" % meas, "%.4f" % (1.0 / math.sqrt(2.0)),
                     "%.2f" % (meas / (1.0 / math.sqrt(2.0)))])
    tab(rows, ["f (Hz)", "VIBE 实测", "A/sqrt(2) = 0.7071", "比值"])

    print()
    wrap("★ 读懂这张表：")
    bullet([
        "f = 2 Hz：被 5 Hz 高通滤掉大半，比值只有 0.37 —— "
        "也就是说【低频机动不会被记成振动】，这正是设计意图。",
        "f ≥ 50 Hz：比值稳定在 1.0 附近 —— 此时 VIBE 就是高频部分的 RMS 幅值，"
        "单位是 m/s²，有明确的物理意义。",
        "所以 ArduPilot 文档里说的「VIBE 超过 30 m/s² 要警惕」是一个"
        "**可以直接和加速度量纲对比**的数。",
    ])

    sub("⑤ ★★ 同一个振动，两种日志给出完全不同的数字")

    wrap("这是本课最重要的一张表：把【同一个物理振动】同时喂给两套指标。"
         "A = 1 m/s²，PX4 侧采样率 250 Hz（SITL 的 IMU_INTEG_RATE），"
         "ArduPilot 侧滤波器采样率 1000 Hz：")
    print()
    rows = []
    for f in (5.0, 20.0, 50.0, 100.0, 200.0):
        # PX4 指标
        fs_px4 = 250.0
        nn = 20000
        a = np.sin(2 * math.pi * f * np.arange(nn) / fs_px4)
        dv = np.abs(a[1:] - a[:-1])
        px4_v = float(np.mean(px4_vibration_metric(dv)[nn // 2:]))
        # ArduPilot VIBE
        nn2 = int(4.0 / dt)
        y = ap_vibe(np.sin(2 * math.pi * f * np.arange(nn2) * dt), dt)
        ap_v = float(np.mean(y[nn2 // 2:]))
        rows.append(["%5.0f" % f, "%.3f" % px4_v, "%.3f" % ap_v,
                     "%.2f" % (px4_v / ap_v if ap_v > 1e-9 else float("inf"))])
    tab(rows, ["f (Hz)", "PX4 accel_vibration_metric", "ArduPilot VIBE (m/s²)", "比值"])

    print()
    wrap("★ 结论（这一条能帮你少走很多弯路）：")
    bullet([
        "同一个振动，PX4 给的数从 0.08 一路涨到 1.23（20 Hz→100 Hz 是 0.32→1.23），"
        "ArduPilot 给的数在 20 Hz 以上稳定在 ≈0.70（5 Hz 那次被高通压到 0.50）。",
        "原因：PX4 量的是【相邻采样差值】（随频率上升，反比于采样率），"
        "ArduPilot 量的是【高通后的 RMS】（与频率无关，单位是 m/s²）。",
        "★ 200 Hz 那一行是个反直觉的坑：fs = 250 Hz 时 200 Hz 的正弦会【混叠】"
        "到 50 Hz 上，PX4 的数于是掉回 0.761 —— 采样率不够，会把高频振动"
        "报成一个偏小的数。看振动之前先确认采样率够不够（≥ 2 倍最高关注频率）。",
        "所以：★ 两个数字【不能互相换算】，也【不能跨平台比较】。"
        "网上说「我的振动是 2」这句话，不说是哪个平台的，等于没说。",
        "★ 同一个平台内部比较才有意义：看趋势、看相对变化、看它和你自己"
        "历史日志的对比。",
    ])

    sub("⑥ 实验 3 小结")

    bullet([
        "DataFlash 没有文件头，靠 FMT 自描述；FMT 描述 FMT 自己，实现自举。",
        "字节序是大端，FMT.length 含 3 字节头 —— 这两条和 ULog 恰好相反。",
        "c/C/e/E 这类「定点 ×100」的格式字符是为了省空间；N/n/Z 是定长字符串。",
        "VIBE = sqrt(LPF_2Hz((accel - LPF_5Hz(accel))^2))，是高频部分的 RMS，"
        "量纲是 m/s²，可以和加速度直接比。",
        "PX4 与 ArduPilot 的振动指标【物理含义不同】，绝不能跨平台比较数字。",
    ])


# ============================================================================
#  实验 4：★★ 日志驱动的诊断 + FailSafe 触发与动作
# ============================================================================

# PX4 的 Action 枚举（src/modules/commander/failsafe/framework.h:52-70）
# ★ 注释原文：「Actions further down take precedence」—— 枚举值越大，优先级越高
PX4_ACTIONS = [
    (0, "None", "什么都不做"),
    (1, "Warn", "只发警告（蜂鸣 + 地面站文字）"),
    (2, "FallbackPosCtrl", "降级到位置控制（不能手动打杆）"),
    (3, "FallbackAltCtrl", "降级到高度控制"),
    (4, "FallbackStab", "降级到自稳"),
    (5, "Hold", "原地悬停/盘旋"),
    (6, "RTL", "自动返航"),
    (7, "Land", "原地降落"),
    (8, "Descend", "原地下降（固定翼常用）"),
    (9, "Disarm", "空中直接上锁（只在起飞阶段用）"),
    (10, "Terminate", "终止飞行（停桨/开伞）"),
]

# ArduPilot Copter 的优先级表（ArduCopter/Copter.h:651-659）
# ★ 数组顺序 = 优先级顺序，索引越小优先级越高；-1 是哨兵
#   注释见 libraries/AP_BattMonitor/AP_BattMonitor.h:308
#     「array of failsafe priorities, sorted highest to lowest priority,
#       -1 indicates no more entries」
AP_PRIORITIES = [
    (5, "TERMINATE", "终止飞行"),
    (1, "LAND", "原地降落"),
    (2, "RTL", "自动返航"),
    (4, "SMARTRTL_LAND", "智能返航并降落"),
    (3, "SMARTRTL", "智能返航"),
    (0, "NONE", "什么都不做"),
]

# PX4 的 ClearCondition（framework.h:72-77）
PX4_CLEAR = [
    ["WhenConditionClears", "触发条件消失就解除"],
    ["OnModeChangeOrDisarm", "条件消失【且】切模式或上锁才解除"],
    ["OnDisarm", "条件消失【且】上锁才解除"],
    ["Never", "永不解除，必须重启"],
]

# PX4 的 Cause（framework.h:79-89）
PX4_CAUSES = [
    "Generic", "ManualControlLoss", "GCSConnectionLoss", "BatteryLow",
    "BatteryCritical", "BatteryEmergency", "RemainingFlightTimeLow",
]


def px4_pick(active):
    """PX4 的选法：framework.cpp:451  if (cur_action.action > selected_action)"""
    best = 0
    for v, _, _ in PX4_ACTIONS:
        if v in active and v > best:
            best = v
    return best


def ap_pick(active):
    """ArduPilot 的选法：AP_BattMonitor.cpp:826-833 沿优先级表从头往下找。"""
    for idx, (v, _, _) in enumerate(AP_PRIORITIES):
        if v in active:
            return idx, v
    return -1, None


def exp4():
    head("实验 4  ★★ 日志驱动的诊断 + FailSafe 触发与动作")

    sub("① 从日志症状到根因：一张可执行的排查表")

    rows = [
        ["accel_clipping 非 0", "振动导致加速度计削顶", "减振、换桨、查桨平衡"],
        ["accel_vibration_metric 突增", "桨叶损伤 / 电机轴承 / 机架共振",
         "加机械阻尼、改滤波（INS_ACCEL_FILTER）"],
        ["姿态误差 RMS 变大 + 振动同时变大", "振动串进了姿态环", "先治振动，别调 PID"],
        ["姿态误差 RMS 变大 + 振动正常", "增益偏低 / 延迟偏大 / 重心偏",
         "查 RATE 环增益、查日志时间戳间隔"],
        ["电机触限比例 > 0", "动力余量不足", "换桨减重，或放宽 MPC_THR_MAX"],
        ["四路极差持续 > 0.05", "重心偏 / 某电机效率低", "配平重心，单独测电机拉力"],
        ["对角两路一高一低", "飞控在持续补偿一个恒定力矩", "先配平，再看是否要调 trim"],
        ["EKF 检验比持续 > 1", "某个传感器被长时间拒绝", "换传感器/换位置/查干扰源"],
        ["日志里有 DROPOUT", "SD 卡写入跟不上", "换高速卡，降日志频率"],
    ]
    tab(rows, ["日志里看到的", "可能的根因", "下一步做什么"])

    print()
    wrap("★ 这张表的用法是【从上往下查】，因为上面几条会污染下面几条的证据："
         "振动会让姿态误差变大、让电机输出抖动、让 EKF 拒绝观测。"
         "如果先看 EKF 再看振动，很容易误判成「传感器坏了」。")

    sub("② 把实验 2 和实验 3 的日志按这张表过一遍")

    rows = [
        ["实验 2 ULog", "19 s 起 accel_clipping > 0", "★ 振动削顶", "先治振动"],
        ["实验 2 ULog", "15-25 s 姿态误差 RMS 5.5°（正常段 0.98°）", "振动串进姿态环",
         "同上，不要调 PID"],
        ["实验 2 ULog", "25-28 s 触限比例高，四路 1、3 高 2、4 低", "动力余量 + 重心偏",
         "配平并检查余量"],
        ["实验 2 ULog", "35-39 s gps_hpos 检验比 > 1", "GPS 多路径 → EKF 拒绝",
         "换位置/加屏蔽，等它自己恢复"],
        ["实验 2 ULog", "38 s 起 failsafe=1（vehicle_status）", "FailSafe 已触发", "看它是哪个动作"],
        ["实验 3 DataFlash", "VIBE 从 0.70 跳到 4.17（15-25 s）", "同一件事的另一种表达",
         "单位是 m/s²，可直接和 30 比"],
        ["实验 3 DataFlash", "PM.Load 稳定在 280-320", "CPU 负载健康", "无需处理"],
    ]
    tab(rows, ["来源", "日志里的证据", "判读", "结论"])

    sub("③ PX4 的 FailSafe：输入是 flags，输出是一个动作")

    wrap("PX4 的 FailSafe 是一个【状态机】，它的输入不是传感器，"
         "而是一组布尔标志 —— msg/FailsafeFlags.msg。这些标志由健康检查"
         "（HealthAndArmingChecks）和 arming 检查写进去。")
    print()
    print("      FailsafeFlags.msg 里的关键字段（msg/FailsafeFlags.msg:25-57）：")
    for s in ("angular_velocity_invalid", "attitude_invalid", "local_position_invalid",
              "global_position_invalid", "manual_control_signal_lost",
              "gcs_connection_lost", "battery_warning", "battery_unhealthy",
              "geofence_breached", "vtol_fixed_wing_system_failure",
              "wind_limit_exceeded", "flight_time_limit_exceeded",
              "local_position_accuracy_low", "fd_critical_failure",
              "fd_imbalanced_prop", "fd_motor_failure"):
        print("        bool %s" % s)

    print()
    wrap("★ 设计要点：把「条件」和「动作」解耦。条件只负责置一个 bool，"
         "动作由状态机按优先级选一个。所以改动作不用改检查逻辑，"
         "加检查也不用改状态机。")

    sub("④ 谁赢？PX4 用「枚举值最大」，ArduPilot 用「优先级表下标最小」")

    print("      PX4 的 Action 枚举（framework.h:52-70，注释：Actions further down take precedence）：")
    rows = []
    for v, nm, desc in PX4_ACTIONS:
        rows.append([v, nm, desc])
    tab(rows, ["枚举值", "名字", "含义"])

    print()
    print("      选法（framework.cpp:441-460）：")
    print("        for (...) { if (cur_action.action > selected_action) selected_action = cur_action.action; }")
    print("      即 max(枚举值)。这是一条【全序】，由枚举顺序保证。")

    print()
    print("      ArduPilot 的优先级表（ArduCopter/Copter.h:651-659）：")
    rows = []
    for idx, (v, nm, desc) in enumerate(AP_PRIORITIES):
        rows.append([idx, v, nm, desc])
    tab(rows, ["表下标", "枚举值", "名字", "含义"])

    print()
    print("      选法（libraries/AP_BattMonitor/AP_BattMonitor.cpp:826-842）：")
    print("        priority = 0;")
    print("        while (_failsafe_priorities[priority] != -1) {")
    print("            if (_failsafe_priorities[priority] == action) break;")
    print("            priority++;")
    print("        }")
    print("        if (priority <= _highest_failsafe_priority) { 触发 }")
    print("      即「表下标越小优先级越高」，而且【只允许升级、不允许降级】")
    print("      （_highest_failsafe_priority 初值 INT8_MAX，见 :431）。")

    print()
    wrap("★ 两种设计的对比：PX4 把优先级【编进枚举值】，靠 C++ 的整数序；"
         "ArduPilot 把优先级做成【一张数据表】，顺序就是语义。"
         "后者更灵活（可以插入新动作而不破坏顺序），前者更简单（不需要额外的表）。")

    print()
    print("      现在验证一个关键问题：ArduPilot 的枚举值在优先级表里是单调的吗？")
    print("      （如果单调，就可以用 max(枚举值) 代替查表）")
    bad = []
    for i in range(len(AP_PRIORITIES) - 1):
        a = AP_PRIORITIES[i]
        c = AP_PRIORITIES[i + 1]
        if a[0] < c[0]:
            bad.append((i, a[1], a[0], c[1], c[0]))
    rows = []
    for i, n1, v1, n2, v2 in bad:
        rows.append(["%d < %d" % (i, i + 1), "%s(%d)" % (n1, v1), "%s(%d)" % (n2, v2)])
    if rows:
        tab(rows, ["表下标", "高优先", "低优先"])
    print()
    print("      共发现 %d 处「高优先级的枚举值反而更小」。" % len(bad))
    wrap("★ 结论：不单调！所以如果 ArduPilot 用 max(枚举值) 来选，"
         "LAND(1) 会输给 RTL(2)、RTL(2) 会输给 SMARTRTL(3)，和设计意图正好相反。"
         "**优先级顺序只能从表里读，不能从枚举值推。**")

    sub("⑤ 数值演练：同一时刻有多个动作，谁被执行？")

    cases = [
        ("电池临界 + 遥控丢失 + 地理围栏越界", [1, 6, 6]),
        ("遥控丢失 + 返航中电池又进紧急", [6, 7]),
        ("飞控自检严重故障 + 任务失败", [6, 10]),
        ("电机不平衡 + 遥控丢失", [4, 5, 6]),
        ("只有警告", [1]),
    ]
    rows = []
    for name, acts in cases:
        pv = px4_pick(acts)
        pname = dict((v, nm) for v, nm, _ in PX4_ACTIONS)[pv]
        # ArduPilot：把 PX4 的动作近似映射到 ArduPilot 的等价物上
        amap = {1: 0, 6: 2, 7: 1, 10: 5, 5: 0, 4: 0}
        aacts = [amap.get(a, 0) for a in acts]
        idx, av = ap_pick(aacts)
        aname = AP_PRIORITIES[idx][1] if idx >= 0 else "-"
        rows.append([name, ",".join(str(a) for a in acts), pname,
                     ",".join(str(a) for a in aacts), aname])
    tab(rows, ["场景", "PX4 动作集", "PX4 选中", "AP 动作集", "AP 选中"])

    sub("⑥ 升级不允许降级：一个真实会咬人的细节")

    wrap("ArduPilot 的 _highest_failsafe_priority 只会变小（优先级只会变高），"
         "所以【低优先级的动作无法覆盖高优先级的】。模拟两块电池先后报警：")
    print()
    order = []

    def simulate(seq):
        """seq: [(电池名, 动作枚举值)]，返回触发了哪些。"""
        highest = 127                      # AP_BattMonitor.cpp:431  INT8_MAX
        fired = []
        for who, act in seq:
            idx, _ = ap_pick([act])
            if idx >= 0 and idx <= highest:
                fired.append((who, AP_PRIORITIES[idx][1], idx))
                highest = idx
            else:
                order.append((who, AP_PRIORITIES[idx][1], highest))
        return fired

    seq = [("电池1", 1), ("电池2", 2)]     # 1 -> LAND，2 -> RTL
    fired = simulate(seq)
    print("      顺序：先报 LAND（表下标 1），再报 RTL（表下标 2）")
    for who, nm, idx in fired:
        print("        %s -> 触发 %s（下标 %d）" % (who, nm, idx))
    for who, nm, highest in order:
        print("        %s -> %s（下标 2）被【忽略】：2 <= %d 不成立" % (who, nm, highest))

    print()
    wrap("再看反过来的顺序：")
    order.clear()
    seq = [("电池1", 2), ("电池2", 1)]
    fired = simulate(seq)
    print("      顺序：先报 RTL（表下标 2），再报 LAND（表下标 1）")
    for who, nm, idx in fired:
        print("        %s -> 触发 %s（下标 %d）" % (who, nm, idx))

    print()
    print("      PX4 侧的等价逻辑在 framework.cpp:462-466（defer）和 :426-431：")
    print("        if (_selected_action == Action::Terminate) { // Terminate never clears")
    print("      以及 Action::Terminate / Action::Disarm 的 clear_condition 被设成")
    print("      ClearCondition::Never（failsafe.cpp:68、failsafe.cpp:111）。")

    print()
    print("      解除条件（ClearCondition，framework.h:72-77）：")
    rows = [[a, b] for a, b in PX4_CLEAR]
    tab(rows, ["名字", "什么时候解除"])

    sub("⑦ FailSafe 的动作是怎么被执行的")

    wrap("状态机只输出一个 Action，真正执行的是 ModeManagement（commander）：")
    print()
    print("      framework.cpp:684  if (enabled && _selected_action > Action::Warn) {")
    print("      framework.h:154    static uint8_t modeFromAction(const Action &action, ...)")
    print()
    wrap("也就是说，FailSafe 不改控制律，它只是【切换飞行模式】。"
         "RTL 是 navigator 里的一个模式，Land 也是一个模式。"
         "这样 FailSafe 永远不用碰控制器代码 —— 这也是「条件/动作解耦」的延伸。")

    print()
    print("      对应的日志证据（本课实验 2 里就写进去了）：")
    print("        vehicle_status.nav_state   模式号")
    print("        vehicle_status.failsafe    非 0 表示 FailSafe 正在生效")
    print("        failsafe_flags             每个条件当前的布尔值")
    print("        （前两个是 add_default_topics 的 :137 加的，")
    print("          failsafe_flags 是 :68 加的）")
    print()
    wrap("★ 事后分析一次 FailSafe 事件，正确的顺序是："
         "先看 failsafe_flags 找出【哪个条件】被置位，"
         "再看 vehicle_status.nav_state 【切到了哪个模式】，"
         "最后回到振动/电机/EKF 的日志里找【条件为什么被置位】。")

    sub("⑧ 本课收束")

    print()
    print("      第 21 课  PX4 怎么组织代码        -> 模块 + uORB")
    print("      第 22 课  ArduPilot 怎么组织代码  -> 任务表 + 库")
    print("      第 23 课  怎么验证（SITL）        -> 能测逻辑，测不了物理")
    print("      第 24 课  怎么发现真机的问题      -> 日志 + FailSafe（本课）")
    print()
    wrap("★ 一句话收尾：SITL 给你的是【可复现的假数据】，"
         "日志给你的是【不可复现的真数据】。"
         "前者让你敢改代码，后者让你知道该改什么。")


# ============================================================================
#  入口
# ============================================================================

def main():
    args = sys.argv[1:]
    if not args:
        which = ["1", "2", "3", "4"]
    else:
        which = args

    table = {"1": exp1, "2": exp2, "3": exp3, "4": exp4}
    for a in which:
        if a not in table:
            print("用法：python lesson24_log_analysis.py [1|2|3|4]（不带参数 = 全部）")
            return 1
        table[a]()

    print()
    print("=" * W)
    print("  完成。")
    print("=" * W)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
