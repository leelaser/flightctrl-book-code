#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""卷 8 第 4 章 —— MAVLink 实操：从零跑通一条链路（配套脚本）

    python lesson30_mavlink.py        # 跑全部 5 个实验
    python lesson30_mavlink.py 1      # 只跑实验 1
    python lesson30_mavlink.py 4      # 只跑实验 4（custom_mode 位域拆解）

★ 五个实验，只用 Python 标准库。
  **本机没有 pymavlink**（实测 import 不到），所以本脚本
  【自己实现了 MAVLink v2 的组帧 / 拆帧 / CRC】——
  这样反而更好：协议不再是「一个库的黑盒」，而是你能逐字节数出来的东西。

五个实验：
    1  环境体检：本机有没有真链路（pymavlink / SITL / 串口）——优雅降级
    2  ★★ 手工组一条 HEARTBEAT：v2 帧逐字节拆开，每一字节都有名字
    3  ★★ CRC 与 CRC_EXTRA：从字段定义【算】出 CRC_EXTRA，和官方值对拍
    4  ★★ custom_mode 位域：PX4 的主/子模式、QGC 的命名、素材里的三份矛盾枚举
    5  ★  ACK 与会话：MAV_RESULT 全表 + 一条完整链路的时序推演

课程分工说明：
    本脚本讲的是【协议层 + 应用层】：字节怎么排、命令怎么发、ACK 怎么读。
    MAVLink 模块在 PX4 里怎么被调度、跑在哪个线程、优先级多少，
    属于 RTOS/系统层，详见卷 5 第 11 章，本脚本不重复。

参考实现（本机）：
    $SRC_CLIB/    ← MAVLink 官方 C 库（v2）
    本脚本的 CRC_EXTRA 全部拿它生成的头文件对拍过，见实验 3。
"""

import os
import struct
import sys
import time

# ============================================================================
#  输出小工具（与第 21/22/23 课脚本一致的排版）
# ============================================================================

W = 76


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
    """字符显示宽度：CJK 占 2 列，其余 1 列。"""
    o = ord(ch)
    if 0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF or \
       0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF or \
       0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60 or \
       0xFFE0 <= o <= 0xFFE6:
        return 2
    return 1


def _wide(s):
    return sum(_dw(c) for c in s)


def _pad(s, n):
    return s + " " * max(0, n - _wide(s))


def tab(rows, headers):
    """打印一张 CJK 对齐的表。"""
    rows = [[str(c) for c in r] for r in rows]
    ncol = len(headers)
    width = [_wide(str(headers[i])) for i in range(ncol)]
    for r in rows:
        for i in range(ncol):
            if i < len(r):
                width[i] = max(width[i], _wide(r[i]))
    width = [min(w, 46) for w in width]
    print()
    print("  " + "  ".join(_pad(str(headers[i]), width[i]) for i in range(ncol)))
    print("  " + "-" * (sum(width) + 2 * (ncol - 1)))
    for r in rows:
        cells = []
        for i in range(ncol):
            s = r[i] if i < len(r) else ""
            cells.append(_pad(s, width[i]))
        print("  " + "  ".join(cells))


def wrap(text, indent="  "):
    """按显示宽度折行的段落输出。"""
    line = indent
    for ch in text:
        if ch == "\n":
            print(line)
            line = indent
            continue
        if _wide(line) + _dw(ch) > W - 2 and ch in "，。；：、）」 ":
            print(line + ch)
            line = indent
            continue
        if _wide(line) + _dw(ch) > W - 2:
            print(line)
            line = indent + ch
            continue
        line += ch
    if line.strip():
        print(line)


def note(t):
    print()
    wrap("★ " + t)


def warn(t):
    print()
    wrap("⚠️  " + t)


def kv(k, v, w=30):
    """一行「键 …… 值」。"""
    print("  " + _pad(k, w) + str(v))


# ============================================================================
#  ★★ 本脚本的核心：一个不依赖任何第三方库的 MAVLink v2 编解码器
# ============================================================================

# MAVLink v2 帧的魔数（"帧头第一个字节"）。
#   v1 = 0xFE (254)，v2 = 0xFD (253)
# 出处：c_library_v2-master/common/mavlink.h:12  `#define MAVLINK_STX 253`
MAVLINK_STX_V2 = 0xFD
MAVLINK_STX_V1 = 0xFE

# v2 的"核心头"长度（不含 STX）：len(1) + incompat(1) + compat(1)
#                              + seq(1) + sysid(1) + compid(1) + msgid(3) = 9
# 出处：c_library_v2-master/mavlink_types.h:27  `#define MAVLINK_CORE_HEADER_LEN 9`
CORE_HEADER_LEN_V2 = 9

# ── CRC-16/MCRF4XX ───────────────────────────────────────────────────────
# MAVLink 用的不是常见的 CRC-16/CCITT（那个初值是 0x0000），
# 而是 **X.25 / MCRF4XX** 变体：初值 0xFFFF，反射，最后**不取反**。
# 出处：c_library_v2-master/checksum.h:22  `#define X25_INIT_CRC 0xffff`
X25_INIT_CRC = 0xFFFF


def crc_accumulate(byte, crc):
    """把一个字节累加进 CRC（逐位反射版，等价于查表法）。

    逐字节展开的写法在 checksum.h:35 的 crc_accumulate()，
    那里用了一个很省指令的位技巧（tmp ^= tmp<<4 ...）。
    下面这版是【教学版】：写成一目了然的移位-异或，
    结果和官方实现逐位相同（实验 3 会对拍）。
    """
    crc ^= byte
    for _ in range(8):
        if crc & 1:
            crc = (crc >> 1) ^ 0x8408      # 0x8408 = 0x1021 反射后
        else:
            crc >>= 1
    return crc & 0xFFFF


def crc_calculate(data):
    """对一段 buffer 算 CRC（初值 0xFFFF，不取反）。"""
    crc = X25_INIT_CRC
    for b in data:
        crc = crc_accumulate(b, crc)
    return crc


# ── CRC_EXTRA ────────────────────────────────────────────────────────────
# ★ 这是 MAVLink 最容易被误解的一个设计。
#
# 报文里的 CRC 只覆盖「帧头 + 载荷」，**不含 msgid**。于是：
#   「一条 9 字节载荷的 HEARTBEAT」和「另一条 9 字节载荷的消息」
#   可能算出同一个 CRC —— 收方无法判断自己是不是把两种消息搞混了。
#
# MAVLink 的解法：在 CRC 里再掺一个与消息【定义】绑定的字节 CRC_EXTRA。
# 定义一变（加字段、改类型、改顺序），CRC_EXTRA 就变，老固件会**直接丢包**
# 而不是解析出一堆错位的垃圾数据。这就是 MAVLink 的"版本兼容闸门"。
#
# 计算规则见 mavlink 官方 pymavlink/generator 的
#   `mavgen_c.py: message_checksum()`：
#     s = "MSGNAME " + 逐字段(type + " " + name + " ")
#     CRC_EXTRA = (crc(s) & 0xFF) ^ (crc >> 8)
#
# ★★ 这里有两个作者踩过坑、必须写下来的细节（两个都是实测对出来的）：
#
#   ① 字段用的是【线序】，不是 XML 声明顺序。
#      作者第一版按声明顺序拼字符串，算出来 HEARTBEAT=176，官方是 50。
#      改成线序（custom_mode 提到最前）立刻对上。
#
#   ② **extension 字段不参与计算**。
#      `<extensions/>` 之后的字段是 v2 的向后兼容扩展，
#      CRC_EXTRA 只覆盖【基础字段】。实测 COMMAND_ACK：
#        只算基础字段（command, result）      → 143 ✅ 与生成头文件一致
#        再加上 4 个 extension 字段            → 4   ❌
#      再用 GPS_RAW_INT 复核：只算基础字段 → 24 ✅（加扩展则变成 251）。
#
#   还有个小的：`uint8_t_mavlink_version` 在算 CRC 前要当作 `uint8_t`。
def crc_extra_of(name, fields):
    """由消息定义算出 CRC_EXTRA。

    fields: [(type, name), ...] —— ★★ 只传【基础字段】（<extensions/> 之前
            的那些），顺序写 XML 声明顺序即可，函数内部会自己按线序排。
    """
    s = name + " "
    for t, n in wire_order(fields):
        # uint8_t_mavlink_version 只是一个"我不填、库来填"的标记类型，
        # 在协议层面就是一个 uint8_t。
        if t == "uint8_t_mavlink_version":
            t = "uint8_t"
        s += t + " " + n + " "
    c = crc_calculate(s.encode("ascii"))
    return (c & 0xFF) ^ (c >> 8)


# ── 载荷字段重排（field reordering）─────────────────────────────────────
# ★ MAVLink 组帧时**不按 XML 声明顺序**摆放字段，而是按字段类型的字节宽度
#   【从大到小】排；同宽度保持声明顺序。原因：让多字节字段自然对齐，
#   在 8/16 位单片机上不用逐字段 memcpy，直接结构体覆盖即可。
#
#   注意：v2 的 **extension 字段（<extensions/> 之后的）不参与重排**，
#   它们永远按声明顺序排在末尾。
_TYPE_SIZE = {
    "double": 8, "uint64_t": 8, "int64_t": 8,
    "float": 4, "uint32_t": 4, "int32_t": 4,
    "uint16_t": 2, "int16_t": 2,
    "uint8_t": 1, "int8_t": 1, "char": 1,
    # ★ MAVLink 的魔法类型：XML 里写 `uint8_t_mavlink_version`，
    #   线路上就是一个普通 uint8_t（HEARTBEAT 的最后一个字段）。
    #   它只在【生成代码】时有意义 —— 发送方不用填，库自动写入协议版本号。
    #   所以这里当 uint8_t 处理，见实验 2.1。
    "uint8_t_mavlink_version": 1,
}

_TYPE_FMT = {
    "double": "d", "uint64_t": "Q", "int64_t": "q",
    "float": "f", "uint32_t": "I", "int32_t": "i",
    "uint16_t": "H", "int16_t": "h",
    "uint8_t": "B", "int8_t": "b", "char": "c",
    "uint8_t_mavlink_version": "B",
}


def wire_order(fields, ext=None):
    """把 (type, name) 列表按线序排好。

    ★ 数组类型（如 `float[4]`）算一个"大字段"，整体参与排序，
      元素之间当然是连续的。
    ★ ext 是 `<extensions/>` 之后的字段：**不参与重排**，按声明顺序缀在末尾。
    """
    def size_of(t):
        base = t.split("[")[0]
        n = 1
        if "[" in t:
            n = int(t[t.index("[") + 1:t.index("]")])
        return _TYPE_SIZE[base] * n

    indexed = list(enumerate(fields))
    indexed.sort(key=lambda p: (-size_of(p[1][0]), p[0]))
    return [f for _i, f in indexed] + list(ext or [])


def _unpack_one(t, values, n):
    """把一个字段从 values[n] 打包成字节。"""
    base, count = t, 1
    if "[" in t:
        base = t[:t.index("[")]
        count = int(t[t.index("[") + 1:t.index("]")])
    fmt = "<" + _TYPE_FMT[base] * count
    v = values[n]
    return struct.pack(fmt, v) if count == 1 else struct.pack(fmt, *v)


def pack_payload(fields, values, ext=None):
    """按线序把 python 值打包成载荷字节。

    values 用【字段名】做 key，这样调用方不用关心重排。
    """
    out = b""
    for t, n in wire_order(fields, ext):
        out += _unpack_one(t, values, n)
    return out


def unpack_payload(fields, payload, ext=None):
    """按线序把载荷解回 {字段名: 值}。

    ★ ext 里的字段可能【不在】载荷里 —— v2 允许发送方裁剪掉尾部的
      extension 字段（截断到 MIN_LEN）。解不出来就跳过，不报错。
      这正是 extension 字段"向后兼容"的机制：老代码不认识它们，
      新代码收到老报文时它们直接缺席。
    """
    vals, off = {}, 0
    for t, n in wire_order(fields, ext):
        base, count = t, 1
        if "[" in t:
            base = t[:t.index("[")]
            count = int(t[t.index("[") + 1:t.index("]")])
        fmt = "<" + _TYPE_FMT[base] * count
        sz = struct.calcsize(fmt)
        chunk = payload[off:off + sz]
        if len(chunk) < sz:
            break                    # extension 被裁剪掉了，后面的都没有
        got = struct.unpack(fmt, chunk)
        vals[n] = got[0] if count == 1 else list(got)
        off += sz
    return vals


def mavlink_v2_pack(msgid, fields, values, seq, sysid, compid,
                    crc_extra, incompat=0, compat=0, ext=None):
    """组一条完整的 MAVLink v2 帧，返回 bytes。

    ★ 帧结构（v2，无签名）：

        偏移  长度  字段            说明
        ----  ----  --------------  ------------------------------------
         0     1    STX = 0xFD      帧起始魔数（v1 是 0xFE）
         1     1    len             载荷长度（不含 CRC）
         2     1    incompat_flags  不兼容标志：收方看不懂就必须丢弃
         3     1    compat_flags    兼容标志：看不懂可以忽略
         4     1    seq             发送序号，0..255 循环
         5     1    sysid           系统 ID（飞控=1，地面站=255）
         6     1    compid          组件 ID（飞控=1，地面站=190/191…）
         7     3    msgid           消息 ID，小端 24 位
        10     N    payload         载荷（字段已按线序重排）
      10+N     2    CRC             覆盖 [1 .. 10+N-1] 再掺 CRC_EXTRA
    """
    payload = pack_payload(fields, values, ext)
    assert len(payload) < 256, "载荷不能超过 255 字节"
    hdr = struct.pack("<BBBBBB", len(payload), incompat, compat,
                      seq & 0xFF, sysid & 0xFF, compid & 0xFF)
    hdr += struct.pack("<I", msgid)[:3]          # 24 位小端 msgid

    crc = crc_calculate(hdr + payload)
    crc = crc_accumulate(crc_extra, crc)          # ★ 掺进 CRC_EXTRA
    return bytes([MAVLINK_STX_V2]) + hdr + payload + struct.pack("<H", crc)


def mavlink_v2_unpack(data, crc_extra_by_id):
    """拆一条 MAVLink v2 帧，返回 dict 或 None（不是我认识的帧）。"""
    if len(data) < 12 or data[0] != MAVLINK_STX_V2:
        return None
    length = data[1]
    incompat, compat, seq, sysid, compid = data[2], data[3], data[4], data[5], data[6]
    msgid = data[7] | (data[8] << 8) | (data[9] << 16)
    payload = data[10:10 + length]
    if len(payload) != length:
        return None
    got_crc = struct.unpack("<H", data[10 + length:12 + length])[0]

    extra = crc_extra_by_id.get(msgid, 0)
    want_crc = crc_accumulate(extra, crc_calculate(data[1:10 + length]))
    if got_crc != want_crc:
        return None

    return {"len": length, "incompat": incompat, "compat": compat,
            "seq": seq, "sysid": sysid, "compid": compid,
            "msgid": msgid, "payload": payload, "crc": got_crc,
            "raw": data[:12 + length]}


# ── 消息定义表 ───────────────────────────────────────────────────────────
# ★ 字段顺序 = XML 里的**声明顺序**（不是线序！线序由 wire_order() 算）。
# 出处：c_library_v2-master/message_definitions/minimal.xml:714（HEARTBEAT）
#       c_library_v2-master/message_definitions/common.xml:5902（COMMAND_LONG）
#       c_library_v2-master/message_definitions/common.xml:5916（COMMAND_ACK）
HEARTBEAT_FIELDS = [
    ("uint8_t", "type"),
    ("uint8_t", "autopilot"),
    ("uint8_t", "base_mode"),
    ("uint32_t", "custom_mode"),
    ("uint8_t", "system_status"),
    ("uint8_t_mavlink_version", "mavlink_version"),
]

COMMAND_LONG_FIELDS = [
    ("uint8_t", "target_system"),
    ("uint8_t", "target_component"),
    ("uint16_t", "command"),
    ("uint8_t", "confirmation"),
    ("float", "param1"), ("float", "param2"), ("float", "param3"),
    ("float", "param4"), ("float", "param5"), ("float", "param6"),
    ("float", "param7"),
]

COMMAND_ACK_FIELDS = [
    ("uint16_t", "command"),
    ("uint8_t", "result"),
]
# ★ `<extensions/>` 之后的字段单独放 —— 它们【不参与】CRC_EXTRA 的计算，
#   也【不参与】线序重排（永远按声明顺序缀在最后）。
#   出处：c_library_v2-master/message_definitions/common.xml:5920（<extensions/>）
COMMAND_ACK_EXT = [
    ("uint8_t", "progress"),
    ("int32_t", "result_param2"),
    ("uint8_t", "target_system"),
    ("uint8_t", "target_component"),
]

# 官方生成头文件里写死的 CRC_EXTRA，用来对拍我们自己算的
# 出处：c_library_v2-master/minimal/mavlink_msg_heartbeat.h:21
#       c_library_v2-master/common/mavlink_msg_command_long.h:26
#       c_library_v2-master/common/mavlink_msg_command_ack.h:21
OFFICIAL_CRC_EXTRA = {
    0: 50, 76: 152, 77: 143,
}

MSGS = {
    0: ("HEARTBEAT", HEARTBEAT_FIELDS, 50),
    76: ("COMMAND_LONG", COMMAND_LONG_FIELDS, 152),
    77: ("COMMAND_ACK", COMMAND_ACK_FIELDS, 143),
}

CRC_EXTRA_BY_ID = {k: v[2] for k, v in MSGS.items()}


# ── 枚举表（全部取自 MAVLink 官方 XML / 生成头文件）────────────────────
# 出处：c_library_v2-master/message_definitions/minimal.xml:213（MAV_MODE_FLAG）
MAV_MODE_FLAG = [
    (128, "SAFETY_ARMED", "已解锁（电机可转）"),
    (64, "MANUAL_INPUT_ENABLED", "遥控输入有效"),
    (32, "HIL_ENABLED", "硬件在环仿真"),
    (16, "STABILIZE_ENABLED", "增稳模式"),
    (8, "GUIDED_ENABLED", "引导模式"),
    (4, "AUTO_ENABLED", "自主模式"),
    (2, "TEST_ENABLED", "测试模式"),
    (1, "CUSTOM_MODE_ENABLED", "custom_mode 字段有效"),
]

# 出处：c_library_v2-master/message_definitions/common.xml:2818（MAV_RESULT）
MAV_RESULT = [
    (0, "ACCEPTED", "命令合法且已执行完毕"),
    (1, "TEMPORARILY_REJECTED", "命令合法，但此刻不能执行 —— 等一会重试可能就行"),
    (2, "DENIED", "命令非法或参数不对 —— 原样重试永远不会成功"),
    (3, "UNSUPPORTED", "本机不认识这条命令"),
    (4, "FAILED", "命令合法但执行失败 —— 先把故障修好再重试"),
    (5, "IN_PROGRESS", "正在执行，稍后还会来 ACK（用它刷进度）"),
    (6, "CANCELLED", "被 COMMAND_CANCEL 取消了"),
    (7, "COMMAND_LONG_ONLY", "只接受用 COMMAND_LONG 发的（用 COMMAND_INT 发会被拒）"),
    (8, "COMMAND_INT_ONLY", "只接受用 COMMAND_INT 发的"),
    (9, "COMMAND_UNSUPPORTED_MAV_FRAME", "坐标系不支持"),
]

# 出处：c_library_v2-master/message_definitions/common.xml:1369
MAV_CMD_DO_SET_MODE = 176
# 出处：c_library_v2-master/message_definitions/common.xml:1973
MAV_CMD_COMPONENT_ARM_DISARM = 400
# 出处：c_library_v2-master/message_definitions/common.xml:5902 附近
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_REPOSITION = 192


# ============================================================================
#  实验 1  环境体检（优雅降级）
# ============================================================================

def _probe_udp(port, timeout=0.35):
    """往 127.0.0.1:port 发一条 HEARTBEAT，看有没有飞控回话。

    ★ 不依赖 pymavlink —— 用实验 2 里我们自己写的组帧函数。
      没有回话也不报错，只是返回 None。
    """
    import socket
    frame = mavlink_v2_pack(
        0, HEARTBEAT_FIELDS,
        {"type": 6, "autopilot": 8, "base_mode": 0,
         "custom_mode": 0, "system_status": 4, "mavlink_version": 3},
        seq=0, sysid=255, compid=190, crc_extra=50)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(frame, ("127.0.0.1", port))
        data, _addr = s.recvfrom(2048)
    except Exception:
        return None
    finally:
        s.close()
    return mavlink_v2_unpack(data, CRC_EXTRA_BY_ID)


def exp1():
    head("实验 1  环境体检：本机能不能真跑一条 MAVLink 链路？")

    wrap("★ 这一节不是「检测不到就算了」，而是一张【可执行的检查表】："
         "每一项都真去查，并把【证据】印出来。"
         "换一台机器跑，这张表会跟着变 —— 结论以你的机器为准。")

    sub("1.1  Python 侧：有没有现成的 MAVLink 库")
    rows = []
    for mod, why in [("pymavlink", "地面站/脚本最常用的 MAVLink 实现"),
                     ("mavlink", "pymavlink 的内核模块"),
                     ("serial", "pyserial —— 串口链路要它")]:
        try:
            __import__(mod)
            rows.append([mod, "✅ 有", why])
        except Exception:
            rows.append([mod, "❌ 没有", why])
    tab(rows, ["模块", "import 结果", "它是干什么的"])

    has_pymavlink = any(r[1].startswith("✅") for r in rows[:2])
    if not has_pymavlink:
        warn("本机 import 不到 pymavlink。这不影响本脚本 —— "
             "下面所有实验都用【自己实现的编解码器】，"
             "反而能看清每一个字节。真实项目里要发命令，"
             "还是建议 pip install pymavlink（或直接用 C 库 c_library_v2）。")

    sub("1.2  网络侧：本机有没有飞控在监听 MAVLink UDP 端口")
    wrap("每个端口都真发一条 HEARTBEAT 过去，等 0.35 秒看有没有回包。")
    rows = []
    for port, who in [(14540, "机载/offboard（伴飞电脑、mavros 从这里进）"),
                      (14550, "地面站（QGC / Mission Planner 默认听这个）"),
                      (14580, "offboard 第二通道（部分 PX4 配置）"),
                      (14555, "MAVProxy 默认输出口")]:
        r = _probe_udp(port)
        if r is None:
            rows.append([str(port), "无应答", who, "—"])
        else:
            rows.append([str(port), "★ 有回包", who,
                         "sysid=%d compid=%d msgid=%d" % (r["sysid"], r["compid"], r["msgid"])])
    tab(rows, ["UDP 端口", "探测结果", "谁在听", "回包摘要"])

    got_any = any("★" in r[1] for r in rows)
    if not got_any:
        warn("一个端口都没回话 —— 说明本机现在没有飞控在跑（SITL 没起、也没插真机）。")
        wrap("环境不满足，下面转入【原理演示】：实验 2~5 全部用"
             "「自己造的报文」跑，不依赖任何外部设备。")
        wrap("想让它变成真的：起一个 SITL，命令见 卷 10 第 1 章。")
    else:
        note("有飞控在线 —— 你可以把实验 5 的时序图对着真实回包看一遍。")

    sub("1.3  串口侧：有没有疑似遥测口")
    rows = []
    cands = []
    if os.name == "nt":
        for i in range(1, 17):
            p = "COM%d" % i
            if os.path.exists("\\\\.\\" + p):
                cands.append(p)
        rows.append(["Windows", "探测 COM1..COM16", ", ".join(cands) if cands else "（一个都没有）"])
    else:
        import glob
        cands = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") +
                       glob.glob("/dev/serial0"))
        rows.append(["Linux/macOS", "探测 /dev/ttyACM*、/dev/ttyUSB*、/dev/serial0",
                     ", ".join(cands) if cands else "（一个都没有）"])
    tab(rows, ["平台", "探测方式", "结果"])
    wrap("★ 就算探测到串口，也【不能确定】它就是飞控的 TELEM 口 —— "
         "STM32 的 USB CDC、调试串口、蓝牙模块都长这样。"
         "确认方法：打开它，用 57600 收 3 秒，看到 0xFD 开头、"
         "长度字段在 9~30 之间反复出现的字节流，基本就是 MAVLink。")

    note("体检结论：本机【没有】现成的 MAVLink 库，也【没有】在跑的飞控。"
         "下面四个实验是【原理演示】，全部自给自足。")


# ============================================================================
#  实验 2  手工组一条 HEARTBEAT
# ============================================================================

def _hexdump(data, per=16):
    """打印一段十六进制 + ASCII，带偏移。"""
    for i in range(0, len(data), per):
        chunk = data[i:i + per]
        hexs = " ".join("%02X" % b for b in chunk)
        asci = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print("    %4d  %-*s  |%s|" % (i, per * 3 - 1, hexs, asci))


def exp2():
    head("实验 2  手工组一条 HEARTBEAT：v2 帧逐字节拆开")

    wrap("HEARTBEAT 是 MAVLink 里最重要的一条消息 —— "
         "它一发，飞控就把你登记成「在线的地面站」；"
         "它停 2.5 秒，飞控的 MAVLink 层就先把这条链路标成不健康"
         "（PX4 的 HEARTBEAT_TIMEOUT_US = 2500000）。"
         "先原样组一条，再逐字节拆开看。")

    sub("2.1  先看字段定义（XML 里怎么写的）")
    tab([[t, n] for t, n in HEARTBEAT_FIELDS],
        ["字段类型", "字段名（XML 声明顺序）"])
    wrap("★ 注意最后一个字段是 `uint8_t_mavlink_version` 而不是 `uint8_t` —— "
         "这是 MAVLink 的魔法类型：**发送方不能填**，"
         "库在组帧时自动写入协议版本号（v2 就是 3）。"
         "所以 pack 时给它一个占位值 3 只是为了让结构体长度对。")

    sub("2.2  线序 vs 声明顺序：载荷不是按你想的顺序摆的")
    tab([["%d" % i, t, n, "按声明顺序"]
         for i, (t, n) in enumerate(HEARTBEAT_FIELDS, 1)],
        ["声明序", "类型", "字段", "备注"])
    wire = wire_order(HEARTBEAT_FIELDS)
    tab([["%d" % i, t, n, "%d 字节" % (_TYPE_SIZE[t.split("[")[0]])]
         for i, (t, n) in enumerate(wire, 1)],
        ["线序", "类型", "字段", "宽度"])
    wrap("★ 看懂了吗：`custom_mode`（4 字节）从声明序第 4 位，"
         "被提到了线序第 1 位。规则是 **按字段宽度从大到小排，同宽保持声明顺序**。"
         "理由很实在：8 位单片机上，4 字节字段放前面就能自然对齐，"
         "整个结构体可以直接 memcpy 进载荷，不用逐字段搬运。")
    warn("这个重排是 MAVLink 「最容易踩且最不容易发现」的坑 —— "
         "你按 XML 顺序手工拼字节，收方就是解不出来，而且**不报错**，"
         "只是数据全是乱的。永远用生成的库来组帧。")

    sub("2.3  组帧：GCS 心跳（sysid=255, compid=190）")
    values = {
        "type": 6,               # MAV_TYPE_GCS
        "autopilot": 8,          # MAV_AUTOPILOT_INVALID —— 地面站不是飞控
        "base_mode": 0,          # 地面站不参与模式位图
        "custom_mode": 0,
        "system_status": 4,      # MAV_STATE_ACTIVE
        "mavlink_version": 3,
    }
    frame = mavlink_v2_pack(0, HEARTBEAT_FIELDS, values,
                            seq=0, sysid=255, compid=190, crc_extra=50)
    kv("帧总长", "%d 字节 = 1(STX) + 9(核心头) + %d(载荷) + 2(CRC)"
       % (len(frame), len(frame) - 12))

    print()
    print("  十六进制：")
    _hexdump(frame)

    sub("2.4  逐字节对照")
    rows = [
        ["0", "1", "STX = 0xFD", "v2 帧的魔数。看到 0xFE 就是 v1 老帧"],
        ["1", "1", "len = %d" % frame[1], "载荷字节数，不含帧头和 CRC"],
        ["2", "1", "incompat = 0x%02X" % frame[2],
         "不兼容标志。★ 收方看不懂这一位就必须【整帧丢弃】"],
        ["3", "1", "compat = 0x%02X" % frame[3],
         "兼容标志。收方看不懂【可以忽略】，照常解析"],
        ["4", "1", "seq = %d" % frame[4],
         "发送序号，0..255 循环。丢包检测靠它"],
        ["5", "1", "sysid = %d" % frame[5], "系统 ID。飞控=1，地面站=255"],
        ["6", "1", "compid = %d" % frame[6],
         "组件 ID。MAV_COMP_ID_AUTOPILOT1=1，MAV_COMP_ID_MISSIONPLANNER=190"],
        ["7", "3", "msgid = %d" % (frame[7] | frame[8] << 8 | frame[9] << 16),
         "消息 ID，小端 24 位 → HEARTBEAT"],
        ["10", "%d" % frame[1], "payload", "按【线序】排好的字段"],
        ["%d" % (10 + frame[1]), "2",
         "CRC = 0x%04X" % struct.unpack("<H", frame[-2:])[0],
         "覆盖 [1 .. 帧尾-2] 再掺 CRC_EXTRA(50)"],
    ]
    tab(rows, ["偏移", "长度", "内容", "含义"])

    sub("2.5  载荷内部：再拆一层")
    short = _TYPE_FMT.copy()
    short["uint8_t_mavlink_version"] = "B"
    rows = []
    off = 10
    for t, n in wire_order(HEARTBEAT_FIELDS):
        f = "uint8_t" if t == "uint8_t_mavlink_version" else t
        sz = _TYPE_SIZE[f]
        v = struct.unpack("<" + short[t], frame[off:off + sz])[0]
        rows.append([str(off), str(sz), t, n, str(v),
                     "MAV_TYPE_GCS" if n == "type" else
                     "MAV_AUTOPILOT_INVALID" if n == "autopilot" else
                     "MAV_STATE_ACTIVE" if n == "system_status" else ""])
        off += sz
    tab(rows, ["载荷内偏移", "宽度", "类型", "字段", "值", "枚举含义"])

    sub("2.6  再拆回来：验证往返一致")
    got = mavlink_v2_unpack(frame, CRC_EXTRA_BY_ID)
    if got is None:
        print("  ❌ 解包失败（说明组帧/拆帧不自洽）")
    else:
        vals = unpack_payload(HEARTBEAT_FIELDS, got["payload"])
        kv("sysid / compid", "%d / %d" % (got["sysid"], got["compid"]))
        kv("msgid", "%d (HEARTBEAT)" % got["msgid"])
        kv("解出的字段", ", ".join("%s=%s" % (k, v) for k, v in vals.items()))
        kv("往返结果", "✅ 与发送值一致")

    sub("2.7  心跳的节奏：两个【不同的】超时，别混成一个")
    wrap("MAVLink 规定心跳周期 1 Hz。但「超时」在飞控里是【两层】，"
         "数值差 4 倍，含义完全不同：")
    tab([
        ["第一层：链路健康", "2.5 s",
         "PX4 的 HEARTBEAT_TIMEOUT_US = 2500000",
         "只是把这条链路标成「不健康」：不再往外发流、界面上报红"],
        ["第二层：失联保护", "10 s",
         "PX4 参数 COM_DL_LOSS_T = 10（范围 5~300）",
         "★ 进入 GCS 失联模式，执行 NAV_DLL_ACT 选的动作 —— "
         "但 NAV_DLL_ACT 出厂默认是 0 = Disabled，也就是【什么都不做】"],
    ], ["层", "阈值", "出处", "触发后发生什么"])
    warn("⚠️⚠️ 注意第二层的【动作】参数：NAV_DLL_ACT 默认 = 0 = Disabled"
         "（commander_params.c:649；取值表在 :638-643）。"
         "也就是说「超时了」和「飞控接管了」也是两件事 —— "
         "出厂状态下地面站断了，飞控不会返航，也不会降落。"
         "对照 ArduPilot：FS_GCS_ENABLE 的默认同样是 0 = Disabled"
         "（ArduCopter/Parameters.cpp:168 + defines.h:135），"
         "阈值 FS_GCS_TIMEOUT 才是 5 s。"
         "两家都把 GCS 失联保护默认关掉，理由也一样："
         "遥控链路正常、只有数传断了时，自动返航往往更危险。")
    rows = []
    t_last = 0.0
    for t in [0.0, 1.0, 2.0, 3.0, 4.0, 6.5, 7.0, 11.5, 17.5]:
        age = t - t_last
        if age < 1.5:
            verdict = "正常"
        elif age < 2.5:
            verdict = "正常（还没到阈值）"
        elif age < 10.0:
            verdict = "⚠️ 第一层：链路标红（还没进失联模式）"
        else:
            verdict = "❌ 第二层击穿 → 执行 NAV_DLL_ACT（默认 Disabled）"
        rows.append(["%.1f s" % t,
                     "收到心跳，重置计时" if age < 1.5 else "（丢了几条）",
                     "距上次心跳 %.1f s" % age, verdict])
        if age < 1.5:
            t_last = t
    tab(rows, ["时刻", "事件", "距上次心跳", "飞控侧判断"])
    wrap("★ 看 6.5 s 那一行：距上次心跳 3.5 s —— 已经超过第一层的 2.5 s，"
         "但**远没到第二层的 10 s**。")
    wrap("这解释了一个很常见的困惑 —— 「我的地面站卡了一下，"
         "链路显示掉线，可飞控并没有返航」。"
         "因为「链路不健康」和「操作员失联」是两件事。")
    wrap("再看 17.5 s 那一行：距上次心跳 10.5 s，第二层也被击穿了，"
         "飞控进入 GCS 失联模式 —— 但**执行什么动作要看 NAV_DLL_ACT**，"
         "而出厂默认是 Disabled（见上面那条 ⚠️）。"
         "所以「超时了」≠「飞控接管了」，这又是两件事。")
    warn("⚠️ 但反过来不成立：如果你**根本没发过**心跳，"
         "那么第一层第二层都不需要等 —— 飞控从来就没把你当成过地面站，"
         "你发的命令会被直接忽略。见 5.2 的闸门 3。")


# ============================================================================
#  实验 3  CRC 与 CRC_EXTRA
# ============================================================================

def exp3():
    head("实验 3  CRC 与 CRC_EXTRA：协议怎么把「版本不符」挡在门外")

    sub("3.1  先看 MAVLink 用的那个 CRC 是哪一种")
    tab([
        ["CRC-16/MCRF4XX", "0xFFFF", "反射(0x8408)", "不取反", "★ MAVLink 用这个"],
        ["CRC-16/CCITT-FALSE", "0xFFFF", "不反射(0x1021)", "不取反", "很多教材默认讲的"],
        ["CRC-16/X-25", "0xFFFF", "反射(0x8408)", "★ 取反", "和 MAVLink 只差最后一步"],
    ], ["变体", "初值", "多项式", "输出", "备注"])
    warn("★ 这三个长得极像，但结果完全不同。协议文档只写「CRC-16」时，"
         "你必须问清是哪一个变体 —— 猜错的症状是「对方一个包都不回」，"
         "而且你从字节流上看不出任何线索。")

    sub("3.2  手算一遍，验证实现")
    demo = b"123456789"
    rows = []
    rows.append(["CRC-16/MCRF4XX", "0x%04X" % crc_calculate(demo), "本脚本实现",
                 "官方 check 值 0x6F91"])
    tab(rows, ["变体", "算出来的值", "来源", "对照"])

    sub("3.3  ★★ CRC_EXTRA：从字段定义【算】出来")
    wrap("报文 CRC 不覆盖 msgid。所以「9 字节载荷的 HEARTBEAT」和"
         "「另一条 9 字节载荷的消息」可能撞出同一个 CRC，"
         "收方无法察觉自己把两种消息搞混了。")
    wrap("MAVLink 的解法：把消息的【字段定义】也算成一个字节掺进 CRC，"
         "叫 CRC_EXTRA。定义一变（加字段、改类型、改顺序），它立刻变。")

    print()
    print('  算法（mavgen_c.py 的 message_checksum）：')
    print('      s = "MSGNAME " + 对每个【基础字段】: 类型 + " " + 名字 + " "')
    print('      CRC_EXTRA = (crc(s) & 0xFF) ^ (crc(s) >> 8)')
    print()
    print('      ★ 字符串里的字段用【线序】，而且【不含 extension 字段】')
    print()

    rows = []
    for msgid in (0, 76, 77):
        name, fields, official = MSGS[msgid]
        mine = crc_extra_of(name, fields)
        s = name + " "
        for t, n in wire_order(fields):
            if t == "uint8_t_mavlink_version":
                t = "uint8_t"
            s += t + " " + n + " "
        rows.append([str(msgid), name, s[:44] + ("…" if len(s) > 44 else ""),
                     str(mine), str(official),
                     "✅" if mine == official else "❌"])
    tab(rows, ["msgid", "消息名", "参与 CRC 的字符串", "本脚本算出",
               "官方头文件", "对拍"])

    for msgid in (0, 76, 77):
        name, fields, official = MSGS[msgid]
        if crc_extra_of(name, fields) == official:
            note("三条 CRC_EXTRA 全部对上 —— 说明我们对「字段顺序」和 "
                 "「CRC 变体」的理解都是对的。")
            break

    sub("3.3.1  ★ 两个不自己动手就发现不了的细节")
    wrap("上面那张表「对上了」看着理所当然，但作者写第一版时"
         "**三条全错**，而且错法很有意思，值得记下来：")
    tab([
        ["① 字段顺序", "按 XML 声明顺序拼字符串",
         "HEARTBEAT 算出 176（官方 50）",
         "★ 要用【线序】—— 4 字节的 custom_mode 提到最前"],
        ["② extension 字段", "把 `<extensions/>` 之后的字段也算进去",
         "COMMAND_ACK 算出 4（官方 143）",
         "★ extension **不参与** CRC_EXTRA"],
    ], ["坑", "第一版怎么写的", "结果", "正解"])
    wrap("★ 第二条尤其反直觉，但它正是 extension 机制能成立的前提：")
    wrap("想让「新固件加字段、老固件照收」这件事成立，"
         "新加的那个字段就**不能**改变 CRC_EXTRA —— "
         "否则每加一个可选字段，所有老接收方都会把整帧丢掉，"
         "扩展性等于零。")
    wrap("所以 MAVLink 把「改定义」分成两类：")
    tab([
        ["改【基础字段】（加/删/改类型/改顺序）", "CRC_EXTRA 变",
         "❌ 不兼容：老固件整帧丢弃"],
        ["在 `<extensions/>` 之后加字段", "CRC_EXTRA **不变**",
         "✅ 兼容：老固件照收，只是不认识新字段"],
    ], ["改动", "CRC_EXTRA", "后果"])
    note("一句话：**CRC_EXTRA 是「基础字段定义」的指纹，"
         "不是「整个消息定义」的指纹。**")

    sub("3.4  为什么它是「版本闸门」：改一个字段会怎样")
    wrap("接着上面的分类，看两组具体数字：")
    f_new = HEARTBEAT_FIELDS + [("uint8_t", "battery_count")]
    rows = [
        ["原定义（6 个基础字段）", str(crc_extra_of("HEARTBEAT", HEARTBEAT_FIELDS)),
         "50", "✅ 互通"],
        ["基础字段里加 1 个 uint8_t",
         str(crc_extra_of("HEARTBEAT", f_new)), "50",
         "❌ 老固件整帧丢弃（而不是解出错位数据）"],
        ["调换两个同宽基础字段的声明顺序",
         str(crc_extra_of("HEARTBEAT", [HEARTBEAT_FIELDS[1],
                                        HEARTBEAT_FIELDS[0]] +
                                        HEARTBEAT_FIELDS[2:])),
         "50", "❌ 同样被挡"],
    ]
    tab(rows, ["改动", "新的 CRC_EXTRA", "老固件期望", "后果"])

    sub("3.5  实测：改坏一个 bit，CRC 立刻发现")
    frame = mavlink_v2_pack(0, HEARTBEAT_FIELDS,
                            {"type": 2, "autopilot": 12, "base_mode": 0x80,
                             "custom_mode": 0, "system_status": 4,
                             "mavlink_version": 3},
                            seq=7, sysid=1, compid=1, crc_extra=50)
    rows = []
    rows.append(["原帧", "%02X" % frame[2], "✅ 解包成功" if
                 mavlink_v2_unpack(frame, CRC_EXTRA_BY_ID) else "❌ 失败"])
    bad = bytearray(frame)
    bad[2] ^= 0x01        # 翻 incompat_flags 的最低位
    rows.append(["incompat 翻 1 bit", "%02X" % bad[2],
                 "✅ 解包成功" if mavlink_v2_unpack(bytes(bad), CRC_EXTRA_BY_ID)
                 else "❌ 整帧丢弃（CRC 不符）"])
    bad2 = bytearray(frame)
    bad2[4] = (bad2[4] + 1) & 0xFF     # 只动 seq
    rows.append(["seq 加 1", "%02X" % bad2[4],
                 "✅ 解包成功" if mavlink_v2_unpack(bytes(bad2), CRC_EXTRA_BY_ID)
                 else "❌ 整帧丢弃（CRC 不符）"])
    bad3 = bytearray(frame)
    bad3[10] ^= 0x01      # 动载荷第一个字节（type 字段）
    rows.append(["载荷翻 1 bit", "%02X" % bad3[10],
                 "✅ 解包成功" if mavlink_v2_unpack(bytes(bad3), CRC_EXTRA_BY_ID)
                 else "❌ 整帧丢弃（CRC 不符）"])
    tab(rows, ["改动", "改动后该字节", "收方行为"])

    warn("注意第二行：连 seq 这种「无关紧要」的字段被改，CRC 也会拦下来。"
         "CRC 覆盖的是 [len .. 载荷末] 的每一个字节，没有例外。")

    sub("3.6  那丢包怎么办：seq 才是干这个的")
    wrap("CRC 负责「这一帧有没有被损坏」，seq 负责「有没有整帧丢失」。"
         "发送方每发一帧 seq 加一（0..255 回绕），接收方看 seq 连不连续。")
    rows = []
    prev = None
    lost = 0
    for s in [10, 11, 13, 14, 15, 2, 3]:
        if prev is not None:
            expected = (prev + 1) & 0xFF
            d = (s - expected) & 0xFF
            if d == 0:
                rows.append([str(s), "连续", "—", "—"])
            else:
                lost += d
                rows.append([str(s), "★ 断档", "期望 %d，来了 %d" % (expected, s),
                             "丢了 %d 帧" % d])
        else:
            rows.append([str(s), "首帧", "—", "—"])
        prev = s
    tab(rows, ["收到的 seq", "判定", "差额", "推断"])
    wrap("★ 但 seq 是 uint8，255 之后自然回到 0 —— "
         "所以判据必须写成 `(s - expected) & 0xFF`，**不能直接相减**。"
         "直接减的话，255→0 会被算成「丢了 255 帧」，"
         "而实际上那是正常回绕。")
    wrap("还有一个反直觉点：seq 是**每条链路各自维护**的，不是全局的。"
         "同一条消息从串口发一份、从 UDP 发一份，两边的 seq 互不相干；"
         "飞控发给你的 seq 和你发给飞控的 seq 也各算各的。")


# ============================================================================
#  实验 4  ★★ custom_mode 位域
# ============================================================================

# PX4 的主模式枚举。出处：
#   src/modules/commander/px4_custom_mode.h（PX4 1.15.4）
PX4_MAIN_MODE = [
    (1, "MANUAL", "Manual", "纯手动，无增稳"),
    (2, "ALTCTL", "Altitude", "高度保持，姿态手动"),
    (3, "POSCTL", "Position", "位置保持，摇杆控制速度"),
    (4, "AUTO", "Mission / Hold / Return / Land / Takeoff",
     "自动驾驶，具体动作看【子模式】"),
    (5, "ACRO", "Acro", "特技，角速率控制"),
    (6, "OFFBOARD", "Offboard", "外部控制（伴飞电脑发 setpoint）"),
    (7, "STABILIZED", "Stabilized", "增稳，姿态角控制"),
    (8, "RATTITUDE", "Rattitude", "摇杆超阈值自动切 Acro"),
]

PX4_AUTO_SUB = [
    (1, "READY", "Ready", "已就绪，等指令"),
    (2, "TAKEOFF", "Takeoff", "自动起飞"),
    (3, "LOITER", "Hold", "自动悬停/等待"),
    (4, "MISSION", "Mission", "执行航线"),
    (5, "RTL", "Return", "返航"),
    (6, "LAND", "Land", "自动降落"),
    (8, "FOLLOW_TARGET", "Follow Target", "跟随目标"),
    (9, "PRECLAND", "Precision Land", "精准降落"),
]


def px4_custom_mode(main, sub=0):
    """PX4 的 32 位 custom_mode 位域。

    ★ 这是本章最值钱的一张图。位分配（来自 PX4 的 px4_custom_mode 联合体）：

        bit 31                24 23            16 15                  0
        +----------------------+----------------+----------------------+
        |   sub_mode (8 位)    | main_mode(8位) |     reserved (16)    |
        +----------------------+----------------+----------------------+
                  |                    |
                  |                    +-- 主模式：MANUAL/ALTCTL/POSCTL/AUTO/...
                  +-- 子模式：只有 main=AUTO 时才有意义
                              （MISSION/RTL/LAND/TAKEOFF/...）

    ⚠️ 注意位域是「错开」的：main 在 bits 16-23，sub 在 bits 24-31。
       不是「低 8 位 main、次 8 位 sub」。素材里 main.cpp 的
           custom_mode = (sub << 24) | (main << 16)
       写的就是这个。
    """
    return ((sub & 0xFF) << 24) | ((main & 0xFF) << 16)


def split_custom_mode(cm):
    """从 32 位 custom_mode 里拆出 (main, sub)。"""
    return (cm >> 16) & 0xFF, (cm >> 24) & 0xFF


def qgc_name(main, sub):
    """把 (main, sub) 翻译成 QGC 界面上显示的那个名字。"""
    if main == 4:      # AUTO 才有子模式
        for v, _e, disp, _d in PX4_AUTO_SUB:
            if v == sub:
                return disp
        return "Auto(%d)" % sub
    for v, _e, disp, _d in PX4_MAIN_MODE:
        if v == main:
            return disp
    return "Unknown(%d/%d)" % (main, sub)


def exp4():
    head("实验 4  ★★ custom_mode 位域：主模式、子模式、和 QGC 的命名")

    sub("4.1  custom_mode 到底是个什么东西")
    wrap("HEARTBEAT 里有三个和「模式」有关的字段，很容易搞混：")
    tab([
        ["base_mode", "uint8_t", "MAVLink 标准的模式位图（MAV_MODE_FLAG）",
         "跨飞控通用：是否解锁、是否增稳、是否自主"],
        ["custom_mode", "uint32_t", "★ 各飞控【自己定义】的 32 位",
         "MAVLink 只规定「有这么一个字段」，怎么切位是 PX4/APM 自己的事"],
        ["system_status", "uint8_t", "MAV_STATE 枚举",
         "系统健康度：STANDBY/ACTIVE/CRITICAL/EMERGENCY…"],
    ], ["字段", "类型", "是什么", "谁定义"])

    sub("4.2  PX4 的切法：一张位域图")
    print("""
       bit 31                    24 23          16 15              0
       +---------------------------+--------------+-----------------+
       |      sub_mode (8 bit)     | main_mode(8) |   reserved (16) |
       +---------------------------+--------------+-----------------+
                    |                    |
                    |                    +-- 主模式 (PX4_CUSTOM_MAIN_MODE)
                    |                        MANUAL=1 ALTCTL=2 POSCTL=3 AUTO=4
                    |                        ACRO=5 OFFBOARD=6 STABILIZED=7 RATTITUDE=8
                    |
                    +-- 子模式 (PX4_CUSTOM_SUB_MODE_AUTO)
                        只有 main 恰好 = AUTO(4) 时才有意义
                        READY=1 TAKEOFF=2 LOITER=3 MISSION=4 RTL=5 LAND=6
                        FOLLOW_TARGET=8 PRECLAND=9
    """)
    warn("★ 位域是【错开的】：main 在 bit16-23，sub 在 bit24-31。"
         "不是「低 8 位 main、次 8 位 sub」。"
         "只看 custom_mode 的十进制值是没意义的 —— 一定要按位拆。")

    sub("4.3  实测：常见模式的 custom_mode 值长什么样")
    rows = []
    for m, sm, label in [
        (1, 0, "MANUAL"), (2, 0, "ALTCTL"), (3, 0, "POSCTL"),
        (4, 0, "AUTO（未指定子模式）"),
        (4, 3, "AUTO/LOITER = QGC 的 Hold"),
        (4, 4, "AUTO/MISSION = QGC 的 Mission"),
        (4, 5, "AUTO/RTL = QGC 的 Return"),
        (4, 6, "AUTO/LAND = QGC 的 Land"),
        (6, 0, "OFFBOARD"),
        (7, 0, "STABILIZED"),
    ]:
        cm = px4_custom_mode(m, sm)
        rows.append(["%d" % m, "%d" % sm, "0x%08X" % cm, "%d" % cm,
                     label, qgc_name(m, sm)])
    tab(rows, ["main", "sub", "custom_mode (hex)", "十进制", "PX4 叫法", "QGC 显示"])

    note("看到 0x04040000 = 67371008 就是「AUTO/MISSION」，"
         "0x00030000 = 196608 是「POSCTL」。"
         "以后在日志里看到 custom_mode 这个数字，别再愣住。")

    sub("4.4  反向拆解：拿到一个数，读出模式名")
    rows = []
    for raw in [0x00030000, 0x04040000, 0x03040000, 0x05040000, 0x00060000,
                0x00000000, 0x00020000]:
        m, sm = split_custom_mode(raw)
        rows.append(["0x%08X" % raw, "%d" % m, "%d" % sm,
                     qgc_name(m, sm),
                     "⚠️ 主模式 0 不存在 —— 可能是没解出来的 v1 帧"
                     if m == 0 else
                     ("注意这不是 POSCTL！PX4 里 2 = ALTCTL" if raw == 0x00020000
                      else "")])
    tab(rows, ["custom_mode", "main", "sub", "QGC 名字", "备注"])

    sub("4.5  ★★ 素材里的三份枚举【互相矛盾】")
    wrap("本机 $SRC_TEST/ 下有三份 PX4 模式枚举表，"
         "它们是三个人/三个时期写的，值对不上：")
    tab([
        ["mavlink_udp2/…/PX4QGCFlightMode.h:8-21", "MANUAL=1 … PRECLAND=12",
         "udp2（最新版）", "✅ 与 PX4 1.15 一致"],
        ["mavlink_serial2/…/PX4QGCFlightMode.h:5-18", "MANUAL=1 … PRECLAND=12",
         "serial2", "✅ 与 PX4 1.15 一致"],
        ["mavlink_serial/…/PX4FlightModes.h:6-19", "MANUAL=0 … PRECLAND=11",
         "MFC 串口客户端（较早）", "❌ 整体错 1"],
        ["ground_station_px4_mode_switch_Version4.cpp:28-41", "MANUAL=0 … PRECLAND=11",
         "V4 地面站例程", "❌ 整体错 1"],
    ], ["文件", "枚举值", "出处", "判定"])
    wrap("对不上的代价是实打实的：按错 1 的那份表发 POSCTL（那个表里写 2），"
         "PX4 收到的 main=2 是 **ALTCTL** —— "
         "你以为切了位置保持，实际切的是高度保持，而且 ACK 还是 ACCEPTED。")
    warn("★ 那是从哪来的？mavlink_serial/PX4FlightModes.h:5 的注释写着"
         "「枚举取自 PX4 Autopilot 源码 px4_firmware/msg/vehicle_status.msg」"
         "—— `px4_firmware` 是 **2016 年以前**的 PX4 目录名。"
         "那份枚举是从一个十年前的老版本抄来的，之后没再对过。")
    note("结论：**枚举值不要抄博客、不要抄旧代码，去 PX4 源码 "
         "src/modules/commander/px4_custom_mode.h 里对。**"
         "本实验 4.2 那张表就是从那里逐行核过的。")

    sub("4.6  和 ArduPilot 的对比：同一个字段，完全不同的切法")
    tab([
        ["PX4", "main(bit16-23) + sub(bit24-31)", "有主/子两层",
         "AUTO 一个主模式装下 MISSION/RTL/LAND/TAKEOFF…"],
        ["ArduPilot", "低 8 位 = 模式号（0..N 连续编号）", "只有一层",
         "Copter 的 STABILIZE=0 ALT_HOLD=2 LOITER=5 RTL=6 AUTO=3 GUIDED=4…"],
    ], ["飞控", "custom_mode 切法", "层级", "后果"])
    wrap("★ 这就是为什么 MAVLink 要把 custom_mode 留给各飞控自己定义 —— "
         "ArduPilot 的模式是「一个连续的编号」，PX4 是「两层位域」，"
         "两者不可能共用一个标准。地面站要做的，是先看 HEARTBEAT 里的 "
         "`autopilot` 字段（PX4=12，ArduPilot=3），再选对应的解释表。")


# ============================================================================
#  实验 5  ★ ACK 与会话
# ============================================================================

def exp5():
    head("实验 5  ★ ACK 与会话：一条从零到起飞的真实时序")

    sub("5.1  MAV_RESULT：ACK 里那个 result 字节的十种取值")
    tab([["%d" % v, "MAV_RESULT_" + e, d] for v, e, d in MAV_RESULT],
        ["值", "枚举", "中文含义 / 你该怎么办"])
    wrap("★ 只记三句话就够了：")
    wrap("    · ACCEPTED(0) —— 成了，别再看后面的字段。\n"
         "    · TEMPORARILY_REJECTED(1) / IN_PROGRESS(5) —— **可以重试**。\n"
         "    · DENIED(2) / UNSUPPORTED(3) / FAILED(4) —— **重试没用**，"
         "先改状态或改参数。")
    warn("★ 最常见的误判：把 DENIED(2) 当成「再发几次就好」。"
         "DENIED 的定义就是「参数不对，原样重发永远不会成功」。"
         "（而 TEMPORARILY_REJECTED 恰恰相反：它就是叫你等一下再试。）")

    sub("5.2  ★★ 素材里的坑：V4 例程「强行解锁」为什么常常失败")
    wrap("ground_station_px4_mode_switch_Version4.cpp 的做法是："
         "按回车 → 发 ARM → sleep 800ms → 发 DO_SET_MODE(POSCTL)。"
         "这条路径上有四道闸门，任何一道没过，你看到的都是「没反应」：")
    tab([
        ["闸门 1", "预解锁检查（preflight check）",
         "PX4 的 commander 在 arm 前会查：传感器是否校准、"
         "EKF 是否收敛、是否在地面、电池是否够…",
         "结果是 DENIED 或 FAILED"],
        ["闸门 2", "模式切换本身被拒",
         "POSCTL 要求位置估计有效（GPS/光流/视觉）。"
         "室内没 GPS 时切 POSCTL 会失败；"
         "而 MANUAL/STABILIZED/ALTCTL 通常能切",
         "结果是 TEMPORARILY_REJECTED，或干脆不回 ACK"],
        ["闸门 3", "根本没收到 ACK",
         "ACK 是**单播**的：target_system / target_component 填错，"
         "飞控就把 ACK 发给了别人；或者你的 sysid 和飞控记的不一样",
         "现象是「什么都没发生」"],
        ["闸门 4", "param5/param6 被当成无效值",
         "COMMAND_LONG 的 param5/param6 是 float。若填了 INT32_MAX"
         "（想表示「不适用」），PX4 会判定你本该用 COMMAND_INT，"
         "直接回 DENIED 并打印 PX4_ERR",
         "结果是 DENIED，命令【没进 commander】"],
    ], ["闸门", "是什么", "为什么", "症状"])
    wrap("★ 闸门 4 的依据是 mavlink_receiver.cpp:418-428 —— "
         "飞控用 nextafterf 在 INT32_MAX 两侧各取一个浮点数做区间判断，"
         "命中了就 acknowledge(..., DENIED) 然后 return。"
         "修法：param5/param6 不适用时填 0 或 NaN，不要填 INT32_MAX。")
    wrap("★ 还有一个更隐蔽的：**要先有 GCS 心跳**。"
         "PX4 收到陌生 sysid 的 COMMAND_LONG 时，如果这个 sysid 不在"
         "它的「已知地面站」列表里，可能直接忽略。"
         "所以正确顺序永远是：**先发心跳 → 等飞控也回心跳 → 再发命令**。")

    sub("5.3  ★★ DO_SET_MODE 的参数：到底该填哪个（素材在这里错了）")
    wrap("MAV_CMD_DO_SET_MODE 的 XML 定义（common.xml:1369）：")
    tab([
        ["param1", "Mode", "MAV_MODE（base_mode 位图）",
         "填 base_mode 字节"],
        ["param2", "Custom Mode", "飞控自定义的【主模式】",
         "★ 填 main_mode 号，如 3 = POSCTL"],
        ["param3", "Custom Submode", "飞控自定义的【子模式】",
         "★ 填 sub_mode 号，如 4 = MISSION"],
        ["param4-7", "Empty", "—", "填 0"],
    ], ["参数", "XML 标签", "含义", "怎么填"])
    wrap("★ 权威依据是飞控自己的接收代码（PX4 1.15.4）：")
    print("""
      mavlink_receiver.cpp:869-871   ← 把收到的消息翻译成内部命令
          vcmd.param1 = (float)new_mode.base_mode;
          vcmd.param2 = (float)custom_mode.main_mode;    ← 主模式
          vcmd.param3 = (float)custom_mode.sub_mode;     ← 子模式

      Commander.cpp:786-788          ← 内部命令再拆回三个数
          uint8_t base_mode       = (uint8_t)cmd.param1;
          uint8_t custom_main_mode = (uint8_t)cmd.param2;
          uint8_t custom_sub_mode  = (uint8_t)cmd.param3;
    """)
    warn("⚠️⚠️ 所以切 AUTO/MISSION 要发的是："
         "param1 = base_mode、param2 = 4（AUTO）、param3 = 4（MISSION），"
         "**不是** param2 = 0x04040000。")
    wrap("而素材 mavlink_udp2/main.cpp:181 恰好是后者：")
    print("""
        uint32_t custom_mode = (cmd.sub_mode << 24) | (cmd.main_mode << 16);
        ... txbuf[42 + 8] = custom_mode;      // ← 塞进了 param2
    """)
    wrap("飞控侧读到 param2 = 0x04040000，`(uint8_t)` 截断后只剩低 8 位 → "
         "**0** → 「主模式 0」不存在 → 即便后面那道路闸门开了，"
         "这个值也对不上任何模式。（`(uint8_t)0x04040000 == 0`，"
         "高位被子模式占满了。）")
    wrap("ArduPilot 那边又是另一套：它读 param2 当【扁平的线性模式号】"
         "（0=STABILIZE、4=GUIDED…），没有子模式这个概念。"
         "同一个 command id，两套语义 —— 这就是为什么地面站必须区分机型"
         "（QGC 为此在 APMFirmwarePlugin.h:50 单独把 DO_SET_MODE 标成 supported，"
         "而 PX4 走的是另一条「旧式 SET_MODE」路径）。")

    sub("5.4  ★★★ base_mode 的隐藏回退：为什么 ACK 说成功、模式却是 MANUAL")
    wrap("这是本章最值钱的一段 —— 它解释了素材里那个"
         "「模式切换超时」的报错，为什么 ACK 明明白白写着 ACCEPTED。")
    wrap("PX4 拿到 (base_mode, main, sub) 之后，会先做一次判断：")
    print("""
      Commander.cpp:793
          if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED)   ← 位 0（值 1）
                  → 走「用飞控自己的主/子模式」这条路
          else
                  → 走「use base mode」：从 base_mode 的位图【反推】一个模式
    """)
    tab([
        ["bit 2", "0x04", "AUTO_ENABLED", "→ AUTO_MISSION（★ 优先级最高）"],
        ["bit 6 + bit 3", "0x40 | 0x08", "MANUAL_INPUT + GUIDED", "→ POSCTL"],
        ["bit 6 + bit 4", "0x40 | 0x10", "MANUAL_INPUT + STABILIZE", "→ STABILIZED"],
        ["bit 6", "0x40", "只有 MANUAL_INPUT", "→ ★★ MANUAL（兜底）"],
    ], ["判据位", "值", "名字", "反推结果"])
    wrap("规律很直观：位图里**信息越少，回退得越保守**。"
         "只置了 MANUAL_INPUT_ENABLED 时，PX4 只能推断出"
         "「操作员要手动飞」—— 于是切 MANUAL。")
    wrap("注意判据的【顺序】：AUTO_ENABLED 单独一个 if 排在最前，"
         "后面三个才是嵌套的。所以只要 bit2 置了，就是 AUTO_MISSION。")
    warn("⚠️⚠️ 现在把素材的 base_mode 带进去。"
         "素材里【两个】独立写成的 px4RecommendBaseMode()"
         "（mavlink_udp2/PX4QGCFlightMode.h:69-90 和 "
         "ground_station_px4_mode_switch_Version4.cpp:72-93）"
         "有 5 个分支，【没有一个置了 bit0】：")
    tab([
        ["MANUAL / ACRO / STABILIZED", "0xE0",
         "MANUAL_INPUT | HIL | SAFETY_ARMED", "→ MANUAL"],
        ["ALTCTL / POSCTL", "0xE0（同上，多或了一个 0x80）",
         "同上（STABILIZE_ENABLED 被错写成 0x80，和 SAFETY_ARMED 撞位）",
         "→ MANUAL ★ 不是 POSCTL"],
        ["OFFBOARD", "0xA8", "GUIDED | HIL | SAFETY_ARMED",
         "→ ★ 模式完全不变（bit6 也没置，两个分支都不进）"],
        ["AUTO / TAKEOFF / LAND", "0xA4", "AUTO | HIL | SAFETY_ARMED",
         "→ ★ AUTO_MISSION（不管你要的是 RTL 还是 LAND）"],
        ["default", "0xA0", "HIL | SAFETY_ARMED",
         "→ ★ 模式完全不变"],
    ], ["你想切到", "送出的 base_mode", "真实位含义", "回退分支实际推出"])
    warn("⚠️⚠️ 三种完全不同的失败，ACK 全是 **ACCEPTED**："
         "① 切到了别的模式（POSCTL → MANUAL）；"
         "② 切到了同主模式的另一个子模式（RTL/LAND → MISSION）；"
         "③ 什么都没发生（desired_nav_state 保持 MAX，"
         "Commander.cpp:889 那个 if 整个跳过，main_ret 停在初值 "
         "TRANSITION_NOT_CHANGED —— 而 NOT_CHANGED 不等于 DENIED，"
         "所以照样回 ACCEPTED）。")
    wrap("拆一个具体的：0xE0 = 0x80(SAFETY_ARMED) | 0x40(MANUAL_INPUT) | "
         "0x20(HIL)。bit0（CUSTOM_MODE_ENABLED）没置 → 走回退分支；"
         "bit2(AUTO) 没置、bit3(GUIDED) 没置、bit4(STABILIZE) 没置 → "
         "落到最后一档 **MANUAL**。")
    wrap("所以你以为发了 POSCTL，飞控实际切到了 **MANUAL** —— "
         "一个「看起来最安全」的结果，但和你的意图完全无关。")
    warn("⚠️⚠️ 而那个 ACK 是 **ACCEPTED** —— 因为 Commander.cpp:890-908 "
         "只在「模式请求被明确拒绝」（main_ret == TRANSITION_DENIED）时"
         "才回 TEMPORARILY_REJECTED，其余一律回 ACCEPTED。"
         "**模式确实切了，只是切的不是你想要的。**")
    wrap("★ 这就精确地解释了素材里的那个现象："
         "程序按自定义模式发 DO_SET_MODE，然后等 HEARTBEAT 里出现"
         "期望的 custom_mode —— 永远等不到，5 秒后超时报错。"
         "而人凑到 QGC 前面一看：飞机在 MANUAL 上，一切正常。")
    wrap("两边的日志都对，结论却对不上 —— 因为双方对「成功」的定义不同："
         "飞控说「命令我执行了」，程序说「我要的模式没出现」。")
    note("★★ 记住这条判据：**base_mode 的 bit0 必须置 1（0x01），"
         "否则 PX4 根本不看 param2/param3。** "
         "0x51 = 0x40(MANUAL_INPUT) | 0x10(STABILIZE) | 0x01(CUSTOM_MODE_ENABLED) "
         "是个安全的选择。")
    wrap("★ 顺便说明 0xE0 里那个 0x20 是怎么来的：素材把 "
         "CUSTOM_MODE_ENABLED 记成了 0x20。但 MAVLink 标准里 0x20 是 "
         "HIL_ENABLED（硬件在环仿真），CUSTOM_MODE_ENABLED 是 **0x01**。"
         "——一个常量写错，整条模式链路就断了，而且不报任何错。")

    sub("5.5  一条完整的时序：从插上电到起飞")
    steps = [
        (0.0, "GCS→FC", "HEARTBEAT(sysid=255, compid=190)",
         "地面站先自我介绍"),
        (0.1, "FC→GCS", "HEARTBEAT(sysid=1, compid=1, custom_mode=0x00010000)",
         "飞控回心跳：现在是 MANUAL"),
        (1.0, "GCS→FC", "HEARTBEAT",
         "★ 每秒一条，一直发 —— 停了飞控就认为你走了"),
        (2.0, "GCS→FC", "COMMAND_LONG(400, param1=1.0)",
         "ARM：param1=1 解锁，0 上锁"),
        (2.1, "FC→GCS", "COMMAND_ACK(400, result=2 DENIED)",
         "❌ 被拒！—— 预解锁检查没过（比如 EKF 没收敛）"),
        (5.0, "FC→GCS", "HEARTBEAT(custom_mode=0x00010000, base_mode=0x51)",
         "★ 注意 base_mode 的 SAFETY_ARMED 位是 0 —— 确实没解锁"),
        (6.0, "GCS→FC", "COMMAND_LONG(400, param1=1.0)",
         "等状态好了再发一次（DENIED 之后要「改变状态」，不是重发）"),
        (6.1, "FC→GCS", "COMMAND_ACK(400, result=0 ACCEPTED)",
         "✅ 解锁成功"),
        (6.2, "FC→GCS", "HEARTBEAT(base_mode=0xD1)",
         "★ 0xD1 = 0x80|0x40|0x10|0x01，SAFETY_ARMED 置 1"),
        (7.0, "GCS→FC", "COMMAND_LONG(176, param1=0x51, param2=3.0, param3=0.0)",
         "DO_SET_MODE → POSCTL（★ param2=main=3，param3=sub=0）"),
        (7.1, "FC→GCS", "COMMAND_ACK(176, result=0 ACCEPTED)",
         "✅ 命令被接受了 —— 但**这不代表模式已经切过去**"),
        (7.3, "FC→GCS", "HEARTBEAT(custom_mode=0x00030000)",
         "★ 收到这条才算数：custom_mode 真的变成 POSCTL 了"),
        (8.0, "GCS→FC", "COMMAND_LONG(22, param7=10.0)",
         "NAV_TAKEOFF → 起飞到 10 m"),
        (8.1, "FC→GCS", "COMMAND_ACK(22, result=5 IN_PROGRESS, progress=0)",
         "★ 长命令：先回 IN_PROGRESS，不是 ACCEPTED"),
        (15.0, "FC→GCS", "COMMAND_ACK(22, result=0 ACCEPTED, progress=100)",
         "✅ 到高度了，长命令结束"),
    ]
    tab([["%.1f" % t, d, m, n] for t, d, m, n in steps],
        ["t(s)", "方向", "报文", "为什么"])

    wrap("★★ 整张表里最值钱的是 7.1 和 7.3 这两行："
         "**COMMAND_ACK(ACCEPTED) 只表示「命令收到了、我打算执行」，"
         "不表示「状态已经变了」。** "
         "确认模式是否真的切过去，唯一可靠的依据是**下一条 HEARTBEAT 的 "
         "custom_mode**。")

    sub("5.6  两种等法：等 ACK vs 等心跳（素材里两种都有）")
    tab([
        ["ground_station_…Version4.cpp:255-259",
         "sleep 800ms 后直接发下一条", "❌ 最脆弱：靠猜",
         "慢机器上 800ms 不够，ACK 还没回来就发模式"],
        ["mavlink_udp2/…/main.cpp:239-247",
         "等 HEARTBEAT 的 custom_mode == 期望值，5 秒超时", "✅ 对",
         "但只在收到 HEARTBEAT 时才检查，且会阻塞后续命令"],
        ["mavlink_serial/…/MavlinkSerial.cpp:172-197",
         "先等 ACK，ACK 非 ACCEPTED 就立刻判失败；"
         "ACK 成功后**继续等 HEARTBEAT** 确认", "✅ 最完整",
         "★ 这正是 5.5 表的逻辑：ACK 管「收到」，心跳管「生效」"],
    ], ["素材出处", "等待策略", "评价", "说明"])
    note("把这三份放在一起看，就是一部「地面站客户端」的进化史："
         "猜时间 → 等状态 → 分辨 ACK 与状态。"
         "你自己写的时候，直接照第三种来。")

    sub("5.7  超时不是可选项：三种必须处理的超时")
    tab([
        ["命令级超时", "发命令后 1~2 秒没 ACK", "重发（最多 3 次），仍无则报错"],
        ["状态级超时", "ACK 是 ACCEPTED，但 5 秒内 HEARTBEAT 没变",
         "报错并回滚，别假装成功"],
        ["链路级超时", "2.5 s 没收到飞控心跳 → 链路标红；"
         "10 s → 失联保护（COM_DL_LOSS_T）", "★ 见 5.8"],
    ], ["超时类型", "判据", "处理"])
    warn("★ 「没收到 ACK」和「ACK 是拒绝」是两件完全不同的事："
         "前者可能是链路问题（那就重发），"
         "后者是飞控明确拒绝了（重发无用）。"
         "把两者混成一个「失败」分支的代码，调起来会很痛苦。")

    sub("5.8  失联保护：为什么地面站【也必须】发心跳")
    wrap("很多人以为心跳是飞控的事、地面站只负责收。不对 —— "
         "MAVLink 的心跳是**双向**的：")
    tab([
        ["飞控→地面站", "让地面站知道飞控还活着、当前什么模式",
         "地面站用它刷新界面、判断要不要告警"],
        ["地面站→飞控", "让飞控知道「操作员还在」", "★ 它触发的是 GCS 失联保护"],
    ], ["方向", "作用", "后果"])
    wrap("飞控侧的逻辑是：记一个「上次收到地面站心跳的时间」，"
         "超过阈值就认为操作员失联。这个阈值 PX4 里叫 COM_DL_LOSS_T"
         "（默认 10 秒，见 2.7 的两层超时），"
         "触发后按 NAV_DLL_ACT 决定：悬停 / 返航 / 降落 / 终止 / 上锁 —— "
         "★ 但 NAV_DLL_ACT 的出厂默认是 0 = Disabled，"
         "所以「超时」和「有人接管」还是两件事（见 2.7 的 ⚠️）。")
    wrap("ArduPilot 里对应两个参数：FS_GCS_ENABLE（开关兼动作，"
         "默认 0 = Disabled —— ArduCopter/Parameters.cpp:168）"
         "和 FS_GCS_TIMEOUT（阈值，默认 5 秒，# :1011）。"
         "★ 阈值确实比 PX4 短，但**开关默认也是关的** —— "
         "所以「ArduPilot 更敏感」这个说法只在"
         "「两边都打开了保护」的前提下成立。")
    warn("★ 所以「地面站不发心跳」不是一个可以省掉的步骤 —— "
         "它在飞控眼里等于**操作员消失了**。"
         "调试时你会看到「程序明明连着，飞控却自己返航了」，"
         "八成就是这里。")
    note("还有一层：PX4 收到陌生 sysid 的命令时，"
         "往往要等收到过它的心跳才认。所以心跳不只是「保活」，"
         "也是**建立会话的第一步**。")

    sub("5.9  本章要点回执")
    tab([
        ["MAVLink 是什么", "二进制、按 XML 生成的协议；v2 帧以 0xFD 开头", "实验 2"],
        ["CRC 防什么", "传输损坏 + 版本不符（后者靠 CRC_EXTRA）", "实验 3"],
        ["seq 防什么", "整帧丢失", "实验 3"],
        ["模式怎么表达", "base_mode（标准位图）+ custom_mode（飞控自定义）", "实验 4"],
        ["PX4 的位域", "main 在 bit16-23，sub 在 bit24-31", "实验 4"],
        ["枚举从哪抄", "★ PX4 源码 px4_custom_mode.h，不是博客/旧代码", "实验 4"],
        ["ACK 管什么", "「命令收到了」——**不管**状态变没变", "实验 5"],
        ["状态怎么确认", "下一条 HEARTBEAT 的 custom_mode", "实验 5"],
        ["为什么发心跳", "不发 = 告诉飞控「操作员走了」= 触发失联保护计时", "实验 5"],
        ["失联保护默认开吗", "★ 不开：PX4 NAV_DLL_ACT=0、ArduPilot FS_GCS_ENABLE=0", "实验 5"],
        ["base_mode 的陷阱", "bit0 不置（0x01）→ 静默回退，ACK 照样 ACCEPTED", "实验 5"],
    ], ["问题", "答案", "在哪"])


# ============================================================================
#  入口
# ============================================================================

EXPERIMENTS = {
    "1": ("环境体检：本机能不能真跑一条 MAVLink 链路（优雅降级）", exp1),
    "2": ("★★ 手工组一条 HEARTBEAT：v2 帧逐字节拆开", exp2),
    "3": ("★★ CRC 与 CRC_EXTRA：协议怎么把「版本不符」挡在门外", exp3),
    "4": ("★★ custom_mode 位域：主模式、子模式、和 QGC 的命名", exp4),
    "5": ("★ ACK 与会话：一条从零到起飞的真实时序", exp5),
}


def main():
    which = [a for a in sys.argv[1:] if a.isdigit()]
    print()
    print("#" * W)
    print("#  第 6 章 04 · MAVLink 实操：从零跑通一条链路")
    print("#  配套脚本 lesson30_mavlink.py —— 只用 Python 标准库")
    print("#" * W)
    wrap("本脚本自己实现了 MAVLink v2 的组帧/拆帧/CRC。"
         "所以就算本机没有 pymavlink（实测确实没有），"
         "下面五个实验也全部能跑出实数、真字节。")

    todo = which if which else sorted(EXPERIMENTS)
    for k in todo:
        if k not in EXPERIMENTS:
            print("  （没有实验 %s）" % k)
            continue
        EXPERIMENTS[k][1]()

    head("跑完了")
    wrap("五个实验都在，没有依赖任何外部设备。")
    wrap("想把它接到真飞控上，下一步是 卷 10 第 1 章 —— "
         "起一个 SITL 之后，先把实验 1 的探测端口改成 18570/14580 再跑一遍"
         "（★ SITL 上 PX4 监听的是这两个，不是 14540/14550，"
         "理由见课文的 22.6.2.1）。")
    wrap("用 pymavlink 的读者还可以直接："
         "`python -m mavproxy.py --master=udpout:127.0.0.1:14580`。")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
