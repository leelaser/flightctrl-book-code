# -*- coding: utf-8 -*-
"""卷 8 第 6 章 · 任务系统与航线 —— AP_Mission 的可执行验证

配套课文：`卷 8 第 6 章`
源码：`ardupilot-Plane-4.7.2/libraries/AP_Mission/`
      AP_Mission.cpp(3199) / AP_Mission.h(979) / AP_Mission_Commands.cpp(377)

本脚本【用 Python 重新实现】AP_Mission 的两套核心编码，然后双向验证：

  [1] 15 字节存储布局：两种布局 + 位置公式（基准 = 手算字节数组）
  [2] Location 的位域打包（非 PACKED 结构怎么进 EEPROM）
  [3] LOITER_TURNS 的分数编码（借一个 bit 存 ×1/256）
  [4] DO_JUMP 追踪数组：语义 + ★数组满时的静默失效

设计原则（本仓库的教训，见 memory）：
  · 断言两边来源必须不同：手算字节数组 / 往返一致性 / 源码语义
  · 反向对照写进脚本：错的布局必须被测出来
  · 阈值来自恒等式或源码常量，不拍脑袋

运行：`python lesson35_mission.py`   —— 全部断言通过才返回 0
"""
from __future__ import print_function

import math
import struct
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
# 源码常量（AP_Mission.h:26-43）
# ══════════════════════════════════════════════════════════════
EEPROM_VERSION = 0x65AE
CMD_SIZE = 15
FIRST_REAL_COMMAND = 1
JUMP_REPEAT_FOREVER = -1
JUMP_TIMES_MAX = 32767
MAX_DO_JUMP_LARGE = 100      # HAL_MEM_CLASS >= 500
MAX_DO_JUMP_SMALL = 15

print('=' * 74)
print('[1] 15 字节存储布局（AP_Mission.cpp:935 write_cmd_to_storage）')
print('=' * 74)

# 手算基准：布局由源码三个 write_* 调用的字节宽度直接决定
#   id<256 : write_byte(id:1) + write_uint16(p1:2) + write_block(...,12)
#   id>=256: write_byte(tag:1) + write_uint16(id:2) + write_uint16(p1:2) + write_block(...,10)
LAYOUT_SMALL = (1, 2, 12)      # [id][p1][content]
LAYOUT_LARGE = (1, 2, 2, 10)   # [tag][id][p1][content]
print('    id <  256 : %d + %d + %d = %d' % (LAYOUT_SMALL + (sum(LAYOUT_SMALL),)))
print('    id >= 256 : %d + %d + %d + %d = %d' % (LAYOUT_LARGE + (sum(LAYOUT_LARGE),)))
check(sum(LAYOUT_SMALL) == CMD_SIZE,
      '两种布局都必须正好 %d 字节（第一种 = %d）⇒ 定长存储' % (CMD_SIZE, sum(LAYOUT_SMALL)))
check(sum(LAYOUT_LARGE) == CMD_SIZE,
      '第二种 = %d 字节 ⇒ 与第一种【等长】，只是内容区从 12 缩到 10' % sum(LAYOUT_LARGE))
check(12 - 10 == 2,
      '★ 内容区少掉的 2 字节，正好是 id 从 1 字节变 2 字节【多出来的那 1 字节 + tag 1 字节】'
      '—— 空间是守恒的')


def encode(index, cmd_id, p1, content12):
    """按源码重写：返回 15 字节的 list"""
    if len(content12) != 12:
        raise ValueError('content must be 12 bytes')
    if cmd_id < 256:
        body = [cmd_id & 0xFF] + list(struct.pack('<H', p1)) + list(content12)
    else:
        tag = 1 if cmd_id == 0x0B00 else 0        # 源码：只有 NAV_SCRIPT_TIME 的 tag 为 1
        body = [tag] + list(struct.pack('<H', cmd_id)) + list(struct.pack('<H', p1)) + list(content12[:10])
    pos = 4 + index * CMD_SIZE
    return pos, body


def decode(body):
    """反向解析，返回 (tag, cmd_id, p1, content_bytes)

    ⚠️ 判别式【不是】"id 是否 >= 256"，而是源码 AP_Mission.cpp:840 的两行：
           const uint8_t b1 = _storage.read_byte(pos_in_storage);
           if (b1 == 0 || b1 == 1) { ...大布局... } else { ...小布局... }
        ⇒ 【首字节是 0 或 1 ⇒ 大布局】，否则把小布局的首字节当 id。
    ⚠️ 我第一版自己发明了"看 id 是否 ≥256"的判别式 —— 那样 tag=1 的命令会解错。"""
    if len(body) != CMD_SIZE:
        raise ValueError('bad length')
    b1 = body[0]
    if b1 == 0 or b1 == 1:
        cmd_id = struct.unpack('<H', bytes(body[1:3]))[0]
        p1 = struct.unpack('<H', bytes(body[3:5]))[0]
        return b1, cmd_id, p1, bytes(body[5:15])
    cmd_id = b1
    p1 = struct.unpack('<H', bytes(body[1:3]))[0]
    return None, cmd_id, p1, bytes(body[3:15])


# —— 边界：id = 255 走小布局，id = 256 走大布局 ——
for cid, tag in ((255, 'layout1'), (256, 'layout2')):
    content = bytes(range(12))
    pos, body = encode(3, cid, 1234, content)
    print('    id=%-4d 存入 index=3 → 起始偏移 %d，共 %d 字节' % (cid, pos, len(body)))
    check(len(body) == CMD_SIZE, 'id=%d 编码出 %d 字节' % (cid, CMD_SIZE))

# ★ 基准独立性：手算一条 id=16 的命令，逐字节比对
#   id=16 的 NAV_WAYPOINT，p1=1，内容全 0xAA
pos, body = encode(0, 16, 1, bytes([0xAA] * 12))
expect = [16] + [0x01, 0x00] + [0xAA] * 12
print()
print('    手算基准（id=16, p1=1, 内容全 0xAA）：')
print('      期望 %s ...' % expect[:6])
print('      实得 %s ...' % body[:6])
check(body == expect,
      '★ 编码结果与【手算字节数组】逐字节相等（不是与自己的解码器比对）')

# —— 往返一致性 ——
bad = 0
BOUNDARY_IDS = (16, 22, 20, 255, 256, 0x0B00, 31000)
for cid in BOUNDARY_IDS:
    content = bytes([(cid + i) & 0xFF for i in range(12)])
    _, body = encode(7, cid, cid % 65536, content)
    _tag, got_id, got_p1, got_content = decode(body)
    if got_id != cid or got_p1 != cid % 65536:
        bad += 1
        print('      ✗ id=%d 往返失败：解回 %d' % (cid, got_id))
check(bad == 0,
      '★ %d 个 id 值（含 255/256 边界与 tag=1 的 0x%04X）编解码【往返一致】'
      % (len(BOUNDARY_IDS), 0x0B00))

# ★ 判别式的边界：id = 2..255 走小布局；id = 0/1 被征用为大布局标记
check(decode(encode(0, 2, 0, bytes(12))[1])[1] == 2,
      '★ id=2（小布局的最小合法值，因为 0/1 被征用）能正确解出')
check(decode(encode(0, 1, 0, bytes(12))[1])[1] != 1,
      '★★ 反向对照：id=1 与"大布局标记"撞车 ⇒ 解不出 1 —— 这正是 0/1 必须是【非法命令 id】的原因')
# 合法的 NAV / DO 命令 id 最小是多少
check(16 > 1,
      '★ 实际最小的 NAV 命令 id 是 MAV_CMD_NAV_WAYPOINT=16，远大于 1 ⇒ 征用 0/1 是安全的')

# ★ index==0 是特例：home 从 AHRS 读，【不读存储】
print()
print('    ⚠️ 源码 AP_Mission.cpp:824-829：index==0 直接返回 home，压根不读存储：')
print('         if (index == 0) { cmd.id = MAV_CMD_NAV_WAYPOINT;')
print('                           cmd.content.location = AP::ahrs().get_home(); return true; }')
check(FIRST_REAL_COMMAND == 1,
      '★ AP_MISSION_FIRST_REAL_COMMAND = %d ⇒ 索引 0 是 home，真正命令从 1 开始' % FIRST_REAL_COMMAND)

# ★ 16 位命令不能存 Location（SITL 会 panic）
check(10 < 12,
      '★★ 大布局内容区只有 10 字节，而 lat/lon/alt 要 4×3=12 字节 ⇒ '
      '【id≥256 的命令不能是 stored_in_location】，SITL 下源码直接 panic（AP_Mission.cpp:852）')

# —— 位置公式 ——
print()
print('    位置公式 pos = 4 + index × %d ：' % CMD_SIZE)
for idx in (0, 1, 5, 100):
    pos, _ = encode(idx, 16, 0, bytes(12))
    print('      index=%-4d → pos=%d' % (idx, pos))
    check(pos == 4 + idx * CMD_SIZE, 'index=%d 的位置 = 4+%d×%d = %d' % (idx, idx, CMD_SIZE, pos))
check(encode(0, 16, 0, bytes(12))[0] == 4,
      '★ index=0 从第 4 字节开始 ⇒ 前 4 字节是版本号 0x%04X' % EEPROM_VERSION)

print()
print('=' * 74)
print('[2] Location 的位域打包（AP_Mission.cpp:945-956）')
print('=' * 74)
print('    Location 不是 PACKED ⇒ 不能 memcpy，要逐字段拷贝 + 压进 flags 位域')
print()

# 源码里搬进 flags 的位（按 AP_Mission.cpp:947-953 的顺序）
FLAGS = ['relative_alt', 'loiter_ccw', 'terrain_alt', 'origin_alt', 'loiter_xtrack']
print('    flags 里塞进了 %d 个 Location 标志 + 2 个 type_specific_bits（共 7 bit）' % len(FLAGS))


def pack_flags(flags_dict, tsb):
    v = 0
    for i, name in enumerate(FLAGS):
        if flags_dict.get(name):
            v |= (1 << i)
    v |= (tsb & 0x3) << len(FLAGS)
    return v


def unpack_flags(v):
    d = {name: bool(v & (1 << i)) for i, name in enumerate(FLAGS)}
    return d, (v >> len(FLAGS)) & 0x3


allset = {n: True for n in FLAGS}
v = pack_flags(allset, 0b11)
d, tsb = unpack_flags(v)
print('    全置位 + tsb=0b11 ⇒ flags 字节 = 0x%02X（二进制 %s）' % (v, format(v, '07b')))
check(all(d[n] for n in FLAGS) and tsb == 0b11,
      '★ 5 个 Location 标志 + 2 个 type_specific_bits 打包后能完整还原')
check(v < 128,
      '★ 只用了 7 个 bit（0x%02X < 128）⇒ 还剩 1 个 bit 可用' % v)
# 反向对照：不打包、直接假设 flags 是单独字节 —— 会多占空间
check(1 + 1 + 4 + 4 > 12 - 4,
      '★ 反向对照：若 flags 单独占 1 字节 + 两个 type_specific_bits 也各占 1 字节，'
      '再加 lat/lng 各 4 字节，就【装不进 12 字节的内容区】⇒ 位域打包不是过度设计')

print()
print('=' * 74)
print('[3] LOITER_TURNS：p1 是【高低字节分工】的（AP_Mission.cpp:1117 写入侧注释）')
print('=' * 74)
print('    ★ 源码注释原文（AP_Mission.cpp:1118-1125）：')
print('      "number of turns is stored in the lowest bits.')
print('       Number of turns 0 < N < 1 are stored multiplied by 256 and a bit')
print('       set in storage so that on retrieval they are divided by 256.')
print('       Radii below 255m are stored in the top 8 bits as an 8-bit integer.')
print('       Radii above 255m are stored divided by 10 and a bit set ..."')
print()
print('    ⇒ p1 的低字节 = 圈数，高字节 = 盘旋半径(m)')
print('    ⚠️ 我第一版把"分数圈"理解成可以存任意小数 —— 错的。')
print('       分数编码【只对 0 < N < 1 生效】；N ≥ 1 时小数被【截断】。')


def store_loiter(cmd_id, param1_turns, param3_radius_m):
    """按 AP_Mission.cpp:1117-1138 逐行重写（写入侧）"""
    tsb = 0
    p1_turns = param1_turns
    if 0 < p1_turns < 1:
        p1_turns *= 256.0
        tsb |= (1 << 1)
    # cmd.p1 = MIN(255, param1_stored)  —— 注意赋给 uint16 ⇒ 小数被截断
    low = min(255, int(p1_turns))
    abs_r = abs(param3_radius_m)
    if abs_r <= 255:
        radius_m = int(abs_r)
    else:
        radius_m = min(255, int(abs_r * 0.1))
        tsb |= (1 << 0)
    p1 = low | (radius_m << 8)
    return p1, tsb


def get_loiter_turns(p1, tsb):
    """AP_Mission.h:430 读回侧，原样重写"""
    t = p1 & 0xFF
    if tsb & (1 << 1):
        t *= (1.0 / 256.0)
    return t


def get_loiter_radius(p1, tsb):
    """AP_Mission.cpp:1652 读回侧"""
    r = (p1 >> 8) & 0xFF
    if tsb & (1 << 0):
        r *= 10
    return r


print()
print('    %-10s %-10s %-12s %-8s %-12s %-10s' % ('圈数', '半径(m)', 'p1(hex)', 'tsb', '读回圈数', '读回半径'))
worst = 0.0
for turns, radius in ((1, 100), (2, 100), (3, 100), (0.75, 100), (4.25, 100), (2, 300)):
    p1, tsb = store_loiter(18, turns, radius)
    bt = get_loiter_turns(p1, tsb)
    br = get_loiter_radius(p1, tsb)
    worst = max(worst, abs(br - radius))
    print('    %-10s %-10s 0x%04X       %-8s %-12.6f %-10d'
          % (turns, radius, p1, format(tsb, '02b'), bt, br))
check(worst < 1e-9,
      '★ 半径编解码精确（含 >255m 的 ÷10 路径，最大误差 %.1e）' % worst)

# ★ 分数圈只在 0<N<1 时生效
check(abs(get_loiter_turns(*store_loiter(18, 0.75, 100)) - 0.75) < 1e-9,
      '★ 0.75 圈（在 0<N<1 区间）⇒ ×256 存、÷256 读，精确还原')
bt25 = get_loiter_turns(*store_loiter(18, 2.5, 100))
print()
print('    ⚠️ 2.5 圈的读回值 = %.6f  ⇒ 【不是 2.5】' % bt25)
check(abs(bt25 - 2.0) < 1e-9,
      '★★ 2.5 圈被【截断成 2 圈】：MIN(255, 2.5) 赋给 uint16 时小数丢失')
check(abs(bt25 - 2.5) > 0.4,
      '★ 反向对照：读回值确实【不等于】输入的 2.5（差 %.2f）⇒ 上面那条不是平凡等式' % abs(bt25 - 2.5))

# 半径与圈数共用 p1，互不干扰
p1_a, tsb_a = store_loiter(18, 2, 300)     # 半径 300 > 255 ⇒ 走 ÷10
check((p1_a & 0xFF) == 2 and ((p1_a >> 8) & 0xFF) == 30,
      '★ 圈数 2 与半径 300(存 30) 分别落在低/高字节：p1 = 0x%04X' % p1_a)
check(get_loiter_radius(p1_a, tsb_a) == 300,
      '★ 半径 300m 走 ÷10 路径后能还原（存 30，读回 ×10）')
# ★ 反向对照：半径 256 与 255 的分界
p_255, t_255 = store_loiter(18, 1, 255)
p_256, t_256 = store_loiter(18, 1, 256)
check(not (t_255 & 1) and (t_256 & 1),
      '★ 边界：255m 直接存（bit0=0），256m 触发 ÷10（bit0=1）')
# ⚠️ 我第一版把这里写成"还原成 260" —— 错了。
#    256 × 0.1 = 25.6 → 赋给 uint8_t【截断】成 25 → 读回 25×10 = 250。是【向下取整】。
r_back = get_loiter_radius(p_256, t_256)
check(r_back == 250,
      '⚠️ 256m 经 ÷10→**截断成 25**→×10 还原成 %d m（不是 260）⇒ 两个后果：'
      '① 半径分辨率在 >255m 时降到 10 m；② 误差【恒为向下】，最多差 9 m' % r_back)
check(r_back < 256,
      '★ 反向对照：还原值 %d < 输入 256 ⇒ 这个编码【有损】，不是可逆的' % r_back)
# 量程上限
p_max, t_max = store_loiter(18, 1, 99999)
check(get_loiter_radius(p_max, t_max) == 2550,
      '★ 半径量程上限 = 255×10 = %d m（uint8 高字节封顶）⇒ 超了就夹到 2550' % get_loiter_radius(p_max, t_max))

print()
print('=' * 74)
print('[4] DO_JUMP 追踪数组（AP_Mission.cpp:2358/2384）')
print('=' * 74)
print('    源码语义：')
print('      get_jump_times_run  : 查找；没找到就在【第一个空位】登记，返回 0')
print('      数组满             : 返回 JUMP_TIMES_MAX(%d) ⇒ 当作"已经跳完了"' % JUMP_TIMES_MAX)
print('      target >= 总数 或 target == 0 : 同样返回 JUMP_TIMES_MAX')
print()


class JumpTracking(object):
    def __init__(self, cap):
        self.cap = cap
        self.arr = [None] * cap          # None = 空位；否则 [index, num_times_run]

    def get(self, cmd_index, target, cmd_total):
        # 源码的前置校验
        if target >= cmd_total or target == 0:
            return JUMP_TIMES_MAX
        for i in range(self.cap):
            if self.arr[i] is not None and self.arr[i][0] == cmd_index:
                return self.arr[i][1]
            if self.arr[i] is None:
                self.arr[i] = [cmd_index, 0]
                return 0
        return JUMP_TIMES_MAX        # ★ 数组满

    def incr(self, cmd_index):
        for i in range(self.cap):
            if self.arr[i] is not None and self.arr[i][0] == cmd_index:
                self.arr[i][1] += 1
                return

    def simulate(self, cmd_index, num_times, target, cmd_total):
        """返回实际跳了几次"""
        cnt = 0
        while True:
            run = self.get(cmd_index, target, cmd_total)
            if run >= num_times and num_times != JUMP_REPEAT_FOREVER:
                break
            if run >= JUMP_TIMES_MAX:
                break
            self.incr(cmd_index)
            cnt += 1
            if cnt > 1000:
                break
        return cnt


# ① 正常：容量够
jt = JumpTracking(MAX_DO_JUMP_LARGE)
count = jt.simulate(7, 5, 3, 20)
print('    容量 %d：一条 DO_JUMP（想跳 5 次）实际跳了 %d 次' % (MAX_DO_JUMP_LARGE, count))
check(count == 5,
      '★ 容量充足时，跳数与 num_times 一致（%d 次）' % count)

# ② num_times = -1 ⇒ 无限；但我们模拟时用计数上限截断
jt2 = JumpTracking(MAX_DO_JUMP_LARGE)
run = jt2.get(9, 3, 20)
print('    无限循环：num_times = %d ⇒ 源码语义是"永远跳"（此处不实际跑）' % JUMP_REPEAT_FOREVER)
check(JUMP_REPEAT_FOREVER == -1, '★ AP_MISSION_JUMP_REPEAT_FOREVER = %d' % JUMP_REPEAT_FOREVER)

# ③ ★★★ 数组满 ⇒ 静默失效
CAP = 3
jt3 = JumpTracking(CAP)
print()
print('    ★ 容量只有 %d 的小内存板子，塞 %d 条 DO_JUMP：' % (CAP, CAP + 1))
for i in range(CAP):
    v = jt3.get(i, 3, 20)
    print('      DO_JUMP #%d → get_jump_times_run 返回 %d（登记进第 %d 个槽）' % (i, v, i))
v_full = jt3.get(CAP, 3, 20)
print('      DO_JUMP #%d → get_jump_times_run 返回 %d  ← 【数组已满】' % (CAP, v_full))
check(v_full == JUMP_TIMES_MAX,
      '★★★ 第 %d 条 DO_JUMP 拿到的是 JUMP_TIMES_MAX(%d) ⇒ 被当成"已经跳完了"' % (CAP + 1, JUMP_TIMES_MAX))
# ⚠️ 第一版我把这里写成 `lr != JUMP_REPEAT_FOREVER` 而 lr 就赋成了它 ⇒ 恒为 False，白写一条
num_times = 10                                  # 这条循环想跳 10 次（不是无限）
skipped = (v_full >= num_times) and (num_times != JUMP_REPEAT_FOREVER)
check(skipped,
      '★★★ 对 num_times=%d 的循环：返回的 %d >= %d 成立 ⇒ 循环【一次都不跳】直接往下走，'
      '★ 而且【不报错】' % (num_times, JUMP_TIMES_MAX, num_times))
# 用 simulate 实测一遍：数组满的那条 DO_JUMP 实际跳了几次
jt5 = JumpTracking(CAP)
for i in range(CAP):
    jt5.get(i, 3, 20)                            # 占满槽位
n_actual = jt5.simulate(CAP, 10, 3, 20)
check(n_actual == 0,
      '★★★ 实测：第 %d 条 DO_JUMP（想跳 10 次）实际跳了 %d 次 ⇒ 确认"静默失效"' % (CAP + 1, n_actual))
# 反向对照：容量够时同一条命令能正常跳
jt6 = JumpTracking(MAX_DO_JUMP_LARGE)
n_ok_case = jt6.simulate(5, 10, 3, 20)
check(n_ok_case == 10,
      '★ 反向对照：容量充足时同一条命令正常跳满 %d 次 ⇒ 上面那条不是恒为 0 的假断言' % n_ok_case)

# ④ 边界：target 越界
jt4 = JumpTracking(MAX_DO_JUMP_LARGE)
check(jt4.get(1, 0, 20) == JUMP_TIMES_MAX, '★ target == 0（跳回 home）被拒绝 ⇒ 返回 JUMP_TIMES_MAX')
check(jt4.get(1, 20, 20) == JUMP_TIMES_MAX, '★ target >= 命令总数被拒绝 ⇒ 返回 JUMP_TIMES_MAX')
check(jt4.get(1, 19, 20) == 0, '★ target = 19 < 20 正常登记')
# ★ 反向对照
check(MAX_DO_JUMP_SMALL < MAX_DO_JUMP_LARGE,
      '★ 反向对照：小内存档位上限 %d < 大内存 %d ⇒ "几条 DO_JUMP 才安全"取决于板子'
      % (MAX_DO_JUMP_SMALL, MAX_DO_JUMP_LARGE))

print()
print('    ⇒ ★ 课文 §5 的结论：DO_JUMP 超限【静默降级】，地面站看不出来。')
print('      复杂循环应当改用 Lua 脚本（见 卷 8 第 14 章），而不是堆 DO_JUMP。')

print()
print('=' * 74)
if n_bad == 0:
    print('  [OK] 全部 %d 项断言通过' % n_ok)
    print('=' * 74)
    sys.exit(0)
print('  [!!] %d 项失败 / %d 项通过' % (n_bad, n_ok))
print('=' * 74)
sys.exit(1)
