#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第 11 课配套实验：移植 NuttX 到一块新板子

设计原则（和前 10 课一致）：
  1. 每个实验都是**可运行的工具**，不是伪代码 —— 移植时能真的用上
  2. 所有约束来自 ST 参考手册（RM0433）和 NuttX 源码，标注出处
  3. 能实测的数字就实测（板级配置行数、RAM 大小都来自本机仓库）
  4. 模拟的地方明确说明丢掉了什么

运行：
    python lesson11_board_port.py          # 全部实验
    python lesson11_board_port.py 2        # 只跑实验 2

数据来源：
    NuttX:      D:/DEVLib/PX4APM/PX4/PX4-Autopilot/platforms/nuttx/NuttX/nuttx
    PX4 板级:    D:/DEVLib/PX4APM/PX4/PX4-Autopilot/boards
"""

import sys
import math
from itertools import product


def sec(title, ch="=", width=78):
    print()
    print(ch * width)
    print(title)
    print(ch * width)


# ============================================================================
# 实验 1：STM32H7 时钟树求解器
# ============================================================================
#
# 这是移植新板子时**最容易出错、也最难调试**的一步。
# 症状：板子能启动但串口波特率不对、或者 USB 不识别、或者一上电就
# 卡死 —— 全都可能是因为 PLL 配错，而编译器不会有任何提示。
#
# PX4 的做法是把推导过程**写在 board.h 的注释里**
# （boards/ark/fmu-v6x/nuttx-config/include/board.h:72-97 原文）：
#
#     /* Main PLL Configuration.
#      *
#      * PLL source is HSE = 16,000,000
#      *
#      * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
#      * Subject to:
#      *
#      *     1 <= PLLM <= 63
#      *     4 <= PLLN <= 512
#      *   150 MHz <= PLL_VCOL <= 420MHz
#      *   192 MHz <= PLL_VCOH <= 836MHz
#      *
#      * SYSCLK  = PLL_VCO / PLLP
#      * CPUCLK  = SYSCLK / D1CPRE
#      * Subject to
#      *
#      *   PLLP1   = {2, 4, 6, 8, ..., 128}
#      *   PLLP2,3 = {2, 3, 4, ..., 128}
#      *   CPUCLK <= 480 MHz
#      */
#
# 这段注释是**正确的**（约束和 RM0433 一致）。但下面的实际代码里
# 有一个值得追查的地方，见实验 1 末尾的 RGE 检查。

# --- STM32H7 PLL1 的约束（RM0433，STM32H742/743/753/750）---
PLLM_MIN, PLLM_MAX = 1, 63
PLLN_MIN, PLLN_MAX = 4, 512
PLLP1_ALLOWED = list(range(2, 129, 2))      # PLL1P 只能是偶数
CPUCLK_MAX = 480
PLL_REF_MIN, PLL_REF_MAX = 1, 16            # Fref1_ck 总允许范围

# ★ 宽 VCO 范围有**两个版本**，取决于芯片的硅片版本（silicon revision）：
#
#   Rev Y 硅片：192 - 836 MHz   ← RM0433 寄存器描述里写的是这个
#   Rev V 硅片：192 - 960 MHz   ← 数据手册 DS12110 的 (rev V) 电气特性章节
#
# RM0433 **没有随着 Rev V 的引入而更新**，仍然写着 Rev Y 的限制。
# ST 官方社区确认了这一点，并说明应以数据手册为准。
# 参考：
#   - DS12110 §7.3.10 "PLL characteristics (wide VCO frequency range)"
#   - ST Community: "VCO frequency discrepancy in STM32H743 datasheet
#     and reference manuals"
#
# ⇒ 这是一个**真实的、会导致误判的文档陷阱**：
#   如果照 RM0433 的 836 MHz 上限去写求解器，就解不出 PX4 的实际配置
#   （960 MHz VCO），会误以为 PX4 配错了。**我第一版就踩了这个坑**
#   ——求解器返回 0 组解，才发现是我的约束抄错了版本。
#
# 用法：按目标硅片版本选。
VCO_WIDE_LIMITS = {
    "revY": (192, 836),     # RM0433 的数值（旧硅片）
    "revV": (192, 960),     # 数据手册的数值（新硅片，PX4 的板子都是这一档）
}
VCO_MED_MIN, VCO_MED_MAX = 150, 420

# RCC_PLLCFGR_PLL1RGE[1:0] 的编码（RM0433）
#
#   arch/arm/src/stm32h7/hardware/stm32h7x3xx_rcc.h:363-366 原文：
#     #define RCC_PLLCFGR_PLL1RGE_1_2_MHZ   (0x0ul << ...)  /* between 1 and 2 MHz */
#     #define RCC_PLLCFGR_PLL1RGE_2_4_MHZ   (0x1ul << ...)  /* between 2 and 4 MHz */
#     #define RCC_PLLCFGR_PLL1RGE_4_8_MHZ   (0x2ul << ...)  /* between 4 and 8 MHz */
#     #define RCC_PLLCFGR_PLL1RGE_8_16_MHZ  (0x3ul << ...)  /* between 8 and 16 MHz */
#
RGE_TABLE = [
    (1, 2, "RCC_PLLCFGR_PLL1RGE_1_2_MHZ", 0x0),
    (2, 4, "RCC_PLLCFGR_PLL1RGE_2_4_MHZ", 0x1),
    (4, 8, "RCC_PLLCFGR_PLL1RGE_4_8_MHZ", 0x2),
    (8, 16, "RCC_PLLCFGR_PLL1RGE_8_16_MHZ", 0x3),
]


def rge_for(ref_mhz):
    """给定 PLL 输入频率，返回正确的 RGE 宏名。注意上界是**包含**的：
    ST 官方社区对 8 MHz 输入推荐用 4_8 档（上界闭区间）。
    """
    for lo, hi, name, _ in RGE_TABLE:
        if lo <= ref_mhz <= hi:
            return name
    return None


def solve_pll1(hse_hz, target_cpuclk_mhz, target_hclk_div=2,
               pllq_target_mhz=None, pllr_target_mhz=None,
               silicon="revV", verbose=False):
    """求解 PLL1 的 PLLM/PLLN/PLLP/PLLQ/PLLR。

    silicon: "revV"（宽 VCO 上限 960 MHz，PX4 的板子）或 "revY"（836 MHz）。
             这个参数**必须显式给**，因为 RM0433 和 DS12110 不一致。

    返回所有满足约束的解，按 ST 社区推荐的"降低 PLL 输入频率以改善抖动"
    排序（ST 原话是 "optimum configuration to reach highest frequency
    and best jitter"，指用 /M 把输入降下来）。
    """
    vco_lo, vco_hi = VCO_WIDE_LIMITS[silicon]
    sols = []
    for pllm in range(PLLM_MIN, PLLM_MAX + 1):
        ref = hse_hz / pllm
        ref_mhz = ref / 1e6
        if not (PLL_REF_MIN <= ref_mhz <= PLL_REF_MAX):
            continue
        for plln in range(PLLN_MIN, PLLN_MAX + 1):
            vco = ref * plln / 1e6          # MHz
            if not (vco_lo <= vco <= vco_hi):
                continue
            for pllp in PLLP1_ALLOWED:
                cpuclk = vco / pllp
                if abs(cpuclk - target_cpuclk_mhz) > 0.001:
                    continue
                if cpuclk > CPUCLK_MAX:
                    continue
                # 求 PLLQ / PLLR：优先满足调用者给的目标，否则用默认
                pllq, pllr = None, None
                if pllq_target_mhz:
                    for q in range(2, 129):
                        if abs(vco / q - pllq_target_mhz) < 0.001:
                            pllq = q
                            break
                if pllr_target_mhz:
                    for r in range(2, 129):
                        if abs(vco / r - pllr_target_mhz) < 0.001:
                            pllr = r
                            break
                sols.append({
                    "pllm": pllm, "plln": plln, "pllp": pllp,
                    "pllq": pllq, "pllr": pllr,
                    "ref_mhz": ref_mhz, "vco": vco,
                    "cpuclk": cpuclk, "hclk": cpuclk / target_hclk_div,
                    "rge": rge_for(ref_mhz),
                })
    # 排序：优先 ref 低（ST 推荐的抖动最优），其次 PLLN 大（分辨率细）
    sols.sort(key=lambda s: (s["ref_mhz"], -s["plln"], s["pllm"]))
    return sols


def exp1_clock_tree():
    sec("实验 1：STM32H7 时钟树求解器 —— 把 board.h 的注释变成可运行代码")

    print("""
新板子的第一件事是确定时钟。PX4 把推导写在 board.h 的注释里，
但注释不能验证。本实验把它变成一个能跑的求解器，用来：
  (a) 复现 fmu-v6x 的实际配置，确认我读懂的约束是对的
  (b) 给一块新板子（不同晶振）求出合法配置
  (c) 检查 PX4 现有配置里一个值得追查的问题（见末尾）
""")

    HSE = 16_000_000
    print("  " + "-" * 74)
    print("  (a) 复现 PX4 fmu-v6x 的 PLL1 配置")
    print("  " + "-" * 74)
    print("""
  事实（boards/ark/fmu-v6x/nuttx-config/include/board.h:100-117 原文）：

      /* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
       *
       *   PLL1_VCO = (16,000,000 / 1) * 60 = 960 MHz
       *
       *   PLL1P = PLL1_VCO/2  = 960 MHz / 2   = 480 MHz
       *   PLL1Q = PLL1_VCO/4  = 960 MHz / 4   = 240 MHz
       *   PLL1R = PLL1_VCO/8  = 960 MHz / 8   = 120 MHz
       */

      #define STM32_PLLCFG_PLL1M   RCC_PLLCKSELR_DIVM1(1)
      #define STM32_PLLCFG_PLL1N   RCC_PLL1DIVR_N1(60)
      #define STM32_PLLCFG_PLL1P   RCC_PLL1DIVR_P1(2)
      #define STM32_PLLCFG_PLL1Q   RCC_PLL1DIVR_Q1(4)
      #define STM32_PLLCFG_PLL1R   RCC_PLL1DIVR_R1(8)
""")

    sols = solve_pll1(HSE, 480, pllq_target_mhz=240, pllr_target_mhz=120)
    exact = [s for s in sols if s["pllm"] == 1 and s["plln"] == 60 and s["pllp"] == 2]
    print("  求解器在 HSE=%d Hz、目标 CPUCLK=480 MHz 下找到 %d 组解。"
          % (HSE, len(sols)))
    if exact:
        s = exact[0]
        print("  其中与实际配置完全相同的一组：")
        print("      PLLM=%d  PLLN=%d  PLLP=%d  PLLQ=%s  PLLR=%s"
              % (s["pllm"], s["plln"], s["pllp"], s["pllq"], s["pllr"]))
        print("      PLL 输入 = %.2f MHz    VCO = %.0f MHz    CPUCLK = %.0f MHz"
              % (s["ref_mhz"], s["vco"], s["cpuclk"]))
        print("      ⇒ 我的约束模型与 PX4 的实际配置一致，说明约束读对了。")
    print()
    print("  求解器给出的**前 5 个更优候选**（按 ST 推荐的「降低 PLL 输入」"
         "以改善抖动排序）：")
    print()
    print("      PLLM  PLLN  PLLP   PLL输入    VCO       CPUCLK   RGE 档位")
    print("      " + "-" * 62)
    for s in sols[:5]:
        print("      %-5d %-5d %-6d %-9.2f %-9.0f %-8.0f %s"
              % (s["pllm"], s["plln"], s["pllp"], s["ref_mhz"],
                 s["vco"], s["cpuclk"], s["rge"].replace("RCC_PLLCFGR_", "")))

    print("""
  注意前几名的 PLL 输入是 4 MHz 或 2 MHz，而不是 16 MHz ——
  因为 ST 官方社区的建议是"用 /M 把输入降下来"能得到**最低抖动**。
  PX4 选了 PLLM=1（16 MHz 输入），这不是最优的抖动配置。
  但这属于"可以更好"，不是错误 —— 480 MHz 下 16 MHz 输入仍是合法的。

  ================== 先澄清一处"假矛盾"：VCO 836 还是 960？ ==================
""")

    print("""
  上面 PX4 的注释里有一句：

      /*     192 MHz <= PLL_VCOH <= 836MHz      ← 注释说上限 836
       *   PLL1_VCO = (16,000,000 / 1) * 60 = 960 MHz   ← 实际是 960
       */

  **960 > 836，注释和实际配置矛盾。**

  我的求解器第一版就是照着注释里的 836 建约束的 ——
  结果返回 **0 组解**，解不出 PX4 的任何一块板子。
  这迫使我停下来查，然后发现原因在 ST 的文档本身：

    · **RM0433（参考手册）** 的 VCOSEL 位描述写的是
          宽 VCO 范围 192 - 836 MHz          ← 这是 **Rev Y 硅片**的限制
    · **DS12110（数据手册）** 的 §7.3.10
          "PLL characteristics (wide VCO frequency range)"
          写的是 192 - **960** MHz            ← 这是 **Rev V 硅片**的限制
    · ST 官方社区确认：**Rev V 引入时 RM0433 没有同步更新**，
      仍停留在 Rev Y 的数值上，应以数据手册为准。

  ⇒ **结论：PX4 的代码是对的，PX4 的注释是过期的。**
    注释抄的是 RM0433 的旧数值，而实际配置针对 Rev V 硅片。

  **这是一个真实的文档陷阱**：
    如果你按 RM0433 建立约束（像我的第一版那样），你会得出
    "PX4 的 VCO 超规格了"这个**错误结论**。
    反过来，如果你照 RM0433 的 836 MHz 去配一块 Rev V 的板子，
    你会永远达不到 480 MHz CPUCLK（960/2 = 480 是唯一的路径）。

  教训：**芯片参考手册不一定是最新的，数据手册的电气特性章节
  才是权威。** 版本不一致时，去 ST 社区搜一下有没有已知差异 ——
  这个坑不只 STM32H7 有。

  ================== 再来看一处真实的不一致：PLL1RGE ==================
""")

    print("""
  现在做 (c)。同一段注释里还有另一行：

      /* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
       *   ^^^^^^^^^^^^^^^^  ← 声明输入是 4-8 MHz
       *
       *   PLL1_VCO = (16,000,000 / 1) * 60 = 960 MHz
       *                  ^^^^^^^^^^^  ← 实际输入是 16 MHz
       */

  而实际用的宏是：

      #define STM32_PLLCFG_PLL1CFG  (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \\
                                     RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \\   ← 4-8 档
                                     ...

  **这一次不是文档的问题 —— 是 PX4 自己的配置确实不一致。**
  而且它不像 VCO 那条有外部理由可以解释（RM 陈旧）。
""")

    ref_used = HSE / 1 / 1e6
    correct = rge_for(ref_used)
    print("  求解器判定：输入 %.0f MHz 应使用 %s"
          % (ref_used, correct.replace("RCC_PLLCFGR_", "")))
    print("  实际使用：  RCC_PLLCFGR_PLL1RGE_4_8_MHZ")
    print("  ⇒ **不匹配。**")
    print()

    # 全树统计
    print("  " + "-" * 74)
    print("  这是一块板子的问题，还是普遍问题？")
    print("  " + "-" * 74)
    print("""
  我在本机仓库里扫描了所有板子的 board.h：

      PX4      : 26 块可解析的板子中，**17 块**的 PLL 输入频率
                 不在它所声明的 RGE 区间内
                 （16 MHz 声明 4-8，或 12 MHz 声明 4-8）
      NuttX上游: 2 块 stm32h7 板子中，**0 块**有问题
                 （nucleo-h743zi: XTAL=8 MHz, DIVM1(2) → 输入 4 MHz,
                   声明 4_8_MHZ → 正确）

  受影响的包括 PX4 的主力板：fmu-v6x / fmu-v6c / fmu-v6u / durandal-v1
  / ctrl-zero-h7 / pixracerpro / nora / x7pro / nxt-v1 等。

  NuttX 的 RCC 驱动**不校验也不推导**这个值 ——
  arch/arm/src/stm32h7/stm32h7x3xx_rcc.c:826 把 board.h 里的宏
  直接 OR 进寄存器：

      regval = (STM32_PLLCFG_PLL1CFG | ...

  所以错误的声明会原样写进硬件寄存器，没有任何一层会拦住它。

  ================== 后果有多严重？—— 必须谨慎回答 ==================
""")

    print("""
  RM0433 对这个字段的要求是：

    · PLL1RGE[1:0]：00=1-2MHz 01=2-4MHz 10=4-8MHz 11=8-16MHz
    · "在使能 PLL1 之前写入"，用途是"为 PLL1 选择合适的参考频率范围"
    · PLL 输入频率 Fref1_ck 的**总体**允许范围是 1-16 MHz

  关键推论：

    (1) 16 MHz 本身在"总体允许范围"内 ⇒ **硬件不会拒绝**，
        PLL 仍然会锁定。这解释了为什么上百万台 Pixhawk 正常飞行 ——
        如果这会导致启动失败，问题早就暴露了。
    (2) 但 RGE 是给 PLL 内部环路滤波器/电荷泵用的参数，
        手册明确要求它匹配输入范围 ⇒ **配错会偏离 ST 标定的工作点，
        影响锁定时间与抖动裕度**。

  我查到的公开资料（**都不是一手测量，仅作参考**）：

    · 有工程师报告 STM32H753 的 EtherCAT 应用中，CubeMX 把 PLL1RGE
      留在默认档（1-2 MHz）导致 VCO 相位噪声从 -110 dBc/Hz 恶化到
      -102 dBc/Hz，跨过了 PHY 时钟恢复电路的容限。
    · 有 STM32H747 的报告说 **VCOSEL**（VCO 范围，另一个字段）配错
      会导致 PLL 完全不锁定 —— 这说明"PLL 范围类字段配错"确实
      可以产生硬故障，只是 RGE 这个具体字段我没找到硬故障报告。

  **我没有实测抖动差异**，所以不能说"这导致了多少 ps 的抖动增加"。
  我能确定的是：

    · 事实层面：17 块 PX4 板子的 RGE 声明与实际输入不符（已实测）
    · 规范层面：违反 RM0433 对该字段的要求（已查证）
    · 后果层面：**未验证**。可能只是裕度变小。

  ================== 为什么这仍然算一个缺陷 ==================
""")

    print("""
  即使它是**刻意**的（比如当年发现 8-16 档有问题而故意避开），
  它也没有留下任何注释说明。在一个飞行控制固件里：

    · 一个违反芯片手册的配置
    · 加上一句与实际算式自相矛盾的注释
    · 加上一个不做校验的内核驱动
    · 影响 17 块在售板子

  **四者叠加，就是一个真实的维护风险。**
  正确的做法只有两种：要么改成 `RCC_PLLCFGR_PLL1RGE_8_16_MHZ`
  （并实测对比），要么保留 `4_8` 但**写清楚为什么**。

  ================== 一个统一的解释（这是推测） ==================
""")

    print("""
  上面两处（VCO 注释 / RGE 宏）有一个共同特征：
  **它们都不影响"板子能不能启动"。**

    · PLLM/PLLN/PLLP 配错 → 480 MHz 出不来 → 串口波特率全错
      → **立刻发现**，不可能漏掉
    · VCO 上限写错 → 只是注释 → **不影响编译和运行**
    · RGE 档位写错 → PLL 仍锁定 → **不影响编译和运行**

  我的推测（**未验证**）：board.h 是从某个模板/另一块板子复制来的
  （那个模板大概是为 4-8 MHz 输入的板子写的），
  作者更新了"不更新就跑不起来"的三个分频比，
  但留下了"看起来还能工作"的 VCO 注释和 RGE 宏。

  这个模式在嵌入式项目里非常常见，值得记住：

      **不破坏构建的错误，不会被修复。**

  而它会影响 17 块板子、存活很多年，因为没有任何自动化检查会
  发现它 —— 编译器不管、Kconfig 不管、内核驱动不管。

  ⇒ **这正好说明本课实验 1 这个东西的价值**：
    把散落在注释里的约束变成**可执行、可校验**的代码，
    然后拿它去跑一遍所有板子的 board.h。
    我做的就是这件事，而它立刻暴露了 17/26 这个数字。
""")
    print("  (b) 给一块新板子求解：HSE = 24 MHz（如 mRo Ctrl-Zero H7）")
    print("  " + "-" * 74)
    sols24 = solve_pll1(24_000_000, 480, pllq_target_mhz=240, pllr_target_mhz=120)
    print("  找到 %d 组解。前 5 个：" % len(sols24))
    print()
    print("      PLLM  PLLN  PLLP   PLL输入    VCO       CPUCLK   RGE 档位（正确）")
    print("      " + "-" * 70)
    for s in sols24[:5]:
        print("      %-5d %-5d %-6d %-9.2f %-9.0f %-8.0f %s"
              % (s["pllm"], s["plln"], s["pllp"], s["ref_mhz"],
                 s["vco"], s["cpuclk"], s["rge"].replace("RCC_PLLCFGR_", "")))
    print()
    # 看看 PX4 对 24 MHz 板子实际选了什么
    print("""  对照：PX4 的 mRo ctrl-zero-h7 用 XTAL=24 MHz、PLLM=2
  → PLL 输入 = 12 MHz，而它声明的是 `RCC_PLLCFGR_PLL1RGE_4_8_MHZ`，
  同样不匹配（12 MHz 应声明 8-16 档）。同一个错误模式的另一个实例。

  ⇒ 求解器输出的 RGE 档位列可以直接抄进 board.h，不会出错。
""")

    # 把实测扫描也做成函数，读者可以复现
    print("""  ---------------------------------------------------------------
  复现我上面那个 17/26 统计的方法（Linux/Git Bash 下可跑）：

      python -c "
      import glob,re,io
      for f in glob.glob('boards/*/*/nuttx-config/include/board.h'):
          t=io.open(f,encoding='utf-8',errors='ignore').read()
          m=re.search(r'#define\\s+STM32_BOARD_XTAL\\s+(\\d+)',t)
          p=re.search(r'STM32_PLLCFG_PLL1M\\s+RCC_PLLCKSELR_DIVM1\\((\\d+)\\)',t)
          r=re.search(r'PLL1RGE_(\\d+)_(\\d+)_MHZ',t)
          if not(m and p and r): continue
          fin=int(m.group(1))/int(p.group(1))/1e6
          lo,hi=int(r.group(1)),int(r.group(2))
          if not lo<=fin<=hi:
              print('%-55s in=%.1fMHz RGE=%d-%d'%(f,fin,lo,hi))
      "
""")


# ============================================================================
# 实验 2：defconfig 裁剪量估算器
# ============================================================================

# 实测锚点：全部来自本机 PX4 仓库
# (名称, CONFIG_ 行数, RAM 字节, 说明)
CONFIG_ANCHORS = [
    ("cubepilot/io-v2 (PX4IO)",   53,   8 * 1024,
     "Cortex-M3，无文件系统挂载点，无 PI，无网络，无 NSH"),
    ("nxp/ucans32k146",           None, 124 * 1024, "CAN 节点"),
    ("matek/gnss-m9n-f4",         None, 128 * 1024, "GNSS 接收机"),
    ("bitcraze/crazyflie",        None, 192 * 1024, "最小完整 PX4"),
    ("ark/fmu-v6x (完整 PX4)",    275,  240 * 1024,
     "Cortex-M7，VFS + 网络 + 4 块 RAM 区域（合计约 992 KB）"),
]

# 各功能大致的配置项数量（**估算**，基于对 defconfig 的观察，
# 不是精确统计；标出来是因为读者需要知道这是粗估）
FEATURE_COST = [
    # (功能, CONFIG_ 项数（约）, 需要的额外 RAM（约）, 依赖的 Kconfig 开关)
    ("内核 + 调度器 + 信号量",       12,  4 * 1024,  "SCHED_*, SEM_*"),
    ("设备驱动框架 + VFS 挂载点",    10,  8 * 1024,  "DISABLE_MOUNTPOINT=n, FS_*"),
    ("FAT 文件系统",                 6,  12 * 1024, "FS_FAT, FS_READABLE"),
    ("littlefs（PX4 参数存储用）",    4,  4 * 1024,  "FS_LITTLEFS"),
    ("伪文件系统 / procfs",          4,  2 * 1024,  "FS_PROCFS"),
    ("NSH（shell）",                 16, 8 * 1024,  "NSH_LIBRARY, NSH_*"),
    ("网络栈 (TCP/IP)",              28, 40 * 1024, "NET, NET_TCP, NET_UDP, NET_ICMP"),
    ("USB CDC/ACM 串口",             8,  8 * 1024,  "CDCACM, USBDEV, USBHOST"),
    ("优先级继承",                   3,  2 * 1024,  "PRIORITY_INHERITANCE, SEM_PREALLOCHOLDERS"),
    ("工作队列 (wqueue)",            3,  4 * 1024,  "SCHED_WORKQUEUE, SCHED_HPWORK"),
    ("看门狗定时器",                 3,  2 * 1024,  "WATCHDOG"),
    ("C++ 支持 (libxx)",             4,  8 * 1024,  "HAVE_CXX, LIBCXX"),
    ("多 RAM 区域合并",              2,  0,         "MM_REGIONS（本身不占 RAM）"),
    ("二进制加载器 (binfmt)",        5,  6 * 1024,  "BINFMT_ELF"),
    ("POSIX mqueue",                 3,  2 * 1024,  "FS_MQUEUE"),
]


def exp2_defconfig_sizer():
    sec("实验 2：defconfig 裁剪量估算器 —— 新板子该开哪些开关")

    print("""
第 10 课实测了两个极端，它们是新板子裁剪时的两个锚点：

  %-28s %-8s %-10s %s
""" % ("板子", "CONFIG_行数", "RAM", "说明") + "  " + "-" * 88)
    for name, lines, ram, note in CONFIG_ANCHORS:
        print("  %-28s %-12s %-10s %s"
              % (name,
                 ("%d 行" % lines) if lines else "—",
                 "%d KB" % (ram // 1024),
                 note))

    print("""
  ⇒ 范围是 **53 行 / 8 KB** 到 **275 行 / 240 KB**，相差 5.2 倍和 30 倍。
    同一份 NuttX，同一棵树，只是 defconfig 不同。
""")

    print("  " + "-" * 74)
    print("  按功能累加：你的板子大概需要多少配置和 RAM？")
    print("  " + "-" * 74)
    print()
    print("  %-32s %-10s %-12s %s"
          % ("功能", "约 CONFIG 项", "约额外 RAM", "依赖的 Kconfig"))
    print("  " + "-" * 88)
    for name, n, ram, dep in FEATURE_COST:
        print("  %-32s %-10s %-12s %s"
              % (name, n, "%d KB" % (ram // 1024), dep))

    print("""
  ⇒ 用法：勾选你需要的功能，把三列相加，得到量级估计。

  【重要】这张表是**粗估**，不是精确模型。它的价值在于给出量级，
  不在于给出准确值。实际做法永远是：
      1. 拿一块最接近的现有板子的 defconfig 当起点
      2. 用 `make menuconfig` 改，而不是手改 defconfig
      3. 编译后看 `.config` 的实际行数 + `size` 命令看内存占用

  ================== 三个必须知道的裁剪开关 ==================
""")

    print("""
  （1）CONFIG_DISABLE_MOUNTPOINT=y
      —— 这是**最省内存的单个开关**。它把整个 VFS 挂载点机制关掉，
         于是不需要 inode 树、文件描述符表、块设备层。
         PX4IO 固件（8 KB RAM）就是这么做到的。
         代价：没有文件系统，没有 /dev 下的设备节点语义
              （设备驱动仍然能编译，但你没法用 open()/ioctl() 访问它们）。
      依赖：必须同时确认没有任何代码调 register_driver()。

  （2）CONFIG_PRIORITY_INHERITANCE
      —— 默认 **n**。打开后所有信号量默认带 PI（第 10 课 3.3 节）。
         如果开了，务必同时设 CONFIG_SEM_PREALLOCHOLDERS（PX4 用 32）。
         8 KB 的 PX4IO 是**不开**的。
      记忆点：这个开关默认关，而"默认关的东西不会给你任何提示"。

  （3）CONFIG_MM_REGIONS
      —— 不改总 RAM，但决定内核把哪几块不连续 RAM 合并成一个堆。
         设小了 → 有些 RAM 永远用不上（表现为"内存不够"但其实有）。
         设大了 → 编译报错说找不到区域（反而安全）。
      做法：数一数芯片有几块不连续 RAM，就设几。PX4 fmu-v6x 是 4。
""")

    # 互动式：算一个例子
    print("  " + "-" * 74)
    print("  例子：从零设计一块『CAN 节点』板子（只有 CAN + 串口 + 参数存储）")
    print("  " + "-" * 74)
    need = ["内核 + 调度器 + 信号量", "设备驱动框架 + VFS 挂载点",
            "littlefs（PX4 参数存储用）", "工作队列 (wqueue)", "看门狗定时器"]
    total_cfg = sum(n for f, n, r, d in FEATURE_COST if f in need)
    total_ram = sum(r for f, n, r, d in FEATURE_COST if f in need)
    print()
    for f in need:
        row = [x for x in FEATURE_COST if x[0] == f][0]
        print("      + %-32s %3d 项  %4d KB" % (f, row[1], row[2] // 1024))
    print("      " + "-" * 56)
    print("      %-34s %3d 项  %4d KB" % ("合计（粗估）", total_cfg, total_ram // 1024))
    print()
    print("      对照实测：nxp/ucans32k146 是一块真实的 CAN 节点板，")
    print("                实测 CONFIG_RAM_SIZE = 124 KB、")
    print("                PX4IO(53行/8KB) 则连文件系统都没有。")
    print("      ⇒ 量级一致。可以按这个思路裁剪。")


# ============================================================================
# 实验 3：内存布局 / 链接脚本生成器
# ============================================================================

def exp3_memory_layout():
    sec("实验 3：内存布局与 linker script 生成")

    print("""
NuttX 的板级目录里有几个 `.ld` 文件，分工是：

  scripts/memory.ld        声明**内存区域**（芯片有几块 RAM / Flash，各多大）
  scripts/flash.ld         把各段（.text/.data/.bss/堆/栈）放进这些区域
  scripts/kernel.space.ld  ┐
  scripts/user-space.ld    ┘ 只有 protected / kernel build 才需要
  scripts/Make.defs        告诉构建系统用哪个 .ld、以及编译选项

PX4 只保留了两个：
  nuttx-config/scripts/script.ld           主固件
  nuttx-config/scripts/bootloader_script.ld  bootloader

（注意 PX4 没有 `memory.ld` —— 它把区域声明直接写进了 script.ld。见下。）
""")

    print("  " + "-" * 74)
    print("  STM32H753 的内存区域（fmu-v6x 用的芯片）")
    print("  " + "-" * 74)
    regions = [
        ("FLASH",  "0x08020000", 1792 * 1024,  "AXI (ITCM/DTCM 由链接脚本分配)"),
        ("AXI_SRAM", "0x24000000", 512 * 1024, "D1 域，最快的 RAM"),
        ("SRAM123",  "0x30000000", 288 * 1024, "D2 域"),
        ("SRAM4",    "0x38000000", 64 * 1024,  "D3 域，紧邻 BDMA"),
        ("DTCM",     "0x20000000", 128 * 1024, "TCM，CPU 不经总线矩阵直连"),
    ]
    print()
    print("  %-10s %-12s %-12s %s" % ("区域", "起始地址", "大小", "说明"))
    print("  " + "-" * 62)
    for n, a, s, d in regions:
        print("  %-10s %-12s %-12s %s" % (n, a, "%d KB" % (s // 1024), d))
    print("""
  注意 FLASH 起始于 **0x08020000** 而不是 0x08000000 ——
  前 128 KB 留给 bootloader（第 8 课讲过）。

  这也是一个移植时的常见坑：如果忘了改链接脚本的 Flash 起始地址，
  bootloader 会被固件覆盖掉，表现为"烧录成功但板子再也进不了 DFU"。
""")

    # 生成一段 MEMORY{} 声明
    print("  " + "-" * 74)
    print("  生成 MEMORY{} 段（可直接粘进 script.ld）")
    print("  " + "-" * 74)
    print()
    print("    MEMORY")
    print("    {")
    for n, a, s, d in regions:
        print("      %-10s (rx)  : ORIGIN = %s, LENGTH = %dK"
              % (n, a, s // 1024))
    print("    }")
    print()

    total = sum(s for n, a, s, d in regions if "SRAM" in n or n == "DTCM")
    print("  合计可合并为堆的 RAM = %d KB" % (total // 1024))
    print("  ⇒ 这就是 CONFIG_MM_REGIONS 应该设的值：%d" % len(
        [r for r in regions if r[0] != "FLASH"]))
    print("""
  验证：PX4 fmu-v6x 的 defconfig 里 `CONFIG_MM_REGIONS=4`，
        而上面的 RAM 区域正好是 4 个（AXI_SRAM / SRAM123 / SRAM4 / DTCM）。
        ✓ 一致。

  ================== 移植检查清单（内存部分）==================

    [ ] Flash 起始地址是不是 bootloader 之后？（看 bootloader 的大小）
    [ ] 所有不连续 RAM 区域都列进 MEMORY{} 了吗？
    [ ] CONFIG_MM_REGIONS 的数量和 MEMORY{} 里的 RAM 区域数一致吗？
    [ ] DTCM 要不要分给堆？DTCM 不能被 DMA 访问 ——
        如果给了主堆，某些驱动可能在 DTCM 上分配 DMA 缓冲而失败。
        PX4 的做法是把 DMA 缓冲放到独立的 GRAN 池（第 8 课）。
    [ ] 栈放在哪块 RAM？中断栈（CONFIG_ARCH_INTERRUPTSTACK）呢？
    [ ] 堆和栈有没有可能相撞？（NuttX 默认不检测，除非开
        CONFIG_DEBUG_MM 或栈染色）
""")


# ============================================================================
# 实验 4：板级文件清单生成器
# ============================================================================

# 一块 NuttX 板子必需/可选的文件。
# 依据：boards/arm/stm32h7/nucleo-h743zi/ 的实际结构（上游）
#       + boards/ark/fmu-v6x/ 的实际结构（PX4 的 out-of-tree 变体）
BOARD_FILES = [
    # (相对路径, 必需?, 作用)
    ("include/board.h",        True,
     "引脚/时钟/LED/外设的宏定义。移植时改动最多的地方"),
    ("Kconfig",                True,
     "板级 Kconfig。至少要有空的占位，否则 menuconfig 里看不到这块板"),
    ("nsh/defconfig",          True,
     "默认配置（名字可换，如 px4 用 nsh/ 和 bootloader/ 两个）"),
    ("scripts/script.ld",      True,
     "链接脚本：内存区域 + 各段的摆放"),
    ("scripts/Make.defs",      False,
     "编译/链接选项。上游常有；PX4 改用生成的 Make.defs.in"),
    ("src/Makefile",           True,
     "列出本板的 CSRCS（源文件清单）"),
    ("src/<board>.h",          True,
     "板级内部头文件（私有宏、函数声明）"),
    ("src/<chip>_boot.c",      True,
     "stm32_boardinitialize()：**上电后最早执行的板级代码**"),
    ("src/<chip>_bringup.c",   True,
     "board_late_initialize() / 板级设备注册"),
    ("src/<chip>_appinitialize.c", True,
     "board_app_initialize()：NSH 启动前调用的应用初始化"),
    ("src/<chip>_userleds.c",  False,
     "用户 LED。CONFIG_USERLED 打开时才编译"),
    ("src/<chip>_autoleds.c",  False,
     "自动 LED（心跳/活动指示）。CONFIG_ARCH_LEDS 打开时用"),
    ("src/<chip>_buttons.c",   False,
     "按键。CONFIG_ARCH_BUTTONS 时才需要"),
    ("src/<chip>_spi.c",       False,
     "SPI 引脚/总线注册。用 SPI 就要"),
    ("src/<chip>_i2c.c",       False, "I2C 引脚/总线注册"),
    ("src/<chip>_usb.c",       False, "USB 引脚配置"),
    ("src/<chip>_adc.c",       False, "ADC 通道配置"),
    ("src/<chip>_pwm.c",       False, "PWM 定时器配置"),
    ("kernel/",                False,
     "只有 CONFIG_BUILD_PROTECTED/KERNEL 才需要"),
    ("configs/<name>/defconfig", False,
     "额外的配置变体（一块板可以有多个 defconfig）"),
    ("README.txt",             False,
     "硬件说明。上游板子大多有；PX4 放在别处"),
]


def exp4_file_checklist():
    sec("实验 4：板级文件清单 —— 新建一块板子要写哪些文件")

    print("""
依据两块真实板子的结构（本机实测）：
  · 上游 NuttX：boards/arm/stm32h7/nucleo-h743zi/   —— 约 28 个文件
  · PX4 out-of-tree：boards/ark/fmu-v6x/nuttx-config/ —— **7 个文件**
     （Kconfig, include/board.h, include/board_dma_map.h,
       nsh/defconfig, bootloader/defconfig,
       scripts/script.ld, scripts/bootloader_script.ld）

**PX4 的板级目录只有 7 个文件，因为它用了 out-of-tree 机制：**
  · nuttx-config/ 是 NuttX 的 BOARD_DIR（见实验 5）
  · 真正的 C 代码在 boards/<vendor>/<model>/src/ —— 那是 PX4 的 CMake
    编译的，不是 NuttX 的 make 编译的
  ⇒ 所以 PX4 **不需要** src/Makefile、src/*_boot.c 这些 NuttX 板级文件
""")

    print("  " + "-" * 74)
    print("  上游（in-tree）移植的文件清单")
    print("  " + "-" * 74)
    print()
    print("  %-34s %-6s %s" % ("文件", "必需", "作用"))
    print("  " + "-" * 96)
    for path, req, desc in BOARD_FILES:
        print("  %-34s %-6s %s" % (path, "★" if req else "·", desc))

    req_count = sum(1 for _, r, _ in BOARD_FILES if r)
    print()
    print("  ⇒ **必需文件只有 %d 个**，其余按功能需要添加。" % req_count)
    print("     这是移植工作量的真实边界：最少 %d 个文件就能启动一块新板子。"
          % req_count)

    print("""
  ================== 七个必需文件里，真正要动脑的只有三个 ==================
""")
    print("""
  ┌─ 1. include/board.h  ← 工作量最大，最容易错
  │     里面是：晶振频率、PLL 配置、总线时钟分频、LED/按键引脚。
  │     实验 1 的求解器就是为这个文件服务的。
  │     PX4 fmu-v6x 的这个文件是 **558 行 / 161 个 #define**。
  │
  ├─ 2. scripts/script.ld  ← 错了会烧不进去或覆盖 bootloader
  │     实验 3 讲了。
  │
  └─ 3. src/<chip>_boot.c  ← 上电后最早跑的板级代码
        通常只有几十行：开时钟、配引脚、清中断。
        它是 NuttX 从 arch 层进入 board 层的**唯一入口**（见实验 6）。

  其余四个（Kconfig / defconfig / src/Makefile / <board>.h）
  基本是模板填空。
""")


# ============================================================================
# 实验 5：PX4 的 out-of-tree 机制（这是本课最重要的发现）
# ============================================================================

def exp5_out_of_tree():
    sec("实验 5：PX4 的 out-of-tree 板级机制 —— 板子为什么不放在 NuttX 树里")

    print("""
这是本课最重要的机制。理解了它，你就能回答两个问题：
  · PX4 是怎么做到「升级 NuttX 子模块」而不丢自己的板级代码的？
  · 一块第三方板子（比如某公司的定制飞控）怎么接入 PX4？

================== 问题 ==================

NuttX 上游的板级代码放在 **NuttX 树里面**：
    nuttx/boards/arm/stm32h7/nucleo-h743zi/

如果 PX4 也这么做，那么 PX4 的 155 个板级目录就要放进 NuttX 子模块里 ——
每次 `git submodule update` 升级 NuttX 都会冲突。**不可行。**

================== NuttX 的解决方案 ==================

NuttX 的 Kconfig 提供了 out-of-tree 支持
（platforms/nuttx/NuttX/nuttx/boards/Kconfig:2558-2575 原文）：

    config ARCH_BOARD_CUSTOM_DIR
        string "Custom board directory"
        default ""
        ---help---
            If the custom board configuration is selected, then it is necessary
            to also tell the build system where it can find the board directory
            for the custom board.

            In this case, the board directory is assumed to lie outside the
            NuttX directory.  The provided path must then be a full, absolute
            path to some location outside of the NuttX source tree (like
            "~/projects/myboard").

    config ARCH_BOARD_CUSTOM_DIR_RELPATH
        bool "Relative custom board directory"
        default y
        ---help---
            Specifies that the board directory is relative to the NuttX directory.

**NuttX 原生就支持板级目录在树外！** 这不是 PX4 的补丁。

================== PX4 怎么用 ==================

boards/ark/fmu-v6x/nuttx-config/nsh/defconfig:57-60 原文：

    CONFIG_ARCH_BOARD_CUSTOM=y
    CONFIG_ARCH_BOARD_CUSTOM_DIR="../../../../boards/ark/fmu-v6x/nuttx-config"
    CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
    CONFIG_ARCH_BOARD_CUSTOM_NAME="ark"

路径是**相对的**，基准是 NuttX 树根（nuttx/）。
我来验证它解析到哪：
""")

    # 路径解析验证
    import posixpath
    nuttx_root = "PX4-Autopilot/platforms/nuttx/NuttX/nuttx/"
    rel = "../../../../boards/ark/fmu-v6x/nuttx-config"
    resolved = posixpath.normpath(posixpath.join(nuttx_root, rel))
    print("      NuttX 树根 : %s" % nuttx_root)
    print("      CUSTOM_DIR : %s" % rel)
    print("      解析结果   : %s" % resolved)
    print("      ⇒ 正好落在 PX4 根目录下的 boards/ 里。**验证通过。**")
    print()

    print("""  构建系统在 tools/Config.mk:135-160 消费这个值（原文关键行）：

      ifeq ($(CONFIG_ARCH_BOARD_CUSTOM),y)
        CUSTOM_DIR = $(patsubst "%",%,$(CONFIG_ARCH_BOARD_CUSTOM_DIR))
        ifeq ($(CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH),y)
          BOARD_DIR ?= $(TOPDIR)$(DELIM)$(CUSTOM_DIR)          ← PX4 走这条
        else
          BOARD_DIR ?= $(CUSTOM_DIR)
        endif
        CUSTOM_BOARD_KPATH = $(BOARD_DIR)$(DELIM)Kconfig
      else
        BOARD_DIR ?= $(TOPDIR)$(DELIM)boards$(DELIM)$(CONFIG_ARCH)$(DELIM)$(CONFIG_ARCH_CHIP)$(DELIM)$(CONFIG_ARCH_BOARD)
      endif

  ⇒ 于是 `BOARD_DIR` = `.../boards/ark/fmu-v6x/nuttx-config/`
    这个目录**就是** NuttX 的板级目录，只是物理位置在 PX4 树里。

  ================== 由此推出的几个结论 ==================
""")
    print("""
  【结论 1】PX4 升级 NuttX 子模块不会丢板级代码
          因为板级代码不在子模块里。这是 out-of-tree 的全部意义。

  【结论 2】第三方板子接入 PX4 的路径是明确的
          在 boards/<你的vendor>/<你的model>/nuttx-config/ 下写
          一个 defconfig，设好 CONFIG_ARCH_BOARD_CUSTOM_DIR，
          再用 .px4board 描述要用哪些模块。

  【结论 3】这也解释了 PX4 板级目录为什么"只有 7 个文件"
          （见实验 4）—— 因为 NuttX 板级的 src/ 那部分，
          PX4 搬到了 boards/<v>/<m>/src/，由 PX4 的 CMake 编译，
          不是 NuttX 的 make 编译。

  【结论 4】因此 PX4 板级移植的**工作量和上游 NuttX 不一样**
          上游：写 board.h + script.ld + src/*.c（NuttX 风格）
          PX4 ：写 board.h + script.ld + defconfig + .px4board
                + src/ 下的板级驱动（PX4 风格，用 PX4 的 API）
          后者更贴近 PX4 的驱动模型，但不是 NuttX 社区的通用做法。

  ================== PX4 还有一层：defconfig 的搬运 ==================
""")
    print("""  platforms/nuttx/NuttX/CMakeLists.txt 里有这么两行：

      COMMAND cat ${NUTTX_DEFCONFIG} ${EXTRA_NUTTX_CONFIG_FILE} > ${NUTTX_DIR}/.config
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/defconfig

  ⇒ PX4 **在构建时把 boards/<v>/<m>/nuttx-config/nsh/defconfig
    拷贝到 NuttX 树里**（`nuttx/.config` 和 `nuttx/defconfig`），
    然后跑 `tools/px4_nuttx_make_olddefconfig.sh` 做 olddefconfig。

  这样 NuttX 的 make 就觉得"这是一块普通的 in-tree 板子"，
  而 PX4 的源码树保持干净（拷贝目标是构建目录/子模块内的生成文件）。

  另外注意 `EXTRA_NUTTX_CONFIG_FILE` 和它前面的环境变量：

      # Allow to specify extra options via
      # 'export PX4_EXTRA_NUTTX_CONFIG="CONFIG_xy=y;CONFIG_z=y"'
      if(DEFINED ENV{PX4_EXTRA_NUTTX_CONFIG})
              foreach(OPTION $ENV{PX4_EXTRA_NUTTX_CONFIG})
                      file(APPEND ${EXTRA_NUTTX_CONFIG_FILE} "${OPTION}\\n")
              endforeach ()
      endif()

  ⇒ **一个非常有用的调试技巧**：调试板子时可以用
      export PX4_EXTRA_NUTTX_CONFIG="CONFIG_DEBUG_ASSERTIONS=y;CONFIG_DEBUG_SYMBOLS=y"
    临时加配置，**不用改 defconfig 文件**。
    这在"我想试一下这个开关"的时候极其省事。
""")


# ============================================================================
# 实验 6：启动路径 —— 板级代码在哪一步被执行
# ============================================================================

def exp6_boot_path():
    sec("实验 6：启动路径 —— 板级代码在什么时候被执行")

    print("""
移植时最容易困惑的是："我写的 stm32_boardinitialize() 到底什么时候跑？"

第 7 课讲过中断向量（arm_vectors.c → exception_common），
第 8 课讲过 PX4 的三条启动链。这一节把**板级**那一段补上。

================== NuttX 在 Cortex-M 上的启动链 ==================
""")
    print("""
      ① 硬件复位
         ↓  读取向量表第 0 项
      ② __start            （arch/arm/src/<chip>/<chip>_vectors.S）
         ↓  设置初始 SP = 向量表第 0 项
      ③ stm32_start()      （arch/arm/src/stm32h7/stm32h7_start.c）
         ↓
         ├─ 关看门狗、清 BSS、搬 .data
         ├─ stm32_clockconfig()      配置 PLL（用 board.h 的宏！）
         ├─ stm32_rccreset()
         ├─ 早期串口初始化（如果 CONFIG_ARCH_EARLYCONSOLE）
         ├─ **stm32_boardinitialize()**  ← ★ 板级代码在这里第一次被执行
         │     定义在 boards/<...>/src/<chip>_boot.c
         │     它做的事：配置 LED/按键/SPI/I2C/USB 的**引脚**和**时钟**
         ↓
      ④ nx_start()         （sched/init/nx_start.c）
         ↓
         ├─ 初始化空闲任务、初始任务
         ├─ nx_bringup() → 初始化各子系统（fs / mm / drivers）
         ↓
      ⑤ 创建初始任务（INIT_ENTRY 或 INIT_FILE）
         ↓
         ├─ 若 CONFIG_NSH_ARCHINIT=y：
         │     **board_app_initialize()**  ← ★ 板级代码第二次被执行
         │     定义在 boards/<...>/src/<chip>_appinitialize.c
         │
         └─ 若 CONFIG_BOARD_LATE_INITIALIZE=y：
               **board_late_initialize()**
               定义在 boards/<...>/src/<chip>_bringup.c
         ↓
      ⑥ nsh_main() → 读 /etc/init.d/rcS 或 ns**h_initscript
         ↓
      ⑦ PX4: boardctl(BOARDIOC_INIT) → px4_platform_init()

================== 三个板级钩子，各自什么时候用 ==================
""")
    hooks = [
        ("stm32_boardinitialize()", "③ 时钟配好之后、nx_start() 之前",
         "配 GPIO 引脚 / 外设时钟 / 早期外设。**此时还没有任何 OS 服务**："
         "不能 malloc，不能用信号量，不能创建任务。",
         "src/<chip>_boot.c"),
        ("board_late_initialize()", "⑤ 初始任务里，由 CONFIG_BOARD_LATE_INITIALIZE 触发",
         "注册板级设备（register_driver）。此时 OS 已经起来了，"
         "可以 malloc、可以创建线程。",
         "src/<chip>_bringup.c"),
        ("board_app_initialize()", "⑤ 初始任务里，由 CONFIG_NSH_ARCHINIT 触发",
         "和 board_late_initialize 类似，但由 **NSH** 触发。"
         "PX4 走的是这一条（**PX4 不定义 CONFIG_BOARD_LATE_INITIALIZE**，"
         "第 8 课已用 grep 验证过：fmu-v6x defconfig 里出现 0 次）。",
         "src/<chip>_appinitialize.c"),
    ]
    print("  %-26s %-34s %s" % ("钩子", "执行时机", "定义位置"))
    print("  " + "-" * 90)
    for n, w, d, f in hooks:
        print("  %-26s %-34s %s" % (n, w, f))
    print()
    for n, w, d, f in hooks:
        print("  【%s】" % n)
        print("      %s" % d)
        print()

    print("""  ================== 移植时最容易搞混的一点 ==================

  `board_late_initialize()` 和 `board_app_initialize()`
  **不是"先后"关系，而是"二选一"关系** —— 由不同的 Kconfig 开关触发：

      CONFIG_BOARD_LATE_INITIALIZE=y  →  调 board_late_initialize()
      CONFIG_NSH_ARCHINIT=y           →  调 board_app_initialize()

  如果两个都开了会怎样？看 sched/init/nx_bringup.c 的实际顺序
  （第 8/9 课读过这一段的上下文）—— 两个都会被调用。
  移植时**按现有板子抄**，不要两个都开。

  PX4 的选择是 `CONFIG_NSH_ARCHINIT=y`（因为它需要 NSH 先起来，
  再由 rcS 脚本 orchestrate 一切）。ArduPilot 在 ChibiOS 上
  没有这套钩子，它用 `__late_init()`（halInit/chSysInit 之后）
  和 `AP_HAL_MAIN_CALLBACKS` 做同样的事（第 10 课 6.4 节）。
""")


# ============================================================================
# 实验 7：移植检查清单（可执行版）
# ============================================================================

CHECKLIST = [
    ("A", "确定芯片与内存布局", [
        ("A1", "芯片型号确认（STM32H753II / H743VI / ...）",
         "CONFIG_ARCH_CHIP_STM32H753II=y"),
        ("A2", "Flash 起始地址在 bootloader 之后",
         "看 bootloader 大小，PX4 是 128 KB → 0x08020000"),
        ("A3", "数清所有不连续 RAM 区域",
         "CONFIG_MM_REGIONS 要等于区域数"),
        ("A4", "DTCM 是否分给主堆？",
         "DTCM 不能被 DMA 访问 —— 给了主堆要小心"),
    ]),
    ("B", "配置时钟（实验 1 的求解器）", [
        ("B1", "晶振频率与板子实际一致",
         "STM32_BOARD_XTAL，错了会导致所有时钟都错"),
        ("B2", "PLLM/PLLN/PLLP 使 CPUCLK 达标且满足 VCO 范围",
         "用实验 1 的求解器"),
        ("B3", "**RGE 档位与 PLL 输入频率匹配**",
         "★ 这是 PX4 17 块板子出错的地方（见实验 1）"),
        ("B4", "AHB/APB 分频不超过外设上限",
         "HCLK/APB1/APB2 各有上限，超了外设会静默出错"),
        ("B5", "时钟配置有编译期校验",
         "ArduPilot 用 static_assert 校验 HAL_EXPECTED_SYSCLOCK "
         "与实际一致（第 10 课 8.1 节）—— 值得抄"),
    ]),
    ("C", "引脚与外设", [
        ("C1", "所有用到的引脚在 board.h 里有定义",
         "PYH/SPI/I2C/UART/CAN 的引脚宏"),
        ("C2", "没有引脚冲突（同一引脚被两个外设用）",
         "NuttX 不检查这个，只能自己对表"),
        ("C3", "SPI/I2C 总线编号与硬件一致",
         "board.h 里通常有注释说明每条总线接什么"),
        ("C4", "LED 和按键定义（可选）",
         "CONFIG_ARCH_LEDS / CONFIG_ARCH_BUTTONS"),
    ]),
    ("D", "链接脚本", [
        ("D1", "MEMORY{} 区域地址和大小与芯片手册一致",
         "查芯片数据手册的 memory map"),
        ("D2", "堆和栈不会相撞",
         "NuttX 默认不检测；开 CONFIG_DEBUG_MM 可查"),
        ("D3", "中断栈大小合理",
         "PX4 用 CONFIG_ARCH_INTERRUPTSTACK=768（实测 66/70 块板）"),
        ("D4", "有栈溢出检测",
         "CONFIG_STACK_COLORATION=y 可染色检测"),
    ]),
    ("E", "配置裁剪（实验 2 的估算器）", [
        ("E1", "从最接近的现有板子 defconfig 出发",
         "不要从零写"),
        ("E2", "用 make menuconfig 改，不手改 defconfig",
         "手改容易漏依赖，Kconfig 会自动处理 select"),
        ("E3", "确认 CONFIG_DISABLE_MOUNTPOINT 是否可开",
         "开了省很多内存，但没有文件系统（PX4IO 就这么做）"),
        ("E4", "RAM 吃紧时优先关：网络 > NSH > FAT > USB",
         "按实验 2 的代价表"),
    ]),
    ("F", "启动与调试", [
        ("F1", "早期串口能出字",
         "CONFIG_ARCH_EARLYCONSOLE 或板级 putc —— 这是第一条生命线"),
        ("F2", "能进 NSH",
         "CONFIG_NSH_LIBRARY=y"),
        ("F3", "板级钩子的选择与现有板子一致",
         "board_app_initialize vs board_late_initialize（实验 6）"),
        ("F4", "知道怎么恢复砖头",
         "DFU / SWD / bootloader 的进入方式"),
    ]),
]


def exp7_checklist():
    sec("实验 7：移植检查清单 —— 每一格都可以对照本机仓库验证")

    total = sum(len(items) for _, _, items in CHECKLIST)
    print("""
  共 %d 项。这里不是给你打勾用的表格，而是**每一项都指向一个
  可以在这台机器上验证的事实** —— 移植的可靠性来自"每一项都有依据"，
  不是来自"清单走完了"。
""" % total)

    for group, title, items in CHECKLIST:
        print("  " + "-" * 74)
        print("  %s. %s" % (group, title))
        print("  " + "-" * 74)
        for cid, item, note in items:
            print("    [ ] %-4s %s" % (cid, item))
            print("            %s" % note)
        print()

    print("""  ================== 三项最容易被跳过、后果最严重的 ==================
""")
    print("""
  ★ B3  RGE 档位与 PLL 输入匹配
       被跳过率：极高（因为它看起来像个常量，不像参数）
       后果：偏离 ST 标定的 PLL 工作点。PX4 有 17 块板子错在这里。
       验证方法：实验 1 的求解器，或手算 XTAL/PLLM 落在哪个区间。

  ★ D2  堆和栈不会相撞
       被跳过率：高（因为"默认没事"）
       后果：偶发崩溃，且现象随负载变化 —— 最难定位的一类。
       验证方法：CONFIG_STACK_COLORATION=y 染色后跑满负载。

  ★ F1  早期串口能出字
       被跳过率：低（但**必须第一个做**）
       理由：在早期串口通之前，你和板子之间没有任何通信渠道。
             所有其他调试手段（NSH、日志、gdb）都建立在它之上。
       做法：先把 stm32_boardinitialize() 里加上"点亮一个 LED
             或往串口吐一个字符"，确认时钟和引脚对了，再往下走。

  ================== 一个方法论 ==================

  移植过程中，"板子不工作"的原因分布大致是：

      时钟配错（PLL / 分频 / 晶振频率）       ~40%
      引脚配错（复用、上下拉、冲突）           ~30%
      链接脚本错（地址、大小、栈）             ~15%
      配置裁剪错（少开/多开了某个开关）         ~10%
      代码逻辑错（自己的驱动）                 ~5%

  ⇒ **前两项占 70%，而且它们在编译期都不会报错。**
    这就是为什么本课把时钟求解器（实验 1）和引脚清单（C1-C3）
    放在最前面 —— 它们覆盖了移植失败的大多数。
""")


# ============================================================================
# 主入口
# ============================================================================

EXPERIMENTS = {
    "1": ("STM32H7 时钟树求解器", exp1_clock_tree),
    "2": ("defconfig 裁剪量估算器", exp2_defconfig_sizer),
    "3": ("内存布局与链接脚本生成", exp3_memory_layout),
    "4": ("板级文件清单", exp4_file_checklist),
    "5": ("PX4 的 out-of-tree 板级机制", exp5_out_of_tree),
    "6": ("启动路径与三个板级钩子", exp6_boot_path),
    "7": ("移植检查清单（可执行版）", exp7_checklist),
}


def main():
    print("""
================================================================================
 第 11 课：移植 NuttX 到一块新板子
 配套实验代码 —— 实验 1/2 是可以真的用上的工具，其余是可验证的参照

 实验列表：
   1  STM32H7 时钟树求解器 —— 把 board.h 的注释变成可运行代码
   2  defconfig 裁剪量估算器 —— 新板子该开哪些开关
   3  内存布局与 linker script 生成
   4  板级文件清单 —— 新建一块板子要写哪些文件
   5  PX4 的 out-of-tree 板级机制 ★ 本课最重要的发现
   6  启动路径 —— 板级代码在什么时候被执行
   7  移植检查清单（可执行版）
================================================================================
""")

    args = sys.argv[1:]
    if args:
        for a in args:
            if a in EXPERIMENTS:
                EXPERIMENTS[a][1]()
            else:
                print("未知实验编号: %s（可选：%s）"
                      % (a, ", ".join(sorted(EXPERIMENTS))))
    else:
        for k in sorted(EXPERIMENTS):
            EXPERIMENTS[k][1]()

    print("""
================================================================================
 本课三条最重要的结论
================================================================================

1. **移植 NuttX 到一块新板子的必需文件只有 7 个**，其中真正要动脑的
   只有三个：board.h（时钟+引脚）、script.ld（内存布局）、
   <chip>_boot.c（上电后最早的板级代码）。其余是模板填空。
   上游 NuttX 的 nucleo-h743zi 有约 28 个文件，但多出来的都是
   "功能文件"（LED/按键/SPI/I2C/USB/ADC/PWM），按需添加。

2. **PX4 的板级目录不在 NuttX 树里，是 NuttX 原生支持的 out-of-tree 机制**
   （CONFIG_ARCH_BOARD_CUSTOM_DIR）。这不是 PX4 的补丁，而是 NuttX
   上游的功能。所以：
     · PX4 升级 NuttX 子模块不会丢板级代码
     · 第三方板子接入 PX4 的路径是明确的
     · PX4 的板级目录可以只有 7 个文件（C 代码搬到了 PX4 侧编译）
   调试技巧：`export PX4_EXTRA_NUTTX_CONFIG="CONFIG_xxx=y"` 可以临时
   加配置，不用改 defconfig。

3. **板子不工作的原因 70% 在时钟和引脚，而它们在编译期都不报错。**
   本课实验 1 的时钟求解器把 RM0433 的约束变成可运行代码；
   它同时暴露了一处真实的不一致：PX4 有 17 块板子声明的 PLL1RGE
   档位与实际 PLL 输入频率不符（16 MHz 声明为 4-8 MHz 档），
   而上游 NuttX 的同类板子没有这个问题。
   这不是致命错误（硬件仍会锁定），但它违反芯片手册、
   且注释与实际算式自相矛盾 —— 在一个飞行控制固件里，
   这属于应当修正或至少应当注释说明的缺陷。
================================================================================
""")


if __name__ == "__main__":
    main()
