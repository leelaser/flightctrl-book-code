# -*- coding: utf-8 -*-
"""源码树位置：**优先环境变量，其次本机默认** —— 读者不必改脚本。

配套脚本要读上游源码（FreeRTOS / PX4 / ArduPilot / NuttX / 自研板 BSP）。
原来这些根目录是**写死**在 26 处的，读者只能改源码。现在：

    export SRC_FREERTOS=/path/to/FreeRTOS
    export SRC_PX4=/path/to/PX4-Autopilot
    export SRC_APM=/path/to/ardupilot          # Copter 4.5.7 / Plane 4.7.1 都指它
    export SRC_NUTTX=/path/to/nuttx
    export SRC_BSP=/path/to/fmu-v6x-qgbf       # 本书自研板的 BSP

不设也能跑：回落作者本机的默认路径。**那棵树你这里没有时，脚本自己会打印
「未找到 …」并跳过相关实验**（不是崩掉）—— 这是原设计就有的行为，保留。

各脚本的文档字符串里出现的 `$SRC_XXX/子路径`，说的就是这里的键：

    上游源码     SRC_FREERTOS  SRC_PX4  SRC_PX4_115  SRC_APM  SRC_APM_471
                 SRC_PLANE  SRC_NUTTX（PX4 树内那份）  SRC_NUTTX_SRC（独立快照）
                 SRC_CLIB（MAVLink C 库）  SRC_BF（Betaflight）  SRC_AEQ  SRC_DCM
                 SRC_QGC  SRC_TEST
    自研硬件     SRC_BSP  SRC_HW（V1.4）  SRC_HW11（V1.1）  SRC_HW_ROOT（总目录）
    素材与数据   SRC_MAT（飞控学习教程）  SRC_MAT_RTOS  SRC_DOC_NUTTX
                 SRC_ULOG（真机日志）

⚠️ 默认值里那条 `硬件/版本V1.4 …` 是**作者本机**的目录名，只为让作者这端仍能跑通；
   读者请用 `SRC_BSP` 指到自己的 BSP 目录。

★ 2026-10 补：**未知名字不再静默返回空串**。
  原来是 `DEFAULTS.get(name, '')` —— 取一个没登记的名字会得到 `''`，于是
  `os.path.join('', 'boards')` 变成**相对路径**、`os.path.isdir('')` 变成假，
  整段逻辑**静默跳过**：打印什么都不少，看着像"这一段没问题"。
  实测就有一处踩中：`_coverage_audit.py` 取 `DOC_NUTTX`，而 DEFAULTS 里
  **从来没有这个键** ⇒ 「NuttX 文档」那一节一次也没跑过，报告照绿。
  ⇒ 现在取不到就**当场抛错**，并列出所有可用名字。
"""
import os

DEFAULTS = {
    # ---- 上游源码树 ----
    'FREERTOS': u'D:/DEVLib/PX4APM/FreeRTOS',
    'PX4': u'D:/DEVLib/PX4APM/PX4/PX4-Autopilot',
    'PX4_115': u'D:/DEVLib/PX4APM/PX4/PX4-Autopilot-1.15.4/PX4-Autopilot-1.15.4',
    'APM': u'D:/DEVLib/PX4APM/APM/ardupilot-Copter-4.5.7/ardupilot-Copter-4.5.7',
    'APM_471': u'D:/DEVLib/PX4APM/APM/ardupilot-Copter-4.7.1',
    # ★ 2026-10 加：书稿迁移的目标版本。
    #   这是一份**完整 ArduPilot 仓库**（ArduCopter/ArduPlane/ArduSub/Rover/
    #   libraries/Tools 全在），所以 **Plane 也用它**，不要再单列一棵
    #   `PLANE_472` —— `ardupilot-Plane-4.7.2/` 与它逐字节相同，
    #   并排登记只会把同名文件的歧义放大一倍
    #   （`_check_citations.py` 里反复强调过这个坑）。
    #
    #   ★ 子模块 pin（2026-10-19 从 GitHub API 查得，下载补齐）：
    #       ArduPilot Copter-4.7.2 → modules/ChibiOS @
    #       d103983f4362e5506bcf7062ccacf345fc0c0859
    #     （4.5.7 是 6a85082c715457d1e0cc9627f9939f349de1143e —— **不同提交**，
    #      所以卷5 的 ChibiOS 引用不能拿 4.5.7 的源码顶上。）
    'APM_472': u'D:/DEVLib/PX4APM/APM/ardupilot-Copter-4.7.2',
    # ★ 固定翼章锚定这一棵（2026-10-19 用户指定）。
    #   实测：它与 `APM_472` 的**共同文件逐字节完全相同**
    #   （全量 MD5 比对 7770 个文件，0 处不同）；唯一差别是 `APM_472`
    #   多了一份已补齐的 `modules/ChibiOS/` 子模块（5514 个文件）。
    #   所以固定翼引用换到这棵树**不需要改任何行号**，
    #   区别只在出处与版本声明的诚实性。
    'APM_472_PLANE': u'D:/DEVLib/PX4APM/APM/ardupilot-Plane-4.7.2',
    'PLANE': u'D:/DEVLib/PX4APM/APM/ardupilot-Plane-4.7.1',
    'NUTTX': u'D:/DEVLib/PX4APM/PX4/PX4-Autopilot/platforms/nuttx/NuttX/nuttx',
    'NUTTX_SRC': u'D:/DEVLib/PX4APM/Nuttx',
    'CLIB': u'D:/DEVLib/PX4APM/PX4/c_library_v2-master',
    'BF': u'D:/DEVLib/PX4APM/betaflight',
    'AEQ': u'D:/DEVLib/PX4APM/attitude_estimator_q',
    'DCM': u'D:/DEVLib/PX4APM/方向余弦',
    'QGC': u'D:/DEVLib/PX4APM/QGC/qgroundcontrol-5.0.4',
    'TEST': u'D:/DEVLib/PX4APM/test',
    # ---- 自研硬件 ----
    'BSP': u'D:/DEVLib/PX4APM/硬件/版本V1.4 - 从零开始手搓无人机-B站：千古八方的玩具'
           u'/PX4-FMU-v6x/fmu-v6x-qgbf-20251116/fmu-v6x-qgbf',
    'HW': u'D:/DEVLib/PX4APM/硬件/版本V1.4 - 从零开始手搓无人机-B站：千古八方的玩具',
    'HW11': u'D:/DEVLib/PX4APM/硬件/版本V1.1-封存归档',
    # 硬件**总目录**（递归到各版本）—— 与上面 `HW`（V1.4 那一版）不是一回事。
    # `_check_citations.py` 的 ROOTS 要的是这个：文档里写的是
    # `硬件/版本V1.4 …/fmu-v6x-qgbf/…`，索引根得能覆盖到各版本才行。
    'HW_ROOT': u'D:/DEVLib/PX4APM/硬件',
    # ---- 素材与数据（教程正文、真机日志） ----
    'MAT': u'D:/DEVLib/PX4APM/飞控学习教程',
    'MAT_RTOS': u'D:/DEVLib/PX4APM/FreeRTOS-ChibiOS教程',
    'DOC_NUTTX': u'D:/DEVLib/PX4APM/NuttX教程',
    'ULOG': u'D:/DEVLib/PX4APM/PX4调参/ulog',
}


def src(name, *sub):
    """`src('FREERTOS')` / `src('PX4', 'src')` —— 环境变量优先，其次默认。

    ★ 名字没登记且没有对应的 `SRC_<名字>` 环境变量 ⇒ **抛 KeyError**。
      返回 `''` 会让调用方悄悄退化成"相对路径 / 目录不存在"，比报错难查得多。
    """
    if 'SRC_' + name in os.environ:
        base = os.environ['SRC_' + name]
    elif name in DEFAULTS:
        base = DEFAULTS[name]
    else:
        raise KeyError(u'src(): 未知名字 %r。已登记：%s（或用环境变量 SRC_%s 指定）'
                       % (name, u'、'.join(sorted(DEFAULTS)), name))
    return os.path.join(base, *sub) if sub else base
