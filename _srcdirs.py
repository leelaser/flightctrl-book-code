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

⚠️ 默认值里那条 `硬件/版本V1.4 …` 是**作者本机**的目录名，只为让作者这端仍能跑通；
   读者请用 `SRC_BSP` 指到自己的 BSP 目录。
"""
import os

DEFAULTS = {
    'FREERTOS': u'D:/DEVLib/PX4APM/FreeRTOS',
    'PX4': u'D:/DEVLib/PX4APM/PX4/PX4-Autopilot',
    'APM': u'D:/DEVLib/PX4APM/APM/ardupilot-Copter-4.5.7/ardupilot-Copter-4.5.7',
    'PLANE': u'D:/DEVLib/PX4APM/APM/ardupilot-Plane-4.7.1',
    'NUTTX': u'D:/DEVLib/PX4APM/PX4/PX4-Autopilot/platforms/nuttx/NuttX/nuttx',
    'BSP': u'D:/DEVLib/PX4APM/硬件/版本V1.4 - 从零开始手搓无人机-B站：千古八方的玩具'
           u'/PX4-FMU-v6x/fmu-v6x-qgbf-20251116/fmu-v6x-qgbf',
    'HW': u'D:/DEVLib/PX4APM/硬件/版本V1.4 - 从零开始手搓无人机-B站：千古八方的玩具',
    'HW11': u'D:/DEVLib/PX4APM/硬件/版本V1.1-封存归档',
    'APM_471': u'D:/DEVLib/PX4APM/APM/ardupilot-Copter-4.7.1',
    'PX4_115': u'D:/DEVLib/PX4APM/PX4/PX4-Autopilot-1.15.4/PX4-Autopilot-1.15.4',
}


def src(name, *sub):
    """`src('FREERTOS')` / `src('PX4', 'src')` —— 环境变量优先，其次默认。"""
    base = os.environ.get('SRC_' + name) or DEFAULTS.get(name, '')
    return os.path.join(base, *sub) if sub else base
