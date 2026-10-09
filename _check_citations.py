# -*- coding: utf-8 -*-
"""引用核查 —— 本线「每个论断都要能落到 `文件:行号`」这条纪律的执行版。

为什么要它
=======================================================================
本线的正文里有一千多条 `文件:行号` 引用。它们不可能靠人眼核对，
而**错的行号不会以任何方式暴露** —— 函数名还是那些，只是指错了地方。

作者在第 17 课就真的写错过：
    `hal_serial_lld.h:655`  ← 想引 `End of the mandatory fields`
    实际那行注释在 `:620` 和 `:654`
    而 655 是个**完全合法**的行号（文件有 1000 行），所以
    "行号在范围内"这种检查**根本抓不到它**。

⇒ 所以这个脚本分三层，越往下越严：

    [1] 文件找得到吗（含"同名文件有多个"的歧义报告）
    [2] 行号在文件范围内吗
    [3] ★ 那一行**有实质内容**吗（不是空行、不是 /*===*/ 分隔线）

★ [3] 只挡得住"指到了空处"这一类错误。**它挡不住"指到了旁边
  一条无关的语句"** —— 那种只能靠 -v 打印出来人工抽查，
  或者靠 `_check_quotes.py` 那种"引文比对"。

用法：
    python _check_citations.py            # 三层检查 + 统计
    python _check_citations.py -v         # 顺带打印每一条引用指向的内容
    python _check_citations.py -v 17      # 只看第 17 课

退出码：0 = 没问题，1 = 发现问题。
"""

import io
import _srcdirs
import os
import re
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)

# 本书自己的书稿根（脚本现在住在 `飞控丛书/code/` 下 ⇒ ROOT 就是书根）。
SELF_BOOK = os.path.join(ROOT, '书稿')

# 要检查的**课文**在哪几个根下（注意：这和 ROOTS 是两回事 ——
# ROOTS 是**被引用源码**的索引，DOC_ROOTS 是**检查对象**所在处）。
DOC_ROOTS = [
    ROOT,                                  # 本教程：FreeRTOS-ChibiOS教程
    _srcdirs.src("MAT"),          # 姊妹教程：飞控学习教程
    # ★ 第三套教程（2026-09 补）——
    #   加飞控教程时**漏了它**，于是同一个错误又犯了一遍：
    #   NuttX 教程 11 课里的 `文件:行号` 同样从来没被验证过。
    #   ⇒ 教训升级：**每新加一套教程，必须同时把它登记进这里**，
    #     否则「加了检查器」给人的安全感是假的。
    _srcdirs.src("DOC_NUTTX"),
    # ★★ 2026-10 补：**本书自己的书稿**（《飞控丛书》）。
    #   本脚本随 `deposit_scripts.py` 并进本书的 `code/` 之后，
    #   才第一次被用来扫书稿 —— 而它**一条都没扫到**：
    #   目录模式写的是 `^\d\d[a-z]?[-_]`，而本书的卷目录叫
    #   `卷1-数学与动力学基础` / `卷4-实时系统原理与FreeRTOS`，
    #   开头是汉字「卷」⇒ 整个 15 卷、115 章被**静默跳过**，
    #   报的是「✅ 4306 条引用全部有效」——**那 4306 条里没有一条来自本书**。
    #   ⚠️ 这和本函数上面三段注释记的**是同一个坑的第四次**：
    #      `03b-`、`10-`、以及这次「卷N-」。三次都是"模式比对象窄"，
    #      三次的症状都是"总数纹丝不动、报告照绿"。
    #   ⇒ 所以这次不只加一个分支，还要**加一条自证**（见 walk_docs 里）。
    SELF_BOOK,
]

# 四个源码根 —— 本线的引用都落在这四棵树里
ROOTS = [
    _srcdirs.src("FREERTOS"),
    _srcdirs.src("APM"),
    # ★★ 2026-09 补第七棵树：**ArduPilot Plane 4.7.1** —— 固定翼那几课的真身。
    #   `03b-机型动力学/06、07、08` 与 `07-仿真与实验/03` 是**固定翼**的，
    #   它们引的 `SIM_Plane.cpp` / `ArduPlane/Plane.cpp` / `vehicleinfo.py`
    #   全部是**对着 Plane 4.7.1 写的**，而原来只有的 4.5.7 那棵树里：
    #     · `libraries/SITL/SIM_Plane.cpp` 406 行（4.7.1 是 551）⇒ `:418 / :464` 越界
    #     · `ArduPlane/Plane.cpp` **只有 36 行（存根）**（4.7.1 是 1110）⇒ `:1052` 无处可指
    #     · `Tools/autotest/pysim/vehicleinfo.py` 487 行（4.7.1 是 575）⇒ `:559` 越界
    #   逐行抽查确认**两版逻辑相同、只是整体差 126 行**
    #   （4.5.7 的 `if (elevons)` 在 283、`rudder = 0` 在 292；4.7.1 在 409 / 418）
    #   ⇒ **不是课文引错了行，是课文引到了另一棵树。** 病根在这里，不在课文。
    #
    #   ⚠️ 只加 **Plane** 这一棵。盘上同时躺着 Copter/Rover/Sub 三棵 4.7.1，
    #   它们的 `libraries/` 与 Plane 树重复 —— 加进来只会把**同名文件的稀释放大 4 倍**
    #   （见本文件反复强调的「同名文件是这类检查器的天敌」），**一个引用也覆盖不到新的**。
    _srcdirs.src("PLANE"),
    _srcdirs.src("PX4"),
    _srcdirs.src("DOC_NUTTX"),
    # ★ NuttX **源码树**（2026-09 补）——
    #   注意：和上面那条「NuttX教程」是两回事，一个是教程文档，一个是上游源码。
    #   漏了它的症状极具迷惑性：把「NuttX教程」登记成文档根之后，
    #   检查器报出 8 处「找不到 led_main.c」，**看起来像教程写错了文件名** ——
    #   而文件好好的在 `Nuttx/stm32h7_fmu-master/src/drivers/led/`，
    #   只是**源码索引里没有这棵树**。
    #   ⇒ 又一次印证那句话：**检查器报错，先分清是「文档错」还是「检查器瞎」。**
    _srcdirs.src("NUTTX_SRC"),
    # ★ 教程自己的根也要在（2026-09 补）——
    #   否则正文里「`code/lessonNN_xxx.py:288`」这种**自引用**会被判成
    #   "找不到文件"，而它明明就在旁边。
    #   当时的处理是逐个把 `:288` 改写成"的 288 行"绕过去 ——
    #   **绕过去 = 没修**：下一个人再写一次 `code/xxx.py:NN`，同样会撞上。
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    # ★ 姊妹教程「飞控学习教程」的根（2026-09 补）——
    #   它和本教程**共用同一批源码仓库**（PX4 / ArduPilot / NuttX），
    #   引用格式一模一样，所以并进同一个 ROOTS 核查，而不是再养第二份检查器
    #   （两份 = 迟早不同步 = 有一份是假的）。
    #
    #   为什么必须加：那套教程已经写了 20+ 课、上千条 `文件:行号`，
    #   而在这之前**没有任何检查器看过它一眼**。
    #   本文件的 ROOTS 推出来的是 FreeRTOS-ChibiOS教程 自己，
    #   飞控教程在隔壁目录 ⇒ `os.walk` 根本走不到 ⇒ 引用再多也是黑盒。
    #   **检查器覆盖不到 = 没检查**，这正是它存在的意义。
    _srcdirs.src("MAT"),
    # ★ 读者的**自研硬件项目**（2026-09 补）。
    #   飞控教程第 6 章 03 课（板级支持包）会引用这块板子的 BSP，
    #   路径形如 `硬件/版本V1.4 .../fmu-v6x-qgbf/src/board_config.h:212`。
    #
    #   **漏了它的后果比"找不到文件"更坏 —— 是"假绿"**：
    #   检查器按 basename 匹配、且「任意一个同名文件命中即算过」，
    #   而 `board_config.h` 全项目有 80 个同名文件、`spi.cpp` 76 个。
    #   于是那些指向**读者板**的行号，被静默解析到了**官方树**上，
    #   只要官方文件对应行号恰好有内容就算"有效" —— 实测举例：
    #     引用 `board_config.h:212`（读者板：HW 版本三引脚）
    #     实际核对的是官方 `:212` = `HW_INFO_INIT_PREFIX "ARKV6X"`
    #   **两个完全不同的东西，检查器报 ✅。**
    #   ⇒ 教训：**「同名文件」是这类检查器的天敌**，
    #     它是为了容忍"同名文件里总有一个对"而放宽的，
    #     代价就是——**同名文件越多，假绿越容易**。
    _srcdirs.src("HW_ROOT"),
    # ★★ 2026-09 再补六棵树 —— 全部是**各课作者报上来的**，不是我想出来的。
    #   规律很清楚：**每写一门引用新素材的课，就要同时把那棵树登记进来**，
    #   否则那一课的引用**从一开始就是无检查状态**（本项目已经犯过三次：
    #   飞控教程的源码引用、NuttX 教程、现在这批）。
    #
    #   报缺的课 → 缺的树：
    #     06-04 MAVLink 课  → test/（读者的 MAVLink C++ 代码）、
    #                          QGC/、PX4/c_library_v2-master/（MAVLink 库）
    #     RTOS 02 任务与调度器 → betaflight/（协作式调度器）
    #     04-05 Mahony 课     → attitude_estimator_q/（旧快照，讲"模块怎么变简单"）
    #     01-04 DCM 课        → 方向余弦/（MatrixPilot 的 libDCM）
    _srcdirs.src("TEST"),
    _srcdirs.src("QGC"),
    _srcdirs.src("CLIB"),
    _srcdirs.src("BF"),
    _srcdirs.src("AEQ"),
    _srcdirs.src("DCM"),
]

# 引用的形态：  name.c:123   name.h:123-456   name.py:12,34
#
# ★ 后缀表 2026-09 补了 `.s`（小写）—— ChibiOS 里有 47 个 `.s`
#   （IAR / RVCT 工具链的汇编），原来只认大写 `.S`，对 `.s` 的引用一律漏检。
#
# ⚠️ 但**不要**往里加 `.txt` / `.json` —— 试过了，会立刻带出 8 处误报，
#   全是 `readme.txt` / `x.txt` / `CMakeLists.txt` 这种**泛指**：
#   正文说"去看 CMakeLists.txt"，指的不是某一个具体文件。
#   ⇒ 判据是：**这个后缀在正文里会不会被当普通名词用**。
#     `.c`/`.h`/`.s` 不会（没人泛指"某个 .c"），`.txt` 会。
# ★ 文件名主体里**允许点号**（2026-09 修）。
#   原来写的是 `[A-Za-z0-9_\-]*` —— 没有点。后果是**多级点号的文件名被从中间截断**：
#     `kernel.space.ld`  → 匹配成 `space.ld`
#     `user-space.ld`    → 这个反而没事（`-` 在类里）
#   于是 NuttX 教程里引用 `kernel.space.ld` 的地方，
#   检查器报「找不到 space.ld」—— 一个**根本不存在的文件名**。
#   ⇒ 报错信息里的文件名如果看着很怪，先怀疑**检查器截错了**，别急着改课文。
# ★ 2026-09 大改：**把「引用里带的路径」也抓出来**。
#   原来的正则只抓文件名，于是本文件最大的弱点一直无法修：
#   它按 **basename** 建索引、且「同名文件里任意一个命中就算过」——
#   而项目里 `board.h` 有 462 个同名文件、`hwdef.dat` 301 个、
#   `board_config.h` 81 个、`spi.cpp` 77 个。
#   ⇒ 那些引用的「✅ 有效」是**打折扣的**：只证明"某个同名文件的该行非空"。
#   实测过一次假绿：飞控教程引读者自研板的 `board_config.h:212`（HW 版本引脚），
#   检查器实际核对的是官方树的 `:212`（`HW_INFO_INIT_PREFIX "ARKV6X"`）——**两回事**。
#
#   现在把路径抓出来，匹配时**优先按路径消歧**（见下面的 `narrow_by_path`）。
#
#   ⚠️ 路径字符类**必须含 `/` 和 `\`** —— 第一版漏了，结果 `src/modules/ekf2/EKF`
#      只能匹配到第一个斜杠前的 `src`，路径形同虚设。**这类"看似生效实则没有"的
#      改动最危险，所以改完必须拿真实样例逐条验证。**
BS = chr(92)          # 反斜杠 —— 显式构造，免得被 heredoc 之类的层吃掉

CITE = re.compile(
    r'(?:(?P<path>[\w一-鿿（）()\[\]《》/\ ．.\-]{1,140})[\/])?'
    r'(?P<name>[A-Za-z_][A-Za-z0-9_.\-]*\.(?:c|h|py|cpp|hpp|mk|S|s|ld|dat|cmake|px4board|cc))'
    r':(?P<spec>\d+(?:\s*[-,]\s*\d+)*)'
)

SKIP_DIRS = {'.git', 'build', '__pycache__', 'node_modules',
             'demos', 'testex', 'testhal', 'os/ex'}
MAX_DEPTH = 12


def build_index():
    """{basename: [完整路径, ...]} —— 只收名字里带扩展名的常规文件。

    ★ 排除 demos/ 和 test*/：那里有几百个同名 chconf.h / halconf.h，
      不排掉的话"歧义"会淹没真正的问题（实测：不排有 300+ 个 chconf.h）。
    """
    idx = {}
    for root in ROOTS:
        if not os.path.isdir(root):
            continue
        # ★ 路径分隔符要归一化（2026-09 修）——
        #   原来写的是 `root.rstrip('/').count('/')`，**只认正斜杠**。
        #   而 `ROOTS` 里那一条"教程自己的根"是用 __file__ 推出来的，
        #   在 Windows 上是**反斜杠**路径 ⇒ `count('/')` 得 0 ⇒
        #   `base_depth` 算成 0，于是整个教程目录被 MAX_DEPTH 判为"太深"而跳过。
        #   **症状是"找不到文件"，病因在深度计算** —— 差了三层。
        base_depth = root.replace('\\', '/').rstrip('/').count('/')
        for dirpath, dirnames, filenames in os.walk(root):
            if dirpath.count('/') - base_depth > MAX_DEPTH:
                dirnames[:] = []
                continue
            dirnames[:] = [d for d in dirnames
                           if d not in SKIP_DIRS and not d.startswith('.')]
            if any(part in SKIP_DIRS for part in dirpath.replace('\\', '/').split('/')):
                continue
            for f in filenames:
                # ★★ 这张表必须和 `CITE` 的**完全一致**。
                #   实测踩过：`CITE` 加了 `.cmake` 而这里没加 ⇒ 报 166 处
                #   「找不到文件 `px4_add_module.cmake`」——**而那个文件明明存在**
                #   （`cmake/px4_add_module.cmake`）。症状极像"课文写错文件名"，
                #   实则是**两张表打架**：一个负责"认得出"，一个负责"找得到"。
                #   同理这里原来还漏了 `.s`（小写），而 `CITE` 里有。
                if f.endswith(('.c', '.h', '.py', '.cpp', '.hpp', '.mk',
                               '.S', '.s', '.ld', '.dat', '.cmake', '.px4board', '.cc')):
                    idx.setdefault(f, []).append(os.path.join(dirpath, f))
    return idx


def parse_ranges(spec):
    r"""`123` / `123-456` / `12,34` ⇒ [(a, b), ...]

    ★★ 2026-09 修一个**方向相反**的 bug（本项目第 6 次栽在同一类坑上）：

       原来的实现是：

           for part in re.split(r'[,\-]', spec):  ...收集数字...
           return [(out[0], out[-1])]             # ← 把首尾两个数当成一个【区间】

       但 docstring 写的是「`12,34` ⇒ 两个独立位置」。
       于是**逗号被当成了连字符**，后果两个方向都坏：

         · `:100,218,250,323,474` → 变成区间 (100, 474)，
           **只要这个宽区间里有任意一行有内容就算过**
           ⇒ **假阴性**：单独某一行号写错，检查器照样报绿。
         · `:42,28`（降序）→ 变成区间 (42, 28)，
           `range(41, 28)` 为空 ⇒ 那两行明明都有内容，却报「空行」
           ⇒ **假阳性**。

       ⇒ **逗号是枚举，连字符才是区间。** 两者必须分开处理。

       ⚠️ 这个 bug 是写第 08 课时被 `AR_PivotTurn.cpp:42,28` 触发的 ——
          降序的巧合让假阳性暴露了出来。**升序的那些一直静默地假绿着。**
    """
    out = []
    for part in re.split(r',', spec):          # 先按逗号切成若干【项】
        part = part.strip()
        if not part:
            continue
        if '-' in part:                        # 每一项内部才可能是区间
            a, _, b = part.partition('-')
            if a.strip().isdigit() and b.strip().isdigit():
                out.append((int(a), int(b)))
        elif part.isdigit():
            n = int(part)
            out.append((n, n))
    return out


def substantive(line):
    r"""这一行算不算「有实质内容」？（用来判断引用的行号是不是指了个空行/分隔线）

    ★ 用**原始字符串**做 docstring：正文里要写 `\*` `\-` 这类正则片段，
      普通三引号字符串会把它们当成非法转义并抛 SyntaxWarning。

    ★★ 2026-09 修一个**静默失效**的 bug（本项目第 4 次栽在同一类坑上）：
       本函数原来三个正则的**行尾锚点 `$` 被吃掉了**，写坏成了 `/?` 和 `/`：

           r'[/\*=\-]{6,}\**$'   →   r'[/\*=\-]{6,}\**/?'
           r'\*+$'               →   r'\*+/'
           r'\*+$'               →   r'\*+/?'

       而 `?` 在正则里是**字面问号**，于是判据悄悄变成了
       「整行必须恰好是 `*****/?` 这种带问号的串」—— **永远不成立**，
       三个分支全部形同虚设，等于这个函数只剩 `if not t`。

       危害方向是**假阴性**：分隔线不再被过滤 ⇒ 引用指向分隔线时不报错
       ⇒ 检查器看着"更绿"，其实漏报。
       （好在方向是漏报而不是误报 —— 误报会推着人去改本来就对的地方。）

       ⇒ 教训：`$` 被吃掉这种错，**跑起来一点症状都没有** ——
         不抛异常、不报错、结果还"更干净"。只能靠拿真实样例验。
    """
    t = line.strip()
    if not t:
        return False
    # `/*=====*/`、`//-----`、`=====` 这类纯分隔线
    if re.fullmatch(r'[/\*=\-]{6,}\**$', t):
        return False
    # 纯 `*`（可带一个收尾 `/`）组成的空注释行；`/**< 带内容的` 不算
    if re.fullmatch(r'\*+/?$', t):
        return False
    return True


def md_files(only=None):
    """列出要检查的课文，返回 [(绝对路径, 显示名)]。

    ★ 2026-09 改成**多文档根**。
      原来只有 `os.listdir('.')` 一棵树（= FreeRTOS-ChibiOS教程），
      姊妹教程「飞控学习教程」在隔壁目录 ⇒ 永远轮不到它。
      症状极具欺骗性：检查器报告「✅ 2847 条引用全部有效」，
      **看起来是绿的**，但那 2847 条里没有一条来自飞控教程 ——
      它那 20+ 课上千条 `文件:行号` 从来没被看过一眼。
      ⇒ 教训：**一个检查器报绿，先问它到底扫了谁**。
    """
    out = []
    for base in DOC_ROOTS:
        if not os.path.isdir(base):
            continue
        for d in sorted(os.listdir(base)):
            # 顶层目录形如 `01-FreeRTOS` / `04-状态估计`。两套教程的目录名
            # 天然不重名（`01-FreeRTOS` vs `01-数学基础`），所以显示名
            # 直接用 `目录/文件` 就够区分，per_lesson 不会串。
            # ★ 2026-09 补：原来是 `^0\d-`，漏了 `03b-机型动力学` 这种
            #   「数字+字母」的章号 —— 整个目录被**静默跳过**，
            #   引用再多也不验。和 §DOC_ROOTS 那条教训是同一个形状：
            #   **目录名模式一变窄，覆盖面就跟着变小，而且不会报错。**
            # ★★ 2026-09 再补（第二次踩同一个坑）：改成了 `^0\d[a-z]?[-_]`，
            #   但那只覆盖 01~09。**`10-ArduPilot算法全解/` 第一个字符是 `1`，
            #   整个新章（4 篇、几十条 `文件:行号`）依然是黑盒** ——
            #   症状还是那个：总数纹丝不动，报绿。
            #   ⇒ 教训升级：**光"把漏的那个补进模式"不够，
            #     要按【章号的一般形式】写，而不是按"目前见过的章号"写。**
            #     章号 = 两位数字 + 可选小写字母。01~99 全覆盖。
            # ★★ 第四次修同一个坑（2026-10）：**本书的卷目录叫 `卷N-...`**，
            #   不含数字开头 ⇒ 上一条模式照样漏。这次按「目录名的一般形式」写：
            #     · 教程：两位数字 + 可选小写字母（01-、03b-、10-、99-）
            #     · 本书：`卷` + 数字 + `-`（卷1-、卷11-）
            #   ⚠️ 但**光加模式还是不够** —— 下面配了一条"扫到 0 个文件就报错"
            #      的自证，免得下一次换目录名又静默跳过。
            if re.match(r'^\d\d[a-z]?[-_]|^卷\d+[-_]', d) \
                    and os.path.isdir(os.path.join(base, d)):
                for f in sorted(os.listdir(os.path.join(base, d))):
                    if f.endswith('.md'):
                        if only and not f.startswith("%02d-" % int(only)):
                            continue
                        out.append((os.path.join(base, d, f), "%s/%s" % (d, f)))
    # ★★ 自证：每个"看着有东西"的根，都必须真的扫出文件来。
    #   前四次踩坑的症状完全一样 —— **某个根贡献 0 个文件，而报告照绿**。
    #   判据：目录存在、且里面**有**符合章号模式的子目录，却没扫到任何 .md
    #   ⇒ 报出来，别默认放行。
    for base in DOC_ROOTS:
        if not os.path.isdir(base):
            continue
        kids = [d for d in os.listdir(base)
                if os.path.isdir(os.path.join(base, d))]
        has_chapdir = any(re.match(r'^\d\d[a-z]?[-_]|^卷\d+[-_]', d) for d in kids)
        got = [x for x in out if os.path.dirname(x[0]).startswith(base)]
        if has_chapdir and not got:
            print('  ⚠️ 根「%s」看着有章节目录，却一条都没扫到 —— '
                  '目录模式可能又窄了（本坑已犯四次）' % base)
    return out


def main():
    verbose = '-v' in sys.argv
    only = None
    for a in sys.argv[1:]:
        if a.isdigit():
            only = int(a)

    print("[0] 建立源码索引")
    idx = build_index()
    total_files = sum(len(v) for v in idx.values())
    print("    %d 个不同文件名 / %d 个文件" % (len(idx), total_files))

    print("\n[1-3] 逐条核对引用")
    bad = []
    ambiguous = {}
    n_cite = 0
    n_narrowed = 0        # 靠路径消歧过的引用条数
    per_lesson = {}
    cache = {}

    def lines_of(path):
        if path not in cache:
            try:
                cache[path] = io.open(path, encoding='utf-8',
                                      errors='replace').read().split('\n')
            except OSError:
                cache[path] = []
        return cache[path]

    for fp, f in md_files(only):
        s = io.open(fp, encoding='utf-8').read()
        cnt = 0
        for m in re.finditer(CITE, s):
            name, spec, pref = m.group('name'), m.group('spec'), m.group('path')
            cnt += 1
            n_cite += 1
            # ★ 2026-09：报错要带**在 md 里的行号**，不能只报文件名。
            #   原来只给 `f`（如「06-软件架构/03-….md」），
            #   而那一课有 2000+ 行、几十条引用 ⇒ 找这一条只能全文搜。
            #   实测：就因为没行号，一个错被反复"看到但定位不到"。
            loc = "%s:%d" % (f, s.count('\n', 0, m.start()) + 1)
            cands = idx.get(name)
            if not cands:
                bad.append("%s —— 找不到文件 `%s`" % (loc, name))
                continue
            was_narrowed = False    # 下面按路径消歧成功时置 True（影响报错措辞）
            # ★ 引用带路径时，先按路径把候选缩到「路径尾匹配」的那些。
            #   这是修「同名假绿」的关键一步：把"撞运气"换成"按作者写的路径找"。
            if pref and len(cands) > 1:
                want = pref.replace(BS, '/').rstrip('/')
                # ★★ 2026-09 修一个【死代码】—— 作者第一版写的是
                #     `q.rstrip('/').endswith(want)`，
                #   其中 `q` 是**完整文件路径**（`.../src/board_config.h`），
                #   而 `want` 是**目录**（`.../src`）⇒ **永远 False**，
                #   `narrowed` 恒为空、`n_narrowed` 恒为 0。
                #   症状极隐蔽：机制装好了、代码看着也对、**就是永远不生效**。
                #   是硬件课的作者拿一个真实引用去验，才发现它一条都没缩。
                #   ⇒ 正确做法是**拿文件所在目录比**（或拿 `want + '/' + name` 比）。
                #   **教训与上面那条一样：改完必须拿真实样例验，不能看代码"像是对的"。**
                narrowed = [q for q in cands
                            if os.path.dirname(q.replace(BS, '/')).rstrip('/').endswith(want)]
                if narrowed:
                    cands = narrowed
                    n_narrowed += 1
                    was_narrowed = True
            if len(cands) > 1:
                # ★ 2026-09：除了记「有几个同名」，还记【谁引用了它】——
                #   因为判据是「同名里任意一个成立就算过」，同名越多越像撞运气。
                #   列出来是为了能**人工抽查**（`-v` 只看一条，这里给全貌）。
                ambiguous.setdefault(name, [len(cands), []])[1].append(
                    "%s:%d" % (f, cnt))
            # 所有同名文件里，只要有一个"行号在范围内且那行有内容"就算过
            #
            # ★★ 2026-09 第二个假阴性：**同一个引用里的多个行号，原来是"任意一个
            #    成立就算过"**（`any(...)`）。于是 `:100,218,250,323,99999`
            #    这种「5 个里错了 1 个」的引用照样报绿 ——
            #    因为另外 4 个是对的。
            #
            #    但一个引用列了 5 个行号，**其中 1 个错，这条引用就是错的**。
            #    ⇒ 判据改成：**所有行号都必须在【同一个文件】里成立**。
            #
            #    ⚠️ 这个假阴性是拿反向测试逼出来的：作者把 474 改成 99999，
            #       grep 确认注入生效，检查器**依然报绿**。
            #       如果只跑"改了没报错"这一步，会得出"修好了"的错误结论。
            ok = False
            hit_desc = ""
            ranges = parse_ranges(spec)
            for p in cands:
                ls = lines_of(p)
                if not ls:
                    continue
                # ★ 这个文件必须让**每一个**行号都成立才算命中
                good = True
                first = None
                for a, b in ranges:
                    if a < 1 or b > len(ls):
                        good = False
                        break
                    # ★ 区间引用【不能只查首行】—— `heap_1.c:69-116` 指的是
                    #   一整个函数，69 行很可能是 `void *pvPortMalloc(...) {`
                    #   前面那行空行。作者第一版只查首行，于是报了一片误报。
                    #   ⇒ 单行：那一行要有内容；区间：区间里【至少有一行】有内容。
                    if not any(substantive(ls[i]) for i in range(a - 1, b)):
                        good = False
                        break
                    if first is None:
                        first = a
                if good:
                    ok = True
                    hit_desc = "%s:%d  %s" % (os.path.basename(p), first,
                                              ls[first - 1].strip()[:76])
                    break
            if not ok:
                in_range = any(1 <= a and b <= len(lines_of(p))
                               for p in cands for a, b in parse_ranges(spec))
                # ★ 2026-09：措辞必须区分「消歧前」和「消歧后」——
                #   本文档自己写过 `stm32h7/stm32_allocateheap.c:345-420`：
                #   消歧后候选只剩 401 行那一个 ⇒ 420 越界（**文档真错**）；
                #   但若报成"超出所有同名文件的范围"，复核的人会去翻 801 行的
                #   `stm32/` 版本（那个装得下 420），从而**误判成检查器瞎报**并
                #   放过真错。实测确认过：这条误判真的发生了。
                #   ⇒ 报错措辞不精确，会把复核引到错误方向。这是"改完必须拿
                #     真实样例验"的又一个反面教材（验的前提是读懂报错在说什么）。
                scope = "按路径消歧后的候选文件" if was_narrowed else "所有同名文件"
                why = ("行号 %s 超出%s的范围" % (spec, scope) if not in_range
                       else "行号 %s 指向的是空行/分隔线" % spec)
                bad.append("%s —— `%s:%s` %s" % (loc, name, spec, why))
            elif verbose:
                print("      %-40s %-34s → %s" % (f[:38], "%s:%s" % (name, spec),
                                                  hit_desc))
        per_lesson[f] = cnt

    print("\n[4] 出处密度")
    for _fp, f in md_files(only):
        print("    %-46s %4d 条引用" % (f, per_lesson.get(f, 0)))
    print("    %-46s %4d 条" % ("合计", n_cite))

    if ambiguous:
        print("\n[5] 有同名文件的引用（不报错，但列出来）")
        for n, (c, whos) in sorted(ambiguous.items(), key=lambda x: -x[1][0])[:12]:
            print("    %-24s 有 %d 个同名文件" % (n, c))
            if whos:
                # 只显示前 3 个出处，避免刷屏
                shown = "、".join(whos[:3])
                more = "（还有 %d 处）" % (len(whos) - 3) if len(whos) > 3 else ""
                print("        ↳ 被引用于：%s%s" % (shown, more))
        print("    ★ 判据说明（2026-09 更新）：")
        if n_narrowed:
            print("      · **按路径成功消歧**的：%d 条 —— 这些引用只认一个文件，不再撞运气。"
                  % n_narrowed)
        else:
            print("      · **按路径成功消歧**的：0 条 —— 说明这些同名引用【都是裸文件名写法】，")
            print("        或者写了路径但没能唯一锁定。它们仍然只能撞运气。")
        print("      · 想让某条引用变可信，**最省事的办法就是在写法上带路径** ——")
        print("        例如把 `board_config.h:323` 写成 "
              "`硬件/…/fmu-v6x-qgbf/src/board_config.h:323`，")
        print("        检查器就只认那一个文件，不再被 81 个同名文件稀释。")
        print("      · 上面的「被引用于」列出来就是给你抽查的：**同名越多、越该去看一眼**。")

    print()
    if bad:
        print("=" * 74)
        for b in bad:
            print("  ❌ " + b)
        print("=" * 74)
        print("\n合计 %d 处问题（共 %d 条引用）" % (len(bad), n_cite))
        return 1
    print("=" * 74)
    print("  ✅ %d 条引用全部有效" % n_cite)
    print("=" * 74)
    return 0


if __name__ == '__main__':
    sys.exit(main())
