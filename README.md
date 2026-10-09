# 《飞控丛书》配套代码

> 仓库：**https://github.com/leelaser/flightctrl-book-code**
> 各卷卷首语 §三「配套代码与里面的数字」有同一份说明的简版。

这套脚本是丛书的**可复算凭证**：正文里的每个数字都由它们跑出来，
而且它们在开工前会 `assert` 自己的输入规模 —— 对不上就直接报错，不会默默给一个值。

> 书里每章末尾的「源码与资料索引」列出该章用到的脚本（`code/…`）；
> 本目录就是那个 `code/`。

## 一、怎么跑

```bash
pip install -r requirements.txt      # numpy / matplotlib（另有 2 个脚本可选装 pyulog、pymavlink）
python lesson52_simparams.py         # 跑某一个
```

- **Python 3.8+**（作者用的是 3.14.3）。
- 多数脚本**不带参数就是跑全部实验**；带数字参数是跑指定实验，例如
  `python lesson23_sitl.py 3`（跑第 3 个实验）。用法见各脚本开头。
- 退出码 0 = 全部断言通过；非 0 = 有断言没过（脚本会打印是哪一条）。

**跑全部**（依次跑完本目录所有脚本，约 20 分钟）：

```bash
for f in *.py; do python "$f" >/dev/null 2>&1 || echo "❌ $f"; done
```

> 跑完**一个 ❌ 都没有**才算过。跳过型脚本（找不到源码树/WSL 时）会打印说明并退出 0，
> 所以"全绿"只说明**它跑过的那些**没问题 —— 与正文的口径一致。

## 二、脚本要读的上游源码：`SRC_*` 环境变量

有 36 个脚本要读**本机上的源码树与素材目录**（FreeRTOS / PX4 / ArduPilot /
NuttX / 本书自研板的 BSP / 真机日志 …）。它们的位置由 **`_srcdirs.py`**
统一解析，**优先环境变量**：

```bash
export SRC_FREERTOS=/path/to/FreeRTOS
export SRC_PX4=/path/to/PX4-Autopilot      # 1.15.4 那份用 SRC_PX4_115
export SRC_APM=/path/to/ardupilot          # Copter 4.5.7；4.7.1 用 SRC_APM_471
export SRC_PLANE=/path/to/ardupilot-Plane  # Plane 4.7.1
export SRC_NUTTX=/path/to/nuttx
export SRC_BSP=/path/to/fmu-v6x-qgbf       # 本书自研板的 BSP（卷 9）
```

**不设也能跑**：会回落到 `_srcdirs.py` 里 `DEFAULTS` 的路径（作者本机的）。
那棵树你这里没有时，脚本会打印「未找到 …」并**跳过**相关实验，不会崩。
想改默认值，改 `_srcdirs.py` 一个文件即可。

- 完整键表（21 个：上游源码 / 自研硬件 / 素材与数据）见 `_srcdirs.py` 开头。
- 脚本的文档字符串里出现的 `$SRC_XXX/子路径`，说的就是这个键。
  它和**书里正文**的 `$SRC/<树名>`（见各卷卷首语 §二）是同一件事的两种写法：
  书里一个根、代码里一棵树一个变量 —— 因为这几棵树在作者机器上并不同源同根，
  统一成一个根反而会在读者那边对不上。
- 取一个没登记的名字会**当场抛 KeyError**（早先返回空串，会让整段逻辑静默跳过）。

## 三、断言与「反向注入」

- **断言**：脚本把结论写成 `check(条件, 说明)`。有条件不成立就报错并给非零退出码 ——
  这是"数字能被复算"的第一层保证。
- **反向注入**：光有断言还不够 —— **断言可能根本没在验东西**。
  `inject_l*.py`（15 个）就是为此存在的：它们把被测脚本里的某个事实/公式**故意改错**，
  然后要求断言**变红**。变红 = 那条断言确实在守；没变红 = 那条断言是空的。

```bash
python inject_l52.py     # 例：对 lesson52_simparams.py 做 39 条注入，逐条要求变红
```

注入脚本会在 `finally` 里把被测文件**原样还原**（中途出错也还原）。

## 四、这套脚本自己的诚实边界

- 正文里的数**不都**出自脚本：源码里的参数默认值、端口号、工业界量级、
  手算的派生量都不是 —— 书里对此有说明（见卷首语「配套代码与里面的数字」）。
- 脚本引的上游源码**按版本核对过**（行号会随版本漂），版本表见各卷卷首语 §一。
- 有些实验要 WSL/Linux（`ch07_*.sh`，卷 7 的启动链与抖动实测），Windows 下会跳过。

## 五、目录

| 是什么 | 例子 |
|---|---|
| 数值实验（每章一套） | `lesson23_sitl.py`、`ch09_clocks.py` |
| 反向注入（验证断言真的在守） | `inject_l38.py` … `inject_l52.py` |
| Linux 实验（需 WSL） | `ch07_boot.sh`、`ch07_jitter.sh` |
| 绘图脚本 → `*.png` | `figures_ch6.py` |
| 数据文件（脚本的独立基准） | `sim_pdef_arducopter471.txt` |
| 上游源码位置解析 | `_srcdirs.py` |
