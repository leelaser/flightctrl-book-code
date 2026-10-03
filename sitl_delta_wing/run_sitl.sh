#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════
#  run_sitl.sh —— 用自研三角翼气动模型启动 ArduPilot SITL
#
#  用法（在 WSL / Linux 下）：
#      ARDUPILOT=~/ardupilot ./run_sitl.sh
#  或先 export ARDUPILOT，再直接跑。
#
#  ⚠️ 本脚本在 Windows 原生命令行里跑不了 —— ArduPilot SITL 需要 Linux。
#     本机 WSL 2 已就绪（2.4.13.0），只差发行版：wsl --install -d Ubuntu-24.04
#     此脚本【尚未实测】；下面每一处"依据"都标了源码出处，便于你自行核对。
# ══════════════════════════════════════════════════════════════════════════
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT="${ARDUPILOT:-$HOME/ardupilot}"

MODEL_SRC="$HERE/delta_wing.json"
PARM_FILE="$HERE/delta_wing.parm"

# ── 前置检查 ────────────────────────────────────────────────────────────
if [ ! -d "$ARDUPILOT/Tools/autotest" ]; then
    echo "✗ 找不到 ArduPilot 源码树：$ARDUPILOT"
    echo "  请设置：export ARDUPILOT=/path/to/ardupilot"
    exit 1
fi
for f in "$MODEL_SRC" "$PARM_FILE"; do
    [ -f "$f" ] || { echo "✗ 缺少文件：$f"; exit 1; }
done

# ── 路径形式：必须用【相对路径】，绝对路径 SITL 认不了 ──────────────────
#
# 实测（2026-09-29，ArduPlane 4.7.1）三种写法：
#     绝对路径 /home/.../delta_wing.json      ✗ PANIC failed to load
#     @ROMFS/models/delta_wing.json           ✗ PANIC（官方模型却能过，原因未查明）
#     相对路径 delta_wing.json                ✓ 成功
#
# 源码依据 libraries/SITL/SIM_Plane.cpp load_coeffs()：
#     if (AP::FS().stat(model_json, &st) == 0) fname = strdup(model_json);
#     else { asprintf(&fname, "@ROMFS/models/%s", model_json);
#            if (AP::FS().stat(model_json, &st) != 0) AP_HAL::panic(...); }
#   注意 else 分支拼了 ROMFS 前缀、却又去 stat【原路径】—— 所以只有
#   "stat 一次就过" 的路径形式能用，绝对路径不在其中。
#
# 做法：把 JSON 复制到 ArduPilot 工作目录，用相对文件名。
cd "$ARDUPILOT" || exit 1
cp "$MODEL_SRC" ./delta_wing.json
MODEL_REL="delta_wing.json"

# 源码依据：SIM_Plane.cpp 里 strstr(frame_str, "-elevon") / "-catapult"
#   -elevon   : 升降副翼（无平尾，本布局必需）
#   -catapult : 15 m/s² × 2 s 弹射起飞（巡飞弹常见发射方式）
FRAME_STR="plane-elevon-catapult:${MODEL_REL}"

echo "════════════════════════════════════════════════════════════════"
echo " ArduPilot SITL · 自研三角翼"
echo "   ArduPilot  : $ARDUPILOT"
echo "   气动模型   : $ARDUPILOT/$MODEL_REL  (相对路径，见上)"
echo "   参数文件   : $PARM_FILE"
echo "   frame 串   : $FRAME_STR"
echo "════════════════════════════════════════════════════════════════"
echo

# ⚠️ 不要 cd 到 Tools/autotest —— SITL 二进制的 cwd 决定相对路径的解析基准，
#    必须停在 ArduPilot 根目录，否则找不到 delta_wing.json。
#
# 源码依据：Tools/autotest/pysim/vehicleinfo.py:559
#     if opts.model is not None: ret["model"] = opts.model
#   => 命令行 --model 会覆盖 frame 自带的模型串，所以自定义 JSON 能生效。
exec python3 Tools/autotest/sim_vehicle.py \
    -v ArduPlane \
    -f plane-elevon \
    --model "$FRAME_STR" \
    --add-param-file "$PARM_FILE" \
    --console --map \
    "$@"
