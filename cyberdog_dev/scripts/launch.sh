#!/bin/bash
# cyberdog_dev 启动脚本：感知节点 + odom 广播 + 主程序。
# 用法：bash scripts/launch.sh sim 1
#       bash scripts/launch.sh real 1-6

set -e
MODE="${1:-sim}"
STAGES="${2:-1-6}"
DEV_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEV_DIR"

python3 - <<'PY'
import sys

missing = []
for module in ("toml", "lcm"):
    try:
        __import__(module)
    except ModuleNotFoundError:
        missing.append(module)

if missing:
    print("缺少 Python 依赖: " + ", ".join(missing), file=sys.stderr)
    print("请在容器内运行: python3 -m pip install -r requirements.txt", file=sys.stderr)
    sys.exit(1)
PY

cleanup() {
    echo "停止所有后台进程..."
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

echo "[1/3] 启动感知节点..."
for node in \
    orange_ball \
    football \
    red_pole \
    block_obstacle \
    lane_edge \
    dashed_line \
    coke
do
    python3 -m perception "$node" --mode "$MODE" &
done
python3 -m perception.lidar_corridor --mode "$MODE" &
python3 -m perception.slope --mode "$MODE" &

echo "[2/3] 启动 odom -> TF 广播..."
python3 -m core.localization.odom_broadcaster --mode "$MODE" &

sleep 2

echo "[3/3] 启动主程序 (mode=$MODE stages=$STAGES)..."
python3 main.py --mode "$MODE" --stages "$STAGES"

cleanup
