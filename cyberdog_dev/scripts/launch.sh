#!/bin/bash
# cyberdog_dev 启动脚本：感知节点 + odom 广播 + 主程序。
# 用法：bash scripts/launch.sh sim 1
#       bash scripts/launch.sh real 1-6

set -e
MODE="${1:-sim}"
STAGES="${2:-1-6}"
DEV_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DEV_DIR"

cleanup() {
    echo "停止所有后台进程..."
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

echo "[1/3] 启动感知节点..."
for node in \
    perception.lidar \
    perception.orange_ball \
    perception.football \
    perception.red_pole \
    perception.block_obstacle \
    perception.lane_edge \
    perception.dashed_line \
    perception.coke
do
    python3 -m "$node" --mode "$MODE" &
done

echo "[2/3] 启动 odom -> TF 广播..."
python3 -m core.odom_broadcaster --mode "$MODE" &

sleep 2

echo "[3/3] 启动主程序 (mode=$MODE stages=$STAGES)..."
python3 main.py --mode "$MODE" --stages "$STAGES"

cleanup
