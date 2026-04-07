#!/bin/bash

# =============================================================================
# unified_start.sh
# 全環境（LIDAR, Bridge, SLAM）を1つのターミナルで起動するスクリプト
# =============================================================================

set -e

# パス設定
PROJECT_ROOT="/home/toyonishiorin/projects/f1tenth-project"
SLAM_WS="/home/toyonishiorin/slam_projects/ros2_ws"
MAIN_WS="$PROJECT_ROOT/ros2_ws"
VENV_ACTIVATE="$PROJECT_ROOT/jetson-ros2/bin/activate"

GREEN='\033[0;32m'
NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC} $*"; }

# 0. 既存のプロセスのクリーンアップ
# 前回終了しきれなかったプロセスがあれば掃除します
info "Cleaning up lingering processes..."
pkill -9 -f ros2 2>/dev/null || true
pkill -9 -f urg_node2 2>/dev/null || true
pkill -9 -f hardware_bridge 2>/dev/null || true
pkill -9 -f real_bridge 2>/dev/null || true
sleep 1

# 1. 仮想環境のアクティベート
if [ -f "$VENV_ACTIVATE" ]; then
    info "Activating virtual environment..."
    source "$VENV_ACTIVATE"
fi

# 2. ROS 2 環境のソース (Humble)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# 3. LIDAR ワークスペースのソース
if [ -f "$SLAM_WS/install/setup.bash" ]; then
    info "Sourcing LIDAR workspace ($SLAM_WS)..."
    source "$SLAM_WS/install/setup.bash"
fi

# 4. メインワークスペースのソース
if [ -f "$MAIN_WS/install/setup.bash" ]; then
    info "Sourcing main workspace ($MAIN_WS)..."
    source "$MAIN_WS/install/setup.bash"
fi

# 終了時にこのスクリプトから生成された全プロセスを止める
cleanup() {
    echo ""
    info "Shutting down all F1TENTH nodes..."
    # このスクリプト(PID)を親に持つ全プロセスを強制終了
    pkill -9 -P $$ 2>/dev/null || true
    # 名前指定でも念押し
    pkill -9 -f urg_node2 2>/dev/null || true
    pkill -9 -f hardware_bridge 2>/dev/null || true
    pkill -9 -f async_slam_toolbox 2>/dev/null || true
    pkill -9 -f rosbridge_websocket 2>/dev/null || true
    info "Cleanup complete."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 5. 統合Launchファイルの実行 (バックグラウンド)
info "Starting environment (LiDAR + Bridge + SLAM) in background..."
ros2 launch "$PROJECT_ROOT/ros2_ws/src/f1tenth_rl/launch/bringup.launch.py" &
LAUNCH_PID=$!

# 6. キーボード操作ノードの実行 (フォアグラウンド)
sleep 7
info "Starting teleop. Press CTRL-C to exit everything."
ros2 run teleop_twist_keyboard teleop_twist_keyboard
