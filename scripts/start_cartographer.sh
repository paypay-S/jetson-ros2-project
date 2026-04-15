#!/bin/bash
# =============================================================================
# start_cartographer.sh (Safety Fix)
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

# 0. 徹底的な掃除 (自分自身 "start_cartographer.sh" を殺さないように注意)
info "Performing deep cleanup of previous processes..."
pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f occupancy_grid_node 2>/dev/null || true
pkill -9 -f foxglove_bridge 2>/dev/null || true
pkill -9 -f rosbridge_server 2>/dev/null || true
pkill -9 -f rosbridge_websocket 2>/dev/null || true
pkill -9 -f hardware_bridge 2>/dev/null || true
pkill -9 -f real_bridge 2>/dev/null || true
pkill -9 -f urg_node2 2>/dev/null || true

# 共有メモリの掃除 (パスワードを聞かれたら入力してください)
sudo rm -rf /dev/shm/fastrtps* || true
sleep 2

# 1. 仮想環境・ROSセットアップ
if [ -f "$VENV_ACTIVATE" ]; then
    source "$VENV_ACTIVATE"
fi
source /opt/ros/humble/setup.bash

# 2. ワークスペースのソース
[ -f "$SLAM_WS/install/setup.bash" ] && source "$SLAM_WS/install/setup.bash"
[ -f "$MAIN_WS/install/setup.bash" ] && source "$MAIN_WS/install/setup.bash"

# 終了時のクリーンアップ
cleanup() {
    echo ""
    info "Shutting down all F1TENTH nodes..."
    pkill -9 -P $$ 2>/dev/null || true
    pkill -9 -f urg_node2 2>/dev/null || true
    pkill -9 -f hardware_bridge 2>/dev/null || true
    pkill -9 -f real_bridge 2>/dev/null || true
    pkill -9 -f cartographer_node 2>/dev/null || true
    pkill -9 -f foxglove_bridge 2>/dev/null || true
    info "Cleanup complete."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo ""
echo "=============================================="
info "F1TENTH 統合 Cartographer システムを開始します"
echo "=============================================="

# 4. 全システムの起動
ros2 launch f1tenth_rl bringup.launch.py slam:=cartographer &
LAUNCH_PID=$!

# 5. キーボード操作ノードをフォアグラウンドで実行
sleep 7
info "Starting teleop keyboard control..."
ros2 run teleop_twist_keyboard teleop_twist_keyboard || true

# 6. テレオプ終了後に全体を止める
cleanup
