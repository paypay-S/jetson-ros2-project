#!/bin/bash
# =============================================================================
# start_cartographer.sh (Reliability Pro Edition)
# =============================================================================

set -e

# パス設定
PROJECT_ROOT="/home/toyonishiorin/projects/f1tenth-project"
SLAM_WS="/home/toyonishiorin/slam_projects/ros2_ws"
MAIN_WS="$PROJECT_ROOT/ros2_ws"
VENV_ACTIVATE="$PROJECT_ROOT/jetson-ros2/bin/activate"
DDS_CONFIG="$PROJECT_ROOT/scripts/fastdds_simple.xml"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info() { echo -e "${GREEN}[INFO]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# 0. 徹底的な掃除
info "Performing deep cleanup..."
pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f occupancy_grid_node 2>/dev/null || true
pkill -9 -f foxglove_bridge 2>/dev/null || true
pkill -9 -f rosbridge_websocket 2>/dev/null || true
pkill -9 -f hardware_bridge 2>/dev/null || true
pkill -9 -f real_bridge 2>/dev/null || true
pkill -9 -f urg_node2 2>/dev/null || true
sudo rm -rf /dev/shm/fastrtps* || true
sleep 1

# 1. 通信設定の適用
if [ -f "$DDS_CONFIG" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_CONFIG"
    info "Applied FastDDS configuration: $DDS_CONFIG"
fi

# 2. 環境セットアップ
if [ -f "$VENV_ACTIVATE" ]; then
    source "$VENV_ACTIVATE"
fi
source /opt/ros/humble/setup.bash
[ -f "$SLAM_WS/install/setup.bash" ] && source "$SLAM_WS/install/setup.bash"
[ -f "$MAIN_WS/install/setup.bash" ] && source "$MAIN_WS/install/setup.bash"

# クリーンアップ関数
cleanup() {
    echo ""
    info "Shutting down all F1TENTH nodes..."
    pkill -9 -P $$ 2>/dev/null || true
    pkill -9 -f urg_node2 2>/dev/null || true
    pkill -9 -f cartographer_node 2>/dev/null || true
    info "Cleanup complete."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo ""
info "Starting F1TENTH Unified System..."

# 3. 全システムの起動 (SLAM: Cartographer)
ros2 launch f1tenth_rl bringup.launch.py slam:=cartographer &
LAUNCH_PID=$!

# 4. LiDAR の状態監視とアクティベート
info "Waiting for LiDAR node to appear..."
for i in {1..15}; do
    if ros2 node list | grep -q "/urg_node2"; then
        info "LiDAR node detected. Attempting to activate..."
        ros2 lifecycle set /urg_node2 configure 2>/dev/null || true
        ros2 lifecycle set /urg_node2 activate 2>/dev/null || true
        break
    fi
    sleep 1
done

# 5. /scan トピックの受信確認
info "Checking for LiDAR data (/scan)..."
SCAN_DETECTED=false
for i in {1..10}; do
    if timeout 2s ros2 topic echo /scan --count 1 >/dev/null 2>&1; then
        info "LiDAR data confirmed! (Topic /scan is active)"
        SCAN_DETECTED=true
        break
    fi
    warn "Waiting for /scan data... (Attempt $i/10)"
done

if [ "$SCAN_DETECTED" = false ]; then
    error "LiDAR is NOT publishing data. Please check physical connection and IP."
fi

# 6. キーボード操作プログラムの開始
info "Starting teleop keyboard control..."
ros2 run teleop_twist_keyboard teleop_twist_keyboard || true

# 終了処理
cleanup
