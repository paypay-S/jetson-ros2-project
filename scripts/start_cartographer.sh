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

# セッションフォルダ名（今日の日付と時刻）の生成
export SESSION_ID="session_$(date +'%m%d_%H%M%S')"
info "Current Session: $SESSION_ID"

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
LIDAR_ACTIVE=false
for i in {1..15}; do
    if ros2 node list | grep -q "/urg_node2"; then
        info "LiDAR node detected. Checking lifecycle state..."
        
        CURRENT_STATE=$(ros2 lifecycle get /urg_node2 2>/dev/null || echo "unknown")
        if [[ "$CURRENT_STATE" == "unconfigured" ]]; then
            info "Configuring LiDAR..."
            ros2 lifecycle set /urg_node2 configure >/dev/null 2>&1 || true
            sleep 1
        fi
        
        CURRENT_STATE=$(ros2 lifecycle get /urg_node2 2>/dev/null || echo "unknown")
        if [[ "$CURRENT_STATE" == "inactive" ]]; then
            info "Activating LiDAR..."
            ros2 lifecycle set /urg_node2 activate >/dev/null 2>&1 || true
            sleep 1
        fi

        # 最終確認
        FINAL_STATE=$(ros2 lifecycle get /urg_node2 2>/dev/null || echo "unknown")
        if [[ "$FINAL_STATE" == *"active"* ]]; then
            info "LiDAR is now ACTIVE."
            LIDAR_ACTIVE=true
            break
        fi
    fi
    sleep 1
done

if [ "$LIDAR_ACTIVE" = false ]; then
    warn "LiDAR node did not reach ACTIVE state, but continuing to check topic..."
fi

# 5. /scan トピックの受信確認
info "Checking for LiDAR data (/scan)..."
SCAN_DETECTED=false
for i in {1..15}; do
    # LiDARドライバは通常 Best Effort QoS でパブリッシュするため、明示的に指定して確認する
    # Discovery遅延も考慮し、timeout 3s を維持
    if timeout 3s ros2 topic echo /scan --count 1 --qos-reliability best_effort >/dev/null 2>&1; then
        info "LiDAR data confirmed! (Topic /scan is active)"
        SCAN_DETECTED=true
        break
    fi
    warn "Waiting for /scan data... (Attempt $i/15)"
    sleep 1
done

if [ "$SCAN_DETECTED" = false ]; then
    warn "LiDAR topic check timed out. However, if SLAM is running locally, it may still be receiving data."
    warn "Please check Foxglove Studio to verify if scan data is visualized."
fi


# 6. 高機能テレオプ & マップマネージャーの開始
info "Starting high-performance teleop & map manager..."
python3 "$PROJECT_ROOT/scripts/teleop_map_manager.py" || true

# 終了処理
cleanup
