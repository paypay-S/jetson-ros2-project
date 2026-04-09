#!/bin/bash
# =============================================================================
# start_cartographer.sh
# F1TENTH の Cartographer版 マッピングセッションを開始するスクリプト
# =============================================================================

set -e

# パス設定
PROJECT_ROOT="/home/toyonishiorin/projects/f1tenth-project"
MAIN_WS="$PROJECT_ROOT/ros2_ws"
VENV_ACTIVATE="$PROJECT_ROOT/jetson-ros2/bin/activate"

GREEN='\033[0;32m'
NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC} $*"; }

# 1. 仮想環境のアクティベート
if [ -f "$VENV_ACTIVATE" ]; then
    info "仮想環境をアクティベート中..."
    source "$VENV_ACTIVATE"
fi

# 2. ワークスペースのソース
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$MAIN_WS/install/setup.bash" ]; then
    info "ROS 2 ワークスペースをセットアップ中..."
    source "$MAIN_WS/install/setup.bash"
else
    echo "エラー: $MAIN_WS/install/setup.bash が見つかりません。colcon build を実行してください。"
    exit 1
fi

echo ""
echo "=============================================="
info "F1TENTH カルトグラファー マッピング を開始します"
echo "----------------------------------------------"
echo "  操作方法:"
echo "  teleop_twist_keyboard:"
echo "    u i o  / 前左・前・前右"
echo "    j k l  / 左回転・停止・右回転"
echo "    m , .  / 後左・後・後右"
echo "    q/z : 速度アップ/ダウン"
echo ""
echo "  マッピング終了: Ctrl+C"
echo "  マップ保存:     (起動中に別ターミナルから) ./scripts/save_map.sh <マップ名>"
echo "=============================================="
echo ""

# 終了時のクリーンアップ処理
cleanup() {
    echo ""
    info "終了します。バックグラウンドプロセスを停止中..."
    pkill -9 -P $$ 2>/dev/null || true
    pkill -9 -f cartographer_node 2>/dev/null || true
    pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
    pkill -9 -f rosbridge_websocket 2>/dev/null || true
    exit 0
}

# Ctrl+Cでクリーンアップを実行
trap cleanup SIGINT SIGTERM EXIT

# 3. バックグラウンドで Cartographer Launch ファイルを起動
info "バックグラウンドで Cartographer（SLAM・可視化）を起動中..."
ros2 launch f1tenth_mapping cartographer.launch.py &
LAUNCH_PID=$!

# 少し待つ
sleep 3

# 4. フォアグラウンドで teleop_twist_keyboard を起動
info "手動操作用ノード (teleop_twist_keyboard) をフォアグラウンドで開始します"
ros2 run teleop_twist_keyboard teleop_twist_keyboard

wait $LAUNCH_PID
