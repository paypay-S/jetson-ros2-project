#!/bin/bash
# =============================================================================
# start_mapping.sh
# F1TENTH マッピングセッションを開始するスクリプト
#
# 使い方:
#   ./scripts/start_mapping.sh
#
# SSH 越しに Jetson で実行することを想定。
# Ctrl+C でマッピングを終了後、save_and_sync.sh でマップを保存する。
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$PROJECT_ROOT/ros2_ws"
VENV_ACTIVATE="$PROJECT_ROOT/jetson-ros2/bin/activate"

# ─── カラー出力 ──────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ─── beep 関数（音声フィードバック）──────────────────────────────────────────
beep_start() {
    # 起動通知: 2回短音
    if command -v beep &>/dev/null; then
        beep -f 880 -l 200 -r 2 -d 100 2>/dev/null || true
    else
        # beep コマンドがない場合は speaker-test で代替
        for i in 1 2; do
            speaker-test -t sine -f 880 -l 1 &>/dev/null || true
            sleep 0.2
        done
    fi
}

beep_error() {
    # エラー通知: 3回短音
    if command -v beep &>/dev/null; then
        beep -f 440 -l 100 -r 3 -d 50 2>/dev/null || true
    fi
}

# ─── 環境チェック ────────────────────────────────────────────────────────────
if [ ! -f "$VENV_ACTIVATE" ]; then
    warn "仮想環境が見つかりません: $VENV_ACTIVATE"
    warn "ROS 2 のシステムインストールを使用します"
else
    info "仮想環境をアクティベート中..."
    source "$VENV_ACTIVATE"
fi

SETUP_BASH="$ROS2_WS/install/setup.bash"
if [ ! -f "$SETUP_BASH" ]; then
    error "ROS 2 ワークスペースがビルドされていません: $SETUP_BASH"
    error "先に以下を実行してください:"
    error "  cd $ROS2_WS && colcon build --packages-select f1tenth_mapping"
    beep_error
    exit 1
fi

info "ROS 2 ワークスペースをセットアップ中..."
source "$SETUP_BASH"

# ─── パッケージ確認 ──────────────────────────────────────────────────────────
if ! ros2 pkg list 2>/dev/null | grep -q "f1tenth_mapping"; then
    error "f1tenth_mapping パッケージが見つかりません"
    error "colcon build を再実行してください"
    beep_error
    exit 1
fi

# ─── 起動 ────────────────────────────────────────────────────────────────────
echo ""
echo "=============================================="
info "F1TENTH マッピングセッションを開始します"
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

beep_start
info "Launch ファイルを起動中..."
ros2 launch f1tenth_mapping mapping.launch.py
