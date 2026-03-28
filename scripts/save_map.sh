#!/bin/bash
# =============================================================================
# save_map.sh
# マッピング完了後にマップを保存するスクリプト
#
# 使い方:
#   ./scripts/save_map.sh <マップ名>
#
# 例:
#   ./scripts/save_map.sh my_circuit_01
#
# 保存先:
#   ./maps/<マップ名>.pgm / .yaml   ← Jetson ローカル
#
# 【注意】マッピング（start_mapping.sh）が起動中に別ターミナルから実行してください。
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$PROJECT_ROOT/ros2_ws"
VENV_ACTIVATE="$PROJECT_ROOT/jetson-ros2/bin/activate"

# マップ保存先ディレクトリ（jetson-ros2-project 内）
LOCAL_MAPS_DIR="$PROJECT_ROOT/maps"

# ─── カラー出力 ──────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; }
success() { echo -e "${CYAN}[OK]${NC}    $*"; }

# ─── beep 関数 ───────────────────────────────────────────────────────────────
beep_complete() {
    # 完了通知: 長音1回
    if command -v beep &>/dev/null; then
        beep -f 880 -l 800 2>/dev/null || true
    else
        speaker-test -t sine -f 880 -l 1 &>/dev/null || true
    fi
}

beep_error() {
    # エラー通知: 3回短音
    if command -v beep &>/dev/null; then
        beep -f 440 -l 100 -r 3 -d 50 2>/dev/null || true
    fi
}

# ─── 引数チェック ────────────────────────────────────────────────────────────
if [ -z "$1" ]; then
    warn "マップ名が指定されていません。デフォルト名 'my_map' を使用します。"
    MAP_NAME="my_map"
else
    # スペースや特殊文字を除去してファイル名を安全にする
    MAP_NAME=$(echo "$1" | tr ' ' '_' | tr -cd '[:alnum:]_-')
fi

SAVE_PATH="$LOCAL_MAPS_DIR/${MAP_NAME}"

info "マップ名: $MAP_NAME"
info "保存先:   $SAVE_PATH.{pgm,yaml}"

# ─── 環境セットアップ ────────────────────────────────────────────────────────
if [ -f "$VENV_ACTIVATE" ]; then
    source "$VENV_ACTIVATE"
fi

SETUP_BASH="$ROS2_WS/install/setup.bash"
if [ -f "$SETUP_BASH" ]; then
    source "$SETUP_BASH"
fi

# ─── 保存ディレクトリ作成 ───────────────────────────────────────────────────
mkdir -p "$LOCAL_MAPS_DIR"

# ─── マップ保存 ──────────────────────────────────────────────────────────────
echo ""
info "マップを保存中..."

if ! ros2 run nav2_map_server map_saver_cli \
    -f "$SAVE_PATH" \
    --ros-args -p save_map_timeout:=10.0; then
    error "マップの保存に失敗しました。"
    error "slam_toolbox が起動中で /map トピックが発行されているか確認してください:"
    error "  ros2 topic list | grep map"
    beep_error
    exit 1
fi

# ─── ファイル存在確認 ───────────────────────────────────────────────────────
if [ ! -f "${SAVE_PATH}.pgm" ] || [ ! -f "${SAVE_PATH}.yaml" ]; then
    error "保存されたファイルが見つかりません: ${SAVE_PATH}.{pgm,yaml}"
    beep_error
    exit 1
fi

success "マップ保存完了:"
success "  PGM:  ${SAVE_PATH}.pgm  ($(du -h "${SAVE_PATH}.pgm" | cut -f1))"
success "  YAML: ${SAVE_PATH}.yaml"

# ─── 完了 ────────────────────────────────────────────────────────────────────
echo ""
echo "=============================================="
success "✅ マップ '${MAP_NAME}' の保存が完了しました！"
echo ""
echo "  次のステップ:"
echo "  1. 生成された PGM/YAML を f1tenth-rl-project/my_maps/ へコピーしてください"
echo "  2. f1tenth-rl-project/src/config.py の MAP_PATH を更新してください"
echo "=============================================="

beep_complete
