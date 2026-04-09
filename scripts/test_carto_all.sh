#!/bin/bash
# =============================================================================
# test_carto_all.sh
# 動作テスト用：全システムを Cartographer モードで一括起動
# =============================================================================

set -e

PROJECT_ROOT="/home/toyonishiorin/projects/f1tenth-project"
cd "$PROJECT_ROOT"

info() { echo -e "\033[0;32m[INFO]\033[0m $*"; }

echo "=============================================="
info "動作テスト: 統合 Cartographer モード"
echo "=============================================="

# unified_start.sh に slam:=cartographer 引数を渡して起動
./scripts/unified_start.sh slam:=cartographer
