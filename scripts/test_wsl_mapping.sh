#!/bin/bash
# =============================================================================
# test_wsl_mapping.sh
# WSL環境用：Dockerを使用してモックロボットとSLAM、RVizを立ち上げるテストスクリプト
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "WSL用 SLAM Mapping テスト環境を起動します"
echo "=============================================="
echo ""
echo "[INFO] このスクリプトは 'osrf/ros:humble-desktop' コンテナを起動し、"
echo "       依存パッケージのインストールとビルドを内部で実行します。"
echo "       RVizのGUI画面が表示されるまで数分かかる場合があります。"
echo ""

# X11 フォワーディングの確認
if [ -z "$DISPLAY" ]; then
    echo "[WARN] DISPLAY環境変数が設定されていません。"
    echo "       RVizが表示されない可能性があります。WSL2のGUI機能(WSLg)を使用するか"
    echo "       VcXsrv等でXサーバーを立ち上げてください。"
fi

# Docker コンテナ内で実行するスクリプト文字列
DOCKER_CMD=$(cat << 'EOF'
#!/bin/bash
set -e

echo "[Docker] 依存パッケージをインストール中..."
apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-nav2-map-server \
    python3-colcon-common-extensions \
    xterm

echo "[Docker] f1tenth_mapping パッケージをビルド中..."
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select f1tenth_mapping

echo "[Docker] 準備完了！ Launchファイルを起動します..."
source /workspace/ros2_ws/install/setup.bash

# teleop_twist_keyboardを別画面(xterm等)ではなく、このターミナルでインタラクティブに実行
ros2 launch f1tenth_mapping test_mock_mapping.launch.py
EOF
)

# Dockerコマンドの実行
#  - コンテナ内でプロジェクトルートを /workspace としてマウント
#  - ネットワークモードはホスト
#  - /tmp/.X11-unix をマウントしてGUIを表示(RViz)
docker run -it --rm --net=host \
    -v "$PROJECT_ROOT":/workspace \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --name f1-mapping-test \
    osrf/ros:humble-desktop bash -c "$DOCKER_CMD"
