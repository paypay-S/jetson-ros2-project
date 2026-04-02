# f1tenth_mapping — F1TENTH 実機用マップ作成ツール

## 概要

このパッケージは、F1TENTH 実機のLiDARを使ってSLAMマップを生成するための ROS 2 パッケージです。
詳細な「マップ作成の手順」や「シミュレーションへの同期方法」については、プロジェクトルートの [README.md](../../../README.md) を参照してください。

---

## 技術仕様

### 構成ノード
- **async_slam_toolbox_node**: SLAM 実行本体。
- **teleop_twist_keyboard**: 手動走行用（SSH経由で操作可能）。
- **rosbridge_server**: Foxglove Studio 等の外部ツールから接続するための WebSocket サーバー。

### パラメータ設定 (`config/mapper_params.yaml`)
主に `slam_toolbox` の動作を制御します。
- `resolution`: マップの解像度 (デフォルト: 0.075 m/pixel)
- `max_laser_range`: LiDAR の有効距離
- `mode`: `async` (Jetson の負荷を抑えるために非同期モードを採用)

### 購読トピック
- `/scan` (`sensor_msgs/LaserScan`): LiDAR データ
- `/tf`: 座標変換

### 配信トピック
- `/map` (`nav2_msgs/OccupancyGrid`): 生成中の地図
- `/map_metadata`: 地図のメタ情報

---

## 開発者向け情報

### ビルド方法
```bash
cd ~/projects/jetson-ros2-project/ros2_ws
colcon build --packages-select f1tenth_mapping
```

### 関連リンク
- [slam_toolbox 公式](https://github.com/SteveMacenski/slam_toolbox)
- [Foxglove Studio](https://studio.foxglove.dev)
