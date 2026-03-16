# F1TENTH RL Project

このプロジェクトは、F1TENTH車両を強化学習(RL)モデル（Stable Baselines3 PPO）を用いて自律走行させるためのROS2ワークスペースを含んでいます。

## ディレクトリ構成

- `ros2_ws/`: ROS2ワークスペース
  - `src/f1tenth_rl/`: 強化学習エージェントを実行するメインのROS2パッケージ。
    - `rl_driver.py`: LiDARとOdometryデータをSubscribeし、学習済みモデルを用いて推論を行い、AckermannDriveStampedメッセージをPublishして車両を制御します。
  - `scripts/`: テストスクリプトやモデルファイルなどが配置されています。モデルファイル（`model.zip`）はデフォルトでここに置く想定です。
- `jetson-ros2/`: (無視設定済み) Pythonの仮想環境など。

## 前提条件

- ROS2 (Foxy / Humble など)
- Python 3
- 必要なPythonパッケージ:
  - `rclpy`
  - `numpy`
  - `stable_baselines3`
- 必要なROS2パッケージ:
  - `sensor_msgs`
  - `nav_msgs`
  - `ackermann_msgs`

## インストールとビルド

1. ワークスペースのビルド

```bash
cd ~/f1tenth-project/ros2_ws
colcon build --packages-select f1tenth_rl
```

2. 環境変数のセットアップ

```bash
source install/setup.bash
```

## 実行方法

モデルファイル(`model.zip`)が `/home/toyonishiorin/f1tenth-project/ros2_ws/scripts/model.zip` に存在することを確認してください。

以下のコマンドでRLドライバノードを起動します。

```bash
ros2 run f1tenth_rl rl_driver
```

### パラメータの変更

モデルファイルのパスが異なる場合は、実行時にパラメータを上書きできます。

```bash
ros2 run f1tenth_rl rl_driver --ros-args -p model_path:=/別の/パス/model.zip
```

## トピック

- **Subscribes:**
  - `/scan` (`sensor_msgs/LaserScan`): LiDARの距離データ (1080データにクロップ/パディングされます)
  - `/odom` (`nav_msgs/Odometry`): 車両の現在速度とステアリング状態の取得
- **Publishes:**
  - `/drive` (`ackermann_msgs/AckermannDriveStamped`): 車両への速度・ステアリング制御指令
