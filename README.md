# F1TENTH RL Project

このプロジェクトは、F1TENTH車両を強化学習(RL)モデル（Stable Baselines3 PPO）を用いて自律走行させるためのROS2ワークスペースを含んでいます。

## ディレクトリ構成

```
f1tenth-project/
├── ros2_ws/
│   ├── models/
│   │   └── model.zip          # 学習済みPPOモデル
│   ├── src/
│   │   └── f1tenth_rl/
│   │       ├── f1tenth_rl/
│   │       │   ├── rl_driver.py       # LiDAR+Odom → PPO推論 → /drive パブリッシュ
│   │       │   └── hardware_bridge.py # /drive → PCA9685 (ステアリング/ESC)
│   │       └── launch/
│   │           └── f1tenth_rl.launch.py  # 2ノード同時起動
│   └── scripts/tests/         # 各種テストスクリプト
└── jetson-ros2/               # Python仮想環境
```

## ノード構成

```
/scan (LaserScan) ──┐
                    ├─▶ rl_driver ──▶ /drive ──▶ hardware_bridge ──▶ PCA9685
/odom (Odometry) ──┘                             ch0: ステアリングサーボ
                                                 ch1: ESC / モーター
```

## 前提条件

- ROS2 (Humble 推奨)
- Python 3
- Pythonパッケージ: `rclpy`, `numpy`, `stable_baselines3`
- ROSパッケージ: `sensor_msgs`, `nav_msgs`, `ackermann_msgs`
- ハードウェアパッケージ（Jetson実機用）: `adafruit-circuitpython-pca9685`

## インストールとビルド

```bash
cd ~/f1tenth-project/ros2_ws
colcon build --packages-select f1tenth_rl
source install/setup.bash
```

## 実機走行

### 1. launchで全ノード起動（推奨）

```bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py
```

モデルパスを変更する場合:
```bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py model_path:=/別の/パス/model
```

### 2. 個別起動

```bash
# ターミナル1: ハードウェアブリッジ（先に起動）
ros2 run f1tenth_rl hardware_bridge

# ターミナル2: RLドライバ
ros2 run f1tenth_rl rl_driver
```

### ハードウェアパラメータ（hardware_bridge）

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `steer_ch` | 0 | ステアリングのPCA9685チャンネル |
| `steer_center` | 4700 | ステアリング中央のduty_cycle |
| `steer_left` | 3700 | 最大左のduty_cycle |
| `steer_right` | 5700 | 最大右のduty_cycle |
| `steer_max_angle` | 0.4 | モデル出力の最大ステア角（rad） |
| `esc_ch` | 1 | ESCのPCA9685チャンネル |
| `esc_stop` | 5200 | 停止時のduty_cycle |
| `esc_forward` | 5800 | 前進時のduty_cycle（固定速度モード） |
| `fixed_speed_mode` | True | Trueで固定速度、Falseでスケール速度 |
| `esc_arm_duration` | 3.0 | 起動時のESCアーム待機秒数 |

パラメータのオーバーライド例:
```bash
ros2 run f1tenth_rl hardware_bridge --ros-args -p steer_center:=4600 -p fixed_esc_duty:=5700
```

## トピック

| トピック | 型 | 方向 | 説明 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe | LiDARデータ（1080点にクロップ/パディング） |
| `/odom` | `nav_msgs/Odometry` | Subscribe | 車両の速度・状態 |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Pub/Sub | 速度・ステアリング指令 |

## テスト（実機なし）

```bash
cd ~/f1tenth-project/ros2_ws/scripts/tests

# モデルロード確認
python3 test.py

# 環境確認
python3 test-cnviroment.py
```

`hardware_bridge`はハードウェア未接続時に自動でDRY-RUNモードに切り替わり、ログ出力のみ行います。
