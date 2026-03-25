# F1TENTH Reinforcement Learning Hardware Integration

このプロジェクトは、F1TENTH 車両を強化学習モデル（SB3 / PPO）を用いて Jetson 実機上で自律走行させるための ROS 2 システムです。
実機特有のハードウェア制御、安全レイヤー、およびキャリブレーション機能が含まれています。

## 🏎️ システム構成

- **rl_driver**: LiDAR / Odometry データを入力とし、AIモデル（PPO）を用いてステアリングと速度を決定します。
- **hardware_bridge**: ROS 2 の `AckermannDrive` 指令を、PCA9685 経由の PWM 信号に変換し、サーボとESCを制御します。
- **Safety Layer**: 前方の障害物を検知すると、AIの推論を待たずに即座に「緊急停止」をかけます。

---

## 🚀 クイックスタート (Jetson)

### 1. 環境構築
まずは Jetson 上で必要なライブラリと仮想環境をセットアップします。

```bash
# プロジェクトルートで実行
chmod +x setup_jetson.sh
./setup_jetson.sh
```

### 2. ビルド
仮想環境を有効にし、ROS 2 ワークスペースをビルドします。

```bash
source jetson-ros2/bin/activate
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. ハードウェアの調整 (非常に重要)
走行前に、ステアリングのセンター位置やESCの動作を確認します。
**※車体を引きずるのを防ぐため、必ず車体を台に乗せてタイヤを浮かせた状態で実行してください。**

```bash
python3 scripts/calibrate_steering.py
```
- `A` / `D` : ステアリング微調整
- `W` / `S` : モーター動作確認
- `Space` : 停止・中央復帰
- ここで得た数値を、後の Launch パラメータ（`steer_bias` など）に反映させます。

---

## 🕹️ 走行・操作方法 (ROS 2 Launch)

### 実機での自律走行
以下のコマンドで、AIノードとハードウェアブリッジを同時に起動します。

```bash
source jetson-ros2/bin/activate
source ros2_ws/install/setup.bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py \
    safety_stop_dist:=0.3 \
    fixed_speed_mode:=True \
    fixed_esc_duty:=5600
```

#### 主要なパラメータ
| パラメータ名 | デフォルト値 | 説明 |
| :--- | :--- | :--- |
| `safety_enable` | `True` | 緊急停止機能を有効にするか |
| `safety_stop_dist` | `0.3` | 緊急停止をかける前方距離 (m) |
| `fixed_speed_mode` | `True` | 一定速度走行モード (初心者におすすめ) |
| `fixed_esc_duty` | `5800` | 前進時のパワー (5200が停止) |
| `steer_flip` | `False` | ステアリングの左右が逆の場合に `True` に設定 |
| `steer_bias` | `0` | ステアリングのセンターオフセット調整 |

---

## 🧪 テストと検証

### 1. ユニットテスト (WSL2 / 実機共通)
ロジック部分（LiDAR 前処理など）は ROS 2 環境なしでもテスト可能です。
```bash
source jetson-ros2/bin/activate
pytest ros2_ws/src/f1tenth_rl/test/test_lidar_processor.py
```

### 2. WSL2 での検証 (ROS 2 Bag 再生)
実機がなくても、過去の走行データを用いて AI の挙動をテストする方法は [WSL2_TEST_GUIDE.md](file:///home/yuta775/projects/jetson-ros2-project/WSL2_TEST_GUIDE.md) を参照してください。

### 3. 安全レイヤーの検証
疑似的に障害物データを流し、システムが正しく「速度 0.0」を出すかを確認します。

```bash
# ターミナル1: AIノード起動
ros2 run f1tenth_rl rl_driver

# ターミナル2: 検証スクリプト
python3 scripts/tests/test_safety.py
```

### SiL (Software-in-the-Loop) 統合テスト
内部パイプラインが正常に繋がっているかを一括チェックします。

```bash
python3 scripts/tests/test_sil_integration.py
```

## 🗺️ マップ作成 (f1tenth_mapping)

実機のLiDARを使ってSLAMでマップを生成し、`f1tenth-rl-project` へ自動同期するツールです。
SSH越しにキーボードで操作することを想定しています（ノーディスプレイ対応）。

### 事前準備

```bash
# slam_toolbox のインストール確認 (Jetson上)
ros2 pkg list | grep slam_toolbox

# インストールされていない場合
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
```

### パッケージのビルド

```bash
cd ~/projects/jetson-ros2-project/ros2_ws
colcon build --packages-select f1tenth_mapping
source install/setup.bash
```

### マッピング手順

**ターミナル1 (SSH): マッピング開始**
```bash
cd ~/projects/jetson-ros2-project
./scripts/start_mapping.sh
# → beep×2音で起動完了。teleop_twist_keyboard の操作画面が表示される。
```

**ターミナル1: キーボードで車体を手動操縦してコースを走る**
```
u i o   ← 前左・前進・前右
j k l   ← 左回転・停止・右回転
q / z   ← 速度アップ / ダウン
```

**ターミナル2 (SSH別ウィンドウ): マップ状態の確認（任意）**
```bash
ros2 topic echo /map_metadata
```

**マッピング完了後: マップ保存 & 同期**
```bash
# ターミナル1 で Ctrl+C してから実行
./scripts/save_and_sync.sh <マップ名>

# 例:
./scripts/save_and_sync.sh circuit_warehouse
# → 長音beep1回で完了。f1tenth-rl-project/my_maps/ に自動コピーされる。
```

### 生成後の使い方

`f1tenth-rl-project/src/config.py` の `MAP_PATH` を更新して新マップでトレーニングを開始：

```python
MAP_PATH = os.environ.get("MAP_PATH", "/workspace/my_maps/circuit_warehouse")
```

---

## 🛠️ トラブルシューティング

- **[DRY-RUN] と表示される場合**: I2C の権限が不足しているか、ライブラリのパスが通っていません。`sudo chmod 666 /dev/i2c-1` を試すか、`setup_jetson.sh` を再実行してください。
- **緊急停止が頻発する場合**: 前方 6cm 程度に LiDAR のノイズがある可能性があります。`rl_driver.py` 内の crop 範囲を確認するか、`safety_stop_dist` を調整してください。
- **slam_toolbox が起動しない場合**: `ros2 topic echo /scan` でLiDARデータが届いているか確認してください。
- **save_and_sync.sh でマップ保存が失敗する場合**: `start_mapping.sh` が起動中のまま別ターミナルから実行してください（`/map` トピックが必要です）。
