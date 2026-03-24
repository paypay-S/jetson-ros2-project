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

### 安全レイヤーの検証
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

## 🛠️ トラブルシューティング

- **[DRY-RUN] と表示される場合**: I2C の権限が不足しているか、ライブラリのパスが通っていません。`sudo chmod 666 /dev/i2c-1` を試すか、`setup_jetson.sh` を再実行してください。
- **緊急停止が頻発する場合**: 前方 6cm 程度に LiDAR のノイズがある可能性があります。`rl_driver.py` 内の crop 範囲を確認するか、`safety_stop_dist` を調整してください。
