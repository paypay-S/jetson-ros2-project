# F1TENTH RL シミュレーション→実機デプロイ 実装計画

## 現状分析

既存のコードベースには以下が揃っている：

| ファイル | 役割 | 現状の問題点 |
|---|---|---|
| [rl_driver.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/f1tenth_rl/rl_driver.py) | LiDAR→PPO推論→`/drive`発行 | 安全レイヤー（緊急停止）なし |
| [hardware_bridge.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/f1tenth_rl/hardware_bridge.py) | `/drive`→PCA9685 PWM変換 | ステアリング反転フラグなし |
| [launch/f1tenth_rl.launch.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/launch/f1tenth_rl.launch.py) | 2ノード同時起動 | 問題なし |
| `scripts/tests/` | 3つのテストスクリプト | SiL統合テストなし |

## User Review Required

> [!IMPORTANT]
> **モデルの観測空間の確認が必要**
> 現在の[rl_driver.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/f1tenth_rl/rl_driver.py)は観測ベクトルを `[lidar×1080, speed, steer]` = **1082次元** で構成しています。
> 学習時の観測空間が `1080` (LiDARのみ) か `1082` (LiDAR+速度+ステア) かを確認してください。
> `model.zip`のポリシーで `model.observation_space.shape` を確認する必要があります。

> [!WARNING]
> **ステアリング正負方向の確認**
> [hardware_bridge.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/f1tenth_rl/hardware_bridge.py)の現在の設定:  
> `steering_angle > 0` → 左 (`steer_left=3700`)  
> `steering_angle < 0` → 右 (`steer_right=5700`)  
> これが実機の動作と一致するか、微低速（ESC stopped状態）でサーボ動作を確認してください。

---

## 提案する変更内容

### STEP 1: Jetson環境構築スクリプト

#### [NEW] [setup_jetson.sh](file:///home/toyonishiorin/f1tenth-project/jetson-ros2/setup_jetson.sh)

Jetson (aarch64) 向けの依存関係インストールスクリプト。
- PyTorch (Jetson対応 wheel URL から) のインストール手順
- `stable-baselines3`, `numpy`, `adafruit-circuitpython-pca9685` のインストール
- ROS 2 依存パッケージ (`ackermann_msgs` 等) のインストール確認

---

### STEP 2: rl_driver.py の改善

#### [MODIFY] [rl_driver.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/f1tenth_rl/rl_driver.py)

**変更1: 観測空間の自動検証**
- モデルロード後に `model.observation_space.shape` を確認してログ出力

**変更2: LiDARクロップ方式の改善**
- 現在の先頭カット(`[:1080]`)→**中心クロップ**（前方中心を保持）に変更

**変更3: 安全レイヤー (前方衝突検知)**
- 前方 ±15° 相当のインデックス(約90ビーム)のLiDARが閾値以下なら強制停止
- パラメータ: `safety_stop_dist` (デフォルト: 0.3m), `safety_enable` (デフォルト: True)

---

### STEP 3: hardware_bridge.py への steer_flip 追加

#### [MODIFY] [hardware_bridge.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/f1tenth_rl/hardware_bridge.py)

- `steer_flip` パラメータ(bool, デフォルト: False)をパラメータ宣言に追加
- [drive_callback](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/f1tenth_rl/hardware_bridge.py#122-161) 内でステアリング角を反転するオプション追加

---

### STEP 4: テストスクリプトの追加

#### [NEW] [test_sil_integration.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/scripts/tests/test_sil_integration.py)

f1tenth_gym_ros なしでも実行できる統合テスト。
- ダミーの `/scan` トピックを発行する ROS 2 ノードを起動
- `rl_driver` の出力 `/drive` を購読して値を検証

#### [NEW] [test_safety.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/scripts/tests/test_safety.py)

安全レイヤーのユニットテスト。
- 前方に 0.2m の障害物がある場合に speed=0 が出力されるか
- 十分な距離がある場合に通常の推論が動作するか

#### [NEW] [calibrate_steering.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/scripts/calibrate_steering.py)

キャリブレーション用スクリプト。
- `/drive` に段階的にステアリング角を送信
- オペレータが目視確認しながら方向の正負を判定できるインタラクティブなCLI

---

### STEP 5: ドキュメント更新

#### [MODIFY] [README.md](file:///home/toyonishiorin/f1tenth-project/README.md)

- Jetson環境構築手順 (setup_jetson.sh の使い方) を追加
- 安全レイヤーのパラメータ説明を追加
- SiLテストの実行手順を追加
- キャリブレーション手順の追記

---

## 検証計画

### 自動テスト (SiL環境)

```bash
# 1. モデルロード + 観測空間確認
cd ~/f1tenth-project/ros2_ws
python3 scripts/tests/test.py

# 2. 安全レイヤー ユニットテスト
python3 scripts/tests/test_safety.py

# 3. SiL統合テスト (ROS 2起動が必要)
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/tests/test_sil_integration.py
```

### 手動検証 (実機デプロイ前)

1. **ステアリングキャリブレーション**
   ```bash
   python3 scripts/calibrate_steering.py
   ```
   CLIの指示に従って、左右が正しく動くか目視確認する。

2. **安全レイヤー確認**  
   手で LiDAR の前方を覆いながら `ros2 launch f1tenth_rl f1tenth_rl.launch.py` を起動し、
   `/drive` トピックの speed が 0 になることを `ros2 topic echo /drive` で確認する。

3. **低速走行テスト**  
   障害物のない広い場所で、`esc_forward` を最小値 (5400 程度) に設定して走行させ、
   コースに沿って走れるかを確認する。
