# F1TENTH RL Project

このプロジェクトは、F1TENTH車両を強化学習(RL)モデル（Stable Baselines3 PPO）を用いて自律走行させるためのROS 2ワークスペースです。

## 特徴
- **可変次元LiDAR入力対応**: モデルの入力次元（108次元など）に合わせて、LiDARデータを動的にダウンサンプリング/パディング可能。
- **実機対応ハードウェアブリッジ**: `/drive` 指令を PCA9685 経由でステアリングサーボとESCに変換。
- **Sim-to-Real 対策**: LiDARへのノイズ追加や、アクション（ステアリング）の平滑化 (EMA) を実装済み。
- **安全設計**: 起動時のESCアーム待機処理、終了時の緊急停止、およびライブラリ未対応時のDRY-RUNモードを搭載。

---

## ディレクトリ構成

```text
f1tenth-project/
├── ros2_ws/
│   ├── models/
│   │   └── model.zip          # 学習済みPPOモデル
│   ├── src/
│   │   └── f1tenth_rl/
│   │       ├── f1tenth_rl/
│   │       │   ├── rl_driver.py       # LiDAR+Odom → PPO推論 → /drive
│   │       │   └── hardware_bridge.py # /drive → PCA9685 (Servo/ESC)
│   │       └── launch/
│   │           └── f1tenth_rl.launch.py  # 全ノード起動設定
│   └── scripts/tests/         # 各種テスト用スクリプト
└── jetson-ros2/               # Python仮想環境 (venv)
```

---

## 準備

### 1. 仮想環境の有効化
ライブラリのインポートエラーを防ぐため、常にこの環境で作業してください。
```bash
source ~/f1tenth-project/jetson-ros2/bin/activate
```

### 2. ビルド
仮想環境がアクティブな状態でビルドを行います。
```bash
cd ~/f1tenth-project/ros2_ws
rm -rf build/ install/ log/  # 初回や環境変更時はクリーン推奨
colcon build --packages-select f1tenth_rl
source install/setup.bash
```

### 3. ハードウェア通信チェック (任意)
PCA9685と正しく通信できているか確認できます。
```bash
python3 src/debug_i2c.py
```

---

## 実行方法

### 推奨：Launchファイルで一括起動
実機のLiDARモデルに合わせてパラメータを指定して起動します。

```bash
# 例：モデルが108次元LiDAR入力を想定している場合
ros2 launch f1tenth_rl f1tenth_rl.launch.py \
    lidar_num_beams:=108 \
    lidar_downsample_step:=10 \
    steer_smoothing:=0.5
```

### 主要な起動パラメータ

#### rl_driver (AI推論)
| パラメータ | デフォルト | 説明 |
|---|---|---|
| `model_path` | `.../models/model` | 学習済みモデルのパス（.zipなし） |
| `lidar_num_beams` | 108 | モデルに入力するLiDARの次元数 |
| `lidar_downsample_step` | 10 | `/scan` トピックの間引き間隔 |
| `steer_smoothing` | 0.5 | ステアリングのEMA平滑化係数 (0.0=変化なし, 1.0=即時反映) |

#### hardware_bridge (実機制御)
| パラメータ | デフォルト | 説明 |
|---|---|---|
| `steer_center` | 4700 | 中央位置の duty_cycle |
| `steer_left` | 3700 | 最大左の duty_cycle |
| `steer_right` | 5700 | 最大右の duty_cycle |
| `fixed_speed_mode` | True | Trueで一定速度で走行。FalseでAIの出力を反映 |
| `fixed_esc_duty` | 5800 | 前進時の一定速度 duty_cycle |
| `esc_arm_duration` | 3.0 | 起動時のESCロック解除待機時間(秒) |

---

## 開発とテスト
- **DRY-RUNモード**: 実機（PCA9685等）が接続されていない、またはライブラリが不足している場合、自動的に `[DRY-RUN]` ログを出力する安全モードで動作します。
- **テストスクリプト**: `src/f1tenth_rl/scripts/tests/` 内に各機能を単体テストするためのスクリプトを用意しています。
