# 🏎️ F1TENTH Jetson ROS 2 Project

![ROS 2](https://img.shields.io/badge/ros2-humble-blue?logo=ros)
![Python](https://img.shields.io/badge/python-3.10-blue?logo=python)
[![ROS 2 CI](https://github.com/paypay-S/jetson-ros2-project/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/paypay-S/jetson-ros2-project/actions/workflows/ros2_ci.yml)

F1TENTH 車両を強化学習（Stable Baselines3 / PPO）を用いて Jetson 上で自律走行させるための ROS 2 システムです。
**WSL2 での開発・検証**から、**Jetson 実機でのデプロイ**までを一貫してサポートするように最適化されています。

---

## 🛠️ プロジェクトの構成と役割

このリポジトリは、以下の 2 つの環境で役割を分担して使用します。

| 環境 | 主な役割 | 実行パッケージ |
| :--- | :--- | :--- |
| **WSL2 (PC)** | 開発、コード修正、AI推論のBag検証、RVizによる視覚化 | `f1tenth_rl` (検証用) |
| **Jetson (実機)** | マッピング（SLAM）、実機走行、PWMハードウェア制御 | `f1tenth_mapping`, `f1tenth_rl` |

---

## 🔄 開発ワークフロー

1.  **Mapping**: Jetson を手動操縦してコースの地図を作成。
2.  **Sync**: `save_and_sync.sh` で地図を RL プロジェクトへ同期。
3.  **Train**: [f1tenth-rl-project](file:///home/yuta775/projects/f1tenth-rl-project) で AI モデルを学習。
4.  **Verify (WSL2)**: 学習済みモデルを [WSL2_TEST_GUIDE.md](file:///home/yuta775/projects/jetson-ros2-project/docs/wsl2_test_guide.md) の手順で擬似検証。
5.  **Deploy (Jetson)**: `setup_jetson.sh` で環境を整え、実機走行を開始。

---

## 🚀 セットアップ

### 1. 依存関係のインストール
```bash
# WSL2 / Jetson 共通
pip install -r requirements-essential.txt
```

### 2. ビルド
```bash
source /opt/ros/humble/setup.bash  # ROS 2 環境のロード
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🕹️ 走行・操作方法

### 実機での自律走行 (Jetson)
```bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py \
    model_path:=models/ppo_10M_exp14_gradual_speedup.onnx \
    fixed_speed_mode:=True \
    fixed_esc_duty:=5600
```
※ PyTorch (`.zip`) と ONNX (`.onnx`) の両方に対応していますが、推論速度の観点から ONNX の使用を推奨します。

### WSL2 での視覚化検証 (RViz2)
過去の走行データを再生しながら、AI の判断を 3D で確認できます。
```bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py rviz:=True
```
※ 詳細は [WSL2_TEST_GUIDE.md](file:///home/yuta775/projects/jetson-ros2-project/docs/wsl2_test_guide.md) を参照。

---

## 🧭 マッピングと同期

Jetson 実機でコースの地図を作成し、RL プロジェクトに同期する手順です。

### 1. マップ作成の開始
**ターミナル 1 (Jetson SSH):**
```bash
cd ~/projects/jetson-ros2-project
./scripts/start_mapping.sh
# → beep×2音で起動完了。操作用の teleop_twist_keyboard 画面が表示されます。
```
キーボードで車体を操作して、コースを一周（またはそれ以上）走ります：
- `u i o`: 前左・前進・前右
- `j k l`: 左回転・停止・右回転
- `q / z`: 速度アップ / ダウン

### 2. マップの保存と同期
**ターミナル 2 (別の SSH ウィンドウ):**
※ `start_mapping.sh` を**起動したまま**実行してください。
```bash
cd ~/projects/jetson-ros2-project
./scripts/save_and_sync.sh <マップ名>

# 例: 
./scripts/save_and_sync.sh circuit_warehouse
# → 長音beep1回で完了。f1tenth-rl-project/my_maps/ に自動コピーされます。
```
同期が完了したら、ターミナル 1 で `Ctrl+C` を押してマッピングを終了します。

### 3. 新マップでのトレーニング
`f1tenth-rl-project/src/config.py` の `MAP_PATH` を更新してトレーニングを開始します。
```python
MAP_PATH = os.environ.get("MAP_PATH", "/workspace/my_maps/circuit_warehouse")
```

---

## 🧪 テスト

### ユニットテスト (ロジック検証)
```bash
pytest ros2_ws/src/f1tenth_rl/test/test_lidar_processor.py
```

## ✨ 主な特徴

- **環境に依存しない構成**: WSL2 と Jetson の両方で動作。
- **パラメータ管理**: `ros2_ws/src/f1tenth_rl/config/params.yaml` で機体設定や AI パラメータを一括管理。
- **Sim-to-Real 最適化**: 指数移動平均 (EMA) やスルーレート制限により、実機の振動や急激な負荷を抑制。
- **WSL2 検証**: 実機がなくても Bag データを用いた AI 推論の 3D 視覚化が可能。

---

## 🛠️ トラブルシューティング

- **[DRY-RUN] と表示される**: 
    - I2C の権限不足 → `sudo chmod 666 /dev/i2c-1`
    - ライブラリ不足 → `setup_jetson.sh` を再実行。
- **緊急停止 (EMERGENCY STOP) が頻発する**: 
    - LiDAR の前方に配線などのノイズがある可能性があります。`params.yaml` の `safety_stop_dist` を調整するか、`rl_driver.py` の crop インデックスを確認してください。
- **RViz2 が表示されない (WSL2)**: 
    - WSL2 の GUI 設定を確認してください（Windows 11 以上推奨）。
- **save_and_sync.sh で保存に失敗する**: 
    - `start_mapping.sh` が別ウィンドウで動作しているか確認してください（`/map` トピックが必要です）。
- **slam_toolbox が起動しない**: 
    - `ros2 topic echo /scan` で雷探器（LiDAR）のデータが届いているか確認してください。
    - **Ubuntu 20.04 をお使いの場合**: `ros-humble-*` はインストールできません。代わりに `ros-foxy-*` をインストールしてください。
      ```bash
      sudo apt install ros-foxy-slam-toolbox ros-foxy-nav2-map-server
      ```
