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
4.  **Verify (WSL2)**: 学習済みモデルを `WSL2_TEST_GUIDE.md` の手順で擬似検証。
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
    fixed_speed_mode:=True \
    fixed_esc_duty:=5600
```

### WSL2 での視覚化検証 (RViz2)
過去の走行データを再生しながら、AI の判断を 3D で確認できます。
```bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py rviz:=True
```
※ 詳細は [WSL2_TEST_GUIDE.md](file:///home/yuta775/projects/jetson-ros2-project/WSL2_TEST_GUIDE.md) を参照。

---

## 🧭 マッピングと同期

1. マッピング開始: `./scripts/start_mapping.sh`
2. 地図の保存と同期: `./scripts/save_and_sync.sh <map_name>`

---

## 🧪 テスト

### ユニットテスト (ロジック検証)
```bash
pytest ros2_ws/src/f1tenth_rl/test/test_lidar_processor.py
```

## ✨ 主な特徴

- **環境に依存しない構成**: WSL2 と Jetson の両方で、ホームディレクトリの絶対パスを意識せずに動作します。
- **パラメータ管理の外部化**: 機体設定や AI パラメータを `params.yaml` で一括管理。コードの変更なしで調整が可能です。
- **強固な検証体制**: `pytest` によるユニットテストと、GitHub Actions による自動ビルド・テスト環境を完備。
- **WSL2 検証ファースト**: 実際のマシンがなくても、Bag データを用いた AI 推論の 3D 視覚化検証が可能です。

---

## 🛠️ トラブルシューティング

- **[DRY-RUN] 表示**: I2C アクセス権限またはライブラリ不足。`setup_jetson.sh` を再実行。
- **緊急停止の頻発**: `config/params.yaml` の `safety_stop_dist` を調整。
- **RViz2 が表示されない**: WSL2 の GUI 設定を確認してください（Windows 11 以上推奨）。

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
