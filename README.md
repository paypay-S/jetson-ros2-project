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

## 🔄 実機開発サイクル

本プロジェクトは、以下の 4 ステップのサイクルで運用することを想定しています。

```mermaid
graph LR
    A[<b>1. Mapping</b><br/>実機で地図作成] --> B[<b>2. Train</b><br/>シミュレータ学習]
    B --> C[<b>3. Verify</b><br/>Bagデータ検証]
    C --> D[<b>4. Deploy</b><br/>実機で自律走行]
    D --> A
```

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

## 🧭 マッピング（コースの地図作成）

実機 LiDAR で SLAM を行い、その結果を学習用マップとしてエクスポートします。

### Step 1: マッピングの開始
SSH で Jetson に接続し、以下を実行します。
```bash
cd ~/projects/jetson-ros2-project
./scripts/start_mapping.sh
```
`beep × 2` が鳴れば起動完了です。ターミナルに手動操作用のキーボード操作画面が表示されます。

### Step 2: Foxglove Studio による可視化（推奨）
一切画面を繋がなくても、ブラウザから地図作成状況をリアルタイムで確認できます。
1. `https://studio.foxglove.dev` へアクセスし、`Open Connection` を開く。
2. 『Rosbridge』を選択し、`ws://<JetsonのIPアドレス>:9090` を入力して接続。
3. パネルから `Map` (`/map`) や `LaserScan` (`/scan`) を追加。

### Step 3: 手動走行によるスキャン
キーボードの `u i o` 等で車体を操作し、コースをゆっくり 1〜2 周します。
> [!TIP]
> **ループクロージャ**: コースを 1 周してスタート地点に戻ると、スキャンマッチングが働き、地図の歪みが自動で補正されます。

### Step 4: マップの保存と同期
**マッピングを起動したまま**、別の SSH ターミナルから保存スクリプトを実行します。
```bash
./scripts/save_map.sh <マップ名>
```
`beep 長音 1 回` が鳴れば、`maps/` への保存と [f1tenth-rl-project](file:///home/yuta775/projects/f1tenth-rl-project) への自動コピーが完了します。
完了後、元のターミナルで `Ctrl + C` を押して終了してください。

---

## 🏎️ 自律走行 (Inference)

学習済みモデル（`.onnx`）を使って実機を走行させます。

### 1. 正規化パラメータの設定
学習時の統計量を `params.yaml` に反映します（エディタで直接編集するか、補助スクリプトを使用）。
```bash
python3 scripts/calibration.py
```

### 2. 走行開始
```bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py \
    model_path:=models/ppo_my_model.onnx \
    fixed_speed_mode:=True \
    fixed_esc_duty:=5600
```
※ `fixed_esc_duty` を上げることで最高速度を調整できます（最初は 5400〜5600 程度を推奨）。

### 3. WSL2 での視覚化検証 (RViz2)
過去の走行データを再生しながら、AI の判断を 3D で確認できます。
```bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py rviz:=True
```
※ 詳細は [WSL2_TEST_GUIDE.md](file:///home/yuta775/projects/jetson-ros2-project/docs/wsl2_test_guide.md) を参照。

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
- **save_map.sh でマップ保存が失敗する場合**: `start_mapping.sh` が起動中のまま別ターミナルから実行してください（`/map` トピックが必要です）。
- **slam_toolbox が起動しない**: 
    - `ros2 topic echo /scan` で雷探器（LiDAR）のデータが届いているか確認してください。
    - **Ubuntu 20.04 をお使いの場合**: `ros-humble-*` はインストールできません。代わりに `ros-foxy-*` をインストールしてください。
      ```bash
      sudo apt install ros-foxy-slam-toolbox ros-foxy-nav2-map-server
      ```
