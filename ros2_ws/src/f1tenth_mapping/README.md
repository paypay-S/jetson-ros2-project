# f1tenth_mapping — F1TENTH 実機用マップ作成ツール

## 概要

このパッケージは、F1TENTH 実機のLiDARを使ってSLAMマップを生成し、強化学習プロジェクト（`f1tenth-rl-project`）のシミュレーション環境へ自動同期するためのツールです。

---

## 背景と目的

### なぜこのツールが必要か

F1TENTH 自律走行システムは以下の2段階で構成されています：

1. **シミュレーション学習** (`f1tenth-rl-project`) — `f110_gym` 上で強化学習 (PPO) モデルを訓練する
2. **実機走行** (`jetson-ros2-project`) — 学習済みモデルを Jetson 上で動かし実機を制御する

シミュレーションで学習したモデルを実機で再現させるためには、**「シミュレーターで使うマップ」と「実機が実際に走る環境」を一致させること**が重要です。

既存のマップ（`my_map.pgm`）は手製のもので、実環境と完全には一致していませんでした。本ツールは **実機 LiDAR でSLAMを行い、そのまま学習用マップとして使える**ワークフローを実現します。

### 設計上の意思決定

| 決定事項 | 選択 | 理由 |
|---|---|---|
| マップ生成方式 | SLAM (`slam_toolbox`) | 実機LiDARデータから実環境を精確に反映できる。CAD図面やグリッドマップでは壁の反射・歪みを再現できない |
| SLAMエンジン | `slam_toolbox` (async) | ROS 2 Humble で最も安定。Jetson の処理能力に合わせて非同期モードを採用 |
| 走行方式 | 手動操作 (teleop_twist_keyboard) | ゆっくり丁寧に走れるため、スキャンマッチング精度が上がる。AI走行では急な挙動で地図がズレるリスクがある |
| 解像度 | `0.075 m/pixel` | 既存の `my_map.yaml` と統一。シミュレーションと実機の座標スケールを一致させる |
| UI設計 | ノーディスプレイ対応 (SSH + beep音) | Jetson はディスプレイなしでAI走行を行う想定。RViz2を使わず、操作状態はbeep音でフィードバック |
| マップ同期 | 自動コピースクリプト | 生成→学習のワークフローを摩擦なく繋ぎ、ファイルの手動コピーミスを防ぐ |

---

## システム構成

```
Jetson（AI走行・マッピング兼用）
│
├─ [ターミナル1] ./scripts/start_mapping.sh
│       ↓
│   slam_toolbox (async_slam_toolbox_node)
│       ← /scan (LiDAR) を購読してマップを構築
│   teleop_twist_keyboard
│       ← キーボード入力を /cmd_vel に変換
│
├─ [ターミナル2] オプション: マップ状態確認
│       ros2 topic echo /map_metadata
│
└─ Ctrl+C 後
   ./scripts/save_and_sync.sh <マップ名>
       ↓
   nav2 map_saver_cli で PGM/YAML 保存
       ↓
   f1tenth-rl-project/my_maps/ へ自動コピー
```

### ファイル構成

```
ros2_ws/src/f1tenth_mapping/
├── README.md                        # このファイル
├── package.xml                      # ROS 2 パッケージ定義
├── setup.py / setup.cfg             # Pythonビルド設定
├── resource/f1tenth_mapping         # ament_index マーカー
├── f1tenth_mapping/
│   └── __init__.py
├── config/
│   └── mapper_params.yaml           # slam_toolbox パラメータ
└── launch/
    └── mapping.launch.py            # 一括起動ファイル

scripts/  （プロジェクトルート直下）
├── start_mapping.sh                 # マッピング開始スクリプト
└── save_and_sync.sh                 # マップ保存・同期スクリプト
```

---

## 導入手順

### 1. 依存パッケージのインストール（Jetson上で1度だけ）

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
```

### 2. パッケージのビルド

```bash
cd ~/projects/jetson-ros2-project/ros2_ws
colcon build --packages-select f1tenth_mapping
source install/setup.bash
```

---

## 使用方法

### マッピングの流れ

**Step 1: マッピング開始**

SSH で Jetson に接続し、以下を実行します。

```bash
cd ~/projects/jetson-ros2-project
./scripts/start_mapping.sh
```

`beep × 2` が鳴れば起動完了です。`teleop_twist_keyboard` の操作画面が表示されます。

**Step 2: キーボードで手動走行**

`teleop_twist_keyboard` の操作キー：

```
u  i  o    ← 前左 / 前進 / 前右
j  k  l    ← 左回転 / 停止 / 右回転
m  ,  .    ← 後左 / 後退 / 後右
q / z      ← 速度アップ / ダウン
```

> **ポイント**: ゆっくり・滑らかに走ると地図の精度が上がります。特にコーナーは慎重に。コース1周すると**ループクロージャ**が働き、地図の歪みが自動補正されます。

**Step 3: マップ状態の確認（任意）**

別ターミナルから確認できます。

```bash
ros2 topic echo /map_metadata
# → width, height, resolution が表示されれば地図が生成されています
```

**Step 4: マップ保存 & 同期**

コースを十分に走ったら `Ctrl+C` でマッピングを終了し、保存スクリプトを実行します。

```bash
./scripts/save_and_sync.sh <マップ名>

# 例:
./scripts/save_and_sync.sh circuit_warehouse
```

`beep 長音 1 回`が鳴れば完了です。以下のファイルが生成・コピーされます：

- `maps/circuit_warehouse.pgm` — Jetson ローカル保存
- `maps/circuit_warehouse.yaml` — Jetson ローカル保存
- `../f1tenth-rl-project/my_maps/circuit_warehouse.pgm` — RL プロジェクトへ自動コピー
- `../f1tenth-rl-project/my_maps/circuit_warehouse.yaml` — RL プロジェクトへ自動コピー

> **注意**: 同名のマップが `my_maps/` に存在する場合は、タイムスタンプ付きバックアップが自動作成されます。

---

## 新マップで学習を開始する

`f1tenth-rl-project/src/config.py` の `MAP_PATH` を新マップ名に変更します。

```python
# config.py
MAP_PATH = os.environ.get("MAP_PATH", "/workspace/my_maps/circuit_warehouse")
```

または環境変数で指定：

```bash
MAP_PATH=/workspace/my_maps/circuit_warehouse python3 scripts/train.py
```

---

## トラブルシューティング

| 症状 | 原因 | 対処 |
|---|---|---|
| `start_mapping.sh` でエラー | ビルド未実施 | `colcon build --packages-select f1tenth_mapping` を実行 |
| `slam_toolbox` が起動しない | `/scan` が届いていない | `ros2 topic echo /scan` でLiDARデータを確認 |
| `save_and_sync.sh` が失敗 | `/map` トピックがない | `start_mapping.sh` が起動中の状態で別ターミナルから実行する |
| `f1tenth-rl-project` への同期が失敗 | パスが異なる | `save_and_sync.sh` 内の `RL_MAPS_DIR` を自環境のパスに合わせる |
| beep音が鳴らない | コマンドがない | `sudo apt install beep` または `sudo modprobe pcspkr` を試す |

---

## 今後の拡張案

- **プロポボタン連携**: 受信機の余剰チャンネル信号を Jetson GPIO で読み取り、マップ保存をプロポのスイッチ操作でトリガーする（現状はSSHコマンドで代替）
- **Rviz2 リモート表示**: 開発PC側で `ROS_DOMAIN_ID` を合わせて Rviz2 を起動し、ネットワーク越しにマップ生成状況を確認する

---

## 関連ドキュメント

- [プロジェクト全体計画書](../../ros2_ws/project_plan.md)
- [f1tenth-rl-project README](../../../f1tenth-rl-project/README.md)
- [slam_toolbox 公式ドキュメント](https://github.com/SteveMacenski/slam_toolbox)
- [F1TENTH 公式ドキュメント](https://f1tenth.readthedocs.io/)

