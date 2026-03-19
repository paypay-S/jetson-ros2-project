# F1TENTH Jetson AI走行 プロジェクト計画書

> **最終ゴール**: 学習済みPPOモデル（stable-baselines3）を使い、Jetson実機でF1TENTH車両をAI自律走行させる。

---

## 1. 前提 (Premise)

| 項目 | 内容 |
|---|---|
| 学習フレームワーク | stable-baselines3 (PPO) + f110_gym |
| モデル入力 | LiDARスキャン 1080本 |
| モデル出力 | ステアリング角・速度の2次元アクション |
| 走行プラットフォーム | F1TENTH標準キット（LiDAR, VESC, Jetson） |
| ターゲットOS | Jetson (aarch64) + Ubuntu + ROS 2 Humble |

---

## 2. ソフトウェアスタック定義 (Definition)

### データフロー

```
LiDAR (/scan: LaserScan)
    │
    ▼
[rl_driver.py]
    ├─ 前処理: LaserScan → 1080点 numpy配列
    ├─ PPO推論: obs → [steering_angle, speed]
    └─ 後処理: アクション → AckermannDriveStamped
    │
    ▼
/drive (AckermannDriveStamped)
    │
    ▼
[hardware_bridge.py]
    ├─ ステアリング: 角度 → PCA9685 PWMデューティ
    └─ 速度: 速度値 → ESC PWMデューティ
    │
    ▼
実機ハードウェア (サーボ / VESC)
```

### 主要コンポーネント

| コンポーネント | ファイル | 役割 |
|---|---|---|
| Inference Node | `rl_driver.py` | LiDARサブスクライブ → PPO推論 → `/drive` 発行 |
| Hardware Bridge | `hardware_bridge.py` | `/drive` → PCA9685 PWM変換 |
| Launch | `f1tenth_rl.launch.py` | 全ノード同時起動 |

---

## 3. 現状の課題 (Current Issues)

> [!IMPORTANT]
> **実装前に確認が必要な重大な問題**

| # | 課題 | 詳細 | リスク |
|---|---|---|---|
| A | **観測空間の次元数** | `rl_driver.py`の観測ベクトルが `[lidar×1080, speed, steer] = 1082次元` だが、学習時が `1080次元`（LiDARのみ）の可能性がある | 推論エラー / 結果が意図しない挙動 |
| B | **ステアリング正負方向** | `hardware_bridge.py`で `steering_angle > 0 → 左(3700)` の設定が実機と一致するか未検証 | 意図と逆方向に操舵 |
| C | **安全レイヤーなし** | 現在の`rl_driver.py`に前方衝突検知・緊急停止機能がない | 実機での衝突事故リスク |
| D | **SiL統合テストなし** | `scripts/tests/` にユニットテストはあるが、ROS 2 トピック経由の統合テストがない | 実機デプロイ前の動作確認不足 |

---

## 4. 実装ステップ (Implementation Steps)

### STEP 1: モデル観測空間の確認【最優先】

**目的**: 観測空間の次元ミスマッチを事前に解消する。

```bash
# Pythonで model.zip の観測空間を確認
cd ~/projects/jetson-ros2-project/ros2_ws
python3 - <<'EOF'
from stable_baselines3 import PPO
model = PPO.load("models/model.zip")
print("observation_space:", model.observation_space)
print("action_space:     ", model.action_space)
EOF
```

**判断基準**:
- 出力が `Box(1080,)` → `rl_driver.py`をLiDARのみの1080次元に修正
- 出力が `Box(1082,)` → 現状の実装のまま（速度・ステア値も渡す）

---

### STEP 2: Jetson環境構築 (`setup_jetson.sh` の作成)

**目的**: aarch64アーキテクチャに対応した依存ライブラリを一括インストールする。

インストールする主なライブラリ:
- PyTorch（Jetson向け専用wheel: NVIDIA公式URLから）
- `stable-baselines3`, `numpy`
- ROS 2 パッケージ: `rclpy`, `ackermann_msgs`, `sensor_msgs`
- ハードウェア制御: `adafruit-circuitpython-pca9685`

**作成ファイル**: `setup_jetson.sh`（プロジェクトルートに配置）

> [!NOTE]
> JetsonのJetPackバージョン（JP5.x / JP6.x）によってPyTorchのインストール方法が異なります。
> 事前に `jetson_release` コマンドでバージョンを確認してください。

---

### STEP 3: `rl_driver.py` の改善

#### 変更1: 観測空間の自動検証ログ
モデルロード後に `model.observation_space.shape` を確認してログ出力。

#### 変更2: LiDARクロップ方式の改善
- **現状**: 先頭カット（`scan_data[:1080]`）
- **改善**: **中心クロップ**（前方中心を保持）

```python
# 改善案（中心クロップ）
n = len(ranges)
center = n // 2
half = 540  # 1080 / 2
cropped = ranges[center - half : center + half]
```

#### 変更3: 安全レイヤー（前方衝突検知）の追加

```python
# 前方 ±15° のビームが閾値 (0.3m) 以下なら強制停止
SAFETY_STOP_DIST = 0.3  # パラメータとして外出し
front_beams = cropped[450:630]  # 中心±90ビーム
if np.min(front_beams) < SAFETY_STOP_DIST:
    # speed=0, steering=0 を発行して即時停止
```

**追加パラメータ**:
| パラメータ | デフォルト | 説明 |
|---|---|---|
| `safety_stop_dist` | `0.3` (m) | 緊急停止する前方距離 |
| `safety_enable` | `True` | 安全レイヤーの有効/無効 |

---

### STEP 4: `hardware_bridge.py` の改善

#### 変更: `steer_flip` パラメータの追加

キャリブレーション結果に応じてステアリングを反転できるフラグを追加。

```python
# パラメータ宣言
self.declare_parameter('steer_flip', False)

# drive_callback内
steer = angle * (-1 if self.get_parameter('steer_flip').value else 1)
```

---

### STEP 5: テストスクリプトの追加

| ファイル | 目的 |
|---|---|
| `scripts/tests/test_sil_integration.py` | ダミー `/scan` を発行 → `/drive` の値を検証するSiL統合テスト |
| `scripts/tests/test_safety.py` | 安全レイヤーのユニットテスト（0.2m障害物でspeed=0になるか） |
| `scripts/calibrate_steering.py` | ステアリング正負をインタラクティブに確認するCLIツール |

---

### STEP 6: f1tenth_gym_ros による SiL テスト

**目的**: 実機に搭載する前に「ROS 2経由の制御」が成立するか検証する。

```bash
# ターミナル1: f1tenth_gym_ros シミュレータ起動
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# ターミナル2: 推論ノード起動
source ~/projects/jetson-ros2-project/ros2_ws/install/setup.bash
ros2 run f1tenth_rl rl_driver

# ターミナル3: 確認
ros2 topic echo /drive          # アクション値を確認
rviz2                            # /scan と車両動作を目視確認
```

**確認ポイント**:
- [ ] `/scan` に値が来ているか
- [ ] `/drive` が適切な周期（>10Hz）で発行されているか
- [ ] モデルが推論エラーなく動いているか
- [ ] 安全レイヤーが正常に動作しているか

---

### STEP 7: 実機デプロイ

#### 7-1. ステアリングキャリブレーション

```bash
python3 scripts/calibrate_steering.py
```

CLIの指示に従い、左右が正しく動くか目視で確認。`steer_flip` の値を決定する。

#### 7-2. 安全レイヤーの動作確認

LiDARの前方を手で覆いながらノードを起動し、`/drive` の `speed` が `0` になることを確認。

```bash
ros2 topic echo /drive  # speed が 0 になることを確認
```

#### 7-3. 低速走行テスト

障害物のない広い場所で `esc_forward` を最小値付近（例: 5400）に設定し、コースに沿って走れるか確認。

```bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py
```

---

## 5. 作業チェックリスト

### Phase 0: 確認・準備
- [ ] `model.zip` の観測空間（1080次元 or 1082次元）を確認
- [ ] JetsonのJetPackバージョンを確認 (`jetson_release`)

### Phase 1: コード改修
- [ ] `rl_driver.py`: LiDARクロップ方式を中心クロップに変更
- [ ] `rl_driver.py`: 安全レイヤーを追加（前方衝突検知）
- [ ] `rl_driver.py`: 観測空間の自動検証ログを追加
- [ ] `hardware_bridge.py`: `steer_flip` パラメータを追加
- [ ] `setup_jetson.sh`: Jetson向け環境構築スクリプトを作成

### Phase 2: テスト作成
- [ ] `test_sil_integration.py` を作成
- [ ] `test_safety.py` を作成
- [ ] `calibrate_steering.py` を作成

### Phase 3: SiL検証（開発PC）
- [ ] f1tenth_gym_ros でシミュレーション走行確認
- [ ] 安全レイヤーのユニットテスト通過
- [ ] SiL統合テスト通過

### Phase 4: Jetsonへのデプロイ
- [ ] Jetsonに環境構築（`setup_jetson.sh` 実行）
- [ ] パッケージをビルド (`colcon build`)
- [ ] ステアリングキャリブレーション実施
- [ ] 安全レイヤー動作確認
- [ ] 低速走行テスト（直線・コーナー）
- [ ] 最終走行テスト

---

## 6. 成功の鍵 (Key Success Factors)

> **シミュレーションから実機への移行成功の鍵は、「LiDAR本数の整合性」と「アクション値の物理変換」の正確さにあります。**

| 要素 | 対策 |
|---|---|
| LiDAR本数の整合性 | 中心クロップで1080点を確実に保証 |
| 観測空間の次元ミスマッチ | STEP 1でmodel.zipを事前確認 |
| ステアリング方向の誤り | カリブレーションスクリプトで事前検証 |
| 実機衝突リスク | 安全レイヤーによる自動緊急停止 |
| 通信・型エラー | SiLテストで事前排除 |

---

## 7. 参考リソース

- [f1tenth_gym_ros リポジトリ](https://github.com/f1tenth/f1tenth_gym_ros) - SiLテスト用シミュレータ
- [NVIDIA JetsonのPyTorchインストール](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/) - aarch64向けwheelの公式手順
- [F1TENTH公式ドキュメント](https://f1tenth.readthedocs.io/) - ハードウェア・ソフトウェアリファレンス
- [stable-baselines3 ドキュメント](https://stable-baselines3.readthedocs.io/) - PPOモデルのロード・推論方法
