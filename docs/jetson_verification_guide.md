# 実機検証（Jetson）用ガイド

本ドキュメントは、`development` ブランチ作成以降に作成・更新された環境構築スクリプトや検証用スクリプトを用い、Jetson 実機での動作を検証するための手順をチームメンバーに共有するためのものです。

---

## 1. 環境構築の検証 (Environment Setup)

WSL上で作成された最新の `setup_jetson.sh` を使用して、Jetson 上に仮想環境と依存関係を構築します。

### 実行コマンド
```bash
# プロジェクトのルートディレクトリで実行
chmod +x setup_jetson.sh
./setup_jetson.sh
```

### 検証ポイント
- `python3-venv`, `i2c-tools` などのシステムパッケージが正しくインストールされるか
- `jetson-ros2` 仮想環境が作成され、`requirements-essential.txt` 内のライブラリ（Stable Baselines3等）がインストールされるか
- `colcon build` が成功し、ROS 2 ワークスペースが正常にビルドされるか
- 実行ユーザーが `i2c` グループに追加されているか（反映には再ログインが必要な場合があります）

---

## 2. ハードウェア接続の検証 (Hardware Verification)

PCA9685（PWMコントローラ）を介したステアリングとスロットルの制御を検証します。

### I2C 通信確認
```bash
i2cdetect -y -r 1
```
- `0x40` (PCA9685) が表示されていることを確認します。

### PCA9685 単体テスト
仮想環境をアクティベートしてから実行してください。
```bash
source jetson-ros2/bin/activate

# ステアリング（Channel 0）のテスト
python3 hardware_tests/test_pca9685_ch0.py

# ESC/スロットル（Channel 1）のテスト
python3 hardware_tests/test_pca9685_ch1.py
```

---

## 3. キャリブレーションと手動制御の検証

ステアリングのセンター位置や ESC のニュートラル位置を調整するためのスクリプトを検証します。

### ステアリング・ESC キャリブレーションツールの実行
```bash
python3 scripts/calibrate_steering.py
```
- `w/s`: 前進・後進
- `a/d`: 左右ステアリング
- `Space`: 停止・センター
- `q`: 終了

---

## 4. ROS 2 ワークスペースのビルドと起動検証

ビルド済みのパッケージが実機のハードウェアブリッジと連携できるか検証します。

### 起動手順
```bash
source jetson-ros2/bin/activate
source ros2_ws/install/setup.bash

# ハードウェアブリッジを含む RL 起動
ros2 launch f1tenth_rl f1tenth_rl.launch.py
```

### 固定速度モード (Fixed Speed Mode) の検証
`development` ブランチでは、安定した走行のために「固定速度モード」が導入されています。
- **デフォルト設定**: `fixed_speed_mode: true` / `fixed_esc_duty: 5800`
- **動作**: `speed_threshold` (0.05) を超える速度入力があった場合、モデルの出力値に関わらず一定の出力(`5800`)で走行します。
- **検証方法**:
  - `ros2 launch` 時に期待通りの速度で一定に走行するか確認してください。
  - 速度を変更したい場合は `ros2_ws/src/f1tenth_rl/config/params.yaml` を編集するか、起動時にパラメータを上書きしてください。

### 検証ポイント
- `/scan` トピックから LiDAR データが取得できているか
- `hardware_bridge` ノードがエラーなく起動し、PCA9685 へのコマンド送信が開始されるか
- 固定速度モードが有効な場合、微小な速度指令でも一定速度で前進が開始されるか

---

## 5. 統合検証用スクリプト (Integration Tests)

`scripts/tests/` 以下のスクリプトを使用して、サブシステムの動作を検証します。

### 仮想環境とモデルの読み込みテスト
```bash
python3 scripts/tests/test-cnviroment.py
```

### ROS 2 連携予測テスト
```bash
# LiDARトピックの入力をシミュレートし、モデルの予測（Action）が出力されるか確認
python3 scripts/tests/test-ros2.py
```

### セーフティチェックの検証
```bash
# 緊急停止や障害物検知のロジックをスタンドアロンで検証
python3 scripts/tests/test_safety.py
```

---

## 備考
- 本ガイドに記載されているパスは、プロジェクトのルートディレクトリを基準としています。
- ステアリングの可動範囲や ESC の値（`params.yaml`）は、`calibrate_steering.py` で得られた値を反映させてください。
