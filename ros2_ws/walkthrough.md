# 実機走行 実装 ウォークスルー

## 変更・追加ファイル一覧

| ファイル | 変更種別 | 内容 |
|---|---|---|
| [hardware_bridge.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/f1tenth_rl/hardware_bridge.py) | NEW | `/drive` → PCA9685 変換ノード |
| [f1tenth_rl.launch.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/launch/f1tenth_rl.launch.py) | NEW | 2ノード同時起動launchファイル |
| [setup.py](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/setup.py) | MODIFY | hardware_bridgeエントリポイント・launchデータ追加 |
| [package.xml](file:///home/toyonishiorin/f1tenth-project/ros2_ws/src/f1tenth_rl/package.xml) | MODIFY | ros2launch・launch_ros依存追加 |
| [README.md](file:///home/toyonishiorin/f1tenth-project/README.md) | MODIFY | 実機走行手順・パラメータ表に全面更新 |

## ビルド確認結果

```
$ colcon build --packages-select f1tenth_rl
Starting >>> f1tenth_rl
Finished <<< f1tenth_rl [OK]
```

```
$ ros2 pkg executables f1tenth_rl
f1tenth_rl hardware_bridge   ✅
f1tenth_rl rl_driver         ✅
```

## 実機走行の起動コマンド

```bash
cd ~/f1tenth-project/ros2_ws
source install/setup.bash
ros2 launch f1tenth_rl f1tenth_rl.launch.py
```

## hardware_bridge の重要な設計ポイント

- **ESCアーム処理**: 起動時にSTOP値を3秒間送信してESCを初期化
- **DRY-RUNモード**: `adafruit_pca9685`が未インストールの環境では自動的にログ出力のみのモードで動作
- **安全シャットダウン**: Ctrl+C時にステアリングをセンターに戻しESCをSTOPにしてからdeinit
- **全パラメータがROS2パラメータで変更可能**: launchファイルまたはコマンドライン引数で調整可

## 次のステップ（実機での調整）

1. `steer_max_angle`（モデルのステアリング出力スケール）の調整
2. ESCのduty_cycle値をVESC/実機にあわせて微調整
3. 必要に応じて`fixed_speed_mode: False`にして速度制御モードに移行
