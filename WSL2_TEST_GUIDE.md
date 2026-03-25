# WSL2 での検証ガイド (ROS 2 Bag 再生)

WSL2 環境では実機の LiDAR やモーターにアクセスできませんが、過去に記録したセンサーデータ（ROS 2 Bag）を再生することで、AI の推論ロジックや安全停止機能が正しく動作するかをテストできます。

## 1. 準備

### 仮想環境の有効化
```bash
source jetson-ros2/bin/activate
source ros2_ws/install/setup.bash
```

### モデルファイルの配置
`ros2_ws/models/model.zip` が存在することを確認してください。

## 2. 検証ノードの起動

ターミナル 1 で `rl_driver` を起動します：
```bash
# 実際のハードウェアがないため、エラーや警告が出ますが、ロジック自体は動きます
ros2 launch f1tenth_rl f1tenth_rl.launch.py
```

## 3. ROS 2 Bag の再生

ターミナル 2 で、手元にある Bag ファイルを再生します：
```bash
# bag_directory は .mcap ファイルが含まれるディレクトリ
ros2 bag play <bag_directory>
```

### Bag ファイルがない場合
ダミーの `LaserScan` トピックを発行して、緊急停止が動くかテストできます：
```bash
# 前方 0.2m に障害物がある想定のデータを 1回発行 (停止距離 0.3m 設定の場合)
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'laser'}, range_min: 0.1, range_max: 10.0, ranges: [0.2]*1080}" -1
```

## 4. 動作の確認

### ログの確認
`rl_driver` のターミナルに以下のようなログが表示されれば成功です：
- `Loaded RL model: ...` (モデル読み込み成功)
- `EMERGENCY STOP! Obj at 0.20m` (安全レイヤーの動作確認)

### 出力の確認
別のターミナルで、AI が出力する操縦指令を確認します：
```bash
ros2 topic echo /drive
```

## 5. まとめ
この手順でロジックを確認してから実機にデプロイすることで、「現場でのデバッグ」時間を大幅に短縮できます。
