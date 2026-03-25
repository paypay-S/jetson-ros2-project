import pytest
import numpy as np
import os
import sys

# テスト実行時にモジュールが見つかるようにパスを通す
# colcon test 時は自動で入るが、単体実行用
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from f1tenth_rl.utils import LidarProcessor

def test_lidar_processor_basic():
    """基本的な処理（加工なし）のテスト"""
    num_beams = 10
    processor = LidarProcessor(num_beams=num_beams, downsample_step=1, center_crop=False, noise_std=0.0)
    ranges = [1.0] * num_beams
    processed = processor.process(ranges, 10.0)
    assert len(processed) == num_beams
    assert np.all(processed == 1.0)

def test_lidar_processor_downsample():
    """ダウンサンプリングのテスト"""
    processor = LidarProcessor(num_beams=5, downsample_step=2, center_crop=False, noise_std=0.0)
    ranges = [float(i) for i in range(10)]
    processed = processor.process(ranges, 10.0)
    assert len(processed) == 5
    # インデックス 0, 2, 4, 6, 8 が選ばれることを期待
    assert np.all(processed == [0.0, 2.0, 4.0, 6.0, 8.0])

def test_lidar_processor_crop():
    """中心クロップのテスト"""
    # 20点から中心10点を取り出す (step=1)
    processor = LidarProcessor(num_beams=10, downsample_step=1, center_crop=True, noise_std=0.0)
    ranges = [float(i) for i in range(20)]
    # required_raw = 10. start_idx = (20 - 10) // 2 = 5. 5番目から14番目まで。
    processed = processor.process(ranges, 20.0)
    assert len(processed) == 10
    assert processed[0] == 5.0
    assert processed[-1] == 14.0

def test_lidar_processor_padding():
    """点数が足りない場合のパディングテスト"""
    processor = LidarProcessor(num_beams=10, downsample_step=1, center_crop=False, noise_std=0.0)
    ranges = [1.0] * 5
    processed = processor.process(ranges, 10.0) # range_max=10.0
    assert len(processed) == 10
    assert np.all(processed[:5] == 1.0)
    assert np.all(processed[5:] == 10.0) # 残りは range_max で埋まる

def test_lidar_processor_noise():
    """ノイズ付与のテスト"""
    processor = LidarProcessor(num_beams=100, downsample_step=1, center_crop=False, noise_std=0.1)
    ranges = [1.0] * 100
    processed = processor.process(ranges, 10.0)
    # ノイズが入っているので、元の値と完全一致はしないはず
    assert not np.all(processed == 1.0)
    # ただし極端な値（負や range_max 超え）にはならないはず
    assert np.all(processed >= 0.0)
    assert np.all(processed <= 10.0)
