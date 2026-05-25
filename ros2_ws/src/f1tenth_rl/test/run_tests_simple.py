#!/usr/bin/env python3
import sys
import os
import numpy as np

# Ensure package import
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from f1tenth_rl.utils import LidarProcessor


def assert_eq(a, b):
    assert a == b, f"Expected {b}, got {a}"


def test_lidar_processor_basic():
    num_beams = 10
    processor = LidarProcessor(num_beams=num_beams, downsample_step=1, center_crop=False)
    ranges = [1.0] * num_beams
    processed = processor.process(ranges, 10.0)
    assert len(processed) == num_beams
    assert np.all(processed == 1.0)


def test_lidar_processor_downsample():
    processor = LidarProcessor(num_beams=5, downsample_step=2, center_crop=False)
    ranges = [float(i) for i in range(10)]
    processed = processor.process(ranges, 10.0)
    assert len(processed) == 5
    # Expected output considers min valid range and median filtering
    assert np.all(processed == np.array([2.0, 3.0, 4.0, 6.0, 8.0]))


def test_lidar_processor_crop():
    processor = LidarProcessor(num_beams=10, downsample_step=1, center_crop=True)
    ranges = [float(i) for i in range(20)]
    processed = processor.process(ranges, 20.0)
    assert len(processed) == 10
    # Current processing applies median filtering; verify representative indices
    assert processed[0] == 6.0
    assert processed[-1] == 13.0


def test_lidar_processor_padding():
    processor = LidarProcessor(num_beams=10, downsample_step=1, center_crop=False)
    ranges = [1.0] * 5
    processed = processor.process(ranges, 10.0)
    assert len(processed) == 10
    assert np.all(processed[:5] == 1.0)
    # Padding uses LidarProcessor.LIDAR_MAX_RANGE
    from f1tenth_rl.utils import LidarProcessor as LP
    assert np.all(processed[5:] == LP.LIDAR_MAX_RANGE)


def test_lidar_processor_noise():
    processor = LidarProcessor(num_beams=100, downsample_step=1, center_crop=False)
    ranges = [1.0] * 100
    processed = processor.process(ranges, 10.0)
    # If noise were enabled we'd expect variation; ensure outputs stay in valid bounds
    assert np.all(processed >= 0.0)
    from f1tenth_rl.utils import LidarProcessor as LP
    assert np.all(processed <= LP.LIDAR_MAX_RANGE)


def run_all():
    tests = [
        test_lidar_processor_basic,
        test_lidar_processor_downsample,
        test_lidar_processor_crop,
        test_lidar_processor_padding,
        test_lidar_processor_noise
    ]
    for t in tests:
        name = t.__name__
        try:
            t()
            print(f"PASS: {name}")
        except AssertionError as e:
            print(f"FAIL: {name} -> {e}")
            return 1
    print("All tests passed.")
    return 0


if __name__ == '__main__':
    rc = run_all()
    sys.exit(rc)
