import numpy as np
import scipy.ndimage
from typing import List, Tuple


class LidarProcessor:
    """
    LiDARデータの前処理クラス（学習環境 f1_env.py と同一の処理を再現）

    処理パイプライン:
      1. NaN/inf クリーニング
      2. 中心クロップ (前方 num_beams*downsample_step 点を抽出)
      3. スパイクノイズ除去 (メディアンフィルタ)
      4. min プーリングダウンサンプリング
      ※ 正規化 (0-1反転) は rl_driver.py 側で実施

    Returns: raw downsampled 距離 [m] (216点、正規化前)
    """
    LIDAR_MAX_RANGE = 30.0

    def __init__(self,
                 num_beams: int = 216,
                 downsample_step: int = 5,
                 center_crop: bool = True,
                 median_filter_size: int = 5):
        self.num_beams = num_beams
        self.downsample_step = downsample_step
        self.center_crop = center_crop
        self.median_filter_size = median_filter_size
        self.min_valid_range = 0.15


    def process(self, ranges: List[float], range_max: float) -> np.ndarray:
        """
        生LiDARデータ → raw ダウンサンプリング済み距離 [m] (正規化なし)
        安全レイヤー・extra特徴量・racing_line の計算はこの値を使う。
        """
        lidar = np.nan_to_num(
            np.array(ranges, dtype=np.float32),
            nan=self.LIDAR_MAX_RANGE,
            posinf=self.LIDAR_MAX_RANGE,
            neginf=0.0
        )
        lidar = np.clip(lidar, 0.0, self.LIDAR_MAX_RANGE)
        # 車体反射ノイズを除去
        lidar[lidar < self.min_valid_range] = self.LIDAR_MAX_RANGE

        # 中心クロップ
        if self.center_crop:
            required = self.num_beams * self.downsample_step
            n = len(lidar)
            if n > required:
                start = (n - required) // 2
                lidar = lidar[start: start + required]

        # メディアンフィルタ (スパイクノイズ除去)
        if self.median_filter_size > 1:
            lidar = scipy.ndimage.median_filter(lidar, size=self.median_filter_size)

        # min プーリングダウンサンプリング (f1_env.py と同じ)
        if self.downsample_step > 1:
            n = len(lidar)
            trunc = (n // self.downsample_step) * self.downsample_step
            if trunc > 0:
                lidar = lidar[:trunc].reshape(-1, self.downsample_step).min(axis=1)

        # サイズ調整
        if len(lidar) >= self.num_beams:
            lidar = lidar[:self.num_beams]
        else:
            pad = self.num_beams - len(lidar)
            lidar = np.pad(lidar, (0, pad), 'constant',
                           constant_values=(self.LIDAR_MAX_RANGE,))

        return lidar  # [m]、正規化なし

    @staticmethod
    def normalize(lidar_raw: np.ndarray,
                  max_range: float = 30.0) -> np.ndarray:
        """
        0-1反転正規化: 近い壁=1.0, 遠い空間=0.0 (f1_env.py と同一)
        """
        return 1.0 - np.clip(lidar_raw, 0.0, max_range) / max_range

    @staticmethod
    def compute_extra_features(lidar_raw: np.ndarray,
                               max_range: float = 30.0) -> np.ndarray:
        """
        extra特徴量 [front_feat, min_feat, lr_asymmetry] を計算 (f1_env.py と同一)
        - lidar_raw: process() の出力 (216点, [m])
        """
        # front_feat: 前方±40° (インデックス 76:140)
        front_vals = lidar_raw[76:140]
        front_raw = float(np.min(front_vals)) if len(front_vals) > 0 else max_range
        front_feat = 1.0 - np.clip(front_raw, 0.0, max_range) / max_range

        # min_feat: 全体の最小距離
        min_raw = float(np.min(lidar_raw))
        min_feat = 1.0 - np.clip(min_raw, 0.0, max_range) / max_range

        # lr_asymmetry: 左前方 vs 右前方 (インデックス 124:156 vs 60:92)
        left_diag = np.min(lidar_raw[124:156])
        right_diag = np.min(lidar_raw[60:92])
        lr_asymmetry = float((left_diag - right_diag) / max_range)

        return np.array([front_feat, min_feat, lr_asymmetry], dtype=np.float32)
