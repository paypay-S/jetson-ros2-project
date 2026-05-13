import numpy as np
import scipy.ndimage
from typing import List

class LidarProcessor:
    """
    LiDARデータのダウンサンプリング、中心クロップ、ノイズ付与、フィルタリングを行うクラス。
    
    Attributes:
        num_beams (int): 出力するビーム数。
        downsample_step (int): ダウンサンプリングのステップ。
        center_crop (bool): 前方中央を抽出するかどうか。
        noise_std (float): Sim-to-Real検証用の疑似ノイズの標準偏差。
        median_filter_size (int): スパイクノイズ除去用のメディアンフィルタサイズ。
        min_valid_range (float): 車体反射などを無視する最小有効距離。
    """
    def __init__(
        self, 
        num_beams: int = 108, 
        downsample_step: int = 10, 
        center_crop: bool = True, 
        noise_std: float = 0.0, 
        median_filter_size: int = 5
    ):
        self.num_beams = num_beams
        self.downsample_step = downsample_step
        self.center_crop = center_crop
        self.noise_std = noise_std
        self.median_filter_size = median_filter_size
        # 実機の車体反射(ノイズ)を無視する最小距離
        self.min_valid_range = 0.05 

    def process(self, ranges: List[float], range_max: float) -> np.ndarray:
        """
        生のLiDARデータを処理してモデル入力用の形式に変換する。

        Args:
            ranges (List[float]): LiDARのスキャンデータ。
            range_max (float): LiDARの最大検知距離。

        Returns:
            np.ndarray: 処理済みのLiDARデータ。
        """
        # NaN / inf を除去
        lidar = np.nan_to_num(
            np.array(ranges, dtype=np.float32),
            nan=range_max,
            posinf=range_max,
            neginf=0.0
        )
        
        # 極端に近いノイズ (車体反射) は最大距離に置き換えて無視する
        lidar[lidar < self.min_valid_range] = range_max

        # 中心クロップ (前方中心を基準に取り出す)
        if self.center_crop:
            n = len(lidar)
            required_raw = self.num_beams * self.downsample_step
            if n > required_raw:
                start_idx = (n - required_raw) // 2
                lidar = lidar[start_idx : start_idx + required_raw]

        # ノイズ付与 (Sim-to-Realのテスト用)
        if self.noise_std > 0:
            noise = np.random.normal(0, self.noise_std, size=lidar.shape).astype(np.float32)
            lidar = np.clip(lidar + noise, 0.0, range_max)

        # 実機のスパイクノイズ対策: メディアンフィルタ
        if self.median_filter_size > 1:
            lidar = scipy.ndimage.median_filter(lidar, size=self.median_filter_size)

        # 間引き処理 (f1_env.py の仕様に合わせて min プーリングを使用)
        if self.downsample_step > 1:
            n = len(lidar)
            # 割り切れる長さに調整
            truncate_len = (n // self.downsample_step) * self.downsample_step
            if truncate_len > 0:
                lidar = lidar[:truncate_len].reshape(-1, self.downsample_step).min(axis=1)

        # モデルの入力次元に合わせる
        if len(lidar) >= self.num_beams:
            lidar = lidar[:self.num_beams]
        else:
            padding_size = self.num_beams - len(lidar)
            lidar = np.pad(lidar, (0, padding_size), 'constant', constant_values=(range_max,))

        return lidar
