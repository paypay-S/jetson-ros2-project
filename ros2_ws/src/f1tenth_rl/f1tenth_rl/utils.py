import numpy as np

class LidarProcessor:
    """LiDARデータのダウンサンプリング、中心クロップ、ノイズ付与、正規化を行うクラス"""
    def __init__(self, num_beams=108, downsample_step=10, center_crop=True, noise_std=0.0):
        self.num_beams = num_beams
        self.downsample_step = downsample_step
        self.center_crop = center_crop
        self.noise_std = noise_std

    def process(self, ranges, range_max):
        # NaN / inf を除去
        lidar = np.nan_to_num(
            np.array(ranges, dtype=np.float32),
            nan=range_max,
            posinf=range_max,
            neginf=0.0
        )

        # 中心クロップ (前方中心を基準に取り出す)
        if self.center_crop:
            n = len(lidar)
            required_raw = self.num_beams * self.downsample_step
            if n > required_raw:
                start_idx = (n - required_raw) // 2
                lidar = lidar[start_idx : start_idx + required_raw]

        # 間引き処理
        if self.downsample_step > 1:
            lidar = lidar[::self.downsample_step]

        # モデルの入力次元に合わせる
        if len(lidar) >= self.num_beams:
            lidar = lidar[:self.num_beams]
        else:
            padding_size = self.num_beams - len(lidar)
            lidar = np.pad(lidar, (0, padding_size), 'constant', constant_values=(range_max,))

        # ノイズ追加 (Sim-to-Real)
        if self.noise_std > 0:
            noise = np.random.normal(0, self.noise_std, size=lidar.shape).astype(np.float32)
            lidar = np.clip(lidar + noise, 0.0, range_max)

        return lidar
