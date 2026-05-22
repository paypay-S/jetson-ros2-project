"""
レーシングライン（中心線ウェイポイント）管理モジュール (ROS2デプロイ用)
f1tenth-rl-project/src/racing_line.py から移植・スタンドアロン化
"""
import numpy as np
import csv
import os


class RacingLine:
    NUM_FEATURES = 4  # [cte, heading_err, curvature, progress]

    @property
    def num_waypoints(self):
        return len(self.xy)

    def __init__(self, csv_path: str, lookahead: int = 12):
        self.csv_path = csv_path
        self.lookahead = lookahead
        self._loaded = False
        self.xy = np.zeros((0, 2), dtype=np.float32)
        self.heading = np.zeros(0, dtype=np.float32)
        self.curvature = np.zeros(0, dtype=np.float32)
        self._last_idx = 0
        self._load(csv_path)

    def _load(self, csv_path: str):
        if not os.path.exists(csv_path):
            print(f"[RacingLine] CSV が見つかりません: {csv_path}")
            print(f"[RacingLine] フォールバック: 全特徴量=0 で動作します。")
            return
        xs, ys, hs, ks = [], [], [], []
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))
                hs.append(float(row["heading"]))
                ks.append(float(row["curvature"]))
        self.xy = np.column_stack([xs, ys]).astype(np.float32)
        self.heading = np.array(hs, dtype=np.float32)
        self.curvature = np.clip(np.array(ks, dtype=np.float32), -10.0, 10.0)
        self._loaded = True
        self._last_idx = 0
        print(f"[RacingLine] ロード完了: {csv_path} ({len(self.xy)} ウェイポイント)")

    def _find_nearest(self, x: float, y: float) -> int:
        if not self._loaded:
            return 0
        N = len(self.xy)
        half = 50
        lo = max(0, self._last_idx - half)
        hi = min(N, self._last_idx + half + 1)
        diff = self.xy[lo:hi] - np.array([x, y], dtype=np.float32)
        local_idx = int(np.argmin(np.sum(diff ** 2, axis=1)))
        idx = lo + local_idx
        if local_idx < 5 or local_idx > (hi - lo - 5):
            diff_all = self.xy - np.array([x, y], dtype=np.float32)
            idx = int(np.argmin(np.sum(diff_all ** 2, axis=1)))
        self._last_idx = idx
        return idx

    def get_nearest_index(self, x: float, y: float) -> int:
        return self._find_nearest(x, y)

    def get_features(self, x: float, y: float, yaw: float,
                     max_cte: float = 2.0, max_curv: float = 10.0) -> np.ndarray:
        if not self._loaded:
            return np.zeros(self.NUM_FEATURES, dtype=np.float32)
        idx = self._find_nearest(x, y)
        N = len(self.xy)
        wp = self.xy[idx]
        wp_head = self.heading[idx]
        dx = x - wp[0]
        dy = y - wp[1]
        cte = -dx * np.sin(wp_head) + dy * np.cos(wp_head)
        cte_norm = float(np.clip(cte / max_cte, -1.0, 1.0))
        head_err = yaw - wp_head
        head_err = (head_err + np.pi) % (2 * np.pi) - np.pi
        head_err_norm = float(np.clip(head_err / np.pi, -1.0, 1.0))
        hi_k = min(N, idx + self.lookahead)
        mean_curv = float(np.mean(self.curvature[idx:hi_k]))
        curv_norm = float(np.clip(mean_curv / max_curv, -1.0, 1.0))
        progress = idx / float(N - 1) if N > 1 else 0.0
        return np.array([cte_norm, head_err_norm, curv_norm, progress], dtype=np.float32)

    def reset(self):
        self._last_idx = 0
