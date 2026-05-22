"""
Pure Pursuit コントローラー (ROS2デプロイ用)
f1tenth-rl-project/src/controllers/pure_pursuit.py から移植・スタンドアロン化
"""
import numpy as np
from .racing_line import RacingLine


class PurePursuitController:
    def __init__(self, racing_line: RacingLine,
                 wheelbase: float = 0.3255,
                 lookahead_dist: float = 0.8):
        self.racing_line = racing_line
        self.wheelbase = wheelbase
        self.lookahead_dist = lookahead_dist

    def get_base_action(self, x: float, y: float, yaw: float,
                        current_speed: float,
                        max_speed: float, min_speed: float):
        """
        Pure Pursuit によるベース制御量を返す。
        Returns:
            steer_rad: ステアリング角 [rad]
            speed: 目標速度 [m/s]
        """
        if self.racing_line is None or not self.racing_line._loaded:
            return 0.0, min_speed

        idx = self.racing_line._find_nearest(x, y)
        N = len(self.racing_line.xy)
        target_idx = idx
        for i in range(1, N):
            check_idx = (idx + i) % N
            dist = np.sqrt((self.racing_line.xy[check_idx, 0] - x) ** 2 +
                           (self.racing_line.xy[check_idx, 1] - y) ** 2)
            if dist >= self.lookahead_dist:
                target_idx = check_idx
                break

        target_wp = self.racing_line.xy[target_idx]
        dx = target_wp[0] - x
        dy = target_wp[1] - y
        local_x = dx * np.cos(-yaw) - dy * np.sin(-yaw)
        local_y = dx * np.sin(-yaw) + dy * np.cos(-yaw)
        L = np.sqrt(local_x ** 2 + local_y ** 2)
        kappa = 2.0 * local_y / (L ** 2) if L > 0 else 0.0
        steer_rad = np.arctan(kappa * self.wheelbase)

        target_curvature = abs(self.racing_line.curvature[target_idx])
        speed = max_speed * np.exp(-1.5 * target_curvature)
        speed = np.clip(speed, min_speed, max_speed)

        return float(steer_rad), float(speed)
