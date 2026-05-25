"""
safety.py

安全管理マネージャー。以下を管理：
- 状態遷移: NORMAL → FALLBACK/COLLISION → SAFE_STOP
- フォールバック戦略: モデル失敗時は Pure Pursuit へ
- 段階的降速: フォールバック中も速度を徐々に落とす
- 前方障害物検知: 衝突判定
"""

import time
import numpy as np
from enum import Enum
from typing import Optional, Tuple, Dict

from .model_health import ModelHealthMonitor
from .collision_recovery import CollisionRecoverySequence, RecoveryPhase
from .pure_pursuit import PurePursuitController


class SafetyState(Enum):
    """安全管理の状態"""
    NORMAL = 1              # モデル制御
    FALLBACK_TO_PP = 2      # Pure Pursuit フォールバック
    COLLISION_DETECTED = 3  # 衝突検出
    SAFE_STOP = 7           # 停止状態


class SafetyManager:
    """
    安全管理マネージャー。
    
    状態遷移と段階的降速を管理し、以下のシナリオに対応：
    - モデル推論の失敗・遅延
    - 衝突検出と段階的復帰
    - タイムアウト時の強制停止
    """
    
    def __init__(
        self,
        model_monitor: ModelHealthMonitor,
        pure_pursuit_controller: PurePursuitController,
        collision_recovery: CollisionRecoverySequence,
        fallback_timeout_sec: float = 10.0,
        collision_recovery_timeout_sec: float = 5.0,
        speed_schedule: Optional[list] = None,
        speed_stage_duration_sec: float = 2.0
    ):
        """
        Args:
            model_monitor: 推論健全性モニター
            pure_pursuit_controller: Pure Pursuit コントローラー
            collision_recovery: 衝突復帰シーケンス
            fallback_timeout_sec: フォールバック最大継続時間
            collision_recovery_timeout_sec: 衝突復帰最大実行時間
            speed_schedule: 段階的降速スケジュール [1.0, 0.8, 0.5, 0.2, 0.0]
            speed_stage_duration_sec: 各降速段階の継続時間
        """
        self.state = SafetyState.NORMAL
        self.model_monitor = model_monitor
        self.pp_controller = pure_pursuit_controller
        self.collision_recovery = collision_recovery
        
        # タイムアウト設定
        self.fallback_timeout_sec = fallback_timeout_sec
        self.collision_recovery_timeout_sec = collision_recovery_timeout_sec
        self.fallback_start_time: Optional[float] = None
        self.collision_start_time: Optional[float] = None
        
        # 段階的降速
        self.speed_schedule = speed_schedule or [1.0, 0.8, 0.5, 0.2, 0.0]
        self.speed_stage_duration_sec = speed_stage_duration_sec
        self.speed_stage = 0
        self.last_speed_stage_time: Optional[float] = None
        
        # 衝突検知パラメータ
        self.collision_check_angle_deg = 60.0  # 前方 ±30°
        self.collision_total_fov_deg = 270.0   # LiDAR FOV 270°
        self.collision_stop_dist = 0.3  # 衝突判定距離
        
        # 統計
        self.mode_switch_count = 0
        self.collision_count = 0
    
    def update_state(
        self,
        model_healthy: bool,
        collision: bool,
        x: float,
        y: float,
        yaw: float,
        current_time: Optional[float] = None
    ) -> None:
        """
        安全状態を更新する。
        
        Args:
            model_healthy: モデルが健全か
            collision: 衝突が検出されたか
            x, y, yaw: 現在位置・姿勢
            current_time: 現在時刻（秒、省略時は time.time()）
        """
        if current_time is None:
            current_time = time.time()
        
        # NORMAL → 他の状態への遷移
        if self.state == SafetyState.NORMAL:
            if collision:
                self._transition_to_collision_detected(x, y, yaw, current_time)
            elif not model_healthy:
                self._transition_to_fallback(current_time)
        
        # FALLBACK_TO_PP 内での処理
        elif self.state == SafetyState.FALLBACK_TO_PP:
            self._update_fallback_state(model_healthy, current_time)
        
        # COLLISION_DETECTED 内での処理
        elif self.state == SafetyState.COLLISION_DETECTED:
            self._update_collision_state(current_time)
    
    def _transition_to_fallback(self, current_time: float) -> None:
        """NORMAL から FALLBACK_TO_PP へ遷移"""
        self.state = SafetyState.FALLBACK_TO_PP
        self.fallback_start_time = current_time
        self.speed_stage = 0
        self.last_speed_stage_time = current_time
        self.mode_switch_count += 1
    
    def _transition_to_collision_detected(
        self,
        x: float,
        y: float,
        yaw: float,
        current_time: float
    ) -> None:
        """NORMAL から COLLISION_DETECTED へ遷移"""
        self.state = SafetyState.COLLISION_DETECTED
        self.collision_start_time = current_time
        self.collision_recovery.start_recovery(x, y, yaw, current_time)
        self.collision_count += 1
    
    def _update_fallback_state(self, model_healthy: bool, current_time: float) -> None:
        """FALLBACK_TO_PP 状態の更新"""
        if self.fallback_start_time is None:
            return
        
        elapsed = current_time - self.fallback_start_time
        
        # モデルが復帰した
        if model_healthy:
            self.state = SafetyState.NORMAL
            self.speed_stage = 0
            return
        
        # タイムアウト: 停止
        if elapsed > self.fallback_timeout_sec:
            self.state = SafetyState.SAFE_STOP
            return
        
        # 段階的降速の進行
        if self.last_speed_stage_time is not None:
            stage_elapsed = current_time - self.last_speed_stage_time
            if stage_elapsed > self.speed_stage_duration_sec:
                if self.speed_stage < len(self.speed_schedule) - 1:
                    self.speed_stage += 1
                    self.last_speed_stage_time = current_time
    
    def _update_collision_state(self, current_time: float) -> None:
        """COLLISION_DETECTED 状態の更新"""
        if self.collision_start_time is None:
            return
        
        elapsed = current_time - self.collision_start_time
        
        # 復帰完了 → NORMAL へ戻る
        if self.collision_recovery.is_completed():
            self.state = SafetyState.NORMAL
            return
        
        # タイムアウト: 停止
        if elapsed > self.collision_recovery_timeout_sec:
            self.state = SafetyState.SAFE_STOP
    
    def get_action(
        self,
        model_action: Optional[np.ndarray],
        pure_pursuit_action: Optional[np.ndarray],
        robot_state: Optional[Tuple],
        x: float,
        y: float,
        yaw: float,
        current_time: Optional[float] = None
    ) -> np.ndarray:
        """
        現在の状態に応じたアクション出力。
        
        Args:
            model_action: NN モデルからの出力 [steering, speed]
            pure_pursuit_action: Pure Pursuit ベース制御出力
            robot_state: ロボット状態 (x, y, yaw, speed) など
            x, y, yaw: 現在位置・姿勢
            current_time: 現在時刻（秒）
        
        Returns:
            (steering, speed)
        """
        if current_time is None:
            current_time = time.time()
        
        if self.state == SafetyState.NORMAL:
            # モデル制御を優先し、モデルが使用できない場合は Pure Pursuit を使う
            if model_action is not None:
                return np.array(model_action, dtype=np.float32)
            elif pure_pursuit_action is not None:
                return np.array(pure_pursuit_action, dtype=np.float32)
            else:
                return np.array([0.0, 0.0], dtype=np.float32)
        
        elif self.state == SafetyState.FALLBACK_TO_PP:
            # Pure Pursuit 制御 + 段階的降速
            try:
                if robot_state is not None:
                    pp_steer, pp_speed = self.pp_controller.get_base_action(
                        x, y, yaw,
                        current_speed=robot_state[3] if len(robot_state) > 3 else 0.0,
                        max_speed=1.0,
                        min_speed=0.1
                    )
                else:
                    pp_steer, pp_speed = self.pp_controller.get_base_action(
                        x, y, yaw,
                        current_speed=0.0,
                        max_speed=1.0,
                        min_speed=0.1
                    )
                
                # 段階的降速を適用
                speed_factor = self.speed_schedule[min(self.speed_stage, len(self.speed_schedule) - 1)]
                return np.array([pp_steer, pp_speed * speed_factor], dtype=np.float32)
            except Exception:
                # Pure Pursuit 失敗時は停止
                return np.array([0.0, 0.0], dtype=np.float32)
        
        elif self.state == SafetyState.COLLISION_DETECTED:
            # 衝突復帰シーケンス
            recovery_action = self.collision_recovery.compute_recovery_action(
                x, y, yaw, current_time
            )
            if recovery_action is not None:
                return np.array(recovery_action, dtype=np.float32)
            else:
                return np.array([0.0, 0.0], dtype=np.float32)
        
        elif self.state == SafetyState.SAFE_STOP:
            # 完全停止
            return np.array([0.0, 0.0], dtype=np.float32)
        
        return np.array([0.0, 0.0], dtype=np.float32)
    
    def check_front_collision(self, lidar: np.ndarray) -> Tuple[bool, float]:
        """
        LiDAR スキャンから前方衝突を検出する。
        
        Args:
            lidar: LiDAR スキャンデータ
        
        Returns:
            (is_collision, front_min_distance)
        """
        num_beams = len(lidar)
        mid = num_beams // 2
        width = int((self.collision_check_angle_deg / self.collision_total_fov_deg) * num_beams)
        
        start_idx = max(0, mid - width)
        end_idx = min(num_beams, mid + width)
        
        check_area = lidar[start_idx:end_idx]
        # NaN と inf を除去
        valid_ranges = check_area[np.isfinite(check_area)]
        
        if len(valid_ranges) == 0:
            front_min = self.collision_stop_dist + 1.0  # 安全な値
        else:
            front_min = np.min(valid_ranges)
        
        is_collision = front_min < self.collision_stop_dist
        return is_collision, front_min
    
    def get_state(self) -> SafetyState:
        """現在の安全状態を取得"""
        return self.state
    
    def get_state_name(self) -> str:
        """現在の安全状態を名前で取得"""
        return self.state.name
    
    def get_stats(self) -> Dict:
        """安全管理の統計情報を取得"""
        return {
            'current_state': self.state.name,
            'speed_stage': self.speed_stage,
            'speed_factor': self.speed_schedule[min(self.speed_stage, len(self.speed_schedule) - 1)],
            'mode_switch_count': self.mode_switch_count,
            'collision_count': self.collision_count,
            'recovery_phase': self.collision_recovery.get_phase().name if self.state == SafetyState.COLLISION_DETECTED else 'N/A',
        }
    
    def reset(self) -> None:
        """リセット（エピソード終了時など）"""
        self.state = SafetyState.NORMAL
        self.fallback_start_time = None
        self.collision_start_time = None
        self.speed_stage = 0
        self.last_speed_stage_time = None
        self.model_monitor.reset()
        self.collision_recovery.reset()


# ============================================================================
# テスト関数
# ============================================================================

class MockModelHealth:
    """推論健全性モニターのモック"""
    def __init__(self):
        self.healthy = True
    def reset(self):
        pass


class MockPurePursuit:
    """Pure Pursuit コントローラーのモック"""
    def get_base_action(self, x, y, yaw, current_speed, max_speed, min_speed):
        return (0.1, 0.5)


class MockCollisionRecovery:
    """衝突復帰シーケンスのモック"""
    def __init__(self):
        self._phase = RecoveryPhase.IDLE
    
    def start_recovery(self, x, y, yaw, current_time=None):
        self._phase = RecoveryPhase.BACKING
    
    def compute_recovery_action(self, x, y, yaw, t):
        if self._phase == RecoveryPhase.BACKING:
            return (0.0, -0.3)
        return None
    
    def is_completed(self):
        return self._phase == RecoveryPhase.COMPLETED
    
    def get_phase(self):
        return self._phase
    
    def reset(self):
        self._phase = RecoveryPhase.IDLE


def test_safety_manager():
    """SafetyManager の単体テスト"""
    
    model_monitor = MockModelHealth()
    pp = MockPurePursuit()
    recovery = MockCollisionRecovery()
    safety = SafetyManager(
        model_monitor, pp, recovery,
        fallback_timeout_sec=2.0,
        speed_stage_duration_sec=0.5
    )
    
    # Test 1: 初期状態
    print("Test 1: 初期状態")
    assert safety.state == SafetyState.NORMAL, "初期状態が NORMAL ではない"
    print("✓ 初期状態が NORMAL")
    
    # Test 2: NORMAL 状態でのアクション出力
    print("\nTest 2: NORMAL 状態でのアクション出力")
    model_action = np.array([0.1, 0.5])
    action = safety.get_action(model_action, None, 0, 0, 0, 0)
    assert np.allclose(action, model_action), "モデルアクションが変更されている"
    print("✓ モデルアクションがそのまま出力された")
    
    # Test 3: モデル失敗でフォールバック
    print("\nTest 3: モデル失敗でフォールバック")
    model_monitor.healthy = False
    safety.update_state(False, False, 0, 0, 0, 0)
    assert safety.state == SafetyState.FALLBACK_TO_PP, "FALLBACK_TO_PP に遷移しない"
    assert safety.mode_switch_count == 1, "モード切り替えカウントが0ではない"
    print("✓ FALLBACK_TO_PP に遷移")
    
    # Test 4: フォールバック中のアクション
    print("\nTest 4: フォールバック中のアクション")
    action = safety.get_action(None, (0, 0, 0, 0), 0, 0, 0, 0)
    assert np.isclose(action[0], 0.1), f"ステアリングが Pure Pursuit の値ではない: {action[0]}"
    assert np.isclose(action[1], 0.5), f"速度が初期段階の値ではない: {action[1]}"
    print("✓ Pure Pursuit アクションが出力される")
    
    # Test 5: 段階的降速
    print("\nTest 5: 段階的降速")
    safety.update_state(False, False, 0, 0, 0, 1.0)
    action = safety.get_action(None, (0, 0, 0, 0), 0, 0, 0, 1.0)
    expected_speed = 0.5 * 0.8
    assert np.isclose(action[1], expected_speed), f"速度が正しく低下していない: {action[1]} (expected {expected_speed})"
    print("✓ 段階的降速が適用される")
    
    # Test 6: 衝突検出
    print("\nTest 6: 衝突検出")
    safety.reset()
    model_monitor.healthy = True
    safety.update_state(True, True, 0, 0, 0, 0)
    assert safety.state == SafetyState.COLLISION_DETECTED, "COLLISION_DETECTED に遷移しない"
    assert safety.collision_count == 1, "衝突カウントが0ではない"
    print("✓ COLLISION_DETECTED に遷移")
    
    # Test 7: 前方障害物検知
    print("\nTest 7: 前方障害物検知")
    lidar_safe = np.ones(270) * 1.0  # 1m 以上
    is_collision, front_min = safety.check_front_collision(lidar_safe)
    assert is_collision == False, "安全な LiDAR が衝突と判定される"
    
    lidar_danger = np.ones(270) * 0.2  # 0.2m （衝突距離 0.3m）
    is_collision, front_min = safety.check_front_collision(lidar_danger)
    assert is_collision == True, "危険な LiDAR が安全と判定される"
    print("✓ LiDAR 衝突検知が正常に機能")
    
    # Test 8: リセット
    print("\nTest 8: リセット")
    safety.reset()
    assert safety.state == SafetyState.NORMAL, "リセット後も NORMAL ではない"
    assert safety.speed_stage == 0, "リセット後も速度段階が0ではない"
    print("✓ リセットが正常に機能")
    
    print("\n" + "="*50)
    print("✅ SafetyManager のすべてのテストが合格しました")
    print("="*50)


if __name__ == "__main__":
    test_safety_manager()
