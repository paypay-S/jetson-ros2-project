"""
collision_recovery.py

衝突後、段階的に復帰するシーケンスを実行するモジュール。
状態遷移: IDLE → BACKING → TURNING → MOVING_FORWARD → COMPLETED
"""

import time
import numpy as np
from typing import Optional, Tuple
from enum import Enum

from .pure_pursuit import PurePursuitController


class RecoveryPhase(Enum):
    """衝突復帰シーケンスの各フェーズ"""
    IDLE = 0
    BACKING = 1        # バック
    TURNING = 2        # 回転
    MOVING_FORWARD = 3  # 前進再開
    COMPLETED = 4      # 復帰完了


class CollisionRecoverySequence:
    """
    衝突復帰シーケンス管理クラス。
    
    衝突検出後、以下のシーケンスを実行：
    1. BACKING (1.5秒): 直線後退
    2. TURNING (2.0秒): 目標方向へ回転
    3. MOVING_FORWARD (1.0秒): 前進再開
    """
    
    def __init__(
        self,
        pure_pursuit_controller: PurePursuitController,
        backing_time: float = 1.5,
        turning_time: float = 2.0,
        moving_forward_time: float = 1.0,
        backing_speed: float = -0.3,
        turning_steer: float = 0.4,
        forward_max_speed: float = 0.2,
        wheelbase: float = 0.33
    ):
        """
        Args:
            pure_pursuit_controller: Pure Pursuit コントローラー
            backing_time: バック時間（秒）
            turning_time: 回転時間（秒）
            moving_forward_time: 前進時間（秒）
            backing_speed: バック時の速度 (負数)
            turning_steer: 回転時のステアリング角
            forward_max_speed: 前進再開時の最大速度
            wheelbase: 車両ホイールベース（m）
        """
        self.pp_controller = pure_pursuit_controller
        
        # 時間設定
        self.backing_time = backing_time
        self.turning_time = turning_time
        self.moving_forward_time = moving_forward_time
        
        # 制御パラメータ
        self.backing_speed = backing_speed
        self.turning_steer = turning_steer
        self.forward_max_speed = forward_max_speed
        self.wheelbase = wheelbase
        
        # 状態
        self.phase = RecoveryPhase.IDLE
        self.start_time: Optional[float] = None
        self.phase_start_time: Optional[float] = None
        self.recovery_start_pose: Optional[Tuple[float, float, float]] = None
    
    def start_recovery(self, x: float, y: float, yaw: float, current_time: Optional[float] = None) -> None:
        """
        復帰シーケンス開始。
        
        Args:
            x, y: 衝突時の位置
            yaw: 衝突時の姿勢角（rad）
            current_time: 現在時刻（秒、省略時は time.time()）
        """
        if current_time is None:
            current_time = time.time()
        
        self.phase = RecoveryPhase.BACKING
        self.start_time = current_time
        self.phase_start_time = current_time
        self.recovery_start_pose = (x, y, yaw)
    
    def compute_recovery_action(
        self,
        x: float,
        y: float,
        yaw: float,
        current_time: Optional[float] = None
    ) -> Optional[Tuple[float, float]]:
        """
        復帰シーケンス内のアクション計算。
        
        Args:
            x, y: 現在位置
            yaw: 現在の姿勢角（rad）
            current_time: 現在時刻（秒、省略時は time.time()）
        
        Returns:
            (steering, speed) のタプル、またはシーケンス完了時は None
        """
        if self.phase == RecoveryPhase.IDLE:
            return None
        
        if current_time is None:
            current_time = time.time()
        
        if self.phase_start_time is None:
            return None
        
        phase_elapsed = current_time - self.phase_start_time
        
        # ========== BACKING フェーズ ==========
        if self.phase == RecoveryPhase.BACKING:
            if phase_elapsed < self.backing_time:
                # 直線後退
                return (0.0, self.backing_speed)
            else:
                # TURNING フェーズへ移行
                self.phase = RecoveryPhase.TURNING
                self.phase_start_time = current_time
                # 再帰的に TURNING フェーズを実行
                return self.compute_recovery_action(x, y, yaw, current_time)
        
        # ========== TURNING フェーズ ==========
        elif self.phase == RecoveryPhase.TURNING:
            if phase_elapsed < self.turning_time:
                # 目標方向を判定して回転
                if self.recovery_start_pose is not None:
                    start_yaw = self.recovery_start_pose[2]
                    # yaw の正規化 [0, 2π)
                    normalized_yaw = start_yaw % (2 * np.pi)
                    # π を中心に判定
                    target_steer = self.turning_steer if normalized_yaw < np.pi else -self.turning_steer
                else:
                    target_steer = self.turning_steer
                return (target_steer, 0.0)
            else:
                # MOVING_FORWARD フェーズへ移行
                self.phase = RecoveryPhase.MOVING_FORWARD
                self.phase_start_time = current_time
                return self.compute_recovery_action(x, y, yaw, current_time)
        
        # ========== MOVING_FORWARD フェーズ ==========
        elif self.phase == RecoveryPhase.MOVING_FORWARD:
            if phase_elapsed < self.moving_forward_time:
                # Pure Pursuit + 速度制限
                try:
                    pp_steer, pp_speed = self.pp_controller.get_base_action(
                        x, y, yaw,
                        current_speed=0.0,
                        max_speed=self.forward_max_speed,
                        min_speed=0.1
                    )
                    # 速度を制限
                    limited_speed = min(pp_speed, self.forward_max_speed)
                    return (pp_steer, limited_speed)
                except Exception:
                    # Pure Pursuit 失敗時は弱い前進
                    return (0.0, self.forward_max_speed)
            else:
                # 復帰完了
                self.phase = RecoveryPhase.COMPLETED
                return None
        
        return None
    
    def is_completed(self) -> bool:
        """復帰シーケンスが完了したか"""
        return self.phase == RecoveryPhase.COMPLETED
    
    def get_phase(self) -> RecoveryPhase:
        """現在のフェーズを取得"""
        return self.phase
    
    def get_elapsed_time(self, current_time: Optional[float] = None) -> float:
        """シーケンス開始からの経過時間（秒）"""
        if self.start_time is None:
            return 0.0
        if current_time is None:
            current_time = time.time()
        return current_time - self.start_time
    
    def reset(self) -> None:
        """リセット（エピソード終了時など）"""
        self.phase = RecoveryPhase.IDLE
        self.start_time = None
        self.phase_start_time = None
        self.recovery_start_pose = None


# ============================================================================
# テスト関数
# ============================================================================

class MockPurePursuitController:
    """Pure Pursuit コントローラーのモック"""
    
    def get_base_action(self, x, y, yaw, current_speed, max_speed, min_speed):
        """モックの Pure Pursuit 出力"""
        return (0.1, 0.3)  # dummy output


def test_collision_recovery_sequence():
    """CollisionRecoverySequence の単体テスト"""
    
    pp_mock = MockPurePursuitController()
    recovery = CollisionRecoverySequence(
        pp_mock,
        backing_time=1.0,
        turning_time=1.0,
        moving_forward_time=1.0
    )
    
    # Test 1: 初期状態
    print("Test 1: 初期状態")
    assert recovery.phase == RecoveryPhase.IDLE, "初期フェーズが IDLE ではない"
    assert recovery.is_completed() == False, "未開始状態で is_completed が True"
    print("✓ 初期状態が正しい")
    
    # Test 2: 復帰開始
    print("\nTest 2: 復帰開始")
    current_time = 0.0
    recovery.start_recovery(x=0.0, y=0.0, yaw=0.0, current_time=current_time)
    assert recovery.phase == RecoveryPhase.BACKING, "フェーズが BACKING ではない"
    assert recovery.recovery_start_pose == (0.0, 0.0, 0.0), "開始位置が保存されていない"
    print("✓ 復帰シーケンスが開始された")
    
    # Test 3: BACKING フェーズ
    print("\nTest 3: BACKING フェーズ")
    action = recovery.compute_recovery_action(0.0, 0.0, 0.0, current_time + 0.5)
    assert action == (0.0, -0.3), f"BACKING アクションが不正: {action}"
    assert recovery.phase == RecoveryPhase.BACKING, "フェーズが BACKING 中に変更された"
    print("✓ BACKING フェーズのアクションが正しい")
    
    # Test 4: TURNING フェーズへの移行
    print("\nTest 4: TURNING フェーズへの移行")
    action = recovery.compute_recovery_action(0.0, 0.0, 0.0, current_time + 1.2)
    assert recovery.phase == RecoveryPhase.TURNING, "フェーズが TURNING に移行しない"
    assert action[1] == 0.0, "TURNING フェーズで速度が0でない"
    assert abs(action[0]) > 0.0, "TURNING フェーズでステアリングが0"
    print("✓ TURNING フェーズに移行、ステアリングが適用された")
    
    # Test 5: MOVING_FORWARD フェーズへの移行
    print("\nTest 5: MOVING_FORWARD フェーズへの移行")
    action = recovery.compute_recovery_action(0.0, 0.0, 0.0, current_time + 2.3)
    assert recovery.phase == RecoveryPhase.MOVING_FORWARD, "フェーズが MOVING_FORWARD に移行しない"
    assert action[1] <= 0.2, f"MOVING_FORWARD で速度が制限されていない: {action[1]}"
    print("✓ MOVING_FORWARD フェーズに移行、速度が制限されている")
    
    # Test 6: 復帰完了
    print("\nTest 6: 復帰完了")
    action = recovery.compute_recovery_action(0.0, 0.0, 0.0, current_time + 3.5)
    assert action is None, "復帰完了後も None が返されない"
    assert recovery.phase == RecoveryPhase.COMPLETED, "フェーズが COMPLETED ではない"
    assert recovery.is_completed() == True, "is_completed が True ではない"
    print("✓ 復帰シーケンスが正常に完了")
    
    # Test 7: リセット
    print("\nTest 7: リセット")
    recovery.reset()
    assert recovery.phase == RecoveryPhase.IDLE, "リセット後もフェーズが IDLE ではない"
    assert recovery.is_completed() == False, "リセット後も is_completed が True"
    print("✓ リセットが正常に機能")
    
    # Test 8: 経過時間取得
    print("\nTest 8: 経過時間取得")
    recovery.reset()
    recovery.start_recovery(0.0, 0.0, 0.0, current_time=current_time)
    elapsed = recovery.get_elapsed_time(current_time + 2.5)
    assert abs(elapsed - 2.5) < 0.01, f"経過時間が不正: {elapsed}"
    print("✓ 経過時間が正しく取得される")
    
    print("\n" + "="*50)
    print("✅ CollisionRecoverySequence のすべてのテストが合格しました")
    print("="*50)


if __name__ == "__main__":
    test_collision_recovery_sequence()
