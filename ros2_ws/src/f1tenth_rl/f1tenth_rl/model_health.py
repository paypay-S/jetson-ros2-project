"""
model_health.py

推論実行時間・出力値を監視し、以下を検出：
- タイムアウト: 推論が 20ms を超過
- NaN/Inf: 出力に無限値や NaN を含む
- 範囲外: 出力が [-1.0, 1.0] 外に出ている
"""

import time
import numpy as np
from collections import deque
from typing import Tuple, Dict, Optional


class ModelHealthMonitor:
    """
    推論の健全性を監視するモニタークラス。
    """
    
    def __init__(self, timeout_sec: float = 0.020, max_history: int = 10):
        """
        Args:
            timeout_sec: 推論タイムアウト時間（秒）。デフォルト 20ms
            max_history: 推論時間の履歴保持数
        """
        self.timeout_sec = timeout_sec
        self.max_history = max_history
        
        self.last_valid_action = None
        self.inference_times = deque(maxlen=max_history)
        self.start_time = None
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
    
    def start_inference(self) -> None:
        """推論開始時刻を記録する"""
        self.start_time = time.time()
    
    def end_inference(self, action: np.ndarray) -> Tuple[bool, Dict[str, bool]]:
        """
        推論完了・検証を行う。
        
        Args:
            action: NN モデルからの出力 [steering, speed]
        
        Returns:
            (is_healthy, error_dict)
            - is_healthy: True ならば健全、False ならば異常検出
            - error_dict: 検出された異常種別
                - 'timeout': タイムアウト
                - 'nan': NaN 値を含む
                - 'inf': 無限値を含む
                - 'out_of_bounds': [-1.0, 1.0] の範囲外
        """
        if self.start_time is None:
            return False, {'no_start_time': True}
        
        elapsed = time.time() - self.start_time
        self.inference_times.append(elapsed)
        
        errors = {}
        
        # タイムアウト検査
        if elapsed > self.timeout_sec:
            errors['timeout'] = True
        
        # NaN 検査
        if np.isnan(action).any():
            errors['nan'] = True
        
        # 無限値検査
        if np.isinf(action).any():
            errors['inf'] = True
        
        # 範囲外検査 [-1.0, 1.0]
        if (action < -1.0).any() or (action > 1.0).any():
            errors['out_of_bounds'] = True
        
        # 結果判定
        if not errors:
            self.last_valid_action = action.copy()
            self.consecutive_errors = 0
            return True, {}
        else:
            self.consecutive_errors += 1
            return False, errors
    
    def get_avg_inference_time(self) -> float:
        """直近の平均推論時間を取得（秒）"""
        if not self.inference_times:
            return 0.0
        return float(np.mean(self.inference_times))
    
    def get_max_inference_time(self) -> float:
        """直近の最大推論時間を取得（秒）"""
        if not self.inference_times:
            return 0.0
        return float(np.max(self.inference_times))
    
    def get_last_valid_action(self) -> Optional[np.ndarray]:
        """最後に有効だったアクションを取得"""
        return self.last_valid_action.copy() if self.last_valid_action is not None else None
    
    def is_consecutive_error_exceeded(self) -> bool:
        """連続エラーが閾値を超えているか"""
        return self.consecutive_errors > self.max_consecutive_errors
    
    def get_stats(self) -> Dict:
        """推論統計を取得"""
        return {
            'avg_inference_time_ms': self.get_avg_inference_time() * 1000,
            'max_inference_time_ms': self.get_max_inference_time() * 1000,
            'consecutive_errors': self.consecutive_errors,
            'history_size': len(self.inference_times),
        }
    
    def reset(self) -> None:
        """リセット（エピソード終了時など）"""
        self.inference_times.clear()
        self.last_valid_action = None
        self.start_time = None
        self.consecutive_errors = 0


# ============================================================================
# テスト関数
# ============================================================================

def test_model_health_monitor():
    """ModelHealthMonitor の単体テスト"""
    import sys
    
    monitor = ModelHealthMonitor(timeout_sec=0.020)
    
    # Test 1: 正常な推論
    print("Test 1: 正常な推論")
    monitor.start_inference()
    time.sleep(0.010)  # 10ms
    action = np.array([0.1, 0.5])
    healthy, errors = monitor.end_inference(action)
    assert healthy == True, "正常な推論が失敗と判定された"
    assert errors == {}, f"エラーが検出された: {errors}"
    assert monitor.consecutive_errors == 0, "エラーカウンタが0になっていない"
    print("✓ 正常な推論が正しく認識された")
    
    # Test 2: タイムアウト
    print("\nTest 2: タイムアウト検出")
    monitor.start_inference()
    time.sleep(0.025)  # 25ms > 20ms
    action = np.array([0.1, 0.5])
    healthy, errors = monitor.end_inference(action)
    assert healthy == False, "タイムアウトが正常と判定された"
    assert errors.get('timeout') == True, "タイムアウトが検出されない"
    assert monitor.consecutive_errors == 1, "エラーカウンタが正しくインクリメントされない"
    print("✓ タイムアウトが正しく検出された")
    
    # Test 3: NaN 検出
    print("\nTest 3: NaN 検出")
    monitor.start_inference()
    time.sleep(0.010)
    action = np.array([np.nan, 0.5])
    healthy, errors = monitor.end_inference(action)
    assert healthy == False, "NaNが正常と判定された"
    assert errors.get('nan') == True, "NaN が検出されない"
    print("✓ NaN が正しく検出された")
    
    # Test 4: 無限値検出
    print("\nTest 4: 無限値検出")
    monitor.start_inference()
    time.sleep(0.010)
    action = np.array([0.5, np.inf])
    healthy, errors = monitor.end_inference(action)
    assert healthy == False, "inf が正常と判定された"
    assert errors.get('inf') == True, "inf が検出されない"
    print("✓ 無限値が正しく検出された")
    
    # Test 5: 範囲外検出
    print("\nTest 5: 範囲外検出")
    monitor.start_inference()
    time.sleep(0.010)
    action = np.array([1.5, 0.5])  # 1.5 > 1.0
    healthy, errors = monitor.end_inference(action)
    assert healthy == False, "範囲外値が正常と判定された"
    assert errors.get('out_of_bounds') == True, "範囲外が検出されない"
    print("✓ 範囲外が正しく検出された")
    
    # Test 6: 最後の有効なアクション取得
    print("\nTest 6: 最後の有効なアクション取得")
    monitor.reset()
    monitor.start_inference()
    time.sleep(0.010)
    valid_action = np.array([0.2, 0.3])
    monitor.end_inference(valid_action)
    last_valid = monitor.get_last_valid_action()
    assert last_valid is not None, "最後の有効アクションが None"
    assert np.allclose(last_valid, valid_action), "最後の有効アクションが一致しない"
    print("✓ 最後の有効なアクションが正しく取得された")
    
    # Test 7: 連続エラー検出
    print("\nTest 7: 連続エラー検出")
    monitor.reset()
    for i in range(6):
        monitor.start_inference()
        time.sleep(0.010)
        action = np.array([np.nan, 0.5])
        monitor.end_inference(action)
    assert monitor.is_consecutive_error_exceeded(), "連続エラーが検出されない"
    print("✓ 連続エラーが正しく検出された")
    
    print("\n" + "="*50)
    print("✅ ModelHealthMonitor のすべてのテストが合格しました")
    print("="*50)


if __name__ == "__main__":
    test_model_health_monitor()
