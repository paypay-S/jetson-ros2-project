import os
import numpy as np
import onnxruntime as ort
from stable_baselines3 import PPO
from typing import Optional, Tuple, Deque, List
from collections import deque

class ModelManager:
    """
    SB3 または ONNX モデルの読み込みと推論を管理するクラス。
    """
    def __init__(self, logger):
        self.logger = logger
        self.model = None
        self.ort_session = None
        self.model_type: Optional[str] = None
        self.input_dim = 0

    def load_model(self,model_path: str):
        if not model_path or model_path.lower() == "none":
            self.logger.info("No model path provided. Running in recovery-only mode.")
            return

        try:
            if model_path.endswith('.onnx'):
                self.model_type = 'onnx'
                self.ort_session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
                self.logger.info(f"Loaded ONNX model: {model_path}")
                self.input_dim = self.ort_session.get_inputs()[0].shape[1]
            else:
                self.model_type = 'sb3'
                # SB3 load logic
                load_path = model_path
                if not os.path.exists(load_path) and os.path.exists(load_path + ".zip"):
                    load_path += ".zip"
                
                if os.path.exists(load_path):
                    self.model = PPO.load(load_path, device="cpu")
                    self.logger.info(f"Loaded SB3 model: {load_path}")
                    self.input_dim = self.model.observation_space.shape[0]
                else:
                    self.logger.error(f"Model file not found: {model_path}")
        except Exception as e:
            self.logger.error(f"Failed to load model: {e}")

    def predict(self, state: np.ndarray) -> Optional[np.ndarray]:
        if self.model_type == 'onnx' and self.ort_session:
            ort_inputs = {self.ort_session.get_inputs()[0].name: state.reshape(1, -1).astype(np.float32)}
            return self.ort_session.run(None, ort_inputs)[0][0]
        elif self.model_type == 'sb3' and self.model:
            action, _ = self.model.predict(state, deterministic=True)
            return action
        return None


class RecoveryManager:
    """
    スタックや衝突時の復帰動作（後退シーケンス）を管理するステートマシン。
    """
    def __init__(self, logger, clock):
        self.logger = logger
        self.clock = clock
        self.in_recovery = False
        self.start_time = None
        self.steer = 0.0
        
        # カウンター
        self.trigger_count_wall = 0
        self.trigger_count_stuck = 0
        self.trigger_limit = 5

    def start(self, steer: float):
        self.in_recovery = True
        self.start_time = self.clock.now()
        self.steer = steer
        self.logger.warn(f"Recovery started with steer: {steer:.2f}")

    def get_command(self, 
                    reverse_speed: float, 
                    d_brake: float, d_stop: float, d_back: float) -> Tuple[float, float, bool]:
        """
        現在の経過時間に基づいた指令値を返す。
        Returns: (speed, steer, is_active)
        """
        if not self.in_recovery:
            return 0.0, 0.0, False

        elapsed = (self.clock.now() - self.start_time).nanoseconds / 1e9
        
        if elapsed < d_brake:
            return reverse_speed, 0.0, True
        elif elapsed < d_brake + d_stop:
            return 0.0, 0.0, True
        elif elapsed < d_brake + d_stop + d_back:
            return reverse_speed, self.steer, True
        else:
            self.logger.info("Recovery sequence complete.")
            self.in_recovery = False
            return 0.0, 0.0, False


# Note: SafetyLayer has been moved to safety.py as SafetyManager
# Please use SafetyManager from safety.py for new code
# SafetyManager provides:
# - State transitions (NORMAL → FALLBACK_TO_PP → SAFE_STOP)
# - Model health monitoring integration
# - Collision recovery sequence management
# - Gradual speed reduction strategy
# - Front collision detection with LiDAR
