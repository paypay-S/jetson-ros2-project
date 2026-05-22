# モデル失敗時対策 & 衝突復帰システム - 実装ガイド

## 🎯 全体目標

Jetson 実機走行でモデル推論が失敗・遅延した場合、**Pure Pursuit 単独での安全走行** へ自動切り替え。  
衝突時は **段階的復帰シーケンス**（バック→回転→前進）を実行する。

---

## 📊 システム構成図

```
┌─────────────────────────────────────────────────────────┐
│  Jetson ROS2 環境                                         │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  [LiDAR Driver]                                         │
│         ↓ /scan                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  jetson_policy_node.py                           │  │
│  │  - ONNX 推論実行                                 │  │
│  │  - 推論タイムアウト検出 (18ms)                   │  │
│  │  - ヘルスチェック監視                            │  │
│  │  → /action (推論出力)                            │  │
│  │  → /model_health (状態)                          │  │
│  └──────────────────────────────────────────────────┘  │
│         ↓                                                │
│  ┌──────────────────────────────────────────────────┐  │
│  │  ros2_safety_node.py                             │  │
│  │  - 状態遷移管理                                  │  │
│  │  - モデル失敗 → PP フォールバック               │  │
│  │  - 衝突復帰シーケンス実行                        │  │
│  │  → /safe_action (確定アクション)                │  │
│  │  ← /emergency_stop (緊急停止)                    │  │
│  └──────────────────────────────────────────────────┘  │
│         ↓ /safe_action                                  │
│  [Motor Driver (VESC)]                                 │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

---

## 📋 実装フェーズ

| フェーズ | 内容 | 依存関係 | 予定期間 |
|:---:|:---:|:---:|:---:|
| **Phase 1** | シミュレーション側: Safety Manager, Model Health, Collision Recovery | なし | 2-3 日 |
| **Phase 2** | `src/f1_env.py` に Safety 統合 | Phase 1 | 1 日 |
| **Phase 3** | Jetson/ROS2 ノード実装 | Phase 1 | 3-4 日 |
| **Phase 4** | テスト & 検証 | Phase 2, 3 | 2 日 |
| **Phase 5** | ドキュメント & チューニング | 全フェーズ | 1 日 |

---

## 📁 ファイルツリー (実装後)

```
docs/
  ├─ 00_IMPLEMENTATION_OVERVIEW.md        (本ファイル)
  ├─ 01_PHASE1_COMPONENTS.md             (基礎コンポーネント設計)
  ├─ 02_PHASE2_SIMULATION.md             (シミュレーション統合)
  ├─ 03_PHASE3_JETSON_ROS2.md            (Jetson/ROS2 実装)
  ├─ 04_TESTING_GUIDE.md                 (テスト手順)
  ├─ 05_JETSON_SETUP.md                  (Jetson セットアップ)
  └─ 06_TROUBLESHOOTING.md               (トラブルシューティング)

src/
  ├─ safety.py                           (NEW: 状態管理)
  ├─ model_health.py                     (NEW: 推論監視)
  ├─ collision_recovery.py                (NEW: 衝突復帰)
  ├─ f1_env.py                           (MODIFIED: Safety 統合)
  └─ config.py                           (MODIFIED: Safety 設定)

sharing/
  ├─ jetson_policy_node.py               (NEW: ポリシー推論ノード)
  ├─ ros2_safety_node.py                 (NEW: 安全管理ノード)
  ├─ jetson_main.py                      (NEW: 統合エントリポイント)
  ├─ test_jetson_integration.py           (NEW: 実機テスト)
  └─ JETSON_DEPLOYMENT_PLAN.md           (UPDATED)

scripts/
  └─ test_safety_system.py               (NEW: シミュレーション テスト)
```

---

## 🔄 状態遷移フロー

```
START
  ↓
[NORMAL] ─────────────────────────────────────────┐
  ├─→ (model_healthy=False) ──→ [FALLBACK_TO_PP] │
  │                                    ├─→ (timeout > 10s) ──→ [SAFE_STOP]
  │                                    └─→ (recovery OK) ────→ [NORMAL]
  │                                                            ↑
  └─→ (collision=True) ──→ [COLLISION_DETECTED]             │
                              ├─→ [BACKING_UP]              │
                              ├─→ [TURNING]                 │
                              ├─→ [MOVING_FORWARD] ─────────┘
                              └─→ (timeout) ──→ [SAFE_STOP]

[SAFE_STOP] (terminal)
  └─→ speed=0, steering=0 (全状態から到達可能)
```

---

## 🚀 クイックスタート

### 1. 基礎コンポーネント実装 (Phase 1)

```bash
cd /home/toyot/projects/f1tenth-rl-project

# 以下のファイルを作成
# - src/model_health.py
# - src/safety.py
# - src/collision_recovery.py

# 詳細は docs/01_PHASE1_COMPONENTS.md を参照
```

### 2. シミュレーション統合 (Phase 2)

```bash
# src/f1_env.py に Safety Manager 組み込み
# src/config.py に Safety 設定追加
# 詳細は docs/02_PHASE2_SIMULATION.md を参照
```

### 3. Jetson/ROS2 実装 (Phase 3)

```bash
# Jetson 環境で以下を実装
# - sharing/jetson_policy_node.py
# - sharing/ros2_safety_node.py
# - sharing/jetson_main.py

# 詳細は docs/03_PHASE3_JETSON_ROS2.md を参照
```

### 4. テスト & 検証 (Phase 4)

```bash
# シミュレーション側
python scripts/test_safety_system.py

# Jetson 側（実機）
python sharing/test_jetson_integration.py
```

---

## ⚙️ 主要パラメータ

| パラメータ | 値 | 説明 |
|:---:|:---:|:---|
| `MODEL_INFERENCE_TIMEOUT_SEC` | 0.020 | 推論最大実行時間 (20ms, 40Hz制御想定) |
| `FALLBACK_PP_TIMEOUT_SEC` | 10.0 | フォールバック最大継続時間 |
| `COLLISION_RECOVERY_TIMEOUT_SEC` | 5.0 | 衝突復帰最大実行時間 |
| `FALLBACK_SPEED_SCHEDULE` | [1.0, 0.8, 0.5, 0.2, 0.0] | 段階的降速 (5段階) |
| `BACKING_TIME_SEC` | 1.5 | バック時間 |
| `TURNING_TIME_SEC` | 2.0 | 回転時間 |
| `MOVING_FORWARD_TIME_SEC` | 1.0 | 前進再開時間 |

詳細設定は各フェーズのドキュメントを参照。

---

## 📍 次のステップ

**今すぐ始める場合:**

1. `docs/01_PHASE1_COMPONENTS.md` を読む
2. `src/model_health.py` から実装を開始
3. 各コンポーネント完成後、`scripts/test_safety_system.py` で単体テスト
4. Phase 2 → Phase 3 と進める

**Jetson 実機作業に向けて:**

- `docs/05_JETSON_SETUP.md` で Jetson 環境を整備
- `docs/03_PHASE3_JETSON_ROS2.md` で実装開始
- `docs/04_TESTING_GUIDE.md` で本番テストを実施


# Phase 1: 基礎コンポーネント実装ガイド

本フェーズでは、以下の3つの基礎コンポーネントを実装します：

1. **Model Health Monitor** - 推論の健全性監視
2. **Safety Manager** - 状態遷移・フェイルセーフ管理
3. **Collision Recovery** - 衝突復帰シーケンス

---

## 📌 実装順序

```
1. model_health.py      (単独で動作可能)
2. collision_recovery.py (Pure Pursuit Controller に依存)
3. safety.py            (1, 2 に依存) ← 最後に
```

---

## 1. ModelHealthMonitor (`src/model_health.py`)

### 目的

推論実行時間・出力値を監視し、以下を検出：
- **タイムアウト**: 推論が 20ms を超過
- **NaN/Inf**: 出力に無限値や NaN を含む
- **範囲外**: 出力が [-1.0, 1.0] 外に出ている

### 実装概要

```python
class ModelHealthMonitor:
    def __init__(self, timeout_sec: float = 0.020):
        self.timeout_sec = timeout_sec
        self.last_valid_action = None
        self.inference_times = deque(maxlen=10)  # 直近10フレーム
        self.start_time = None
    
    def start_inference(self):
        """推論開始時刻を記録"""
        self.start_time = time.time()
    
    def end_inference(self, action: np.ndarray) -> tuple[bool, dict]:
        """
        推論完了・検証
        Returns:
            (is_healthy, error_dict)
        """
        elapsed = time.time() - self.start_time
        self.inference_times.append(elapsed)
        
        errors = {}
        if elapsed > self.timeout_sec:
            errors['timeout'] = True
        if np.isnan(action).any():
            errors['nan'] = True
        if np.isinf(action).any():
            errors['inf'] = True
        if (action < -1.0).any() or (action > 1.0).any():
            errors['out_of_bounds'] = True
        
        if not errors:
            self.last_valid_action = action.copy()
            return True, {}
        else:
            return False, errors
    
    def get_avg_inference_time(self) -> float:
        return np.mean(self.inference_times) if self.inference_times else 0.0
    
    def reset(self):
        self.inference_times.clear()
        self.last_valid_action = None
```

### テスト方法

```python
# シミュレーション内でテスト
monitor = ModelHealthMonitor(timeout_sec=0.020)

# 正常な推論
monitor.start_inference()
time.sleep(0.010)  # 10ms
action = np.array([0.1, 0.5])
healthy, errors = monitor.end_inference(action)
assert healthy == True
assert errors == {}

# タイムアウト
monitor.start_inference()
time.sleep(0.025)  # 25ms > 20ms
action = np.array([0.1, 0.5])
healthy, errors = monitor.end_inference(action)
assert healthy == False
assert errors.get('timeout') == True

# NaN 検出
monitor.start_inference()
time.sleep(0.010)
action = np.array([np.nan, 0.5])
healthy, errors = monitor.end_inference(action)
assert healthy == False
assert errors.get('nan') == True
```

---

## 2. CollisionRecovery (`src/collision_recovery.py`)

### 目的

衝突後、段階的に復帰するシーケンスを実行：

```
[BACKING]    (1.5秒) ─→ [TURNING]      (2.0秒) ─→ [MOVING_FORWARD] (1.0秒)
speed=-0.3             steering=±0.4              speed=0.2 + PP
steering=0             speed=0
```

### 実装概要

```python
class CollisionRecoverySequence:
    """
    衝突復帰シーケンス管理
    状態遷移: IDLE → BACKING → TURNING → MOVING_FORWARD → COMPLETED
    """
    
    def __init__(self, pure_pursuit_controller, wheelbase: float = 0.33):
        self.pp_controller = pure_pursuit_controller
        self.wheelbase = wheelbase
        
        # 時間設定
        self.BACKING_TIME = 1.5
        self.TURNING_TIME = 2.0
        self.MOVING_FORWARD_TIME = 1.0
        
        # 状態
        self.state = "IDLE"
        self.start_time = None
        self.phase_start_time = None
        self.recovery_start_pose = None
    
    def start_recovery(self, x: float, y: float, yaw: float):
        """復帰シーケンス開始"""
        self.state = "BACKING"
        self.start_time = time.time()
        self.phase_start_time = self.start_time
        self.recovery_start_pose = (x, y, yaw)
    
    def compute_recovery_action(
        self,
        x: float,
        y: float,
        yaw: float,
        current_time: float
    ) -> Optional[tuple[float, float]]:
        """
        復帰シーケンス内のアクション計算
        Returns:
            (steering, speed) or None if completed
        """
        if self.state == "IDLE":
            return None
        
        phase_elapsed = current_time - self.phase_start_time
        
        # ========== BACKING ==========
        if self.state == "BACKING":
            if phase_elapsed < self.BACKING_TIME:
                # 直線後退
                return (0.0, -0.3)
            else:
                # 次フェーズへ
                self.state = "TURNING"
                self.phase_start_time = current_time
        
        # ========== TURNING ==========
        if self.state == "TURNING":
            if phase_elapsed < self.TURNING_TIME:
                # 目標方向を判定して回転
                start_yaw = self.recovery_start_pose[2]
                target_steer = 0.4 if (start_yaw % (2 * np.pi)) < np.pi else -0.4
                return (target_steer, 0.0)
            else:
                # 次フェーズへ
                self.state = "MOVING_FORWARD"
                self.phase_start_time = current_time
        
        # ========== MOVING_FORWARD ==========
        if self.state == "MOVING_FORWARD":
            if phase_elapsed < self.MOVING_FORWARD_TIME:
                # Pure Pursuit + 速度制限
                pp_action = self.pp_controller.compute_action(
                    (x, y, yaw, 0.0)  # 仮の速度 (使わない)
                )
                # 速度を制限 (0.2m/s 以下)
                return (pp_action[0], min(pp_action[1], 0.2))
            else:
                # 復帰完了
                self.state = "COMPLETED"
                return None
        
        return None
    
    def is_completed(self) -> bool:
        return self.state == "COMPLETED"
    
    def reset(self):
        self.state = "IDLE"
        self.start_time = None
        self.phase_start_time = None
        self.recovery_start_pose = None
```

### テスト方法

```python
# Mock Pure Pursuit Controller
class MockPurePursuit:
    def compute_action(self, robot_state):
        return (0.1, 0.5)  # dummy output

recovery = CollisionRecoverySequence(MockPurePursuit())
current_time = 0.0

# 復帰開始
recovery.start_recovery(x=0.0, y=0.0, yaw=0.0)
assert recovery.state == "BACKING"

# BACKING フェーズ
action = recovery.compute_recovery_action(0.0, 0.0, 0.0, current_time + 0.5)
assert action == (0.0, -0.3)

# TURNING フェーズへ移行
action = recovery.compute_recovery_action(0.0, 0.0, 0.0, current_time + 2.0)
assert recovery.state == "TURNING"
assert action == (-0.4, 0.0) or action == (0.4, 0.0)

# MOVING_FORWARD フェーズへ移行
action = recovery.compute_recovery_action(0.0, 0.0, 0.0, current_time + 4.5)
assert recovery.state == "MOVING_FORWARD"
assert action[1] <= 0.2  # 速度制限確認

# 復帰完了
action = recovery.compute_recovery_action(0.0, 0.0, 0.0, current_time + 6.0)
assert action is None
assert recovery.is_completed()
```

---

## 3. SafetyManager (`src/safety.py`)

### 目的

以下を管理：
- **状態遷移**: NORMAL → FALLBACK/COLLISION → SAFE_STOP
- **フォールバック戦略**: モデル失敗時は Pure Pursuit へ
- **段階的降速**: フォールバック中も速度を徐々に落とす

### 実装概要

```python
from enum import Enum

class SafetyState(Enum):
    NORMAL = 1              # モデル制御
    FALLBACK_TO_PP = 2      # Pure Pursuit フォールバック
    COLLISION_DETECTED = 3  # 衝突復帰実行中
    RECOVERY_BACKING = 4
    RECOVERY_TURNING = 5
    RECOVERY_MOVING = 6
    SAFE_STOP = 7           # 停止状態

class SafetyManager:
    """
    安全管理マネージャー
    状態遷移と段階的降速を管理
    """
    
    def __init__(
        self,
        model_monitor: ModelHealthMonitor,
        pure_pursuit_controller,
        collision_recovery: CollisionRecoverySequence,
        fallback_timeout: float = 10.0,
        collision_recovery_timeout: float = 5.0
    ):
        self.state = SafetyState.NORMAL
        self.model_monitor = model_monitor
        self.pp_controller = pure_pursuit_controller
        self.collision_recovery = collision_recovery
        
        # タイムアウト設定
        self.fallback_timeout = fallback_timeout
        self.collision_recovery_timeout = collision_recovery_timeout
        self.fallback_start_time = None
        self.collision_start_time = None
        
        # 段階的降速スケジュール
        self.speed_schedule = [1.0, 0.8, 0.5, 0.2, 0.0]
        self.speed_stage = 0
        self.speed_stage_duration = 2.0  # 各段階 2秒間
        self.last_speed_stage_time = None
    
    def update_state(
        self,
        model_healthy: bool,
        collision: bool,
        x: float,
        y: float,
        yaw: float,
        current_time: float
    ):
        """状態遷移を更新"""
        
        # NORMAL → 他の状態への遷移
        if self.state == SafetyState.NORMAL:
            if collision:
                self.state = SafetyState.COLLISION_DETECTED
                self.collision_start_time = current_time
                self.collision_recovery.start_recovery(x, y, yaw)
            elif not model_healthy:
                self.state = SafetyState.FALLBACK_TO_PP
                self.fallback_start_time = current_time
                self.speed_stage = 0
                self.last_speed_stage_time = current_time
        
        # FALLBACK_TO_PP 内での処理
        elif self.state == SafetyState.FALLBACK_TO_PP:
            elapsed = current_time - self.fallback_start_time
            
            if model_healthy:
                # モデルが復帰した
                self.state = SafetyState.NORMAL
                self.speed_stage = 0
            elif elapsed > self.fallback_timeout:
                # タイムアウト: 停止
                self.state = SafetyState.SAFE_STOP
            
            # 段階的降速の進行
            stage_elapsed = current_time - self.last_speed_stage_time
            if stage_elapsed > self.speed_stage_duration:
                if self.speed_stage < len(self.speed_schedule) - 1:
                    self.speed_stage += 1
                    self.last_speed_stage_time = current_time
        
        # COLLISION_DETECTED 内での処理
        elif self.state == SafetyState.COLLISION_DETECTED:
            elapsed = current_time - self.collision_start_time
            
            if self.collision_recovery.is_completed():
                # 復帰完了 → NORMAL へ戻る
                self.state = SafetyState.NORMAL
            elif elapsed > self.collision_recovery_timeout:
                # タイムアウト: 停止
                self.state = SafetyState.SAFE_STOP
    
    def get_action(
        self,
        model_action: Optional[np.ndarray],
        robot_state: Optional[tuple],
        x: float,
        y: float,
        yaw: float,
        current_time: float
    ) -> np.ndarray:
        """
        現在の状態に応じたアクション出力
        
        Args:
            model_action: NN モデルからの出力 [steering, speed]
            robot_state: (x, y, yaw, speed) など
            x, y, yaw: 現在位置・姿勢
            current_time: 現在時刻
        
        Returns:
            (steering, speed)
        """
        
        if self.state == SafetyState.NORMAL:
            return model_action if model_action is not None else np.array([0.0, 0.0])
        
        elif self.state == SafetyState.FALLBACK_TO_PP:
            # Pure Pursuit 制御 + 段階的降速
            pp_action = self.pp_controller.compute_action(robot_state)
            speed_factor = self.speed_schedule[min(self.speed_stage, len(self.speed_schedule) - 1)]
            return np.array([pp_action[0], pp_action[1] * speed_factor], dtype=np.float32)
        
        elif self.state == SafetyState.COLLISION_DETECTED:
            # 衝突復帰シーケンス
            recovery_action = self.collision_recovery.compute_recovery_action(x, y, yaw, current_time)
            if recovery_action is not None:
                return np.array(recovery_action, dtype=np.float32)
            else:
                # 復帰完了待ちの間
                return np.array([0.0, 0.0], dtype=np.float32)
        
        elif self.state == SafetyState.SAFE_STOP:
            # 完全停止
            return np.array([0.0, 0.0], dtype=np.float32)
        
        return np.array([0.0, 0.0], dtype=np.float32)
    
    def reset(self):
        """エピソード終了時のリセット"""
        self.state = SafetyState.NORMAL
        self.fallback_start_time = None
        self.collision_start_time = None
        self.speed_stage = 0
        self.last_speed_stage_time = None
        self.model_monitor.reset()
        self.collision_recovery.reset()
```

### テスト方法

```python
# Mock コンポーネント
class MockModelHealth:
    def __init__(self):
        self.healthy = True
    def reset(self):
        pass

class MockPurePursuit:
    def compute_action(self, state):
        return (0.1, 0.5)

class MockCollisionRecovery:
    def __init__(self):
        self.state = "IDLE"
    def start_recovery(self, x, y, yaw):
        self.state = "BACKING"
    def compute_recovery_action(self, x, y, yaw, t):
        return (0.0, -0.3) if self.state == "BACKING" else None
    def is_completed(self):
        return self.state == "COMPLETED"
    def reset(self):
        self.state = "IDLE"

# Safety Manager テスト
model_monitor = MockModelHealth()
pp = MockPurePursuit()
recovery = MockCollisionRecovery()
safety = SafetyManager(model_monitor, pp, recovery, fallback_timeout=2.0)

# NORMAL 状態でのアクション出力
model_action = np.array([0.1, 0.5])
action = safety.get_action(model_action, None, 0, 0, 0, 0)
assert safety.state == SafetyState.NORMAL
assert np.allclose(action, model_action)

# モデル失敗でフォールバック
model_monitor.healthy = False
safety.update_state(False, False, 0, 0, 0, 0)
assert safety.state == SafetyState.FALLBACK_TO_PP

# PP アクション取得
action = safety.get_action(None, (0, 0, 0, 0), 0, 0, 0, 0)
assert action[1] == 0.5  # 初期段階は速度 100%

# 段階的降速（2秒×5段階 = 10秒）
for stage in range(1, 5):
    safety.update_state(False, False, 0, 0, 0, stage * 2.0 + 0.1)
    action = safety.get_action(None, (0, 0, 0, 0), 0, 0, 0, stage * 2.0 + 0.1)
    expected_speed = 0.5 * safety.speed_schedule[stage]
    assert action[1] == expected_speed

# タイムアウトで停止
safety.update_state(False, False, 0, 0, 0, 15.0)
assert safety.state == SafetyState.SAFE_STOP

print("✓ All SafetyManager tests passed")
```

---

## ✅ Phase 1 完了チェックリスト

- [ ] `src/model_health.py` 実装完了
- [ ] `src/collision_recovery.py` 実装完了
- [ ] `src/safety.py` 実装完了
- [ ] 各コンポーネントのユニットテスト合格
- [ ] インテグレーションテスト（3つのコンポーネント併用）で合格
- [ ] コード品質チェック (PEP8, type hints)

---

## 次のステップ

→ `docs/02_PHASE2_SIMULATION.md` で `src/f1_env.py` へ統合


# Phase 2: シミュレーション環境への統合

Phase 1 で実装したコンポーネントを、シミュレーション環境 (`src/f1_env.py`) に組み込みます。

---

## 📌 変更ファイル

1. **`src/f1_env.py`** - Safety Manager 統合
2. **`src/config.py`** - Safety 関連設定追加

---

## 1. `src/config.py` への追加設定

### 追加内容

`src/config.py` の末尾に以下を追加：

```python
# ============================================================
# Safety & Failsafe Settings
# ============================================================

# Safety Manager の有効化
ENABLE_SAFETY_MANAGER = True

# 衝突復帰機能の有効化
ENABLE_COLLISION_RECOVERY = True

# モデル推論のタイムアウト [秒]
# 40Hz 制御 (25ms/フレーム) に対し、20ms でタイムアウト
MODEL_INFERENCE_TIMEOUT_SEC = 0.020

# フォールバック時の最大継続時間 [秒]
# この時間を超過するとシステムは安全停止に移行
FALLBACK_PP_TIMEOUT_SEC = 10.0

# 衝突復帰シーケンスの最大実行時間 [秒]
COLLISION_RECOVERY_TIMEOUT_SEC = 5.0

# 段階的降速スケジュール
# モデル失敗時、各段階 2秒間で速度を段階的に低減
# [100%, 80%, 50%, 20%, 0%]
FALLBACK_SPEED_SCHEDULE = [1.0, 0.8, 0.5, 0.2, 0.0]

# 降速の段階継続時間 [秒]
FALLBACK_SPEED_STAGE_DURATION_SEC = 2.0

# ============================================================
# Collision Recovery Sequence Parameters
# ============================================================

# バック走行時間 [秒]
COLLISION_BACKING_TIME_SEC = 1.5

# ステアリング回転時間 [秒]
COLLISION_TURNING_TIME_SEC = 2.0

# 復帰後の前進時間 [秒]
COLLISION_MOVING_FORWARD_TIME_SEC = 1.0

# 復帰時の速度制限 [m/s]
COLLISION_RECOVERY_MAX_SPEED = 0.2
```

---

## 2. `src/f1_env.py` への統合

### 2.1 初期化 (`__init__` メソッド)

`F1TenthRL.__init__` の最後に以下を追加：

```python
# ============================================================
# Safety Components Initialization
# ============================================================

if config.ENABLE_SAFETY_MANAGER:
    from .model_health import ModelHealthMonitor
    from .safety import SafetyManager
    from .collision_recovery import CollisionRecoverySequence
    
    self.model_monitor = ModelHealthMonitor(
        timeout_sec=config.MODEL_INFERENCE_TIMEOUT_SEC
    )
    
    self.collision_recovery = CollisionRecoverySequence(
        self.pp_controller,
        wheelbase=config.CAR_LENGTH * 0.7
    ) if config.ENABLE_COLLISION_RECOVERY else None
    
    self.safety_manager = SafetyManager(
        self.model_monitor,
        self.pp_controller,
        self.collision_recovery,
        fallback_timeout=config.FALLBACK_PP_TIMEOUT_SEC,
        collision_recovery_timeout=config.COLLISION_RECOVERY_TIMEOUT_SEC
    )
else:
    self.model_monitor = None
    self.safety_manager = None
    self.collision_recovery = None
```

### 2.2 Step メソッドの修正

`F1TenthRL.step()` メソッドを以下のように修正：

**変更前:**
```python
def step(self, action):
    # ... 既存コード ...
    obs, reward, done, info = self.env.step(...)
    return obs, reward, done, info
```

**変更後:**
```python
def step(self, action):
    """
    Step の実行
    
    フロー:
    1. モデル出力検証
    2. 衝突検知
    3. 状態更新
    4. アクション決定（安全性を考慮）
    5. シミュレータ実行
    """
    import time
    
    # ========================================
    # A. モデル推論結果の検証
    # ========================================
    if self.safety_manager:
        self.model_monitor.start_inference()
        model_healthy, error_info = self.model_monitor.end_inference(action)
    else:
        model_healthy = True
        error_info = {}
    
    # ========================================
    # B. 現在の観測と状態を取得
    # ========================================
    raw_obs = self.env.unwrapped.obs_dict(self.env.unwrapped.agents[0])
    x = raw_obs['poses_x'][0]
    y = raw_obs['poses_y'][0]
    yaw = raw_obs['poses_theta'][0]
    speed = raw_obs['linear_vels_x'][0]
    steering = raw_obs['steering_angle'][0]
    
    # ========================================
    # C. 安全管理による状態遷移と処理
    # ========================================
    current_time = time.time()
    
    if self.safety_manager:
        # 衝突検知（一度の step では collision がセットされる）
        # 次のステップで検知可能な状態にするため、前フレームの info を参照
        collision = getattr(self, '_last_collision', False)
        
        # 状態更新
        self.safety_manager.update_state(
            model_healthy,
            collision,
            x, y, yaw,
            current_time
        )
        
        # アクション決定
        robot_state = (x, y, yaw, speed)
        safe_action = self.safety_manager.get_action(
            action,
            robot_state,
            x, y, yaw,
            current_time
        )
        
        action_to_use = safe_action
    else:
        action_to_use = action
    
    # ========================================
    # D. シミュレータステップ実行
    # ========================================
    obs, reward, done, info = self.env.step(action_to_use)
    
    # ========================================
    # E. 衝突フラグの記録（次ステップ用）
    # ========================================
    collision_current = info.get('collision', False)
    self._last_collision = collision_current
    
    # ========================================
    # F. 安全情報をログに含める
    # ========================================
    if self.safety_manager:
        info['safety_state'] = self.safety_manager.state.name
        info['model_healthy'] = model_healthy
        info['model_error_info'] = error_info
        info['model_avg_inference_time'] = self.model_monitor.get_avg_inference_time()
    
    return obs, reward, done, info
```

### 2.3 Reset メソッドの修正

`F1TenthRL.reset()` メソッドにセーフティ初期化を追加：

```python
def reset(self):
    """
    エピソードリセット
    Safety Manager も同時にリセット
    """
    obs = self.env.reset()
    
    if self.safety_manager:
        self.safety_manager.reset()
    
    # 衝突フラグの初期化
    self._last_collision = False
    
    return obs
```

---

## 3. `src/f1_env.py` への追加メソッド（オプション）

### 状態取得ヘルパー

```python
def get_safety_state(self) -> dict:
    """
    現在のセーフティ状態を取得
    
    Returns:
        {
            'state': SafetyState.name,
            'model_healthy': bool,
            'speed_stage': int,
            'collision_recovery_active': bool
        }
    """
    if not self.safety_manager:
        return {}
    
    return {
        'state': self.safety_manager.state.name,
        'model_healthy': self.model_monitor.last_valid_action is not None,
        'speed_stage': self.safety_manager.speed_stage,
        'collision_recovery_active': (
            self.safety_manager.state.name.startswith('RECOVERY')
        ),
        'avg_inference_time': self.model_monitor.get_avg_inference_time()
    }
```

### ログ記録ヘルパー

```python
def log_safety_event(self, event_type: str, details: dict = None):
    """
    セーフティイベントをログファイルに記録
    
    Args:
        event_type: 'MODEL_TIMEOUT', 'COLLISION_DETECTED', 'FALLBACK_START' など
        details: イベント詳細
    """
    import json
    from pathlib import Path
    
    log_dir = Path('logs/safety_events')
    log_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    log_file = log_dir / f'safety_log_{timestamp}.jsonl'
    
    event = {
        'timestamp': time.time(),
        'event_type': event_type,
        'safety_state': self.safety_manager.state.name if self.safety_manager else 'N/A',
        'details': details or {}
    }
    
    with open(log_file, 'a') as f:
        f.write(json.dumps(event) + '\n')
```

---

## 4. テスト: `scripts/test_safety_system.py`

Phase 2 統合のテストを実施します。

### 4.1 基本的なテストスケルトン

```python
import numpy as np
import sys
from pathlib import Path

# パスを調整
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.f1_env import F1TenthRL
import time

def test_normal_operation():
    """通常動作: モデル推論が正常な場合"""
    print("Test 1: Normal Operation")
    env = F1TenthRL('my_maps/my_map')
    
    obs = env.reset()
    
    for step in range(100):
        # 正常なアクション
        action = np.array([0.1, 0.5], dtype=np.float32)
        obs, reward, done, info = env.step(action)
        
        # 検証
        assert info['safety_state'] == 'NORMAL', \
            f"Expected NORMAL, got {info['safety_state']}"
        assert info['model_healthy'] == True
        
        if done:
            break
    
    print("✓ Test 1 passed\n")

def test_model_timeout():
    """モデルタイムアウト時の PP フォールバック"""
    print("Test 2: Model Timeout → Fallback to Pure Pursuit")
    env = F1TenthRL('my_maps/my_map')
    obs = env.reset()
    
    timeout_detected = False
    fallback_detected = False
    
    for step in range(200):
        if step < 50:
            # 正常なアクション
            action = np.array([0.1, 0.5], dtype=np.float32)
        else:
            # タイムアウトを発生させる
            action = np.array([np.nan, 0.5], dtype=np.float32)
        
        obs, reward, done, info = env.step(action)
        
        # フォールバック検知
        if step > 50 and info['safety_state'] == 'FALLBACK_TO_PP':
            fallback_detected = True
            break
        
        if done:
            break
    
    assert fallback_detected, "Fallback was not triggered"
    print("✓ Test 2 passed\n")

def test_collision_recovery():
    """衝突復帰シーケンスの実行"""
    print("Test 3: Collision Recovery Sequence")
    env = F1TenthRL('my_maps/my_map')
    obs = env.reset()
    
    collision_detected = False
    recovery_started = False
    
    for step in range(500):
        action = np.array([0.5, 0.8], dtype=np.float32)  # 高速で走行
        obs, reward, done, info = env.step(action)
        
        # 衝突検知
        if info.get('collision', False):
            collision_detected = True
        
        # 復帰シーケンス検知
        if collision_detected and 'RECOVERY' in info['safety_state']:
            recovery_started = True
            print(f"  Recovery phase: {info['safety_state']}")
        
        if done:
            break
    
    if collision_detected:
        assert recovery_started, "Recovery sequence was not triggered"
        print("✓ Test 3 passed\n")
    else:
        print("⚠ Test 3 skipped: No collision in this run\n")

def test_fallback_timeout():
    """フォールバックタイムアウト時の停止"""
    print("Test 4: Fallback Timeout → Safe Stop")
    env = F1TenthRL('my_maps/my_map')
    
    # タイムアウト値を短く設定
    if env.safety_manager:
        env.safety_manager.fallback_timeout = 3.0
    
    obs = env.reset()
    start_time = time.time()
    safe_stop_reached = False
    
    for step in range(500):
        # 常にタイムアウトを発生
        action = np.array([np.nan, 0.5], dtype=np.float32)
        obs, reward, done, info = env.step(action)
        
        if info['safety_state'] == 'SAFE_STOP':
            safe_stop_reached = True
            elapsed = time.time() - start_time
            print(f"  Safe stop reached in {elapsed:.2f}s")
            break
        
        if done:
            break
    
    assert safe_stop_reached, "Safe stop was not reached"
    print("✓ Test 4 passed\n")

if __name__ == '__main__':
    print("=" * 60)
    print("Safety System Integration Tests")
    print("=" * 60 + "\n")
    
    try:
        test_normal_operation()
        test_model_timeout()
        test_collision_recovery()
        test_fallback_timeout()
        
        print("=" * 60)
        print("All tests passed! ✓")
        print("=" * 60)
    except AssertionError as e:
        print(f"Test failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
```

### 4.2 テスト実行

```bash
cd /home/toyot/projects/f1tenth-rl-project

# テストスクリプトを実行
python scripts/test_safety_system.py
```

期待出力：
```
============================================================
Safety System Integration Tests
============================================================

Test 1: Normal Operation
✓ Test 1 passed

Test 2: Model Timeout → Fallback to Pure Pursuit
✓ Test 2 passed

Test 3: Collision Recovery Sequence
  Recovery phase: COLLISION_DETECTED
  Recovery phase: RECOVERY_BACKING
  Recovery phase: RECOVERY_TURNING
  Recovery phase: RECOVERY_MOVING
✓ Test 3 passed

Test 4: Fallback Timeout → Safe Stop
  Safe stop reached in 3.12s
✓ Test 4 passed

============================================================
All tests passed! ✓
============================================================
```

---

## ✅ Phase 2 完了チェックリスト

- [ ] `src/config.py` に Safety 設定追加
- [ ] `src/f1_env.py` に Safety Manager 統合
- [ ] `src/f1_env.py` の `__init__`, `step`, `reset` を修正
- [ ] `scripts/test_safety_system.py` を実装
- [ ] 全テスト (test 1-4) 合格
- [ ] ログ出力で安全イベントが記録されることを確認

---

## 次のステップ

→ `docs/03_PHASE3_JETSON_ROS2.md` で Jetson/ROS2 実装に進む


# Phase 3: Jetson/ROS2 実装ガイド

Phase 2 まででシミュレーション側は完成。  
本フェーズでは Jetson 上で実機走行用の ROS2 ノードを実装します。

---

## 📌 主要実装ファイル

1. **`sharing/jetson_policy_node.py`** - ONNX 推論ノード
2. **`sharing/ros2_safety_node.py`** - 安全管理ノード
3. **`sharing/jetson_main.py`** - 統合実行スクリプト

---

## 1. Jetson Policy Node (`sharing/jetson_policy_node.py`)

### 目的

- Jetson 上で ONNX モデルを実行
- LiDAR データを受け取り、推論を実施
- 推論タイムアウト・エラー検出
- ヘルスチェック情報を発行

### 実装概要

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Float32
import onnxruntime as ort
import numpy as np
from collections import deque
import time
import os

class JetsonPolicyNode(Node):
    """
    Jetson 上の ONNX 推論ノード
    
    Subscribes:
        /scan: LiDAR スキャン (LaserScan)
    
    Publishes:
        /action: 推論出力アクション (Float32MultiArray)
        /model_health: ヘルスチェック情報 (Float32MultiArray)
        /inference_time: 推論実行時間 (Float32)
    """
    
    def __init__(self):
        super().__init__('jetson_policy_node')
        
        # ONNX モデルロード
        model_path = os.getenv('MODEL_PATH', 'models/best_model.onnx')
        self.get_logger().info(f"Loading ONNX model from: {model_path}")
        
        try:
            self.ort_session = ort.InferenceSession(
                model_path,
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )
            self.get_logger().info("✓ Model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise
        
        self.input_name = self.ort_session.get_inputs()[0].name
        self.output_name = self.ort_session.get_outputs()[0].name
        
        # タイムアウト設定
        self.timeout_sec = 0.018  # 18ms
        self.max_consecutive_failures = 5
        
        # 状態
        self.inference_times = deque(maxlen=20)
        self.failed_inferences = 0
        self.consecutive_failures = 0
        self.health_status = "HEALTHY"
        self.last_valid_action = np.array([0.0, 0.0], dtype=np.float32)
        
        # LiDAR バッファ（フレームスタック）
        self.lidar_buffer = deque(maxlen=4)
        self.lidar_scan_params = None
        
        # ROS2 インターフェース
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.action_pub = self.create_publisher(
            Float32MultiArray,
            '/action',
            10
        )
        
        self.health_pub = self.create_publisher(
            Float32MultiArray,
            '/model_health',
            10
        )
        
        self.inference_time_pub = self.create_publisher(
            Float32,
            '/inference_time',
            10
        )
        
        # 40Hz 推論タイマー (25ms 周期)
        self.timer = self.create_timer(0.025, self.inference_timer_callback)
        
        self.get_logger().info("JetsonPolicyNode initialized")
    
    def scan_callback(self, msg: LaserScan):
        """
        LiDAR スキャン受信・前処理
        
        処理内容:
        1. スキャン範囲を 270° に制限 (実機 Hokuyo URG)
        2. NaN/Inf クリーニング
        3. ダウンサンプリング (10 → 5)
        4. バッファに追加
        """
        # スキャン範囲を記録
        if self.lidar_scan_params is None:
            self.lidar_scan_params = {
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
            }
        
        try:
            # [180:1260] = 270°範囲の 1080 点
            scans_raw = np.array(msg.ranges[180:1260], dtype=np.float32)
            
            # クリーニング (NaN/Inf → 30.0)
            scans_clean = np.nan_to_num(
                scans_raw,
                nan=30.0,
                posinf=30.0,
                neginf=0.0
            )
            
            # クリップ [0, 30]
            scans_clipped = np.clip(scans_clean, 0.0, 30.0)
            
            # ダウンサンプリング: 1080 → 216点
            # (ダウンサンプリング係数=5: 1080/5=216)
            downsampled = scans_clipped.reshape(216, 5).min(axis=1)
            
            # バッファに追加
            self.lidar_buffer.append(downsampled)
            
        except Exception as e:
            self.get_logger().warn(f"Error processing LiDAR scan: {e}")
    
    def inference_timer_callback(self):
        """
        40Hz 推論タイマーコールバック
        
        フロー:
        1. フレームスタック確認
        2. ONNX 推論実行
        3. 結果検証
        4. パブリッシュ
        """
        # フレームスタック確認 (4フレーム = 100ms)
        if len(self.lidar_buffer) < 4:
            return
        
        try:
            # ========================================
            # 1. 入力準備: フレームスタック作成
            # ========================================
            stacked = np.concatenate(
                list(self.lidar_buffer),
                axis=0
            ).reshape(1, -1).astype(np.float32)
            
            # ========================================
            # 2. 推論実行（タイムアウト保護）
            # ========================================
            start_time = time.perf_counter()
            outputs = self.ort_session.run(
                None,
                {self.input_name: stacked}
            )
            elapsed = time.perf_counter() - start_time
            
            # 推論時間ログ
            self.inference_times.append(elapsed)
            
            # ========================================
            # 3. 出力検証
            # ========================================
            action = outputs[0][0].astype(np.float32)
            is_valid = self._validate_action(action, elapsed)
            
            if is_valid:
                self.last_valid_action = action.copy()
                self.consecutive_failures = 0
                self.health_status = "HEALTHY"
            else:
                self.consecutive_failures += 1
                self.failed_inferences += 1
                
                if self.consecutive_failures >= self.max_consecutive_failures:
                    self.health_status = "UNHEALTHY"
                
                # 前回の正常な値を再利用
                action = self.last_valid_action
            
            # ========================================
            # 4. パブリッシュ
            # ========================================
            self._publish_action(action)
            self._publish_health()
            self._publish_inference_time(elapsed)
            
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.health_status = "ERROR"
            self.consecutive_failures += 1
            self._publish_action(self.last_valid_action)
            self._publish_health()
    
    def _validate_action(self, action: np.ndarray, inference_time: float) -> bool:
        """アクション検証"""
        
        # タイムアウト判定
        if inference_time > self.timeout_sec:
            self.get_logger().warn(
                f"Inference timeout: {inference_time*1000:.2f}ms > {self.timeout_sec*1000:.2f}ms"
            )
            return False
        
        # NaN/Inf 判定
        if np.isnan(action).any():
            self.get_logger().warn("Action contains NaN")
            return False
        if np.isinf(action).any():
            self.get_logger().warn("Action contains Inf")
            return False
        
        # 範囲判定 [-1, 1]
        if (action < -1.0).any() or (action > 1.0).any():
            self.get_logger().warn(f"Action out of bounds: {action}")
            return False
        
        return True
    
    def _publish_action(self, action: np.ndarray):
        """推論アクション発行"""
        msg = Float32MultiArray()
        msg.data = action.tolist()
        self.action_pub.publish(msg)
    
    def _publish_health(self):
        """ヘルスチェック情報発行"""
        msg = Float32MultiArray()
        avg_inference_time = (
            np.mean(self.inference_times)
            if self.inference_times
            else 0.0
        )
        msg.data = [
            1.0 if self.health_status == "HEALTHY" else 0.0,
            avg_inference_time,
            float(self.consecutive_failures),
            float(self.failed_inferences)
        ]
        self.health_pub.publish(msg)
    
    def _publish_inference_time(self, elapsed: float):
        """推論実行時間発行"""
        msg = Float32(data=float(elapsed))
        self.inference_time_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JetsonPolicyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 2. ROS2 Safety Node (`sharing/ros2_safety_node.py`)

### 目的

- Policy Node からのアクションを監視
- ヘルスチェック情報を受け取り、状態遷移を判定
- Pure Pursuit フォールバック・衝突復帰を実行
- 最終的なアクションを `/safe_action` で発行

### 実装概要

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from collections import deque
import time
import os
import sys

# 一つ上のディレクトリから import
sys.path.insert(0, os.path.dirname(__file__))

class ROS2SafetyNode(Node):
    """
    Jetson 上の安全管理ノード
    
    Subscribes:
        /action: ONNX モデル出力 (Float32MultiArray)
        /model_health: ヘルスチェック情報 (Float32MultiArray)
        /emergency_stop: 緊急停止フラグ (Bool)
        /odometry/filtered: ロボット姿勢 (nav_msgs/Odometry)
    
    Publishes:
        /safe_action: 確定アクション (Float32MultiArray)
        /safety_state: 安全状態 (Float32MultiArray)
    """
    
    def __init__(self):
        super().__init__('ros2_safety_node')
        
        # 状態管理
        self.state = "NORMAL"  # NORMAL, FALLBACK, COLLISION_RECOVERY, SAFE_STOP
        self.model_healthy = True
        self.collision_detected = False
        self.fallback_start_time = None
        self.recovery_start_time = None
        self.recovery_phase = None
        
        # タイムアウト設定
        self.fallback_timeout = 10.0
        self.collision_recovery_timeout = 5.0
        
        # Pure Pursuit 初期化
        self._init_pure_pursuit()
        
        # 衝突復帰初期化
        self._init_collision_recovery()
        
        # バッファ
        self.last_model_action = np.array([0.0, 0.0], dtype=np.float32)
        self.last_valid_action = np.array([0.0, 0.0], dtype=np.float32)
        
        # 段階的降速
        self.speed_schedule = [1.0, 0.8, 0.5, 0.2, 0.0]
        self.speed_stage = 0
        self.speed_stage_time = None
        
        # 現在の位置・姿勢
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # ROS2 インターフェース
        self.action_sub = self.create_subscription(
            Float32MultiArray,
            '/action',
            self.action_callback,
            10
        )
        
        self.health_sub = self.create_subscription(
            Float32MultiArray,
            '/model_health',
            self.health_callback,
            10
        )
        
        self.emergency_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_callback,
            10
        )
        
        self.safe_action_pub = self.create_publisher(
            Float32MultiArray,
            '/safe_action',
            10
        )
        
        self.safety_state_pub = self.create_publisher(
            Float32MultiArray,
            '/safety_state',
            10
        )
        
        # 監視タイマー (10Hz)
        self.timer = self.create_timer(0.1, self.safety_timer_callback)
        
        self.get_logger().info("ROS2SafetyNode initialized")
    
    def _init_pure_pursuit(self):
        """Pure Pursuit コントローラ初期化"""
        try:
            from src.controllers.pure_pursuit import PurePursuitController
            from src.racing_line import RacingLine
            
            # レーシングライン読み込み
            racing_line_path = os.getenv(
                'RACING_LINE_PATH',
                'my_maps/my_map.csv'
            )
            
            self.racing_line = RacingLine(racing_line_path)
            
            # Pure Pursuit コントローラ
            self.pp_controller = PurePursuitController(
                self.racing_line,
                wheelbase=0.33,
                lookahead_dist=0.6
            )
            
            self.get_logger().info("✓ Pure Pursuit initialized")
        except Exception as e:
            self.get_logger().warn(f"Pure Pursuit init failed: {e}")
            self.pp_controller = None
    
    def _init_collision_recovery(self):
        """衝突復帰シーケンス初期化"""
        try:
            from src.collision_recovery import CollisionRecoverySequence
            
            self.collision_recovery = CollisionRecoverySequence(self.pp_controller)
            self.get_logger().info("✓ Collision recovery initialized")
        except Exception as e:
            self.get_logger().warn(f"Collision recovery init failed: {e}")
            self.collision_recovery = None
    
    def action_callback(self, msg: Float32MultiArray):
        """モデル出力アクション受信"""
        self.last_model_action = np.array(msg.data, dtype=np.float32)
    
    def health_callback(self, msg: Float32MultiArray):
        """モデルヘルスチェック受信"""
        health_ok = msg.data[0] > 0.5
        avg_inference_time = msg.data[1]
        consecutive_failures = int(msg.data[2])
        
        self.model_healthy = health_ok
        
        if health_ok:
            self.last_valid_action = self.last_model_action
        
        # ログ
        if consecutive_failures > 0:
            self.get_logger().info(
                f"Model health: {'OK' if health_ok else 'NG'}, "
                f"avg_time={avg_inference_time*1000:.2f}ms, "
                f"consecutive_failures={consecutive_failures}"
            )
    
    def emergency_callback(self, msg: Bool):
        """緊急停止トピック受信"""
        if msg.data:
            self.state = "SAFE_STOP"
            self.get_logger().error("Emergency stop activated!")
    
    def safety_timer_callback(self):
        """安全管理タイマー (10Hz)"""
        current_time = time.time()
        
        # ========================================
        # 状態遷移
        # ========================================
        self._update_state(current_time)
        
        # ========================================
        # アクション決定
        # ========================================
        safe_action = self._compute_action(current_time)
        
        # ========================================
        # パブリッシュ
        # ========================================
        self._publish_safe_action(safe_action)
        self._publish_safety_state(current_time)
    
    def _update_state(self, current_time: float):
        """状態遷移更新"""
        
        # NORMAL → 他の状態へ
        if self.state == "NORMAL":
            if not self.model_healthy:
                self.state = "FALLBACK"
                self.fallback_start_time = current_time
                self.speed_stage = 0
                self.speed_stage_time = current_time
                self.get_logger().info("State → FALLBACK")
        
        # FALLBACK 内での処理
        elif self.state == "FALLBACK":
            elapsed = current_time - self.fallback_start_time
            
            if self.model_healthy:
                # モデルが復帰
                self.state = "NORMAL"
                self.speed_stage = 0
                self.get_logger().info("State → NORMAL (model recovered)")
            
            elif elapsed > self.fallback_timeout:
                # タイムアウト
                self.state = "SAFE_STOP"
                self.get_logger().error("State → SAFE_STOP (fallback timeout)")
            
            # 段階的降速の進行 (2秒ごと)
            stage_elapsed = current_time - self.speed_stage_time
            if stage_elapsed > 2.0:
                if self.speed_stage < len(self.speed_schedule) - 1:
                    self.speed_stage += 1
                    self.speed_stage_time = current_time
                    self.get_logger().info(
                        f"Speed stage → {self.speed_stage} "
                        f"({self.speed_schedule[self.speed_stage]*100:.0f}%)"
                    )
    
    def _compute_action(self, current_time: float) -> np.ndarray:
        """状態に応じたアクション計算"""
        
        if self.state == "NORMAL":
            return self.last_model_action
        
        elif self.state == "FALLBACK":
            # Pure Pursuit + 段階的降速
            if self.pp_controller:
                try:
                    pp_action = self.pp_controller.compute_action(
                        (self.current_x, self.current_y, self.current_yaw, 0.0)
                    )
                    speed_factor = self.speed_schedule[
                        min(self.speed_stage, len(self.speed_schedule) - 1)
                    ]
                    return np.array(
                        [pp_action[0], pp_action[1] * speed_factor],
                        dtype=np.float32
                    )
                except Exception as e:
                    self.get_logger().warn(f"PP computation failed: {e}")
                    return np.array([0.0, 0.2], dtype=np.float32)  # Safe default
            else:
                return self.last_valid_action
        
        elif self.state == "SAFE_STOP":
            return np.array([0.0, 0.0], dtype=np.float32)
        
        return np.array([0.0, 0.0], dtype=np.float32)
    
    def _publish_safe_action(self, action: np.ndarray):
        """確定アクション発行"""
        msg = Float32MultiArray()
        msg.data = action.tolist()
        self.safe_action_pub.publish(msg)
    
    def _publish_safety_state(self, current_time: float):
        """安全状態発行"""
        msg = Float32MultiArray()
        msg.data = [
            1.0 if self.state == "NORMAL" else 0.0,
            float(self.model_healthy),
            float(self.speed_stage),
        ]
        self.safety_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ROS2SafetyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 3. Jetson 統合メインプログラム (`sharing/jetson_main.py`)

### 目的

両ノードを一括で起動・管理

```python
#!/usr/bin/env python3
"""
Jetson メイン実行スクリプト

使用方法:
    python jetson_main.py

環境変数:
    MODEL_PATH: ONNX モデルのパス
    RACING_LINE_PATH: レーシングラインの CSV ファイルパス
"""

import subprocess
import signal
import sys
import os
import time

def run_nodes():
    """ROS2 ノードを起動"""
    
    # 環境変数設定
    os.environ.setdefault('MODEL_PATH', 'models/best_model.onnx')
    os.environ.setdefault('RACING_LINE_PATH', 'my_maps/my_map.csv')
    
    print("=" * 60)
    print("Jetson F1Tenth Policy Executor")
    print("=" * 60)
    print(f"Model: {os.environ['MODEL_PATH']}")
    print(f"Racing line: {os.environ['RACING_LINE_PATH']}")
    print("=" * 60 + "\n")
    
    # ノードプロセス管理
    processes = []
    
    try:
        # Policy Node 起動
        print("[1/2] Starting Jetson Policy Node...")
        policy_proc = subprocess.Popen(
            [sys.executable, '-m', 'sharing.jetson_policy_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        processes.append(('jetson_policy_node', policy_proc))
        time.sleep(1)  # 起動待機
        
        # Safety Node 起動
        print("[2/2] Starting ROS2 Safety Node...")
        safety_proc = subprocess.Popen(
            [sys.executable, '-m', 'sharing.ros2_safety_node'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        processes.append(('ros2_safety_node', safety_proc))
        
        print("\n✓ All nodes started successfully\n")
        print("Press Ctrl+C to stop...")
        
        # ノード実行監視
        while True:
            for name, proc in processes:
                if proc.poll() is not None:
                    # プロセスが終了した
                    print(f"\n⚠ {name} terminated unexpectedly")
                    raise RuntimeError(f"{name} crashed")
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    
    finally:
        # クリーンアップ
        for name, proc in processes:
            if proc.poll() is None:
                print(f"Stopping {name}...")
                proc.terminate()
                proc.wait(timeout=5)
        
        print("✓ All nodes stopped")

if __name__ == '__main__':
    run_nodes()
```

---

## ✅ Phase 3 完了チェックリスト

- [ ] `sharing/jetson_policy_node.py` 実装完了
- [ ] `sharing/ros2_safety_node.py` 実装完了
- [ ] `sharing/jetson_main.py` 実装完了
- [ ] ONNX モデルを Jetson で実行可能か確認 (< 18ms)
- [ ] ROS2 トピックの通信が正常か確認
- [ ] Pure Pursuit コントローラが Jetson で動作可能か確認

---

## 次のステップ

→ `docs/04_TESTING_GUIDE.md` でテスト手順を確認  
→ `docs/05_JETSON_SETUP.md` で Jetson 環境セットアップを参照


# テスト実施ガイド

本ガイドではシミュレーション環境と Jetson 実機環境でのテスト方法を説明します。

---

## 🔄 テスト順序

```
1. Phase 1 単体テスト (コンポーネント各 3-4 時間)
2. Phase 2 シミュレーション統合テスト (4 時間)
3. Phase 3 Jetson 環境テスト (2-3 時間)
4. 本番走行テスト (別途スケジュール)
```

---

## Phase 1: コンポーネント単体テスト

### 1.1 ModelHealthMonitor テスト

```bash
cd /home/toyot/projects/f1tenth-rl-project

# テストスクリプト
python -c "
from src.model_health import ModelHealthMonitor
import numpy as np
import time

monitor = ModelHealthMonitor(timeout_sec=0.020)

# Test 1: 正常な推論
print('Test 1: Normal inference')
monitor.start_inference()
time.sleep(0.010)
action = np.array([0.1, 0.5])
healthy, errors = monitor.end_inference(action)
assert healthy == True, 'Expected healthy'
print('✓ Passed')

# Test 2: タイムアウト
print('Test 2: Timeout')
monitor.start_inference()
time.sleep(0.025)
action = np.array([0.1, 0.5])
healthy, errors = monitor.end_inference(action)
assert healthy == False, 'Expected unhealthy'
assert 'timeout' in errors, 'Expected timeout error'
print('✓ Passed')

# Test 3: NaN 検出
print('Test 3: NaN detection')
monitor.start_inference()
time.sleep(0.010)
action = np.array([np.nan, 0.5])
healthy, errors = monitor.end_inference(action)
assert healthy == False, 'Expected unhealthy'
assert 'nan' in errors, 'Expected NaN error'
print('✓ Passed')

print('\\nAll ModelHealthMonitor tests passed ✓')
"
```

### 1.2 SafetyManager テスト

```bash
python -c "
from src.safety import SafetyManager, SafetyState
from src.model_health import ModelHealthMonitor
import numpy as np

# Mock コンポーネント
class MockPP:
    def compute_action(self, state):
        return (0.1, 0.5)

class MockCollisionRecovery:
    def __init__(self):
        self.state = 'IDLE'
    def start_recovery(self, x, y, yaw):
        self.state = 'RECOVERING'
    def compute_recovery_action(self, x, y, yaw, t):
        return (0.0, -0.3) if self.state == 'RECOVERING' else None
    def is_completed(self):
        return False
    def reset(self):
        self.state = 'IDLE'

model_monitor = ModelHealthMonitor()
pp = MockPP()
recovery = MockCollisionRecovery()
safety = SafetyManager(model_monitor, pp, recovery)

# Test: NORMAL → FALLBACK
print('Test: State transition NORMAL → FALLBACK')
assert safety.state == SafetyState.NORMAL
safety.update_state(False, False, 0, 0, 0, 0)
assert safety.state == SafetyState.FALLBACK_TO_PP
print('✓ Passed')

print('\\nAll SafetyManager tests passed ✓')
"
```

---

## Phase 2: シミュレーション統合テスト

### 2.1 テスト実行

```bash
cd /home/toyot/projects/f1tenth-rl-project

# 統合テストスクリプト実行
python scripts/test_safety_system.py
```

### 2.2 期待される出力

```
============================================================
Safety System Integration Tests
============================================================

Test 1: Normal Operation
✓ Test 1 passed

Test 2: Model Timeout → Fallback to Pure Pursuit
✓ Test 2 passed

Test 3: Collision Recovery Sequence
  Recovery phase: COLLISION_DETECTED
  Recovery phase: RECOVERY_BACKING
  Recovery phase: RECOVERY_TURNING
  Recovery phase: RECOVERY_MOVING
✓ Test 3 passed

Test 4: Fallback Timeout → Safe Stop
  Safe stop reached in 3.12s
✓ Test 4 passed

============================================================
All tests passed! ✓
============================================================
```

### 2.3 結果の確認ポイント

| 項目 | 期待値 | 検証方法 |
|:---:|:---:|:---|
| **正常走行** | 完走率 > 90% | Test 1 の 100 ステップ中に done に至るかを確認 |
| **PP フォールバック** | 無限ループしない | Test 2 で 200 ステップ以内に FALLBACK 検知 |
| **衝突復帰** | 少なくとも 1 回の復帰 | Test 3 で 500 ステップ内に衝突が発生して復帰シーケンス開始 |
| **段階的降速** | 10秒以内に停止 | Test 4 で FALLBACK → SAFE_STOP の遷移時間が 3-4 秒 |

---

## Phase 3: Jetson 環境テスト

### 3.1 Jetson 環境セットアップ確認

```bash
# Jetson で実行
cd /home/toyot/projects/f1tenth-rl-project

# 環境確認
python -c "
import onnxruntime as ort
print('ONNX Runtime available:', hasattr(ort, 'InferenceSession'))

import rclpy
print('ROS2 available:', hasattr(rclpy, 'init'))

import numpy as np
print('NumPy version:', np.__version__)
"
```

### 3.2 ONNX 推論速度テスト

```bash
python -c "
import onnxruntime as ort
import numpy as np
import time

print('Testing ONNX inference speed...')

# モデルロード
session = ort.InferenceSession(
    'models/best_model.onnx',
    providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
)

input_name = session.get_inputs()[0].name
dummy_input = np.random.randn(1, 864).astype(np.float32)

# 30回の推論時間を測定
times = []
for i in range(30):
    start = time.perf_counter()
    session.run(None, {input_name: dummy_input})
    times.append((time.perf_counter() - start) * 1000)  # ms に変換

avg_time = np.mean(times)
max_time = np.max(times)
min_time = np.min(times)

print(f'Inference time: {avg_time:.2f}ms (min: {min_time:.2f}ms, max: {max_time:.2f}ms)')

if avg_time < 18:
    print('✓ Inference speed acceptable (< 18ms)')
else:
    print('⚠ Warning: Inference speed may be slow')
"
```

### 3.3 ROS2 ノード実行テスト

```bash
# Terminal 1: Policy Node 起動
ros2 run f1tenth_rl jetson_policy_node

# Terminal 2: Safety Node 起動
ros2 run f1tenth_rl ros2_safety_node

# Terminal 3: 監視スクリプト実行
python sharing/test_jetson_integration.py
```

### 3.4 トピック通信確認

```bash
# Terminal A: Policy Node トピック確認
ros2 topic list
ros2 topic echo /action

# Terminal B: Safety Node トピック確認
ros2 topic echo /safe_action
ros2 topic echo /model_health
```

期待される出力：
```
publisher: /jetson_policy_node
  subscriber: /ros2_safety_node

---
[0.15, 0.45]
[0.15, 0.47]
[0.16, 0.46]
...
```

### 3.5 ダミーセンサデータ送信テスト

```python
# dummy_scan_publisher.py
import rclpy
from sensor_msgs.msg import LaserScan
import numpy as np

def main():
    rclpy.init()
    node = rclpy.create_node('dummy_scan_publisher')
    pub = node.create_publisher(LaserScan, '/scan', 10)
    
    # ダミースキャンデータ
    scan_msg = LaserScan()
    scan_msg.header.frame_id = 'laser'
    scan_msg.angle_min = -2.35  # -135°
    scan_msg.angle_max = 2.35   # +135°
    scan_msg.angle_increment = 0.00436  # 1440ビーム
    scan_msg.ranges = [1.0] * 1440  # 1m の距離
    
    for i in range(100):
        scan_msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(scan_msg)
        node.get_logger().info(f'Published scan {i}')
        rclpy.spin_once(node, timeout_sec=0.05)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
python dummy_scan_publisher.py
```

---

## 本番走行前の最終チェック

### ✅ シミュレーション側

- [ ] Phase 1-3 のテスト全て PASS
- [ ] 安全設定 (`config.py`) が正しく読み込まれている
- [ ] ログファイルが `logs/safety_events/` に出力されている

### ✅ Jetson 側

- [ ] ONNX 推論速度が 18ms 以下
- [ ] ROS2 ノード起動時にエラーが出ていない
- [ ] `/action`, `/safe_action`, `/model_health` トピックが通信できている
- [ ] 緊急停止トピック (`/emergency_stop`) が機能している
- [ ] 衝突復帰に使用する `my_maps/my_map.csv` が存在する

### ✅ ハードウェア側

- [ ] Jetson の温度が正常範囲 (< 70°C)
- [ ] LiDAR が正常に動作している (スキャン頻度 40Hz)
- [ ] VESC モーター制御が正常に応答している
- [ ] バッテリー電圧が安定している

---

## トラブルシューティング

### ONNX 推論が遅い場合

```bash
# CUDAExecutionProvider が使用されているか確認
python -c "
import onnxruntime as ort
session = ort.InferenceSession('models/best_model.onnx')
print('Providers:', session.get_providers())
"
```

期待: `['CUDAExecutionProvider', 'CPUExecutionProvider']`

**対策**: CUDA ドライバ・cuDNN を確認

### ROS2 トピックが通信されない場合

```bash
# ノード間の接続確認
ros2 node list
ros2 node info /jetson_policy_node
ros2 node info /ros2_safety_node

# トピック確認
ros2 topic list
ros2 topic info /action
```

**対策**: ファイアウォール設定・ネットワーク接続を確認

### モデルエラー「無効な入力形状」

```python
# 入力形状確認
import onnxruntime as ort
session = ort.InferenceSession('models/best_model.onnx')
for inp in session.get_inputs():
    print(f"Input: {inp.name}, Shape: {inp.shape}")
```

**対策**: LiDAR スタック方法がモデルの入力形状と一致しているか確認

---

## 次のステップ

→ [本番走行](06_TROUBLESHOOTING.md) に向けてのチェックリスト確認


# Jetson セットアップガイド

本番走行前に Jetson 環境を整備するためのステップバイステップガイドです。

---

## 前提条件

- Jetson Nano / Xavier / Orin がセットアップ済み
- Ubuntu 20.04 / 22.04 がインストール済み
- ROS2 Foxy / Humble がセットアップ済み

---

## 📋 セットアップ手順

### Step 1: 基本パッケージのインストール

```bash
sudo apt-get update
sudo apt-get upgrade -y

# Python 3.8 以上が必要
sudo apt-get install -y python3-pip python3-dev

# ONNX Runtime インストール
# (GPU サポート)
pip3 install onnxruntime-gpu

# または (CPU のみ)
pip3 install onnxruntime

# ROS2 関連
sudo apt-get install -y python3-colcon-common-extensions
pip3 install rclpy
```

### Step 2: プロジェクトコードの配置

```bash
# Jetson 上の作業ディレクトリ
mkdir -p ~/f1tenth_ws
cd ~/f1tenth_ws

# プロジェクトのコピー
# (GitHub から clone または scp で転送)
git clone <your-repo-url>
cd f1tenth-rl-project

# 仮想環境作成 (オプション)
python3 -m venv venv
source venv/bin/activate

# 依存関係インストール
pip3 install -r requirements.txt
```

### Step 3: ONNX モデルの配置

```bash
# モデルファイルが存在することを確認
ls -la models/

# 出力例:
# -rw-r--r-- 1 user user 12345678 May 22 12:34 best_model.onnx
# -rw-r--r-- 1 user user 23456789 May 22 12:34 ppo_model_exp39.onnx
```

### Step 4: レーシングラインデータの配置

```bash
# コース情報 (CSV ファイル)
ls -la my_maps/

# 出力例:
# -rw-r--r-- 1 user user 98765   May 22 12:34 my_map.csv
# -rw-r--r-- 1 user user 123456  May 22 12:34 my_map.pgm
# -rw-r--r-- 1 user user 567     May 22 12:34 my_map.yaml
```

### Step 5: ROS2 ワークスペース構築

```bash
cd ~/f1tenth_ws

# ワークスペース初期化 (必要に応じて)
colcon build --symlink-install

# または単にパスに追加
export PYTHONPATH=$PYTHONPATH:$(pwd)
```

### Step 6: 環境変数設定

```bash
# ~/.bashrc に追加
cat >> ~/.bashrc << 'EOF'

# F1Tenth ROS2 Settings
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Jetson F1Tenth Project
export PYTHONPATH=$PYTHONPATH:$HOME/f1tenth_ws/f1tenth-rl-project
export MODEL_PATH=$HOME/f1tenth_ws/f1tenth-rl-project/models/best_model.onnx
export RACING_LINE_PATH=$HOME/f1tenth_ws/f1tenth-rl-project/my_maps/my_map.csv
EOF

source ~/.bashrc
```

### Step 7: 接続の確認

```bash
# LiDAR デバイスの確認
ls -la /dev/ttyUSB*
# または
ros2 topic list | grep scan

# VESC モーター制御の確認
ros2 topic list | grep /drive
```

### Step 8: 動作テスト

```bash
# ノード起動テスト
python3 sharing/jetson_main.py

# 別ターミナルでトピック監視
ros2 topic list
ros2 topic echo /action  # モデル出力を確認
```

---

## 🔧 詳細設定

### ONNX Runtime の GPU サポート確認

```bash
python3 -c "
import onnxruntime as ort

print('Available Execution Providers:')
for provider in ort.get_available_providers():
    print(f'  - {provider}')

# GPU が利用可能な場合
if 'CUDAExecutionProvider' in ort.get_available_providers():
    print('✓ GPU support enabled')
else:
    print('⚠ GPU support disabled (using CPU)')
"
```

**結果の例:**
```
Available Execution Providers:
  - CUDAExecutionProvider
  - CPUExecutionProvider
✓ GPU support enabled
```

### 電力管理 (Jetson Nano の場合)

```bash
# Jetson Nano では電力消費が大きいため、モード設定が重要
sudo jetson_clocks --show
sudo nvpmodel -m 0  # 最大パフォーマンスモード
```

### 熱対策

```bash
# 温度監視
watch -n 1 nvidia-smi

# 目安:
# - < 60°C: 安全
# - 60-70°C: 警告
# - > 70°C: 危険 (スロットル開始)
```

---

## 📝 設定ファイル例

### `/etc/systemd/system/f1tenth-policy.service`

自動起動サービス（オプション）:

```ini
[Unit]
Description=F1Tenth Policy Executor
After=network-online.target ros2.service

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/f1tenth_ws/f1tenth-rl-project
ExecStart=/usr/bin/python3 sharing/jetson_main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

起動/停止:
```bash
sudo systemctl start f1tenth-policy
sudo systemctl status f1tenth-policy
sudo systemctl stop f1tenth-policy

# ログ確認
sudo journalctl -u f1tenth-policy -f
```

---

## 🚨 緊急停止ボタン設定

ROS2 から緊急停止トピック (`/emergency_stop`) を発行するシンプルなスクリプト:

```python
# emergency_stop_publisher.py
import rclpy
from std_msgs.msg import Bool
import sys

def main():
    rclpy.init()
    node = rclpy.create_node('emergency_stop_publisher')
    pub = node.create_publisher(Bool, '/emergency_stop', 10)
    
    # 確認メッセージ
    print("Emergency Stop Publisher")
    print("Press Enter to publish emergency stop...")
    input()
    
    msg = Bool(data=True)
    pub.publish(msg)
    print("✓ Emergency stop published")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
python3 emergency_stop_publisher.py
```

---

## ✅ セットアップチェックリスト

実機走行前に確認してください：

- [ ] ONNX Runtime が GPU サポート対応で動作
- [ ] ROS2 ノード (Policy + Safety) が起動可能
- [ ] `/scan` トピックが LiDAR から流れている
- [ ] `/action` トピックが推論出力を出力している
- [ ] `/safe_action` トピックが安全制御出力を出力している
- [ ] モデル推論時間が 18ms 以下
- [ ] Jetson 温度が 60°C 以下
- [ ] 緊急停止トピック (`/emergency_stop`) が機能している
- [ ] バッテリー電圧が安定している (表示値で確認)
- [ ] モーター制御 (VESC) が反応している

---

## 🐛 トラブルシューティング

### 問題 1: ONNX Runtime インストール失敗

```bash
# GPU なしでも動作するように
pip3 install onnxruntime

# CPU 版で動作確認後、GPU 版をインストール
pip3 install onnxruntime-gpu
```

### 問題 2: ROS2 トピック通信なし

```bash
# domain ID を確認
echo $ROS_DOMAIN_ID

# 同じ domain ID で起動
export ROS_DOMAIN_ID=0
python3 sharing/jetson_main.py
```

### 問題 3: メモリ不足エラー

```bash
# 利用可能メモリ確認
free -h

# プロセス監視
ps aux | grep python

# メモリ使用量が多い場合、モデルを軽量化するか、
# フレームスタック数を削減 (config.py の FRAME_STACK)
```

### 問題 4: モデルの入力形状エラー

```bash
# 入力形状を確認
python3 -c "
from src.config import config
from src.racing_line import RacingLine

print(f'LiDAR size: {1080 // config.LIDAR_DOWNSAMPLE_FACTOR}')
print(f'Racing line size: {RacingLine.NUM_FEATURES if config.INCLUDE_RACING_LINE else 0}')
print(f'Expected total: {1080 // config.LIDAR_DOWNSAMPLE_FACTOR + RacingLine.NUM_FEATURES}')

# ONNX モデルの入力形状
import onnxruntime as ort
session = ort.InferenceSession('models/best_model.onnx')
for inp in session.get_inputs():
    print(f'Model input shape: {inp.shape}')
"
```

---

## 次のステップ

→ [テスト実施](04_TESTING_GUIDE.md) に進む  
→ [トラブルシューティング](06_TROUBLESHOOTING.md) を参照


# トラブルシューティング & FAQ

本番走行中に遭遇する可能性がある問題と対処法をまとめています。

---

## 実機走行時のよくある問題

### 🔴 モデル推論タイムアウト

**症状:**  
- ログに「Inference timeout: 25.3ms > 20ms」と出力
- `/model_health` トピックで `health=0`
- 車が Pure Pursuit に自動切り替わる

**原因:**
1. Jetson 温度が高い (> 70°C)
2. 他のプロセスが CPU/GPU リソースを消費している
3. ONNX Runtime が CPU フォールバックしている

**対処:**
```bash
# 1. 温度確認
nvidia-smi
watch -n 1 nvidia-smi  # リアルタイム監視

# 2. プロセス確認
top
ps aux | sort -k3 -r | head -20

# 3. GPU 状態確認
nvidia-smi -pm 1
nvidia-smi -pm 0

# 4. ONNX Runtime の確認
python3 -c "
import onnxruntime as ort
providers = ort.get_available_providers()
print('Providers:', providers)
# CUDAExecutionProvider が最初に来ていればOK
"
```

**解決策:**
- Jetson の冷却ファンを増強
- 背景プロセスを終了
- CPU/GPU クロック周波数を確認
- 必要に応じてモデルを軽量化

---

### 🔴 衝突復帰の失敗 (無限ループ)

**症状:**
- 衝突後、BACKING → TURNING → MOVING_FORWARD のシーケンスが無限ループ
- 車が同じ場所で回転し続ける

**原因:**
1. レーシングラインのデータが不正確
2. 位置推定 (odometry) がずれている
3. 復帰シーケンスのタイミング設定が適切でない

**対処:**
```python
# collision_recovery.py のタイムアウト値を短くする
BACKING_TIME = 1.0         # 1.5秒 → 1.0秒
TURNING_TIME = 1.5         # 2.0秒 → 1.5秒
MOVING_FORWARD_TIME = 0.8  # 1.0秒 → 0.8秒

# または、復帰シーケンス開始前に停止時間を入れる
if collision:
    self.state = "STOP_AND_ASSESS"
    # 0.5秒停止して状態を確認
    time.sleep(0.5)
```

**確認方法:**
```bash
# ログから復帰シーケンスの詳細を確認
tail -f logs/safety_events/safety_log_*.jsonl | grep RECOVERY

# 位置推定の精度を確認
ros2 topic echo /odometry/filtered
```

---

### 🔴 Pure Pursuit フォールバック後の無限降速

**症状:**
- モデル失敗後、速度が 1.0 → 0.8 → 0.5 → 0.2 → 0.0 と段階的に低下
- 10秒後に完全停止して復帰しない

**原因:**
- モデルが復帰していない（持続的な推論エラー）
- 推論タイムアウトが継続している

**確認:**
```bash
# ヘルスチェック情報を監視
ros2 topic echo /model_health

# 出力例:
# data: [0.0, 0.025, 5, 50]  # health=0(不健康), time=25ms, failures=5, total=50
```

**対処:**
1. モデルの異常を特定してログから確認
2. 推論タイムアウトが解消されるまで待つ
3. 必要に応じて Jetson を再起動

```python
# config.py の FALLBACK_PP_TIMEOUT_SEC を調整
# デフォルト: 10秒 → 5秒に短縮して素早く停止
FALLBACK_PP_TIMEOUT_SEC = 5.0
```

---

### 🟡 LiDAR ノイズによる不安定な走行

**症状:**
- 障害物がないのに突然減速
- 走行ルートが左右にぐらぐらしている

**原因:**
1. LiDAR が反射の多い環境 (ガラス壁、金属) で反応
2. LiDAR 自体のノイズ・異常値

**対処:**
```python
# src/f1_env.py の LiDAR クリーニング強化
scans_clean = np.nan_to_num(
    scans_raw,
    nan=30.0,
    posinf=30.0,
    neginf=0.0
)

# ノイズフィルター追加
scans_filtered = gaussian_filter1d(scans_clean, sigma=1.0)  # 平滑化
```

**確認：**
```bash
# 生の LiDAR データを確認
ros2 topic echo /scan | head -20

# 異常な値 (inf, -inf, 0 に近い値) が多い場合は LiDAR を疑う
```

---

### 🟡 ROS2 トピック通信遅延

**症状:**
- `/safe_action` の更新が遅い (100ms 以上の遅延)
- モーターの応答に遅れが生じる

**原因:**
1. ネットワークが混雑している
2. ROS2 の QoS 設定が不適切
3. ノードのスピナーが追い付いていない

**対処:**
```python
# sharing/ros2_safety_node.py の QoS 設定を改善
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # UDP ライク
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

self.safe_action_pub = self.create_publisher(
    Float32MultiArray,
    '/safe_action',
    qos
)
```

**確認：**
```bash
# トピック遅延の測定
ros2 topic hz /safe_action
# 40Hz (25ms 周期) より遅い場合は要改善
```

---

## よくある質問 (FAQ)

### Q1: 実機走行でモデルが使われているのか確認したい

**A:**
```bash
# `/model_health` トピックを監視
ros2 topic echo /model_health

# 出力が変わり続ければモデルが推論中
# data: [1.0, 0.015, 0, ...]  # health=1(健康), time=15ms
```

または

```python
# ログファイルから推論の履歴を確認
import json
for line in open('logs/safety_events/safety_log_*.jsonl'):
    event = json.loads(line)
    if event['event_type'] == 'MODEL_INFERENCE':
        print(event)
```

---

### Q2: フォールバック (Pure Pursuit) のみで走行させたい

**A:**

`src/config.py` でモデルを無効化：

```python
# モデルを使用しない設定に切り替え
ENABLE_SAFETY_MANAGER = True

# 常に Pure Pursuit で走行させるため、
# 意図的にモデル推論をスキップ

# 別案: 学習済みモデルを使わず、Pure Pursuit 直結で走行
# src/f1_env.py の step() で:
action = self.pp_controller.compute_action(robot_state)
```

---

### Q3: 衝突復帰のバック走行がうまくいかない

**A:**

バック走行時の速度・時間を調整：

```python
# src/collision_recovery.py
if self.state == "BACKING":
    if phase_elapsed < 1.5:
        # 速度を上げる
        return (0.0, -0.5)  # -0.3 から -0.5 に変更
    else:
        self.state = "TURNING"
```

または、バック走行を省略して回転から開始：

```python
def start_recovery(self, x, y, yaw):
    # BACKING をスキップして TURNING から開始
    self.state = "TURNING"
    self.phase_start_time = time.time()
```

---

### Q4: 段階的降速の速度を変更したい

**A:**

`src/config.py` で `FALLBACK_SPEED_SCHEDULE` を編集：

```python
# デフォルト
FALLBACK_SPEED_SCHEDULE = [1.0, 0.8, 0.5, 0.2, 0.0]

# より早く停止 (3段階)
FALLBACK_SPEED_SCHEDULE = [1.0, 0.5, 0.0]

# より緩やかに低下 (7段階)
FALLBACK_SPEED_SCHEDULE = [1.0, 0.85, 0.7, 0.55, 0.4, 0.2, 0.0]
```

---

### Q5: 緊急停止がシステムに反映されない

**A:**

緊急停止トピック `/emergency_stop` が正しく受信されているか確認：

```bash
# Terminal 1: Safety Node を起動
python3 sharing/jetson_main.py

# Terminal 2: 緊急停止信号を送信
python3 -c "
import rclpy
from std_msgs.msg import Bool

rclpy.init()
node = rclpy.create_node('test')
pub = node.create_publisher(Bool, '/emergency_stop', 10)

msg = Bool(data=True)
pub.publish(msg)
print('Emergency stop published')

rclpy.shutdown()
"

# Terminal 1 のログで確認
# [ERROR] [ros2_safety_node]: Emergency stop activated!
```

---

### Q6: テスト時だけモデルを无視したい

**A:**

ダミーモデルを使用：

```python
# src/f1_env.py の step() 内で
if config.DUMMY_MODEL_MODE:
    # ランダムアクション
    action = np.random.uniform(-1, 1, 2)
else:
    # 実際のモデル推論
    action = model.predict(obs)[0]
```

`config.py` に追加：

```python
DUMMY_MODEL_MODE = False  # テスト時に True に変更
```

---

### Q7: ログを有効にしたい

**A:**

`src/f1_env.py` で ログ出力を有効化：

```python
def log_safety_event(self, event_type: str, details: dict = None):
    import json
    from pathlib import Path
    import time
    
    log_dir = Path('logs/safety_events')
    log_dir.mkdir(parents=True, exist_ok=True)
    
    event = {
        'timestamp': time.time(),
        'event_type': event_type,
        'safety_state': self.safety_manager.state.name if self.safety_manager else 'N/A',
        'details': details or {}
    }
    
    log_file = log_dir / f'safety_log_{time.strftime("%Y%m%d_%H%M%S")}.jsonl'
    with open(log_file, 'a') as f:
        f.write(json.dumps(event) + '\n')

# 各イベント時に呼び出し
if model_healthy == False:
    self.log_safety_event('MODEL_FAILURE', {
        'error_info': error_info,
        'inference_time': elapsed
    })
```

---

### Q8: 複数台の F1Tenth を同時走行させたい

**A:**

ROS2 の `ROS_DOMAIN_ID` を変更して隔離：

```bash
# Robot 1
export ROS_DOMAIN_ID=1
python3 sharing/jetson_main.py

# Robot 2 (別マシン)
export ROS_DOMAIN_ID=2
python3 sharing/jetson_main.py
```

---

## 本番走行チェックリスト (最終版)

走行 **30 分以内** に以下を確認してください：

### ✅ ハードウェア
- [ ] Jetson 温度 < 70°C
- [ ] バッテリー電圧: 11.5V 以上
- [ ] LiDAR スキャン周波数: 40Hz
- [ ] モーター制御 (VESC): 反応良好

### ✅ ソフトウェア
- [ ] ONNX 推論: < 18ms
- [ ] ROS2 ノード: 全て起動
- [ ] `/action`, `/safe_action` トピック: 通信OK
- [ ] `/model_health`: health=1 (正常)

### ✅ 安全機能
- [ ] Pure Pursuit フォールバック: テスト OK
- [ ] 衝突復帰シーケンス: テスト OK (シミュレーション)
- [ ] 緊急停止: トピック発行で即停止確認
- [ ] 段階的降速: タイムアウト時に機能確認

### ✅ ロジスティクス
- [ ] コース周辺に障害物がない
- [ ] スタート地点が明確
- [ ] カメラ / スマートフォンで記録準備
- [ ] 人員配置: 誰か１人は常に見守り状態

---

## 緊急時の対応

### 走行中に異常が発生した場合

1. **緊急停止ボタンを押す** (即座に停止)
   ```bash
   python3 emergency_stop_publisher.py
   ```

2. **Jetson を再起動** (リセット)
   ```bash
   ssh jetson@<ip_address>
   sudo reboot
   ```

3. **ログを確認** (原因特定)
   ```bash
   tail -f logs/safety_events/safety_log_*.jsonl
   ```

4. **本番走行を一時停止** して原因を究明してから再開

---

## 次のステップ

→ [本番走行](00_IMPLEMENTATION_OVERVIEW.md) を参照
