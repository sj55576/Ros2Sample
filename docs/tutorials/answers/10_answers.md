# チュートリアル 10 解答例: コントローラーと経路追従

## Pure Pursuit パラメータチューニングの理論

演習の解答を読む前に、Pure Pursuit のパラメータがロボット挙動に与える影響の「なぜ」を理解しましょう。

### `lookahead_distance` が小さすぎる場合

```
lookahead_distance = 0.05m（例）

パス: ──────●──●──●──●──────── (ゴール)
              ↑ すぐ前のポイントだけを見る

ロボットは「直前のポイント」に向かって方向転換を繰り返すため、
小さな誤差に対して過剰反応し、ジグザグ（振動）になる。

時系列:
  t=0: → 右に 5° 曲がる
  t=1: → 左に 8° 曲がる（過修正）
  t=2: → 右に 6° 曲がる（過修正）
  ... （振動が収まらない）
```

**理由**: 曲率 `kappa = 2*sin(alpha)/lookahead_distance` において、`lookahead_distance` が小さいほど `kappa` が大きくなり、わずかな角度誤差 `alpha` でも大きな角速度コマンドが生成されます。

### `lookahead_distance` が大きすぎる場合

```
lookahead_distance = 1.0m（例）

パス: ──●──────────────────●── (ゴール)
        ↑ ここを見ている    ↑ ロボットはここ

「先を見すぎる」ため、カーブやコーナーで
パスから大きく外れる（ショートカット）。
```

**理由**: ルックアヘッドポイントが遠すぎると、曲線の経路を「直線で結んだ近道」として処理してしまいます。ロボットは常に「先の直線目標」に向かうため、途中のカーブを無視します。

### 適切な `lookahead_distance` の目安

```
lookahead_distance ≈ 1〜3 × (ロボットの速度 × 制御周期)

例: 速度 0.2 m/s、制御周期 0.1 s の場合
  最小推奨 ≈ 0.2 × 0.1 × 1 = 0.02 m（小さすぎる）
  実用的な値: 0.2〜0.5 m（速度の 1〜2.5 秒先）
```

実際は環境（通路の幅、カーブの曲率）に合わせて試行錯誤で決めます。Nav2 の RPP は `lookahead_distance` を障害物との距離に応じて自動調整します。

---

## 演習 1: 複数の経路を比較する

### 考え方

「斜め移動あり（`diagonal_movement = true`）」の経路と「斜め移動なし（`diagonal_movement = false`）」の経路では、経路の形状が根本的に異なります。これが Pure Pursuit の `angular.z` 出力に与える影響を比較します。

### 期待される `/cmd_vel` の違い

**L 字型経路（斜め移動なし）の場合**:

```
経路:  S──→→→→→→┐
                   ↓
                   ↓
                   G

経路の形: 直角コーナーが 1 つある L 字型
angular.z の変化:
  直線区間: angular.z ≈ 0（まっすぐ）
  コーナー: angular.z が急激に大きくなる（急旋回）
  コーナー後: angular.z ≈ 0（再びまっすぐ）
```

**斜め経路（斜め移動あり）の場合**:

```
経路:  S
        ↘
          ↘
            G

経路の形: なだらかな斜め移動
angular.z の変化:
  出発時: 少し旋回して斜め方向に向く
  移動中: angular.z ≈ 0〜小さい値（緩やかなカーブ）
  全体的に滑らか
```

### 動作確認手順

```bash
# ターミナル 1: マップパブリッシャー
ros2 run nav2_learning simple_map_publisher

# ターミナル 2: 経路計画ノード
ros2 run nav2_learning simple_path_planner

# ターミナル 3: 経路追従ノード
ros2 run nav2_learning simple_path_follower

# ターミナル 4: /cmd_vel を表示（別ターミナル）
ros2 topic echo /cmd_vel

# ターミナル 5: L 字経路（斜め移動なし）
ros2 param set /simple_path_planner diagonal_movement false
ros2 service call /plan_path nav2_learning/srv/PlanPath \
  "{start: {x: 0.0, y: 0.0}, goal: {x: 0.8, y: 0.8}}"
# → angular.z が急変する瞬間を確認

# L 字経路の確認後、斜め経路に切り替え
ros2 param set /simple_path_planner diagonal_movement true
ros2 service call /plan_path nav2_learning/srv/PlanPath \
  "{start: {x: 0.0, y: 0.0}, goal: {x: 0.8, y: 0.8}}"
# → angular.z が滑らかに変化することを確認
```

---

## 演習 2: 速度制限を実装する

### 考え方

Pure Pursuit が計算した速度コマンドは理論値であり、ロボットのハードウェア上限を超える場合があります。速度クランプ（上下限の切り捨て）を入れることで安全な速度範囲内に収めます。これは実際の Nav2（RPP）でも行われています。

### 完全な解答コード

`src/nav2_learning/nav2_learning/simple_path_follower.py` を以下のように変更します。主要な変更箇所を示します:

```python
# simple_path_follower.py への変更

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SimplePathFollower(Node):

    def __init__(self):
        super().__init__('simple_path_follower')

        # --- パラメータの宣言（変更前から存在するもの）---
        self.declare_parameter('lookahead_distance', 0.2)
        self.declare_parameter('linear_velocity', 0.2)

        # --- 演習 2: 速度制限パラメータを追加 ---
        self.declare_parameter('max_linear_velocity', 0.3)    # m/s
        self.declare_parameter('max_angular_velocity', 1.0)   # rad/s

        # ... サブスクライバー・パブリッシャーの設定（既存のまま）...
        self._path_sub = self.create_subscription(Path, '/plan', self._path_callback, 10)
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(0.1, self._control_loop)

        self._path = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_heading = 0.0

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """値を [min_val, max_val] の範囲にクランプする"""
        return max(min_val, min(max_val, value))

    def _control_loop(self):
        if self._path is None or len(self._path.poses) == 0:
            return

        # パラメータの取得
        lookahead = self.get_parameter('lookahead_distance').value
        linear_v  = self.get_parameter('linear_velocity').value
        max_lin   = self.get_parameter('max_linear_velocity').value    # 追加
        max_ang   = self.get_parameter('max_angular_velocity').value   # 追加

        # ルックアヘッドポイントを探す
        lookahead_x, lookahead_y = self._find_lookahead_point(lookahead)
        if lookahead_x is None:
            return

        # Pure Pursuit の速度計算
        angle_to_target = math.atan2(
            lookahead_y - self._robot_y,
            lookahead_x - self._robot_x
        )
        alpha = angle_to_target - self._robot_heading
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # 正規化

        kappa   = 2.0 * math.sin(alpha) / lookahead
        linear  = linear_v
        angular = linear_v * kappa

        # --- 演習 2: 速度クランプを適用 ---
        linear  = self._clamp(linear,  -max_lin, max_lin)
        angular = self._clamp(angular, -max_ang, max_ang)

        # 速度コマンドを送信
        cmd = Twist()
        cmd.linear.x  = linear
        cmd.angular.z = angular
        self._cmd_pub.publish(cmd)

    # ... _path_callback, _odom_callback, _find_lookahead_point は既存のまま ...
```

### パラメータで制限を動的に変更する

```bash
# 速度を大幅に下げる（慎重モード）
ros2 param set /simple_path_follower max_linear_velocity 0.1
ros2 param set /simple_path_follower max_angular_velocity 0.5

# 速度を上げる（高速モード）
ros2 param set /simple_path_follower max_linear_velocity 0.5
ros2 param set /simple_path_follower max_angular_velocity 2.0
```

### 速度制限の効果

| max_linear | max_angular | 挙動の特徴 |
|-----------|-----------|---------|
| 0.1 m/s   | 0.5 rad/s | 遅く慎重。コーナーで曲がりきれずに経路から外れることがある |
| 0.3 m/s   | 1.0 rad/s | バランスが取れたデフォルト値 |
| 0.5 m/s   | 2.0 rad/s | 速い。急カーブでオーバーシュートしやすい |

### ポイント

実際の Nav2（RPP）では `max_linear_velocity` に加えて「障害物に近いほど速度を下げる」規制（Regulated velocity）も持っています。固定値のクランプよりも動的な速度調整の方が安全性が高いです。

---

## 演習 3: ゴール到達判定を改善する

### 考え方

現在のシンプルな実装は「パスの最後のウェイポイントとの距離が閾値以下」という判定です。これには以下の問題があります:

1. **ロボットが最後のウェイポイントを通過した後も判定が通る**: 惰性で進んでいても距離が増えなければ停止しない
2. **向きが考慮されていない**: ゴール地点に到達しても向きが全く違う場合でも「到達」と判定される
3. **閾値がハードコードされている**: 調整が困難

### 改善解答コード

```python
# simple_path_follower.py への変更

class SimplePathFollower(Node):

    def __init__(self):
        super().__init__('simple_path_follower')

        # ... 既存のパラメータ ...
        self.declare_parameter('lookahead_distance', 0.2)
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)

        # --- 演習 3: ゴール到達判定パラメータを追加 ---
        self.declare_parameter('goal_tolerance', 0.1)          # m（位置の許容誤差）
        self.declare_parameter('yaw_goal_tolerance', 0.2)      # rad（向きの許容誤差）

        self._goal_reached = False  # ゴール到達フラグ

        # ... サブスクライバー・パブリッシャー（既存のまま） ...

    def _is_goal_reached(self, path: 'nav_msgs/Path') -> bool:
        """
        ゴール到達判定:
        1. 最後のウェイポイントまでの距離が goal_tolerance 以下
        2. 向きの誤差が yaw_goal_tolerance 以下（オプション）
        """
        if not path.poses:
            return False

        goal_tolerance = self.get_parameter('goal_tolerance').value
        yaw_tolerance  = self.get_parameter('yaw_goal_tolerance').value

        goal_pose = path.poses[-1].pose

        # 位置の誤差
        dx = goal_pose.position.x - self._robot_x
        dy = goal_pose.position.y - self._robot_y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)

        if distance_to_goal > goal_tolerance:
            return False

        # 向きの誤差（クォータニオン → ヨー角）
        q = goal_pose.orientation
        goal_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        yaw_error = abs(math.atan2(
            math.sin(goal_yaw - self._robot_heading),
            math.cos(goal_yaw - self._robot_heading)
        ))

        return yaw_error <= yaw_tolerance

    def _control_loop(self):
        if self._path is None or len(self._path.poses) == 0:
            return

        # ゴール到達チェック
        if self._is_goal_reached(self._path):
            if not self._goal_reached:
                self.get_logger().info('ゴールに到達しました！')
                self._goal_reached = True
            # 停止コマンドを送信
            self._cmd_pub.publish(Twist())
            return

        self._goal_reached = False  # 経路更新後にリセット

        # ... 既存の Pure Pursuit 計算 ...

    def _path_callback(self, msg: Path) -> None:
        """経路が更新されたらゴール到達フラグをリセット"""
        self._path = msg
        self._goal_reached = False  # 新しい経路が来たらリセット
```

### パラメータで閾値を調整する

```bash
# 厳密な到達判定（位置 5cm 以内 + 向き 0.1 rad 以内）
ros2 param set /simple_path_follower goal_tolerance 0.05
ros2 param set /simple_path_follower yaw_goal_tolerance 0.1

# ゆるい到達判定（位置 20cm 以内、向きは問わない）
ros2 param set /simple_path_follower goal_tolerance 0.2
ros2 param set /simple_path_follower yaw_goal_tolerance 3.14  # 事実上無制限
```

### ポイント

Nav2 の Controller Server も同様に `xy_goal_tolerance`（位置許容誤差）と `yaw_goal_tolerance`（向き許容誤差）をパラメータとして持っています。倉庫ロボットの棚前への正確な停止では向きの精度も重要ですが、単純な通過点では位置のみの判定で十分なことが多いです。用途に応じた使い分けが大切です。
