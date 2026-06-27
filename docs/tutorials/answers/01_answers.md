# チュートリアル 1 解答例: Publisher と Subscriber

このファイルでは、チュートリアル 1 の演習問題に対するヒント、考え方、および完全な解答例を説明します。コードをすぐに見るのではなく、まずヒントを読んで自分で考えてみてください。

---

## 演習 1: 送信レートをパラメータで変更する

### 考え方

`minimal_publisher` ノードはすでに `publish_rate_hz` パラメータを持っています。コードを変更する必要はなく、**起動コマンドに `--ros-args -p` オプションを追加するだけ**です。

ROS 2 のパラメータシステムでは、ノードが `declare_parameter` で宣言したパラメータに対して、起動時に外部から値を上書きできます。`minimal_publisher.py` の `__init__` を見ると、`publish_rate_hz` パラメータを読み取ってタイマー周期を設定していることがわかります。

### 解答例

```bash
# 5 Hz で送信する
ros2 run ros2_learning minimal_publisher --ros-args -p publish_rate_hz:=5.0

# 別ターミナルで受信レートを確認
ros2 topic hz /chatter
```

期待される出力:

```
average rate: 5.000
	min: 0.199s max: 0.201s std dev: 0.00015s window: 10
```

**確認ポイント**: `ros2 topic hz /chatter` の `average rate` が指定した値（5.000）と一致することを確認してください。

さらに試してみましょう:

```bash
# 0.5 Hz（2 秒に 1 回）に変更してみる
ros2 run ros2_learning minimal_publisher --ros-args -p publish_rate_hz:=0.5
ros2 topic hz /chatter
# average rate: 0.500 と表示されるはず
```

---

## 演習 2: メッセージ型を geometry_msgs/Point に変更する

### 考え方

この演習の本質は「**メッセージ型を変えると何を変える必要があるか**」を体験することです。ROS 2 のトピック通信では、Publisher と Subscriber が**同じトピック名**かつ**同じメッセージ型**を使う必要があります。

変更が必要な箇所は以下の 3 点です:
1. `import` 文（`String` → `Point`）
2. `create_publisher` の第 1 引数（メッセージ型）
3. コールバック内のメッセージオブジェクト生成と値の設定

`geometry_msgs/Point` は `x`、`y`、`z` という 3 つの `float64` フィールドを持ちます。`std_msgs/String` の `data` フィールドとは異なる構造です。

### 実装の流れ

既存の `minimal_publisher.py` を参考に、新しいファイル（例: `point_publisher.py`）を作る場合の手順は以下の通りです:

1. `from std_msgs.msg import String` を `from geometry_msgs.msg import Point` に変更する
2. `create_publisher(String, 'chatter', 10)` を `create_publisher(Point, 'point_data', 10)` に変更する（トピック名も変更することを推奨）
3. コールバック内のメッセージ生成部分を変更する

### 完全な解答例

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # (1) 変更点: インポートを変える


class PointPublisher(Node):
    def __init__(self):
        super().__init__('point_publisher')

        self.declare_parameter('publish_rate_hz', 1.0)
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # (2) 変更点: メッセージ型とトピック名を変える
        self._publisher = self.create_publisher(Point, 'point_data', 10)
        self._timer = self.create_timer(1.0 / rate, self._timer_callback)
        self._count = 0

        self.get_logger().info('Point パブリッシャーを起動しました')

    def _timer_callback(self):
        # (3) 変更点: Point メッセージを組み立てる
        msg = Point()
        msg.x = float(self._count)
        msg.y = float(self._count * 2)
        msg.z = 0.0
        self._publisher.publish(msg)
        self.get_logger().info(f'送信: x={msg.x}, y={msg.y}, z={msg.z}')
        self._count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 動作確認方法

このコードをファイルとして保存した場合、以下のコマンドで動作を確認できます:

```bash
# メッセージ型の定義を確認する
ros2 interface show geometry_msgs/msg/Point
```

期待される出力:

```
float64 x
float64 y
float64 z
```

ノードを起動してトピックを確認する:

```bash
# 別ターミナルで以下を実行
ros2 topic echo /point_data
```

期待される出力例:

```
x: 3.0
y: 6.0
z: 0.0
---
x: 4.0
y: 8.0
z: 0.0
---
```

### なぜこの演習が重要か

ROS 2 では標準メッセージ型（`std_msgs`、`geometry_msgs`、`sensor_msgs` など）を使うことで、異なるパッケージのノードが簡単に連携できます。例えば `geometry_msgs/Twist` はロボットの速度指令に広く使われており、どのロボットフレームワークでも共通の形式で扱えます。メッセージ型を意識的に選ぶことが、再利用性の高い ROS 2 システム設計の第一歩です。

---

## 演習 3: drone_sim のトピックを確認する

### 考え方

この演習は「**実際のロボットシステムでは Publisher をどのように使うか**」を読み解く演習です。`drone_sim` の `sim_drone` ノードは、チュートリアルの `minimal_publisher` と同じ仕組みを使いながら、複数のトピックを同時にパブリッシュしています。

### 解答例

```bash
# drone_sim をビルドしてセットアップする
colcon build --packages-select drone_sim sample_interfaces
source install/setup.bash

# バックグラウンドでノードを起動する
ros2 run drone_sim sim_drone &

# トピック一覧を確認する
ros2 topic list
```

期待される出力例（主要なもの）:

```
/battery
/cmd_vel
/imu
/odom
/pose
/rosout
/status
```

各トピックの型を確認する:

```bash
ros2 topic info /odom
ros2 topic info /imu
ros2 topic info /pose
```

`/odom` の最初のメッセージを受信する:

```bash
ros2 topic echo /odom --once
```

期待される出力例:

```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [...]
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [...]
```

### `sim_drone.py` との対応を理解する

`src/drone_sim/drone_sim/sim_drone.py` を読むと、以下の Publisher が定義されています:

| トピック名 | メッセージ型 | 用途 |
|-----------|------------|------|
| `/odom` | `nav_msgs/Odometry` | ドローンの位置・速度 |
| `/pose` | `geometry_msgs/PoseStamped` | ドローンの姿勢 |
| `/imu` | `sensor_msgs/Imu` | IMU（慣性センサ）データ |
| `/battery` | `sensor_msgs/BatteryState` | バッテリー状態 |
| `/status` | `sample_interfaces/RobotStatus` | カスタム状態メッセージ |

これらはすべて `minimal_publisher.py` と同じ `create_publisher` + `create_timer` + `publish` の構造で実装されています。型が異なるだけで、基本的な仕組みは同じです。

起動後は必ずバックグラウンドプロセスを終了してください:

```bash
# バックグラウンドプロセスを終了する
kill %1
# または
ros2 lifecycle set /sim_drone shutdown  # ライフサイクルノードの場合
```
