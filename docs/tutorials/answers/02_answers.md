# チュートリアル 2 解答例: サービスとアクション

このファイルでは、チュートリアル 2 の演習問題に対するヒント、考え方、および完全な解答例を説明します。コードをすぐに見るのではなく、まずヒントを読んで自分で考えてみてください。

---

## 演習 1: サービスを CLI で操作する

### 考え方

この演習は「**コードを書かずに CLI だけでサービスを理解する**」ことを目的としています。`ros2 service` コマンドは、サービスサーバーのコードを持っていなくても、型情報とサービス名さえわかれば呼び出せます。

### 解答例

**ターミナル 1**: サーバーを起動する

```bash
ros2 run ros2_learning minimal_service_server
```

期待される出力:

```
[INFO] [minimal_service_server]: 'set_flag' サービスの待機を開始しました。
```

**ターミナル 2**: 各コマンドを順に実行する

```bash
# 起動しているサービスの一覧を確認する
ros2 service list
```

期待される出力（`/set_flag` が含まれること）:

```
/set_flag
/minimal_service_server/describe_parameters
...
```

```bash
# /set_flag サービスの型を確認する
ros2 service type /set_flag
```

期待される出力:

```
std_srvs/srv/SetBool
```

```bash
# SetBool サービス型のフィールドを確認する
ros2 interface show std_srvs/srv/SetBool
```

期待される出力:

```
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

`---` の上がリクエストフィールド、下がレスポンスフィールドです。

```bash
# True を送る
ros2 service call /set_flag std_srvs/srv/SetBool "{data: true}"
```

期待される出力:

```
requester: making request: std_srvs.srv.SetBool_Request(data=True)

response:
  std_srvs.srv.SetBool_Response(success=True, message='フラグをONにしました')
```

ターミナル 1 のサーバーログにも以下が表示されます:

```
[INFO] [minimal_service_server]: サービス呼び出し: data=True -> フラグをONにしました
```

```bash
# False を送る
ros2 service call /set_flag std_srvs/srv/SetBool "{data: false}"
```

期待される出力:

```
response:
  std_srvs.srv.SetBool_Response(success=True, message='フラグをOFFにしました')
```

### なぜ CLI で操作できるのか

ROS 2 のサービスは**型情報を自己記述的**に持っています。`ros2 service type` で型を調べ、`ros2 interface show` でフィールドを確認すれば、クライアントコードを書かなくてもサービスを呼び出せます。これはデバッグや動作確認に非常に便利です。

---

## 演習 2: サービスとトピックを組み合わせる

### 考え方

この演習の本質は「**サービスのコールバック内でトピックを同時に使う**」パターンを理解することです。

ROS 2 のノードは複数の通信手段を同時に持てます。サービスサーバーとして動作しながら、同時に Publisher としてトピックを送信できます。この組み合わせは実際のロボットシステムで頻繁に使われます。

例えば:
- `emergency_stop` サービスを受け取ったら、`/cmd_vel` トピックに速度ゼロを送信する
- 設定変更サービスを受け取ったら、変更後の状態を状態トピックで配信する

### 実装の流れ

`minimal_service_server.py` を改造する手順:

1. `Bool` メッセージ型をインポートする
2. `__init__` 内に Publisher を追加する
3. `_handle_set_flag` コールバック内でフラグの状態をパブリッシュする

変更が必要なのは **3 箇所だけ**です。既存のコードに追加する形で実装できます。

### 完全な解答例

`minimal_service_server.py` への変更箇所を示します（`+` が追加行です）:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool  # (1) 追加: Bool メッセージ型をインポートする


class MinimalServiceServer(Node):
    def __init__(self):
        super().__init__('minimal_service_server')
        self._flag = False

        self._srv = self.create_service(
            SetBool,
            'set_flag',
            self._handle_set_flag,
        )

        # (2) 追加: flag_status トピックへの Publisher を作成する
        self._flag_pub = self.create_publisher(Bool, 'flag_status', 10)

        self.get_logger().info("'set_flag' サービスの待機を開始しました。")

    def _handle_set_flag(self, request, response):
        self._flag = request.data

        response.success = True
        if request.data:
            response.message = 'フラグをONにしました'
        else:
            response.message = 'フラグをOFFにしました'

        # (3) 追加: フラグの状態をトピックとしてパブリッシュする
        flag_msg = Bool()
        flag_msg.data = self._flag
        self._flag_pub.publish(flag_msg)

        self.get_logger().info(
            f'サービス呼び出し: data={request.data} -> {response.message}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MinimalServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 動作確認方法

**ターミナル 1**: 改造後のサーバーを起動する

```bash
ros2 run ros2_learning minimal_service_server
```

**ターミナル 2**: `flag_status` トピックを監視する

```bash
ros2 topic echo /flag_status
```

**ターミナル 3**: サービスを呼び出して変化を観察する

```bash
ros2 service call /set_flag std_srvs/srv/SetBool "{data: true}"
```

ターミナル 2 の出力に以下が表示されることを確認する:

```
data: true
---
```

```bash
ros2 service call /set_flag std_srvs/srv/SetBool "{data: false}"
```

ターミナル 2 の出力に以下が表示されることを確認する:

```
data: false
---
```

### このパターンの応用例

`ground_robot_sim` の `ground_robot_node.py` では、`emergency_stop` サービスを受け取ったときに内部フラグを変更し、メインループがそのフラグを確認して `cmd_vel` トピックへの速度送信を停止する、という組み合わせが使われています。サービスとトピックの組み合わせは「**外部からのコマンドで状態を変え、その状態を継続的に配信する**」というよくあるパターンです。

---

## 演習 3: NavigateWaypoints アクション型を読む

### 考え方

アクション型の `.action` ファイルは 3 つのセクション（Goal / Result / Feedback）を `---` 区切りで定義します。それぞれが担う役割を理解することが重要です。

### 解答例

`sample_interfaces/action/NavigateWaypoints.action` の内容:

```
# ---- Goal ----
geometry_msgs/PoseStamped[] waypoints  # 巡回するウェイポイントのリスト
bool loop                               # true のとき繰り返し巡回する
float64 tolerance_m                     # ウェイポイント到達と判定する距離
---
# ---- Result ----
bool success
uint32 waypoints_completed
string message
---
# ---- Feedback ----
uint32 current_index           # 現在向かっているウェイポイントの番号
uint32 total_waypoints
float64 distance_to_current    # 現在地から目標ウェイポイントまでの距離
geometry_msgs/Point current_position
```

**Q1. Goal のフィールドはどれか**

Goal フィールドは `---` より前の部分です:
- `waypoints`: 巡回する目標位置のリスト（`geometry_msgs/PoseStamped` の配列）
- `loop`: `true` の場合、ウェイポイントリストを繰り返し巡回し続ける
- `tolerance_m`: 何メートル以内に近づいたらそのウェイポイントに「到達した」とみなすか

**Q2. Feedback でリアルタイムに送られる情報はどれか**

Feedback フィールドは 2 つ目の `---` より後の部分です:
- `current_index`: 現在向かっているウェイポイントの番号（0 始まり）
- `total_waypoints`: ウェイポイントの総数
- `distance_to_current`: 現在地から次のウェイポイントまでの距離（メートル）
- `current_position`: ロボットの現在位置

クライアントはこれらを受け取ることで、「今何番目のウェイポイントに向かっていて、あと何メートルか」をリアルタイムに知ることができます。

**Q3. `waypoints_completed` はどのような場面で使うか**

`waypoints_completed` は Result フィールドに含まれます。例えば 5 つのウェイポイントを指定して途中でキャンセルされた場合、`success=False`、`waypoints_completed=3`（3 つ目まで到達）という形で返ります。これにより、タスクが完全に成功したかどうかだけでなく、「どこまで進んだか」という詳細情報を受け取れます。

### `navigate_waypoints_server.py` のフィードバック送信タイミング

`src/ground_robot_sim/ground_robot_sim/navigate_waypoints_server.py` の `_execute_callback` では、各ウェイポイントへ移動中にフィードバックが送信されています。実装を確認するコマンド:

```bash
ros2 interface show sample_interfaces/action/NavigateWaypoints
```

期待される出力:

```
geometry_msgs/PoseStamped[] waypoints
bool loop
float64 tolerance_m
---
bool success
uint32 waypoints_completed
string message
---
uint32 current_index
uint32 total_waypoints
float64 distance_to_current
geometry_msgs/Point current_position
```
