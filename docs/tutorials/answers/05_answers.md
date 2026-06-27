# チュートリアル 5 演習解答: カスタムメッセージ・サービス・アクション定義

## 演習 1: フィールドを追加してリビルドする

### 考え方

`.msg` ファイルへのフィールド追加は「ビルドシステムへの通信契約の変更」です。追加するだけでは不十分で、**ビルド → 環境再読み込み → 確認** という 3 ステップが必要です。

また、`sample_interfaces` に依存するパッケージ（`drone_sim`、`ground_robot_sim` など）は既存コードが新フィールドを知らない状態ですが、フィールドを追加するだけであれば既存コードはコンパイルエラーにはなりません（新フィールドがデフォルト値で初期化される）。

### 実装手順

**ステップ 1**: `RobotStatus.msg` を編集してフィールドを追加します。

対象ファイル: `src/sample_interfaces/msg/RobotStatus.msg`

末尾に以下の行を追加します。

```
float64 temperature   # センサー温度 [℃]
```

追加後のファイル全体:

```
# src/sample_interfaces/msg/RobotStatus.msg

std_msgs/Header header

string robot_name

string state

float64 battery_percentage

geometry_msgs/Point position

geometry_msgs/Vector3 linear_velocity

float64 heading_rad

float64 temperature   # ← 追加したフィールド
```

**ステップ 2**: ビルドします。

```bash
colcon build --packages-select sample_interfaces
```

期待されるビルド出力:
```
Starting >>> sample_interfaces
Finished <<< sample_interfaces [XX.Xs]
Summary: 1 package finished [XX.Xs]
```

**ステップ 3**: 環境を再読み込みします。

```bash
source install/setup.bash
```

**ステップ 4**: 新しいフィールドが認識されているか確認します。

```bash
ros2 interface show sample_interfaces/msg/RobotStatus
```

期待される出力（末尾に `float64 temperature` が追加されている）:
```
std_msgs/Header header
	builtin_interfaces/Time stamp
	string frame_id
string robot_name
string state
float64 battery_percentage
geometry_msgs/Point position
	float64 x
	float64 y
	float64 z
geometry_msgs/Vector3 linear_velocity
	float64 x
	float64 y
	float64 z
float64 heading_rad
float64 temperature
```

### Python でのフィールドの使い方

```python
from sample_interfaces.msg import RobotStatus

msg = RobotStatus()
msg.temperature = 42.5  # 新しいフィールドに値を設定
```

### 重要なポイント

- フィールドを追加した場合でも、既存の送信ノードは古いフィールドのみを設定し `temperature` はデフォルト値（0.0）のままになる
- 既存のノードがエラーなく動き続けるのは「追加」だからであり、フィールドを削除したり型を変えたりすると既存コードのコンパイルエラーになる
- 本番システムでは後方互換性を慎重に考慮する必要がある

---

## 演習 2: 新しいサービス定義を作成する

### 考え方

新しいサービス定義ファイルを作成するだけでは ROS2 に認識されません。`CMakeLists.txt` の `rosidl_generate_interfaces()` に新しいファイルのパスを追加してからビルドする必要があります。

`.srv` ファイルの構造は「リクエスト定義 `---` レスポンス定義」という形式です。

### 実装手順

**ステップ 1**: 新しいサービス定義ファイルを作成します。

対象ファイル: `src/sample_interfaces/srv/SetRobotMode.srv`（新規作成）

```
# リクエスト: 設定したいモード
string mode        # "manual" or "autonomous"
---
# レスポンス: 設定の成否
bool success
string message
```

**ステップ 2**: `CMakeLists.txt` を更新します。

対象ファイル: `src/sample_interfaces/CMakeLists.txt`

`rosidl_generate_interfaces()` に新しいファイルを追記します。

変更前:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/GetRobotStatus.srv"
  "action/NavigateWaypoints.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)
```

変更後:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/GetRobotStatus.srv"
  "srv/SetRobotMode.srv"
  "action/NavigateWaypoints.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)
```

**ステップ 3**: ビルドして確認します。

```bash
colcon build --packages-select sample_interfaces
source install/setup.bash
ros2 interface show sample_interfaces/srv/SetRobotMode
```

期待される出力:
```
string mode
---
bool success
string message
```

### 実際にサービスを使う Python コードの例

サービスサーバー側（新しいサービスを提供するノード）:

```python
from sample_interfaces.srv import SetRobotMode

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self._set_mode_srv = self.create_service(
            SetRobotMode,
            'set_robot_mode',
            self._handle_set_mode,
        )

    def _handle_set_mode(self, request, response):
        if request.mode in ('manual', 'autonomous'):
            self._current_mode = request.mode
            response.success = True
            response.message = f'モードを {request.mode} に変更しました'
        else:
            response.success = False
            response.message = f'不明なモード: {request.mode}'
        return response
```

CLI でサービスを呼び出す場合:

```bash
ros2 service call /set_robot_mode sample_interfaces/srv/SetRobotMode "{mode: 'autonomous'}"
```

期待される出力:
```
response:
  sample_interfaces.srv.SetRobotMode_Response(success=True, message='モードを autonomous に変更しました')
```

### なぜ CMakeLists.txt の変更が必要か

`rosidl_generate_interfaces()` は「このファイルからコードを自動生成してください」という指示です。この指示がなければビルドシステムは `.srv` ファイルの存在を知ることができません。ファイルを作成しただけでは何も生成されないため、必ず `CMakeLists.txt` への追記とビルドがセットになります。

---

## 演習 3: CLI でインターフェースを確認する

### 考え方

`ros2 interface` コマンドは ROS2 の型システムを調べる標準的な方法です。ノードの実装を読む前にインターフェース定義を確認することで、どのフィールドを送受信するかを素早く把握できます。

### 各コマンドの詳細解説

**`ros2 interface show`**: 型の定義を表示します。

```bash
ros2 interface show sample_interfaces/msg/RobotStatus
```

ネストされた型（`std_msgs/Header` や `geometry_msgs/Point`）のフィールドも展開して表示されます。

```bash
ros2 interface show sample_interfaces/srv/GetRobotStatus
```

期待される出力（空のリクエスト、詳細なレスポンス）:
```
# Request (空のリクエスト)
---
sample_interfaces/RobotStatus status
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string robot_name
	string state
	float64 battery_percentage
	geometry_msgs/Point position
		float64 x
		float64 y
		float64 z
	geometry_msgs/Vector3 linear_velocity
		float64 x
		float64 y
		float64 z
	float64 heading_rad
bool success
string message
```

```bash
ros2 interface show sample_interfaces/action/NavigateWaypoints
```

アクション定義には `---` が 2 つあり、ゴール・結果・フィードバックの 3 セクションに分かれています。

**`ros2 interface list`**: すべての型を一覧表示します（grep と組み合わせて使うのが便利）。

```bash
ros2 interface list | grep sample_interfaces
```

期待される出力:
```
sample_interfaces/action/NavigateWaypoints
sample_interfaces/msg/RobotStatus
sample_interfaces/srv/GetRobotStatus
```

**`ros2 interface packages`**: インターフェースを提供しているパッケージ一覧を表示します。

```bash
ros2 interface packages
```

出力に `sample_interfaces` が含まれていれば、パッケージが正しく認識されています。

### インターフェース確認のベストプラクティス

ノードのソースコードを読む前にまず以下の手順を行うと、実装の意図が把握しやすくなります。

1. `ros2 interface list | grep <パッケージ名>` で利用可能な型を確認する
2. `ros2 interface show <型名>` でフィールド構造を確認する
3. ノードのパブリッシャー / サブスクライバーが使っている型と照合する

この習慣を身につけることで、初めて見るパッケージのコードも読み解きやすくなります。
