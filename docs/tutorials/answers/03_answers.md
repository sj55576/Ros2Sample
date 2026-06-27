# チュートリアル 3 解答例: Launch ファイルとパラメータ

このファイルでは、チュートリアル 3 の演習問題に対するヒント、考え方、および完全な解答例を説明します。コードをすぐに見るのではなく、まずヒントを読んで自分で考えてみてください。

---

## 演習 1: パラメータを動的に変更して動作を確認する

### 考え方

この演習の目的は「**コードを再ビルドせずにノードの動作を変える**」体験をすることです。ROS 2 のパラメータシステムにより、実行中のノードのふるまいをリアルタイムで調整できます。

重要なポイントは、`enable_logging` を `false` にしてもトピックへの送信は続くという点です。これは `_publish_info` メソッドが「ログ出力するかどうか」と「トピックにパブリッシュするかどうか」を独立して制御しているためです。

### 解答例

**ターミナル 1**: ノードを起動する

```bash
ros2 run ros2_learning parameter_demo
```

**ターミナル 2**: `robot_info` トピックを監視する

```bash
ros2 topic echo /robot_info
```

**ターミナル 3**: パラメータを順番に変更して動作の変化を観察する

```bash
# enable_logging を false にする（ターミナル 1 のログが止まる）
ros2 param set /parameter_demo enable_logging false
```

確認事項:
- ターミナル 1 のログが止まる
- ターミナル 2 のトピック受信は続く（パブリッシュは継続）

```bash
# update_rate_hz を 5.0 に変更する（トピック送信が速くなる）
ros2 param set /parameter_demo update_rate_hz 5.0
```

確認事項:
- ターミナル 2 のメッセージ更新が速くなる
- `ros2 topic hz /robot_info` で 5.0 Hz になることを確認できる

```bash
# robot_name を変更する（トピック内容が変わる）
ros2 param set /parameter_demo robot_name "my_robot"
```

確認事項:
- ターミナル 2 の出力が `robot_name=my_robot, ...` に変わる

### 期待される出力例

`ros2 topic echo /robot_info` の出力（`robot_name` 変更後）:

```
data: 'robot_name=my_robot, max_speed=1.00, enable_logging=False'
---
data: 'robot_name=my_robot, max_speed=1.00, enable_logging=False'
---
```

ターミナル 1 のログ（パラメータ変更のたびに表示される）:

```
[INFO] [parameter_demo]: パラメータ変更: enable_logging True -> False
[INFO] [parameter_demo]: パラメータ変更: update_rate_hz 2.0 -> 5.0
[INFO] [parameter_demo]: パラメータ変更: robot_name 'learning_bot' -> 'my_robot'
```

---

## 演習 2: 自前の YAML 設定ファイルを作る

### 考え方

YAML でパラメータを管理することの利点は「**再現性**」です。コマンドラインに毎回 `-p` オプションを並べる代わりに、YAML ファイルに書いておけば、いつでも同じ設定でノードを起動できます。チームで開発するときや、複数の設定プリセットを切り替えるときに特に便利です。

YAML ファイルの書式で注意すべき点:
- ノード名（`parameter_demo`）は `ros2 node info` で確認できる正確な名前を使う
- `ros__parameters` のアンダースコアは**2 つ**（`ros_parameters` ではなく `ros__parameters`）
- インデントはスペース 2 つが一般的

### 解答例

`my_demo.yaml` を作成する（任意の場所に保存可能）:

```yaml
# my_demo.yaml
parameter_demo:
  ros__parameters:
    robot_name: "my_yaml_bot"
    max_speed: 0.5
    update_rate_hz: 0.5
    enable_logging: true
```

このファイルを使ってノードを起動する:

```bash
ros2 run ros2_learning parameter_demo \
    --ros-args --params-file my_demo.yaml
```

期待されるログ出力:

```
[INFO] [parameter_demo]: パラメータデモ起動: robot_name=my_yaml_bot, max_speed=0.5, update_rate_hz=0.5, enable_logging=True
```

起動後に値が反映されているかを確認する:

```bash
ros2 param get /parameter_demo robot_name
```

期待される出力:

```
String value is: my_yaml_bot
```

```bash
ros2 param get /parameter_demo max_speed
```

期待される出力:

```
Double value is: 0.5
```

### YAML + 起動時 CLI override の組み合わせ

YAML ファイルで基本設定をしながら、`-p` オプションで一部だけ上書きすることもできます:

```bash
# YAML の robot_name を上書きして起動する
ros2 run ros2_learning parameter_demo \
    --ros-args --params-file my_demo.yaml \
    -p robot_name:="override_bot"
```

この場合、`robot_name` だけが `override_bot` になり、他のパラメータは YAML の値が使われます。

### Launch ファイルから YAML を読み込む方法

自分で Launch ファイルを作る場合は、以下のように YAML ファイルを `parameters` に渡します:

```python
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージのインストール先からYAMLを読み込む
    share_dir = get_package_share_directory('ros2_learning')
    config_file = join(share_dir, 'config', 'parameter_demo.yaml')

    return LaunchDescription([
        Node(
            package='ros2_learning',
            executable='parameter_demo',
            name='parameter_demo',
            output='screen',
            parameters=[config_file],
        ),
    ])
```

`ros2_learning` パッケージにはすでに `parameter_demo.launch.py` が含まれており、`src/ros2_learning/config/parameter_demo.yaml` を読み込みます:

```bash
ros2 launch ros2_learning parameter_demo.launch.py
```

---

## 演習 3: swarm.launch.py の名前空間を観察する

### 考え方

この演習は「**名前空間がなぜ必要か**」を体験することです。同じ種類のノードを複数起動すると、トピック名やサービス名が衝突してしまいます。名前空間（namespace）を使うと、同じコードのノードを `/drone_1/...` と `/drone_2/...` のように独立した空間で動かせます。

実際のロボットシステムでは、複数のセンサ・アクチュエータが同じ種類のインターフェースを持つため、名前空間は不可欠な仕組みです。

### 解答例

`drone_sim` をビルドしてセットアップする:

```bash
colcon build --packages-select drone_sim sample_interfaces
source install/setup.bash
```

2 機のドローンで起動する:

```bash
ros2 launch drone_sim swarm.launch.py drone_count:=2
```

別ターミナルでトピック一覧を確認する:

```bash
ros2 topic list | grep odom
```

期待される出力（名前空間ごとに独立したトピックが確認できること）:

```
/drone_1/odom
/drone_2/odom
```

各ドローンのトピックを確認する:

```bash
# ドローン 1 のオドメトリ
ros2 topic echo /drone_1/odom --once

# ドローン 2 のオドメトリ
ros2 topic echo /drone_2/odom --once
```

ノード一覧でも名前空間が確認できる:

```bash
ros2 node list
```

期待される出力例:

```
/drone_1/sim_drone
/drone_1/waypoint_commander
/drone_1/battery_monitor
/drone_2/sim_drone
/drone_2/waypoint_commander
/drone_2/battery_monitor
```

### 名前空間の仕組みを理解する

`swarm.launch.py` では `PushRosNamespace` を使って名前空間を設定しています。ノードのコード（`sim_drone.py`）はトピック名を `/odom` として定義していますが、Launch ファイルで名前空間 `drone_1` が適用されると、実際のトピック名は `/drone_1/odom` になります。

このおかげでノードのコードを変更せずに、複数インスタンスを共存させられます。

### 引数を変えて試してみる

```bash
# 5 機のドローン、間隔 3m で起動する
ros2 launch drone_sim swarm.launch.py drone_count:=5 spacing_m:=3.0

# 別ターミナルで確認する
ros2 topic list | grep odom
# /drone_1/odom
# /drone_2/odom
# /drone_3/odom
# /drone_4/odom
# /drone_5/odom
```

Launch 引数の一覧を確認する:

```bash
ros2 launch drone_sim swarm.launch.py --show-args
```

期待される出力:

```
Arguments (pass arguments as '<name>:=<value>'):

    'drone_count':
        起動するドローンの数
        (default: '3')

    'spacing_m':
        ドローン間の間隔（メートル）
        (default: '2.0')

    'altitude_m':
        ホバリング高度（メートル）
        (default: '1.2')
```
