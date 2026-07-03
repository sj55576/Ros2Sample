# ROS 2 Jazzy 学習パス

このリポジトリは、ROS 2 Jazzy の基礎概念を実際に動くコードで段階的に学ぶための学習用リポジトリです。シミュレーションパッケージ（ドローン・地上ロボット・マニピュレータ・センサフュージョン）と、基礎概念を最小構成で学ぶ `ros2_learning` パッケージで構成されています。

## 対象読者

- ROS 2 を初めて学ぶ方
- Ubuntu 24.04 と Python の基本操作ができる方
- ロボットプログラミングの概念に興味がある方

---

## 前提条件

| 項目 | 内容 |
|------|------|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco（インストール済み） |
| Python | 3.12 以上、基本的な文法を理解していること |
| Git | リポジトリのクローンができること |

ROS 2 Jazzy のインストールは [公式ドキュメント](https://docs.ros.org/en/jazzy/Installation.html) を参照してください。

---

## ビルドと実行準備

各チュートリアルを試す前に、以下の手順でパッケージをビルドしてください。

```bash
source /opt/ros/jazzy/setup.bash
cd Ros2Sample
colcon build --packages-select ros2_learning sample_interfaces
source install/setup.bash
```

Navigation2 と RViz のチュートリアルを進める場合は、`nav2_learning` とシミュレーションパッケージもビルドします。

```bash
colcon build --packages-select \
  ros2_learning \
  sample_interfaces \
  nav2_learning \
  ground_robot_sim \
  drone_sim \
  manipulator_sim
source install/setup.bash
```

> **注意**: `source install/setup.bash` はターミナルを新しく開くたびに実行する必要があります。毎回入力する手間を省くには `~/.bashrc` に追記することもできます。

---

## 学習パス概要

以下の順番で進めることを推奨します。各ステップは前のステップの知識を前提としています。

| ステップ | チュートリアル | 目安時間 | 内容 |
|----------|---------------|----------|------|
| 1 | [Publisher と Subscriber](01_publisher_subscriber.md) | 30 分 | トピック通信の基礎 |
| 2 | [サービスとアクション](02_service_action.md) | 45 分 | リクエスト/レスポンス型通信 |
| 3 | [Launch ファイルとパラメータ](03_launch_params.md) | 30 分 | ノード管理と設定 |
| 4 | [TF と座標変換](04_tf_transforms.md) | 45 分 | フレーム間の位置関係 |
| 5 | [カスタムインターフェース](05_custom_interfaces.md) | 30 分 | 独自メッセージ型定義 |
| 6 | [ライフサイクルノードと QoS](06_lifecycle_qos.md) | 45 分 | 高度なノード管理 |
| 7 | [Navigation2 の全体像](07_nav2_overview.md) | 45 分 | ナビゲーションスタックの理解 |
| 8 | [マップとコストマップ](08_costmap_and_map.md) | 45 分 | 環境認識の基礎 |
| 9 | [経路計画](09_path_planning.md) | 60 分 | A* アルゴリズムと経路生成 |
| 10 | [コントローラーと経路追従](10_nav2_controller.md) | 45 分 | Pure Pursuit による移動制御 |
| 11 | [ビヘイビアツリー入門](11_behavior_tree.md) | 45 分 | タスク管理とリカバリ |
| 12 | [RViz 可視化](12_rviz_visualization.md) | 45 分 | TF、センサー、マップ、経路の可視化 |
| 13 | [ROS 2 デバッグ入門](13_debugging_ros2_systems.md) | 45 分 | CLI と rqt_graph による切り分け |
| 14 | [既存パッケージを読み解く](14_reading_existing_packages.md) | 60 分 | 実装コードの読み方と概念の対応 |
| 15 | [ミニ課題プロジェクト集](15_mini_projects.md) | 90 分 | 複数概念を組み合わせた実践課題 |
| 16 | [トラブルシューティング集](16_troubleshooting.md) | — | よくあるエラーと対処法 |
| 17 | [Gazebo / GZ Sim 連携入門](17_gazebo_integration.md) | 60 分 | GZ Sim、URDF/SDF、ros_gz_bridge の接続 |
| 18 | [ROS 2 テストの書き方](18_testing_ros2.md) | 45 分 | pytest 単体テスト、colcon test、CI 連携 |
| 19 | [ROS 2 Bag の記録・再生](19_rosbag2.md) | 45 分 | トピック記録・再生・オフライン解析 |
| 20 | [Component と Composition](20_composition.md) | 45 分 | プロセス内通信と Composable Node |
| 21 | [マルチロボット通信と DDS Discovery](21_multi_robot_communication.md) | 60 分 | ROS_DOMAIN_ID、DDS Discovery、Docker 間通信 |
| 22 | [MoveIt2 / マニピュレータ経路計画入門](22_moveit2_manipulator_planning.md) | 60 分 | MoveIt2 設定、moveit_py、障害物回避 |

### 学習全体の見取り図

```mermaid
flowchart LR
    start["ROS 2 の通信を知る"] --> topic["1. Topic<br/>Publisher / Subscriber"]
    topic --> request["2. Request 型通信<br/>Service / Action"]
    request --> config["3. 起動と設定<br/>Launch / Parameter"]
    config --> tf["4. ロボットの座標<br/>TF2"]
    tf --> iface["5. 型を設計する<br/>msg / srv / action"]
    iface --> ops["6. 運用の基礎<br/>Lifecycle / QoS"]
    ops --> nav2["7. Nav2 全体像"]
    nav2 --> map["8. Map / Costmap"]
    map --> planner["9. Planner"]
    planner --> controller["10. Controller"]
    controller --> bt["11. Behavior Tree"]
    bt --> rviz["12. RViz で見る"]
    rviz --> debug["13. CLI で調べる"]
    debug --> apps["14. 既存パッケージを読む<br/>ground_robot_sim / drone_sim"]
    apps --> mini["15. ミニ課題で実践"]
    mini --> trouble["16. トラブルシューティング"]
    trouble --> gz["17. GZ Sim とつなぐ"]
    gz --> testing["18. テストを書く"]
    testing --> rosbag["19. Bag で記録・再生"]
    rosbag --> composition["20. Composition"]
    composition --> dds["21. DDS Discovery"]
    dds --> moveit2["22. MoveIt2<br/>マニピュレータ経路計画"]
```

前半の 1〜6 は ROS 2 の共通部品を学ぶ段階です。7〜11 は、その部品を自律移動システムに組み合わせる段階です。12〜13 は、動作を目で確認し、問題を CLI で切り分けるための実践編です。14 は、学んだ概念が実際のパッケージでどう使われているかを読み解く総まとめです。15 はミニ課題で複数の概念を組み合わせて実装力を磨き、16 は開発中に遭遇しやすいエラーの対処法をまとめています。17 は軽量 Python シミュレーションから GZ Sim へ進む発展編です。18 はテストの書き方を学び、品質を維持しながら開発を進める方法を身につけます。19 はトピックデータの記録・再生によるデバッグとオフライン解析を学びます。20 は Composition によるプロセス内通信とパフォーマンス最適化の入口です。21 は複数ドメインや複数マシン・コンテナにまたがる通信設定を学び、実環境へのデプロイに備えます。22 は MoveIt2 によるマニピュレータ経路計画へ進みます。途中で詰まった場合は、図の左側に戻って前提概念を確認してください。

---

### ステップ 1: Publisher と Subscriber（30 分）

ROS 2 の最も基本的な通信方式です。ノードがトピックにメッセージを送信（Publish）し、別のノードがそれを受信（Subscribe）します。センサデータの配信やコマンドの送受信など、ROS 2 のほぼすべてのシステムで使われます。

- 学習ファイル: `src/ros2_learning/ros2_learning/minimal_publisher.py`
- 学習ファイル: `src/ros2_learning/ros2_learning/minimal_subscriber.py`

### ステップ 2: サービスとアクション（45 分）

サービスは「質問して答えをもらう」同期型通信、アクションは「長時間タスクの進捗を受け取りながら待つ」非同期型通信です。ナビゲーションや制御コマンドによく使われます。

- 学習ファイル: `src/ros2_learning/ros2_learning/minimal_service_server.py`
- 学習ファイル: `src/ros2_learning/ros2_learning/minimal_service_client.py`

### ステップ 3: Launch ファイルとパラメータ（30 分）

複数のノードを一度に起動したり、ノードの動作を設定ファイルで調整する方法を学びます。実用的なシステム構築に必須の知識です。

- 学習ファイル: `src/ros2_learning/ros2_learning/parameter_demo.py`

### ステップ 4: TF と座標変換（45 分）

ロボットの各部位（ボディ・センサ・地図原点）の位置関係を管理する TF ライブラリを学びます。複数フレーム間の座標変換はロボティクスの要です。

- 学習ファイル: `src/ros2_learning/ros2_learning/tf_broadcaster_demo.py`
- 学習ファイル: `src/ros2_learning/ros2_learning/tf_listener_demo.py`

### ステップ 5: カスタムインターフェース（30 分）

`std_msgs` や `geometry_msgs` に用意されていない独自のメッセージ・サービス・アクション型を定義する方法を学びます。

- 参照パッケージ: `src/sample_interfaces/`

### ステップ 6: ライフサイクルノードと QoS（45 分）

ノードの起動・設定・アクティブ・シャットダウンという状態遷移を管理するライフサイクルノードと、通信品質を制御する QoS（Quality of Service）を学びます。

- 学習ファイル: `src/ros2_learning/ros2_learning/lifecycle_demo.py`

### ステップ 7: Navigation2 の全体像（45 分）

ROS 2 向け自律移動フレームワーク Navigation2（Nav2）のアーキテクチャを俯瞰します。Planner Server・Controller Server・BT Navigator・Costmap 2D などの主要コンポーネントの役割と、`ground_robot_sim` のカスタム実装と Nav2 の対応関係を理解します。

- 学習ファイル: `docs/tutorials/07_nav2_overview.md`
- 参照パッケージ: `src/ground_robot_sim/`

### ステップ 8: マップとコストマップ（45 分）

ナビゲーションの基盤となる `nav_msgs/OccupancyGrid` メッセージの構造と、静的マップ・動的コストマップ・インフレーションレイヤーの仕組みを学びます。簡易マップパブリッシャーで実際に地図を配信して RViz で確認します。

- 学習ファイル: `src/nav2_learning/nav2_learning/simple_map_publisher.py`
- 学習ファイル: `src/nav2_learning/nav2_learning/map_utils.py`

### ステップ 9: 経路計画（60 分）

A* アルゴリズムの動作原理（コスト関数・ヒューリスティック・8 方向探索）を学び、OccupancyGrid 上での実装を通して経路計画を体験します。Nav2 の NavFn・Smac プランナーとの比較も行います。

- 学習ファイル: `src/nav2_learning/nav2_learning/simple_path_planner.py`

### ステップ 10: コントローラーと経路追従（45 分）

計画された経路に沿ってロボットを動かす Pure Pursuit アルゴリズムを実装します。`ground_robot_sim` の PID 制御との比較を通して、パス追従制御の考え方を理解します。Nav2 の DWB・RPP・MPPI コントローラーとの比較も行います。

- 学習ファイル: `src/nav2_learning/nav2_learning/simple_path_follower.py`

### ステップ 11: ビヘイビアツリー入門（45 分）

Nav2 がナビゲーション全体を管理するために使うビヘイビアツリー（BT）の基本概念を学びます。Sequence・Fallback・Action・Condition の各ノード種別と、リプランニング・リカバリを BT で表現する方法を理解します。

- 参照ファイル: `/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

### ステップ 12: RViz 可視化（45 分）

TF、Odometry、LaserScan、OccupancyGrid、Path を RViz 上に表示し、CLI で見ていたデータを座標系上で確認します。各パッケージに同梱された RViz 設定も利用します。

- 学習ファイル: `docs/tutorials/12_rviz_visualization.md`
- 参照設定: `src/ground_robot_sim/rviz/ground_robot.rviz`
- 参照設定: `src/drone_sim/rviz/drone_sim.rviz`
- 参照設定: `src/nav2_learning/rviz/nav2_learning.rviz`

### ステップ 13: ROS 2 デバッグ入門（45 分）

`ros2 node`、`ros2 topic`、`ros2 service`、`ros2 action`、`ros2 param`、`tf2_echo`、`rqt_graph` を使って、ノードや topic が期待通りに動かない原因を順番に切り分けます。

- 学習ファイル: `docs/tutorials/13_debugging_ros2_systems.md`

### ステップ 14: 既存パッケージを読み解く（60 分）

チュートリアル 01〜13 で学んだ概念が `ground_robot_sim`・`drone_sim`・`manipulator_sim` のソースコードでどう使われているかを読み解きます。各パッケージの「どのファイルから読むか」と、チュートリアル章との対応表を示します。

- 学習ファイル: `docs/tutorials/14_reading_existing_packages.md`
- 参照パッケージ: `src/ground_robot_sim/`、`src/drone_sim/`、`src/manipulator_sim/`

### ステップ 15: ミニ課題プロジェクト集（90 分）

チュートリアル 01〜14 で学んだ概念を複数組み合わせて、小さな機能を自力で実装する課題集です。カスタム msg の定義、Service による動作モード切替、TF 付きセンサーの追加、障害物停止ノード、Waypoint 巡回の改造の 5 課題を段階的に取り組みます。

- 学習ファイル: `docs/tutorials/15_mini_projects.md`

### ステップ 16: トラブルシューティング集

ROS 2 開発中によく遭遇するエラーを症状別に整理しています。Package not found、colcon build 失敗、topic が流れない、TF がつながらないなど、初学者がつまずきやすいポイントとその対処法をまとめています。

- 学習ファイル: `docs/tutorials/16_troubleshooting.md`

### ステップ 17: Gazebo / GZ Sim 連携入門（60 分）

`ground_robot_sim` の GZ 用 URDF / SDF / launch ファイルを使い、GZ Sim 上に差動二輪ロボットを spawn します。`ros_gz_bridge` で `/cmd_vel`、`/odom`、`/tf`、`/joint_states` を接続し、軽量シミュレーションから物理シミュレーションへ進むための基本構成を学びます。

- 学習ファイル: `docs/tutorials/17_gazebo_integration.md`
- 参照ファイル: `src/ground_robot_sim/launch/gazebo.launch.py`
- 参照ファイル: `src/ground_robot_sim/urdf/ground_robot_gazebo.urdf`
- 参照ファイル: `src/ground_robot_sim/worlds/default.sdf`

### ステップ 18: ROS 2 テストの書き方（45 分）

pytest による単体テストの書き方を、リポジトリの既存テストコードを題材に学びます。`pytest.approx`・fixture・parametrize の使い方、`ament_flake8` / `ament_pep257` による Linter テスト、`colcon test` の実行と結果の読み方、GitHub Actions での CI 自動実行を扱います。

- 学習ファイル: `docs/tutorials/18_testing_ros2.md`
- 参照ファイル: `src/drone_sim/test/test_pid.py`
- 参照ファイル: `src/drone_sim/test/test_math_utils.py`
- 参照ファイル: `src/drone_sim/test/test_battery_monitor.py`

### ステップ 21: マルチロボット通信と DDS Discovery（60 分）

`ROS_DOMAIN_ID` によるネットワーク分離、DDS の Simple Discovery と Discovery Server の使い分け、Docker コンテナ間通信の設定を学びます。既存の `swarm.launch.py` と `multi_robot.launch.py` を複数ドメインで動かす実践も行います。

- 学習ファイル: `docs/tutorials/21_multi_robot_communication.md`
- 参照ファイル: `src/drone_sim/launch/swarm.launch.py`
- 参照ファイル: `src/ground_robot_sim/launch/multi_robot.launch.py`
- 参照ファイル: `src/drone_sim/launch/formation_demo.launch.py`
- 参照ファイル: `docker/compose.multi.yml`

### ステップ 22: MoveIt2 / マニピュレータ経路計画入門（60 分）

MoveIt2 の Planning Pipeline、Kinematics、Collision Detection、Planning Scene の役割を学びます。`manipulator_sim` の URDF を MoveIt Setup Assistant に読み込ませ、生成した設定パッケージと `moveit_py` から計画軌道を作り、`moveit_trajectory_bridge` 経由で既存シミュレータを動かします。

- 学習ファイル: `docs/tutorials/22_moveit2_manipulator_planning.md`
- 参照ファイル: `src/manipulator_sim/urdf/planar_manipulator.urdf`
- 参照ファイル: `src/manipulator_sim/manipulator_sim/moveit_trajectory_bridge.py`
- 参照ファイル: `src/manipulator_sim/launch/moveit_bridge_demo.launch.py`

---

## 既存パッケージとの関係

`ros2_learning` パッケージは最小構成のサンプルコードです。各チュートリアルで概念を理解したら、既存のシミュレーションパッケージで実際のシステムでの使われ方を確認することを強く推奨します。

```
ros2_learning（基礎を学ぶ）
    │
    ├── drone_sim        : Publisher/TF/パラメータの実例
    │   └── sim_drone.py  → odom, pose, imu をパブリッシュ
    │
    ├── ground_robot_sim : サービス/アクションの実例
    │   └── navigate_waypoints_server.py → NavigateWaypoints アクション
    │   └── ground_robot_node.py → emergency_stop サービス
    │
    ├── manipulator_sim  : カスタム型 / 制御ループの実例
    │
    ├── sensor_fusion_sim: ライフサイクルノードの実例
    │   └── lifecycle_data_recorder → 状態遷移管理
    │
    ├── sample_interfaces: カスタム msg/srv/action 定義の実例
    │   ├── RobotStatus.msg
    │   ├── GetRobotStatus.srv
    │   └── NavigateWaypoints.action
    │
    └── nav2_learning   : Navigation2 の概念を学ぶ実装例
        ├── simple_map_publisher.py → OccupancyGrid の生成と配信
        ├── simple_path_planner.py → A* 経路計画
        ├── simple_path_follower.py → Pure Pursuit 経路追従
        ├── nav2_waypoint_client.py → Nav2 アクションクライアント
        └── costmap_monitor.py → コストマップの観察
```

---

## Navigation2 学習の進め方

Navigation2 のチュートリアル（ステップ 7〜11）は、ステップ 1〜6 の基礎知識を前提としています。
特に以下の概念が重要です:

- **トピック通信**（ステップ 1）: Nav2 の全コンポーネントがトピックで通信します
- **アクション**（ステップ 2）: Nav2 のナビゲーション指令はアクションインターフェースです
- **TF**（ステップ 4）: Nav2 は map→odom→base_link の TF チェーンを必須とします
- **ライフサイクルノード**（ステップ 6）: Nav2 の各サーバーはライフサイクルノードです

ステップ 7 から順番に進むことを推奨しますが、特定のコンポーネントだけを学びたい場合は以下を参考にしてください:

| 興味のあるトピック | 推奨ステップ |
|------------------|------------|
| Nav2 の全体像だけ知りたい | ステップ 7 のみ |
| マップ表現を理解したい | ステップ 7 → 8 |
| 経路計画アルゴリズムを学びたい | ステップ 7 → 8 → 9 |
| ロボット制御を実装したい | ステップ 7 → 9 → 10 |
| Nav2 の仕組みを完全に理解したい | ステップ 7 → 8 → 9 → 10 → 11（順番通り） |

---

## 効果的な学習のヒント

1. **手を動かす**: コードを読むだけでなく、必ず実際に動かしてみましょう。ログ出力を見るだけでも理解が深まります。
2. **コマンドラインツールを使う**: `ros2 topic echo`、`ros2 service call`、`ros2 node info` などの CLI ツールは、システムの状態をリアルタイムで確認するのに非常に便利です。
3. **コードを改造する**: サンプルコードのパラメータ値や処理内容を変えてみることで、各要素の役割が体感的に理解できます。
4. **既存パッケージを読む**: `drone_sim` や `ground_robot_sim` のコードは、実際のロボットシステムでの ROS 2 の使い方を示しています。チュートリアルを終えたら積極的に読んでみましょう。
5. **エラーを恐れない**: ビルドエラーや実行時エラーのメッセージは情報の宝庫です。エラー文をよく読むと解決策が見つかることがほとんどです。

---

## 確認チェックリスト

このチェックリストを使って、チュートリアルを始める前に開発環境が正しくセットアップされているかを確認してください。

### 環境セットアップの確認

- [ ] ROS 2 Jazzy のソースを読み込めることを確認する

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

期待される出力例:

```
ros2 jazzy
```

- [ ] ワークスペースをビルドできることを確認する

```bash
cd ~/Ros2Sample
colcon build --packages-select ros2_learning sample_interfaces
```

期待される出力例:

```
Starting >>> sample_interfaces
Finished <<< sample_interfaces [...]
Starting >>> ros2_learning
Finished <<< ros2_learning [...]

Summary: 2 packages finished [...]
```

- [ ] インストール済みパッケージが認識されることを確認する

```bash
source install/setup.bash
ros2 pkg list | grep ros2_learning
ros2 pkg list | grep sample_interfaces
```

期待される出力例:

```
ros2_learning
sample_interfaces
```

- [ ] `ros2_learning` パッケージの実行ファイルが見えることを確認する

```bash
ros2 pkg executables ros2_learning
```

期待される出力例:

```
ros2_learning lifecycle_demo
ros2_learning minimal_action_client
ros2_learning minimal_action_server
ros2_learning minimal_publisher
ros2_learning minimal_service_client
ros2_learning minimal_service_server
ros2_learning minimal_subscriber
ros2_learning parameter_demo
ros2_learning tf_broadcaster_demo
ros2_learning tf_listener_demo
```

- [ ] カスタムインターフェースが認識されることを確認する

```bash
ros2 interface list | grep sample_interfaces
```

期待される出力例:

```
sample_interfaces/action/NavigateWaypoints
sample_interfaces/msg/RobotStatus
sample_interfaces/srv/GetRobotStatus
```

### 完了条件

上記のコマンドがすべてエラーなく動作し、期待される出力が確認できれば、チュートリアル 01 に進む準備ができています。

### トラブルシューティング

**`ros2: command not found` が表示される場合**

ROS 2 のセットアップスクリプトを読み込んでいない可能性があります。以下を実行してください:

```bash
source /opt/ros/jazzy/setup.bash
```

毎回実行する手間を省くには `~/.bashrc` に追記します:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**`colcon build` でエラーが出る場合**

パッケージの依存関係が不足している可能性があります。以下を試してください:

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**`ros2 pkg list` にパッケージが表示されない場合**

`source install/setup.bash` を実行し忘れている可能性があります。ビルド後は必ずこのコマンドを実行してください。ターミナルを新しく開いた場合も同様です。
