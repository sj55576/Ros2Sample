# nav2_learning

Navigation2 の概念を段階的に学ぶための学習パッケージです。

## 概要

このパッケージは、Navigation2 の核心的な概念を簡易実装で理解するためのサンプルコードを提供します。
Nav2 を使わずに動作する独立した実装で、コストマップ・経路計画・経路追従の仕組みを学びます。

## ノード一覧

| ノード | 説明 |
|--------|------|
| `simple_map_publisher` | OccupancyGrid マップの生成と配信 |
| `simple_path_planner` | A* アルゴリズムによる経路計画 |
| `simple_path_follower` | Pure Pursuit による経路追従制御 |
| `nav2_waypoint_client` | Nav2 NavigateToPose アクションクライアント |
| `costmap_monitor` | コストマップデータの観察・解析 |

## 前提条件

- ROS 2 Jazzy
- ground_robot_sim パッケージ（経路追従デモで使用）

## ビルド

```bash
source /opt/ros/jazzy/setup.bash
cd Ros2Sample
colcon build --packages-select nav2_learning
source install/setup.bash
```

## デモの実行

### マップ生成デモ

```bash
ros2 launch nav2_learning simple_map_demo.launch.py
```

### 経路計画・追従デモ

```bash
ros2 launch nav2_learning simple_planning_demo.launch.py
```

### Nav2 ウェイポイントクライアントデモ

Nav2 スタックを別ターミナルで起動した後に実行してください。

```bash
ros2 launch nav2_learning nav2_waypoint_demo.launch.py
```

## チュートリアル

詳細な学習ガイドは [docs/tutorials/](../../docs/tutorials/) を参照してください:

- [07: Navigation2 の全体像](../../docs/tutorials/07_nav2_overview.md)
- [08: マップとコストマップ](../../docs/tutorials/08_costmap_and_map.md)
- [09: 経路計画](../../docs/tutorials/09_path_planning.md)
- [10: コントローラーと経路追従](../../docs/tutorials/10_nav2_controller.md)
- [11: ビヘイビアツリー入門](../../docs/tutorials/11_behavior_tree.md)
