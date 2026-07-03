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
| `simple_occupancy_mapper` | log-odds によるオンライン占有格子地図マッピング（SLAM入門） |

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

### 占有格子地図マッピングデモ

```bash
ros2 launch nav2_learning occupancy_mapping_demo.launch.py
```

## 占有格子地図マッピング（SLAM入門）

### 概要

`simple_occupancy_mapper` は、LiDAR スキャンとオドメトリから log-odds 方式でオンラインに
`nav_msgs/OccupancyGrid` を構築するノードです。`ground_robot_sim` のロボットを走らせながら
未知環境の地図を少しずつ埋めていく、SLAM の「マッピング」部分だけを取り出した学習用の簡易実装です。
オドメトリを真値として扱うため、位置推定（ローカリゼーション）やループクロージャは行いません。

### ノード説明

`simple_occupancy_mapper` は `scan`（`sensor_msgs/LaserScan`）と `odom`（`nav_msgs/Odometry`）を
購読し、各スキャンを `nav2_learning.mapping_utils.integrate_scan` で log-odds グリッドへ統合します。
一定周期で `nav2_learning.mapping_utils.log_odds_to_occupancy` によって確率を占有値（100/0/-1）へ変換し、
`/map` トピックへ配信します（`simple_map_publisher` と同じ Latched QoS）。起動時に `map`→`odom` の
静止 TF を配信し、オドメトリをそのまま地図座標として扱います。

### パラメータ表

| パラメータ | デフォルト値 | 説明 |
|-----------|------------|------|
| `map_width` | `220` | グリッド幅[セル] |
| `map_height` | `220` | グリッド高さ[セル] |
| `resolution` | `0.05` | 1セルのサイズ[m/cell] |
| `origin_x` / `origin_y` | `-5.5` | マップ原点[m]（ground_robot_sim のワールド±5.0mをカバー） |
| `publish_rate` | `1.0` | `/map` 配信レート[Hz] |
| `hit_log_odds` | `0.85` | ヒット（障害物検出）時のlog-odds加算量 |
| `miss_log_odds` | `-0.4` | ミス（自由空間通過）時のlog-odds加算量 |
| `log_odds_min` / `log_odds_max` | `-4.0` / `4.0` | log-odds値の飽和範囲 |
| `occupied_threshold` | `0.65` | この確率を超えたら占有(100)とみなす |
| `free_threshold` | `0.35` | この確率を下回ったら自由(0)とみなす |
| `laser_offset_x` | `0.0` | センサの前方オフセット[m]（ground_robot_node はレイをbase位置から飛ばすため既定値は0.0） |
| `map_frame` | `'map'` | マップの座標フレーム名 |
| `scan_throttle` | `1` | Nスキャンに1回統合する（CPU負荷と更新頻度のトレードオフ） |

### 実行方法

```bash
ros2 launch nav2_learning occupancy_mapping_demo.launch.py
```

`use_patrol`（既定 `true`）を `false` にすると `diff_drive_patrol` を起動せず、代わりに
`ros2 run ground_robot_sim teleop_keyboard` を別ターミナルで実行して手動走行させながら
マッピングを試せます。`use_rviz`（既定 `true`）で RViz2 の自動起動を無効化できます。

```bash
# 手動走行でマッピングしたい場合
ros2 launch nav2_learning occupancy_mapping_demo.launch.py use_patrol:=false
# 別ターミナルで
ros2 run ground_robot_sim teleop_keyboard
```

より詳しい解説は [チュートリアル 08: マップとコストマップ](../../docs/tutorials/08_costmap_and_map.md)
の「発展」セクションを参照してください。

## チュートリアル

詳細な学習ガイドは [docs/tutorials/](../../docs/tutorials/) を参照してください:

- [07: Navigation2 の全体像](../../docs/tutorials/07_nav2_overview.md)
- [08: マップとコストマップ](../../docs/tutorials/08_costmap_and_map.md)
- [09: 経路計画](../../docs/tutorials/09_path_planning.md)
- [10: コントローラーと経路追従](../../docs/tutorials/10_nav2_controller.md)
- [11: ビヘイビアツリー入門](../../docs/tutorials/11_behavior_tree.md)
