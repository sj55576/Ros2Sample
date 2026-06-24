# チュートリアル 7: Navigation2 の全体像

## 学習目標

- Nav2 のアーキテクチャとコンポーネント構成を理解する
- 各コンポーネント（Planner、Controller、BT Navigator 等）の役割を説明できる
- 既存の `ground_robot_sim` カスタム実装と Nav2 の対応関係を理解する

---

## Nav2 とは何か

Navigation2（Nav2）は ROS 2 向けの自律移動ロボット用ナビゲーションフレームワークです。目標地点への経路計画・経路追従・障害物回避・リカバリ動作を統合的に提供し、倉庫ロボット・サービスロボット・自動搬送車（AGV）などの実用システムに広く使われています。

Nav2 の主な特徴は以下の通りです:

- **プラグインアーキテクチャ**: 経路計画アルゴリズムや制御アルゴリズムをプラグインとして差し替え可能
- **Behavior Tree によるタスク管理**: 複雑なナビゲーションシーケンスとリカバリをBTで宣言的に記述
- **ライフサイクルノード**: 各サーバーはライフサイクル管理付きで、起動・設定・終了を安全に制御可能
- **標準インターフェース**: `nav_msgs/OccupancyGrid`、`nav_msgs/Path`、`geometry_msgs/Twist` など標準メッセージ型を使用

---

## Nav2 のアーキテクチャ

```
┌─────────────────────────────────────────────────────────┐
│                   Behavior Tree (bt_navigator)           │
│  ┌──────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ Planner  │  │  Controller  │  │  Recovery/Smoother│  │
│  │ Server   │  │  Server      │  │  Servers          │  │
│  └────┬─────┘  └──────┬───────┘  └──────────────────┘  │
│       │               │                                  │
│  ┌────▼───────────────▼──────────────────────┐          │
│  │         Costmap 2D (global / local)        │          │
│  └─────────────────┬─────────────────────────┘          │
└────────────────────┼────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│  Robot: odom, scan, TF (map→odom→base_link→sensors)    │
└─────────────────────────────────────────────────────────┘
```

BT Navigator が全体を統括し、Planner Server で経路を計算、Controller Server でロボットを動かします。何か問題が起きたときは Recovery Server が自動的に回復動作を実行します。

---

## ground_robot_sim との対応関係

このリポジトリの `ground_robot_sim` は Nav2 を使わずにカスタム実装でナビゲーションを実現しています。両者の対応関係を理解することで、Nav2 が何を解決しているかが明確になります。

| 機能 | ground_robot_sim（カスタム実装） | Nav2 |
|------|--------------------------------|------|
| 経路追従 | `waypoint_follower.py`（PID 制御） | Controller Server（DWB / RPP / MPPI） |
| 障害物回避 | `lidar_obstacle_avoid.py`（直接 LiDAR） | Costmap 2D + Controller |
| 経路計画 | なし（事前定義ウェイポイント） | Planner Server（NavFn / Smac） |
| タスク管理 | なし | Behavior Tree |
| リカバリ | なし | Recovery Server（回転 / 後退 / 待機） |

`ground_robot_sim` のカスタム実装は理解しやすい一方、障害物の多い環境や複雑なミッションには限界があります。Nav2 を使うと、これらの課題をフレームワークとして解決できます。

---

## Nav2 の主要コンポーネント

### Planner Server

スタート地点からゴール地点までの経路を計画するコンポーネントです。グローバルコストマップ上で動作し、NavFn（Dijkstra / A* ベース）や Smac Planner（ハイブリッド A*、格子ベース）などのアルゴリズムをプラグインとして使用できます。生成した経路は `nav_msgs/Path` メッセージとして `/plan` トピックに配信されます。

### Controller Server

Planner Server が生成した経路に沿ってロボットを動かすコンポーネントです。ローカルコストマップ上でリアルタイムに動作し、`geometry_msgs/Twist` の速度コマンドを `/cmd_vel` トピックに送信します。DWB（Dynamic Window Based）、RPP（Regulated Pure Pursuit）、MPPI など複数のアルゴリズムに対応しています。

### BT Navigator

Behavior Tree を実行してナビゲーション全体のシーケンスを管理するコンポーネントです。NavigateToPose アクションのサーバーとして動作し、経路計画・経路追従・リカバリの順序と条件を BT XML ファイルで定義します。

### Costmap 2D

センサデータ（LiDAR 等）と静的マップを統合して、ロボットが走行可能な領域のコスト値マップを生成するコンポーネントです。グローバルコストマップ（全体地図）とローカルコストマップ（ロボット周辺の動的障害物）の 2 種類があります。

### Recovery Server

経路計画や経路追従が失敗した場合に自動で回復動作を実行するコンポーネントです。Spin（その場で回転）、BackUp（後退）、Wait（一時停止）、ClearCostmap（コストマップのリセット）などのリカバリ動作を提供します。

### Smoother Server

Planner が生成した経路をよりなめらかに修正するコンポーネントです。急カーブや不連続な経路を修正し、Controller Server による追従精度を向上させます。

### Waypoint Follower

複数のウェイポイントを順番にナビゲートするコンポーネントです。`NavigateWaypoints` アクションのサーバーとして動作し、各ウェイポイントで任意のタスクプラグインを実行できます。

### Velocity Smoother

Controller Server が出力する速度コマンドを平滑化するコンポーネントです。急激な加減速を防いでハードウェアへの負担を軽減し、より自然なロボット動作を実現します。

---

## TF フレームの要件

Nav2 は以下の TF チェーンが正しく配信されていることを前提としています。

```
map
└── odom
    └── base_link
        └── base_scan（または lidar 等のセンサフレーム）
```

| フレーム | 役割 | 配信者 |
|----------|------|--------|
| `map` | 静的なワールド座標系（マップ原点） | `map_server` または SLAM |
| `odom` | ドリフトを含むローカル座標系 | ロボットドライバ |
| `base_link` | ロボット本体の座標系 | ロボットドライバ |
| `base_scan` | LiDAR センサの座標系 | ロボットドライバ（静的 TF） |

`ground_robot_sim` の `ground_robot_node.py` はすでに `odom → base_link` の TF を配信しているため、`map → odom` を追加すれば Nav2 が要求する TF チェーンを満たせます。

---

## nav2_learning パッケージの位置づけ

このチュートリアルシリーズ（ステップ 7〜11）では、Nav2 の各コンポーネントを段階的に理解するために `nav2_learning` パッケージを使います。

```
nav2_learning（Nav2 の概念を学ぶ）
    │
    ├── simple_map_publisher.py  → OccupancyGrid の生成と配信（ステップ 8）
    ├── map_utils.py             → 座標変換ユーティリティ（ステップ 8）
    ├── simple_path_planner.py   → A* 経路計画の実装（ステップ 9）
    ├── simple_path_follower.py  → Pure Pursuit 経路追従（ステップ 10）
    ├── nav2_waypoint_client.py  → Nav2 アクションクライアント（ステップ 11）
    └── costmap_monitor.py       → コストマップの観察（ステップ 8）
```

簡易実装でアルゴリズムの動作原理を理解してから、本物の Nav2 スタックの設定や使い方を学ぶという順序で進めます。

---

## Step 1: Nav2 の依存関係を確認する

Nav2 がインストールされているか確認しましょう。

```bash
apt list --installed 2>/dev/null | grep nav2
```

インストールされている場合、以下のようなパッケージ一覧が表示されます:

```
ros-jazzy-nav2-bringup/...
ros-jazzy-nav2-bt-navigator/...
ros-jazzy-nav2-controller/...
ros-jazzy-nav2-costmap-2d/...
ros-jazzy-nav2-planner/...
```

インストールされていない場合は、以下のコマンドでインストールできます:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

---

## Step 2: Nav2 のトピック構成を理解する

Nav2 が使用する主なトピックを確認しておきましょう。

```bash
# Nav2 起動後に確認できるトピック一覧
ros2 topic list
```

| トピック | 型 | 役割 |
|----------|----|------|
| `/plan` | `nav_msgs/Path` | グローバルプランナーが生成した経路 |
| `/cmd_vel` | `geometry_msgs/Twist` | コントローラーが出力する速度コマンド |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | グローバルコストマップ |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | ローカルコストマップ |
| `/map` | `nav_msgs/OccupancyGrid` | 静的マップ |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL への初期位置指定 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz からのゴール指定 |
| `/behavior_tree_log` | `nav2_msgs/BehaviorTreeLog` | BT の実行ログ |

これらのトピックは `ground_robot_sim` が使う `/odom`、`/scan`、`/cmd_vel` と一部共通しています。Nav2 は `/cmd_vel` を書き込み、ロボットドライバはそれを読み取るという役割分担は変わりません。

---

## 既存パッケージでの応用

`ground_robot_sim` はすでに Nav2 互換のインターフェースを備えています。

ソースファイル: `src/ground_robot_sim/ground_robot_sim/ground_robot_node.py`

```python
# Nav2 が期待するインターフェースをすでに実装している
self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)  # 速度コマンド受信
self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)       # オドメトリ配信
self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)      # LiDAR 配信
# TF も odom → base_link を配信
```

つまり `ground_robot_sim` のロボットノードはそのまま Nav2 のコントローラーと接続できます。`cmd_vel` を受け取り、`odom` と `scan` を返す、という標準インターフェースを守っているためです。

---

## 演習問題

### 演習 1: Nav2 コンポーネントと役割の対応

以下の各説明が Nav2 のどのコンポーネントに対応するか答えてください:

1. 「スタート地点からゴール地点への衝突しない経路を計算する」
2. 「計画された経路に沿って `/cmd_vel` を送信してロボットを動かす」
3. 「LiDAR データをリアルタイムに反映してロボット周辺のコスト値を更新する」
4. 「経路追従が失敗したときにロボットをその場で回転させて再試行する」

### 演習 2: TF チェーンの確認

`ground_robot_sim` を起動して、TF チェーンを確認してみましょう:

```bash
# ターミナル 1: ground_robot_sim を起動
ros2 run ground_robot_sim ground_robot_node

# ターミナル 2: TF ツリーを確認
ros2 run tf2_tools view_frames
```

`odom → base_link` が配信されていることを確認してください。Nav2 で動かすには、ここに `map → odom` を追加する必要があります。

### 演習 3: ground_robot_sim と Nav2 の比較

`ground_robot_sim` の `waypoint_follower.py` と `lidar_obstacle_avoid.py` のコードを読んで、以下の点を考えてみましょう:

- これらのカスタム実装はどのような状況では十分か？
- どのような状況で Nav2 のような本格的なナビゲーションスタックが必要になるか？
- `ground_robot_sim` では実装されていない機能（例: リカバリ動作）を Nav2 はどのように実現しているか？
