# チュートリアル 7 演習解答: Navigation2 の全体像

## 演習 1: Nav2 コンポーネントと役割の対応

### 考え方

Nav2 の各コンポーネントは「責務の分離」の原則で設計されています。コンポーネントの名前から役割を推測しようとすると間違えやすいですが、「何のデータを入力として、何を出力するか」というデータフローで考えると整理しやすくなります。

### 解答

**1. 「スタート地点からゴール地点への衝突しない経路を計算する」**

→ **Planner Server**

Planner Server はグローバルコストマップ上で A*（NavFn）や Smac Planner などのアルゴリズムを使い、スタートからゴールまでの衝突しない経路を計算します。出力は `nav_msgs/Path` メッセージとして `/plan` トピックに配信されます。

Planner は「全体地図を見て最適なルートを引く」役割です。リアルタイム制御は担いません。

**2. 「計画された経路に沿って `/cmd_vel` を送信してロボットを動かす」**

→ **Controller Server**

Controller Server は Planner が生成した経路に沿ってロボットを動かすための速度コマンド（`geometry_msgs/Twist`）を計算し、`/cmd_vel` トピックに送信します。ローカルコストマップ（リアルタイムに更新される周辺マップ）を使って局所的な障害物を回避しながら経路を追従します。

Controller は「今この瞬間どの方向にどの速度で動くか」を担当します。

**3. 「LiDAR データをリアルタイムに反映してロボット周辺のコスト値を更新する」**

→ **Costmap 2D (ローカルコストマップ)**

Costmap 2D はセンサデータ（LiDAR 等）を受け取り、走行可能領域と障害物のコスト値マップを管理します。グローバルコストマップ（静的マップ全体）とローカルコストマップ（ロボット周辺の動的更新）の 2 種類があります。

「LiDAR データをリアルタイムに反映」はローカルコストマップの役割です。

**4. 「経路追従が失敗したときにロボットをその場で回転させて再試行する」**

→ **Recovery Server**

Recovery Server は経路計画や経路追従が失敗した場合に自動で回復動作を実行します。「その場で回転」は `Spin` リカバリに対応します。他にも `BackUp`（後退）、`Wait`（一時停止）、`ClearCostmap`（コストマップリセット）などのリカバリ動作を提供します。

### データフローで整理する

```
NavigateToPose ゴール
        ↓
  BT Navigator（全体の指揮）
        ↓
  Planner Server ← グローバルコストマップ（静的マップ）
  （経路計算: nav_msgs/Path を生成）
        ↓
  Controller Server ← ローカルコストマップ（LiDAR リアルタイム更新）
  （速度制御: geometry_msgs/Twist を /cmd_vel に送信）
        ↓
    ロボット
        ↓ (何か失敗したら)
  Recovery Server
  （Spin / BackUp / Wait / ClearCostmap）
```

---

## 演習 2: TF チェーンの確認

### 考え方

Nav2 が動作するには `map → odom → base_link` という TF チェーンが必要です。`ground_robot_sim` はすでに `odom → base_link` を配信しているため、Nav2 との接続に必要なのは `map → odom` の変換を追加することだけです。

この演習では、まず現状（`odom → base_link` のみ）を確認し、Nav2 で完全に動かすために何が不足しているかを理解することが目的です。

### 手順と解答

**ターミナル 1**: `ground_robot_node` を起動します。

```bash
ros2 run ground_robot_sim ground_robot_node
```

**ターミナル 2**: TF ツリーを確認します。

```bash
ros2 run tf2_tools view_frames
```

生成される `frames.pdf` の TF ツリー構造:

```
odom
└── base_link
```

`odom → base_link` が配信されていることを確認できます。

TF が配信されているか直接確認する場合:

```bash
ros2 topic echo /tf --once
```

期待される出力:

```
transforms:
- header:
    frame_id: odom
  child_frame_id: base_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      w: 1.0
---
```

### Nav2 が必要とする完全な TF チェーン

Nav2 が必要とする TF チェーン:

```
map
└── odom
    └── base_link
        └── base_scan（LiDAR センサフレーム）
```

`ground_robot_sim` で提供されている部分:
- `odom → base_link`: 提供済み（`ground_robot_node.py` が配信）

不足している部分:
- `map → odom`: AMCL（確率的自己位置推定）または `map_server` が配信する

実際の Nav2 スタックでは `map_server` が静的マップを配信し、`amcl` がロボットの自己位置を推定して `map → odom` の変換を更新します。

### map → odom を手動で静的配信する場合（学習目的）

Nav2 を使わずに TF チェーンを完成させる場合:

```bash
# map → odom の静的変換を配信する（ロボットが原点にいる前提）
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 map odom
```

これで `map → odom → base_link` のチェーンが完成しますが、実際の自己位置推定は行われないため、ロボットが動いても `map → odom` は更新されません。本格的な使用には AMCL が必要です。

---

## 演習 3: ground_robot_sim と Nav2 の比較

### 考え方

「カスタム実装と既成フレームワークのどちらを使うか」はロボット開発で常に問われる判断です。シンプルな実装は理解しやすく改造しやすい一方、複雑な状況への対応力に限界があります。

### 解答と考察

**これらのカスタム実装はどのような状況では十分か？**

`ground_robot_sim` の `waypoint_follower.py`（PID 制御）と `lidar_obstacle_avoid.py`（直接 LiDAR）は以下の状況で十分機能します:

- 事前にウェイポイントが定義されており、経路計画が不要な場合
- 障害物が少なく、単純な回避（減速・停止）で十分な場合
- 環境が静的で変化が少ない場合
- デモや教育目的など、動作の理解しやすさが重要な場合
- 小規模なプロジェクトで開発速度が優先される場合

**どのような状況で Nav2 のような本格的なナビゲーションスタックが必要になるか？**

以下の状況では Nav2 の導入を検討すべきです:

- **動的障害物への対応**: 人や他のロボットが頻繁に通路を塞ぐ環境では、コストマップの動的更新と再経路計画が不可欠
- **グローバル経路計画が必要**: 事前に経路が決まっておらず、ゴール指定に応じて最適経路を計算する必要がある場合
- **リカバリが必要**: 狭い場所でのスタックや障害物への衝突後の自動回復が必要な場合
- **複雑なミッション**: 複数のウェイポイントを条件に応じて選択するなど、BT による複雑なタスク管理が必要な場合
- **マルチロボット**: 複数ロボットがコストマップを共有して協調する場合

**`ground_robot_sim` では実装されていない機能を Nav2 はどのように実現しているか？**

| 未実装機能 | Nav2 での実現方法 |
|------------|-----------------|
| **リカバリ動作** | Recovery Server が `Spin` / `BackUp` / `Wait` / `ClearCostmap` を自動実行。BT Navigator が失敗を検知して Recovery にフォールバック |
| **グローバル経路計画** | Planner Server が NavFn（A*）や Smac Planner を使い、静的マップ全体から最適経路を計算 |
| **動的障害物考慮** | ローカルコストマップが LiDAR を継続的に取り込み、Controller Server がリアルタイムに経路調整 |
| **タスク管理** | Behavior Tree (BT Navigator) が遷移条件を XML で宣言的に定義し、失敗・リトライ・フォールバックを管理 |

### ground_robot_sim のコードで理解を深める

`waypoint_follower.py` の核心部分（擬似コード）:

```python
# 事前定義ウェイポイントへの移動（単純な角度制御 + 直進）
error_angle = target_angle - current_angle
cmd_vel.angular.z = Kp * error_angle  # P 制御
cmd_vel.linear.x = 0.3 if abs(error_angle) < threshold else 0.0
```

Nav2 の Controller Server（DWB）との違い:
- DWB はローカルコストマップ上でサンプリングした複数の速度候補を評価して最適値を選ぶ
- ロボットのキネマティクス（最大加速度・回転半径）を考慮した物理的に実現可能な速度を出力する
- 障害物へのコスト評価が経路追従に組み込まれている

`lidar_obstacle_avoid.py` との違い:
- `lidar_obstacle_avoid.py` は LiDAR のレーザー距離を直接チェックして減速するシンプルな実装
- Costmap 2D はセンサデータを格子マップに変換し、膨張（Inflation）処理で安全マージンを設けた上でコスト値を管理するより洗練されたアプローチ

### まとめ

`ground_robot_sim` のカスタム実装を先に理解することで、Nav2 が「何を解決しようとしているか」が明確に見えるようになります。Nav2 はゼロから書く必要があった部分（経路計画・局所回避・リカバリ・タスク管理）を汎用的なフレームワークとして提供しています。プロジェクトの規模と要件に応じて、カスタム実装か Nav2 活用かを判断できることが重要です。
