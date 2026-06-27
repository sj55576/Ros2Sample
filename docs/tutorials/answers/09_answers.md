# チュートリアル 9 解答例: 経路計画

## 演習 1: 障害物を追加して経路変化を確認する

### 考え方

A* は障害物セル（値 `>= cost_threshold`）を通れないノードとして扱います。縦方向の壁を追加すると、スタートとゴールを直線で結ぶ経路が遮断されるため、A* は壁の端を回り込む迂回経路を生成します。これが A* の「完全性」の実証になります。

### 解答: simple_map_publisher.py への変更

```python
# src/nav2_learning/nav2_learning/simple_map_publisher.py

# 既存の障害物リストに縦方向の壁を追加
# col=10 の位置に row=0〜14 まで壁を作る
wall_obstacles = [(10, i) for i in range(0, 15)]

# 元の障害物と合わせて設定
obstacles = [
    # 既存の障害物（例）
    (3, 3), (4, 3), (5, 3),
    # 新しい縦壁（廊下を塞ぐ）
] + wall_obstacles

for (col, row) in obstacles:
    index = row * self.width + col
    if 0 <= index < len(data):
        data[index] = 100
```

### リビルドと動作確認

```bash
colcon build --packages-select nav2_learning
source install/setup.bash

# ターミナル 1
ros2 run nav2_learning simple_map_publisher

# ターミナル 2
ros2 run nav2_learning simple_path_planner

# ターミナル 3: 壁をまたぐ経路を計画
ros2 service call /plan_path nav2_learning/srv/PlanPath \
  "{start: {x: 0.0, y: 0.0}, goal: {x: 0.8, y: 0.8}}"
```

### 期待される結果

壁 `col=10` の右側（x > 0.0m）からゴールへ向かう直線経路が遮断され、壁の下端（`row=14` より下）を回り込む迂回経路が生成されます。RViz の `Path` 表示で経路が L 字型または U 字型になることを確認してください。

### ポイント

- 壁がスタートとゴールを完全に分断した場合（壁が端から端まで伸びている）、A* は「Open リストが空になった」として「経路なし」を返します
- 壁に隙間があれば、A* はそこを通る最短経路を必ず見つけます（完全性の保証）

---

## 演習 2: ヒューリスティックの効果を観察する

### 考え方（重要）

A* と Dijkstra 法の本質的な違いは「どの方向に優先的に探索するか」にあります。

**A* の場合（ヒューリスティックあり）**:
- `f(n) = g(n) + h(n)` でゴール方向のノードを優先
- ゴールに近い候補を先に調べるため、無関係な方向の探索を省略できる
- 探索ノード数が少ない = 高速

**Dijkstra 法の場合（ヒューリスティックなし）**:
- `f(n) = g(n) + 0 = g(n)` でスタートから近いノードを全方向均等に探索
- ゴール方向の優先がないため、同じコストのノードをすべて展開してしまう
- 探索ノード数が多い = 低速（ただし必ず最短経路を見つける）

この違いは障害物のない広いマップで特に顕著です。ゴールが遠いほど、Dijkstra は「ゴールと逆方向のノード」まで探索してしまいます。

### 解答: simple_path_planner.py のヒューリスティック変更

```python
# src/nav2_learning/nav2_learning/simple_path_planner.py

# --- 変更前（A* のヒューリスティック）---
def heuristic(col1, row1, col2, row2, resolution):
    """ユークリッド距離をヒューリスティックとして使用"""
    dx = (col2 - col1) * resolution
    dy = (row2 - row1) * resolution
    return math.sqrt(dx * dx + dy * dy)

# --- 変更後（Dijkstra 法と同等）---
def heuristic(col1, row1, col2, row2, resolution):
    """ヒューリスティックを 0 にする（= Dijkstra 法）"""
    return 0.0
```

### ログで探索ノード数を確認する

`simple_path_planner.py` がノード数をログ出力している場合:

```bash
# A* 実行時のログ例
[INFO] [simple_path_planner]: Path found. Explored nodes: 42, Path length: 12 points

# Dijkstra 実行時のログ例（同じマップ・同じスタート/ゴール）
[INFO] [simple_path_planner]: Path found. Explored nodes: 187, Path length: 12 points
```

探索ノード数（Explored nodes）が大幅に増えていることが確認できます。経路の長さ（Path length）は同じです。A* は経路の最適性を保ちながら探索範囲を絞っています。

### ノード数比較の可視化イメージ

```
A* の探索（ヒューリスティックあり）:   Dijkstra の探索（ヒューリスティックなし）:
S . . . X . . .                        S * * * X * * *
* . . . . . . .                        * * * * * * * *
* * . . . . . .                        * * * * * * * .
. * * * * * G .                        . * * * * * G .

* = 探索されたセル（A* は少ない）       * = 探索されたセル（Dijkstra は多い）
```

### ポイント

- 生成される経路は A* も Dijkstra も同じ最短経路（h(n)=0 はアドミッシブルなので最適性は保たれる）
- 違いは「探索効率」だけ
- 広いマップや障害物の少ない環境ほど差が大きく出る
- 実際の Nav2（NavFn）もヒューリスティックの係数（`use_astar` フラグ）で切り替え可能

---

## 演習 3: 経路の長さを計算する

### 考え方

`/plan` トピックは `nav_msgs/Path` 型で、`poses` フィールドに `PoseStamped` の配列が格納されています。隣接する 2 点間のユークリッド距離を順番に積算することで経路の総距離が計算できます。

### 完全な解答コード: path_length_calculator.py

以下のファイルを `src/nav2_learning/nav2_learning/path_length_calculator.py` として作成します:

```python
#!/usr/bin/env python3
"""
経路の総距離を計算するサブスクライバーノード。
/plan トピック（nav_msgs/Path）を受け取り、
隣接ウェイポイント間の距離を積算して総距離を ROS ログに出力する。
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class PathLengthCalculator(Node):
    """
    /plan トピックをサブスクライブして経路長を計算するノード。
    """

    def __init__(self):
        super().__init__('path_length_calculator')

        self._subscription = self.create_subscription(
            Path,
            '/plan',
            self._path_callback,
            10  # QoS depth
        )
        self.get_logger().info('PathLengthCalculator が起動しました。/plan を待機中...')

    def _path_callback(self, msg: Path) -> None:
        """
        /plan トピックを受信したときに呼ばれるコールバック。
        経路の総距離を計算してログに出力する。
        """
        length = self._calculate_path_length(msg)
        num_points = len(msg.poses)

        self.get_logger().info(
            f'経路を受信しました: '
            f'ウェイポイント数 = {num_points}, '
            f'総距離 = {length:.4f} m'
        )

    @staticmethod
    def _calculate_path_length(path: Path) -> float:
        """
        Path メッセージの総距離をメートル単位で返す。

        Args:
            path: nav_msgs/Path メッセージ

        Returns:
            総距離（m）。ウェイポイントが 1 点以下の場合は 0.0。
        """
        poses = path.poses
        if len(poses) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(poses)):
            prev = poses[i - 1].pose.position
            curr = poses[i].pose.position
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            total += math.sqrt(dx * dx + dy * dy)

        return total


def main(args=None):
    rclpy.init(args=args)
    node = PathLengthCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### setup.py への登録

`src/nav2_learning/setup.py` の `entry_points` に追加します:

```python
entry_points={
    'console_scripts': [
        # ... 既存のエントリ ...
        'path_length_calculator = nav2_learning.path_length_calculator:main',
    ],
},
```

### 動作確認

```bash
# ビルド
colcon build --packages-select nav2_learning
source install/setup.bash

# ターミナル 1: マップパブリッシャー
ros2 run nav2_learning simple_map_publisher

# ターミナル 2: 経路計画ノード
ros2 run nav2_learning simple_path_planner

# ターミナル 3: 経路長計算ノード（先に起動しておく）
ros2 run nav2_learning path_length_calculator

# ターミナル 4: 経路計画を実行
ros2 service call /plan_path nav2_learning/srv/PlanPath \
  "{start: {x: 0.0, y: 0.0}, goal: {x: 0.8, y: 0.8}}"
```

### 期待される出力

```
[INFO] [path_length_calculator]: PathLengthCalculator が起動しました。/plan を待機中...
[INFO] [path_length_calculator]: 経路を受信しました: ウェイポイント数 = 13, 総距離 = 1.1314 m
```

### 出力値の考察

スタート `(0.0, 0.0)` からゴール `(0.8, 0.8)` への直線距離は `√(0.8²+0.8²) ≈ 1.131 m` です。障害物がない場合、A* が斜め移動を使った経路では理論最短距離に近い値が得られます。障害物を追加すると迂回分だけ距離が長くなることを確認してみましょう。

### ポイント

- `Path.poses` は `geometry_msgs/PoseStamped` の配列
- `pose.position.x` / `.y` でワールド座標を取得できる（z は 2D ナビでは 0.0）
- このサブスクライバーはトピックを受信するたびに計算するため、経路が更新されるたびに新しい距離が表示される
