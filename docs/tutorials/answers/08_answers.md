# チュートリアル 8 解答例: マップとコストマップ

## 演習 1: 障害物の配置を変更する

### 考え方

`simple_map_publisher.py` は `data` 配列に値を書き込んでいます。障害物セルに `100` を、空きセルに `0` を設定することで任意の形状を表現できます。重要なのはインデックス計算 `row * width + col` を正しく使うことです。

### 解答コード

`src/nav2_learning/nav2_learning/simple_map_publisher.py` の障害物定義部分を以下のように変更します:

```python
# L 字型の障害物（縦ライン + 横ライン）
obstacles = [
    # 縦のライン（col=5, row=5〜8）
    (5, 5), (5, 6), (5, 7), (5, 8),
    # 横のライン（col=6〜8, row=8）
    (6, 8), (7, 8), (8, 8),
]

# data 配列に反映する例
for (col, row) in obstacles:
    index = row * self.width + col
    if 0 <= index < len(data):
        data[index] = 100
```

### 確認方法

```bash
colcon build --packages-select nav2_learning
source install/setup.bash
ros2 run nav2_learning simple_map_publisher
```

別ターミナルで RViz を起動し、`/map` トピックを追加すると L 字型の黒い障害物が表示されます。`ros2 topic echo /map --once` で `data` 配列の `100` の位置も確認できます。

### ポイント

- `width = 20`、`height = 20` の場合、有効なセルインデックスは `0` から `399`（= 20×20−1）
- `col` は横方向（x）、`row` は縦方向（y）に対応する
- 境界外アクセスを防ぐため `0 <= index < len(data)` のチェックを忘れずに

---

## 演習 2: resolution を変更して比較する

### 考え方

`resolution` を変えると同じ物理空間を表現するためのセル数が変わります。物理サイズが固定（例: 2m × 2m の部屋）であれば、解像度を細かくするほどセル数が増え、粗くすると減ります。

### 解答

`simple_map_publisher.py` の `__init__` または地図生成部分で `resolution` を変更します:

```python
# 5cm 解像度（細かい）
self.resolution = 0.05
self.width  = int(2.0 / self.resolution)   # = 40
self.height = int(2.0 / self.resolution)   # = 40

# 10cm 解像度（学習用デフォルト）
self.resolution = 0.1
self.width  = 20
self.height = 20

# 20cm 解像度（粗い）
self.resolution = 0.2
self.width  = int(2.0 / self.resolution)   # = 10
self.height = int(2.0 / self.resolution)   # = 10
```

### 期待される出力の違い

`ros2 topic echo /map --once` の `info` フィールド比較:

| resolution | width | height | data 配列サイズ |
|-----------|-------|--------|--------------|
| 0.05      | 40    | 40     | 1600         |
| 0.1       | 20    | 20     | 400          |
| 0.2       | 10    | 10     | 100          |

RViz での見え方の違い:
- `resolution = 0.05`: 障害物の輪郭が細かく、滑らかに見える
- `resolution = 0.2`: 障害物がブロック状に粗く見える

### ポイント

resolution を細かくすれば精度は上がりますが、`data` 配列のサイズが増えてメモリと計算時間がかかります。Nav2 の実環境では `0.05`（5cm）がよく使われますが、広いエリアでは `0.1` や `0.2` を選ぶこともあります。

---

## 演習 3: 座標変換を手計算で確認する

### 考え方

`world_to_grid` 関数の計算式を追って、ワールド座標からグリッドインデックスを導きます。

### 手計算の答え

与えられた条件:
- `resolution = 0.1`
- `width = 20`
- `origin = (-1.0, -1.0)`
- 変換したい座標: `(x, y) = (0.85, 0.55)`

```
col = floor((x - origin_x) / resolution)
    = floor((0.85 - (-1.0)) / 0.1)
    = floor(1.85 / 0.1)
    = floor(18.5)
    = 18

row = floor((y - origin_y) / resolution)
    = floor((0.55 - (-1.0)) / 0.1)
    = floor(1.55 / 0.1)
    = floor(15.5)
    = 15

配列インデックス = row * width + col
               = 15 * 20 + 18
               = 300 + 18
               = 318
```

### 答え: `col = 18`、`row = 15`、インデックス = `318`

### map_utils.py で検証するコード

```python
from nav2_learning.map_utils import world_to_grid

col, row = world_to_grid(
    x=0.85, y=0.55,
    origin_x=-1.0, origin_y=-1.0,
    resolution=0.1
)
print(f"col={col}, row={row}")  # → col=18, row=15

index = row * 20 + col
print(f"index={index}")  # → index=318
```

### ポイント

`int()` と `floor()` の違いに注意してください。正の値では同じ結果になりますが、負の値では異なります（例: `floor(-0.3) = -1`、`int(-0.3) = 0`）。ワールド座標がマップ原点より小さい場合（`x < origin_x`）は負の `col` が生まれるため、境界チェックが必要です。
