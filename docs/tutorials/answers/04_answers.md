# チュートリアル 4 演習解答: TF2 と座標変換

## 演習 1: 軌道半径の変更

### 考え方

`orbit_radius` パラメータは `tf_broadcaster_demo.py` のタイマーコールバックで円軌道の計算に使われています。ROS2 のパラメータサーバーを通じて実行時に変更できるため、ノードを再起動せずに値を変えて動作を観察できます。

ポイントは「パラメータを変更したあとに TF ツリーを再生成して比較する」という流れです。

### 手順と解答

**ターミナル 1**: ブロードキャスターを起動します。

```bash
ros2 run ros2_learning tf_broadcaster_demo
```

**ターミナル 2**: デフォルト（半径 2.0 m）の TF ツリーを生成します。

```bash
ros2 run tf2_tools view_frames
mv frames.pdf frames_radius2.pdf
```

次に、パラメータを変更します。

```bash
ros2 param set /tf_broadcaster_demo orbit_radius 5.0
```

期待される出力:
```
Set parameter successful
```

変更後の TF ツリーを再生成します（`tf2_echo` でリアルタイム確認も可能）。

```bash
ros2 run tf2_ros tf2_echo world learning_robot
```

期待される出力（x, y の値が半径 5.0 m 付近になる）:
```
At time ...
- Translation: [4.9..., 0.9..., 0.0]
- Rotation: in Quaternion [0.0, 0.0, ...]
```

半径を小さくした場合（0.5 m）:

```bash
ros2 param set /tf_broadcaster_demo orbit_radius 0.5
```

`tf2_echo` での Translation の x, y が 0.5 m 付近の値に変わることを確認できます。

### 学んだこと

- ROS2 のパラメータは実行時に `ros2 param set` で変更できる
- `tf_broadcaster_demo` は毎タイマーコールバックでパラメータを参照しているため、変更が即座に反映される
- `tf2_echo` は変換値をリアルタイムで確認する際に便利なデバッグコマンド

---

## 演習 2: sensor_frame の位置を変更する

### 考え方

`_publish_static_transform` メソッドは `StaticTransformBroadcaster` を使って `learning_robot → sensor_frame` の変換を配信しています。静的変換はノード起動時に一度だけ配信されるため、変更するにはコードを編集してリビルドが必要です。

「Z 方向 +0.1m」から「X 方向 +0.5m」に変更するには、`TransformStamped` の `translation.z = 0.1` を `translation.x = 0.5` に書き換えます。

### 実装

対象ファイル: `src/ros2_learning/ros2_learning/tf_broadcaster_demo.py`

変更前の静的変換コード（`_publish_static_transform` メソッド内）:

```python
static_msg = TransformStamped()
static_msg.header.stamp = self.get_clock().now().to_msg()
static_msg.header.frame_id = self._child_frame   # 'learning_robot'
static_msg.child_frame_id = 'sensor_frame'
static_msg.transform.translation.z = 0.1         # ← ここを変更する
static_msg.transform.rotation.w = 1.0
self._static_broadcaster.sendTransform(static_msg)
```

変更後:

```python
static_msg = TransformStamped()
static_msg.header.stamp = self.get_clock().now().to_msg()
static_msg.header.frame_id = self._child_frame   # 'learning_robot'
static_msg.child_frame_id = 'sensor_frame'
static_msg.transform.translation.x = 0.5         # Z から X に変更
static_msg.transform.translation.z = 0.0         # Z はゼロにリセット
static_msg.transform.rotation.w = 1.0
self._static_broadcaster.sendTransform(static_msg)
```

### リビルドと確認

```bash
colcon build --packages-select ros2_learning
source install/setup.bash
ros2 run ros2_learning tf_broadcaster_demo
```

別ターミナルで確認します。

```bash
ros2 topic echo /tf_static
```

期待される出力（`translation.x` が 0.5 になっている）:
```
transforms:
- header:
    frame_id: learning_robot
  child_frame_id: sensor_frame
  transform:
    translation:
      x: 0.5
      y: 0.0
      z: 0.0
    rotation:
      w: 1.0
```

TF リスナーを起動して距離を確認します。

```bash
ros2 run ros2_learning tf_listener_demo
```

期待されるログ（`learning_robot` の円軌道半径 2.0m に X 方向 0.5m のオフセットが加わる）:
```
[INFO] [tf_listener_demo]: sensor_frame の距離: 2.5... m  (ロボットが world の X+ 方向にいる場合)
```

### 学んだこと

- `StaticTransformBroadcaster` で配信した変換はコード変更 → リビルド → 再起動が必要
- `translation` の各軸（x, y, z）は独立して設定できる
- `tf_listener_demo` は `world → sensor_frame` の合成変換（`world → learning_robot → sensor_frame`）を取得するため、X オフセットがロボットの向きによって異なる距離として見える

---

## 演習 3: manipulator_sim の TF チェーンを確認する

### 考え方

`manipulator_sim` は 2-DOF ロボットアームをシミュレートするパッケージです。各関節の回転に応じて TF チェーンが更新されます。`view_frames` で生成される PDF には TF ツリーの構造と各フレームの更新頻度が表示されるため、どのフレームが動的か静的かを視覚的に確認できます。

### 手順と解答

**ターミナル 1**: `manipulator_sim` を起動します。

```bash
ros2 launch manipulator_sim planar_reach_demo.launch.py
```

**ターミナル 2**: TF ツリーを PDF として生成します。

```bash
ros2 run tf2_tools view_frames
```

生成される `frames.pdf` に表示される TF ツリー構造:

```
base_link
└── link1   (動的変換: 関節 1 の角度で更新)
    └── link2  (動的変換: 関節 2 の角度で更新)
        └── tool  (静的変換: ツール先端の固定オフセット)
```

ツール先端フレームの変換をリアルタイムで確認します。

```bash
ros2 run tf2_ros tf2_echo base_link tool
```

期待される出力（アームが動いている場合、値が変化する）:
```
At time ...
- Translation: [x, y, 0.0]  ← ツール先端の base_link からの位置
- Rotation: in Quaternion [0.0, 0.0, sin(θ/2), cos(θ/2)]
```

各フレームの変換情報を個別に確認します。

```bash
# 関節 1 の変換
ros2 run tf2_ros tf2_echo base_link link1

# 関節 2 の変換
ros2 run tf2_ros tf2_echo link1 link2

# ツール先端の変換 (link2 からの静的オフセット)
ros2 run tf2_ros tf2_echo link2 tool
```

### TF チェーンの意味

アームの「順運動学」は TF の積み重ねで表現されています。

```
base_link → link1 の変換: 関節 1 の回転
link1 → link2 の変換: 関節 2 の回転
link2 → tool の変換: ツール先端の固定オフセット

→ lookup_transform(base_link, tool) で全体を合成した変換が得られる
```

これが TF の本質です。個々の変換をチェーン状につなぐことで、複雑な座標変換を自動的に計算できます。

### 学んだこと

- TF チェーンは複数フレームをツリー状につないで合成変換を自動計算する
- `view_frames` の PDF にはフレームの更新頻度も表示され、動的 / 静的の違いが確認できる
- `tf2_echo` は 2 つのフレームを指定するだけでチェーン全体を通じた変換を返してくれる
- 順運動学の計算が TF によって自動化されている点が ROS2 の大きな強みの一つ
