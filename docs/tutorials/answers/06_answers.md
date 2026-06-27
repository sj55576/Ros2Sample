# チュートリアル 6 演習解答: ライフサイクルノードと QoS

## 演習 1: lifecycle_demo を手動で状態遷移させてログを確認する

### 考え方

ライフサイクルノードの状態遷移を「手動で」行うことで、各コールバックが呼ばれるタイミングと、各状態でできることとできないことの違いを体感することが目的です。

特に重要な観察ポイントは:
- `configure` 前はパブリッシャーが存在しない（`/lifecycle_output` に接続できない）
- `activate` 後だけメッセージが流れる
- `deactivate` でタイマーが停止するため配信も止まる

### 完全な手順と期待される出力

**ターミナル 1**: ノードを起動します。

```bash
ros2 run ros2_learning lifecycle_demo
```

起動直後のログ（`on_configure` は呼ばれていない）:
```
[INFO] [lifecycle_demo]: LifecycleDemo ノードが起動しました (Unconfigured)
```

**ターミナル 2**: 状態遷移を行います。

```bash
# 現在の状態を確認
ros2 lifecycle get /lifecycle_demo
```

出力:
```
unconfigured [1]
```

```bash
# configure: リソース確保
ros2 lifecycle set /lifecycle_demo configure
```

出力:
```
Transitioning successful
```

ターミナル 1 のログ:
```
[INFO] [lifecycle_demo]: on_configure が呼ばれました
[INFO] [lifecycle_demo]: パブリッシャーを作成しました
```

```bash
# activate: タイマー開始
ros2 lifecycle set /lifecycle_demo activate
```

ターミナル 1 のログ（タイマーが開始され count がカウントアップする）:
```
[INFO] [lifecycle_demo]: on_activate が呼ばれました
[INFO] [lifecycle_demo]: Publishing: count=1
[INFO] [lifecycle_demo]: Publishing: count=2
```

**ターミナル 3**: 配信を確認します。

```bash
ros2 topic echo /lifecycle_output
```

期待される出力:
```
data: 'Lifecycle demo: count=3'
---
data: 'Lifecycle demo: count=4'
---
```

**ターミナル 2**: 一時停止します。

```bash
ros2 lifecycle set /lifecycle_demo deactivate
```

ターミナル 1 のログ（タイマーが停止する）:
```
[INFO] [lifecycle_demo]: on_deactivate が呼ばれました
[INFO] [lifecycle_demo]: タイマーを停止しました
```

ターミナル 3 の `ros2 topic echo` が止まることを確認します。

**ターミナル 2**: クリーンアップします。

```bash
ros2 lifecycle set /lifecycle_demo cleanup
```

ターミナル 1 のログ:
```
[INFO] [lifecycle_demo]: on_cleanup が呼ばれました
[INFO] [lifecycle_demo]: パブリッシャーを破棄しました
```

```bash
# 初期状態に戻ったことを確認
ros2 lifecycle get /lifecycle_demo
```

出力:
```
unconfigured [1]
```

### 状態遷移まとめ

```
unconfigured → (configure) → inactive → (activate) → active
                                ↑                        |
                           (cleanup)               (deactivate)
                                |                        |
                                +------------------------+
```

各遷移でコールバックが呼ばれる順序:
- `configure`: `on_configure` → `SUCCESS` なら `inactive` へ
- `activate`: `on_activate` + `super().on_activate` → `active` へ（`super()` でパブリッシャーが有効化される）
- `deactivate`: `on_deactivate` + `super().on_deactivate` → `inactive` へ
- `cleanup`: `on_cleanup` → `unconfigured` へ

### 重要な実装の注意点

`on_activate` と `on_deactivate` では `super()` の呼び出しが必須です。

```python
def on_activate(self, state):
    self._timer = self.create_timer(1.0, self._publish)
    return super().on_activate(state)  # ← これがないとパブリッシャーが有効化されない
```

`super().on_activate` を呼ばないと `create_lifecycle_publisher` で作成したパブリッシャーが有効化されず、`active` 状態でも `publish()` が送信されません。

---

## 演習 2: sensor_fusion_sim の QoS プロファイルを変更して挙動の違いを観察する

### 考え方

QoS の互換性ルールを理解することが目的です。最重要ルール:

**`BEST_EFFORT` パブリッシャーに `RELIABLE` サブスクライバーを接続しようとすると、接続が確立されません。**

理由: `RELIABLE` サブスクライバーは「必ず届けてほしい」と要求しているが、`BEST_EFFORT` パブリッシャーはその保証を提供できないため。

### 変更手順

**ステップ 1**: `noisy_sensor_node.py` の IMU QoS を変更します。

対象ファイル: `src/sensor_fusion_sim/sensor_fusion_sim/noisy_sensor_node.py`

変更前:
```python
best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
self.create_publisher(Imu, 'imu', best_effort_qos)
```

変更後:
```python
reliable_imu_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
self.create_publisher(Imu, 'imu', reliable_imu_qos)
```

**ステップ 2**: `complementary_filter_node.py` のサブスクライバー QoS も同様に変更します。

対象ファイル: `src/sensor_fusion_sim/sensor_fusion_sim/complementary_filter_node.py`

`imu` トピックを購読している箇所の QoS を `RELIABLE` に変更します。

```python
reliable_imu_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
self.create_subscription(Imu, 'imu', self._on_imu,
                         reliable_imu_qos,  # ← 変更
                         callback_group=sensor_cb_group)
```

**ステップ 3**: ビルドして動作を確認します。

```bash
colcon build --packages-select sensor_fusion_sim
source install/setup.bash
ros2 launch sensor_fusion_sim sensor_fusion_demo.launch.py
```

`/imu` のレートを確認します。

```bash
ros2 topic hz /imu
```

`RELIABLE` に変更しても IMU の 50 Hz 配信は維持されます（ローカル DDS 通信では再送のコストが低いため）。

### QoS 不一致を意図的に起こす実験

パブリッシャーを `BEST_EFFORT` のままにして、サブスクライバーを `RELIABLE` にすると接続が確立されません。

```bash
# QoS 不一致状態で /imu トピックを確認する
ros2 topic info /imu --verbose
```

接続されていない場合、サブスクライバー側の `Subscription count: 0` のままになります。

### QoS 互換性の早見表

| パブリッシャー | サブスクライバー | 接続結果 |
|--------------|----------------|---------|
| RELIABLE     | RELIABLE       | 接続成功 |
| RELIABLE     | BEST_EFFORT    | 接続成功（サブスクライバーが緩い要求） |
| BEST_EFFORT  | RELIABLE       | **接続失敗** |
| BEST_EFFORT  | BEST_EFFORT    | 接続成功 |

---

## 演習 3: complementary_filter.py のコールバックグループを確認する

### 考え方

コールバックグループは「同時実行を許可するかどうか」を制御します。`ReentrantCallbackGroup` は並列実行を許可し、`MutuallyExclusiveCallbackGroup` は排他的（一度に 1 つのみ実行）です。

ただし並列実行は `MultiThreadedExecutor` と組み合わせて初めて有効になります。

### 確認手順

**ステップ 1**: ソースファイルを確認します。

対象ファイル: `src/sensor_fusion_sim/sensor_fusion_sim/complementary_filter_node.py`

```python
# センサーコールバック: 複数センサーを並列処理してよい
sensor_cb_group = ReentrantCallbackGroup()

# 出力タイマー: 他のコールバックと排他的に実行する
timer_cb_group = MutuallyExclusiveCallbackGroup()
```

**1. `sensor_cb_group` (ReentrantCallbackGroup) に属するコールバック:**
- `self._on_imu` (IMU サブスクライバーのコールバック)
- `self._on_gps` (GPS サブスクライバーのコールバック)

これらは独立したセンサーデータを受け取るため、同時に実行されても問題ありません。

**2. `timer_cb_group` (MutuallyExclusiveCallbackGroup) に属するコールバック:**
- `self._publish` (出力タイマーのコールバック)

フィルタ結果の出力タイマーは、センサー受信処理の途中に割り込むと計算状態が不整合になる可能性があります。`MutuallyExclusiveCallbackGroup` を使うことで、他のコールバックの実行中には起動しないことを保証します。

**3. `main()` 関数のエグゼキュータ:**
```python
executor = MultiThreadedExecutor()
```

`ReentrantCallbackGroup` の並列実行を有効化するために `MultiThreadedExecutor` を使っています。

**ステップ 2**: `ros2 node info` で実行時の情報を確認します。

```bash
ros2 node info /complementary_filter
```

コールバックグループの情報が表示されます。

**ステップ 3**: `ReentrantCallbackGroup` を `MutuallyExclusiveCallbackGroup` に変更する実験。

対象ファイル: `src/sensor_fusion_sim/sensor_fusion_sim/complementary_filter_node.py`

変更前:
```python
sensor_cb_group = ReentrantCallbackGroup()
```

変更後:
```python
sensor_cb_group = MutuallyExclusiveCallbackGroup()
```

ビルドして起動します。

```bash
colcon build --packages-select sensor_fusion_sim
source install/setup.bash
ros2 launch sensor_fusion_sim sensor_fusion_demo.launch.py
```

IMU（50 Hz）と GPS（1 Hz）の処理が直列化されるため、高負荷時にセンサーデータの処理が遅延する可能性があります。ただし低負荷な環境では見かけ上の違いが出にくいことも多いです。

### まとめ: いつどちらのコールバックグループを使うか

| 状況 | 推奨グループ |
|------|------------|
| 独立した処理（センサー受信）を並列化したい | `ReentrantCallbackGroup` + `MultiThreadedExecutor` |
| 共有状態を持つコールバック間の競合を避けたい | `MutuallyExclusiveCallbackGroup` |
| シンプルなノードで並列化が不要 | グループ指定なし（デフォルト: `MutuallyExclusive`） |

`MultiThreadedExecutor` を使っても、コールバックグループが `MutuallyExclusiveCallbackGroup` のままでは並列実行は起きません。グループとエグゼキュータのセットアップがセットで必要です。
