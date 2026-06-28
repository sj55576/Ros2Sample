# チュートリアル 20: Component と Composition（プロセス内通信）

## 学習目標

- プロセス間通信とプロセス内通信の違いを理解できる
- Composable Node の概念とメリットを説明できる
- C++ での Composition パターンを概念的に理解できる
- Python で複数ノードを同一プロセスで実行する方法を実装できる
- Launch ファイルで Composition を設定できる

---

## プロセス間通信 vs プロセス内通信

ROS 2 では通常、各ノードは独立したプロセスとして起動します。ノード間のメッセージは DDS（Data Distribution Service）を経由して送受信されます。しかし、同一マシン上で密接に連携するノードを別プロセスで実行すると、シリアライズ・デシリアライズのオーバーヘッドが発生します。

Composition（コンポジション）は、複数のノードを **同一プロセス内** で実行し、このオーバーヘッドを削減する仕組みです。

```mermaid
flowchart LR
    subgraph 従来のアーキテクチャ（プロセス間通信）
        direction LR
        subgraph ProcessA["プロセス A"]
            N1["Node 1\n(センサー)"]
        end
        subgraph ProcessB["プロセス B"]
            N2["Node 2\n(フィルター)"]
        end
        N1 -->|"DDS 通信\n(シリアライズ → ネットワーク → デシリアライズ)"| N2
    end

    subgraph Compositionアーキテクチャ（プロセス内通信）
        direction LR
        subgraph SingleProcess["単一プロセス"]
            direction LR
            CN1["Node 1\n(センサー)"]
            CN2["Node 2\n(フィルター)"]
            CN1 -->|"共有メモリ\n(ゼロコピー可能)"| CN2
        end
    end
```

---

## なぜ Composition が必要か

### プロセス間通信のオーバーヘッド

通常の ROS 2 ノード間通信では、以下のステップが発生します。

1. **シリアライズ**: 送信ノードがメッセージをバイト列に変換する
2. **ネットワーク送信**: DDS ミドルウェアがデータを転送する
3. **デシリアライズ**: 受信ノードがバイト列をメッセージ型に変換する
4. **コンテキストスイッチ**: OS がプロセス間で CPU を切り替える
5. **メモリコピー**: 送受信それぞれのプロセスにデータが存在する

センサーデータを高頻度（例: 1000 Hz）で処理するロボットシステムでは、このオーバーヘッドがリアルタイム性能に影響します。

### プロセス内通信のメリット

- **低レイテンシ**: シリアライズが不要でメッセージを直接渡せる
- **低メモリ使用量**: データのコピーを削減できる
- **高スループット**: コンテキストスイッチが削減される
- **ゼロコピー**: 大きなメッセージ（画像など）を参照渡しできる

### Composition を使わない方が良いケース

> **注意**: Composition が常に最善とは限りません。以下のケースでは独立プロセスの方が適切です。
>
> - **耐障害性が重要な場合**: 1 つのノードがクラッシュすると同一プロセスの全ノードに影響する
> - **独立したデプロイが必要な場合**: ノードを別々にアップデートしたい場合
> - **異なるマシンで実行する場合**: 分散システムでは DDS 通信が必要
> - **デバッグ中**: 独立プロセスの方が問題の切り分けが容易

---

## C++ での Composition（概念説明）

ROS 2 の標準的な Composition 機能は **C++（rclcpp）** 向けに設計されています。このリポジトリは Python ベースですが、概念を理解するために C++ のアプローチを説明します。

### Composable Node の登録

C++ では、ノードクラスをコンポーネントとして登録するマクロを使用します。

```cpp
// C++ の例（概念説明用）
#include "rclcpp_components/register_node_macro.hpp"

class MyNode : public rclcpp::Node {
public:
    explicit MyNode(const rclcpp::NodeOptions & options)
    : Node("my_node", options) { ... }
};

RCLCPP_COMPONENTS_REGISTER_NODE(my_package::MyNode)
```

### component_container の起動

C++ の場合、`component_container` という実行ファイルにノードを動的にロードします。

```bash
# コンポーネントコンテナを起動
ros2 run rclcpp_components component_container

# 別ターミナルでノードを動的にロード
ros2 component load /ComponentManager my_package my_package::MyNode

# 現在ロードされているコンポーネントを確認
ros2 component list
```

### Launch ファイルでの C++ Composition

```python
# C++ Composition の launch ファイル例（概念説明用）
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_package',
                plugin='my_package::SensorNode',
                name='sensor_node',
            ),
            ComposableNode(
                package='my_package',
                plugin='my_package::FilterNode',
                name='filter_node',
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])
```

> **注意**: `ComposableNodeContainer` と `ComposableNode` は C++ パッケージ専用です。Python ノードには使用できません。Python での同等パターンは次のセクションで説明します。

---

## Python での Composition パターン

Python ノードは `rclcpp_components` の仕組みを使用できません。しかし、**複数ノードを同一プロセスで実行する** という目標は、`rclpy` の Executor を活用することで達成できます。

### パターン 1: 共有 Executor による複数ノードの実行

最もシンプルな Python Composition パターンです。

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_package.node_a import NodeA
from my_package.node_b import NodeB

def main():
    rclpy.init()

    node_a = NodeA()
    node_b = NodeB()

    # 単一の Executor に複数ノードを登録
    executor = MultiThreadedExecutor()
    executor.add_node(node_a)
    executor.add_node(node_b)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node_a.destroy_node()
        node_b.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### パターン 2: sensor_fusion_sim の実装例

このリポジトリの `sensor_fusion_sim` パッケージには、既に MultiThreadedExecutor を活用した実装があります。

`src/sensor_fusion_sim/sensor_fusion_sim/complementary_filter_node.py` では、以下のパターンが使用されています。

```python
# complementary_filter_node.py の構造（抜粋イメージ）
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class ComplementaryFilterNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('complementary_filter')

        # センサーデータ受信用: 並行実行可能
        self.reentrant_group = ReentrantCallbackGroup()

        # 状態更新用: 排他的実行
        self.exclusive_group = MutuallyExclusiveCallbackGroup()

        self.subscription = self.create_subscription(
            SensorData,
            'noisy_sensor_data',
            self.sensor_callback,
            10,
            callback_group=self.reentrant_group
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_callback,
            callback_group=self.exclusive_group
        )


def main():
    rclpy.init()
    node = ComplementaryFilterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

この実装では、`MultiThreadedExecutor` によって複数のコールバックが並行して処理されます。

### パターン 3: パイプライン全体を単一エントリポイントで実行

sensor_fusion_sim の全ノードを 1 つのプロセスで起動する例です。

```python
# combined_pipeline.py（新規作成例）
import rclpy
from rclpy.executors import MultiThreadedExecutor

from sensor_fusion_sim.noisy_sensor_node import NoisySensorNode
from sensor_fusion_sim.complementary_filter_node import ComplementaryFilterNode


def main():
    rclpy.init()

    # 各ノードをインスタンス化
    sensor = NoisySensorNode()
    filter_node = ComplementaryFilterNode()

    # スレッド数を指定して MultiThreadedExecutor を作成
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(sensor)
    executor.add_node(filter_node)

    print("センサーフュージョンパイプラインを単一プロセスで起動中...")

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor.destroy_node()
        filter_node.destroy_node()
        rclpy.shutdown()
        print("シャットダウン完了")


if __name__ == '__main__':
    main()
```

---

## Callback Groups と Executor の関係

Python での Composition を理解するには、Callback Groups と Executor の関係を把握することが重要です。

### Executor の種類

| Executor | 説明 | 適用場面 |
|---|---|---|
| `SingleThreadedExecutor` | 1 スレッドで全コールバックを順番に処理 | シンプルなノード、デバッグ |
| `MultiThreadedExecutor` | 複数スレッドでコールバックを並行処理 | 高頻度センサー処理、並行タスク |

### Callback Group の種類

| Callback Group | 説明 | 使用例 |
|---|---|---|
| `MutuallyExclusiveCallbackGroup` | グループ内のコールバックを排他的に実行（デフォルト） | 共有状態の更新、排他制御が必要な処理 |
| `ReentrantCallbackGroup` | グループ内のコールバックを並行実行可能 | 独立したセンサー受信、ステートレスな処理 |

### 組み合わせの例

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class SensorProcessingNode(Node):
    def __init__(self):
        super().__init__('sensor_processing')

        # 並行処理グループ: センサー A と B は独立して並行受信
        parallel_group = ReentrantCallbackGroup()

        # 排他処理グループ: 状態の更新は排他的に実行
        exclusive_group = MutuallyExclusiveCallbackGroup()

        # センサー A のサブスクリプション（並行実行可能）
        self.sub_a = self.create_subscription(
            Float32, 'sensor_a', self.callback_a, 10,
            callback_group=parallel_group
        )

        # センサー B のサブスクリプション（並行実行可能）
        self.sub_b = self.create_subscription(
            Float32, 'sensor_b', self.callback_b, 10,
            callback_group=parallel_group
        )

        # 状態更新タイマー（排他的実行）
        self.timer = self.create_timer(
            0.05, self.update_state,
            callback_group=exclusive_group
        )
```

### sensor_fusion_sim における実装の意味

`complementary_filter_node.py` が `ReentrantCallbackGroup` と `MutuallyExclusiveCallbackGroup` を両方使用しているのは、以下の理由です。

- **センサーデータ受信** (`ReentrantCallbackGroup`): 複数センサーのコールバックを並行処理し、データ取りこぼしを防ぐ
- **フィルター出力** (`MutuallyExclusiveCallbackGroup`): 状態の整合性を保ちながらパブリッシュする

この設計により、単一プロセス内で高いスループットと状態の一貫性を両立しています。

---

## Launch ファイルでの設定

### 標準アプローチ（独立プロセス）

通常の launch ファイルでは、各ノードを独立したプロセスとして起動します。

```python
# sensor_fusion_demo.launch.py（標準パターン）
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    noisy_sensor = Node(
        package='sensor_fusion_sim',
        executable='noisy_sensor_node',
        name='noisy_sensor_node',
        output='screen',
    )

    complementary_filter = Node(
        package='sensor_fusion_sim',
        executable='complementary_filter_node',
        name='complementary_filter',
        output='screen',
    )

    # 各ノードが独立したプロセスで起動される
    return LaunchDescription([
        noisy_sensor,
        complementary_filter,
    ])
```

このリポジトリの実際の launch ファイルは `src/sensor_fusion_sim/launch/sensor_fusion_demo.launch.py` を参照してください。

### 統合プロセスアプローチ

Python ノードを同一プロセスで実行するには、カスタムエントリポイントを launch ファイルから起動します。

```python
# sensor_fusion_combined.launch.py（統合プロセスパターン）
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # combined_pipeline エントリポイントを使用
    # このエントリポイント内で MultiThreadedExecutor に複数ノードを登録する
    combined_pipeline = Node(
        package='sensor_fusion_sim',
        executable='combined_pipeline',  # setup.py に登録が必要
        name='sensor_fusion_pipeline',
        output='screen',
        parameters=[{
            'num_threads': 4,
        }],
    )

    return LaunchDescription([combined_pipeline])
```

### setup.py へのエントリポイント登録

```python
# setup.py の entry_points 設定例
from setuptools import setup

setup(
    name='sensor_fusion_sim',
    # ...
    entry_points={
        'console_scripts': [
            'noisy_sensor_node = sensor_fusion_sim.noisy_sensor_node:main',
            'complementary_filter_node = sensor_fusion_sim.complementary_filter_node:main',
            # 統合エントリポイントを追加
            'combined_pipeline = sensor_fusion_sim.combined_pipeline:main',
        ],
    },
)
```

> **注意**: エントリポイントを追加した後は、パッケージを再ビルドする必要があります。
>
> ```bash
> cd Ros2Sample
> colcon build --packages-select sensor_fusion_sim
> source install/setup.bash
> ```

---

## パフォーマンス比較（概念的）

### 通信方式の比較表

| 項目 | プロセス間通信 | プロセス内通信 |
|---|---|---|
| レイテンシ | 高い（シリアライズあり） | 低い（直接参照） |
| メモリ使用量 | 多い（各プロセスにコピー） | 少ない（共有可能） |
| スループット | 中程度 | 高い |
| 耐障害性 | 高い（ノードが独立） | 低い（プロセスを共有） |
| デバッグのしやすさ | 容易（独立して観察可能） | やや複雑 |
| スケーラビリティ | 高い（別マシンに分散可能） | 低い（同一プロセスに限定） |
| 起動の複雑さ | 低い（標準的な Node() 起動） | 中程度（エントリポイント設計が必要） |

### ユースケース別の推奨

**プロセス間通信が適切な場面**:
- ノードの独立性と耐障害性が重要なシステム
- ノードを別々にアップデート・再起動したい場合
- 異なるマシンへの分散が必要な場合
- プロトタイプ開発・デバッグフェーズ

**プロセス内通信が適切な場面**:
- センサーデータを高頻度（100 Hz 以上）で処理するパイプライン
- レイテンシが厳しく制約されるリアルタイム制御システム
- 大きなデータ（点群、画像）を複数ノードで共有する場合
- 本番環境で確定した、安定したノード構成

### Python での現実的な考慮事項

> **注意**: Python の GIL（Global Interpreter Lock）により、`MultiThreadedExecutor` を使用しても CPU 集約的なタスクは真の並行実行が制限されます。I/O 待機やコールバックの切り替えには効果がありますが、計算集約的な処理には C++ の方が適しています。
>
> Python での Composition パターンは、主に以下の目的で有効です:
> - プロセス起動・シャットダウンのオーバーヘッド削減
> - ノード間のデータ受け渡しの簡略化
> - システム管理の一元化

---

## 実際に試してみる

### ステップ 1: 現在の sensor_fusion_sim を個別プロセスで実行

```bash
# ワークスペースのセットアップ
cd Ros2Sample
source install/setup.bash

# 標準の launch ファイルで起動（各ノードが独立プロセス）
ros2 launch sensor_fusion_sim sensor_fusion_demo.launch.py
```

```text
[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [noisy_sensor_node-1]: process started with pid [12345]
[INFO] [complementary_filter_node-1]: process started with pid [12346]
[INFO] [lifecycle_data_recorder-1]: process started with pid [12347]
```

実行中のノードを確認します。

```bash
ros2 node list
```

```text
/complementary_filter
/noisy_sensor_node
```

### ステップ 2: トピックのレイテンシを確認

```bash
# トピックの統計情報を確認
ros2 topic hz /gps
ros2 topic hz /fused_odom
```

### ステップ 3: 現在の Executor 動作を理解する

```bash
# complementary_filter_node の実装を確認
cat src/sensor_fusion_sim/sensor_fusion_sim/complementary_filter_node.py
```

`MultiThreadedExecutor` と 2 種類の CallbackGroup がどのように使われているか観察してください。

---

## 練習問題

### 練習 1: 統合エントリポイントの作成

`sensor_fusion_sim` パッケージに `combined_pipeline.py` を作成し、`NoisySensorNode` と `ComplementaryFilterNode` を単一プロセスで実行してみましょう。

ファイルを作成する場所: `src/sensor_fusion_sim/sensor_fusion_sim/combined_pipeline.py`

実装のヒント:
1. 両方のノードクラスをインポートする
2. `MultiThreadedExecutor` に両ノードを追加する
3. `setup.py` にエントリポイントを追加する
4. パッケージを再ビルドする

### 練習 2: Executor の違いを観察する

`SingleThreadedExecutor` と `MultiThreadedExecutor` を切り替えて、コールバックの処理順序の違いを観察してください。

```python
# SingleThreadedExecutor の場合
from rclpy.executors import SingleThreadedExecutor
executor = SingleThreadedExecutor()

# MultiThreadedExecutor の場合（スレッド数を変えて試す）
from rclpy.executors import MultiThreadedExecutor
executor = MultiThreadedExecutor(num_threads=2)  # 2スレッド
executor = MultiThreadedExecutor(num_threads=4)  # 4スレッド
```

`get_logger().info()` でコールバックの開始・終了タイムスタンプを出力し、並行性の違いを確認してください。

### 練習 3: プロセス数の確認

独立プロセス起動と統合プロセス起動でシステムのプロセス数がどう変わるか確認してください。

```bash
# launch ファイルで起動後にプロセスを確認
ps aux | grep ros2

# ノードのプロセス ID を確認
ros2 node info /noisy_sensor_node
```

2 つのノードが同一 PID で動作している場合、統合プロセスが正常に動作しています。

---

## まとめ

このチュートリアルでは、ROS 2 の Composition（プロセス内通信）について学びました。

**重要なポイント**:

- **Composition の目的**: 複数ノードを同一プロセスで実行し、DDS 通信のオーバーヘッドを削減する
- **C++ が標準**: `rclcpp_components` を使った公式の Composition は C++ 専用であり、`component_container` と `ComposableNodeContainer` によって実現される
- **Python での代替手段**: `MultiThreadedExecutor` に複数ノードを登録することで、同等の効果が得られる
- **Callback Groups の活用**: `ReentrantCallbackGroup` と `MutuallyExclusiveCallbackGroup` を組み合わせることで、並行性と安全性を両立できる（`sensor_fusion_sim/complementary_filter_node.py` 参照）
- **トレードオフ**: プロセス内通信は低レイテンシだが耐障害性が低下する。システムの要件に応じて選択する

**関連チュートリアル**:

- チュートリアル 06: ライフサイクル管理と QoS 設定
- チュートリアル 14: 既存パッケージの読み方とプロジェクト構造の理解

パフォーマンスが重要な本番システムでは、C++ への移行または C++ と Python のハイブリッド構成も検討に値します。しかし、Python での `MultiThreadedExecutor` パターンは、プロトタイプ開発や Python が適しているユースケースでは十分に実用的な選択肢です。
