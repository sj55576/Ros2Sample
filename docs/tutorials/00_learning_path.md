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

> **注意**: `source install/setup.bash` はターミナルを新しく開くたびに実行する必要があります。毎回入力する手間を省くには `~/.bashrc` に追記することもできます。

---

## 学習パス概要

以下の順番で進めることを推奨します。各ステップは前のステップの知識を前提としています。

| ステップ | チュートリアル | 目安時間 | 内容 |
|----------|---------------|----------|------|
| 1 | [Publisher と Subscriber](01_publisher_subscriber.md) | 30 分 | トピック通信の基礎 |
| 2 | [サービスとアクション](02_service_action.md) | 45 分 | リクエスト/レスポンス型通信 |
| 3 | [Launch ファイルとパラメータ](03_launch_params.md) | 30 分 | ノード管理と設定 |
| 4 | TF と座標変換 *(準備中)* | 45 分 | フレーム間の位置関係 |
| 5 | カスタムインターフェース *(準備中)* | 30 分 | 独自メッセージ型定義 |
| 6 | ライフサイクルノードと QoS *(準備中)* | 45 分 | 高度なノード管理 |

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

- 学習ファイル: `src/ros2_learning/ros2_learning/tf_broadcaster_demo.py`（準備中）
- 学習ファイル: `src/ros2_learning/ros2_learning/tf_listener_demo.py`（準備中）

### ステップ 5: カスタムインターフェース（30 分）

`std_msgs` や `geometry_msgs` に用意されていない独自のメッセージ・サービス・アクション型を定義する方法を学びます。

- 参照パッケージ: `src/sample_interfaces/`

### ステップ 6: ライフサイクルノードと QoS（45 分）

ノードの起動・設定・アクティブ・シャットダウンという状態遷移を管理するライフサイクルノードと、通信品質を制御する QoS（Quality of Service）を学びます。

- 学習ファイル: `src/ros2_learning/ros2_learning/lifecycle_demo.py`（準備中）

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
    └── sample_interfaces: カスタム msg/srv/action 定義の実例
        ├── RobotStatus.msg
        ├── GetRobotStatus.srv
        └── NavigateWaypoints.action
```

---

## 効果的な学習のヒント

1. **手を動かす**: コードを読むだけでなく、必ず実際に動かしてみましょう。ログ出力を見るだけでも理解が深まります。
2. **コマンドラインツールを使う**: `ros2 topic echo`、`ros2 service call`、`ros2 node info` などの CLI ツールは、システムの状態をリアルタイムで確認するのに非常に便利です。
3. **コードを改造する**: サンプルコードのパラメータ値や処理内容を変えてみることで、各要素の役割が体感的に理解できます。
4. **既存パッケージを読む**: `drone_sim` や `ground_robot_sim` のコードは、実際のロボットシステムでの ROS 2 の使い方を示しています。チュートリアルを終えたら積極的に読んでみましょう。
5. **エラーを恐れない**: ビルドエラーや実行時エラーのメッセージは情報の宝庫です。エラー文をよく読むと解決策が見つかることがほとんどです。
