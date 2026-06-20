# Ros2Sample

**Ros2Sample** は、ROS 2 でロボット／ドローンのサンプルを学習・検証するためのワークスペースです。<br>
このリポジトリは **日本語を第一言語** として、開発者が Ubuntu 20.04 / 24.04 / 26.04 と ROS 2 Foxy / Lyrical / Jazzy / Kilted / Rolling の環境で、依存関係の取得、ビルド、実行、CI 検証を迷わず行えることを目標にしています。

> 現在は依存を軽くした Python ベースの地上ロボット／クアッドローターシミュレーションサンプルを `src/` 以下に収録しています。Gazebo 等の重いシミュレータに進む前に、ROS 2 の topic、launch、namespace、TF、センサー風データの流れを確認するための入口として使えます。

## 対象環境

| OS | ROS 2 | 用途 | 備考 |
| --- | --- | --- | --- |
| Ubuntu 26.04 LTS | Lyrical Luth | 推奨・CI 対象 | 2026年5月リリースの新 LTS。サポート期間 2031年5月まで。 |
| Ubuntu 24.04 LTS | Jazzy Jalisco | 安定版・CI 対象 | Lyrical でも Noble は 2029年まで引き続きサポートされます。 |
| Ubuntu 20.04 LTS | Foxy Fitzroy | 互換対象 | EOL 済みのため新規採用は非推奨ですが、既存環境向けに軽量 Python サンプルのビルド・実行互換性を維持します。Gazebo/GZ 連携は対象外です。 |
| Ubuntu 24.04 LTS | Kilted Kaiju | 開発候補 | パッケージ互換性を確認しながら利用してください。 |
| Ubuntu 24.04 / 26.04 | Rolling Ridley | 最新 API 検証 | API 変更が頻繁に入るため、CI 失敗時は変更内容を確認してください。 |

## リポジトリ構成

```text
.
├── .github/workflows/ci.yml   # GitHub Actions: 手動実行で Foxy/Jazzy/Lyrical の colcon build/test
├── docker/                    # Docker 開発環境
├── docs/                      # 補足ドキュメント
├── scripts/                   # 開発者向けヘルパースクリプト
├── ros2.repos                 # vcs import 用の依存リポジトリ定義
└── src/                       # ROS 2 サンプルパッケージ配置先
```

## パッケージ一覧

現在、このリポジトリには次の ROS 2 パッケージがあります。

| パッケージ | 目的 | 主な実行ファイル |
| --- | --- | --- |
| `ground_robot_sim` | 差動二輪風の地上ロボット、LiDAR 風停止判定、PID閉ループウェイポイント追従、障害物回避、緊急停止サービス、複数ロボット namespace の軽量サンプル | `ground_robot_node`, `diff_drive_patrol`, `lidar_obstacle_stop`, `lidar_obstacle_avoid`, `waypoint_follower` |
| `drone_sim` | クアッドローター風の位置・PID高度制御、waypoint 指令、バッテリーモニター、緊急着陸、小規模 swarm namespace の軽量サンプル | `sim_drone`, `altitude_hold`, `waypoint_commander`, `battery_monitor`, `emergency_land` |
| `manipulator_sim` | 2自由度平面マニピュレータの JointState / TF / 目標姿勢追従を学ぶ軽量サンプル | `manipulator_simulator`, `target_commander` |
| `sample_interfaces` | カスタム msg / srv / action 定義（ROS 2 インターフェース定義の学習用） | _(ライブラリパッケージ：実行ファイルなし)_ |

検出結果は `colcon list` で確認できます。

## 依存関係

### 基本ツール

Ubuntu 26.04/Lyrical の例です。Jazzy/Kilted/Rolling を使う場合は `ROS_DISTRO` を変更してください。

```bash
sudo apt update
sudo apt install -y \
  curl \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool
```

ROS 2 の apt リポジトリ設定とインストールは、利用する ROS 2 ディストリビューションの公式手順に従ってください。インストール後、次のように環境を読み込みます。

```bash
# Lyrical (推奨)
source /opt/ros/lyrical/setup.bash

# Jazzy
source /opt/ros/jazzy/setup.bash

# Foxy (Ubuntu 20.04 既存環境向け)
source /opt/ros/foxy/setup.bash
```

### rosdep

初回のみ rosdep を初期化します。

```bash
sudo rosdep init || true
rosdep update
```

ワークスペース依存関係を解決します。

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro lyrical
```

`src/` のパッケージが増えた場合も、`package.xml` に依存関係を追加していれば同じコマンドで解決できます。

## セットアップ

```bash
git clone <this-repository-url> Ros2Sample
cd Ros2Sample

# 外部リポジトリ依存が追加された場合に利用します。
vcs import src < ros2.repos

# ROS 2 環境を読み込みます (Lyrical 推奨)。
source /opt/ros/lyrical/setup.bash

# 依存関係を解決します。
./scripts/rosdep-install.sh lyrical
```

`ros2.repos` は現在空に近いテンプレートです。外部 ROS 2 パッケージを固定したい場合は、`repositories:` 以下に追加してください。

## ビルド

```bash
./scripts/build.sh
```

内部では `colcon build --symlink-install` を実行します。ビルド後、ワークスペースを読み込みます。

```bash
source install/setup.bash
```

## テストと lint

```bash
./scripts/lint.sh
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

各パッケージの `test/` 以下に pytest 単体テスト（純粋関数向け）と flake8 / pep257 の lint テストを収録しています。`scripts/lint.sh` は検出できるパッケージに対して colcon test の lint 系テストを実行します。パッケージが存在しない作業途中の状態でも成功するようにしており、初期セットアップや段階的な開発でも CI を通しやすくしています。

## 実行例

ビルド後に `source install/setup.bash` を実行してから、別々のターミナルでノードを起動します。

```bash
# 地上ロボット状態を publish するサンプル
ros2 run ground_robot_sim ground_robot_node

# 差動二輪風の巡回コマンドを publish するサンプル
ros2 run ground_robot_sim diff_drive_patrol

# LiDAR 風データで停止判断するサンプル
ros2 run ground_robot_sim lidar_obstacle_stop

# 閉ループウェイポイント追従サンプル
ros2 run ground_robot_sim waypoint_follower

# LiDAR 風データで障害物を回避するサンプル
ros2 run ground_robot_sim lidar_obstacle_avoid

# ドローン状態を publish するサンプル
ros2 run drone_sim sim_drone

# PID高度維持コマンドを publish するサンプル
ros2 run drone_sim altitude_hold

# waypoint 指令を publish するサンプル
ros2 run drone_sim waypoint_commander

# バッテリーモニター（電力消費シミュレーション）
ros2 run drone_sim battery_monitor

# 緊急着陸（バッテリー低下時の自動降下 + サービストリガー）
ros2 run drone_sim emergency_land

# マニピュレータの状態 publish サンプル
ros2 run manipulator_sim manipulator_simulator

# 平面ターゲット列から関節指令を publish するサンプル
ros2 run manipulator_sim target_commander

# 地上ロボットの緊急停止サービス呼び出し例
ros2 service call /emergency_stop std_srvs/srv/Trigger
ros2 service call /reset_emergency std_srvs/srv/Trigger

# ノード一覧確認
ros2 node list

# topic 確認
ros2 topic list
```

launch ファイルと RViz 設定も同梱しています。代表的なデモは次の通りです。

```bash
# 地上ロボット: 巡回、LiDAR停止、ウェイポイント追従、障害物回避、複数ロボット
ros2 launch ground_robot_sim diff_drive_patrol.launch.py
ros2 launch ground_robot_sim lidar_obstacle_stop.launch.py
ros2 launch ground_robot_sim waypoint_follower.launch.py
ros2 launch ground_robot_sim lidar_obstacle_avoid.launch.py
ros2 launch ground_robot_sim multi_robot.launch.py

# ドローン: waypoint飛行、高度維持、バッテリーデモ、小規模swarm
ros2 launch drone_sim single_quad_waypoint.launch.py
ros2 launch drone_sim altitude_hold.launch.py
ros2 launch drone_sim battery_demo.launch.py
ros2 launch drone_sim swarm.launch.py drone_count:=5

# マニピュレータ: 平面到達デモ（JointState / TF / tool pose）
ros2 launch manipulator_sim planar_reach_demo.launch.py
```

RViz を使う場合は、ビルド後に `install/<package>/share/<package>/rviz/` 以下の設定ファイルを開いてください。

## 詳細仕様ドキュメント

ROSシミュレーションの挙動、topic / service / action、launch シナリオ、主要パラメータは [`docs/simulation_spec.md`](docs/simulation_spec.md) にまとめています。

## Docker 開発環境

Ubuntu 26.04 + ROS 2 Lyrical の開発コンテナを既定にしています。Ubuntu 20.04 + ROS 2 Foxy など、ROS 2 公式 Docker イメージに存在する組み合わせも `ROS_DISTRO` と `UBUNTU_CODENAME` で指定できます。

```bash
# Lyrical (デフォルト・推奨)
docker compose -f docker/compose.yml build
docker compose -f docker/compose.yml run --rm ros2sample

# Jazzy を使う場合
ROS_DISTRO=jazzy UBUNTU_CODENAME=noble docker compose -f docker/compose.yml build
ROS_DISTRO=jazzy UBUNTU_CODENAME=noble docker compose -f docker/compose.yml run --rm ros2sample

# Foxy / Ubuntu 20.04 を使う場合（既存環境互換）
ROS_DISTRO=foxy UBUNTU_CODENAME=focal docker compose -f docker/compose.yml build
ROS_DISTRO=foxy UBUNTU_CODENAME=focal docker compose -f docker/compose.yml run --rm ros2sample
```

コンテナ内ではリポジトリが `/workspace/Ros2Sample` にマウントされます。

## CI

GitHub Actions は `.github/workflows/ci.yml` で定義しています。自動トリガーは無効のまま、手動実行 (`workflow_dispatch`) で Ubuntu 20.04/Foxy、Ubuntu 24.04/Lyrical、Ubuntu 24.04/Jazzy の基本検証を実行できます。

1. `rosdep install`
2. `./scripts/lint.sh`
3. `./scripts/build.sh`
4. `colcon test`
5. `colcon test-result --verbose`

`src/` のパッケージが追加・変更されても、ワークスペース全体の基本検証を同じ流れで実行できるようにしています。

## トラブルシューティング

### `colcon: command not found`

`python3-colcon-common-extensions` が未インストールです。

```bash
sudo apt install -y python3-colcon-common-extensions
```

### `rosdep: command not found`

`python3-rosdep` をインストールしてください。

```bash
sudo apt install -y python3-rosdep
```

### `rosdep init` が失敗する

既に初期化済みの場合があります。`rosdep update` を実行してください。

```bash
rosdep update
```

### `No packages found` と表示される

`src/` が取得できていない、またはパッケージ構成が壊れている可能性があります。`find src -maxdepth 2 -name package.xml -print` と `colcon list` で認識状況を確認してください。

### ROS 2 ディストリビューションを切り替えたい

環境変数 `ROS_DISTRO` またはスクリプト引数で指定します。

```bash
# Jazzy に切り替える例
source /opt/ros/jazzy/setup.bash
./scripts/rosdep-install.sh jazzy

# Rolling に切り替える例
source /opt/ros/rolling/setup.bash
./scripts/rosdep-install.sh rolling
```

## English summary

Ros2Sample is a ROS 2 workspace for robot and drone examples. Documentation and tooling are Japanese-first, with Ubuntu 20.04 / 24.04 / 26.04 and ROS 2 Foxy / Lyrical / Jazzy / Kilted / Rolling in mind. The default and CI-primary distribution is Lyrical Luth (May 2026 LTS). Use `scripts/build.sh`, `scripts/lint.sh`, and `scripts/rosdep-install.sh` for common development tasks.

Packages include `ground_robot_sim` (diff-drive robot with synthetic LiDAR, PID waypoint following, emergency stop service), `drone_sim` (quadrotor with PID altitude hold, battery monitoring, emergency landing), and `sample_interfaces` (custom msg/srv/action definitions for learning ROS 2 interface design).
