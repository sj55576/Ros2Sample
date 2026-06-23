# ROSシミュレーション仕様書

この文書は、Ros2Sample に含まれる ROS 2 サンプルの「シミュレーションで何が起きるか」と「各サンプルの詳細仕様」をまとめたものです。Gazebo などの重量級物理エンジンを前提にせず、ROS 2 の topic、service、action、launch、namespace、TF、センサー風データの流れを短時間で確認するための軽量シミュレーションを対象にしています。

ノード間接続、制御式、入力検証、失敗モード、受け入れ確認については [implementation_spec.md](implementation_spec.md) を参照してください。

## 1. 全体像

### 1.1 収録パッケージ

| パッケージ | 役割 | 主なシミュレーション対象 |
| --- | --- | --- |
| `ground_robot_sim` | 差動二輪風の地上ロボット、LiDAR 風スキャン、ウェイポイント追従、障害物停止・回避、複数ロボット namespace | 2D 平面移動ロボット |
| `drone_sim` | クアッドローター風の運動、3D waypoint、PID 高度維持、風外乱、ジオフェンス、フォーメーション制御、テレメトリ、バッテリー消費、緊急着陸、swarm namespace | 3D 空間内の簡易ドローン |
| `manipulator_sim` | 2 自由度平面マニピュレータ、関節追従、順運動学、逆運動学、TF | 2 リンク平面アーム |
| `sensor_fusion_sim` | ノイズ付きセンサー、相補フィルタ、ライフサイクルノード、QoS、コールバックグループ | センサーフュージョン |
| `sample_interfaces` | サンプル共通の msg / srv / action 定義 | 状態取得と waypoint action |

### 1.2 設計方針

- **軽量性を優先**: Python / ament ベースで、物理エンジンなしでも topic と TF の流れを確認できます。
- **ROS 2 の基本要素を分離して学べる構成**: シミュレータ、指令ノード、監視ノードを分け、launch で組み合わせます。
- **CI・教育用途を想定**: 純粋関数の単体テスト、lint、colcon build/test を通しやすい構成です。
- **namespace 対応**: 地上ロボットとドローンは複数台起動時に topic と TF frame の衝突を避けるサンプルを持ちます。

## 2. シミュレーションの見え方

### 2.1 地上ロボット

地上ロボットは正方形の 2D ワールド内を移動する差動二輪風ロボットです。`cmd_vel` の直進速度と旋回速度を積分して `odom` と `odom -> base_link` TF を更新し、前方 180 度の擬似 `LaserScan` を発行します。LiDAR は円形障害物とワールド境界に対してレイを飛ばし、最短距離を range 値として返します。

代表的なデモの挙動は次の通りです。

- `diff_drive_patrol.launch.py`: 一定時間直進し、一時停止して旋回する動作を繰り返します。
- `lidar_obstacle_stop.launch.py`: 前方扇形内の障害物が停止距離内に入ると停止し、障害物がない場合は低速で前進します。
- `lidar_obstacle_avoid.launch.py`: 前方を左右に分けて最短距離を見積もり、近い側と反対方向へ旋回しながら障害物を避けます。
- `waypoint_follower.launch.py`: `odom` を閉ループで読み、PID 制御で `(x, y)` waypoint を順に追従します。
- `multi_robot.launch.py`: `/robot1`、`/robot2`、`/robot3` の namespace で 3 台を同時起動します。

### 2.2 ドローン

ドローンは 3D 空間内の簡易クアッドローターです。力学を厳密に解くのではなく、目標速度に対して加速度制限付きで速度を近づけ、位置と yaw を積分します。状態として `odom`、`pose`、`imu`、TF、`robot_status` を発行します。

代表的なデモの挙動は次の通りです。

- `single_quad_waypoint.launch.py`: `waypoint_commander` が `setpoint_pose` を発行し、`sim_drone` が目標位置へ追従します。
- `altitude_hold.launch.py`: `altitude_hold` が `odom` の高度を見て `cmd_vel.linear.z` を PID 制御し、指定高度を維持します。
- `battery_demo.launch.py`: waypoint 飛行中に `battery_monitor` が消費電力を見積もり、低バッテリー時に `emergency_land` が降下指令を出します。
- `wind_demo.launch.py`: `wind_disturbance` が時間変化する風速を発行し、`geofence_monitor` が境界逸脱を検出し、`telemetry_logger` が飛行統計を要約します。
- `formation_demo.launch.py`: `/leader` の odom をもとに `/follower_1` と `/follower_2` が相対 offset を保つ setpoint を生成します。
- `swarm.launch.py`: `/drone_1` 以降の namespace で複数ドローンを格子状に配置し、それぞれ別の waypoint を巡回します。

### 2.3 センサーフュージョン

センサーフュージョンは、ノイズを含むセンサーデータから推定状態を復元する過程を学ぶためのデモです。`noisy_sensor_node` が円軌道の真値に GPS・IMU・wheel odom のノイズを加えて publish し、`complementary_filter` がそれらを相補フィルタで融合して `fused_odom` を出力します。`lifecycle_data_recorder` はライフサイクル管理されたノードで、activate 状態の間だけ `fused_odom` をバッファに記録します。

- `sensor_fusion_demo.launch.py`: 3 ノードを同時起動し、レコーダーを自動で configure → activate へ遷移させます。

このデモでは次の ROS 2 機能を実践できます。

- **QoS プロファイル**: GPS / wheel odom は RELIABLE、IMU は BEST_EFFORT の QoS を使い分けます。
- **コールバックグループ**: センサー入力に `ReentrantCallbackGroup`、publish に `MutuallyExclusiveCallbackGroup` を使い、`MultiThreadedExecutor` で並行処理します。
- **動的パラメータ更新**: `gps_alpha`、`odom_alpha`、`imu_yaw_weight` を実行中に `ros2 param set` で変更できます。
- **ライフサイクルノード**: レコーダーが `unconfigured → inactive → active` の状態遷移を持ち、launch から自動駆動されます。

### 2.4 マニピュレータ

マニピュレータは 2 リンクの平面アームです。`joint_target` を受け取り、関節速度上限に従って現在関節角を目標角へ近づけます。各 tick で `joint_states`、手先 `tool_pose`、`base_link -> link1 -> tool0` TF を発行します。

`planar_reach_demo.launch.py` では、`target_commander` が平面 `(x, y)` 目標列を逆運動学で関節角へ変換し、`manipulator_simulator` がそれを追従します。

## 3. 共通インターフェース

### 3.1 `sample_interfaces/msg/RobotStatus`

地上ロボットとドローンが共通で発行する状態メッセージです。

| フィールド | 型 | 意味 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 時刻と基準 frame |
| `robot_name` | `string` | ロボット識別名 |
| `state` | `string` | `idle`、`moving`、`emergency_stop`、`landing`、`landed` などの状態 |
| `battery_percentage` | `float64` | バッテリー残量 [%] |
| `position` | `geometry_msgs/Point` | odom frame 上の位置 |
| `linear_velocity` | `geometry_msgs/Vector3` | 線形速度 |
| `heading_rad` | `float64` | yaw 角 [rad] |

### 3.2 `sample_interfaces/srv/GetRobotStatus`

現在の `RobotStatus` スナップショットを service で取得します。地上ロボットとドローンのシミュレータが `get_robot_status` service を提供します。

### 3.3 `sample_interfaces/action/NavigateWaypoints`

地上ロボット向けの waypoint action 定義です。goal は `geometry_msgs/PoseStamped[] waypoints`、`loop`、`tolerance_m` を持ち、result は成功可否、完了 waypoint 数、メッセージを返します。feedback は現在 index、総 waypoint 数、現在 waypoint までの距離、現在位置です。

## 4. 地上ロボット仕様

### 4.1 `ground_robot_node`

| 項目 | 仕様 |
| --- | --- |
| ノード名 | `ground_robot` |
| 入力 topic | `cmd_vel` (`geometry_msgs/Twist`) |
| 出力 topic | `odom`、`scan`、`robot_status` |
| service | `emergency_stop`、`reset_emergency`、`get_robot_status` |
| TF | `odom_frame -> base_frame`、`base_frame -> laser_frame` |
| 運動モデル | `x += v cos(yaw) dt`、`y += v sin(yaw) dt`、`yaw += omega dt` |
| LiDAR | -90 度から +90 度までの 180 度スキャン |
| 障害物 | `[x, y, radius, ...]` の円形障害物リスト |
| ワールド | `world_half_size` を半幅とする正方形領域 |

主なパラメータは次の通りです。

| パラメータ | 既定値 | 説明 |
| --- | --- | --- |
| `publish_rate` | `30.0` | odom / TF / status 更新周期 [Hz] |
| `scan_rate` | `10.0` | LaserScan 発行周期 [Hz] |
| `max_linear_speed` | `0.8` | `cmd_vel.linear.x` の上限 [m/s] |
| `max_angular_speed` | `1.8` | `cmd_vel.angular.z` の上限 [rad/s] |
| `scan_range_min` / `scan_range_max` | `0.08` / `8.0` | LiDAR 距離範囲 [m] |
| `scan_samples` | `181` | 180 度スキャンのサンプル数 |
| `world_half_size` | `5.0` | 正方形ワールド半幅 [m] |

### 4.2 地上ロボット制御ノード

| ノード | 入力 | 出力 | 制御内容 |
| --- | --- | --- | --- |
| `diff_drive_patrol` | なし | `cmd_vel` | `forward`、`pause_before_turn`、`turn`、`pause_before_forward` の状態遷移で巡回 |
| `lidar_obstacle_stop` | `scan` | `cmd_vel` | 前方扇形の最短距離が `stop_distance` 以下なら停止 |
| `lidar_obstacle_avoid` | `scan` | `cmd_vel` | 前方左右の最短距離を比較し、近い側から離れるよう旋回 |
| `waypoint_follower` | `odom` | `cmd_vel` | waypoint への方位誤差と距離を PID 制御 |
| `navigate_waypoints_server` | action goal、`odom` | `cmd_vel`、action feedback/result | action で与えられた waypoint 列を追従 |

### 4.3 地上ロボット launch シナリオ

| launch | 主な起動ノード | シナリオ |
| --- | --- | --- |
| `diff_drive_patrol.launch.py` | `robot_state_publisher`、`ground_robot_node`、`diff_drive_patrol` | 開ループ巡回 |
| `lidar_obstacle_stop.launch.py` | `ground_robot_node`、`lidar_obstacle_stop` | 障害物検出時に停止 |
| `lidar_obstacle_avoid.launch.py` | `ground_robot_node`、`lidar_obstacle_avoid` | 障害物を避けながら前進 |
| `waypoint_follower.launch.py` | `ground_robot_node`、`waypoint_follower` | 正方形 waypoint 追従 |
| `navigate_waypoints.launch.py` | `ground_robot_node`、`navigate_waypoints_server` | action 経由の waypoint 追従 |
| `multi_robot.launch.py` | 3 組の `ground_robot_node` + `diff_drive_patrol` | namespace 付き複数台巡回 |
| `gazebo.launch.py` | Gazebo Sim、`ros_gz_bridge`、`diff_drive_patrol` | Gazebo 連携の入口。WSL2/headless を想定して GUI は既定無効 |

## 5. ドローン仕様

### 5.1 `sim_drone`

| 項目 | 仕様 |
| --- | --- |
| ノード名 | `sim_drone` |
| 入力 topic | `cmd_vel`、`setpoint_pose`、`battery` |
| 出力 topic | `odom`、`pose`、`imu`、`robot_status` |
| service | `get_robot_status` |
| TF | `frame_id -> base_frame_id` |
| 運動モデル | 目標速度へ加速度制限付きで近づけ、位置と yaw を積分 |
| 目標選択 | 新しい `setpoint_pose` が timeout 内なら位置追従を優先し、なければ timeout 内の `cmd_vel` を使用 |

主なパラメータは次の通りです。

| パラメータ | 既定値 | 説明 |
| --- | --- | --- |
| `publish_rate_hz` | `50.0` | 状態更新周期 [Hz] |
| `linear_accel_limit` | `3.0` | 並進加速度上限 [m/s²] |
| `yaw_accel_limit` | `4.0` | yaw 加速度上限 [rad/s²] |
| `max_linear_speed` | `5.0` | 並進速度上限 [m/s] |
| `max_yaw_rate` | `2.5` | yaw 角速度上限 [rad/s] |
| `cmd_timeout_sec` | `0.6` | `cmd_vel` の有効時間 [s] |
| `setpoint_timeout_sec` | `1.0` | `setpoint_pose` の有効時間 [s] |
| `position_kp` | `1.2` | setpoint 位置追従ゲイン |
| `yaw_kp` | `1.8` | setpoint 方位追従ゲイン |

### 5.2 ドローン制御・監視ノード

| ノード | 入力 | 出力 / service | 仕様 |
| --- | --- | --- | --- |
| `waypoint_commander` | `odom` | `setpoint_pose` | 3D waypoint `[x, y, z, ...]` を順に発行。到着後 `hold_time_sec` 待機 |
| `altitude_hold` | `odom` | `cmd_vel` | 高度誤差を PID に通して `linear.z` を発行 |
| `wind_disturbance` | なし | `wind_velocity` | base wind、sin gust、簡易 turbulence を合成した風速ベクトルを発行 |
| `geofence_monitor` | `odom` | `geofence_breach`、`geofence_setpoint` | 3D bounding box を監視し、逸脱時に境界内へ戻す setpoint を発行 |
| `formation_controller` | leader `odom` | `setpoint_pose` | leader 位置に offset を足し、平滑化した follower 目標を発行 |
| `telemetry_logger` | `odom`、`battery` | `telemetry_summary` | 総飛行距離、最大高度、最大速度、バッテリー、飛行時間、現在位置を集計 |
| `battery_monitor` | `cmd_vel` | `battery`、`low_battery` | idle + throttle 比例の消費電力で Wh を減算 |
| `emergency_land` | `low_battery`、`odom` | `cmd_vel`、`emergency_land` service | 低バッテリーまたは service 呼び出しで降下速度を発行 |

### 5.3 ドローン launch シナリオ

| launch | 主な起動ノード | シナリオ |
| --- | --- | --- |
| `single_quad_waypoint.launch.py` | `sim_drone`、`waypoint_commander` | 1 台の 3D waypoint 飛行 |
| `altitude_hold.launch.py` | `sim_drone`、`altitude_hold` | PID 高度維持 |
| `battery_demo.launch.py` | `sim_drone`、`waypoint_commander`、`battery_monitor`、`emergency_land` | 飛行、電池消費、低電池時の自動着陸 |
| `wind_demo.launch.py` | `sim_drone`、`waypoint_commander`、`wind_disturbance`、`geofence_monitor`、`telemetry_logger` | 風外乱、境界監視、飛行統計の統合デモ |
| `formation_demo.launch.py` | `/leader` の `sim_drone` + `waypoint_commander`、follower ごとの `sim_drone` + `formation_controller` | leader-follower の相対位置維持 |
| `swarm.launch.py` | namespace ごとの `sim_drone`、`waypoint_commander` | 小規模 swarm。`drone_count`、`spacing_m`、`altitude_m` で台数・配置を変更 |

## 6. マニピュレータ仕様

### 6.1 `manipulator_simulator`

| 項目 | 仕様 |
| --- | --- |
| ノード名 | `manipulator_simulator` |
| 入力 topic | `joint_target` (`sensor_msgs/JointState`) |
| 出力 topic | `joint_states`、`tool_pose` |
| TF | `base_link -> link1 -> tool0` |
| 関節数 | 2 |
| 運動モデル | 各関節を `max_joint_speed` の範囲で目標角へ近づける |
| 手先姿勢 | 順運動学で `(x, y)` を計算し、yaw は `theta1 + theta2` |

主なパラメータは次の通りです。

| パラメータ | 既定値 | 説明 |
| --- | --- | --- |
| `joint_names` | `[joint1, joint2]` | 関節名 |
| `link_lengths` | `[0.8, 0.6]` | リンク長 [m] |
| `initial_joint_positions` | `[0.0, 0.0]` | 初期関節角 [rad] |
| `max_joint_speed` | `1.2` | 関節速度上限 [rad/s] |
| `publish_rate_hz` | `30.0` | 状態更新周期 [Hz] |

### 6.2 `target_commander`

| 項目 | 仕様 |
| --- | --- |
| 入力 topic | `joint_states` |
| 出力 topic | `joint_target` |
| 目標形式 | `targets_xy` に `[x1, y1, x2, y2, ...]` を指定 |
| 逆運動学 | 2 リンク平面アームの IK。`elbow_up` で解の枝を選択 |
| 到着判定 | 2 関節とも目標角との差が `tolerance_rad` 以下 |
| 待機 | 到着後 `hold_time_sec` 待機して次目標へ進む |

## 7. センサーフュージョン仕様

### 7.1 `noisy_sensor_node`

| 項目 | 仕様 |
| --- | --- |
| ノード名 | `noisy_sensor_node` |
| 出力 topic | `gps` (`PointStamped`, RELIABLE, 1 Hz)、`imu` (`Imu`, BEST_EFFORT, 50 Hz)、`wheel_odom` (`Odometry`, RELIABLE, 10 Hz)、`ground_truth` (`Odometry`, RELIABLE, 10 Hz) |
| TF | `world -> base_link_truth` |
| 運動モデル | 円軌道 `x = R cos(ωt)`、`y = R sin(ωt)` にノイズを付加 |

主なパラメータは次の通りです。

| パラメータ | 既定値 | 説明 |
| --- | --- | --- |
| `circle_radius` | `5.0` | 円軌道半径 [m] |
| `circle_omega` | `0.3` | 円軌道角速度 [rad/s] |
| `gps_rate_hz` | `1.0` | GPS 発行周期 [Hz] |
| `gps_noise_stddev` | `0.5` | GPS ノイズ標準偏差 [m] |
| `imu_rate_hz` | `50.0` | IMU 発行周期 [Hz] |
| `imu_accel_stddev` | `0.1` | IMU 加速度ノイズ標準偏差 [m/s²] |
| `imu_gyro_stddev` | `0.02` | IMU ジャイロノイズ標準偏差 [rad/s] |
| `odom_rate_hz` | `10.0` | wheel odom 発行周期 [Hz] |
| `odom_noise_stddev` | `0.05` | wheel odom ノイズ標準偏差 [m] |

### 7.2 `complementary_filter`

| 項目 | 仕様 |
| --- | --- |
| ノード名 | `complementary_filter` |
| 入力 topic | `gps` (RELIABLE)、`imu` (BEST_EFFORT)、`wheel_odom` (RELIABLE) |
| 出力 topic | `fused_odom` (`Odometry`, RELIABLE)、`filter_diagnostics` (`String`) |
| フュージョン | GPS と wheel odom の位置を `alpha` で相補フィルタ、IMU yaw を `imu_yaw_weight` でブレンド |
| コールバックグループ | センサー: `ReentrantCallbackGroup`、publish timer: `MutuallyExclusiveCallbackGroup` |
| Executor | `MultiThreadedExecutor` (4 スレッド) |
| 動的パラメータ | `gps_alpha`、`odom_alpha`、`imu_yaw_weight` を実行中に変更可能（0--1 のバリデーション付き） |

主なパラメータは次の通りです。

| パラメータ | 既定値 | 説明 |
| --- | --- | --- |
| `gps_alpha` | `0.15` | GPS 位置の相補フィルタ重み |
| `odom_alpha` | `0.30` | wheel odom 位置の相補フィルタ重み |
| `imu_yaw_weight` | `0.05` | IMU yaw ブレンド重み |
| `publish_rate_hz` | `20.0` | フュージョン結果の発行周期 [Hz] |

### 7.3 `lifecycle_data_recorder`

| 項目 | 仕様 |
| --- | --- |
| ノード名 | `lifecycle_data_recorder` |
| ノード種別 | `LifecycleNode` |
| 入力 topic | `fused_odom` (`Odometry`、activate 時にサブスクライブ) |
| 出力 topic | `recording_status` (`String`)、`recording_summary` (`String`、JSON) |
| 状態遷移 | `on_configure` → `on_activate` → `on_deactivate` → `on_cleanup` / `on_shutdown` |
| バッファ | 最大 `max_buffer_size` 件を保持（古いレコードは先頭から削除） |

主なパラメータは次の通りです。

| パラメータ | 既定値 | 説明 |
| --- | --- | --- |
| `max_buffer_size` | `500` | データバッファ最大サイズ |
| `publish_rate_hz` | `1.0` | ステータス発行周期 [Hz] |
| `input_topic` | `fused_odom` | 入力 topic 名 |

### 7.4 センサーフュージョン launch シナリオ

| launch | 主な起動ノード | シナリオ |
| --- | --- | --- |
| `sensor_fusion_demo.launch.py` | `noisy_sensor_node`、`complementary_filter`、`lifecycle_data_recorder` | ノイズ付きセンサーから相補フィルタでフュージョンし、ライフサイクルレコーダーで記録。レコーダーは自動 configure → activate される |

## 8. 実行・観測手順

### 8.1 ビルド

```bash
source /opt/ros/<rosdistro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 8.2 代表デモの起動

```bash
# 地上ロボット
ros2 launch ground_robot_sim waypoint_follower.launch.py
ros2 launch ground_robot_sim lidar_obstacle_avoid.launch.py
ros2 launch ground_robot_sim multi_robot.launch.py

# ドローン
ros2 launch drone_sim single_quad_waypoint.launch.py
ros2 launch drone_sim altitude_hold.launch.py target_altitude_m:=2.0
ros2 launch drone_sim battery_demo.launch.py
ros2 launch drone_sim wind_demo.launch.py
ros2 launch drone_sim formation_demo.launch.py
ros2 launch drone_sim swarm.launch.py drone_count:=5

# マニピュレータ
ros2 launch manipulator_sim planar_reach_demo.launch.py

# センサーフュージョン
ros2 launch sensor_fusion_sim sensor_fusion_demo.launch.py
```

### 8.3 topic / service の確認例

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /robot_status
ros2 topic echo /wind_velocity
ros2 topic echo /geofence_breach
ros2 topic echo /telemetry_summary
ros2 topic echo /fused_odom
ros2 topic echo /filter_diagnostics
ros2 topic echo /recording_status
ros2 service call /get_robot_status sample_interfaces/srv/GetRobotStatus
ros2 service call /emergency_stop std_srvs/srv/Trigger
ros2 service call /reset_emergency std_srvs/srv/Trigger
```

namespace 付きのデモでは、次のように prefix を付けて確認します。

```bash
ros2 topic echo /robot1/odom
ros2 topic echo /drone_1/pose
ros2 service call /drone_1/get_robot_status sample_interfaces/srv/GetRobotStatus
```

## 9. 仕様変更時の更新ポイント

- 新しいノード、topic、service、action を追加したら、この文書の該当表を更新してください。
- launch ファイルや config YAML の既定値を変更した場合は、シナリオ表とパラメータ表を更新してください。
- 物理エンジンや外部依存を追加した場合は、README、`package.xml`、`ros2.repos`、Docker 設定もあわせて更新してください。
