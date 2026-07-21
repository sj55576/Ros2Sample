# openusd_bridge

ROS 2 の `nav_msgs/msg/Odometry` を購読し、位置・姿勢を OpenUSD の時系列
`Xform` として `.usda` / `.usdc` / `.usd` ファイルへ記録する最小サンプルです。

## できること

- ROS timestamp を USD time code へ変換
- ROS の quaternion `(x, y, z, w)` を OpenUSD の `Gf.Quatd` へ変換
- `/World/Robot` の translate / orient を時系列サンプルとして記録
- Z-up、meters-per-unit = 1 の stage と簡易 robot / ground geometry を作成

## OpenUSD Python bindings

OpenUSD は通常の `rosdep` 対象に含めず、実行時だけ必要なオプション依存にして
います。ROS 2 が使う Python 環境へ `pxr` module を提供してください。PyPI wheel
を利用する場合の例です。

```bash
python3 -m venv --system-site-packages .venv-openusd
source .venv-openusd/bin/activate
python -m pip install --upgrade pip
python -m pip install usd-core
python -c "from pxr import Usd; print(Usd.GetVersion())"
```

OpenUSD のビルドや別ディストリビューションを使う場合も、`ros2` コマンドを
起動する Python から `from pxr import Usd` が成功することを確認してください。

## ビルドと実行

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select openusd_bridge
source install/setup.bash

ros2 launch openusd_bridge ground_robot_openusd.launch.py
```

数秒走らせて `Ctrl+C` で終了すると、既定では
`/tmp/ros2_openusd/robot_motion.usda` が保存されます。保存先は変更できます。

```bash
ros2 launch openusd_bridge ground_robot_openusd.launch.py \
  output_path:=$HOME/robot_motion.usda
```

既に動作している任意の `Odometry` topic を記録することもできます。

```bash
ros2 run openusd_bridge odom_to_usd --ros-args \
  -p input_topic:=/drone_1/odom \
  -p output_path:=/tmp/drone_motion.usda \
  -p robot_prim_path:=/World/Drone
```

生成ファイルは `usdview /tmp/ros2_openusd/robot_motion.usda` などで開き、timeline
を再生して確認します。

## parameter

| parameter | default | 説明 |
| --- | --- | --- |
| `input_topic` | `odom` | 購読する `nav_msgs/msg/Odometry` topic |
| `output_path` | `/tmp/ros2_openusd/robot_motion.usda` | 保存先。`.usd`、`.usda`、`.usdc` を許可 |
| `robot_prim_path` | `/World/Robot` | pose を記録する absolute prim path |
| `time_codes_per_second` | `30.0` | 1 秒あたりの USD time code |
| `save_every_n_samples` | `30` | root layer を保存する間隔 |

これは ROS 2 と OpenUSD のデータ表現を学ぶための片方向 recorder です。USD stage
から ROS 2 への再生、TF tree 全体、sensor data、URDF の mesh/material 変換、
live collaboration は対象外です。

## 参考資料

- [OpenUSD API: UsdStage](https://openusd.org/release/api/class_usd_stage.html)
- [OpenUSD API: UsdGeomXformable](https://openusd.org/release/api/class_usd_geom_xformable.html)
