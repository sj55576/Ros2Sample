# 23. ROS 2 の軌跡を OpenUSD へ記録する

この章では、地上ロボットが publish する `nav_msgs/msg/Odometry` を
`openusd_bridge` で購読し、OpenUSD stage の animation として保存します。

## 学習目標

- ROS timestamp と USD time code の関係を理解する
- ROS quaternion と `Gf.Quatd` の成分順の違いを理解する
- topic をファイルベースの scene description へ橋渡しする

## 準備

OpenUSD Python bindings はオプション依存です。ROS 2 package を見つけられる venv を
作り、その中へ PyPI の `usd-core` wheel を導入する例を示します。

```bash
python3 -m venv --system-site-packages .venv-openusd
source .venv-openusd/bin/activate
python -m pip install --upgrade pip
python -m pip install usd-core
python -c "from pxr import Usd; print(Usd.GetVersion())"

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 記録する

```bash
ros2 launch openusd_bridge ground_robot_openusd.launch.py
```

別 terminal で odometry が流れていることを確認します。

```bash
ros2 topic hz /odom
```

数秒後に launch を `Ctrl+C` で終了します。stage は既定で
`/tmp/ros2_openusd/robot_motion.usda` に保存されます。

## stage を確認する

OpenUSD tool が利用できる環境では、構文と composition を検証できます。

```bash
usdchecker /tmp/ros2_openusd/robot_motion.usda
usdview /tmp/ros2_openusd/robot_motion.usda
```

`usdview` の timeline を再生し、青い robot body が巡回すれば成功です。USDA は
text format なので、`xformOp:translate.timeSamples` と
`xformOp:orient.timeSamples` を editor で直接観察することもできます。

## 別 topic を記録する

`input_topic`、保存先、prim path は parameter で切り替えられます。

```bash
ros2 run openusd_bridge odom_to_usd --ros-args \
  -p input_topic:=/drone_1/odom \
  -p output_path:=/tmp/drone_motion.usda \
  -p robot_prim_path:=/World/Drone
```

この最小実装は pose recorder に限定しています。次の発展課題として、複数 robot の
prim 分離、TF tree の階層化、URDF mesh/material の参照、USD stage から ROS 2 への
playback を検討できます。
