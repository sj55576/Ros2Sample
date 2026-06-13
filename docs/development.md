# 開発ガイド

この文書は Ros2Sample の開発者向けメモです。README は利用者向けの入口として、ここでは日々の開発フローを補足します。

## ブランチ作業の基本

- 他の作業者の変更を上書きしないでください。
- `src/` にパッケージを追加・改名したら、README のパッケージ一覧と実行例も更新してください。
- 外部ソース依存を追加した場合は、`ros2.repos` にリポジトリ URL とバージョンを固定してください。
- apt/rosdep 依存を追加した場合は、各パッケージの `package.xml` に宣言してください。

## 推奨コマンド

```bash
source /opt/ros/lyrical/setup.bash
./scripts/rosdep-install.sh lyrical
./scripts/lint.sh
./scripts/build.sh
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## ROS 2 ディストリビューション方針

- **Lyrical Luth**: Ubuntu 26.04 を一次プラットフォームとする 2026年5月リリースの新 LTS です。CI の最優先ターゲットです。Ubuntu 24.04 (Noble) でも動作します。
- **Jazzy**: Ubuntu 24.04 の安定版です。Jazzy は 2029年まで引き続きサポートされるため、CI でも検証を継続します。
- **Foxy**: Ubuntu 20.04 の既存環境互換ターゲットです。EOL 済みのため新規採用は非推奨ですが、軽量 Python サンプルのビルド・実行互換性を維持します。Gazebo/GZ 連携は `ros_gz_*` パッケージ事情が異なるため対象外です。
- **Kilted**: Jazzy/Lyrical からの差分を確認する開発候補です。
- **Rolling**: 最新 API 確認用です。破壊的変更が入る可能性を前提に扱います。

## 現在のサンプルパッケージ

| パッケージ | 役割 |
| --- | --- |
| `ground_robot_sim` | 地上ロボット向けの軽量 ROS 2 Python サンプルです。 |
| `drone_sim` | ドローン向けの軽量 ROS 2 Python サンプルです。 |

## パッケージ追加時のチェックリスト

1. `src/<package_name>/package.xml` に依存関係を明記する。
2. `colcon list` でパッケージが検出されることを確認する。
3. `./scripts/rosdep-install.sh <rosdistro>` を実行する。
4. `./scripts/build.sh` を実行する。
5. テストがある場合は `colcon test` と `colcon test-result --verbose` を実行する。
6. README のパッケージ一覧と実行例を更新する。
