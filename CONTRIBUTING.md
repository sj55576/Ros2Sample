# コントリビューションガイド

Ros2Sample への貢献に興味を持っていただきありがとうございます。このドキュメントは、issue 報告や Pull Request を送る前に確認してほしい最低限のルールをまとめたものです。日々の開発フローの詳細は [`docs/development.md`](docs/development.md) を参照してください。

## ドキュメント方針

このリポジトリは **日本語を第一言語** としています。

- README、チュートリアル、PR 説明文は日本語で記述してください。
- コードの docstring は既存パッケージの慣例に合わせて英語で統一しています。コメントやログメッセージは対象パッケージの既存スタイルに合わせてください。
- 英語話者向けの要約が必要な場合は、README 末尾の「English summary」のように、日本語ドキュメントに付随する短い補足として追加してください。英語のみのドキュメント追加は避けてください。

## 開発環境のセットアップ

1. README の [セットアップ](README.md#セットアップ) と [依存関係](README.md#依存関係) を参照し、ROS 2 環境（Foxy / Jazzy / Lyrical など）を用意してください。
2. 日々の開発フロー（パッケージ追加時のチェックリストや推奨コマンドなど）は [`docs/development.md`](docs/development.md) にまとめています。作業前に一読してください。
3. Docker で環境を揃えたい場合は README の [Docker 開発環境](README.md#docker-開発環境) を利用してください。

## ブランチ・コミットメッセージの慣例

- `main` ブランチに対して直接コミットせず、作業用ブランチを作成してください。
- コミットメッセージは [Conventional Commits](https://www.conventionalcommits.org/) 風に、`<type>(<scope>): <概要>` の形式を基本としています。`scope` は対象パッケージ名（例、`ground_robot_sim`）を指定すると分かりやすくなります。
  - `feat: 主要ノードの動的パラメータ更新に対応 (#51)`
  - `feat(sensor_fusion_sim): add EKF node for GPS/IMU/odom fusion (#46)`
  - `feat(ground_robot_sim): キーボード teleop ノードを追加 (#48)`
  - `docs: マルチロボット通信と DDS Discovery チュートリアル追加 (#38)`
  - `fix: 03_launch_paramsの無効なros2コマンドを修正`
- 主に使う `type` は次の通りです。
  - `feat`: 新しいノード・パッケージ・チュートリアルなどの追加
  - `docs`: ドキュメントのみの追加・修正
  - `fix`: 不具合修正
  - `test`: テストの追加・修正
  - `refactor`: 挙動を変えないコード整理
- コミットメッセージの概要は日本語・英語のどちらでも構いませんが、本文で背景や変更点を補足する場合は日本語を優先してください。
- 対応する issue がある場合は、コミットメッセージや PR 本文に `(#123)` の形式で番号を含めてください。

## ローカルでの lint・テスト実行

Pull Request を送る前に、ROS 2 環境を読み込んだ上で次のコマンドを実行し、成功することを確認してください。

```bash
source /opt/ros/lyrical/setup.bash   # 利用しているディストリビューションに合わせて変更
./scripts/rosdep-install.sh lyrical
./scripts/build.sh
./scripts/lint.sh
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

特定パッケージの pytest（純粋関数のユニットテスト）のみを素早く確認したい場合は、各パッケージディレクトリで直接実行できます。

```bash
cd src/ground_robot_sim
python3 -m pytest test/ -q
```

## 変更内容に応じた更新箇所

- `src/` にパッケージを追加・改名した場合は、README のパッケージ一覧・実行例と `docs/development.md` の一覧を更新してください。
- topic / service / action / launch / parameter を変更した場合は、`docs/simulation_spec.md`、`docs/implementation_spec.md`、該当パッケージの README を更新してください。
- 外部リポジトリ依存を追加した場合は `ros2.repos` に、apt/rosdep 依存を追加した場合は各パッケージの `package.xml` に反映してください。
- CI に影響する変更（`src/`、`scripts/`、`ros2.repos`、`.github/workflows/`）を行った場合は、`.github/workflows/ci.yml` が対象パッケージを正しくビルド・テストできるか確認してください。

## Issue / Pull Request の作成

- バグ報告、機能要望、質問はそれぞれ専用の issue テンプレート（`.github/ISSUE_TEMPLATE/`）を利用してください。
- Pull Request は [`.github/pull_request_template.md`](.github/pull_request_template.md) のチェックリストに沿って、変更概要・関連 issue・動作確認結果を記載してください。

## その他

- 他の作業者の変更を上書きしないよう、作業前に最新の `main` ブランチを取り込んでください。
- 不明点があれば、まず「質問・チュートリアルサポート」issue テンプレートで質問してください。
