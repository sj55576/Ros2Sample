# チュートリアル 11 解答例: ビヘイビアツリー入門

## 演習 1: BT を設計する

### シナリオ

警備ロボットがエリアをパトロールする。障害物で通れなければ 3 回まで別経路を試みる。それでも失敗したら管理者に通知する。

### 考え方

このシナリオは「メイン動作（パトロール）が失敗しても最大 3 回リトライし、それでも駄目なら最終手段（通知）を実行する」という構造です。

- `RetryUntilSuccessful(num_attempts=3)` でリトライを実現
- リトライが全て失敗した場合は `Fallback` の次の子（通知）が実行される

### 解答 BT（ASCII アート表現）

```
Root
└── Sequence →
    ├── Fallback ?
    │   ├── RetryUntilSuccessful ◇ (num_attempts=3)
    │   │   └── Sequence →
    │   │       ├── ComputeAlternativePath □   ← 別経路を計算
    │   │       └── FollowPath □               ← その経路を追従
    │   └── NotifyOperator □                   ← 3 回失敗 → 管理者通知
    └── [次のパトロールポイントへ...]
```

### XML 形式での表現

Nav2 BT XML（BehaviorTree.CPP 形式）では以下のように記述します:

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="PatrolBT">
  <BehaviorTree ID="PatrolBT">
    <Sequence name="patrol_sequence">

      <Fallback name="navigate_or_notify">

        <!-- 最大 3 回リトライ -->
        <RetryUntilSuccessful num_attempts="3" name="retry_navigation">
          <Sequence name="navigate_attempt">
            <ComputeAlternativePath goal="{current_patrol_point}"
                                    path="{computed_path}"
                                    name="compute_path"/>
            <FollowPath path="{computed_path}"
                        name="follow_path"/>
          </Sequence>
        </RetryUntilSuccessful>

        <!-- リトライが 3 回とも失敗した場合 -->
        <NotifyOperator message="パトロール失敗: 障害物を回避できません"
                        name="notify_operator"/>

      </Fallback>

    </Sequence>
  </BehaviorTree>
</root>
```

### 実行フローの説明

```
1 回目の試行:
  ComputeAlternativePath → FAILURE（経路なし）
  Sequence → FAILURE
  RetryUntilSuccessful → リトライ（残り 2 回）

2 回目の試行:
  ComputeAlternativePath → SUCCESS（別経路発見）
  FollowPath → FAILURE（途中で詰まった）
  Sequence → FAILURE
  RetryUntilSuccessful → リトライ（残り 1 回）

3 回目の試行:
  ComputeAlternativePath → FAILURE（経路なし）
  Sequence → FAILURE
  RetryUntilSuccessful → FAILURE（上限到達）

Fallback の次の子へ:
  NotifyOperator → SUCCESS（通知送信完了）
Fallback → SUCCESS（通知が成功したので全体は成功扱い）
```

### ポイント

`RetryUntilSuccessful` は「子が SUCCESS を返すまで最大 N 回実行する」デコレータです。失敗回数が上限に達すると `FAILURE` を返すため、Fallback の次の候補（通知）が実行されます。リトライと最終手段の組み合わせは実際の Nav2 BT でも多用されるパターンです。

---

## 演習 2: FSM と BT を比較する

### シナリオ 1: 「障害物を検知したら 3 秒待って再試行する」

#### FSM での実装

```python
# navigate_waypoints_server.py（FSM 風）への追加

# 新しい状態を追加する必要がある
states = ['idle', 'moving', 'obstacle_wait', 'arrived', 'done']

# 遷移の追加
if state == 'moving':
    if obstacle_detected:
        state = 'obstacle_wait'
        wait_start_time = current_time  # タイマー開始
    elif distance_to_waypoint < threshold:
        state = 'arrived'

elif state == 'obstacle_wait':
    if current_time - wait_start_time >= 3.0:
        state = 'moving'  # 3 秒後に再試行
    else:
        stop_robot()  # 待機中は停止
```

**FSM の問題点**:
- 新しい状態 `obstacle_wait` を追加し、既存の全状態からの遷移を見直す必要がある
- タイマーの管理を自分で実装しなければならない
- 状態が増えるほど遷移の組み合わせが増え、バグが混入しやすい

#### BT での実装

```
Sequence →
├── Fallback ?
│   ├── IsPathClear ○           ← 経路が空きかチェック
│   └── Sequence →              ← 障害物あり時
│       ├── Wait(3.0) □         ← 3 秒待機（既成ノードを使うだけ）
│       └── ClearLocalCostmap □ ← コストマップをリセット
└── FollowPath □
```

**BT の利点**:
- `Wait` ノードは Nav2 が標準で提供（自分で実装不要）
- 既存のツリーに `Fallback` と `Wait` を組み込むだけ
- 他の部分に影響しない

### シナリオ 2: 「バッテリーが 20% 以下になったら充電ステーションへ移動する」

#### FSM での実装

```python
# 全ての状態からバッテリーチェックを追加する必要がある

elif state == 'moving':
    if battery_level < 0.20:  # 全状態に追加が必要
        saved_state = state
        saved_waypoint = current_waypoint
        state = 'going_to_charge'
    elif ...

elif state == 'going_to_charge':
    if at_charging_station:
        state = 'charging'

elif state == 'charging':
    if battery_level > 0.80:
        # 元の状態に戻す（どこに戻るか管理が複雑）
        state = saved_state
        current_waypoint = saved_waypoint
```

**FSM の問題点**:
- バッテリーチェックを全状態（idle/moving/arrived/done/obstacle_wait ...）に追加する必要がある
- 充電後に「どこまで戻るか」の状態管理が複雑
- 新しい状態（going_to_charge/charging）を追加するたびに既存の遷移をすべて見直す

#### BT での実装

```
Root
└── Fallback ?
    ├── Sequence →（通常ミッション - バッテリー十分な時）
    │   ├── IsBatteryOK ○ (threshold=0.20)   ← 条件チェック
    │   └── ExecuteWaypointMission □          ← 既存のミッション
    └── Sequence →（充電が必要な時）
        ├── NavigateToChargingStation □       ← 充電ステーションへ
        └── WaitForCharge(target=0.80) □      ← 充電完了を待つ
```

**BT の利点**:
- バッテリー条件チェックは 1 箇所（`IsBatteryOK` ノード）だけ
- ミッション部分（`ExecuteWaypointMission`）は変更不要
- 充電後は自動的にメインのミッションに戻る（Fallback が再評価するため）

### まとめ：どちらが保守しやすいか

| 比較項目 | FSM | BT |
|---------|-----|----|
| 機能追加 | 全状態に変更が必要（影響範囲が広い） | サブツリーの追加のみ（影響範囲が限定的） |
| 再利用 | 困難（状態間の依存が強い） | 容易（サブツリーを別ミッションに流用可能） |
| デバッグ | 状態遷移ログを追う | ノードの SUCCESS/FAILURE を追う |
| コード量 | 機能追加ごとに増える | ほぼ一定（ノードを組み合わせるだけ） |

**結論**: システムが複雑になるほど BT の優位性が大きくなります。シンプルな 2〜3 状態のロボットなら FSM で十分ですが、リカバリや複数の割り込み条件（バッテリー・障害物・タイムアウト等）が必要な場合は BT が推奨されます。

---

## 演習 3: Nav2 の BT ログを読む

### `/behavior_tree_log` の読み方

```bash
ros2 topic echo /behavior_tree_log
```

典型的な出力例:

```yaml
timestamp:
  sec: 1234
  nanosec: 567890
event_log:
  - timestamp: {sec: 1234, nanosec: 100000}
    node_name: ComputePathToPose
    node_status: RUNNING        ← 経路計算中
  - timestamp: {sec: 1234, nanosec: 850000}
    node_name: ComputePathToPose
    node_status: SUCCESS        ← 経路計算完了
  - timestamp: {sec: 1234, nanosec: 860000}
    node_name: FollowPath
    node_status: RUNNING        ← 経路追従中
  - timestamp: {sec: 1239, nanosec: 200000}
    node_name: FollowPath
    node_status: SUCCESS        ← ゴール到達
  - timestamp: {sec: 1239, nanosec: 210000}
    node_name: NavigateToPose
    node_status: SUCCESS        ← ナビゲーション全体が完了
```

### 各質問への解答

**Q1: どのノードが最初に実行されるか**

`ComputePathToPose` が最初に `RUNNING` になります。Nav2 の BT は Sequence の先頭から順に実行するため、経路計画（Planner Server の呼び出し）が最初のステップです。

**Q2: 経路追従中、各ノードの状態**

| ノード | 状態 | 理由 |
|-------|------|------|
| `ComputePathToPose` | `IDLE` または定期的に `RUNNING` | `RateController` デコレータによってリプランニング周期（例: 1 Hz）でのみ実行される |
| `FollowPath` | `RUNNING` | 経路追従中は継続的に RUNNING を返す |

**Q3: ゴールに到達したとき最後に SUCCESS を返すノード**

```
FollowPath → SUCCESS
  ↑
Sequence → SUCCESS
  ↑
NavigateToPose → SUCCESS（最外側のノードが最後に SUCCESS を返す）
```

最後に `SUCCESS` を返すのは、BT の最上位ノード（多くの場合 `NavigateToPose` または `Sequence`）です。`FollowPath` が `SUCCESS` を返すことで親の `Sequence` が `SUCCESS` になり、最終的にルートノードが `SUCCESS` を返します。

### ログを読む実践的なデバッグ手順

```bash
# 1. ナビゲーション開始前にログ監視を開始
ros2 topic echo /behavior_tree_log &

# 2. ナビゲーションを実行
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}}"

# 3. FAILURE が出た場合は直前のノード名に注目
#    ComputePathToPose が FAILURE → 経路が見つからない（マップの問題）
#    FollowPath が FAILURE       → 経路追従中にスタック（コントローラーの問題）
```

### ポイント

`/behavior_tree_log` は BT の現在の状態スナップショットを高頻度で発行します。`FAILURE` が出た瞬間のノード名を見ることで、問題がナビゲーションの「どのフェーズ」で発生しているかを素早く特定できます。これは実際のシステムデバッグで非常に役立つスキルです。
