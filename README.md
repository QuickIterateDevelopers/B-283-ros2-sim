# B-283 ROS2 × CCNx シミュレータ

**ROS2 Humble の自律移動ロボット（模擬）が、未知の障害物を検知・迂回し、その状況を
自然言語アラートとして CCNx（RFC8569/8609）経由で監視卓へ通知する**縦一本のデモ。

CCNx 側は発明せず、実物を使う:

- フォワーダ = **Cefore (cefnetd) v0.12.0**（公開リポジトリ https://github.com/cefore/cefore をビルド）
- ワイヤの encode/decode = 姉妹リポジトリ
  [B-283-ccnx-swarm](https://github.com/QuickIterateDevelopers/B-283-ccnx-swarm) の
  C 実装 `codec_test`（RFC8609 準拠・独立2実装で機械検証済み）

```
┌─────────────────────  ROS2 Humble  ─────────────────────┐
│ sim/robot_sim.py                bridge/ccnx_bridge.py   │
│  廊下巡回・障害物遭遇・迂回      /sim/events,/odom 購読   │
│  /scan /odom /imu /sim/events →  言語化 → 最新アラート保持 │
└──────────────────────────────────┬──────────────────────┘
                     RFC8609 UDP   │ /sim/robot/0/alert に CO 応答
                                   ▼
                       ┌───────────────────┐
       Interest ──────▶│ cefnetd v0.12.0   │◀────── monitor/alert_monitor.py
       ContentObject ◀─│ FIB: /sim → :9301 │        （模擬コントローラ・ROS2非依存）
                       └───────────────────┘
```

## 特徴

- **ロボット出力は ROS2 標準メッセージ型**（sensor_msgs/LaserScan・nav_msgs/Odometry・
  sensor_msgs/Imu）。実機（LiDAR・深度カメラ・IMU 搭載の差動二輪クラス）と同じ形
- **通知の全経路が実 CCNx ワイヤ**。監視卓は cefnetd に Interest を送り、ブリッジが
  ContentObject で応える。独自プロトコルの発明なし（Wireshark で `udp port 9695` を
  見れば RFC8609 バイトが流れている）
- **言語化は決定論テンプレート**（`bridge/ccnx_bridge.py` の `describe()`）。
  ローカル VLM/LLM（OpenAI 互換 API）への差し替え点をこの1関数に固定してある
- ROS2 は **RoboStack（conda-forge）で root 権限なしに導入**できる（下記手順）

## セットアップ

### 1) ROS2 Humble（root 不要・RoboStack）

```bash
curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj bin/micromamba
export MAMBA_ROOT_PREFIX=$HOME/micromamba
./bin/micromamba create -n ros2 -c conda-forge -c robostack-staging ros-humble-ros-base -y
```

### 2) cefnetd（Cefore v0.12.0）

```bash
git clone --branch v0.12.0 https://github.com/cefore/cefore.git && cd cefore
./configure --prefix=$HOME/cefore --enable-cache && make && make install
# 鍵パスを $HOME 配下に向けて cefnetd を起動（詳細は cefnetd.conf）
cefnetd &
```

### 3) 自作 CCNx codec（姉妹リポジトリ）

```bash
git clone https://github.com/QuickIterateDevelopers/B-283-ccnx-swarm.git
( cd B-283-ccnx-swarm && make all )   # → client/codec_test
```

## 実行

```bash
# ターミナル1: ロボット（ROS2）
micromamba run -n ros2 python sim/robot_sim.py
# ターミナル2: ブリッジ（ROS2）
micromamba run -n ros2 python bridge/ccnx_bridge.py
# cefnetd に経路登録（1回だけ）
cefroute add ccnx:/sim udp 127.0.0.1:9301
# ターミナル3: 監視卓（ROS2 非依存・CCNx のみ）
python3 monitor/alert_monitor.py
```

監視卓の表示例:

```
[14:17:53] 進行方向 2.5m 先に未知の障害物（推定幅 1.2m）を検知し、左側へ迂回します。 pos=[70.8, 1.4] heading=0.0°
[14:17:57] detour_completed pos=[73.1, 1.7] heading=0.0°
```

障害物はデモ用に周期的に再出現する（`sim/robot_sim.py` の `OBSTACLE_AT`）。

## 検証済み環境

| OS | ROS2 | cefnetd | 結果 |
|---|---|---|---|
| Ubuntu 24.04 (x86_64) | Humble (RoboStack) | v0.12.0 (--enable-cache) | 全経路で動作（上記表示例は実ログ） |

## ライセンス

MIT
