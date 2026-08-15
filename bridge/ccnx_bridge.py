#!/usr/bin/env python3
"""ccnx_bridge — ROS2 トピック ⇄ CCNx(RFC8609) ブリッジ。

役割:
  1. /sim/events(JSON) と /odom を購読し、障害物イベントを「言語化」して最新アラートを保持
  2. UDP フェイス（既定 :9301）で RFC8609 Interest を待ち受け、
     /sim/robot/0/alert への Interest に最新アラートを ContentObject で応答する

cefnetd に `cefroute add ccnx:/sim udp 127.0.0.1:9301` を登録しておくと、
任意の CCNx コンシューマ → cefnetd → 本ブリッジ → cefnetd → コンシューマ の
経路で、実 RFC8609 ワイヤのままアラートを取得できる。

エンコード/デコードは同リポジトリ群の C 実装 `codec_test`（RFC8609 準拠・
独立2実装で機械検証済み）を子プロセスとして用いる。ワイヤに独自形式を発明しない。

言語化は決定論テンプレート（describe()）。ローカル VLM/LLM（OpenAI 互換 API）に
差し替える場合は describe() を置き換えるだけでよい（インターフェース固定）。
"""
import json
import math
import os
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

CODEC = os.environ.get(
    "CCNX_CODEC",
    os.path.expanduser("~/codes/B-283-ccnx-swarm/client/codec_test"))
LISTEN_PORT = int(os.environ.get("BRIDGE_PORT", "9301"))
ALERT_NAME = "/sim/robot/0/alert"


def codec_encode_co(name: str, payload: bytes) -> bytes:
    out = subprocess.run([CODEC, "encode", name, "C", payload.hex()],
                         capture_output=True, text=True, check=True)
    return bytes.fromhex(out.stdout.strip().split()[-1])


def codec_decode(frame: bytes):
    out = subprocess.run([CODEC, "decode", frame.hex()],
                         capture_output=True, text=True, check=True)
    kind, name = out.stdout.strip().split("\t")[:2]
    return kind, name


def describe(evt: dict) -> str:
    """障害物イベントの決定論的な言語化（VLM/LLM 差し替え点）。"""
    size = evt.get("size_est_m", 0)
    rng = evt.get("range_m", 0)
    return (f"進行方向 {rng:.1f}m 先に未知の障害物"
            f"（推定幅 {size:.1f}m）を検知し、左側へ迂回します。")


class CcnxBridge(Node):
    def __init__(self):
        super().__init__("ccnx_bridge")
        self.latest = {"status": "patrol", "desc": "巡回中。異常なし。"}
        self.pose = (0.0, 0.0, 0.0)
        self.create_subscription(String, "/sim/events", self.on_event, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", LISTEN_PORT))
        threading.Thread(target=self.serve, daemon=True).start()
        self.get_logger().info(
            f"ccnx_bridge: serving {ALERT_NAME} on udp:{LISTEN_PORT} (codec={CODEC})")

    def on_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.degrees(2 * math.atan2(q.z, q.w)))

    def on_event(self, m: String):
        evt = json.loads(m.data)
        x, y, hd = self.pose
        alert = {
            "time": time.strftime("%H:%M:%S"),
            "event": evt["event"],
            "desc": describe(evt) if evt["event"] == "obstacle_detected"
                    else evt["event"],
            "pos": [round(x, 1), round(y, 1)],
            "heading_deg": round(hd, 0),
        }
        self.latest = alert
        self.get_logger().info(f"alert updated: {alert['desc']}")

    def serve(self):
        while True:
            frame, addr = self.sock.recvfrom(4096)
            try:
                kind, name = codec_decode(frame)
            except subprocess.CalledProcessError:
                continue                      # RFC8609 として解釈できないものは黙って捨てる
            if kind != "Interest" or not name.startswith(ALERT_NAME):
                continue
            payload = json.dumps(self.latest, ensure_ascii=False).encode()
            self.sock.sendto(codec_encode_co(name, payload), addr)


def main():
    rclpy.init()
    node = CcnxBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
