#!/usr/bin/env python3
"""alert_monitor — 模擬コントローラ（監視卓）。

cefnetd に RFC8609 Interest を送り、ロボットの最新アラートを ContentObject で
受け取ってコンソール表示する。ROS2 には依存しない（通信は CCNx のみ）。

経路:  monitor ──Interest──▶ cefnetd ──FIB(/sim)──▶ ccnx_bridge(ROS2)
              ◀──ContentObject(最新アラートJSON)────┘
"""
import json
import os
import socket
import subprocess
import sys
import time

CODEC = os.environ.get(
    "CCNX_CODEC",
    os.path.expanduser("~/codes/B-283-ccnx-swarm/client/codec_test"))
CEFNETD = ("127.0.0.1", int(os.environ.get("CEFNETD_PORT", "9695")))
NAME = "/sim/robot/0/alert"


def fetch(timeout=2.0):
    enc = subprocess.run([CODEC, "encode", NAME, "I"],
                         capture_output=True, text=True, check=True)
    interest = bytes.fromhex(enc.stdout.strip().split()[-1])
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    s.sendto(interest, CEFNETD)
    frame, _ = s.recvfrom(4096)
    dec = subprocess.run([CODEC, "decode", frame.hex()],
                         capture_output=True, text=True, check=True)
    kind, name, payload_hex = dec.stdout.strip().split("\t")[:3]
    assert kind == "ContentObject" and name == NAME, (kind, name)
    return json.loads(bytes.fromhex(payload_hex))


def main():
    interval = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    print(f"alert_monitor: polling {NAME} via cefnetd {CEFNETD[0]}:{CEFNETD[1]}")
    last = None
    while True:
        try:
            alert = fetch()
            if alert != last:
                print(f"[{alert.get('time','--')}] {alert.get('desc')}  "
                      f"pos={alert.get('pos')} heading={alert.get('heading_deg')}°")
                last = alert
        except (socket.timeout, subprocess.CalledProcessError) as e:
            print(f"(no response: {e.__class__.__name__})")
        time.sleep(interval)


if __name__ == "__main__":
    main()
