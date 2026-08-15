#!/usr/bin/env python3
"""robot_sim — ROS2 Humble 自律移動ロボット出力シミュレータ。

実機ロボット（差動二輪・LiDAR・IMU・オドメトリ搭載を想定）が廊下を巡回し、
途中で「事前マップに無い未知の障害物」に遭遇して迂回する、という一連の出力を
ROS2 標準メッセージ型で配信する。

配信トピック:
  /scan        sensor_msgs/LaserScan   2D LiDAR（360 本・10Hz）
  /odom        nav_msgs/Odometry       オドメトリ（10Hz）
  /imu         sensor_msgs/Imu         方位・角速度（10Hz）
  /sim/events  std_msgs/String         シナリオイベント(JSON)。障害物検知・迂回開始・迂回完了

障害物はシミュレーション開始 OBSTACLE_AT 秒後に進行方向に出現する。
ロボットは LiDAR 前方扇の最近傍距離が閾値を切ると検知イベントを発行し、
左に回り込む迂回経路をとり、通過後に復帰イベントを発行する。
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

RATE_HZ = 10.0
N_BEAMS = 360
RANGE_MAX = 12.0
CORRIDOR_HALF = 2.0        # 廊下半幅 [m]
SPEED = 0.6                # 巡回速度 [m/s]
OBSTACLE_AT = 12.0         # 開始からの出現時刻 [s]
OBSTACLE_SIZE = 0.6        # 障害物半径 [m]
DETECT_DIST = 2.5          # 検知距離 [m]


class RobotSim(Node):
    def __init__(self):
        super().__init__("robot_sim")
        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_evt = self.create_publisher(String, "/sim/events", 10)
        self.t0 = time.monotonic()
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.state = "PATROL"          # PATROL → DETOUR → PATROL
        self.obstacle = None           # (ox, oy) 出現後にセット
        self.detour_until = 0.0
        self.timer = self.create_timer(1.0 / RATE_HZ, self.step)
        self.get_logger().info("robot_sim start (corridor patrol)")

    # ---- シナリオ進行 -------------------------------------------------
    def step(self):
        t = time.monotonic() - self.t0
        if self.obstacle is None and t >= OBSTACLE_AT:
            # 進行方向 4m 先に未知障害物が出現
            self.obstacle = (self.x + 4.0 * math.cos(self.yaw),
                             self.y + 4.0 * math.sin(self.yaw))
            self.emit("obstacle_appeared", {"pos": [round(v, 2) for v in self.obstacle]})

        front = self.front_min_range()
        if self.state == "PATROL":
            if front < DETECT_DIST:
                self.state = "DETOUR"
                self.detour_until = t + 4.0
                self.emit("obstacle_detected", {
                    "range_m": round(front, 2),
                    "bearing_deg": 0,
                    "size_est_m": OBSTACLE_SIZE * 2,
                    "robot": [round(self.x, 2), round(self.y, 2)],
                    "heading_deg": round(math.degrees(self.yaw), 1),
                })
            self.yaw += 0.0
        elif self.state == "DETOUR":
            self.yaw = 0.5 * math.sin((self.detour_until - t) * 1.2)  # 左に膨らんで戻る
            if t >= self.detour_until:
                self.state = "PATROL"
                self.yaw = 0.0
                self.emit("detour_completed", {
                    "robot": [round(self.x, 2), round(self.y, 2)]})
                # デモを回し続けるため、障害物は一定時間後に別の場所で再出現する
                self.obstacle = None
                self.t0 = time.monotonic()   # 次の出現タイマーをリセット

        self.x += SPEED / RATE_HZ * math.cos(self.yaw)
        self.y += SPEED / RATE_HZ * math.sin(self.yaw)
        now = self.get_clock().now().to_msg()
        self.publish_scan(now)
        self.publish_odom(now)
        self.publish_imu(now)

    # ---- センサ模擬 ---------------------------------------------------
    def front_min_range(self):
        if self.obstacle is None:
            return RANGE_MAX
        dx, dy = self.obstacle[0] - self.x, self.obstacle[1] - self.y
        d = math.hypot(dx, dy) - OBSTACLE_SIZE
        rel = abs((math.atan2(dy, dx) - self.yaw + math.pi) % (2 * math.pi) - math.pi)
        return max(0.1, d) if rel < math.radians(25) else RANGE_MAX

    def publish_scan(self, stamp):
        m = LaserScan()
        m.header.stamp = stamp
        m.header.frame_id = "base_scan"
        m.angle_min, m.angle_max = -math.pi, math.pi
        m.angle_increment = 2 * math.pi / N_BEAMS
        m.range_min, m.range_max = 0.1, RANGE_MAX
        ranges = []
        for i in range(N_BEAMS):
            a = m.angle_min + i * m.angle_increment          # ロボ座標系
            wall = CORRIDOR_HALF / max(abs(math.sin(a + self.yaw)), 1e-3)
            r = min(wall, RANGE_MAX)
            if self.obstacle is not None:
                dx, dy = self.obstacle[0] - self.x, self.obstacle[1] - self.y
                rel = (math.atan2(dy, dx) - self.yaw - a + math.pi) % (2 * math.pi) - math.pi
                if abs(rel) < math.radians(8):
                    r = min(r, max(0.1, math.hypot(dx, dy) - OBSTACLE_SIZE))
            ranges.append(r)
        m.ranges = ranges
        self.pub_scan.publish(m)

    def publish_odom(self, stamp):
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z = math.sin(self.yaw / 2)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2)
        m.twist.twist.linear.x = SPEED
        self.pub_odom.publish(m)

    def publish_imu(self, stamp):
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = "imu_link"
        m.orientation.z = math.sin(self.yaw / 2)
        m.orientation.w = math.cos(self.yaw / 2)
        self.pub_imu.publish(m)

    def emit(self, kind, data):
        payload = {"event": kind, "t": round(time.monotonic() - self.t0, 1), **data}
        self.pub_evt.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(f"event: {payload}")


def main():
    rclpy.init()
    node = RobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
