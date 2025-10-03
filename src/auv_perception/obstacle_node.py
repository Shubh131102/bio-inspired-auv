#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoid(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_avoid")
        self.declare_parameter("min_distance_m", 0.8)
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.6)
        self.declare_parameter("front_sector_deg", 60.0)  # +/- degrees around front

        self.min_d = float(self.get_parameter("min_distance_m").value)
        self.vx = float(self.get_parameter("forward_speed").value)
        self.wz = float(self.get_parameter("turn_speed").value)
        self.sector = math.radians(float(self.get_parameter("front_sector_deg").value))

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

        self.get_logger().info("Obstacle avoidance node started.")

    def on_scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        # convert front sector indices
        # front = angle 0. We take [-sector, +sector].
        indices: List[int] = []
        for i, r in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -self.sector <= angle <= self.sector:
                indices.append(i)

        front = [ranges[i] for i in indices if not math.isinf(ranges[i]) and not math.isnan(ranges[i])]
        min_front = min(front) if front else float("inf")

        cmd = Twist()
        if min_front < self.min_d:
            # turn away (choose direction by comparing left vs right halves)
            left = [ranges[i] for i in indices if (msg.angle_min + i * msg.angle_increment) > 0.0 and not math.isinf(ranges[i]) and not math.isnan(ranges[i])]
            right = [ranges[i] for i in indices if (msg.angle_min + i * msg.angle_increment) < 0.0 and not math.isinf(ranges[i]) and not math.isnan(ranges[i])]
            sum_left = sum(left) / len(left) if left else 0.0
            sum_right = sum(right) / len(right) if right else 0.0
            # turn toward the more open side
            cmd.angular.z = self.wz if sum_left > sum_right else -self.wz
            cmd.linear.x = 0.0
        else:
            # go forward
            cmd.linear.x = self.vx
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = ObstacleAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
