#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu


class SimpleYawFusion(Node):
    """Complementary filter: yaw = alpha*(yaw + gyro*dt) + (1-alpha)*mag_yaw"""
    def __init__(self) -> None:
        super().__init__("yaw_fusion")

        self.declare_parameter("alpha", 0.98)   # gyro trust
        self.declare_parameter("rate_hz", 100.0)

        self.alpha = float(self.get_parameter("alpha").value)
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)

        self.yaw_est: float = 0.0
        self.gyro_z: float = 0.0
        self.mag_yaw: Optional[float] = None

        self.sub_imu = self.create_subscription(Imu, "/imu/data", self.on_imu, 30)
        self.sub_mag = self.create_subscription(Float32, "/mag/yaw", self.on_mag, 10)
        self.pub_yaw = self.create_publisher(Float32, "/yaw_filtered", 10)

        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Yaw complementary filter node started.")

    def on_imu(self, msg: Imu) -> None:
        self.gyro_z = float(msg.angular_velocity.z)

    def on_mag(self, msg: Float32) -> None:
        # keep wrapped [-pi, pi]
        self.mag_yaw = (float(msg.data) + math.pi) % (2 * math.pi) - math.pi

    def update(self) -> None:
        # integrate gyro
        yaw_pred = self.yaw_est + self.gyro_z * self.dt
        # wrap
        yaw_pred = (yaw_pred + math.pi) % (2 * math.pi) - math.pi

        if self.mag_yaw is not None:
            # complementary blend
            # compute smallest-angle difference
            err = (self.mag_yaw - yaw_pred + math.pi) % (2 * math.pi) - math.pi
            yaw = yaw_pred + (1.0 - self.alpha) * err
        else:
            yaw = yaw_pred

        # publish
        self.yaw_est = (yaw + math.pi) % (2 * math.pi) - math.pi
        self.pub_yaw.publish(Float32(data=self.yaw_est))


def main():
    rclpy.init()
    node = SimpleYawFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
