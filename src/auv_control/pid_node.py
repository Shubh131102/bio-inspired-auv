#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """Return yaw (rad) from quaternion (w,x,y,z)."""
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AUVPIDController(Node):
    def __init__(self) -> None:
        super().__init__("auv_pid_controller")

        # Parameters (can be overridden via YAML)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("depth_setpoint", 0.5)       # meters (+ down if using NED; adjust to your convention)
        self.declare_parameter("yaw_setpoint", 0.0)         # radians (0 = facing x+)
        self.declare_parameter("depth_kp", 1.0)
        self.declare_parameter("depth_ki", 0.05)
        self.declare_parameter("depth_kd", 0.1)
        self.declare_parameter("yaw_kp", 0.8)
        self.declare_parameter("yaw_ki", 0.02)
        self.declare_parameter("yaw_kd", 0.05)
        self.declare_parameter("anti_windup", True)
        self.declare_parameter("i_limit", 0.5)              # clamp for integrator

        self.rate_hz = float(self.get_parameter("rate_hz").value)

        # State
        self.depth: Optional[float] = None
        self.yaw: Optional[float] = None
        self.prev_depth_err = 0.0
        self.prev_yaw_err = 0.0
        self.int_depth = 0.0
        self.int_yaw = 0.0

        # I/O
        self.sub_depth = self.create_subscription(Float32, "/depth", self.on_depth, 10)
        self.sub_imu = self.create_subscription(Imu, "/imu/data", self.on_imu, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.update)

        self.get_logger().info("AUV PID controller node started.")

    def on_depth(self, msg: Float32) -> None:
        self.depth = float(msg.data)

    def on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self.yaw = quat_to_yaw(q.w, q.x, q.y, q.z)

    def update(self) -> None:
        # Skip until we have both signals
        if self.depth is None or self.yaw is None:
            return

        # Load params (allows dynamic changes)
        depth_sp = float(self.get_parameter("depth_setpoint").value)
        yaw_sp = float(self.get_parameter("yaw_setpoint").value)

        d_kp = float(self.get_parameter("depth_kp").value)
        d_ki = float(self.get_parameter("depth_ki").value)
        d_kd = float(self.get_parameter("depth_kd").value)
        y_kp = float(self.get_parameter("yaw_kp").value)
        y_ki = float(self.get_parameter("yaw_ki").value)
        y_kd = float(self.get_parameter("yaw_kd").value)
        anti_windup = bool(self.get_parameter("anti_windup").value)
        i_limit = float(self.get_parameter("i_limit").value)

        # Errors
        depth_err = depth_sp - self.depth
        # wrap yaw error to [-pi, pi]
        yaw_err = (yaw_sp - self.yaw + math.pi) % (2 * math.pi) - math.pi

        # Integrators
        self.int_depth += depth_err / self.rate_hz
        self.int_yaw += yaw_err / self.rate_hz
        if anti_windup:
            self.int_depth = max(min(self.int_depth, i_limit), -i_limit)
            self.int_yaw = max(min(self.int_yaw, i_limit), -i_limit)

        # Derivatives
        d_depth = (depth_err - self.prev_depth_err) * self.rate_hz
        d_yaw = (yaw_err - self.prev_yaw_err) * self.rate_hz

        # PID outputs
        heave = d_kp * depth_err + d_ki * self.int_depth + d_kd * d_depth       # -> linear.z
        yaw_rate = y_kp * yaw_err + y_ki * self.int_yaw + y_kd * d_yaw          # -> angular.z

        # Publish
        cmd = Twist()
        cmd.linear.z = heave
        cmd.angular.z = yaw_rate
        self.pub_cmd.publish(cmd)

        # Save prev
        self.prev_depth_err = depth_err
        self.prev_yaw_err = yaw_err

    

def main():
    rclpy.init()
    node = AUVPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
