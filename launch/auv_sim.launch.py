```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # PID controller (placeholder pkg/exe names)
        Node(
            package="auv_control",
            executable="pid_node",
            name="pid_controller",
            output="screen",
            parameters=["config/pid.yaml"]
        ),
        # IMU fusion (EKF)
        Node(
            package="auv_fusion",
            executable="ekf_node",
            name="ekf_fusion",
            output="screen",
            parameters=["config/ekf.yaml"]
        ),
        # Perception (camera–LiDAR fusion)
        Node(
            package="auv_perception",
            executable="obstacle_node",
            name="obstacle_avoidance",
            output="screen",
            parameters=["config/perception.yaml"]
        ),
    ])
