from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_control",
                executable="robo_direction_node",
            ),
            Node(
                package="server",
                executable="slam_node",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
            ),
            Node(package="rviz2", executable="rviz2"),
        ]
    )
