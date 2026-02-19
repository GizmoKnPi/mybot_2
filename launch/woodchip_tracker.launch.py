from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mybot_2wd',
            executable='woodchip_tracker.py',
            name='woodchip_tracker_node',
            output='screen'
        )
    ])