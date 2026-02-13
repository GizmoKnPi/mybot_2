from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[{
                'image_transport': 'raw',
                'family': '36h11',
                'size': 0.16,   # tag size in meters (VERY IMPORTANT)
                'camera_frame': 'camera_link_optical'
            }],
            remappings=[
                ('image_rect', '/camera/camera/image_raw'),
                ('camera_info', '/camera/camera/camera_info')
            ]
        )
    ])
