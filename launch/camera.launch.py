import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                # Explicitly select Logitech C270
                'video_device': '/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_44725A50-video-index0',

                # Image properties
                'image_size': [640, 480],

                # Frame ID for TF
                'camera_info_frame_id': 'camera_link_optical',

                # Optional but recommended
                # 'frame_rate': 30.0
            }]
        )

    ])

