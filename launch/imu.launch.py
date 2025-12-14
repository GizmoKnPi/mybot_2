#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Serial port for MPU6050 (using by-id for consistency)'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='921600',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='TF frame ID for IMU'
        ),
        
        # IMU Serial Node
        Node(
            package='imu_serial',
            executable='imu_serial_node',
            name='imu_serial_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'frame_id': LaunchConfiguration('frame_id'),
                'topic': '/imu_raw'
            }]
        ),
        
        # IMU Filter Madgwick Node
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'use_dt_from_msg_header': False,
                'publish_tf': False,  # CHANGED: EKF will handle TF
                'fixed_frame': 'odom',
                'world_frame': 'enu',
                'constant_dt': 0.01,
                'gain': 0.1,
                'zeta': 0.01,
            }],
            remappings=[
                ('/imu/data_raw', '/imu_raw'),
                ('/imu/data', '/imu/data_filtered')
            ]
        ),
        
        # Static TF publisher
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_imu_tf',
        #     arguments=['--x', '0', '--y', '0', '--z', '0',
        #                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #                '--frame-id', 'base_link', '--child-frame-id', 
        #                LaunchConfiguration('frame_id')]
        # ),
    ])