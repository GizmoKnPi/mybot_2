#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for MPU6050'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='921600',
        description='Baud rate for serial communication'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='TF frame ID for IMU'
    )
    
    # IMU Serial Node
    imu_serial_node = Node(
        package='imu_serial',
        executable='imu_serial_node',
        name='imu_serial_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'topic': '/imu_raw',
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        arguments=[
            '--port', LaunchConfiguration('port'),
            '--baud', LaunchConfiguration('baud'),
            '--frame', LaunchConfiguration('frame_id')
        ]
    )
    
    # IMU Filter Madgwick Node
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'use_dt_from_msg_header': False,
            'fixed_frame': 'odom',
            'world_frame': 'enu',
            'publish_tf': True,
            'reverse_tf': False,
            'constant_dt': 0.01,
            'gain': 0.1,
            'zeta': 0.01,
        }],
        remappings=[
            ('/imu/data_raw', '/imu_raw'),
            ('/imu/data', '/imu/data_filtered'),
            ('/imu/mag', '/imu/mag')
        ]
    )
    
    # Optional: Static TF publisher (imu_link to base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', LaunchConfiguration('frame_id')]
    )
    
    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_id_arg,
        imu_serial_node,
        imu_filter_node,
        static_tf,
    ])