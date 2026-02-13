from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'mybot_2wd'
    
    # Launch arguments
    target_tag_id_arg = DeclareLaunchArgument(
        'target_tag_id',
        default_value='0',
        description='ID of the AprilTag to dock with'
    )
    
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='0.5',
        description='Target distance from wall in meters'
    )
    
    tag_height_arg = DeclareLaunchArgument(
        'tag_height',
        default_value='0.75',
        description='Height of tag on wall (0.5-1.0 meters)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # AprilTag detection node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[{
            'image_transport': 'raw',
            'family': '36h11',
            'size': 0.16,  # Tag size in meters
            'camera_frame': 'camera_link_optical'
        }],
        remappings=[
            ('image_rect', '/camera/camera/image_raw'),
            ('camera_info', '/camera/camera/camera_info')
        ]
    )
    
        # Autonomous docking node - use full path to executable
    from ament_index_python.packages import get_package_prefix
    package_prefix = get_package_prefix(package_name)
    docking_script = os.path.join(package_prefix, 'lib', package_name, 'autonomous_docking_node.py')
    
    docking_node = Node(
        package=package_name,
        executable='autonomous_docking_node.py',  # Include .py extension
        name='autonomous_docking_node',
        output='screen',
        parameters=[{
            'target_tag_id': LaunchConfiguration('target_tag_id'),
            'target_distance': LaunchConfiguration('target_distance'),
            'tag_height': LaunchConfiguration('tag_height'),
            'max_linear_vel': 0.2,
            'max_angular_vel': 0.5,
            'kp_linear': 0.8,
            'kd_linear': 0.1,
            'kp_angular': 1.2,
            'kd_angular': 0.15,
            'approach_threshold': 1.5,
            'docking_tolerance': 0.05,
            'angle_tolerance': 0.1,
            'min_detection_distance': 0.3,
            'max_detection_distance': 5.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        target_tag_id_arg,
        target_distance_arg,
        tag_height_arg,
        use_sim_time_arg,
        apriltag_node,
        docking_node
    ])