from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('mybot_2wd')

    return LaunchDescription([

        #3D Lidar Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('my_bot'),
                        'launch',
                        'bringup.launch.py'
                    )
                )
            ),

        # Robot + Arduino + TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'launch_robot.launch.py')
            )
        ),

        # Lidar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'rplidar.launch.py')
            )
        ),

        # Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'camera.launch.py')
            )
        ),

        DeclareLaunchArgument(
            'map',
            default_value='/home/gizmoros2/ros2_ws/sitout.yaml',
            description='Full path to map yaml file'
            ),

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'map': LaunchConfiguration('map')
                }.items()
            ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'navigation_launch.py')
            )
        ),

        # Woodchip Tracker
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'woodchip_tracker.launch.py')
            )
        ),

        # Behavior Tree System Manager
        # Node(
        #     package='wheel_loader_manager',
        #     executable='system_manager',
        #     output='screen'
        # )
    ])