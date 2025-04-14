import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_name = 'aruco_tracker'

    sim_mode = LaunchConfiguration('sim_mode')
    sim_mode_dec = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Run in simulation (true) or on real robot (false)'
    )

    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg_name), 'launch', 'aruco_tracker.launch.py')
        ]),
        launch_arguments={
            'image_topic': '/camera/image_raw',
            # 'cmd_vel_topic': '/cmd_vel_tracker',
            'cmd_vel_topic': '/cmd_vel',
            'use_sim_time': sim_mode
        }.items()
    )

    return LaunchDescription([
        sim_mode_dec,
        tracker_launch,
    ])