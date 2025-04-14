from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    params_file_dec = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('aruco_tracker'),
            'config',
            'aruco_tracker_params_sim.yaml'
        ),
        description='Full path to params file for aruco_tracker nodes.'
    )

    detect_only = LaunchConfiguration('detect_only')
    detect_only_dec = DeclareLaunchArgument(
        'detect_only',
        default_value='false',
        description='Run only the detection node, useful for testing ArUco detection.'
    )

    follow_only = LaunchConfiguration('follow_only')
    follow_only_dec = DeclareLaunchArgument(
        'follow_only',
        default_value='false',
        description='Run only the follow node, useful for testing with manual position data.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Enable sim time for nodes.'
    )

    image_topic = LaunchConfiguration('image_topic')
    image_topic_dec = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='The name of the input image topic.'
    )

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_topic_dec = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='The name of the output command vel topic.'
    )

    detect_node = Node(
        package='aruco_tracker',
        executable='detect_aruco',
        name='detect_aruco',
        parameters=[params_file],
        remappings=[('/image_in', image_topic)],
        condition=UnlessCondition(follow_only),
        output='screen'
    )

    follow_node = Node(
        package='aruco_tracker',
        executable='follow_aruco',
        name='follow_aruco',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', cmd_vel_topic)],
        condition=UnlessCondition(detect_only),
        output='screen'
    )

    return LaunchDescription([
        params_file_dec,
        detect_only_dec,
        follow_only_dec,
        use_sim_time_dec,
        image_topic_dec,
        cmd_vel_topic_dec,
        detect_node,
        follow_node,
    ])