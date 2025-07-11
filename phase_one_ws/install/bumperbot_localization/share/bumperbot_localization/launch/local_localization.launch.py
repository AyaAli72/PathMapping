from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('bumperbot_localization')
    
    # Arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Bumperbot Localization Node (replaces AMCL from Waffle)
    bumperbot_localizer = Node(
        package='bumperbot_localization',
        executable='bumperbot_localization.py',
        name='bumperbot_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Robot Localization EKF (similar to Waffle setup)
    ekf_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'ekf.yaml'
    ])

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', 'odom'),
            ('/set_pose', '/initialpose')  # Waffle-compatible topic
        ]
    )

    # TF Static Transform Publisher (if needed)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    return LaunchDescription([
        use_sim_time,
        bumperbot_localizer,
        robot_localization,
        static_tf
    ])
