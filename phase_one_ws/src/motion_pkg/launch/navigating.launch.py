from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    random_points_generator_node = Node(
        package="motion_pkg",
        executable="random_points_generator",
    )

    points_sender_node = Node(
        package="motion_pkg",
        executable="points_sender",
    )

    speed_control_node = Node(
        package="motion_pkg",
        executable="speed_control",
    )


    return LaunchDescription(
        [
            random_points_generator_node,
            points_sender_node,
            speed_control_node
        ]
    )
