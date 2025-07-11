import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()


    ld.add_action(DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='ROS image topic to subscribe to'
    ))
    ld.add_action(DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='ROS camera_info topic (if your camera publishes it)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'output_folder',
        default_value=os.path.join(os.path.expanduser('~'), 'images'),
        description='Absolute path of folder into which images will be saved'
    ))
    ld.add_action(DeclareLaunchArgument(
        'filename_pattern',
        default_value='frame%04i.png',
        description='Pattern for filenames; %04i will be replaced by frame index'
    ))
    ld.add_action(DeclareLaunchArgument(
        'sec_per_frame',
        default_value='0.0',
        description='Seconds per frame. 0.0 means save every incoming frame.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'save_all_image',
        default_value='true',
        description='If true, saves continuously; if false, waits for /save service call.'
    ))
    ld.add_action(DeclareLaunchArgument(
        'launch_cam_demo',
        default_value='false',
        description='If "true", also launch a demo cam2image publisher for testing'
    ))

    # demo_cam = Node(
    #     package='image_tools',
    #     executable='cam2image',
    #     name='cam2image_demo',
    #     remappings=[('image', LaunchConfiguration('camera_topic'))],
    #     parameters=[{}],
    #     condition=LaunchConfiguration('launch_cam_demo')
    # )
    # ld.add_action(demo_cam)

    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')

    
    filename_pattern = [
        LaunchConfiguration('output_folder'),
        TextSubstitution(text=f'/cam_{timestamp_str}_frame%04i.png')
    ]
    
    saver = Node(
        package='image_view',
        executable='image_saver',
        name='image_saver',
        output='screen',
        remappings=[
            ('/image', LaunchConfiguration('camera_topic')),
            # ('/camera_info', LaunchConfiguration('camera_info_topic')),
        ],
        parameters=[
            {
                
                'save_all_image': LaunchConfiguration('save_all_image'),
                'sec_per_frame': LaunchConfiguration('sec_per_frame'),
                'filename_format': filename_pattern
            
            }
        ]
    )
    ld.add_action(saver)

    return ld
