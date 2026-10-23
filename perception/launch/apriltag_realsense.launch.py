from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',

            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],

            parameters=[
                '/opt/ros/jazzy/share/apriltag_ros/cfg/tags_36h11.yaml'
            ]
        )
    ])
