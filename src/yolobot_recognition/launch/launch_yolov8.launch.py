from launch import LaunchDescription
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='yolobot_recognition', 
            executable='yolov8_ros2_pt.py', 
            # parameters=[{'system_id': 2}],
            output='screen',
        ),
    ])