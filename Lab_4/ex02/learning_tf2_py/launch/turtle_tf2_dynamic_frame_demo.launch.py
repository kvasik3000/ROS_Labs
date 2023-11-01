import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    radius_arg = DeclareLaunchArgument('radius', default_value='10.0', description='Radius of rotation')
    direction_arg = DeclareLaunchArgument('direction_of_rotation', default_value='1', description='Direction of rotation (1 - clockwise, -1 - counterclockwise)')

    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_py'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
            launch_arguments={'target_frame': 'carrot1'}.items(),
        )

    return LaunchDescription([
        radius_arg,
        direction_arg,
        demo_nodes,
        Node(
            package='learning_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
        ),
    ])
