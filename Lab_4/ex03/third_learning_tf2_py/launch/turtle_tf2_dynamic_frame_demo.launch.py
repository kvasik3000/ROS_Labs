import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch_ros.actions import Node


def generate_launch_description():
    
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('third_learning_tf2_py'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
        )


    return LaunchDescription([
        
        SetParameter(
            name='delay',
            value=3.0  
        ),
        
        demo_nodes,
        Node(
            package='third_learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='dynamic_broadcaster',
            
        ),
    ])
