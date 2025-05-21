from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(os.getcwd(), 'world.world')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),
    ])