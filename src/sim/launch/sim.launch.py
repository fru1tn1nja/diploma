from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_share = os.path.join(
        os.environ['AMENT_PREFIX_PATH'].split(':')[0], 'share')
    vrx_pkg = os.path.join(pkg_share, 'vrx_gz')
    world_path = os.path.join(vrx_pkg, 'worlds', 'ocean_waves.world')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(vrx_pkg, 'launch', 'spawn_wamv.launch.py')]
            ),
            launch_arguments={'world': world_path}.items()
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['thruster_controller', '-c', '/controller_manager'],
            output='screen'
        ),
    ])
