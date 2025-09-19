import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from datetime import datetime


def generate_launch_description():

    pkg_agriculture = get_package_share_directory('fame_agricultural')

    tractor_ns_1 = 'Tractor_1'
    tractor_ns_2 = 'Tractor_2'

    drone_ns = 'Drone'

    tractor_1 = os.path.join(pkg_agriculture, 'robot_description',
                             str(tractor_ns_1 + '.sdf'))

    tractor_2 = os.path.join(pkg_agriculture, 'robot_description',
                             str(tractor_ns_2 + '.sdf'))

    drone_1 = os.path.join(
        pkg_agriculture, 'robot_description', str(drone_ns + '.sdf'))

    world_path = os.path.join(pkg_agriculture, 'worlds', 'agriculture.world')

    return LaunchDescription([
        # Launch Gazebo, loading the world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Spawn robots.urdf
        Node(package='fame_agricultural', executable='spawn_elements.py', output='screen',
             arguments=[tractor_1, '0', '1', '0', '0', tractor_ns_1]),

        Node(package='fame_agricultural', executable='spawn_elements.py', output='screen',
             arguments=[tractor_2, '0', '2', '0', '0', tractor_ns_2]),

        Node(package='fame_agricultural', executable='spawn_elements.py', output='screen',
             arguments=[drone_1, '0', '5', '0', '0', drone_ns]),

    ])
