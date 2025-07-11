import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg = get_package_share_directory('fame_simulation')

    nav_pkg = get_package_share_directory('nav2_bringup')

    slam_pkg = get_package_share_directory('slam_toolbox')

    tractor_ns_1 = 'REX'
    tractor_ns_2 = 'DINGO'

    drone_ns_1 = 'DRONE'

    tractor_1 = os.path.join(pkg, 'robot_description',
                             str('tractor_1.sdf'))

    tractor_2 = os.path.join(pkg, 'robot_description',
                             str('tractor_2.sdf'))

    drone_1 = os.path.join(pkg, 'robot_description',
                             str('Drone.sdf'))# "Drone_1" doesn't work for some reasons

    sc = os.path.join(pkg, 'robot_description',                      str('smart_car.sdf'))

    # world_path = os.path.join(pkg, 'worlds', 'empty.world')
    world_path = os.path.join(pkg, 'worlds', 'simple.world')

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

        Node(package='fame_simulation', executable='spawn_elements.py', output='screen',
             arguments=[tractor_1, '1', '2.2', '0', '-1.57', tractor_ns_1]),

        Node(package='fame_simulation', executable='spawn_elements.py', output='screen',
             arguments=[tractor_2, '0', '0', '0', '0', tractor_ns_2]),
             
        Node(package='fame_simulation', executable='spawn_elements.py', output='screen',
             arguments=[drone_1, '2', '0', '0', '0', drone_ns_1]),



    ])
