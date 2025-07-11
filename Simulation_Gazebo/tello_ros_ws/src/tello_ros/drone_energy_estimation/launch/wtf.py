import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ns = 'tello_1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    urdf_path_1 = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    return LaunchDescription([
        # Launch Gazebo, loading simple.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path_1]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen',
             namespace=ns),

        # Joystick controller, generates /namespace/cmd_vel messages
        Node(package='tello_driver', executable='tello_joy_main', output='screen'),

        Node(package='tello_driver', executable='tello_driver_main', output='screen'),

        Node(package='tello_position', executable='tello_position_cal', output='screen'),

        # Spawn tello_1.urdf
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-file', urdf_path_1, '-entity', 'tello_drone']),

    ])
