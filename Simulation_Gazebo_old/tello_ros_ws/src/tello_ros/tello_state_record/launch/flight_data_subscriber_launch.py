import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', output='screen'),
        Node(package='tello_driver', executable='tello_joy_main', output='screen'),
        Node(package='tello_driver', executable='tello_driver_main', output='screen'),
        Node(package='tello_state_record', executable='flight_data_subscriber', name='flight_data_subscriber', output='screen',
        )
    ])