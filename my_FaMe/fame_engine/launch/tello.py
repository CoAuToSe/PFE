
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    share_directory = get_package_share_directory('fame_engine')

    launch_args = [
        DeclareLaunchArgument('namespace', default_value='tello_7'),
        DeclareLaunchArgument('bpmn', default_value='tello_7')
    ]
    # revise path to your nodejs start file
    start_js_file = os.path.join(
        share_directory,
        'dist',
        'controller.js')

    start_rex_node = Node(
        name = 'engine_node',
        executable = 'node',
        output = 'screen',
        namespace = LaunchConfiguration('namespace'),
        parameters = [{
            'use_sim_time': use_sim_time,
        }],
        arguments = [
            start_js_file,
            '--bpmn', LaunchConfiguration('bpmn')
        ],
        cwd = share_directory
    )


    ld = LaunchDescription()
    for e in launch_args:
        ld.add_action(e)
    ld.add_action(start_rex_node)


    return ld
