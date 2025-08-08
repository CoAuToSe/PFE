# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          choices=[
                              'construction',
                              'office',
                              'orchard',
                              'pipeline',
                              'solar_farm',
                              'warehouse',
                          ],
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot1_config_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/robot1.yaml'],
                          description='Path to robot1.yaml config'),

    DeclareLaunchArgument('robot2_config_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/robot2.yaml'],
                          description='Path to robot2.yaml config'),
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.3',
                 description='z component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_clearpath_gz = get_package_share_directory(
        'clearpath_gz')
    pkg_clearpath_MRS = get_package_share_directory(
        'clearpath_MRS')

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_clearpath_MRS, 'launch', 'robot_spawn_MRS.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    # First robot
    robot_spawn_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments=[
            ('robot_config_path', LaunchConfiguration('robot1_config_path')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('world', LaunchConfiguration('world')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', '0.0'),
            ('y', '0.0'),
            ('z', LaunchConfiguration('z')),
            ('yaw', '0.0'),
        ]
    )

    # Second robot
    robot_spawn_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments=[
            ('robot_config_path', LaunchConfiguration('robot2_config_path')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('world', LaunchConfiguration('world')),
            ('rviz', 'false'),  # pour éviter un 2e RViz
            ('x', '2.0'),
            ('y', '0.0'),
            ('z', LaunchConfiguration('z')),
            ('yaw', '0.0'),
        ]
    )

    # Assemble launch description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim)
    ld.add_action(robot_spawn_1)
    ld.add_action(robot_spawn_2)
    return ld