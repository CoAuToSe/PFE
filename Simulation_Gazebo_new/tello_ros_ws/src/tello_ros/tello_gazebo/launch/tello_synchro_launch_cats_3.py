import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    namespaces = {
        'tello_7': [0,0,0, False],
        'tello_5': [1,1,1, False],
        'tello_2': [2,2,2, False],
    }
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    to_return = [
        # Launch Gazebo, loading simple.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),
    ]
    for namespace, data in namespaces.items():
        urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', namespace + '.urdf')
        to_return.extend([
            # Publish static transforms
            Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', namespace=namespace,
                arguments=[urdf_path]),
        ])
        if data[3]:
            to_return.extend([
                # Joystick driver, generates /namespace/joy messages
                Node(package='joy', executable='joy_node', output='screen', namespace=namespace),        
            
                # Joystick controller, generates /namespace/cmd_vel messages
                Node(package='tello_driver', executable='tello_joy_main', output='screen', namespace=namespace),        
            
                Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace=namespace,
                    #      parameters=[{
                    # 'drone_ip': '192.168.50.103',
                    # 'drone_port': 8889,
                    # 'command_port': 38065,
                    # 'data_port': 8890,
                    # 'video_port': 11111
                    # }] 
                ),        
            ])
            to_return.extend([
                Node(package='tello_position', executable='tello_position_cal_CATS', output='screen', namespace=namespace, 
                parameters=[{
                    'initial_x': data[0],
                    'initial_y': data[1],
                    'initial_z': data[2]
                }]),        
            ])
        to_return.extend([
            # Spawn tello_1.urdf
            Node(package='gazebo_ros', executable='spawn_entity.py', output='screen', namespace=namespace,
                arguments=[
                    '-file', urdf_path,
                    '-entity', namespace,
                    '-robot_namespace', namespace,
                    '-x', str(data[0]),
                    '-y', str(data[1]),
                    '-z', str(data[2])
                ]
            ),
        ])
    return LaunchDescription(to_return)
