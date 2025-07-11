import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    drone_config = os.path.join(get_package_share_directory('drone_energy_estimation'), 'config', 'drone_params.yaml')

#     drones = [
#           {'ns': 'drone1', 'ip': '192.168.50.103', 'command_port':38065, 'data_port':8890, 'video_port':11111, 'x': 0, 'y': 0, 'z': 1, 'urdf': 1},
#           {'ns': 'drone2', 'ip': '192.168.50.33', 'command_port':38065, 'data_port':8890, 'video_port':11111,  'x': 1, 'y': 0, 'z': 1, 'urdf': 2},
#           # Ajouter plus de drones ici si besoin
#      ]
    return LaunchDescription([

        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

     #    # Spawn tello.urdf
     #    Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
     #         arguments=[urdf_path, '0', '0', '1', '0']),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen'),

        # Joystick controller, generates /namespace/cmd_vel messages
        Node(package='tello_driver', executable='tello_joy_main', output='screen'), 

        Node(package='tello_driver', executable='tello_driver_main', output='screen',
          #     parameters=[{
          #       'drone_ip': '192.168.50.33',
          #       'drone_port': 8889,
          #       'command_port': 38065,
          #       'data_port': 8890,
          #       'video_port': 11111
          #   }] 
          #     parameters=[{
          #       'drone_ip': drones[1]["ip"],
          #       'drone_port': 8889,
          #       'command_port': drones[1]["command_port"],
          #       'data_port': drones[1]["data_port"],
          #       'video_port': drones[1]["video_port"]
          #   }] 
            ), #rajouté

        Node(package='tello_position', executable='tello_position_cal', output='screen'), #rajouté

        Node(package='drone_energy_estimation', executable='battery_state', name='battery_state',
             output='screen', parameters=[drone_config]),

        # Spawn tello_1.urdf
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-file', urdf_path, '-entity', 'tello_drone']),
    ])

