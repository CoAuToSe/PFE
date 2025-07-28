import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_drone_gazebo(namesp, ip, command_port, data_port, video_port, x, y, z, idx):
    suffix = '_' + str(idx)
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello' + suffix + '.urdf')
    drone_config = os.path.join(get_package_share_directory('drone_energy_estimation'), 'config', 'drone_params.yaml')

    return [


        # Robot state publisher for TF
        Node(
            namespace = namesp,
            # namespace = 'drone'+suffix,
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ), # namespace good
        # Estimation position
        Node(
            namespace = namesp,
            # namespace = 'drone'+suffix,
            package='tello_position',
            executable='tello_position_cal_CATS',
            output='screen',
            parameters=[{
                'initial_x': x,
                'initial_y': y,
                'initial_z': z
            }]
        ), # namespace good
        # # Estimation consommation batterie
        # Node(
        # namespace = namesp,    
        # # namespace = 'drone'+suffix,
        #     package='drone_energy_estimation',
        #     executable='battery_state',
        #     name='battery_state',
        #     output='screen',
        #     parameters=[drone_config, {'namespace': ns}]
        #     # parameters=[drone_config]
        # ), # namespace good

        Node(
            namespace = namesp,
            # namespace = 'drone'+suffix,
            package='gazebo_ros',
            executable='spawn_entity.py',
            # name='spawn_' + ns,
            arguments=[
                '-file', urdf_path,
                '-entity', 'drone'+suffix,
                '-robot_namespace', namesp, # added
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
            ],
            output='screen'
        ), # namespace good
    ]

def tello_driver_drone(namesp, ip, command_port, data_port, video_port, x, y, z, idx):
    suffix = '_' + str(idx)
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello' + suffix + '.urdf')
    drone_config = os.path.join(get_package_share_directory('drone_energy_estimation'), 'config', 'drone_params.yaml')

    return [

        

        # Manette (joy_node) – à adapter si plusieurs manettes
        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ), # no namespace

        # Joystick vers cmd_vel
        Node(
            package='tello_driver',
            executable='tello_joy_main',
            output='screen'
        ), # no namespace

        # Driver Tello (connexion au vrai drone via IP)
        Node(
            package='tello_driver',
            executable='tello_driver_main',
            output='screen',
            parameters=[{
                'drone_ip': ip,
                'drone_port': 8889,
                'command_port': command_port,
                'data_port': data_port,
                'video_port': video_port
            }]        
        ), # no namespace



    ]

def generate_launch_description():

    # namespace_arg = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='drone1',
    #     description='Namespace for the drone nodes'
    # )
    # namespace = LaunchConfiguration('namespace')

    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')

    drones = [
        {'ns': 'drone_1', 'ip': '192.168.50.103', 'command_port':38065, 'data_port':8890, 'video_port':11111, 'x': 0., 'y': 1., 'z': 1., 'urdf': 1},
        {'ns': 'drone_2', 'ip': '192.168.50.33', 'command_port':38065, 'data_port':8890, 'video_port':11111,  'x': 1., 'y': 0., 'z': 1., 'urdf': 2},
        # Ajouter plus de drones ici si besoin
    ]

    drone_nodes = []
    
    index_util = 0
    # while namespace != drones[index_util]['ns'] and index_util < len(drones):
    #     print(namespace,drones[index_util]['ns'])
    #     index_util += 1
    # if index_util == len(drones):
    #     raise Exception("was not able to find which drone to control")

    for drone in drones:
        drone_nodes.extend(launch_drone_gazebo(
            drone['ns'], 
            drone['ip'], 
            drone['command_port'], 
            drone['data_port'], 
            drone['video_port'], 
            drone['x'], 
            drone['y'], 
            drone['z'], 
            drone['urdf']
        ))
    
    drone_nodes.extend(tello_driver_drone(
        drones[index_util]['ns'],
        drones[index_util]['ip'],
        drones[index_util]['command_port'],
        drones[index_util]['data_port'],
        drones[index_util]['video_port'],
        drones[index_util]['x'],
        drones[index_util]['y'],
        drones[index_util]['z'],
        drones[index_util]['urdf']
    ))

    return LaunchDescription([
        # Lancement de Gazebo
        ExecuteProcess(
            cmd=[
                'gazebo', 
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_path
            ],
            output='screen'
        ),
        *drone_nodes
    ])
