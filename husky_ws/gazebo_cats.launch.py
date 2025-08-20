from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(get_package_share_directory('husky_description')).
                                                    parent.resolve())])

    # Launch args
    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "control.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_husky_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        arguments=['-entity',
                   'husky',
                   '-topic',
                   'robot_description'],
        output='screen',
    )

    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))

    # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))

    

    ld = LaunchDescription(ARGUMENTS)

    import os
    namespaces = {
        'tello_7': [0,0,0, False],
        'tello_5': [1,1,1, False],
        'tello_2': [2,2,2, False],
    }
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    # to_return = [
    #     # Launch Gazebo, loading simple.world
    #     ExecuteProcess(cmd=[
    #         'gazebo',
    #         '--verbose',
    #         '-s', 'libgazebo_ros_init.so',  # Publish /clock
    #         '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
    #         world_path
    #     ], output='screen'),
    # ]
    for namespace, data in namespaces.items():
        urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', namespace + '.urdf')

        ld.add_action(
        # to_return.extend([
            # Publish static transforms
            Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', namespace=namespace,
                arguments=[urdf_path]),
        # ])
        )
        if data[3]:
            ld.add_action(
            # to_return.extend([
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
            )
            ld.add_action(        
            # ])
            # to_return.extend([
                Node(package='tello_position', executable='tello_position_cal_CATS', output='screen', namespace=namespace, 
                parameters=[{
                    'initial_x': data[0],
                    'initial_y': data[1],
                    'initial_z': data[2]
                }]),        
            # ])
        )
        ld.add_action(
        # to_return.extend([
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
        # ])
        )

    ld.add_action(gz_resource_path)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(diffdrive_controller_spawn_callback)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)

    return ld
