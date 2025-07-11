
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='tello',
			executable='tello',
			name='tello',
			output='screen',
		),

		Node(
			package='joy',
			executable='joy_node',
			name='joy_node',
		),

		Node(
			package='control_manette_joy',
			executable='control_node',
			name='control_manette_joy',
			remappings=[
                ('/control', '/secure_cmd')
            ],
		),

		Node(
			package='control_manette_joy',
			executable='tello_behavior',
			name='tello_behavior',
		),

		Node(
			package='qrcode_reader',
			executable='qrcode_reader',
			name='qrcode_reader',
		),

		
		Node(
			package='rqt_image_view',
			executable='rqt_image_view',
			name='rqt_image_view',
			output='screen',
			parameters=[{
				'topic':'/camera/image_raw',
			}],
		),

	])
