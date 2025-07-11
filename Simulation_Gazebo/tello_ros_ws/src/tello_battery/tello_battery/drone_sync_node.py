import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry

class DroneSyncNode(Node):
    def __init__(self):
        super().__init__('drone_sync_node')

        self.model_name = '/drone1'

        self.cli = self.create_client(SetModelState, '/gazebo/set_model_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /gazebo/set_model_state service...')

        self.create_subscription(Odometry, '/drone1/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        state = ModelState()
        state.model_name = self.model_name
        state.pose = msg.pose.pose
        state.twist = msg.twist.twist
        state.reference_frame = 'world'

        req = SetModelState.Request()
        req.model_state = state
        self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = DroneSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
