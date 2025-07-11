import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        # On s'abonne au topic '$2'
        self.subscription = self.create_subscription(
            String,
            '/cmd_vel',
            self.listener_callback,
            10  # taille de la queue
        )
        self.subscription  # éviter que l'objet soit garbage collecté

    def listener_callback(self, msg):
        self.get_logger().info(f"Message reçu: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
