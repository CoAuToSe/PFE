import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self, topic_name):
        super().__init__("listener_node" + topic_name.replace("/","_"))
        self.get_logger().info(f"subscribing to: {topic_name}")
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10  # taille de la queue
        )
        self.subscription  # éviter que l'objet soit garbage collecté

    def listener_callback(self, msg):
        self.get_logger().info(f"Message reçu: {msg.data}")

def main(args=None):
    print("DEPRECATED")
    print("DEPRECATED")
    print("DEPRECATED")
    rclpy.init(args=args)
    node = ListenerNode('/chatter')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
