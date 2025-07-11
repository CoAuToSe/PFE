import rclpy
from rclpy.node import Node
import json
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('waypoints_file', 'waypoints/waypoints.json')
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        self.publisher_cmd = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.publisher_takeoff = self.create_publisher(Empty, '/tello/takeoff', 10)
        self.publisher_land = self.create_publisher(Empty, '/tello/land', 10)

        self.subscription_obstacle = self.create_subscription(
            Bool,
            '/obstacle_detected',
            self.obstacle_callback,
            10
        )

        self.obstacle_detected = False
        self.get_logger().info("🚁 Path Follower avec évitement d'obstacle activé !")

        # Charger et exécuter le chemin
        self.follow_path()

    def obstacle_callback(self, msg):
        """ Callback exécuté à chaque message sur `/obstacle_detected` """
        self.obstacle_detected = msg.data
        if self.obstacle_detected:
            self.get_logger().warn("⛔ Obstacle détecté ! Drone en pause.")
            self.stop_drone()

    def stop_drone(self):
        """ Arrête immédiatement le drone """
        twist = Twist()
        self.publisher_cmd.publish(twist)

    def send_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0, duration=1.0):
        """ Envoie une commande de mouvement, avec gestion des obstacles """
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = linear_z
        twist.angular.z = angular_z

        start_time = time.time()
        while time.time() - start_time < duration:
            if self.obstacle_detected:
                self.get_logger().warn("⏸ Pause à cause d'un obstacle...")
                while self.obstacle_detected:
                    time.sleep(0.5)  # Attendre que l'obstacle disparaisse
                self.get_logger().info("✅ Reprise du vol !")

            self.publisher_cmd.publish(twist)
            time.sleep(0.1)

        # Stop après mouvement
        twist.linear.x = twist.linear.y = twist.linear.z = twist.angular.z = 0.0
        self.publisher_cmd.publish(twist)

    def follow_path(self):
        """ Charge et suit la liste des waypoints """
        with open(self.waypoints_file, 'r') as f:
            waypoints = json.load(f)["wp"]

        if not waypoints:
            self.get_logger().warn("⚠️ Aucun waypoint trouvé !")
            return

        # Décollage
        self.get_logger().info("🛫 Décollage...")
        self.publisher_takeoff.publish(Empty())
        time.sleep(3)  # Attendre le décollage

        for wp in waypoints:
            dist_cm = wp["dist_cm"]
            angle_deg = wp["angle_deg"]

            self.get_logger().info(f"➡️ Déplacement : {dist_cm} cm, Rotation : {angle_deg}°")

            # Rotation (si nécessaire)
            if angle_deg != 0:
                self.send_command(angular_z=angle_deg * 0.0175, duration=2)  # Conversion en rad/s

            # Avancer
            self.send_command(linear_x=0.5, duration=dist_cm / 50)  # 0.5 m/s

        # Atterrissage
        self.get_logger().info("🛬 Atterrissage...")
        self.publisher_land.publish(Empty())

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
