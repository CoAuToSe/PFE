import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class TelloAutoScan(Node):
    def __init__(self):
        super().__init__('tello_auto_scan')

        # Convertisseur image ROS2 <-> OpenCV
        self.bridge = CvBridge()

        # Subscriber pour la caméra
        self.image_sub = self.create_subscription(
            Image, '/drone1/image_raw', self.image_callback, 10)

        # Publisher pour envoyer des commandes de mouvement
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

        # Stocker les dimensions détectées
        self.wall_width_m = None
        self.wall_height_m = None
        self.grid_size = (3, 3)  # Grid 3x3 (modifiable)

        # Timer pour démarrer la mission après 5 secondes
        self.timer = self.create_timer(5.0, self.start_scan)

    def image_callback(self, msg):
        """Détecte les contours du mur et estime ses dimensions"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Trouver les contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Prendre le plus grand contour (supposé être le mur)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)

                # Estimation en mètres
                self.wall_width_m = w * 0.05
                self.wall_height_m = h * 0.05

                self.get_logger().info(f"🔍 Mur détecté : {self.wall_width_m:.2f}m x {self.wall_height_m:.2f}m")

        except Exception as e:
            self.get_logger().error(f"Erreur détection mur : {e}")

    def send_velocity_command(self, x=0.0, y=0.0, z=0.0, yaw=0.0, duration=2.0):
        """Envoie une commande de déplacement"""
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.z = yaw

        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)
        self.cmd_vel_pub.publish(Twist())

    def start_scan(self):
        """Scanne le mur en suivant une grille"""
        if self.wall_width_m is None or self.wall_height_m is None:
            self.get_logger().warn("Dimensions du mur inconnues, scan annulé")
            return

        self.get_logger().info("Début du scan du mur en grille")

        rows, cols = self.grid_size
        step_x = self.wall_width_m / cols
        step_y = self.wall_height_m / rows

        # Se positionner en haut à gauche
        self.send_velocity_command(z=0.5, duration=3)  

        for i in range(rows):
            for j in range(cols):
                # Déplacement grille
                self.send_velocity_command(x=step_x, duration=1)
                self.send_velocity_command(y=-step_y, duration=1)

                # Prendre une photo (simulé par un log ici)
                self.get_logger().info(f"📸 Photo prise à ({i}, {j})")

            # Retour début de ligne
            self.send_velocity_command(x=-self.wall_width_m, duration=3)

        self.get_logger().info("✅ Scan terminé")

def main(args=None):
    rclpy.init(args=args)
    node = TelloAutoScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
