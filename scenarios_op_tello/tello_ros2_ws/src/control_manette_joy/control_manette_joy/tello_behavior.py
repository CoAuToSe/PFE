from control_interface.srv import DroneMode
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty, Bool
import rclpy
from rclpy.node import Node
import time
import json

class DroneChoix(Node):

    def __init__(self):
        super().__init__('tello_behavior')  # Nom du nœud
        self.srv = self.create_service(DroneMode, 'drone_mode', self.reponse_callback)
        self.mode = 0  # Mode par défaut (manuel)

        # Création des publishers
        self.takeoff_publisher = self.create_publisher(Empty, '/takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, '/land', 10)
        self.emergency_publisher = self.create_publisher(Empty, '/emergency', 10)
        self.move_publisher = self.create_publisher(Twist, '/control', 10)

        # Abonnement au topic /secure_cmd pour le mode manuel
        self.subscription = self.create_subscription(
            Twist,
            '/secure_cmd',
            self.control_callback,
            10)

        # Abonnement au topic /obstacle_detected
        self.obstacle_subscription = self.create_subscription(
            Bool,
            '/obstacle_detected',
            self.obstacle_callback,
            10)

        # Abonnement au topic /waypoints pour recevoir la trajectoire
        self.waypoints_subscription = self.create_subscription(
            PoseStamped,
            '/waypoints',
            self.path_follower_callback,
            10)

        self.current_waypoint_index = 0
        self.waypoints = []

    def reponse_callback(self, request, response):
        self.mode = request.mode  # Récupérer l'entier depuis la requête
        response.success = True

        # Mode manuel
        if self.mode == 0:
            self.get_logger().info("Mode manuel activé !")

        # Mode surveillance (rotation sur place)
        elif self.mode == 1:
            self.get_logger().info("Mode surveillance activé !")
            takeoff_msg = Empty()
            self.takeoff_publisher.publish(takeoff_msg)

            move_msg = Twist()
            move_msg.angular.z = 100.0
            start_time = time.time()  # Temps de départ
            self.move_publisher.publish(move_msg)

            while time.time() - start_time < 30:
                time.sleep(1)

            land_msg = Empty()
            self.land_publisher.publish(land_msg)

        # Mode Path Follower
        elif self.mode == 2:
            self.get_logger().info("Mode Path Follower activé !")
            takeoff_msg = Empty()
            self.takeoff_publisher.publish(takeoff_msg)

        return response

    def control_callback(self, msg):
        """Callback pour le mode manuel."""
        if self.mode == 0:  # On ne laisse passer les commandes que si on est en mode manuel
            self.move_publisher.publish(msg)
            self.get_logger().info("Mouvement en cours...")

    def obstacle_callback(self, msg):
        """Callback pour détecter les obstacles."""
        if msg.data:  # Si un obstacle est détecté
            self.get_logger().warn("Obstacle détecté ! Arrêt du drone.")
            stop_msg = Twist()
            self.move_publisher.publish(stop_msg)  # Stopper le drone

    def path_follower_callback(self, msg):
        """Callback pour suivre le chemin (Mode 2)."""
        if self.mode == 2:
            if self.current_waypoint_index < len(self.waypoints):
                waypoint = self.waypoints[self.current_waypoint_index]
                
                move_msg = Twist()
                move_msg.linear.x = 0.2  # Avancer doucement
                move_msg.angular.z = 0.0  # Pas de rotation pour l'instant

                self.move_publisher.publish(move_msg)
                self.get_logger().info(f"Déplacement vers waypoint {self.current_waypoint_index}...")

                self.current_waypoint_index += 1  # Passer au prochain waypoint
            else:
                self.get_logger().info("Tous les waypoints ont été atteints. Atterrissage...")
                land_msg = Empty()
                self.land_publisher.publish(land_msg)

def main():
    rclpy.init()
    drone_choix = DroneChoix()
    rclpy.spin(drone_choix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
