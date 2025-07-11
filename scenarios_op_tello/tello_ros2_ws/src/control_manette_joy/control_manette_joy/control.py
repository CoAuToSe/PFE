import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy  # Importer les messages du package Joy
from geometry_msgs.msg import Twist
import time


class Control(Node):

    def __init__(self):
        super().__init__('control')

          # Publisher pour les commandes de décollage et d'atterrissage
        self.takeoff_publisher = self.create_publisher(Empty, '/takeoff', 1)
        self.land_publisher = self.create_publisher(Empty, '/land', 1)
        self.emergency_publisher = self.create_publisher(Empty, '/emergency', 1)
        self.flip_publisher = self.create_publisher(String, '/flip', 1)


        self.move_publisher = self.create_publisher(Twist, '/control', 1)

        self.StatusTakeoff = False
        self.StatusLand = True
        self.StatusFlip = False
       
        # Souscription au topic Joy pour recevoir les données de la manette Xbox 360
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        self.subscription = self.create_subscription(
            Empty,
            'takeoff',
            self.changeStatus,
            10)
        
        self.subscription = self.create_subscription(
            Empty,
            'land',
            self.changeStatus,
            10)
        
        self.subscription = self.create_subscription(
            String,
            'flip',
            self.changeStatusFlip,
            10)

        # Variables par défaut manette
        # self.leftStick_left_and_right = -0.0
        # self.leftStick_up_and_down = -0.0
        # self.LT = 1.0
        # self.rightStick_left_and_right = -0.0
        # self.rightStick_up_and_down = -0.0
        # self.RT = 1.0
        # self.arrows_left_and_right = 0.0
        # self.arrows_up_and_down = 0.0

        # self.A = 0
        # self.B = 0
        # self.X = 0
        # self.Y = 0
        # self.LB = 0
        # self.RB = 0
        # self.select = 0
        # self.start = 0
        # self.xbox = 0
        # self.leftStick = 0
        # self.rightStick = 0

        self.last_input_time = 0
        self.debounce_time = 0.200 #200 ms

    def handle_debounce(self):
        current_time = time.time()

        if current_time - self.last_input_time > self.debounce_time:
            self.last_input_time = current_time
            return True
        return False
    
    def changeStatus(self, msg):
        # Fonction pour récupérer le status du topic Takeoff and land
        self.StatusTakeoff = not self.StatusTakeoff
        self.StatusLand = not self.StatusLand

    def changeStatusFlip(self, msg):
        # Fonction pour récupérer le status du topic flip
        self.StatusFlip = not self.StatusFlip


    def joy_callback(self, msg):
        # Vérification si le bouton A (décollage) ou B (atterrissage) a été pressé
        if msg.buttons[7] == 1 :  # Bouton Start pour décollage
            if self.handle_debounce() and not self.StatusTakeoff:
                self.takeoff()
        elif msg.buttons[6] == 1 and not self.StatusLand:  # Bouton Select pour atterrissage
            if self.handle_debounce() :
                self.land()
        elif msg.buttons[8] == 1 :  # Bouton central pour atterrissage d'urgence
            if self.handle_debounce() :
                self.emergency()
        elif msg.buttons[1] == 1 :  # Bouton B pour flip right
            if self.handle_debounce() and not self.StatusFlip:
                self.flip("r")
        elif msg.buttons[2] == 1 :  # Bouton X pour flip left
            if self.handle_debounce() and not self.StatusFlip:
                self.flip("l")
        elif msg.buttons[3] == 1 :  # Bouton Y pour flip forward
            if self.handle_debounce() and not self.StatusFlip:
                self.flip("f")
        elif msg.buttons[0] == 1 :  # Bouton A pour flip backward
            if self.handle_debounce() and not self.StatusFlip:
                self.flip("b")


        self.move(msg.axes[0]*(-100), msg.axes[1]*100, msg.axes[4]*100, msg.axes[3]*(-100))

    def move(self, speed_forward_backward, speed_left_right, speed_height, speed_angular):
        
        move_msg = Twist()
        move_msg.linear.x = speed_forward_backward
        move_msg.linear.y = speed_left_right
        move_msg.linear.z = speed_height
        move_msg.angular.z = speed_angular
        self.move_publisher.publish(move_msg)

    def takeoff(self):
        # Publier un message vide sur le topic /takeoff pour décoller
        self.get_logger().info('Décollage!')
        takeoff_msg = Empty()
        self.takeoff_publisher.publish(takeoff_msg)

    def land(self):
        # Publier un message vide sur le topic /land pour atterrir
        self.get_logger().info('Atterrissage!')
        land_msg = Empty()
        self.land_publisher.publish(land_msg)

    def emergency(self):
        # Publier un message vide sur le topic /emergency pour atterrir en urgence (éteindre le drone)
        self.get_logger().info('ALERTE!')
        emergency_msg = Empty()
        self.emergency_publisher.publish(emergency_msg)

    def flip(self, arg):
        # Publier un message vide sur le topic /flip
        self.get_logger().info('Do A ROLL!!')
        flip_msg = String()
        flip_msg.data= arg
        self.flip_publisher.publish(flip_msg)

def main(args=None):
    rclpy.init(args=args)

    control_node = Control()

    rclpy.spin(control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()