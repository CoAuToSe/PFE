import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class BatteryEstimator(Node):
    def __init__(self):
        super().__init__('battery_estimator')

        # Paramètres du drone
        self.k1 = 0.005  
        self.k2 = 0.003
        self.k3 = 0.02
        self.battery_voltage = 3.8  

        # Souscriptions
        self.create_subscription(Imu, '/drone1/imu', self.imu_callback, 10)
        self.create_subscription(Twist, '/drone1/cmd_vel', self.velocity_callback, 10)

        # Publication de la consommation estimée
        self.battery_pub = self.create_publisher(Float32, '/drone1/battery_estimated', 10)

        self.thrust = 0.0
        self.velocity = 0.0

    def imu_callback(self, msg):
        self.thrust = abs(msg.linear_acceleration.z)

    def velocity_callback(self, msg):
        self.velocity = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5
        power = self.k1 * (self.thrust ** 3) + self.k2 * (self.velocity ** 2) + self.k3
        energy = power / self.battery_voltage  

        msg = Float32()
        msg.data = energy
        self.battery_pub.publish(msg)
        self.get_logger().info(f'Estimation batterie : {energy:.4f} Wh')

def main():
    rclpy.init()
    node = BatteryEstimator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
