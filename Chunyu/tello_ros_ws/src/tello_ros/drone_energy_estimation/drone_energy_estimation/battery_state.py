import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
import math
import sys
import numpy as np

from energy_report.msg import MotorState

# 添加模块路径
module_path = "/home/chunyu/tello_ros_ws/install/drone_energy_estimation/lib/python3.8/site-packages/drone_energy_estimation"
if module_path not in sys.path:
    sys.path.append(module_path)

from motor_model import MotorModel
from drone_model import DroneKinematic
from drone_model import DroneDynamics
from battery_model import BatteryModel

class BatteryState(Node):
    def __init__(self):
        super().__init__('battery_state')

        # Declare parameters
        self.declare_parameter('namespace', 'drone1')
        self.declare_parameter('drone.mass', 0.08)
        self.declare_parameter('drone.Ix', 0.679e-2)
        self.declare_parameter('drone.Iy', 0.679e-2)
        self.declare_parameter('drone.Iz', 1.313e-2)
        self.declare_parameter('drone.kx', 3.365e-2)
        self.declare_parameter('drone.ky', 3.365e-2)
        self.declare_parameter('drone.kz', 3.365e-2)
        self.declare_parameter('drone.d', [0.0]*6)
        self.declare_parameter('drone.rho', 0.566e-5)
        self.declare_parameter('drone.L', 0.06)
        self.declare_parameter('drone.gamma', 0.762e-7)

        self.declare_parameter('battery.V0', 3.8)
        self.declare_parameter('battery.alpha', -0.05)
        self.declare_parameter('battery.Q', 3600*1.1)
        self.declare_parameter('battery.R', 0.01)

        self.declare_parameter('motor.L', 0.2)
        self.declare_parameter('motor.R', 0.01)
        self.declare_parameter('motor.K_e', 6.8e-3)
        self.declare_parameter('motor.K_T', 6.8e-3)
        self.declare_parameter('motor.K_d', 0.762e-7)
        self.declare_parameter('motor.J', 0.2)
        self.declare_parameter('motor.f', 0.01)

        self.declare_parameter('imu_callback_frequency', 10.0)  # IMU callback frequency in Hz

        # Get parameters
        self.namespace = self.get_parameter('namespace').value
        self.imu_callback_frequency = self.get_parameter('imu_callback_frequency').value

        m = self.get_parameter('drone.mass').value
        Ix = self.get_parameter('drone.Ix').value
        Iy = self.get_parameter('drone.Iy').value
        Iz = self.get_parameter('drone.Iz').value
        kx = self.get_parameter('drone.kx').value
        ky = self.get_parameter('drone.ky').value
        kz = self.get_parameter('drone.kz').value
        d = self.get_parameter('drone.d').value
        rho = self.get_parameter('drone.rho').value
        L = self.get_parameter('drone.L').value
        gamma = self.get_parameter('drone.gamma').value

        V0 = self.get_parameter('battery.V0').value
        alpha = self.get_parameter('battery.alpha').value
        Q = self.get_parameter('battery.Q').value
        R_batt = self.get_parameter('battery.R').value

        L_motor = self.get_parameter('motor.L').value
        R_motor = self.get_parameter('motor.R').value
        K_e = self.get_parameter('motor.K_e').value
        K_T = self.get_parameter('motor.K_T').value
        K_d = self.get_parameter('motor.K_d').value
        J = self.get_parameter('motor.J').value
        f = self.get_parameter('motor.f').value

        self.dt = 1/self.imu_callback_frequency
        self.drone_kinematic = DroneKinematic(dt=self.dt)
        self.drone_dynamics = DroneDynamics(m, Ix, Iy, Iz, kx, ky, kz, d, rho, L, gamma)
        self.motors = [MotorModel(L_motor, R_motor, K_e, K_T, K_d, J, f, self.dt) for _ in range(4)]
        self.battery = BatteryModel(V0, alpha, Q, R_batt, self.dt)

        self.omegas = [0.0]*4
        self.flight_state = 'landed'

        # Subscriptions and publications
        imu_topic = f'{self.namespace}/imu/data_raw'
        battery_topic = f'{self.namespace}/battery_remaining'
        motor_topic = f'{self.namespace}/motor_state'
        flight_state_topic = f'{self.namespace}/flight_state'

        self.subscription = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.battery_pub = self.create_publisher(Float32, battery_topic, 1)
        self.motor_pub = self.create_publisher(MotorState, motor_topic, 10)
        self.flight_state_sub = self.create_subscription(String, flight_state_topic, self.flight_state_callback, 10)
        
        # Create a timer to control the callback frequency
        self.timer = self.create_timer(1.0 / self.imu_callback_frequency, self.timer_callback)

        # Open a file to write motor states and battery remaining information
        try:
            self.file = open('drone_data.txt', 'a')
        except Exception as e:
            self.get_logger().error(f"Failed to open file: {str(e)}")
            self.file = None

    def imu_callback(self, msg):
        self.last_imu_msg = msg

    def flight_state_callback(self, msg):
        self.flight_state = msg.data

    def timer_callback(self):
        motor_state_msg = MotorState()

        if hasattr(self, 'last_imu_msg'):
            msg = self.last_imu_msg
            orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            angular_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            linear_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z - 9.8]

            state_curr = self.drone_kinematic.update(orientation, angular_vel, linear_acc)

            if self.flight_state in ['flying', 'taking_off', 'landing']:
                motor_state_msg.state = "working"
                omegas_current = self.drone_dynamics.compute_rotating_speed(state_curr)
            else:
                motor_state_msg.state = "not working"
                omegas_current = [0.0, 0.0, 0.0, 0.0]

            for i in range(4):
                self.motors[i].update(omegas_current[i], (omegas_current[i] - self.omegas[i]) / self.dt)
                self.omegas[i] = omegas_current[i]

            i_all = sum(motor.i for motor in self.motors)

            if not math.isnan(i_all):
                self.battery.compute_capacity(i_all)

            if self.battery.it > self.battery.Q:
                self.get_logger().info("Battery depleted!")
            #    rclpy.shutdown()

            battery_remaining_msg = Float32()
            battery_remaining_msg.data = 1.0 - self.battery.it / self.battery.Q
            self.battery_pub.publish(battery_remaining_msg)

            
            motor_state_msg.omega = [omegas_current[0], omegas_current[1], omegas_current[2], omegas_current[3]]
            motor_state_msg.i = [self.motors[0].i, self.motors[1].i, self.motors[2].i, self.motors[3].i]
            self.motor_pub.publish(motor_state_msg)

            # Write motor state and battery remaining information to the file
            if self.file:
                self.file.write(f"Linear Acceleration: {linear_acc}\n")
                self.file.write(f"State: {state_curr}\n")
                self.file.write(f"Omega: {omegas_current}\n")
                self.file.write(f"Currents: {[motor.i for motor in self.motors]}\n")
                self.file.write(f"battery: {1.0 - self.battery.it / self.battery.Q}\n")

    def __del__(self):
        if self.file:
            try:
                self.file.close()
            except Exception as e:
                self.get_logger().error(f"Failed to close file: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

