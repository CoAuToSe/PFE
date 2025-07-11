#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include <cmath>
#include <tello_msgs/msg/tello_position.hpp>

class TelloPosition : public rclcpp::Node
{
public:
    TelloPosition() : Node("tello_position"), x_(0.0), y_(0.0), z_(0.0), vx_(0.0), vy_(0.0), vz_(0.0), last_time_(this->now())
    {
        subscription_ = this->create_subscription<tello_msgs::msg::FlightData>(
            "/flight_data",
            10, 
            std::bind(
                &TelloPosition::flight_data_callback,
                this,
                std::placeholders::_1
        ));
        publisher_ = this->create_publisher<tello_msgs::msg::TelloPosition>("/tello_position", 10);

        initial_barometer_ = 0.0;
        initial_barometer_set_ = false;
    }

private:
    void flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
    {
      auto current_time = this->now();
      double dt = (current_time - last_time_).seconds();
      last_time_ = current_time;

      if (!initial_barometer_set_)
      {
        initial_barometer_ = msg->baro;
        initial_barometer_set_ = true;
      }

      double yaw = msg->yaw * M_PI / 180.0;
      double pitch = msg->pitch * M_PI / 180.0;
      double roll = msg->roll * M_PI / 180.0;

      double vx_natural = msg->vgx * (std::cos(yaw) * std::cos(pitch)) + 
                          msg->vgy * (-std::sin(yaw) * std::cos(roll) + std::cos(yaw) * std::sin(pitch) * std::sin(roll)) +
                          msg->vgz * (std::sin(yaw) * std::sin(roll) + std::cos(yaw) * std::sin(pitch) * std::cos(roll));

      double vy_natural = msg->vgx * (std::sin(yaw) * std::cos(pitch)) + 
                          msg->vgy * (std::cos(yaw) * std::cos(roll) + std::sin(yaw) * std::sin(pitch) * std::sin(roll)) +
                          msg->vgz * (-std::cos(yaw) * std::sin(roll) + std::sin(yaw) * std::sin(pitch) * std::cos(roll));

      double vz_natural = msg->vgx * (-std::sin(pitch)) + 
                          msg->vgy * (std::cos(pitch) * std::sin(roll)) +
                          msg->vgz * (std::cos(pitch) * std::cos(roll));

      double ax_natural = msg->agx * (std::cos(yaw) * std::cos(pitch)) + 
                          msg->agy * (-std::sin(yaw) * std::cos(roll) + std::cos(yaw) * std::sin(pitch) * std::sin(roll)) +
                          (msg->agz + 1000) * (std::sin(yaw) * std::sin(roll) + std::cos(yaw) * std::sin(pitch) * std::cos(roll));

      double ay_natural = msg->agx * (std::sin(yaw) * std::cos(pitch)) + 
                          msg->agy * (std::cos(yaw) * std::cos(roll) + std::sin(yaw) * std::sin(pitch) * std::sin(roll)) +
                          (msg->agz + 1000) * (-std::cos(yaw) * std::sin(roll) + std::sin(yaw) * std::sin(pitch) * std::cos(roll));

      double az_natural = msg->agx * (-std::sin(pitch)) + 
                          msg->agy * (std::cos(pitch) * std::sin(roll)) +
                          (msg->agz + 1000) * (std::cos(pitch) * std::cos(roll));

      kalman_update(vx_, ax_natural, dt);
      kalman_update(vy_, ay_natural, dt);
      kalman_update(vz_, az_natural, dt);

      x_ += vx_natural / 100.0 * dt + 0.5 * ax_natural * dt * dt / 100.0;
      y_ += vy_natural / 100.0 * dt + 0.5 * ay_natural * dt * dt / 100.0;
      z_ += vz_natural / 100.0 * dt + 0.5 * az_natural * dt * dt / 100.0;

      double height_tof = (msg->tof / 100.0) * std::cos(pitch) * std::cos(roll);
      if (msg->tof == 6553) {
        height_tof = z_;  // 使用当前高度估计值
      }

      double height_relevant = msg->h / 100.0;
      double height_barometer = msg->baro - initial_barometer_;

      z_ = 0.4 * height_tof + 0.3 * height_relevant + 0.2 * z_ + 0.1 * height_barometer;

      auto position_msg = tello_msgs::msg::TelloPosition();
      position_msg.x = x_;
      position_msg.y = y_;
      position_msg.z = z_;
      position_msg.pitch = msg->pitch;
      position_msg.roll = msg->roll;
      position_msg.yaw = msg->yaw;

      publisher_->publish(position_msg);
    }

    void kalman_update(double& velocity, double acceleration, double dt)
    {
      static double P = 1.0;
      static double R = 1.0;
      static double Q = 0.1;

      double velocity_predict = velocity + acceleration * dt;
      P += Q;

      double K = P / (P + R);
      velocity = velocity_predict + K * (velocity - velocity_predict);
      P = (1 - K) * P;
    }

    rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr subscription_;
    rclcpp::Publisher<tello_msgs::msg::TelloPosition>::SharedPtr publisher_;
    double x_, y_, z_;
    double vx_, vy_, vz_;
    rclcpp::Time last_time_;

    double initial_barometer_;
    bool initial_barometer_set_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TelloPosition>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
