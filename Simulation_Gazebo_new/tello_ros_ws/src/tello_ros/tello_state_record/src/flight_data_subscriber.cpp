#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>

class FlightDataSubscriber : public rclcpp::Node
{
public:
  FlightDataSubscriber(const rclcpp::NodeOptions& options) : Node("flight_data_subscriber", options)
  {
    subscription_ = this->create_subscription<tello_msgs::msg::FlightData>(
      "flight_data", 10, std::bind(&FlightDataSubscriber::topic_callback, this, std::placeholders::_1));

    data_file_.open("flight_data.txt", std::ios::out | std::ios::trunc);
    if (!data_file_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing");
    }
  }

  ~FlightDataSubscriber()
  {
    if (data_file_.is_open())
    {
      data_file_.close();
    }
  }

private:
  void topic_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
  {
    if (!data_file_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "File is not open for writing");
      return;
    }

    // 获取当前时间
    auto now = this->now();
    std::chrono::nanoseconds ns = std::chrono::nanoseconds(now.nanoseconds());
    std::chrono::system_clock::time_point time_point(ns);
    std::time_t now_c = std::chrono::system_clock::to_time_t(time_point);
    std::stringstream time_stream;
    time_stream << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    // 写入数据到文件
    data_file_ << "Timestamp: " << time_stream.str() << "\n";
    data_file_ << "Raw: " << msg->raw << "\n";
    data_file_ << "SDK: " << msg->sdk << "\n";
    data_file_ << "Pitch: " << msg->pitch << "\n";
    data_file_ << "Roll: " << msg->roll << "\n";
    data_file_ << "Yaw: " << msg->yaw << "\n";
    data_file_ << "VGX: " << msg->vgx << "\n";
    data_file_ << "VGY: " << msg->vgy << "\n";
    data_file_ << "VGZ: " << msg->vgz << "\n";
    data_file_ << "TempL: " << msg->templ << "\n";
    data_file_ << "TempH: " << msg->temph << "\n";
    data_file_ << "TOF: " << msg->tof << "\n";
    data_file_ << "Height: " << msg->h << "\n";
    data_file_ << "Battery: " << msg->bat << "\n";
    data_file_ << "Barometer: " << msg->baro << "\n";
    data_file_ << "Time: " << msg->time << "\n";
    data_file_ << "AGX: " << msg->agx << "\n";
    data_file_ << "AGY: " << msg->agy << "\n";
    data_file_ << "AGZ: " << msg->agz << "\n";
    data_file_ << "----------------------------------------\n";
  }

  rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr subscription_;
  std::ofstream data_file_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
  auto node = std::make_shared<FlightDataSubscriber>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}