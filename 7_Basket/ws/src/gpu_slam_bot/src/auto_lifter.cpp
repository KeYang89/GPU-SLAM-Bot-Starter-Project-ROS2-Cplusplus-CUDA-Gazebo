#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"

class AutoLifter : public rclcpp::Node
{
public:
  AutoLifter() : Node("auto_lifter")
  {
    pub_ = this->create_publisher<std_msgs::msg::Float64>("/front_lifter_cmd", 1);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&AutoLifter::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double min_dist = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    auto command = std_msgs::msg::Float64();
    command.data = (min_dist < 0.2) ? 0.2 : 0.0;
    pub_->publish(command);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoLifter>());
  rclcpp::shutdown();
  return 0;
}
