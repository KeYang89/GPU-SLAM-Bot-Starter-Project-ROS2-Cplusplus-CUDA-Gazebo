#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <cmath>

class BoxDetector : public rclcpp::Node
{
public:
    BoxDetector()
        : Node("box_detector")
    {
        declare_parameter("min_detection_range", 0.1);
        declare_parameter("max_detection_range", 0.3);
        declare_parameter("min_cluster_size", 5);

        min_range_ = get_parameter("min_detection_range").as_double();
        max_range_ = get_parameter("max_detection_range").as_double();
        min_cluster_size_ = get_parameter("min_cluster_size").as_int();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&BoxDetector::scan_callback, this, std::placeholders::_1));
        box_pub_ = create_publisher<std_msgs::msg::Bool>("/box_detected", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std_msgs::msg::Bool box_msg;
        box_msg.data = false;

        int cluster_size = 0;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] >= min_range_ && msg->ranges[i] <= max_range_) {
                cluster_size++;
                if (cluster_size >= min_cluster_size_) {
                    box_msg.data = true;
                    break;
                }
            } else {
                cluster_size = 0;
            }
        }

        box_pub_->publish(box_msg);
        if (box_msg.data) {
            RCLCPP_INFO(this->get_logger(), "Box detected within range %.2f-%.2f m", min_range_, max_range_);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr box_pub_;
    double min_range_;
    double max_range_;
    int min_cluster_size_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxDetector>());
    rclcpp::shutdown();
    return 0;
}