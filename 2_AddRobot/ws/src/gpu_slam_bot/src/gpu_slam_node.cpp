#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gpu_slam_bot/gpu_grid.hpp"

using std::placeholders::_1;

class GpuSlamNode : public rclcpp::Node
{
public:
    GpuSlamNode()
    : Node("gpu_slam_node")
    {
        // Initialize your grid (unchanged)
        gpu_slam_bot::GridParams params;
        params.width = 200;
        params.height = 200;
        params.resolution = 0.05;
        params.origin_x = -5.0;
        params.origin_y = -5.0;
        grid_ = std::make_shared<gpu_slam_bot::GpuGrid>(params);

        map_msg_.header.frame_id = "map";
        map_msg_.info.width = params.width;
        map_msg_.info.height = params.height;
        map_msg_.info.resolution = params.resolution;
        map_msg_.info.origin.position.x = params.origin_x;
        map_msg_.info.origin.position.y = params.origin_y;
        map_msg_.info.origin.orientation.w = 1.0;
        map_msg_.data.resize(params.width * params.height, -1);

        // Publishers
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscriber
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&GpuSlamNode::onScan, this, _1));

        // Timer for autonomous movement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GpuSlamNode::moveRobot, this)
        );

        RCLCPP_INFO(this->get_logger(), "GPU SLAM Node initialized");
    }

private:
    void onScan(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Dummy robot pose
        float robot_x = 0.0;
        float robot_y = 0.0;
        float yaw     = 0.0;

        // Convert scan to vectors
        std::vector<float> ranges = scan->ranges;
        std::vector<float> angles(ranges.size());
        float angle = scan->angle_min;
        for (size_t i = 0; i < ranges.size(); ++i) {
            angles[i] = angle;
            angle += scan->angle_increment;
        }

        // Integrate scan into GPU grid
        grid_->integrateScan(ranges, angles, robot_x, robot_y, yaw, scan->range_max);

        std::vector<int8_t> occ_data;
        grid_->downloadToOcc(occ_data);
        map_msg_.data = occ_data;
        map_msg_.header.stamp = this->get_clock()->now();
        map_pub_->publish(map_msg_);
    }

    void moveRobot()
    {
        // Simple autonomous behavior: move forward, turn randomly
        auto cmd = geometry_msgs::msg::Twist();

        // 80% forward, 20% random turn
        double r = (double)rand() / RAND_MAX;
        if (r < 0.8) {
            cmd.linear.x = 0.2;   // forward speed
            cmd.angular.z = 0.0;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = (rand() % 2 == 0 ? 0.5 : -0.5); // turn left/right
        }

        cmd_pub_->publish(cmd);
    }

    std::shared_ptr<gpu_slam_bot::GpuGrid> grid_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpuSlamNode>());
    rclcpp::shutdown();
    return 0;
}
