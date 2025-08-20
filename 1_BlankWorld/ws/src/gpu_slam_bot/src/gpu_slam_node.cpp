// gpu_slam_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "gpu_slam_bot/gpu_grid.hpp"

using std::placeholders::_1;

class GpuSlamNode : public rclcpp::Node
{
public:
    GpuSlamNode()
    : Node("gpu_slam_node")
    {
        // Initialize grid parameters
        gpu_slam_bot::GridParams params;
        params.width = 200;
        params.height = 200;
        params.resolution = 0.05; // 5cm per cell
        params.origin_x = -5.0;
        params.origin_y = -5.0;

        grid_ = std::make_shared<gpu_slam_bot::GpuGrid>(params);

        // Initialize occupancy grid message
        map_msg_.header.frame_id = "map";
        map_msg_.info.width = params.width;
        map_msg_.info.height = params.height;
        map_msg_.info.resolution = params.resolution;
        map_msg_.info.origin.position.x = params.origin_x;
        map_msg_.info.origin.position.y = params.origin_y;
        map_msg_.info.origin.position.z = 0.0;
        map_msg_.info.origin.orientation.w = 1.0;
        map_msg_.data.resize(params.width * params.height, -1); // unknown

        // Publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

        // Subscriber
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&GpuSlamNode::onScan, this, _1));

        RCLCPP_INFO(this->get_logger(), "GPU SLAM Node initialized");
    }

private:
    void onScan(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // TODO: replace with actual robot pose from TF or odometry
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

        // Download occupancy grid
        std::vector<int8_t> occ_data;
        grid_->downloadToOcc(occ_data);

        // Copy into map message
        map_msg_.data = occ_data;

        // Update header timestamp
        map_msg_.header.stamp = this->get_clock()->now();

        // Publish
        map_pub_->publish(map_msg_);
    }

    std::shared_ptr<gpu_slam_bot::GpuGrid> grid_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpuSlamNode>());
    rclcpp::shutdown();
    return 0;
}
