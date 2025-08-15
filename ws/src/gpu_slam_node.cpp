#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "gpu_slam_bot/gpu_grid.hpp"
#include <cmath>

using std::placeholders::_1;

/**
 * @brief Subscribes to /scan (LiDAR) topic.
 *
 * Uses TF2 to get the robot’s position and orientation.
 * Passes scan data + pose to GpuGrid (GPU-accelerated mapping).
 * Publishes the generated occupancy grid to /map.
 */

class GpuSlamNode : public rclcpp::Node {
public:
  GpuSlamNode()
  : Node("gpu_slam_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) 
  {
    declare_parameter("resolution", 0.05);
    declare_parameter("width", 800);
    declare_parameter("height", 800);
    declare_parameter("origin_x", -20.0);
    declare_parameter("origin_y", -20.0);
    declare_parameter("base_frame", "base_link");
    declare_parameter("odom_frame", "odom");

    gpu_slam_bot::GridParams gp;
    gp.resolution = get_parameter("resolution").as_double();
    gp.width = get_parameter("width").as_int();
    gp.height = get_parameter("height").as_int();
    gp.origin_x = get_parameter("origin_x").as_double();
    gp.origin_y = get_parameter("origin_y").as_double();

    grid_ = std::make_unique<gpu_slam_bot::GpuGrid>(gp);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&GpuSlamNode::onScan, this, _1)
    );

    // Pre-fill map metadata
    map_msg_.info.resolution = gp.resolution;
    map_msg_.info.width = gp.width;
    map_msg_.info.height = gp.height;
    map_msg_.info.origin.position.x = gp.origin_x;
    map_msg_.info.origin.position.y = gp.origin_y;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.resize(gp.width * gp.height, -1);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    try {
      auto base_frame = get_parameter("base_frame").as_string();
      auto odom_frame = get_parameter("odom_frame").as_string();

      geometry_msgs::msg::TransformStamped tf_stamped;
      tf_stamped = tf_buffer_.lookupTransform(
        odom_frame, base_frame, scan->header.stamp,
        tf2::durationFromSec(0.1)
      );

      double robot_x = tf_stamped.transform.translation.x;
      double robot_y = tf_stamped.transform.translation.y;
      tf2::Quaternion q;
      tf2::fromMsg(tf_stamped.transform.rotation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // Update grid on GPU
      grid_->updateFromScan(*scan, robot_x, robot_y, yaw);

      // Copy grid to OccupancyGrid msg
      grid_->copyToOccupancyGrid(map_msg_.data);

      map_msg_.header.stamp = this->now();
      map_msg_.header.frame_id = odom_frame;
      map_pub_->publish(map_msg_);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF unavailable: %s", ex.what());
    }
  }

  std::unique_ptr<gpu_slam_bot::GpuGrid> grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  nav_msgs::msg::OccupancyGrid map_msg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpuSlamNode>());
  rclcpp::shutdown();
  return 0;
}
