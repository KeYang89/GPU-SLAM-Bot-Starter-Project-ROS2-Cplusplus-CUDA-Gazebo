#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/box_detections", 10);

        RCLCPP_INFO(this->get_logger(), "BoxDetector initialized");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = msg->header.frame_id;
        cloud.header.stamp = this->now();
        cloud.height = 1;
        cloud.width = 0;
        cloud.fields.resize(3);
        cloud.fields[0].name = "x";
        cloud.fields[0].offset = 0;
        cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[0].count = 1;
        cloud.fields[1].name = "y";
        cloud.fields[1].offset = 4;
        cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[1].count = 1;
        cloud.fields[2].name = "z";
        cloud.fields[2].offset = 8;
        cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[2].count = 1;
        cloud.point_step = 12;
        cloud.is_dense = true;

        int cluster_size = 0;
        size_t cluster_start = 0;
        std::vector<float> cluster_x, cluster_y;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] >= min_range_ && msg->ranges[i] <= max_range_ && !std::isinf(msg->ranges[i]) && !std::isnan(msg->ranges[i])) {
                if (cluster_size == 0) cluster_start = i;
                cluster_size++;
                float angle = msg->angle_min + i * msg->angle_increment;
                float x = msg->ranges[i] * std::cos(angle);
                float y = msg->ranges[i] * std::sin(angle);
                cluster_x.push_back(x);
                cluster_y.push_back(y);
                if (cluster_size >= min_cluster_size_) {
                    // Compute centroid of cluster
                    float avg_x = 0.0, avg_y = 0.0;
                    for (size_t j = 0; j < cluster_x.size(); ++j) {
                        avg_x += cluster_x[j];
                        avg_y += cluster_y[j];
                    }
                    avg_x /= cluster_x.size();
                    avg_y /= cluster_y.size();
                    float z = 0.0;  // 2D scan, assume z=0
                    cloud.data.insert(cloud.data.end(), reinterpret_cast<const uint8_t*>(&avg_x), reinterpret_cast<const uint8_t*>(&avg_x) + sizeof(float));
                    cloud.data.insert(cloud.data.end(), reinterpret_cast<const uint8_t*>(&avg_y), reinterpret_cast<const uint8_t*>(&avg_y) + sizeof(float));
                    cloud.data.insert(cloud.data.end(), reinterpret_cast<const uint8_t*>(&z), reinterpret_cast<const uint8_t*>(&z) + sizeof(float));
                    cloud.width = 1;
                    cloud.row_step = cloud.point_step * cloud.width;
                    cloud_pub_->publish(cloud);
                    RCLCPP_INFO(this->get_logger(), "Box detected at (%.2f, %.2f), range %.2f-%.2f m", avg_x, avg_y, min_range_, max_range_);
                    return;  // Publish one centroid per scan
                }
            } else {
                cluster_size = 0;
                cluster_x.clear();
                cluster_y.clear();
            }
        }

        // No valid cluster found, publish empty cloud
        cloud_pub_->publish(cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
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