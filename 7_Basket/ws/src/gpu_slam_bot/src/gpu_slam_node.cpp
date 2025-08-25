#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "gpu_slam_bot/gpu_grid.hpp"

#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <utility>
#include <limits>
#include <algorithm>
#include <mutex>

using std::placeholders::_1;

class GpuSlamNode : public rclcpp::Node
{
public:
    GpuSlamNode()
        : Node("gpu_slam_node"),
          have_pose_(false),
          path_index_(0),
          is_navigation_paused_(false)
    {
        // Declare parameters
        this->declare_parameter("goal_x", 2.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("wall_min_size", 0.3);

        goal_x_ = this->get_parameter("goal_x").as_double();
        goal_y_ = this->get_parameter("goal_y").as_double();
        wall_min_size_ = this->get_parameter("wall_min_size").as_double();

        RCLCPP_INFO(this->get_logger(), "GPU SLAM Node initialized - Navigation only mode");

        // Initialize grid
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

        // Subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&GpuSlamNode::onScan, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&GpuSlamNode::onOdom, this, _1));
        robot_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_state", 10, std::bind(&GpuSlamNode::onRobotState, this, _1));

        // Timer for autonomous movement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GpuSlamNode::moveRobot, this));

        RCLCPP_INFO(this->get_logger(), "GPU SLAM Node initialized with goal (%.2f, %.2f)", goal_x_, goal_y_);
    }

private:
    void onRobotState(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        std::string state = msg->data;
        
        // Pause navigation during manipulation operations
        if (state == "APPROACHING" || state == "PICKING" || state == "LIFTING") {
            is_navigation_paused_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Navigation paused for state: %s", state.c_str());
        } else {
            is_navigation_paused_ = false;
            RCLCPP_DEBUG(this->get_logger(), "Navigation resumed for state: %s", state.c_str());
        }
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_pose_ = odom->pose.pose;
        current_velocity_ = odom->twist.twist;
        have_pose_ = true;
    }

    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!have_pose_ || scan->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid or empty laser scan or no pose available");
            return;
        }

        // Filter out small obstacles (boxes) from SLAM map
        std::vector<float> filtered_ranges = scan->ranges;
        std::vector<float> angles(filtered_ranges.size());
        float angle = scan->angle_min;
        
        for (size_t i = 0; i < filtered_ranges.size(); ++i) {
            angles[i] = angle;
            float r = filtered_ranges[i];
            
            // Filter out small obstacles that are likely boxes
            if (std::isfinite(r) && r > 0.15 && r < 0.8) {
                // Check if this looks like a small isolated object
                bool is_small_object = true;
                int neighbors_to_check = 3;
                
                for (int offset = -neighbors_to_check; offset <= neighbors_to_check; offset++) {
                    if (offset == 0) continue;
                    int neighbor_idx = static_cast<int>(i) + offset;
                    if (neighbor_idx >= 0 && neighbor_idx < static_cast<int>(filtered_ranges.size())) {
                        float neighbor_r = scan->ranges[neighbor_idx];
                        if (std::isfinite(neighbor_r) && std::abs(neighbor_r - r) < 0.2) {
                            is_small_object = false; // Part of larger structure
                            break;
                        }
                    }
                }
                
                // If it looks like a small isolated object, treat it as free space
                if (is_small_object) {
                    filtered_ranges[i] = scan->range_max;
                }
            }
            
            angle += scan->angle_increment;
        }

        float robot_x = current_pose_.position.x;
        float robot_y = current_pose_.position.y;

        double siny_cosp = 2 * (current_pose_.orientation.w * current_pose_.orientation.z +
                                current_pose_.orientation.x * current_pose_.orientation.y);
        double cosy_cosp = 1 - 2 * (current_pose_.orientation.y * current_pose_.orientation.y +
                                   current_pose_.orientation.z * current_pose_.orientation.z);
        float yaw = std::atan2(siny_cosp, cosy_cosp);

        // Use filtered ranges for SLAM
        grid_->integrateScan(filtered_ranges, angles, robot_x, robot_y, yaw, scan->range_max);

        std::vector<int8_t> occ_data;
        grid_->downloadToOcc(occ_data);
        map_msg_.data = occ_data;
        map_msg_.header.stamp = this->get_clock()->now();
        map_pub_->publish(map_msg_);

        // Store original scan for obstacle avoidance
        latest_scan_ = scan;
    }

    std::vector<std::pair<int, int>> findPath()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        std::vector<std::pair<int, int>> path;
        if (!have_pose_) {
            return path;
        }

        float res = map_msg_.info.resolution;
        float ox = map_msg_.info.origin.position.x;
        float oy = map_msg_.info.origin.position.y;
        int width = map_msg_.info.width;
        int height = map_msg_.info.height;

        int start_x = static_cast<int>((current_pose_.position.x - ox) / res);
        int start_y = static_cast<int>((current_pose_.position.y - oy) / res);
        int goal_xg = static_cast<int>((goal_x_ - ox) / res);
        int goal_yg = static_cast<int>((goal_y_ - oy) / res);

        auto isValid = [&](int x, int y) -> bool {
            if (x < 0 || x >= width || y < 0 || y >= height) {
                return false;
            }
            int8_t val = map_msg_.data[y * width + x];
            return val <= 30;
        };

        if (!isValid(start_x, start_y) || !isValid(goal_xg, goal_yg)) {
            return path;
        }

        auto make_key = [width](int x, int y) -> long {
            return static_cast<long>(x) + static_cast<long>(y) * width;
        };

        auto heuristic = [](int x1, int y1, int x2, int y2) -> float {
            return std::sqrt(static_cast<float>((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
        };

        std::priority_queue<std::pair<float, std::pair<int, int>>,
                            std::vector<std::pair<float, std::pair<int, int>>>,
                            std::greater<std::pair<float, std::pair<int, int>>>> openQueue;

        std::unordered_map<long, float> gScore;
        std::unordered_map<long, std::pair<int, int>> cameFrom;
        std::unordered_set<long> closedSet;

        long start_key = make_key(start_x, start_y);
        gScore[start_key] = 0.0f;
        openQueue.push(std::make_pair(heuristic(start_x, start_y, goal_xg, goal_yg), std::make_pair(start_x, start_y)));

        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        while (!openQueue.empty()) {
            auto top = openQueue.top();
            openQueue.pop();
            int cx = top.second.first;
            int cy = top.second.second;
            long ckey = make_key(cx, cy);

            if (closedSet.count(ckey)) {
                continue;
            }
            closedSet.insert(ckey);

            if (cx == goal_xg && cy == goal_yg) {
                std::pair<int, int> current = {cx, cy};
                while (current.first != start_x || current.second != start_y) {
                    path.push_back(current);
                    long key = make_key(current.first, current.second);
                    current = cameFrom[key];
                }
                path.push_back({start_x, start_y});
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto& dir : directions) {
                int nx = cx + dir.first;
                int ny = cy + dir.second;
                if (!isValid(nx, ny)) {
                    continue;
                }
                long nkey = make_key(nx, ny);
                if (closedSet.count(nkey)) {
                    continue;
                }
                float dist = (std::abs(dir.first) + std::abs(dir.second) == 2) ? 1.414f : 1.0f;
                float tentG = gScore[ckey] + dist;

                auto it = gScore.find(nkey);
                if (it == gScore.end() || tentG < it->second) {
                    cameFrom[nkey] = {cx, cy};
                    gScore[nkey] = tentG;
                    float h = heuristic(nx, ny, goal_xg, goal_yg);
                    openQueue.push(std::make_pair(tentG + h, std::make_pair(nx, ny)));
                }
            }
        }

        return path;
    }

    void moveRobot()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!have_pose_) {
            return;
        }

        geometry_msgs::msg::Twist cmd;

        // Pause navigation during manipulation operations
        if (is_navigation_paused_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        // Basic obstacle avoidance for walls only
        if (latest_scan_ && !latest_scan_->ranges.empty()) {
            float min_wall_front = latest_scan_->range_max;
            float min_wall_left = latest_scan_->range_max;
            float min_wall_right = latest_scan_->range_max;

            size_t num_rays = latest_scan_->ranges.size();
            float angle_min = latest_scan_->angle_min;
            float angle_inc = latest_scan_->angle_increment;

            for (size_t i = 0; i < num_rays; ++i) {
                float angle = angle_min + i * angle_inc;
                float r = latest_scan_->ranges[i];
                if (!std::isfinite(r) || r > latest_scan_->range_max) {
                    continue;
                }

                // Only consider obstacles as walls if they are very close or part of continuous structure
                bool is_wall = false;
                
                if (r < 0.12) {
                    is_wall = true; // Very close = always avoid
                } else if (r < 0.5) {
                    // Check if this is part of a continuous structure
                    int continuous_count = 0;
                    for (int offset = -2; offset <= 2; offset++) {
                        int neighbor_idx = static_cast<int>(i) + offset;
                        if (neighbor_idx >= 0 && neighbor_idx < static_cast<int>(num_rays)) {
                            float neighbor_r = latest_scan_->ranges[neighbor_idx];
                            if (std::isfinite(neighbor_r) && std::abs(neighbor_r - r) < 0.15) {
                                continuous_count++;
                            }
                        }
                    }
                    is_wall = (continuous_count >= 3);
                }

                if (is_wall) {
                    if (std::abs(angle) < M_PI / 6.0) {
                        min_wall_front = std::min(min_wall_front, r);
                    }
                    if (angle > M_PI / 6.0 && angle < M_PI / 2.0) {
                        min_wall_left = std::min(min_wall_left, r);
                    }
                    if (angle < -M_PI / 6.0 && angle > -M_PI / 2.0) {
                        min_wall_right = std::min(min_wall_right, r);
                    }
                }
            }

            // Emergency stop for very close walls
            if (min_wall_front < 0.1) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Emergency stop: wall at %.2fm", min_wall_front);
                cmd.linear.x = -0.3;
                cmd.angular.z = (min_wall_left > min_wall_right) ? 1.0 : -1.0;
                cmd_pub_->publish(cmd);
                return;
            }

            // Gentle avoidance for walls
            if (min_wall_front < 0.25) {
                cmd.linear.x = std::max(0.2, 0.4 * (min_wall_front - 0.1) / 0.15);
                cmd.angular.z = (min_wall_left > min_wall_right) ? 0.8 : -0.8;
                cmd_pub_->publish(cmd);
                return;
            }
        }

        // Path planning and following
        if (path_.empty() || path_index_ >= static_cast<int>(path_.size())) {
            path_ = findPath();
            path_index_ = 0;
            if (path_.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path found to goal");
                // Exploration behavior
                cmd.linear.x = 0.3;
                cmd.angular.z = 0.1;
                cmd_pub_->publish(cmd);
                return;
            }
        }

        std::pair<int, int> waypoint = path_[path_index_];
        int gx = waypoint.first;
        int gy = waypoint.second;
        double wx = map_msg_.info.origin.position.x + gx * map_msg_.info.resolution;
        double wy = map_msg_.info.origin.position.y + gy * map_msg_.info.resolution;

        double dx = wx - current_pose_.position.x;
        double dy = wy - current_pose_.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < 0.08) {
            path_index_++;
            if (path_index_ >= static_cast<int>(path_.size())) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                cmd_pub_->publish(cmd);
                return;
            }
            waypoint = path_[path_index_];
            gx = waypoint.first;
            gy = waypoint.second;
            wx = map_msg_.info.origin.position.x + gx * map_msg_.info.resolution;
            wy = map_msg_.info.origin.position.y + gy * map_msg_.info.resolution;
            dx = wx - current_pose_.position.x;
            dy = wy - current_pose_.position.y;
        }

        double desired_yaw = std::atan2(dy, dx);
        double siny_cosp = 2 * (current_pose_.orientation.w * current_pose_.orientation.z +
                                current_pose_.orientation.x * current_pose_.orientation.y);
        double cosy_cosp = 1 - 2 * (current_pose_.orientation.y * current_pose_.orientation.y +
                                   current_pose_.orientation.z * current_pose_.orientation.z);
        double current_yaw = std::atan2(siny_cosp, cosy_cosp);
        double yaw_error = desired_yaw - current_yaw;
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

        double base_speed = 0.5; // Moderate speed
        double speed_factor = std::min(1.0, dist / 0.5);
        cmd.linear.x = base_speed * speed_factor;
        cmd.angular.z = 2.0 * yaw_error;

        // Keep some forward motion even during turns
        if (std::abs(yaw_error) > M_PI / 4.0) {
            cmd.linear.x = 0.2;
        }

        cmd_pub_->publish(cmd);
    }

    // Member variables
    std::shared_ptr<gpu_slam_bot::GpuGrid> grid_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex state_mutex_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    bool have_pose_;
    std::vector<std::pair<int, int>> path_;
    int path_index_;
    bool is_navigation_paused_;
    double goal_x_;
    double goal_y_;
    double wall_min_size_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpuSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}