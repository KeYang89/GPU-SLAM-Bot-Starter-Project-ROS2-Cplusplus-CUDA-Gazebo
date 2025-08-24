#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

enum class EscapeState {
    NORMAL,
    CONTACT_DETECTED,
    PUSHING_AWAY,
    BACKING_UP,
    TURNING_AWAY,
    REORIENTING
};

enum class EscapeStrategy {
    SIMPLE_BACK_TURN,
    WALL_SLIDE,
    AGGRESSIVE_PUSH,
    CORNER_ESCAPE,
    WIGGLE_ESCAPE,
    SPIRAL_ESCAPE
};

class GpuSlamNode : public rclcpp::Node
{
public:
    GpuSlamNode()
        : Node("gpu_slam_node"),
      escape_state_(EscapeState::NORMAL),
      escape_start_time_(this->get_clock()->now()),
      current_strategy_(EscapeStrategy::SIMPLE_BACK_TURN),
      have_pose_(false),
      left_contact_(false),
      right_contact_(false),
      escape_attempts_(0),
      stall_count_(0),
      path_index_(0),
      is_corner_trapped_(false),
      initial_contact_distance_(0.0),
      escape_direction_(1.0),
      last_left_gripper_angle_(0.0),
      last_right_gripper_angle_(0.0),
      last_lifter_height_(0.0)
    {
        // Declare parameters
        this->declare_parameter("goal_x", 2.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("max_escape_duration", 30.0);
        this->declare_parameter("gripper_retract_angle", 0.5);
        this->declare_parameter("gripper_extend_angle", -0.5);
        this->declare_parameter("gripper_sense_angle", 0.2);
        this->declare_parameter("lifter_height", 0.1);
        this->declare_parameter("ignore_boxes", true);  // NEW PARAMETER
        this->declare_parameter("wall_min_size", 0.3);  // Minimum size to be considered a wall

        goal_x_ = this->get_parameter("goal_x").as_double();
        goal_y_ = this->get_parameter("goal_y").as_double();
        max_escape_duration_ = this->get_parameter("max_escape_duration").as_double();
        gripper_retract_angle_ = this->get_parameter("gripper_retract_angle").as_double();
        gripper_extend_angle_ = this->get_parameter("gripper_extend_angle").as_double();
        gripper_sense_angle_ = this->get_parameter("gripper_sense_angle").as_double();
        lifter_height_ = this->get_parameter("lifter_height").as_double();
        ignore_boxes_ = this->get_parameter("ignore_boxes").as_bool();
        wall_min_size_ = this->get_parameter("wall_min_size").as_double();

        RCLCPP_INFO(this->get_logger(), "GPU SLAM Node initialized - Box ignoring: %s", 
                    ignore_boxes_ ? "ENABLED" : "DISABLED");

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
        gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("gripper_cmd", 10);
        lifter_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("front_lifter_cmd", 10);

        // Subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&GpuSlamNode::onScan, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&GpuSlamNode::onOdom, this, _1));
        left_contact_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
            "left_gripper_contact", 10, std::bind(&GpuSlamNode::onLeftContact, this, _1));
        right_contact_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
            "right_gripper_contact", 10, std::bind(&GpuSlamNode::onRightContact, this, _1));
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&GpuSlamNode::onJointState, this, _1));

        // Timer for autonomous movement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GpuSlamNode::moveRobot, this));

        RCLCPP_INFO(this->get_logger(), "GPU SLAM Node initialized with goal (%.2f, %.2f)", goal_x_, goal_y_);
    }
private:
    bool ignore_boxes_;
    double wall_min_size_;
private:
    void sendGripperCommand(double left_angle)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->get_clock()->now();
        traj.joint_names = {"left_gripper_joint", "right_gripper_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {left_angle, -left_angle};
        point.time_from_start = builtin_interfaces::msg::Duration();
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 500000000; // 0.5s
        traj.points = {point};
        gripper_pub_->publish(traj);
        RCLCPP_DEBUG(this->get_logger(), "Gripper command sent: left=%.2f, right=%.2f", left_angle, -left_angle);
    }

    void sendLifterCommand(double height)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->get_clock()->now();
        traj.joint_names = {"front_lifter_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {height};
        point.time_from_start = builtin_interfaces::msg::Duration();
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 500000000; // 0.5s
        traj.points = {point};
        lifter_pub_->publish(traj);
        RCLCPP_DEBUG(this->get_logger(), "Lifter command sent: height=%.2f", height);
    }

    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "left_gripper_joint") {
                last_left_gripper_angle_ = msg->position[i];
                RCLCPP_DEBUG(this->get_logger(), "Left gripper joint state: %.2f", last_left_gripper_angle_);
            } else if (msg->name[i] == "right_gripper_joint") {
                last_right_gripper_angle_ = msg->position[i];
                RCLCPP_DEBUG(this->get_logger(), "Right gripper joint state: %.2f", last_right_gripper_angle_);
            } else if (msg->name[i] == "front_lifter_joint") {
                last_lifter_height_ = msg->position[i];
                RCLCPP_DEBUG(this->get_logger(), "Lifter joint state: %.2f", last_lifter_height_);
            }
        }
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_pose_ = odom->pose.pose;
        current_velocity_ = odom->twist.twist;
        have_pose_ = true;
        checkStall();
    }

    void onLeftContact(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        left_contact_ = !msg->states.empty();
        if (left_contact_ && escape_state_ == EscapeState::NORMAL) {
            RCLCPP_INFO(this->get_logger(), "Left gripper contact detected");
            sendGripperCommand(gripper_retract_angle_); // Immediate retraction
            initiateEscapeSequence();
        }
    }

    void onRightContact(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        right_contact_ = !msg->states.empty();
        if (right_contact_ && escape_state_ == EscapeState::NORMAL) {
            RCLCPP_INFO(this->get_logger(), "Right gripper contact detected");
            sendGripperCommand(gripper_retract_angle_); // Immediate retraction
            initiateEscapeSequence();
        }
    }

    void checkStall()
    {
        if (escape_state_ == EscapeState::NORMAL) return;
        double speed = std::sqrt(std::pow(current_velocity_.linear.x, 2) + std::pow(current_velocity_.angular.z, 2));
        if (speed < 0.01 && (left_contact_ || right_contact_)) {
            stall_count_++;
            if (stall_count_ > 20) { // ~1s at 50ms timer
                RCLCPP_WARN(this->get_logger(), "Stall detected, switching strategy");
                switchEscapeStrategy();
                stall_count_ = 0;
            }
        } else {
            stall_count_ = 0;
        }
    }

    void initiateEscapeSequence()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        escape_state_ = EscapeState::CONTACT_DETECTED;
        escape_start_time_ = this->get_clock()->now();
        last_progress_time_ = escape_start_time_;
        escape_start_pose_ = current_pose_;
        escape_attempts_++;
        stall_count_ = 0;

        path_.clear();
        path_index_ = 0;

        sendLifterCommand(lifter_height_);

        analyzeTrapSituation();
        current_strategy_ = selectEscapeStrategy();

        RCLCPP_INFO(this->get_logger(), "Initiating escape sequence (attempt %d), strategy: %s",
                    escape_attempts_, getStrategyName(current_strategy_).c_str());
    }

    void analyzeTrapSituation()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_scan_) return;

        std::vector<float> sector_clearances(8, 0.0f);
        std::vector<int> sector_counts(8, 0);

        size_t num_rays = latest_scan_->ranges.size();
        float angle_min = latest_scan_->angle_min;
        float angle_inc = latest_scan_->angle_increment;

        float min_front = latest_scan_->range_max;
        int blocked_sectors = 0;

        for (size_t i = 0; i < num_rays; ++i) {
            float angle = angle_min + i * angle_inc;
            float r = latest_scan_->ranges[i];
            if (!std::isfinite(r) || r > latest_scan_->range_max) {
                r = latest_scan_->range_max;
            }

            float normalized_angle = std::fmod(angle + 2 * M_PI, 2 * M_PI);
            int sector = static_cast<int>(normalized_angle / (M_PI / 4.0)) % 8;
            sector_clearances[sector] += r;
            sector_counts[sector]++;

            if (std::abs(angle) < M_PI / 6.0) {
                min_front = std::min(min_front, r);
            }
        }

        for (int i = 0; i < 8; ++i) {
            if (sector_counts[i] > 0) {
                sector_clearances[i] /= sector_counts[i];
                if (sector_clearances[i] < 0.25) {
                    blocked_sectors++;
                }
            }
        }

        initial_contact_distance_ = min_front;
        is_corner_trapped_ = blocked_sectors >= 4;
        escape_direction_ = (left_contact_ && !right_contact_) ? -1.0 : (right_contact_ && !left_contact_) ? 1.0 : determineBestEscapeDirection();

        RCLCPP_INFO(this->get_logger(), "Trap analysis: front_dist=%.2f, blocked_sectors=%d, corner_trapped=%s, direction=%.1f",
                    min_front, blocked_sectors, is_corner_trapped_ ? "true" : "false", escape_direction_);
    }

    EscapeStrategy selectEscapeStrategy()
    {
        if (is_corner_trapped_ || escape_attempts_ >= 3) {
            return EscapeStrategy::WIGGLE_ESCAPE;
        } else if (initial_contact_distance_ < 0.1) {
            return EscapeStrategy::AGGRESSIVE_PUSH;
        } else if (escape_attempts_ >= 2) {
            return EscapeStrategy::CORNER_ESCAPE;
        } else if (escape_attempts_ == 1) {
            return EscapeStrategy::WALL_SLIDE;
        } else {
            return EscapeStrategy::SIMPLE_BACK_TURN;
        }
    }

    std::string getStrategyName(EscapeStrategy strategy)
    {
        switch (strategy) {
            case EscapeStrategy::SIMPLE_BACK_TURN: return "SIMPLE_BACK_TURN";
            case EscapeStrategy::WALL_SLIDE: return "WALL_SLIDE";
            case EscapeStrategy::AGGRESSIVE_PUSH: return "AGGRESSIVE_PUSH";
            case EscapeStrategy::CORNER_ESCAPE: return "CORNER_ESCAPE";
            case EscapeStrategy::WIGGLE_ESCAPE: return "WIGGLE_ESCAPE";
            case EscapeStrategy::SPIRAL_ESCAPE: return "SPIRAL_ESCAPE";
            default: return "UNKNOWN";
        }
    }

    double determineBestEscapeDirection()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_scan_) return 1.0;

        float left_clearance = 0.0;
        float right_clearance = 0.0;
        int left_count = 0;
        int right_count = 0;

        size_t num_rays = latest_scan_->ranges.size();
        float angle_min = latest_scan_->angle_min;
        float angle_inc = latest_scan_->angle_increment;

        for (size_t i = 0; i < num_rays; ++i) {
            float angle = angle_min + i * angle_inc;
            float r = latest_scan_->ranges[i];
            if (!std::isfinite(r) || r > latest_scan_->range_max) {
                r = latest_scan_->range_max;
            }

            if (angle > M_PI / 6.0 && angle < M_PI / 2.0) {
                left_clearance += r;
                left_count++;
            } else if (angle < -M_PI / 6.0 && angle > -M_PI / 2.0) {
                right_clearance += r;
                right_count++;
            }
        }

        double avg_left = left_count > 0 ? left_clearance / left_count : 0.0;
        double avg_right = right_count > 0 ? right_clearance / right_count : 0.0;

        return (avg_left > avg_right) ? 1.0 : -1.0;
    }

    bool executeEscapeSequence()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto now = this->get_clock()->now();
        double elapsed = (now - escape_start_time_).seconds();

        if (elapsed > max_escape_duration_) {
            RCLCPP_WARN(this->get_logger(), "Escape sequence timed out");
            escape_state_ = EscapeState::NORMAL;
            escape_attempts_ = 0;
            left_contact_ = false;
            right_contact_ = false;
            sendLifterCommand(0.0);
            sendGripperCommand(0.0);
            return true;
        }

        if ((now - last_progress_time_).seconds() > 2.0) {
            if (!checkEscapeProgress()) {
                RCLCPP_WARN(this->get_logger(), "No progress detected, switching strategy");
                switchEscapeStrategy();
                return false;
            }
            last_progress_time_ = now;
        }

        switch (current_strategy_) {
            case EscapeStrategy::SIMPLE_BACK_TURN:
                return executeSimpleBackTurn(elapsed);
            case EscapeStrategy::WALL_SLIDE:
                return executeWallSlide(elapsed);
            case EscapeStrategy::AGGRESSIVE_PUSH:
                return executeAggressivePush(elapsed);
            case EscapeStrategy::CORNER_ESCAPE:
                return executeCornerEscape(elapsed);
            case EscapeStrategy::WIGGLE_ESCAPE:
                return executeWiggleEscape(elapsed);
            case EscapeStrategy::SPIRAL_ESCAPE:
                return executeSpiralEscape(elapsed);
            default:
                return executeSimpleBackTurn(elapsed);
        }
    }

    bool checkEscapeProgress()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_scan_) return true;

        float current_clearance = 0.0f;
        size_t num_rays = latest_scan_->ranges.size();
        int valid_rays = 0;

        for (size_t i = 0; i < num_rays; ++i) {
            float r = latest_scan_->ranges[i];
            if (std::isfinite(r) && r <= latest_scan_->range_max) {
                current_clearance += r;
                valid_rays++;
            }
        }

        if (valid_rays > 0) {
            current_clearance /= valid_rays;
        }

        double distance_moved = std::sqrt(
            std::pow(current_pose_.position.x - escape_start_pose_.position.x, 2) +
            std::pow(current_pose_.position.y - escape_start_pose_.position.y, 2));

        bool made_progress = (current_clearance > initial_contact_distance_ + 0.03) ||
                            (distance_moved > 0.05);

        RCLCPP_DEBUG(this->get_logger(), "Progress check: clearance=%.2f, distance_moved=%.2f, progress=%s",
                    current_clearance, distance_moved, made_progress ? "true" : "false");

        return made_progress;
    }

    void switchEscapeStrategy()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        switch (current_strategy_) {
            case EscapeStrategy::SIMPLE_BACK_TURN:
                current_strategy_ = EscapeStrategy::WALL_SLIDE;
                break;
            case EscapeStrategy::WALL_SLIDE:
                current_strategy_ = EscapeStrategy::AGGRESSIVE_PUSH;
                break;
            case EscapeStrategy::AGGRESSIVE_PUSH:
                current_strategy_ = EscapeStrategy::CORNER_ESCAPE;
                break;
            case EscapeStrategy::CORNER_ESCAPE:
                current_strategy_ = EscapeStrategy::WIGGLE_ESCAPE;
                break;
            case EscapeStrategy::WIGGLE_ESCAPE:
                current_strategy_ = EscapeStrategy::SPIRAL_ESCAPE;
                break;
            default:
                current_strategy_ = EscapeStrategy::SIMPLE_BACK_TURN;
                break;
        }

        escape_start_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Switched to strategy: %s", getStrategyName(current_strategy_).c_str());
    }

    bool executeSimpleBackTurn(double elapsed)
    {
        geometry_msgs::msg::Twist cmd;
        if (elapsed < 1.5) {
            sendGripperCommand(gripper_retract_angle_);
            sendLifterCommand(lifter_height_);
            cmd.linear.x = -0.6;
            cmd.angular.z = 0.0;
        } else if (elapsed < 4.0) {
            sendGripperCommand(gripper_sense_angle_);
            cmd.linear.x = -0.4;
            cmd.angular.z = escape_direction_ * 2.5;
        } else {
            RCLCPP_INFO(this->get_logger(), "Simple escape completed");
            escape_state_ = EscapeState::NORMAL;
            sendLifterCommand(0.0);
            sendGripperCommand(0.0);
            return true;
        }

        cmd_pub_->publish(cmd);
        return false;
    }

    bool executeWallSlide(double elapsed)
    {
        geometry_msgs::msg::Twist cmd;
        if (elapsed < 2.0) {
            sendGripperCommand(gripper_retract_angle_);
            sendLifterCommand(lifter_height_);
            cmd.linear.x = 0.0;
            cmd.angular.z = escape_direction_ * 1.5;
        } else if (elapsed < 8.0) {
            sendGripperCommand(gripper_sense_angle_);
            cmd.linear.x = 0.5;
            float wall_distance = getWallDistance();
            if (wall_distance < 0.1) {
                cmd.angular.z = -escape_direction_ * 0.7;
            } else if (wall_distance > 0.2) {
                cmd.angular.z = escape_direction_ * 0.7;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Wall slide completed");
            escape_state_ = EscapeState::NORMAL;
            sendLifterCommand(0.0);
            sendGripperCommand(0.0);
            return true;
        }

        cmd_pub_->publish(cmd);
        return false;
    }

    bool executeAggressivePush(double elapsed)
    {
        geometry_msgs::msg::Twist cmd;
        if (elapsed < 1.2) {
            sendGripperCommand(gripper_extend_angle_);
            sendLifterCommand(lifter_height_);
            cmd.linear.x = 0.5;
            cmd.angular.z = 0.0;
        } else if (elapsed < 3.0) {
            sendGripperCommand(gripper_retract_angle_);
            cmd.linear.x = -0.8;
            cmd.angular.z = escape_direction_ * 1.2;
        } else if (elapsed < 5.0) {
            sendGripperCommand(gripper_sense_angle_);
            cmd.linear.x = -0.5;
            cmd.angular.z = escape_direction_ * 2.0;
        } else {
            RCLCPP_INFO(this->get_logger(), "Aggressive push completed");
            escape_state_ = EscapeState::NORMAL;
            sendLifterCommand(0.0);
            sendGripperCommand(0.0);
            return true;
        }

        cmd_pub_->publish(cmd);
        return false;
    }

    bool executeCornerEscape(double elapsed)
    {
        geometry_msgs::msg::Twist cmd;
        if (elapsed < 1.5) {
            sendGripperCommand(gripper_extend_angle_);
            sendLifterCommand(lifter_height_);
            cmd.linear.x = -0.5;
            cmd.angular.z = 0.0;
        } else if (elapsed < 5.0) {
            sendGripperCommand(gripper_sense_angle_);
            cmd.linear.x = -0.3;
            cmd.angular.z = escape_direction_ * 2.5;
        } else if (elapsed < 8.0) {
            sendGripperCommand(gripper_retract_angle_);
            cmd.linear.x = 0.3;
            cmd.angular.z = escape_direction_ * 1.0;
            float front_distance = getFrontDistance();
            if (front_distance < 0.1) {
                cmd.linear.x = -0.3;
                cmd.angular.z *= 1.8;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Corner escape completed");
            escape_state_ = EscapeState::NORMAL;
            sendLifterCommand(0.0);
            sendGripperCommand(0.0);
            return true;
        }

        cmd_pub_->publish(cmd);
        return false;
    }

    bool executeWiggleEscape(double elapsed)
    {
        geometry_msgs::msg::Twist cmd;
        double cycle_time = std::fmod(elapsed, 1.0);
        if (elapsed < 10.0) {
            sendLifterCommand(cycle_time < 0.5 ? lifter_height_ : 0.0); // Toggle lifter
            if (cycle_time < 0.5) {
                sendGripperCommand(gripper_extend_angle_);
            } else {
                sendGripperCommand(gripper_retract_angle_);
            }
            cmd.linear.x = -0.5 * (1.0 + 0.5 * std::sin(elapsed * M_PI));
            cmd.angular.z = escape_direction_ * (2.0 + 0.5 * std::sin(elapsed * M_PI * 2.0));
            float front_distance = getFrontDistance();
            if (front_distance < 0.1) {
                cmd.linear.x *= -1.5;
                cmd.angular.z *= 1.5;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Wiggle escape completed");
            escape_state_ = EscapeState::NORMAL;
            sendLifterCommand(0.0);
            sendGripperCommand(0.0);
            return true;
        }

        cmd_pub_->publish(cmd);
        return false;
    }

    bool executeSpiralEscape(double elapsed)
    {
        geometry_msgs::msg::Twist cmd;
        double spiral_phase = elapsed * 0.8;
        if (elapsed < 12.0) {
            sendGripperCommand(gripper_sense_angle_);
            sendLifterCommand(lifter_height_);
            cmd.linear.x = 0.4;
            cmd.angular.z = 1.5 + 0.4 * std::sin(spiral_phase);
            float front_distance = getFrontDistance();
            if (front_distance < 0.1) {
                cmd.linear.x = -0.3;
                cmd.angular.z *= 1.8;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Spiral escape completed");
            escape_state_ = EscapeState::NORMAL;
            sendLifterCommand(0.0);
            sendGripperCommand(0.0);
            return true;
        }

        cmd_pub_->publish(cmd);
        return false;
    }

    float getWallDistance()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_scan_) return 1.0;

        float min_side_distance = latest_scan_->range_max;
        size_t num_rays = latest_scan_->ranges.size();
        float angle_min = latest_scan_->angle_min;
        float angle_inc = latest_scan_->angle_increment;

        for (size_t i = 0; i < num_rays; ++i) {
            float angle = angle_min + i * angle_inc;
            float r = latest_scan_->ranges[i];
            if (!std::isfinite(r)) continue;

            bool is_target_side = (escape_direction_ > 0 && angle > M_PI / 4 && angle < 3 * M_PI / 4) ||
                                 (escape_direction_ < 0 && angle < -M_PI / 4 && angle > -3 * M_PI / 4);

            if (is_target_side) {
                min_side_distance = std::min(min_side_distance, r);
            }
        }

        return min_side_distance;
    }

    float getFrontDistance()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_scan_) return 1.0;

        float min_front = latest_scan_->range_max;
        size_t num_rays = latest_scan_->ranges.size();
        float angle_min = latest_scan_->angle_min;
        float angle_inc = latest_scan_->angle_increment;

        for (size_t i = 0; i < num_rays; ++i) {
            float angle = angle_min + i * angle_inc;
            float r = latest_scan_->ranges[i];
            if (!std::isfinite(r)) continue;

            if (std::abs(angle) < M_PI / 6.0) {
                min_front = std::min(min_front, r);
            }
        }

        return min_front;
    }

    // void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    // {
    //     std::lock_guard<std::mutex> lock(state_mutex_);
    //     if (!have_pose_ || scan->ranges.empty()) {
    //         RCLCPP_WARN(this->get_logger(), "Invalid or empty laser scan or no pose available");
    //         return;
    //     }

    //     float robot_x = current_pose_.position.x;
    //     float robot_y = current_pose_.position.y;

    //     double siny_cosp = 2 * (current_pose_.orientation.w * current_pose_.orientation.z +
    //                             current_pose_.orientation.x * current_pose_.orientation.y);
    //     double cosy_cosp = 1 - 2 * (current_pose_.orientation.y * current_pose_.orientation.y +
    //                                current_pose_.orientation.z * current_pose_.orientation.z);
    //     float yaw = std::atan2(siny_cosp, cosy_cosp);

    //     std::vector<float> ranges = scan->ranges;
    //     std::vector<float> angles(ranges.size());
    //     float angle = scan->angle_min;
    //     for (size_t i = 0; i < ranges.size(); ++i) {
    //         angles[i] = angle;
    //         angle += scan->angle_increment;
    //     }

    //     grid_->integrateScan(ranges, angles, robot_x, robot_y, yaw, scan->range_max);

    //     std::vector<int8_t> occ_data;
    //     grid_->downloadToOcc(occ_data);
    //     map_msg_.data = occ_data;
    //     map_msg_.header.stamp = this->get_clock()->now();
    //     map_pub_->publish(map_msg_);

    //     latest_scan_ = scan;
    // }
void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!have_pose_ || scan->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid or empty laser scan or no pose available");
            return;
        }

        // Filter out box-like obstacles from the scan
        std::vector<float> filtered_ranges = scan->ranges;
        std::vector<float> angles(filtered_ranges.size());
        float angle = scan->angle_min;
        
        for (size_t i = 0; i < filtered_ranges.size(); ++i) {
            angles[i] = angle;
            float r = filtered_ranges[i];
            
            // Filter out small obstacles (likely boxes) that are close
            // Keep only obstacles that are:
            // 1. Very close (< 0.15m) - these are walls/permanent obstacles
            // 2. Far away (> 0.8m) - these are definitely walls
            // 3. Filter out medium distance (0.15-0.8m) small returns (likely boxes)
            
            if (std::isfinite(r) && r > 0.15 && r < 0.8) {
                // Check if this looks like a small object (box)
                // by examining neighboring rays
                bool is_small_object = true;
                int neighbors_to_check = 3;
                
                // Check if neighboring rays are much longer (indicating isolated object)
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
                    filtered_ranges[i] = scan->range_max; // Set to max range (free space)
                    RCLCPP_DEBUG(this->get_logger(), "Filtered potential box at angle %.2f, range %.2f", 
                                angle, r);
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

        // Store original scan for obstacle avoidance (if needed)
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

    // void moveRobot()
    // {
    //     std::lock_guard<std::mutex> lock(state_mutex_);
    //     if (!have_pose_) {
    //         return;
    //     }

    //     if (escape_state_ != EscapeState::NORMAL) {
    //         if (executeEscapeSequence()) {
    //             RCLCPP_INFO(this->get_logger(), "Resuming normal navigation");
    //             escape_attempts_ = 0;
    //             left_contact_ = false;
    //             right_contact_ = false;
    //             path_.clear();
    //             path_index_ = 0;
    //         }
    //         return;
    //     }

    //     geometry_msgs::msg::Twist cmd;
    //     if (left_contact_ || right_contact_) {
    //         initiateEscapeSequence();
    //         return;
    //     }

    //     if (latest_scan_ && !latest_scan_->ranges.empty()) {
    //         float min_front = latest_scan_->range_max;
    //         float min_left = latest_scan_->range_max;
    //         float min_right = latest_scan_->range_max;
    //         float very_close_front = latest_scan_->range_max;

    //         size_t num_rays = latest_scan_->ranges.size();
    //         float angle_min = latest_scan_->angle_min;
    //         float angle_inc = latest_scan_->angle_increment;

    //         for (size_t i = 0; i < num_rays; ++i) {
    //             float angle = angle_min + i * angle_inc;
    //             float r = latest_scan_->ranges[i];
    //             if (!std::isfinite(r) || r > latest_scan_->range_max) {
    //                 continue;
    //             }

    //             if (std::abs(angle) < M_PI / 9.0) {
    //                 very_close_front = std::min(very_close_front, r);
    //             }
    //             if (std::abs(angle) < M_PI / 6.0) {
    //                 min_front = std::min(min_front, r);
    //             }
    //             if (angle > M_PI / 6.0 && angle < M_PI / 2.0) {
    //                 min_left = std::min(min_left, r);
    //             }
    //             if (angle < -M_PI / 6.0 && angle > -M_PI / 2.0) {
    //                 min_right = std::min(min_right, r);
    //             }
    //         }

    //         if (very_close_front < 0.08) {
    //             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //                 "Emergency stop: obstacle at %.2fm", very_close_front);
    //             cmd.linear.x = -0.4;
    //             cmd.angular.z = (min_left > min_right) ? 1.5 : -1.5;
    //             sendGripperCommand(gripper_retract_angle_);
    //             cmd_pub_->publish(cmd);
    //             return;
    //         }

    //         if (min_front < 0.25) {
    //             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //                 "Proactive avoidance: front obstacle at %.2fm", min_front);
    //             cmd.linear.x = std::max(0.0, 0.3 * (min_front - 0.08) / 0.17);
    //             cmd.angular.z = (min_left > min_right) ? 1.2 : -1.2;
    //             path_.clear();
    //             path_index_ = 0;
    //             cmd_pub_->publish(cmd);
    //             return;
    //         }
    //     }

    //     if (path_.empty() || path_index_ >= static_cast<int>(path_.size())) {
    //         path_ = findPath();
    //         path_index_ = 0;
    //         if (path_.empty()) {
    //             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path found to goal");
    //             cmd_pub_->publish(cmd);
    //             return;
    //         }
    //     }

    //     std::pair<int, int> waypoint = path_[path_index_];
    //     int gx = waypoint.first;
    //     int gy = waypoint.second;
    //     double wx = map_msg_.info.origin.position.x + gx * map_msg_.info.resolution;
    //     double wy = map_msg_.info.origin.position.y + gy * map_msg_.info.resolution;

    //     double dx = wx - current_pose_.position.x;
    //     double dy = wy - current_pose_.position.y;
    //     double dist = std::sqrt(dx * dx + dy * dy);

    //     if (dist < 0.08) {
    //         path_index_++;
    //         if (path_index_ >= static_cast<int>(path_.size())) {
    //             RCLCPP_INFO(this->get_logger(), "Goal reached!");
    //             cmd_pub_->publish(cmd);
    //             return;
    //         }
    //         waypoint = path_[path_index_];
    //         gx = waypoint.first;
    //         gy = waypoint.second;
    //         wx = map_msg_.info.origin.position.x + gx * map_msg_.info.resolution;
    //         wy = map_msg_.info.origin.position.y + gy * map_msg_.info.resolution;
    //         dx = wx - current_pose_.position.x;
    //         dy = wy - current_pose_.position.y;
    //     }

    //     double desired_yaw = std::atan2(dy, dx);
    //     double siny_cosp = 2 * (current_pose_.orientation.w * current_pose_.orientation.z +
    //                             current_pose_.orientation.x * current_pose_.orientation.y);
    //     double cosy_cosp = 1 - 2 * (current_pose_.orientation.y * current_pose_.orientation.y +
    //                                current_pose_.orientation.z * current_pose_.orientation.z);
    //     double current_yaw = std::atan2(siny_cosp, cosy_cosp);
    //     double yaw_error = desired_yaw - current_yaw;
    //     yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

    //     double base_speed = 0.8;
    //     double speed_factor = std::min(1.0, dist / 0.5);
    //     cmd.linear.x = base_speed * speed_factor;
    //     cmd.angular.z = 2.5 * yaw_error;

    //     if (std::abs(yaw_error) > M_PI / 4.0) {
    //         cmd.linear.x = 0.0;
    //     }

    //     cmd_pub_->publish(cmd);
    // }
void moveRobot()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!have_pose_) {
            return;
        }

        if (escape_state_ != EscapeState::NORMAL) {
            if (executeEscapeSequence()) {
                RCLCPP_INFO(this->get_logger(), "Resuming normal navigation");
                escape_attempts_ = 0;
                left_contact_ = false;
                right_contact_ = false;
                path_.clear();
                path_index_ = 0;
            }
            return;
        }

        geometry_msgs::msg::Twist cmd;
        if (left_contact_ || right_contact_) {
            initiateEscapeSequence();
            return;
        }

        if (latest_scan_ && !latest_scan_->ranges.empty()) {
            float min_wall_front = latest_scan_->range_max;
            float min_wall_left = latest_scan_->range_max;
            float min_wall_right = latest_scan_->range_max;
            float very_close_wall = latest_scan_->range_max;

            size_t num_rays = latest_scan_->ranges.size();
            float angle_min = latest_scan_->angle_min;
            float angle_inc = latest_scan_->angle_increment;

            for (size_t i = 0; i < num_rays; ++i) {
                float angle = angle_min + i * angle_inc;
                float r = latest_scan_->ranges[i];
                if (!std::isfinite(r) || r > latest_scan_->range_max) {
                    continue;
                }

                // Only consider obstacles as "walls" if they are:
                // 1. Very close (< 0.12m) - immediate collision risk
                // 2. Part of a continuous structure (not isolated)
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
                    is_wall = (continuous_count >= 3); // Part of larger structure
                }

                if (is_wall) {
                    if (std::abs(angle) < M_PI / 9.0) {
                        very_close_wall = std::min(very_close_wall, r);
                    }
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

            // Only stop for walls, not boxes
            if (very_close_wall < 0.08) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Emergency stop: wall at %.2fm", very_close_wall);
                cmd.linear.x = -0.4;
                cmd.angular.z = (min_wall_left > min_wall_right) ? 1.5 : -1.5;
                sendGripperCommand(gripper_retract_angle_);
                cmd_pub_->publish(cmd);
                return;
            }

            // Gentle avoidance for walls, but push through boxes
            if (min_wall_front < 0.2) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Wall avoidance: obstacle at %.2fm", min_wall_front);
                cmd.linear.x = std::max(0.3, 0.5 * (min_wall_front - 0.08) / 0.12); // Maintain forward momentum
                cmd.angular.z = (min_wall_left > min_wall_right) ? 0.8 : -0.8; // Gentler steering
                cmd_pub_->publish(cmd);
                return;
            }
        }

        // ... rest of navigation logic remains the same ...

        if (path_.empty() || path_index_ >= static_cast<int>(path_.size())) {
            path_ = findPath();
            path_index_ = 0;
            if (path_.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path found to goal");
                // If no path due to boxes, try moving forward anyway
                cmd.linear.x = 0.3;
                cmd.angular.z = 0.1; // Slight turn to explore
                cmd_pub_->publish(cmd);
                return;
            }
        }

        // Continue with normal path following...
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

        double base_speed = 0.6; // Good speed since we ignore boxes
        double speed_factor = std::min(1.0, dist / 0.5);
        cmd.linear.x = base_speed * speed_factor;
        cmd.angular.z = 2.5 * yaw_error;

        // Keep moving even during turns since boxes won't stop us
        if (std::abs(yaw_error) > M_PI / 4.0) {
            cmd.linear.x = 0.3; // Maintain forward motion
        }

        cmd_pub_->publish(cmd);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Navigation: ignoring boxes, only avoiding walls");
    }
    // Member variables
    std::shared_ptr<gpu_slam_bot::GpuGrid> grid_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lifter_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr left_contact_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr right_contact_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex state_mutex_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    bool have_pose_;
    bool left_contact_;
    bool right_contact_;
    EscapeState escape_state_;
    rclcpp::Time escape_start_time_;
    rclcpp::Time last_progress_time_;
    geometry_msgs::msg::Pose escape_start_pose_;
    int escape_attempts_;
    int stall_count_;
    EscapeStrategy current_strategy_;
    std::vector<std::pair<int, int>> path_;
    int path_index_;
    bool is_corner_trapped_;
    float initial_contact_distance_;
    double escape_direction_;
    double goal_x_;
    double goal_y_;
    double max_escape_duration_;
    double gripper_retract_angle_;
    double gripper_extend_angle_;
    double gripper_sense_angle_;
    double lifter_height_;
    double last_left_gripper_angle_;
    double last_right_gripper_angle_;
    double last_lifter_height_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpuSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}