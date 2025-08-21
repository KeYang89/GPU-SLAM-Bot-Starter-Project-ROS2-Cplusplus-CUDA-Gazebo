#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <random>
#include <chrono>
#include <mutex>
#include <atomic>
#include <cmath>
#include <signal.h>

enum class GripperMode {
    NORMAL_CYCLE,
    WALL_PUSH,
    OBSTACLE_AVOID,
    CONTACT_RESPONSE
};

enum class GripperState {
    OPENING,
    OPEN,
    CLOSING,
    CLOSED,
    PUSHING
};

class AutonomousGripperController : public rclcpp::Node
{
public:
    AutonomousGripperController()
    : Node("autonomous_gripper_controller"),
      current_mode_(GripperMode::NORMAL_CYCLE),
      current_state_(GripperState::OPEN),
      gripper_open_(true),
      left_contact_(false),
      right_contact_(false),
      robot_moving_(false),
      front_obstacle_distance_(10.0),
      current_left_gripper_(0.0),
      current_right_gripper_(0.0),
      current_lifter_(0.0),
      last_action_time_(this->get_clock()->now()),
      wall_push_start_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      obstacle_start_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      shutting_down_(false),
      rng_(std::chrono::steady_clock::now().time_since_epoch().count())
    {
        using std::placeholders::_1;

        // Parameters
        this->declare_parameter("min_cycle_time", 2.0);
        this->declare_parameter("max_cycle_time", 3.5);
        this->declare_parameter("gripper_open_angle", 0.4);
        this->declare_parameter("gripper_closed_angle", 0.0);
        this->declare_parameter("gripper_push_angle", -0.3);
        this->declare_parameter("lifter_down", 0.0);
        this->declare_parameter("lifter_up", 0.15);
        this->declare_parameter("lifter_push_height", 0.08);
        this->declare_parameter("wall_detection_distance", 0.3);
        this->declare_parameter("obstacle_push_distance", 0.2);

        min_cycle_time_ = std::max(0.1, this->get_parameter("min_cycle_time").as_double());
        max_cycle_time_ = std::max(min_cycle_time_, this->get_parameter("max_cycle_time").as_double());
        gripper_open_angle_ = this->get_parameter("gripper_open_angle").as_double();
        gripper_closed_angle_ = this->get_parameter("gripper_closed_angle").as_double();
        gripper_push_angle_ = this->get_parameter("gripper_push_angle").as_double();
        lifter_down_ = this->get_parameter("lifter_down").as_double();
        lifter_up_ = this->get_parameter("lifter_up").as_double();
        lifter_push_height_ = this->get_parameter("lifter_push_height").as_double();
        wall_detection_distance_ = std::max(0.05, this->get_parameter("wall_detection_distance").as_double());
        obstacle_push_distance_ = std::max(0.05, this->get_parameter("obstacle_push_distance").as_double());

        next_action_delay_ = getRandomDelay();

        // Publishers
        gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("gripper_cmd", 10);
        lifter_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("front_lifter_cmd", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscribers
        left_contact_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
            "left_gripper_contact", 10, std::bind(&AutonomousGripperController::onLeftContact, this, _1));
        right_contact_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
            "right_gripper_contact", 10, std::bind(&AutonomousGripperController::onRightContact, this, _1));
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&AutonomousGripperController::onJointState, this, _1));
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&AutonomousGripperController::onLaserScan, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&AutonomousGripperController::onOdometry, this, _1));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AutonomousGripperController::autonomousControlCallback, this));

        RCLCPP_INFO(this->get_logger(), "Autonomous Gripper Controller initialized");
        RCLCPP_INFO(this->get_logger(), "First gripper action in %.1f seconds", next_action_delay_);
    }

    ~AutonomousGripperController() { shutdown(); }

    void shutdown()
    {
        if (!shutting_down_.exchange(true)) {
            RCLCPP_INFO(this->get_logger(), "Shutting down autonomous gripper controller...");
            if (control_timer_) control_timer_->cancel();
            left_contact_sub_.reset();
            right_contact_sub_.reset();
            joint_state_sub_.reset();
            laser_sub_.reset();
            odom_sub_.reset();
            gripper_pub_.reset();
            lifter_pub_.reset();
            cmd_vel_pub_.reset();
            RCLCPP_INFO(this->get_logger(), "Shutdown complete");
        }
    }

private:
    double getRandomDelay()
    {
        std::uniform_real_distribution<double> dist(min_cycle_time_, max_cycle_time_);
        return dist(rng_);
    }

    void sendGripperCommand(double left_angle, double duration_sec = 0.5)
    {
        if (shutting_down_.load() || !gripper_pub_) return;

        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->get_clock()->now();
        traj.joint_names = {"left_gripper_joint", "right_gripper_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {left_angle, -left_angle};

        uint32_t sec_part = static_cast<uint32_t>(std::floor(duration_sec));
        uint32_t nsec_part = static_cast<uint32_t>((duration_sec - sec_part) * 1e9);
        point.time_from_start.sec = sec_part;
        point.time_from_start.nanosec = nsec_part;

        traj.points = {point};
        gripper_pub_->publish(traj);
    }

    void sendLifterCommand(double height, double duration_sec = 0.5)
    {
        if (shutting_down_.load() || !lifter_pub_) return;

        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->get_clock()->now();
        traj.joint_names = {"front_lifter_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {height};

        uint32_t sec_part = static_cast<uint32_t>(std::floor(duration_sec));
        uint32_t nsec_part = static_cast<uint32_t>((duration_sec - sec_part) * 1e9);
        point.time_from_start.sec = sec_part;
        point.time_from_start.nanosec = nsec_part;

        traj.points = {point};
        lifter_pub_->publish(traj);
    }

    void pushRobotBackward(double linear_speed = -0.2, double duration_sec = 0.5)
    {
        if (shutting_down_.load() || !cmd_vel_pub_) return;

        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear_speed;
        cmd_vel_pub_->publish(msg);
    }

    // --- Subscriber callbacks ---
    void onLeftContact(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        bool prev_contact = left_contact_;
        left_contact_ = !msg->states.empty();

        if (left_contact_ && !prev_contact) {
            current_mode_ = GripperMode::CONTACT_RESPONSE;
            contact_start_time_ = this->get_clock()->now();
        }
    }

    void onRightContact(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        bool prev_contact = right_contact_;
        right_contact_ = !msg->states.empty();

        if (right_contact_ && !prev_contact) {
            current_mode_ = GripperMode::CONTACT_RESPONSE;
            contact_start_time_ = this->get_clock()->now();
        }
    }

    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            if (msg->name[i] == "left_gripper_joint") current_left_gripper_ = msg->position[i];
            if (msg->name[i] == "right_gripper_joint") current_right_gripper_ = msg->position[i];
            if (msg->name[i] == "front_lifter_joint") current_lifter_ = msg->position[i];
        }
    }

    void onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        front_obstacle_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    }

    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        robot_moving_ = std::abs(msg->twist.twist.linear.x) > 0.01 || std::abs(msg->twist.twist.angular.z) > 0.01;
    }

    void executeContactResponse()
    {
        RCLCPP_INFO(this->get_logger(), "Executing contact response: pushing backward");
        pushRobotBackward();
        sendGripperCommand(gripper_open_angle_);
        sendLifterCommand(lifter_down_);
        current_mode_ = GripperMode::NORMAL_CYCLE;
        next_action_delay_ = getRandomDelay();
    }

    void autonomousControlCallback()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_mode_ == GripperMode::CONTACT_RESPONSE) {
            executeContactResponse();
            return;
        }

        // Normal cycle
        rclcpp::Time now = this->get_clock()->now();
        if ((now - last_action_time_).seconds() >= next_action_delay_) {
            if (current_state_ == GripperState::OPEN) {
                sendGripperCommand(gripper_closed_angle_);
                current_state_ = GripperState::CLOSING;
            } else {
                sendGripperCommand(gripper_open_angle_);
                current_state_ = GripperState::OPENING;
            }
            last_action_time_ = now;
            next_action_delay_ = getRandomDelay();
        }
    }

private:
    std::mutex state_mutex_;
    std::atomic<bool> shutting_down_;

    GripperMode current_mode_;
    GripperState current_state_;
    bool gripper_open_;
    bool left_contact_;
    bool right_contact_;
    bool robot_moving_;
    double front_obstacle_distance_;
    double current_left_gripper_;
    double current_right_gripper_;
    double current_lifter_;
    rclcpp::Time last_action_time_;
    rclcpp::Time wall_push_start_time_;
    rclcpp::Time obstacle_start_time_;
    rclcpp::Time contact_start_time_;
    std::mt19937 rng_;

    double next_action_delay_;
    double min_cycle_time_;
    double max_cycle_time_;
    double gripper_open_angle_;
    double gripper_closed_angle_;
    double gripper_push_angle_;
    double lifter_down_;
    double lifter_up_;
    double lifter_push_height_;
    double wall_detection_distance_;
    double obstacle_push_distance_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lifter_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr left_contact_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr right_contact_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
};

// ------------------------
// Main function
// ------------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousGripperController>();
    rclcpp::spin(node);
    node->shutdown();
    rclcpp::shutdown();
    return 0;
}
