#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <vector>

class AutonomousGripperController : public rclcpp::Node
{
public:
  AutonomousGripperController() : Node("autonomous_gripper_controller")
  {
    // Declare parameters
    this->declare_parameter("use_sim_time", true);
    this->declare_parameter("min_cycle_time", 2.0);
    this->declare_parameter("max_cycle_time", 3.5);
    this->declare_parameter("gripper_open_angle", 0.5);
    this->declare_parameter("gripper_closed_angle", 0.0);
    this->declare_parameter("gripper_push_angle", -0.3);
    this->declare_parameter("lifter_down", 0.0);
    this->declare_parameter("lifter_up", 0.5);
    this->declare_parameter("lifter_push_height", 0.15);
    this->declare_parameter("wall_detection_distance", 0.3);
    this->declare_parameter("obstacle_push_distance", 0.2);

    // Get parameters
    min_cycle_time_ = this->get_parameter("min_cycle_time").as_double();
    max_cycle_time_ = this->get_parameter("max_cycle_time").as_double();
    gripper_open_angle_ = this->get_parameter("gripper_open_angle").as_double();
    gripper_closed_angle_ = this->get_parameter("gripper_closed_angle").as_double();
    gripper_push_angle_ = this->get_parameter("gripper_push_angle").as_double();
    lifter_down_ = this->get_parameter("lifter_down").as_double();
    lifter_up_ = this->get_parameter("lifter_up").as_double();
    lifter_push_height_ = this->get_parameter("lifter_push_height").as_double();
    wall_detection_distance_ = this->get_parameter("wall_detection_distance").as_double();
    obstacle_push_distance_ = this->get_parameter("obstacle_push_distance").as_double();

    // Publishers
    gripper_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/gripper_controller/commands", 10);
    lifter_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/lifter_controller/commands", 10);

    // Subscribers
    box_detection_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/box_detections", 10, std::bind(&AutonomousGripperController::boxCallback, this, std::placeholders::_1));

    // Service client
    set_entity_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>(
        "/set_entity_state");

    // Timer for control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AutonomousGripperController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Autonomous Gripper Controller initialized");
  }

private:
  void boxCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Simplified: Assume closest point in cloud is the target box
    // In practice, parse point cloud to find box centroid within detection range
    box_detected_ = false;
    box_distance_ = std::numeric_limits<double>::max();
    box_name_ = "";

    // Check if box is within detection range (front of robot)
    if (!msg->data.empty()) {
      // Assuming point cloud is in robot frame, check x-coordinate
      float x, y, z;
      // Simplified: Take first point for demo (replace with clustering logic)
      memcpy(&x, &msg->data[0], sizeof(float));
      memcpy(&y, &msg->data[4], sizeof(float));
      memcpy(&z, &msg->data[8], sizeof(float));

      double distance = std::sqrt(x*x + y*y + z*z);
      if (distance >= min_detection_range_ && distance <= max_detection_range_) {
        box_detected_ = true;
        box_distance_ = distance;
        box_position_.x = x;
        box_position_.y = y;
        box_position_.z = z;
        // Generate a dummy box name (replace with actual name from detection)
        box_name_ = "detected_box";
      }
    }
  }

  void controlLoop()
  {
    std_msgs::msg::Float64MultiArray gripper_cmd;
    std_msgs::msg::Float64MultiArray lifter_cmd;

    if (box_detected_ && !box_picked_up_) {
      // Open grippers when approaching box
      gripper_cmd.data = {gripper_open_angle_, -gripper_open_angle_}; // Left, Right
      lifter_cmd.data = {lifter_push_height_}; // Lift to box height

      if (box_distance_ < obstacle_push_distance_) {
        // Close grippers to grasp
        gripper_cmd.data = {gripper_closed_angle_, -gripper_closed_angle_};
        lifter_cmd.data = {lifter_up_}; // Lift box
        box_picked_up_ = true;

        // Teleport box to basket
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = box_name_;
        request->state.reference_frame = "gpu_slam_bot";
        request->state.pose.position.x = -0.15; // Basket position
        request->state.pose.position.y = 0.0;
        request->state.pose.position.z = 0.2;
        request->state.pose.orientation.w = 1.0;

        auto future = set_entity_state_client_->async_send_request(request);
        // Note: In production, handle future result
      }
    } else if (box_picked_up_) {
      // Keep grippers closed and lifter up until reset
      gripper_cmd.data = {gripper_closed_angle_, -gripper_closed_angle_};
      lifter_cmd.data = {lifter_up_};

      // Reset after a delay (e.g., 3 seconds)
      static auto last_pickup_time = this->now();
      if ((this->now() - last_pickup_time).seconds() > 3.0) {
        box_picked_up_ = false;
        box_detected_ = false;
        gripper_cmd.data = {gripper_open_angle_, -gripper_open_angle_};
        lifter_cmd.data = {lifter_down_};
      }
    } else {
      // Default: Open grippers, lower lifter
      gripper_cmd.data = {gripper_open_angle_, -gripper_open_angle_};
      lifter_cmd.data = {lifter_down_};
    }

    gripper_cmd_pub_->publish(gripper_cmd);
    lifter_cmd_pub_->publish(lifter_cmd);
  }

  // Parameters
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
  double min_detection_range_ = 0.05; // From launch file
  double max_detection_range_ = 0.3;

  // State
  bool box_detected_ = false;
  bool box_picked_up_ = false;
  double box_distance_ = std::numeric_limits<double>::max();
  std::string box_name_;
  geometry_msgs::msg::Point box_position_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lifter_cmd_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr box_detection_sub_;

  // Service client
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomousGripperController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}