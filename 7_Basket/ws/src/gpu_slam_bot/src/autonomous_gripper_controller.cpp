#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <vector>
#include <future>

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
    this->declare_parameter("gripper_closed_angle", -0.4);
    this->declare_parameter("gripper_safe_angle", 0.2);  // Safe navigation angle
    this->declare_parameter("lifter_down", 0.0);
    this->declare_parameter("lifter_up", 0.4);
    this->declare_parameter("lifter_safe_height", 0.15); // Safe navigation height
    this->declare_parameter("obstacle_push_distance", 0.25);
    this->declare_parameter("box_name_prefix", "obstacle_");
    this->declare_parameter("detection_timeout", 3.0);

    // Get parameters
    min_cycle_time_ = this->get_parameter("min_cycle_time").as_double();
    max_cycle_time_ = this->get_parameter("max_cycle_time").as_double();
    gripper_open_angle_ = this->get_parameter("gripper_open_angle").as_double();
    gripper_closed_angle_ = this->get_parameter("gripper_closed_angle").as_double();
    gripper_safe_angle_ = this->get_parameter("gripper_safe_angle").as_double();
    lifter_down_ = this->get_parameter("lifter_down").as_double();
    lifter_up_ = this->get_parameter("lifter_up").as_double();
    lifter_safe_height_ = this->get_parameter("lifter_safe_height").as_double();
    obstacle_push_distance_ = this->get_parameter("obstacle_push_distance").as_double();
    box_name_prefix_ = this->get_parameter("box_name_prefix").as_string();
    detection_timeout_ = this->get_parameter("detection_timeout").as_double();

    // Publishers - Use consistent topic names and message types
    gripper_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "gripper_cmd", 10);
    lifter_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "lifter_cmd", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/robot_state", 10);

    // Subscribers
    box_detection_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/box_detections", 10, std::bind(&AutonomousGripperController::boxCallback, this, std::placeholders::_1));

    // Service client
    set_entity_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>(
        "/gazebo/set_entity_state");

    // Timer for control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AutonomousGripperController::controlLoop, this));

    // Initialize to safe navigation state
    last_detection_time_ = this->get_clock()->now();
    sendGripperCommand(gripper_safe_angle_);
    sendLifterCommand(lifter_safe_height_);
    publishState("IDLE");

    RCLCPP_INFO(this->get_logger(), "Autonomous Gripper Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Safe gripper angle: %.2f, Safe lifter height: %.2f", 
                gripper_safe_angle_, lifter_safe_height_);
  }

private:
  void sendGripperCommand(double angle)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->get_clock()->now();
    // Use correct joint names from URDF
    traj.joint_names = {"left_finger_joint", "right_finger_joint"};
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {angle, -angle};  // Mirror for left/right
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 800000000; // 0.8s for smoother motion
    traj.points = {point};
    
    gripper_cmd_pub_->publish(traj);
    RCLCPP_DEBUG(this->get_logger(), "Gripper command sent: %.2f", angle);
  }

  void sendLifterCommand(double height)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->get_clock()->now();
    // Use correct joint name from URDF
    traj.joint_names = {"gripper_lift_joint"};
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {height};
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 800000000; // 0.8s for smoother motion
    traj.points = {point};
    
    lifter_cmd_pub_->publish(traj);
    RCLCPP_DEBUG(this->get_logger(), "Lifter command sent: %.2f", height);
  }

  void publishState(const std::string& state)
  {
    std_msgs::msg::String state_msg;
    state_msg.data = state;
    state_pub_->publish(state_msg);
    
    // Log state changes
    static std::string last_state = "";
    if (state != last_state) {
      RCLCPP_INFO(this->get_logger(), "State changed: %s -> %s", last_state.c_str(), state.c_str());
      last_state = state;
    }
  }

  void boxCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    box_detected_ = false;
    box_distance_ = std::numeric_limits<double>::max();
    
    if (msg->width > 0 && !msg->data.empty()) {
      // Extract first point from point cloud
      float x, y, z;
      const uint8_t* data_ptr = msg->data.data();
      memcpy(&x, data_ptr, sizeof(float));
      memcpy(&y, data_ptr + 4, sizeof(float));
      memcpy(&z, data_ptr + 8, sizeof(float));
      
      double distance = std::sqrt(x * x + y * y);
      
      // Valid detection range for boxes
      if (distance >= 0.08 && distance <= 0.4) {
        box_detected_ = true;
        box_distance_ = distance;
        box_position_.x = x;
        box_position_.y = y;
        box_position_.z = z;
        last_detection_time_ = this->get_clock()->now();
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Box detected at (%.2f, %.2f), distance %.2f", x, y, distance);
      }
    }
    
    // Check for detection timeout
    auto now = this->get_clock()->now();
    if ((now - last_detection_time_).seconds() > detection_timeout_) {
      box_detected_ = false;
    }
  }

  void controlLoop()
  {
    auto now = this->get_clock()->now();
    
    // Check if we've lost box detection
    if ((now - last_detection_time_).seconds() > detection_timeout_) {
      if (box_detected_) {
        RCLCPP_INFO(this->get_logger(), "Box detection timed out");
      }
      box_detected_ = false;
    }

    if (box_detected_ && !box_picked_up_) {
      if (box_distance_ > obstacle_push_distance_) {
        // Approaching box - prepare gripper
        publishState("APPROACHING");
        sendLifterCommand(lifter_down_);  // Lower to box level
        sendGripperCommand(gripper_open_angle_);  // Open gripper wide
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Approaching box at distance %.2f", box_distance_);
      } else {
        // Close enough to pick up
        publishState("PICKING");
        sendLifterCommand(lifter_down_);  // Ensure at box level
        sendGripperCommand(gripper_closed_angle_);  // Close gripper
        
        // Wait for gripper to close
        if (pickup_timer_ == 0) {
          pickup_timer_ = 20;  // Wait 2 seconds (20 * 100ms)
          RCLCPP_INFO(this->get_logger(), "Starting pickup sequence...");
        } else if (--pickup_timer_ <= 0) {
          // Lift the box
          sendLifterCommand(lifter_up_);
          box_picked_up_ = true;
          pickup_complete_timer_ = 30;  // Hold for 3 seconds
          
          // Try to teleport box to basket
          teleportBoxToBasket();
          
          RCLCPP_INFO(this->get_logger(), "Box picked up and lifted!");
        }
      }
    } else if (box_picked_up_) {
      // Keep box lifted and gripper closed
      publishState("LIFTING");
      sendGripperCommand(gripper_closed_angle_);
      sendLifterCommand(lifter_up_);

      // Reset after timer expires
      if (--pickup_complete_timer_ <= 0) {
        publishState("IDLE");
        box_picked_up_ = false;
        box_detected_ = false;
        pickup_timer_ = 0;
        
        // Return to safe navigation position
        sendGripperCommand(gripper_safe_angle_);
        sendLifterCommand(lifter_safe_height_);
        
        RCLCPP_INFO(this->get_logger(), "Pickup sequence complete, returning to safe navigation mode");
      }
    } else {
      // Default: Safe navigation mode - keep gripper up and partially closed
      publishState("IDLE");
      sendGripperCommand(gripper_safe_angle_);  // Partially closed for safety
      sendLifterCommand(lifter_safe_height_);   // Raised for ground clearance
      pickup_timer_ = 0;  // Reset pickup timer
    }
  }

  void teleportBoxToBasket()
  {
    static int box_id = 1;
    std::string box_name = box_name_prefix_ + std::to_string(box_id++);
    
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = box_name;
    request->state.reference_frame = "world";
    
    // Position in basket (relative to robot's current position)
    request->state.pose.position.x = box_position_.x - 0.35;  // Behind robot in basket
    request->state.pose.position.y = box_position_.y;
    request->state.pose.position.z = 0.25;  // In basket height
    request->state.pose.orientation.w = 1.0;

    if (set_entity_state_client_->wait_for_service(std::chrono::milliseconds(100))) {
      auto future = set_entity_state_client_->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "Teleporting box %s to basket", box_name.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "SetEntityState service not available");
    }
  }

  // Parameters
  double min_cycle_time_;
  double max_cycle_time_;
  double gripper_open_angle_;
  double gripper_closed_angle_;
  double gripper_safe_angle_;
  double lifter_down_;
  double lifter_up_;
  double lifter_safe_height_;
  double obstacle_push_distance_;
  double detection_timeout_;
  std::string box_name_prefix_;

  // State
  bool box_detected_ = false;
  bool box_picked_up_ = false;
  double box_distance_ = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point box_position_;
  int pickup_timer_ = 0;
  int pickup_complete_timer_ = 0;
  rclcpp::Time last_detection_time_;

  // Publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lifter_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

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