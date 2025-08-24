#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"

using namespace std::chrono_literals;

class AutonomousGripperController : public rclcpp::Node
{
public:
    AutonomousGripperController()
        : Node("autonomous_gripper_controller"),
          box_picked_up_(false)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        box_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/box_detected", 10,
            std::bind(&AutonomousGripperController::box_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            100ms, std::bind(&AutonomousGripperController::control_loop, this));

        set_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>(
            "/set_entity_state");
    }

private:
    void box_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        box_detected_ = msg->data;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd_vel;

        if (box_detected_ && !box_picked_up_)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_pub_->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), "Box detected! Moving box...");

            if (set_state_client_->wait_for_service(1s))
            {
                auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

                // Move the box above the robot
                gazebo_msgs::msg::EntityState state_msg;
                state_msg.name = "box_1";       // pick the first box (can iterate nearest box)
                state_msg.reference_frame = "gpu_slam_bot"; // relative to robot
                state_msg.pose.position.x = 0.0;
                state_msg.pose.position.y = 0.0;
                state_msg.pose.position.z = 0.3; // lifted above gripper
                state_msg.pose.orientation.w = 1.0;

                request->state = state_msg;

                auto result = set_state_client_->async_send_request(request);
            }

            box_picked_up_ = true;
            RCLCPP_INFO(this->get_logger(), "Box moved above robot!");
        }
        else
        {
            cmd_vel.linear.x = 0.2;
            cmd_pub_->publish(cmd_vel);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr box_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_client_;

    bool box_detected_{false};
    bool box_picked_up_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousGripperController>());
    rclcpp::shutdown();
    return 0;
}
