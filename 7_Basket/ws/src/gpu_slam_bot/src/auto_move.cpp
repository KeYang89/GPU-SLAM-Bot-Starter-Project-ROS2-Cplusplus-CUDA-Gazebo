#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

class SmoothMover : public rclcpp::Node
{
public:
    SmoothMover() : Node("smooth_mover"), current_speed_(0.0), is_moving_allowed_(true)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_state", 10, std::bind(&SmoothMover::state_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SmoothMover::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "SmoothMover initialized - will coordinate with gripper controller");
    }

private:
    void state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string state = msg->data;
        
        // Allow movement during idle and general moving states
        // Stop during all manipulation operations for precise control
        if (state == "IDLE" || state == "MOVING") {
            is_moving_allowed_ = true;
        } else {
            // Stop during: APPROACHING, PICKING, LIFTING
            is_moving_allowed_ = false;
        }
        
        // Log state changes for debugging
        static std::string last_state = "";
        if (state != last_state) {
            RCLCPP_INFO(this->get_logger(), "Robot state: %s -> Movement: %s", 
                       state.c_str(), is_moving_allowed_ ? "ALLOWED" : "PAUSED");
            last_state = state;
        }
    }

    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;

        if (is_moving_allowed_) {
            double target_speed = 0.3;    // Moderate speed for better control
            double accel = 0.008;         // Gentle acceleration

            if (current_speed_ < target_speed) {
                current_speed_ += accel;
                if (current_speed_ > target_speed) current_speed_ = target_speed;
            }
        } else {
            // Smooth stop for manipulation operations
            double decel = 0.02;
            if (current_speed_ > 0.0) {
                current_speed_ -= decel;
                if (current_speed_ < 0.0) current_speed_ = 0.0;
            }
        }

        msg.linear.x = current_speed_;
        msg.angular.z = 0.0;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double current_speed_;
    bool is_moving_allowed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmoothMover>());
    rclcpp::shutdown();
    return 0;
}