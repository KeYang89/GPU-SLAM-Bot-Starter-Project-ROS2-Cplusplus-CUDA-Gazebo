#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp> // For robot state messages

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
    }

private:
    void state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string state = msg->data;
        
        // Allow movement during exploration and searching
        // Stop movement during precise manipulation operations
        is_moving_allowed_ = (state == "IDLE" || state == "MOVING");
        
        // Gripper controller will handle movement during APPROACHING state
        // Stop auto_move during all manipulation states
        if (state == "APPROACHING" || state == "POSITIONING" || state == "PICKING" || 
            state == "LIFTING" || state == "ROTATING_UP" || state == "PLACING" ||  // NEW
            state == "RELEASING" || state == "RETRACTING" || state == "ESCAPE") {
            is_moving_allowed_ = false;
        }
                
        if (!is_moving_allowed_ && current_speed_ > 0.0) {
            current_speed_ = 0.0; // Stop smoothly if movement is disallowed
        }
        
        // Log state changes for debugging
        static std::string last_state = "";
        if (state != last_state) {
            RCLCPP_INFO(this->get_logger(), "Robot state changed: %s -> Movement allowed: %s", 
                       state.c_str(), is_moving_allowed_ ? "YES" : "NO");
            last_state = state;
        }
    }

    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;

        if (is_moving_allowed_) {
            double target_speed = 0.6;    // m/s (exploration speed)
            double accel = 0.02;          // how much speed increases every cycle

            // Gradual acceleration
            if (current_speed_ < target_speed) {
                current_speed_ += accel;
                if (current_speed_ > target_speed) current_speed_ = target_speed;
            }
        } else {
            // Gradual deceleration for smooth stops
            if (current_speed_ > 0.0) {
                current_speed_ -= 0.02;
                if (current_speed_ < 0.0) current_speed_ = 0.0;
            }
        }

        msg.linear.x = current_speed_;
        msg.angular.z = 0.0;

        publisher_->publish(msg);
        
        // Less frequent logging to reduce spam
        static int log_counter = 0;
        if (++log_counter % 20 == 0) {  // Log every 2 seconds (20 * 100ms)
            RCLCPP_INFO(this->get_logger(), "Speed: %.2f m/s, Allowed: %s", 
                       current_speed_, is_moving_allowed_ ? "YES" : "NO");
        }
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