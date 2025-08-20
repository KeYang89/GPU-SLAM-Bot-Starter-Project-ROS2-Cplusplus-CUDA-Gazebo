#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class SmoothMover : public rclcpp::Node
{
public:
    SmoothMover() : Node("smooth_mover"), current_speed_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SmoothMover::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;

        double target_speed = 1.0;    // m/s (your desired max speed)
        double accel = 0.05;          // how much speed increases every cycle

        // Gradual acceleration
        if (current_speed_ < target_speed) {
            current_speed_ += accel;
        }

        msg.linear.x = current_speed_;
        msg.angular.z = 0.0;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Speed: %.2f m/s", current_speed_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double current_speed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmoothMover>());
    rclcpp::shutdown();
    return 0;
}
