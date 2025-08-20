#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class AutoMove : public rclcpp::Node
{
public:
    AutoMove() : Node("auto_move")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&AutoMove::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.2;   // forward speed
        msg.angular.z = 0.0;  // no rotation
        pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoMove>());
    rclcpp::shutdown();
    return 0;
}
