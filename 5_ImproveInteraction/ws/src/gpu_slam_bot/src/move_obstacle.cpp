#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <random>

using namespace std::chrono_literals;

class MoveObstacleNode : public rclcpp::Node
{
public:
    MoveObstacleNode()
    : Node("move_obstacle")
    {
        this->declare_parameter<std::string>("entity_name", "obstacle_1");
        entity_name_ = this->get_parameter("entity_name").as_string();

        client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

        // Timer starts after 10 seconds
        timer_ = this->create_wall_timer(
            10s,
            std::bind(&MoveObstacleNode::start_moving, this)
        );

        RCLCPP_INFO(this->get_logger(), "MoveObstacleNode ready for [%s], will move after 10s", entity_name_.c_str());
    }

private:
    void start_moving()
    {
        if (!client_->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "/gazebo/set_entity_state not available yet");
            return;
        }

        // Stop one-shot timer, switch to repeated motion
        timer_->cancel();

        move_timer_ = this->create_wall_timer(
            1s,
            std::bind(&MoveObstacleNode::move_once, this)
        );
    }

    void move_once()
    {
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        gazebo_msgs::msg::EntityState state;

        state.name = entity_name_;
        state.pose.position.x = random_range(-2.0, 2.0);
        state.pose.position.y = random_range(-2.0, 2.0);
        state.pose.position.z = 0.1;
        state.pose.orientation.w = 1.0;

        request->state = state;

        auto future = client_->async_send_request(request);
        // No blocking wait — avoid deadlocks

        RCLCPP_INFO(this->get_logger(), "Moved [%s] to (%.2f, %.2f)",
                    entity_name_.c_str(), state.pose.position.x, state.pose.position.y);
    }

    double random_range(double min, double max)
    {
        std::uniform_real_distribution<double> dist(min, max);
        return dist(rng_);
    }

    std::string entity_name_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr move_timer_;
    std::mt19937 rng_{std::random_device{}()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveObstacleNode>());
    rclcpp::shutdown();
    return 0;
}
