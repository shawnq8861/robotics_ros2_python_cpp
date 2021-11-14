#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class BaseController : public rclcpp::Node
{
public:
    BaseController()
    : Node("base_controller")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&BaseController::cmd_vel_callback, this, _1));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: [%f]", msg->linear.x);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
