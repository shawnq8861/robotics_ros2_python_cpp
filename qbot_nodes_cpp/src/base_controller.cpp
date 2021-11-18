#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class BaseController : public rclcpp::Node
{
public:
    BaseController(double width, double diameter)
    : Node("base_controller")
    {
        wheel_base_ = width;
        wheel_diameter_ = diameter;
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&BaseController::cmd_vel_callback, this, _1));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard forward speed: [%f]", msg->linear.x);
        RCLCPP_INFO(this->get_logger(), "I heard angular speed: [%f]", msg->angular.z);
        drive_wheels();
    }
    void drive_wheels() {
        //
        // use kinematic model to compute each wheel rotational velocity
        // output to the RoboClaw
        //

    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    double wheel_base_;
    double wheel_diameter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    double width = 18.0;
    double diameter = 6.0;
    auto node = std::make_shared<BaseController>(width, diameter);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
