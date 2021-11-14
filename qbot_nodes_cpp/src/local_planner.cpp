#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LocalPlanner : public rclcpp::Node
{
  public:
    LocalPlanner()
    : Node("local_planner")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&LocalPlanner::timer_callback, this));
      cmd_vel_msg.linear.y = 0.0;
      cmd_vel_msg.linear.z = 0.0;
      cmd_vel_msg.angular.x = 0.0;
      cmd_vel_msg.angular.y = 0.0;
    }

  private:
    void timer_callback()
    {
      cmd_vel_msg.linear.x = 1.37;
      RCLCPP_INFO(this->get_logger(), "Publishing linear x: '%f'", cmd_vel_msg.linear.x);
      cmd_vel_msg.angular.z = .259;
      RCLCPP_INFO(this->get_logger(), "Publishing angular z: '%f'", cmd_vel_msg.angular.z);
      publisher_->publish(cmd_vel_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist cmd_vel_msg;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
