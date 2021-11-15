#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "qbot_nodes_cpp/srv/heading_speed.hpp"

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
        heading_ = 0.0;
        speed_ = 0.0;
        //
        // use lambda function to handle service callback
        //
        // command to test:  ros2 service call /set_heading_speed 
        // qbot_nodes_cpp/srv/HeadingSpeed "{heading: 3.24, speed: 12.9}"
        //
        auto handle_set_heading_speed =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<qbot_nodes_cpp::srv::HeadingSpeed::Request> request,
                            std::shared_ptr<qbot_nodes_cpp::srv::HeadingSpeed::Response> response) -> void
        {
            (void)request_header;
            heading_ = request->heading;
            speed_ = request->speed;
            RCLCPP_INFO(this->get_logger(), "Requested heading: '%f', speed: '%f'", heading_, speed_);
            response = 0;
        };
        service_ = create_service<qbot_nodes_cpp::srv::HeadingSpeed>("set_heading_speed", handle_set_heading_speed);
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
    rclcpp::Service<qbot_nodes_cpp::srv::HeadingSpeed>::SharedPtr service_;
    double heading_;
    double speed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
