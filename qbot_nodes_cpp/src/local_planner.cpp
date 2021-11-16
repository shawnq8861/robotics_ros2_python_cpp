#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "qbot_nodes_cpp/srv/heading_speed.hpp"

using namespace std::chrono_literals;

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner()
    : Node("local_planner")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        period_ = 500ms;
        period_mag_ = period_.count();
        timer_ = this->create_wall_timer(
            period_, std::bind(&LocalPlanner::timer_callback, this));
        heading_ = 0.0;
        new_heading_ = heading_;
        omega_ = 0.0;
        rotation_step_ = 0.50;
        speed_ = 0.0;
        new_speed_ = speed_;
        speed_delta_ = 0.25;
        cmd_vel_msg.linear.x = speed_;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.linear.z = 0.0;
        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = omega_;
        //
        // use lambda expression to handle service callback
        // to avoid using std::bind, which is messy for sevice callbacks
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
            new_heading_ = request->heading;
            new_speed_ = request->speed;
            RCLCPP_INFO(this->get_logger(), "Requested heading: '%f', speed: '%f'", new_heading_, new_speed_);
            response = 0;
        };
        service_ = create_service<qbot_nodes_cpp::srv::HeadingSpeed>("set_heading_speed", handle_set_heading_speed);
    }

private:
    void timer_callback()
    {
        //
        // update heading and speed, then publish
        //
        if (abs(new_speed_ - speed_) > speed_delta_) {
            if (speed_ > new_speed_) {
                speed_ -= speed_delta_;
            }
            else {
                speed_ += speed_delta_;
            }
        }
        else {
            speed_ = new_speed_;
        }
        if (abs(new_heading_ - heading_) > rotation_step_) {
            if (heading_ > new_heading_) {
                heading_ -= rotation_step_;
            }
            else {
                heading_ += rotation_step_;
            }
            omega_ = rotation_step_ / ((double)period_mag_ / 1000.0);
        }
        else {
            heading_ = new_heading_;
            omega_ = 0.0;
        }
        RCLCPP_INFO(this->get_logger(), "Heading: '%f', speed: '%f'", heading_, speed_);
        cmd_vel_msg.linear.x = speed_;
        RCLCPP_INFO(this->get_logger(), "Publishing linear x: '%f'", cmd_vel_msg.linear.x);
        cmd_vel_msg.angular.z = omega_;
        RCLCPP_INFO(this->get_logger(), "Publishing angular z: '%f'", cmd_vel_msg.angular.z);
        publisher_->publish(cmd_vel_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist cmd_vel_msg;
    rclcpp::Service<qbot_nodes_cpp::srv::HeadingSpeed>::SharedPtr service_;
    double heading_;
    double new_heading_;
    double omega_;
    double rotation_step_;
    double speed_;
    double new_speed_;
    double speed_delta_;
    std::chrono::milliseconds period_;
    unsigned int period_mag_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
