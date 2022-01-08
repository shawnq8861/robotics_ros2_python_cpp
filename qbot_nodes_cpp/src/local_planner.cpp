//
// to run from commad line:
// ros2 run qbot_nodes_cpp local_planner
//

#include <chrono>
#include <functional>
#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "qbot_nodes_cpp/srv/heading_speed.hpp"
#include "camera_interface_header.hpp"
#include "robot_configuration.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner()
    : Node("local_planner")
    {
        count_ = 0;
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        period_ = 500ms;
        period_mag_ = period_.count();
        timer_ = this->create_wall_timer(
            period_, std::bind(&LocalPlanner::timer_callback, this));
        curr_heading_ = 0.0;
        new_heading_ = curr_heading_;
        old_heading_ = curr_heading_;
        omega_ = 0.0;
        //rotation_step_ = max_v_angular;
        curr_speed_ = 0.0;
        new_speed_ = curr_speed_;
        old_speed_ = curr_speed_;
        speed_delta_ = max_delta_v;
        heading_delta_ = max_v_angular * (period_mag_ / 1000.0);
        cmd_vel_msg_.linear.x = curr_speed_;
        cmd_vel_msg_.linear.y = 0.0;
        cmd_vel_msg_.linear.z = 0.0;
        cmd_vel_msg_.angular.x = 0.0;
        cmd_vel_msg_.angular.y = 0.0;
        cmd_vel_msg_.angular.z = omega_;
        //
        // use lambda expression to handle service callback
        // to avoid using std::bind, which is messy for sevice callbacks
        //
        // command to test:  ros2 service call /set_heading_speed 
        // qbot_nodes_cpp/srv/HeadingSpeed "{heading: 3.24, speed: 12.9}"
        // where:  heading is in degrees, speed is in in/sec
        //
        auto handle_set_heading_speed =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<qbot_nodes_cpp::srv::HeadingSpeed::Request> request,
                            std::shared_ptr<qbot_nodes_cpp::srv::HeadingSpeed::Response> response) -> void
        {
            (void)request_header;
            new_heading_ = request->heading;
            //
            // convert to radians
            //
            new_heading_ = new_heading_ * pi / 180.0;
            new_speed_ = request->speed;
            if (new_speed_ > max_v_forward) {
                new_speed_ = max_v_forward;
            }
            RCLCPP_INFO(this->get_logger(), "Requested heading: '%f', speed: '%f'", new_heading_, new_speed_);
            response = 0;
        };
        service_ = create_service<qbot_nodes_cpp::srv::HeadingSpeed>("set_heading_speed", handle_set_heading_speed);
        //
        // subscribe to image published by camera node
        //
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_processed", 10, std::bind(&LocalPlanner::img_subscriber_callback, this, _1));
    }

private:
    void timer_callback()
    {
        if (new_speed_ < old_speed_ && curr_speed_ > new_speed_) {
            if ((curr_speed_ - new_speed_) < speed_delta_) {
                curr_speed_ = new_speed_;
            }
            else {
                curr_speed_ -= speed_delta_;
            }
        }
        else if (new_speed_ > old_speed_ && curr_speed_ < new_speed_) {
            if ((new_speed_ - curr_speed_) < speed_delta_) {
                curr_speed_ = new_speed_;
            }
            else {
                curr_speed_ += speed_delta_;
            }
        }
        else {
            curr_speed_ = new_speed_;
            old_speed_ = new_speed_;
        }
        if (new_heading_ < old_heading_ && curr_heading_ > new_heading_) {
            if ((curr_heading_ - new_heading_) < heading_delta_) {
                curr_heading_ = new_heading_;
                omega_ = -max_v_angular * ((curr_heading_ - new_heading_) / heading_delta_);
            }
            else {
                curr_heading_ -= heading_delta_;
                omega_ = -max_v_angular;
            }
        }
        else if (new_heading_ > old_heading_ && curr_heading_ < new_heading_) {
            if ((new_heading_ - curr_heading_) < heading_delta_) {
                curr_heading_ = new_heading_;
                omega_ = max_v_angular * ((new_heading_ - curr_heading_) / heading_delta_);
            }
            else {
                curr_heading_ += heading_delta_;
                omega_ = max_v_angular;
            }
        }
        else {
            curr_heading_ = new_heading_;
            old_heading_ = new_heading_;
            omega_ = 0.0;
        }
        RCLCPP_INFO(this->get_logger(), "Heading: '%f', speed: '%f'", curr_heading_, curr_speed_);
        cmd_vel_msg_.linear.x = curr_speed_;
        RCLCPP_INFO(this->get_logger(), "Publishing linear x: '%f'", cmd_vel_msg_.linear.x);
        cmd_vel_msg_.angular.z = omega_;
        RCLCPP_INFO(this->get_logger(), "Publishing angular z: '%f'", cmd_vel_msg_.angular.z);
        publisher_->publish(cmd_vel_msg_);
    }

    void img_subscriber_callback(const sensor_msgs::msg::Image::SharedPtr img_ptr)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "\n\nsubscriber count_: " << count_ << "\n");
        ++count_;
        int rows = img_ptr->height;
        int cols = img_ptr->width;
        int type = CV_8UC3;
        cv::Mat image(rows, cols, type, &img_ptr->data[0]);
        if (count_ == 5) {
            count_ = 0;
            compute_heading(image);
        }
    }

    void compute_heading(cv::Mat image)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "rows: " << image.rows);
        RCLCPP_INFO_STREAM(this->get_logger(), "cols: " << image.cols);
        std::string homedir = getenv("HOME");
        cv::imwrite(homedir + "/Pictures/saved_image.jpg", image);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist cmd_vel_msg_;
    rclcpp::Service<qbot_nodes_cpp::srv::HeadingSpeed>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    double curr_heading_;
    double new_heading_;
    double old_heading_;
    double omega_;
    double curr_speed_;
    double new_speed_;
    double old_speed_;
    double speed_delta_;
    double heading_delta_;
    std::chrono::milliseconds period_;
    unsigned int period_mag_;
    int count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>();
    //
    // use rt extensions
    //
    int rc = -1;
	struct sched_param my_params;
    //
	// Passing zero specifies caller’s (our) policy
    //
	my_params.sched_priority = local_planner_priority;
    //
	// Passing zero specifies callers (our) pid
    // Set policy to SCHED_RR, no preemption with time slicing
    //
	rc = sched_setscheduler(0, SCHED_RR, &my_params);
    if ( rc == -1 ) {
        std::cout << "could not change scheduler policy" << std::endl;
    }
    else {
        std::cout << "changed scheduler policy" << std::endl;
    }
    //
    // lock memory to prevent paging after instantiations are complete
    //
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rclcpp::spin(node);
    //
    // unlock memory before teardown
    //
    munlockall();
    rclcpp::shutdown();
    return 0;
}
