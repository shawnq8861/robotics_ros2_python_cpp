#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "roboclaw.hpp"
#include "qbot_nodes_cpp/msg/encoder_counts.hpp"
#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include "robot_configuration.hpp"

using std::placeholders::_1;

class BaseController : public rclcpp::Node
{
public:
    BaseController()
    : Node("base_controller"), port_("/dev/ttymxc2"), baudrate_(38400), 
        address_(0x80), v_linear_(0.0), v_angular_(0.0), duty_cycle_left_(0), 
        duty_cycle_right_(0), enc_m1_(0), enc_m2_(0)
    {
        enc_counts.enc1_cnt = 0;
        enc_counts.enc2_cnt = 0;
        period_ = timer_period;
        period_mag_ = period_.count();
        timer_ = this->create_wall_timer(
            period_, std::bind(&BaseController::timer_callback, this));
        robo_ = roboclaw_init(port_.c_str(), baudrate_);
        if (robo_ == nullptr) {
            RCLCPP_INFO_STREAM(this->get_logger(), "unable to instantiate roboclaw object...\n");
        }
        drive_wheels();
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&BaseController::cmd_vel_callback, this, _1));
        publisher_ = this->create_publisher<qbot_nodes_cpp::msg::EncoderCounts>("enc_counts", 10);
    }

    ~BaseController()
    {
        roboclaw_close(robo_);
    }

private:
    void timer_callback()
    {
        //
        // read encoders and publish counts      
        //
        // read encoders
        //
        int retry_count = 0;
        int response = ROBOCLAW_ERROR;
        if (roboclaw_encoders(robo_, address_, &enc_m1_, &enc_m2_) != ROBOCLAW_OK) {
            while (response != ROBOCLAW_OK && retry_count < max_retries) {
                ++retry_count;
                response = roboclaw_encoders(robo_, address_, &enc_m1_, &enc_m2_);
            }
        }
        //
        // publish counts
        // put in a timer callback instead
        //
        enc_counts.enc1_cnt = enc_m1_;
        enc_counts.enc2_cnt = enc_m2_;
        RCLCPP_INFO(this->get_logger(), "left encoder: '%d', right encoder: '%d'", enc_counts.enc1_cnt, enc_counts.enc2_cnt);
        publisher_->publish(enc_counts);
    }
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        v_linear_ = msg->linear.x;
        v_angular_ = msg->angular.z;
        RCLCPP_INFO(this->get_logger(), "I heard forward speed: [%f]", v_linear_);
        RCLCPP_INFO(this->get_logger(), "I heard angular speed: [%f]", v_angular_);
        //
        // need to read and publish encoder count
        //
        drive_wheels();
    }
    void drive_wheels() {
        int retry_count = 0;
        int response = ROBOCLAW_ERROR;
        //
        // use kinematic model to compute each wheel rotational velocity
        // output to the RoboClaw
        //
        // linear velocity units are inches/second
        // angular velocity units are radians/second
        // max revs/sec (rps) determines limits of linear and angular
        // max rps occurs at duty cycle = 100
        //
        // linear = pi * wheel diameter * rps
        // linear = (linear right + linear left) / 2.0
        // linear left = (2.0 * linear) - linear right
        // rps = linear / (pi * wheel diameter)
        // angular = (linear right - linear left) / (wheel base)
        // linear right  - linear left = angular * (wheel base)
        // linear right - ((2.0 * linear) - linear right) = angular * (wheel base)
        // 2.0 * (linear right) - 2.0 * linear = angular * (wheel base)
        // linear right = linear + ((angular * wheel base) / 2.0)
        // linear left = (2.0 * linear) - linear right
        // rps right = (linear right)/ (pi * wheel diameter)
        // rps left = (linear left)/ (pi * wheel diameter)
        // rpm right = 60 * rps right
        // rpm left = 60 * rps left
        // 
        // set the duty cycles
        //
        // duty right = rpm right / rpm max
        // duty left = rpm left / rpm max
        //
        double linear_right = v_linear_ + ((v_angular_ * wheel_base) / 2.0);
        RCLCPP_INFO_STREAM(this->get_logger(), "linear_right: " << linear_right);
        double linear_left = (2.0 * v_linear_) - linear_right;
        RCLCPP_INFO_STREAM(this->get_logger(), "linear_left: " << linear_left);
        double rpm_right = 60.0 * (linear_right / (pi * wheel_diameter));
        double rpm_left = 60.0 * (linear_left / (pi * wheel_diameter));
		//	
		// 32767 is max duty cycle setpoint that roboclaw accepts
        //
        duty_cycle_right_ = (int)(rpm_right * 100.0 / rpm_max);
        duty_cycle_right_ = (float)duty_cycle_right_/100.0f * 32767;
        duty_cycle_left_ = (int)(rpm_left * 100 / rpm_max);
        duty_cycle_left_ = (float)duty_cycle_left_/100.0f * 32767;
        //
        // move the motors
        //
        response = roboclaw_duty_m1m2(robo_, address_, duty_cycle_left_, duty_cycle_right_);
		if (response != ROBOCLAW_OK) {
            while (response != ROBOCLAW_OK && retry_count < max_retries) {
                ++retry_count;
                response = roboclaw_duty_m1m2(robo_, address_, duty_cycle_left_, duty_cycle_right_);
            }
		}
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<qbot_nodes_cpp::msg::EncoderCounts>::SharedPtr publisher_;
    std::string port_;
    int baudrate_;
    uint8_t address_;
    struct roboclaw *robo_;
    double v_linear_;
    double v_angular_;
    int duty_cycle_left_;
    int duty_cycle_right_;
    qbot_nodes_cpp::msg::EncoderCounts enc_counts;
    int32_t enc_m1_;
    int32_t enc_m2_;
    std::chrono::milliseconds period_;
    unsigned int period_mag_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //
    // instantiate the node
    //
    auto node = std::make_shared<BaseController>();
    //
    // use rt extensions
    //
    int rc = -1;
	struct sched_param my_params;
    //
	// Passing zero specifies caller’s (our) policy
    //
	my_params.sched_priority = base_controller_priority;
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
